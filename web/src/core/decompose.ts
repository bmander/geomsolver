/* Stage 3 — cluster merging (Fudos-Hoffmann, generalised) -> plan -> replay.
 *
 * Decomposition (once per topology):
 *   * every PP/PL edge seeds a 2-element rigid cluster; ground (fixed points + x-axis) is a
 *     fixed cluster; lines carry direction classes (union-find with angular offsets);
 *   * repeatedly merge a pair or a triple of clusters when what they share — points, lines,
 *     directions — determines their relative rigid transforms.  F-H's triangle rule is the
 *     common case; the decision is made generally by the rank of the small merge Jacobian at
 *     generic (witness) poses, with self-motions of degenerate clusters accounted for, so
 *     parallels, perpendiculars and H/V need no special cases;
 *   * when pair/triple merging stalls, look for a small core: a minimal subset of clusters
 *     that is rigid as a whole (Stage 3b), merge it as one numeric step, resume tree merging;
 *   * the merge sequence is the plan; the clusters left over are the roots.
 *
 * Execution (every solve / drag frame, no graph analysis):
 *   * leaf poses from the live dimension values, warm-started on the current geometry;
 *   * PPP triangle merges by ruler-and-compass with an explicit chirality flag; other merges
 *     by a small minimum-norm Newton (DogLeg if it does not converge);
 *   * unfixed roots placed by least-change (Procrustes onto current positions);
 *   * write back; verify with the compiled System; numeric fallback if needed.
 */
import { ConstraintGraph, Edge, El, X_AXIS, build, cmpEl, el, elIdx, elKind, lineNormal, normalOf, remainder, sortEls } from './cgraph.js';
import { dulmageMendelsohn } from './graph.js';
import { Mat, mat, minNormSolve, rankRrqr } from './linalg.js';
import { Arc, Circle, Point, Sketch } from './model.js';
import { Rng } from './rng.js';
import { Drag, Method, SolveResult, System, Triangle, increments, orientation } from './system.js';

/** point: (x, y); line: (nx, ny, c) */
export type Pose = Float64Array;
const X_POSE = Float64Array.of(0, 1, 0);

export type Pair = [number, number, El];
export type DPair = [number, number, El, El, number];

export class Cluster {
  constructor(public id: number, public els: Map<El, Pose>, public fixed: boolean) {}

  clone(): Cluster {
    return new Cluster(this.id, new Map(this.els), this.fixed);
  }
}

/** One merge, lowered for replay.  ids[0] is the reference cluster (identity transform);
 *  pairs/dpairs use positions into `ids`; a PPP triangle carries its (x, y, z) construction
 *  and `branch` (+-1 chirality: the orientation of (x, z, y)) — set by the replay from the
 *  sketch when null, so a persisted plan replays the recorded root. */
export class Step {
  constructor(
    readonly ids: number[],
    readonly pairs: Pair[],
    readonly dpairs: DPair[],
    readonly ppp: [El, El, El] | null = null,
    public branch: number | null = null,
  ) {}

  get out(): number {
    return this.ids[0];
  }

  /** Document-stable identity of a closed-form construction (null for numeric merges).
   *  Keyed by the sketch indices of the three points, not by compiled element indices, so it
   *  survives save/load and edits that renumber elements. */
  key(graph: ConstraintGraph): string | null {
    if (!this.ppp) return null;
    return 'ppp:' + this.ppp.map((e) => graph.pointIndex(e)).join('|');
  }
}

export class Plan {
  /** True: replay the recorded chirality even if the sketch moved (Stage 5). */
  stickyBranches = false;

  constructor(
    readonly graph: ConstraintGraph,
    readonly leaves: [number, number][],
    readonly groundId: number,
    readonly singletons: [number, El][],
    readonly steps: Step[],
    readonly roots: number[],
  ) {}

  get fullyDecomposed(): boolean {
    return !this.graph.unsupported.length && this.roots.length === 1;
  }

  /** Recorded root choices of the closed-form merges, keyed stably for persistence. */
  branches(): Map<string, number> {
    const out = new Map<string, number>();
    for (const st of this.steps) {
      const k = st.key(this.graph);
      if (k !== null && st.branch !== null) out.set(k, st.branch);
    }
    return out;
  }

  /** Install recorded root choices (e.g. from a document); returns how many matched. */
  applyBranches(branches: Map<string, number>): number {
    let n = 0;
    for (const st of this.steps) {
      const k = st.key(this.graph);
      if (k !== null && branches.has(k)) {
        st.branch = branches.get(k)! >= 0 ? 1 : -1;
        n++;
      }
    }
    return n;
  }

  /** (index, step) of every merge that places `e`: closed-form ones where `e` is the
   *  constructed apex, else numeric merges that share it. */
  stepsPlacing(e: El): [number, Step][] {
    const closed: [number, Step][] = [];
    this.steps.forEach((st, i) => { if (st.ppp && st.ppp[1] === e) closed.push([i, st]); });
    if (closed.length) return closed;
    const num: [number, Step][] = [];
    this.steps.forEach((st, i) => {
      if (!st.ppp && st.pairs.some(([, , x]) => x === e)) num.push([i, st]);
    });
    return num;
  }

  /** Flip the root of every closed-form merge that constructs `e`; returns how many. */
  flip(e: El): number {
    const sts = this.stepsPlacing(e).map(([, st]) => st).filter((st) => st.ppp !== null);
    for (const st of sts) st.branch = -(st.branch ?? 1);
    return sts.length;
  }

  summary(): string {
    return `${this.leaves.length} leaves, ${this.steps.length} merges -> ${this.roots.length} root(s); `
      + `${this.graph.unsupported.length} unsupported constraint(s)`;
  }
}

/* -- direction classes: weighted union-find, potential = angle relative to the root ------- */

class Dirs {
  private parent = new Map<El, El>();
  private pot = new Map<El, number>();

  add(e: El): void {
    if (!this.parent.has(e)) { this.parent.set(e, e); this.pot.set(e, 0); }
  }

  /** (root, angle of e relative to the root). */
  find(e: El): [El, number] {
    this.add(e);
    const path: El[] = [];
    while (this.parent.get(e) !== e) { path.push(e); e = this.parent.get(e)!; }
    const root = e;
    let acc = 0;
    for (let i = path.length - 1; i >= 0; i--) {   // path compression carrying the potentials
      const x = path[i];
      acc += this.pot.get(x)!;
      this.parent.set(x, root);
      this.pot.set(x, acc);
    }
    return [root, path.length ? this.pot.get(path[0])! : 0];
  }

  /** Impose n_b = rot(phi) n_a.  Returns false if it contradicts an existing relation. */
  join(a: El, b: El, phi: number): boolean {
    const [ra, pa] = this.find(a);
    const [rb, pb] = this.find(b);
    if (ra === rb) return Math.abs(remainder(pb - pa - phi, Math.PI)) < 1e-9;
    this.parent.set(rb, ra);
    this.pot.set(rb, pa + phi - pb);
    return true;
  }

  offset(a: El, b: El): number | null {
    const [ra, pa] = this.find(a);
    const [rb, pb] = this.find(b);
    return ra === rb ? pb - pa : null;
  }
}

/* -- rigid transforms ------------------------------------------------------------------- */

/** Apply the transform T = (cos, sin, tx, ty) to an element's pose. */
export function applyT(T: Float64Array, e: El, pose: Pose): Pose {
  const [c, s, tx, ty] = T;
  if (elKind(e) === 'P') return Float64Array.of(c * pose[0] - s * pose[1] + tx, s * pose[0] + c * pose[1] + ty);
  const nx2 = c * pose[0] - s * pose[1], ny2 = s * pose[0] + c * pose[1];
  return Float64Array.of(nx2, ny2, pose[2] + nx2 * tx + ny2 * ty);
}

export function makeT(theta: number, tx: number, ty: number): Float64Array {
  return Float64Array.of(Math.cos(theta), Math.sin(theta), tx, ty);
}

function poseOf(e: El, pose: Pose, th: number, tx: number, ty: number): Pose {
  const c = Math.cos(th), s = Math.sin(th);
  if (elKind(e) === 'P') return Float64Array.of(c * pose[0] - s * pose[1] + tx, s * pose[0] + c * pose[1] + ty);
  const n0 = c * pose[0] - s * pose[1], n1 = s * pose[0] + c * pose[1];
  return Float64Array.of(n0, n1, pose[2] + n0 * tx + n1 * ty);
}

/** Pose of `e` under (theta, t) and its Jacobian with respect to (theta, tx, ty). */
function poseJac(e: El, pose: Pose, th: number, tx: number, ty: number): [Pose, number[][]] {
  const c = Math.cos(th), s = Math.sin(th);
  if (elKind(e) === 'P') {
    const x = pose[0], y = pose[1];
    return [
      Float64Array.of(c * x - s * y + tx, s * x + c * y + ty),
      [[-s * x - c * y, 1, 0], [c * x - s * y, 0, 1]],
    ];
  }
  const nx = pose[0], ny = pose[1], cc = pose[2];
  const n0 = c * nx - s * ny, n1 = s * nx + c * ny;
  const d0 = -s * nx - c * ny, d1 = c * nx - s * ny;
  return [
    Float64Array.of(n0, n1, cc + n0 * tx + n1 * ty),
    [[d0, 0, 0], [d1, 0, 0], [d0 * tx + d1 * ty, n0, n1]],
  ];
}

/** Rigid transform (c, s, tx, ty) mapping points src onto dst in least squares. */
function procrustes(src: [number, number][], dst: [number, number][]): Float64Array {
  const n = src.length;
  if (n === 0) return Float64Array.of(1, 0, 0, 0);
  let msx = 0, msy = 0, mdx = 0, mdy = 0;
  for (let i = 0; i < n; i++) { msx += src[i][0]; msy += src[i][1]; mdx += dst[i][0]; mdy += dst[i][1]; }
  msx /= n; msy /= n; mdx /= n; mdy /= n;
  if (n === 1) return Float64Array.of(1, 0, mdx - msx, mdy - msy);
  let c = 0, s = 0;
  for (let i = 0; i < n; i++) {
    const ax = src[i][0] - msx, ay = src[i][1] - msy;
    const bx = dst[i][0] - mdx, by = dst[i][1] - mdy;
    c += ax * bx + ay * by;
    s += ax * by - ay * bx;
  }
  const L = Math.hypot(c, s) || 1;
  c /= L; s /= L;
  return Float64Array.of(c, s, mdx - (c * msx - s * msy), mdy - (s * msx + c * msy));
}

/** Rigid transform taking the segment p->q onto p2->q2 (exact when the lengths agree). */
function fit2(p: Pose, q: Pose, p2: Pose, q2: Pose): Float64Array {
  const ux = q[0] - p[0], uy = q[1] - p[1];
  const vx = q2[0] - p2[0], vy = q2[1] - p2[1];
  let c = ux * vx + uy * vy;
  let s = ux * vy - uy * vx;
  const L = Math.hypot(c, s) || 1;
  c /= L; s /= L;
  return Float64Array.of(c, s, p2[0] - (c * p[0] - s * p[1]), p2[1] - (s * p[0] + c * p[1]));
}

/* -- the merge system (shared by the generic-rank decision and by execution) -------------- */

interface MergeSystem {
  m: number;
  n: number;
  fun(u: Float64Array): Float64Array;
  jac(u: Float64Array): Mat;
}

/** Residual/Jacobian for the transforms of cl[1..] (cl[0] is the reference, identity). */
function mergeSystem(cl: Cluster[], pairs: Pair[], dpairs: DPair[], kMovable: number): MergeSystem {
  const size = (e: El): number => (elKind(e) === 'P' ? 2 : 3);
  let m = 0;
  for (const [, , e] of pairs) m += size(e);
  m += dpairs.length;
  const n = 3 * kMovable;

  const pose = (u: Float64Array, ci: number, e: El): Pose => {
    const p = cl[ci].els.get(e)!;
    return ci === 0 ? p : poseOf(e, p, u[3 * (ci - 1)], u[3 * (ci - 1) + 1], u[3 * (ci - 1) + 2]);
  };
  const dpose = (u: Float64Array, ci: number, e: El): [Pose, number[][]] => {
    const p = cl[ci].els.get(e)!;
    if (ci === 0) return [p, Array.from({ length: p.length }, () => [0, 0, 0])];
    return poseJac(e, p, u[3 * (ci - 1)], u[3 * (ci - 1) + 1], u[3 * (ci - 1) + 2]);
  };

  return {
    m, n,
    fun(u: Float64Array): Float64Array {
      const out = new Float64Array(m);
      let r = 0;
      for (const [i, j, e] of pairs) {
        const a = pose(u, i, e), b = pose(u, j, e);
        for (let t = 0; t < a.length; t++) out[r++] = a[t] - b[t];
      }
      for (const [i, j, la, lb, phi] of dpairs) {
        const na = pose(u, i, la), nb = pose(u, j, lb);
        const ang = Math.atan2(na[0] * nb[1] - na[1] * nb[0], na[0] * nb[0] + na[1] * nb[1]);
        out[r++] = remainder(ang - phi, 2 * Math.PI);
      }
      return out;
    },
    jac(u: Float64Array): Mat {
      const J = mat(m, n);
      let r = 0;
      for (const [i, j, e] of pairs) {
        const [pi, Ji] = dpose(u, i, e);
        const [, Jj] = dpose(u, j, e);
        for (let t = 0; t < pi.length; t++, r++) {
          if (i > 0) for (let q = 0; q < 3; q++) J.data[r * n + 3 * (i - 1) + q] += Ji[t][q];
          if (j > 0) for (let q = 0; q < 3; q++) J.data[r * n + 3 * (j - 1) + q] -= Jj[t][q];
        }
      }
      for (const [i, j] of dpairs) {          // d angle / d theta_b = 1, / d theta_a = -1
        if (j > 0) J.data[r * n + 3 * (j - 1)] += 1;
        if (i > 0) J.data[r * n + 3 * (i - 1)] -= 1;
        r++;
      }
      return J;
    },
  };
}

/** Plain minimum-norm Newton for the tiny merge systems (3k unknowns, warm-started at the
 *  identity).  No trust region: merges are near-linear from a warm start, and the caller
 *  falls back to the globalised solver if this does not converge. */
function newtonSmall(sys: MergeSystem, u0: Float64Array, tol = 1e-13, maxIter = 40): [Float64Array, number] {
  let u = new Float64Array(u0);
  let r = sys.fun(u);
  for (let it = 0; it < maxIter; it++) {
    if (!r.length || maxAbs(r) < tol) break;
    const neg = new Float64Array(r.length);
    for (let i = 0; i < r.length; i++) neg[i] = -r[i];
    const { x: p } = minNormSolve(sys.jac(u), neg);
    for (let i = 0; i < u.length; i++) u[i] += p[i];
    r = sys.fun(u);
    if (maxAbs(p) < 1e-15) break;
  }
  return [u, r.length ? maxAbs(r) : 0];
}

/** Globalised fallback: DogLeg on the same tiny system, in TypeScript because the residual
 *  is a cluster placement rather than a compiled sketch. */
function doglegSmall(sys: MergeSystem, u0: Float64Array, maxIter = 300): Float64Array {
  let u = new Float64Array(u0);
  let r = sys.fun(u);
  let delta = Infinity;
  const n = sys.n;
  for (let it = 0; it < maxIter; it++) {
    if (maxAbs(r) < 1e-13) break;
    const J = sys.jac(u);
    const f = 0.5 * dotf(r, r);
    const g = new Float64Array(n);
    for (let i = 0; i < sys.m; i++) for (let j = 0; j < n; j++) g[j] += J.data[i * n + j] * r[i];
    if (maxAbs(g) < 1e-18) break;
    const neg = new Float64Array(sys.m);
    for (let i = 0; i < sys.m; i++) neg[i] = -r[i];
    const { x: pGn } = minNormSolve(J, neg);
    let p: Float64Array;
    const gnNorm = Math.hypot(...pGn);
    if (gnNorm <= delta) {
      p = pGn;
    } else {
      const Jg = new Float64Array(sys.m);
      for (let i = 0; i < sys.m; i++) for (let j = 0; j < n; j++) Jg[i] += J.data[i * n + j] * g[j];
      const jg = dotf(Jg, Jg);
      const alpha = jg > 0 ? dotf(g, g) / jg : 0;
      const pSd = new Float64Array(n);
      for (let i = 0; i < n; i++) pSd[i] = -alpha * g[i];
      const sdNorm = Math.sqrt(dotf(pSd, pSd));
      if (sdNorm >= delta) {
        p = new Float64Array(n);
        for (let i = 0; i < n; i++) p[i] = (pSd[i] * delta) / (sdNorm || 1);
      } else {
        let aa = 0, bb = 0;
        for (let i = 0; i < n; i++) { const d = pGn[i] - pSd[i]; aa += d * d; bb += 2 * pSd[i] * d; }
        const cc = sdNorm * sdNorm - delta * delta;
        const disc = Math.max(0, bb * bb - 4 * aa * cc);
        const tau = aa > 0 ? (-bb + Math.sqrt(disc)) / (2 * aa) : 0;
        p = new Float64Array(n);
        for (let i = 0; i < n; i++) p[i] = pSd[i] + tau * (pGn[i] - pSd[i]);
      }
    }
    const pnorm = Math.sqrt(dotf(p, p));
    if (pnorm < 1e-14 * (1 + Math.sqrt(dotf(u, u)))) break;
    const uNew = new Float64Array(n);
    for (let i = 0; i < n; i++) uNew[i] = u[i] + p[i];
    const rNew = sys.fun(uNew);
    const fNew = 0.5 * dotf(rNew, rNew);
    let lin = 0;
    for (let i = 0; i < sys.m; i++) {
      let v = r[i];
      for (let j = 0; j < n; j++) v += J.data[i * n + j] * p[j];
      lin += v * v;
    }
    const pred = f - 0.5 * lin;
    const rho = pred > 0 ? (f - fNew) / pred : (fNew < f ? 1 : -1);
    if (rho > 0) {
      u = uNew; r = rNew;
      if (rho > 0.75) { if (isFinite(delta)) delta = Math.max(delta, 3 * pnorm); }
      else if (rho < 0.25) delta = 0.5 * pnorm;
    } else {
      delta = 0.25 * pnorm;
    }
    if (delta < 1e-15 * (1 + Math.sqrt(dotf(u, u)))) break;
  }
  return u;
}

const maxAbs = (v: ArrayLike<number>): number => {
  let m = 0;
  for (let i = 0; i < v.length; i++) { const a = Math.abs(v[i]); if (a > m) m = a; }
  return m;
};
const dotf = (a: ArrayLike<number>, b: ArrayLike<number>): number => {
  let s = 0;
  for (let i = 0; i < a.length; i++) s += a[i] * b[i];
  return s;
};

/** Dimension of the rigid motions that leave every element of the cluster in place: empty 3,
 *  a lone point 1, lines only and all parallel 1, otherwise 0.  (Poses are generic, so two
 *  points are distinct and non-parallel lines are transversal.) */
function selfMotion(c: Cluster): number {
  let nPts = 0;
  let firstN: Pose | null = null;
  for (const [e, pose] of c.els) {
    if (elKind(e) === 'P') {
      nPts++;
      if (nPts >= 2) return 0;
    } else {
      if (nPts) return 0;
      if (firstN === null) firstN = pose;
      else if (Math.abs(firstN[0] * pose[1] - firstN[1] * pose[0]) > 1e-9) return 0;
    }
  }
  if (nPts === 1) return firstN === null ? 1 : 0;
  return firstN === null ? 3 : 1;
}

/** Cluster ids with the reference (a fixed one if any, else the largest) first. */
function orderRefFirst(cl: Map<number, Cluster>, ids: number[]): number[] {
  let ref = ids.find((i) => cl.get(i)!.fixed);
  if (ref === undefined) {
    ref = ids[0];
    for (const i of ids) if (cl.get(i)!.els.size > cl.get(ref)!.els.size) ref = i;
  }
  return [ref, ...ids.filter((i) => i !== ref)];
}

/* -- decomposition (topology only) -------------------------------------------------------- */

export function decompose(graph: ConstraintGraph, seed = 0, coreMax = 12): Plan {
  const rng = new Rng(seed);
  const dirs = new Dirs();
  for (const d of graph.dirs) dirs.join(d.a, d.b, d.phi);
  const elements = graph.elements;
  // generic (witness) poses: random points; lines get a random normal per direction class
  // (plus their class offset) and a random offset — merge decisions are structural, so they
  // must not depend on the user's possibly-degenerate geometry
  const baseAngle = new Map<El, number>();
  const generic = new Map<El, Pose>();
  const droot = new Map<El, El>();
  for (const e of elements) {
    if (elKind(e) === 'P') {
      generic.set(e, Float64Array.of(rng.uniform(-100, 100), rng.uniform(-100, 100)));
    } else {
      const [root, pot] = dirs.find(e);
      droot.set(e, root);
      if (!baseAngle.has(root)) baseAngle.set(root, rng.uniform(0, 2 * Math.PI));
      const ang = baseAngle.get(root)! + pot;
      generic.set(e, Float64Array.of(Math.cos(ang), Math.sin(ang), rng.uniform(-100, 100)));
    }
  }

  const clusters = new Map<number, Cluster>();
  const of = new Map<El, Set<number>>();
  for (const e of elements) of.set(e, new Set());
  const dirOf = new Map<El, Set<number>>();
  const cdirs = new Map<number, Map<El, El>>();
  let nextId = 0;

  const relMemo = new Map<string, [El[], [El, El, number][]]>();
  const relKeys = new Map<number, Set<string>>();

  const register = (cid: number, els: El[]): void => {
    for (const e of els) {
      of.get(e)!.add(cid);
      if (elKind(e) !== 'P') {
        const r = droot.get(e)!;
        if (!dirOf.has(r)) dirOf.set(r, new Set());
        dirOf.get(r)!.add(cid);
        const cd = cdirs.get(cid)!;
        if (!cd.has(r)) cd.set(r, e);
      }
    }
  };

  const add = (els: El[], fx: boolean): number => {
    const cid = nextId++;
    const m = new Map<El, Pose>();
    for (const e of els) m.set(e, generic.get(e)!);
    clusters.set(cid, new Cluster(cid, m, fx));
    cdirs.set(cid, new Map());
    register(cid, sortEls(els));
    return cid;
  };

  const removeCluster = (cid: number): Cluster => {
    const c = clusters.get(cid)!;
    clusters.delete(cid);
    for (const e of c.els.keys()) of.get(e)!.delete(cid);
    for (const r of cdirs.get(cid)!.keys()) dirOf.get(r)?.delete(cid);
    cdirs.delete(cid);
    for (const key of relKeys.get(cid) ?? []) relMemo.delete(key);
    relKeys.delete(cid);
    return c;
  };

  const ground = add([X_AXIS, ...graph.groundPoints.map((i) => el('P', i))], true);
  const leaves: [number, number][] = graph.edges.map((e, i) => [add([e.a, e.b], false), i]);
  const singletons: [number, El][] = elements.filter((e) => of.get(e)!.size === 0).map((e) => [add([e], false), e]);
  const steps: Step[] = [];

  /** What two clusters share (memoised; entries die with either cluster). */
  const pairRel = (a: number, b: number): [El[], [El, El, number][]] => {
    const k0 = Math.min(a, b), k1 = Math.max(a, b);
    const key = `${k0},${k1}`;
    const hit = relMemo.get(key);
    if (hit) return hit;
    const A = clusters.get(k0)!, B = clusters.get(k1)!;
    const [small, big] = A.els.size <= B.els.size ? [A, B] : [B, A];
    const common = sortEls([...small.els.keys()].filter((e) => big.els.has(e)));
    const seen = new Set(common.filter((e) => elKind(e) !== 'P').map((e) => droot.get(e)!));
    const da = cdirs.get(k0)!, db = cdirs.get(k1)!;
    const [src, other] = da.size <= db.size ? [da, db] : [db, da];
    const drels: [El, El, number][] = [];
    for (const [root, la] of src) {
      if (seen.has(root) || !other.has(root)) continue;
      const inA = A.els.has(la);
      const la_ = inA ? la : da.get(root)!;
      const lb_ = inA ? db.get(root)! : la;
      const phi = dirs.offset(la_, lb_);
      if (phi === null) continue;
      drels.push([la_, lb_, phi]);
    }
    const res: [El[], [El, El, number][]] = [common, drels];
    relMemo.set(key, res);
    if (!relKeys.has(a)) relKeys.set(a, new Set());
    if (!relKeys.has(b)) relKeys.set(b, new Set());
    relKeys.get(a)!.add(key);
    relKeys.get(b)!.add(key);
    return res;
  };

  /** Shared rows between clusters `ids` (positions into ids); the direction relation is
   *  oriented from ids[i]'s line to ids[j]'s line. */
  const relations = (ids: number[]): [Pair[], DPair[]] => {
    const pairs: Pair[] = [];
    const dpairs: DPair[] = [];
    for (let i = 0; i < ids.length; i++) {
      for (let j = i + 1; j < ids.length; j++) {
        const [common, drels] = pairRel(ids[i], ids[j]);
        for (const e of common) pairs.push([i, j, e]);
        for (const [la, lb, phi] of drels) {
          if (clusters.get(ids[i])!.els.has(la)) dpairs.push([i, j, la, lb, phi]);
          else dpairs.push([i, j, lb, la, -phi]);
        }
      }
    }
    return [pairs, dpairs];
  };

  const selfm = new Map<number, number>();
  const selfMotionOf = (cid: number): number => {
    let v = selfm.get(cid);
    if (v === undefined) selfm.set(cid, (v = selfMotion(clusters.get(cid)!)));
    return v;
  };

  /** Relative rigid-transform DOF left after imposing everything the clusters share
   *  (0 <=> the merge is determined).  Generic rank of the merge Jacobian at witness poses. */
  const deficiency = (idsIn: number[]): number => {
    const ids = orderRefFirst(clusters, idsIn);
    const [pairs, dpairs] = relations(ids);
    const k = ids.length - 1;
    let need = 3 * k;
    for (let i = 1; i < ids.length; i++) need -= selfMotionOf(ids[i]);
    if (need <= 0) return 0;
    const bound = 2 * pairs.length + dpairs.length;
    if (bound < need) return need - bound;             // cheap upper bound on the rank
    const sys = mergeSystem(ids.map((i) => clusters.get(i)!), pairs, dpairs, k);
    const J = sys.jac(new Float64Array(3 * k));
    const rank = J.rows && J.cols ? rankRrqr(J, 1e-9) : 0;
    return Math.max(0, need - rank);
  };

  const determined = (ids: number[]): boolean => deficiency(ids) === 0;

  const merge = (idsIn: number[]): number => {
    const ids = orderRefFirst(clusters, idsIn);
    const [pairs, dpairs] = relations(ids);
    let ppp: [El, El, El] | null = null;
    if (ids.length === 3 && !dpairs.length && pairs.length === 3
        && pairs.every(([, , e]) => elKind(e) === 'P')
        && new Set(pairs.map(([i, j]) => `${i},${j}`)).size === 3) {
      const share = new Map(pairs.map(([i, j, e]) => [`${i},${j}`, e] as const));
      // x = ref&B, y = B&C, z = C&ref
      ppp = [share.get('0,1')!, share.get('1,2')!, share.get('0,2')!];
    }
    const keep = ids[0];
    const K = clusters.get(keep)!;                     // small-into-large: absorb into the reference
    selfm.delete(keep);
    for (const key of relKeys.get(keep) ?? []) relMemo.delete(key);
    relKeys.delete(keep);
    for (const i of ids.slice(1)) {
      const c = removeCluster(i);
      selfm.delete(i);
      const fresh: El[] = [];
      for (const [e, pose] of c.els) if (!K.els.has(e)) { K.els.set(e, pose); fresh.push(e); }
      register(keep, fresh);
      K.fixed = K.fixed || c.fixed;
    }
    steps.push(new Step(ids, pairs, dpairs, ppp));
    return keep;
  };

  const neighbours = (a: number): Set<number> => {
    const nb = new Set<number>();
    for (const e of clusters.get(a)!.els.keys()) for (const c of of.get(e)!) nb.add(c);
    for (const r of cdirs.get(a)!.keys()) for (const c of dirOf.get(r) ?? []) nb.add(c);
    nb.delete(a);
    return nb;
  };

  const maximalClusters = (): number[] => {
    const ids = [...clusters.keys()].sort((a, b) => a - b);
    const keys = new Map(ids.map((i) => [i, clusters.get(i)!.els] as const));
    return ids.filter((cid) => !ids.some((o) => {
      if (o === cid) return false;
      const a = keys.get(cid)!, b = keys.get(o)!;
      if (a.size >= b.size) return false;
      for (const e of a.keys()) if (!b.has(e)) return false;
      return true;
    }));
  };

  /** Worklist: a cluster is re-examined when it is created or a neighbour changes. */
  const treeMerges = (seedIds: number[]): void => {
    const work: number[] = [...seedIds];
    let head = 0;
    const queued = new Set(work);
    while (head < work.length) {
      const a = work[head++];
      queued.delete(a);
      if (!clusters.has(a)) continue;
      const nbs = [...neighbours(a)].sort((x, y) => x - y);
      let out = -1;
      for (const b of nbs) if (determined([a, b])) { out = merge([a, b]); break; }
      if (out < 0) {
        outer:
        for (let i = 0; i < nbs.length; i++) {
          const nbB = neighbours(nbs[i]);
          for (let j = i + 1; j < nbs.length; j++) {
            if (nbB.has(nbs[j]) && determined([a, nbs[i], nbs[j]])) {
              out = merge([a, nbs[i], nbs[j]]);
              break outer;
            }
          }
        }
      }
      if (out >= 0) {
        for (const x of [out, ...[...neighbours(out)].sort((p, q) => p - q)]) {
          if (!queued.has(x)) { work.push(x); queued.add(x); }
        }
      }
      if (head > 4096 && head * 2 > work.length) { work.splice(0, head); head = 0; }
    }
  };

  /** Smallest rigid subset of >= 4 clusters found by greedy growth from every seed (pairs and
   *  triples are already exhausted).  null if nothing rigid within `coreMax`. */
  const findCore = (): number[] | null => {
    let best: number[] | null = null;
    const live = maximalClusters();
    if (live.length > 400) return null;
    for (const seed2 of live) {
      const S = [seed2];
      const inS = new Set([seed2]);
      while (S.length < coreMax && (best === null || S.length + 1 < best.length)) {
        const frontier = new Set<number>();
        for (const x of S) for (const nb of neighbours(x)) if (!inS.has(nb)) frontier.add(nb);
        if (!frontier.size) break;
        let bestN = -1, bestD = Infinity, bestSize = Infinity;
        for (const nb of [...frontier].sort((a, b) => a - b)) {
          const d = deficiency([...S, nb]);
          const size = clusters.get(nb)!.els.size;
          if (d < bestD || (d === bestD && size < bestSize)) { bestD = d; bestSize = size; bestN = nb; }
        }
        S.push(bestN);
        inS.add(bestN);
        if (bestD === 0) {
          if (best === null || S.length < best.length) best = [...S];
          break;
        }
      }
    }
    return best;
  };

  treeMerges([...clusters.keys()].sort((a, b) => a - b));
  for (;;) {
    const core = findCore();
    if (!core) break;
    const out = merge(core);
    treeMerges([out, ...[...neighbours(out)].sort((a, b) => a - b)]);
  }
  return new Plan(graph, leaves, ground, singletons, steps, maximalClusters());
}

/* -- execution ---------------------------------------------------------------------------- */

function worldPose(graph: ConstraintGraph, e: El): Pose {
  if (elKind(e) === 'P') {
    const p = graph.members[elIdx(e)][0];
    return Float64Array.of(p.x.value, p.y.value);
  }
  if (e === X_AXIS) return Float64Array.from(X_POSE);
  if (elKind(e) === 'L') return Float64Array.from(lineNormal(graph.lines[elIdx(e)]));
  const [ea, eb] = graph.virtual[elIdx(e)];
  const a = worldPose(graph, ea), b = worldPose(graph, eb);
  return Float64Array.from(normalOf(a[0], a[1], b[0], b[1]));
}

/** Poses of a 2-element cluster satisfying its edge, nearest the current geometry. */
function leafPoses(graph: ConstraintGraph, edge: Edge): Map<El, Pose> {
  const a = worldPose(graph, edge.a), b = worldPose(graph, edge.b);
  const v = edge.value();
  const out = new Map<El, Pose>();
  if (edge.kind === 'PP') {
    const dx = b[0] - a[0], dy = b[1] - a[1];
    const L = Math.hypot(dx, dy);
    const ux = L > 1e-12 ? dx / L : 1, uy = L > 1e-12 ? dy / L : 0;
    out.set(edge.a, a);
    out.set(edge.b, Float64Array.of(a[0] + v * ux, a[1] + v * uy));
    return out;
  }
  const nx = b[0], ny = b[1], c = b[2];                 // PL: n.p - c = v
  const off = nx * a[0] + ny * a[1] - c - v;
  out.set(edge.a, Float64Array.of(a[0] - off * nx, a[1] - off * ny));
  out.set(edge.b, b);
  return out;
}

/** Triangle merge with all shared elements points: ref&B = {x}, B&C = {y}, C&ref = {z}.
 *  In ref's frame y is a circle-circle intersection; `sign` is the chirality — the
 *  orientation of the triangle (x, z, y), invariant under rigid motions of the whole. */
function mergePPP(ref: Cluster, B: Cluster, C: Cluster, x: El, y: El, z: El, sign: number):
    [Float64Array, Float64Array] {
  const xa = ref.els.get(x)!, za = ref.els.get(z)!;
  const bx = B.els.get(x)!, by = B.els.get(y)!, cz = C.els.get(z)!, cy = C.els.get(y)!;
  const dB = Math.hypot(by[0] - bx[0], by[1] - bx[1]);
  const dC = Math.hypot(cy[0] - cz[0], cy[1] - cz[1]);
  const ex = za[0] - xa[0], ey = za[1] - xa[1];
  const L = Math.hypot(ex, ey);
  const ux = L > 1e-12 ? ex / L : 1, uy = L > 1e-12 ? ey / L : 0;
  const aa = L > 1e-12 ? (dB * dB - dC * dC + L * L) / (2 * L) : 0;
  const h2 = dB * dB - aa * aa;
  const h = h2 > 0 ? Math.sqrt(h2) : 0;
  const fx = xa[0] + aa * ux, fy = xa[1] + aa * uy;
  const ya = sign > 0                                   // +1: y left of x->z
    ? Float64Array.of(fx - h * uy, fy + h * ux)
    : Float64Array.of(fx + h * uy, fy - h * ux);
  return [fit2(bx, by, xa, ya), fit2(cz, cy, za, ya)];
}

/** Write a point element's pose back to every Point of its coincidence class. */
export function writePoint(graph: ConstraintGraph, e: El, pose: Pose): void {
  if (elKind(e) !== 'P') return;
  for (const p of graph.members[elIdx(e)]) {
    p.x.value = pose[0];
    p.y.value = pose[1];
  }
}

/** Transform for an unfixed root: elements already placed by earlier roots are aligned
 *  exactly (>= 2 shared points: a rigid fit on them; 1 shared point: it pins the translation
 *  and the rest vote on the rotation); the remainder is least-change onto current geometry. */
function placeRoot(c: Cluster, placed: Map<El, Pose>, g: ConstraintGraph): Float64Array {
  const pts = [...c.els.keys()].filter((e) => elKind(e) === 'P');
  const shared = pts.filter((e) => placed.has(e));
  const xy = (p: Pose): [number, number] => [p[0], p[1]];
  if (shared.length >= 2) {
    return procrustes(shared.map((e) => xy(c.els.get(e)!)), shared.map((e) => xy(placed.get(e)!)));
  }
  const src = pts.map((e) => xy(c.els.get(e)!));
  const dst = pts.map((e) => xy(placed.get(e) ?? worldPose(g, e)));
  let T = procrustes(src, dst);
  if (shared.length === 1) {
    const e = shared[0];
    const moved = applyT(T, e, c.els.get(e)!);
    T = Float64Array.from(T);
    T[2] += placed.get(e)![0] - moved[0];
    T[3] += placed.get(e)![1] - moved[1];
  }
  return T;
}

/** Replay the plan on the current sketch values and write the result back.
 *  `capture = i` returns copies of the clusters entering step i instead (no write-back). */
export function execute(plan: Plan, capture: number | null = null): Cluster[] | null {
  const g = plan.graph;
  const cl = new Map<number, Cluster>();
  const gels = new Map<El, Pose>([[X_AXIS, Float64Array.from(X_POSE)]]);
  for (const i of g.groundPoints) gels.set(el('P', i), worldPose(g, el('P', i)));
  cl.set(plan.groundId, new Cluster(plan.groundId, gels, true));
  for (const [cid, ei] of plan.leaves) cl.set(cid, new Cluster(cid, leafPoses(g, g.edges[ei]), false));
  for (const [cid, e] of plan.singletons) cl.set(cid, new Cluster(cid, new Map([[e, worldPose(g, e)]]), false));

  for (let si = 0; si < plan.steps.length; si++) {
    const st = plan.steps[si];
    const parts = st.ids.map((i) => { const c = cl.get(i)!; cl.delete(i); return c; });
    if (capture === si) return parts.map((c) => c.clone());
    const ref = parts[0];
    const movable = parts.slice(1);
    let Ts: Float64Array[];
    if (st.ppp) {
      const [x, y, z] = st.ppp;
      if (st.branch === null || !plan.stickyBranches) {
        // sketch-guided chirality: the same signed area the drag's order-type guards watch
        const tri: Triangle = [g.members[elIdx(x)][0], g.members[elIdx(z)][0], g.members[elIdx(y)][0]];
        st.branch = orientation(tri[0], tri[1], tri[2]) >= 0 ? 1 : -1;
      }
      Ts = mergePPP(ref, movable[0], movable[1], x, y, z, st.branch);
    } else if (movable.length) {
      // identity is the natural warm start: leaves are re-derived from the current geometry,
      // so the root the sketch is on is the one nearest the identity (sticky by nature)
      const sys = mergeSystem(parts, st.pairs, st.dpairs, movable.length);
      const u0 = new Float64Array(3 * movable.length);
      let [u, res] = newtonSmall(sys, u0);
      if (res > 1e-9) u = doglegSmall(sys, u0);   // cores or bad warm starts: globalise
      Ts = movable.map((_, i) => makeT(u[3 * i], u[3 * i + 1], u[3 * i + 2]));
    } else {
      Ts = [];
    }
    const els = ref.els;                          // absorb in place (parts were removed above)
    movable.forEach((c, i) => {
      for (const [e, pose] of c.els) if (!els.has(e)) els.set(e, applyT(Ts[i], e, pose));
      ref.fixed = ref.fixed || c.fixed;
    });
    cl.set(st.out, ref);
  }

  // place the roots: fixed ones are in the world frame; others least-change onto the current
  // geometry, aligning any element already placed by an earlier root exactly
  const placed = new Map<El, Pose>();
  const order = [...plan.roots].sort((a, b) => {
    const ca = cl.get(a)!, cb = cl.get(b)!;
    return (Number(!ca.fixed) - Number(!cb.fixed)) || (cb.els.size - ca.els.size) || (a - b);
  });
  for (const rid of order) {
    const c = cl.get(rid)!;
    if (!c.fixed) {
      const T = placeRoot(c, placed, g);
      for (const [e, pose] of c.els) c.els.set(e, applyT(T, e, pose));
    }
    for (const [e, pose] of c.els) if (!placed.has(e)) placed.set(e, pose);
  }
  for (const [e, pose] of placed) writePoint(g, e, pose);
  for (const circle of [...g.sketch.circles, ...g.sketch.arcs] as (Circle | Arc)[]) {
    const r = g.knownRadius.get(circle.radius);
    if (r !== undefined && !circle.radius.fixed) circle.radius.value = r;
  }
  return null;
}

/* -- plan solver ---------------------------------------------------------------------------- */

export interface PlanResult {
  success: boolean;
  maxResidual: number;
  fellBack: boolean;
  timeS: number;
  plan: Plan;
  numeric: SolveResult | null;
}

const now = (): number => (typeof performance !== 'undefined' ? performance.now() : Date.now()) / 1000;

/** The same outcome in the solver's common result type (method 'plan' or the fallback's). */
export function asSolveResult(pr: PlanResult): SolveResult {
  if (pr.numeric) return pr.numeric;
  return {
    success: pr.success, status: 0, message: 'plan',
    residualNorm: pr.maxResidual, maxResidual: pr.maxResidual,
    nfev: pr.plan.steps.length, njev: 0, timeS: pr.timeS, method: 'plan', iterations: 0, rank: null,
  };
}

/** Compile once per topology (graph + decomposition + a System for verification); `solve()`
 *  replays the plan and falls back to the numeric core when the residual says the plan did
 *  not (fully) determine the sketch. */
export class PlanSolver {
  readonly graph: ConstraintGraph;
  readonly plan: Plan;
  readonly system: System;

  constructor(readonly sketch: Sketch, sticky = false) {
    this.graph = build(sketch);
    this.plan = decompose(this.graph);
    this.plan.stickyBranches = sticky;
    this.system = new System(sketch);
  }

  dispose(): void {
    this.system.dispose();
  }

  /** Flip the root of every closed-form construction that places `e`, recording the choice in
   *  the sketch (root choices are document state — `solve()` reads them back). */
  flip(e: El): number {
    const n = this.plan.flip(e);
    if (n) for (const [k, v] of this.plan.branches()) this.sketch.branches.set(k, v);
    return n;
  }

  solve(tol = 1e-9, fallback = true, method: Method = 'dogleg'): PlanResult {
    const t0 = now();
    // root choices are document state, read every solve — a plan cached per topology must not
    // replay a stale branch after the user flips one
    this.plan.applyBranches(this.sketch.branches);
    execute(this.plan);
    this.system.refreshConsts();                 // dimensions may have been edited since compile
    let mx = this.system.maxHardResidual();
    let numeric: SolveResult | null = null;
    let fellBack = false;
    if (mx > tol * this.system.scale && fallback) {
      fellBack = true;
      numeric = this.system.solve({ method });
      mx = numeric.maxResidual;
    }
    for (const [k, v] of this.plan.branches()) this.sketch.branches.set(k, v);
    return { success: mx <= 1e-6 * this.system.scale, maxResidual: mx, fellBack, timeS: now() - t0, plan: this.plan, numeric };
  }
}

/** The closed-form merges' triangles (x, z, y) as Points — the order-type invariants to guard. */
export function pppTriangles(plan: Plan): Triangle[] {
  const g = plan.graph;
  const out: Triangle[] = [];
  for (const st of plan.steps) {
    if (!st.ppp) continue;
    const [x, y, z] = st.ppp;
    out.push([g.members[elIdx(x)][0], g.members[elIdx(z)][0], g.members[elIdx(y)][0]]);
  }
  return out;
}

/** DCM-style drag: the dragged point joins the ground (fixed at the cursor) and the cached
 *  plan replays per frame — no graph analysis while dragging, recorded roots are sticky, and
 *  under-constrained roots move least.  Large cursor jumps are taken in increments so the
 *  solution tracks its branch.  If the plan cannot determine the sketch with the point pinned
 *  (fully constrained sketches, unsupported constraints) the numeric pull/polish `Drag` takes
 *  over — pass `guards` from an already-compiled plan to keep graph analysis out entirely. */
export class PlanDrag {
  solver!: PlanSolver;
  numeric: Drag | null = null;
  private maxStep: number;
  private guards: Triangle[] | null;

  constructor(readonly sketch: Sketch, readonly point: Point, x: number, y: number,
              guards: Triangle[] | null = null, maxStepRel = 0.05) {
    this.maxStep = maxStepRel * Math.max(1, sketch.extent());
    this.guards = guards;
    const x0 = sketch.getX();
    const was = point.isFixed;
    point.fix(true);
    let usable = false;
    try {
      this.solver = new PlanSolver(sketch, true);
      // the plan can drive the drag iff it understands every constraint, pinning the point
      // does not over-determine the sketch, and the replay reproduces the configuration
      usable = !this.solver.graph.unsupported.length && !this.overDetermined()
        && this.replay(point.xy[0], point.xy[1]) <= 1e-9 * this.solver.system.scale;
    } finally {
      point.fix(was);
    }
    if (!usable) {
      sketch.setX(x0);
      this.numeric = new Drag(sketch, point, x, y, 'dogleg', 1.0, this.guardTriangles());
    }
  }

  /** True while the cached plan is driving the drag (false once it handed over). */
  get usable(): boolean {
    return this.numeric === null;
  }

  /** Structural redundancy of the pinned system — the cheap half of a diagnosis, on the
   *  System the solver already compiled. */
  private overDetermined(): boolean {
    const { adj } = this.solver.system.structure();
    return adj.length > dulmageMendelsohn(adj, this.solver.system.nFree).rank;
  }

  /** Order-type invariants for the numeric path: the closed-form triangles of the sketch's own
   *  (unpinned) plan.  Computed at most once per drag — never inside a move. */
  guardTriangles(): Triangle[] {
    if (this.guards === null) this.guards = pppTriangles(decompose(build(this.sketch)));
    return this.guards;
  }

  private replay(x: number, y: number): number {
    this.point.x.value = x;
    this.point.y.value = y;
    execute(this.solver.plan);
    return this.solver.system.maxHardResidual();
  }

  move(x: number, y: number): SolveResult {
    if (this.numeric) return this.numeric.move(x, y);
    const t0 = now();
    const xPrev = this.sketch.getX();
    const [px, py] = this.point.xy;
    const path = increments(px, py, x, y, this.maxStep);
    let mx = 0;
    for (const [tx, ty] of path) {
      mx = this.replay(tx, ty);
      if (mx > 1e-6 * this.solver.system.scale) {
        // the plan cannot follow (a limit of the geometry was hit): hand over to the numeric
        // drag from the last good state
        this.sketch.setX(xPrev);
        this.numeric = new Drag(this.sketch, this.point, px, py, 'dogleg', 1.0, this.guardTriangles());
        return this.numeric.move(x, y);
      }
    }
    return {
      success: true, status: 0, message: 'plan-drag', residualNorm: mx, maxResidual: mx,
      nfev: path.length, njev: 0, timeS: now() - t0, method: 'plan', iterations: 0, rank: null,
    };
  }

  end(): void {
    if (this.numeric) this.numeric.end();
    this.solver.dispose();
  }

  get flips(): Triangle[] {
    return this.numeric ? this.numeric.flips : [];
  }

  branches(): Map<string, number> {
    return this.solver.plan.branches();
  }
}
