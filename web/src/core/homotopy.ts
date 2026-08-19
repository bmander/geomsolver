/* Stage 5 — homotopy continuation to enumerate the solutions of a small merge system (a
 * decomposition core or a closed-form triangle): "we can show you the other solutions".
 *
 * The merge system in the (c, s, tx, ty) parametrisation per movable cluster is polynomial:
 * shared points, line normals and direction rows are linear in the unknowns, line offsets are
 * bilinear, and c^2 + s^2 = 1 is quadratic.  We square the system with random complex
 * combinations (linear rows among themselves, degree-2 rows among themselves), keep the
 * linear part fixed along the path, and run a total-degree homotopy on the quadratic rows
 * with the gamma trick:
 *
 *     H(w, t) = (1 - t) * gamma * (w_sigma^2 - 1)  +  t * Qtilde(w)     (with Atilde w = btilde)
 *
 * tracked from the 2^(n_Q) start points by Euler prediction and Newton correction in complex
 * arithmetic.  Real endpoints (polished on the original system) are the alternatives, sorted
 * by distance from the current solution.  Small cores only — the number of paths is
 * exponential in the number of rotations, which is exactly the cost decomposition minimises.
 */
import { El, elKind } from './cgraph.js';
import { CMat, cmat, cmatvec, cmulReal, cnorm, csolve, freeColumns } from './complex.js';
import { Cluster, Plan, Step, applyT, execute, makeT, writePoint } from './decompose.js';
import { absmax, mat, rankRrqr } from './linalg.js';
import { Rng } from './rng.js';

export interface Alternative {
  /** Transform (theta, tx, ty) per movable cluster, relative to the current leaves. */
  u: Float64Array;
  /** ||w - w_identity||: 0 for the root the sketch is on. */
  distance: number;
  /** Where a requested point element would land. */
  location: [number, number] | null;
}

export const isCurrent = (a: Alternative): boolean => a.distance < 1e-6;

/** Merge system F(w) = [A w - b ; Q(w)] in (c, s, tx, ty) per movable cluster: A holds the
 *  constant degree-1 rows, Q the degree-2 rows (line offsets and c^2 + s^2 - 1). */
class Poly {
  readonly k: number;
  readonly n: number;
  readonly A: Float64Array;      /* rowsA x n, row-major */
  readonly rowsA: number;
  readonly b: Float64Array;
  readonly mQ: number;
  private off: [[number, number, number, number], [number, number, number, number]][] = [];

  constructor(readonly parts: Cluster[], step: Step) {
    this.k = parts.length - 1;
    this.n = 4 * this.k;
    const n = this.n;
    const rows: number[][] = [];
    const rhs: number[] = [];
    const offsets: [number, number, El][] = [];

    /** Affine part of a pose (2 rows): coefficient matrix and constant vector.  Points
     *  contribute both coordinates; lines contribute their normal (the offset coordinate is
     *  bilinear and lives in Q). */
    const linPose = (ci: number, e: El): [number[][], [number, number]] => {
      const p = parts[ci].els.get(e)!;
      const M = [new Array<number>(n).fill(0), new Array<number>(n).fill(0)];
      if (ci === 0) return [M, [p[0], p[1]]];
      const o = 4 * (ci - 1);
      const a = p[0], b = p[1];
      const isP = elKind(e) === 'P';
      M[0][o] = a; M[0][o + 1] = -b; M[0][o + 2] = isP ? 1 : 0; M[0][o + 3] = 0;
      M[1][o] = b; M[1][o + 1] = a; M[1][o + 2] = 0; M[1][o + 3] = isP ? 1 : 0;
      return [M, [0, 0]];
    };

    for (const [i, j, e] of step.pairs) {
      const [Ai, ci] = linPose(i, e);
      const [Aj, cj] = linPose(j, e);
      rows.push(Ai[0].map((v, t) => v - Aj[0][t]), Ai[1].map((v, t) => v - Aj[1][t]));
      rhs.push(cj[0] - ci[0], cj[1] - ci[1]);
      if (elKind(e) !== 'P') offsets.push([i, j, e]);
    }
    for (const [i, j, la, lb, phi] of step.dpairs) {      // n_b' - rot(phi) n_a' = 0
      const [Aa, ca] = linPose(i, la);
      const [Ab, cb] = linPose(j, lb);
      const c = Math.cos(phi), s = Math.sin(phi);
      rows.push(
        Ab[0].map((v, t) => v - (c * Aa[0][t] - s * Aa[1][t])),
        Ab[1].map((v, t) => v - (s * Aa[0][t] + c * Aa[1][t])),
      );
      rhs.push(c * ca[0] - s * ca[1] - cb[0], s * ca[0] + c * ca[1] - cb[1]);
    }
    this.rowsA = rows.length;
    this.A = new Float64Array(this.rowsA * n);
    rows.forEach((r, i) => this.A.set(r, i * n));
    this.b = Float64Array.from(rhs);
    this.mQ = offsets.length + this.k;
    // per-offset constants, hoisted out of the tracking loops
    this.off = offsets.map(([i, j, e]) => {
      const pi = parts[i].els.get(e)!, pj = parts[j].els.get(e)!;
      return [[i, pi[0], pi[1], pi[2]], [j, pj[0], pj[1], pj[2]]] as
        [[number, number, number, number], [number, number, number, number]];
    });
  }

  /** Offset coordinate of a line pose under a cluster's transform; accumulates into `grad`. */
  private offset(wr: Float64Array, wi: Float64Array, d: [number, number, number, number],
                 gr: Float64Array | null, gi: Float64Array | null, sign: number): [number, number] {
    const [ci, nx, ny, cc] = d;
    if (ci === 0) return [cc, 0];
    const o = 4 * (ci - 1);
    const cr = wr[o], cim = wi[o], sr = wr[o + 1], si = wi[o + 1];
    const txr = wr[o + 2], txi = wi[o + 2], tyr = wr[o + 3], tyi = wi[o + 3];
    const n0r = cr * nx - sr * ny, n0i = cim * nx - si * ny;
    const n1r = sr * nx + cr * ny, n1i = si * nx + cim * ny;
    if (gr && gi) {
      gr[o] += sign * (nx * txr + ny * tyr);
      gi[o] += sign * (nx * txi + ny * tyi);
      gr[o + 1] += sign * (-ny * txr + nx * tyr);
      gi[o + 1] += sign * (-ny * txi + nx * tyi);
      gr[o + 2] += sign * n0r; gi[o + 2] += sign * n0i;
      gr[o + 3] += sign * n1r; gi[o + 3] += sign * n1i;
    }
    return [
      cc + (n0r * txr - n0i * txi) + (n1r * tyr - n1i * tyi),
      (n0r * txi + n0i * txr) + (n1r * tyi + n1i * tyr),
    ];
  }

  /** Quadratic rows and (optionally) their Jacobian — one pass, since the offset rows produce
   *  value and gradient together. */
  QJ(wr: Float64Array, wi: Float64Array, wantJac = true): { qr: Float64Array; qi: Float64Array; J: CMat | null } {
    const qr = new Float64Array(this.mQ), qi = new Float64Array(this.mQ);
    const J = wantJac ? cmat(this.mQ, this.n) : null;
    this.off.forEach(([a, b], r) => {
      const gr = J ? J.re.subarray(r * this.n, (r + 1) * this.n) : null;
      const gi = J ? J.im.subarray(r * this.n, (r + 1) * this.n) : null;
      const va = this.offset(wr, wi, a, gr, gi, 1);
      const vb = this.offset(wr, wi, b, gr, gi, -1);
      qr[r] = va[0] - vb[0];
      qi[r] = va[1] - vb[1];
    });
    const nOff = this.off.length;
    for (let q = 0; q < this.k; q++) {
      const cr = wr[4 * q], ci = wi[4 * q], sr = wr[4 * q + 1], si = wi[4 * q + 1];
      qr[nOff + q] = cr * cr - ci * ci + sr * sr - si * si - 1;
      qi[nOff + q] = 2 * cr * ci + 2 * sr * si;
      if (J) {
        J.re[(nOff + q) * this.n + 4 * q] = 2 * cr;
        J.im[(nOff + q) * this.n + 4 * q] = 2 * ci;
        J.re[(nOff + q) * this.n + 4 * q + 1] = 2 * sr;
        J.im[(nOff + q) * this.n + 4 * q + 1] = 2 * si;
      }
    }
    return { qr, qi, J };
  }

  /** F(w) = [A w - b ; Q(w)]. */
  F(wr: Float64Array, wi: Float64Array): [Float64Array, Float64Array] {
    const { qr, qi } = this.QJ(wr, wi, false);
    const m = this.rowsA + this.mQ;
    const fr = new Float64Array(m), fi = new Float64Array(m);
    for (let i = 0; i < this.rowsA; i++) {
      let sr = -this.b[i], si = 0;
      for (let j = 0; j < this.n; j++) { sr += this.A[i * this.n + j] * wr[j]; si += this.A[i * this.n + j] * wi[j]; }
      fr[i] = sr; fi[i] = si;
    }
    fr.set(qr, this.rowsA);
    fi.set(qi, this.rowsA);
    return [fr, fi];
  }

  J(wr: Float64Array, wi: Float64Array): CMat {
    const jq = this.QJ(wr, wi, true).J!;
    const out = cmat(this.rowsA + this.mQ, this.n);
    for (let i = 0; i < this.rowsA; i++) for (let j = 0; j < this.n; j++) out.re[i * this.n + j] = this.A[i * this.n + j];
    out.re.set(jq.re, this.rowsA * this.n);
    out.im.set(jq.im, this.rowsA * this.n);
    return out;
  }
}

function wToU(wr: Float64Array): Float64Array {
  const k = wr.length / 4;
  const u = new Float64Array(3 * k);
  for (let q = 0; q < k; q++) {
    const c = wr[4 * q], s = wr[4 * q + 1];
    u[3 * q] = Math.atan2(s, c);
    u[3 * q + 1] = wr[4 * q + 2];
    u[3 * q + 2] = wr[4 * q + 3];
  }
  return u;
}

export interface EnumerateOptions {
  locate?: El | null;
  seed?: number;
  maxPaths?: number;
  maxSteps?: number;
  divergeRel?: number;
}

/** Real solutions of the merge at `stepIndex` (the current one first).  Returns [] if the
 *  merge is not isolated (under-determined) or too large.  `locate` asks where that point
 *  element would land under each alternative. */
export function enumerateStep(plan: Plan, stepIndex: number, opts: EnumerateOptions = {}): Alternative[] {
  const { locate = null, seed = 0, maxPaths = 256, maxSteps = 400, divergeRel = 50 } = opts;
  const rng = new Rng(seed);
  const parts = execute(plan, stepIndex);
  if (!parts || parts.length < 2) return [];
  const step = plan.steps[stepIndex];
  const P = new Poly(parts, step);
  const n = P.n, k = P.k;
  const wId = new Float64Array(n);
  for (let q = 0; q < k; q++) wId[4 * q] = 1;               // the current solution: identity

  // -- square the system: Atilde w = btilde (rank r) and n - r combinations of the quadratic rows --
  const r = P.rowsA ? rankRrqr(mat(P.rowsA, n, P.A), 1e-9) : 0;
  const nQ = n - r;
  if (nQ <= 0 || P.mQ < nQ || 2 ** nQ > maxPaths) return [];
  const cRand = (rows: number, cols: number): CMat => {
    const M = cmat(rows, cols);
    for (let i = 0; i < rows * cols; i++) { M.re[i] = rng.normal(); M.im[i] = rng.normal(); }
    return M;
  };
  const M1 = cRand(r, P.rowsA);
  const M2 = cRand(nQ, P.mQ);
  const At = cmulReal(M1, P.A, P.rowsA, n);
  const [btR, btI] = cmatvec(M1, P.b, new Float64Array(P.rowsA));

  // -- start system: the same linear rows plus w_sigma^2 - 1 on variables free w.r.t. them --
  const sigma = freeColumns(At).free.slice(0, nQ);
  if (sigma.length < nQ) return [];
  const gAng = 2 * Math.PI * rng.next();
  const gammaR = Math.cos(gAng), gammaI = Math.sin(gAng);

  const cmul = (ar: number, ai: number, br: number, bi: number): [number, number] =>
    [ar * br - ai * bi, ar * bi + ai * br];

  /** Row q of the random combination M2 applied to the quadratic rows. */
  const m2q = (q: number, qr: Float64Array, qi: Float64Array): [number, number] => {
    let mr = 0, mi = 0;
    for (let j = 0; j < P.mQ; j++) {
      const ar = M2.re[q * P.mQ + j], ai = M2.im[q * P.mQ + j];
      mr += ar * qr[j] - ai * qi[j];
      mi += ar * qi[j] + ai * qr[j];
    }
    return [mr, mi];
  };

  /** The start system's row q: gamma * (w_sigma^2 - 1). */
  const startRow = (wr: Float64Array, wi: Float64Array, q: number): [number, number] => {
    const s = sigma[q];
    const [g2r, g2i] = cmul(wr[s], wi[s], wr[s], wi[s]);
    return cmul(gammaR, gammaI, g2r - 1, g2i);
  };

  /** H(w, t) and its Jacobian — Poly's offset rows give value and gradient in one pass. */
  const HJ = (wr: Float64Array, wi: Float64Array, t: number): { hr: Float64Array; hi: Float64Array; J: CMat } => {
    const { qr, qi, J: jq } = P.QJ(wr, wi, true);
    const hr = new Float64Array(n), hi = new Float64Array(n);
    const J = cmat(n, n);
    for (let i = 0; i < r; i++) {
      let sr = -btR[i], si = -btI[i];
      for (let j = 0; j < n; j++) {
        const ar = At.re[i * n + j], ai = At.im[i * n + j];
        sr += ar * wr[j] - ai * wi[j];
        si += ar * wi[j] + ai * wr[j];
        J.re[i * n + j] = ar;
        J.im[i * n + j] = ai;
      }
      hr[i] = sr; hi[i] = si;
    }
    for (let q = 0; q < nQ; q++) {
      // (1-t) * gamma * (w_s^2 - 1)  +  t * (M2 Q)
      const s = sigma[q];
      const [sr0, si0] = startRow(wr, wi, q);
      const [mr, mi] = m2q(q, qr, qi);
      hr[r + q] = (1 - t) * sr0 + t * mr;
      hi[r + q] = (1 - t) * si0 + t * mi;
      for (let j = 0; j < n; j++) {
        let jr = 0, ji = 0;
        for (let p = 0; p < P.mQ; p++) {
          const ar = M2.re[q * P.mQ + p], ai = M2.im[q * P.mQ + p];
          const br = jq!.re[p * n + j], bi = jq!.im[p * n + j];
          jr += ar * br - ai * bi;
          ji += ar * bi + ai * br;
        }
        J.re[(r + q) * n + j] = t * jr;
        J.im[(r + q) * n + j] = t * ji;
      }
      const [dr, di] = cmul(gammaR, gammaI, 2 * wr[s], 2 * wi[s]);
      J.re[(r + q) * n + s] += (1 - t) * dr;
      J.im[(r + q) * n + s] += (1 - t) * di;
    }
    return { hr, hi, J };
  };

  /** dH/dt: the quadratic rows swap the start system for the target one. */
  const Ht = (wr: Float64Array, wi: Float64Array): [Float64Array, Float64Array] => {
    const { qr, qi } = P.QJ(wr, wi, false);
    const dr = new Float64Array(n), di = new Float64Array(n);
    for (let q = 0; q < nQ; q++) {
      const [sr0, si0] = startRow(wr, wi, q);
      const [mr, mi] = m2q(q, qr, qi);
      dr[r + q] = -sr0 + mr;
      di[r + q] = -si0 + mi;
    }
    return [dr, di];
  };

  // start points: every sign pattern on the sigma variables; one factorisation, all right-hand sides
  const nPaths = 2 ** nQ;
  const S = cmat(n, n);
  for (let i = 0; i < r; i++) for (let j = 0; j < n; j++) { S.re[i * n + j] = At.re[i * n + j]; S.im[i * n + j] = At.im[i * n + j]; }
  for (let q = 0; q < nQ; q++) S.re[(r + q) * n + sigma[q]] = 1;
  const RHS = cmat(n, nPaths);
  for (let p = 0; p < nPaths; p++) {
    for (let i = 0; i < r; i++) { RHS.re[i * nPaths + p] = btR[i]; RHS.im[i * nPaths + p] = btI[i]; }
    for (let q = 0; q < nQ; q++) RHS.re[(r + q) * nPaths + p] = (p >> q) & 1 ? 1 : -1;
  }
  if (!csolve(n, S, RHS)) return [];

  const newton = (wr: Float64Array, wi: Float64Array, t: number, iters = 4, tol = 1e-10): boolean => {
    for (let it = 0; it < iters; it++) {
      const { hr, hi, J } = HJ(wr, wi, t);
      if (cnorm(hr, hi) < tol * (1 + cnorm(wr, wi))) return true;
      const B = cmat(n, 1);
      B.re.set(hr); B.im.set(hi);
      if (!csolve(n, J, B)) return false;
      for (let i = 0; i < n; i++) { wr[i] -= B.re[i]; wi[i] -= B.im[i]; }
    }
    const { hr, hi } = HJ(wr, wi, t);
    return cnorm(hr, hi) < 1e-6 * (1 + cnorm(wr, wi));
  };

  // Paths that run off to infinity are dead ends: cut them at a multiple of the sketch scale
  // (w holds cos/sin and translations, so an absolute bound would depend on the sketch size).
  let scale = 1;
  for (const v of P.b) scale = Math.max(scale, Math.abs(v));
  const diverge = divergeRel * scale;

  const ends: [Float64Array, Float64Array][] = [];
  for (let p = 0; p < nPaths; p++) {
    const wr = new Float64Array(n), wi = new Float64Array(n);
    for (let i = 0; i < n; i++) { wr[i] = RHS.re[i * nPaths + p]; wi[i] = RHS.im[i * nPaths + p]; }
    let t = 0, dt = 0.02;
    for (let it = 0; it < maxSteps; it++) {
      if (t >= 1 || cnorm(wr, wi) > diverge) break;
      const t1 = Math.min(1, t + dt);
      const { J } = HJ(wr, wi, t);
      const [dtr, dti] = Ht(wr, wi);
      const B = cmat(n, 1);
      B.re.set(dtr); B.im.set(dti);
      if (!csolve(n, J, B)) {
        dt *= 0.5;
        if (dt < 1e-10) break;
        continue;
      }
      const nr = Float64Array.from(wr), ni = Float64Array.from(wi);
      for (let i = 0; i < n; i++) { nr[i] -= B.re[i] * (t1 - t); ni[i] -= B.im[i] * (t1 - t); }
      const ok = newton(nr, ni, t1);
      const dr = new Float64Array(n), di = new Float64Array(n);
      for (let i = 0; i < n; i++) { dr[i] = nr[i] - wr[i]; di[i] = ni[i] - wi[i]; }
      if (ok && cnorm(dr, di) < 0.5 * (1 + cnorm(wr, wi))) {
        wr.set(nr); wi.set(ni);
        t = t1;
        dt = Math.min(0.2, dt * 1.5);
      } else {
        dt *= 0.5;
        if (dt < 1e-10) break;
      }
    }
    if (t >= 1 && cnorm(wr, wi) <= diverge) {
      for (let it = 0; it < 5; it++) {              // polish on the original system
        const [fr, fi] = P.F(wr, wi);
        if (cnorm(fr, fi) < 1e-12) break;
        const J = P.J(wr, wi);
        // least squares via the normal equations J^H J x = J^H f
        const m = J.rows;
        const A = cmat(n, n);
        const B = cmat(n, 1);
        for (let i = 0; i < n; i++) {
          for (let j = 0; j < n; j++) {
            let sr = 0, si = 0;
            for (let q = 0; q < m; q++) {
              const ar = J.re[q * n + i], ai = -J.im[q * n + i];
              const br = J.re[q * n + j], bi = J.im[q * n + j];
              sr += ar * br - ai * bi;
              si += ar * bi + ai * br;
            }
            A.re[i * n + j] = sr; A.im[i * n + j] = si;
          }
          let sr = 0, si = 0;
          for (let q = 0; q < m; q++) {
            const ar = J.re[q * n + i], ai = -J.im[q * n + i];
            sr += ar * fr[q] - ai * fi[q];
            si += ar * fi[q] + ai * fr[q];
          }
          B.re[i] = sr; B.im[i] = si;
        }
        if (!csolve(n, A, B)) break;
        for (let i = 0; i < n; i++) { wr[i] -= B.re[i]; wi[i] -= B.im[i]; }
      }
      ends.push([wr, wi]);
    }
  }

  const out: Alternative[] = [];
  const kept: Float64Array[] = [];
  let qOf: number | null = null;
  if (locate) {
    for (let i = 1; i < parts.length; i++) if (parts[i].els.has(locate)) { qOf = i - 1; break; }
  }
  for (const [wr, wi] of ends) {
    if (absmax(wi) > 1e-6 * (1 + absmax(wr))) continue;
    const zero = new Float64Array(n);
    const [fr, fi] = P.F(wr, zero);
    if (cnorm(fr, fi) > 1e-6) continue;
    if (kept.some((kv) => {
      let s = 0;
      for (let i = 0; i < n; i++) s += (wr[i] - kv[i]) ** 2;
      return Math.sqrt(s) < 1e-6;
    })) continue;
    kept.push(Float64Array.from(wr));
    const u = wToU(wr);
    let loc: [number, number] | null = null;
    if (locate && qOf !== null) {
      const T = makeT(u[3 * qOf], u[3 * qOf + 1], u[3 * qOf + 2]);
      const pos = applyT(T, locate, parts[qOf + 1].els.get(locate)!);
      loc = [pos[0], pos[1]];
    }
    let d = 0;                                             // the imaginary part is ~0 by now
    for (let i = 0; i < n; i++) d += (wr[i] - wId[i]) ** 2 + wi[i] ** 2;
    out.push({ u, distance: Math.sqrt(d), location: loc });
  }
  out.sort((a, b) => a.distance - b.distance);
  return out;
}

/** Put the sketch on this root: write the alternative placement of the merged clusters into
 *  the points (leaves are re-derived from geometry, so later replays stay on it), then replay
 *  the whole plan so dependent geometry follows.  Triangles also flip their branch. */
export function applyAlternative(plan: Plan, stepIndex: number, alt: Alternative): void {
  const parts = execute(plan, stepIndex);
  if (!parts) return;
  const g = plan.graph;
  const st = plan.steps[stepIndex];
  if (st.ppp && !isCurrent(alt) && st.branch !== null) {
    st.branch = -st.branch;
    for (const [k, v] of plan.branches()) g.sketch.branches.set(k, v);   // document state
  }
  parts.slice(1).forEach((c, q) => {
    const T = makeT(alt.u[3 * q], alt.u[3 * q + 1], alt.u[3 * q + 2]);
    for (const [e, pose] of c.els) writePoint(g, e, applyT(T, e, pose));
  });
  plan.stickyBranches = true;
  execute(plan);
}
