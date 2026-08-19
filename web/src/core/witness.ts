/* Stage 4 — the witness configuration method (Michelucci & Foufou 2006).
 *
 * Structural analysis (Stage 2) cannot see dependencies that follow from geometric theorems
 * (three concurrent altitudes, an EqualLength cycle, Pappus).  A witness is a configuration
 * with the sketch's incidence structure but generic dimensions; the Jacobian there tells the
 * truth about the system:
 *
 *   * rank deficiency in the rows = dependent constraints (theorem-induced ones included) —
 *     pivoted QR on J^T picks a maximal independent set and, for each leftover equation, the
 *     equations it is implied by;
 *   * the null space of J = the infinitesimal motions = exactly which DOFs remain and what
 *     they look like (rigid-body motions separated from internal ones, modes localised).
 *
 * The user's own sketch is often an adequate witness (it satisfies the incidences by
 * construction).  Otherwise we jitter every dimension the constraints declare and re-solve;
 * if that cannot converge we satisfy the incidence-type constraints alone from a perturbed
 * start.  The rank test is relative, and pivoted QR is cross-checked against the SVD.
 */
import { Constraint } from './constraints.js';
import { Mat, absmax, mat, minNormLstsq, norm, orthonormalize, rrqr, selectRows, svd, transpose } from './linalg.js';
import { Param, Sketch } from './model.js';
import { Rng } from './rng.js';
import { System } from './system.js';

export interface Dependency {
  constraint: Constraint;        /* a dependent (redundant) equation's constraint */
  impliedBy: Constraint[];       /* constraints whose equations span it */
  theorem: boolean;              /* structural analysis could not see it */
}

/** An infinitesimal motion: velocity per free parameter, scaled to unit max displacement. */
export interface Motion {
  velocity: Float64Array;
  rigid: boolean;                /* a rigid-body motion of the whole sketch */
  params: Param[];
}

export function movingParams(m: Motion, rel = 1e-3): Param[] {
  const mx = absmax(m.velocity) || 1;
  return m.params.filter((_, i) => Math.abs(m.velocity[i]) > rel * mx);
}

export interface WitnessReport {
  xWitness: Float64Array;
  usedCurrent: boolean;          /* the sketch itself served as witness */
  numericRank: number;
  dependencies: Dependency[];
  motions: Motion[];             /* null-space basis: rigid modes first, then internal DOFs */
  movable: number[];             /* free-parameter indices taking part in some motion */
  warnings: string[];
}

export const nDof = (w: WitnessReport): number => w.motions.length;
export const nInternalDof = (w: WitnessReport): number => w.motions.filter((m) => !m.rigid).length;

export function witnessSummary(w: WitnessReport): string {
  const parts = [
    `witness rank ${w.numericRank}`,
    `${nDof(w)} DOF (${nInternalDof(w)} internal, ${nDof(w) - nInternalDof(w)} rigid-body)`,
  ];
  if (w.dependencies.length) {
    const th = w.dependencies.filter((d) => d.theorem).length;
    parts.push(`${w.dependencies.length} dependent constraint(s)${th ? `, ${th} theorem-type` : ''}`);
  }
  return [...parts, ...w.warnings].join('; ');
}

/* -------------------------------------------------------------------------- */

/** A configuration with the sketch's incidence structure and generic dimensions.  Leaves the
 *  sketch's values and dimensions untouched. */
export function makeWitness(sketch: Sketch, seed = 0, jitter = 0.05, tol = 1e-8): Float64Array {
  const x0 = sketch.getX();
  const rng = new Rng(seed);
  const hard = sketch.hardConstraints();
  const dimensioned: [Constraint, string, string][] = [];
  for (const c of hard) for (const [name, kind] of c.dimensions()) dimensioned.push([c, name, kind]);
  const saved = dimensioned.map(([c, name]) => [c, name, (c as never as Record<string, number>)[name]] as const);
  const savedC = sketch.constraints;
  try {
    // 1. generic dimensions (lengths scaled, angles offset), re-solved from current geometry
    for (const [c, name, kind] of dimensioned) {
      const rec = c as never as Record<string, number>;
      const v = rec[name];
      rec[name] = kind === 'length' ? v * (1 + jitter * rng.normal()) : v + jitter * rng.normal();
    }
    sketch.constraints = hard;
    const sys = new System(sketch);
    try {
      const res = sys.solve({ maxIter: 60 });
      if (res.success && res.maxResidual <= tol * sys.scale) return sketch.getX();
    } finally {
      sys.dispose();
    }
    // 2. incidences only (always satisfiable) from a perturbed start
    sketch.setX(x0);
    sketch.constraints = hard.filter((c) => !c.dimensions().length);
    sketch.perturb(0.02 * Math.max(1, sketch.extent()), seed);
    const sys2 = new System(sketch);
    try {
      sys2.solve({ maxIter: 60 });
    } finally {
      sys2.dispose();
    }
    return sketch.getX();
  } finally {
    sketch.constraints = savedC;
    for (const [c, name, v] of saved) (c as never as Record<string, number>)[name] = v;
    sketch.setX(x0);
  }
}

/** Rows of the null-space basis that are nonzero: the parameters taking part in some
 *  infinitesimal motion of the configuration. */
export function movableColumns(N: Mat, rtol = 1e-8): number[] {
  if (!N.rows || !N.cols) return [];
  const w = new Float64Array(N.rows);
  let wmax = 0;
  for (let i = 0; i < N.rows; i++) {
    w[i] = absmax(N.data.subarray(i * N.cols, (i + 1) * N.cols));
    wmax = Math.max(wmax, w[i]);
  }
  const out: number[] = [];
  for (let i = 0; i < N.rows; i++) if (w[i] > rtol * wmax) out.push(i);
  return out;
}

/** Rank, dependencies and motions of the sketch's constraint system at a witness.
 *
 *  `overIds` are the constraints the structural analysis already put in its over-determined
 *  block; a dependency outside that set is theorem-type — invisible to the graph. */
export function analyze(sketch: Sketch, xWitness: Float64Array | null = null, opts: {
  system?: System | null;
  overIds?: ReadonlySet<Constraint>;
  rtol?: number;
  seed?: number;
} = {}): WitnessReport {
  const rtol = opts.rtol ?? 1e-9;
  const overIds = opts.overIds ?? new Set<Constraint>();
  const x0 = sketch.getX();
  const own = !(opts.system && opts.system.sketch === sketch);
  const sys = own ? new System(sketch) : opts.system!;
  try {
    const usedCurrent = xWitness === null && sys.maxHardResidual() <= 1e-9 * sys.scale;
    const xw = xWitness ?? (usedCurrent ? x0 : makeWitness(sketch, opts.seed ?? 0));
    sketch.setX(xw);
    const freeParams = Array.from(sys.free, (i) => sketch.params[i]);
    const dense = sys.jacobianDense(sys.z0());
    const hardRows: number[] = [];
    for (let i = 0; i < sys.nRes; i++) if (sys.hard[i]) hardRows.push(i);
    const J = selectRows(dense, hardRows);
    const { rowC } = sys.structure();
    const m = J.rows, n = J.cols;
    const warnings: string[] = [];
    if (m === 0 || n === 0) {
      const I = mat(n, n);
      for (let i = 0; i < n; i++) I.data[i * n + i] = 1;
      return {
        xWitness: xw, usedCurrent, numericRank: 0, dependencies: [],
        motions: classifyMotions(I, freeParams, sketch),
        movable: Array.from({ length: n }, (_, i) => i), warnings,
      };
    }
    // rank: RRQR on J^T (pivots = a maximal independent row set), cross-checked with the SVD
    // that also yields the null space
    const { rank: rankQr, piv } = rrqr(transpose(J), rtol);
    const { s, Vt } = svd(J, false);
    const mn = Math.min(m, n);
    let rankSvd = 0;
    if (mn && s[0] > 0) for (let i = 0; i < mn; i++) if (s[i] > rtol * s[0]) rankSvd++;
    let rank = rankQr;
    if (rankQr !== rankSvd) {
      warnings.push(`rank ambiguous: QR ${rankQr} vs SVD ${rankSvd} (near-degenerate witness)`);
      rank = Math.min(rankQr, rankSvd);
    }
    // dependent rows: the non-pivot rows, each expressed in the pivot rows' span (one
    // factorisation for all of them)
    const indep = Array.from(piv.subarray(0, rank));
    const depRows = Array.from(piv.subarray(rank)).filter((r) => rowC[r] !== undefined);
    const deps: Dependency[] = [];
    if (depRows.length && indep.length) {
      const A = transpose(selectRows(J, indep));           // n x rank
      const B = transpose(selectRows(J, depRows));         // n x |dep|
      const { x: coefs } = minNormLstsq(A, B);             // rank x |dep|
      depRows.forEach((r, col) => {
        const c = rowC[r];
        if (deps.some((d) => d.constraint === c)) return;
        const coef = Array.from({ length: rank }, (_, k) => coefs.data[k * coefs.cols + col]);
        const lim = 1e-6 * (absmax(coef) || 1);
        const order = coef.map((v, k) => [Math.abs(v), k] as const)
          .sort((a, b) => b[0] - a[0]).filter(([a]) => a > lim).map(([, k]) => k);
        const implied: Constraint[] = [];
        for (const k of order) {
          const s2 = rowC[indep[k]];
          if (s2 !== c && !implied.includes(s2)) implied.push(s2);
        }
        deps.push({ constraint: c, impliedBy: implied, theorem: !overIds.has(c) });
      });
    }
    const nn = n - rank;
    const N = mat(n, nn);
    for (let i = 0; i < n; i++) for (let j = 0; j < nn; j++) N.data[i * nn + j] = Vt.data[(rank + j) * n + i];
    return {
      xWitness: xw, usedCurrent, numericRank: rank, dependencies: deps,
      motions: classifyMotions(N, freeParams, sketch), movable: movableColumns(N), warnings,
    };
  } finally {
    sketch.setX(x0);
    if (own) sys.dispose();
  }
}

/** Split the null space into rigid-body modes (translations/rotation of everything that can
 *  move together) and internal DOFs; localise the internal ones (sparse basis). */
function classifyMotions(N: Mat, params: Param[], sketch: Sketch): Motion[] {
  const n = N.rows, d = N.cols;
  if (d === 0) return [];
  // rigid-body generators, from the model's own parameter identity (not from names)
  const axis = new Map<Param, [number, number, number]>();
  for (const pt of sketch.points) {
    axis.set(pt.x, [0, pt.x.value, pt.y.value]);
    axis.set(pt.y, [1, pt.x.value, pt.y.value]);
  }
  let cx = 0, cy = 0;
  if (sketch.points.length) {
    for (const p of sketch.points) { cx += p.x.value; cy += p.y.value; }
    cx /= sketch.points.length;
    cy /= sketch.points.length;
  }
  const tx = new Float64Array(n), ty = new Float64Array(n), rot = new Float64Array(n);
  params.forEach((p, i) => {
    const got = axis.get(p);
    if (!got) return;                             // a radius: invariant under rigid motions
    const [which, x, y] = got;
    (which === 0 ? tx : ty)[i] = 1;
    rot[i] = which === 0 ? -(y - cy) : x - cx;
  });
  const inNull = (v: Float64Array): boolean => {
    let s = 0;
    for (let j = 0; j < d; j++) {
      let acc = 0;
      for (let i = 0; i < n; i++) acc += N.data[i * d + j] * v[i];
      s += acc * acc;
    }
    return Math.sqrt(s) >= (1 - 1e-6) * norm(v);   // N has orthonormal columns
  };
  const scaled = (v: Float64Array): Float64Array => {
    const mx = absmax(v) || 1;
    const out = new Float64Array(n);
    for (let i = 0; i < n; i++) out[i] = v[i] / mx;
    return out;
  };
  const rigid: Float64Array[] = [];
  for (const v of [tx, ty, rot]) if (v.some((a) => a !== 0) && inNull(v)) rigid.push(scaled(v));
  const motions: Motion[] = rigid.map((v) => ({ velocity: v, rigid: true, params }));

  let Ni: Mat;
  if (rigid.length) {                             // internal DOFs = the null space minus the rigid span
    const Q = orthonormalize(rigid);
    const M = mat(n, d);
    for (let i = 0; i < n; i++) for (let j = 0; j < d; j++) M.data[i * d + j] = N.data[i * d + j];
    for (const q of Q) {
      const proj = new Float64Array(d);
      for (let j = 0; j < d; j++) {
        let acc = 0;
        for (let i = 0; i < n; i++) acc += q[i] * M.data[i * d + j];
        proj[j] = acc;
      }
      for (let i = 0; i < n; i++) for (let j = 0; j < d; j++) M.data[i * d + j] -= q[i] * proj[j];
    }
    const { U, s } = svd(M, true);
    const keep: number[] = [];
    for (let j = 0; j < s.length; j++) if (s[j] > 1e-6) keep.push(j);   // N orthonormal: absolute threshold
    Ni = mat(n, keep.length);
    for (let i = 0; i < n; i++) keep.forEach((j, c) => { Ni.data[i * keep.length + c] = U.data[i * U.cols + j]; });
  } else {
    Ni = N;
  }
  if (Ni.cols) {
    // localise: rotate the basis so each mode is 1 at a pivot parameter and 0 at the others
    const k = Ni.cols;
    const { piv } = rrqr(transpose(Ni));
    const rows = Array.from(piv.subarray(0, k));
    const A = transpose(selectRows(Ni, rows));    // k x k
    const B = transpose(Ni);                      // k x n
    const { x: sol } = minNormLstsq(A, B);        // k x n
    for (let c = 0; c < k; c++) {
      const v = new Float64Array(n);
      for (let i = 0; i < n; i++) v[i] = sol.data[c * sol.cols + i];
      motions.push({ velocity: scaled(v), rigid: false, params });
    }
  }
  return motions;
}
