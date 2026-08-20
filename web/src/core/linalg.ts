/* Dense linear algebra: thin wrappers over the core's own factorizations.
 *
 * Matrices are row-major `Mat { rows, cols, data }`.  There is no LAPACK/BLAS anywhere in the
 * project — the pivoted QR, the complete orthogonal decomposition, the SVD and the LU are ours,
 * and the Python suite checks them against numpy. */
import { core, withBuf } from './wasm.js';

export interface Mat {
  rows: number;
  cols: number;
  data: Float64Array;
}

export function mat(rows: number, cols: number, data?: ArrayLike<number>): Mat {
  const d = new Float64Array(rows * cols);
  if (data) d.set(data as Float64Array);
  return { rows, cols, data: d };
}

/** Rows i of A for i in `keep`, as a new matrix. */
export function selectRows(A: Mat, keep: ArrayLike<number>): Mat {
  const out = mat(keep.length, A.cols);
  for (let i = 0; i < keep.length; i++) {
    out.data.set(A.data.subarray(keep[i] * A.cols, (keep[i] + 1) * A.cols), i * A.cols);
  }
  return out;
}

export function transpose(A: Mat): Mat {
  const T = mat(A.cols, A.rows);
  for (let i = 0; i < A.rows; i++) {
    for (let j = 0; j < A.cols; j++) T.data[j * A.rows + i] = A.data[i * A.cols + j];
  }
  return T;
}

export const dot = (a: ArrayLike<number>, b: ArrayLike<number>): number => {
  let s = 0;
  for (let i = 0; i < a.length; i++) s += a[i] * b[i];
  return s;
};

export const norm = (a: ArrayLike<number>): number => Math.sqrt(dot(a, a));

export const absmax = (a: ArrayLike<number>): number => {
  let m = 0;
  for (let i = 0; i < a.length; i++) {
    const v = Math.abs(a[i]);
    if (v > m) m = v;
  }
  return m;
};

/** Minimum-norm least-squares solution of A X = B (neither is modified). */
export function minNormLstsq(A: Mat, B: Mat, rcond = 1e-12): { x: Mat; rank: number } {
  const { rows: m, cols: n } = A;
  const X = mat(n, B.cols);
  if (m === 0 || n === 0 || B.cols === 0) return { x: X, rank: 0 };
  return withBuf(m * n, 8, (a) => withBuf(m * B.cols, 8, (b) => withBuf(n * B.cols, 8, (x) => {
    a.set(A.data);
    b.set(B.data);
    const rank = core().gcs_min_norm_lstsq(m, n, B.cols, a.ptr, b.ptr, rcond, x.ptr);
    X.data.set(x.f64);
    return { x: X, rank };
  })));
}

export function minNormSolve(A: Mat, b: ArrayLike<number>, rcond = 1e-12):
    { x: Float64Array; rank: number } {
  const r = minNormLstsq(A, mat(A.rows, 1, b), rcond);
  return { x: r.x.data, rank: r.rank };
}

/** Rank-revealing QR: numerical rank and the column pivots. */
export function rrqr(A: Mat, rcond = 1e-10): { rank: number; piv: Int32Array } {
  const { rows: m, cols: n } = A;
  if (m === 0 || n === 0) return { rank: 0, piv: new Int32Array(0) };
  return withBuf(m * n, 8, (a) => withBuf(n, 4, (p) => {
    a.set(A.data);
    const rank = core().gcs_rrqr(m, n, a.ptr, rcond, p.ptr);
    return { rank, piv: p.i32.slice() };
  }));
}

export const rankRrqr = (A: Mat, rcond = 1e-10): number => rrqr(A, rcond).rank;

export interface Svd {
  U: Mat;      /* m x min(m, n) */
  s: Float64Array;
  Vt: Mat;     /* n x n */
}

export function svd(A: Mat, wantU = true): Svd {
  const { rows: m, cols: n } = A;
  const mn = Math.min(m, n);
  if (m === 0 || n === 0) return { U: mat(m, mn), s: new Float64Array(mn), Vt: mat(n, n) };
  return withBuf(m * n, 8, (a) => withBuf(Math.max(m * mn, 1), 8, (u) =>
    withBuf(mn, 8, (s) => withBuf(n * n, 8, (vt) => {
      a.set(A.data);
      core().gcs_svd(m, n, a.ptr, wantU ? u.ptr : 0, s.ptr, vt.ptr);
      return {
        U: wantU ? mat(m, mn, u.f64) : mat(m, 0),
        s: s.f64.slice(),
        Vt: mat(n, n, vt.f64),
      };
    }))));
}

/** (rank, null-space basis as n x (n-rank), singular values) — the shared seam that keeps
 *  diagnosis, witness analysis and decomposition agreeing on what "rank" means. */
export function rankAndNullspace(A: Mat, rcond = 1e-10):
    { rank: number; N: Mat; s: Float64Array } {
  const { rows: m, cols: n } = A;
  if (n === 0) return { rank: 0, N: mat(0, 0), s: new Float64Array(0) };
  return withBuf(Math.max(m * n, 1), 8, (a) => withBuf(n * n, 8, (nb) => withBuf(n, 8, (s) => {
    a.set(A.data);
    const rank = core().gcs_rank_nullspace(m, n, a.ptr, rcond, nb.ptr, s.ptr);
    const nn = n - rank;
    return { rank, N: mat(n, nn, nb.f64.subarray(0, n * nn)), s: s.f64.slice(0, Math.min(m, n)) };
  })));
}

/** Orthonormal basis of the column span of `cols` (modified Gram-Schmidt). */
export function orthonormalize(cols: Float64Array[], tol = 1e-12): Float64Array[] {
  const out: Float64Array[] = [];
  for (const c of cols) {
    const v = new Float64Array(c);
    for (const q of out) {
      const d = dot(q, v);
      for (let i = 0; i < v.length; i++) v[i] -= d * q[i];
    }
    const nv = norm(v);
    if (nv > tol) {
      for (let i = 0; i < v.length; i++) v[i] /= nv;
      out.push(v);
    }
  }
  return out;
}
