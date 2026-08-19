/* Just enough complex linear algebra for the homotopy continuation of Stage 5: matrices as
 * split real/imaginary Float64Arrays, an LU solve, and a reduced-row-echelon pass used to
 * pick a set of variables that is free with respect to the linear part of a merge system.
 *
 * This is an on-demand feature (enumerating the alternative solutions of one small merge),
 * so it stays in TypeScript rather than crossing the WebAssembly boundary. */

export interface CMat {
  rows: number;
  cols: number;
  re: Float64Array;
  im: Float64Array;
}

export function cmat(rows: number, cols: number): CMat {
  return { rows, cols, re: new Float64Array(rows * cols), im: new Float64Array(rows * cols) };
}

export function cfromReal(rows: number, cols: number, data: ArrayLike<number>): CMat {
  const M = cmat(rows, cols);
  M.re.set(data as Float64Array);
  return M;
}

export const cnorm = (re: Float64Array, im: Float64Array): number => {
  let s = 0;
  for (let i = 0; i < re.length; i++) s += re[i] * re[i] + im[i] * im[i];
  return Math.sqrt(s);
};

export const cabsmax = (re: Float64Array, im: Float64Array): number => {
  let m = 0;
  for (let i = 0; i < re.length; i++) m = Math.max(m, Math.hypot(re[i], im[i]));
  return m;
};

/** C = A * B with A complex and B real (row-major). */
export function cmulReal(A: CMat, Bre: Float64Array, bRows: number, bCols: number): CMat {
  const C = cmat(A.rows, bCols);
  for (let i = 0; i < A.rows; i++) {
    for (let k = 0; k < bRows; k++) {
      const ar = A.re[i * A.cols + k], ai = A.im[i * A.cols + k];
      if (ar === 0 && ai === 0) continue;
      for (let j = 0; j < bCols; j++) {
        const b = Bre[k * bCols + j];
        C.re[i * bCols + j] += ar * b;
        C.im[i * bCols + j] += ai * b;
      }
    }
  }
  return C;
}

/** y = A * x with A complex and x complex. */
export function cmatvec(A: CMat, xr: Float64Array, xi: Float64Array): [Float64Array, Float64Array] {
  const yr = new Float64Array(A.rows), yi = new Float64Array(A.rows);
  for (let i = 0; i < A.rows; i++) {
    let sr = 0, si = 0;
    for (let j = 0; j < A.cols; j++) {
      const ar = A.re[i * A.cols + j], ai = A.im[i * A.cols + j];
      sr += ar * xr[j] - ai * xi[j];
      si += ar * xi[j] + ai * xr[j];
    }
    yr[i] = sr; yi[i] = si;
  }
  return [yr, yi];
}

/** Solve the square complex system A X = B in place (partial-pivoting LU).  Returns false
 *  if A is numerically singular. */
export function csolve(n: number, A: CMat, B: CMat): boolean {
  const nrhs = B.cols;
  for (let k = 0; k < n; k++) {
    let p = k, best = -1;
    for (let i = k; i < n; i++) {
      const m = Math.hypot(A.re[i * n + k], A.im[i * n + k]);
      if (m > best) { best = m; p = i; }
    }
    if (best <= 0) return false;
    if (p !== k) {
      for (let j = 0; j < n; j++) {
        let t = A.re[k * n + j]; A.re[k * n + j] = A.re[p * n + j]; A.re[p * n + j] = t;
        t = A.im[k * n + j]; A.im[k * n + j] = A.im[p * n + j]; A.im[p * n + j] = t;
      }
      for (let j = 0; j < nrhs; j++) {
        let t = B.re[k * nrhs + j]; B.re[k * nrhs + j] = B.re[p * nrhs + j]; B.re[p * nrhs + j] = t;
        t = B.im[k * nrhs + j]; B.im[k * nrhs + j] = B.im[p * nrhs + j]; B.im[p * nrhs + j] = t;
      }
    }
    const pr = A.re[k * n + k], pi = A.im[k * n + k];
    const den = pr * pr + pi * pi;
    for (let i = k + 1; i < n; i++) {
      const ar = A.re[i * n + k], ai = A.im[i * n + k];
      if (ar === 0 && ai === 0) continue;
      const fr = (ar * pr + ai * pi) / den;
      const fi = (ai * pr - ar * pi) / den;
      A.re[i * n + k] = 0; A.im[i * n + k] = 0;
      for (let j = k + 1; j < n; j++) {
        const br = A.re[k * n + j], bi = A.im[k * n + j];
        A.re[i * n + j] -= fr * br - fi * bi;
        A.im[i * n + j] -= fr * bi + fi * br;
      }
      for (let j = 0; j < nrhs; j++) {
        const br = B.re[k * nrhs + j], bi = B.im[k * nrhs + j];
        B.re[i * nrhs + j] -= fr * br - fi * bi;
        B.im[i * nrhs + j] -= fr * bi + fi * br;
      }
    }
  }
  for (let i = n - 1; i >= 0; i--) {
    const pr = A.re[i * n + i], pi = A.im[i * n + i];
    const den = pr * pr + pi * pi;
    for (let j = 0; j < nrhs; j++) {
      let sr = B.re[i * nrhs + j], si = B.im[i * nrhs + j];
      for (let k = i + 1; k < n; k++) {
        const ar = A.re[i * n + k], ai = A.im[i * n + k];
        const xr = B.re[k * nrhs + j], xi = B.im[k * nrhs + j];
        sr -= ar * xr - ai * xi;
        si -= ar * xi + ai * xr;
      }
      B.re[i * nrhs + j] = (sr * pr + si * pi) / den;
      B.im[i * nrhs + j] = (si * pr - sr * pi) / den;
    }
  }
  return true;
}

/** Column indices that are free with respect to A's row space: Gaussian elimination with
 *  partial pivoting records the pivot columns, and everything else is free.  Fixing the free
 *  variables makes the (full row rank) system A w = b square and uniquely solvable — which is
 *  exactly what the homotopy's start system needs. */
export function freeColumns(A: CMat, tol = 1e-9): { pivots: number[]; free: number[] } {
  const { rows: m, cols: n } = A;
  const re = Float64Array.from(A.re), im = Float64Array.from(A.im);
  const pivots: number[] = [];
  let r = 0;
  let scale = 0;
  for (let i = 0; i < re.length; i++) scale = Math.max(scale, Math.hypot(re[i], im[i]));
  const lim = tol * (scale || 1);
  for (let c = 0; c < n && r < m; c++) {
    let p = r, best = -1;
    for (let i = r; i < m; i++) {
      const v = Math.hypot(re[i * n + c], im[i * n + c]);
      if (v > best) { best = v; p = i; }
    }
    if (best <= lim) continue;
    if (p !== r) {
      for (let j = 0; j < n; j++) {
        let t = re[r * n + j]; re[r * n + j] = re[p * n + j]; re[p * n + j] = t;
        t = im[r * n + j]; im[r * n + j] = im[p * n + j]; im[p * n + j] = t;
      }
    }
    const pr = re[r * n + c], pi = im[r * n + c];
    const den = pr * pr + pi * pi;
    for (let i = r + 1; i < m; i++) {
      const ar = re[i * n + c], ai = im[i * n + c];
      if (ar === 0 && ai === 0) continue;
      const fr = (ar * pr + ai * pi) / den;
      const fi = (ai * pr - ar * pi) / den;
      for (let j = c; j < n; j++) {
        const br = re[r * n + j], bi = im[r * n + j];
        re[i * n + j] -= fr * br - fi * bi;
        im[i * n + j] -= fr * bi + fi * br;
      }
    }
    pivots.push(c);
    r++;
  }
  const ps = new Set(pivots);
  const free: number[] = [];
  for (let c = 0; c < n; c++) if (!ps.has(c)) free.push(c);
  return { pivots, free };
}
