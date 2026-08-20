//! Just enough complex linear algebra for the homotopy continuation of Stage 5: matrices as split
//! real/imaginary buffers, an LU solve, and a reduced-row-echelon pass used to pick a set of
//! variables that is free with respect to the linear part of a merge system.

#[derive(Clone, Debug)]
pub struct CMat {
    pub rows: usize,
    pub cols: usize,
    pub re: Vec<f64>,
    pub im: Vec<f64>,
}

impl CMat {
    pub fn zeros(rows: usize, cols: usize) -> CMat {
        CMat { rows, cols, re: vec![0.0; rows * cols], im: vec![0.0; rows * cols] }
    }
}

pub fn cnorm(re: &[f64], im: &[f64]) -> f64 {
    let mut s = 0.0;
    for i in 0..re.len() {
        s += re[i] * re[i] + im[i] * im[i];
    }
    s.sqrt()
}

pub fn cmul(ar: f64, ai: f64, br: f64, bi: f64) -> (f64, f64) {
    (ar * br - ai * bi, ar * bi + ai * br)
}

/// C = A * B with A complex and B real (row-major).
pub fn cmul_real(a: &CMat, b_re: &[f64], b_rows: usize, b_cols: usize) -> CMat {
    let mut c = CMat::zeros(a.rows, b_cols);
    for i in 0..a.rows {
        for k in 0..b_rows {
            let (ar, ai) = (a.re[i * a.cols + k], a.im[i * a.cols + k]);
            if ar == 0.0 && ai == 0.0 {
                continue;
            }
            for j in 0..b_cols {
                let b = b_re[k * b_cols + j];
                c.re[i * b_cols + j] += ar * b;
                c.im[i * b_cols + j] += ai * b;
            }
        }
    }
    c
}

/// y = A x with A complex and x complex.
pub fn cmatvec(a: &CMat, xr: &[f64], xi: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let mut yr = vec![0.0; a.rows];
    let mut yi = vec![0.0; a.rows];
    for i in 0..a.rows {
        let (mut sr, mut si) = (0.0, 0.0);
        for j in 0..a.cols {
            let (ar, ai) = (a.re[i * a.cols + j], a.im[i * a.cols + j]);
            sr += ar * xr[j] - ai * xi[j];
            si += ar * xi[j] + ai * xr[j];
        }
        yr[i] = sr;
        yi[i] = si;
    }
    (yr, yi)
}

/// Solve the square complex system `A X = B` in place (partial-pivoting LU).  `false` if A is
/// numerically singular.
pub fn csolve(n: usize, a: &mut CMat, b: &mut CMat) -> bool {
    let nrhs = b.cols;
    for k in 0..n {
        let mut p = k;
        let mut best = -1.0f64;
        for i in k..n {
            let m = a.re[i * n + k].hypot(a.im[i * n + k]);
            if m > best {
                best = m;
                p = i;
            }
        }
        if best <= 0.0 {
            return false;
        }
        if p != k {
            for j in 0..n {
                a.re.swap(k * n + j, p * n + j);
                a.im.swap(k * n + j, p * n + j);
            }
            for j in 0..nrhs {
                b.re.swap(k * nrhs + j, p * nrhs + j);
                b.im.swap(k * nrhs + j, p * nrhs + j);
            }
        }
        let (pr, pi) = (a.re[k * n + k], a.im[k * n + k]);
        let den = pr * pr + pi * pi;
        for i in k + 1..n {
            let (ar, ai) = (a.re[i * n + k], a.im[i * n + k]);
            if ar == 0.0 && ai == 0.0 {
                continue;
            }
            let fr = (ar * pr + ai * pi) / den;
            let fi = (ai * pr - ar * pi) / den;
            a.re[i * n + k] = 0.0;
            a.im[i * n + k] = 0.0;
            for j in k + 1..n {
                let (br, bi) = (a.re[k * n + j], a.im[k * n + j]);
                a.re[i * n + j] -= fr * br - fi * bi;
                a.im[i * n + j] -= fr * bi + fi * br;
            }
            for j in 0..nrhs {
                let (br, bi) = (b.re[k * nrhs + j], b.im[k * nrhs + j]);
                b.re[i * nrhs + j] -= fr * br - fi * bi;
                b.im[i * nrhs + j] -= fr * bi + fi * br;
            }
        }
    }
    for i in (0..n).rev() {
        let (pr, pi) = (a.re[i * n + i], a.im[i * n + i]);
        let den = pr * pr + pi * pi;
        for j in 0..nrhs {
            let (mut sr, mut si) = (b.re[i * nrhs + j], b.im[i * nrhs + j]);
            for k in i + 1..n {
                let (ar, ai) = (a.re[i * n + k], a.im[i * n + k]);
                let (xr, xi) = (b.re[k * nrhs + j], b.im[k * nrhs + j]);
                sr -= ar * xr - ai * xi;
                si -= ar * xi + ai * xr;
            }
            b.re[i * nrhs + j] = (sr * pr + si * pi) / den;
            b.im[i * nrhs + j] = (si * pr - sr * pi) / den;
        }
    }
    true
}

/// Column indices that are free with respect to A's row space: Gaussian elimination with partial
/// pivoting records the pivot columns, and everything else is free.  Fixing the free variables
/// makes the (full row rank) system `A w = b` square and uniquely solvable — which is exactly what
/// the homotopy's start system needs.
pub fn free_columns(a: &CMat, tol: f64) -> (Vec<usize>, Vec<usize>) {
    let (m, n) = (a.rows, a.cols);
    let mut re = a.re.clone();
    let mut im = a.im.clone();
    let mut pivots: Vec<usize> = Vec::new();
    let mut r = 0usize;
    let mut scale = 0.0f64;
    for i in 0..re.len() {
        scale = scale.max(re[i].hypot(im[i]));
    }
    let lim = tol * if scale == 0.0 { 1.0 } else { scale };
    let mut c = 0usize;
    while c < n && r < m {
        let mut p = r;
        let mut best = -1.0f64;
        for i in r..m {
            let v = re[i * n + c].hypot(im[i * n + c]);
            if v > best {
                best = v;
                p = i;
            }
        }
        if best <= lim {
            c += 1;
            continue;
        }
        if p != r {
            for j in 0..n {
                re.swap(r * n + j, p * n + j);
                im.swap(r * n + j, p * n + j);
            }
        }
        let (pr, pi) = (re[r * n + c], im[r * n + c]);
        let den = pr * pr + pi * pi;
        for i in r + 1..m {
            let (ar, ai) = (re[i * n + c], im[i * n + c]);
            if ar == 0.0 && ai == 0.0 {
                continue;
            }
            let fr = (ar * pr + ai * pi) / den;
            let fi = (ai * pr - ar * pi) / den;
            for j in c..n {
                let (br, bi) = (re[r * n + j], im[r * n + j]);
                re[i * n + j] -= fr * br - fi * bi;
                im[i * n + j] -= fr * bi + fi * br;
            }
        }
        pivots.push(c);
        r += 1;
        c += 1;
    }
    let free: Vec<usize> = (0..n).filter(|c| !pivots.contains(c)).collect();
    (pivots, free)
}
