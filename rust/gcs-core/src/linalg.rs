//! Dense linear algebra: rank-revealing QR, the complete orthogonal decomposition behind the
//! minimum-norm least-squares step, a Golub–Reinsch SVD and an LU solve.
//!
//! Everything is row-major.  The rank convention is the codebase's single one:
//! `|R_ii| > rcond * |R_00|` after a pivoted QR, and `sigma_i > rcond * sigma_0` after an SVD.
//! No LAPACK/BLAS: these routines are ours, and `tests/test_linalg.py` checks them against numpy.

/// Row-major dense matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct Mat {
    pub rows: usize,
    pub cols: usize,
    pub data: Vec<f64>,
}

impl Mat {
    pub fn zeros(rows: usize, cols: usize) -> Mat {
        Mat { rows, cols, data: vec![0.0; rows * cols] }
    }

    pub fn from_vec(rows: usize, cols: usize, data: Vec<f64>) -> Mat {
        debug_assert_eq!(data.len(), rows * cols);
        Mat { rows, cols, data }
    }

    pub fn identity(n: usize) -> Mat {
        let mut m = Mat::zeros(n, n);
        for i in 0..n {
            m.data[i * n + i] = 1.0;
        }
        m
    }

    #[inline]
    pub fn at(&self, i: usize, j: usize) -> f64 {
        self.data[i * self.cols + j]
    }

    #[inline]
    pub fn set(&mut self, i: usize, j: usize, v: f64) {
        self.data[i * self.cols + j] = v;
    }

    pub fn row(&self, i: usize) -> &[f64] {
        &self.data[i * self.cols..(i + 1) * self.cols]
    }

    pub fn transpose(&self) -> Mat {
        let mut t = Mat::zeros(self.cols, self.rows);
        for i in 0..self.rows {
            for j in 0..self.cols {
                t.data[j * self.rows + i] = self.data[i * self.cols + j];
            }
        }
        t
    }

    /// Rows `keep` of this matrix, in the given order.
    pub fn select_rows(&self, keep: &[usize]) -> Mat {
        let mut out = Mat::zeros(keep.len(), self.cols);
        for (i, &r) in keep.iter().enumerate() {
            out.data[i * self.cols..(i + 1) * self.cols]
                .copy_from_slice(&self.data[r * self.cols..(r + 1) * self.cols]);
        }
        out
    }

    /// y = A x
    pub fn mul_vec(&self, x: &[f64]) -> Vec<f64> {
        let mut y = vec![0.0; self.rows];
        for i in 0..self.rows {
            let mut s = 0.0;
            let row = self.row(i);
            for j in 0..self.cols {
                s += row[j] * x[j];
            }
            y[i] = s;
        }
        y
    }

    /// y = Aᵀ x
    pub fn mul_t_vec(&self, x: &[f64]) -> Vec<f64> {
        let mut y = vec![0.0; self.cols];
        for i in 0..self.rows {
            let xi = x[i];
            if xi == 0.0 {
                continue;
            }
            let row = self.row(i);
            for j in 0..self.cols {
                y[j] += row[j] * xi;
            }
        }
        y
    }
}

pub fn dot(a: &[f64], b: &[f64]) -> f64 {
    let mut s = 0.0;
    for i in 0..a.len() {
        s += a[i] * b[i];
    }
    s
}

pub fn norm(a: &[f64]) -> f64 {
    dot(a, a).sqrt()
}

/// max |a|, with NaN winning.  `x > m` would skip a NaN, and a residual vector with a NaN in it
/// would then pass the convergence test — a broken sketch reported as solved on iteration zero.
pub fn absmax(a: &[f64]) -> f64 {
    let mut m = 0.0f64;
    for &v in a {
        if v.is_nan() {
            return f64::NAN;
        }
        let x = v.abs();
        if x > m {
            m = x;
        }
    }
    m
}

fn dsign(a: f64, b: f64) -> f64 {
    if b >= 0.0 {
        a.abs()
    } else {
        -a.abs()
    }
}

/* -- Householder ---------------------------------------------------------- */

/// Reflector zeroing `x[1..len-1]` (stride `stride` through `a` from `off`): returns beta (the
/// new x[0]) and tau, leaving v[1..] in x[1..] (v[0] = 1 implicit).
fn house_gen(a: &mut [f64], off: usize, len: usize, stride: usize) -> (f64, f64) {
    let alpha = a[off];
    let mut xnorm = 0.0;
    for i in 1..len {
        let v = a[off + i * stride];
        xnorm += v * v;
    }
    if xnorm == 0.0 {
        return (alpha, 0.0);
    }
    xnorm = xnorm.sqrt();
    let beta = -dsign(alpha.hypot(xnorm), alpha);
    let tau = (beta - alpha) / beta;
    let s = 1.0 / (alpha - beta);
    for i in 1..len {
        a[off + i * stride] *= s;
    }
    (beta, tau)
}

/// Householder QR with column pivoting.  `a` (m*n row-major) is overwritten: R in the upper
/// triangle, the reflectors below it.  Returns (tau, pivots, rank).
fn qrp(m: usize, n: usize, a: &mut [f64], rcond: f64) -> (Vec<f64>, Vec<i32>, usize) {
    let k = m.min(n);
    let mut tau = vec![0.0; k];
    let mut piv: Vec<i32> = (0..n as i32).collect();
    let mut cn = vec![0.0; n];
    let mut cn0 = vec![0.0; n];
    for j in 0..n {
        let mut s = 0.0;
        for i in 0..m {
            let v = a[i * n + j];
            s += v * v;
        }
        cn[j] = s.sqrt();
        cn0[j] = cn[j];
    }
    for p in 0..k {
        let mut best = p;
        for j in p + 1..n {
            if cn[j] > cn[best] {
                best = j;
            }
        }
        if best != p {
            for i in 0..m {
                a.swap(i * n + p, i * n + best);
            }
            cn.swap(p, best);
            cn0.swap(p, best);
            piv.swap(p, best);
        }
        let (beta, t) = house_gen(a, p * n + p, m - p, n);
        tau[p] = t;
        if t != 0.0 {
            for j in p + 1..n {
                let mut w = a[p * n + j];
                for i in p + 1..m {
                    w += a[i * n + p] * a[i * n + j];
                }
                w *= t;
                a[p * n + j] -= w;
                for i in p + 1..m {
                    a[i * n + j] -= w * a[i * n + p];
                }
            }
        }
        a[p * n + p] = beta;
        // downdate the trailing column norms, recomputing when cancellation bites
        for j in p + 1..n {
            if cn[j] == 0.0 {
                continue;
            }
            let r = a[p * n + j] / cn[j];
            let mut f = 1.0 - r * r;
            if f < 0.0 {
                f = 0.0;
            }
            let g = cn[j] / if cn0[j] > 0.0 { cn0[j] } else { 1.0 };
            if f * g * g < 1e-8 {
                let mut s = 0.0;
                for i in p + 1..m {
                    let v = a[i * n + j];
                    s += v * v;
                }
                cn[j] = s.sqrt();
                cn0[j] = cn[j];
            } else {
                cn[j] *= f.sqrt();
            }
        }
    }
    let mut rank = 0;
    if k > 0 {
        let d0 = a[0].abs();
        if d0 > 0.0 {
            for i in 0..k {
                if a[i * n + i].abs() > rcond * d0 {
                    rank += 1;
                }
            }
        }
    }
    (tau, piv, rank)
}

/// B (m*nrhs) <- Qᵀ B, using the reflectors left in `a` by `qrp`.
fn apply_qt(m: usize, n: usize, k: usize, a: &[f64], tau: &[f64], b: &mut [f64], nrhs: usize) {
    for p in 0..k {
        let t = tau[p];
        if t == 0.0 {
            continue;
        }
        for j in 0..nrhs {
            let mut w = b[p * nrhs + j];
            for i in p + 1..m {
                w += a[i * n + p] * b[i * nrhs + j];
            }
            w *= t;
            b[p * nrhs + j] -= w;
            for i in p + 1..m {
                b[i * nrhs + j] -= w * a[i * n + p];
            }
        }
    }
}

/// RZ factorization of the k*n trapezoid in `a` (k <= n): [R11 R12] Z = [T11 0].
fn tzrz(k: usize, n: usize, a: &mut [f64]) -> Vec<f64> {
    let mut ztau = vec![0.0; k.max(1)];
    if n <= k {
        return ztau;
    }
    let nz = n - k;
    for i in (0..k).rev() {
        let alpha = a[i * n + i];
        let mut xnorm = 0.0;
        for j in 0..nz {
            let v = a[i * n + k + j];
            xnorm += v * v;
        }
        if xnorm == 0.0 {
            ztau[i] = 0.0;
            continue;
        }
        xnorm = xnorm.sqrt();
        let beta = -dsign(alpha.hypot(xnorm), alpha);
        let t = (beta - alpha) / beta;
        let s = 1.0 / (alpha - beta);
        for j in 0..nz {
            a[i * n + k + j] *= s;
        }
        ztau[i] = t;
        a[i * n + i] = beta;
        for r in 0..i {
            let mut w = a[r * n + i];
            for j in 0..nz {
                w += a[r * n + k + j] * a[i * n + k + j];
            }
            w *= t;
            a[r * n + i] -= w;
            for j in 0..nz {
                a[r * n + k + j] -= w * a[i * n + k + j];
            }
        }
    }
    ztau
}

/// y <- Zᵀ y.
fn apply_zt(k: usize, n: usize, a: &[f64], ztau: &[f64], y: &mut [f64]) {
    if n <= k {
        return;
    }
    let nz = n - k;
    for i in 0..k {
        let t = ztau[i];
        if t == 0.0 {
            continue;
        }
        let mut w = y[i];
        for j in 0..nz {
            w += a[i * n + k + j] * y[k + j];
        }
        w *= t;
        y[i] -= w;
        for j in 0..nz {
            y[k + j] -= w * a[i * n + k + j];
        }
    }
}

/// Rank-revealing QR: `(rank, column pivots)`.  The first `rank` pivots index a maximal
/// independent set of columns.
pub fn rrqr(a: &Mat, rcond: f64) -> (usize, Vec<i32>) {
    let (m, n) = (a.rows, a.cols);
    if m == 0 || n == 0 {
        return (0, Vec::new());
    }
    let mut w = a.data.clone();
    let (_, piv, rank) = qrp(m, n, &mut w, rcond);
    (rank, piv)
}

pub fn rank_rrqr(a: &Mat, rcond: f64) -> usize {
    rrqr(a, rcond).0
}

/// Minimum-norm least-squares solution of `A X = B` via a complete orthogonal decomposition
/// (rank-revealing QR + RZ) — LAPACK dgelsy's algorithm.  Returns `(X, rank)`.
pub fn min_norm_lstsq(a: &Mat, b: &Mat, rcond: f64) -> (Mat, usize) {
    let (m, n, nrhs) = (a.rows, a.cols, b.cols);
    let mut x = Mat::zeros(n, nrhs);
    if n == 0 || nrhs == 0 || m == 0 {
        return (x, 0);
    }
    let k = m.min(n);
    let mut aw = a.data.clone();
    let mut bw = b.data.clone();
    let (tau, piv, rank) = qrp(m, n, &mut aw, rcond);
    apply_qt(m, n, k, &aw, &tau, &mut bw, nrhs);
    if rank > 0 {
        let ztau = tzrz(rank, n, &mut aw);
        let mut y = vec![0.0; n];
        for c in 0..nrhs {
            for v in y.iter_mut() {
                *v = 0.0;
            }
            for i in (0..rank).rev() {
                let mut s = bw[i * nrhs + c];
                for j in i + 1..rank {
                    s -= aw[i * n + j] * y[j];
                }
                y[i] = s / aw[i * n + i];
            }
            apply_zt(rank, n, &aw, &ztau, &mut y);
            for i in 0..n {
                x.data[piv[i] as usize * nrhs + c] = y[i];
            }
        }
    }
    (x, rank)
}

/// Minimum-norm least-squares solution for a single right-hand side.
pub fn min_norm_solve(a: &Mat, b: &[f64], rcond: f64) -> (Vec<f64>, usize) {
    let bm = Mat::from_vec(a.rows, 1, b.to_vec());
    let (x, rank) = min_norm_lstsq(a, &bm, rcond);
    (x.data, rank)
}

/* -- SVD (Golub–Reinsch) ---------------------------------------------------- */

fn pythag(a: f64, b: f64) -> f64 {
    let (aa, ab) = (a.abs(), b.abs());
    if aa > ab {
        let t = ab / aa;
        aa * (1.0 + t * t).sqrt()
    } else if ab == 0.0 {
        0.0
    } else {
        let t = aa / ab;
        ab * (1.0 + t * t).sqrt()
    }
}

/// Householder bidiagonalization followed by an implicit-shift QR sweep on the bidiagonal — the
/// classic algorithm, chosen over one-sided Jacobi because diagnosis SVDs a Jacobian on every
/// edit and Jacobi's sweep count makes that quadratically too slow at sketch sizes.
///
/// `a` (m*n, m >= n) is overwritten with U when `want_u`; `w` receives the singular values in
/// bidiagonalization order and `v` (n*n) the right factor.
fn gr_svd(m: usize, n: usize, a: &mut [f64], w: &mut [f64], v: &mut [f64], want_u: bool) -> bool {
    let mut rv1 = vec![0.0; n];
    let (mut g, mut scale, mut anorm) = (0.0f64, 0.0f64, 0.0f64);
    let mut l = 0usize;

    for i in 0..n {
        l = i + 1;
        rv1[i] = scale * g;
        g = 0.0;
        scale = 0.0;
        let mut s = 0.0;
        if i < m {
            for k in i..m {
                scale += a[k * n + i].abs();
            }
            if scale != 0.0 {
                for k in i..m {
                    a[k * n + i] /= scale;
                    s += a[k * n + i] * a[k * n + i];
                }
                let f = a[i * n + i];
                g = -dsign(s.sqrt(), f);
                let h = f * g - s;
                a[i * n + i] = f - g;
                for j in l..n {
                    let mut ss = 0.0;
                    for k in i..m {
                        ss += a[k * n + i] * a[k * n + j];
                    }
                    let ff = ss / h;
                    for k in i..m {
                        a[k * n + j] += ff * a[k * n + i];
                    }
                }
                for k in i..m {
                    a[k * n + i] *= scale;
                }
            }
        }
        w[i] = scale * g;
        g = 0.0;
        scale = 0.0;
        s = 0.0;
        if i < m && i + 1 != n {
            for k in l..n {
                scale += a[i * n + k].abs();
            }
            if scale != 0.0 {
                for k in l..n {
                    a[i * n + k] /= scale;
                    s += a[i * n + k] * a[i * n + k];
                }
                let f = a[i * n + l];
                g = -dsign(s.sqrt(), f);
                let h = f * g - s;
                a[i * n + l] = f - g;
                for k in l..n {
                    rv1[k] = a[i * n + k] / h;
                }
                for j in l..m {
                    let mut ss = 0.0;
                    for k in l..n {
                        ss += a[j * n + k] * a[i * n + k];
                    }
                    for k in l..n {
                        a[j * n + k] += ss * rv1[k];
                    }
                }
                for k in l..n {
                    a[i * n + k] *= scale;
                }
            }
        }
        let am = w[i].abs() + rv1[i].abs();
        if am > anorm {
            anorm = am;
        }
    }
    // right-hand transformations
    for i in (0..n).rev() {
        if i + 1 < n {
            if g != 0.0 {
                for j in l..n {
                    v[j * n + i] = (a[i * n + j] / a[i * n + l]) / g;
                }
                for j in l..n {
                    let mut s = 0.0;
                    for k in l..n {
                        s += a[i * n + k] * v[k * n + j];
                    }
                    for k in l..n {
                        v[k * n + j] += s * v[k * n + i];
                    }
                }
            }
            for j in l..n {
                v[i * n + j] = 0.0;
                v[j * n + i] = 0.0;
            }
        }
        v[i * n + i] = 1.0;
        g = rv1[i];
        l = i;
    }
    // left-hand transformations
    if want_u {
        for i in (0..m.min(n)).rev() {
            l = i + 1;
            g = w[i];
            for j in l..n {
                a[i * n + j] = 0.0;
            }
            if g != 0.0 {
                g = 1.0 / g;
                for j in l..n {
                    let mut s = 0.0;
                    for k in l..m {
                        s += a[k * n + i] * a[k * n + j];
                    }
                    let f = (s / a[i * n + i]) * g;
                    for k in i..m {
                        a[k * n + j] += f * a[k * n + i];
                    }
                }
                for j in i..m {
                    a[j * n + i] *= g;
                }
            } else {
                for j in i..m {
                    a[j * n + i] = 0.0;
                }
            }
            a[i * n + i] += 1.0;
        }
    }
    // diagonalize the bidiagonal form
    for k in (0..n).rev() {
        for its in 0..60 {
            // Find the start `ll` of the unconverged block.  rv1[0] is always 0, so the first
            // test fires at ll = 0 and w[-1] is never consulted (as in the original).
            let mut flag = true;
            let mut ll = 0usize;
            let mut nm = 0usize;
            for li in (0..=k).rev() {
                ll = li;
                if ll == 0 || rv1[ll].abs() + anorm == anorm {
                    flag = false;
                    break;
                }
                nm = ll - 1;
                if w[nm].abs() + anorm == anorm {
                    break;
                }
            }
            if flag {
                // cancel rv1[l] with Givens rotations
                let (mut c, mut s) = (0.0f64, 1.0f64);
                for i in ll..=k {
                    let f = s * rv1[i];
                    rv1[i] *= c;
                    if f.abs() + anorm == anorm {
                        break;
                    }
                    g = w[i];
                    let mut h = pythag(f, g);
                    w[i] = h;
                    h = 1.0 / h;
                    c = g * h;
                    s = -f * h;
                    if want_u {
                        for j in 0..m {
                            let y = a[j * n + nm];
                            let z = a[j * n + i];
                            a[j * n + nm] = y * c + z * s;
                            a[j * n + i] = z * c - y * s;
                        }
                    }
                }
            }
            let mut z = w[k];
            if ll == k {
                if z < 0.0 {
                    w[k] = -z;
                    for j in 0..n {
                        v[j * n + k] = -v[j * n + k];
                    }
                }
                break;
            }
            if its == 59 {
                return false;
            }
            let mut x = w[ll];
            let nm2 = k - 1;
            let mut y = w[nm2];
            let mut h = rv1[k];
            g = rv1[nm2];
            let mut f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            g = pythag(f, 1.0);
            f = ((x - z) * (x + z) + h * ((y / (f + dsign(g, f))) - h)) / x;
            let (mut c, mut s) = (1.0f64, 1.0f64);
            for j in ll..=nm2 {
                let i = j + 1;
                g = rv1[i];
                y = w[i];
                h = s * g;
                g *= c;
                z = pythag(f, h);
                rv1[j] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y *= c;
                for jj in 0..n {
                    let xx = v[jj * n + j];
                    let zz = v[jj * n + i];
                    v[jj * n + j] = xx * c + zz * s;
                    v[jj * n + i] = zz * c - xx * s;
                }
                z = pythag(f, h);
                w[j] = z;
                if z != 0.0 {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }
                f = c * g + s * y;
                x = c * y - s * g;
                if want_u {
                    for jj in 0..m {
                        let yy = a[jj * n + j];
                        let zz = a[jj * n + i];
                        a[jj * n + j] = yy * c + zz * s;
                        a[jj * n + i] = zz * c - yy * s;
                    }
                }
            }
            rv1[ll] = 0.0;
            rv1[k] = f;
            w[k] = x;
        }
    }
    true
}

pub struct Svd {
    /// m x min(m, n); empty when `want_u` was false.
    pub u: Mat,
    /// min(m, n) singular values, descending.
    pub s: Vec<f64>,
    /// n x n right factor (rows are the right singular vectors).
    pub vt: Mat,
    /// False if the QR sweeps ran out before every superdiagonal split — the factors are not to
    /// be trusted, and a rank read off them is meaningless rather than merely approximate.
    pub converged: bool,
}

/// Singular values (descending) and the full right factor.  A wide matrix is padded with zero
/// rows: the algorithm wants m >= n, and the padding leaves the singular values alone while
/// still producing the full n*n right factor whose trailing rows are the null space.
pub fn svd(a: &Mat, want_u: bool) -> Svd {
    let (m, n) = (a.rows, a.cols);
    let mn = m.min(n);
    if m == 0 || n == 0 {
        return Svd { u: Mat::zeros(m, mn), s: vec![0.0; mn], vt: Mat::zeros(n, n), converged: true };
    }
    let mm = m.max(n);
    let mut w = vec![0.0; mm * n];
    for i in 0..m {
        w[i * n..(i + 1) * n].copy_from_slice(a.row(i));
    }
    let mut v = vec![0.0; n * n];
    let mut sv = vec![0.0; n];
    let converged = gr_svd(mm, n, &mut w, &mut sv, &mut v, want_u);

    let mut ord: Vec<usize> = (0..n).collect();
    for i in 0..n.saturating_sub(1) {
        let mut b = i;
        for j in i + 1..n {
            if sv[ord[j]] > sv[ord[b]] {
                b = j;
            }
        }
        ord.swap(i, b);
    }
    let mut s = vec![0.0; mn];
    for i in 0..mn {
        s[i] = sv[ord[i]];
    }
    let mut vt = Mat::zeros(n, n);
    for i in 0..n {
        for j in 0..n {
            vt.data[i * n + j] = v[j * n + ord[i]];
        }
    }
    let u = if want_u {
        let mut u = Mat::zeros(m, mn);
        for c in 0..mn {
            for i in 0..m {
                u.data[i * mn + c] = w[i * n + ord[c]];
            }
        }
        u
    } else {
        Mat::zeros(0, 0)
    };
    Svd { u, s, vt, converged }
}

pub struct RankNull {
    pub rank: usize,
    /// n x (n - rank) orthonormal basis of the null space.
    pub n: Mat,
    pub s: Vec<f64>,
    /// False if the SVD behind this did not converge; `rank` then says nothing.
    pub converged: bool,
}

/// `(rank, null-space basis, singular values)` from one SVD — the shared seam that keeps
/// diagnosis, witness analysis and decomposition agreeing on what "rank" means.
pub fn rank_and_nullspace(a: &Mat, rcond: f64) -> RankNull {
    let (m, n) = (a.rows, a.cols);
    if n == 0 {
        return RankNull { rank: 0, n: Mat::zeros(0, 0), s: Vec::new(), converged: true };
    }
    if m == 0 {
        return RankNull { rank: 0, n: Mat::identity(n), s: Vec::new(), converged: true };
    }
    let d = svd(a, false);
    let mn = m.min(n);
    let mut rank = 0;
    if mn > 0 && d.s[0] > 0.0 {
        for i in 0..mn {
            if d.s[i] > rcond * d.s[0] {
                rank += 1;
            }
        }
    }
    let nn = n - rank;
    let mut null = Mat::zeros(n, nn);
    for i in 0..n {
        for j in 0..nn {
            null.data[i * nn.max(1) + j] = d.vt.data[(rank + j) * n + i];
        }
    }
    RankNull { rank, n: null, s: d.s, converged: d.converged }
}

/// Solve the n*n system `A x = b` in place (partial-pivoting LU).  `false` if A is singular.
pub fn lu_solve(n: usize, a: &mut [f64], b: &mut [f64]) -> bool {
    for k in 0..n {
        let mut p = k;
        for i in k + 1..n {
            if a[i * n + k].abs() > a[p * n + k].abs() {
                p = i;
            }
        }
        if a[p * n + k] == 0.0 {
            return false;
        }
        if p != k {
            for j in 0..n {
                a.swap(k * n + j, p * n + j);
            }
            b.swap(k, p);
        }
        let piv = a[k * n + k];
        for i in k + 1..n {
            let f = a[i * n + k] / piv;
            if f == 0.0 {
                continue;
            }
            a[i * n + k] = 0.0;
            for j in k + 1..n {
                a[i * n + j] -= f * a[k * n + j];
            }
            b[i] -= f * b[k];
        }
    }
    for i in (0..n).rev() {
        let mut s = b[i];
        for j in i + 1..n {
            s -= a[i * n + j] * b[j];
        }
        b[i] = s / a[i * n + i];
    }
    true
}

/// Orthonormal basis of the column span of `cols` (modified Gram–Schmidt).
pub fn orthonormalize(cols: &[Vec<f64>], tol: f64) -> Vec<Vec<f64>> {
    let mut out: Vec<Vec<f64>> = Vec::new();
    for c in cols {
        let mut v = c.clone();
        for q in &out {
            let d = dot(q, &v);
            for i in 0..v.len() {
                v[i] -= d * q[i];
            }
        }
        let nv = norm(&v);
        if nv > tol {
            for x in v.iter_mut() {
                *x /= nv;
            }
            out.push(v);
        }
    }
    out
}
