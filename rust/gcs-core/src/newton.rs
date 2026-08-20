//! Powell's DogLeg (the default) and Levenberg–Marquardt, both minimising ½‖r(z)‖².
//!
//! The Gauss–Newton step is the *minimum-norm* least-squares solution of J p = −r, so
//! under-constrained sketches (the normal case while editing) move as little as possible —
//! least-change behaviour is what users expect from dragging.
//!
//! Dense path: the complete orthogonal decomposition in `linalg`, which also reports the rank.
//! Sparse path: the regularized normal equations (JᵀJ + εI) p = −g factored by LDLᵀ, which keeps
//! rank-deficient systems solvable.
//!
//! Reference: Nocedal & Wright ch. 4 & 10; PlaneGCS's DogLeg.

use crate::linalg::{absmax, dot, lu_solve, min_norm_lstsq, norm, Mat};
use crate::system::{System, DENSE_MAX};

const EPS_REL: f64 = 1e-12;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Method {
    DogLeg,
    Lm,
}

impl Method {
    pub fn parse(s: &str) -> Option<Method> {
        match s {
            "dogleg" => Some(Method::DogLeg),
            "lm" => Some(Method::Lm),
            _ => None,
        }
    }
    pub fn as_str(self) -> &'static str {
        match self {
            Method::DogLeg => "dogleg",
            Method::Lm => "lm",
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Info {
    /// 0 ftol, 1 xtol, 2 gtol, 3 trust region collapsed, 4 max iterations, -1 failed.
    pub status: i32,
    pub nfev: i32,
    pub njev: i32,
    pub iterations: i32,
    /// Numerical rank of J at the solution, or -1 (sparse path).
    pub rank: i32,
}

pub fn status_message(status: i32) -> &'static str {
    match status {
        0 => "residual tolerance reached",
        1 => "step size below xtol",
        2 => "gradient below gtol",
        3 => "trust region collapsed / damping exhausted",
        4 => "max iterations reached",
        -1 => "failed",
        _ => "unknown",
    }
}

/// The Jacobian, dense or sparse, behind one interface.
struct JacCtx {
    dense: bool,
    m: usize,
    n: usize,
    j: Mat,
    rank: i32,
}

impl JacCtx {
    fn eval(&mut self, sys: &mut System, z: &[f64]) {
        if self.dense {
            self.j = sys.jacobian_dense(z);
        } else {
            sys.compute_csr(z);
        }
    }

    fn jt_mul(&self, sys: &System, v: &[f64], out: &mut [f64]) {
        if self.dense {
            let r = self.j.mul_t_vec(v);
            out.copy_from_slice(&r);
        } else {
            sys.jt_mul_sparse(v, out);
        }
    }

    fn j_mul(&self, sys: &System, v: &[f64], out: &mut [f64]) {
        if self.dense {
            let r = self.j.mul_vec(v);
            out.copy_from_slice(&r);
        } else {
            sys.j_mul_sparse(v, out);
        }
    }

    /// p <- the Gauss–Newton step solving J p ≈ −r (minimum norm on the dense path).
    fn gn_step(&mut self, sys: &mut System, r: &[f64], g: &[f64], p: &mut [f64]) {
        if self.dense {
            let b = Mat::from_vec(self.m, 1, r.iter().map(|v| -v).collect());
            let (x, rank) = min_norm_lstsq(&self.j, &b, 1e-12);
            self.rank = rank as i32;
            p.copy_from_slice(&x.data);
            return;
        }
        let n = self.n;
        let mut work = vec![0.0; n];
        {
            let values: Vec<f64> = sys.csr_values().to_vec();
            let ata = sys.ata_mut();
            ata.fill(&values);
            ata.diag(&mut work);
        }
        let mut dmax = 0.0f64;
        for i in 0..n {
            if work[i] > dmax {
                dmax = work[i];
            }
        }
        let mut eps = EPS_REL * dmax;
        if eps <= 0.0 {
            eps = 1e-30;
        }
        for i in 0..n {
            work[i] = eps;
            p[i] = -g[i];
        }
        self.rank = -1;
        sys.ata_mut().solve(&work, p);
    }
}

/// What DogLeg needs of a system, so the trust-region loop is written once.
///
/// Two quite different things are minimised by it: the sketch's compiled `System` (sparse or
/// dense, thousands of rows) and the tiny rigid-motion systems a cluster merge produces (3k
/// unknowns, dense).  The loop below is the same for both — the only thing that differs is how a
/// Jacobian is applied and how a Gauss–Newton step is obtained.
pub trait TrustRegion {
    /// Unknowns.
    fn n(&self) -> usize;
    /// Residual rows.
    fn m(&self) -> usize;
    fn residuals_into(&mut self, z: &[f64], out: &mut [f64]);
    /// Prepare the Jacobian at `z` for the three operations below.
    fn jacobian_at(&mut self, z: &[f64]);
    /// out <- Jᵀ v
    fn jt_mul(&mut self, v: &[f64], out: &mut [f64]);
    /// out <- J v
    fn j_mul(&mut self, v: &[f64], out: &mut [f64]);
    /// p <- the Gauss–Newton step solving J p ≈ −r (minimum norm where the path reports a rank).
    fn gn_step(&mut self, r: &[f64], g: &[f64], p: &mut [f64]);
    /// Numerical rank of J, or -1 where the path does not produce one.
    fn rank(&self) -> i32 {
        -1
    }
}

/// The stopping tolerances a trust-region run is given.
#[derive(Clone, Copy, Debug)]
pub struct Tol {
    /// max |r| below which the run has converged.
    pub ftol: f64,
    /// step norm, relative to ‖z‖, below which no progress is being made.
    pub xtol: f64,
    /// max |Jᵀr| below which the point is stationary.
    pub gtol: f64,
}

/// Powell's DogLeg on any `TrustRegion`.  `z` and `r` are updated in place; `r` must hold the
/// residuals at `z` on entry.
pub fn dogleg<T: TrustRegion + ?Sized>(
    t: &mut T,
    z: &mut [f64],
    r: &mut [f64],
    tol: Tol,
    max_iter: i32,
    max_nfev: i32,
) -> Info {
    let (m, n) = (t.m(), t.n());
    let mut g = vec![0.0; n];
    let mut p = vec![0.0; n];
    let mut p_gn = vec![0.0; n];
    let mut p_sd = vec![0.0; n];
    let mut z_new = vec![0.0; n];
    let mut r_new = vec![0.0; m.max(1)];
    let mut tmp = vec![0.0; m.max(1)];
    let (mut nfev, mut njev) = (1i32, 0i32);
    let mut status = 4;
    let mut it = 0i32;
    let mut delta = f64::INFINITY;
    while it < max_iter {
        if absmax(&r[..m]) < tol.ftol {
            status = 0;
            break;
        }
        t.jacobian_at(z);
        njev += 1;
        let f = 0.5 * dot(&r[..m], &r[..m]);
        t.jt_mul(&r[..m], &mut g);
        if absmax(&g) < tol.gtol {
            status = 2;
            break;
        }
        t.gn_step(&r[..m], &g, &mut p_gn);
        let gn_norm = norm(&p_gn);
        if gn_norm <= delta {
            p.copy_from_slice(&p_gn);
        } else {
            t.j_mul(&g, &mut tmp);
            let jg = dot(&tmp[..m], &tmp[..m]);
            let alpha = if jg > 0.0 { dot(&g, &g) / jg } else { 0.0 };
            for i in 0..n {
                p_sd[i] = -alpha * g[i];
            }
            let sd_norm = norm(&p_sd);
            if sd_norm >= delta {
                let f2 = if sd_norm > 0.0 { delta / sd_norm } else { 0.0 };
                for i in 0..n {
                    p[i] = p_sd[i] * f2;
                }
            } else {
                let (mut aa, mut bb) = (0.0, 0.0);
                for i in 0..n {
                    let d = p_gn[i] - p_sd[i];
                    aa += d * d;
                    bb += 2.0 * p_sd[i] * d;
                }
                let cc = sd_norm * sd_norm - delta * delta;
                let disc = bb * bb - 4.0 * aa * cc;
                let tau =
                    if aa > 0.0 { (-bb + disc.max(0.0).sqrt()) / (2.0 * aa) } else { 0.0 };
                for i in 0..n {
                    p[i] = p_sd[i] + tau * (p_gn[i] - p_sd[i]);
                }
            }
        }
        let pnorm = norm(&p);
        if pnorm < tol.xtol * (1.0 + norm(z)) {
            status = 1;
            break;
        }
        for i in 0..n {
            z_new[i] = z[i] + p[i];
        }
        t.residuals_into(&z_new, &mut r_new[..m]);
        nfev += 1;
        let f_new = 0.5 * dot(&r_new[..m], &r_new[..m]);
        t.j_mul(&p, &mut tmp);
        let mut lin = 0.0;
        for i in 0..m {
            let v = r[i] + tmp[i];
            lin += v * v;
        }
        let pred = f - 0.5 * lin;
        let rho = if pred > 0.0 {
            (f - f_new) / pred
        } else if f_new < f {
            1.0
        } else {
            -1.0
        };
        if rho > 0.0 {
            z.copy_from_slice(&z_new);
            r[..m].copy_from_slice(&r_new[..m]);
            if rho > 0.75 {
                if delta.is_finite() {
                    delta = delta.max(3.0 * pnorm);
                }
            } else if rho < 0.25 {
                delta = 0.5 * pnorm;
            }
        } else {
            delta = 0.25 * pnorm;
        }
        if delta < 1e-15 * (1.0 + norm(z)) {
            status = 3;
            it += 1;
            break;
        }
        if nfev >= max_nfev {
            status = 4;
            it += 1;
            break;
        }
        it += 1;
    }
    Info { status, nfev, njev, iterations: it, rank: t.rank() }
}

/// The sketch's compiled `System` as a `TrustRegion`: the Jacobian is sparse or dense by size.
struct SysTr<'a> {
    sys: &'a mut System,
    c: JacCtx,
}

impl TrustRegion for SysTr<'_> {
    fn n(&self) -> usize {
        self.c.n
    }
    fn m(&self) -> usize {
        self.c.m
    }
    fn residuals_into(&mut self, z: &[f64], out: &mut [f64]) {
        self.sys.residuals_into(z, out);
    }
    fn jacobian_at(&mut self, z: &[f64]) {
        self.c.eval(self.sys, z);
    }
    fn jt_mul(&mut self, v: &[f64], out: &mut [f64]) {
        self.c.jt_mul(self.sys, v, out);
    }
    fn j_mul(&mut self, v: &[f64], out: &mut [f64]) {
        self.c.j_mul(self.sys, v, out);
    }
    fn gn_step(&mut self, r: &[f64], g: &[f64], p: &mut [f64]) {
        self.c.gn_step(self.sys, r, g, p);
    }
    fn rank(&self) -> i32 {
        self.c.rank
    }
}

fn lm_core(
    sys: &mut System,
    c: &mut JacCtx,
    z: &mut [f64],
    r: &mut [f64],
    ftol: f64,
    xtol: f64,
    gtol: f64,
    max_iter: i32,
    max_nfev: i32,
) -> Info {
    const TAU0: f64 = 1e-8;
    let (m, n) = (c.m, c.n);
    let mut g = vec![0.0; n];
    let mut p = vec![0.0; n];
    let mut d = vec![0.0; n];
    let mut damp = vec![0.0; n];
    let mut z_new = vec![0.0; n];
    let mut r_new = vec![0.0; m.max(1)];
    let mut a = if c.dense { vec![0.0; n * n] } else { Vec::new() };
    let mut ad = if c.dense { vec![0.0; n * n] } else { Vec::new() };
    let (mut nfev, mut njev) = (1i32, 0i32);
    let mut status = 4;
    let mut it = 0i32;
    let (mut lam, mut nu) = (-1.0f64, 2.0f64);
    'outer: while it < max_iter {
        if absmax(r) < ftol {
            status = 0;
            break;
        }
        c.eval(sys, z);
        njev += 1;
        c.jt_mul(sys, r, &mut g);
        if absmax(&g) < gtol {
            status = 2;
            break;
        }
        if c.dense {
            for i in 0..n {
                for j in 0..n {
                    let mut s = 0.0;
                    for kr in 0..m {
                        s += c.j.data[kr * n + i] * c.j.data[kr * n + j];
                    }
                    a[i * n + j] = s;
                }
            }
            for i in 0..n {
                d[i] = a[i * n + i];
            }
        } else {
            let values: Vec<f64> = sys.csr_values().to_vec();
            let ata = sys.ata_mut();
            ata.fill(&values);
            ata.diag(&mut d);
        }
        let mut dmax = 0.0f64;
        for i in 0..n {
            if d[i] > dmax {
                dmax = d[i];
            }
        }
        let floor = if dmax > 0.0 { 1e-8 * dmax } else { 1e-8 };
        for i in 0..n {
            if d[i] < floor {
                d[i] = floor;
            }
        }
        if lam < 0.0 {
            lam = TAU0 * if dmax > 0.0 { dmax } else { 1.0 };
        }
        let f = 0.5 * dot(r, r);
        loop {
            for i in 0..n {
                damp[i] = lam * d[i];
            }
            let bad;
            if c.dense {
                ad.copy_from_slice(&a);
                for i in 0..n {
                    ad[i * n + i] += damp[i];
                }
                for i in 0..n {
                    p[i] = -g[i];
                }
                bad = !lu_solve(n, &mut ad, &mut p);
            } else {
                for i in 0..n {
                    p[i] = -g[i];
                }
                bad = !sys.ata_mut().solve(&damp, &mut p);
            }
            if bad {
                lam *= nu;
                nu *= 2.0;
                if lam > 1e32 {
                    status = 3;
                    break 'outer;
                }
                continue;
            }
            let pnorm = norm(&p);
            if pnorm < xtol * (1.0 + norm(z)) {
                status = 1;
                break 'outer;
            }
            for i in 0..n {
                z_new[i] = z[i] + p[i];
            }
            sys.residuals_into(&z_new, &mut r_new[..m]);
            nfev += 1;
            let f_new = 0.5 * dot(&r_new[..m], &r_new[..m]);
            let mut pred = 0.0;
            for i in 0..n {
                pred += p[i] * (damp[i] * p[i] - g[i]);
            }
            pred *= 0.5;
            let rho = if pred > 0.0 { (f - f_new) / pred } else { -1.0 };
            if rho > 0.0 {
                z.copy_from_slice(&z_new);
                r[..m].copy_from_slice(&r_new[..m]);
                let t = 1.0 - (2.0 * rho - 1.0).powi(3);
                lam *= t.max(1.0 / 3.0);
                nu = 2.0;
                break;
            }
            lam *= nu;
            nu *= 2.0;
            if nfev >= max_nfev || lam > 1e32 {
                status = if nfev >= max_nfev { 4 } else { 3 };
                it += 1;
                break 'outer;
            }
        }
        it += 1;
    }
    Info { status, nfev, njev, iterations: it, rank: -1 }
}

/// Minimise ½‖r(z)‖² from `z` (updated in place).  `dense = None` picks by size.
pub fn solve_system(
    sys: &mut System,
    method: Method,
    ftol: f64,
    xtol: f64,
    gtol: f64,
    max_iter: i32,
    max_nfev: i32,
    dense: Option<bool>,
    z: &mut [f64],
) -> Info {
    if sys.n_free == 0 || sys.n_res == 0 {
        return Info { status: 0, nfev: 1, njev: 0, iterations: 0, rank: -1 };
    }
    let dense = dense.unwrap_or(sys.n_free <= DENSE_MAX);
    let max_nfev = if max_nfev <= 0 { 4 * max_iter } else { max_nfev };
    let mut ctx =
        JacCtx { dense, m: sys.n_res, n: sys.n_free, j: Mat::zeros(0, 0), rank: -1 };
    let mut r = sys.residuals(z);
    let info = if method == Method::Lm {
        lm_core(sys, &mut ctx, z, &mut r, ftol, xtol, gtol, max_iter, max_nfev)
    } else {
        let mut t = SysTr { sys, c: ctx };
        dogleg(&mut t, z, &mut r, Tol { ftol, xtol, gtol }, max_iter, max_nfev)
    };
    // leave the core's x in step with the returned z
    let _ = sys.residuals(z);
    info
}
