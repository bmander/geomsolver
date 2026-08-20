//! Stage 5 — homotopy continuation to enumerate the solutions of a small merge system (a
//! decomposition core or a closed-form triangle): "we can show you the other solutions".
//!
//! The merge system in the (c, s, tx, ty) parametrisation per movable cluster is polynomial:
//! shared points, line normals and direction rows are linear in the unknowns, line offsets are
//! bilinear, and c² + s² = 1 is quadratic.  We square the system with random complex combinations
//! (linear rows among themselves, degree-2 rows among themselves), keep the linear part fixed
//! along the path, and run a total-degree homotopy on the quadratic rows with the gamma trick:
//!
//! ```text
//! H(w, t) = (1 - t) * gamma * (w_sigma² - 1)  +  t * Qtilde(w)     (with Atilde w = btilde)
//! ```
//!
//! tracked from the 2^(n_Q) start points by Euler prediction and Newton correction in complex
//! arithmetic.  Real endpoints (polished on the original system) are the alternatives, sorted by
//! distance from the current solution.  Small cores only — the number of paths is exponential in
//! the number of rotations, which is exactly the cost decomposition minimises.

use crate::cgraph::El;
use crate::complex::{cmatvec, cmul, cmul_real, cnorm, csolve, free_columns, CMat};
use crate::decompose::{apply_t, execute, make_t, write_point, Cluster, Plan, Step};
use crate::linalg::{absmax, rank_rrqr, Mat};
use crate::model::Sketch;
use crate::rng::Rng;

#[derive(Clone, Debug)]
pub struct Alternative {
    /// Transform (theta, tx, ty) per movable cluster, relative to the current leaves.
    pub u: Vec<f64>,
    /// ‖w − w_identity‖: 0 for the root the sketch is on.
    pub distance: f64,
    /// Where a requested point element would land.
    pub location: Option<(f64, f64)>,
}

impl Alternative {
    pub fn is_current(&self) -> bool {
        self.distance < 1e-6
    }
}

/// Merge system F(w) = [A w − b ; Q(w)] in (c, s, tx, ty) per movable cluster: A holds the constant
/// degree-1 rows, Q the degree-2 rows (line offsets and c² + s² − 1).
struct Poly {
    k: usize,
    n: usize,
    a: Vec<f64>,
    rows_a: usize,
    b: Vec<f64>,
    m_q: usize,
    /// per-offset constants, hoisted out of the tracking loops: [(cluster, nx, ny, c); 2]
    off: Vec<[(usize, f64, f64, f64); 2]>,
}

impl Poly {
    fn new(parts: &[Cluster], step: &Step) -> Poly {
        let k = parts.len() - 1;
        let n = 4 * k;
        let mut rows: Vec<Vec<f64>> = Vec::new();
        let mut rhs: Vec<f64> = Vec::new();
        let mut offsets: Vec<(usize, usize, El)> = Vec::new();

        // Affine part of a pose (2 rows): coefficient matrix and constant vector.  Points
        // contribute both coordinates; lines contribute their normal (the offset coordinate is
        // bilinear and lives in Q).
        let lin_pose = |ci: usize, e: El| -> ([Vec<f64>; 2], (f64, f64)) {
            let p = &parts[ci].els[&e];
            let mut m = [vec![0.0; n], vec![0.0; n]];
            if ci == 0 {
                return (m, (p[0], p[1]));
            }
            let o = 4 * (ci - 1);
            let (a, b) = (p[0], p[1]);
            let is_p = e.is_point();
            m[0][o] = a;
            m[0][o + 1] = -b;
            m[0][o + 2] = if is_p { 1.0 } else { 0.0 };
            m[1][o] = b;
            m[1][o + 1] = a;
            m[1][o + 3] = if is_p { 1.0 } else { 0.0 };
            (m, (0.0, 0.0))
        };

        for &(i, j, e) in &step.pairs {
            let (ai, ci) = lin_pose(i, e);
            let (aj, cj) = lin_pose(j, e);
            rows.push((0..n).map(|t| ai[0][t] - aj[0][t]).collect());
            rows.push((0..n).map(|t| ai[1][t] - aj[1][t]).collect());
            rhs.push(cj.0 - ci.0);
            rhs.push(cj.1 - ci.1);
            if !e.is_point() {
                offsets.push((i, j, e));
            }
        }
        for &(i, j, la, lb, phi) in &step.dpairs {
            // n_b' − rot(phi) n_a' = 0
            let (aa, ca) = lin_pose(i, la);
            let (ab, cb) = lin_pose(j, lb);
            let (c, s) = (phi.cos(), phi.sin());
            rows.push((0..n).map(|t| ab[0][t] - (c * aa[0][t] - s * aa[1][t])).collect());
            rows.push((0..n).map(|t| ab[1][t] - (s * aa[0][t] + c * aa[1][t])).collect());
            rhs.push(c * ca.0 - s * ca.1 - cb.0);
            rhs.push(s * ca.0 + c * ca.1 - cb.1);
        }
        let rows_a = rows.len();
        let mut a = vec![0.0; rows_a * n];
        for (i, r) in rows.iter().enumerate() {
            a[i * n..(i + 1) * n].copy_from_slice(r);
        }
        let m_q = offsets.len() + k;
        let off = offsets
            .iter()
            .map(|&(i, j, e)| {
                let pi = &parts[i].els[&e];
                let pj = &parts[j].els[&e];
                [(i, pi[0], pi[1], pi[2]), (j, pj[0], pj[1], pj[2])]
            })
            .collect();
        Poly { k, n, a, rows_a, b: rhs, m_q, off }
    }

    /// Offset coordinate of a line pose under a cluster's transform; accumulates into `grad`.
    #[allow(clippy::too_many_arguments)]
    fn offset(
        &self,
        wr: &[f64],
        wi: &[f64],
        d: (usize, f64, f64, f64),
        gr: Option<&mut [f64]>,
        gi: Option<&mut [f64]>,
        sign: f64,
    ) -> (f64, f64) {
        let (ci, nx, ny, cc) = d;
        if ci == 0 {
            return (cc, 0.0);
        }
        let o = 4 * (ci - 1);
        let (cr, cim) = (wr[o], wi[o]);
        let (sr, si) = (wr[o + 1], wi[o + 1]);
        let (txr, txi) = (wr[o + 2], wi[o + 2]);
        let (tyr, tyi) = (wr[o + 3], wi[o + 3]);
        let n0r = cr * nx - sr * ny;
        let n0i = cim * nx - si * ny;
        let n1r = sr * nx + cr * ny;
        let n1i = si * nx + cim * ny;
        if let (Some(gr), Some(gi)) = (gr, gi) {
            gr[o] += sign * (nx * txr + ny * tyr);
            gi[o] += sign * (nx * txi + ny * tyi);
            gr[o + 1] += sign * (-ny * txr + nx * tyr);
            gi[o + 1] += sign * (-ny * txi + nx * tyi);
            gr[o + 2] += sign * n0r;
            gi[o + 2] += sign * n0i;
            gr[o + 3] += sign * n1r;
            gi[o + 3] += sign * n1i;
        }
        (
            cc + (n0r * txr - n0i * txi) + (n1r * tyr - n1i * tyi),
            (n0r * txi + n0i * txr) + (n1r * tyi + n1i * tyr),
        )
    }

    /// Quadratic rows and (optionally) their Jacobian — one pass, since the offset rows produce
    /// value and gradient together.
    fn qj(&self, wr: &[f64], wi: &[f64], want_jac: bool) -> (Vec<f64>, Vec<f64>, Option<CMat>) {
        let mut qr = vec![0.0; self.m_q];
        let mut qi = vec![0.0; self.m_q];
        let mut j = if want_jac { Some(CMat::zeros(self.m_q, self.n)) } else { None };
        for (r, pair) in self.off.iter().enumerate() {
            let (va, vb) = match &mut j {
                Some(jm) => {
                    let (gre, gim) = (&mut jm.re, &mut jm.im);
                    let mut grow = vec![0.0; self.n];
                    let mut giow = vec![0.0; self.n];
                    let va = self.offset(wr, wi, pair[0], Some(&mut grow), Some(&mut giow), 1.0);
                    let vb = self.offset(wr, wi, pair[1], Some(&mut grow), Some(&mut giow), -1.0);
                    gre[r * self.n..(r + 1) * self.n].copy_from_slice(&grow);
                    gim[r * self.n..(r + 1) * self.n].copy_from_slice(&giow);
                    (va, vb)
                }
                None => (
                    self.offset(wr, wi, pair[0], None, None, 1.0),
                    self.offset(wr, wi, pair[1], None, None, -1.0),
                ),
            };
            qr[r] = va.0 - vb.0;
            qi[r] = va.1 - vb.1;
        }
        let n_off = self.off.len();
        for q in 0..self.k {
            let (cr, ci) = (wr[4 * q], wi[4 * q]);
            let (sr, si) = (wr[4 * q + 1], wi[4 * q + 1]);
            qr[n_off + q] = cr * cr - ci * ci + sr * sr - si * si - 1.0;
            qi[n_off + q] = 2.0 * cr * ci + 2.0 * sr * si;
            if let Some(jm) = &mut j {
                jm.re[(n_off + q) * self.n + 4 * q] = 2.0 * cr;
                jm.im[(n_off + q) * self.n + 4 * q] = 2.0 * ci;
                jm.re[(n_off + q) * self.n + 4 * q + 1] = 2.0 * sr;
                jm.im[(n_off + q) * self.n + 4 * q + 1] = 2.0 * si;
            }
        }
        (qr, qi, j)
    }

    /// F(w) = [A w − b ; Q(w)].
    fn f(&self, wr: &[f64], wi: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let (qr, qi, _) = self.qj(wr, wi, false);
        let m = self.rows_a + self.m_q;
        let mut fr = vec![0.0; m];
        let mut fi = vec![0.0; m];
        for i in 0..self.rows_a {
            let mut sr = -self.b[i];
            let mut si = 0.0;
            for j in 0..self.n {
                sr += self.a[i * self.n + j] * wr[j];
                si += self.a[i * self.n + j] * wi[j];
            }
            fr[i] = sr;
            fi[i] = si;
        }
        fr[self.rows_a..].copy_from_slice(&qr);
        fi[self.rows_a..].copy_from_slice(&qi);
        (fr, fi)
    }

    fn jac(&self, wr: &[f64], wi: &[f64]) -> CMat {
        let jq = self.qj(wr, wi, true).2.unwrap();
        let mut out = CMat::zeros(self.rows_a + self.m_q, self.n);
        for i in 0..self.rows_a {
            for j in 0..self.n {
                out.re[i * self.n + j] = self.a[i * self.n + j];
            }
        }
        out.re[self.rows_a * self.n..].copy_from_slice(&jq.re);
        out.im[self.rows_a * self.n..].copy_from_slice(&jq.im);
        out
    }
}

fn w_to_u(wr: &[f64]) -> Vec<f64> {
    let k = wr.len() / 4;
    let mut u = vec![0.0; 3 * k];
    for q in 0..k {
        let (c, s) = (wr[4 * q], wr[4 * q + 1]);
        u[3 * q] = s.atan2(c);
        u[3 * q + 1] = wr[4 * q + 2];
        u[3 * q + 2] = wr[4 * q + 3];
    }
    u
}

#[derive(Clone, Copy, Debug)]
pub struct EnumerateOptions {
    pub locate: Option<El>,
    pub seed: u32,
    pub max_paths: usize,
    pub max_steps: usize,
    pub diverge_rel: f64,
}

impl Default for EnumerateOptions {
    fn default() -> EnumerateOptions {
        EnumerateOptions {
            locate: None,
            seed: 0,
            max_paths: 256,
            max_steps: 400,
            diverge_rel: 50.0,
        }
    }
}

/// Real solutions of the merge at `step_index` (the current one first).  Empty if the merge is not
/// isolated (under-determined) or too large.  `locate` asks where that point element would land.
pub fn enumerate_step(
    plan: &mut Plan,
    sk: &mut Sketch,
    step_index: usize,
    opts: EnumerateOptions,
) -> Vec<Alternative> {
    let mut rng = Rng::new(opts.seed);
    let Some(parts) = execute(plan, sk, Some(step_index)) else { return Vec::new() };
    if parts.len() < 2 {
        return Vec::new();
    }
    let step = &plan.steps[step_index];
    let p = Poly::new(&parts, step);
    let (n, k) = (p.n, p.k);
    let mut w_id = vec![0.0; n];
    for q in 0..k {
        w_id[4 * q] = 1.0; // the current solution: identity
    }

    // -- square the system: Atilde w = btilde (rank r) and n − r combinations of the quadratic rows
    let r = if p.rows_a > 0 {
        rank_rrqr(&Mat::from_vec(p.rows_a, n, p.a.clone()), 1e-9)
    } else {
        0
    };
    if n <= r {
        return Vec::new();
    }
    let n_q = n - r;
    if p.m_q < n_q || (1usize << n_q) > opts.max_paths {
        return Vec::new();
    }
    let c_rand = |rows: usize, cols: usize, rng: &mut Rng| -> CMat {
        let mut m = CMat::zeros(rows, cols);
        for i in 0..rows * cols {
            m.re[i] = rng.normal(0.0, 1.0);
            m.im[i] = rng.normal(0.0, 1.0);
        }
        m
    };
    let m1 = c_rand(r, p.rows_a, &mut rng);
    let m2 = c_rand(n_q, p.m_q, &mut rng);
    let at = cmul_real(&m1, &p.a, p.rows_a, n);
    let (bt_r, bt_i) = cmatvec(&m1, &p.b, &vec![0.0; p.rows_a]);

    // -- start system: the same linear rows plus w_sigma² − 1 on variables free w.r.t. them --
    let (_, free) = free_columns(&at, 1e-9);
    if free.len() < n_q {
        return Vec::new();
    }
    let sigma: Vec<usize> = free[..n_q].to_vec();
    let g_ang = 2.0 * std::f64::consts::PI * rng.next();
    let (gamma_r, gamma_i) = (g_ang.cos(), g_ang.sin());

    /// Row q of the random combination M2 applied to the quadratic rows.
    fn m2q(m2: &CMat, m_q: usize, q: usize, qr: &[f64], qi: &[f64]) -> (f64, f64) {
        let (mut mr, mut mi) = (0.0, 0.0);
        for j in 0..m_q {
            let (ar, ai) = (m2.re[q * m_q + j], m2.im[q * m_q + j]);
            mr += ar * qr[j] - ai * qi[j];
            mi += ar * qi[j] + ai * qr[j];
        }
        (mr, mi)
    }

    let start_row = |wr: &[f64], wi: &[f64], q: usize| -> (f64, f64) {
        let s = sigma[q];
        let (g2r, g2i) = cmul(wr[s], wi[s], wr[s], wi[s]);
        cmul(gamma_r, gamma_i, g2r - 1.0, g2i)
    };

    // H(w, t) and its Jacobian — Poly's offset rows give value and gradient in one pass.
    let hj = |wr: &[f64], wi: &[f64], t: f64| -> (Vec<f64>, Vec<f64>, CMat) {
        let (qr, qi, jq) = p.qj(wr, wi, true);
        let jq = jq.unwrap();
        let mut hr = vec![0.0; n];
        let mut hi = vec![0.0; n];
        let mut j = CMat::zeros(n, n);
        for i in 0..r {
            let mut sr = -bt_r[i];
            let mut si = -bt_i[i];
            for jj in 0..n {
                let (ar, ai) = (at.re[i * n + jj], at.im[i * n + jj]);
                sr += ar * wr[jj] - ai * wi[jj];
                si += ar * wi[jj] + ai * wr[jj];
                j.re[i * n + jj] = ar;
                j.im[i * n + jj] = ai;
            }
            hr[i] = sr;
            hi[i] = si;
        }
        for q in 0..n_q {
            // (1−t) * gamma * (w_s² − 1)  +  t * (M2 Q)
            let s = sigma[q];
            let (sr0, si0) = start_row(wr, wi, q);
            let (mr, mi) = m2q(&m2, p.m_q, q, &qr, &qi);
            hr[r + q] = (1.0 - t) * sr0 + t * mr;
            hi[r + q] = (1.0 - t) * si0 + t * mi;
            for jj in 0..n {
                let (mut jr, mut ji) = (0.0, 0.0);
                for pp in 0..p.m_q {
                    let (ar, ai) = (m2.re[q * p.m_q + pp], m2.im[q * p.m_q + pp]);
                    let (br, bi) = (jq.re[pp * n + jj], jq.im[pp * n + jj]);
                    jr += ar * br - ai * bi;
                    ji += ar * bi + ai * br;
                }
                j.re[(r + q) * n + jj] = t * jr;
                j.im[(r + q) * n + jj] = t * ji;
            }
            let (dr, di) = cmul(gamma_r, gamma_i, 2.0 * wr[s], 2.0 * wi[s]);
            j.re[(r + q) * n + s] += (1.0 - t) * dr;
            j.im[(r + q) * n + s] += (1.0 - t) * di;
        }
        (hr, hi, j)
    };

    // dH/dt: the quadratic rows swap the start system for the target one.
    let ht = |wr: &[f64], wi: &[f64]| -> (Vec<f64>, Vec<f64>) {
        let (qr, qi, _) = p.qj(wr, wi, false);
        let mut dr = vec![0.0; n];
        let mut di = vec![0.0; n];
        for q in 0..n_q {
            let (sr0, si0) = start_row(wr, wi, q);
            let (mr, mi) = m2q(&m2, p.m_q, q, &qr, &qi);
            dr[r + q] = -sr0 + mr;
            di[r + q] = -si0 + mi;
        }
        (dr, di)
    };

    // start points: every sign pattern on the sigma variables; one factorisation, all right-hand
    // sides
    let n_paths = 1usize << n_q;
    let mut s_mat = CMat::zeros(n, n);
    for i in 0..r {
        for j in 0..n {
            s_mat.re[i * n + j] = at.re[i * n + j];
            s_mat.im[i * n + j] = at.im[i * n + j];
        }
    }
    for q in 0..n_q {
        s_mat.re[(r + q) * n + sigma[q]] = 1.0;
    }
    let mut rhs = CMat::zeros(n, n_paths);
    for pp in 0..n_paths {
        for i in 0..r {
            rhs.re[i * n_paths + pp] = bt_r[i];
            rhs.im[i * n_paths + pp] = bt_i[i];
        }
        for q in 0..n_q {
            rhs.re[(r + q) * n_paths + pp] = if (pp >> q) & 1 == 1 { 1.0 } else { -1.0 };
        }
    }
    if !csolve(n, &mut s_mat, &mut rhs) {
        return Vec::new();
    }

    let newton = |wr: &mut Vec<f64>, wi: &mut Vec<f64>, t: f64| -> bool {
        for _ in 0..4 {
            let (hr, hi, mut j) = hj(wr, wi, t);
            if cnorm(&hr, &hi) < 1e-10 * (1.0 + cnorm(wr, wi)) {
                return true;
            }
            let mut b = CMat::zeros(n, 1);
            b.re.copy_from_slice(&hr);
            b.im.copy_from_slice(&hi);
            if !csolve(n, &mut j, &mut b) {
                return false;
            }
            for i in 0..n {
                wr[i] -= b.re[i];
                wi[i] -= b.im[i];
            }
        }
        let (hr, hi, _) = hj(wr, wi, t);
        cnorm(&hr, &hi) < 1e-6 * (1.0 + cnorm(wr, wi))
    };

    // Paths that run off to infinity are dead ends: cut them at a multiple of the sketch scale (w
    // holds cos/sin and translations, so an absolute bound would depend on the sketch size).
    let mut scale = 1.0f64;
    for v in &p.b {
        scale = scale.max(v.abs());
    }
    let diverge = opts.diverge_rel * scale;

    let mut ends: Vec<(Vec<f64>, Vec<f64>)> = Vec::new();
    for pp in 0..n_paths {
        let mut wr: Vec<f64> = (0..n).map(|i| rhs.re[i * n_paths + pp]).collect();
        let mut wi: Vec<f64> = (0..n).map(|i| rhs.im[i * n_paths + pp]).collect();
        let mut t = 0.0f64;
        let mut dt = 0.02f64;
        for _ in 0..opts.max_steps {
            if t >= 1.0 || cnorm(&wr, &wi) > diverge {
                break;
            }
            let t1 = 1.0f64.min(t + dt);
            let (_, _, mut j) = hj(&wr, &wi, t);
            let (dtr, dti) = ht(&wr, &wi);
            let mut b = CMat::zeros(n, 1);
            b.re.copy_from_slice(&dtr);
            b.im.copy_from_slice(&dti);
            if !csolve(n, &mut j, &mut b) {
                dt *= 0.5;
                if dt < 1e-10 {
                    break;
                }
                continue;
            }
            let mut nr = wr.clone();
            let mut ni = wi.clone();
            for i in 0..n {
                nr[i] -= b.re[i] * (t1 - t);
                ni[i] -= b.im[i] * (t1 - t);
            }
            let ok = newton(&mut nr, &mut ni, t1);
            let dr: Vec<f64> = (0..n).map(|i| nr[i] - wr[i]).collect();
            let di: Vec<f64> = (0..n).map(|i| ni[i] - wi[i]).collect();
            if ok && cnorm(&dr, &di) < 0.5 * (1.0 + cnorm(&wr, &wi)) {
                wr = nr;
                wi = ni;
                t = t1;
                dt = 0.2f64.min(dt * 1.5);
            } else {
                dt *= 0.5;
                if dt < 1e-10 {
                    break;
                }
            }
        }
        if t >= 1.0 && cnorm(&wr, &wi) <= diverge {
            for _ in 0..5 {
                // polish on the original system
                let (fr, fi) = p.f(&wr, &wi);
                if cnorm(&fr, &fi) < 1e-12 {
                    break;
                }
                let j = p.jac(&wr, &wi);
                // least squares via the normal equations JᴴJ x = Jᴴ f
                let m = j.rows;
                let mut a = CMat::zeros(n, n);
                let mut b = CMat::zeros(n, 1);
                for i in 0..n {
                    for jj in 0..n {
                        let (mut sr, mut si) = (0.0, 0.0);
                        for q in 0..m {
                            let (ar, ai) = (j.re[q * n + i], -j.im[q * n + i]);
                            let (br, bi) = (j.re[q * n + jj], j.im[q * n + jj]);
                            sr += ar * br - ai * bi;
                            si += ar * bi + ai * br;
                        }
                        a.re[i * n + jj] = sr;
                        a.im[i * n + jj] = si;
                    }
                    let (mut sr, mut si) = (0.0, 0.0);
                    for q in 0..m {
                        let (ar, ai) = (j.re[q * n + i], -j.im[q * n + i]);
                        sr += ar * fr[q] - ai * fi[q];
                        si += ar * fi[q] + ai * fr[q];
                    }
                    b.re[i] = sr;
                    b.im[i] = si;
                }
                if !csolve(n, &mut a, &mut b) {
                    break;
                }
                for i in 0..n {
                    wr[i] -= b.re[i];
                    wi[i] -= b.im[i];
                }
            }
            ends.push((wr, wi));
        }
    }

    let mut out: Vec<Alternative> = Vec::new();
    let mut kept: Vec<Vec<f64>> = Vec::new();
    let mut q_of: Option<usize> = None;
    if let Some(loc) = opts.locate {
        for i in 1..parts.len() {
            if parts[i].els.contains_key(&loc) {
                q_of = Some(i - 1);
                break;
            }
        }
    }
    for (wr, wi) in ends {
        if absmax(&wi) > 1e-6 * (1.0 + absmax(&wr)) {
            continue;
        }
        let zero = vec![0.0; n];
        let (fr, fi) = p.f(&wr, &zero);
        if cnorm(&fr, &fi) > 1e-6 {
            continue;
        }
        if kept.iter().any(|kv| {
            let mut s = 0.0;
            for i in 0..n {
                s += (wr[i] - kv[i]).powi(2);
            }
            s.sqrt() < 1e-6
        }) {
            continue;
        }
        kept.push(wr.clone());
        let u = w_to_u(&wr);
        let mut loc = None;
        if let (Some(l), Some(q)) = (opts.locate, q_of) {
            let t = make_t(u[3 * q], u[3 * q + 1], u[3 * q + 2]);
            let pos = apply_t(&t, l, &parts[q + 1].els[&l]);
            loc = Some((pos[0], pos[1]));
        }
        let mut d = 0.0; // the imaginary part is ~0 by now
        for i in 0..n {
            d += (wr[i] - w_id[i]).powi(2) + wi[i] * wi[i];
        }
        out.push(Alternative { u, distance: d.sqrt(), location: loc });
    }
    out.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
    out
}

/// Put the sketch on this root: write the alternative placement of the merged clusters into the
/// points (leaves are re-derived from geometry, so later replays stay on it), then replay the whole
/// plan so dependent geometry follows.  Triangles also flip their branch.
pub fn apply_alternative(plan: &mut Plan, sk: &mut Sketch, step_index: usize, alt: &Alternative) {
    let Some(parts) = execute(plan, sk, Some(step_index)) else { return };
    let is_ppp = plan.steps[step_index].ppp.is_some();
    if is_ppp && !alt.is_current() {
        if let Some(b) = plan.steps[step_index].branch {
            plan.steps[step_index].branch = Some(-b);
            for (k, v) in plan.branches() {
                sk.branches.insert(k, v); // document state
            }
        }
    }
    for (q, c) in parts[1..].iter().enumerate() {
        let t = make_t(alt.u[3 * q], alt.u[3 * q + 1], alt.u[3 * q + 2]);
        let writes: Vec<(El, Vec<f64>)> =
            c.els.iter().map(|(&e, pose)| (e, apply_t(&t, e, pose))).collect();
        for (e, pose) in writes {
            write_point(&plan.graph, sk, e, &pose);
        }
    }
    plan.sticky_branches = true;
    execute(plan, sk, None);
}
