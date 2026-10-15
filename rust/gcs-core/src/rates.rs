//! **Rates along a running formal** (#339 D2): a traced body that states a condition on a
//! derivative along the formal it runs over — `rate(p over s) parallel tl` — integrated from its
//! home rather than solved again at each place.
//!
//! A trace's body is solved at each `u` by itself (`locus.rs`, `traced.rs`).  A body that states
//! a **rate** cannot be: its instance at `u` depends on the road from the home.  So its body is
//! three sets of rows over one variable table `[outer] ++ q ++ w ++ q′` — `outer` the running
//! formal, the columns `θ` and the given numbers, `q` the body's own coordinates, `w` the numbers
//! it reads over the formal (a tape each), `q′` the rates of `q`:
//!
//! - its **algebraic** rows `r(q; u, θ) = 0`, which hold everywhere;
//! - its **rate** rows `R(q′, q; u, θ) = 0`, which read a rate and hold away from the home;
//! - its **initial** rows (`initially { … }`), which hold at the home alone and say there, for an
//!   instance written in place, what the rates leave free.  A drawn instance *is* its home: its
//!   pose on the sheet is the initial value, and its coordinates are columns of the curve.
//!
//! **The rates at a place** are one linear system: the algebraic rows differentiated along the
//! formal, `J_q q′ + ∂r/∂u = 0`, and the rate rows — square in a plane; in space, where a
//! `parallel` is two rows and the body's own rows may imply one, at least square and of full
//! rank, its rows past the count agreeing on the body's rows.  Its matrix `M = [J_q; ∂R/∂q′]` is
//! the index check, a rank question read at every evaluation; a place where it loses rank is
//! refused there (`ode::Refusal::Undefined`).  A rate row is solved by Newton from the rate before
//! it (the flow's branch), which takes one step for the rows the language admits — an incidence
//! or a component along a drawn direction, each linear in the rate.
//!
//! **The instance at `u` is the flow integrated from the home** (`ode::march`), with its
//! sensitivities to every column through the same substeps (the variational equations).  The
//! flow's Jacobian is exact: `∂q′/∂z = −M⁻¹ ∂G/∂z`, `G` the system above, the algebraic rows'
//! part a mixed second derivative read by **polarisation** over Taylor jets — `r` along `u + t,
//! q + t q′ ± t e_z` has `±2 ∂²r[ẋ, e_z]` between its two second orders — and the rate rows' a
//! first derivative read the same way.  Nothing is differenced.
//!
//! **The curve's higher orders** are the solution's Taylor coefficients, each one more solve with
//! `M`: along `q(t) = Σ q_k t^k` the algebraic rows' `t^k` coefficient is `J_q q_k + R_k` and the
//! rate rows' `t^{k−1}` is `k ∂R/∂q′ q_k + S_{k−1}`, `R_k`, `S_{k−1}` read with `q_k` at zero.
//!
//! **The home** is a drawn instance's pose — projected onto the body's algebraic rows, so a solve
//! from a rough start integrates no frame the body could not be (`Body::project`) — or, written in
//! place, the algebraic and initial rows solved from the seeds.
//!
//! What this certifies is the flow from the home: no branch is chosen anywhere else, so the
//! march needs no `continues` check.

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::kernels::KERNELS;
use crate::ode;
use crate::taylor::{self, Jet};

/// The most coordinates a body that states a rate integrates.
pub const MAX_Q: usize = 48;
/// The most columns one row reads.
const COLS: usize = 32;

/// Below this pivot, of the rate system with each row scaled to unit length, the rates have lost
/// rank and are refused there.
const RANK: f64 = 1e-10;
/// Newton on the rate rows from the rate before: one step for rows linear in the rate.
const RATE_NEWTON: usize = 8;
/// The home solve of an instance written in place.
const HOME_NEWTON: usize = 40;
/// Newton on a drawn pose's nearest point on its rows.
const PROJECT_NEWTON: usize = 40;
const RESTARTS: usize = 8;

/* -- the body ------------------------------------------------------------------------------- */

/// One row of a body: a static kernel, the variable-table slot each column reads, its constants.
#[derive(Clone, Debug)]
pub struct Row<'a> {
    pub kid: usize,
    pub cols: Vec<usize>,
    pub consts: &'a [f64],
}

/// A body that states a rate, as its owner's encoding reads it (`locus::View`, `traced::View`).
/// The variable table is `[outer] ++ q ++ w ++ q′`: `outer` is `[u, θ…, values…]` (`n_outer`),
/// `θ` its `n_theta` columns, the last `n_q` of them the drawn instance's pose where `pose`.
pub struct Body<'a> {
    pub n_outer: usize,
    pub n_theta: usize,
    pub n_q: usize,
    pub pose: bool,
    /// The traced point's first coordinate in `q`, and how many it has (2 in a plane, 3 in space).
    pub traced: usize,
    pub dim: usize,
    pub w: Vec<&'a [f64]>,
    pub seeds: Vec<&'a [f64]>,
    pub alg: Vec<Row<'a>>,
    pub rate: Vec<Row<'a>>,
    pub init: Vec<Row<'a>>,
}

/// Residual rows a set of rows makes.
pub fn n_res(rows: &[Row]) -> usize {
    rows.iter().map(|r| KERNELS[r.kid].n_res).sum()
}

/// Rows over plain numbers, into `out`.
pub(crate) fn residual(rows: &[Row], z: &[f64], out: &mut Vec<f64>) {
    out.clear();
    let mut x = [0.0f64; COLS];
    for r in rows {
        let kn = &KERNELS[r.kid];
        for (x, &c) in x.iter_mut().zip(&r.cols) {
            *x = z[c];
        }
        let at = out.len();
        out.resize(at + kn.n_res, 0.0);
        (kn.res)(1, &x[..r.cols.len()], r.consts, &mut out[at..]);
    }
}

/// Rows over jets, into `out`; `false` where a row has no form.
pub(crate) fn jets(rows: &[Row], z: &[Jet], out: &mut Vec<Jet>) -> bool {
    out.clear();
    let mut jrow = Vec::new();
    let mut v = [Jet::default(); COLS];
    for r in rows {
        let kn = &KERNELS[r.kid];
        for (v, &c) in v.iter_mut().zip(&r.cols) {
            *v = z[c];
        }
        let at = out.len();
        out.resize(at + kn.n_res, Jet::default());
        if !taylor::residual(r.kid, &v[..r.cols.len()], r.consts, &mut out[at..], &mut jrow) {
            return false;
        }
    }
    true
}

/// A matrix factorised with its rows scaled to unit length — by LU where it is square, by
/// Householder QR where it has more rows than unknowns, its rank the index check either way
/// (`factor`).
pub(crate) struct Factored {
    rows: usize,
    n: usize,
    /// LU (`piv` its swaps), or QR: `R` above the diagonal, the reflectors on and below it, `R`'s
    /// diagonal in `diag` and each reflector's `2/vᵀv` in `beta`.
    a: Vec<f64>,
    piv: Vec<usize>,
    diag: Vec<f64>,
    beta: Vec<f64>,
    scale: Vec<f64>,
}

/// The rate system at a place: the rates, and `M` factorised.
struct Solved {
    q1: Vec<f64>,
    m: Factored,
}

impl Factored {
    /// `M x = b` (`b` as long as `M` has rows), `x` into `b[..n]`: for more rows, the least
    /// squares solution, which is the solution of a system that holds.
    pub(crate) fn solve(&self, b: &mut [f64]) {
        let (r, n) = (self.rows, self.n);
        for (x, s) in b.iter_mut().zip(&self.scale) {
            *x *= s;
        }
        if r == n {
            crate::linalg::lu_apply(n, &self.a, &self.piv, b);
            return;
        }
        for k in 0..n {
            let mut dot = 0.0;
            for i in k..r {
                dot += self.a[i * n + k] * b[i];
            }
            let f = self.beta[k] * dot;
            for i in k..r {
                b[i] -= f * self.a[i * n + k];
            }
        }
        for k in (0..n).rev() {
            let mut v = b[k];
            for j in k + 1..n {
                v -= self.a[k * n + j] * b[j];
            }
            b[k] = v / self.diag[k];
        }
    }
}

impl<'a> Body<'a> {
    /// Where the rate slots start.
    fn r0(&self) -> usize {
        self.n_outer + self.n_q + self.w.len()
    }

    /// The columns that are no pose: the geometry the body is written over and the drawing's
    /// numbers, which the flow carries as constant states.
    pub fn n_free(&self) -> usize {
        self.n_theta - if self.pose { self.n_q } else { 0 }
    }

    /// Whether the rows describe a body: every column in the table, every row within the widths
    /// the evaluation's buffers hold, the algebraic and initial rows reading no rate, and the
    /// rows at least as many as the rates (and, written in place, as the coordinates at home).
    pub fn check(&self) -> Result<(), String> {
        let width = self.r0() + self.n_q;
        let n_q = self.n_q;
        if n_q == 0 || n_q > MAX_Q || self.traced + self.dim > n_q {
            return Err("a body that states a rate places between 1 and 48 numbers".into());
        }
        if self.n_theta + 1 > self.n_outer || (self.pose && self.n_q > self.n_theta) {
            return Err("rate body shape mismatch".into());
        }
        for (rows, reads) in [(&self.alg, false), (&self.init, false), (&self.rate, true)] {
            for r in rows.iter() {
                let kn = KERNELS.get(r.kid).ok_or("no such kernel")?;
                if kn.n_par != r.cols.len() || kn.n_const != r.consts.len() || kn.n_par > COLS
                    || kn.n_res > 8 || r.cols.iter().any(|&c| c >= width)
                {
                    return Err("rate row shape mismatch".into());
                }
                if !taylor::has_form(r.kid) {
                    return Err(format!("a body that states a rate reads its rows' derivatives \
                                        exactly, and `{}` has no Taylor form", kn.name));
                }
                if !reads && r.cols.iter().any(|&c| c >= self.r0()) {
                    return Err("only a rate row reads a rate".into());
                }
            }
        }
        if n_res(&self.alg) + n_res(&self.rate) < n_q {
            return Err(format!("a body that states a rate must determine its rates: {n_q} \
                unknowns against {} rows and {} rate rows", n_res(&self.alg), n_res(&self.rate)));
        }
        if !self.pose && n_res(&self.alg) + n_res(&self.init) < n_q {
            return Err(format!("an instance written in place starts where `initially {{ … }}` \
                says: {n_q} unknowns against {} rows and {} initial rows",
                n_res(&self.alg), n_res(&self.init)));
        }
        Ok(())
    }

    /// The variable table over plain numbers.
    fn table(&self, outer: &[f64], q: &[f64], q1: &[f64]) -> Option<Vec<f64>> {
        let mut z = Vec::with_capacity(self.r0() + self.n_q);
        z.extend_from_slice(&outer[..self.n_outer]);
        z.extend_from_slice(q);
        for t in &self.w {
            z.push(crate::tape::eval_num::<f64>(t, &outer[..self.n_outer])?);
        }
        z.extend_from_slice(q1);
        Some(z)
    }

    /// The variable table over jets: the tapes read along the outer jets.
    fn jet_table(&self, outer: &[Jet], q: &[Jet], q1: &[Jet]) -> Option<Vec<Jet>> {
        let mut z = Vec::with_capacity(self.r0() + self.n_q);
        z.extend_from_slice(outer);
        z.extend_from_slice(q);
        for t in &self.w {
            z.push(crate::tape::eval_num::<Jet>(t, outer)?);
        }
        z.extend_from_slice(q1);
        Some(z)
    }

    /// Rows' Jacobian in the `n` slots from `from`, row-major into `out` (`n_res(rows) × n`).
    fn jacobian(rows: &[Row], z: &[f64], from: usize, n: usize, out: &mut Vec<f64>) {
        out.clear();
        out.resize(n_res(rows) * n, 0.0);
        let mut row = 0;
        let mut x = [0.0f64; COLS];
        let mut jr = [0.0f64; COLS * 8];
        for r in rows {
            let kn = &KERNELS[r.kid];
            for (x, &c) in x.iter_mut().zip(&r.cols) {
                *x = z[c];
            }
            let jr = &mut jr[..kn.n_res * kn.n_par];
            jr.iter_mut().for_each(|v| *v = 0.0);
            kn.jac_into(&x[..kn.n_par], r.consts, jr);
            for t in 0..kn.n_res {
                for (c, &col) in r.cols.iter().enumerate() {
                    if (from..from + n).contains(&col) {
                        out[(row + t) * n + col - from] += jr[t * kn.n_par + c];
                    }
                }
            }
            row += kn.n_res;
        }
    }

    /// The outer variables as jets moving along `dir` (one entry an outer variable).
    fn outer_along(&self, outer: &[f64], dir: impl Fn(usize) -> f64) -> Vec<Jet> {
        (0..self.n_outer).map(|i| Jet::from(&[outer[i], dir(i)])).collect()
    }

    /// The outer variables as jets: `u + t` and, where `along` names one, a column moving with it
    /// at `c`.
    fn outer_jets(&self, outer: &[f64], along: Option<(usize, f64)>) -> Vec<Jet> {
        self.outer_along(outer, |i| match along {
            _ if i == 0 => 1.0,
            Some((j, c)) if j == i => c,
            _ => 0.0,
        })
    }

    /// `rows`' derivative along outer variable `i`, `q` standing: their first order, the tapes
    /// over it with them.
    fn along_column(&self, rows: &[Row], outer: &[f64], q: &[f64], i: usize) -> Option<Vec<f64>> {
        let oj = self.outer_along(outer, |k| if k == i { 1.0 } else { 0.0 });
        let qj: Vec<Jet> = q.iter().map(|&x| Jet::constant(x)).collect();
        let z = self.jet_table(&oj, &qj, &vec![Jet::constant(0.0); self.n_q])?;
        let mut r = Vec::new();
        jets(rows, &z, &mut r).then(|| r.iter().map(|j| j.0[1]).collect())
    }

    /// The rates at `(outer, q)`, by Newton on the rate rows from `start` — `None` where the
    /// system loses rank or the rows will not hold.
    ///
    /// Rows beyond the rates' count (in space, a `parallel`'s second row, which the body's own
    /// rows imply) agree where the body's rows hold; `strict` asks that they do, as they must
    /// at the home and at a reading, and between, where an integrator's substep stands off the
    /// rows by its own error, the rates are their least squares.
    fn rates(&self, outer: &[f64], q: &[f64], start: &[f64], strict: bool) -> Option<Solved> {
        let n = self.n_q;
        // the algebraic rows' first order along the formal with `q` standing: `∂r/∂u`, the
        // tapes' own rates in it
        let oj = self.outer_jets(outer, None);
        let qj: Vec<Jet> = q.iter().map(|&x| Jet::constant(x)).collect();
        let zero = vec![Jet::constant(0.0); n];
        let z = self.jet_table(&oj, &qj, &zero)?;
        let mut rj = Vec::new();
        if !jets(&self.alg, &z, &mut rj) {
            return None;
        }
        let b: Vec<f64> = rj.iter().map(|j| j.0[1]).collect();
        let mut z0: Vec<f64> = z.iter().map(|j| j.0[0]).collect();
        let n_alg = b.len();
        let mut jq = Vec::new();
        Self::jacobian(&self.alg, &z0, self.n_outer, n, &mut jq);
        let mut q1 = start[..n].to_vec();
        let (mut rr, mut jr) = (Vec::new(), Vec::new());
        let mut solved = None;
        for _ in 0..RATE_NEWTON {
            z0[self.r0()..].copy_from_slice(&q1);
            residual(&self.rate, &z0, &mut rr);
            Self::jacobian(&self.rate, &z0, self.r0(), n, &mut jr);
            let mut g = Vec::with_capacity(n);
            for i in 0..n_alg {
                g.push(-(b[i] + (0..n).map(|j| jq[i * n + j] * q1[j]).sum::<f64>()));
            }
            g.extend(rr.iter().map(|x| -x));
            let mut m = jq.clone();
            m.extend_from_slice(&jr);
            let m = factor(n_alg + rr.len(), n, m)?;
            m.solve(&mut g);
            let big = q1.iter().fold(1.0f64, |a, x| a.max(x.abs()));
            let step = g[..n].iter().fold(0.0f64, |a, x| a.max(x.abs()));
            for (x, d) in q1.iter_mut().zip(&g[..n]) {
                *x += d;
            }
            if !step.is_finite() {
                return None;
            }
            if step <= 1e-14 * big {
                // more rows than rates: what the least squares leaves over is how far they
                // disagree, which rows that hold leave at rounding
                let off = g[n..].iter().fold(0.0f64, |a, x| a.max(x.abs()));
                if strict && off > 1e-8 * big {
                    return None;
                }
                solved = Some(Solved { q1: q1.clone(), m });
                break;
            }
        }
        solved
    }

    /// `∂q′/∂z` for each state `z` — the body's coordinates, then its free columns — as columns
    /// of `out` (`n_q × (n_q + n_free)`, row-major), by polarisation (the module's docs).
    fn rate_jacobian(&self, outer: &[f64], q: &[f64], s: &Solved, out: &mut [f64]) -> bool {
        let (n, nf) = (self.n_q, self.n_free());
        let width = n + nf;
        let q1j: Vec<Jet> = s.q1.iter().map(|&x| Jet::constant(x)).collect();
        let qj: Vec<Jet> = (0..n).map(|i| Jet::from(&[q[i], s.q1[i]])).collect();
        let (mut alg, mut rate) = (Vec::new(), Vec::new());
        let mut pair = [Vec::new(), Vec::new()];
        let mut dg = Vec::with_capacity(n);
        // along a coordinate the outer variables and the tapes over them are the formal's alone,
        // read once; along a column, read again with it moving
        let Some(base) = self.jet_table(&self.outer_jets(outer, None), &qj, &q1j) else {
            return false;
        };
        let mut z = base.clone();
        for zc in 0..width {
            // a column nothing the flow reads moves no rate
            if zc >= n && !self.flow_reads(1 + zc - n) {
                for i in 0..n {
                    out[i * width + zc] = 0.0;
                }
                continue;
            }
            for (side, c) in [1.0, -1.0].into_iter().enumerate() {
                if zc < n {
                    z[self.n_outer + zc] = Jet::from(&[q[zc], s.q1[zc] + c]);
                } else {
                    let oj = self.outer_jets(outer, Some((1 + zc - n, c)));
                    let Some(t) = self.jet_table(&oj, &qj, &q1j) else { return false };
                    z = t;
                }
                if !jets(&self.alg, &z, &mut alg) || !jets(&self.rate, &z, &mut rate) {
                    return false;
                }
                pair[side].clear();
                pair[side].extend(alg.iter().map(|j| j.0[2]).chain(rate.iter().map(|j| j.0[1])));
            }
            if zc < n {
                z[self.n_outer + zc] = base[self.n_outer + zc];
            }
            dg.clear();
            dg.extend(pair[0].iter().zip(&pair[1]).map(|(a, b)| -(a - b) * 0.5));
            s.m.solve(&mut dg);
            for i in 0..n {
                out[i * width + zc] = dg[i];
            }
        }
        true
    }

    /// Whether the flow reads outer variable `c`: an algebraic or a rate row names it, or a
    /// tape does.
    fn flow_reads(&self, c: usize) -> bool {
        self.alg.iter().chain(&self.rate).any(|r| r.cols.contains(&c))
            || self.w.iter().any(|t| crate::tape::reads_var(t, c))
    }

    /// The Taylor coefficients `q_2`, `q_3` of the solution through `q` at `outer`, the rates
    /// `q_1` solved (`s`): each one more solve with `M`.
    fn higher(&self, outer: &[f64], q: &[f64], s: &Solved, upto: usize) -> Option<Vec<Vec<f64>>> {
        let n = self.n_q;
        let mut coef: Vec<Vec<f64>> = vec![q.to_vec(), s.q1.clone()];
        let oj = self.outer_jets(outer, None);
        let (mut alg, mut rate) = (Vec::new(), Vec::new());
        for k in 2..=upto {
            let qj: Vec<Jet> = (0..n).map(|i| {
                let c: Vec<f64> = coef.iter().map(|v| v[i]).collect();
                Jet::from(&c)
            }).collect();
            // q′(t) = Σ (j + 1) q_{j+1} t^j, the unknown `k q_k` at zero
            let q1j: Vec<Jet> = (0..n).map(|i| {
                let c: Vec<f64> = (1..k).map(|j| j as f64 * coef[j][i]).collect();
                Jet::from(&c)
            }).collect();
            let z = self.jet_table(&oj, &qj, &q1j)?;
            if !jets(&self.alg, &z, &mut alg) || !jets(&self.rate, &z, &mut rate) {
                return None;
            }
            let mut rhs: Vec<f64> = alg.iter().map(|j| -j.0[k])
                .chain(rate.iter().map(|j| -j.0[k - 1] / k as f64)).collect();
            s.m.solve(&mut rhs);
            rhs.truncate(n);
            coef.push(rhs);
        }
        Some(coef)
    }

    /// The home of an instance written in place: its algebraic and initial rows solved at
    /// `outer` from its seeds, and the solution's sensitivity to each column (`n_q × n_theta`).
    fn home(&self, outer: &[f64]) -> Option<(Vec<f64>, Vec<f64>)> {
        let n = self.n_q;
        let rows: Vec<Row> = self.alg.iter().chain(&self.init).cloned().collect();
        let at = &outer[..self.n_outer];
        let seed: Option<Vec<f64>> =
            self.seeds.iter().map(|t| crate::tape::eval_num::<f64>(t, at)).collect();
        let seed = seed?;
        let mut q = self.newton(&rows, outer, &seed);
        if q.is_none() {
            let scale = outer[1..1 + self.n_theta].iter().chain(&seed)
                .fold(1.0f64, |m, &x| m.max(x.abs()));
            let mut rng = crate::rng::Rng::new(0x7a7e);
            for _ in 0..RESTARTS {
                let start: Vec<f64> =
                    seed.iter().map(|&x| x + rng.uniform(-scale, scale)).collect();
                q = self.newton(&rows, outer, &start);
                if q.is_some() {
                    break;
                }
            }
        }
        let q = q?;
        // the implicit function theorem: J dq = −∂r/∂θ, a column a direction of θ
        let zero = vec![0.0; n];
        let z = self.table(outer, &q, &zero)?;
        let mut j = Vec::new();
        Self::jacobian(&rows, &z, self.n_outer, n, &mut j);
        let s = factor(n_res(&rows), n, j)?;
        let mut sens = vec![0.0; n * self.n_theta];
        for c in 0..self.n_theta {
            let mut d: Vec<f64> =
                self.along_column(&rows, outer, &q, 1 + c)?.iter().map(|x| -x).collect();
            s.solve(&mut d);
            for i in 0..n {
                sens[i * self.n_theta + c] = d[i];
            }
        }
        Some((q, sens))
    }

    /// Damped Newton on `rows` (square in `q`) at `outer`, from `q`.
    fn newton(&self, rows: &[Row], outer: &[f64], q: &[f64]) -> Option<Vec<f64>> {
        let n = self.n_q;
        let zero = vec![0.0; n];
        let mut z = self.table(outer, q, &zero)?;
        let size = z.iter().fold(1.0f64, |a, x| a.max(x.abs()));
        let tol = 1e-13 * size;
        let (mut r, mut j) = (Vec::new(), Vec::new());
        let norm = |r: &[f64]| r.iter().fold(0.0f64, |a, x| a.max(x.abs()));
        residual(rows, &z, &mut r);
        let mut now = norm(&r);
        for _ in 0..HOME_NEWTON {
            if !now.is_finite() {
                return None;
            }
            if now <= tol {
                return Some(z[self.n_outer..self.n_outer + n].to_vec());
            }
            Self::jacobian(rows, &z, self.n_outer, n, &mut j);
            let s = factor(r.len(), n, j.clone())?;
            let mut step: Vec<f64> = r.iter().map(|x| -x).collect();
            s.solve(&mut step);
            step.truncate(n);
            let mut lambda = 1.0;
            loop {
                let mut trial = z.clone();
                for (k, d) in step.iter().enumerate() {
                    trial[self.n_outer + k] += lambda * d;
                }
                residual(rows, &trial, &mut r);
                let next = norm(&r);
                if next.is_finite() && (next < now || lambda < 1e-4) {
                    z = trial;
                    now = next;
                    break;
                }
                lambda *= 0.5;
            }
        }
        (now <= tol * 1e3).then(|| z[self.n_outer..self.n_outer + n].to_vec())
    }

    /// **A drawn instance's pose, on its body's rows**: the nearest point `q₀` to `pose` where
    /// the algebraic rows hold — `q₀ − pose = Jᵀλ`, `r(q₀) = 0` — and its sensitivity to each
    /// column (`n_q × n_theta`).  At a solution of the drawing the pose is on its rows and this
    /// is the pose, its sensitivity the projection onto the rows' tangent space; on the way
    /// there, the curve is the flow of the consistent instance nearest the pose, so a solve from
    /// a rough start integrates no frame the body could not be.  Newton on the whole condition,
    /// `W = Σ λᵢ ∇²rᵢ` read by polarisation, so the sensitivity is exact off the rows too.
    fn project(&self, outer: &[f64], pose: &[f64]) -> Option<(Vec<f64>, Vec<f64>)> {
        let (n, nf, nt) = (self.n_q, self.n_free(), self.n_theta);
        let m = n_res(&self.alg);
        let k = n + m;
        let zero = vec![0.0; n];
        let size = pose.iter().chain(&outer[1..1 + nt]).fold(1.0f64, |a, x| a.max(x.abs()));
        let mut q = pose.to_vec();
        let mut lam = vec![0.0; m];
        let (mut r, mut j) = (Vec::new(), Vec::new());
        let mut kkt = None;
        for it in 0..PROJECT_NEWTON {
            let z = self.table(outer, &q, &zero)?;
            residual(&self.alg, &z, &mut r);
            Self::jacobian(&self.alg, &z, self.n_outer, n, &mut j);
            // F = [q − pose − Jᵀλ; r]
            let mut f = vec![0.0; k];
            for i in 0..n {
                f[i] = q[i] - pose[i] - (0..m).map(|a| j[a * n + i] * lam[a]).sum::<f64>();
            }
            f[n..].copy_from_slice(&r);
            let w = self.hessian(outer, &q, &lam, n)?;
            let mut mat = vec![0.0; k * k];
            for a in 0..n {
                for b in 0..n {
                    mat[a * k + b] = if a == b { 1.0 } else { 0.0 } - w[a * n + b];
                }
                for c in 0..m {
                    mat[a * k + n + c] = -j[c * n + a];
                    mat[(n + c) * k + a] = j[c * n + a];
                }
            }
            // judged at the size of what it reads, the pose's and the point's: at rounding there
            let size = q.iter().fold(size, |a, x| a.max(x.abs()));
            let small = f.iter().fold(0.0f64, |a, x| a.max(x.abs())) <= 1e-13 * size;
            let s = factor(k, k, mat)?;
            if small {
                kkt = Some(s);
                break;
            }
            let mut step: Vec<f64> = f.iter().map(|x| -x).collect();
            s.solve(&mut step);
            for i in 0..n {
                q[i] += step[i];
            }
            for c in 0..m {
                lam[c] += step[n + c];
            }
            if !step.iter().all(|x| x.is_finite()) || it + 1 == PROJECT_NEWTON {
                return None;
            }
        }
        let s = kkt?;
        // the sensitivities: K [dq; dλ] = [dpose + φ_qθ dθ; −r_θ dθ], a column of θ at a time
        let mut sens = vec![0.0; n * nt];
        let cross = self.cross(outer, &q, &lam)?;
        for c in 0..nt {
            let mut rhs = vec![0.0; k];
            if c >= nf {
                rhs[c - nf] = 1.0;
            } else {
                for i in 0..n {
                    rhs[i] = cross.0[i * nf + c];
                }
                for a in 0..m {
                    rhs[n + a] = -cross.1[a * nf + c];
                }
            }
            s.solve(&mut rhs);
            for i in 0..n {
                sens[i * nt + c] = rhs[i];
            }
        }
        Some((q, sens))
    }

    /// The second order of `λ·r` along a direction of `(q, θ free)` at `q`: `coef₂`, half of
    /// `D²(λ·r)[v, v]`.
    fn second(&self, outer: &[f64], q: &[f64], lam: &[f64], v: &[f64]) -> Option<f64> {
        let (n, nf) = (self.n_q, self.n_free());
        let oj = self.outer_along(outer, |i| match (1..1 + nf).contains(&i) {
            true => v[n + i - 1],
            false => 0.0,
        });
        let qj: Vec<Jet> = (0..n).map(|i| Jet::from(&[q[i], v[i]])).collect();
        let zj = vec![Jet::constant(0.0); n];
        let z = self.jet_table(&oj, &qj, &zj)?;
        let mut r = Vec::new();
        if !jets(&self.alg, &z, &mut r) {
            return None;
        }
        Some(r.iter().zip(lam).map(|(j, l)| j.0[2] * l).sum())
    }

    /// `D²(λ·r)[e_a, e_b]` for two directions of `(q, θ free)`, by polarisation.
    fn mixed(&self, outer: &[f64], q: &[f64], lam: &[f64], a: usize, b: usize) -> Option<f64> {
        let w = self.n_q + self.n_free();
        let mut v = vec![0.0; w];
        if a == b {
            v[a] = 1.0;
            return Some(2.0 * self.second(outer, q, lam, &v)?);
        }
        v[a] = 1.0;
        v[b] = 1.0;
        let plus = self.second(outer, q, lam, &v)?;
        v[b] = -1.0;
        let minus = self.second(outer, q, lam, &v)?;
        Some((plus - minus) * 0.5)
    }

    /// `W = ∇²_qq (λ·r)` (`n × n`), zero where `λ` is.
    fn hessian(&self, outer: &[f64], q: &[f64], lam: &[f64], n: usize) -> Option<Vec<f64>> {
        let mut w = vec![0.0; n * n];
        if lam.iter().all(|&l| l == 0.0) {
            return Some(w);
        }
        for a in 0..n {
            for b in a..n {
                let h = self.mixed(outer, q, lam, a, b)?;
                w[a * n + b] = h;
                w[b * n + a] = h;
            }
        }
        Some(w)
    }

    /// `∇_q ∂(λ·r)/∂θ` (`n × n_free`) and `∂r/∂θ` (`m × n_free`), over the free columns.
    fn cross(&self, outer: &[f64], q: &[f64], lam: &[f64]) -> Option<(Vec<f64>, Vec<f64>)> {
        let (n, nf) = (self.n_q, self.n_free());
        let m = n_res(&self.alg);
        let mut qt = vec![0.0; n * nf];
        let mut rt = vec![0.0; m * nf];
        for c in 0..nf {
            for (a, x) in self.along_column(&self.alg, outer, q, 1 + c)?.into_iter().enumerate() {
                rt[a * nf + c] = x;
            }
            if lam.iter().any(|&l| l != 0.0) {
                for i in 0..n {
                    qt[i * nf + c] = self.mixed(outer, q, lam, i, n + c)?;
                }
            }
        }
        Some((qt, rt))
    }

    /// Where the flow starts at `outer` (its anchor in `outer[0]`): the state `[q, θ free]` and
    /// its sensitivities to the columns, `Φ₀` (`(n_q + n_free) × n_theta`).
    fn start(&self, outer: &[f64]) -> Option<Vec<f64>> {
        let (n, nf, nt) = (self.n_q, self.n_free(), self.n_theta);
        let width = n + nf;
        let mut y = vec![0.0; width * (1 + nt)];
        let phi = width;
        let (q, sens) = match self.pose {
            true => self.project(outer, &outer[1 + nf..1 + nf + n])?,
            false => self.home(outer)?,
        };
        y[..n].copy_from_slice(&q);
        y[phi..phi + n * nt].copy_from_slice(&sens);
        for c in 0..nf {
            y[n + c] = outer[1 + c];
            y[phi + (n + c) * nt + c] = 1.0;
        }
        Some(y)
    }
}

/// `M` (`rows × n`, `rows ≥ n`) factorised with each row scaled to unit length — `None`
/// where it has lost rank: the index check.  A row that is all zero says nothing, and is left so.
pub(crate) fn factor(rows: usize, n: usize, mut m: Vec<f64>) -> Option<Factored> {
    if rows < n {
        return None;
    }
    let mut scale = vec![0.0; rows];
    for i in 0..rows {
        // each row's length, which is smooth in the state: where rows beyond the rates' count
        // disagree (off the body's rows, at a substep) the least squares weighs them by it, and
        // a weight with a kink would cost the extrapolation its order
        let big = m[i * n..(i + 1) * n].iter().map(|x| x * x).sum::<f64>().sqrt();
        if !big.is_finite() {
            return None;
        }
        if big > 0.0 {
            scale[i] = 1.0 / big;
            for x in &mut m[i * n..(i + 1) * n] {
                *x /= big;
            }
        }
    }
    let mut piv = Vec::new();
    let (mut diag, mut beta) = (Vec::new(), Vec::new());
    if rows == n {
        if !crate::linalg::lu_factor(n, &mut m, &mut piv) {
            return None;
        }
        if (0..n).any(|k| !(m[k * n + k].abs() > RANK)) {
            return None;
        }
    } else {
        for k in 0..n {
            let norm = (k..rows).map(|i| m[i * n + k] * m[i * n + k]).sum::<f64>().sqrt();
            if !(norm > RANK) {
                return None;
            }
            let alpha = if m[k * n + k] > 0.0 { -norm } else { norm };
            m[k * n + k] -= alpha;
            let vv: f64 = (k..rows).map(|i| m[i * n + k] * m[i * n + k]).sum();
            let b = if vv > 0.0 { 2.0 / vv } else { 0.0 };
            for j in k + 1..n {
                let dot: f64 = (k..rows).map(|i| m[i * n + k] * m[i * n + j]).sum();
                let f = b * dot;
                for i in k..rows {
                    m[i * n + j] -= f * m[i * n + k];
                }
            }
            diag.push(alpha);
            beta.push(b);
            if !(alpha.abs() > RANK) {
                return None;
            }
        }
    }
    Some(Factored { rows, n, a: m, piv, diag, beta, scale })
}

/* -- the flow ------------------------------------------------------------------------------- */

/// The rate a flow carries from one evaluation to the next: the rates it last found, where the
/// next Newton on the rate rows starts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Branch(pub [f64; MAX_Q]);

/// The body's flow at one set of columns: the state `[q, θ free]`, the free columns constant.
struct Flow<'b, 'a> {
    body: &'b Body<'a>,
    outer: Vec<f64>,
    branch: Branch,
}

impl ode::Rate for Flow<'_, '_> {
    type Branch = Branch;

    fn width(&self) -> usize {
        self.body.n_q + self.body.n_free()
    }

    fn rate(&mut self, u: f64, y: &[f64], f: &mut [f64], jac: Option<&mut [f64]>) -> bool {
        let (n, nf) = (self.body.n_q, self.body.n_free());
        self.outer[0] = u;
        self.outer[1..1 + nf].copy_from_slice(&y[n..n + nf]);
        let Some(s) = self.body.rates(&self.outer, &y[..n], &self.branch.0[..n], false) else {
            return false;
        };
        f[..n].copy_from_slice(&s.q1);
        f[n..n + nf].iter_mut().for_each(|x| *x = 0.0);
        self.branch.0[..n].copy_from_slice(&s.q1);
        if let Some(jac) = jac {
            jac.iter_mut().for_each(|x| *x = 0.0);
            // the free columns' rows are zero: they are constant states
            let w = n + nf;
            if !self.body.rate_jacobian(&self.outer, &y[..n], &s, &mut jac[..n * w]) {
                return false;
            }
        }
        f.iter().all(|x| x.is_finite())
    }

    fn branch(&self) -> Branch {
        self.branch
    }

    fn set_branch(&mut self, b: Branch) {
        self.branch = b;
    }
}

/// The body at one place: the traced point, its derivatives along the formal to the third
/// (`d[k]` is `dᵏC/duᵏ`, NaN past `orders`), its sensitivities to each column (`dc`) and, asked
/// for, those of its first and second derivatives along the formal (`dd1`, `dd2`).
#[derive(Clone, Debug)]
pub struct Reading {
    pub d: [[f64; 3]; 4],
    pub orders: u8,
    /// `dc[j][c]`: `∂C_c/∂θ_j`.
    pub dc: Vec<[f64; 3]>,
    pub dd1: Vec<[f64; 3]>,
    pub dd2: Vec<[f64; 3]>,
}

/// One march of the body from its home: the columns it was made at, the path, and — where the
/// march was refused short of where it was asked to run — the refusal, the path then reaching
/// the place it was refused at.
pub struct March {
    pub outer: Vec<f64>,
    pub home: f64,
    pub path: ode::Path<Branch>,
    pub refused: Option<ode::Refusal>,
}

impl March {
    /// Whether the march reaches `u`.
    pub fn reaches(&self, u: f64) -> bool {
        let last = self.path.nodes.last().map_or(self.home, |n| n.u);
        let (to, reached) = (u - self.home, last - self.home);
        to * reached >= 0.0 && to.abs() <= reached.abs()
    }
}

/// How many marches are remembered at once: a cache, cleared past its bound.
const MARCHES_MAX: usize = 32;

thread_local! {
    /// The marches the kernel and drawing paths read, by what each is a function of: the body's
    /// encoding, the columns and numbers it was made at, its home and how far it runs — so a
    /// remembered one is the march made again.  By content, not by where the encoding lives:
    /// every contact on a curve carries a copy, and they read one march.
    static MARCHES: std::cell::RefCell<Vec<(Vec<u64>, Marched)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

/// A march as remembered: made, or refused (`None` where it could not start).
type Marched = Result<std::rc::Rc<March>, Option<ode::Refusal>>;

/// Where a march to read `u` at runs to, from `home`: the end of the curve's `domain` on `u`'s
/// side, so that every place within the domain is read off one march — the march a function of
/// the columns alone, and a reading a function of its place.  Past the end, the end's distance
/// doubled until it covers `u`, so a solve's trial places there share a march too.  Without a
/// domain, `u` itself.
pub fn reach(home: f64, u: f64, domain: Option<(f64, f64)>) -> f64 {
    let Some((a, b)) = domain else { return u };
    let (lo, hi) = (a.min(b), a.max(b));
    let past = |end: f64| {
        let mut d = (end - home).abs().max(1e-9 * (1.0 + home.abs()));
        while d < (u - home).abs() {
            d *= 2.0;
        }
        home + d * (u - home).signum()
    };
    if u > home {
        if u <= hi { hi } else { past(hi) }
    } else if u < home {
        if u >= lo { lo } else { past(lo) }
    } else {
        home
    }
}

/// The march of `body` (encoded as `flat`) from `home` at the columns of `outer` to `to`,
/// remembered.  One refused keeps the road to where it was refused, read as far as it reaches.
pub fn remembered(body: &Body, flat: &[f64], outer: &[f64], home: f64, to: f64) -> Marched {
    // an encoding is the core's own, but a march is no place to find out it is not
    if body.check().is_err() || outer.len() < body.n_outer {
        return Err(None);
    }
    let mut key: Vec<u64> = Vec::with_capacity(flat.len() + body.n_outer + 2);
    key.extend(flat.iter().map(|x| x.to_bits()));
    key.extend(outer[1..body.n_outer].iter().map(|x| x.to_bits()));
    key.extend([home.to_bits(), to.to_bits()]);
    let seen = MARCHES.with(|c| c.borrow().iter().find(|(k, _)| *k == key).map(|e| e.1.clone()));
    if let Some(m) = seen {
        return m;
    }
    let mut at = outer[..body.n_outer].to_vec();
    at[0] = home;
    let m = match body.march(&at, to - home) {
        Err(Some(refusal)) if refusal.at() != home => body.march(&at, refusal.at() - home)
            .map(|mut m| {
                m.refused = Some(refusal);
                m
            })
            .map_err(|_| Some(refusal)),
        other => other,
    }.map(std::rc::Rc::new);
    MARCHES.with(|c| {
        let mut c = c.borrow_mut();
        if c.len() >= MARCHES_MAX {
            c.clear();
        }
        c.push((key, m.clone()));
    });
    m
}

/// How far a march reached toward an end: there, or refused (`None` where it could not start).
pub type Reached = Result<(), Option<ode::Refusal>>;

/// How far `body` (encoded as `flat`) is integrated from `home` at the columns of `outer` toward
/// each of `ends` — the place a march is refused at is where its rates lose rank, or where its
/// bound of steps ran out.
pub fn reaches(body: &Body, flat: &[f64], outer: &[f64], home: f64, ends: [f64; 2])
    -> [Reached; 2] {
    ends.map(|end| match end == home {
        true => Ok(()),
        false => remembered(body, flat, outer, home, end)
            .and_then(|m| m.refused.map_or(Ok(()), |r| Err(Some(r)))),
    })
}

impl<'a> Body<'a> {
    /// The flow from the home at `outer` (`outer[0]` the anchor) across `span` (either sign).
    /// `Err` where it is refused: the place it stopped, or `None` where it could not start.
    pub fn march(&self, outer: &[f64], span: f64) -> Result<March, Option<ode::Refusal>> {
        let y0 = self.start(outer).ok_or(None)?;
        let (n, nf) = (self.n_q, self.n_free());
        let branch = Branch([0.0; MAX_Q]);
        let mut flow = Flow { body: self, outer: outer[..self.n_outer].to_vec(), branch };
        // the rates at the home start the branch
        let s = self.rates(outer, &y0[..n], &[0.0; MAX_Q][..n], true)
            .ok_or(Some(ode::Refusal::Undefined { at: outer[0] }))?;
        flow.branch.0[..n].copy_from_slice(&s.q1);
        let size = y0[..n + nf].iter().fold(1.0f64, |a, x| a.max(x.abs()));
        let scale = vec![size; n + nf];
        let settings = ode::Settings::default();
        let path = ode::march(&mut flow, outer[0], &y0, self.n_theta, span, &scale, &settings)
            .map_err(Some)?;
        Ok(March { outer: outer[..self.n_outer].to_vec(), home: outer[0], path, refused: None })
    }

    /// The body at `u` along a march: its reading to `need` orders (1 to 3), with its
    /// sensitivities to the columns where `sens` (else zero) and those of its derivatives along
    /// the formal too where `frame`.  `None` where it cannot be had there.
    pub fn read(&self, m: &March, u: f64, need: u8, sens: bool, frame: bool) -> Option<Reading> {
        let (n, nf, nt) = (self.n_q, self.n_free(), self.n_theta);
        if !m.reaches(u) {
            return None;
        }
        // the sensitivities ride the step to `u` only where a gradient is asked for
        let sens = sens || frame;
        let mut flow = Flow { body: self, outer: m.outer.clone(), branch: Branch([0.0; MAX_Q]) };
        let node = m.path.at(&mut flow, u, sens).ok()?;
        let y = &node.y;
        let mut outer = m.outer.clone();
        outer[0] = u;
        outer[1..1 + nf].copy_from_slice(&y[n..n + nf]);
        let s = self.rates(&outer, &y[..n], &node.branch.0[..n], true)?;
        let upto = need.clamp(1, 3) as usize;
        let coef = self.higher(&outer, &y[..n], &s, upto)?;
        let (t, dim) = (self.traced, self.dim);
        let mut d = [[f64::NAN; 3]; 4];
        let fact = [1.0, 1.0, 2.0, 6.0];
        for (k, c) in coef.iter().enumerate() {
            for a in 0..dim {
                d[k][a] = c[t + a] * fact[k];
            }
        }
        let w = n + nf;
        let zero = vec![0.0; w * nt];
        let phi = if sens { &y[w..] } else { &zero[..] };
        let dc: Vec<[f64; 3]> = (0..nt)
            .map(|j| std::array::from_fn(|a| if a < dim { phi[(t + a) * nt + j] } else { 0.0 }))
            .collect();
        let (mut dd1, mut dd2) = (Vec::new(), Vec::new());
        if frame {
            // ∂C′/∂θ = (∂f/∂y) Φ, exact
            let mut jac = vec![0.0; n * w];
            if !self.rate_jacobian(&outer, &y[..n], &s, &mut jac) {
                return None;
            }
            dd1 = (0..nt).map(|j| std::array::from_fn(|a| if a < dim {
                (0..w).map(|k| jac[(t + a) * w + k] * phi[k * nt + j]).sum()
            } else { 0.0 })).collect();
            if upto >= 3 {
                // ∂C″/∂θ: the second order a function of the state, differenced along Φ's column
                // from this place — no march, a local map
                let size = y[..w].iter().fold(1.0f64, |a, x| a.max(x.abs()));
                for j in 0..nt {
                    let len = (0..w).map(|k| phi[k * nt + j].abs()).fold(0.0f64, f64::max);
                    let h = if len > 0.0 { 1e-6 * size / len } else { 0.0 };
                    if h == 0.0 {
                        dd2.push([0.0; 3]);
                        continue;
                    }
                    let yp: Vec<f64> = (0..w).map(|k| y[k] + h * phi[k * nt + j]).collect();
                    let mut op = outer.clone();
                    op[1..1 + nf].copy_from_slice(&yp[n..n + nf]);
                    let sp = self.rates(&op, &yp[..n], &s.q1, false)?;
                    let cp = self.higher(&op, &yp[..n], &sp, 2)?;
                    dd2.push(std::array::from_fn(|a| if a < dim {
                        (cp[2][t + a] - coef[2][t + a]) * 2.0 / h
                    } else { 0.0 }));
                }
            }
        }
        Some(Reading { d, orders: upto as u8, dc, dd1, dd2 })
    }
}
