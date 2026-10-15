//! **A traced body in space** (#326, R3 and R5): a point of a component as one or two of its
//! numeric formals run, where the component states relations in space.
//!
//! `k := Helix(ax, r: 5, lead: 3).p over u in (0, 720)` is a **curve in space** and
//! `S := Element(P, L, …).X over (phi in (a, b), s in (c, d))` a **surface**: §6.5's curve, the
//! component's body solved in space rather than in one plane, over one formal or two.  The body is
//! lowered once (`program::traced`) to **rows** — static kernels, each with a map from its columns
//! into one variable table, as a planar trace's are (`locus.rs`) — over `[formals, θ, values] ++ q
//! ++ w`: the running formals, the coordinates of the geometry the component is written over
//! (each point's lift, three numbers), the constants it was given, then the body's own points
//! `q` and the numbers it reads over the formals `w` (a tape each, read by its row's free twin).
//! A point a motion carries (`X coincident(under: m, t == −phi) x`, R4) is a row of its own, the
//! motion's steps its constants and its axes columns of the variable table.
//!
//! **Its point is a small Newton solve**, started at the body's seeds (tapes over the formals) at
//! the asked place and, where that fails, marched there from the body's home: no pose is
//! remembered, so a reading is a function of its place alone.  **Its derivatives are exact.**
//! Along any direction `d` of the outer variables the point is a Taylor series — the implicit
//! function's, one order a solve with the factorisation the root paid for (Wagner, Walther and
//! Schaefer, as `locus::higher_orders` reads one along `u`), every row read over `taylor::Jet`s
//! (each kernel's form) and every tape over them too (`tape::eval_num`).  A derivative across
//! directions is read by **polarisation**: `D²X[a, b] = (S(a + b) − S(a) − S(b))/2`, `S(v)` the
//! second order along `v`, and the third order likewise over seven series — so a surface's normal
//! `C_φ × C_s` and its curvature are exact, and their Jacobians in every column too.
//!
//! **Contacts** with a traced body own a place `[u, s]` (a curve's `s` held at none): a point on
//! it (three rows), a line along a curve's tangent (two), a line in a surface's tangent plane
//! (one), along its normal or a plane touching it (two each), and a circle osculating a surface's
//! section by the circle's plane (three, Meusnier, as an envelope's, `envelope::held`).  Each is
//! written once over `kernels::Num` from a `Frame` — the point and its derivatives in the formals —
//! read exact in every column, so a kernel's Jacobian is exact by forward differentiation.
//!
//! What this certifies is local: a root near where it was seeded, on the branch the seeds pick;
//! not that the body is regular over its whole domain (`regularity` says where it is not).

use crate::envelope::held::{col3, cross, dot, unit, vscale, vsub};
use crate::envelope::{held, screw};
use crate::kernels::{Dual, Kernel, KernelFn, Num, KERNELS};
use crate::taylor::{self, Jet};

/// Bounds on what a document may ask for: a document is untrusted input, and the encoding is
/// range-checked as it is read.
pub const MAX_Q: usize = 48;
pub const MAX_ROWS: usize = 64;
pub const MAX_W: usize = 16;
/// A traced body runs over one formal (a curve) or two (a surface).
pub const MAX_FORMALS: usize = 2;
/// The place a contact owns: two numbers whatever the body, a curve's second held at none.
pub const PLACE: usize = 2;

const NEWTON_MAX: usize = 40;
const MARCH: usize = 32;

/* -- the compiled form -------------------------------------------------------------------- */

/// One row of the body: a static kernel (`kid`) or a point a motion carries (`None`,
/// `screw::Reading::Carried`, its constants the motion's steps), the variable-table slot each of
/// its columns reads, and its constants.
#[derive(Clone, Debug)]
pub struct Row {
    pub kid: Option<usize>,
    pub cols: Vec<u32>,
    pub consts: Vec<f64>,
}

impl Row {
    fn kernel(kid: Option<usize>, n_par: usize, n_const: usize) -> Option<Kernel> {
        match kid {
            Some(k) => KERNELS.get(k).copied(),
            None => Some(screw::kernel(screw::Reading::Carried, n_par, n_const)),
        }
    }
}

/// A compiled body, held only as its encoding — what rides in a contact's constants.  The
/// variable table its rows index is `[formals, θ, values] ++ q ++ w`.
#[derive(Clone, Debug, PartialEq)]
pub struct Body {
    pub flat: Vec<f64>,
}

const HEADER: usize = 11;

/// What a body that states a rate adds (#339 D2): the rows that read a rate, those that hold at
/// its home alone (`initially { … }`), and the interval its curve runs over, which its march
/// reaches.  Empty for every other body.
#[derive(Clone, Debug, Default)]
pub struct Rates {
    pub rows: Vec<Row>,
    pub init: Vec<Row>,
    pub domain: [f64; 2],
}

impl Body {
    /// Validate and encode.  An `Err` is a diagnostic for the body, not a panic.  `rates` is what
    /// a body that states a rate adds (#339 D2).
    #[allow(clippy::too_many_arguments)]
    pub fn new(n_formals: usize, n_outer: usize, n_theta: usize, n_q: usize, traced: usize,
               w: Vec<crate::tape::Tape>, seeds: Vec<crate::tape::Tape>, rows: Vec<Row>,
               rates: Rates) -> Result<Body, String> {
        let stated = !rates.rows.is_empty();
        if stated && n_formals != 1 {
            return Err("a body that states a rate runs over one formal: one running over two \
                is marched along the first (#339 D4), which is not yet read".into());
        }
        if !stated && !rates.init.is_empty() {
            return Err("`initially { … }` says what holds at a home, and only a body that \
                states a rate has one: its rates are integrated from it".into());
        }
        if stated && rows.iter().chain(&rates.rows).chain(&rates.init).any(|r| r.kid.is_none()) {
            return Err("a body that states a rate reads no point a motion carries".into());
        }
        if !(1..=MAX_FORMALS).contains(&n_formals) || n_formals + n_theta > n_outer {
            return Err("a traced body runs over one formal or two".into());
        }
        if n_q == 0 || n_q > MAX_Q {
            return Err(format!("a traced body in space places between 1 and {MAX_Q} numbers"));
        }
        if rows.len() > MAX_ROWS || w.len() > MAX_W {
            return Err(format!("a traced body may state at most {MAX_ROWS} relations and \
                                {MAX_W} expressions"));
        }
        if seeds.len() != n_q || traced + 3 > n_q {
            return Err("traced body shape mismatch".into());
        }
        let mut n_res = 0;
        for r in &rows {
            let kn = Row::kernel(r.kid, r.cols.len(), r.consts.len()).ok_or("no such kernel")?;
            if let Some(k) = r.kid {
                if !taylor::has_form(k) {
                    return Err(format!("a body traced in space reads its rows' derivatives \
                                        exactly, and `{}` has no Taylor form", KERNELS[k].name));
                }
            }
            n_res += kn.n_res;
        }
        if !stated && n_res != n_q {
            return Err(format!("a traced body must determine its points: {n_q} unknowns against \
                                {n_res} equations"));
        }
        let mut f = vec![n_formals as f64, n_outer as f64, n_theta as f64, n_q as f64,
                         w.len() as f64, rows.len() as f64, traced as f64,
                         rates.rows.len() as f64, rates.init.len() as f64,
                         rates.domain[0], rates.domain[1]];
        for t in &w {
            f.push(t.flat.len() as f64);
            f.extend_from_slice(&t.flat);
        }
        for r in rows.iter().chain(&rates.rows).chain(&rates.init) {
            f.push(r.kid.map_or(-1.0, |k| k as f64));
            f.push(r.cols.len() as f64);
            f.push(r.consts.len() as f64);
            f.extend(r.cols.iter().map(|&c| c as f64));
            f.extend_from_slice(&r.consts);
        }
        for t in &seeds {
            f.push(t.flat.len() as f64);
            f.extend_from_slice(&t.flat);
        }
        // every row's shape and column is checked once, as any encoding is read
        let v = view(&f).ok_or("traced body shape mismatch")?;
        if stated {
            v.rate_body().check()?;
        }
        Ok(Body { flat: f })
    }

    /// Whether the body states a rate (#339 D2).
    pub fn states_rate(&self) -> bool {
        view(&self.flat).is_some_and(|v| !v.rates.is_empty())
    }

    /// How many formals it runs over.
    pub fn n_formals(&self) -> usize {
        view(&self.flat).map_or(0, |v| v.nf)
    }

    /// How many coordinates of geometry it is written over (its columns past the place).
    pub fn n_theta(&self) -> usize {
        view(&self.flat).map_or(0, |v| v.n_theta)
    }
}

/// The encoding, decoded to borrowed slices, every length and column checked on the way in.
struct View<'a> {
    nf: usize,
    n_outer: usize,
    n_theta: usize,
    n_q: usize,
    traced: usize,
    w: Vec<&'a [f64]>,
    rows: Vec<(Kernel, Option<usize>, Vec<usize>, &'a [f64])>,
    seeds: Vec<&'a [f64]>,
    /// A body that states a rate (#339 D2): its rate rows, its home's, its interval.
    rates: Vec<(Kernel, Option<usize>, Vec<usize>, &'a [f64])>,
    init: Vec<(Kernel, Option<usize>, Vec<usize>, &'a [f64])>,
    domain: [f64; 2],
}

impl<'a> View<'a> {
    /// The body as the integrator reads it (`rates.rs`): over one formal, in space.
    fn rate_body(&self) -> crate::rates::Body<'a> {
        type Read<'a> = (Kernel, Option<usize>, Vec<usize>, &'a [f64]);
        let rows = |rs: &[Read<'a>]| -> Vec<crate::rates::Row<'a>> {
            rs.iter().map(|(_, kid, cols, consts)| crate::rates::Row {
                kid: kid.unwrap_or(usize::MAX),
                cols: cols.clone(),
                consts,
            }).collect()
        };
        crate::rates::Body {
            n_outer: self.n_outer,
            n_theta: self.n_theta,
            n_q: self.n_q,
            pose: false,
            traced: self.traced,
            dim: 3,
            w: self.w.clone(),
            seeds: self.seeds.clone(),
            alg: rows(&self.rows),
            rate: rows(&self.rates),
            init: rows(&self.init),
        }
    }
}

fn view(flat: &[f64]) -> Option<View<'_>> {
    let g = |i: usize| flat.get(i).copied()
        .filter(|v| v.is_finite() && *v >= 0.0 && v.fract() == 0.0).map(|v| v as usize);
    let (nf, n_outer, n_theta, n_q, n_w, n_rows, traced) =
        (g(0)?, g(1)?, g(2)?, g(3)?, g(4)?, g(5)?, g(6)?);
    let (n_rates, n_init) = (g(7)?, g(8)?);
    let domain = [*flat.get(9)?, *flat.get(10)?];
    if !(1..=MAX_FORMALS).contains(&nf) || n_q > MAX_Q || n_w > MAX_W || n_rows > MAX_ROWS
        || n_rates > MAX_ROWS || n_init > MAX_ROWS
        || traced + 3 > n_q || nf + n_theta > n_outer || n_outer > 4096 {
        return None;
    }
    // a body that states a rate has its rates as columns past `w` (#339 D2)
    let width = n_outer + n_q + n_w + if n_rates > 0 { n_q } else { 0 };
    let mut at = HEADER;
    let take = |n: usize, at: &mut usize| -> Option<&[f64]> {
        let s = flat.get(*at..*at + n)?;
        *at += n;
        Some(s)
    };
    let mut w = Vec::with_capacity(n_w);
    for _ in 0..n_w {
        let n = g(at)?;
        at += 1;
        w.push(take(n, &mut at)?);
    }
    let mut all = Vec::with_capacity(n_rows + n_rates + n_init);
    for _ in 0..n_rows + n_rates + n_init {
        let kid = match flat.get(at).copied()? {
            k if k == -1.0 => None,
            _ => Some(g(at)?),
        };
        let (n_cols, n_consts) = (g(at + 1)?, g(at + 2)?);
        at += 3;
        let cols: Vec<usize> = take(n_cols, &mut at)?.iter().map(|&c| c as usize).collect();
        if cols.iter().any(|&c| c >= width) {
            return None;
        }
        let consts = take(n_consts, &mut at)?;
        let kn = Row::kernel(kid, n_cols, n_consts)?;
        if kid.is_some() && (kn.n_par != n_cols || kn.n_const != n_consts) {
            return None;
        }
        if kid.is_none() {
            let steps = held::read_steps(consts)?;
            if n_cols != 7 + held::steps_width(&steps) {
                return None;
            }
        }
        all.push((kn, kid, cols, consts));
    }
    let init = all.split_off(n_rows + n_rates);
    let rates = all.split_off(n_rows);
    let rows = all;
    let mut seeds = Vec::with_capacity(n_q);
    for _ in 0..n_q {
        let n = g(at)?;
        at += 1;
        seeds.push(take(n, &mut at)?);
    }
    if rates.is_empty() && rows.iter().map(|r| r.0.n_res).sum::<usize>() != n_q {
        return None;
    }
    Some(View { nf, n_outer, n_theta, n_q, traced, w, rows, seeds, rates, init, domain })
}

/* -- solving the body at a place ---------------------------------------------------------- */

/// The body solved at one place: the variable table there and the factorised `∂r/∂q`.
struct Solved {
    z: Vec<f64>,
    lu: Vec<f64>,
    piv: Vec<usize>,
}

impl View<'_> {
    /// The derived values over the outer variables.
    fn derived(&self, outer: &[f64]) -> Option<Vec<f64>> {
        self.w.iter().map(|t| crate::tape::eval_num::<f64>(t, outer)).collect()
    }

    /// The body's residuals over the whole variable table `z`, into `r`.
    fn residual(&self, z: &[f64], r: &mut Vec<f64>) {
        r.resize(self.n_q, 0.0);
        let mut at = 0;
        for (kn, _, cols, consts) in &self.rows {
            let x: Vec<f64> = cols.iter().map(|&c| z[c]).collect();
            (kn.res)(1, &x, consts, &mut r[at..at + kn.n_res]);
            at += kn.n_res;
        }
    }

    /// `∂r/∂q`, row-major `n_q × n_q`.
    fn jacobian(&self, z: &[f64]) -> Vec<f64> {
        let n = self.n_q;
        let mut j = vec![0.0; n * n];
        let mut row = 0;
        for (kn, _, cols, consts) in &self.rows {
            let x: Vec<f64> = cols.iter().map(|&c| z[c]).collect();
            let mut jr = vec![0.0; kn.n_res * kn.n_par];
            kn.jac_into(&x, consts, &mut jr);
            for t in 0..kn.n_res {
                for (c, &col) in cols.iter().enumerate() {
                    if (self.n_outer..self.n_outer + n).contains(&col) {
                        j[(row + t) * n + col - self.n_outer] += jr[t * kn.n_par + c];
                    }
                }
            }
            row += kn.n_res;
        }
        j
    }

    /// The seeds' places at the outer variables.
    fn seeded(&self, outer: &[f64]) -> Option<Vec<f64>> {
        self.seeds.iter().map(|t| crate::tape::eval_num::<f64>(t, outer)).collect()
    }

    /// Damped Newton on `q` at `outer`, from `q`.  The root, or `None`.
    fn newton(&self, outer: &[f64], q: &[f64]) -> Option<Solved> {
        let w = self.derived(outer)?;
        let mut z: Vec<f64> = outer.iter().chain(q).chain(&w).copied().collect();
        let scale = z.iter().fold(1.0f64, |a, x| a.max(x.abs()));
        let tol = 1e-12 * scale;
        let mut r = Vec::with_capacity(self.n_q);
        self.residual(&z, &mut r);
        let norm = |r: &[f64]| r.iter().fold(0.0f64, |a, x| a.max(x.abs()));
        let mut now = norm(&r);
        for _ in 0..NEWTON_MAX {
            if !now.is_finite() {
                return None;
            }
            let mut lu = self.jacobian(&z);
            let mut piv = Vec::new();
            let regular = crate::linalg::lu_factor(self.n_q, &mut lu, &mut piv);
            if now <= tol {
                return regular.then_some(Solved { z, lu, piv });
            }
            if !regular {
                return None;
            }
            let mut step: Vec<f64> = r.iter().map(|x| -x).collect();
            crate::linalg::lu_apply(self.n_q, &lu, &piv, &mut step);
            let mut lambda = 1.0;
            let at = self.n_outer;
            loop {
                let mut trial = z.clone();
                for (k, s) in step.iter().enumerate() {
                    trial[at + k] += lambda * s;
                }
                self.residual(&trial, &mut r);
                let next = norm(&r);
                if next.is_finite() && (next < now || lambda < 1e-4) {
                    z = trial;
                    now = next;
                    break;
                }
                lambda *= 0.5;
                if lambda < 1e-6 {
                    return None;
                }
            }
        }
        None
    }

    /// The body at `outer`: from its seeds there, else marched from `home` (the formals' anchor,
    /// the rest of the outer variables as asked) in steps, each from the last.
    fn solve(&self, outer: &[f64], home: &[f64]) -> Option<Solved> {
        if let Some(s) = self.seeded(outer).and_then(|q| self.newton(outer, &q)) {
            return Some(s);
        }
        let mut at = outer.to_vec();
        at[..self.nf].copy_from_slice(&home[..self.nf]);
        let mut s = self.seeded(&at).and_then(|q| self.newton(&at, &q))?;
        for k in 1..=MARCH {
            let f = k as f64 / MARCH as f64;
            for i in 0..self.nf {
                at[i] = home[i] + f * (outer[i] - home[i]);
            }
            let q = s.z[self.n_outer..self.n_outer + self.n_q].to_vec();
            s = self.newton(&at, &q)?;
        }
        Some(s)
    }

    /// The traced point's Taylor series along `dir` (one entry per outer variable), to `order`:
    /// `order + 1` coefficients of each coordinate.  `None` where a row or a tape cannot be read
    /// over jets.
    fn series(&self, s: &Solved, dir: &[f64], order: usize) -> Option<[Jet; 3]> {
        let (no, nq) = (self.n_outer, self.n_q);
        let outer: Vec<Jet> = (0..no).map(|i| Jet::from(&[s.z[i], dir[i]])).collect();
        let w: Vec<Jet> = self.w.iter().map(|t| crate::tape::eval_num::<Jet>(t, &outer))
            .collect::<Option<_>>()?;
        let mut q: Vec<Jet> = (0..nq).map(|i| Jet::constant(s.z[no + i])).collect();
        let mut jrow = Vec::new();
        // the tapes over jets are exact to the third order (`tape::eval_num`)
        for k in 1..=order.min(taylor::ORDER - 2) {
            let z: Vec<Jet> = outer.iter().chain(&q).chain(&w).copied().collect();
            let mut rk = Vec::with_capacity(nq);
            for (kn, kid, cols, consts) in &self.rows {
                let v: Vec<Jet> = cols.iter().map(|&c| z[c]).collect();
                let mut r = vec![Jet::default(); kn.n_res];
                match kid {
                    Some(id) => {
                        if !taylor::residual(*id, &v, consts, &mut r, &mut jrow) {
                            return None;
                        }
                    }
                    None => screw::rows(screw::Reading::Carried, &v, consts, &mut r)?,
                }
                rk.extend(r.iter().map(|j| -j.0[k]));
            }
            crate::linalg::lu_apply(nq, &s.lu, &s.piv, &mut rk);
            for (i, x) in rk.into_iter().enumerate() {
                q[i].0[k] = x;
            }
        }
        let t = self.traced;
        Some([q[t], q[t + 1], q[t + 2]])
    }
}

/* -- the frame a contact reads ------------------------------------------------------------- */

/// The traced point and its derivatives in the formals, at a place: `x`, `d[a] = ∂x/∂a` and
/// `dd[a][b] = ∂²x/∂a∂b` (zero past the body's formals, and where not asked for).
#[derive(Clone, Copy, Debug)]
pub struct Frame<T> {
    pub x: [T; 3],
    pub d: [[T; 3]; MAX_FORMALS],
    pub dd: [[[T; 3]; MAX_FORMALS]; MAX_FORMALS],
}

/// How much of the frame to read.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Need {
    /// The point.
    Point,
    /// And its first derivatives.
    Tangent,
    /// And its second.
    Curvature,
}

/// A frame read exactly, and — where asked — its Jacobian in the body's columns: the place (two)
/// then `θ`, each a `[f64; 3]` per entry, so `grad[c]` is the derivative along column `c`.
pub struct Read {
    pub frame: Frame<f64>,
    /// `x`, then `d[a]`, then `dd[a][b]` (in the frame's order), each against every column.
    grad: Option<Vec<[[f64; 3]; SLOTS]>>,
}

/// A frame's entries: `x`, `d[a]`, `dd[a][b]`.
const SLOTS: usize = 1 + MAX_FORMALS + MAX_FORMALS * MAX_FORMALS;

/// One coordinate set's `k`-th derivative from a series: the coefficient times `k!`.
fn nth(s: &[Jet; 3], k: usize) -> [f64; 3] {
    s.map(|j| j.nth(k))
}

/// A memo of series along directions, keyed by the direction's coefficients — a frame reads many
/// directions more than once.
struct Series<'a, 'b> {
    v: &'a View<'b>,
    s: &'a Solved,
    order: usize,
    memo: Vec<(Vec<f64>, [Jet; 3])>,
}

impl Series<'_, '_> {
    fn along(&mut self, dir: Vec<f64>) -> Option<[Jet; 3]> {
        if let Some((_, j)) = self.memo.iter().find(|(d, _)| *d == dir) {
            return Some(*j);
        }
        let j = self.v.series(self.s, &dir, self.order)?;
        self.memo.push((dir, j));
        Some(j)
    }

    /// The unit direction of outer variable `i`, and sums of them.
    fn dir(&self, terms: &[usize]) -> Vec<f64> {
        let mut d = vec![0.0; self.v.n_outer];
        for &i in terms {
            d[i] += 1.0;
        }
        d
    }

    /// `D^k X` along the sum of the unit directions `terms`.
    fn power(&mut self, terms: &[usize], k: usize) -> Option<[f64; 3]> {
        let d = self.dir(terms);
        Some(nth(&self.along(d)?, k))
    }

    /// The symmetric `k`-linear derivative `D^k X[e_terms…]` by polarisation over the series along
    /// the sums of every subset of `terms` (`k` of them, two or three).
    fn mixed(&mut self, terms: &[usize]) -> Option<[f64; 3]> {
        let k = terms.len();
        let mut acc = [0.0; 3];
        // Σ over subsets S of (−1)^(k − |S|) D^k X[Σ_S e]^k / k!
        for mask in 1u32..(1 << k) {
            let sub: Vec<usize> =
                (0..k).filter(|i| mask & (1 << i) != 0).map(|i| terms[i]).collect();
            let sign = if (k - sub.len()) % 2 == 0 { 1.0 } else { -1.0 };
            let p = self.power(&sub, k)?;
            for c in 0..3 {
                acc[c] += sign * p[c];
            }
        }
        let fact: f64 = (1..=k).map(|x| x as f64).product();
        Some(acc.map(|x| x / fact))
    }
}

/// The body over its constants at the outer variables `outer` (`[place, θ, values]`): the frame
/// `need` asks for, and with `grad` its derivative in every column of the place and `θ`.  `home`
/// is where a march starts.  `None` where the body cannot be solved there.
pub fn read(flat: &[f64], outer: &[f64], home: &[f64], need: Need, grad: bool) -> Option<Read> {
    let v = view(flat)?;
    if outer.len() != v.n_outer || home.len() < v.nf {
        return None;
    }
    if !v.rates.is_empty() {
        return rate_read(flat, &v, outer, home[0], need, grad);
    }
    let s = v.solve(outer, home)?;
    let order = match need {
        Need::Point => 1,
        Need::Tangent => 2,
        Need::Curvature => 3,
    } - usize::from(!grad);
    let nf = v.nf;
    let mut ser = Series { v: &v, s: &s, order: order.max(1), memo: Vec::new() };
    let t = v.traced;
    let x = [s.z[v.n_outer + t], s.z[v.n_outer + t + 1], s.z[v.n_outer + t + 2]];
    let zero = [0.0; 3];
    let mut frame = Frame { x, d: [zero; MAX_FORMALS], dd: [[zero; MAX_FORMALS]; MAX_FORMALS] };
    if need >= Need::Tangent {
        for a in 0..nf {
            frame.d[a] = ser.power(&[a], 1)?;
        }
    }
    if need >= Need::Curvature {
        for a in 0..nf {
            for b in a..nf {
                let m = if a == b { ser.power(&[a], 2)? } else { ser.mixed(&[a, b])? };
                frame.dd[a][b] = m;
                frame.dd[b][a] = m;
            }
        }
    }
    let grad = if grad {
        // every column of the place and the geometry: the formals are the first outer
        // variables, θ the next; a curve's second place column moves nothing
        let n_c = PLACE + v.n_theta;
        let mut g = vec![[zero; SLOTS]; n_c];
        for (c, gc) in g.iter_mut().enumerate() {
            let k = match c {
                c if c < PLACE && c >= nf => continue,
                c if c < PLACE => c,
                c => nf + c - PLACE,
            };
            gc[0] = ser.power(&[k], 1)?;
            if need >= Need::Tangent {
                for a in 0..nf {
                    gc[1 + a] = ser.mixed(&[a, k])?;
                }
            }
            if need >= Need::Curvature {
                for a in 0..nf {
                    for b in 0..nf {
                        gc[1 + MAX_FORMALS + a * MAX_FORMALS + b] = ser.mixed(&[a, b, k])?;
                    }
                }
            }
        }
        Some(g)
    } else {
        None
    };
    Some(Read { frame, grad })
}

/// A body that states a rate, read at `outer[0]` (#339 D2): the flow from its home, its frame
/// and, with `grad`, the frame's derivative in its place and every `θ` column — along the formal
/// the flow's own higher orders, along a column its sensitivities.
fn rate_read(flat: &[f64], v: &View, outer: &[f64], home: f64, need: Need, grad: bool)
    -> Option<Read> {
    let body = v.rate_body();
    let u = outer[0];
    let (lo, hi) = (v.domain[0], v.domain[1]);
    let m = crate::rates::remembered(&body, flat, outer, home,
                                     crate::rates::reach(home, u, Some((lo, hi)))).ok()?;
    // a Jacobian reads one order past the frame
    let order = match need {
        Need::Point => 1,
        Need::Tangent => 2,
        Need::Curvature => 3,
    } + u8::from(grad);
    let r = body.read(&m, u, order.min(3), grad, grad)?;
    let zero = [0.0; 3];
    let mut frame =
        Frame { x: r.d[0], d: [zero; MAX_FORMALS], dd: [[zero; MAX_FORMALS]; MAX_FORMALS] };
    if need >= Need::Tangent {
        frame.d[0] = r.d[1];
    }
    if need >= Need::Curvature {
        frame.dd[0][0] = r.d[2];
    }
    let grad = grad.then(|| {
        let mut g = vec![[zero; SLOTS]; PLACE + v.n_theta];
        g[0][0] = r.d[1];
        g[0][1] = r.d[2];
        if r.orders >= 3 {
            g[0][1 + MAX_FORMALS] = r.d[3];
        }
        for (c, gc) in g.iter_mut().enumerate().skip(PLACE) {
            let j = c - PLACE;
            gc[0] = r.dc[j];
            if let Some(d) = r.dd1.get(j) {
                gc[1] = *d;
            }
            if let Some(d) = r.dd2.get(j) {
                gc[1 + MAX_FORMALS] = *d;
            }
        }
        g
    });
    Some(Read { frame, grad })
}

impl Read {
    /// The frame over numbers carrying derivatives, linearised: `x + Σ δ_c ∂x/∂c` over the body's
    /// columns `c` (place then `θ`), `δ_c` the caller's motion of column `c` (`None` where it has
    /// none) — exact to first order, all a Jacobian reads.
    fn linear<T: Num>(&self, delta: impl Fn(usize) -> Option<T>) -> Frame<T> {
        let n = self.grad.as_ref().map_or(0, |g| g.len());
        let deltas: Vec<Option<T>> = (0..n).map(&delta).collect();
        let lift = |v: [f64; 3], slot: usize| -> [T; 3] {
            std::array::from_fn(|i| {
                let mut acc = T::cst(v[i]);
                for (g, d) in self.grad.iter().flatten().zip(&deltas) {
                    if let (Some(d), true) = (d, g[slot][i] != 0.0) {
                        acc = acc + *d * T::cst(g[slot][i]);
                    }
                }
                acc
            })
        };
        let f = &self.frame;
        Frame {
            x: lift(f.x, 0),
            d: std::array::from_fn(|a| lift(f.d[a], 1 + a)),
            dd: std::array::from_fn(|a| std::array::from_fn(|b| {
                lift(f.dd[a][b], 1 + MAX_FORMALS + a * MAX_FORMALS + b)
            })),
        }
    }
}

/* -- the contacts ---------------------------------------------------------------------------- */

/// Which statement a contact with a traced body makes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Contact {
    /// `p coincident k`: `p − X`.  Lead: the point's lift.
    Point = 0,
    /// `l tangent(at: p) k`, `k` a curve: the line along its tangent.  Lead: the line's ends.
    Tangent = 1,
    /// `l tangent(at: p) S`, `S` a surface: the line in its tangent plane.  Lead: the line's ends.
    InPlane = 2,
    /// `l perpendicular(at: p) S`: the line along its normal.  Lead: the line's ends.
    Normal = 3,
    /// `P tangent(at: p) S`: the plane's normal along the surface's.  Lead: the plane's two axes.
    Plane = 4,
    /// `S curvature(at: p) k`: the circle osculating the surface's section by its plane at `p`.
    /// Lead: the centre's lift, the radius, the plane's two axes.
    Curvature = 5,
}

impl Contact {
    pub const ALL: [Contact; 6] = [Contact::Point, Contact::Tangent, Contact::InPlane,
                                   Contact::Normal, Contact::Plane, Contact::Curvature];

    pub fn rows(self) -> usize {
        match self {
            Contact::Point | Contact::Curvature => 3,
            Contact::Tangent | Contact::Normal | Contact::Plane => 2,
            Contact::InPlane => 1,
        }
    }

    pub fn lead(self) -> usize {
        match self {
            Contact::Point => 3,
            Contact::Tangent | Contact::InPlane | Contact::Normal | Contact::Plane => 6,
            Contact::Curvature => 10,
        }
    }

    fn need(self) -> Need {
        match self {
            Contact::Point => Need::Point,
            Contact::Curvature => Need::Curvature,
            _ => Need::Tangent,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Contact::Point => "point_on_traced",
            Contact::Tangent => "traced_tangent_at",
            Contact::InPlane => "traced_surface_tangent_at",
            Contact::Normal => "traced_normal_at",
            Contact::Plane => "traced_plane_at",
            Contact::Curvature => "traced_curvature_at",
        }
    }

    /// A point's and a circle's rows are lengths; a direction's are cosines.
    fn degree(self) -> u32 {
        match self {
            Contact::Point | Contact::Curvature => 1,
            _ => 0,
        }
    }

    /// The body's formals the contact is about: a tangent along a curve, the rest of a surface.
    pub fn formals(self) -> Option<usize> {
        match self {
            Contact::Point => None,
            Contact::Tangent => Some(1),
            _ => Some(2),
        }
    }
}

/// A contact's constants: `[n_values, values…, home…(2), body…]`.
pub fn consts(values: &[f64], home: [f64; 2], body: &Body) -> Vec<f64> {
    let mut k = vec![values.len() as f64];
    k.extend_from_slice(values);
    k.extend(home);
    k.extend_from_slice(&body.flat);
    k
}

/// The constants read back: the values, the home and the body's encoding.
fn parts(k: &[f64]) -> Option<(&[f64], [f64; 2], &[f64])> {
    let n = k.first().copied().filter(|v| v.is_finite() && *v >= 0.0 && *v <= 4096.0)? as usize;
    let values = k.get(1..1 + n)?;
    let home = [*k.get(1 + n)?, *k.get(2 + n)?];
    Some((values, home, k.get(3 + n..)?))
}

/// The smallest normal curvature a contact divides by.
const FLAT: f64 = 1e-12;

/// A contact's rows from its lead columns and the frame, into `out`.
fn rows<T: Num>(c: Contact, lead: &[T], f: &Frame<T>, out: &mut [T]) {
    let n = || unit(cross(f.d[0], f.d[1]));
    let dir = || unit(vsub(col3(lead, 3), col3(lead, 0)));
    match c {
        Contact::Point => {
            for i in 0..3 {
                out[i] = lead[i] - f.x[i];
            }
        }
        Contact::Tangent => {
            let (e1, e2) = screw::square_to(unit(f.d[0]));
            let l = dir();
            out[0] = dot(l, e1);
            out[1] = dot(l, e2);
        }
        Contact::InPlane => out[0] = dot(dir(), n()),
        Contact::Normal | Contact::Plane => {
            let l = if c == Contact::Plane {
                unit(cross(col3(lead, 0), col3(lead, 3)))
            } else {
                dir()
            };
            let t1 = unit(f.d[0]);
            let t2 = cross(n(), t1);
            out[0] = dot(l, t1);
            out[1] = dot(l, t2);
        }
        // Meusnier, as an envelope's (`envelope::held`): the section by the plane of normal `m`
        // turns `κₙ / cos θ` toward `n − (n·m)m`, `κₙ` the normal curvature along `m × n`
        Contact::Curvature => {
            let (centre, r) = (col3(lead, 0), lead[3]);
            let (pu, pv) = (col3(lead, 4), col3(lead, 7));
            let m = unit(cross(pu, pv));
            let nn = n();
            let w = unit(cross(m, nn));
            let (xu, xv) = (f.d[0], f.d[1]);
            let (e, ff, g) = (dot(xu, xu), dot(xu, xv), dot(xv, xv));
            let (l, mm, nn2) = (dot(f.dd[0][0], nn), dot(f.dd[0][1], nn), dot(f.dd[1][1], nn));
            let (b1, b2) = (dot(xu, w), dot(xv, w));
            let det = e * g - ff * ff;
            let det = if det.val() != 0.0 { det } else { T::cst(1.0) };
            let (a, b) = ((g * b1 - ff * b2) / det, (e * b2 - ff * b1) / det);
            let two = T::cst(2.0);
            let second = l * a * a + two * mm * a * b + nn2 * b * b;
            let first = e * a * a + two * ff * a * b + g * b * b;
            let first = if first.val() != 0.0 { first } else { T::cst(1.0) };
            let kappa = second / first;
            let kappa = if kappa.val().abs() >= FLAT { kappa }
                else { T::cst(if kappa.val() < 0.0 { -FLAT } else { FLAT }) };
            let inward = vsub(nn, vscale(m, dot(nn, m)));
            let to = vsub(centre, f.x);
            let off = vsub(to, vscale(inward, T::cst(1.0) / kappa));
            let a = unit(pu);
            let bb = unit(cross(m, a));
            out[0] = dot(off, a);
            out[1] = dot(off, bb);
            out[2] = dot(to, to).sqrt() - r;
        }
    }
}

/// The body's outer variables at a contact's columns: its place, its `θ` and the values.
fn outer_of(c: Contact, cols: &[f64], values: &[f64], nf: usize) -> Vec<f64> {
    let at = c.lead();
    let mut o = cols[at..at + nf].to_vec();
    o.extend_from_slice(&cols[at + PLACE..]);
    o.extend_from_slice(values);
    o
}

/// A contact's residual over its columns and constants, into `out`; `None` where they do not
/// describe one or the body cannot be solved there.
fn residual(c: Contact, cols: &[f64], k: &[f64], out: &mut [f64]) -> Option<()> {
    let (values, home, flat) = parts(k)?;
    let v = view(flat)?;
    if cols.len() != c.lead() + PLACE + v.n_theta || out.len() != c.rows()
        || c.formals().is_some_and(|n| n != v.nf) {
        return None;
    }
    let outer = outer_of(c, cols, values, v.nf);
    let r = read(flat, &outer, &home, c.need(), false)?;
    rows(c, &cols[..c.lead()], &r.frame, out);
    out.iter().all(|x| x.is_finite()).then_some(())
}

/// The same, its Jacobian row-major into `j`: the lead by `Dual` directly, the place and `θ`
/// through the frame's exact derivatives.
fn jacobian(c: Contact, cols: &[f64], k: &[f64], j: &mut [f64]) -> Option<()> {
    const P: usize = held::PASS;
    let (values, home, flat) = parts(k)?;
    let v = view(flat)?;
    let w = cols.len();
    if w != c.lead() + PLACE + v.n_theta || j.len() != c.rows() * w
        || c.formals().is_some_and(|n| n != v.nf) {
        return None;
    }
    let outer = outer_of(c, cols, values, v.nf);
    let r = read(flat, &outer, &home, c.need(), true)?;
    let mut lead = vec![Dual::<P>::cst(0.0); c.lead()];
    let mut out = vec![Dual::<P>::cst(0.0); c.rows()];
    for start in (0..w).step_by(P) {
        for (q, x) in lead.iter_mut().enumerate() {
            *x = if (start..start + P).contains(&q) {
                Dual::var(cols[q], q - start)
            } else {
                Dual::cst(cols[q])
            };
        }
        let from = c.lead();
        let f = r.linear(|q| (start..start + P).contains(&(from + q))
            .then(|| Dual::<P>::var(0.0, from + q - start)));
        rows(c, &lead, &f, &mut out);
        held::scatter_pass(&out, j, start);
    }
    Some(())
}

fn res<const C: u8>(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let c = Contact::ALL[C as usize];
    held::block_res(n, v, k, r, c.rows(), |x, kc, out| residual(c, x, kc, out));
}

fn jac<const C: u8>(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let c = Contact::ALL[C as usize];
    let (n_par, _) = crate::kernels::curve_widths(n, v, k);
    held::block_res(n, v, k, j, c.rows() * n_par, |x, kc, out| jacobian(c, x, kc, out));
}

/// The kernel of contact `c` at a block's widths (`system::build_kernel`).
pub fn kernel(c: Contact, n_par: usize, n_const: usize) -> Kernel {
    let (res, jac): (KernelFn, KernelFn) = match c {
        Contact::Point => (res::<0>, jac::<0>),
        Contact::Tangent => (res::<1>, jac::<1>),
        Contact::InPlane => (res::<2>, jac::<2>),
        Contact::Normal => (res::<3>, jac::<3>),
        Contact::Plane => (res::<4>, jac::<4>),
        Contact::Curvature => (res::<5>, jac::<5>),
    };
    Kernel { name: c.name(), n_res: c.rows(), n_par, n_const, degree: c.degree(), res, jac,
             const_jac: None, form: None }
}

/* -- reading a body apart from the solve --------------------------------------------------- */

/// The traced point at a place `[u, s]` over the body's columns' values `theta` and constants:
/// its position, its derivatives in the formals and, on a surface, its unit normal.  What a
/// seed, a report and a check read.
#[derive(Clone, Copy, Debug)]
pub struct Reading {
    pub position: [f64; 3],
    pub d: [[f64; 3]; MAX_FORMALS],
    pub dd: [[[f64; 3]; MAX_FORMALS]; MAX_FORMALS],
    pub normal: Option<[f64; 3]>,
}

/// The body over a contact's constants `k` (`consts`), at `place` and over `theta`.
pub fn at(k: &[f64], theta: &[f64], place: [f64; 2], need: Need) -> Option<Reading> {
    let (values, home, flat) = parts(k)?;
    let v = view(flat)?;
    if theta.len() != v.n_theta {
        return None;
    }
    let mut outer = place[..v.nf].to_vec();
    outer.extend_from_slice(theta);
    outer.extend_from_slice(values);
    let r = read(flat, &outer, &home, need, false)?;
    let f = r.frame;
    let normal = (v.nf == 2 && need >= Need::Tangent).then(|| {
        let n = crate::space::cross(f.d[0], f.d[1]);
        let l = crate::space::norm(n);
        if l > 0.0 { n.map(|x| x / l) } else { n }
    });
    Some(Reading { position: f.x, d: f.d, dd: f.dd, normal })
}

/// Where a point `p` sits nearest on the body over a domain: a coarse grid of places, then
/// Gauss–Newton on `|X − p|²` — a seed, not a projection.  A curve's second place is 0.
pub fn nearest(k: &[f64], theta: &[f64], p: [f64; 3], domain: [[f64; 2]; 2]) -> Option<[f64; 2]> {
    let (_, _, flat) = parts(k)?;
    let nf = view(flat)?.nf;
    let grid = |i: usize, n: usize| -> Vec<f64> {
        if i >= nf {
            return vec![0.0];
        }
        let [a, b] = domain[i];
        (0..=n).map(|j| a + (b - a) * j as f64 / n as f64).collect()
    };
    let mut best: Option<([f64; 2], f64)> = None;
    for &u in &grid(0, 24) {
        for &s in &grid(1, 12) {
            let Some(r) = at(k, theta, [u, s], Need::Point) else { continue };
            let d = crate::space::distance(r.position, p);
            if best.is_none_or(|(_, b)| d < b) {
                best = Some(([u, s], d));
            }
        }
    }
    let (mut place, _) = best?;
    for _ in 0..30 {
        let r = at(k, theta, place, Need::Tangent)?;
        let e = crate::space::sub(r.position, p);
        // the normal equations in the formals
        let g: Vec<f64> = (0..nf).map(|a| crate::space::dot(r.d[a], e)).collect();
        let h = |a: usize, b: usize| crate::space::dot(r.d[a], r.d[b]);
        let step = if nf == 1 {
            let d = h(0, 0);
            if d <= 0.0 { break }
            [-g[0] / d, 0.0]
        } else {
            let det = h(0, 0) * h(1, 1) - h(0, 1) * h(0, 1);
            if det.abs() <= 1e-300 { break }
            [-(h(1, 1) * g[0] - h(0, 1) * g[1]) / det, -(h(0, 0) * g[1] - h(0, 1) * g[0]) / det]
        };
        place = [place[0] + step[0], place[1] + step[1]];
        if step[0].abs() + step[1].abs() < 1e-13 {
            break;
        }
    }
    place.iter().all(|x| x.is_finite()).then_some(place)
}

/// Whether the body is **regular** at a place: its derivatives in the formals independent
/// (`rank [C_φ, C_s] = 2` on a surface, `C_u ≠ 0` on a curve), read as the sine between them —
/// the undercut check of a flank seen from its member (#326 R5).  `None` where it cannot be read.
pub fn regularity(k: &[f64], theta: &[f64], place: [f64; 2]) -> Option<f64> {
    let r = at(k, theta, place, Need::Tangent)?;
    match r.normal {
        Some(_) => {
            let (a, b) = (r.d[0], r.d[1]);
            let (na, nb) = (crate::space::norm(a), crate::space::norm(b));
            Some(crate::space::norm(crate::space::cross(a, b)) / (na * nb).max(1e-300))
        }
        None => Some(crate::space::norm(r.d[0])),
    }
}

/* -- reading the sketch ---------------------------------------------------------------------- */

use crate::model::{EntKind, Sketch, TracedE};

/// The points whose lifts a traced body's columns read: its points, its lines' ends, and every
/// point its motions' axes read.
pub fn lifted(sk: &Sketch, t: &TracedE) -> Vec<usize> {
    t.args.iter().flat_map(|e| match e.kind {
        EntKind::Point => vec![e.i()],
        EntKind::Line => {
            let l = &sk.lines[e.i()];
            vec![l.p1 as usize, l.p2 as usize]
        }
        EntKind::Motion => held::Moving::of_any(sk, e.i()).map(|m| m.lifted()).unwrap_or_default(),
        _ => Vec::new(),
    }).collect()
}

/// A traced body's columns past a contact's place, every point's lift minted: `θ` in the
/// variable table's order.
pub fn columns(sk: &Sketch, t: &TracedE) -> Vec<u32> {
    let lift = |p: usize| -> Vec<u32> {
        let k = sk.lift_of(p).expect("a traced body's points are lifted at the add");
        sk.lifts[k].x.to_vec()
    };
    t.args.iter().flat_map(|e| match e.kind {
        EntKind::Point => lift(e.i()),
        EntKind::Line => {
            let l = &sk.lines[e.i()];
            [lift(l.p1 as usize), lift(l.p2 as usize)].concat()
        }
        EntKind::Motion => {
            held::Moving::of_any(sk, e.i()).map(|m| m.columns(sk)).unwrap_or_default()
        }
        _ => Vec::new(),
    }).collect()
}

/// The columns' values as the drawing stands, a point not lifted yet read where it is.
pub fn theta(sk: &Sketch, t: &TracedE) -> Vec<f64> {
    t.args.iter().flat_map(|e| match e.kind {
        EntKind::Point => sk.lifted(e.i()).to_vec(),
        EntKind::Line => {
            let l = &sk.lines[e.i()];
            [sk.lifted(l.p1 as usize), sk.lifted(l.p2 as usize)].concat()
        }
        EntKind::Motion => {
            held::Moving::of_any(sk, e.i()).map(|m| m.values(sk)).unwrap_or_default()
        }
        _ => Vec::new(),
    }).collect()
}

/// A contact's constants over traced body `t`.
pub fn consts_of(t: &TracedE) -> Vec<f64> {
    consts(&t.values, t.home, &t.body)
}

/// Traced body `i` at a place, as the drawing stands.
pub fn reading(sk: &Sketch, i: usize, place: [f64; 2], need: Need) -> Option<Reading> {
    let t = sk.traced.get(i)?;
    at(&consts_of(t), &theta(sk, t), place, need)
}

/// How far traced body `i`, where it states a rate (#339 D2), is integrated across its interval
/// from its home: `None` for any other body, else each way's refusal (`locus::rate_reach`).
pub fn rate_reach(sk: &Sketch, i: usize) -> Option<[crate::rates::Reached; 2]> {
    let t = sk.traced.get(i)?;
    let v = view(&t.body.flat).filter(|v| !v.rates.is_empty())?;
    let k = consts_of(t);
    let (values, home, flat) = parts(&k)?;
    let mut outer = vec![home[0]];
    outer.extend(theta(sk, t));
    outer.extend_from_slice(values);
    Some(crate::rates::reaches(&v.rate_body(), flat, &outer, home[0], v.domain))
}

/// Where point `p` (in space) sits nearest on traced body `i`, as the drawing stands.
pub fn place_of(sk: &Sketch, i: usize, p: [f64; 3]) -> Option<[f64; 2]> {
    let t = sk.traced.get(i)?;
    nearest(&consts_of(t), &theta(sk, t), p, t.domain)
}

/* -- a traced surface as a meshing's flank --------------------------------------------------- */

/// What marks a meshing flank's constants as a traced surface's (`envelope::meshing`): where a
/// held flank's carry its source's code.
pub const FLANK: f64 = 100.0;

/// A meshing flank's constants over traced surface `t`: `[0, FLANK, domain×4, consts…]`.
pub fn flank_consts(t: &TracedE) -> Vec<f64> {
    let d = t.domain;
    [vec![0.0, FLANK, d[0][0], d[0][1], d[1][0], d[1][1]], consts_of(t)].concat()
}

/// A traced flank's constants read back: its domain and a contact's constants.
pub(crate) fn flank_parts(k: &[f64]) -> Option<([[f64; 2]; 2], &[f64])> {
    if k.len() < 6 || k[0] != 0.0 || k[1] != FLANK {
        return None;
    }
    let rest = &k[6..];
    let (_, _, flat) = parts(rest)?;
    (view(flat)?.nf == 2).then_some(([[k[2], k[3]], [k[4], k[5]]], rest))
}

/// How many columns a traced flank reads past its place.
pub(crate) fn flank_width(k: &[f64]) -> usize {
    flank_parts(k).and_then(|(_, c)| parts(c)).and_then(|(_, _, f)| view(f))
        .map_or(0, |v| v.n_theta)
}

/// What a meshing reads of a traced flank over its columns `c` — `[u, s, t, θ…]`, the roll `t`
/// held at none — as numbers of any kind: its point, its unit normal `C_u × C_s` and `C_u`, each
/// exact in every column where the numbers carry derivatives (the frame's, linearised: a `Dual`'s
/// first order is all a meshing's Jacobian reads).  `None` where the body cannot be solved there.
pub(crate) fn seen<T: Num>(k: &[f64], c: &[T]) -> Option<([T; 3], [T; 3], [T; 3])> {
    let (values, home, flat) = parts(k)?;
    let v = view(flat)?;
    if v.nf != 2 || c.len() != 3 + v.n_theta {
        return None;
    }
    let vals: Vec<f64> = c.iter().map(|x| x.val()).collect();
    let mut outer = vec![vals[0], vals[1]];
    outer.extend_from_slice(&vals[3..]);
    outer.extend_from_slice(values);
    let r = read(flat, &outer, &home, Need::Tangent, c.iter().any(|x| x.moves()))?;
    // the place's two columns, then the body's, past the held roll
    let f = r.linear(|q| {
        let at = if q < PLACE { q } else { q + 1 };
        Some(c[at] - T::cst(vals[at]))
    });
    Some((f.x, unit(cross(f.d[0], f.d[1])), f.d[0]))
}
