//! **The method of lines** (#339 D4): a body traced over two formals whose relations read a rate
//! along either — a first-order PDE — marched along the first and sampled along the second.
//!
//! `S := Front(c, a: 40mm, b: 25mm).p over (d in (0, 15), u in (-60deg, 60deg))` runs over `d`,
//! **marched**, and `u`, **sampled**: the interval of `u` is cut into pieces (`Grid`), each with
//! the six Gauss–Lobatto nodes of #243 L2's walls, a joint node shared by the two pieces it ends.
//! The body's coordinates at the nodes are one state, integrated in `d` (`ode::march`), and a rate
//! along `u` is the derivative of the quintic through a piece's nodes — at a joint, the mean of
//! the two pieces'.  `initially { … }` is the **Cauchy data**: what holds at the home of `d` for
//! every `u`.  Between nodes the surface is that quintic, so its derivatives in `u` are its own.
//!
//! **What is integrated and what is solved** (D0's finding 3).  A row that reads a rate along `u`
//! is algebraic at each step — the rate is the nodes' — but differentiated along `d` it would read
//! `x_{ud} = D x_d`, coupling every node of a piece into one system.  So the body's coordinates are
//! split by what its rate rows read (`Plan`):
//!
//! - **the coordinates the rates move** (`I`): those whose rate along `d` a rate row reads.  They
//!   are the state, integrated, their rate along `u` read off the nodes;
//! - **the coordinates the rows place** (`A`): the rest, solved at each node and each step from
//!   where the last left them (a branch, as a trace's), by the rows that read them — square in
//!   them, and the only rows that may read a rate along `u` — given `I` and its rates along `u`.
//!
//! The rates at a node are then one small system, as a curve's are (`rates.rs`): the rows over
//! `I` alone differentiated along `d`, and the rate rows, `M = [J_I; ∂R/∂I′]` with each row scaled
//! to unit length, a pivot under `1e-10` a **rank loss**, refused there.  Nothing couples two
//! nodes but the derivative along `u`, so the flow's Jacobian is banded by piece and never formed:
//! the variational equations are read along `Φ`'s columns (`ode::Rate::along`), each a directional
//! derivative at every node — the placed coordinates' by the implicit function theorem, the rows'
//! second derivatives by polarisation over `taylor::Jet`s, exactly as a curve's.
//!
//! **The surface's regularity** `rank [x_d, x_u] = 2` is checked at every node at every
//! evaluation: where the area `|x_d × x_u|` falls below `FOLD` of the node's at the home the
//! characteristics are crossing, a **fold** — an undercut, for a flank — refused there.
//!
//! **A march is held to the march on twice the pieces** (D0's decision 4): the spurious modes of
//! a discretisation in `u` grow faster than the front does, so a representation error is
//! amplified, and no solve between nodes is there to check against.  `check` says how far apart
//! the two are; the gate holds it to bound the error.

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::kernels::KERNELS;
use crate::ode;
use crate::rates::{factor, n_res, Factored, Row};
use crate::taylor::{self, Jet};
use std::cell::RefCell;
use std::rc::Rc;

/// The pieces a sampled formal's interval is cut into, unless a caller asks for another count.
pub const PIECES: usize = 32;
/// Below this part of a node's area `|x_d × x_u|` at the home, the surface is folding there.
pub const FOLD: f64 = 1e-2;
/// The most nodes a march carries.
pub const MAX_NODES: usize = 2048;
/// The most columns one row reads.
const COLS: usize = 32;
const A_NEWTON: usize = 24;
const RATE_NEWTON: usize = 8;
const HOME_NEWTON: usize = 40;
const RESTARTS: usize = 8;

/* -- the body ------------------------------------------------------------------------------- */

/// A body over two formals that states a rate, as `traced::View` reads it.  The variable table at
/// a node is `[outer] ++ q ++ w ++ q_d ++ q_u`: `outer` is `[d, u, θ…, values…]`, `q` the body's
/// coordinates, `w` the numbers it reads over the formals (a tape each), `q_d` and `q_u` the rates
/// of `q` along the marched formal and along the sampled one.
pub struct Body<'a> {
    pub n_outer: usize,
    pub n_theta: usize,
    pub n_q: usize,
    /// The traced point's first coordinate in `q`.
    pub traced: usize,
    pub w: Vec<&'a [f64]>,
    pub seeds: Vec<&'a [f64]>,
    pub alg: Vec<Row<'a>>,
    pub rate: Vec<Row<'a>>,
    pub init: Vec<Row<'a>>,
    /// The two formals' intervals.
    pub domain: [[f64; 2]; 2],
    /// What a rate along each formal is per: its derivative along the formal's number times this
    /// (`180/π` along an `Angle`, whose number is degrees: a rate along a turn is per radian).
    pub per: [f64; 2],
    pub pieces: usize,
}

/// The split of a body's coordinates (the module's docs): those the rates move (`i`), those its
/// rows place (`a`), the algebraic rows over `I` alone and those that place `A`, and where the
/// traced point stands in `I`.
#[derive(Clone, Debug)]
pub struct Plan<'a> {
    pub i: Vec<usize>,
    pub a: Vec<usize>,
    pub alg_i: Vec<Row<'a>>,
    pub alg_a: Vec<Row<'a>>,
    pub t: usize,
    /// Each rate row that is a `parallel` in space: the line (its first column) whose direction
    /// its two rows are read across, at every node (`Body::across`).
    pub across: Vec<Option<usize>>,
}

/// Why a march stopped: its rates lost rank (or the body's rows would not place what they place)
/// at a node, the surface folded there, or the integrator ran past its bound of steps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Why {
    Rank { sigma: f64 },
    Fold { sigma: f64 },
    Steps,
}

impl<'a> Body<'a> {
    fn r_d(&self) -> usize {
        self.n_outer + self.n_q + self.w.len()
    }

    fn r_u(&self) -> usize {
        self.r_d() + self.n_q
    }

    fn width(&self) -> usize {
        self.r_u() + self.n_q
    }

    /// Whether `row` reads slot range `[from, from + n)` at `k`'s place for some `k` in `set`.
    fn reads(row: &Row, from: usize, set: &[usize]) -> bool {
        row.cols.iter().any(|&c| c >= from && set.contains(&(c - from)))
    }

    /// The split, every row checked against it (the module's docs) — an `Err` saying what is
    /// wrong.  The lowering names the points; an encoding read back is checked here again.
    pub fn plan(&self) -> Result<Plan<'a>, String> {
        let (n, rd, ru) = (self.n_q, self.r_d(), self.r_u());
        if n == 0 || n > crate::rates::MAX_Q || self.traced + 3 > n
            || self.n_theta + 2 > self.n_outer || self.pieces == 0
            || self.pieces * 5 + 1 > MAX_NODES || self.seeds.len() != n {
            return Err("a body marched by the method of lines places between 1 and 48 numbers \
                        over a body of its shape".into());
        }
        if !self.domain.iter().flatten().all(|v| v.is_finite())
            || self.domain[1][0] >= self.domain[1][1]
            || !self.per.iter().all(|p| p.is_finite() && *p > 0.0) {
            return Err("a marched body runs over two intervals".into());
        }
        let width = self.width();
        for rows in [&self.alg, &self.rate, &self.init] {
            for r in rows.iter() {
                let kn = KERNELS.get(r.kid).ok_or("no such kernel")?;
                if kn.n_par != r.cols.len() || kn.n_const != r.consts.len() || kn.n_par > COLS
                    || kn.n_res > 8 || r.cols.iter().any(|&c| c >= width) {
                    return Err("rate row shape mismatch".into());
                }
                if !taylor::has_form(r.kid) {
                    return Err(format!("a body that states a rate reads its rows' derivatives \
                                        exactly, and `{}` has no Taylor form", kn.name));
                }
            }
        }
        let all: Vec<usize> = (0..n).collect();
        let i: Vec<usize> = all.iter().copied()
            .filter(|&k| self.rate.iter().any(|r| r.cols.contains(&(rd + k)))).collect();
        let a: Vec<usize> = all.iter().copied().filter(|k| !i.contains(k)).collect();
        if i.is_empty() {
            return Err("a body over two formals that states a rate states one along the first: \
                        the formal it is marched along".into());
        }
        if self.alg.iter().chain(&self.init).any(|r| Self::reads(r, rd, &all)) {
            return Err("only a rate row reads a rate along the marched formal".into());
        }
        if self.alg.iter().chain(&self.rate).any(|r| Self::reads(r, ru, &a)) {
            return Err("a rate along the sampled formal is of a point the rates move".into());
        }
        if self.init.iter().any(|r| Self::reads(r, ru, &all) || Self::reads(r, self.n_outer, &a)) {
            return Err("`initially { … }` places the points the rates move, and reads no rate"
                .into());
        }
        let (alg_a, alg_i): (Vec<usize>, Vec<usize>) =
            (0..self.alg.len()).partition(|&k| Self::reads(&self.alg[k], self.n_outer, &a));
        if alg_i.iter().any(|&k| Self::reads(&self.alg[k], ru, &all)) {
            return Err("a row that reads a rate along the sampled formal places a point the \
                        rates do not move".into());
        }
        let count =
            |ks: &[usize]| ks.iter().map(|&k| KERNELS[self.alg[k].kid].n_res).sum::<usize>();
        if count(&alg_a) != a.len() {
            return Err(format!("the body's rows must place what its rates do not move: {} \
                numbers against {} rows", a.len(), count(&alg_a)));
        }
        if count(&alg_i) + n_res(&self.rate) < i.len() {
            return Err(format!("a body that states a rate must determine its rates: {} unknowns \
                against {} rows and {} rate rows", i.len(), count(&alg_i), n_res(&self.rate)));
        }
        if count(&alg_i) + n_res(&self.init) < i.len() {
            return Err(format!("a marched body starts where `initially {{ … }}` says: {} \
                unknowns against {} rows and {} initial rows", i.len(), count(&alg_i),
                n_res(&self.init)));
        }
        let t = i.iter().position(|&k| k == self.traced)
            .filter(|&t| i.get(t + 2) == Some(&(self.traced + 2)))
            .ok_or("a marched surface traces a point its rates move")?;
        let pick = |ks: &[usize]| ks.iter().map(|&k| self.alg[k].clone()).collect();
        // a `parallel` is read across the line that is no rate along the marched formal
        let across = self.rate.iter().map(|r| {
            (r.kid == crate::kernels::K::Parallel3 as usize).then(|| {
                let marched =
                    |at: usize| r.cols[at..at + 6].iter().any(|&c| (rd..rd + n).contains(&c));
                if marched(0) { 6 } else { 0 }
            })
        }).collect();
        Ok(Plan { i, a, alg_i: pick(&alg_i), alg_a: pick(&alg_a), t, across })
    }

    /// The variable table at a node over plain numbers.
    fn table(&self, outer: &[f64], q: &[f64], qd: &[f64], qu: &[f64]) -> Option<Vec<f64>> {
        let mut z = Vec::with_capacity(self.width());
        z.extend_from_slice(&outer[..self.n_outer]);
        z.extend_from_slice(q);
        for t in &self.w {
            z.push(crate::tape::eval_num::<f64>(t, &outer[..self.n_outer])?);
        }
        z.extend_from_slice(qd);
        z.extend_from_slice(qu);
        Some(z)
    }

    /// The variable table over jets.
    fn jet_table(&self, outer: &[Jet], q: &[Jet], qd: &[Jet], qu: &[Jet]) -> Option<Vec<Jet>> {
        let mut z = Vec::with_capacity(self.width());
        z.extend_from_slice(outer);
        z.extend_from_slice(q);
        for t in &self.w {
            z.push(crate::tape::eval_num::<Jet>(t, outer)?);
        }
        z.extend_from_slice(qd);
        z.extend_from_slice(qu);
        Some(z)
    }
}

/// Rows over plain numbers, into `out`.
fn residual(rows: &[Row], z: &[f64], out: &mut Vec<f64>) {
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

/// Rows' Jacobian in the slots `slots` (any order), row-major into `out` (`n_res(rows) × n`).
fn jacobian(rows: &[Row], z: &[f64], slots: &[usize], out: &mut Vec<f64>) {
    let n = slots.len();
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
        for (c, &col) in r.cols.iter().enumerate() {
            if let Some(k) = slots.iter().position(|&s| s == col) {
                for t in 0..kn.n_res {
                    out[(row + t) * n + k] += jr[t * kn.n_par + c];
                }
            }
        }
        row += kn.n_res;
    }
}

/// Rows over jets, into `out`; `false` where a row has no form.
fn jets(rows: &[Row], z: &[Jet], out: &mut Vec<Jet>) -> bool {
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

fn absmax(v: &[f64]) -> f64 {
    v.iter().fold(0.0f64, |a, x| a.max(x.abs()))
}

/* -- the sampled formal ------------------------------------------------------------------- */

/// The six Gauss–Lobatto points of degree five on `[−1, 1]`, a piece's nodes.
const NODES: [f64; 6] = crate::brep::family::NODES;

/// The Lagrange basis through `NODES` at `t`, and its first and second derivatives in `t`.
fn basis(t: f64) -> [[f64; 6]; 3] {
    let mut out = [[0.0; 6]; 3];
    for i in 0..6 {
        // the polynomial Π_{j≠i} (t − x_j)/(x_i − x_j), by its coefficients
        let mut c = [0.0f64; 6];
        c[0] = 1.0;
        let mut deg = 0;
        let mut den = 1.0;
        for (j, &xj) in NODES.iter().enumerate() {
            if j == i {
                continue;
            }
            for k in (0..=deg).rev() {
                c[k + 1] += c[k];
                c[k] *= -xj;
            }
            deg += 1;
            den *= NODES[i] - xj;
        }
        let (mut p, mut d1, mut d2) = (0.0, 0.0, 0.0);
        for k in (0..6).rev() {
            d2 = d2 * t + 2.0 * d1;
            d1 = d1 * t + p;
            p = p * t + c[k];
        }
        out[0][i] = p / den;
        out[1][i] = d1 / den;
        out[2][i] = d2 / den;
    }
    out
}

/// The sampled formal's interval cut into equal pieces of six nodes: open, or — a whole turn of
/// an angle whose ends meet — closed, its last node its first.
#[derive(Clone, Debug, PartialEq)]
pub struct Grid {
    pub pieces: usize,
    pub closed: bool,
    pub span: [f64; 2],
    /// Each node's place along the formal.
    pub nodes: Vec<f64>,
    d6: [[f64; 6]; 6],
}

impl Grid {
    pub fn new(pieces: usize, span: [f64; 2], closed: bool) -> Grid {
        let h = (span[1] - span[0]) / pieces as f64;
        let mut nodes = Vec::with_capacity(5 * pieces + 1);
        for p in 0..pieces {
            for (i, &x) in NODES.iter().enumerate() {
                if p > 0 && i == 0 {
                    continue;
                }
                nodes.push(span[0] + h * (p as f64 + (x + 1.0) / 2.0));
            }
        }
        if closed {
            nodes.pop();
        }
        let d6 = std::array::from_fn(|i| basis(NODES[i])[1]);
        Grid { pieces, closed, span, nodes, d6 }
    }

    pub fn n(&self) -> usize {
        self.nodes.len()
    }

    fn h(&self) -> f64 {
        (self.span[1] - self.span[0]) / self.pieces as f64
    }

    /// The node at a piece's `i`-th point.
    fn index(&self, p: usize, i: usize) -> usize {
        (5 * p + i) % if self.closed { self.n() } else { usize::MAX }
    }

    /// The derivative along the formal of `vals` (`n × width`, node-major) at every node: the
    /// quintic's through each piece's nodes, the mean of the two at a joint.
    fn derive(&self, vals: &[f64], width: usize) -> Vec<f64> {
        let n = self.n();
        let mut out = vec![0.0; n * width];
        let mut count = vec![0u8; n];
        let k = 2.0 / self.h();
        for p in 0..self.pieces {
            let idx: [usize; 6] = std::array::from_fn(|i| self.index(p, i));
            for i in 0..6 {
                let j = idx[i];
                for c in 0..width {
                    let mut acc = 0.0;
                    for (m, &jm) in idx.iter().enumerate() {
                        acc += self.d6[i][m] * vals[jm * width + c];
                    }
                    out[j * width + c] += k * acc;
                }
                count[j] += 1;
            }
        }
        for (j, &c) in count.iter().enumerate() {
            if c > 1 {
                for v in &mut out[j * width..(j + 1) * width] {
                    *v /= c as f64;
                }
            }
        }
        out
    }

    /// Which piece reads place `s`, its nodes, and the basis there and its derivatives along the
    /// formal (past an open end, the end piece's polynomial; a closed formal wrapped).
    fn at(&self, s: f64) -> ([usize; 6], [[f64; 6]; 3]) {
        let (lo, hi) = (self.span[0], self.span[1]);
        let h = self.h();
        let s = if self.closed { lo + (s - lo).rem_euclid(hi - lo) } else { s };
        let p = (((s - lo) / h).floor().max(0.0) as usize).min(self.pieces - 1);
        let t = 2.0 * (s - lo - p as f64 * h) / h - 1.0;
        let mut b = basis(t);
        let k = 2.0 / h;
        for v in &mut b[1] {
            *v *= k;
        }
        for v in &mut b[2] {
            *v *= k * k;
        }
        (std::array::from_fn(|i| self.index(p, i)), b)
    }
}

/* -- one node --------------------------------------------------------------------------------- */

/// The body solved at one node: its variable table, the rates of `I` (per the formal's unit) and
/// the factorisations a directional derivative reuses.
struct At {
    z: Vec<f64>,
    v: Vec<f64>,
    /// The rate rows' constants at this node (`Body::rate_rows`).
    consts: Vec<Vec<f64>>,
}

/// A direction at a node: along the marched formal, `I`, `I`'s rates along the sampled formal
/// (per its unit) and the columns.
struct Dir<'d> {
    d: f64,
    q: &'d [f64],
    s: &'d [f64],
    theta: &'d [f64],
}

impl<'a> Body<'a> {
    /// The body at one node: `outer` at it, `q_i` and `s_i` the coordinates the rates move and
    /// their rates along the sampled formal (per its unit), `a0` and `v0` where the placed
    /// coordinates and the rates were last found.  `None` where its rows will not place `A` or its
    /// rates lose rank.  `strict` asks that rows past the rates' count agree, as at a home or a
    /// reading.
    fn node(&self, plan: &Plan<'a>, outer: &[f64], q_i: &[f64], s_i: &[f64], a0: &[f64], v0: &[f64],
            strict: bool) -> Option<At> {
        let n = self.n_q;
        let (ni, na) = (plan.i.len(), plan.a.len());
        let mut q = vec![0.0; n];
        let mut qu = vec![0.0; n];
        for (k, &ix) in plan.i.iter().enumerate() {
            q[ix] = q_i[k];
            qu[ix] = s_i[k];
        }
        for (k, &ix) in plan.a.iter().enumerate() {
            q[ix] = a0[k];
        }
        let mut z = self.table(outer, &q, &vec![0.0; n], &qu)?;
        let size = absmax(&z[..self.r_d()]).max(1.0);
        // the placed coordinates, by Newton from where they were, judged by their residual; the
        // factorisations a derivative reads are made at the root, where one is asked (`lin`)
        if na > 0 {
            let alg_a = &plan.alg_a;
            let slots: Vec<usize> = plan.a.iter().map(|&k| self.n_outer + k).collect();
            let (mut r, mut j, mut piv) = (Vec::new(), Vec::new(), Vec::new());
            residual(alg_a, &z, &mut r);
            let mut now = absmax(&r);
            for it in 0.. {
                if !now.is_finite() || it > A_NEWTON {
                    return None;
                }
                if now <= 1e-13 * size {
                    break;
                }
                jacobian(alg_a, &z, &slots, &mut j);
                if !crate::linalg::lu_factor(na, &mut j, &mut piv) {
                    return None;
                }
                let mut step: Vec<f64> = r.iter().map(|x| -x).collect();
                crate::linalg::lu_apply(na, &j, &piv, &mut step);
                let mut lambda = 1.0;
                loop {
                    for (k, s) in slots.iter().zip(&step) {
                        z[*k] += lambda * s;
                    }
                    residual(alg_a, &z, &mut r);
                    let next = absmax(&r);
                    if next.is_finite() && (next < now || lambda < 1e-3) {
                        now = next;
                        break;
                    }
                    for (k, s) in slots.iter().zip(&step) {
                        z[*k] -= lambda * s;
                    }
                    lambda *= 0.5;
                }
            }
        }
        // the rates: the rows over `I` differentiated along the marched formal, `J_I v + per·r_d
        // = 0`, and the rate rows, by Newton from the rates before
        let slots_d: Vec<usize> = plan.i.iter().map(|&k| self.r_d() + k).collect();
        let (b, ji) = self.over_moved(plan, &z, &qu)?;
        let n_alg = b.len();
        let mut v = v0.to_vec();
        let consts = self.across(plan, &z);
        let rate = Self::rate_rows(&self.rate, &consts);
        let (mut rr, mut jr) = (Vec::new(), Vec::new());
        for _ in 0..RATE_NEWTON {
            for (s, x) in slots_d.iter().zip(&v) {
                z[*s] = *x;
            }
            residual(&rate, &z, &mut rr);
            jacobian(&rate, &z, &slots_d, &mut jr);
            let mut g = Vec::with_capacity(n_alg + rr.len());
            for r in 0..n_alg {
                g.push(-(b[r] + (0..ni).map(|c| ji[r * ni + c] * v[c]).sum::<f64>()));
            }
            g.extend(rr.iter().map(|x| -x));
            let mut mat = ji.clone();
            mat.extend_from_slice(&jr);
            let m = factor(n_alg + rr.len(), ni, mat)?;
            m.solve(&mut g);
            let big = v.iter().fold(1.0f64, |a, x| a.max(x.abs()));
            let step = absmax(&g[..ni]);
            if !step.is_finite() {
                return None;
            }
            for (x, d) in v.iter_mut().zip(&g[..ni]) {
                *x += d;
            }
            // Newton's convergence is quadratic: a step this small leaves the next at rounding
            if step <= 1e-8 * big {
                if strict && absmax(&g[ni..]) > 1e-8 * big {
                    return None;
                }
                for (s, x) in slots_d.iter().zip(&v) {
                    z[*s] = *x;
                }
                return Some(At { z, v, consts });
            }
        }
        None
    }

    /// The rows over `I` alone at a table: `per·∂r/∂d` with the coordinates standing (their first
    /// order along the marched formal, the tapes' rates in it) and `∂r/∂I`.
    fn over_moved(&self, plan: &Plan<'a>, z: &[f64], qu: &[f64]) -> Option<(Vec<f64>, Vec<f64>)> {
        let alg_i = &plan.alg_i;
        if alg_i.is_empty() {
            return Some((Vec::new(), Vec::new()));
        }
        let n = self.n_q;
        let along = self.outer_jets(z, |k| if k == 0 { 1.0 } else { 0.0 });
        let qj: Vec<Jet> = z[self.n_outer..self.n_outer + n].iter().map(|&x| Jet::constant(x))
            .collect();
        let zero = vec![Jet::constant(0.0); n];
        let quj: Vec<Jet> = qu.iter().map(|&x| Jet::constant(x)).collect();
        let zj = self.jet_table(&along, &qj, &zero, &quj)?;
        let mut rj = Vec::new();
        if !jets(alg_i, &zj, &mut rj) {
            return None;
        }
        let b = rj.iter().map(|j| self.per[0] * j.0[1]).collect();
        let slots_i: Vec<usize> = plan.i.iter().map(|&k| self.n_outer + k).collect();
        let mut ji = Vec::new();
        jacobian(alg_i, z, &slots_i, &mut ji);
        Some((b, ji))
    }

    /// The rate rows' constants at a node: their own, but a `parallel`'s two directions across
    /// the line it holds the rate to as that line stands there.  Stated once, across a line of no
    /// length in the scratch sketch, they would lose rank wherever the line turned square to the
    /// frame; across it, they hold exactly where the cross product vanishes, so their own
    /// derivative, which multiplies it, is nothing at a solution.
    fn across(&self, plan: &Plan, z: &[f64]) -> Vec<Vec<f64>> {
        self.rate.iter().zip(&plan.across).map(|(r, at)| match at {
            Some(at) => {
                let p = |k: usize| -> [f64; 3] { std::array::from_fn(|c| z[r.cols[at + k + c]]) };
                let (e1, e2) = crate::space::across(crate::space::sub(p(3), p(0)));
                [e1, e2].concat()
            }
            None => r.consts.to_vec(),
        }).collect()
    }

    /// The rate rows over constants of a node's.
    fn rate_rows<'c>(rows: &[Row], consts: &'c [Vec<f64>]) -> Vec<Row<'c>> {
        rows.iter().zip(consts).map(|(r, k)| Row { kid: r.kid, cols: r.cols.clone(), consts: k })
            .collect()
    }

    /// The outer variables of a table as jets, moving along `dir`.
    fn outer_jets(&self, z: &[f64], dir: impl Fn(usize) -> f64) -> Vec<Jet> {
        (0..self.n_outer).map(|k| Jet::from(&[z[k], dir(k)])).collect()
    }

    /// A node's first-order picture (`Lin`), made once: the placed rows' and the rate rows'
    /// Jacobians over the whole variable table, and each tape's derivative in each outer variable
    /// that moves (the place along the sampled formal never does).
    fn lin(&self, plan: &Plan<'a>, at: &At) -> Option<Lin> {
        let w = self.width();
        let full = |rows: &[Row]| -> Vec<f64> {
            let mut out = vec![0.0; n_res(rows) * w];
            let mut row = 0;
            let mut x = [0.0f64; COLS];
            let mut jr = [0.0f64; COLS * 8];
            for r in rows {
                let kn = &KERNELS[r.kid];
                for (x, &c) in x.iter_mut().zip(&r.cols) {
                    *x = at.z[c];
                }
                let jr = &mut jr[..kn.n_res * kn.n_par];
                jr.iter_mut().for_each(|v| *v = 0.0);
                kn.jac_into(&x[..kn.n_par], r.consts, jr);
                for (c, &col) in r.cols.iter().enumerate() {
                    for t in 0..kn.n_res {
                        out[(row + t) * w + col] += jr[t * kn.n_par + c];
                    }
                }
                row += kn.n_res;
            }
            out
        };
        let no = self.n_outer;
        let mut dw = vec![0.0; self.w.len() * no];
        for o in (0..no).filter(|&o| o != 1) {
            let oj = self.outer_jets(&at.z, |k| if k == o { 1.0 } else { 0.0 });
            for (k, t) in self.w.iter().enumerate() {
                if crate::tape::reads_var(t, o) {
                    dw[k * no + o] =
                        crate::tape::eval_num::<Jet>(t, &oj).map_or(f64::NAN, |j| j.0[1]);
                }
            }
        }
        let (ja, jr) = (full(&plan.alg_a), full(&Self::rate_rows(&self.rate, &at.consts)));
        // `∂r/∂A` of the placed rows, and the rate system `M`, at the root
        let (na, ni) = (plan.a.len(), plan.i.len());
        let mut lu_a: Vec<f64> = (0..na * na)
            .map(|k| ja[(k / na) * w + self.n_outer + plan.a[k % na]]).collect();
        let mut piv_a = Vec::new();
        let placed = na == 0 || crate::linalg::lu_factor(na, &mut lu_a, &mut piv_a);
        let qu: Vec<f64> = at.z[self.r_u()..self.r_u() + self.n_q].to_vec();
        let (_, mut mat) = self.over_moved(plan, &at.z, &qu)?;
        let rows = mat.len() / ni.max(1) + jr.len() / w;
        mat.extend((0..jr.len() / w).flat_map(|r| {
            plan.i.iter().map(move |&k| (r, k))
        }).map(|(r, k)| jr[r * w + self.r_d() + k]));
        let m = factor(rows, ni, mat).filter(|_| placed)?;
        Some(Lin { ja, jr, dw, lu_a, piv_a, m })
    }

    /// The derivative of the rates at a node (`at`, the rates per the unit `f = v/per`) along a
    /// direction of the state and the columns (`dir`): the placed coordinates' by the implicit
    /// function theorem, the rows over `I` by polarisation, the rate rows' first order — each
    /// with the rates standing.  Per the formal's unit, as `v` is.
    fn moved(&self, plan: &Plan<'a>, at: &At, lin: &Lin, dir: &Dir) -> Option<Vec<f64>> {
        let n = self.n_q;
        let (ni, na, no, nt) = (plan.i.len(), plan.a.len(), self.n_outer, self.n_theta);
        let z = &at.z;
        let w = self.width();
        // the direction over the whole table, the rates along the marched formal standing
        let mut dz = vec![0.0; w];
        dz[0] = dir.d;
        dz[2..2 + nt].copy_from_slice(dir.theta);
        for (k, &ix) in plan.i.iter().enumerate() {
            dz[no + ix] = dir.q[k];
            dz[self.r_u() + ix] = dir.s[k];
        }
        for k in 0..self.w.len() {
            dz[no + n + k] = (0..no).map(|o| lin.dw[k * no + o] * dz[o]).sum();
        }
        let times = |rows: &[f64], dz: &[f64]| -> Vec<f64> {
            rows.chunks_exact(w).map(|r| r.iter().zip(dz).map(|(a, b)| a * b).sum()).collect()
        };
        // the placed coordinates move with the rows that place them
        if na > 0 {
            let mut g: Vec<f64> = times(&lin.ja, &dz).iter().map(|x| -x).collect();
            crate::linalg::lu_apply(na, &lin.lu_a, &lin.piv_a, &mut g);
            for (k, &ix) in plan.a.iter().enumerate() {
                dz[no + ix] = g[k];
            }
        }
        let mut rhs = Vec::new();
        // the rows over `I`, `per·Dr[ẋ]` with ẋ = (d: 1, I: f): `per·D²r[ẋ, δ]`, half the
        // difference of the second orders along ẋ ± δ
        let alg_i = &plan.alg_i;
        if !alg_i.is_empty() {
            let per = self.per[0];
            let qdj: Vec<Jet> = z[self.r_d()..self.r_d() + n].iter().map(|&x| Jet::constant(x))
                .collect();
            let quj: Vec<Jet> = (0..n).map(|k| Jet::constant(z[self.r_u() + k])).collect();
            let mut pair = [Vec::new(), Vec::new()];
            for (side, c) in [1.0, -1.0].into_iter().enumerate() {
                let oj = self.outer_jets(z, |k| (if k == 0 { 1.0 } else { 0.0 }) + c * dz[k]);
                let qj: Vec<Jet> = (0..n).map(|k| {
                    let f = if plan.i.contains(&k) { z[self.r_d() + k] / per } else { 0.0 };
                    Jet::from(&[z[no + k], f + c * dz[no + k]])
                }).collect();
                let zj = self.jet_table(&oj, &qj, &qdj, &quj)?;
                let mut r = Vec::new();
                if !jets(alg_i, &zj, &mut r) {
                    return None;
                }
                pair[side] = r.iter().map(|j| j.0[2]).collect::<Vec<f64>>();
            }
            rhs.extend(pair[0].iter().zip(&pair[1]).map(|(a, b)| -per * (a - b) * 0.5));
        }
        // the rate rows, the rates standing
        rhs.extend(times(&lin.jr, &dz).iter().map(|x| -x));
        lin.m.solve(&mut rhs);
        rhs.truncate(ni);
        rhs.iter().all(|x| x.is_finite()).then_some(rhs)
    }
}

/// A node's first-order picture (`Body::lin`): the placed rows' and the rate rows' Jacobians over
/// the whole variable table (row-major), and the tapes' derivatives in the outer variables
/// (`n_w × n_outer`).
struct Lin {
    ja: Vec<f64>,
    jr: Vec<f64>,
    dw: Vec<f64>,
    lu_a: Vec<f64>,
    piv_a: Vec<usize>,
    m: Factored,
}

/* -- the home --------------------------------------------------------------------------------- */

/// Where a march starts: the grid (closed where a whole turn's ends meet), the state `[Q_I, θ]`
/// with its sensitivities to the columns where asked, the branch (each node's placed coordinates
/// and rates) and each node's area `|x_d × x_u|`.
struct Start {
    grid: Grid,
    y: Vec<f64>,
    branch: Vec<f64>,
    area: Vec<f64>,
}

impl<'a> Body<'a> {
    /// The coordinates the rates move at one node of the home: the initial rows and the rows over
    /// `I` alone, by Gauss–Newton from the seeds (least squares where they are more), and their
    /// sensitivity to each column (`|I| × n_theta`).
    fn cauchy(&self, plan: &Plan<'a>, outer: &[f64]) -> Option<(Vec<f64>, Vec<f64>)> {
        let (n, ni, nt) = (self.n_q, plan.i.len(), self.n_theta);
        let rows: Vec<Row> = self.init.iter().chain(&plan.alg_i).cloned().collect();
        let slots: Vec<usize> = plan.i.iter().map(|&k| self.n_outer + k).collect();
        let seed: Vec<f64> = self.seeds.iter()
            .map(|t| crate::tape::eval_num::<f64>(t, &outer[..self.n_outer]))
            .collect::<Option<_>>()?;
        let zero = vec![0.0; n];
        let solve = |q0: &[f64]| -> Option<Vec<f64>> {
            let mut z = self.table(outer, q0, &zero, &zero)?;
            let size = absmax(&z[..self.r_d()]).max(1.0);
            let (mut r, mut j) = (Vec::new(), Vec::new());
            residual(&rows, &z, &mut r);
            let mut now = absmax(&r);
            for _ in 0..HOME_NEWTON {
                if !now.is_finite() {
                    return None;
                }
                if now <= 1e-13 * size {
                    return Some(z);
                }
                jacobian(&rows, &z, &slots, &mut j);
                let m = factor(r.len(), ni, j.clone())?;
                let mut step: Vec<f64> = r.iter().map(|x| -x).collect();
                m.solve(&mut step);
                let mut lambda = 1.0;
                loop {
                    let mut trial = z.clone();
                    for (k, s) in slots.iter().zip(&step) {
                        trial[*k] += lambda * s;
                    }
                    residual(&rows, &trial, &mut r);
                    let next = absmax(&r);
                    if next.is_finite() && (next < now || lambda < 1e-4) {
                        z = trial;
                        now = next;
                        break;
                    }
                    lambda *= 0.5;
                }
            }
            (now <= 1e-10 * size).then_some(z)
        };
        let mut z = solve(&seed);
        if z.is_none() {
            let scale = outer[2..2 + nt].iter().chain(&seed).fold(1.0f64, |m, &x| m.max(x.abs()));
            let mut rng = crate::rng::Rng::new(0x11e5);
            for _ in 0..RESTARTS {
                let start: Vec<f64> =
                    seed.iter().map(|&x| x + rng.uniform(-scale, scale)).collect();
                z = solve(&start);
                if z.is_some() {
                    break;
                }
            }
        }
        let z = z?;
        // the implicit function theorem, a column at a time
        let mut j = Vec::new();
        jacobian(&rows, &z, &slots, &mut j);
        let m = factor(n_res(&rows), ni, j)?;
        let mut sens = vec![0.0; ni * nt];
        let qj: Vec<Jet> = z[self.n_outer..self.n_outer + n].iter().map(|&x| Jet::constant(x))
            .collect();
        let zj = vec![Jet::constant(0.0); n];
        for c in 0..nt {
            let oj = self.outer_jets(&z, |k| if k == 2 + c { 1.0 } else { 0.0 });
            let t = self.jet_table(&oj, &qj, &zj, &zj)?;
            let mut r = Vec::new();
            if !jets(&rows, &t, &mut r) {
                return None;
            }
            let mut d: Vec<f64> = r.iter().map(|j| -j.0[1]).collect();
            m.solve(&mut d);
            for k in 0..ni {
                sens[k * nt + c] = d[k];
            }
        }
        let q: Vec<f64> = plan.i.iter().map(|&k| z[self.n_outer + k]).collect();
        Some((q, sens))
    }

    /// The home at `outer` (its marched formal's place in `outer[0]`) on `pieces` pieces.
    fn start(&self, plan: &Plan<'a>, outer: &[f64], pieces: usize, sens: bool) -> Option<Start> {
        let (ni, na, nt) = (plan.i.len(), plan.a.len(), self.n_theta);
        let span = self.domain[1];
        // a whole turn of an angle is closed where its two ends' data meet
        let whole = self.per[1] != 1.0 && ((span[1] - span[0]) - 360.0).abs() < 1e-9;
        let open = Grid::new(pieces, span, false);
        let mut o = outer[..self.n_outer].to_vec();
        let mut homes = Vec::with_capacity(open.n());
        for &s in &open.nodes {
            o[1] = s;
            homes.push(self.cauchy(plan, &o)?);
        }
        let closed = whole && {
            let (a, b) = (&homes[0].0, &homes[homes.len() - 1].0);
            let size = absmax(a).max(1.0);
            a.iter().zip(b).all(|(x, y)| (x - y).abs() <= 1e-9 * size)
        };
        let grid = if closed { Grid::new(pieces, span, true) } else { open };
        homes.truncate(grid.n());
        let n_nodes = grid.n();
        let width = n_nodes * ni + nt;
        let m = if sens { nt } else { 0 };
        let mut y = vec![0.0; width * (1 + m)];
        for (j, (q, s)) in homes.iter().enumerate() {
            y[j * ni..(j + 1) * ni].copy_from_slice(q);
            if sens {
                for k in 0..ni {
                    for c in 0..nt {
                        y[width + (j * ni + k) * nt + c] = s[k * nt + c];
                    }
                }
            }
        }
        for c in 0..nt {
            y[n_nodes * ni + c] = outer[2 + c];
            if sens {
                y[width + (n_nodes * ni + c) * nt + c] = 1.0;
            }
        }
        // the placed coordinates at the home, from their seeds, and the rates there
        let su = grid.derive(&y[..n_nodes * ni], ni);
        let mut branch = vec![0.0; n_nodes * (na + ni)];
        let mut area = vec![0.0; n_nodes];
        for j in 0..n_nodes {
            o[1] = grid.nodes[j];
            let seed: Vec<f64> = plan.a.iter()
                .map(|&k| crate::tape::eval_num::<f64>(self.seeds[k], &o[..self.n_outer]))
                .collect::<Option<_>>()?;
            let s: Vec<f64> = su[j * ni..(j + 1) * ni].iter().map(|x| x * self.per[1]).collect();
            let q = &y[j * ni..(j + 1) * ni];
            let at = self.node(plan, &o, q, &s, &seed, &vec![0.0; ni], true)?;
            for (k, &ix) in plan.a.iter().enumerate() {
                branch[j * na + k] = at.z[self.n_outer + ix];
            }
            branch[n_nodes * na + j * ni..n_nodes * na + (j + 1) * ni].copy_from_slice(&at.v);
            let t = plan.t;
            let f: [f64; 3] = std::array::from_fn(|c| at.v[t + c] / self.per[0]);
            let u: [f64; 3] = std::array::from_fn(|c| su[j * ni + t + c]);
            area[j] = crate::space::norm(crate::space::cross(f, u));
            // a surface not regular at its home is no surface to march
            if !(area[j] > 0.0) {
                return None;
            }
        }
        Some(Start { grid, y, branch, area })
    }
}

/* -- the flow --------------------------------------------------------------------------------- */

/// The body's flow on a grid: the state `[Q_I (node-major), θ]`, the columns constant.
struct Flow<'b, 'a> {
    body: &'b Body<'a>,
    plan: &'b Plan<'a>,
    grid: &'b Grid,
    outer: Vec<f64>,
    branch: Vec<f64>,
    area: &'b [f64],
    strict: bool,
    why: Option<Why>,
}

/// Every node solved at one place, and each node's first-order picture once it is asked for.
struct Evaluated {
    ats: Vec<At>,
    lins: RefCell<Vec<Option<Lin>>>,
}

impl Flow<'_, '_> {
    fn eval(&mut self, u: f64, y: &[f64], f: &mut [f64]) -> Option<Evaluated> {
        let (b, plan, grid) = (self.body, self.plan, self.grid);
        let (ni, na, nt, nn) = (plan.i.len(), plan.a.len(), b.n_theta, grid.n());
        self.outer[0] = u;
        self.outer[2..2 + nt].copy_from_slice(&y[nn * ni..nn * ni + nt]);
        let su = grid.derive(&y[..nn * ni], ni);
        let mut ats = Vec::with_capacity(nn);
        let mut s = vec![0.0; ni];
        for j in 0..nn {
            self.outer[1] = grid.nodes[j];
            for (k, x) in s.iter_mut().enumerate() {
                *x = su[j * ni + k] * b.per[1];
            }
            let (a0, v0) = self.branch.split_at(nn * na);
            let at = b.node(plan, &self.outer, &y[j * ni..(j + 1) * ni], &s,
                            &a0[j * na..(j + 1) * na], &v0[j * ni..(j + 1) * ni], self.strict);
            let Some(at) = at else {
                self.why = Some(Why::Rank { sigma: grid.nodes[j] });
                return None;
            };
            for k in 0..ni {
                f[j * ni + k] = at.v[k] / b.per[0];
            }
            let t = plan.t;
            let fd: [f64; 3] = std::array::from_fn(|c| f[j * ni + t + c]);
            let fu: [f64; 3] = std::array::from_fn(|c| su[j * ni + t + c]);
            let area = crate::space::norm(crate::space::cross(fd, fu));
            if !(area >= FOLD * self.area[j]) {
                self.why = Some(Why::Fold { sigma: grid.nodes[j] });
                return None;
            }
            ats.push(at);
        }
        for (j, at) in ats.iter().enumerate() {
            for (k, &ix) in plan.a.iter().enumerate() {
                self.branch[j * na + k] = at.z[b.n_outer + ix];
            }
            self.branch[nn * na + j * ni..nn * na + (j + 1) * ni].copy_from_slice(&at.v);
        }
        f[nn * ni..nn * ni + nt].iter_mut().for_each(|x| *x = 0.0);
        let lins = RefCell::new((0..nn).map(|_| None).collect());
        f.iter().all(|x| x.is_finite()).then_some(Evaluated { ats, lins })
    }

    /// The derivative of the flow's rate at every node along a direction of the state `dq`
    /// (`I` at every node) and the columns `dtheta`, the marched formal moving at `dd`: in the
    /// formal's own number, node-major.
    fn moved(&self, ev: &Evaluated, nodes: &[usize], dd: f64, dq: &[f64], dtheta: &[f64])
        -> Option<Vec<(usize, Vec<f64>)>> {
        let (b, plan, grid) = (self.body, self.plan, self.grid);
        let ni = plan.i.len();
        let du = grid.derive(dq, ni);
        let mut out = Vec::with_capacity(nodes.len());
        let mut s = vec![0.0; ni];
        for &j in nodes {
            for (k, x) in s.iter_mut().enumerate() {
                *x = du[j * ni + k] * b.per[1];
            }
            let dir = Dir { d: dd, q: &dq[j * ni..(j + 1) * ni], s: &s, theta: dtheta };
            let mut lins = ev.lins.borrow_mut();
            if lins[j].is_none() {
                lins[j] = Some(b.lin(plan, &ev.ats[j])?);
            }
            let dv = b.moved(plan, &ev.ats[j], lins[j].as_ref()?, &dir)?;
            out.push((j, dv.iter().map(|x| x / b.per[0]).collect()));
        }
        Some(out)
    }
}

impl ode::Rate for Flow<'_, '_> {
    type Branch = Vec<f64>;

    fn width(&self) -> usize {
        self.grid.n() * self.plan.i.len() + self.body.n_theta
    }

    fn rate(&mut self, u: f64, y: &[f64], f: &mut [f64], jac: Option<&mut [f64]>) -> bool {
        // the flow's Jacobian is banded by piece and read along `Φ` (`along`), never formed
        jac.is_none() && self.eval(u, y, f).is_some()
    }

    fn along(&mut self, u: f64, y: &[f64], phi: &[f64], m: usize, f: &mut [f64],
             dphi: &mut [f64]) -> Option<bool> {
        let Some(ev) = self.eval(u, y, f) else { return Some(false) };
        let (ni, nt, nn) = (self.plan.i.len(), self.body.n_theta, self.grid.n());
        let width = nn * ni + nt;
        let all: Vec<usize> = (0..nn).collect();
        dphi.iter_mut().for_each(|x| *x = 0.0);
        let mut dq = vec![0.0; nn * ni];
        let mut dt = vec![0.0; nt];
        for c in 0..m {
            for (r, x) in dq.iter_mut().enumerate() {
                *x = phi[r * m + c];
            }
            for (r, x) in dt.iter_mut().enumerate() {
                *x = phi[(nn * ni + r) * m + c];
            }
            let Some(moved) = self.moved(&ev, &all, 0.0, &dq, &dt) else { return Some(false) };
            for (j, dv) in moved {
                for k in 0..ni {
                    dphi[(j * ni + k) * m + c] = dv[k];
                }
            }
        }
        Some(dphi[..width * m].iter().all(|x| x.is_finite()))
    }

    fn branch(&self) -> Vec<f64> {
        self.branch.clone()
    }

    fn set_branch(&mut self, b: Vec<f64>) {
        self.branch = b;
    }
}


/* -- marches ---------------------------------------------------------------------------------- */

/// One march of the body from its home on a grid: the columns it was made at, the path, each
/// node's area at the home, and — where it was refused short of where it was asked to run — the
/// refusal and why, the path then reaching the place it was refused at.
pub struct March {
    pub outer: Vec<f64>,
    pub home: f64,
    pub grid: Grid,
    pub sens: bool,
    pub path: ode::Path<Vec<f64>>,
    pub area: Vec<f64>,
    pub refused: Option<(ode::Refusal, Why)>,
    /// The state at places read, so every place along the sampled formal at one place along the
    /// marched one is read off one step.
    states: RefCell<Vec<(u64, Rc<ode::Node<Vec<f64>>>)>>,
}

impl March {
    /// Whether the march reaches `u`.
    pub fn reaches(&self, u: f64) -> bool {
        let last = self.path.nodes.last().map_or(self.home, |n| n.u);
        let (to, reached) = (u - self.home, last - self.home);
        to * reached >= 0.0 && to.abs() <= reached.abs()
    }

    /// The last place the march reached.
    pub fn last(&self) -> f64 {
        self.path.nodes.last().map_or(self.home, |n| n.u)
    }
}

/// A march as remembered: made, or refused (`None` where it could not start).
pub type Marched = Result<Rc<March>, Option<(ode::Refusal, Why)>>;

impl<'a> Body<'a> {
    /// The flow from the home at `outer` (`outer[0]` the marched formal's home) across `span`
    /// (either sign) on `pieces` pieces, carrying the sensitivities to the columns where `sens`:
    /// refused part way, as far as it reached, the refusal and why with it.  `None` where it
    /// could not start.
    pub fn march(&self, plan: &Plan<'a>, outer: &[f64], span: f64, pieces: usize, sens: bool)
        -> Option<March> {
        let st = self.start(plan, outer, pieces, sens)?;
        let mut flow = Flow { body: self, plan, grid: &st.grid,
                              outer: outer[..self.n_outer].to_vec(), branch: st.branch.clone(),
                              area: &st.area, strict: false, why: None };
        let width = ode::Rate::width(&flow);
        let size = absmax(&st.y[..width]).max(1.0);
        let scale = vec![size; width];
        let settings = ode::Settings::default();
        let m = if sens { self.n_theta } else { 0 };
        let (path, refused) =
            ode::march_until(&mut flow, outer[0], &st.y, m, span, &scale, &settings);
        let refused = refused.map(|r| {
            let why = match r {
                ode::Refusal::Steps { .. } => Why::Steps,
                _ => flow.why.unwrap_or(Why::Rank { sigma: f64::NAN }),
            };
            (r, why)
        });
        Some(March { outer: outer[..self.n_outer].to_vec(), home: outer[0], grid: st.grid, sens,
                     path, area: st.area, refused, states: RefCell::new(Vec::new()) })
    }

    /// The flow over a march's grid, at its columns, from a branch.
    fn flow<'b>(&'b self, plan: &'b Plan<'a>, m: &'b March, branch: Vec<f64>, strict: bool)
        -> Flow<'b, 'a> {
        Flow { body: self, plan, grid: &m.grid, outer: m.outer.clone(), branch, area: &m.area,
               strict, why: None }
    }

    /// The state at `u` along a march, remembered.
    fn state(&self, plan: &Plan<'a>, m: &March, u: f64, sens: bool)
        -> Option<Rc<ode::Node<Vec<f64>>>> {
        if !m.reaches(u) || (sens && !m.sens) {
            return None;
        }
        let key = u.to_bits();
        if let Some((_, s)) = m.states.borrow().iter().find(|(k, s)| *k == key
            && (!sens || s.y.len() > m.path.n)) {
            return Some(s.clone());
        }
        let mut flow = self.flow(plan, m, Vec::new(), false);
        let node = Rc::new(m.path.at_within(&mut flow, u, sens && m.sens).ok()?);
        let mut states = m.states.borrow_mut();
        if states.len() >= 64 {
            states.clear();
        }
        states.push((key, node.clone()));
        Some(node)
    }

    /// The surface at `(u, s)` along a march: the traced point, its derivatives in the two formals
    /// (`d[0]` along the marched, `d[1]` along the sampled), its second derivatives where `need`
    /// is 2 and, with `grad`, the derivative of the point and of its first derivatives along each
    /// formal and each column (`[[x, d[0], d[1]]; 2 + n_theta]`).  `None` where it cannot be had,
    /// and where a Jacobian of the second derivatives is asked (a third order, not read).
    pub fn read(&self, plan: &Plan<'a>, m: &March, u: f64, s: f64, need: u8, grad: bool)
        -> Option<Reading> {
        if need >= 2 && grad {
            return None;
        }
        let (ni, nt) = (plan.i.len(), self.n_theta);
        let node = self.state(plan, m, u, grad)?;
        let nn = m.grid.n();
        let width = nn * ni + nt;
        let mut flow = self.flow(plan, m, node.branch.clone(), true);
        let mut f = vec![0.0; width];
        let ev = flow.eval(u, &node.y[..width], &mut f)?;
        let (idx, bas) = m.grid.at(s);
        let t = plan.t;
        let y = &node.y;
        let sum = |w: &[f64; 6], vals: &dyn Fn(usize) -> f64| -> f64 {
            (0..6).map(|i| w[i] * vals(idx[i])).sum()
        };
        let comp = |vals: &[f64], w: &[f64; 6]| -> [f64; 3] {
            std::array::from_fn(|c| sum(w, &|j| vals[j * ni + t + c]))
        };
        let x = comp(&y[..nn * ni], &bas[0]);
        let xu = comp(&y[..nn * ni], &bas[1]);
        let xuu = comp(&y[..nn * ni], &bas[2]);
        let xd = comp(&f, &bas[0]);
        let xdu = comp(&f, &bas[1]);
        let zero = [0.0; 3];
        let mut r = Reading { x, d: [xd, xu], dd: [[zero; 2]; 2], grad: None };
        if need < 2 && !grad {
            return Some(r);
        }
        // along the march itself: the rate's derivative along the flow
        let near: Vec<usize> = {
            let mut v: Vec<usize> = idx.to_vec();
            v.dedup();
            v
        };
        let picked = |moved: Vec<(usize, Vec<f64>)>| -> Vec<f64> {
            let mut out = vec![0.0; nn * ni];
            for (j, dv) in moved {
                out[j * ni..(j + 1) * ni].copy_from_slice(&dv);
            }
            out
        };
        let a = picked(flow.moved(&ev, &near, 1.0, &f[..nn * ni], &vec![0.0; nt])?);
        let xdd = comp(&a, &bas[0]);
        r.dd = [[xdd, xdu], [xdu, xuu]];
        if grad {
            let mut g = vec![[zero; 3]; 2 + nt];
            g[0] = [xd, xdd, xdu];
            g[1] = [xu, xdu, xuu];
            let phi = &y[width..];
            let mut dq = vec![0.0; nn * ni];
            let mut dt = vec![0.0; nt];
            for c in 0..nt {
                for (k, x) in dq.iter_mut().enumerate() {
                    *x = phi[k * nt + c];
                }
                dt.iter_mut().enumerate().for_each(|(k, x)| *x = if k == c { 1.0 } else { 0.0 });
                let dphi = picked(flow.moved(&ev, &near, 0.0, &dq, &dt)?);
                g[2 + c] = [comp(&dq, &bas[0]), comp(&dphi, &bas[0]), comp(&dq, &bas[1])];
            }
            r.grad = Some(g);
        }
        Some(r)
    }
}

/// The surface at a place (`Body::read`).
#[derive(Clone, Debug)]
pub struct Reading {
    pub x: [f64; 3],
    pub d: [[f64; 3]; 2],
    pub dd: [[[f64; 3]; 2]; 2],
    /// Per column — the marched formal, the sampled, then `θ` — the derivative of `x`, `d[0]`
    /// and `d[1]`.
    pub grad: Option<Vec<[[f64; 3]; 3]>>,
}

/// How many marches are remembered at once: a cache, cleared past its bound.
const MARCHES_MAX: usize = 16;

thread_local! {
    /// The marches the contacts and the drawing read, by what each is a function of: the body's
    /// encoding, the columns and numbers, the home, how far it runs, the pieces and whether it
    /// carries sensitivities.  By content, as a curve's are (`rates::remembered`).
    static MARCHES: RefCell<Vec<(Vec<u64>, bool, Marched)>> = const { RefCell::new(Vec::new()) };
}

/// The march of `body` (encoded as `flat`) from `home` at the columns of `outer` to `to`,
/// remembered; one carrying sensitivities serves a reading that asks none.  One refused keeps the
/// road to where it was refused (`Err` only where it could not start).
pub fn remembered(body: &Body, flat: &[f64], outer: &[f64], home: f64, to: f64, pieces: usize,
                  sens: bool) -> Marched {
    let Ok(plan) = body.plan() else { return Err(None) };
    if outer.len() < body.n_outer {
        return Err(None);
    }
    let mut key: Vec<u64> = Vec::with_capacity(flat.len() + body.n_outer + 3);
    key.extend(flat.iter().map(|x| x.to_bits()));
    key.extend(outer[2..body.n_outer].iter().map(|x| x.to_bits()));
    key.extend([home.to_bits(), to.to_bits(), pieces as u64]);
    let seen = MARCHES.with(|c| c.borrow().iter()
        .find(|(k, s, _)| *k == key && (*s || !sens)).map(|e| e.2.clone()));
    if let Some(m) = seen {
        return m;
    }
    let mut at = outer[..body.n_outer].to_vec();
    at[0] = home;
    let m = body.march(&plan, &at, to - home, pieces, sens).map(Rc::new).ok_or(None);
    MARCHES.with(|c| {
        let mut c = c.borrow_mut();
        if c.len() >= MARCHES_MAX {
            c.clear();
        }
        c.push((key, sens, m.clone()));
    });
    m
}

/// How far a march reached toward an end: there, or refused and why (`None` where it could not
/// start).
pub type Reached = Result<(), Option<(ode::Refusal, Why)>>;

/// How far `body` is integrated from `home` toward each of the marched formal's `ends`.
pub fn reaches(body: &Body, flat: &[f64], outer: &[f64], home: f64, ends: [f64; 2])
    -> [Reached; 2] {
    ends.map(|end| match end == home {
        true => Ok(()),
        false => remembered(body, flat, outer, home, end, body.pieces, false)
            .and_then(|m| m.refused.map_or(Ok(()), |r| Err(Some(r)))),
    })
}

/// **A march held to the march on twice the pieces** (the module's docs): the greatest distance
/// between the two surfaces at the coarse grid's nodes, at `places` along the marched formal each
/// reaches — what bounds the coarse march's error, as the gate holds it to.  `None` where either
/// cannot be read.
pub fn check(body: &Body, flat: &[f64], outer: &[f64], home: f64, to: f64, pieces: usize,
             places: &[f64]) -> Option<f64> {
    let plan = body.plan().ok()?;
    let coarse = remembered(body, flat, outer, home, to, pieces, false).ok()?;
    let fine = remembered(body, flat, outer, home, to, 2 * pieces, false).ok()?;
    let mut worst = 0.0f64;
    for &u in places {
        if !coarse.reaches(u) || !fine.reaches(u) {
            return None;
        }
        for &s in &coarse.grid.nodes {
            let a = body.read(&plan, &coarse, u, s, 0, false)?;
            let b = body.read(&plan, &fine, u, s, 0, false)?;
            worst = worst.max(crate::space::distance(a.x, b.x));
        }
    }
    Some(worst)
}
