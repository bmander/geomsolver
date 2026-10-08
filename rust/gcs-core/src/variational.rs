//! **Variational curves** (#121): `minimize` compiled to stationarity.
//!
//! An energy `E` is a sum of integrals over splines, `c · ∫ f(p, t) ds`.  "The curve that
//! minimises `E` subject to the drawing" is, at a regular point, where `E`'s gradient in the
//! curve's own unknowns is a combination of the constraints' — the KKT conditions
//!
//! ```text
//!     ∂E/∂y + Σ_r λ_r ∂g_r/∂y = 0
//! ```
//!
//! one row per free coordinate `y` of the curve's **interior** control points, `g_r` every row of
//! the drawing that reads one, and `λ_r` a multiplier per such row — an unknown of its own
//! (`Sketch::multipliers`, minted here, never saved).  The ends are not varied: they are what the
//! curve hangs between, so they keep the freedom they had, and a `minimize` closes exactly the
//! curve's own.  Several statements over one curve are one energy: the group's first
//! `Stationary` constraint carries the rows, the rest none.
//!
//! The rows' Jacobian is the Lagrangian's Hessian `∂²E/∂y∂x + Σ λ ∂²g/∂y∂x` beside `∂g/∂y`.  An
//! integral's Hessian is exact — the curve is linear in its control points over a fixed basis, so
//! a node's Hessian is the integrand's in `C` and `C'`, from four tapes each differentiated first
//! in one of them (`tape::eval_series_flat`).  A row's is its Taylor form's (`taylor::residual`),
//! the second coefficient polarised as `kernels::dual_kernel` reads it; a spline's length and its
//! gauge give theirs (`integral.rs`).  A row with neither is refused.
//!
//! **The gauge.**  A spline's control points can slide along it without moving its shape, so an
//! energy of the shape alone is stationary along those slides at best to second order: a saddle,
//! and a solve that wanders.  Every varied spline is held to equal span lengths
//! (`CKind::SplineGauge`, intrinsic, minted here), which is a choice of the discretisation, not
//! of the curve — as the control-point count is.

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::constraints::{Arg, CKind, Constraint};
use crate::expr::{Ast, Op};
use crate::integral;
use crate::kernels::{self, Kernel, KernelKey, KERNELS, K};
use crate::model::{EntRef, Sketch};
use crate::taylor::Jet;
use crate::tape::{self, Tape};
use std::collections::{BTreeMap, BTreeSet};

/// One energy's stationarity, as `Sketch::settle_variational` last read it.
#[derive(Clone, Debug, Default)]
pub struct Group {
    /// The `Stationary` constraint that carries the rows: the group's lowest id.
    pub leader: u32,
    pub members: Vec<u32>,
    /// The splines varied, ascending.
    pub splines: Vec<usize>,
    /// The free coordinates varied: each spline's interior control points', in order.
    pub y: Vec<u32>,
    /// Every hard row that reads one, in sketch order.
    pub rows: Vec<u32>,
    /// The kernel's columns: `y`, then everything else the terms and rows read, then the
    /// multipliers, row by row.
    pub cols: Vec<u32>,
}

/* -- integrands --------------------------------------------------------------------------- */

/// The integrand's variables, in the order a node hands them over: the point, then `C'`.
const VARS: [&str; 4] = ["cx", "cy", "dx", "dy"];

/// An integrand compiled: `F(C, C') = f(C, C'/|C'|)·|C'|`, so `∫ F dt` is `∫ f ds`, as four tapes,
/// the `k`th over the variables with the `k`th first — `tape::Series` differentiates in the first,
/// so its gradient of the first order is the Hessian's `k`th row.
pub struct Integrand {
    pub tapes: [Tape; 4],
}

/// `text` over `p.x`, `p.y`, `t.x`, `t.y` (`flatten::values::settle_integrand`) compiled.
pub fn integrand(text: &str, units: crate::units::Units) -> Result<Integrand, String> {
    let parsed = crate::expr::parse_in(text, units)?;
    let speed = || Ast::Call("hypot".into(), vec![Ast::Var("dx".into()), Ast::Var("dy".into())]);
    fn rewrite(a: &Ast, speed: &dyn Fn() -> Ast) -> Ast {
        match a {
            Ast::Var(v) => match v.as_str() {
                "p.x" => Ast::Var("cx".into()),
                "p.y" => Ast::Var("cy".into()),
                "t.x" => Ast::Bin(Op::Div, Box::new(Ast::Var("dx".into())), Box::new(speed())),
                "t.y" => Ast::Bin(Op::Div, Box::new(Ast::Var("dy".into())), Box::new(speed())),
                _ => a.clone(),
            },
            Ast::Neg(x) => Ast::Neg(Box::new(rewrite(x, speed))),
            Ast::Bin(o, x, y) => Ast::Bin(*o, Box::new(rewrite(x, speed)), Box::new(rewrite(y, speed))),
            Ast::Call(f, xs) => Ast::Call(f.clone(), xs.iter().map(|x| rewrite(x, speed)).collect()),
            other => other.clone(),
        }
    }
    let f = Ast::Bin(Op::Mul, Box::new(rewrite(&parsed.body, &speed)), Box::new(speed()));
    let tape = |k: usize| {
        let mut vars = vec![VARS[k].to_string()];
        vars.extend(VARS.iter().filter(|v| **v != VARS[k]).map(|v| v.to_string()));
        Tape::compile(&f, &vars)
    };
    Ok(Integrand { tapes: [tape(0)?, tape(1)?, tape(2)?, tape(3)?] })
}

/// The power of length an integrand carries — `p.y` is one, a length's integrand none — so the
/// rows its stationarity is stated in are of that degree (`∂E/∂y`, `E` being one power more).
/// Where the document names a unit the point is a length (`Units::read_length`) and the
/// integrand is dimension-checked, refused where it is no whole power or mixes dimensions; where
/// it names none every number is plain, and the degree is read off how the integrand grows with
/// the point.
pub fn integrand_degree(text: &str, units: crate::units::Units) -> Result<u32, String> {
    let parsed = crate::expr::parse_in(text, units)?;
    let at = |s: f64, dim: crate::units::Dim| -> Result<crate::expr::Aff, String> {
        let env: BTreeMap<String, crate::expr::Aff> = [
            ("p.x", crate::expr::Aff::of_dim(1.3 * s, dim)),
            ("p.y", crate::expr::Aff::of_dim(0.7 * s, dim)),
            ("t.x", crate::expr::Aff::num(0.6)),
            ("t.y", crate::expr::Aff::num(0.8)),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_string(), v))
        .collect();
        crate::expr::eval(&parsed.body, &env)
    };
    if units.length.is_none() {
        let (a, b) = (at(1.0, units.read_length())?, at(2.0, units.read_length())?);
        let d = (b.c / a.c).abs().log2().round();
        return Ok(if d.is_finite() { d.clamp(0.0, 4.0) as u32 } else { 1 });
    }
    let a = at(1.0, units.read_length())?;
    if !a.dim.ang.is_zero() {
        return Err("an integrand is a length or a power of one, never an angle".into());
    }
    match a.dim.len.whole() {
        Some(n) if n >= 0 => Ok(n as u32),
        _ => Err("an integrand is a whole power of length".into()),
    }
}

/* -- groups ------------------------------------------------------------------------------- */

impl Sketch {
    /// Bring every energy's group, gauge rows and multipliers into step with the constraints —
    /// at the seams where the expressions are (`add`, `remove`, `graft`, a document read, the end
    /// of elaboration).  Returns the rows refused, by constraint id: one reading a varied
    /// coordinate that no second derivative can be taken of.
    pub fn settle_variational(&mut self) -> Vec<(u32, String)> {
        let stat: Vec<usize> = (0..self.constraints.len())
            .filter(|&i| self.constraints[i].kind == CKind::Stationary && !self.constraints[i].claim)
            .collect();
        // a drawing with no energy, and none before, has nothing to bring into step
        if stat.is_empty() && self.variational.is_empty() && self.multipliers.is_empty() {
            return Vec::new();
        }
        let varied: BTreeSet<usize> = stat.iter().map(|&i| self.constraints[i].args[0].ent().i()).collect();
        self.settle_gauges(&varied);
        let mut faults = Vec::new();
        // each varied spline's free interior coordinates
        let mut owner: BTreeMap<u32, usize> = BTreeMap::new();
        let mut y_of: BTreeMap<usize, Vec<u32>> = BTreeMap::new();
        for &s in &varied {
            let ctrl = &self.splines[s].ctrl;
            let mut ys = Vec::new();
            for &c in &ctrl[1..ctrl.len() - 1] {
                for p in self.point_params(c as usize) {
                    if !self.params[p as usize].fixed {
                        ys.push(p);
                        owner.insert(p, s);
                    }
                }
            }
            y_of.insert(s, ys);
        }
        // splines read by one row are one energy, as two terms over one spline are
        let mut root: BTreeMap<usize, usize> = varied.iter().map(|&s| (s, s)).collect();
        fn find(root: &BTreeMap<usize, usize>, mut s: usize) -> usize {
            while root[&s] != s {
                s = root[&s];
            }
            s
        }
        let mut reading: Vec<(u32, BTreeSet<usize>)> = Vec::new();
        for c in &self.constraints {
            if !c.acts() || c.kind == CKind::Stationary {
                continue;
            }
            let touched: BTreeSet<usize> =
                c.params_on(self, None).iter().filter_map(|p| owner.get(p).copied()).collect();
            if touched.is_empty() {
                continue;
            }
            if let Err(why) = hessian_tag(self, c) {
                faults.push((c.id, why));
            }
            let mut it = touched.iter();
            let first = find(&root, *it.next().unwrap());
            for &s in it {
                let r = find(&root, s);
                root.insert(r, first);
            }
            reading.push((c.id, touched));
        }
        let mut groups: BTreeMap<usize, Group> = BTreeMap::new();
        for &i in &stat {
            let c = &self.constraints[i];
            let g = groups.entry(find(&root, c.args[0].ent().i())).or_default();
            g.members.push(c.id);
        }
        for (&s, ys) in &y_of {
            let g = groups.get_mut(&find(&root, s)).expect("a varied spline is in a group");
            g.splines.push(s);
            g.y.extend(ys);
        }
        for (id, touched) in &reading {
            let g = groups.get_mut(&find(&root, *touched.iter().next().unwrap())).unwrap();
            g.rows.push(*id);
        }
        // the multipliers, one per row the group reads, kept where they were
        let mut used = BTreeSet::new();
        let mut out = Vec::with_capacity(groups.len());
        for (_, mut g) in groups {
            g.members.sort();
            g.leader = g.members[0];
            let ys: BTreeSet<u32> = g.y.iter().copied().collect();
            let mut cols = g.y.clone();
            let mut seen = ys.clone();
            let mut more = |ps: Vec<u32>, cols: &mut Vec<u32>| {
                for p in ps {
                    if seen.insert(p) {
                        cols.push(p);
                    }
                }
            };
            for &s in &g.splines {
                more(self.entity_params(EntRef::spline(s)), &mut cols);
            }
            for &r in &g.rows {
                let c = self.constraint(r).expect("a row read is in the sketch");
                more(c.params_on(self, None), &mut cols);
            }
            for &r in &g.rows {
                let n = self.constraint(r).map_or(0, |c| c.rows_in(self));
                for k in 0..n {
                    let key = (g.leader, r, k as u8);
                    let p = match self.multipliers.get(&key) {
                        Some(&p) => {
                            self.params[p as usize].fixed = false;
                            p
                        }
                        None => {
                            let p = self.param(0.0, false, &format!("c{}.λ{r}.{k}", g.leader)) as u32;
                            self.multipliers.insert(key, p);
                            p
                        }
                    };
                    used.insert(p);
                    cols.push(p);
                }
            }
            g.cols = cols;
            out.push(g);
        }
        for &p in self.multipliers.values() {
            if !used.contains(&p) {
                self.params[p as usize].fixed = true;
            }
        }
        self.variational = out;
        faults
    }

    /// One gauge row per pair of neighbouring non-empty spans of every varied spline, and none
    /// on a spline nothing varies.
    fn settle_gauges(&mut self, varied: &BTreeSet<usize>) {
        let want: BTreeSet<(usize, usize)> = varied
            .iter()
            .flat_map(|&s| {
                let sp = &self.splines[s];
                let n = sp.ctrl.len();
                let k = &sp.knots;
                (crate::curve::DEGREE..n.saturating_sub(1))
                    .filter(move |&j| k[j] < k[j + 1] && k[j + 1] < k[j + 2])
                    .map(move |j| (s, j))
            })
            .collect();
        let gauge = |c: &Constraint| (c.args[0].ent().i(), c.args[1].num() as usize);
        self.constraints.retain(|c| c.kind != CKind::SplineGauge || want.contains(&gauge(c)));
        let have: BTreeSet<(usize, usize)> =
            self.constraints.iter().filter(|c| c.kind == CKind::SplineGauge).map(gauge).collect();
        for &(s, j) in want.difference(&have) {
            let mut c = Constraint::new(
                CKind::SplineGauge,
                vec![Arg::Ent(EntRef::spline(s)), Arg::Int(j as i64)],
            );
            c.intrinsic = true;
            self.add_quiet(c);
        }
    }

    /// The group whose rows constraint `cid` carries, when it leads one.
    pub fn leads(&self, cid: u32) -> Option<&Group> {
        self.variational.iter().find(|g| g.leader == cid)
    }
}

/// How a row's second derivative is taken inside a stationarity (`pack`'s tag): its static
/// kernel's id, `-1` for a spline's length — or why there is none.
fn hessian_tag(sk: &Sketch, c: &Constraint) -> Result<f64, String> {
    let what = || crate::model::article(&crate::syntax::snake(c.kind.name()));
    if c.along.is_some() {
        return Err(format!("{} is stated as a derivative, which an energy cannot vary", what()));
    }
    match c.kernel_key(sk) {
        KernelKey::SplineLength { .. } => Ok(-1.0),
        KernelKey::Static(kid)
            if kid == K::SplineGauge as usize || crate::taylor::has_form(kid) => Ok(kid as f64),
        _ => Err(format!(
            "{} reads the curve an energy varies, and has no second derivative to vary it by",
            what()
        )),
    }
}

/* -- the kernel's columns and constants --------------------------------------------------- */

/// The columns constraint `cid` reads: its group's, where it leads one, else none.
pub fn columns(sk: &Sketch, cid: u32) -> Vec<u32> {
    sk.leads(cid).map(|g| g.cols.clone()).unwrap_or_default()
}

/// Its rows: one per varied coordinate where it leads a group, else none.
pub fn rows(sk: &Sketch, cid: u32) -> usize {
    sk.leads(cid).map_or(0, |g| g.y.len())
}

/// The kernel of the group constraint `cid` leads: `rows` × `columns`, over `pack`'s constants,
/// of the degree its terms' integrands carry.
pub fn kernel(sk: &Sketch, cid: u32) -> Kernel {
    let n_res = rows(sk, cid);
    let degree = sk.constraint(cid).map_or(0, |c| c.args[3].num() as u32);
    Kernel {
        name: "stationary",
        n_res,
        n_par: columns(sk, cid).len(),
        n_const: if n_res == 0 { 0 } else { pack(sk, cid).len() },
        degree,
        res: stationary_res,
        jac: stationary_jac,
        const_jac: None,
    }
}

/// Where each of `ps` stands among the group's columns.
fn colmap(index: &BTreeMap<u32, usize>, ps: &[u32]) -> Vec<f64> {
    ps.iter().map(|p| index.get(p).map_or(-1.0, |&i| i as f64)).collect()
}

/// Everything the group's kernel reads besides its columns: a header `[n_y, n_cols, n_terms,
/// n_rows]`; per term `[coef, n, colmap (2n), knots (n + 4), weights (n), 4 × (len, tape…)]`; per
/// row `[tag, n_res, n_par, n_const, λ at, colmap (n_par), constants (n_const)]`.
pub fn pack(sk: &Sketch, cid: u32) -> Vec<f64> {
    let Some(g) = sk.leads(cid) else { return Vec::new() };
    let index: BTreeMap<u32, usize> = g.cols.iter().enumerate().map(|(i, &p)| (p, i)).collect();
    let terms: Vec<&Constraint> =
        g.members.iter().filter_map(|&m| sk.constraint(m)).collect();
    let mut k = vec![g.y.len() as f64, g.cols.len() as f64, terms.len() as f64, g.rows.len() as f64];
    for c in &terms {
        let s = c.args[0].ent().i();
        let sp = &sk.splines[s];
        let n = sp.ctrl.len();
        let sign = if matches!(c.args[4], Arg::Bool(true)) { -1.0 } else { 1.0 };
        k.push(sign * c.args[1].num());
        k.push(n as f64);
        k.extend(colmap(&index, &sk.entity_params(EntRef::spline(s))));
        k.extend_from_slice(&sp.knots);
        match &sp.weights {
            Some(w) => k.extend_from_slice(w),
            None => k.extend(std::iter::repeat_n(1.0, n)),
        }
        let Arg::Str(text) = &c.args[2] else { unreachable!("an energy's integrand is text") };
        match integrand(text, sk.units) {
            Ok(f) => {
                for t in &f.tapes {
                    k.push(t.flat.len() as f64);
                    k.extend_from_slice(&t.flat);
                }
            }
            // refused at elaboration; a document read carries what it was given, so a text that
            // no longer compiles reads as an integrand of nothing rather than a panic
            Err(_) => k.extend([0.0; 4]),
        }
    }
    // the multipliers are the last columns, row by row
    let n_lam: usize = g.rows.iter().map(|&r| sk.constraint(r).map_or(0, |c| c.rows_in(sk))).sum();
    let mut at = g.cols.len() - n_lam;
    for &r in &g.rows {
        let c = sk.constraint(r).expect("a row read is in the sketch");
        let kn = c.kernel_in(sk);
        let ps = c.params_on(sk, None);
        let consts = c.consts_on(sk, None);
        k.push(hessian_tag(sk, c).unwrap_or(f64::NAN));
        k.extend([kn.n_res as f64, kn.n_par as f64, consts.len() as f64, at as f64]);
        k.extend(colmap(&index, &ps));
        k.extend(consts);
        at += kn.n_res;
    }
    k
}

/* -- evaluation --------------------------------------------------------------------------- */

/// One energy term, read back out of the constants.
struct Term<'a> {
    coef: f64,
    n: usize,
    map: &'a [f64],
    knots: &'a [f64],
    weights: &'a [f64],
    tapes: [&'a [f64]; 4],
}

/// One row the energy is stationary under, read back.
struct Row<'a> {
    tag: f64,
    n_res: usize,
    map: &'a [f64],
    consts: &'a [f64],
    lam: usize,
}

struct Packed<'a> {
    n_y: usize,
    terms: Vec<Term<'a>>,
    rows: Vec<Row<'a>>,
}

fn read(k: &[f64]) -> Option<Packed<'_>> {
    let mut at = 0usize;
    let mut take = |n: usize| -> Option<&[f64]> {
        let s = k.get(at..at + n)?;
        at += n;
        Some(s)
    };
    let h = take(4)?;
    let (n_y, n_terms, n_rows) = (h[0] as usize, h[2] as usize, h[3] as usize);
    let mut terms = Vec::with_capacity(n_terms);
    for _ in 0..n_terms {
        let h = take(2)?;
        let (coef, n) = (h[0], h[1] as usize);
        let map = take(2 * n)?;
        let knots = take(n + crate::curve::DEGREE + 1)?;
        let weights = take(n)?;
        let mut tapes: [&[f64]; 4] = [&[]; 4];
        for t in tapes.iter_mut() {
            let len = take(1)?[0] as usize;
            *t = take(len)?;
        }
        terms.push(Term { coef, n, map, knots, weights, tapes });
    }
    let mut rows = Vec::with_capacity(n_rows);
    for _ in 0..n_rows {
        let h = take(5)?;
        let (tag, n_res, n_par, n_const, lam) =
            (h[0], h[1] as usize, h[2] as usize, h[3] as usize, h[4] as usize);
        let map = take(n_par)?;
        let consts = take(n_const)?;
        rows.push(Row { tag, n_res, map, consts, lam });
    }
    Some(Packed { n_y, terms, rows })
}

impl Term<'_> {
    fn nodes(&self) -> Vec<integral::Node> {
        let weighted = self.weights.iter().any(|&w| w != 1.0);
        integral::nodes(self.knots, weighted.then_some(self.weights), self.n)
    }

    /// The control points' values, from the group's columns.
    fn points(&self, v: &[f64]) -> Vec<f64> {
        self.map.iter().map(|&c| v[c as usize]).collect()
    }
}

impl Row<'_> {
    fn kernel(&self) -> Kernel {
        if self.tag < 0.0 {
            let (n, free) = integral::length_widths(self.map.len());
            kernels::spline_length_kernel(n, free)
        } else {
            KERNELS[self.tag as usize]
        }
    }

    fn values(&self, v: &[f64]) -> Vec<f64> {
        self.map.iter().map(|&c| v[c as usize]).collect()
    }
}

/// Add `f` into the stationarity row of column `c`, where `c` is a varied coordinate.
fn into_row(r: &mut [f64], n_y: usize, c: f64, f: f64) {
    if c >= 0.0 && (c as usize) < n_y {
        r[c as usize] += f;
    }
}

fn stationary_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    if n != 1 {
        r.fill(f64::NAN);
        return;
    }
    let Some(p) = read(k) else {
        r.fill(f64::NAN);
        return;
    };
    r.fill(0.0);
    let mut s = tape::Scratch::new();
    for t in &p.terms {
        let pts = t.points(v);
        for q in t.nodes() {
            let (c, d) = q.frame(&pts);
            let val = tape::eval_flat(t.tapes[0], 4, &[c[0], c[1], d[0], d[1]], &mut s);
            for a in 0..crate::curve::SPAN_N {
                for i in 0..2 {
                    let col = t.map[2 * (q.first + a) + i];
                    into_row(r, p.n_y, col, t.coef * q.w * (q.b[a] * val.d[i] + q.d[a] * val.d[2 + i]));
                }
            }
        }
    }
    for row in &p.rows {
        let kn = row.kernel();
        let x = row.values(v);
        let mut j = vec![0.0; kn.n_res * kn.n_par];
        kn.jac_into(&x, row.consts, &mut j);
        for kk in 0..row.n_res {
            let lam = v[row.lam + kk];
            for (c, &col) in row.map.iter().enumerate() {
                into_row(r, p.n_y, col, lam * j[kk * kn.n_par + c]);
            }
        }
    }
}

fn stationary_jac(n: usize, v: &[f64], k: &[f64], out: &mut [f64]) {
    let n_par = v.len();
    out.fill(0.0);
    let Some(p) = read(k).filter(|_| n == 1) else {
        out.fill(f64::NAN);
        return;
    };
    let n_y = p.n_y;
    let mut add = |row: f64, col: f64, f: f64| {
        if row >= 0.0 && (row as usize) < n_y && col >= 0.0 {
            out[row as usize * n_par + col as usize] += f;
        }
    };
    // the energy's Hessian, node by node: the integrand's in (C, C') by the chain rule, each a
    // sum of the control points times the basis there
    let mut s = tape::Scratch::new();
    for t in &p.terms {
        let pts = t.points(v);
        for q in t.nodes() {
            let (c, d) = q.frame(&pts);
            let x = [c[0], c[1], d[0], d[1]];
            let mut h = [[0.0; 4]; 4];
            for (kv, row) in h.iter_mut().enumerate() {
                let mut xs = vec![x[kv]];
                xs.extend((0..4).filter(|&j| j != kv).map(|j| x[j]));
                let ser = tape::eval_series_flat(t.tapes[kv], 4, &xs, &mut s);
                let order: Vec<usize> = std::iter::once(kv).chain((0..4).filter(|&j| j != kv)).collect();
                for (jj, &var) in order.iter().enumerate() {
                    row[var] = ser.g[1][jj];
                }
            }
            for a in 0..crate::curve::SPAN_N {
                for b in 0..crate::curve::SPAN_N {
                    for i in 0..2 {
                        for jx in 0..2 {
                            let f = q.b[a] * q.b[b] * h[i][jx]
                                + q.b[a] * q.d[b] * h[i][2 + jx]
                                + q.d[a] * q.b[b] * h[2 + i][jx]
                                + q.d[a] * q.d[b] * h[2 + i][2 + jx];
                            add(t.map[2 * (q.first + a) + i], t.map[2 * (q.first + b) + jx], t.coef * q.w * f);
                        }
                    }
                }
            }
        }
    }
    // each row: its gradient where its multiplier is, and its multipliers' Hessian
    for row in &p.rows {
        let kn = row.kernel();
        let x = row.values(v);
        let m = kn.n_par;
        let mut j = vec![0.0; kn.n_res * m];
        kn.jac_into(&x, row.consts, &mut j);
        for kk in 0..row.n_res {
            for (c, &col) in row.map.iter().enumerate() {
                add(col, (row.lam + kk) as f64, j[kk * m + c]);
            }
        }
        let lam: Vec<f64> = (0..row.n_res).map(|kk| v[row.lam + kk]).collect();
        let need: Vec<bool> = row.map.iter().map(|&c| c >= 0.0 && (c as usize) < n_y).collect();
        let mut hl = vec![0.0; m * m];
        row_hessian(row.tag, &x, row.consts, &lam, &need, &mut hl);
        for a in (0..m).filter(|&a| need[a]) {
            for b in 0..m {
                add(row.map[a], row.map[b], hl[a * m + b]);
            }
        }
    }
}

/// `Σ_k λ_k ∂²g_k` over a row's own columns into `h` (`m × m`), rows `a` with `need[a]` only.
fn row_hessian(tag: f64, x: &[f64], kc: &[f64], lam: &[f64], need: &[bool], h: &mut [f64]) {
    let m = x.len();
    if tag < 0.0 {
        // a spline's length: the free column, where there is one, enters linearly
        let (n, _) = integral::length_widths(m);
        let nodes = integral::length_nodes(n, kc);
        integral::length_hess(&nodes, &x[..2 * n], lam[0], None, h, m);
        return;
    }
    let kid = tag as usize;
    if kid == K::SplineGauge as usize {
        let n = kernels::GAUGE_CTRL;
        let nodes = integral::length_nodes(n, kc);
        integral::length_hess(&nodes, x, lam[0], Some(1), h, m);
        integral::length_hess(&nodes, x, -lam[0], Some(0), h, m);
        return;
    }
    if crate::taylor::is_affine(kid) {
        return;
    }
    // the second Taylor coefficient along `w` is `½ wᵀ H w`, so `S(e_a)` is half the diagonal
    // and `S(e_a + e_b) − S(e_a) − S(e_b)` the entry between
    let nr = KERNELS[kid].n_res;
    let (mut path, mut r, mut jrow) = (Vec::with_capacity(m), vec![Jet::default(); nr], Vec::new());
    let mut s = |w: &[f64]| -> f64 {
        path.clear();
        path.extend(x.iter().zip(w).map(|(&xi, &wi)| Jet::from(&[xi, wi])));
        if !crate::taylor::residual(kid, &path, kc, &mut r, &mut jrow) {
            return f64::NAN;
        }
        (0..nr).map(|k| lam[k] * r[k].0[2]).sum()
    };
    let mut w = vec![0.0; m];
    let mut one = vec![0.0; m];
    for a in 0..m {
        w[a] = 1.0;
        one[a] = s(&w);
        w[a] = 0.0;
    }
    for a in (0..m).filter(|&a| need[a]) {
        h[a * m + a] = 2.0 * one[a];
        for b in (0..m).filter(|&b| b != a) {
            w[a] = 1.0;
            w[b] = 1.0;
            h[a * m + b] = s(&w) - one[a] - one[b];
            w[a] = 0.0;
            w[b] = 0.0;
        }
    }
}

/* -- multipliers -------------------------------------------------------------------------- */

/// Each group's multipliers where they best answer its stationarity at the current pose: the
/// least-squares `λ` of `∂E/∂y + (∂g/∂y)ᵀ λ = 0` — the multipliers *are* a function of the pose at
/// a solution, so this is the start a solve deserves, and zero (the gradient alone) is far from it.
pub fn seed_multipliers(sk: &mut Sketch) {
    for g in sk.variational.clone() {
        let Some(c) = sk.constraint(g.leader).cloned() else { continue };
        let n_y = g.y.len();
        if n_y == 0 {
            continue;
        }
        let n_lam: usize = g.rows.iter().map(|&r| sk.constraint(r).map_or(0, |c| c.rows_in(sk))).sum();
        let lam0 = g.cols.len() - n_lam;
        let mut v: Vec<f64> = g.cols.iter().map(|&p| sk.params[p as usize].value).collect();
        v[lam0..].fill(0.0);
        let kn = c.kernel_in(sk);
        let (r, j) = kernels::eval_with(&kn, &v, &c.consts(sk));
        if r.iter().any(|x| !x.is_finite()) {
            continue;
        }
        let mut a = crate::linalg::Mat::zeros(n_y, n_lam);
        for i in 0..n_y {
            for l in 0..n_lam {
                a.data[i * n_lam + l] = j[i * kn.n_par + lam0 + l];
            }
        }
        let b: Vec<f64> = r.iter().map(|x| -x).collect();
        let (lam, _) = crate::linalg::min_norm_solve(&a, &b, 1e-12);
        for (l, &p) in g.cols[lam0..].iter().enumerate() {
            if lam[l].is_finite() {
                sk.params[p as usize].value = lam[l];
            }
        }
    }
}
