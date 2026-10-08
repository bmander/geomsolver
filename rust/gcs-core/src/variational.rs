//! **Variational curves** (#121): `k minimizes …` compiled to stationarity.
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
//! curve hangs between, so they keep the freedom they had, and an energy closes exactly the
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
use crate::integral::{self, Node};
use crate::kernels::{self, Kernel, KernelKey, KERNELS, K};
use crate::linalg::Mat;
use crate::model::{EntRef, Sketch};
use crate::taylor::Jet;
use crate::tape::{self, Tape};
use std::collections::{BTreeMap, BTreeSet};

/// The control points a free curve (`spline(a, b)`) is drawn with: its two ends and the interior
/// the drawing finds.  A discretisation, not a statement — the curve it stands for is the one the
/// energy makes, and sixteen hold the catenary to 4e-5 of its span.
pub const FREE_CTRL: usize = 16;

/// One energy's stationarity, as `Sketch::settle_variational` last read it.
#[derive(Clone, Debug, Default)]
pub struct Group {
    /// The `Stationary` constraint that carries the rows: the group's lowest id.
    pub leader: u32,
    pub members: Vec<u32>,
    /// The splines varied, ascending.
    pub splines: Vec<usize>,
    /// The free coordinates varied: each spline's interior control points', in order, then the
    /// unknowns of the rows reading them own (a contact's place along the curve) — the
    /// configuration the energy is minimised over is the curve and where it touches.
    pub y: Vec<u32>,
    /// Every hard row that reads one, in sketch order, with how many residuals it has.
    pub rows: Vec<(u32, usize)>,
    /// The kernel's columns: `y`, then everything else the terms and rows read, then the
    /// multipliers, row by row from `lam0`.
    pub cols: Vec<u32>,
    pub lam0: usize,
    /// Each term, its integrand compiled once (`integrand`).
    pub terms: Vec<Term>,
    /// The power of length the terms' integrands carry: the rows' degree.
    pub degree: u32,
    /// A row reads a varied coordinate with no second derivative to take (`hessian_tag`): the
    /// group states no stationarity at all, rather than a wrong one.
    pub refused: bool,
}

/// One term of an energy: `coef · ∫ F ds` along a spline, `coef` signed as the energy is
/// minimised (a `maximizes` turns it), `F` as four tapes (`Integrand`).
#[derive(Clone, Debug, Default)]
pub struct Term {
    pub spline: usize,
    pub coef: f64,
    pub tapes: [Vec<f64>; 4],
}

/* -- integrands --------------------------------------------------------------------------- */

/// The integrand's variables, in the order a node hands them over: the point, then `C'`.
const VARS: [&str; 4] = ["cx", "cy", "dx", "dy"];

/// The variables in the order the `k`th tape reads them: the `k`th first, so the tape's series
/// differentiates in it (`tape::Series`), then the rest in order.
const ORDER: [[usize; 4]; 4] = [[0, 1, 2, 3], [1, 0, 2, 3], [2, 0, 1, 3], [3, 0, 1, 2]];

/// What an integrand may read: the point, and the unit tangent there.
pub const BOUND: [&str; 4] = ["p.x", "p.y", "t.x", "t.y"];

/// An integrand compiled: `F(C, C') = f(C, C'/|C'|)·|C'|`, so `∫ F dt` is `∫ f ds`, as four tapes,
/// the `k`th over the variables in `ORDER[k]` — `tape::Series` differentiates in the first, so its
/// gradient of the first order is the Hessian's `k`th row — and the power of length it carries
/// (`degree`).
pub struct Integrand {
    pub tapes: [Tape; 4],
    pub degree: u32,
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
            // a number, or a measurement (refused before a text gets here)
            other => other.clone(),
        }
    }
    let f = Ast::Bin(Op::Mul, Box::new(rewrite(&parsed.body, &speed)), Box::new(speed()));
    let tape = |k: usize| {
        let vars: Vec<String> = ORDER[k].iter().map(|&j| VARS[j].to_string()).collect();
        Tape::compile(&f, &vars)
    };
    Ok(Integrand {
        tapes: [tape(0)?, tape(1)?, tape(2)?, tape(3)?],
        degree: degree(&parsed.body, units)?,
    })
}

/// The power of length an integrand carries — `p.y` is one, a length's integrand none — so the
/// rows its stationarity is stated in are of that degree (`∂E/∂y`, `E` being one power more).
/// Where the document names a unit the point is a length (`Units::read_length`) and the
/// integrand is dimension-checked, refused where it is no whole power or mixes dimensions; where
/// it names none every number is plain, and the degree is read off how the integrand grows with
/// the point.
fn degree(body: &Ast, units: crate::units::Units) -> Result<u32, String> {
    let at = |s: f64| -> Result<crate::expr::Aff, String> {
        let dim = units.read_length();
        let env: BTreeMap<String, crate::expr::Aff> = BOUND
            .iter()
            .zip([
                crate::expr::Aff::of_dim(1.3 * s, dim),
                crate::expr::Aff::of_dim(0.7 * s, dim),
                crate::expr::Aff::num(0.6),
                crate::expr::Aff::num(0.8),
            ])
            .map(|(k, v)| (k.to_string(), v))
            .collect();
        crate::expr::eval(body, &env)
    };
    if units.length.is_none() {
        let d = (at(2.0)?.c / at(1.0)?.c).abs().log2().round();
        return Ok(if d.is_finite() { d.clamp(0.0, 4.0) as u32 } else { 1 });
    }
    let a = at(1.0)?;
    if !a.dim.ang.is_zero() {
        return Err("an integrand is a length or a power of one, never an angle".into());
    }
    match a.dim.len.whole() {
        Some(n) if n >= 0 => Ok(n as u32),
        _ => Err("an integrand is a whole power of length".into()),
    }
}

/// Whether an energy constraint is a `maximizes`'s term.
pub fn maximizes(c: &Constraint) -> bool {
    matches!(c.args[4], Arg::Bool(true))
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
        // every coordinate of a varied curve, its ends included: what a row may read and still be
        // about the curve alone
        let curve: BTreeSet<u32> =
            varied.iter().flat_map(|&s| self.entity_params(EntRef::spline(s))).collect();
        // splines read by one row are one energy, as two terms over one spline are
        let mut sets = crate::graph::UnionFind::new(self.splines.len());
        // each row a contact reads: its id, residuals, a spline it reads, whether it is refused, and
        // its own free unknowns (a contact's place along the curve)
        let mut reading: Vec<(u32, usize, usize, bool, Vec<u32>)> = Vec::new();
        for c in &self.constraints {
            if !c.acts() || c.kind == CKind::Stationary {
                continue;
            }
            let ps = c.params_on(self, None);
            let touched: BTreeSet<usize> = ps.iter().filter_map(|p| owner.get(p).copied()).collect();
            let Some(&first) = touched.first() else { continue };
            // **a contact, or a relation of the drawing**: a row that reads free geometry besides
            // the curve and its own unknowns is satisfied by that geometry — a line drawn tangent
            // to a hanging rope moves to touch it, and the rope hangs as it would — so it holds
            // the energy to nothing and takes no multiplier.  One whose every other column is
            // held presses on the curve: the rope is pushed to touch a line that cannot move.
            // "Held" is the `fix` bit: a line whose free coordinates other rows pin down all the
            // same is read as free, and its tangency reported unsolvable rather than pressing.
            let own = c.aux_params();
            let elsewhere = ps.iter().any(|p| {
                !curve.contains(p) && !own.contains(p) && !self.params[*p as usize].fixed
            });
            if elsewhere {
                continue;
            }
            let tag = hessian_tag(self, c);
            for &s in &touched {
                sets.union(first, s);
            }
            let free: Vec<u32> = own.into_iter().filter(|&p| !self.params[p as usize].fixed).collect();
            reading.push((c.id, c.rows_in(self), first, tag.is_err(), free));
            if let Err(why) = tag {
                faults.push((c.id, why));
            }
        }
        let mut groups: BTreeMap<usize, Group> = BTreeMap::new();
        for &i in &stat {
            let c = &self.constraints[i];
            let s = c.args[0].ent().i();
            let g = groups.entry(sets.find(s)).or_default();
            g.members.push(c.id);
            let Arg::Str(text) = &c.args[2] else { continue };
            // refused at elaboration; a document read carries what it was given, so a text
            // that does not compile is an integrand of nothing rather than a panic
            let f = integrand(text, self.units).ok();
            let sign = if maximizes(c) { -1.0 } else { 1.0 };
            g.degree = f.as_ref().map_or(g.degree, |f| f.degree);
            g.terms.push(Term {
                spline: s,
                coef: sign * c.args[1].num(),
                tapes: f.map(|f| f.tapes.map(|t| t.flat)).unwrap_or_default(),
            });
        }
        for (&s, ys) in &y_of {
            let g = groups.get_mut(&sets.find(s)).expect("a varied spline is in a group");
            g.splines.push(s);
            g.y.extend(ys);
        }
        // a row's own unknown (a contact's place on the curve) is the energy's to vary too: held
        // by the row alone, with no stationarity of its own it would be a freedom the drawing
        // does not have
        let mut own_seen = BTreeSet::new();
        for (id, n, s, refused, free) in reading {
            let g = groups.get_mut(&sets.find(s)).expect("a row read is in a group");
            g.rows.push((id, n));
            g.refused |= refused;
            g.y.extend(free.into_iter().filter(|&p| own_seen.insert(p)));
        }
        // the columns, and the multipliers — one per residual of each row the group reads, kept
        // where they were
        let mut used = BTreeSet::new();
        let mut out = Vec::with_capacity(groups.len());
        for (_, mut g) in groups {
            g.leader = *g.members.iter().min().expect("a group has an energy");
            let mut cols = g.y.clone();
            let mut seen: BTreeSet<u32> = g.y.iter().copied().collect();
            let read = g.splines.iter().map(|&s| self.entity_params(EntRef::spline(s)))
                .chain(g.rows.iter().filter_map(|&(r, _)| self.constraint(r)).map(|c| c.params_on(self, None)));
            for ps in read.collect::<Vec<_>>() {
                cols.extend(ps.into_iter().filter(|&p| seen.insert(p)));
            }
            g.lam0 = cols.len();
            for &(r, n) in &g.rows {
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

    /// The group whose rows constraint `cid` carries, when it leads one that states any — and
    /// whose every row and term is still in the sketch: a caller that filters the constraints in
    /// place (the conflict search) is left an energy that states nothing rather than one that
    /// names rows it no longer has.
    pub fn leads(&self, cid: u32) -> Option<&Group> {
        let g = self.variational.iter().find(|g| g.leader == cid && !g.refused)?;
        let here = |id: &u32| self.constraint(*id).is_some();
        (g.members.iter().all(here) && g.rows.iter().all(|(id, _)| here(id))).then_some(g)
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
    let g = sk.leads(cid);
    Kernel {
        name: "stationary",
        n_res: g.map_or(0, |g| g.y.len()),
        n_par: g.map_or(0, |g| g.cols.len()),
        n_const: pack(sk, cid).len(),
        degree: g.map_or(0, |g| g.degree),
        res: stationary_res,
        jac: stationary_jac,
        const_jac: None,
    }
}

/// Everything the group's kernel reads besides its columns: a header `[n_y, n_terms, n_rows]`;
/// per term `[coef, n, colmap (2n), n_nodes, nodes (n_nodes × NODE_W), 4 × (len, tape…)]`; per
/// row `[tag, n_res, n_par, n_const, λ at, colmap (n_par), constants (n_const)]` — a column map
/// being where each column stands among the group's.
pub fn pack(sk: &Sketch, cid: u32) -> Vec<f64> {
    let Some(g) = sk.leads(cid) else { return Vec::new() };
    let index: BTreeMap<u32, usize> = g.cols.iter().enumerate().map(|(i, &p)| (p, i)).collect();
    let colmap = |ps: Vec<u32>| ps.into_iter().map(|p| index[&p] as f64);
    let mut k = vec![g.y.len() as f64, g.terms.len() as f64, g.rows.len() as f64];
    for t in &g.terms {
        let sp = &sk.splines[t.spline];
        let nodes = integral::nodes(&sp.knots, sp.weights.as_deref(), sp.ctrl.len());
        k.extend([t.coef, sp.ctrl.len() as f64]);
        k.extend(colmap(sk.entity_params(EntRef::spline(t.spline))));
        k.push(nodes.len() as f64);
        integral::write_nodes(&nodes, 0, &mut k);
        for tape in &t.tapes {
            k.push(tape.len() as f64);
            k.extend_from_slice(tape);
        }
    }
    let mut at = g.lam0;
    for &(r, n) in &g.rows {
        let c = sk.constraint(r).expect("a row read is in the sketch");
        let ps = c.params_on(sk, None);
        let consts = c.consts_on(sk, None);
        k.push(hessian_tag(sk, c).unwrap_or(f64::NAN));
        k.extend([n as f64, ps.len() as f64, consts.len() as f64, at as f64]);
        k.extend(colmap(ps));
        k.extend(consts);
        at += n;
    }
    k
}

/* -- evaluation --------------------------------------------------------------------------- */

/// One energy term, read back out of the constants.
struct TermIn<'a> {
    coef: f64,
    map: &'a [f64],
    nodes: Vec<Node>,
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
    terms: Vec<TermIn<'a>>,
    rows: Vec<Row<'a>>,
}

fn read(k: &[f64]) -> Option<Packed<'_>> {
    let mut at = 0usize;
    let mut take = |n: usize| -> Option<&[f64]> {
        let s = k.get(at..at + n)?;
        at += n;
        Some(s)
    };
    let h = take(3)?;
    let (n_y, n_terms, n_rows) = (h[0] as usize, h[1] as usize, h[2] as usize);
    let mut terms = Vec::with_capacity(n_terms);
    for _ in 0..n_terms {
        let h = take(2)?;
        let (coef, n) = (h[0], h[1] as usize);
        let map = take(2 * n)?;
        let n_nodes = take(1)?[0] as usize;
        let nodes = integral::read_nodes(take(n_nodes * integral::NODE_W)?);
        let mut tapes: [&[f64]; 4] = [&[]; 4];
        for t in tapes.iter_mut() {
            let len = take(1)?[0] as usize;
            *t = take(len)?;
        }
        terms.push(TermIn { coef, map, nodes, tapes });
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

/// The values of the group's columns `map` names.
fn gather(map: &[f64], v: &[f64]) -> Vec<f64> {
    map.iter().map(|&c| v[c as usize]).collect()
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

    /// The row's Jacobian at the group's columns `v` (`n_res × n_par`).
    fn jacobian(&self, v: &[f64]) -> Vec<f64> {
        let kn = self.kernel();
        let mut j = vec![0.0; kn.n_res * kn.n_par];
        kn.jac_into(&gather(self.map, v), self.consts, &mut j);
        j
    }
}

/// The varied coordinate column `c` is, if it is one.
fn varied(n_y: usize, c: f64) -> Option<usize> {
    (c as usize).lt(&n_y).then_some(c as usize)
}

/// `∂E/∂y` at the group's columns `v`, into `r` (`n_y`).
fn energy_gradient(p: &Packed, v: &[f64], r: &mut [f64]) {
    let mut s = tape::Scratch::new();
    for t in &p.terms {
        let pts = gather(t.map, v);
        for q in &t.nodes {
            let (c, d) = q.frame(&pts);
            let val = tape::eval_flat(t.tapes[0], 4, &[c[0], c[1], d[0], d[1]], &mut s);
            for a in 0..crate::curve::SPAN_N {
                for i in 0..2 {
                    if let Some(y) = varied(p.n_y, t.map[2 * (q.first + a) + i]) {
                        r[y] += t.coef * q.w * (q.b[a] * val.d[i] + q.d[a] * val.d[2 + i]);
                    }
                }
            }
        }
    }
}

/// `∂g/∂y` of every row, a row per residual in multiplier order (`n_λ × n_y`).
fn constraint_gradient(p: &Packed, v: &[f64]) -> Mat {
    let n_lam: usize = p.rows.iter().map(|r| r.n_res).sum();
    let mut g = Mat::zeros(n_lam, p.n_y);
    let mut l = 0;
    for row in &p.rows {
        let j = row.jacobian(v);
        let m = row.map.len();
        for kk in 0..row.n_res {
            for (c, &col) in row.map.iter().enumerate() {
                if let Some(y) = varied(p.n_y, col) {
                    g.data[(l + kk) * p.n_y + y] += j[kk * m + c];
                }
            }
        }
        l += row.n_res;
    }
    g
}

fn stationary_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let Some(p) = read(k).filter(|_| n == 1) else {
        r.fill(f64::NAN);
        return;
    };
    r.fill(0.0);
    energy_gradient(&p, v, r);
    let g = constraint_gradient(&p, v);
    let lam0 = p.rows.first().map_or(v.len(), |row| row.lam);
    for l in 0..g.rows {
        for y in 0..p.n_y {
            r[y] += v[lam0 + l] * g.data[l * p.n_y + y];
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
    let mut add = |row: f64, col: usize, f: f64| {
        if let Some(y) = varied(n_y, row) {
            out[y * n_par + col] += f;
        }
    };
    // the energy's Hessian, node by node: the integrand's in (C, C') by the chain rule, each a
    // sum of the control points times the basis there
    let mut s = tape::Scratch::new();
    for t in &p.terms {
        let pts = gather(t.map, v);
        for q in &t.nodes {
            let (c, d) = q.frame(&pts);
            let x = [c[0], c[1], d[0], d[1]];
            let mut h = [[0.0; 4]; 4];
            for (kv, row) in h.iter_mut().enumerate() {
                let ser = tape::eval_series_flat(t.tapes[kv], 4, &ORDER[kv].map(|j| x[j]), &mut s);
                for (jj, &var) in ORDER[kv].iter().enumerate() {
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
                            let col = t.map[2 * (q.first + b) + jx] as usize;
                            add(t.map[2 * (q.first + a) + i], col, t.coef * q.w * f);
                        }
                    }
                }
            }
        }
    }
    // each row: its gradient where its multiplier is, and its multipliers' Hessian
    for row in &p.rows {
        let x = gather(row.map, v);
        let m = x.len();
        let j = row.jacobian(v);
        for kk in 0..row.n_res {
            for (c, &col) in row.map.iter().enumerate() {
                add(col, row.lam + kk, j[kk * m + c]);
            }
        }
        let lam = &v[row.lam..row.lam + row.n_res];
        let need: Vec<bool> = row.map.iter().map(|&c| varied(n_y, c).is_some()).collect();
        let mut hl = vec![0.0; m * m];
        row_hessian(row.tag, &x, row.consts, lam, &need, &mut hl);
        for a in (0..m).filter(|&a| need[a]) {
            for b in 0..m {
                add(row.map[a], row.map[b] as usize, hl[a * m + b]);
            }
        }
    }
}

/// `Σ_k λ_k ∂²g_k` over a row's own columns into `h` (`m × m`), rows `a` with `need[a]` only.
fn row_hessian(tag: f64, x: &[f64], kc: &[f64], lam: &[f64], need: &[bool], h: &mut [f64]) {
    let m = x.len();
    if tag < 0.0 {
        // a spline's length: the free column, where there is one, enters linearly
        let (n, free) = integral::length_widths(m);
        let (_, nodes) = integral::length_parts(free, kc);
        integral::length_hess(&nodes, &x[..2 * n], lam[0], None, h, m);
        return;
    }
    let kid = tag as usize;
    if kid == K::SplineGauge as usize {
        let nodes = integral::read_nodes(kc);
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
        if !crate::taylor::along(kid, x, w, kc, &mut path, &mut r, &mut jrow) {
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
            // symmetric: an entry another needed row already has is read, not worked out again
            if b < a && need[b] {
                h[a * m + b] = h[b * m + a];
                continue;
            }
            w[a] = 1.0;
            w[b] = 1.0;
            h[a * m + b] = s(&w) - one[a] - one[b];
            w[a] = 0.0;
            w[b] = 0.0;
        }
    }
}

/// A group's leading constraint's constants, unpacked, and its columns' values at the sketch's
/// pose — what the seed and the verdict read the stationarity through.
fn at_pose(sk: &Sketch, g: &Group) -> Option<(Vec<f64>, Vec<f64>)> {
    if g.y.is_empty() {
        return None;
    }
    let v = g.cols.iter().map(|&p| sk.params[p as usize].value).collect();
    Some((pack(sk, g.leader), v))
}

/* -- multipliers -------------------------------------------------------------------------- */

/// Each group's multipliers where they best answer its stationarity at the current pose: the
/// least-squares `λ` of `∂E/∂y + (∂g/∂y)ᵀ λ = 0` — the multipliers *are* a function of the pose at
/// a solution, so this is the start a solve deserves, and zero (the gradient alone) is far from it.
/// Gradients only: no Hessian is asked for.
pub fn seed_multipliers(sk: &mut Sketch) {
    for g in sk.variational.clone() {
        let Some((k, v)) = at_pose(sk, &g) else { continue };
        let Some(p) = read(&k) else { continue };
        let mut grad = vec![0.0; p.n_y];
        energy_gradient(&p, &v, &mut grad);
        if grad.iter().any(|x| !x.is_finite()) {
            continue;
        }
        let b: Vec<f64> = grad.iter().map(|x| -x).collect();
        let (lam, _) = crate::linalg::min_norm_solve(&constraint_gradient(&p, &v).transpose(), &b, 1e-12);
        for (l, &q) in g.cols[g.lam0..].iter().enumerate() {
            if lam[l].is_finite() {
                sk.params[q as usize].value = lam[l];
            }
        }
    }
}

/* -- a free curve's start ----------------------------------------------------------------- */

/// The energy a group's terms come to at the sketch's pose.
fn energy(sk: &Sketch, g: &Group) -> f64 {
    let Some((k, v)) = at_pose(sk, g) else { return 0.0 };
    let Some(p) = read(&k) else { return 0.0 };
    let mut s = tape::Scratch::new();
    p.terms.iter().map(|t| {
        let pts = gather(t.map, &v);
        t.nodes.iter().map(|q| {
            let (c, d) = q.frame(&pts);
            t.coef * q.w * tape::eval_flat(t.tapes[0], 4, &[c[0], c[1], d[0], d[1]], &mut s).v
        }).sum::<f64>()
    }).sum()
}

/// Where a free curve's interior starts (`spline(a, b)`): on its chord, bowed off it as far as its
/// stated length asks — a parabola of sagitta `s` is `c + 8s²/3c` long — or a quarter of the chord
/// where none is stated, to whichever side the energy is lower.  An implementation's choice of
/// start, as `program::scatter` is; it picks the branch a hanging rope and Dido's arc hang on.
pub fn seed_free(sk: &mut Sketch) {
    for g in sk.variational.clone() {
        for &si in &g.splines {
            let sp = sk.splines[si].clone();
            if !sp.free {
                continue;
            }
            let n = sp.ctrl.len();
            let (a, b) = (sk.point_xy(sp.ctrl[0] as usize), sk.point_xy(sp.ctrl[n - 1] as usize));
            let c = (b.0 - a.0).hypot(b.1 - a.1);
            if c <= 0.0 {
                continue;
            }
            let normal = (-(b.1 - a.1) / c, (b.0 - a.0) / c);
            let length = sk.constraints.iter()
                .find(|k| k.kind == CKind::SplineLength && k.args[0].ent().i() == si)
                .map(|k| k.args[1].num());
            let sag = match length {
                Some(l) if l > c => (3.0 * c * (l - c) / 8.0).sqrt(),
                Some(_) => 0.0,
                None => c / 4.0,
            };
            let place = |sk: &mut Sketch, side: f64| {
                for (i, &p) in sp.ctrl.iter().enumerate().take(n - 1).skip(1) {
                    let u = i as f64 / (n - 1) as f64;
                    let off = side * 4.0 * sag * u * (1.0 - u);
                    let xy = sk.point_params(p as usize);
                    sk.params[xy[0] as usize].value = a.0 + u * (b.0 - a.0) + off * normal.0;
                    sk.params[xy[1] as usize].value = a.1 + u * (b.1 - a.1) + off * normal.1;
                }
            };
            place(sk, 1.0);
            let left = energy(sk, &g);
            place(sk, -1.0);
            let right = energy(sk, &g);
            if left < right {
                place(sk, 1.0);
            }
        }
    }
}

/* -- the second-order verdict ------------------------------------------------------------- */

/// What a stationary curve is, to second order: read off the inertia of the Lagrangian's Hessian
/// on the motions the rows that hold the curve leave it (`verdict`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Extremum {
    Minimum,
    Maximum,
    Saddle,
    /// A motion the second order cannot see either way: no verdict.
    Degenerate,
}

impl Extremum {
    pub fn name(self) -> &'static str {
        match self {
            Extremum::Minimum => "minimum",
            Extremum::Maximum => "maximum",
            Extremum::Saddle => "saddle",
            Extremum::Degenerate => "degenerate",
        }
    }

    /// The same verdict read for the energy's negation — what a `maximizes` asked about.
    fn turned(self) -> Extremum {
        match self {
            Extremum::Minimum => Extremum::Maximum,
            Extremum::Maximum => Extremum::Minimum,
            other => other,
        }
    }
}

/// The verdict on group `g` at the sketch's pose, of the energy as it is minimised: the reduced
/// Hessian `Zᵀ (∂²E + Σ λ ∂²g) Z`, `Z` a basis of the motions of the curve's coordinates the rows
/// holding it leave free to first order (`∂g/∂y · Z = 0`), positive definite at a minimum,
/// negative at a maximum, of both signs at a saddle.  Its inertia is the same in any basis of
/// those motions (Sylvester), so neither the units nor the basis decide it.  `None` where the rows
/// leave the curve no motion at all, and so nothing for the energy to choose.
pub fn verdict(sk: &Sketch, g: &Group) -> Option<Extremum> {
    let (k, v) = at_pose(sk, g)?;
    let p = read(&k)?;
    let n_y = p.n_y;
    let mut j = vec![0.0; n_y * v.len()];
    stationary_jac(1, &v, &k, &mut j);
    if j.iter().any(|x| !x.is_finite()) {
        return None;
    }
    let z = crate::linalg::rank_and_nullspace(&constraint_gradient(&p, &v), 1e-10).null();
    let dim = z.cols;
    if dim == 0 {
        return None;
    }
    // `Zᵀ H Z` over the y block, made symmetric: the energy's and each row's Hessian are, up to
    // rounding
    let h = |a: usize, b: usize| 0.5 * (j[a * v.len() + b] + j[b * v.len() + a]);
    let hz: Vec<f64> = (0..n_y * dim)
        .map(|i| (0..n_y).map(|b| h(i / dim, b) * z.data[b * dim + i % dim]).sum())
        .collect();
    let mut m = Mat::zeros(dim, dim);
    for p in 0..dim {
        for q in 0..dim {
            m.data[p * dim + q] = (0..n_y).map(|a| z.data[a * dim + p] * hz[a * dim + q]).sum();
        }
    }
    let ev = crate::linalg::sym_eigenvalues(&m);
    let big = ev.iter().fold(0.0f64, |m, e| m.max(e.abs()));
    let tol = 1e-9 * big;
    let (pos, neg) = (ev.iter().filter(|&&e| e > tol).count(), ev.iter().filter(|&&e| e < -tol).count());
    Some(match (pos, neg) {
        (_, _) if big == 0.0 => Extremum::Degenerate,
        (p, 0) if p == dim => Extremum::Minimum,
        (0, q) if q == dim => Extremum::Maximum,
        (p, q) if p > 0 && q > 0 => Extremum::Saddle,
        _ => Extremum::Degenerate,
    })
}

/// Every energy statement's verdict, by constraint, as each asked it: a `maximizes` that found a
/// maximum is answered `maximum`.
pub fn extrema(sk: &Sketch) -> Vec<(u32, Extremum)> {
    let mut out = Vec::new();
    for g in &sk.variational {
        let Some(v) = verdict(sk, g) else { continue };
        for &m in &g.members {
            let max = sk.constraint(m).is_some_and(maximizes);
            out.push((m, if max { v.turned() } else { v }));
        }
    }
    out
}

/// What energy constraint `cid`'s statement asked for: a minimum, or — `maximizes` — a maximum.
pub fn asked(sk: &Sketch, cid: u32) -> Extremum {
    match sk.constraint(cid).is_some_and(maximizes) {
        true => Extremum::Maximum,
        false => Extremum::Minimum,
    }
}
