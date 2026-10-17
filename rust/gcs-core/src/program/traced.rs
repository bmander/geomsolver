//! **Bodies traced in space** (#326, R3 and R5): a component's point as one or two of its formals
//! run, where its body is solved in space — compiled to a `traced::Body` and built as a
//! `TracedE`.  A curve is traced in space where it runs over two formals, where what it is
//! written over stands in space (a point or a line drawn in no plane), or where it is given a
//! motion; every other curve is §6.5's, traced in its plane (`curves.rs`).
//!
//! The body is lowered once per instance through a scratch sketch in space: every point is a
//! point in space, its own lift, so a relation's columns are what `Constraint::params` says of
//! its kind in space (`reading::in_space` reads a word between points in space as the relation
//! there), each mapped into the variable table `[formals, θ, values] ++ q ++ w`.  A dimension
//! over the formals is a tape read by its row's free twin (`w`), a place pinned over them a tape
//! too, and a point a motion carries (`X coincident(under: m, t == −phi) x`) a row of the
//! motion's steps over its axes' columns.  A dimension that reads the body's own points (`p.x`,
//! #378) is a **placed number**: one of `q`'s own, held to its expression by a row
//! (`traced::RowKind::Number`), read by the free twin where a tape would be.
use super::reading::{in_space, Reader};
use super::relations::{ent_arg, scalar_arg};
use super::resolve::{follow, Resolver};
use super::Code;
use crate::constraints::{Arg as CArg, CKind, Constraint, OrdinateForm, SpecKind};
use crate::ir::{Decl, Operation as StmtKind, Statement as Stmt};
use crate::model::{EntKind, EntRef, Sketch, TracedE};
use crate::syntax::{Arg, Program, Span, Ty};
use std::collections::BTreeMap;

type Fail = (Code, Span, String);

/// Whether a curve declaration is traced in space whatever its body says: two formals run, or
/// the instance is given a frame that moves (#383).  A body over one formal and no motion is traced
/// in its plane where it reads as one, and in space where it does not (`curves::build_curve`).
pub(super) fn in_space_traced(sk: &Sketch, res: &Resolver, d: &Decl,
                              insts: &[crate::flatten::InstanceInfo]) -> bool {
    let Some(c) = d.curve.as_ref() else { return false };
    if c.second.is_some() {
        return true;
    }
    let Some(of) = &c.of else { return false };
    let Some(info) = insts.iter().find(|i| i.prefix == of.instance) else { return false };
    info.ents.iter().filter_map(|(_, a)| a.as_ref()).filter_map(|a| res.of.get(a))
        .any(|e| e.kind == EntKind::Frame
            && sk.frames.get(e.i()).is_some_and(|f| f.joint.is_some()))
}

/// A traced body's place in the sketch before what it is written over is all built: its name
/// and class, its body compiled once the motions are (`traced_bodies`).
pub(super) fn placeholder(d: &Decl) -> TracedE {
    TracedE { body: crate::traced::Body { flat: Vec::new() }, args: Vec::new(), values: Vec::new(),
              over: Vec::new(), home: [0.0; 2], name: d.name.key().text.clone(),
              class: d.class.clone() }
}

/// Every body traced in space compiled, once the motions it may be written over are built and
/// before any relation reads one (#326): each placeholder `build_curve` left replaced, or the
/// statement refused saying why.
pub(super) fn traced_bodies(sk: &mut Sketch, res: &Resolver, body: &[&Stmt], prog: &Program,
                            insts: &[crate::flatten::InstanceInfo], diags: &mut Vec<super::Diag>) {
    for st in body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        if d.kind != EntKind::Curve {
            continue;
        }
        let Some(e) = res.of.get(&d.name.key().text).filter(|e| e.kind == EntKind::Traced) else {
            continue;
        };
        if e.i() >= sk.traced.len() || !sk.traced[e.i()].body.flat.is_empty() {
            continue;
        }
        match build_traced(sk, res, d, st, prog, insts) {
            Ok(Some(t)) => sk.traced[e.i()] = t,
            Ok(None) => {}
            Err((code, span, message)) => {
                diags.push(super::Diag::within(code, span, st.id, message))
            }
        }
    }
}

/// The traced body a curve declaration draws in space.  `Ok(None)` where the flattener already
/// said what is wrong with it.
pub(super) fn build_traced(sk: &Sketch, res: &Resolver, d: &Decl, st: &Stmt, prog: &Program,
                           insts: &[crate::flatten::InstanceInfo])
                           -> Result<Option<TracedE>, Fail> {
    let Some(c) = d.curve.as_ref() else { return Ok(None) };
    let Some(of) = &c.of else { return Ok(None) };
    let Some(info) = insts.iter().find(|i| i.prefix == of.instance) else { return Ok(None) };
    if info.drawn {
        return Err((Code::E103, st.span, "a body traced in space is written in place, \
            `Component(…).point over …`: an instance drawn on the sheet stands in its plane"
            .into()));
    }
    let comp = &prog.components[info.comp];
    let mut running: Vec<(&crate::syntax::Name, &(String, String))> = vec![(&c.swept, &c.domain)];
    if let Some((s, dom)) = &c.second {
        running.push((s, dom));
    }
    for (f, _) in &running {
        let numeric = comp.formals.iter().any(|x| {
            x.name.text == f.text && !matches!(x.ty, Ty::Ent(_) | Ty::Path | Ty::Group | Ty::Set)
        });
        if !numeric {
            return Err((Code::E040, f.span, format!("`{}` is not a numeric formal of `{}`",
                f.text, info.component)));
        }
    }
    if running.len() == 2 && running[0].0.text == running[1].0.text {
        return Err((Code::E040, running[1].0.span,
                    "a surface runs over two different formals".into()));
    }
    // the variable table: the formals that run, the geometry's coordinates, then the numbers
    let mut vars: Vec<String> = running.iter().map(|(f, _)| f.text.clone()).collect();
    let mut formals: Vec<Formal> = Vec::new();
    let mut args: Vec<EntRef> = Vec::new();
    let mut values: Vec<String> = Vec::new();
    let mut ents = info.ents.iter();
    for fo in &comp.formals {
        let name = fo.name.text.as_str();
        match fo.ty {
            Ty::Ent(k) => {
                let (_, actual) = ents.next().ok_or_else(|| missing(st, &info.component, name))?;
                let abs = actual.as_ref().ok_or_else(|| missing(st, &info.component, name))?;
                let e = *res.of.get(abs)
                    .ok_or_else(|| (Code::E101, st.span, format!("no such entity: `{abs}`")))?;
                // a `curve` is any curve the drawing has: a spline, or one traced in space (#376)
                let curve = k == EntKind::Curve
                    && matches!(e.kind, EntKind::Spline | EntKind::Traced);
                if e.kind != k && !curve {
                    return Err((Code::E040, st.span, format!("`{name}` is {}, and {} was given",
                        k.a(), e.kind.a())));
                }
                let k = e.kind;
                let mut steps = Vec::new();
                let mut drawn = None;
                let width = match k {
                    EntKind::Point => {
                        vars.extend(["x", "y", "z"].map(|c| format!("{name}.{c}")));
                        3
                    }
                    EntKind::Line => {
                        for p in ["p1", "p2"] {
                            vars.extend(["x", "y", "z"].map(|c| format!("{name}.{p}.{c}")));
                        }
                        6
                    }
                    // a frame its motion's steps (#383): its joint's, or none at rest — the world,
                    // as an observer it leaves a reading alone
                    EntKind::Frame => match sk.frames[e.i()].joint.as_ref().and_then(|j| j.motion) {
                        Some(m) => {
                            let m = crate::envelope::held::Moving::of_any(sk, m as usize)
                                .map_err(|why| (Code::E103, st.span, why))?;
                            let w = m.width();
                            vars.extend((0..w).map(|i| format!("{name}#{i}")));
                            steps = m.steps;
                            w
                        }
                        None => 0,
                    },
                    // a curve the drawing has, which a marched body's Cauchy data reads (#376)
                    EntKind::Spline | EntKind::Curve | EntKind::Traced => {
                        let d = Drawn::of(sk, e).map_err(|m| (Code::E103, st.span, format!(
                            "`{name}`: {m}")))?;
                        let w = d.width;
                        vars.extend((0..w).map(|i| format!("{name}#{i}")));
                        drawn = Some(d);
                        w
                    }
                    other => {
                        return Err((Code::E103, st.span, format!("a body traced in space is \
                            written over points, lines, motions and curves, not {}", other.a())));
                    }
                };
                let at = vars.len() - width;
                formals.push(Formal { name: name.to_string(), kind: k, at, steps, drawn });
                args.push(e);
            }
            Ty::Path | Ty::Group | Ty::Set => {
                return Err((Code::E103, fo.span, "a body traced in space is written over \
                    points, lines, motions and numbers".into()));
            }
            _ if running.iter().any(|(f, _)| f.text == name) => {}
            _ => values.push(name.to_string()),
        }
    }
    let n_theta = vars.len() - running.len();
    let numbers: Vec<f64> = values.iter().map(|n| {
        info.values.get(n).and_then(|a| a.number()).ok_or_else(|| (Code::E103, st.span, format!(
            "`{}` is not given `{n}` as a number: a body traced in space reads the drawing's \
             unknowns only through the geometry it is written over", info.component)))
    }).collect::<Result<_, _>>()?;
    vars.extend(values.iter().cloned());
    // the body, expanded symbolically: every number a free value named after itself
    let ex = crate::flatten::expand_component(prog, comp, sk.units);
    if let Some(dg) = ex.diagnostics.iter().find(|d| d.severity() == super::Severity::Error) {
        return Err((Code::E103, dg.span, dg.message.clone()));
    }
    let traced = ex.aliases.get(&of.point).cloned().unwrap_or_else(|| of.point.clone());
    let body: Vec<&Stmt> = ex.flat.iter().collect();
    let mut low = Lowering::new(sk.units, &vars, formals);
    let angle = |name: &str| comp.formals.iter().any(|f| f.name.text == name && f.ty == Ty::Angle);
    low.swept = running[0].0.text.clone();
    low.angle = angle(&low.swept);
    // over two formals the second is sampled, a rate along it read off the nodes (#339 D4)
    low.second = running.get(1).map(|(f, _)| f.text.clone());
    let degrees = 180.0 / std::f64::consts::PI;
    let per = std::array::from_fn(|k| match running.get(k) {
        Some((f, _)) if angle(&f.text) => degrees,
        _ => 1.0,
    });
    for st in &body {
        if let StmtKind::Decl(d) = &st.kind {
            low.declare(st, d)?;
        }
    }
    for st in &body {
        match &st.kind {
            StmtKind::Decl(_) => continue,
            // a relation that reads a rate is a row of the rates' system, one written in
            // `initially` a row of the home's (#339 D2)
            StmtKind::Relation(r) if r.reads_rate() => {
                let (r, marched) = low.rate_relation(st, r)?;
                // a row that reads only rates along the sampled formal is algebraic at each step:
                // the rates are the nodes' (#339 D4)
                low.target = if marched { super::curves::Target::Rate }
                             else { super::curves::Target::Alg };
                low.relation(st, &r)?;
            }
            StmtKind::Relation(r) => {
                low.target = if r.initially { super::curves::Target::Init }
                             else { super::curves::Target::Alg };
                low.relation(st, r)?;
            }
            _ => return Err((Code::E103, st.span,
                "a traced component holds declarations and constraints only".into())),
        }
    }
    low.settle_numbers();
    let at = vars.len() + low.n_q + low.w.len();
    super::curves::place_rates(low.rates.iter_mut().chain(&mut low.rows).map(|r| &mut r.cols), at,
                               low.n_q);
    let Some(&EntRef { kind: EntKind::Point, idx }) = low.scope.get(&traced) else {
        return Err((Code::E103, comp.span, format!("`{}` must be a point the component declares",
            of.point)));
    };
    let first = low.sk.point_params(idx as usize)[0];
    let t = match low.slot.get(&first) {
        Some(&s) if s >= vars.len() => s - vars.len(),
        _ => return Err((Code::E103, comp.span, format!("`{}` is geometry the component is \
            written over, and does not move with what runs", of.point))),
    };
    let num = |t: &str| crate::expr::literal(t);
    // each formal that runs, one parameter axis (#391 S1)
    let mut over: Vec<crate::model::Running> = Vec::with_capacity(running.len());
    for (f, (a, b)) in &running {
        let (Some(lo), Some(hi)) = (num(a), num(b)) else {
            return Err((Code::E103, f.span, "a traced formal's interval is two numbers".into()));
        };
        if lo >= hi {
            return Err((Code::E103, f.span,
                        "a traced formal runs over an increasing interval".into()));
        }
        let dim = comp.formals.iter().find(|x| x.name.text == f.text)
            .map_or(crate::units::Dim::SCALAR, |x| x.ty.dim());
        over.push(crate::model::Running::new(f.text.clone(), dim, lo, hi));
    }
    if !low.rates.is_empty() && low.second.is_some() {
        low.marched(at, t, comp.span)?;
    } else if !low.rates.is_empty() {
        let alg = super::curves::reach(low.rows.iter().map(|r| (r.n_res(), &r.cols[..])),
                                       vars.len(), low.n_q);
        let rates = super::curves::reach(low.rates.iter().map(|r| (r.n_res(), &r.cols[..])), at,
                                         low.n_q);
        if let Some(m) = super::curves::short(&low.swept, low.n_q, &alg, &rates, &low.owner, false)
        {
            return Err((Code::E103, comp.span, m));
        }
    } else if let Some(span) = low.sampled {
        return Err((Code::E103, span, format!("a rate along `{}` is read off the nodes of a body \
            marched along `{}`, and this body states no rate along `{}`: say how it moves",
            low.second.as_deref().unwrap_or_default(), low.swept, low.swept)));
    }
    let two = low.second.is_some();
    let rates = crate::traced::Rates { rows: std::mem::take(&mut low.rates),
        init: std::mem::take(&mut low.init), domain: over[0].span(),
        second: if two { over[1].span() } else { [0.0; 2] }, per: if two { per } else { [1.0; 2] },
        pieces: if two { crate::lines::PIECES } else { 0 } };
    let flat = crate::traced::Body::new(running.len(), vars.len(), n_theta, low.n_q, t,
        low.w, low.seeds, low.rows, rates).map_err(|m| (Code::E103, comp.span, m))?;
    // where a march starts: what the call gave each formal, else its interval's start
    let home = std::array::from_fn(|k| match running.get(k) {
        Some((f, _)) => info.values.get(&f.text).and_then(|a| a.number()).unwrap_or(over[k].from),
        None => 0.0,
    });
    Ok(Some(TracedE { body: flat, args, values: numbers, over, home,
        name: d.name.key().text.clone(), class: d.class.clone() }))
}

fn missing(st: &Stmt, comp: &str, what: &str) -> Fail {
    (Code::E101, st.span, format!("`{comp}` was not given `{what}`"))
}

/// An entity formal: its name, its kind, where its coordinates start in the variable table and,
/// a motion, its steps; a curve, what a point on it reads.
struct Formal {
    name: String,
    kind: EntKind,
    at: usize,
    steps: Vec<crate::envelope::held::Step>,
    drawn: Option<Drawn>,
}

/// **A curve the drawing has, as a traced body reads it** (#376): what a point on it at a number
/// of the formals is lowered to (`Lowering::on_drawn`) — a spline's row over its control points'
/// lifts, a curve drawn in a held plane read in space, or a curve traced in space's contact —
/// how many columns of the variable table it reads, the row's constants, and the curve's kind.
#[derive(Clone, Debug)]
struct Drawn {
    row: crate::traced::RowKind,
    width: usize,
    consts: Vec<f64>,
    kind: EntKind,
}

impl Drawn {
    fn of(sk: &Sketch, e: EntRef) -> Result<Drawn, String> {
        use crate::traced::RowKind;
        match e.kind {
            EntKind::Spline => {
                let s = &sk.splines[e.i()];
                let n = s.ctrl.len();
                let mut consts = s.knots.clone();
                consts.extend(s.weights.clone().unwrap_or_else(|| vec![1.0; n]));
                Ok(Drawn { row: RowKind::Spline, width: 3 * n, consts, kind: e.kind })
            }
            EntKind::Curve => {
                let i = e.i();
                let cv = &sk.curves[i];
                let d = &sk.curve_defs[cv.def as usize];
                use crate::model::CurveBody as B;
                let body = match &d.body {
                    B::Exprs { .. } => crate::kernels::FORMULA,
                    B::Trace(_) => crate::kernels::TRACE,
                    B::Envelope(_) => crate::kernels::ENVELOPE,
                    B::Extremal(_) => crate::kernels::EXTREMAL,
                    B::Followed => return Err("a point traced over a parameter is drawn, and no \
                        marched body starts on it (#391)".into()),
                };
                if cv.extrusion || sk.curve_turn(i).is_some() {
                    return Err("a curve standing for a surface, or a ring's turned copy, is no \
                        curve a marched body starts on".into());
                }
                if sk.curve_view(i).is_some_and(|v| !sk.plane_fixed(v)) {
                    return Err("a curve drawn in a plane is read in space through its plane's \
                        basis, which must be held: `fix` its plane's axes and origin".into());
                }
                let width = sk.entity_params(e).len();
                if width + 1 + d.values.len() != d.vars.len() {
                    return Err("its columns are not its definition's".into());
                }
                let mut consts = vec![body as f64];
                consts.extend(sk.extrusion_frame(i));
                consts.extend(crate::constraints::curve_contact_consts(sk, i));
                Ok(Drawn { row: RowKind::Curve, width, consts, kind: e.kind })
            }
            _ => {
                let t = sk.traced.get(e.i()).filter(|t| !t.body.flat.is_empty())
                    .ok_or("a curve traced in space is read once it is drawn: declare it first")?;
                if t.body.n_formals() != 1 {
                    return Err("a surface is no curve: a point on a curve is said at one number"
                        .into());
                }
                Ok(Drawn { row: RowKind::Traced, width: t.body.n_theta(),
                           consts: crate::traced::consts_of(t), kind: e.kind })
            }
        }
    }
}

/// A body being lowered in space.
struct Lowering<'v> {
    sk: Sketch,
    scope: BTreeMap<String, EntRef>,
    /// The frames it is written over, by name: their motions' steps and the columns they read —
    /// none for a frame at rest — and a reading's frame seen from another (#383).
    motions: BTreeMap<String, (Vec<crate::envelope::held::Step>, Vec<usize>)>,
    /// The curves it is written over (#376), by name: what a point on one reads, and where its
    /// columns start.
    drawn: BTreeMap<String, (Drawn, usize)>,
    /// scratch parameter → variable-table slot
    slot: BTreeMap<u32, usize>,
    vars: &'v [String],
    n_outer: usize,
    n_q: usize,
    seeds: Vec<crate::tape::Tape>,
    w: Vec<crate::tape::Tape>,
    rows: Vec<crate::traced::Row>,
    units: crate::units::Units,
    /// Each number the body places, by the name of the point it is of — what a short body's
    /// report names.
    owner: Vec<String>,
    /// A body that states a rate (#339 D2), as `curves::Lowering` holds one.
    swept: String,
    angle: bool,
    target: super::curves::Target,
    rates: Vec<crate::traced::Row>,
    init: Vec<crate::traced::Row>,
    /// Each point whose rate is read, along the marched formal (`false`) or the sampled one
    /// (`true`), and its scratch line.
    rated: BTreeMap<(usize, bool), String>,
    origin: Option<usize>,
    /// Over two formals, the sampled one (#339 D4), each algebraic and initial row's statement,
    /// and where a rate along the sampled formal is first read.
    second: Option<String>,
    spans: Vec<Span>,
    rate_spans: Vec<Span>,
    init_spans: Vec<Span>,
    sampled: Option<Span>,
    /// Each coordinate of a point the body declares, by name (`p.x`), and its seed as an
    /// expression over the formals — what a placed number reads, and how its own seed is written
    /// (#378).
    own: BTreeMap<String, usize>,
    seed_of: Vec<crate::expr::Ast>,
    /// Each placed number's text and seed, in the order of their slots past the declared
    /// coordinates — placeholders (`NUMBER_BASE`) until `settle_numbers` puts them in `q`.
    numbers: Vec<(String, crate::tape::Tape)>,
}

/// Where a placed number stands until `settle_numbers` puts it past the declared coordinates:
/// below the rates' placeholders (`curves::RATE_BASE`), above every slot of the table.
const NUMBER_BASE: usize = 1 << 19;

/// A dimension as a traced body reads it: a tape over the formals, the geometry and the given
/// numbers (`w`), or a placed number, by its slot (#378).
enum Dimension {
    Tape(crate::tape::Tape),
    Number(usize),
}

impl<'v> Lowering<'v> {
    fn new(units: crate::units::Units, vars: &'v [String], formals: Vec<Formal>) -> Lowering<'v> {
        let mut sk = Sketch::new();
        sk.units = units;
        let mut scope = BTreeMap::new();
        let mut slot = BTreeMap::new();
        let mut motions = BTreeMap::new();
        let mut drawn = BTreeMap::new();
        let point = |sk: &mut Sketch, slot: &mut BTreeMap<u32, usize>, at: usize, name: &str| {
            let p = sk.point3([0.0; 3], false, name);
            sk.lift_point(p);
            for (k, q) in sk.point_all_params(p).into_iter().enumerate() {
                slot.insert(q, at + k);
            }
            p
        };
        for f in formals {
            match f.kind {
                EntKind::Point => {
                    let p = point(&mut sk, &mut slot, f.at, &f.name);
                    scope.insert(f.name.clone(), EntRef::point(p));
                }
                EntKind::Line => {
                    let a = point(&mut sk, &mut slot, f.at, &f.name);
                    let b = point(&mut sk, &mut slot, f.at + 3, &f.name);
                    scope.insert(f.name.clone(), EntRef::line(sk.line(a, b)));
                }
                EntKind::Frame => {
                    let width = crate::envelope::held::steps_width(&f.steps);
                    motions.insert(f.name, (f.steps, (f.at..f.at + width).collect()));
                }
                _ => {
                    if let Some(d) = f.drawn {
                        drawn.insert(f.name, (d, f.at));
                    }
                }
            }
        }
        Lowering { sk, scope, motions, drawn, slot, vars, n_outer: vars.len(), n_q: 0,
                   seeds: Vec::new(), w: Vec::new(), rows: Vec::new(), units, owner: Vec::new(),
                   swept: String::new(), angle: false, target: super::curves::Target::Alg,
                   rates: Vec::new(), init: Vec::new(), rated: BTreeMap::new(), origin: None,
                   second: None, spans: Vec::new(), rate_spans: Vec::new(),
                   init_spans: Vec::new(), sampled: None, own: BTreeMap::new(),
                   seed_of: Vec::new(), numbers: Vec::new() }
    }

    /// A row, to the rows its relation is lowered to, with the statement it is of.
    fn push(&mut self, row: crate::traced::Row, span: Span) {
        match self.target {
            super::curves::Target::Alg => {
                self.rows.push(row);
                self.spans.push(span);
            }
            super::curves::Target::Rate => {
                self.rates.push(row);
                self.rate_spans.push(span);
            }
            super::curves::Target::Init => {
                self.init.push(row);
                self.init_spans.push(span);
            }
        }
    }

    /// A relation that reads a rate, rewritten to one over scratch geometry in space, as
    /// `curves::Lowering::rate_relation` rewrites one in the plane — and whether it reads a rate
    /// along the marched formal.  Over two formals (#339 D4) a rate may be along either, at most
    /// one along the marched: a rate along the sampled one is a number of the state.
    fn rate_relation(&mut self, st: &Stmt, r: &crate::ir::Relation)
        -> Result<(crate::ir::Relation, bool), Fail> {
        let Some(second) = self.second.clone() else {
            let (r, rf, unary) = super::curves::rate_operand(st, r, &self.swept, self.angle)
                .map_err(|(s, m)| (Code::E103, s, m))?;
            let p = self.rated_point(&rf)?;
            let line = self.rate_line(p, rf.span, false)?;
            let k = r.form.written().map_or(0, |w| w.rates[0].op);
            return Ok((super::curves::rate_rewritten(r, &[(k, line, rf.span)], unary), true));
        };
        let w = r.form.written()
            .ok_or((Code::E103, st.span, "a rate is written in a relation".to_string()))?;
        if r.initially {
            return Err((Code::E103, st.span, "a rate is no row at the home: `initially { … }` \
                says what holds there of the instance itself".into()));
        }
        let mut marched = 0;
        for rate in &w.rates {
            if rate.over.text == self.swept {
                marched += 1;
            } else if rate.over.text == second {
                self.sampled.get_or_insert(rate.over.span);
            } else {
                return Err((Code::E103, rate.over.span, format!("`rate(… over {})` is read along \
                    `{}`, and the surface runs over `{}` and `{second}`: a rate is along a \
                    formal the body runs over", rate.over.text, rate.over.text, self.swept)));
            }
        }
        if marched > 1 {
            return Err((Code::E103, st.span, format!("a relation reads one rate along `{}`, the \
                formal it is marched along: two would make the rates' system quadratic, leaving \
                a root to choose at every step", self.swept)));
        }
        let unary = w.fixity == crate::constraints::Fixity::Infix && w.ops.len() == 1;
        let mut lines = Vec::new();
        for rate in &w.rates {
            let rf = &w.ops[rate.op];
            let p = self.rated_point(rf)?;
            lines.push((rate.op, self.rate_line(p, rf.span, rate.over.text != self.swept)?,
                        rf.span));
        }
        let out = super::curves::rate_rewritten(r.clone(), &lines, unary);
        Ok((out, marched == 1))
    }

    /// **A body over two formals that states a rate, split** (#339 D4, `lines.rs`): the points
    /// whose rate along the marched formal a rate row reads are integrated, the rest placed by
    /// the rows that read them at each step — each statement checked against the split, naming
    /// what is wrong.  `at` is where the rates' slots start, `t` the traced point's first number.
    fn marched(&self, at: usize, t: usize, span: Span) -> Result<(), Fail> {
        let (n0, n, u) = (self.n_outer, self.n_q, self.second.clone().unwrap_or_default());
        let d = &self.swept;
        let reads =
            |cols: &[u32], from: usize, k: usize| cols.iter().any(|&c| c as usize == from + k);
        let any =
            |cols: &[u32], from: usize, set: &[usize]| set.iter().any(|&k| reads(cols, from, k));
        let moved: Vec<usize> =
            (0..n).filter(|&k| self.rates.iter().any(|r| reads(&r.cols, at, k))).collect();
        let placed: Vec<usize> = (0..n).filter(|k| !moved.contains(k)).collect();
        let names = |ks: &[usize]| -> String {
            let mut v: Vec<&str> = Vec::new();
            for &k in ks {
                if let Some(o) = self.owner.get(k).filter(|o| !v.contains(&o.as_str())) {
                    v.push(o);
                }
            }
            v.iter().map(|o| format!("`{o}`")).collect::<Vec<_>>().join(", ")
        };
        let first = |cols: &[u32], from: usize, set: &[usize]| {
            set.iter().copied().find(|&k| reads(cols, from, k))
        };
        let stated =
            self.rows.iter().zip(&self.spans).chain(self.rates.iter().zip(&self.rate_spans));
        for (r, sp) in stated {
            if let Some(k) = first(&r.cols, at + n, &placed) {
                return Err((Code::E103, *sp, format!("`rate({} over {u})` is of a point no rate \
                    row moves along `{d}`: a rate along the sampled formal is read off the nodes \
                    of a point the rates move", self.owner[k])));
            }
        }
        // a placed number (#378) is placed, last of all, by its own row
        let numbers: Vec<usize> = (n - self.numbers.len()..n).collect();
        let others: Vec<usize> = placed.iter().copied().filter(|k| !numbers.contains(k)).collect();
        let mut over_moved = Vec::new();
        let mut places = 0;
        for (r, sp) in self.rows.iter().zip(&self.spans) {
            let n_res = r.n_res();
            if r.kind != crate::traced::RowKind::Number && any(&r.cols, n0, &numbers)
                && !any(&r.cols, n0, &others) {
                return Err((Code::E103, *sp, format!("this relation's dimension reads the points \
                    the rates move, and it places only them: over two formals a number read at \
                    each step stands in a rate row, or in a row that places a point no rate row \
                    moves along `{d}`")));
            }
            if any(&r.cols, n0, &placed) {
                places += n_res;
            } else if any(&r.cols, at + n, &moved) {
                return Err((Code::E103, *sp, format!("this relation reads a rate along `{u}` and \
                    places only points the rates move: differentiated along `{d}` it would tie \
                    every node of a piece together — state it of a point the body's rows place at \
                    each step, as a front's normal is placed square to it")));
            } else {
                over_moved.push(r);
            }
        }
        if places != placed.len() {
            return Err((Code::E103, span, format!("the body's rows must place {} at each step, \
                the points no rate row moves along `{d}`: {} numbers against {places} rows",
                names(&placed), placed.len())));
        }
        for (r, sp) in self.init.iter().zip(&self.init_spans) {
            if let Some(k) = first(&r.cols, n0, &placed) {
                return Err((Code::E103, *sp, format!("`initially` says where the points the rates \
                    move start, and `{}` is placed by the body's rows at each step",
                    self.owner[k])));
            }
        }
        // the rates' count, over the moved points alone
        let index = |c: usize, from: usize| {
            (c >= from).then(|| moved.iter().position(|&k| from + k == c)).flatten()
        };
        let reach = |rows: &[&crate::traced::Row], from: usize| -> Vec<Vec<usize>> {
            rows.iter().flat_map(|r| {
                let cols: Vec<usize> =
                    r.cols.iter().filter_map(|&c| index(c as usize, from)).collect();
                let n_res = r.n_res();
                std::iter::repeat(cols).take(n_res)
            }).collect()
        };
        let rate_rows: Vec<&crate::traced::Row> = self.rates.iter().collect();
        let owner: Vec<String> = moved.iter().map(|&k| self.owner[k].clone()).collect();
        if let Some(m) = super::curves::short(d, moved.len(), &reach(&over_moved, n0),
                                              &reach(&rate_rows, at), &owner, false) {
            return Err((Code::E103, span, m));
        }
        let init: usize = self.init.iter().chain(over_moved.iter().copied())
            .map(|r| r.n_res()).sum();
        if init < moved.len() {
            return Err((Code::E103, span, format!("a marched body starts where `initially {{ … }}` \
                says, at every `{u}`: {} places {} numbers against {init} rows", names(&moved),
                moved.len())));
        }
        if !moved.contains(&t) {
            return Err((Code::E103, span, format!("the traced point is placed by the body's rows \
                at each step, and a marched surface traces a point its rates move along `{d}`")));
        }
        Ok(())
    }

    /// The point a rate is of.
    fn rated_point(&self, rf: &crate::syntax::Ref) -> Result<usize, Fail> {
        let e = self.scope.get(&rf.root.text).copied()
            .ok_or((Code::E101, rf.span, format!("no such entity: `{}`", rf.root.text)))?;
        let e = follow(&self.sk, e, &rf.path).map_err(|m| (Code::E040, rf.span, m))?;
        if e.kind != EntKind::Point {
            return Err((Code::E103, rf.span, format!("`rate(… over {})` is of a point: {} has no \
                rate the body integrates", self.swept, e.kind.a())));
        }
        Ok(e.i())
    }

    /// The scratch line in space from the origin to point `p`'s rate — along the marched formal,
    /// or (`sampled`) along the second (#339 D4) — made once a point and formal.
    fn rate_line(&mut self, p: usize, span: Span, sampled: bool) -> Result<String, Fail> {
        if let Some(n) = self.rated.get(&(p, sampled)) {
            return Ok(n.clone());
        }
        let base = if sampled { super::curves::SAMPLED_BASE } else { super::curves::RATE_BASE };
        let mut tips = Vec::new();
        for c in self.sk.point_all_params(p) {
            match self.slot.get(&c) {
                Some(&s) if (self.n_outer..self.n_outer + self.n_q).contains(&s) => {
                    tips.push(base + s - self.n_outer)
                }
                _ => return Err((Code::E103, span, format!("the point is geometry the component \
                    is written over, and does not move with `{}`: its rate is no rate of the body",
                    self.swept))),
            }
        }
        let zero = match self.origin {
            Some(z) => z,
            None => {
                let z = self.n_outer + self.n_q + self.w.len();
                let t = self.constant(0.0);
                self.w.push(t);
                self.origin = Some(z);
                z
            }
        };
        let name = format!("#{}rate{}", if sampled { "u" } else { "" }, self.rated.len());
        let o = self.sk.point3([0.0; 3], false, &format!("{name}.o"));
        self.sk.lift_point(o);
        let t = self.sk.point3([0.0; 3], false, &format!("{name}.t"));
        self.sk.lift_point(t);
        for c in self.sk.point_all_params(o) {
            self.slot.insert(c, zero);
        }
        for (c, s) in self.sk.point_all_params(t).into_iter().zip(tips) {
            self.slot.insert(c, s);
        }
        let l = EntRef::line(self.sk.line(o, t));
        self.scope.insert(format!("{name}.o"), EntRef::point(o));
        self.scope.insert(format!("{name}.t"), EntRef::point(t));
        self.scope.insert(name.clone(), l);
        self.rated.insert((p, sampled), name.clone());
        Ok(name)
    }



    fn tape(&self, text: &str, span: Span) -> Result<crate::tape::Tape, Fail> {
        let ast = crate::expr::parse_in(text, self.units).map_err(|e| (Code::E103, span, e))?;
        crate::tape::Tape::compile_wide(&ast.body, self.vars).map_err(|e| (Code::E103, span, e))
    }

    fn constant(&self, v: f64) -> crate::tape::Tape {
        let ast = crate::expr::Ast::Num(v, crate::units::Dim::SCALAR);
        crate::tape::Tape::compile_wide(&ast, self.vars).expect("a number always compiles")
    }

    /// A dimension's text: a tape over the formals, the geometry and the given numbers, or —
    /// where it reads a coordinate of a point the body declares — a **placed number** (#378): one
    /// of `q`'s own, seeded by the text over the seeds of what it reads, held to the text by a row
    /// of the body's algebraic rows (`traced::RowKind::Number`, its columns the number and what
    /// the text reads).  One text is one number, however many rows read it.
    fn dimension(&mut self, text: &str, span: Span) -> Result<Dimension, Fail> {
        let ast = crate::expr::parse_in(text, self.units).map_err(|e| (Code::E103, span, e))?;
        let reads = ast.body.deps();
        if !reads.iter().any(|n| self.own.contains_key(n)) {
            return crate::tape::Tape::compile_wide(&ast.body, self.vars)
                .map(Dimension::Tape).map_err(|e| (Code::E103, span, e));
        }
        if self.target == super::curves::Target::Init && self.second.is_some() {
            return Err((Code::E103, span, "`initially { … }` is a marched body's Cauchy data, \
                read over the formals and the geometry: a number that reads the body's own points \
                is placed at each step, the points first".into()));
        }
        if let Some(j) = self.numbers.iter().position(|(t, _)| t == text) {
            return Ok(Dimension::Number(NUMBER_BASE + j));
        }
        let j = self.numbers.len();
        let mut names = Vec::new();
        let mut cols = vec![(NUMBER_BASE + j) as u32];
        for n in &reads {
            let slot = match (self.vars.iter().position(|v| v == n), self.own.get(n)) {
                (Some(s), _) => s,
                (None, Some(&k)) => self.n_outer + k,
                (None, None) => return Err((Code::E103, span,
                                            format!("`{n}` is not something this curve can read"))),
            };
            names.push(n.clone());
            cols.push(slot as u32);
        }
        if cols.len() > crate::traced::NUMBER_COLS {
            return Err((Code::E103, span, format!("a number a traced body places reads at most {} \
                coordinates", crate::traced::NUMBER_COLS - 1)));
        }
        let tape = crate::tape::Tape::compile_wide(&ast.body, &names)
            .map_err(|e| (Code::E103, span, e))?;
        let seeded =
            ast.body.map_vars(&|n| self.own.get(n).and_then(|&k| self.seed_of.get(k)).cloned());
        let seed = crate::tape::Tape::compile_wide(&seeded, self.vars)
            .map_err(|e| (Code::E103, span, e))?;
        self.numbers.push((text.to_string(), seed));
        // the number holds everywhere, whatever row reads it
        self.rows.push(crate::traced::Row { kind: crate::traced::RowKind::Number, cols,
                                            consts: tape.flat });
        self.spans.push(span);
        Ok(Dimension::Number(NUMBER_BASE + j))
    }

    /// The placed numbers put past the declared coordinates (#378): every slot past those moves
    /// on by as many, and a placed number's placeholder lands at its slot.
    fn settle_numbers(&mut self) {
        let (n0, n) = (self.n_outer + self.n_q, self.numbers.len());
        if n == 0 {
            return;
        }
        let rows = self.rows.iter_mut().chain(&mut self.rates).chain(&mut self.init);
        for c in rows.flat_map(|r| r.cols.iter_mut()) {
            let s = *c as usize;
            if (NUMBER_BASE..super::curves::RATE_BASE).contains(&s) {
                *c = (n0 + s - NUMBER_BASE) as u32;
            } else if (n0..NUMBER_BASE).contains(&s) {
                *c = (s + n) as u32;
            }
        }
        self.n_q += n;
        for (text, seed) in self.numbers.iter().cloned() {
            self.owner.push(text);
            self.seeds.push(seed);
        }
    }

    /// A declaration drawn in space: a point, its three numbers inner unknowns seeded by its
    /// hint; a line between two.
    fn declare(&mut self, st: &Stmt, d: &Decl) -> Result<(), Fail> {
        if d.computed.is_some() || d.seed_at.is_some() {
            return Err((Code::E103, st.span, "a point traced in space is placed by relations and \
                seeded by numbers".into()));
        }
        let e = match d.kind {
            EntKind::Point => {
                let p = self.sk.point3([0.0; 3], false, &d.name.key().text);
                self.sk.lift_point(p);
                for (k, q) in self.sk.point_all_params(p).into_iter().enumerate() {
                    let at = *d.seed_spans.get(k).unwrap_or(&st.span);
                    let ast = match d.seed_text.get(k).and_then(|t| t.as_ref()) {
                        Some(t) => crate::expr::parse_in(t, self.units)
                            .map_err(|e| (Code::E103, at, e))?.body,
                        None => crate::expr::Ast::Num(d.seed.get(k).copied().unwrap_or(0.0),
                                                      crate::units::Dim::SCALAR),
                    };
                    let seed = crate::tape::Tape::compile_wide(&ast, self.vars)
                        .map_err(|e| (Code::E103, at, e))?;
                    self.seeds.push(seed);
                    self.seed_of.push(ast);
                    if let Some(c) = ["x", "y", "z"].get(k) {
                        self.own.insert(format!("{}.{c}", d.name.key().text), self.n_q);
                    }
                    self.slot.insert(q, self.n_outer + self.n_q);
                    self.owner.push(d.name.key().text.clone());
                    self.n_q += 1;
                }
                EntRef::point(p)
            }
            EntKind::Line => {
                let end = |i: usize| -> Result<usize, Fail> {
                    let r = d.children.get(i).and_then(|v| v.first()).and_then(|k| k.as_ref())
                        .ok_or((Code::E103, st.span,
                                "a line traced in space names its ends".into()))?;
                    let e = self.scope.get(&r.root.text).copied()
                        .ok_or((Code::E101, r.span, format!("no such entity: `{}`", r.root.text)))?;
                    let e = follow(&self.sk, e, &r.path).map_err(|m| (Code::E040, r.span, m))?;
                    if e.kind != EntKind::Point {
                        return Err((Code::E040, r.span, "a line runs between points".into()));
                    }
                    Ok(e.i())
                };
                let (a, b) = (end(0)?, end(1)?);
                EntRef::line(self.sk.line(a, b))
            }
            other => {
                return Err((Code::E103, st.span, format!("a body traced in space draws points and \
                    lines, not {}", other.a())))
            }
        };
        self.scope.insert(d.name.key().text.clone(), e);
        Ok(())
    }

    /// A place over the formals: a pinned number or expression, read as a tape into `w`.
    fn pinned(&mut self, given: Option<&Arg>, name: &str, span: Span) -> Result<usize, Fail> {
        let place = match given {
            Some(Arg::Seed { value, pinned: true }) => self.constant(*value),
            Some(Arg::SeedExpr { text, pinned: true, span }) => self.tape(text, *span)?,
            // a carried point's roll stated, `at: τ` (#366): its text, in degrees as `t` is
            Some(Arg::Dim { text, span }) => self.tape(text, *span)?,
            Some(Arg::Tie { name, span, .. }) if self.vars.contains(name) => {
                self.tape(name, *span)?
            }
            _ => return Err((Code::E103, span, format!("`{name}` must be pinned in a traced body, \
                `{name} == …`: a body infers nothing"))),
        };
        self.w.push(place);
        Ok(self.n_outer + self.n_q + self.w.len() - 1)
    }

    fn relation(&mut self, st: &Stmt, r: &crate::ir::Relation) -> Result<(), Fail> {
        let seen = self.seen_from(r)?;
        let r = seen.as_ref().unwrap_or(r);
        let (sk, scope, motions, drawn) = (&self.sk, &self.scope, &self.motions, &self.drawn);
        let r = r.resolve(&|rf| {
            let bare = rf.path.is_empty();
            if bare && motions.contains_key(&rf.root.text) {
                return Some(EntKind::Motion);
            }
            match drawn.get(&rf.root.text) {
                Some((d, _)) if bare => Some(d.kind),
                _ => scope.get(&rf.root.text).copied()
                    .and_then(|e| follow(sk, e, &rf.path).ok()).map(|e| e.kind),
            }
        }).map_err(|(s, m)| (Code::E040, s, m))?;
        if r.kind == CKind::CarriedPoint {
            return self.carried(st, &r);
        }
        let on = r.args.iter().flatten().find_map(|a| match a {
            Arg::Ref(rf) if rf.path.is_empty() && self.drawn.contains_key(&rf.root.text) => {
                Some(rf.root.text.clone())
            }
            _ => None,
        });
        if let Some(name) = on {
            return self.on_drawn(st, &r, &name);
        }
        if self.target == super::curves::Target::Rate {
            super::curves::admissible(&r).map_err(|m| (Code::E103, st.span, m))?;
        }
        let spec = r.kind.spec();
        if r.kind.gauge() || r.kind.soft_by_default() || r.kind.built()
            || matches!(r.kind, CKind::Ccw | CKind::Cw) {
            return Err((Code::E103, st.span, format!("`{}` cannot appear in a body traced in space",
                crate::syntax::snake(r.kind.name()))));
        }
        let mut cargs: Vec<CArg> = Vec::with_capacity(spec.len());
        let mut dim: Option<(SpecKind, Dimension)> = None;
        let mut places: Vec<(usize, usize)> = Vec::new();
        for (i, (name, kind)) in spec.iter().enumerate() {
            let given = r.args.get(i).and_then(|a| a.as_ref());
            if r.kind.form_slot() == Some(i) {
                cargs.push(CArg::Int(OrdinateForm::Space as i64));
                continue;
            }
            match (kind, given) {
                (k, Some(Arg::Ref(rf))) if k.is_entity() => {
                    let found = self.scope.get(&rf.root.text).copied();
                    cargs.push(ent_arg(&self.sk, found, *k, rf).map_err(|(c, m)| (c, rf.span, m))?);
                }
                (k, Some(Arg::Dim { text, span })) if k.is_dimension() => {
                    dim = Some((*k, self.dimension(text, *span)?));
                    cargs.push(CArg::Num(0.0));
                }
                (k, given) if k.is_param() => {
                    let s = self.pinned(given, name, st.span)?;
                    let p = self.sk.param(0.0, false, name) as u32;
                    places.push((p as usize, s));
                    cargs.push(CArg::Param(p));
                }
                (k, None) if k.is_dimension() || k.is_entity() || r.kind.infers_arg(i) => {
                    return Err((Code::E103, st.span, format!("`{name}` must be stated: a traced \
                        body infers nothing")));
                }
                (_, None) => cargs.push(r.kind.default_arg(i)),
                (k, Some(a)) => match scalar_arg(*k, a) {
                    Some(v) => cargs.push(v),
                    None => return Err((Code::E103, st.span, format!("`{name}`: {} is wanted here",
                        k.a()))),
                },
            }
        }
        for (p, s) in &places {
            self.slot.insert(*p as u32, *s);
        }
        // a word between points in space is the relation in space
        let (kind, cargs) = match in_space(&self.sk, r.kind, &cargs, Reader::Drawn)
            .map_err(|(_, m)| (Code::E062, st.span, m))? {
            Some((k, args, left_out)) => {
                if left_out.iter().enumerate().any(|(i, &l)| l && k.infers_arg(i)) {
                    return Err((Code::E103, st.span, format!("`{}` must state what it would read \
                        off the geometry: a traced body infers nothing",
                        crate::syntax::snake(k.name()))));
                }
                (k, args)
            }
            None if r.kind.word_slot().is_some() => (r.kind, cargs),
            None => return Err((Code::E062, st.span, format!("`{}` relates its operands in one \
                plane, and a body traced in space has none", crate::syntax::snake(r.kind.name())))),
        };
        let c = Constraint::new(kind, cargs);
        let mut cols: Vec<u32> = Vec::new();
        for p in c.params(&self.sk) {
            let s = self.slot.get(&p).copied()
                .ok_or((Code::E103, st.span, "traced lowering lost a column".to_string()))?;
            cols.push(s as u32);
        }
        match dim {
            Some((k, t)) => {
                let twin = match kind {
                    CKind::Ordinate => Some(crate::kernels::K::OrdinateSpaceFree),
                    k => k.free_kernel(),
                }.ok_or((Code::E103, st.span, format!("{} cannot be stated over the running \
                    formals here", crate::model::article(&crate::syntax::snake(kind.name())))))?;
                match t {
                    Dimension::Tape(t) => {
                        cols.push((self.n_outer + self.n_q + self.w.len()) as u32);
                        self.w.push(t);
                    }
                    Dimension::Number(slot) => cols.push(slot as u32),
                }
                let sign = c.side().unwrap_or(1.0);
                self.push(crate::traced::Row {
                    kind: crate::traced::RowKind::Kernel(twin as usize),
                    cols,
                    consts: vec![sign * crate::expr::to_arg_units(k, 1.0), 0.0],
                }, st.span);
            }
            None => {
                let consts = c.consts_on(&self.sk, None);
                let kind = crate::traced::RowKind::Kernel(c.kernel_id());
                self.push(crate::traced::Row { kind, cols, consts }, st.span);
            }
        }
        Ok(())
    }

    /// The variable-table slots of the point a relation's argument `i` names, or `what` refused.
    fn point_cols(&self, st: &Stmt, r: &crate::ir::ResolvedRelation<'_>, i: usize, what: &str)
        -> Result<Vec<u32>, Fail> {
        let Some(Arg::Ref(rf)) = r.args.get(i).and_then(|a| a.as_ref()) else {
            return Err((Code::E103, st.span, what.into()));
        };
        let e = self.scope.get(&rf.root.text).copied()
            .ok_or((Code::E101, rf.span, format!("no such entity: `{}`", rf.root.text)))?;
        let e = follow(&self.sk, e, &rf.path).map_err(|m| (Code::E040, rf.span, m))?;
        if e.kind != EntKind::Point {
            return Err((Code::E040, rf.span, what.into()));
        }
        self.sk.point_all_params(e.i()).into_iter()
            .map(|q| self.slot.get(&q).map(|&s| s as u32)
                .ok_or((Code::E103, st.span, "traced lowering lost a column".to_string())))
            .collect()
    }

    /// **A point on a curve the drawing has at a number of the formals** (#376): `p
    /// coincident(t == u) k` in a marched body's `initially`, its Cauchy data read off `k` — a
    /// spline, a curve drawn in a held plane, or a curve traced in space.  One row of three: the
    /// point less the curve there, its place pinned over the formals as any place is.
    fn on_drawn(&mut self, st: &Stmt, r: &crate::ir::ResolvedRelation<'_>, name: &str)
        -> Result<(), Fail> {
        use crate::traced::RowKind;
        let (d, at) = self.drawn.get(name).cloned()
            .ok_or((Code::E103, st.span, "traced lowering lost a curve".to_string()))?;
        if !matches!(r.kind, CKind::PointOnSpline | CKind::PointOnCurve | CKind::PointOnExtrusion
            | CKind::PointOnTraced) {
            return Err((Code::E103, st.span, format!("a marched body reads `{name}` as its Cauchy \
                data alone: a point on it at a number of the formals, `p coincident(t == u) \
                {name}`")));
        }
        if self.target != super::curves::Target::Init || self.second.is_none() {
            return Err((Code::E103, st.span, format!("a point on `{name}` is Cauchy data: it is \
                said in `initially {{ … }}` of a body marched over two formals, where it places \
                the points the rates move at the home")));
        }
        let x = self.point_cols(st, r, 0, "a point on a curve names the point")?;
        let slot = if r.kind == CKind::PointOnTraced { "u" } else { "t" };
        let t = self.pinned(r.args.get(2).and_then(|a| a.as_ref()), slot, st.span)? as u32;
        let theta = (at..at + d.width).map(|c| c as u32);
        let cols: Vec<u32> = match d.row {
            RowKind::Traced => {
                // a curve's second place is held at none
                let s = (self.n_outer + self.n_q + self.w.len()) as u32;
                let zero = self.constant(0.0);
                self.w.push(zero);
                x.into_iter().chain([t, s]).chain(theta).collect()
            }
            _ => [t].into_iter().chain(theta).chain(x).collect(),
        };
        self.push(crate::traced::Row { kind: d.row, cols, consts: d.consts }, st.span);
        Ok(())
    }

    /// A carried point's `under: m, seen_from: g` over two frames the body is written over (#383):
    /// `m`'s steps then `g`'s, `g`'s columns after `m`'s, and the relative of the two — kept under
    /// a key no source spells, the relation returned reading it.  `None` for any other relation.
    fn seen_from(&mut self, r: &crate::ir::Relation) -> Result<Option<crate::ir::Relation>, Fail> {
        use crate::envelope::held::Step;
        use crate::syntax::OpArg;
        let Some(w) = r.form.written() else { return Ok(None) };
        let named = |k: &str| w.args.iter().position(|a| matches!(a,
            OpArg::Named(n, Arg::Ref(_)) if n.text == k));
        let (Some(u), Some(g)) = (named("under"), named("seen_from")) else { return Ok(None) };
        let (OpArg::Named(_, Arg::Ref(ur)), OpArg::Named(_, Arg::Ref(gr))) =
            (&w.args[u], &w.args[g]) else { return Ok(None) };
        let get = |r: &crate::syntax::Ref| self.motions.get(&r.root.text).cloned()
            .ok_or((Code::E103, r.span,
                format!("`{}` is no frame the traced component is written over", r.root.text)));
        let ((su, cu), (sg, cg)) = (get(ur)?, get(gr)?);
        if su.is_empty() {
            return Err((Code::E103, ur.span, format!("`{}` does not move: a carried point is \
                carried by a frame's joint", ur.root.text)));
        }
        let key = format!("#seen{}", self.motions.len());
        let (steps, cols) = if sg.is_empty() { (su, cu) } else {
            let (n, wu) = (su.len(), cu.len());
            let shift = |f: Option<crate::envelope::held::Free>|
                f.map(|f| crate::envelope::held::Free { at: f.at + wu, ..f });
            let mut steps = su;
            steps.extend(sg.into_iter().map(|s| match s {
                Step::Rotation { at, ratio, advance, free, dir } =>
                    Step::Rotation { at: at + wu, ratio, advance, free: shift(free), dir },
                Step::Translation { at, advance, free, dir } =>
                    Step::Translation { at: at + wu, advance, free: shift(free), dir },
                Step::Turn { at, ratio, free } =>
                    Step::Turn { at: at + wu, ratio, free: shift(free) },
                Step::Relative { source, observer } =>
                    Step::Relative { source: source + n, observer: observer + n },
            }));
            steps.push(Step::Relative { source: n - 1, observer: steps.len() - 1 });
            (steps, cu.into_iter().chain(cg).collect())
        };
        self.motions.insert(key.clone(), (steps, cols));
        let mut w = w.clone();
        if let OpArg::Named(_, Arg::Ref(ur)) = &mut w.args[u] {
            ur.root.text = key;
            ur.path.clear();
        }
        w.args.remove(g);
        let mut out = r.clone();
        out.form = crate::syntax::RelationForm::Written(w);
        Ok(Some(out))
    }

    /// `X coincident(under: m, t == τ) x`: the carried point's lift, the point's, the roll's tape
    /// and the motion's axes, its steps the constants.  In a body that states a rate (#377) it is
    /// one of the body's rows or, in `initially { … }`, of its home's.
    fn carried(&mut self, st: &Stmt, r: &crate::ir::ResolvedRelation<'_>) -> Result<(), Fail> {
        let what = "a carried point names two points";
        let (x, at) = (self.point_cols(st, r, 0, what)?, self.point_cols(st, r, 1, what)?);
        let Some(Arg::Ref(m)) = r.args.get(2).and_then(|a| a.as_ref()) else {
            return Err((Code::E103, st.span,
                        "a carried point names the motion `under:` it".into()));
        };
        let (steps, from) = self.motions.get(&m.root.text).cloned().ok_or((Code::E103, m.span,
            format!("`{}` is no frame the traced component is written over", m.root.text)))?;
        if steps.is_empty() {
            return Err((Code::E103, m.span, format!("`{}` does not move: a carried point is \
                carried by a frame's joint", m.root.text)));
        }
        let t = self.pinned(r.args.get(3).and_then(|a| a.as_ref()), "t", st.span)?;
        let cols: Vec<u32> = x.into_iter().chain(at).chain([t as u32])
            .chain(from.iter().map(|&c| c as u32)).collect();
        self.push(crate::traced::Row { kind: crate::traced::RowKind::Carried, cols,
            consts: crate::envelope::held::encode_steps(&steps) }, st.span);
        Ok(())
    }
}

