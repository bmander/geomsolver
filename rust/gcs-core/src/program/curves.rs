//! Compile computed curves and traces into model definitions.

use super::relations::{ent_arg, scalar_arg};
use super::resolve::{follow, Resolver};
use super::{Code, Diag, Severity};
use crate::constraints::{Arg as CArg, CKind, Constraint, OrdinateForm, SpecKind};
use crate::ir::{Decl, Operation as StmtKind, Statement as Stmt};
use crate::kernels::KERNELS;
use crate::model::{EntKind, EntRef, Sketch};
use crate::syntax::{Arg, Program, Ref, Span};
use std::collections::BTreeMap;

/// Compile a component point over a numeric formal. Variable order is the swept
/// formal, entity-formal coordinates in model order, then other numeric formals;
/// it must match kernel Jacobian columns.
/// `id` is the component's identity across the program (`Program::component_id`), which keys
/// the definition, and `written` how the call spelled it, which a lifted program writes.
#[allow(clippy::too_many_arguments)]
fn compile_curve(
    prog: &Program,
    comp: &crate::syntax::Component,
    id: &str,
    written: &str,
    swept: &str,
    point: &str,
    columns: &[String],
    home: bool,
    units: crate::units::Units,
) -> Result<crate::model::CurveDef, (Span, String)> {
    use crate::syntax::Ty;
    let cname = comp.name.as_ref().map(|n| n.text.clone()).unwrap_or_default();
    let mut vars = vec![swept.to_string()];
    let mut formals = Vec::new();
    let mut values = Vec::new();
    for fo in &comp.formals {
        match fo.ty {
            Ty::Group | Ty::Set => return Err((fo.span,
                "a curve needs fixed scalar or entity formals; pass the group's members individually".into())),
            Ty::Path => return Err((fo.span,
                "a curve cannot be written over a path: it has no fixed number of coordinates"
                    .into())),
            Ty::Ent(k) => {
                let names = k.scalar_names(&fo.name.text).ok_or_else(|| {
                    (
                        fo.span,
                        format!(
                            "a curve cannot be written over {}: it has no fixed number of \
                             coordinates",
                            k.a()
                        ),
                    )
                })?;
                vars.extend(names);
                formals.push((fo.name.text.clone(), k));
            }
            _ if fo.name.text != swept && !columns.contains(&fo.name.text) => {
                values.push(fo.name.text.clone())
            }
            _ => {}
        }
    }
    // the unknowns among the numbers are columns, beside the entities' scalars; the constants
    // come after, where nothing differentiates in them
    vars.extend(columns.iter().cloned());
    vars.extend(values.iter().cloned());
    let ex = crate::flatten::expand_component(prog, comp, units);
    if let Some(d) = ex.diagnostics.iter().find(|d| d.severity() == Severity::Error) {
        return Err((d.span, d.message.clone()));
    }
    if let Some(member) = ex.private_names.keys().find(|member| {
        point == member.as_str() || point.strip_prefix(member.as_str()).is_some_and(|tail| tail.starts_with('.'))
    }) {
        return Err((comp.span, format!("`{point}` names private member `{member}`")));
    }
    let traced = ex.aliases.get(point).cloned().unwrap_or_else(|| point.to_string());
    if formals.iter().any(|(n, _)| *n == traced) {
        return Err((
            comp.span,
            format!(
                "`{point}` is geometry `{cname}` is written over, and does not move with `{swept}`"
            ),
        ));
    }
    let body: Vec<&Stmt> = ex.flat.iter().collect();
    // a computed point is the whole of what a component may say about it: nothing on the
    // sheet or in a block can hold a point to a formula beside placed geometry, so the body
    // is the one computed point — decided by shape, not by which statement is found first
    let computed = body.iter().find_map(|st| match &st.kind {
        StmtKind::Decl(d) if d.name.key().text == traced => d.computed.as_ref(),
        _ => None,
    });
    let (body, pose_of) = match computed {
        Some([(x, xspan), (y, yspan)]) => {
            if body.len() != 1 {
                return Err((
                    comp.span,
                    format!(
                        "`{point}` is a computed point, so `{cname}` may hold nothing else: a \
                         formula and placed geometry cannot both say where a point is"
                    ),
                ));
            }
            let tape = |text: &str, span: Span| -> Result<crate::tape::Tape, (Span, String)> {
                let ast = crate::expr::parse_in(text, units).map_err(|e| (span, e))?;
                crate::tape::Tape::compile(&ast.body, &vars).map_err(|e| (span, e))
            };
            let body = crate::model::CurveBody::Exprs { x: tape(x, *xspan)?, y: tape(y, *yspan)? };
            (body, Vec::new())
        }
        None => {
            // **a drawn instance of a body that states a rate is its home** (#339 D2): its pose
            // on the sheet is the initial value, so its coordinates are columns of the curve,
            // after the numbers left unknown and before the constants
            let rated = home && body.iter().any(|st| states_rate(st));
            if rated {
                let n_q = Lowering::count(comp.span, &body, &vars, &formals, units)?;
                let at = vars.len() - values.len();
                let names = (0..n_q).map(|k| format!("#home.{k}"));
                vars.splice(at..at, names);
            }
            let angle = comp.formals.iter().any(|f| f.name.text == swept && f.ty == Ty::Angle);
            let trace = Trace { swept, angle, n_columns: columns.len(), n_values: values.len(),
                                pose: rated };
            let (locus, pose_of) =
                compile_trace(comp.span, &traced, &body, &vars, &formals, &trace, units)?;
            (crate::model::CurveBody::Trace(locus), pose_of)
        }
    };
    let homed = matches!(&body, crate::model::CurveBody::Trace(l) if home && l.states_rate());
    let key = crate::model::CurveDef::key(id, point, swept, columns);
    Ok(crate::model::CurveDef {
        name: if homed { format!("{key}{}", crate::model::CurveDef::HOME) } else { key },
        home: homed,
        component: written.to_string(),
        port: point.to_string(),
        formals,
        columns: columns.to_vec(),
        values,
        param: swept.to_string(),
        turns: comp.formals.iter().any(|f| f.name.text == swept && f.ty == Ty::Angle),
        vars,
        body,
        pose_of,
    })
}

/// What a trace is lowered as, besides its body: the formal it runs over, how many numeric
/// columns and constants its variable table has, and whether its drawn instance's pose is the
/// initial value of the rates it states (#339 D2) — the `n_q` columns before the constants.
struct Trace<'s> {
    swept: &'s str,
    /// Whether the formal is an `Angle`, along which no rate is read yet.
    angle: bool,
    n_columns: usize,
    n_values: usize,
    pose: bool,
}

/// Whether a statement of an expanded body reads a rate (#339 D2).
fn states_rate(st: &Stmt) -> bool {
    matches!(&st.kind, StmtKind::Relation(r) if r.reads_rate())
}

/// Lower a trace into scratch geometry and tapes, using `Constraint::params_on`
/// for kernel columns. Swept dimensions use free-twin kernels whose final column
/// reads a tape-derived value, including its gradient.  A relation that reads a rate is a row
/// of the rates' system, one in `initially` a row of the home's (#339 D2).
fn compile_trace(
    span: Span,
    point: &str,
    body: &[&Stmt],
    vars: &[String],
    formals: &[(String, EntKind)],
    trace: &Trace,
    units: crate::units::Units,
) -> Result<(crate::locus::Locus, Vec<(String, usize)>), (Span, String)> {
    let mut low = Lowering::new(span, vars, formals, units)?;
    low.swept = trace.swept.to_string();
    low.angle = trace.angle;
    // the numeric columns follow the entities' scalars; nothing in the block is built over them,
    // and they are read only through the tapes of the dimensions that name them.  A drawn
    // instance's pose, where it is the initial value, follows them
    let n_pose = match trace.pose {
        true => vars.len() - 1 - low.slot.len() - trace.n_columns - trace.n_values,
        false => 0,
    };
    let n_theta = low.slot.len() + trace.n_columns + n_pose;
    debug_assert_eq!(low.n_outer, 1 + n_theta + trace.n_values, "variable table shape");
    // -- pass 1: declarations, so a statement may read a point declared after it --------
    for st in body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        low.declare(st, d)?;
    }
    if trace.pose && n_pose != low.n_q {
        return Err((span, "trace lowering lost its home".to_string()));
    }
    // -- pass 2: constraints and orientations, lowered to rows and predicates ----------
    for st in body {
        let r = match &st.kind {
            StmtKind::Decl(_) => continue,
            StmtKind::Relation(r) => r,
            _ => {
                return Err((
                    st.span,
                    "a traced component holds declarations and constraints only".to_string(),
                ))
            }
        };
        if r.reads_rate() {
            low.target = Target::Rate;
            let r = low.rate_relation(st, r)?;
            low.relation(st, &r)?;
        } else {
            low.target = if r.initially { Target::Init } else { Target::Alg };
            low.relation(st, r)?;
        }
    }
    let at = low.n_outer + low.n_q + low.w.len();
    place_rates(low.rates.iter_mut().map(|r| &mut r.cols), at, low.n_q);
    if !low.rates.is_empty() {
        low.short(span)?;
    }
    let Lowering {
        sk, scope, slot, n_outer, n_q, seeds, pose_of, w, rows, preds, rates, init, ..
    } = low;
    let traced = match scope.get(point) {
        Some(e) if e.kind == EntKind::Point => {
            let p = sk.point_params(e.i())[0];
            match slot.get(&p) {
                Some(&s) if s >= n_outer => s - n_outer,
                _ => {
                    return Err((span, format!("`{point}` must be a point the component declares")))
                }
            }
        }
        _ => return Err((span, format!("`{point}` must be a point the component declares"))),
    };
    let rates = crate::locus::Rates { rows: rates, init, pose: trace.pose };
    let locus =
        crate::locus::Locus::new(n_outer, n_theta, n_q, traced, w, seeds, rows, preds, rates)
            .map_err(|m| (span, m))?;
    Ok((locus, pose_of))
}

/// Which rows a relation of a trace block is lowered to (#339 D2).
#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum Target {
    /// It holds at every place.
    Alg,
    /// It reads a rate: a row of the rate system.
    Rate,
    /// It holds at the home alone (`initially { … }`).
    Init,
}

/// Where a rate's slots stand while the block is lowered: past every slot, since the derived
/// values are not all counted yet (`Lowering::place_rates` puts them past `w`).
pub(super) const RATE_BASE: usize = 1 << 20;
/// Where a rate along a second, sampled formal stands while a body is lowered (#339 D4): past
/// every rate along the first.
pub(super) const SAMPLED_BASE: usize = 1 << 21;

/// A relation that reads a rate, checked (#339 D2): one rate, along the formal the trace runs
/// over, and not in `initially`.  The relation, the rated operand and whether the rate stands
/// alone before its word (`rate(p over s) distance(1, along: tl)`).
pub(super) fn rate_operand(
    st: &Stmt, r: &crate::ir::Relation, swept: &str, angle: bool,
) -> Result<(crate::ir::Relation, crate::syntax::Ref, bool), (Span, String)> {
    let w = r.form.written().ok_or((st.span, "a rate is written in a relation".to_string()))?;
    if r.initially {
        return Err((st.span, "a rate is no row at the home: `initially { … }` says what holds \
            there of the instance itself".to_string()));
    }
    if w.rates.len() > 1 {
        return Err((st.span, "a relation reads one rate: two along the running formal would \
            make the rates' system quadratic, leaving a root to choose at every step".to_string()));
    }
    let rate = &w.rates[0];
    if rate.over.text != swept {
        return Err((rate.over.span, format!("`rate(… over {})` is read along `{}`, and the curve \
            runs over `{swept}`: a rate is along the formal a trace runs over", rate.over.text,
            rate.over.text)));
    }
    // the formal's number is degrees, and a rate per degree is no rate a person states: which
    // unit one along a turn is in is D3's to settle
    if angle {
        return Err((rate.over.span, format!("`{swept}` is an angle, and a rate along an angle is \
            not yet read: run the body over a `Scalar` or a `Length`")));
    }
    let unary = w.fixity == crate::constraints::Fixity::Infix && w.ops.len() == 1;
    Ok((r.clone(), w.ops[rate.op].clone(), unary))
}

/// A relation that reads a rate, rewritten over the scratch line `line` from an origin to the
/// rate (`{line}.o`, `{line}.t`): the line where a direction is read, its two points where the
/// rate stands alone before `distance(k, along: l)`.  Each of `lines` is the operand a rate is
/// of, its scratch line and its span (over two formals a relation may read two, #339 D4).
pub(super) fn rate_rewritten(
    mut r: crate::ir::Relation, lines: &[(usize, String, Span)], unary: bool,
) -> crate::ir::Relation {
    let at = |n: String, span: Span| crate::syntax::Ref { span, ..crate::syntax::Ref::new(n) };
    if let Some(w) = r.form.written_mut() {
        for (k, line, span) in lines {
            if unary {
                w.ops = vec![at(format!("{line}.o"), *span), at(format!("{line}.t"), *span)];
            } else {
                w.ops[*k] = at(line.clone(), *span);
            }
        }
        w.rates.clear();
    }
    r
}

/// **A rate stands where a direction is read** (#339 D0, decision 1): `parallel`,
/// `perpendicular`, a directed `angle`, its component `distance(k, along: l)` — never a place,
/// and never a magnitude, whose row is quadratic in the rate.
pub(super) fn admissible(r: &crate::ir::ResolvedRelation<'_>) -> Result<(), String> {
    if matches!(r.kind, CKind::Parallel | CKind::Perpendicular | CKind::Angle | CKind::Ordinate) {
        return Ok(());
    }
    let word =
        r.written.map_or_else(|| crate::syntax::snake(r.kind.name()), |w| w.word.text.clone());
    Err(format!("a rate stands where a direction is read — `parallel`, `perpendicular`, a \
        directed `angle`, or its component `distance(k, along: l)` — and `{word}` reads it \
        otherwise: a rate is never a place, and never a magnitude, whose row would be quadratic \
        in it"))
}

/// Each residual row of `rows` (its count and its columns) and which of the body's `n_q` numbers
/// it reads, its columns read from slot `from` — what `short` matches: an algebraic row from the
/// body's numbers (its derivative reads their rates), a rate row from the rates.
pub(super) fn reach<'r>(rows: impl Iterator<Item = (usize, &'r [u32])>, from: usize, n_q: usize)
    -> Vec<Vec<usize>> {
    rows.flat_map(|(n_res, cols)| {
        let cols: Vec<usize> = cols.iter().map(|&c| c as usize)
            .filter(|c| (from..from + n_q).contains(c)).map(|c| c - from).collect();
        std::iter::repeat(cols).take(n_res)
    }).collect()
}

/// The rates' slots placed at `at`, past every derived value, now that those are counted: each
/// column at `RATE_BASE` or past it is a rate's, and at `SAMPLED_BASE` or past it a rate along
/// a second formal's, past the `n_q` along the first (#339 D4).
pub(super) fn place_rates<'r>(cols: impl Iterator<Item = &'r mut Vec<u32>>, at: usize, n_q: usize) {
    for c in cols.flatten() {
        if *c as usize >= SAMPLED_BASE {
            *c = (at + n_q + *c as usize - SAMPLED_BASE) as u32;
        } else if *c as usize >= RATE_BASE {
            *c = (at + *c as usize - RATE_BASE) as u32;
        }
    }
}

/// **A body short of rate rows**, reported with the freedom it leaves (#339 D0, finding 1):
/// `None` where its rows and rate rows are enough — square by count in the plane; in space,
/// where a `parallel` is two rows one of which the body's own rows imply, at least as many, the
/// rest a rank read at every place.  `alg` and `rates` are each residual row's reach into the
/// body's `n_q` numbers (an algebraic row's derivative reads the rates of what it reads), and
/// `owner` names each number's point: the freedom is the under-determined block of that
/// structure, whichever matching.
pub(super) fn short(
    swept: &str, n_q: usize, alg: &[Vec<usize>], rates: &[Vec<usize>], owner: &[String],
    plane: bool,
) -> Option<String> {
    let (n_alg, n_rate) = (alg.len(), rates.len());
    if n_alg + n_rate == n_q || (!plane && n_alg + n_rate > n_q) {
        return None;
    }
    let rows = |n: usize| if n == 1 { "row" } else { "rows" };
    let head = format!("a body that states a rate must determine its rates along `{swept}`: \
        {n_q} unknowns against {n_alg} {} and {n_rate} rate {}", rows(n_alg), rows(n_rate));
    if n_alg + n_rate > n_q {
        return Some(format!("{head} — it states {} more than it places in the plane",
            n_alg + n_rate - n_q));
    }
    let reach: Vec<Vec<usize>> = alg.iter().chain(rates).cloned().collect();
    let dm = crate::graph::dulmage_mendelsohn(&reach, n_q);
    let mut names: Vec<&str> = Vec::new();
    for &c in &dm.under_cols {
        if let Some(n) = owner.get(c) {
            if !names.contains(&n.as_str()) {
                names.push(n);
            }
        }
    }
    let who = names.iter().map(|n| format!("`{n}`")).collect::<Vec<_>>().join(", ");
    let k = n_q - n_alg - n_rate;
    Some(format!("{head} — its rates leave {k} {} among those of {}: state {k} more rate {} of \
        how they move", if k == 1 { "freedom" } else { "freedoms" },
        if who.is_empty() { "its points".to_string() } else { who }, rows(k)))
}

/// **A trace block being lowered** (`compile_trace`): the scratch sketch it is drawn in, its
/// names, which variable-table slot each scratch parameter is, and what the block comes to so far
/// — each inner unknown's seed and owner, the tapes of the numbers read over `u` (`w`), the rows
/// and the orientations.
struct Lowering<'v> {
    sk: Sketch,
    scope: BTreeMap<String, EntRef>,
    /// scratch parameter index -> variable-table slot
    slot: BTreeMap<u32, usize>,
    vars: &'v [String],
    units: crate::units::Units,
    n_outer: usize,
    n_q: usize,
    seeds: Vec<crate::tape::Tape>,
    /// each inner unknown's owner, so a drawn instance's pose can be read off the sheet
    pose_of: Vec<(String, usize)>,
    w: Vec<crate::tape::Tape>,
    rows: Vec<crate::locus::Row>,
    preds: Vec<crate::locus::Pred>,
    /// A body that states a rate (#339 D2): the formal the trace runs over, which rows a
    /// relation is lowered to, the rate rows and the home's, and for each point whose rate is
    /// read the scratch line from the origin to its rate (`rated`), the origin's slots a zero.
    swept: String,
    angle: bool,
    target: Target,
    rates: Vec<crate::locus::Row>,
    init: Vec<crate::locus::Row>,
    rated: BTreeMap<usize, String>,
    origin: Option<usize>,
}

impl<'v> Lowering<'v> {
    /// The scratch sketch with the entity formals drawn in it, each scalar a slot after the
    /// parameter's (slot 0).
    fn new(
        span: Span,
        vars: &'v [String],
        formals: &[(String, EntKind)],
        units: crate::units::Units,
    ) -> Result<Lowering<'v>, (Span, String)> {
        let mut sk = Sketch::new();
        // the scratch sketch the block is lowered through reads numbers, so it is in the
        // document's units too — `at_seed` asks it what a bearing's literal is written in
        sk.units = units;
        let mut scope: BTreeMap<String, EntRef> = BTreeMap::new();
        let mut slot: BTreeMap<u32, usize> = BTreeMap::new();
        let mut next = 1usize; // slot 0 is the parameter
        for (name, kind) in formals {
            let e = match kind {
                EntKind::Point => EntRef::point(sk.point(0.0, 0.0, false, name)),
                EntKind::Line => {
                    let a = sk.point(0.0, 0.0, false, name);
                    let b = sk.point(1.0, 0.0, false, name);
                    EntRef::line(sk.line(a, b))
                }
                EntKind::Circle => {
                    let c = sk.point(0.0, 0.0, false, name);
                    EntRef::circle(sk.circle(c, 1.0, name))
                }
                // a plane: its attitude is no column of the curve, so the front plane's will do
                EntKind::Frame => EntRef::frame(sk.fixed_plane(crate::plane::Basis::page(), name)),
                other => {
                    return Err((
                        span,
                        format!("a trace cannot yet be written over {}", other.a()),
                    ))
                }
            };
            for p in sk.entity_params(e) {
                slot.insert(p, next);
                next += 1;
            }
            scope.insert(name.clone(), e);
        }
        Ok(Lowering {
            sk,
            scope,
            slot,
            vars,
            units,
            n_outer: vars.len(),
            n_q: 0,
            seeds: Vec::new(),
            pose_of: Vec::new(),
            w: Vec::new(),
            rows: Vec::new(),
            preds: Vec::new(),
            swept: String::new(),
            angle: false,
            target: Target::Alg,
            rates: Vec::new(),
            init: Vec::new(),
            rated: BTreeMap::new(),
            origin: None,
        })
    }

    /// How many numbers the block places — its declarations' own scalars — read by lowering
    /// them alone: the width of a drawn instance's pose, before the columns are laid out.
    fn count(
        span: Span,
        body: &[&Stmt],
        vars: &[String],
        formals: &[(String, EntKind)],
        units: crate::units::Units,
    ) -> Result<usize, (Span, String)> {
        let mut low = Lowering::new(span, vars, formals, units)?;
        for st in body {
            if let StmtKind::Decl(d) = &st.kind {
                low.declare(st, d)?;
            }
        }
        Ok(low.n_q)
    }

    /// A row, to the rows its relation is lowered to.
    fn push(&mut self, row: crate::locus::Row) {
        match self.target {
            Target::Alg => self.rows.push(row),
            Target::Rate => self.rates.push(row),
            Target::Init => self.init.push(row),
        }
    }

    /// **A relation that reads a rate** (#339 D2), rewritten to one over scratch geometry: each
    /// `rate(p over s)` the line from an origin (its slots a zero) to `p`'s rate (its slots the
    /// rates of `p`'s coordinates) — a direction, which `parallel`, `perpendicular` and a
    /// directed `angle` read — and a rate alone before `distance(k, along: l)` the two points,
    /// so the ordinate is the rate's component along `l`.
    fn rate_relation(
        &mut self, st: &Stmt, r: &crate::ir::Relation,
    ) -> Result<crate::ir::Relation, (Span, String)> {
        let (r, rf, unary) = rate_operand(st, r, &self.swept, self.angle)?;
        let e = self
            .scope
            .get(&rf.root.text)
            .copied()
            .ok_or((rf.span, format!("no such entity: `{}`", rf.root.text)))?;
        let e = follow(&self.sk, e, &rf.path).map_err(|m| (rf.span, m))?;
        if e.kind != EntKind::Point {
            return Err((rf.span, format!("`rate(… over {})` is of a point: {} has no rate the \
                body integrates", self.swept, e.kind.a())));
        }
        let line = self.rate_line(e.i(), rf.span)?;
        let k = r.form.written().map_or(0, |w| w.rates[0].op);
        Ok(rate_rewritten(r, &[(k, line, rf.span)], unary))
    }

    /// The scratch line from the origin to point `p`'s rate, made once a point, by name.
    fn rate_line(&mut self, p: usize, span: Span) -> Result<String, (Span, String)> {
        if let Some(n) = self.rated.get(&p) {
            return Ok(n.clone());
        }
        let ps = self.sk.point_params(p);
        let mut tips = Vec::with_capacity(ps.len());
        for &c in &ps {
            match self.slot.get(&c) {
                Some(&s) if (self.n_outer..self.n_outer + self.n_q).contains(&s) => {
                    tips.push(RATE_BASE + s - self.n_outer)
                }
                _ => return Err((span, format!("the point is geometry the component is \
                    written over, and does not move with `{}`: its rate is no rate of the \
                    body", self.swept))),
            }
        }
        let zero = match self.origin {
            Some(z) => z,
            None => {
                // every declaration was consumed in pass 1, so `n_q` is final here
                let z = self.n_outer + self.n_q + self.w.len();
                let t = self.constant(0.0);
                self.w.push(t);
                self.origin = Some(z);
                z
            }
        };
        let name = format!("#rate{}", self.rated.len());
        let o = self.sk.point(0.0, 0.0, false, &format!("{name}.o"));
        let t = self.sk.point(0.0, 0.0, false, &format!("{name}.t"));
        for c in self.sk.point_params(o) {
            self.slot.insert(c, zero);
        }
        for (c, s) in self.sk.point_params(t).into_iter().zip(tips) {
            self.slot.insert(c, s);
        }
        let l = EntRef::line(self.sk.line(o, t));
        self.scope.insert(format!("{name}.o"), EntRef::point(o));
        self.scope.insert(format!("{name}.t"), EntRef::point(t));
        self.scope.insert(name.clone(), l);
        self.rated.insert(p, name.clone());
        Ok(name)
    }

    /// A body whose rows and rate rows leave its rates free is refused naming the freedom
    /// (`short`).
    fn short(&self, span: Span) -> Result<(), (Span, String)> {
        let (n_q, q0) = (self.n_q, self.n_outer);
        let alg = reach(self.rows.iter().map(|r| (KERNELS[r.kid].n_res, &r.cols[..])), q0, n_q);
        let at = q0 + n_q + self.w.len();
        let rates = reach(self.rates.iter().map(|r| (KERNELS[r.kid].n_res, &r.cols[..])), at, n_q);
        let owner: Vec<String> = self.pose_of.iter().map(|(n, _)| n.clone()).collect();
        match short(&self.swept, n_q, &alg, &rates, &owner, true) {
            Some(m) => Err((span, m)),
            None => Ok(()),
        }
    }

    /// A number's text compiled over the variable table.
    fn tape(&self, text: &str, span: Span) -> Result<crate::tape::Tape, (Span, String)> {
        let ast = crate::expr::parse_in(text, self.units).map_err(|e| (span, e))?;
        crate::tape::Tape::compile(&ast.body, self.vars).map_err(|e| (span, e))
    }

    /// A number, as a tape.
    fn constant(&self, v: f64) -> crate::tape::Tape {
        crate::tape::Tape::compile(&crate::expr::Ast::Num(v, crate::units::Dim::SCALAR), self.vars)
            .expect("a number always compiles")
    }

    /// A declaration of the block drawn in the scratch sketch, each scalar of its own an inner
    /// unknown with its seed's tape.
    fn declare(&mut self, st: &Stmt, d: &Decl) -> Result<(), (Span, String)> {
        // a computed point that is not the one traced is its formula and has no unknowns here
        if d.computed.is_some() {
            return Ok(());
        }
        if d.seed_at.is_some() && d.kind != EntKind::Point {
            return Err((st.span, "only a point takes a geometric seed".to_string()));
        }
        let seed_tape = |low: &Self, i: usize| -> Result<crate::tape::Tape, (Span, String)> {
            match d.seed_text.get(i).and_then(|t| t.as_ref()) {
                Some(t) => low.tape(t, *d.seed_spans.get(i).unwrap_or(&st.span)),
                None => Ok(low.constant(d.seed.get(i).copied().unwrap_or(0.0))),
            }
        };
        let child = |sk: &Sketch,
                     scope: &BTreeMap<String, EntRef>,
                     g: usize|
         -> Result<EntRef, (Span, String)> {
            let r = d
                .children
                .get(g)
                .and_then(|v| v.first())
                .and_then(|k| k.as_ref())
                // an anonymous declaration's key is the elaboration's, not the writer's, so the
                // message spells the kind instead — `decl_head`, the one wording both the
                // parser's errors and these diagnostics use
                .ok_or_else(|| {
                    let who = crate::syntax::decl_head(d.kind, &d.name);
                    (st.span, format!("`{who}` needs its points named"))
                })?;
            let e = scope
                .get(&r.root.text)
                .copied()
                .ok_or((r.span, format!("no such entity: `{}`", r.root.text)))?;
            follow(sk, e, &r.path).map_err(|m| (r.span, m))
        };
        let e = match d.kind {
            EntKind::Point => {
                let (sx, sy) = match &d.seed_at {
                    Some(a) => at_seed(&self.sk, &self.scope, &self.slot, self.vars, &self.seeds,
                                       self.n_outer, a, st.span)?,
                    None => (seed_tape(self, 0)?, seed_tape(self, 1)?),
                };
                // a scratch sketch nobody's DOF dialog reads, so the key is a fine label here
                let e = EntRef::point(self.sk.point(0.0, 0.0, false, &d.name.key().text));
                self.seeds.push(sx);
                self.seeds.push(sy);
                for (j, p) in self.sk.own_params(e).into_iter().enumerate() {
                    self.slot.insert(p, self.n_outer + self.n_q);
                    self.pose_of.push((d.name.key().text.clone(), j));
                    self.n_q += 1;
                }
                e
            }
            EntKind::Line => {
                let a = child(&self.sk, &self.scope, 0)?;
                let b = child(&self.sk, &self.scope, 1)?;
                if a.kind != EntKind::Point || b.kind != EntKind::Point {
                    return Err((st.span, "a line runs between points".to_string()));
                }
                EntRef::line(self.sk.line(a.i(), b.i()))
            }
            EntKind::Circle => {
                let c = child(&self.sk, &self.scope, 0)?;
                if c.kind != EntKind::Point {
                    return Err((st.span, "a circle's centre is a point".to_string()));
                }
                let sr = seed_tape(self, 0)?;
                let e = EntRef::circle(self.sk.circle(c.i(), 0.0, &d.name.key().text));
                self.seeds.push(sr);
                let (j, p) = (0usize, self.sk.own_params(e)[0]);
                self.slot.insert(p, self.n_outer + self.n_q);
                self.pose_of.push((d.name.key().text.clone(), j));
                self.n_q += 1;
                e
            }
            other => {
                return Err((
                    st.span,
                    format!("a trace block cannot yet draw {}", other.a()),
                ))
            }
        };
        if self.scope.insert(d.name.key().text.clone(), e).is_some() {
            // two anonymous declarations are two offsets, so only a written name can collide —
            // but the message asks `shown` all the same, so a key cannot leak into it
            let who = d
                .name
                .shown()
                .map_or_else(|| crate::syntax::decl_head(d.kind, &d.name), |n| n.text.clone());
            return Err((st.span, format!("`{who}` is declared twice")));
        }
        Ok(())
    }

    /// The variable-table slot of scratch parameter `p`, or the lowering lost it.
    fn slot_of(&self, p: u32, span: Span) -> Result<usize, (Span, String)> {
        self.slot.get(&p).copied().ok_or((span, "trace lowering lost a column".to_string()))
    }

    /// A relation of the block lowered: an orientation to a predicate, an ordinate in the block's
    /// plane, or a row over the scratch parameters' slots, a number over `u` a tape's column.
    fn relation(&mut self, st: &Stmt, r: &crate::ir::Relation) -> Result<(), (Span, String)> {
        let (sk, scope) = (&self.sk, &self.scope);
        let r = r.resolve(&|rf| {
            scope
                .get(&rf.root.text)
                .copied()
                .and_then(|e| follow(sk, e, &rf.path).ok())
                .map(|e| e.kind)
        })?;
        // `ccw(a, b, x)` — no residual: it selects among the discrete solution components,
        // which is what a branch is, and it is how a block says one *as a fact* rather than
        // as a place to start looking
        if matches!(r.kind, CKind::Ccw | CKind::Cw) {
            return self.orientation(st, &r);
        }
        if self.target == Target::Rate {
            admissible(&r).map_err(|m| (st.span, m))?;
        }
        let spec = r.kind.spec();
        if r.kind.gauge() {
            return Err((
                st.span,
                format!(
                    "`{}` cannot appear in a trace block: a traced component holds declarations \
                     and constraints only, and its unknowns are what it places",
                    crate::syntax::snake(r.kind.name())
                ),
            ));
        }
        // a contact's place is the block's to state only where it is a fraction of a drawn
        // run (#248): a spline or a curve is no entity a block draws or is written over
        let placed = spec.iter().any(|(_, k)| k.is_param()) && r.kind.fraction_slots().is_none();
        if r.kind.soft_by_default() || r.kind.built() || placed {
            return Err((st.span, format!("{} cannot appear in a trace block", r.kind.name())));
        }
        // the block is lowered through a scratch sketch of one plane, which has no place in
        // space for a relation in space to read
        if r.kind.spatial() {
            return Err((
                st.span,
                format!(
                    "`{}` relates its operands in space, and a traced component places its points \
                     in one plane",
                    r.kind.operator().map_or("a relation", |(w, _)| w)
                ),
            ));
        }
        // **an ordinate in a trace** is read in the block's one plane: the run and the rise by
        // their word, a drawn line of the block by itself, and a plane's own `u`/`v` as the
        // point's own coordinate in it — the point is drawn there, the block having no other
        let ord = match r.kind.word_slot() {
            Some(_) => match trace_ordinate(&self.sk, &self.scope, &r, st.span)? {
                TraceOrdinate::Coordinate(p, axis) => return self.coordinate(st, &r, p, axis),
                TraceOrdinate::Form(f) => Some(f),
            },
            None => None,
        };
        let mut cargs: Vec<CArg> = Vec::with_capacity(spec.len());
        let mut dim: Option<(SpecKind, crate::tape::Tape)> = None;
        for (i, (name, kind)) in spec.iter().enumerate() {
            let given = r.args.get(i).and_then(|a| a.as_ref());
            // an ordinate's form is the block's reading of it, and a direction a word named is
            // read by that form, so its slot holds nothing
            if let Some(f) = ord {
                if r.kind.form_slot() == Some(i) {
                    cargs.push(CArg::Int(f as i64));
                    continue;
                }
                if i == 2 && given.is_none() {
                    cargs.push(CArg::Num(0.0));
                    continue;
                }
            }
            match (kind, given) {
                (k, Some(Arg::Ref(rf))) if k.is_entity() => {
                    let found = self.scope.get(&rf.root.text).copied();
                    cargs.push(ent_arg(&self.sk, found, *k, rf).map_err(|(_, m)| (rf.span, m))?);
                }
                (k, Some(Arg::Dim { text, span })) if k.is_dimension() => {
                    dim = Some((*k, self.tape(text, *span)?));
                    cargs.push(CArg::Num(0.0));
                }
                // **a place along a run** (#248) is stated as every number in the block is:
                // pinned, and read as a dimension over `u` is, a column a tape computes (`w`),
                // standing in a scratch Param the constraint's columns name
                (k, given) if k.is_param() => {
                    let place = match given {
                        Some(Arg::Seed { value, pinned: true }) => self.constant(*value),
                        Some(Arg::SeedExpr { text, pinned: true, span }) => self.tape(text, *span)?,
                        Some(Arg::Tie { name, span, .. }) if self.vars.contains(name) => {
                            self.tape(name, *span)?
                        }
                        _ => {
                            return Err((st.span, format!("`{name}` must be pinned in a trace \
                                block, `{name} == u`: a trace block infers nothing, and a point \
                                anywhere on the run is `coincident` with no `{name}`")))
                        }
                    };
                    let p = self.sk.param(0.0, false, name) as u32;
                    // every declaration was consumed in pass 1, so `n_q` is final here
                    self.slot.insert(p, self.n_outer + self.n_q + self.w.len());
                    self.w.push(place);
                    cargs.push(CArg::Param(p));
                }
                // a slot the core would read off the geometry (a tangency's side or sense) is
                // required too: there is no drawn geometry here to read it from, and a default
                // would silently pick the branch
                (k, None) if k.is_dimension() || k.is_entity() || r.kind.infers_arg(i) => {
                    return Err((
                        st.span,
                        format!("`{name}` must be stated: a trace block infers nothing"),
                    ));
                }
                (_, None) => cargs.push(r.kind.default_arg(i)),
                (k, Some(a)) => match scalar_arg(*k, a) {
                    Some(v) => cargs.push(v),
                    None => {
                        return Err((
                            st.span,
                            format!("`{name}`: {} is wanted here, not {a:?}", k.a()),
                        ))
                    }
                },
            }
        }
        let c = Constraint::new(r.kind, cargs);
        let mut cols: Vec<u32> = Vec::new();
        for p in c.params(&self.sk) {
            cols.push(self.slot_of(p, st.span)? as u32);
        }
        let kid = match dim {
            Some((k, t)) => {
                // an ordinate's twin is its form's
                let form_twin = ord.map(|f| match f {
                    OrdinateForm::PageU => crate::kernels::K::OrdinateUFree,
                    OrdinateForm::PageV => crate::kernels::K::OrdinateVFree,
                    _ => crate::kernels::K::OrdinateLineFree,
                });
                let twin = form_twin.or(r.kind.free_kernel()).ok_or((
                    st.span,
                    format!("{} cannot be stated over `u` here", crate::model::article(&crate::syntax::snake(r.kind.name()))),
                ))?;
                // every declaration was consumed in pass 1, so `n_q` is final here
                cols.push((self.n_outer + self.n_q + self.w.len()) as u32);
                self.w.push(t);
                // the tape works in the units a person writes (degrees); (m, c) are the
                // conversion to what the kernel reads, the same seam `expr::set_dimension` is,
                // turned by a word that says which way as a stated number is
                let sign = c.side().unwrap_or(1.0);
                self.push(crate::locus::Row {
                    kid: twin as usize,
                    cols,
                    consts: vec![sign * crate::expr::to_arg_units(k, 1.0), 0.0],
                });
                return Ok(());
            }
            None => c.kernel_id(),
        };
        let consts = c.consts_on(&self.sk, None);
        self.push(crate::locus::Row { kid, cols, consts });
        Ok(())
    }

    /// `ccw(a, b, x)` lowered to a predicate over the three points' slots, the third one the
    /// block places: the point a violated predicate reflects.
    fn orientation(
        &mut self, st: &Stmt, r: &crate::ir::ResolvedRelation<'_>,
    ) -> Result<(), (Span, String)> {
        let pts: Vec<&Ref> = r
            .args
            .iter()
            .filter_map(|a| match a {
                Some(Arg::Ref(rf)) => Some(rf),
                _ => None,
            })
            .collect();
        if pts.len() != 3 {
            return Err((st.span, "an orientation names three points".to_string()));
        }
        let mut cols = [0u32; 6];
        for (k, rf) in pts.iter().enumerate() {
            let e = self
                .scope
                .get(&rf.root.text)
                .copied()
                .ok_or((rf.span, format!("no such entity: `{}`", rf.root.text)))?;
            let e = follow(&self.sk, e, &rf.path).map_err(|m| (rf.span, m))?;
            if e.kind != EntKind::Point {
                return Err((rf.span, "an orientation is about points".to_string()));
            }
            let ps = self.sk.point_params(e.i());
            for (j, &p) in ps.iter().enumerate() {
                cols[2 * k + j] = self.slot_of(p, st.span)? as u32;
            }
        }
        // the placed point is the one a violated predicate reflects, so it must be one
        // the block actually places
        if (cols[4] as usize) < self.n_outer {
            return Err((
                pts[2].span,
                "the third point must be one the block places".to_string(),
            ));
        }
        self.preds.push(crate::locus::Pred { ccw: r.kind == CKind::Ccw, cols });
        Ok(())
    }

    /// An ordinate along a plane's own `u` or `v`: point `p`'s coordinate `axis`, its number over
    /// the swept formal `radius`'s free twin over the one column, as any dimension's is — a level
    /// that coordinate at nothing.
    fn coordinate(
        &mut self, st: &Stmt, r: &crate::ir::ResolvedRelation<'_>, p: usize, axis: usize,
    ) -> Result<(), (Span, String)> {
        let col = self.sk.point_params(p)[axis];
        let col = self.slot_of(col, st.span)?;
        match r.args.get(3).and_then(|a| a.as_ref()) {
            // the coordinate's number over the swept formal: `radius`'s free twin
            // over the one column, as any dimension's is
            Some(Arg::Dim { text, span }) if r.kind == CKind::Ordinate => {
                let cols = vec![col as u32, (self.n_outer + self.n_q + self.w.len()) as u32];
                let t = self.tape(text, *span)?;
                self.w.push(t);
                let units = crate::expr::to_arg_units(SpecKind::Length, 1.0);
                let kid = crate::kernels::K::RadiusFree as usize;
                self.push(crate::locus::Row { kid, cols, consts: vec![units, 0.0] });
            }
            _ if r.kind == CKind::Level => self.push(crate::locus::Row {
                kid: crate::kernels::K::Radius as usize,
                cols: vec![col as u32],
                consts: vec![0.0],
            }),
            _ => {
                return Err((st.span, "`d` must be stated: a trace block infers nothing"
                    .to_string()))
            }
        }
        Ok(())
    }
}

/// How a trace block reads an ordinate (`compile_trace`).
enum TraceOrdinate {
    /// Point `.0`'s own coordinate `.1` (0 its x, 1 its y) in the plane it is measured against.
    Coordinate(usize, usize),
    /// Between two of the block's points, by a form of its one plane.
    Form(OrdinateForm),
}

/// An ordinate or a level in a trace block, read in the block's one plane: a page word between
/// two points is the run or the rise, a line the block draws is a direction in it, a plane's `u`
/// or `v` is the point's own coordinate; any other direction, and a plane's normal, are in space.
fn trace_ordinate(
    sk: &Sketch,
    scope: &BTreeMap<String, EntRef>,
    r: &crate::ir::ResolvedRelation<'_>,
    span: Span,
) -> Result<TraceOrdinate, (Span, String)> {
    use crate::constraints::Toward;
    let ent = |i: usize| -> Option<EntRef> {
        let Some(Arg::Ref(rf)) = r.args.get(i).and_then(|a| a.as_ref()) else { return None };
        scope.get(&rf.root.text).copied().and_then(|e| follow(sk, e, &rf.path).ok())
    };
    let ws = r.kind.word_slot().expect("an ordinate has a word slot");
    let word = match r.args.get(ws).and_then(|a| a.as_ref()) {
        Some(Arg::Word(w)) => Toward::of(w),
        _ => None,
    };
    let in_space = || {
        let w = r.kind.operator().map_or("an ordinate", |(w, _)| w);
        (span, format!("`{w}` relates its operands in space, and a traced component places its \
            points in one plane"))
    };
    let plane = ent(1).filter(|e| e.kind == EntKind::Frame);
    Ok(match (plane, word, ent(2)) {
        (Some(_), Some(Toward::PlaneU | Toward::PlaneV), _) => {
            let p = ent(0).filter(|e| e.kind == EntKind::Point).ok_or_else(in_space)?;
            TraceOrdinate::Coordinate(p.i(), usize::from(word == Some(Toward::PlaneV)))
        }
        // between two points `u` and `v` are the view's, as `right` and `up` are
        (None, Some(t), _) if t.in_view() == Some(Toward::PageU) => {
            TraceOrdinate::Form(OrdinateForm::PageU)
        }
        (None, Some(t), _) if t.in_view() == Some(Toward::PageV) => {
            TraceOrdinate::Form(OrdinateForm::PageV)
        }
        (None, None, Some(t)) if t.kind == EntKind::Line => {
            TraceOrdinate::Form(OrdinateForm::InView)
        }
        _ => return Err(in_space()),
    })
}

/// A seed named geometrically, compiled to the tapes a written pair would be: the place a point
/// already names, or **the point at the edge of a circle** at a bearing from the x-axis of the
/// coordinates the circle is drawn in —
/// `hint(at: c, bearing: u + phase)`, which is what that place is called in this language rather
/// than the trigonometry it comes to.
fn at_seed(
    sk: &Sketch,
    scope: &BTreeMap<String, EntRef>,
    slot: &BTreeMap<u32, usize>,
    vars: &[String],
    seeds: &[crate::tape::Tape],
    n_outer: usize,
    a: &crate::syntax::AtRef,
    span: Span,
) -> Result<(crate::tape::Tape, crate::tape::Tape), (Span, String)> {
    use crate::expr::{Ast, Op};
    use crate::tape::Tape;
    let e = scope
        .get(&a.what.root.text)
        .copied()
        .ok_or((a.what.span, format!("no such entity: `{}`", a.what.root.text)))?;
    let e = follow(sk, e, &a.what.path).map_err(|m| (a.what.span, m))?;
    // what one scratch parameter is seeded from: a formal's coordinate is a variable of the
    // family; an inner point's is whatever seeded it, which is why declaration order matters
    let of = |p: u32| -> Result<Tape, (Span, String)> {
        let s = *slot.get(&p).ok_or((span, "trace lowering lost a column".to_string()))?;
        if s < n_outer {
            Tape::compile(&Ast::Var(vars[s].clone()), vars).map_err(|m| (span, m))
        } else {
            seeds.get(s - n_outer).cloned().ok_or((
                a.what.span,
                format!("`{}` is declared after this point", a.what.root.text),
            ))
        }
    };
    if let Some(t) = a.toward.as_ref().or(a.along.as_ref()) {
        // composing the two places' seed tapes with the step is the case nobody has asked for
        return Err((t.span, "a traced point's seed is a place, not a step toward one".to_string()));
    }
    match (e.kind, &a.bearing) {
        (EntKind::Point, None) => {
            let ps = sk.point_params(e.i());
            Ok((of(ps[0])?, of(ps[1])?))
        }
        (EntKind::Point, Some(_)) => {
            Err((span, "a point is already a place; a bearing needs a circle".to_string()))
        }
        (EntKind::Circle, Some((text, bsp))) => {
            let c = &sk.circles[e.i()];
            // the edge of an *inner* circle would need its seed tapes composed with the
            // bearing's; the case nobody has asked for yet
            let name = |p: u32| -> Result<String, (Span, String)> {
                match slot.get(&p) {
                    Some(&s) if s < n_outer => Ok(vars[s].clone()),
                    _ => Err((
                        a.what.span,
                        "the circle must be one the family is written over".to_string(),
                    )),
                }
            };
            let ctr = sk.point_params(c.center as usize);
            let (cx, cy, r) = (name(ctr[0])?, name(ctr[1])?, name(c.radius)?);
            let beta = crate::expr::parse_in(text, sk.units).map_err(|m| (*bsp, m))?.body;
            let coord = |centre: &str, trig: &str| -> Result<Tape, (Span, String)> {
                let ast = Ast::Bin(
                    Op::Add,
                    Box::new(Ast::Var(centre.to_string())),
                    Box::new(Ast::Bin(
                        Op::Mul,
                        Box::new(Ast::Var(r.clone())),
                        Box::new(Ast::Call(trig.to_string(), vec![beta.clone()])),
                    )),
                );
                Tape::compile(&ast, vars).map_err(|m| (*bsp, m))
            };
            Ok((coord(&cx, "cos")?, coord(&cy, "sin")?))
        }
        (EntKind::Circle, None) => Err((
            span,
            "where on the edge?  `hint(at: c, bearing: …)` says the bearing".to_string(),
        )),
        (k, _) => Err((a.what.span, format!("a seed cannot be at {}", k.a()))),
    }
}
/// A curve instance: a family, the entities it is written over, and the numbers it takes.
pub(super) fn build_curve(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    st: &Stmt,
    diags: &mut Vec<Diag>,
    prog: &Program,
    insts: &[crate::flatten::InstanceInfo],
) -> Option<EntRef> {
    // a body traced in space is no curve of a plane (#326): built as what it is — over two
    // formals or a motion, compiled once the motions are built (`traced::traced_bodies`)
    if super::traced::in_space_traced(sk, res, d, insts) {
        sk.traced.push(super::traced::placeholder(d));
        return Some(EntRef::new(EntKind::Traced, sk.traced.len() - 1));
    }
    match curve_entity(sk, res, d, st, prog, insts) {
        Ok(Some(cv)) => {
            sk.curves.push(cv);
            Some(EntRef::new(EntKind::Curve, sk.curves.len() - 1))
        }
        Ok(None) => None,
        // a body that does not read as a plane's may read as one in space; where neither does,
        // what the plane's says
        Err((code, span, message)) => match super::traced::build_traced(sk, res, d, st, prog, insts) {
            Ok(Some(t)) => {
                sk.traced.push(t);
                Some(EntRef::new(EntKind::Traced, sk.traced.len() - 1))
            }
            _ => {
                diags.push(Diag::within(code, span, st.id, message));
                None
            }
        },
    }
}

/// **A point of the drawing traced as a declared parameter runs** (#391 S4, `trace(m over t)`):
/// a curve of the point's view over the parameter's interval, or a stretch of it, whose points are
/// wherever the drawing followed along the parameter puts the point (`CurveBody::Followed`).  One
/// definition per parameter, shared by every point traced over it.  Refused: a name that is no
/// point and a stretch outside the parameter's interval.  A point in space is traced in space
/// (#398 F, `Sketch::followed_world`), drawn in the scene and in no view.
fn followed(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    c: &crate::syntax::CurveSpec,
    point: &str,
) -> Result<crate::model::CurveE, (Code, Span, String)> {
    let swept = c.swept.text.as_str();
    let span = c.swept.span;
    let p = res.of.get(point).copied().filter(|e| e.kind == EntKind::Point).ok_or_else(|| {
        (Code::E040, span, format!("`{point}` is traced as `{swept}` runs, and only a point is"))
    })?;
    let Some(running) = sk.parameters.get(swept).cloned() else {
        return Err((Code::E103, span, format!("`{swept}` is no parameter the document declares")));
    };
    let num = |t: &str| crate::expr::literal(t);
    let (from, to) = (num(&c.domain.0).unwrap_or(running.from), num(&c.domain.1)
        .unwrap_or(running.to));
    if from < running.from || to > running.to || from >= to {
        return Err((Code::E103, span, format!("a trace over `{swept}` runs over an increasing \
            stretch of its interval, `({}, {})`", crate::syntax::num(running.from),
            crate::syntax::num(running.to))));
    }
    let key = format!("followed:{swept}");
    let def = match sk.curve_defs.iter().position(|x| x.name == key) {
        Some(i) => i,
        None => {
            sk.curve_defs.push(crate::model::CurveDef {
                name: key,
                component: String::new(),
                port: String::new(),
                formals: vec![("p".to_string(), EntKind::Point)],
                columns: Vec::new(),
                values: Vec::new(),
                param: swept.to_string(),
                turns: running.turns(),
                vars: Vec::new(),
                body: crate::model::CurveBody::Followed(None),
                pose_of: Vec::new(),
                home: false,
            });
            sk.curve_defs.len() - 1
        }
    };
    Ok(crate::model::CurveE {
        def: def as u32,
        args: vec![p],
        unknowns: Vec::new(),
        values: Vec::new(),
        over: crate::model::Running::new(swept, running.dim, from, to),
        home: crate::model::Home::At(from),
        pose: Vec::new(),
        class: d.class.clone(),
        trim: None,
        extrusion: false,
        length: None,
        pegs: Vec::new(),
        slides: Vec::new(),
    })
}

/// The curve a declaration draws: its definition (compiled on first use, shared after), the
/// entities and numbers the instance gave it, and where its trace is anchored.  `Ok(None)` when
/// the flattener already reported what is wrong with it.
fn curve_entity(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    st: &Stmt,
    prog: &Program,
    insts: &[crate::flatten::InstanceInfo],
) -> Result<Option<crate::model::CurveE>, (Code, Span, String)> {
    let Some(c) = d.curve.as_ref() else {
        return Err((
            Code::E103,
            st.span,
            "a curve is `name := trace(instance.point over formal in (a, b))`".to_string(),
        ));
    };
    // the instance the point belongs to — found by the flattener, which reported it if it
    // found none
    let Some(of) = &c.of else { return Ok(None) };
    if of.instance.is_empty() {
        return followed(sk, res, d, c, &of.point).map(Some);
    }
    let Some(info) = insts.iter().find(|i| i.prefix == of.instance) else { return Ok(None) };
    let comp = &prog.components[info.comp];
    let id = prog.component_id(info.comp);
    let swept = c.swept.text.as_str();
    let numeric = comp
        .formals
        .iter()
        .any(|f| f.name.text == swept && !matches!(f.ty, crate::syntax::Ty::Ent(_) | crate::syntax::Ty::Path));
    if !numeric {
        return Err((
            Code::E040,
            c.swept.span,
            format!("`{swept}` is not a numeric formal of `{}`", info.component),
        ));
    }
    // the numbers a drawn instance left unbound are the drawing's unknowns (`leg.h`), and the
    // curve is written over them as over its entities: they are its columns.  Only as
    // themselves — a formal given `2 * w` over an unknown `w` would be a column read through an
    // affine map no kernel carries
    let mut columns: Vec<String> = Vec::new();
    let mut unknowns: Vec<String> = Vec::new();
    for fo in &comp.formals {
        let name = fo.name.text.as_str();
        if name == swept || matches!(fo.ty, crate::syntax::Ty::Ent(_) | crate::syntax::Ty::Path | crate::syntax::Ty::Group | crate::syntax::Ty::Set) {
            continue;
        }
        let Some(a) = info.values.get(name) else { continue };
        let Some(free) = a.free.as_ref().filter(|_| info.drawn && a.number().is_none()) else {
            continue;
        };
        if a.m != 1.0 || a.c != 0.0 {
            return Err((
                Code::E103,
                st.span,
                format!(
                    "`{}` is given an expression in the unknown `{free}`, and a curve is written \
                     over an unknown number only as the number itself",
                    name
                ),
            ));
        }
        columns.push(name.to_string());
        unknowns.push(free.clone());
    }
    // one definition per (component, point, formal, columns), shared by every instance asked —
    // and, for a body that states a rate, per whether it is drawn: a drawn one runs from its
    // pose, whose coordinates are its columns (#339 D2)
    let key = crate::model::CurveDef::key(&id, &of.point, swept, &columns);
    let homed = format!("{key}{}", crate::model::CurveDef::HOME);
    let found = |name: &str| sk.curve_defs.iter().position(|x| x.name == name);
    let rated = |i: usize| matches!(&sk.curve_defs[i].body,
        crate::model::CurveBody::Trace(l) if l.states_rate());
    let di = match (found(&homed).filter(|_| info.drawn), found(&key)) {
        (Some(i), _) => i,
        (None, Some(i)) if !(info.drawn && rated(i)) => i,
        _ => {
            let def = compile_curve(prog, comp, &id, &info.component, swept, &of.point, &columns,
                                    info.drawn, sk.units)
                .map_err(|(span, m)| (Code::E103, span, m))?;
            sk.curve_defs.push(def);
            sk.curve_defs.len() - 1
        }
    };
    let def = &sk.curve_defs[di];
    let missing =
        |what: &str| (Code::E101, st.span, format!("`{}` was not given `{what}`", info.component));
    // the entities the instance was given, in the component's order
    let mut args: Vec<EntRef> = Vec::with_capacity(def.formals.len());
    for ((fname, k), (_, actual)) in def.formals.iter().zip(&info.ents) {
        let abs = actual.as_ref().ok_or_else(|| missing(fname))?;
        let e = *res
            .of
            .get(abs)
            .ok_or_else(|| (Code::E101, st.span, format!("no such entity: `{abs}`")))?;
        if e.kind != *k {
            return Err((
                Code::E040,
                st.span,
                format!("`{fname}` is {}, and {} was given", k.a(), e.kind.a()),
            ));
        }
        args.push(e);
    }
    // the numbers it was given, in the definition's order
    let values = def
        .values
        .iter()
        .map(|n| info.values.get(n).and_then(|a| a.number()).ok_or_else(|| missing(n)))
        .collect::<Result<Vec<f64>, _>>()?;
    let num = |t: &str| crate::expr::literal(t);
    // the swept formal, one parameter axis (#391 S1)
    let dim = comp.formals.iter().find(|f| f.name.text == swept)
        .map_or(crate::units::Dim::SCALAR, |f| f.ty.dim());
    let over = crate::model::Running::new(swept, dim, num(&c.domain.0).unwrap_or(0.0),
                                          num(&c.domain.1).unwrap_or(1.0));
    // the anchor: what the swept formal was given — a number, or the drawing's unknown a drawn
    // instance left it as — or the interval's start for an instance written in place
    let home = match info.values.get(swept).map(|a| (a.number(), &a.free)) {
        Some((Some(v), _)) => crate::model::Home::At(v),
        Some((None, Some(n))) if info.drawn => crate::model::Home::Free(n.clone()),
        _ => crate::model::Home::At(over.from),
    };
    // the pose a drawn instance stands in, per inner unknown of the block — whole or nothing
    let pose: Vec<(EntRef, usize)> = if info.drawn {
        def.pose_of
            .iter()
            .filter_map(|(n, j)| res.of.get(&format!("{}{n}", of.instance)).map(|&e| (e, *j)))
            .collect()
    } else {
        Vec::new()
    };
    let pose = crate::model::whole(pose, def.pose_of.len());
    if def.home {
        if pose.is_empty() {
            return Err((Code::E103, st.span, format!("`{}` runs from its drawn instance, whose \
                pose is not all on the sheet", info.component)));
        }
        if !matches!(home, crate::model::Home::At(_)) {
            return Err((Code::E103, st.span, format!("a body that states a rate is drawn at its \
                home, where it starts: give `{swept}` a number")));
        }
    }
    Ok(Some(crate::model::CurveE {
        def: di as u32,
        args,
        unknowns,
        values,
        over,
        home,
        pose,
        class: d.class.clone(),
        trim: None,
        extrusion: false,
        length: None,
        pegs: Vec::new(),
        slides: Vec::new(),
    }))
}
