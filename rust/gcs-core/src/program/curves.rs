//! Compile computed curves and traces into model definitions.

use super::relations::{ent_arg, scalar_arg};
use super::resolve::{follow, Resolver};
use super::{Code, Diag, Severity};
use crate::constraints::{Arg as CArg, CKind, Constraint, OrdinateForm, SpecKind};
use crate::ir::{Decl, Operation as StmtKind, Statement as Stmt};
use crate::model::{EntKind, EntRef, Sketch};
use crate::syntax::{Arg, Program, Ref, Span};
use std::collections::BTreeMap;

/// Compile a component point over a numeric formal. Variable order is the swept
/// formal, entity-formal coordinates in model order, then other numeric formals;
/// it must match kernel Jacobian columns.
/// `id` is the component's identity across the program (`Program::component_id`), which keys
/// the definition, and `written` how the call spelled it, which a lifted program writes.
fn compile_curve(
    prog: &Program,
    comp: &crate::syntax::Component,
    id: &str,
    written: &str,
    swept: &str,
    point: &str,
    columns: &[String],
    units: crate::units::Units,
) -> Result<crate::model::CurveDef, (Span, String)> {
    use crate::syntax::Ty;
    let cname = comp.name.as_ref().map(|n| n.text.clone()).unwrap_or_default();
    let mut vars = vec![swept.to_string()];
    let mut formals = Vec::new();
    let mut values = Vec::new();
    for fo in &comp.formals {
        match fo.ty {
            Ty::Group => return Err((fo.span,
                "a curve needs fixed scalar or entity formals; pass the group's members individually".into())),
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
            let (locus, pose_of) =
                compile_trace(comp.span, &traced, &body, &vars, &formals, columns.len(),
                              values.len(), units)?;
            (crate::model::CurveBody::Trace(locus), pose_of)
        }
    };
    Ok(crate::model::CurveDef {
        name: crate::model::CurveDef::key(id, point, swept, columns),
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

/// Lower a trace into scratch geometry and tapes, using `Constraint::params_on`
/// for kernel columns. Swept dimensions use free-twin kernels whose final column
/// reads a tape-derived value, including its gradient.
#[allow(clippy::too_many_arguments)]
fn compile_trace(
    span: Span,
    point: &str,
    body: &[&Stmt],
    vars: &[String],
    formals: &[(String, EntKind)],
    n_columns: usize,
    n_values: usize,
    units: crate::units::Units,
) -> Result<(crate::locus::Locus, Vec<(String, usize)>), (Span, String)> {
    use crate::locus::{Locus, Pred, Row};
    use crate::tape::Tape;
    let mut sk = Sketch::new();
    // the scratch sketch the block is lowered through reads numbers, so it is in the document's
    // units too — `at_seed` asks it what a bearing's literal is written in
    sk.units = units;
    let mut scope: BTreeMap<String, EntRef> = BTreeMap::new();
    // scratch parameter index -> variable-table slot
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
            EntKind::Plane => EntRef::plane(sk.fixed_plane(crate::plane::Basis::page(), name)),
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
    // the numeric columns follow the entities' scalars; nothing in the block is built over them,
    // and they are read only through the tapes of the dimensions that name them
    let n_theta = next - 1 + n_columns;
    let n_outer = vars.len();
    debug_assert_eq!(n_outer, 1 + n_theta + n_values, "variable table shape");
    let tape = |text: &str, span: Span| -> Result<Tape, (Span, String)> {
        let ast = crate::expr::parse_in(text, units).map_err(|e| (span, e))?;
        Tape::compile(&ast.body, vars).map_err(|e| (span, e))
    };
    let constant = |v: f64| -> Tape {
        Tape::compile(&crate::expr::Ast::Num(v, crate::units::Dim::SCALAR), vars)
            .expect("a number always compiles")
    };

    // -- pass 1: declarations, so a statement may read a point declared after it --------
    let mut n_q = 0usize;
    let mut seeds: Vec<Tape> = Vec::new();
    // each inner unknown's owner, so a drawn instance's pose can be read off the sheet
    let mut pose_of: Vec<(String, usize)> = Vec::new();
    for st in body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        // a computed point that is not the one traced is its formula and has no unknowns here
        if d.computed.is_some() {
            continue;
        }
        if d.seed_at.is_some() && d.kind != EntKind::Point {
            return Err((st.span, "only a point takes a geometric seed".to_string()));
        }
        let seed_tape = |i: usize| -> Result<Tape, (Span, String)> {
            match d.seed_text.get(i).and_then(|t| t.as_ref()) {
                Some(t) => tape(t, *d.seed_spans.get(i).unwrap_or(&st.span)),
                None => Ok(constant(d.seed.get(i).copied().unwrap_or(0.0))),
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
                    Some(a) => at_seed(&sk, &scope, &slot, vars, &seeds, n_outer, a, st.span)?,
                    None => (seed_tape(0)?, seed_tape(1)?),
                };
                // a scratch sketch nobody's DOF dialog reads, so the key is a fine label here
                let e = EntRef::point(sk.point(0.0, 0.0, false, &d.name.key().text));
                seeds.push(sx);
                seeds.push(sy);
                for (j, p) in sk.own_params(e).into_iter().enumerate() {
                    slot.insert(p, n_outer + n_q);
                    pose_of.push((d.name.key().text.clone(), j));
                    n_q += 1;
                }
                e
            }
            EntKind::Line => {
                let a = child(&sk, &scope, 0)?;
                let b = child(&sk, &scope, 1)?;
                if a.kind != EntKind::Point || b.kind != EntKind::Point {
                    return Err((st.span, "a line runs between points".to_string()));
                }
                EntRef::line(sk.line(a.i(), b.i()))
            }
            EntKind::Circle => {
                let c = child(&sk, &scope, 0)?;
                if c.kind != EntKind::Point {
                    return Err((st.span, "a circle's centre is a point".to_string()));
                }
                let sr = seed_tape(0)?;
                let e = EntRef::circle(sk.circle(c.i(), 0.0, &d.name.key().text));
                seeds.push(sr);
                let (j, p) = (0usize, sk.own_params(e)[0]);
                slot.insert(p, n_outer + n_q);
                pose_of.push((d.name.key().text.clone(), j));
                n_q += 1;
                e
            }
            other => {
                return Err((
                    st.span,
                    format!("a trace block cannot yet draw {}", other.a()),
                ))
            }
        };
        if scope.insert(d.name.key().text.clone(), e).is_some() {
            // two anonymous declarations are two offsets, so only a written name can collide —
            // but the message asks `shown` all the same, so a key cannot leak into it
            let who = d
                .name
                .shown()
                .map_or_else(|| crate::syntax::decl_head(d.kind, &d.name), |n| n.text.clone());
            return Err((st.span, format!("`{who}` is declared twice")));
        }
    }

    // -- pass 2: constraints and orientations, lowered to rows and predicates ----------
    let mut w: Vec<Tape> = Vec::new();
    let mut rows: Vec<Row> = Vec::new();
    let mut preds: Vec<Pred> = Vec::new();
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
        let r = r.resolve(&|rf| {
            scope
                .get(&rf.root.text)
                .copied()
                .and_then(|e| follow(&sk, e, &rf.path).ok())
                .map(|e| e.kind)
        })?;
        // `ccw(a, b, x)` — no residual: it selects among the discrete solution components,
        // which is what a branch is, and it is how a block says one *as a fact* rather than
        // as a place to start looking
        if matches!(r.kind, CKind::Ccw | CKind::Cw) {
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
                let e = scope
                    .get(&rf.root.text)
                    .copied()
                    .ok_or((rf.span, format!("no such entity: `{}`", rf.root.text)))?;
                let e = follow(&sk, e, &rf.path).map_err(|m| (rf.span, m))?;
                if e.kind != EntKind::Point {
                    return Err((rf.span, "an orientation is about points".to_string()));
                }
                let ps = sk.point_params(e.i());
                for (j, &p) in ps.iter().enumerate() {
                    let Some(&s) = slot.get(&p) else {
                        return Err((st.span, "trace lowering lost a column".to_string()));
                    };
                    cols[2 * k + j] = s as u32;
                }
            }
            // the placed point is the one a violated predicate reflects, so it must be one
            // the block actually places
            if (cols[4] as usize) < n_outer {
                return Err((
                    pts[2].span,
                    "the third point must be one the block places".to_string(),
                ));
            }
            preds.push(Pred { ccw: r.kind == CKind::Ccw, cols });
            continue;
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
        if r.kind.soft_by_default() || r.kind.built() || spec.iter().any(|(_, k)| k.is_param()) {
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
            Some(_) => match trace_ordinate(&sk, &scope, &r, st.span)? {
                TraceOrdinate::Coordinate(p, axis) => {
                    let col = sk.point_params(p)[axis];
                    let Some(&col) = slot.get(&col) else {
                        return Err((st.span, "trace lowering lost a column".to_string()));
                    };
                    match r.args.get(3).and_then(|a| a.as_ref()) {
                        // the coordinate's number over the swept formal: `radius`'s free twin
                        // over the one column, as any dimension's is
                        Some(Arg::Dim { text, span }) if r.kind == CKind::Ordinate => {
                            let cols = vec![col as u32, (n_outer + n_q + w.len()) as u32];
                            w.push(tape(text, *span)?);
                            let units = crate::expr::to_arg_units(SpecKind::Length, 1.0);
                            let kid = crate::kernels::K::RadiusFree as usize;
                            rows.push(Row { kid, cols, consts: vec![units, 0.0] });
                        }
                        _ if r.kind == CKind::Level => rows.push(Row {
                            kid: crate::kernels::K::Radius as usize,
                            cols: vec![col as u32],
                            consts: vec![0.0],
                        }),
                        _ => {
                            return Err((st.span, "`d` must be stated: a trace block infers nothing"
                                .to_string()))
                        }
                    }
                    continue;
                }
                TraceOrdinate::Form(f) => Some(f),
            },
            None => None,
        };
        let mut cargs: Vec<CArg> = Vec::with_capacity(spec.len());
        let mut dim: Option<(SpecKind, Tape)> = None;
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
                    let found = scope.get(&rf.root.text).copied();
                    cargs.push(ent_arg(&sk, found, *k, rf).map_err(|(_, m)| (rf.span, m))?);
                }
                (k, Some(Arg::Dim { text, span })) if k.is_dimension() => {
                    dim = Some((*k, tape(text, *span)?));
                    cargs.push(CArg::Num(0.0));
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
        for p in c.params(&sk) {
            let Some(&s) = slot.get(&p) else {
                return Err((st.span, "trace lowering lost a column".to_string()));
            };
            cols.push(s as u32);
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
                cols.push((n_outer + n_q + w.len()) as u32);
                w.push(t);
                // the tape works in the units a person writes (degrees); (m, c) are the
                // conversion to what the kernel reads, the same seam `expr::set_dimension` is,
                // turned by a word that says which way as a stated number is
                let sign = c.side().unwrap_or(1.0);
                rows.push(Row {
                    kid: twin as usize,
                    cols,
                    consts: vec![sign * crate::expr::to_arg_units(k, 1.0), 0.0],
                });
                continue;
            }
            None => c.kernel_id(),
        };
        let consts = c.consts_on(&sk, None);
        rows.push(Row { kid, cols, consts });
    }
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
    let locus =
        Locus::new(n_outer, n_theta, n_q, traced, w, seeds, rows, preds).map_err(|m| (span, m))?;
    Ok((locus, pose_of))
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
    let plane = ent(1).filter(|e| e.kind == EntKind::Plane);
    Ok(match (plane, word, ent(2)) {
        (Some(_), Some(Toward::PlaneU | Toward::PlaneV), _) => {
            let p = ent(0).filter(|e| e.kind == EntKind::Point).ok_or_else(in_space)?;
            TraceOrdinate::Coordinate(p.i(), usize::from(word == Some(Toward::PlaneV)))
        }
        (None, Some(Toward::PageU), _) => TraceOrdinate::Form(OrdinateForm::PageU),
        (None, Some(Toward::PageV), _) => TraceOrdinate::Form(OrdinateForm::PageV),
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
    match curve_entity(sk, res, d, st, prog, insts) {
        Ok(Some(cv)) => {
            sk.curves.push(cv);
            Some(EntRef::new(EntKind::Curve, sk.curves.len() - 1))
        }
        Ok(None) => None,
        Err((code, span, message)) => {
            diags.push(Diag { code, span, stmt: Some(st.id), message });
            None
        }
    }
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
            "a curve is `name := instance.point over formal in (a, b)`".to_string(),
        ));
    };
    // the instance the point belongs to — found by the flattener, which reported it if it
    // found none
    let Some(of) = &c.of else { return Ok(None) };
    let Some(info) = insts.iter().find(|i| i.prefix == of.instance) else { return Ok(None) };
    let comp = &prog.components[info.comp];
    let id = prog.component_id(info.comp);
    let swept = c.swept.text.as_str();
    let numeric = comp
        .formals
        .iter()
        .any(|f| f.name.text == swept && !matches!(f.ty, crate::syntax::Ty::Ent(_)));
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
        if name == swept || matches!(fo.ty, crate::syntax::Ty::Ent(_) | crate::syntax::Ty::Group) {
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
    // one definition per (component, point, formal, columns), shared by every instance asked
    let key = crate::model::CurveDef::key(&id, &of.point, swept, &columns);
    let di = match sk.curve_defs.iter().position(|x| x.name == key) {
        Some(i) => i,
        None => {
            let def = compile_curve(prog, comp, &id, &info.component, swept, &of.point, &columns,
                                    sk.units)
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
    let domain = (num(&c.domain.0).unwrap_or(0.0), num(&c.domain.1).unwrap_or(1.0));
    // the anchor: what the swept formal was given — a number, or the drawing's unknown a drawn
    // instance left it as — or the interval's start for an instance written in place
    let home = match info.values.get(swept).map(|a| (a.number(), &a.free)) {
        Some((Some(v), _)) => crate::model::Home::At(v),
        Some((None, Some(n))) if info.drawn => crate::model::Home::Free(n.clone()),
        _ => crate::model::Home::At(domain.0),
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
    Ok(Some(crate::model::CurveE {
        def: di as u32,
        args,
        unknowns,
        values,
        domain,
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
