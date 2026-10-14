//! Settle relations into constraints and resolve their arguments.  Gauges are applied in
//! `gauges`, and region terms built in `regions`.

use super::resolve::{follow, Operand, Resolver};
use super::gauges::{apply_gauge, fix_spelling};
use super::regions::{
    ends_term, face_term, probe_of, region_term, settled_number, term_name, term_number,
    written_number,
};
use super::{Code, Diag, Elab, SourceMap};
use crate::constraints::{Arg as CArg, CKind, Constraint, SpecKind};
use crate::ir::{PathStep, Relation, ResolvedRelation, Statement as Stmt};
use crate::model::{EntKind, EntRef, Sketch, Toward};
use crate::syntax::{Arg, Ref, RelationForm, Span, StmtId};
use std::collections::BTreeSet;
use crate::fmath::Det;
use crate::{expr, io};

/// Resolve an operator to its constraint kind and registry-ordered arguments.
pub(crate) fn settle(
    w: &crate::syntax::Written,
    kind_of: &dyn Fn(&Ref) -> Option<EntKind>,
) -> Result<(CKind, Vec<Option<Arg>>), (Span, String)> {
    use crate::constraints::Fixity;
    let word = w.word.text.as_str();
    // a gauge or an orientation is settled by its word alone: which numbers `fix` may hold is
    // the entity's, and `ccw(a, b, c)` has no operand outside its parentheses; what each operand
    // must be is checked where the statement is applied
    if let Some(k) = crate::constraints::gauge_op(word) {
        if k == CKind::Fix {
            fix_spelling(w)?;
        }
        return Ok((k, w.assemble(k)?));
    }
    // `along:` is a direction word or a reference (the parser reads a name outside the table as
    // one); anything else — a number, a flag — is neither, said at the key before the operands
    // are asked about (issue #48, item 4)
    let along = w.args.iter().find_map(|a| match a {
        crate::syntax::OpArg::Named(n, v) if n.text == "along" => Some(v),
        _ => None,
    });
    if let Some(v) = along.filter(|v| !matches!(v, Arg::Ref(_) | Arg::Word(_))) {
        let m = no_direction(&crate::syntax::sel_text(v));
        return Err((w.key_span("along").unwrap_or(w.word.span), m));
    }
    let kinds: Vec<Option<EntKind>> = w.ops.iter().map(kind_of).collect();
    let kind = match w.fixity {
        Fixity::Prefix => {
            let Some(a) = kinds.first().copied().flatten() else {
                let m = format!("`{word}` needs to know what `{}` is", w.ops[0].root.text);
                return Err((w.word.span, m));
            };
            crate::constraints::prefix_op(word, a).ok_or_else(|| {
                let m = match crate::constraints::retired_prefix(word, a) {
                    // a retired spelling (#269), said with the one it became — without the
                    // operand, which in a block's copy is a name the source never wrote
                    Some(now) => format!(
                        "`{word}` relates two things and `{now}` measures one: the {now} of {} \
                         is written `{now}(…)` before it",
                        a.a(),
                    ),
                    None => format!("`{word}` does not apply to {}", a.a()),
                };
                (w.word.span, m)
            })?
        }
        Fixity::Infix => {
            let (a, b) = (kinds.first().copied().flatten(), kinds.get(1).copied().flatten());
            let (Some(a), Some(b)) = (a, b) else {
                return Err((w.word.span, format!("`{word}` needs to know what its operands are")));
            };
            // **a symmetric word reads either way round**: `P coincident p` is `p coincident P`,
            // and `t perpendicular P` and `P perpendicular t` are one statement — the operands are
            // put in the order the kind names them
            let symmetric = matches!(word, "coincident" | "parallel" | "perpendicular");
            // a selector by name, and a contact's place where it is written, pinned or seeded
            // (`t == 0.25`, `hint(t: 0.3)`): what makes a point on a run a fraction of it (#248)
            let place = w.slots().find(|(k, _)| k.text == "t").map(|(k, _)| k.span);
            let sel = |n: &str| w.sel(n).or_else(|| (n == "t" && place.is_some()).then(String::new));
            let circle = matches!((a, b), (EntKind::Point, EntKind::Circle) | (EntKind::Circle, EntKind::Point));
            if let (true, Some(at)) = (word == "coincident" && circle, place) {
                return Err((at, "a circle has no start or end, so no fraction of its turn: \
                    draw an arc, whose `t` runs from its start to its end".to_string()));
            }
            // **and a tangency between a line and a curve** (#251): the curve is named first
            // (`g tangent l`, `g tangent(at: m) l`), and a line written first is put second —
            // but for `l tangent c` of a circle, its own kind, and `arc tangent line`, at the arc's
            // end, apart from it
            let at_point = w.sel("at").is_some_and(|s| s.is_empty());
            let curved = matches!(b, EntKind::Spline | EntKind::Curve);
            let curve_last = word == "tangent"
                && a == EntKind::Line
                && (curved || (at_point && b != EntKind::Line));
            if curve_last {
                let mut swapped = w.clone();
                swapped.ops.swap(0, 1);
                return settle(&swapped, kind_of);
            }
            // and an envelope in space is named first by whatever touches it (#283)
            let either_way = symmetric || b == EntKind::Envelope;
            if either_way && crate::constraints::infix_op(word, a, b, &sel).is_none() {
                if let Some(k) = crate::constraints::infix_op(word, b, a, &sel) {
                    let mut swapped = w.clone();
                    swapped.ops.swap(0, 1);
                    return Ok((k, swapped.assemble(k)?));
                }
            }
            crate::constraints::infix_op(word, a, b, &sel).ok_or_else(|| {
                (w.word.span, format!("`{word}` does not relate {} to {}", a.a(), b.a()))
            })?
        }
        // only a gauge word is written as a call, and those were settled above
        Fixity::Call => return Err((w.word.span, format!("`{word}` is not a call"))),
    };
    // **an angle stated as another angle** (`l1 angle(l3, l4) l2`): the second pair of lines
    // stands in the parentheses where the number would, so the word between two lines is the
    // equality whenever its parentheses hold entities.  A kind states at most one number, so a
    // pair there can be nothing else — `op_args` has already read it as two references
    let kind = if kind == CKind::Angle
        && w.args.iter().any(|a| matches!(a, crate::syntax::OpArg::Ent(_)))
    {
        CKind::EqualAngle
    } else {
        kind
    };
    // the tangent at a point of a line is the line's direction (#251): `parallel`, the point
    // saying nothing a line does not
    if kind == CKind::Parallel && word == "tangent" {
        let mut along = w.clone();
        along.args.retain(|a| !matches!(a, crate::syntax::OpArg::Named(n, _) if n.text == "at"));
        return Ok((kind, along.assemble(kind)?));
    }
    Ok((kind, w.assemble(kind)?))
}

/// Whether a plane's membership (`p coincident P`, `l coincident P`) is of geometry drawn in `P`
/// already — itself or its twin there (§6.7) — so that the row would be identically zero.
fn drawn_on(sk: &Sketch, ckind: CKind, args: &[CArg]) -> bool {
    let on = |p: usize, plane: EntRef| sk.twin_in(p, plane.i()).is_some();
    match (ckind, args) {
        (CKind::PointOnPlane, [CArg::Ent(p), CArg::Ent(plane), ..]) => on(p.i(), *plane),
        (CKind::LineOnPlane, [CArg::Ent(l), CArg::Ent(plane), ..]) => {
            crate::model::edge_ends(sk, *l)
                .is_some_and(|(a, b)| on(a as usize, *plane) && on(b as usize, *plane))
        }
        _ => false,
    }
}

/// An E040 said once at its span, however many of a use's rows find it.
fn say_once(diags: &mut Vec<Diag>, span: Span, stmt: StmtId, message: String) {
    if !diags.iter().any(|d| d.span == span && d.message == message) {
        diags.push(Diag::within(Code::E040, span, stmt, message));
    }
}

/// A relation stated: settled into a constraint and added to the sketch, its id returned — or
/// applied as a gauge, taken as a region's term, or refused, and no id. The steps below, in the
/// order each may end it.
pub(super) fn constrain(cx: &mut Elab<'_>, r: &Relation, st: &Stmt) -> Option<u32> {
    // what the relation is, by what its operands resolve to (`shape`)
    let shape = r.form.written().map_or(Shape::Other, |w| shape(&cx.sk, &cx.res, w));
    if face_or_ends(cx, r, st, &shape) {
        return None;
    }
    // **a point inside or outside a plane is a bound** (§9.6): the point's ordinate along the
    // plane's normal from its origin, at most or at least zero
    if let (Shape::PlaneSide, Some(w)) = (&shape, r.form.written()) {
        let lowered = side_of_plane(r, w);
        return constrain(cx, &lowered, st);
    }
    if solid_word(cx, r, st) {
        return None;
    }
    // the defined word it was written with, its operands the entities they name (§9.9)
    // — but for a set (§6.21), which is no entity and is named as written
    let word = r.word.as_ref().and_then(|w| {
        let ops = w.ops.iter().map(|o| cx.res.entity(&cx.sk, o));
        Some(crate::constraints::WordUse {
            word: w.word.clone(),
            ops: ops.collect::<Option<Vec<EntRef>>>()?,
            args: w.args.clone(),
            sets: w.sets.clone(),
        })
    });
    // a region's term is named by the set the region's body used, where it used one (#212):
    // `tip` for `p inside tip` in `region`'s body, else the region's own name
    let term_name = r.word.as_ref().map_or_else(String::new, |w| term_name(&w.through));
    // where a set's use was written, said there
    let used = r.word.as_ref().filter(|w| !w.sets.is_empty()).map(|w| w.span);
    let touch = set_words(cx, r, st, &word)?;
    // a set's body row stated as its derivative at a tangency's point (§6.21)
    // — what moves its point, or the kind a line was expected in place of
    let along = r.along.as_ref().map(|a| {
        let ent = |x: &Ref| cx.res.entity(&cx.sk, x);
        let toward = match &a.toward {
            crate::syntax::AlongBy::Line(l) => ent(l).map(|l| match l.kind {
                EntKind::Line => Ok(Toward::Line(l.i())),
                EntKind::Axis => Ok(Toward::Axis(l.i())),
                kind => Err(kind),
            }),
            crate::syntax::AlongBy::Chart(k) => Some(Ok(Toward::Chart { k: *k, axis: None })),
        };
        (ent(&a.point), toward, a)
    });
    // what reads the operands' planes: a set's body (§6.21), its derivative along a chart moving
    // its point out of any plane (only a set's body is stated as one)
    let chart = r.along.as_ref().is_some_and(|a| matches!(a.toward, crate::syntax::AlongBy::Chart(_)));
    // made by a set's body (§6.21): read before `r` is resolved into what it states
    let from_set = r.space;
    let reader = if chart {
        super::reading::Reader::Moved
    } else if r.space {
        super::reading::Reader::Set
    } else {
        super::reading::Reader::Drawn
    };
    let r = match r
        .resolve(&|r| cx.res.entity(&cx.sk, r).map(|e| e.kind))
    {
        Ok(r) => r,
        Err((span, message)) => {
            cx.diags.push(Diag::within(Code::E040, span, st.id, message));
            return None;
        }
    };
    let ckind = r.kind;
    // a gauge is applied, not added: it holds parameters or records a root choice, and there
    // is no constraint for the map to know it by.  A claim on one is refused below the way any
    // unclaimable kind is, so it is checked first.
    if ckind.gauge() {
        gauge(cx, &r, st);
        return None;
    }
    let spec = ckind.spec();
    let (mut args, mut left_out) = arguments(cx, &r, st)?;
    // a plane's membership of what is drawn on it already holds by construction: no row.  The
    // point a plane touches a set at, and a set's body met by a point of the plane — the body is
    // a predicate, true of that point, where the same words said of it directly ask nothing
    // (E061)
    if (touch || from_set) && along.is_none() && drawn_on(&cx.sk, ckind, &args) {
        return None;
    }
    // a curve standing for a surface (a prism's side generating, `CurveE::extrusion`) is met in
    // space, by the point's place in the curve's view, whatever view the point is drawn in
    let ckind = match (ckind, args.get(1)) {
        (CKind::PointOnCurve, Some(CArg::Ent(e))) if cx.sk.curves[e.i()].extrusion => CKind::PointOnExtrusion,
        _ => ckind,
    };
    if ckind.word_slot().is_some() {
        if let Err((span, message)) =
            ordinate_operands(&cx.sk, &cx.res, &r, st, cx.doc, &mut args, &mut left_out)
        {
            cx.diags.push(Diag::within(Code::E040, span, st.id, message));
            return None;
        }
    }
    let (ckind, spec, mut args, left_out) =
        across_views(cx, &r, st, (ckind, spec), args, left_out, reader)?;
    if refused_value(cx, &r, st, ckind, spec, &args) {
        return None;
    }
    let bound = inferred(cx, &r, st, ckind, &mut args, &left_out)?;
    let mut c = Constraint::new(ckind, args);
    c.claim = r.claim;
    // an interval runs upward from its low end, the number as settled
    if let (Some((b, at)), Some(i)) = (bound, ckind.dimension_slot()) {
        if b.hi.is_some_and(|hi| hi < c.args[i].num()) {
            let message = "an interval runs from its low end to its high end".to_string();
            let span = at.unwrap_or(st.span);
            cx.diags.push(Diag::within(Code::E040, span, st.id, message));
            return None;
        }
    }
    c.bound = bound.map(|(b, _)| b);
    c.class = r.class.clone();
    c.written = written(&r.args, r.kind.spec(), st, cx.doc);
    c.word = word;
    if let Some(found) = along {
        // said once at the use, however many of the body's rows find it
        derivative(cx, st, &mut c, found, used.unwrap_or(st.span))?;
    }
    region_or_row(cx, &r, st, c, term_name)
}

/// **A region's point against a face** (#183 F1), or where an edge ends (F7): a term of the
/// region, read off the body — whether the relation was one, taken or refused.
fn face_or_ends(cx: &mut Elab<'_>, r: &Relation, st: &Stmt, shape: &Shape) -> bool {
    let probes = &mut cx.probes;
    let term = match (shape, r.form.written()) {
        (Shape::Face { point, face }, Some(w)) => {
            Some(face_term(&cx.sk, &cx.res, w, *point, face, probes))
        }
        (Shape::Ends { point, edge }, Some(w)) => Some(ends_term(w, *point, *edge, probes)),
        _ => None,
    };
    let Some(made) = term else { return false };
    match made {
        Ok((probe, t)) => probes.terms.get_mut(&probe).expect("a probe's terms").push(t),
        Err(message) => {
            cx.diags.push(Diag::on(Code::E040, st, message))
        }
    }
    true
}

/// **A word that relates two solids is a claim, judged and never solved** (§9.8).  Picked up by
/// the solids phase: nothing here has a kernel, and saying so twice would report one statement
/// twice.  And `outside` anywhere but a plane's side is refused.  Whether it was either.
fn solid_word(cx: &mut Elab<'_>, r: &Relation, st: &Stmt) -> bool {
    let Some(w) = r.form.written() else { return false };
    if w.word.text == "outside" {
        let m = "`outside` says which side of a plane a point is on: `p outside P`";
        let message = m.to_string();
        cx.diags.push(Diag::on(Code::E040, st, message));
        return true;
    }
    crate::constraints::solid_word(&w.word.text).is_some()
}

/// What a set's use may be: a set's points are points — `coincident` puts one on it, `inside` and
/// `outside` one in it or out of it, and nothing else (§6.21) — and only a plane touches a set,
/// at a point the word names (#145), each said once at the use, in its words, and none of its
/// rows stated (`None`). Else whether the use is a plane's touch: what it states besides the
/// tangency — the apex and the point on the plane — is stated only where the drawing does not
/// draw them there.
fn set_words(
    cx: &mut Elab<'_>, r: &Relation, st: &Stmt, word: &Option<crate::constraints::WordUse>,
) -> Option<bool> {
    let point_word = |w: &&crate::syntax::Worded| {
        matches!(w.word.as_str(), "coincident" | "inside" | "outside")
    };
    if let (Some(w), Some(found)) = (r.word.as_ref().filter(point_word), word) {
        if let (Some(&op), Some((_, set))) = (found.ops.first(), w.sets.first()) {
            if op.kind != EntKind::Point {
                let at = w.span;
                let m = format!("`{set}` is a set of points, and `{}` puts a point on it, not {}",
                    w.word, op.kind.a());
                say_once(&mut cx.diags, at, st.id, m);
                return None;
            }
        }
    }
    let touching = r.word.as_ref().filter(|w| w.word == "tangent" && w.sets.len() == 1);
    if let (Some(w), Some(found)) = (touching, word) {
        let set = &w.sets[0].1;
        let wrong = match (found.ops.first().map(|e| e.kind), w.args.is_empty()) {
            (Some(EntKind::Plane), true) => Some(format!(
                "a plane touches a set at a point, which the word names: `P tangent(at: m) {set}`"
            )),
            (Some(kind), false) if kind != EntKind::Plane => Some(format!(
                "a set is touched at a point by a plane, `P tangent({}) {set}`, not by {}",
                w.args,
                kind.a()
            )),
            _ => None,
        };
        if let Some(m) = wrong {
            say_once(&mut cx.diags, w.span, st.id, m);
            return None;
        }
    }
    Some(touching.is_some_and(|w| !w.args.is_empty()))
}

/// A gauge applied (`apply_gauge`) — but a claim on one refused, as any unclaimable kind is.
fn gauge(cx: &mut Elab<'_>, r: &ResolvedRelation<'_>, st: &Stmt) {
    if r.claim {
        cx.diags.push(Diag::on(
            Code::E040,
            st,
            format!(
                "`{}` is judged by nothing a claim can weigh: it holds a number or picks a \
                 root, and adds no row for the diagnosis to rank",
                crate::syntax::snake(r.kind.name())
            ),
        ));
        return;
    }
    apply_gauge(cx, r, st);
}

/// Every slot of the kind's spec converted (`to_arg`), with which the source left out — or the
/// first refused, said at its argument.
fn arguments(
    cx: &mut Elab<'_>, r: &ResolvedRelation<'_>, st: &Stmt,
) -> Option<(Vec<CArg>, Vec<bool>)> {
    let (sk, res) = (&cx.sk, &cx.res);
    let ckind = r.kind;
    let spec = ckind.spec();
    let mut args: Vec<CArg> = Vec::with_capacity(spec.len());
    let mut left_out = vec![false; spec.len()];
    for (i, (name, kind)) in spec.iter().enumerate() {
        let given = r.args.get(i).and_then(|a| a.as_ref());
        let Some(a) = given else {
            left_out[i] = true;
            args.push(ckind.default_arg(i));
            continue;
        };
        // a selector's value has no span of its own, so a complaint about one is shown at the key
        let where_ = |a: &Arg| {
            arg_span(a).or_else(|| r.written.and_then(|w| w.key_span(name))).unwrap_or(st.span)
        };
        // an ordinate's second operand may be the plane it is measured from (`p distance(d,
        // along: u) P`), which `ordinate_operands` reads as that plane's origin below
        let kind = match (ckind.word_slot(), i, a) {
            (Some(_), 1, Arg::Ref(r))
                if res.entity(sk, r)
                    .is_some_and(|e| e.kind == EntKind::Plane) => &SpecKind::Plane,
            _ => kind,
        };
        match to_arg(sk, res, *kind, a) {
            Ok(v) => {
                // the word is one of the kind's own, or it is a typo: unchecked, anything that
                // was not `start` silently meant `end` (issue #48, item 4)
                if let (Some(words), CArg::Str(w)) = (ckind.words(i), &v) {
                    // the empty word is the slot's own default and says nothing — a selector
                    // nobody wrote (issue #48, item 4), which the printers leave out again
                    if !w.is_empty() && !words.contains(&w.as_str()) {
                        cx.diags.push(Diag::within(
                            Code::E040,
                            where_(a),
                            st.id,
                            format!(
                                "`{name}` is {}, not `{w}`",
                                crate::syntax::one_of(words)
                            ),
                        ));
                        return None;
                    }
                }
                args.push(v)
            }
            // a point a tangent is at that names nothing may be an end's word misspelt (#251):
            // `at: banana` once silently meant `end`, and is said as both it could have been
            Err((Code::E101, _)) if *name == "at" && ckind.is_tangency() && kind.takes_ref() => {
                let what = match a {
                    Arg::Ref(r) => crate::syntax::ref_text(r),
                    other => crate::syntax::sel_text(other),
                };
                let arc = |x: &Arg| {
                    matches!(x, Arg::Ref(o) if res.entity(sk, o).is_some_and(|e| e.kind == EntKind::Arc))
                };
                let ends = match r.args.first().and_then(|a| a.as_ref()).is_some_and(arc) {
                    true => format!("{}, or ", crate::syntax::one_of(&["start", "end"])),
                    false => String::new(),
                };
                let message =
                    format!("`at` is {ends}a point on what is touched, and `{what}` names nothing");
                cx.diags.push(Diag::within(Code::E040, where_(a), st.id, message));
                return None;
            }
            // a direction that names nothing is a word misspelt, more likely than an entity
            Err((Code::E101, _)) if ckind.word_slot().is_some() && i == 2 => {
                let what = match a {
                    Arg::Ref(r) => crate::syntax::ref_text(r),
                    other => crate::syntax::sel_text(other),
                };
                let message = no_direction(&what);
                // a retired word is said at the key, as every word the slot will not take is
                // (#268); `level(x)` has none, and is said at the word
                let at = match crate::constraints::retired_word(&what) {
                    Some(_) => r.written.and_then(|w| w.key_span("along")),
                    None => None,
                };
                let at = at.unwrap_or_else(|| where_(a));
                cx.diags.push(Diag::within(Code::E040, at, st.id, message));
                return None;
            }
            Err((code, msg)) => {
                cx.diags.push(Diag::within(
                    code,
                    where_(a),
                    st.id,
                    format!("{}: {msg}", name),
                ));
                return None;
            }
        }
    }
    Some((args, left_out))
}

/// **Across views, a word means the relation in space**: the planes its operands' points are
/// drawn in (`reading`) decide it, and the statement is the kind in space from here on — or
/// refused, where the word has no meaning there or a selector says nothing there. A point drawn
/// in several planes is read in the one its reader is drawn in first (§6.7) — but a projection
/// relates two planes by definition, each operand read in its own.
#[allow(clippy::type_complexity)]
fn across_views(
    cx: &mut Elab<'_>,
    r: &ResolvedRelation<'_>,
    st: &Stmt,
    (ckind, spec): (CKind, &'static [(&'static str, SpecKind)]),
    mut args: Vec<CArg>,
    left_out: Vec<bool>,
    reader: super::reading::Reader,
) -> Option<(CKind, &'static [(&'static str, SpecKind)], Vec<CArg>, Vec<bool>)> {
    if ckind != CKind::Project {
        super::reading::read_twins(&cx.sk, &mut args);
    }
    match super::reading::in_space(&cx.sk, ckind, &args, reader) {
        Ok(None) => Some((ckind, spec, args, left_out)),
        Ok(Some((k, a, l))) => Some((k, k.spec(), a, l)),
        Err((selector, message)) => {
            let key = ["side", "sense"].into_iter().find_map(|k| r.written.and_then(|w| w.key_span(k)));
            cx.diags.push(Diag::within(
                if selector { Code::E040 } else { Code::E062 },
                if selector { key.unwrap_or(st.span) } else { st.span },
                st.id,
                message,
            ));
            None
        }
    }
}

/// The numbers a kind refuses by value, each said where written: whether one was.
fn refused_value(
    cx: &mut Elab<'_>,
    r: &ResolvedRelation<'_>,
    st: &Stmt,
    ckind: CKind,
    spec: &[(&'static str, SpecKind)],
    args: &[CArg],
) -> bool {
    let sk = &cx.sk;
    // a magnitude stated negative: the kernel would square the sign away and the drawing show
    // the positive, so the document and the drawing would disagree about what the thing is
    // **A number that says which way is a word** (§9.2, issue #48 item 4).  Where the sign was a
    // *convention about a side* — a distance measured from a line, which the kernel cannot tell
    // one side of from the other — it is now `side: left`, and the number is a magnitude whose
    // negative is refused below, whatever it was arrived at: a component handed `v: -hw` is
    // caught at the call rather than quietly placed on the other side.  Where the sign is
    // *arithmetic* — the run and the rise, measured from the first point to the second, and the
    // directed angle — the word (`along: left`, `sense: cw`) is the spelling a drawing should
    // use, and the minus stays legal, because there a component computes it: `dy` is a
    // coordinate and `alphaL` is a bank leaning the other way, and by the time a statement is
    // settled the flattener has folded both into a number that no longer says how it was
    // written.
    if ckind.magnitude() {
        if let Some(i) = spec.iter().position(|(_, k)| *k == SpecKind::Length) {
            let v = written_number(&args[i], sk).unwrap_or(args[i].num());
            if v < 0.0 {
                // where the type has a side to name, the minus was *saying* which side, and the
                // word is where that belongs now (issue #48, item 4) — so the message names it
                // rather than leaving a reader to guess what a positive would have meant
                let fix = match ckind.side_words() {
                    // the word that means what the minus meant — the one the table gives −1 — and
                    // the key it is written under, which is `side` of a line and `along` the page
                    Some((slot, table)) => format!(
                        ", and which way is a word: write `{}({}, {}: {})`",
                        ckind.operator().map(|(w, _)| w).unwrap_or("distance"),
                        crate::syntax::num(-v),
                        spec[slot].0,
                        table.iter().find(|(_, s)| *s < 0.0).map(|(n, _)| *n).unwrap_or("")
                    ),
                    None => String::new(),
                };
                cx.diags.push(Diag::within(
                    Code::E040,
                    r
                        .args
                        .get(i)
                        .and_then(|a| a.as_ref())
                        .and_then(arg_span)
                        .unwrap_or(st.span),
                    st.id,
                    format!(
                        "{} is a magnitude and cannot be negative{fix}",
                        crate::model::article(&crate::syntax::snake(ckind.name()))
                    ),
                ));
                return true;
            }
        }
    }
    // **an angle in space at 0 or half a turn** is parallel said by a cosine, which does not move
    // there: the row is a double root, and the regular statement is `parallel`, its sense the
    // seed's.  Refused by value as a negative magnitude is — a stated number; one an unknown
    // moves is read where it is, never here
    if ckind == CKind::Angle3 {
        if let Some(i) = spec.iter().position(|(_, k)| *k == SpecKind::Angle) {
            let cos = match &args[i] {
                // the text as written, in the document's degrees
                CArg::Expr(_) => written_number(&args[i], sk).map(|deg| deg.to_radians().dcos()),
                // a number already in the kernels' radians
                a => Some(a.num().dcos()),
            };
            if cos.is_some_and(|c| c.abs() > 1.0 - 1e-12) {
                cx.diags.push(Diag::within(
                    Code::E040,
                    r.args.get(i).and_then(|a| a.as_ref()).and_then(arg_span).unwrap_or(st.span),
                    st.id,
                    "an angle in space of 0 or 180 degrees says the two are parallel, and \
                     is read by a cosine that does not move there: write `parallel`, \
                     and the seed says which way"
                        .to_string(),
                ));
                return true;
            }
        }
    }
    // `distance` between two circles is the radial gap between *concentric* ones — a kernel
    // that reads two radii and neither centre (`AnnularDistance`).  Written over two circles
    // centred apart, it says nothing about the gap a person meant and then duplicates the two
    // radii it does read (#43.21), so it is refused with the reading it has.
    if ckind == CKind::AnnularDistance {
        let centre = |e: EntRef| sk.children(e).first().copied();
        if centre(args[0].ent()) != centre(args[1].ent()) {
            cx.diags.push(Diag::on(
                Code::E040,
                st,
                "`distance` between two circles is the radial gap between concentric \
                 ones, and these are centred on different points — dimension the \
                 centres, or make the circles concentric"
                    .to_string(),
            ));
            return true;
        }
    }
    // a claim is judged, never solved for, so it may own no unknown — `CKind::claimable` is the
    // rule, shared with the document readers; elaboration's job is only to give it a span
    if r.claim && !ckind.claimable() {
        cx.diags.push(Diag::on(
            Code::E040,
            st,
            format!(
                "`{}` carries an unknown of its own, and a claim may add none",
                crate::syntax::snake(ckind.name())
            ),
        ));
        return true;
    }
    false
}

/// The inferred slots the source left out — read off the geometry, the one place that rule
/// lives, shared with the document reader and the bindings' constraint records — and what the
/// model refuses once they are in, in its own words, given this statement's span; then the bound
/// its number is (`bound_of`).  `None` where refused.
#[allow(clippy::type_complexity)]
fn inferred(
    cx: &mut Elab<'_>,
    r: &ResolvedRelation<'_>,
    st: &Stmt,
    ckind: CKind,
    args: &mut [CArg],
    left_out: &[bool],
) -> Option<Option<(crate::constraints::Bound, Option<Span>)>> {
    let (sk, map) = (&cx.sk, &cx.map);
    let name = |e: EntRef| map.name_of(e).cloned().unwrap_or_else(|| io::entity_name(e));
    if let Err(message) = crate::constraints::on_the_arc(ckind, args, &name) {
        cx.diags.push(Diag::on(Code::E040, st, message));
        return None;
    }
    if let Err(message) = io::seed_omitted_named(sk, ckind, args, |i| left_out[i], &name) {
        cx.diags.push(Diag::on(Code::E061, st, message));
        return None;
    }
    match bound_of(sk, &cx.res, r, ckind) {
        Ok(b) => Some(b),
        Err((span, message)) => {
            let span = span.unwrap_or(st.span);
            cx.diags.push(Diag::within(Code::E040, span, st.id, message));
            None
        }
    }
}

/// A set's body row stated as its derivative at a tangency's point (§6.21): one derivative per
/// use, the geometry the use made, under its prefix, moving with it. `None` where the row is
/// stated not at all — it reads nothing the use moves, or is refused, said once at `used`.
fn derivative(
    cx: &mut Elab<'_>,
    st: &Stmt,
    c: &mut Constraint,
    found: (Option<EntRef>, Option<Result<Toward, EntKind>>, &crate::syntax::Along),
    used: Span,
) -> Option<()> {
    let diags = &mut cx.diags;
    let mut say = |m: String| say_once(diags, used, st.id, m);
    let (Some(p), Some(toward), a) = found else { return None };
    let toward = match toward {
        Ok(t) => t,
        Err(kind) => {
            say(format!("a set is touched by a line, not {}", kind.a()));
            return None;
        }
    };
    let (sk, map) = (&mut cx.sk, &cx.map);
    let d = *cx.duals.entry(a.key.clone()).or_insert_with(|| {
        let made = a.made.iter().flat_map(|m| map.ents_under(m));
        sk.add_dual(p.i(), toward, made.filter(|&e| e != p).collect())
    });
    match c.differentiable(sk, d) {
        Ok(true) => c.along = Some(d),
        // a row that reads nothing the use moves holds still as it moves: no derivative
        Ok(false) => return None,
        Err(m) => {
            say(m);
            return None;
        }
    }
    Some(())
}

/// The constraint added, its id returned — but a bound a set put on a region solid's probe is a
/// term of the solid, never a row (§6.21), and an offset a set put on its number (F4) one too.
fn region_or_row(
    cx: &mut Elab<'_>, r: &ResolvedRelation<'_>, st: &Stmt, c: Constraint, term_name: String,
) -> Option<u32> {
    let grow = match r.written.and_then(|w| w.grow()) {
        None => None,
        Some((text, span)) => {
            let arg = crate::syntax::Arg::Dim { text: text.clone(), span: *span };
            let read = to_arg(&cx.sk, &cx.res, SpecKind::Length, &arg).ok();
            match read.as_ref().and_then(|a| term_number(a, &cx.sk, SpecKind::Length)) {
                Some(g) => Some(g),
                None => {
                    let message = "an offset is stated or reads one unknown of the drawing";
                    cx.diags.push(Diag::on(Code::E040, st, message.into()));
                    return None;
                }
            }
        }
    };
    // what a set's body states at its envelope's hidden point is the surface the envelope reads
    // (#283), never a row
    if let Some(e) = super::envelopes::probe_envelope(&cx.sk, &c) {
        let read = match grow {
            Some(_) => Err("an envelope's set is a surface, not one grown from another".into()),
            None => super::envelopes::set_shape(&mut cx.sk, &c, e),
        };
        if let Err(message) = read {
            cx.diags.push(Diag::on(Code::E040, st, message));
        }
        return None;
    }
    if let Some(probe) = probe_of(&cx.sk, &c, &cx.probes) {
        match region_term(&cx.sk, &c, probe, grow, term_name) {
            Ok(t) => cx.probes.terms.get_mut(&probe).expect("a probe's terms").push(t),
            Err(message) => {
                cx.diags.push(Diag::on(Code::E040, st, message))
            }
        }
        return None;
    }
    if grow.is_some() {
        let message = "an offset of a cone is a region solid's: a point is put at a distance from \
                       one by its foot on it"
            .to_string();
        cx.diags.push(Diag::on(Code::E040, st, message));
        return None;
    }
    Some(cx.sk.add_quiet(c))
}

/// **An entity given where a kind is asked for is of that kind**, refused at the argument once
/// the elaborator knows what the name it was given is: a `path` formal's (#251), a line, an arc,
/// a circle, a spline or a curve, what a point runs along; an interface's entity member's
/// (§8.1 [0.84]), the kind it declares.
pub(super) fn kinds(map: &SourceMap, given: &[crate::flatten::KindAsked], diags: &mut Vec<Diag>) {
    for k in given {
        let Some(e) = map.ent_named(&k.target) else { continue };
        let fits = match k.ty {
            crate::syntax::Ty::Ent(want) => e.kind == want,
            _ => e.kind.runs_along(),
        };
        if fits {
            continue;
        }
        let message = match (&k.interface, k.ty) {
            (None, _) => format!(
                "`{}` is a path — a line, an arc, a circle, a spline or a curve — not {}",
                k.what, e.kind.a()
            ),
            (Some(i), crate::syntax::Ty::Ent(want)) => {
                format!("`{}` is {}, where `{i}` asks for {}", k.what, e.kind.a(), want.a())
            }
            (Some(i), _) => format!("`{}` is {}, where `{i}` asks for a path", k.what, e.kind.a()),
        };
        diags.push(Diag::new(Code::E103, k.span, None, message));
    }
}

/// **A tangent at a point reads the place of the contact holding the point** (#251): `g
/// tangent(at: m) l` over a spline or a curve says which way the line runs where `m` is on `g`,
/// and that is a contact's to say. One with none is refused, naming the statement it wants.
pub(super) fn tangents_at(sk: &Sketch, map: &SourceMap, diags: &mut Vec<Diag>) {
    for c in &sk.constraints {
        let Some((slots, at, on)) = c.tangent_at() else { continue };
        let place: Vec<Option<u32>> = slots.iter().map(|&t| match c.args[t] {
            CArg::Param(p) => Some(p),
            _ => None,
        }).collect();
        let held = sk.holding(at, on).is_some_and(|h| h.into_iter().map(Some).eq(place));
        let Some(site) = map.site_of_constraint(c.id).filter(|_| !held) else { continue };
        let name = |e: EntRef| {
            let written = |n: &String| format!("`{}`", super::source_map::public_path(n));
            map.name_of(e).map_or_else(|| e.kind.a(), written)
        };
        let (m, g) = (name(EntRef::point(at)), name(on));
        // what is taken there: an envelope's normal or its section's curvature, else the tangent
        let what = match c.kind {
            CKind::EnvelopeNormalAt => "normal",
            CKind::EnvelopeCurvatureAt => "curvature",
            CKind::EnvelopePlaneAt => "tangent plane",
            _ => "tangent",
        };
        diags.push(Diag::within(
            Code::E040,
            site.span,
            site.stmt,
            format!(
                "the {what} at {m} is taken where a contact holds {m} on {g}, and none does: \
                 state `{} coincident {}`",
                m.trim_matches('`'),
                g.trim_matches('`')
            ),
        ));
    }
}

/// **What a written relation is, by what its operands resolve to** (#188): the one reading
/// `constrain` and the claims about solids ask, so neither detects the shapes again.
pub(in crate::program) enum Shape {
    /// A point against a solid's faces (#183 F1): `p inside F`, `p outside F`, `p distance(…)
    /// F`, `p coincident F.surface`.
    Face { point: EntRef, face: crate::model::FaceRef },
    /// A point `inside` (or `outside`) where the edge a set is ends (#183 F7): `p inside S.ends`.
    Ends { point: EntRef, edge: EntRef },
    /// A point `inside` or `outside` a plane (§9.6): a bound.
    PlaneSide,
    Other,
}

/// What `w` is (`Shape`): a point first, then a solid's faces, where an edge ends or a plane.
pub(in crate::program) fn shape(sk: &Sketch, res: &Resolver, w: &crate::syntax::Written) -> Shape {
    let [p, f] = w.ops.as_slice() else { return Shape::Other };
    let Some(point) = res.entity(sk, p).filter(|e| e.kind == EntKind::Point) else {
        return Shape::Other;
    };
    let Ok(of) = res.operand(sk, f) else { return Shape::Other };
    // a side as written or as a set's body carries it (`Written::side`, #192)
    let side = w.side().is_some();
    match of {
        Operand::Ends(edge) if side => Shape::Ends { point, edge },
        Operand::Ent(e) if side && e.kind == EntKind::Plane => Shape::PlaneSide,
        _ if matches!(w.word.text.as_str(), "inside" | "outside" | "distance" | "coincident") => {
            of.faces().map_or(Shape::Other, |face| Shape::Face { point, face })
        }
        _ => Shape::Other,
    }
}

/// `p inside P` / `p outside P` between a point and a plane (`Shape::PlaneSide`), as what it says
/// (§9.6): the point's ordinate along `P`'s normal from `P.origin`, at most zero (behind the
/// normal) or at least zero.
fn side_of_plane(r: &Relation, w: &crate::syntax::Written) -> Relation {
    use crate::syntax::{Arg as A, Name, OpArg, RelationForm, Written};
    // as written or as a set's body carries it (`Written::side`, #192); grown by a distance
    // (§6.21, F4): `side: outside` at most `d` before it, `side: inside` at most `d` behind it
    let (cmp, level) = w.side().expect("a side");
    let at = w.word.span;
    let name = |t: &str| Name { text: t.to_string(), span: at };
    let level = level.map_or_else(|| "0".to_string(), |(d, _)| d);
    let args = vec![
        OpArg::Bound { cmp, span: at, hi: None, grow: None },
        OpArg::Dim(level, at),
        OpArg::Named(name("along"), A::Word("n".to_string())),
    ];
    let written = Written { word: name("distance"), args, ..w.clone() };
    Relation { form: RelationForm::Written(written), ..r.clone() }
}

/// The bound a relation's number is, where it is one (§9.6), and where it is written, checked: a
/// kind whose reading a solve can be steered along (`CKind::boundable`), not a claim, its
/// interval's high end a number.  The low end is the constraint's own number, read as any
/// dimension is.
fn bound_of(
    sk: &Sketch,
    res: &Resolver,
    r: &ResolvedRelation<'_>,
    ckind: CKind,
) -> Result<Option<(crate::constraints::Bound, Option<Span>)>, (Option<Span>, String)> {
    let Some((cmp, hi)) = &r.bound else { return Ok(None) };
    let cmp = *cmp;
    let at = r.written.and_then(|w| w.args.iter().find_map(|a| match a {
        crate::syntax::OpArg::Bound { span, .. } => Some(*span),
        _ => None,
    }));
    match ckind {
        // an angle written in a view is signed: bounded, it would be a half-plane, not a cone's
        // wedge, which a set's body says (read in space, §6.21)
        CKind::Angle => {
            return Err((at, "an angle in a view is signed, and bounded would be a half-plane: a \
                             region within an angle of a line is a cone's, `std.Cone`, whose \
                             angle is read in space, unsigned"
                .to_string()))
        }
        k if !k.boundable() => {
            return Err((at, format!(
                "a bound is read off a distance or an ordinate, and `{}` states a number it holds",
                crate::syntax::snake(k.name())
            )))
        }
        _ => {}
    }
    if r.claim {
        let m = "a bound chooses which solution is meant, and a claim asks about the one there \
                 is: write one or the other";
        return Err((at, m.to_string()));
    }
    let hi = match hi {
        None => None,
        // in the number's own kind: an angle's in radians
        Some(a) => {
            let not = || (arg_span(a), format!("`{}` is not a number", crate::syntax::sel_text(a)));
            let v = to_arg(sk, res, ckind.dimension_kind(), a).ok();
            Some(v.and_then(|v| settled_number(&v, sk)).ok_or_else(not)?)
        }
    };
    Ok(Some((crate::constraints::Bound { cmp, hi }, at)))
}

/// **An ordinate's operands, read** (`docs/ordinate-plan.md`): a plane's own direction is
/// measured from its origin, so `q distance(d, along: u) P` is the ordinate from `P.origin` to
/// `q` along `P.u` (`P`'s normal for `n`) and the arguments are rewritten so — between two points
/// `u` and `v` are the view's they are both drawn in, the plane at hand (#268); a direction written
/// as a reference is an axis or a line, never a plane; a word of the table that is also the name
/// of a direction in scope is refused as the two readings it has; and a number written as zero is
/// a level, not a dimension.
fn ordinate_operands(
    sk: &Sketch,
    res: &Resolver,
    r: &ResolvedRelation<'_>,
    st: &Stmt,
    doc: &crate::syntax::Program,
    args: &mut [CArg],
    left_out: &mut [bool],
) -> Result<(), (Span, String)> {
    use crate::constraints::Toward;
    let ckind = r.kind;
    let level = ckind == CKind::Level;
    let at_key = r.written.and_then(|w| w.key_span("along")).unwrap_or(st.span);
    let word = crate::constraints::ordinate_word(ckind, args).to_string();
    let toward = Toward::of(&word);
    let spell = |d: &str| match level {
        true => format!("level({d})"),
        false => format!("distance(…, along: {d})"),
    };
    // the word read as a word, though the scope has a direction by that name
    if toward.is_some() {
        let named = res.lookup(&Ref::new(word.as_str())).is_some_and(|e| {
            matches!(e.kind, EntKind::Axis | EntKind::Line)
        });
        if named {
            return Err((at_key, format!(
                "`{word}` is a direction word and also the name of a direction here: `{}` reads \
                 it as the word, so name the direction otherwise",
                spell(&word)
            )));
        }
    }
    // a direction written as a reference is one a line runs along; a plane's is said in its word
    if let CArg::Ent(t) = &args[2] {
        if t.kind == EntKind::Plane {
            return Err((at_key, format!(
                "a direction along a plane reads as one within it: its normal is `n`, against \
                 the plane — `p {} P`",
                spell("n")
            )));
        }
    }
    let plane = match &args[1] {
        CArg::Ent(e) if e.kind == EntKind::Plane => Some(e.i()),
        _ => None,
    };
    match (plane, toward) {
        (Some(pl), Some(t)) if t.of_plane() => {
            if level && t == Toward::PlaneN {
                return Err((st.span, "a point level with a plane's origin along its normal is on \
                    the plane: `p coincident P`".to_string()));
            }
            let p = &sk.planes[pl];
            let along = match t {
                Toward::PlaneU => EntRef::axis(p.u as usize),
                Toward::PlaneV => EntRef::axis(p.v as usize),
                _ => EntRef::plane(pl),
            };
            args[1] = args[0].clone();
            args[0] = CArg::Ent(EntRef::point(p.origin as usize));
            args[2] = CArg::Ent(along);
            left_out[2] = false;
        }
        (Some(_), _) => {
            return Err((at_key, format!(
                "against a plane the direction is one of its own: `{}`, `{}` or `{}`",
                spell("u"),
                spell("v"),
                spell("n")
            )));
        }
        // `u` and `v` between two points are the view's they are drawn in (`Toward::in_view`);
        // a normal is a plane's alone
        (None, Some(t)) if t.in_view().is_none() => {
            return Err((at_key, format!(
                "`{word}` is a plane's own direction, measured from its origin: the plane is the \
                 second operand, `p {} P`",
                spell(&word)
            )));
        }
        _ => {}
    }
    // zero, written so: the two are level, which is a relation and draws no number — but for
    // a bound, whose edge zero may be (§9.6)
    if !level && r.bound.is_none() {
        if let Some(Arg::Dim { span, .. }) = r.args.get(3).and_then(|a| a.as_ref()) {
            let text = doc.span_text(*span).unwrap_or_default();
            if written_zero(text, sk.units) {
                let say = level_spelling(r.written, &word);
                return Err((*span, format!(
                    "an ordinate of zero says the two are level, which is a relation and states no \
                     number: `{say}`"
                )));
            }
        }
    }
    Ok(())
}

/// What an ordinate of zero says, in the words its statement wrote: `a level(v) b` for the
/// view's own axis, `p level(u) P` for a plane's own word, `a level(t) b` for a reference.
fn level_spelling(w: Option<&crate::syntax::Written>, word: &str) -> String {
    use crate::constraints::Toward;
    let ops: Vec<String> =
        w.map(|w| w.ops.iter().map(crate::syntax::ref_text).collect()).unwrap_or_default();
    let op = |i: usize, or: &str| ops.get(i).cloned().unwrap_or_else(|| or.to_string());
    let (a, b) = (op(0, "a"), op(1, "b"));
    let reference = w.and_then(|w| w.args.iter().find_map(|a| match a {
        crate::syntax::OpArg::Named(n, Arg::Ref(t)) if n.text == "along" => {
            Some(crate::syntax::ref_text(t))
        }
        _ => None,
    }));
    match Toward::of(word) {
        Some(Toward::PlaneN) => format!("{a} coincident {b}"),
        Some(_) => format!("{a} level({word}) {b}"),
        None => format!("{a} level({}) {b}", reference.unwrap_or_else(|| "t".to_string())),
    }
}

/// Whether a dimension's text is a zero written as one: `0`, `0mm`, `-0.0` — not a name or an
/// expression that comes to it.
fn written_zero(text: &str, units: crate::units::Units) -> bool {
    let text = text.trim();
    expr::is_literal(text)
        && expr::parse_in(text, units).ok()
            .and_then(|p| expr::eval(&p.body, &Default::default()).ok())
            .and_then(|a| a.number())
            .is_some_and(|v| v == 0.0)
}

/// What `along:` is, refusing what it is not.
fn no_direction(what: &str) -> String {
    // a retired word is said as what it became: `x` is the view's `u`, `y` its `v` (#268)
    if let Some(now) = crate::constraints::retired_word(what) {
        return format!(
            "`{what}` is no longer a direction word: a view's own directions are `u` and `v`, as \
             a plane's are, and `right`, `left`, `up` and `down` say the sign — write `{now}`"
        );
    }
    let words = crate::syntax::one_of(&crate::constraints::ALONG_WORDS);
    format!("`along` is a direction — {words}, an axis or a line — not `{what}`")
}

/// A dimension as it was written, when that differs from the text it reached here as — see
/// `Constraint::written`.  The only thing the flattener writes into a dimension's text is a number
/// a name stood for (a `param`, a formal, a module's or a group's member), so a difference is
/// exactly a name worked out; a named dimension keeps its name already.  Read off the source at
/// the argument's span, so only where that span is the document's own text: a statement of the
/// root, or of a component body written in this file — a module's span indexes a text this is
/// not.  An instance's body draws its formula over the formals, true of every instance, so
/// `design.module` reads as itself and not as the 2 one caller passed; no module path is taken
/// off it, since a closed body names no module's number and `design` there is a formal.  A copy
/// of a block keeps its numbers: its copies share a label, and `repeated` would draw one of them.
fn written(
    args: &[Option<Arg>],
    spec: &[(&str, SpecKind)],
    st: &Stmt,
    doc: &crate::syntax::Program,
) -> Option<String> {
    if st.path.iter().any(|s| matches!(s, PathStep::Copy { .. })) {
        return None;
    }
    let (text, span) = spec.iter().zip(args).find_map(|((_, k), a)| match a {
        Some(Arg::Dim { text, span }) if k.is_dimension() => Some((text, *span)),
        _ => None,
    })?;
    if !doc.owns(span) {
        return None;
    }
    let was = doc.text().get(span.lo as usize..span.hi as usize)?.trim();
    if was.is_empty() || was == text.trim() {
        return None;
    }
    Some(if st.path.is_empty() { unqualified(was, doc) } else { was.to_string() })
}

/// A dimension's text as a drawing shows it: a used module's path taken off the names it reads,
/// so `engine.dims.D` is drawn `D` — the path says where a number comes from, which is the
/// source's business and not the sheet's.
fn unqualified(text: &str, doc: &crate::syntax::Program) -> String {
    let mut paths: Vec<&str> = doc.uses.iter().map(|u| u.name.as_str()).collect();
    paths.sort_by_key(|p| std::cmp::Reverse(p.len()));
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    'scan: while !rest.is_empty() {
        let starts_word =
            out.chars().last().is_none_or(|c| !(c.is_alphanumeric() || c == '_' || c == '.'));
        if starts_word {
            for p in &paths {
                if let Some(after) = rest.strip_prefix(*p).and_then(|r| r.strip_prefix('.')) {
                    rest = after;
                    continue 'scan;
                }
            }
        }
        let ch = rest.chars().next().unwrap_or_default();
        out.push(ch);
        rest = &rest[ch.len_utf8()..];
    }
    out
}

/// Mark a dimension another copy of the same block already states — see
/// `Constraint::repeated`.  The key is the statement, the path to it with every copy index
/// erased (so two instances of a component are two arrays, and one block's copies are one), and
/// the label it draws: copies stating *different* numbers (`distance(i * 10)`) are each a
/// dimension of their own.  Statement order is copy order, so the first copy is the one drawn.
pub(super) fn repeated(
    sk: &mut Sketch,
    id: u32,
    st: &Stmt,
    seen: &mut BTreeSet<(StmtId, Vec<PathStep>, String)>,
) {
    if !st.path.iter().any(|s| matches!(s, PathStep::Copy { .. })) {
        return;
    }
    let Some(c) = sk.constraint(id) else { return };
    let Some(label) = io::dimension_text(c) else { return };
    let path = st.path.iter().map(|s| match s {
        PathStep::Copy { block, .. } => PathStep::Copy { block: *block, index: 0 },
        s => s.clone(),
    });
    if !seen.insert((st.id, path.collect(), label)) {
        if let Some(c) = sk.constraint_mut(id) {
            c.repeated = true;
        }
    }
}

pub(super) fn arg_span(a: &Arg) -> Option<Span> {
    match a {
        Arg::Ref(r) => Some(r.span),
        Arg::Dim { span, .. } => Some(*span),
        _ => None,
    }
}

/// The written forms of a plain value argument — an int, a flag, a word, a float.  One table,
/// read by `to_arg` and by `compile_trace`, so an integer in a `Float` slot means the same thing
/// in a component body and in a trace block.
pub(super) fn scalar_arg(kind: SpecKind, a: &Arg) -> Option<CArg> {
    Some(match (kind, a) {
        (SpecKind::Int, Arg::Int(v)) => CArg::Int(*v),
        (SpecKind::Int, Arg::Num(v)) => CArg::Int(*v as i64),
        (SpecKind::Bool, Arg::Bool(b)) => CArg::Bool(*b),
        (SpecKind::Str, Arg::Word(w)) => CArg::Str(w.clone()),
        (SpecKind::Float, Arg::Num(v)) => CArg::Num(*v),
        (SpecKind::Float, Arg::Int(v)) => CArg::Num(*v as f64),
        _ => return None,
    })
}

/// An entity argument, resolved: follow the reference's path and check the kind — one statement
/// of the rule, shared by `to_arg` and `compile_trace`, so the two readers of a spec cannot
/// drift on what an entity slot accepts or how it says no.
pub(super) fn ent_arg(
    sk: &Sketch,
    found: Option<EntRef>,
    kind: SpecKind,
    r: &Ref,
) -> Result<CArg, (Code, String)> {
    let e = found.ok_or_else(|| (Code::E101, format!("no such entity: `{}`", r.root.text)))?;
    let e = follow(sk, e, &r.path).map_err(|m| (Code::E040, m))?;
    if !crate::constraints::kind_matches(kind, e.kind) {
        return Err((
            Code::E040,
            format!(
                "`{}` is {}, and {} is wanted here",
                r.root.text,
                e.kind.a(),
                kind.a()
            ),
        ));
    }
    Ok(CArg::Ent(e))
}

pub(super) fn to_arg(sk: &Sketch, res: &Resolver, kind: SpecKind, a: &Arg) -> Result<CArg, (Code, String)> {
    if let Some(v) = scalar_arg(kind, a) {
        return Ok(v);
    }
    Ok(match (kind, a) {
        (k, Arg::Ref(r)) if k.is_entity() => ent_arg(sk, res.lookup(r), k, r)?,
        // a dimension: the text as written, handed to `expr.rs`, which owns that little language
        (k, Arg::Dim { text, .. }) if k.is_dimension() => {
            if text.len() > expr::MAX_TEXT {
                return Err((Code::E040, format!("longer than {} characters", expr::MAX_TEXT)));
            }
            match expr::literal(text) {
                Some(n) => CArg::Num(expr::to_arg_units(k, n)),
                None => {
                    // in the *document's* units: `80mm` is a number here only where the document
                    // said what a number is, and saying so is `unit mm` (spec §3.3).  Valued now
                    // where it reads no unknown, so a reader before `expr::evaluate` (a bound's
                    // interval, a region solid's term) reads the number written
                    let parsed = expr::parse_in(text, sk.units)
                        .map_err(|e| (Code::E040, e.to_string()))?;
                    let value = expr::eval(&parsed.body, &Default::default()).ok()
                        .and_then(|a| a.number()).map_or(0.0, |v| expr::to_arg_units(k, v));
                    CArg::Expr(expr::Expr::new(text.trim().to_string(), value))
                }
            }
        }
        (k, Arg::Num(v)) if k.is_dimension() => CArg::Num(expr::to_arg_units(k, *v)),
        (SpecKind::Param, Arg::Seed { value, pinned }) => {
            CArg::Seed { value: *value, pinned: *pinned }
        }
        // expansion turns one of these into a `Seed`; one that reaches here was written outside
        // any component, where there are no parameters for it to be over
        (SpecKind::Param, Arg::SeedExpr { text, pinned, .. }) => CArg::Seed {
            value: expr::literal(text).ok_or_else(|| {
                (Code::E040, format!("`{text}` is not a number this contact can start at"))
            })?,
            pinned: *pinned,
        },
        (SpecKind::Param, Arg::Tie { name, seed, .. }) => {
            CArg::Shared { name: name.clone(), seed: *seed }
        }
        (k, other) => {
            return Err((Code::E040, format!("{} is wanted here, not {other:?}", k.a())))
        }
    })
}

impl Relation {
    pub(crate) fn resolve(
        &self,
        kind_of: &dyn Fn(&Ref) -> Option<EntKind>,
    ) -> Result<ResolvedRelation<'_>, (Span, String)> {
        let (kind, args) = match &self.form {
            RelationForm::Written(w) => settle(w, kind_of)?,
            RelationForm::Canonical { kind, args, .. } => (*kind, args.clone()),
        };
        Ok(ResolvedRelation {
            kind,
            args,
            written: self.form.written(),
            claim: self.claim,
            class: &self.class,
            bound: self.form.bound(),
        })
    }
}
