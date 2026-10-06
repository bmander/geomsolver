//! Resolve constraint arguments and apply gauges.

use super::resolve::{follow, Resolver};
use super::{Code, Diag, SourceMap};
use crate::constraints::{Arg as CArg, CKind, Constraint, SpecKind};
use crate::ir::{PathStep, Relation, ResolvedRelation, Statement as Stmt};
use crate::model::{EntKind, EntRef, Sketch};
use crate::syntax::{Arg, Ref, RelationForm, Span, StmtId};
use std::collections::BTreeSet;
use crate::fmath::Det;
use crate::{decompose, expr, io};

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
    // `along:` chooses the kind and fills no slot, so this is the only place its word can be
    // checked — and unchecked, `along: z` came back as "`distance` does not relate a point to a
    // point", a complaint about the operands for a mistake in the selector (issue #48, item 4)
    if let Some(v) = w.sel("along") {
        if !crate::constraints::ALONG.iter().any(|(n, _)| *n == v) {
            let words: Vec<&str> = crate::constraints::ALONG.iter().map(|(n, _)| *n).collect();
            let m = format!("`along` is {}, not `{v}`", crate::syntax::one_of(&words));
            return Err((w.key_span("along").unwrap_or(w.word.span), m));
        }
    }
    let kinds: Vec<Option<EntKind>> = w.ops.iter().map(kind_of).collect();
    let kind = match w.fixity {
        Fixity::Prefix => {
            let Some(a) = kinds.first().copied().flatten() else {
                let m = format!("`{word}` needs to know what `{}` is", w.ops[0].root.text);
                return Err((w.word.span, m));
            };
            crate::constraints::prefix_op(word, a).ok_or_else(|| {
                (w.word.span, format!("`{word}` does not apply to {}", a.a()))
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
            if symmetric && crate::constraints::infix_op(word, a, b, &|n| w.sel(n)).is_none() {
                if let Some(k) = crate::constraints::infix_op(word, b, a, &|n| w.sel(n)) {
                    let mut swapped = w.clone();
                    swapped.ops.swap(0, 1);
                    return Ok((k, swapped.assemble(k)?));
                }
            }
            crate::constraints::infix_op(word, a, b, &|n| w.sel(n)).ok_or_else(|| {
                let mut m = format!(
                    "`{word}` does not relate {} to {}",
                    a.a(),
                    b.a()
                );
                // a sphere touches a line or a sphere; a circle against one says two things
                if word == "tangent" && (a == EntKind::Sphere || b == EntKind::Sphere) {
                    m.push_str(": a sphere is tangent to a line or to another sphere, with the \
                                sphere written first");
                    if matches!(a, EntKind::Circle | EntKind::Arc)
                        || matches!(b, EntKind::Circle | EntKind::Arc)
                    {
                        m.push_str(". A circle and a sphere may touch at a point or all the \
                                    way round, so the word does not say which: a circle lying \
                                    on the sphere is `c coincident s`");
                    }
                }
                // a cone or a cylinder takes the words it has kernels for, and says so
                let axial = |k: EntKind| matches!(k, EntKind::Cone | EntKind::Cylinder);
                if axial(a) || axial(b) {
                    m.push_str(match word {
                        "coincident" if a == EntKind::Line => ": a line on a cone or a cylinder (a \
                            generator) is not a relation yet; say it of the line's points — its \
                            start at the apex and its end `coincident` the cone, or both ends \
                            `coincident` the cylinder and the line `parallel` to the axis",
                        "coincident" => ": a point is `coincident` a cone or a cylinder",
                        "tangent" => ": a line touches a cylinder (`c tangent l`, the cylinder \
                            first), and two cones touch at a point (`k1 tangent(M) k2`)",
                        _ => ": a cone takes `coincident`, `angle` and `tangent`, and a cylinder \
                            `coincident`, `radius` and `tangent`",
                    });
                }
                (w.word.span, m)
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
    // two cones touch at a named point: without one there is no place to read their normals
    if kind == CKind::ConeTangentCone
        && !w.args.iter().any(|a| matches!(a, crate::syntax::OpArg::Ent(_)))
    {
        return Err((w.word.span, "two cones touch at a point, and the statement names it: \
                                  `k1 tangent(M) k2`, with `M coincident k1` and `M coincident k2` beside it"
            .to_string()));
    }
    Ok((kind, w.assemble(kind)?))
}

/// `fix` states every number it holds, pinned whole or by member (§9.2: inside the parentheses
/// `==` pins and `:` selects): `fix((0, 0)) p`, `fix(x == 0) p`, `fix(dir == (1, 0, 0)) t`.  A
/// bare `fix p`, a held number without its name and one written as a selector are each refused
/// where they stand, with the spelling.
fn fix_spelling(w: &crate::syntax::Written) -> Result<(), (Span, String)> {
    use crate::syntax::OpArg;
    let of = w.ops.first().map_or("p".to_string(), |r| r.root.text.clone());
    let refused = |key: &str| format!("`fix` holds a point itself, `fix((0, 0)) {of}`, or what an \
        entity has — `x`, `r`, `half`, `dir`, `origin`, or one component, `dir.x` — not `{key}`");
    for a in &w.args {
        match a {
            OpArg::Named(key, _) => {
                let m = format!("`fix` pins a number with `==`: `fix({} == …) {of}`", key.text);
                return Err((key.span, m))
            }
            OpArg::Dim(text, span) => {
                return Err((*span, format!("`fix` names each number it holds: \
                    `fix(r == {text}) {of}`, and a point's place is a vector, `fix((0, 0)) {of}`")))
            }
            OpArg::Slot { key, .. } => {
                if !CKind::Fix.spec()[1..].iter().any(|(n, _)| *n == key.text) {
                    return Err((key.span, refused(&key.text)))
                }
            }
            OpArg::Vector { key: Some(key), .. } => {
                if !matches!(key.text.as_str(), "dir" | "origin") {
                    return Err((key.span, refused(&key.text)))
                }
            }
            _ => {}
        }
    }
    if !w.args.iter().any(|a| matches!(a, OpArg::Slot { .. } | OpArg::Vector { .. })) {
        return Err((w.word.span, format!("`fix` states the numbers it holds: `fix((0, 0)) {of}`")));
    }
    Ok(())
}

/// Whether a statement is a `fix`, which is applied before the seeds that read geometry are
/// worked out (`program::build`), so a seed reading a held point reads where it is held.
pub(super) fn is_fix(r: &Relation) -> bool {
    match &r.form {
        RelationForm::Written(w) => w.word.text == "fix",
        RelationForm::Canonical { kind, .. } => *kind == CKind::Fix,
    }
}

pub(super) fn constrain(
    sk: &mut Sketch,
    res: &Resolver,
    r: &Relation,
    st: &Stmt,
    doc: &crate::syntax::Program,
    map: &SourceMap,
    diags: &mut Vec<Diag>,
) -> Option<u32> {
    // **a word that relates two solids is a claim, judged and never solved** (§9.8).  Picked up
    // by the solids phase: nothing here has a kernel, and saying so twice would report one
    // statement twice.
    if r.form.written().is_some_and(|w| crate::constraints::solid_word(&w.word.text).is_some()) {
        return None;
    }
    let r = match r
        .resolve(&|r| res.lookup(r).and_then(|e| follow(sk, e, &r.path).ok()).map(|e| e.kind))
    {
        Ok(r) => r,
        Err((span, message)) => {
            diags.push(Diag { code: Code::E040, span, stmt: Some(st.id), message });
            return None;
        }
    };
    let ckind = r.kind;
    // a gauge is applied, not added: it holds parameters or records a root choice, and there
    // is no constraint for the map to know it by.  A claim on one is refused below the way any
    // unclaimable kind is, so it is checked first.
    if ckind.gauge() {
        if r.claim {
            diags.push(Diag {
                code: Code::E040,
                span: st.span,
                stmt: Some(st.id),
                message: format!(
                    "`{}` is judged by nothing a claim can weigh: it holds a number or picks a \
                     root, and adds no row for the diagnosis to rank",
                    crate::syntax::snake(ckind.name())
                ),
            });
            return None;
        }
        apply_gauge(sk, res, &r, st, diags);
        return None;
    }
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
        match to_arg(sk, res, *kind, a) {
            Ok(v) => {
                // the word is one of the kind's own, or it is a typo: unchecked, anything that
                // was not `start` silently meant `end` (issue #48, item 4)
                if let (Some(words), CArg::Str(w)) = (ckind.words(i), &v) {
                    // the empty word is the slot's own default and says nothing — a selector
                    // nobody wrote (issue #48, item 4), which the printers leave out again
                    if !w.is_empty() && !words.contains(&w.as_str()) {
                        diags.push(Diag {
                            code: Code::E040,
                            span: where_(a),
                            stmt: Some(st.id),
                            message: format!(
                                "`{name}` is {}, not `{w}`",
                                crate::syntax::one_of(words)
                            ),
                        });
                        return None;
                    }
                }
                args.push(v)
            }
            Err((code, msg)) => {
                diags.push(Diag {
                    code,
                    span: where_(a),
                    stmt: Some(st.id),
                    message: format!("{}: {msg}", name),
                });
                return None;
            }
        }
    }
    // **across views, a word means the relation in space**: the planes its operands' points are
    // drawn in (`reading`) decide it, and the statement is the kind in space from here on — or
    // refused, where the word has no meaning there or a selector says nothing there
    // a curve standing for a surface (a prism's side generating, `CurveE::extrusion`) is met in
    // space, by the point's place in the curve's view, whatever view the point is drawn in
    let ckind = match (ckind, args.get(1)) {
        (CKind::PointOnCurve, Some(CArg::Ent(e))) if sk.curves[e.i()].extrusion => CKind::PointOnExtrusion,
        _ => ckind,
    };
    let (ckind, spec, mut args, left_out) = match super::reading::in_space(sk, ckind, &args) {
        Ok(None) => (ckind, spec, args, left_out),
        Ok(Some((k, a, l))) => (k, k.spec(), a, l),
        Err((selector, message)) => {
            let key = ["side", "sense"].into_iter().find_map(|k| r.written.and_then(|w| w.key_span(k)));
            diags.push(Diag {
                code: if selector { Code::E040 } else { Code::E062 },
                span: if selector { key.unwrap_or(st.span) } else { st.span },
                stmt: Some(st.id),
                message,
            });
            return None;
        }
    };
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
                diags.push(Diag {
                    code: Code::E040,
                    span: r
                        .args
                        .get(i)
                        .and_then(|a| a.as_ref())
                        .and_then(arg_span)
                        .unwrap_or(st.span),
                    stmt: Some(st.id),
                    message: format!(
                        "{} is a magnitude and cannot be negative{fix}",
                        crate::model::article(&crate::syntax::snake(ckind.name()))
                    ),
                });
                return None;
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
                diags.push(Diag {
                    code: Code::E040,
                    span: r.args.get(i).and_then(|a| a.as_ref()).and_then(arg_span).unwrap_or(st.span),
                    stmt: Some(st.id),
                    message: "an angle in space of 0 or 180 degrees says the two are parallel, and \
                              is read by a cosine that does not move there: write `parallel`, \
                              and the seed says which way"
                        .to_string(),
                });
                return None;
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
            diags.push(Diag {
                code: Code::E040,
                span: st.span,
                stmt: Some(st.id),
                message: "`distance` between two circles is the radial gap between concentric \
                          ones, and these are centred on different points — dimension the \
                          centres, or make the circles concentric"
                    .to_string(),
            });
            return None;
        }
    }
    // a claim is judged, never solved for, so it may own no unknown — `CKind::claimable` is the
    // rule, shared with the document readers; elaboration's job is only to give it a span
    if r.claim && !ckind.claimable() {
        diags.push(Diag {
            code: Code::E040,
            span: st.span,
            stmt: Some(st.id),
            message: format!(
                "`{}` carries an unknown of its own, and a claim may add none",
                crate::syntax::snake(ckind.name())
            ),
        });
        return None;
    }
    // the inferred slots the source left out — read off the geometry, the one place that rule
    // lives, shared with the document reader and the bindings' constraint records — and what
    // the model refuses once they are in, in its own words, given this statement's span
    let name = |e: EntRef| map.name_of(e).cloned().unwrap_or_else(|| io::entity_name(e));
    if let Err(message) = io::seed_omitted_named(sk, ckind, &mut args, |i| left_out[i], &name) {
        diags.push(Diag { code: Code::E061, span: st.span, stmt: Some(st.id), message });
        return None;
    }
    let mut c = Constraint::new(ckind, args);
    c.claim = r.claim;
    c.class = r.class.clone();
    c.written = written(&r.args, r.kind.spec(), st, doc);
    Some(sk.add_quiet(c))
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

fn to_arg(sk: &Sketch, res: &Resolver, kind: SpecKind, a: &Arg) -> Result<CArg, (Code, String)> {
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
                    // said what a number is, and saying so is `unit mm` (spec §3.3)
                    expr::parse_in(text, sk.units).map_err(|e| (Code::E040, e.to_string()))?;
                    CArg::Expr(expr::Expr::new(text.trim().to_string(), 0.0))
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

/// A gauge or an orientation predicate, **applied** (issue #47, item 5): written and settled as
/// every other relation — an operator, a class, a placement — but holding parameters or
/// recording a root choice instead of becoming a constraint the sketch holds, so `constrain`
/// returns no id for it and the map never knows it.  The checks are the ones the two statement
/// kinds always made, in their words.
fn apply_gauge(
    sk: &mut Sketch,
    res: &Resolver,
    r: &ResolvedRelation<'_>,
    st: &Stmt,
    diags: &mut Vec<Diag>,
) {
    let refs: Vec<&Ref> = r
        .args
        .iter()
        .filter_map(|a| match a {
            Some(Arg::Ref(rf)) => Some(rf),
            _ => None,
        })
        .collect();
    let mut bad = |code: Code, span: Span, message: String| {
        diags.push(Diag { code, span, stmt: Some(st.id), message })
    };
    match r.kind {
        CKind::Fix => {
            let Some(rf) = refs.first().copied() else {
                bad(Code::E103, st.span, "`fix` names what it holds".to_string());
                return;
            };
            let Some(e) = res.lookup(rf).and_then(|e| follow(sk, e, &rf.path).ok()) else {
                bad(Code::E101, rf.span, format!("no such entity: `{}`", rf.root.text));
                return;
            };
            // a plane holds where it stands; an axis its direction and its origin
            let members = e.kind.members();
            let spec = r.kind.spec();
            let own = sk.own_params(e);
            let owned = &members[..own.len().min(members.len())];
            let kind = e.kind.as_str();
            let article = if kind.starts_with(['a', 'e', 'i', 'o', 'u']) { "an" } else { "a" };
            // a vector held whole is as long as the vector it holds: a point in a plane is two
            // numbers and one in space three, so `(0, 0)` cannot leave a height free unsaid
            for a in r.written.map_or(&[][..], |w| &w.args[..]) {
                let crate::syntax::OpArg::Vector { key, parts, span } = a else { continue };
                let k = key.as_ref().map_or("", |k| k.text.as_str());
                let Some(&(_, first)) = e.kind.vectors().iter().find(|(v, _)| *v == k) else {
                    let m = match k {
                        _ if owned.is_empty() => format!("{article} {kind} has no number of its \
                                                          own to fix"),
                        "" => format!("{article} {kind} is no vector: it has {}", said(e.kind, owned)),
                        _ => format!("{article} {kind} has {}, not `{k}`", said(e.kind, owned)),
                    };
                    bad(Code::E105, *span, m);
                    return;
                };
                let n = owned.len().saturating_sub(first).min(3);
                if parts.len() != n {
                    let m = match (k, n) {
                        ("", 2) => "a point in a plane has two coordinates: `(x, y)`".to_string(),
                        ("", _) => "a point in space has three coordinates: `(x, y, z)`".to_string(),
                        _ => format!("`{k}` has three components: `{k} == (x, y, z)`"),
                    };
                    bad(Code::E105, *span, m);
                    return;
                }
            }
            for (i, a) in r.args.iter().enumerate().skip(1) {
                let Some(a) = a else { continue };
                let field = spec[i].0;
                let Some(at) = owned.iter().position(|&n| n == field) else {
                    bad(
                        Code::E105,
                        st.span,
                        if owned.is_empty() {
                            format!("{article} {kind} has no number of its own to fix")
                        } else {
                            format!("{article} {kind} has {}, not `{field}`", said(e.kind, owned))
                        },
                    );
                    continue;
                };
                // a `fix` holds a number, and an unknown is not one
                if let Arg::Tie { span, .. } = a {
                    let m = format!("`fix` holds a number, and `{field}` is pinned to an unknown");
                    bad(Code::E040, *span, m);
                    continue;
                }
                // what the flattener settled the pin to; an expression it could not work out was
                // reported there
                let Arg::Seed { value, .. } = a else { continue };
                // a cone's half-angle is written in degrees, as its hint is (`Sketch::seed_value`)
                let v = if e.kind == EntKind::Cone { value.to_radians() } else { *value };
                // where an axis is (its numbers after the direction's three), held: placed first,
                // so no later relation reading its place frees what this holds
                if e.kind == EntKind::Axis && at >= 3 {
                    sk.place_axis(e.i());
                }
                let p = &mut sk.params[own[at] as usize];
                p.value = v;
                p.fixed = true;
            }
        }
        CKind::Ccw | CKind::Cw => {
            if refs.len() != 3 {
                bad(Code::E103, st.span, "an orientation names three points".to_string());
                return;
            }
            let mut pts = [0usize; 3];
            for (i, rf) in refs.iter().enumerate() {
                match res.lookup(rf).and_then(|e| follow(sk, e, &rf.path).ok()) {
                    Some(e) if e.kind == EntKind::Point => pts[i] = e.i(),
                    _ => {
                        bad(Code::E101, rf.span, format!("no such point: `{}`", rf.root.text));
                        return;
                    }
                }
            }
            // canonical, so the choice the document states and the one the plan replays are one
            // record and not two that never meet (issue #48, item 4)
            let (key, v) = decompose::branch_record(pts, r.kind == CKind::Ccw);
            sk.branches.insert(key, v);
        }
        _ => unreachable!("{:?} is not a gauge", r.kind),
    }
}

/// What an entity has to hold, as the source names it: a point's coordinates (`x and y`), and
/// for anything else its vectors whole (`dir and origin`) beside its scalars (`r`).
fn said(kind: EntKind, owned: &[&str]) -> String {
    let mut names: Vec<&str> = Vec::new();
    for m in owned {
        let head = m.split_once('.').map_or(*m, |(v, _)| v);
        if !names.contains(&head) {
            names.push(head);
        }
    }
    if kind == EntKind::Point && names.len() == 3 {
        return "x, y and z".to_string();
    }
    names.join(" and ")
}

impl Relation {
    pub(crate) fn resolve(
        &self,
        kind_of: &dyn Fn(&Ref) -> Option<EntKind>,
    ) -> Result<ResolvedRelation<'_>, (Span, String)> {
        let (kind, args) = match &self.form {
            RelationForm::Written(w) => settle(w, kind_of)?,
            RelationForm::Canonical { kind, args } => (*kind, args.clone()),
        };
        Ok(ResolvedRelation {
            kind,
            args,
            written: self.form.written(),
            claim: self.claim,
            class: &self.class,
        })
    }
}

/// What a dimension's text comes to as written, in the document's units (degrees for an angle):
/// `None` for a number with no text, or a text that reads an unknown.
fn written_number(a: &CArg, sk: &Sketch) -> Option<f64> {
    let CArg::Expr(e) = a else { return None };
    expr::parse_in(&e.text, sk.units).ok()
        .and_then(|p| expr::eval(&p.body, &Default::default()).ok())
        .and_then(|a| a.number())
}
