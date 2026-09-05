//! Resolve constraint arguments and apply gauges.

use super::resolve::{follow, Resolver};
use super::solids::is_body_on;
use super::{Code, Diag};
use crate::constraints::{Arg as CArg, CKind, Constraint, SpecKind};
use crate::ir::{Relation, ResolvedRelation, Statement as Stmt};
use crate::model::{EntKind, EntRef, Field, Sketch};
use crate::syntax::{Arg, Ref, RelationForm, Seg, Span};
use crate::{decompose, expr, io};

/// Resolve an operator to its constraint kind and registry-ordered arguments.
pub(crate) fn settle(
    w: &crate::syntax::Written,
    kind_of: &dyn Fn(&Ref) -> Option<EntKind>,
) -> Result<(CKind, Vec<Option<Arg>>), (Span, String)> {
    use crate::constraints::Fixity;
    let word = w.word.text.as_str();
    // a gauge or an orientation is settled by its word alone: `fix c.r` names a number and not
    // an entity, and `ccw(a, b, c)` has no operand outside its parentheses; what each operand
    // must be is checked where the statement is applied, in the words the gauges always used
    if let Some(k) = crate::constraints::gauge_op(word) {
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
    // a name that resolves to nothing is reported on the argument itself, where the message can
    // say which name it was; here it only means the word cannot be settled
    let named = |k: Option<EntKind>| k.map(|k| k.as_str()).unwrap_or("that");
    let kind = match w.fixity {
        Fixity::Prefix => {
            let Some(a) = kinds.first().copied().flatten() else {
                let m = format!("`{word}` needs to know what `{}` is", w.ops[0].root.text);
                return Err((w.word.span, m));
            };
            crate::constraints::prefix_op(word, a).ok_or_else(|| {
                (w.word.span, format!("`{word}` does not apply to a {}", a.as_str()))
            })?
        }
        Fixity::Infix => {
            let (a, b) = (kinds.first().copied().flatten(), kinds.get(1).copied().flatten());
            let (Some(a), Some(b)) = (a, b) else {
                return Err((w.word.span, format!("`{word}` needs to know what its operands are")));
            };
            crate::constraints::infix_op(word, a, b, &|n| w.sel(n)).ok_or_else(|| {
                let m = format!(
                    "`{word}` does not relate a {} to a {}",
                    named(Some(a)),
                    named(Some(b))
                );
                (w.word.span, m)
            })?
        }
        // only a gauge word is written as a call, and those were settled above
        Fixity::Call => return Err((w.word.span, format!("`{word}` is not a call"))),
    };
    Ok((kind, w.assemble(kind)?))
}

pub(super) fn constrain(
    sk: &mut Sketch,
    res: &Resolver,
    r: &Relation,
    st: &Stmt,
    diags: &mut Vec<Diag>,
) -> Option<u32> {
    // **`on` between two solids is the body rule and not a constraint** (§6.9).  A word means
    // the kinds of its operands, and this is that rule reaching one word further out: `p on c`
    // holds a point to a circle, and `boss on cyl` says what a body is made of.  It is picked
    // up by the solids phase, so nothing is added here — and nothing is *said* here either, or
    // one statement would be reported twice.
    if is_body_on(res, r) {
        // **a claim on the body rule is refused** (§9.7's rule, one stratum out): a claim is
        // judged by rank and `on` between two solids adds none, so the word says nothing.  It is
        // said *here* because this is where the claim flag is still in hand — the solids phase
        // reads the statement again and would have to ask a second time.
        if r.claim {
            diags.push(Diag {
                code: Code::E040,
                span: st.span,
                stmt: Some(st.id),
                message: "`on` between two solids says what a body is made of, and a claim is \
                          judged by rank: there is no row here to claim about"
                    .to_string(),
            });
        }
        return None;
    }
    // **a word that relates two solids is a claim, judged and never solved** (§9.8).  Picked up
    // by the solids phase for the reason `on` is: nothing here has a kernel, and saying so twice
    // would report one statement twice.
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
            let v = args[i].num();
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
                        "a {} is a magnitude and cannot be negative{fix}",
                        crate::syntax::snake(ckind.name())
                    ),
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
    if let Err(message) = io::seed_omitted(sk, ckind, &mut args, |i| left_out[i]) {
        diags.push(Diag { code: Code::E061, span: st.span, stmt: Some(st.id), message });
        return None;
    }
    let mut c = Constraint::new(ckind, args);
    c.claim = r.claim;
    c.class = r.class.clone();
    Some(sk.add_quiet(c))
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
                "`{}` is a {}, and a {} is wanted here",
                r.root.text,
                e.kind.as_str(),
                kind.as_str()
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
        (k, other) => {
            return Err((Code::E040, format!("a {} is wanted here, not {other:?}", k.as_str())))
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
        CKind::Ground | CKind::Fix => {
            let Some(rf) = refs.first().copied() else {
                bad(
                    Code::E103,
                    st.span,
                    format!("`{}` names what it pins", crate::syntax::snake(r.kind.name())),
                );
                return;
            };
            let Some(e) = res.lookup(rf) else {
                bad(Code::E101, rf.span, format!("no such entity: `{}`", rf.root.text));
                return;
            };
            if r.kind == CKind::Ground {
                let e = follow(sk, e, &rf.path).unwrap_or(e);
                if e.kind != EntKind::Point {
                    bad(
                        Code::E105,
                        st.span,
                        "ground pins a point; a scalar is pinned with fix".to_string(),
                    );
                    return;
                }
                sk.fix_point(e.i(), true);
                return;
            }
            // `fix c.r`: the entity's own scalar, named by the field it is.  The document
            // stores one flag per scalar and nothing finer, so neither does this.
            let field = match rf.path.first() {
                Some(Seg::Field(f)) => f.text.clone(),
                _ => String::new(),
            };
            let own = sk.own_params(e);
            let scalars: Vec<&str> = e
                .kind
                .fields()
                .iter()
                .filter(|(_, f)| *f == Field::Scalar)
                .map(|(n, _)| *n)
                .collect();
            match scalars.iter().position(|&n| n == field) {
                Some(i) if i < own.len() => sk.params[own[i] as usize].fixed = true,
                _ => bad(
                    Code::E105,
                    st.span,
                    if scalars.is_empty() {
                        format!("a {} has no number of its own to fix", e.kind.as_str())
                    } else {
                        format!(
                            "a {} has {}, not `{field}`",
                            e.kind.as_str(),
                            scalars.join(" and ")
                        )
                    },
                ),
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
