//! Solid claims, face bearing and placement after construction.
use super::*;

/// Apply solid claims after body construction, preserving statement spans for diagnostics.
pub(in crate::program) fn solid_claims(
    sk: &mut Sketch,
    res: &Resolver,
    map: &mut SourceMap,
    body: &[&Stmt],
    skip: &BTreeSet<StmtId>,
    diags: &mut Vec<Diag>,
) {
    // A claim over a sweep is the *same* claims with an interval attached, so both forms go
    // through one reader: the block says how its body is judged and asserts nothing itself.
    for st in body {
        match &st.kind {
            StmtKind::Relation(_) => solid_claim(sk, res, st, None, map, skip, diags),
            StmtKind::ClaimOver(c) if !skip.contains(&st.id) => {
                let over = match sweep_of_claim(sk, res, c, st, diags) {
                    Some(o) => o,
                    None => continue,
                };
                for inner in &c.body {
                    solid_claim(sk, res, inner, Some(&over), map, skip, diags);
                }
                map.record(st, Made::Gauge);
            }
            _ => {}
        }
    }
}

fn solid_claim(
    sk: &mut Sketch,
    res: &Resolver,
    st: &Stmt,
    over: Option<&crate::model::Sweep>,
    map: &mut SourceMap,
    skip: &BTreeSet<StmtId>,
    diags: &mut Vec<Diag>,
) {
    let StmtKind::Relation(r) = &st.kind else {
        return;
    };
    if skip.contains(&st.id) {
        return;
    }
    let Some(w) = r.form.written() else { return };
    let Some(word) = crate::constraints::solid_word(&w.word.text) else {
        return;
    };
    let mut say = |code: Code, span: Span, m: String| {
        diags.push(Diag {
            code,
            span,
            stmt: Some(st.id),
            message: m,
        });
    };
    // **it is a claim whether or not it says so.**  These words cannot act — there is no row for
    // them — so a document that writes one without `claim` is saying the same thing, and refusing
    // it would be a rule about spelling rather than about meaning.  Written *with* `claim` is the
    // reading to prefer, and the printer spells it that way.
    if w.ops.len() != 2 {
        say(
            Code::E040,
            st.span,
            format!("`{}` relates two solids", word.as_str()),
        );
        return;
    }
    let mut ends = Vec::new();
    for o in &w.ops {
        match res.lookup(o) {
            Some(e) if e.kind == EntKind::Solid => ends.push(e.idx),
            Some(e) => {
                say(
                    Code::E040,
                    o.span,
                    format!(
                        "`{}` relates solids, and `{}` is a {}",
                        word.as_str(),
                        o.root.text,
                        e.kind.as_str()
                    ),
                );
                return;
            }
            None => {
                say(
                    Code::E101,
                    o.span,
                    format!("no such entity: `{}`", o.root.text),
                );
                return;
            }
        }
    }
    use crate::model::{Length, SolidRequirement};
    let requirement = match (word.takes_gap(), w.args.as_slice()) {
        (false, []) => SolidRequirement::Inside,
        (true, [crate::syntax::OpArg::Dim(text, span)]) => {
            match crate::flatten::value_aff(text, &BTreeMap::new(), sk.units) {
                Ok(v)
                    if v.c.is_finite()
                        && v.dim
                            .require(crate::units::Dim::LENGTH, word.as_str())
                            .is_ok() =>
                {
                    let gap = Length::written(v.c, text.trim().to_string()).unwrap();
                    match word {
                        crate::constraints::SolidWord::Fits => SolidRequirement::Fits { gap },
                        crate::constraints::SolidWord::Clear => SolidRequirement::Clear { gap },
                        _ => unreachable!(),
                    }
                }
                _ => {
                    say(
                        Code::E103,
                        *span,
                        format!("`{}` asks for a finite length", word.as_str()),
                    );
                    return;
                }
            }
        }
        (true, []) => {
            say(
                Code::E040,
                st.span,
                format!(
                    "`{}` asks for room: `{}(2mm)`",
                    word.as_str(),
                    word.as_str()
                ),
            );
            return;
        }
        _ => {
            say(
                Code::E040,
                st.span,
                format!(
                    "`{}` takes {}",
                    word.as_str(),
                    if word.takes_gap() {
                        "exactly one length argument"
                    } else {
                        "no arguments"
                    }
                ),
            );
            return;
        }
    };
    sk.solid_claims.push(crate::model::SolidClaim {
        requirement,
        a: ends[0],
        b: ends[1],
        over: over.cloned(),
        stmt: st.id.0,
    });
    map.record(st, Made::Gauge);
}

/// The interval a swept claim runs over: a free variable of the drawing, and where it goes.
fn sweep_of_claim(
    sk: &Sketch,
    res: &Resolver,
    c: &crate::ir::ClaimOver,
    st: &Stmt,
    diags: &mut Vec<Diag>,
) -> Option<crate::model::Sweep> {
    let mut say = |code: Code, span: Span, m: String| {
        diags.push(Diag {
            code,
            span,
            stmt: Some(st.id),
            message: m,
        });
    };
    let name = crate::syntax::ref_text(&c.formal);
    // **a named motion's roll**: the claim read at every pose of the solids placed under it
    let motion = res.lookup(&c.formal).filter(|e| e.kind == crate::model::EntKind::Motion && e.i() < sk.motions.len());
    // **a free variable, and nothing else**: a `param` is a number the document already fixed,
    // and sweeping it would be sweeping a constant
    if motion.is_none() && !sk.free_vars.contains_key(&name) {
        if res.lookup(&c.formal).is_some() {
            say(
                Code::E040,
                c.formal.span,
                format!("`{name}` is geometry, not a free variable or a motion"),
            );
        } else {
            say(
                Code::E040,
                c.formal.span,
                format!(
                    "`{name}` is not a free variable or a motion of this drawing: a swept claim \
                         runs along an unknown the solver answers for, or a motion's roll"
                ),
            );
        }
        return None;
    }
    let dimension = if motion.is_some() { crate::units::Dim::ANGLE } else { *sk.free_dimensions.get(&name)? };
    // Free parameters are stored in user units. In particular, an angular unknown is in
    // degrees; its readers' affine coefficients own the radians conversion into kernels.
    // Check the inferred variable dimension before assigning either endpoint.
    let mut num = |a: &crate::syntax::Arg, what: &str| -> Option<f64> {
        let crate::syntax::Arg::Dim { text, span } = a else {
            say(
                Code::E103,
                st.span,
                format!(
                    "`{what}` for `{name}` must be a finite {}",
                    dimension.name()
                ),
            );
            return None;
        };
        match crate::flatten::value_aff(text, &BTreeMap::new(), sk.units) {
            Ok(v) if v.c.is_finite() && v.dim.require(dimension, &name).is_ok() => Some(v.c),
            Ok(_) => {
                say(
                    Code::E103,
                    *span,
                    format!(
                        "`{what}` for `{name}` must be a finite {}",
                        dimension.name()
                    ),
                );
                None
            }
            Err(e) => {
                say(Code::E103, *span, format!("`{what}`: {e}"));
                None
            }
        }
    };
    let (from, to) = (num(&c.from, "from")?, num(&c.to, "to")?);
    Some(crate::model::Sweep {
        name,
        from,
        to,
        dimension,
        motion: motion.map(|e| e.idx),
    })
}
