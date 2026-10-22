//! **A frame's members, elaborated** (#510 S0): each `arm := beam(span, A: …) of steel` into a
//! `model::Beam` — its lines resolved by the selectors every operand is (`Resolver::operand`),
//! its section's numbers worked out and held to what they are — read once the drawing's
//! geometry is built.  Nothing is solved and no row is stated: a beam adds nothing to the
//! drawing, and only a scenario's analysis reads it (`beams::read`).

use super::resolve::Resolver;
use super::{Code, Diag};
use crate::ir::{Operation, Statement};
use crate::model::{Beam, EntKind, Section, SectionProp, Sketch};
use crate::syntax::ref_text;
use std::collections::BTreeMap;

/// Every beam of the document, in the order written; one that cannot be read is said and left
/// out whole, since a frame missing a member is another frame.
pub(super) fn beams(
    sk: &Sketch,
    res: &Resolver,
    body: &[&Statement],
    diags: &mut Vec<Diag>,
) -> Vec<Beam> {
    let mut out: Vec<Beam> = Vec::new();
    for st in body {
        let Operation::Beam(b) = &st.kind else { continue };
        let said = diags.len();
        let mut say = |code, span, message: String| {
            diags.push(Diag::within(code, span, st.id, message));
        };
        let mut lines = Vec::new();
        for r in &b.lines {
            match res.operand(sk, r) {
                Ok(op) => match op.ent() {
                    Some(e) if e.kind == EntKind::Line => {
                        let [p, q] = sk.line_ends(e.i());
                        if sk.plane_of(p).is_none() || sk.plane_of(p) != sk.plane_of(q) {
                            say(Code::E040, r.span, format!(
                                "`{}` stands in space, and a beam is drawn in a plane, both its \
                                 ends in it: its section's z is the plane's normal", ref_text(r)));
                        }
                        lines.push(e.i());
                    }
                    _ => say(Code::E040, r.span, format!(
                        "a beam is drawn on lines, and `{}` is none", ref_text(r))),
                },
                Err(m) => say(Code::E101, r.span, m),
            }
        }
        let mut props = BTreeMap::new();
        for (label, (text, span)) in &b.section {
            let p = match SectionProp::named(&label.text) {
                Ok(p) => p,
                Err(m) => {
                    say(Code::E103, label.span, m);
                    continue;
                }
            };
            if props.contains_key(&p) {
                say(Code::E001, label.span, format!("`{}` is given twice", label.text));
                continue;
            }
            match number(sk, text, p) {
                Ok(v) => {
                    props.insert(p, v);
                }
                Err((code, m)) => say(code, *span, m),
            }
        }
        let written = |p: &SectionProp| b.section.iter().any(|(l, _)| l.text == p.name());
        let missing: Vec<String> = SectionProp::STIFFNESS.iter()
            .filter(|p| !props.contains_key(p) && !written(p))
            .map(|p| format!("`{}`", p.name()))
            .collect();
        if !missing.is_empty() {
            say(Code::E103, b.name.span, format!(
                "a beam states its section's stiffness, `A`, `Iy`, `Iz` and `J`, and `{}` \
                 leaves out {}", b.name.text, crate::vocab::all_of(&missing)));
        }
        if diags.len() > said {
            continue;
        }
        let material = b.material.as_ref().map(ref_text).filter(|m| sk.materials.contains_key(m));
        out.push(Beam { name: b.name.text.clone(), lines, section: Section { props }, material });
    }
    out
}

/// A section's number as written, worked out: of its dimension, positive, and never an unknown
/// of the solve, since nothing drives a section until a requirement does (#510 S1).
fn number(sk: &Sketch, text: &str, p: SectionProp) -> Result<f64, (Code, String)> {
    let v = crate::flatten::value_read(text, &BTreeMap::new(), &sk.declared, sk.units)
        .map_err(|m| (Code::E103, format!("`{text}`: {m}")))?;
    v.dim.require(p.dim(), p.name()).map_err(|m| (Code::E103, format!("`{text}`: {m}")))?;
    match v.number() {
        Some(x) if x.is_finite() && x > 0.0 => Ok(x),
        Some(x) => Err((Code::E103, format!("`{}` is positive, and `{text}` is {x}", p.name()))),
        None => Err((Code::E040, format!(
            "`{text}` reads an unknown of the drawing, and a beam's section is a number it states \
             until a requirement drives one"))),
    }
}
