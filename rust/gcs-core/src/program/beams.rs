//! **A frame's members, elaborated** (#510 S0): each `arm := beam(span, A: …) of steel` into a
//! `model::Beam` — its lines resolved by the selectors every operand is (`Resolver::operand`),
//! its section's numbers worked out and held to what they are — read once the drawing's
//! geometry is built.  A beam adds nothing to the drawing: a scenario's analysis reads it
//! (`beams::read`), and a driving deflection's row (#510 S1), which a section read over the
//! drawing's unknowns sizes (`model::Over`, `sized`).

use super::resolve::Resolver;
use super::{Code, Diag};
use crate::ir::{Operation, Statement};
use crate::model::{Beam, EntKind, Over, Section, SectionProp, Sketch};
use crate::units::Dim;
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
        let (mut props, mut over) = (BTreeMap::new(), BTreeMap::new());
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
                Ok((v, reads)) => {
                    props.insert(p, v);
                    if let Some(o) = reads {
                        over.insert(p, o);
                    }
                }
                Err((code, m)) => say(code, *span, m),
            }
        }
        let missing: Vec<String> = SectionProp::STIFFNESS.iter()
            .filter(|p| !b.section.iter().any(|(l, _)| l.text == p.name()))
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
        out.push(Beam { name: b.name.text.clone(), lines, section: Section { props, over }, material });
    }
    out
}

/// A section's number as written, worked out: of its dimension and positive — and where it reads
/// unknowns of the drawing (#510 S1), worked out at their seeds and kept as the text it is
/// (`model::Over`), so a driving requirement can size it.  An unknown it reads is a length or a
/// plain number (a tape reads no angle as the drawing holds one) and is seeded.
fn number(sk: &Sketch, text: &str, p: SectionProp) -> Result<(f64, Option<Over>), (Code, String)> {
    let parsed = crate::expr::parse_in(text, sk.units)
        .map_err(|m| (Code::E103, format!("`{text}`: {m}")))?;
    let reads = crate::expr::declared_reads(&parsed.body, |_| false, &sk.declared);
    if reads.is_empty() {
        let why = "a beam's section is a number it states, or one over the drawing's unknowns";
        let x = super::scenarios::stated(sk, text, p.dim(), p.name(), why)?;
        return positive(text, p, x).map(|x| (x, None));
    }
    let mut env = BTreeMap::new();
    for name in reads.keys() {
        let decl = &sk.declared[name];
        if decl.dim != Dim::LENGTH && decl.dim != Dim::SCALAR {
            return Err((Code::E040, format!(
                "`{name}` is {}, and a section is read over lengths and plain numbers",
                decl.dim.name())));
        }
        let seed = decl.seed.ok_or_else(|| (Code::E040, format!(
            "`{name}` has no seed, and a section read over it starts there: `hint(…)` it")))?;
        env.insert(name.clone(), crate::expr::Aff::of_dim(seed, decl.dim));
    }
    let v = crate::expr::eval(&parsed.body, &env)
        .map_err(|e| (Code::E103, format!("`{text}`: {e}")))?;
    v.dim.require(p.dim(), p.name()).map_err(|m| (Code::E103, format!("`{text}`: {m}")))?;
    let over = Over { text: text.to_string(), unknowns: reads.into_keys().collect() };
    over.tape(&over.unknowns, sk.units).map_err(|m| (Code::E103, format!("`{text}`: {m}")))?;
    let x = v.number().filter(|x| x.is_finite())
        .ok_or_else(|| (Code::E103, format!("`{text}` comes to no number")))?;
    positive(text, p, x).map(|x| (x, Some(over)))
}

/// `x`, where it is positive.
fn positive(text: &str, p: SectionProp, x: f64) -> Result<f64, (Code, String)> {
    if x > 0.0 {
        Ok(x)
    } else {
        Err((Code::E103, format!("`{}` is positive, and `{text}` is {x}", p.name())))
    }
}

/// **A section read over an unknown nothing sizes** (#510 S1): every beam whose section reads an
/// unknown no row of the drawing binds — no driving requirement on its frame, and no dimension
/// — is **E040** at the number, since nothing determines what it is.
pub(super) fn sized(sk: &Sketch, body: &[&Statement], diags: &mut Vec<Diag>) {
    for st in body {
        let Operation::Beam(b) = &st.kind else { continue };
        let Some(beam) = sk.beams.iter().find(|m| m.name == b.name.text) else { continue };
        for (label, (_, span)) in &b.section {
            let Ok(p) = SectionProp::named(&label.text) else { continue };
            let Some(over) = beam.section.over.get(&p) else { continue };
            let loose = over.unknowns.iter().find(|n| {
                sk.free_vars.get(*n).is_none_or(|&q| sk.params[q as usize].fixed)
            });
            if let Some(n) = loose {
                diags.push(Diag::within(Code::E040, *span, st.id, format!(
                    "`{n}` is an unknown nothing determines: a section is read over one a \
                     requirement drives, `deflection(…) == …` in a scenario on `{}`'s frame",
                    b.name.text)));
            }
        }
    }
}
