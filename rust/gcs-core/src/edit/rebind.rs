//! **A document's values rebound for one run** (#656): `solventc --set checks=1 bot.sv`.
//!
//! A switch at the top of a document (`checks := 0`, `shaped := 1`) is meant to be flipped per
//! run, and a copy of the file with the line changed drifts from the file, is easy to commit, and
//! is reported under the wrong name.  So a host hands the text and the values over and gets the
//! text back with each binding's value spliced in — the same splice a dimension typed over a
//! `param` is (`edit::dimension`) — and elaborates that, under the document's own name and beside
//! its own modules.  The value is text, read where the binding stands, so it is a Solvent
//! expression in the document's units (`stem_travel=3mm`) and may read the document's names.
//!
//! Only a value the document binds at its top level can be given: a name inside a component is
//! the component's, one a `use`d module binds is the module's, and an input nothing binds is an
//! unknown of the solve rather than a value.  Each is refused, naming the binding.

use crate::syntax::{self, StmtKind};

/// One value given over the one the document writes.
#[derive(Debug)]
pub struct Rebound {
    pub name: String,
    /// What the document writes after `:=`.
    pub written: String,
    /// What the run reads there instead.
    pub value: String,
}

/// `src` with each `(name, value)` given in place of the value its top-level `name := …` writes,
/// and what was given over what, in the order asked; or why one cannot be.
///
/// Every line stays where it was (a value is one line), so a diagnostic still names the line
/// the author wrote it on.
pub fn rebind(src: &str, values: &[(String, String)]) -> Result<(String, Vec<Rebound>), String> {
    let (prog, _) = syntax::parse(src);
    let mut splices: Vec<super::Splice> = Vec::new();
    let mut rebound: Vec<Rebound> = Vec::new();
    for (name, value) in values {
        let (name, value) = (name.trim(), value.trim());
        if name.is_empty() {
            return Err(format!("a value `{value}` is given no name"));
        }
        if rebound.iter().any(|r| r.name == name) {
            return Err(format!("`{name}` is given a value twice"));
        }
        if value.is_empty() {
            return Err(format!("`{name}` is given no value"));
        }
        if value.contains(['\n', '\r']) {
            return Err(format!("`{name}` is given a value of more than one line"));
        }
        if let Err(why) = crate::expr::parse(value) {
            return Err(format!("`{name}` is given `{value}`, which is no expression: {why}"));
        }
        let at = top_level(&prog, name)?;
        let written = src[at.lo as usize..at.hi as usize].to_string();
        // a bare number is read in the unit of the value it replaces (#940): `--set g=-6` over
        // `g := 1"` is `-6"`, a length still, and not a plain number a `max` with a length refuses
        let value = match unit_of(&prog, name, &written) {
            Some(unit) if bare(value) => format!("{value}{unit}"),
            _ => value.to_string(),
        };
        splices.push(super::Splice { at, with: value.clone() });
        rebound.push(Rebound { name: name.to_string(), written, value });
    }
    Ok((super::splice(src, splices), rebound))
}

/// Whether `rebind`'s refusal `why` is that the document binds no value of the name at its top
/// level at all — what a drawing's run-wide `--set` skips a model for (#940), as against a value
/// that cannot be given where it is bound.
pub fn binds_no_value(why: &str) -> bool {
    why.starts_with("the document binds no value")
}

/// Whether a value given is a number that names no unit: `-6`, `2.5`, `3 1/8`.
fn bare(value: &str) -> bool {
    let unsigned = value.trim().trim_start_matches(['-', '+']).trim_start();
    (crate::expr::literal(unsigned).is_some() || crate::expr::notation(unsigned))
        && !crate::expr::names_unit(unsigned)
}

/// **The unit a bare number given for `name` is read in** (#940): the one its written value names
/// where that is one literal (`1"`, `3mm`, `30deg`, `400lbf`); else, where the value is a length or
/// an angle — by its declared type, or worked out from the values the document binds above it —
/// the document's unit, and degrees; else none, a plain number staying one.
fn unit_of(prog: &syntax::Program, name: &str, written: &str) -> Option<String> {
    if crate::expr::names_unit(written) {
        // the suffix after the last digit: `"` of `1' 6 3/16"`, the smallest unit it is written in
        let t = written.trim();
        let end = t.rfind(|c: char| c.is_ascii_digit() || c == '.')? + 1;
        let unit = t[end..].trim();
        return (!unit.is_empty()).then(|| unit.to_string());
    }
    let units = prog.root().body.iter().find_map(|st| match &st.kind {
        StmtKind::Unit(n) => crate::units::Units::with_length(&n.text).ok(),
        _ => None,
    }).unwrap_or_default();
    let dim = dim_of(prog, name, units)?;
    if dim == crate::units::Dim::LENGTH {
        Some(units.name().to_string())
    } else if dim == crate::units::Dim::ANGLE {
        Some("deg".to_string())
    } else {
        None
    }
}

/// The dimension of the document's top-level value `name`: its declared type where it has one,
/// else its value worked out over the values bound above it, as elaboration works them out.
/// `None` where it reads what this cannot (geometry, a component's member).
fn dim_of(prog: &syntax::Program, name: &str, units: crate::units::Units)
    -> Option<crate::units::Dim>
{
    use crate::expr::Aff;
    let mut env: std::collections::BTreeMap<String, Aff> = Default::default();
    for st in &prog.root().body {
        let StmtKind::Param(p) = &st.kind else { continue };
        if !p.bound() { continue }
        let declared = p.input.as_ref().and_then(|i| i.ty).map(syntax::Ty::dim)
            .filter(|d| !d.is_scalar());
        let value = crate::expr::parse_in(&p.text, units).ok()
            .and_then(|parsed| crate::expr::eval(&parsed.body, &env).ok());
        let dim = declared.or(value.as_ref().map(|v| v.dim));
        if p.name.text == name { return dim; }
        if let Some(v) = value {
            env.insert(p.name.text.clone(), match declared { Some(d) => v.as_dim(d), None => v });
        }
    }
    None
}

/// Where the value of the document's top-level `name := …` is written, or why there is none to
/// give another.
fn top_level(prog: &syntax::Program, name: &str) -> Result<syntax::Span, String> {
    let bound = prog.root().body.iter()
        .find(|st| st.kind.bound_name().is_some_and(|n| n.text == name));
    match bound.map(|st| &st.kind) {
        Some(StmtKind::Param(p)) if p.bound() => Ok(p.span),
        Some(StmtKind::Param(_)) => Err(format!(
            "`{name}` is an input nothing binds, an unknown of the solve: there is no value to \
             give another")),
        Some(_) => Err(format!("`{name}` is bound at the top level, but not to a value")),
        None => {
            let inside = prog.components.iter().rev().skip(1)
                .filter(|c| c.module.is_none())
                .find(|c| c.body.iter().any(|st| {
                    st.kind.bound_name().is_some_and(|n| n.text == name)
                }));
            Err(match inside.and_then(|c| c.name.as_ref()) {
                Some(c) => format!(
                    "`{name}` is bound inside component `{}`, and only a value the document binds \
                     at its top level can be given",
                    c.text),
                None => format!("the document binds no value `{name}` at its top level"),
            })
        }
    }
}
