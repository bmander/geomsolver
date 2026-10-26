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
#[derive(Clone, Debug, PartialEq, Eq)]
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
    let mut splices: Vec<(syntax::Span, Rebound)> = Vec::new();
    for (name, value) in values {
        let value = value.trim();
        if values.iter().filter(|(n, _)| n == name).count() > 1 {
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
        splices.push((at, Rebound { name: name.clone(), written, value: value.to_string() }));
    }
    let mut out = src.to_string();
    let mut back: Vec<&(syntax::Span, Rebound)> = splices.iter().collect();
    back.sort_by_key(|(at, _)| std::cmp::Reverse(at.lo));
    for (at, r) in back {
        out.replace_range(at.lo as usize..at.hi as usize, &r.value);
    }
    Ok((out, splices.into_iter().map(|(_, r)| r).collect()))
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
