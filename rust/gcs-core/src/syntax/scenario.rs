//! **`scenario NAME { … }`, a load case** (#510 P4): the supports, the loads and the requirements
//! judged under them, as written.  `program::scenarios` elaborates one into a
//! `model::Scenario`; nothing in it is solved.

use super::names::write_ref;
use super::print::vector_text;
use super::{Name, Ref, Span};
use crate::expr::Cmp;
use crate::model::{Hold, LoadKind, Output};

/// `scenario lift { … }`, or `scenario overload: lift { … }` extending another.
#[derive(Clone, Debug)]
pub struct Scenario {
    pub name: Name,
    /// The scenario whose supports and loads come first, `: lift`.
    pub extends: Option<Name>,
    pub items: Vec<Item>,
    pub span: Span,
}

/// One line of a scenario's body.
#[derive(Clone, Debug)]
pub struct Item {
    pub kind: ItemKind,
    pub span: Span,
}

/// A number as written, `(text, span)`: the flattener writes the scope's numbers into the text,
/// and the elaborator works it out.
pub type Number = (String, Span);

#[derive(Clone, Debug)]
pub enum ItemKind {
    /// `fixed arm.base`, `pinned b along: up`.
    Support { hold: Hold, at: Ref, along: Option<Ref> },
    /// `load arm.tip (0, -500N, 0)`, `load tip 500N along: down`, `pressure lid 2MPa`,
    /// `accelerate arm (0, 0, -30m / 1s^2)`.
    Load { kind: LoadKind, at: Ref, value: LoadText },
    /// `claim mode(arm, 1) >= 120Hz`, `deflection(arm.tip) <= 0.5mm`, `safety(arm) == 2`.
    Requirement(RequirementText),
}

/// A load's number as written.
#[derive(Clone, Debug)]
pub enum LoadText {
    /// `(fx, fy, fz)`, in the world's axes.
    Vector(Vec<Number>),
    /// `500N along: down`.
    Along { magnitude: Number, along: Ref },
    /// A pressure's one number.
    Scalar(Number),
}

#[derive(Clone, Debug)]
pub struct RequirementText {
    pub claim: bool,
    pub output: Output,
    pub of: Ref,
    /// `deflection(p, along: d)`.
    pub along: Option<Ref>,
    /// `mode(b, n)`'s `n`.
    pub count: Option<Number>,
    pub compare: Cmp,
    pub value: Number,
}

impl Scenario {
    /// Every reference it makes, in the order written: what the flattener rescopes.
    pub fn refs_mut(&mut self) -> Vec<&mut Ref> {
        let mut out = Vec::new();
        for it in &mut self.items {
            match &mut it.kind {
                ItemKind::Support { at, along, .. } => {
                    out.push(at);
                    out.extend(along.as_mut());
                }
                ItemKind::Load { at, value, .. } => {
                    out.push(at);
                    if let LoadText::Along { along, .. } = value {
                        out.push(along);
                    }
                }
                ItemKind::Requirement(r) => {
                    out.push(&mut r.of);
                    out.extend(r.along.as_mut());
                }
            }
        }
        out
    }

    /// Every number's text, mutably: what the flattener writes the scope's numbers into.
    pub fn numbers_mut(&mut self) -> Vec<&mut String> {
        let mut out = Vec::new();
        for it in &mut self.items {
            match &mut it.kind {
                ItemKind::Support { .. } => {}
                ItemKind::Load { value, .. } => match value {
                    LoadText::Vector(parts) => out.extend(parts.iter_mut().map(|(t, _)| t)),
                    LoadText::Along { magnitude, .. } => out.push(&mut magnitude.0),
                    LoadText::Scalar(n) => out.push(&mut n.0),
                },
                ItemKind::Requirement(r) => {
                    out.extend(r.count.as_mut().map(|(t, _)| t));
                    out.push(&mut r.value.0);
                }
            }
        }
        out
    }

    /// As it is written: the header, then a line of the body per item, indented two spaces.
    pub fn write(&self, out: &mut String) {
        out.push_str("scenario ");
        out.push_str(&self.name.text);
        if let Some(base) = &self.extends {
            out.push_str(": ");
            out.push_str(&base.text);
        }
        out.push_str(" {\n");
        for it in &self.items {
            out.push_str("  ");
            it.write(out);
            out.push('\n');
        }
        out.push('}');
    }
}

/// `{sep}along: d`, where a direction is written.
fn write_along(out: &mut String, sep: &str, along: Option<&Ref>) {
    if let Some(d) = along {
        out.push_str(sep);
        out.push_str("along: ");
        write_ref(out, d);
    }
}

impl Item {
    pub fn write(&self, out: &mut String) {
        match &self.kind {
            ItemKind::Support { hold, at, along } => {
                out.push_str(hold.word());
                out.push(' ');
                write_ref(out, at);
                write_along(out, " ", along.as_ref());
            }
            ItemKind::Load { kind, at, value } => {
                out.push_str(kind.word());
                out.push(' ');
                write_ref(out, at);
                out.push(' ');
                match value {
                    LoadText::Vector(parts) => {
                        out.push_str(&vector_text(parts.iter().map(|(t, _)| t.clone())))
                    }
                    LoadText::Along { magnitude, along } => {
                        out.push_str(&magnitude.0);
                        write_along(out, " ", Some(along));
                    }
                    LoadText::Scalar(n) => out.push_str(&n.0),
                }
            }
            ItemKind::Requirement(r) => {
                if r.claim {
                    out.push_str("claim ");
                }
                r.write_head(out);
                out.push_str(&format!(" {} {}", r.compare.text(), r.value.0));
            }
        }
    }
}

impl RequirementText {
    /// Its output as written, `deflection(tip, along: down)`, `mode(arm, 2)`: what it is, and what
    /// a report says it of.
    pub fn write_head(&self, out: &mut String) {
        out.push_str(self.output.word());
        out.push('(');
        write_ref(out, &self.of);
        if let Some((n, _)) = &self.count {
            out.push_str(", ");
            out.push_str(n.trim());
        }
        write_along(out, ", ", self.along.as_ref());
        out.push(')');
    }
}
