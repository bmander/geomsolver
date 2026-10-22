//! **`arm := beam(span, A: …, Iy: …, Iz: …, J: …) of steel`, a frame's members** (#510 S0), as
//! written.  `program::beams` elaborates one into a `model::Beam`; the solve never reads it.

use super::names::write_ref;
use super::{Name, Ref, Span};

/// `arm := beam(span, A: b * h, Iy: …, Iz: …, J: …) of steel`: the lines it makes members,
/// its section's numbers as written, and the material it is made of.
#[derive(Clone, Debug)]
pub struct BeamDecl {
    pub name: Name,
    pub lines: Vec<Ref>,
    /// Each number of its section: its label, and its text and where it is written — the
    /// flattener writes the scope's numbers into the text, and the elaborator works it out.
    pub section: Vec<(Name, (String, Span))>,
    pub material: Option<Ref>,
}

impl BeamDecl {
    /// Every number's text, mutably.
    pub fn numbers_mut(&mut self) -> Vec<&mut String> {
        self.section.iter_mut().map(|(_, (t, _))| t).collect()
    }

    /// As it is written: `arm := beam(span, A: b * h, …) of steel`.
    pub fn write(&self, out: &mut String) {
        out.push_str(&self.name.text);
        out.push_str(" := beam(");
        let mut first = true;
        for l in &self.lines {
            if !first {
                out.push_str(", ");
            }
            first = false;
            write_ref(out, l);
        }
        for (label, (text, _)) in &self.section {
            out.push_str(", ");
            out.push_str(&label.text);
            out.push_str(": ");
            out.push_str(text);
        }
        out.push(')');
        if let Some(m) = &self.material {
            out.push_str(" of ");
            write_ref(out, m);
        }
    }
}
