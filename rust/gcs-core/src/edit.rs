//! Editing a program: what a gesture on the drawing does to the source.
//!
//! This is the module that makes the source the *document* rather than a view of one.  Every edit
//! here is a **splice** — a few characters replaced in the text somebody wrote — and never a
//! reprint.  That distinction is the whole design:
//!
//! * a reprint would flatten a hand-written `component` into the entities it elaborates to, throw
//!   away every comment, and reflow every line, on the first drag;
//! * a splice rewrites the six characters of one seed and leaves the rest of the file alone.
//!
//! So a gear written as a `Tooth` in a `cycle` stays written that way while its points are
//! dragged, and the panel shows what the author wrote rather than what the solver made of it.
//!
//! Splices run **back to front**, so a span computed before the edit is still valid when its turn
//! comes.  Nothing here re-parses: the caller applies the returned text and elaborates once.

use crate::lowering::Shape;
use crate::model::{EntKind, EntRef, Sketch};
use crate::program::{Elaborated, Made, Site};
use crate::syntax::{self, num, Decl, Program, Span, Stmt, StmtKind};

mod seeds;
mod append;
mod entities;
mod solids;
mod datums;
mod remove;
mod dimension;
mod reconcile;
mod component;

pub use seeds::*;
pub use append::*;
pub use entities::*;
pub use solids::*;
pub use datums::*;
pub use remove::*;
pub use dimension::*;
pub use reconcile::*;
pub use component::make_component;

/// What an edit did to the document, and what it costs to take up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// Statements were added or removed: the sketch has to be built again.
    Structural,
    /// Only numbers a solve is allowed to move changed.  The topology is untouched, so a caller
    /// holding a compiled plan may keep it.
    Numeric,
    /// Nothing to do, or nothing this edit is able to do.
    None,
}

#[derive(Clone, Debug)]
pub struct Edit {
    pub text: String,
    pub kind: Kind,
    /// The names a declaration was given, in the order they were made.
    pub names: Vec<String>,
    /// Why, when an edit could not be made.
    pub refused: Option<String>,
    /// What a removal was asked to take and left standing: entities a component made, which have
    /// no statement of their own to delete.
    pub kept: usize,
}

impl Edit {
    fn none(prog: &Program, why: Option<String>) -> Edit {
        Edit {
            text: prog.text().to_string(),
            kind: Kind::None,
            names: Vec::new(),
            refused: why,
            kept: 0,
        }
    }

    /// The source with `edits` applied, an edit of the class `kind`.
    fn spliced(prog: &Program, edits: Vec<Splice>, kind: Kind) -> Edit {
        Edit { text: splice(prog.text(), edits), kind, names: Vec::new(), refused: None, kept: 0 }
    }
}

/// Whether a seed is a number a solve may write over: a literal, or one in notation (`3 1/8`,
/// `30deg`) — not an expression, which is the author's arithmetic.
fn writable_seed(text: &str) -> bool {
    crate::expr::is_literal(text)
}

/// A solved seed as it is written back: in degrees where an angle's text named its unit.
fn seed_literal(text: &str, v: f64, angle: bool) -> String {
    if angle && crate::expr::names_unit(text) { format!("{}deg", num(v)) } else { num(v) }
}

/// One replacement in the source.
#[derive(Clone)]
struct Splice {
    at: Span,
    with: String,
}

/// Apply a set of replacements, back to front so earlier spans stay valid.
fn splice(text: &str, mut edits: Vec<Splice>) -> String {
    edits.sort_by_key(|e| std::cmp::Reverse(e.at.lo));
    let mut out = text.to_string();
    for e in edits {
        let (lo, hi) = (e.at.lo as usize, e.at.hi as usize);
        if lo > hi || hi > out.len() || !out.is_char_boundary(lo) || !out.is_char_boundary(hi) {
            continue; // a span that does not name a place in this text edits nothing
        }
        out.replace_range(lo..hi, &e.with);
    }
    out
}

/// Whether a statement is one of the root component's own.
///
/// Not the same question as "is it reached once".  A component instantiated a single time makes
/// one entity, so its pose is unambiguous and a seed *can* be written back — but the statement is
/// the *component's*, and deleting it would edit a reusable thing on behalf of a gesture that
/// named one entity.  So writeback asks "reached once" and deletion asks this.
fn in_root(prog: &Program, id: crate::syntax::StmtId) -> bool {
    prog.root().body.iter().any(|s| s.id == id)
}

fn decl_of<'a>(prog: &'a Program, site: &Site) -> Option<&'a Decl> {
    match &prog.stmt(site.stmt)?.kind {
        StmtKind::Decl(d) => Some(d),
        _ => None,
    }
}
