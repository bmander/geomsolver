//! **Lowering** (§6.21, #105, #140): a set whose body is the body of an element the drawing has
//! is drawn as that element — `{ p | p coincident P; p distance(r) o }` is `circle(center: o)`
//! with `radius(r)` on it, solved, dragged, dimensioned and exported as the circle.  This is the
//! table: what each shape is (`shape`, read by the flattener, which draws it, and by
//! `edit::set_dimension`, which edits its number), and when it holds (`refused`, the elaborator's
//! judgment).  A new lowering is a variant of each, and the matches say where else it is read.
//!
//! Lowering chooses a representation and never a meaning: a set drawn as a circle and the same
//! set walked as a set have the same solutions, and differ only in rows, topology and callouts.
//! So the flattener draws what a shape says, and the elaborator judges it once every membership
//! is in — those a use of *another* set makes included (`o coincident on`, `on` a set on a plane,
//! may be what draws a circle's centre in its plane), which is why the judgment cannot come
//! before the expansion.  A set refused is walked again as a set, and stays one: refusals only
//! grow, so `program::elaborate`'s passes end, and a pass ends at the judgment, before any
//! constraint is stated.
//!
//! A *use* is lowered elsewhere, and needs no judgment: `q coincident P` of a point in space is
//! its membership of `P` (`program::planes::incidences`).

use crate::model::{EntKind, EntRef, Sketch};
use crate::syntax::{Instance, Name, Ref, SetLit, Span, Stmt, StmtKind};
use std::collections::BTreeSet;

/// What a set's body is the body of.
pub(crate) enum Shape<'a> {
    /// `p coincident plane` and `p distance(r) centre`, in either order and either way round, and
    /// nothing else.
    Circle { plane: &'a Ref, centre: &'a Ref, radius: Radius<'a> },
}

/// A shape's number as its body writes it: the text, where it stands, and the word it is written
/// with — what a callout draws and an edit writes.
pub(crate) struct Radius<'a> {
    pub text: &'a str,
    pub span: Span,
    pub word: &'a Name,
}

impl Radius<'_> {
    /// Where the number is written: in the body, or — for a family's instance, `call` — where
    /// the call gives the formal the body reads (`None` where it gives none).  Where a callout is
    /// drawn from and an edit writes.
    pub(crate) fn at(&self, call: Option<&Instance>) -> Option<Span> {
        match call {
            None => Some(self.span),
            Some(inst) => inst.given(self.text.trim()),
        }
    }
}

impl Shape<'_> {
    /// What the elaborator judges the shape by, its references as the body writes them.
    pub(crate) fn form(&self) -> Form {
        match self {
            Shape::Circle { plane, .. } => Form::Circle { plane: Some((*plane).clone()) },
        }
    }
}

/// What `lit`'s body is the body of, read off the statements as written; `None` for a body that
/// is only a set.
pub(crate) fn shape<'a>(lit: &'a SetLit) -> Option<Shape<'a>> {
    let [a, b] = lit.body.as_slice() else { return None };
    // the one operand that is not the set's point, of a plain relation between two
    fn other<'s>(st: &'s Stmt, word: &str, bound: &str)
        -> Option<(&'s Ref, &'s crate::syntax::Written)> {
        let StmtKind::Relation(r) = &st.kind else { return None };
        let w = r.form.written()?;
        let plain = !r.claim && r.word.is_none() && r.along.is_none();
        if !plain || w.word.text != word || w.ops.len() != 2 {
            return None;
        }
        let is = |r: &Ref| r.path.is_empty() && r.root.text == bound;
        match (is(&w.ops[0]), is(&w.ops[1])) {
            (true, false) => Some((&w.ops[1], w)),
            (false, true) => Some((&w.ops[0], w)),
            _ => None,
        }
    }
    let bound = lit.bound.text.as_str();
    let circle = |on: &'a Stmt, at: &'a Stmt| {
        let (plane, w) = other(on, "coincident", bound)?;
        if !w.args.is_empty() {
            return None;
        }
        let (centre, d) = other(at, "distance", bound)?;
        let [crate::syntax::OpArg::Dim(text, span)] = d.args.as_slice() else { return None };
        let radius = Radius { text: text.as_str(), span: *span, word: &d.word };
        Some(Shape::Circle { plane, centre, radius })
    };
    circle(a, b).or_else(|| circle(b, a))
}

/// A set the flattener drew as an element, for the elaborator to judge: the key its element is
/// built under, and what its shape names, resolved where its body reads it.
#[derive(Clone, Debug)]
pub struct Lowered {
    pub key: String,
    pub form: Form,
}

/// What a lowering is judged by.
#[derive(Clone, Debug)]
pub enum Form {
    /// The plane the circle's point is put on — `None` where that names nothing a circle can be
    /// drawn in (a set, or nothing).
    Circle { plane: Option<Ref> },
}

impl Form {
    /// Every reference the judgment reads, for the flattener to make absolute.
    pub(crate) fn refs(&mut self) -> Vec<&mut Option<Ref>> {
        match self {
            Form::Circle { plane } => vec![plane],
        }
    }

    /// Whether a use by `word` is a use of the element: a point is put on a circle as on any —
    /// but a tangency reads the body's rows, whatever the set is drawn as.
    pub(crate) fn answers(&self, word: &str) -> bool {
        match self {
            Form::Circle { .. } => word == "coincident",
        }
    }
}

/// The sets drawn as elements that are none here, by key — each walked again as a set.  `ent`
/// resolves a reference, `key` finds what a key built.
pub(crate) fn refused(
    sk: &Sketch,
    lowered: &[Lowered],
    ent: impl Fn(&Ref) -> Option<EntRef>,
    key: impl Fn(&str) -> Option<EntRef>,
) -> BTreeSet<String> {
    let holds = |l: &Lowered| match &l.form {
        // the circle about a point drawn in its plane: `P` a line, or the centre standing off it,
        // and the body says something else, which the set itself says when walked as one
        Form::Circle { plane } => {
            let plane = plane.as_ref().and_then(&ent).filter(|e| e.kind == EntKind::Plane);
            let circle = key(&l.key).filter(|e| e.kind == EntKind::Circle);
            plane.zip(circle).is_some_and(|(plane, k)| {
                sk.plane_of(sk.circles[k.i()].center as usize) == Some(plane.i())
            })
        }
    };
    lowered.iter().filter(|l| !holds(l)).map(|l| l.key.clone()).collect()
}
