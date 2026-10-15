//! **A step** (#330): `step(u: s, dv: b) l` says where line `l`'s end stands, one ordinate at a
//! time — `du`, `dv` from the line's start, `u`, `v` from a frame's origin — and keeps the
//! ordinate it does not name, so `step(u: s)` is a run to station `s` level with where it began.
//! The frame is `from:`, or the view the line is drawn in, which the parser hands on from the
//! `in` block it stands in (`stamp_plane`).
//!
//! It is expanded where it stands, in the statement's own scope, into the relations it is short
//! for: an ordinate per ordinate named (`p distance(d, along: u) q`) and a level per one kept
//! (`l.p1 level(v) l.p2`).  Each is the statement's — its id, its span and the word as written
//! (`Relation::word`) — and each number keeps the place it was written at, where its callout
//! reads it and an edit rewrites it.

use super::*;
use crate::constraints::{Fixity, STEP_AXES};
use crate::syntax::{written_parts, Arg, OpArg, Relation, RelationForm, Worded, Written};

/// One ordinate a step names: from the frame's origin or from the line's start, as written.
struct Ordinate {
    absolute: bool,
    text: String,
    span: Span,
}

/// What a step's parentheses say: the ordinate it names along each axis of its frame, and the
/// frame — with whether the source named it, so it is read along its own axes by reference,
/// where the view a line is drawn in is read along the page's words.
struct Said {
    ords: [Option<Ordinate>; 2],
    frame: Ref,
    named: bool,
}

/// A fault in a step, and where it is said.
type Fault = (Code, Span, String);

/// `r.name`, where `r` is written.
fn field(r: &Ref, name: &str) -> Ref {
    let mut out = r.clone();
    out.path.push(Seg::Field(Name { text: name.to_string(), span: r.span }));
    out
}

/// A step's parentheses read, or the first thing they cannot mean.
fn said(w: &Written) -> Result<Said, Fault> {
    let mut ords: [Option<Ordinate>; 2] = [None, None];
    let mut frame: Option<(Ref, bool)> = None;
    for a in &w.args {
        match a {
            // the view the line is drawn in, handed on by the parser where none is written
            OpArg::Named(n, Arg::Ref(r)) if n.text == "from" => frame = Some((r.clone(), false)),
            OpArg::Named(n, Arg::Dim { text, span }) if n.text == "from" => {
                let r = crate::syntax::reference_at(text, span.lo as usize).ok_or_else(|| {
                    let m = format!("`from:` names the plane a step is measured in, not `{text}`");
                    (Code::E040, *span, m)
                })?;
                frame = Some((r, true));
            }
            OpArg::Named(n, Arg::Dim { text, span }) => {
                let label = |(k, &(abs, rel)): (usize, &(&str, &str))| {
                    (n.text == abs).then_some((k, true)).or((n.text == rel).then_some((k, false)))
                };
                let Some((k, absolute)) = STEP_AXES.iter().enumerate().find_map(label) else {
                    let m = format!("`step` takes `u`, `v`, `du`, `dv` and `from`, not `{}`",
                                    n.text);
                    return Err((Code::E040, n.span, m));
                };
                if ords[k].is_some() {
                    let (abs, rel) = STEP_AXES[k];
                    let m = format!("`{}` says again where the end stands along `{abs}`: a step \
                                     says it once, from the frame (`{abs}`) or from the line's \
                                     start (`{rel}`)", n.text);
                    return Err((Code::E040, n.span, m));
                }
                ords[k] = Some(Ordinate { absolute, text: text.clone(), span: *span });
            }
            OpArg::Dim(text, span) => {
                let m = format!("a step names each ordinate: `step(du: {text})` from the line's \
                                 start, `step(u: {text})` from the frame's origin");
                return Err((Code::E004, *span, m));
            }
            _ => {
                let m = "`step` takes its ordinates by label".into();
                return Err((Code::E040, w.word.span, m));
            }
        }
    }
    if ords.iter().all(Option::is_none) {
        let m = "a step says where the line's end stands: `step(du: …)` from its start, \
                 `step(u: …)` from the frame's origin";
        return Err((Code::E040, w.word.span, m.into()));
    }
    let Some((frame, named)) = frame else {
        let m = "a step is measured in the view its line is drawn in, and this one stands in no \
                 `in` block: write it in one, or name the frame, `from: P`";
        return Err((Code::E040, w.word.span, m.into()));
    };
    Ok(Said { ords, frame, named })
}

impl Walk<'_> {
    /// `true` where the relation is a step, whatever became of it: its relations stated, or what
    /// it cannot be said once at what was written.
    pub(super) fn walk_step(
        &mut self,
        rel: &Relation,
        st: &Stmt,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
        path: &[PathStep],
    ) -> bool {
        let Some(w) = rel.form.written().filter(|w| w.is_step()) else { return false };
        let Some(line) = w.ops.first() else { return true };
        let Said { ords, frame, named } = match said(w) {
            Ok(s) => s,
            Err((code, span, m)) => {
                self.once(code, span, m);
                return true;
            }
        };
        let at = w.word.span;
        let name = |text: &str| Name { text: text.to_string(), span: at };
        let (p1, p2, origin) = (field(line, "p1"), field(line, "p2"), field(&frame, "origin"));
        // what the statement is described as: the word as written, with no frame it was handed
        let shown: Vec<OpArg> =
            w.args.iter().filter(|a| !matches!(a, OpArg::Named(_, Arg::Ref(_)))).cloned().collect();
        let worded = Worded {
            word: w.word.text.clone(),
            ops: w.ops.clone(),
            args: written_parts(&shown, false).0.join(", "),
            sets: Vec::new(),
            through: Vec::new(),
            span: at,
        };
        let mut placed = false;
        for (k, ord) in ords.iter().enumerate() {
            // the axis: the named frame's own, or the page's word for the view's, which a level
            // takes where an entity stands and a distance as its `along:` word
            let axis = STEP_AXES[k].0;
            let own = named.then(|| field(&frame, axis));
            let dir = OpArg::Ent(own.clone().unwrap_or(Ref { span: at, ..Ref::new(axis) }));
            let along = own.map_or(Arg::Word(axis.to_string()), Arg::Ref);
            let relation = |word: &str, from: Ref, args: Vec<OpArg>| Written {
                word: name(word),
                fixity: Fixity::Infix,
                ops: vec![from, p2.clone()],
                args,
                span: w.span,
                rates: Vec::new(),
            };
            let made = match ord {
                None => relation("level", p1.clone(), vec![dir]),
                Some(o) => {
                    let from = if o.absolute { origin.clone() } else { p1.clone() };
                    // back onto the frame's axis, or none along it: a level — a relation, which
                    // states no number, as `distance(0, along: …)` would (`written_zero`)
                    match crate::program::written_zero(&o.text, self.units) {
                        true => relation("level", from, vec![dir]),
                        false => relation("distance", from, vec![
                            OpArg::Dim(o.text.clone(), o.span),
                            OpArg::Named(name("along"), along),
                        ]),
                    }
                }
            };
            // where a callout was dragged to is the statement's first dimension's
            let place = made.word.text == "distance" && !placed;
            placed |= place;
            let r = Relation {
                form: RelationForm::Written(made),
                place: if place { rel.place } else { None },
                place_span: if place { rel.place_span } else { Span::default() },
                word: Some(worded.clone()),
                ..rel.clone()
            };
            let Some(r2) = self.settle_relation(&r, vals, scope) else { continue };
            self.emit(StmtKind::Relation(r2), st, scope, path);
        }
        true
    }
}
