//! **A step** (#330): `step(u: s, dv: b) l` says where line `l`'s end stands, one ordinate at a
//! time — `du`, `dv` from the line's start, `u`, `v` from a frame's origin — and keeps the
//! ordinate it does not name, so `step(u: s)` is a run to station `s` level with where it began.
//! The frame is `from:`, or the view the line is drawn in, which the parser hands on from the
//! `in` block it stands in (`stamp_plane`).
//!
//! It is expanded where it stands, in the statement's own scope, into the relations it is short
//! for: an ordinate per ordinate named (`p distance(d, along: u) q`) and a level per one kept
//! (`l.p1 level(up) l.p2`).  Each is the statement's — its id, its span and the word as written
//! (`Relation::word`) — and each number keeps the place it was written at, where its callout
//! reads it and an edit rewrites it.

use super::*;
use crate::constraints::{step_word, Fixity};
use crate::syntax::{written_parts, Arg, OpArg, Relation, RelationForm, Worded, Written};

/// The two axes of a step's frame: the label of each ordinate from the frame's origin, the one
/// from the line's start, and the page's word for the direction a level along it is written with.
const AXES: [(&str, &str, &str); 2] = [("u", "du", "right"), ("v", "dv", "up")];

/// One ordinate a step names: from the frame's origin or from the line's start, as written.
struct Ordinate {
    absolute: bool,
    text: String,
    span: Span,
}

/// `r.name`, where `r` is written.
fn field(r: &Ref, name: &str) -> Ref {
    let mut out = r.clone();
    out.path.push(Seg::Field(Name { text: name.to_string(), span: r.span }));
    out
}

impl Walk<'_> {
    /// `true` where the relation is a step, whatever became of it: its relations stated, or what
    /// it cannot be said once at the word.
    pub(super) fn walk_step(
        &mut self,
        rel: &Relation,
        st: &Stmt,
        scope: &Scope,
        vals: &BTreeMap<String, Aff>,
        path: &[PathStep],
    ) -> bool {
        let Some(w) = rel.form.written() else { return false };
        if w.fixity != Fixity::Prefix || !step_word(&w.word.text) {
            return false;
        }
        let at = w.word.span;
        let Some(line) = w.ops.first() else { return true };
        let mut ords: [Option<Ordinate>; 2] = [None, None];
        // the frame, and whether the source wrote it: a written one is read along its own axes
        // by reference, the view a line is drawn in by the page's words
        let mut frame: Option<(Ref, bool)> = None;
        for a in &w.args {
            match a {
                OpArg::Named(n, Arg::Ref(r)) if n.text == "from" => {
                    frame.get_or_insert((r.clone(), false));
                }
                OpArg::Named(n, Arg::Dim { text, span }) if n.text == "from" => {
                    match crate::syntax::reference_at(text, span.lo as usize) {
                        Some(r) => frame = Some((r, true)),
                        None => {
                            let m = format!("`from:` names the plane a step is measured in, not \
                                             `{text}`");
                            self.once(Code::E040, *span, m);
                            return true;
                        }
                    }
                }
                OpArg::Named(n, Arg::Dim { text, span }) => {
                    let Some((k, absolute)) = AXES.iter().enumerate().find_map(|(k, ax)| {
                        (n.text == ax.0).then_some((k, true))
                            .or_else(|| (n.text == ax.1).then_some((k, false)))
                    }) else {
                        let m = format!("`step` takes `u`, `v`, `du`, `dv` and `from`, not `{}`",
                                        n.text);
                        self.once(Code::E040, n.span, m);
                        return true;
                    };
                    if ords[k].is_some() {
                        let (abs, rel) = (AXES[k].0, AXES[k].1);
                        let m = format!("`{}` says again where the end stands along `{abs}`: a \
                                         step says it once, from the frame (`{abs}`) or from the \
                                         line's start (`{rel}`)", n.text);
                        self.once(Code::E040, n.span, m);
                        return true;
                    }
                    ords[k] = Some(Ordinate { absolute, text: text.clone(), span: *span });
                }
                OpArg::Dim(text, span) => {
                    let m = format!("a step names each ordinate: `step(du: {text})` from the \
                                     line's start, `step(u: {text})` from the frame's origin");
                    self.once(Code::E004, *span, m);
                    return true;
                }
                _ => {
                    self.once(Code::E040, at, "`step` takes its ordinates by label");
                    return true;
                }
            }
        }
        if ords.iter().all(Option::is_none) {
            let m = "a step says where the line's end stands: `step(du: …)` from its start, \
                     `step(u: …)` from the frame's origin";
            self.once(Code::E040, at, m);
            return true;
        }
        let Some((frame, written)) = frame else {
            let m = "a step is measured in the view its line is drawn in, and this one stands in \
                     no `in` block: write it in one, or name the frame, `from: P`";
            self.once(Code::E040, at, m);
            return true;
        };
        let name = |text: &str| Name { text: text.to_string(), span: at };
        let (p1, p2) = (field(line, "p1"), field(line, "p2"));
        // what the statement is described as: the word as written, with no frame it was handed
        let shown: Vec<OpArg> =
            w.args.iter().filter(|a| !matches!(a, OpArg::Named(_, Arg::Ref(_)))).cloned().collect();
        let worded = Worded {
            word: w.word.text.clone(),
            ops: w.ops.clone(),
            args: written_parts(&shown).0.join(", "),
            sets: Vec::new(),
            through: Vec::new(),
            span: at,
        };
        let mut placed = false;
        for (k, ord) in ords.iter().enumerate() {
            let (axis, _, kept) = AXES[k];
            let along = match written {
                true => Arg::Ref(field(&frame, axis)),
                false => Arg::Word(axis.to_string()),
            };
            // the direction a level along this axis is written with
            let dir = OpArg::Ent(match written {
                true => field(&frame, axis),
                false => Ref { span: at, ..Ref::new(kept) },
            });
            let level = |from: Ref| Written {
                word: name("level"),
                fixity: Fixity::Infix,
                ops: vec![from, p2.clone()],
                args: vec![dir.clone()],
                span: w.span,
            };
            let made = match ord {
                // a step back onto the frame's axis, or none along it, is a level — a relation,
                // which states no number (`written_zero`), as `distance(0, along: …)` would
                Some(o) if crate::program::written_zero(&o.text, self.units) => {
                    level(if o.absolute { field(&frame, "origin") } else { p1.clone() })
                }
                Some(o) => {
                    let from = if o.absolute { field(&frame, "origin") } else { p1.clone() };
                    Written {
                        word: name("distance"),
                        fixity: Fixity::Infix,
                        ops: vec![from, p2.clone()],
                        args: vec![
                            OpArg::Dim(o.text.clone(), o.span),
                            OpArg::Named(name("along"), along),
                        ],
                        span: w.span,
                    }
                }
                None => level(p1.clone()),
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
