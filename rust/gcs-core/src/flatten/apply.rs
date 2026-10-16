//! **Applying a predicate** (#103): one routine for a relation word's use (§9.9) and a set's
//! (§6.21).  A predicate is a body over **parameters**, given where it is called (`Sphere(c, r:
//! 12mm)`, `above(d: 5mm)`), and **bound variables**, written where it is defined (`{ p | … }`,
//! `a above(d) b`) and filled where it is used (`q coincident ball`, `r above(d: 5mm) q`).  A
//! component instance is the case with none: its body is stated where it is written.
//!
//! A use is walked as an instance's body is — under a prefix of the use's own, so what the body
//! declares privately is made once per use — with each bound variable an alias of the operand the
//! use wrote, and the body's other names read in the scope the predicate closes over.  What it
//! makes is then **the use's**: every relation is stamped with the statement as written (its id,
//! its span, its claim, its placement), described as written (`Relation::word`), and every place
//! in it is the word's — so a fault in an expansion is said where the word is, and a dimension that
//! is a parameter keeps its argument's place, where its callout reads and edits it.  Declarations
//! stay the definition's, at the use's path.
//!
//! A use inside a body walked for its derivative (`Scope::twin`, `l tangent S`) is differentiated
//! too, at the same contact along the same line.

use super::*;
use crate::syntax::{Along, AlongBy, OpArg, Relation, Worded};

/// A predicate's use, as the statement wrote it.
pub(super) struct Use<'u> {
    pub st: &'u Stmt,
    pub rel: &'u Relation,
    pub path: &'u [PathStep],
    /// Where the statement is written: what the operands are read in.
    pub scope: &'u Scope,
    /// The word as written, where every place in the expansion goes.
    pub at: Span,
}

/// One walk of a predicate's body (`Walk::apply`): as itself, for only what it makes, or as its
/// derivative.
#[derive(Clone, Debug, Default)]
pub(super) enum Pass {
    #[default]
    Itself,
    Made,
    Along(Along),
}

impl From<Option<Along>> for Pass {
    fn from(a: Option<Along>) -> Pass {
        a.map_or(Pass::Itself, Pass::Along)
    }
}

/// One use's application in progress: the prefix its names are made under, the scope its body is
/// walked in, and where in the walk's output what it makes begins.
pub(super) struct Application {
    prefix: String,
    scope: Scope,
    from: usize,
    /// A set's (§6.21): what it makes is a relation in space (`Relation::space`).
    set: bool,
}

impl Application {
    /// The prefix the application's names are made under.
    pub(super) fn prefix(&self) -> &str {
        &self.prefix
    }

    /// The scope the body is walked in.
    pub(super) fn scope(&self) -> &Scope {
        &self.scope
    }

    /// The application a set's use makes: its body's relations are in space (§6.21).
    pub(super) fn of_set(mut self) -> Application {
        self.set = true;
        self
    }

    /// A number the body reads by a name only the walk writes (`#grow`), worked out where the
    /// use is.
    pub(super) fn give(&mut self, name: &str, value: Aff) {
        self.scope.vals.insert(name.to_string(), value);
    }
}

/// A reference standing for a name the application made under its prefix.
pub(super) fn local(text: impl Into<String>, span: Span) -> Ref {
    Ref { root: Name { text: text.into(), span }, path: Vec::new(), span }
}

impl<'a> Walk<'a> {
    /// An application begun for one use: the scope the predicate closes over, under a prefix of
    /// the use's own (`{where the use stands}#{statement}.0.`), turned with the use where it
    /// stands in a ring's copy and dressed in its classes.  What the walk makes from here on is
    /// the use's.
    pub(super) fn begin(&mut self, u: &Use, closure: &Scope) -> Application {
        self.begin_nth(u, closure, 0)
    }

    /// `begin`, the `n`th of several applications one use makes (`S1 tangent(at: m) S2`, one per
    /// set), each under a prefix of its own.  The statement is the one the use was written as —
    /// in a body, its own and not the outer use's it carries (`PredWalk::written`), so two sets used
    /// in one body are applied apart.
    pub(super) fn begin_nth(&mut self, u: &Use, closure: &Scope, n: usize) -> Application {
        let id = u.scope.pred.written.unwrap_or(u.st.id);
        let prefix = keys::application(u.scope.prefix(), id.0, n);
        let mut scope = closure.clone();
        scope.pred.written = None;
        scope.prefixes.insert(0, prefix.clone());
        scope.block.copies = true;
        scope.stamps.anonymous = true;
        scope.block.cyc = None;
        scope.pred.pass = Pass::Itself;
        scope.pred.held = u.scope.pred.held;
        scope.block.ring = u.scope.block.ring.clone();
        for c in u.scope.stamps.class.0.iter().chain(&u.rel.class.0) {
            if !scope.stamps.class.has(c) {
                scope.stamps.class.0.push(c.clone());
            }
        }
        Application { prefix, scope, from: self.out.len(), set: false }
    }

    /// A name of the body, under the application's prefix, standing for what the use wrote.
    pub(super) fn bind_to_use(&mut self, app: &Application, name: &str, r: Ref, u: &Use) {
        let key = format!("{}{name}", app.prefix);
        self.words.use_aliases.insert(key.clone());
        self.words.arguments.insert(key.clone());
        self.aliases.push((key, r, u.scope.clone()));
    }

    /// The derivative a use inherits — the line and contact of a body being walked for one,
    /// whether the use was written in it (`Scope::twin`) or set aside from it (`Relation::along`)
    /// — read again under the application's prefix.
    pub(super) fn inherited_twin(&mut self, app: &Application, u: &Use) -> Option<Along> {
        let t = u.rel.along.clone().or_else(|| u.scope.twin().cloned())?;
        self.bind_to_use(app, keys::ALONG_POINT, t.point, u);
        let toward = match t.toward {
            AlongBy::Line(l) => {
                self.bind_to_use(app, keys::LINE, l, u);
                AlongBy::Line(local(keys::LINE, u.at))
            }
            chart => chart,
        };
        Some(Along { point: local(keys::ALONG_POINT, u.at), toward, ..t })
    }

    /// The body walked once per entry of `walks` — as itself, for only what it makes, or as its derivative —
    /// under the use's path, and everything the application made stamped as the use's.  `dims`
    /// is where a dimension that is a parameter was given, by the parameter's name (`dims_at`).
    pub(super) fn apply(
        &mut self,
        u: &Use,
        app: Application,
        mut body: Vec<Stmt>,
        dims: &BTreeMap<String, Span>,
        walks: &[Pass],
        worded: Worded,
        depth: usize,
    ) {
        // the statement every relation made reads as: the use's own, or — a use inside another's
        // expansion — the outermost one, stamped over this when that one's application ends
        // the sets its relations come through (#212): those the use was made through, where an
        // outer application made it, then this use's own
        let own = worded.sets.iter().map(|(_, n)| n.clone());
        let through: Vec<String> =
            u.rel.word.as_ref().map_or(Vec::new(), |w| w.through.clone()).into_iter().chain(own).collect();
        let worded = u.rel.word.clone().unwrap_or(worded);
        let ops: Vec<Ref> = worded.ops.iter().enumerate().map(|(i, r)| {
            self.bind_to_use(&app, &keys::word_operand(i), r.clone(), u);
            local(keys::word_operand(i), r.span)
        }).collect();
        let stamp = Worded { ops, ..worded };
        let Application { mut scope, from, set, .. } = app;
        dims_at(&mut body, dims, u.at);
        let mut path = u.path.to_vec();
        path.push(PathStep::Instance(u.st.id));
        for pass in walks {
            scope.pred.pass = pass.clone();
            let mut vals = scope.vals.clone();
            self.body(&body, &scope, &mut vals, &path, depth + 1);
        }
        // where the callout sits is the statement's: the relation it made, or the one of them that
        // states a dimension
        let relations: Vec<usize> = (from..self.out.len())
            .filter(|&i| matches!(self.out[i].0.kind, StmtKind::Relation(_)))
            .collect();
        let dimensioned: Vec<usize> = relations.iter().copied().filter(|&i| match &self.out[i].0.kind {
            StmtKind::Relation(r) => r.form.written().is_some_and(|w| w.args.iter().any(|a| {
                matches!(a, OpArg::Dim(..) | OpArg::Named(_, crate::syntax::Arg::Dim { .. }))
            })),
            _ => false,
        }).collect();
        let placed = match (relations.as_slice(), dimensioned.as_slice()) {
            ([one], _) | (_, [one]) => Some(*one),
            _ => None,
        };
        // what the body declares (by its key under the use's prefix, read by its last name): under
        // a claim it is made all the same, and the `coincident`
        // that places it is how it is made — stated, never claimed (#365)
        let made: std::collections::BTreeSet<String> = self.out[from..].iter()
            .filter_map(|(st, _, _)| match &st.kind {
                StmtKind::Decl(d) => Some(d.name.key().text.clone()),
                _ => None,
            })
            .collect();
        let places = |r: &Relation| r.form.written().is_some_and(|w| {
            w.word.text == "coincident" && w.rates.is_empty()
                && w.ops.first().is_some_and(|o| o.path.is_empty() && made.iter().any(|m| {
                    m.rsplit('.').next() == Some(o.root.text.as_str())
                }))
        });
        // the sets a statement came through (#212): the use's, then those an inner use stamped
        let through_all = |inner: Vec<String>| through.iter().cloned().chain(inner).collect();
        for (i, (st, p, sc)) in self.out.iter_mut().enumerate().skip(from) {
            // an energy the word states is the use's statement, as its relations are (#121)
            let energy = match &mut st.kind {
                StmtKind::Measured(m) => {
                    m.through = through_all(std::mem::take(&mut m.through));
                    true
                }
                StmtKind::Minimize(_) => true,
                _ => false,
            };
            if energy {
                sc.pred.written.get_or_insert(st.id);
                st.id = u.st.id;
                st.span = u.st.span;
                *p = u.path.to_vec();
                continue;
            }
            let StmtKind::Relation(r) = &mut st.kind else { continue };
            // a set measured from states its point's equalities as bounds at their numbers, but
            // for a use of another set, whose own body is held when it is applied (#192)
            if sc.pred.held {
                if let Some(w) = r.form.written_mut().filter(|w| !super::sets::may_use(w)) {
                    w.held();
                }
            }
            sc.pred.written.get_or_insert(st.id);
            st.id = u.st.id;
            st.span = u.st.span;
            st.chained = u.st.chained;
            *p = u.path.to_vec();
            let inner = r.word.take().map_or(Vec::new(), |w| w.through);
            r.word = Some(Worded { through: through_all(inner), ..stamp.clone() });
            // in space, a set's body (§6.21), which no later stamp undoes
            r.space |= set;
            r.claim |= u.rel.claim && !places(r);
            r.initially |= u.rel.initially;
            if placed == Some(i) {
                r.place = u.rel.place;
                r.place_span = u.rel.place_span;
            }
            if let Some(w) = r.form.written_mut() {
                w.word.span = u.at;
                w.span = u.st.span;
                for a in w.args.iter_mut() {
                    match a {
                        OpArg::Named(n, _) => n.span = u.at,
                        OpArg::Slot { key, .. } => key.span = u.at,
                        OpArg::Vector { key, span, .. } => {
                            *span = u.at;
                            if let Some(k) = key {
                                k.span = u.at;
                            }
                        }
                        OpArg::Bound { span, .. } => *span = u.at,
                        OpArg::Ent(_) | OpArg::Dim(..) => {}
                    }
                }
            }
        }
    }
}

/// A body's dimensions put where the use is, before their texts are worked out: one that is a
/// parameter at the argument it was given (`dims`, by name), so its callout reads the argument
/// and editing it edits the argument, and every other at an empty place before the word — the
/// number is drawn.
fn dims_at(body: &mut [Stmt], dims: &BTreeMap<String, Span>, at: Span) {
    let nowhere = Span::at(at.lo as usize);
    for st in body {
        let StmtKind::Relation(r) = &mut st.kind else { continue };
        let Some(w) = r.form.written_mut() else { continue };
        for (text, span) in super::words::texts_mut(w) {
            *span = dims.get(text.trim()).copied().unwrap_or(nowhere);
        }
    }
}
