//! **Sets** (§6.21): a shape written as the points that satisfy a predicate, `{ p | p
//! distance(r) c }`, and a family of them as a component whose body is one,
//! `component Sphere(center: point, r: Length) := { p | … }`.
//!
//! Making a set makes nothing: the walk records where it stands (`Site`) and moves on.  What a
//! set means is said where it is used, and a use is expanded here, once every name is known (a
//! body is a set of statements, P2, so a use may stand before the set it names):
//!
//! * `q coincident S` is the body with `q` for the bound point — walked like an instance's,
//!   under a prefix of the use's own, so what the body makes privately is made once per use;
//! * `l tangent S` is a contact point declared under that prefix, on `l`, in `S` — the body at
//!   the contact — and the body walked a second time, each relation stated as its
//!   **linearisation** at the contact along `l` (`Scope::twin`, `syntax::Along`): `l`'s
//!   direction is in the set's tangent space there.  A body that makes points of its own would
//!   need their tangents too, so a tangency to one is refused.
//!
//! Every relation a use makes reads as the statement wrote it (`Relation::word`, the set by
//! name), so a culprit is `l tangent shaft` and not the distance inside the cylinder.

use super::*;
use crate::program::public_path;
use crate::syntax::{Along, Chained, Relation, RelationForm, SetLit, Worded};

/// Where a set stands: its literal, the scope its body reads names in (an instance's, with its
/// formals bound, for a family's), and how deep the walk was there.  A use's expansion stands
/// where the use does, at the use's path.
pub(super) struct Site {
    lit: SetLit,
    scope: Scope,
    depth: usize,
}

/// A reference standing for a name the expansion made under its prefix.
fn local(text: impl Into<String>, span: Span) -> Ref {
    Ref { root: Name { text: text.into(), span }, path: Vec::new(), span }
}

impl<'a> Walk<'a> {
    /// A set, made where the walk met it (§6.21).
    pub(super) fn set_made(&mut self, abs: String, lit: &SetLit, scope: &Scope, depth: usize) {
        self.sets.insert(abs, Site { lit: lit.clone(), scope: scope.clone(), depth });
    }

    /// A statement's use of a set: the relation, which operand names the set and the sets its
    /// operands name, by absolute name — `None` for a statement naming none.  Only `coincident`
    /// and `tangent` between two operands may use one.
    fn set_use<'s>(
        &self,
        st: &'s Stmt,
        sc: &Scope,
        alias: &BTreeMap<String, String>,
    ) -> Option<(&'s Relation, Vec<(usize, String)>)> {
        let StmtKind::Relation(rel) = &st.kind else { return None };
        let w = rel.form.written().filter(|w| may_use(w))?;
        let found: Vec<(usize, String)> = w
            .ops
            .iter()
            .enumerate()
            .filter_map(|(i, r)| match lookup_raw(r, sc, &self.names, alias, self.units) {
                Some((abs, rest)) if rest.is_empty() && self.sets.contains_key(&abs) => Some((i, abs)),
                _ => None,
            })
            .collect();
        (!found.is_empty()).then_some((rel, found))
    }

    /// Expand every use of a set, each where the walk met it, until none is left.  A use's
    /// expansion may hold a use of another set (a set defined over one), so this runs in rounds;
    /// one still standing after `MAX_DEPTH` of them is a set defined in terms of itself.
    pub(super) fn expand_sets(&mut self) {
        if self.sets.is_empty() {
            return;
        }
        for round in 0..=MAX_DEPTH {
            // no word a set is used by, no table to build: the last round is usually this
            let words = |st: &Stmt| match &st.kind {
                StmtKind::Relation(r) => r.form.written().is_some_and(may_use),
                _ => false,
            };
            if !self.out.iter().any(|(st, _, _)| words(st)) {
                return;
            }
            let alias = self.alias_table();
            let out = std::mem::take(&mut self.out);
            let mut next = Vec::with_capacity(out.len());
            let mut any = false;
            let mut rest = out.into_iter();
            while let Some((st, path, sc)) = rest.next() {
                let Some((rel, found)) = self.set_use(&st, &sc, &alias) else {
                    next.push((st, path, sc));
                    continue;
                };
                any = true;
                if round == MAX_DEPTH {
                    let (_, abs) = &found[0];
                    let m = format!("`{}` is defined in terms of itself", public_path(abs));
                    self.once(Code::E003, st.span, m);
                    continue;
                }
                if found.len() > 1 {
                    let m = "`coincident` and `tangent` relate a set to a point or a line, not to \
                             another set";
                    self.once(Code::E040, st.span, m);
                    continue;
                }
                self.held = next.len() + rest.len();
                let (k, abs) = found.into_iter().next().unwrap();
                self.expand_use(&st, rel, &path, &sc, k, &abs);
                next.append(&mut self.out);
                self.held = 0;
            }
            self.out = next;
            if !any {
                return;
            }
        }
    }

    /// One use of the set `abs`, operand `k` of the statement `st` (§6.21).
    fn expand_use(
        &mut self,
        st: &Stmt,
        rel: &Relation,
        path: &[PathStep],
        sc: &Scope,
        k: usize,
        abs: &str,
    ) {
        let Some(w) = rel.form.written() else { return };
        let word = w.word.text.as_str();
        let at = w.word.span;
        let name = public_path(abs);
        if !w.args.is_empty() {
            let m = format!("`{word}` with a set takes nothing in its parentheses");
            self.once(Code::E040, at, m);
            return;
        }
        let tangent = word == "tangent";
        if tangent && rel.claim {
            let m = format!("a tangency to a set is stated, not claimed: the contact on it is \
                             found by the solve (`l tangent {name}`)");
            self.once(Code::E040, at, m);
            return;
        }
        if tangent && rel.along.is_some() {
            let m = format!("a set's body may put its point on another set, not a line \
                             tangent to one (`{word} {name}`)");
            self.once(Code::E040, at, m);
            return;
        }
        let site = &self.sets[abs];
        let (lit, depth) = (site.lit.clone(), site.depth);
        let mut scope = site.scope.clone();
        if tangent {
            if let Some(what) = makes_points(&lit) {
                let m = format!(
                    "`{word} {name}` is the set's body at a contact and its linearisation \
                     along the line, and `{what}` in its body is geometry of its own, whose \
                     motion along the set the linearisation would need as well"
                );
                self.once(Code::E040, at, m);
                return;
            }
        }
        // a prefix of the use's own: what the body makes privately is made once per use, and a
        // use's two walks — the body and its linearisation — share it
        let prefix = format!("{}#{}.0.", sc.prefix(), st.id.0);
        scope.prefixes.insert(0, prefix.clone());
        scope.copies = true;
        scope.anonymous = true;
        scope.cyc = None;
        // turned with the use, where it stands in a ring's copy; dressed as the use is
        scope.ring = sc.ring.clone();
        for c in sc.in_class.0.iter().chain(&rel.class.0) {
            if !scope.in_class.has(c) {
                scope.in_class.0.push(c.clone());
            }
        }
        let other = w.ops[1 - k].clone();
        let bound = lit.bound.text.clone();
        let mut sub_path = path.to_vec();
        sub_path.push(PathStep::Instance(st.id));
        // the statement every relation of the use reads as: the use's own, or — a use inside
        // a set's body — the one that body was expanded for
        let worded = match rel.word.as_ref().filter(|w| w.set.is_some()) {
            Some(outer) => outer.clone(),
            None => Worded {
                word: word.to_string(),
                ops: vec![other.clone()],
                args: String::new(),
                set: Some((k, written(&w.ops[k]))),
                span: at,
            },
        };
        let from = self.out.len();
        // the line a tangency is taken along: this one's, or — a use inside a body walked for its
        // linearisation — that body's, at the same contact
        let line = match (&rel.along, tangent) {
            (Some(along), _) => Some(along.line.clone()),
            (None, true) => Some(other.clone()),
            (None, false) => None,
        };
        if let Some(l) = line {
            self.aliases.push((format!("{prefix}#line"), l, sc.clone()));
        }
        match tangent {
            true => self.contact(st, &prefix, &bound, at, &scope, &sub_path),
            false => self.aliases.push((format!("{prefix}{bound}"), other, sc.clone())),
        }
        let twin = Along { point: local(&bound, at), line: local("#line", at) };
        // the body, at the point — linearised already where the use is — and a tangency's body
        // a second time, linearised
        let walks = match (&rel.along, tangent) {
            (Some(_), _) => vec![Some(twin)],
            (None, true) => vec![None, Some(twin)],
            (None, false) => vec![None],
        };
        for twin in walks {
            scope.twin = twin;
            let mut vals = scope.vals.clone();
            self.body(&lit.body, &scope, &mut vals, &sub_path, depth + 1);
        }
        // every relation made reads as the statement, its operands named where it was written
        let ops: Vec<Ref> = worded.ops.iter().enumerate().map(|(i, r)| {
            self.aliases.push((format!("{prefix}#word.{i}"), r.clone(), sc.clone()));
            local(format!("#word.{i}"), r.span)
        }).collect();
        let stamp = Worded { ops, ..worded };
        for (made, _, _) in self.out[from..].iter_mut() {
            if let StmtKind::Relation(r) = &mut made.kind {
                r.word = Some(stamp.clone());
                r.claim |= rel.claim;
            }
        }
    }

    /// `l tangent S`'s contact: a point standing in space, on the line, seeded at its middle —
    /// declared at the bound name under the use's prefix, so the body reads it as its point.
    fn contact(
        &mut self,
        st: &Stmt,
        prefix: &str,
        bound: &str,
        at: Span,
        scope: &Scope,
        path: &[PathStep],
    ) {
        let abs = format!("{prefix}{bound}");
        self.names.insert(abs.clone());
        let line = local("#line", at);
        let end = |e: &str| {
            let mut r = line.clone();
            r.path.push(Seg::Field(Name { text: e.to_string(), span: at }));
            r
        };
        let seed_at = crate::syntax::AtRef {
            what: end("p1"),
            bearing: None,
            toward: Some(end("p2")),
            along: None,
            by: Some(("0.5".to_string(), at)),
            turn: None,
            x: None,
            y: None,
        };
        let key = Name { text: abs, span: Span::new(at.lo as usize, at.lo as usize) };
        let decl = Decl::point(crate::syntax::DeclName::Key(key), [0.0; 2], Some(seed_at));
        let on = Relation {
            form: RelationForm::Written(crate::syntax::Written {
                word: Name { text: "coincident".to_string(), span: at },
                fixity: crate::constraints::Fixity::Infix,
                ops: vec![local(bound, at), line],
                args: Vec::new(),
                span: st.span,
            }),
            place: None,
            place_span: Span::default(),
            claim: false,
            class: Default::default(),
            class_span: Span::default(),
            word: None,
            along: None,
        };
        for kind in [StmtKind::Decl(decl), StmtKind::Relation(on)] {
            let made = Stmt { id: st.id, kind, span: st.span, chained: Chained::No };
            self.out.push((made, path.to_vec(), scope.clone()));
        }
    }
}

/// What in a set's body is geometry of its own — a declaration naming no points it was given,
/// or anything that makes some — where a tangency would need its motion too; `None` where the
/// body only relates the bound point and what the set was given (a line between two of them is
/// no geometry of its own: it moves as its ends do).
fn makes_points(lit: &SetLit) -> Option<String> {
    for st in &lit.body {
        match &st.kind {
            StmtKind::Relation(_) | StmtKind::Param(_) | StmtKind::Group(_) => {}
            StmtKind::Decl(d)
                if d.kind == crate::model::EntKind::Line
                    && d.children.iter().all(|g| !g.is_empty() && g.iter().all(|k| matches!(k, Kid::Ref(_)))) => {}
            StmtKind::Decl(d) => return Some(d.name.key().text.clone()),
            other => return other.bound_name().map(|n| n.text.clone()).or(Some("a statement".into())),
        }
    }
    None
}

/// Whether a written relation is one a set may be used by: `coincident` or `tangent` between
/// two operands.
fn may_use(w: &crate::syntax::Written) -> bool {
    matches!(w.word.text.as_str(), "coincident" | "tangent") && w.ops.len() == 2
}
