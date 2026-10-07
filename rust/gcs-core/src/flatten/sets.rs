//! **Sets** (§6.21): a shape written as the points that satisfy a predicate, `{ p | p
//! distance(r) c }`, and a family of them as a component whose body is one,
//! `component Sphere(center: point, r: Length) := { p | … }`.
//!
//! Making a set makes nothing: the walk records where it stands (`Site`) and moves on.  What a
//! set means is said where it is used, and a use is applied here — by the routine a relation word
//! is (`flatten::apply`, #103), the set's point its one bound variable — once every name is known
//! (a body is a set of statements, P2, so a use may stand before the set it names):
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
use super::apply::{local, Application, Use};
use crate::syntax::{Along, Chained, Relation, RelationForm, SetLit, Worded};

/// Where a set stands: its literal, the scope its body reads names in (an instance's, with its
/// formals bound, for a family's), and how deep the walk was there.  A use's expansion stands
/// where the use does, at the use's path.
pub(super) struct Site {
    lit: SetLit,
    scope: Scope,
    depth: usize,
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
        let (lit, depth, closure) = (site.lit.clone(), site.depth, site.scope.clone());
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
        let u = Use { st, rel, path, scope: sc, at };
        let app = self.begin(&u, &closure);
        let other = w.ops[1 - k].clone();
        let bound = lit.bound.text.clone();
        // a tangency: the body at a contact on the line, then linearised along it; else the body
        // at the point — linearised already where the use is in a body being linearised (a
        // tangency there was refused above)
        let walks = if tangent {
            self.bind_to_use(&app, "#line", other.clone(), &u);
            self.contact(&app, st, &bound, at, path);
            vec![None, Some(Along { point: local(&bound, at), line: local("#line", at) })]
        } else {
            self.bind_to_use(&app, &bound, other.clone(), &u);
            vec![self.inherited_twin(&app, &u)]
        };
        let worded = Worded {
            word: word.to_string(),
            ops: vec![other],
            args: String::new(),
            set: Some((k, self.set_written(&w.ops[k]))),
            span: at,
        };
        self.apply(&u, app, lit.body, &BTreeMap::new(), &walks, worded, depth);
    }

    /// A set as the statement wrote it: its name, or — written in place — its text, on one line.
    fn set_written(&self, r: &Ref) -> String {
        match self.prog.span_text(r.span).filter(|_| r.root.text.starts_with('#')) {
            Some(text) => text.split_whitespace().collect::<Vec<_>>().join(" "),
            None => written(r),
        }
    }

    /// `l tangent S`'s contact: a point standing in space, on the line, seeded at its middle —
    /// declared at the bound name under the use's prefix, so the body reads it as its point.
    fn contact(&mut self, app: &Application, st: &Stmt, bound: &str, at: Span, path: &[PathStep]) {
        let abs = format!("{}{bound}", app.prefix());
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
        let on = Relation::of(RelationForm::Written(crate::syntax::Written {
            word: Name { text: "coincident".to_string(), span: at },
            fixity: crate::constraints::Fixity::Infix,
            ops: vec![local(bound, at), line],
            args: Vec::new(),
            span: st.span,
        }));
        for kind in [StmtKind::Decl(decl), StmtKind::Relation(on)] {
            let made = Stmt { id: st.id, kind, span: st.span, chained: Chained::No };
            self.out.push((made, path.to_vec(), app.scope().clone()));
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
            other => return Some(other.bound_name().map_or_else(|| "a statement".into(), |n| n.text.clone())),
        }
    }
    None
}

/// Whether a written relation is one a set may be used by: `coincident` or `tangent` between
/// two operands.
fn may_use(w: &crate::syntax::Written) -> bool {
    matches!(w.word.text.as_str(), "coincident" | "tangent") && w.ops.len() == 2
}
