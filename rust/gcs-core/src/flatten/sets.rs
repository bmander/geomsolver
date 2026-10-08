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
//!   **derivative** at the contact along `l` (`Scope::twin`, `syntax::Along`): `l`'s direction
//!   is in the set's tangent space there.  What the body made of its own moves with the contact
//!   by a tangent unknown of its own (`model::Dual`), solved with the rest;
//! * `S1 tangent(at: m) S2` is each body walked for what it makes and then as its derivative
//!   along two directions solved for at `m`, shared by both: their tangent spaces there are one.
//!
//! Every relation a use makes reads as the statement wrote it (`Relation::word`, the set by
//! name), so a culprit is `l tangent shaft` and not the distance inside the cylinder.

use super::*;
use crate::program::public_path;
use super::apply::{local, Application, Pass, Use};
use crate::syntax::{Along, AlongBy, Chained, DeclName, Relation, RelationForm, SetLit, Worded};

/// Where a set stands: its literal, the scope its body reads names in (an instance's, with its
/// formals bound, for a family's), and how deep the walk was there.  A use's expansion stands
/// where the use does, at the use's path.
pub(super) struct Site {
    lit: SetLit,
    scope: Scope,
    depth: usize,
}

/// Where the walk made a set: the statement that made it — the set written in place, or an
/// instance of a family — at its path, what it is called there, and the scope that statement
/// stands in.
pub(super) struct Made<'m> {
    pub st: &'m Stmt,
    pub path: &'m [PathStep],
    /// The name as written, before the walk's prefix: the set's, or the instance's.
    pub name: DeclName,
    /// An instance of a family, whose body's numbers are its formals and not the document's.
    pub family: bool,
    pub outer: &'m Scope,
}

/// A set made a circle (#105): the plane its body put its point on, as written, and the scope
/// that is read in — the elaborator's to judge, since only it knows what the name is.
pub(super) struct Round {
    plane: Ref,
    scope: Scope,
}

/// A set the walk made a circle, for the elaborator to judge (#105): the circle's key, and the
/// plane its body puts its point on — `None` where that names nothing a circle can be drawn in.
#[derive(Clone, Debug)]
pub struct SetCircle {
    pub key: String,
    pub plane: Option<Ref>,
}

impl<'a> Walk<'a> {
    /// A set, made where the walk met it (§6.21) — and where its body is a circle's, the circle.
    pub(super) fn set_made(
        &mut self,
        abs: String,
        lit: &SetLit,
        scope: &Scope,
        depth: usize,
        made: Made,
    ) {
        self.sets.insert(abs.clone(), Site { lit: lit.clone(), scope: scope.clone(), depth });
        // a set made inside a predicate's use is made once per use, and is that use's: no
        // drawing of its own
        let applied = self.applying_sets || !self.applying_words.is_empty();
        if self.sym.is_some() || applied || !matches!(scope.pass, Pass::Itself)
            || self.plain.contains(&abs)
        {
            return;
        }
        if let Some((plane, centre, radius)) = round(lit) {
            self.make_round(abs, plane, centre, radius, scope, made);
        }
    }

    /// **A set whose body is a circle's is the circle** (#105, §6.21): `{ p | p coincident P; p
    /// distance(r) o }` is drawn as `circle(center: o)` with `radius(r)` on it, so it solves,
    /// drags and is dimensioned as one, and a point is put on it as on any circle.  The circle is
    /// what the set's statement made, under the set's name; its radius is the body's distance,
    /// drawn where the body writes it — or, for a family's instance, where the call gives it.
    /// Whether `P` is a plane and `o` a point drawn in it is the elaborator's question
    /// (`SetCircle`): where either is not, the set is walked again as a set.
    fn make_round(
        &mut self,
        abs: String,
        plane: Ref,
        centre: Ref,
        radius: (String, Span, Name),
        sc: &Scope,
        made: Made,
    ) {
        let (text, span, word) = radius;
        let Made { st, path, name, family, outer } = made;
        let seed = value_aff(&text, &sc.vals, self.units).ok().and_then(|a| a.number())
            .filter(|r| r.is_finite() && *r > 0.0);
        // a set written in place, or an unnamed call, is named nothing the source can say
        let name = if outer.anonymous || name.key().text.starts_with('#') {
            let at = name.span().lo as usize;
            DeclName::Key(Name { text: abs.clone(), span: Span::new(at, at) })
        } else {
            name.prefixed(abs.clone(), outer.copies)
        };
        let mut d = Decl::point(name, [0.0; 2], None);
        d.kind = crate::model::EntKind::Circle;
        d.children = vec![vec![Kid::Ref(centre)]];
        d.seed = vec![seed.unwrap_or(1.0)];
        d.seed_text = vec![None];
        self.names.insert(abs.clone());
        let decl = Stmt { id: st.id, kind: StmtKind::Decl(d), span: st.span, chained: Chained::No };
        self.out.push((decl, path.to_vec(), sc.clone()));
        // a family's number is the call's: drawn and edited where the call gives the formal the
        // body reads, else drawn as what it comes to
        let span = match &st.kind {
            _ if !family => span,
            StmtKind::Instance(inst) => inst.args.iter().find_map(|a| match (&a.label, &a.value) {
                (Some(l), crate::syntax::InstVal::Expr(t)) if l.text == text.trim() => {
                    Some(Span::new(a.span.hi as usize - t.len(), a.span.hi as usize))
                }
                _ => None,
            }).unwrap_or(Span::new(st.span.lo as usize, st.span.lo as usize)),
            _ => Span::new(st.span.lo as usize, st.span.lo as usize),
        };
        let on = Ref {
            root: Name { text: abs.clone(), span: word.span },
            path: Vec::new(),
            span: word.span,
        };
        let radius = Relation::of(RelationForm::Written(crate::syntax::Written {
            word: Name { text: "radius".to_string(), span: word.span },
            fixity: crate::constraints::Fixity::Prefix,
            ops: vec![on],
            args: vec![crate::syntax::OpArg::Dim(text, span)],
            span: st.span,
        }));
        if let Some(r) = self.settle_relation(&radius, &sc.vals, sc) {
            // the circle by its absolute name, which the body's scope may be closed to
            let named = Scope { prefixes: vec![String::new()], closed: false, ..sc.clone() };
            let kind = StmtKind::Relation(r);
            let rel = Stmt { id: st.id, kind, span: st.span, chained: Chained::No };
            self.out.push((rel, path.to_vec(), named));
        }
        self.rounds.insert(abs, Round { plane, scope: sc.clone() });
    }

    /// Every set made a circle, its plane resolved where its body reads it (`SetCircle`).
    pub(super) fn set_circles(&self, alias: &BTreeMap<String, String>) -> Vec<SetCircle> {
        self.rounds.iter().map(|(key, round)| {
            let plane = lookup_raw(&round.plane, &round.scope, &self.names, alias, self.units)
                .filter(|(abs, rest)| !(rest.is_empty() && self.sets.contains_key(abs)))
                .map(|(abs, rest)| Ref {
                    root: Name { text: abs, span: round.plane.span },
                    path: rest.into_iter().map(|f| Seg::Field(Name::new(f))).collect(),
                    span: round.plane.span,
                });
            SetCircle { key: key.clone(), plane }
        }).collect()
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
        // a set made a circle is one, and a point is put on it as on any circle (#105) — but
        // for a derivative, which reads the body's rows
        let circle = matches!(found.as_slice(), [(_, abs)] if self.rounds.contains_key(abs))
            && w.word.text == "coincident" && rel.along.is_none() && sc.twin().is_none();
        (!found.is_empty() && !circle).then_some((rel, found))
    }

    /// Expand every use of a set, each where the walk met it, until none is left.  A use's
    /// expansion may hold a use of another set (a set defined over one), so this runs in rounds;
    /// one still standing after `MAX_DEPTH` of them is a set defined in terms of itself.
    pub(super) fn expand_sets(&mut self) {
        if self.sets.is_empty() {
            return;
        }
        self.applying_sets = true;
        self.expand_set_uses();
        self.applying_sets = false;
    }

    /// `expand_sets`' rounds, while every set made is made inside a use.
    fn expand_set_uses(&mut self) {
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
                let pair = found.len() == 2
                    && rel.form.written().is_some_and(|w| w.word.text == "tangent");
                if found.len() > 1 && !pair {
                    let m = "`coincident` relates a set to a point, and `tangent` a set to a line \
                             or to another set at a point";
                    self.once(Code::E040, st.span, m);
                    continue;
                }
                self.held = next.len() + rest.len();
                if pair {
                    self.expand_pair(&st, rel, &path, &sc, &found);
                } else {
                    let (k, abs) = found.into_iter().next().unwrap();
                    self.expand_use(&st, rel, &path, &sc, k, &abs);
                }
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
        let u = Use { st, rel, path, scope: sc, at };
        let app = self.begin(&u, &closure);
        let other = w.ops[1 - k].clone();
        let bound = lit.bound.text.clone();
        // a tangency: the body at a contact on the line, then differentiated along it; else the
        // body at the point — differentiated already where the use is in a body being
        // differentiated (a tangency there was refused above)
        let walks = if tangent {
            self.bind_to_use(&app, "#line", other.clone(), &u);
            self.contact(&app, st, &bound, at, path);
            let toward = AlongBy::Line(local("#line", at));
            let key = app.prefix().to_string();
            let made = Some(key.clone());
            vec![Pass::Itself, Pass::Along(Along { point: local(&bound, at), toward, key, made })]
        } else {
            self.bind_to_use(&app, &bound, other.clone(), &u);
            vec![self.inherited_twin(&app, &u).into()]
        };
        let worded = Worded {
            word: word.to_string(),
            ops: vec![other],
            args: String::new(),
            sets: vec![(k, self.set_written(&w.ops[k]))],
            span: at,
        };
        self.apply(&u, app, lit.body, &BTreeMap::new(), &walks, worded, depth);
    }

    /// Two sets tangent at a point, `S1 tangent(at: m) S2` (§6.21): their tangent spaces at `m`
    /// are one — every direction along the first there is along the second.  Two directions span
    /// the first's, each solved for in a chart (`model::Toward::Chart`), and each set's body is
    /// stated as its derivative along both at `m`: four rows over two unknowns for two surfaces.
    /// `m` on each is stated beside it (`m coincident S1`); the word walks each body only for
    /// what it makes, so a body with geometry of its own, which the incidence would place, is
    /// refused.
    fn expand_pair(
        &mut self,
        st: &Stmt,
        rel: &Relation,
        path: &[PathStep],
        sc: &Scope,
        found: &[(usize, String)],
    ) {
        let Some(w) = rel.form.written() else { return };
        let at = w.word.span;
        let names: Vec<String> = found.iter().map(|(k, _)| written(&w.ops[*k])).collect();
        let point = match w.args.as_slice() {
            [crate::syntax::OpArg::Named(n, v)] if n.text == "at" => match v {
                crate::syntax::Arg::Ref(r) => Some(r.clone()),
                crate::syntax::Arg::Word(x) => Some(local(x.clone(), n.span)),
                _ => None,
            },
            _ => None,
        };
        let spell = |m: &str| format!("`{} tangent(at: {m}) {}`", names[0], names[1]);
        let Some(point) = point else {
            let m = format!("two sets touch at a point, which the word names: {}", spell("m"));
            self.once(Code::E040, at, m);
            return;
        };
        let spelled = spell(&written(&point));
        if rel.claim {
            let m = format!("a tangency between two sets is stated, not claimed: {spelled}");
            self.once(Code::E040, at, m);
            return;
        }
        if rel.along.is_some() || sc.twin().is_some() {
            let m = format!("a set's body may put its point on another set, not state {spelled}");
            self.once(Code::E040, at, m);
            return;
        }
        for ((_, abs), name) in found.iter().zip(&names) {
            if let Some(what) = makes_points(&self.sets[abs].lit) {
                let m = format!(
                    "{spelled} reads each set's body at the point, where `{name}`'s makes \
                     `{what}` of its own, which only the point on it would place"
                );
                self.once(Code::E040, at, m);
                return;
            }
        }
        // one derivative per direction, which both sets' rows share
        let keys = [0u8, 1].map(|k| format!("{}#{}.t{k}.", sc.prefix(), st.id.0));
        let worded = Worded {
            word: w.word.text.clone(),
            ops: Vec::new(),
            args: format!("at: {}", written(&point)),
            sets: found.iter().map(|(k, _)| (*k, self.set_written(&w.ops[*k]))).collect(),
            span: at,
        };
        let u = Use { st, rel, path, scope: sc, at };
        for (n, (_, abs)) in found.iter().enumerate() {
            let site = &self.sets[abs];
            let (lit, depth, closure) = (site.lit.clone(), site.depth, site.scope.clone());
            let app = self.begin_nth(&u, &closure, n);
            let bound = lit.bound.text.clone();
            self.bind_to_use(&app, &bound, point.clone(), &u);
            let along = |k: u8| Pass::Along(Along {
                point: local(&bound, at),
                toward: AlongBy::Chart(k),
                key: keys[k as usize].clone(),
                made: None,
            });
            let walks = [Pass::Made, along(0), along(1)];
            self.apply(&u, app, lit.body, &BTreeMap::new(), &walks, worded.clone(), depth);
        }
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
/// or anything that makes some — which a tangency at a point cannot place; `None` where the body
/// only relates the bound point and what the set was given (a line between two of them is no
/// geometry of its own: it moves as its ends do).
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

/// A set's body that is a circle's (#105): `p coincident P` and `p distance(r) o`, in either
/// order and either way round, and nothing else — the plane and the centre as written, and the
/// radius's text, where it is written and the word it is written with.
fn round(lit: &SetLit) -> Option<(Ref, Ref, (String, Span, Name))> {
    let [a, b] = lit.body.as_slice() else { return None };
    let bound = |r: &Ref| r.path.is_empty() && r.root.text == lit.bound.text;
    // the one operand that is not the set's point, of a plain relation between two
    let other = |st: &Stmt, word: &str| -> Option<(Ref, crate::syntax::Written)> {
        let StmtKind::Relation(r) = &st.kind else { return None };
        let w = r.form.written()?;
        let plain = !r.claim && r.word.is_none() && r.along.is_none();
        if !plain || w.word.text != word || w.ops.len() != 2 {
            return None;
        }
        match (bound(&w.ops[0]), bound(&w.ops[1])) {
            (true, false) => Some((w.ops[1].clone(), w.clone())),
            (false, true) => Some((w.ops[0].clone(), w.clone())),
            _ => None,
        }
    };
    let pair = |on: &Stmt, at: &Stmt| -> Option<(Ref, Ref, (String, Span, Name))> {
        let (plane, w) = other(on, "coincident")?;
        if !w.args.is_empty() {
            return None;
        }
        let (centre, d) = other(at, "distance")?;
        let [crate::syntax::OpArg::Dim(text, span)] = d.args.as_slice() else { return None };
        Some((plane, centre, (text.clone(), *span, d.word.clone())))
    };
    pair(a, b).or_else(|| pair(b, a))
}
