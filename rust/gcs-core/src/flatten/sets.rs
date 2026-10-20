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
use crate::lowering::{self, Lowered, Radius, Shape};
use crate::constraints::Fixity::{Infix, Prefix};
use crate::syntax::{
    Along, AlongBy, Chained, Cmp, DeclName, Member, Relation, RelationForm, SetLit, Worded, Written,
};

/// Where a set stands: its literal, the scope its body reads names in (an instance's, with its
/// formals bound, for a family's), how deep the walk was there, and what its body is (`shape`).
/// A use's expansion stands where the use does, at the use's path.
#[derive(Clone)]
pub(super) struct Site {
    lit: SetLit,
    scope: Scope,
    depth: usize,
    shape: BodyShape,
}

/// What a set's body is, read once where the set is made (`Site::of`): every question a use asks
/// of the body, so none scans it again.
#[derive(Clone, Debug, Default)]
struct BodyShape {
    /// A **region** (§6.21, §9.6): its body bounds a number, or says its point is inside or
    /// outside something — so it has no surface to touch.
    region: bool,
    /// The statements that are relations or bounds on measurements, by index: what the body says
    /// of its point.
    relations: Vec<usize>,
    /// The statements stating a number of the set's own, by index: a dimension or a bound on
    /// measurements, but not a bounded side of another set (`region_body`).
    numbers: Vec<usize>,
    /// The statements using another set (`may_use`), by index.
    uses: Vec<usize>,
    /// What the body makes of its own, which a tangency at a point cannot place (`makes_points`).
    makes: Option<String>,
    /// A set of one side of another, `{ q | q outside S }` (its body one bounded use of `S`, no
    /// number of its own): `S` as written, and whether it is the outside.
    side_of: Option<(Ref, bool)>,
}

impl BodyShape {
    fn of(lit: &SetLit) -> BodyShape {
        let body = &lit.body;
        let measured = |st: &Stmt| matches!(st.kind, StmtKind::Measured(_));
        let indices =
            |f: &dyn Fn(&Stmt) -> bool| (0..body.len()).filter(|&i| f(&body[i])).collect();
        // a side of another set, grown or not (a bounded distance from it), is a use of it, not a
        // number of this one's; a distance from one is (`std.Offset`'s body)
        let side_of_another = |w: &Written| reaches(w) && w.bound().is_some();
        BodyShape {
            region: body.iter().filter_map(written_of).any(|w| w.states_bound())
                || body.iter().any(measured),
            relations: indices(&|st| matches!(st.kind, StmtKind::Relation(_)) || measured(st)),
            // a bound on measurements is a number of this one's, a bound already (#183 F3)
            numbers: indices(&|st| measured(st)
                || written_of(st).is_some_and(|w| w.dim().is_some() && !side_of_another(w))),
            uses: indices(&|st| written_of(st).is_some_and(may_use)),
            makes: makes_points(lit),
            side_of: side_of(lit),
        }
    }
}

/// Whether what a path reads past a set is its surface (`S.surface`).
fn names_surface(rest: &[String]) -> bool {
    matches!(rest, [m] if Member::of(m) == Some(Member::Surface))
}

impl Site {
    /// A set made other than by a statement the walk met: a face of a solid (`faces`).
    pub(super) fn new(lit: SetLit, scope: Scope, depth: usize) -> Site {
        Site::of(lowered(lit), scope, depth)
    }

    /// A set whose body is already as a site holds it, its shape read.
    fn of(lit: SetLit, scope: Scope, depth: usize) -> Site {
        let shape = BodyShape::of(&lit);
        Site { lit, scope, depth, shape }
    }
}

/// A set's body with its sides said as bounds (`Written::lower_side`, #192): the one form a body
/// is turned round, made a surface and held in (`region_body`, `surface_of`, `Walk::apply`).
fn lowered(mut lit: SetLit) -> SetLit {
    for st in lit.body.iter_mut() {
        if let StmtKind::Relation(r) = &mut st.kind {
            if let Some(w) = r.form.written_mut() {
                w.lower_side();
            }
        }
    }
    lit
}

/// Where the walk made a set: the statement that made it — the set written in place, or an
/// instance of a family — at its path, and the scope that statement stands in.
pub(super) struct Made<'m> {
    pub st: &'m Stmt,
    pub path: &'m [PathStep],
    pub outer: &'m Scope,
}

/// One use of a set (§6.21), as `expand_set_uses` found it: the statement as written (`Use`), its
/// relation, the operands naming sets — each by its place and the set by absolute name — and
/// how every relation the use makes reads (`Relation::word`), its parentheses left to each case.
struct SetUse<'u> {
    u: Use<'u>,
    w: &'u Written,
    found: Vec<(usize, String)>,
    worded: Worded,
}

impl SetUse<'_> {
    /// The one set a use of one uses: its operand, and the set by absolute name.
    fn set(&self) -> (usize, &str) {
        let (k, abs) = &self.found[0];
        (*k, abs)
    }

    /// Why the use, a claim or stated in a body walked for its derivative, cannot place the
    /// point it reads its sets' bodies at — a point the solve finds, which a claim cannot add and
    /// a derivative cannot move — each said its own way; `None` where it can.  A contact is
    /// refused only where the use was set aside from such a body (`Relation::along`).
    fn refuse_derivative(&self, placed: &Placed) -> Option<String> {
        let rel = self.u.rel;
        let derived = rel.along.is_some() || self.u.scope.twin().is_some();
        Some(match *placed {
            Placed::Contact { name, .. } if rel.claim => format!(
                "a tangency to a set is stated, not claimed: the contact on it is found by the \
                 solve (`l tangent {name}`)"
            ),
            Placed::Contact { word, name } if rel.along.is_some() => format!(
                "a set's body may put its point on another set, not a line tangent to one \
                 (`{word} {name}`)"
            ),
            Placed::Pair(spelled) if rel.claim => {
                format!("a tangency between two sets is stated, not claimed: {spelled}")
            }
            Placed::Pair(spelled) if derived => {
                format!("a set's body may put its point on another set, not state {spelled}")
            }
            Placed::Touch { spelled, .. } if rel.claim || derived => format!(
                "a plane's tangency to a set is stated, and never inside a set's body: {spelled}"
            ),
            Placed::Foot(set) if rel.claim => format!(
                "a point's distance from `{set}` is found by its foot on it, which a claim \
                 cannot add"
            ),
            Placed::Foot(set) if derived => format!(
                "a point's distance from `{set}` has a foot on it, which a derivative cannot \
                 move"
            ),
            _ => return None,
        })
    }
}

/// The point a set's use reads the set's body at, for the refusals the uses share
/// (`Walk::refused`): each says what the use is, as the statement spells it.
enum Placed<'s> {
    /// `l tangent S`'s contact: the word and the set's name.
    Contact { word: &'s str, name: &'s str },
    /// `S1 tangent(at: m) S2`'s point, the statement spelled.
    Pair(&'s str),
    /// `P tangent(at: m) S`'s point, the statement spelled, and `S` as written.
    Touch { spelled: &'s str, set: &'s str },
    /// `p distance(d, side: outside) S`'s foot, `S` as written.
    Foot(&'s str),
}

impl Placed<'_> {
    /// Why a set's body making `what` of its own, `name` as the statement names the set, cannot
    /// be read at the point — which only the point on it would place; `None` for a contact, what
    /// the body makes moving with it by a tangent unknown of its own (`model::Dual`).
    fn made(&self, name: &str, what: &str) -> Option<String> {
        Some(match *self {
            Placed::Contact { .. } => return None,
            Placed::Pair(spelled) => format!(
                "{spelled} reads each set's body at the point, where `{name}`'s makes `{what}` of \
                 its own, which only the point on it would place"
            ),
            Placed::Touch { spelled, set } => format!(
                "{spelled} reads the set's body at the point, where `{set}`'s makes `{what}` of \
                 its own, which only the point on it would place"
            ),
            Placed::Foot(set) => format!(
                "a point's distance from `{set}` reads its body at the foot, where it makes \
                 `{what}` of its own"
            ),
        })
    }
}

/// A reach read off its set (`Walk::expand_reach`), for the way it is applied: the body measured
/// from — the set's, or its one bound's surface — and the statement in it holding the number
/// its surface is at, the bound on the signed distance toward the outside (`None` an equation),
/// the distance as written and as read, whether the offset grows the set, the scope and depth
/// the set stands at, how the use reads and the set as written.
struct Reached {
    lit: SetLit,
    i: usize,
    cmp: Option<Cmp>,
    text: String,
    d: Aff,
    grows: bool,
    closure: Scope,
    depth: usize,
    worded: Worded,
    set: String,
}

impl<'a> Walk<'a> {
    /// A set, made where the walk met it (§6.21) — and where its body is an element's, the
    /// element (`lowering`).
    pub(super) fn set_made(
        &mut self,
        abs: String,
        lit: &SetLit,
        scope: &Scope,
        depth: usize,
        made: Made,
    ) {
        self.sets.made.insert(abs.clone(), Site::new(lit.clone(), scope.clone(), depth));
        // a set made inside a predicate's use is made once per use, and is that use's: no
        // drawing of its own
        let applied = self.sets.applying || !self.words.applying.is_empty();
        if self.sym.is_some() || applied || !matches!(scope.pred.pass, Pass::Itself)
            || self.sets.unlowered.contains(&abs)
        {
            return;
        }
        let Some(shape) = lowering::shape(lit) else { return };
        let form = shape.form();
        let drawn = match shape {
            Shape::Circle { centre, radius, .. } => {
                self.lower_circle(&abs, centre, radius, scope, made)
            }
        };
        if drawn {
            self.sets.lowered.insert(abs, form);
        }
    }

    /// **A set whose body is a circle's is the circle** (#105, §6.21): `{ p | p coincident P; p
    /// distance(r) o }` is drawn as `circle(center: o)` with `radius(r)` on it, so it solves,
    /// drags and is dimensioned as one, and a point is put on it as on any circle.  The circle is
    /// what the set's statement made, under the set's name; its radius is the body's distance,
    /// drawn where the body writes it — or, for a family's instance, where the call gives it.
    /// Whether `P` is a plane and `o` a point drawn in it is the elaborator's question
    /// (`lowering::refused`): where either is not, the set is walked again as a set.  Whether
    /// it drew one.
    fn lower_circle(&mut self, abs: &str, centre: &Ref, radius: Radius, sc: &Scope, made: Made)
        -> bool {
        let Made { st, path, outer } = made;
        // what the source calls it, and for a family's instance the call its number is given in
        let (written, call) = match &st.kind {
            StmtKind::Set(set) => (&set.name, None),
            StmtKind::Instance(inst) => (&inst.name, Some(inst)),
            _ => return false,
        };
        let seed = value_aff(radius.text, &*sc.vals, self.units).ok().and_then(|a| a.number())
            .filter(|r| r.is_finite() && *r > 0.0);
        // a set written in place, or an unnamed call, is named nothing the source can say
        let name = if outer.stamps.anonymous || written.text.starts_with('#') {
            let at = written.span.lo as usize;
            DeclName::Key(Name { text: abs.to_string(), span: Span::at(at) })
        } else {
            DeclName::Written(written.clone()).prefixed(abs.to_string(), outer.block.copies)
        };
        let mut d = Decl::point(name, [0.0; 2], None);
        d.kind = crate::model::EntKind::Circle;
        d.children = vec![vec![Kid::Ref(centre.clone())]];
        d.seed = vec![seed.unwrap_or(1.0)];
        d.seed_text = vec![None];
        self.names.insert(abs.to_string());
        let decl = Stmt { id: st.id, kind: StmtKind::Decl(d), span: st.span, chained: Chained::No };
        self.out.push((decl, path.to_vec(), sc.clone()));
        // drawn and edited where the number is written; a call that gives none draws what it
        // comes to
        let span = radius.at(call).unwrap_or(Span::at(st.span.lo as usize));
        let word = radius.word.span;
        let dim = crate::syntax::OpArg::Dim(radius.text.to_string(), span);
        let stated = relation(("radius", word), Prefix, vec![local(abs, word)], vec![dim], st.span);
        if let Some(r) = self.settle_relation(&stated, &sc.vals, sc) {
            // the circle by its absolute name, which the body's scope may be closed to
            let named = Scope { prefixes: vec![String::new()], closed: false, ..sc.clone() };
            let kind = StmtKind::Relation(r);
            let rel = Stmt { id: st.id, kind, span: st.span, chained: Chained::No };
            self.out.push((rel, path.to_vec(), named));
        }
        true
    }

    /// Every set drawn as an element, what its shape names resolved where its body reads it —
    /// for the elaborator to judge (`lowering::refused`).
    pub(super) fn to_judge(&self, alias: &BTreeMap<String, String>) -> Vec<Lowered> {
        self.sets.lowered.iter().map(|(key, form)| {
            let site = &self.sets.made[key];
            let mut form = form.clone();
            for r in form.refs() {
                *r = r.take().and_then(|r| self.absolute(&r, &site.scope, alias));
            }
            Lowered { key: key.clone(), form }
        }).collect()
    }

    /// A reference read where `sc` reads it, made absolute — `None` where it names a set or
    /// nothing.
    fn absolute(&self, r: &Ref, sc: &Scope, alias: &BTreeMap<String, String>) -> Option<Ref> {
        let (abs, rest) = self.look(alias).find(r, sc)?;
        if rest.is_empty() && self.sets.made.contains_key(&abs) {
            return None;
        }
        let path = rest.into_iter().map(|f| Seg::Field(Name::new(f))).collect();
        Some(Ref { root: Name { text: abs, span: r.span }, path, span: r.span })
    }

    /// A statement's use of a set: the relation, which operand names the set and the sets its
    /// operands name, by absolute name — `None` for a statement naming none.  Only `coincident`
    /// and `tangent` between two operands may use one.
    fn set_use<'s>(
        &mut self,
        st: &'s Stmt,
        sc: &Scope,
        alias: &BTreeMap<String, String>,
    ) -> Option<(&'s Relation, Vec<(usize, String)>)> {
        let StmtKind::Relation(rel) = &st.kind else { return None };
        let w = rel.form.written().filter(|w| may_use(w))?;
        let mut found: Vec<(usize, String)> = Vec::new();
        for (i, r) in w.ops.iter().enumerate() {
            let Some((abs, rest)) = self.look(alias).raw(r, sc)
                .filter(|(abs, _)| self.sets.made.contains_key(abs)) else { continue };
            if rest.is_empty() {
                found.push((i, abs));
            } else if names_surface(&rest) {
                // a set's surface (`S.surface`): the set with each bound an equality, a set of its
                // own (#214: a point on a side grown, where a topping leaves a crown's flank)
                let surface = self.surface_of(&abs);
                found.push((i, surface));
            }
        }
        // a set drawn as an element is one, and a point is put on it as on any (#105) — but for
        // a derivative, which reads the body's rows
        let element = matches!(found.as_slice(),
                [(_, abs)] if self.sets.lowered.get(abs).is_some_and(|f| f.answers(&w.word.text)))
            && w.args.is_empty()
            && rel.along.is_none() && sc.twin().is_none();
        (!found.is_empty() && !element).then_some((rel, found))
    }

    /// Expand every use of a set, each where the walk met it, until none is left.  A use's
    /// expansion may hold a use of another set (a set defined over one), so this runs in rounds;
    /// one still standing after `MAX_DEPTH` of them is a set defined in terms of itself.
    pub(super) fn expand_sets(&mut self) {
        if self.sets.made.is_empty() {
            return;
        }
        self.sets.applying = true;
        self.expand_set_uses();
        self.sets.applying = false;
    }

    /// `expand_sets`' rounds, while every set made is made inside a use.
    fn expand_set_uses(&mut self) {
        for round in 0..=MAX_DEPTH {
            // no word a set is used by, no table to build: the last round is usually this
            let words = |st: &Stmt| match &st.kind {
                StmtKind::Relation(r) => r.form.written().is_some_and(may_use),
                StmtKind::Measured(_) => true,
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
                // a set measured from, or where one ends (#183 F2, F7): a point of its own
                if let Some(made) = self.measured_sets(&st, &path, &sc, &alias) {
                    any = true;
                    next.extend(made);
                    continue;
                }
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
                    let m = "`coincident`, `inside`, `outside` and `distance` relate a set to a \
                             point, and `tangent` a set to a line or to another set at a point";
                    self.once(Code::E040, st.span, m);
                    continue;
                }
                // a region has no surface to touch (§6.21) — by a line, a plane or a set alike
                let tangent = rel.form.written().filter(|w| w.word.text == "tangent");
                let bounded = found.iter().find(|(_, abs)| self.sets.made[abs].shape.region);
                if let (Some(w), Some((_, abs))) = (tangent, bounded) {
                    let m = format!("`{}` is a region, and has no surface to touch: touch the set \
                                     it is bounded by", public_path(abs));
                    self.once(Code::E040, w.word.span, m);
                    continue;
                }
                self.aside.held = next.len() + rest.len();
                let su = self.set_use_of(&st, rel, &path, &sc, found);
                // one set touched at a point: by a plane, the one thing that can be
                let touch = !pair && su.w.word.text == "tangent" && !su.w.args.is_empty();
                if pair {
                    self.expand_pair(&su);
                } else if reaches(su.w) {
                    // a distance from a set, or a side of one grown (F4)
                    self.expand_reach(&su, &alias);
                } else if touch {
                    self.expand_touch(&su);
                } else {
                    self.expand_use(&su);
                }
                next.append(&mut self.out);
                self.aside.held = 0;
            }
            self.out = next;
            if !any {
                return;
            }
        }
    }

    /// **A set measured from** (#183 F2): `distance(p, S)` in a bound on measurements, and `p
    /// inside S.ends` (F7), each set given a hidden point of its own, keyed by the statement and
    /// the set (`#{stmt}.m{k}`), where `S`'s body is applied (`inner coincident S`) — so its
    /// bounds are the set's conditions, a term of the region it is measured in reads them
    /// (`model::Target::Set`), and the measurement names the point in the set's place.  `None`
    /// where the statement measures no set.
    fn measured_sets(
        &mut self,
        st: &Stmt,
        path: &[PathStep],
        sc: &Scope,
        alias: &BTreeMap<String, String>,
    ) -> Option<Vec<(Stmt, Vec<PathStep>, Scope)>> {
        let set_of = |me: &Self, r: &Ref| -> Option<(String, Vec<String>)> {
            let (abs, rest) = me.look(alias).raw(r, sc)?;
            me.sets.made.contains_key(&abs).then_some((abs, rest))
        };
        // the sets the statement reads, by the reference as written: a measurement's argument
        // (a set, or a set's surface, `S.surface`), or the operand `S.ends`
        let mut sets: Vec<(String, Ref)> = Vec::new();
        let mut surfaces: BTreeMap<String, String> = BTreeMap::new();
        match &st.kind {
            StmtKind::Measured(m) => {
                for r in m.lhs.measures().into_iter().chain(m.rhs.measures()).flat_map(|x| x.refs()) {
                    let Some((abs, rest)) = set_of(self, r) else { continue };
                    let text = crate::syntax::ref_text(r);
                    let surface = names_surface(&rest);
                    if (rest.is_empty() || surface) && !sets.iter().any(|(t, _)| *t == text) {
                        if surface {
                            surfaces.insert(text.clone(), self.surface_of(&abs));
                        }
                        sets.push((text, r.clone()));
                    }
                }
            }
            StmtKind::Relation(r) => {
                let w = r.form.written()?;
                let [_, s] = w.ops.as_slice() else { return None };
                // `S.ends`: the member, where `S` is a set and declares no `ends` (below)
                let ends = matches!(s.path.last(), Some(Seg::Field(f))
                    if Member::of(&f.text) == Some(Member::Ends));
                if !ends || w.side().is_none() {
                    return None;
                }
                // a name declared as written is itself, whatever it is called
                if self.look(alias).raw(s, sc).is_some_and(|(_, r)| r.is_empty()) {
                    return None;
                }
                let mut set = s.clone();
                set.path.pop();
                set_of(self, &set).filter(|(_, rest)| rest.is_empty())?;
                sets.push((crate::syntax::ref_text(&set), set));
            }
            _ => return None,
        }
        if sets.is_empty() {
            return None;
        }
        let mut out = Vec::new();
        let mut named: BTreeMap<String, String> = BTreeMap::new();
        // the statement as written, which an application's stamp does not hide: two statements
        // of one body stamped alike stay apart
        let id = sc.pred.written.unwrap_or(st.id).0;
        for (k, (name, written)) in sets.iter().enumerate() {
            let local = keys::measured(id, k);
            let key = format!("{}{local}", sc.prefix());
            self.names.insert(key.clone());
            self.sets.probes.push(key.clone());
            let at = st.span;
            let point = probe(key, Span::at(at.lo as usize));
            out.push((Stmt { kind: StmtKind::Decl(point), ..st.clone() }, path.to_vec(), sc.clone()));
            let set = match surfaces.get(name) {
                // a set's surface, made beside it, by its absolute name
                Some(abs) => Ref { root: Name { text: abs.clone(), span: at }, path: Vec::new(), span: at },
                None => {
                    let root = Name { span: at, ..written.root.clone() };
                    Ref { root, path: written.path.clone(), span: at }
                }
            };
            let on = crate::syntax::Written {
                word: Name { text: "coincident".to_string(), span: at },
                fixity: Infix,
                ops: vec![Ref::new(&local), set],
                args: Vec::new(),
                span: at,
                rates: Vec::new(),
            };
            let rel = Relation::of(RelationForm::Written(on));
            // applied under a prefix of its own, so two sets one statement measures from bind
            // their points apart
            let mut usc = sc.clone();
            usc.pred.held = true;
            usc.prefixes.insert(0, keys::measured_prefix(sc.prefix(), id, k));
            if !usc.prefixes.iter().any(|p| p.is_empty()) {
                usc.prefixes.push(String::new());
            }
            out.push((Stmt { kind: StmtKind::Relation(rel), ..st.clone() }, path.to_vec(), usc));
            named.insert(name.clone(), local);
        }
        let mut st = st.clone();
        match &mut st.kind {
            // each measurement naming a set names its point instead, in the reference's place
            StmtKind::Measured(m) => {
                for x in m.lhs.measures_mut().into_iter().chain(m.rhs.measures_mut()) {
                    for r in x.refs_mut() {
                        if let Some(local) = named.get(&crate::syntax::ref_text(r)) {
                            *r = Ref { span: r.span, ..Ref::new(local) };
                        }
                    }
                }
            }
            StmtKind::Relation(r) => {
                if let Some(w) = r.form.written_mut() {
                    let local = named.values().next().expect("one set").clone();
                    // the member, read here once: the elaborator matches it, not the word (#188)
                    let at = w.ops[1].span;
                    let path = vec![Seg::Member(Member::Ends)];
                    w.ops[1] = Ref { root: Name { text: local, span: at }, path, span: at };
                }
            }
            _ => {}
        }
        out.push((st, path.to_vec(), sc.clone()));
        Some(out)
    }

    /// **A set's surface** (#183): the set made beside `abs` whose body is `abs`'s with each bound
    /// made the equality at its number (`Written::equality`) — a ball's sphere, a half-space's
    /// plane, a side's surface, a grown side's the offset one — by its absolute name
    /// (`{abs}#surface`), made once.  An interval's two surfaces are no one surface: kept.
    fn surface_of(&mut self, abs: &str) -> String {
        let key = keys::surface(abs);
        if !self.sets.made.contains_key(&key) {
            let site = &self.sets.made[abs];
            let mut lit = site.lit.clone();
            for st in lit.body.iter_mut() {
                let StmtKind::Relation(r) = &mut st.kind else { continue };
                if let Some(w) = r.form.written_mut() {
                    w.equality();
                }
            }
            let made = Site::of(lit, site.scope.clone(), site.depth);
            self.sets.made.insert(key.clone(), made);
            self.names.insert(key.clone());
        }
        key
    }

    /// A use as `expand_set_uses` found it (`set_use`), described as it was written: the word,
    /// the operands that are no set, and the sets as written — its parentheses each case's own.
    fn set_use_of<'u>(
        &self,
        st: &'u Stmt,
        rel: &'u Relation,
        path: &'u [PathStep],
        sc: &'u Scope,
        found: Vec<(usize, String)>,
    ) -> SetUse<'u> {
        let w = rel.form.written().expect("a set's use is written");
        let worded = Worded {
            word: w.word.text.clone(),
            ops: (0..w.ops.len())
                .filter(|i| found.iter().all(|(k, _)| k != i))
                .map(|i| w.ops[i].clone())
                .collect(),
            args: String::new(),
            sets: found.iter().map(|(k, _)| (*k, self.set_written(&w.ops[*k]))).collect(),
            through: Vec::new(),
            span: w.word.span,
        };
        SetUse { u: Use { st, rel, path, scope: sc, at: w.word.span }, w, found, worded }
    }

    /// Whether a use cannot place the point it reads its sets' bodies at (`placed`) — a claim,
    /// a body walked for its derivative, or a body making geometry of its own only the point on
    /// it would place — said, once, where the word is.
    fn refused(&mut self, su: &SetUse, placed: Placed) -> bool {
        let m = su.refuse_derivative(&placed).or_else(|| {
            su.found.iter().find_map(|(k, abs)| {
                let what = self.sets.made[abs].shape.makes.as_deref()?;
                placed.made(&ref_text(&su.w.ops[*k]), what)
            })
        });
        let Some(m) = m else { return false };
        self.once(Code::E040, su.u.at, m);
        true
    }

    /// One use of one set, `q coincident S`, `q inside S` or `l tangent S` (§6.21).
    fn expand_use(&mut self, su: &SetUse) {
        let (k, abs) = su.set();
        let (w, at) = (su.w, su.u.at);
        let word = w.word.text.as_str();
        let name = public_path(abs);
        // a side, said as a bound (`Written::lower_side`): the body with its number bounded
        let mut lowered = w.clone();
        lowered.lower_side();
        let side = match lowered.args.as_slice() {
            [] => None,
            [crate::syntax::OpArg::Bound { cmp, hi: None, grow: None, .. }]
                if lowered.word.text == "coincident" => Some(*cmp),
            _ => {
                let m = format!("`{word}` with a set takes nothing in its parentheses");
                self.once(Code::E040, at, m);
                return;
            }
        };
        let tangent = word == "tangent";
        if tangent && self.refused(su, Placed::Contact { word, name: &name }) {
            return;
        }
        let site = self.sets.made[abs].clone();
        let mut body = site.lit.body;
        // **inside or outside a set** (§9.6): its body with its one number a bound
        if let Some(side) = side {
            if let Err(m) = region_body(&mut body, &site.shape, side, word, at, &name) {
                self.once(Code::E040, at, m);
                return;
            }
        }
        let u = &su.u;
        let app = self.begin(u, &site.scope).of_set();
        let other = w.ops[1 - k].clone();
        let bound = site.lit.bound.text;
        // a tangency: the body at a contact on the line, then differentiated along it; else the
        // body at the point — differentiated already where the use is in a body being
        // differentiated (a tangency there was refused above)
        let walks = if tangent {
            self.bind_to_use(&app, keys::LINE, other, u);
            self.contact(&app, u.st, &bound, at, u.path);
            let toward = AlongBy::Line(local(keys::LINE, at));
            let key = app.prefix().to_string();
            let made = Some(key.clone());
            vec![Pass::Itself, Pass::Along(Along { point: local(&bound, at), toward, key, made })]
        } else {
            self.bind_to_use(&app, &bound, other, u);
            vec![self.inherited_twin(&app, u).into()]
        };
        self.apply(u, app, body, &BTreeMap::new(), &walks, su.worded.clone(), site.depth);
    }

    /// Two sets tangent at a point, `S1 tangent(at: m) S2` (§6.21): their tangent spaces at `m`
    /// are one — every direction along the first there is along the second.  Two directions span
    /// the first's, each solved for in a chart (`model::Toward::Chart`), and each set's body is
    /// stated as its derivative along both at `m`: four rows over two unknowns for two surfaces.
    /// `m` on each is stated beside it (`m coincident S1`); the word walks each body only for
    /// what it makes, so a body with geometry of its own, which the incidence would place, is
    /// refused.
    fn expand_pair(&mut self, su: &SetUse) {
        let (w, at) = (su.w, su.u.at);
        let names: Vec<String> = su.found.iter().map(|(k, _)| ref_text(&w.ops[*k])).collect();
        let spell = |m: &str| format!("`{} tangent(at: {m}) {}`", names[0], names[1]);
        let Some(point) = at_point(w) else {
            let m = format!("two sets touch at a point, which the word names: {}", spell("m"));
            self.once(Code::E040, at, m);
            return;
        };
        if self.refused(su, Placed::Pair(&spell(&ref_text(&point)))) {
            return;
        }
        // one derivative per direction, which both sets' rows share — under the statement as
        // written, so two tangencies of one body take directions apart
        let u = &su.u;
        let id = u.scope.pred.written.unwrap_or(u.st.id).0;
        let charts = [0u8, 1].map(|k| keys::chart(u.scope.prefix(), id, k));
        let worded = Worded { args: format!("at: {}", ref_text(&point)), ..su.worded.clone() };
        for (n, (_, abs)) in su.found.iter().enumerate() {
            let site = self.sets.made[abs].clone();
            let app = self.begin_nth(u, &site.scope, n).of_set();
            let bound = site.lit.bound.text;
            self.bind_to_use(&app, &bound, point.clone(), u);
            let along = |k: u8| charted(local(&bound, at), k, charts[k as usize].clone());
            let walks = [Pass::Made, along(0), along(1)];
            let body = site.lit.body;
            self.apply(u, app, body, &BTreeMap::new(), &walks, worded.clone(), site.depth);
        }
    }

    /// A plane tangent to a set at a point, `P tangent(at: m) S` (§6.21, #148): `P` is `S`'s
    /// tangent plane at `m` — the body differentiated at `m` along each of `P`'s axes, as two
    /// sets' tangency is along its charts, and `m` on `P` (a row only where the drawing does not
    /// draw it there, `program::relations`).  `m` on `S` is stated beside it, as for two sets.
    /// What the body already says along the plane — a cone's along its generator, where the
    /// apex is on `P` — makes the two derivatives dependent: the system's rank finds that
    /// (`diagnose`), and nothing here knows a shape.
    fn expand_touch(&mut self, su: &SetUse) {
        let (k, abs) = su.set();
        let (w, at) = (su.w, su.u.at);
        let set = self.set_written(&w.ops[k]);
        let spell = |m: &str| format!("`P tangent(at: {m}) {set}`");
        let Some(point) = at_point(w) else {
            let m = format!("a plane touches a set at a point, which the word names: {}",
                spell("m"));
            self.once(Code::E040, at, m);
            return;
        };
        if self.refused(su, Placed::Touch { spelled: &spell(&ref_text(&point)), set: &set }) {
            return;
        }
        let plane = w.ops[1 - k].clone();
        let worded = Worded { args: format!("at: {}", ref_text(&point)), ..su.worded.clone() };
        let u = &su.u;
        let site = self.sets.made[abs].clone();
        let app = self.begin(u, &site.scope).of_set();
        let bound = site.lit.bound.text;
        self.bind_to_use(&app, &bound, point.clone(), u);
        // the plane's own two directions, each a derivative of its own, under the statement as
        // written
        let id = u.scope.pred.written.unwrap_or(u.st.id).0;
        let mut walks = vec![Pass::Made];
        for (n, axis) in ["u", "v"].into_iter().enumerate() {
            let mut r = plane.clone();
            r.path.push(Seg::Field(Name { text: axis.to_string(), span: at }));
            let name = keys::plane_axis(axis);
            self.bind_to_use(&app, &name, r, u);
            walks.push(Pass::Along(Along {
                point: local(&bound, at),
                toward: AlongBy::Line(local(&name, at)),
                key: keys::plane_along(u.scope.prefix(), id, n),
                made: None,
            }));
        }
        self.apply(u, app, site.lit.body, &BTreeMap::new(), &walks, worded.clone(), site.depth);
        // and the point on the plane
        let mut on = relation(("coincident", at), Infix, vec![point, plane], Vec::new(), u.st.span);
        on.word = Some(worded);
        let kind = StmtKind::Relation(on);
        let made = Stmt { id: u.st.id, kind, span: u.st.span, chained: Chained::No };
        self.out.push((made, u.path.to_vec(), u.scope.clone()));
    }

    /// The set a set is one side of (`BodyShape::side_of`), by absolute name, and whether it is
    /// the outside — `None` for any other.
    fn side_of_set(&self, abs: &str, alias: &BTreeMap<String, String>) -> Option<(String, bool)> {
        let site = self.sets.made.get(abs)?;
        let (other, outside) = site.shape.side_of.as_ref()?;
        let (abs, rest) = self.look(alias).raw(other, &site.scope)?;
        (rest.is_empty() && self.sets.made.contains_key(&abs)).then_some((abs, *outside))
    }

    /// **A point's distance from a set, and a side of a set grown** (§6.21, F4): `p distance(d,
    /// side: outside) S` is `p` the signed distance `d` from `S`'s surface, positive on its
    /// outside, where its measure exceeds its number; `p distance(<= d, side: outside) S` is
    /// within `S` grown by `d` and `side: inside` within its outside grown by `d` — a bound on
    /// that signed distance (`inside(d)` is retired, #266).
    /// One meaning, said three ways, by what `S`'s body measures and where `p` is:
    ///
    /// * a **distance** (a sphere, a cylinder, a level): the signed distance is the measure less
    ///   its number, so the offset moves the number — `p distance(r + d) c` — anywhere;
    /// * another measure (a cone's angle) at a **region solid's probe**: the bound carries the
    ///   offset (`OpArg::Bound::grow`), a term of the solid that is the cone's exact offset;
    /// * a drawn point: a **foot** on `S`, declared at the body's point and walked as the body,
    ///   `p` at `d` from it on the side named, and `p − foot` square to `S` there — the body
    ///   and the distance each differentiated along two directions solved for at the foot
    ///   (charts, as two sets touching are), which is `S` tangent there to the sphere about `p`.
    ///
    /// This reads the reach off its set (`reach_across` where the set is one side of another)
    /// and applies it by `reach_number` or `reach_foot`.
    fn expand_reach(&mut self, su: &SetUse, alias: &BTreeMap<String, String>) {
        use crate::syntax::OpArg;
        let (k, abs) = su.set();
        let (w, rel, at) = (su.w, su.u.rel, su.u.at);
        let word = w.word.text.as_str();
        let set = self.set_written(&w.ops[k]);
        // a side grown, as a set's body carries it (`Written::lower_side`): a bounded distance
        let mut lowered = w.clone();
        lowered.lower_side();
        if lowered.word.text != "distance" {
            let m = format!("`{word}` grows `{set}` by one distance: `{word}(d) {set}`");
            return self.once(Code::E040, at, m);
        }
        if let Some((other, outside)) = self.side_of_set(abs, alias) {
            return self.reach_across(su, lowered, other, outside, alias);
        }
        let Reach { cmp, text, grows, args } = match reach_of(&lowered, &set) {
            Ok(r) => r,
            Err(m) => return self.once(Code::E040, at, m),
        };
        // described as written: a side grown by its distance
        let args = if word == "distance" { args } else { text.clone() };
        // a region of one bound has the surface the bound's number makes (#183): measured from,
        // its outside the bound's (the larger side of `<=`, the smaller of `>=`)
        let mut grows = grows;
        let mut flipped = false;
        let mut surface: Option<SetLit> = None;
        let site = &self.sets.made[abs];
        if site.shape.region {
            match one_bound(site) {
                Some((bare, ge)) => {
                    grows ^= ge;
                    flipped = ge;
                    surface = Some(bare);
                }
                None => {
                    let m = format!("`{set}` is a region, with no surface to measure from: \
                                     measure from the set it is bounded by");
                    return self.once(Code::E040, at, m);
                }
            }
        }
        let lit = surface.as_ref().unwrap_or(&site.lit);
        let cmp = if flipped { cmp.and_then(|c| c.turned()) } else { cmp };
        // the one relation of its point, holding the number its surface is at (a surface's is the
        // region's one)
        let [i] = site.shape.relations[..] else {
            let m = format!("a distance is measured from a surface, and `{set}` says {} things of \
                             its point",
                site.shape.relations.len());
            return self.once(Code::E040, at, m);
        };
        if !written_of(&lit.body[i]).is_some_and(|w| w.dim().is_some()) {
            let m = format!("`{set}` holds no number its surface is at, to measure from");
            return self.once(Code::E040, at, m);
        }
        // the distance, read where the use is — a length, which a bare number is taken as where
        // one is wanted (§3.3) — and signed toward the outside; a text the use's scope wrote over
        // an unknown reads it as declared
        let d = match value_read(&text, &*su.u.scope.vals, &self.unknowns, self.units) {
            Ok(d) if d.dim.fits(crate::units::Dim::LENGTH) => d.as_dim(crate::units::Dim::LENGTH),
            Ok(_) => {
                let m = format!("`{text}` is a distance from `{set}`, a length");
                return self.once(Code::E103, at, m);
            }
            Err(e) => return self.once(Code::E103, at, format!("`{text}`: {e}")),
        };
        // a distance a number comes to nothing (`std.Offset(S, d: 0mm)`, an allowance of none):
        // a bound on it is the set's own side, and a point at it is on the set, as `coincident`
        // says; a zero written so, not by a set's application, is said so
        let written = !rel.space && w.args.iter().any(|a| matches!(a, OpArg::Dim(t, _) if expr::is_literal(t)));
        if d.number() == Some(0.0) && word == "distance" && cmp.is_none() {
            if surface.is_none() && !written {
                let mut on = w.clone();
                on.word.text = "coincident".to_string();
                on.args.clear();
                let rel = Relation { form: RelationForm::Written(on), ..rel.clone() };
                let on = self.set_use_of(su.u.st, &rel, su.u.path, su.u.scope, su.found.clone());
                return self.expand_use(&on);
            }
            let m = format!("a point at no distance from `{set}` is on it: `coincident`");
            return self.once(Code::E040, at, m);
        }
        let distance = written_of(&lit.body[i]).is_some_and(|m| m.word.text == "distance");
        let probe = self.look(alias).raw(&w.ops[1 - k], su.u.scope)
            .is_some_and(|(p, rest)| rest.is_empty() && self.sets.probes.contains(&p));
        let reached = Reached {
            lit: lit.clone(),
            i,
            cmp,
            text,
            d,
            grows,
            closure: site.scope.clone(),
            depth: site.depth,
            worded: Worded { args, ..su.worded.clone() },
            set,
        };
        if distance || probe {
            self.reach_number(su, reached, distance)
        } else {
            self.reach_foot(su, reached)
        }
    }

    /// A distance from one side of another set, `{ q | q outside S }` or `{ q | q inside S }`
    /// (#209): from `S`'s surface, the sides turned round from its outside and as they are from
    /// its inside — `lowered` the use, its side said as a bound.
    fn reach_across(
        &mut self,
        su: &SetUse,
        mut lowered: Written,
        other: String,
        outside: bool,
        alias: &BTreeMap<String, String>,
    ) {
        if outside {
            lowered.turn_side();
        }
        let rel = Relation { form: RelationForm::Written(lowered), ..su.u.rel.clone() };
        let found = vec![(su.set().0, other)];
        let across = self.set_use_of(su.u.st, &rel, su.u.path, su.u.scope, found);
        self.expand_reach(&across, alias)
    }

    /// A reach whose set's number is a **distance**, moved by the offset, or whose point is a
    /// region solid's **probe**, the bound on the set's number carrying it (`OpArg::Bound::grow`).
    fn reach_number(&mut self, su: &SetUse, reached: Reached, distance: bool) {
        use crate::syntax::OpArg;
        let Reached { mut lit, i, cmp, text, d, grows, closure, depth, worded, set } = reached;
        let at = su.u.at;
        // the number moved, or the bound on the number offset
        if !distance && cmp.is_none() {
            let m = format!("a region solid's point is bounded: `inside({text}) {set}`");
            return self.once(Code::E040, at, m);
        }
        let StmtKind::Relation(r) = &mut lit.body[i].kind else { unreachable!("a relation") };
        let measured = r.form.written_mut().expect("a number is written");
        let n = measured.dim_index().expect("a number");
        let mut grow = if grows { d } else { d.times(-1.0) };
        if distance {
            let OpArg::Dim(t, _) = &mut measured.args[n] else { unreachable!("a number") };
            // the number moved, worked out where the set states it: added as numbers, an
            // unknown either reads keeps its dimension, which a text written over it would
            // lose (a free name in a text is a plain number until its slot says otherwise)
            let moved = value_aff(t, &*closure.vals, self.units).ok()
                .filter(|a| a.dim.fits(crate::units::Dim::LENGTH))
                .and_then(|a| a.plus(&grow, 1.0).ok())
                .map(|sum| sum.as_dim(crate::units::Dim::LENGTH));
            match moved {
                Some(sum) => {
                    *t = keys::GROW.to_string();
                    grow = sum;
                }
                None => *t = format!("({t}) + {}", keys::GROW),
            }
        }
        if let Some(cmp) = cmp {
            let carried = (!distance).then(|| (keys::GROW.to_string(), at));
            measured.args.insert(n, OpArg::Bound { cmp, span: at, hi: None, grow: carried });
        }
        let u = &su.u;
        let mut app = self.begin(u, &closure).of_set();
        app.give(keys::GROW, grow);
        let bound = lit.bound.text.clone();
        self.bind_to_use(&app, &bound, su.w.ops[1 - su.set().0].clone(), u);
        let walks = vec![self.inherited_twin(&app, u).into()];
        self.apply(u, app, lit.body, &BTreeMap::new(), &walks, worded, depth);
    }

    /// A reach from a drawn point: a **foot** on the set, at the body's point, the point at the
    /// distance from it on the side named and square to the set there.
    fn reach_foot(&mut self, su: &SetUse, reached: Reached) {
        use crate::syntax::OpArg;
        let Reached { mut lit, cmp, d, grows, closure, depth, worded, set, .. } = reached;
        let (u, at) = (&su.u, su.u.at);
        // a foot on the set: refused where it cannot be one — a choice of root, a claim, a body
        // walked for its derivative, or one making geometry only the point on it would place
        if cmp.is_some() {
            let m = format!("a bound on a point's distance from `{set}` chooses among roots its \
                             foot does not say: bound a region solid's point, or state the \
                             distance");
            return self.once(Code::E040, at, m);
        }
        if self.refused(su, Placed::Foot(&set)) {
            return;
        }
        let k = su.set().0;
        let mut app = self.begin(u, &closure).of_set();
        app.give(keys::REACH, d);
        self.bind_to_use(&app, keys::POINT, su.w.ops[1 - k].clone(), u);
        self.bind_to_use(&app, keys::SET, su.w.ops[k].clone(), u);
        let bound = lit.bound.text.clone();
        // the foot, at the body's point, where the point is to start
        let seed = crate::syntax::AtRef::at(local(keys::POINT, at));
        self.hidden_point(&app, u.st, &bound, seed, at, u.path);
        // the point at the distance from it, on the side named — and the foot, which stands on
        // the surface, seeded on its other side, so the two start apart the way they end; each
        // a statement of the body of its own, so the uses among them are applied apart
        // (`PredWalk::written`), stamped as the use's once walked
        let span = u.st.span;
        let dim = OpArg::Dim(keys::REACH.to_string(), at);
        let ops = vec![local(keys::POINT, at), local(&bound, at)];
        let (side, other) = if grows { ("outside", "inside") } else { ("inside", "outside") };
        let set = local(keys::SET, at);
        let made = [
            relation(("distance", at), Infix, ops, vec![dim], span),
            relation((side, at), Infix, vec![local(keys::POINT, at), set.clone()], vec![], span),
            relation((other, at), Infix, vec![local(&bound, at), set], Vec::new(), span),
        ];
        for (n, r) in made.into_iter().enumerate() {
            let id = crate::syntax::StmtId(FOOT_STATEMENTS - n as u32);
            let kind = StmtKind::Relation(r);
            lit.body.push(Stmt { id, kind, span, chained: Chained::No });
        }
        // the body at the foot, then it and the distance along two directions there
        let along = |k: u8| charted(local(&bound, at), k, keys::foot_chart(app.prefix(), k));
        let walks = [Pass::Itself, along(0), along(1)];
        self.apply(u, app, lit.body, &BTreeMap::new(), &walks, worded, depth);
    }

    /// A set as the statement wrote it: its name, or — written in place — its text, on one line.
    fn set_written(&self, r: &Ref) -> String {
        match self.prog.span_text(r.span).filter(|_| r.root.text.starts_with('#')) {
            Some(text) => text.split_whitespace().collect::<Vec<_>>().join(" "),
            None => ref_text(r),
        }
    }

    /// `l tangent S`'s contact: a point standing in space, on the line, seeded at its middle —
    /// declared at the bound name under the use's prefix, so the body reads it as its point.
    fn contact(&mut self, app: &Application, st: &Stmt, bound: &str, at: Span, path: &[PathStep]) {
        let line = local(keys::LINE, at);
        let end = |e: &str| {
            let mut r = line.clone();
            r.path.push(Seg::Field(Name { text: e.to_string(), span: at }));
            r
        };
        let seed_at = crate::syntax::AtRef {
            toward: Some(end("p2")),
            by: Some(("0.5".to_string(), at)),
            ..crate::syntax::AtRef::at(end("p1"))
        };
        self.hidden_point(app, st, bound, seed_at, at, path);
        let ops = vec![local(bound, at), line];
        let on = relation(("coincident", at), Infix, ops, Vec::new(), st.span);
        let kind = StmtKind::Relation(on);
        let made = Stmt { id: st.id, kind, span: st.span, chained: Chained::No };
        self.out.push((made, path.to_vec(), app.scope().clone()));
    }

    /// A point standing in space at the bound name under the use's prefix — a tangency's contact,
    /// a distance's foot — so the body reads it as its point, seeded at `seed`.
    fn hidden_point(
        &mut self,
        app: &Application,
        st: &Stmt,
        bound: &str,
        seed: crate::syntax::AtRef,
        at: Span,
        path: &[PathStep],
    ) {
        let abs = format!("{}{bound}", app.prefix());
        self.names.insert(abs.clone());
        let key = Name { text: abs, span: Span::at(at.lo as usize) };
        let decl = Decl::point(DeclName::Key(key), [0.0; 2], Some(seed));
        let kind = StmtKind::Decl(decl);
        let made = Stmt { id: st.id, kind, span: st.span, chained: Chained::No };
        self.out.push((made, path.to_vec(), app.scope().clone()));
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

/// A relation a set's use states, written as the source would: its word and where the word is
/// said, its fixity, operands and parentheses, and the statement's span.
pub(super) fn relation(
    (word, at): (&str, Span),
    fixity: crate::constraints::Fixity,
    ops: Vec<Ref>,
    args: Vec<crate::syntax::OpArg>,
    span: Span,
) -> Relation {
    let word = Name { text: word.to_string(), span: at };
    Relation::of(RelationForm::Written(crate::syntax::Written {
        word, fixity, ops, args, span, rates: Vec::new(),
    }))
}

/// The point a tangency names, `tangent(at: m)` — `None` where its parentheses hold anything else.
fn at_point(w: &crate::syntax::Written) -> Option<Ref> {
    match w.args.as_slice() {
        [crate::syntax::OpArg::Named(n, v)] if n.text == "at" => match v {
            crate::syntax::Arg::Ref(r) => Some(r.clone()),
            crate::syntax::Arg::Word(x) => Some(local(x.clone(), n.span)),
            _ => None,
        },
        _ => None,
    }
}

/// Whether a written relation is one a set may be used by: `coincident`, `tangent`, `inside` or
/// `outside` between two operands, or a distance from one (`reaches`).
pub(super) fn may_use(w: &crate::syntax::Written) -> bool {
    w.ops.len() == 2
        && (matches!(w.word.text.as_str(), "coincident" | "tangent" | "inside" | "outside")
            || reaches(w))
}

/// Whether a written relation measures from a set's surface (F4): a distance with a side only a
/// surface has (`side: outside`) — so a plain distance never enters the walk of a set's uses.
fn reaches(w: &crate::syntax::Written) -> bool {
    w.word.text == "distance" && matches!(w.sel("side").as_deref(), Some("outside" | "inside"))
}

/// The set a set is one side of, as its body writes it, and whether it is the outside: `{ q | q
/// outside S }` (its body one bounded use of `S`, no number of its own) — `None` for any other.
fn side_of(lit: &SetLit) -> Option<(Ref, bool)> {
    let [st] = lit.body.as_slice() else { return None };
    let w = written_of(st)?;
    if !may_use(w) || w.dim().is_some() {
        return None;
    }
    let (cmp, _) = w.bound()?;
    let bound = &lit.bound.text;
    let other = match (&w.ops[0], &w.ops[1]) {
        (q, s) | (s, q) if q.path.is_empty() && q.root.text == *bound => s,
        _ => return None,
    };
    Some((other.clone(), cmp == Cmp::Ge))
}

/// **A family's body, split by its point** (§6.21, F3): what reads the bound point — through
/// the declarations that read it — is the set, and what reads none of it is the instance's,
/// made once where the instance is rather than at each use (`std.ConeThrough`'s apex, placed by
/// the point it is given).  Only declarations and written relations are judged; anything else
/// stays with the set.  Each part keeps the order written.
pub(super) fn instance_part(lit: &SetLit) -> (Vec<Stmt>, SetLit) {
    use crate::syntax::{Arg, OpArg};
    // the names a statement reads, by their roots, and the name it defines — `None` for one
    // the split does not judge
    fn reads(st: &Stmt) -> Option<(Vec<&str>, Option<&str>)> {
        match &st.kind {
            StmtKind::Decl(d) if d.computed.is_none() && d.curve.is_none() => {
                let mut out: Vec<&str> = Vec::new();
                for kid in d.children.iter().flatten() {
                    match kid {
                        Kid::Ref(r) => out.push(&r.root.text),
                        Kid::Hint(ks) => {
                            out.extend(ks.dir.iter().flat_map(|d| d.refs()).map(|r| r.root.text.as_str()));
                        }
                        _ => return None,
                    }
                }
                if let Some(at) = &d.seed_at {
                    out.extend([Some(&at.what), at.toward.as_ref(), at.along.as_ref()]
                        .into_iter().flatten().map(|r| r.root.text.as_str()));
                }
                out.extend(d.membership.plane().into_iter().chain(d.membership.also())
                    .map(|r| r.root.text.as_str()));
                Some((out, Some(&d.name.key().text)))
            }
            StmtKind::Relation(r) => {
                let w = r.form.written()?;
                let mut out: Vec<&str> = w.ops.iter().map(|r| r.root.text.as_str()).collect();
                for a in &w.args {
                    match a {
                        OpArg::Ent(r) | OpArg::Named(_, Arg::Ref(r))
                        | OpArg::Slot { arg: Arg::Ref(r), .. } => out.push(&r.root.text),
                        OpArg::Named(_, Arg::Word(x)) => out.push(x),
                        _ => {}
                    }
                }
                Some((out, None))
            }
            _ => None,
        }
    }
    let judged: Vec<_> = lit.body.iter().map(reads).collect();
    // the names that read the point, grown until no statement adds one
    let mut reading: BTreeSet<&str> = BTreeSet::from([lit.bound.text.as_str()]);
    let mut set = vec![false; lit.body.len()];
    let mut grew = true;
    while grew {
        grew = false;
        for (k, j) in judged.iter().enumerate() {
            let on_point = |(ns, _): &(Vec<&str>, _)| ns.iter().any(|n| reading.contains(n));
            if !set[k] && j.as_ref().is_none_or(on_point) {
                set[k] = true;
                grew = true;
                reading.extend(j.as_ref().and_then(|(_, d)| *d));
            }
        }
    }
    let (body, own): (Vec<_>, Vec<_>) = lit.body.iter().cloned().zip(set).partition(|(_, s)| *s);
    let stmts = |v: Vec<(Stmt, bool)>| v.into_iter().map(|(st, _)| st).collect();
    (stmts(own), SetLit { bound: lit.bound.clone(), body: stmts(body), span: lit.span })
}

/// A statement's written relation, where it is one.
fn written_of(st: &Stmt) -> Option<&crate::syntax::Written> {
    match &st.kind {
        StmtKind::Relation(r) => r.form.written(),
        _ => None,
    }
}

/// A set's body as `inside` or `outside` reads it (§6.21, §9.6), rewritten in place: its **one
/// dimension** a bound running `side` (`inside` at most its number, `outside` at least), a
/// region's own bound kept or turned round, and every other statement as written, so a disc is
/// its circle's plane and a bound.  A body stating no number of its own is read as written by
/// `inside`; `outside` turns round its one side of another set (`{ q | q outside toe }`, `{ q |
/// q distance(<= 1mm, side: outside) S }`), and of several is no complement it can say: that is
/// a union, a choice of root.  Every side is a bound here (`lowered`), so turning one is turning
/// its `Cmp`.  `at` is the use's word, where the bound is written; `shape` is the body's, as its
/// site read it.
fn region_body(
    body: &mut [Stmt],
    shape: &BodyShape,
    side: Cmp,
    word: &str,
    at: Span,
    name: &str,
) -> Result<(), String> {
    let measured = |st: &Stmt| matches!(st.kind, StmtKind::Measured(_));
    let dims = &shape.numbers;
    let either = || format!(
        "outside `{name}`'s interval is either side of it: a choice of root, not a region \
         `outside` can say"
    );
    // every number a bound already: `inside` is where all of them hold, as written
    let bounded = |st: &Stmt| measured(st) || written_of(st).is_some_and(|w| w.bound().is_some());
    if dims.len() > 1 && side == Cmp::Le && dims.iter().all(|&i| bounded(&body[i])) {
        return Ok(());
    }
    // the outside of a set of one side of another is that side turned round: outside `{ q | q
    // outside toe }` is inside `toe` (#183, a region's own side as `std.Chamfered` takes it),
    // outside `{ q | q distance(<= 1mm, side: outside) S }` beyond `S` grown (#192)
    if dims.is_empty() && side == Cmp::Ge {
        if let ([i], 1) = (shape.uses.as_slice(), shape.relations.len()) {
            if let StmtKind::Relation(r) = &mut body[*i].kind {
                if let Some(w) = r.form.written_mut().filter(|w| w.bound().is_some()) {
                    return w.turn_bound().ok_or_else(either);
                }
            }
        }
    }
    let [i] = dims[..] else {
        let uses = !shape.uses.is_empty();
        return match (dims.len(), uses, side) {
            (0, true, Cmp::Le) => Ok(()),
            (0, true, _) => Err(format!(
                "`{name}` is where several things hold, and what is outside it is where any one \
                 fails: a choice of root, not a region `outside` can say"
            )),
            (0, false, _) => Err(format!("`{name}` states no number for `{word}` to bound")),
            (n, _, _) => Err(format!(
                "`{name}` states {n} numbers, and `{word}` bounds one: say which with a set of one"
            )),
        };
    };
    if let StmtKind::Measured(m) = &mut body[i].kind {
        if side != Cmp::Le {
            m.cmp = m.cmp.turned().expect("a bound on measurements is one-sided");
        }
        return Ok(());
    }
    let StmtKind::Relation(r) = &mut body[i].kind else {
        unreachable!("a dimension is a relation")
    };
    let w = r.form.written_mut().expect("a dimension is written");
    if w.bound().is_none() {
        let k = w.dim_index().expect("a dimension's number");
        w.args.insert(k, crate::syntax::OpArg::Bound { cmp: side, span: at, hi: None, grow: None });
        return Ok(());
    }
    // a region already: `inside` keeps its bound, `outside` turns it round
    match side {
        Cmp::Le => Ok(()),
        _ => w.turn_bound().ok_or_else(either),
    }
}

/// A region of one bounded relation as its surface (#183): the body with the bound made the
/// equality (`Written::equality`), and whether the bound ran `>=` (its outside the smaller side).
fn one_bound(site: &Site) -> Option<(SetLit, bool)> {
    let [i] = site.shape.relations[..] else { return None };
    let mut bare = site.lit.clone();
    let StmtKind::Relation(r) = &mut bare.body[i].kind else { return None };
    let w = r.form.written_mut()?;
    let cmp = w.equality()?;
    w.dim().is_some().then_some((bare, cmp == Cmp::Ge))
}

/// **A probe** under `key`: a point standing in space, held at a place no drawing chooses
/// (irrational, off every axis and plane through the origin), so a cone's line from an apex there
/// to it has a length; where it stands decides nothing else — its bounds are terms, never rows.
/// Not `scatter`'s unseeded place: an empty hint span would be written back into the statement
/// that made it.
pub(super) fn probe(key: String, at: Span) -> Decl {
    let mut point = Decl::point(DeclName::Key(Name { text: key, span: at }), [0.0; 2], None);
    point.seed = vec![0.7548776662, 0.5698402910, 0.4301597090];
    point.seed_text = vec![None; 3];
    point
}

/// The statement ids a distance's foot gives what it adds to the set's body (`expand_reach`), so
/// each is applied under a prefix of its own: past any a program numbers.
const FOOT_STATEMENTS: u32 = u32::MAX;

/// A set's body walked along the `k`th of two directions solved for at `point` (`Toward::Chart`).
fn charted(point: Ref, k: u8, key: String) -> Pass {
    Pass::Along(Along { point, toward: AlongBy::Chart(k), key, made: None })
}

/// What a distance from a set, or a side of one grown, says (§6.21, F4): the bound on the signed
/// distance from its surface toward its outside (`None` an equation), the distance as written,
/// whether the offset grows the set (toward its outside) or shrinks it, and its parentheses as
/// the culprit reads them.
struct Reach {
    cmp: Option<crate::syntax::Cmp>,
    text: String,
    grows: bool,
    args: String,
}

/// `p distance(d, side: outside) S`, `p distance(<= d, side: inside) S`, … — `side:` said,
/// since a surface has two (`reaches`).
fn reach_of(w: &crate::syntax::Written, set: &str) -> Result<Reach, String> {
    use crate::syntax::{Arg, OpArg};
    let (mut cmp, mut text) = (None, None);
    for a in &w.args {
        match a {
            OpArg::Bound { hi: Some(_), .. } => {
                return Err(format!("a distance from `{set}` is bounded on one side of a number"))
            }
            OpArg::Bound { cmp: c, .. } => cmp = Some(*c),
            OpArg::Dim(t, _) => text = Some(t.clone()),
            OpArg::Named(n, Arg::Word(_)) if n.text == "side" => {}
            _ => {
                return Err(format!("a distance from `{set}` takes its number and `side: \
                                    outside` or `side: inside`"))
            }
        }
    }
    let Some(text) = text else { return Err(format!("a distance from `{set}` states a number")) };
    let grows = w.sel("side").as_deref() == Some("outside");
    // a bound on the distance inside is the other way on the distance outside
    let said = cmp.map_or(String::new(), |c| format!("{} ", c.text()));
    let side = if grows { "outside" } else { "inside" };
    Ok(Reach {
        cmp: if grows { cmp } else { cmp.and_then(|c| c.turned()) },
        args: format!("{said}{text}, side: {side}"),
        text,
        grows,
    })
}
