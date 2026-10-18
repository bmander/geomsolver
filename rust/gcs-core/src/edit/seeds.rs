//! Writing a solve back: the seeds a solve moved, spliced into their `hint(…)` clauses.

use super::*;

/* -- writing a solve back ---------------------------------------------------------- */

/// Put the solved coordinates back into the seeds they came from.
///
/// > A seed is writable iff it is inside a `hint(…)` clause, is a literal and not an expression,
/// > and is reached by exactly one instance path.
///
/// The first is lexical, which is what makes this a test rather than an analysis: a number in a
/// `hint(…)` is one a solve may move, and every other number — `== 80`, `w := 100` — is not.
/// The second keeps `hint(r: Rr)` — a radius written in terms of a component's parameter — from
/// being overwritten with the number it happened to come to.  The third is why a point inside a
/// `cycle` of thirty does not write back at all: thirty instances share one statement, and there
/// is no one pose to record.
///
/// A seed the source **never wrote** is the case the clause makes real: a radius is a seed a
/// person may perfectly well omit, and a solve moves it anyway.  There is
/// then no span to splice, so the clause is written out whole at the point the parser recorded
/// for it (`Decl::hint_span`) — one splice, and the statement around it untouched.  Leaving it
/// alone instead would mean a drawing whose pose its source cannot express. A driving radius
/// dimension already records that scalar, so it does not need an automatically added hint.
///
/// `Kind::Numeric`, always: a seed is not a statement, so nothing recompiles.
pub fn commit_seeds(e: &Elaborated, sk: &Sketch, prog: &Program) -> Edit {
    // a drawing shown at an instant of its march is not where it is drawn (#398 L): the seeds say
    // where it starts, and what is shown is presentation
    if sk.standing.is_some() {
        return Edit::none(prog, None);
    }
    // The walk is over the root component's own statements, which is the question `in_root` was
    // asking one statement at a time: a statement inside a component is written in the
    // component's terms, and a pose put there is a pose put on every instance of it.  What each
    // one made is `SourceMap::ents_made_by`, whose order — the declaration's own entity first,
    // then the children it minted — is what `reconcile` reads too, so neither the entity index
    // nor the find-by-kind that re-derived the parent is needed.
    let radii = dimensioned_radii(sk);
    let mut edits = Vec::new();
    for (st, nested) in seeded_statements(prog) {
        let StmtKind::Decl(d) = &st.kind else { continue };
        // `hint(at: t)` names a *place*, and has no coordinates to write.  Faces and
        // solids also own no seeds: their children are boundaries and operands, not points.
        if d.seed_at.is_some() || d.kind.spatial() {
            continue;
        }
        if nested && !one_path(e, st.id) {
            continue;
        }
        // a declaration that could not be built made nothing, and has no pose to record
        let Some(parent) = e.map.ents_made_by(st.id).next() else { continue };
        edits.extend(DeclPose::new(sk, prog, st, d, parent, &radii).splices(nested));
    }
    edits.extend(unknown_seeds(sk, prog, &e.map));
    if edits.is_empty() {
        return Edit::none(prog, None);
    }
    Edit::spliced(prog, edits, Kind::Numeric)
}

/// What a driving radius dimension holds: it already records the scalar.  Hints are kept for
/// free radii and claims, which do not determine it, and so are hints the author wrote.
fn dimensioned_radii(sk: &Sketch) -> std::collections::BTreeSet<EntRef> {
    sk.constraints.iter()
        .filter(|c| c.kind == crate::constraints::CKind::Radius
            && c.acts() && c.free.is_none())
        .map(|c| c.args[0].ent()).collect()
}

/// The statements a solve's pose is written into, each with whether it is a definition's
/// (`nested`) rather than the root's.
fn seeded_statements(prog: &Program) -> Vec<(&Stmt, bool)> {
    // and a `ring`'s body at the root: one statement however many copies, but the copies are
    // turns of the first, so the first's pose is the one there is to record (§12.4); and a
    // `when`'s, made once if at all, so its statements are the root's (#210)
    fn written<'a>(body: &'a [syntax::Stmt], out: &mut Vec<&'a syntax::Stmt>) {
        for st in body {
            match &st.kind {
                StmtKind::Block(b) if b.kind == syntax::BlockKind::Ring => out.extend(&b.body),
                StmtKind::Block(b) if b.kind == syntax::BlockKind::When => written(&b.body, out),
                _ => out.push(st),
            }
        }
    }
    let mut root = Vec::new();
    written(&prog.root().body, &mut root);
    // and a component of the document's own that is drawn once: one instance path reaches each
    // of its statements, so the pose there is that instance's and its text the one spelling of
    // it — what Make component leaves behind.  Only a seed the definition already writes is
    // spliced; a clause it left out is not written into it (`missing` below), so a `Rectangle`
    // the Rect tool defined stays the bare chain it is
    let mut inner = Vec::new();
    for c in prog.components.iter().filter(|c| c.module.is_none() && !std::ptr::eq(*c, prog.root()))
    {
        written(&c.body, &mut inner);
    }
    root.into_iter().map(|st| (st, false)).chain(inner.into_iter().map(|st| (st, true))).collect()
}

/// One declaration's pose beside its text: what it made, and what its statement wrote.
struct DeclPose<'a> {
    sk: &'a Sketch,
    prog: &'a Program,
    st: &'a Stmt,
    d: &'a Decl,
    parent: EntRef,
    kids: Vec<EntRef>,
    /// A radius a driving dimension holds and the clause never wrote: nothing to record.
    omit_radius: bool,
}

/// The splices recording a pose where the source wrote its seeds, and what they left over.
struct InPlace {
    mine: Vec<Splice>,
    /// Something moved that the source never wrote, so there is nowhere to splice it.
    missing: bool,
    /// The clause writes a number a `fix` holds.
    redundant: bool,
}

impl<'a> DeclPose<'a> {
    fn new(
        sk: &'a Sketch,
        prog: &'a Program,
        st: &'a Stmt,
        d: &'a Decl,
        parent: EntRef,
        radii: &std::collections::BTreeSet<EntRef>,
    ) -> Self {
        let omit_radius =
            radii.contains(&parent) && d.seed_spans.first().is_none_or(|span| span.is_empty());
        DeclPose { sk, prog, st, d, parent, kids: sk.children(parent), omit_radius }
    }

    /// The statement's slot for each child, in field order — the same order `sk.children` hands
    /// them back in.  A slot may be empty (an implicit child), so for the per-slot kinds the
    /// groups are indexed directly; a `List` kind's one group is flattened.
    fn slot_kid(&self, j: usize) -> Option<&'a syntax::Kid> {
        let d = self.d;
        if d.children.len() == self.kids.len() {
            d.children.get(j).and_then(|g| g.first())
        } else {
            d.children.iter().flatten().nth(j)
        }
    }

    /// Whether a kid's text is this statement's own: a chain's thread fills slots with references
    /// written in *another* link (or written nowhere, with an empty span), and a writeback must
    /// not mistake those for a list the source wrote here.
    fn written_here(&self, kid: &syntax::Kid) -> bool {
        match kid {
            syntax::Kid::Ref(r) => {
                !r.span.is_empty() && r.span.lo >= self.st.span.lo && r.span.hi <= self.st.span.hi
            }
            syntax::Kid::Hint(_) | syntax::Kid::Face { .. } | syntax::Kid::Trim { .. } => true,
        }
    }

    /// One seed at a time: where the source wrote it, the splice that records it; where it did
    /// not, the news that something has moved with nowhere to write it.
    fn one(&self, v: f64, text: Option<&String>, span: Span) -> (Option<Splice>, bool) {
        if text.is_some() {
            return (None, false); // an expression: a solve does not rewrite arithmetic
        }
        let now = num(v);
        if span.is_empty() {
            // an omitted scalar reads as 0, so it needs recording only when it is not 0
            (None, v != 0.0)
        } else if span.slice(self.prog.text()) != now {
            (Some(Splice { at: span, with: now }), false)
        } else {
            (None, false)
        }
    }

    /// A number a `fix` holds is stated there, and is its own seed: no seed of it is written, or
    /// counted missing — and one the clause wrote is taken out of it (`fix(x == 3) p` makes
    /// `hint((3, 7))` say `hint(y: 7)`).
    fn held(&self, p: u32) -> bool {
        self.sk.params[p as usize].fixed
    }

    /// Whether the clause wrote the declaration's own `i`th number.
    fn written(&self, i: usize) -> bool {
        self.d.seed_spans.get(i).is_some_and(|s| !s.is_empty())
    }

    /// A child's seed is a point's place, or an axis's direction (a plane's `u:` and `v:`).
    fn seeded(&self, k: EntRef) -> Vec<u32> {
        match k.kind {
            EntKind::Point => self.sk.point_params(k.i()).to_vec(),
            EntKind::Axis => self.sk.axes[k.i()].d.to_vec(),
            _ => Vec::new(),
        }
    }

    /// What this declaration gives the writeback: each seed spliced where the source wrote it,
    /// and where it did not, the clause (or the list) written out.
    fn splices(&self, nested: bool) -> Vec<Splice> {
        let InPlace { mine, missing, redundant } = self.in_place();
        if !missing {
            return self.without_held(mine, redundant);
        }
        // inside a definition, only what it already wrote
        if nested {
            return mine;
        }
        self.unwritten(mine, redundant)
    }

    /// Each seed the solve moved, spliced where the source wrote it: the declaration's own
    /// numbers, then each anonymous child's.
    fn in_place(&self) -> InPlace {
        let (sk, d, parent) = (self.sk, self.d, self.parent);
        let mut mine: Vec<Splice> = Vec::new();
        let mut missing = false;
        let mut redundant = false;
        for (i, p) in sk.own_params(parent).iter().enumerate() {
            if self.omit_radius { continue; }
            if self.held(*p) {
                redundant |= self.written(i);
                continue;
            }
            let v = sk.seed_value(parent, *p);
            let text = d.seed_text.get(i).and_then(|t| t.as_ref());
            let (sp, miss) = self.one(v, text, d.seed_spans.get(i).copied().unwrap_or_default());
            mine.extend(sp);
            missing |= miss;
        }
        // An anonymous child's seed lives in the parent's statement, in a slot of its own — and
        // the slots stand in the order `sk.children` hands the children back in, so the two walk
        // together.  A slot the source wrote a *name* in is a point declared elsewhere, and is
        // written back where it was declared; one it wrote nothing for at all was minted.
        for (j, &k) in self.kids.iter().enumerate() {
            if self.seeded(k).iter().all(|&p| self.held(p)) {
                continue;
            }
            let seed = match self.slot_kid(j) {
                Some(syntax::Kid::Ref(_) | syntax::Kid::Face { .. } | syntax::Kid::Trim { .. }) => continue,
                // a direction read off geometry is a seed given as geometry, as a place is: a
                // solve writes nothing back over it
                Some(syntax::Kid::Hint(s)) if s.dir.is_some() => continue,
                Some(syntax::Kid::Hint(s)) => Some(s),
                None => None,
            };
            let v: Vec<f64> =
                self.seeded(k).iter().map(|&p| sk.params[p as usize].value).collect();
            match seed {
                // A slot that keyed one coordinate and left the other out has nowhere to splice
                // the one it left out, and the clause it is written in is the smallest thing
                // that can carry both — which is what `KidSeed::span` is for.  Not when a
                // coordinate is an expression: rewriting the clause would rewrite the arithmetic.
                Some(s)
                    if s.spans.iter().any(|sp| sp.is_empty())
                        && s.text.iter().all(|t| t.is_none()) =>
                {
                    let now = syntax::hint_numbers(&v);
                    if s.span.slice(self.prog.text()) != now {
                        mine.push(Splice { at: s.span, with: now });
                    }
                }
                Some(s) => {
                    for i in 0..v.len() {
                        let (sp, miss) = self.one(v[i], s.text[i].as_ref(), s.spans[i]);
                        mine.extend(sp);
                        missing |= miss;
                    }
                }
                None => missing |= v.iter().any(|&c| c != 0.0),
            }
        }
        InPlace { mine, missing, redundant }
    }

    /// Nothing is missing: the splices in place, and the clause rewritten without what a `fix`
    /// holds — the numbers it keeps as they stand (spliced in place where the solve moved one),
    /// and the whole clause gone with the space before it where it keeps none.
    fn without_held(&self, mut mine: Vec<Splice>, redundant: bool) -> Vec<Splice> {
        let (sk, d, parent) = (self.sk, self.d, self.parent);
        if let Some(at) = d.hint_span.filter(|at| redundant && !at.is_empty()) {
            let own = sk.own_params(parent);
            let free = own.iter().enumerate()
                .filter(|&(i, &p)| self.written(i) && !self.held(p))
                .map(|(i, &p)| (i, sk.seed_value(parent, p)));
            let hint = free_clause(parent.kind, d, self.prog.text(), free);
            mine.retain(|s| s.at.lo >= at.hi || s.at.hi <= at.lo);
            mine.push(clause_splice(self.prog.text(), at, (!hint.is_empty()).then_some(hint)));
        }
        mine
    }

    /// Something the source never wrote has moved — an omitted radius, an endpoint of a bare
    /// `l := line`.  There is nowhere to splice, so what the source left out is written: the
    /// argument list, the `hint(…)` clause, or both.
    fn unwritten(&self, mine: Vec<Splice>, redundant: bool) -> Vec<Splice> {
        let d = self.d;
        let Some(at) = d.hint_span else { return Vec::new() };
        let hint = self.pose_clause();
        let list = self.list();
        // a clause the source wrote whose every number a `fix` now holds goes, with the space
        // before it
        let gone = (redundant && hint.is_empty() && !at.is_empty())
            .then(|| clause_splice(self.prog.text(), at, None));
        if list.is_none() && gone.is_none() && hint.is_empty() {
            // nothing to write here: what moved is in a slot the source wrote, and splices there
            return mine;
        }
        // a slot the source *did* write still splices in place; only what it did not is here
        let mut edits: Vec<Splice> =
            mine.into_iter().filter(|s| s.at.lo >= at.hi || s.at.hi <= at.lo).collect();
        if let Some(gone) = gone {
            edits.push(gone);
            if let Some(args) = list {
                edits.push(Splice { at: d.list_span, with: args });
            }
            return edits;
        }
        match list {
            // Both are missing and both would go at the same offset — the parser records the
            // clause's home just past the name when there is no clause — so they are written as
            // one edit.  Two insertions at one position would race for it.
            Some(args) if at.is_empty() && d.list_span.is_empty() => {
                let with = if hint.is_empty() { args } else { format!("{args} {hint}") };
                edits.push(Splice { at, with })
            }
            // The clause has a home of its own, and the *list* belongs to the name: written at
            // the clause's position it would land past whatever trailer stands between them,
            // where an argument list is not a thing a declaration can say.  The list is
            // *replaced* where one stands — a list none of whose slots this statement wrote is
            // still a list, and a second beside the first would be two — and inserted at the
            // name's end where none does.
            Some(args) => {
                edits.push(Splice { at: d.list_span, with: args });
                if !hint.is_empty() {
                    let with = if at.is_empty() { format!(" {hint}") } else { hint };
                    edits.push(Splice { at, with });
                }
            }
            // an insertion has to bring the space that separates it from the statement; a
            // replacement stands between the two the clause already had
            None => {
                let with = if at.is_empty() { format!(" {hint}") } else { hint };
                edits.push(Splice { at, with });
            }
        }
        edits
    }

    /// The clause, as the pose the solve arrived at; empty when the kind owns no scalar at all —
    /// a line's numbers are its two points', and they are written in the slots.
    fn pose_clause(&self) -> String {
        let (sk, d, parent) = (self.sk, self.d, self.parent);
        let mut pose = d.seed.clone();
        for (i, p) in sk.own_params(parent).iter().enumerate() {
            if let Some(v) = pose.get_mut(i) {
                *v = sk.seed_value(parent, *p);
            }
        }
        let own = sk.own_params(parent);
        if self.omit_radius {
            String::new()
        } else if own.iter().any(|&p| self.held(p)) {
            // only the numbers no `fix` holds: `hint(y: 7)` beside `fix(x == 3) p`, and none at
            // all where every one is held
            let free = own.iter().zip(&pose).enumerate()
                .filter(|&(_, (&p, _))| !self.held(p))
                .map(|(i, (_, &v))| (i, v));
            free_clause(parent.kind, d, self.prog.text(), free)
        } else {
            syntax::hint_clause(d, &pose)
        }
    }

    /// The argument list, where no slot of it is the source's own text (`None` where one is, or
    /// where the kind has no children).  A chain's thread fills slots with references written in
    /// *another* link, or written nowhere at all, and neither is a list this statement can
    /// splice into.  It is spelled by the printer that spells every other statement — write it
    /// here and an `arc`'s `center:`/`start:`/`end:` labels are dropped by the one path that
    /// wrote its own — and a `Decl` of the solved pose is what that printer takes.
    fn list(&self) -> Option<String> {
        let (sk, d, st) = (self.sk, self.d, self.st);
        let mine_here = d.children.iter().flatten().any(|kid| self.written_here(kid));
        if mine_here || self.kids.is_empty() {
            return None;
        }
        let mut d2 = d.clone();
        // a slot the thread filled keeps the name it threaded — unless that name is one the
        // source cannot write (an anonymous link's `#`-keyed boundary), in which case the
        // slot is left empty: the marker threads it again on the next parse, which keeps
        // the weld the corner's and the pose the owning link's.  Every other slot is a
        // child this statement minted, and is written as the `hint(…)` its pose is.
        // Leaving a slot empty is safe only because a chain's marker threads it again on
        // the next parse — so the statement must *be* in a chain.  Nothing else can put a
        // reference the source cannot write into a slot, and `Chained` records that rather
        // than sniffing it back out of the text; asserted, since a silent violation would
        // be a slot nothing refills and a point that reseeds from `scatter`.
        debug_assert!(
            !matches!(st.chained, syntax::Chained::No)
                || !d.children.iter().flatten().any(|kid| match kid {
                    syntax::Kid::Ref(r) => syntax::hidden(&r.root.text),
                    syntax::Kid::Hint(_) | syntax::Kid::Face { .. } | syntax::Kid::Trim { .. } => false,
                }),
            "an unwritable reference outside a chain has nothing to re-thread its slot",
        );
        let mut filled = self.kids.iter().enumerate().map(|(j, k)| match self.slot_kid(j) {
            Some(syntax::Kid::Ref(r)) if syntax::hidden(&r.root.text) => None,
            // a child a `fix` holds is placed there, and its slot is left empty
            None if self.seeded(*k).iter().all(|&p| self.held(p)) => None,
            Some(syntax::Kid::Ref(r)) => Some(syntax::Kid::Ref(r.clone())),
            Some(syntax::Kid::Hint(s)) if s.dir.is_some() => Some(syntax::Kid::Hint(s.clone())),
            _ => {
                let mut v = [0.0; 3];
                for (x, p) in v.iter_mut().zip(self.seeded(*k)) {
                    *x = sk.params[p as usize].value;
                }
                let axis = k.kind == EntKind::Axis;
                Some(syntax::Kid::Hint(syntax::KidSeed { v, axis, ..Default::default() }))
            }
        });
        for g in d2.children.iter_mut() {
            *g = filled.by_ref().take(g.len().max(1)).flatten().collect();
        }
        Some(syntax::decl_args(&d2))
    }
}

/// Whether a statement is reached by exactly one instance path — the third clause of the
/// writeback rule, asked of a statement in a component's body: everything it made was made under
/// one path, and no block's copy is on it (a block's copies are one statement however many).
fn one_path(e: &Elaborated, id: syntax::StmtId) -> bool {
    let mut paths =
        e.map.ents_made_by(id).filter_map(|r| e.map.of_entity.get(&r)).map(|s| &s.path.0);
    match paths.next() {
        Some(first) => {
            !first.iter().any(|p| matches!(p, crate::ir::PathStep::Copy { .. }))
                && paths.all(|p| p == first)
        }
        None => false,
    }
}

/// A `hint(…)` clause of the numbers given, each by its field's name: a seed written as an
/// expression kept as written (a solve does not rewrite arithmetic), any other as the number it
/// came to.  Empty where none is given.
fn free_clause(kind: EntKind, d: &Decl, text: &str, free: impl Iterator<Item = (usize, f64)>) -> String {
    let free: Vec<(usize, f64)> = free.filter(|&(i, _)| i < kind.members().len()).collect();
    let given: Vec<usize> = free.iter().map(|&(i, _)| i).collect();
    let value = |i: usize| {
        let v = free.iter().find(|&&(j, _)| j == i).map_or(0.0, |&(_, v)| v);
        match (d.seed_text.get(i).and_then(|t| t.as_ref()), d.seed_spans.get(i)) {
            (Some(_), Some(s)) if !s.is_empty() => s.slice(text).to_string(),
            _ => num(v),
        }
    };
    let point = if given.contains(&2) { 3 } else { 2 };
    let keys = syntax::said_text(kind, point, &given, value, ": ");
    if keys.is_empty() { String::new() } else { format!("hint({})", keys.join(", ")) }
}

/// Each unknown the document declares (`param beta: Angle hint(30deg)`, or a call's formal left
/// unbound and seeded, `Cone(gax, half: hint(30deg))`), read by a dimension or shared by
/// contacts, its seed written as the solve left it: spliced over a literal seed, or a clause
/// written where none was — and only where the solve moved it, so an unread unknown and an
/// unmoved one change nothing.  A seed written as an expression is the author's arithmetic, and is
/// left alone.
fn unknown_seeds(sk: &Sketch, prog: &Program, map: &crate::program::SourceMap) -> Vec<Splice> {
    let solved = |name: &str| {
        let p = sk.free_vars.get(name).copied().or(sk.shared.get(name).map(|s| s.param))?;
        Some(sk.params[p as usize].value)
    };
    let mut out = Vec::new();
    for st in &prog.root().body {
        match &st.kind {
            StmtKind::Param(p) => {
                let Some(input) = p.input.as_ref().filter(|_| !p.bound()) else { continue };
                let Some(v) = solved(&p.name.text) else { continue };
                let angle = input.ty == Some(syntax::Ty::Angle);
                match &input.seed {
                    Some((text, span)) => out.extend(seed_splice(prog, text, *span, v, angle)),
                    None if v != 0.0 => {
                        out.push(Splice { at: p.span, with: format!(" hint({})", num(v)) });
                    }
                    None => {}
                }
            }
            // a number is given by label, so a seeded formal is found by its own
            StmtKind::Instance(inst) => {
                let Ok(c) = prog.resolve_component(&inst.component.text, None) else { continue };
                for a in &inst.args {
                    let (Some(l), syntax::InstVal::Hint(text, span)) = (&a.label, &a.value) else {
                        continue;
                    };
                    let Some(f) = prog.components[c].formals.iter().find(|f| f.name.text == l.text)
                    else {
                        continue;
                    };
                    // a point the instance owns, seeded where the call wrote its place (§5 [0.95])
                    if f.within.is_some() {
                        let named = map.ent_named(&format!("{}.{}", inst.name.text, l.text));
                        out.extend(named.and_then(|e| owned_splice(sk, prog, text, *span, e)));
                        continue;
                    }
                    let Some(v) = solved(&format!("{}.{}", inst.name.text, l.text)) else { continue };
                    out.extend(seed_splice(prog, text, *span, v, f.ty == syntax::Ty::Angle));
                }
            }
            _ => {}
        }
    }
    out
}

/// A point an instance owns, its seed `(x, y)` at `span` rewritten to where the solve left it in
/// its plane — `None` where it reads the same, a `fix` holds it, or a coordinate is no literal
/// this can write, as `seed_splice` says of a number.
fn owned_splice(sk: &Sketch, prog: &Program, text: &str, span: Span, e: EntRef)
    -> Option<Splice> {
    let inner = text.trim().strip_prefix('(')?.strip_suffix(')')?;
    let parts: Vec<&str> = inner.split(',').map(str::trim).collect();
    let params = sk.point_params(e.i());
    let plain = |t: &&str| writable_seed(t) && !crate::expr::names_unit(t);
    let held = params.iter().any(|&p| sk.params[p as usize].fixed);
    if e.kind != EntKind::Point || parts.len() != 2 || params.len() != 2
        || !parts.iter().all(plain) || held || span.is_empty()
    {
        return None;
    }
    let [x, y] = [0, 1].map(|k| sk.params[params[k] as usize].value);
    let with = format!("({}, {})", num(x), num(y));
    (span.slice(prog.text()) != with).then_some(Splice { at: span, with })
}

/// A literal seed, `text` at `span`, rewritten to the value `v` a solve left its unknown at —
/// `None` where it reads the same, or is not a literal this can write: an expression, or a length
/// written in another unit (`2in` in an `mm` document), each left as written.
fn seed_splice(prog: &Program, text: &str, span: Span, v: f64, angle: bool) -> Option<Splice> {
    if !writable_seed(text) || !angle && crate::expr::names_unit(text) || span.is_empty() {
        return None;
    }
    let with = seed_literal(text, v, angle);
    (span.slice(prog.text()) != with).then_some(Splice { at: span, with })
}
