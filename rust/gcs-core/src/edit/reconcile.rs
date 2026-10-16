//! Bringing the source back into step with a drawing a gesture changed.

use super::*;

/* -- bringing the source back into step ------------------------------------------- */

/// **The source after a gesture that changed the drawing.**
///
/// The front end draws by mutating the elaborated sketch — that is how a tool gets to solve, snap
/// and show what it is doing while the pointer is still down.  When the gesture ends, the source
/// has to say what the drawing now says, and this is the one verb that makes it so: what the
/// sketch has that the elaboration did not gets a statement appended, what the elaboration had and
/// the sketch no longer does has its statement taken out, and every seed is committed.
///
/// It is a **splice**, like everything here, so a `component`, a `cycle` and every comment in the
/// file survive a gesture that adds a line beside them.  The alternative — re-printing the sketch
/// — would replace a gear written as thirty instances of one tooth with a hundred and twenty point
/// declarations the first time somebody drew a construction line next to it.
///
/// It reads *only* the append: entities and constraints are appended to their vectors, so anything
/// past what the map accounts for is new, and anything the map names that is gone was removed.  A
/// mutation that renumbers (a rebuild — `io::without`, `graft`) is **not** something this can
/// follow, and the front end does not do one: deletion and paste are program edits of their own.
///
/// **It applies itself.**  The drawing did not change here — it had already changed, and the
/// source is only catching up — so rebuilding it would be a lie about what happened, and an
/// expensive one: a new `Sketch` invalidates every proxy the caller is still holding, which is
/// exactly the state a tool is in between two clicks.  So the elaboration takes the new text and
/// extends its own map onto the statements just written, and `Kind` is only a report.
pub fn reconcile(e: &mut Elaborated, sk: &Sketch) -> Edit {
    let prog = e.program.clone();
    Edit::refused_or(&prog, reconciled(e, sk, &prog))
}

type Names = std::collections::BTreeMap<EntRef, String>;

/// `reconcile`'s phases, in order, over `prog`, the program as it was elaborated: an `Err` is a
/// gesture the source cannot follow, said with its cause.
fn reconciled(e: &mut Elaborated, sk: &Sketch, prog: &Program) -> Result<Edit, String> {
    let mut names = Vec::new();
    let mut taken = taken_names(prog);
    // a name for each new entity first, so a line drawn between two new points can refer to them
    let minted = new_entities(e, sk, &mut taken);
    // **What the sketch holds fixed, walked once**: the naming pass below must know what a gauge
    // will have to name, and `gauges` writes those same holds out as statements further down.
    // Two walks were two readings of one question — and the walk is not cheap, `root_declared`
    // ending in a scan of the root body.
    let held = held_refs(sk, &e.map.held_within, &|r| root_declared(e, prog, r));
    let mut needed = needed(e, sk, &minted, &held);
    let memberships = memberships(e, sk, prog, &mut needed)?;
    let (named, renamed) = name_anonymous(e, sk, prog, needed, &minted, &mut taken, &mut names)?;
    let said = Said { sk, e, minted: &minted, renamed: &renamed };

    let mut adds: Vec<StmtKind> = Vec::new();
    // what an appended statement was written *for*, so the map can be extended onto it without
    // elaborating anything
    let mut made: Vec<Made> = Vec::new();
    declarations(&said, &mut adds, &mut made, &mut names);
    relations(&said, &mut adds, &mut made);
    let mut doomed = withdrawn(e, sk, prog);
    // Presentation stays in the drawing/editor, never in model source.
    // memberships, worked out above and written now that every plane has a name: the clause
    // replaced where it stands, written where the parser said one would go, or taken out
    // with the space in front of it
    let flags: Vec<Splice> = memberships.iter().map(|(d, now)| {
        let with = now.map(|p| format!(" in {}", said.name_of(EntRef::frame(p))));
        clause_splice(prog.text(), d.membership.span(), with)
    }).collect();
    gauges(&said, prog, &held, &mut adds, &mut made, &mut doomed);

    if adds.is_empty() && doomed.is_empty() && flags.is_empty() && named.is_empty() {
        // nothing structural: the drawing only moved, and the seeds record where to
        let seeds = commit_seeds(e, sk, prog);
        if seeds.kind != Kind::None {
            e.retext(&seeds.text);
        }
        return Ok(seeds);
    }

    let text = splice(prog.text(), composed(prog, &doomed, &adds, flags, named)?);
    if !e.adopt(&text, &made) {
        return Err("the drawing could not be written down".into());
    }
    // an entity just named in place resolves by that name from here on — and is *called* it,
    // which it was called nothing until now, so a later reconcile in this same elaboration
    // reads a name and not the key it elaborated under
    for (r, n) in &renamed {
        e.map.bind(n, *r, syntax::Named::Written);
    }
    // and now the seeds, against the source that finally has statements for everything
    let after = e.program.clone();
    let seeds = commit_seeds(e, sk, &after);
    let text = if seeds.kind == Kind::None { text } else { seeds.text };
    if seeds.kind != Kind::None {
        e.retext(&text);
    }
    Ok(Edit { text, kind: Kind::Structural, names, refused: None, kept: 0 })
}

/// A plane's origin is the plane's own, minted with it and never declared apart: it is called
/// `P.origin` after its plane.  The plane, for a point that is one's origin.
fn origin_of(sk: &Sketch, r: EntRef) -> Option<usize> {
    (r.kind == EntKind::Point).then(|| sk.plane_of_origin(r.i())).flatten()
}

/// A fresh name for each entity the sketch has past what the elaboration made — the high-water
/// mark, per kind: past it is new — but a plane's origin.
fn new_entities(
    e: &Elaborated,
    sk: &Sketch,
    taken: &mut std::collections::BTreeSet<String>,
) -> Names {
    let mut high: std::collections::BTreeMap<EntKind, usize> = std::collections::BTreeMap::new();
    for r in e.map.of_entity.keys() {
        let n = high.entry(r.kind).or_insert(0);
        *n = (*n).max(r.i() + 1);
    }
    let mut minted = Names::new();
    for r in sk.primitives() {
        if r.i() < high.get(&r.kind).copied().unwrap_or(0) || origin_of(sk, r).is_some() {
            continue;
        }
        minted.insert(r, next_name(taken, r.kind));
    }
    minted
}

/// Everything a statement this reconcile appends will have to *say the name of*.  Small: a
/// gesture states one or two constraints, and every entity that already has a written name falls
/// out at the first test in `name_anonymous`.
fn needed(
    e: &Elaborated,
    sk: &Sketch,
    minted: &Names,
    held: &[(EntRef, Vec<(&'static str, f64)>)],
) -> std::collections::BTreeSet<EntRef> {
    let mut needed = std::collections::BTreeSet::new();
    // a new constraint names its entities…
    for c in sk.user_constraints() {
        if e.map.of_constraint.contains_key(&c.id) {
            continue;
        }
        for a in c.args.iter() {
            if let crate::constraints::Arg::Ent(r) = a {
                needed.insert(*r);
            }
        }
    }
    // …a new entity names its children, which may be points the drawing already had, and the
    // plane its points are on…
    for &r in minted.keys() {
        needed.extend(sk.children(r));
        if let Some(p) = crate::program::plane_of_entity(sk, r) {
            needed.insert(EntRef::frame(p));
        }
    }
    // …a gauge names what it holds…
    needed.extend(held.iter().map(|(r, _)| *r));
    needed
}

/* …and a membership names its plane.  `point a in top` is neither an entity nor a constraint, so
 * nothing above notices it: it is read off the sketch and compared with what the statement says,
 * declaration by declaration, the way a class is — but worked out *here*, since the plane it will
 * name may be one nothing has named yet. */

/// Each root declaration whose plane the drawing changed, and the plane it is on now (`None`: in
/// none), the plane it names added to `needed`.
fn memberships<'p>(
    e: &Elaborated,
    sk: &Sketch,
    prog: &'p Program,
    needed: &mut std::collections::BTreeSet<EntRef>,
) -> Result<Vec<(&'p Decl, Option<usize>)>, String> {
    let mut memberships: Vec<(&Decl, Option<usize>)> = Vec::new();
    // a drawing with no view in it has no membership to write, and the walk below is a scan of
    // the root body per entity — so the question is asked once, of the sketch, first
    for (r, site) in e.map.of_entity.iter().filter(|_| !sk.frames.is_empty()) {
        if !site.path.0.is_empty() || !in_root(prog, site.stmt) {
            continue;
        }
        // the declaration's own entity, not a child it minted: the clause is the statement's
        if e.map.ents_made_by(site.stmt).next() != Some(*r) {
            continue;
        }
        let Some(d) = decl_of(prog, site) else { continue };
        if !d.kind.bears_points() {
            continue;
        }
        let now = crate::program::plane_of_entity(sk, *r);
        let was = d
            .membership
            .plane()
            .as_ref()
            // the whole path: `in std.front` is `std` and a field, and `std` names no plane
            .and_then(|p| e.map.ent_named(&syntax::ref_text(p)))
            .filter(|p| p.kind == EntKind::Frame)
            .map(|p| p.i());
        if now == was {
            continue;
        }
        // a point a `coincident` drew in its plane is in it by that statement, which no clause
        // here repeats (#105)
        let points = || match r.kind {
            EntKind::Point => vec![*r],
            _ => sk.children(*r),
        };
        if was.is_none() && now.is_some()
            && points().iter().any(|p| e.map.drawn.get(&p.i()).copied() == now)
        {
            continue;
        }
        // a membership the `in … { }` block around the statement gave it: there is no clause
        // here to splice, and writing one would say `in` twice
        // the clause is not this statement's to rewrite — a block's, or an enclosing
        // instance's — and which it is is the membership's to say, not this sentence's
        if !d.membership.editable() {
            return Err(format!(
                "that point is {}, so its plane is not this statement's to change; \
                 move the statement in the source instead",
                d.membership.cause()
            ));
        }
        // a declaration with no clause whose every point is declared elsewhere — `l := line(a, b)`
        // — says nothing about planes; its points' own declarations do.  **Before the straddle
        // refusal below**: a line drawn between a point in a view and a point in space is
        // exactly that declaration, and refusing it would stop the source tracking the drawing
        // from then on — `syncSource` only reports a refusal, so the jam is silent.
        let names_all = d.kind != EntKind::Point
            && !d.children.iter().any(|g| g.is_empty())
            && d.children.iter().flatten().all(|k| matches!(k, syntax::Kid::Ref(_)));
        if d.membership.plane().is_none() && names_all {
            continue;
        }
        // its points *are* this statement's to say, and they are on different planes: one
        // clause cannot say two, so the gesture is refused with the cause rather than written
        // wrong.  Only reachable where the statement mints or seeds a point of its own.
        if now.is_none() && sk.children(*r).iter().any(|k| sk.plane_of(k.i()).is_some()) {
            return Err(format!(
                "the points of `{}` are on different planes, which one statement cannot say",
                d.name.shown().map_or_else(|| d.kind.as_str().to_string(), |n| n.text.clone())
            ));
        }
        if let Some(p) = now {
            needed.insert(EntRef::frame(p));
        }
        memberships.push((d, now));
    }
    Ok(memberships)
}

/* An **anonymous declaration** (issue #33) has no name a statement can say: the `#`-keyed name
 * it resolves by is the elaboration's, not the source's.  So whatever a new statement is about to
 * reach for is named *now* — a real name minted and spliced into the declaration, at the empty
 * span the parser recorded where one would go — the same bargain `commit_seeds` strikes with an
 * unwritten `hint(…)` clause.  On demand, and only on demand: an anonymous element nothing
 * references stays unnamed. */

/// The splices naming each declaration a statement this reconcile writes has to name and the
/// source calls nothing, and the name each entity it made is then called; each name given pushed
/// onto `names`.  Refused, with the cause, where nothing can be named.
fn name_anonymous(
    e: &Elaborated,
    sk: &Sketch,
    prog: &Program,
    needed: std::collections::BTreeSet<EntRef>,
    minted: &Names,
    taken: &mut std::collections::BTreeSet<String>,
    names: &mut Vec<String>,
) -> Result<(Vec<Splice>, Names), String> {
    let mut named: Vec<Splice> = Vec::new();
    let mut renamed = Names::new();
    // a plane's origin is said through its plane's name
    let needed: std::collections::BTreeSet<EntRef> = needed
        .into_iter()
        .map(|r| match origin_of(sk, r) {
            Some(p) if e.map.writable_name(r).is_none() => EntRef::frame(p),
            _ => r,
        })
        .collect();
    for r in &needed {
        if minted.contains_key(r) || renamed.contains_key(r) {
            continue; // new (its statement carries the name), or its statement already named
        }
        if let Some(member) = e.map.name_of(*r).and_then(|name| e.map.private_from_root(name)) {
            return Err(format!(
                "`{member}` is private to its component; add the relation inside that component"));
        }
        if e.map.writable_name(*r).is_some() {
            continue; // the source already calls it something a statement may say
        }
        // Called something a statement may *not* say, which is one copy of a block: `#3.0.p`
        // says which copy — so it is shown and selected by — and carries a `#` no tokenizer
        // will give back.  Two questions, and the map answers both from what the flattener told
        // it; the alternative was writing the prefix out for `adopt` to fail on half a function
        // later, with the cause lost.
        if e.map.name_of(*r).is_some() {
            return Err("that is one copy of a block, and only the flattener has a name for it; \
                        write the statement inside the block, where it holds for every copy"
                .into());
        }
        let Some(site) = e.map.of_entity.get(r) else { continue };
        if site.path.0.iter().any(|step| match step {
            crate::ir::PathStep::Instance(id) => matches!(prog.stmt(*id).map(|s| &s.kind),
                Some(StmtKind::Instance(i)) if i.name.text.starts_with('#')),
            _ => false,
        }) {
            return Err("give the component call a name before referring to its geometry".into());
        }
        if !site.path.0.is_empty() || !in_root(prog, site.stmt) {
            // Nothing calls it and there is no statement of the root's to put a name on: it is
            // anonymous inside a component or a block, so refuse now with the cause rather than
            // later with none.
            return Err("that is anonymous inside a component or a block, so no statement \
                        can name it; give its declaration a name there first"
                .into());
        }
        // Made by a statement that is no declaration — a set drawn as its element (#105) — so
        // there is no `name := ` to put on it: refused with the cause, where carrying on wrote
        // the sketch's own label (`c0`), which resolves to nothing (#142).
        let Some(d) = decl_of(prog, site) else {
            let st = prog.stmt(site.stmt);
            let in_place = st.is_some_and(|s| matches!(s.chained, syntax::Chained::Link)
                && matches!(s.kind, StmtKind::Set(_) | StmtKind::Instance(_)));
            // a named call's geometry is writable and never reaches here: an unnamed one's is
            return Err(if in_place {
                "that is a set written in place, so no statement can name it; define the set \
                 by name (`k := { p | … }`) and use it by that name".into()
            } else if matches!(st.map(|s| &s.kind), Some(StmtKind::Instance(_))) {
                "give the component call a name before referring to its geometry".into()
            } else {
                "no declaration made that, so no statement can name it; give it a name in the \
                 source first".into()
            });
        };
        if d.name.shown().is_some() {
            continue; // named since the map was made
        }
        // One name per statement, and *every* entity the statement made follows it at once —
        // the declaration first, then the children it minted, which is the order they were
        // recorded in.  All of them, because a later gesture in this same elaboration must
        // find a name where this one left the source saying a name.
        let mut made_ents = e.map.ents_made_by(site.stmt);
        let Some(parent) = made_ents.next() else { continue };
        let name = next_name(taken, d.kind);
        // `name := ` before the statement's value, or round a chain link's declaration, since
        // `:=` binds looser than `->` (`Decl::mint_close`)
        match d.mint_close {
            None => named.push(Splice { at: d.name.span(), with: format!("{name} := ") }),
            Some(hi) => {
                named.push(Splice { at: d.name.span(), with: format!("({name} := ") });
                named.push(Splice { at: Span::new(hi, hi), with: ")".to_string() });
            }
        }
        // a name this edit gave a declaration, which is what `Edit::names` reports — a caller
        // reads it to refer to what it just made, and a name minted into a declaration that
        // was already there is as much this edit's doing as one on a statement it appended
        names.push(name.clone());
        renamed.insert(parent, name.clone());
        // each child's new name is the dotted path `program::child_names` would have given it
        // under the name just chosen — its slot read off its *position* among the parent's
        // children, which is where the path came from in the first place.  A `List` kind has no
        // dotted paths and mints no anonymous children either (`build` refuses with E103), which
        // is the same pairing `commit_seeds` spells out over its own slot walk.
        let paths = crate::program::child_names(d.kind, &name);
        let kids = sk.children(parent);
        for k in made_ents {
            if let Some(path) = kids.iter().position(|&c| c == k).and_then(|i| paths.get(i)) {
                renamed.insert(k, path.clone());
            }
        }
    }
    Ok((named, renamed))
}

/// What a statement this reconcile writes calls an entity: the name minted for a new one, the one
/// just given an anonymous declaration's, or the elaboration's own.
struct Said<'a> {
    sk: &'a Sketch,
    e: &'a Elaborated,
    minted: &'a Names,
    renamed: &'a Names,
}

impl Said<'_> {
    /// The elaboration's own name for an entity it made — what a new statement has to refer to.
    /// Never a key: the map files one under `by_name` alone, so what it *calls* an entity is a
    /// name a statement may say or nothing at all.
    fn plain(&self, r: EntRef) -> String {
        self.minted
            .get(&r)
            .cloned()
            .or_else(|| self.renamed.get(&r).cloned())
            .or_else(|| self.e.map.name_of(r).cloned())
            .unwrap_or_else(|| syntax::entity_name(r))
    }

    /// `plain`, a plane's origin said through its plane's name.
    fn name_of(&self, r: EntRef) -> String {
        match origin_of(self.sk, r) {
            Some(p) if self.e.map.name_of(r).is_none() => {
                format!("{}.origin", self.plain(EntRef::frame(p)))
            }
            _ => self.plain(r),
        }
    }
}

/// A declaration for each new entity, under the name minted for it.
fn declarations(said: &Said, adds: &mut Vec<StmtKind>, made: &mut Vec<Made>, names: &mut Vec<String>) {
    let sk = said.sk;
    for r in sk.primitives() {
        let Some(name) = said.minted.get(&r) else { continue };
        let mut d = crate::program::lift_decl(sk, r);
        d.name = syntax::DeclName::Written(syntax::Name::new(name.clone()));
        rename_children(&mut d, sk, r, &|k| said.name_of(k));
        // the plane its points are on, by the name the document calls it
        d.membership = crate::program::plane_of_entity(sk, r)
            .map(|p| syntax::Membership::lifted(syntax::Ref::new(said.name_of(EntRef::frame(p)))))
            .unwrap_or_default();
        names.push(name.clone());
        made.push(Made::Ent(r));
        adds.push(StmtKind::Decl(d));
    }
}

/// A relation for each constraint the map does not know: new.
fn relations(said: &Said, adds: &mut Vec<StmtKind>, made: &mut Vec<Made>) {
    let sk = said.sk;
    for c in sk.user_constraints() {
        if said.e.map.of_constraint.contains_key(&c.id) {
            continue;
        }
        let mut rel = crate::program::lift_relation(sk, c);
        // a lift names entities positionally (`P3`); the document calls them what it calls them
        for (a, ca) in rel.form.canonical_args_mut().iter_mut().zip(c.args.iter()) {
            if let (Some(syntax::Arg::Ref(r)), crate::constraints::Arg::Ent(er)) = (a, ca) {
                *r = syntax::Ref::new(said.name_of(*er));
            }
        }
        made.push(Made::Con(c.id));
        adds.push(StmtKind::Relation(rel));
    }
}

/// The root statements whose every constraint is gone: one the map knows that the sketch no
/// longer has was removed — and a statement that made two things is only gone when both are.
fn withdrawn(e: &Elaborated, sk: &Sketch, prog: &Program)
    -> std::collections::BTreeSet<crate::syntax::StmtId> {
    let live: std::collections::BTreeSet<u32> =
        sk.user_constraints().iter().map(|c| c.id).collect();
    let mut doomed = std::collections::BTreeSet::new();
    for (id, site) in e.map.of_constraint.iter() {
        if !live.contains(id) && site.path.0.is_empty() && in_root(prog, site.stmt) {
            doomed.insert(site.stmt);
        }
    }
    for (_, site) in e.map.of_constraint.iter().filter(|(id, _)| live.contains(id)) {
        doomed.remove(&site.stmt);
    }
    for site in e.map.of_entity.values() {
        doomed.remove(&site.stmt);
    }
    doomed
}

/// `fix((0, 0)) p`: a statement per held entity, added and taken away — the holds walked once,
/// in `reconciled`, and named here now that there is a name for each.
fn gauges(
    said: &Said,
    prog: &Program,
    held: &[(EntRef, Vec<(&'static str, f64)>)],
    adds: &mut Vec<StmtKind>,
    made: &mut Vec<Made>,
    doomed: &mut std::collections::BTreeSet<crate::syntax::StmtId>,
) {
    let held_now: std::collections::BTreeMap<GaugeKey, (EntRef, &[(&str, f64)])> =
        held.iter()
            .map(|(r, h)| {
                (gauge_key_of(said.name_of(*r), h.iter().map(|(f, _)| *f)), (*r, h.as_slice()))
            })
            .collect();
    let held_was: std::collections::BTreeSet<GaugeKey> = prog
        .root()
        .body
        .iter()
        .filter_map(|st| match &st.kind {
            StmtKind::Relation(r) => gauge_key(r),
            _ => None,
        })
        .collect();
    for st in prog.root().body.iter() {
        let StmtKind::Relation(r) = &st.kind else { continue };
        let Some(k) = gauge_key(r) else { continue };
        // only a name this elaboration made: a gauge over something a component made stays
        if said.e.map.ent_named(&k.0).is_none() {
            continue;   // a gauge over something a component made is the component's, not ours
        }
        if !held_now.contains_key(&k) {
            doomed.insert(st.id);
        }
    }
    for (k, &(r, h)) in held_now.iter() {
        if held_was.contains(k) {
            continue;
        }
        let point = crate::program::point_len(said.sk, r);
        adds.push(StmtKind::Relation(crate::program::lift_gauge(&k.0, r.kind, point, h)));
        made.push(Made::Gauge);
    }
}

/// Every splice of the reconcile, in the order they must be applied.
fn composed(
    prog: &Program,
    doomed: &std::collections::BTreeSet<crate::syntax::StmtId>,
    adds: &[StmtKind],
    flags: Vec<Splice>,
    named: Vec<Splice>,
) -> Result<Vec<Splice>, String> {
    // composed, so a joint whose every word fell at once — both relations of one run
    // withdrawn together — is unwritten as one splice rather than left as two links with
    // nothing between them; a statement with no splice is dropped, and `adopt` is the net
    let mut edits: Vec<Splice> =
        doomed_splices(prog.text(), &prog.root().body, doomed).into_iter().flatten().collect();
    if !adds.is_empty() {
        edits.push(appending(prog, adds)?);
    }
    // after the append, deliberately: insertions at one offset land in *reverse* application
    // order, and `splice`'s stable sort applies equal offsets in this vec's order — so where
    // the file's last statement is the one being named or flagged, this order is the layout:
    // the appended statements go past the line's end, a class clause stands before them, and a
    // minted name lands against its keyword with everything else after it
    edits.extend(flags);
    edits.extend(named);
    Ok(edits)
}

/// A lifted declaration refers to its children by their *positional* names (`P3`, `C0`); the
/// document calls them whatever it calls them.  One walk swaps them over.
fn rename_children(
    d: &mut Decl,
    sk: &Sketch,
    r: EntRef,
    name_of: &dyn Fn(EntRef) -> String,
) {
    let kids = sk.children(r);
    let mut i = 0usize;
    for slot in d.children.iter_mut() {
        for c in slot.iter_mut() {
            if let Some(&k) = kids.get(i) {
                *c = syntax::Kid::Ref(syntax::Ref::new(name_of(k)));
            }
            i += 1;
        }
    }
}

/// What a `fix` holds: the entity as written, and the names of the numbers it holds, sorted —
/// so a statement and the sketch's holds are compared by what is held and not by spelling.
type GaugeKey = (String, Vec<String>);

fn gauge_key_of<'a>(name: String, fields: impl Iterator<Item = &'a str>) -> GaugeKey {
    let mut fields: Vec<String> = fields.map(str::to_string).collect();
    fields.sort();
    (name, fields)
}

/// What a `fix` statement holds, and `None` for a relation that is no `fix`.  Read off the
/// statement as it was written, or as it was built (`program::lift_gauge`).
fn gauge_key(r: &syntax::Relation) -> Option<GaugeKey> {
    use crate::constraints::CKind;
    match &r.form {
        syntax::RelationForm::Written(w) => {
            if crate::constraints::gauge_op(&w.word.text)? != CKind::Fix {
                return None;
            }
            let fields: Vec<syntax::Name> = w.slots().map(|(key, _)| key).collect();
            Some(gauge_key_of(syntax::ref_text(w.ops.first()?), fields.iter().map(|k| k.text.as_str())))
        }
        syntax::RelationForm::Canonical { kind: CKind::Fix, args, .. } => {
            let Some(Some(syntax::Arg::Ref(rf))) = args.first() else { return None };
            let spec = CKind::Fix.spec();
            let fields = args.iter().enumerate().skip(1)
                .filter(|(_, a)| a.is_some())
                .map(|(i, _)| spec[i].0);
            Some(gauge_key_of(syntax::ref_text(rf), fields))
        }
        syntax::RelationForm::Canonical { .. } => None,
    }
}

/// Everything the sketch itself holds fixed — each entity, and the numbers of its own held, by
/// field and at what — before any name is put to it.  **The one walk**, made once per
/// reconcile: the anonymous-naming pass reads it to know what a gauge will have to name, and the
/// gauge statements are written from the same list, so the two cannot disagree about what is
/// held.  Names are put to it only at the second, by which point every hold has one.  A number
/// a component's `fix` holds is left out, though the root declared what it is of: the component
/// says it (`std.Turn` holding a frame's joint), and a gauge at the root would say it twice.
fn held_refs(
    sk: &Sketch,
    within: &std::collections::BTreeSet<u32>,
    ours: &dyn Fn(EntRef) -> bool,
) -> Vec<(EntRef, Vec<(&'static str, f64)>)> {
    sk.primitives()
        .into_iter()
        .filter(|&r| ours(r))
        .map(|r| (r, crate::program::holds_except(sk, r, within)))
        .filter(|(_, h)| !h.is_empty())
        .collect()
}

/// Whether an entity's declaration is a statement of the root — as against one a component made,
/// which is where its `fix` is written too, and neither is ours to add or take away.
fn root_declared(e: &Elaborated, prog: &Program, r: EntRef) -> bool {
    match e.map.of_entity.get(&r) {
        Some(site) => site.path.0.is_empty() && in_root(prog, site.stmt),
        // not in the map at all: a gesture just made it, so it is as root as anything gets
        None => true,
    }
}
