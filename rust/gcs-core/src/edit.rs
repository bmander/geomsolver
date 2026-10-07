//! Editing a program: what a gesture on the drawing does to the source.
//!
//! This is the module that makes the source the *document* rather than a view of one.  Every edit
//! here is a **splice** — a few characters replaced in the text somebody wrote — and never a
//! reprint.  That distinction is the whole design:
//!
//! * a reprint would flatten a hand-written `component` into the entities it elaborates to, throw
//!   away every comment, and reflow every line, on the first drag;
//! * a splice rewrites the six characters of one seed and leaves the rest of the file alone.
//!
//! So a gear written as a `Tooth` in a `cycle` stays written that way while its points are
//! dragged, and the panel shows what the author wrote rather than what the solver made of it.
//!
//! Splices run **back to front**, so a span computed before the edit is still valid when its turn
//! comes.  Nothing here re-parses: the caller applies the returned text and elaborates once.

use crate::model::{EntKind, EntRef, Sketch};
use crate::program::{Elaborated, Made, Site};
use crate::syntax::{self, num, Decl, Program, Span, Stmt, StmtKind};

/// What an edit did to the document, and what it costs to take up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// Statements were added or removed: the sketch has to be built again.
    Structural,
    /// Only numbers a solve is allowed to move changed.  The topology is untouched, so a caller
    /// holding a compiled plan may keep it.
    Numeric,
    /// Nothing to do, or nothing this edit is able to do.
    None,
}

#[derive(Clone, Debug)]
pub struct Edit {
    pub text: String,
    pub kind: Kind,
    /// The names a declaration was given, in the order they were made.
    pub names: Vec<String>,
    /// Why, when an edit could not be made.
    pub refused: Option<String>,
}

impl Edit {
    fn none(prog: &Program, why: Option<String>) -> Edit {
        Edit {
            text: prog.text().to_string(),
            kind: Kind::None,
            names: Vec::new(),
            refused: why,
        }
    }

    /// The source with `edits` applied, an edit of the class `kind`.
    fn spliced(prog: &Program, edits: Vec<Splice>, kind: Kind) -> Edit {
        Edit { text: splice(prog.text(), edits), kind, names: Vec::new(), refused: None }
    }
}

/// Whether a seed is a number a solve may write over: a literal, or one in notation (`3 1/8`,
/// `30deg`) — not an expression, which is the author's arithmetic.
fn writable_seed(text: &str) -> bool {
    crate::expr::literal(text).is_some() || crate::expr::notation(text)
}

/// A solved seed as it is written back: in degrees where an angle's text named its unit.
fn seed_literal(text: &str, v: f64, angle: bool) -> String {
    if angle && crate::expr::names_unit(text) { format!("{}deg", num(v)) } else { num(v) }
}

/// One replacement in the source.
#[derive(Clone)]
struct Splice {
    at: Span,
    with: String,
}

/// Apply a set of replacements, back to front so earlier spans stay valid.
fn splice(text: &str, mut edits: Vec<Splice>) -> String {
    edits.sort_by_key(|e| std::cmp::Reverse(e.at.lo));
    let mut out = text.to_string();
    for e in edits {
        let (lo, hi) = (e.at.lo as usize, e.at.hi as usize);
        if lo > hi || hi > out.len() || !out.is_char_boundary(lo) || !out.is_char_boundary(hi) {
            continue; // a span that does not name a place in this text edits nothing
        }
        out.replace_range(lo..hi, &e.with);
    }
    out
}

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
    // The walk is over the root component's own statements, which is the question `in_root` was
    // asking one statement at a time: a statement inside a component is written in the
    // component's terms, and a pose put there is a pose put on every instance of it.  What each
    // one made is `SourceMap::ents_made_by`, whose order — the declaration's own entity first,
    // then the children it minted — is what `reconcile` reads too, so neither the entity index
    // nor the find-by-kind that re-derived the parent is needed.
    // A driving radius dimension already records the scalar. Keep hints for free radii
    // and claims, which do not determine it, and preserve hints the author wrote explicitly.
    let dimensioned_radii: std::collections::BTreeSet<EntRef> = sk.constraints.iter()
        .filter(|c| c.kind == crate::constraints::CKind::Radius
            && !c.claim && !c.soft && c.free.is_none())
        .map(|c| c.args[0].ent()).collect();
    let mut edits = Vec::new();
    // and a `ring`'s body at the root: one statement however many copies, but the copies are
    // turns of the first, so the first's pose is the one there is to record (§12.4)
    let ring_bodies = prog.root().body.iter().filter_map(|st| match &st.kind {
        StmtKind::Block(b) if b.kind == syntax::BlockKind::Ring => Some(b.body.iter()),
        _ => None,
    });
    for st in prog.root().body.iter().chain(ring_bodies.flatten()) {
        let StmtKind::Decl(d) = &st.kind else { continue };
        // `hint(at: t)` names a *place*, and has no coordinates to write.  Faces and
        // solids also own no seeds: their children are boundaries and operands, not points.
        if d.seed_at.is_some() || d.kind.spatial() {
            continue;
        }
        // a declaration that could not be built made nothing, and has no pose to record
        let Some(parent) = e.map.ents_made_by(st.id).next() else { continue };
        let omit_radius = dimensioned_radii.contains(&parent)
            && d.seed_spans.first().is_none_or(|span| span.is_empty());
        let kids = sk.children(parent);
        // the statement's slot for each child, in field order — the same order `sk.children`
        // hands them back in.  A slot may be empty (an implicit child), so for the per-slot
        // kinds the groups are indexed directly; a `List` kind's one group is flattened.
        let slot_kid = |j: usize| -> Option<&syntax::Kid> {
            if d.children.len() == kids.len() {
                d.children.get(j).and_then(|g| g.first())
            } else {
                d.children.iter().flatten().nth(j)
            }
        };
        // whether a kid's text is this statement's own: a chain's thread fills slots with
        // references written in *another* link (or written nowhere, with an empty span), and a
        // writeback must not mistake those for a list the source wrote here
        let written_here = |kid: &syntax::Kid| match kid {
            syntax::Kid::Ref(r) => {
                !r.span.is_empty() && r.span.lo >= st.span.lo && r.span.hi <= st.span.hi
            }
            syntax::Kid::Hint(_) | syntax::Kid::Face { .. } | syntax::Kid::Trim { .. } => true,
        };

        // One seed at a time: where the source wrote it, the splice that records it; where it
        // did not, the news that something has moved with nowhere to write it.
        let one = |v: f64, text: Option<&String>, span: Span| -> (Option<Splice>, bool) {
            if text.is_some() {
                return (None, false); // an expression: a solve does not rewrite arithmetic
            }
            let now = num(v);
            if span.is_empty() {
                // an omitted scalar reads as 0, so it needs recording only when it is not 0
                (None, v != 0.0)
            } else if span.slice(prog.text()) != now {
                (Some(Splice { at: span, with: now }), false)
            } else {
                (None, false)
            }
        };
        let mut mine: Vec<Splice> = Vec::new();
        let mut missing = false;
        // a number a `fix` holds is stated there, and is its own seed: no seed of it is written,
        // or counted missing — and one the clause wrote is taken out of it (`fix(x == 3) p`
        // makes `hint((3, 7))` say `hint(y: 7)`)
        let held = |p: u32| sk.params[p as usize].fixed;
        let written = |i: usize| d.seed_spans.get(i).is_some_and(|s| !s.is_empty());
        let mut redundant = false;
        for (i, p) in sk.own_params(parent).iter().enumerate() {
            if omit_radius { continue; }
            if held(*p) {
                redundant |= written(i);
                continue;
            }
            let v = sk.seed_value(parent, *p);
            let text = d.seed_text.get(i).and_then(|t| t.as_ref());
            let (sp, miss) = one(v, text, d.seed_spans.get(i).copied().unwrap_or_default());
            mine.extend(sp);
            missing |= miss;
        }
        // An anonymous child's seed lives in the parent's statement, in a slot of its own — and
        // the slots stand in the order `sk.children` hands the children back in, so the two walk
        // together.  A slot the source wrote a *name* in is a point declared elsewhere, and is
        // written back where it was declared; one it wrote nothing for at all was minted.  A
        // child's seed is a point's place, or an axis's direction (a plane's `u:` and `v:`).
        let seeded = |k: EntRef| -> Vec<u32> {
            match k.kind {
                EntKind::Point => sk.point_params(k.i()).to_vec(),
                EntKind::Axis => sk.axes[k.i()].d.to_vec(),
                _ => Vec::new(),
            }
        };
        for (j, &k) in kids.iter().enumerate() {
            if seeded(k).iter().all(|&p| held(p)) {
                continue;
            }
            let seed = match slot_kid(j) {
                Some(syntax::Kid::Ref(_) | syntax::Kid::Face { .. } | syntax::Kid::Trim { .. }) => continue,
                Some(syntax::Kid::Hint(s)) => Some(s),
                None => None,
            };
            let v: Vec<f64> = seeded(k).iter().map(|&p| sk.params[p as usize].value).collect();
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
                    if s.span.slice(prog.text()) != now {
                        mine.push(Splice { at: s.span, with: now });
                    }
                }
                Some(s) => {
                    for i in 0..v.len() {
                        let (sp, miss) = one(v[i], s.text[i].as_ref(), s.spans[i]);
                        mine.extend(sp);
                        missing |= miss;
                    }
                }
                None => missing |= v.iter().any(|&c| c != 0.0),
            }
        }

        if !missing {
            // the clause rewritten without what a `fix` holds: the numbers it keeps as they stand
            // (spliced above where the solve moved one), and the whole clause gone with the
            // space before it where it keeps none
            if let Some(at) = d.hint_span.filter(|at| redundant && !at.is_empty()) {
                let own = sk.own_params(parent);
                let free = own.iter().enumerate()
                    .filter(|&(i, &p)| written(i) && !held(p))
                    .map(|(i, &p)| (i, sk.seed_value(parent, p)));
                let hint = free_clause(parent.kind, d, prog.text(), free);
                mine.retain(|s| s.at.lo >= at.hi || s.at.hi <= at.lo);
                mine.push(clause_splice(prog.text(), at, (!hint.is_empty()).then_some(hint)));
            }
            edits.extend(mine);
            continue;
        }
        // Something the source never wrote has moved — an omitted radius, an endpoint of a bare
        // `l := line`.  There is nowhere to splice, so what the source left out is written: the
        // argument list, the `hint(…)` clause, or both.
        let Some(at) = d.hint_span else { continue };
        let mut pose = d.seed.clone();
        for (i, p) in sk.own_params(parent).iter().enumerate() {
            if let Some(v) = pose.get_mut(i) {
                *v = sk.seed_value(parent, *p);
            }
        }
        // the clause, as the pose the solve arrived at; empty when the kind owns no scalar at
        // all — a line's numbers are its two points', and they are written in the slots
        let own = sk.own_params(parent);
        let hint = if omit_radius {
            String::new()
        } else if own.iter().any(|&p| held(p)) {
            // only the numbers no `fix` holds: `hint(y: 7)` beside `fix(x == 3) p`, and none at
            // all where every one is held
            let free = own.iter().zip(&pose).enumerate()
                .filter(|&(_, (&p, _))| !held(p))
                .map(|(i, (_, &v))| (i, v));
            free_clause(parent.kind, d, prog.text(), free)
        } else {
            syntax::hint_clause(d, &pose)
        };
        // No slot of this list is the source's own text, so the list has to be written too —
        // a chain's thread fills slots with references written in *another* link, or written
        // nowhere at all, and neither is a list this statement can splice into.  It is spelled
        // by the printer that spells every other statement — write it here and an `arc`'s
        // `center:`/`start:`/`end:` labels are dropped by the one path that wrote its own —
        // and a `Decl` of the solved pose is what that printer takes.
        let mine_here = d.children.iter().flatten().any(|kid| written_here(kid));
        let list = (!mine_here && !kids.is_empty()).then(|| {
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
            let mut filled = kids.iter().enumerate().map(|(j, k)| match slot_kid(j) {
                Some(syntax::Kid::Ref(r)) if syntax::hidden(&r.root.text) => None,
                // a child a `fix` holds is placed there, and its slot is left empty
                None if seeded(*k).iter().all(|&p| held(p)) => None,
                Some(syntax::Kid::Ref(r)) => Some(syntax::Kid::Ref(r.clone())),
                _ => {
                    let mut v = [0.0; 3];
                    for (x, p) in v.iter_mut().zip(seeded(*k)) {
                        *x = sk.params[p as usize].value;
                    }
                    let axis = k.kind == EntKind::Axis;
                    Some(syntax::Kid::Hint(syntax::KidSeed { v, axis, ..Default::default() }))
                }
            });
            for g in d2.children.iter_mut() {
                *g = filled.by_ref().take(g.len().max(1)).flatten().collect();
            }
            syntax::decl_args(&d2)
        });
        // a clause the source wrote whose every number a `fix` now holds goes, with the space
        // before it
        let gone = (redundant && hint.is_empty() && !at.is_empty())
            .then(|| clause_splice(prog.text(), at, None));
        if list.is_none() && gone.is_none() && hint.is_empty() {
            // nothing to write here: what moved is in a slot the source wrote, and splices there
            edits.extend(mine);
            continue;
        }
        // a slot the source *did* write still splices in place; only what it did not is here
        edits.extend(mine.into_iter().filter(|s| s.at.lo >= at.hi || s.at.hi <= at.lo));
        if let Some(gone) = gone {
            edits.push(gone);
            if let Some(args) = list {
                edits.push(Splice { at: d.list_span, with: args });
            }
            continue;
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
    }
    edits.extend(unknown_seeds(sk, prog));
    if edits.is_empty() {
        return Edit::none(prog, None);
    }
    Edit::spliced(prog, edits, Kind::Numeric)
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
fn unknown_seeds(sk: &Sketch, prog: &Program) -> Vec<Splice> {
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
                    let Some(v) = solved(&format!("{}.{}", inst.name.text, l.text)) else { continue };
                    out.extend(seed_splice(prog, text, *span, v, f.ty == syntax::Ty::Angle));
                }
            }
            _ => {}
        }
    }
    out
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

/// Whether a statement is one of the root component's own.
///
/// Not the same question as "is it reached once".  A component instantiated a single time makes
/// one entity, so its pose is unambiguous and a seed *can* be written back — but the statement is
/// the *component's*, and deleting it would edit a reusable thing on behalf of a gesture that
/// named one entity.  So writeback asks "reached once" and deletion asks this.
fn in_root(prog: &Program, id: crate::syntax::StmtId) -> bool {
    prog.root().body.iter().any(|s| s.id == id)
}

fn decl_of<'a>(prog: &'a Program, site: &Site) -> Option<&'a Decl> {
    match &prog.stmt(site.stmt)?.kind {
        StmtKind::Decl(d) => Some(d),
        _ => None,
    }
}

/* -- adding ------------------------------------------------------------------------ */

/// Where a new statement goes: the end of the root component's body.
///
/// The *end of its last statement*, not the end of the file — a program may have a trailing
/// comment, and a drawing tool should not write past it.
fn append_at(prog: &Program) -> (Span, String) {
    let root = prog.root();
    // past the statement, or past the `in` block it was hoisted out of: an appended statement
    // writes its own membership, which inside the block would be said twice
    let after = |st: &syntax::Stmt| {
        let hi = prog.in_blocks.iter()
            .find(|b| b.header.lo <= st.span.lo && st.span.hi <= b.close.hi)
            .map_or(st.span.hi, |b| b.close.hi) as usize;
        (Span::new(hi, hi), "\n".to_string())
    };
    if let Some(preview) = prog.preview {
        if let Some(st) = root.body.iter().rev().find(|st| preview.contains(st.span.lo)) {
            return after(st);
        }
        let close = preview.hi as usize - 1;
        return (Span::new(close, close), "\n".into());
    }
    match root.body.last() {
        Some(st) => after(st),
        None => {
            let n = prog.text().len();
            let lead = if prog.text().ends_with('\n') || n == 0 { "" } else { "\n" };
            (Span::new(n, n), lead.to_string())
        }
    }
}

/// A name nothing has taken, in the kind's own alphabet: `p0`, `l1`, `c0`.
pub fn mint(prog: &Program, kind: EntKind) -> String {
    next_name(&mut taken_names(prog), kind)
}

/// Every name a declaration in the program already binds — component bodies included, and the
/// `#a…` keys anonymous declarations carry among them.  What "already spoken for" means, said
/// once, beside the loop that consumes it.
fn taken_names(prog: &Program) -> std::collections::BTreeSet<String> {
    prog.stmts()
        .filter_map(|s| match &s.kind {
            StmtKind::Decl(d) => Some(d.name.key().text.clone()),
            StmtKind::Chain(c) => Some(c.name.key().text.clone()),
            _ => None,
        })
        .collect()
}

/// The next name `taken` does not hold, in the kind's own alphabet — and now taken.  The one
/// spelling of the fresh-name rule: `mint` asks it for a caller, and `reconcile` asks it for a
/// new entity and again for an anonymous declaration a statement is about to reference.
fn next_name(taken: &mut std::collections::BTreeSet<String>, kind: EntKind) -> String {
    let c = syntax::kind_initial(kind);
    let name = (0..)
        .map(|i| format!("{c}{i}"))
        .find(|n| !taken.contains(n))
        .expect("an unbounded sequence always holds a fresh name");
    taken.insert(name.clone());
    name
}

/// Append one statement, whatever it is.
fn append(prog: &Program, kind: StmtKind, names: Vec<String>) -> Edit {
    let (at, lead) = append_at(prog);
    let mut line = lead;
    if let Err(e) = syntax::write_stmt_to(&mut line, &kind) {
        return Edit::none(prog, Some(e.to_string()));
    }
    Edit {
        text: splice(prog.text(), vec![Splice { at, with: line }]),
        kind: Kind::Structural,
        names,
        refused: None,
    }
}

/// `use NAME`, so the document reaches a module it did not — the workspace adds `use std` when
/// something is first drawn on one of the standard planes.  A line of its own, after the last
/// `use` the document writes, or else before its first statement (below a heading comment and
/// `unit`); nothing when the document already says it.
pub fn add_use(prog: &Program, name: &str) -> Edit {
    if prog.uses.iter().any(|u| u.name == name) {
        return Edit::none(prog, None);
    }
    let text = prog.text();
    let line = format!("use {name}\n");
    let at = match prog.uses.iter().map(|u| u.span.hi as usize).filter(|&hi| hi <= text.len()).max() {
        // the end of the last `use`'s line
        Some(hi) => match text[hi..].find('\n') {
            Some(k) => hi + k + 1,
            None => text.len(),
        },
        // the start of the line the first statement or component definition begins on — not
        // `unit`, which a `use` reads after, and not the document's own anonymous root, which is
        // a component spanning the whole text
        None => {
            let root = prog.root();
            let first = root.body.iter()
                .filter(|st| !matches!(st.kind, StmtKind::Unit(_)))
                .map(|st| st.span.lo as usize)
                .chain(prog.components.iter().filter(|c| !std::ptr::eq(*c, root)).map(|c| c.span.lo as usize))
                .chain(prog.words.iter().filter(|d| d.module.is_none()).map(|d| d.span.lo as usize))
                .filter(|&lo| lo <= text.len())
                .min();
            match first {
                Some(lo) => text[..lo].rfind('\n').map_or(0, |k| k + 1),
                None => text.len(),
            }
        }
    };
    let lead = if at == text.len() && !text.is_empty() && !text.ends_with('\n') { "\n" } else { "" };
    Edit {
        text: splice(text, vec![Splice { at: Span::new(at, at), with: format!("{lead}{line}") }]),
        kind: Kind::Structural,
        names: Vec::new(),
        refused: None,
    }
}

/// `pN := point hint((…, …))`
/// The rectangle the Rect tool draws: a **reusable component**, defined once per document, and
/// one instance per gesture.  The definition is the chain a person would write — four lines
/// welded corner to corner at right angles, the first two carrying the width and the height —
/// so what a gesture leaves in the source is one statement, `r0 := Rectangle(w: 120, h: 60)`,
/// and the drawing owns a `Rectangle` any later statement may instance again.  Where the
/// figure *sits* is not in the statement: an instance's geometry is written in the component's
/// terms, so its pose is the session's (the tool seeds it at the gesture) and a reload starts
/// it from `scatter`, the same bargain every component's interior strikes.
pub fn add_rectangle(prog: &Program, w: f64, h: f64, plane: Option<&str>) -> Edit {
    let (at, lead) = append_at(prog);
    let mut with = lead;
    let mut edits = Vec::new();
    // the document's own: a module's `Rectangle` is no name a bare call reaches (§14.4)
    if prog.component("Rectangle").is_none() {
        let definition = "component Rectangle(w: Length, h: Length) {\n  \
             distance(w) (l1 := line) -> perpendicular distance(h) (l2 := line) -> \
             perpendicular (l3 := line) -> perpendicular (l4 := line) -> close\n}\n\n";
        if let Some(preview) = prog.preview {
            let start = preview.lo as usize;
            edits.push(Splice { at: Span::new(start, start), with: definition.into() });
        } else {
            if at.lo > 0 { with.push('\n'); }
            with.push_str(definition);
        }
    }
    // a fresh instance name, past every name the document already binds
    let taken: std::collections::BTreeSet<&str> = prog
        .stmts()
        .filter_map(|s| s.kind.bound_name().map(|n| n.text.as_str()))
        .collect();
    let name =
        (0..).map(|i| format!("r{i}")).find(|n| !taken.contains(n.as_str())).unwrap_or_default();
    let arg = |label: &str, v: f64| syntax::InstArg {
        label: Some(syntax::Name::new(label)),
        value: syntax::InstVal::Expr(num(v)),
        span: Span::default(),
    };
    let inst = syntax::Instance {
        annotations: Default::default(),
        name: syntax::Name::new(name.clone()),
        component: syntax::Name::new("Rectangle"),
        args: vec![arg("w", w), arg("h", h)],
        span: Span::default(),
        // drawn in a view when the caller says so: the instance joins it whole (§6.7)
        membership: plane
            .map(|p| syntax::Membership::lifted(syntax::Ref::new(p)))
            .unwrap_or_default(),
        class: Default::default(),
    };
    let mut line = String::new();
    if let Err(e) = syntax::write_stmt_to(&mut line, &StmtKind::Instance(inst)) {
        return Edit::none(prog, Some(e.to_string()));
    }
    with.push_str(&line);
    edits.push(Splice { at, with });
    Edit {
        text: splice(prog.text(), edits),
        kind: Kind::Structural,
        names: vec![name],
        refused: None,
    }
}

pub fn add_point(prog: &Program, x: f64, y: f64) -> Edit {
    let name = mint(prog, EntKind::Point);
    let d = Decl::point(syntax::DeclName::Written(syntax::Name::new(name.clone())), [x, y], None);
    append(prog, StmtKind::Decl(d), vec![name])
}

/// An entity built from names that already exist — a line from two points, a circle from a centre.
pub fn add_entity(prog: &Program, kind: EntKind, args: &[String], seed: &[f64]) -> Edit {
    add_entity_with(prog, kind, args, seed, None)
}

/// A plane over two axes or drawn lines, `args` their names — `plane(u: a, v: b)` — and, when
/// the caller has one, the name it asked for.  A name already in use is refused rather than
/// silently renamed: the caller is about to refer to it.
pub fn add_plane(prog: &Program, args: &[String], name: Option<&str>) -> Edit {
    add_entity_with(prog, EntKind::Plane, args, &[], name)
}

fn add_entity_with(
    prog: &Program,
    kind: EntKind,
    args: &[String],
    seed: &[f64],
    name: Option<&str>,
) -> Edit {
    if kind == EntKind::Point || kind == EntKind::Curve {
        return Edit::none(prog, Some(format!("{} is not built this way", kind.a())));
    }
    let name = match name {
        Some(n) if taken_names(prog).contains(n) => {
            return Edit::none(prog, Some(format!("`{n}` is already a name in this document")));
        }
        Some(n) if !crate::syntax::is_name(n) => {
            return Edit::none(prog, Some(format!("`{n}` is not a name a statement can say")));
        }
        Some(n) => n.to_string(),
        None => mint(prog, kind),
    };
    let mut children: Vec<Vec<syntax::Kid>> = Vec::new();
    let mut taken = 0usize;
    for (_, f) in kind.fields() {
        match f {
            crate::model::Field::Child => {
                children.push(
                    args.get(taken)
                        .map(|a| vec![syntax::Kid::Ref(syntax::Ref::new(a.clone()))])
                        .unwrap_or_default(),
                );
                taken += 1;
            }
            crate::model::Field::List => {
                children.push(
                    args[taken.min(args.len())..]
                        .iter()
                        .map(|a| syntax::Kid::Ref(syntax::Ref::new(a.clone())))
                        .collect(),
                );
                taken = args.len();
            }
            crate::model::Field::Scalar => {}
        }
    }
    let n_scalar = kind.fields().iter().filter(|(_, f)| *f == crate::model::Field::Scalar).count();
    let d = Decl {
        annotations: Default::default(),
        kind,
        name: syntax::DeclName::Written(syntax::Name::new(name.clone())),
        children,
        seed: (0..n_scalar).map(|i| seed.get(i).copied().unwrap_or(0.0)).collect(),
        seed_text: vec![None; n_scalar],
        seed_spans: Vec::new(),
        hint_span: None,
        knots: None,
        weights: None,
        curve: None,
        computed: None,
        class: Default::default(),
        class_span: Span::default(),
        seed_at: None,
        seed_names: Vec::new(),
        // a gesture never draws a solid: the sheet is where the drawing is, and a solid is
        // written over what is drawn there
        sweep: None, motion: None, angular_span: None,
        membership: Default::default(),
        list_span: Span::default(),
        close: None,
        mint_close: None,
    };
    append(prog, StmtKind::Decl(d), vec![name])
}

/// One constraint, written the way the registry names it.
pub fn add_relation(prog: &Program, r: syntax::Relation) -> Edit {
    append(prog, StmtKind::Relation(r), Vec::new())
}

/* -- removing ---------------------------------------------------------------------- */

/// Take entities and constraints out of the source.
///
/// A statement goes when it declares something being removed, or when it *mentions* one — which
/// is the same rule `io::without` follows on a sketch, said about text instead: a constraint
/// comes along exactly when all its entities did.
///
/// Refused when what is being removed was made inside a component or a block.  There is no one
/// statement to delete there — the statement makes N of them — and quietly deleting the whole
/// component would be a much larger edit than the gesture asked for.
///
/// The **sketch is passed**, as `reconcile` and `commit_seeds` take theirs and for the same
/// reason: a front end moves the sketch out of the elaboration into a handle of its own
/// (`Elaborated::taken`), so `e.sketch` is empty there.  The one question here only the model
/// can answer is which constraints name an entity their text never spells — a projection's
/// planes — and asked of an empty sketch it silently answers none.
pub fn remove(
    e: &Elaborated,
    prog: &Program,
    sk: &Sketch,
    ents: &[EntRef],
    cons: &[u32],
) -> Edit {
    let mut doomed: std::collections::BTreeSet<crate::syntax::StmtId> =
        std::collections::BTreeSet::new();
    let mut names: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
    for r in ents {
        let Some(site) = e.map.of_entity.get(r) else { continue };
        if !site.path.0.is_empty() || !in_root(prog, site.stmt) {
            return Edit::none(
                prog,
                Some(
                    "that comes from a component, so deleting it would edit the component and                      everything else drawn from it"
                        .into(),
                ),
            );
        }
        doomed.insert(site.stmt);
        if let Some(d) = decl_of(prog, site) {
            names.insert(d.name.key().text.clone());
        }
    }
    for id in cons {
        if let Some(site) = e.map.of_constraint.get(id) {
            if site.path.0.is_empty() && in_root(prog, site.stmt) {
                doomed.insert(site.stmt);
            }
        }
    }
    // and every statement that names one of the gone — to a fixed point, because a line that
    // named a deleted point is itself deleted, and the constraint on *that* line goes with it
    loop {
        let mut grew = false;
        for st in prog.root().body.iter() {
            if doomed.contains(&st.id) || mentions(st, &names).is_empty() {
                continue;
            }
            doomed.insert(st.id);
            if let StmtKind::Decl(d) = &st.kind {
                names.insert(d.name.key().text.clone());   // and now whatever names *it* goes too
            }
            grew = true;
        }
        if !grew {
            break;
        }
    }
    // and every statement whose *constraint* names one of the gone without its text doing so:
    // a projection's planes are its points' memberships, inferred and never spelled, so the
    // model drops it when a plane goes (`io::without`'s rule) and its statement goes with it
    // only a kind with an inferred entity slot can be in that position, so a document with none
    // skips the walk — `user_constraints` and `entities` both allocate
    let gone: std::collections::BTreeSet<EntRef> = sk
        .constraints
        .iter()
        .any(|c| c.kind.spec().iter().enumerate().any(|(i, (_, k))| k.is_entity() && c.kind.infers_arg(i)))
        .then(|| names.iter().filter_map(|n| e.map.ent_named(n)).collect())
        .unwrap_or_default();
    for c in sk.user_constraints().into_iter().filter(|_| !gone.is_empty()) {
        if !c.entities().iter().any(|x| gone.contains(x)) {
            continue;
        }
        if let Some(site) = e.map.of_constraint.get(&c.id) {
            if site.path.0.is_empty() && in_root(prog, site.stmt) {
                doomed.insert(site.stmt);
            }
        }
    }
    if doomed.is_empty() {
        return Edit::none(prog, None);
    }
    // a membership names a plane without depending on it: a point whose plane goes stays,
    // and only its `in …` clause comes out — with the space that set it off
    let mut clauses: Vec<Splice> = Vec::new();
    for st in prog.root().body.iter() {
        if doomed.contains(&st.id) {
            continue;
        }
        let m = match &st.kind {
            StmtKind::Decl(d) => &d.membership,
            StmtKind::Instance(i) => &i.membership,
            _ => continue,
        };
        // only a clause this statement wrote has a span here to take out; a block's comes out
        // with the block's own header below
        let (Some(p), at) = (m.written(), m.span()) else { continue };
        if names.contains(&p.root.text) && !at.is_empty() {
            clauses.push(clause_splice(prog.text(), at, None));
        }
    }
    // and an `in PLANE { … }` block whose plane goes: the header and its brace come out, and
    // the statements stay — points in space now, exactly as a clause's point stays.  The block's
    // own decls have no clause span, so the pass above never reaches into the header.
    for b in &prog.in_blocks {
        if !names.contains(&b.plane.root.text) {
            continue;
        }
        for at in [b.header, b.close] {
            clauses.push(clause_splice(prog.text(), at, None));
        }
    }
    // a link of a chain has no deletion splice, so the gesture is refused rather than half-done
    let refuse = || {
        Edit::none(
            prog,
            Some(
                "that is part of a chain, which deletion cannot unpick; edit the source instead"
                    .into(),
            ),
        )
    };
    let mut edits: Vec<Splice> = clauses;
    for s in doomed_splices(prog.text(), &prog.root().body, &doomed) {
        match s {
            Some(e) => edits.push(e),
            None => return refuse(),
        }
    }
    // a prefix word stands inside the statement of the declaration it qualifies once a name is
    // written before both (`l := horizontal line(a, b)`): the declaration's splice takes it
    let inside = |e: &Splice, f: &Splice| {
        f.at.lo <= e.at.lo && e.at.hi <= f.at.hi && (f.at.lo, f.at.hi) != (e.at.lo, e.at.hi)
    };
    let edits: Vec<Splice> =
        edits.iter().filter(|e| !edits.iter().any(|f| inside(e, f))).cloned().collect();
    let text = splice(prog.text(), edits);
    // and where a chain was touched, the result must still parse: a chain can weave what no
    // set of per-statement splices unpicks — a name link left dangling between two doomed
    // joints — and a deletion that corrupts the source is worse than one that is refused.
    // Whole-line deletions cannot introduce an error, so they skip both parses; a clean
    // result cannot have more errors than the old text, so it skips the baseline one.
    let woven = prog
        .root()
        .body
        .iter()
        .any(|s| doomed.contains(&s.id) && !matches!(s.chained, syntax::Chained::No));
    if woven {
        let errs = crate::syntax::parse(&text).1.len();
        if errs > 0 && errs > crate::syntax::parse(prog.text()).1.len() {
            return refuse();
        }
    }
    Edit { text, kind: Kind::Structural, names: Vec::new(), refused: None }
}

/// The names a statement refers to, of those given.
fn mentions(st: &Stmt, names: &std::collections::BTreeSet<String>) -> Vec<String> {
    let mut hit = Vec::new();
    let mut look = |r: &syntax::Ref| {
        if names.contains(&r.root.text) {
            hit.push(r.root.text.clone());
        }
    };
    match &st.kind {
        StmtKind::Group(g) => {
            let mut fields: Vec<&syntax::InstArg> = g.fields.iter().collect();
            while let Some(field) = fields.pop() {
                match &field.value {
                    syntax::InstVal::Ref(r) => look(r),
                    syntax::InstVal::Group(inner) => fields.extend(inner),
                    syntax::InstVal::Expr(_) | syntax::InstVal::Hint(..) => {}
                }
            }
        }
        StmtKind::Chain(c) => {
            for r in &c.links {
                look(r);
            }
        }
        StmtKind::Decl(d) => {
            for g in &d.children {
                for r in g.iter().flat_map(|k| k.refs()) {
                    look(r);
                }
            }
            // a plane's axes are its children, looked at above; a membership (`in …`) is a
            // label the point survives losing, and is not counted
            if let Some(motion) = &d.motion {
                match motion {
                    syntax::MotionSpec::Rotation {axis,..} | syntax::MotionSpec::Translation {axis,..} => look(axis),
                    syntax::MotionSpec::Relative {source,observer} => { look(source); look(observer); }
                }
                // a number measured off the drawing is defined from what it measures, and a
                // motion measuring a deleted line goes with it, as a dimension on it would
                let mut spec = motion.clone();
                for a in spec.args_mut() {
                    if let syntax::Arg::Dim {text,..} = a {
                        for n in crate::expr::measured_names(text) {
                            let root = n.split('.').next().unwrap_or_default();
                            look(&syntax::Ref {root:syntax::Name::new(root),path:vec![],span:Default::default()});
                        }
                    }
                }
            }
            // a curve is a point of an instance, and goes with what it is written over: the
            // instance's point, or — written in place — the entities the instance was given
            if let Some(c) = &d.curve {
                match &c.target {
                    syntax::CurveTarget::Drawn(r) => look(r),
                    syntax::CurveTarget::Anon(inst, _) => {
                        for a in &inst.args {
                            if let syntax::InstVal::Ref(r) = &a.value {
                                look(r);
                            }
                        }
                    }
                }
            }
        }
        // a set goes with what its body names (§6.21), as a component's call goes with its
        // arguments
        StmtKind::Set(set) => {
            for inner in &set.lit.body {
                hit.extend(mentions(inner, names));
            }
        }
        StmtKind::Relation(rel) => {
            for a in rel.form.canonical_args().iter().flatten() {
                if let syntax::Arg::Ref(r) = a {
                    look(r);
                }
            }
            // Written operators name operands and may name another entity in parentheses.
            if let Some(w) = rel.form.written() {
                for r in w.ops.iter() {
                    look(r);
                }
                for a in &w.args {
                    match a {
                        syntax::OpArg::Ent(r) | syntax::OpArg::Named(_, syntax::Arg::Ref(r)) => {
                            look(r)
                        }
                        _ => {}
                    }
                }
            }
        }
        _ => {}
    }
    hit
}

/// One trailing clause, written where the parser said it goes or taken out of the line.
///
/// **The separator dance, once.**  A clause the source wrote is *replaced* between the spaces
/// it already had; one it did not is *inserted* and brings the space that sets it off (the
/// empty-span idiom `class_span`, `plane_span` and `place_span` all use); and one that goes
/// takes the blanks in front of it with it, or the line is left with a gap.  `class`, `in` and
/// a callout's placement are all this, and it had been written out at five sites — where the
/// fiddly half is the same at every one and the wrong half is invisible until a statement
/// prints with two spaces or none.
fn clause_splice(text: &str, at: Span, with: Option<String>) -> Splice {
    match with {
        Some(s) if at.is_empty() => Splice { at, with: s },
        Some(s) => Splice { at, with: s.trim_start().to_string() },
        None => {
            let lo = back_over_spaces(text, at.lo as usize);
            Splice { at: Span::new(lo, at.hi as usize), with: String::new() }
        }
    }
}

/// Back over the blanks before an offset — so a clause deleted from the middle of a line does
/// not leave the space that set it off behind.
fn back_over_spaces(text: &str, mut lo: usize) -> usize {
    let b = text.as_bytes();
    while lo > 0 && (b[lo - 1] == b' ' || b[lo - 1] == b'\t') {
        lo -= 1;
    }
    lo
}

/// The splice that takes a doomed statement out of the text — `None` where there is none.
///
/// Almost always the statement's whole line.  A chain (spec §6.6) puts several statements on one
/// line, so which characters go is a question about *how the statement was written*, and the
/// parser answered it while desugaring: a joint steps down to `->` (the corner stays, the claim
/// goes, and the chain still parses), a prefix word is deleted where it stands with the spaces
/// that set it off, and a link has no splice at all — nothing takes one link out and leaves a
/// chain behind, so it is refused.  Reading the answer back out of the characters would instead
/// rest on "a longhand relation always carries a `(`", which nothing states and a qualified
/// joint would quietly break.
fn doom_splice(text: &str, st: &Stmt) -> Option<Splice> {
    doom_at(text, st.span, &st.chained)
}

/// The doom splices for a set of statements at once — and the composition the set needs: the
/// words of one joint hang together, so while each doomed alone splices out where it stands
/// and the rest hold the line, a joint whose every *written* word is doomed (an entity
/// deletion dooms every relation naming it) would leave two links with nothing between them.
/// Such a joint yields the one splice its only word's doom would be — the `fall` its members
/// carry — and its other members yield nothing.  Counted against `out_of`, the words as
/// written, so a word that was refused at desugar (emitting no statement) holds the joint's
/// text in place.  A `None` is a statement with no splice at all, the caller's refusal.
fn doomed_splices(
    text: &str,
    body: &[Stmt],
    doomed: &std::collections::BTreeSet<syntax::StmtId>,
) -> Vec<Option<Splice>> {
    let mut fell: std::collections::BTreeMap<Span, usize> = Default::default();
    for st in body.iter().filter(|s| doomed.contains(&s.id)) {
        if let syntax::Chained::Member { of, .. } = st.chained {
            *fell.entry(of).or_insert(0) += 1;
        }
    }
    let mut out = Vec::new();
    for st in body.iter().filter(|s| doomed.contains(&s.id)) {
        match st.chained {
            syntax::Chained::Member { of, fall, out_of } => match fell.get(&of) {
                // every written word fell: the joint composes to its only word's doom, once
                // — taking the entry marks it done
                Some(&n) if n == out_of as usize => {
                    fell.remove(&of);
                    out.push(doom_at(text, of, &fall.into()));
                }
                // siblings hold the line: this word goes out alone
                Some(_) => out.push(doom_splice(text, st)),
                // the joint's one composed splice went with its first member
                None => {}
            },
            _ => out.push(doom_splice(text, st)),
        }
    }
    out
}

/// The splice for one doomed spelling over one span — `doom_splice`'s own body, split out so
/// `remove` can compose a joint whose every word fell at once: it dooms the `fall` the members
/// carry, over the joint's whole span.
fn doom_at(text: &str, at: Span, how: &syntax::Chained) -> Option<Splice> {
    let one_word = |with: &str| Some(Splice { at, with: with.to_string() });
    match how {
        syntax::Chained::No => Some(Splice { at: with_line(text, at), with: String::new() }),
        syntax::Chained::Link => None,
        // a threaded joint steps down to the bare corner: the claim goes, the weld stays
        syntax::Chained::Joint => one_word("->"),
        // an unthreaded joint states only the relation, so its span — grown at desugar time
        // over a terminal name-link that would otherwise dangle — becomes a statement break;
        // one that is the whole of its line would leave only a blank one, so the line goes
        syntax::Chained::Infix => {
            let lo = back_over_spaces(text, at.lo as usize);
            let hi = skip_spaces(text, at.hi as usize);
            let whole = (lo == 0 || text.as_bytes()[lo - 1] == b'\n')
                && matches!(text.as_bytes().get(hi), None | Some(&b'\n'));
            match whole {
                true => Some(Splice { at: with_line(text, at), with: String::new() }),
                false => Some(Splice { at, with: "\n".to_string() }),
            }
        }
        // an unthreaded joint in a chain that closes: a break would re-aim the `close` at
        // another link, so there is no splice and the gesture is refused
        syntax::Chained::Stuck => None,
        // one of a joint's several words, or a prefix word, goes out where it stands with
        // the blanks after it — a comment or a line break beside it survives.  A doomed
        // member leaves the corner and the joint's other statements standing; the whole
        // joint doomed at once is composed by `doomed_splices` from the `fall` it carries
        syntax::Chained::Member { .. } | syntax::Chained::Prefix => Some(Splice {
            at: Span::new(at.lo as usize, skip_spaces(text, at.hi as usize)),
            with: String::new(),
        }),
        // the joint word and `close` share a span, and only the claim goes
        syntax::Chained::Close => {
            let tail = at.slice(text);
            let close = tail.rfind("close").map(|i| &tail[i..]).unwrap_or("close");
            one_word(&format!("-> {close}"))
        }
    }
}

/// Past the blanks at an offset — what a deletion swallows so it leaves no ragged gap.
fn skip_spaces(text: &str, mut hi: usize) -> usize {
    let b = text.as_bytes();
    while hi < b.len() && (b[hi] == b' ' || b[hi] == b'\t' || b[hi] == b'\r') {
        hi += 1;
    }
    hi
}

/// A statement's span, grown to swallow the newline that ends it — so deleting one does not
/// leave a blank line where it stood.
fn with_line(text: &str, s: Span) -> Span {
    let mut hi = skip_spaces(text, s.hi as usize);
    if text.as_bytes().get(hi) == Some(&b'\n') {
        hi += 1;
    }
    Span::new(s.lo as usize, hi)
}

/* -- editing a number -------------------------------------------------------------- */

/// Write a dimension's text — a number, or an expression somebody typed.
///
/// `Kind::Numeric` when the text is a plain number and was one before: the topology cannot have
/// moved, so a compiled plan survives.  A text that names anything is `Structural`, because the
/// name may be an unknown, and that *is* a column.  A number typed over a dimension that reads a
/// `param` (`distance(w)`) is written into the param's own line.
pub fn set_dimension(e: &Elaborated, prog: &Program, cid: u32, attr: &str, text: &str) -> Edit {
    let Some(site) = e.map.of_constraint.get(&cid) else {
        return Edit::none(prog, Some("no such constraint".into()));
    };
    if !site.path.0.is_empty() || !in_root(prog, site.stmt) {
        return Edit::none(prog, Some("that dimension is written inside a component".into()));
    }
    let Some(st) = prog.stmt(site.stmt) else { return Edit::none(prog, None) };
    let StmtKind::Relation(rel) = &st.kind else { return Edit::none(prog, None) };
    // the number a statement states is the unlabelled thing in its operator's parentheses
    // (spec §9.1), which is where a *written* statement carries it
    let dim = match &rel.form {
        syntax::RelationForm::Written(w) => w.args.iter().find_map(|a| match a {
            syntax::OpArg::Dim(text, span) => Some((*span, text.clone())),
            _ => None,
        }).or_else(|| word_dimension(prog, w)),
        syntax::RelationForm::Canonical { kind, args } => kind
            .spec()
            .iter()
            .position(|(n, _)| *n == attr)
            .and_then(|i| args.get(i).and_then(|a| a.as_ref()))
            .and_then(|a| match a {
                syntax::Arg::Dim { span, text } => Some((*span, text.clone())),
                _ => None,
            }),
    };
    let Some((span, was)) = dim else {
        return Edit::none(prog, Some("that argument is not a dimension".into()));
    };
    let was = was.as_str();
    // a dimension that reads a `param` (`distance(w)`) is edited where the number is: a plain
    // number typed over it is the param's new value, and every dimension reading it follows
    if writable_seed(text) {
        let param = prog.root().body.iter().find_map(|st| match &st.kind {
            StmtKind::Param(p) if p.name.text == was.trim() && p.bound() => Some(p),
            _ => None,
        });
        if let Some(p) = param {
            let value = Splice { at: p.span, with: text.trim().to_string() };
            return Edit::spliced(prog, vec![value], Kind::Structural);
        }
    }
    let plain = crate::expr::literal(text).is_some() && crate::expr::literal(was).is_some();
    let number = Splice { at: span, with: text.trim().to_string() };
    Edit::spliced(prog, vec![number], if plain { Kind::Numeric } else { Kind::Structural })
}

/// **A defined word's number, where the statement wrote it** (§9.9): where the word's body
/// states its dimension as one of its parameters (`a above(d) b := b distance(d, along: up) a`),
/// the argument the statement gave that parameter (`r above(d: 7) q`'s `7`) — the text the
/// callout draws, so the text an edit of it writes.
fn word_dimension(prog: &Program, w: &syntax::Written) -> Option<(Span, String)> {
    let k = prog.resolve_word(&w.word.text, w.fixity, None)?;
    let body = prog.words[k].body.form.written()?;
    let param = body.args.iter().find_map(|a| match a {
        syntax::OpArg::Dim(t, _) => Some(t.trim()),
        _ => None,
    })?;
    w.args.iter().find_map(|a| match a {
        syntax::OpArg::Named(n, syntax::Arg::Dim { text, span }) if n.text == param => {
            Some((*span, text.clone()))
        }
        _ => None,
    })
}

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
    let prog = &prog;
    let mut names = Vec::new();
    let mut adds: Vec<StmtKind> = Vec::new();
    // what an appended statement was written *for*, so the map can be extended onto it without
    // elaborating anything
    let mut made: Vec<Made> = Vec::new();

    // what the elaboration made, per kind, is the high-water mark: past it is new
    let mut high: std::collections::BTreeMap<EntKind, usize> = std::collections::BTreeMap::new();
    for r in e.map.of_entity.keys() {
        let n = high.entry(r.kind).or_insert(0);
        *n = (*n).max(r.i() + 1);
    }
    // a name for each new entity first, so a line drawn between two new points can refer to them
    let mut minted: std::collections::BTreeMap<EntRef, String> = std::collections::BTreeMap::new();
    let mut taken = taken_names(prog);
    // a plane's origin is the plane's own, minted with it and never declared apart: it is
    // called `P.origin` after its plane
    let origin_of = |r: EntRef| {
        (r.kind == EntKind::Point)
            .then(|| sk.plane_of_origin(r.i()))
            .flatten()
    };
    for r in sk.primitives() {
        if r.i() < high.get(&r.kind).copied().unwrap_or(0) || origin_of(r).is_some() {
            continue;
        }
        minted.insert(r, next_name(&mut taken, r.kind));
    }

    /* An **anonymous declaration** (issue #33) has no name a statement can say: the `#`-keyed
     * name it resolves by is the elaboration's, not the source's.  So whatever a new statement
     * is about to reach for is named *now* — a real name minted and spliced into the
     * declaration, at the empty span the parser recorded where one would go — the same bargain
     * `commit_seeds` strikes with an unwritten `hint(…)` clause.  On demand, and only on
     * demand: an anonymous element nothing references stays unnamed. */
    let mut named: Vec<Splice> = Vec::new();
    let mut renamed: std::collections::BTreeMap<EntRef, String> = std::collections::BTreeMap::new();
    // **What the sketch holds fixed, walked once**: the naming pass below must know what a gauge
    // will have to name, and `gauges` writes those same holds out as statements further down.
    // Two walks were two readings of one question — and the walk is not cheap, `root_declared`
    // ending in a scan of the root body.
    let held = held_refs(sk, &|r| root_declared(e, prog, r));
    // Everything a statement this reconcile appends will have to *say the name of*.  Small: a
    // gesture states one or two constraints, and every entity that already has a written name
    // falls out at the first test below.
    let mut needed: std::collections::BTreeSet<EntRef> = std::collections::BTreeSet::new();
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
            needed.insert(EntRef::plane(p));
        }
    }
    // …a gauge names what it holds…
    needed.extend(held.iter().map(|(r, _)| *r));
    /* …and a membership names its plane.  `point a in top` is neither an entity nor a
     * constraint, so nothing above notices it: it is read off the sketch and compared with
     * what the statement says, declaration by declaration, the way a class is — but worked out
     * *here*, since the plane it will name may be one nothing has named yet. */
    let mut memberships: Vec<(&Decl, Option<usize>)> = Vec::new();
    // a drawing with no view in it has no membership to write, and the walk below is a scan of
    // the root body per entity — so the question is asked once, of the sketch, first
    for (r, site) in e.map.of_entity.iter().filter(|_| !sk.planes.is_empty()) {
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
            .filter(|p| p.kind == EntKind::Plane)
            .map(|p| p.i());
        if now == was {
            continue;
        }
        // a membership the `in … { }` block around the statement gave it: there is no clause
        // here to splice, and writing one would say `in` twice
        // the clause is not this statement's to rewrite — a block's, or an enclosing
        // instance's — and which it is is the membership's to say, not this sentence's
        if !d.membership.editable() {
            return Edit::none(
                prog,
                Some(format!(
                    "that point is {}, so its plane is not this statement's to change; \
                     move the statement in the source instead",
                    d.membership.cause()
                )),
            );
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
            return Edit::none(
                prog,
                Some(format!(
                    "the points of `{}` are on different planes, which one statement cannot say",
                    d.name.shown().map_or_else(|| d.kind.as_str().to_string(), |n| n.text.clone())
                )),
            );
        }
        if let Some(p) = now {
            needed.insert(EntRef::plane(p));
        }
        memberships.push((d, now));
    }
    // a plane's origin is said through its plane's name
    let needed: std::collections::BTreeSet<EntRef> = needed
        .into_iter()
        .map(|r| match origin_of(r) {
            Some(p) if e.map.writable_name(r).is_none() => EntRef::plane(p),
            _ => r,
        })
        .collect();
    for r in &needed {
        if minted.contains_key(r) || renamed.contains_key(r) {
            continue; // new (its statement carries the name), or its statement already named
        }
        if let Some(member) = e.map.name_of(*r).and_then(|name| e.map.private_from_root(name)) {
            return Edit::none(prog, Some(format!(
                "`{member}` is private to its component; add the relation inside that component")));
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
            return Edit::none(
                prog,
                Some(
                    "that is one copy of a block, and only the flattener has a name for it; \
                     write the statement inside the block, where it holds for every copy"
                        .into(),
                ),
            );
        }
        let Some(site) = e.map.of_entity.get(r) else { continue };
        if site.path.0.iter().any(|step| match step {
            crate::ir::PathStep::Instance(id) => matches!(prog.stmt(*id).map(|s| &s.kind),
                Some(StmtKind::Instance(i)) if i.name.text.starts_with('#')),
            _ => false,
        }) {
            return Edit::none(prog, Some(
                "give the component call a name before referring to its geometry".into()));
        }
        if !site.path.0.is_empty() || !in_root(prog, site.stmt) {
            // Nothing calls it and there is no statement of the root's to put a name on: it is
            // anonymous inside a component or a block, so refuse now with the cause rather than
            // later with none.
            return Edit::none(
                prog,
                Some(
                    "that is anonymous inside a component or a block, so no statement \
                     can name it; give its declaration a name there first"
                        .into(),
                ),
            );
        }
        let Some(d) = decl_of(prog, site) else { continue };
        if d.name.shown().is_some() {
            continue; // named since the map was made
        }
        // One name per statement, and *every* entity the statement made follows it at once —
        // the declaration first, then the children it minted, which is the order they were
        // recorded in.  All of them, because a later gesture in this same elaboration must
        // find a name where this one left the source saying a name.
        let mut made_ents = e.map.ents_made_by(site.stmt);
        let Some(parent) = made_ents.next() else { continue };
        let name = next_name(&mut taken, d.kind);
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

    // the elaboration's own name for an entity it made — what a new statement has to refer to.
    // Never a key: the map files one under `by_name` alone, so what it *calls* an entity is a
    // name a statement may say or nothing at all.
    let plain = |r: EntRef| -> String {
        minted
            .get(&r)
            .cloned()
            .or_else(|| renamed.get(&r).cloned())
            .or_else(|| e.map.name_of(r).cloned())
            .unwrap_or_else(|| syntax::entity_name(r))
    };
    let name_of = |r: EntRef| -> String {
        match origin_of(r) {
            Some(p) if e.map.name_of(r).is_none() => format!("{}.origin", plain(EntRef::plane(p))),
            _ => plain(r),
        }
    };

    for r in sk.primitives() {
        let Some(name) = minted.get(&r) else { continue };
        let mut d = crate::program::lift_decl(sk, r);
        d.name = syntax::DeclName::Written(syntax::Name::new(name.clone()));
        rename_children(&mut d, sk, r, &name_of);
        // the plane its points are on, by the name the document calls it
        d.membership = crate::program::plane_of_entity(sk, r)
            .map(|p| syntax::Membership::lifted(syntax::Ref::new(name_of(EntRef::plane(p)))))
            .unwrap_or_default();
        names.push(name.clone());
        made.push(Made::Ent(r));
        adds.push(StmtKind::Decl(d));
    }

    // constraints: an id the map does not know is new, one it knows that is gone was removed
    let live: std::collections::BTreeSet<u32> =
        sk.user_constraints().iter().map(|c| c.id).collect();
    for c in sk.user_constraints() {
        if e.map.of_constraint.contains_key(&c.id) {
            continue;
        }
        let mut rel = crate::program::lift_relation(sk, c);
        // a lift names entities positionally (`P3`); the document calls them what it calls them
        for (a, ca) in rel.form.canonical_args_mut().iter_mut().zip(c.args.iter()) {
            if let (Some(syntax::Arg::Ref(r)), crate::constraints::Arg::Ent(er)) = (a, ca) {
                *r = syntax::Ref::new(name_of(*er));
            }
        }
        made.push(Made::Con(c.id));
        adds.push(StmtKind::Relation(rel));
    }
    let mut doomed: std::collections::BTreeSet<crate::syntax::StmtId> =
        std::collections::BTreeSet::new();
    for (id, site) in e.map.of_constraint.iter() {
        if !live.contains(id) && site.path.0.is_empty() && in_root(prog, site.stmt) {
            doomed.insert(site.stmt);
        }
    }
    // a statement that made two things is only gone when both are
    for (_, site) in e.map.of_constraint.iter().filter(|(id, _)| live.contains(id)) {
        doomed.remove(&site.stmt);
    }
    for site in e.map.of_entity.values() {
        doomed.remove(&site.stmt);
    }

    // Presentation stays in the drawing/editor, never in model source.
    let mut flags: Vec<Splice> = Vec::new();
    // memberships, worked out above and written now that every plane has a name: the clause
    // replaced where it stands, written where the parser said one would go, or taken out
    // with the space in front of it
    for (d, now) in memberships {
        let with = now.map(|p| format!(" in {}", name_of(EntRef::plane(p))));
        flags.push(clause_splice(prog.text(), d.membership.span(), with));
    }
    // `fix((0, 0)) p`: a statement per held entity, added and taken away — the holds
    // walked once, above, and named here now that there is a name for each
    let held_now: std::collections::BTreeMap<GaugeKey, (EntRef, &[(&str, f64)])> =
        held.iter()
            .map(|(r, h)| (gauge_key_of(name_of(*r), h.iter().map(|(f, _)| *f)), (*r, h.as_slice())))
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
        if e.map.ent_named(&k.0).is_none() {
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
        let point = crate::program::point_len(sk, r);
        adds.push(StmtKind::Relation(crate::program::lift_gauge(&k.0, r.kind, point, h)));
        made.push(Made::Gauge);
    }

    if adds.is_empty() && doomed.is_empty() && flags.is_empty() && named.is_empty() {
        // nothing structural: the drawing only moved, and the seeds record where to
        let seeds = commit_seeds(e, sk, prog);
        if seeds.kind != Kind::None {
            e.retext(&seeds.text);
        }
        return seeds;
    }

    // composed, so a joint whose every word fell at once — both relations of one run
    // withdrawn together — is unwritten as one splice rather than left as two links with
    // nothing between them; a statement with no splice is dropped, and `adopt` is the net
    let mut edits: Vec<Splice> =
        doomed_splices(prog.text(), &prog.root().body, &doomed).into_iter().flatten().collect();
    if !adds.is_empty() {
        let (at, lead) = append_at(prog);
        let mut line = String::new();
        for (i, k) in adds.iter().enumerate() {
            // the first joins what is already there the way `append` does; each one after it
            // starts a line of its own, or they would all run together on one
            line.push_str(if i == 0 { lead.as_str() } else { "\n" });
            if let Err(e) = syntax::write_stmt_to(&mut line, k) {
                return Edit::none(prog, Some(e.to_string()));
            }
        }
        edits.push(Splice { at, with: line });
    }
    // after the append, deliberately: insertions at one offset land in *reverse* application
    // order, and `splice`'s stable sort applies equal offsets in this vec's order — so where
    // the file's last statement is the one being named or flagged, this order is the layout:
    // the appended statements go past the line's end, a class clause stands before them, and a
    // minted name lands against its keyword with everything else after it
    edits.extend(flags);
    edits.extend(named);
    let text = splice(prog.text(), edits);
    if !e.adopt(&text, &made) {
        return Edit::none(prog, Some("the drawing could not be written down".into()));
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
    Edit { text, kind: Kind::Structural, names, refused: None }
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
        syntax::RelationForm::Canonical { kind: CKind::Fix, args } => {
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
/// held.  Names are put to it only at the second, by which point every hold has one.
fn held_refs(
    sk: &Sketch,
    ours: &dyn Fn(EntRef) -> bool,
) -> Vec<(EntRef, Vec<(&'static str, f64)>)> {
    sk.primitives()
        .into_iter()
        .filter(|&r| ours(r))
        .map(|r| (r, crate::program::holds(sk, r)))
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
