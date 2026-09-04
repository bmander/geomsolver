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
}

/// One replacement in the source.
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
/// `hint(…)` is one a solve may move, and every other number — `== 80`, `param w = 100` — is not.
/// The second keeps `hint(r: Rr)` — a radius written in terms of a component's parameter — from
/// being overwritten with the number it happened to come to.  The third is why a point inside a
/// `cycle` of thirty does not write back at all: thirty instances share one statement, and there
/// is no one pose to record.
///
/// A seed the source **never wrote** is the case the clause makes real: a radius and a frame's
/// rotor are seeds a person may perfectly well omit, and a solve moves them anyway.  There is
/// then no span to splice, so the clause is written out whole at the point the parser recorded
/// for it (`Decl::hint_span`) — one splice, and the statement around it untouched.  Leaving it
/// alone instead would mean a drawing whose pose its source cannot express.
///
/// `Kind::Numeric`, always: a seed is not a statement, so nothing recompiles.
pub fn commit_seeds(e: &Elaborated, sk: &Sketch, prog: &Program) -> Edit {
    // The walk is over the root component's own statements, which is the question `in_root` was
    // asking one statement at a time: a statement inside a component is written in the
    // component's terms, and a pose put there is a pose put on every instance of it.  What each
    // one made is `SourceMap::ents_made_by`, whose order — the declaration's own entity first,
    // then the children it minted — is what `reconcile` reads too, so neither the entity index
    // nor the find-by-kind that re-derived the parent is needed.
    let mut edits = Vec::new();
    for st in &prog.root().body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        // `hint(at: t)` names a *place*, and has no coordinates to write.  Faces and
        // solids also own no seeds: their children are boundaries and operands, not points.
        if d.seed_at.is_some() || d.kind.spatial() {
            continue;
        }
        // a declaration that could not be built made nothing, and has no pose to record
        let Some(parent) = e.map.ents_made_by(st.id).next() else { continue };
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
            syntax::Kid::Hint(_) => true,
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
        for (i, p) in sk.own_params(parent).iter().enumerate() {
            let v = sk.params[*p as usize].value;
            let text = d.seed_text.get(i).and_then(|t| t.as_ref());
            let (sp, miss) = one(v, text, d.seed_spans.get(i).copied().unwrap_or_default());
            mine.extend(sp);
            missing |= miss;
        }
        // An anonymous child's seed lives in the parent's statement, in a slot of its own — and
        // the slots stand in the order `sk.children` hands the children back in, so the two walk
        // together.  A slot the source wrote a *name* in is a point declared elsewhere, and is
        // written back where it was declared; one it wrote nothing for at all was minted.
        for (j, &k) in kids.iter().enumerate() {
            let seed = match slot_kid(j) {
                Some(syntax::Kid::Ref(_)) => continue,
                Some(syntax::Kid::Hint(s)) => Some(s),
                None => None,
            };
            let v = sk.point_params(k.i()).map(|p| sk.params[p as usize].value);
            match seed {
                // A slot that keyed one coordinate and left the other out has nowhere to splice
                // the one it left out, and the clause it is written in is the smallest thing
                // that can carry both — which is what `KidSeed::span` is for.  Not when a
                // coordinate is an expression: rewriting the clause would rewrite the arithmetic.
                Some(s)
                    if s.spans.iter().any(|sp| sp.is_empty())
                        && s.text.iter().all(|t| t.is_none()) =>
                {
                    let now = syntax::hint_xy(v[0], v[1]);
                    if s.span.slice(prog.text()) != now {
                        mine.push(Splice { at: s.span, with: now });
                    }
                }
                Some(s) => {
                    for i in 0..2 {
                        let (sp, miss) = one(v[i], s.text[i].as_ref(), s.spans[i]);
                        mine.extend(sp);
                        missing |= miss;
                    }
                }
                None => missing |= v.iter().any(|&c| c != 0.0),
            }
        }

        if !missing {
            edits.extend(mine);
            continue;
        }
        // Something the source never wrote has moved — an omitted radius, an endpoint of a bare
        // `line l`.  There is nowhere to splice, so what the source left out is written: the
        // argument list, the `hint(…)` clause, or both.
        let Some(at) = d.hint_span else { continue };
        let mut pose = d.seed.clone();
        for (i, p) in sk.own_params(parent).iter().enumerate() {
            if let Some(v) = pose.get_mut(i) {
                *v = sk.params[*p as usize].value;
            }
        }
        // the clause, as the pose the solve arrived at; empty when the kind owns no scalar at
        // all — a line's numbers are its two points', and they are written in the slots
        let hint = syntax::hint_clause(d, &pose);
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
                        syntax::Kid::Hint(_) => false,
                    }),
                "an unwritable reference outside a chain has nothing to re-thread its slot",
            );
            let mut filled = kids.iter().enumerate().map(|(j, k)| match slot_kid(j) {
                Some(syntax::Kid::Ref(r)) if syntax::hidden(&r.root.text) => None,
                Some(syntax::Kid::Ref(r)) => Some(syntax::Kid::Ref(r.clone())),
                _ => {
                    let v = sk.point_params(k.i()).map(|p| sk.params[p as usize].value);
                    Some(syntax::Kid::Hint(syntax::KidSeed { v, ..Default::default() }))
                }
            });
            for g in d2.children.iter_mut() {
                *g = filled.by_ref().take(g.len().max(1)).flatten().collect();
            }
            (syntax::decl_args(&d2), syntax::decl_tail(&d2, &pose))
        });
        if list.is_none() && hint.is_empty() {
            // nothing to write here: what moved is in a slot the source wrote, and splices there
            edits.extend(mine);
            continue;
        }
        // a slot the source *did* write still splices in place; only what it did not is here
        edits.extend(mine.into_iter().filter(|s| s.at.lo >= at.hi || s.at.hi <= at.lo));
        match list {
            // Both are missing and both would go at the same offset — the parser records the
            // clause's home just past the name when there is no clause — so they are written as
            // one edit.  Two insertions at one position would race for it.
            Some((_, tail)) if at.is_empty() && d.list_span.is_empty() => {
                edits.push(Splice { at, with: tail })
            }
            // The clause has a home of its own, and the *list* belongs to the name: written at
            // the clause's position it would land past whatever trailer stands between them,
            // where an argument list is not a thing a declaration can say.  The list is
            // *replaced* where one stands — a plane that wrote its attitude and no children
            // has a list none of whose slots is its own, and a second list beside the first
            // would be two — and inserted at the name's end where none does.
            Some((args, _)) => {
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
    if edits.is_empty() {
        return Edit::none(prog, None);
    }
    Edit {
        text: splice(prog.text(), edits),
        kind: Kind::Numeric,
        names: Vec::new(),
        refused: None,
    }
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
    match root.body.last() {
        Some(st) => (Span::new(st.span.hi as usize, st.span.hi as usize), "\n".to_string()),
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
    syntax::write_stmt_to(&mut line, &kind);
    Edit {
        text: splice(prog.text(), vec![Splice { at, with: line }]),
        kind: Kind::Structural,
        names,
        refused: None,
    }
}

/// `point pN hint(x: …, y: …)`
/// The rectangle the Rect tool draws: a **reusable component**, defined once per document, and
/// one instance per gesture.  The definition is the chain a person would write — four lines
/// welded corner to corner at right angles, the first two carrying the width and the height —
/// so what a gesture leaves in the source is one statement, `r0: Rectangle(w: 120, h: 60)`,
/// and the drawing owns a `Rectangle` any later statement may instance again.  Where the
/// figure *sits* is not in the statement: an instance's geometry is written in the component's
/// terms, so its pose is the session's (the tool seeds it at the gesture) and a reload starts
/// it from `scatter`, the same bargain every component's interior strikes.
pub fn add_rectangle(prog: &Program, w: f64, h: f64, plane: Option<&str>) -> Edit {
    let (at, lead) = append_at(prog);
    let mut with = lead;
    if !prog
        .components
        .iter()
        .any(|c| c.name.as_ref().is_some_and(|n| n.text == "Rectangle"))
    {
        if at.lo > 0 {
            with.push('\n'); // a blank line between the drawing and the definition
        }
        with.push_str(
            "component Rectangle(w: Length, h: Length) {\n  distance(w) line l1 -> \
             perpendicular distance(h) line l2 -> perpendicular line l3 -> perpendicular \
             line l4 -> close\n}\n\n",
        );
    }
    // a fresh instance name, past every name the document already binds
    let taken: std::collections::BTreeSet<&str> = prog
        .stmts()
        .filter_map(|s| match &s.kind {
            StmtKind::Decl(d) => Some(d.name.key().text.as_str()),
            StmtKind::Instance(i) => Some(i.name.text.as_str()),
            StmtKind::Param(p) => Some(p.name.text.as_str()),
            _ => None,
        })
        .collect();
    let name =
        (0..).map(|i| format!("r{i}")).find(|n| !taken.contains(n.as_str())).unwrap_or_default();
    let arg = |label: &str, v: f64| syntax::InstArg {
        label: Some(syntax::Name::new(label)),
        value: syntax::InstVal::Expr(num(v)),
        span: Span::default(),
    };
    let inst = syntax::Instance {
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
    syntax::write_stmt_to(&mut line, &StmtKind::Instance(inst));
    with.push_str(&line);
    Edit {
        text: splice(prog.text(), vec![Splice { at, with }]),
        kind: Kind::Structural,
        names: vec![name],
        refused: None,
    }
}

pub fn add_point(prog: &Program, x: f64, y: f64) -> Edit {
    let name = mint(prog, EntKind::Point);
    let d = Decl {
        kind: EntKind::Point,
        name: syntax::DeclName::Written(syntax::Name::new(name.clone())),
        children: Vec::new(),
        seed: vec![x, y],
        seed_text: vec![None, None],
        seed_spans: Vec::new(),
        hint_span: None,
        knots: None,
        curve: None,
        computed: None,
        class: Default::default(),
        class_span: Span::default(),
        seed_at: None,
        seed_names: Vec::new(),
        attitude: Default::default(),
        sweep: None,
        membership: Default::default(),
        list_span: Span::default(),
        close: None,
    };
    append(prog, StmtKind::Decl(d), vec![name])
}

/// An entity built from names that already exist — a line from two points, a circle from a centre.
pub fn add_entity(prog: &Program, kind: EntKind, args: &[String], seed: &[f64]) -> Edit {
    add_entity_with(prog, kind, args, seed, Default::default(), None, &[])
}

/// A plane, with the attitude its statement will spell — folded from another, or given a basis
/// — and, when the caller has one, the name it asked for.  A name already in use is refused
/// rather than silently renamed: the caller is about to refer to it.
///
/// `places` seeds the origin and the toward point *in the statement* (`origin: hint(x: …)`),
/// for a slot `args` leaves unnamed.  In the statement and not written into the points
/// afterwards, because a datum's rotor and its chord-length unknown are seeded from the chord
/// when the plane is *built*: two points moved by hand after the fact leave both stale, and a
/// solve from there lands on the degenerate frame with its two points together.
pub fn add_plane(
    prog: &Program,
    args: &[String],
    attitude: syntax::Attitude,
    name: Option<&str>,
    places: &[(f64, f64)],
) -> Edit {
    add_entity_with(prog, EntKind::Plane, args, &[], attitude, name, places)
}

fn add_entity_with(
    prog: &Program,
    kind: EntKind,
    args: &[String],
    seed: &[f64],
    attitude: syntax::Attitude,
    name: Option<&str>,
    places: &[(f64, f64)],
) -> Edit {
    if kind == EntKind::Point || kind == EntKind::Curve {
        return Edit::none(prog, Some(format!("a {} is not built this way", kind.as_str())));
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
    // a place seeds a child slot nothing named — the same `hint(…)` clause, one level down
    for (g, &(x, y)) in children.iter_mut().zip(places) {
        if g.is_empty() {
            g.push(syntax::Kid::Hint(syntax::KidSeed { v: [x, y], ..Default::default() }));
        }
    }
    let n_scalar = kind.fields().iter().filter(|(_, f)| *f == crate::model::Field::Scalar).count();
    let d = Decl {
        kind,
        name: syntax::DeclName::Written(syntax::Name::new(name.clone())),
        children,
        seed: (0..n_scalar).map(|i| seed.get(i).copied().unwrap_or(0.0)).collect(),
        seed_text: vec![None; n_scalar],
        seed_spans: Vec::new(),
        hint_span: None,
        knots: None,
        curve: None,
        computed: None,
        class: Default::default(),
        class_span: Span::default(),
        seed_at: None,
        seed_names: Vec::new(),
        attitude,
        // a gesture never draws a solid: the sheet is where the drawing is, and a solid is
        // written over what is drawn there
        sweep: None,
        membership: Default::default(),
        list_span: Span::default(),
        close: None,
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
    // the statements stay — page geometry now, exactly as a clause's point stays.  The block's
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
        StmtKind::Decl(d) => {
            for g in &d.children {
                for r in g.iter().filter_map(|k| k.as_ref()) {
                    look(r);
                }
            }
            // a plane folded from a deleted one is defined from nothing, and goes with it; a
            // membership (`in …`) is a label the point survives losing, and is not counted
            if let Some(r) = d.attitude.plane_ref() {
                look(r);
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
        StmtKind::Relation(rel) => {
            for a in rel.args.iter().flatten() {
                if let syntax::Arg::Ref(r) = a {
                    look(r);
                }
            }
            // what a *written* statement names: its operands, and a third entity in the
            // parentheses.  `args` is the settled form and is empty until elaboration.
            if let Some(w) = &rel.poly {
                for r in w.ops.iter() {
                    look(r);
                }
                for a in &w.args {
                    if let syntax::OpArg::Ent(r) = a {
                        look(r);
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
/// moved, so a compiled plan survives.  A text that names anything is `Structural`, because a
/// name nothing defines is a free variable and that *is* a column.
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
    let dim = match &rel.poly {
        Some(w) => w.args.iter().find_map(|a| match a {
            syntax::OpArg::Dim(text, span) => Some((*span, text.clone())),
            _ => None,
        }),
        None => rel
            .kind
            .spec()
            .iter()
            .position(|(n, _)| *n == attr)
            .and_then(|i| rel.args.get(i).and_then(|a| a.as_ref()))
            .and_then(|a| match a {
                syntax::Arg::Dim { span, text } => Some((*span, text.clone())),
                _ => None,
            }),
    };
    let Some((span, was)) = dim else {
        return Edit::none(prog, Some("that argument is not a dimension".into()));
    };
    let was = was.as_str();
    let plain = crate::expr::literal(text).is_some() && crate::expr::literal(was).is_some();
    Edit {
        text: splice(prog.text(), vec![Splice { at: span, with: text.trim().to_string() }]),
        kind: if plain { Kind::Numeric } else { Kind::Structural },
        names: Vec::new(),
        refused: None,
    }
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
    for r in sk.primitives() {
        if r.i() < high.get(&r.kind).copied().unwrap_or(0) {
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
            .and_then(|p| e.map.ent_named(&p.root.text))
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
        // a declaration with no clause whose every point is declared elsewhere — `line l(a, b)`
        // — says nothing about planes; its points' own declarations do.  **Before the straddle
        // refusal below**: a line drawn between a point in a view and a point on the page is
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
    for r in &needed {
        if minted.contains_key(r) || renamed.contains_key(r) {
            continue; // new (its statement carries the name), or its statement already named
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
        named.push(Splice { at: d.name.span(), with: format!(" {name}") });
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
        let paths = crate::program::child_names(d, &name);
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
    let name_of = |r: EntRef| -> String {
        minted
            .get(&r)
            .cloned()
            .or_else(|| renamed.get(&r).cloned())
            .or_else(|| e.map.name_of(r).cloned())
            .unwrap_or_else(|| syntax::entity_name(r))
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
        for (a, ca) in rel.args.iter_mut().zip(c.args.iter()) {
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

    /* classes and gauges.  A `class` clause and a `ground` are neither an entity nor a
     * constraint, so nothing above notices them: they are read off the sketch and compared
     * against what the source says, statement by statement. */
    let mut flags: Vec<Splice> = Vec::new();
    for (r, site) in e.map.of_entity.iter() {
        if !site.path.0.is_empty() || !in_root(prog, site.stmt) {
            continue;   // inside a component: one statement, many instances, no one class list
        }
        // A declaration's class belongs to its own entity, not to children it minted (a
        // face's closing lines, for example, carry `.closure` independently of the face).
        if e.map.ents_made_by(site.stmt).next() != Some(*r) {
            continue;
        }
        let Some(d) = decl_of(prog, site) else { continue };
        let now = sk.class_of(*r);
        if now == d.class {
            continue;
        }
        // the whole clause, replaced where it stands — or written at the point the parser
        // recorded for it, which is where it would have gone
        let with = (!now.is_empty()).then(|| format!(" class {}", now.0.join(" ")));
        flags.push(clause_splice(prog.text(), d.class_span, with));
    }
    // memberships, worked out above and written now that every plane has a name: the clause
    // replaced where it stands, written where the parser said one would go, or taken out
    // with the space in front of it
    for (d, now) in memberships {
        let with = now.map(|p| format!(" in {}", name_of(EntRef::plane(p))));
        flags.push(clause_splice(prog.text(), d.membership.span(), with));
    }
    /* where a callout sits.  A placement is document state saved on the statement it qualifies
     * (spec §13.1), so a callout dragged somewhere else is a source edit like any other — and
     * one nothing above notices, since it makes no entity and no constraint.  Read off the
     * sketch and compared with what the statement says, exactly as the construction word is. */
    for (id, site) in e.map.of_constraint.iter() {
        if !site.path.0.is_empty() || !in_root(prog, site.stmt) {
            continue;
        }
        let Some(st) = prog.stmt(site.stmt) else { continue };
        let StmtKind::Relation(rel) = &st.kind else { continue };
        let now = sk.placements.get(id).copied();
        if now == rel.place {
            continue;
        }
        match now {
            // rewrite the two numbers where they stand, or write the whole clause at the
            // spot the parser recorded for it — `place_span` as an empty span is where one
            // *would* go, the `class_span` idiom.  A statement whose line offers no spot (it
            // shares the line's relations, or the line ends in a declaration) records none,
            // and the pose stays the layout's — the bargain `commit_seeds` strikes with a
            // decl seeded by place.
            // the one guard this clause has that the others do not: a line with no spot for a
            // placement records `Span::default()`, and the pose stays the layout's
            Some((t, r)) if rel.place_span != Span::default() => {
                let with = format!(" at ({}, {})", num(t), num(r));
                flags.push(clause_splice(prog.text(), rel.place_span, Some(with)));
            }
            // back where the layout would put it: the clause goes, and the space before it
            None if !rel.place_span.is_empty() => {
                flags.push(clause_splice(prog.text(), rel.place_span, None));
            }
            _ => {}
        }
    }
    // `ground(p)` and `fix(c.r)`: a statement per held parameter, added and taken away — the
    // holds walked once, above, and named here now that there is a name for each
    let held_now: std::collections::BTreeSet<(String, Option<String>)> =
        held.iter().map(|(r, f)| (name_of(*r), f.map(str::to_string))).collect();
    let held_was: std::collections::BTreeSet<(String, Option<String>)> = prog
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
        if !held_now.contains(&k) {
            doomed.insert(st.id);
        }
    }
    for k in held_now.iter() {
        if held_was.contains(k) {
            continue;
        }
        adds.push(StmtKind::Relation(crate::program::lift_gauge(&k.0, k.1.as_deref())));
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
            syntax::write_stmt_to(&mut line, k);
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

/// What a `ground` or a `fix` statement holds — a name, and a field when it is one scalar
/// rather than a point — and `None` for a relation that is neither.  Read off the word as it
/// was written, or off the kind of one that was built (`program::lift_gauge`).
fn gauge_key(r: &syntax::Relation) -> Option<(String, Option<String>)> {
    use crate::constraints::CKind;
    let (kind, rf) = match &r.poly {
        Some(w) => (crate::constraints::gauge_op(&w.word.text)?, w.ops.first()?),
        None => match r.args.first() {
            Some(Some(syntax::Arg::Ref(rf))) => (r.kind, rf),
            _ => return None,
        },
    };
    if !matches!(kind, CKind::Ground | CKind::Fix) {
        return None;
    }
    let field = match rf.path.first() {
        Some(syntax::Seg::Field(n)) => Some(n.text.clone()),
        _ => None,
    };
    Some((rf.root.text.clone(), field))
}

/// Everything the sketch itself holds fixed — the entity, and the field when it is one scalar
/// rather than a point — before any name is put to it.  **The one walk**, made once per
/// reconcile: the anonymous-naming pass reads it to know what a gauge will have to name, and the
/// gauge statements are written from the same list, so the two cannot disagree about what is
/// held.  Names are put to it only at the second, by which point every hold has one.
fn held_refs(sk: &Sketch, ours: &dyn Fn(EntRef) -> bool) -> Vec<(EntRef, Option<&'static str>)> {
    let mut out = Vec::new();
    for i in 0..sk.points.len() {
        if sk.point_fixed(i) && ours(EntRef::point(i)) {
            out.push((EntRef::point(i), None));
        }
    }
    for r in sk.primitives() {
        if r.kind == EntKind::Point || !ours(r) {
            continue;
        }
        let scalars: Vec<&'static str> = r
            .kind
            .fields()
            .iter()
            .filter(|(_, f)| *f == crate::model::Field::Scalar)
            .map(|(n, _)| *n)
            .collect();
        for (i, &pi) in sk.own_params(r).iter().enumerate() {
            if sk.params[pi as usize].fixed {
                out.push((r, Some(scalars.get(i).copied().unwrap_or("r"))));
            }
        }
    }
    out
}

/// Whether an entity's declaration is a statement of the root — as against one a component made,
/// which is where its `ground` is written too, and neither is ours to add or take away.
fn root_declared(e: &Elaborated, prog: &Program, r: EntRef) -> bool {
    match e.map.of_entity.get(&r) {
        Some(site) => site.path.0.is_empty() && in_root(prog, site.stmt),
        // not in the map at all: a gesture just made it, so it is as root as anything gets
        None => true,
    }
}
