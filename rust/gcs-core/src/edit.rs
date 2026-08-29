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
    // one made is `SourceMap::made_by`, in the order `build` made it — the declaration's own
    // entity first, then the children it minted — so neither the entity index nor the
    // find-by-kind that re-derived the parent is needed.
    let mut edits = Vec::new();
    for st in &prog.root().body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        // `hint at t` names a *place*, and has no coordinates to write
        if d.seed_at.is_some() {
            continue;
        }
        // a declaration that could not be built made nothing, and has no pose to record
        let Some(Made::Ent(parent)) = e.map.made_by(st.id).first().copied() else { continue };
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
            Some((_, tail)) if at.is_empty() => edits.push(Splice { at, with: tail }),
            // The clause has a home of its own, and the *list* belongs to the name: written at
            // the clause's position it would land past whatever trailer stands between them,
            // where an argument list is not a thing a declaration can say.
            Some((args, _)) => {
                let end = d.name.span.hi as usize;
                edits.push(Splice { at: Span::new(end, end), with: args });
                if !hint.is_empty() {
                    edits.push(Splice { at, with: hint });
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
            StmtKind::Decl(d) => Some(d.name.text.clone()),
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
pub fn add_rectangle(prog: &Program, w: f64, h: f64) -> Edit {
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
            StmtKind::Decl(d) => Some(d.name.text.as_str()),
            StmtKind::Instance(i) => Some(i.name.text.as_str()),
            StmtKind::Param(p) => Some(p.name.text.as_str()),
            StmtKind::Port(p) => Some(p.name.text.as_str()),
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
        name: syntax::Name::new(name.clone()),
        children: Vec::new(),
        seed: vec![x, y],
        seed_text: vec![None, None],
        seed_spans: Vec::new(),
        hint_span: None,
        knots: None,
        def: None,
        values: Vec::new(),
        domain: None,
        class: Default::default(),
        class_span: Span::default(),
        seed_at: None,
    };
    append(prog, StmtKind::Decl(d), vec![name])
}

/// An entity built from names that already exist — a line from two points, a circle from a centre.
pub fn add_entity(prog: &Program, kind: EntKind, args: &[String], seed: &[f64]) -> Edit {
    if kind == EntKind::Point || kind == EntKind::Curve {
        return Edit::none(prog, Some(format!("a {} is not built this way", kind.as_str())));
    }
    let name = mint(prog, kind);
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
        kind,
        name: syntax::Name::new(name.clone()),
        children,
        seed: (0..n_scalar).map(|i| seed.get(i).copied().unwrap_or(0.0)).collect(),
        seed_text: vec![None; n_scalar],
        seed_spans: Vec::new(),
        hint_span: None,
        knots: None,
        def: None,
        values: Vec::new(),
        domain: None,
        class: Default::default(),
        class_span: Span::default(),
        seed_at: None,
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
pub fn remove(e: &Elaborated, prog: &Program, ents: &[EntRef], cons: &[u32]) -> Edit {
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
            names.insert(d.name.text.clone());
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
                names.insert(d.name.text.clone());   // and now whatever names *it* goes too
            }
            grew = true;
        }
        if !grew {
            break;
        }
    }
    if doomed.is_empty() {
        return Edit::none(prog, None);
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
    let mut edits: Vec<Splice> = Vec::new();
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
        StmtKind::Gauge(g) => match g {
            syntax::Gauge::Ground(r) | syntax::Gauge::Fix(r) => look(r),
        },
        StmtKind::Orient(o) => {
            for r in &o.pts {
                look(r);
            }
        }
        _ => {}
    }
    hit
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
    let mut made: Vec<crate::program::Made> = Vec::new();

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
    // …a new entity names its children, which may be points the drawing already had…
    for &r in minted.keys() {
        needed.extend(sk.children(r));
    }
    // …and a gauge names what it holds
    needed.extend(held.iter().map(|(r, _)| *r));
    for r in &needed {
        if minted.contains_key(r) || renamed.contains_key(r) {
            continue; // new (its statement carries the name), or its statement already named
        }
        if e.map.written_name(*r).is_some() {
            continue; // the source already calls it something
        }
        let Some(site) = e.map.of_entity.get(r) else { continue };
        if !site.path.0.is_empty() || !in_root(prog, site.stmt) {
            // no statement of the root's to put a name on — for a declaration that is
            // *anonymous*, refuse now with the cause, rather than writing the hidden key
            // and refusing later with none.  (An entity a block prefix names keeps its
            // long-standing path; its declaration has a name already.)
            if e.map.names.get(r).and_then(|v| v.first()).is_some_and(|n| syntax::nameless(n)) {
                return Edit::none(
                    prog,
                    Some(
                        "that is anonymous inside a component or a block, so no statement \
                         can name it; give its declaration a name there first"
                            .into(),
                    ),
                );
            }
            continue;
        }
        let Some(d) = decl_of(prog, site) else { continue };
        if !syntax::hidden(&d.name.text) {
            continue; // named since the map was made
        }
        // one name per statement — and *every* entity the statement made follows it at
        // once, because the map's hidden keys go stale the moment the name lands, and a
        // later gesture in this same elaboration must never meet one first
        let name = next_name(&mut taken, d.kind);
        named.push(Splice { at: d.name.span, with: format!(" {name}") });
        // a name this edit gave a declaration, which is what `Edit::names` reports — a caller
        // reads it to refer to what it just made, and a name minted into a declaration that
        // was already there is as much this edit's doing as one on a statement it appended
        names.push(name.clone());
        for m in e.map.made_by(site.stmt) {
            let crate::program::Made::Ent(k) = *m else { continue };
            let Some(h) = e.map.names.get(&k).and_then(|v| v.first()) else { continue };
            if !syntax::hidden(h) {
                continue;
            }
            // a child's key is its parent's plus the dotted path (`#a41.p2`), and the path
            // is the *stable* half: the parent's key is an offset an earlier retext may
            // have moved, so the path is read off the map's own name and never compared
            // against the declaration's
            let text = match h.find('.') {
                Some(dot) if h.starts_with('#') => format!("{name}{}", &h[dot..]),
                _ => name.clone(),
            };
            renamed.insert(k, text);
        }
    }

    // the elaboration's own name for an entity it made — what a new statement has to refer to.
    // Never a hidden key while a written name exists: a mint in an earlier reconcile of this
    // same elaboration leaves both in the map, favoured order or not.
    let name_of = |r: EntRef| -> String {
        minted
            .get(&r)
            .cloned()
            .or_else(|| renamed.get(&r).cloned())
            .or_else(|| e.map.written_name(r).cloned())
            .or_else(|| e.map.names.get(&r).and_then(|v| v.first()).cloned())
            .unwrap_or_else(|| syntax::entity_name(r))
    };

    for r in sk.primitives() {
        let Some(name) = minted.get(&r) else { continue };
        let mut d = crate::program::lift_decl(sk, r);
        d.name = syntax::Name::new(name.clone());
        rename_children(&mut d, sk, r, &name_of);
        names.push(name.clone());
        made.push(crate::program::Made::Ent(r));
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
        made.push(crate::program::Made::Con(c.id));
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
        let Some(d) = decl_of(prog, site) else { continue };
        let now = sk.class_of(*r);
        if now == d.class {
            continue;
        }
        // the whole clause, replaced where it stands — or written at the point the parser
        // recorded for it, which is where it would have gone
        let at = d.class_span;
        if now.is_empty() {
            // gone: the clause takes the space in front of it with it
            let lo = back_over_spaces(prog.text(), at.lo as usize);
            flags.push(Splice { at: Span::new(lo, at.hi as usize), with: String::new() });
        } else {
            let with = format!(" class {}", now.0.join(" "));
            flags.push(Splice {
                at,
                with: if at.is_empty() { with } else { with.trim_start().to_string() },
            });
        }
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
            Some((t, r)) => {
                let with = format!("at ({}, {})", num(t), num(r));
                let at = rel.place_span;
                if at != Span::default() {
                    let with = if at.is_empty() { format!(" {with}") } else { with };
                    flags.push(Splice { at, with });
                }
            }
            // back where the layout would put it: the clause goes, and the space before it
            None if !rel.place_span.is_empty() => {
                let lo = back_over_spaces(prog.text(), rel.place_span.lo as usize);
                flags.push(Splice {
                    at: Span::new(lo, rel.place_span.hi as usize),
                    with: String::new(),
                });
            }
            None => {}
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
            StmtKind::Gauge(g) => Some(gauge_key(g)),
            _ => None,
        })
        .collect();
    for st in prog.root().body.iter() {
        let StmtKind::Gauge(g) = &st.kind else { continue };
        let k = gauge_key(g);
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
        adds.push(StmtKind::Gauge(match &k.1 {
            None => crate::syntax::Gauge::Ground(syntax::Ref::new(k.0.clone())),
            Some(f) => crate::syntax::Gauge::Fix(syntax::Ref::field(k.0.clone(), f)),
        }));
        made.push(crate::program::Made::Gauge);
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
    // an entity just named in place resolves by that name from here on — the map learns it now,
    // *first*, so a later reconcile in this same elaboration reads the written name and not the
    // hidden key it elaborated under
    for (r, n) in &renamed {
        e.map.favor(n, *r);
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

/// What a gauge statement holds: a name, and a field when it is one scalar rather than a point.
fn gauge_key(g: &crate::syntax::Gauge) -> (String, Option<String>) {
    let r = match g {
        crate::syntax::Gauge::Ground(r) | crate::syntax::Gauge::Fix(r) => r,
    };
    let field = match r.path.first() {
        Some(crate::syntax::Seg::Field(n)) => Some(n.text.clone()),
        _ => None,
    };
    (r.root.text.clone(), field)
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
