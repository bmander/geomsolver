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
use crate::program::{Elaborated, Site};
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
/// > A seed is writable iff it is written with `=` and not `==`, is a literal and not an
/// > expression, and is reached by exactly one instance path.
///
/// The first is lexical, which is what makes this a test rather than an analysis: `at (0, 0)` and
/// `r: 25` are hints a solve may move, and `== 80` states a number it may not.  The second keeps
/// `r: Rr` — a radius written in terms of a component's parameter — from being overwritten with
/// the number it happened to come to.  The third is why a point inside a `cycle` of thirty does
/// not write back at all: thirty instances share one statement, and there is no one pose to
/// record.
///
/// `Kind::Numeric`, always: a seed is not a statement, so nothing recompiles.
pub fn commit_seeds(e: &Elaborated, sk: &Sketch, prog: &Program) -> Edit {
    let mut edits = Vec::new();
    let mut seen: std::collections::BTreeMap<crate::syntax::StmtId, usize> =
        std::collections::BTreeMap::new();
    for site in e.map.of_entity.values() {
        *seen.entry(site.stmt).or_insert(0) += 1;
    }
    for (ent, site) in &e.map.of_entity {
        // reached more than once: several instances of one statement, and no single pose to write
        if seen.get(&site.stmt).copied().unwrap_or(0) > 1 || !site.path.0.is_empty() {
            continue;
        }
        // and the statement must be the root's own: one inside a component is written in the
        // component's terms, and a pose put there is a pose put on every instance of it
        if !in_root(prog, site.stmt) {
            continue;
        }
        let Some(d) = decl_of(prog, site) else { continue };
        let own = sk.own_params(*ent);
        for (i, p) in own.iter().enumerate() {
            let Some(span) = d.seed_spans.get(i).copied() else { continue };
            if span.is_empty() || d.seed_text.get(i).and_then(|t| t.as_ref()).is_some() {
                continue; // built rather than written, or written as an expression
            }
            let v = sk.params[*p as usize].value;
            let was = span.slice(prog.text());
            let now = num(v);
            if was != now {
                edits.push(Splice { at: span, with: now });
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
    let taken: std::collections::BTreeSet<&str> = prog
        .stmts()
        .filter_map(|s| match &s.kind {
            StmtKind::Decl(d) => Some(d.name.text.as_str()),
            _ => None,
        })
        .collect();
    let c = syntax::kind_initial(kind);
    (0..).map(|i| format!("{c}{i}")).find(|n| !taken.contains(n.as_str())).unwrap_or_default()
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

/// `point pN at (x, y)`
pub fn add_point(prog: &Program, x: f64, y: f64) -> Edit {
    let name = mint(prog, EntKind::Point);
    let d = Decl {
        kind: EntKind::Point,
        name: syntax::Name::new(name.clone()),
        children: Vec::new(),
        seed: vec![x, y],
        seed_text: vec![None, None],
        seed_spans: Vec::new(),
        knots: None,
        def: None,
        values: Vec::new(),
        domain: None,
        construction: false,
    };
    append(prog, StmtKind::Decl(d), vec![name])
}

/// An entity built from names that already exist — a line from two points, a circle from a centre.
pub fn add_entity(prog: &Program, kind: EntKind, args: &[String], seed: &[f64]) -> Edit {
    if kind == EntKind::Point || kind == EntKind::Curve {
        return Edit::none(prog, Some(format!("a {} is not built this way", kind.as_str())));
    }
    let name = mint(prog, kind);
    let mut children: Vec<Vec<syntax::Ref>> = Vec::new();
    let mut taken = 0usize;
    for (_, f) in kind.fields() {
        match f {
            crate::model::Field::Child => {
                children.push(
                    args.get(taken).map(|a| vec![syntax::Ref::new(a.clone())]).unwrap_or_default(),
                );
                taken += 1;
            }
            crate::model::Field::List => {
                children.push(args[taken.min(args.len())..].iter().map(|a| syntax::Ref::new(a.clone())).collect());
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
        knots: None,
        def: None,
        values: Vec::new(),
        domain: None,
        construction: false,
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
    let edits: Vec<Splice> = prog
        .root()
        .body
        .iter()
        .filter(|s| doomed.contains(&s.id))
        .map(|s| Splice { at: with_line(prog.text(), s.span), with: String::new() })
        .collect();
    Edit {
        text: splice(prog.text(), edits),
        kind: Kind::Structural,
        names: Vec::new(),
        refused: None,
    }
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
                for r in g {
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

/// A statement's span, grown to swallow the newline that ends it — so deleting one does not
/// leave a blank line where it stood.
fn with_line(text: &str, s: Span) -> Span {
    let b = text.as_bytes();
    let mut hi = s.hi as usize;
    while hi < b.len() && (b[hi] == b' ' || b[hi] == b'\t' || b[hi] == b'\r') {
        hi += 1;
    }
    if hi < b.len() && b[hi] == b'\n' {
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
    let Some(i) = rel.kind.spec().iter().position(|(n, _)| *n == attr) else {
        return Edit::none(prog, Some(format!("no argument `{attr}`")));
    };
    let Some(syntax::Arg::Dim { span, text: was }) = rel.args.get(i).and_then(|a| a.as_ref())
    else {
        return Edit::none(prog, Some("that argument is not a dimension".into()));
    };
    let plain = crate::expr::literal(text).is_some() && crate::expr::literal(was).is_some();
    Edit {
        text: splice(prog.text(), vec![Splice { at: *span, with: text.trim().to_string() }]),
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
    let mut taken: std::collections::BTreeSet<String> = prog
        .stmts()
        .filter_map(|s| match &s.kind {
            StmtKind::Decl(d) => Some(d.name.text.clone()),
            _ => None,
        })
        .collect();
    for r in sk.primitives() {
        if r.i() < high.get(&r.kind).copied().unwrap_or(0) {
            continue;
        }
        let c = syntax::kind_initial(r.kind);
        let name = (0..).map(|i| format!("{c}{i}")).find(|n| !taken.contains(n)).unwrap_or_default();
        taken.insert(name.clone());
        minted.insert(r, name);
    }
    // the elaboration's own name for an entity it made — what a new statement has to refer to
    let name_of = |r: EntRef| -> String {
        minted
            .get(&r)
            .cloned()
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

    /* flags and gauge.  A construction word and a `ground` are neither an entity nor a
     * constraint, so nothing above notices them: they are read off the sketch and compared
     * against what the source says, statement by statement. */
    let mut flags: Vec<Splice> = Vec::new();
    for (r, site) in e.map.of_entity.iter() {
        if !site.path.0.is_empty() || !in_root(prog, site.stmt) {
            continue;   // inside a component: one statement, many instances, no one flag
        }
        let Some(d) = decl_of(prog, site) else { continue };
        let now = crate::program::construction_of(sk, *r);
        if now == d.construction {
            continue;
        }
        let Some(st) = prog.stmt(site.stmt) else { continue };
        if now {
            let at = Span::new(st.span.hi as usize, st.span.hi as usize);
            flags.push(Splice { at, with: " construction".into() });
        } else if let Some(i) =
            prog.text()[st.span.lo as usize..st.span.hi as usize].rfind(" construction")
        {
            let lo = st.span.lo as usize + i;
            flags.push(Splice {
                at: Span::new(lo, lo + " construction".len()),
                with: String::new(),
            });
        }
    }
    // `ground(p)` and `fix(c.r)`: a statement per held parameter, added and taken away
    let held_now = gauges(sk, &name_of, &|r| root_declared(e, prog, r));
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

    if adds.is_empty() && doomed.is_empty() && flags.is_empty() {
        // nothing structural: the drawing only moved, and the seeds record where to
        let seeds = commit_seeds(e, sk, prog);
        if seeds.kind != Kind::None {
            e.retext(&seeds.text);
        }
        return seeds;
    }

    let mut edits: Vec<Splice> = prog
        .root()
        .body
        .iter()
        .filter(|s| doomed.contains(&s.id))
        .map(|s| Splice { at: with_line(prog.text(), s.span), with: String::new() })
        .collect();
    edits.extend(flags);
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
    let text = splice(prog.text(), edits);
    if !e.adopt(&text, &made) {
        return Edit::none(prog, Some("the drawing could not be written down".into()));
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
                *c = syntax::Ref::new(name_of(k));
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

/// Everything the sketch itself holds fixed, written the way a gauge statement writes it.
fn gauges(
    sk: &Sketch,
    name_of: &dyn Fn(EntRef) -> String,
    ours: &dyn Fn(EntRef) -> bool,
) -> std::collections::BTreeSet<(String, Option<String>)> {
    let mut out = std::collections::BTreeSet::new();
    for i in 0..sk.points.len() {
        if sk.point_fixed(i) && ours(EntRef::point(i)) {
            out.insert((name_of(EntRef::point(i)), None));
        }
    }
    for r in sk.primitives() {
        if r.kind == EntKind::Point || !ours(r) {
            continue;
        }
        let scalars: Vec<&str> = r
            .kind
            .fields()
            .iter()
            .filter(|(_, f)| *f == crate::model::Field::Scalar)
            .map(|(n, _)| *n)
            .collect();
        for (i, &pi) in sk.own_params(r).iter().enumerate() {
            if sk.params[pi as usize].fixed {
                let f = scalars.get(i).copied().unwrap_or("r");
                out.insert((name_of(r), Some(f.to_string())));
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
