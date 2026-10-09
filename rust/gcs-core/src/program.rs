//! Elaborate Solvent into a sketch, preserving partial geometry alongside diagnostics.
//!
//! Build in registry order through model constructors, matching JSON parameter order.
//! Evaluate expressions once after all declarations and constraints are built.

mod curves;
mod diagnostics;
mod entities;
mod lift;
mod planes;
mod views;
mod reading;
mod relations;
mod resolve;
mod solids;
mod surfaces;
mod motions;
mod envelopes;
mod generated;
mod patches;
mod seams;
mod vertices;
mod edges;
mod spatial_faces;
mod rings;
mod source_map;
mod variational;
mod words;

pub use diagnostics::{Code, Diag, Severity};

/// Geometry diagnostics belong after solving: a poor hint is not an invalid final profile.
pub fn solid_diagnostics(sk: &crate::model::Sketch, map: &SourceMap) -> Vec<Diag> {
    let mut diags: Vec<_> = sk.solids.iter().enumerate().filter(|(_, s)| !matches!(s.def, crate::model::SolidDef::Body { .. }))
        .filter_map(|(i, _)| {
            let message = crate::solid::validate(sk, i).err()?;
            let site = map.site_of(crate::model::EntRef::solid(i));
            Some(Diag { code: Code::E080, span: site.map(|s| s.span).unwrap_or_default(),
                stmt: site.map(|s| s.stmt), message })
        }).collect();
    diags.extend(crate::solid::bearing_errors(sk).into_iter().map(|(b, message)| Diag {
        code: Code::E082, span: b.span, stmt: Some(crate::syntax::StmtId(b.stmt)), message,
    }));
    // a prism's side is read in the drawing, where its envelope stands for it (`extruded_envelopes`)
    let prism = |i: usize| matches!(sk.solids[sk.surfaces[i].solid as usize].def, crate::model::SolidDef::Prism { .. });
    for i in (0..sk.surfaces.len()).filter(|&i| !prism(i)) {
        if let Err(message) = crate::solid::RevolvedSurface::named(sk,i) {
            let site = map.site_of(crate::model::EntRef::new(crate::model::EntKind::Surface,i));
            diags.push(Diag {code:Code::E080,span:site.map(|s| s.span).unwrap_or_default(),
                stmt:site.map(|s| s.stmt),message});
        }
    }
    for i in 0..sk.motions.len() {
        if let Err(message) = crate::motion::evaluate(sk,i,0.) {
            let site = map.site_of(crate::model::EntRef::new(crate::model::EntKind::Motion,i));
            diags.push(Diag {code:Code::E080,span:site.map(|s| s.span).unwrap_or_default(),
                stmt:site.map(|s| s.stmt),message});
        }
    }
    diags.extend(views::degenerate(sk, map));
    diags.extend(views::coplanar(sk, map));
    for i in 0..sk.envelopes.len() {
        if let Err(message) = crate::envelope::GeneratedEnvelope::named(sk,i) {
            let site = map.site_of(crate::model::EntRef::new(crate::model::EntKind::Envelope,i));
            diags.push(Diag {code:Code::E080,span:site.map(|s| s.span).unwrap_or_default(),
                stmt:site.map(|s| s.stmt),message});
        }
    }
    diags
}
pub use lift::{dumps, to_program};
pub use source_map::{public_path, Elaborated, InstPath, Made, Site, SourceMap};

use crate::expr;
use crate::model::{EntKind, EntRef, Sketch};
use crate::syntax::{Name, Program, Span, Stmt, StmtId, StmtKind};
pub(crate) use entities::child_names;
use entities::{build, crosses_views, settle_deferred, Deferred};
pub(crate) use lift::{holds, lift_decl, lift_gauge, lift_relation, point_len};
use planes::memberships;
pub(crate) use planes::{plane_of_entity, plane_of_entity_by};
use relations::{constrain, repeated, Gauges};
use resolve::Resolver;
use solids::{solid_claims, solids};
use std::collections::{BTreeMap, BTreeSet};

/// Warn when a declaration shadows a built-in name; expressions still read the
/// built-in. Check nested bodies and formals as well as root declarations.
fn shadowing(p: &Program, diags: &mut Vec<Diag>) {
    fn say(name: &Name, what: &str, stmt: Option<StmtId>, diags: &mut Vec<Diag>) {
        let Some(kind) = expr::builtin(&name.text) else { return };
        diags.push(Diag {
            code: Code::W112,
            span: name.span,
            stmt,
            message: format!(
                "`{}` is {kind}, and {what} of that name does not shadow it — an expression \
                 reads the built-in wherever a number is worked out (§3.3), so this name is \
                 read two ways.  Rename it.",
                name.text
            ),
        });
    }
    fn body(stmts: &[Stmt], diags: &mut Vec<Diag>) {
        for st in stmts {
            match &st.kind {
                StmtKind::Param(d) => say(&d.name, "a `param`", Some(st.id), diags),
                StmtKind::Group(d) => say(&d.name, "a `group`", Some(st.id), diags),
                StmtKind::Set(d) => say(&d.name, "a set", Some(st.id), diags),
                StmtKind::Block(b) => {
                    if let Some(i) = &b.binder {
                        say(i, "a block's index", Some(st.id), diags);
                    }
                    if let Some(o) = &b.over {
                        say(&o.var, "a block's edge", Some(st.id), diags);
                    }
                    body(&b.body, diags);
                }
                _ => {}
            }
        }
    }
    // the root stands among the components, and a module's components with it; what a module's
    // own body adds is its exported top-level values (§6.3)
    let mut said: Vec<Diag> = Vec::new();
    for c in &p.components {
        for f in &c.formals {
            say(&f.name, "a formal", None, &mut said);
        }
        body(&c.body, &mut said);
    }
    for m in &p.modules {
        body(&m.root.body, &mut said);
    }
    // in the order a reader meets them: the components come out of the program in link order,
    // and a document is read down the page
    said.sort_by_key(|d| d.span.lo);
    diags.append(&mut said);
}

/// **A component defined twice in the document is E071**, as one defined twice in a module is
/// (`modules::link`): the first stands, so every call still reads one definition, and the second
/// is said where it is written.  Asked of the text, like `shadowing`.
fn defined_twice(p: &Program, diags: &mut Vec<Diag>) {
    let mut first: BTreeMap<&str, Span> = BTreeMap::new();
    for c in p.components.iter().filter(|c| c.module.is_none()) {
        let Some(n) = &c.name else { continue };
        match first.get(n.text.as_str()) {
            Some(&was) => diags.push(defined_again(p, n, was)),
            None => {
                first.insert(&n.text, n.span);
            }
        }
    }
}

/// E071 at `n`, defined in its file already at `was`.
fn defined_again(p: &Program, n: &crate::syntax::Name, was: Span) -> Diag {
    let line = p.line_col(was.lo as usize).0;
    let message = format!("`{}` is defined twice; the first is at line {line}", n.text);
    Diag { code: Code::E071, span: n.span, stmt: None, message }
}

pub fn elaborate(p: &Program) -> Elaborated {
    // a set the flattener drew as an element that is none here is walked again as a set, until
    // every element drawn is one (`lowering`): whether one is can rest on what another set's use
    // draws, so it is judged after the expansion, and a pass ends at the judgment, before any
    // constraint is stated.  Each pass refuses at least one more set, and a refusal is final
    let mut refused = BTreeSet::new();
    loop {
        match elaborate_in(p, &refused) {
            Ok(e) => return e,
            Err(more) => refused.extend(more),
        }
    }
}

/// `elaborate`, with the sets in `refused` walked as sets — or `Err` naming the sets drawn as
/// elements that are none, for another pass.
fn elaborate_in(p: &Program, refused: &BTreeSet<String>) -> Result<Elaborated, BTreeSet<String>> {
    let mut diags: Vec<Diag> = Vec::new();
    let mut map = SourceMap::default();
    let mut sk = Sketch::new();

    // -- phase 0: the document's unit, before *anything* reads a number.
    //
    // A literal with a unit on it converts to this one, and a document that names none is in
    // drawing units (spec §3.3).  It is read from the **unexpanded root** because a unit is
    // document preamble and a component is reusable: a `unit` line anywhere else is refused
    // below rather than quietly doing nothing.  It comes before the curve families, because a
    // family's body is a number-bearing text like any other.
    let mut said = false;
    for st in &p.root().body {
        let crate::syntax::StmtKind::Unit(n) = &st.kind else { continue };
        if said {
            diags.push(Diag {
                code: Code::E040,
                span: n.span,
                stmt: Some(st.id),
                message: "the document's unit is already stated above — one document, one \
                          unit"
                    .to_string(),
            });
            continue;
        }
        said = true;
        match crate::units::Units::with_length(&n.text) {
            Ok(u) => sk.units = u,
            Err(message) => {
                diags.push(Diag { code: Code::E040, span: n.span, stmt: Some(st.id), message })
            }
        }
    }

    // -- a name a document declares over a built-in is said, before anything reads either.
    shadowing(p, &mut diags);
    defined_twice(p, &mut diags);
    // -- and every relation word and import, as written (§9.9, §14.4)
    words::check(p, &mut diags);

    // -- phase 1: names, in one pre-pass.  Indices come from declaration order within a kind,
    // which is `primitives()` order, which is the order phase 2 builds in.
    let mut expansion = crate::flatten::expand_with(p, sk.units, refused);
    map.private_names = expansion.private_names.clone();
    // the unknowns the source declared — a solved fold and the expression graph read their
    // seeds and dimensions as they are built
    sk.declared = std::mem::take(&mut expansion.unknowns);
    diags.extend(expansion.diagnostics.iter().cloned());
    let mut res = Resolver::default();
    let mut count: BTreeMap<EntKind, u32> = BTreeMap::new();
    // a redeclaration is skipped rather than merged, and *which* statement was skipped is
    // remembered here: inferring it later from the name would find the one that won
    let mut skip: BTreeSet<StmtId> = BTreeSet::new();
    use crate::ir::{Operation as StmtKind, Statement};
    let body: Vec<&Statement> = expansion.flat.iter().collect();
    // what states something: every statement but a `ring`'s turned copy's, whose relations,
    // holds and claims are its representative's turned (§12.4) — its declarations are built
    let stating: Vec<&Statement> = body.iter().copied().filter(|st| !st.turned).collect();
    // the style sheet, before anything is built: it says nothing about what the drawing is, so
    // it is collected once and never consulted again by anything here (spec §14)
    let mut sheet = crate::style::Sheet::new();
    for st in &body {
        // a `unit` inside a component or a block is expanded into the flat list and read by
        // nobody — phase 0 takes the root's alone, a component being reusable and a unit being
        // the document's.  Silence there is exactly what §13.1 forbids, so it is a diagnostic.
        if let StmtKind::Unit(n) = &st.kind {
            if !p.root().body.iter().any(|r| r.id == st.id) {
                diags.push(Diag {
                    code: Code::E040,
                    span: n.span,
                    stmt: Some(st.id),
                    message: "a document's unit is stated once, at the top — not inside a \
                              component or a block"
                        .to_string(),
                });
            }
        }
        if let StmtKind::Style(r) = &st.kind {
            // a class stated twice cascades, later over earlier — the same rule that decides a
            // conflicting property between two classes on one declaration
            sheet.entry(r.name.text.clone()).or_default().over(&r.style);
        }
    }
    sk.set_sheet(sheet);
    for st in &body {
        let (name, kind) = match &st.kind {
            StmtKind::Decl(d) => (&d.name, d.kind),
            StmtKind::Chain(c) => (&c.name, EntKind::Face),
            _ => continue,
        };
        // the *key*, which is resolution's question: an anonymous key is its own offset and a
        // copy's carries its prefix, so only a name the source wrote twice can actually collide
        let key = &name.key().text;
        if let Some(&was) = res.declared_at.get(key) {
            // …but the message shows what the source calls it, and spells the kind where it
            // calls it nothing, so a key cannot leak here even if that argument ever breaks
            let who = name.shown().map_or_else(
                || crate::syntax::decl_head(kind, name),
                |n| n.text.clone(),
            );
            diags.push(Diag {
                code: Code::E001,
                span: name.span(),
                stmt: Some(st.id),
                message: format!(
                    "`{who}` is declared twice; the first is at line {}",
                    p.line_col(was.lo as usize).0
                ),
            });
            skip.insert(st.id);
            continue; // the second, so every later reference still resolves to the first
        }
        res.declared_at.insert(key.clone(), name.span());
        if let StmtKind::Chain(c) = &st.kind {
            res.chains.insert(key.clone(), c.clone());
            if !c.closed {
                continue; // an open traversal binds a name but allocates no face
            }
        }
        let n = count.entry(kind).or_insert(0);
        // Spatial faces depend on edges of generated surfaces, after all planar
        // profiles and primitive solids. Their indices are assigned in that phase.
        let late_face = matches!(&st.kind, StmtKind::Decl(d)
            if d.kind == EntKind::Face && d.children.get(2).is_some_and(|g| !g.is_empty()));
        res.of.insert(key.clone(), EntRef::new(kind, if late_face { u32::MAX as usize } else { *n as usize }));
        if let StmtKind::Decl(d) = &st.kind {
            res.kids.insert(
                key.clone(),
                d.children.iter().map(|g| match g.first() {
                    Some(crate::ir::Kid::Ref(r)) if g.len() == 1 => Some(r.clone()),
                    _ => None,
                }).collect(),
            );
        }
        if !late_face { *n += 1; }
        // a plane's origin is a point minted with the points, in statement order
        if kind == EntKind::Plane {
            let n = count.entry(EntKind::Point).or_insert(0);
            res.origins.insert(key.clone(), *n as usize);
            *n += 1;
        }
    }

    // -- phase 2: geometry, per kind in `primitives()` order.  The same walk `io::from_json`
    // makes, through the same constructors, so the two produce the same parameter vector.
    let mut built: BTreeMap<EntRef, bool> = BTreeMap::new();
    // seeds that read geometry, settled once every declaration has a seed of its own
    let mut deferred: Vec<Deferred> = Vec::new();
    // `primitives()` order, and **curves last**: a curve is written over other entities, so
    // every kind it may name has to exist before it does.  The same reason `io::graft` grafts
    // them last.
    for kind in [
        EntKind::Point,
        EntKind::Line,
        EntKind::Circle,
        EntKind::Arc,
        EntKind::Spline,
        // a plane is built over its axes
        EntKind::Axis,
        EntKind::Plane,
        EntKind::Curve,
    ] {
        for st in &body {
            let StmtKind::Decl(d) = &st.kind else { continue };
            // a plane's origin, minted with the points in statement order (`Resolver::origins`)
            if kind == EntKind::Point && d.kind == EntKind::Plane && !skip.contains(&st.id) {
                let key = &d.name.key().text;
                let o = sk.point(0.0, 0.0, true, &format!("{key}.origin"));
                debug_assert_eq!(res.origins.get(key), Some(&o));
                // bound and recorded with its plane, after it (`build_plane`), so the plane is
                // the first thing its statement made
                built.insert(EntRef::point(o), true);
                continue;
            }
            if d.kind != kind || skip.contains(&st.id) {
                continue;
            }
            let mut anon: Vec<(String, EntRef)> = Vec::new();
            match build(
                &mut sk,
                &res,
                d,
                st,
                &mut diags,
                &mut anon,
                &mut deferred,
                p,
                &expansion.instances,
            ) {
                Some(e) => {
                    built.insert(e, true);
                    map.bind(&d.name.key().text, e, d.name.named());
                    map.record(st, Made::Ent(e));
                    // an anonymous child's name *is* its dotted path, so it is bound like any
                    // other: that is what lets a dimension name it, a selection survive a
                    // re-elaboration, and a drag of it find the slot it came from.  Whether it
                    // is a name anyone may *say* is the declaration's answer, not its own —
                    // `l.p1` under a named line, nothing under an anonymous one
                    for (name, k) in anon {
                        built.insert(k, true);
                        map.bind(&name, k, d.name.named());
                        map.record(st, Made::Ent(k));
                    }
                }
                None => {
                    // a declaration that could not be built leaves its name unbound, so every
                    // reference to it is reported where the reference is — and made nothing,
                    // so every later entity of its kind sits one index below where phase 1
                    // put it: the resolver is shifted with them, or a reference to a later arc
                    // would read the one after it (or past the end of the list)
                    if let Some(gone) = res.of.remove(&d.name.key().text) {
                        for e in res.of.values_mut() {
                            if e.kind == gone.kind && e.idx > gone.idx {
                                e.idx -= 1;
                            }
                        }
                        if gone.kind == EntKind::Point {
                            for o in res.origins.values_mut().filter(|o| **o > gone.i()) {
                                *o -= 1;
                            }
                        }
                    }
                }
            }
        }
    }

    // memberships, once every kind is built and before anything reads one: `point a in top`
    // names a plane built after the point, and `project` infers its planes from these — and then
    // every point no `in` reached stands in space (`places`), and a plane written over a drawn
    // line has its hidden axis held along it
    memberships(&mut sk, &res, &map, &body, &skip, &mut diags);
    // and `q coincident P` of a point in space is its membership of `P`, said another way
    let drawn = planes::incidences(&mut sk, &res, &mut map, &stating, &deferred, &expansion.rings);
    // and a set drawn as an element is one only where what it reads says so (`lowering`)
    let ent = |r: &crate::syntax::Ref| {
        res.lookup(r).and_then(|e| resolve::follow(&sk, e, &r.path).ok())
    };
    let more = crate::lowering::refused(&sk, &expansion.lowered, ent, |k| res.of.get(k).copied());
    if !more.is_empty() {
        return Err(more);
    }
    entities::places(&mut sk, &deferred, &mut diags);
    // the numbers `fix` holds, once every point has its place and before anything reads one: a
    // held number is its own seed, so nothing that holds one needs a `hint` saying it again — an
    // axis along a line, a motion, a place reading a held point all read where it is held, and
    // a seed never writes a held number (`settle_deferred`)
    // each tangency's derivative, by the key the flattener gave its use (§6.21)
    let mut duals = BTreeMap::new();
    // every gauge gathered before any is applied, so neither statement order nor a second hold
    // decides what is held (#113)
    let mut gauges = Gauges::default();
    for st in &stating {
        let StmtKind::Relation(r) = &st.kind else { continue };
        if relations::is_fix(r) {
            constrain(&mut sk, &res, r, st, p, &map, &mut duals, &mut gauges, &mut diags);
        }
    }
    gauges.hold_all(&mut sk, &mut diags);
    // a plane whose axes are held stands where they meet (#84), and one held elsewhere takes
    // its free axes with it
    diags.extend(views::origins_on_axes(&mut sk, &map));
    views::stand_axes(&mut sk);
    entities::drawn_in_planes(&sk, &map, &mut diags);
    entities::axes_along(&mut sk, &deferred);
    // a ring's copies, turns of its representative: after the holds, which a copy may not have
    rings::rings(&mut sk, &res, &map, &expansion.rings, &mut diags);

    // motions, once every line and point they are written over is built — and before the
    // profiles a planar motion generates, which are curves of the drawing a contact may name
    // (§6.15.1).  A motion is evaluated after the solve wherever it is read in space; building it
    // here only resolves what it is written over.
    motions::motions(&mut sk, &mut res, &mut map, &body, &skip, &mut diags);
    let planar = generated::planar_envelopes(&mut sk, &mut res, &mut map, &body, &skip, &mut diags);

    // seeds named by geometry, once every entity has a seed to be read: in statement order, so
    // a seed that reads a seed read from a third is settled after both (§6.4)
    let first = diags.len();
    settle_deferred(&mut sk, &res, &deferred, &mut diags);
    let mut settled = first..diags.len();

    // a prism's side generating under a motion that keeps its view stands for a surface the
    // solve can hold a point to: built with the drawing, once the memberships say the view
    let extruded = generated::extruded_envelopes(&mut sk, &mut res, &mut map, &body, &skip, &mut diags);

    // -- phase 3: constraints, in statement order
    let mut arrays = BTreeSet::new();
    for (i, st) in stating.iter().enumerate() {
        // an energy's terms are constraints of their own, stationary together (#121)
        if let StmtKind::Minimize(m) = &st.kind {
            variational::state(&mut sk, &res, m, st, &mut map, &mut diags);
            continue;
        }
        let StmtKind::Relation(r) = &st.kind else { continue };
        if relations::is_fix(r) || drawn.contains(&i) {
            continue;
        }
        let made = constrain(&mut sk, &res, r, st, p, &map, &mut duals, &mut gauges, &mut diags);
        if let Some(id) = made {
            map.record(st, Made::Con(id));
            if let Some(place) = r.place {
                sk.placements.insert(id, place);
            }
            repeated(&mut sk, id, st, &mut arrays);
        }
    }

    rings::contacts(&sk, &map, &mut diags);
    // a tangency at a point gauges its directions where the point stands (§6.21)
    sk.choose_charts();

    // where the planes stand, from the statements that say so, before the drawing in them is
    // read through them: a place drawn in another plane is read in space, through that plane's
    // pose, so the seeds are settled again, in statement order, once the planes are placed.
    // Only then — a document whose places are all in their own planes settles once.
    // A plane placed over seeds read through another plane is read through in turn, so the two
    // alternate until the seeds stand still — a few rounds, one per plane a chain of them crosses.
    views::place(&mut sk);
    if crosses_views(&sk, &res, &deferred) {
        for _ in 0..4 {
            let before: Vec<f64> = sk.params.iter().map(|p| p.value).collect();
            // each reading's findings stand where the first's did
            let mut again = Vec::new();
            settle_deferred(&mut sk, &res, &deferred, &mut again);
            let found = settled.start..settled.start + again.len();
            diags.splice(settled, again);
            settled = found;
            // and the planes placed again over the points the seeds moved
            views::place(&mut sk);
            let moved = sk.params.iter().zip(&before).any(|(p, &b)| (p.value - b).abs() > 1e-12 * (1.0 + b.abs()));
            if !moved {
                break;
            }
        }
    }

    // a ring's copies where their representatives' seeds put them, whatever a seed said of a copy
    sk.settle_turns();

    // -- phase 3b: faces, then solids (§6.8, §6.9).  **After every other kind and after the
    // constraints**, because a face is written over edges the drawing already has and a solid
    // over faces and other solids; and *evaluated* rather than solved, because nothing about
    // either is an unknown.  This is the stratification as a phase: everything above it is the
    // drawing, everything below reads what the drawing came to.
    // (Motions were built with the primitives: solids retain their indices.)
    solids(&mut sk, &mut res, &mut map, &body, &skip, &mut diags);
    surfaces::surfaces(&mut sk, &mut res, &mut map, &body, &skip, &mut diags);
    let spatial: BTreeSet<StmtId> = skip.iter().chain(&planar).chain(&extruded).copied().collect();
    envelopes::envelopes(&mut sk, &mut res, &mut map, &body, &spatial, &mut diags);
    patches::patches(&mut sk, &mut res, &mut map, &body, &skip, &mut diags);
    seams::seams(&mut sk, &mut res, &mut map, &body, &skip, &mut diags);
    vertices::vertices(&mut sk, &mut res, &mut map, &body, &skip, &mut diags);
    edges::edges(&mut sk, &mut res, &mut map, &body, &skip, &mut diags);
    spatial_faces::faces(&mut sk, &mut res, &mut map, &body, &skip, &mut diags);

    // -- phase 4: every expression against the whole document, once (per-statement evaluation would
    // be quadratic in the expression count).  The claims about solids read the free variables
    // `evaluate` allocates, so they come after it — the one pass of the elaboration that is *below*
    // the expressions
    let post_expr = expr::evaluate(&mut sk);
    // every free curve's energy, once every row it may be held by is stated — a length an
    // unknown sets included (#121, #144)
    variational::settle(&mut sk, &map, &mut diags);
    solid_claims(&mut sk, &res, &mut map, &stating, &skip, &mut diags);
    for item in post_expr {
        let span = map.site_of_constraint(item.id).map(|s| s.span).unwrap_or_default();
        let stmt = map.site_of_constraint(item.id).map(|s| s.stmt);
        if let Some(err) = &item.error {
            // what the fault *is* decides what is said (#43.11): a number that is not what its
            // slot takes is the E103 every `param` already gets — §3.3 names `distance(45deg)`
            // as an error — and a claim binding a free name is the E040 §9.7 promises;
            // only an expression that would not compute is a warning, since the last number
            // stands and the drawing goes on
            let (code, tail) = match err.fault {
                expr::Fault::Dimension => (Code::E103, ""),
                expr::Fault::ClaimFree => (Code::E040, ""),
                expr::Fault::Uncomputable => (Code::W110, " — the last number stands"),
                expr::Fault::Measure => (Code::E107, ""),
                expr::Fault::Place => (Code::E040, ""),
            };
            diags.push(Diag { code, span, stmt, message: format!("`{}`: {err}{tail}", item.text) });
        }
    }

    // a curve written over a number its drawn instance left unknown reads that unknown as a
    // column, and only the expression graph allocates one — when a dimension of the drawing
    // reads it.  One nothing reads would leave the curve a column short
    for (i, cv) in sk.curves.iter().enumerate() {
        for n in cv.unknowns.iter().filter(|n| !sk.free_vars.contains_key(*n)) {
            let site = map.site_of(EntRef::new(EntKind::Curve, i));
            diags.push(Diag {
                code: Code::E103,
                span: site.map(|s| s.span).unwrap_or_default(),
                stmt: site.map(|s| s.stmt),
                message: format!(
                    "`{n}` is left unknown, and nothing on the sheet reads it: give the \
                     instance a number for it, or state what it is"
                ),
            });
        }
    }

    // -- phase 5: a root choice under a key no triple of points spells, kept verbatim
    for st in &stating {
        if let StmtKind::Branch(b) = &st.kind {
            gauges.branch(b, st);
        }
    }
    gauges.choose_all(&mut sk, &mut diags);

    // Roles follow creation provenance, including unnamed children. Referenced geometry
    // belongs to its own declaration, even when a construction component borrows it.
    let roles: BTreeMap<_, _> = body.iter().filter_map(|st| {
        let roles = match &st.kind {
            StmtKind::Decl(d) => d.roles,
            StmtKind::Chain(c) => c.annotations.roles,
            _ => return None,
        };
        (roles != Default::default()).then_some(((st.id, st.path.clone()), roles))
    }).collect();
    for (&entity, site) in &map.of_entity {
        if let Some(&roles) = roles.get(&(site.stmt, site.path.0.clone())) {
            sk.roles.insert(entity, roles);
        }
    }
    // a measurement read before the solve is one refusal wherever it stood — a param, a seed, a
    // constraint's number, an extent — and every one of those paths says it in the same words
    // (`expr::measure_refusal`), so the code is given once, here, the way the flattener sorts
    // its plain errors by message
    for d in &mut diags {
        if d.message.contains(expr::MEASURE_MARK) {
            d.code = Code::E107;
        }
    }
    crate::modules::localize(p, &mut diags);
    Ok(Elaborated { sketch: sk, map, diags, program: p.clone(), taken: false })
}

/// An angle a declaration is bounded by (`from:`, `to:`), in degrees: written as an angle,
/// bound to a number and finite, each refused naming `what` — a surface's span, an envelope's
/// roll.  The one reader of such a bound.
fn bound_angle(sk: &Sketch, a: &crate::syntax::Arg, what: &str) -> Result<f64, String> {
    let crate::syntax::Arg::Dim { text, .. } = a else { return Err(format!("a {what} needs an angle")) };
    let value = crate::flatten::value_aff(text, &BTreeMap::new(), sk.units)?;
    value.dim.require(crate::units::Dim::ANGLE, what)?;
    let value = value.number().ok_or_else(|| format!("a {what} must be bound"))?;
    if !value.is_finite() {
        return Err(format!("a {what} must be finite"));
    }
    Ok(value)
}
