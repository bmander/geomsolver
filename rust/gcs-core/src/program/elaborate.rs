//! One pass of the elaboration (`program::elaborate`), phase by phase over one context (`Elab`):
//! the document's unit, the checks of its text, the flattener's expansion, names, geometry,
//! memberships and holds, motions and generated profiles, seeds, constraints, the planes' places,
//! solids, expressions and root choices — each phase a function, in the order they run.

use super::entities::{self, build, crosses_views, settle_deferred, Deferred};
use super::gauges::{self, Gauges};
use super::regions::{self, Probes};
use super::relations::{self, constrain, repeated};
use super::resolve::{self, Resolver};
use super::solids::{solid_claims, solids};
use super::{
    defined_twice, envelopes, generated, motions, planes, rings, shadowing, surfaces,
    variational, views, words,
};
use super::{Again, Code, Diag, Elaborated, Made, SourceMap};
use crate::expr;
use crate::flatten::Expansion;
use crate::ir::{Operation as StmtKind, Statement};
use crate::model::{EntKind, EntRef, Sketch};
use crate::solid::faces::FacePath;
use crate::syntax::{Program, StmtId};
use std::collections::{BTreeMap, BTreeSet};
use std::ops::Range;

/// **What an elaboration pass writes**, carried through every phase and into each relation it
/// states (`constrain`, `apply_gauge`): the document read, the sketch it comes to, the names
/// resolved and the source map, the diagnostics, and what statements gather before it is applied
/// — each tangency's derivative by the key the flattener gave its use, every gauge (applied once
/// all are in, #113) and each region solid's terms by its probe.
pub(super) struct Elab<'p> {
    pub(super) doc: &'p Program,
    pub(super) sk: Sketch,
    pub(super) res: Resolver,
    pub(super) map: SourceMap,
    pub(super) diags: Vec<Diag>,
    pub(super) duals: BTreeMap<String, usize>,
    pub(super) gauges: Gauges,
    pub(super) probes: Probes,
}

/// The flattener's statements, as the phases read them: every one (`body`), those that state
/// something (`stating`: every statement but a `ring`'s turned copy's, whose relations, holds and
/// claims are its representative's turned, §12.4 — its declarations are built), and the
/// redeclarations skipped (`names`).
struct Flat<'a> {
    expansion: &'a Expansion,
    body: Vec<&'a Statement>,
    stating: Vec<&'a Statement>,
    skip: BTreeSet<StmtId>,
}

/// `elaborate`, with the sets in `refused` walked as sets — or `Err` naming the sets drawn as
/// elements that are none, for another pass.
/// `crossed`: points a bound carried across in a pass before, by key, and the numbers that pass
/// gave them, written over their seeds (`elaborate`).
pub(super) fn elaborate_in(
    p: &Program,
    refused: &BTreeSet<String>,
    crossed: &BTreeMap<String, Vec<f64>>,
    faces: &BTreeMap<String, Vec<FacePath>>,
) -> Result<Elaborated, Again> {
    let mut cx = Elab {
        doc: p,
        sk: Sketch::new(),
        res: Resolver::default(),
        map: SourceMap::default(),
        diags: Vec::new(),
        duals: BTreeMap::new(),
        gauges: Gauges::default(),
        probes: Probes::default(),
    };
    unit(&mut cx);
    check_text(&mut cx);
    let expansion = expand(&mut cx, refused, faces);
    let body: Vec<&Statement> = expansion.flat.iter().collect();
    let stating: Vec<&Statement> = body.iter().copied().filter(|st| !st.turned).collect();
    style_sheet(&mut cx, &body);
    let skip = names(&mut cx, &body);
    let flat = Flat { expansion: &expansion, body, stating, skip };
    let deferred = geometry(&mut cx, &flat);
    station_reads(&mut cx, &flat);
    let (drawn, kept_off) = memberships(&mut cx, &flat, &deferred)?;
    let held = holds(&mut cx, &flat, &deferred, crossed);
    let planar = motions(&mut cx, &flat);
    let settled = seeds(&mut cx, &deferred, &held);
    let extruded = prism_sides(&mut cx, &flat);
    let (generated, held_in_space) = held_envelopes(&mut cx, &flat, &planar, &extruded);
    let after_solids = constraints(&mut cx, &flat, &drawn, &kept_off);
    walked(&mut cx, &deferred, &held);
    twins(&mut cx, &flat);
    place_planes(&mut cx, &deferred, &held, settled);
    solids_and_surfaces(&mut cx, &flat, faces, &generated, &held_in_space)?;
    expressions(&mut cx, &flat);
    regions_read(&mut cx, &flat, &after_solids);
    Ok(finish(cx, &flat))
}

/// **The numbers of the geometry a face family's stations read** (#243 L1), once the geometry is
/// built and before any expression is worked out: each held as its scalar stands, so it is no
/// unknown of the drawing and every station reads it where the drawing put it.
fn station_reads(cx: &mut Elab<'_>, flat: &Flat<'_>) {
    use crate::model::{HELD_R, HELD_X, HELD_Y, HELD_Z};
    for (read, abs, rest) in &flat.expansion.reads {
        let found = cx.res.of.get(abs).copied().and_then(|e| {
            let (last, path) = rest.split_last()?;
            let path: Vec<crate::syntax::Seg> = path.iter()
                .map(|f| crate::syntax::Seg::Field(crate::syntax::Name::new(f.as_str())))
                .collect();
            let e = resolve::follow(&cx.sk, e, &path).ok()?;
            let field = match (e.kind, last.as_str()) {
                (EntKind::Point, "x") => HELD_X,
                (EntKind::Point, "y") => HELD_Y,
                (EntKind::Point, "z") => HELD_Z,
                (EntKind::Circle | EntKind::Arc, "r") => HELD_R,
                _ => return None,
            };
            Some((e, field, cx.sk.held_read(e, field)?))
        });
        let Some((e, field, value)) = found else {
            let at = cx.res.declared_at.get(abs).copied().unwrap_or_default();
            cx.diags.push(Diag { code: Code::E103, span: at, stmt: None, message: format!(
                "`{read}` is no number of the geometry a station can read: a point's `x`, `y` or \
                 `z`, or a circle's or an arc's `r`") });
            continue;
        };
        cx.sk.hold(read, value, crate::units::Dim::LENGTH);
        if let Some(h) = cx.sk.held.get_mut(read) {
            h.reads = Some((e, field));
        }
    }
}

/// **Phase 0: the document's unit**, before *anything* reads a number.
///
/// A literal with a unit on it converts to this one, and a document that names none is in
/// millimetres (spec §3.3.2, [0.93]).  It is read from the **unexpanded root** because a unit is
/// document preamble and a component is reusable: a `unit` line anywhere else is refused
/// (`style_sheet`) rather than quietly doing nothing.  It comes before the curve families,
/// because a family's body is a number-bearing text like any other.
fn unit(cx: &mut Elab<'_>) {
    let mut said = false;
    for st in &cx.doc.root().body {
        let crate::syntax::StmtKind::Unit(n) = &st.kind else { continue };
        if said {
            cx.diags.push(Diag::within(
                Code::E040,
                n.span,
                st.id,
                "the document's unit is already stated above — one document, one \
                 unit"
                    .to_string(),
            ));
            continue;
        }
        said = true;
        match crate::units::Units::with_length(&n.text) {
            Ok(u) => cx.sk.units = u,
            Err(message) => {
                cx.diags.push(Diag::within(Code::E040, n.span, st.id, message))
            }
        }
    }
}

/// What is asked of the text before anything is expanded: a name a document declares over a
/// built-in, before anything reads either; a component defined twice; every relation word and
/// import, as written (§9.9, §14.4); every interface and the types naming one (§8.1 [0.84]).
fn check_text(cx: &mut Elab<'_>) {
    shadowing(cx.doc, &mut cx.diags);
    defined_twice(cx.doc, &mut cx.diags);
    words::check(cx.doc, &mut cx.diags);
    crate::flatten::interface_faults(cx.doc, &mut cx.diags);
}

/// **Phase 1a: the flattener's expansion**, its diagnostics said and the unknowns the source
/// declared handed to the sketch — a solved fold and the expression graph read their seeds and
/// dimensions as they are built.
fn expand(
    cx: &mut Elab<'_>,
    refused: &BTreeSet<String>,
    faces: &BTreeMap<String, Vec<FacePath>>,
) -> Expansion {
    let mut expansion = crate::flatten::expand_faces(cx.doc, cx.sk.units, refused, faces);
    cx.map.private_names = expansion.private_names.clone();
    cx.sk.declared = std::mem::take(&mut expansion.unknowns);
    // each face family's running number, held where its drawn station stands (#243 L1)
    for (name, &(dim, at)) in &expansion.held {
        cx.sk.hold(name, at, dim);
    }
    cx.diags.extend(expansion.diagnostics.iter().cloned());
    cx.res.normals = expansion.normals.clone();
    cx.res.charts = expansion.charts.clone();
    cx.res.joints = expansion.joints.clone();
    expansion
}

/// The style sheet, before anything is built: it says nothing about what the drawing is, so it
/// is collected once and never consulted again by anything here (spec §14) — and a `unit` the
/// expansion carried out of a component or a block refused.
fn style_sheet(cx: &mut Elab<'_>, body: &[&Statement]) {
    let mut sheet = crate::style::Sheet::new();
    for st in body {
        // a `unit` inside a component or a block is expanded into the flat list and read by
        // nobody — phase 0 takes the root's alone, a component being reusable and a unit being
        // the document's.  Silence there is exactly what §13.1 forbids, so it is a diagnostic.
        if let StmtKind::Unit(n) = &st.kind {
            if !cx.doc.root().body.iter().any(|r| r.id == st.id) {
                cx.diags.push(Diag::within(
                    Code::E040,
                    n.span,
                    st.id,
                    "a document's unit is stated once, at the top — not inside a \
                     component or a block"
                        .to_string(),
                ));
            }
        }
        if let StmtKind::Style(r) = &st.kind {
            // a class stated twice cascades, later over earlier — the same rule that decides a
            // conflicting property between two classes on one declaration
            sheet.entry(r.name.text.clone()).or_default().over(&r.style);
        }
    }
    cx.sk.set_sheet(sheet);
}

/// **Phase 1b: names, in one pre-pass.**  Indices come from declaration order within a kind,
/// which is `primitives()` order, which is the order phase 2 builds in.  The statements skipped,
/// returned: a redeclaration is skipped rather than merged, and *which* statement was skipped is
/// remembered — inferring it later from the name would find the one that won.
fn names(cx: &mut Elab<'_>, body: &[&Statement]) -> BTreeSet<StmtId> {
    let (p, res) = (cx.doc, &mut cx.res);
    let mut count: BTreeMap<EntKind, u32> = BTreeMap::new();
    let mut skip: BTreeSet<StmtId> = BTreeSet::new();
    for st in body {
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
            cx.diags.push(Diag::within(
                Code::E001,
                name.span(),
                st.id,
                format!(
                    "`{who}` is declared twice; the first is at line {}",
                    p.line_col(was.lo as usize).0
                ),
            ));
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
        res.of.insert(key.clone(), EntRef::new(kind, *n as usize));
        if let StmtKind::Decl(d) = &st.kind {
            res.kids.insert(
                key.clone(),
                d.children.iter().map(|g| match g.first() {
                    Some(crate::ir::Kid::Ref(r)) if g.len() == 1 => Some(r.clone()),
                    _ => None,
                }).collect(),
            );
        }
        *n += 1;
        // a plane's origin is a point minted with the points, in statement order — and a
        // frame's declared bare where its axes are read (#383)
        let charted = |d: &crate::ir::Decl| !d.bare || res.charts.contains(&d.name.key().text);
        if kind == EntKind::Frame && matches!(&st.kind, StmtKind::Decl(d) if charted(d)) {
            let n = count.entry(EntKind::Point).or_insert(0);
            res.origins.insert(key.clone(), *n as usize);
            *n += 1;
        }
    }
    skip
}

/// **Phase 2: geometry, per kind in `primitives()` order.**  The same walk `io::from_json` makes,
/// through the same constructors, so the two produce the same parameter vector. The seeds that
/// read geometry are returned, to be settled once every declaration has a seed of its own.
fn geometry(cx: &mut Elab<'_>, flat: &Flat<'_>) -> Vec<Deferred> {
    let Elab { doc: p, sk, res, map, diags, .. } = cx;
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
        EntKind::Frame,
        EntKind::Curve,
    ] {
        for st in &flat.body {
            let StmtKind::Decl(d) = &st.kind else { continue };
            // a plane's origin, minted with the points in statement order (`Resolver::origins`)
            if kind == EntKind::Point && d.kind == EntKind::Frame && !flat.skip.contains(&st.id)
                && (!d.bare || res.charts.contains(&d.name.key().text))
            {
                let key = &d.name.key().text;
                let o = sk.point(0.0, 0.0, true, &format!("{key}.origin"));
                debug_assert_eq!(res.origins.get(key), Some(&o));
                // bound and recorded with its plane, after it (`build_plane`), so the plane is
                // the first thing its statement made
                continue;
            }
            if d.kind != kind || flat.skip.contains(&st.id) {
                continue;
            }
            let mut anon: Vec<(String, EntRef)> = Vec::new();
            let insts = &flat.expansion.instances;
            match build(sk, res, d, st, diags, &mut anon, &mut deferred, p, insts) {
                // a curve traced in space is built as a traced body (#326): its name takes it,
                // and every later curve sits one index below where phase 1 put it
                Some(e) if e.kind != d.kind => {
                    if let Some(gone) = res.of.insert(d.name.key().text.clone(), e) {
                        for x in res.of.values_mut() {
                            if x.kind == gone.kind && x.idx > gone.idx {
                                x.idx -= 1;
                            }
                        }
                    }
                    map.bind(&d.name.key().text, e, d.name.named());
                    map.record(st, Made::Ent(e));
                }
                Some(e) => {
                    map.bind(&d.name.key().text, e, d.name.named());
                    map.record(st, Made::Ent(e));
                    // an anonymous child's name *is* its dotted path, so it is bound like any
                    // other: that is what lets a dimension name it, a selection survive a
                    // re-elaboration, and a drag of it find the slot it came from.  Whether it
                    // is a name anyone may *say* is the declaration's answer, not its own —
                    // `l.p1` under a named line, nothing under an anonymous one
                    for (name, k) in anon {
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
    deferred
}

/// **Memberships**, once every kind is built and before anything reads one: the points drawn in
/// further planes with their twins, the region solids' probes, `in` (`point a in top` names a
/// plane built after the point, and `project` infers its planes from these), `q coincident P` of
/// a point in space as its membership of `P` said another way — the statements so drawn, and
/// those of points kept in space (`planes::incidences`), returned — and the sets drawn as
/// elements judged (`lowering`), a pass ending where one is none.
fn memberships(
    cx: &mut Elab<'_>,
    flat: &Flat<'_>,
    deferred: &[Deferred],
) -> Result<(BTreeSet<usize>, BTreeSet<usize>), Again> {
    let expansion = flat.expansion;
    // the points drawn in further planes, each with its twins (§6.7): read by the memberships
    // below, so a reader drawn in one of those planes takes the point's twin there
    for (point, twin) in &expansion.twins {
        if let (Some(p), Some(t)) = (cx.res.of.get(point), cx.res.of.get(twin)) {
            cx.sk.twins.entry(p.i()).or_default().push(t.i());
            cx.map.twin_of.insert(*t, *p);
        }
    }
    // the points region solids' sets are applied to, whose bounds are the solids' terms (§6.21)
    cx.probes = Probes {
        terms: expansion.probes.iter().filter_map(|key| cx.res.of.get(key))
            .map(|p| (p.i(), Vec::new()))
            .collect(),
        idle: BTreeMap::new(),
    };
    // memberships, once every kind is built and before anything reads one: `point a in top`
    // names a plane built after the point, and `project` infers its planes from these — and then
    // every point no `in` reached stands in space (`places`), and a plane written over a drawn
    // line has its hidden axis held along it
    planes::memberships(&mut cx.sk, &cx.res, &cx.map, &flat.body, &flat.skip, &mut cx.diags);
    // and `q coincident P` of a point in space is its membership of `P`, said another way — as
    // is a point on a curve drawn in `P`, where one kept in space is refused (`kept_off`)
    let (drawn, kept_off) = planes::incidences(
        &mut cx.sk, &cx.res, &mut cx.map, &flat.stating, deferred, &expansion.rings, &mut cx.diags,
    );
    // and a set drawn as an element is one only where what it reads says so (`lowering`)
    let (sk, res) = (&cx.sk, &cx.res);
    let ent = |r: &crate::syntax::Ref| {
        res.lookup(r).and_then(|e| resolve::follow(sk, e, &r.path).ok())
    };
    let more = crate::lowering::refused(sk, &expansion.lowered, ent, |k| res.of.get(k).copied());
    if !more.is_empty() {
        return Err(Again::Refused(more));
    }
    Ok((drawn, kept_off))
}

/// **Places and holds**: every point no `in` reached stands in space; a point a bound carried
/// across in a pass before stands where it was carried; then the numbers `fix` holds, the planes
/// over held axes, the axes along drawn lines and the rings' copies. The parameters the carried
/// points hold, returned: a seed never writes them.
fn holds(
    cx: &mut Elab<'_>,
    flat: &Flat<'_>,
    deferred: &[Deferred],
    crossed: &BTreeMap<String, Vec<f64>>,
) -> BTreeSet<u32> {
    entities::places(&mut cx.sk, deferred, &mut cx.diags);
    // a point a bound carried across in a pass before stands where it was carried, whatever its
    // own seed says — before anything is placed through it or seeded from it (§9.6, `elaborate`)
    let mut held = BTreeSet::new();
    for (key, values) in crossed {
        let Some(e) = cx.map.ent_named(key) else { continue };
        for (q, &v) in cx.sk.own_params(e).into_iter().zip(values) {
            cx.sk.params[q as usize].value = v;
            held.insert(q);
        }
    }
    // the numbers `fix` holds, once every point has its place and before anything reads one: a
    // held number is its own seed, so nothing that holds one needs a `hint` saying it again — an
    // axis along a line, a motion, a place reading a held point all read where it is held, and
    // a seed never writes a held number (`settle_deferred`).  Every gauge gathered before any is
    // applied, so neither statement order nor a second hold decides what is held (#113)
    for st in &flat.stating {
        let StmtKind::Relation(r) = &st.kind else { continue };
        if gauges::is_fix(r) {
            constrain(cx, r, st);
        }
    }
    cx.gauges.hold_all(&mut cx.sk, &mut cx.diags);
    // a plane whose axes are held stands where they meet (#84), and one held elsewhere takes
    // its free axes with it
    cx.diags.extend(views::origins_on_axes(&mut cx.sk, &cx.map));
    views::stand_axes(&mut cx.sk);
    entities::splines_in_space(&mut cx.sk, &mut cx.res, &mut cx.map);
    let back = entities::walked_back(&cx.sk, &cx.res, &mut cx.diags);
    entities::arcs_in_space(&mut cx.sk, &cx.map, deferred, &back, &mut cx.diags);
    entities::circles_in_space(&mut cx.sk, &cx.res, &cx.map, deferred, &mut cx.diags);
    entities::drawn_in_planes(&cx.sk, &cx.map, &mut cx.diags);
    entities::axes_along(&mut cx.sk, deferred);
    // each normal read turned along its plane as the holds and the lines have stood it
    cx.sk.seed_normals();
    // a ring's copies, turns of its representative: after the holds, which a copy may not have
    rings::rings(&mut cx.sk, &cx.res, &cx.map, &flat.expansion.rings, &mut cx.diags);
    held
}

/// **Motions**, once every line and point they are written over is built — and before the
/// profiles a planar motion generates, which are curves of the drawing a contact may name
/// (§6.15.1).  A motion is evaluated after the solve wherever it is read in space; building it
/// here only resolves what it is written over. The statements of the planar envelopes built,
/// returned.
fn motions(cx: &mut Elab<'_>, flat: &Flat<'_>) -> BTreeSet<StmtId> {
    let Elab { sk, res, map, diags, .. } = cx;
    motions::motions(sk, res, map, &flat.body, &flat.skip, diags);
    generated::planar_envelopes(sk, res, map, &flat.body, &flat.skip, diags)
}

/// **Seeds named by geometry**, once every entity has a seed to be read: in statement order, so a
/// seed that reads a seed read from a third is settled after both (§6.4). Where their findings
/// stand among the diagnostics, returned: each later reading's stand there (`place_planes`).
fn seeds(cx: &mut Elab<'_>, deferred: &[Deferred], held: &BTreeSet<u32>) -> Range<usize> {
    let first = cx.diags.len();
    settle_deferred(&mut cx.sk, &cx.res, deferred, held, &mut cx.diags);
    first..cx.diags.len()
}

/// The plane each face of a solid used as a set is drawn in, and its sides' axes (§6.21); then a
/// prism's side generating under a motion that keeps its view, which stands for a surface the
/// solve can hold a point to: built with the drawing, once the memberships say the view. The
/// statements of those envelopes, returned.
fn prism_sides(cx: &mut Elab<'_>, flat: &Flat<'_>) -> BTreeSet<StmtId> {
    let Elab { sk, res, map, diags, .. } = cx;
    planes::face_planes(sk, res, &flat.expansion.face_planes, &flat.expansion.face_axes, diags);
    generated::extruded_envelopes(sk, res, map, &flat.body, &flat.skip, diags)
}

/// Every other envelope — in space — with what the solve reads of its source, so a contact may
/// name it (#283). The statements of the envelopes generated in the drawing (and those skipped),
/// and of those held in space, returned.
fn held_envelopes(
    cx: &mut Elab<'_>, flat: &Flat<'_>, planar: &BTreeSet<StmtId>, extruded: &BTreeSet<StmtId>,
) -> (BTreeSet<StmtId>, BTreeSet<StmtId>) {
    let Elab { doc, sk, res, map, diags, .. } = cx;
    // the bodies traced in space, which a meshing may turn and a relation may hold to (#326)
    super::traced::traced_bodies(sk, res, &flat.body, doc, &flat.expansion.instances, diags);
    let generated: BTreeSet<StmtId> =
        flat.skip.iter().chain(planar).chain(extruded).copied().collect();
    let held = envelopes::held_envelopes(sk, res, map, &flat.body, &generated);
    envelopes::held_meshings(sk, res, map, &flat.body, &flat.skip, diags);
    (generated, held)
}

/// **Phase 3: constraints, in statement order** — but the holds, stated already (`holds`), and
/// the incidences drawn as memberships or kept in space (`memberships`).
fn constraints(
    cx: &mut Elab<'_>, flat: &Flat<'_>, drawn: &BTreeSet<usize>, kept_off: &BTreeSet<usize>,
) -> Vec<usize> {
    let mut arrays = BTreeSet::new();
    let mut later = Vec::new();
    for (i, st) in flat.stating.iter().enumerate() {
        // an energy's terms are constraints of their own, stationary together (#121)
        if let StmtKind::Minimize(m) = &st.kind {
            variational::state(&mut cx.sk, &cx.res, m, st, &mut cx.map, &mut cx.diags);
            continue;
        }
        // a bound on measurements is a term of the region solid whose point it reads (#183 F3)
        if let StmtKind::Measured(m) = &st.kind {
            regions::measured_term(&cx.sk, &cx.res, m, st, &mut cx.probes, &mut cx.diags);
            continue;
        }
        let StmtKind::Relation(r) = &st.kind else { continue };
        if gauges::is_fix(r) || drawn.contains(&i) || kept_off.contains(&i) {
            continue;
        }
        // a relation reading a region's terms is stated once the solids hold them (#345)
        if r.form.written().is_some_and(|w| crate::constraints::reads_regions(&w.word.text)) {
            later.push(i);
            continue;
        }
        state(cx, r, st, &mut arrays);
    }
    later
}

/// A relation constrained, and what it made recorded against its statement.
fn state(
    cx: &mut Elab<'_>, r: &crate::ir::Relation, st: &Statement,
    arrays: &mut BTreeSet<(crate::syntax::StmtId, Vec<crate::ir::PathStep>, String)>,
) {
    if let Some(id) = constrain(cx, r, st) {
        cx.map.record(st, Made::Con(id));
        if let Some(place) = r.place {
            cx.sk.placements.insert(id, place);
        }
        repeated(&mut cx.sk, id, st, arrays);
    }
}

/// **The relations that read a region's terms** (#345: a flank regular within a blank), stated
/// once the solids are built and hold them, and the unknowns their numbers read are bound.
fn regions_read(cx: &mut Elab<'_>, flat: &Flat<'_>, later: &[usize]) {
    let mut arrays = BTreeSet::new();
    for &i in later {
        let st = flat.stating[i];
        let StmtKind::Relation(r) = &st.kind else { continue };
        let before = cx.sk.constraints.len();
        state(cx, r, st, &mut arrays);
        // its number, written as an expression, is worked out now the drawing's unknowns are
        // bound — as `expr::evaluate` worked out every number stated before it
        for ci in before..cx.sk.constraints.len() {
            let c = &cx.sk.constraints[ci];
            let Some(at) = c.kind.dimension_slot() else { continue };
            let crate::constraints::Arg::Expr(e) = &c.args[at] else { continue };
            let (id, text, written) = (c.id, e.text.clone(), c.written.clone());
            let name = c.kind.spec()[at].0;
            let said = expr::set_dimension(&mut cx.sk, id, name, &text);
            if let Some(c) = cx.sk.constraint_mut(id) {
                c.written = written;
            }
            if let Err(m) | Ok(Some(m)) = said {
                cx.diags.push(Diag::on(Code::E040, st, format!("`{text}`: {m}")));
            } else if cx.sk.constraint(id).is_some_and(|c| c.free.is_some()) {
                let m = format!("`{text}` reads an unknown: a flank's least area factor is held \
                                 to a number, and the unknowns it finds are the drawing's");
                cx.diags.push(Diag::on(Code::E040, st, m));
            }
        }
    }
}

/// **The corners a walk of steps places**, seeded where the steps say once their relations are
/// stated and read (`walk_seeds`, #330): after every written seed, so a seed reading a corner
/// reads its provisional start, as one reading a seed written below does.
fn walked(cx: &mut Elab<'_>, deferred: &[Deferred], held: &BTreeSet<u32>) {
    entities::walk_seeds(&mut cx.sk, deferred, held);
}

/// Once the relations are stated: what each `path` formal was given runs along something, and
/// each tangent at a point reads a contact's place there (#251); each point drawn in further
/// planes tied to its twins, once the holds say which planes are fixed and the relations what
/// else holds it on each (§6.7); the rings' contacts; and each tangency at a point gauging its
/// directions where the point stands (§6.21).
fn twins(cx: &mut Elab<'_>, flat: &Flat<'_>) {
    relations::kinds(&cx.sk, &cx.map, &flat.expansion.kinds, &mut cx.diags);
    relations::tangents_at(&cx.sk, &cx.map, &mut cx.diags);
    planes::tie_twins(&mut cx.sk, &cx.map, &mut cx.diags);
    cx.sk.hold_twins();
    rings::contacts(&cx.sk, &cx.map, &mut cx.diags);
    cx.sk.choose_charts();
}

/// **Where the planes stand**, from the statements that say so, before the drawing in them is
/// read through them: a place drawn in another plane is read in space, through that plane's
/// pose, so the seeds are settled again, in statement order, once the planes are placed.  Only
/// then — a document whose places are all in their own planes settles once.  A plane placed over
/// seeds read through another plane is read through in turn, so the two alternate until the
/// seeds stand still — a few rounds, one per plane a chain of them crosses.
fn place_planes(
    cx: &mut Elab<'_>, deferred: &[Deferred], held: &BTreeSet<u32>, mut settled: Range<usize>,
) {
    let mut placed = views::place(&mut cx.sk);
    // each point a motion carries to a stated roll where the motion carries it, from the planes
    // placed, and the planes drawn over such points placed again — a carried frame's over its
    // carried frame's in turn (#366)
    for _ in 0..4 {
        // a joint held to a frame's axis turns about it as that frame now stands
        cx.sk.seed_joints();
        if !cx.sk.seed_carried() {
            break;
        }
        views::relift(&mut cx.sk);
        placed = views::place(&mut cx.sk);
    }
    if crosses_views(&cx.sk, &cx.res, deferred) {
        for _ in 0..4 {
            let before: Vec<f64> = cx.sk.params.iter().map(|p| p.value).collect();
            // each reading's findings stand where the first's did
            let mut again = Vec::new();
            settle_deferred(&mut cx.sk, &cx.res, deferred, held, &mut again);
            let found = settled.start..settled.start + again.len();
            cx.diags.splice(settled, again);
            settled = found;
            // and the planes placed again over the points the seeds moved
            placed = views::place(&mut cx.sk);
            let moved = cx.sk.params.iter().zip(&before)
                .any(|(p, &b)| (p.value - b).abs() > 1e-12 * (1.0 + b.abs()));
            if !moved {
                break;
            }
        }
    }
    // a point drawn in planes that never meet is on none of them at once (§6.7): judged of the
    // planes placed, and of the rest where the solve leaves them (`views::degenerate`)
    planes::parallel_twins(&cx.sk, &cx.map, &placed, &mut cx.diags);
    // a ring's copies where their representatives' seeds put them, whatever a seed said of a copy
    cx.sk.settle_turns();
    cx.sk.seed_normals();
    cx.sk.seed_joints();
}

/// **Phase 3b: faces, then solids** (§6.8, §6.9).  **After every other kind and after the
/// constraints**, because a face is written over edges the drawing already has and a solid over
/// faces and other solids; and *evaluated* rather than solved, because nothing about either is an
/// unknown.  This is the stratification as a phase: everything above it is the drawing,
/// everything below reads what the drawing came to.  (Motions were built with the primitives:
/// solids retain their indices.)  Then the analytic surfaces and what is written over them; a
/// pass ends where a block runs over faces not yet named.
fn solids_and_surfaces(
    cx: &mut Elab<'_>,
    flat: &Flat<'_>,
    faces: &BTreeMap<String, Vec<FacePath>>,
    generated: &BTreeSet<StmtId>,
    held: &BTreeSet<StmtId>,
) -> Result<(), Again> {
    let Elab { sk, res, map, diags, probes, .. } = cx;
    let (body, skip) = (&flat.body, &flat.skip);
    solids(sk, res, map, body, skip, probes, diags);
    super::solids::carry_legs(sk);
    // the faces of the solids a block runs over, named for the next pass (#183 F4)
    let unnamed: BTreeMap<String, Vec<FacePath>> = flat.expansion.face_repeats.iter()
        .filter(|abs| !faces.contains_key(*abs))
        .filter_map(|abs| {
            res.of.get(abs).filter(|e| e.kind == EntKind::Solid && e.i() < sk.solids.len())
                .map(|e| (abs.clone(), crate::solid::faces::face_paths(sk, e.i())))
        })
        .collect();
    if !unnamed.is_empty() {
        return Err(Again::Faces(unnamed));
    }
    surfaces::surfaces(sk, res, map, body, skip, diags);
    envelopes::envelopes(sk, res, map, body, generated, held, diags);
    Ok(())
}

/// **Phase 4: every expression against the whole document, once** (per-statement evaluation
/// would be quadratic in the expression count).  The claims about solids read the free variables
/// `evaluate` allocates, so they come after it — the one pass of the elaboration that is *below*
/// the expressions.
fn expressions(cx: &mut Elab<'_>, flat: &Flat<'_>) {
    let Elab { sk, res, map, diags, .. } = cx;
    let post_expr = expr::evaluate(sk);
    // every free curve's energy, once every row it may be held by is stated — a length an
    // unknown sets included (#121, #144)
    variational::settle(sk, map, diags);
    solid_claims(sk, res, map, &flat.stating, &flat.skip, diags);
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
            diags.push(Diag::new(code, span, stmt, format!("`{}`: {err}{tail}", item.text)));
        }
    }

    // a curve written over a number its drawn instance left unknown reads that unknown as a
    // column, and only the expression graph allocates one — when a dimension of the drawing
    // reads it.  One nothing reads would leave the curve a column short
    for (i, cv) in sk.curves.iter().enumerate() {
        for n in cv.unknowns.iter().filter(|n| !sk.free_vars.contains_key(*n)) {
            let site = map.site_of(EntRef::new(EntKind::Curve, i));
            diags.push(Diag::new(
                Code::E103,
                site.map(|s| s.span).unwrap_or_default(),
                site.map(|s| s.stmt),
                format!(
                    "`{n}` is left unknown, and nothing on the sheet reads it: give the \
                     instance a number for it, or state what it is"
                ),
            ));
        }
    }
}

/// **Phase 5: a root choice under a key no triple of points spells**, kept verbatim, and every
/// root choice applied; the roles each statement gave what it made; the diagnostics' last words
/// — and the pass's drawing.
fn finish(mut cx: Elab<'_>, flat: &Flat<'_>) -> Elaborated {
    for st in &flat.stating {
        if let StmtKind::Branch(b) = &st.kind {
            cx.gauges.branch(b, st);
        }
    }
    cx.gauges.choose_all(&mut cx.sk, &mut cx.diags);

    // Roles follow creation provenance, including unnamed children. Referenced geometry
    // belongs to its own declaration, even when a construction component borrows it.
    let roles: BTreeMap<_, _> = flat.body.iter().filter_map(|st| {
        let roles = match &st.kind {
            StmtKind::Decl(d) => d.roles,
            StmtKind::Chain(c) => c.annotations.roles,
            _ => return None,
        };
        (roles != Default::default()).then_some(((st.id, st.path.clone()), roles))
    }).collect();
    for (&entity, site) in &cx.map.of_entity {
        if let Some(&roles) = roles.get(&(site.stmt, site.path.0.clone())) {
            cx.sk.roles.insert(entity, roles);
        }
    }
    // a measurement read before the solve is one refusal wherever it stood — a param, a seed, a
    // constraint's number, an extent — and every one of those paths says it in the same words
    // (`expr::measure_refusal`), so the code is given once, here, the way the flattener sorts
    // its plain errors by message
    for d in &mut cx.diags {
        if d.message.contains(expr::MEASURE_MARK) {
            d.code = Code::E107;
        }
    }
    // a declaration reading a formal whose argument was said to name nothing is not said again
    // where the body reads it (#253): the argument is where the fault is
    let quiet = &flat.expansion.quiet;
    cx.diags.retain(|d| !(d.code == Code::E101 && d.stmt.is_some() && quiet.contains(&d.span)));
    let p = cx.doc;
    crate::modules::localize(p, &mut cx.diags);
    Elaborated { sketch: cx.sk, map: cx.map, diags: cx.diags, program: p.clone(), taken: false }
}
