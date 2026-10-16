//! Build primitive entities and resolve their initial seeds.

#[allow(unused_imports)]
use crate::fmath::Det;
use super::curves::build_curve;
use super::resolve::{follow, follow_building, Resolver, NORMAL};
use super::{Code, Diag};
use crate::ir::{Decl, Kid, Statement as Stmt};
use crate::model::{EntKind, EntRef, Field, Sketch};
use crate::rng::Rng;
use crate::style::Classes;
use crate::syntax::{AtRef, DirRef, Program, Seg, Span, StmtId};
use crate::{curve, expr, io};
use std::collections::{BTreeMap, BTreeSet};

/// Default radius for unseeded circles and degenerate arcs. Keep it nonzero so
/// the radius gradient can move the initial geometry.
const UNSEEDED_RADIUS: f64 = 1.0;

fn scatter(i: usize) -> (f64, f64) {
    // the bearing walks a fixed step per minted point, in creation order — which for a chain's
    // corners is traversal order, so a contour of implicit points seeds as a *simple polygon*
    // and not as a pile or a self-crossing quad, whose nearest solution is a collapsed side (a
    // zero-length line satisfies every direction constraint on it).  The step is irrational in
    // turns (half the golden angle), so no later point lands back on an earlier bearing.
    const STEP: f64 = 1.199982;
    let mut rng = Rng::new(0x5eed_u32 ^ (i as u32).wrapping_mul(2_654_435_761));
    let th = i as f64 * STEP + rng.uniform(-0.2, 0.2);
    let r = rng.uniform(0.8, 1.2);
    (r * th.dcos(), r * th.dsin())
}

/// Where an unseeded axis points: the `i`th of a spiral over the sphere, so two axes a document
/// left unseeded neither coincide nor start opposed — either is a stationary point of an angle
/// between them.  The same irrational steps as `scatter`, one in bearing and one in height.
fn scatter_direction(i: usize) -> [f64; 3] {
    const STEP: f64 = 2.399963;
    let z = ((i as f64 * 0.618034 + 0.31) % 1.0) * 1.6 - 0.8;
    let th = i as f64 * STEP + 0.4;
    let r = (1.0 - z * z).sqrt();
    [r * th.dcos(), r * th.dsin(), z]
}

/// Child display names under `base`, such as `l.p1` and `a.center`.
/// Anonymous declarations use their generated display name as the base.
pub(crate) fn child_names(kind: EntKind, base: &str) -> Vec<String> {
    kind.fields()
        .iter()
        .filter(|(_, f)| *f == Field::Child)
        .map(|(n, _)| format!("{base}.{n}"))
        .collect()
}

/// Use the declared display name, or a positional model name for an anonymous
/// declaration. Internal resolution keys must not appear in parameter labels.
fn shown(sk: &Sketch, d: &Decl) -> String {
    match d.name.shown() {
        None => crate::syntax::entity_name(EntRef::new(d.kind, sk.count(d.kind))),
        Some(n) => n.text.clone(),
    }
}
/// **A free curve** (#144), `rope := curve(a, b)`: from its first point to its second, its shape
/// the drawing's to find — the solution of the energy stated over it (`extremal.rs`), until one is
/// stated a curve of no shape.  It owns one number, its length, seeded a gentle sag over the chord
/// (a `length` row holds it, or the energy's stationarity in it sets it).
fn build_free_curve(sk: &mut Sketch, res: &Resolver, d: &Decl, st: &Stmt, diags: &mut Vec<Diag>) -> Option<EntRef> {
    let err = |diags: &mut Vec<Diag>, code, span, message: String| {
        diags.push(Diag::within(code, span, st.id, message));
        None
    };
    let mut ends = Vec::with_capacity(2);
    for kid in d.children.iter().flatten() {
        let Kid::Ref(r) = kid else {
            return err(diags, Code::E040, st.span, "a free curve runs between two points it names, `curve(a, b)`".into());
        };
        let Some(e) = res.lookup(r) else {
            return err(diags, Code::E101, r.span, format!("no such entity: `{}`", r.root.text));
        };
        let e = match follow_building(sk, res, e, r) {
            Ok(e) => e,
            Err(msg) => return err(diags, Code::E040, r.span, msg),
        };
        if e.kind != EntKind::Point {
            return err(diags, Code::E040, r.span, format!("`{}` is {}, and a free curve runs between points", r.root.text, e.kind.a()));
        }
        ends.push(EntRef::point(e.i()));
    }
    if ends.len() != 2 {
        return err(
            diags,
            Code::E103,
            st.span,
            format!("a free curve runs between two points, `curve(a, b)`, and {} were given", ends.len()),
        );
    }
    let (a, b) = (sk.point_xy(ends[0].i()), sk.point_xy(ends[1].i()));
    // a gentle sag over the chord, until `Sketch::seed_extremals` seeds it once the ends are placed
    let chord = (b.0 - a.0).dhypot(b.1 - a.1);
    let length = sk.param((crate::extremal::shoot::EASY * chord).max(1.0), false, &format!("{}.length", shown(sk, d))) as u32;
    let def = match sk.curve_defs.iter().position(|c| c.name == "extremal:") {
        Some(k) => k,
        None => {
            sk.curve_defs.push(crate::model::extremal_def("extremal:".into(), None, 0, 0));
            sk.curve_defs.len() - 1
        }
    };
    sk.curves.push(crate::model::CurveE {
        def: def as u32,
        args: ends,
        unknowns: Vec::new(),
        values: Vec::new(),
        domain: (0.0, 1.0),
        home: crate::model::Home::At(0.0),
        pose: Vec::new(),
        class: d.class.clone(),
        trim: None,
        extrusion: false,
        length: Some(length),
        pegs: Vec::new(),
        slides: Vec::new(),
    });
    Some(EntRef::new(EntKind::Curve, sk.curves.len() - 1))
}

#[allow(clippy::too_many_arguments)]
pub(super) fn build(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    st: &Stmt,
    diags: &mut Vec<Diag>,
    anon: &mut Vec<(String, EntRef)>,
    deferred: &mut Vec<Deferred>,
    prog: &Program,
    insts: &[crate::flatten::InstanceInfo],
) -> Option<EntRef> {
    // a curve is the one kind whose arguments need not be points, so it is built before the
    // walk that insists they are
    if d.kind == EntKind::Curve {
        if d.curve.is_none() {
            return build_free_curve(sk, res, d, st, diags);
        }
        return build_curve(sk, res, d, st, diags, prog, insts);
    }
    // and a plane's are two axes, and its origin is its own; a frame declared bare owns
    // nothing until its axes are read (#383)
    if d.kind == EntKind::Frame {
        let e = if d.bare && !res.charts.contains(&d.name.key().text) {
            sk.frames.push(crate::model::FrameE::bare());
            let e = EntRef::frame(sk.frames.len() - 1);
            set_class(sk, e, d.class.clone());
            e
        } else {
            build_plane(sk, res, d, st, diags, anon, deferred)?
        };
        // its joint, where the document reads it (#383): `f.axis`, minted with it
        if res.joints.contains(&d.name.key().text) {
            let j = sk.mint_joint(e.i(), &shown(sk, d));
            anon.push((format!("{}.axis", d.name.key().text),
                EntRef::new(EntKind::Axis, j.axis as usize)));
        }
        return Some(e);
    }
    // a seed named by a place (`hint(at: k, bearing: b)`) is a point's; every other kind has
    // a scalar of its own the clause seeds by name
    if d.seed_at.is_some() && d.kind != EntKind::Point {
        diags.push(Diag::on(
            Code::E103,
            st,
            "only a point takes a geometric seed (`hint(at: …)`)".to_string(),
        ));
        return None;
    }
    let kids = children(sk, res, d, st, diags, anon, deferred)?;
    let idx = make(sk, d, st, &kids, diags, deferred)?;
    let e = EntRef::new(d.kind, idx);
    if d.kind == EntKind::Arc && d.sense == Some(crate::syntax::Sense::Cw) {
        deferred.push(Deferred::Clockwise { arc: idx, span: st.span, stmt: st.id });
    }
    if let (EntKind::Circle, Some(n)) = (d.kind, &d.normal) {
        deferred.push(Deferred::Normal { circle: idx, normal: n.clone(), span: st.span,
                                         stmt: st.id });
    }
    set_class(sk, e, d.class.clone());
    defer_seeds(sk, d, st, e, deferred);
    Some(e)
}

/// Every child a declaration names or mints, in field order, each a point — or `None`, refused.
fn children(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    st: &Stmt,
    diags: &mut Vec<Diag>,
    anon: &mut Vec<(String, EntRef)>,
    deferred: &mut Vec<Deferred>,
) -> Option<Vec<usize>> {
    // every child a declaration names, flattened in field order and checked to be a Point —
    // which every other child of every other kind is, and which is what an alias class must
    // agree about.  A slot may hold a *seed* instead of a name, and a declaration may write no
    // list at all: both mint a point nothing names, reached as `l.p1` (spec §6.1, §6.2).
    let written: usize = d.children.iter().map(|g| g.len()).sum();
    // the dotted names, worked out only where one is needed: every child a document names is a
    // `Kid::Ref`, and formatting names nothing would read is a string per slot per elaboration
    let anonymous = written == 0
        || d.children.iter().any(|g| g.is_empty() || g.iter().any(|k| matches!(k, Kid::Hint(_))));
    // The child's **name**, which is its dotted path, and what to **call** it — the same string
    // for a declaration the source named, and two different ones for an anonymous declaration,
    // whose key is an offset nobody should be shown.
    let dotted = if anonymous { child_names(d.kind, &d.name.key().text) } else { Vec::new() };
    let label = if anonymous { child_names(d.kind, &shown(sk, d)) } else { Vec::new() };
    let mut kids: Vec<usize> = Vec::new();
    // `Some(0)` and `None` both mean there is nothing to mint, and so does a written list
    let mint = if written == 0 { d.kind.children_arity().unwrap_or(0) } else { 0 };
    for k in 0..mint {
        let (x, y) = scatter(sk.points.len());
        // the dotted path *is* the point's name — there is no other — so it is what the map
        // binds; the sketch carries what a reader is shown
        let i = sk.point(x, y, false, &label[k]);
        deferred.push(Deferred::Scattered { point: i });
        kids.push(i);
        anon.push((dotted[k].clone(), EntRef::point(i)));
    }
    let mut slot = 0usize;
    for group in &d.children {
        // a slot nothing names or seeds is an *implicit child*, minted exactly as a declaration
        // that writes no list at all mints them (spec §6.2) — which is what lets a chain's
        // thread fill only the slots it speaks for (`(l1 := line) -> (l2 := line)`) and leave the
        // rest the drawing's own
        if group.is_empty() && written != 0 {
            // a `List` kind has no arity to mint from, and a slot with no dotted path has no
            // name to be reached by
            if let Some(name) = d.kind.children_arity().and(dotted.get(slot)) {
                let (x, y) = scatter(sk.points.len());
                let i = sk.point(x, y, false, label.get(slot).unwrap_or(name));
                deferred.push(Deferred::Scattered { point: i });
                kids.push(i);
                anon.push((name.clone(), EntRef::point(i)));
            }
            slot += 1;
            continue;
        }
        for kid in group {
            let r = match kid {
                Kid::Ref(r) => r,
                // only solids accept inline sections, and only a face's loop runs along a curve
                Kid::Face { .. } | Kid::Trim { .. } => return None,
                Kid::Hint(seed) => {
                    // a list slot has no arity, so it has no dotted path to be named by, and a
                    // point nothing can name is a point nothing can constrain or drag
                    let Some(name) = dotted.get(slot) else {
                        diags.push(Diag::on(
                            Code::E103,
                            st,
                            format!(
                                "{}'s control points have no names to be reached by, so each \
                                 one is declared",
                                d.kind.a()
                            ),
                        ));
                        return None;
                    };
                    // a child is drawn where its parent is; a point in space is declared
                    if !seed.spans[2].is_empty() {
                        diags.push(Diag::within(
                            Code::E040,
                            seed.spans[2],
                            st.id,
                            "an anonymous point has no `z`: it is drawn where its parent \
                             is, and a point in space is declared, `p := point hint(x: …, \
                             y: …, z: …)`".to_string(),
                        ));
                        return None;
                    }
                    let i = sk.point(seed.v[0], seed.v[1], false, label.get(slot).unwrap_or(name));
                    for (k, t) in seed.text.iter().enumerate() {
                        if let Some(t) = t {
                            deferred.push(Deferred::Text {
                                param: sk.point_params(i)[k],
                                text: t.clone(),
                                names: d.seed_names.clone(),
                                span: seed.spans[k],
                                stmt: st.id,
                            });
                        }
                    }
                    kids.push(i);
                    anon.push((name.clone(), EntRef::point(i)));
                    slot += 1;
                    continue;
                }
            };
            slot += 1;
            let Some(e) = res.lookup(r) else {
                diags.push(Diag::within(
                    Code::E101,
                    r.span,
                    st.id,
                    format!("no such entity: `{}`", r.root.text),
                ));
                return None;
            };
            let e = match follow_building(sk, res, e, r) {
                Ok(e) => e,
                Err(msg) => {
                    diags.push(Diag::within(
                        Code::E040,
                        r.span,
                        st.id,
                        msg,
                    ));
                    return None;
                }
            };
            if e.kind != EntKind::Point {
                diags.push(Diag::within(
                    Code::E040,
                    r.span,
                    st.id,
                    format!(
                        "`{}` is {}, and {} is built from points",
                        r.root.text,
                        e.kind.a(),
                        d.kind.a()
                    ),
                ));
                return None;
            }
            kids.push(e.i());
        }
    }
    // a slot carries a name, a seed, or nothing — an implicit child, minted above — so the one
    // thing left to refuse is *more* than the kind has slots for
    let want = d.kind.children_arity();
    if let Some(n) = want {
        if kids.len() != n {
            diags.push(Diag::on(
                Code::E103,
                st,
                format!(
                    "{} is built from {n} point(s), and {} were given",
                    d.kind.a(),
                    written
                ),
            ));
            return None;
        }
    }
    Some(kids)
}

/// The entity a declaration of a point-built kind makes over its children, its index in its
/// kind's list — or `None`: a kind built by a later phase, or refused.
fn make(
    sk: &mut Sketch,
    d: &Decl,
    st: &Stmt,
    kids: &[usize],
    diags: &mut Vec<Diag>,
    deferred: &mut Vec<Deferred>,
) -> Option<usize> {
    let seed = |i: usize| d.seed.get(i).copied().unwrap_or(0.0);
    // what a reader is shown: the declaration's own name, or what the drawing calls it where
    // the source named nothing — a scalar carries this into every list of parameters
    let show = shown(sk, d);
    // A point whose source wrote no seed at all — no `hint(…)` clause (the empty span where
    // one would go) and no place — starts where a minted child does, not at the origin: two
    // such points on top of each other put every distance between them at a stationary point
    // of its own residual, and the first document anybody writes solved as a conflict (#43).
    // A declaration lifted from a sketch has no span (`None`) and carries its numbers.
    let unseeded = d.unseeded;
    // A scalar the source never wrote reads as 0, and for a radius 0 is a stationary point of
    // every on-circle row (∂/∂r of |p−c|² − r² is −2r): an `arc` with its ends grounded and no
    // `hint(r: )` could not solve at all, and a conflict elsewhere in the drawing was blamed on
    // the arc's own intrinsic, the first row a search from that pose could not satisfy (#45.6).
    // So a radius is *written or computed*, never defaulted: the constructor's geometric one for
    // an arc, `UNSEEDED_RADIUS` for a circle and wherever the
    // geometry gives nothing.  A declaration lifted from a sketch has no spans and carries its
    // numbers, as for a point above.
    let wrote = |i: usize| d.seed_explicit.get(i).copied().unwrap_or(false);
    let nonzero = |r: f64| if r.abs() > 1e-9 { r } else { UNSEEDED_RADIUS };
    let idx = match d.kind {
        // built by their own phase, after every other kind: a face is written over edges and a
        // solid over faces and solids, so neither can be minted by the walk that makes points
        EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Meshing | EntKind::Traced => return None,
        EntKind::Point if unseeded => {
            let (x, y) = scatter(sk.points.len());
            let p = sk.point(x, y, false, &show);
            deferred.push(Deferred::Scattered { point: p });
            p
        }
        EntKind::Point => {
            let p = sk.point(seed(0), seed(1), false, &show);
            if wrote(2) {
                let span = d.seed_spans.get(2).copied().unwrap_or(st.span);
                let text = d.seed_text.get(2).cloned().flatten();
                deferred.push(Deferred::Height {
                    point: p,
                    z: seed(2),
                    text,
                    names: d.seed_names.clone(),
                    span,
                    stmt: st.id,
                });
            }
            p
        }
        EntKind::Line => sk.line(kids[0], kids[1]),
        EntKind::Circle => {
            let r = if wrote(0) { seed(0) } else { UNSEEDED_RADIUS };
            sk.circle(kids[0], r, &show)
        }
        EntKind::Arc => {
            // `arc` adds the two intrinsic `PointOnCircle`s here and nowhere else, and computes a
            // radius from the geometry that a *written* seed then replaces
            let ai = sk.arc(kids[0], kids[1], kids[2], &show);
            let rp = sk.arcs[ai].radius as usize;
            sk.params[rp].value = if wrote(0) { seed(0) } else { nonzero(sk.params[rp].value) };
            // and computed again once the places are settled, since a centre or an end seeded
            // by one stands where it was placed only then
            if !wrote(0) {
                deferred.push(Deferred::Radius { arc: ai });
            }
            ai
        }
        EntKind::Spline => {
            if kids.len() > io::MAX_CTRL {
                diags.push(Diag::on(
                    Code::E104,
                    st,
                    format!(
                        "a curve may not have more than {} control points",
                        io::MAX_CTRL
                    ),
                ));
                return None;
            }
            match sk.spline_weighted(kids, d.knots.clone(), d.weights.clone()) {
                Some(si) => si,
                None => {
                    let message = if d.weights.as_ref().is_some_and(|w| !curve::weights_valid(w, kids.len())) {
                        format!("a curve's weights are one per control point ({}), each a positive number", kids.len())
                    } else {
                        format!(
                            "a curve needs more than {} control points and a matching knot vector",
                            curve::DEGREE
                        )
                    };
                    diags.push(Diag::on(Code::E103, st, message));
                    return None;
                }
            }
        }
        // a direction, seeded where the source wrote one, and a place no relation reads yet
        EntKind::Axis => {
            let dir = if (0..3).any(wrote) {
                [seed(0), seed(1), seed(2)]
            } else {
                scatter_direction(sk.axes.len())
            };
            let ri = sk.axis(dir, &show);
            for k in 0..3 {
                if wrote(3 + k) {
                    let p = sk.axes[ri].a[k] as usize;
                    sk.params[p].value = seed(3 + k);
                }
            }
            ri
        }
        EntKind::Curve => unreachable!("a curve is built before this walk"),
        EntKind::Frame => unreachable!("built by `build_plane`"),
    };
    Some(idx)
}

/// The seeds a declaration wrote over geometry, settled once everything is built: a point's
/// place, and any scalar's text that read another entity's.
fn defer_seeds(sk: &Sketch, d: &Decl, st: &Stmt, e: EntRef, deferred: &mut Vec<Deferred>) {
    let idx = e.i();
    if let Some(at) = &d.seed_at {
        deferred.push(Deferred::At {
            point: idx,
            at: at.clone(),
            names: d.seed_names.clone(),
            span: st.span,
            stmt: st.id,
        });
    }
    // the declaration's own scalars are the last of `entity_params`, after its children's
    let params = sk.entity_params(e);
    let own = &params[params.len().saturating_sub(d.seed_text.len())..];
    for (k, t) in d.seed_text.iter().enumerate() {
        if let (Some(t), Some(param)) = (t, own.get(k).copied()) {
            let span = d.seed_spans.get(k).copied().unwrap_or(st.span);
            deferred.push(Deferred::Text {
                param,
                text: t.clone(),
                names: d.seed_names.clone(),
                span,
                stmt: st.id,
            });
        }
    }
}

/// A plane: two axes — or drawn lines, each given a hidden axis parallel to it — and an origin of
/// its own, a point drawn in it at `(0, 0)` reached as `P.origin`; where it stands is its `x`,
/// `y`, `z`, seeded by `hint((, , ))`.
fn build_plane(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    st: &Stmt,
    diags: &mut Vec<Diag>,
    anon: &mut Vec<(String, EntRef)>,
    deferred: &mut Vec<Deferred>,
) -> Option<EntRef> {
    let fail = |diags: &mut Vec<Diag>, span: Span, message: String| {
        diags.push(Diag::within(Code::E103, span, st.id, message));
    };
    let show = shown(sk, d);
    // the slots in field order: `u`, `v`, then `origin`, which is never written
    if d.children.get(2).is_some_and(|g| !g.is_empty()) {
        fail(diags, st.span, "a plane's origin is its own, a point drawn in it at (0, 0): reach it \
            as `P.origin`, and place it with `P.origin coincident p`".to_string());
        return None;
    }
    let mut axes = [0usize; 2];
    let mut minted: Vec<(&str, usize)> = Vec::new();
    for (k, key) in ["u", "v"].iter().enumerate() {
        axes[k] = plane_axis(sk, res, d, st, k, key, &show, diags, &mut minted, deferred)?;
    }
    let wrote = |i: usize| d.seed_explicit.get(i).copied().unwrap_or(false);
    let o = [0, 1, 2].map(|i| if wrote(i) { d.seed.get(i).copied().unwrap_or(0.0) } else { 0.0 });
    // the origin was minted with the points (`Resolver::origins`)
    let Some(&origin) = res.origins.get(&d.name.key().text) else { return None };
    let pi = sk.plane_over(axes[0], axes[1], o, origin, &show);
    // a frame declared bare whose axes are read: spatial, its origin at (0, 0, 0) along them
    if d.bare {
        sk.spatial_chart(pi);
    }
    if let Some(n) = d.name.shown() {
        sk.plane_names.insert(pi as u32, n.text.clone());
    }
    anon.push((format!("{}.origin", d.name.key().text), EntRef::point(origin)));
    for (key, ax) in minted {
        anon.push((format!("{}.{key}", d.name.key().text), EntRef::new(EntKind::Axis, ax)));
    }
    // its normal, where the document reads it: minted here, so a declaration after it may name
    // it, and recorded with the statement as a minted axis is
    if res.normals.contains(&d.name.key().text) {
        let n = sk.plane_normal(pi, &format!("{show}.{NORMAL}"));
        anon.push((format!("{}.{NORMAL}", d.name.key().text), EntRef::new(EntKind::Axis, n)));
    }
    let e = EntRef::frame(pi);
    set_class(sk, e, d.class.clone());
    Some(e)
}

/// A plane's slot `k`, `key` (`u`, `v`): the axis it is — named, a hidden one held along a drawn
/// line, or one of the plane's own, free, seeded where the slot says (`minted` takes each axis the
/// statement made) — or `None`, refused.
#[allow(clippy::too_many_arguments)]
fn plane_axis<'k>(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    st: &Stmt,
    k: usize,
    key: &'k str,
    show: &str,
    diags: &mut Vec<Diag>,
    minted: &mut Vec<(&'k str, usize)>,
    deferred: &mut Vec<Deferred>,
) -> Option<usize> {
    let fail = |diags: &mut Vec<Diag>, span: Span, message: String| {
        diags.push(Diag::within(Code::E103, span, st.id, message));
    };
    let r = match d.children.get(k).map(|g| g.as_slice()) {
        Some([Kid::Ref(r)]) => r,
        // a slot left out is an axis of the plane's own, free: `p := plane` is an origin and
        // two directions, seeded as the front's unless a `hint((, , ))` in the slot says
        // which way it starts
        None | Some([]) | Some([Kid::Hint(_)]) => {
            let seed = match d.children.get(k).and_then(|g| g.first()) {
                Some(Kid::Hint(s)) => Some(s),
                _ => None,
            };
            let dir = match seed {
                // a direction read off geometry is read once everything has its seed; until
                // then the axis starts as an unseeded one does
                Some(s) if s.dir.is_some() => if k == 0 { [1.0, 0.0, 0.0] } else { [0.0, 0.0, 1.0] },
                // what an expression comes to is settled later, as a named axis's is
                Some(s) if s.v.iter().any(|&x| x != 0.0) || s.text.iter().any(|t| t.is_some()) => s.v,
                Some(s) => {
                    fail(diags, s.span, format!(
                        "a plane's `{key}` seeds an axis's direction, and one of no length \
                         points nowhere"
                    ));
                    return None;
                }
                None if k == 0 => [1.0, 0.0, 0.0],
                // a spatial frame starts as the world stands, up along `y` and out along `z`
                None if d.bare => [0.0, 1.0, 0.0],
                None => [0.0, 0.0, 1.0],
            };
            let axis = sk.axis(dir, &format!("{show}.{key}"));
            if let Some(dir) = seed.and_then(|s| s.dir.as_ref()) {
                deferred.push(Deferred::Dir {
                    axis,
                    dir: dir.clone(),
                    names: d.seed_names.clone(),
                    span: seed.map_or(st.span, |s| s.span),
                    stmt: st.id,
                });
            }
            for (j, t) in seed.iter().flat_map(|s| s.text.iter().enumerate()) {
                let (Some(t), Some(s)) = (t, seed) else { continue };
                deferred.push(Deferred::Text {
                    param: sk.axes[axis].d[j],
                    text: t.clone(),
                    names: d.seed_names.clone(),
                    span: s.spans[j],
                    stmt: st.id,
                });
            }
            minted.push((key, axis));
            return Some(axis);
        }
        Some(_) => {
            fail(diags, st.span, format!(
                "a plane's `{key}` is an axis or a line, named: `plane(u: a1, v: a2)`, or an \
                 axis of its own, left out or seeded: `plane(u: hint(dir: (0, 1, 0)))`"
            ));
            return None;
        }
    };
    let Some(e) = res.lookup(r) else {
        diags.push(Diag::within(
            Code::E101,
            r.span,
            st.id,
            format!("no such entity: `{}`", r.root.text),
        ));
        return None;
    };
    let e = match follow_building(sk, res, e, r) {
        Ok(e) => e,
        Err(msg) => {
            diags.push(Diag::within(Code::E040, r.span, st.id, msg));
            return None;
        }
    };
    Some(match e.kind {
        EntKind::Axis => e.i(),
        // a drawn line is an axis from its start toward its end: a hidden one, held parallel
        // to it once its points stand somewhere — minted by this statement as a left-out
        // slot's is, so recorded with it, or `reconcile` reads it as one a gesture drew
        EntKind::Line => {
            let axis = sk.axis([1.0, 0.0, 0.0], &format!("{show}.{key}"));
            deferred.push(Deferred::Along { axis, line: e.i() });
            minted.push((key, axis));
            axis
        }
        other => {
            fail(diags, r.span, format!(
                "`{}` is {}, and a plane's `{key}` is an axis or a line",
                r.root.text,
                other.a()
            ));
            return None;
        }
    })
}

/// **Points in space** (`docs/planes-plan.md`): every point no `in` reached — no membership once
/// every membership is in — stands in space, and is given its third coordinate here, at its
/// `hint(z: …)` or at 0.  A `z` seed on a point drawn in a plane is refused: there it has two
/// coordinates, its plane's.
pub(super) fn places(sk: &mut Sketch, deferred: &[Deferred], diags: &mut Vec<Diag>) {
    let mut height: BTreeMap<usize, f64> = BTreeMap::new();
    for d in deferred {
        let &Deferred::Height { point, z, span, stmt, .. } = d else { continue };
        if sk.plane_of(point).is_some() {
            diags.push(Diag::within(
                Code::E040,
                span,
                stmt,
                "a point drawn in a plane has that plane's two coordinates, and no `z`"
                    .to_string(),
            ));
        } else {
            height.insert(point, z);
        }
    }
    for p in 0..sk.points.len() {
        if sk.points[p].plane.is_none() && sk.points[p].z.is_none() {
            sk.give_place(p, height.get(&p).copied().unwrap_or(0.0));
        }
    }
}

/// **A spline over points in space is a curve traced in space** (#363 S3): `k := spline(a, b, c,
/// d)` with a control point no plane reaches is the body `X = Σ Rᵢ(u)·Pᵢ` over the control points'
/// lifts (`traced::RowKind::Spline`), its formal the spline's parameter over its domain — so its
/// contacts are a traced curve's, exact, it is a `path`, and `spline(through: …)` over corners in
/// space is the cubic through them.  A representation, never a meaning: the curve is the spline's
/// own, its knots and weights.  Its name reads the traced curve, and the spline, kept so no index
/// moves, is no primitive (`Sketch::spline_traced`).  Asked once every membership is in.
pub(super) fn splines_in_space(sk: &mut Sketch, res: &mut Resolver, map: &mut super::SourceMap) {
    for i in 0..sk.splines.len() {
        let s = &sk.splines[i];
        if sk.spline_traced.contains_key(&(i as u32))
            || s.ctrl.iter().all(|&p| sk.plane_of(p as usize).is_some()) {
            continue;
        }
        let n = s.ctrl.len();
        let consts = [s.knots.clone(), s.weights.clone().unwrap_or_else(|| vec![1.0; n])].concat();
        // the variable table: the parameter, the control points' lifts, then the point
        let n_outer = 1 + 3 * n;
        let vars: Vec<String> = (0..n_outer).map(|k| format!("#v{k}")).collect();
        let zero = crate::tape::Tape::compile_wide(
            &crate::expr::Ast::Num(0.0, crate::units::Dim::SCALAR), &vars)
            .expect("a number always compiles");
        let row = crate::traced::Row { kind: crate::traced::RowKind::Spline,
            cols: (0..n_outer as u32 + 3).collect(), consts };
        let seeds = vec![zero.clone(), zero.clone(), zero];
        let Ok(body) = crate::traced::Body::new(1, n_outer, 3 * n, 3, 0, Vec::new(), seeds,
                                                vec![row], Default::default())
        else { continue };
        let (lo, hi) = curve::domain(sk, i);
        let e = EntRef::spline(i);
        let name = map.name_of(e).cloned().unwrap_or_else(|| io::entity_name(e));
        let t = crate::model::TracedE {
            body, args: s.ctrl.iter().map(|&p| EntRef::point(p as usize)).collect(),
            values: Vec::new(), domain: [[lo, hi], [0.0, 0.0]], home: [0.5 * (lo + hi), 0.0],
            name, class: s.class.clone(),
        };
        let traced = EntRef::new(EntKind::Traced, sk.traced.len());
        sk.traced.push(t);
        sk.spline_traced.insert(i as u32, traced.i() as u32);
        for x in res.of.values_mut().filter(|x| **x == e) {
            *x = traced;
        }
        map.moved(e, traced);
    }
}

/// **An arc over points in space is drawn in the plane through them** (#363 S6): `k := arc(center:
/// c, start: s, end: e)` with a point no plane reaches mints that plane — its origin at `c`, its
/// `u` toward `s`, its normal `(s − c) × (e − c)`, so the arc is the short way round, less than a
/// half turn — and draws the arc there over points of its own: its centre the plane's origin, its
/// start on the plane's `u` (a gauge, the plane's turn in itself), its end free, each tied to the
/// point in space it is (`Coincident3`, intrinsic), and the plane's axes held square.  The points
/// written stay where they stand — held in space, or anywhere — and the arc is an arc: its radius,
/// its contacts, a chain's joints and a family along it are an arc's in a plane.  Three points in
/// a line take no one plane, and are refused (E060).  Asked with the splines, before E060 is.
///
/// **An arc drawn in a plane said to turn clockwise**, `arc(center: c, sense: cw)` (#363 S4), is
/// drawn the same way in that plane turned over — its normal the plane's reversed — so it runs
/// counter-clockwise there from its start: a chain in a plane enters every link at its start, and
/// turns right at such an arc.  Its ends a half turn apart take no one plane either (E060); one
/// over points in space turns as its points stand, and `sense:` there is refused (E040).
///
/// **An arc a chain walks from its end** (`back`, `walked_back`) is drawn from there: its start
/// and end swapped and its turn reversed — in a plane, counter-clockwise becomes clockwise and
/// the other way; in space the plane through its points faces the other way — the same arc, so
/// the chain enters it at its start.
pub(super) fn arcs_in_space(sk: &mut Sketch, map: &super::SourceMap, deferred: &[Deferred],
                            back: &BTreeSet<usize>, diags: &mut Vec<Diag>) {
    use crate::constraints::{CKind, Constraint};
    use crate::space::{cross, dot, norm, sub};
    let clockwise: BTreeMap<usize, (Span, StmtId)> = deferred.iter().filter_map(|d| match d {
        Deferred::Clockwise { arc, span, stmt } => Some((*arc, (*span, *stmt))),
        _ => None,
    }).collect();
    for i in 0..sk.arcs.len() {
        let walked_back = back.contains(&i);
        if walked_back {
            let a = &mut sk.arcs[i];
            std::mem::swap(&mut a.start, &mut a.end);
            sk.walked_back.insert(i as u32);
        }
        let a = sk.arcs[i].clone();
        let pts = [a.center, a.start, a.end].map(|p| p as usize);
        let planar = pts.iter().all(|&p| sk.plane_of(p).is_some());
        let cw = clockwise.get(&i);
        if let (false, Some(&(span, stmt))) = (planar, cw) {
            diags.push(Diag::within(Code::E040, span, stmt, "an arc over points in space turns \
                the way its points stand, the short way round: `sense:` says which way an arc \
                drawn in a plane turns".to_string()));
            continue;
        }
        // walked from its end, an arc in a plane turns the other way from there
        if planar && cw.is_some() == walked_back {
            continue;
        }
        let arc = EntRef::arc(i);
        let [c, s, e] = pts.map(|p| sk.world_point(p));
        let (to_s, to_e) = (sub(s, c), sub(e, c));
        // in space the normal its points make; clockwise in a plane, the plane's reversed
        let n = match sk.plane_of(pts[0]).filter(|_| planar) {
            Some(pl) => sk.basis(pl).normal().map(|x| -x),
            None => cross(to_s, to_e),
        };
        if !(norm(cross(to_s, to_e)) > 1e-9 * norm(to_s) * norm(to_e)) {
            let Some(site) = map.site_of(arc) else { continue };
            let said = if planar {
                "an arc turning clockwise — said to, or walked by a chain from its end — is drawn \
                 in its plane turned over, through its centre and its ends, and these stand in a \
                 line — a half turn, either way round: draw two arcs"
            } else {
                "an arc over points in space is drawn in the plane through its centre and its \
                 ends, and these stand in a line, which no one plane is through: draw it `in` one"
            };
            diags.push(Diag::within(Code::E060, site.span, site.stmt, said.to_string()));
            continue;
        }
        let name = map.name_of(arc).cloned().unwrap_or_else(|| io::entity_name(arc));
        let u = to_s.map(|x| x / norm(to_s));
        let w = cross(n, to_s);
        let v = w.map(|x| x / norm(w));
        let (ua, va, pi, origin) = round_plane(sk, &name, [u, v], c);
        let start = sk.point(norm(to_s), 0.0, false, &format!("{name}#start"));
        sk.params[sk.points[start].y as usize].fixed = true;
        let end = sk.point(dot(to_e, u), dot(to_e, v), false, &format!("{name}#end"));
        for p in [start, end] {
            sk.set_plane(p, Some(pi));
        }
        sk.arcs[i].center = origin as u32;
        sk.arcs[i].start = start as u32;
        sk.arcs[i].end = end as u32;
        let r = sk.arcs[i].radius as usize;
        sk.params[r].value = norm(to_s);
        // the arc's own rows, its ends on it, over its new ends
        for c in sk.constraints.iter_mut().filter(|c| c.intrinsic && c.kind == CKind::PointOnCircle
            && c.args[1].ent() == arc) {
            let at = c.args[0].ent().i();
            let to = match at {
                x if x == a.start as usize => start,
                x if x == a.end as usize => end,
                _ => continue,
            };
            c.args[0] = crate::constraints::Arg::Ent(EntRef::point(to));
        }
        let ties = [(origin, pts[0]), (start, pts[1]), (end, pts[2])];
        for &(p, q) in &ties {
            sk.stands_for.insert(p as u32, q as u32);
        }
        let mut rows: Vec<Constraint> = ties.iter().map(|&(p, q)| coincident3(p, q)).collect();
        rows.push(Constraint::two_line(CKind::Perpendicular3, EntRef::new(EntKind::Axis, ua),
                                       EntRef::new(EntKind::Axis, va)));
        add_intrinsic(sk, rows);
    }
}

/// **The arcs a chain walks from their end** (#363 S4): a chain enters each link where the one
/// before it leaves, and an arc it enters at its end — its end shared with the link before, or,
/// first, its start with the link after — it walks from there to its start.  Each such arc, for
/// `arcs_in_space` to draw from its end.  An arc two chains walk opposite ways is refused (E080):
/// it is drawn one way.  A chain whose links do not meet is left to `solids::faces::
/// validate_chain`, which says so.
pub(super) fn walked_back(sk: &Sketch, res: &Resolver, diags: &mut Vec<Diag>) -> BTreeSet<usize> {
    let (mut back, mut forward) = (BTreeSet::new(), BTreeSet::new());
    for c in res.chains.values() {
        let links: Option<Vec<(EntRef, [u32; 2])>> = c.links.iter().map(|r| {
            let e = res.entity(sk, r).filter(|e| e.i() < sk.count(e.kind))
                .filter(|e| matches!(e.kind, EntKind::Line | EntKind::Arc))?;
            crate::model::edge_ends(sk, e).map(|(s, t)| (e, [s, t]))
        }).collect();
        let Some(links) = links else { continue };
        let meets = |p: u32, k: usize| links.get(k).is_some_and(|(_, ends)| ends.contains(&p));
        let mut entry = None;
        for (r, &(e, [s, t])) in c.links.iter().zip(&links) {
            let reversed = match entry {
                None => e.kind == EntKind::Arc && meets(s, 1) && !meets(t, 1),
                Some(p) if p == s => false,
                Some(p) if p == t && e.kind == EntKind::Arc => true,
                Some(_) => break,
            };
            let (mine, other) = if reversed { (&mut back, &forward) } else { (&mut forward, &back) };
            if e.kind == EntKind::Arc {
                if other.contains(&e.i()) {
                    diags.push(Diag::new(Code::E080, r.span, None, format!("`{}` is walked \
                        from its start in one chain and from its end in another, and an arc is \
                        drawn one way", crate::syntax::ref_text(r))));
                }
                mine.insert(e.i());
            }
            entry = Some(if reversed { s } else { t });
        }
    }
    back.difference(&forward).copied().collect()
}

/// **A circle about a point in space stands square to its `normal:`** (#363 S6): `k := circle(
/// center: o, normal: ax)` with `o` standing in space mints the plane through `o` square to `ax`
/// — an axis, or a line read as one — and draws the circle there about its origin, tied to `o`
/// (`Coincident3`, intrinsic), its axes square to the normal and to each other.  A circle has no
/// turn of its own, so the plane's turn about the normal is a gauge, held square to the world's
/// axis least along the normal where the circle is drawn.  A `normal:` of a circle drawn in a
/// plane is refused, as one naming no direction is.
pub(super) fn circles_in_space(
    sk: &mut Sketch,
    res: &Resolver,
    map: &super::SourceMap,
    deferred: &[Deferred],
    diags: &mut Vec<Diag>,
) {
    use crate::constraints::{CKind, Constraint};
    use crate::space::{cross, norm, sub};
    for d in deferred {
        let Deferred::Normal { circle, normal, span, stmt } = d else { continue };
        let (i, c) = (*circle, sk.circles[*circle].center as usize);
        if sk.plane_of(c).is_some() {
            diags.push(Diag::within(Code::E040, *span, *stmt, "`normal:` stands a circle about a \
                point in space square to an axis, and this one's centre is drawn in a plane, \
                which is the plane it is drawn in".to_string()));
            continue;
        }
        let axis = res.entity(sk, normal)
            .filter(|e| matches!(e.kind, EntKind::Axis | EntKind::Line));
        let dir = axis.map(|e| match e.kind {
            EntKind::Axis => sk.axes[e.i()].d.map(|p| sk.params[p as usize].value),
            _ => {
                let l = &sk.lines[e.i()];
                sub(sk.world_point(l.p2 as usize), sk.world_point(l.p1 as usize))
            }
        });
        let (Some(axis), Some(dir)) = (axis, dir.filter(|d| norm(*d) > 0.0)) else {
            diags.push(Diag::within(Code::E103, normal.span, *stmt, format!("`normal:` names the \
                axis a circle stands square to, an axis or a line, and `{}` is none",
                crate::syntax::ref_text(normal))));
            continue;
        };
        let n = dir.map(|x| x / norm(dir));
        let k = (0..3).min_by(|&a, &b| n[a].abs().total_cmp(&n[b].abs())).unwrap_or(0);
        let mut e = [0.0; 3];
        e[k] = 1.0;
        let w = cross(n, e);
        let u = w.map(|x| x / norm(w));
        let v = cross(n, u);
        let at = sk.world_point(c);
        let name = map.name_of(EntRef::circle(i)).cloned()
            .unwrap_or_else(|| io::entity_name(EntRef::circle(i)));
        let (ua, va, _, origin) = round_plane(sk, &name, [u, v], at);
        // the gauge: the world's axis least along the normal, held, and taken again where a
        // solve leaves the normal near it (`Sketch::retake_round_gauges`)
        let gauge = sk.axis(e, &format!("{name}#gauge"));
        for &p in &sk.axes[gauge].d.clone() {
            sk.params[p as usize].fixed = true;
        }
        sk.round_gauges.insert(gauge as u32, [ua as u32, va as u32]);
        sk.circles[i].center = origin as u32;
        sk.stands_for.insert(origin as u32, c as u32);
        let ax = |a: usize| EntRef::new(EntKind::Axis, a);
        add_intrinsic(sk, [coincident3(origin, c),
            Constraint::two_line(CKind::Perpendicular3, ax(ua), axis),
            Constraint::two_line(CKind::Perpendicular3, ax(va), axis),
            Constraint::two_line(CKind::Perpendicular3, ax(ua), ax(va)),
            Constraint::two_line(CKind::Perpendicular3, ax(ua), ax(gauge))]);
    }
}

/// The plane a round over points in space is drawn in (#363 S6): its axes along `u` and `v`,
/// through `at`, named under the round's `name` — its axes, the plane and its origin.
fn round_plane(sk: &mut Sketch, name: &str, [u, v]: [[f64; 3]; 2], at: [f64; 3])
    -> (usize, usize, usize, usize) {
    let ua = sk.axis(u, &format!("{name}#plane.u"));
    let va = sk.axis(v, &format!("{name}#plane.v"));
    let pi = sk.plane(ua, va, at, &format!("{name}#plane"));
    sk.set_plane_origin(pi, at);
    (ua, va, pi, sk.frames[pi].origin as usize)
}

/// Points `p` and `q` one point in space.
fn coincident3(p: usize, q: usize) -> crate::constraints::Constraint {
    let mut c = crate::constraints::Constraint::coincident(EntRef::point(p), EntRef::point(q));
    c.kind = crate::constraints::CKind::Coincident3;
    c
}

/// Each of `rows` added as the drawing's own statement, no relation of the document's.
fn add_intrinsic(sk: &mut Sketch, rows: impl IntoIterator<Item = crate::constraints::Constraint>) {
    for mut c in rows {
        c.intrinsic = true;
        sk.add(c);
    }
}

/// A circle, an arc and a spline are drawn in a plane: one over a point standing in space has
/// none to be drawn in, and is refused at its declaration (E060).  Asked once every membership
/// is in, so a point an `in` reaches later is in its plane by then.
pub(super) fn drawn_in_planes(sk: &Sketch, map: &super::SourceMap, diags: &mut Vec<Diag>) {
    let curves = (0..sk.circles.len()).map(EntRef::circle)
        .chain((0..sk.arcs.len()).map(EntRef::arc))
        .chain((0..sk.splines.len()).filter(|&i| !sk.spline_traced.contains_key(&(i as u32)))
            .map(EntRef::spline))
        .chain((0..sk.curves.len()).filter(|&i| sk.curve_extremal(i)).map(|i| EntRef::new(EntKind::Curve, i)));
    for e in curves {
        let Some(&p) = sk.children(e).iter().find(|k| sk.plane_of(k.i()).is_none()) else { continue };
        let Some(site) = map.site_of(e) else { continue };
        let who = |r: EntRef| map.name_of(r).cloned().unwrap_or_else(|| io::entity_name(r));
        diags.push(Diag::within(
            Code::E060,
            site.span,
            site.stmt,
            format!(
                "{} is drawn in a plane, and `{}` stands in space: draw it `in` one{}",
                match e.kind {
                    EntKind::Arc => "an arc",
                    EntKind::Circle => "a circle",
                    EntKind::Curve => "a free curve",
                    _ => "a spline",
                },
                who(p),
                if e.kind == EntKind::Circle {
                    ", or stand it square to an axis, `circle(center: o, normal: ax)`"
                } else {
                    ""
                }
            ),
        ));
    }
}

/// The hidden axes planes were written over drawn lines with, each now seeded along its line as
/// the line stands — its points have their places — and held to it, intrinsically: the plane's
/// own statement, never a relation of the document's.  A hidden axis is the line: parallel to
/// it, and through one of its ends, the one it shares with the plane's other line where the two
/// meet at an end (`std.Turned`'s two lines from `o`), else its start.  A plane whose two axes
/// stand on one point stands there: its origin is that point, three rows in place of the four
/// `PlaneAxis` rows, which would say one thing twice where no count of columns can see it.
pub(super) fn axes_along(sk: &mut Sketch, deferred: &[Deferred]) {
    use crate::constraints::{Arg, CKind, Constraint};
    let along: BTreeMap<usize, usize> = deferred.iter()
        .filter_map(|d| match *d { Deferred::Along { axis, line } => Some((axis, line)), _ => None })
        .collect();
    if along.is_empty() {
        return;
    }
    let ends = |sk: &Sketch, l: usize| [sk.lines[l].p1 as usize, sk.lines[l].p2 as usize];
    for p in sk.charted() {
        let (u, v) = (sk.frames[p].u as usize, sk.frames[p].v as usize);
        let (lu, lv) = (along.get(&u).copied(), along.get(&v).copied());
        // the end the two lines share, if they meet at one: each line's own image of it, which
        // for a point drawn in two planes may be its twin (§6.7)
        let shared = match (lu, lv) {
            (Some(a), Some(b)) => ends(sk, a).into_iter().find_map(|x| {
                ends(sk, b).into_iter().find(|&y| sk.twinned(x, y)).map(|y| [x, y])
            }),
            _ => None,
        };
        for (k, (axis, line)) in [(u, lu), (v, lv)].into_iter().enumerate() {
            let Some(line) = line else { continue };
            sk.turn_axis_along(axis, line);
            let mut c = Constraint::two_line(
                CKind::Parallel3,
                EntRef::new(EntKind::Axis, axis),
                EntRef::line(line),
            );
            c.intrinsic = true;
            sk.add(c);
            let at = shared.map_or(sk.lines[line].p1 as usize, |ends| ends[k]);
            let x = sk.world_point(at);
            sk.stand_axis_through(axis, x);
            let mut c = Constraint::new(
                CKind::PointOnAxis,
                vec![Arg::Ent(EntRef::point(at)), Arg::Ent(EntRef::new(EntKind::Axis, axis))],
            );
            c.intrinsic = true;
            sk.add(c);
        }
        // both axes on one point, which is drawn elsewhere: the plane stands at it
        let origin = sk.frames[p].origin as usize;
        let Some([at, _]) = shared
            .filter(|ends| ends.iter().all(|&x| sk.points[x].plane != Some(p as u32)))
        else {
            continue;
        };
        let rows: Vec<u32> = sk.constraints.iter()
            // its normal's stays: it stands through the origin wherever that is
            .filter(|c| c.intrinsic && c.kind == CKind::PlaneAxis && c.args[0].ent() == EntRef::frame(p)
                && [u, v].contains(&c.args[1].ent().i()))
            .map(|c| c.id)
            .collect();
        for id in rows {
            sk.remove(id);
        }
        let x = sk.world_point(at);
        for k in 0..3 {
            let q = sk.frames[p].o[k] as usize;
            sk.params[q].value = x[k];
        }
        let mut c = Constraint::new(
            CKind::Coincident3,
            vec![Arg::Ent(EntRef::point(origin)), Arg::Ent(EntRef::point(at))],
        );
        c.intrinsic = true;
        sk.add(c);
    }
}

/// Defer geometric seed expressions until all declarations have initial values.
/// These initialize parameters without adding constraints.
pub(super) enum Deferred {
    Text { param: u32, text: String, names: Vec<(String, String)>, span: Span, stmt: StmtId },
    /// A plane written over a drawn line: the hidden axis its `u` or `v` is, held parallel to the
    /// line once the line's points have their places (`axes_along`).
    Along { axis: usize, line: usize },
    /// A point's `hint(z: …)`: its height in space, once memberships say it is in space
    /// (`places`), and refused where they say it is drawn in a plane.  A text reading geometry
    /// (`z: q.z + 1`) is settled with the other seed texts, once the point has its `z`.
    Height {
        point: usize,
        z: f64,
        text: Option<String>,
        names: Vec<(String, String)>,
        span: Span,
        stmt: StmtId,
    },
    At { point: usize, at: AtRef, names: Vec<(String, String)>, span: Span, stmt: StmtId },
    /// A plane's axis seeded along a direction read off geometry, `hint(dir: up)`.
    Dir { axis: usize, dir: DirRef, names: Vec<(String, String)>, span: Span, stmt: StmtId },
    /// An arc's radius the source left unwritten: its centre to its start, as `Sketch::arc`
    /// first computed it.
    Radius { arc: usize },
    /// A point the source wrote no seed for, started where a minted child is (`scatter`): one a
    /// step may place instead (`walk_seeds`, #330).
    Scattered { point: usize },
    /// A circle's `normal:` (#363 S6): the axis it stands square to, read where its centre
    /// stands in space (`circles_in_space`), once every axis is built.
    Normal { circle: usize, normal: crate::syntax::Ref, span: Span, stmt: StmtId },
    /// An arc said to turn clockwise from its start, `sense: cw` (#363 S4): drawn in its plane
    /// turned over (`arcs_in_space`).
    Clockwise { arc: usize, span: Span, stmt: StmtId },
}

/// The seed a dotted name reads: `pin.x`, `k.center.y`, `base.r`, `e.b` — `dotted` as the
/// flattener resolved it, its root the entity's absolute name, and the last segment the scalar by
/// the kind's own `scalar_names`.
fn seed_read(sk: &Sketch, res: &Resolver, dotted: &str) -> Result<(f64, crate::units::Dim), String> {
    let (path, scalar) =
        dotted.rsplit_once('.').ok_or_else(|| format!("`{dotted}` is not a number here"))?;
    let segs: Vec<&str> = path.split('.').collect();
    let (e, fields) =
        res.dotted(&segs).ok_or_else(|| format!("no such entity: `{}`", segs[0]))?;
    let e = follow(sk, e, &fields)?;
    let names = sk
        .scalar_names(e, path)
        .ok_or_else(|| format!("{} has no scalar to read by name", e.kind.a()))?;
    let at = names.iter().position(|n| *n == dotted).ok_or_else(|| {
        format!(
            "{} has no `{scalar}`; its scalars are {}",
            e.kind.a(),
            names.iter().map(|n| format!("`{n}`")).collect::<Vec<_>>().join(", ")
        )
    })?;
    let p = *sk.entity_params(e).get(at).ok_or_else(|| format!("`{dotted}` has no seed yet"))?;
    Ok((sk.params[p as usize].value, crate::units::Dim::LENGTH))
}

/// An expression over geometry's seeds, come to its number.  `names` is what each dotted name
/// in it resolved to (`Decl::seed_names`); one it does not list is read as written.
fn seed_eval(
    sk: &Sketch,
    res: &Resolver,
    text: &str,
    names: &[(String, String)],
) -> Result<f64, String> {
    let p = expr::parse_in(text, sk.units)?;
    let mut env: BTreeMap<String, expr::Aff> = BTreeMap::new();
    for dep in p.body.deps() {
        if dep.contains('.') {
            let abs = names.iter().find(|(w, _)| *w == dep).map(|(_, a)| a.as_str());
            let (v, dim) = seed_read(sk, res, abs.unwrap_or(&dep))?;
            env.insert(dep.clone(), expr::Aff::of_dim(v, dim));
        }
    }
    let a = expr::eval(&p.body, &env)?;
    match a.number() {
        Some(v) if v.is_finite() => Ok(v),
        Some(v) => Err(format!("comes to {v}")),
        None => Err(format!(
            "`{}` is not a number here — a component's parameters are, and the document's \
             dimensions are not",
            a.free.unwrap_or_default()
        )),
    }
}

/// Where a seed is written: a parameter a `fix` holds or a bound carried across is left where it
/// stands, and a ring's turned copy (§12.3) is its representative's place turned, never its own
/// seed — so each write turns the copies over what it moved at once, and a seed read later in
/// statement order reads a copy where the solve will put it (#225).
struct Seeds<'h> {
    held: &'h BTreeSet<u32>,
    derived: Vec<crate::model::Derivation>,
    /// The derivations over each base parameter, by index into `derived`.
    over: BTreeMap<u32, Vec<usize>>,
    /// Every derived parameter: a copy's own.
    copies: BTreeSet<u32>,
}

impl<'h> Seeds<'h> {
    fn new(sk: &Sketch, held: &'h BTreeSet<u32>) -> Self {
        let derived = sk.derived();
        let mut over: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
        for (i, d) in derived.iter().enumerate() {
            for &(b, _) in &d.terms {
                over.entry(b).or_default().push(i);
            }
        }
        let copies = derived.iter().map(|d| d.param).collect();
        Seeds { held, derived, over, copies }
    }

    fn write(&self, sk: &mut Sketch, p: u32, v: f64) {
        if sk.params[p as usize].fixed || self.held.contains(&p) || self.copies.contains(&p) {
            return;
        }
        sk.params[p as usize].value = v;
        for &i in self.over.get(&p).into_iter().flatten() {
            sk.settle_derivation(&self.derived[i]);
        }
    }
}

/// **Seeds that follow a walk** (#330): a corner a step places, and that the source wrote no seed
/// for, starts where the step says — from where the line's start stands, or from the frame's
/// origin — worked out in statement order, which along a chain is the walk's.  So a chain of
/// steps starts as the outline it states and its seeds follow its numbers, with nothing restated
/// in a `hint`.  Twice round, so a closed walk whose first corner its last step places starts
/// there.  Only a step in the view the line is drawn in is walked: one measured in a frame of its
/// own is read in space, and its corner left where it was minted.
pub(super) fn walk_seeds(sk: &mut Sketch, deferred: &[Deferred], held: &BTreeSet<u32>) {
    use crate::constraints::{step_word, Arg, CKind, OrdinateForm as F};
    let steps: Vec<usize> = (0..sk.constraints.len())
        .filter(|&i| {
            let c = &sk.constraints[i];
            !c.claim && c.word.as_ref().is_some_and(|w| step_word(&w.word))
        })
        .collect();
    if steps.is_empty() {
        return;
    }
    let scattered: BTreeSet<usize> = deferred
        .iter()
        .filter_map(|d| match *d {
            Deferred::Scattered { point } => Some(point),
            _ => None,
        })
        .collect();
    let seeds = Seeds::new(sk, held);
    for _ in 0..2 {
        for &i in &steps {
            let c = &sk.constraints[i];
            let (Some(Arg::Ent(p)), Some(Arg::Ent(q))) = (c.args.first(), c.args.get(1)) else {
                continue;
            };
            let (p, q) = (p.i(), q.i());
            if !scattered.contains(&q) {
                continue;
            }
            let form = match c.kind.form_slot().and_then(|k| c.args.get(k)) {
                Some(Arg::Int(f)) => F::from_int(*f),
                _ => continue,
            };
            let at = sk.point_xy(p);
            // a number the flattener left naming something reads an unknown, which has no place
            // to start from yet: walked at its provisional zero, the line would start collapsed
            // onto its own start, a stationary point of every direction read off it
            let unknown = |t: &str| {
                crate::expr::parse_in(t, sk.units).map_or(true, |p| !p.body.deps().is_empty())
            };
            // an ordinate's number, read only of an ordinate: a level's slot there is its word
            let number = || match c.args.get(3) {
                Some(Arg::Expr(x)) if unknown(&x.text) => None,
                a => a.map(|a| a.value(sk)),
            };
            let (axis, value) = match (c.kind, form) {
                (CKind::Ordinate, F::PageU) => (0, number().map(|d| at.0 + d)),
                (CKind::Ordinate, F::PageV) => (1, number().map(|d| at.1 + d)),
                (CKind::Ordinate, F::CoordU) => (0, number()),
                (CKind::Ordinate, F::CoordV) => (1, number()),
                (CKind::Level, F::PageU | F::CoordU) => (0, Some(at.0)),
                (CKind::Level, F::PageV | F::CoordV) => (1, Some(at.1)),
                _ => continue,
            };
            let (Some(v), Some(&param)) =
                (value, sk.entity_params(EntRef::point(q)).get(axis))
            else {
                continue;
            };
            seeds.write(sk, param, v);
        }
    }
}

/// Seeds that read geometry, worked out in statement order — never over a number a `fix` holds,
/// which was applied first.
/// `held`: the parameters left where they stand — a point a bound carried across (§9.6), whose
/// own seed would put it back.
pub(super) fn settle_deferred(
    sk: &mut Sketch,
    res: &Resolver,
    deferred: &[Deferred],
    held: &BTreeSet<u32>,
    diags: &mut Vec<Diag>,
) {
    let seeds = Seeds::new(sk, held);
    for d in deferred {
        let (span, stmt, result) = match d {
            Deferred::Text { param, text, names, span, stmt } => {
                let r = seed_eval(sk, res, text, names).map(|v| seeds.write(sk, *param, v));
                (*span, *stmt, r.map_err(|e| format!("`{text}`: {e}")))
            }
            Deferred::At { point, at, names, span, stmt } => {
                let r = place_of(sk, res, *point, at, names).map(|place| {
                    for (p, v) in sk.point_all_params(*point).into_iter().zip(place) {
                        seeds.write(sk, p, v);
                    }
                });
                (*span, *stmt, r)
            }
            Deferred::Dir { axis, dir, names, span, stmt } => {
                let r = direction_of(sk, res, dir, names).map(|d| {
                    for (p, v) in sk.axes[*axis].d.into_iter().zip(d) {
                        seeds.write(sk, p, v);
                    }
                });
                (*span, *stmt, r)
            }
            Deferred::Height { point, text: Some(text), names, span, stmt, .. } => {
                // refused already where the point is drawn in a plane (`places`)
                let Some(param) = sk.points[*point].z else { continue };
                let r = seed_eval(sk, res, text, names).map(|v| seeds.write(sk, param, v));
                (*span, *stmt, r.map_err(|e| format!("`{text}`: {e}")))
            }
            Deferred::Along { .. } | Deferred::Height { .. } | Deferred::Scattered { .. }
            | Deferred::Normal { .. } | Deferred::Clockwise { .. } => {
                continue
            }
            Deferred::Radius { arc } => {
                let a = &sk.arcs[*arc];
                let ((cx, cy), (sx, sy)) =
                    (sk.point_xy(a.center as usize), sk.point_xy(a.start as usize));
                let r = (sx - cx).dhypot(sy - cy);
                seeds.write(sk, a.radius, if r.abs() > 1e-9 { r } else { UNSEEDED_RADIUS });
                continue;
            }
        };
        if let Err(message) = result {
            diags.push(Diag::within(Code::E103, span, stmt, message));
        }
    }
}

/// Where `hint(at: …)` puts `point` on the sheet: the seed of the point it names, the edge of
/// the circle it names at the bearing given, or a step from the point it names — `by:` (1 if
/// unsaid) of the way toward another or of a line's run — turned `turn:` about it.  Each place
/// is read in `point`'s view (`seed_in`); `at_seed` is the traced counterpart.  The numbers
/// are `point`'s own: two in its plane, three for a point in space.
fn place_of(
    sk: &Sketch,
    res: &Resolver,
    point: usize,
    a: &AtRef,
    names: &[(String, String)],
) -> Result<Vec<f64>, String> {
    if sk.points[point].z.is_none() {
        return place_in(sk, res, sk.plane_of(point), a, names).map(|(x, y)| vec![x, y]);
    }
    let w = place_in_space(sk, res, a, names)?;
    // a point drawn in a spatial frame's chart has its numbers along its axes (#383)
    Ok(match sk.points[point].chart {
        Some(f) => sk.chart_coords(f as usize, [w[0], w[1], w[2]]).to_vec(),
        None => w,
    })
}

/// Where `hint(at: …)` puts a point in space, in the world's coordinates.
fn place_in_space(sk: &Sketch, res: &Resolver, a: &AtRef, names: &[(String, String)])
    -> Result<Vec<f64>, String> {
    // a place in a spatial frame's coordinates, three of them, read in space (#383)
    let e = place(sk, res, &a.what)?;
    if e.kind == EntKind::Frame && !sk.frames[e.i()].planar() {
        if sk.frames[e.i()].chart.is_none() {
            return Err("a place in a frame is read along its axes, and this frame has none: \
                        name `f.u`, `f.v` or `f.origin` where they are placed".to_string());
        }
        let number = |t: &Option<(String, Span)>| step_by(sk, res, t, 0.0, names);
        let p = [number(&a.x)?, number(&a.y)?, number(&a.z)?];
        return Ok(sk.chart_lift(e.i(), p).to_vec());
    }
    // a point in space: the place is read in the plane it is drawn in and lifted from there,
    // or, where it stands in space itself, read where it stands
    let e = place(sk, res, &a.what)?;
    // a step from a point, with no turn, is the same step in space wherever its point is drawn:
    // read there, so a step along a line square to that plane goes somewhere (#214: up a
    // cutter's axis from a point of the pitch plane)
    let plain = a.turn.is_none() && a.bearing.is_none() && a.x.is_none() && a.y.is_none()
        && a.z.is_none();
    if e.kind == EntKind::Point && plain {
        if let Some(step) = step_ends(sk, res, e, a)? {
            let f = step_by(sk, res, &a.by, 1.0, names)?;
            let (at, d) = (sk.world_point(e.i()), step.world(sk));
            return Ok((0..3).map(|k| at[k] + f * d[k]).collect());
        }
    }
    let home = match e.kind {
        EntKind::Frame => Some(e.i()),
        EntKind::Circle => sk.plane_of(sk.circles[e.i()].center as usize),
        _ => sk.plane_of(e.i()),
    };
    if home.is_some() || e.kind != EntKind::Point {
        return place_in(sk, res, home, a, names).map(|q| sk.world_in(home, q).to_vec());
    }
    if !plain {
        return Err("a turn or a bearing is taken in a plane, and this place stands in space"
            .to_string());
    }
    Ok(sk.world_point(e.i()).to_vec())
}

/// Which way `hint(dir: …)` points, in space and of unit length: an axis's own seed, a line's
/// from its `p1` to its `p2` where each end stands, a plane's normal (`P.normal`, or `P.n`) —
/// turned `turn:` about a second of these, counter-clockwise looking down it (Rodrigues).
fn direction_of(
    sk: &Sketch,
    res: &Resolver,
    d: &DirRef,
    names: &[(String, String)],
) -> Result<[f64; 3], String> {
    use crate::space::{add, cross, dot, norm, scale};
    let read = |r: &crate::syntax::Ref| -> Result<[f64; 3], String> {
        // a plane stands for its normal by the word `n`, as an ordinate reads one, or by its
        // member `normal` — read off its axes either way, as they are seeded now
        let normal = matches!(r.path.last(), Some(Seg::Field(f)) if f.text == "n" || f.text == NORMAL);
        let e = if normal {
            let root = *res.of.get(&r.root.text)
                .ok_or_else(|| format!("no such entity: `{}`", r.root.text))?;
            follow(sk, root, &r.path[..r.path.len() - 1])?
        } else {
            place(sk, res, r)?
        };
        let v = match e.kind {
            EntKind::Frame if normal => sk.basis(e.i()).normal(),
            EntKind::Axis => sk.axes[e.i()].d.map(|p| sk.params[p as usize].value),
            EntKind::Line => {
                let l = &sk.lines[e.i()];
                crate::space::sub(sk.world_point(l.p2 as usize), sk.world_point(l.p1 as usize))
            }
            k => {
                return Err(format!(
                    "a direction is an axis's, a line's or a plane's normal (`P.normal`), not {}'s",
                    k.a()
                ));
            }
        };
        let n = norm(v);
        if n.is_nan() || n <= 1e-12 {
            return Err(format!("`{}` points nowhere yet", r.root.text));
        }
        Ok(scale(v, 1.0 / n))
    };
    let v = read(&d.what)?;
    let (Some(about), Some(turn)) = (&d.about, &d.turn) else { return Ok(v) };
    let k = read(about)?;
    // a turn is an angle, and what it comes to is in degrees, as a place's is
    let (s, c) = step_by(sk, res, &Some(turn.clone()), 0.0, names)?.to_radians().dsin_cos();
    let turned = add(add(scale(v, c), scale(cross(k, v), s)), scale(k, dot(k, v) * (1.0 - c)));
    Ok(scale(turned, 1.0 / norm(turned)))
}

/// The step a place takes: between two points (toward one, or a line's run), or along a path
/// as it runs where the place's point meets it, in space.
enum Step {
    Between(usize, usize),
    Along([f64; 3]),
}

impl Step {
    /// The step in space.
    fn world(&self, sk: &Sketch) -> [f64; 3] {
        match *self {
            Step::Between(from, to) => crate::space::sub(sk.world_point(to), sk.world_point(from)),
            Step::Along(d) => d,
        }
    }

    /// The step on `view`'s page, taken from the point `at`.
    fn in_view(&self, sk: &Sketch, at: usize, view: Option<usize>) -> (f64, f64) {
        match *self {
            Step::Between(from, to) => {
                let ((fx, fy), (tx, ty)) = (seed_in(sk, from, view), seed_in(sk, to, view));
                (tx - fx, ty - fy)
            }
            Step::Along(d) => {
                let w = sk.world_point(at);
                let a = sk.on_view_sheet(w, view);
                let b = sk.on_view_sheet(crate::space::add(w, d), view);
                (b.0 - a.0, b.1 - a.1)
            }
        }
    }
}

/// The step a place takes from `e`: toward a point (`toward:`), a line's run (`along:` a line),
/// an axis's direction (`along:` an axis, a unit a step), or along any other path (`along:` an
/// arc, a circle, a spline, a curve, #243 L4, or a curve traced in space, #363 S1) — its rate along its own run where `e` meets it (the foot of `e` on
/// it, within its run), as a line's run is its rate from `p1` to `p2`: an arc's turned by its
/// sweep and a circle's by a whole turn, counter-clockwise, a spline's and a curve's by its
/// parameter, a traced curve's by its formal.  `None` where it takes no step.
fn step_ends(sk: &Sketch, res: &Resolver, e: EntRef, a: &AtRef)
    -> Result<Option<Step>, String> {
    Ok(match (&a.toward, &a.along) {
        (Some(t), _) => {
            let b = place(sk, res, t)?;
            if b.kind != EntKind::Point {
                return Err(format!("a step is taken toward a point, not {}", b.kind.a()));
            }
            Some(Step::Between(e.i(), b.i()))
        }
        (None, Some(l)) => {
            let l = place(sk, res, l)?;
            match l.kind {
                EntKind::Line => {
                    Some(Step::Between(sk.lines[l.i()].p1 as usize, sk.lines[l.i()].p2 as usize))
                }
                k if matches!(k, EntKind::Arc | EntKind::Circle | EntKind::Spline | EntKind::Curve)
                    || super::relations::traced_curve(sk, l) => {
                    if e.kind != EntKind::Point {
                        return Err("a step along a path is taken from a point".to_string());
                    }
                    Some(Step::Along(rate_along(sk, l, sk.world_point(e.i()))?))
                }
                // along an axis, its direction: a plane's normal, a frame's axis (#366)
                EntKind::Axis => {
                    let d = sk.axes[l.i()].d.map(|q| sk.params[q as usize].value);
                    Some(Step::Along(crate::space::normalised(d)
                        .ok_or("the axis has no direction yet")?))
                }
                _ => {
                    return Err(format!("a step is taken along a line, not {}, or along a path",
                                       super::relations::said_kind(l)))
                }
            }
        }
        (None, None) => None,
    })
}

/// How path `g` runs where the point `w` (in space) meets it: its rate along its own run at the
/// foot of `w` on it, in space (see `step_ends`) — a traced curve's `d[0]` at its nearest place.
fn rate_along(sk: &Sketch, g: EntRef, w: [f64; 3]) -> Result<[f64; 3], String> {
    if g.kind == EntKind::Traced {
        use crate::traced::{place_of, reading, Need};
        let at = place_of(sk, g.i(), w).ok_or("the curve has no points yet")?;
        let d = reading(sk, g.i(), at, Need::Tangent).ok_or("the curve has no run there")?.d[0];
        if !d.iter().all(|v| v.is_finite()) || crate::space::norm(d) <= 1e-12 {
            return Err("the path does not run there yet".to_string());
        }
        return Ok(d);
    }
    let view = super::planes::drawn_plane(sk, g);
    let basis = view.map_or(crate::plane::Basis::page(), |p| sk.basis(p));
    let (x, y) = basis.view_coords(w);
    let lift = |(dx, dy): (f64, f64)| crate::space::sub(basis.lift(dx, dy), basis.lift(0.0, 0.0));
    let round = |c: usize, r: f64, (lo, hi): (f64, f64)| {
        let (cx, cy) = sk.point_xy(c);
        let th = (y - cy).datan2(x - cx);
        // the foot within the run: the angle turned into it, then held to its ends
        let mid = 0.5 * (lo + hi);
        let th = (th - mid + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
            - std::f64::consts::PI + mid;
        let th = th.clamp(lo.min(hi), lo.max(hi));
        let (sn, cs) = th.dsin_cos();
        lift(((hi - lo) * -r * sn, (hi - lo) * r * cs))
    };
    let d = match g.kind {
        EntKind::Arc => {
            let arc = &sk.arcs[g.i()];
            round(arc.center as usize, sk.radius_value(g).abs(), sk.arc_angles(g.i()))
        }
        EntKind::Circle => {
            let c = &sk.circles[g.i()];
            round(c.center as usize, sk.radius_value(g).abs(), (0.0, std::f64::consts::TAU))
        }
        EntKind::Spline => {
            let t = crate::curve::closest(sk, g.i(), x, y).0;
            lift(crate::curve::eval(sk, g.i(), t).d1)
        }
        _ => {
            let pts = sk.curve_polyline_within(g.i(), 1e-3);
            // the nearest of its points, and the chord across its neighbours by parameter
            let off = |i: usize| { let (_, (px, py)) = pts[i]; (px - x).dhypot(py - y) };
            let k = (0..pts.len()).min_by(|&i, &j| off(i).total_cmp(&off(j)))
                .ok_or("the curve has no points yet")?;
            let (u0, (x0, y0)) = pts[k.saturating_sub(1)];
            let (u1, (x1, y1)) = pts[(k + 1).min(pts.len() - 1)];
            if u1 == u0 {
                return Err("the curve has no run yet".to_string());
            }
            lift(((x1 - x0) / (u1 - u0), (y1 - y0) / (u1 - u0)))
        }
    };
    if !d.iter().all(|v| v.is_finite()) || crate::space::norm(d) <= 1e-12 {
        return Err("the path does not run there yet".to_string());
    }
    Ok(d)
}

/// A place's number: what the text comes to, or what an unwritten key means.
fn step_by(sk: &Sketch, res: &Resolver, t: &Option<(String, Span)>, unsaid: f64,
           names: &[(String, String)]) -> Result<f64, String> {
    match t {
        Some((text, _)) => seed_eval(sk, res, text, names).map_err(|m| format!("`{text}`: {m}")),
        None => Ok(unsaid),
    }
}

/// `place_of` read in `view`'s coordinates.
fn place_in(
    sk: &Sketch,
    res: &Resolver,
    view: Option<usize>,
    a: &AtRef,
    names: &[(String, String)],
) -> Result<(f64, f64), String> {
    let e = place(sk, res, &a.what)?;
    let number = |t: &Option<(String, Span)>, unsaid: f64| step_by(sk, res, t, unsaid, names);
    // the step a place takes: toward a point, or a line's run
    let step = step_ends(sk, res, e, a)?;
    // a place in a plane's own coordinates, read in space and seen where the point is
    if a.x.is_some() || a.y.is_some() {
        if e.kind != EntKind::Frame {
            return Err(format!(
                "`x:` and `y:` are a place in a plane, and `at:` here names {}",
                e.kind.a()
            ));
        }
        if a.z.is_some() || !sk.frames[e.i()].planar() {
            return Err("a place in a plane is its `x:` and `y:`, and one in a spatial frame \
                        three numbers, seeding a point in space".to_string());
        }
        let (u, v) = (number(&a.x, 0.0)?, number(&a.y, 0.0)?);
        let at = sk.basis(e.i()).lift(u, v);
        return Ok(sk.on_view_sheet(at, view));
    }
    match (e.kind, &a.bearing, step) {
        (EntKind::Point, None, None) => Ok(seed_in(sk, e.i(), view)),
        (EntKind::Point, None, Some(step)) => {
            let (ax, ay) = seed_in(sk, e.i(), view);
            let f = number(&a.by, 1.0)?;
            let (dx, dy) = step.in_view(sk, e.i(), view);
            // a turn is an angle, and what it comes to is in degrees, as a bearing's is
            let (dx, dy) = match &a.turn {
                None => (dx, dy),
                Some(_) => {
                    let (s, c) = number(&a.turn, 0.0)?.to_radians().dsin_cos();
                    (c * dx - s * dy, s * dx + c * dy)
                }
            };
            Ok((ax + f * dx, ay + f * dy))
        }
        (EntKind::Point, Some(_), _) => {
            Err("a point is already a place; a bearing needs a circle".to_string())
        }
        (EntKind::Circle, Some(_), None) => {
            let c = &sk.circles[e.i()];
            let (cx, cy) = seed_in(sk, c.center as usize, view);
            let r = sk.params[c.radius as usize].value;
            // a bearing is an angle, so the text may say `90deg` or read a `param` already
            // written in; what it comes to is in the document's angle unit, which is degrees
            let b = number(&a.bearing, 0.0)?.to_radians();
            Ok((cx + r * b.dcos(), cy + r * b.dsin()))
        }
        (EntKind::Circle, _, Some(_)) => {
            Err("a step starts at a point; a circle's place is its edge at a bearing".to_string())
        }
        (EntKind::Circle, None, None) => {
            Err("where on the edge?  `hint(at: c, bearing: …)` says the bearing".to_string())
        }
        (k, _, _) => Err(format!("a seed cannot be at {}", k.a())),
    }
}

/// The entity a place key names.
fn place(sk: &Sketch, res: &Resolver, r: &crate::syntax::Ref) -> Result<EntRef, String> {
    let e = *res.of.get(&r.root.text).ok_or_else(|| format!("no such entity: `{}`", r.root.text))?;
    follow(sk, e, &r.path)
}

/// Where point `p`'s seed stands in `view`'s page coordinates (`None`: the page): its own seed
/// where it is drawn in that view, and otherwise where it stands in space projected into the
/// view — a point drawn in one plane, read in another, is its image there, which is what
/// `project` says of the pair.  Before memberships are read every point is on the page, so the
/// first settle reads every seed as written.
fn seed_in(sk: &Sketch, p: usize, view: Option<usize>) -> (f64, f64) {
    if sk.plane_of(p) == view {
        return sk.point_xy(p);
    }
    sk.on_view_sheet(sk.world_point(p), view)
}

/// Whether a place reads a point of another view than the point it seeds: then the seeds are
/// settled again once memberships and the views' poses are in (`program::build`).
pub(super) fn crosses_views(sk: &Sketch, res: &Resolver, deferred: &[Deferred]) -> bool {
    deferred.iter().any(|d| {
        if let Deferred::Dir { dir, .. } = d {
            return dir.refs().any(|r| reads_unplaced(sk, res, r));
        }
        let Deferred::At { point, at, .. } = d else { return false };
        let view = sk.plane_of(*point);
        let point_of = |e: EntRef| match e.kind {
            EntKind::Point => Some(e.i()),
            EntKind::Circle => Some(sk.circles[e.i()].center as usize),
            _ => None,
        };
        // a line's ends, and the points any other path a step runs along is drawn through
        let line_ends = |r: &crate::syntax::Ref| match place(sk, res, r) {
            Ok(e) if e.kind == EntKind::Line => {
                vec![sk.lines[e.i()].p1 as usize, sk.lines[e.i()].p2 as usize]
            }
            Ok(e) if matches!(e.kind, EntKind::Arc | EntKind::Circle | EntKind::Spline) => {
                sk.children(e).into_iter().filter(|k| k.kind == EntKind::Point)
                    .map(|k| k.i()).collect()
            }
            _ => vec![],
        };
        // a place in another plane's coordinates is read through that plane's pose
        let in_plane = matches!(place(sk, res, &at.what), Ok(e) if e.kind == EntKind::Frame
            && Some(e.i()) != view);
        in_plane || std::iter::once(&at.what).chain(at.toward.iter())
            .filter_map(|r| place(sk, res, r).ok().and_then(point_of))
            .chain(at.along.iter().flat_map(line_ends))
            .any(|p| sk.plane_of(p) != view)
    })
}

/// Whether a direction read off `r` reads through what the document places: an axis or a
/// plane not held, or a line with an end drawn in such a plane.
fn reads_unplaced(sk: &Sketch, res: &Resolver, r: &crate::syntax::Ref) -> bool {
    let normal = matches!(r.path.last(), Some(Seg::Field(f)) if f.text == "n" || f.text == NORMAL);
    let e = match normal {
        true => res.of.get(&r.root.text).and_then(|&e| follow(sk, e, &r.path[..r.path.len() - 1]).ok()),
        false => place(sk, res, r).ok(),
    };
    let in_unplaced = |p: u32| sk.plane_of(p as usize).is_some_and(|pl| !sk.plane_fixed(pl));
    match e {
        Some(e) if e.kind == EntKind::Frame => !sk.plane_fixed(e.i()),
        Some(e) if e.kind == EntKind::Axis => !super::views::held(sk, &sk.axes[e.i()].d),
        Some(e) if e.kind == EntKind::Line => {
            let l = &sk.lines[e.i()];
            in_unplaced(l.p1) || in_unplaced(l.p2)
        }
        _ => false,
    }
}

fn set_class(sk: &mut Sketch, e: EntRef, c: Classes) {
    match e.kind {
        EntKind::Point => {}
        EntKind::Line => sk.lines[e.i()].class = c,
        EntKind::Curve => sk.curves[e.i()].class = c,
        EntKind::Circle => sk.circles[e.i()].class = c,
        EntKind::Axis => sk.axes[e.i()].class = c,
        EntKind::Arc => sk.arcs[e.i()].class = c,
        EntKind::Spline => sk.splines[e.i()].class = c,
        EntKind::Frame => sk.frames[e.i()].class = c,
        EntKind::Face => sk.faces[e.i()].class = c,
        EntKind::Solid => sk.solids[e.i()].class = c,
        EntKind::Surface => sk.surfaces[e.i()].class = c,
        EntKind::Motion => sk.motions[e.i()].class = c,
        EntKind::Envelope => sk.envelopes[e.i()].class = c,
        EntKind::Meshing => sk.meshings[e.i()].class = c,
        EntKind::Traced => sk.traced[e.i()].class = c,
    }
}
