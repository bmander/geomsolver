//! Which plane each point is drawn in (§6.7).

use super::resolve::Resolver;
use super::{Code, Diag, SourceMap};
use crate::ir::{Operation as StmtKind, Statement as Stmt};
use crate::constraints::CKind;
use crate::model::{EntKind, EntRef, Sketch};
use crate::syntax::{Span, StmtId};
use crate::io;
use std::collections::{BTreeMap, BTreeSet};

pub(super) fn memberships(
    sk: &mut Sketch,
    res: &Resolver,
    map: &SourceMap,
    body: &[&Stmt],
    skip: &BTreeSet<StmtId>,
    diags: &mut Vec<Diag>,
) {
    // a point drawn in several planes and its twins first (§6.7), so a reader drawn in one of
    // them that names the point finds it there, whatever order they are written in
    let imaged: BTreeSet<usize> =
        sk.twins.iter().flat_map(|(&p, ts)| std::iter::once(p).chain(ts.iter().copied())).collect();
    let first = |st: &&&Stmt| match &st.kind {
        StmtKind::Decl(d) => res.of.get(&d.name.key().text)
            .is_some_and(|e| e.kind == EntKind::Point && imaged.contains(&e.i())),
        _ => false,
    };
    let order = body.iter().filter(first).chain(body.iter().filter(|st| !first(st)));
    for st in order {
        let StmtKind::Decl(d) = &st.kind else { continue };
        let Some(r) = d.membership.plane() else { continue };
        if skip.contains(&st.id) {
            continue;
        }
        let mut fail = |code: Code, span: Span, message: String| {
            diags.push(Diag { code, span, stmt: Some(st.id), message });
        };
        let plane = match res.lookup(r) {
            None => {
                fail(Code::E101, r.span, format!("no such entity: `{}`", r.root.text));
                continue;
            }
            Some(e) if e.kind != EntKind::Plane || !r.path.is_empty() => {
                fail(
                    Code::E040,
                    r.span,
                    format!("`{}` is {}, and `in` names a plane", r.root.text, e.kind.a()),
                );
                continue;
            }
            Some(e) => e.i(),
        };
        // a declaration that could not be built made nothing to put anywhere
        let Some(me) = res.of.get(&d.name.key().text).copied() else { continue };
        let points: Vec<usize> = match me.kind {
            EntKind::Point => vec![me.i()],
            _ => sk.children(me).into_iter().map(|k| k.i()).collect(),
        };
        for p in points {
            // a point drawn in this plane too is read here by its twin (§6.7)
            let twin = if me.kind == EntKind::Point { None } else { sk.twin_in(p, plane) };
            match sk.plane_of(p) {
                Some(q) if q != plane && twin.is_some() => {
                    sk.replace_point(me, p, twin.expect("matched"));
                }
                Some(q) if q != plane => {
                    let who =
                        |e: EntRef| map.name_of(e).cloned().unwrap_or_else(|| io::entity_name(e));
                    fail(
                        Code::E060,
                        // the ref's own span: the clause's word, or the block header's
                        r.span,
                        format!(
                            "`{}` is already in `{}`",
                            who(EntRef::point(p)),
                            who(EntRef::plane(q))
                        ),
                    );
                }
                _ => sk.set_plane(p, Some(plane)),
            }
        }
    }
}

/// Each point drawn in further planes tied to its twins in space (§6.7, `Sketch::tie_twin`), so
/// the point has the one freedom of the line its planes meet on.  A point drawn twice in one
/// plane is refused where it is declared.
pub(super) fn tie_twins(sk: &mut Sketch, map: &SourceMap, diags: &mut Vec<Diag>) {
    let twins: Vec<(usize, Vec<usize>)> = sk.twins.iter().map(|(&p, t)| (p, t.clone())).collect();
    for (p, ts) in twins {
        let mut planes: Vec<Option<usize>> = vec![sk.plane_of(p)];
        for t in ts {
            let plane = sk.plane_of(t);
            if plane.is_none() || planes.contains(&plane) {
                let site = map.site_of(EntRef::point(p));
                let who = map.name_of(EntRef::point(p)).cloned()
                    .unwrap_or_else(|| io::entity_name(EntRef::point(p)));
                diags.push(Diag {
                    code: Code::E040,
                    span: site.map(|s| s.span).unwrap_or_default(),
                    stmt: site.map(|s| s.stmt),
                    message: format!("`{who}` is drawn in each plane it names once"),
                });
                continue;
            }
            planes.push(plane);
            sk.tie_twin(p, t);
        }
    }
}

/// A point drawn in two planes that are parallel, where they stand once placed: they meet on no
/// line, so the point can be on both only were they one (E061, the rule `project` states).
/// Judged only of planes turned as the statements say (`placed`, from `views::place`): one that
/// merely starts at its seed is judged where the solve leaves it (`views::degenerate`).
pub(super) fn parallel_twins(sk: &Sketch, map: &SourceMap, placed: &[bool], diags: &mut Vec<Diag>) {
    let placed = |pl: usize| placed.get(pl).copied().unwrap_or(false);
    for (&p, ts) in &sk.twins {
        let Some(first) = sk.plane_of(p).filter(|&pl| placed(pl)) else { continue };
        for &t in ts {
            let Some(other) = sk.plane_of(t).filter(|&q| q != first && placed(q)) else { continue };
            diags.extend(twin_parallel(sk, map, p, first, other));
        }
    }
}

/// E061 for point `p`, drawn in `first` and `other`, where the two stand parallel.
pub(super) fn twin_parallel(sk: &Sketch, map: &SourceMap, p: usize, first: usize, other: usize)
    -> Option<Diag> {
    let (n, m) = (sk.basis(first).normal(), sk.basis(other).normal());
    if crate::space::norm(crate::space::cross(n, m)) > crate::plane::PARALLEL_TOL {
        return None;
    }
    let site = map.site_of(EntRef::point(p));
    let name = |e: EntRef| map.name_of(e).cloned().unwrap_or_else(|| io::entity_name(e));
    Some(Diag {
        code: Code::E061,
        span: site.map(|s| s.span).unwrap_or_default(),
        stmt: site.map(|s| s.stmt),
        message: format!(
            "`{}` and `{}` are parallel, and meet on no line `{}` could be on",
            name(EntRef::plane(first)),
            name(EntRef::plane(other)),
            name(EntRef::point(p)),
        ),
    })
}

/// **A plane is a set, and `in` is membership of it** (§6.7, #105): `q coincident P`, said of a
/// point that no `in` reached, draws it in `P` as `q := point in P` would — two coordinates of
/// `P`'s and its lift, where a point in space would be three unknowns and a row holding one of
/// them to the plane.  One statement, two spellings, the same rows.  A line is its ends, so `l
/// coincident P` draws each end not yet in `P` there.  Read once every membership is in, in
/// statement order, so the first plane a point is put on is the one it is drawn in and any
/// other stays a row in space.  The index of each statement read so (into `stating`), which
/// then states nothing, and of each refused here.
///
/// A point stays in space, its statement a row, where its own declaration or a statement says
/// it stands there: its seed has a height (`hint(z: …)`), a `fix` holds its three numbers, a
/// tangency's derivative moves it (§6.21), or it is a `ring`'s, whose copies turn it in space.
///
/// **A curve drawn in a plane is read the same way** (#249): `m coincident g`, `g` a spline or a
/// traced curve drawn in `P` and `m` standing in space, draws `m` in `P` and the statement is the
/// contact in `P` it always was; `l tangent g` draws `l`'s free ends there.  Neither has a contact
/// in space, so a point kept there is refused with what keeps it (E062), said once: its
/// statement is the second set returned.
pub(super) fn incidences(
    sk: &mut Sketch,
    res: &Resolver,
    map: &mut SourceMap,
    stating: &[&Stmt],
    deferred: &[super::Deferred],
    rings: &[crate::flatten::RingInfo],
    diags: &mut Vec<Diag>,
) -> (BTreeSet<usize>, BTreeSet<usize>) {
    let ent = |r: &crate::syntax::Ref, sk: &Sketch| {
        res.entity(sk, r)
    };
    // each point a statement or its declaration keeps in space, and what keeps it there
    let mut kept: BTreeMap<usize, &str> = BTreeMap::new();
    for d in deferred {
        if let super::Deferred::Height { point, .. } = d {
            kept.insert(*point, "its seed has a `z`");
        }
    }
    for r in rings {
        kept.extend(map.ents_under(&r.prefix).into_iter()
            .filter(|e| e.kind == EntKind::Point).map(|e| (e.i(), "a ring turns it there")));
    }
    for st in stating {
        let StmtKind::Relation(r) = &st.kind else { continue };
        if let Some(e) = r.along.as_ref().and_then(|a| ent(&a.point, sk)) {
            kept.insert(e.i(), "a tangency's derivative moves it there");
        }
        if super::gauges::is_fix(r) {
            let held = r.form.written().and_then(|w| w.ops.first()).and_then(|o| ent(o, sk));
            kept.extend(held.filter(|e| e.kind == EntKind::Point)
                .map(|e| (e.i(), "a `fix` holds it there")));
        }
    }
    let (mut lowered, mut refused) = (BTreeSet::new(), BTreeSet::new());
    for (i, st) in stating.iter().enumerate() {
        let StmtKind::Relation(r) = &st.kind else { continue };
        let Some(w) = r.form.written() else { continue };
        if r.claim || r.along.is_some() || !matches!(w.word.text.as_str(), "coincident" | "tangent")
            || w.ops.len() != 2 || w.bound().is_some()
        {
            continue;
        }
        let (Some(a), Some(b)) = (ent(&w.ops[0], sk), ent(&w.ops[1], sk)) else { continue };
        // what the statement says is the kind it settles to, as `constrain` reads it: a word
        // the operands do not take is left for `constrain` to refuse
        let kind_of = |r: &crate::syntax::Ref| ent(r, sk).map(|e| e.kind);
        let Ok((kind, _)) = super::relations::settle(w, &kind_of) else { continue };
        let on_curve = match kind {
            CKind::PointOnPlane | CKind::LineOnPlane if w.args.is_empty() => false,
            CKind::PointOnSpline | CKind::PointOnCurve | CKind::SplineTangentLine
            | CKind::CurveTangentLine | CKind::SplineTangentAt | CKind::CurveTangentAt => true,
            _ => continue,
        };
        // the operand drawn in a plane (the plane itself, a spline, a curve) and the point or
        // line said to be on it
        let (on, what) = if matches!(a.kind, EntKind::Point | EntKind::Line) { (b, a) } else { (a, b) };
        let plane = match on.kind {
            EntKind::Plane => on.i(),
            // a curve standing for a surface (a prism's side) is met in space as it is
            EntKind::Curve if sk.curves[on.i()].extrusion => continue,
            _ => match drawn_plane(sk, on) {
                Some(p) => p,
                None => continue,
            },
        };
        let points = match what.kind {
            EntKind::Point => vec![what.i()],
            _ => sk.children(what).into_iter().map(|k| k.i()).collect(),
        };
        // a point of another view is put on this plane in space; one of this plane's already is
        // on it, which the row's own refusal says
        if points.iter().any(|&p| sk.plane_of(p).is_some_and(|v| v != plane)) {
            continue;
        }
        let free: Vec<usize> = points.into_iter().filter(|&p| sk.plane_of(p).is_none()).collect();
        match free.iter().find_map(|p| kept.get_key_value(p)) {
            // on a plane, a kept point's statement stays a row in space
            Some(_) if !on_curve => continue,
            // on a curve, which has no contact in space, it is refused here, said once
            Some((&p, why)) => {
                let name = |e: EntRef| map.name_of(e).cloned().unwrap_or_else(|| io::entity_name(e));
                let kind = if on.kind == EntKind::Spline { "spline" } else { "curve" };
                diags.push(Diag {
                    code: Code::E062,
                    span: st.span,
                    stmt: Some(st.id),
                    message: format!(
                        "`{}` stands in space ({why}), and a {} `{}` is drawn in `{}`, the \
                         plane the {kind} is drawn in: a {kind} has no contact in space",
                        name(EntRef::point(p)),
                        if what.kind == EntKind::Point { "point on" } else { "line tangent to" },
                        name(on),
                        name(EntRef::plane(plane)),
                    ),
                });
                refused.insert(i);
                continue;
            }
            None if free.is_empty() => continue,
            None => {}
        }
        for p in free {
            sk.set_plane(p, Some(plane));
            map.drawn.insert(p, plane);
        }
        if !on_curve {
            lowered.insert(i);
        }
    }
    (lowered, refused)
}

/// The plane an entity is drawn in: a curve, owning no point, its tool's (`Sketch::curve_view`),
/// and anything else the one every point of it is on.
pub(crate) fn drawn_plane(sk: &Sketch, e: EntRef) -> Option<usize> {
    match e.kind {
        EntKind::Curve => sk.curve_view(e.i()),
        _ => plane_of_entity(sk, e),
    }
}

/// The one plane every point of an entity is on, or `None` — for a point, its own.
pub(crate) fn plane_of_entity(sk: &Sketch, e: EntRef) -> Option<usize> {
    plane_of_entity_by(sk, e, |p| sk.plane_of(p))
}

/// The same walk over any reading of where a point is — membership here, and the overview's
/// table of it read once for every point (`overview::views`) — so the rule "every point it is
/// made of agrees" is written once.
pub(crate) fn plane_of_entity_by(
    sk: &Sketch,
    e: EntRef,
    at: impl Fn(usize) -> Option<usize>,
) -> Option<usize> {
    // a point answers for itself, and a datum or a curve has no points of its own to answer
    // for; everything else is its children, walked without collecting them twice
    if !e.kind.bears_points() {
        return None;
    }
    if e.kind == EntKind::Point {
        return at(e.i());
    }
    let kids = sk.children(e);
    let first = at(kids.first()?.i())?;
    kids.iter().all(|k| at(k.i()) == Some(first)).then_some(first)
}

/// **The plane each face used as a set is drawn in** (`flatten::faces`, #162 F5): bound under the
/// hidden name its carrier reads (`{face}#plane`), once memberships are in and before any
/// relation reads it — the plane every item of the face agrees about, the rule a face is built
/// by.  And each side's hidden axis seeded square to its edge within that plane, so the solve
/// starts where the side is.
pub(super) fn face_planes(
    sk: &mut Sketch,
    res: &mut Resolver,
    planes: &[crate::flatten::FacePlane],
    axes: &[crate::flatten::FaceAxis],
    diags: &mut Vec<Diag>,
) {
    fn ent(sk: &Sketch, res: &Resolver, name: &str) -> Option<EntRef> {
        let segs: Vec<&str> = name.split('.').collect();
        res.dotted(&segs).and_then(|(e, path)| super::resolve::follow(sk, e, &path).ok())
    }
    for f in planes {
        let views: Vec<Option<usize>> =
            f.edges.iter().map(|n| ent(sk, res, n).and_then(|e| plane_of_entity(sk, e))).collect();
        match views.first().copied().flatten() {
            Some(p) if views.iter().all(|v| *v == Some(p)) => {
                res.of.insert(f.key.clone(), EntRef::new(EntKind::Plane, p));
            }
            _ => diags.push(Diag {
                code: Code::E040,
                span: f.span,
                stmt: None,
                message: format!("`{}` is drawn in no one plane, so a solid swept from it has \
                                  no faces to draw on", f.face),
            }),
        }
    }
    for a in axes {
        let (Some(m), Some(e), Some(p)) =
            (ent(sk, res, &a.key), ent(sk, res, &a.edge), res.of.get(&a.plane).copied())
        else {
            continue;
        };
        let (EntKind::Axis, EntKind::Line) = (m.kind, e.kind) else { continue };
        let l = &sk.lines[e.i()];
        let d = crate::space::sub(sk.world_point(l.p2 as usize), sk.world_point(l.p1 as usize));
        let Some(dir) = crate::plane::unit(crate::space::cross(sk.basis(p.i()).normal(), d)) else {
            continue;
        };
        for (q, v) in sk.axes[m.i()].d.into_iter().zip(dir) {
            sk.params[q as usize].value = v;
        }
    }
}
