//! Which plane each point is drawn in (§6.7).

use super::resolve::Resolver;
use super::{Code, Diag, SourceMap};
use crate::ir::{Operation as StmtKind, Statement as Stmt};
use crate::model::{EntKind, EntRef, Sketch};
use crate::syntax::{Span, StmtId};
use crate::io;
use std::collections::BTreeSet;

pub(super) fn memberships(
    sk: &mut Sketch,
    res: &Resolver,
    map: &SourceMap,
    body: &[&Stmt],
    skip: &BTreeSet<StmtId>,
    diags: &mut Vec<Diag>,
) {
    for st in body {
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
            match sk.plane_of(p) {
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

/// **A plane is a set, and `in` is membership of it** (§6.7, #105): `q coincident P`, said of a
/// point that no `in` reached, draws it in `P` as `q := point in P` would — two coordinates of
/// `P`'s and its lift, where a point in space would be three unknowns and a row holding one of
/// them to the plane.  One statement, two spellings, the same rows.  A line is its ends, so `l
/// coincident P` draws each end not yet in `P` there.  Read once every membership is in, in
/// statement order, so the first plane a point is put on is the one it is drawn in and any
/// other stays a row in space.  The index of each statement read so (into `stating`), which
/// then states nothing.
///
/// A point stays in space, its statement a row, where its own declaration or a statement says
/// it stands there: its seed has a height (`hint(z: …)`), a `fix` holds its three numbers, a
/// tangency's derivative moves it (§6.21), or it is a `ring`'s, whose copies turn it in space.
pub(super) fn incidences(
    sk: &mut Sketch,
    res: &Resolver,
    map: &mut SourceMap,
    stating: &[&Stmt],
    deferred: &[super::Deferred],
    rings: &[crate::flatten::RingInfo],
) -> BTreeSet<usize> {
    use super::resolve::follow;
    let ent = |r: &crate::syntax::Ref, sk: &Sketch| {
        res.lookup(r).and_then(|e| follow(sk, e, &r.path).ok())
    };
    let mut kept: BTreeSet<usize> = deferred.iter().filter_map(|d| match d {
        super::Deferred::Height { point, .. } => Some(*point),
        _ => None,
    }).collect();
    for r in rings {
        kept.extend(map.ents_under(&r.prefix).into_iter()
            .filter(|e| e.kind == EntKind::Point).map(|e| e.i()));
    }
    for st in stating {
        let StmtKind::Relation(r) = &st.kind else { continue };
        if let Some(e) = r.along.as_ref().and_then(|a| ent(&a.point, sk)) {
            kept.insert(e.i());
        }
        if super::relations::is_fix(r) {
            let held = r.form.written().and_then(|w| w.ops.first()).and_then(|o| ent(o, sk));
            kept.extend(held.filter(|e| e.kind == EntKind::Point).map(|e| e.i()));
        }
    }
    let mut lowered = BTreeSet::new();
    for (i, st) in stating.iter().enumerate() {
        let StmtKind::Relation(r) = &st.kind else { continue };
        let Some(w) = r.form.written() else { continue };
        if r.claim || r.along.is_some() || w.word.text != "coincident" || w.ops.len() != 2
            || !w.args.is_empty()
        {
            continue;
        }
        let ops = [ent(&w.ops[0], sk), ent(&w.ops[1], sk)];
        let (plane, what) = match ops {
            [Some(p), Some(e)] | [Some(e), Some(p)] if p.kind == EntKind::Plane => (p.i(), e),
            _ => continue,
        };
        let points = match what.kind {
            EntKind::Point => vec![what.i()],
            EntKind::Line => sk.children(what).into_iter().map(|k| k.i()).collect(),
            _ => continue,
        };
        // a point of another view is put on this plane in space; one of this plane's already is
        // on it, which the row's own refusal says
        if points.iter().any(|&p| sk.plane_of(p).is_some_and(|v| v != plane)) {
            continue;
        }
        let free: Vec<usize> = points.into_iter().filter(|&p| sk.plane_of(p).is_none()).collect();
        if free.is_empty() || free.iter().any(|p| kept.contains(p)) {
            continue;
        }
        for p in free {
            sk.set_plane(p, Some(plane));
            map.drawn.insert(p, plane);
        }
        lowered.insert(i);
    }
    lowered
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
