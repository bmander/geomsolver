//! **Which plane a relation is read in** (`docs/planes-plan.md`): a relation whose points are
//! all drawn in one plane is the 2D relation it always was — which, the lift being rigid, is the
//! same statement as the one in space — and any other is the relation in space, over the points'
//! places there.  No selector says so; the operands' planes do.  A point in space is in no plane,
//! so a relation naming one is in space.

use crate::constraints::{Arg, CKind};
use crate::model::{EntKind, EntRef, Sketch};

/// The drawn points an operand stands on: a point itself, a line's ends, a round thing's centre
/// (and an arc's ends), a spline's control points.  A plane, a curve, an axis and the spatial
/// kinds stand on none a plane reads.
fn operand_points(sk: &Sketch, e: EntRef) -> Vec<usize> {
    match e.kind {
        EntKind::Point => vec![e.i()],
        EntKind::Line | EntKind::Circle | EntKind::Arc | EntKind::Spline => sk
            .children(e)
            .iter()
            .filter(|c| c.kind == EntKind::Point)
            .map(|c| c.i())
            .collect(),
        _ => Vec::new(),
    }
}

/// **A point is read where its reader is drawn** (§6.7): an operand point drawn in further
/// planes too is taken by its twin in the plane every other operand is drawn in, so `O distance(5)
/// q`, with `q` drawn in `G` and `O` in `P, G`, is `G`'s own distance and not one in space.  A
/// relation whose operands cannot all be read in one plane so is left as it is, for `in_space`.
pub(super) fn read_twins(sk: &Sketch, args: &mut [Arg]) {
    if sk.twins.is_empty() {
        return;
    }
    let ents: Vec<EntRef> = args.iter().filter_map(|a| match a {
        Arg::Ent(e) => Some(*e),
        _ => None,
    }).collect();
    let mut planes: Vec<usize> = ents.iter()
        .flat_map(|&e| operand_points(sk, e))
        .filter_map(|p| sk.plane_of(p))
        .collect();
    planes.sort_unstable();
    planes.dedup();
    if planes.len() < 2 {
        return;
    }
    // the one plane every operand is drawn in, a point by its twin where it has one there
    let reads = |x: usize| ents.iter().all(|&e| match e.kind {
        EntKind::Point => sk.twin_in(e.i(), x).is_some(),
        _ => operand_points(sk, e).iter().all(|&p| sk.plane_of(p) == Some(x)),
    });
    let Some(x) = planes.into_iter().find(|&x| reads(x)) else { return };
    for a in args.iter_mut() {
        if let Arg::Ent(e) = a {
            if e.kind == EntKind::Point {
                *e = EntRef::point(sk.twin_in(e.i(), x).expect("reads in x"));
            }
        }
    }
}

/// The points a relation of this kind reads in a plane, and whether it is a kind whose operands'
/// planes decide what it means.  A radius, two equal radii and a ring's width read only radii,
/// which the lift carries unchanged, and a projection relates two planes by definition; a
/// relation in space is already what it is.
fn view_points(sk: &Sketch, kind: CKind, args: &[Arg]) -> Option<Vec<usize>> {
    if kind.spatial()
        || kind.gauge()
        || matches!(
            kind,
            CKind::Ordinate
                | CKind::Level
                | CKind::Project
                | CKind::Radius
                | CKind::EqualRadius
                | CKind::AnnularDistance
                | CKind::DragTarget
                | CKind::DragSeen
                | CKind::Lift
                | CKind::AxisUnit
                | CKind::AxisFoot
                | CKind::PlaneAxis
        )
    {
        return None;
    }
    let mut pts = Vec::new();
    for a in args {
        if let Arg::Ent(e) = a {
            pts.extend(operand_points(sk, *e));
        }
    }
    Some(pts)
}

/// What a settled relation means once its operands' planes are read: `Ok(None)` is the kind as
/// written, in one plane; `Ok(Some((k, args, left_out)))` the relation in space it is otherwise,
/// its arguments in `k`'s spec order with the inferred slots left for `io::seed_omitted`; `Err`
/// a word that has no meaning in space, or a selector that says nothing there, as (E040 or
/// E062, the message).
pub(super) fn in_space(
    sk: &Sketch,
    kind: CKind,
    args: &[Arg],
) -> Result<Option<(CKind, Vec<Arg>, Vec<bool>)>, (bool, String)> {
    // an ordinate is one statement in a view and in space — its form says which kernel reads it
    // (`constraints::OrdinateForm`) — but a direction said as a view's own word is an axis of
    // the view both points are drawn in, and across views there is none
    if kind.word_slot().is_some() {
        let word = crate::constraints::ordinate_word(kind, args);
        if crate::constraints::Toward::of(word).is_some_and(|t| !t.of_plane()) {
            let (p, q) = (args[0].ent().i(), args[1].ent().i());
            if crate::constraints::shared_view(sk, p, q).is_none() {
                return Err((false, format!(
                    "a view's own direction (`{word}`) has no meaning in space, and these points \
                     are not drawn in one view: name the direction, an axis such as `std.z` or a \
                     line"
                )));
            }
        }
        return Ok(None);
    }
    let Some(pts) = view_points(sk, kind, args) else { return Ok(None) };
    // where each point is read: its plane, or `None` for a point in space (a document gives
    // every point one or the other before a relation is read: `entities::places`)
    let views: Vec<Option<usize>> = pts.iter().map(|&p| sk.plane_of(p)).collect();
    // one plane, the 2D relation
    let one_plane =
        views.first().is_some_and(|v| v.is_some()) && views.iter().all(|v| *v == views[0]);
    if pts.is_empty() || one_plane {
        return Ok(None);
    }
    let word = kind.operator().map(|(w, _)| w).unwrap_or("this");
    let chosen = |i: usize| matches!(args.get(i), Some(Arg::Str(w)) if !w.is_empty());
    let k3 = match kind {
        CKind::Coincident => CKind::Coincident3,
        CKind::Distance => CKind::Distance3,
        CKind::PointOnLine => CKind::PointOnLine3,
        CKind::PointOnCircle => CKind::PointOnCircle3,
        CKind::Perpendicular => CKind::Perpendicular3,
        CKind::Parallel => CKind::Parallel3,
        CKind::EqualLength => CKind::EqualLength3,
        // the midpoint and the mirror are as well defined in space as in a plane: the mirror
        // in a line is the half turn about it, which on the line's own plane is the reflection
        CKind::Midpoint => CKind::Midpoint3,
        CKind::Symmetric => CKind::Symmetric3,
        CKind::PointLineDistance | CKind::ParallelDistance => {
            if chosen(3) {
                return Err((true, "`side:` names a side of a line in its plane, and in space a \
                    line has no sides: the distance is a magnitude".to_string()));
            }
            if kind == CKind::PointLineDistance { CKind::PointLine3 } else { CKind::LineLine3 }
        }
        CKind::Angle => {
            if chosen(3) {
                return Err((true, "`sense:` is which way an angle turns in a plane, and two lines \
                    in space turn neither way: the angle is unsigned".to_string()));
            }
            CKind::Angle3
        }
        _ => {
            return Err((false, format!(
                "`{word}` relates its operands in one plane, and these are not drawn in one: it \
                 has no meaning in space"
            )))
        }
    };
    // the entities in order, then the number where both kinds state one; every other slot of
    // the kind in space is inferred (a skew distance's side) or has its default
    let ents: Vec<Arg> = args.iter().filter(|a| matches!(a, Arg::Ent(_))).cloned().collect();
    let dim = kind.spec().iter().position(|(_, s)| s.is_dimension()).map(|i| args[i].clone());
    let mut out = Vec::new();
    let mut left_out = Vec::new();
    let mut ents = ents.into_iter();
    for (i, (_, s)) in k3.spec().iter().enumerate() {
        if s.is_entity() {
            out.push(ents.next().expect("the two kinds name the same operands"));
            left_out.push(false);
        } else if s.is_dimension() {
            out.push(dim.clone().expect("the two kinds state the same number"));
            left_out.push(false);
        } else {
            out.push(k3.default_arg(i));
            left_out.push(true);
        }
    }
    Ok(Some((k3, out, left_out)))
}
