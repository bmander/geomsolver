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

/// The points a relation of this kind reads in a plane, and whether it is a kind whose operands'
/// planes decide what it means.  A radius, two equal radii and a ring's width read only radii,
/// which the lift carries unchanged, and a projection relates two planes by definition; a
/// relation in space is already what it is.
fn view_points(sk: &Sketch, kind: CKind, args: &[Arg]) -> Option<Vec<usize>> {
    if kind.spatial()
        || kind.gauge()
        || matches!(
            kind,
            CKind::CoordinateU
                | CKind::CoordinateV
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
    // an ordinate along a plane's axis is the point's own coordinate where it is drawn in that
    // plane, and how far it stands along the axis in space where it is not
    if matches!(kind, CKind::CoordinateU | CKind::CoordinateV) {
        let (p, plane) = (args[0].ent().i(), args[1].ent().i());
        if sk.plane_of(p) == Some(plane) {
            return Ok(None);
        }
        let k3 = if kind == CKind::CoordinateU { CKind::Ordinate3U } else { CKind::Ordinate3V };
        return Ok(Some((k3, args.to_vec(), vec![false; args.len()])));
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
