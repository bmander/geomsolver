//! **Which view a relation is read in** (`docs/spatial-constraints-plan.md`, P2b): a relation
//! between entities drawn in different views is a relation in space, and one within a view is the
//! 2D relation it always was — which, the lift being rigid, is the same statement.  No selector
//! says so; the operands' views do.
//!
//! The one subtlety is the **role rule**, which the P0 audit found in the corpus.  A plane's own
//! datum points — its origin and its toward — place the view on the sheet, and their membership
//! says nothing about what they mean in a statement: `std.origin` is a member of `std.front`,
//! every `bracket.sv` view is drawn from one of its own corners, and 212 layout statements relate
//! datum points of different views to each other.  So a datum point is read by what it is
//! related to: beside only other datum points it is sheet layout, on the page; beside a view's own
//! points it is that view's (an ordinate from the datum the view is drawn from); and otherwise it
//! is where its membership puts it.  Read that way, no statement in the corpus changes meaning.

use crate::constraints::{Arg, CKind};
use crate::model::{EntKind, EntRef, Sketch};

/// The planes point `p` places on the sheet, as their origin or their toward.
fn places(sk: &Sketch, p: usize) -> Vec<usize> {
    (0..sk.planes.len())
        .filter(|&i| {
            let f = &sk.planes[i].frame;
            f.origin as usize == p || f.toward as usize == p
        })
        .collect()
}

/// Whether point `p` is some plane's datum point — what the page-placement gauge and the role
/// rule both ask.
pub fn is_datum(sk: &Sketch, p: usize) -> bool {
    !places(sk, p).is_empty()
}

/// The drawn points an operand stands on: a point itself, a line's ends, a round thing's centre
/// (and an arc's ends), a spline's control points.  A plane, a curve and the spatial kinds stand
/// on none a view reads.
pub fn operand_points(sk: &Sketch, e: EntRef) -> Vec<usize> {
    match e.kind {
        EntKind::Point => vec![e.i()],
        EntKind::Line | EntKind::Circle | EntKind::Arc | EntKind::Spline | EntKind::Sphere => sk
            .children(e)
            .iter()
            .filter(|c| c.kind == EntKind::Point)
            .map(|c| c.i())
            .collect(),
        _ => Vec::new(),
    }
}

/// The view each of a relation's points is read in, by the role rule: `None` is the page.
///
/// A point that is no plane's datum point reads where its membership puts it.  A datum point
/// reads in a view one of the relation's *other* points is drawn in, when it is that view's own
/// datum point; with no such point beside it — a statement among datum points only — it is sheet
/// layout, on the page; and otherwise its membership says.
pub fn reading_views(sk: &Sketch, pts: &[usize]) -> Vec<Option<usize>> {
    let roles: Vec<Vec<usize>> = pts.iter().map(|&p| places(sk, p)).collect();
    if roles.iter().all(|r| !r.is_empty()) {
        return vec![None; pts.len()];
    }
    let drawn: Vec<Option<usize>> = pts
        .iter()
        .zip(&roles)
        .filter(|(_, r)| r.is_empty())
        .map(|(&p, _)| sk.plane_of(p))
        .collect();
    pts.iter()
        .zip(&roles)
        .map(|(&p, r)| match r.iter().find(|&&v| drawn.contains(&Some(v))) {
            Some(&v) => Some(v),
            None => sk.plane_of(p),
        })
        .collect()
}

/// The points a relation of this kind reads in a view, and whether it is a kind whose operands'
/// views decide what it means.  A radius, two equal radii and a ring's width read only radii,
/// which the lift carries unchanged, and a projection relates two views by definition; a relation
/// in space and a hinge are already what they are.
fn view_points(sk: &Sketch, kind: CKind, args: &[Arg]) -> Option<Vec<usize>> {
    if kind.spatial()
        || kind.hinge()
        || kind.gauge()
        || matches!(
            kind,
            // an ordinate `along: u`/`v` measures against a datum as it stands on the sheet,
            // wherever the point is drawn — the page reading it always had, which the corpus
            // uses between views (`paired_references.sv`), and which a statement in space would
            // not be: a point has no ordinate on a view's axes unless it is drawn there
            CKind::CoordinateU
                | CKind::CoordinateV
                | CKind::Project
                | CKind::Radius
                | CKind::EqualRadius
                | CKind::AnnularDistance
                | CKind::SphereRadius
                | CKind::DragTarget
                | CKind::FrameUnit
                | CKind::FrameAlign
                | CKind::QuatUnit
                | CKind::Lift
                | CKind::LiftFixed
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

/// What a settled relation means once its operands' views are read: `Ok(None)` is the kind as
/// written, in one view; `Ok(Some((k, args)))` the relation in space it is across views, its
/// arguments in `k`'s spec order with the inferred slots left for `io::seed_omitted`; `Err` a
/// word that has no meaning in space, or a selector that says nothing there, as (E040 or E062,
/// the message).
pub fn in_space(
    sk: &Sketch,
    kind: CKind,
    args: &[Arg],
) -> Result<Option<(CKind, Vec<Arg>, Vec<bool>)>, (bool, String)> {
    let Some(pts) = view_points(sk, kind, args) else { return Ok(None) };
    let views = reading_views(sk, &pts);
    let mut distinct = views.clone();
    distinct.sort();
    distinct.dedup();
    if distinct.len() < 2 {
        return Ok(None);
    }
    // across views: every point is lifted from the view its membership puts it in, so a datum
    // point the role rule reads in some other view, or a point on the page, has no place in space
    for (&p, &v) in pts.iter().zip(&views) {
        let name = crate::io::entity_name(EntRef::point(p));
        if v.is_none() {
            return Err((false, format!(
                "{name} is on the page and the rest of the statement is drawn in a view: the page \
                 has no place in space, so draw it `in` the view it belongs to"
            )));
        }
        if v != sk.plane_of(p) {
            return Err((false, format!(
                "{name} is a datum point of {}, drawn in another view: across views a point is \
                 read where it is drawn, so state this in one view",
                crate::io::entity_name(EntRef::plane(v.expect("checked above")))
            )));
        }
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
        CKind::PointLineDistance | CKind::ParallelDistance => {
            if chosen(3) {
                return Err((true, "`side:` names a side of a line on the page, and in space a \
                    line has no sides: across views the distance is a magnitude".to_string()));
            }
            if kind == CKind::PointLineDistance { CKind::PointLine3 } else { CKind::LineLine3 }
        }
        CKind::Angle => {
            if chosen(3) {
                return Err((true, "`sense:` is which way an angle turns on a page, and two lines \
                    in space turn neither way: across views an angle is unsigned".to_string()));
            }
            CKind::Angle3
        }
        _ => {
            return Err((false, format!(
                "`{word}` relates its operands in one view, and these are drawn in different \
                 views: it has no meaning in space"
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

/// **The page-placement gauge** (P2b): where a solved view's picture sits on the sheet — its
/// datum's origin and toward, the x, y and turn of the picture — is presentation, not a freedom
/// of the object.  So a view with an attitude has its datum points held where they were drawn,
/// and the ledger does not count them, *unless the document states something about them*: a
/// relation naming either (directly, or as an end or a centre of what it names) or a gauge, and
/// then they are ordinary points, as every datum point of a stated view always was.  A datum
/// point shared with a stated view is that view's too, and is left alone.
///
/// Run after every relation is in, since a relation is what releases the hold.
pub(crate) fn hold_page_placement(sk: &mut Sketch) {
    let mut named = std::collections::BTreeSet::new();
    for c in sk.constraints.iter().filter(|c| !c.intrinsic) {
        for a in &c.args {
            if let Arg::Ent(e) = a {
                named.extend(operand_points(sk, *e));
            }
        }
    }
    let mut held = Vec::new();
    for i in 0..sk.planes.len() {
        if sk.planes[i].att.is_none() {
            continue;
        }
        let f = &sk.planes[i].frame;
        for p in [f.origin as usize, f.toward as usize] {
            let solved = places(sk, p).iter().all(|&v| sk.planes[v].att.is_some());
            let (x, y) = (sk.points[p].x as usize, sk.points[p].y as usize);
            let stated = sk.params[x].fixed || sk.params[y].fixed;
            if solved && !stated && !named.contains(&p) {
                held.push(p);
            }
        }
    }
    for p in held {
        sk.fix_point(p, true);
        sk.page_held.insert(p as u32);
    }
}
