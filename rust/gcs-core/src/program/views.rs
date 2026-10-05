//! What a solve says about planes that elaboration could not: two planes of a projection that
//! came out parallel, two lines meant to have a common perpendicular that came out parallel
//! (E065), and two planes that lie on one another (W113, `docs/planes-plan.md`).

#[allow(unused_imports)]
use crate::fmath::Det;
use super::{Code, Diag, SourceMap};
use crate::constraints::CKind;
use crate::model::{EntKind, EntRef, Sketch};

/// A plane this close to another, in turn and in where it stands, lies on it.
const ON_PLANE: f64 = 1e-9;

/// E065 for every statement a solve left degenerate: a projection whose two planes came out
/// parallel, a skew distance whose two lines did.
pub(crate) fn degenerate(sk: &Sketch, map: &SourceMap) -> Vec<Diag> {
    let mut out = Vec::new();
    for c in &sk.constraints {
        let message = match c.kind {
            CKind::ProjectSolved => {
                let (a, b) = (c.args[2].ent().i(), c.args[3].ent().i());
                let (na, nb) = (sk.basis(a).normal(), sk.basis(b).normal());
                (crate::space::norm(crate::space::cross(na, nb)) <= crate::plane::PARALLEL_TOL)
                    .then(|| format!(
                        "`{}` and `{}` came out parallel, so no fold line relates them and the \
                         projection says nothing",
                        sk.plane_name(a), sk.plane_name(b)
                    ))
            }
            CKind::LineLine3 | CKind::CylinderTangentLine => {
                let dir = |i: usize| {
                    // a cylinder's line is its axis
                    let e = c.args[i].ent();
                    let e = if e.kind == EntKind::Cylinder {
                        EntRef::line(sk.axial(e).axis as usize)
                    } else {
                        e
                    };
                    let l = &sk.lines[e.i()];
                    crate::space::sub(sk.lifted(l.p2 as usize), sk.lifted(l.p1 as usize))
                };
                let (a, b) = (dir(0), dir(1));
                let m = crate::space::norm(crate::space::cross(a, b));
                (m <= crate::plane::PARALLEL_TOL * crate::space::norm(a) * crate::space::norm(b))
                    .then(|| "the two lines came out parallel in space, and parallel lines have \
                              no common perpendicular to measure"
                        .to_string())
            }
            _ => None,
        };
        if let Some(message) = message {
            let site = map.site_of_constraint(c.id);
            out.push(Diag {
                code: Code::E065,
                span: site.map(|s| s.span).unwrap_or_default(),
                stmt: site.map(|s| s.stmt),
                message,
            });
        }
    }
    out
}

/// W113 for two planes that lie on one another — turned alike up to a turn in themselves and
/// standing in one place — where a relation reads points drawn in each: one plane in space, read
/// twice, and the relation is read in space where the plane would do.  Permitted (a part's plane
/// turned within the front is the common case, and two parts drawn on one plane relate nothing),
/// and said once, at the later plane's declaration.
pub(crate) fn coplanar(sk: &Sketch, map: &SourceMap) -> Vec<Diag> {
    use std::collections::BTreeSet;
    // the planes each relation reads drawn points of — a plane's own origin is where the plane
    // stands, not drawing in it
    let mut pairs: BTreeSet<(usize, usize)> = BTreeSet::new();
    for c in sk.constraints.iter().filter(|c| c.acts() && !c.intrinsic) {
        let mut planes: BTreeSet<usize> = BTreeSet::new();
        for e in c.entities() {
            let mut pts = Vec::new();
            points(sk, e, &mut pts, 0);
            planes.extend(pts.iter().filter(|&&p| sk.plane_of_origin(p).is_none())
                .filter_map(|&p| sk.plane_of(p)));
        }
        for &i in &planes {
            for &j in planes.range(i + 1..) {
                pairs.insert((i, j));
            }
        }
    }
    let mut out = Vec::new();
    let mut said: BTreeSet<usize> = BTreeSet::new();
    let tol = ON_PLANE * sk.extent().max(1.0);
    for &(i, j) in &pairs {
        if !sk.basis(i).coplanar(&sk.basis(j), tol) || !said.insert(j) {
            continue;
        }
        let Some(site) = map.site_of(EntRef::plane(j)) else { continue };
        out.push(Diag {
            code: Code::W113,
            span: site.span,
            stmt: Some(site.stmt),
            message: format!(
                "`{}` lies on `{}`, and a relation reads points drawn in each: one plane in \
                 space, so the relation is read in space where one plane would read it on the \
                 plane",
                sk.plane_name(j),
                sk.plane_name(i)
            ),
        });
    }
    out
}

/// The points an entity stands on, its axes and planes apart.
fn points(sk: &Sketch, e: EntRef, out: &mut Vec<usize>, depth: u32) {
    match e.kind {
        EntKind::Point => out.push(e.i()),
        EntKind::Plane | EntKind::Axis => {}
        _ if depth < 4 => {
            for k in sk.children(e) {
                points(sk, k, out, depth + 1);
            }
        }
        _ => {}
    }
}

/// **Where the planes stand, before the drawing is solved** (`docs/planes-plan.md`): a seed
/// that reads a place in another plane is read through that plane's pose, and a plane whose
/// axes or place the document solves for starts where its seeds put it — a fold line at the
/// world origin, say, when it stands on a line a hundred millimetres off.  So the statements
/// that say where the planes are — about axes and planes themselves, and about points of planes
/// already placed — are solved first, alone, round by round: a statement is taken once every
/// point it reads stands in a placed plane (or is held, or is a plane's own origin), and an axis
/// or a plane is placed once the statements taken determine it.  The drawing in a plane never
/// places it: its seeds are what is to be read through it.  Nothing else moves; the hidden
/// points are put back where their points now stand.  What nothing places keeps its seed.
pub(crate) fn place(sk: &mut Sketch) {
    use std::collections::BTreeSet;
    align(sk);
    let held = |sk: &Sketch, ps: &[u32]| ps.iter().all(|&q| sk.params[q as usize].fixed);
    let mut axis_placed: Vec<bool> =
        (0..sk.axes.len()).map(|r| held(sk, &sk.axes[r].d)).collect();
    let mut plane_placed: Vec<bool> = (0..sk.planes.len()).map(|p| sk.plane_fixed(p)).collect();
    if axis_placed.iter().all(|&b| b) && plane_placed.iter().all(|&b| b) {
        return;
    }
    // what each statement reads: its axes and planes, a plane's origin standing for its plane,
    // the planes the other points it reads are drawn in (a held point waits for none), and every
    // point, whose hidden point follows the planes while it is solved
    struct Reads { id: u32, axes: Vec<usize>, planes: Vec<usize>, through: Vec<usize>, pts: Vec<usize> }
    let mut reads: Vec<Reads> = Vec::new();
    for c in sk.constraints.iter().filter(|c| c.acts()) {
        if matches!(c.kind, CKind::Lift | CKind::AxisUnit | CKind::AxisFoot | CKind::DragTarget) {
            continue;
        }
        // a point measured in a plane — an ordinate, a height off it — is the drawing placed by
        // the plane, not the plane by the drawing, unless the point is where a plane stands
        if matches!(c.kind, CKind::Ordinate3U | CKind::Ordinate3V | CKind::PointPlaneDistance) {
            let p = c.args[0].ent().i();
            if sk.plane_of_origin(p).is_none() && !sk.point_held(p) {
                continue;
            }
        }
        let (mut axes, mut planes, mut pts) = (Vec::new(), Vec::new(), Vec::new());
        for e in c.entities() {
            match e.kind {
                EntKind::Axis => axes.push(e.i()),
                EntKind::Plane => planes.push(e.i()),
                _ => points(sk, e, &mut pts, 0),
            }
        }
        let mut through = Vec::new();
        let mut drawing = false;
        for &p in &pts {
            if let Some(pl) = sk.plane_of_origin(p) {
                planes.push(pl);
            } else if sk.point_held(p) {
                // held: wherever its plane stands, it is where it is said to be
            } else if let Some(pl) = sk.plane_of(p) {
                through.push(pl);
            } else {
                // a point in space that is not held is the drawing's to place, never a plane's
                drawing = true;
            }
        }
        for &pl in &planes {
            axes.extend([sk.planes[pl].u as usize, sk.planes[pl].v as usize]);
        }
        if drawing || axes.is_empty() && planes.is_empty() {
            continue;
        }
        reads.push(Reads { id: c.id, axes, planes, through, pts });
    }
    let mut taken: BTreeSet<u32> = BTreeSet::new();
    let along = along(sk);
    for _ in 0..sk.planes.len() + sk.axes.len() + 1 {
        // an axis held along a line whose ends are placed is placed with them: turned to the line
        // here, sense and all, and held so the solve below cannot turn it round
        let mut newly = false;
        for &(r, l) in &along {
            let ends = [sk.lines[l].p1 as usize, sk.lines[l].p2 as usize];
            let fine = ends.iter().all(|&p| {
                sk.plane_of_origin(p).is_some()
                    || sk.plane_of(p).is_some_and(|pl| plane_placed[pl])
                    || sk.point_held(p)
            });
            if fine && !axis_placed[r] && sk.turn_axis_along(r, l) {
                axis_placed[r] = true;
                newly = true;
            }
        }
        for p in 0..sk.planes.len() {
            let pl = &sk.planes[p];
            if !plane_placed[p] && held(sk, &pl.o) && axis_placed[pl.u as usize] && axis_placed[pl.v as usize] {
                plane_placed[p] = true;
                newly = true;
            }
        }
        let before = taken.len();
        for r in &reads {
            let unplaced = r.axes.iter().any(|&k| !axis_placed[k])
                || r.planes.iter().any(|&p| !plane_placed[p]);
            let ready = r.through.iter().all(|&p| plane_placed[p]);
            if unplaced && ready {
                taken.insert(r.id);
            }
        }
        if taken.len() == before {
            if newly {
                continue;
            }
            break;
        }
        // the taken statements alone: the unplaced axes and places they read free, everything
        // else held, the hidden points free to follow
        let mut part = sk.clone();
        let mut free: BTreeSet<u32> = BTreeSet::new();
        let mut read: BTreeSet<usize> = BTreeSet::new();
        for r in reads.iter().filter(|r| taken.contains(&r.id)) {
            read.extend(&r.pts);
            for &k in r.axes.iter().filter(|&&k| !axis_placed[k]) {
                free.extend(sk.axes[k].d);
                if sk.axes[k].placed {
                    free.extend(sk.axes[k].a);
                }
            }
            for &p in r.planes.iter().filter(|&&p| !plane_placed[p]) {
                free.extend(sk.planes[p].o);
            }
        }
        let item: BTreeSet<u32> = free.clone();
        // the hidden points of the points read follow their planes; no other is in the part
        for l in &sk.lifts {
            if sk.points[l.point as usize].z.is_none() && read.contains(&(l.point as usize)) {
                free.extend(l.x);
            }
        }
        for (i, q) in part.params.iter_mut().enumerate() {
            q.fixed = q.fixed || !free.contains(&(i as u32));
        }
        let free_axis = |r: usize| sk.axes[r].d.iter().any(|q| free.contains(q));
        part.constraints.retain(|c| {
            taken.contains(&c.id)
                || match c.kind {
                    CKind::Lift => read.contains(&c.args[0].ent().i()),
                    CKind::AxisUnit | CKind::AxisFoot => free_axis(c.args[0].ent().i()),
                    _ => false,
                }
        });
        crate::solve::solve(&mut part, crate::solve::SolveOpts::default());
        for &q in &item {
            sk.params[q as usize].value = part.params[q as usize].value;
        }
        // placed: what the statements taken leave no freedom
        let d = crate::diagnose::diagnose(&mut part, Default::default());
        let under: BTreeSet<u32> = d.under_params.iter().copied().collect();
        let fixed = |ps: &[u32]| ps.iter().all(|q| !under.contains(q));
        for k in 0..sk.axes.len() {
            if !axis_placed[k] && sk.axes[k].d.iter().all(|q| item.contains(q)) {
                axis_placed[k] = fixed(&sk.axes[k].d);
            }
        }
        for p in 0..sk.planes.len() {
            let pl = &sk.planes[p];
            let o_ok = pl.o.iter().all(|q| !item.contains(q) || !under.contains(q))
                && (held(sk, &pl.o) || pl.o.iter().all(|q| item.contains(q)));
            if !plane_placed[p] {
                plane_placed[p] = o_ok && axis_placed[pl.u as usize] && axis_placed[pl.v as usize];
            }
        }
    }
    relift(sk);
}

/// Every axis a plane holds along a drawn line (`program::entities::axes_along`) turned to the
/// line as it now stands, its sense included — `parallel` is satisfied both ways, so an axis
/// seeded before its line's points were placed may stand against it, and a plane over it turn
/// its back.
fn align(sk: &mut Sketch) {
    for (r, l) in along(sk) {
        if !sk.axes[r].d.iter().any(|&q| sk.params[q as usize].fixed) {
            sk.turn_axis_along(r, l);
        }
    }
}

/// The axes held along drawn lines, and their lines.
fn along(sk: &Sketch) -> Vec<(usize, usize)> {
    sk.constraints.iter()
        .filter(|c| c.intrinsic && c.kind == CKind::Parallel3)
        .filter_map(|c| match (c.args[0].ent(), c.args[1].ent()) {
            (r, l) if r.kind == EntKind::Axis && l.kind == EntKind::Line => Some((r.i(), l.i())),
            _ => None,
        })
        .collect()
}

/// Every hidden point put where its point now stands — after a pass that moved planes or
/// points without the lift rows that tie them.
pub(crate) fn relift(sk: &mut Sketch) {
    for k in 0..sk.lifts.len() {
        let p = sk.lifts[k].point as usize;
        if sk.points[p].z.is_none() {
            let at = sk.world_point(p);
            for i in 0..3 {
                let q = sk.lifts[k].x[i] as usize;
                sk.params[q].value = at[i];
            }
        }
    }
}
