//! **The workspace**: the drawing as one scene in space, which an editor draws in.
//!
//! The glass box (`scene`, `scene3d`) shows a multiview document folded up; this is what makes the
//! same picture *editable*.  Every view's geometry is stored in the plane's own coordinates, which
//! it stands in space on (`Basis::lift`).  That is affine — a view's page stands in space by one
//! 3×3 placement — and, seen by an orthographic eye, so is the whole of it: each view's page maps
//! onto the eye's picture plane by one 2×3 matrix.  So a front end needs no 3D arithmetic to draw
//! a sketch on a tilted plane, or to turn a click back into a place on one — it composes the
//! view's map with its own 2D camera and inverts a 2×2 — and every question about what is under
//! the pointer is asked here, of the figures as the eye sees them, because views whose own
//! coordinates overlap are nowhere near one another in space.
//!
//! The page itself is a view: geometry in no plane stands on the front plane (`Basis::page`),
//! measured from the world origin.

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::model::{grow, polyline_distance, Box2, EntKind, EntRef, Sketch};
use crate::plane::{cross, dot, Basis};
use crate::space::{eye, sub};

use super::{drawable, views};

/// An affine map of a view's page coordinates onto the eye's picture plane:
/// `(x, y) ↦ (m[0]·x + m[1]·y + m[2], m[3]·x + m[4]·y + m[5])`.
pub type Map = [f64; 6];

/// The map that leaves the page where it is — a callout picked on the page it is laid out on.
pub const IDENTITY: Map = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0];

/// Where `p` lands under `m`.
pub fn apply(m: &Map, p: (f64, f64)) -> (f64, f64) {
    (m[0] * p.0 + m[1] * p.1 + m[2], m[3] * p.0 + m[4] * p.1 + m[5])
}

/// A view's page seen by the eye whose picture plane is spanned by `right` and `up`: a point drawn
/// in a plane is in the plane's own coordinates, so its page stands in space as `Basis::lift`.
fn seen_map(b: &Basis, right: [f64; 3], up: [f64; 3]) -> Map {
    let row = |e: [f64; 3]| (dot(e, b.u), dot(e, b.v), dot(e, b.o));
    let (a, bb, t) = row(right);
    let (d, e, f) = row(up);
    [a, bb, t, d, e, f]
}

/// The same page standing in the same place: a figure with a point in each is one figure.
fn same_page(x: &Basis, y: &Basis, tol: f64) -> bool {
    (0..3).all(|k| {
        (x.u[k] - y.u[k]).abs() <= 1e-12
            && (x.v[k] - y.v[k]).abs() <= 1e-12
            && (x.o[k] - y.o[k]).abs() <= tol
    })
}

/// What does not depend on the eye: where each view's page stands in space, the view each point
/// stands in, which views are one place, and the eye square on to each.  Indexed with the page
/// first and then every plane in order (`slot`).
pub struct Views {
    placements: Vec<Basis>,
    /// `overview::view_of` for each point: its membership.
    points: Vec<Option<usize>>,
    /// Which points stand in space: in no view, seen where they are (`Projection::point`).
    space: Vec<bool>,
    /// The first view standing on the same plane in space as each view (`Basis::coplanar`).
    places: Vec<Option<usize>>,
    /// A length below which two places in space are one, scaled to the drawing.
    tol: f64,
}

/// Where a view sits in `Views`' tables: the page first, then each plane.
fn slot(view: Option<usize>) -> usize {
    view.map_or(0, |i| i + 1)
}

/// The view a table index stands for.
fn view_at(k: usize) -> Option<usize> {
    k.checked_sub(1)
}

/// Where every view's page stands in space, the page first.
fn placements(sk: &Sketch) -> Vec<Basis> {
    let mut all = vec![Basis::page()];
    all.extend((0..sk.frames.len()).map(|i| sk.basis(i)));
    all
}

/// The eye at bearing `az` and elevation `el` (radians): its picture plane's right and up, and
/// toward the viewer (`Projection::toward`) — the one basis everything is seen in, and what a
/// front end with a camera of its own (the box's) sets it from, so its picture is the picking's.
pub fn eye_frame(az: f64, el: f64) -> [[f64; 3]; 3] {
    let (right, up) = eye(az, el);
    [right, up, cross(right, up)]
}

/// Every view's map onto the picture plane of the eye at `az`, `el` (radians), the page first —
/// what a front end asks per frame, so it reads the planes and nothing else.
pub fn maps(sk: &Sketch, az: f64, el: f64) -> Vec<Map> {
    let (right, up) = eye(az, el);
    placements(sk).iter().map(|p| seen_map(p, right, up)).collect()
}

/// Where the eye at `az`, `el` (radians) sees each point in space, by point index — `None` for a
/// point drawn in a plane, which its view's map places.  Per frame, beside `maps`, so a front end
/// draws a point in space without any arithmetic in three dimensions.
pub fn space_points(sk: &Sketch, az: f64, el: f64) -> Vec<Option<(f64, f64)>> {
    let (right, up) = eye(az, el);
    (0..sk.points.len())
        .map(|p| sk.points[p].z.is_some().then(|| {
            let x = sk.world_point(p);
            (dot(right, x), dot(up, x))
        }))
        .collect()
}

/// Where in space a point drawn in no plane is put, seen by the eye at `az`, `el` at `at` on its
/// picture plane: on the plane through the world origin square to the eye's ray — the depth the
/// eye cannot see is left at nought, and is the constraints' to say, as a drag in space leaves it
/// (`PlanDrag::seen`).  `space_points` sees it at `at` again.
pub fn unseen(az: f64, el: f64, at: (f64, f64)) -> [f64; 3] {
    let (right, up) = eye(az, el);
    std::array::from_fn(|k| right[k] * at.0 + up[k] * at.1)
}

impl Views {
    pub fn new(sk: &Sketch) -> Views {
        let placements = placements(sk);
        let tol = 1e-9 * sk.extent();
        let places = (0..placements.len())
            .map(|k| {
                let first = (0..k).find(|&j| placements[j].coplanar(&placements[k], tol)).unwrap_or(k);
                view_at(first)
            })
            .collect();
        let space = sk.points.iter().map(|p| p.z.is_some()).collect();
        Views { placements, points: views(sk), space, tol, places }
    }

    /// The view point `p` stands in.
    pub fn of_point(&self, p: usize) -> Option<usize> {
        self.points[p]
    }

    /// The first view on the same plane in space as `view` — its place, which a reader can compare.
    pub fn place(&self, view: Option<usize>) -> Option<usize> {
        self.places[slot(view)]
    }

    /// Whether two views' pages stand in the same place, so a figure may be drawn in either.
    fn same(&self, v: Option<usize>, w: Option<usize>) -> bool {
        v == w || same_page(&self.placements[slot(v)], &self.placements[slot(w)], self.tol)
    }

    /// The one view an entity is drawn in, `Err` where its points stand in views apart in space.
    /// Views standing in the same place — the page and `std.front` — are one, so a figure with a
    /// point in each is still drawn in one.
    pub fn entity_view(&self, sk: &Sketch, e: EntRef) -> Result<Option<usize>, ()> {
        // a point in space stands in no view: it is seen where it is, and nothing is laid out
        // on a page for it
        if e.kind == EntKind::Point {
            return if self.space[e.i()] { Err(()) } else { Ok(self.points[e.i()]) };
        }
        // a curve owns no point: it is drawn in the view of what it is written over
        if e.kind == EntKind::Curve {
            return Ok(sk.curve_view(e.i()));
        }
        if !e.kind.bears_points() {
            return Ok(None);
        }
        let mut first: Option<Option<usize>> = None;
        for k in sk.children(e).into_iter().filter(|k| k.kind == EntKind::Point) {
            if self.space[k.i()] {
                return Err(());
            }
            let v = self.points[k.i()];
            match first {
                None => first = Some(v),
                Some(w) if !self.same(w, v) => return Err(()),
                Some(_) => {}
            }
        }
        Ok(first.flatten())
    }

    /// The view a constraint's figure — its callout — is laid out in: the one every entity it
    /// names stands in, `Err` where they stand apart.
    pub fn constraint_view(&self, sk: &Sketch, c: &crate::constraints::Constraint) -> Result<Option<usize>, ()> {
        let mut seen: Option<Option<usize>> = None;
        for a in &c.args {
            let crate::constraints::Arg::Ent(r) = a else { continue };
            let v = self.entity_view(sk, *r)?;
            match seen {
                None => seen = Some(v),
                Some(w) if !self.same(w, v) => return Err(()),
                Some(_) => {}
            }
        }
        Ok(seen.flatten())
    }

    /// The eye square on to a view (`look_at`).
    pub fn look(&self, sk: &Sketch, view: Option<usize>) -> (f64, f64) {
        look_at(&view.map_or_else(Basis::page, |i| sk.basis(i)))
    }
}

/// Every view's map for one position of the eye, over `Views`.
///
/// Built per question rather than kept: it reads the sketch's current pose, and it costs one pass
/// over the planes and one over the points.
pub struct Projection {
    pub views: Views,
    /// The eye's picture plane, as the two world directions its axes run along.
    right: [f64; 3],
    up: [f64; 3],
    maps: Vec<Map>,
}

impl Projection {
    /// The maps as the eye at bearing `az` and elevation `el` (radians) sees them.
    pub fn new(sk: &Sketch, az: f64, el: f64) -> Projection {
        let (right, up) = eye(az, el);
        let views = Views::new(sk);
        let maps = views.placements.iter().map(|p| seen_map(p, right, up)).collect();
        Projection { views, right, up, maps }
    }

    /// Where a point in space is seen.
    pub fn seen(&self, x: [f64; 3]) -> (f64, f64) {
        (dot(self.right, x), dot(self.up, x))
    }

    /// Toward the viewer: the direction the eye's ray runs back along.
    pub fn toward(&self) -> [f64; 3] {
        cross(self.right, self.up)
    }

    /// The map of a view — `None` is the page.
    pub fn map(&self, view: Option<usize>) -> &Map {
        &self.maps[slot(view)]
    }

    /// The view point `p` stands in.
    pub fn view_of(&self, p: usize) -> Option<usize> {
        self.views.of_point(p)
    }

    /// Where point `p` is seen: through its view's map, or where it stands for a point in space.
    pub fn point(&self, sk: &Sketch, p: usize) -> (f64, f64) {
        if self.views.space[p] {
            return self.seen(sk.world_point(p));
        }
        apply(self.map(self.view_of(p)), sk.point_xy(p))
    }

    /// The view a constraint's callout is laid out in (`Views::constraint_view`).
    pub fn constraint_view(&self, sk: &Sketch, c: &crate::constraints::Constraint) -> Result<Option<usize>, ()> {
        self.views.constraint_view(sk, c)
    }

    /// What an entity is drawn as, as the eye sees it.  A line is drawn between its ends wherever
    /// each end stands (a projector between two views is one); anything else stands in its one
    /// view.  Datums and solids have no figure here — a renderer with a depth buffer draws those.
    pub fn figure(&self, sk: &Sketch, e: EntRef, unit: f64) -> Vec<Vec<(f64, f64)>> {
        match e.kind {
            EntKind::Point => vec![vec![self.point(sk, e.i())]],
            EntKind::Line => {
                let l = &sk.lines[e.i()];
                vec![vec![self.point(sk, l.p1 as usize), self.point(sk, l.p2 as usize)]]
            }
            EntKind::Frame => Vec::new(),
            _ => {
                let Ok(view) = self.views.entity_view(sk, e) else { return Vec::new() };
                let m = self.map(view);
                drawable(sk, e, unit)
                    .into_iter()
                    .map(|poly| poly.into_iter().map(|p| apply(m, p)).collect())
                    .collect()
            }
        }
    }

    /// How near an entity's figure comes to `at`, or infinity — the box round it on its page,
    /// seen, is asked first, so a figure out of reach is never tessellated.
    fn reach(&self, sk: &Sketch, e: EntRef, at: (f64, f64), tol: f64, unit: f64) -> f64 {
        if !matches!(e.kind, EntKind::Line | EntKind::Point) {
            let Ok(view) = self.views.entity_view(sk, e) else { return f64::INFINITY };
            let (x0, y0, x1, y1) = sk.bounds(e);
            if !(x0 <= x1 && y0 <= y1) {
                return f64::INFINITY;
            }
            let m = self.map(view);
            let mut b: Box2 = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
            for p in [(x0, y0), (x1, y0), (x0, y1), (x1, y1)] {
                grow(&mut b, apply(m, p));
            }
            if at.0 < b.0 - tol || at.0 > b.2 + tol || at.1 < b.1 - tol || at.1 > b.3 + tol {
                return f64::INFINITY;
            }
        }
        self.figure(sk, e, unit)
            .iter()
            .map(|poly| polyline_distance(poly, at.0, at.1))
            .fold(f64::INFINITY, f64::min)
    }
}

/// The nearest point to `at` as the eye sees it, and how far that is.
pub fn nearest_point(sk: &Sketch, proj: &Projection, at: (f64, f64)) -> (Option<usize>, f64) {
    let mut best = (None, f64::INFINITY);
    for p in 0..sk.points.len() {
        let (x, y) = proj.point(sk, p);
        let d = (x - at.0).dhypot(y - at.1);
        if d < best.1 {
            best = (Some(p), d);
        }
    }
    best
}

/// What a click at `at` on the eye's picture plane picks: `model::pick`'s rule, asked of the
/// figures where the eye sees them.  `unit` is the eye length of one screen pixel, which refines a
/// round figure.
///
/// **The drawing outranks a datum**: an axis, which has no figure, is picked only where nothing
/// drawn is in reach — along the line the box draws it as, and never seen end on, where it
/// would read as a dot under everything else.  A plane, beneath its axes, is picked by its pane's
/// frame (`nearest_pane`).
pub fn pick(sk: &Sketch, proj: &Projection, at: (f64, f64), tol: f64, unit: f64) -> Option<EntRef> {
    crate::model::pick_by(sk, nearest_point(sk, proj, at), tol, |e| proj.reach(sk, e, at, tol, unit))
        .or_else(|| nearest_axis(sk, proj, at, tol))
        .or_else(|| nearest_pane(sk, proj, at, tol))
}

/// The axis whose drawn line, seen, passes nearest `at` within `tol`.  Two lying on one another
/// (`std.back` is `std.x` reversed) are one distance but for rounding, so the first declared wins.
fn nearest_axis(sk: &Sketch, proj: &Projection, at: (f64, f64), tol: f64) -> Option<EntRef> {
    let reach = super::axis_reach(sk);
    let mut best: Option<(f64, usize)> = None;
    for i in (0..sk.axes.len()).filter(|&i| sk.normal_of(i).is_none()) {
        let Some((tail, tip, _)) = super::axis_segment(sk, i, reach) else { continue };
        let (a, b) = (proj.seen(tail), proj.seen(tip));
        if (b.0 - a.0).dhypot(b.1 - a.1) < 2.0 * tol {
            continue;
        }
        let d = crate::model::seg_distance(at, a, b);
        if d <= tol && best.is_none_or(|(e, _)| d < e - 1e-9 * tol) {
            best = Some((d, i));
        }
    }
    best.map(|(_, i)| EntRef::new(EntKind::Axis, i))
}

/// How foreshortened a pane may be and still be landed on: `ViewCam::readable`'s default, a pane
/// seen as good as exactly edge on and no more.
const EDGE_ON: f64 = 1e6;

/// Whether plane `i`'s pane is seen face on enough to be landed on (`EDGE_ON`).
fn pane_readable(proj: &Projection, i: usize) -> bool {
    let m = proj.map(Some(i));
    let det = m[0] * m[4] - m[1] * m[3];
    let k = m[0].dhypot(m[3]).max(m[1].dhypot(m[4]));
    det.abs() > 0.0 && k * k / det.abs() < EDGE_ON
}

/// The plane whose pane's frame, seen, passes nearest `at` within `tol` — a plane is picked where
/// the box draws it, as an axis is, and by its frame alone, so a press inside a pane still lands
/// on what is drawn there, or starts a band.  A place is one pane, its first plane's
/// (`overview::places`), and one seen edge on is no place to land (`panes_at`'s rule).  Two
/// frames one distance apart but for rounding go to the first declared.
fn nearest_pane(sk: &Sketch, proj: &Projection, at: (f64, f64), tol: f64) -> Option<EntRef> {
    let views = super::views(sk);
    let places = super::places(sk);
    let least = sk.extent() * super::LEAST_SIDE;
    let mut best: Option<(f64, usize)> = None;
    for i in (0..sk.frames.len()).filter(|&i| places[i] == Some(i) && pane_readable(proj, i)) {
        let basis = sk.basis(i);
        let (x0, y0, x1, y1) = super::pane(sk, i, &views, &places, least);
        let c = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)].map(|(x, y)| proj.seen(basis.lift(x, y)));
        let d = (0..4).map(|k| crate::model::seg_distance(at, c[k], c[(k + 1) % 4]))
            .fold(f64::INFINITY, f64::min);
        if d <= tol && best.is_none_or(|(e, _)| d < e - 1e-9 * tol) {
            best = Some((d, i));
        }
    }
    best.map(|(_, i)| EntRef::frame(i))
}

/// The planes whose panes `at` on the eye's picture plane falls inside, the one nearest the eye
/// first (ties in plane order) — what a double-click on a pane asks.  The pane is `pane`'s, the
/// same rule the box draws it by, and its whole face answers, not only its frame.
pub fn panes_at(sk: &Sketch, proj: &Projection, at: (f64, f64)) -> Vec<usize> {
    let views = super::views(sk);
    let places = super::places(sk);
    let least = sk.extent() * super::LEAST_SIDE;
    let toward = proj.toward();
    let mut hits: Vec<(f64, usize)> = Vec::new();
    for i in 0..sk.frames.len() {
        // a face family's station plane has no pane (#243 L1)
        if !pane_readable(proj, i) || places[i].is_none() {
            continue;
        }
        let m = proj.map(Some(i));
        let det = m[0] * m[4] - m[1] * m[3];
        // where the eye's ray through `at` meets the plane, in its own coordinates
        let (x, y) = (at.0 - m[2], at.1 - m[5]);
        let (px, py) = ((m[4] * x - m[1] * y) / det, (-m[3] * x + m[0] * y) / det);
        let (x0, y0, x1, y1) = super::pane(sk, i, &views, &places, least);
        if px < x0 || px > x1 || py < y0 || py > y1 {
            continue;
        }
        hits.push((dot(toward, sk.basis(i).lift(px, py)), i));
    }
    hits.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
    hits.into_iter().map(|(_, i)| i).collect()
}

/// A face of an object the eye's ray meets: which object (an index into `sketch.solids`), the
/// face's path, and how far toward the viewer it stands.  The path names the face where it was
/// made, which may be an operand (`bore.wall` on `body`), so the object is said beside it.
#[derive(Clone, Debug, PartialEq)]
pub struct SolidHit {
    pub solid: usize,
    pub face: String,
    pub depth: f64,
    /// Where in space the eye's ray met the face.
    pub at: [f64; 3],
}

/// The object face the eye's ray through `at` meets nearest the viewer — what a click on a solid
/// picks.  Asked of each object's mesh (`ApproximationPolicy::Mesh`, the box's own), culled by its
/// box: the ray is a line, since the eye is orthographic and stands outside everything.
pub fn pick_solid(sk: &Sketch, proj: &Projection, at: (f64, f64)) -> Option<SolidHit> {
    let toward = proj.toward();
    let foot = [0, 1, 2].map(|k| at.0 * proj.right[k] + at.1 * proj.up[k]);
    let mut best: Option<SolidHit> = None;
    for i in super::objects(sk) {
        let Ok(solid) = sk.evaluated_solid(i, crate::solid::ApproximationPolicy::Mesh) else { continue };
        let w = solid.world_bounds();
        if !w.meets(foot, toward, f64::NEG_INFINITY) {
            continue;
        }
        let o = solid.to_local(crate::solid::WorldPoint(foot)).0;
        let m = solid.mesh();
        for group in &m.groups {
            for t in group.start..group.start + group.count {
                let Some(depth) = line_meets_triangle(o, toward, &m.positions[9 * t..9 * t + 9]) else {
                    continue;
                };
                if best.as_ref().is_none_or(|b| depth > b.depth) {
                    let at = std::array::from_fn(|k| foot[k] + depth * toward[k]);
                    best = Some(SolidHit { solid: i, face: group.path.clone(), depth, at });
                }
            }
        }
    }
    best
}

/// Where two faces of object `solid` meet, in space (`EvaluatedSolid::face_edges`): what the
/// round and chamfer tools pick and light.  Empty for no object, or one that does not evaluate.
pub fn solid_edges(sk: &Sketch, solid: usize) -> Vec<crate::mesh::FaceEdge> {
    let Ok(s) = sk.evaluated_solid(solid, crate::solid::ApproximationPolicy::Mesh) else {
        return Vec::new();
    };
    let world = |p: [f64; 3]| s.to_world(crate::solid::LocalPoint(p)).0;
    s.face_edges().iter().map(|e| crate::mesh::FaceEdge {
        faces: e.faces.clone(),
        segs: e.segs.iter().map(|&[a, b]| [world(a), world(b)]).collect(),
    }).collect()
}

/// An edge of an object the eye sees under the pointer: which object, the two faces that meet
/// there (`FaceEdge::faces`), and the place on it nearest the pointer.
#[derive(Clone, Debug, PartialEq)]
pub struct EdgeHit {
    pub solid: usize,
    pub faces: (String, String),
    pub at: [f64; 3],
}

/// The edge of an object nearest `at` on the eye's picture plane, within `tol` there, that the
/// eye sees — an edge behind the object's material is not picked through it.  Asked of each
/// object's mesh, as `pick_solid` is: a place on an edge is hidden where the eye's ray through it
/// meets a face nearer the viewer than it, by more than a thousandth of the object.
pub fn pick_edge(sk: &Sketch, proj: &Projection, at: (f64, f64), tol: f64) -> Option<EdgeHit> {
    let toward = proj.toward();
    let foot = [0, 1, 2].map(|k| at.0 * proj.right[k] + at.1 * proj.up[k]);
    let mut near: Vec<(f64, EdgeHit, f64)> = Vec::new();
    for i in super::objects(sk) {
        let Ok(solid) = sk.evaluated_solid(i, crate::solid::ApproximationPolicy::Mesh) else {
            continue;
        };
        // culled by its box, as `pick_solid` is, grown by the reach of a pick
        let w = solid.world_bounds();
        if !w.grown(tol).meets(foot, toward, f64::NEG_INFINITY) {
            continue;
        }
        let slack = 1e-3 * (0..3).fold(0f64, |m, k| m.max(w.hi[k] - w.lo[k]));
        // the solid stands translated, and seeing is linear: its origin is seen once
        let o = solid.origin().0;
        let (ox, oy) = proj.seen(o);
        for e in solid.face_edges() {
            // the place on this edge nearest the pointer, as the eye sees it
            let mut best: Option<(f64, [f64; 3])> = None;
            for &[a, b] in &e.segs {
                let (sa, sb) = (proj.seen(a), proj.seen(b));
                let (sa, sb) = ((sa.0 + ox, sa.1 + oy), (sb.0 + ox, sb.1 + oy));
                let d = (sb.0 - sa.0, sb.1 - sa.1);
                let len2 = d.0 * d.0 + d.1 * d.1;
                let t = if len2 > 0.0 {
                    (((at.0 - sa.0) * d.0 + (at.1 - sa.1) * d.1) / len2).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let gap = (sa.0 + t * d.0 - at.0).dhypot(sa.1 + t * d.1 - at.1);
                if gap <= tol && best.is_none_or(|(g, _)| gap < g) {
                    best = Some((gap, crate::space::add(crate::space::lerp(a, b, t), o)));
                }
            }
            if let Some((gap, p)) = best {
                near.push((gap, EdgeHit { solid: i, faces: e.faces.clone(), at: p }, slack));
            }
        }
    }
    near.sort_by(|a, b| a.0.total_cmp(&b.0));
    near.into_iter().find(|(_, hit, slack)| {
        let seen = proj.seen(hit.at);
        let depth = dot(hit.at, toward);
        pick_solid(sk, proj, seen).is_none_or(|nearest| nearest.depth <= depth + slack)
    }).map(|(_, hit, _)| hit)
}

/// Where the line `o + t·d` crosses the triangle `p` (nine doubles), as `t` — either side, its
/// edges included (Möller–Trumbore).  `None` for a miss or a line in the triangle's plane.
fn line_meets_triangle(o: [f64; 3], d: [f64; 3], p: &[f64]) -> Option<f64> {
    let v0 = [p[0], p[1], p[2]];
    let e1 = sub([p[3], p[4], p[5]], v0);
    let e2 = sub([p[6], p[7], p[8]], v0);
    let h = cross(d, e2);
    let a = dot(e1, h);
    if a == 0.0 || !a.is_finite() {
        return None;
    }
    let f = 1.0 / a;
    let s = sub(o, v0);
    let u = f * dot(s, h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = cross(s, e1);
    let v = f * dot(d, q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    Some(f * dot(e2, q))
}

/// The arrow an extrusion is sized by, as the eye sees it, and what it says: from the middle of the
/// prism's face to its far end along the face's normal — no tip where the eye looks down the
/// normal, an arrow there being a dot — which way it runs, and how thick it is.
#[derive(Clone, Debug, PartialEq)]
pub struct ExtrudeHandle {
    pub base: (f64, f64),
    pub tip: Option<(f64, f64)>,
    /// `None` for extents none of the three ways say (`from: 2, to: 7`, written by hand).
    pub way: Option<crate::model::Way>,
    pub thick: f64,
}

/// How short a unit normal may look and still be dragged along: under it, the eye is looking
/// down the normal.
const EDGE_ON_NORMAL: f64 = 0.05;

/// A straight sizing handle as the eye sees it — an extrusion's or an offset plane's arrow: where
/// it starts and one unit along the normal it runs on (seeing is linear, so a point `t` along it
/// is seen at `base + t·step`).
#[derive(Clone, Copy, Debug)]
struct SeenLinear {
    base: (f64, f64),
    step: (f64, f64),
}

impl SeenLinear {
    /// The line through `at` along `dir`, in space, as the eye sees it.
    fn of(proj: &Projection, at: [f64; 3], dir: [f64; 3]) -> SeenLinear {
        SeenLinear { base: proj.seen(at), step: proj.seen(dir) }
    }

    /// Whether it can be dragged along: not where the eye looks down it, an arrow there being a
    /// dot.
    fn readable(&self) -> bool {
        self.step.0.dhypot(self.step.1) >= EDGE_ON_NORMAL
    }

    /// The point `t` along it, seen — the tip of an arrow `t` long — or `None` where the eye
    /// looks down it.
    fn tip(&self, t: f64) -> Option<(f64, f64)> {
        let (b, d) = (self.base, self.step);
        self.readable().then(|| (b.0 + t * d.0, b.1 + t * d.1))
    }

    /// How far along it the eye's ray through `at` passes nearest: `t`, or `None` where the eye
    /// looks down it — what dragging its arrow's tip to `at` sizes it to.
    fn along(&self, at: (f64, f64)) -> Option<f64> {
        let (b, d) = (self.base, self.step);
        let along = (at.0 - b.0) * d.0 + (at.1 - b.1) * d.1;
        self.readable().then(|| along / (d.0 * d.0 + d.1 * d.1))
    }
}

/// Prism `solid`'s arrow as the eye sees it, from its face's middle along its face's normal, and
/// its extents — `None` for a solid that is not a prism.
fn seen_prism(sk: &Sketch, proj: &Projection, solid: usize) -> Option<(SeenLinear, f64, f64)> {
    let crate::model::SolidDef::Prism { face, from, to, .. } = &sk.solids.get(solid)?.def else {
        return None;
    };
    let (poly, middle) = face_middle(sk, *face)?;
    Some((SeenLinear::of(proj, middle, poly.basis.normal()), from.value, to.value))
}

/// Face `face`'s outline and its area's middle in space — where a sizing handle starts, read
/// fine enough to place a handle by, not a volume: a round face is no finer than it need be.
fn face_middle(sk: &Sketch, face: u32) -> Option<(crate::solid::FacePoly, [f64; 3])> {
    let unit = (sk.extent() / 256.0).max(crate::solid::REPORT_UNIT);
    let poly = crate::solid::face_poly(sk, face as usize, unit)?;
    let (cx, cy) = crate::solid::ring_centroid(&poly.pts);
    let middle = poly.basis.lift(cx, cy);
    Some((poly, middle))
}

/// The arrow prism `solid` is sized by, seen: `None` for a solid that is not a prism.
pub fn extrude_handle(sk: &Sketch, proj: &Projection, solid: usize) -> Option<ExtrudeHandle> {
    let (arrow, from, to) = seen_prism(sk, proj, solid)?;
    let far = if to.abs() >= from.abs() { to } else { from };
    let (way, thick) = match crate::model::Way::of(from, to) {
        Some((way, d)) => (Some(way), d),
        None => (None, to - from),
    };
    Some(ExtrudeHandle { base: arrow.base, tip: arrow.tip(far), way, thick })
}

/// How far along prism `solid`'s normal the point under `at` is, as the eye sees it: the signed
/// distance from its face of the point on the normal through the face's middle that the eye's
/// ray through `at` passes nearest — what dragging the arrow's tip to `at` sizes it to.
pub fn extent_at(sk: &Sketch, proj: &Projection, solid: usize, at: (f64, f64)) -> Option<f64> {
    seen_prism(sk, proj, solid)?.0.along(at)
}

/// The arrow an offset plane is sized by (#162 F5), seen: from the plane it is set off to that
/// plane's normal times the distance — no tip where the eye looks down the normal — and the
/// distance.
#[derive(Clone, Debug, PartialEq)]
pub struct OffsetHandle {
    pub base: (f64, f64),
    pub tip: Option<(f64, f64)>,
    pub distance: f64,
}

/// `P distance(d) Q`, the constraint `id`, as the eye sees its arrow, from `P`'s origin along
/// `P`'s normal, and `d` — `None` for any other constraint.
fn seen_offset(sk: &Sketch, proj: &Projection, id: u32) -> Option<(SeenLinear, f64)> {
    let c = sk.constraint(id).filter(|c| c.kind == crate::constraints::CKind::PlaneDistance)?;
    let p = c.entities().first()?.i();
    let b = sk.basis(p);
    Some((SeenLinear::of(proj, b.o, b.normal()), c.args.get(2)?.value(sk)))
}

/// The arrow the plane distance `id` is sized by, seen: `None` for any other constraint.
pub fn offset_handle(sk: &Sketch, proj: &Projection, id: u32) -> Option<OffsetHandle> {
    let (arrow, distance) = seen_offset(sk, proj, id)?;
    Some(OffsetHandle { base: arrow.base, tip: arrow.tip(distance), distance })
}

/// What dragging the plane distance `id`'s arrow to `at` sizes it to: `extent_at` for a plane set
/// off another.
pub fn offset_at(sk: &Sketch, proj: &Projection, id: u32, at: (f64, f64)) -> Option<f64> {
    seen_offset(sk, proj, id)?.0.along(at)
}

/// The handle a revolution is sized by (#162 F3), seen: the middle of its profile, the arc that
/// middle turns through — a whole circle for a whole turn — and its end, which a press takes
/// hold of (no end where the eye looks along the turning plane, the arc there being a line), and
/// how far it turns: degrees, positive right-handed about the axis (`sense: ccw`).
#[derive(Clone, Debug, PartialEq)]
pub struct TurnHandle {
    pub base: (f64, f64),
    pub arc: Vec<(f64, f64)>,
    pub tip: Option<(f64, f64)>,
    pub turn: f64,
}

/// Revolution `solid`'s turning frame as the eye sees it: the foot on the axis of its profile's
/// middle, the middle less that foot and the same turned a right angle about the axis (seeing
/// is linear, so the middle turned `φ` is seen at `foot + cos φ·p + sin φ·q`), the determinant of
/// the two arms where the turn reads as more than a line (`None` where the eye looks along the
/// turning plane), and how far it turns, signed (radians) — `None` for a solid that is not a
/// revolution, or one whose middle is on its axis.
fn seen_turn(sk: &Sketch, proj: &Projection, solid: usize)
    -> Option<((f64, f64), (f64, f64), (f64, f64), Option<f64>, f64)> {
    let crate::model::SolidDef::Revolve { face, axis, sweep, sense } = &sk.solids.get(solid)?.def
    else {
        return None;
    };
    let (poly, c) = face_middle(sk, *face)?;
    let (a, b) = sk.revolve_axis(*face, *axis)?;
    let a = poly.basis.lift(a.0, a.1);
    let d = crate::plane::unit(sub(poly.basis.lift(b.0, b.1), a))?;
    let foot = crate::space::add(a, crate::space::scale(d, dot(sub(c, a), d)));
    let e1 = sub(c, foot);
    let r2 = dot(e1, e1);
    if !(r2 > 0.0) {
        return None;
    }
    let (p, q) = (proj.seen(e1), proj.seen(cross(d, e1)));
    // the arms are `r` long in space: seen nearly parallel, the eye looks along the plane
    let det = p.0 * q.1 - p.1 * q.0;
    let readable = (det.abs() >= EDGE_ON_NORMAL * r2).then_some(det);
    let sign = if *sense == crate::model::Sense::Cw { -1.0 } else { 1.0 };
    Some((proj.seen(foot), p, q, readable, sign * sweep.value))
}

/// How finely a turn handle's arc is drawn: steps a whole turn.
const TURN_STEPS: usize = 96;

/// The handle revolution `solid` is sized by, seen: `None` for a solid that is not a revolution.
pub fn turn_handle(sk: &Sketch, proj: &Projection, solid: usize) -> Option<TurnHandle> {
    let (f, p, q, readable, turn) = seen_turn(sk, proj, solid)?;
    let at = |phi: f64| {
        let (sn, cs) = phi.dsin_cos();
        (f.0 + cs * p.0 + sn * q.0, f.1 + cs * p.1 + sn * q.1)
    };
    let steps = ((turn.abs() / std::f64::consts::TAU * TURN_STEPS as f64).ceil() as usize).max(2);
    let arc = (0..=steps).map(|k| at(turn * k as f64 / steps as f64)).collect();
    let tip = readable.map(|_| at(turn));
    Some(TurnHandle { base: at(0.0), arc, tip, turn: turn.to_degrees() })
}

/// How far revolution `solid` turns where the eye sees its handle's end dragged to `at`, in signed
/// degrees (positive right-handed about the axis): the angle of the point of its turning plane
/// the eye's ray through `at` meets, taken the whole turns round nearest `near` and within a whole
/// turn either way — so a drag carried back past the start turns it the other way.  `None` where
/// the eye looks along the turning plane.
pub fn turn_at(sk: &Sketch, proj: &Projection, solid: usize, at: (f64, f64), near: f64)
    -> Option<f64> {
    let (f, p, q, readable, _) = seen_turn(sk, proj, solid)?;
    let det = readable?;
    // the point's coordinates along the two arms, by Cramer's rule
    let (x, y) = (at.0 - f.0, at.1 - f.1);
    let (c, s) = ((x * q.1 - y * q.0) / det, (p.0 * y - p.1 * x) / det);
    let phi = s.datan2(c).to_degrees();
    let k = ((near - phi) / 360.0).round();
    Some((phi + 360.0 * k).clamp(-360.0, 360.0))
}

/// Every entity whose figure touches the box `lo`–`hi` on the eye's picture plane — a rubber
/// band's "crossing" selection, asked where the figures are seen: a point inside it, or a stroke
/// passing through it.  What counts is what is drawn, so a box inside a circle's rim takes nothing.
pub fn overlapping(sk: &Sketch, proj: &Projection, lo: (f64, f64), hi: (f64, f64), unit: f64) -> Vec<EntRef> {
    let b = (lo.0.min(hi.0), lo.1.min(hi.1), lo.0.max(hi.0), lo.1.max(hi.1));
    sk.drawn()
        .into_iter()
        .filter(|&e| {
            proj.figure(sk, e, unit).iter().any(|poly| match poly.as_slice() {
                [p] => segment_meets_box(*p, *p, b),
                ps => ps.windows(2).any(|w| segment_meets_box(w[0], w[1], b)),
            })
        })
        .collect()
}

/// Whether the segment `p`–`q` meets the box `(x0, y0, x1, y1)`, edges included: the segment
/// clipped to each slab in turn (Liang–Barsky), met while some stretch of it is left.  A figure
/// with no finite place meets nothing.
fn segment_meets_box(p: (f64, f64), q: (f64, f64), (x0, y0, x1, y1): Box2) -> bool {
    if ![p.0, p.1, q.0, q.1].iter().all(|v| v.is_finite()) {
        return false;
    }
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    for (a, d, lo, hi) in [(p.0, q.0 - p.0, x0, x1), (p.1, q.1 - p.1, y0, y1)] {
        if d == 0.0 {
            if a < lo || a > hi {
                return false;
            }
            continue;
        }
        let (s, u) = ((lo - a) / d, (hi - a) / d);
        t0 = t0.max(s.min(u));
        t1 = t1.min(s.max(u));
        if t0 > t1 {
            return false;
        }
    }
    true
}

/// The extent of everything the workspace shows, on the eye's picture plane — every drawn figure,
/// and the box round each object seen from here (its eight corners), since a solid reaches past
/// the profiles it is made from: a turned section draws half of what it makes.  A drawing with
/// nothing drawn but at one place — a blank sheet's `std.origin` — is framed on its panes, the
/// workspace (`overview::WORKSPACE_MM`), rather than on a point.  `None` when nothing is drawn
/// and there is no pane.
pub fn bounds(sk: &Sketch, proj: &Projection, unit: f64) -> Option<Box2> {
    let mut b: Box2 = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for e in sk.drawn() {
        for p in proj.figure(sk, e, unit).into_iter().flatten() {
            grow(&mut b, p);
        }
    }
    for i in super::objects(sk) {
        let Ok(solid) = sk.evaluated_solid(i, crate::solid::ApproximationPolicy::Mesh) else { continue };
        let w = solid.world_bounds();
        if w.is_empty() {
            continue;
        }
        for k in 0..8 {
            let pick = |a: usize| if k >> a & 1 == 0 { w.lo[a] } else { w.hi[a] };
            grow(&mut b, proj.seen([pick(0), pick(1), pick(2)]));
        }
    }
    if !(b.0 < b.2 || b.1 < b.3) {
        let views = super::views(sk);
        let places = super::places(sk);
        let least = sk.extent() * super::LEAST_SIDE;
        for i in (0..sk.frames.len()).filter(|&i| places[i] == Some(i)) {
            let basis = sk.basis(i);
            let (x0, y0, x1, y1) = super::pane(sk, i, &views, &places, least);
            for (x, y) in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)] {
                grow(&mut b, proj.seen(basis.lift(x, y)));
            }
        }
    }
    (b.0 <= b.2).then_some(b)
}

/// The eye square on to a plane: the bearing and elevation (radians) from which `basis` is seen
/// face on, its normal toward the viewer.  Looking straight down or up the bearing is free, and is
/// chosen so the plane's `u` runs to the right.
pub fn look_at(basis: &Basis) -> (f64, f64) {
    let n = basis.normal();
    let el = n[2].clamp(-1.0, 1.0).dasin();
    let level = n[0].dhypot(n[1]);
    let az = if level > 1e-9 { n[1].datan2(n[0]) } else { (-basis.u[0]).datan2(basis.u[1]) };
    (az, el)
}
