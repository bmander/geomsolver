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
use crate::space::sub;

use super::{drawable, eye, views};

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
    all.extend((0..sk.planes.len()).map(|i| sk.basis(i)));
    all
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
            EntKind::Plane => Vec::new(),
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
/// would read as a dot under everything else.
pub fn pick(sk: &Sketch, proj: &Projection, at: (f64, f64), tol: f64, unit: f64) -> Option<EntRef> {
    crate::model::pick_by(sk, nearest_point(sk, proj, at), tol, |e| proj.reach(sk, e, at, tol, unit))
        .or_else(|| nearest_axis(sk, proj, at, tol))
}

/// The axis whose drawn line, seen, passes nearest `at` within `tol`.  Two lying on one another
/// (`std.back` is `std.x` reversed) are one distance but for rounding, so the first declared wins.
fn nearest_axis(sk: &Sketch, proj: &Projection, at: (f64, f64), tol: f64) -> Option<EntRef> {
    let reach = super::axis_reach(sk);
    let mut best: Option<(f64, usize)> = None;
    for i in 0..sk.axes.len() {
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

/// The planes whose panes `at` on the eye's picture plane falls inside, the one nearest the eye
/// first (ties in plane order) — what a double-click on a pane asks.  The pane is `pane`'s, the
/// same rule the box draws it by, and its whole face answers, not only its frame.
pub fn panes_at(sk: &Sketch, proj: &Projection, at: (f64, f64)) -> Vec<usize> {
    let views = super::views(sk);
    let least = sk.extent() * super::LEAST_SIDE;
    let toward = proj.toward();
    let mut hits: Vec<(f64, usize)> = Vec::new();
    for i in 0..sk.planes.len() {
        let m = proj.map(Some(i));
        let det = m[0] * m[4] - m[1] * m[3];
        let k = m[0].dhypot(m[3]).max(m[1].dhypot(m[4]));
        if !(det.abs() > 0.0 && k * k / det.abs() < EDGE_ON) {
            continue;
        }
        // where the eye's ray through `at` meets the plane, in its own coordinates
        let (x, y) = (at.0 - m[2], at.1 - m[5]);
        let (px, py) = ((m[4] * x - m[1] * y) / det, (-m[3] * x + m[0] * y) / det);
        let (x0, y0, x1, y1) = super::pane(sk, i, &views, least);
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
                    best = Some(SolidHit { solid: i, face: group.path.clone(), depth });
                }
            }
        }
    }
    best
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

/// The arrow an extrusion is sized by, as the eye sees it: from the middle of the prism's face to
/// its far end along the face's normal, and the two extents it runs between.
#[derive(Clone, Debug, PartialEq)]
pub struct ExtrudeHandle {
    pub base: (f64, f64),
    pub tip: (f64, f64),
    pub from: f64,
    pub to: f64,
}

/// Where prism `solid`'s face sits and which way it is swept: the face's middle and its plane's
/// normal, in space, and its two extents — `None` for a solid that is not a prism.
fn prism_frame(sk: &Sketch, solid: usize) -> Option<([f64; 3], [f64; 3], f64, f64)> {
    let crate::model::SolidDef::Prism { face, from, to } = &sk.solids.get(solid)?.def else {
        return None;
    };
    let poly = crate::solid::face_poly(sk, *face as usize, crate::solid::REPORT_UNIT)?;
    let basis = match sk.faces[*face as usize].plane().ok()? {
        Some(p) => sk.basis(p as usize),
        None => Basis::page(),
    };
    let (cx, cy) = centroid(&poly.pts);
    Some((basis.lift(cx, cy), basis.normal(), from.value, to.value))
}

/// The area centroid of a closed ring, or the mean of its corners where it encloses nothing.
fn centroid(pts: &[(f64, f64)]) -> (f64, f64) {
    let n = pts.len();
    let (mut a, mut x, mut y) = (0.0, 0.0, 0.0);
    for i in 0..n {
        let (p, q) = (pts[i], pts[(i + 1) % n]);
        let w = p.0 * q.1 - q.0 * p.1;
        a += w;
        x += (p.0 + q.0) * w;
        y += (p.1 + q.1) * w;
    }
    if a.abs() > 0.0 {
        return (x / (3.0 * a), y / (3.0 * a));
    }
    let k = n.max(1) as f64;
    (pts.iter().map(|p| p.0).sum::<f64>() / k, pts.iter().map(|p| p.1).sum::<f64>() / k)
}

/// The arrow prism `solid` is sized by, seen: `None` for a solid that is not a prism, or one whose
/// normal the eye looks straight down, where an arrow along it would be a dot.
pub fn extrude_handle(sk: &Sketch, proj: &Projection, solid: usize) -> Option<ExtrudeHandle> {
    let (base, n, from, to) = prism_frame(sk, solid)?;
    let along = |t: f64| proj.seen([0, 1, 2].map(|k| base[k] + t * n[k]));
    let (b, one) = (along(0.0), along(1.0));
    if (one.0 - b.0).dhypot(one.1 - b.1) < 0.05 {
        return None;
    }
    let far = if to.abs() >= from.abs() { to } else { from };
    Some(ExtrudeHandle { base: b, tip: along(far), from, to })
}

/// How far along prism `solid`'s normal the point under `at` is, as the eye sees it: the signed
/// distance from its face of the point on the normal through the face's middle that the eye's
/// ray through `at` passes nearest — what dragging the arrow's tip to `at` sizes it to.
pub fn extent_at(sk: &Sketch, proj: &Projection, solid: usize, at: (f64, f64)) -> Option<f64> {
    let (base, n, ..) = prism_frame(sk, solid)?;
    let b = proj.seen(base);
    let one = proj.seen([0, 1, 2].map(|k| base[k] + n[k]));
    let d = (one.0 - b.0, one.1 - b.1);
    let len2 = d.0 * d.0 + d.1 * d.1;
    (len2 > 0.05 * 0.05).then(|| ((at.0 - b.0) * d.0 + (at.1 - b.1) * d.1) / len2)
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
/// the profiles it is made from: a turned section draws half of what it makes.  `None` when
/// nothing is drawn.
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
