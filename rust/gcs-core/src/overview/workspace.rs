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
use crate::plane::{dot, Basis};

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
    /// view.  Datums, solids and the kinds that live in space (spheres, cones, cylinders) have no
    /// figure here — a renderer with a depth buffer draws those.
    pub fn figure(&self, sk: &Sketch, e: EntRef, unit: f64) -> Vec<Vec<(f64, f64)>> {
        match e.kind {
            EntKind::Point => vec![vec![self.point(sk, e.i())]],
            EntKind::Line => {
                let l = &sk.lines[e.i()];
                vec![vec![self.point(sk, l.p1 as usize), self.point(sk, l.p2 as usize)]]
            }
            EntKind::Plane | EntKind::Sphere | EntKind::Cone | EntKind::Cylinder => Vec::new(),
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
pub fn pick(sk: &Sketch, proj: &Projection, at: (f64, f64), tol: f64, unit: f64) -> Option<EntRef> {
    crate::model::pick_by(sk, nearest_point(sk, proj, at), tol, |e| proj.reach(sk, e, at, tol, unit))
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
    // toward the viewer: `right × up`
    let (r, u) = (proj.right, proj.up);
    let toward = [r[1] * u[2] - r[2] * u[1], r[2] * u[0] - r[0] * u[2], r[0] * u[1] - r[1] * u[0]];
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

/// Every entity whose whole figure lies inside the box `lo`–`hi` on the eye's picture plane — a
/// rubber band's "window" selection, asked where the figures are seen.
pub fn inside(sk: &Sketch, proj: &Projection, lo: (f64, f64), hi: (f64, f64), unit: f64) -> Vec<EntRef> {
    let (x0, x1) = (lo.0.min(hi.0), lo.0.max(hi.0));
    let (y0, y1) = (lo.1.min(hi.1), lo.1.max(hi.1));
    let within = |p: &(f64, f64)| p.0 >= x0 && p.0 <= x1 && p.1 >= y0 && p.1 <= y1;
    sk.drawn()
        .into_iter()
        .filter(|&e| {
            let fig = proj.figure(sk, e, unit);
            !fig.is_empty() && fig.iter().flatten().all(within)
        })
        .collect()
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
