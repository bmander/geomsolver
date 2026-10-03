//! **The workspace**: the drawing as one scene in space, which an editor draws in.
//!
//! The glass box (`scene`, `scene3d`) shows a multiview document folded up; this is what makes the
//! same picture *editable*.  Every view's geometry is stored as page coordinates, and a view reads
//! them through its own placement on the page (`plane::in_view`) and stands them on its own plane
//! (`Basis::lift`).  Seen by an orthographic eye, that whole chain is **affine**: each view's page
//! maps onto the eye's picture plane by one 2×3 matrix.  So a front end needs no 3D arithmetic to
//! draw a sketch on a tilted plane, or to turn a click back into a place on one — it composes the
//! view's map with its own 2D camera and inverts a 2×2 — and every question about what is under
//! the pointer is asked here, of the figures as the eye sees them, because views that sit on top
//! of one another on the page are nowhere near one another in space.
//!
//! The page itself is a view: geometry in no plane stands on the front plane (`Basis::page`),
//! measured from the world origin.

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::model::{seg_distance, EntKind, EntRef, Sketch};
use crate::plane::{dot, Basis};

use super::{drawable, entity_view, eye, views};

/// An affine map of a view's page coordinates onto the eye's picture plane:
/// `(x, y) ↦ (m[0]·x + m[1]·y + m[2], m[3]·x + m[4]·y + m[5])`.
pub type Map = [f64; 6];

/// Where `p` lands under `m`.
pub fn apply(m: &Map, p: (f64, f64)) -> (f64, f64) {
    (m[0] * p.0 + m[1] * p.1 + m[2], m[3] * p.0 + m[4] * p.1 + m[5])
}

/// The map of a view placed on the page at `o` turned by the rotor `(c, s)`, standing on `basis`,
/// seen by the eye whose picture plane is spanned by `right` and `up`.
///
/// `in_view` reads `(a, b) = (c·dx + s·dy, −s·dx + c·dy)` with `(dx, dy) = p − o`, `lift` stands
/// that at `o₃ + a·u + b·v`, and the eye keeps `(right·X, up·X)`; multiplied out, each eye
/// coordinate is linear in `p`.
fn map_of(basis: &Basis, o: (f64, f64), c: f64, s: f64, right: [f64; 3], up: [f64; 3]) -> Map {
    let row = |e: [f64; 3]| {
        let (eu, ev) = (dot(e, basis.u), dot(e, basis.v));
        // ∂/∂x and ∂/∂y of eu·a + ev·b, and the constant the origin leaves
        let (mx, my) = (eu * c - ev * s, eu * s + ev * c);
        (mx, my, dot(e, basis.o) - mx * o.0 - my * o.1)
    };
    let (a, b, t) = row(right);
    let (d, e, f) = row(up);
    [a, b, t, d, e, f]
}

/// Every view's map for one position of the eye, and the view each point stands in.
///
/// Built per question rather than kept: it reads the sketch's current pose, and it costs one pass
/// over the planes and one over the points.
pub struct Projection {
    /// The eye's picture plane, as the two world directions its axes run along.
    right: [f64; 3],
    up: [f64; 3],
    page: Map,
    planes: Vec<Map>,
    /// `overview::view_of` for each point: its membership, or the plane it is a datum point of.
    views: Vec<Option<usize>>,
}

impl Projection {
    /// The maps as the eye at bearing `az` and elevation `el` (radians) sees them.
    pub fn new(sk: &Sketch, az: f64, el: f64) -> Projection {
        let (right, up) = eye(az, el);
        let planes = (0..sk.planes.len())
            .map(|i| {
                let f = &sk.planes[i].frame;
                let o = sk.point_xy(f.origin as usize);
                let (c, s) = (sk.params[f.c as usize].value, sk.params[f.s as usize].value);
                map_of(&sk.basis(i), o, c, s, right, up)
            })
            .collect();
        Projection {
            right,
            up,
            page: map_of(&Basis::page(), (0.0, 0.0), 1.0, 0.0, right, up),
            planes,
            views: views(sk),
        }
    }

    /// Where a point in space is seen.
    pub fn seen(&self, x: [f64; 3]) -> (f64, f64) {
        (dot(self.right, x), dot(self.up, x))
    }

    /// The map of a view — `None` is the page.
    pub fn map(&self, view: Option<usize>) -> &Map {
        view.map_or(&self.page, |i| &self.planes[i])
    }

    /// The view point `p` stands in.
    pub fn view_of(&self, p: usize) -> Option<usize> {
        self.views[p]
    }

    /// Where point `p` is seen.
    pub fn point(&self, sk: &Sketch, p: usize) -> (f64, f64) {
        apply(self.map(self.views[p]), sk.point_xy(p))
    }

    /// The one view an entity is drawn in, `None` where its points stand in different ones.
    /// Views whose maps agree are one place in space — the page and `std.front` are — so a figure
    /// with a point in each is still drawn in one.
    pub fn entity_view(&self, sk: &Sketch, e: EntRef) -> Result<Option<usize>, ()> {
        if let Some(v) = entity_view(sk, e, &self.views) {
            return Ok(Some(v));
        }
        let kids: Vec<usize> = if e.kind == EntKind::Point {
            vec![e.i()]
        } else {
            sk.children(e).iter().filter(|k| k.kind == EntKind::Point).map(|k| k.i()).collect()
        };
        let Some(&first) = kids.first() else { return Ok(None) };
        let v = self.views[first];
        let same = |m: &Map, n: &Map| m.iter().zip(n).all(|(a, b)| (a - b).abs() <= 1e-12 * (1.0 + a.abs()));
        if kids.iter().all(|&k| same(self.map(self.views[k]), self.map(v))) {
            Ok(v)
        } else {
            Err(())
        }
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
                let Ok(view) = self.entity_view(sk, e) else { return Vec::new() };
                let m = self.map(view);
                drawable(sk, e, unit)
                    .into_iter()
                    .map(|poly| poly.into_iter().map(|p| apply(m, p)).collect())
                    .collect()
            }
        }
    }

    /// The view a constraint's figure — its callout — is laid out in: the one every point it
    /// names stands in, or `None` where they do not agree.
    pub fn constraint_view(&self, sk: &Sketch, c: &crate::constraints::Constraint) -> Result<Option<usize>, ()> {
        let mut seen: Option<Option<usize>> = None;
        for a in &c.args {
            let crate::constraints::Arg::Ent(r) = a else { continue };
            let v = self.entity_view(sk, *r)?;
            match seen {
                None => seen = Some(v),
                Some(w) if w == v => {}
                Some(w) => {
                    if self.map(w) != self.map(v) {
                        return Err(());
                    }
                }
            }
        }
        Ok(seen.flatten())
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

/// The nearest a set of polylines comes to `p`.
fn figure_distance(fig: &[Vec<(f64, f64)>], p: (f64, f64)) -> f64 {
    let mut best = f64::INFINITY;
    for poly in fig {
        if poly.len() == 1 {
            best = best.min((p.0 - poly[0].0).dhypot(p.1 - poly[0].1));
        }
        for w in poly.windows(2) {
            best = best.min(seg_distance(p, w[0], w[1]));
        }
    }
    best
}

/// What a click at `at` on the eye's picture plane picks: `model::pick`'s rule — a point within
/// `tol` wins outright, else the nearest figure within it — asked of the figures where the eye sees
/// them.  `unit` is the eye length of one screen pixel, which refines a round figure.
pub fn pick(sk: &Sketch, proj: &Projection, at: (f64, f64), tol: f64, unit: f64) -> Option<EntRef> {
    if let (Some(i), d) = nearest_point(sk, proj, at) {
        if d <= tol {
            return Some(EntRef::point(i));
        }
    }
    let mut best: Option<(EntRef, f64)> = None;
    for e in sk.drawn() {
        if e.kind == EntKind::Point {
            continue;
        }
        let d = figure_distance(&proj.figure(sk, e, unit), at);
        if d <= tol && best.map_or(true, |(_, bd)| d < bd) {
            best = Some((e, d));
        }
    }
    best.map(|(e, _)| e)
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

/// The extent of everything the workspace shows, on the eye's picture plane, as
/// `(xmin, ymin, xmax, ymax)` — every drawn figure, and the box round each object seen from here
/// (its eight corners), since a solid reaches past the profiles it is made from: a turned section
/// draws half of what it makes.  `None` when nothing is drawn.
pub fn bounds(sk: &Sketch, proj: &Projection, unit: f64) -> Option<(f64, f64, f64, f64)> {
    let mut b: Option<(f64, f64, f64, f64)> = None;
    let mut grow = |p: (f64, f64)| {
        b = Some(match b {
            None => (p.0, p.1, p.0, p.1),
            Some((x0, y0, x1, y1)) => (x0.min(p.0), y0.min(p.1), x1.max(p.0), y1.max(p.1)),
        });
    };
    for e in sk.drawn() {
        for p in proj.figure(sk, e, unit).into_iter().flatten() {
            grow(p);
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
            grow(proj.seen([pick(0), pick(1), pick(2)]));
        }
    }
    b
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

/// The plane index `look_at` asks about, with the page as `None`.
pub fn look_at_view(sk: &Sketch, view: Option<usize>) -> (f64, f64) {
    look_at(&view.map_or_else(Basis::page, |i| sk.basis(i)))
}
