//! The drawing as the box it was unfolded from.
//!
//! A multiview document (§6.7) is several 2D pictures on one sheet, each on a stated plane in
//! space.  This folds them back up: every view stands on its own plane, and the object the views
//! are *of* is reconstructed in the middle.
//!
//! **Nothing here is solved for, and nothing is stored.**  The whole scene is arithmetic over
//! what the document already says:
//!
//! * a point drawn in view P has view coordinates `(a, b) = plane::in_view(…)` — the same
//!   reading `project`'s residual takes;
//! * every plane's origin is the image of one shared origin in space, so the point sits at
//!   `a·u_P + b·v_P` (`Basis::lift`);
//! * a corner tied by `project` into two non-parallel views is **over-determined and exact**:
//!   each image contributes `u_P·X = a` and `v_P·X = b`, so two views give four rows in three
//!   unknowns, consistent precisely *because* the projection holds.  The rank tells a corner
//!   that can be placed in space from one seen in a single view, which is a ray and stays on
//!   its plane.
//!
//! The **core projects and the front end strokes**, the seam `callout.rs` and `plane::glyph`
//! already sit on — so the scene comes out in 2D world coordinates and the app's camera, a 2D
//! similarity, maps it to the screen with no vector arithmetic of its own.

use crate::linalg::{min_norm_solve, Mat};
use crate::model::{grow, Box2, EntKind, EntRef, Sketch};
use crate::plane::{dot, Basis};
use std::collections::BTreeMap;

/// The rank tolerance a corner is placed at, relative to the largest singular value of its four
/// rows.  Two views make a corner only if they are *views* — for planes at an angle θ the
/// smallest singular value is about sin θ / √2, so this says two planes within about a degree
/// of parallel are one view, and a corner seen in them is a ray.  A looser rule (1e-9, the
/// ordinary dimensionless one) passed such a pair and then amplified whatever residual the
/// projection carried by 1/σ₃, flinging the corner across the page; `validate` refuses only the
/// exactly-parallel pair, and an explicit `u:`/`v:` basis can be as close as it likes.
const RCOND: f64 = 1e-2;

/// Two reconstructed points closer than this, as a fraction of the sketch's extent, are one
/// point: a corner's position comes out of an independent least-squares solve per pair of
/// views, and agrees between pairs only to the solve's tolerance, never to the bit.
const SAME_POINT: f64 = 1e-4;

/// How far a plane's face is drawn past the geometry standing on it, as a fraction of that
/// geometry's extent — enough to read as a pane of the box rather than a tight box round the
/// picture.
const FACE_MARGIN: f64 = 0.15;

/// The side a pane may not be thinner than, as a fraction of the sketch's extent.  **Every
/// plane is a pane**, drawn in or not: a view is a place to draw, and one that does not show
/// until something is in it cannot be gone to.  So a view with nothing in it — or with only its
/// own origin, or only points along one line — is still a square about its origin, sized off the
/// drawing so it reads as a pane beside the others rather than as nothing at all.
const LEAST_SIDE: f64 = 0.5;

/// One polyline of the scene, already projected to 2D, and what it came from.
#[derive(Clone, Debug)]
pub struct Item {
    /// The entity it is drawn from — the front end resolves style and selection through this,
    /// exactly as it does on the sheet.  `None` for the parts of the scene no entity owns: the
    /// reconstructed solid.
    pub of: Option<EntRef>,
    /// **The view it belongs to.**  A pane and its axes are their plane's; a drawn polyline is
    /// the plane its entity is in; the solid is of no one view, being the thing the views are
    /// *of*.  This is what lets a front end say "go to the view I just double-clicked" without
    /// asking which plane an entity is in a second time and in its own words.
    pub in_plane: Option<EntRef>,
    pub what: Part,
    pub pts: Vec<(f64, f64)>,
}

/// What a scene item *is*, so the front end can ink the three kinds apart.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Part {
    /// A view's own geometry, standing on its plane.
    Drawn,
    /// A plane's face: the pane of the glass box.
    Face,
    /// A plane's own x and y at its origin — which way the view's coordinates run, which is the
    /// one thing a pane alone cannot say.
    Axis,
    /// The object itself, reconstructed from the views.
    Solid,
}

impl Part {
    pub fn as_str(self) -> &'static str {
        match self {
            Part::Drawn => "drawn",
            Part::Face => "face",
            Part::Axis => "axis",
            Part::Solid => "solid",
        }
    }
}

/// The whole overview, in 2D world coordinates.
#[derive(Clone, Debug, Default)]
pub struct Scene {
    pub items: Vec<Item>,
    /// (xmin, ymin, xmax, ymax) over everything drawn — what a "fit to screen" wants.
    pub bounds: (f64, f64, f64, f64),
}

/// One corner of the object: the two images that place it, and where it is.
#[derive(Clone, Copy, Debug)]
pub struct Corner {
    /// The two images, **ordered by the index of the plane each is drawn in** — never by the
    /// order the `project` statement happened to name them, which is a fact about the source
    /// and not about the corner, and which the edge walk below must not depend on.
    pub images: [usize; 2],
    pub at: [f64; 3],
}

/// Every corner the document places, one per projection.
///
/// **A pair, deliberately, and not a transitive class.**  `a project b` says two images are of
/// one point, but an image may serve *two* corners: in the front view the near and far ends of a
/// vertical edge coincide, so `bracket.sv` states both `Ff project Fa` and `Ff project F2a` and
/// means two different corners of the part.  Merged transitively they become one, and the object
/// collapses along every edge that runs away from a view.  Two images are what place a point
/// anyway — `u_P·X = a`, `v_P·X = b` twice over is four rows in three unknowns, and consistent
/// exactly *because* the projection holds — so the pair is both the honest unit and the
/// sufficient one.  Kept at **rank 3**, which is "two views that are not parallel"; `validate`
/// refuses the parallel pair at the add, so a rank-2 answer here means the drawing moved since.
///
/// The **plane origins are corners too**, with no projection stated between them: every plane's
/// origin is the image of one shared origin in space, which is the convention the whole scheme
/// is written against (`Basis::lift`), so they are paired here rather than left to a document to
/// say twice.
pub fn corners(sk: &Sketch) -> Vec<Corner> {
    corners_in(sk, &views(sk))
}

fn corners_in(sk: &Sketch, views: &[Option<usize>]) -> Vec<Corner> {
    let mut pairs: Vec<(usize, usize)> = Vec::new();
    for c in &sk.constraints {
        // a claim is judged, never solved for: it relates nothing, here as in the solver
        if c.kind == crate::constraints::CKind::Project && !c.claim {
            pairs.push((c.args[0].ent().i(), c.args[1].ent().i()));
        }
    }
    // the origins, pairwise: they are images of one point by construction
    let origins: Vec<usize> = (0..sk.planes.len())
        .map(|i| sk.planes[i].frame.origin as usize)
        .collect();
    for i in 0..origins.len() {
        for j in i + 1..origins.len() {
            pairs.push((origins[i], origins[j]));
        }
    }
    let mut out = Vec::new();
    for (a, b) in pairs {
        let (Some(pa), Some(pb)) = (views[a], views[b]) else { continue };
        if pa == pb {
            continue;
        }
        // by plane, so that two corners of one edge compare image to image whatever order
        // their two statements were written in
        let ((a, pa), (b, pb)) = if pa < pb { ((a, pa), (b, pb)) } else { ((b, pb), (a, pa)) };
        let mut rows: Vec<f64> = Vec::with_capacity(12);
        let mut rhs: Vec<f64> = Vec::with_capacity(4);
        for (i, p) in [(a, pa), (b, pb)] {
            let basis = sk.planes[p].basis;
            let (x, y) = view_xy(sk, p, sk.point_xy(i));
            rows.extend_from_slice(&basis.u);
            rhs.push(x);
            rows.extend_from_slice(&basis.v);
            rhs.push(y);
        }
        let m = Mat::from_vec(4, 3, rows);
        let (x, rank) = min_norm_solve(&m, &rhs, RCOND);
        if rank == 3 {
            out.push(Corner { images: [a, b], at: [x[0], x[1], x[2]] });
        }
    }
    out
}

/// The view a point **stands in** when the box is folded up: the plane it is a member of, or —
/// for a point that is a plane's own origin or `toward` point and a member of none — that
/// plane.  Membership is what `project` reads and what `in` writes, and a datum's points are
/// deliberately outside it (they place the view; they are not drawn in it), but in space they
/// are nowhere else: every plane's origin is the one shared origin, which is the convention the
/// whole reconstruction is written against.  `None` is page geometry, a picture of nothing.
fn view_of(sk: &Sketch, p: usize) -> Option<usize> {
    sk.plane_of(p).or_else(|| {
        sk.planes
            .iter()
            .position(|pl| pl.frame.origin as usize == p || pl.frame.toward as usize == p)
    })
}

/// `view_of` for every point at once — asked per point per plane by the panes, per image by
/// the corners and per end by the lines, so it is answered once and read.
fn views(sk: &Sketch) -> Vec<Option<usize>> {
    (0..sk.points.len()).map(|p| view_of(sk, p)).collect()
}

/// A point of the page, read in the view it is drawn in.
pub fn view_xy(sk: &Sketch, plane: usize, p: (f64, f64)) -> (f64, f64) {
    let f = &sk.planes[plane].frame;
    let o = sk.point_xy(f.origin as usize);
    let (c, s) = (sk.params[f.c as usize].value, sk.params[f.s as usize].value);
    crate::plane::in_view(c, s, o, p)
}

/// Where a page point drawn in `plane` — or on the page itself, when it is in none — sits in
/// space.  Geometry with no membership lies on the page plane, measured from the world origin,
/// which is what "a point with none is simply on the page" already means.
fn in_space(sk: &Sketch, plane: Option<usize>, p: (f64, f64)) -> [f64; 3] {
    let (basis, (a, b)) = match plane {
        Some(i) => (sk.planes[i].basis, view_xy(sk, i, p)),
        None => (Basis::page(), p),
    };
    basis.lift(a, b)
}

/// The view an entity stands in: the one every point it is made of stands in, or `None` where
/// they disagree or it has none — `program::plane_of_entity`'s walk with `view_of`'s reading of
/// a point in place of bare membership, for the reason given there.
fn entity_view(sk: &Sketch, e: EntRef, views: &[Option<usize>]) -> Option<usize> {
    crate::program::plane_of_entity_by(sk, e, |p| views[p])
}

/// What an entity is **drawn as**, in world coordinates, as polylines.
///
/// The per-kind walk `svg::entity` and `paint.ts` each make for their own output — one into SVG
/// strings, one into canvas paths — said once as geometry.  A round kind has no polyline
/// anywhere else in the core, because on a sheet it is drawn as an arc; folded onto a tilted
/// plane it is not one, so it is tessellated here against the same screen flatness
/// `curve::tessellate` refines to.
pub fn drawable(sk: &Sketch, e: EntRef, unit: f64) -> Vec<Vec<(f64, f64)>> {
    let i = e.i();
    match e.kind {
        // nothing on the page: a face is the edges the document already drew, and what is drawn
        // of a solid is a derived view, which is its own geometry
        EntKind::Face | EntKind::Solid => Vec::new(),
        EntKind::Point => vec![vec![sk.point_xy(i)]],
        EntKind::Line => {
            let l = &sk.lines[i];
            vec![vec![sk.point_xy(l.p1 as usize), sk.point_xy(l.p2 as usize)]]
        }
        EntKind::Circle => {
            let c = sk.point_xy(sk.round_center(e));
            let r = sk.radius_value(e).abs();
            let mut pts = round(c, r, 0.0, std::f64::consts::TAU, unit);
            pts.push(pts[0]);   // a rim is closed
            vec![pts]
        }
        EntKind::Arc => {
            let c = sk.point_xy(sk.round_center(e));
            let r = sk.radius_value(e).abs();
            let (a0, a1) = sk.arc_angles(i);
            vec![round(c, r, a0, a1 - a0, unit)]
        }
        EntKind::Spline => vec![crate::curve::tessellate(sk, i, unit)],
        EntKind::Curve => vec![sk.curve_polyline(i)],
        // a datum's glyph is already two segments of world geometry
        EntKind::Plane => crate::plane::glyph(sk, i, unit).iter().map(|(a, b)| vec![*a, *b]).collect(),
    }
}

/// A circular sweep, refined to the same screen flatness a curve is: the chord of a step strays
/// less than `FLATNESS_PX` pixels from the rim, so a big circle gets more steps than a small one
/// and a zoomed-out one fewer.
fn round(c: (f64, f64), r: f64, from: f64, sweep: f64, unit: f64) -> Vec<(f64, f64)> {
    let tol = crate::curve::flatness(unit);
    // sagitta of a step of angle θ on radius r is r(1 − cos(θ/2)); invert for θ
    let step = if r > tol { 2.0 * (1.0 - tol / r).acos() } else { std::f64::consts::TAU };
    let n = ((sweep.abs() / step).ceil() as usize).clamp(2, 4096);
    (0..=n)
        .map(|k| {
            let a = from + sweep * k as f64 / n as f64;
            (c.0 + r * a.cos(), c.1 + r * a.sin())
        })
        .collect()
}

/// The whole scene, projected.
///
/// `az` and `el` are the orbit, in radians: the direction the box is looked at from.  The
/// projection is **orthographic** — a drawing is orthographic, and a perspective one would make
/// the views it is folded from stop being the views.
pub fn scene(sk: &Sketch, unit: f64, az: f64, el: f64) -> Scene {
    let (right, up) = eye(az, el);
    let flat = |p: [f64; 3]| (dot(p, right), dot(p, up));
    let mut items: Vec<Item> = Vec::new();
    let views = views(sk);
    let least = sk.extent() * LEAST_SIDE;

    // the panes of the box, under everything — **one per plane, drawn in or not** — each with
    // its own axes at its origin
    for i in 0..sk.planes.len() {
        let of = Some(EntRef::plane(i));
        let basis = sk.planes[i].basis;
        let rect = pane(sk, i, &views, least);
        items.push(Item {
            of,
            in_plane: of,
            what: Part::Face,
            pts: face(&basis, rect).into_iter().map(&flat).collect(),
        });
        for arm in axes(&basis, rect) {
            items.push(Item {
                of,
                in_plane: of,
                what: Part::Axis,
                pts: arm.into_iter().map(&flat).collect(),
            });
        }
    }
    // every view's own geometry, standing on its plane
    for e in sk.drawn() {
        // a point is a place, and a plane's glyph is the sheet's way of showing a datum — in
        // the box the pane says it
        if matches!(e.kind, EntKind::Point | EntKind::Plane) {
            continue;
        }
        let plane = entity_view(sk, e, &views);
        // a line stands with each end where that end is: a projector between two views, or a
        // line from a datum's own origin into its view, is neither a stray stroke on the page
        // nor anyone's — it belongs to a view only when both its ends do
        if e.kind == EntKind::Line {
            let l = &sk.lines[e.i()];
            let end = |q: u32| flat(in_space(sk, views[q as usize], sk.point_xy(q as usize)));
            items.push(Item {
                of: Some(e),
                in_plane: plane.map(EntRef::plane),
                what: Part::Drawn,
                pts: vec![end(l.p1), end(l.p2)],
            });
            continue;
        }
        for poly in drawable(sk, e, unit) {
            if poly.len() < 2 {
                continue;
            }
            items.push(Item {
                of: Some(e),
                in_plane: plane.map(EntRef::plane),
                what: Part::Drawn,
                pts: poly.into_iter().map(|p| flat(in_space(sk, plane, p))).collect(),
            });
        }
    }
    // and the object itself.  **An edge is one that is seen as an edge in both views**: two
    // corners whose images are joined by a line in each of the two views that place them.  That
    // is what the pairing above leaves as the honest rule — a corner is placed by two views, so
    // an edge between two corners is drawn only where those same two views both draw it, and a
    // line running away from a view (which that view sees as a point) never invents one.
    // Image to image is well-defined because a corner's images are ordered by plane.
    let cs = corners_in(sk, &views);
    let lines: BTreeMap<(usize, usize), usize> = sk
        .lines
        .iter()
        .enumerate()
        .map(|(i, l)| ((l.p1.min(l.p2) as usize, l.p1.max(l.p2) as usize), i))
        .collect();
    let joined = |a: usize, b: usize| lines.get(&(a.min(b), a.max(b))).copied();
    // **Keyed by the edge in space, not by the line that drew it.**  One line of one view may be
    // two edges of the object — in the front view the near and far edges of the inclined face
    // are the same stroke — so deduplicating by the source line collapses exactly the edges a
    // box is drawn to show.  Two pairs of views that agree on an edge give the same endpoints
    // to within the solve, and that is the thing to have once: compared by distance, since each
    // corner is its own least-squares answer and two of them agree to a tolerance, not to a bit.
    let tol = sk.extent() * SAME_POINT;
    let same = |p: [f64; 3], q: [f64; 3]| {
        (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2) <= tol * tol
    };
    let mut edges: Vec<([f64; 3], [f64; 3])> = Vec::new();
    for (i, c1) in cs.iter().enumerate() {
        for c2 in cs.iter().skip(i + 1) {
            let Some(li) = joined(c1.images[0], c2.images[0]) else { continue };
            if joined(c1.images[1], c2.images[1]).is_none() {
                continue;
            }
            // one edge however many pairs of views agree on it
            if edges.iter().any(|&(a, b)| {
                (same(a, c1.at) && same(b, c2.at)) || (same(a, c2.at) && same(b, c1.at))
            }) {
                continue;
            }
            edges.push((c1.at, c2.at));
            items.push(Item {
                of: Some(EntRef::line(li)),
                // of no one view: it is the thing the views are of
                in_plane: None,
                what: Part::Solid,
                pts: vec![flat(c1.at), flat(c2.at)],
            });
        }
    }
    let mut bounds = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for it in &items {
        for &p in &it.pts {
            grow(&mut bounds, p);
        }
    }
    if !bounds.0.is_finite() {
        bounds = (0.0, 0.0, 1.0, 1.0);
    }
    Scene { items, bounds }
}

/// The rectangle a view is drawn on, in its own view coordinates: the box of everything standing
/// on that plane *and its origin* — where its axes cross, so a pane never cuts them off — grown
/// a little, and never thinner than `least` (`LEAST_SIDE` of the sketch's extent) either way,
/// which is what a view with nothing in it, or only its origin, or only points along one line,
/// gets.
///
/// One rule for the face and its axes both, so the two cannot come to disagree about how far a
/// view reaches.
fn pane(sk: &Sketch, plane: usize, views: &[Option<usize>], least: f64) -> Box2 {
    let mut b = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for (i, v) in views.iter().enumerate() {
        if *v == Some(plane) {
            grow(&mut b, view_xy(sk, plane, sk.point_xy(i)));
        }
    }
    let widen = |lo: f64, hi: f64| {
        let short = (least - (hi - lo)).max(0.0) / 2.0;
        (lo - short, hi + short)
    };
    let ((x0, x1), (y0, y1)) = (widen(b.0, b.2), widen(b.1, b.3));
    let m = (x1 - x0).max(y1 - y0) * FACE_MARGIN;
    (x0 - m, y0 - m, x1 + m, y1 + m)
}

/// The pane as a closed polygon in space.
fn face(basis: &Basis, (x0, y0, x1, y1): Box2) -> Vec<[f64; 3]> {
    [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        .iter()
        .map(|&(x, y)| basis.lift(x, y))
        .collect()
}

/// A plane's own x and y: the two lines of its view coordinates, **run right across its pane**
/// and crossing at its origin.
///
/// The same mark the sheet draws across the canvas, folded up — which is the point of drawing it
/// at all: a pane is a little sheet, and what makes it read as one is its axes.  A short tick at
/// the origin would be truer to a datum glyph and is what this was; at the size a box is looked
/// at, it disappeared into whatever the view had drawn near its corner.
fn axes(basis: &Basis, (x0, y0, x1, y1): Box2) -> [Vec<[f64; 3]>; 2] {
    [
        vec![basis.lift(x0, 0.0), basis.lift(x1, 0.0)],
        vec![basis.lift(0.0, y0), basis.lift(0.0, y1)],
    ]
}

/// The screen axes of an orbit: where `right` and `up` point in space, looking from bearing
/// `az` and elevation `el`.  Orthonormal, so the projection carries lengths faithfully in any
/// plane facing the viewer — which is what makes the front view flatten to the sheet's own
/// picture when the box is looked at square on.
fn eye(az: f64, el: f64) -> ([f64; 3], [f64; 3]) {
    let (sa, ca) = az.sin_cos();
    let (se, ce) = el.sin_cos();
    // the viewer stands at (ca·ce, sa·ce, se) and looks back at the origin
    let right = [-sa, ca, 0.0];
    let up = [-ca * se, -sa * se, ce];
    (right, up)
}
