//! The drawing as the box it was unfolded from.
//!
//! A multiview document (§6.7) is several 2D pictures, each drawn in a plane in space.  This
//! folds them back up: every view stands on its own plane, and the object the views are *of* is
//! reconstructed in the middle.
//!
//! **Nothing here is solved for, and nothing is stored.**  The whole scene is arithmetic over
//! what the document already says:
//!
//! * a point drawn in view P has view coordinates `(a, b)`, its own coordinates in P — the
//!   reading `project`'s residual takes;
//! * the point sits at `o_P + a·u_P + b·v_P` (`Basis::lift`);
//! * a corner tied by `project` into two non-parallel views is **over-determined and exact**:
//!   each image contributes `u_P·X = a` and `v_P·X = b`, so two views give four rows in three
//!   unknowns, consistent precisely *because* the projection holds.  The rank tells a corner
//!   that can be placed in space from one seen in a single view, which is an axis and stays on
//!   its plane.
//!
//! The **core projects and the front end strokes**, the seam `callout.rs` and `plane::glyph`
//! already sit on — so the scene comes out in 2D world coordinates and the app's camera, a 2D
//! similarity, maps it to the screen with no vector arithmetic of its own.

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::linalg::{min_norm_solve, Mat};
use crate::model::{grow, Box2, EntKind, EntRef, Sketch};
pub use crate::model::objects;
use crate::plane::{dot, Basis};
use crate::space::eye;
use std::collections::BTreeMap;

pub mod outline;
pub mod region;
pub mod scenario;
pub mod workspace;

/// The rank tolerance a corner is placed at, relative to the largest singular value of its four
/// rows.  Two views make a corner only if they are *views* — for planes at an angle θ the
/// smallest singular value is about sin θ / √2, so this says two planes within about a degree
/// of parallel are one view, and a corner seen in them is an axis.  A looser rule (1e-9, the
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

/// How wide the workspace is at the least, in millimetres: about five inches, the sheet a part is
/// drawn on at a desk.  A pane is never narrower than this, an axis reaches half of it either side
/// of its foot, and a drawing with nothing yet drawn opens framed on it (`workspace::bounds`) —
/// sized off the drawing alone, a blank sheet's `std` was a millimetre across.  A drawing larger
/// than this sizes its workspace itself.
const WORKSPACE_MM: f64 = 125.0;

/// `WORKSPACE_MM` in the document's own lengths.
pub(crate) fn workspace_span(sk: &Sketch) -> f64 {
    WORKSPACE_MM / sk.units.mm_per()
}

/// How large a datum is drawn beside an assembly shown on its parts (#679), as a fraction of the
/// largest side of the box round its objects: a small frame about its origin, the size of a part
/// or so, rather than the workspace or the drawing every part's insides reach.
const PART_DATUM: f64 = 0.5;

/// **How large the panes are drawn** — and the axes, which reach as far: no thinner than `least`
/// either way and no narrower than `span`, round what stands on each.  `shown_only` counts only
/// what is shown (`Sketch::shown`), so a closed part's sketches do not size the assembly's
/// planes.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Panes {
    least: f64,
    span: f64,
    shown_only: bool,
    /// How far either side of its foot an axis is drawn.
    pub(crate) reach: f64,
}

/// The rule the panes and axes are drawn by: off the drawing and the workspace (`WORKSPACE_MM`),
/// or — an assembly shown on its parts (`Sketch::on_parts`, #679) with an object built — off the
/// objects alone (`PART_DATUM`), what is hidden left out.
pub(crate) fn panes(sk: &Sketch) -> Panes {
    if sk.on_parts {
        if let Some(side) = objects_side(sk) {
            let span = side * PART_DATUM;
            return Panes { least: span, span, shown_only: true, reach: span / 2.0 };
        }
    }
    let (extent, span) = (sk.extent(), workspace_span(sk));
    Panes { least: extent * LEAST_SIDE, span, shown_only: false, reach: extent.max(span / 2.0) }
}

/// The largest side of the box round every object shown and built, or `None` with none —
/// remembered against the drawing and the objects shown (`Caches::objects_side`).
fn objects_side(sk: &Sketch) -> Option<f64> {
    let shown = shown_objects(sk);
    // and against a shape changed in place, and the swept surfaces a host has supplied since —
    // compared where they stand, a pick asking it on every pointer move
    let epochs = [sk.shape_epoch as usize, sk.field_surfaces.borrow().len()];
    if let Some((at, then, of, side)) = &*sk.caches.objects_side.borrow() {
        let same = at.len() == sk.params.len()
            && at.iter().zip(&sk.params).all(|(a, p)| *a == p.value);
        if same && *then == epochs && *of == shown {
            return *side;
        }
    }
    let side = side_of(sk, &shown);
    *sk.caches.objects_side.borrow_mut() = Some((sk.get_x(), epochs, shown, side));
    side
}

/// The largest side of the box round each of `objects` that builds.
fn side_of(sk: &Sketch, objects: &[usize]) -> Option<f64> {
    let mut b = crate::solid::Box3::empty();
    for &i in objects {
        let Ok(s) = sk.evaluated_solid(i, crate::solid::ApproximationPolicy::Mesh) else {
            continue;
        };
        let w = s.world_bounds();
        if !w.is_empty() {
            b.add(w.lo);
            b.add(w.hi);
        }
    }
    let side = b.side();
    (side > 0.0).then_some(side)
}

/// How many of its own widths across the box is refined for, when `scene3d` is asked for the
/// object's own scale rather than the sheet's zoom.  It is a *drawing* is being cut into
/// polylines, so the number that matters is how many pixels the box is looked at across, and a
/// few hundred is the honest answer for a panel on a screen.  Scale-free — the same in
/// millimetres and in inches — which is the whole point: refined this way, a zoom moves the
/// camera and rebuilds nothing.
const SCENE_PX: f64 = 800.0;

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
    /// For a `Shell` face, how squarely it faces the light: 0 for edge-on, 1 for full on.
    ///
    /// A *number* and not a colour, because which is which is the front end's chrome and the
    /// geometry is the core's — the same division `Part` itself draws.  `None` for everything
    /// that is a line rather than a surface.
    pub shade: Option<f64>,
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
    /// The object itself: its edges, reconstructed from the views or read off the term.
    Solid,
    /// **A face of the solid**, filled — what "show solid" asks for.  Back-facing surfaces are
    /// dropped and the rest come **far first**, so a front end that paints them in the order it
    /// is given gets the near ones over the far ones and needs no depth buffer.
    Shell,
}

impl Part {
    pub fn as_str(self) -> &'static str {
        match self {
            Part::Drawn => "drawn",
            Part::Face => "face",
            Part::Axis => "axis",
            Part::Solid => "solid",
            Part::Shell => "shell",
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
/// The rows read an image's coordinates as measured from the world origin: a plane's own `o` is
/// not added in, so the corner is where `Basis::lift` puts it only for planes whose origins
/// stand there.
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
            // drawn at (x, y), the image stands at o + x·u + y·v: u·X = x + u·o, v·X = y + v·o
            let basis = sk.basis(p);
            let (x, y) = sk.point_xy(i);
            rows.extend_from_slice(&basis.u);
            rhs.push(x + crate::space::dot(basis.u, basis.o));
            rows.extend_from_slice(&basis.v);
            rhs.push(y + crate::space::dot(basis.v, basis.o));
        }
        let m = Mat::from_vec(4, 3, rows);
        let (x, rank) = min_norm_solve(&m, &rhs, RCOND);
        if rank == 3 {
            out.push(Corner { images: [a, b], at: [x[0], x[1], x[2]] });
        }
    }
    out
}

/// The plane a point **stands in**: its membership.  `None` is a point in space, or a point of
/// a 2D sketch.
fn view_of(sk: &Sketch, p: usize) -> Option<usize> {
    sk.plane_of(p)
}

/// `view_of` for every point at once — asked per point per plane by the panes, per image by
/// the corners and per end by the lines, so it is answered once and read.
fn views(sk: &Sketch) -> Vec<Option<usize>> {
    (0..sk.points.len()).map(|p| view_of(sk, p)).collect()
}


/// The view an entity stands in: the one every point it is made of stands in, or `None` where
/// they disagree or it has none — `program::plane_of_entity`'s walk over the `views` table, read
/// once for every point.  A curve owns no point, and is drawn in its tool's view
/// (`Sketch::curve_view`).
fn entity_view(sk: &Sketch, e: EntRef, views: &[Option<usize>]) -> Option<usize> {
    match e.kind {
        EntKind::Curve => sk.curve_view(e.i()),
        _ => crate::program::plane_of_entity_by(sk, e, |p| views[p]),
    }
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
        EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Meshing | EntKind::Traced => Vec::new(),
        // nothing on its view's page either: an axis is in space (`scene3d` draws it there)
        EntKind::Axis => Vec::new(),
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
        // a plane's glyph is already two segments, in its own coordinates
        EntKind::Frame => crate::plane::glyph(unit).iter().map(|(a, b)| vec![*a, *b]).collect(),
    }
}

/// A circular sweep, refined to the same screen flatness a curve is: the chord of a step strays
/// less than `FLATNESS_PX` pixels from the rim, so a big circle gets more steps than a small one
/// and a zoomed-out one fewer.
fn round(c: (f64, f64), r: f64, from: f64, sweep: f64, unit: f64) -> Vec<(f64, f64)> {
    let tol = crate::curve::flatness(unit);
    // sagitta of a step of angle θ on radius r is r(1 − cos(θ/2)); invert for θ
    let step = if r > tol { 2.0 * (1.0 - tol / r).dacos() } else { std::f64::consts::TAU };
    let n = ((sweep.abs() / step).ceil() as usize).clamp(2, 4096);
    (0..=n)
        .map(|k| {
            let a = from + sweep * k as f64 / n as f64;
            (c.0 + r * a.dcos(), c.1 + r * a.dsin())
        })
        .collect()
}

/// The whole scene, projected.
///
/// `az` and `el` are the orbit, in radians: the direction the box is looked at from.  The
/// projection is **orthographic** — a drawing is orthographic, and a perspective one would make
/// the views it is folded from stop being the views.
/// The scene, with the object shown as edges alone — what the box has always drawn.
pub fn scene(sk: &Sketch, unit: f64, az: f64, el: f64) -> Scene {
    scene_with(sk, unit, az, el, false)
}

/// One polyline of the scene **in space**, before any projection.
#[derive(Clone, Debug)]
pub struct Item3 {
    pub of: Option<EntRef>,
    pub in_plane: Option<EntRef>,
    pub what: Part,
    pub pts: Vec<[f64; 3]>,
}

/// How many spans a curve traced in space is first cut into, and how many times a span is halved
/// at most where its chord strays further from the curve than the flatness.
const TRACED_SPANS: usize = 16;
const TRACED_DEPTH: u32 = 8;

/// **A curve traced in space, as a polyline in space** (#363): its point read at places over its
/// formal's domain (`traced::reading`), each span halved where the curve at its middle strays
/// from the chord by more than the sheet's flatness at `unit` (`curve::flatness`).  Empty for a
/// surface, and where the body cannot be read.
pub fn traced_polyline(sk: &Sketch, i: usize, unit: f64) -> Vec<[f64; 3]> {
    use crate::space::{add, distance, scale};
    let Some(t) = sk.traced.get(i).filter(|t| t.formals() == 1) else { return Vec::new() };
    let [lo, hi] = t.domain()[0];
    let at = |u: f64| crate::traced::reading(sk, i, [u, 0.0], crate::traced::Need::Point)
        .map(|r| r.position);
    let tol = crate::curve::flatness(unit);
    fn refine(at: &dyn Fn(f64) -> Option<[f64; 3]>, (a, pa): (f64, [f64; 3]),
              (b, pb): (f64, [f64; 3]), tol: f64, depth: u32, out: &mut Vec<[f64; 3]>) {
        let m = 0.5 * (a + b);
        if let Some(pm) = at(m).filter(|_| depth > 0) {
            if distance(pm, scale(add(pa, pb), 0.5)) > tol {
                refine(at, (a, pa), (m, pm), tol, depth - 1, out);
                refine(at, (m, pm), (b, pb), tol, depth - 1, out);
                return;
            }
        }
        out.push(pb);
    }
    let Some(first) = at(lo) else { return Vec::new() };
    let mut out = vec![first];
    let mut last = (lo, first);
    for k in 1..=TRACED_SPANS {
        let u = lo + (hi - lo) * k as f64 / TRACED_SPANS as f64;
        let Some(p) = at(u) else { return Vec::new() };
        refine(&at, last, (u, p), tol, TRACED_DEPTH, &mut out);
        last = (u, p);
    }
    out
}

/// **The scene as it stands in space**, for a front end that has a camera of its own.
///
/// `scene` flattens because a 2D canvas must be handed something it can stroke; a renderer with a
/// depth buffer wants the box itself and will do its own projecting.  Both come out of the same
/// helpers — `pane` is still the one rule for how far a view reaches, `drawable` still the one
/// per-kind walk — so the two are one scene seen two ways and cannot come to disagree about what
/// is in it.
///
/// The **solid is not here**: a mesh is not a polyline, and a renderer with a depth buffer wants
/// `mesh::grouped` rather than the hidden-line edge set `scene` computes for a canvas that has no
/// way to occlude anything.  That is the whole saving of doing this in 3D.
pub fn scene3d(sk: &Sketch, unit: f64) -> Vec<Item3> {
    scene3d_of(sk, unit, Layer::All)
}

/// Which of the scene a renderer asks for.  The datums — panes, their axes, the axes in space —
/// follow every frame of a drag that moves them, and cost little; the objects' creases are
/// re-cut once a pose settles; the rest of what is drawn is the sheet's to stroke, asked only for
/// the whole scene.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Layer {
    All,
    Datums,
    Objects,
}

/// The scene3d, or one layer of it.
pub fn scene3d_of(sk: &Sketch, unit: f64, layer: Layer) -> Vec<Item3> {
    // **Nothing in the box follows the sheet's zoom.**  `unit` is the world length of one screen
    // pixel, which is the right refinement for a drawing being looked at on a page — and this is
    // not that: it is a scene handed to a renderer with a camera and a depth buffer of its own,
    // which zooms by moving the camera.  Refined by `unit`, every wheel tick re-evaluated the
    // term (158 ms for the V-twin cylinder's edges and 332 ms for its mesh, natively, and finer
    // without bound as you zoomed in), while an orbit moved nothing.  So a caller says `0` and
    // gets the scale-free rule `mesh_unit` already states for the mesh: refine to the object.
    let unit = if unit > 0.0 { unit } else { (sk.extent() / SCENE_PX).max(crate::solid::REPORT_UNIT) };
    let mut items: Vec<Item3> = Vec::new();
    let (datums, objects_too) = (layer != Layer::Objects, layer != Layer::Datums);
    let views = views(sk);
    let places = places(sk);
    let rule = panes(sk);
    // one pane for each place, its first plane's — none for a plane the outline hid (#435), and
    // none of what follows either
    for i in (0..sk.frames.len())
        .filter(|&i| datums && places[i] == Some(i) && sk.shown(EntRef::frame(i))) {
        let of = Some(EntRef::frame(i));
        let basis = sk.basis(i);
        let rect = pane(sk, i, &views, &places, &rule);
        items.push(Item3 { of, in_plane: of, what: Part::Face, pts: face(&basis, rect) });
        for arm in axes(&basis, rect) {
            items.push(Item3 { of, in_plane: of, what: Part::Axis, pts: arm.to_vec() });
        }
    }
    // the axes already drawn, each as its line: a later one lying on one of them is not drawn again
    let mut lines: Vec<([f64; 3], [f64; 3])> = Vec::new();
    for e in sk.seen().into_iter().filter(|_| datums) {
        if matches!(e.kind, EntKind::Point | EntKind::Frame) {
            continue;
        }
        if layer == Layer::Datums && e.kind != EntKind::Axis {
            continue;
        }
        let plane = entity_view(sk, e, &views);
        let in_plane = plane.map(EntRef::frame);
        // an axis is in space and in no view: its line across the drawing's reach, and a head at
        // the end it points to.  One lying on an axis already drawn (`std.back` is `std.x`
        // reversed) is that line, drawn once, the first declared's: a second head at the other
        // end would read as one axis pointing both ways, and a click there picks the first
        // (`workspace::nearest_axis`) anyway
        if e.kind == EntKind::Axis {
            use crate::space::{across, add, cross, norm, scale, sub};
            let reach = rule.reach;
            let Some((tail, tip, d)) = axis_segment(sk, e.i(), reach) else { continue };
            let foot = scale(add(tail, tip), 0.5);
            let tol = 1e-9 * reach;
            let on = |&(p, q): &([f64; 3], [f64; 3])| {
                norm(cross(d, q)) < 1e-9 && norm(cross(sub(foot, p), q)) < tol
            };
            if lines.iter().any(on) {
                continue;
            }
            lines.push((foot, d));
            items.push(Item3 { of: Some(e), in_plane: None, what: Part::Drawn, pts: vec![tail, tip] });
            let u = across(d).0;
            let head = reach * 0.04;
            for sgn in [1.0, -1.0] {
                let back = add(tip, add(scale(d, -head), scale(u, sgn * head * 0.5)));
                items.push(Item3 { of: Some(e), in_plane: None, what: Part::Drawn, pts: vec![tip, back] });
            }
            continue;
        }
        // a line stands with each end where that end is — the same rule as on the flat path,
        // and the reason a projector between two views belongs to neither
        if e.kind == EntKind::Line {
            let l = &sk.lines[e.i()];
            let end = |q: u32| sk.world_point(q as usize);
            items.push(Item3 {
                of: Some(e),
                in_plane,
                what: Part::Drawn,
                pts: vec![end(l.p1), end(l.p2)],
            });
            continue;
        }
        // a point in space traced over a parameter stands in space, in no view (#398 F)
        if e.kind == EntKind::Curve {
            let pts = sk.followed_world(e.i());
            if pts.len() >= 2 {
                items.push(Item3 { of: Some(e), in_plane: None, what: Part::Drawn, pts });
                continue;
            }
        }
        for poly in drawable(sk, e, unit) {
            if poly.len() < 2 {
                continue;
            }
            items.push(Item3 {
                of: Some(e),
                in_plane,
                what: Part::Drawn,
                pts: poly.into_iter().map(|p| sk.world_in(plane, p)).collect(),
            });
        }
    }
    // a curve traced in space — a spline over points in space among them (#363) — stands in
    // space and in no view, as an axis does: its polyline refined to the scene's flatness
    if datums {
        let stations = sk.stations();
        for i in 0..sk.traced.len() {
            let e = EntRef::new(EntKind::Traced, i);
            if stations.contains(&e) || !sk.shown(e) {
                continue;
            }
            let pts = traced_polyline(sk, i, unit);
            if pts.len() >= 2 {
                items.push(Item3 { of: Some(e), in_plane: None, what: Part::Drawn, pts });
            }
        }
    }
    // **the object's creases, and none of its silhouettes.**  A silhouette is a fact about a
    // viewpoint, and this scene has none: it is handed to a renderer that owns the camera and
    // turns it as the pointer moves.  What draws the round of a cylinder there is the shading,
    // which the flat path could not have and had to say in lines; what is left for a line to
    // say is where the surface actually breaks.  So a `smooth` seam is dropped outright rather
    // than tested against an eye direction, and no edge is hidden-line removed either — a depth
    // buffer settles that per pixel, which is the whole of why this path exists.
    //
    for i in shown_objects(sk).into_iter().filter(|_| objects_too) {
        let Ok(solid) = sk.evaluated_solid(i, crate::solid::ApproximationPolicy::Mesh) else { continue };
        for e in solid.world_edges() {
            if e.smooth {
                continue;
            }
            items.push(Item3 { of: None, in_plane: None, what: Part::Solid, pts: vec![e.a, e.b] });
        }
    }
    items
}

/// The scene, `shaded` asking for the solid's **surfaces** as well as its edges (`Part::Shell`).
///
/// It is a *view* option and not document state — `underlay`'s rule, like the orbit itself: a
/// drawing is the same drawing whether or not you are looking at it filled in.
pub fn scene_with(sk: &Sketch, unit: f64, az: f64, el: f64, shaded: bool) -> Scene {
    let (right, up) = eye(az, el);
    let flat = |p: [f64; 3]| (dot(p, right), dot(p, up));
    let mut items: Vec<Item> = Vec::new();
    let views = views(sk);
    let rule = panes(sk);

    // the panes of the box, under everything — **one per place, drawn in or not** — each with
    // its own axes at its origin
    let places = places(sk);
    for i in (0..sk.frames.len()).filter(|&i| places[i] == Some(i)) {
        let of = Some(EntRef::frame(i));
        let basis = sk.basis(i);
        let rect = pane(sk, i, &views, &places, &rule);
        items.push(Item {
            of,
            in_plane: of,
            what: Part::Face,
            pts: face(&basis, rect).into_iter().map(&flat).collect(),
            shade: None,
        });
        for arm in axes(&basis, rect) {
            items.push(Item {
                of,
                in_plane: of,
                what: Part::Axis,
                pts: arm.into_iter().map(&flat).collect(),
                shade: None,
            });
        }
    }
    // every view's own geometry, standing on its plane
    for e in sk.drawn() {
        // a point is a place, and a plane's glyph is the sheet's way of showing a datum — in
        // the box the pane says it
        if matches!(e.kind, EntKind::Point | EntKind::Frame) {
            continue;
        }
        let plane = entity_view(sk, e, &views);
        // a line stands with each end where that end is: a projector between two views is
        // neither a stray stroke on the page nor anyone's — it belongs to a view only when both
        // its ends do
        if e.kind == EntKind::Line {
            let l = &sk.lines[e.i()];
            let end = |q: u32| flat(sk.world_point(q as usize));
            items.push(Item {
                of: Some(e),
                in_plane: plane.map(EntRef::frame),
                what: Part::Drawn,
                pts: vec![end(l.p1), end(l.p2)],
                shade: None,
            });
            continue;
        }
        for poly in drawable(sk, e, unit) {
            if poly.len() < 2 {
                continue;
            }
            items.push(Item {
                of: Some(e),
                in_plane: plane.map(EntRef::frame),
                what: Part::Drawn,
                pts: poly.into_iter().map(|p| flat(sk.world_in(plane, p))).collect(),
                shade: None,
            });
        }
    }
    // **and the object itself.**  Where the document *has* a solid, that is the object and there
    // is nothing to reconstruct: its edges are the term's own, classified against the orbit's eye
    // rather than against a view's normal, so a box of a part shows what a part is and not what
    // two pictures of it happen to agree about.  The wireframe below is what a drawing with no
    // solid in it can still be shown as — several views of an object nothing in the document
    // names — and it is skipped outright when a solid names one.
    if !sk.solids.is_empty() {
        // **the objects, and not the features they are made of.**  A document names `block`,
        // `bore`, `passage` and `body`, and only the last of those is a thing: the rest are
        // operands of it — a bore is a hole in a part and not a part beside it.  Shown whole
        // they were each drawn as an object in its own right, hidden-line tested against
        // themselves, which is why a bore appeared to float in front of the face it is drilled
        // through.  A solid is the object exactly when nothing else is made of it.
        let shown = objects(sk);
        let eye = eye(az, el);
        let dir = [
            eye.0[1] * eye.1[2] - eye.0[2] * eye.1[1],
            eye.0[2] * eye.1[0] - eye.0[0] * eye.1[2],
            eye.0[0] * eye.1[1] - eye.0[1] * eye.1[0],
        ];
        // **the surfaces first, so the edges land on top of them.**  Back-facing pieces are
        // dropped — a closed solid never shows its inside — and the rest are handed over *far
        // first*, so a front end that fills them in the order it is given gets the near ones
        // over the far ones without a depth buffer.  A painter's ordering is exact for a convex
        // part and very nearly right for anything a drawing describes; what makes it read
        // correctly regardless is that the *edges* over it are hidden-line removed properly.
        if shaded {
            let mut all: Vec<crate::csg::Piece> = Vec::new();
            for &i in &shown {
                if let Ok(solid) = sk.evaluated_solid(i, crate::solid::ApproximationPolicy::Mesh) {
                    all.extend(solid.world_boundary());
                }
            }
            for it in shell(&all, dir, &eye, flat) {
                items.push(it);
            }
        }
        for &i in &shown {
            let Ok(solid) = sk.evaluated_solid(i, crate::solid::ApproximationPolicy::Mesh) else { continue };
            let renderer = crate::renderer::Renderer::prepare(&solid);
            // the eye in the solid's own coordinates, where its edges and normals are
            let along = solid.vector_to_local(dir);
            for e in solid.edges() {
                if e.smooth {
                    let (a, b) = (crate::plane::dot(e.na, along), crate::plane::dot(e.nb, along));
                    if a * b > 0.0 || (a.abs() < 1e-12 && b.abs() < 1e-12) {
                        continue;
                    }
                }
                let m = [(e.a[0] + e.b[0]) / 2.0, (e.a[1] + e.b[1]) / 2.0, (e.a[2] + e.b[2]) / 2.0];
                if renderer.occludes(crate::solid::LocalPoint(m), along, None) {
                    continue;
                }
                items.push(Item {
                    of: None,
                    in_plane: None,
                    what: Part::Solid,
                    pts: vec![flat(solid.to_world(crate::solid::LocalPoint(e.a)).0), flat(solid.to_world(crate::solid::LocalPoint(e.b)).0)],
                    shade: None,
                });
            }
        }
        return finish(items);
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
        (p[0] - q[0]).dpowi(2) + (p[1] - q[1]).dpowi(2) + (p[2] - q[2]).dpowi(2) <= tol * tol
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
                shade: None,
            });
        }
    }
    finish(items)
}

fn finish(items: Vec<Item>) -> Scene {
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

/// **Each plane's place**: the first plane standing on the same plane in space
/// (`Basis::coplanar`, `workspace::Views`' rule and tolerance, the page left out — it has no
/// pane).  `std.up` is `std.front` turned a quarter, so its place is the front's.  A face
/// family's station plane has none (#243 L1): no pane, and none taken from a plane it lies on.
fn places(sk: &Sketch) -> Vec<Option<usize>> {
    let bases: Vec<_> = (0..sk.frames.len()).map(|i| sk.basis(i)).collect();
    let stations = sk.stations();
    // a spatial frame draws no pane: its points stand in space (#383)
    let placed = |k: usize| !stations.contains(&EntRef::frame(k)) && sk.frames[k].planar();
    let tol = 1e-9 * sk.extent();
    (0..bases.len())
        .map(|k| placed(k).then(|| {
            (0..k).find(|&j| placed(j) && bases[j].coplanar(&bases[k], tol)).unwrap_or(k)
        }))
        .collect()
}

/// The rectangle a view is drawn on, in its own view coordinates: the box of everything standing
/// in its **place** — on it, or on any plane in the same place in space — *and the origins
/// there* — where axes cross, so a pane never cuts them off — grown a little, and never thinner
/// than `least` (`LEAST_SIDE` of the sketch's extent) either way, which is what a view with
/// nothing in it, or only its origin, or only points along one line, gets — nor, margin and all,
/// than the workspace (`WORKSPACE_MM`).
///
/// One rule for the face and its axes both, so the two cannot come to disagree about how far a
/// view reaches; and one pane for a place, so drawing on `std.front` does not leave `std.up`'s
/// behind as a second frame round the same plane (the box draws the place's first plane only).
fn pane(sk: &Sketch, plane: usize, views: &[Option<usize>], places: &[Option<usize>],
        rule: &Panes) -> Box2 {
    let Panes { least, span, shown_only, .. } = *rule;
    let counted = |e: EntRef| !shown_only || sk.shown(e);
    let basis = sk.basis(plane);
    let mut b = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for j in (0..places.len())
        .filter(|&j| j != plane && places[j] == places[plane] && counted(EntRef::frame(j))) {
        grow(&mut b, basis.view_coords(sk.basis(j).o));
    }
    for (i, v) in views.iter().enumerate().filter(|&(i, _)| counted(EntRef::point(i))) {
        match *v {
            Some(j) if j == plane => grow(&mut b, sk.point_xy(i)),
            Some(j) if places[j] == places[plane] => {
                grow(&mut b, basis.view_coords(sk.world_point(i)))
            }
            _ => {}
        }
    }
    let widen = |lo: f64, hi: f64, least: f64| {
        let short = (least - (hi - lo)).max(0.0) / 2.0;
        (lo - short, hi + short)
    };
    let ((x0, x1), (y0, y1)) = (widen(b.0, b.2, least), widen(b.1, b.3, least));
    let m = (x1 - x0).max(y1 - y0) * FACE_MARGIN;
    // and never narrower than the workspace, or a part's datum, either way
    let ((x0, x1), (y0, y1)) = (widen(x0 - m, x1 + m, span), widen(y0 - m, y1 + m, span));
    (x0, y0, x1, y1)
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

/// The objects the outline has not hidden (#435), while objects are shown (#678): what the
/// workspace draws, lights, picks and frames.  `objects` stays the document's — what is exported,
/// reported and built.
pub fn shown_objects(sk: &Sketch) -> Vec<usize> {
    let mut v = objects(sk);
    if sk.hides_any() {
        v.retain(|&i| sk.shown(EntRef::solid(i)));
    }
    v
}

/// **The objects that are not drawn, and why**: each object (`objects`) that does not evaluate
/// at the box's approximation (`ApproximationPolicy::Mesh`, the one it is meshed at), with the
/// reason the evaluation gives — `` `b1`: the revolution axis crosses the profile ``.  A swept
/// object whose surface a host is still meshing (`FieldMeshing::Deferred`, none supplied yet) is
/// not one: it is on its way, and the host says so.  Evaluations are the sketch's cached ones,
/// so asking after the box has drawn costs nothing more.
pub fn unbuilt(sk: &Sketch) -> Vec<(usize, String)> {
    let meshing = |i: usize| sk.is_swept(i) && sk.supplied_field(i).is_none()
        && sk.field_meshing.get() == crate::solid::FieldMeshing::Deferred;
    objects(sk).into_iter().filter(|&i| !meshing(i))
        .filter_map(|i| sk.evaluated_solid(i, crate::solid::ApproximationPolicy::Mesh).err()
            .map(|why| (i, why)))
        .collect()
}

/// **The surfaces of a solid, as this eye sees them.**
///
/// Two culls and then an ordering, and the first cull is the one that matters.  A closed solid
/// never shows its inside, so a back-facing piece goes.  What is left still includes every
/// *void's* wall — a bore is a hole, and the wall of a hole faces the eye from inside the
/// material — and those must go too, or a part is drawn with its bores floating in front of the
/// face they are drilled through.
///
/// Sorting by depth does not fix it, and it is worth saying why: a painter's order compares
/// **centroids**, and a big front face has its centroid in the middle while a small piece behind
/// it may sit nearer than that middle.  Ordering can only be right between polygons that do not
/// overlap in the picture, which is exactly the case this is not.
///
/// So a piece is kept when **nothing stands between it and the eye** — an axis from its centroid,
/// against every other piece of the boundary.  That is exact where a whole face is hidden, which
/// is every void; a face *partly* covered is all-or-nothing, which the depth sort below then
/// mostly settles, and which is the honest limit of a schematic without a depth buffer.
fn shell(
    all: &[crate::csg::Piece],
    dir: [f64; 3],
    eye: &([f64; 3], [f64; 3]),
    flat: impl Fn([f64; 3]) -> (f64, f64),
) -> Vec<Item> {
    // depth along the eye, so a piece is only ever tested against what is nearer than it
    let mut order: Vec<(f64, usize)> = all
        .iter()
        .enumerate()
        .map(|(i, p)| (crate::plane::dot(p.centroid(), dir), i))
        .collect();
    order.sort_by(|a, b| a.0.partial_cmp(&b.0).expect("a finite drawing"));
    let planes: Vec<([f64; 3], f64)> =
        all.iter().map(|p| (p.n, crate::plane::dot(p.n, p.pts[0]))).collect();
    let mut out = Vec::new();
    for (k, &(_, i)) in order.iter().enumerate() {
        let p = &all[i];
        if crate::plane::dot(p.n, dir) <= 0.0 {
            continue;
        }
        let c = p.centroid();
        // only the pieces in front of it can hide it, which the sort has already named
        let hidden = order[k + 1..].iter().any(|&(_, j)| {
            let (n, w) = planes[j];
            let denom = crate::plane::dot(n, dir);
            if denom.abs() < 1e-12 {
                return false;
            }
            // in front of it, along the eye — the sort has already said `j` is nearer, so a
            // hit behind the centroid is the far side of a piece and hides nothing
            let t = (w - crate::plane::dot(n, c)) / denom;
            if t <= 1e-9 {
                return false;
            }
            let x = [c[0] + t * dir[0], c[1] + t * dir[1], c[2] + t * dir[2]];
            in_outline(&all[j].pts, n, x)
        });
        if hidden {
            continue;
        }
        out.push(Item {
            of: None,
            in_plane: None,
            what: Part::Shell,
            pts: p.pts.iter().map(|&q| flat(q)).collect(),
            shade: Some(lambert(p.n, eye)),
        });
    }
    out
}

/// Is `x`, already on the piece's plane, inside its outline?
fn in_outline(pts: &[[f64; 3]], n: [f64; 3], x: [f64; 3]) -> bool {
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        let e = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let w = [x[0] - a[0], x[1] - a[1], x[2] - a[2]];
        if crate::plane::dot(n, crate::plane::cross(e, w)) < 0.0 {
            return false;
        }
    }
    true
}

/// How squarely a surface faces the light, 0 to 1.
///
/// The light is a **headlight, tilted**: over the viewer's shoulder and a little up and to the
/// left, which is where a draughtsman's light has come from since long before there were
/// screens.  Straight down the eye would flatten every face that faces you into one tone and
/// lose the corner between them, which is the whole thing a shaded picture is for.
fn lambert(n: [f64; 3], eye: &([f64; 3], [f64; 3])) -> f64 {
    let (right, up) = *eye;
    let dir = crate::plane::cross(right, up);
    let l = [
        dir[0] - 0.45 * right[0] + 0.35 * up[0],
        dir[1] - 0.45 * right[1] + 0.35 * up[1],
        dir[2] - 0.45 * right[2] + 0.35 * up[2],
    ];
    let l = crate::plane::unit(l).unwrap_or(dir);
    crate::plane::dot(n, l).clamp(0.0, 1.0)
}

/// How far either side of its foot an axis is drawn: the drawing's reach, and no less than half
/// the workspace (`WORKSPACE_MM`), so a blank sheet's axes run to its panes' edges — or, an
/// assembly shown on its parts, half its datums' span (`panes`, #679).
pub(crate) fn axis_reach(sk: &Sketch) -> f64 {
    panes(sk).reach
}

/// Where axis `i` is drawn: `reach` (`axis_reach`) either side of its foot, tail then tip, and its
/// unit direction — the one segment the box draws and a click picks by.  `None` for no direction.
pub(crate) fn axis_segment(sk: &Sketch, i: usize, reach: f64) -> Option<([f64; 3], [f64; 3], [f64; 3])> {
    use crate::space::{add, normalised, scale};
    let r = &sk.axes[i];
    let p = r.a.map(|q| sk.params[q as usize].value);
    let d = normalised(r.d.map(|q| sk.params[q as usize].value))?;
    Some((add(p, scale(d, -reach)), add(p, scale(d, reach)), d))
}

