//! **What a solid is, and the one question everything asks of it** (Solvent §6.9).
//!
//! A feature tree is imperative because it is *stateful*: step *n* acts on the anonymous "body
//! as of step *n − 1*" and names faces by the order they were made in.  Solvent names
//! everything, so a solid here is a **term** — its stock, plus everything `on` it, minus
//! everything `through` it — over primitives that are faces swept.  The order lives inside the
//! term, over names, exactly as it lives inside `h = w / 2`; between statements there is no
//! order at all, which is P2 and is what a feature tree cannot have.
//!
//! **Nothing three-dimensional is ever solved for.**  A solid owns no parameter: every extent is
//! an expression the flattener settled, and the geometry it is swept from is the drawing, solved
//! in 2D as it always was.  The strata run one way — the sketch solves, the depths are worked
//! out, the terms are ordered, and the outputs are read — with no edge back.
//!
//! **The kernel is the term, and every output is a question asked of it.**  Nothing is built and
//! nothing is stored: a view, a section, a mesh, a volume and a clearance are all
//! *classification* against `Csg` (Requicha & Voelcker's boundary evaluation).  There is no
//! B-rep, which is the reason the crate still has no dependency.
//!
//! **One rule holds the whole thing together: the classifier reads the facets the candidates are
//! cut from.**  A primitive is reduced to a closed polyhedron of planar convex facets — arcs and
//! circles tessellated by the sagitta rule the drawing itself is drawn by, a revolution faceted
//! about its axis the same way — and `classify` casts a ray against *those* facets and no ideal
//! surface.  Classify exactly against the true circle instead, and every facet centroid of a
//! bore's wall would lie inside the true bore by the sagitta, both its samples would read
//! *outside*, and the wall would silently vanish.

use crate::model::{EntKind, EntRef, Sense, Sketch, SolidDef};
use crate::plane::{self, Basis};
use std::collections::BTreeMap;

/// How far off a face a sample is pushed before it is classified, as a fraction of the drawing's
/// extent.  Comfortably above `SNAP` so no sliver survives between the two, and comfortably
/// below any feature a person draws.
pub const EPS: f64 = 1e-5;

/// How near two planes must be, relatively, to be *one* plane — and how short a side must be to
/// be no side at all.
///
/// The 2D solve agrees only to its own tolerance, so a bore's cap solved at `ct + 3e-11` and the
/// face it is flush with at `ct` are one plane to a draughtsman and two to arithmetic.  The
/// answer is to compare at this tolerance, **not** to round the coordinates to a grid: a grid
/// fine enough to leave the drawing's own numbers alone is far finer than the noise it was meant
/// to collapse, and one coarse enough to collapse the noise moves every vertex — a block sixty
/// across came out `72000.0036` instead of `72000`, which is a wrong answer bought to fix a
/// problem that was already handled.  What actually keeps the classifier out of the gap is
/// `EPS`, four orders coarser than the noise, and `same_plane`, which reads the two as one and
/// never splits a facet by its own plane.
pub const SNAP: f64 = 1e-9;

/// The faceting a *report* is computed at.
///
/// A volume is a property of the document and not of the zoom, so it may not be asked at the
/// screen's `unit` (§16.3) — and it need not be asked at the finest faceting either.  A round
/// surface cut into `n` chords is under the true one by about `6.6/n²` of its volume, so this
/// buys a report good to one part in ten thousand, which is four digits more than a drawing
/// states.  Ten times finer costs eight times the facets and every boolean over them: the
/// O-ring groove test ran ten seconds at `2e-4` and under two here, for a number that agreed to
/// the digit either way.
pub const REPORT_UNIT: f64 = 2e-3;

/// How fine a *mesh* of a solid is cut, as a fraction of the object's own diagonal.
///
/// **A volume and a mesh want different faceting, and giving them one number is a mistake that
/// costs an order of magnitude.**  A volume is a number quoted to four digits, so `REPORT_UNIT`
/// is chosen to be good to one part in ten thousand — and a mesh inheriting it cut the V-twin
/// cylinder's 16 mm bore into 257 flats, a six ten-thousandths of a millimetre sagitta, for a
/// part whose printer resolves a tenth of a millimetre and whose viewer resolves a pixel.  That
/// is 98,000 triangles where 8,000 are indistinguishable.
///
/// So a mesh is cut to the **object** and not to the report: a sagitta this fraction of the
/// solid's own diagonal, which is scale-free and therefore says the same thing whether a
/// document is written in millimetres or in inches.
pub const MESH_SAGITTA: f64 = 1e-4;

/// The `unit` a mesh of this solid should be cut at — `MESH_SAGITTA` of its own diagonal, put
/// back through `curve::flatness`, which is the one conversion between a tolerance and a `unit`.
///
/// Asked of the *solid's* bounds and not the sketch's: a part sheet's extent is the whole sheet,
/// three views wide, and a part is not.
pub fn mesh_unit(sk: &Sketch, i: usize) -> f64 {
    // from the primitives' own boxes and not from an evaluated boundary: the answer only has to
    // pick a faceting, and paying for a fine boundary to decide how fine a boundary to build
    // would be the tail wagging the dog.  A coarse tessellation gives a box right to its own
    // sagitta, which is far below anything this then rounds to.
    let b = resolve(sk, i, REPORT_UNIT * 20.0).bbox();
    if b.is_empty() {
        return REPORT_UNIT;
    }
    let diag = (0..3).map(|k| (b.hi[k] - b.lo[k]).powi(2)).sum::<f64>().sqrt();
    (diag * MESH_SAGITTA / crate::curve::FLATNESS_PX).max(REPORT_UNIT)
}

/// A planar convex facet of a primitive's boundary, with its outward normal and the name the
/// document reaches it by.
#[derive(Clone, Debug)]
pub struct Facet {
    pub pts: Vec<[f64; 3]>,
    pub n: [f64; 3],
    /// Which of the primitive's faces this is a piece of — `near`, `far`, or the edge it was
    /// swept from.  A path, never an index: a boolean never renames, so this is what
    /// `body.bore.wall` resolves through.
    pub face: usize,
    /// True when this facet's seams with its neighbours around the sweep are a *tessellation*
    /// joint rather than a corner of the design — the flats of a bore's wall.  A draughtsman
    /// draws a cylinder's silhouette and not its facets, and this is what lets `hidden` tell
    /// them apart.
    pub smooth: bool,
}

impl Facet {
    pub fn centroid(&self) -> [f64; 3] {
        let k = 1.0 / self.pts.len() as f64;
        let mut c = [0.0; 3];
        for p in &self.pts {
            for i in 0..3 {
                c[i] += p[i] * k;
            }
        }
        c
    }

    pub fn bbox(&self) -> Box3 {
        let mut b = Box3::empty();
        for p in &self.pts {
            b.add(*p);
        }
        b
    }

    /// `n·x = d`, the facet's own plane.
    pub fn offset(&self) -> f64 {
        plane::dot(self.n, self.pts[0])
    }
}

/// An axis-aligned box in space — every pairwise walk in the kernel is culled by one.
#[derive(Clone, Copy, Debug)]
pub struct Box3 {
    pub lo: [f64; 3],
    pub hi: [f64; 3],
}

impl Box3 {
    pub fn empty() -> Box3 {
        Box3 { lo: [f64::INFINITY; 3], hi: [f64::NEG_INFINITY; 3] }
    }
    pub fn add(&mut self, p: [f64; 3]) {
        for i in 0..3 {
            self.lo[i] = self.lo[i].min(p[i]);
            self.hi[i] = self.hi[i].max(p[i]);
        }
    }
    pub fn grown(&self, k: f64) -> Box3 {
        Box3 {
            lo: [self.lo[0] - k, self.lo[1] - k, self.lo[2] - k],
            hi: [self.hi[0] + k, self.hi[1] + k, self.hi[2] + k],
        }
    }
    pub fn overlaps(&self, o: &Box3) -> bool {
        (0..3).all(|i| self.lo[i] <= o.hi[i] && o.lo[i] <= self.hi[i])
    }
    pub fn holds(&self, p: [f64; 3]) -> bool {
        (0..3).all(|i| p[i] >= self.lo[i] && p[i] <= self.hi[i])
    }
    pub fn is_empty(&self) -> bool {
        self.lo[0] > self.hi[0]
    }
}

/// One swept face, as the closed polyhedron the classifier and the boundary walk both read.
#[derive(Clone, Debug)]
pub struct Prim {
    pub facets: Vec<Facet>,
    pub bbox: Box3,
    /// The names of this primitive's faces, by the index a `Facet` carries: `near`, `far`, the
    /// drawn edge each side was swept from, `start`/`end` for a partial revolution.
    pub faces: Vec<String>,
    /// The solid statement this primitive came from — what a face path is prefixed by.
    pub of: String,
}

/// The term a solid *is*.  Order lives here, over names, and nowhere else.
#[derive(Clone, Debug)]
pub enum Term {
    Prim(usize),
    Union(Box<Term>, Box<Term>),
    Diff(Box<Term>, Box<Term>),
    /// A term nothing could be built for — a face that would not close, a degenerate sweep.
    /// Classifies as empty, so an output is missing rather than wrong.
    Empty,
}

/// A solid, resolved: the primitives it is made of and the term over them.
#[derive(Clone, Debug)]
pub struct Csg {
    pub prims: Vec<Prim>,
    pub term: Term,
}

impl Csg {
    pub fn bbox(&self) -> Box3 {
        let mut b = Box3::empty();
        for p in &self.prims {
            if !p.bbox.is_empty() {
                b.add(p.bbox.lo);
                b.add(p.bbox.hi);
            }
        }
        b
    }

    /// **Is `p` inside this solid?**  The one question, and the only thing any output asks.
    ///
    /// A primitive answers by casting a ray against its own facets and counting crossings, so
    /// what is classified is exactly what is drawn and meshed.  A ray that grazes an edge or a
    /// vertex is not perturbed away — the *direction* is, from a fixed table, so the answer
    /// stays deterministic (a jittered point would move the boundary instead of the question).
    pub fn inside(&self, p: [f64; 3]) -> bool {
        self.eval(&self.term, p)
    }

    fn eval(&self, t: &Term, p: [f64; 3]) -> bool {
        match t {
            Term::Empty => false,
            Term::Prim(i) => in_prim(&self.prims[*i], p),
            Term::Union(a, b) => self.eval(a, p) || self.eval(b, p),
            Term::Diff(a, b) => self.eval(a, p) && !self.eval(b, p),
        }
    }
}

/// The directions a ray is cast along, in order.  Irrational ratios, so a facet of a drawing
/// written in round numbers is never parallel to one; tried in turn when a cast is degenerate.
const RAYS: [[f64; 3]; 4] = [
    [0.4472135954999579, 0.6155870112510924, 0.6494442148951877],
    [-0.7071067811865476, 0.5000000000000000, 0.5000000000000000],
    [0.3015113445777636, -0.9045340337332909, 0.3015113445777636],
    [0.5773502691896258, 0.5773502691896258, -0.5773502691896258],
];

fn in_prim(prim: &Prim, p: [f64; 3]) -> bool {
    if !prim.bbox.grown(1e-12).holds(p) {
        return false;
    }
    for d in RAYS {
        if let Some(hits) = cast(prim, p, d) {
            return hits % 2 == 1;
        }
    }
    false
}

/// Crossings of the ray `p + t·d`, `t > 0`, with the primitive's facets.  `None` when the ray
/// passes too near an edge for the count to be trusted — the caller tries another direction.
fn cast(prim: &Prim, p: [f64; 3], d: [f64; 3]) -> Option<usize> {
    let mut hits = 0usize;
    for f in &prim.facets {
        let denom = plane::dot(f.n, d);
        if denom.abs() < 1e-12 {
            continue;
        }
        let t = (f.offset() - plane::dot(f.n, p)) / denom;
        if t <= 0.0 {
            continue;
        }
        let x = [p[0] + t * d[0], p[1] + t * d[1], p[2] + t * d[2]];
        match in_facet(f, x) {
            Hit::In => hits += 1,
            Hit::Out => {}
            Hit::Edge => return None,
        }
    }
    Some(hits)
}

enum Hit {
    In,
    Out,
    Edge,
}

/// Is `x` — already on the facet's plane — inside its convex outline?  `Edge` when it is within
/// a hair of the outline, where a crossing count would be a coin toss.
fn in_facet(f: &Facet, x: [f64; 3]) -> Hit {
    let mut scale = 0.0f64;
    for i in 0..f.pts.len() {
        let a = f.pts[i];
        let b = f.pts[(i + 1) % f.pts.len()];
        scale = scale.max(plane::norm([b[0] - a[0], b[1] - a[1], b[2] - a[2]]));
    }
    let tol = scale.max(1e-12) * 1e-9;
    let mut sign = 0i32;
    for i in 0..f.pts.len() {
        let a = f.pts[i];
        let b = f.pts[(i + 1) % f.pts.len()];
        let e = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let w = [x[0] - a[0], x[1] - a[1], x[2] - a[2]];
        let c = plane::dot(f.n, plane::cross(e, w));
        let len = plane::norm(e).max(1e-300);
        if (c / len).abs() < tol {
            return Hit::Edge;
        }
        let s = if c > 0.0 { 1 } else { -1 };
        if sign == 0 {
            sign = s;
        } else if sign != s {
            return Hit::Out;
        }
    }
    Hit::In
}

// -- building a primitive out of a face --------------------------------------------------------

/// A face as the classifier wants it: its loop in the plane's own view coordinates, the plane it
/// is on, and where each vertex came from — so a swept side can be named by the edge it was
/// swept from and a tessellated arc's facets can be told from a corner.
#[derive(Clone, Debug)]
pub struct FacePoly {
    /// The loop, closed implicitly, in the plane's 2D view coordinates.
    pub pts: Vec<(f64, f64)>,
    /// Per vertex, the index into `names` of the edge that *leaves* it, and whether the step to
    /// the next vertex is a tessellation chord rather than a drawn straight edge.
    pub of: Vec<(usize, bool)>,
    /// The drawn edges, in traversal order, by the name the document calls them.
    pub names: Vec<String>,
    pub basis: Basis,
    /// The plane's page pose: rotor and origin, for `in_view`/`on_page`.
    pub pose: (f64, f64, (f64, f64)),
}

impl FacePoly {
    pub fn area(&self) -> f64 {
        let n = self.pts.len();
        let mut a = 0.0;
        for i in 0..n {
            let (x0, y0) = self.pts[i];
            let (x1, y1) = self.pts[(i + 1) % n];
            a += x0 * y1 - x1 * y0;
        }
        a / 2.0
    }

    /// The loop, turned counter-clockwise in view coordinates — the winding every sweep below
    /// assumes, so that a prism's `near` cap faces the viewer.
    fn ccw(&self) -> FacePoly {
        if self.area() >= 0.0 {
            return self.clone();
        }
        let n = self.pts.len();
        let mut pts = self.pts.clone();
        pts.reverse();
        // the edge that *leaves* vertex i, reversed, is the one that used to arrive at it
        let mut of = Vec::with_capacity(n);
        for i in 0..n {
            of.push(self.of[(n - 1 - i + n - 1) % n]);
        }
        FacePoly { pts, of, names: self.names.clone(), basis: self.basis, pose: self.pose }
    }

    pub fn lift(&self, i: usize) -> [f64; 3] {
        let (a, b) = self.pts[i];
        self.basis.lift(a, b)
    }
}

/// Read a face off the solved drawing: its loop, walked edge by edge, arcs and circles
/// tessellated by the sagitta rule the sheet itself is drawn by.
///
/// `None` when the loop does not close or an edge is degenerate — the elaborator has already
/// refused those (E080), so this is the runtime's own guard rather than a diagnosis.
pub fn face_poly(sk: &Sketch, fi: usize, unit: f64) -> Option<FacePoly> {
    let f = sk.faces.get(fi)?;
    if f.edges.is_empty() {
        return None;
    }
    let basis = match f.plane {
        Some(p) => sk.planes.get(p as usize)?.basis,
        None => Basis::page(),
    };
    let pose = match f.plane {
        Some(p) => {
            let fr = &sk.planes.get(p as usize)?.frame;
            (
                sk.params[fr.c as usize].value,
                sk.params[fr.s as usize].value,
                sk.point_xy(fr.origin as usize),
            )
        }
        None => (1.0, 0.0, (0.0, 0.0)),
    };
    // Every edge is walked in *page* coordinates and the whole loop is turned into the plane's
    // own view coordinates at the end.  `in_view` is a rigid motion, so tessellating before it
    // and after it are the same chords; doing it once here is what keeps a face on a tilted
    // plane from being read as if it were drawn on the page.
    let view = |p: (f64, f64)| plane::in_view(pose.0, pose.1, pose.2, p);

    let names: Vec<String> = f.edge_names.clone();
    let mut pts: Vec<(f64, f64)> = Vec::new();
    let mut of: Vec<(usize, bool)> = Vec::new();

    // a circle standing alone is the whole loop
    if f.edges.len() == 1 && f.edges[0].kind == EntKind::Circle {
        let c = &sk.circles[f.edges[0].i()];
        let ctr = sk.point_xy(c.center as usize);
        let r = sk.params[c.radius as usize].value;
        if r.abs() <= 0.0 {
            return None;
        }
        let ring = tessellate_arc(ctr, r.abs(), 0.0, std::f64::consts::TAU, unit);
        for p in ring.iter().take(ring.len() - 1) {
            pts.push(*p);
            of.push((0, true));
        }
        let pts = pts.into_iter().map(view).collect();
        return Some(tidy_poly(FacePoly { pts, of, names, basis, pose }).ccw());
    }

    // otherwise: every edge in traversal order, each starting where the last one ended
    let mut at: Option<u32> = None;
    for (i, e) in f.edges.iter().enumerate() {
        let (a, b) = crate::model::edge_ends(sk, *e)?;
        // which end this edge is entered by: the one the walk is standing on
        let (from, to) = match at {
            None => {
                // the first edge is entered by whichever end the *last* edge shares
                let (la, lb) = crate::model::edge_ends(sk, *f.edges.last()?)?;
                if a == la || a == lb {
                    (a, b)
                } else {
                    (b, a)
                }
            }
            Some(p) if p == a => (a, b),
            Some(p) if p == b => (b, a),
            Some(_) => return None,
        };
        walk_edge(sk, *e, from, to, i, unit, &mut pts, &mut of)?;
        at = Some(to);
    }
    if pts.len() < 3 {
        return None;
    }
    let pts = pts.into_iter().map(view).collect();
    Some(tidy_poly(FacePoly { pts, of, names, basis, pose }).ccw())
}

/// A zero-length side is no side: two coincident vertices would give a facet with no normal and
/// a seam with no direction.  Coordinates are left exactly as the solve found them.
fn tidy_poly(mut f: FacePoly) -> FacePoly {
    let scale = f.pts.iter().fold(1.0f64, |m, p| m.max(p.0.abs()).max(p.1.abs()));
    let g = scale * SNAP;
    let mut pts = Vec::with_capacity(f.pts.len());
    let mut of = Vec::with_capacity(f.of.len());
    for i in 0..f.pts.len() {
        let j = (i + 1) % f.pts.len();
        let d = (f.pts[j].0 - f.pts[i].0).hypot(f.pts[j].1 - f.pts[i].1);
        if d > g {
            pts.push(f.pts[i]);
            of.push(f.of[i]);
        }
    }
    if pts.len() >= 3 {
        f.pts = pts;
        f.of = of;
    }
    f
}

fn walk_edge(
    sk: &Sketch,
    e: EntRef,
    from: u32,
    _to: u32,
    idx: usize,
    unit: f64,
    pts: &mut Vec<(f64, f64)>,
    of: &mut Vec<(usize, bool)>,
) -> Option<()> {
    match e.kind {
        EntKind::Line => {
            pts.push(sk.point_xy(from as usize));
            of.push((idx, false));
            Some(())
        }
        EntKind::Arc => {
            let a = &sk.arcs[e.i()];
            let c = sk.point_xy(a.center as usize);
            let r = sk.params[a.radius as usize].value.abs();
            let ang = |p: u32| {
                let q = sk.point_xy(p as usize);
                (q.1 - c.1).atan2(q.0 - c.0)
            };
            let (a0, a1) = (ang(a.start), ang(a.end));
            // **How far an arc goes is the arc's own fact; which way is the walk's.**  An arc
            // runs CCW from start to end, and entered at `end` it is walked the other way — the
            // *same* stretch of the circle backwards, never the complement of it.  Normalising
            // `a0 - a1` instead gave `TAU - extent` there, so a face that happened to enter an
            // arc by its end came out as the rest of the circle: the V-twin plate's plenum, a
            // channel between two concentric arcs, closed as a bowtie of twelve times the area
            // and meshed with seventy-six unpaired edges.
            let ccw = from == a.start;
            let mut extent = a1 - a0;
            while extent <= 0.0 {
                extent += std::f64::consts::TAU;
            }
            let start = if ccw { a0 } else { a1 };
            let step = if ccw { extent } else { -extent };
            let ring = tessellate_arc(c, r, start, step, unit);
            for p in ring.iter().take(ring.len() - 1) {
                pts.push(*p);
                of.push((idx, true));
            }
            // the first vertex of an arc is a real corner, not a chord joint
            if let Some(last) = of.len().checked_sub(ring.len() - 1) {
                of[last].1 = ring.len() > 2;
            }
            Some(())
        }
        _ => None,
    }
}

/// A circle or arc as chords no further from it than the sheet's own flatness — `overview::round`'s
/// rule, said here in page coordinates so the solid and the drawing round a corner alike.
fn tessellate_arc(c: (f64, f64), r: f64, from: f64, sweep: f64, unit: f64) -> Vec<(f64, f64)> {
    let tol = crate::curve::flatness(unit);
    let step = if r > tol { 2.0 * (1.0 - tol / r).acos() } else { std::f64::consts::TAU };
    let n = ((sweep.abs() / step).ceil() as usize).clamp(2, 4096);
    (0..=n)
        .map(|k| {
            let a = from + sweep * k as f64 / n as f64;
            (c.0 + r * a.cos(), c.1 + r * a.sin())
        })
        .collect()
}

// -- sweeping ----------------------------------------------------------------------------------

/// Ear clipping: a simple polygon as triangles, by index.  The one triangulation in the kernel,
/// used for a prism's caps and a partial revolution's.
fn ears(pts: &[(f64, f64)]) -> Vec<[usize; 3]> {
    let n = pts.len();
    if n < 3 {
        return Vec::new();
    }
    let area2 = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| {
        (b.0 - a.0) * (c.1 - a.1) - (c.0 - a.0) * (b.1 - a.1)
    };
    let mut idx: Vec<usize> = (0..n).collect();
    let mut out = Vec::with_capacity(n.saturating_sub(2));
    let mut guard = 0;
    while idx.len() > 3 && guard < 4 * n + 16 {
        let m = idx.len();
        let mut cut = false;
        for i in 0..m {
            let (ia, ib, ic) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
            let (a, b, c) = (pts[ia], pts[ib], pts[ic]);
            if area2(a, b, c) <= 0.0 {
                continue;
            }
            let clear = idx.iter().all(|&k| {
                if k == ia || k == ib || k == ic {
                    return true;
                }
                let p = pts[k];
                !(area2(a, b, p) >= 0.0 && area2(b, c, p) >= 0.0 && area2(c, a, p) >= 0.0)
            });
            if clear {
                out.push([ia, ib, ic]);
                idx.remove(i);
                cut = true;
                break;
            }
        }
        if !cut {
            break;
        }
        guard += 1;
    }
    if idx.len() == 3 {
        out.push([idx[0], idx[1], idx[2]]);
    }
    out
}

/// A face swept along its plane's normal, from `lo` to `hi` — a prism (§6.9).
pub fn prism(poly: &FacePoly, lo: f64, hi: f64, of: &str) -> Option<Prim> {
    let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
    if hi - lo <= 0.0 {
        return None;
    }
    let n = poly.pts.len();
    let nrm = poly.basis.normal();
    let at = |i: usize, k: f64| {
        let p = poly.lift(i);
        [p[0] + k * nrm[0], p[1] + k * nrm[1], p[2] + k * nrm[2]]
    };
    // face 0 is `far` (at lo), face 1 is `near` (at hi), then one per drawn edge
    let mut faces = vec!["far".to_string(), "near".to_string()];
    faces.extend(poly.names.iter().cloned());
    let mut facets = Vec::new();
    for t in ears(&poly.pts) {
        // The loop is CCW in view coordinates, so a triangle taken in its own order has the
        // plane's own normal by the right-hand rule.  The cap at `hi` is the far end along +n
        // and keeps that winding; the cap at `lo` faces the other way and is reversed.  The
        // winding and the declared normal must agree or the divergence sum reads the volume of
        // a surface that is inside out — which is exactly what a cap contributing nothing at
        // the origin will hide from every test that does not stand the solid away from it.
        facets.push(Facet {
            pts: vec![at(t[0], hi), at(t[1], hi), at(t[2], hi)],
            n: nrm,
            face: 1,
            smooth: false,
        });
        facets.push(Facet {
            pts: vec![at(t[2], lo), at(t[1], lo), at(t[0], lo)],
            n: plane::scaled(nrm, -1.0),
            face: 0,
            smooth: false,
        });
    }
    for i in 0..n {
        let j = (i + 1) % n;
        let (edge, smooth) = poly.of[i];
        let quad = vec![at(i, lo), at(j, lo), at(j, hi), at(i, hi)];
        let Some(nn) = facet_normal(&quad) else { continue };
        facets.push(Facet { pts: quad, n: nn, face: 2 + edge, smooth });
    }
    Some(finish(facets, faces, of))
}

/// A face swept about a line in its own plane — a revolution (§6.9, §17.1's `ring` about a line).
pub fn revolve(
    poly: &FacePoly,
    axis: ((f64, f64), (f64, f64)),
    sweep: f64,
    sense: Sense,
    unit: f64,
    of: &str,
) -> Option<Prim> {
    let full = sweep >= std::f64::consts::TAU - 1e-9;
    let sweep = if full { std::f64::consts::TAU } else { sweep };
    if sweep <= 0.0 {
        return None;
    }
    // the axis in the plane's own 2D coordinates, and the meridian frame off it
    let (a2, b2) = axis;
    let dir2 = (b2.0 - a2.0, b2.1 - a2.1);
    let len = dir2.0.hypot(dir2.1);
    if len <= 0.0 {
        return None;
    }
    let dir2 = (dir2.0 / len, dir2.1 / len);
    // signed distance across the axis; the face must lie on one side of it
    let across = |p: (f64, f64)| -(p.0 - a2.0) * dir2.1 + (p.1 - a2.1) * dir2.0;
    let along = |p: (f64, f64)| (p.0 - a2.0) * dir2.0 + (p.1 - a2.1) * dir2.1;
    let s: Vec<f64> = poly.pts.iter().map(|p| across(*p)).collect();
    let scale = poly.pts.iter().fold(1.0f64, |m, p| m.max(p.0.abs()).max(p.1.abs()));
    let tol = scale * SNAP;
    if s.iter().any(|v| *v > tol) && s.iter().any(|v| *v < -tol) {
        return None; // the axis crosses the face: a double cover, refused
    }
    let flip = s.iter().any(|v| *v < -tol);
    let sign = if flip { -1.0 } else { 1.0 };
    // the meridian: (r, z) per vertex, r ≥ 0
    let mer: Vec<(f64, f64)> = poly.pts.iter().map(|p| (sign * across(*p), along(*p))).collect();

    // the frame in space: W along the axis, P the in-plane perpendicular the face lies on, and
    // Q = W × P, which is ±n — right-handed about the axis's own p1 → p2
    let o3 = poly.basis.lift(a2.0, a2.1);
    let w = {
        let p1 = poly.basis.lift(b2.0, b2.1);
        plane::unit([p1[0] - o3[0], p1[1] - o3[1], p1[2] - o3[2]])?
    };
    let pdir = plane::scaled(plane::unit(plane::cross(poly.basis.normal(), w))?, sign);
    let qdir = plane::cross(w, pdir);
    let turn = if sense == Sense::Cw { -1.0 } else { 1.0 };
    let at = |r: f64, z: f64, phi: f64| {
        let (sp, cp) = (turn * phi).sin_cos();
        [
            o3[0] + z * w[0] + r * (cp * pdir[0] + sp * qdir[0]),
            o3[1] + z * w[1] + r * (cp * pdir[1] + sp * qdir[1]),
            o3[2] + z * w[2] + r * (cp * pdir[2] + sp * qdir[2]),
        ]
    };
    // faceted about the axis by the same sagitta rule an arc is drawn by, on the widest radius
    let rmax = mer.iter().fold(0.0f64, |m, p| m.max(p.0));
    let tolf = crate::curve::flatness(unit);
    let step =
        if rmax > tolf { 2.0 * (1.0 - tolf / rmax).acos() } else { std::f64::consts::TAU };
    let steps = ((sweep / step).ceil() as usize).clamp(3, 2048);

    let n = mer.len();
    // a revolution's faces: one per drawn edge, then `start` and `end` for a partial turn
    let mut faces: Vec<String> = poly.names.clone();
    let (fi_start, fi_end) = (faces.len(), faces.len() + 1);
    faces.push("start".into());
    faces.push("end".into());

    // the meridian loop must run so that the swept surface faces outward; with r ≥ 0 and the
    // frame above, a CCW loop in (r, z) gives an outward normal under the right-hand rule
    let mer_ccw = {
        let mut a = 0.0;
        for i in 0..n {
            let (x0, y0) = mer[i];
            let (x1, y1) = mer[(i + 1) % n];
            a += x0 * y1 - x1 * y0;
        }
        a * turn >= 0.0
    };

    let mut facets = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        let (edge, smooth_in_plane) = poly.of[i];
        let (r0, z0) = mer[i];
        let (r1, z1) = mer[j];
        for k in 0..steps {
            let p0 = sweep * k as f64 / steps as f64;
            let p1 = sweep * (k + 1) as f64 / steps as f64;
            // A meridian running counter-clockwise in `(r, z)` has the outer wall going up, and
            // the quad that faces *away* from the axis there is the one taken against the sweep.
            // Wound the other way the whole solid is inside out, which the divergence sum reports
            // as a negative volume and nothing else notices.
            let quad = if mer_ccw {
                vec![at(r0, z0, p1), at(r1, z1, p1), at(r1, z1, p0), at(r0, z0, p0)]
            } else {
                vec![at(r0, z0, p0), at(r1, z1, p0), at(r1, z1, p1), at(r0, z0, p1)]
            };
            // a quad on a surface of revolution is not planar: two triangles, always
            for tri in [[0usize, 1, 2], [0, 2, 3]] {
                let pts: Vec<[f64; 3]> = tri.iter().map(|&t| quad[t]).collect();
                let Some(nn) = facet_normal(&pts) else { continue };
                // every seam around the sweep is a tessellation joint; a chord of a drawn arc
                // is one too
                facets.push(Facet { pts, n: nn, face: edge, smooth: true });
            }
            let _ = smooth_in_plane;
        }
    }
    if !full {
        for (phi, fi, rev) in [(0.0, fi_start, true), (sweep, fi_end, false)] {
            for t in ears(&mer) {
                let mut pts: Vec<[f64; 3]> =
                    t.iter().map(|&k| at(mer[k].0, mer[k].1, phi)).collect();
                // the start cap faces back along the sweep and the end cap forward, so exactly
                // one of the two keeps the meridian's own winding
                if rev != mer_ccw {
                    pts.reverse();
                }
                let Some(nn) = facet_normal(&pts) else { continue };
                facets.push(Facet { pts, n: nn, face: fi, smooth: false });
            }
        }
    }
    Some(finish(facets, faces, of))
}

fn facet_normal(pts: &[[f64; 3]]) -> Option<[f64; 3]> {
    let mut acc = [0.0; 3];
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        acc[0] += a[1] * b[2] - a[2] * b[1];
        acc[1] += a[2] * b[0] - a[0] * b[2];
        acc[2] += a[0] * b[1] - a[1] * b[0];
    }
    plane::unit(acc)
}

fn finish(facets: Vec<Facet>, faces: Vec<String>, of: &str) -> Prim {
    let mut bbox = Box3::empty();
    for f in &facets {
        for p in &f.pts {
            bbox.add(*p);
        }
    }
    Prim { facets, bbox, faces, of: of.to_string() }
}

// -- resolving a document's solids into terms ---------------------------------------------------

/// What a caller wants of a solid, and what the cache is keyed by beside it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Want {
    /// The boundary: the mesh, the volume, the bounds.
    Boundary,
    /// The edges a view of it would draw, before visibility.
    Edges,
    /// The welded, face-grouped mesh a printer and a viewer take.
    Mesh,
}

/// What a `Want` came to, remembered against everything it was read from.
#[derive(Clone, Debug)]
pub enum Cached {
    Boundary(Vec<crate::csg::Piece>),
    Edges(Vec<crate::csg::Edge>),
    Mesh(crate::mesh::Mesh),
}

/// **Every number a solid was built from**, in one flat vector — the memo key.
///
/// `curve_polyline`'s bargain: a result is remembered against what it reads rather than
/// invalidated by whoever writes, so a repaint over a drawing that has not changed costs a
/// comparison.  Every scalar of every edge of every face the term reaches, each plane's pose,
/// basis and origin, and every extent.
pub fn reads(sk: &Sketch, si: usize, unit: f64) -> Vec<f64> {
    let mut v = vec![unit];
    let mut seen = std::collections::BTreeSet::new();
    let mut stack = vec![si as u32];
    while let Some(s) = stack.pop() {
        if !seen.insert(s) {
            continue;
        }
        let Some(sol) = sk.solids.get(s as usize) else { continue };
        match &sol.def {
            SolidDef::Prism { face, from, to } => {
                v.push(from.value);
                v.push(to.value);
                face_reads(sk, *face, &mut v);
            }
            SolidDef::Revolve { face, axis, sweep, sense } => {
                v.push(sweep.value);
                v.push(if *sense == Sense::Cw { -1.0 } else { 1.0 });
                face_reads(sk, *face, &mut v);
                if let Some(l) = sk.lines.get(*axis as usize) {
                    for p in [l.p1, l.p2] {
                        let q = sk.point_xy(p as usize);
                        v.push(q.0);
                        v.push(q.1);
                    }
                }
            }
            SolidDef::Body { .. } => {}
        }
        stack.extend(sol.operands());
    }
    v
}

fn face_reads(sk: &Sketch, fi: u32, v: &mut Vec<f64>) {
    let Some(f) = sk.faces.get(fi as usize) else { return };
    for e in &f.edges {
        for p in sk.entity_params(*e) {
            v.push(sk.params[p as usize].value);
        }
    }
    if let Some(p) = f.plane {
        if let Some(pl) = sk.planes.get(p as usize) {
            v.extend(pl.basis.u);
            v.extend(pl.basis.v);
            v.extend(pl.basis.o);
            v.push(sk.params[pl.frame.c as usize].value);
            v.push(sk.params[pl.frame.s as usize].value);
            let o = sk.point_xy(pl.frame.origin as usize);
            v.push(o.0);
            v.push(o.1);
        }
    }
}

/// **Resolve a solid into primitives and a term.**  The term walk: a body is its stock, plus
/// everything `on` it, minus everything `through` it, each operand resolved the same way.
///
/// The document's order is irrelevant and the *statement* order inside a body is too: both
/// groups are sets.  A cycle cannot reach here — the elaborator refuses it (E041) — so a depth
/// guard is all that stands between this and a malformed sketch built by hand.
pub fn resolve(sk: &Sketch, si: usize, unit: f64) -> Csg {
    let mut prims = Vec::new();
    let mut names = BTreeMap::new();
    let term = build(sk, si as u32, unit, &mut prims, &mut names, 0);
    Csg { prims, term }
}

fn build(
    sk: &Sketch,
    si: u32,
    unit: f64,
    prims: &mut Vec<Prim>,
    names: &mut BTreeMap<u32, Term>,
    depth: usize,
) -> Term {
    if depth > 64 {
        return Term::Empty;
    }
    if let Some(t) = names.get(&si) {
        return t.clone();
    }
    let Some(sol) = sk.solids.get(si as usize) else { return Term::Empty };
    let name = sol.name.clone();
    let t = match &sol.def {
        SolidDef::Prism { face, from, to } => match face_poly(sk, *face as usize, unit)
            .and_then(|p| prism(&p, from.value, to.value, &name))
        {
            Some(p) => {
                prims.push(p);
                Term::Prim(prims.len() - 1)
            }
            None => Term::Empty,
        },
        SolidDef::Revolve { face, axis, sweep, sense } => {
            let built = face_poly(sk, *face as usize, unit).and_then(|p| {
                let l = sk.lines.get(*axis as usize)?;
                let (c, s, o) = p.pose;
                let a = plane::in_view(c, s, o, sk.point_xy(l.p1 as usize));
                let b = plane::in_view(c, s, o, sk.point_xy(l.p2 as usize));
                revolve(&p, (a, b), sweep.value, *sense, unit, &name)
            });
            match built {
                Some(p) => {
                    prims.push(p);
                    Term::Prim(prims.len() - 1)
                }
                None => Term::Empty,
            }
        }
        SolidDef::Body { stock, on, through } => {
            let mut t = build(sk, *stock, unit, prims, names, depth + 1);
            for a in on {
                t = Term::Union(Box::new(t), Box::new(build(sk, *a, unit, prims, names, depth + 1)));
            }
            for b in through {
                t = Term::Diff(Box::new(t), Box::new(build(sk, *b, unit, prims, names, depth + 1)));
            }
            t
        }
    };
    names.insert(si, t.clone());
    t
}
