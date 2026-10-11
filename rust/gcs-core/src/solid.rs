//! **What a solid is, and the one question everything asks of it** (Solvent §6.9).
//!
//! A feature tree is imperative because it is *stateful*: step *n* acts on the anonymous "body
//! as of step *n − 1*" and names faces by the order they were made in.  Solvent names
//! everything, so a solid here is a **term** — its stock, plus everything in `union` with it,
//! minus everything that `cut`s it — over primitives that are faces swept.  The order lives
//! inside the term, over names, exactly as it lives inside `h = w / 2`; between statements there
//! is no order at all, which is P2 and is what a feature tree cannot have.
//!
//! **Nothing three-dimensional is ever solved for.**  A solid owns no parameter: every extent is
//! an expression the flattener settled, and the geometry it is swept from is the drawing, solved
//! in 2D as it always was.  The strata run one way — the sketch solves, the depths are worked
//! out, the terms are ordered, and the outputs are read — with no edge back.
//!
//! **Consumers share a validated `EvaluatedSolid` for a solved pose and approximation policy.**
//! It owns the local frame, CSG classifier, retained boundary and provenance, with lazy edges
//! and mesh. The kernel remains classification against `Csg` (Requicha & Voelcker's boundary
//! evaluation); no additional B-rep or replacement CSG engine is introduced.
//!
//! **One rule holds the whole thing together: the classifier reads the facets the candidates are
//! cut from.**  A primitive is reduced to a closed polyhedron of planar convex facets — arcs and
//! circles tessellated by the sagitta rule the drawing itself is drawn by, a revolution faceted
//! about its axis the same way — and `classify` casts a ray against *those* facets and no ideal
//! surface.  Classify exactly against the true circle instead, and every facet centroid of a
//! bore's wall would lie inside the true bore by the sagitta, both its samples would read
//! *outside*, and the wall would silently vanish.

#[allow(unused_imports)]
use crate::fmath::Det;
mod evaluated;
mod exact;
pub use exact::Exact;
pub(crate) use exact::build as build_exact;
mod field_mesh;
pub use field_mesh::{FieldMesher, FieldMeshing, FieldProgress, FieldSurface, ExactFaces, FieldJob, field_key, FINENESS as FIELD_FINENESS};
pub use field_mesh::{first_pass as field_first_pass, creases as field_creases};
pub mod cad;
pub mod region;
mod profile;
mod primitive;
mod document;
pub use profile::{FacePoly,face_poly};
pub(crate) use profile::ring_area;
pub(crate) use section::inside_ring;
use profile::loop_poly;
pub use primitive::{prism,revolve};
pub(crate) use primitive::area_vector;
use primitive::{facet_normal,finish};
pub use document::{reads,resolve};
mod static_boundary;
pub use static_boundary::{static_solid,indexed,indexed_faces,primitive_triangles,StaticSolid,static_solid_at_unit};
pub(crate) use document::{evaluation_operands,operand_paths};
use document::{frame_origin,resolve_at};
mod field;
pub use field::{PlanarField,RevolvedField,ExtrudedField,SpatialField,SweptField,SweepEvaluator,SweepError,CHORD_SLACK};
pub use field::{MaterialField,MaterialEvaluator,MaterialBounds,MaterialSweepQuery,SIDE_EVALUATIONS};
pub use field::{Millimetres,MillimetreEvaluator};
pub use field::{OperandId,Query,Reading,Source,Want,Resolution,crease};
pub use field::{MaterialProbe,ProbeState};
pub use field::{BoundaryOptions,BoundaryError,BoundaryStage,FieldBoundary,BoundaryCell,BoundaryPoint,BoundaryCrossing};
pub use field::BoundaryComponent;
mod raycast;
mod section;
mod loft;
pub(crate) mod surface;
mod sweep_contacts;
pub mod admission;
pub mod constant_twist;
pub mod planar_class;
pub mod agreement;
pub mod accuracy;
pub mod contracts;
pub mod sector;
pub mod export;
pub mod contact_trace;
pub mod blank_features;
pub use sweep_contacts::{ExtrudedSurface,SweepContacts,TimedContact,PatchEdge,PointContactError,ToolSurface};
pub use surface::{RegionLocation,RegionSample,RevolvedRegion,RevolvedSurface,
    SurfaceProjection,SurfaceProjector,RevolvedContact};
pub(crate) use raycast::RayIndex;
use section::face_polys;
pub use evaluated::{ApproximationPolicy, EvaluatedSolid, LocalPoint, WorldPoint, PagePoint, PageFrame, RoundFeature};

use crate::model::{EntKind, EntRef, Sense, Sketch, SolidDef};
use crate::plane::{self, Basis};
use std::collections::BTreeMap;

/// How far off a face a sample is pushed before it is classified, as a fraction of the solid's
/// smallest positive primitive dimension. Comfortably above `SNAP` so no sliver survives
/// between the two, and comfortably below any feature a person draws.
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
    let origin = frame_origin(sk, i, REPORT_UNIT * 20.0);
    let b = resolve_at(sk, i, REPORT_UNIT * 20.0, origin).bbox();
    if b.is_empty() {
        return REPORT_UNIT;
    }
    let diag = (0..3).map(|k| (b.hi[k] - b.lo[k]).dpowi(2)).sum::<f64>().sqrt();
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
    /// Whether the line `p + t·d`, for `t` past `near`, passes through the box (the slab test):
    /// a ray from `p` with `near` 0, the whole line with −∞.
    pub fn meets(&self, p: [f64; 3], d: [f64; 3], near: f64) -> bool {
        if self.is_empty() { return false }
        let (mut near, mut far) = (near, f64::INFINITY);
        for k in 0..3 {
            if d[k] == 0.0 {
                if p[k] < self.lo[k] || p[k] > self.hi[k] { return false }
            } else {
                let a = (self.lo[k] - p[k]) / d[k];
                let z = (self.hi[k] - p[k]) / d[k];
                near = near.max(a.min(z));
                far = far.min(a.max(z));
                if far < near { return false }
            }
        }
        true
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
    /// The facets are an exact B-rep's mesh (`solid::Exact`): each of `faces` is a whole path, `of`
    /// prefixes nothing, and two faces meet at a crease however their facets lie.
    pub exact: bool,
}

/// The term a solid *is*.  Order lives here, over names, and nowhere else.
#[derive(Clone, Debug)]
pub enum Term {
    Prim(usize),
    Union(Box<Term>, Box<Term>),
    Diff(Box<Term>, Box<Term>),
    /// What two operands share: the `bound` side of the body rule (§6.9).
    Inter(Box<Term>, Box<Term>),
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
    /// Classification tolerance depends only on this term's primitives, never on sheet layout.
    /// Use the smallest positive primitive dimension so a long thin part keeps its thin features.
    pub fn epsilon(&self) -> f64 {
        let feature = self
            .prims
            .iter()
            .flat_map(|p| (0..3).map(move |k| p.bbox.hi[k] - p.bbox.lo[k]))
            .filter(|d| *d > 0.0 && d.is_finite())
            .fold(f64::INFINITY, f64::min);
        if feature.is_finite() {
            feature * EPS
        } else {
            EPS
        }
    }

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
        self.eval(&self.term, p, None)
    }

    pub(crate) fn inside_indexed(&self, p: [f64; 3], indices: &[RayIndex]) -> bool {
        self.eval(&self.term, p, Some(indices))
    }

    fn eval(&self, t: &Term, p: [f64; 3], indices: Option<&[RayIndex]>) -> bool {
        match t {
            Term::Empty => false,
            Term::Prim(i) => in_prim(&self.prims[*i], p, indices.map(|v| &v[*i])),
            Term::Union(a, b) => self.eval(a, p, indices) || self.eval(b, p, indices),
            Term::Diff(a, b) => self.eval(a, p, indices) && !self.eval(b, p, indices),
            Term::Inter(a, b) => self.eval(a, p, indices) && self.eval(b, p, indices),
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

fn in_prim(prim: &Prim, p: [f64; 3], index: Option<&RayIndex>) -> bool {
    if !prim.bbox.grown(1e-12).holds(p) {
        return false;
    }
    for d in RAYS {
        if let Some(hits) = cast(prim, p, d, index) {
            return hits % 2 == 1;
        }
    }
    false
}

/// Crossings of the ray `p + t·d`, `t > 0`, with the primitive's facets.  `None` when the ray
/// passes too near an edge for the count to be trusted — the caller tries another direction.
fn cast(prim: &Prim, p: [f64; 3], d: [f64; 3], index: Option<&RayIndex>) -> Option<usize> {
    let mut hits = 0usize;
    let mut visit = |i: usize| {
        let f = &prim.facets[i];
        let denom = plane::dot(f.n, d);
        if denom.abs() < 1e-12 {
            return Some(());
        }
        let t = (f.offset() - plane::dot(f.n, p)) / denom;
        if t <= 0.0 {
            return Some(());
        }
        let x = [p[0] + t * d[0], p[1] + t * d[1], p[2] + t * d[2]];
        match in_facet(f, x) {
            Hit::In => hits += 1,
            Hit::Out => {}
            Hit::Edge => return None,
        }
        Some(())
    };
    if let Some(index) = index {
        index.visit(p, d, &mut visit)?;
    } else {
        for i in 0..prim.facets.len() { visit(i)?; }
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

/// Validate solved geometry, not the hint that existed before the constraint solve.
/// Exporters and diagnostics share this check, including every operand of a body.
pub fn validate(sk: &Sketch, si: usize) -> Result<(), String> {
    validate_at(sk, si, REPORT_UNIT).map(|_| ())
}

/// Whether a continuous motion sweep is among the solid's operands: its boundary is then the
/// field's, meshed once whatever approximation is asked for. False for a solid that does not
/// validate, whose operands are not known.
pub(crate) fn has_sweep(sk: &Sketch, si: usize) -> bool {
    validate_at(sk, si, REPORT_UNIT).map(|ops| sweeps_among(sk, &ops)).unwrap_or(false)
}

/// Whether a continuous motion sweep is among `operands` (`validate_at`'s): the one test, asked by
/// `has_sweep` and by an evaluation that has validated the operands already at its own unit.
fn sweeps_among(sk: &Sketch, operands: &std::collections::BTreeSet<usize>) -> bool {
    operands.iter().any(|&i| matches!(sk.solids[i].def, SolidDef::Swept { .. }))
}

fn validate_at(sk: &Sketch, si: usize, unit: f64) -> Result<std::collections::BTreeSet<usize>, String> {
    let mut pending = vec![(si, false)];
    let mut seen = std::collections::BTreeSet::new();
    let mut active = std::collections::BTreeSet::new();
    while let Some((i, ready)) = pending.pop() {
        if seen.contains(&i) { continue; }
        let s = sk.solids.get(i).ok_or_else(|| format!("no solid at index {i}"))?;
        if ready { active.remove(&i); seen.insert(i); continue; }
        if !active.insert(i) { return Err(format!("`{}`: cyclic solid operands", s.name)); }
        pending.push((i, true));
        let fail = |why: &str| format!("`{}`: {why}", s.name);
        let face = match &s.def {
            SolidDef::Swept { source, motion, from, to } => {
                if !from.value.is_finite() || !to.value.is_finite() || from.value >= to.value {
                    return Err(fail("a continuous motion sweep needs finite increasing angular bounds"));
                }
                let family = crate::motion::Family::read(sk,*motion as usize)?;
                family.at(from.value)?; family.at(to.value)?;
                pending.push((*source as usize,false));
                continue;
            }
            SolidDef::Placed { source, motion, at } => {
                crate::motion::Family::read(sk,*motion as usize)?.at(at.value)?;
                pending.push((*source as usize,false));
                continue;
            }
            SolidDef::Prism { face, from, to } => {
                if !from.value.is_finite() || !to.value.is_finite() || from.value == to.value {
                    return Err(fail("a prism needs two distinct finite ordinates"));
                }
                *face
            }
            SolidDef::Through { face, .. } => {
                pending.extend(evaluation_operands(sk, i)?.into_iter().rev().map(|o| (o as usize, false)));
                *face
            }
            SolidDef::Revolve { face, .. } => *face,
            SolidDef::Loft { face, end, guide } => {
                loft::prepare(sk, *face, *end, *guide, unit).map_err(|m| fail(&m))?;
                continue;
            }
            SolidDef::Body { .. } => { pending.extend(s.operands().into_iter().rev().map(|o| (o as usize, false))); continue; }
            // its terms on one axis, bounded all round: what its meridian says
            SolidDef::Region { .. } => {
                region::meridian(sk, i).map_err(|m| fail(&m))?;
                continue;
            }
        };
        let polys = face_polys(sk, face as usize, unit).map_err(|m| fail(&m))?;
        let poly = &polys[0];
        let reserved: &[&str] = match &s.def {
            SolidDef::Prism { .. } | SolidDef::Through { .. } => &["near", "far"],
            SolidDef::Revolve { sweep, .. } if sweep.value < std::f64::consts::TAU - 1e-9 => {
                &["start", "end"]
            }
            _ => &[],
        };
        if let Some(name) = polys.iter().flat_map(|p| &p.names).find(|n| reserved.contains(&n.as_str())) {
            return Err(fail(&format!(
                "edge `{name}` collides with a sweep cap; rename the source edge"
            )));
        }
        if let SolidDef::Revolve { axis, sweep, .. } = &s.def {
            if !sweep.value.is_finite() || sweep.value <= 0.0 {
                return Err(fail("a revolution needs a finite positive sweep"));
            }
            let axis = sk.lines.get(*axis as usize).ok_or_else(|| fail("no revolution axis"))?;
            let (a, b) = (sk.point_xy(axis.p1 as usize), sk.point_xy(axis.p2 as usize));
            let length = (b.0 - a.0).dhypot(b.1 - a.1);
            if !length.is_finite() || length <= 0.0 { return Err(fail("a revolution axis must have nonzero finite length")); }
            let distances: Vec<_> = poly.pts.iter().map(|p|
                (-(p.0 - a.0) * (b.1 - a.1) + (p.1 - a.1) * (b.0 - a.0)) / length).collect();
            let tol = distances.iter().fold(0.0f64, |m, x| m.max(x.abs())) * SNAP;
            if distances.iter().any(|&x| x > tol) && distances.iter().any(|&x| x < -tol) {
                return Err(fail("the revolution axis crosses the profile"));
            }
        }
    }
    Ok(seen)
}

pub fn bearing_errors(sk: &Sketch) -> Vec<(&crate::model::SolidBearing, String)> {
    sk.solid_bearings.iter().filter_map(|b| {
        let exists = sk.evaluated_solid(b.solid as usize, ApproximationPolicy::Report)
            .is_ok_and(|s| s.surviving_faces().contains(&b.path));
        (!exists).then(|| (b, format!("`{}` has no surviving face to bear on", b.path)))
    }).collect()
}
