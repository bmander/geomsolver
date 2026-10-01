//! A swept solid's surface, meshed from its material field (docs/field-meshing.md).
//!
//! A solid with a continuous sweep among its operands has no facet term, so its boundary is the
//! field's, found by Delaunay refinement. `FieldMesher` runs that refinement in steps, so a host
//! may run it where it likes — a worker beside the page, handing back a `FieldSurface` at
//! intervals — and give each surface to the sketch the page draws from (`Sketch::supply_field`),
//! which then never meshes a field itself (`FieldMeshing::Deferred`). A host that does neither (the
//! terminal, a test) has the sketch mesh the field to the end when a solid is first asked for
//! (`FieldMeshing::Now`).
#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;
use crate::delaunay::refine::{Criteria, Domain, Progressive, Readings, Stage};
use super::{Query, Resolution, Source, Want};
use crate::space::box_centre_diagonal;

/// Where a swept solid's surface is meshed when a sketch is asked for the solid and holds no
/// surface supplied for it (`Sketch::field_meshing`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FieldMeshing {
    /// Here and now, to the end: seconds of refinement inside the call that asked. The terminal's
    /// and the tests'.
    #[default]
    Now,
    /// Elsewhere: the host meshes each swept solid itself (a page's worker) and supplies the
    /// surface, and until it has the solid is refused rather than meshed on the thread that draws.
    Deferred,
}

/// A field-meshed solid's facet size and surface distance, as fractions of its support's
/// diagonal: a preview's, coarse enough to mesh a small part in about a second natively.
const FIELD_FACETS: f64 = 40.0;
const FIELD_DISTANCE: f64 = 2000.0;

/// The finenesses a host may ask of `FieldMesher::with_fineness`: facets that many times smaller
/// than the preview's.  Past the top a gear's surface outgrows `Criteria::max_points`.
pub const FINENESS: std::ops::RangeInclusive<f64> = 0.25..=4.0;

/// A swept solid's surface in world coordinates: outward triangles over shared vertices.
/// `provisional` while the refinement that made it is still going, when it may be open.
#[derive(Clone, Debug, Default)]
pub struct FieldSurface {
    pub vertices: Vec<[f64; 3]>,
    pub triangles: Vec<[u32; 3]>,
    pub provisional: bool,
}

/// One surface a host that meshes swept solids elsewhere has to mesh (`Sketch::field_jobs`):
/// the solid, its name, and `key` — a digest of everything it was built from (`solid::reads`),
/// equal exactly when a surface meshed for one drawing is the surface of another. So a host
/// asks for no surface twice, re-supplies a finished one to a new elaboration of the same
/// drawing, and can check that two copies of the core are meshing the same solid.
#[derive(Clone, Debug, PartialEq)]
pub struct FieldJob {
    pub solid: usize,
    pub name: String,
    pub key: u64,
}

/// The digest `FieldJob::key` is: FNV-1a over the bits of `solid::reads`, so equal reads give
/// equal keys on every host (a JSON number could not carry it whole; it crosses as hex).
pub fn field_key(sk: &Sketch, si: usize) -> u64 {
    super::reads(sk, si, 0.0).iter().fold(0xcbf2_9ce4_8422_2325u64, |h, x| {
        x.to_bits().to_le_bytes().iter().fold(h, |h, &b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3))
    })
}

/// A material field as the refinement reads it: signs and readings alike from the sweeps'
/// adaptive distance fields (`resolution`) — what the refinement asks is a few hundred thousand
/// values, most a cell from one already asked, where each exact one is a search over the roll.
/// Read so, the readings agree with the signs (to the field's tolerance) and none continues a
/// search from the last (`Readings::Agreeing`).
struct FieldDomain {
    field: MaterialField,
    /// The sign every point is asked for, read from the sweeps' adaptive distance fields.
    sign: Query,
    resolution: Resolution,
    /// The bounding ball's radius, which scales the accuracy a reading is made to.
    radius: f64,
}

impl Domain for FieldDomain {
    fn value(&mut self, p: [f64; 3]) -> f64 { self.field.query(p, &mut self.sign).value }
    fn readings(&self) -> Readings { Readings::Agreeing }
    fn reading(&mut self, p: [f64; 3], _continues: bool) -> (f64, [f64; 3]) {
        // a crossing is placed to the bisection tolerance, so its value is needed to a tenth of
        // that, and far from the boundary to a thousandth of itself
        let mut q = Query { accuracy: 1e-6 * self.radius, source: Source::Cached(self.resolution), ..Query::at(p) };
        let r = self.field.query(p, &mut q);
        (r.value, r.gradient)
    }
}

/// Delaunay refinement of one solid's material field, a step at a time, in two passes: a coarse
/// one without features, whose edges find the field's creases (`crease::features`), then the
/// surface itself with those creases protected, so a sharp edge is kept sharp. The first pass's
/// surface is what a host shows while the creases are traced.
pub struct FieldMesher {
    run: Progressive<'static>,
    phase: Phase,
    /// The first pass's surface, shown until the second has one.
    first: Option<FieldSurface>,
    /// Creases traced, once they are.
    curves: Option<usize>,
    /// The worst facet waiting at any point of the pass running: what `progress` measures by.
    peak: f64,
    failed: bool,
}

/// Which pass a `FieldMesher` is in, holding what the next needs.
enum Phase {
    /// The first pass runs; the final pass is to be set up from this.
    First(Second),
    /// The first pass has finished and its creases are to be traced at the next step, a step of
    /// its own so a host can say so before it starts: its surface, and the final pass's setup.
    Tracing(crate::delaunay::refine::Mesh, Second),
    Final,
}

/// Where a `FieldMesher` stands (`FieldMesher::progress`): its phase — `first pass`, `tracing
/// edges`, `final pass` — the refinement's own state, the creases traced once they are, how far
/// the pass in hand has come (`within`: the worst waiting facet's badness fallen on a log scale
/// from its peak in the pass, 0 while tracing) and a fraction of the whole, the first pass a
/// quarter of it and the final pass the rest. Estimates, never a count of work left: no
/// refinement knows that in advance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FieldProgress {
    pub phase: &'static str,
    pub refine: crate::delaunay::refine::Progress,
    pub curves: Option<usize>,
    pub within: f64,
    pub fraction: f64,
    pub failed: bool,
}

impl FieldProgress {
    /// What the mesher is doing, as a host shows it: the phase, and the refinement's stage where
    /// it says more than the phase does — `first pass, building`, `final pass, repairing (rebuild
    /// 2)` — so a front end prints the core's words and compares none of them.
    pub fn doing(&self) -> String {
        let r = &self.refine;
        match r.stage {
            _ if self.phase == "tracing edges" => self.phase.to_string(),
            Stage::Repairing if r.rebuild > 0 => format!("{}, repairing (rebuild {})", self.phase, r.rebuild),
            Stage::Repairing => format!("{}, repairing", self.phase),
            Stage::Building => format!("{}, building", self.phase),
            _ => self.phase.to_string(),
        }
    }
}

struct Second {
    field: MaterialField,
    centre: [f64; 3],
    radius: f64,
    criteria: Criteria,
    /// The diagonal of the box the material is in (`MaterialField::tight_support`).
    extent: f64,
}

/// A box in space, as the fields bound things.
type Bounds = [crate::interval::Interval; 3];

/// The box `field`'s material is in (`MaterialField::tight_support`), where one is found.
fn tight(field: &MaterialField) -> Option<Bounds> { field.tight_support(6, 4096).ok().flatten() }

/// A box's diagonal, or infinity for none.
fn diagonal_of(b: Option<Bounds>) -> f64 {
    b.map_or(f64::INFINITY, |b| box_centre_diagonal(&b).1)
}

/// The first pass's facets are this much coarser than the surface's: three times, which costs a
/// fifth of the whole on a surface with no crease, and shows a rough surface at once.
const SEED_COARSENING: f64 = 3.0;

/// The adaptive distance fields' finest cells, as a multiple of the pass's facet distance.
const CACHE_CELLS: f64 = 2.0;

/// How finely a pass to `criteria` reads sweeps' distance fields: no coarser than a facet near the
/// surface, interpolated to a tenth of the surface distance, down to `CACHE_CELLS` of it.
fn resolution(criteria: &Criteria) -> Resolution {
    Resolution { finest: CACHE_CELLS * criteria.facet_distance, coarsest: criteria.facet_size,
        tolerance: 0.1 * criteria.facet_distance }
}

/// The final pass's facets keep their vertices' normals within this many degrees of each other,
/// which is what sizes them to the local feature (`Criteria::normal_angle`); the first pass asks
/// nothing of them, having no sharp edges protected, where facets spanning a crease always differ.
const NORMAL_ANGLE: f64 = 45.0;

/// A first pass's facets are never coarser than this fraction of the material's own extent.
const FIRST_FACETS: f64 = 20.0;

/// Where and how finely a first pass meshes, without features, to find them: within the box the
/// material is in (`MaterialField::tight_support`), whose centre its rays leave from — a support
/// box can be far larger than the part (a bevel blank's is its heel sphere's), and a pass sized to
/// it would be coarser than the part itself. Its facets stand off the surface by no less than a
/// thirtieth of their size: an unprotected sharp edge can never meet a finer distance, and
/// refining along it to try costs what the rest of the pass does. The centre, radius and criteria.
pub fn first_pass(field: &MaterialField, criteria: &Criteria, centre: [f64; 3], radius: f64)
    -> ([f64; 3], f64, Criteria) {
    first_pass_within(tight(field), criteria, centre, radius)
}

fn first_pass_within(tight: Option<Bounds>, criteria: &Criteria, centre: [f64; 3], radius: f64)
    -> ([f64; 3], f64, Criteria) {
    let (mut centre, mut radius, mut facet_size) = (centre, radius, SEED_COARSENING * criteria.facet_size);
    if let Some(tight) = tight {
        let (middle, diagonal) = box_centre_diagonal(&tight);
        if diagonal > 0.0 && 0.5 * diagonal * 1.05 < radius {
            centre = middle;
            radius = 0.5 * diagonal * 1.05;
            facet_size = facet_size.min(diagonal / FIRST_FACETS);
        }
    }
    let scale = facet_size / criteria.facet_size;
    (centre, radius, Criteria {
        facet_size, facet_distance: (scale * criteria.facet_distance).max(facet_size / 30.0),
        // no curves to space: the spacing sets only the least surface ball refined (a twentieth of it),
        // and a first pass refining its unprotected sharp edges to the final pass's least spent
        // more points there than the final pass does in all; a fifth of a facet is enough to find them
        edge_size: 4.0 * facet_size, max_points: 100_000, normal_angle: 0.0,
        ..criteria.clone()
    })
}

/// The creases' step is never longer than this fraction of the material's own extent.
const CREASE_STEPS: f64 = 100.0;

/// The feature curves of a field for refinement to `criteria`, found from a first pass's surface
/// (`crease::features`): traced half the feature spacing at a time, and never more than a
/// hundredth of the material's own extent (`MaterialField::tight_support`) — a gear's support box
/// is its heel sphere's, and a step sized by it crossed a tooth space whole — to a billionth of
/// the bounding ball.
pub fn creases(field: &MaterialField, first: &crate::delaunay::refine::Mesh, criteria: &Criteria, centre: [f64; 3],
    radius: f64) -> Vec<Vec<[f64; 3]>> {
    creases_within(field, first, criteria, centre, radius, diagonal_of(tight(field)))
}

fn creases_within(field: &MaterialField, first: &crate::delaunay::refine::Mesh, criteria: &Criteria, centre: [f64; 3],
    radius: f64, extent: f64) -> Vec<Vec<[f64; 3]>> {
    let options = crease::CreaseOptions {
        step: (criteria.edge_size / 2.0).min(extent / CREASE_STEPS), tolerance: 1e-9 * radius, time_gap: 0.1, centre,
        radius, max_points: 200_000,
    };
    crease::features(field, &first.vertices, &first.triangles, &options)
}

impl FieldMesher {
    pub fn new(sk: &Sketch, si: usize) -> Result<Self, String> { Self::with_fineness(sk, si, 1.0) }

    /// Refinement `fineness` times finer than the preview's: facets that much smaller, and the
    /// surface distance they may stand off the field the square of it smaller, as a chord's
    /// sagitta goes, so a finer surface is the same shape of facet over a curve, only more of them.
    pub fn with_fineness(sk: &Sketch, si: usize, fineness: f64) -> Result<Self, String> {
        let name = sk.solid_name(si);
        if !FINENESS.contains(&fineness) {
            return Err(format!("`{name}`: a mesh fineness of {fineness} is outside {} to {}",
                FINENESS.start(), FINENESS.end()));
        }
        let field = MaterialField::read(sk, si, cad::AXIS_TOLERANCE)?;
        let support = field.support_bounds().map_err(|e| format!("`{name}`: {e:?}"))?
            .ok_or_else(|| format!("`{name}`: the material has no finite support to mesh in"))?;
        let (centre, diagonal) = box_centre_diagonal(&support);
        if !(diagonal > 0.0) || !diagonal.is_finite() {
            return Err(format!("`{name}`: the material's support is empty"));
        }
        let radius = 0.5 * diagonal * 1.05;
        let facet = diagonal / (FIELD_FACETS * fineness);
        let criteria = Criteria {
            facet_size: facet, facet_distance: diagonal / (FIELD_DISTANCE * fineness * fineness), facet_angle: 25.0,
            edge_size: facet, bisection: 1e-5 * radius, max_points: 500_000, normal_angle: NORMAL_ANGLE,
        };
        let bounds = tight(&field);
        let extent = diagonal_of(bounds);
        let (near, within, coarse) = first_pass_within(bounds, &criteria, centre, radius);
        let run = Self::pass(&field, near, within, Vec::new(), coarse);
        Ok(Self { run, phase: Phase::First(Second { field, centre, radius, criteria, extent }), first: None,
            curves: None, peak: 1.0, failed: false })
    }

    fn pass(field: &MaterialField, centre: [f64; 3], radius: f64, curves: Vec<Vec<[f64; 3]>>, criteria: Criteria)
        -> Progressive<'static> {
        let resolution = resolution(&criteria);
        let sign = Query { want: Want::Sign, source: Source::Cached(resolution), ..Query::sign() };
        let domain = FieldDomain { field: field.clone(), sign, resolution, radius };
        Progressive::new(Box::new(domain), centre, radius, curves, criteria)
    }

    /// Refine at most about `budget` facets: whether the refinement has finished.
    pub fn step(&mut self, budget: usize) -> Result<bool, String> {
        match std::mem::replace(&mut self.phase, Phase::Final) {
            Phase::Tracing(coarse, Second { field, centre, radius, criteria, extent }) => {
                let curves = creases_within(&field, &coarse, &criteria, centre, radius, extent);
                self.curves = Some(curves.len());
                self.run = Self::pass(&field, centre, radius, curves, criteria);
                self.peak = 1.0;
                Ok(false)
            }
            Phase::First(second) => {
                // The first pass's surface need not close: an unprotected sharp edge may leave it
                // short of a manifold (a refinement that finishes with `done_error`), and its edges
                // still cross the creases. A step that fails — the points run out, the
                // triangulation refuses one — has no surface to go on from, and says so.
                match self.run.step(budget) {
                    Ok(false) => {
                        self.phase = Phase::First(second);
                        return Ok(false);
                    }
                    Ok(true) => {}
                    Err(e) => {
                        self.failed = true;
                        return Err(format!("the first pass stopped: {e}"));
                    }
                }
                let coarse = self.run.snapshot();
                self.first = Some(FieldSurface { vertices: coarse.vertices.clone(), triangles: coarse.triangles.clone(),
                    provisional: true });
                self.phase = Phase::Tracing(coarse, second);
                Ok(false)
            }
            Phase::Final => {
                let result = self.run.step(budget).and_then(|done| match done {
                    true => self.run.done_error().map_or(Ok(true), Err),
                    false => Ok(false),
                });
                self.failed |= result.is_err();
                result
            }
        }
    }

    /// Where the meshing stands, for a host to show (`FieldProgress`).
    pub fn progress(&mut self) -> FieldProgress {
        let refine = self.run.progress();
        self.peak = self.peak.max(refine.worst);
        let within = if refine.stage == Stage::Done || refine.worst <= 1.0 || self.peak <= 1.0 { 1.0 }
            else { (1.0 - refine.worst.dln() / self.peak.dln()).clamp(0.0, 1.0) };
        let (phase, within, fraction) = match self.phase {
            Phase::Tracing(..) => ("tracing edges", 0.0, 0.25),
            Phase::First(_) => ("first pass", within, 0.25 * within),
            Phase::Final => ("final pass", within, 0.25 + 0.75 * within),
        };
        FieldProgress { phase, refine, curves: self.curves, within, fraction, failed: self.failed }
    }

    /// The surface as it stands: provisional until the refinement has finished, and then the
    /// closed surface it finished with.
    pub fn snapshot(&mut self) -> FieldSurface {
        let m = self.run.snapshot();
        if m.triangles.is_empty() { if let Some(first) = &self.first { return first.clone(); } }
        let finished = matches!(self.phase, Phase::Final) && self.run.done();
        FieldSurface { vertices: m.vertices, triangles: m.triangles, provisional: self.failed || !finished }
    }

    /// Refine to the end: the closed surface, or why there is none.
    pub fn finish(mut self) -> Result<FieldSurface, String> {
        while !self.step(usize::MAX)? {}
        let m = self.run.finished()?;
        Ok(FieldSurface { vertices: m.vertices, triangles: m.triangles, provisional: false })
    }
}
