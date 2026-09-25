//! A swept solid's surface, meshed from its material field (docs/field-meshing.md).
//!
//! A solid with a continuous sweep among its operands has no facet term, so its boundary is the
//! field's, found by Delaunay refinement. `FieldMesher` runs that refinement in steps, so a host
//! may run it where it likes — a worker beside the page, handing back a `FieldSurface` at
//! intervals — and give each surface to the sketch the page draws from (`Sketch::supply_field`),
//! which then never meshes a field itself (`Sketch::defer_fields`). A host that does neither (the
//! terminal, a test) has the sketch mesh the field to the end when a solid is first asked for.
use super::*;
use crate::delaunay::refine::{Criteria, Progressive};
use super::{ReadingOptions, Resolution};

/// A field-meshed solid's facet size and surface distance, as fractions of its support's
/// diagonal: a preview's, coarse enough to mesh a small part in about a second natively.
const FIELD_FACETS: f64 = 40.0;
const FIELD_DISTANCE: f64 = 2000.0;

/// A swept solid's surface in world coordinates: outward triangles over shared vertices.
/// `provisional` while the refinement that made it is still going, when it may be open.
#[derive(Clone, Debug, Default)]
pub struct FieldSurface {
    pub vertices: Vec<[f64; 3]>,
    pub triangles: Vec<[u32; 3]>,
    pub provisional: bool,
}

/// Delaunay refinement of one solid's material field, a step at a time, in two passes: a coarse
/// one without features, whose edges find the field's creases (`crease::features`), then the
/// surface itself with those creases protected, so a sharp edge is kept sharp. The first pass's
/// surface is what a host shows while the creases are traced.
pub struct FieldMesher {
    run: Progressive<'static>,
    /// While the first pass runs: what the second needs.
    second: Option<Second>,
    /// The first pass's surface, shown until the second has one.
    first: Option<FieldSurface>,
    /// The first pass has finished and its creases are to be traced at the next step: a step of
    /// its own, so a host can say so before it starts.
    tracing: Option<crate::delaunay::refine::Mesh>,
    /// Creases traced, once they are.
    curves: Option<usize>,
    /// The worst facet waiting at any point of the pass running: what `progress` measures by.
    peak: f64,
    failed: bool,
}

/// Where a `FieldMesher` stands (`FieldMesher::progress`): its phase — `first pass`, `tracing
/// edges`, `final pass` — the refinement's own state, the creases traced once they are, and a
/// fraction of the whole: the worst waiting facet's badness fallen on a log scale from its peak in
/// the pass, the first pass a quarter of the whole and the final pass the rest. An estimate,
/// never a count of work left: no refinement knows that in advance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FieldProgress {
    pub phase: &'static str,
    pub refine: crate::delaunay::refine::Progress,
    pub curves: Option<usize>,
    pub fraction: f64,
    pub failed: bool,
}

struct Second {
    field: MaterialField,
    centre: [f64; 3],
    radius: f64,
    criteria: Criteria,
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
    let (mut centre, mut radius, mut facet_size) = (centre, radius, SEED_COARSENING * criteria.facet_size);
    if let Ok(Some(tight)) = field.tight_support(6, 4096) {
        let [lo, hi] = [0, 1].map(|k| tight.map(|x| x.bounds()[k]));
        let diagonal = (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt();
        if diagonal > 0.0 && 0.5 * diagonal * 1.05 < radius {
            centre = std::array::from_fn(|k| 0.5 * (lo[k] + hi[k]));
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
/// (`crease::features`): traced a quarter of the feature spacing at a time, and never more than a
/// two-hundredth of the material's own extent (`MaterialField::tight_support`) — a gear's support
/// box is its heel sphere's, and a step sized by it crossed a tooth space whole — to a billionth
/// of the bounding ball.
pub fn creases(field: &MaterialField, first: &crate::delaunay::refine::Mesh, criteria: &Criteria, centre: [f64; 3],
    radius: f64) -> Vec<Vec<[f64; 3]>> {
    let extent = field.tight_support(6, 4096).ok().flatten().map_or(f64::INFINITY, |b| {
        b.iter().map(|x| { let [lo, hi] = x.bounds(); (hi - lo) * (hi - lo) }).sum::<f64>().sqrt()
    });
    let options = crease::CreaseOptions {
        step: (criteria.edge_size / 2.0).min(extent / CREASE_STEPS), tolerance: 1e-9 * radius, time_gap: 0.1, centre,
        radius, max_points: 200_000,
    };
    crease::features(field, &first.vertices, &first.triangles, &options)
}

impl FieldMesher {
    pub fn new(sk: &Sketch, si: usize) -> Result<Self, String> {
        let name = sk.solid_name(si);
        let field = MaterialField::read(sk, si, 1e-10)?;
        let support = field.support_bounds().map_err(|e| format!("`{name}`: {e:?}"))?
            .ok_or_else(|| format!("`{name}`: the material has no finite support to mesh in"))?;
        let [lo, hi] = [0, 1].map(|k| support.map(|x| x.bounds()[k]));
        let centre: [f64; 3] = std::array::from_fn(|k| 0.5 * (lo[k] + hi[k]));
        let diagonal = (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt();
        if !(diagonal > 0.0) || !diagonal.is_finite() {
            return Err(format!("`{name}`: the material's support is empty"));
        }
        let radius = 0.5 * diagonal * 1.05;
        let facet = diagonal / FIELD_FACETS;
        let criteria = Criteria {
            facet_size: facet, facet_distance: diagonal / FIELD_DISTANCE, facet_angle: 25.0,
            edge_size: facet, bisection: 1e-5 * radius, max_points: 500_000, normal_angle: NORMAL_ANGLE,
        };
        let (near, within, coarse) = first_pass(&field, &criteria, centre, radius);
        let r = resolution(&coarse);
        let run = Self::pass(&field, near, within, Vec::new(), coarse, r);
        Ok(Self { run, second: Some(Second { field, centre, radius, criteria }), first: None, tracing: None,
            curves: None, peak: 1.0, failed: false })
    }

    fn pass(field: &MaterialField, centre: [f64; 3], radius: f64, curves: Vec<Vec<[f64; 3]>>, criteria: Criteria,
        resolution: Resolution) -> Progressive<'static> {
        let (side, reader) = (field.clone(), field.clone());
        // Sweeps are read from their adaptive distance fields (`resolution`): what the refinement
        // asks is a few hundred thousand values, most a cell from one already asked, where each
        // exact one is a search over the roll.
        Progressive::new(Box::new(move |p| side.side_cached(p, resolution)), centre, radius, curves, criteria)
            .with_reading(Box::new({
                // the contact times of the last reading, for the next one along the same crossing
                let mut hints = Vec::new();
                move |p, warm| {
                    // a crossing is placed to the bisection tolerance, so its value is needed to a
                    // tenth of that, and far from the boundary to a thousandth of itself; every
                    // sweep is read from its distance field, as `side_cached` reads it, so the two
                    // agree (to the field's tolerance) and nothing is continued locally
                    let options = ReadingOptions { accuracy: 1e-6 * radius, cached: Some(resolution), ..ReadingOptions::at(p) };
                    if !warm { hints.clear(); }
                    let r = reader.reading_warm(p, &options, &mut 0, &mut hints);
                    (r.value, r.gradient)
                }
            }))
            .readings_agree()
    }

    /// Refine at most about `budget` facets: whether the refinement has finished.
    pub fn step(&mut self, budget: usize) -> Result<bool, String> {
        if let Some(coarse) = self.tracing.take() {
            let Second { field, centre, radius, criteria } = self.second.take().unwrap();
            let curves = creases(&field, &coarse, &criteria, centre, radius);
            self.curves = Some(curves.len());
            let r = resolution(&criteria);
            self.run = Self::pass(&field, centre, radius, curves, criteria, r);
            self.peak = 1.0;
            return Ok(false);
        }
        if self.second.is_some() {
            // The first pass's surface need not close: an unprotected sharp edge may leave it
            // short of a manifold, and its edges still cross the creases.
            if !self.run.step(budget).unwrap_or(true) { return Ok(false); }
            let coarse = self.run.snapshot();
            self.first = Some(FieldSurface { vertices: coarse.vertices.clone(), triangles: coarse.triangles.clone(),
                provisional: true });
            self.tracing = Some(coarse);
            return Ok(false);
        }
        let result = self.run.step(budget).and_then(|done| match done {
            true => self.run.done_error().map_or(Ok(true), Err),
            false => Ok(false),
        });
        self.failed |= result.is_err();
        result
    }

    /// The pass in hand's refinement counts.
    pub fn report(&self) -> crate::delaunay::refine::Report { self.run.report() }

    /// Where the meshing stands, for a host to show (`FieldProgress`).
    pub fn progress(&mut self) -> FieldProgress {
        let refine = self.run.progress();
        self.peak = self.peak.max(refine.worst);
        let within = if refine.stage == "done" || refine.worst <= 1.0 || self.peak <= 1.0 { 1.0 }
            else { (1.0 - refine.worst.ln() / self.peak.ln()).clamp(0.0, 1.0) };
        let (phase, fraction) = if self.tracing.is_some() { ("tracing edges", 0.25) }
            else if self.second.is_some() { ("first pass", 0.25 * within) }
            else { ("final pass", 0.25 + 0.75 * within) };
        FieldProgress { phase, refine, curves: self.curves, fraction, failed: self.failed }
    }

    /// The surface as it stands: provisional until the refinement has finished, and then the
    /// closed surface it finished with.
    pub fn snapshot(&mut self) -> FieldSurface {
        let m = self.run.snapshot();
        if m.triangles.is_empty() { if let Some(first) = &self.first { return first.clone(); } }
        let finished = self.second.is_none() && self.run.done();
        FieldSurface { vertices: m.vertices, triangles: m.triangles, provisional: self.failed || !finished }
    }

    /// Refine to the end: the closed surface, or why there is none.
    pub fn finish(mut self) -> Result<FieldSurface, String> {
        while !self.step(usize::MAX)? {}
        let m = self.run.finished()?;
        Ok(FieldSurface { vertices: m.vertices, triangles: m.triangles, provisional: false })
    }
}
