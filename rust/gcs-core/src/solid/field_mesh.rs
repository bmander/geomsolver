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
use super::ReadingOptions;

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
    failed: bool,
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
        edge_size: scale * criteria.edge_size, max_points: 100_000, ..criteria.clone()
    })
}

/// The feature curves of a field for refinement to `criteria`, found from a first pass's surface
/// (`crease::features`): traced a quarter of the feature spacing at a time, to a billionth of the
/// bounding ball.
pub fn creases(field: &MaterialField, first: &crate::delaunay::refine::Mesh, criteria: &Criteria, centre: [f64; 3],
    radius: f64) -> Vec<Vec<[f64; 3]>> {
    let options = crease::CreaseOptions {
        step: criteria.edge_size / 4.0, tolerance: 1e-9 * radius, time_gap: 0.1, centre, radius, max_points: 200_000,
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
            edge_size: facet, bisection: 1e-5 * radius, max_points: 500_000,
        };
        let (near, within, coarse) = first_pass(&field, &criteria, centre, radius);
        let run = Self::pass(&field, near, within, Vec::new(), coarse);
        Ok(Self { run, second: Some(Second { field, centre, radius, criteria }), first: None, failed: false })
    }

    fn pass(field: &MaterialField, centre: [f64; 3], radius: f64, curves: Vec<Vec<[f64; 3]>>, criteria: Criteria)
        -> Progressive<'static> {
        let (side, reader) = (field.clone(), field.clone());
        Progressive::new(Box::new(move |p| side.side(p)), centre, radius, curves, criteria)
            .with_reading(Box::new({
                // the contact times of the last reading, for the next one along the same crossing
                let mut hints = Vec::new();
                move |p, warm| {
                    // a crossing is placed to the bisection tolerance, so its value is needed to a
                    // tenth of that, and far from the boundary to a thousandth of itself
                    let options = ReadingOptions { accuracy: 1e-6 * radius, local: warm, ..ReadingOptions::at(p) };
                    if !warm { hints.clear(); }
                    let r = reader.reading_warm(p, &options, &mut 0, &mut hints);
                    (r.value, r.gradient)
                }
            }))
    }

    /// Refine at most about `budget` facets: whether the refinement has finished.
    pub fn step(&mut self, budget: usize) -> Result<bool, String> {
        if self.second.is_some() {
            // The first pass's surface need not close: an unprotected sharp edge may leave it
            // short of a manifold, and its edges still cross the creases.
            if !self.run.step(budget).unwrap_or(true) { return Ok(false); }
            let Second { field, centre, radius, criteria } = self.second.take().unwrap();
            let coarse = self.run.snapshot();
            let curves = creases(&field, &coarse, &criteria, centre, radius);
            self.first = Some(FieldSurface { vertices: coarse.vertices, triangles: coarse.triangles, provisional: true });
            self.run = Self::pass(&field, centre, radius, curves, criteria);
            return Ok(false);
        }
        let result = self.run.step(budget).and_then(|done| match done {
            true => self.run.done_error().map_or(Ok(true), Err),
            false => Ok(false),
        });
        self.failed |= result.is_err();
        result
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
