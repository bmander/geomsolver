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

/// Delaunay refinement of one solid's material field, a step at a time.
pub struct FieldMesher {
    run: Progressive<'static>,
    failed: bool,
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
        let reader = field.clone();
        let run = Progressive::new(Box::new(move |p| field.side(p)), centre, radius, Vec::new(), criteria)
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
            }));
        Ok(Self { run, failed: false })
    }

    /// Refine at most about `budget` facets: whether the refinement has finished.
    pub fn step(&mut self, budget: usize) -> Result<bool, String> {
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
        FieldSurface { vertices: m.vertices, triangles: m.triangles, provisional: self.failed || !self.run.done() }
    }

    /// Refine to the end: the closed surface, or why there is none.
    pub fn finish(mut self) -> Result<FieldSurface, String> {
        while !self.run.step(usize::MAX)? {}
        let m = self.run.finished()?;
        Ok(FieldSurface { vertices: m.vertices, triangles: m.triangles, provisional: false })
    }
}
