//! Spatial corners at intersections of seams, made of their parts.  Identity is the caller's:
//! coincidence of two separately built corners does not merge them.
use crate::{envelope::{check_tolerance,Error,Intersection,IntersectionOptions},
    patch::EnvelopePatch,
    seam::{BoundarySeamTolerance,EnvelopeSeam,SeamIntersectionOptions},
    solid::SurfaceProjector};

/// One checked local root. Parameters remain in the canonical generating chart;
/// residual meanings follow the selected corner's solve method. A vertex has no
/// unique face normal or generating velocity, so neither is exposed as its own.
#[derive(Clone,Copy,Debug)]
pub struct SolvedVertex {
    pub position: [f64;3],
    pub parameters: [f64;3],
    pub residuals: [f64;3],
    pub iterations: u32,
}

impl From<Intersection> for SolvedVertex {
    fn from(p: Intersection) -> Self {
        Self {position:p.contact.position,parameters:p.parameters,residuals:p.residuals,iterations:p.iterations}
    }
}

fn incidence(boundary: &SurfaceProjector,p: [f64;3],tolerance: f64) -> Result<(),Error> {
    check_tolerance(tolerance)?;
    if boundary.project(p)?.incidence_error > tolerance { Err(Error::OutsideDomain) } else { Ok(()) }
}

/// A generated face meeting two finite surfaces. It owns one copy of that face
/// with its material conditions, not two boundary-seam copies of the same face.
#[derive(Clone,Debug)]
pub struct BoundaryVertex {
    pub name: String,
    face: EnvelopePatch,
    boundaries: [SurfaceProjector;2],
}

impl BoundaryVertex {
    /// A generating face meeting two finite surfaces, made of its parts (#212).
    pub fn new(name: String,face: &crate::patch::TrimmedPatch,boundaries: [SurfaceProjector;2]) -> Result<Self,String> {
        Ok(Self {name,face:face.envelope_patch()?.clone(),boundaries})
    }

    pub fn domain(&self) -> [[f64;2];3] { self.face.envelope().domain() }

    pub fn position(&self,p: [f64;3],tolerance: BoundarySeamTolerance) -> Result<[f64;3],Error> {
        check_tolerance(tolerance.incidence)?;
        let c = self.face.at(p,tolerance.normal_velocity,tolerance.trim)?;
        for b in &self.boundaries { incidence(b,c.position,tolerance.incidence)?; }
        Ok(c.position)
    }

    /// Residuals and tolerances: envelope, first boundary, second boundary.
    /// Bounds and seeds restrict a local search; they do not prove global uniqueness.
    pub fn solve(&self,seed: [f64;3],options: IntersectionOptions,trim_tolerance: f64)
        -> Result<SolvedVertex,Error> {
        check_tolerance(trim_tolerance)?;
        let mut result = self.face.envelope().intersect_boundaries(
            [&self.boundaries[0],&self.boundaries[1]],seed,options)?;
        result.contact = self.face.at(result.parameters,options.residual_tolerance[0],trim_tolerance)?;
        Ok(result.into())
    }
}

/// A tangent generating junction meeting a finite surface. The boundary seam's
/// generating face is already one of the junction's faces and is not copied again.
#[derive(Clone,Debug)]
pub struct JunctionVertex {
    pub name: String,
    junction: EnvelopeSeam,
    boundary: SurfaceProjector,
    boundary_face: usize,
}

impl JunctionVertex {
    /// A generating junction meeting a finite surface on its face `boundary_face` (0 or 1, in the
    /// junction's operand order), made of its parts (#212).
    pub fn new(name: String,junction: EnvelopeSeam,boundary: SurfaceProjector,boundary_face: usize) -> Result<Self,String> {
        if boundary_face > 1 { return Err("a junction has two faces".into()); }
        Ok(Self {name,junction,boundary,boundary_face})
    }

    /// Original chart of the junction's first face, with its u fixed structurally.
    pub fn domain(&self) -> [[f64;2];3] { self.junction.domain() }

    pub fn position(&self,p: [f64;3],tolerance: BoundarySeamTolerance) -> Result<[f64;3],Error> {
        check_tolerance(tolerance.incidence)?;
        let c = self.junction.contacts(p,tolerance.normal_velocity,tolerance.trim)?;
        incidence(&self.boundary,c[self.boundary_face].position,tolerance.incidence)?;
        Ok(c[0].position)
    }

    /// Only [v,roll] are unknown. Residuals and tolerances are both generating
    /// equations followed by boundary incidence, in the junction's operand order.
    pub fn solve(&self,seed: [f64;2],options: SeamIntersectionOptions,trim_tolerance: f64)
        -> Result<SolvedVertex,Error> {
        check_tolerance(trim_tolerance)?;
        let result = self.junction.intersect(|c| self.boundary.project(c.position)
            .map(|p| p.signed_residual).unwrap_or(f64::NAN),seed,options,trim_tolerance)?;
        self.position(result.parameters,BoundarySeamTolerance {
            normal_velocity:options.normal_tolerance,incidence:options.section_tolerance,
            trim:trim_tolerance,
        })?;
        Ok(result.into())
    }
}
