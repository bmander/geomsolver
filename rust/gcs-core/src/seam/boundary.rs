//! The implicit intersection of a generated face and a finite analytic boundary.
use crate::{envelope::{check_tolerance,Contact,Error,Intersection,IntersectionOptions},
    model::{EntKind,Sketch},patch::EnvelopePatch,solid::SurfaceProjector};

/// Explicit checks for a retained boundary-seam point. Normal velocity is per
/// radian of generating roll; incidence and material trim tolerances are lengths.
#[derive(Clone,Copy,Debug)]
pub struct BoundarySeamTolerance {
    pub normal_velocity: f64,
    pub incidence: f64,
    pub trim: f64,
}

impl BoundarySeamTolerance {
    fn check(self) -> Result<(),Error> {
        for t in [self.normal_velocity,self.incidence,self.trim] { check_tolerance(t)?; }
        Ok(())
    }
}

/// One generated face and one finite boundary surface, each read exactly once.
/// The seam's chart is the envelope's original [u,v,roll]. Reading a snapshot does
/// not select a connected branch or certify transversality. Local section solves
/// check all three equations and rank through the common intersection solver.
#[derive(Clone,Debug)]
pub struct BoundarySeam {
    pub name: String,
    face: EnvelopePatch,
    boundary: SurfaceProjector,
}

impl BoundarySeam {
    pub fn named(sk: &Sketch,index: usize,axis_tolerance: f64) -> Result<Self,String> {
        check_tolerance(axis_tolerance).map_err(|_| "invalid clipping-axis tolerance")?;
        let s = sk.seams.get(index).ok_or("no such seam")?;
        if s.second.kind != EntKind::Surface || s.first.kind == EntKind::Surface {
            return Err("a boundary seam needs an envelope followed by a boundary surface".into());
        }
        super::validate(sk,[s.first,s.second])?;
        Ok(Self {name:s.name.clone(),face:EnvelopePatch::read(sk,s.first,axis_tolerance)?,
            boundary:SurfaceProjector::named(sk,s.second.i())?})
    }

    /// A search domain, not a rectangular parameterization of the curve itself.
    pub fn domain(&self) -> [[f64;2];3] { self.face.envelope().domain() }

    pub fn evaluate(&self,p: [f64;3],tolerance: BoundarySeamTolerance) -> Result<Contact,Error> {
        tolerance.check()?;
        let c = self.face.at(p,tolerance.normal_velocity,tolerance.trim)?;
        let b = self.boundary.project(c.position)?;
        if b.incidence_error > tolerance.incidence { return Err(Error::OutsideDomain); }
        Ok(c)
    }

    /// Solve the envelope equation, boundary support equation and supplied spatial
    /// section. Their tolerances and returned residuals follow that order. Trials
    /// may leave material trims or finite boundary spans; retained roots cannot.
    pub fn intersect(&self,section: impl Fn(&Contact) -> f64,seed: [f64;3],
        options: IntersectionOptions,trim_tolerance: f64) -> Result<Intersection,Error> {
        check_tolerance(trim_tolerance)?;
        let mut result = self.face.envelope().intersect(|_,c| [
            self.boundary.project(c.position).map(|p| p.signed_residual)
                .unwrap_or(f64::NAN),section(c)],seed,options)?;
        result.contact = self.evaluate(result.parameters,BoundarySeamTolerance {
            normal_velocity:options.residual_tolerance[0],incidence:options.residual_tolerance[1],
            trim:trim_tolerance,
        })?;
        Ok(result)
    }
}
