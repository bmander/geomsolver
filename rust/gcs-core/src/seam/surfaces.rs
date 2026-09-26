//! Local transverse intersections of two finite analytic surface patches.
use crate::{envelope::{check_tolerance,Error,IntersectionOptions,SurfacePoint},
    model::Sketch,plane::cross,solid::{RevolvedSurface,SurfaceProjection,SurfaceProjector}};

use crate::space::length;

type V = [f64;3];
fn unit(v: V) -> Result<V,Error> {
    let n = length(v);
    if !n.is_finite() || n == 0. { Err(Error::Degenerate) } else { Ok(v.map(|x| x/n)) }
}
fn controls(incidence: f64,transverse: f64) -> Result<(),Error> {
    check_tolerance(incidence)?; check_tolerance(transverse)?;
    if transverse >= 1. { Err(Error::InvalidOptions) } else { Ok(()) }
}

#[derive(Clone,Copy,Debug)]
pub struct SurfaceSeamPoint {
    /// Position on operand one; operand two lies within incidence_error.
    pub position: V,
    pub parameters: [[f64;2];2],
    /// Unit normals from each bounded surface's own analytic derivatives.
    pub normals: [V;2],
    /// Unit tangent in the direction first_normal cross second_normal.
    pub tangent: V,
    pub incidence_error: f64,
    /// Sine of the angle between the two tangent planes, independent of units.
    pub transversality: f64,
}

#[derive(Clone,Copy,Debug)]
pub struct SurfaceSeamOptions {
    pub bounds: [[f64;2];2],
    pub parameter_scale: [f64;2],
    /// First tolerance: support incidence (length). Second: the section's units.
    pub residual_tolerance: [f64;2],
    /// A lower threshold for the sine between the two unit surface normals.
    pub min_transversality: f64,
    pub max_iterations: u32,
}

#[derive(Clone,Copy,Debug)]
pub struct SurfaceSeamIntersection {
    pub point: SurfaceSeamPoint,
    pub residuals: [f64;2],
    pub iterations: u32,
}

/// Operand one supplies the two search coordinates. Operand two owns a finite
/// projector. Both are immutable solved snapshots, not independently mutable
/// trial/checked copies. No generating motion is implied by a surface intersection.
#[derive(Clone,Debug)]
pub struct SurfaceSeam {
    pub name: String,
    first: RevolvedSurface,
    second: SurfaceProjector,
}

impl SurfaceSeam {
    pub fn named(sk: &Sketch,index: usize) -> Result<Self,String> {
        if super::kind(sk,index)? != super::SeamKind::Surfaces {
            return Err("a surface seam needs two analytic surface operands".into());
        }
        let s = &sk.seams[index];
        Ok(Self {name:s.name.clone(),first:RevolvedSurface::named(sk,s.first.i())?,
            second:SurfaceProjector::named(sk,s.second.i())?})
    }

    pub fn domain(&self) -> [[f64;2];2] { self.first.domain() }
    pub fn domains(&self) -> [[[f64;2];2];2] { [self.first.domain(),self.second.surface().domain()] }

    fn trial(&self,p: [f64;2]) -> Result<(SurfacePoint,SurfaceProjection),Error> {
        let a = self.first.at(p[0],p[1])?;
        Ok((a,self.second.project(a.position)?))
    }

    fn retain(&self,p: [f64;2],a: SurfacePoint,b: SurfaceProjection,
        incidence: f64,transverse: f64) -> Result<SurfaceSeamPoint,Error> {
        if b.incidence_error > incidence { return Err(Error::OutsideDomain); }
        let domain = self.second.surface().domain();
        let q = std::array::from_fn(|i| b.parameters[i].clamp(domain[i][0],domain[i][1]));
        let second = self.second.surface().at(q[0],q[1])?;
        let normals = [unit(cross(a.du,a.dv))?,unit(cross(second.du,second.dv))?];
        let tangent = cross(normals[0],normals[1]);
        let transversality = length(tangent);
        if !transversality.is_finite() || transversality <= transverse {
            return Err(Error::SingularIntersection);
        }
        Ok(SurfaceSeamPoint {position:a.position,parameters:[p,q],normals,
            tangent:unit(tangent)?,incidence_error:b.incidence_error,transversality})
    }

    /// A retained point must belong to both finite patches and pass the explicit
    /// normal-angle threshold. This local check is not a whole-curve certificate.
    pub fn evaluate(&self,p: [f64;2],incidence: f64,min_transversality: f64)
        -> Result<SurfaceSeamPoint,Error> {
        controls(incidence,min_transversality)?;
        let (a,b) = self.trial(p)?;
        self.retain(p,a,b,incidence,min_transversality)
    }

    /// Solve second-surface incidence and one section in the first surface's chart.
    /// The common bounded solver checks every residual and local rank, including a
    /// zero-residual seed. Retention then checks finite incidence and transversality.
    pub fn intersect(&self,section: impl Fn([f64;2],&SurfacePoint) -> f64,seed: [f64;2],
        options: SurfaceSeamOptions) -> Result<SurfaceSeamIntersection,Error> {
        controls(options.residual_tolerance[0],options.min_transversality)?;
        let domain = self.domain();
        if (0..2).any(|i| options.bounds[i][0] < domain[i][0] || options.bounds[i][1] > domain[i][1]) {
            return Err(Error::OutsideDomain);
        }
        let result = crate::intersection::solve(|p| {
            let p = [p[0],p[1]];
            let (a,b) = self.trial(p)?;
            Ok(((a,b),[b.signed_residual,section(p,&a),0.]))
        },[seed[0],seed[1],0.],IntersectionOptions {
            bounds:[options.bounds[0],options.bounds[1],[0.,0.]],
            parameter_scale:[options.parameter_scale[0],options.parameter_scale[1],1.],
            residual_tolerance:[options.residual_tolerance[0],options.residual_tolerance[1],1.],
            max_iterations:options.max_iterations,
        })?;
        let (a,b) = result.value;
        let point = self.retain([result.parameters[0],result.parameters[1]],a,b,
            options.residual_tolerance[0],options.min_transversality)?;
        Ok(SurfaceSeamIntersection {point,residuals:[result.residuals[0],result.residuals[1]],
            iterations:result.iterations})
    }
}
