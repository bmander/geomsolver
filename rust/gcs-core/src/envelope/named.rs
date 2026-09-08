//! A named implicit envelope of an analytic surface under a solved rigid-motion family.
use super::{Contact,Error,Intersection,IntersectionOptions};
use crate::{model::Sketch,motion::Family,solid::RevolvedSurface};

/// Solved snapshot. The envelope is the zero set of normal_velocity in [u,v,roll],
/// not every transformed source point. It may have multiple branches or singularities;
/// reading it does not certify global regularity, material orientation, or solid closure.
#[derive(Clone,Debug)]
pub struct GeneratedEnvelope {
    pub name: String,
    roll: [f64;2],
    surface: RevolvedSurface,
    motion: Family,
}

impl GeneratedEnvelope {
    pub(crate) fn generating_surface(&self) -> &RevolvedSurface { &self.surface }

    pub fn domain(&self) -> [[f64;2];3] {
        let [u,v] = self.surface.domain();
        [u,v,self.roll]
    }
    pub fn named(sk: &Sketch, index: usize) -> Result<Self,String> {
        let e = sk.envelopes.get(index).ok_or("no such envelope")?;
        if !e.roll.iter().all(|v| v.is_finite()) || e.roll[0] >= e.roll[1] {
            return Err("an envelope needs finite increasing roll bounds".into());
        }
        Ok(Self {name:e.name.clone(),roll:e.roll,
            surface:RevolvedSurface::named(sk,e.surface as usize)?,
            motion:Family::read(sk,e.motion as usize)?})
    }

    /// Evaluate the defining equation and moving contact data at a trial [u,v,roll].
    /// The first two parameters traverse the source patch; roll is radians. A nonzero
    /// normal_velocity is a point off the envelope, not a valid generated surface point.
    pub fn evaluate(&self, p: [f64;3]) -> Result<Contact,Error> {
        if !p.iter().all(|v| v.is_finite()) { return Err(Error::NonFinite); }
        if p[2] < self.roll[0] || p[2] > self.roll[1] { return Err(Error::OutsideDomain); }
        let s = self.surface.at(p[0],p[1])?;
        let m = self.motion.at(p[2]).map_err(|_| Error::NonFinite)?;
        super::contact(s,m)
    }

    /// Evaluate a retained point, requiring the envelope equation as well as the
    /// declared domain. Use `evaluate` only for trials that may be off the locus.
    pub fn at(&self,p: [f64;3],normal_tolerance: f64) -> Result<Contact,Error> {
        super::check_tolerance(normal_tolerance)?;
        let c = self.evaluate(p)?;
        if c.normal_velocity.abs() > normal_tolerance { return Err(Error::OutsideDomain); }
        Ok(c)
    }

    /// Intersect with two finite analytic boundary patches. Supporting-surface equations
    /// guide the solve; the final point must also lie on both finite patches within each
    /// section tolerance. This refuses roots on an undeclared continuation of a boundary.
    pub fn intersect_boundaries(&self, boundaries: [&crate::solid::SurfaceProjector;2],
        seed: [f64;3], options: IntersectionOptions) -> Result<Intersection,Error> {
        let result = self.intersect(|_,c| boundaries.map(|b|
            b.project(c.position).map(|p| p.signed_residual).unwrap_or(f64::NAN)),seed,options)?;
        for (i,b) in boundaries.into_iter().enumerate() {
            let p = b.project(result.contact.position)?;
            if p.incidence_error > options.residual_tolerance[i+1] { return Err(Error::OutsideDomain); }
        }
        Ok(result)
    }

    /// Solve the envelope equation and two section equations using the shared DogLeg
    /// solver. Search bounds must stay inside the declared domain. Neither the seed nor
    /// a converged local intersection certifies uniqueness or admissible material sides.
    pub fn intersect(&self, section: impl Fn([f64;3],&Contact) -> [f64;2],
        seed: [f64;3], options: IntersectionOptions) -> Result<Intersection,Error> {
        let domain = self.domain();
        if (0..3).any(|i| options.bounds[i][0] < domain[i][0]
            || options.bounds[i][1] > domain[i][1]) { return Err(Error::OutsideDomain); }
        super::intersect_evaluated(|p| self.evaluate(p),section,seed,options)
    }
}
