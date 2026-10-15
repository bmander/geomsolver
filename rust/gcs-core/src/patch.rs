//! A spatial surface region selected by closed material-side conditions, made of its parts: a
//! source surface or envelope and the regions it is trimmed to.
use crate::{envelope::{check_tolerance,Contact,Error,GeneratedEnvelope,Intersection,
        IntersectionOptions,SurfacePoint},
    solid::{RegionLocation,RevolvedRegion,RevolvedSurface,SurfaceProjector}};

#[derive(Clone,Debug)]
enum Source {
    Surface { surface: RevolvedSurface, trims: MaterialTrims },
    Envelope(EnvelopePatch),
}

/// Material tests do not establish source incidence. Keep this internal so every
/// retained-point API must check its surface/envelope before applying the trims.
#[derive(Clone,Debug,Default)]
struct MaterialTrims {
    inside: Vec<RevolvedRegion>,
    outside: Vec<RevolvedRegion>,
}

impl MaterialTrims {
    fn check(&self,p: [f64;3],tolerance: f64) -> Result<(),Error> {
        // Even an untrimmed envelope must reject invalid numerical controls.
        check_tolerance(tolerance)?;
        for (regions,excluded) in [(&self.inside,RegionLocation::Outside),
            (&self.outside,RegionLocation::Inside)] {
            for region in regions {
                if region.classify(p,tolerance)?.location == excluded {
                    return Err(Error::OutsideDomain);
                }
            }
        }
        Ok(())
    }
}

/// One envelope and its material conditions, used for both patches and seam operands.
/// There is no separately stored untrimmed copy that can diverge from this source.
#[derive(Clone,Debug)]
pub(crate) struct EnvelopePatch {
    source: GeneratedEnvelope,
    trims: MaterialTrims,
}

impl EnvelopePatch {
    pub(crate) fn envelope(&self) -> &GeneratedEnvelope { &self.source }

    pub(crate) fn at(&self,p: [f64;3],normal_tolerance: f64,trim_tolerance: f64)
        -> Result<Contact,Error> {
        check_tolerance(trim_tolerance)?;
        let c = self.source.at(p,normal_tolerance)?;
        self.trims.check(c.position,trim_tolerance)?;
        Ok(c)
    }

    fn retain(&self,result: Intersection,trim_tolerance: f64) -> Result<Intersection,Error> {
        // Intersection already checks all original equations and local rank.
        self.trims.check(result.contact.position,trim_tolerance)?;
        Ok(result)
    }
}

/// Solved analytic snapshot. Every inside/outside condition must hold, including its
/// boundary. This selects a set of source points; it does not assert connectedness,
/// regularity, outward orientation, or a single envelope branch. A closed solid still
/// needs oriented faces and checked common boundaries.
#[derive(Clone,Debug)]
pub struct TrimmedPatch {
    pub name: String,
    source: Source,
}

impl TrimmedPatch {
    /// An envelope's points within every region of `inside` and outside every one of `outside`,
    /// made of its parts rather than read off a declaration (#212): a body's cut face, its trims
    /// the body's own material.
    pub fn of_envelope(name: String,source: GeneratedEnvelope,inside: Vec<RevolvedRegion>,outside: Vec<RevolvedRegion>) -> Self {
        Self {name,source:Source::Envelope(EnvelopePatch {source,trims:MaterialTrims {inside,outside}})}
    }

    /// A revolved surface's points within every region of `inside` and outside every one of
    /// `outside`.
    pub fn of_surface(name: String,surface: RevolvedSurface,inside: Vec<RevolvedRegion>,outside: Vec<RevolvedRegion>) -> Self {
        Self {name,source:Source::Surface {surface,trims:MaterialTrims {inside,outside}}}
    }

    pub(crate) fn envelope_patch(&self) -> Result<&EnvelopePatch,String> {
        match &self.source { Source::Envelope(e) => Ok(e),Source::Surface {..} => Err("a generating face needs an envelope source".into()) }
    }

    pub fn surface_at(&self,u: f64,v: f64,trim_tolerance: f64) -> Result<SurfacePoint,Error> {
        check_tolerance(trim_tolerance)?;
        let Source::Surface {surface,trims} = &self.source else { return Err(Error::InvalidOptions); };
        let p = surface.at(u,v)?;
        trims.check(p.position,trim_tolerance)?;
        Ok(p)
    }

    /// A point must satisfy both the envelope equation and every trim. Off-locus
    /// trials belong to GeneratedEnvelope::evaluate, not to this bounded surface.
    pub fn envelope_at(&self,p: [f64;3],normal_tolerance: f64,trim_tolerance: f64)
        -> Result<Contact,Error> {
        let Source::Envelope(s) = &self.source else { return Err(Error::InvalidOptions); };
        s.at(p,normal_tolerance,trim_tolerance)
    }

    /// Solve on the untrimmed source, then require the result to belong to this
    /// region. Trial points may be outside it; a seed does not designate material.
    pub fn intersect(&self,section: impl Fn([f64;3],&Contact) -> [f64;2],
        seed: [f64;3],options: IntersectionOptions,trim_tolerance: f64)
        -> Result<Intersection,Error> {
        check_tolerance(trim_tolerance)?;
        let Source::Envelope(s) = &self.source else { return Err(Error::InvalidOptions); };
        s.retain(s.source.intersect(section,seed,options)?,trim_tolerance)
    }

    pub fn intersect_boundaries(&self,boundaries: [&SurfaceProjector;2],seed: [f64;3],
        options: IntersectionOptions,trim_tolerance: f64) -> Result<Intersection,Error> {
        check_tolerance(trim_tolerance)?;
        let Source::Envelope(s) = &self.source else { return Err(Error::InvalidOptions); };
        s.retain(s.source.intersect_boundaries(boundaries,seed,options)?,trim_tolerance)
    }
}
