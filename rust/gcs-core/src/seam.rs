//! Curves shared by generating faces or analytic surfaces, made of their parts.  A seam is
//! built once and its snapshot shared by the oriented uses face topology makes of it.
#[allow(unused_imports)]
use crate::fmath::Det;
mod boundary;
pub use boundary::{BoundarySeam,BoundarySeamTolerance};
mod surfaces;
pub use surfaces::{SurfaceSeam,SurfaceSeamPoint,SurfaceSeamIntersection,SurfaceSeamOptions};
use crate::{envelope::{self,Contact,Error,Intersection,IntersectionOptions,Motion},
    patch::EnvelopePatch};

fn distance(a: [f64;3],b: [f64;3]) -> f64 {
    (a[0]-b[0]).dhypot(a[1]-b[1]).dhypot(a[2]-b[2])
}

/// Explicit geometric tolerances for a solved seam snapshot. `position` and `axis`
/// are model lengths; `normal` is the Euclidean difference between unit normals,
/// allowing either orientation of the common tangent plane.
#[derive(Clone,Copy,Debug)]
pub struct SeamTolerance {
    pub position: f64,
    pub normal: f64,
    pub axis: f64,
}

/// Search controls for the two unknowns [v,roll]. The shared endpoint's u is
/// structural and cannot be freed by a caller. Both envelope equations use the
/// same normal-velocity tolerance; the section equation has its own units.
#[derive(Clone,Copy,Debug)]
pub struct SeamIntersectionOptions {
    pub bounds: [[f64;2];2],
    pub parameter_scale: [f64;2],
    pub normal_tolerance: f64,
    pub section_tolerance: f64,
    pub max_iterations: u32,
}

#[derive(Clone,Debug)]
pub struct EnvelopeSeam {
    pub name: String,
    faces: [EnvelopePatch;2],
    ends: [f64;2],
    domain: [[f64;2];3],
    tolerance: SeamTolerance,
}

impl SeamTolerance {
    /// Finite and nonnegative, the normal's below one.
    fn checked(&self) -> Result<(),String> {
        if [self.position,self.normal,self.axis].iter().any(|x| !x.is_finite() || *x < 0.) || self.normal >= 1. {
            return Err("a seam needs finite nonnegative tolerances and normal tolerance below one".into());
        }
        Ok(())
    }
}

impl EnvelopeSeam {
    /// Two generating faces meeting where their sources do (#212), made of their parts: `ends`
    /// each face's source edge's `u` at the vertex the two share, which the caller knows from the
    /// cutter's profile — checked coincident and tangent there, never found by proximity.
    pub fn new(name: String,faces: [&crate::patch::TrimmedPatch;2],ends: [f64;2],tolerance: SeamTolerance) -> Result<Self,String> {
        tolerance.checked()?;
        let [a,b] = faces.map(|f| f.envelope_patch().cloned());
        Self::between(name,[a?,b?],ends,tolerance)
    }

    fn between(name: String,faces: [EnvelopePatch;2],ends: [f64;2],tolerance: SeamTolerance) -> Result<Self,String> {
        let domains = faces.each_ref().map(|f| f.envelope().domain());
        let domain = [[ends[0],ends[0]],
            [domains[0][1][0].max(domains[1][1][0]),domains[0][1][1].min(domains[1][1][1])],
            [domains[0][2][0].max(domains[1][2][0]),domains[0][2][1].min(domains[1][2][1])]];
        if domain[1..].iter().any(|b| b[0] >= b[1]) {
            return Err("the generating faces need overlapping angular domains".into());
        }
        // Both endpoint frames undergo the same revolution and then the same rigid
        // generating motion. Their coincidence and tangent-plane angle are therefore
        // invariant in v and roll; one source-frame check establishes that relationship.
        let v = (domain[1][0]+domain[1][1])/2.;
        let contacts = [0,1].map(|i| faces[i].envelope().generating_surface().at(ends[i],v)
            .and_then(|s| envelope::contact(s,Motion::identity())));
        let [a,b] = contacts;
        let [a,b] = [a.map_err(|e| format!("invalid seam junction: {e:?}"))?,
            b.map_err(|e| format!("invalid seam junction: {e:?}"))?];
        Self::check_pair(a,b,tolerance).map_err(|_| format!(
            "`{}`: the source junction must be coincident and tangent; position error {:e} (limit {:e}), normal-plane error {:e} (limit {:e})",
            name,distance(a.position,b.position),tolerance.position,
            distance(a.normal,b.normal).min(distance(a.normal,b.normal.map(|x| -x))),tolerance.normal))?;
        Ok(Self {name,faces,ends,domain,tolerance})
    }

    fn check_pair(a: Contact,b: Contact,t: SeamTolerance) -> Result<(),Error> {
        if distance(a.position,b.position) > t.position
            || distance(a.normal,b.normal).min(distance(a.normal,b.normal.map(|x| -x))) > t.normal {
            return Err(Error::Degenerate);
        }
        Ok(())
    }

    /// The first envelope's original chart; its u coordinate is the shared endpoint.
    /// A local solve has only v and roll unknown. The implicit locus can still be
    /// singular; section intersections check rank. Orientation/face loops are separate.
    pub fn domain(&self) -> [[f64;2];3] { self.domain }

    /// The shared vertex's original u coordinate on each generating edge, in operand order.
    pub fn endpoint_parameters(&self) -> [f64;2] { self.ends }

    pub fn evaluate(&self,p: [f64;3],normal_tolerance: f64,trim_tolerance: f64)
        -> Result<Contact,Error> {
        Ok(self.contacts(p,normal_tolerance,trim_tolerance)?[0])
    }

    /// Retain both checked contacts when a downstream boundary names one specific
    /// face. Agreement within seam tolerance does not transfer finer incidence.
    pub(crate) fn contacts(&self,p: [f64;3],normal_tolerance: f64,trim_tolerance: f64)
        -> Result<[Contact;2],Error> {
        envelope::check_tolerance(normal_tolerance)?;
        envelope::check_tolerance(trim_tolerance)?;
        if !p.iter().all(|x| x.is_finite()) { return Err(Error::NonFinite); }
        if (0..3).any(|i| p[i] < self.domain[i][0] || p[i] > self.domain[i][1]) {
            return Err(Error::OutsideDomain);
        }
        let read = |i: usize| self.faces[i].at([self.ends[i],p[1],p[2]],
            normal_tolerance,trim_tolerance);
        let a = read(0)?; let b = read(1)?;
        Self::check_pair(a,b,self.tolerance)?;
        Ok([a,b])
    }

    /// Intersect the shared characteristic with one spatial section equation.
    /// Trial points need not satisfy material trims; the retained solution must satisfy both faces.
    /// The returned parameters are [first-source u,v,roll]; its residuals are the
    /// two envelope equations followed by the section equation.
    pub fn intersect(&self,section: impl Fn(&Contact) -> f64,seed: [f64;2],
        options: SeamIntersectionOptions,trim_tolerance: f64) -> Result<Intersection,Error> {
        envelope::check_tolerance(trim_tolerance)?;
        if (0..2).any(|i| options.bounds[i][0] < self.domain[i+1][0]
            || options.bounds[i][1] > self.domain[i+1][1]) { return Err(Error::OutsideDomain); }
        let first = self.faces[0].envelope();
        let second = self.faces[1].envelope();
        let mut result = envelope::intersect_evaluated(|p| first.evaluate(p),|p,c| [
            second.evaluate([self.ends[1],p[1],p[2]])
                .map(|c| c.normal_velocity).unwrap_or(f64::NAN),section(c)],
            [self.ends[0],seed[0],seed[1]],IntersectionOptions {
                bounds:[self.domain[0],options.bounds[0],options.bounds[1]],
                parameter_scale:[1.,options.parameter_scale[0],options.parameter_scale[1]],
                residual_tolerance:[options.normal_tolerance,options.normal_tolerance,
                    options.section_tolerance],max_iterations:options.max_iterations,
            })?;
        result.contact = self.evaluate(result.parameters,options.normal_tolerance,trim_tolerance)?;
        Ok(result)
    }
}
