//! Finite seam extents between two corner identities, made of their parts. A spatial direction
//! supplies a geometric slicing coordinate; each requested slice still needs a regular local
//! intersection. This is not a global branch-uniqueness or curve-error certificate.
use crate::{envelope::{check_iterations,check_tolerance,Contact,Error,Intersection,IntersectionOptions},
    seam::{BoundarySeam,BoundarySeamTolerance,EnvelopeSeam,SeamIntersectionOptions,SeamTolerance}};

#[derive(Clone,Copy,Debug)]
pub struct EdgeTolerance {
    pub junction: SeamTolerance,
    /// Positive normal-velocity and incidence tolerances are required for solves.
    /// Incidence also controls the spatial slicing equation; trim may be zero.
    pub point: BoundarySeamTolerance,
}

#[derive(Clone,Copy,Debug)]
pub struct EdgePoint {
    pub position: [f64;3],
    /// Original chart of this edge's seam, not necessarily its vertex's chart.
    pub parameters: [f64;3],
}

/// The seam an edge is a stretch of: a generating face against a finite boundary, or a junction.
#[derive(Clone,Debug)]
pub enum Source { Boundary(BoundarySeam), Junction(EnvelopeSeam) }

impl Source {
    fn evaluate(&self,p: [f64;3],t: BoundarySeamTolerance) -> Result<Contact,Error> {
        match self {
            Self::Boundary(s) => s.evaluate(p,t),
            Self::Junction(s) => s.evaluate(p,t.normal_velocity,t.trim),
        }
    }

    fn intersect(&self,section: impl Fn(&Contact) -> f64,seed: [f64;3],
        t: BoundarySeamTolerance,max_iterations: u32) -> Result<Intersection,Error> {
        match self {
            Self::Boundary(s) => s.intersect(section,seed,IntersectionOptions {
                bounds:s.domain(),parameter_scale:[1.;3],
                residual_tolerance:[t.normal_velocity,t.incidence,t.incidence],max_iterations,
            },t.trim),
            Self::Junction(s) => {
                let [_,v,roll] = s.domain();
                s.intersect(section,[seed[1],seed[2]],SeamIntersectionOptions {
                    bounds:[v,roll],parameter_scale:[1.;2],normal_tolerance:t.normal_velocity,
                    section_tolerance:t.incidence,max_iterations,
                },t.trim)
            }
        }
    }
}

/// One owned seam snapshot, two checked endpoint witnesses and a solved slicing
/// axis. The fraction [0,1] runs from the first corner to the second, even with a reversed axis.
/// Construction checks endpoint incidence, not regularity of every interior slice.
#[derive(Clone,Debug)]
pub struct SpatialEdge {
    pub name: String,
    vertices: [usize;2],
    endpoints: [EdgePoint;2],
    source: Source,
    direction: [f64;3],
    span: f64,
    tolerance: EdgeTolerance,
}

use crate::space::{dot,sub as delta,length};

/// An edge's tolerances, finite and nonnegative, its solves' positive, the junction's normal
/// tolerance below one.
fn checked(t: EdgeTolerance) -> Result<(),String> {
    for v in [t.junction.position,t.junction.normal,t.junction.axis,
        t.point.normal_velocity,t.point.incidence,t.point.trim] {
        check_tolerance(v).map_err(|_| "an edge needs finite nonnegative tolerances")?;
    }
    if t.junction.normal >= 1. || t.point.normal_velocity == 0. || t.point.incidence == 0. {
        return Err("an edge needs positive solve tolerances and junction normal tolerance below one".into());
    }
    Ok(())
}

/// The slicing line's unit direction.
fn along(d: [f64;3]) -> Result<[f64;3],String> {
    let n = length(d);
    if !n.is_finite() || n == 0. { return Err("an edge needs a finite nondegenerate along line".into()); }
    Ok(d.map(|x| x/n))
}

impl SpatialEdge {
    /// A stretch of `source` between two corners, made of its parts (#212): `vertices` the
    /// corners' identities (a caller's numbering), `endpoints` each corner in the seam's own chart
    /// (a junction's second face's `u` already put in where the corner is read on that face), and
    /// the slicing line's `direction`. Each endpoint is checked on the seam.
    pub fn new(name: String,source: Source,vertices: [usize;2],endpoints: [EdgePoint;2],direction: [f64;3],
        t: EdgeTolerance) -> Result<Self,String> {
        checked(t)?;
        if vertices[0] == vertices[1] { return Err("an edge needs two distinct corners".into()); }
        let direction = along(direction)?;
        for p in &endpoints {
            let c = source.evaluate(p.parameters,t.point).map_err(|e| format!("invalid edge endpoint: {e:?}"))?;
            if length(delta(c.position,p.position)) > t.junction.position {
                return Err("the edge and its corner disagree in position".into());
            }
        }
        let span = dot(delta(endpoints[1].position,endpoints[0].position),direction);
        if !span.is_finite() || span.abs()/2. <= t.point.incidence {
            return Err("edge endpoints need distinct positions along the slicing line".into());
        }
        Ok(Self {name,vertices,endpoints,source,direction,span,tolerance:t})
    }

    pub fn vertices(&self) -> [usize;2] { self.vertices }
    pub fn endpoints(&self) -> [EdgePoint;2] { self.endpoints }

    /// Linear interpolation in signed distance along the slicing line. The core
    /// solves the original seam equations at each slice and checks local rank,
    /// finite incidence and material trims in the interior. The endpoint's full
    /// vertex constraints were already checked; do not replace them by a slice
    /// equation that can be tangent there. No spatial polyline is interpolated.
    pub fn sample(&self,fraction: f64,max_iterations: u32) -> Result<EdgePoint,Error> {
        check_iterations(max_iterations)?;
        if !fraction.is_finite() { return Err(Error::NonFinite); }
        if !(0. ..=1.).contains(&fraction) { return Err(Error::OutsideDomain); }
        let [a,b] = self.endpoints;
        if fraction == 0. { return Ok(a); }
        if fraction == 1. { return Ok(b); }
        let seed = std::array::from_fn(|i| (1.-fraction)*a.parameters[i]+fraction*b.parameters[i]);
        let p = self.source.intersect(|c| dot(delta(c.position,a.position),self.direction)-fraction*self.span,
            seed,self.tolerance.point,max_iterations)?;
        Ok(EdgePoint {parameters:p.parameters,position:p.contact.position})
    }
}
