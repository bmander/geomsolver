//! Finite seam extents between declared corner identities. A spatial line supplies
//! a geometric slicing coordinate; each requested slice still needs a regular local
//! intersection. This is not a global branch-uniqueness or curve-error certificate.
use crate::{envelope::{check_iterations,check_tolerance,Contact,Error,Intersection,IntersectionOptions},
    model::{EdgeE,Sketch},seam::{BoundarySeam,BoundarySeamTolerance,EnvelopeSeam,
        SeamIntersectionOptions,SeamTolerance},vertex};

pub(crate) fn validate(sk: &Sketch,e: &EdgeE) -> Result<(),String> {
    let seam = sk.seams.get(e.seam as usize).ok_or("no such edge seam")?;
    crate::seam::validate(sk,[seam.first,seam.second])?;
    if e.start == e.end { return Err("an edge needs two distinct named vertices".into()); }
    for i in [e.start,e.end] {
        vertex::seam_chart(sk,i as usize,e.seam as usize)?;
    }
    sk.lines.get(e.along as usize).ok_or("an edge needs an along line")?;
    Ok(())
}

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

#[derive(Clone,Debug)]
enum Source { Boundary(BoundarySeam), Junction(EnvelopeSeam) }

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
/// axis. The fraction [0,1] follows declaration order, even with a reversed axis.
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

impl SpatialEdge {
    /// Endpoint witnesses use each vertex's canonical chart. They are rechecked
    /// against the current sketch and converted by declared face identity.
    pub fn named(sk: &Sketch,index: usize,parameters: [[f64;3];2],t: EdgeTolerance)
        -> Result<Self,String> {
        for v in [t.junction.position,t.junction.normal,t.junction.axis,
            t.point.normal_velocity,t.point.incidence,t.point.trim] {
            check_tolerance(v).map_err(|_| "an edge needs finite nonnegative tolerances")?;
        }
        if t.junction.normal >= 1. || t.point.normal_velocity == 0. || t.point.incidence == 0. {
            return Err("an edge needs positive solve tolerances and junction normal tolerance below one".into());
        }
        let e = sk.edges.get(index).ok_or("no such edge")?;
        validate(sk,e)?;
        let axis = &sk.lines[e.along as usize];
        for p in [axis.p1,axis.p2] { sk.points.get(p as usize).ok_or("no such edge axis point")?; }
        let d = delta(sk.world_point(axis.p2 as usize),sk.world_point(axis.p1 as usize));
        let n = length(d);
        if !n.is_finite() || n == 0. { return Err("an edge needs a finite nondegenerate along line".into()); }
        let direction = d.map(|x| x/n);
        let source = match crate::seam::kind(sk,e.seam as usize)? {
            crate::seam::SeamKind::Boundary =>
                Source::Boundary(BoundarySeam::named(sk,e.seam as usize,t.junction.axis)?),
            crate::seam::SeamKind::Generating =>
                Source::Junction(EnvelopeSeam::named(sk,e.seam as usize,t.junction)?),
            crate::seam::SeamKind::Surfaces =>
                return Err("surface seam edges need surface-chart vertices".into()),
        };
        let vertices = [e.start as usize,e.end as usize];
        let read = |i: usize| -> Result<EdgePoint,String> {
            let (parameters,position) = vertex::on_seam(sk,vertices[i],e.seam as usize,
                parameters[i],t.junction,t.point)?;
            let c = source.evaluate(parameters,t.point).map_err(|e| format!("invalid edge endpoint: {e:?}"))?;
            if length(delta(c.position,position)) > t.junction.position {
                return Err("the edge and its named vertex disagree in position".into());
            }
            Ok(EdgePoint {parameters,position})
        };
        let endpoints = [read(0)?,read(1)?];
        let span = dot(delta(endpoints[1].position,endpoints[0].position),direction);
        if !span.is_finite() || span.abs()/2. <= t.point.incidence {
            return Err("edge endpoints need distinct positions along the slicing line".into());
        }
        Ok(Self {name:e.name.clone(),vertices,endpoints,source,direction,span,tolerance:t})
    }

    pub fn vertices(&self) -> [usize;2] { self.vertices }
    pub fn endpoints(&self) -> [EdgePoint;2] { self.endpoints }

    /// Linear interpolation in signed distance along the named line. The core
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
