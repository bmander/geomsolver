//! Local trim-curve construction on original source charts. These are numerical
//! intersection candidates, not a complete visibility arrangement or accepted
//! boundary. Coincident/tangent supports and exhausted searches remain explicit.
use crate::{envelope,interval::{Interval as I,minimum},solid::{SweepContacts,SweptField,
    sweep_source::{self,Chart,SourcePoint}},space::{cross,distance,norm,normalised,sub}};
pub mod edge;
type V3 = [f64;3];
type UV = [f64;2];

#[derive(Clone,Copy,Debug)]
pub struct Options {
    /// Maximum separation of the two original source evaluations, in model units.
    pub agreement: f64,
    pub contact_tolerance: f64,
    /// Refuse intersections whose sampled normals are almost parallel.
    pub minimum_sine: f64,
    pub max_iterations: u32,
}
impl Default for Options {
    fn default() -> Self { Self {agreement:1e-11,contact_tolerance:1e-9,minimum_sine:1e-6,max_iterations:80} }
}
#[derive(Clone,Debug,PartialEq)]
pub enum Error {
    InvalidOptions,
    Source(sweep_source::Error),
    Solve(envelope::Error),
    NonTransverse,
    Budget,
    Resolution,
}

/// One local intersection branch, sought within explicit native-coordinate
/// boxes on two consumers of the same immutable source/motion snapshot. Fixing
/// one of [a.u,a.v,b.u,b.v] leaves three geometric equations in three unknowns.
/// The box and the local root index do not certify uniqueness or continuation.
pub struct Pair<'a> {
    sweep: &'a SweepContacts,
    charts: [Chart;2],
    bounds: [[f64;2];4],
    coordinate: usize,
    options: Options,
}
#[derive(Clone,Debug)]
pub struct Point {
    pub coordinate: f64,
    pub parameters: [UV;2],
    pub source: [SourcePoint;2],
    pub roll: [f64;2],
    /// Keep both evaluations. Do not move either source onto a spatial average.
    pub positions: [V3;2],
    pub iterations: u32,
}
impl<'a> Pair<'a> {
    pub fn new(sweep: &'a SweepContacts,charts: [Chart;2],bounds: [[f64;2];4],coordinate: usize,options: Options)
        -> Result<Self,Error> {
        if coordinate >= 4 || bounds.iter().any(|b| !b.iter().all(|x| x.is_finite()) || b[0] >= b[1]
                || !(b[1]-b[0]).is_finite())
            || ![options.agreement,options.contact_tolerance,options.minimum_sine].iter().all(|x| x.is_finite() && *x > 0.)
            || options.minimum_sine > 1. || options.max_iterations == 0 || options.max_iterations > 10000 {
            return Err(Error::InvalidOptions);
        }
        Ok(Self {sweep,charts,bounds,coordinate,options})
    }
    pub fn charts(&self) -> [Chart;2] { self.charts }
    pub fn bounds(&self) -> [[f64;2];4] { self.bounds }
    pub fn coordinate(&self) -> usize { self.coordinate }
    fn evaluate(&self,p: [f64;4]) -> Result<Point,Error> {
        if p.iter().enumerate().any(|(i,x)| !x.is_finite() || !(self.bounds[i][0]..=self.bounds[i][1]).contains(x)) {
            return Err(Error::Source(sweep_source::Error::OutsideDomain));
        }
        let parameters = [[p[0],p[1]],[p[2],p[3]]];
        let a = self.charts[0].at(self.sweep,parameters[0],self.options.contact_tolerance).map_err(Error::Source)?;
        let b = self.charts[1].at(self.sweep,parameters[1],self.options.contact_tolerance).map_err(Error::Source)?;
        if !a.2.position.iter().chain(&b.2.position).all(|x| x.is_finite()) { return Err(Error::Solve(envelope::Error::NonFinite)); }
        Ok(Point {coordinate:p[self.coordinate],parameters,source:[a.0,b.0],roll:[a.1,b.1],
            positions:[a.2.position,b.2.position],iterations:0})
    }
    fn normal(&self,consumer: usize,p: UV) -> Result<V3,Error> {
        let mut tangents = [[0.;3];2];
        for k in 0..2 {
            let b = self.bounds[2*consumer+k]; let h = (b[1]-b[0])*1e-5;
            let (mut lo,mut hi) = (p,p); lo[k] = (p[k]-h).max(b[0]); hi[k] = (p[k]+h).min(b[1]);
            if lo[k] == hi[k] { return Err(Error::NonTransverse); }
            let at = |uv| self.charts[consumer].at(self.sweep,uv,self.options.contact_tolerance)
                .map(|x| x.2.position).map_err(Error::Source);
            tangents[k] = sub(at(hi)?,at(lo)?);
        }
        normalised(cross(tangents[0],tangents[1])).filter(|n| n.iter().all(|x| x.is_finite()))
            .ok_or(Error::NonTransverse)
    }
    /// Solve the original position equality, rechecking incidence and sampled
    /// transversality even for an exact seed. Uses the shared bounded DogLeg
    /// adapter, not a second Newton loop or a nearest mesh-point operation.
    pub fn at(&self,coordinate: f64,seed: [UV;2]) -> Result<Point,Error> {
        let mut p = [seed[0][0],seed[0][1],seed[1][0],seed[1][1]];
        p[self.coordinate] = coordinate;
        self.evaluate(p)?;
        let axes: Vec<_> = (0..4).filter(|&i| i != self.coordinate).collect();
        let options = envelope::IntersectionOptions {
            bounds:std::array::from_fn(|k| self.bounds[axes[k]]),
            parameter_scale:std::array::from_fn(|k| self.bounds[axes[k]][1]-self.bounds[axes[k]][0]),
            residual_tolerance:[self.options.agreement/3_f64.sqrt();3],max_iterations:self.options.max_iterations,
        };
        let result = crate::intersection::solve(|q| {
            let mut p = p; for k in 0..3 { p[axes[k]] = q[k]; }
            let point = self.evaluate(p).map_err(|e| match e {
                Error::Source(sweep_source::Error::Surface(e)) | Error::Solve(e) => e,
                _ => envelope::Error::OutsideDomain,
            })?;
            let residual = sub(point.positions[0],point.positions[1]);
            Ok((point,residual))
        },std::array::from_fn(|k| p[axes[k]]),options).map_err(Error::Solve)?;
        let mut point = result.value; point.iterations = result.iterations;
        if distance(point.positions[0],point.positions[1]) > self.options.agreement {
            return Err(Error::Solve(envelope::Error::NotConverged));
        }
        let normals = [self.normal(0,point.parameters[0])?,self.normal(1,point.parameters[1])?];
        if !(norm(cross(normals[0],normals[1])) >= self.options.minimum_sine) { return Err(Error::NonTransverse); }
        Ok(point)
    }

    /// Adaptive sampling of one regular branch. The completed and unresolved
    /// intervals partition the requested range; no failed span is bridged.
    /// Midpoint chord and parameter-step tests are sampled diagnostics, not an
    /// interval uniqueness, visibility, or whole-curve approximation proof.
    pub fn trace(&self,range: [f64;2],seed: [UV;2],options: TraceOptions) -> Result<Trace,Error> {
        let bounds = self.bounds[self.coordinate];
        if !range.iter().all(|x| x.is_finite()) || range[0] >= range[1] || range[0] < bounds[0] || range[1] > bounds[1]
            || !(options.sagitta > 0. && options.sagitta.is_finite())
            || !(options.parameter_step > 0. && options.parameter_step <= 1.) || options.max_depth > 48 {
            return Err(Error::InvalidOptions);
        }
        let mut out = Trace::default();
        let mut solve = |at,seed| {
            if out.evaluations >= options.max_solves { return Err(Error::Budget); }
            out.evaluations += 1; self.at(at,seed)
        };
        let a = match solve(range[0],seed) {
            Ok(p) => p,Err(error) => { out.unresolved.push(Unresolved {range,error}); return Ok(out); }
        };
        let b = match solve(range[1],a.parameters) {
            Ok(p) => p,Err(error) => { out.points.push(a); out.unresolved.push(Unresolved {range,error}); return Ok(out); }
        };
        out.points.extend([a,b]);
        let mut pending = vec![(0usize,1usize,0u8)];
        while let Some((a,b,depth)) = pending.pop() {
            let range = [out.points[a].coordinate,out.points[b].coordinate];
            let mid = range[0]*0.5+range[1]*0.5;
            if mid == range[0] || mid == range[1] {
                out.unresolved.push(Unresolved {range,error:Error::Resolution}); continue;
            }
            let seed = std::array::from_fn(|i| std::array::from_fn(|k|
                0.5*(out.points[a].parameters[i][k]+out.points[b].parameters[i][k])));
            let point = match solve(mid,seed) {
                Ok(p) => p,Err(error) => { out.unresolved.push(Unresolved {range,error}); continue; }
            };
            let m = out.points.len(); out.points.push(point);
            let chord_error = (0..2).map(|i| {
                let chord = std::array::from_fn(|k| 0.5*(out.points[a].positions[i][k]+out.points[b].positions[i][k]));
                distance(chord,out.points[m].positions[i])
            }).fold(0.,f64::max);
            let mut step = 0_f64;
            for (a,b) in [(a,m),(m,b)] { for i in 0..2 { for k in 0..2 {
                let width = self.bounds[2*i+k][1]-self.bounds[2*i+k][0];
                step = step.max((out.points[a].parameters[i][k]-out.points[b].parameters[i][k]).abs()/width);
            } } }
            if chord_error <= options.sagitta && step <= options.parameter_step {
                out.segments.push([a,m,b]); out.sampled_error = out.sampled_error.max(chord_error);
            } else if depth >= options.max_depth {
                out.unresolved.push(Unresolved {range,error:Error::Resolution});
            } else { pending.push((m,b,depth+1)); pending.push((a,m,depth+1)); }
        }
        out.segments.sort_by(|a,b| out.points[a[0]].coordinate.total_cmp(&out.points[b[0]].coordinate));
        out.unresolved.sort_by(|a,b| a.range[0].total_cmp(&b.range[0]));
        Ok(out)
    }
}
#[derive(Clone,Copy,Debug)]
pub struct TraceOptions {
    pub sagitta: f64,
    /// Maximum sampled step as a fraction of each native parameter box width.
    pub parameter_step: f64,
    pub max_depth: u8,
    pub max_solves: usize,
}
#[derive(Clone,Debug)]
pub struct Unresolved { pub range: [f64;2], pub error: Error }
#[derive(Clone,Debug,Default)]
pub struct Trace {
    pub points: Vec<Point>,
    /// Indices of [low,mid,high] in points, with the same two consumers throughout.
    pub segments: Vec<[usize;3]>,
    pub unresolved: Vec<Unresolved>,
    pub evaluations: usize,
    pub sampled_error: f64,
}

/// A strict whole-box interior witness at one actual generating pose. A source
/// chart can be locally in contact yet hidden by this other time in the sweep.
#[derive(Clone,Copy,Debug)]
pub struct Hidden {
    point: [I;3],
    roll: f64,
    value: I,
}
impl Hidden {
    pub fn point(&self) -> [I;3] { self.point }
    pub fn roll(&self) -> f64 { self.roll }
    pub fn value(&self) -> I { self.value }
}
#[derive(Clone,Copy,Debug)]
pub struct HidingReport {
    /// Search work only; confirming the witness uses one additional source bound.
    pub minimum: minimum::Minimum,
    pub witness: Option<Hidden>,
}
/// Find a sufficient hiding witness over the complete roll domain, using the
/// existing bounded continuous minimizer. No witness means unresolved, NEVER
/// exposure. Keep the enclosure/status/budget and recheck the actual witness.
pub fn hiding(sweep: &SweepContacts,point: [I;3],options: minimum::Options) -> Result<HidingReport,String> {
    let domain = sweep.domain(); let domain = I::new(domain[0],domain[1]).map_err(|e| format!("{e:?}"))?;
    let field = SweptField::new(sweep.source_material().clone(),sweep.motion().clone(),domain);
    let result = field.evaluator(256).bounds_outside(point,I::ZERO,options).map_err(|e| format!("{e:?}"))?;
    let roll = result.witness;
    let pose = sweep.motion().bounds(I::point(roll).map_err(|e| format!("{e:?}"))?).map_err(|e| format!("{e:?}"))?;
    let value = sweep.source_material().bounds(pose.inverse_point(point).map_err(|e| format!("{e:?}"))?).map_err(|e| format!("{e:?}"))?;
    Ok(HidingReport {minimum:result,witness:(value.bounds()[1] < 0.).then_some(Hidden {point,roll,value})})
}
