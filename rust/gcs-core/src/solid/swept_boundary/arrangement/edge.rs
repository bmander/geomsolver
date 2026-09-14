//! Isolate a zero of an incident face's normal velocity along a declared edge.
//! Strict endpoint signs and a nonzero derivative establish one root for EVERY
//! roll in the requested interval. This is contact evidence, not visibility.
use crate::{interval::{Interval as I,Error as Arithmetic},solid::{SweepContacts,ToolFace,EdgeChart}};

#[derive(Clone,Copy,Debug)]
pub struct Options { pub parameter_width: f64, pub max_steps: usize }
#[derive(Clone,Debug,PartialEq)]
pub enum Error { InvalidOptions, MissingEdge, MissingFace, NotIncident, UnsupportedChart,
    Arithmetic(Arithmetic), Surface(crate::envelope::Error), SingularSource, Unbracketed, NonMonotone }
impl From<Arithmetic> for Error { fn from(e: Arithmetic) -> Self { Self::Arithmetic(e) } }

#[derive(Clone,Debug)]
pub struct Root {
    edge: usize,
    face: usize,
    roll: I,
    domain: I,
    endpoint_values: [I;2],
    derivative: I,
    enclosure: I,
    steps: usize,
    resolved: bool,
}
impl Root {
    pub fn edge(&self) -> usize { self.edge }
    pub fn face(&self) -> usize { self.face }
    pub fn roll(&self) -> I { self.roll }
    pub fn domain(&self) -> I { self.domain }
    pub fn endpoint_values(&self) -> [I;2] { self.endpoint_values }
    pub fn derivative(&self) -> I { self.derivative }
    pub fn enclosure(&self) -> I { self.enclosure }
    pub fn steps(&self) -> usize { self.steps }
    /// The root is isolated even when refinement stops before the requested
    /// width. The retained enclosure may then be too wide for construction.
    pub fn resolved(&self) -> bool { self.resolved }
}

/// Uniform edge-coordinate enclosure over a whole roll interval. Currently
/// supports fixed-u/fixed-v edges of revolved faces. Other chart families
/// explicitly refuse; numerical edge samples are not interval evaluators.
pub fn isolate(sweep: &SweepContacts,edge: usize,face: usize,domain: I,roll: I,options: Options) -> Result<Root,Error> {
    let [lo,hi] = domain.bounds(); let [from,to] = sweep.domain(); let time = roll.bounds();
    if !(options.parameter_width > 0. && options.parameter_width.is_finite()) || options.max_steps > 128
        || lo < 0. || hi > 1. || lo >= hi || time[0] < from || time[1] > to { return Err(Error::InvalidOptions); }
    let e = sweep.edges().get(edge).ok_or(Error::MissingEdge)?;
    let consumer = e.faces.iter().position(|&f| f == face).ok_or(Error::NotIncident)?;
    let f = sweep.faces().get(face).ok_or(Error::MissingFace)?;
    let ToolFace::Revolved(surface) = f else { return Err(Error::UnsupportedChart); };
    let chart = e.charts[consumer];
    let value = |t: I| -> Result<(I,I),Error> {
        let (u,v,axis,scale) = match chart {
            EdgeChart::FixedU(u) => {
                let d = surface.domain()[1]; let scale = I::point(d[1])?.sub(I::point(d[0])?)?;
                (I::point(u)?,I::point(d[0])?.add(t.mul(scale)?)?,1,scale)
            }
            EdgeChart::FixedV(v) => (t,I::point(v)?,0,I::ONE),
            _ => return Err(Error::UnsupportedChart),
        };
        // Preserve the full interval after outward-rounded affine mapping,
        // including periodic roundoff at the native seam. Partial charts refuse.
        let source = surface.angular_chart(v.bounds()).map_err(Error::Surface)?;
        let b = source.bounds(u,v)?;
        // A vanishing source normal makes G=0 vacuous at a pole. Refuse unless
        // the entire source interval is regular, independently of G's slope.
        let area2 = b.normal.into_iter().try_fold(I::ZERO,|s,n| s.add(n.square()?))?;
        if area2.bounds()[0] <= 0. { return Err(Error::SingularSource); }
        let (n,m) = if axis == 0 { (b.normal_du,b.moment_du) } else { (b.normal_dv,b.moment_dv) };
        let g = sweep.motion().normal_velocity_moment_bounds(b.normal,b.moment)?.at(roll)?;
        let d = sweep.motion().normal_velocity_moment_bounds(n,m)?.at(roll)?.mul(scale)?;
        Ok((g,d))
    };
    let ends = [value(I::point(lo)?)?.0,value(I::point(hi)?)?.0];
    let derivative = value(domain)?.1;
    let increasing = ends[0].bounds()[1] < 0. && ends[1].bounds()[0] > 0.;
    let decreasing = ends[0].bounds()[0] > 0. && ends[1].bounds()[1] < 0.;
    if !increasing && !decreasing { return Err(Error::Unbracketed); }
    if derivative.contains(0.) || (derivative.bounds()[0] > 0.) != increasing { return Err(Error::NonMonotone); }
    let mut root = Root {edge,face,roll,domain,endpoint_values:ends,derivative,enclosure:domain,steps:0,resolved:false};
    for _ in 0..options.max_steps {
        let [a,b] = root.enclosure.bounds();
        if I::point(b)?.sub(I::point(a)?)?.bounds()[1] <= options.parameter_width { break; }
        let mid = a*0.5+b*0.5;
        if mid == a || mid == b { break; }
        let g = value(I::point(mid)?)?.0;
        let derivative = value(root.enclosure)?.1;
        // The original sign bracket established existence/uniqueness. Each
        // interval Newton contraction retains that same root for all rolls.
        let next = I::point(mid)?.sub(g.div(derivative)?)?.bounds();
        let next = I::new(a.max(next[0]),b.min(next[1]))?;
        root.steps += 1;
        if next == root.enclosure { break; }
        root.enclosure = next;
    }
    let [a,b] = root.enclosure.bounds();
    root.resolved = I::point(b)?.sub(I::point(a)?)?.bounds()[1] <= options.parameter_width;
    Ok(root)
}
