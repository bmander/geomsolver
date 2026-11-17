//! **A frame carried along a curve** (`Along`, K2): the rigid motion that takes what is drawn at
//! the curve's start along it, its frame square to the curve's tangent turning only as the tangent
//! does — the rotation-minimising frame, `ω = T × T′` — so along a curve in a plane its axis
//! square to the plane stays put, and a tool drawn upright at the start is carried upright, its
//! first axis along the tangent wherever it was along it at the start.
//!
//! The motion is `M(u) x = R(u) (x − c(a)) + c(u)`, `R(u) = F(u) F(a)ᵀ` the turn from the frame at
//! the curve's start `a` to the one at `u`, so the choice of the frame's axes at the start (square
//! to the tangent) moves nothing.  The frame is integrated by double reflection (Wang, Jüttler,
//! Zheng and Liu, 2008, `reflected`) over a grid laid from the start, any other parameter one
//! step from its nearest node: fourth order, its error at the grid's step a part in 10¹¹ of a
//! turn; along a curve in a plane each reflection keeps the plane's normal, so the frame is the
//! tangent, the normal and their cross product exactly but for rounding.
//!
//! Enclosed over any stretch of the parameter (`Path::stretch`, what a sweep's speed bound reads)
//! by the curve's hodographs: `|C′|` and `|C″|` bounded by their control polygons' longest pole, a
//! polynomial curve's alone — a rational one has no such bound here, and fails closed.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::nurbs::BSpline;
use crate::util::interval::Interval;
use crate::util::space::{across,cross,dot,norm,normalised,scale,sub};
use crate::{Kind,KernelError};
use super::{reflected,Motion,Path,Pose,Stretch};

type V = [f64;3];
type M = [[f64;3];3];

/// How many steps of the grid the frame is integrated over, end to end.
const GRID: usize = 4096;

/// How many samples a stretch is enclosed from (`Along::stretch`).
const SAMPLES: usize = 64;

/// A frame as three unit vectors: the tangent and two directions square to it.
type Frame = [V;3];

/// **A frame carried along a curve, from its start** (the module's): the curve, the frame at its
/// nodes, and bounds on its first two derivatives where it has them.
#[derive(Clone,Debug)]
pub struct Along {
    curve: BSpline,
    domain: [f64;2],
    /// The curve's point at its start, and the frame there.
    origin: V,
    home: Frame,
    /// The frame at the grid's nodes, in order of the parameter: each its parameter, the curve's
    /// point and the frame.
    nodes: Vec<(f64,V,Frame)>,
    /// `|C′|` and `|C″|` over the whole curve, where they are known for certain.
    bounds: Option<[f64;2]>,
}

/// The turn taking frame `b` to frame `a`: `Σ aₖ bₖᵀ`.
fn turn(a: &Frame,b: &Frame) -> M {
    std::array::from_fn(|i| std::array::from_fn(|j| a[0][i]*b[0][j]+a[1][i]*b[1][j]+a[2][i]*b[2][j]))
}

fn mv(m: &M,v: V) -> V { m.map(|r| dot(r,v)) }

/// `[ω]× m`, the rate of a turn `m` turning at `ω`.
fn spun(w: V,m: &M) -> M {
    let k = [[0.,-w[2],w[1]],[w[2],0.,-w[0]],[-w[1],w[0],0.]];
    std::array::from_fn(|i| std::array::from_fn(|j| k[i][0]*m[0][j]+k[i][1]*m[1][j]+k[i][2]*m[2][j]))
}

/// The longest pole of a polynomial curve's first two hodographs: bounds on `|C′|` and `|C″|`
/// over its whole domain (a B-spline lies in its control polygon's hull). None for a rational one.
fn hodograph_bounds(c: &BSpline) -> Option<[f64;2]> {
    if c.is_rational() { return None }
    let mut poles = c.poles.clone();
    let mut out = [0.;2];
    for (order,slot) in out.iter_mut().enumerate() {
        let degree = c.degree.checked_sub(order)?;
        let next: Vec<V> = (0..poles.len().saturating_sub(1)).map(|i| {
            let dt = c.knots[i+order+degree+1]-c.knots[i+order+1];
            if dt > 0. { scale(sub(poles[i+1],poles[i]),degree as f64/dt) } else { [0.;3] }
        }).collect();
        *slot = next.iter().map(|&q| norm(q)).fold(0.,f64::max);
        poles = next;
    }
    Some(out)
}

impl Along {
    /// **The frame carried along `curve` from its start**: refused where the curve runs over no
    /// interval, or has no tangent somewhere on the grid the frame is integrated over.
    pub fn new(curve: BSpline) -> Result<Along,KernelError> {
        let domain = curve.domain();
        let [a,b] = domain;
        if !(a < b) { return Err(KernelError::new(Kind::Failed,"a frame along a curve: the curve runs over no interval")) }
        let (origin,d) = curve.d1(a);
        let t = tangent(a,d)?;
        let (u,v) = across(t);
        let home = [t,u,v];
        let mut nodes = Vec::with_capacity(GRID+1);
        nodes.push((a,origin,home));
        for k in 1..=GRID {
            let s = if k == GRID { b } else { a+(b-a)*k as f64/GRID as f64 };
            let (x,d) = curve.d1(s);
            let (_,x0,f0) = nodes[k-1];
            nodes.push((s,x,reflected(&f0,x0,x,tangent(s,d)?)));
        }
        Ok(Along {bounds:hodograph_bounds(&curve),curve,domain,origin,home,nodes})
    }

    /// The curve the frame runs along.
    pub fn curve(&self) -> &BSpline { &self.curve }

    /// The frame at `u` (a step from its nearest node), the curve's point and first two derivatives
    /// there.
    fn frame(&self,u: f64) -> Result<(Frame,[V;3]),KernelError> {
        let [a,b] = self.domain;
        if !(u >= a-1e-12*(b-a) && u <= b+1e-12*(b-a)) {
            return Err(KernelError::new(Kind::Failed,format!("a frame along a curve runs over ({a}, {b}), and {u} is past it")))
        }
        let (x,d,dd) = self.curve.d2(u);
        let t = tangent(u,d)?;
        let k = self.nodes.partition_point(|n| n.0 < u);
        let near = match (k.checked_sub(1).map(|j| &self.nodes[j]),self.nodes.get(k)) {
            (Some(p),Some(q)) => if u-p.0 <= q.0-u { p } else { q },
            (Some(p),None) => p,
            (None,Some(q)) => q,
            (None,None) => unreachable!("the grid has its ends"),
        };
        Ok((reflected(&near.2,near.1,x,t),[x,d,dd]))
    }
}

/// The unit tangent of a curve whose derivative at `u` is `d`, or why it has none.
fn tangent(u: f64,d: V) -> Result<V,KernelError> {
    normalised(d).ok_or_else(|| KernelError::new(Kind::Failed,format!("a frame along a curve: the curve has no tangent at {u}")))
}

impl Path for Along {
    /// The pose at `u` and its rate: `x ↦ R (x − c(a)) + c(u)`, turning at `ω = T × T′`.
    fn at(&self,u: f64) -> Result<Motion,KernelError> {
        let (f,[x,d,dd]) = self.frame(u)?;
        let r = turn(&f,&self.home);
        let t = f[0];
        let bent = scale(sub(dd,scale(t,dot(dd,t))),1./norm(d));
        let dr = spun(cross(t,bent),&r);
        Ok(Motion::of_parts(r,dr,sub(x,mv(&r,self.origin)),sub(d,mv(&dr,self.origin))))
    }

    fn pose(&self,u: f64) -> Result<Pose,KernelError> {
        let (f,[x,..]) = self.frame(u)?;
        let r = turn(&f,&self.home);
        Ok(Pose {r,p:sub(x,mv(&r,self.origin))})
    }

    /// **What the motion does over `[a, b]`, enclosed**, read from samples a step `w` apart: every
    /// point within `|C′| w / 2` of one, every entry of the turn within `|ω| w / 2` of one's, and
    /// `|ω| ≤ |C″| / |C′|` with `|C′|` no less than the slowest sample less `|C″| w / 2`. None for a
    /// rational curve, or where the curve may stop.
    fn stretch(&self,a: f64,b: f64) -> Option<Stretch> {
        let [d1,d2] = self.bounds?;
        if !(a <= b) { return None }
        let (a,b) = (a.max(self.domain[0]),b.min(self.domain[1]));
        if !(a <= b) { return None }
        let w = (b-a)/SAMPLES as f64;
        let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
        let (mut rlo,mut rhi) = ([[f64::INFINITY;3];3],[[f64::NEG_INFINITY;3];3]);
        let mut slow = f64::INFINITY;
        for j in 0..=SAMPLES {
            let u = if j == SAMPLES { b } else { a+w*j as f64 };
            let (f,[x,d,_]) = self.frame(u).ok()?;
            let r = turn(&f,&self.home);
            for k in 0..3 {
                lo[k] = lo[k].min(x[k]);
                hi[k] = hi[k].max(x[k]);
                for m in 0..3 { rlo[k][m] = rlo[k][m].min(r[k][m]); rhi[k][m] = rhi[k][m].max(r[k][m]); }
            }
            slow = slow.min(norm(d));
        }
        let slowest = slow-d2*w/2.;
        if !(slowest > 0.) && d2 > 0. { return None }
        let spin = if d2 == 0. { 0. } else { d2/slowest };
        // outward by a sample's rounding; a turn's entry within ±1
        let pad = |x: f64| x.abs()*1e-12+1e-15;
        let around = |lo: f64,hi: f64,by: f64,clamp: f64| Interval::new((lo-by-pad(lo)).max(-clamp),(hi+by+pad(hi)).min(clamp)).ok();
        let mut point = [Interval::ZERO;3];
        let mut turned = [[Interval::ZERO;3];3];
        for k in 0..3 {
            point[k] = around(lo[k],hi[k],d1*w/2.,f64::INFINITY)?;
            for m in 0..3 { turned[k][m] = around(rlo[k][m],rhi[k][m],spin*w/2.,1.)?; }
        }
        Some(Stretch {point,turn:turned,speed:d1*(1.+1e-12),spin:spin*(1.+1e-12),origin:self.origin})
    }
}
