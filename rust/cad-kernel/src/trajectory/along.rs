//! **A frame carried along a curve** (`Along`, K2): the rigid motion that takes what is drawn at
//! the curve's start along it, its frame square to the curve's tangent turning only as the tangent
//! does — the rotation-minimising frame, `ω = T × T′ = C′ × C″ / |C′|²` — so along a curve in a
//! plane its axis square to the plane stays put, and a tool drawn upright at the start is carried
//! upright, its first axis along the tangent wherever it was along it at the start.
//!
//! The motion is `M(u) x = R(u) (x − c(a)) + c(u)`, `R(u) = F(u) F(a)ᵀ` the turn from the frame at
//! the curve's start `a` to the one at `u`, so the choice of the frame's axes at the start (square
//! to the tangent) moves nothing.  The frame is integrated by double reflection (Wang, Jüttler,
//! Zheng and Liu, 2008, `reflected`) over a uniform grid laid from the start, any other parameter
//! one step from its nearest node: fourth order, its error at the grid's step a part in 10¹¹ of a
//! turn; along a curve in a plane each reflection keeps the plane's normal, so the frame is the
//! tangent, the normal and their cross product exactly but for rounding.
//!
//! Enclosed over any stretch of the parameter (`Path::stretch`, what a sweep's speed bound reads)
//! by the curve's hodographs (`BSpline::derivative_bounds`), a polynomial curve's alone — a
//! rational one has no such bound here, and fails closed.
use crate::nurbs::BSpline;
use crate::util::interval::Interval;
use crate::util::space::{across,cross,dot,norm,normalised,scale,sub};
use crate::{Kind,KernelError};
use super::{mm,mv,reflected,transpose,Motion,Path,Pose,Stretch,M3,V3};

/// How many steps of the grid the frame is integrated over, end to end.
const GRID: usize = 4096;

/// How many samples a stretch is enclosed from (`Along::stretch`).
const SAMPLES: usize = 64;

/// **A frame carried along a curve, from its start** (the module's): the curve, the curve's point
/// and the frame (a tangent and two directions square to it) at the grid's nodes from its start,
/// bounds on its first two derivatives where it has them, and the whole curve's stretch.
#[derive(Debug)]
pub struct Along { curve: BSpline,nodes: Vec<(V3,[V3;3])>,bounds: Option<[f64;2]>,whole: Option<Stretch> }

impl Along {
    /// **The frame carried along `curve` from its start**: refused where the curve runs over no
    /// interval, or has no tangent somewhere on the grid the frame is integrated over.
    pub fn new(curve: BSpline) -> Result<Along,KernelError> {
        let [a,b] = curve.domain();
        if !(a < b) { return Err(KernelError::new(Kind::Failed,"a frame along a curve: the curve runs over no interval")) }
        let (origin,d) = curve.d1(a);
        let t = tangent(a,d)?;
        let (u,v) = across(t);
        let mut nodes = Vec::with_capacity(GRID+1);
        nodes.push((origin,[t,u,v]));
        for k in 1..=GRID {
            let s = if k == GRID { b } else { a+(b-a)*k as f64/GRID as f64 };
            let (x,d) = curve.d1(s);
            let (x0,f0) = nodes[k-1];
            nodes.push((x,reflected(&f0,x0,x,tangent(s,d)?)));
        }
        let mut along = Along {bounds:curve.derivative_bounds(),curve,nodes,whole:None};
        along.whole = along.enclosed(a,b);
        Ok(along)
    }

    /// The curve the frame runs along.
    pub fn curve(&self) -> &BSpline { &self.curve }

    /// The curve's point at its start.
    fn origin(&self) -> V3 { self.nodes[0].0 }

    /// The turn at `u` from the start, `F(u) F(a)ᵀ`, the curve's point `x` and derivative `d` there:
    /// the frame a step from its nearest node.
    fn turn(&self,u: f64,x: V3,d: V3) -> Result<M3,KernelError> {
        let [a,b] = self.curve.domain();
        if !(u >= a-1e-12*(b-a) && u <= b+1e-12*(b-a)) {
            return Err(KernelError::new(Kind::Failed,format!("a frame along a curve runs over ({a}, {b}), and {u} is past it")))
        }
        let (x0,f0) = self.nodes[((u-a)/(b-a)*GRID as f64).round().clamp(0.,GRID as f64) as usize];
        Ok(mm(transpose(reflected(&f0,x0,x,tangent(u,d)?)),self.nodes[0].1))
    }

    /// `stretch` over `[a, b]` within the domain, read from samples a step `w` apart: every point
    /// within `|C′| w / 2` of one, every entry of the turn within `|ω| w / 2` of one's, and
    /// `|ω| ≤ |C″| / |C′|` with `|C′|` no less than the slowest sample less `|C″| w / 2`.
    fn enclosed(&self,a: f64,b: f64) -> Option<Stretch> {
        let [d1,d2] = self.bounds?;
        let w = (b-a)/SAMPLES as f64;
        let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
        let (mut rlo,mut rhi) = ([[f64::INFINITY;3];3],[[f64::NEG_INFINITY;3];3]);
        let mut slow = f64::INFINITY;
        for j in 0..=SAMPLES {
            let u = if j == SAMPLES { b } else { a+w*j as f64 };
            let (x,d) = self.curve.d1(u);
            let r = self.turn(u,x,d).ok()?;
            for k in 0..3 {
                (lo[k],hi[k]) = (lo[k].min(x[k]),hi[k].max(x[k]));
                for m in 0..3 { (rlo[k][m],rhi[k][m]) = (rlo[k][m].min(r[k][m]),rhi[k][m].max(r[k][m])) }
            }
            slow = slow.min(norm(d));
        }
        let slowest = slow-d2*w/2.;
        if !(slowest > 0.) { return None }
        let spin = d2/slowest;
        // outward by a sample's rounding
        let around = |lo: f64,hi: f64,by: f64| Interval::new(lo-by-lo.abs()*1e-12-1e-15,hi+by+hi.abs()*1e-12+1e-15).ok();
        let point = [around(lo[0],hi[0],d1*w/2.)?,around(lo[1],hi[1],d1*w/2.)?,around(lo[2],hi[2],d1*w/2.)?];
        // (every entry of a turn lies within ±1)
        let entry = |k: usize,m: usize| {
            let [lo,hi] = around(rlo[k][m],rhi[k][m],spin*w/2.)?.bounds();
            Interval::new(lo.max(-1.),hi.min(1.)).ok()
        };
        let mut turn = [[Interval::ZERO;3];3];
        for k in 0..3 { for m in 0..3 { turn[k][m] = entry(k,m)? } }
        Some(Stretch {point,turn,speed:d1*(1.+1e-12),spin:spin*(1.+1e-12),origin:self.origin()})
    }
}

/// The unit tangent of a curve whose derivative at `u` is `d`, or why it has none.
fn tangent(u: f64,d: V3) -> Result<V3,KernelError> {
    normalised(d).ok_or_else(|| KernelError::new(Kind::Failed,format!("a frame along a curve: the curve has no tangent at {u}")))
}

impl Path for Along {
    /// The pose at `u` and its rate: `x ↦ R (x − c(a)) + c(u)`, turning at `ω = C′ × C″ / |C′|²`.
    fn at(&self,u: f64) -> Result<Motion,KernelError> {
        let (x,d,dd) = self.curve.d2(u);
        let r = self.turn(u,x,d)?;
        let w = scale(cross(d,dd),1./dot(d,d));
        let dr = mm([[0.,-w[2],w[1]],[w[2],0.,-w[0]],[-w[1],w[0],0.]],r);
        Ok(Motion::of_parts(r,dr,sub(x,mv(r,self.origin())),sub(d,mv(dr,self.origin()))))
    }

    /// The pose alone, off the curve's first derivative.
    fn pose(&self,u: f64) -> Result<Pose,KernelError> {
        let (x,d) = self.curve.d1(u);
        let r = self.turn(u,x,d)?;
        Ok(Pose {r,p:sub(x,mv(r,self.origin()))})
    }

    /// **What the motion does over `[a, b]`, enclosed** (`enclosed`; the whole curve's read once).
    /// None for a rational curve, or where the curve may stop.
    fn stretch(&self,a: f64,b: f64) -> Option<Stretch> {
        let [lo,hi] = self.curve.domain();
        if !(a <= b) { return None }
        if a <= lo && b >= hi { return self.whole }
        self.enclosed(a.max(lo),b.min(hi))
    }
}
