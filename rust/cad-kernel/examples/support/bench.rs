//! brep-bench's cases the kernel's examples share, written once: the runner's frame along the
//! bench's cubic (`ring_build`, `ring_depth`, `ring_profile`) and the tumbling torus
//! (`depth_probe`, `ring_profile`, `size_check`). Cargo builds each example as a crate of its own,
//! so each takes this file in as a module by `#[path = "support/bench.rs"] mod bench;`; a
//! directory with no `main.rs` is no example itself.  Each is the computation it was where it was
//! first written, to the bit: the gates these examples measure pin volumes to 1e-7.
#![allow(dead_code)]
use cad_kernel::nurbs::BSpline;
use cad_kernel::trajectory::{Motion,Path,Screw,Trajectory};

/// The cubic brep-bench's ring is carried along, in the plane `z = 0`.
pub fn cubic() -> BSpline {
    let points = vec![[0.,0.,0.],[10.,0.,0.],[20.,20.,0.],[30.,20.,0.]];
    BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],points).unwrap()
}

/// The frame brep-bench's runner carries the ring by: its `x` along the curve's tangent, its `z`
/// up.
#[derive(Debug)]
pub struct Runner(pub BSpline);
impl Path for Runner {
    fn at(&self,t: f64) -> Result<Motion,cad_kernel::KernelError> {
        let (point,d,dd) = self.0.d2(t);
        let speed2 = d[0]*d[0]+d[1]*d[1];
        let (c,s) = (d[0]/speed2.sqrt(),d[1]/speed2.sqrt());
        let turning = (d[0]*dd[1]-d[1]*dd[0])/speed2;
        let r = [[c,-s,0.],[s,c,0.],[0.,0.,1.]];
        let dr = [[-s*turning,-c*turning,0.],[c*turning,-s*turning,0.],[0.;3]];
        Ok(Motion::of_parts(r,dr,point,d))
    }
}

/// The runner's frame along the bench's cubic.
pub fn runner() -> Trajectory { Trajectory::General(std::sync::Arc::new(Runner(cubic()))) }

/// brep-bench's tumbling torus: a torus of major radius one about `centre`, its axis `z`, turned by
/// `ratio · t` about the vertical line through `(hub, 0, 0)`, seen from a frame turning by
/// `observer · t` about the line through `(0, h cos ℓ, h sin ℓ)` along
/// `(cos α, sin α cos ℓ, sin α sin ℓ)`.
#[allow(clippy::too_many_arguments)]
pub fn tumbling(centre: [f64;3],hub: f64,tube: f64,ratio: f64,lean: f64,heading: f64,height: f64,
    observer: f64) -> (cad_kernel::topo::Brep,Trajectory) {
    tumbling_moved(centre,hub,tube,ratio,lean,heading,height,observer,|origin| origin)
}

/// The tumbling torus with its observer's origin `(0, h cos ℓ, h sin ℓ)` as `moved` gives it
/// back.
#[allow(clippy::too_many_arguments)]
pub fn tumbling_moved(centre: [f64;3],hub: f64,tube: f64,ratio: f64,lean: f64,heading: f64,
    height: f64,observer: f64,moved: impl FnOnce([f64;3]) -> [f64;3])
    -> (cad_kernel::topo::Brep,Trajectory) {
    let tool = cad_kernel::build::torus(centre,[0.,0.,1.],1.,tube).unwrap();
    let (sl,cl) = lean.to_radians().sin_cos();
    let (sa,ca) = heading.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[hub,0.,0.],axis:[0.,0.,1.],ratio,advance:0.});
    let origin = moved([0.,height*cl,height*sl]);
    let frame = Trajectory::Twist(Screw {origin,axis:[ca,sa*cl,sa*sl],ratio:observer,
        advance:0.});
    (tool,Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)})
}

/// brep-bench's `bull-nose-along-a-looping-spline`: the block (−10, −10, −8)–(30, 25, 0), a
/// bull-nose mill of radius 2 from 1.5 below the top to 5 above it, its corner rounded to 0.5,
/// carried without turning along a cubic Bézier that crosses itself, its foot on the curve.
pub mod bull_nose {
    use cad_kernel::build::{Profile,ProfileEdge,revolve};
    use cad_kernel::geom::Frame;
    use cad_kernel::nurbs::BSpline;
    use cad_kernel::topo::Brep;
    use cad_kernel::trajectory::{Motion,Path,Stretch,Trajectory};
    use cad_kernel::util::interval::Interval;
    use cad_kernel::util::space::{norm,sub};
    use std::f64::consts::{FRAC_PI_2,TAU};
    type V = [f64;3];
    pub const STOCK_LO: V = [-10.,-10.,-8.];
    pub const STOCK_HI: V = [30.,25.,0.];
    pub const CONTROL: [V;4] = [[0.,0.,0.],[30.,20.,0.],[-10.,20.,0.],[20.,0.,0.]];
    pub const FOOT: V = [0.,0.,-1.5];
    pub const RADIUS: f64 = 2.;
    pub const CORNER: f64 = 0.5;
    pub const TOP: f64 = 5.;
    pub const BAR: f64 = 0.002;

    /// The mill: its meridian in the plane `y = 0` revolved a whole turn about the `z` axis
    /// through its foot.
    pub fn mill() -> Result<Brep,String> {
        let at = |x: f64,z: f64| [x,0.,z];
        let [_,_,z0] = FOOT;
        let corner = Frame::new(at(RADIUS-CORNER,z0+CORNER),[0.,-1.,0.],[1.,0.,0.]);
        let edges = vec![
            ProfileEdge::Line {a:at(0.,z0),b:at(RADIUS-CORNER,z0)},
            ProfileEdge::Arc {frame:corner,r:CORNER,span:Some([-FRAC_PI_2,0.])},
            ProfileEdge::Line {a:at(RADIUS,z0+CORNER),b:at(RADIUS,TOP)},
            ProfileEdge::Line {a:at(RADIUS,TOP),b:at(0.,TOP)},
            ProfileEdge::Line {a:at(0.,TOP),b:at(0.,z0)},
        ];
        let meridian = Profile {origin:FOOT,normal:[0.,-1.,0.],loops:vec![edges],names:vec![]};
        revolve(&meridian,FOOT,[0.,0.,1.],TAU)
    }

    /// The curve's hodograph's poles, `3 (P[i+1] − P[i])`.
    fn hodograph(control: &[V;4]) -> [V;3] {
        std::array::from_fn(|i| sub(control[i+1],control[i]).map(|x| 3.*x))
    }

    /// The mill carried without turning along the curve, the curve's start at the mill's foot.
    #[derive(Debug)]
    pub struct Along(pub BSpline,pub [V;4]);
    impl Path for Along {
        fn at(&self,t: f64) -> Result<Motion,cad_kernel::KernelError> {
            let (point,rate) = self.0.d1(t);
            Motion::translation(sub(point,self.1[0]),rate)
                .map_err(|e| cad_kernel::KernelError::from(format!("the curve's pose at {t}: {e:?}")))
        }
        /// The curve within its poles' box, no turn, its speed within its hodograph's longest
        /// pole.
        fn stretch(&self,_: f64,_: f64) -> Option<Stretch> {
            let moved = self.1.map(|p| sub(p,self.1[0]));
            let one = |i: usize,j: usize| Interval::point(if i == j { 1. } else { 0. }).ok();
            let turn = [[one(0,0)?,one(0,1)?,one(0,2)?],[one(1,0)?,one(1,1)?,one(1,2)?],
                [one(2,0)?,one(2,1)?,one(2,2)?]];
            let speed = hodograph(&self.1).iter().map(|&h| norm(h)).fold(0.,f64::max);
            let axis = |k: usize| {
                let lo = moved.iter().map(|p| p[k]).fold(f64::INFINITY,f64::min);
                let hi = moved.iter().map(|p| p[k]).fold(f64::NEG_INFINITY,f64::max);
                Interval::new(lo,hi).ok()
            };
            Some(Stretch {point:[axis(0)?,axis(1)?,axis(2)?],turn,speed,spin:0.,origin:[0.;3]})
        }
    }

    /// The curve through `control`.
    pub fn curve(control: [V;4]) -> BSpline {
        BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],control.to_vec()).unwrap()
    }

    /// The case with the curve's poles `control`: stock, tool, path, roll, bar and tolerance.
    pub fn case(control: [V;4]) -> (Brep,Brep,Trajectory,[f64;2],f64,f64) {
        let stock = cad_kernel::build::block(STOCK_LO,STOCK_HI).unwrap();
        let tool = mill().unwrap();
        let path = Trajectory::General(std::sync::Arc::new(Along(curve(control),control)));
        let size = stock.size().max(tool.size());
        (stock,tool,path,[0.,1.],BAR,cad_kernel::geom::tolerance(size))
    }
}
