//! brep-bench's swept cases as the kernel takes them, written once for `swept_rings`,
//! `swept_depth`, `swept_tube`, `swept_tumbles` and `swept_upright`: the tumbling torus of tube 0.16 over a half or
//! a whole turn, the thin one, the bench's own tumbling torus of tube 0.5, the bench runner's frame
//! along its cubic, and the bull nose carried along its looping spline. Each is the computation it
//! was where it was first written, to the bit — the gates pin volumes to 1e-7 and read inputs
//! moved by an ulp — so a change here is a change to every gate that reads it.
use gcs_core::brep::nurbs::BSpline;
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Motion,Path,Screw,Trajectory};

/// brep-bench's tumbling torus of tube 0.16: turned 0.375 a radian about one line and back a
/// radian about another, `half` the roll each way in degrees — its `tumbling-torus-half-turn` at
/// 90, its `tumbling-torus-full-turn` at 180 — its tool, motion, roll and tolerance.
pub fn tumbling(half: f64) -> (Brep,Trajectory,[f64;2],f64) {
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.16).unwrap();
    let (sl,cl) = 30f64.to_radians().sin_cos();
    let (sa,ca) = 20f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.375,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.7071*cl,0.7071*sl],axis:[ca,sa*cl,sa*sl],
        ratio:1.,advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    (tool,path,[-half.to_radians(),half.to_radians()],0.16/50_000.)
}

/// brep-bench's `tumbling-torus-thin`: a torus of tube 0.015 turned a quarter of a radian about
/// the vertical line through (2, 0, 0) and back a radian about the line through (0, h cos 45°,
/// h sin 45°) along `x`, `h` √½, over ±75°, at the bench's tolerance, a 50 000th of the tube.
pub fn thin() -> (Brep,Trajectory,[f64;2],f64) {
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.015).unwrap();
    let (sl,cl) = 45f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.25,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.5f64.sqrt()*cl,0.5f64.sqrt()*sl],
        axis:[1.,0.,0.],ratio:1.,advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    (tool,path,[-75f64.to_radians(),75f64.to_radians()],0.015/50_000.)
}

/// brep-bench's `tumbling-torus`: the thin's motion (a quarter of a radian about the vertical line
/// through (2, 0, 0), back a radian about the line through (0, ½, ½) along `x`) carrying a torus of
/// tube 0.5 over ±75°, at the bench's tolerance, a 50 000th of the tube.
pub fn tumbling_torus() -> (Brep,Trajectory,[f64;2],f64) {
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.5).unwrap();
    let (sl,cl) = 45f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.25,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.5f64.sqrt()*cl,0.5f64.sqrt()*sl],
        axis:[1.,0.,0.],ratio:1.,advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    (tool,path,[-75f64.to_radians(),75f64.to_radians()],0.5/50_000.)
}

/// brep-bench's crossed tumbling tori (`tumbling-torus-crossed-forward` at `observer` 1,
/// `-reverse` at −1): a torus of tube 0.22 turned 0.7 a radian about the vertical line through
/// (2, 0, 0), seen from a frame turning by `observer` a radian about the line through
/// (0, h cos 45°, h sin 45°), `h` 0.7071, along (cos 20°, sin 20° cos 45°, sin 20° sin 45°), over
/// ±100°, at the bench's tolerance, a 50 000th of the tube.
pub fn crossed(observer: f64) -> (Brep,Trajectory,[f64;2],f64) { crossed_moved(observer,|o| o) }

/// `crossed` with its observer's origin `(0, h cos 45°, h sin 45°)` as `moved` gives it back.
pub fn crossed_moved(observer: f64,moved: impl FnOnce([f64;3]) -> [f64;3]) -> (Brep,Trajectory,[f64;2],f64) {
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.22).unwrap();
    let (sl,cl) = 45f64.to_radians().sin_cos();
    let (sa,ca) = 20f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.7,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:moved([0.,0.7071*cl,0.7071*sl]),axis:[ca,sa*cl,sa*sl],
        ratio:observer,advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    (tool,path,[-100f64.to_radians(),100f64.to_radians()],0.22/50_000.)
}

/// **A tumbling torus as brep-bench's runner states one** (`tumble.rs`'s `Tumble`): a torus of
/// radius one about the vertical through `centre`, of tube `tube`, turned `ratio` a radian about the
/// vertical line through `(hub, 0, 0)` and seen from a frame turning `observer` a radian about the
/// line through `(0, h cos lean, h sin lean)` along `(cos heading, sin heading cos lean, sin heading
/// sin lean)`, over `roll` (degrees), at a 50 000th of the tube. `crossed`'s members are
/// `Tumble::crossed(±1)`.
#[derive(Clone,Copy,Debug)]
pub struct Tumble {
    pub centre: [f64;3],
    pub hub: f64,
    pub tube: f64,
    pub ratio: f64,
    pub lean: f64,
    pub heading: f64,
    pub height: f64,
    pub observer: f64,
    pub roll: [f64;2],
}

impl Tumble {
    /// brep-bench's crossed members: `observer` 1 the forward, −1 the reverse.
    pub fn crossed(observer: f64) -> Tumble {
        Tumble {centre:[3.,0.,2.],hub:2.,tube:0.22,ratio:0.7,lean:45.,heading:20.,height:0.7071,observer,roll:[-100.,100.]}
    }

    /// Its tool, motion, roll (radians) and tolerance.
    pub fn sweep(&self) -> (Brep,Trajectory,[f64;2],f64) {
        let tool = gcs_core::brep::build::torus(self.centre,[0.,0.,1.],1.,self.tube).unwrap();
        let (sl,cl) = self.lean.to_radians().sin_cos();
        let (sa,ca) = self.heading.to_radians().sin_cos();
        let own = Trajectory::Twist(Screw {origin:[self.hub,0.,0.],axis:[0.,0.,1.],ratio:self.ratio,advance:0.});
        let frame = Trajectory::Twist(Screw {origin:[0.,self.height*cl,self.height*sl],axis:[ca,sa*cl,sa*sl],
            ratio:self.observer,advance:0.});
        let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
        (tool,path,self.roll.map(f64::to_radians),self.tube/50_000.)
    }

    /// Swept as the runner sweeps it (`swept::closed::ring`, its bar the tube's 250th).
    pub fn ring(&self) -> Result<Brep,gcs_core::brep::swept::closed::Unbuilt> {
        let (tool,path,roll,tol) = self.sweep();
        gcs_core::brep::swept::closed::ring(&tool,&path,roll,self.tube/250.,tol)
    }
}

/// The cubic brep-bench's ring is carried along, in the plane `z = 0`.
pub fn cubic() -> BSpline {
    let points = vec![[0.,0.,0.],[10.,0.,0.],[20.,20.,0.],[30.,20.,0.]];
    BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],points).unwrap()
}

/// The frame brep-bench's runner carries the ring by along a curve in the plane `z = 0`: its `x`
/// along the curve's tangent, its `z` up; a path read only by evaluating it.
#[derive(Debug)]
pub struct Runner(pub BSpline);
impl Path for Runner {
    fn at(&self,t: f64) -> Result<Motion,gcs_core::brep::KernelError> {
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

/// brep-bench's `bull-nose-along-a-looping-spline`: the block (−10, −10, −8)–(30, 25, 0), a
/// bull-nose mill of radius 2 from 1.5 below the top to 5 above it, its corner rounded to 0.5,
/// carried without turning along a cubic Bézier that crosses itself, its foot on the curve.
pub mod bull_nose {
    use gcs_core::brep::build::{Profile,ProfileEdge,revolve};
    use gcs_core::brep::geom::Frame;
    use gcs_core::brep::nurbs::BSpline;
    use gcs_core::brep::topo::Brep;
    use gcs_core::brep::trajectory::{Motion,Path,Stretch,Trajectory};
    use gcs_core::interval::Interval;
    use gcs_core::space::{norm,sub};
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
        fn at(&self,t: f64) -> Result<Motion,gcs_core::brep::KernelError> {
            let (point,rate) = self.0.d1(t);
            Motion::translation(sub(point,self.1[0]),rate)
                .map_err(|e| gcs_core::brep::KernelError::from(format!("the curve's pose at {t}: {e:?}")))
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
        let stock = gcs_core::brep::build::block(STOCK_LO,STOCK_HI).unwrap();
        let tool = mill().unwrap();
        let path = Trajectory::General(std::sync::Arc::new(Along(curve(control),control)));
        let size = stock.size().max(tool.size());
        (stock,tool,path,[0.,1.],BAR,gcs_core::brep::geom::tolerance(size))
    }
}
