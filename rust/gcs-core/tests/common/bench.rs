//! brep-bench's swept cases as the kernel takes them, written once for `swept_rings` and
//! `swept_depth`: the tumbling torus of tube 0.16 over a half or a whole turn, the thin one, and
//! the bench runner's frame along its cubic. Each is the computation it was where it was first
//! written, to the bit — the gates pin volumes to 1e-7 and read inputs moved by an ulp — so a
//! change here is a change to every gate that reads it.
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
