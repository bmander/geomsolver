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
