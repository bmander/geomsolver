//! A probe of the upright road's edges (#880 T1f): tools the road does not read — a ball end, a
//! pointed taper, a concave meridian, a flat step — carried along the bull nose's looping spline,
//! and what the closed class does with each.
#[path = "support/bench.rs"]
mod bench;
use bench::bull_nose::*;
use cad_kernel::build::{Profile,ProfileEdge,revolve};
use cad_kernel::geom::Frame;

fn tool(edges: Vec<ProfileEdge>) -> cad_kernel::topo::Brep {
    revolve(&Profile {origin:FOOT,normal:[0.,-1.,0.],loops:vec![edges],names:vec![]},FOOT,[0.,0.,1.],std::f64::consts::TAU).unwrap()
}

fn main() {
    let at = |x: f64,z: f64| [x,0.,z];
    let z0 = FOOT[2];
    let ball = tool(vec![
        ProfileEdge::Arc {frame:Frame::new(at(0.,z0+RADIUS),[0.,-1.,0.],[1.,0.,0.]),r:RADIUS,span:Some([-std::f64::consts::FRAC_PI_2,0.])},
        ProfileEdge::Line {a:at(RADIUS,z0+RADIUS),b:at(RADIUS,TOP)},
        ProfileEdge::Line {a:at(RADIUS,TOP),b:at(0.,TOP)},
        ProfileEdge::Line {a:at(0.,TOP),b:at(0.,z0)}]);
    let taper = tool(vec![
        ProfileEdge::Line {a:at(0.,z0),b:at(RADIUS,z0+1.)},
        ProfileEdge::Line {a:at(RADIUS,z0+1.),b:at(RADIUS,TOP)},
        ProfileEdge::Line {a:at(RADIUS,TOP),b:at(0.,TOP)},
        ProfileEdge::Line {a:at(0.,TOP),b:at(0.,z0)}]);
    let step = tool(vec![
        ProfileEdge::Line {a:at(0.,z0),b:at(1.,z0)},
        ProfileEdge::Line {a:at(1.,z0),b:at(1.,z0+1.)},
        ProfileEdge::Line {a:at(1.,z0+1.),b:at(RADIUS,z0+1.)},
        ProfileEdge::Line {a:at(RADIUS,z0+1.),b:at(RADIUS,TOP)},
        ProfileEdge::Line {a:at(RADIUS,TOP),b:at(0.,TOP)},
        ProfileEdge::Line {a:at(0.,TOP),b:at(0.,z0)}]);
    let hollow = tool(vec![
        ProfileEdge::Line {a:at(0.,z0),b:at(RADIUS,z0)},
        ProfileEdge::Line {a:at(RADIUS,z0),b:at(RADIUS,z0+0.5)},
        ProfileEdge::Arc {frame:Frame::new(at(RADIUS,z0+1.),[0.,1.,0.],[-1.,0.,0.]),r:0.5,span:Some([-std::f64::consts::FRAC_PI_2,std::f64::consts::FRAC_PI_2])},
        ProfileEdge::Line {a:at(RADIUS,z0+1.5),b:at(RADIUS,TOP)},
        ProfileEdge::Line {a:at(RADIUS,TOP),b:at(0.,TOP)},
        ProfileEdge::Line {a:at(0.,TOP),b:at(0.,z0)}]);
    let (stock,_,path,roll,bar,tol) = case(CONTROL);
    for (name,t) in [("ball end",ball),("pointed taper",taper),("flat step",step),("concave meridian",hollow)] {
        let clock = std::time::Instant::now();
        let read = cad_kernel::swept::upright::swept(&t,&path,roll,bar,tol).map(|b| b.is_some());
        let built = cad_kernel::swept::closed::cut_from(&stock,&t,&path,roll,bar,tol);
        println!("{name}: the road reads it: {read:?}; cut_from in {:?}: {}",clock.elapsed(),match built {
            Ok(b) => format!("built, volume {:.6}",cad_kernel::props::volume(&b)),Err(e) => format!("{e:?}") });
    }
}
