//! **A solid of revolution carried square to its axis, swept closed** (#721 K1,
//! `brep::swept::upright`): a tool every slice of which square to its line is a disc about it,
//! moved along a path of slides square to that line and turns about lines along it, the path's
//! direction kept where its pieces meet — its axial section carried along the path, its halves at
//! the ends, sewn into one solid, nothing met in space but the blank.
//!
//! Held apart from the build by the closed forms of the definition: at each height the sweep is
//! the region within the tool's radius there of the axis's path, a band and two half discs, so its
//! volume is the section's area along the path and the tool's own (Pappus on an arc: the section is
//! symmetric about the path), and its boundary the section's perimeter along the path and the
//! tool's own curved faces. Brep-bench's `swept/bull-nose-along-line-arc-line`, which the
//! arrangement refused as wrong after five minutes, to its closed forms; and the readings that hand
//! a sweep on to the other roads — a broken direction, a ring for a tool, a path that comes back
//! on itself, a turn about a line the tool reaches.
use gcs_core::brep::build::{revolve,Profile,ProfileEdge};
use gcs_core::brep::geom::{tolerance,Frame,Rigid};
use gcs_core::brep::props::volume;
use gcs_core::brep::swept::closed::{closed,cut_from};
use gcs_core::brep::swept::upright;
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Piece,Screw,Trajectory};
use crate::brep::{block,total_area};
use std::f64::consts::{FRAC_PI_2,PI,TAU};

type V = [f64;3];

/// How finely the tool is meshed for the membership the build is judged by.
const BAR: f64 = 0.002;

/// Brep-bench's bull-nose mill (`swept_closed::bull_nose`): radius 2 from its flat bottom at −1.5
/// up to 5, its bottom corner rounded 0.5.
fn bull_nose() -> Brep {
    crate::swept_closed::bull_nose(0.,2.,0.5,6.5).moved(&Rigid {t:[0.,0.,-1.5],..Rigid::identity()})
}

/// A tool turned about `z` from its meridian's edges in the half-plane `y = 0`, `x ≥ 0`.
fn turned(edges: Vec<ProfileEdge>) -> Brep {
    revolve(&Profile {origin:[0.;3],normal:[0.,1.,0.],loops:vec![edges],names:Vec::new()},[0.;3],[0.,0.,1.],TAU).unwrap()
}

/// `(r, z)` in the half-plane `y = 0`.
fn at(r: f64,z: f64) -> V { [r,0.,z] }

/// A ball-end mill: a half ball of radius 1 about the origin under a cylinder up to 3.
fn ball_end() -> Brep {
    turned(vec![ProfileEdge::Arc {frame:Frame::new([0.;3],[0.,-1.,0.],[1.,0.,0.]),r:1.,span:Some([-FRAC_PI_2,0.])},
        ProfileEdge::Line {a:at(1.,0.),b:at(1.,3.)},ProfileEdge::Line {a:at(1.,3.),b:at(0.,3.)},ProfileEdge::Line {a:at(0.,3.),b:at(0.,-1.)}])
}

/// A step of a path in the plane `z = 0`, taken at unit speed from where the last left the tool:
/// on along the heading, a turn of a radius through an angle (to the left where it is positive),
/// or a new heading where it stands — a corner.
enum Go { Slide(f64),Turn(f64,f64),Head(V) }

/// The path of `steps` from the origin heading along `x`, and its roll.
fn path(steps: &[Go]) -> (Trajectory,[f64;2]) {
    let (mut pieces,mut t,mut at,mut heading): (Vec<Piece>,f64,V,V) = (Vec::new(),0.,[0.;3],[1.,0.,0.]);
    for step in steps {
        let (twist,length) = match *step {
            Go::Head(h) => { heading = h; continue }
            Go::Slide(l) => (Screw {origin:[0.;3],axis:heading,ratio:0.,advance:TAU},l),
            Go::Turn(r,w) => {
                let c = [at[0]-r*w.signum()*heading[1],at[1]+r*w.signum()*heading[0],0.];
                (Screw {origin:c,axis:[0.,0.,1.],ratio:w.signum()/r,advance:0.},r*w.abs())
            }
        };
        // taken up where the last piece left the tool, standing there at `t`
        let path = match pieces.last() {
            None => Trajectory::Twist(twist),
            Some(p) => Trajectory::Placed {twist,at:p.path.pose(t).unwrap().then(twist.pose(-t))},
        };
        let end = path.pose(t+length).unwrap();
        (at,heading) = (end.point([0.;3]),end.vector([1.,0.,0.]));
        pieces.push(Piece {over:[t,t+length],path});
        t += length;
    }
    (Trajectory::pieces(pieces,1e-9,100.).unwrap(),[0.,t])
}

fn near(got: f64,want: f64,rel: f64,what: &str) {
    assert!((got-want).abs() <= rel*want.abs(),"{what}: {got} where {want} ({:e} off)",(got-want).abs()/want.abs());
}

/// **The bench's groove** (brep-bench `swept/bull-nose-along-line-arc-line`): the block
/// (−10, −10, −10)–(40, 40, 0) less the mill run along the line, the arc and the line, to its
/// closed forms — below the top the mill's radius is `R(z) = 1.5 + √(0.25 − (z + 1)²)` under
/// `z = −1` and 2 above it, its section's area `A = 2 ∫R = 4 + 2(0.75 + π/16)` and the mill's
/// volume there `V₀ = π ∫R² = π (4 + 1.25 + 3π/16 − 1/24)`, along `L = 40 + 5π`; the opening, the
/// floor, the walls and the mill's own faces below the top give the area. Built by this road (it
/// is asked first), closed and right side out.
#[test]
fn the_bull_nose_run_along_a_line_an_arc_and_a_line_cuts_its_groove() {
    let (stock,tool) = (block([-10.,-10.,-10.],[40.,40.,0.]),bull_nose());
    let (path,roll) = path(&[Go::Slide(20.),Go::Turn(10.,FRAC_PI_2),Go::Slide(20.)]);
    let tol = tolerance(stock.size().max(tool.size()));
    let l = 40.+5.*PI;
    let a = 4.+2.*(0.75+PI/16.);
    let v0 = PI*(4.+1.25+3.*PI/16.-1./24.);
    let s = 7000.-(4.*l+4.*PI)+(3.*l+2.25*PI)+2.*l*(0.5*PI/2.+1.)+2.*PI*0.5*(1.5*PI/2.+0.5)+2.*PI*2.*1.;
    let built = upright::swept(&tool,&path,roll,BAR,tol,Some(&stock)).unwrap().expect("this road takes it");
    built.check(tol).unwrap();
    near(volume(&built),25000.-(a*l+v0),1e-9,"the volume");
    near(total_area(&built),s,1e-9,"the area");
    // and the kernel's closed sweep takes this road
    let cut = cut_from(&stock,&tool,&path,roll,BAR,tol).unwrap();
    near(volume(&cut),25000.-(a*l+v0),1e-9,"the cut's volume");
}

/// **The same path swept closed, with no blank**: the whole mill's section along the path and the
/// mill itself — its section `2(6·2) + 2(0.75 + π/16)` (the round's `∫R` below `z = −1`) and its
/// volume `π(4·6 + 1.25 + 3π/16 − 1/24)`; its boundary the section's perimeter `3 + π/2 + 12 + 4`
/// along the path, and the mill's faces: its flat bottom, its quarter round, its side and its top.
#[test]
fn the_bull_nose_swept_closed_is_its_section_along_the_path_and_itself() {
    let tool = bull_nose();
    let (path,roll) = path(&[Go::Slide(20.),Go::Turn(10.,FRAC_PI_2),Go::Slide(20.)]);
    let tol = tolerance(tool.size()*10.);
    let l = 40.+5.*PI;
    let a = 24.+2.*(0.75+PI/16.);
    let v = PI*(24.+1.25+3.*PI/16.-1./24.);
    let s = (19.+PI/2.)*l+2.25*PI+PI*(0.75*PI+0.5)+24.*PI+4.*PI;
    let built = closed(&tool,&path,roll,BAR,tol).unwrap();
    built.check(tol).unwrap();
    near(volume(&built),a*l+v,1e-9,"the volume");
    near(total_area(&built),s,1e-9,"the area");
}

/// **A ball-end mill along a slide and a half turn**: its section's two arcs meet at its pole, one
/// arc across it — a half disc of radius 1 under a rectangle 2 by 3 — and Pappus about the turn's
/// line again: `(π/2 + 6) L + V`, `L = 10 + 3π` and `V = 2π/3 + 3π`.
#[test]
fn a_ball_end_mill_along_a_slide_and_a_half_turn_is_its_section_along_the_path() {
    let (path,roll) = path(&[Go::Slide(10.),Go::Turn(3.,PI)]);
    let tol = tolerance(40.);
    let built = upright::swept(&ball_end(),&path,roll,BAR,tol,None).unwrap().expect("this road takes it");
    built.check(tol).unwrap();
    near(volume(&built),(PI/2.+6.)*(10.+3.*PI)+2.*PI/3.+3.*PI,1e-9,"the volume");
}

/// **What this road hands on**, read before anything is built: two slides meeting at a corner (the
/// path's direction broken there), a ring for a tool (a slice no disc), a path coming back onto
/// itself (round a stadium to where it began), and a turn about a line the tool reaches (radius 1.5
/// for a mill of radius 2) — while a U whose return runs 6 from its start, wider than the mill's 4,
/// is taken.
#[test]
fn a_broken_path_a_ring_a_path_lapping_itself_and_a_turn_the_tool_reaches_are_handed_on() {
    let mill = bull_nose();
    let tol = tolerance(100.);
    let handed_on = |tool: &Brep,steps: &[Go]| { let (p,roll) = path(steps); upright::swept(tool,&p,roll,BAR,tol,None).unwrap().is_none() };
    assert!(handed_on(&mill,&[Go::Slide(10.),Go::Head([0.,1.,0.]),Go::Slide(10.)]),"a corner");
    let ring = turned(vec![ProfileEdge::Line {a:at(1.,0.),b:at(2.,0.)},ProfileEdge::Line {a:at(2.,0.),b:at(2.,1.)},
        ProfileEdge::Line {a:at(2.,1.),b:at(1.,1.)},ProfileEdge::Line {a:at(1.,1.),b:at(1.,0.)}]);
    assert!(handed_on(&ring,&[Go::Slide(20.),Go::Turn(10.,FRAC_PI_2),Go::Slide(20.)]),"a ring");
    assert!(!handed_on(&mill,&[Go::Slide(10.),Go::Turn(3.,PI),Go::Slide(20.)]),"a U wide enough");
    assert!(handed_on(&mill,&[Go::Slide(10.),Go::Turn(3.,PI),Go::Slide(10.),Go::Turn(3.,PI)]),"round a stadium");
    assert!(handed_on(&mill,&[Go::Slide(10.),Go::Turn(1.5,PI),Go::Slide(20.)]),"a turn the mill reaches");
}
