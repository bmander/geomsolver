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
use crate::brep::block;
use gcs_core::brep::geom::{tolerance,Frame};
use gcs_core::brep::props::{area,volume};
use gcs_core::brep::swept::closed::{closed,cut_from};
use gcs_core::brep::swept::upright;
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Piece,Pose,Screw,Trajectory};
use std::f64::consts::{FRAC_PI_2,PI,TAU};

type V = [f64;3];

/// How finely the tool is meshed for the membership the build is judged by.
const BAR: f64 = 0.002;

/// A tool turned about `z` through the origin from its meridian in the half-plane `y = 0`, `x ≥ 0`,
/// given as `(r, z)` edges: lines, and arcs as `(centre, radius, from, to)` angles.
enum M { Line([f64;2],[f64;2]),Arc([f64;2],f64,f64,f64) }

fn turned(meridian: &[M]) -> Brep {
    let at = |p: [f64;2]| [p[0],0.,p[1]];
    let edges = meridian.iter().map(|e| match *e {
        M::Line(a,b) => ProfileEdge::Line {a:at(a),b:at(b)},
        M::Arc(c,r,a0,a1) => ProfileEdge::Arc {frame:Frame::new(at(c),[0.,-1.,0.],[1.,0.,0.]),r,span:Some([a0,a1])},
    }).collect();
    let foot = match meridian[0] { M::Line(a,_) => at(a),M::Arc(c,r,a0,_) => at([c[0]+r*a0.cos(),c[1]+r*a0.sin()]) };
    revolve(&Profile {origin:foot,normal:[0.,1.,0.],loops:vec![edges],names:Vec::new()},foot,[0.,0.,1.],TAU).unwrap()
}

/// Brep-bench's bull-nose mill: radius 2 from its flat bottom at −1.5 to 5, its bottom corner
/// rounded 0.5.
fn bull_nose() -> Brep {
    turned(&[M::Line([0.,-1.5],[1.5,-1.5]),M::Arc([1.5,-1.],0.5,-FRAC_PI_2,0.),M::Line([2.,-1.],[2.,5.]),
        M::Line([2.,5.],[0.,5.]),M::Line([0.,5.],[0.,-1.5])])
}

/// A ball-end mill: a half ball of radius 1 about the origin under a cylinder up to 3.
fn ball_end() -> Brep {
    turned(&[M::Arc([0.,0.],1.,-FRAC_PI_2,0.),M::Line([1.,0.],[1.,3.]),M::Line([1.,3.],[0.,3.]),M::Line([0.,3.],[0.,-1.])])
}

/// The constant twist `twist` taken up at `start` where the piece before left the tool, `left`.
fn taken_up(twist: Screw,left: Pose,start: f64) -> Trajectory {
    Trajectory::Placed {twist,at:left.then(twist.pose(-start))}
}

/// A slide along `d` (unit) at unit speed.
fn along(d: V) -> Screw { Screw {origin:[0.;3],axis:d,ratio:0.,advance:TAU} }

/// The bench's path, at unit speed: from the origin along `x` for `first`, a quarter turn of
/// radius `radius` toward `+y` (about the upright line through `(first, radius)`), and on along `y`
/// for `last`; and its roll.
fn line_arc_line(first: f64,radius: f64,last: f64) -> (Trajectory,[f64;2]) {
    let turn = Screw {origin:[first,radius,0.],axis:[0.,0.,1.],ratio:1./radius,advance:0.};
    let bends = [first,first+FRAC_PI_2*radius];
    let end = bends[1]+last;
    let second = taken_up(turn,along([1.,0.,0.]).pose(bends[0]),bends[0]);
    let left = second.pose(bends[1]).unwrap();
    let pieces = vec![Piece {over:[0.,bends[0]],path:Trajectory::Twist(along([1.,0.,0.]))},
                      Piece {over:bends,path:second},
                      Piece {over:[bends[1],end],path:taken_up(along([0.,1.,0.]),left,bends[1])}];
    (Trajectory::pieces(pieces,1e-9,100.).unwrap(),[0.,end])
}

fn near(got: f64,want: f64,rel: f64,what: &str) {
    assert!((got-want).abs() <= rel*want.abs(),"{what}: {got} where {want} ({:e} off)",(got-want).abs()/want.abs());
}

/// **The bench's groove** (brep-bench `swept/bull-nose-along-line-arc-line`): the block
/// (−10, −10, −10)–(40, 40, 0) less the mill run along the line, the arc and the line, to its
/// closed forms — below the top the mill's radius is `R(z) = 1.5 + √(0.25 − (z + 1)²)` under
/// `z = −1` and 2 above it, its section's area `A = 2 ∫R = 4 + 2(0.75 + π/16)` and the mill's
/// volume there `V₀ = π ∫R² = π (2 + 1.25 + 3π/16 − 1/24)`, along `L = 40 + 5π`; the opening, the
/// floor, the walls and the mill's own faces below the top give the area. Built by this road (it
/// is asked first), closed and right side out.
#[test]
fn the_bull_nose_run_along_a_line_an_arc_and_a_line_cuts_its_groove() {
    let (stock,tool) = (block([-10.,-10.,-10.],[40.,40.,0.]),bull_nose());
    let (path,roll) = line_arc_line(20.,10.,20.);
    let tol = tolerance(stock.size().max(tool.size()));
    let l = 40.+5.*PI;
    let a = 4.+2.*(0.75+PI/16.);
    let v0 = PI*(2.+1.25+3.*PI/16.-1./24.);
    let s = 7000.-(4.*l+4.*PI)+(3.*l+2.25*PI)+2.*l*(0.5*PI/2.+1.)+2.*PI*0.5*(1.5*PI/2.+0.5)+2.*PI*2.*1.;
    let built = upright::swept(&tool,&path,roll,BAR,tol,Some(&stock)).unwrap().expect("this road takes it");
    built.check(tol).unwrap();
    near(volume(&built),25000.-(a*l+v0),1e-9,"the volume");
    near(area(&built),s,1e-9,"the area");
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
    let (path,roll) = line_arc_line(20.,10.,20.);
    let tol = tolerance(tool.size()*10.);
    let l = 40.+5.*PI;
    let a = 24.+2.*(0.75+PI/16.);
    let v = PI*(24.+1.25+3.*PI/16.-1./24.);
    let s = (19.+PI/2.)*l+2.25*PI+PI*(0.75*PI+0.5)+24.*PI+4.*PI;
    let built = closed(&tool,&path,roll,BAR,tol).unwrap();
    built.check(tol).unwrap();
    near(volume(&built),a*l+v,1e-9,"the volume");
    near(area(&built),s,1e-9,"the area");
}

/// **A ball-end mill along a slide and a half turn**: its section's two arcs meet at its pole, one
/// arc across it — a half disc of radius 1 under a rectangle 2 by 3 — and Pappus about the turn's
/// line again: `(π/2 + 6) L + V`, `L = 10 + 3π` and `V = 2π/3 + 3π`.
#[test]
fn a_ball_end_mill_along_a_slide_and_a_half_turn_is_its_section_along_the_path() {
    let tool = ball_end();
    let turn = Screw {origin:[10.,3.,0.],axis:[0.,0.,1.],ratio:1./3.,advance:0.};
    let bends = [10.,10.+3.*PI];
    let pieces = vec![Piece {over:[0.,10.],path:Trajectory::Twist(along([1.,0.,0.]))},
                      Piece {over:bends,path:taken_up(turn,along([1.,0.,0.]).pose(10.),10.)}];
    let path = Trajectory::pieces(pieces,1e-9,100.).unwrap();
    let tol = tolerance(40.);
    let built = upright::swept(&tool,&path,[0.,bends[1]],BAR,tol,None).unwrap().expect("this road takes it");
    built.check(tol).unwrap();
    let l = 10.+3.*PI;
    near(volume(&built),(PI/2.+6.)*l+2.*PI/3.+3.*PI,1e-9,"the volume");
}

/// **What this road hands on**, read before anything is built: two slides meeting at a corner (the
/// path's direction broken there), a ring for a tool (a slice no disc), a path coming back within the
/// tool's width of itself (two quarter turns of radius 3 and a slide back beside the first, 1 from
/// it), and a turn about a line the tool reaches (radius 1.5 for a mill of radius 2).
#[test]
fn a_broken_path_a_ring_a_path_lapping_itself_and_a_turn_the_tool_reaches_are_handed_on() {
    let mill = bull_nose();
    let tol = tolerance(100.);
    let corner = Trajectory::pieces(vec![
        Piece {over:[0.,10.],path:Trajectory::Twist(along([1.,0.,0.]))},
        Piece {over:[10.,20.],path:taken_up(along([0.,1.,0.]),along([1.,0.,0.]).pose(10.),10.)}],1e-9,100.).unwrap();
    assert!(upright::swept(&mill,&corner,[0.,20.],BAR,tol,None).unwrap().is_none(),"a corner");
    let ring = turned(&[M::Line([1.,0.],[2.,0.]),M::Line([2.,0.],[2.,1.]),M::Line([2.,1.],[1.,1.]),M::Line([1.,1.],[1.,0.])]);
    let (path,roll) = line_arc_line(20.,10.,20.);
    assert!(upright::swept(&ring,&path,roll,BAR,tol,None).unwrap().is_none(),"a ring");
    // along x for 10, a half turn of radius 3 about (10, 3), back along −x past the start, 6 from
    // the first run: wider than the mill's 4, so taken; after a half turn of radius 2.2 the slide
    // back runs 4.4 from the first, nearer than the mill is wide and its samples' spacing, and is
    // handed on — as is a half turn of radius 1.5, about a line the mill reaches
    let u = |radius: f64| {
        let turn = Screw {origin:[10.,radius,0.],axis:[0.,0.,1.],ratio:1./radius,advance:0.};
        let bends = [10.,10.+PI*radius];
        let second = taken_up(turn,along([1.,0.,0.]).pose(10.),10.);
        let left = second.pose(bends[1]).unwrap();
        (Trajectory::pieces(vec![Piece {over:[0.,10.],path:Trajectory::Twist(along([1.,0.,0.]))},
                                 Piece {over:bends,path:second},
                                 Piece {over:[bends[1],bends[1]+20.],path:taken_up(along([-1.,0.,0.]),left,bends[1])}],1e-9,100.).unwrap(),
         [0.,bends[1]+20.])
    };
    let (wide,roll) = u(3.);
    assert!(upright::swept(&mill,&wide,roll,BAR,tol,None).unwrap().is_some(),"a U wide enough");
    let (tight,roll) = u(2.2);
    assert!(upright::swept(&mill,&tight,roll,BAR,tol,None).unwrap().is_none(),"a U too tight");
    let (reached,roll) = u(1.5);
    assert!(upright::swept(&mill,&reached,roll,BAR,tol,None).unwrap().is_none(),"a turn the mill reaches");
}
