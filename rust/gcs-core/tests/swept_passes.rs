//! **A tool of revolution under a program of passes** (`brep::swept::passes`): a flat or a
//! bull-nose mill carried round a path of slides and turns about lines along its axis, plunged
//! and raised along it, each move's sweep built in closed form, the runs of moves that follow on
//! glued into tubes, and taken from the blank at once — the whole entered through
//! `closed::cut_from`, which asks this class first. Held to Pappus: round a closed path whose arcs
//! are wider than the tool, the groove's section is the same all round, symmetric about the path,
//! so it takes `A·L` from the plate and adds `P·L` to its area less the opening, `A` the section's
//! area, `P` its walls' and floor's length, `L` the path's. And a tool no solid of revolution, or a
//! move no slide or turn the class takes, is left to the arrangement.
use crate::brep::block;
use gcs_core::brep::build::{revolve,Profile,ProfileEdge};
use gcs_core::brep::geom::{tolerance,Frame,Surface,V};
use gcs_core::brep::props::{area,volume};
use gcs_core::brep::swept::closed::cut_from;
use gcs_core::brep::swept::passes;
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Piece,Pose,Screw,Trajectory};
use gcs_core::space::{norm,scale,sub};
use std::f64::consts::{FRAC_PI_2,PI,TAU};

/// A mill about the upright line through `tip`, its tip there: a flat end of radius `r - corner`,
/// its corner rounded to `corner` (none for a flat end mill), its side up to `length`.
fn mill(tip: V,r: f64,corner: f64,length: f64) -> Brep {
    let at = |x: f64,z: f64| [tip[0]+x,tip[1],tip[2]+z];
    let mut edges = vec![ProfileEdge::Line {a:at(0.,0.),b:at(r-corner,0.)}];
    if corner > 0. {
        edges.push(ProfileEdge::Arc {frame:Frame::new(at(r-corner,corner),[0.,-1.,0.],[1.,0.,0.]),r:corner,span:Some([-FRAC_PI_2,0.])});
    }
    edges.extend([ProfileEdge::Line {a:at(r,corner),b:at(r,length)},ProfileEdge::Line {a:at(r,length),b:at(0.,length)},
        ProfileEdge::Line {a:at(0.,length),b:at(0.,0.)}]);
    revolve(&Profile {origin:tip,normal:[0.,1.,0.],loops:vec![edges],names:Vec::new()},tip,[0.,0.,1.],TAU).unwrap()
}

/// A program's path of pieces, each move a constant twist taken up where the last left the tool:
/// the tool drawn standing at `from`.
struct Program { pieces: Vec<Piece>,pose: Pose,roll: f64,from: V }

impl Program {
    fn new(from: V) -> Program { Program {pieces:Vec::new(),pose:Pose {r:[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],p:[0.;3]},roll:0.,from} }
    fn tip(&self) -> V { self.pose.point(self.from) }
    fn take(&mut self,twist: Screw,length: f64) {
        let start = self.roll;
        self.pieces.push(Piece {over:[start,start+length],path:Trajectory::Placed {twist,at:self.pose.then(twist.pose(-start))}});
        self.roll += length;
        self.pose = self.pose.then(twist.pose(length));
    }
    fn line(&mut self,to: V) {
        let d = sub(to,self.tip());
        let l = norm(d);
        self.take(Screw {origin:[0.;3],axis:scale(d,1./l),ratio:0.,advance:TAU},l);
    }
    /// Round the upright line through `c` by `angle` (anticlockwise for more than nought).
    fn arc(&mut self,c: [f64;2],angle: f64) {
        let t = self.tip();
        let length = (t[0]-c[0]).hypot(t[1]-c[1])*angle.abs();
        self.take(Screw {origin:[c[0],c[1],0.],axis:[0.,0.,1.],ratio:angle/length,advance:0.},length);
    }
    fn path(self) -> (Trajectory,[f64;2]) {
        let end = self.roll;
        (Trajectory::pieces(self.pieces,1e-9,100.).unwrap(),[0.,end])
    }
}

/// Once round the square with corners `lo` and `hi`, its corners rounded to `round`, at height
/// `z`, anticlockwise from the middle of its lowest side, the tip there.
fn lap(p: &mut Program,lo: [f64;2],hi: [f64;2],round: f64,z: f64) {
    let mid = 0.5*(lo[0]+hi[0]);
    p.line([hi[0]-round,lo[1],z]);
    p.arc([hi[0]-round,lo[1]+round],FRAC_PI_2);
    p.line([hi[0],hi[1]-round,z]);
    p.arc([hi[0]-round,hi[1]-round],FRAC_PI_2);
    p.line([lo[0]+round,hi[1],z]);
    p.arc([lo[0]+round,hi[1]-round],FRAC_PI_2);
    p.line([lo[0],lo[1]+round,z]);
    p.arc([lo[0]+round,lo[1]+round],FRAC_PI_2);
    p.line([mid,lo[1],z]);
}

fn total_area(b: &Brep) -> f64 { b.faces.iter().map(|f| area(b,f)).sum() }

/// The plate (−10, −10, −6)–(30, 30, 0), its volume and area.
fn plate() -> (Brep,f64,f64) { (block([-10.,-10.,-6.],[30.,30.,0.]),40.*40.*6.,2.*1600.+4.*40.*6.) }

/// **A flat end mill run round a rounded square at two depths cuts one groove**: the shallower
/// lap's walls lie on the deeper's, and the plunges and the rise lie within it. The square
/// (0, 0)–(20, 20), its corners rounded to 3, more than the mill's 1, is `80 − 4·(2·3) + 2π·3`
/// round; the groove 2 wide and 4 deep.
#[test]
fn a_flat_end_mill_round_a_rounded_square_cuts_its_groove_by_pappus() {
    let (stock,v0,a0) = plate();
    let start = [10.,0.,2.];
    let tool = mill(start,1.,0.,10.);
    let mut p = Program::new(start);
    for z in [-2.,-4.] {
        p.line([10.,0.,z]);
        lap(&mut p,[0.,0.],[20.,20.],3.,z);
    }
    p.line(start);
    let (path,roll) = p.path();
    let built = cut_from(&stock,&tool,&path,roll,0.01,tolerance(stock.size())).unwrap();
    let l = 80.-24.+6.*PI;
    let (v,a) = (v0-2.*4.*l,a0+2.*4.*l);
    assert!((volume(&built)-v).abs() <= 1e-9*v,"volume {} against {v}",volume(&built));
    assert!((total_area(&built)-a).abs() <= 1e-9*a,"area {} against {a}",total_area(&built));
}

/// **A bull-nose mill run round a rounded square cuts a groove with rounded edges**: its corner of
/// 0.5 sweeps a quarter cylinder along each side and a quarter of a torus round each arc, and the
/// groove's section is `2r·d − (4 − π)c²/2`; its floor `2(r − c)` wide, its corners `πc/2` round
/// each, its walls `d − c` high.
#[test]
fn a_bull_nose_mill_round_a_rounded_square_cuts_its_rounded_groove_by_pappus() {
    let (stock,v0,a0) = plate();
    let (r,c,d) = (1.5,0.5,3.);
    let start = [10.,0.,2.];
    let tool = mill(start,r,c,10.);
    let mut p = Program::new(start);
    p.line([10.,0.,-d]);
    lap(&mut p,[0.,0.],[20.,20.],4.,-d);
    p.line(start);
    let (path,roll) = p.path();
    let built = cut_from(&stock,&tool,&path,roll,0.01,tolerance(stock.size())).unwrap();
    let l = 80.-32.+8.*PI;
    let section = 2.*r*d-(4.-PI)*c*c/2.;
    let walls = 2.*(d-c)+PI*c+2.*(r-c)-2.*r;
    let (v,a) = (v0-section*l,a0+walls*l);
    assert!((volume(&built)-v).abs() <= 1e-9*v,"volume {} against {v}",volume(&built));
    assert!((total_area(&built)-a).abs() <= 1e-9*a,"area {} against {a}",total_area(&built));
}

/// **A whole turn and a raster beside it**: a flat end mill carried once round a circle about a
/// line along its axis (one move, a ring), and slid across beside it, each pass plunged and raised
/// — the ring and the slot cut separately, their measures adding.
#[test]
fn a_whole_turn_and_a_slot_cut_apart_add() {
    let (stock,v0,a0) = plate();
    let start = [5.,5.,2.];
    let tool = mill(start,1.,0.,10.);
    let mut p = Program::new(start);
    p.line([5.,5.,-2.]);
    p.arc([2.,5.],TAU);
    p.line([5.,5.,2.]);
    p.line([15.,0.,2.]);
    p.line([15.,0.,-2.]);
    p.line([25.,0.,-2.]);
    p.line([25.,0.,2.]);
    let (path,roll) = p.path();
    let built = cut_from(&stock,&tool,&path,roll,0.01,tolerance(stock.size())).unwrap();
    // the ring: radii 2 to 4 about (2, 5), 2 deep; the slot: a stadium 10 long, 2 wide, 2 deep
    let (ring,slot) = (PI*(16.-4.)*2.,(20.+PI)*2.);
    let v = v0-ring-slot;
    let a = a0+2.*PI*(4.+2.)*2.+(2.*10.+2.*PI)*2.;
    assert!((volume(&built)-v).abs() <= 1e-9*v,"volume {} against {v}",volume(&built));
    assert!((total_area(&built)-a).abs() <= 1e-9*a,"area {} against {a}",total_area(&built));
}

/// **What is not the class's is left to the arrangement**: a block is no solid of revolution, and
/// a slide aslant a mill's axis no move the class takes.
#[test]
fn a_tool_or_a_move_the_class_does_not_take_is_left_to_the_arrangement() {
    let box_tool = block([0.,0.,0.],[1.,1.,1.]);
    let slide = Trajectory::Twist(Screw {origin:[0.;3],axis:[1.,0.,0.],ratio:0.,advance:TAU});
    assert!(passes::built(&box_tool,&slide,[0.,1.],0.01,1e-9,None).unwrap().is_none());
    let tool = mill([0.;3],1.,0.,5.);
    let aslant = Trajectory::Twist(Screw {origin:[0.;3],axis:scale([1.,0.,1.],0.5f64.sqrt()),ratio:0.,advance:TAU});
    assert!(passes::built(&tool,&aslant,[0.,1.],0.01,1e-9,None).unwrap().is_none());
    // a slide square to the axis is the class's, its sweep a stadium's prism
    let built = passes::built(&tool,&slide,[0.,1.],0.01,1e-9,None).unwrap().expect("the class's");
    let v = (PI+2.)*5.;
    assert!((volume(&built)-v).abs() <= 1e-9*v,"volume {} against {v}",volume(&built));
}

/// **A circle in a plane square to a torus's axis meets it in closed form** (`query`): touching the
/// torus's lowest rim in its tangent plane from outside, one touch where the quartic along the
/// circle has a fourfold root; across its section's two circles, four crossings.
#[test]
fn a_circle_level_with_a_torus_meets_it_in_closed_form() {
    use gcs_core::brep::geom::Curve;
    use gcs_core::brep::query::{curve_surface,Meets};
    let (big,r) = (2.5,1.5);
    let torus = Surface::Torus(Frame::about([0.,0.,1.5],[0.,0.,1.]),big,r);
    // in the plane z = 0, the torus's tangent plane below: a circle of radius 3 about (5.5, 0)
    // touches the rim (radius 2.5) at (2.5, 0, 0)
    let touching = Curve::Circle(Frame::about([5.5,0.,0.],[0.,0.,1.]),3.);
    let Meets::At(roots) = curve_surface(&touching,[0.,TAU],&torus,1e-9) else { panic!("no meeting read") };
    assert_eq!(roots.len(),1,"{roots:?}");
    let p = touching.point(roots[0].0);
    assert!(norm(sub(p,[2.5,0.,0.])) <= 1e-7,"{p:?}");
    // through the middle of the tube: the section's circles of radii 1 and 4, crossed twice each by
    // a circle of radius 2 about (2.5, 0)
    let across = Curve::Circle(Frame::about([2.5,0.,1.5],[0.,0.,1.]),2.);
    let Meets::At(roots) = curve_surface(&across,[0.,TAU],&torus,1e-9) else { panic!("no meeting read") };
    assert_eq!(roots.len(),4,"{roots:?}");
    for (t,_) in roots { let q = across.point(t); assert!(torus.implicit(q).abs() <= 1e-9,"{q:?}") }
}
