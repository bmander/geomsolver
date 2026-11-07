//! **One reading a surface, its corners partway along it** (#721 F1d): the faces a Boolean cuts
//! from one surface of revolution are read once, over the span of its meridian they reach
//! (`read::once_a_surface`), and a face over part of a turn beside a face of revolution about the
//! same axis is read about it, whatever its loop (`Tool::read`): a part-cylinder's kind is no
//! longer its loop's. The walk reads a corner partway along a face's meridian where the face goes no
//! further at the walk's point, and crosses it there (`constant_twist`'s partway corners). Held
//! here in the kernel alone: a drum with a window ground down through its side by a quarter-turn
//! groove, turned about a line square to its axis, its characteristic run up its side, across the
//! window's floor and up its back wall; and a partial revolution, its side a strip meeting its
//! rounds' rings, pushed through a plate as the whole revolution is.
use gcs_core::brep::boolean::{boolean,Op};
use gcs_core::brep::build::{revolve,Profile,ProfileEdge};
use gcs_core::brep::swept::admission::{check_screw,Generating,Sampling};
use gcs_core::brep::swept::constant_twist::Characteristic;
use gcs_core::brep::swept::tool::{Tool,ToolSurface};
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Screw,Trajectory};
use gcs_core::space::segment_distance;

type V = [f64;3];

/// The rectangle from radius `r0` to `r1` and height `z0` to `z1` in the half-plane `angle` radians
/// round z from +x, turned about z through `sweep`.
fn turned(r0: f64,r1: f64,z0: f64,z1: f64,angle: f64,sweep: f64) -> Brep {
    let (s,c) = angle.sin_cos();
    let at = |r: f64,z: f64| -> V { [r*c,r*s,z] };
    let corners = [at(r0,z0),at(r1,z0),at(r1,z1),at(r0,z1)];
    let profile = Profile {origin:at(r0,z0),normal:[-s,c,0.],
        loops:vec![(0..4).map(|k| ProfileEdge::Line {a:corners[k],b:corners[(k+1)%4]}).collect()],names:Vec::new()};
    revolve(&profile,[0.;3],[0.,0.,1.],sweep).unwrap()
}

/// A drum of radius 2 standing 8 high on z, a window ground down through its side and its top to
/// height 3, a quarter of a turn about +x, down to radius 1.5.
fn windowed() -> Brep {
    let drum = turned(0.,2.,0.,8.,0.,std::f64::consts::TAU);
    let groove = turned(1.5,3.,3.,9.,-std::f64::consts::FRAC_PI_4,std::f64::consts::FRAC_PI_2);
    boolean(&drum,&groove,Op::Cut,1e-9).unwrap()
}

/// The turn about the line along x through (0, 0, 28), above the drum, a radian a unit of the roll.
/// (Its axis meets the drum's, so a ring whose normals pass through where they meet is in contact
/// all round, #684 X: none lies on the faces and corners the blank is reached by.)
fn turn() -> Screw { Screw {origin:[0.,0.,28.],axis:[1.,0.,0.],ratio:1.,advance:0.} }

/// The blank: x 0.5 to 3, and from 21 to 27 below the turn's axis, 0.1 to 0.4 radians round it
/// ahead of the drum, which the turn carries the drum's side at +x through between heights 1 and 7.
fn inside(p: V) -> bool {
    let (y,z) = (p[1],28.-p[2]);
    let (r,psi) = (y.hypot(z),y.atan2(z));
    p[0] > 0.5 && p[0] < 3. && r > 21. && r < 27. && psi > 0.1 && psi < 0.4
}

/// The characteristic the turn's normals make on the drum where it reaches the blank: in the
/// half-plane y = 0 at +x, up the side to the window, across its floor and up its back wall: the
/// distance from `p` to that path.
fn off(p: V) -> f64 {
    let path: [V;4] = [[2.,0.,0.],[2.,0.,3.],[1.5,0.,3.],[1.5,0.,8.]];
    path.windows(2).map(|w| segment_distance(p,w[0],w[1])).fold(f64::INFINITY,f64::min)
}

/// **The drum's side is read once, its window's floor and back wall about its axis**: one face of
/// revolution the side, whole round over its height; the window's floor a flat and its back wall a
/// part-cylinder, each read as a revolution about z although its loop is a strip's; the corner
/// where the floor meets the side partway along the side's meridian, at height 3, and where the
/// back wall meets the top partway along the top's.
#[test]
fn a_windowed_drum_is_read_a_surface_at_a_time() {
    let tool = Tool::read(&windowed(),1.).unwrap();
    let revolved: Vec<_> = tool.faces.iter().filter_map(|f| match f { ToolSurface::Revolved(s) => Some(s),_ => None }).collect();
    let sides: Vec<_> = revolved.iter().filter(|s| s.at(0.5,0.).is_ok_and(|p| (p.position[0].hypot(p.position[1])-2.).abs() < 1e-9)
        && s.flat().is_none()).collect();
    assert_eq!(sides.len(),1,"the drum's side read once: {:?}",tool.faces.iter().map(|f| f.name()).collect::<Vec<_>>());
    let back = revolved.iter().filter(|s| s.at(0.5,0.).is_ok_and(|p| (p.position[0].hypot(p.position[1])-1.5).abs() < 1e-9)).count();
    assert_eq!(back,1,"the window's back wall a revolution about z");
    // each corner partway along a face, where on that face it lies
    let partway: Vec<V> = tool.edges.iter().flat_map(|e| (0..2).filter(|&s| e.ends[s] > 0. && e.ends[s] < 1.)
        .filter_map(|s| tool.faces[e.patches[s]].at(e.ends[s],0.).ok()).map(|p| p.position).collect::<Vec<_>>()).collect();
    eprintln!("corners partway: {partway:?}");
    let ring = |p: V| [p[0].hypot(p[1]),p[2]];
    assert!(partway.iter().any(|&p| (ring(p)[0]-2.).abs() < 1e-9 && (ring(p)[1]-3.).abs() < 1e-9),"the floor's corner: {partway:?}");
    assert!(partway.iter().any(|&p| (ring(p)[0]-1.5).abs() < 1e-9 && (ring(p)[1]-8.).abs() < 1e-9),"the back wall's top: {partway:?}");
}

/// **The walk crosses a corner partway along a face**: the windowed drum turned through the blank,
/// its characteristic walked up its side to the window's floor, where the side goes no further at
/// the walk's point (`Walker::partway`), across the floor and up the back wall out of the blank:
/// every node on that path, and the path walked through the window.
#[test]
fn the_walk_crosses_a_corner_partway_along_a_face() {
    let tool = Tool::read(&windowed(),1.).unwrap();
    let path = Trajectory::Twist(turn());
    let sweep = Generating {tool:&tool,path:&path,roll:[0.,0.5],whole:false,mm:1.};
    let walked: Characteristic = check_screw(&sweep,&inside,None,&Sampling::default(),None)
        .unwrap_or_else(|e| panic!("the walk: {e:?}")).0;
    let worst = walked.nodes.iter().map(|n| off(n.position)).fold(0.,f64::max);
    let lowest = walked.nodes.iter().map(|n| n.position[2]).fold(f64::INFINITY,f64::min);
    let highest = walked.nodes.iter().map(|n| n.position[2]).fold(f64::NEG_INFINITY,f64::max);
    let back = walked.nodes.iter().filter(|n| (n.position[0]-1.5).abs() < 1e-9 && n.position[2] > 3.5 && n.position[2] < 6.5).count();
    eprintln!("{} nodes from height {lowest:.3} to {highest:.3}, {worst:.2e} off the path, {back} up the back wall",walked.nodes.len());
    assert!(worst <= 1e-9,"a node {worst:.2e} off the characteristic's path");
    assert!(lowest < 1. && highest > 7.,"walked through the blank's heights: {lowest} to {highest}");
    assert!(back > 0,"walked up the window's back wall");
}

/// **A partial revolution's strips meet its rounds, and its ends hold no contact**: the
/// bull-nosed drum (`swept_charts::drum`) turned three quarters of a turn, the quarter it leaves
/// out about +z, pushed up through the plate. Its side over part of a turn is a strip, which meets
/// each round, a revolution, along the ring at its meridian's end: the walk crosses from the one to
/// the other there (`Junctions::new`'s strips on a ring). Its ends are planes through its axis that
/// pass through the plate, but the push's normals never lie in them, so they hold no
/// characteristic and are no refusal (T1 under a constant twist, #721 F1d). Its characteristic is
/// the whole drum's: the push's silhouette lies in what the three quarters keep.
#[test]
fn a_partial_revolutions_strips_cross_onto_its_rounds() {
    use crate::swept_charts::{alike,drum,drum_turned,push,Plate};
    use gcs_core::brep::swept::dispatch::Blank;
    let quarters = Tool::read(&drum_turned(std::f64::consts::FRAC_PI_4,1.5*std::f64::consts::PI),1.).unwrap();
    let kinds: Vec<&str> = quarters.faces.iter().map(|f| match f { ToolSurface::Revolved(_) => "revolved",ToolSurface::Extruded(_) => "strip",
        ToolSurface::Cap(_) => "cap" }).collect();
    eprintln!("three quarters read as {kinds:?}");
    assert!(kinds.contains(&"strip") && kinds.contains(&"cap"),"its side a strip, its ends caps");
    // the push up through the plate, walked
    let path = push();
    let screw = path.screw().expect("the push is a slide");
    let walk = |tool: &Tool| {
        let sweep = Generating {tool,path:&path,roll:[0.,std::f64::consts::TAU],whole:false,mm:1.};
        let extent = Plate.extent().map(|(lo,hi)| screw.extent(lo,hi,gcs_core::brep::trajectory::Motion::identity()));
        check_screw(&sweep,&|p| Plate.inside(p),extent,&Sampling::default(),None).unwrap_or_else(|e| panic!("the walk: {e:?}")).0
    };
    alike("three quarters",&walk(&Tool::read(&drum(0.),1.).unwrap()),&walk(&quarters));
}
