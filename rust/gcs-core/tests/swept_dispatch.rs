//! **A sweep's method is chosen by the kernel** (#721 F1c, `brep::swept::dispatch::sweep`): one
//! `sweep` reads the tool from its B-rep and the trajectory by its structure, asks the classes in
//! order and admits by the first that takes it. Built here in the kernel alone, with no drawing:
//! a sweep depends on the set, not on how it was written, so a drum built as a disc's prism and as
//! a rectangle's revolution is one tool to the dispatch, and a ball built as a sphere is the
//! canal's whatever carries it.
use gcs_core::brep::build::{disc,prism,revolve,sphere,Profile,ProfileEdge};
use gcs_core::brep::swept::admission::Sampling;
use gcs_core::brep::swept::dispatch::{self,Admitted,Ask,Blank,Class,Refused,Sweep};
use gcs_core::brep::swept::subtraction::Names;
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Motion,Screw,Trajectory};

type V = [f64;3];

/// A box blank (model units, a millimetre each), its least and greatest corner.
struct Boxed(V,V);

impl Blank for Boxed {
    fn inside(&self,p: V) -> bool { (0..3).all(|k| p[k] > self.0[k] && p[k] < self.1[k]) }
    fn extent(&self) -> Option<(V,V)> { Some((self.0,self.1)) }
    fn boxed(&self) -> Option<(V,V)> { Some((self.0,self.1)) }
}

/// The block x 8 to 12, y −3 to 3, z −1 to 1, which a tool 10 from z turned ±60° about it passes
/// wholly through.
const BLOCK: Boxed = Boxed([8.,-3.,-1.],[12.,3.,1.]);

/// The turn about z, a radian a unit of the roll.
fn turn() -> Trajectory { Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:0.}) }

/// The sweep of `tool` along `path` over ±60°, cut from `BLOCK` where it stands.
fn swept(tool: &Brep,path: &Trajectory) -> Result<Admitted,Refused> {
    let names = Names {sweep:"groove".into(),tool:"tool".into(),motion:"turn".into()};
    let s = Sweep {tool:Ok(tool),path,roll:[-60f64.to_radians(),60f64.to_radians()],mm:1.,names:&names,pocket:None};
    let sampling = Sampling {rows:100,columns:400,coarse_rows:24,coarse_columns:1440,root_tolerance:1e-9,least_factor:1e-3};
    dispatch::sweep(&s,&BLOCK,&Ask {poses:&[Motion::identity()],revolved:None,sampling,margin:1e-6})
}

/// The region a planar subtraction's one slab sweeps: its loops' areas.
fn areas(a: &Admitted) -> Vec<f64> {
    let Class::Subtraction(found) = &a.class else { panic!("the planar subtraction class: {:?}",a.class) };
    found.slabs.iter().flat_map(|s| s.stations.iter().flat_map(|(_,r)| r.loops.iter().map(|l| l.area))).collect()
}

/// **A drum is its set, however it was built**: radius 1 and 1 high, standing 10 from z, built as a
/// disc's prism and as a rectangle's revolution about its axis. Read from its B-rep each stands
/// along z, which the turn keeps every plane square to, so the planar subtraction class (asked
/// before the constant twist's rows) reads both, the region its disc sweeps through the block the
/// same to rounding.
#[test]
fn a_drum_drawn_two_ways_is_one_tool_to_the_dispatch() {
    let prismed = prism(&disc([10.,0.,-0.5],[0.,0.,1.],1.),0.,1.).unwrap();
    let corners = [[10.,0.,-0.5],[11.,0.,-0.5],[11.,0.,0.5],[10.,0.,0.5]];
    let rectangle = Profile {origin:[10.,0.,0.],normal:[0.,1.,0.],
        loops:vec![(0..4).map(|k| ProfileEdge::Line {a:corners[k],b:corners[(k+1)%4]}).collect()],names:Vec::new()};
    let revolved = revolve(&rectangle,[10.,0.,0.],[0.,0.,1.],std::f64::consts::TAU).unwrap();
    let (a,b) = (swept(&prismed,&turn()).unwrap(),swept(&revolved,&turn()).unwrap());
    let (a,b) = (areas(&a),areas(&b));
    eprintln!("the prism's region {a:?}, the revolution's {b:?}");
    assert!(!a.is_empty() && a.len() == b.len(),"{a:?} against {b:?}");
    for (x,y) in a.iter().zip(&b) { assert!((x-y).abs() <= 1e-9*x.abs().max(1.),"{x} against {y}") }
}

/// **A ball is the canal's**: built as a sphere, carried by the turn, it sweeps its tube whatever
/// drew it; slid along z instead, its tube too.
#[test]
fn a_ball_is_the_canals_whatever_carries_it() {
    let ball = sphere([10.,0.,0.],[0.,0.,1.],0.5).unwrap();
    assert!(matches!(swept(&ball,&turn()).unwrap().class,Class::Canal(_)),"turned");
    let slide = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,1.,0.],ratio:0.,advance:std::f64::consts::TAU});
    assert!(matches!(swept(&ball,&slide).unwrap().class,Class::Canal(_)),"slid");
}

/// **What no class reads is refused by a typed row**: a tool the drawing has no B-rep of is refused
/// at T1 in the words the drawing gives, by the class its motion asks.
#[test]
fn a_tool_with_no_boundary_is_refused_at_t1() {
    let names = Names::default();
    let s = Sweep {tool:Err("`tool` builds no boundary to read"),path:&turn(),roll:[0.,1.],mm:1.,names:&names,pocket:None};
    let sampling = Sampling {rows:100,columns:400,coarse_rows:24,coarse_columns:1440,root_tolerance:1e-9,least_factor:1e-3};
    let r = dispatch::sweep(&s,&BLOCK,&Ask {poses:&[Motion::identity()],revolved:None,sampling,margin:1e-6}).unwrap_err();
    assert_eq!((r.row,r.asked),(dispatch::Row::Tool,dispatch::Asked::ConstantTwist));
    assert!(matches!(&r.seen,dispatch::Seen::Words(m) if m.contains("no boundary")),"{:?}",r.seen);
}
