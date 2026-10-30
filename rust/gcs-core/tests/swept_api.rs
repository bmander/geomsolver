//! **The kernel's swept cut without a document** (`brep::sweep::cut`, cad-kernel's model-free
//! entry point) held to the model's pipeline: the solid sweeps series' documents
//! (`rust/examples/solid_sweeps/`) built by the export `solventc` writes from, and again through the
//! kernel's API alone from the same numbers — the blank and the tool built from plain data, the
//! motion a slide, a turn, a screw, or a slide seen from a turn — each within 0.01 mm, their
//! volumes and areas agreeing. The two read the blank apart (the pipeline its analytic field, the
//! API its boundary), and a rack's sheet apart (its planar envelope extruded, and traced), so they
//! agree to the fit, not to the bit.
use gcs_core::brep::build::{disc,prism,revolve,Profile,ProfileEdge};
use gcs_core::brep::boolean::{combined,Op};
use gcs_core::brep::props::{area,volume};
use gcs_core::brep::sweep::cut::{swept_cut_with,Ask,Motion,Tool};
use gcs_core::brep::sweep::Say;
use gcs_core::brep::topo::Brep;
use gcs_core::solid::export::Tolerance;
use std::f64::consts::{PI,TAU};

type V = [f64;3];

const TOLERANCE: f64 = 0.01;

fn measured(b: &Brep) -> (f64,f64) { (volume(b),b.faces.iter().map(|f| area(b,f)).sum()) }

/// Body `body` of the series' document `name`, built by the export within the tolerance.
fn by_the_pipeline(name: &str,body: &str) -> (f64,f64) {
    let e = fixtures::read(&crate::solid_sweeps_examples::text(name));
    let say = Say {stage:&|_| {},mark:&|_| {}};
    let exact = gcs_core::brep::export::exact(&e.sketch,fixtures::solid(&e,body),None,Some(Tolerance::new(TOLERANCE).unwrap()),&say)
        .unwrap_or_else(|r| panic!("{r}"));
    measured(&exact.solid)
}

/// The blank cut by the tool under the motion over the roll, through the kernel's API alone.
fn by_the_api(blank: &Brep,tool: &Tool,motion: &Motion,roll: [f64;2]) -> (f64,f64) {
    let say = Say {stage:&|_| {},mark:&|_| {}};
    let cut = swept_cut_with(blank,tool,motion,roll,&Ask {tolerance:Some(TOLERANCE),..Ask::default()},&say).unwrap_or_else(|r| panic!("{r}"));
    cut.check(1e-6).unwrap();
    measured(&cut)
}

/// The two, each measure within its bar of the other's.
fn agree(name: &str,pipeline: (f64,f64),api: (f64,f64),bars: [f64;2]) {
    let off = |a: f64,b: f64| (a-b).abs()/b.abs();
    eprintln!("{name}: volume {:.12} by the pipeline, {:.12} by the API ({:.1e}); area {:.12}, {:.12} ({:.1e})",pipeline.0,api.0,
        off(api.0,pipeline.0),pipeline.1,api.1,off(api.1,pipeline.1));
    assert!(off(api.0,pipeline.0) <= bars[0] && off(api.1,pipeline.1) <= bars[1],"{name}: {api:?} by the API against {pipeline:?}");
}

/// A profile of the polygon through `points`, in the plane through `origin` square to `normal`.
fn polygon(origin: V,normal: V,points: &[V]) -> Profile {
    let n = points.len();
    Profile {origin,normal,names:Vec::new(),loops:vec![(0..n).map(|k| ProfileEdge::Line {a:points[k],b:points[(k+1)%n]}).collect()]}
}

/// A torus about the line through `centre` along `axis`: its tube, of radius `tube`, centred `ring`
/// from the axis along `out` (square to it).
fn torus(name: &str,centre: V,axis: V,out: V,ring: f64,tube: f64) -> Tool {
    let normal = gcs_core::space::cross(out,axis);
    let at = std::array::from_fn(|k| centre[k]+ring*out[k]);
    Tool::Revolved {name:name.into(),profile:disc(at,normal,tube),origin:centre,axis}
}

/// The rod the slide, the screw, the V wheel and the beads groove: radius 5 about z, 20 long.
fn rod() -> Brep { prism(&disc([0.;3],[0.,0.,1.],5.),0.,20.).unwrap() }

/// `slide.sv`: the wheel slid 40 mm up the rod over a turn of the roll.
#[test]
fn the_slide_agrees() {
    let wheel = torus("wheel",[11.5,0.,-9.],[0.,1.,0.],[-1.,0.,0.],6.,2.);
    let api = by_the_api(&rod(),&wheel,&Motion::Slide {direction:[0.,0.,1.],advance:40.},[0.,TAU]);
    agree("slide.sv",by_the_pipeline("slide.sv","rod"),api,[1e-8,1e-7]);
}

/// `union.sv`: the two beads, united, slid as `slide.sv`'s wheel is.
#[test]
fn the_union_agrees() {
    let beads = Tool::Union(vec![torus("bead",[11.5,0.,-9.],[0.,1.,0.],[-1.,0.,0.],6.,2.),
        torus("bead2",[11.5,1.5,-9.],[0.,1.,0.],[-1.,0.,0.],6.,1.5)]);
    let api = by_the_api(&rod(),&beads,&Motion::Slide {direction:[0.,0.,1.],advance:40.},[0.,TAU]);
    agree("union.sv",by_the_pipeline("union.sv","rod"),api,[1e-8,1e-7]);
}

/// The screw both screw documents turn their wheels by: about z, 30 mm a turn.
fn screw() -> Motion { Motion::Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:30.} }

/// `screw.sv`: the wheel leaning by the helix's angle at 5.5 mm, screwed from below the rod to above.
#[test]
fn the_screw_agrees() {
    let h = (TAU*5.5/30.).atan();
    let wheel = torus("wheel",[11.5,0.,0.],[0.,h.cos(),-h.sin()],[-1.,0.,0.],6.,2.);
    let api = by_the_api(&rod(),&wheel,&screw(),[-108f64.to_radians(),348f64.to_radians()]);
    agree("screw.sv",by_the_pipeline("screw.sv","rod"),api,[1e-7,1e-6]);
}

/// `vthread.sv`: the V wheel, its rim 4 mm from the rod's axis, screwed as `screw.sv`'s wheel is.
#[test]
fn the_v_thread_agrees() {
    let h = (TAU*4.5/30.).atan();
    let along = [0.,h.cos(),-h.sin()];
    let side = 2.5*(PI/6.).tan();
    let corner = |s: f64| [6.5,s*along[1],s*along[2]];
    let vwheel = Tool::Revolved {name:"vwheel".into(),profile:polygon([4.,0.,0.],[0.,h.sin(),h.cos()],&[[4.,0.,0.],corner(-side),corner(side)]),
        origin:[14.,0.,0.],axis:along};
    let api = by_the_api(&rod(),&vwheel,&screw(),[-144f64.to_radians(),384f64.to_radians()]);
    agree("vthread.sv",by_the_pipeline("vthread.sv","rod"),api,[1e-7,1e-6]);
}

/// `turn.sv`: the leaning torus turned about z across the ring sector, from −140° to 140°.
#[test]
fn the_turn_agrees() {
    let ring = revolve(&polygon([0.;3],[0.,-1.,0.],&[[8.,0.,-1.],[12.,0.,-1.],[12.,0.,1.],[8.,0.,1.]]),[0.;3],[0.,0.,1.],TAU).unwrap();
    let wedge = prism(&polygon([0.,0.,-2.],[0.,0.,1.],&[[0.,0.,-2.],[24.,0.,-2.],[12.,12.*3f64.sqrt(),-2.]]),0.,4.).unwrap();
    let block = combined(&ring,&wedge,Op::Common,0.).unwrap();
    let lean = 25f64.to_radians();
    let cutter = torus("cutter",[5.,0.,0.],[0.,lean.sin(),lean.cos()],[1.,0.,0.],2.5,2.);
    let api = by_the_api(&block,&cutter,&Motion::Turn {origin:[0.;3],axis:[0.,0.,1.],ratio:1.},[-140f64.to_radians(),140f64.to_radians()]);
    agree("turn.sv",by_the_pipeline("turn.sv","block"),api,[1e-7,1e-6]);
}

/// `rack.sv`: the rack's tooth slid along the pitch line seen from the blank turning under it, the
/// pitch line rolling on the pitch circle of 20 mm, from 60° one side to 60° the other. The pipeline
/// extrudes the tooth's planar envelope, the API traces it: they agree to the traced sheet's fit.
#[test]
fn the_rack_agrees() {
    let blank = prism(&disc([0.;3],[0.,1.,0.],22.),0.,6.).unwrap();
    let tooth = Tool::Prism {name:"tooth".into(),profile:polygon([0.;3],[0.,1.,0.],
        &[[18.,0.,-0.842856],[18.,0.,0.842856],[24.,0.,3.026666],[24.,0.,-3.026666]]),from:-2.,to:8.,draft:None};
    let rack = Motion::Slide {direction:[0.,0.,1.],advance:TAU*20.}.seen_from(Motion::Turn {origin:[0.;3],axis:[0.,-1.,0.],ratio:1.});
    let api = by_the_api(&blank,&tooth,&rack,[-60f64.to_radians(),60f64.to_radians()]);
    agree("rack.sv",by_the_pipeline("rack.sv","gear"),api,[5e-6,5e-6]);
}
