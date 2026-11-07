//! **A sweep does not depend on where a chart starts** (#721 F1d): a face of revolution's chart
//! starts somewhere round its turn, and nothing the sweep reads may depend on where. A tool is read
//! from its B-rep (`Tool::read`) with each revolution's chart starting where its own surface's
//! frame does, not in a half-plane shared by every face about its axis; its corners are probed
//! with the second face at the first's half-plane (`Tool::corner`), faces of one surface are told
//! by their meridians about one axis (`read::once_a_surface`), and the walk crosses from face to
//! face by position. Held here in the kernel alone: one tool drawn with its profile in two
//! half-planes, so its B-rep's charts start in each, and its characteristic walked by the dispatch
//! held to one curve; and the reading's charts started further round again
//! (`RevolvedSurface::started`), the walk the same.
use gcs_core::brep::build::{revolve,Profile,ProfileEdge};
use gcs_core::brep::geom::Frame;
use gcs_core::brep::swept::admission::{check_screw,Generating,Sampling};
use gcs_core::brep::swept::constant_twist::Characteristic;
use gcs_core::brep::swept::dispatch::{self,Ask,Blank,Class};
use gcs_core::brep::swept::subtraction::Names;
use gcs_core::brep::swept::tool::{Tool,ToolSurface};
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Motion,Screw,Trajectory};

type V = [f64;3];

/// The plate x −10 to 10, y −5 to 5, z 0 to 4.
pub(crate) struct Plate;

impl Blank for Plate {
    fn inside(&self,p: V) -> bool { p[0].abs() < 10. && p[1].abs() < 5. && p[2] > 0. && p[2] < 4. }
    fn extent(&self) -> Option<(V,V)> { Some(([-10.,-5.,0.],[10.,5.,4.])) }
    fn boxed(&self) -> Option<(V,V)> { self.extent() }
}

/// A drum of radius 2 lying along x, 8 long about (0, 0, −6), its flat ends rounded into its side
/// by quarter rounds of radius 0.5 (`closed_characteristics::bull_nosed_punch`'s), its profile laid
/// in the half-plane `angle` radians round x from +z.
pub(crate) fn drum(angle: f64) -> Brep { drum_turned(angle,std::f64::consts::TAU) }

/// `drum`'s profile laid `angle` round, turned through `sweep` from there.
pub(crate) fn drum_turned(angle: f64,sweep: f64) -> Brep {
    let (s,c) = angle.sin_cos();
    let d = [0.,-s,c];
    let at = |a: f64,r: f64| -> V { [a,r*d[1],-6.+r*d[2]] };
    let normal = [0.,-d[2],d[1]];
    let line = |a: V,b: V| ProfileEdge::Line {a,b};
    // a quarter round about (a, 1.5) from the flat end's to the side's, `x` out along the axis
    let round = |a: f64,out: f64| ProfileEdge::Arc {frame:Frame::new(at(a,1.5),normal,[out,0.,0.]),r:0.5,
        span:Some(if out > 0. { [0.,std::f64::consts::FRAC_PI_2] } else { [-std::f64::consts::FRAC_PI_2,0.] })};
    let profile = Profile {origin:at(0.,0.),normal,loops:vec![vec![line(at(-4.,0.),at(-4.,1.5)),round(-3.5,-1.),
        line(at(-3.5,2.),at(3.5,2.)),round(3.5,1.),line(at(4.,1.5),at(4.,0.)),line(at(4.,0.),at(-4.,0.))]],names:Vec::new()};
    revolve(&profile,[0.,0.,-6.],[1.,0.,0.],sweep).unwrap()
}

/// The push up along z, 16 over a turn of the roll.
pub(crate) fn push() -> Trajectory { Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:0.,advance:16.}) }

/// The drum's characteristic as the dispatch walks it, pushed through the plate.
fn dispatched(tool: &Brep) -> Characteristic {
    let names = Names {sweep:"slot".into(),tool:"drum".into(),motion:"push".into()};
    let path = push();
    let s = dispatch::Sweep {tool:Ok(tool),path:&path,roll:[0.,std::f64::consts::TAU],mm:1.,names:&names,pocketed:false};
    let a = dispatch::sweep(&s,&Plate,&Ask {poses:&[Motion::identity()],revolved:None,sampling:Sampling::default(),margin:1e-6})
        .unwrap_or_else(|r| panic!("{:?}: {:?}",r.row,r.seen));
    let Class::ConstantTwist(found) = a.class else { panic!("the constant twist's class: {:?}",a.class) };
    *found
}

/// The distance from `p` to the polyline through `nodes`.
fn off(p: V,nodes: &[V]) -> f64 {
    nodes.windows(2).map(|w| {
        let d: V = std::array::from_fn(|k| w[1][k]-w[0][k]);
        let q: V = std::array::from_fn(|k| p[k]-w[0][k]);
        let dd: f64 = d.iter().map(|x| x*x).sum();
        let t = if dd > 0. { (q.iter().zip(&d).map(|(a,b)| a*b).sum::<f64>()/dd).clamp(0.,1.) } else { 0. };
        (0..3).map(|k| (q[k]-t*d[k]).powi(2)).sum::<f64>().sqrt()
    }).fold(f64::INFINITY,f64::min)
}

/// The greatest distance from a node of either walk to the other's polyline, and the longest step
/// between two nodes of either.
fn apart(a: &Characteristic,b: &Characteristic) -> (f64,f64) {
    let all = |c: &Characteristic| -> Vec<V> { c.nodes.iter().map(|n| n.position).collect() };
    let gap = all(a).iter().map(|&p| off(p,&all(b))).chain(all(b).iter().map(|&p| off(p,&all(a)))).fold(0.,f64::max);
    let step = [a,b].iter().flat_map(|c| c.nodes.windows(2).map(|w| {
        (0..3).map(|k| (w[1].position[k]-w[0].position[k]).powi(2)).sum::<f64>().sqrt()
    })).fold(0.,f64::max);
    (gap,step)
}

/// Two walks of one tool's characteristic held to one curve: each closed, creased alike, and each
/// node within a millionth of a step of the other's polyline.
pub(crate) fn alike(what: &str,a: &Characteristic,b: &Characteristic) {
    let (gap,step) = apart(a,b);
    eprintln!("{what}: {} nodes against {}, {gap:.3e} apart, steps up to {step:.3e}",b.nodes.len(),a.nodes.len());
    assert_eq!((a.closed,a.sharp().len()),(b.closed,b.sharp().len()),"{what}: closed, and its creases");
    assert!(gap <= 1e-6*step.max(1.),"{what}: the walks lie {gap:.3e} apart, steps up to {step:.3e}");
}

/// **The walk is the same curve wherever a chart starts**: the bull-nosed drum pushed edge first
/// through the plate, its characteristic closed on itself through it (#684 W) and creased at four
/// corners, walked by the dispatch with its profile drawn in three half-planes, so its B-rep's
/// charts start in each; then the first reading walked again with each face of revolution's chart
/// started a different way round. Every walk is one curve.
#[test]
fn a_walk_is_the_same_curve_wherever_its_charts_start() {
    let first = drum(0.);
    let base = dispatched(&first);
    assert!(base.closed,"the characteristic closes on itself through the plate");
    for angle in [1.1,-2.4] { alike(&format!("drawn {angle} round"),&base,&dispatched(&drum(angle))) }
    let tool = Tool::read(&first,1.).unwrap();
    let revolutions = tool.faces.iter().filter(|f| matches!(f,ToolSurface::Revolved(_))).count();
    assert!(revolutions >= 5,"the drum's flats, rounds and side: {revolutions} faces of revolution");
    let path = push();
    let walk = |tool: &Tool| {
        let sweep = Generating {tool,path:&path,roll:[0.,std::f64::consts::TAU],whole:false,mm:1.};
        let extent = Some(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:0.,advance:16.}.extent([-10.,-5.,0.],[10.,5.,4.],Motion::identity()));
        check_screw(&sweep,&|p| Plate.inside(p),extent,&Sampling::default(),None).unwrap_or_else(|e| panic!("the walk: {e:?}")).0
    };
    alike("read again",&base,&walk(&tool));
    for phase in [0.7,2.9] {
        let mut turned = tool.clone();
        for (k,f) in turned.faces.iter_mut().enumerate() {
            if let ToolSurface::Revolved(s) = f { *s = s.started(phase+1.9*k as f64) }
        }
        alike(&format!("started {phase} round"),&base,&walk(&turned));
    }
}
