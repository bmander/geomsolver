//! **A prism under a motion that keeps a plane, swept closed** (#880 Q2, `brep::swept::planar_solid`):
//! the tool's section's planar sweep, read over the whole roll with no blank, extruded through the
//! tool's slabs. Held on brep-bench's three cases of the kind — `square-on-a-planet-carrier`,
//! `rack-cut-tooth-space` and `plus-cut-wheel` — to what is known apart from the build: the rack's
//! tooth space to the volume two other kernels agree on, and each swept solid to a Monte Carlo of
//! the sweep's definition, read in the plane on the exact section over poses the point's speed
//! bounds between; each judged by membership before it is returned, and written as a closed STEP
//! B-rep.
use gcs_core::brep::boolean::{combined,Op};
use gcs_core::brep::build::{disc,prism,Profile,ProfileEdge};
use gcs_core::brep::geom::tolerance;
use gcs_core::brep::props::volume;
use gcs_core::brep::swept::closed::closed;
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Screw,Trajectory};
use std::f64::consts::{PI,TAU};

type V = [f64;3];

/// The prism on the polygon `corners` (counter-clockwise seen from `+normal`'s side, in the plane
/// through the first), from it to `depth` along the normal.
fn polygon_prism(corners: &[V],normal: V,depth: f64) -> Brep {
    let n = corners.len();
    let edges = (0..n).map(|k| ProfileEdge::Line {a:corners[k],b:corners[(k+1)%n]}).collect();
    prism(&Profile {origin:corners[0],normal,loops:vec![edges],names:Vec::new()},0.,depth).unwrap()
}

/// A turn about the line through `origin` along `axis`, `ratio` radians a radian.
fn turn(origin: V,axis: V,ratio: f64) -> Trajectory { Trajectory::Twist(Screw {origin,axis,ratio,advance:0.}) }

/// `source` seen from `observer`.
fn seen(source: Trajectory,observer: Trajectory) -> Trajectory {
    Trajectory::Relative {source:Box::new(source),observer:Box::new(observer)}
}

/// The polygon `corners` (in its plane's coordinates) carried back by the poses of a planar motion
/// at many rolls: whether the page point `p` lies in it at some roll — the definition, read on the
/// exact section. The section's signed distance carried back is one-Lipschitz in the point and moves
/// no faster than `speed` a radian, so between two readings a stretch is settled out where the lesser
/// less `speed` times half the stretch is positive, and halved otherwise.
fn swept_polygon(corners: &[[f64;2]],pose: &dyn Fn(f64) -> ([[f64;2];2],[f64;2]),roll: [f64;2],speed: f64,p: [f64;2]) -> bool {
    let distance = |q: [f64;2]| -> f64 {
        let n = corners.len();
        let mut d = f64::INFINITY;
        let mut inside = false;
        for k in 0..n {
            let (a,b) = (corners[k],corners[(k+1)%n]);
            let (e,w) = ([b[0]-a[0],b[1]-a[1]],[q[0]-a[0],q[1]-a[1]]);
            let t = ((w[0]*e[0]+w[1]*e[1])/(e[0]*e[0]+e[1]*e[1])).clamp(0.,1.);
            d = d.min((w[0]-t*e[0]).hypot(w[1]-t*e[1]));
            if (a[1] > q[1]) != (b[1] > q[1]) && q[0] < a[0]+(q[1]-a[1])*(b[0]-a[0])/(b[1]-a[1]) { inside = !inside }
        }
        if inside { -d } else { d }
    };
    let read = |t: f64| { let (r,c) = pose(t); let w = [p[0]-c[0],p[1]-c[1]]; distance([r[0][0]*w[0]+r[1][0]*w[1],r[0][1]*w[0]+r[1][1]*w[1]]) };
    let n = 256;
    let ts: Vec<f64> = (0..=n).map(|k| roll[0]+(roll[1]-roll[0])*k as f64/n as f64).collect();
    let vs: Vec<f64> = ts.iter().map(|&t| read(t)).collect();
    if vs.iter().any(|&v| v <= 0.) { return true }
    let mut stack: Vec<(f64,f64,f64,f64)> = (0..n).map(|k| (ts[k],ts[k+1],vs[k],vs[k+1])).collect();
    while let Some((a,b,va,vb)) = stack.pop() {
        if va.min(vb)-speed*(b-a)/2. > 0. { continue }
        if b-a < 1e-12 { return true }
        let m = 0.5*(a+b);
        let vm = read(m);
        if vm <= 0. { return true }
        stack.push((a,m,va,vm));
        stack.push((m,b,vm,vb));
    }
    false
}

/// The area of the region a polygon sweeps, by a Monte Carlo of `swept_polygon` over the box `lo`–
/// `hi`: the estimate and its standard error.
fn area_by_definition(lo: [f64;2],hi: [f64;2],n: usize,holds: &(dyn Fn([f64;2]) -> bool+Sync)) -> (f64,f64) {
    let mut rng = gcs_core::rng::Rng::new(0x9_1880);
    let points: Vec<[f64;2]> = (0..n).map(|_| [rng.uniform(lo[0],hi[0]),rng.uniform(lo[1],hi[1])]).collect();
    let inside = gcs_core::par::map(&points,|&p| holds(p)).into_iter().filter(|&h| h).count() as f64;
    let (size,share) = ((hi[0]-lo[0])*(hi[1]-lo[1]),inside/n as f64);
    (size*share,size*(share*(1.-share)/n as f64).sqrt())
}

/// A swept solid's checks apart from its volume: valid, closed, and its STEP file the solid's.
fn holds_as_a_solid(b: &Brep,name: &str) {
    b.check(tolerance(b.size())).unwrap();
    let text = gcs_core::brep::step::write(b,name,1e-7).unwrap();
    gcs_core::brep::step_check::verify(&text,&gcs_core::brep::step_check::Solid::of(b)).unwrap();
}

/// **The square bar carried round on a planet is swept closed** (brep-bench
/// `square-on-a-planet-carrier`, #721 K3): the prism from z = 0 to 2 on the square of side 2 about
/// (5, 0), carried round the circle of radius 5 about z by t while it turns about its own centre by
/// −2t, over a whole turn. Before Q2 it took K2's road and was refused at a corner (`Curved`). Its
/// section's region — the envelopes of its sides and the trochoids its corners trace — extruded
/// through its height, held to the definition's area in the plane, read on the exact square.
#[test]
fn the_square_on_a_planet_carrier_is_swept_closed() {
    let bar = polygon_prism(&[[4.,-1.,0.],[6.,-1.,0.],[6.,1.,0.],[4.,1.,0.]],[0.,0.,1.],2.);
    let up = [0.,0.,1.];
    let path = seen(turn([5.,0.,0.],up,-3.),turn([0.;3],up,-1.));
    let clock = std::time::Instant::now();
    let built = closed(&bar,&path,[0.,TAU],0.01,tolerance(bar.size())).unwrap();
    eprintln!("planet: built in {:?}, {} faces, volume {:.9}",clock.elapsed(),built.faces.len(),volume(&built));
    holds_as_a_solid(&built,"planet");
    // in the plane: the square about (5, 0) turned by −3t about its centre, then by t about the
    // origin; a point of the page within 9 of the origin moves at most 2·9 + 5·3 a radian seen
    // from the square
    let square = [[4.,-1.],[6.,-1.],[6.,1.],[4.,1.]];
    let pose = |t: f64| {
        // x ↦ R(t) (R(−3t)(x − c) + c): back, y ↦ R(3t)(R(−t) y − c) + c
        let (s,c) = (-2.*t).sin_cos();
        let r = [[c,-s],[s,c]];
        let (s1,c1) = t.sin_cos();
        let shift = [5.*c1-(r[0][0]*5.),5.*s1-(r[1][0]*5.)];
        (r,shift)
    };
    let (area,err) = area_by_definition([-7.5,-7.5],[7.5,7.5],200_000,&|p| swept_polygon(&square,&pose,[0.,TAU],40.,p));
    let v = volume(&built);
    eprintln!("planet: definition's area {area:.4} ± {err:.4}, the build's {:.6}",v/2.);
    assert!((v/2.-area).abs() <= 4.*err,"the build's area {} against the definition's {area} ± {err}",v/2.);
}

/// **The rack's tooth space, its sweep built closed and cut by the Boolean** (brep-bench
/// `rack-cut-tooth-space`, settled): module 2, 20 teeth, 20° pressure angle; the tooth's prism rolled
/// on the pitch circle from 60° on one side of the pitch point to 60° on the other, seen from the
/// blank. The swept solid is the trapezoid's planar sweep through the tooth's length — its flanks
/// the envelopes of the rack's (involutes of the base circle), its root the trochoids of its tip
/// corners, and its end poses — and the blank less it is held to the volume next.BREP and cadrs
/// agree on to 8e-11, at a millionth.
#[test]
fn the_rack_cut_tooth_space_is_cut_by_the_closed_sweep() {
    // the trapezoid in the plane y = −2, walked counter-clockwise seen from +y, extruded to y = 8
    let (tip,root) = (PI/2.-2.*20f64.to_radians().tan(),PI/2.+4.*20f64.to_radians().tan());
    let tooth = polygon_prism(&[[18.,-2.,tip],[24.,-2.,root],[24.,-2.,-root],[18.,-2.,-tip]],[0.,1.,0.],10.);
    let blank = prism(&disc([0.;3],[0.,1.,0.],22.),0.,6.).unwrap();
    // the blank turns about y while the rack slides along the pitch line x = 20, 2π·20 a turn the
    // way the blank's rim moves there: seen from the blank, the rack rolls
    let rack = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:0.,advance:-TAU*20.});
    let path = seen(rack,turn([0.;3],[0.,1.,0.],1.));
    let roll = [-PI/3.,PI/3.];
    let clock = std::time::Instant::now();
    let swept = closed(&tooth,&path,roll,0.01,tolerance(tooth.size())).unwrap();
    eprintln!("rack: built in {:?}, {} faces, volume {:.9}",clock.elapsed(),swept.faces.len(),volume(&swept));
    holds_as_a_solid(&swept,"rack");
    let cut = combined(&blank,&swept,Op::Cut,0.).unwrap();
    let v = volume(&cut);
    eprintln!("rack: the blank less the sweep {v:.9} ({:?})",clock.elapsed());
    assert!((v/9042.502788955-1.).abs() <= 1e-6,"the tooth space's blank {v} against 9042.502788955");
}

/// **The wheel's sweep by the plus in mesh with it, built closed** (brep-bench `plus-cut-wheel`):
/// the plus turning about z by t while the wheel turns by −2t/3 about the vertical line through
/// (−50, 0), over t from 0 to 540° — a whole period, so no end pose bounds it. Seen from the wheel the
/// plus rolls in its plane; its section's region is an annulus about the wheel's axis whose hole is
/// the wheel left standing, held to the definition's area in the plane.
#[test]
fn the_plus_cut_wheel_is_swept_closed() {
    let (a,w) = (30.,4.);
    let plus: Vec<V> = [[a,-w],[a,w],[w,w],[w,a],[-w,a],[-w,w],[-a,w],[-a,-w],[-w,-w],[-w,-a],[w,-a],[w,-w]].iter()
        .map(|p| [p[0],p[1],-10.]).collect();
    let tool = polygon_prism(&plus,[0.,0.,1.],10.);
    let up = [0.,0.,1.];
    let path = seen(turn([0.;3],up,1.),turn([-50.,0.,0.],up,-2./3.));
    let clock = std::time::Instant::now();
    let built = closed(&tool,&path,[0.,3.*PI],0.01,tolerance(tool.size())).unwrap();
    eprintln!("plus: built in {:?}, {} faces, volume {:.9}",clock.elapsed(),built.faces.len(),volume(&built));
    holds_as_a_solid(&built,"plus");
    let section: Vec<[f64;2]> = plus.iter().map(|p| [p[0],p[1]]).collect();
    // seen from the wheel: x ↦ R(−φ)(R(t) x − c) + c, φ = −2t/3, c = (−50, 0)
    let pose = |t: f64| {
        let phi = -2.*t/3.;
        let (s,c) = (t-phi).sin_cos();
        let r = [[c,-s],[s,c]];
        let (sp,cp) = (-phi).sin_cos();
        // R(−φ)(−c) + c
        let shift = [cp*50.-50.,sp*50.];
        (r,shift)
    };
    let (area,err) = area_by_definition([-135.,-85.],[35.,85.],200_000,&|p| swept_polygon(&section,&pose,[0.,3.*PI],200.,p));
    let v = volume(&built);
    eprintln!("plus: definition's area {area:.4} ± {err:.4}, the build's {:.6}",v/10.);
    assert!((v/10.-area).abs() <= 4.*err,"the build's area {} against the definition's {area} ± {err}",v/10.);
}
