//! **A prism under a motion that keeps a plane, swept closed** (#880 Q2, `brep::swept::planar_solid`):
//! the tool's section's planar sweep, read over the whole roll with no blank, extruded through the
//! tool's slabs. Held on brep-bench's three cases of the kind — `square-on-a-planet-carrier`,
//! `rack-cut-tooth-space` and `plus-cut-wheel` — to what is known apart from the build: the rack's
//! tooth space to the volume two other kernels agree on, and the other two swept solids to the
//! area the sweep's definition gives in the plane, read line by line on the exact section over
//! dense poses; each judged by membership before it is returned, and written as a closed STEP
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

/// **The area the polygon `corners` sweeps in its plane, read line by line** — the definition, read
/// apart from the kernel on the exact section: the polygon at each of `poses` poses over `roll`
/// (`pose(t)`: the pose's turn and shift, `x ↦ R x + c`) and, between each two, each edge's
/// quadrilateral from one placing to the next (a bow where the edge passes the instant centre, its two
/// triangles), so that every point of an edge moves along a chord of its path, the sweep held to the
/// square of the step — the placings alone would leave each corner's trace notched to the step
/// itself; on each of `lines` lines across `[y0, y1]` (at their middles) the union of the intervals
/// these cover, by crossing parity, its length integrated by the midpoint rule.
fn area_by_lines(corners: &[[f64;2]],pose: &(dyn Fn(f64) -> ([[f64;2];2],[f64;2])+Sync),roll: [f64;2],poses: usize,
    [y0,y1]: [f64;2],lines: usize) -> f64 {
    let placed: Vec<Vec<[f64;2]>> = (0..=poses).map(|k| {
        let (r,c) = pose(roll[0]+(roll[1]-roll[0])*k as f64/poses as f64);
        corners.iter().map(|p| [r[0][0]*p[0]+r[0][1]*p[1]+c[0],r[1][0]*p[0]+r[1][1]*p[1]+c[1]]).collect()
    }).collect();
    let n = corners.len();
    let mut shapes: Vec<Vec<[f64;2]>> = placed.clone();
    for w in placed.windows(2) {
        for k in 0..n { shapes.push(vec![w[0][k],w[0][(k+1)%n],w[1][(k+1)%n],w[1][k]]) }
    }
    let reach: Vec<[f64;2]> = shapes.iter().map(|h| h.iter().fold([f64::INFINITY,f64::NEG_INFINITY],|[lo,hi],p| [lo.min(p[1]),hi.max(p[1])])).collect();
    let ys: Vec<f64> = (0..lines).map(|k| y0+(y1-y0)*(k as f64+0.5)/lines as f64).collect();
    let lengths = gcs_core::par::map(&ys,|&y| {
        let mut spans: Vec<[f64;2]> = Vec::new();
        let mut xs: Vec<f64> = Vec::new();
        for (h,&[lo,hi]) in shapes.iter().zip(&reach) {
            if y <= lo || y >= hi { continue }
            xs.clear();
            for k in 0..h.len() {
                let (a,b) = (h[k],h[(k+1)%h.len()]);
                if (a[1] > y) != (b[1] > y) { xs.push(a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1])) }
            }
            xs.sort_by(f64::total_cmp);
            spans.extend(xs.chunks_exact(2).map(|w| [w[0],w[1]]));
        }
        spans.sort_by(|a,b| a[0].total_cmp(&b[0]));
        let (mut total,mut open): (f64,Option<[f64;2]>) = (0.,None);
        for s in spans {
            open = match open {
                Some(o) if s[0] <= o[1] => Some([o[0],o[1].max(s[1])]),
                Some(o) => { total += o[1]-o[0]; Some(s) }
                None => Some(s),
            };
        }
        total+open.map_or(0.,|o| o[1]-o[0])
    });
    lengths.iter().sum::<f64>()*(y1-y0)/lines as f64
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
/// through its height, held to the definition's area in the plane, read line by line on the exact
/// square.
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
    let area = area_by_lines(&square,&pose,[0.,TAU],9000,[-7.5,7.5],12000);
    let v = volume(&built);
    eprintln!("planet: the area read line by line {area:.6}, the build's {:.6}",v/2.);
    assert!((v/2./area-1.).abs() <= 1e-6,"the build's area {} against the definition's {area}",v/2.);
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
    let area = area_by_lines(&section,&pose,[0.,3.*PI],9000,[-85.,85.],16000);
    let v = volume(&built);
    eprintln!("plus: the area read line by line {area:.6}, the build's {:.6}",v/10.);
    assert!((v/10./area-1.).abs() <= 1e-6,"the build's area {} against the definition's {area}",v/10.);
}
