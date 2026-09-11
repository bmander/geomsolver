//! Milestone 4: sheets that cross. Where one sheet turns inner across
//! another, the kept parts must meet along the crease between them, and the
//! shell must still close and certify.
use super::{closed::{closed,closed_shell_at,volume},harness::{self,V3},motions,tools};
use gcs_core::{model::SolidDef,motion::Family};

const SAGITTA: f64 = 0.02;

/// The swept volume by sampled membership: a midpoint grid over the box,
/// each point material when the tool's closed-form membership holds at any
/// of `poses` parameters. Uses only `Family::at`, never the field.
fn sampled_volume(source: &str,member: &dyn Fn(V3) -> bool,lo: V3,hi: V3,grid: usize,poses: usize) -> f64 {
    let e = harness::read(source);
    let swept = harness::solid(&e,"swept");
    let SolidDef::Swept {motion,from,to,..} = &e.sketch.solids[swept].def else { panic!("not a sweep") };
    let family = Family::read(&e.sketch,*motion as usize).unwrap();
    let inverses: Vec<_> = (0..=poses).map(|k| family.at(from.value+(to.value-from.value)*k as f64/poses as f64).unwrap().inverse()).collect();
    let step: V3 = std::array::from_fn(|k| (hi[k]-lo[k])/grid as f64);
    let mut count = 0usize;
    for i in 0..grid { for j in 0..grid { for k in 0..grid {
        let p = [lo[0]+(i as f64+0.5)*step[0],lo[1]+(j as f64+0.5)*step[1],lo[2]+(k as f64+0.5)*step[2]];
        if inverses.iter().any(|m| member(m.point(p))) { count += 1; }
    } } }
    count as f64*step[0]*step[1]*step[2]
}

/// The area a plane region sweeps turned by `alpha` either way about the
/// origin, by ring quadrature: on each circle about the pivot (out to
/// `r_max`) the region's arcs, grown by the turn and their union measured,
/// at a resolution of `steps` around the circle.
fn turned_area(member: &dyn Fn(f64,f64) -> bool,r_max: f64,alpha: f64) -> f64 {
    let (rings,steps) = (1000,3600);
    let reach = (alpha/2./(std::f64::consts::TAU/steps as f64)).round() as usize;
    let mut area = 0.;
    for i in 0..rings {
        let r = r_max*(i as f64+0.5)/rings as f64;
        let on: Vec<bool> = (0..steps).map(|k| { let t = std::f64::consts::TAU*k as f64/steps as f64; member(r*t.cos(),r*t.sin()) }).collect();
        // grown by the turn: covered where some point of the region is
        // within reach, read off a running count round the circle
        let mut prefix = vec![0usize;3*steps+1];
        for k in 0..3*steps { prefix[k+1] = prefix[k]+on[k%steps] as usize; }
        let covered = (0..steps).filter(|&k| prefix[steps+k+reach+1] > prefix[steps+k-reach]).count();
        area += covered as f64/steps as f64*std::f64::consts::TAU*r*r_max/rings as f64;
    }
    area
}

#[test]
fn a_turning_prism_closes_along_its_creases() {
    // the prism's section (3, -0.8), (4.5, 0), (3, 0.8) (in x and z) extruded
    // 3 along y, turned 100° about the vertical through (2.5, 0): at each
    // height its section is a rectangle, swept by ring quadrature
    let source = format!("{}{}{}",tools::TRIANGLE_PRISM,motions::TURN_OFFSET,motions::swept("turn",-50.,50.));
    let (mesh,certificate) = closed_shell_at(&source,SAGITTA);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    let alpha = 100_f64.to_radians();
    let layers = 100;
    let expected: f64 = (0..layers).map(|j| {
        let z = -0.8+1.6*(j as f64+0.5)/layers as f64;
        let x_far = 4.5-1.875*z.abs();
        turned_area(&|x,y| x >= 0.5 && x <= x_far-2.5 && y.abs() <= 1.5,((x_far-2.5).powi(2)+2.25).sqrt(),alpha)*1.6/layers as f64
    }).sum();
    let v = volume(&mesh);
    eprintln!("turning prism: volume {v:.4}, expected {expected:.4}");
    assert!(v <= expected*(1.+1e-3) && v >= expected*(1.-3.*SAGITTA/0.5),"volume {v} against {expected}");
}

/// Not closed yet (milestone 5, the tracer): the sphere the stationary
/// ring sweeps is inner everywhere but at second order (within a quarter
/// sagitta of the boundary over a patch a fifth wide about each fixed
/// point of the axis, which the labels keep), the generators through the
/// fixed points sweep bowtie sectors whose halves face opposite ways, and
/// the rims' sweeps fold where a rim's tangent runs along its velocity.
/// Runs in four seconds and leaves nine loops and a few dozen refused
/// triangles; `SOLVENT_SHEETS=1` prints the stages.
#[test]
#[ignore]
fn a_tumbling_cylinder_closes_along_its_creases() {
    // radius 1, height 2 about the vertical through (3, 0), tumbled ±30°
    // about the horizontal line through its centre
    let source = format!("{}{}{}",tools::CYLINDER,motions::TUMBLE,motions::swept("turn",-30.,30.));
    let (mesh,certificate) = closed_shell_at(&source,SAGITTA);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    let member = |p: V3| (p[0]-3.).powi(2)+p[1]*p[1] <= 1. && p[2].abs() <= 1.;
    let expected = sampled_volume(&source,&member,[1.5,-1.5,-1.5],[4.5,1.5,1.5],64,256);
    let v = volume(&mesh);
    eprintln!("tumbling cylinder: volume {v:.4}, sampled {expected:.4}");
    assert!((v-expected).abs() <= 0.03*expected,"volume {v} against sampled {expected}");
}

/// Not closed yet: the end faces' edges sweep in their own plane, and a
/// segment turning in its plane folds at its envelope arc (consecutive
/// columns cross), so the planar union is right but its slivers along the
/// arc are thinner than the split's tolerance, and the split and the zip
/// lay fills over them. One refused triangle and one two-vertex loop are
/// left; the volume is within a percent.
#[test]
#[ignore]
fn a_box_turned_about_its_face_centre_closes_along_its_creases() {
    // the 2 x 3 x 2 box turned 30° about the axis through the centre of its
    // x = 4 face along x: a 2 x 3 rectangle turned about its own centre
    // (the corners' arcs cross the faces and the faces turn inner), times 2
    let source = format!("{}{}{}",tools::BOX,motions::turn_about(4.,-1.5,5.,-1.5),motions::swept("turn",0.,30.));
    let (mesh,certificate) = closed_shell_at(&source,SAGITTA);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    let expected = 2.*turned_area(&|y,z| y.abs() <= 1. && z.abs() <= 1.5,(1_f64+2.25).sqrt(),30_f64.to_radians());
    let v = volume(&mesh);
    eprintln!("turned box: volume {v:.4}, expected {expected:.4}");
    assert!(v <= expected*(1.+1e-3) && v >= expected*(1.-3.*SAGITTA/1.),"volume {v} against {expected}");
}

#[test]
fn a_lens_turned_about_the_spindle_closes_along_its_crease() {
    // the lens of two unit spheres 0.8 apart, turned ±60° about the spindle:
    // its convex crease circle sweeps a fan
    let source = format!("{}{}{}",tools::LENS,motions::TURN_SPINDLE,motions::swept("turn",-60.,60.));
    let (mesh,certificate) = closed_shell_at(&source,SAGITTA);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    // every circle about the spindle meets the lens in one arc: at height z
    // it spans r from 3.8 - ρ to 3 + ρ (ρ the spheres' section radius), so
    // the sweep adds α (r_max² - r_min²) / 2 to the lens's own section
    let alpha = 120_f64.to_radians();
    let (h,layers) = (0.6_f64*(2.-0.6)/1_f64,2000);
    let h = h.sqrt(); // the crease's radius: sqrt(1 - 0.4²)
    let expected: f64 = (0..layers).map(|j| {
        let z = -h+2.*h*(j as f64+0.5)/layers as f64;
        let rho = (1.-z*z).sqrt();
        let (r_min,r_max) = (3.8-rho,3.+rho);
        // the section: two circular segments of the disks of radius ρ, cut 0.4 from their centres
        let segment = |d: f64| rho*rho*(d/rho).acos()-d*(rho*rho-d*d).sqrt();
        (2.*segment(0.4)+alpha*(r_max*r_max-r_min*r_min)/2.)*2.*h/layers as f64
    }).sum();
    let v = volume(&mesh);
    eprintln!("turned lens: volume {v:.4}, expected {expected:.4}");
    assert!(v <= expected*(1.+1e-3) && v >= expected*(1.-3.*SAGITTA/0.5),"volume {v} against {expected}");
}

/// Not closed yet (project 2, the Boolean meshes): the union's mesh is
/// forty-five thousand facets of the bar's wall shredded by the balls'
/// facet planes, so the caps take most of a minute and their slivers leave
/// four loops at the crease junctions and twenty-odd refused triangles.
#[test]
#[ignore]
fn a_dumbbell_translated_along_x_closes_along_its_creases() {
    // balls of radius 0.5 on a bar of radius 0.25, advanced 4 along x: the
    // two concave crease circles sweep, and every line along x meets the
    // tool in one segment
    let source = format!("{}{}{}",tools::DUMBBELL,motions::slide_x(4.),motions::swept("feed",0.,360.));
    let (mesh,certificate) = closed_shell_at(&source,SAGITTA);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    let member = |p: V3| ((p[0]-3.).powi(2)+p[1]*p[1] <= 0.0625 && p[2].abs() <= 1.) || (p[0]-3.).powi(2)+p[1]*p[1]+(p[2]-1.).powi(2) <= 0.25 || (p[0]-3.).powi(2)+p[1]*p[1]+(p[2]+1.).powi(2) <= 0.25;
    let expected = sampled_volume(&source,&member,[2.4,-0.6,-1.6],[7.6,0.6,1.6],64,256);
    let v = volume(&mesh);
    eprintln!("dumbbell: volume {v:.4}, sampled {expected:.4}");
    assert!((v-expected).abs() <= 0.03*expected,"volume {v} against sampled {expected}");
}
