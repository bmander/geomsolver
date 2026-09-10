//! Fixed contact curves of constant-twist sweeps against closed forms.
use gcs_core::{program,syntax,solve,solid::{SweepContacts,constant_twist}};
use std::f64::consts::PI;

fn read(source: &str) -> program::Elaborated {
    let (mut p,errors) = syntax::parse(source); assert!(errors.is_empty(),"{errors:?}");
    let errors = gcs_core::modules::link(&mut p,&mut |name| gcs_core::library::resolve(name));
    assert!(errors.is_empty(),"{errors:?}");
    let mut e = program::elaborate(&p); assert!(e.ok(),"{:?}",e.diags);
    assert!(solve::solve(&mut e.sketch,solve::SolveOpts {tol:1e-16,acceptance_tol:1e-12,..Default::default()}).success);
    e
}

const SPHERE: &str = "unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point center
center distance(3mm, along: u) std.front
center distance(0mm, along: v) std.front
private point bottom hint(x: 3, y: -1)
private point top hint(x: 3, y: 1)
private line diameter(bottom, top)
center midpoint diameter
diameter parallel spindle
distance(2mm) diameter
private arc meridian(center: center, start: bottom, end: top)
radius(1mm) meridian
construction solid tool(face(meridian, diameter), about: diameter)
";

fn distance(a: [f64;3],b: [f64;3]) -> f64 { (0..3).map(|k| (a[k]-b[k]).powi(2)).sum::<f64>().sqrt() }

#[test]
fn a_sphere_under_a_parallel_rotation_has_one_great_circle_characteristic() {
    let e = read(&format!("{SPHERE}motion turn(about: spindle)\nsolid swept(tool, under: turn, from: -60deg, to: 60deg)\n"));
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("swept").unwrap().i(),1e-10).unwrap();
    assert!(constant_twist(sweep.motion(),sweep.domain(),1e-9).unwrap());
    let curves = sweep.characteristics(1e-9).unwrap();
    assert_eq!(curves.len(),1,"{} characteristics",curves.len());
    let c = &curves[0];
    assert!(c.closed && c.points.len() > 100);
    // The great circle in the plane through the spindle (world z) and the centre (3,0,0).
    for (p,n) in c.points.iter().zip(&c.normals) {
        assert!(p[1].abs() < 1e-9,"off the meridian plane: {p:?}");
        assert!((distance(*p,[3.,0.,0.])-1.).abs() < 1e-9);
        let radial = [p[0]-3.,p[1],p[2]];
        assert!((0..3).all(|k| (n[k]-radial[k]).abs() < 1e-6),"normal {n:?} at {p:?}");
    }
}

#[test]
fn a_sphere_under_a_translation_has_one_great_circle_characteristic() {
    let e = read(&format!("{SPHERE}motion feed(along: spindle, advance: 10mm)\nsolid swept(tool, under: feed, from: 0deg, to: 360deg)\n"));
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("swept").unwrap().i(),1e-10).unwrap();
    assert!(constant_twist(sweep.motion(),sweep.domain(),1e-9).unwrap());
    let curves = sweep.characteristics(1e-9).unwrap();
    assert_eq!(curves.len(),1);
    let c = &curves[0];
    assert!(c.closed);
    // The equator perpendicular to the direction of travel (world z).
    for p in &c.points {
        assert!(p[2].abs() < 1e-9,"{p:?}");
        assert!((distance(*p,[3.,0.,0.])-1.).abs() < 1e-9);
    }
}

#[test]
fn a_screw_is_constant_twist_and_a_relative_rotation_is_not() {
    let e = read(&format!("{SPHERE}construction centerline line other(std.origin, std.front.toward)\n\
        motion tap(about: spindle, advance: 2mm)\nmotion spin(about: other, ratio: 2)\nmotion relative(tap, relative_to: spin)\n\
        solid a(tool, under: tap, from: 0deg, to: 90deg)\nsolid b(tool, under: relative, from: 0deg, to: 90deg)\n"));
    let a = SweepContacts::read(&e.sketch,e.map.ent_named("a").unwrap().i(),1e-10).unwrap();
    let b = SweepContacts::read(&e.sketch,e.map.ent_named("b").unwrap().i(),1e-10).unwrap();
    assert!(constant_twist(a.motion(),a.domain(),1e-9).unwrap());
    assert!(!constant_twist(b.motion(),b.domain(),1e-9).unwrap());
    let _ = PI;
}

const CYLINDER: &str = "unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point c0 hint(x: 3, y: -1)
private point c1 hint(x: 4, y: -1)
private point c2 hint(x: 4, y: 1)
private point c3 hint(x: 3, y: 1)
ground c0
ground c1
ground c2
ground c3
private line bottom(c0, c1)
private line wall(c1, c2)
private line top(c2, c3)
private line axis(c3, c0)
construction solid tool(face(bottom, wall, top, axis), about: axis)
";

/// A cylinder plunging along its own axis: its wall is stationary and
/// contributes nothing, its two rims are the sharp edges whose normal cones
/// straddle the velocity, and each sweeps the extended wall.
#[test]
fn a_plunging_cylinder_contributes_its_two_rims() {
    let e = read(&format!("{CYLINDER}motion plunge(along: axis, advance: 5mm)\nsolid swept(tool, under: plunge, from: 0deg, to: 360deg)\n"));
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("swept").unwrap().i(),1e-10).unwrap();
    assert!(constant_twist(sweep.motion(),sweep.domain(),1e-9).unwrap());
    let curves = sweep.characteristics(1e-9).unwrap();
    assert_eq!(curves.len(),2,"{} characteristics: {:?}",curves.len(),curves.iter().map(|c| (c.points.len(),c.closed)).collect::<Vec<_>>());
    let mut heights: Vec<f64> = Vec::new();
    for c in &curves {
        assert!(c.closed);
        let z = c.points[0][2];
        for p in &c.points {
            assert!((p[2]-z).abs() < 1e-9 && ((p[0]-3.).hypot(p[1])-1.).abs() < 1e-9,"{p:?}");
        }
        heights.push(z);
    }
    heights.sort_by(f64::total_cmp);
    assert!((heights[0]+1.).abs() < 1e-9 && (heights[1]-1.).abs() < 1e-9,"{heights:?}");
}
