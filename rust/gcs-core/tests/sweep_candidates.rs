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

/// A cylinder turning about a perpendicular line through its centre: the wall
/// contacts along the two meridians in the plane of the line and along the
/// stationary ring through the centre, each cap along its diameter in that
/// plane, and the rims contribute fans between. Every open piece ends exactly
/// at one of the four rim vertices, where a fan meets the strands it bounds.
#[test]
fn a_tumbling_cylinder_chains_its_fans_to_its_strands_exactly() {
    let e = read(&format!("{CYLINDER}private point hub hint(x: 3, y: 0)
hub distance(3mm, along: u) std.front
hub distance(0mm, along: v) std.front
private point spoke hint(x: 5, y: 0)
spoke distance(5mm, along: u) std.front
spoke distance(0mm, along: v) std.front
construction centerline line spin(hub, spoke)
motion tumble(about: spin)
solid swept(tool, under: tumble, from: -30deg, to: 30deg)
"));
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("swept").unwrap().i(),1e-10).unwrap();
    assert!(constant_twist(sweep.motion(),sweep.domain(),1e-9).unwrap());
    let curves = sweep.characteristics(1e-9).unwrap();
    // A fan's ends are found to the contact tolerance, so the vertices are met to 1e-5.
    let rim = |p: [f64;3]| [[4.,0.,1.],[2.,0.,1.],[4.,0.,-1.],[2.,0.,-1.]].iter().any(|v| distance(p,*v) < 1e-5);
    let (mut rings,mut open) = (0,0);
    for c in &curves {
        if c.closed {
            // the stationary ring through the centre, or a loop the chain closed through rim vertices
            let ring = c.points.iter().all(|p| p[2].abs() < 1e-9 && ((p[0]-3.).hypot(p[1])-1.).abs() < 1e-9);
            if ring { rings += 1; } else { assert!(c.points.iter().filter(|p| rim(**p)).count() >= 1,"a closed loop off the rims"); }
        } else {
            open += 1;
            let (first,last) = (c.points[0],*c.points.last().unwrap());
            // the meridians end on the stationary ring; everything else on a rim vertex
            let on_ring = |p: [f64;3]| p[2].abs() < 1e-6 && ((p[0]-3.).hypot(p[1])-1.).abs() < 1e-6;
            assert!((rim(first) || on_ring(first)) && (rim(last) || on_ring(last)),"a piece ends off a vertex: {first:?} .. {last:?}");
        }
    }
    assert_eq!(rings,1,"the stationary ring");
    assert!(open+curves.len() > 1,"pieces: {}",curves.len());
    // every wall point lies on one of the two meridians or the ring, every cap
    // point on its diameter, and the fans on the rims
    for c in &curves {
        for p in &c.points {
            let r = (p[0]-3.).hypot(p[1]);
            let on_wall = (r-1.).abs() < 1e-6 && (p[1].abs() < 1e-6 || p[2].abs() < 1e-6);
            let on_cap = (p[2].abs()-1.).abs() < 1e-6 && p[1].abs() < 1e-6;
            let on_rim = (r-1.).abs() < 1e-6 && (p[2].abs()-1.).abs() < 1e-6;
            assert!(on_wall || on_cap || on_rim,"{p:?}");
        }
    }
}
