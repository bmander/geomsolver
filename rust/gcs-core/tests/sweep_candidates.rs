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

const BOX: &str = "unit mm
use std
private point b0 hint(x: 2, y: -1)
private point b1 hint(x: 4, y: -1)
private point b2 hint(x: 4, y: 1)
private point b3 hint(x: 2, y: 1)
ground b0
ground b1
ground b2
ground b3
private line e0(b0, b1)
private line e1(b1, b2)
private line e2(b2, b3)
private line e3(b3, b0)
construction solid tool(face(e0, e1, e2, e3), from: 0mm, to: 3mm)
private point r0 hint(x: 0, y: 0)
private point r1 hint(x: 10, y: 0)
ground r0
ground r1
construction centerline line rail(r0, r1)
";

/// A box translated along one of its edge directions: four faces are
/// stationary and the two across the travel never contact, so the candidates
/// are the edges round the leading and trailing faces, each a segment carrying
/// the normal of the side it bounds. The page is x across and z up with depth
/// along -y, so the box is x in [2,4], z in [-1,1], y in [-3,0].
#[test]
fn a_box_translated_along_x_contributes_the_edges_round_its_ends() {
    let e = read(&format!("{BOX}motion feed(along: rail, advance: 10mm)\nsolid swept(tool, under: feed, from: 0deg, to: 360deg)\n"));
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("swept").unwrap().i(),1e-10).unwrap();
    assert!(constant_twist(sweep.motion(),sweep.domain(),1e-9).unwrap());
    assert_eq!(sweep.faces().len(),6);
    assert_eq!(sweep.edges().len(),12);
    let curves = sweep.characteristics(1e-9).unwrap();
    for c in &curves {
        for (p,n) in c.points.iter().zip(&c.normals) {
            assert!((p[0]-2.).abs() < 1e-9 || (p[0]-4.).abs() < 1e-9,"off the ends: {p:?}");
            // on the boundary of an end face, carrying that side's normal
            let on_side = (p[2].abs()-1.).abs() < 1e-9 || p[1].abs() < 1e-9 || (p[1]+3.).abs() < 1e-9;
            assert!(on_side,"{p:?}");
            assert!(n[0].abs() < 1e-6,"a side normal has no x: {n:?} at {p:?}");
        }
    }
    let total: f64 = curves.iter().map(|c| c.points.windows(2).map(|w| distance(w[0],w[1])).sum::<f64>()
        + if c.closed { distance(c.points[0],*c.points.last().unwrap()) } else { 0. }).sum();
    // two rectangles of 2 by 3, less nothing: 20 mm of edge, each edge once
    assert!((total-20.).abs() < 1e-3,"edge length {total}: {:?}",curves.iter().map(|c| (c.points.len(),c.closed)).collect::<Vec<_>>());
}

/// Two closed polylines zip into a closed band: every edge of it paired, and
/// every vertex of both rings used.
#[test]
fn zipping_two_rings_gives_a_closed_band() {
    let ring = |r: f64,n: usize,phase: f64| -> Vec<[f64;3]> { (0..n).map(|i| { let a = phase+std::f64::consts::TAU*i as f64/n as f64; [r*a.cos(),r*a.sin(),0.] }).collect() };
    let (a,b) = (ring(1.,40,0.),ring(1.2,53,0.3));
    let triangles = gcs_core::solid::zip_polylines(&a,&b,true);
    assert_eq!(triangles.len(),40+53,"{} triangles",triangles.len());
    let mut directed: std::collections::HashMap<((bool,u32),(bool,u32)),usize> = Default::default();
    for t in &triangles { for k in 0..3 { *directed.entry((t[k],t[(k+1)%3])).or_default() += 1; } }
    for (&(p,q),&n) in &directed {
        assert_eq!(n,1,"edge {p:?}->{q:?} used {n} times");
        let opposite = directed.get(&(q,p)).copied().unwrap_or(0);
        let rung = p.0 != q.0;
        assert!(if rung { opposite == 1 } else { opposite == 0 },"edge {p:?}->{q:?}: opposite used {opposite} times");
    }
    let used: std::collections::HashSet<(bool,u32)> = triangles.iter().flatten().copied().collect();
    assert_eq!(used.len(),40+53);
}

const TRIANGLE_PRISM: &str = "unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point t0 hint(x: 3, y: -0.8)
private point t1 hint(x: 4.5, y: 0)
private point t2 hint(x: 3, y: 0.8)
ground t0
ground t1
ground t2
private line e0(t0, t1)
private line e1(t1, t2)
private line e2(t2, t0)
construction solid tool(face(e0, e1, e2), from: -1.5mm, to: 1.5mm)
private point a0 hint(x: 2.5, y: 0)
a0 distance(2.5mm, along: u) std.front
a0 distance(0mm, along: v) std.front
private point a1 hint(x: 2.5, y: 5)
a1 distance(2.5mm, along: u) std.front
a1 distance(5mm, along: v) std.front
construction centerline line pivot(a0, a1)
motion turn(about: pivot)
solid swept(tool, under: turn, from: -50deg, to: 50deg)
";

/// A prism turning about an axis beside it: at every parameter the contact
/// pieces chain into closed loops, since the boundary between the advancing
/// and receding parts of a closed surface is closed.
#[test]
fn a_turning_prisms_contact_pieces_close_at_every_parameter() {
    let e = read(TRIANGLE_PRISM);
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("swept").unwrap().i(),1e-10).unwrap();
    for t in [-0.8,-0.3,0.0,0.3,0.5,0.8] {
        let pieces = sweep.pieces_at(t,1e-9).unwrap();
        let summary: Vec<String> = pieces.iter().map(|(s,c)| format!("{s}: {} pts {:?}..{:?}",c.points.len(),c.points[0].map(|v| (v*1e3).round()/1e3),c.points.last().unwrap().map(|v| (v*1e3).round()/1e3))).collect();
        // every piece end must be shared with another piece's end
        let ends: Vec<[f64;3]> = pieces.iter().filter(|(_,c)| !c.closed).flat_map(|(_,c)| [c.points[0],*c.points.last().unwrap()]).collect();
        for (i,p) in ends.iter().enumerate() {
            let shared = ends.iter().enumerate().any(|(j,q)| j != i && distance(*p,*q) < 1e-6);
            assert!(shared,"at {t}: loose end {p:?}\n{}",summary.join("\n"));
        }
    }
}
