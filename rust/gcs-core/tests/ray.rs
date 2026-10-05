//! **Rays** (`docs/planes-plan.md`, #81): a directed line in space with no start — a direction
//! and a place — placed by the relations every other entity takes, drawn on no sheet.  Held here
//! against closed forms: the direction two relations leave, the place a point gives, the freedoms
//! a ray counts (a direction alone is two, and a place nothing reads is none), the refusal of an
//! unsigned angle at 0 or half a turn, and a ray across JSON and a copy.
use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::io;
use gcs_core::model::{EntKind, EntRef, Sketch};
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{cross, dot, norm, sub};

use crate::common::{ent, read, refused, unit};

fn solved(e: &Elaborated) -> Sketch {
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    sk
}

fn dof(sk: &mut Sketch) -> i64 {
    diagnose(sk, DiagnoseOptions::default()).dof
}

/// A ray's direction and the point on it nearest the origin, as solved.
fn ray(sk: &Sketch, e: EntRef) -> ([f64; 3], [f64; 3]) {
    let r = &sk.rays[e.i()];
    (r.d.map(|p| sk.params[p as usize].value), r.a.map(|p| sk.params[p as usize].value))
}

/// The world's axes as rays fixed outright, as `std.sv` will state them.
const AXES: &str = "\
unit mm
x_axis := ray hint(x: 1, y: 0, z: 0)
fix(x == 1, y == 0, z == 0) x_axis
z_axis := ray hint(x: 0, y: 0, z: 1)
fix(x == 0, y == 0, z == 1) z_axis
";

/// Square to one axis and at a stated angle to another is a direction, and the seed picks which
/// of the two it could be: here, in the xz-plane 30° from x toward z.
#[test]
fn two_relations_give_a_ray_its_direction() {
    let src = format!("{AXES}y_axis := ray hint(x: 0, y: 1, z: 0)\nfix(x == 0, y == 1, z == 0) y_axis\n\
                       t := ray hint(x: 0.8, y: 0.1, z: 0.5)\nt perpendicular y_axis\nt angle(30deg) x_axis\n");
    let e = read(&src);
    let mut sk = solved(&e);
    let (d, _) = ray(&sk, ent(&e, "t"));
    let want = [30f64.to_radians().cos(), 0.0, 30f64.to_radians().sin()];
    assert!(norm(sub(d, want)) < 1e-9, "the ray points along {d:?}, not {want:?}");
    assert_eq!(dof(&mut sk), 0, "a direction is two freedoms, and two relations take them");
    let kinds: Vec<CKind> = sk.user_constraints().iter().map(|c| c.kind).collect();
    assert!(kinds.contains(&CKind::Perpendicular3) && kinds.contains(&CKind::Angle3), "{kinds:?}");
}

/// A ray nothing places has a direction and no place: two freedoms, not five — its point is
/// read by no row, so it is no freedom of the drawing.
#[test]
fn a_ray_read_only_as_a_direction_counts_two_freedoms() {
    let e = read("t := ray\n");
    let mut sk = e.sketch.clone();
    let t = ent(&e, "t");
    assert!(!sk.rays[t.i()].placed);
    assert_eq!(dof(&mut sk), 2);
    // an unseeded ray still points somewhere definite, and two of them apart
    let e = read("t := ray\ns := ray\n");
    let (dt, _) = ray(&e.sketch, ent(&e, "t"));
    let (ds, _) = ray(&e.sketch, ent(&e, "s"));
    assert!((norm(dt) - 1.0).abs() < 1e-12 && (norm(ds) - 1.0).abs() < 1e-12);
    assert!(norm(cross(dt, ds)) > 0.1, "two unseeded rays start apart: {dt:?} {ds:?}");
}

/// A point on a ray gives it a place: the ray's own point is the foot of the perpendicular
/// from the origin, and the ray passes through the point.  With its direction fixed by
/// `parallel`, a fixed point leaves it nothing.
#[test]
fn a_point_coincident_a_ray_places_it() {
    let src = format!("{AXES}use std\nin std.front {{\n  p := point hint(x: 10, y: 5)\n  fix(x == 10, y == 5) p\n}}\n\
                       t := ray hint(x: 0.9, y: 0, z: 0.1)\nt parallel x_axis\np coincident t\n");
    let e = read(&src);
    let t = ent(&e, "t");
    assert!(e.sketch.rays[t.i()].placed, "a relation reading where the ray is placed it");
    let kinds: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
    assert!(kinds.contains(&CKind::PointOnRay), "{kinds:?}");
    let mut sk = solved(&e);
    let (d, a) = ray(&sk, t);
    // the page is x right and z up, so p stands at (10, 0, 5); the ray is the line through it
    // along x, whose point nearest the origin is (0, 0, 5)
    assert!(norm(sub(unit(d), [1.0, 0.0, 0.0])) < 1e-9, "{d:?}");
    assert!(norm(sub(a, [0.0, 0.0, 5.0])) < 1e-9, "the foot is {a:?}");
    assert!(dot(a, d).abs() < 1e-12);
    assert_eq!(dof(&mut sk), 0);
}

/// Removing the last relation that reads where a ray is takes its place away again: held, its
/// foot row gone, two freedoms and not five.  One still reading it keeps it placed.
#[test]
fn a_ray_no_relation_places_is_held_again() {
    let e = read("use std\nt := ray hint(x: 0, y: 0, z: 1)\np := point hint(x: 1, y: 2, z: 3)\n\
                  q := point hint(x: 1, y: 2, z: 5)\nfix(x == 1, y == 2, z == 3) p\n\
                  fix(x == 1, y == 2, z == 5) q\np coincident t\nq coincident t\n");
    let mut sk = e.sketch.clone();
    let t = ent(&e, "t").i();
    let on: Vec<u32> = sk.user_constraints().iter()
        .filter(|c| c.kind == CKind::PointOnRay).map(|c| c.id).collect();
    assert_eq!(on.len(), 2);
    sk.remove(on[0]);
    assert!(sk.rays[t].placed, "q still reads where it is");
    sk.remove(on[1]);
    assert!(!sk.rays[t].placed);
    assert!(sk.rays[t].a.iter().all(|&q| sk.params[q as usize].fixed));
    assert!(!sk.constraints.iter().any(|c| c.kind == CKind::RayFoot));
    assert_eq!(dof(&mut sk), 2, "a direction, and no place");
}

/// An angle in space at 0 or half a turn is parallel said by a cosine that does not move there,
/// and is refused with the spelling that holds; any other angle is the relation it was.
#[test]
fn an_unsigned_angle_of_zero_or_half_a_turn_is_refused() {
    for a in ["0deg", "180deg"] {
        let src = format!("{AXES}t := ray hint(x: 0.9, y: 0, z: 0.1)\nt angle({a}) x_axis\n");
        refused(&src, "E040", "write `parallel`", a);
    }
    let e = read(&format!("{AXES}t := ray hint(x: 0.9, y: 0, z: 0.1)\nt angle(45deg) x_axis\n"));
    assert!(e.ok());
}

/// A ray across JSON and a copy: its direction, its fixed flags and its place come with it, and
/// the relation placing it places it again.
#[test]
fn a_ray_survives_json_and_a_copy() {
    let src = format!("{AXES}use std\nin std.front {{\n  p := point hint(x: 10, y: 5)\n  fix(x == 10, y == 5) p\n}}\n\
                       t := ray hint(x: 0.9, y: 0, z: 0.1)\nt parallel x_axis\np coincident t\n");
    let sk = solved(&read(&src));
    for back in [io::from_json(&io::to_json(&sk)).expect("reads back"), io::copy(&sk, &sk.primitives())] {
        assert_eq!(back.rays.len(), sk.rays.len());
        for i in 0..sk.rays.len() {
            let (e0, e1) = (EntRef::new(EntKind::Ray, i), EntRef::new(EntKind::Ray, i));
            let ((d0, a0), (d1, a1)) = (ray(&sk, e0), ray(&back, e1));
            assert!(norm(sub(d0, d1)) < 1e-12 && norm(sub(a0, a1)) < 1e-12, "ray {i}");
            assert_eq!(sk.rays[i].placed, back.rays[i].placed, "ray {i} placed");
            let fixed = |s: &Sketch| s.rays[i].d.map(|p| s.params[p as usize].fixed);
            assert_eq!(fixed(&sk), fixed(&back), "ray {i} fixed");
        }
    }
}

/// `in` has nothing to put on a plane in a ray, and a ray is drawn on no sheet.
#[test]
fn a_ray_is_in_no_view() {
    assert!(!EntKind::Ray.bears_points());
    assert!(EntKind::Ray.fields().iter().all(|(_, f)| *f == gcs_core::model::Field::Scalar));
}
