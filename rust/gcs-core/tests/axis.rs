//! **Axes** (`docs/planes-plan.md`, #81): a directed line in space with no start — a direction
//! and a place — placed by the relations every other entity takes, drawn on no sheet.  Held here
//! against closed forms: the direction two relations leave, the place a point gives, the freedoms
//! an axis counts (a direction alone is two, and a place nothing reads is none), the refusal of an
//! unsigned angle at 0 or half a turn, and an axis across JSON and a copy.
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

/// An axis's direction and the point on it nearest the origin, as solved.
fn axis(sk: &Sketch, e: EntRef) -> ([f64; 3], [f64; 3]) {
    let r = &sk.axes[e.i()];
    (r.d.map(|p| sk.params[p as usize].value), r.a.map(|p| sk.params[p as usize].value))
}

/// The world's axes as axes fixed outright, as `std.sv` will state them.
const AXES: &str = "\
unit mm
x_axis := axis hint(x: 1, y: 0, z: 0)
fix(x == 1, y == 0, z == 0) x_axis
z_axis := axis hint(x: 0, y: 0, z: 1)
fix(x == 0, y == 0, z == 1) z_axis
";

/// Square to one axis and at a stated angle to another is a direction, and the seed picks which
/// of the two it could be: here, in the xz-plane 30° from x toward z.
#[test]
fn two_relations_give_an_axis_its_direction() {
    let src = format!("{AXES}y_axis := axis hint(x: 0, y: 1, z: 0)\nfix(x == 0, y == 1, z == 0) y_axis\n\
                       t := axis hint(x: 0.8, y: 0.1, z: 0.5)\nt perpendicular y_axis\nt angle(30deg) x_axis\n");
    let e = read(&src);
    let mut sk = solved(&e);
    let (d, _) = axis(&sk, ent(&e, "t"));
    let want = [30f64.to_radians().cos(), 0.0, 30f64.to_radians().sin()];
    assert!(norm(sub(d, want)) < 1e-9, "the axis points along {d:?}, not {want:?}");
    assert_eq!(dof(&mut sk), 0, "a direction is two freedoms, and two relations take them");
    let kinds: Vec<CKind> = sk.user_constraints().iter().map(|c| c.kind).collect();
    assert!(kinds.contains(&CKind::Perpendicular3) && kinds.contains(&CKind::Angle3), "{kinds:?}");
}

/// An axis nothing places has a direction and no place: two freedoms, not five — its point is
/// read by no row, so it is no freedom of the drawing.
#[test]
fn an_axis_read_only_as_a_direction_counts_two_freedoms() {
    let e = read("t := axis\n");
    let mut sk = e.sketch.clone();
    let t = ent(&e, "t");
    assert!(!sk.axes[t.i()].placed);
    assert_eq!(dof(&mut sk), 2);
    // an unseeded axis still points somewhere definite, and two of them apart
    let e = read("t := axis\ns := axis\n");
    let (dt, _) = axis(&e.sketch, ent(&e, "t"));
    let (ds, _) = axis(&e.sketch, ent(&e, "s"));
    assert!((norm(dt) - 1.0).abs() < 1e-12 && (norm(ds) - 1.0).abs() < 1e-12);
    assert!(norm(cross(dt, ds)) > 0.1, "two unseeded axes start apart: {dt:?} {ds:?}");
}

/// A point on an axis gives it a place: the axis's own point is the foot of the perpendicular
/// from the origin, and the axis passes through the point.  With its direction fixed by
/// `parallel`, a fixed point leaves it nothing.
#[test]
fn a_point_coincident_an_axis_places_it() {
    let src = format!("{AXES}use std\nin std.front {{\n  p := point hint(x: 10, y: 5)\n  fix(x == 10, y == 5) p\n}}\n\
                       t := axis hint(x: 0.9, y: 0, z: 0.1)\nt parallel x_axis\np coincident t\n");
    let e = read(&src);
    let t = ent(&e, "t");
    assert!(e.sketch.axes[t.i()].placed, "a relation reading where the axis is placed it");
    let kinds: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
    assert!(kinds.contains(&CKind::PointOnAxis), "{kinds:?}");
    let mut sk = solved(&e);
    let (d, a) = axis(&sk, t);
    // the page is x right and z up, so p stands at (10, 0, 5); the axis is the line through it
    // along x, whose point nearest the origin is (0, 0, 5)
    assert!(norm(sub(unit(d), [1.0, 0.0, 0.0])) < 1e-9, "{d:?}");
    assert!(norm(sub(a, [0.0, 0.0, 5.0])) < 1e-9, "the foot is {a:?}");
    assert!(dot(a, d).abs() < 1e-12);
    assert_eq!(dof(&mut sk), 0);
}

/// Removing the last relation that reads where an axis is takes its place away again: held, its
/// foot row gone, two freedoms and not five.  One still reading it keeps it placed.
#[test]
fn an_axis_no_relation_places_is_held_again() {
    let e = read("use std\nt := axis hint(x: 0, y: 0, z: 1)\np := point hint(x: 1, y: 2, z: 3)\n\
                  q := point hint(x: 1, y: 2, z: 5)\nfix(x == 1, y == 2, z == 3) p\n\
                  fix(x == 1, y == 2, z == 5) q\np coincident t\nq coincident t\n");
    let mut sk = e.sketch.clone();
    let t = ent(&e, "t").i();
    let on: Vec<u32> = sk.user_constraints().iter()
        .filter(|c| c.kind == CKind::PointOnAxis).map(|c| c.id).collect();
    assert_eq!(on.len(), 2);
    sk.remove(on[0]);
    assert!(sk.axes[t].placed, "q still reads where it is");
    sk.remove(on[1]);
    assert!(!sk.axes[t].placed);
    assert!(sk.axes[t].a.iter().all(|&q| sk.params[q as usize].fixed));
    let foot = EntRef::new(EntKind::Axis, t);
    assert!(!sk.constraints.iter().any(|c| c.kind == CKind::AxisFoot && c.entities().contains(&foot)));
    assert_eq!(dof(&mut sk), 2, "a direction, and no place");
}

/// An angle in space at 0 or half a turn is parallel said by a cosine that does not move there,
/// and is refused with the spelling that holds; any other angle is the relation it was.
#[test]
fn an_unsigned_angle_of_zero_or_half_a_turn_is_refused() {
    for a in ["0deg", "180deg"] {
        let src = format!("{AXES}t := axis hint(x: 0.9, y: 0, z: 0.1)\nt angle({a}) x_axis\n");
        refused(&src, "E040", "write `parallel`", a);
    }
    let e = read(&format!("{AXES}t := axis hint(x: 0.9, y: 0, z: 0.1)\nt angle(45deg) x_axis\n"));
    assert!(e.ok());
}

/// An axis across JSON and a copy: its direction, its fixed flags and its place come with it, and
/// the relation placing it places it again.
#[test]
fn an_axis_survives_json_and_a_copy() {
    let src = format!("{AXES}use std\nin std.front {{\n  p := point hint(x: 10, y: 5)\n  fix(x == 10, y == 5) p\n}}\n\
                       t := axis hint(x: 0.9, y: 0, z: 0.1)\nt parallel x_axis\np coincident t\n");
    let sk = solved(&read(&src));
    for back in [io::from_json(&io::to_json(&sk)).expect("reads back"), io::copy(&sk, &sk.primitives())] {
        assert_eq!(back.axes.len(), sk.axes.len());
        for i in 0..sk.axes.len() {
            let (e0, e1) = (EntRef::new(EntKind::Axis, i), EntRef::new(EntKind::Axis, i));
            let ((d0, a0), (d1, a1)) = (axis(&sk, e0), axis(&back, e1));
            assert!(norm(sub(d0, d1)) < 1e-12 && norm(sub(a0, a1)) < 1e-12, "axis {i}");
            assert_eq!(sk.axes[i].placed, back.axes[i].placed, "axis {i} placed");
            let fixed = |s: &Sketch| s.axes[i].d.map(|p| s.params[p as usize].fixed);
            assert_eq!(fixed(&sk), fixed(&back), "axis {i} fixed");
        }
    }
}

/// `in` has nothing to put on a plane in an axis, and an axis is drawn on no sheet.
#[test]
fn an_axis_is_in_no_view() {
    assert!(!EntKind::Axis.bears_points());
    assert!(EntKind::Axis.fields().iter().all(|(_, f)| *f == gcs_core::model::Field::Scalar));
}

/// Two axes `coincident` are one line, either way round: the second runs along the first and
/// stands on it, wherever its seed points.  Both are placed by it, and with the first held, the
/// second keeps one freedom — sliding along the line moves nothing, so none: its foot is fixed by
/// the line.  Removing the relation takes the second's place away again.
#[test]
fn two_coincident_axes_are_one_line_either_way() {
    for seed in ["0.9, y: 0.1, z: 0.2", "-0.9, y: 0.1, z: -0.2"] {
        let src = format!("{AXES}p := point hint(x: 3, y: 4, z: 5)\nfix(x == 3, y == 4, z == 5) p\n\
                           p coincident x_axis\nt := axis hint(x: {seed})\nt coincident x_axis\n");
        let e = read(&src);
        let mut sk = solved(&e);
        let (t, x) = (ent(&e, "t"), ent(&e, "x_axis"));
        assert!(sk.axes[t.i()].placed && sk.axes[x.i()].placed, "the relation reads both places");
        let ((dt, at), (dx, ax)) = (axis(&sk, t), axis(&sk, x));
        assert!(norm(cross(unit(dt), dx)) < 1e-9, "{dt:?} along {dx:?}");
        assert!(dot(dt, dx).signum() == seed.chars().next().map_or(1.0, |c| if c == '-' { -1.0 } else { 1.0 }),
            "the seed picks the sense: {dt:?}");
        assert!(norm(sub(at, ax)) < 1e-9, "one foot, so one line: {at:?} {ax:?}");
        assert!(norm(sub(ax, [0.0, 4.0, 5.0])) < 1e-9, "the line through p along x: {ax:?}");
        assert_eq!(dof(&mut sk), 0);
        let id = sk.user_constraints().iter().find(|c| c.kind == CKind::AxisCoincident).unwrap().id;
        sk.remove(id);
        assert!(!sk.axes[t.i()].placed, "nothing reads where t is now");
        assert!(sk.axes[x.i()].placed, "p still reads where x is");
        assert_eq!(dof(&mut sk), 2, "t is a direction again");
    }
}

/// A drawn line and an axis are parallel either way round, in whatever view the line is drawn.
#[test]
fn an_axis_and_a_line_are_parallel_either_way_round() {
    for stated in ["t parallel l", "l parallel t"] {
        let src = format!("{AXES}use std\nin std.front {{\n  a := point hint(x: 0, y: 0)\n  \
                           b := point hint(x: 10, y: 3)\n  fix(x == 0, y == 0) a\n  l := line(a, b)\n}}\n\
                           t := axis hint(x: 1, y: 0.2, z: 0)\nfix(x == 0.6, y == 0, z == 0.8) t\n{stated}\n");
        let e = read(&src);
        let kinds: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
        assert!(kinds.contains(&CKind::Parallel3), "{stated}: {kinds:?}");
        let sk = solved(&e);
        let (pa, pb) = (sk.world_point(ent(&e, "a").i()), sk.world_point(ent(&e, "b").i()));
        let along = unit(sub(pb, pa));
        assert!(norm(cross(along, [0.6, 0.0, 0.8])) < 1e-9, "{stated}: the line runs {along:?}");
    }
}

/// A line lying on an axis, either way round: both its ends on the axis's line, so it runs along
/// it, and the axis is placed by it.  Over a held axis a line with one end fixed keeps only its
/// slide along it.
#[test]
fn a_line_coincident_an_axis_lies_on_it() {
    for stated in ["l coincident t", "t coincident l", "l @ t"] {
        let src = format!("{AXES}use std\nin std.front {{\n  a := point hint(x: 1, y: 4)\n  \
                           b := point hint(x: 9, y: 7)\n  fix(x == 2, y == 5) a\n  l := line(a, b)\n}}\n\
                           t := axis hint(x: 1, y: 0.2, z: 0.3)\nfix(x == 0.6, y == 0, z == 0.8) t\n{stated}\n");
        let e = read(&src);
        let t = ent(&e, "t");
        assert!(e.sketch.axes[t.i()].placed, "{stated}: a line on the axis says where it is");
        let kinds: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
        assert_eq!(kinds, [CKind::LineOnAxis], "{stated}");
        let mut sk = solved(&e);
        let (d, a) = axis(&sk, t);
        for p in ["a", "b"] {
            let x = sk.world_point(ent(&e, p).i());
            assert!(norm(cross(sub(x, a), unit(d))) < 1e-9, "{stated}: {p} at {x:?} is off the axis");
        }
        // the axis passes through a's (2, 0, 5) along (0.6, 0, 0.8): its place is fixed by `a`,
        // and `b` slides along it, one freedom
        assert!(dot(sub([2.0, 0.0, 5.0], a), [0.0, 1.0, 0.0]).abs() < 1e-9);
        assert_eq!(dof(&mut sk), 1, "{stated}: b slides along the axis");
    }
}

/// A line in space on a free axis gives the axis both its direction and its place.
#[test]
fn a_line_in_space_on_an_axis_gives_it_direction_and_place() {
    let e = read("use std\np := point hint(x: 1, y: 2, z: 3)\nq := point hint(x: 4, y: 6, z: 3)\n\
                  fix(x == 1, y == 2, z == 3) p\nfix(x == 4, y == 6, z == 3) q\nl := line(p, q)\n\
                  t := axis hint(x: 0, y: 0.2, z: 1)\nl coincident t\n");
    let mut sk = solved(&e);
    let (d, a) = axis(&sk, ent(&e, "t"));
    assert!(norm(cross(unit(d), [0.6, 0.8, 0.0])) < 1e-9, "the axis runs {d:?}");
    assert!(norm(cross(sub([1.0, 2.0, 3.0], a), unit(d))) < 1e-9, "and passes through p");
    assert_eq!(dof(&mut sk), 0);
}
