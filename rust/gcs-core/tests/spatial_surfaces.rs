//! Cones and cylinders (`docs/spatial-constraints-plan.md`): surfaces in space a relation can
//! name, built about an axis already drawn in a view and owning one number each — a half-angle
//! or a radius — and the words they take, against closed forms; the hypoid's pitch cones named
//! and stated to touch, against the fold construction of them (`spatial_lang.rs`); `against`
//! between solved views; and a stated plane's origin through a lifted program.
use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, summary, DiagnoseOptions};
use gcs_core::io;
use gcs_core::model::{EntKind, EntRef, Sketch};
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{cross, dot, norm, sub};

use crate::common::{ends, ent, off_line, read, refused, unit};

fn at(sk: &Sketch, e: &Elaborated, n: &str) -> [f64; 3] {
    sk.world_point(ent(e, n).i())
}

/// A cone's or a cylinder's axis — its start (a cone's apex) and unit direction — and its own
/// number, as solved.
fn axial(sk: &Sketch, e: EntRef) -> ([f64; 3], [f64; 3], f64) {
    let a = sk.axial(e);
    let (p, q) = ends(sk, EntRef::line(a.axis as usize));
    (p, unit(sub(q, p)), sk.params[a.param as usize].value)
}

/// The angle in degrees between two directions.
fn angle(a: [f64; 3], b: [f64; 3]) -> f64 {
    dot(unit(a), unit(b)).clamp(-1.0, 1.0).acos().to_degrees()
}

fn solved(e: &Elaborated) -> Sketch {
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    sk
}

fn dof(src: &str) -> i64 {
    let mut sk = solved(&read(src));
    diagnose(&mut sk, DiagnoseOptions::default()).dof
}

/// Two planes square to each other: an axis grounded in the front one, and a point and a line in
/// the side one.
const VIEWS: &str = "\
unit mm
use std
side := plane(u: std.z, v: std.y)
fix(origin == (0, 0, 0)) side
in std.front {
  ax := line
  fix((0, 0)) ax.p1
  fix((0, 50)) ax.p2
  bx := line
  fix((0, 20)) bx.p1
  fix((50, 20)) bx.p2
}
a := point hint((10, 20)) in side
l := line(hint((5, 3)), hint((30, 20))) in side
c := cylinder(axis: ax) hint(r: 10)
cb := cylinder(axis: bx) hint(r: 10)
k := cone(axis: ax) hint(half: 30deg)
";

/// The kind one statement over `VIEWS` settles to.
fn settles(stmt: &str) -> CKind {
    let e = read(&format!("{VIEWS}{stmt}\n"));
    let cs = e.sketch.user_constraints();
    assert_eq!(cs.len(), 1, "{stmt}: {:?}", cs.iter().map(|c| c.kind).collect::<Vec<_>>());
    cs[0].kind
}

#[test]
fn a_cone_and_a_cylinder_are_entities_about_a_drawn_axis() {
    let e = read(VIEWS);
    let (c, k) = (ent(&e, "c"), ent(&e, "k"));
    assert_eq!((c.kind, k.kind), (EntKind::Cylinder, EntKind::Cone));
    let sk = &e.sketch;
    // made of the axis, owning one number: the seed, and a half-angle held in radians
    assert_eq!(sk.children(c), vec![ent(&e, "ax")]);
    assert_eq!(sk.params[sk.axial(c).param as usize].value, 10.0);
    assert!((sk.params[sk.axial(k).param as usize].value - 30f64.to_radians()).abs() < 1e-15);
    // no freedom of their own beyond that number: a grounded axis leaves the two numbers
    let held = format!("{VIEWS}fix(r == 10) c\nfix(r == 10) cb\nfix(half == 30deg) k\n");
    assert_eq!(dof(VIEWS) - dof(&held), 3);
    // and nothing on a sheet: neither is drawn on a page
    assert_eq!(gcs_core::overview::drawable(sk, c, 1.0).len(), 0);
}

#[test]
fn the_words_a_cone_and_a_cylinder_take() {
    assert_eq!(settles("a coincident c"), CKind::CylinderOn);
    assert_eq!(settles("a coincident k"), CKind::ConeOn);
    assert_eq!(settles("radius(15) c"), CKind::CylinderRadius);
    assert_eq!(settles("angle(25deg) k"), CKind::ConeAngle);
    assert_eq!(settles("c tangent l"), CKind::CylinderTangentLine);
    // a point of the axis's own view is on a cylinder in space too
    assert_eq!(settles("b := point hint((4, 4)) in std.front\nb coincident c"), CKind::CylinderOn);
}

/// A point on a cylinder stands its radius off the axis, and `radius` states it.
#[test]
fn a_point_on_a_cylinder_is_its_radius_off_the_axis() {
    let src = format!("{VIEWS}radius(15) c\na coincident c\n");
    let e = read(&src);
    let sk = solved(&e);
    let (p, d, r) = axial(&sk, ent(&e, "c"));
    assert!((r - 15.0).abs() < 1e-12);
    let got = off_line(at(&sk, &e, "a"), p, d);
    assert!((got - 15.0).abs() < 1e-9, "{got}");
    // one equation: the point keeps one of its two freedoms
    assert_eq!(dof(&format!("{VIEWS}radius(15) c\n")) - dof(&src), 1);
}

/// A point on a cone makes the half-angle with the axis at the apex, on the nappe the axis
/// points into.
#[test]
fn a_point_on_a_cone_makes_its_half_angle_at_the_apex() {
    let src = format!("{VIEWS}angle(25deg) k\na coincident k\n");
    let e = read(&src);
    let sk = solved(&e);
    let (apex, d, half) = axial(&sk, ent(&e, "k"));
    assert!((half.to_degrees() - 25.0).abs() < 1e-12);
    let w = sub(at(&sk, &e, "a"), apex);
    assert!((angle(w, d) - 25.0).abs() < 1e-9, "{}", angle(w, d));
    assert!(dot(w, d) > 0.0, "on the nappe the axis points into");
    assert_eq!(dof(&format!("{VIEWS}angle(25deg) k\n")) - dof(&src), 1);
    // and a free half-angle is found by the point
    let e = read(&format!("{VIEWS}fix((130, 20)) a\na coincident k\n"));
    let sk = solved(&e);
    let (apex, d, half) = axial(&sk, ent(&e, "k"));
    assert!((half.to_degrees() - angle(sub(at(&sk, &e, "a"), apex), d)).abs() < 1e-9);
}

/// A line touching a cylinder is its radius from the axis along their common perpendicular.
#[test]
fn a_line_touching_a_cylinder_is_its_radius_from_the_axis() {
    // the axis runs square to the side view, so the line touches it where it passes a circle
    // about the point the axis crosses that view at
    let src = format!("{VIEWS}radius(12) cb\nfix((5, 3)) l.p1\ncb tangent l\n");
    let e = read(&src);
    let sk = solved(&e);
    let (p, d, _) = axial(&sk, ent(&e, "cb"));
    let (a, b) = ends(&sk, ent(&e, "l"));
    let m = cross(d, sub(b, a));
    let gap = dot(unit(m), sub(a, p)).abs();
    assert!((gap - 12.0).abs() < 1e-9, "{gap}");
}

/// **The named hypoid** (the plan's P4 gate): the app's `examples/hypoid_pitch_cones.sv`, its
/// pitch cones named — the gear's axial plane stated, the pinion's solved, `M on` both cones and
/// `gc tangent(M) pc` — against the fold construction `fixtures/hypoid_pitch_cones.sv` states.  The two
/// documents put their views differently on the sheet, so they are compared by what a hypoid
/// is: the pitch angles, the offset angle, the apexes' distances from M and from each other, all
/// to 1e-9.
#[test]
fn the_hypoid_with_its_pitch_cones_named_agrees_with_the_fold_construction() {
    let named = read(include_str!("../../examples/hypoid_pitch_cones.sv"));
    let mut sk = solved(&named);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "{}", summary(&d));
    assert_eq!(d.n_equations, d.structural_rank, "{}", summary(&d));
    let clean = d.over.is_empty() && d.conflicts.as_deref().unwrap_or(&[]).is_empty();
    assert!(clean, "{}", summary(&d));
    let folds = read(include_str!("fixtures/hypoid_pitch_cones.sv"));
    let fk = solved(&folds);
    // what each document came to, read off the lifted geometry alone
    let measure = |sk: &Sketch, e: &Elaborated| {
        let m = at(sk, e, "M");
        let ((o, g2), (a, p2)) = (ends(sk, ent(e, "gax")), ends(sk, ent(e, "pax")));
        [
            angle(sub(m, o), sub(g2, o)),
            angle(sub(m, a), sub(p2, a)),
            angle(sub(a, m), sub(o, m)),
            norm(sub(o, m)),
            norm(sub(a, m)),
            norm(sub(a, o)),
        ]
    };
    let (x, y) = (measure(&sk, &named), measure(&fk, &folds));
    eprintln!("named: Γ = {:.9}°, γ = {:.9}°, ε = {:.9}°", x[0], x[1], x[2]);
    eprintln!("named: |MO| = {:.9}, |MA| = {:.9}, |OA| = {:.9}", x[3], x[4], x[5]);
    let names = ["gear pitch angle", "pinion pitch angle", "offset angle", "|MO|", "|MA|", "|OA|"];
    for k in 0..6 {
        let gap = (x[k] - y[k]).abs();
        eprintln!("{}: {gap:.3e}", names[k]);
        assert!(gap < 1e-9, "{}: {} named against {} folded", names[k], x[k], y[k]);
    }
    // the cones' own numbers are those angles, and they touch at M: each cone's surface normal
    // there, worked out here from its apex, axis and half-angle, is the other's and the pitch
    // plane's
    let m = at(&sk, &named, "M");
    let normal = |n: &str| {
        let (apex, d, half) = axial(&sk, ent(&named, n));
        let w = sub(m, apex);
        let h = dot(w, d);
        let radial = unit(sub(w, [d[0] * h, d[1] * h, d[2] * h]));
        assert!((angle(w, d) - half.to_degrees()).abs() < 1e-9, "M on {n}");
        [0, 1, 2].map(|t| radial[t] * half.cos() - d[t] * half.sin())
    };
    let (ng, np) = (normal("gc"), normal("pc"));
    let pn = sk.basis(ent(&named, "std.front").i()).normal();
    assert!(norm(cross(ng, np)) < 1e-9, "one tangent plane at M: {:e}", norm(cross(ng, np)));
    assert!(norm(cross(ng, pn)) < 1e-9, "and it is P: {:e}", norm(cross(ng, pn)));
    // the pinion's apex is on P though nothing says so: two cones with one tangent plane at M
    let a = ends(&sk, ent(&named, "pax")).0;
    assert!(dot(pn, sub(a, m)).abs() < 1e-9, "{:e}", dot(pn, sub(a, m)));
    let (_, _, gp) = axial(&sk, ent(&named, "pc"));
    assert!((gp.to_degrees() - y[1]).abs() < 1e-9);
}

/// A cone, a cylinder and their words come through JSON and a lifted program, and a half-angle
/// goes back into the source in degrees, as it was written.
#[test]
fn cones_and_cylinders_round_trip() {
    let named = read(include_str!("../../examples/hypoid_pitch_cones.sv"));
    let sk = solved(&named);
    let text = io::dumps(&sk, None);
    let back = io::loads(&text).expect("reads back");
    assert_eq!(io::dumps(&back, None), text);
    assert_eq!((back.cones.len(), back.cylinders.len()), (2, 0));
    // a lifted program spells both cones and the contact, and reads back to the same drawing
    let lifted = gcs_core::program::to_program(&sk).text().to_string();
    let half = lifted.split("n0 := cone(axis: l0) hint(half: ").nth(1).and_then(|t| t.split(')').next())
        .unwrap_or_else(|| panic!("{lifted}"));
    assert!((half.parse::<f64>().unwrap() - 60.0).abs() < 1e-9, "in degrees: {lifted}");
    let m = ent(&named, "M").i();
    assert!(lifted.contains(&format!("n0 tangent(p{m}) n1")), "{lifted}");
    let again = solved(&read(&lifted));
    for i in 0..sk.points.len() {
        assert!(norm(sub(sk.world_point(i), again.world_point(i))) < 1e-7, "p{i}");
    }
    // a cylinder's side comes through a document, where nobody wrote it
    let e = read(&format!("{VIEWS}radius(12) cb\nfix((5, 3)) l.p1\ncb tangent l\n"));
    let sk = solved(&e);
    let back = io::loads(&io::dumps(&sk, None)).expect("reads back");
    assert_eq!(io::dumps(&back, None), io::dumps(&sk, None));
    // the solve's half-angle written back where the seed was, in degrees (a literal written with
    // its unit is text a writeback leaves alone, as a sphere's `hint(r: 25mm)` is)
    let src = format!("{}angle(40deg) k\n", VIEWS.replace("hint(half: 30deg)", "hint(half: 30)"));
    let e = read(&src);
    let sk = solved(&e);
    let out = gcs_core::edit::commit_seeds(&e, &sk, &e.program).text;
    assert!(out.contains("k := cone(axis: ax) hint(half: 40)"), "{out}");
}

/// Each is drawn in the glass box: two circles square to the axis and four rulings.
#[test]
fn the_glass_box_draws_them() {
    let e = read(&format!("{VIEWS}radius(15) c\nangle(25deg) k\n"));
    let sk = solved(&e);
    let items = gcs_core::overview::scene3d(&sk, 0.0);
    for n in ["c", "k"] {
        let of = ent(&e, n);
        let mine = items.iter().filter(|i| i.of == Some(of)).count();
        assert!(mine >= 5, "{n}: {mine}");
    }
    // the cylinder's circles and rulings stand its radius off the axis
    let (p, d, r) = axial(&sk, ent(&e, "c"));
    for item in items.iter().filter(|i| i.of == Some(ent(&e, "c"))) {
        for q in &item.pts {
            assert!((off_line(*q, p, d) - r).abs() < 1e-9);
        }
    }
}

/// What is not a relation yet is said, and a cone or a cylinder is built about a line.
#[test]
fn what_a_cone_or_a_cylinder_does_not_take_is_refused() {
    let with = |s: &str| format!("{VIEWS}k2 := cone(axis: l) hint(half: 20deg)\n{s}\n");
    refused(&with("l coincident k"), "E040", "a line on a cone or a cylinder", "coincident");
    refused(&with("l coincident c"), "E040", "a line on a cone or a cylinder", "coincident");
    refused(&with("k tangent l"), "E040", "a line touches a cylinder", "tangent");
    refused(&with("k tangent k2"), "E040", "names it", "tangent");
    refused(&with("angle(20deg) c"), "E040", "does not apply to a cylinder", "angle");
    refused(&with("radius(20) k"), "E040", "does not apply to a cone", "radius");
    refused(&format!("{VIEWS}bad := cone(axis: a)\n"), "E103", "axis is a line", "a");
    refused(&format!("{VIEWS}bad := cylinder\n"), "E103", "built about a line", "bad := cylinder");
    refused(&format!("{VIEWS}radius(-3) c\n"), "E040", "magnitude", "-3");
}

/* -- `against` with solved views ------------------------------------------------------------ */

/// **A plane stood off the origin keeps where it stands when lifted**: its axes say how it turns,
/// and its `hint(…)` where it is.
#[test]
fn a_lifted_plane_keeps_its_origin() {
    let src = "unit mm\nuse std\n\
               back := plane hint(origin: (0, -12, 0))\n\
               fix(origin == (0, -12, 0)) back\n\
               fix(dir == (1, 0, 0)) back.u\n\
               fix(dir == (0, 0, 1)) back.v\n\
               r := axis hint(dir: (0.8, 0.6, 0))\nfix(dir == (0.8, 0.6, 0)) r\n\
               side := plane(u: r) hint(origin: (3, 4, 5))\n\
               fix(origin == (3, 4, 5)) side\nfix(dir == (0, 0, 1)) side.v\n\
               a := point hint((5, 7)) in back\nb := point hint((9, -3)) in side\n";
    let e = read(src);
    let sk = solved(&e);
    let lifted = gcs_core::program::to_program(&sk).text().to_string();
    let again = read(&lifted);
    for i in 0..sk.planes.len() {
        let (x, y) = (sk.basis(i), again.sketch.basis(i));
        assert!(norm(sub(x.o, y.o)) < 1e-12 && norm(sub(x.u, y.u)) < 1e-12, "v{i}: {x:?} {y:?}");
    }
    for i in 0..sk.points.len() {
        assert!(norm(sub(sk.world_point(i), again.sketch.world_point(i))) < 1e-9, "p{i}");
    }
}

/// A drag is built on the dragged point's part, and a point drawn in a plane brings that plane
/// with it, held: the part's relations in space have a place to read it from.  `pb`, drawn in
/// `std.side` and held to a ball centred in `std.front`, drags round its circle on the ball.
#[test]
fn a_point_on_a_ball_drags_on_its_own_part() {
    let e = read(include_str!("../../examples/sphere_cone_cylinder.sv"));
    let mut sk = e.sketch.clone();
    assert!(gcs_core::solve::solve(&mut sk, Default::default()).success);
    let (pb, bc) = (ent(&e, "pb").i(), ent(&e, "bc").i());
    let (x, y) = sk.point_xy(pb);
    let mut d = gcs_core::decompose::PlanDrag::new(&sk, pb, x, y, None, 0.05);
    assert!(d.part().unwrap().sketch.planes.len() >= 2, "the planes its points are drawn in");
    for k in 1..10 {
        let r = d.move_to(&mut sk, None, x + 0.5 * k as f64, y - k as f64);
        assert!(r.success, "{k}: {r:?}");
        let gap = norm(sub(sk.world_point(pb), sk.world_point(bc)));
        assert!((gap - 12.0).abs() < 1e-6, "{k}: {gap} from the ball's centre");
    }
    d.end();
}
