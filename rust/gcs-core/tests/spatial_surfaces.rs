//! Cones and cylinders, the library's (`std.Cone`, `std.Cylinder`): a line and a number each, sets
//! of points (§6.21, `tests/sets.rs`), and what stands on them — a point on a cylinder its radius
//! from the axis, one on a cone at its half-angle at the apex, a line its radius from a cylinder's
//! axis by the common perpendicular, two cones' tangent planes made one (`std.TangentCones`) —
//! against closed forms; the hypoid's pitch cones named and stated to
//! touch, against the fold construction of them (`spatial_lang.rs`); and a stated plane's origin
//! through a lifted program.
use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, summary, DiagnoseOptions};
use gcs_core::io;
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{cross, dot, norm, sub};

use crate::common::{ends, ent, off_line, read, refused, unit};

fn at(sk: &Sketch, e: &Elaborated, n: &str) -> [f64; 3] {
    sk.world_point(ent(e, n).i())
}

/// A line's start (a cone's apex) and unit direction, as solved.
fn axis(sk: &Sketch, e: &Elaborated, n: &str) -> ([f64; 3], [f64; 3]) {
    let (p, q) = ends(sk, ent(e, n));
    (p, unit(sub(q, p)))
}

/// What the drawing solved an unknown it declared to, in the units it was declared in (an angle
/// in degrees).
fn unknown(sk: &Sketch, name: &str) -> f64 {
    sk.params[sk.free_vars[name] as usize].value
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

/// Two planes square to each other: two axes grounded in the front one, and a point and a line in
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
";

/// `VIEWS` with a cylinder about each axis and a cone about the first, and `more` after them.
fn with(c: &str, k: &str, more: &str) -> String {
    format!("{VIEWS}c := std.Cylinder(ax, r: {c})\ncb := std.Cylinder(bx, r: {c})\n\
             k := std.Cone(ax, half: {k})\n{more}\n")
}

/// The kind the one statement over the surfaces settles to.
fn settles(stmt: &str) -> CKind {
    let e = read(&with("10", "30deg", stmt));
    let cs = e.sketch.user_constraints();
    assert_eq!(cs.len(), 1, "{stmt}: {:?}", cs.iter().map(|c| c.kind).collect::<Vec<_>>());
    cs[0].kind
}

/// A cylinder and a cone add nothing to the drawing: no unknown, no equation, nothing drawn.
#[test]
fn a_cone_and_a_cylinder_are_a_line_and_a_number() {
    let (bare, named) = (read(VIEWS), read(&with("10", "30deg", "")));
    assert_eq!(named.sketch.params.len(), bare.sketch.params.len());
    assert_eq!(named.sketch.user_constraints().len(), 0);
    assert_eq!(named.sketch.drawn().len(), bare.sketch.drawn().len());
}

#[test]
fn the_words_on_a_cone_and_a_cylinder() {
    // a distance from the axis, in space across views and on the page within one
    assert_eq!(settles("a coincident c"), CKind::PointLine3);
    assert_eq!(settles("b := point hint((4, 4)) in std.front\nb coincident c"),
               CKind::PointLineDistance);
    assert_eq!(settles("c.about distance(c.r) l"), CKind::LineLine3);
    // an angle at the apex
    assert_eq!(settles("a coincident k"), CKind::Angle3);
    // and the words are gone: `cone` and `cylinder` are names like any other
    refused(&format!("{VIEWS}k := cone(axis: ax)\n"), "E103", "no component named `cone`", "cone");
}

/// A point on a cylinder stands its radius off the axis.
#[test]
fn a_point_on_a_cylinder_is_its_radius_off_the_axis() {
    let src = with("15", "30deg", "a coincident c");
    let e = read(&src);
    let sk = solved(&e);
    let (p, d) = axis(&sk, &e, "ax");
    let got = off_line(at(&sk, &e, "a"), p, d);
    assert!((got - 15.0).abs() < 1e-9, "{got}");
    // one equation: the point keeps one of its two freedoms
    assert_eq!(dof(&with("15", "30deg", "")) - dof(&src), 1);
}

/// A point on a cone makes the half-angle with the axis at the apex, on the nappe the axis
/// points into.
#[test]
fn a_point_on_a_cone_makes_its_half_angle_at_the_apex() {
    let src = with("10", "25deg", "a coincident k");
    let e = read(&src);
    let sk = solved(&e);
    let (apex, d) = axis(&sk, &e, "ax");
    let w = sub(at(&sk, &e, "a"), apex);
    assert!((angle(w, d) - 25.0).abs() < 1e-9, "{}", angle(w, d));
    assert!(dot(w, d) > 0.0, "on the nappe the axis points into");
    assert_eq!(dof(&with("10", "25deg", "")) - dof(&src), 1);
    // and a half-angle left unbound is found by the point
    let e = read(&with("10", "hint(30deg)", "fix((130, 20)) a\na coincident k"));
    let sk = solved(&e);
    let (apex, d) = axis(&sk, &e, "ax");
    let half = unknown(&sk, "k.half");
    assert!((half - angle(sub(at(&sk, &e, "a"), apex), d)).abs() < 1e-9, "{half}");
}

/// A line touching a cylinder is its radius from the axis along their common perpendicular.
#[test]
fn a_line_touching_a_cylinder_is_its_radius_from_the_axis() {
    // the axis runs square to the side view, so the line touches it where it passes a circle
    // about the point the axis crosses that view at
    let e = read(&with("12", "30deg", "fix((5, 3)) l.p1\ncb.about distance(cb.r) l"));
    let sk = solved(&e);
    let (p, d) = axis(&sk, &e, "bx");
    let (a, b) = ends(&sk, ent(&e, "l"));
    let m = cross(d, sub(b, a));
    let gap = dot(unit(m), sub(a, p)).abs();
    assert!((gap - 12.0).abs() < 1e-9, "{gap}");
}

/// **The named hypoid** (the plan's P4 gate): the app's `examples/hypoid_pitch_cones.sv`, its
/// pitch cones named — the gear's axial plane stated, the pinion's solved, M on both cones and
/// `std.TangentCones(gc, pc, M)` — against the fold construction `fixtures/hypoid_pitch_cones.sv`
/// states.  The two documents put their views differently on the sheet, so they are compared by
/// what a hypoid is: the pitch angles, the offset angle, the apexes' distances from M and from
/// each other, all to 1e-9.
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
    let normal = |n: &str, half: f64| {
        let (apex, d) = axis(&sk, &named, n);
        let w = sub(m, apex);
        let h = dot(w, d);
        let radial = unit(sub(w, [d[0] * h, d[1] * h, d[2] * h]));
        assert!((angle(w, d) - half.to_degrees()).abs() < 1e-9, "M on {n}");
        [0, 1, 2].map(|t| radial[t] * half.cos() - d[t] * half.sin())
    };
    let gp = unknown(&sk, "pc.half");
    let (ng, np) = (normal("gax", 60f64.to_radians()), normal("pax", gp.to_radians()));
    let pn = sk.basis(ent(&named, "std.front").i()).normal();
    assert!(norm(cross(ng, np)) < 1e-9, "one tangent plane at M: {:e}", norm(cross(ng, np)));
    assert!(norm(cross(ng, pn)) < 1e-9, "and it is P: {:e}", norm(cross(ng, pn)));
    // the pinion's apex is on P though nothing says so: two cones with one tangent plane at M
    let a = ends(&sk, ent(&named, "pax")).0;
    assert!(dot(pn, sub(a, m)).abs() < 1e-9, "{:e}", dot(pn, sub(a, m)));
    assert!((gp - y[1]).abs() < 1e-9);
}

/// The named hypoid comes through JSON and a lifted program (the pinion's unknown half-angle,
/// `pc.half`, declared as `pc_half`), and a half-angle left unbound goes back into its `hint(…)`
/// in degrees, as it was written.
#[test]
fn the_named_hypoid_round_trips() {
    let named = read(include_str!("../../examples/hypoid_pitch_cones.sv"));
    let sk = solved(&named);
    let text = io::dumps(&sk, None);
    let back = io::loads(&text).expect("reads back");
    assert_eq!(io::dumps(&back, None), text);
    let lifted = gcs_core::program::to_program(&sk).text().to_string();
    assert!(lifted.contains("param pc_half: Angle hint("), "{lifted}");
    let again = solved(&read(&lifted));
    for i in 0..sk.points.len() {
        assert!(norm(sub(sk.world_point(i), again.world_point(i))) < 1e-7, "p{i}");
    }
    let src = with("10", "hint(30deg)", "fix((130, 20)) a\na coincident k");
    let e = read(&src);
    let sk = solved(&e);
    let out = gcs_core::edit::commit_seeds(&e, &sk, &e.program).text;
    let half = angle(sub(at(&sk, &e, "a"), [0.0; 3]), [0.0, 0.0, 1.0]);
    let written = out.split("half: hint(").nth(1).and_then(|t| t.split("deg)").next())
        .unwrap_or_else(|| panic!("{out}"));
    assert!((written.parse::<f64>().unwrap() - half).abs() < 1e-6, "{written} against {half}");
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
