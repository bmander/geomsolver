//! Planes over axes, in the language (§6.7; `docs/planes-plan.md`): a plane whose axis or place
//! is solved for, its seeds, its refusals, and `project` between a stated and a solved plane;
//! the words across planes and in space; spheres; and the hypoid's pitch cones laid out by
//! construction.
use gcs_core::constraints::{CKind, Constraint};
use gcs_core::diagnose::{diagnose, view_freedoms, DiagnoseOptions};
use gcs_core::edit;
use gcs_core::io;
use gcs_core::model::Sketch;
use gcs_core::program::{solid_diagnostics, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{cross, dot, norm, sub};
use gcs_core::syntax::write_stmt_to;

use crate::common::{ends, ent, off_line, read, refused, unit};

/// The gear and pinion axes of a hypoid layout: the gear's drawn on the front plane, up the page;
/// the pinion's level in a plane through the origin over the page's `y` and an axis `t` the
/// document solves for, square to `y`.
const AXES: &str = "\
unit mm
use std
in std.front {
  gax := line
  fix(x == 0, y == 0) gax.p1
  fix(x == 0, y == 50) gax.p2
}
t := axis hint(x: 0.87, y: 0, z: 0.5)
t perpendicular std.y
side := plane(u: t, v: std.y)
side.origin coincident std.origin
in side {
  pax := line(hint(x: 0, y: 10), hint(x: 60, y: 12))
  pax.p1 distance(0, along: u) side
  pax.p2 distance(60, along: u) side
  pax.p1 horizontal pax.p2
}
";

/// The axis `t` of a drawing over `AXES`, as a direction.
fn axis_t(e: &Elaborated, sk: &Sketch) -> [f64; 3] {
    let r = ent(e, "t").i();
    sk.axes[r].d.map(|k| sk.params[k as usize].value)
}

#[test]
fn the_shaft_angle_and_the_offset_solve_the_axis() {
    let e = read(AXES);
    assert!(e.diags.is_empty(), "{:?}", e.diags.iter().map(|d| &d.message).collect::<Vec<_>>());
    let mut sk = e.sketch.clone();
    let mut d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 2, "the axis's turn about y and the pinion axis's height");
    let (gax, pax) = (ent(&e, "gax"), ent(&e, "pax"));
    let offset = 17.5;
    let angle = Constraint::in_space(&sk, CKind::Angle3, &[gax, pax], Some(90f64.to_radians()))
        .unwrap();
    sk.add(angle);
    let skew = Constraint::in_space(&sk, CKind::LineLine3, &[gax, pax], Some(offset)).unwrap();
    sk.add(skew);
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
    assert!(d.over.is_empty() && d.conflicts.as_deref().unwrap_or(&[]).is_empty());
    // an independent reading: the two axes' ends in space
    let ((a, b), (c, dd)) = (ends(&sk, gax), ends(&sk, pax));
    let (e1, e2) = (sub(b, a), sub(dd, c));
    let cosang = dot(e1, e2) / (norm(e1) * norm(e2));
    assert!(cosang.abs() < 1e-9, "the shaft angle is 90°: cos = {cosang}");
    let m = cross(e1, e2);
    let gap = dot(m, sub(c, a)).abs() / norm(m);
    assert!((gap - offset).abs() < 1e-9, "the offset is {offset}: {gap}");
    // and the axis came round level, from a seed 30° up
    let t = axis_t(&e, &sk);
    assert!(t[2].abs() < 1e-9 && (t[0] - 1.0).abs() < 1e-9, "t = {t:?}");
}

/// The gate fixture's statements in space, stated through the Rust API rather than in words.
fn gate() -> (Elaborated, Sketch) {
    let e = read(AXES);
    let mut sk = e.sketch.clone();
    let (gax, pax) = (ent(&e, "gax"), ent(&e, "pax"));
    let angle = Constraint::in_space(&sk, CKind::Angle3, &[gax, pax], Some(90f64.to_radians()));
    sk.add(angle.unwrap());
    sk.add(Constraint::in_space(&sk, CKind::LineLine3, &[gax, pax], Some(17.5)).unwrap());
    (e, sk)
}

#[test]
fn the_gate_round_trips_through_json() {
    let (_, mut sk) = gate();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let text = io::dumps(&sk, None);
    let mut back = io::loads(&text).expect("loads");
    // the same unknowns and the same statements, the intrinsic rows minted again
    assert_eq!(back.params.len(), sk.params.len());
    assert_eq!(back.constraints.len(), sk.constraints.len());
    assert_eq!(back.axes.len(), sk.axes.len());
    // already solved: nothing moves, and the diagnosis reads the same
    let before: Vec<f64> = back.params.iter().map(|p| p.value).collect();
    assert!(solve(&mut back, SolveOpts::default()).success);
    for (a, p) in before.iter().zip(&back.params) {
        assert!((a - p.value).abs() < 1e-9, "{} moved", p.name);
    }
    assert_eq!(diagnose(&mut back, DiagnoseOptions::default()).dof, 0);
    // and written again, byte for byte
    assert_eq!(io::dumps(&back, None), text);
}

/// Each declaration prints back as it was written, and what it prints parses to the same print.
#[test]
fn the_declarations_print_back() {
    let src = "\
use std
t := axis hint(x: 1, y: 2, z: 0)
r := axis
side := plane(u: t, v: std.z)
top := plane(u: std.x, v: r) hint(x: 0, y: 0, z: 12)
s := point hint(x: 1, y: 2, z: 3)
m := point hint(x: 5, y: 7) in side
";
    let print = |text: &str| -> String {
        let (prog, errs, _) = gcs_core::library::parse_linked(text);
        assert!(errs.is_empty(), "{errs:?}\n{text}");
        prog.root().body.iter().map(|st| {
            let mut out = String::new();
            write_stmt_to(&mut out, &st.kind).unwrap();
            out
        }).collect::<Vec<_>>().join("\n")
    };
    let once = print(src);
    for decl in ["t := axis hint(x: 1, y: 2, z: 0)", "r := axis", "side := plane(u: t, v: std.z)",
                 "top := plane(u: std.x, v: r) hint(x: 0, y: 0, z: 12)",
                 "s := point hint(x: 1, y: 2, z: 3)", "m := point hint(x: 5, y: 7) in side"] {
        assert!(once.contains(decl), "`{decl}` is not in\n{once}");
    }
    assert_eq!(print(&once), once);
    read(src);
}

/// A plane over a drawn line holds the line's direction — a hidden axis kept parallel to it — so
/// it follows the line when the line moves; it adds no freedom of its own beyond where it stands.
#[test]
fn a_plane_over_a_drawn_line_follows_the_line() {
    let doc = |deg: f64| format!("\
unit mm
use std
in std.front {{
  base := line
  fix(x == 0, y == 0) base.p1
  fix(x == 50, y == 0) base.p2
  l := line(p2: hint(x: 40, y: 20))
  fix(x == 10, y == 5) l.p1
  l.p1 distance(40) l.p2
  base angle({deg}deg) l
}}
side := plane(u: l, v: std.y)
side.origin coincident l.p1
");
    for deg in [30.0, 55.0] {
        let e = read(&doc(deg));
        let mut sk = e.sketch.clone();
        assert!(solve(&mut sk, SolveOpts::default()).success);
        let d = diagnose(&mut sk, DiagnoseOptions::default());
        assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
        let side = ent(&e, "side").i();
        let b = sk.basis(side);
        let (p1, p2) = ends(&sk, ent(&e, "l"));
        let dir = sub(p2, p1);
        // the plane contains the line: its direction and its first end
        assert!(dot(b.normal(), dir).abs() < 1e-9 * norm(dir), "{b:?}");
        assert!(dot(b.normal(), sub(p1, b.o)).abs() < 1e-9);
        // square to the front, and the line at the stated bearing in space
        assert!(b.normal()[1].abs() < 1e-12);
        let bearing = dir[2].atan2(dir[0]).to_degrees();
        assert!((bearing - deg).abs() < 1e-7, "{bearing}");
        assert!(view_freedoms(&sk, &d).is_empty());
    }
}

/// `P coincident m` stands the plane through `m` and follows it; its origin still slides in it,
/// two freedoms the ledger names.
#[test]
fn a_plane_through_a_point_follows_it() {
    let doc = |h: f64| format!("\
unit mm
use std
in std.front {{
  m := point hint(x: 12, y: 3)
  std.origin distance(12, along: x) m
  std.origin distance({h}, along: y) m
}}
top := plane(u: std.x, v: std.y) hint(x: 0, y: 0, z: 30)
top coincident m
");
    for h in [8.0, -3.5] {
        let e = read(&doc(h));
        let mut sk = e.sketch.clone();
        assert!(solve(&mut sk, SolveOpts::default()).success);
        let d = diagnose(&mut sk, DiagnoseOptions::default());
        assert_eq!(d.dof, 2, "{}", gcs_core::diagnose::summary(&d));
        assert_eq!(view_freedoms(&sk, &d), vec!["top.origin".to_string()]);
        let b = sk.basis(ent(&e, "top").i());
        // the top plane's normal is +z, and the front plane's z is its drawn y
        assert!((b.normal()[2] - 1.0).abs() < 1e-12);
        assert!((b.along_normal() - h).abs() < 1e-9, "{} against {h}", b.along_normal());
    }
}

/// A plane over free axes starts where their seeds say, and stands where its own seed says.
#[test]
fn a_free_plane_starts_at_its_seeds() {
    let e = read("\
unit mm
a := axis hint(x: 0, y: 1, z: 0)
b := axis hint(x: 0, y: 0, z: 2)
q := plane(u: a, v: b) hint(x: 5, y: 0, z: 0)
");
    let mut sk = e.sketch.clone();
    let q = ent(&e, "q").i();
    let b = sk.basis(q);
    assert_eq!((b.u, b.v, b.o), ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [5.0, 0.0, 0.0]));
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 2 + 2 + 3, "{}", gcs_core::diagnose::summary(&d));
    assert_eq!(view_freedoms(&sk, &d), vec!["q.attitude".to_string(), "q.origin".to_string()]);
    // and `--where q` reports where it stands in space
    let at = gcs_core::report::positions(&sk, &e.map);
    for k in ["q.u.x", "q.v.z", "q.n.y", "q.x"] {
        assert!(at.iter().any(|(n, _)| n == k), "no `{k}`: {at:?}");
    }
}

/// `project` between a stated plane and a solved one is the projector rule in space: the fold
/// line two planes share is where their images agree, and it turns with the solved plane.  The
/// side plane holds the page's `y` and an axis square to it at β above `x`, so the fold line is
/// that axis, at `(cos β, sin β)` on the front plane: `b` on it 50 along means
/// `cos β·30 + sin β·40 = 50`, so β = atan2(40, 30).
#[test]
fn a_projection_between_a_stated_and_a_solved_plane() {
    let e = read("\
unit mm
use std
t := axis hint(x: 0.94, y: 0, z: 0.34)
t perpendicular std.y
side := plane(u: t, v: std.y)
side.origin coincident std.origin
in std.front {
  a := point hint(x: 30, y: 40)
  fix(x == 30, y == 40) a
}
in side {
  b := point hint(x: 50, y: 0)
  fix(x == 50, y == 0) b
}
a project b
");
    let mut sk = e.sketch.clone();
    assert!(sk.constraints.iter().any(|c| c.kind == CKind::ProjectSolved));
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
    let t = axis_t(&e, &sk);
    let beta = t[2].atan2(t[0]);
    assert!((beta - 40f64.atan2(30.0)).abs() < 1e-6, "beta := {}", beta.to_degrees());
    assert!(solid_diagnostics(&sk, &e.map).is_empty());
}

/// Two solved planes a `project` relates that come out parallel are refused after the solve, and
/// two planes that come out lying on one another are a lint.
#[test]
fn planes_that_come_out_parallel_are_said() {
    let e = read("\
use std
r1 := axis hint(x: 1, y: 0, z: 0)
r2 := axis hint(x: 0, y: 0, z: 1)
q := plane(u: r1, v: r2)
a := point hint(x: 3, y: 4) in std.front
b := point hint(x: 3, y: 4) in q
a project b
");
    let diags = solid_diagnostics(&e.sketch, &e.map);
    assert!(diags.iter().any(|d| d.code.as_str() == "E065" && d.message.contains("parallel")),
            "{diags:?}");
    let e = read("\
use std
q := plane(u: std.x, v: std.z)
q.origin coincident std.origin
a := point hint(x: 3, y: 4) in std.front
b := point hint(x: 7, y: 1) in q
a distance(5) b
");
    let diags = solid_diagnostics(&e.sketch, &e.map);
    assert!(diags.iter().any(|d| d.code.as_str() == "W113"), "{diags:?}");
}

/// A plane is two axes and a place: what is not an axis or a line is refused at the slot, and a
/// plane's origin is its own and not written.  A point drawn in a plane takes no height.
#[test]
fn a_plane_is_refused_what_it_cannot_stand_on() {
    refused("use std\np := point hint(x: 1, y: 2, z: 3)\nq := plane(u: std.x, v: p)\n",
            "E103", "an axis or a line", "p");
    refused("use std\nq := plane(u: std.x)\n", "E103", "two axes", "q := plane(u: std.x)");
    refused("use std\nq := plane(u: std.x, v: std.y, origin: std.origin)\n",
            "E103", "origin is its own", "q := plane(u: std.x, v: std.y, origin: std.origin)");
    refused("use std\nm := point hint(x: 1, y: 2, z: 3) in std.front\n", "E040", "no `z`", "3");
}

/// A solve's axis goes back into the source as its seed, where the seed was written.
#[test]
fn the_solved_axis_is_written_back_to_its_seed() {
    let (e, mut sk) = gate();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let ed = edit::commit_seeds(&e, &sk, &e.program);
    let line = ed.text.lines().find(|l| l.starts_with("t := axis hint(")).expect("the axis's seed");
    assert!(line.contains("z: 0)") || line.contains("z: -0)") || line.contains("e-"), "{line}");
    // and the text it wrote elaborates to the same drawing
    read(&ed.text);
}

/// A skew distance read from a document that left its side out has the side inferred from the
/// geometry, as a `null` there always did — not defaulted to one the drawing may not stand on.
#[test]
fn a_skew_side_left_out_of_a_document_is_read_off_the_geometry() {
    let mut sides = Vec::new();
    // the pinion's axis drawn on one side of the gear's, and on the other
    let below = AXES.replace("y: 10), hint(x: 60, y: 12)", "y: -10), hint(x: 60, y: -12)");
    for doc in [AXES.to_string(), below] {
        let e = read(&doc);
        let mut sk = e.sketch.clone();
        let ls = [ent(&e, "gax"), ent(&e, "pax")];
        let c = Constraint::in_space(&sk, CKind::LineLine3, &ls, Some(5.0)).unwrap();
        let side = c.args[3].clone();
        sk.add(c);
        let text = io::dumps(&sk, None);
        let written = format!("5.0,{}]}}", if side.num() < 0.0 { "-1" } else { "1" });
        assert!(text.contains(&written), "{text}");
        let back = io::loads(&text.replace(&written, "5.0]}")).expect("loads");
        let c = back.constraints.iter().find(|c| c.kind == CKind::LineLine3).unwrap();
        assert_eq!(c.args[3].num(), side.num(), "the side as the geometry has it");
        sides.push(side.num());
    }
    // the two stand on opposite sides, so one of them is the side a default would miss
    assert_eq!(sides[0], -sides[1]);
}

/// A plane over a deleted axis or line is defined from nothing, and goes with it.
#[test]
fn deleting_what_a_plane_stands_on_deletes_the_plane() {
    let src = "use std\nl := line(hint(x: 0, y: 0), hint(x: 5, y: 5)) in std.front\n\
               r := axis\n\
               s := plane(u: l, v: std.y)\n\
               u := plane(u: r, v: std.z)\n";
    let e = read(src);
    let out = edit::remove(&e, &e.program, &e.sketch, &[ent(&e, "l")], &[]);
    assert!(!out.text.contains("s := plane(") && out.text.contains("u := plane("), "{}", out.text);
    read(&out.text);
    let out = edit::remove(&e, &e.program, &e.sketch, &[ent(&e, "r")], &[]);
    assert!(out.text.contains("s := plane(") && !out.text.contains("u := plane("), "{}", out.text);
    read(&out.text);
}

/// An axis inside a component is that instance's own: two instances turn independently.
#[test]
fn an_axis_in_a_component_is_the_instances_own() {
    let e = read("\
use std
component Wing(f: plane) {
  r := axis hint(x: 1, y: 1, z: 0)
  r perpendicular f.v
  w := plane(u: r, v: f.v)
}
one := Wing(std.front)
two := Wing(std.top)
");
    assert_eq!(e.sketch.axes.len(), 4 + 2, "the standard four and one each");
    let (a, b) = (ent(&e, "one.r").i(), ent(&e, "two.r").i());
    assert_ne!(a, b);
    for r in [a, b] {
        assert!(e.sketch.axes[r].d.iter().all(|&k| !e.sketch.params[k as usize].fixed));
    }
}

/* -- the words across planes ------------------------------------------------------------- */

/// The gate, written in words: the shaft angle and the offset are `angle` and `distance` between
/// two axes drawn in different planes, and so are relations in space.
const AXES_IN_WORDS: &str = "\
gax angle(90deg) pax
gax distance(17.5) pax
";

#[test]
fn the_gate_in_words_solves_the_axis() {
    let e = read(&format!("{AXES}{AXES_IN_WORDS}"));
    let mut sk = e.sketch.clone();
    let kinds: Vec<CKind> = sk.user_constraints().iter().map(|c| c.kind).collect();
    assert!(kinds.contains(&CKind::Angle3) && kinds.contains(&CKind::LineLine3), "{kinds:?}");
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
    assert_eq!(d.n_equations, d.structural_rank, "{}", gcs_core::diagnose::summary(&d));
    let ((a, b), (c, dd)) = (ends(&sk, ent(&e, "gax")), ends(&sk, ent(&e, "pax")));
    let (e1, e2) = (sub(b, a), sub(dd, c));
    assert!((dot(e1, e2) / (norm(e1) * norm(e2))).abs() < 1e-9);
    let m = cross(e1, e2);
    assert!((dot(m, sub(c, a)).abs() / norm(m) - 17.5).abs() < 1e-9);
    // the same solve the Rust API's statements (`gate`) come to, point for point in space
    let (ge, mut gk) = gate();
    assert!(solve(&mut gk, SolveOpts::default()).success);
    let (gc, gd) = ends(&gk, ent(&ge, "pax"));
    for (x, y) in [(c, gc), (dd, gd)] {
        assert!(norm(sub(x, y)) < 1e-9, "{x:?} against {y:?}");
    }
    // and the statements round-trip through JSON as the kinds they settled to
    let back = io::loads(&io::dumps(&sk, None)).expect("loads");
    let kinds: Vec<CKind> = back.user_constraints().iter().map(|c| c.kind).collect();
    assert!(kinds.contains(&CKind::Angle3) && kinds.contains(&CKind::LineLine3), "{kinds:?}");
    assert_eq!(io::dumps(&back, None), io::dumps(&sk, None));
}

/// The front and side planes, square to each other, with a point, a line and a circle drawn in
/// each, and a point in space.
const TWO_VIEWS: &str = "\
unit mm
use std
in std.front {
  a := point hint(x: 10, y: 20)
  la := line(hint(x: 0, y: 0), hint(x: 30, y: 10))
}
in std.side {
  b := point hint(x: 10, y: 5)
  lb := line(hint(x: 5, y: 3), hint(x: 30, y: 20))
  cb := circle(hint(x: 20, y: 10)) hint(r: 8)
}
s := point hint(x: 4, y: -6, z: 9)
r := axis hint(x: 1, y: 1, z: 1)
q := plane(u: std.x, v: std.y) hint(x: 0, y: 0, z: 20)
";

/// The kind one statement over `TWO_VIEWS` settles to.
fn settles(stmt: &str) -> CKind {
    let e = read(&format!("{TWO_VIEWS}{stmt}\n"));
    let cs = e.sketch.user_constraints();
    assert_eq!(cs.len(), 1, "{stmt}: {:?}", cs.iter().map(|c| c.kind).collect::<Vec<_>>());
    cs[0].kind
}

/// Every word that has a meaning in space means it across planes, and the same word within one
/// plane is the 2D relation it always was.
#[test]
fn each_word_across_planes_is_the_relation_in_space() {
    for (stmt, kind) in [
        ("a coincident b", CKind::Coincident3),
        ("a distance(30) b", CKind::Distance3),
        ("a distance(30) s", CKind::Distance3),
        ("s coincident a", CKind::Coincident3),
        ("a distance(5) lb", CKind::PointLine3),
        ("s distance(5) la", CKind::PointLine3),
        ("la distance(5) lb", CKind::LineLine3),
        ("la angle(60deg) lb", CKind::Angle3),
        ("la perpendicular lb", CKind::Perpendicular3),
        ("la parallel lb", CKind::Parallel3),
        ("la equal lb", CKind::EqualLength3),
        ("a coincident lb", CKind::PointOnLine3),
        ("a midpoint lb", CKind::Midpoint3),
        ("a symmetry(lb) la.p1", CKind::Symmetric3),
        ("a coincident cb", CKind::PointOnCircle3),
        // a plane is a place in space whatever plane the point is drawn in
        ("a coincident std.side", CKind::PointOnPlane),
        ("s coincident std.side", CKind::PointOnPlane),
        ("la coincident std.side", CKind::LineOnPlane),
        ("a distance(5, along: n) std.side", CKind::PointPlaneDistance),
        // an ordinate is a coordinate in its own plane, and how far along the axis in another
        ("b distance(5, along: u) std.side", CKind::CoordinateU),
        ("a distance(5, along: u) std.side", CKind::Ordinate3U),
        ("s distance(5, along: v) std.side", CKind::Ordinate3V),
        // an axis and a plane
        ("s coincident r", CKind::PointOnAxis),
        ("r coincident q", CKind::AxisOnPlane),
        ("r parallel q", CKind::AxisParallelPlane),
        ("r perpendicular q", CKind::AxisPerpendicularPlane),
        ("r perpendicular std.z", CKind::Perpendicular3),
        ("la parallel r", CKind::Parallel3),
        ("q distance(5) std.top", CKind::PlaneDistance),
        // within one plane, the plane's words
        ("a distance(30) la.p1", CKind::Distance),
        ("b coincident lb", CKind::PointOnLine),
        ("la angle(60deg) la", CKind::Angle),
    ] {
        assert_eq!(settles(stmt), kind, "{stmt}");
    }
}

/// A statement in space reads back as the word it was written with.
#[test]
fn a_relation_in_space_is_described_by_its_word() {
    let e = read(&format!("{TWO_VIEWS}la distance(5) lb\na distance(3, along: n) std.side\n"));
    let said: Vec<String> = e.sketch.user_constraints().iter()
        .map(|c| io::describe_with(c, &|r| e.map.name_of(r).cloned())).collect();
    assert_eq!(said, vec!["la distance(5) lb".to_string(),
                          "a distance(3, along: n) std.side".to_string()]);
}

/// A plane's origin is a point drawn in it: beside that plane's points it is a 2D relation, and
/// beside another's one in space.
#[test]
fn a_planes_origin_is_drawn_in_it() {
    assert_eq!(settles("std.side.origin distance(15) b"), CKind::Distance);
    assert_eq!(settles("std.front.origin distance(15) b"), CKind::Distance3);
    assert_eq!(settles("std.origin distance(10, along: x) a"), CKind::HorizontalDistance);
}

fn refused_as(stmt: &str, code: &str, needle: &str, at: &str) {
    refused(&format!("{TWO_VIEWS}{stmt}\n"), code, needle, at);
}

/// A word with no meaning in space, used across planes, is refused and says so; a selector that
/// names a page direction says nothing in space and is refused at its key.
#[test]
fn a_word_with_no_meaning_in_space_is_refused_across_planes() {
    refused_as("a horizontal b", "E062", "no meaning in space", "a horizontal b");
    refused_as("a distance(5, along: x) b", "E062", "no meaning in space", "a distance(5, along: x) b");
    refused_as("a distance(5, along: x) s", "E062", "no meaning in space", "a distance(5, along: x) s");
    refused_as("la tangent cb", "E062", "no meaning in space", "la tangent cb");
    // an angle stated as another is a relation of the plane's turns, and has none in space
    refused_as("la angle(la, lb) lb", "E062", "no meaning in space", "la angle(la, lb) lb");
    refused_as("la angle(60deg, sense: cw) lb", "E040", "unsigned", "sense");
    refused_as("a distance(5, side: left) lb", "E040", "magnitude", "side");
    // an unsigned angle cannot say which way two lines run alike
    refused_as("la angle(0deg) lb", "E040", "parallel", "0deg");
    // a point on its own plane
    refused_as("b coincident std.side", "E061", "every point of a view is on it", "b coincident std.side");
}

/// A sphere about a centre drawn in one plane: a point of another plane on it, its radius, and
/// its tangency to a line and to a second sphere, each in space.
#[test]
fn a_sphere_takes_its_words_in_space() {
    let with = |stmt: &str| format!(
        "{TWO_VIEWS}sp := sphere(hint(x: 20, y: 10)) hint(r: 12) in std.side\n\
         s2 := sphere(hint(x: 10, y: 40)) hint(r: 5) in std.front\n{stmt}\n");
    let kind = |stmt: &str| read(&with(stmt)).sketch.user_constraints()[0].kind;
    assert_eq!(kind("a coincident sp"), CKind::SphereOn);
    assert_eq!(kind("radius(12) sp"), CKind::SphereRadius);
    assert_eq!(kind("sp tangent la"), CKind::SphereTangentLine);
    assert_eq!(kind("sp tangent s2"), CKind::SphereTangentSphere);
    refused(&with("sp tangent cb"), "E040", "a sphere is tangent to a line or to another sphere", "tangent");
    refused(&with("sp tangent cb"), "E040", "a circle lying on the sphere is `c coincident s`", "tangent");
    assert_eq!(kind("cb coincident s2"), CKind::CircleOnSphere);
    // and solved: the centre held, a point of the other plane on it, and a line tangent to it
    let e = read(&with("fix(x == 20, y == 10) sp.center\na coincident sp\nfix(x == 0, y == 0) la.p1\n\
                            fix(x == 30, y == 10) la.p2\nsp tangent la"));
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let s = ent(&e, "sp");
    let c = sk.world_point(sk.round_center(s));
    let rad = sk.radius_value(s);
    let pa = sk.world_point(ent(&e, "a").i());
    assert!((norm(sub(pa, c)) - rad).abs() < 1e-9, "{} against {rad}", norm(sub(pa, c)));
    let (l1, l2) = ends(&sk, ent(&e, "la"));
    let dl = sub(l2, l1);
    let gap = norm(cross(sub(c, l1), dl)) / norm(dl);
    assert!((gap - rad).abs() < 1e-9, "the line touches the sphere: {gap} against {rad}");
    // on the sphere is one equation: a point of the other plane keeps one of its two freedoms
    let dof = |src: String| {
        let mut sk = read(&src).sketch;
        assert!(solve(&mut sk, SolveOpts::default()).success);
        diagnose(&mut sk, DiagnoseOptions::default()).dof
    };
    let tied = "fix(x == 20, y == 10) sp.center\nfix(x == 0, y == 0) la.p1\n\
                 fix(x == 30, y == 10) la.p2\nsp tangent la";
    assert_eq!(dof(with(tied)) - dof(with(&format!("{tied}\na coincident sp"))), 1);
}

/* -- the hypoid's pitch cones, and the rest of the words in space -------------------------- */

/// The hypoid by construction (the plan's P3 gate): a hypoid's pitch cones laid out through the
/// mean point in Solvent words — the pitch plane stated, the gear and pinion axial planes stood
/// square to it over the two pitch generators, the axes drawn in them from the apexes, and the
/// pitch radii, the gear's pitch angle, the shaft angle and the offset stated.
const HYPOID: &str = include_str!("fixtures/hypoid_pitch_cones.sv");

#[test]
fn the_hypoid_pitch_cones_touch_at_the_mean_point() {
    let e = read(HYPOID);
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
    assert!(d.over.is_empty() && d.conflicts.as_deref().unwrap_or(&[]).is_empty(),
            "{}", gcs_core::diagnose::summary(&d));
    assert_eq!(d.n_equations, d.structural_rank, "{}", gcs_core::diagnose::summary(&d));
    let at = |n: &str| sk.world_point(ent(&e, n).i());
    let view = |n: &str| sk.basis(ent(&e, n).i());
    let (m, o, a) = (at("M"), at("O"), at("A"));
    let ((g1, g2), (p1, p2)) = (ends(&sk, ent(&e, "gax")), ends(&sk, ent(&e, "pax")));
    let (ag, ap) = (unit(sub(g2, g1)), unit(sub(p2, p1)));
    let (np, ng, nq) = ([0.0, -1.0, 0.0], view("G").normal(), view("Q").normal());
    let mut worst: Vec<(&str, f64)> = Vec::new();
    // each axis starts at its own apex, as drawn in the pitch plane
    worst.push(("gear axis through O", norm(sub(g1, o))));
    worst.push(("pinion axis through Ap", norm(sub(p1, a))));
    // each axial plane is square to P and holds its generator and its axis
    for (what, n, apex, axis) in [("G", ng, o, ag), ("Q", nq, a, ap)] {
        worst.push((what, dot(n, np).abs()));
        worst.push((what, dot(n, unit(sub(apex, m))).abs()));
        worst.push((what, dot(n, sub(m, view(what).o)).abs()));
        worst.push((what, dot(n, axis).abs()));
    }
    // the shaft angle and the offset, from the lifted axes alone
    worst.push(("shaft angle 90°", dot(ag, ap).abs()));
    let mm = cross(ag, ap);
    let offset = dot(mm, sub(p1, g1)).abs() / norm(mm);
    worst.push(("offset 20", (offset - 20.0).abs()));
    // M's distances from the two axes: the pitch radii
    worst.push(("gear pitch radius 96", (off_line(m, g1, ag) - 96.0).abs()));
    worst.push(("pinion pitch radius 48", (off_line(m, p1, ap) - 48.0).abs()));
    // the gear's pitch angle, between the generator from its apex and its axis
    let gamma = dot(unit(sub(m, o)), ag).acos().to_degrees();
    worst.push(("gear pitch angle 60°", (gamma - 60.0).abs()));
    // **the common pitch plane**: each cone's surface normal at M — in the plane of its axis and
    // its generator, square to the generator — is P's normal, so P touches both cones along
    // their generators at M
    for (what, apex, axis) in [("gear cone", o, ag), ("pinion cone", a, ap)] {
        let g = unit(sub(m, apex));
        worst.push((what, dot(np, g).abs()));
        worst.push((what, dot(np, unit(cross(g, axis))).abs()));
        let normal = unit(cross(g, cross(g, axis)));
        worst.push((what, norm(cross(normal, np))));
    }
    for (what, r) in &worst {
        eprintln!("{what}: {r:.3e}");
        assert!(*r < 1e-9, "{what}: {r}");
    }
    // the pinion's cone came out as the hypoid's, not the bevel's: its apex off the gear's
    // generator, and its pitch angle what the offset leaves (cos ε = tan Γ·tan γ)
    let eps = dot(unit(sub(a, m)), unit(sub(o, m))).acos();
    let gp = dot(unit(sub(m, a)), ap).acos();
    eprintln!("ε = {:.6}°, γ = {:.6}°, |MA| = {:.6}", eps.to_degrees(), gp.to_degrees(), norm(sub(a, m)));
    assert!(eps.to_degrees() > 1.0);
    assert!((eps.cos() - 60f64.to_radians().tan() * gp.tan()).abs() < 1e-9);
}

/// A sketch lifted to a program and read back: the text, and the drawing it elaborates to.
fn relift(sk: &Sketch) -> (String, Elaborated) {
    let text = gcs_core::program::to_program(sk).text().to_string();
    let again = read(&text);
    (text, again)
}

/// Every point of `a` where the same point of `b` is, in space.
fn same_points(a: &Sketch, b: &Sketch) {
    assert_eq!(a.points.len(), b.points.len());
    for i in 0..a.points.len() {
        let (x, y) = (a.world_point(i), b.world_point(i));
        assert!(norm(sub(x, y)) < 1e-7, "p{i}: {x:?} against {y:?}");
    }
}

/// **A solved plane lifts as the axes that solve it**: the axis seeded where the solve left it,
/// the plane over it, and read back it solves to the same place with the same freedoms.
#[test]
fn a_lifted_program_keeps_its_rays() {
    let e = read(&format!("{AXES}{AXES_IN_WORDS}"));
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let (text, again) = relift(&sk);
    assert!(text.contains(" := axis") && text.contains(" := plane(u: "), "{text}");
    assert!(text.contains("angle(90") && text.contains("distance(17.5)"), "{text}");
    let mut back = again.sketch.clone();
    let kinds: Vec<CKind> = back.user_constraints().iter().map(|c| c.kind).collect();
    assert!(kinds.contains(&CKind::Angle3) && kinds.contains(&CKind::LineLine3), "{kinds:?}\n{text}");
    // seeded where the solve left it: already solved, and nothing moves
    let before: Vec<[f64; 3]> = (0..back.points.len()).map(|i| back.world_point(i)).collect();
    assert!(solve(&mut back, SolveOpts::default()).success);
    for (i, x) in before.iter().enumerate() {
        assert!(norm(sub(*x, back.world_point(i))) < 1e-7, "p{i} moved");
    }
    same_points(&sk, &back);
    assert_eq!(diagnose(&mut back, DiagnoseOptions::default()).dof, 0);
}

/// The hypoid's planes over its generators lift as planes over the lines, not as stated axes
/// beside them, and read back solve to the same points.
#[test]
fn a_lifted_program_keeps_its_planes_over_lines() {
    let e = read(HYPOID);
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let (text, again) = relift(&sk);
    let count = |s: &Sketch, k: CKind| s.constraints.iter().filter(|c| c.kind == k).count();
    let mut back = again.sketch.clone();
    for k in [CKind::Parallel3, CKind::PointOnPlane, CKind::ProjectSolved] {
        assert_eq!(count(&back, k), count(&sk, k), "{k:?}\n{text}");
    }
    assert!(solve(&mut back, SolveOpts::default()).success);
    same_points(&sk, &back);
    let d = diagnose(&mut back, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
}

/// **A circle on a sphere**: `c coincident s` puts every point of a circle drawn in one plane on
/// a sphere about a centre drawn in another — the sphere's centre on the circle's axis, and the
/// radii and the gap a right triangle.  The toe or heel circle of a gear blank on its end sphere.
#[test]
fn a_circle_on_a_sphere_is_on_it_all_the_way_round() {
    for free in [false, true] {
        let side = if free {
            "a := axis hint(x: 0, y: 1, z: 0)\nb := axis hint(x: 0, y: 0, z: 1)\n\
             side := plane(u: a, v: b) hint(x: 120, y: 0, z: 0)"
        } else {
            ""
        };
        let side_name = if free { "side" } else { "std.side" };
        let doc = |tie: &str| format!("\
unit mm
use std
{side}
s := sphere(hint(x: 10, y: 20)) hint(r: 30) in std.front
radius(30) s
k := circle(hint(x: 15, y: 12)) hint(r: 15) in {side_name}
radius(18) k
{tie}
");
        let e = read(&doc("k coincident s"));
        let mut sk = e.sketch.clone();
        assert!(sk.user_constraints().iter().any(|c| c.kind == CKind::CircleOnSphere));
        let r = solve(&mut sk, SolveOpts::default());
        assert!(r.success, "{}", r.message);
        let (s, k) = (ent(&e, "s"), ent(&e, "k"));
        let (sc, kc) = (sk.world_point(sk.round_center(s)), sk.world_point(sk.round_center(k)));
        let b = sk.basis(sk.plane_of(sk.round_center(k)).unwrap());
        let (n, u, v) = (b.normal(), b.u, b.v);
        // the sphere's centre on the circle's axis, and every point of the circle 30 from it
        assert!(norm(cross(sub(sc, kc), n)) < 1e-9, "off the axis: {sc:?} {kc:?}");
        for i in 0..12 {
            let a = i as f64 * std::f64::consts::TAU / 12.0;
            let x = [0, 1, 2].map(|t| kc[t] + 18.0 * (a.cos() * u[t] + a.sin() * v[t]));
            assert!((norm(sub(x, sc)) - 30.0).abs() < 1e-9, "{}", norm(sub(x, sc)));
        }
        // three equations, independent wherever the circle is off the sphere's centre
        let d = diagnose(&mut sk, DiagnoseOptions::default());
        let mut wk = read(&doc("")).sketch;
        let dw = diagnose(&mut wk, DiagnoseOptions::default());
        assert_eq!(dw.dof, d.dof + 3, "three equations");
        assert!(d.over.is_empty(), "{}", gcs_core::diagnose::summary(&d));
    }
}

/// The midpoint and the mirror in a line, across planes, are the same statements in space.
#[test]
fn the_midpoint_and_the_mirror_read_in_space() {
    let e = read(&format!("{TWO_VIEWS}\
fix(x == 5, y == 3) lb.p1
fix(x == 0, y == 0) la.p1
fix(x == 30, y == 10) la.p2
c := point hint(x: 20, y: 5) in std.front
c midpoint lb
d := point hint(x: 5, y: 30) in std.front
f := point hint(x: 10, y: 30) in std.side
d symmetry(la) f
"));
    let mut sk = e.sketch.clone();
    let kinds: Vec<CKind> = sk.user_constraints().iter().map(|c| c.kind).collect();
    assert!(kinds.contains(&CKind::Midpoint3) && kinds.contains(&CKind::Symmetric3), "{kinds:?}");
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let at = |n: &str| sk.world_point(ent(&e, n).i());
    let (b1, b2) = ends(&sk, ent(&e, "lb"));
    let mid = [0, 1, 2].map(|t| 0.5 * (b1[t] + b2[t]));
    assert!(norm(sub(at("c"), mid)) < 1e-9, "{:?} against {mid:?}", at("c"));
    // f is d turned half round la: their midpoint on la, and their chord square to it
    let (a1, a2) = ends(&sk, ent(&e, "la"));
    let (dd, ff) = (at("d"), at("f"));
    let m = [0, 1, 2].map(|t| 0.5 * (dd[t] + ff[t]));
    let dir = sub(a2, a1);
    assert!(norm(cross(sub(m, a1), dir)) / norm(dir) < 1e-9, "the midpoint is on the line");
    assert!(dot(sub(ff, dd), dir).abs() < 1e-9, "the chord is square to the line");
}

