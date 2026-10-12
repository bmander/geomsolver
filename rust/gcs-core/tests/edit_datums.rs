//! **A button drops a datum** (#162 F1): a free plane or axis, constrained to what is selected —
//! through a point, along a line, square to a plane, where two planes meet — each relation an
//! ordinary statement, the whole one splice, seeded off the drawing so the first solve lands on
//! the answer meant.  `edit::add_datum`'s table, row by row: the text, and the geometry solved.

use gcs_core::edit::{self, Kind};
use gcs_core::model::{EntKind, Sketch};
use gcs_core::program::{self, Elaborated};
use gcs_core::{library, solve};

/// On the front: `a` at the origin, `m` and `b` along x, `c` at (30, 40) — in space (30, 0, 40)
/// — with `ab` along x and a circle `k`; `s` in space at (10, 20, 30); and an axis `t` along y
/// through (10, 0, 0), where it crosses `ab`.
const DRAWN: &str = "unit mm\nuse std\nin std.front {\n\
    a := point hint((0, 0))\nm := point hint((15, 0))\nb := point hint((30, 0))\n\
    c := point hint((30, 40))\nab := line(a, b)\nk := circle(center: c) hint(r: 5)\n\
    fix((0, 0)) a\nfix((15, 0)) m\nfix((30, 0)) b\nfix((30, 40)) c\nfix(r == 5) k\n}\n\
    s := point hint((10, 20, 30))\nfix((10, 20, 30)) s\n\
    t := axis hint(dir: (0, 1, 0))\nfix(dir == (0, 1, 0), origin == (10, 0, 0)) t\n";

fn build(src: &str) -> Elaborated {
    let (p, errors, linked) = library::parse_linked(src);
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}\n{src}");
    let e = program::elaborate(&p);
    assert!(e.ok(), "{:?}\n{src}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    e
}

/// Drop a datum over the drawing, and the document it makes, solved.
fn drop(kind: EntKind, from: &[&str], current: Option<&str>) -> (String, Elaborated) {
    let e = build(DRAWN);
    let from: Vec<String> = from.iter().map(|n| n.to_string()).collect();
    let out = edit::add_datum(&e, &e.sketch, kind, &from, current, None);
    assert_eq!(out.refused, None, "{from:?}");
    assert_eq!(out.kind, Kind::Structural);
    let mut next = build(&out.text);
    assert!(solve::solve(&mut next.sketch, Default::default()).success, "{}", out.text);
    (out.text, next)
}

fn refused(kind: EntKind, from: &[&str]) -> String {
    let e = build(DRAWN);
    let from: Vec<String> = from.iter().map(|n| n.to_string()).collect();
    let out = edit::add_datum(&e, &e.sketch, kind, &from, Some("std.front"), None);
    assert_eq!(out.text, e.program.text());
    out.refused.unwrap_or_else(|| panic!("{from:?} is not refused"))
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn len(a: [f64; 3]) -> f64 { dot(a, a).sqrt() }
/// Alike either way along.
fn along(a: [f64; 3], b: [f64; 3]) -> bool { len(cross(a, b)) < 1e-7 * len(a) * len(b) }

fn point(e: &Elaborated, n: &str) -> [f64; 3] {
    e.sketch.world_point(e.map.ent_named(n).unwrap().i())
}

/// A plane's normal and origin, solved.
fn plane(e: &Elaborated, n: &str) -> ([f64; 3], [f64; 3]) {
    let b = e.sketch.basis(e.map.ent_named(n).unwrap().i());
    (b.normal(), b.o)
}

/// An axis's direction and its place, solved.
fn axis(e: &Elaborated, n: &str) -> ([f64; 3], [f64; 3]) {
    let sk: &Sketch = &e.sketch;
    let ax = &sk.axes[e.map.ent_named(n).unwrap().i()];
    (ax.d.map(|q| sk.params[q as usize].value), ax.a.map(|q| sk.params[q as usize].value))
}

fn on_plane(e: &Elaborated, pl: &str, x: [f64; 3]) -> bool {
    let (n, o) = plane(e, pl);
    dot(sub(x, o), n).abs() < 1e-7
}

fn on_axis(e: &Elaborated, ax: &str, x: [f64; 3]) -> bool {
    let (d, a) = axis(e, ax);
    len(cross(sub(x, a), d)) < 1e-7
}

#[test]
fn a_plane_dropped_on_nothing_is_free_and_set_off_the_one_being_drawn_on() {
    let (text, e) = drop(EntKind::Plane, &[], Some("std.front"));
    assert!(text.contains("v0 := plane(u: hint(dir: (1, 0, 0)), v: hint(dir: (0, 0, 1))) hint(origin: (0, "),
        "{text}");
    let (n, o) = plane(&e, "v0");
    assert!(along(n, [0.0, 1.0, 0.0]) && o[1].abs() > 1.0, "{n:?} {o:?}");
}

#[test]
fn a_plane_through_a_point_and_square_to_a_ray() {
    let (text, e) = drop(EntKind::Plane, &["c"], Some("std.top"));
    assert!(text.contains("v0 coincident c\n"), "{text}");
    assert!(on_plane(&e, "v0", point(&e, "c")));
    // square to an axis, said outright
    let (text, e) = drop(EntKind::Plane, &["s", "t"], None);
    assert!(text.contains("v0 coincident s\nt perpendicular v0\n"), "{text}");
    assert!(on_plane(&e, "v0", point(&e, "s")) && along(plane(&e, "v0").0, [0.0, 1.0, 0.0]));
    // square to a drawn line, by standing square to both the plane's own axes
    let (text, e) = drop(EntKind::Plane, &["s", "ab"], None);
    assert!(text.contains("v0 coincident s\nab perpendicular v0.u\nab perpendicular v0.v\n"), "{text}");
    assert!(on_plane(&e, "v0", point(&e, "s")) && along(plane(&e, "v0").0, [1.0, 0.0, 0.0]));
}

#[test]
fn a_plane_holds_two_rays_or_three_points() {
    let (text, e) = drop(EntKind::Plane, &["ab", "t"], None);
    assert!(text.contains("ab coincident v0\nt coincident v0\n"), "{text}");
    assert!(along(plane(&e, "v0").0, [0.0, 0.0, 1.0]));
    assert!(on_plane(&e, "v0", point(&e, "b")));
    let (text, e) = drop(EntKind::Plane, &["a", "b", "s"], None);
    assert!(text.contains("v0 coincident a\nv0 coincident b\nv0 coincident s\n"), "{text}");
    for p in ["a", "b", "s"] {
        assert!(on_plane(&e, "v0", point(&e, p)), "{p}");
    }
}

/// An offset plane (#162 F5): parallel, and as far off as it is set, said — so the number is
/// there to size.
#[test]
fn a_plane_from_a_plane_is_parallel_and_set_off_by_a_stated_distance() {
    let (text, e) = drop(EntKind::Plane, &["std.top"], None);
    assert!(text.contains("v0 parallel std.top\n"), "{text}");
    let d: f64 = text.split("std.top distance(").nth(1).and_then(|t| t.split(") v0\n").next())
        .and_then(|t| t.parse().ok()).unwrap_or_else(|| panic!("no distance stated\n{text}"));
    let (n, o) = plane(&e, "v0");
    assert!(along(n, [0.0, 0.0, 1.0]) && (o[2] - d).abs() < 1e-9 && d > 1.0, "{n:?} {o:?} {d}");
}

/// A 40 × 30 block on the front plane (normal −y), 20 deep behind it.
const BLOCK: &str = "use std\nin std.front {\n  a := point hint((0, 0))\n  b := point hint((40, 0))\n\
    c := point hint((40, 30))\n  d := point hint((0, 30))\n  ab := line(a, b)\n\
    bc := line(b, c)\n  cd := line(c, d)\n  da := line(d, a)\n}\n\
    fix((0, 0)) a\nfix((40, 0)) b\nfix((40, 30)) c\nfix((0, 30)) d\n\
    f0 := face(ab, bc, cd, da)\nb0 := solid(f0, depth: 20)\n";

/// Sketch on `face` of the block where the eye's ray met it, and the document it makes, solved.
fn on_face(face: &str, at: Option<[f64; 3]>) -> (String, Elaborated) {
    let mut e = build(BLOCK);
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    let b0 = e.map.ent_named("b0").unwrap().i();
    let out = edit::sketch_on_face(&e, &e.sketch, b0, face, at, None);
    assert_eq!(out.refused, None, "{face}");
    assert_eq!(out.kind, Kind::Structural);
    let mut next = build(&out.text);
    assert!(solve::solve(&mut next.sketch, Default::default()).success, "{}", out.text);
    (out.text, next)
}

/// **Sketch on a face** (#162 F5): a plane lying on the face, its origin where the eye's ray met
/// the face, said as the set the face is — and found lying there afterwards, so a second sketch
/// on the face is drawn on it.
#[test]
fn sketching_on_a_cap_writes_a_plane_touching_it_where_it_was_clicked() {
    let (text, e) = on_face("b0.far", Some([10.0, 23.0, 12.0]));
    assert!(text.contains("v0 := plane(u: hint(dir: (1, 0, 0)), v: hint(dir: (0, 0, 1))) \
                           hint(origin: (10, 20, 12))\n"), "{text}");
    assert!(text.ends_with("v0.origin coincident b0.far.surface\n\
                            v0 tangent(at: v0.origin) b0.far.surface\n"), "{text}");
    let (n, o) = plane(&e, "v0");
    assert!(along(n, [0.0, 1.0, 0.0]) && (o[1] - 20.0).abs() < 1e-9, "{n:?} {o:?}");
    let b0 = e.map.ent_named("b0").unwrap().i();
    assert_eq!(edit::face_plane(&e, &e.sketch, b0, "b0.far").as_deref(), Some("v0"));
    // the cap on the profile is the profile's own plane
    assert_eq!(edit::face_plane(&e, &e.sketch, b0, "b0.near").as_deref(), Some("std.front"));
    assert_eq!(edit::face_plane(&e, &e.sketch, b0, "b0.cd"), None);
}

/// A side is the plane through its edge along the profile's normal; with no click the sketch
/// starts at the side's middle.
#[test]
fn sketching_on_a_side_starts_at_its_middle() {
    let (text, e) = on_face("b0.cd", None);
    assert!(text.contains("hint(origin: (20, 10, 30))"), "{text}");
    let (n, o) = plane(&e, "v0");
    assert!(along(n, [0.0, 0.0, 1.0]) && (o[2] - 30.0).abs() < 1e-9, "{n:?} {o:?}");
}

#[test]
fn a_face_with_nothing_to_sketch_on_is_refused() {
    let e = build(BLOCK);
    let b0 = e.map.ent_named("b0").unwrap().i();
    let out = edit::sketch_on_face(&e, &e.sketch, b0, "b0.top", None, None);
    assert!(out.refused.is_some_and(|m| m.contains("names no face")));
    assert_eq!(out.text, e.program.text());
}

#[test]
fn an_axis_dropped_on_nothing_or_a_point_stands_square_to_the_plane_drawn_on() {
    let (text, e) = drop(EntKind::Axis, &[], Some("std.front"));
    assert!(text.contains("x0 := axis hint(dir: (0, -1, 0), origin: ("), "{text}");
    assert!(along(axis(&e, "x0").0, [0.0, 1.0, 0.0]));
    let (text, e) = drop(EntKind::Axis, &["c"], Some("std.front"));
    assert!(text.contains("c coincident x0\n"), "{text}");
    assert!(on_axis(&e, "x0", point(&e, "c")) && along(axis(&e, "x0").0, [0.0, 1.0, 0.0]));
}

#[test]
fn an_axis_through_two_points_or_along_a_line() {
    let (text, e) = drop(EntKind::Axis, &["a", "c"], None);
    assert!(text.contains("x0 := axis hint(dir: (0.6, 0, 0.8))\na coincident x0\nc coincident x0\n"),
        "{text}");
    assert!(on_axis(&e, "x0", point(&e, "a")) && on_axis(&e, "x0", point(&e, "c")));
    let (text, e) = drop(EntKind::Axis, &["ab"], None);
    assert!(text.contains("x0 := axis hint(dir: (1, 0, 0))\nab coincident x0\n"), "{text}");
    assert!(on_axis(&e, "x0", point(&e, "m")));
}

#[test]
fn an_axis_from_planes_is_a_normal_or_where_two_meet() {
    let (text, e) = drop(EntKind::Axis, &["s", "std.top"], None);
    assert!(text.contains("s coincident x0\nx0 perpendicular std.top\n"), "{text}");
    assert!(on_axis(&e, "x0", point(&e, "s")) && along(axis(&e, "x0").0, [0.0, 0.0, 1.0]));
    let (text, e) = drop(EntKind::Axis, &["std.top"], None);
    assert!(text.contains("x0 perpendicular std.top\nstd.top.origin coincident x0\n"), "{text}");
    assert!(on_axis(&e, "x0", [0.0, 0.0, 0.0]));
    let (text, e) = drop(EntKind::Axis, &["std.front", "std.side"], None);
    assert!(text.contains("x0 coincident std.front\nx0 coincident std.side\n"), "{text}");
    let (d, _) = axis(&e, "x0");
    assert!(along(d, [0.0, 0.0, 1.0]) && on_axis(&e, "x0", [0.0, 0.0, 7.0]), "{d:?}");
}

#[test]
fn a_selection_no_rule_takes_or_that_says_nothing_is_refused_with_the_cause() {
    assert!(refused(EntKind::Axis, &["a", "a"]).contains("one place"));
    assert!(refused(EntKind::Plane, &["a", "m", "b"]).contains("on a line"));
    assert!(refused(EntKind::Axis, &["std.front", "std.front"]).contains("parallel"));
    assert!(refused(EntKind::Plane, &["k"]).contains("does not place"));
    assert!(refused(EntKind::Axis, &["a", "b", "c"]).contains("an axis is dropped"));
    assert!(refused(EntKind::Plane, &["nothing"]).contains("nothing in this drawing"));
}

/// The user's sketch (#170): a plane through a free point, square to the line from the front's
/// origin to it.  Dragging the point carries the plane — through the point, square to the line —
/// at every frame.
#[test]
fn dragging_a_point_carries_a_plane_held_to_it() {
    use gcs_core::decompose::PlanDrag;
    let src = "use std\np0 := point hint((0.235, 0.18)) in std.front\n\
        l0 := line(std.front.origin, p0) in std.front\n\
        v0 := plane(u: hint(dir: (-0.608, 0, 0.794)), v: hint(dir: (0, -1, 0))) \
        hint(origin: (0.235, 0, 0.18))\np0 coincident v0\nl0 perpendicular v0.u\n\
        l0 perpendicular v0.v\n";
    let mut e = build(src);
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    let p = e.map.ent_named("p0").unwrap().i();
    let (x0, y0) = e.sketch.point_xy(p);
    let mut drag = PlanDrag::new(&e.sketch, p, x0, y0, None, 0.05);
    for k in 1..=10 {
        let (x, y) = (x0 + 0.1 * k as f64, y0 + 0.05 * k as f64);
        let res = drag.move_to(&mut e.sketch, None, x, y);
        assert!(res.success, "frame {k}: {res:?}");
        let at = e.sketch.point_xy(p);
        assert!((at.0 - x).hypot(at.1 - y) < 1e-6, "frame {k}: the point lags the cursor: {at:?}");
        let world = e.sketch.world_point(p);
        assert!(on_plane(&e, "v0", world), "frame {k}: the plane stayed behind");
        assert!(along(plane(&e, "v0").0, world), "frame {k}: the plane is not square to the line");
    }
    drag.end();
}
