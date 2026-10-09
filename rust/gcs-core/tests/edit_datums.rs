//! **A gesture writes a datum** (#162 F1): an axis along a line, through two points, or along a
//! plane's normal through a point — the axis and the relations that say so, one splice, seeded
//! with the direction the drawing already has — and a plane over two axes, refused where the
//! language refuses one.

use gcs_core::edit::{self, AxisOn, Kind};
use gcs_core::program::{self, Elaborated};
use gcs_core::{library, solve};

/// On the front: `a` at the origin, `c` at (30, 40) — in space (30, 0, 40) — and `ab` along x.
const DRAWN: &str = "unit mm\nuse std\nin std.front {\n\
    a := point hint((0, 0))\nb := point hint((30, 0))\nc := point hint((30, 40))\n\
    ab := line(a, b)\nfix((0, 0)) a\nfix((30, 0)) b\nfix((30, 40)) c\n}\n";

fn build(src: &str) -> Elaborated {
    let (p, errors, linked) = library::parse_linked(src);
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}\n{src}");
    let e = program::elaborate(&p);
    assert!(e.ok(), "{:?}\n{src}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    e
}

/// The axis an edit wrote, solved: its direction, and how far each named point stands off it.
fn solved_axis(text: &str, name: &str, on: &[&str]) -> ([f64; 3], Vec<f64>) {
    let mut e = build(text);
    assert!(solve::solve(&mut e.sketch, Default::default()).success, "{text}");
    let sk = &e.sketch;
    let ax = &sk.axes[e.map.ent_named(name).unwrap().i()];
    let d = ax.d.map(|q| sk.params[q as usize].value);
    let a = ax.a.map(|q| sk.params[q as usize].value);
    let off = on.iter().map(|n| {
        let p = sk.world_point(e.map.ent_named(n).unwrap().i());
        let r = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
        let along = r[0] * d[0] + r[1] * d[1] + r[2] * d[2];
        (0..3).map(|k| (r[k] - along * d[k]).powi(2)).sum::<f64>().sqrt()
    }).collect();
    (d, off)
}

fn near(a: [f64; 3], b: [f64; 3]) -> bool {
    // an axis is a direction either way along
    let same = (0..3).all(|k| (a[k] - b[k]).abs() < 1e-9);
    let back = (0..3).all(|k| (a[k] + b[k]).abs() < 1e-9);
    same || back
}

#[test]
fn an_axis_through_two_points_is_seeded_their_way_and_holds_both() {
    let e = build(DRAWN);
    let on = AxisOn::Points("a".into(), "c".into());
    let out = edit::add_axis(&e, &e.sketch, &on, None);
    assert_eq!((out.refused.as_deref(), out.kind, out.names.clone()), (None, Kind::Structural, vec!["x0".into()]));
    assert!(out.text.ends_with("x0 := axis hint(dir: (0.6, 0, 0.8))\na coincident x0\nc coincident x0\n"),
        "{}", out.text);
    let (d, off) = solved_axis(&out.text, "x0", &["a", "c"]);
    assert!(near(d, [0.6, 0.0, 0.8]), "{d:?}");
    assert!(off.iter().all(|o| *o < 1e-7), "{off:?}");
}

#[test]
fn an_axis_along_a_line_lies_on_it() {
    let e = build(DRAWN);
    let out = edit::add_axis(&e, &e.sketch, &AxisOn::Line("ab".into()), Some("hinge"));
    assert!(out.text.ends_with("hinge := axis hint(dir: (1, 0, 0))\nab coincident hinge\n"), "{}", out.text);
    let (d, off) = solved_axis(&out.text, "hinge", &["a", "b"]);
    assert!(near(d, [1.0, 0.0, 0.0]), "{d:?}");
    assert!(off.iter().all(|o| *o < 1e-7), "{off:?}");
}

#[test]
fn an_axis_along_a_planes_normal_stands_square_to_it_through_the_point() {
    let e = build(DRAWN);
    let on = AxisOn::Normal { plane: "std.front".into(), point: "c".into() };
    let out = edit::add_axis(&e, &e.sketch, &on, None);
    // seeded toward the front's viewer, its normal
    assert!(out.text.contains("x0 := axis hint(dir: (0, -1, 0))\nx0 perpendicular std.front\n\
        c coincident x0\n"), "{}", out.text);
    let (d, off) = solved_axis(&out.text, "x0", &["c"]);
    assert!(near(d, [0.0, 1.0, 0.0]), "{d:?}");
    assert!(off[0] < 1e-7, "{off:?}");
}

#[test]
fn an_axis_without_a_direction_or_over_the_wrong_kind_is_refused() {
    let e = build(DRAWN);
    let refused = |on: AxisOn, name: Option<&str>| {
        let out = edit::add_axis(&e, &e.sketch, &on, name);
        assert_eq!(out.text, e.program.text());
        out.refused.unwrap_or_default()
    };
    assert!(refused(AxisOn::Points("a".into(), "a".into()), None).contains("no direction"));
    assert!(refused(AxisOn::Line("a".into()), None).contains("is not a line"));
    assert!(refused(AxisOn::Line("ab".into()), Some("ab")).contains("already a name"));
}

#[test]
fn a_plane_over_axes_that_make_none_is_refused_at_the_gesture() {
    let e = build(DRAWN);
    let fine = edit::add_plane(&e.program, &["std.x".into(), "std.y".into()], None);
    assert_eq!(fine.refused, None);
    assert!(fine.text.ends_with("v0 := plane(u: std.x, v: std.y)\n"), "{}", fine.text);
    let out = edit::add_plane(&e.program, &["std.x".into(), "std.x".into()], None);
    assert!(out.refused.is_some(), "{}", out.text);
    assert_eq!(out.text, e.program.text());
}
