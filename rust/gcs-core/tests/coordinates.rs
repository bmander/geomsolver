//! Signed datum coordinates retain the shared solve and independent plane membership.
use gcs_core::{callout, diagnose, io, library, program, solve, syntax};
use gcs_core::constraints::CKind;
use gcs_core::model::Sketch;

fn build(src: &str) -> program::Elaborated {
    let (p, errors, linked) = library::parse_linked(src);
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}");
    let e = program::elaborate(&p);
    assert!(e.ok(), "{:?}", e.diags);
    e
}

fn solved(sk: &mut Sketch) {
    let r = solve::solve(sk, Default::default());
    assert!(r.success, "{}", r.message);
}

fn close(a: (f64, f64), b: (f64, f64)) {
    assert!((a.0 - b.0).hypot(a.1 - b.1) < 1e-6, "{a:?} != {b:?}");
}

#[test]
fn signed_coordinates_follow_a_moving_datum_even_from_the_wrong_quadrant() {
    let mut e = build("unit mm\npoint o hint(x: 10, y: 20)\npoint q hint(x: 14, y: 23)\nground o\nground q\nplane f(origin: o, toward: q)\npoint p hint(x: 100, y: -200)\np distance(-5mm, along: u) f\np distance(2mm, along: v) f\n");
    let p = e.map.ent_named("p").unwrap().i();
    solved(&mut e.sketch);
    close(e.sketch.point_xy(p), (4.8, 18.6));
    let q = e.map.ent_named("q").unwrap().i();
    let [x, y] = e.sketch.point_params(q);
    e.sketch.params[x as usize].value = 7.0;
    e.sketch.params[y as usize].value = 24.0;
    solved(&mut e.sketch);
    close(e.sketch.point_xy(p), (11.4, 14.8));
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
    let back = io::loads(&io::dumps(&e.sketch, None)).unwrap();
    assert_eq!(back.constraints.iter().filter(|c| matches!(c.kind, CKind::CoordinateU | CKind::CoordinateV)).count(), 2);
    let c = e.sketch.constraints.iter().find(|c| c.kind == CKind::CoordinateU).unwrap();
    let Some(callout::Frame::Linear { d, .. }) = callout::frame(&e.sketch, c) else { panic!("no ordinate callout") };
    close(d, (-0.6, 0.8));
}

#[test]
fn an_external_constraint_moves_the_datum_through_an_aliased_component_point() {
    let mut e = build("unit mm\ncomponent Offset(f: plane, p: point) {\np distance(3mm, along: u) f\np distance(-2mm, along: v) f\n}\npoint o hint(x: 10, y: 20)\npoint q hint(x: 14, y: 20)\nline axis(o, q)\nhorizontal axis\no distance(4mm, along: x) q\nplane f(origin: o, toward: q)\npoint p hint(x: 23, y: 28)\nground p\ni: Offset(f, p)\n");
    solved(&mut e.sketch);
    close(e.sketch.point_xy(e.map.ent_named("o").unwrap().i()), (20.0, 30.0));
    assert_eq!(e.sketch.points.len(), 3, "the argument is an alias");
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
}

#[test]
fn both_ordinates_can_be_solved_as_independent_dimensions() {
    let mut e = build("unit mm\ncomponent Read(f: plane, p: point, u: Length, v: Length) {\np distance(u, along: u) f\np distance(v, along: v) f\n}\npoint o hint(x: 10, y: 20)\npoint q hint(x: 14, y: 23)\npoint p hint(x: 4.8, y: 18.6)\nground o\nground q\nground p\nplane f(origin: o, toward: q)\nr: Read(f, p)\n");
    super::common::fd_jacobian(&e.sketch, 1e-6);
    solved(&mut e.sketch);
    let value = |name: &str| e.sketch.params[e.sketch.free_vars[name] as usize].value;
    assert!((value("r.u") + 5.0).abs() < 1e-6);
    assert!((value("r.v") - 2.0).abs() < 1e-6);
}

#[test]
fn loc_seeds_read_the_datum_and_membership_passes_through_nested_instances() {
    let mut e = build("unit mm\nuse std\ncomponent Part(f: plane) { p: Loc(f, u: -5mm, v: 2mm) }\npoint o hint(x: 10, y: 20)\npoint q hint(x: o.x + 4mm, y: o.y + 3mm)\nground o\nground q\nplane datum(origin: o, toward: q)\nplane view\ni: Part(datum) in view\n");
    let p = e.map.ent_named("i.p.p").unwrap().i();
    close(e.sketch.point_xy(p), (4.8, 18.6));
    let view = e.map.ent_named("view").unwrap().i();
    assert_eq!(e.sketch.points[p].plane, Some(view as u32));
    assert_eq!(e.sketch.points[e.map.ent_named("o").unwrap().i()].plane, None);
    solved(&mut e.sketch);
    close(e.sketch.point_xy(p), (4.8, 18.6));
}

#[test]
fn zero_ordinates_are_regular_and_claims_remain_assertions() {
    let mut e = build("unit mm\npoint o hint(x: 10, y: 20)\npoint q hint(x: 14, y: 23)\nground o\nground q\nplane f(origin: o, toward: q)\npoint p\np distance(0mm, along: u) f\np distance(0mm, along: v) f\nclaim p distance(0mm, along: u) f\n");
    solved(&mut e.sketch);
    close(e.sketch.point_xy(e.map.ent_named("p").unwrap().i()), (10.0, 20.0));
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).status, diagnose::State::Well);
}

#[test]
fn datum_axes_require_a_datum_and_lengths() {
    for relation in ["p distance(2mm, along: u) q", "p distance(2mm, along: x) f", "p distance(2deg, along: u) f", "p distance(2mm, along: z) f"] {
        let source = format!("unit mm\npoint p\npoint q\nplane f\n{relation}\n");
        let (p, errs) = syntax::parse(&source);
        assert!(errs.is_empty(), "{errs:?}");
        assert!(!program::elaborate(&p).ok(), "accepted {relation}");
    }
}

#[test]
fn a_datum_child_remains_a_point_through_aliases_and_face_corners() {
    let e = build("unit mm\ncomponent Inner(p: point) {\npoint q hint(x: p.x + 1mm, y: p.y + 2mm)\nline edge(p, q)\n}\ncomponent Outer(f: plane) {\ni: Inner(f.origin)\nface triangle(f.origin, i.q, f.toward, -> close)\n}\npoint o hint(x: 10, y: 20)\npoint t hint(x: 14, y: 23)\nplane f(origin: o, toward: t)\nx: Outer(f)\n");
    let edge = &e.sketch.lines[e.map.ent_named("x.i.edge").unwrap().i()];
    assert_eq!(edge.p1 as usize, e.map.ent_named("o").unwrap().i());
    close(e.sketch.point_xy(edge.p2 as usize), (11.0, 22.0));
    assert_eq!(e.sketch.points.len(), 3);
    assert_eq!(e.sketch.faces.len(), 1);
}

#[test]
fn loc_can_leave_both_coordinates_unknown() {
    let mut e = build("unit mm\nuse std\npoint o hint(x: 10, y: 20)\npoint q hint(x: 14, y: 23)\nground o\nground q\nplane f(origin: o, toward: q)\ni: Loc(f)\npoint target hint(x: 4.8, y: 18.6)\nground target\ni.p coincident target\n");
    solved(&mut e.sketch);
    assert!((e.sketch.params[e.sketch.free_vars["i.u"] as usize].value + 5.0).abs() < 1e-6);
    assert!((e.sketch.params[e.sketch.free_vars["i.v"] as usize].value - 2.0).abs() < 1e-6);
}

#[test]
fn vtwin_datums_follow_a_complete_crank_turn() {
    let mut e = build(gcs_core::examples::source("vtwin").unwrap());
    solved(&mut e.sketch);
    let theta = e.sketch.free_vars["crank.theta"] as usize;
    e.sketch.params[theta].fixed = true;
    let point = |e: &program::Elaborated, name: &str| {
        e.sketch.point_xy(e.map.ent_named(name).unwrap().i())
    };
    for step in 0..=8 {
        e.sketch.params[theta].value = 180.0 + step as f64 * 45.0;
        solved(&mut e.sketch);
        let pin = point(&e, "crank.pin");
        for (bank, pivot) in [("bankR", "plate.r.piv"), ("bankL", "plate.l.piv")] {
            let pivot = point(&e, pivot);
            let (dx, dy) = (pivot.0 - pin.0, pivot.1 - pin.1);
            let length = dx.hypot(dy);
            let (c, s) = (dx / length, dy / length);
            let crown = (pin.0 + 46.0 * c, pin.1 + 46.0 * s);
            close(point(&e, &format!("{bank}.crown")), crown);
            // Cylinder mouth corner: eight below the pivot and twelve to its left.
            close(point(&e, &format!("{bank}.cyl.k_bl.p")),
                  (pivot.0 - 8.0 * c - 12.0 * s, pivot.1 - 8.0 * s + 12.0 * c));
            // The left flank starts fourteen down from the crown, 2.5 left of the rod.
            close(point(&e, &format!("{bank}.pis.ra.p")),
                  (crown.0 - 14.0 * c - 2.5 * s, crown.1 - 14.0 * s + 2.5 * c));
        }
    }
}
