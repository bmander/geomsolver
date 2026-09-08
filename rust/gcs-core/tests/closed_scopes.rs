//! Components are closed over their arguments; groups bundle numbers and geometry aliases.
use gcs_core::{library, program, solve, syntax, diagnose};
fn build(src: &str) -> program::Elaborated {
    let (p, errors, linked) = library::parse_linked(src);
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}");
    program::elaborate(&p)
}
fn solved(src: &str) -> program::Elaborated {
    let mut e = build(src);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    e
}
#[test]
fn groups_pass_units_and_geometry_without_copying_it() {
    let mut e = solved("unit mm\ncomponent Bar(d: group) {\n\
        point tip hint(x: d.origin.x + d.length, y: d.origin.y)\n\
        d.origin distance(d.length) tip\nd.origin distance(0mm, along: y) tip\n}\n\
        part: Bar(design)\ngroup design(length: width, origin: o)\n\
        param width = 2cm\npoint o hint(x: 7mm, y: 3mm)\nground o\n");
    assert_eq!(e.sketch.points.len(), 2);
    let tip = e.sketch.point_xy(e.map.ent_named("part.tip").unwrap().i());
    assert!((tip.0 - 27.0).abs() < 1e-8 && (tip.1 - 3.0).abs() < 1e-8, "{tip:?}");
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
}
#[test]
fn nested_groups_forward_through_components_and_repetitions() {
    let e = solved("unit mm\nuse std\n\
        component Inner(d: group) {\npoint tip hint(x: d.frame.origin.x + d.sizes.width * d.frame.c, y: d.frame.origin.y + d.sizes.width * d.frame.s)\nline axis(d.frame.origin, d.frame.toward)\ntip on axis\nd.frame.origin distance(d.sizes.width) tip\n}\n\
        component Outer(d: group) { repeat 2 { Inner(d) } }\n\
        Outer(design)\ngroup design(frame: std.front, sizes: sizes)\n\
        group sizes(width: 12mm)\n");
    assert_eq!(e.sketch.points.len(), 5);
}
#[test]
fn layout_instances_can_be_passed_forward_without_an_extra_solve() {
    let mut e = solved("component Layout() { point a hint(x: 0,y: 0)\nground a\npoint b hint(x: 20,y: 0) }\n\
        component Part(layout: group) { line bar(layout.a, layout.b)\nhorizontal bar\nlayout.a distance(20) layout.b }\n\
        part: Part(layout)\nlayout: Layout()\n");
    assert_eq!(e.sketch.points.len(), 2);
    assert_eq!(e.sketch.lines.len(), 1);
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
}
#[test]
fn components_cannot_capture_global_values_geometry_or_standard_datums() {
    for src in [
        "param length = 20\ncomponent Bad() { point a\npoint b\na distance(length) b }\nb: Bad()",
        "point a\ncomponent Bad() { ground a }\nb: Bad()",
        "use std\ncomponent Bad() { line axis(std.origin, std.up.toward) }\nb: Bad()",
        "component Inner() { ground a }\ncomponent Outer() { point a\ni: Inner() }\no: Outer()",
        "param length = 20\ncomponent Bad() { param width = length / 2\npoint a }\nb: Bad()",
    ] { assert!(!build(src).ok(), "captured ambient value: {src}"); }
}
#[test]
fn groups_reject_missing_duplicate_and_wrong_kind_members() {
    for src in [
        "component Bad(d: group) { point a }\nb: Bad()",
        "component Bad(d: group) { point a }\npoint p\nb: Bad(p)",
        "component Bad(d: group) { point a\npoint b\na distance(d.missing) b }\ngroup dims(width: 20)\nb: Bad(dims)",
        "group dims(width: 20, width: 30)",
        "unit mm\ncomponent Bad(d: group) { point a\npoint b\na distance(d.width) b }\ngroup dims(width: 20deg)\nb: Bad(dims)",
    ] { assert!(!build(src).ok(), "accepted invalid group: {src}"); }
}
#[test]
fn groups_round_trip_in_source() {
    let src = "group dims(width: 20mm, origin: o)\n";
    let (p, errors) = syntax::parse(src);
    assert!(errors.is_empty());
    let mut text = String::new();
    syntax::write_stmt_to(&mut text, &p.root().body[0].kind).unwrap();
    assert_eq!(text.trim(), src.trim());
}

#[test]
fn layout_groups_resolve_nested_and_indexed_members_forward() {
    let e = solved(r#"
component Layout() {
  repeat 2 as i {
    point p hint(x: i * 20, y: 0)
    ground p
  }
}
component Part(d: group) {
  group local(layout: d.layout)
  line bar(local.layout.p[0], local.layout.p[1])
}
part: Part(design)
group design(layout: sketch)
sketch: Layout()
"#);
    assert_eq!(e.sketch.points.len(), 2);
    assert_eq!(e.sketch.lines.len(), 1);
}

#[test]
fn groups_do_not_create_new_unknowns_or_hide_invalid_definitions() {
    for src in [
        "group bad(member: missing)",
        "group bad(self: bad)",
        "group a(next: b)\ngroup b(next: a)",
        "group dims(width: 20)\npoint dims",
        "group dims(width: 20)\nparam dims = 3",
        "component Empty() {}\ngroup dims(width: 20)\ndims: Empty()",
        "component Bad(d: group) { group d(width: 30) }\ngroup dims(width: 20)\nb: Bad(dims)",
        "component Bad(d: group) { point p }\ngroup dims(width: 20)\nb: Bad(dims, d: dims)",
        "group dims(width: 20)\npoint a\npoint b\na distance(dims.missing) b",
    ] { assert!(!build(src).ok(), "accepted invalid group: {src}"); }
}

#[test]
fn nested_calls_receive_only_the_arguments_written_at_the_call() {
    assert!(!build(r#"
component Inner() { line border(p, next.p) }
cycle 3 {
  point p
  part: Inner()
}
"#).ok());
    let e = solved(r#"
component Inner(a: point, b: point) { line border(a, b) }
cycle 3 as i {
  point p hint(x: i * 20, y: 0)
  ground p
  part: Inner(p, next.p)
}
"#);
    assert_eq!(e.sketch.points.len(), 3);
    assert_eq!(e.sketch.lines.len(), 3);
}

#[test]
fn groups_keep_free_formals_local_and_preserve_dimension_types() {
    let e = build(r#"
unit mm
component Part(width: Length) {
  group dims(width: width)
  point a
  point b
  a distance(dims.width) b
}
one: Part()
two: Part()
"#);
    assert!(e.ok(), "{:?}", e.diags);
    assert_eq!(e.sketch.free_vars.keys().map(String::as_str).collect::<Vec<_>>(), vec!["one.width", "two.width"]);
    for src in [
        "unit mm\ngroup dims(width: 20mm)\ncomponent Bad(d: group) { point a\npoint b\na distance(d.width + 1deg) b }\nb: Bad(dims)",
        "unit mm\ngroup dims(width: -8mm)\npoint a\npoint b\nline axis(a,b)\ncomponent Bad(ax: line,d: group) { point p\np distance(d.width) ax }\nb: Bad(axis,dims)",
        "use std\ncomponent Bad(u: Angle) { point p = (std.origin.x + cos(u), sin(u)) }\ncurve c = Bad().p over u in (0,90)",
    ] { assert!(!build(src).ok(), "accepted invalid component: {src}"); }
}

#[test]
fn numeric_groups_preserve_derived_dimensions() {
    let e = solved(r#"
unit mm
group dims(area: 400mm * 1mm, root: sqrt(16mm))
component Part(d: group) {
  param length = sqrt(d.area) + d.root * d.root
  point a hint(x: 0, y: 0)
  ground a
  point b hint(x: length, y: 0)
  a distance(length, along: x) b
  a distance(0mm, along: y) b
}
p: Part(dims)
"#);
    let (x, _) = e.sketch.point_xy(e.map.ent_named("p.b").unwrap().i());
    assert!((x - 36.0).abs() < 1e-8, "{x}");
}

#[test]
fn standard_centered_rectangle_has_dimensioned_sides_and_a_private_diagonal() {
    let mut e = build("unit mm\nuse std\npoint c hint(x: 7mm, y: -3mm)\nground c\nr: CenteredRectangle(c, w: 20mm, h: 12mm)");
    assert!(e.ok(), "{:?}", e.diags);
    for i in 0..e.sketch.points.len() {
        for (axis, param) in e.sketch.point_params(i).into_iter().enumerate() {
            if !e.sketch.params[param as usize].fixed {
                e.sketch.params[param as usize].value += 0.2 * ((i * 5 + axis) as f64).sin();
            }
        }
    }
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
    for (name, expected) in [("r.a", (-3.0, -9.0)), ("r.c", (17.0, 3.0))] {
        let actual = e.sketch.point_xy(e.map.ent_named(name).unwrap().i());
        assert!((actual.0 - expected.0).abs() < 1e-7 && (actual.1 - expected.1).abs() < 1e-7, "{actual:?}");
    }
    assert!(e.map.entity_path(&e.sketch, "r.loop").is_some());
    assert!(e.map.entity_path(&e.sketch, "r.diagonal").is_none());
    assert!(e.sketch.roles_of(e.map.ent_named("r.diagonal").unwrap()).construction);
    assert!(!e.sketch.roles_of(e.map.ent_named("c").unwrap()).construction);
}
