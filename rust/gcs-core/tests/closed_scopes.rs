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
        tip := point hint(x: d.origin.x + d.length, y: d.origin.y)\n\
        d.origin distance(d.length) tip\nd.origin distance(0mm, along: y) tip\n}\n\
        part := Bar(design)\ndesign := group(length: width, origin: o)\n\
        width := 2cm\no := point\nfix(x == 7mm, y == 3mm) o\n");
    assert_eq!(e.sketch.points.len(), 2);
    let tip = e.sketch.point_xy(e.map.ent_named("part.tip").unwrap().i());
    assert!((tip.0 - 27.0).abs() < 1e-8 && (tip.1 - 3.0).abs() < 1e-8, "{tip:?}");
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
}
#[test]
fn nested_groups_forward_through_components_and_repetitions() {
    let e = solved("unit mm\nuse std\n\
        component Inner(d: group) {\ntip := point hint(x: d.frame.origin.x + d.sizes.width * d.frame.c, y: d.frame.origin.y + d.sizes.width * d.frame.s)\naxis := line(d.frame.origin, d.frame.toward)\ntip on axis\nd.frame.origin distance(d.sizes.width) tip\n}\n\
        component Outer(d: group) { repeat 2 { Inner(d) } }\n\
        Outer(design)\ndesign := group(frame: std.front, sizes: sizes)\n\
        sizes := group(width: 12mm)\n");
    // the two tips, beside the standard datums' four points
    assert_eq!(e.sketch.points.len(), 6);
}
#[test]
fn layout_instances_can_be_passed_forward_without_an_extra_solve() {
    let mut e = solved("component Layout() { a := point\nfix(x == 0, y == 0) a\nb := point hint(x: 20,y: 0) }\n\
        component Part(layout: group) { bar := line(layout.a, layout.b)\nhorizontal bar\nlayout.a distance(20) layout.b }\n\
        part := Part(layout)\nlayout := Layout()\n");
    assert_eq!(e.sketch.points.len(), 2);
    assert_eq!(e.sketch.lines.len(), 1);
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
}
#[test]
fn components_cannot_capture_global_values_geometry_or_standard_datums() {
    for src in [
        "length := 20\ncomponent Bad() { a := point\nb := point\na distance(length) b }\nb := Bad()",
        "a := point\ncomponent Bad() { fix(x == 0, y == 0) a }\nb := Bad()",
        "use std\ncomponent Bad() { axis := line(std.origin, std.up.toward) }\nb := Bad()",
        "component Inner() { fix(x == 0, y == 0) a }\ncomponent Outer() { a := point\ni := Inner() }\no := Outer()",
        "length := 20\ncomponent Bad() { width := length / 2\na := point }\nb := Bad()",
    ] { assert!(!build(src).ok(), "captured ambient value: {src}"); }
}
#[test]
fn groups_reject_missing_duplicate_and_wrong_kind_members() {
    for src in [
        "component Bad(d: group) { a := point }\nb := Bad()",
        "component Bad(d: group) { a := point }\np := point\nb := Bad(p)",
        "component Bad(d: group) { a := point\nb := point\na distance(d.missing) b }\ndims := group(width: 20)\nb := Bad(dims)",
        "dims := group(width: 20, width: 30)",
        "unit mm\ncomponent Bad(d: group) { a := point\nb := point\na distance(d.width) b }\ndims := group(width: 20deg)\nb := Bad(dims)",
    ] { assert!(!build(src).ok(), "accepted invalid group: {src}"); }
}
#[test]
fn groups_round_trip_in_source() {
    let src = "dims := group(width: 20mm, origin: o)\n";
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
    p := point
    fix(x == i * 20, y == 0) p
  }
}
component Part(d: group) {
  local := group(layout: d.layout)
  bar := line(local.layout.p[0], local.layout.p[1])
}
part := Part(design)
design := group(layout: sketch)
sketch := Layout()
"#);
    assert_eq!(e.sketch.points.len(), 2);
    assert_eq!(e.sketch.lines.len(), 1);
}

#[test]
fn groups_do_not_create_new_unknowns_or_hide_invalid_definitions() {
    for src in [
        "bad := group(member: missing)",
        "bad := group(self: bad)",
        "a := group(next: b)\nb := group(next: a)",
        "dims := group(width: 20)\ndims := point",
        "dims := group(width: 20)\ndims := 3",
        "component Empty() {}\ndims := group(width: 20)\ndims := Empty()",
        "component Bad(d: group) { d := group(width: 30) }\ndims := group(width: 20)\nb := Bad(dims)",
        "component Bad(d: group) { p := point }\ndims := group(width: 20)\nb := Bad(dims, d: dims)",
        "dims := group(width: 20)\na := point\nb := point\na distance(dims.missing) b",
    ] { assert!(!build(src).ok(), "accepted invalid group: {src}"); }
}

#[test]
fn nested_calls_receive_only_the_arguments_written_at_the_call() {
    assert!(!build(r#"
component Inner() { border := line(p, next.p) }
cycle 3 {
  p := point
  part := Inner()
}
"#).ok());
    let e = solved(r#"
component Inner(a: point, b: point) { border := line(a, b) }
cycle 3 as i {
  p := point
  fix(x == i * 20, y == 0) p
  part := Inner(p, next.p)
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
  dims := group(width: width)
  a := point
  b := point
  a distance(dims.width) b
}
one := Part()
two := Part()
"#);
    assert!(e.ok(), "{:?}", e.diags);
    assert_eq!(e.sketch.free_vars.keys().map(String::as_str).collect::<Vec<_>>(), vec!["one.width", "two.width"]);
    for src in [
        "unit mm\ndims := group(width: 20mm)\ncomponent Bad(d: group) { a := point\nb := point\na distance(d.width + 1deg) b }\nb := Bad(dims)",
        "unit mm\ndims := group(width: -8mm)\na := point\nb := point\naxis := line(a,b)\ncomponent Bad(ax: line,d: group) { p := point\np distance(d.width) ax }\nb := Bad(axis,dims)",
        "use std\ncomponent Bad(u: Angle) { p := point(x: std.origin.x + cos(u), y: sin(u)) }\nc := Bad().p over u in (0,90)",
    ] { assert!(!build(src).ok(), "accepted invalid component: {src}"); }
}

#[test]
fn numeric_groups_preserve_derived_dimensions() {
    let e = solved(r#"
unit mm
dims := group(area: 400mm * 1mm, root: sqrt(16mm))
component Part(d: group) {
  length := sqrt(d.area) + d.root * d.root
  a := point
  fix(x == 0, y == 0) a
  b := point hint(x: length, y: 0)
  a distance(length, along: x) b
  a distance(0mm, along: y) b
}
p := Part(dims)
"#);
    let (x, _) = e.sketch.point_xy(e.map.ent_named("p.b").unwrap().i());
    assert!((x - 36.0).abs() < 1e-8, "{x}");
}

#[test]
fn standard_centered_rectangle_has_dimensioned_sides_and_a_private_diagonal() {
    let mut e = build("unit mm\nuse std\nc := point\nfix(x == 7mm, y == -3mm) c\nr := std.CenteredRectangle(c, w: 20mm, h: 12mm)");
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
