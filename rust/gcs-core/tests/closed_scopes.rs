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
    let mut e = solved("\
unit mm
use std
component Bar(d: group) {
tip := point hint((d.origin.x + d.length, d.origin.y))
d.origin distance(d.length) tip
d.origin distance(0mm, along: y) tip
}
in std.front {
part := Bar(design)
design := {length: width, origin: o}
width := 2cm
o := point
fix((7mm, 3mm)) o
}
");
    assert_eq!(e.sketch.points.len(), 2 + crate::common::STD_POINTS);
    let tip = e.sketch.point_xy(e.map.ent_named("part.tip").unwrap().i());
    assert!((tip.0 - 27.0).abs() < 1e-8 && (tip.1 - 3.0).abs() < 1e-8, "{tip:?}");
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
}
#[test]
fn nested_groups_forward_through_components_and_repetitions() {
    let src = "\
unit mm
use std
component Inner(d: group) {
tip := point hint(at: d.frame.axes, (d.sizes.width, 0mm))
ax := line(d.frame.u.p1, d.frame.u.p2)
tip coincident ax
d.frame.u.p1 distance(d.sizes.width) tip
}
component Outer(d: group) { repeat 2 { Inner(d) } }
in std.front {
o := point
fix((0, 0)) o
t := point
fix((40, 0)) t
frame := std.Turned(o, t)
Outer(design)
}
design := {frame: frame, sizes: sizes}
sizes := {width: 12mm}
";
    let e = solved(src);
    // the two tips, beside what the frame and the standard datums make
    let without = solved(&src.replace("Outer(design)\n", ""));
    assert_eq!(e.sketch.points.len(), without.sketch.points.len() + 2);
    let tips: Vec<_> = (0..e.sketch.points.len())
        .filter(|&i| e.map.name_of(gcs_core::model::EntRef::point(i)).is_some_and(|n| n.ends_with(".tip")))
        .collect();
    for i in tips {
        let (x, y) = e.sketch.point_xy(i);
        assert!((x - 12.0).abs() < 1e-9 && y.abs() < 1e-9, "{x} {y}");
    }
}
#[test]
fn layout_instances_can_be_passed_forward_without_an_extra_solve() {
    let mut e = solved("\
use std
component Layout() { a := point
fix((0, 0)) a
b := point hint((20, 0)) }
component Part(layout: group) { bar := line(layout.a, layout.b)
horizontal bar
layout.a distance(20) layout.b }
in std.front {
part := Part(layout)
layout := Layout()
}
");
    assert_eq!(e.sketch.points.len(), 2 + crate::common::STD_POINTS);
    assert_eq!(e.sketch.lines.len(), 1);
    assert_eq!(diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
}
#[test]
fn components_cannot_capture_global_values_geometry_or_standard_datums() {
    for src in [
        "use std\nlength := 20\ncomponent Bad() { a := point\nb := point\na distance(length) b }\nin std.front {\nb := Bad()\n}\n",
        "use std\nin std.front {\na := point\n}\ncomponent Bad() { fix((0, 0)) a }\nin std.front {\nb := Bad()\n}\n",
        "use std\ncomponent Bad() { ax := line(std.origin, hint((0, 1))) }\nin std.front {\nb := Bad()\n}\n",
        "use std\ncomponent Inner() { fix((0, 0)) a }\ncomponent Outer() { a := point\ni := Inner() }\nin std.front {\no := Outer()\n}\n",
        "use std\nlength := 20\ncomponent Bad() { width := length / 2\na := point }\nin std.front {\nb := Bad()\n}\n",
    ] { assert!(!build(src).ok(), "captured ambient value: {src}"); }
}
#[test]
fn groups_reject_missing_duplicate_and_wrong_kind_members() {
    for src in [
        "use std\ncomponent Bad(d: group) { a := point }\nin std.front {\nb := Bad()\n}\n",
        "use std\ncomponent Bad(d: group) { a := point }\nin std.front {\np := point\nb := Bad(p)\n}\n",
        "use std\ncomponent Bad(d: group) { a := point\nb := point\na distance(d.missing) b }\ndims := {width: 20}\nin std.front {\nb := Bad(dims)\n}\n",
        "dims := {width: 20, width: 30}",
        "dims := group(width: 20)",
        "unit mm\nuse std\ncomponent Bad(d: group) { a := point\nb := point\na distance(d.width) b }\ndims := {width: 20deg}\nin std.front {\nb := Bad(dims)\n}\n",
    ] { assert!(!build(src).ok(), "accepted invalid group: {src}"); }
}
/// A group's braces hold a list of members, not a body of statements: like an argument list,
/// it may run across lines (a trailing comma too), and it may stand inside a body's braces.
#[test]
fn a_group_may_be_written_across_lines() {
    let e = solved("\
unit mm
use std
component Bar(d: group) {
tip := point hint((d.origin.x + d.length, d.origin.y))
d.origin distance(d.length) tip
d.origin distance(0mm, along: y) tip
}
component Host(o: point) {
  design := {
    length: 2cm,
    origin: o,
  }
part := Bar(design)
}
in std.front {
o := point
fix((7mm, 3mm)) o
h := Host(o)
}
");
    let tip = e.sketch.point_xy(e.map.ent_named("h.part.tip").unwrap().i());
    assert!((tip.0 - 27.0).abs() < 1e-8 && (tip.1 - 3.0).abs() < 1e-8, "{tip:?}");
}
#[test]
fn groups_round_trip_in_source() {
    let src = "dims := {width: 20mm, origin: o}\n";
    let (p, errors) = crate::common::parse(src);
    assert!(errors.is_empty());
    let mut text = String::new();
    syntax::write_stmt_to(&mut text, &p.root().body[0].kind).unwrap();
    assert_eq!(text.trim(), src.trim());
}

#[test]
fn layout_groups_resolve_nested_and_indexed_members_forward() {
    let e = solved(r#"use std

component Layout() {
  repeat 2 as i {
    p := point
    fix((i * 20, 0)) p
  }
}
component Part(d: group) {
  local := {layout: d.layout}
  bar := line(local.layout.p[0], local.layout.p[1])
}
in std.front {
part := Part(design)
design := {layout: sketch}
sketch := Layout()
}
"#);
    assert_eq!(e.sketch.points.len(), 2 + crate::common::STD_POINTS);
    assert_eq!(e.sketch.lines.len(), 1);
}

#[test]
fn groups_do_not_create_new_unknowns_or_hide_invalid_definitions() {
    for src in [
        "bad := {member: missing}",
        "bad := {self: bad}",
        "a := {next: b}\nb := {next: a}",
        "use std\ndims := {width: 20}\nin std.front {\ndims := point\n}\n",
        "dims := {width: 20}\ndims := 3",
        "use std\ncomponent Empty() {}\ndims := {width: 20}\nin std.front {\ndims := Empty()\n}\n",
        "use std\ncomponent Bad(d: group) { d := {width: 30} }\ndims := {width: 20}\nin std.front {\nb := Bad(dims)\n}\n",
        "use std\ncomponent Bad(d: group) { p := point }\ndims := {width: 20}\nin std.front {\nb := Bad(dims, d: dims)\n}\n",
        "use std\ndims := {width: 20}\nin std.front {\na := point\nb := point\na distance(dims.missing) b\n}\n",
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
use std
component Part(width: Length) {
  dims := {width: width}
  a := point
  b := point
  a distance(dims.width) b
}
in std.front {
one := Part()
two := Part()
}
"#);
    assert!(e.ok(), "{:?}", e.diags);
    assert_eq!(e.sketch.free_vars.keys().map(String::as_str).collect::<Vec<_>>(), vec!["one.width", "two.width"]);
    for src in [
        "unit mm\nuse std\ndims := {width: 20mm}\ncomponent Bad(d: group) { a := point\nb := point\na distance(d.width + 1deg) b }\nin std.front {\nb := Bad(dims)\n}\n",
        "unit mm\nuse std\ndims := {width: -8mm}\nin std.front {\na := point\nb := point\nax := line(a,b)\n}\ncomponent Bad(ax: line,d: group) { p := point\np distance(d.width) ax }\nin std.front {\nb := Bad(ax,dims)\n}\n",
        "use std\ncomponent Bad(u: Angle) { p := point(x: std.origin.x + cos(u), y: sin(u)) }\nc := Bad().p over u in (0,90)",
    ] { assert!(!build(src).ok(), "accepted invalid component: {src}"); }
}

#[test]
fn numeric_groups_preserve_derived_dimensions() {
    let e = solved(r#"
unit mm
use std
dims := {area: 400mm * 1mm, root: sqrt(16mm)}
component Part(d: group) {
  length := sqrt(d.area) + d.root * d.root
  a := point
  fix((0, 0)) a
  b := point hint((length, 0))
  a distance(length, along: x) b
  a distance(0mm, along: y) b
}
in std.front {
p := Part(dims)
}
"#);
    let (x, _) = e.sketch.point_xy(e.map.ent_named("p.b").unwrap().i());
    assert!((x - 36.0).abs() < 1e-8, "{x}");
}

#[test]
fn standard_centered_rectangle_has_dimensioned_sides_and_a_private_diagonal() {
    let mut e = build("unit mm\nuse std\nin std.front {\nc := point\nfix((7mm, -3mm)) c\nr := std.CenteredRectangle(c, w: 20mm, h: 12mm)\n}\n");
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

/// A group's member may be a group written in place: its members are reached through the
/// outer group (`d.cyl.bore`), and it is a group of its own, handed on as `dims.cyl`.  Three
/// levels deep, across lines, and with geometry beside the numbers.
#[test]
fn a_group_member_may_be_a_group_written_in_place() {
    let e = solved("\
unit mm
use std
component Bar(d: group) {
tip := point hint((d.at.x + d.size.length, d.at.y))
d.at distance(d.size.length) tip
d.at distance(0mm, along: y) tip
}
component Pin(s: group) {
q := point hint((s.length, 9mm))
fix(x == 0mm) q
s.at distance(s.length) q
}
in std.front {
o := point
fix((7mm, 3mm)) o
design := {
  bar: {
    at: o,
    size: {length: 2cm, half: 1cm},
  },
pin: {length: design.bar.size.half, at: o},
}
part := Bar(design.bar)
pin := Pin(design.pin)
}
");
    let tip = e.sketch.point_xy(e.map.ent_named("part.tip").unwrap().i());
    assert!((tip.0 - 27.0).abs() < 1e-8 && (tip.1 - 3.0).abs() < 1e-8, "{tip:?}");
    let q = e.sketch.point_xy(e.map.ent_named("pin.q").unwrap().i());
    assert!((q.0 - 0.0).abs() < 1e-8 && ((q.0 - 7.0).hypot(q.1 - 3.0) - 10.0).abs() < 1e-8, "{q:?}");
    // the members alias and add nothing: the two points drawn, beside `o`
    assert_eq!(e.sketch.points.len(), 3 + crate::common::STD_POINTS);
}

/// Written in place or defined by name and referred to, a nested group is the same group.
#[test]
fn a_group_in_place_reads_as_one_referred_to() {
    let tail = "\
use std
component Bar(d: group) {
tip := point hint((d.at.x + d.size.length, d.at.y))
d.at distance(d.size.length) tip
d.at distance(0, along: y) tip
}
in std.front {
o := point
fix((7, 3)) o
part := Bar(design.bar)
}
";
    let place = solved(&format!("design := {{bar: {{at: o, size: {{length: 20}}}}}}\n{tail}"));
    let named = solved(&format!(
        "size := {{length: 20}}\nbar := {{at: o, size: size}}\ndesign := {{bar: bar}}\n{tail}"));
    let at = |e: &program::Elaborated| e.sketch.point_xy(e.map.ent_named("part.tip").unwrap().i());
    assert_eq!(at(&place), at(&named));
}

#[test]
fn a_group_in_place_round_trips_in_source() {
    let src = "dims := {bore: 16mm, cyl: {ax: datum, wall: {t: 2mm}}}\n";
    let (p, errors) = crate::common::parse(src);
    assert!(errors.is_empty(), "{errors:?}");
    let mut text = String::new();
    syntax::write_stmt_to(&mut text, &p.root().body[0].kind).unwrap();
    assert_eq!(text.trim(), src.trim());
}

#[test]
fn a_group_in_place_is_refused_where_a_group_would_be() {
    for src in [
        // every member named, at every level
        "dims := {cyl: {16mm}}",
        // one name per member, at every level
        "dims := {cyl: {bore: 16, bore: 20}}",
        "dims := {cyl: {bore: 16}, cyl: {bore: 20}}",
        // a member that is not there
        "use std\ndims := {cyl: {bore: 16}}\nin std.front {\na := point\nb := point\na distance(dims.cyl.wall) b\n}\n",
        // braces in a call: a group is given by name
        "use std\ncomponent Bar(d: group) { a := point }\nin std.front {\nb := Bar(d: {w: 20})\n}\n",
    ] { assert!(!build_or_parse_fails(src), "accepted invalid group: {src}"); }
}

/// Whether `src` parses and elaborates cleanly.
fn build_or_parse_fails(src: &str) -> bool {
    let (p, errors, linked) = library::parse_linked(src);
    errors.is_empty() && linked.is_empty() && program::elaborate(&p).ok()
}
