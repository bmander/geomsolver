//! **A group is a value** (#594): it may be given another name (`small := sizes`), written in
//! braces where a `group` formal takes it (`Bar({length: 2cm, origin: o})`), and handed on from
//! a body that reads it (a file's group passed to a part the body draws), as a number may.
use gcs_core::modules::link;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::parse;
use crate::common::linked::{build, solved};

/// A part drawing its tip `d.length` along `u` from `d.origin`: what each test hands a group to.
const BAR: &str = "\
unit mm
use std (horizontal)
component Bar(d: group) {
tip := point hint((d.origin.x + d.length, d.origin.y))
d.origin distance(d.length) tip
d.origin horizontal tip
}
";

/// A module with a number, a group, a name for that group, and parts reading them.
const DIMS: &str = "\
unit mm
use std (horizontal)
gap := 3mm
handle := {length: 40mm}
grip := handle
component Bar(d: group, origin: point) {
tip := point hint((origin.x + d.length, origin.y))
origin distance(d.length) tip
origin horizontal tip
}
component Pair(origin: point) {
a := Bar(handle, origin)
b := Bar(grip, origin)
c := Bar({length: handle.length / 2}, origin)
}
";

fn read(src: &str) -> Elaborated {
    let (mut prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let linked = link(&mut prog, &mut |name| match name {
        "lib.dims" => Some(DIMS.to_string()),
        _ => gcs_core::library::resolve(name),
    });
    assert!(linked.is_empty(), "{linked:?}");
    elaborate(&prog)
}

fn solved_linked(src: &str) -> Elaborated {
    let mut e = read(src);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    e
}

/// Where the point `name` landed, in its plane.
fn at(e: &Elaborated, name: &str) -> (f64, f64) {
    let p = e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`"));
    e.sketch.point_xy(p.i())
}

fn near(p: (f64, f64), q: (f64, f64)) -> bool {
    (p.0 - q.0).abs() < 1e-8 && (p.1 - q.1).abs() < 1e-8
}

/// The errors said, as one text.
fn said(e: &Elaborated) -> String {
    e.diags.iter().map(|d| format!("{d:?}\n")).collect()
}

#[test]
fn a_group_may_be_given_another_name() {
    // its members, numbers and geometry, read through either name; a name for a name, even one
    // defined below it
    let e = solved(&format!("{BAR}\
in std.front {{
o := point
fix((7mm, 3mm)) o
sizes := {{length: 2cm, origin: o}}
small := sizes
tiny := smaller
smaller := small
a := Bar(small)
b := Bar(tiny)
q := point
o distance(small.length / 2) q
o horizontal q
}}
"));
    assert!(near(at(&e, "a.tip"), (27.0, 3.0)), "{:?}", at(&e, "a.tip"));
    assert!(near(at(&e, "b.tip"), (27.0, 3.0)), "{:?}", at(&e, "b.tip"));
    let q = at(&e, "q");
    assert!(((q.0 - 7.0).abs() - 10.0).abs() < 1e-8 && (q.1 - 3.0).abs() < 1e-8, "{q:?}");
    // a name adds nothing to solve: the points drawn are the two parts' tips, `o` and `q`
    assert_eq!(e.sketch.points.len(), 4 + crate::common::STD_POINTS);

    // and a module's group, named by its path, or by a name the module gives it
    let e = solved_linked("\
unit mm
use std (horizontal)
use lib.dims
in std.front {
o := point
fix((0mm, 0mm)) o
hd := lib.dims.handle
a := lib.dims.Bar(hd, o)
b := lib.dims.Bar(lib.dims.grip, o)
}
");
    assert!(near(at(&e, "a.tip"), (40.0, 0.0)), "{:?}", at(&e, "a.tip"));
    assert!(near(at(&e, "b.tip"), (40.0, 0.0)), "{:?}", at(&e, "b.tip"));
}

#[test]
fn a_group_may_be_written_where_it_is_taken() {
    // at the root, and in a body over its own formals — what was named only to be passed once
    let e = solved(&format!("{BAR}\
component Offset(d: group, o: Length) {{
part := Bar({{length: d.length + 2 * o, origin: d.origin}})
}}
in std.front {{
o := point
fix((7mm, 3mm)) o
a := Bar({{length: 2cm, origin: o}})
b := Offset({{length: 10mm, origin: o}}, o: 5mm)
}}
"));
    assert!(near(at(&e, "a.tip"), (27.0, 3.0)), "{:?}", at(&e, "a.tip"));
    assert!(near(at(&e, "b.part.tip"), (27.0, 3.0)), "{:?}", at(&e, "b.part.tip"));
    assert_eq!(e.sketch.points.len(), 3 + crate::common::STD_POINTS);

    // a group in place in it, read as one named is
    let e = solved(&format!("{BAR}\
component Nested(d: group) {{
part := Bar(d.bar)
}}
in std.front {{
o := point
fix((7mm, 3mm)) o
n := Nested({{bar: {{length: 2cm, origin: o}}}})
}}
"));
    assert!(near(at(&e, "n.part.tip"), (27.0, 3.0)), "{:?}", at(&e, "n.part.tip"));

    // an interface judges it at the argument, naming the formal
    let typed = "\
unit mm
use std (horizontal)
interface Size { length: Length }
interface Layout {
  origin: point
  size: Size
}
component Bar(d: Layout) {
tip := point hint((d.origin.x + d.size.length, d.origin.y))
d.origin distance(d.size.length) tip
d.origin horizontal tip
}
in std.front {
o := point
fix((7mm, 3mm)) o
";
    let e = solved(&format!("{typed}a := Bar({{origin: o, size: {{length: 2cm}}}})\n}}\n"));
    assert!(near(at(&e, "a.tip"), (27.0, 3.0)), "{:?}", at(&e, "a.tip"));
    let e = build(&format!("{typed}a := Bar({{origin: o, size: {{length: 2deg}}}})\n}}\n"));
    assert!(said(&e).contains("`d.size.length` is Length, and this is Angle"), "{}", said(&e));
    let e = build(&format!("{typed}a := Bar({{origin: o}})\n}}\n"));
    assert!(said(&e).contains("`d` has no `size`, which `Layout` asks for"), "{}", said(&e));
    assert!(!said(&e).contains('#'), "a key no source writes is said: {}", said(&e));

    // braces are no argument for a formal that takes no group
    let e = build("use std\ncomponent P(a: point) { }\nin std.front {\np := P({x: 1})\n}\n");
    assert!(said(&e).contains("`a` takes no group"), "{}", said(&e));
}

#[test]
fn a_body_hands_on_the_groups_of_its_file() {
    // a file's group read by a body is a group argument there too, by its name or a name for it
    let e = solved(&format!("{BAR}\
sizes := {{length: 2cm}}
component Pair(o: point) {{
mine := sizes
a := Bar({{length: sizes.length, origin: o}})
b := Bar({{length: mine.length / 2, origin: o}})
c := Hand(sizes, o)
}}
component Hand(s: group, o: point) {{
part := Bar({{length: s.length, origin: o}})
}}
in std.front {{
o := point
fix((7mm, 3mm)) o
p := Pair(o)
}}
"));
    assert!(near(at(&e, "p.a.tip"), (27.0, 3.0)), "{:?}", at(&e, "p.a.tip"));
    assert!(near(at(&e, "p.b.tip"), (17.0, 3.0)), "{:?}", at(&e, "p.b.tip"));
    assert!(near(at(&e, "p.c.part.tip"), (27.0, 3.0)), "{:?}", at(&e, "p.c.part.tip"));

    // a module's body hands on the module's groups, and a body the groups of a module its
    // file uses, by their paths
    let e = solved_linked("\
unit mm
use std (horizontal)
use lib.dims
component Mine(o: point) {
a := lib.dims.Bar(lib.dims.handle, o)
}
in std.front {
o := point
fix((0mm, 0mm)) o
p := lib.dims.Pair(o)
m := Mine(o)
}
");
    for (name, x) in [("p.a.tip", 40.0), ("p.b.tip", 40.0), ("p.c.tip", 20.0), ("m.a.tip", 40.0)] {
        assert!(near(at(&e, name), (x, 0.0)), "{name}: {:?}", at(&e, name));
    }

    // a formal hides the file's group of its name, as it hides a constant
    let e = solved(&format!("{BAR}\
sizes := {{length: 2cm}}
component Hand(sizes: group, o: point) {{
part := Bar({{length: sizes.length, origin: o}})
}}
in std.front {{
o := point
fix((7mm, 3mm)) o
h := Hand({{length: 1cm}}, o)
}}
"));
    assert!(near(at(&e, "h.part.tip"), (17.0, 3.0)), "{:?}", at(&e, "h.part.tip"));

    // but a group naming the file's geometry is its drawing, which a body is handed or not
    let e = build(&format!("{BAR}\
in std.front {{
o := point
}}
geo := {{length: 2cm, origin: o}}
component Pair() {{
a := Bar(geo)
}}
p := Pair()
"));
    assert!(said(&e).contains("`geo` names geometry of the file"), "{}", said(&e));
}

#[test]
fn a_group_read_as_a_number_is_said_to_be_one() {
    let e = read("unit mm\nuse lib.dims\nx := lib.dims.handle * 2\n");
    assert!(said(&e).contains("`lib.dims.handle` is a group, and a group is not a number"),
            "{}", said(&e));
    let e = build("unit mm\nsizes := {length: 2cm}\nx := sizes + 1mm\n");
    assert!(said(&e).contains("`sizes` is a group, and a group is not a number: read one of its \
                               members, as `sizes.length`"), "{}", said(&e));
}
