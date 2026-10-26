//! **Interfaces** (§8.1 [0.84]): `interface Gear { O: point, M: point, pitch: Length }` names a
//! group's shape, and a formal typed by one (`gear: Gear`) is a `group` formal the call must give
//! every member to, each of its type, through which the body may read only those members.
use gcs_core::modules::link;
use gcs_core::program::{elaborate, Code, Diag, Elaborated};
use gcs_core::{library, syntax};
use std::collections::BTreeMap;
use crate::common::linked::{build, solved};

/// The errors a document comes to, each its code and message.
fn errors(src: &str) -> Vec<(Code, String)> {
    build(src).errors().map(|d| (d.code, d.message.clone())).collect()
}

/// That exactly one error is said, of that code, its message holding each of `words`.
fn said(src: &str, code: Code, words: &[&str]) {
    let errs = errors(src);
    assert_eq!(errs.len(), 1, "{errs:#?}\n{src}");
    let (c, m) = &errs[0];
    assert_eq!(*c, code, "{m}");
    for w in words {
        assert!(m.contains(w), "`{m}` does not say `{w}`");
    }
}

/// A gear's cone as `HypoidPair` hands it on: an instance carrying points, a line and a number.
const CONE: &str = "\
unit mm
use std
interface Cone {
  O: point
  M: point
  generator: line
  teeth: Int, module: Length
}
component GearCone(teeth: Int, module: Length) {
  O := point
  M := point hint((10, 0))
  generator := line(O, M)
  hidden := point hint((0, 5))
  fix((0, 0)) O
  O distance(teeth * module / 2) M
  fix(y == 0) M
  fix((0, 5)) hidden
}
";

#[test]
fn an_instance_with_the_members_is_one() {
    let src = format!("{CONE}\
component Trace(gear: Cone, R: Length) {{
  C := point hint((gear.teeth * gear.module, 4))
  gear.M distance(R) C
  gear.O distance(gear.teeth * gear.module) C
}}
in std.front {{
  cone := GearCone(teeth: 10, module: 2mm)
  t := Trace(cone, R: 15mm)
}}
");
    let e = solved(&src);
    let c = e.sketch.point_xy(e.map.ent_named("t.C").unwrap().i());
    let o = e.sketch.point_xy(e.map.ent_named("cone.O").unwrap().i());
    assert!(((c.0 - o.0).hypot(c.1 - o.1) - 20.0).abs() < 1e-8, "{c:?}");
    // the same body over an untyped group draws the same
    let untyped = solved(&src.replace("gear: Cone", "gear: group"));
    let u = untyped.sketch.point_xy(untyped.map.ent_named("t.C").unwrap().i());
    assert!((u.0 - c.0).abs() < 1e-9 && (u.1 - c.1).abs() < 1e-9);
}

#[test]
fn a_group_with_the_members_is_one_too() {
    let e = solved("\
unit mm
use std
use std.place (horizontal)
interface Layout { origin: point, length: Length }
component Bar(d: Layout) {
  tip := point hint((d.origin.x + d.length, d.origin.y))
  d.origin distance(d.length) tip
  d.origin horizontal tip
}
in std.front {
  o := point
  fix((7mm, 3mm)) o
  design := {length: 2cm, origin: o, colour: 3}
  bar := Bar(design)
}
");
    let tip = e.sketch.point_xy(e.map.ent_named("bar.tip").unwrap().i());
    assert!((tip.0 - 27.0).abs() < 1e-8 && (tip.1 - 3.0).abs() < 1e-8, "{tip:?}");
}

#[test]
fn a_member_missing_is_said_at_the_argument() {
    let doc = |members: &str| format!("\
unit mm
use std
interface Layout {{ origin: point, length: Length, far: point }}
component Bar(d: Layout) {{ d.origin distance(d.length) d.far }}
in std.front {{
  o := point
  q := point
  design := {{{members}}}
  bar := Bar(design)
}}
");
    said(&doc("length: 2cm, origin: o"), Code::E103, &["`design` has no `far`", "`Layout`", "a point"]);
    said(&doc("origin: o, far: q"), Code::E103, &["no number `length`", "Length"]);
    // a number of another dimension
    said(&doc("length: 30deg, origin: o, far: q"), Code::E103, &["Length"]);
    // and an entity of another kind, once the elaborator knows what the name is
    let line = doc("length: 2cm, origin: o, far: l").replace("  q := point\n", "  l := line\n");
    said(&line, Code::E103, &["`design.far` is a line", "`Layout` asks for a point"]);
}

#[test]
fn a_body_reads_only_what_its_interface_declares() {
    let doc = |body: &str| format!("{CONE}\
component Trace(gear: Cone) {{
  C := point
  {body}
}}
in std.front {{
  cone := GearCone(teeth: 10, module: 2mm)
  t := Trace(cone)
}}
");
    // the instance has `hidden`, but the interface does not say so
    said(&doc("gear.hidden distance(5mm) C"), Code::E101, &["`gear.hidden`", "`Cone` does not declare"]);
    // nor may a number be read past what it declares, nor a member handed on
    said(&doc("gear.O distance(gear.pitch) C"), Code::E101, &["no numeric member `gear.pitch`"]);
    let handed = format!("component Pin(p: point) {{ }}\n{}", doc("Pin(gear.hidden)"));
    said(&handed, Code::E101, &["`gear.hidden`"]);
    // a field of a declared member is the member's own
    let e = build(&doc("C coincident gear.generator\ngear.generator.p2 distance(4mm) C"));
    assert!(e.ok(), "{:?}", e.diags);
}

#[test]
fn interfaces_nest() {
    let doc = |given: &str| format!("\
unit mm
use std
interface Size {{ length: Length }}
interface Design {{ at: point, size: Size }}
component Bar(d: Design) {{
  tip := point hint((d.at.x + d.size.length, d.at.y))
  d.at level(v) tip
  d.at distance(d.size.length, along: u) tip
}}
in std.front {{
  o := point
  fix((0, 0)) o
  sizes := {given}
  design := {{at: o, size: sizes}}
  bar := Bar(design)
}}
");
    let e = build(&doc("{length: 12mm, width: 3mm}"));
    assert!(e.ok(), "{:?}", e.diags);
    said(&doc("{width: 3mm}"), Code::E103, &["no number `length`", "`Size`"]);
    // a group given where an interface asks for a nested one, and the body reading past it
    let past = doc("{length: 12mm, width: 3mm}").replace("distance(d.size.length, along: u) tip",
        "distance(d.size.width, along: u) tip");
    said(&past, Code::E101, &["no numeric member `d.size.width`"]);
}

#[test]
fn a_body_hands_on_only_what_it_was_promised() {
    let doc = |inner: &str| format!("{CONE}\
interface Pitched {{ O: point, {inner} }}
component Inner(g: Pitched) {{ }}
component Outer(gear: Cone) {{ Inner(gear) }}
in std.front {{
  cone := GearCone(teeth: 10, module: 2mm)
  Outer(cone)
}}
");
    assert!(build(&doc("module: Length")).ok());
    // the instance has `hidden`, but `Cone` does not promise it: `gear` holds no such member
    said(&doc("hidden: point"), Code::E103, &["`gear` has no `hidden`", "`Pitched` asks for"]);
    said(&doc("module: Angle"), Code::E103, &["`g.module` is Angle"]);
    // nor does it hand an untyped group formal more than it holds
    let open = doc("module: Length").replace("component Inner(g: Pitched) { }",
        "component Inner(g: group) { q := point\nq coincident g.O }");
    assert!(build(&open).ok(), "{:?}", build(&open).diags);
    let past = open.replace("q coincident g.O", "q coincident g.hidden");
    assert!(!build(&past).ok());
}

#[test]
fn sets_and_solids_are_members() {
    let doc = |given: &str| format!("\
unit mm
use std
use std.sets
interface Ball {{ about: point, bulk: set }}
component Touch(b: Ball) {{
  q := point hint((3, 4))
  q coincident b.bulk
  b.about level(v) q
}}
in std.front {{
  o := point
  fix((0, 0)) o
  ball := {{about: o, bulk: {given}}}
  t := Touch(ball)
}}
s := std.sets.Sphere(o, r: 5mm)
");
    let e = solved(&doc("s"));
    let q = e.sketch.point_xy(e.map.ent_named("t.q").unwrap().i());
    assert!((q.0.hypot(q.1) - 5.0).abs() < 1e-8, "{q:?}");
    said(&doc("o"), Code::E103, &["`ball.bulk` is not a set"]);
}

#[test]
fn a_private_member_is_not_given() {
    let src = "\
use std
interface Ends { a: point, b: point }
component Seg() { a := point
private b := point }
component Use(e: Ends) { }
in std.front {
  s := Seg()
  Use(s)
}
";
    said(src, Code::E101, &["`s.b` names private member"]);
}

#[test]
fn what_is_wrong_with_one_as_written() {
    let base = "use std\ncomponent Bar(d: Layout) { }\n";
    said(base, Code::E101, &["`Layout` is not a type"]);
    said(&format!("{base}interface Layout {{ a: point }}\ninterface Layout {{ b: point }}\n"),
        Code::E071, &["`Layout` is defined twice"]);
    said(&format!("{base}interface Layout {{ a: point, a: line }}\n"), Code::E001, &["`a`"]);
    said(&format!("{base}interface Layout {{ s: Side }}\n"), Code::E103, &["no side"]);
    said(&format!("{base}interface Layout {{ next: Layout }}\n"), Code::E003, &["holds itself"]);
    said(&format!("{base}interface Layout {{ a: Missing }}\n"), Code::E101, &["`Missing`"]);
    said("component Layout() { }\ninterface Layout { a: point }\n", Code::E071, &["component"]);
    // a type of the language's own is no interface's name, and none is defined in a body
    for bad in ["interface Length { a: point }\n", "component C() {\ninterface I { a: point }\n}\n"] {
        let (_, errs) = syntax::parse(bad);
        assert_eq!(errs.len(), 1, "{errs:?}");
    }
}

#[test]
fn a_module_exports_one() {
    let shelf: BTreeMap<&str, &str> = [(
        "lib.gear",
        "interface Cone { O: point, module: Length }\n\
         component Mark(c: Cone) { c.O distance(c.module, along: u) q\nc.O level(v) q\n\
         q := point hint((1, 1)) }\n",
    )]
    .into_iter()
    .collect();
    let read = |src: &str| -> (Elaborated, Vec<Diag>) {
        let (mut prog, errs) = syntax::parse(src);
        assert!(errs.is_empty(), "{errs:?}");
        let linked = link(&mut prog, &mut |name| {
            shelf.get(name).map(|t| t.to_string()).or_else(|| library::resolve(name))
        });
        (elaborate(&prog), linked)
    };
    let doc = |head: &str, ty: &str| format!("\
unit mm
use std
{head}
component Again(c: {ty}) {{ lib.gear.Mark(c) }}
in std.front {{
  o := point
  fix((0, 0)) o
  design := {{O: o, module: 3mm}}
  Again(design)
}}
");
    for (head, ty) in [("use lib.gear", "lib.gear.Cone"), ("use lib.gear (Cone)", "Cone")] {
        let (e, linked) = read(&doc(head, ty));
        assert!(linked.is_empty() && e.ok(), "{:?}", e.diags);
    }
    let (e, _) = read(&doc("use lib.gear", "Cone"));
    let errs: Vec<_> = e.errors().collect();
    assert!(errs.len() == 1 && errs[0].code == Code::E101 && errs[0].message.contains("lib.gear.Cone"),
        "{errs:?}");
}

#[test]
fn the_word_is_coloured_and_kept_out_of_flat_text() {
    let src = "interface Layout { a: point }\n";
    let runs = syntax::highlight(src);
    assert!(runs.iter().any(|(t, s)| *t == syntax::Tint::Word && &src[s.lo as usize..s.hi as usize] == "interface"));
    let (mut p, errs) = syntax::parse(src);
    assert!(errs.is_empty());
    assert_eq!(syntax::render_flat(&mut p).unwrap_err().construct, "interface definitions");
}

#[test]
fn the_specifications_example_solves() {
    let e = solved("\
unit mm
use std
interface Size { length: Length }
interface Layout { origin: point, ax: line, size: Size }

component Bar(layout: Layout) {
  tip := point hint(at: layout.ax.p2)
  tip coincident layout.ax
  layout.origin distance(layout.size.length) tip
}
in std.front {
  o := point
  fix((0, 0)) o
  q := point
  fix((30, 40)) q
  ax := line(o, q)
  sizes := {length: 10mm}
  layout := {origin: o, ax: ax, size: sizes}
  bar := Bar(layout)
}
");
    let tip = e.sketch.point_xy(e.map.ent_named("bar.tip").unwrap().i());
    assert!((tip.0 - 6.0).abs() < 1e-8 && (tip.1 - 8.0).abs() < 1e-8, "{tip:?}");
}
