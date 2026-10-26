//! **An instance's numbers, read from outside it** ([0.188]): `shield.top` names the number
//! `Shield`'s body defines as `top`, worked out from what the call gave it, as `ball.r` names the
//! number a call gave the formal `r` — in a definition, a dimension or another call's argument,
//! in whatever order they are written, through instances nested in it (`st.base.top`) and in a
//! body read from a module.  So the formula is written once, in the component, and a reader outside
//! it has no copy to drift from it.

use crate::common::linked::build as elaborated;
use gcs_core::modules::link;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::parse;
use gcs_core::units::Dim;

fn ok(src: &str) -> Elaborated {
    let e = elaborated(src);
    assert!(e.ok(), "{:?}\n{src}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    e
}

/// The number the document names `name`, in its unit, and its dimension.
fn value(e: &Elaborated, name: &str) -> (f64, Dim) {
    let v = e.values.get(name).unwrap_or_else(|| panic!("no value `{name}`: {:?}", e.values));
    (v.value, v.dim)
}

/// What `src` is refused with: each error's code and words.
fn refused(src: &str) -> Vec<(String, String)> {
    let e = elaborated(src);
    let said: Vec<(String, String)> =
        e.errors().map(|d| (d.code.as_str().to_string(), d.message.clone())).collect();
    assert!(!said.is_empty(), "elaborated\n{src}");
    said
}

/// A plate's sizes, and the numbers its body works out of them.
const PLATE: &str = "\
interface Sizes { w: Length, t: Length }
sizes := { w: 10mm, t: 2mm }
component Plate(s: Sizes, lift: Length = 1mm) {
  half := s.w / 2
  top := half + s.t + lift
  dims := { inner: top - s.t }
}
";

/// **A number a body defines is read by the instance's name**, as a formal is: in a definition,
/// a dimension and another call's argument; a group's member by its path (`b.dims.inner`); a
/// formal the call left to its default as the default.
#[test]
fn a_number_a_body_defines_is_read_from_outside_it() {
    let e = ok(&format!("{PLATE}\
use std
b := Plate(sizes, lift: 3mm)
z := b.top + 2mm
inner := b.dims.inner
lift := b.lift
low := Plate(sizes)
low_top := low.top
twice := Plate(sizes, lift: b.top)
in std.front {{
  u := point
  v := point hint((9, 1))
  fix((0, 0)) u
  u distance(b.top) v
}}
"));
    assert_eq!(value(&e, "z"), (12.0, Dim::LENGTH));
    assert_eq!(value(&e, "inner"), (8.0, Dim::LENGTH));
    assert_eq!(value(&e, "lift"), (3.0, Dim::LENGTH));
    assert_eq!(value(&e, "low_top"), (8.0, Dim::LENGTH));
    // the instance's own walk says the same number under its own name
    assert_eq!(value(&e, "b.top"), (10.0, Dim::LENGTH));
    assert_eq!(value(&e, "twice.top"), (17.0, Dim::LENGTH));
    let mut sk = e.sketch.clone();
    assert!(gcs_core::solve::solve(&mut sk, Default::default()).success);
    let (x, y) = sk.point_xy(e.map.ent_named("v").unwrap().i());
    assert!((x.hypot(y) - 10.0).abs() < 1e-9, "{x} {y}");
}

/// **A body is a set**, and so is the document reading it: a definition above the call reads it,
/// and the call's own numbers may be defined below both — each worked out once what it reads is.
/// A formal read in a definition (`b.lift`) is ready there too, where it once was read only by
/// the statements after the definitions.
#[test]
fn what_a_call_reads_is_worked_out_before_its_numbers_are_read() {
    let e = ok(&format!("{PLATE}\
z := b.top * 2
b := Plate(sizes, lift: up + 1mm)
up := lift_of_b / 2
lift_of_b := 4mm
l := b.lift
"));
    assert_eq!(value(&e, "z"), (20.0, Dim::LENGTH));
    assert_eq!(value(&e, "l"), (3.0, Dim::LENGTH));
}

/// **Through an instance the body writes**: `st.base.top` is `base`'s `top` inside `st`, worked
/// out over the numbers `st`'s body gives `base` — its own definitions among them.
#[test]
fn a_path_reads_through_instances_nested_in_one() {
    let e = ok(&format!("{PLATE}\
component Stack(s: Sizes, n: Int) {{
  extra := s.t * n
  base := Plate(s, lift: extra)
  height := base.top * n
}}
st := Stack(sizes, n: 2)
h := st.height
deep := st.base.top
"));
    assert_eq!(value(&e, "deep"), (11.0, Dim::LENGTH));
    assert_eq!(value(&e, "h"), (22.0, Dim::LENGTH));
}

/// **A body's names hide its file's constants** where it is read from outside, as where it is
/// walked; and a body of the document reads the document's constants, defined in any order.
#[test]
fn a_body_read_from_outside_reads_its_own_names_first_and_then_its_files() {
    let e = ok("\
k := c * 2
c := 4mm
pitch := 100mm
component P(w: Length) {
  pitch := 1mm
  top := w + c + pitch
}
p := P(w: k)
z := p.top
");
    assert_eq!(value(&e, "z"), (13.0, Dim::LENGTH));
}

/// **Inside a body**, an instance it writes is read so too: a definition and a call's argument
/// read `inner.top`, and the document reads `o.tall` and `o.again.top` through it.
#[test]
fn a_body_reads_the_numbers_of_the_instances_it_writes() {
    let e = ok("\
component P(w: Length) {
  top := w + 1mm
}
component Outer(w: Length) {
  inner := P(w: w * 2)
  tall := inner.top + 1mm
  again := P(w: inner.top)
}
o := Outer(w: 1mm)
tall := o.tall
again := o.again.top
");
    assert_eq!(value(&e, "tall"), (4.0, Dim::LENGTH));
    assert_eq!(value(&e, "again"), (4.0, Dim::LENGTH));
}

/// The modules a test reads: the radiator bot's boards, in small — an interface, a group of
/// sizes and a component of the module reading them, and its own constant.
fn module_read(src: &str) -> Elaborated {
    let boards = "\
interface UnoSizes { pcb_t: Length, socket_h: Length, usb_h: Length }
uno_dims := { pcb_t: 1.6mm, socket_h: 8.5mm, usb_h: 3.2mm }
header := 2.5mm
component Uno(s: UnoSizes) {
  z1 := s.pcb_t
  usb_z := z1 + s.usb_h / 2
}
component Shield(u: UnoSizes, pcb_t: Length) {
  top := u.pcb_t + u.socket_h + header + pcb_t
}
";
    let (mut prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let linked = link(&mut prog, &mut |name| {
        (name == "boards").then(|| boards.to_string()).or_else(|| gcs_core::library::resolve(name))
    });
    assert!(linked.is_empty(), "{linked:?}");
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    e
}

/// **A module's component read from the document** (the issue's case): the shield's top face and
/// the Uno's USB axis are the components' own numbers, read where the document places the
/// boards, over the sizes it hands them and the module's own constants.
#[test]
fn a_modules_component_is_read_from_the_document_that_calls_it() {
    let e = module_read("\
use boards (Uno, Shield, uno_dims)
uno_z := 40mm
uno := Uno(uno_dims)
shield := Shield(u: uno_dims, pcb_t: 1.6mm)
shield_z := uno_z + shield.top
usb_z := uno_z + uno.usb_z
");
    let (z, dim) = value(&e, "shield_z");
    assert!((z - 54.2).abs() < 1e-9 && dim == Dim::LENGTH, "{z}");
    let (z, _) = value(&e, "usb_z");
    assert!((z - 43.2).abs() < 1e-9, "{z}");
}

/// **What a number reads of the call it is read through** waits for it, so a call given a number
/// that reads the instance back reads itself (E041), each said where it is written.
#[test]
fn a_call_reading_its_own_number_back_is_defined_in_terms_of_itself() {
    let said = refused("\
component P(w: Length) {
  top := w + 1mm
}
q := P(w: z)
z := q.top
");
    let cycle = "`z` is defined in terms of itself";
    assert!(said.iter().any(|(c, m)| c == "E041" && m.contains(cycle)), "{said:?}");
}

/// **A number the call leaves the body unable to work out** is said where it is read from
/// outside, with why: here `w * w` of a formal left unbound, which is an unknown of the drawing.
/// One the call leaves unbound and the number only offsets is affine in that unknown, as the
/// formal itself is (`free.w`), and no value of the document's.
#[test]
fn a_number_the_call_does_not_work_out_is_said_where_it_is_read() {
    let said = refused("\
component P(w: Length) {
  top := w + 1mm
  sq := w * w
}
free := P()
s := free.sq
");
    assert!(said.iter().any(|(c, m)| c == "E103"
        && m.contains("`free.sq` is read from outside `free`, and `P`'s `sq` is not worked out")),
        "{said:?}");
    let e = elaborated("\
component P(w: Length) {
  top := w + 1mm
}
free := P()
t := free.top
");
    assert!(!e.errors().any(|d| d.message.contains("free.top")),
        "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert!(!e.values.contains_key("t"));
}

/// **Only a number is read so**: geometry an instance makes is still named by its path, and a
/// name its body does not define is still nothing.
#[test]
fn a_name_that_is_no_number_of_the_body_is_not_read_as_one() {
    let said = refused(&format!("{PLATE}\
b := Plate(sizes)
z := b.nothing
"));
    assert!(said.iter().any(|(c, m)| c == "E103" && m.contains("`b.nothing` is not a number here")),
        "{said:?}");
}

/// **A name the body gives a group is no number**: `x.alias`, where the body says `alias :=
/// sizes`, is the group as before, and reading it whole says nothing of numbers; a number of the
/// group itself is read by its path (`x.sizes.k`).
#[test]
fn a_name_for_a_group_read_from_outside_is_the_group() {
    let e = elaborated("\
component P(w: Length) {
  sizes := { k: w }
  alias := sizes
}
component G(s: group) {
  t := 1mm
}
x := P(w: 5mm)
g := G(x.alias)
k := x.sizes.k
");
    let said: Vec<&String> = e.errors().map(|d| &d.message).collect();
    assert!(!said.iter().any(|m| m.contains("read from outside")), "{said:?}");
    assert_eq!(value(&e, "k"), (5.0, Dim::LENGTH));
}
