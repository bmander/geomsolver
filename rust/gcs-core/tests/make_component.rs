//! **Make component** (`edit::make_component`): what is selected, packaged as a `component` of
//! the document and drawn again as one instance of it — the moved text kept as written, what it
//! reads from outside handed in as formals, what stays outside naming it through the instance,
//! and the drawing the same drawing.

use gcs_core::edit::{self, Kind};
use gcs_core::model::EntRef;
use gcs_core::program::Elaborated;
use crate::common::{gestures::taken, parsed::read};

/// `src` elaborated and solved.
fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "does not solve: {}\n{src}", r.message);
    e
}

fn named(e: &Elaborated, n: &str) -> EntRef {
    e.map.ent_named(n).unwrap_or_else(|| panic!("no `{n}`"))
}

fn at(e: &Elaborated, n: &str) -> (f64, f64) {
    e.sketch.point_xy(named(e, n).i())
}

fn make_named(src: &str, names: &[&str], name: Option<&str>) -> edit::Edit {
    let e = read(src);
    let ents: Vec<EntRef> = names.iter().map(|n| named(&e, n)).collect();
    edit::make_component(&e, &e.program, &e.sketch, &ents, &[], name)
}

fn make(src: &str, names: &[&str]) -> edit::Edit {
    make_named(src, names, None)
}

const BASE: &str = "\
// a base, and this comment stays where it is
unit mm
use std
w := 100mm
in std.front {
a := point
b := point hint((100, 0))
ab := line(a, b)      // the base
horizontal ab
fix((0, 0)) a
a distance(w) b
}
c := point hint((50, 40)) in std.front
c distance(50mm, along: u) std.front
c distance(40mm, along: v) std.front
ac := line(a, c) in std.front
";

#[test]
fn a_selection_becomes_a_component_and_the_drawing_stays_the_drawing() {
    let d = make(BASE, &["a", "b", "ab"]);
    assert_eq!(d.names, ["Part", "part0"]);
    let text = taken(d);
    assert!(
        text.contains(
            "component Part(front: plane, w: Length) {\n  in front {\n    a := point\n    \
             b := point hint((100, 0))\n    ab := line(a, b)      // the base\n    \
             horizontal ab\n    fix((0, 0)) a\n    a distance(w) b\n  }\n}\n"
        ),
        "{text}"
    );
    assert!(text.contains("part0 := Part(std.front, w: w)\n"), "{text}");
    // the block held nothing else, so it went with what it held
    assert!(!text.contains("in std.front {"), "{text}");
    // what stayed outside names what moved through the instance
    assert!(text.contains("ac := line(part0.a, c) in std.front"), "{text}");
    assert!(text.starts_with("// a base, and this comment stays where it is\n"), "{text}");
    let (before, after) = (solved(BASE), solved(&text));
    for (was, is) in [("a", "part0.a"), ("b", "part0.b"), ("c", "c")] {
        let (p, q) = (at(&before, was), at(&after, is));
        assert!((p.0 - q.0).abs() < 1e-9 && (p.1 - q.1).abs() < 1e-9, "{was}: {p:?} {is}: {q:?}");
    }
}

#[test]
fn what_the_moved_text_reads_from_outside_is_handed_in_by_the_name_it_had() {
    let text = taken(make(BASE, &["c", "ac"]));
    // `a` stays, so the line is given it; the plane is std's, handed in as `front`
    assert!(
        text.contains(
            "component Part(front: plane, a: point) {\n  c := point hint((50, 40)) in front\n  \
             c distance(50mm, along: u) front\n  c distance(40mm, along: v) front\n  \
             ac := line(a, c) in front\n}"
        ),
        "{text}"
    );
    assert!(text.contains("part0 := Part(std.front, a)\n"), "{text}");
    // the block it did not touch is left as written
    assert!(text.contains("in std.front {\na := point\n"), "{text}");
    read(&text);
}

#[test]
fn a_relation_goes_with_what_it_relates_and_one_reaching_outside_stays() {
    // `ab` alone: its points stay, so they are formals, and the relations on the points stay
    // beside them, while `horizontal ab` names only what moved and would go — a definition that
    // says only that its two given points are level, refused in the elaborator's words (#562)
    let d = make(BASE, &["ab"]);
    assert_eq!(d.kind, Kind::None);
    assert!(d.refused.unwrap().contains("a fact about what it is given"));
    // `c` and `ac`: the line goes with its end, and reads `a`, which stays, as a formal
    let text = taken(make(BASE, &["c", "ac"]));
    assert!(text.contains("ac := line(a, c) in front"), "{text}");
    assert!(text.contains("fix((0, 0)) a\na distance(w) b\n}"), "{text}");
    read(&text);
}

#[test]
fn a_value_is_handed_in_typed_by_its_dimension() {
    let src = "\
unit mm
use std
r := 12mm
t := 30deg
k := 2
in std.front {
o := point
fix((0, 0)) o
q := point hint((20, 5))
o distance(r * k) q
std.x angle(t) line(o, q)
}
";
    let text = taken(make(src, &["o", "q"]));
    assert!(
        text.contains("component Part(front: plane, x: axis, r: Length, k: Scalar, t: Angle) {"),
        "{text}"
    );
    assert!(text.contains("part0 := Part(std.front, std.x, r: r, k: k, t: t)"), "{text}");
    // the anonymous line the angle is written over went with the relation it is part of
    assert!(text.contains("    x angle(t) line(o, q)\n  }\n}"), "{text}");
    assert!(text.ends_with("part0 := Part(std.front, std.x, r: r, k: k, t: t)\n"), "{text}");
    read(&text);
}

#[test]
fn an_instance_selected_moves_whole_and_the_definition_follows_the_documents_own() {
    let src = "\
use std
component Rung(a: point, len: Length) {
  b := point hint((50, 0))
  e := line(a, b)
  horizontal e
  a distance(len) b
}
in std.front {
l0 := point
r0 := point hint((50, 0))
t0 := Rung(l0, len: 50)
r0 coincident t0.b
fix((0, 0)) l0
}
";
    // one entity the instance made is enough to take the statement that made it
    let text = taken(make(src, &["t0.e", "r0"]));
    let rung = text.find("component Rung").unwrap();
    let part = text.find("component Part").unwrap();
    assert!(rung < part, "after the document's own components:\n{text}");
    assert!(
        text.contains("component Part(front: plane, l0: point) {\n  in front {\n    \
                       r0 := point hint((50, 0))\n    t0 := Rung(l0, len: 50)\n    \
                       r0 coincident t0.b\n  }\n}"),
        "{text}"
    );
    assert!(text.contains("part0 := Part(std.front, l0)\nin std.front {\nl0 := point\n"), "{text}");
    let (before, after) = (solved(src), solved(&text));
    let (p, q) = (at(&before, "r0"), at(&after, "part0.r0"));
    assert!((p.0 - q.0).abs() < 1e-9 && (p.1 - q.1).abs() < 1e-9, "{p:?} {q:?}");
}

#[test]
fn what_a_module_made_stays_and_is_counted() {
    let e = read(BASE);
    let ents = [named(&e, "a"), named(&e, "std.origin")];
    let d = edit::make_component(&e, &e.program, &e.sketch, &ents, &[], None);
    assert_eq!(d.kept, 1);
    read(&taken(d));
    // and std's alone is no component at all
    let d = edit::make_component(&e, &e.program, &e.sketch, &[named(&e, "std.origin")], &[], None);
    assert_eq!(d.kind, Kind::None);
    assert!(d.refused.unwrap().contains("no statement here to package"));
}

#[test]
fn a_name_is_the_callers_and_one_taken_is_refused() {
    let text = taken(make_named(BASE, &["c"], Some("Peg")));
    assert!(text.contains("component Peg(front: plane) {"), "{text}");
    assert!(text.contains("peg0 := Peg(std.front)"), "{text}");
    // a second package beside the first takes the next free name
    let again = make(&text, &["ac"]);
    assert_eq!(again.names, ["Part", "part0"]);
    let d = make_named(BASE, &["c"], Some("ab"));
    assert!(d.refused.unwrap().contains("already a name"));
    let d = make_named(BASE, &["c"], Some("line"));
    assert!(d.refused.unwrap().contains("cannot name a component"));
}

#[test]
fn a_line_shared_with_what_is_not_selected_is_refused_whole() {
    let src = "\
use std
in std.front {
p := point
q := point hint((40, 0))
r := point hint((40, 30))
(pq := line(p, q)) -> perpendicular (qr := line(q, r))
fix((0, 0)) p
fix((40, 0)) q
length(30mm) qr
}
";
    let d = make(src, &["pq"]);
    assert_eq!(d.kind, Kind::None);
    assert!(d.refused.unwrap().contains("written on one line"));
    // the whole chain goes, and the joint between its links with it
    let text = taken(make(src, &["pq", "qr", "p", "q", "r"]));
    let chain = "    (pq := line(p, q)) -> perpendicular (qr := line(q, r))\n";
    assert!(text.contains(chain), "{text}");
    read(&text);
}

#[test]
fn what_cannot_be_handed_in_is_refused_by_its_name() {
    let src = "\
use std
in std.front {
o := point
repeat 3 as i {
  p := point hint((10 * i, 5))
  fix((10 * i, 5)) p
}
q := point hint((0, 20))
fix((0, 20)) q
l := line(q, p[1])
}
";
    let d = make(src, &["q", "l"]);
    assert_eq!(d.kind, Kind::None);
    assert!(d.refused.unwrap().contains("reads `p`, which cannot be handed"));
    // the block's copies selected with it go whole, and then nothing is out of reach
    let e = read(src);
    let second = |ns: &Vec<String>| ns.iter().any(|n| n.ends_with(".1.p"));
    let copy = *e.map.names.iter().find(|(_, ns)| second(ns)).unwrap().0;
    let ents = [named(&e, "q"), named(&e, "l"), copy];
    let text = taken(edit::make_component(&e, &e.program, &e.sketch, &ents, &[], None));
    let block = "    repeat 3 as i {\n      p := point hint((10 * i, 5))\n      fix((10 * i, 5)) p\n    }\n";
    assert!(text.contains(block), "{text}");
}

#[test]
fn nothing_selected_is_refused() {
    let e = read(BASE);
    let d = edit::make_component(&e, &e.program, &e.sketch, &[], &[], None);
    assert_eq!(d.kind, Kind::None);
    assert!(d.refused.is_some());
}

/// What was packaged keeps being dragged: a component drawn once is reached by one instance
/// path, so its seeds are written back where the definition writes them (the writeback rule's
/// third clause), and drawn twice it is two poses of one statement and left alone.
#[test]
fn a_component_drawn_once_takes_its_drag_and_one_drawn_twice_does_not() {
    let text = taken(make(BASE, &["c"]));
    let mut e = read(&text);
    let c = named(&e, "part0.c").i();
    let [x, y] = e.sketch.point_params(c);
    e.sketch.params[x as usize].value = 61.0;
    e.sketch.params[y as usize].value = 47.0;
    let d = edit::commit_seeds(&e, &e.sketch, &e.program);
    assert_eq!(d.kind, Kind::Numeric);
    assert!(d.text.contains("  c := point hint((61, 47)) in front\n"), "{}", d.text);
    // the rest is as it was: only the numbers moved
    assert_eq!(d.text.replace("hint((61, 47))", "hint((50, 40))"), text);

    let twice = text.replace(
        "part0 := Part(std.front)\n",
        "part0 := Part(std.front)\npart1 := Part(std.front)\n",
    );
    let mut e = read(&twice);
    let c = named(&e, "part0.c").i();
    let [x, _] = e.sketch.point_params(c);
    e.sketch.params[x as usize].value = 61.0;
    let d = edit::commit_seeds(&e, &e.sketch, &e.program);
    assert!(d.text.contains("c := point hint((50, 40)) in front"), "{}", d.text);
}
