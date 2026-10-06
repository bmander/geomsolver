//! Seeds that read geometry (§6.4): `hint(x: k.center.x + k.r, y: pin.y)` reads the *seeds* of
//! the scalars it names, `hint(at: pin)` and `hint(at: k, bearing: b)` name a place outright, and a
//! seed inside a child slot is settled over a component's parameters like any other.  A seed is
//! where a solve begins and nothing more, so none of this changes what a document says.

use gcs_core::edit;
use gcs_core::program::{elaborate, Elaborated};
use crate::common::parse;

fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let e = elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}\n{src}",
        e.errors().map(|d| d.message.clone()).collect::<Vec<_>>()
    );
    e
}

fn refused(src: &str, needle: &str) {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let e = elaborate(&prog);
    assert!(
        e.errors().any(|d| d.message.contains(needle)),
        "expected `{needle}`\n{}",
        e.diags.iter().map(|d| d.message.clone()).collect::<Vec<_>>().join("\n")
    );
}

fn xy(e: &Elaborated, name: &str) -> (f64, f64) {
    // `q[1]` is copy 1 of a block's `q`, which the map calls `#<stmt>.1.q`
    let r = match name.split_once('[') {
        Some((leaf, k)) => {
            let tail = format!(".{}.{leaf}", k.trim_end_matches(']'));
            *e.map
                .names
                .iter()
                .find(|(_, ns)| ns.iter().any(|n| n.ends_with(&tail)))
                .map(|(r, _)| r)
                .unwrap_or_else(|| panic!("no `{name}`"))
        }
        None => e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")),
    };
    e.sketch.point_xy(r.i())
}

#[test]
fn a_seed_reads_the_seed_of_what_it_names() {
    let e = read(
        "\
use std
in std.front {
a := point hint(x: 5, y: 1)
k := circle(center: a) hint(r: 4)
b := point hint(x: a.x + 10, y: k.r)
c := point hint(at: a)
d := point hint(at: k, bearing: 90deg)
e := point hint(x: k.center.x + k.r, y: 0)
l := line(hint(x: a.x, y: 1), hint(x: a.x + 1, y: 1))
}
",
    );
    assert_eq!(xy(&e, "b"), (15.0, 4.0));
    assert_eq!(xy(&e, "c"), (5.0, 1.0));
    let (dx, dy) = xy(&e, "d");
    assert!((dx - 5.0).abs() < 1e-9 && (dy - 5.0).abs() < 1e-9, "{dx} {dy}");
    assert_eq!(xy(&e, "e"), (9.0, 0.0));
    assert_eq!(xy(&e, "l.p2"), (6.0, 1.0));
}

#[test]
fn a_component_seeds_from_its_formals_geometry() {
    let e = read(
        "\
use std
component Off(p: point, d: Length) {
q := point hint(x: p.x + d, y: p.y)
s := line(p, hint(x: p.x + d / 2, y: p.y + d))
}
in std.front {
a := point hint(x: 10, y: 20)
o := Off(a, d: 6)
}
",
    );
    assert_eq!(xy(&e, "o.q"), (16.0, 20.0));
    assert_eq!(xy(&e, "o.s.p2"), (13.0, 26.0));
}

#[test]
fn a_component_receives_the_files_values_explicitly() {
    let e = read(
        "\
use std
w := 30
component Bar(a: point, w: Length) {
h := w / 2
b := point hint(x: a.x + w, y: a.y + h)
l := line(a, b)
a distance(w) b
}
in std.front {
o := point hint(x: 0, y: 0)
r := Bar(o, w: w)
}
",
    );
    assert_eq!(xy(&e, "r.b"), (30.0, 15.0));
    // the caller may choose a different value
    let e = read(
        "\
use std
w := 30
component Bar(a: point, w: Length) {
b := point hint(x: a.x + w, y: a.y)
}
in std.front {
o := point hint(x: 0, y: 0)
r := Bar(o, w: 7)
}
",
    );
    assert_eq!(xy(&e, "r.b"), (7.0, 0.0));
}

#[test]
fn a_geometric_seed_follows_the_scope_it_was_written_in() {
    // the tangent span: its seeds name the circles it is written over, through the formals
    let e = read(
        "\
unit mm
use std
component Span(k1: circle, k2: circle, side: Scalar) {
a := point hint(at: k1, bearing: atan2(k2.center.y - k1.center.y, k2.center.x - k1.center.x) + side * 90deg)
b := point hint(at: k2, bearing: atan2(k2.center.y - k1.center.y, k2.center.x - k1.center.x) + side * 90deg)
s := line(a, b)
}
in std.front {
o := point hint(x: 0, y: 0)
c := point hint(x: 100, y: 0)
k1 := circle(center: o) hint(r: 10)
k2 := circle(center: c) hint(r: 20)
up := Span(k1, k2, side: 1)
dn := Span(k1, k2, side: -1)
}
",
    );
    let (ax, ay) = xy(&e, "up.a");
    assert!((ax - 0.0).abs() < 1e-9 && (ay - 10.0).abs() < 1e-9, "{ax} {ay}");
    let (bx, by) = xy(&e, "dn.b");
    assert!((bx - 100.0).abs() < 1e-9 && (by + 20.0).abs() < 1e-9, "{bx} {by}");
    // and inside a block, a copy's own
    let e = read(
        "repeat 2 as i {\n\
           p := point hint(x: i * 10, y: 0)\n\
           q := point hint(x: p.x + 1, y: p.y)\n\
         }\n",
    );
    assert_eq!(xy(&e, "q[1]"), (11.0, 0.0));
}

#[test]
fn a_seed_that_reads_geometry_is_never_written_back() {
    let src = "use std\nin std.front {\na := point hint(x: 5, y: 1)\nb := point hint(x: a.x + 10, y: 2)\nc := point hint(at: a)\na distance(3) b\n}\n";
    let e = read(src);
    let mut sk = e.sketch.clone();
    gcs_core::solve::solve(&mut sk, Default::default());
    let out = edit::commit_seeds(&e, &sk, &e.program);
    // `b`'s x is an expression and `c` is a place: neither is spliced; only the moved numbers are
    assert!(out.text.contains("hint(x: a.x + 10, y: "), "{}", out.text);
    assert!(out.text.contains("c := point hint(at: a)"), "{}", out.text);
}

/// **A place is a step from a point** (§6.4): `toward:` a point, `by:` of the way (all of it
/// unsaid), `turn:`ed about the start — or `along:` a line's run in place of `toward:`.  A
/// midpoint, a reflection, an extension and a quarter turn are each one clause, read off the
/// seeds of what they name in statement order, as every other place is; the printer spells the
/// clause back as it was read, and a commit never writes one back.
#[test]
fn a_place_is_a_step_from_a_point() {
    let src = "\
use std
in std.front {
a := point hint(x: 10, y: 0)
b := point hint(x: 30, y: 0)
l := line(a, b)
mid := point hint(at: a, toward: b, by: 0.5)
back := point hint(at: a, toward: b, by: -1)
up := point hint(at: a, toward: b, turn: 90deg)
on := point hint(at: mid, along: l, by: 2, turn: -90deg)
a distance(20) b
}
";
    let e = read(src);
    for (name, want) in [("mid", (20.0, 0.0)), ("back", (-10.0, 0.0)), ("up", (10.0, 20.0)),
                         ("on", (20.0, -40.0))] {
        let (x, y) = xy(&e, name);
        assert!((x - want.0).abs() < 1e-9 && (y - want.1).abs() < 1e-9, "{name}: {x} {y}");
    }
    let (prog, errs) = parse("\
use std
in std.front {
q := point hint(at: a, toward: b.p2, by: 0.5, turn: 90deg)
r := point hint(at: a, along: l, by: -2)
}
");
    assert!(errs.is_empty(), "{errs:?}");
    let printed: Vec<String> = prog.root().body.iter().map(|st| {
        let mut out = String::new();
        gcs_core::syntax::write_stmt_to(&mut out, &st.kind).unwrap();
        out.split_whitespace().collect::<Vec<_>>().join(" ")
    }).collect();
    assert_eq!(printed, ["q := point hint(at: a, toward: b.p2, by: 0.5, turn: 90deg)",
                         "r := point hint(at: a, along: l, by: -2)"]);
    let mut sk = e.sketch.clone();
    gcs_core::solve::solve(&mut sk, Default::default());
    let out = edit::commit_seeds(&e, &sk, &e.program);
    assert!(out.text.contains("mid := point hint(at: a, toward: b, by: 0.5)"), "{}", out.text);
    let a = "use std\nin std.front {\na := point hint(x: 0, y: 0)\nl := line(a, a)\n}\n";
    for (clause, want) in [
        ("hint(at: a, by: 0.5)", "need `toward:` or `along:`"),
        ("hint(toward: a)", "`toward:` says where from a place"),
        ("hint(at: a, toward: a, along: l)", "one or the other"),
        ("hint(at: a, toward: a, bearing: 3)", "one or the other"),
    ] {
        let (_, errs) = parse(&format!("{a}q := point {clause}\n"));
        let said: Vec<_> = errs.iter().map(|e| &e.message).collect();
        assert!(said.iter().any(|m| m.contains(want)), "expected `{want}` from {clause}: {said:?}");
    }
    let k = "use std\nin std.front {\na := point hint(x: 0, y: 0)\nk := circle(center: a) hint(r: 3)\n}\n";
    refused(&format!("{k}q := point hint(at: k, toward: a)\n"), "a step starts at a point");
    refused(&format!("{k}q := point hint(at: a, toward: k)\n"), "toward a point, not a circle");
    refused(&format!("{k}q := point hint(at: a, along: a)\n"), "along a line, not a point");
}

/// **A place in another view is read in space** and projected into the seeded point's view,
/// which is what `project` says of the pair: a page point on the fold line of a view folded
/// square to the page starts at its own image there.  The memberships and the views' poses
/// that reading needs are worked out after the first settle, so the seeds are settled again in
/// statement order — a seed reading a projected one follows it — and the second settle's
/// findings replace the first's rather than doubling them.
#[test]
fn a_place_in_another_view_is_read_in_space() {
    let views = "use std\nside := plane(u: std.z, v: std.y)\nfix(x == 0, y == 0, z == 0) side\n\
         t := point hint(x: 10, y: 0) in std.front\n\
         p := point hint(x: 7, y: 3) in std.front\n";
    // the front is u = x, v = z, and `side` u = z, v = y: `p` at (7, 3) in the front is (3, 0)
    // in `side`, and a step from it toward `t` goes halfway
    let e = read(&format!("{views}q := point hint(at: p) in side\n\
         r := point hint(at: p, toward: t, by: 0.5) in side\n\
         s := point hint(at: r, toward: q, by: 2) in side\n\
         u := point hint(at: q) in std.front\n"));
    let wants = [("q", (3.0, 0.0)), ("r", (1.5, 0.0)), ("s", (4.5, 0.0)), ("u", (0.0, 3.0))];
    for (name, want) in wants {
        let (x, y) = xy(&e, name);
        assert!((x - want.0).abs() < 1e-9 && (y - want.1).abs() < 1e-9, "{name}: {x} {y}");
    }
    // the same findings as a document settled once, though these seeds were settled twice
    let findings = |src: String| {
        let (prog, errs) = parse(&src);
        assert!(errs.is_empty(), "{errs:?}");
        elaborate(&prog).errors().map(|d| d.message.clone()).collect::<Vec<_>>()
    };
    let bad = "in std.front {\nw := point hint(x: nobody.x, y: 0)\n}\n";
    let once = findings(format!("{views}q := point hint(x: 3, y: 0) in side\n{bad}"));
    assert!(!once.is_empty());
    assert_eq!(findings(format!("{views}q := point hint(at: p) in side\n{bad}")), once);
}

/// An arc's radius the source leaves unwritten is its centre to its start — and read once the
/// places are settled, since a centre seeded by one stands where it was placed only then.
#[test]
fn an_unwritten_arc_radius_reads_its_placed_centre() {
    let e = read("\
use std
in std.front {
a := point hint(x: 0, y: 0)
b := point hint(x: 20, y: 0)
s := point hint(x: 0, y: 10)
f := point hint(x: 20, y: 10)
c := point hint(at: a, toward: b, by: 0.5)
k := arc(center: c, start: s, end: f)
}
");
    let r = e.sketch.params[e.sketch.arcs[0].radius as usize].value;
    assert!((r - 200f64.sqrt()).abs() < 1e-9, "{r}");
}

#[test]
fn what_a_seed_may_read_is_checked() {
    refused("use std\nin std.front {\na := point hint(x: 0, y: 0)\nb := point hint(x: a.z, y: 0)\n}\n", "has no `z`");
    refused("use std\nin std.front {\na := point hint(x: 0, y: 0)\nb := point hint(x: nobody.x, y: 0)\n}\n", "no such entity");
    refused("use std\nin std.front {\na := point hint(x: 0, y: 0)\nk := circle(center: a) hint(r: 3)\nd := point hint(at: k)\n}\n", "where on the edge");
    refused("use std\nin std.front {\na := point hint(x: 0, y: 0)\nk := circle(center: a) hint(r: a.x)\nl := line(a, hint(x: 1, y: 1)) hint(at: a)\n}\n", "only a point");
    // a `param` is not a seed: it feeds constraints, and a seed may not change what a document says
    refused("use std\nin std.front {\na := point hint(x: 0, y: 0)\n}\nw := a.x + 3\n", "not a number here");
}

#[test]
fn a_geometry_read_is_a_length_where_the_document_names_a_unit() {
    read("unit mm\nuse std\nin std.front {\na := point hint(x: 5, y: 1)\nb := point hint(x: a.x + 10mm, y: 0)\n}\n");
    refused("unit mm\nuse std\nin std.front {\na := point hint(x: 5, y: 1)\nb := point hint(x: a.x + 10, y: 0)\n}\n", "cannot be added");
    // and a bare number where it does not — there is nothing else a number could be
    read("use std\nin std.front {\na := point hint(x: 5, y: 1)\nb := point hint(x: a.x + 10, y: 0)\n}\n");
}

/// **A place is two keys of the one seed clause** (issue #47, item 2): `at:` and `bearing:`
/// beside `x`, `y` and `r`, in any order, and the printer spells the clause back as it was
/// read.  A clause naming a place carries no scalar, a bearing needs a place, `at:` names and
/// is not a pair, and the grammar the keys replaced says what it became — each refused where
/// it is written, since the mistake is the key's and not the declaration's.
#[test]
fn a_place_is_two_keys_of_the_seed_clause() {
    let e = read(
        "\
use std
in std.front {
a := point hint(x: 5, y: 1)
k := circle(center: a) hint(r: 4)
d := point hint(bearing: 90deg, at: k)
}
",
    );
    let (dx, dy) = xy(&e, "d");
    assert!((dx - 5.0).abs() < 1e-9 && (dy - 5.0).abs() < 1e-9, "{dx} {dy}");
    // the printer's one spelling of the clause is the keyed one
    let (prog, errs) = parse("use std\nin std.front {\nd := point hint(at: k, bearing: 90deg)\nc := point hint(at: a.p1)\n}\n");
    assert!(errs.is_empty(), "{errs:?}");
    let printed: Vec<String> = prog
        .root()
        .body
        .iter()
        .map(|st| {
            let mut out = String::new();
            gcs_core::syntax::write_stmt_to(&mut out, &st.kind).unwrap();
            // the printer pads the kind to a column; the clause is what is under test
            out.split_whitespace().collect::<Vec<_>>().join(" ")
        })
        .collect();
    assert_eq!(printed, ["d := point hint(at: k, bearing: 90deg)", "c := point hint(at: a.p1)"]);
    for (src, want) in [
        ("a := point hint(x: 0, y: 0)\nq := point hint(z: 3, at: a)\n", "carries no scalar"),
        ("r := point hint(bearing: 30)\n", "needs `at:`"),
        ("s := point hint(at: (3, 4))\n", "a coordinate seed is `hint(x: …, y: …)`"),
        ("a := point hint(x: 0, y: 0)\nq := point hint(at: a, at: a)\n", "written twice"),
        ("a := point hint(x: 0, y: 0)\nq := point hint at a\n", "a place is keyed now"),
        ("a := point hint(x: 0, y: 0)\nq := point hint at a bearing (30)\n", "a place is keyed now"),
    ] {
        let (_, errs) = parse(src);
        assert!(
            errs.iter().any(|e| e.message.contains(want)),
            "expected `{want}` from {src:?}, got {:?}",
            errs.iter().map(|e| &e.message).collect::<Vec<_>>()
        );
    }
    // `x:` and `y:` beside `at:` are a place in a plane, so a point is no place for them
    let (prog, _) = crate::common::parse(
        "use std\nin std.front {\na := point hint(x: 0, y: 0)\nq := point hint(at: a, x: 3)\n}\n");
    let e = elaborate(&prog);
    assert!(e.errors().any(|d| d.message.contains("are a place in a plane")), "{:?}", e.diags);
}

/// **A held number is its own seed**: whatever a `fix` holds needs no `hint(…)` saying it again,
/// so a document with the hints and one without elaborate to the same numbers, bit for bit —
/// the places of a plane held off the origin, the axes it minted standing through it, an axis
/// held along a line whose ends are held, and a point held in that plane.
#[test]
fn a_fix_is_its_own_seed() {
    let with = "use std
P := plane hint(x: 5, y: 40, z: -3)
fix(x == 5, y == 40, z == -3) P
fix(x == 1, y == 0, z == 0) P.u
fix(x == 0, y == 0, z == 1) P.v
in P {
  q := point hint(x: 2, y: 9)
  fix(x == 2, y == 9) q
}
in std.front {
  a := point hint(x: 10, y: 20)
  b := point hint(x: 30, y: 25)
  fix(x == 10, y == 20) a
  fix(x == 30, y == 25) b
  l := line(a, b)
  m := line(a, hint(x: 10, y: 40))
}
Q := plane(u: l, v: m)
";
    let without = with.replace(" hint(x: 5, y: 40, z: -3)", "")
        .replace(" hint(x: 2, y: 9)", "")
        .replace(" hint(x: 10, y: 20)", "")
        .replace(" hint(x: 30, y: 25)", "");
    assert_ne!(with, without);
    let (a, b) = (read(with), read(&without));
    let values = |e: &Elaborated| e.sketch.params.iter().map(|p| (p.name.clone(), p.value, p.fixed))
        .collect::<Vec<_>>();
    assert_eq!(values(&a), values(&b));
}
