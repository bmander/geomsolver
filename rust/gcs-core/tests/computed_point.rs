//! `port` is retired (issue #47, item 1): everything an instance makes is reached by its dotted
//! name, so a port was a second name for a thing that already had one.  What survives is the
//! one real construct the keyword carried — the computed point — written now as the
//! declaration it is, `p := point(x: xexpr, y: yexpr)`, beside `p := point hint(x: …)`.

use gcs_core::program::elaborate;
use gcs_core::syntax::{write_stmt_to};
use crate::common::parse;

fn errors(src: &str) -> Vec<String> {
    let (prog, errs) = parse(src);
    let e = elaborate(&prog);
    let mut all: Vec<String> = errs.iter().map(|x| x.message.clone()).collect();
    all.extend(e.errors().map(|d| d.message.clone()));
    all
}

fn refuses(src: &str, needle: &str) {
    let msgs = errors(src);
    assert!(msgs.iter().any(|m| m.contains(needle)), "expected `{needle}`\n{src}\n{msgs:?}");
}

/// Each of the three forms a port took is refused, and the message names what replaces them.
#[test]
fn port_is_retired_and_says_what_to_write() {
    for form in ["port lo: point hint((0, 0))", "port hub = c", "port p = (c.x, c.y)"] {
        let src = format!("component H(c: point) {{\n  {form}\n}}\no := point hint((0, 0))\nh := H(o)\n");
        refuses(&src, "`port` is retired");
        refuses(&src, "`p := point(x: …, y: …)`");
    }
}

/// Everything an instance makes is reached by its dotted name — a point of the body, a line,
/// the entity a formal aliases, a copy inside a block — with no export list between.
#[test]
fn an_instances_entities_are_reached_by_dotted_name() {
    let src = "\
use std
component Rung(a: point, len: Length) {
  b := point hint((a.x + len, a.y))
  e := line(a, b)
  a distance(len) b
  repeat 2 as i {
    q := point hint((a.x, a.y + 10 * (i + 1)))
  }
}
in std.front {
o := point
r := Rung(o, len: 30)
fix((0, 0)) r.a
horizontal r.e
r.b distance(5) r.q[1]
}
";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    // the formal is the actual — one entity under two names — so `fix(…) r.a` pinned `o`
    assert_eq!(e.sketch.points.len(), 4 + crate::common::STD_POINTS,
        "o, b and the two copies of q, and no fifth for `r.a`");
    let o = e.map.ent_named("o").expect("o");
    assert!(e.sketch.own_params(o).iter().all(|&p| e.sketch.params[p as usize].fixed));
    assert!(e.map.ent_named("r.b").is_some(), "a point of the body");
    assert!(e.map.ent_named("r.e").is_some(), "a line of the body");
}

/// A computed point is a declaration made of a formula: it prints as written, is drawn only as
/// a curve, and is refused on the sheet, on any kind but a point, and without a name.
#[test]
fn a_computed_point_is_a_declaration() {
    let ray = "component Ray(c: point, u: Angle) {\n  p := point(x: c.x + cos(u), y: c.y + sin(u))\n}\n";
    let src = format!("{ray}o := point\nf := Ray(o).p over u in (0, 90)\nfix(x == 0, y == 0) o\n");
    let (prog, errs) = parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(e.sketch.curves.len(), 1);
    // the printer spells it as the declaration it is
    let comp = prog
        .components
        .iter()
        .find(|c| c.name.as_ref().is_some_and(|n| n.text == "Ray"))
        .expect("the component");
    let mut out = String::new();
    write_stmt_to(&mut out, &comp.body[0].kind).unwrap();
    assert_eq!(out.split_whitespace().collect::<Vec<_>>().join(" "), "p := point(x: c.x + cos(u), y: c.y + sin(u))");
    // nothing on the sheet holds a point to a formula — neither written there nor drawn there
    refuses("use std\nin std.front {\no := point hint((0, 0))\np := point(x: o.x + 1, y: o.y)\n}\n", "computed point");
    refuses(&format!("{ray}o := point hint((0, 0))\nr := Ray(o)\n"), "drawn only as a curve");
    // and the form is a point's alone
    refuses("p := point(x: 1)\n", "both `x:` and `y:`");
    refuses("point(x: 1, y: 2)\n", "a computed point is named");
}
