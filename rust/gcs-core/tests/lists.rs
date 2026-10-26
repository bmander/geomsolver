//! **Lists, and a block over one** (§12.10): `holes := [(a, b), (c, d)]` is a group whose
//! members are its items by place, and `repeat h in holes { … }` (or the list written in place,
//! `repeat h in [(a, b), …]`) makes a copy per item, `h` that copy's item — a number, or a group
//! of numbers read by member (`h.x`).  A pattern that sits on no grid is said as the places it
//! is, not as an index's arithmetic.
//!
//! The copies are a `repeat`'s in every other respect (`#<id>.<k>.`, `hole[2]` from outside), so
//! what is worth testing is the binding: that `h` is the item and nothing else, wherever the
//! list is reached from, and that what is not a list is refused where it is written.

use gcs_core::modules::link;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::{highlight, parse, render_flat, StmtKind, Tint};
use crate::common::linked::solved;

/// Every error an elaboration reports, as `code: message`.
fn errors(src: &str) -> Vec<String> {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    elaborate(&prog).errors().map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect()
}

fn refused(src: &str, code: &str, needle: &str) {
    let errs = errors(src);
    assert!(
        errs.iter().any(|m| m.starts_with(code) && m.contains(needle)),
        "expected {code} `{needle}`\n{src}\n{errs:?}"
    );
}

/// Where the point `name` landed, in its plane.
fn at(e: &Elaborated, name: &str) -> (f64, f64) {
    let p = e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`"));
    let [x, y] = e.sketch.point_params(p.i());
    (e.sketch.params[x as usize].value, e.sketch.params[y as usize].value)
}

/// The points named `….{leaf}` of the copies, in copy order.
fn copies(e: &Elaborated, leaf: &str) -> Vec<(f64, f64)> {
    let mut names: Vec<String> = e.map.names.values().flatten()
        .filter(|n| n.starts_with('#') && n.ends_with(&format!(".{leaf}")))
        .cloned().collect();
    names.sort();
    names.iter().map(|n| at(e, n)).collect()
}

fn near(got: &[(f64, f64)], want: &[(f64, f64)]) {
    assert_eq!(got.len(), want.len(), "{got:?}");
    for (g, w) in got.iter().zip(want) {
        assert!((g.0 - w.0).abs() < 1e-9 && (g.1 - w.1).abs() < 1e-9, "{got:?} against {want:?}");
    }
}

/// The case the construct is for: an Uno's four mounting holes, which sit on no grid.  Each copy
/// reads its own item, one of them through a name the list reads, and its index besides.
#[test]
fn a_copy_per_item_reads_its_own() {
    let e = solved("\
unit mm
use std
use std.place (coords)
top_y := 48.26mm
holes := [(-13.97mm, 0mm), (-12.7mm, top_y), (38.1mm, 5.08mm), (38.1mm, 33.02mm)]
in std.top {
  repeat h in holes as i {
    c := point
    c coords(du: h.x, dv: h.y) std.top
    hole := circle(center: c) hint(r: 1)
    radius(1.6mm + i * 0.1mm) hole
  }
}
");
    near(&copies(&e, "c"), &[(-13.97, 0.0), (-12.7, 48.26), (38.1, 5.08), (38.1, 33.02)]);
    let mut radii: Vec<f64> =
        e.sketch.circles.iter().map(|c| e.sketch.params[c.radius as usize].value).collect();
    radii.sort_by(f64::total_cmp);
    let want = [1.6, 1.7, 1.8, 1.9];
    assert!(radii.iter().zip(want).all(|(r, w)| (r - w).abs() < 1e-9), "{radii:?}");
}

/// A list written in place, of numbers: `h` is the number.  The copies are reached by index
/// from outside, as a `repeat`'s are.
#[test]
fn a_list_written_in_place_and_its_copies_by_index() {
    let e = solved("\
unit mm
use std
use std.place (coords)
in std.top {
  repeat d in [2mm, 3mm, 2mm + 3mm] {
    q := point
    q coords(du: d, dv: 1mm) std.top
  }
  o := point hint((5, 8))
  fix(x == 5) o
  q[2] distance(10mm) o
}
");
    near(&copies(&e, "q"), &[(2.0, 1.0), (3.0, 1.0), (5.0, 1.0)]);
    near(&[at(&e, "o")], &[(5.0, 11.0)]);
}

/// A list is a group, so it is a member of one, imported from a module and read inside a
/// definition's body as a group's numbers are; an item that is a group of numbers is handed on
/// as one, and a `cycle` over a list closes as any `cycle` does.
#[test]
fn a_list_is_reached_as_a_group_is() {
    const BOARD: &str = "\
unit mm
board_dims := {
  t: 1.6mm,
  holes: [(1mm, 2mm), (3mm, 4mm), {x: 5mm, y: 6mm}],
}
pins := [1mm, 2mm, 3mm]
";
    let src = "\
unit mm
use std
use std.place (coords)
use lib.board (board_dims, pins)
component Pins(at: Length) {
  in std.top {
    repeat x in pins as i {
      p := point
      p coords(du: x + at, dv: i * 1mm) std.top
    }
  }
}
component Hole(spot: group) {
  in std.top {
    c := point
    c coords(du: spot.x, dv: spot.y) std.top
  }
}
local := [{x: 1mm, y: 1mm, d: 2mm}, {x: 2mm, y: 3mm, d: board_dims.t}]
in std.top {
  repeat h in board_dims.holes {
    hole := Hole(h)
  }
  cycle g in local {
    m := point
    m coords(du: g.x, dv: g.y + g.d) std.top
    l := line(m, next.m)
  }
}
pp := Pins(at: 10mm)
";
    let (mut prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let linked = link(&mut prog, &mut |name| match name {
        "lib.board" => Some(BOARD.to_string()),
        _ => gcs_core::library::resolve(name),
    });
    assert!(linked.is_empty(), "{linked:?}");
    let mut e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    near(&copies(&e, "hole.c"), &[(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]);
    near(&copies(&e, "m"), &[(1.0, 3.0), (2.0, 4.6)]);
    let mut pins: Vec<(f64, f64)> = e.map.names.values().flatten()
        .filter(|n| n.starts_with("pp.") && n.ends_with(".p"))
        .map(|n| at(&e, n)).collect();
    pins.sort_by(|a, b| a.0.total_cmp(&b.0));
    near(&pins, &[(11.0, 0.0), (12.0, 1.0), (13.0, 2.0)]);
}

/// A list is printed as it is written, its vectors in their brackets.
#[test]
fn a_list_prints_as_written() {
    let (mut prog, errs) = parse("holes := [(1mm, 2mm), (3mm, 4mm)]\nd := {r: [2mm, 3mm]}\n");
    assert!(errs.is_empty(), "{errs:?}");
    assert!(prog.stmts().all(|s| matches!(s.kind, StmtKind::Group(_))));
    let out = render_flat(&mut prog).expect("prints").to_string();
    assert!(out.contains("holes := [(1mm, 2mm), (3mm, 4mm)]"), "{out}");
    assert!(out.contains("d := {r: [2mm, 3mm]}"), "{out}");
}

/// The item's name is coloured as a declaration, as an edge's is, over a list written in place.
#[test]
fn the_item_is_coloured_as_a_declaration() {
    let src = "repeat h in [(1mm, 2mm), w] as i { }\n";
    let runs = highlight(src);
    let tint_at = |at: usize| runs.iter().find(|(_, s)| s.lo as usize == at).map(|(t, _)| *t);
    assert_eq!(tint_at(0), Some(Tint::Word));
    assert_eq!(tint_at(7), Some(Tint::Def), "{runs:?}");
    assert_eq!(tint_at(src.find(" i ").unwrap() + 1), Some(Tint::Def), "the index too: {runs:?}");
}

/// What is not a list, and an item that is no value, are said where they are written.
#[test]
fn what_is_not_a_list_is_refused() {
    let doc = |body: &str| format!("unit mm\ndims := {{a: 1mm, b: 2mm}}\nw := 3mm\n{body}\n");
    refused(&doc("repeat h in [1mm, nope] { p := point }"), "E103", "`nope`");
    refused(&doc("repeat h in dims { p := point }"), "E103", "a face nor a list");
    refused(&doc("repeat h in [1mm] { h := point }"), "E001", "`h` is declared twice");
    refused(&doc("repeat w in [1mm] { p := point }"), "E002", "may not shadow");
    let (_, errs) = parse("repeat h in [1mm, a: 2mm] { p := point }\n");
    assert!(errs.iter().any(|e| e.message.contains("in order and unnamed")), "{errs:?}");
}
