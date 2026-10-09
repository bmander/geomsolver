//! **The region under the cursor** (`overview::region`, #162 F0): the smallest loop of drawn
//! edges around a place, and the loops inside it that are its holes — so that a click inside a
//! profile can be written as a face.

use gcs_core::edit::{self, SolidSweep};
use gcs_core::overview::region::{region_at, written};
use gcs_core::program::{self, Elaborated};
use gcs_core::library;
use std::collections::BTreeSet;

fn build(src: &str) -> Elaborated {
    let (p, errors, linked) = library::parse_linked(src);
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}\n{src}");
    let e = program::elaborate(&p);
    assert!(e.ok(), "{:?}\n{src}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    e
}

/// A square of four named lines on the front, `n` its prefix, from (x, y) with side `s`.
fn square(n: &str, x: f64, y: f64, s: f64) -> String {
    format!("{n}a := point hint(({x}, {y}))\n{n}b := point hint(({}, {y}))\n\
        {n}c := point hint(({}, {}))\n{n}d := point hint(({x}, {}))\n\
        {n}1 := line({n}a, {n}b)\n{n}2 := line({n}b, {n}c)\n{n}3 := line({n}c, {n}d)\n\
        {n}4 := line({n}d, {n}a)\n", x + s, x + s, y + s, y + s)
}

fn front(body: &str) -> String {
    format!("unit mm\nuse std\nin std.front {{\n{body}}}\n")
}

/// What the region around `at` on the front is written as: the outer loop's names, and each
/// hole's — or the refusal.
fn names_at(e: &Elaborated, at: (f64, f64)) -> Result<Option<(Vec<String>, Vec<Vec<String>>)>, String> {
    let plane = e.map.ent_named("std.front").map(|p| p.i());
    let Some(r) = region_at(&e.sketch, plane, at, 0.05)? else { return Ok(None) };
    written(&e.map, &r).map(Some)
}

fn set(names: &[String]) -> BTreeSet<String> {
    names.iter().cloned().collect()
}

fn of(names: &[&str]) -> BTreeSet<String> {
    names.iter().map(|n| n.to_string()).collect()
}

/// The loop a region is written as, walked in order: each edge meets the next.  Held to the one
/// judge that matters, the face statement written over it.
fn writes_a_face(src: &str, outer: &[String], holes: &[Vec<String>]) -> Elaborated {
    let (p, _, _) = library::parse_linked(src);
    let out = edit::add_face(&p, outer, holes, Some("sec"));
    assert_eq!(out.refused, None, "{outer:?} {holes:?}");
    build(&out.text)
}

#[test]
fn a_square_is_its_four_edges_and_outside_it_nothing() {
    let src = front(&square("s", 0.0, 0.0, 10.0));
    let e = build(&src);
    let (outer, holes) = names_at(&e, (5.0, 5.0)).unwrap().expect("inside the square");
    assert_eq!(set(&outer), of(&["s1", "s2", "s3", "s4"]));
    assert!(holes.is_empty());
    writes_a_face(&src, &outer, &holes);
    assert_eq!(names_at(&e, (20.0, 5.0)).unwrap(), None);
}

#[test]
fn two_squares_sharing_an_edge_are_two_regions() {
    // a domino: the middle line is both halves' edge
    let src = front("a := point hint((0, 0))\nm := point hint((10, 0))\nb := point hint((20, 0))\n\
        c := point hint((20, 10))\nn := point hint((10, 10))\nd := point hint((0, 10))\n\
        am := line(a, m)\nmb := line(m, b)\nbc := line(b, c)\ncn := line(c, n)\nnd := line(n, d)\n\
        da := line(d, a)\nmn := line(m, n)\n");
    let e = build(&src);
    let (left, _) = names_at(&e, (5.0, 5.0)).unwrap().unwrap();
    assert_eq!(set(&left), of(&["am", "mn", "nd", "da"]));
    let (right, _) = names_at(&e, (15.0, 5.0)).unwrap().unwrap();
    assert_eq!(set(&right), of(&["mb", "bc", "cn", "mn"]));
    writes_a_face(&src, &left, &[]);
    writes_a_face(&src, &right, &[]);
}

#[test]
fn an_arc_and_its_chord_are_a_region() {
    let src = front("o := point hint((0, 0))\np := point hint((10, 0))\nq := point hint((-10, 0))\n\
        top := arc(o, p, q)\nchord := line(q, p)\n");
    let e = build(&src);
    let (outer, _) = names_at(&e, (0.0, 3.0)).unwrap().unwrap();
    assert_eq!(set(&outer), of(&["top", "chord"]));
    assert_eq!(names_at(&e, (0.0, -3.0)).unwrap(), None);
    writes_a_face(&src, &outer, &[]);
}

#[test]
fn loops_inside_are_holes_and_an_island_in_a_hole_is_not() {
    let src = front(&format!("{}{}{}o := point hint((40, 15))\nring := circle(center: o) hint(r: 5)\n",
        square("s", 0.0, 0.0, 60.0), square("h", 10.0, 10.0, 20.0), square("i", 15.0, 15.0, 10.0)));
    let e = build(&src);
    let (outer, holes) = names_at(&e, (5.0, 5.0)).unwrap().unwrap();
    assert_eq!(set(&outer), of(&["s1", "s2", "s3", "s4"]));
    let holes: BTreeSet<BTreeSet<String>> = holes.iter().map(|h| set(h)).collect();
    let expected: BTreeSet<BTreeSet<String>> =
        [of(&["h1", "h2", "h3", "h4"]), of(&["ring"])].into_iter().collect();
    assert_eq!(holes, expected, "the island inside the square hole is not the plate's hole");
    // between the hole and its island: the hole's square, holed by the island
    let (outer, holes) = names_at(&e, (12.0, 12.0)).unwrap().unwrap();
    assert_eq!(set(&outer), of(&["h1", "h2", "h3", "h4"]));
    assert_eq!(holes.iter().map(|h| set(h)).collect::<Vec<_>>(), vec![of(&["i1", "i2", "i3", "i4"])]);
    // and in the island, the island alone
    let (outer, holes) = names_at(&e, (20.0, 20.0)).unwrap().unwrap();
    assert_eq!(set(&outer), of(&["i1", "i2", "i3", "i4"]));
    assert!(holes.is_empty());
    // inside the circle: the circle
    let (outer, _) = names_at(&e, (40.0, 15.0)).unwrap().unwrap();
    assert_eq!(outer, vec!["ring"]);
}

#[test]
fn what_dangles_bounds_nothing() {
    let src = front(&format!("{}e := point hint((5, 5))\nf := point hint((7, 3))\n\
        spur := line(sa, e)\nloose := line(e, f)\ng := point hint((2, 8))\nk := point hint((3, 8))\n\
        apart := line(g, k)\n", square("s", 0.0, 0.0, 10.0)));
    let e = build(&src);
    let (outer, holes) = names_at(&e, (8.0, 8.0)).unwrap().unwrap();
    assert_eq!(set(&outer), of(&["s1", "s2", "s3", "s4"]));
    assert!(holes.is_empty(), "{holes:?}");
}

#[test]
fn a_line_joining_two_loops_is_no_face_and_says_so() {
    let src = front(&format!("{}{}{}bridge := line(lb, rd)\n", square("s", 0.0, 0.0, 60.0),
        square("l", 10.0, 10.0, 10.0), square("r", 30.0, 10.0, 10.0)));
    let e = build(&src);
    let err = names_at(&e, (5.0, 5.0)).unwrap_err();
    assert!(err.contains("twice"), "{err}");
    // inside one of the joined loops is still a region of its own
    let (outer, _) = names_at(&e, (15.0, 15.0)).unwrap().unwrap();
    assert_eq!(set(&outer), of(&["l1", "l2", "l3", "l4"]));
}

#[test]
fn a_rectangle_instance_is_named_through_its_instance() {
    let src = "unit mm\nuse std\ncomponent Rectangle(w: Length, h: Length) {\n  \
        distance(w) (l1 := line) -> perpendicular distance(h) (l2 := line) -> \
        perpendicular (l3 := line) -> perpendicular (l4 := line) -> close\n}\n\
        r0 := Rectangle(w: 30, h: 20) in std.front\n";
    let mut e = build(src);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let corners: Vec<(f64, f64)> = ["r0.l1", "r0.l2", "r0.l3", "r0.l4"].iter()
        .map(|n| e.sketch.point_xy(e.sketch.lines[e.map.ent_named(n).unwrap().i()].p1 as usize))
        .collect();
    let mid = (corners.iter().map(|c| c.0).sum::<f64>() / 4.0, corners.iter().map(|c| c.1).sum::<f64>() / 4.0);
    let (outer, _) = names_at(&e, mid).unwrap().unwrap();
    assert_eq!(set(&outer), of(&["r0.l1", "r0.l2", "r0.l3", "r0.l4"]));
}

#[test]
fn a_region_written_and_swept_is_the_solid_its_area_says() {
    // the end to end of F0: a click inside, the face written over what it found, swept
    let src = front(&format!("{}{}o := point hint((45, 20))\nring := circle(center: o) hint(r: 5)\n",
        square("s", 0.0, 0.0, 60.0), square("h", 10.0, 10.0, 20.0)));
    let e = build(&src);
    let (outer, holes) = names_at(&e, (5.0, 5.0)).unwrap().unwrap();
    let with_face = writes_a_face(&src, &outer, &holes);
    let out = edit::add_solid(&with_face.program, "sec", &SolidSweep { depth: Some("2".into()),
        ..Default::default() }, Some("plate"));
    assert_eq!(out.refused, None);
    let e = build(&out.text);
    let i = e.map.ent_named("plate").unwrap().i();
    let v = e.sketch.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::Report).unwrap().volume();
    let expected = 2.0 * (3600.0 - 400.0 - 25.0 * std::f64::consts::PI);
    assert!((v / expected - 1.0).abs() < 1e-3, "{v} {expected}");
}
