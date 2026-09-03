//! **Where a part stands is what it bears against** (Solvent §6.10) — issue #48, item 8.
//!
//! The V-twin's crankshaft stack was `zA = fwA + D / 2`, `tdisc = zA - rw / 2 - wsh - zdisc`, and
//! a side view that drew each part as a `Box` at those offsets — three files keeping one chain of
//! subtractions in step by hand.  What the drawing *meant* was "cylinder B's face against the
//! plate's front; a washer between the disc and rod A".  That is what these say, and the numbers
//! come out the same.

use gcs_core::program::{elaborate, Code, Elaborated};
use gcs_core::syntax::parse;

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

fn refused(src: &str, code: Code, needle: &str) {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    let saw: Vec<String> =
        e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect();
    assert!(
        e.diags.iter().any(|d| d.code == code && d.message.contains(needle)),
        "expected {} `{needle}`\n{src}\n{saw:#?}",
        code.as_str()
    );
}

/// A square `w` on a side at the origin, drawn in `plane`, as face `f{tag}` and solid `{tag}`,
/// standing between `lo` and `hi` along that plane's normal.
fn part(tag: &str, plane: &str, w: f64, lo: &str, hi: &str) -> String {
    format!(
        "point a{tag} hint(x: 0, y: 0) in {plane}\npoint b{tag} hint(x: {w}, y: 0) in {plane}\n\
         point c{tag} hint(x: {w}, y: {w}) in {plane}\npoint d{tag} hint(x: 0, y: {w}) in {plane}\n\
         line p{tag}(a{tag}, b{tag}) -> line q{tag}(b{tag}, c{tag}) -> \
         line r{tag}(c{tag}, d{tag}) -> line s{tag}(d{tag}, a{tag}) -> close\n\
         horizontal p{tag}\nvertical q{tag}\nhorizontal r{tag}\nvertical s{tag}\n\
         a{tag} distance({w}) b{tag}\na{tag} distance({w}) d{tag}\na{tag} coincident o\n\
         face f{tag}(p{tag}, q{tag}, r{tag}, s{tag})\n\
         solid {tag}(f{tag}, from: {lo}, to: {hi})\n"
    )
}

/// The page, and one plane parallel to it that a mate must place.
const HEAD: &str = "\
unit mm
point o hint(x: 0, y: 0)
point qq hint(x: 40, y: 0)
ground o
horizontal line ref(o, qq)
o distance(40) qq
plane front(origin: o, toward: qq)
plane back(origin: o, toward: qq, from: front)
";

/// How far a solid reaches **along the page's own normal**, as the report gives it.
///
/// The page's normal is −y, so a position along it is a negative y: the report says where a
/// thing is and this says how far along it stands, which is the number a stack is written in.
fn span(e: &Elaborated, name: &str) -> (f64, f64) {
    let p = gcs_core::report::positions(&e.sketch, &e.map);
    let at = |k: &str| {
        p.iter()
            .find(|(n, _)| *n == format!("{name}.bounds.{k}"))
            .unwrap_or_else(|| panic!("no `{name}.bounds.{k}`"))
            .1
    };
    // the page's normal is −y, so a solid swept along it stands at −y
    (-at("y1"), -at("y0"))
}

#[test]
fn a_mate_places_a_plane_and_the_offset_is_a_consequence() {
    let src = format!(
        "{HEAD}{}{}back_part.far against front_part.near\n",
        part("front_part", "front", 30.0, "-6mm", "0mm"),
        part("back_part", "back", 20.0, "-20mm", "12mm")
    );
    let e = read(&src);
    // the plate stands 6 deep off the page; the part bearing on its near face therefore stands
    // from there, and its far face is exactly where the plate's near face is
    let (lo, hi) = span(&e, "back_part");
    assert!(lo.abs() < 1e-9, "its far face is on the plate's near face: {lo}");
    assert!((hi - 32.0).abs() < 1e-9, "and it reaches its own full depth: {hi}");
    // **the number nobody wrote**: the placed plane stands 20 along the normal, which is the
    // `zA = fwA + D / 2` the V-twin kept in three files
    let b = e.sketch.planes[1].basis;
    assert!((b.along_normal() - 20.0).abs() < 1e-9, "placed at {}", b.along_normal());
    assert_eq!(e.sketch.planes[0].basis.along_normal(), 0.0, "and the datum stands where it did");
}

#[test]
fn a_stack_of_three_is_worked_out_in_order() {
    // washer, then part: each stands on the last, and the walk finds the order the way
    // `expr::evaluate` finds a dimension's
    let src = format!(
        "{HEAD}plane mid(origin: o, toward: qq, from: front)\n{}{}{}\
         mid_part.far against front_part.near\nback_part.far against mid_part.near\n",
        part("front_part", "front", 30.0, "-6mm", "0mm"),
        part("mid_part", "mid", 25.0, "-2mm", "0mm"),
        part("back_part", "back", 20.0, "-10mm", "0mm")
    );
    let e = read(&src);
    let (lo, hi) = span(&e, "back_part");
    // the washer is 2 thick, so the part behind it starts 2 off the plate and reaches 12
    assert!((lo - 2.0).abs() < 1e-9, "two off the plate: {lo}");
    assert!((hi - 12.0).abs() < 1e-9, "and ten deep from there: {hi}");
}

#[test]
fn a_document_that_says_nothing_or_two_things_about_where_a_part_stands_is_refused() {
    // a plane nothing places
    refused(
        &format!("{HEAD}{}", part("front_part", "front", 30.0, "-6mm", "0mm")),
        Code::E083,
        "nothing places",
    );
    let two = format!(
        "{HEAD}{}{}",
        part("front_part", "front", 30.0, "-6mm", "0mm"),
        part("back_part", "back", 20.0, "-10mm", "0mm")
    );
    // two mates on one plane
    refused(
        &format!("{two}back_part.far against front_part.near\nback_part.near against front_part.far\n"),
        Code::E083,
        "placed twice",
    );
    // a mate onto a plane the document already placed
    refused(
        &format!("{two}front_part.far against back_part.near\nback_part.far against front_part.near\n"),
        Code::E083,
        "already says where it stands",
    );
    // faces that look the same way cannot be in contact
    refused(&format!("{two}back_part.near against front_part.near\n"), Code::E083, "look at each other");
    // a face a stack cannot bear on
    refused(&format!("{two}back_part.pback_part against front_part.near\n"), Code::E082, "no flat face");
}

#[test]
fn a_stack_that_stands_on_itself_is_refused() {
    let src = format!(
        "{HEAD}plane mid(origin: o, toward: qq, from: front)\n{}{}\
         mid_part.far against back_part.near\nback_part.far against mid_part.near\n",
        part("mid_part", "mid", 25.0, "-2mm", "0mm"),
        part("back_part", "back", 20.0, "-10mm", "0mm")
    );
    refused(&src, Code::E041, "stands on what stands on it");
}

#[test]
fn a_feature_carries_its_own_rule() {
    // Issue #48, item 5: the O-ring groove sized by hand and got wrong the first time.  The rule
    // — a moving seal wants 10–20% squeeze on the ring's section, and the groove is a third
    // wider than it — is stated in `hardware` and cut by `hardware.Groove`, so a design says
    // *a groove for a #014* and the arithmetic is the library's.
    //
    // **A component contributes a `through` to a body it was handed**, which is the body rule
    // being a set and not a sequence (§6.9): the feature owns the void it cuts.
    let src = "\
unit mm
use std
use hardware
point fo
point fq hint(x: -10, y: 0)
plane f(origin: fo, toward: fq)
point o hint(x: 0, y: 0) in f
point up hint(x: 0, y: 40) in f
point side hint(x: -10, y: 0) in f
ground o
line ax(o, up)
line ac(o, side)
vertical ax
horizontal ac
o distance(40) up
o distance(10) side
point p0 hint(x: 0, y: 0) in f
point p1 hint(x: 8, y: 0) in f
point p2 hint(x: 8, y: 20) in f
point p3 hint(x: 0, y: 20) in f
line e0(p0, p1) -> line e1(p1, p2) -> line e2(p2, p3) -> line e3(p3, p0) -> close
p0 coincident o
horizontal e0
vertical e1
p0 distance(8) p1
p1 distance(20) p2
horizontal e2
face sec(e0, e1, e2, e3)
solid blank(sec, about: ax)
solid pis(blank)
g: Groove(body: pis, f: f, o: o, ax: ax, ac: ac, dir: 90deg, r: 8mm, z: 15mm, cs: oring014_cs)
";
    let (prog, errs, linked) = gcs_core::library::parse_linked(src);
    assert!(errs.is_empty() && linked.is_empty(), "{errs:?} {linked:?}");
    let e = elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}",
        e.errors().map(|d| d.message.clone()).collect::<Vec<_>>()
    );
    let p = gcs_core::report::positions(&e.sketch, &e.map);
    let at = |n: &str| p.iter().find(|(k, _)| k == n).map(|(_, v)| *v);
    // the groove's bottom is the bore less twice 88% of the ring's section: 8 − 0.88 × 1.78
    let bottom = at("g.e0.p1.x").expect("the groove's inner corner");
    assert!(
        (bottom.abs() - (8.0 - 0.88 * 1.78)).abs() < 1e-6,
        "the rule, not a number somebody rounded: {bottom}"
    );
    // and it is `oring_groove_w` sections wide
    let (a, b) = (
        at("g.e1.p1.y").expect("the groove's near wall"),
        at("g.e1.p2.y").expect("its far wall"),
    );
    assert!(((a - b).abs() - 1.35 * 1.78).abs() < 1e-6, "a third wider than the section: {}", a - b);
    // the void it cuts belongs to the body it was handed
    let pis = e.map.ent_named("pis").expect("the piston");
    let vol = gcs_core::mesh::volume(&e.sketch.solid_boundary(pis.i(), gcs_core::solid::REPORT_UNIT));
    let blank = e.map.ent_named("blank").expect("the blank");
    let full = gcs_core::mesh::volume(&e.sketch.solid_boundary(blank.i(), gcs_core::solid::REPORT_UNIT));
    assert!(vol < full - 1.0, "the groove came out of it: {vol} against {full}");
}
