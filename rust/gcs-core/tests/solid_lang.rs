//! **Faces and solids as the document writes them** (Solvent §6.8, §6.9).
//!
//! `tests/solid.rs` holds the kernel against arithmetic; this holds the *language* against the
//! kernel — that what a person writes reaches it, that the body rule is order-free, and that
//! every way of writing it wrong is refused where it is written.

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

/// A document refused, with the code and the words it is refused in — a message is part of the
/// contract here, since what these check is that a mistake is reported *where it was made*.
fn refused(src: &str, code: Code, needle: &str) {
    let (prog, errs) = parse(src);
    // a parse error is E100 by the same rule `report` sorts one under, so both halves of the
    // front end answer in the same vocabulary
    let mut saw: Vec<String> = errs.iter().map(|e| format!("E100: {}", e.message)).collect();
    let mut hit =
        code == Code::E100 && errs.iter().any(|d| d.message.contains(needle));
    if !hit && errs.is_empty() {
        let e = elaborate(&prog);
        saw.extend(e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)));
        hit = e.diags.iter().any(|d| d.code == code && d.message.contains(needle));
    }
    assert!(hit, "expected {} `{needle}`\n{src}\n{saw:#?}", code.as_str());
}

/// A 60 × 40 rectangle on the page, fully dimensioned and grounded, as `sec`.
const RECT: &str = "\
unit mm
point a hint(x: 0, y: 0)
point b hint(x: 60, y: 0)
point c hint(x: 60, y: 40)
point d hint(x: 0, y: 40)
line ab(a, b) -> line bc(b, c) -> line cd(c, d) -> line da(d, a) -> close
horizontal ab
vertical bc
a distance(60) b
a distance(40) d
ground a
face sec(ab, bc, cd, da)
";

/// A circle of radius 5 at the middle of that rectangle, as `hole_f`.
const HOLE: &str = "\
point o hint(x: 30, y: 20)
a distance(30, along: x) o
a distance(20, along: y) o
circle hole(center: o) hint(r: 5)
radius(5) hole
face hole_f(hole)
";

fn volume(e: &gcs_core::program::Elaborated, name: &str) -> f64 {
    let key = format!("{name}.volume");
    gcs_core::report::positions(&e.sketch, &e.map)
        .into_iter()
        .find(|(n, _)| *n == key)
        .unwrap_or_else(|| panic!("no `{key}` in the report"))
        .1
}

#[test]
fn a_part_written_once_reports_itself() {
    let e = read(&format!("{RECT}solid block(sec, depth: 30mm)\n"));
    assert!((volume(&e, "block") - 72000.0).abs() < 1e-6);
    // and the report carries where its faces are, which is the only picture of an object no
    // view of the sheet shows whole (issue #48, item 3)
    let p = gcs_core::report::positions(&e.sketch, &e.map);
    let has = |n: &str| p.iter().any(|(k, _)| k == n);
    assert!(has("block.near.area") && has("block.ab.area"), "its faces, by the names it wrote");
    assert!(has("block.bounds.z1"), "and the box it stands in");
}

#[test]
fn an_extent_is_an_expression_and_never_an_unknown() {
    let e = read(&format!("param t = 30mm\n{RECT}solid block(sec, depth: t)\n"));
    assert!((volume(&e, "block") - 72000.0).abs() < 1e-6);
    // P3's other half: nothing about the solid is a parameter, so no solve can move it
    let before = e.sketch.params.len();
    let plain = read(RECT);
    assert_eq!(before, plain.sketch.params.len(), "a solid allocates no unknown");
}

#[test]
fn the_body_rule_does_not_care_what_order_it_was_written_in() {
    // P2 at the language: `bore through body` says what `body` *is*, wherever it stands
    let after = format!(
        "{RECT}{HOLE}solid stock(sec, depth: 30mm)\nsolid bore(hole_f, depth: 30mm)\n\
         solid body(stock)\nbore through body\n"
    );
    let before = format!(
        "{RECT}{HOLE}solid stock(sec, depth: 30mm)\nsolid bore(hole_f, depth: 30mm)\n\
         bore through body\nsolid body(stock)\n"
    );
    let (x, y) = (volume(&read(&after), "body"), volume(&read(&before), "body"));
    assert!((x - y).abs() < 1e-9, "one body, whichever order: {x} vs {y}");
    assert!(x < 72000.0 && x > 69000.0, "and the bore came out of it: {x}");
}

#[test]
fn a_boss_is_written_with_the_word_the_kinds_of_its_operands_choose() {
    // `on` is already five constraints; between two solids it is the body rule (§9.2)
    let src = format!(
        "{RECT}point e hint(x: 20, y: 10)\npoint f hint(x: 40, y: 10)\n\
         point g hint(x: 40, y: 30)\npoint h hint(x: 20, y: 30)\n\
         line ef(e, f) -> line fg(f, g) -> line gh(g, h) -> line he(h, e) -> close\n\
         a distance(20, along: x) e\na distance(10, along: y) e\n\
         a distance(40, along: x) g\na distance(30, along: y) g\n\
         horizontal ef\nvertical fg\nhorizontal gh\nvertical he\n\
         face boss_f(ef, fg, gh, he)\n\
         solid block(sec, depth: 30mm)\nsolid boss(boss_f, from: 0mm, to: 10mm)\n\
         solid body(block)\nboss on body\n"
    );
    let e = read(&src);
    let want = 60.0 * 40.0 * 30.0 + 20.0 * 20.0 * 10.0;
    assert!((volume(&e, "body") - want).abs() < 1e-6, "a boss adds: {}", volume(&e, "body"));
}

#[test]
fn a_revolution_turns_about_a_line_in_its_own_plane() {
    let src = "\
unit mm
point p0 hint(x: 10, y: 0)
point p1 hint(x: 14, y: 0)
point p2 hint(x: 14, y: 6)
point p3 hint(x: 10, y: 6)
line e0(p0, p1) -> line e1(p1, p2) -> line e2(p2, p3) -> line e3(p3, p0) -> close
point q0 hint(x: 0, y: 0)
point q1 hint(x: 0, y: 10)
line ax(q0, q1)
ground q0
vertical ax
horizontal e0
vertical e1
p0 distance(10, along: x) q0
p0 distance(0, along: y) q0
p0 distance(4) p1
p1 distance(6) p2
horizontal e2
vertical e3
face sec(e0, e1, e2, e3)
solid ring(sec, about: ax)
";
    let e = read(src);
    let want = std::f64::consts::TAU * 12.0 * 24.0;
    let got = volume(&e, "ring");
    assert!((got - want).abs() < 2e-3 * want, "Pappus from the source: want ≈ {want}, got {got}");
}

#[test]
fn what_is_written_wrong_is_refused_where_it_is_written() {
    // a loop that does not close
    refused(
        &format!("{RECT}point z hint(x: 90, y: 90)\nline zz(z, a)\nface bad(ab, zz, cd, da)\n"),
        Code::E080,
        "share no point",
    );
    // a circle standing in a loop rather than being one
    refused(&format!("{RECT}{HOLE}face bad(ab, hole)\n"), Code::E080, "a whole loop");
    // a face of something that is not an edge
    refused(&format!("{RECT}face bad(a, b)\n"), Code::E080, "bounded by lines");
    // a swept solid over something that is not a face
    refused(&format!("{RECT}solid bad(ab, depth: 3mm)\n"), Code::E080, "written over a face");
    // a body made of what is not a solid
    refused(&format!("{RECT}solid bad(sec)\n"), Code::E080, "made of solids");
    // a prism swept nowhere
    refused(
        &format!("{RECT}solid bad(sec, from: 0mm, to: 0mm)\n"),
        Code::E080,
        "swept nowhere",
    );
    // **a selector is a word, never a sign**: a negative sweep is refused at the value
    refused(
        &format!("{RECT}point q0\npoint q1 hint(y: 10)\nline ax(q0, q1)\n\
                  solid bad(sec, about: ax, sweep: -90deg)\n"),
        Code::E040,
        "which way it turns is `sense: cw`",
    );
    // a revolution about something that is not a line
    refused(&format!("{RECT}solid bad(sec, about: a)\n"), Code::E081, "turns about a line");
    // a body made of itself
    refused(
        &format!("{RECT}solid s(sec, depth: 3mm)\nsolid x(s)\nsolid y(x)\nx through y\ny through x\n"),
        Code::E041,
        "made of itself",
    );
    // a feature written into a solid that is a face swept, not a body
    refused(
        &format!("{RECT}{HOLE}solid s(sec, depth: 3mm)\nsolid h(hole_f, depth: 3mm)\nh through s\n"),
        Code::E080,
        "only a body takes features",
    );
    // a mixture of the two sweeps
    refused(
        &format!("{RECT}point q0\npoint q1 hint(y: 10)\nline ax(q0, q1)\n\
                  solid bad(sec, depth: 3mm, about: ax)\n"),
        Code::E100,
        "not both",
    );
}

#[test]
fn a_plane_may_be_stood_off_another_and_the_projector_rule_is_untouched() {
    // `from:` with neither clause is a plane *moved*, which is what a stack is written in;
    // the offset is along the normal, so the fold line two views share cannot see it (§6.7)
    let e = read("\
unit mm
point o
point q hint(x: 40)
plane front(origin: o, toward: q)
plane back(origin: o, toward: q, from: front, offset: 12mm)
");
    let b = |n: usize| e.sketch.planes[n].basis;
    assert_eq!(b(0).u, b(1).u, "parallel");
    assert_eq!(b(0).v, b(1).v);
    assert!((b(1).along_normal() - 12.0).abs() < 1e-9, "twelve along its own normal");
    assert_eq!(b(0).o, [0.0; 3], "and the view it came from stands where it did");
    // the two are parallel, so no `project` between them is possible — which is what
    // `constraints::validate` already refuses, and is why the row is unchanged
    assert!(gcs_core::plane::fold_line(&b(0), &b(1)).is_none());
}
