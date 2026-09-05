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

/// **A face closes itself** (§6.8, issue #49 item 1): the same region, written with the corners
/// it turns at instead of the construction lines between them.
#[test]
fn a_face_written_with_its_corners_is_the_face_written_with_its_edges() {
    // `a` is a corner the loop goes straight to and straight on from; `bc` and `cd` are edges
    // the drawing already has, and `-> close` seals the run back to `a`.  Two straight runs are
    // minted — `a`→`b` and `d`→`a` — which is `ab` and `da` by another name.
    let long = read(&format!("{RECT}solid block(sec, depth: 30mm)\n"));
    let short = read(&format!(
        "{RECT}face brief(a, bc, cd, -> close)\nsolid block(brief, depth: 30mm)\n"
    ));
    assert!((volume(&short, "block") - volume(&long, "block")).abs() < 1e-9);
    assert!((volume(&short, "block") - 72000.0).abs() < 1e-6);
    // the two runs are lines of the sketch like any other, and are the only new ones
    assert_eq!(short.sketch.lines.len(), long.sketch.lines.len() + 2);
    // **and nothing draws them.**  A closing run carries no design: it exists so that a region
    // has a boundary, which is what the thirty-two hand-written `class gone` lines were saying.
    let hidden = short
        .sketch
        .lines
        .iter()
        .filter(|l| !gcs_core::style::resolve(&short.sketch.sheet, &l.class).shown())
        .count();
    assert_eq!(hidden, 2, "a minted run is not on the sheet");
    // a face of the solid is named for each, so the report can still spell every face
    let p = gcs_core::report::positions(&short.sketch, &short.map);
    let has = |n: &str| p.iter().any(|(k, _)| k == n);
    assert!(has("block.close0.area") && has("block.close1.area"), "and each run names a face");
    assert!(has("block.bc.area"), "beside the edges the source wrote");
    // and the marker survives a print: the source is the document, so what is written must be
    // what comes back
    let (mut prog, _) = gcs_core::syntax::parse("face brief(a, bc, cd, -> close)\n");
    let text = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    assert_eq!(text.split_whitespace().collect::<Vec<_>>().join(" "),
        "face brief(a, bc, cd, -> close)");
}

/// A loop of nothing but corners: the rectangle again, with no line drawn at all.
#[test]
fn a_face_may_be_written_as_its_corners_alone() {
    let e = read(&format!("{RECT}face quad(a, b, c, d, -> close)\nsolid slab(quad, depth: 2mm)\n"));
    assert!((volume(&e, "slab") - 4800.0).abs() < 1e-6, "{}", volume(&e, "slab"));
}

#[test]
fn closing_lines_do_not_take_an_existing_edges_name() {
    let e = read(&format!(
        "{RECT}line close0(b, c)\nface brief(a, close0, cd, -> close)\n\
         solid block(brief, depth: 2mm)\n"
    ));
    let positions = gcs_core::report::positions(&e.sketch, &e.map);
    for (name, want) in [("close0", 80.0), ("close1", 120.0), ("close2", 80.0)] {
        let key = format!("block.{name}.area");
        let got = positions.iter().find(|(n, _)| *n == key).unwrap().1;
        assert!((got - want).abs() < 1e-9, "{key}: expected {want}, got {got}");
    }
}

/// `-> close` on a loop that already meets says something true, and mints nothing.
#[test]
fn a_loop_that_already_meets_may_still_say_it_closes() {
    let plain = read(&format!("{RECT}solid block(sec, depth: 30mm)\n"));
    let said = read(&format!(
        "{RECT}face same(ab, bc, cd, da, -> close)\nsolid block(same, depth: 30mm)\n"
    ));
    assert_eq!(said.sketch.lines.len(), plain.sketch.lines.len(), "nothing to mint");
    assert!((volume(&said, "block") - volume(&plain, "block")).abs() < 1e-9);
    let circle = read(&format!("{RECT}{HOLE}face same(hole, -> close)\n"));
    assert_eq!(circle.sketch.lines.len(), plain.sketch.lines.len());
    assert_eq!(circle.sketch.faces.last().unwrap().edges.len(), 1);
}

#[test]
fn a_mixed_faces_seed_writeback_changes_only_the_points() {
    use gcs_core::edit::{self, Kind};

    let src = format!("{RECT}face brief(a, bc, cd, -> close)\nsolid block(brief, depth: 2mm)\n");
    let mut e = read(&src);
    let mut sk = std::mem::take(&mut e.sketch);
    let unchanged = edit::commit_seeds(&e, &sk, &e.program);
    assert_eq!(unchanged.kind, Kind::None);
    assert_eq!(unchanged.text, src);

    let a = e.map.ent_named("a").unwrap();
    let [x, _] = sk.point_params(a.i());
    sk.params[x as usize].value = -5.0;
    let want = src.replace("point a hint(x: 0, y: 0)", "point a hint(x: -5, y: 0)");
    let moved = edit::commit_seeds(&e, &sk, &e.program);
    assert_eq!(moved.kind, Kind::Numeric);
    assert_eq!(moved.text, want);
    let synced = edit::reconcile(&mut e, &sk);
    assert!(synced.refused.is_none(), "{:?}", synced.refused);
    assert_eq!(synced.text, want);
    let back = read(&synced.text);
    assert_eq!(back.sketch.lines.len(), sk.lines.len());
    assert_eq!(back.sketch.point_xy(a.i()), (-5.0, 0.0));
}

#[test]
fn closing_lines_stay_implicit_across_reconciliation_and_reload() {
    use gcs_core::edit::{self, Kind};

    for src in [
        format!("{RECT}face quad(a, b, c, d, -> close) class region\n"),
        "component Patch() {\npoint a hint(x: 0, y: 0)\npoint b hint(x: 60, y: 0)\n\
         point c hint(x: 0, y: 40)\nface tri(a, b, c, -> close) class region\n}\n\
         part: Patch()\n".to_string(),
    ] {
        let initial = read(&src).sketch.lines.len();
        let mut text = src.clone();
        for _ in 0..3 {
            let mut e = read(&text);
            let mut sk = std::mem::take(&mut e.sketch);
            assert_eq!(sk.lines.len(), initial);
            let unchanged = edit::reconcile(&mut e, &sk);
            assert_eq!(unchanged.kind, Kind::None);
            assert_eq!(unchanged.text, src);
            text = unchanged.text;

            // Accounting for generated lines must still let a newly drawn line get a
            // declaration, and must not copy the children's class onto the face.
            sk.line(0, 1);
            let added = edit::reconcile(&mut e, &sk);
            assert!(added.refused.is_none(), "{:?}", added.refused);
            assert_eq!(added.names.len(), 1);
            assert_eq!(read(&added.text).sketch.lines.len(), initial + 1);
            assert!(!added.text.contains("class closure"), "{}", added.text);
            assert_eq!(edit::reconcile(&mut e, &sk).kind, Kind::None);
        }
    }
}

#[test]
fn a_face_must_leave_each_edge_where_the_next_item_starts() {
    for walk in ["a, ab, a, c, -> close", "ab, a, c, a", "c, a, ab, a, -> close"] {
        let src = format!("{RECT}face bad({walk})\n");
        refused(&src, Code::E080, "along the walk");
        let (p, _) = parse(&src);
        let e = elaborate(&p);
        assert_eq!(e.sketch.lines.len(), 4, "a refused face leaves no closing lines");
        assert_eq!(e.sketch.faces.len(), 1, "only the original rectangle remains");
    }
    // A failure after minting some lines must also leave nothing for reconciliation to
    // interpret as newly drawn geometry.
    let (p, _) = parse(&format!("{RECT}face bad(a, c, d)\n"));
    let e = elaborate(&p);
    assert!(!e.ok());
    assert_eq!(e.sketch.lines.len(), 4);
}

#[test]
fn an_arc_and_its_chord_share_both_ends_and_still_form_a_loop() {
    let src = "unit mm\npoint o hint(x: 0, y: 0)\npoint a hint(x: -5, y: 0)\n\
               point b hint(x: 5, y: 0)\narc rim(center: o, start: a, end: b) hint(r: 5)\n\
               line chord(a, b)\n";
    let mut volumes = Vec::new();
    for walk in ["rim, chord", "chord, rim", "a, rim, b, -> close", "b, rim, a, -> close"] {
        let e = read(&format!("{src}face half({walk})\nsolid slab(half, depth: 2mm)\n"));
        let v = volume(&e, "slab");
        assert!((v - 25.0 * std::f64::consts::PI).abs() < 0.2, "{walk}: {v}");
        volumes.push(v);
    }
    assert!(volumes.iter().all(|v| (v - volumes[0]).abs() < 1e-9));
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
    // a face of something that is neither an edge nor a corner
    refused(&format!("{RECT}face bad(sec)\n"), Code::E080, "bounded by lines");
    // -- and the shorthand does not swallow a mistake (issue #49, item 1) --------------------
    // a loop whose last item does not come back to its first says so, or says `-> close`
    refused(&format!("{RECT}face bad(a, ab, bc)\n"), Code::E080, "`-> close`");
    // an edge between two gaps has two readings and no statement choosing one
    refused(&format!("{RECT}face bad(a, bc, d, -> close)\n"), Code::E080, "meets neither");
    // a straight loop between two corners is a line drawn twice
    refused(&format!("{RECT}face bad(a, b, -> close)\n"), Code::E080, "three corners");
    // and one item is a loop only when it is a circle
    refused(&format!("{RECT}face bad(ab)\n"), Code::E080, "not a loop by itself");
    // `-> close` seals a loop, and only a face is one
    refused(&format!("{RECT}line bad(a, -> close)\n"), Code::E100, "not a loop");
    refused(&format!("{RECT}face bad(a, -> close, ab)\n"), Code::E100, "last thing in the list");
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
