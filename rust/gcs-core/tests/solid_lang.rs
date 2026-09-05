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
fn inline_sections_have_the_same_geometry_and_reports_as_named_sections() {
    for (boundary, sweep) in [
        ("ab, bc, cd, da", "depth: 30mm"),
        ("a, bc, cd, -> close", "from: face, to: back"),
        ("hole", "depth: 30mm"),
    ] {
        let base = format!("{RECT}{HOLE}param face = -30mm\nparam back = 0mm\n");
        let named = read(&format!("{base}face section({boundary})\nsolid block(section, {sweep})\n"));
        let src = format!("{base}solid block(face({boundary}), {sweep})\n");
        let inline = read(&src);
        let report = |e: &Elaborated| gcs_core::report::positions(&e.sketch, &e.map)
            .into_iter().filter(|(n, _)| n.starts_with("block.")).collect::<Vec<_>>();
        assert_eq!(report(&inline), report(&named));
        assert_eq!(inline.sketch.params.len(), named.sketch.params.len());
        assert_eq!(inline.sketch.lines.len(), named.sketch.lines.len());
        let section = gcs_core::model::EntRef::face(inline.sketch.faces.len() - 1);
        assert!(inline.map.name_of(section).is_none(), "an inline section publishes no name");

        let (mut p, _) = parse(&src);
        let printed = gcs_core::syntax::render_flat(&mut p).unwrap().to_string();
        assert!(printed.contains(&format!("face({boundary})")), "{printed}");
        assert_eq!(report(&read(&printed)), report(&inline));
    }
}

#[test]
fn inline_sections_preserve_depth_validation_when_printed() {
    let src = format!("{RECT}solid block(face(ab, bc, cd, da), depth: 2mm - 7mm)\n");
    let (mut p, errors) = parse(&src);
    assert!(errors.is_empty(), "{errors:?}");
    let printed = gcs_core::syntax::render_flat(&mut p).unwrap().to_string();
    for text in [&src, &printed] {
        refused(text, Code::E080, "positive magnitude");
    }
}

#[test]
fn inline_sections_resolve_component_formals_and_repeated_instances() {
    let src = format!("{RECT}\n\
        component Slab(a: Point, b: Point, c: Point, d: Point, t: Length) {{\n\
          solid block(face(a, b, c, d, -> close), depth: t)\n\
        }}\n\
        first: Slab(a, b, c, d, t: 2mm)\n\
        second: Slab(a, b, c, d, t: 3mm)\n\
        repeat 2 as i {{\ncopy: Slab(a, b, c, d, t: (i + 1) * 1mm)\n}}\n");
    let e = read(&src);
    assert!((volume(&e, "first.block") - 4800.0).abs() < 1e-6);
    assert!((volume(&e, "second.block") - 7200.0).abs() < 1e-6);
    assert_eq!(e.sketch.faces.len(), 5);
    assert_eq!(e.sketch.solids.len(), 4);
    assert_eq!(e.sketch.lines.len(), 20);
}

#[test]
fn inline_sections_inherit_the_boundary_plane_and_coexist_with_forward_faces() {
    let src = "unit mm\npoint o\npoint q hint(x: 40)\n\
        plane front(origin: o, toward: q, u: (1, 0, 0), v: (0, 1, 0))\n\
        plane back(origin: o, toward: q, from: front, offset: 12mm)\n\
        in back {\npoint a hint(x: 0, y: 0)\npoint b hint(x: 60, y: 0)\n\
        point c hint(x: 60, y: 40)\npoint d hint(x: 0, y: 40)\n}\n\
        solid slab(face(a, b, c, d, -> close), from: 0mm, to: 2mm)\n\
        solid named(sec, from: 0mm, to: 2mm)\nface sec(a, b, c, d, -> close)\n";
    let e = read(src);
    assert!(e.sketch.faces.iter().all(|f| f.plane == Some(1)));
    let p = gcs_core::report::positions(&e.sketch, &e.map);
    assert!(p.iter().any(|(n, v)| n == "slab.bounds.z0" && (*v - 12.0).abs() < 1e-9));
    assert_eq!(volume(&e, "slab"), volume(&e, "named"));
    refused(&src.replace("point d hint(x: 0, y: 40)\n}", "}\npoint d hint(x: 0, y: 40)"),
        Code::E080, "one plane");
}

#[test]
fn invalid_inline_sections_report_the_loop_and_leave_no_geometry() {
    for (boundary, sweep, needle) in [
        ("a, b, -> close", "depth: 2mm", "three corners"),
        ("a, bc, d, -> close", "depth: 2mm", "meets neither"),
        ("a, b, c, d, -> close", "depth: 0mm", "positive magnitude"),
        ("a, b, c, d, -> close", "", "made of solids"),
        ("a, b, c, d, -> close", "sec, depth: 2mm", "over one face"),
    ] {
        let src = format!("{RECT}solid bad(face({boundary}), {sweep})\nsolid good(sec, depth: 2mm)\n");
        refused(&src, Code::E080, needle);
        let (p, _) = parse(&src);
        let e = elaborate(&p);
        assert_eq!(e.sketch.faces.len(), 1);
        assert_eq!(e.sketch.lines.len(), 4);
        assert!((volume(&e, "good") - 4800.0).abs() < 1e-6);
        if needle == "three corners" {
            let d = e.diags.iter().find(|d| d.message.contains(needle)).unwrap();
            assert_eq!(d.span.slice(&src), "(a, b, -> close)");
        }
    }
    refused(&format!("{RECT}solid bad(face(missing), depth: 2mm)\n"), Code::E101, "missing");
    refused("face bad(face(a, b, c, -> close))\n", Code::E100, "a solid's section");
    refused("solid bad(face(a, -> close, b), depth: 2mm)\n", Code::E100, "last thing");
}

#[test]
fn the_body_rule_does_not_care_what_order_it_was_written_in() {
    // P2 at the language: `bore cut body` says what `body` *is*, wherever it stands
    let after = format!(
        "{RECT}{HOLE}solid stock(sec, depth: 30mm)\nsolid bore(hole_f, depth: 30mm)\n\
         solid body(stock)\nbore cut body\n"
    );
    let before = format!(
        "{RECT}{HOLE}solid stock(sec, depth: 30mm)\nsolid bore(hole_f, depth: 30mm)\n\
         bore cut body\nsolid body(stock)\n"
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
    for sweep in ["", ", sweep: 90deg, sense: cw"] {
        let named = src.replace("about: ax)", &format!("about: ax{sweep})"));
        let inline = named.replace("face sec(e0, e1, e2, e3)\n", "")
            .replace("solid ring(sec,", "solid ring(face(e0, e1, e2, e3),");
        assert_eq!(volume(&read(&inline), "ring"), volume(&read(&named), "ring"));
        let (mut p, _) = parse(&inline);
        let printed = gcs_core::syntax::render_flat(&mut p).unwrap().to_string();
        assert_eq!(volume(&read(&printed), "ring"), volume(&read(&inline), "ring"));
    }
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
        format!("{RECT}solid slab(face(a, b, c, d, -> close) class region, depth: 2mm) class part\n"),
        format!(
            "unit mm\ncomponent Patch() {{\n{}\
             solid slab(face(a, bc, cd, -> close), depth: 2mm)\n}}\npart: Patch()\n",
            RECT.trim_start_matches("unit mm\n"),
        ),
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
        &format!("{RECT}solid s(sec, depth: 3mm)\nsolid x(s)\nsolid y(x)\nx cut y\ny cut x\n"),
        Code::E041,
        "made of itself",
    );
    // a feature written into a solid that is a face swept, not a body
    refused(
        &format!("{RECT}{HOLE}solid s(sec, depth: 3mm)\nsolid h(hole_f, depth: 3mm)\nh cut s\n"),
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

// Issue #49.3: naming the traversal must not change the drawing it traverses.
fn named_rect() -> String {
    RECT.replace("line ab(a, b)", "profile = line ab(a, b)")
        .replace("face sec(ab, bc, cd, da)\n", "")
}

#[test]
fn named_chains_sweep_the_same_geometry_and_keep_edge_names() {
    let old = read(&format!("{RECT}solid block(sec, depth: 8mm)\n"));
    let src = format!("{}solid block(profile, depth: 8mm)\n", named_rect());
    let new = read(&src);
    assert_eq!(volume(&new, "block"), 19200.0);
    assert_eq!(new.sketch.params.len(), old.sketch.params.len());
    assert_eq!(new.sketch.constraints.len(), old.sketch.constraints.len());
    assert_eq!(new.sketch.lines.len(), old.sketch.lines.len());
    assert_eq!(new.sketch.faces.len(), old.sketch.faces.len());
    assert_eq!(new.map.ent_named("ab"), old.map.ent_named("ab"));
    let report = |e: &Elaborated| gcs_core::report::positions(&e.sketch, &e.map)
        .into_iter().filter(|(n, _)| n.starts_with("block.")).collect::<Vec<_>>();
    assert_eq!(report(&new), report(&old));
    let written = gcs_core::edit::commit_seeds(&new, &new.sketch, &new.program);
    assert!(written.text.contains("profile = line ab"), "{}", written.text);
    assert_eq!(volume(&read(&written.text), "block"), volume(&new, "block"));
    let no_change = gcs_core::edit::remove(&new, &new.program, &new.sketch,
        &[new.map.ent_named("profile").unwrap()], &[]);
    assert!(no_change.refused.is_some(), "deleting a group must not strand its edge readers");
}

#[test]
fn named_chains_support_anonymous_links_and_constraint_words() {
    let e = read("profile = distance(10) line -> equal line -> equal line -> close\n\
                  solid block(profile, depth: 8)\n");
    assert_eq!(e.sketch.points.len(), 3);
    assert_eq!(e.sketch.lines.len(), 3);
    assert_eq!(e.sketch.faces.len(), 1);
    assert_eq!(e.sketch.user_constraints().len(), 3);
    assert!(e.map.ent_named("profile").is_some());
    let single = read("trail = line\n");
    assert_eq!(single.sketch.lines.len(), 1);
    assert_eq!(single.sketch.faces.len(), 0);
    let edit = gcs_core::edit::remove(&single, &single.program, &single.sketch,
        &[gcs_core::model::EntRef::line(0)], &[]);
    assert!(edit.refused.is_some());
}

#[test]
fn anonymous_chain_sides_have_stable_names_without_hiding_named_sides() {
    let src = "point a hint(x: 0, y: 0)\npoint b hint(x: 10, y: 0)\n\
        point c hint(x: 0, y: 10)\n\
        profile = line(a, b) -> line edge0(b, c) -> line(c, a) -> close\n\
        solid part(profile, depth: 8)\n";
    let old = read(src);
    let moved = read(&format!("// Moving source text must not rename surfaces.\n{src}"));
    let report = |e: &Elaborated| gcs_core::report::positions(&e.sketch, &e.map)
        .into_iter().filter(|(n, _)| n.starts_with("part.")).collect::<Vec<_>>();
    let values = report(&old);
    assert_eq!(values, report(&moved));
    for side in ["edge0", "edge1", "edge2"] {
        assert!(values.iter().any(|(n, _)| n == &format!("part.{side}.area")));
    }
    assert!(values.iter().all(|(n, _)| !n.contains('#')));
    let named_area = values.iter().find(|(n, _)| n == "part.edge0.area").unwrap().1;
    assert!((named_area - 200.0_f64.sqrt() * 8.0).abs() < 1e-8);
}

#[test]
fn named_chains_are_component_members_and_resolve_forward_and_through_formals() {
    let src = "unit mm\n\
        component Shape() {\n\
          point a hint(x: 0, y: 0)\npoint b hint(x: 10, y: 0)\n\
          point c hint(x: 10, y: 20)\npoint d hint(x: 0, y: 20)\n\
          profile = line ab(a, b) -> line bc(b, c) -> line cd(c, d) -> line da(d, a) -> close\n\
        }\n\
        component Slab(section: face, t: Length) {\nsolid body(section, depth: t)\n}\n\
        solid first(s.profile, depth: 2mm)\n\
        x: Slab(s.profile, t: 3mm)\n\
        repeat 2 as i {\ncopy: Shape()\nsolid prism(copy.profile, depth: (i + 1) * 1mm)\n}\n\
        solid selected(copy[1].profile, depth: 4mm)\n\
        s: Shape()\n";
    let e = read(src);
    assert_eq!(e.sketch.faces.len(), 3);
    assert_eq!(e.sketch.solids.len(), 5);
    assert_eq!(volume(&e, "first"), 400.0);
    assert_eq!(volume(&e, "x.body"), 600.0);
    assert_eq!(volume(&e, "selected"), 800.0);
    assert!(e.map.ent_named("s.ab").is_some());
}

#[test]
fn named_open_chains_can_be_closed_explicitly_but_cannot_be_swept_directly() {
    let src = named_rect().replace(" -> line da(d, a) -> close", "");
    let e = read(&src);
    assert_eq!(e.sketch.lines.len(), 3);
    assert_eq!(e.sketch.faces.len(), 0);
    refused(&format!("{src}solid bad(profile, depth: 8mm)\n"), Code::E080, "open chain");
    refused(&format!("{src}face bad(profile)\n"), Code::E080, "share no point");
    let e = read(&format!("{src}solid block(face(profile, -> close), depth: 8mm)\n"));
    assert_eq!(e.sketch.lines.len(), 4);
    assert_eq!(volume(&e, "block"), 19200.0);
    let e = read(&format!("{src}face sec(profile, a)\nsolid block(sec, depth: 8mm)\n"));
    assert_eq!(volume(&e, "block"), 19200.0);
}

#[test]
fn named_chains_inherit_planes_and_revolve_with_arcs() {
    let src = "unit mm\npoint o\npoint q hint(x: 10)\n\
        plane front(origin: o, toward: q, u: (1, 0, 0), v: (0, 1, 0))\n\
        plane back(origin: o, toward: q, from: front, offset: 12mm)\n\
        in back {\npoint a hint(x: 0, y: -5)\npoint b hint(x: 0, y: 5)\n\
          point c hint(x: 0, y: 0)\n\
          profile = arc rim(center: c, start: a, end: b) hint(r: 5) -> line axis(b, a) -> close\n\
          solid ball(profile, about: axis)\n}\n";
    let e = read(src);
    assert_eq!(e.sketch.faces[0].plane, Some(1));
    // Reports integrate the faceted surface; the analytic sphere is an independent check.
    let want = 4.0 / 3.0 * std::f64::consts::PI * 125.0;
    assert!((volume(&e, "ball") / want - 1.0).abs() < 0.002,
        "sphere volume: {} versus {want}", volume(&e, "ball"));
    let old = src.replace("profile = ", "")
        .replace("solid ball(profile,", "face sec(rim, axis)\nsolid ball(sec,");
    assert_eq!(volume(&e, "ball"), volume(&read(&old), "ball"));
    assert!(gcs_core::program::solid_diagnostics(&e.sketch, &e.map).is_empty());
}

#[test]
fn named_chains_validate_names_links_and_plane_membership() {
    refused("profile = circle\n", Code::E080, "lines and arcs");
    refused("profile = line equal line\n", Code::E100, "every pair");
    refused("component A() { profile = line -> }\na: A()\n",
        Code::E100, "must finish");
    refused("profile = line\npoint profile\n", Code::E001, "declared twice");
    refused("point profile\nprofile = line\n", Code::E001, "declared twice");
    refused("circle c\nprofile = c\n", Code::E080, "lines and arcs");
    refused("profile = missing\n", Code::E101, "missing");
    refused("point p\npoint q\n\
        profile = line a(p, q) -> profile.nope -> line b(q, p) -> close\n",
        Code::E080, "lines and arcs");
    refused(&format!("{}solid bad(profile.typo, depth: 8mm)\n", named_rect()),
        Code::E080, "not a member");
    let src = named_rect();
    let src = format!("plane v(origin: a, toward: b)\n{}", src)
        .replace("point d hint(x: 0, y: 40)", "point d hint(x: 0, y: 40) in v");
    refused(&src, Code::E080, "plane");
}

#[test]
fn named_chain_source_is_retained_and_its_binding_is_highlighted() {
    let src = "profile = line -> line -> line -> close\n";
    let (mut p, errors) = parse(src);
    assert!(errors.is_empty(), "{errors:?}");
    assert_eq!(gcs_core::syntax::render_flat(&mut p).unwrap_err().construct, "named chains");
    assert_eq!(p.text(), src);
    let colors = gcs_core::syntax::highlight(src);
    assert!(colors.iter().any(|(t, span)| *t == gcs_core::syntax::Tint::Def
        && &src[span.lo as usize..span.hi as usize] == "profile"));
}

#[test]
fn named_chain_syntax_errors_point_to_the_offending_joint() {
    for (src, message, joint) in [
        ("trail = line equal line\npoint unrelated\n", "every pair", "equal"),
        ("component A() { trail = line -> }\npoint unrelated\n", "must finish", "->"),
    ] {
        let (_, errors) = parse(src);
        let error = errors.iter().find(|e| e.message.contains(message)).unwrap();
        assert_eq!(&src[error.span.lo as usize..error.span.hi as usize], joint);
    }
}

#[test]
fn a_refused_named_loop_does_not_misnumber_later_sections() {
    let src = format!("broken = line -> line -> close\n{}\n\
        solid block(profile, depth: 8mm)\n", named_rect());
    let (p, errors) = parse(&src);
    assert!(errors.is_empty(), "{errors:?}");
    let e = elaborate(&p);
    assert!(!e.ok());
    assert_eq!(e.sketch.faces.len(), 1);
    assert_eq!(e.sketch.solids.len(), 1);
    assert_eq!(volume(&e, "block"), 19200.0);
}

#[test]
fn through_extent_spans_additions_but_only_cut_subtracts() {
    let src = format!("{RECT}{HOLE}\n\
        solid stock(sec, depth: 10mm)\n\
        solid boss(sec, from: 0mm, to: 5mm)\n\
        solid body(stock)\nboss on body\n\
        solid bore(hole_f, through: body)\n");
    let uncut = read(&src);
    assert_eq!(volume(&uncut, "body"), 36000.0);
    let cut = read(&format!("{src}bore cut body\n"));
    let explicit = read(&format!("{}bore cut body\n", src.replace("through: body", "from: -11mm, to: 6mm")));
    assert!((volume(&cut, "body") - volume(&explicit, "body")).abs() < 1e-6);
    let i = cut.map.ent_named("bore").unwrap().i();
    let csg = gcs_core::solid::resolve(&cut.sketch, i, gcs_core::solid::REPORT_UNIT);
    assert_eq!(csg.prims.len(), 1, "extent sources are not cutter geometry");
    let b = csg.bbox();
    assert!(b.lo[1] < -5.0 && b.hi[1] > 10.0);
    assert!(b.lo[0] >= 25.0 - 1e-6 && b.hi[0] <= 35.0 + 1e-6);
    let mut sk = cut.sketch.clone();
    let boss = cut.map.ent_named("boss").unwrap().i();
    let body = cut.map.ent_named("body").unwrap().i();
    let before = sk.evaluated_solid(body, gcs_core::solid::ApproximationPolicy::Report).unwrap();
    if let gcs_core::model::SolidDef::Prism { to, .. } = &mut sk.solids[boss].def {
        to.value = 20.0;
    }
    let after = sk.evaluated_solid(body, gcs_core::solid::ApproximationPolicy::Report).unwrap();
    assert!(after.volume() > before.volume(), "target edits invalidate the evaluated body");
    let b = gcs_core::solid::resolve(&sk, i, gcs_core::solid::REPORT_UNIT).bbox();
    assert!(b.lo[1] < -20.0, "the cutter follows target changes");
}

#[test]
fn through_extent_is_order_independent_and_ignores_other_cutters() {
    let declarations = [
        "solid stock(sec, depth: 10mm)",
        "solid body(stock)",
        "solid bore(hole_f, through: body)",
        "bore cut body",
        "solid huge(hole_f, from: -1000mm, to: 1000mm)",
        "huge cut body",
    ];
    let mut expected: Option<f64> = None;
    for reverse in [false, true] {
        let mut lines = declarations.to_vec();
        if reverse { lines.reverse(); }
        let src = format!("{RECT}{HOLE}{}\n", lines.join("\n"));
        let e = read(&src);
        let i = e.map.ent_named("bore").unwrap().i();
        let b = gcs_core::solid::resolve(&e.sketch, i, gcs_core::solid::REPORT_UNIT).bbox();
        assert!(b.lo[1] > -1.0 && b.hi[1] < 11.0, "cuts cannot inflate extent");
        let v = volume(&e, "body");
        if let Some(want) = expected { assert!((v - want).abs() < 1e-6); }
        expected = Some(v);
    }
}

#[test]
fn through_targets_resolve_inside_components_and_round_trip() {
    let src = format!("{RECT}{HOLE}\n\
        component Drill(section: face, target: solid) {{\n\
          solid tool(section, through: target)\ntool cut target\n}}\n\
        d: Drill(hole_f, body)\nsolid body(stock)\nsolid stock(sec, depth: 10mm)\n");
    let e = read(&src);
    assert!(volume(&e, "body") < 24000.0);
    let flat = format!("{RECT}{HOLE}solid stock(sec, depth: 10mm)\nsolid body(stock)\nsolid tool(hole_f, through: body)\ntool cut body\n");
    let (mut p, _) = parse(&flat);
    let printed = gcs_core::syntax::render_flat(&mut p).unwrap().to_string();
    assert!(printed.contains("through: body") && printed.contains("tool cut body"));
    assert!((volume(&read(&printed), "body") - volume(&e, "body")).abs() < 1e-6);
    let body = e.map.ent_named("body").unwrap();
    let copied = gcs_core::io::copy(&e.sketch, &[body]);
    assert_eq!(copied.solids.len(), e.sketch.solids.len());
    let deleted = gcs_core::io::without(&e.sketch, &[e.map.ent_named("hole").unwrap()], &[]);
    assert_eq!(deleted.solids.iter().map(|s| s.name.as_str()).collect::<Vec<_>>(), ["stock"],
        "losing a cutter must remove dependent bodies, not silently fill their holes");
    let i = copied.solids.iter().position(|s| s.name == "body").unwrap();
    assert!((copied.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::Report).unwrap().volume() - volume(&e, "body")).abs() < 1e-6);
}

#[test]
fn through_extent_refuses_mixed_labels_bad_targets_and_real_cycles() {
    for label in ["depth: 3mm", "from: 0mm, to: 3mm", "about: ab", "sweep: 90deg", "sense: cw"] {
        refused(&format!("{RECT}solid s(sec, through: target, {label})\n"), Code::E100, "cannot be combined");
    }
    refused(&format!("{RECT}solid s(sec, through: a)\n"), Code::E080, "solid target");
    refused(&format!("{RECT}solid s(sec, through: missing)\n"), Code::E101, "no such entity");
    refused(&format!("{RECT}solid s(sec, through: s)\n"), Code::E041, "made of itself");
    refused(&format!("{RECT}solid s(sec, through: body)\nsolid body(s)\n"), Code::E041, "made of itself");
    refused(&format!("{RECT}solid stock(sec, depth: 10mm)\nsolid s(sec, through: body)\nsolid body(stock)\ns on body\n"), Code::E041, "made of itself");
    refused(&format!("{RECT}solid a1(sec, through: b1)\nsolid b1(sec, through: a1)\n"), Code::E041, "made of itself");
    refused(&format!("{RECT}solid stock(sec, depth: 10mm)\nsolid body(stock)\nstock through body\n"), Code::E100, "now `cut`");
}

#[test]
fn through_extent_uses_the_cutters_normal_and_target_world_placement() {
    let src = format!("unit mm\npoint origin hint(x: 0, y: 0)\npoint toward hint(x: 1, y: 0)\n\
        plane front(origin: origin, toward: toward, u: (1, 0, 0), v: (0, 1, 0))\n\
        plane side0(origin: origin, toward: toward, u: (1, 0, 0), v: (0, 0, 1))\n\
        plane side(origin: origin, toward: toward, from: side0, offset: 80mm)\n\
        in front {{\n{}\n}}\n\
        in side {{\npoint hc hint(x: 30, y: -5)\ncircle h(center: hc) hint(r: 2)\nface hf(h)\n}}\n\
        solid stock(sec, depth: 10mm)\nsolid body(stock)\n\
        solid tool(hf, through: body)\ntool cut body\n", RECT.replace("unit mm\n", ""));
    let mut e = read(&src);
    let explicit = read(&src.replace("through: body", "from: -121mm, to: -79mm"));
    let want = volume(&explicit, "body");
    assert!(want < 24000.0 && want > 23000.0, "cut body volume: {want}");
    assert!((volume(&e, "body") - want).abs() < 1e-6);
    let tool = e.map.ent_named("tool").unwrap().i();
    let body = e.map.ent_named("body").unwrap().i();
    let a = e.sketch.evaluated_solid(tool, gcs_core::solid::ApproximationPolicy::Report).unwrap();
    assert!(a.world_bounds().lo[1] < 0.0 && a.world_bounds().hi[1] > 40.0);
    // A different placement of the target along the cutter normal changes its cached extent.
    e.sketch.planes[0].basis.o[1] = 100.0;
    let b = e.sketch.evaluated_solid(tool, gcs_core::solid::ApproximationPolicy::Report).unwrap();
    assert!(!std::rc::Rc::ptr_eq(&a, &b));
    assert!(b.world_bounds().lo[1] > 99.0 && b.world_bounds().hi[1] > 140.0);
    assert!((volume(&e, "body") - want).abs() < 1e-6);
    // Rotate both frames so the world box is no longer aligned with the extrusion axis.
    let rotate = |v: [f64; 3]| {
        let k = std::f64::consts::FRAC_1_SQRT_2;
        [k * (v[0] - v[1]), k * (v[0] + v[1]), v[2]]
    };
    for p in &mut e.sketch.planes {
        p.basis.u = rotate(p.basis.u);
        p.basis.v = rotate(p.basis.v);
        p.basis.o = rotate(p.basis.o);
    }
    assert!((volume(&e, "body") - want).abs() < 1e-6);
    let copied = gcs_core::io::copy(&e.sketch, &[gcs_core::model::EntRef::solid(body)]);
    let copied_body = copied.solids.iter().position(|s| s.name == "body")
        .expect("copying a placed body must retain its planes and geometry");
    assert!((copied.evaluated_solid(copied_body, gcs_core::solid::ApproximationPolicy::Report)
        .unwrap().volume() - want).abs() < 1e-6);
    let mesh = e.sketch.solid_mesh(body, 0.0);
    assert!(!mesh.positions.is_empty());
    let mut edges = std::collections::BTreeMap::new();
    for t in mesh.positions.chunks_exact(9) {
        let v: Vec<Vec<i64>> = t.chunks_exact(3).map(|p| p.iter().map(|x| (x * 1e6).round() as i64).collect()).collect();
        for j in 0..3 { *edges.entry((v[j].clone(), v[(j + 1) % 3].clone())).or_insert(0) += 1; }
    }
    for ((a, b), count) in &edges {
        assert_eq!(edges.get(&(b.clone(), a.clone())), Some(count), "mesh edges pair");
    }
}
