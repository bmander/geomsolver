//! **Faces and solids as the document writes them** (Solvent §6.8, §6.9).
//!
//! `tests/solid.rs` holds the kernel against arithmetic; this holds the *language* against the
//! kernel — that what a person writes reaches it, that the body rule is order-free, and that
//! every way of writing it wrong is refused where it is written.

use gcs_core::program::{elaborate, Code, Elaborated};
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
use std
in std.front {
a := point
b := point hint((60, 0))
c := point hint((60, 40))
d := point hint((0, 40))
(ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
horizontal ab
vertical bc
a distance(60) b
a distance(40) d
fix((0, 0)) a
}
sec := face(ab, bc, cd, da)
";

/// A circle of radius 5 at the middle of that rectangle, as `hole_f`.
const HOLE: &str = "\
in std.front {
o := point hint((30, 20))
a distance(30, along: x) o
a distance(20, along: y) o
hole := circle(center: o) hint(r: 5)
radius(5) hole
}
hole_f := face(hole)
";

#[test]
fn solid_operands_follow_surface_children_before_surfaces_are_built() {
    for statements in [
        "body := solid(side.solid)\nside.solid cut body\nside.solid union body\n",
        "side.solid union body\nside.solid cut body\nbody := solid(side.solid)\n",
    ] {
        let e = read(&format!("{statements}{RECT}\nax := line(a,d)\n\
            stock := solid(sec, about: ax)\nside := surface(stock, bc)\n"));
        let stock = e.map.ent_named("stock").unwrap().idx;
        let body = e.map.ent_named("body").unwrap().i();
        let gcs_core::model::SolidDef::Body {stock:s,on,through,..} = &e.sketch.solids[body].def
            else { panic!("expected body") };
        assert_eq!(*s,stock);
        assert_eq!(on,&vec![stock]);
        assert_eq!(through,&vec![stock]);
    }
}

/// A reported volume is the exact solid's, a sum of its faces' fluxes: the number a box's
/// arithmetic gives, to rounding.
fn assert_volume(got: f64, want: f64) {
    assert!((got - want).abs() <= 1e-12 * want.abs(), "{got} != {want}");
}

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
    let e = read(&format!("{RECT}block := solid(sec, depth: 30mm)\n"));
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
    let e = read(&format!("t := 30mm\n{RECT}block := solid(sec, depth: t)\n"));
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
        let base = format!("{RECT}{HOLE}face := -30mm\nback := 0mm\n");
        let named = read(&format!("{base}section := face({boundary})\nblock := solid(section, {sweep})\n"));
        let src = format!("{base}block := solid(face({boundary}), {sweep})\n");
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
    let src = format!("{RECT}block := solid(face(ab, bc, cd, da), depth: 2mm - 7mm)\n");
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
          block := solid(face(a, b, c, d, -> close), depth: t)\n\
        }}\n\
        first := Slab(a, b, c, d, t: 2mm)\n\
        second := Slab(a, b, c, d, t: 3mm)\n\
        repeat 2 as i {{\ncopy := Slab(a, b, c, d, t: (i + 1) * 1mm)\n}}\n");
    let e = read(&src);
    assert!((volume(&e, "first.block") - 4800.0).abs() < 1e-6);
    assert!((volume(&e, "second.block") - 7200.0).abs() < 1e-6);
    assert_eq!(e.sketch.faces.len(), 5);
    assert_eq!(e.sketch.solids.len(), 4);
    assert_eq!(e.sketch.lines.len(), 20);
}

#[test]
fn inline_sections_inherit_the_boundary_plane_and_coexist_with_forward_faces() {
    let src = "unit mm\nuse std\n\
        back := plane hint(origin: (0, 0, 12))\nfix(origin == (0, 0, 12)) back\nfix(dir == (1, 0, 0)) back.u\nfix(dir == (0, 1, 0)) back.v\n\
        in back {\na := point hint((0, 0))\nb := point hint((60, 0))\n\
        c := point hint((60, 40))\nd := point hint((0, 40))\n}\n\
        slab := solid(face(a, b, c, d, -> close), from: 0mm, to: 2mm)\n\
        named := solid(sec, from: 0mm, to: 2mm)\nsec := face(a, b, c, d, -> close)\n";
    let e = read(src);
    let back = e.map.ent_named("back").unwrap().i() as u32;
    assert!(e.sketch.faces.iter().all(|f| f.plane() == Ok(Some(back))));
    let p = gcs_core::report::positions(&e.sketch, &e.map);
    assert!(p.iter().any(|(n, v)| n == "slab.bounds.z0" && (*v - 12.0).abs() < 1e-9));
    assert_eq!(volume(&e, "slab"), volume(&e, "named"));
    refused(&src.replace("d := point hint((0, 40))\n}", "}\nd := point hint((0, 40))"),
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
        let src = format!("{RECT}bad := solid(face({boundary}), {sweep})\ngood := solid(sec, depth: 2mm)\n");
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
    refused(&format!("{RECT}bad := solid(face(missing), depth: 2mm)\n"), Code::E101, "missing");
    refused("bad := face(face(a, b, c, -> close))\n", Code::E100, "a solid's section");
    refused("bad := solid(face(a, -> close, b), depth: 2mm)\n", Code::E100, "last thing");
}

#[test]
fn the_body_rule_does_not_care_what_order_it_was_written_in() {
    // P2 at the language: `bore cut body` says what `body` *is*, wherever it stands
    let after = format!(
        "{RECT}{HOLE}stock := solid(sec, depth: 30mm)\nbore := solid(hole_f, depth: 30mm)\n\
         body := solid(stock)\nbore cut body\n"
    );
    let before = format!(
        "{RECT}{HOLE}stock := solid(sec, depth: 30mm)\nbore := solid(hole_f, depth: 30mm)\n\
         bore cut body\nbody := solid(stock)\n"
    );
    let (x, y) = (volume(&read(&after), "body"), volume(&read(&before), "body"));
    assert!((x - y).abs() < 1e-9, "one body, whichever order: {x} vs {y}");
    assert!(x < 72000.0 && x > 69000.0, "and the bore came out of it: {x}");
}

#[test]
fn a_swept_solid_takes_features_as_the_body_over_its_own_sweep() {
    // `bore cut stock` makes `stock` the body over its own sweep: the same object
    // `body := solid(stock)` names, with no second name for it
    let parts = format!("{RECT}{HOLE}stock := solid(sec, depth: 30mm)\nbore := solid(hole_f, depth: 30mm)\n");
    let body = volume(&read(&format!("{parts}body := solid(stock)\nbore cut body\n")), "body");
    let e = read(&format!("{parts}bore cut stock\n"));
    assert_volume(volume(&e, "stock"), body);
    assert_volume(volume(&read(&format!("{RECT}{HOLE}bore cut stock\nstock := solid(sec, depth: 30mm)\n\
         bore := solid(hole_f, depth: 30mm)\n")), "stock"), body);
    // a face is called what it was before anything stood on it, and a feature's through it
    let p = gcs_core::report::positions(&e.sketch, &e.map);
    let has = |n: &str| p.iter().any(|(k, _)| k == n);
    assert!(has("stock.near.area") && has("stock.ab.area") && has("stock.bore.hole.area"), "{p:?}");
    assert!(!has("stock.stock.near.area"), "the sweep adds no step to its own faces' names");
    // and the name means the whole object wherever it is read: a cutter through it, a body
    // over it
    let through = read(&format!("{parts}pin := solid(hole_f, through: stock)\npin cut stock\n"));
    assert_volume(volume(&through, "stock"), body);
    let over = read(&format!("{parts}bore cut stock\nwhole := solid(stock)\n"));
    assert_volume(volume(&over, "whole"), body);
    refused(&format!("{parts}stock cut stock\n"), Code::E080, "is cut itself");
}

#[test]
fn a_body_keeps_what_lies_within_everything_that_bounds_it() {
    // `bound` is the body rule's third side: the block within the bore is the bore's own
    // polygon (what `cut` takes out of it), and it is a set like the other two
    let stock = format!("{RECT}{HOLE}stock := solid(sec, depth: 30mm)\nbore := solid(hole_f, depth: 30mm)\n");
    let within = volume(&read(&format!("{stock}body := solid(stock)\nbore bound body\n")), "body");
    let before = volume(&read(&format!("{stock}bore bound body\nbody := solid(stock)\n")), "body");
    let cut = volume(&read(&format!("{stock}body := solid(stock)\nbore cut body\n")), "body");
    assert!((within - before).abs() < 1e-9, "one body, whichever order: {within} vs {before}");
    assert!((within + cut - 72000.0).abs() < 1e-6, "within plus cut is the stock: {within} + {cut}");
    assert!(within > 2300.0 && within < 2400.0, "the bore's own polygon: {within}");
    // difference and intersection commute, so a body bounded and cut is the same either way
    let both = format!(
        "{RECT}{HOLE}in std.front {{\nq := point hint((30, 20))\na distance(30, along: x) q\na distance(20, along: y) q\n\
         wide := circle(center: q) hint(r: 15)\nradius(15) wide\n}}\nwide_f := face(wide)\n\
         stock := solid(sec, depth: 30mm)\nbore := solid(hole_f, depth: 30mm)\ndisc := solid(wide_f, depth: 30mm)\n\
         body := solid(stock)\ndisc bound body\nbore cut body\n"
    );
    let disc = volume(&read(&both), "body");
    let annulus = volume(&read(&format!("{RECT}{HOLE}in std.front {{\nq := point hint((30, 20))\na distance(30, along: x) q\n\
         a distance(20, along: y) q\nwide := circle(center: q) hint(r: 15)\nradius(15) wide\n}}\nwide_f := face(wide)\n\
         bore := solid(hole_f, depth: 30mm)\ndisc := solid(wide_f, depth: 30mm)\nbody := solid(disc)\nbore cut body\n")), "body");
    assert!((disc - annulus).abs() < 1e-6, "bounded then cut is the annulus: {disc} vs {annulus}");
    refused(
        &format!("{RECT}s := solid(sec, depth: 3mm)\nx := solid(s)\ny := solid(x)\nx bound y\ny bound x\n"),
        Code::E041,
        "made of itself",
    );
    refused(&format!("{RECT}s := solid(sec, depth: 3mm)\nx := solid(s)\nx bound x\n"), Code::E080, "is bound itself");
}

#[test]
fn a_boss_is_united_with_its_body() {
    // `union` is the body rule's own word (§6.9); `on` is a constraint and relates no solids
    let src = format!(
        "{RECT}in std.front {{\ne := point hint((20, 10))\nf := point hint((40, 10))\n\
         g := point hint((40, 30))\nh := point hint((20, 30))\n\
         (ef := line(e, f)) -> (fg := line(f, g)) -> (gh := line(g, h)) -> (he := line(h, e)) -> close\n\
         a distance(20, along: x) e\na distance(10, along: y) e\n\
         a distance(40, along: x) g\na distance(30, along: y) g\n\
         horizontal ef\nvertical fg\nhorizontal gh\nvertical he\n}}\n\
         boss_f := face(ef, fg, gh, he)\n\
         block := solid(sec, depth: 30mm)\nboss := solid(boss_f, from: 0mm, to: 10mm)\n\
         body := solid(block)\nboss union body\n"
    );
    let e = read(&src);
    let want = 60.0 * 40.0 * 30.0 + 20.0 * 20.0 * 10.0;
    assert!((volume(&e, "body") - want).abs() < 1e-6, "a boss adds: {}", volume(&e, "body"));
    refused(&src.replace("boss union body", "boss coincident body"), Code::E040, "does not relate a solid to a solid");
}

#[test]
fn a_revolution_turns_about_a_line_in_its_own_plane() {
    let src = "\
unit mm
use std (horizontal)
in std.front {
p0 := point hint((10, 0))
p1 := point hint((14, 0))
p2 := point hint((14, 6))
p3 := point hint((10, 6))
(e0 := line(p0, p1)) -> (e1 := line(p1, p2)) -> (e2 := line(p2, p3)) -> (e3 := line(p3, p0)) -> close
q0 := point
q1 := point hint((0, 10))
ax := line(q0, q1)
fix((0, 0)) q0
vertical ax
horizontal e0
vertical e1
p0 distance(10, along: x) q0
p0 horizontal q0
p0 distance(4) p1
p1 distance(6) p2
horizontal e2
vertical e3
}
sec := face(e0, e1, e2, e3)
ring := solid(sec, about: ax)
";
    let e = read(src);
    let want = std::f64::consts::TAU * 12.0 * 24.0;
    let got = volume(&e, "ring");
    assert!((got - want).abs() < 2e-3 * want, "Pappus from the source: want ≈ {want}, got {got}");
    for sweep in ["", ", sweep: 90deg, sense: cw"] {
        let named = src.replace("about: ax)", &format!("about: ax{sweep})"));
        let inline = named.replace("sec := face(e0, e1, e2, e3)\n", "")
            .replace("ring := solid(sec,", "ring := solid(face(e0, e1, e2, e3),");
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
    let long = read(&format!("{RECT}block := solid(sec, depth: 30mm)\n"));
    let short = read(&format!(
        "{RECT}brief := face(a, bc, cd, -> close)\nblock := solid(brief, depth: 30mm)\n"
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
    let (mut prog, _) = crate::common::parse("brief := face(a, bc, cd, -> close)\n");
    let text = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    assert_eq!(text.split_whitespace().collect::<Vec<_>>().join(" "),
        "brief := face(a, bc, cd, -> close)");
}

/// A loop of nothing but corners: the rectangle again, with no line drawn at all.
#[test]
fn a_face_may_be_written_as_its_corners_alone() {
    let e = read(&format!("{RECT}quad := face(a, b, c, d, -> close)\nslab := solid(quad, depth: 2mm)\n"));
    assert!((volume(&e, "slab") - 4800.0).abs() < 1e-6, "{}", volume(&e, "slab"));
}

#[test]
fn closing_lines_do_not_take_an_existing_edges_name() {
    let e = read(&format!(
        "{RECT}close0 := line(b, c)\nbrief := face(a, close0, cd, -> close)\n\
         block := solid(brief, depth: 2mm)\n"
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
    let plain = read(&format!("{RECT}block := solid(sec, depth: 30mm)\n"));
    let said = read(&format!(
        "{RECT}same := face(ab, bc, cd, da, -> close)\nblock := solid(same, depth: 30mm)\n"
    ));
    assert_eq!(said.sketch.lines.len(), plain.sketch.lines.len(), "nothing to mint");
    assert!((volume(&said, "block") - volume(&plain, "block")).abs() < 1e-9);
    let circle = read(&format!("{RECT}{HOLE}same := face(hole, -> close)\n"));
    assert_eq!(circle.sketch.lines.len(), plain.sketch.lines.len());
    assert_eq!(circle.sketch.faces.last().unwrap().edges.len(), 1);
}

#[test]
fn a_mixed_faces_seed_writeback_changes_only_the_points() {
    use gcs_core::edit::{self, Kind};

    let src = format!("{RECT}brief := face(a, bc, cd, -> close)\nblock := solid(brief, depth: 2mm)\n");
    let mut e = read(&src);
    let mut sk = std::mem::take(&mut e.sketch);
    let unchanged = edit::commit_seeds(&e, &sk, &e.program);
    assert_eq!(unchanged.kind, Kind::None);
    assert_eq!(unchanged.text, src);

    let b = e.map.ent_named("b").unwrap();
    let [x, _] = sk.point_params(b.i());
    sk.params[x as usize].value = 65.0;
    let want = src.replace("b := point hint((60, 0))", "b := point hint((65, 0))");
    let moved = edit::commit_seeds(&e, &sk, &e.program);
    assert_eq!(moved.kind, Kind::Numeric);
    assert_eq!(moved.text, want);
    let synced = edit::reconcile(&mut e, &sk);
    assert!(synced.refused.is_none(), "{:?}", synced.refused);
    assert_eq!(synced.text, want);
    let back = read(&synced.text);
    assert_eq!(back.sketch.lines.len(), sk.lines.len());
    assert_eq!(back.sketch.point_xy(b.i()), (65.0, 0.0));
}

#[test]
fn closing_lines_stay_implicit_across_reconciliation_and_reload() {
    use gcs_core::edit::{self, Kind};

    for src in [
        format!("{RECT}quad := face(a, b, c, d, -> close)\n"),
        format!("{RECT}slab := solid(face(a, b, c, d, -> close), depth: 2mm)\n"),
        format!(
            "unit mm\nuse std\ncomponent Patch(f: plane) {{\n{}\
             slab := solid(face(a, bc, cd, -> close), depth: 2mm)\n}}\npart := Patch(std.front)\n",
            RECT.trim_start_matches("unit mm\nuse std\n").replace("in std.front", "in f"),
        ),
        "\
use std
component Patch() {
a := point hint((0, 0))
b := point hint((60, 0))
c := point hint((0, 40))
tri := face(a, b, c, -> close)
}
in std.front {
part := Patch()
}
".to_string(),
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
        let src = format!("{RECT}bad := face({walk})\n");
        refused(&src, Code::E080, "along the walk");
        let (p, _) = parse(&src);
        let e = elaborate(&p);
        assert_eq!(e.sketch.lines.len(), 4, "a refused face leaves no closing lines");
        assert_eq!(e.sketch.faces.len(), 1, "only the original rectangle remains");
    }
    // A failure after minting some lines must also leave nothing for reconciliation to
    // interpret as newly drawn geometry.
    let (p, _) = parse(&format!("{RECT}bad := face(a, c, d)\n"));
    let e = elaborate(&p);
    assert!(!e.ok());
    assert_eq!(e.sketch.lines.len(), 4);
}

#[test]
fn an_arc_and_its_chord_share_both_ends_and_still_form_a_loop() {
    let src = "\
unit mm
use std
in std.front {
o := point hint((0, 0))
a := point hint((-5, 0))
b := point hint((5, 0))
rim := arc(center: o, start: a, end: b) hint(r: 5)
chord := line(a, b)
}
";
    let mut volumes = Vec::new();
    for walk in ["rim, chord", "chord, rim", "a, rim, b, -> close", "b, rim, a, -> close"] {
        let e = read(&format!("{src}half := face({walk})\nslab := solid(half, depth: 2mm)\n"));
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
        &format!("{RECT}z := point hint((90, 90))\nzz := line(z, a)\nbad := face(ab, zz, cd, da)\n"),
        Code::E080,
        "share no point",
    );
    // a circle standing in a loop rather than being one
    refused(&format!("{RECT}{HOLE}bad := face(ab, hole)\n"), Code::E080, "a whole loop");
    // a face of something that is neither an edge nor a corner
    refused(&format!("{RECT}bad := face(sec)\n"), Code::E080, "bounded by lines");
    // -- and the shorthand does not swallow a mistake (issue #49, item 1) --------------------
    // a loop whose last item does not come back to its first says so, or says `-> close`
    refused(&format!("{RECT}bad := face(a, ab, bc)\n"), Code::E080, "`-> close`");
    // an edge between two gaps has two readings and no statement choosing one
    refused(&format!("{RECT}bad := face(a, bc, d, -> close)\n"), Code::E080, "meets neither");
    // a straight loop between two corners is a line drawn twice
    refused(&format!("{RECT}bad := face(a, b, -> close)\n"), Code::E080, "three corners");
    // and one item is a loop only when it is a circle
    refused(&format!("{RECT}bad := face(ab)\n"), Code::E080, "not a loop by itself");
    // `-> close` seals a loop, and only a face is one
    refused(&format!("{RECT}bad := line(a, -> close)\n"), Code::E100, "not a loop");
    refused(&format!("{RECT}bad := face(a, -> close, ab)\n"), Code::E100, "last thing in the list");
    // a swept solid over something that is not a face
    refused(&format!("{RECT}bad := solid(ab, depth: 3mm)\n"), Code::E080, "written over a face");
    // a body made of what is not a solid
    refused(&format!("{RECT}bad := solid(sec)\n"), Code::E080, "made of solids");
    // a prism swept nowhere
    refused(
        &format!("{RECT}bad := solid(sec, from: 0mm, to: 0mm)\n"),
        Code::E080,
        "swept nowhere",
    );
    // **a selector is a word, never a sign**: a negative sweep is refused at the value
    refused(
        &format!("{RECT}in std.front {{\nq0 := point\nq1 := point hint(y: 10)\nax := line(q0, q1)\n}}\n\
                  bad := solid(sec, about: ax, sweep: -90deg)\n"),
        Code::E040,
        "which way it turns is `sense: cw`",
    );
    // a revolution about something that is not a line
    refused(&format!("{RECT}bad := solid(sec, about: a)\n"), Code::E081, "turns about a line");
    // a body made of itself
    refused(
        &format!("{RECT}s := solid(sec, depth: 3mm)\nx := solid(s)\ny := solid(x)\nx cut y\ny cut x\n"),
        Code::E041,
        "made of itself",
    );
    // two sweeps cutting each other are each made of the other
    refused(
        &format!("{RECT}{HOLE}s := solid(sec, depth: 3mm)\nh := solid(hole_f, depth: 3mm)\nh cut s\ns cut h\n"),
        Code::E041,
        "made of itself",
    );
    // a mixture of the two sweeps
    refused(
        &format!("{RECT}in std.front {{\nq0 := point\nq1 := point hint(y: 10)\nax := line(q0, q1)\n}}\n\
                  bad := solid(sec, depth: 3mm, about: ax)\n"),
        Code::E100,
        "not both",
    );
}

#[test]
fn a_plane_may_be_stood_off_another() {
    // a plane over axes parallel to another's is parallel to it, and a distance between the two
    // stands one off the other along its normal
    let mut e = read("\
unit mm
use std
front := plane(u: std.x, v: std.z)
back := plane hint(origin: (0, -12, 0))
back.u parallel std.x
back.v parallel std.z
front distance(12mm) back
");
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let b = |n: &str| e.sketch.basis(e.map.ent_named(n).unwrap().i());
    let near = |x: [f64; 3], y: [f64; 3]| (0..3).all(|k| (x[k] - y[k]).abs() < 1e-12);
    assert!(near(b("front").u, b("back").u) && near(b("front").v, b("back").v), "parallel");
    assert!((b("back").along_normal() - 12.0).abs() < 1e-9, "twelve along its own normal: {}",
        b("back").along_normal());
    assert_eq!(b("front").o, [0.0; 3], "and the plane it stands off stands where it did");
}

// Issue #49.3: naming the traversal must not change the drawing it traverses.
fn named_rect() -> String {
    RECT.replace("(ab := line(a, b))", "profile := (ab := line(a, b))")
        .replace("sec := face(ab, bc, cd, da)\n", "")
}

#[test]
fn named_chains_sweep_the_same_geometry_and_keep_edge_names() {
    let old = read(&format!("{RECT}block := solid(sec, depth: 8mm)\n"));
    let src = format!("{}block := solid(profile, depth: 8mm)\n", named_rect());
    let new = read(&src);
    assert_volume(volume(&new, "block"), 19200.0);
    assert_eq!(new.sketch.params.len(), old.sketch.params.len());
    assert_eq!(new.sketch.constraints.len(), old.sketch.constraints.len());
    assert_eq!(new.sketch.lines.len(), old.sketch.lines.len());
    assert_eq!(new.sketch.faces.len(), old.sketch.faces.len());
    assert_eq!(new.map.ent_named("ab"), old.map.ent_named("ab"));
    let report = |e: &Elaborated| gcs_core::report::positions(&e.sketch, &e.map)
        .into_iter().filter(|(n, _)| n.starts_with("block.")).collect::<Vec<_>>();
    assert_eq!(report(&new), report(&old));
    let written = gcs_core::edit::commit_seeds(&new, &new.sketch, &new.program);
    assert!(written.text.contains("profile := (ab := line(a, b))"), "{}", written.text);
    assert_eq!(volume(&read(&written.text), "block"), volume(&new, "block"));
    let no_change = gcs_core::edit::remove(&new, &new.program, &new.sketch,
        &[new.map.ent_named("profile").unwrap()], &[]);
    assert!(no_change.refused.is_some(), "deleting a group must not strand its edge readers");
}

#[test]
fn named_chains_support_anonymous_links_and_constraint_words() {
    let e = read("\
use std
in std.front {
profile := distance(10) line -> equal line -> equal line -> close
}
block := solid(profile, depth: 8)
");
    assert_eq!(e.sketch.points.len(), 3 + crate::common::STD_POINTS);
    assert_eq!(e.sketch.lines.len(), 3);
    assert_eq!(e.sketch.faces.len(), 1);
    assert_eq!(e.sketch.user_constraints().len(), 3);
    assert!(e.map.ent_named("profile").is_some());
    // with no `->` there is no traversal: the definition names the line, which deletes whole
    let single = read("use std\nin std.front {\ntrail := line\n}\n");
    assert_eq!(single.sketch.lines.len(), 1);
    assert_eq!(single.sketch.faces.len(), 0);
    assert!(single.map.ent_named("trail") == Some(gcs_core::model::EntRef::line(0)));
    let edit = gcs_core::edit::remove(&single, &single.program, &single.sketch,
        &[gcs_core::model::EntRef::line(0)], &[]);
    assert!(edit.refused.is_none());
    assert_eq!(edit.text.trim(), "use std\nin std.front {\n}");
}

#[test]
fn anonymous_chain_sides_have_stable_names_without_hiding_named_sides() {
    let src = "\
use std
in std.front {
a := point hint((0, 0))
b := point hint((10, 0))
c := point hint((0, 10))
profile := line(a, b) -> (edge0 := line(b, c)) -> line(c, a) -> close
}
part := solid(profile, depth: 8)
";
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
    let src = "\
unit mm
use std
component Shape() {
a := point hint((0, 0))
b := point hint((10, 0))
c := point hint((10, 20))
d := point hint((0, 20))
profile := (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
}
component Slab(section: face, t: Length) {
body := solid(section, depth: t)
}
first := solid(s.profile, depth: 2mm)
in std.front {
x := Slab(s.profile, t: 3mm)
repeat 2 as i {
copy := Shape()
prism := solid(copy.profile, depth: (i + 1) * 1mm)
}
selected := solid(copy[1].profile, depth: 4mm)
s := Shape()
}
";
    let e = read(src);
    assert_eq!(e.sketch.faces.len(), 3);
    assert_eq!(e.sketch.solids.len(), 5);
    assert_eq!(volume(&e, "first"), 400.0);
    assert_volume(volume(&e, "x.body"), 600.0);
    assert_eq!(volume(&e, "selected"), 800.0);
    assert!(e.map.ent_named("s.ab").is_some());
}

#[test]
fn named_open_chains_can_be_closed_explicitly_but_cannot_be_swept_directly() {
    let src = named_rect().replace(" -> (da := line(d, a)) -> close", "");
    let e = read(&src);
    assert_eq!(e.sketch.lines.len(), 3);
    assert_eq!(e.sketch.faces.len(), 0);
    refused(&format!("{src}bad := solid(profile, depth: 8mm)\n"), Code::E080, "open chain");
    refused(&format!("{src}bad := face(profile)\n"), Code::E080, "share no point");
    let e = read(&format!("{src}block := solid(face(profile, -> close), depth: 8mm)\n"));
    assert_eq!(e.sketch.lines.len(), 4);
    assert_volume(volume(&e, "block"), 19200.0);
    let e = read(&format!("{src}sec := face(profile, a)\nblock := solid(sec, depth: 8mm)\n"));
    assert_volume(volume(&e, "block"), 19200.0);
}

#[test]
fn named_chains_inherit_planes_and_revolve_with_arcs() {
    let src = "unit mm\nuse std\n\
        back := plane hint(origin: (0, 0, 12))\nfix(origin == (0, 0, 12)) back\nfix(dir == (1, 0, 0)) back.u\nfix(dir == (0, 1, 0)) back.v\n\
        in back {\na := point hint((0, -5))\nb := point hint((0, 5))\n\
          c := point hint((0, 0))\n\
          profile := (rim := arc(center: c, start: a, end: b) hint(r: 5)) -> (ax := line(b, a)) -> close\n\
          ball := solid(profile, about: ax)\n}\n";
    let e = read(src);
    let back = e.map.ent_named("back").unwrap().i() as u32;
    assert_eq!(e.sketch.faces[0].plane(), Ok(Some(back)));
    // Reports integrate the faceted surface; the analytic sphere is an independent check.
    let want = 4.0 / 3.0 * std::f64::consts::PI * 125.0;
    assert!((volume(&e, "ball") / want - 1.0).abs() < 0.002,
        "sphere volume: {} versus {want}", volume(&e, "ball"));
    let old = src.replace("profile := ", "")
        .replace("ball := solid(profile,", "sec := face(rim, ax)\nball := solid(sec,");
    assert_eq!(volume(&e, "ball"), volume(&read(&old), "ball"));
    assert!(gcs_core::program::solid_diagnostics(&e.sketch, &e.map).is_empty());
}

#[test]
fn named_chains_validate_names_links_and_plane_membership() {
    refused("profile := line equal line\n", Code::E100, "every pair");
    refused("component A() { profile := line -> }\na := A()\n",
        Code::E100, "must finish");
    refused("use std\nin std.front {\nprofile := line\nprofile := point\n}\n", Code::E001, "declared twice");
    refused("use std\nin std.front {\nprofile := point\nprofile := line\n}\n", Code::E001, "declared twice");
    refused("\
use std
in std.front {
p := point
q := point
c := circle
profile := (a := line(p, q)) -> c -> (b := line(q, p)) -> close
}
",
        Code::E080, "lines and arcs");
    refused("\
use std
in std.front {
p := point
q := point
profile := (a := line(p, q)) -> profile.nope -> (b := line(q, p)) -> close
}
",
        Code::E080, "lines and arcs");
    refused(&format!("{}bad := solid(profile.typo, depth: 8mm)\n", named_rect()),
        Code::E080, "not a member");
    let src = named_rect();
    let src = format!("{}v := plane(u: std.x, v: std.y)\nin v {{\nd := point hint((0, 40))\n}}\n", src)
        .replace("d := point hint((0, 40))\nprofile", "profile");
    // a corner drawn in another plane: the links drawn in `std.front` claim it there
    refused(&src, Code::E060, "already in `std.front`");
}

#[test]
fn named_chain_source_is_retained_and_its_binding_is_highlighted() {
    let src = "use std\nin std.front {\nprofile := line -> line -> line -> close\n}\n";
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
        ("trail := line equal line\nunrelated := point\n", "every pair", "equal"),
        ("component A() { trail := line -> }\nunrelated := point\n", "must finish", "->"),
    ] {
        let (_, errors) = parse(src);
        let error = errors.iter().find(|e| e.message.contains(message)).unwrap();
        assert_eq!(&src[error.span.lo as usize..error.span.hi as usize], joint);
    }
}

#[test]
fn a_refused_named_loop_does_not_misnumber_later_sections() {
    let src = format!("broken := line -> line -> close\n{}\n\
        block := solid(profile, depth: 8mm)\n", named_rect());
    let (p, errors) = parse(&src);
    assert!(errors.is_empty(), "{errors:?}");
    let e = elaborate(&p);
    assert!(!e.ok());
    assert_eq!(e.sketch.faces.len(), 1);
    assert_eq!(e.sketch.solids.len(), 1);
    assert_volume(volume(&e, "block"), 19200.0);
}

#[test]
fn through_extent_spans_additions_but_only_cut_subtracts() {
    let src = format!("{RECT}{HOLE}\n\
        stock := solid(sec, depth: 10mm)\n\
        boss := solid(sec, from: 0mm, to: 5mm)\n\
        body := solid(stock)\nboss union body\n\
        bore := solid(hole_f, through: body)\n");
    let uncut = read(&src);
    assert_volume(volume(&uncut, "body"), 36000.0);
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
        "stock := solid(sec, depth: 10mm)",
        "body := solid(stock)",
        "bore := solid(hole_f, through: body)",
        "bore cut body",
        "huge := solid(hole_f, from: -1000mm, to: 1000mm)",
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
          tool := solid(section, through: target)\ntool cut target\n}}\n\
        d := Drill(hole_f, body)\nbody := solid(stock)\nstock := solid(sec, depth: 10mm)\n");
    let e = read(&src);
    assert!(volume(&e, "body") < 24000.0);
    let flat = format!("{RECT}{HOLE}stock := solid(sec, depth: 10mm)\nbody := solid(stock)\ntool := solid(hole_f, through: body)\ntool cut body\n");
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
        refused(&format!("{RECT}s := solid(sec, through: target, {label})\n"), Code::E100, "cannot be combined");
    }
    refused(&format!("{RECT}s := solid(sec, through: a)\n"), Code::E080, "solid target");
    refused(&format!("{RECT}s := solid(sec, through: missing)\n"), Code::E101, "no such entity");
    refused(&format!("{RECT}s := solid(sec, through: s)\n"), Code::E041, "made of itself");
    refused(&format!("{RECT}s := solid(sec, through: body)\nbody := solid(s)\n"), Code::E041, "made of itself");
    refused(&format!("{RECT}stock := solid(sec, depth: 10mm)\ns := solid(sec, through: body)\nbody := solid(stock)\ns union body\n"), Code::E041, "made of itself");
    refused(&format!("{RECT}a1 := solid(sec, through: b1)\nb1 := solid(sec, through: a1)\n"), Code::E041, "made of itself");
    refused(&format!("{RECT}stock := solid(sec, depth: 10mm)\nbody := solid(stock)\nstock through body\n"), Code::E100, "now `cut`");
}

#[test]
fn through_extent_uses_the_cutters_normal_and_target_world_placement() {
    let src = format!("unit mm\nuse std\n\
        front := plane(u: std.x, v: std.y)\nfix(origin == (0, 0, 0)) front\n\
        side := plane hint(origin: (0, -80, 0))\nfix(origin == (0, -80, 0)) side\nfix(dir == (1, 0, 0)) side.u\nfix(dir == (0, 0, 1)) side.v\n\
        {}\n\
        in side {{\nhc := point hint((30, -5))\nh := circle(center: hc) hint(r: 2)\nhf := face(h)\n}}\n\
        stock := solid(sec, depth: 10mm)\nbody := solid(stock)\n\
        tool := solid(hf, through: body)\ntool cut body\n",
        RECT.replace("unit mm\nuse std\n", "").replace("in std.front", "in front"));
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
    let mut o = e.sketch.basis(0).o;
    o[1] = 100.0;
    e.sketch.set_plane_origin(0, o);
    let b = e.sketch.evaluated_solid(tool, gcs_core::solid::ApproximationPolicy::Report).unwrap();
    assert!(!std::rc::Rc::ptr_eq(&a, &b));
    assert!(b.world_bounds().lo[1] > 99.0 && b.world_bounds().hi[1] > 140.0);
    assert!((volume(&e, "body") - want).abs() < 1e-6);
    // Rotate both frames so the world box is no longer aligned with the extrusion axis.
    let rotate = |v: [f64; 3]| {
        let k = std::f64::consts::FRAC_1_SQRT_2;
        [k * (v[0] - v[1]), k * (v[0] + v[1]), v[2]]
    };
    crate::common::move_space(&mut e.sketch, rotate, [0.0; 3]);
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

const ANNULUS: &str = "unit mm\nuse std\nin std.front {\ncenter := point hint((0, 0))\nouter := circle(center: center) hint(r: 10)\ninner := circle(center: center) hint(r: 5)\n}\nsection := face(outer, holes: inner)\n";

#[test]
fn section_holes_sweep_and_cut_with_source_face_names() {
    use gcs_core::solid::{ApproximationPolicy::Report, WorldPoint};
    let e = read(&format!("{ANNULUS}sleeve := solid(section, depth: 3mm)\nstock := solid(face(outer), depth: 6mm)\nbody := solid(stock)\nsleeve cut body\n"));
    let ring = e.sketch.evaluated_solid(e.map.ent_named("sleeve").unwrap().i(), Report).unwrap();
    assert!((ring.volume() - 225.0 * std::f64::consts::PI).abs() < 0.6);
    assert!(!ring.contains_world(WorldPoint([0.0, 1.0, 0.0])));
    assert!(ring.contains_world(WorldPoint([7.0, 1.0, 0.0])));
    for name in ["sleeve.outer", "sleeve.inner", "sleeve.near", "sleeve.far"] {
        assert!(ring.surviving_faces().contains(name), "{name}");
    }
    assert_eq!(ring.round_features().len(), 2);
    assert_eq!(super::solid::unpaired(ring.mesh()), 0);
    let body = e.sketch.evaluated_solid(e.map.ent_named("body").unwrap().i(), Report).unwrap();
    assert!(body.contains_world(WorldPoint([0.0, 1.0, 0.0])), "the groove leaves the core standing");
    assert!(!body.contains_world(WorldPoint([7.0, 1.0, 0.0])));
}

#[test]
fn section_holes_round_trip_copy_delete_and_invalidate() {
    use gcs_core::{model::EntRef, solid::ApproximationPolicy::Report};
    let src = format!("{ANNULUS}sleeve := solid(face(outer, holes: inner), from: 1mm, to: 4mm)\n");
    let (mut p, _) = parse(&src);
    let printed = gcs_core::syntax::render_flat(&mut p).unwrap().to_string();
    assert!(printed.contains("holes: inner"));
    let mut e = read(&printed);
    let i = e.map.ent_named("sleeve").unwrap().i();
    let before = e.sketch.evaluated_solid(i, Report).unwrap();
    let hole = e.map.ent_named("inner").unwrap();
    let radius = e.sketch.circles[hole.i()].radius as usize;
    e.sketch.params[radius].value = 6.0;
    let after = e.sketch.evaluated_solid(i, Report).unwrap();
    assert!(after.volume() < before.volume());
    assert!(!std::rc::Rc::ptr_eq(&before, &after));
    let copy = gcs_core::io::copy(&e.sketch, &[EntRef::solid(i)]);
    assert!((copy.evaluated_solid(0, Report).unwrap().volume() - after.volume()).abs() < 1e-8);
    let deleted = gcs_core::io::without(&e.sketch, &[hole], &[]);
    assert!(deleted.faces.is_empty() && deleted.solids.is_empty());
}

#[test]
fn section_holes_work_in_through_cutters_and_their_targets() {
    let e = read(&format!("{ANNULUS}stock := solid(section, depth: 8mm)\nbody := solid(stock)\ntool := solid(face(outer, holes: inner), through: body)\ntool cut body\n"));
    assert!(volume(&e, "body").abs() < 1e-8);
    assert!(volume(&e, "tool") > volume(&e, "stock"));
}

#[test]
fn section_holes_accept_multiple_component_loops_and_revolutions() {
    // A section of area 2400 - 100 - 100 = 2200, with both holes at x=30.
    // Revolving about the left edge gives 2*pi*30*2200 by Pappus's theorem.
    let src = format!("{RECT}\ncomponent Hole(y: Length) {{\n\
        p := point hint((25, y))\nq := point hint((35, y))\n\
        r := point hint((35, y + 10mm))\ns := point hint((25, y + 10mm))\n\
        profile := (a := line(p,q)) -> (b := line(q,r)) -> (c := line(r,s)) -> (d := line(s,p)) -> close\n}}\n\
        in std.front {{\nh0 := Hole(y: 5mm)\nh1 := Hole(y: 25mm)\n}}\n\
        section := face(ab, bc, cd, da, holes: h0.profile, h1.profile)\n\
        slab := solid(section, depth: 3mm)\nturned := solid(section, about: da)\n\
        quarter := solid(section, about: da, sweep: 90deg)\n");
    let e = read(&src);
    let slab = e.sketch.evaluated_solid(e.map.ent_named("slab").unwrap().i(),
        gcs_core::solid::ApproximationPolicy::Report).unwrap();
    assert!((slab.volume() - 6600.0).abs() < 1e-7);
    let expected = 2.0 * std::f64::consts::PI * 30.0 * 2200.0;
    for (name, fraction) in [("turned", 1.0), ("quarter", 0.25)] {
        let i = e.map.ent_named(name).unwrap().i();
        let solid = e.sketch.evaluated_solid(i,
            gcs_core::solid::ApproximationPolicy::View { unit: 0.1 }).unwrap();
        assert!((solid.volume() / (expected * fraction) - 1.0).abs() < 0.005);
        assert!(solid.surviving_faces().contains(&format!("{name}.h0.profile.a")));
    }
}

#[test]
fn section_holes_refuse_invalid_boundaries() {
    use gcs_core::solid::ApproximationPolicy::Report;
    for (x, r, extra, message) in [
        (11, 2, "", "strictly inside"),
        (9, 2, "", "strictly inside"),
        (8, 2, "", "strictly inside"),
        (0, 0, "", "degenerate hole"),
        (0, 6, ", second", "disjoint"),
        (4, 3, ", second", "disjoint"),
        (7, 2, ", second", "disjoint"),
    ] {
        let src = format!("unit mm\nuse std\nin std.front {{\no := point hint((0, 0))\nh := point hint(({x}, 0))\nouter := circle(center: o) hint(r: 10)\ninner := circle(center: h) hint(r: {r})\nsecond := circle(center: o) hint(r: 5)\n}}\nsleeve := solid(face(outer, holes: inner{extra}), depth: 3mm)\n");
        let e = read(&src);
        let err = e.sketch.evaluated_solid(0, Report).err().expect("invalid section must be refused");
        assert!(err.contains(message), "{src}\n{err}");
    }
    refused(&format!("{ANNULUS}bad := line(center, center)\nbadface := face(outer, holes: bad)\n"), Code::E080, "circle or named closed loop");
    refused("unit mm\nuse std\nother := plane\nin std.front {\no := point\nouter := circle(center: o) hint(r: 10)\n}\nin other { h := point\ninner := circle(center: h) hint(r: 2) }\nbad := face(outer, holes: inner)\n", Code::E080, "one plane");
}

#[test]
fn section_holes_check_source_arc_and_line_tangencies_on_a_placed_plane() {
    use gcs_core::solid::ApproximationPolicy::Report;
    for (x, valid) in [(1, false), (2, true), (4, false)] {
        let src = format!("unit mm\nuse std\n\
            back := plane hint(origin: (0, 0, 12))\nfix(origin == (0, 0, 12)) back\nfix(dir == (1, 0, 0)) back.u\nfix(dir == (0, 1, 0)) back.v\n\
            in back {{\nc := point hint((0, 0))\n\
            a := point hint((0, -5))\nb := point hint((0, 5))\n\
            outline := (rim := arc(center: c, start: a, end: b) hint(r: 5)) -> (side := line(b,a)) -> close\n\
            h := point hint(({x}, 0))\ninner := circle(center: h) hint(r: 1)\n\
            sleeve := solid(face(outline, holes: inner), depth: 2mm)\n}}\n");
        let e = read(&src);
        let result = e.sketch.evaluated_solid(0, Report);
        assert_eq!(result.is_ok(), valid, "x={x}: {:?}", result.as_ref().err());
        if let Ok(solid) = result {
            assert!((solid.volume() - 23.0 * std::f64::consts::PI).abs() < 0.1);
            assert_eq!(super::solid::unpaired(solid.mesh()), 0);
        }
    }
}

#[test]
fn section_holes_keep_component_paths_in_named_and_inline_sections() {
    let e = read("\
unit mm
use std
component Bore(x: Length) {
c := point hint((x, 0))
bore := circle(center: c) hint(r: 1)
}
component Part() {
o := point
outer := circle(center: o) hint(r: 10)
left := Bore(x: -3mm)
right := Bore(x: 3mm)
section := face(outer, holes: left.bore, right.bore)
named := solid(section, depth: 2mm)
inline := solid(face(outer, holes: left.bore, right.bore), depth: 2mm)
}
in std.front {
p := Part()
}
");
    for name in ["p.named", "p.inline"] {
        let solid = e.sketch.evaluated_solid(e.map.ent_named(name).unwrap().i(),
            gcs_core::solid::ApproximationPolicy::Report).unwrap();
        for side in ["outer", "left.bore", "right.bore"] {
            assert!(solid.surviving_faces().contains(&format!("{name}.{side}")), "{:?}", solid.surviving_faces());
        }
        assert!((solid.volume() - 196.0 * std::f64::consts::PI).abs() < 0.6);
    }
}

#[test]
fn section_holes_keep_repeat_paths_stable_when_statements_are_inserted() {
    let source = "\
unit mm
use std
in std.front {
center := point
outer := circle(center: center) hint(r: 10)
repeat 2 as i {
c := point hint(((i * 6 - 3) * 1mm, 0))
bore := circle(center: c) hint(r: 1)
}
}
sleeve := solid(face(outer, holes: bore[0], bore[1]), depth: 2mm)
";
    for src in [source.to_string(), format!("unrelated := point\n{source}")] {
        let e = read(&src);
        let solid = e.sketch.evaluated_solid(e.map.ent_named("sleeve").unwrap().i(),
            gcs_core::solid::ApproximationPolicy::Report).unwrap();
        for name in ["sleeve.outer", "sleeve.bore[0]", "sleeve.bore[1]"] {
            assert!(solid.surviving_faces().contains(name), "{:?}", solid.surviving_faces());
        }
    }
}

#[test]
fn section_holes_preserve_explicit_closure_winding_and_failed_face_cleanup() {
    let (p, errors) = parse(&format!("{RECT}{HOLE}\n\
        bad := face(a,b,c,d, holes: ab, -> close)\n\
        good := face(d,c,b,a, holes: hole, -> close)\nslab := solid(good, depth: 2mm)\n"));
    assert!(errors.is_empty());
    let e = elaborate(&p);
    assert!(e.diags.iter().any(|d| d.code == Code::E080));
    assert_eq!(e.sketch.faces.len(), 3, "a failed face leaves no intermediate boundaries");
    assert_eq!(e.sketch.lines.len(), 8, "only the good face retains its four closing edges");
    let expected = (2400.0 - 25.0 * std::f64::consts::PI) * 2.0;
    assert!((volume(&e, "slab") - expected).abs() < 0.1);
    let src = format!("{RECT}{HOLE}\nslab := solid(face(a,b,c,d, holes: hole, -> close), depth: 2mm)\n");
    let (mut p, _) = parse(&src);
    let printed = gcs_core::syntax::render_flat(&mut p).unwrap().to_string();
    assert!((volume(&read(&printed), "slab") - volume(&e, "slab")).abs() < 1e-8);
}

#[test]
fn vtwin_intake_passages_connect_without_opening_into_the_exhaust() {
    use gcs_core::{clear, constraints::SolidWord, solid::{ApproximationPolicy, WorldPoint}};
    let (p, errors, linked) = gcs_core::library::parse_linked(gcs_core::examples::VTWIN_PLATE);
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}");
    let mut e = elaborate(&p);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let solid = |name: &str| e.sketch.evaluated_solid(
        e.map.ent_named(&format!("plate.{name}")).unwrap().i(),
        ApproximationPolicy::View { unit: 0.02 }).unwrap();
    let intake = ["air.plenum", "air.feedL.body", "air.feedR.body", "inlet.passage_s", "portLi", "portRi"];
    let exhaust = ["portLe", "portRe", "ventL.body", "ventR.body"];
    for a in intake {
        for b in exhaust {
            let verdict = clear::judge_evaluated(SolidWord::Clear, &solid(a), &solid(b), 3.9);
            assert_eq!(verdict.holds(), Some(true), "{a} versus {b}: {verdict:?}");
        }
    }

    // Each radial feed overlaps its port and the plenum by a finite area.
    // The inlet also reaches inside the arc, rather than merely touching its outside.
    let point = |name: &str| e.sketch.point_xy(e.map.ent_named(&format!("plate.{name}")).unwrap().i());
    let plenum = solid("air.plenum");
    let outer = point("air.co1");
    let rman = outer.0.hypot(outer.1) - 2.0;
    for (bank, feed, port, side) in [("l", "air.feedL.body", "portLi", -1.0), ("r", "air.feedR.body", "portRi", 1.0)] {
        let p = point(&format!("{bank}.ip.p"));
        let radius = p.0.hypot(p.1);
        let (c, s) = (p.0 / radius, p.1 / radius);
        let shared = WorldPoint([c * (rman + 0.5) - s * side * 0.5, 0.0,
            s * (rman + 0.5) + c * side * 0.5]);
        assert!(plenum.contains_world(shared) && solid(feed).contains_world(shared), "{feed} meets plenum");
        let shared = WorldPoint([p.0 - c * 0.5, 0.0, p.1 - s * 0.5]);
        assert!(solid(feed).contains_world(shared) && solid(port).contains_world(shared), "{feed} meets port");
    }
    let shared = WorldPoint([0.0, 0.0, rman + 1.0]);
    assert!(plenum.contains_world(shared) && solid("inlet.passage_s").contains_world(shared));
}

#[test]
fn throttle_revolution_matches_the_extruded_design_in_both_placements() {
    use gcs_core::solid::{ApproximationPolicy, WorldPoint};
    for (phi, height, page_x, page_y, offset) in [(0, 0, 0, 0, 0), (35, 72, 100, -80, 7)] {
        let src = format!("unit mm\nuse std\nuse hardware\nuse components.dims\nuse components.parts\nuse components.throttle\n\
            torgb := 2 * components.dims.rbar - 2 * (1 - hardware.oring_squeeze) * components.dims.tor\n\
            torw := hardware.oring_groove_w * components.dims.tor\n\
            front := plane hint(origin: ({page_x}, {back}, {page_y}))\n\
            fix(origin == ({page_x}, {back}, {page_y})) front\n\
            fix(dir == (1, 0, 0)) front.u\n\
            fix(dir == (0, 0, 1)) front.v\n\
            in front {{\nO := point\nfix((0, 0)) O\nc := components.parts.At(O, dx: 0mm, dy: {height}mm)\naxes := components.parts.Axes(O)\n\
            core := circle(center: c.p) hint(r: torgb / 2)\nradius(torgb / 2) core\n}}\n\
            thr := components.throttle.Throttle(front, c.p, axes.ax, phi: {phi}deg, dims: components.dims.vtwin_dims)\n\
            old_barrel := solid(face(thr.barrel), from: -(components.dims.bossz / 2 + components.dims.tback), to: components.dims.bossz / 2)\n\
            old_hub := solid(face(thr.hub), from: components.dims.bossz / 2, to: components.dims.bossz / 2 + components.dims.levw)\n\
            groove_section := face(thr.barrel, holes: core)\n\
            groove0 := solid(groove_section, from: components.dims.torz - torw / 2, to: components.dims.torz + torw / 2)\n\
            groove1 := solid(groove_section, from: -components.dims.torz - torw / 2, to: -components.dims.torz + torw / 2)\n\
            groove2 := solid(groove_section, from: -(components.dims.bossz / 2 + components.dims.tretain) - torw / 2, to: -(components.dims.bossz / 2 + components.dims.tretain) + torw / 2)\n\
            reference := solid(old_barrel)\nold_hub union reference\nthr.arm union reference\nthr.knob_s union reference\n\
            thr.cross cut reference\ngroove0 cut reference\ngroove1 cut reference\ngroove2 cut reference\n",
            back = -offset);
        let (p, errors, linked) = gcs_core::library::parse_linked(&src);
        assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}");
        let mut e = elaborate(&p);
        assert!(e.ok(), "{:?}", e.diags);
        assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
        let policy = ApproximationPolicy::View { unit: 0.02 };
        let new = e.sketch.evaluated_solid(e.map.ent_named("thr.body").unwrap().i(), policy).unwrap();
        let old = e.sketch.evaluated_solid(e.map.ent_named("reference").unwrap().i(), policy).unwrap();
        assert!((new.volume() / old.volume() - 1.0).abs() < 0.002,
            "phi={phi}: revolved {} versus extruded {}", new.volume(), old.volume());
        let mesh = e.sketch.evaluated_solid(e.map.ent_named("thr.body").unwrap().i(), ApproximationPolicy::Mesh).unwrap();
        assert_eq!(super::solid::unpaired(new.mesh()), 0, "phi={phi}: view mesh closes");
        assert_eq!(super::solid::unpaired(mesh.mesh()), 0, "phi={phi}: export mesh closes");
        let (a, b) = (new.world_bounds(), old.world_bounds());
        for k in 0..3 {
            assert!((a.lo[k] - b.lo[k]).abs() < 0.03 && (a.hi[k] - b.hi[k]).abs() < 0.03,
                "phi={phi}: {a:?} versus {b:?}");
        }
        let fi = e.map.ent_named("front").unwrap().i();
        let basis = e.sketch.basis(fi);
        let uv = e.sketch.point_xy(e.map.ent_named("c.p").unwrap().i());
        let normal = basis.normal();
        for x in [-4.5, -3.0, 0.0, 3.0, 4.5] {
            for y in [-4.5, -3.0, 0.0, 3.0, 4.5, 12.0, 22.0] {
                for z in [-13.5, -11.5, -8.0, -5.5, 0.0, 5.5, 8.0, 12.0] {
                    let p = basis.lift(uv.0 + x, uv.1 + y);
                    let p = WorldPoint(std::array::from_fn(|k| p[k] + z * normal[k]));
                    assert_eq!(new.contains_world(p), old.contains_world(p), "phi={phi}, {p:?}");
                }
            }
        }
        assert_eq!(e.sketch.solids.iter().filter(|s| s.name.starts_with("thr.")).count(), 5);
    }
}

/// A parabola `y := x²` about `o`, as a formula curve, with `a` and `b` held on it at `x = ∓1`.
const PARABOLA: &str = "\
unit mm
use std
component Par(o: point, u: Length) {
  p := point(x: o.x + u, y: o.y + u * u / 1mm)
}
in std.front {
o := point
fix((0mm, 0mm)) o
}
k := Par(o).p over u in (-2mm, 2mm)
in std.front {
a := point hint(y: 1mm)
b := point hint(y: 1mm)
c := point
fix((0mm, 3mm)) c
a coincident k hint(t: -1)
b coincident k hint(t: 1)
fix(x == -1mm) a
fix(x == 1mm) b
}
";

#[test]
fn a_face_runs_along_the_stretch_of_a_curve_between_two_points_held_on_it() {
    // the parabola from a to b and the chord back: 2 − 2/3 across, 3 deep
    let e = read(&format!("{PARABOLA}s := solid(face(k from a to b, -> close), depth: 3mm)\n"));
    let v = volume(&e, "s");
    assert!((v - 4.0).abs() < 2e-3, "{v}");
    // the exact kernel reads the stretch as a B-spline fitted within `FIT_MM` of the curve
    let mut sk = e.sketch.clone();
    gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
    let root = (0..sk.solids.len()).find(|&i| sk.solids[i].name == "s").unwrap();
    let recipe = gcs_core::solid::cad::recipe(&sk, root).unwrap();
    let b = gcs_core::brep::recipe::build(&recipe).unwrap();
    b.check(1e-9).unwrap();
    let exact = gcs_core::brep::props::volume(&b);
    assert!((exact - 4.0).abs() <= 3.0 * 2.0 * 3.0 * gcs_core::solid::cad::FIT_MM, "{exact}");
    // either way round, and printed as written
    let back = read(&format!("{PARABOLA}s := solid(face(k from b to a, -> close), depth: 3mm)\n"));
    assert!((volume(&back, "s") - 4.0).abs() < 2e-3);
    let (prog, _) = parse("f := face(k from a to b, -> close)\n");
    let mut text = String::new();
    gcs_core::syntax::write_stmt_to(&mut text, &prog.stmts().next().unwrap().kind).unwrap();
    assert!(text.contains("f := face(k from a to b, -> close)"), "{text}");
}

#[test]
fn a_curve_in_a_face_is_a_stretch_between_points_held_on_it() {
    refused(&format!("{PARABOLA}f := face(k, -> close)\n"), Code::E080, "name the stretch");
    refused(&format!("{PARABOLA}f := face(k from a to c, -> close)\n"), Code::E080, "`c` is not held on `k`");
    refused(&format!("{PARABOLA}f := face(k from a to a, -> close)\n"), Code::E080, "to itself");
    refused(&format!("{PARABOLA}l := line(a, c)\nf := face(l from a to c, -> close)\n"), Code::E080, "runs along a curve");
    refused(&format!("{PARABOLA}f := face(k from a to b, k from b to a)\n"), Code::E080, "twice");
    // and a face refused after minting one stretch leaves no stretch behind
    let (prog, _) = parse(&format!("{PARABOLA}f := face(k from a to b, k from b to a)\n"));
    assert_eq!(elaborate(&prog).sketch.curves.len(), 1);
}
