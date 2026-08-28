//! Presentation, and the line between it and what the drawing *is*.
//!
//! A class on a declaration and a `style` block that says what the class looks like are the
//! whole of it, and the property under test is mostly a *negative* one: changing how a drawing
//! is presented changes nothing the core computes.  Same solve, same DOF, same diagnosis.

use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::model::{EntKind, EntRef, Sketch};
use gcs_core::program::elaborate;
use gcs_core::style::Classes;
use gcs_core::syntax::parse;

fn read(src: &str) -> Sketch {
    let (p, errs) = parse(src);
    assert!(errs.is_empty(), "{src}: {errs:?}");
    let e = elaborate(&p);
    assert!(e.ok(), "{src}: {:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    e.sketch
}

const PLAIN: &str = "\
point a hint(x: 0, y: 0)
point b hint(x: 60, y: 0)
line ab(a, b) class construction
a distance(60) b
horizontal ab
ground a
";

/// `class construction` draws exactly as the retired keyword did — from the base sheet, which is
/// the one rule the implementation ships and a document may override.
#[test]
fn the_base_sheet_is_what_makes_construction_dashed() {
    let sk = read(PLAIN);
    let l = EntRef::new(EntKind::Line, 0);
    assert!(sk.class_of(l).has("construction"));
    assert_eq!(sk.style_of(l).dash, Some(vec![7.0, 4.0]));
    assert_eq!(sk.style_of(l).width, None, "the base sheet says nothing else");
    assert_eq!(sk.style_of(l).color, None);
}

/// **A document that overrides `.construction` changes how it draws and nothing else.**
///
/// Same drawing, same freedoms, same verdict — and a JSON export that differs in nothing, since
/// a sheet is not a fact about any entity.
#[test]
fn overriding_a_class_changes_only_how_it_draws() {
    let styled =
        format!("style .construction {{ dash: 2 2; width: 0.5; color: #888888 }}\n{PLAIN}");
    let mut a = read(PLAIN);
    let mut b = read(&styled);
    assert_eq!(gcs_core::io::dumps(&a, Some(1)), gcs_core::io::dumps(&b, Some(1)));
    let opts = DiagnoseOptions::default();
    let (da, db) = (diagnose(&mut a, opts), diagnose(&mut b, opts));
    assert_eq!(da.dof, db.dof);
    assert_eq!(da.status, db.status);
    assert_eq!(da.status, State::Well);

    let l = EntRef::new(EntKind::Line, 0);
    let s = b.style_of(l);
    assert_eq!(s.dash, Some(vec![2.0, 2.0]), "the document's rule wins over the base sheet");
    assert_eq!(s.width, Some(0.5));
    assert_eq!(s.color.as_deref(), Some("#888888"));
}

/// Several classes cascade, later over earlier, and only on the properties the later one states.
#[test]
fn a_later_class_wins_only_what_it_says() {
    let src = "\
style .centerline { dash: 12 3 2 3; width: 0.5; color: #888888 }
style .heavy      { width: 2.5 }
point a hint(x: 0, y: 0)
point b hint(x: 60, y: 0)
line ab(a, b) class centerline heavy
";
    let sk = read(src);
    let s = sk.style_of(EntRef::new(EntKind::Line, 0));
    assert_eq!(s.width, Some(2.5), "a centreline drawn thick");
    assert_eq!(s.dash, Some(vec![12.0, 3.0, 2.0, 3.0]), "and still a centreline");
    assert_eq!(s.color.as_deref(), Some("#888888"));
}

/// An unmatched class draws as plain geometry and is not a diagnostic — exactly as in CSS, which
/// is also what makes paste work: a figure keeps its class names and picks up whatever the
/// destination says about them, or nothing.
#[test]
fn an_unmatched_class_is_not_an_error() {
    let sk = read("point a hint(x: 0, y: 0)\npoint b hint(x: 1, y: 0)\nline ab(a, b) class nobody\n");
    let l = EntRef::new(EntKind::Line, 0);
    assert!(sk.class_of(l).has("nobody"));
    assert_eq!(sk.style_of(l), gcs_core::style::Style::default());
}

/// Copy and paste carry classes: a figure copied out of a document brings its presentation.
#[test]
fn copy_and_paste_carry_classes() {
    let sk = read(PLAIN);
    let clip = gcs_core::io::copy(&sk, &[EntRef::new(EntKind::Line, 0)]);
    assert!(clip.lines[0].class.has("construction"));
    let mut dst = Sketch::new();
    gcs_core::io::paste(&mut dst, &clip, 10.0, 10.0);
    assert!(dst.lines[0].class.has("construction"));
}

/// The one key JSON reads and never writes: an export from before there were classes still opens.
#[test]
fn an_old_export_still_loads() {
    let src = r#"{"version":1,"points":[{"x":0,"y":0},{"x":10,"y":0}],
                  "lines":[{"p1":0,"p2":1,"construction":true}],"constraints":[]}"#;
    let sk = gcs_core::io::loads(src).expect("an old export loads");
    assert_eq!(sk.lines[0].class, Classes::one("construction"));
    let out = gcs_core::io::dumps(&sk, None);
    assert!(!out.contains("\"construction\":"), "the old key is never written back: {out}");
    assert!(out.contains("\"class\""));
}

/// A style block round-trips through the printer, which is what keeps a hand-written sheet from
/// being reformatted into something else on the first edit.
#[test]
fn a_style_block_prints_back() {
    let src = "style .centerline { dash: 12 3 2 3; width: 0.5; color: #888888 }\n";
    let (p, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let mut out = String::new();
    gcs_core::syntax::write_stmt_to(&mut out, &p.root().body[0].kind);
    assert_eq!(out, "style .centerline { dash: 12 3 2 3; width: 0.5; color: #888888 }");
}

/* -- a callout's placement (spec §13.1) ---------------------------------------------------- */

/// **A placement stays on the statement it qualifies, and everything a callout *shares* is in
/// the sheet.**
///
/// That is the whole of the decision issue #16 asked for.  A class is a rule many statements
/// share; a placement is a fact about one.  The sheet owns the ink, the weight and the dash —
/// what every callout in a document has in common — and the statement keeps the one pair of
/// numbers that is about that statement alone.
#[test]
fn a_callouts_shared_presentation_is_in_the_sheet() {
    let sk = read(PLAIN);
    // the three rules a callout is drawn with, and nothing in them about *where* one sits.  A
    // claimed dimension is drawn with two of them: it *is* a dimension, and `.reference` says
    // the one thing that differs, so it takes the shared weight and its own lighter ink.
    assert_eq!(sk.style_named("dimension").color.as_deref(), Some("#0f6f7a"));
    assert_eq!(sk.style_named("dimension reference").color.as_deref(), Some("#7aa7ad"));
    assert_eq!(sk.style_named("dimension reference").width, sk.style_named("dimension").width);
    assert_eq!(sk.style_named("extension").dash, Some(vec![4.0, 3.0]));

    // and a document may say otherwise, changing nothing about what the drawing is
    let styled = format!("style .dimension {{ color: #b00020; width: 2 }}\n{PLAIN}");
    let b = read(&styled);
    assert_eq!(b.style_named("dimension").color.as_deref(), Some("#b00020"));
    assert_eq!(b.style_named("dimension").width, Some(2.0));
    assert_eq!(gcs_core::io::dumps(&read(PLAIN), Some(1)), gcs_core::io::dumps(&b, Some(1)));
}

/// **What a document says beats what the implementation ships, whichever class it is written
/// on.**  The base sheet is a layer *under* the document's, not a rule interleaved between its
/// classes — so one `style .dimension` recolours every callout, and not the half of them that
/// are not also `.reference`.
#[test]
fn a_document_rule_beats_a_shipped_one_on_a_later_class() {
    let b = read(&format!("style .dimension {{ color: #b00020; width: 2 }}\n{PLAIN}"));
    let claimed = b.style_named("dimension reference");
    assert_eq!(claimed.color.as_deref(), Some("#b00020"), "a stated colour reaches a claim too");
    assert_eq!(claimed.width, Some(2.0));
    // and the document may still say how a reference dimension differs
    let sheet = "style .dimension { color: #b00020 }\nstyle .reference { color: #e5989b }\n";
    let c = read(&format!("{sheet}{PLAIN}"));
    assert_eq!(c.style_named("dimension").color.as_deref(), Some("#b00020"));
    assert_eq!(c.style_named("dimension reference").color.as_deref(), Some("#e5989b"));
}

/// **Both front ends ask the same thing of a claimed callout.**  `svg::render` is the second one
/// that strokes a sheet, so the rule that a reference dimension is drawn `dimension reference`
/// is pinned here too: written once per front end, it is the pair that drifts.
#[test]
fn an_svg_export_draws_a_claimed_dimension_in_the_documents_ink() {
    let src = "\
style .dimension { color: #b00020 }
point a hint(x: 0, y: 0)
point b hint(x: 60, y: 0)
ground a
line ab(a, b)
horizontal ab
a distance(60) b at (10, -20)
claim a distance(60, along: x) b at (10, 20)
";
    let out = gcs_core::svg::render(&read(src), 400.0);
    assert!(out.contains("(60)"), "the claim is drawn as a reference dimension:\n{out}");
    assert!(!out.contains("#7aa7ad"), "and takes the document's ink, not the shipped one:\n{out}");
}

/// A property whose value the sheet cannot read is **dropped**, exactly as a property it does
/// not know is.  `color:` with nothing after it stored an empty string, which is not nullish and
/// so travelled to `ctx.fillStyle` — where an unparseable assignment is ignored and leaves the
/// previous colour standing, drawing every dimension's label in the background colour.
#[test]
fn a_property_with_no_value_says_nothing() {
    let sk = read(&format!("style .dimension {{ color: ; width: }}\n{PLAIN}"));
    let ink = sk.style_named("dimension");
    assert_eq!(ink.color.as_deref(), Some("#0f6f7a"), "the base ink stands");
    assert_eq!(ink.width, Some(1.0));
    // `dash` is the one that reads an empty list: absent or empty is solid, so it states solid
    let d = read(&format!("style .construction {{ dash: }}\n{PLAIN}"));
    assert_eq!(d.style_named("construction").dash, Some(vec![]));
}

/// Inserting or deleting a statement above a dimension does not move its callout — the failure
/// §13.1 names, and the reason a placement is keyed by neither position nor entity index.
#[test]
fn a_statement_inserted_above_does_not_move_a_callout() {
    let src = "\
point a hint(x: 0, y: 0)
point b hint(x: 60, y: 0)
a distance(60) b at (12, -4)
a horizontal b at (3, 5)
";
    let before = read(src);
    let after = read(&src.replace("point a hint", "point z hint(x: 9, y: 9)\npoint a hint"));
    let places = |sk: &Sketch| -> Vec<(f64, f64)> {
        sk.user_constraints().iter().filter_map(|c| sk.placements.get(&c.id).copied()).collect()
    };
    assert_eq!(places(&before), vec![(12.0, -4.0), (3.0, 5.0)]);
    assert_eq!(places(&after), places(&before), "a callout did not follow a position");
}

/// A placement whose dimension is gone is **gone** — never silently inert while the document
/// still carries it.  It rides on the statement, so deleting the statement takes it; and in the
/// sketch, `Sketch::remove` drops it with the constraint.
#[test]
fn a_placement_dies_with_its_dimension() {
    let mut sk = read("point a hint(x: 0, y: 0)\npoint b hint(x: 60, y: 0)\na distance(60) b at (12, -4)\n");
    let id = sk.user_constraints()[0].id;
    assert_eq!(sk.placements.get(&id).copied(), Some((12.0, -4.0)));
    sk.remove(id);
    assert!(sk.placements.is_empty(), "a placement outlived the dimension it qualified");
    assert!(!gcs_core::io::dumps(&sk, None).contains("place"));
}

/// Copying a figure brings its callouts, and pasting it twice gives two sets.
#[test]
fn copying_a_figure_brings_its_callouts() {
    let sk = read("point a hint(x: 0, y: 0)\npoint b hint(x: 60, y: 0)\na distance(60) b at (12, -4)\n");
    let clip = gcs_core::io::copy(&sk, &[EntRef::point(0), EntRef::point(1)]);
    assert_eq!(clip.placements.len(), 1, "the callout came with the figure");
    let mut dst = Sketch::new();
    gcs_core::io::paste(&mut dst, &clip, 0.0, 0.0);
    gcs_core::io::paste(&mut dst, &clip, 100.0, 0.0);
    assert_eq!(dst.placements.len(), 2, "two pastes, two sets");
}
