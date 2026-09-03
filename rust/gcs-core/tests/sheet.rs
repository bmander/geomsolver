//! **The sheet is a report** (Solvent §6.12) — issue #48, item 10.
//!
//! Half the edits to every part sheet were `at (t, r)` placements, moving callouts off each other
//! by trial and then rendering to see whether they had landed.  The human needs the picture; the
//! machine should produce it.
//!
//! What is under test is the boundary as much as the feature: a machine generates the dimensions
//! that **follow from the object** — the extents, and the diameter of a round feature seen square
//! on — and leaves the ones that are a *design* decision to whoever is designing.  A test that
//! asserted a generated sheet was a finished sheet would be asserting something untrue.

use gcs_core::callout;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::parse;

/// Elaborated **and solved** — which is not a convenience here but the point: a generated
/// dimension is a reading of the drawing, so it says what the geometry came to and not what a
/// statement asked for.  Read off an unsolved pose it would say where the seeds happened to be.
fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let mut e = elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}\n{src}",
        e.errors().map(|d| d.message.clone()).collect::<Vec<_>>()
    );
    let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "does not solve\n{src}");
    e
}

/// A 60 × 40 plate with a Ø16 hole through it, and a side view beside the front.
const PART: &str = "\
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
plane front(origin: a, toward: b)
point o hint(x: 30, y: 20)
a distance(30, along: x) o
a distance(20, along: y) o
circle hole(center: o) hint(r: 8)
radius(8) hole
face hole_f(hole)
solid stock(sec, depth: 30mm)
solid bore(hole_f, depth: 30mm)
solid body(stock)
bore through body
point p2 hint(x: 110, y: 0)
point q2 hint(x: 150, y: 0)
plane side(origin: p2, toward: q2, from: front, fold: -90deg)
ground p2
p2 distance(40, along: x) q2
p2 distance(0, along: y) q2
";

/// The generated callouts alone — the ones past `callout::GENERATED`.
fn made(e: &Elaborated, unit: f64) -> Vec<callout::Callout> {
    callout::layout(&e.sketch, unit)
        .into_iter()
        .filter(|c| c.id >= callout::GENERATED)
        .collect()
}

#[test]
fn a_sheet_can_ask_for_its_own_dimensions() {
    let e = read(&format!("{PART}dimensions(body) in front\n"));
    let cs = made(&e, 0.05);
    let texts: Vec<&str> = cs.iter().map(|c| c.text.as_str()).collect();
    // the two extents, and the hole this view sees square on
    assert!(texts.contains(&"60"), "its width: {texts:?}");
    assert!(texts.contains(&"40"), "its height: {texts:?}");
    assert!(texts.contains(&"⌀16"), "and the bore, as a diameter: {texts:?}");
    assert_eq!(cs.len(), 3, "and nothing a machine would be guessing at: {texts:?}");
}

#[test]
fn a_view_dimensions_what_that_view_sees() {
    // the side view sees the part's depth, and sees the bore edge-on — so it says 40 × 30 and
    // does not invent a diameter for a circle that is not a circle from there
    let e = read(&format!("{PART}dimensions(body) in side\n"));
    let cs = made(&e, 0.05);
    let texts: Vec<&str> = cs.iter().map(|c| c.text.as_str()).collect();
    assert!(texts.contains(&"40") && texts.contains(&"30"), "the side view's own two: {texts:?}");
    assert!(!texts.iter().any(|t| t.starts_with('⌀')), "no diameter seen edge-on: {texts:?}");
}

#[test]
fn nothing_is_placed_by_hand_and_nothing_lands_on_the_part() {
    // **the engine that already lays out every stated dimension lays these out too**, which is
    // the whole of "never placed by the LLM": an extent stands off the outline it measures
    let e = read(&format!("{PART}dimensions(body) in front\n"));
    let cs = made(&e, 0.05);
    let (x0, y0, x1, y1) = e.sketch.drawn_bounds();
    for c in &cs {
        if c.text.starts_with('⌀') {
            continue;   // a diameter is taken across its own circle and belongs there
        }
        let inside = c.label.iter().all(|p| p.0 > x0 && p.0 < x1 && p.1 > y0 && p.1 < y1);
        assert!(!inside, "`{}` was drawn through the part it measures", c.text);
    }
    // and the document said nothing about where any of them go
    assert!(e.sketch.placements.is_empty(), "no `at (t, r)` anywhere");
}

#[test]
fn a_generated_dimension_is_a_reading_and_not_a_statement() {
    // it adds no equation, no unknown and no freedom — the property that makes a sheet a
    // *report* rather than a second document to keep in step
    let plain = read(PART);
    let sheet = read(&format!("{PART}dimensions(body) in front\n"));
    assert_eq!(plain.sketch.params.len(), sheet.sketch.params.len());
    assert_eq!(plain.sketch.constraints.len(), sheet.sketch.constraints.len());
    // its id is past any constraint, so a front end resolving one back to a statement finds
    // nothing there — which is the truth
    for c in made(&sheet, 0.05) {
        assert!(sheet.sketch.constraint(c.id).is_none(), "no statement stands behind it");
    }
}

#[test]
fn the_sheet_follows_the_drawing() {
    // an edit to the design moves the generated dimensions, because they were never numbers
    // anybody wrote down
    let wider = PART.replace("a distance(60) b", "a distance(75) b");
    let e = read(&format!("{wider}dimensions(body) in front\n"));
    let texts: Vec<String> = made(&e, 0.05).into_iter().map(|c| c.text).collect();
    assert!(texts.iter().any(|t| t == "75"), "the sheet says what the part is: {texts:?}");
}
