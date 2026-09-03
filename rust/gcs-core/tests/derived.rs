//! **Views and sections, derived** (Solvent §6.11) — issue #48, item 9.
//!
//! The claim under test is the one the V-twin's cylinder makes the case for: a part is written
//! once, and every view of it is a *question*.  So the gate is not that a derived view matches
//! some pixels, it is that it matches **what a draughtsman would have drawn by hand in the same
//! plane** — which is a thing this document can state, because `project` already says where the
//! images of one point in two views must land.

use gcs_core::hidden;
use gcs_core::program::{elaborate, Elaborated};
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

/// A 60 × 40 rectangle grounded on the page, its face `sec`, and the front view it lies in.
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
plane front(origin: a, toward: b)
";

/// The right view, a hand's breadth to the side — `std`'s own `ThreeViews` fold.
const SIDE: &str = "\
point p2 hint(x: 100, y: 0)
point q2 hint(x: 140, y: 0)
plane side(origin: p2, toward: q2, from: front, fold: -90deg)
ground p2
p2 distance(40, along: x) q2
p2 distance(0, along: y) q2
";

const UNIT: f64 = 0.05;

fn strokes(e: &Elaborated) -> Vec<hidden::Drawn> {
    hidden::layout(&e.sketch, UNIT)
}

fn near(a: (f64, f64), b: (f64, f64), tol: f64) -> bool {
    (a.0 - b.0).hypot(a.1 - b.1) <= tol
}

/// Is the segment `(p, q)` covered by one of the drawn strokes, within `tol`?
fn covered(v: &[hidden::Drawn], p: (f64, f64), q: (f64, f64), tol: f64) -> bool {
    v.iter().any(|d| {
        d.pts.windows(2).any(|w| {
            (near(w[0], p, tol) && near(w[1], q, tol)) || (near(w[0], q, tol) && near(w[1], p, tol))
        })
    })
}

#[test]
fn a_view_in_the_plane_a_face_was_drawn_in_is_the_face_itself() {
    // **the strongest thing a derived view can be held to**: extrude a face along its own
    // normal and look at it square on, and the outline that comes back is the outline that was
    // drawn — the same four corners, at the same page coordinates.  Nothing here compares one
    // kernel against another; it compares the kernel against the drawing it was written over.
    let e = read(&format!("{RECT}solid block(sec, depth: 30mm)\nview(block) in front\n"));
    let v = strokes(&e);
    let tol = 1e-6;
    for (p, q) in [
        ((0.0, 0.0), (60.0, 0.0)),
        ((60.0, 0.0), (60.0, 40.0)),
        ((60.0, 40.0), (0.0, 40.0)),
        ((0.0, 40.0), (0.0, 0.0)),
    ] {
        assert!(covered(&v, p, q, tol), "the drawn edge {p:?}–{q:?} is in the derived view");
    }
    let seen: Vec<&hidden::Drawn> = v.iter().filter(|d| !d.hidden).collect();
    assert_eq!(seen.len(), 4, "four edges and no more: {:#?}", seen.iter().map(|d| &d.pts));
    assert!(!v.iter().any(|d| d.hidden), "nothing is behind anything, seen square on");
}

#[test]
fn a_bore_along_the_eye_is_two_hidden_lines_in_the_view_beside_it() {
    let src = format!(
        "{RECT}{SIDE}point o hint(x: 30, y: 20)\n\
         a distance(30, along: x) o\na distance(20, along: y) o\n\
         circle hole(center: o) hint(r: 8)\nradius(8) hole\nface hole_f(hole)\n\
         solid stock(sec, depth: 30mm)\nsolid bore(hole_f, depth: 30mm)\n\
         solid body(stock)\nbore through body\nview(body) in side\n"
    );
    let e = read(&src);
    let v = strokes(&e);
    assert!(v.iter().any(|d| d.hidden), "the bore's walls are behind the block's face");
    // the block's outline in the right view is 40 across (its height, folded) and 30 tall (its
    // depth), and every hidden line stands inside it
    let hidden_x: Vec<f64> =
        v.iter().filter(|d| d.hidden).flat_map(|d| d.pts.iter().map(|p| p.0)).collect();
    assert!(!hidden_x.is_empty());
    let lo = hidden_x.iter().cloned().fold(f64::INFINITY, f64::min);
    let hi = hidden_x.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    assert!(lo > 100.0 - 40.0 && hi < 100.0, "the bore is inside the block: {lo}..{hi}");
    // and it is 16 across, which is the bore's own diameter — the number the sheet never wrote
    // in this view and would have had to keep in step by hand
    assert!((hi - lo - 16.0).abs() < 0.2, "the bore reads its own diameter: {}", hi - lo);
}

#[test]
fn a_cylinder_is_two_lines_at_every_zoom() {
    // the draughtsman's rule, and the reason `smooth` exists: a round surface is drawn by its
    // silhouette, never by the facets the kernel happens to have cut it into
    let src = format!(
        "{RECT}{SIDE}point o hint(x: 30, y: 20)\n\
         a distance(30, along: x) o\na distance(20, along: y) o\n\
         circle rim(center: o) hint(r: 10)\nradius(10) rim\nface rf(rim)\n\
         solid cyl(rf, depth: 25mm)\nview(cyl) in side\n"
    );
    let e = read(&src);
    for unit in [2.0, 0.5, 0.05] {
        let v = hidden::layout(&e.sketch, unit);
        let sil = v.iter().filter(|d| d.silhouette).count();
        assert_eq!(sil, 2, "two silhouettes at unit {unit}, and got {sil}");
        // its two rims are seen edge-on and are one line each; nothing else is drawn
        assert!(v.len() <= 6, "a cylinder from the side is a rectangle, at unit {unit}: {}", v.len());
    }
}

#[test]
fn a_section_shows_the_cut() {
    let src = format!(
        "{RECT}point o hint(x: 30, y: 20)\n\
         a distance(30, along: x) o\na distance(20, along: y) o\n\
         circle hole(center: o) hint(r: 8)\nradius(8) hole\nface hole_f(hole)\n\
         solid stock(sec, depth: 30mm)\nsolid bore(hole_f, depth: 30mm)\n\
         solid body(stock)\nbore through body\nsection(body, at: front) in front\n"
    );
    let e = read(&src);
    let v = strokes(&e);
    assert!(!v.is_empty(), "a section of a bored block draws something");
    // the cut is at the block's own near face, so what it shows is the outline and the bore
    let xs: Vec<f64> = v.iter().flat_map(|d| d.pts.iter().map(|p| p.0)).collect();
    let lo = xs.iter().cloned().fold(f64::INFINITY, f64::min);
    let hi = xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    assert!(lo > -0.001 && hi < 60.001, "the cut is the size of the part: {lo}..{hi}");
}

#[test]
fn a_section_is_drawn_in_a_view_parallel_to_its_cut() {
    let src = format!(
        "{RECT}{SIDE}solid block(sec, depth: 30mm)\nsection(block, at: front) in side\n"
    );
    let (prog, errs) = parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    assert!(
        e.diags.iter().any(|d| d.code == gcs_core::program::Code::E084),
        "a cut across the view it is drawn in shows no true shape"
    );
}

#[test]
fn a_part_carries_no_views_and_a_sheet_asks_for_them() {
    // issue #48, item 9, as an accounting: the solid is the design, and each picture is one
    // statement.  No `Int` draw flag, no second copy of the geometry, no `project` to keep in
    // step — which is the sixty lines `vtwin/cylinder.sv` spent on its two extra views
    let e = read(&format!(
        "{RECT}{SIDE}solid block(sec, depth: 30mm)\nview(block) in front\nview(block) in side\n"
    ));
    assert_eq!(e.sketch.derived.len(), 2, "two pictures of one solid");
    assert_eq!(e.sketch.solids.len(), 1, "written once");
    let v = strokes(&e);
    assert!(v.iter().any(|d| d.pts.iter().any(|p| p.0 < 61.0)), "one where the design is");
    assert!(v.iter().any(|d| d.pts.iter().any(|p| p.0 > 61.0)), "one beside it");
}
