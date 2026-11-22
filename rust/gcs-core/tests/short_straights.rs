//! **A body its seeds do not build is read where the drawing solves** (#994, fixture 112): a
//! slot whose corner rounds leave a straight of 0.05 between them, its far side held by a
//! dimension and seeded a hair short — as a drawing's hints are — so that, seeded from their
//! corners, the rounds' ends pass each other on the short side and the pocket builds nothing
//! where the drawing is seeded.  `when a meets b` read such a body's faces as every two meeting,
//! and a round over a whole body made a copy for every two and every three of its faces: the
//! expansion of a plate with three such slots ran past `MAX_FLAT` (E103, `std.rounds` reached),
//! and of one slot ran ten times as long as its drawing.  It is read where the drawing solves
//! (`flatten::Unread`, as a face held along a direction is, #948), so the words expand there as
//! they do seeded where the drawing stands, and the 0.05 straight is a face like any other:
//! built, and the closed form's volume.  A straight of none is no face, and is said so.
use std::f64::consts::PI;

use crate::common::lenient::read;
use crate::common::rounds::exported;
use crate::common::{close, text::solved};
use gcs_core::brep::props;

/// A plate 60 × 40 × 20 with a slot 10 deep cut into it: 20 long and `width` across, its corners
/// rounded 1.5 — its short sides `width − 3` straight — its far side held `width` from its near
/// one by dimensions and seeded at `seed`, and `word` after.
fn slot(width: f64, seed: f64, word: &str) -> String {
    format!("unit mm\nuse std\nuse std.solids (cut)\nuse std.rounds (round, chamfer)\n\
        in std.top {{\n  a := point\n  b := point\n  c := point\n  d := point\n\
        fix((-30, -20)) a\n  fix((30, -20)) b\n  fix((30, 20)) c\n  fix((-30, 20)) d\n\
        plate_f := face(a, b, c, d, -> close)\n\
        p0 := point\n  p1 := point\n  p2 := point hint((10, {seed}))\n  p3 := point hint((-10, {seed}))\n\
        fix((-10, 0)) p0\n  fix((10, 0)) p1\n\
        p1 distance({width}, along: up) p2\n  p0 distance({width}, along: up) p3\n\
        p1 distance(0, along: right) p2\n  p0 distance(0, along: right) p3\n\
        (e0 := line(p0, p1)) -> (k1 := round(r: 1.5mm)) (e1 := line(p1, p2)) -> \
        (k2 := round(r: 1.5mm)) (e2 := line(p2, p3)) -> (k3 := round(r: 1.5mm)) \
        (e3 := line(p3, p0)) -> (k0 := round(r: 1.5mm)) close\n\
        pocket_f := face(e0, k1, e1, k2, e2, k3, e3, k0)\n}}\n\
        plate := solid(std.solids.Prism(plate_f, from: 0mm, to: 20mm))\n\
        pocket := solid(std.solids.Prism(pocket_f, from: 10mm, to: 21mm))\n\
        pocket cut plate\n{word}\n")
}

/// The closed form: the plate less the slot, plus a floor round of `r` (`Some(r)`) — the square
/// `r` on a side less the ball's quarter disc along each wall, turned by Pappus through each
/// corner — or less a rim chamfer of `d` (`None` with `d`), its triangle along and round so.
fn volume(width: f64, floor: Option<f64>, d: f64) -> f64 {
    let big = 1.5;
    let area = 20.0 * width - 4.0 * big * big * (1.0 - PI / 4.0);
    let walls = 2.0 * (20.0 - 2.0 * big) + 2.0 * (width - 2.0 * big);
    let turn = PI / 2.0;
    let plate = 60.0 * 40.0 * 20.0 - 10.0 * area;
    match floor {
        Some(r) => {
            let a = big - r;
            let corner = turn * (r * (big * big - a * a) / 2.0 - a * PI * r * r / 4.0 - r * r * r / 3.0);
            plate + (1.0 - PI / 4.0) * r * r * walls + 4.0 * corner
        }
        None => plate - d * d / 2.0 * walls - 4.0 * turn * d * d / 2.0 * (big + d / 3.0),
    }
}

/// The slot's short sides 0.05, 0.01 and 0.5 straight, seeded where they stand and seeded short
/// so that the rounds' ends pass each other there: built every way, the floor rounded 0.5 and 1
/// and the rim chamfered, each the closed form's.
#[test]
fn a_short_straight_between_corner_rounds_is_a_face() {
    for (straight, seed) in [(0.05, 2.9), (0.05, 3.05), (0.01, 2.9), (0.5, 3.4)] {
        let width = 3.0 + straight;
        for (word, floor, d) in [
            ("plate.pocket.far round(r: 0.5mm) plate.pocket", Some(0.5), 0.0),
            ("plate.pocket.far round(r: 1mm) plate.pocket", Some(1.0), 0.0),
            ("plate.near chamfer(d: 0.5mm) plate.pocket", None, 0.5),
        ] {
            let e = solved(&slot(width, seed, word));
            let said = gcs_core::program::solid_diagnostics(&e.sketch, &e.map, &e.program);
            assert!(said.is_empty(), "{straight} seeded {seed}, {word}: {said:?}");
            close(props::volume(&exported(&e, "plate")), volume(width, floor, d));
        }
    }
}

/// How many solids the elaboration of `src` made — a copy of a round's words for each edge and
/// corner — with nothing refused.
fn solids(src: &str) -> usize {
    let (e, d) = read(src);
    assert!(d.is_empty(), "{d:?}");
    e.sketch.solids.len()
}

/// Every edge of the plate rounded, `round(r: 0.5mm) plate`: seeded short, the slot's faces
/// are read where it solves, so the words make the copies they make for the slot seeded where it
/// stands — not one for every two and three of its faces.
#[test]
fn a_body_seeded_unbuilt_expands_as_it_does_where_it_solves() {
    let word = "round(r: 0.5mm) plate";
    assert_eq!(solids(&slot(3.05, 2.9, word)), solids(&slot(3.05, 3.05, word)));
}

/// What the plate's slot `width` across seeded at `seed` and rounded all over says, its drawing
/// solved as far as it goes: what elaborating it says, then what building its solids does.
fn said(width: f64, seed: f64) -> Vec<String> {
    let (mut e, mut d) = read(&slot(width, seed, "round(r: 0.5mm) plate"));
    gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    let built = gcs_core::program::solid_diagnostics(&e.sketch, &e.map, &e.program);
    d.extend(built.iter().map(|d| format!("{} {}", d.code.as_str(), d.message)));
    assert!(!d.iter().any(|m| m.contains("statements once expanded")), "{d:?}");
    d
}

/// A straight of none is no face, nor one within the kernel's tolerance of none (1e-8 of 3): the
/// slot's face is refused once, at the face, naming the edge, and the round over the plate is
/// not built, built on it — no copy made for its faces, never a count of statements.
#[test]
fn a_straight_of_none_is_refused_by_name() {
    for (width, seed) in [(3.0, 3.0), (3.0 + 1e-8, 3.0 + 1e-8)] {
        let d = said(width, seed);
        assert!(d.iter().any(|m| m.starts_with("E080 `pocket_f`'s edge `e1` has no length")),
                "{width}: {d:?}");
        assert!(d.iter().any(|m| m.starts_with("N080 not built: it is built on")
            && m.contains("`pocket_f`")), "{width}: {d:?}");
    }
}
