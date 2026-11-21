//! **A hole's rim on a face in pieces of one surface** (#994's first item, fixture 101): a plate
//! 100 × 100 × 20 with a post of radius 15 standing on it, the post's foot flush with the plate's
//! bottom, a hole of radius 6.5 through both, its rims chamfered 1.5 or rounded 1.5.  Where the post
//! crosses the plate's edge (at x = 50) the bottom is two faces of one plane, `stock.far` and
//! `post.far`, each named by its own operand, and the hole's bottom rim lies on both (its centre at
//! 45 or 55) or on one beside the other (at 40, the post's piece clear of the rim; at 50, the
//! rim halved).  A flat or a ball standing against one of them is as far from the other: a face
//! the same surface goes on into is no other face of the chain (`lowering::going_on`), and
//! `piece::stands_within` read it as one, refusing the chamfer "within 0mm of `shell.post.far`".
//! Each is the closed form's to 1e-9: the plate and the post, less the hole, less each rim's
//! section turned about the hole's axis (Pappus); with the fillet of 2.5 where the post meets the
//! plate's side, its section along the plate's 20 either side.
use std::f64::consts::PI;

use gcs_core::brep::props;

use crate::common::rounds::{exported, inset, line_beside_circle, square_run};
use crate::common::{close, ent, text::solved};

/// The plate with the post about `(x, 0)`, the hole through both, and `words` (each a statement
/// over `part`) after it.
fn plate(x: f64, words: &[&str]) -> String {
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (cut, union)\nuse std.rounds (round, chamfer)\n\
        in std.top {{\n  a := point\n  b := point\n  c := point\n  d := point\n  fix((-50, -50)) a\n  \
        fix((50, -50)) b\n  fix((50, 50)) c\n  fix((-50, 50)) d\n  s0 := line(a, b)\n  s1 := line(b, c)\n  \
        s2 := line(c, d)\n  s3 := line(d, a)\n  plate_f := face(s0, s1, s2, s3)\n  o := point\n  \
        fix(({x}, 0)) o\n  bc := circle(center: o) hint(r: 15)\n  radius(15mm) bc\n  \
        hc := circle(center: o) hint(r: 6.5)\n  radius(6.5mm) hc\n}}\n\
        stock := solid(std.solids.Prism(plate_f, from: 0mm, to: 20mm))\n\
        post := solid(std.solids.Prism(face(bc), from: 0mm, to: 30mm))\n\
        shell := solid(stock)\npost union shell\n\
        cutter := solid(std.solids.Prism(face(hc), from: -1mm, to: 31mm))\n\
        part := solid(shell)\ncutter cut part\n{}\n", words.join("\n"))
}

/// The four places of the issue's table: the post within the plate, across its edge, centred on
/// it, across it the other way.
const AT: [f64; 4] = [40.0, 45.0, 50.0, 55.0];

/// The part's closed form, untreated: the plate, the post above it and its piece past the plate's
/// edge (a segment of its disc, `d = 50 − x` off its centre, either side), less the hole.
fn part_volume(x: f64) -> f64 {
    let (big, d) = (15.0, 50.0 - x);
    let segment = big * big * (d / big).acos() - d * (big * big - d * d).sqrt();
    100.0 * 100.0 * 20.0 + PI * big * big * 10.0 + 20.0 * segment - PI * 6.5 * 6.5 * 30.0
}

/// What each rim of the hole loses: a chamfer's triangle `d` on a side, or a round's section,
/// turned about the axis at the hole's radius and its centroid's inset (Pappus).
fn rim(r: f64, flat: bool) -> f64 {
    if flat {
        2.0 * PI * (6.5 + r / 3.0) * r * r / 2.0
    } else {
        2.0 * PI * (6.5 + inset(r)) * square_run(r)
    }
}

/// **The issue's table**: the hole's rims chamfered 1.5 with the post at each place — refused
/// at 45 and 55 before, where the rim lies on both pieces of the bottom — each built and the closed
/// form's.
#[test]
fn a_hole_through_a_flush_post_is_chamfered_wherever_the_post_stands() {
    for x in AT {
        let b = exported(&solved(&plate(x, &["part.cutter chamfer(d: 1.5mm) part.shell"])), "part");
        close(props::volume(&b), part_volume(x) - 2.0 * rim(1.5, true));
    }
}

/// **The same rims rounded 1.5**, the ball against the bottom in pieces as the flat is.
#[test]
fn a_hole_through_a_flush_post_is_rounded_wherever_the_post_stands() {
    for x in AT {
        let b = exported(&solved(&plate(x, &["part.cutter round(r: 1.5mm) part.shell"])), "part");
        close(props::volume(&b), part_volume(x) - 2.0 * rim(1.5, false));
    }
}

/// **Fixture 101 whole**: the post's foot filled 2.5 where it meets the plate's side (the two
/// upright concave edges at x = 50, `w = 50 − x` off the post's axis, the fill cut square by the
/// bottom and the plate's top), then the hole's rims chamfered or rounded — each place of the
/// table, each the closed forms' sum.
#[test]
fn the_post_filled_at_the_plates_side_then_its_hole_chamfered() {
    for x in AT {
        let fill = 2.0 * 20.0 * line_beside_circle(50.0 - x, 15.0, 2.5);
        for (word, lost) in [("chamfer(d: 1.5mm)", rim(1.5, true)), ("round(r: 1.5mm)", rim(1.5, false))] {
            let src = plate(x, &["part.shell.post.bc round(r: 2.5mm) part.shell.stock.s1",
                &format!("part.cutter {word} part.shell")]);
            let e = solved(&src);
            assert!(e.diags.is_empty(), "{:?}", e.diags);
            let _ = ent(&e, "part");
            close(props::volume(&exported(&e, "part")), part_volume(x) + fill - 2.0 * lost);
        }
    }
}

/// **What the check is for still refused, with the top in pieces**: a pocket's floor rounded or
/// chamfered 8 against one wall alone (`tight_corners`'s), its plate two halves unioned flush, so
/// the plate's top — and the pocket's wall, cut across the seam — are pieces of one surface.  The
/// wall goes on smoothly into the corners' arcs and the walls beyond, a different surface: the
/// ball's centres stop 8 short of `e3`, 3 short of the edge's ends, refused naming it; rounded 5
/// the run is its section along the wall's 50, as on a plate of one piece.
#[test]
fn a_wall_alone_is_still_refused_where_its_ball_reaches_the_next_wall() {
    let tray = |word: &str| format!("unit mm\nuse std\nuse std.solids (cut, union)\nuse std.rounds (round, chamfer)\n\
        in std.top {{\n  a := point\n  b := point\n  c := point\n  d := point\n  m := point\n  n := point\n  \
        fix((-60, -40)) a\n  fix((0, -40)) m\n  fix((0, 40)) n\n  fix((60, -40)) b\n  fix((60, 40)) c\n  \
        fix((-60, 40)) d\n  left_f := face(a, m, n, d, -> close)\n  right_f := face(m, b, c, n, -> close)\n  \
        p0 := point\n  p1 := point\n  p2 := point\n  p3 := point\n  fix((-30, -20)) p0\n  fix((30, -20)) p1\n  \
        fix((30, 20)) p2\n  fix((-30, 20)) p3\n  (e0 := line(p0, p1)) -> (k1 := round(r: 5mm)) \
        (e1 := line(p1, p2)) -> (k2 := round(r: 5mm)) (e2 := line(p2, p3)) -> (k3 := round(r: 5mm)) \
        (e3 := line(p3, p0)) -> (k0 := round(r: 5mm)) close\n  pocket_f := face(e0, k1, e1, k2, e2, k3, e3, k0)\n}}\n\
        left := solid(std.solids.Prism(left_f, from: 0mm, to: 20mm))\n\
        right := solid(std.solids.Prism(right_f, from: 0mm, to: 20mm))\n\
        plate := solid(left)\nright union plate\n\
        pocket := solid(std.solids.Prism(pocket_f, from: 10mm, to: 21mm))\npocket cut plate\n\
        plate.pocket.far {word} plate.pocket.e0\n");
    let refused = |src: &str| {
        let e = solved(src);
        let say = gcs_core::brep::sweep::Say { stage: &|_: &str| {}, mark: &|_| {} };
        gcs_core::brep::export::exact(&e.sketch, ent(&e, "plate").i(), None, None, &say).err().expect("refused").message
    };
    let plain = 120.0 * 80.0 * 20.0 - 10.0 * (2400.0 - 25.0 * (4.0 - PI));
    close(props::volume(&exported(&solved(&tray("round(r: 5mm)")), "plate")), plain + 50.0 * square_run(5.0));
    close(props::volume(&exported(&solved(&tray("chamfer(d: 5mm)")), "plate")), plain + 50.0 * 12.5);
    let why = refused(&tray("round(r: 8mm)"));
    assert!(why.contains("its ball, of radius 8mm, rolled against `pocket.e0` comes within 5mm of `pocket.e3`"), "{why}");
    let why = refused(&tray("chamfer(d: 8mm)"));
    assert!(why.contains("its flat, reaching 8mm from `pocket.e0`, comes within 5mm of `pocket.e3`"), "{why}");
}
