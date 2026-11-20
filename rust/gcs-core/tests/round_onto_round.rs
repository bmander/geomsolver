//! **A round onto a face that is itself a round** (#962's fifth item): a frame whose inside edge
//! is already filled `R`, a boss standing across it, and the boss rounded `r` into the frame.
//! Against the fill's concave face a ball of its own radius has its centre on the fill's axis
//! alone: where the boss's side crosses the fill on a circle about that axis (a side square to it)
//! the round there is a ring's stretch closed into a quarter ball, as a pocket's corner is
//! (`tight_corners`); anywhere else the ball stands still where the axis meets the side's offset,
//! and rounds the stretch of the meeting between the fill's two seams — the sphere's patch between
//! the sections through the stretch's ends (`fillet::dome`).  The run along the floor and the run
//! up the wall take on from those sections where the ball's contact stands on each seam there;
//! where it does not (a round boss off the fill's axis, a side skewed across it) the round is
//! refused so, as a rolled run's is.  Each volume is the closed form's to 1e-9: the L, the fill,
//! the boss outside them, and each round's section along its run or turned (Pappus), the dome a
//! quadrature of a smooth integrand.
use std::f64::consts::PI;

use gcs_core::brep::{export, props, sweep::Say};

use crate::common::{close, ent, text::solved};
use crate::common::rounds::{exported, kinds, root_ring, square_run};

/// A frame, an L 60 long and 40 tall, 10 thick each way, 40 deep along y (y from −40 to 0): its
/// inside edge, where the floor's top `z = 10` meets the wall's face `x = 10`, filled `big`; and a
/// boss, `boss` drawn in the top plane (its `foot`), from z = 5 to 25 — across the floor, the
/// fill and the wall — unioned with it and treated by `word` (`round(r: 2mm)`) into the frame.
fn framed(boss: &str, big: f64, word: &str) -> String {
    format!("unit mm\nuse std\nuse std.solids (union)\nuse std.rounds (round, chamfer)\n\
        in std.front {{\n  p0 := point\n  p1 := point\n  p2 := point\n  p3 := point\n  p4 := point\n  \
        p5 := point\n  fix((0, 0)) p0\n  fix((60, 0)) p1\n  fix((60, 10)) p2\n  fix((10, 10)) p3\n  \
        fix((10, 40)) p4\n  fix((0, 40)) p5\n  (base := line(p0, p1)) -> (end := line(p1, p2)) -> \
        (floor := line(p2, p3)) -> (wall := line(p3, p4)) -> (cap := line(p4, p5)) -> (back := line(p5, p0)) \
        -> close\n  section := face(base, end, floor, wall, cap, back)\n}}\n\
        in std.top {{\n{boss}}}\n\
        frame := solid(std.solids.Prism(section, from: 0mm, to: 40mm))\n\
        filled := frame.floor round(r: {big}mm) frame.wall\n\
        part := solid(frame)\n\
        boss := solid(std.solids.Prism(foot, from: 5mm, to: 25mm))\n\
        boss union part\n\
        part.boss {word} part.frame\n")
}

/// A round boss of radius `rb` about `(x, y)`.
fn round_boss(x: f64, y: f64, rb: f64) -> String {
    format!("  o := point\n  fix(({x}, {y})) o\n  rim := circle(center: o) hint(r: {rb})\n  radius({rb}mm) rim\n  \
        foot := face(rim)\n")
}

/// A plane section's area by Green's theorem, its boundary in turn: a segment to the next point,
/// or an arc about a centre through a turn (`Arc(centre, radius, from, to)`, radians).
enum Step { To([f64; 2]), Arc([f64; 2], f64, f64, f64) }

fn area(start: [f64; 2], steps: &[Step]) -> f64 {
    let mut at = start;
    let mut twice = 0.0;
    for s in steps {
        match *s {
            Step::To(q) => {
                twice += at[0] * q[1] - at[1] * q[0];
                at = q;
            }
            Step::Arc([a, b], rho, t0, t1) => {
                twice += rho * rho * (t1 - t0) + rho * a * (t1.sin() - t0.sin()) - rho * b * (t1.cos() - t0.cos());
                at = [a + rho * t1.cos(), b + rho * t1.sin()];
            }
        }
    }
    (0.5 * twice).abs()
}

/// `∫₀^h f`, by Gauss–Legendre on 64 nodes.
fn integral(h: f64, f: &dyn Fn(f64) -> f64) -> f64 {
    crate::common::rounds::gauss_legendre(64).into_iter().map(|(x, w)| 0.5 * h * w * f(0.5 * h * (x + 1.0))).sum()
}

/// What a still ball of radius `r` fills where a round boss of radius `rb` crosses a fill of the
/// same radius `r`, the boss's axis meeting the fill's: within the quarter disc of the fill's air
/// side (`s` toward the wall, `t` toward the floor, `s² + t² ≤ r²`), from the boss's side out to
/// the plane of the ball's circle, less the ball:
/// `(rb + r) π r² / 4 − ∫₀ʳ √(r² − s²) √(rb² − s²) ds − π r³ / 6`.
fn standing_dome(rb: f64, r: f64) -> f64 {
    // (s = r sin θ: smooth)
    let boss = integral(0.5 * PI, &|th| { let s = r * th.sin(); r * th.cos() * (rb * rb - s * s).sqrt() * r * th.cos() });
    (rb + r) * PI * r * r / 4.0 - boss - PI * r * r * r / 6.0
}

/// A boss whose foot is the quadrilateral `qs` (counter-clockwise).
fn quad_boss(qs: [[f64; 2]; 4]) -> String {
    let mut s = String::new();
    for (k, [x, y]) in qs.iter().enumerate() {
        s += &format!("  q{k} := point\n  fix(({x}, {y})) q{k}\n");
    }
    s + "  (b0 := line(q0, q1)) -> (b1 := line(q1, q2)) -> (b2 := line(q2, q3)) -> (b3 := line(q3, q0)) -> close\n  \
        foot := face(b0, b1, b2, b3)\n"
}

/// A block 60 × 10 (x, z), 40 deep, its top's far edge (`top` with `end`, at x = 60) rounded
/// convex `big` (`taken`), and a round boss of radius `rb` about `(x, −20)` from z = 2 to 20 across
/// it, treated by `word` into the block.
fn convex(x: f64, rb: f64, big: f64, word: &str) -> String {
    format!("unit mm\nuse std\nuse std.solids (union)\nuse std.rounds (round, chamfer)\n\
        in std.front {{\n  p0 := point\n  p1 := point\n  p2 := point\n  p3 := point\n  fix((0, 0)) p0\n  \
        fix((60, 0)) p1\n  fix((60, 10)) p2\n  fix((0, 10)) p3\n  (base := line(p0, p1)) -> (end := line(p1, p2)) -> \
        (top := line(p2, p3)) -> (back := line(p3, p0)) -> close\n  section := face(base, end, top, back)\n}}\n\
        in std.top {{\n{}}}\n\
        frame := solid(std.solids.Prism(section, from: 0mm, to: 40mm))\n\
        taken := frame.top round(r: {big}mm) frame.end\n\
        part := solid(frame)\n\
        boss := solid(std.solids.Prism(foot, from: 2mm, to: 20mm))\n\
        boss union part\n\
        part.boss {word} part.frame\n", round_boss(x, -20.0, rb))
}

/// Solid `part` refused, and why.
fn refused(src: &str) -> String {
    let e = solved(src);
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    export::exact(&e.sketch, ent(&e, "part").i(), None, None, &say).err().expect("refused").message
}

/// `framed`'s volume with a round boss of radius `rb` whose axis meets the fill's, at x = 10 + `r`,
/// the fill and the boss's round both `r`: the L and its fill; the boss from z = 10 to 25 beyond
/// the wall's face (the disc less its segment behind the wall); the round along the floor turned
/// half round the boss (Pappus); the ball standing at each side over the fill; the run up the wall
/// each side from the fill's seam (z = 10 + r) to the boss's top, its section between the wall,
/// the boss and the ball (Green's theorem); and the fill under the boss less, the run along the
/// boss's top against the wall plus — the same integral, so neither is written.
fn standing_volume(rb: f64, y: f64, r: f64) -> f64 {
    let x = 10.0 + r;
    let behind = rb * rb * (r / rb).acos() - r * (rb * rb - r * r).sqrt();
    let boss = 15.0 * (PI * rb * rb - behind);
    let k = (rb * rb - r * r).sqrt();
    // the wall's run's section: the crease, the wall's touch, the ball's arc to its touch on the
    // boss, the boss's arc back
    let section = area([10.0, y - k], &[
        Step::To([10.0, y - rb - r]),
        Step::Arc([x, y - rb - r], r, PI, 0.5 * PI),
        Step::Arc([x, y], rb, -0.5 * PI, (-k).atan2(-r)),
    ]);
    36000.0 + 40.0 * square_run(r) + boss + 0.5 * root_ring(rb, r) + 2.0 * standing_dome(rb, r)
        + 2.0 * (25.0 - x) * section
}

/// **A round boss standing across a filled edge, its axis meeting the fill's**: the boss rounded
/// to the fill's own radius stands a ball each side on the fill's axis, its dome over the stretch
/// where the boss's side crosses the fill — a sphere's patch, glued to the half ring along the
/// floor and the runs up the wall at the sections through the stretch's ends, where each ball's
/// contact stands on the fill's seams.  2 against a boss of 6; 1.5 against a boss of 4
/// moved along the frame; the issue's 0.5 and 2 against a boss of 3, 0.5 against 6; the two
/// statements written the other way round.
#[test]
fn a_round_boss_across_a_filled_edge_on_its_axis_stands_a_ball_each_side() {
    for (rb, y, r) in [(6.0, -20.0, 2.0), (4.0, -10.0, 1.5), (3.0, -20.0, 0.5), (3.0, -20.0, 2.0), (6.0, -20.0, 0.5)] {
        let src = framed(&round_boss(10.0 + r, y, rb), r, &format!("round(r: {r}mm)"));
        let b = exported(&solved(&src), "part");
        close(props::volume(&b), standing_volume(rb, y, r));
        assert_eq!(kinds(&b).get("sphere"), Some(&2), "{rb} {r}: {:?}", kinds(&b));
        // which statement comes first says nothing: `part` names the frame its fill made
        let (round, rest) = src.rsplit_once("part.boss").unwrap();
        let turned = round.replace("filled :=", &format!("part.boss{rest}filled :="));
        close(props::volume(&exported(&solved(&turned), "part")), standing_volume(rb, y, r));
    }
}

/// **A square boss across a filled edge** (the twin the lowering builds as a ring, `tight_corners`'
/// pocket corner): its sides square to the fill's axis cross it on circles about it, so each round
/// there is the ring's stretch turned a quarter about the axis and closed into a quarter ball.  The
/// boss runs out to the frame's end: the L, its fill, the boss beyond them, the floor's runs along
/// each side (48), the wall's up each (13) and a quarter ball each side, `π r³ / 12`; the fill
/// under the boss and the run along its top against the wall cancel.
#[test]
fn a_square_boss_across_a_filled_edge_is_a_quarter_ball_each_side() {
    let src = framed(&quad_boss([[5.0, -25.0], [60.0, -25.0], [60.0, -15.0], [5.0, -15.0]]), 2.0, "round(r: 2mm)");
    let b = exported(&solved(&src), "part");
    let a = square_run(2.0);
    close(props::volume(&b), 36000.0 + 40.0 * a + 7500.0 + 2.0 * 48.0 * a + 2.0 * 13.0 * a + 2.0 * PI * 8.0 / 12.0);
    assert_eq!(kinds(&b).get("sphere"), Some(&2), "{:?}", kinds(&b));
}

/// **A ball wider than the fill** never touches its face (no point stands its radius inside the
/// fill's concave side), and what none reaches runs along the fill without end, as round a pocket's
/// corner (`tight_corners`): refused, saying the ball's radius and the face's, and the runs along
/// the floor and up the wall each refused as coming nearer the wall (the floor) than the ball is
/// wide.  A millionth over, and a tenth.
#[test]
fn a_ball_wider_than_the_fill_is_refused_saying_both_radii() {
    for r in ["2.000001", "2.1"] {
        let why = refused(&framed(&round_boss(12.0, -20.0, 6.0), 2.0, &format!("round(r: {r}mm)")));
        assert!(why.contains(&format!("of radius {r}, is larger than `")) && why.contains("curved round it to a radius \
            of 2: no ball of that radius touches it"), "{why}");
    }
}

/// **What is not built yet is refused, saying why**: a round boss off the fill's axis, and a square
/// one skewed across it, whose balls' contacts cross the fill's seam with the floor in other
/// sections than the stretch's ends — the run along the floor would take on from another section
/// (its ball there does not stand within the floor's smooth surface either); the ball a hair under
/// the fill's radius, rolled round a spindle so short no section is carried past where the seams
/// cut it off; and the chamfer, whose flat closes the boss's side to a point over a stretch.
#[test]
fn a_round_onto_a_round_not_built_yet_is_refused_saying_why() {
    let seam = "runs on smoothly into `frame.floor`, but its contact crosses their seam in another section than its \
        edge does";
    for boss in [round_boss(14.0, -20.0, 6.0), quad_boss([[5.0, -25.0], [20.0, -23.0], [20.0, -13.0], [5.0, -15.0]])] {
        let why = refused(&framed(&boss, 2.0, "round(r: 2mm)"));
        assert!(why.contains(seam), "{why}");
    }
    let why = refused(&framed(&round_boss(12.0, -20.0, 6.0), 2.0, "round(r: 1.999mm)"));
    assert!(why.contains("cannot roll on past where `frame.floor` and `frame.wall` cut it off"), "{why}");
    let why = refused(&framed(&round_boss(12.0, -20.0, 6.0), 2.0, "chamfer(d: 2mm)"));
    assert!(why.contains("no stretch of a flat closing to a point is cut off yet"), "{why}");
}

/// **The convex twin**: a boss across an edge rounded convex (`taken`), whose face's offset away
/// from the material never closes — an ordinary ball rolled against a cylinder, its canal fitted,
/// glued to the ring on the top and the runs down the end.  Built where the boss's axis meets the
/// round's, each ball's contact on the seams at the stretch's ends; off it refused as the concave
/// boss is.
#[test]
fn a_round_boss_across_a_convex_round_is_rolled() {
    let b = exported(&solved(&convex(58.0, 6.0, 2.0, "round(r: 2mm)")), "part");
    assert_eq!(kinds(&b).get("bspline"), Some(&2), "{:?}", kinds(&b));
    let why = refused(&convex(60.0, 6.0, 2.0, "round(r: 2mm)"));
    assert!(why.contains("runs on smoothly into `frame.top`, but its contact crosses their seam"), "{why}");
}

/// `rounds/onto_round.sv`, the case in the library: the issue's 2 against a boss of 6; and its
/// comment's edits, refused over 2 and off the fill's axis.
#[test]
fn the_onto_round_example_is_its_closed_form() {
    let src = include_str!("../../examples/rounds/onto_round.sv");
    close(props::volume(&exported(&solved(src), "part")), standing_volume(6.0, -20.0, 2.0));
    let why = refused(&src.replace("\nr := 2mm", "\nr := 2.5mm"));
    assert!(why.contains("of radius 2.5, is larger than"), "{why}");
    let why = refused(&src.replace("fix((12, -20)) o", "fix((14, -20)) o"));
    assert!(why.contains("crosses their seam in another section"), "{why}");
}

/// A plate 120 × 80 × 20 with a pocket 100 × 60 cut 10 deep, its plan corners drawn round to 5 and
/// its floor filled `big` all round (`filled`: four runs and four stretches of torus), a round boss
/// `boss9` of radius 3 about `at` standing from z = 5 to 18, unioned, and rounded `big` into the
/// plate (`part.boss9 round(r: …) part.plate`, the issue's words).
fn tray(at: [f64; 2], big: f64) -> String {
    format!("unit mm\nuse std\nuse std.solids (cut, union)\nuse std.rounds (round)\n\
        in std.top {{\n  a := point\n  b := point\n  c := point\n  d := point\n  fix((-60, -40)) a\n  \
        fix((60, -40)) b\n  fix((60, 40)) c\n  fix((-60, 40)) d\n  outline := face(a, b, c, d, -> close)\n  \
        p := point\n  q := point\n  s := point\n  t := point\n  fix((-50, -30)) p\n  fix((50, -30)) q\n  \
        fix((50, 30)) s\n  fix((-50, 30)) t\n  (south := line(p, q)) -> (se := round(r: 5mm)) (east := line(q, s)) -> \
        (ne := round(r: 5mm)) (north := line(s, t)) -> (nw := round(r: 5mm)) (west := line(t, p)) -> \
        (sw := round(r: 5mm)) close\n  mouth := face(south, se, east, ne, north, nw, west, sw)\n  \
        o := point\n  fix(({}, {})) o\n  b9 := circle(center: o) hint(r: 3)\n  radius(3mm) b9\n}}\n\
        plate := solid(std.solids.Prism(outline, from: 0mm, to: 20mm))\n\
        pocket := solid(std.solids.Prism(mouth, from: 10mm, to: 21mm))\n\
        pocket cut plate\n\
        filled := plate.pocket.far round(r: {big}mm) plate.pocket\n\
        part := solid(plate)\n\
        boss9 := solid(std.solids.Prism(face(b9), from: 5mm, to: 18mm))\n\
        boss9 union part\n\
        part.boss9 round(r: {big}mm) part.plate\n", at[0], at[1])
}

/// `tray`'s volume, the boss's axis meeting the south fill's (`y = −30 + r`): the plate less the
/// pocket with its corners, the floor's fill along each wall and turned round each corner (Pappus),
/// and the boss's own, as `standing_volume`'s — the boss 8 tall beyond the wall, the half ring, the
/// two balls, the runs up the wall from the fill's seam to the boss's top at 18.
fn tray_volume(r: f64) -> f64 {
    let (big, rb) = (5.0, 3.0);
    let a = big - r;
    let corner = 0.5 * PI * (r * (big * big - a * a) / 2.0 - a * PI * r * r / 4.0 - r * r * r / 3.0);
    let walls = 2.0 * (100.0 - 2.0 * big) + 2.0 * (60.0 - 2.0 * big);
    let plate = 192000.0 - 10.0 * (6000.0 - 4.0 * big * big * (1.0 - PI / 4.0)) + 4.0 * corner + square_run(r) * walls;
    let behind = rb * rb * (r / rb).acos() - r * (rb * rb - r * r).sqrt();
    let k = (rb * rb - r * r).sqrt();
    let section = area([0.0, -k], &[
        Step::To([0.0, -rb - r]),
        Step::Arc([r, -rb - r], r, PI, 0.5 * PI),
        Step::Arc([r, 0.0], rb, -0.5 * PI, (-k).atan2(-r)),
    ]);
    plate + 8.0 * (PI * rb * rb - behind) + 0.5 * root_ring(rb, r) + 2.0 * standing_dome(rb, r)
        + 2.0 * (18.0 - (10.0 + r)) * section
}

/// **The issue's own shape**: a tray whose pocket's floor is filled all round, its plan corners
/// arcs — so the fill's smooth chain is runs and stretches of torus, which the field now reads face
/// by face as it does a pocket's walls (`BodyFaces::bulging`) — and a boss across its south wall
/// rounded into it, at the issue's 0.5 and at 2: the closed form to 1e-9.  Off the fill's axis the
/// boss is refused as the frame's is.
#[test]
fn a_boss_across_a_trays_filled_wall_is_its_closed_form() {
    for r in [0.5, 2.0] {
        let b = exported(&solved(&tray([0.0, -30.0 + r], r)), "part");
        close(props::volume(&b), tray_volume(r));
        assert_eq!(kinds(&b).get("sphere"), Some(&2), "{r}: {:?}", kinds(&b));
    }
    let why = refused(&tray([0.0, -27.0], 0.5));
    assert!(why.contains("runs on smoothly into `pocket.far`, but its contact crosses their seam"), "{why}");
}
