//! **A round onto a face that is itself a round** (#962's fifth item): a frame whose inside edge
//! is already filled `R`, a boss standing across it, and the boss rounded `R` into the frame.
//! Against the fill's concave face a ball of its own radius has its centre on the fill's axis
//! alone: the ball rolling along the floor beside the boss stops where its centre meets that axis,
//! stands still there against the boss and the fill (`fillet::dome`), and rolls on up the wall from
//! there — one fill all round the boss's foot.  Each pair of faces is a region of its own, and
//! each region is the part of that fill between the sections square to its crease at the crease's
//! ends: where the floor's run stops short of its crease's end (the boss's axis farther from the
//! wall than the fill's), the standing ball's piece past that end is the floor's region's
//! (`fillet::roll_beside`, `piece::stops_short`); where it runs on past the fill's seam before the
//! ball stands (nearer the wall), its canal from there to the standing ball is the fill's region's
//! (`fillet::dome`'s `dome_short`).  Each volume is the closed form's to 1e-9: the L, the fill, the
//! boss outside them, the floor's run turned round the boss (Pappus) less what of it lies in the
//! fill, the standing ball's piece each side, and the wall's runs; the integrals by tanh–sinh.
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

/// A boss whose foot is the quadrilateral `qs` (counter-clockwise).
fn quad_boss(qs: [[f64; 2]; 4]) -> String {
    let mut s = String::new();
    for (k, [x, y]) in qs.iter().enumerate() {
        s += &format!("  q{k} := point\n  fix(({x}, {y})) q{k}\n");
    }
    s + "  (b0 := line(q0, q1)) -> (b1 := line(q1, q2)) -> (b2 := line(q2, q3)) -> (b3 := line(q3, q0)) -> close\n  \
        foot := face(b0, b1, b2, b3)\n"
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

/// `∫ₐᵇ f` by tanh–sinh quadrature (steady to the last figures where `f` is smooth inside and
/// only its ends are singular, a square root's at a circle's end).
fn de(f: &dyn Fn(f64) -> f64, a: f64, b: f64) -> f64 {
    if b <= a {
        return 0.0;
    }
    let (c, d, h) = (0.5 * (a + b), 0.5 * (b - a), 1.0 / 32.0);
    let mut s = 0.0;
    for k in -110..=110 {
        let t = k as f64 * h;
        let u = 0.5 * PI * t.sinh();
        let (x, w) = (u.tanh(), 0.5 * PI * t.cosh() / (u.cosh() * u.cosh()));
        let p = c + d * x;
        if w > 1e-300 && p > a && p < b {
            s += w * f(p);
        }
    }
    s * h * d
}

/// What one round boss of radius `rb`, its axis `c` from the wall's face, adds standing across the
/// fill of radius `r` and rounded `r` into the frame, its own `h` beyond the wall's face and over
/// the floor and its top at `top` (the floor at 10): in the boss's frame (the wall's face `y = 0`,
/// the floor `z = 0`, the fill's axis `y = z = r`, the boss's axis `(0, c)`):
/// - the boss beyond the wall: its disc less the segment behind the wall, times `h`;
/// - the floor's run turned round the boss from one standing ball to the other (Pappus), less
///   what of it lies in the fill, nearer the wall than the boss's axis is (`c < r`);
/// - at each standing ball (on the fill's axis, `rb + r` from the boss's), the cell of the floor,
///   the wall and the boss less the ball and the fill: over its plan — the wall, the ball's
///   section across the fill's axis, the meridian through the ball, the boss — the column of air
///   up to `r` less the ball's sector;
/// - the wall's run each side from the fill's seam to the top, its section between the wall, the
///   boss and the ball;
/// - the fill under the boss less, the run along its top against the wall plus — one integral.
fn boss_terms(rb: f64, c: f64, r: f64, h: f64, top: f64) -> f64 {
    let xc = ((rb + r) * (rb + r) - (c - r) * (c - r)).sqrt();
    let xq = (rb * rb - c * c).sqrt();
    let zf = |y: f64| r - (r * r - (r - y) * (r - y)).max(0.0).sqrt();
    // the cell's plan: from the wall's corner with the boss along the wall to the ball's section,
    // up it to the ball, back along the meridian to the boss, round the boss
    let w = [rb * xc / (rb + r), c + rb * (r - c) / (rb + r)];
    let plan = area([xq, 0.0], &[Step::To([xc, 0.0]), Step::To([xc, r]), Step::To(w),
        Step::Arc([0.0, c], rb, (w[1] - c).atan2(w[0]), (-c).atan2(xq))]);
    let width = |y: f64| {
        let along = if c < r { xc * (y - c) / (r - c) } else { f64::NEG_INFINITY };
        xc - (rb * rb - (y - c) * (y - c)).sqrt().max(along)
    };
    let split = if c < r && w[1] > 0.0 && w[1] < r { vec![0.0, w[1], r] } else { vec![0.0, r] };
    let under: f64 = split.windows(2).map(|s| de(&|y| zf(y) * width(y), s[0], s[1])).sum();
    let sector = (-0.5 * PI - (c - r).atan2(-xc)).rem_euclid(2.0 * PI);
    let cell = r * plan - under - sector * r * r * r / 3.0;
    // the floor's run, and what of it lies in the fill where it passes over it
    let turn = PI + 2.0 * ((c - r) / (rb + r)).asin();
    let over = if c >= r { 0.0 } else {
        let ball = |q: f64| r - (r * r - (rb + r - q) * (rb + r - q)).max(0.0).sqrt();
        let inner = |t: f64| {
            let reach = ((r - c) / t.sin()).min(rb + r);
            de(&|q: f64| { let y = c + q * t.sin(); if y < r { q * ball(q).min(zf(y)).max(0.0) } else { 0.0 } }, rb, reach)
        };
        de(&inner, ((r - c) / (rb + r)).asin(), ((r - c) / rb).asin())
    };
    // the wall's run's section
    let below = (c - r).atan2(-xc);
    let wall = area([xq, 0.0], &[Step::To([xc, 0.0]),
        Step::Arc([xc, r], r, -0.5 * PI, if below > -0.5 * PI { below - 2.0 * PI } else { below }),
        Step::Arc([0.0, c], rb, (r - c).atan2(xc), (-c).atan2(xq))]);
    let behind = rb * rb * (c / rb).acos() - c * xq;
    h * (PI * rb * rb - behind) + root_ring(rb, r) * turn / (2.0 * PI) - 2.0 * over + 2.0 * cell
        + 2.0 * (top - 10.0 - r) * wall
}

/// `framed`'s volume with a round boss of radius `rb` about `(x, …)`, filled and rounded `r`.
fn framed_volume(rb: f64, x: f64, r: f64) -> f64 {
    36000.0 + 40.0 * square_run(r) + boss_terms(rb, x - 10.0, r, 15.0, 25.0)
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

/// **A round boss standing across a filled edge, wherever it stands**: rounded to the fill's own
/// radius, the ball along the floor stops where its centre meets the fill's axis and stands still
/// there, a sphere's patch each side, glued to the floor's ring and the wall's runs.  On the fill's
/// axis (the boss's axis `r` from the wall) the crease's ends are the ball's sections; farther off
/// the floor's run stops short and the standing ball's piece past its crease's end is the floor's
/// region's (a sphere's patch in two); nearer, and on the wall's line itself, the floor's ball
/// rolls on over the fill to the standing ball, a torus's stretch the fill's region's.  Bosses of
/// 3 to 10, rounds of 0.5 to 2, one moved along the frame; the closed form to 1e-9.
#[test]
fn a_round_boss_across_a_filled_edge_stands_a_ball_wherever_it_stands() {
    // (the boss's radius, its axis's x — the wall's face at 10 — and y, the fill's and round's radius)
    for (rb, x, y, r) in [(6.0, 12.0, -20.0, 2.0), (6.0, 14.0, -20.0, 2.0), (6.0, 13.0, -20.0, 2.0), (4.0, 13.0, -10.0, 2.0),
        (10.0, 16.0, -20.0, 2.0), (6.0, 11.0, -20.0, 2.0), (6.0, 10.0, -20.0, 2.0), (3.0, 11.0, -20.0, 0.5),
        (3.0, 10.2, -20.0, 0.5), (4.0, 12.5, -20.0, 1.5)] {
        let src = framed(&round_boss(x, y, rb), r, &format!("round(r: {r}mm)"));
        let b = exported(&solved(&src), "part");
        close(props::volume(&b), framed_volume(rb, x, r));
        // the ball's patch each side, in two where the floor's region holds part of it
        let spheres = if x - 10.0 > r + 1e-9 { 4 } else { 2 };
        assert_eq!(kinds(&b).get("sphere"), Some(&spheres), "{rb} {x} {r}: {:?}", kinds(&b));
    }
}

/// Which statement comes first says nothing: `part` names the frame its fill made.  Off the fill's
/// axis each way, written the other way round.
#[test]
fn a_boss_rounded_onto_a_filled_edge_reads_alike_written_either_way() {
    for x in [14.0, 11.0] {
        let src = framed(&round_boss(x, -20.0, 6.0), 2.0, "round(r: 2mm)");
        let (round, rest) = src.rsplit_once("part.boss").unwrap();
        let turned = round.replace("filled :=", &format!("part.boss{rest}filled :="));
        close(props::volume(&exported(&solved(&turned), "part")), framed_volume(6.0, x, 2.0));
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

/// **A flat-sided boss skewed 20° across the fill**: its sides run along the floor at a slant to
/// the fill's axis, so one side's floor run stops short of its crease's end (the standing ball's
/// patch in two) and the other's rolls on over the fill (a cylinder's stretch, its canal along the
/// side's line).  Built and exported; the field draws the kernel's boundary round the boss's foot,
/// point by point, up to below its top (where the run along its top against the wall ends on the
/// slanted sides, which is no round onto a round).
#[test]
fn a_flat_boss_skewed_across_a_filled_edge_is_the_fields() {
    let t = 20f64.to_radians().tan();
    let src = framed(&quad_boss([[5.0, -25.0], [20.0, -25.0 + 15.0 * t], [20.0, -15.0 + 15.0 * t], [5.0, -15.0]]), 2.0,
        "round(r: 2mm)");
    let e = solved(&src);
    let b = exported(&e, "part");
    assert_eq!(kinds(&b).get("sphere"), Some(&3), "{:?}", kinds(&b));
    crate::common::rounds::field_is_the_kernels(&e, "part", [9.0, -27.0, 9.0], [22.0, -8.0, 20.0], 0.05, 3000);
}

/// **A ball wider than the fill** never touches its face (no point stands its radius inside the
/// fill's concave side), and what none reaches runs along the fill without end, as round a pocket's
/// corner (`tight_corners`): refused, saying the ball's radius and the face's.  A millionth over,
/// and a tenth.
#[test]
fn a_ball_wider_than_the_fill_is_refused_saying_both_radii() {
    for r in ["2.000001", "2.1"] {
        let why = refused(&framed(&round_boss(12.0, -20.0, 6.0), 2.0, &format!("round(r: {r}mm)")));
        assert!(why.contains(&format!("of radius {r}, is larger than `")) && why.contains("curved round it to a radius \
            of 2: no ball of that radius touches it"), "{why}");
    }
}

/// **What is not built yet is refused, saying why**: a ball a hair under the fill's radius, rolled
/// round a spindle so short no section is carried past where the seams cut it off, on the fill's
/// axis and off it; the chamfer, whose flat closes the boss's side to a point over a stretch; and a
/// boss standing over the frame's end, the fill's meeting with it ending on the end's face, which
/// cuts across it.
#[test]
fn a_round_onto_a_round_not_built_yet_is_refused_saying_why() {
    let why = refused(&framed(&round_boss(12.0, -20.0, 6.0), 2.0, "round(r: 1.999mm)"));
    assert!(why.contains("cannot roll on past where `frame.floor` and `frame.wall` cut it off"), "{why}");
    let why = refused(&framed(&round_boss(14.0, -20.0, 6.0), 2.0, "round(r: 1.9mm)"));
    assert!(why.contains("its contact crosses their seam in another section than its edge does"), "{why}");
    let why = refused(&framed(&round_boss(12.0, -20.0, 6.0), 2.0, "chamfer(d: 2mm)"));
    assert!(why.contains("no stretch of a flat closing to a point is cut off yet"), "{why}");
    let why = refused(&framed(&round_boss(13.0, 5.5, 6.0), 2.0, "round(r: 2mm)"));
    assert!(why.contains("cuts across it: no stretch of its dome is carried past and cut off by a face yet"), "{why}");
}

/// **The convex twin**: a boss across an edge rounded convex (`taken`), whose face's offset away
/// from the material never closes — an ordinary ball rolled against a cylinder, its canal fitted,
/// glued to the ring on the top and the runs down the end.  Built where the boss's axis meets the
/// round's, each ball's contact on the seams at the stretch's ends; off it refused, as a rolled
/// run whose contact crosses a seam in another section is.
#[test]
fn a_round_boss_across_a_convex_round_is_rolled() {
    let b = exported(&solved(&convex(58.0, 6.0, 2.0, "round(r: 2mm)")), "part");
    assert_eq!(kinds(&b).get("bspline"), Some(&2), "{:?}", kinds(&b));
    let why = refused(&convex(60.0, 6.0, 2.0, "round(r: 2mm)"));
    assert!(why.contains("runs on smoothly into `frame.top`, but its contact crosses their seam"), "{why}");
}

/// `rounds/onto_round.sv`, the case in the library: the issue's 2 against a boss of 6; and its
/// comment's edits, the boss moved off the fill's axis either way, and refused over 2.
#[test]
fn the_onto_round_example_is_its_closed_form() {
    let src = include_str!("../../examples/rounds/onto_round.sv");
    close(props::volume(&exported(&solved(src), "part")), framed_volume(6.0, 12.0, 2.0));
    for x in [14.0, 11.0] {
        let moved = src.replace("fix((12, -20)) o", &format!("fix(({x}, -20)) o"));
        close(props::volume(&exported(&solved(&moved), "part")), framed_volume(6.0, x, 2.0));
    }
    let why = refused(&src.replace("\nr := 2mm", "\nr := 2.5mm"));
    assert!(why.contains("of radius 2.5, is larger than"), "{why}");
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

/// `tray`'s volume, the boss's axis `c` from the south wall: the plate less the pocket with its
/// corners, the floor's fill along each wall and turned round each corner (Pappus), and the boss's
/// own (`boss_terms`: 8 tall beyond the wall, its top at 18).
fn tray_volume(c: f64, r: f64) -> f64 {
    let big = 5.0;
    let a = big - r;
    let corner = 0.5 * PI * (r * (big * big - a * a) / 2.0 - a * PI * r * r / 4.0 - r * r * r / 3.0);
    let walls = 2.0 * (100.0 - 2.0 * big) + 2.0 * (60.0 - 2.0 * big);
    192000.0 - 10.0 * (6000.0 - 4.0 * big * big * (1.0 - PI / 4.0)) + 4.0 * corner + square_run(r) * walls
        + boss_terms(3.0, c, r, 8.0, 18.0)
}

/// **The issue's own shape**: a tray whose pocket's floor is filled all round, its plan corners
/// arcs — so the fill's smooth chain is runs and stretches of torus, which the field reads face by
/// face as it does a pocket's walls (`BodyFaces::bulging`) — and a boss standing anywhere across
/// its south wall, rounded into it, at the issue's 0.5 and at 2: on the fill's axis, farther from
/// the wall and nearer it; the closed form to 1e-9.
#[test]
fn a_boss_across_a_trays_filled_wall_is_its_closed_form() {
    for (c, r) in [(0.5, 0.5), (1.5, 0.5), (0.2, 0.5), (2.0, 2.0), (2.8, 2.0), (1.0, 2.0)] {
        let b = exported(&solved(&tray([7.0, -30.0 + c], r)), "part");
        close(props::volume(&b), tray_volume(c, r));
    }
}
