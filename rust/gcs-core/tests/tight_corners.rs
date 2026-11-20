//! **A round in a corner as tight as its ball** (#962's first item): a pocket's floor rounded
//! where its plan corners are arcs, the ball as wide as an arc or a hair short of it, and the
//! mirrored case, a boss's top rim.  Round each arc the ball's centre stands `R − r` from the
//! corner's axis: short of the axis the ring's face is a spindle torus, on it a sphere — the round
//! a quarter ball where the corner turns a right angle — and past it no ring is the set, refused
//! saying the corner's radius where the round runs against the ball's.  The field reads the ball's
//! centres off the pocket's walls face by face (`Reading::bulging`), so it reads the corner of
//! their offset where the arc's offset closes onto its axis, or has passed it, and the gate holds
//! the construction to it as ever.  Each volume is the closed form's to 1e-9: the plate less the
//! pocket, plus the round's section along each wall and turned through each corner (Pappus).
use gcs_core::brep::{export, props, sweep::Say};
use std::f64::consts::PI;

use crate::common::{close, ent, text::solved};
use crate::common::rounds::{exported, kinds};

/// A plate 120 × 80 × 20 with a pocket 10 deep cut into it, the pocket the polygon `outline` (each
/// vertex in turn, counter-clockwise) its corners rounded by `corners`, and its floor's edges
/// treated by `word` (`round(r: 5mm)`).
fn tray(outline: &[[f64; 2]], corners: &[f64], word: &str) -> String { tray_against(outline, corners, word, "") }

/// `tray`, the floor's word against the pocket's faces under `against` alone (`.e0`: one wall).
fn tray_against(outline: &[[f64; 2]], corners: &[f64], word: &str, against: &str) -> String {
    let n = outline.len();
    let mut s = String::from("unit mm\nuse std\nuse std.solids (cut)\nuse std.rounds (round, chamfer)\n\
        in std.top {\n  a := point\n  b := point\n  c := point\n  d := point\n  fix((-60, -40)) a\n  \
        fix((60, -40)) b\n  fix((60, 40)) c\n  fix((-60, 40)) d\n  plate_f := face(a, b, c, d, -> close)\n");
    for (k, [x, y]) in outline.iter().enumerate() {
        s += &format!("  p{k} := point\n  fix(({x}, {y})) p{k}\n");
    }
    let mut chain = Vec::new();
    let mut names = Vec::new();
    for k in 0..n {
        chain.push(format!("(e{k} := line(p{k}, p{})) -> (k{} := round(r: {}mm))", (k + 1) % n, (k + 1) % n,
            corners[(k + 1) % n]));
        names.push(format!("e{k}, k{}", (k + 1) % n));
    }
    s += &format!("  {} close\n  pocket_f := face({})\n}}\n", chain.join(" "), names.join(", "));
    s + "plate := solid(std.solids.Prism(plate_f, from: 0mm, to: 20mm))\n\
        pocket := solid(std.solids.Prism(pocket_f, from: 10mm, to: 21mm))\n\
        pocket cut plate\n" + &format!("plate.pocket.far {word} plate.pocket{against}\n")
}

/// The pocket 60 × 40 of the issue.
const RECT: [[f64; 2]; 4] = [[-30.0, -20.0], [30.0, -20.0], [30.0, 20.0], [-30.0, 20.0]];

/// What is added in a corner of radius `big` turning through `turn`, and along a straight wall
/// per unit of its length: a round's section — the square `r` on a side less the quarter disc of
/// the ball — or a chamfer's triangle (`flat`), turned about the corner's axis by Pappus.
fn added(big: f64, turn: f64, r: f64, flat: bool) -> (f64, f64) {
    if flat {
        return (turn * r * r / 2.0 * (big - r / 3.0), r * r / 2.0);
    }
    let a = big - r;
    (turn * (r * (big * big - a * a) / 2.0 - a * PI * r * r / 4.0 - r * r * r / 3.0), (1.0 - PI / 4.0) * r * r)
}

/// The closed form: the plate less the pocket 10 deep, plus what the floor's round or chamfer adds
/// along each wall and round each corner.
fn tray_volume(outline: &[[f64; 2]], corners: &[f64], r: f64, flat: bool) -> f64 {
    let n = outline.len();
    let at = |k: usize| outline[k % n];
    let (mut area, mut fill) = (0.0, 0.0);
    // each corner's turn, and how far its arc's tangent points stand from the vertex
    let turn = |k: usize| {
        let ([x0, y0], [x1, y1], [x2, y2]) = (at(k + n - 1), at(k), at(k + 1));
        let (u, v) = ([x1 - x0, y1 - y0], [x2 - x1, y2 - y1]);
        (u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1])
    };
    for k in 0..n {
        let ([x0, y0], [x1, y1]) = (at(k), at(k + 1));
        area += 0.5 * (x0 * y1 - x1 * y0);
        let phi = turn(k);
        let (big, half) = (corners[k], (0.5 * phi).tan());
        // the corner's plan area outside its arc, and the round turned through the corner
        area -= big * big * (half - 0.5 * phi);
        let (ring, run) = added(big, phi, r, flat);
        fill += ring;
        // the wall from this corner's arc to the next's
        let length = (x1 - x0).hypot(y1 - y0) - big * half - corners[(k + 1) % n] * (0.5 * turn(k + 1)).tan();
        fill += run * length;
    }
    120.0 * 80.0 * 20.0 - 10.0 * area + fill
}

/// Why each region of a document an entry recognises is refused, region by region.
fn regions_refused(src: &str) -> Vec<String> {
    let e = solved(src);
    (0..e.sketch.solids.len()).filter_map(|i| match gcs_core::solid::lowering::lowered(&e.sketch, i) {
        Some(Err(why)) => Some(why),
        _ => None,
    }).collect()
}

/// Solid `name` refused, and why.
fn refused(src: &str, name: &str) -> String {
    let e = solved(src);
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    export::exact(&e.sketch, ent(&e, name).i(), None, None, &say).err().expect("refused").message
}

/// **The issue's pocket**, its corners rounded 5: the floor rounded 4 (a spindle torus at each
/// corner), 5 (a quarter ball: the torus's middle circle closed onto the corner's axis) and a
/// hair either side of 5.  Each built, held to its field, and the closed form's volume.
#[test]
fn a_pocket_floor_rounded_to_its_corners_radius_is_built() {
    for r in [4.0, 4.9, 4.999, 4.9999, 4.99999, 5.0 - 1e-8, 5.0] {
        let b = exported(&solved(&tray(&RECT, &[5.0; 4], &format!("round(r: {r}mm)"))), "plate");
        close(props::volume(&b), tray_volume(&RECT, &[5.0; 4], r, false));
        // short of the axis a torus at each corner, on it (to the kernel's tolerance) a sphere
        let corner = if 5.0 - r > 1e-6 { "torus" } else { "sphere" };
        assert_eq!(kinds(&b), [("cylinder", 8), ("plane", 11), (corner, 4)].into(), "r {r}");
    }
}

/// `rounds/corner.sv`, the case in the library: the floor rounded to the corners' 5, four quarter
/// balls; and its comment's edits, a spindle torus under 5 and refused over it.
#[test]
fn the_corner_example_is_its_closed_form() {
    let src = include_str!("../../examples/rounds/corner.sv");
    let b = exported(&solved(src), "plate");
    close(props::volume(&b), 192000.0 - 10.0 * (2400.0 - 25.0 * (4.0 - PI)) + 160.0 * (1.0 - PI / 4.0) * 25.0
        + 125.0 * PI / 3.0);
    assert_eq!(kinds(&b), [("cylinder", 8), ("plane", 11), ("sphere", 4)].into());
    let under = src.replace("\nr := 5mm", "\nr := 4mm");
    close(props::volume(&exported(&solved(&under), "plate")), tray_volume(&RECT, &[5.0; 4], 4.0, false));
    let why = refused(&src.replace("\nr := 5mm", "\nr := 6mm"), "plate");
    assert!(why.contains("past `pocket.sw`, of radius 5mm where the round runs and tighter than the ball"), "{why}");
}

/// **Wider than the corner**: the ball never touches the corner's arc, and what none reaches runs
/// up the corner without end (`std.rounds`' centres are every ball within both faces), so no ring
/// is the set — refused saying the arc's radius against the ball's, to the figures that tell them
/// apart.  The walls' runs either side are refused too, their balls stopped short of the edge's
/// ends by the wall across the corner (`piece::stands_within`), saying the corner they pass and its
/// radius: the body says the first.  The same just over, and at the issue's 8.
#[test]
fn a_pocket_floor_rounded_wider_than_its_corners_is_refused_saying_why() {
    for (r, said) in [(5.001, "5.001mm"), (5.0 + 1e-6, "5.000001mm"), (8.0, "8mm")] {
        let src = tray(&RECT, &[5.0; 4], &format!("round(r: {r}mm)"));
        let regions = regions_refused(&src);
        assert!(regions.iter().any(|why| why.contains(&format!("its ball, of radius {said}, is wider than the arc it \
            rolls round, of radius 5mm where the round runs: it touches the faces either side of `pocket.k"))),
            "{regions:?}");
        let why = refused(&src, "plate");
        assert!(why.contains(&format!("its ball, of radius {said}, rolled against `pocket.e0` comes within")), "{why}");
        assert!(why.contains("of radius 5mm where the round runs and tighter than the ball"), "{why}");
        assert!(!why.contains("revolution"), "{why}");
    }
}

/// **The chamfer at the same sizes**: its flat's leg on the floor ends on the corner's axis at
/// 5 — a cone to its apex there, the closed form's — and past it crosses the axis, refused saying
/// so.
#[test]
fn a_pocket_floor_chamfered_to_its_corners_radius() {
    for d in [4.0, 4.9999, 5.0] {
        let b = exported(&solved(&tray(&RECT, &[5.0; 4], &format!("chamfer(d: {d}mm)"))), "plate");
        close(props::volume(&b), tray_volume(&RECT, &[5.0; 4], d, true));
    }
    let src = tray(&RECT, &[5.0; 4], "chamfer(d: 5.001mm)");
    let regions = regions_refused(&src);
    assert!(regions.iter().any(|why| why.contains("its flat's leg, 5.001mm, is longer than the radius of the arc it \
        runs round, 5mm")), "{regions:?}");
    let why = refused(&src, "plate");
    assert!(why.contains("of radius 5mm where the chamfer runs and tighter than the flat's leg"), "{why}");
}

/// **Corners of unequal radii in one pocket**: rounded 3, 5, 5 and 8, the floor rounded 3 is a
/// quarter ball in the tightest and a torus in the rest; rounded 5, the tightest refuses it.
#[test]
fn a_pocket_whose_corners_differ_is_rounded_to_its_tightest() {
    let corners = [3.0, 5.0, 5.0, 8.0];
    let b = exported(&solved(&tray(&RECT, &corners, "round(r: 3mm)")), "plate");
    close(props::volume(&b), tray_volume(&RECT, &corners, 3.0, false));
    assert_eq!(kinds(&b), [("cylinder", 8), ("plane", 11), ("sphere", 1), ("torus", 3)].into());
    let why = refused(&tray(&RECT, &corners, "round(r: 5mm)"), "plate");
    assert!(why.contains("past `pocket.k0`, of radius 3mm where the round runs and tighter than the ball"), "{why}");
}

/// **Corners other than a right angle**: a trapezoid whose corners turn 60° and 120°, each rounded
/// 5, its floor rounded 5 — each corner's ball the sphere turned through that corner's turn — and
/// short of it.  A hair short (4.9999) the floor keeps a hole 0.1 µm across about each corner's
/// axis, where the spindle torus meets it: built and held to its field like the rest, but its
/// mesh's chords round a 120° turn of so small a circle (`mesh::edge_params` keeps a circle's
/// turns however small) are under what float32 tells apart, so its STL is refused as slivers.
/// Its exact solid is held to the closed form.
#[test]
fn a_pocket_whose_corners_are_not_square_is_rounded_to_their_radius() {
    let h = 30.0 * 3f64.sqrt() / 2.0;
    let outline = [[-30.0, -15.0], [30.0, -15.0], [15.0, -15.0 + h], [-15.0, -15.0 + h]];
    for r in [4.9, 4.999, 5.0] {
        let b = exported(&solved(&tray(&outline, &[5.0; 4], &format!("round(r: {r}mm)"))), "plate");
        close(props::volume(&b), tray_volume(&outline, &[5.0; 4], r, false));
    }
    let e = solved(&tray(&outline, &[5.0; 4], "round(r: 4.9999mm)"));
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let b = export::exact(&e.sketch, ent(&e, "plate").i(), None, None, &say).unwrap().solid;
    close(props::volume(&b), tray_volume(&outline, &[5.0; 4], 4.9999, false));
}

/// A plate 120 × 80 × 10 with a boss standing 20 on it, the rectangle 60 × 40 its corners rounded
/// `big`, drafted `draft` (degrees, none at 0) from the plate's top, and the boss's top rim rounded
/// `r`.
fn boss(big: f64, draft: f64, r: f64) -> String {
    let solid = if draft == 0.0 {
        "std.solids.Prism(boss_f, from: 0mm, to: 20mm)".to_string()
    } else {
        format!("std.solids.Drafted(boss_f, from: 0mm, to: 20mm, draft: {draft}deg)")
    };
    format!("unit mm\nuse std\nuse std.solids (union)\nuse std.rounds (round)\nin std.top {{\n\
        a := point\nb := point\nc := point\nd := point\n\
        fix((-60, -40)) a\nfix((60, -40)) b\nfix((60, 40)) c\nfix((-60, 40)) d\n\
        plate_f := face(a, b, c, d, -> close)\n\
        p := point\nq := point\nr := point\ns := point\n\
        fix((-30, -20)) p\nfix((30, -20)) q\nfix((30, 20)) r\nfix((-30, 20)) s\n\
        (e0 := line(p, q)) -> (k1 := round(r: {big}mm)) (e1 := line(q, r)) -> (k2 := round(r: {big}mm)) \
        (e2 := line(r, s)) -> (k3 := round(r: {big}mm)) (e3 := line(s, p)) -> (k0 := round(r: {big}mm)) close\n\
        boss_f := face(e0, k1, e1, k2, e2, k3, e3, k0)\n}}\n\
        plate := solid(std.solids.Prism(plate_f, from: -10mm, to: 0mm))\n\
        boss := solid({solid})\nboss union plate\nplate.boss.near round(r: {r}mm) plate.boss\n")
}

/// **The mirrored case, a boss's top rim** (convex: the ball within the boss): rounded to its
/// corners' radius each corner is a quarter ball and the top a rectangle; a hair short, a spindle
/// torus.  What is taken is the pocket's fill turned inside out, the same closed form.
#[test]
fn a_boss_rim_rounded_to_its_corners_radius_is_built() {
    for r in [4.9999, 5.0] {
        let b = exported(&solved(&boss(5.0, 0.0, r)), "plate");
        let mut taken = 0.0;
        for k in 0..4 {
            let (ring, run) = added(5.0, 0.5 * PI, r, false);
            taken += ring + run * if k % 2 == 0 { 50.0 } else { 30.0 };
        }
        let area = 60.0 * 40.0 - 25.0 * (4.0 - PI);
        close(props::volume(&b), 120.0 * 80.0 * 10.0 + 20.0 * area - taken);
    }
}

/// **A drafted boss whose corners shrink under the ball** (fixture 126's class: a draft takes a
/// corner of 10 at the plate to 10 − 20 tan 10° = 6.473 at the top): rounded 8 there, the ball is
/// wider than the corner where the round runs, said so — the top's radius, not the drawn one.
#[test]
fn a_drafted_rim_whose_corner_shrinks_under_the_ball_is_refused_saying_its_radius_there() {
    let src = boss(10.0, 10.0, 8.0);
    let regions = regions_refused(&src);
    assert!(regions.iter().any(|why| why.contains("its ball, of radius 8mm, is wider than the arc it rolls round, of \
        radius 6.47mm where the round runs")), "{regions:?}");
    let why = refused(&src, "plate");
    assert!(why.contains("of radius 6.47mm where the round runs and tighter than the ball"), "{why}");
}

/// A plate with a round pocket or a round boss of radius 5 at (10, 5): the pocket cut 10 deep and
/// its floor rounded `r`, or the boss standing 20 on the plate and its top rim rounded `r`.
fn round(boss: bool, r: f64) -> String {
    let (word, body, solid) = if boss {
        ("union", "hole union plate\nplate.hole.near round(r: {r}mm) plate.hole\n",
            "hole := solid(std.solids.Prism(face(rim), from: 20mm, to: 40mm))\n")
    } else {
        ("cut", "hole cut plate\nplate.hole.far round(r: {r}mm) plate.hole\n",
            "hole := solid(std.solids.Prism(face(rim), from: 10mm, to: 21mm))\n")
    };
    format!("unit mm\nuse std\nuse std.solids ({word})\nuse std.rounds (round)\nin std.top {{\n\
        a := point\nb := point\nc := point\nd := point\n\
        fix((-60, -40)) a\nfix((60, -40)) b\nfix((60, 40)) c\nfix((-60, 40)) d\n\
        plate_f := face(a, b, c, d, -> close)\no := point\nfix((10, 5)) o\nrim := radius(5mm) circle(center: o)\n}}\n\
        plate := solid(std.solids.Prism(plate_f, from: 0mm, to: 20mm))\n{solid}{}", body.replace("{r}", &r.to_string()))
}

/// **A round pocket's floor rounded to its radius** (the commonest of the family: a ball-end
/// cutter's pocket) — one cylinder, no chain, read face by face as one convex solid all the same
/// (`BodyFaces::bulging`): a hair short a spindle torus, at it a hemisphere, the pocket less the
/// fill a cylinder less a hemisphere.  Its convex twin, a round boss's rim, is a dome.  Over the
/// radius no ball of it stands inside the cylinder at all: the field reads the whole of the
/// pocket above its floor, up past the plate without end, and the refusal says so.
#[test]
fn a_round_pockets_floor_and_a_round_boss_rounded_to_their_radius() {
    for r in [4.9, 4.999, 4.9999, 5.0] {
        let b = exported(&solved(&round(false, r)), "plate");
        close(props::volume(&b), 120.0 * 80.0 * 20.0 - 10.0 * 25.0 * PI + added(5.0, 2.0 * PI, r, false).0);
        let b = exported(&solved(&round(true, r)), "plate");
        close(props::volume(&b), 120.0 * 80.0 * 20.0 + 20.0 * 25.0 * PI - added(5.0, 2.0 * PI, r, false).0);
    }
    for (r, said) in [(5.0001, "5.0001mm"), (6.0, "6mm")] {
        for boss in [false, true] {
            let why = refused(&round(boss, r), "plate");
            assert!(why.contains(&format!("its ball, of radius {said}, is wider than the circle it rolls round, of \
                radius 5mm where the round runs: no ball of that radius stands on the round's side of `hole.rim` at \
                all")), "{why}");
        }
    }
}

/// **The floor rounded against one wall alone** (`plate.pocket.e0`): the wall goes on smoothly
/// into the corners' arcs and the walls beyond, so the ball's centres stop 8 short of `e3` and `e1`
/// — and rounded 8, 3 short of the edge's ends, where what no ball reaches is more than the run's
/// section: refused, naming the wall it comes within 5 of.  Rounded 5 the centres reach the edge's
/// ends on the arcs' axes, and the run is the prism of its section over the edge's 50, the closed
/// form's; the chamfer the same at 5, refused at 8.
#[test]
fn a_floor_rounded_against_one_wall_stops_where_its_ball_does() {
    let rect = |word: &str| tray_against(&RECT, &[5.0; 4], word, ".e0");
    let plain = 120.0 * 80.0 * 20.0 - 10.0 * (2400.0 - 25.0 * (4.0 - PI));
    for r in [4.0, 5.0] {
        close(props::volume(&exported(&solved(&rect(&format!("round(r: {r}mm)"))), "plate")),
            plain + 50.0 * (1.0 - PI / 4.0) * r * r);
    }
    close(props::volume(&exported(&solved(&rect("chamfer(d: 5mm)")), "plate")), plain + 50.0 * 12.5);
    let why = refused(&rect("round(r: 8mm)"), "plate");
    assert!(why.contains("its ball, of radius 8mm, rolled against `pocket.e0` comes within 5mm of `pocket.e3`"), "{why}");
    let why = refused(&rect("chamfer(d: 8mm)"), "plate");
    assert!(why.contains("its flat, reaching 8mm from `pocket.e0`, comes within 5mm of `pocket.e3`"), "{why}");
}

/// An obround pocket, two walls 10 apart joined by half circles of 5, its floor rounded `r`.
fn obround(r: f64) -> String {
    format!("unit mm\nuse std\nuse std.solids (cut)\nuse std.rounds (round)\nin std.top {{\n\
        a := point\nb := point\nc := point\nd := point\n\
        fix((-60, -40)) a\nfix((60, -40)) b\nfix((60, 40)) c\nfix((-60, 40)) d\n\
        plate_f := face(a, b, c, d, -> close)\n\
        p := point\nq := point\nr := point\ns := point\ncl := point\ncr := point\n\
        fix((-25, -5)) p\nfix((25, -5)) q\nfix((25, 5)) r\nfix((-25, 5)) s\nfix((25, 0)) cr\nfix((-25, 0)) cl\n\
        e0 := line(p, q)\nk1 := arc(center: cr, start: q, end: r)\ne2 := line(r, s)\n\
        k3 := arc(center: cl, start: s, end: p)\npocket_f := face(e0, k1, e2, k3)\n}}\n\
        plate := solid(std.solids.Prism(plate_f, from: 0mm, to: 20mm))\n\
        pocket := solid(std.solids.Prism(pocket_f, from: 10mm, to: 21mm))\n\
        pocket cut plate\nplate.pocket.far round(r: {r}mm) plate.pocket\n")
}

/// **Walls closer than the ball is wide**: an obround 10 wide rounded 5 — its walls' rounds tangent
/// along its middle, its ends half balls — and a hair short; rounded 5.0001 the ball rolled
/// against one wall comes within 4.9999 of the other: refused, saying so.
#[test]
fn an_obround_rounded_as_wide_as_it_is() {
    for r in [4.9999, 5.0] {
        let b = exported(&solved(&obround(r)), "plate");
        close(props::volume(&b), 120.0 * 80.0 * 20.0 - 10.0 * (500.0 + 25.0 * PI)
            + 100.0 * (1.0 - PI / 4.0) * r * r + 2.0 * added(5.0, PI, r, false).0);
    }
    let why = refused(&obround(5.0001), "plate");
    assert!(why.contains("its ball, of radius 5.0001mm, rolled against `pocket.e0` comes within 4.9999mm of `pocket.e2`"),
        "{why}");
}
