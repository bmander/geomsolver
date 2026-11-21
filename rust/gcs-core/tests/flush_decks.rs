//! **A run whose edge ends running into a face flush with one of its own** (#994 item 2b, fixture
//! 117).  A deck stands on a block, its side `k4` in the block's side plane `x = 0`, flush, and its
//! corner arc `k3` runs tangent into that side.  The fill round the deck's foot runs along the
//! deck's sides and round the arc to where the arc meets the flush side: there the edge ends, and
//! along `k4` there is none — the deck's side and the block's are one plane.  The block's side `j3`
//! goes on tangent from the arc (its normal there the arc's), but it meets the block's top in a
//! convex edge: it carries no fill on, so the fill ends on it (`BodyFaces::carries_on`), cut off at
//! its plane and kept within the block — a fillet thinning to nothing where it runs out tangent,
//! not a lip hanging past the wall.  The field read the deck's side going on into the block's
//! (`dck.k3.smooth` holds `base.j3`) by the face whose patch is nearest, so a point over the
//! block's top beside its rim read the wall's side, within, and a fill ran along the wall's top
//! edge in the air (`BodyFaces::nearest_face`); and from within the block it read the chain as
//! one convex solid, the arc carried down through the block by its tangent planes, though the
//! wall's patch lies outside it (`BodyFaces::bulging`'s one sheet).  The mirrored case is the
//! block's top rim rounded along its flush side, running out against the deck's arc, kept outside
//! it.
//!
//! The family, each drawn whole at DOF 0 and its volume the closed form's to 1e-9: the deck's
//! corner square (meeting the flush side at a right angle), an arc of 15, 2 and 1 tangent to it,
//! and an arc of 15 meeting it at an angle; its foot filled 2 and 1; the block's rim along the flush
//! side rounded 2 and 1.  Each is exported (STEP and STL, read back) with no face thinner than the
//! export's tolerance.  A chamfer at the deck's arc is still refused, by the faces it is of.
use gcs_core::brep::{export, props, sweep::Say};
use std::f64::consts::PI;

use crate::common::rounds::{gauss_legendre, refused};
use crate::common::{close, ent, text::{dof, solved}};

/// The block `[0, 80] × [0, 60] × [0, 20]` and the deck on it, 20 to 30: its outline from
/// `(0, 10)` along `y = 10` to `(60, 10)`, up to `(60, 50)`, back along `y = 50` to `(c, 50)`, round
/// an arc of `big` about `(c, 50 − big)` to the side `x = 0` (tangent where `c` is `big`; none where
/// `big` is nothing, the corner square), and down that side, flush with the block's; `word` after.
fn deck(big: f64, c: f64, word: &str) -> String {
    let corner = if big > 0.0 {
        let yq = 50.0 - big + (big * big - c * c).sqrt();
        format!("  r := point\n  fix(({c}, 50)) r\n  q := point\n  fix((0, {yq:.17})) q\n  o := point\n  \
            fix(({c}, {})) o\n  k0 := line(p, t)\n  k1 := line(t, s)\n  k2 := line(s, r)\n  \
            k3 := arc(center: o, start: r, end: q) hint(r: {big})\n  k4 := line(q, p)\n  \
            dk := face(k0, k1, k2, k3, k4)\n", 50.0 - big)
    } else {
        "  r := point\n  fix((0, 50)) r\n  k0 := line(p, t)\n  k1 := line(t, s)\n  k2 := line(s, r)\n  \
            k4 := line(r, p)\n  dk := face(k0, k1, k2, k4)\n".to_string()
    };
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (union)\nuse std.rounds (round, chamfer)\n\
        in std.top {{\n  a := point\n  fix((0, 0)) a\n  b := point\n  fix((80, 0)) b\n  c := point\n  \
        fix((80, 60)) c\n  d := point\n  fix((0, 60)) d\n  j0 := line(a, b)\n  j1 := line(b, c)\n  \
        j2 := line(c, d)\n  j3 := line(d, a)\n  blk := face(j0, j1, j2, j3)\n  p := point\n  fix((0, 10)) p\n  \
        t := point\n  fix((60, 10)) t\n  s := point\n  fix((60, 50)) s\n{corner}}}\n\
        base := solid(std.solids.Prism(blk, from: 0mm, to: 20mm))\n\
        dck := solid(std.solids.Prism(dk, from: 20mm, to: 30mm))\n\
        part := solid(base)\ndck union part\n{word}\n")
}

/// The block and the deck unrounded: the deck's outline's area (the arc's segment over its chord
/// past the polygon) ten deep.
fn bare(big: f64, c: f64) -> f64 {
    let outline = if big > 0.0 {
        let yq = 50.0 - big + (big * big - c * c).sqrt();
        let p = [[0.0, 10.0], [60.0, 10.0], [60.0, 50.0], [c, 50.0], [0.0, yq]];
        let shoelace: f64 = (0..5).map(|i| p[i][0] * p[(i + 1) % 5][1] - p[(i + 1) % 5][0] * p[i][1]).sum();
        let turn = (-c / big).acos() - 0.5 * PI;
        0.5 * shoelace + 0.5 * big * big * (turn - turn.sin())
    } else {
        2400.0
    };
    96000.0 + 10.0 * outline
}

/// A straight run's section: the square `r` on a side less the quarter disc of the ball.
fn run(r: f64) -> f64 { (1.0 - PI / 4.0) * r * r }

/// What the foot's fill adds round the deck's arc of `big` about `x = c`: each point of its section
/// at `ρ` from the arc's axis turned from the arc's start, a quarter turn back from the side, to
/// where `x = c + ρ cos θ` meets the side — `π/2 − acos(c/ρ)` — kept within the block.  Over the
/// section by `t`, `ρ = big + r (1 − cos t)` and the height `r (1 − sin t)`: smooth where the
/// arc runs out tangent (`acos(big/ρ)` goes as `t`).
fn ring(big: f64, c: f64, r: f64) -> f64 {
    gauss_legendre(60).into_iter().map(|(x, w)| {
        let t = 0.25 * PI * (1.0 + x);
        let rho = big + r * (1.0 - t.cos());
        0.25 * PI * w * rho * r * (1.0 - t.sin()) * (0.5 * PI - (c / rho).acos()) * r * t.sin()
    }).sum()
}

/// The deck's foot filled `r`: the bare part, the runs along `k0`, `k1` and `k2` (none along the
/// flush side), and the ring round the arc (square, the deck's corner turns along `k2` to the side).
fn filled(big: f64, c: f64, r: f64) -> f64 {
    let along = 60.0 + 40.0 + (60.0 - c);
    bare(big, c) + along * run(r) + if big > 0.0 { ring(big, c, r) } else { 0.0 }
}

/// The block's rim along its flush side rounded `r`, the arc of `big` tangent to that side: the run
/// from the block's corner to the deck's side `k0` (10 long), and the run from the far corner kept
/// outside the deck's arc — each point of its section `x` in from the side, `r − √(x(2r − x))`
/// deep, going on from `y = 60` to where the arc's cylinder stands, `10 + big − √(x(2 big − x))`
/// long.  By `x = big (1 − cos s)` where the arc is there, so both roots go smooth.
fn rimmed(big: f64, r: f64) -> f64 {
    let depth = |x: f64| r - (x * (2.0 * r - x)).max(0.0).sqrt();
    let reach = r.min(2.0 * big);
    let top = (1.0 - reach / big).acos();
    let under: f64 = gauss_legendre(60).into_iter().map(|(x, w)| {
        let s = 0.5 * top * (1.0 + x);
        let at = big * (1.0 - s.cos());
        0.5 * top * w * depth(at) * (10.0 + big - big * s.sin()) * big * s.sin()
    }).sum();
    // past the arc's reach, the run whole to the deck's flush side
    let past: f64 = if r > reach {
        gauss_legendre(60).into_iter().map(|(x, w)| {
            let at = reach + 0.5 * (r - reach) * (1.0 + x);
            0.5 * (r - reach) * w * depth(at) * (10.0 + big)
        }).sum()
    } else {
        0.0
    };
    bare(big, big) - 10.0 * run(r) - under - past
}

/// Solid `name` of `src` built exactly, exported as STEP and STL (each read back), no face of it
/// thinner than the export's tolerance; its volume.
fn built(src: &str, name: &str) -> f64 {
    assert_eq!(dof(src), 0, "{src}");
    let e = solved(src);
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let body = ent(&e, name).i();
    let exact = export::exact(&e.sketch, body, None, None, &say).unwrap_or_else(|r| panic!("{name}: {r:?}\n{src}"));
    if let Some(s) = export::slivers(&e.sketch, body, &exact, None) {
        panic!("{}\n{src}", s.message);
    }
    export::step(&exact, name, None, &say).unwrap();
    let stl = export::stl(&e.sketch, body, &exact, None, &say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
    props::volume(&exact.solid)
}

/// **The deck's foot filled where its corner arc runs tangent into its flush side** (#994's repro,
/// the arc of 15 filled 2, and the arcs of 2 and 1 the issue saw refused next), each filled 2 and 1:
/// the ring round the arc thins to nothing at the side, kept within the block.  With the corner
/// square the run along `k2` ends on the side at a right angle, as it built before.
#[test]
fn a_decks_foot_is_filled_to_where_its_arc_runs_tangent_into_its_flush_side() {
    for big in [0.0, 15.0, 2.0, 1.0] {
        for r in [2.0, 1.0] {
            let src = deck(big, big, &format!("step := part.base.near round(r: {r}mm) part.dck"));
            close(built(&src, "part"), filled(big, big, r));
        }
    }
}

/// The arc meeting the flush side at an angle (its centre 10 from the side, its radius 15), which
/// built before: the ring turned clear past the side and cut off there — the tangent arc's limit.
#[test]
fn a_decks_foot_is_filled_to_where_its_arc_meets_its_flush_side_at_an_angle() {
    for r in [2.0, 1.0] {
        let src = deck(15.0, 10.0, &format!("part.base.near round(r: {r}mm) part.dck"));
        close(built(&src, "part"), filled(15.0, 10.0, r));
    }
}

/// **The mirrored case** (#994: "a round on the body's top rim, at its flush side"): the block's top
/// rim along `j3` rounded, the run past the deck running out against its arc, kept outside it — so
/// nothing is cut from under the deck's foot.  Square, the run ends on `k2` at a right angle.
#[test]
fn a_blocks_rim_along_its_flush_side_runs_out_against_the_decks_arc() {
    for big in [15.0, 2.0, 1.0] {
        for r in [2.0, 1.0] {
            let src = deck(big, big, &format!("part.base.near round(r: {r}mm) part.base.j3"));
            close(built(&src, "part"), rimmed(big, r));
        }
    }
    for r in [2.0, 1.0] {
        let src = deck(0.0, 0.0, &format!("part.base.near round(r: {r}mm) part.base.j3"));
        close(built(&src, "part"), bare(0.0, 0.0) - 20.0 * run(r));
    }
}

/// A chamfer at the deck's arc is still refused: its flat's leg is read from the deck's side and
/// the block's together (`distance(p, b.smooth)`, the patches'), and near the block's rim the
/// block's side is the nearer — said by the faces it is of.
#[test]
fn a_chamfer_at_the_decks_arc_is_refused_by_its_faces() {
    let src = deck(15.0, 15.0, "part.base.near chamfer(d: 1.5mm) part.dck");
    let why = refused(&solved(&src), "part");
    assert!(why.contains("`base.j3`, which `dck.k3` goes on into smoothly"), "{why}");
}
