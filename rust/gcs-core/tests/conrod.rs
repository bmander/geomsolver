//! **The forged connecting rod** (`rust/examples/conrod/rod.sv`, #197, #66's rung 4, #183's L8):
//! the closing case, every round and chamfer a statement of `std.sv`'s words over the faces the
//! rod's solids gave it. It elaborates, solves and has material where its comments say; it is
//! built exactly, the bores' chamfers and the eyes' rims Pappus's to 1e-9 of the rod; the field's
//! reading of its rounds is the kernel's (the slow tier). The edges it leaves sharp are refused by
//! name.
//!
//! Two kernel rules it found: a traced meeting arriving tangent to an edge crosses it once
//! (`query::crossings_in`: the flanges' runs, left and right alike), and two chains of a lowered
//! round touching only along a line stand beside one another as two shells (`recipe::beside`: a
//! pocket's rim, its run and its ring's stretch each side of a corner).
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use gcs_core::solve::{solve, SolveOpts};
use std::f64::consts::PI;

use crate::common::{ent, read};
use crate::region_general::field_volume;

const ROD: &str = include_str!("../../examples/conrod/rod.sv");

fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    e
}

/// The rod with no round or chamfer, then `more` statements.
fn sharp(more: &[&str]) -> String {
    let mut out: String = ROD.lines()
        .filter(|l| {
            let worded = l.contains("round(r:") || l.contains("chamfer(d:");
            l.trim_start().starts_with("//") || !worded
        })
        .map(|l| format!("{l}\n")).collect();
    for m in more {
        out.push_str(m);
        out.push('\n');
    }
    out
}

/// The rod as this kernel builds it, or why it does not.
fn built(e: &Elaborated) -> Result<f64, String> {
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = export::exact(&e.sketch, ent(e, "rod").i(), None, None, &say)
        .map_err(|r| r.message)?;
    Ok(props::volume(&exact.solid))
}

/// The volume the statements `more` take from the sharp rod, built exactly (negative where they
/// add).
fn taken(more: &[&str]) -> f64 {
    let v = |src: &str| built(&solved(src)).unwrap_or_else(|why| panic!("{why}\n{src}"));
    v(&sharp(&[])) - v(&sharp(more))
}

/// Within 1e-9 of the rod's volume: what a statement takes is the difference of two builds of it.
fn close(what: &str, got: f64, want: f64) {
    eprintln!("{what}: {got:.9} against {want:.9}");
    assert!((got - want).abs() <= 1e-9 * ROD_VOLUME, "{what}: {got} against {want}");
}

/// About the rod's volume, mm³: the scale its builds are held to.
const ROD_VOLUME: f64 = 6e4;

/// Where the rod has material and where it has none: the web and the pockets' void; a pocket's
/// floor corner filled; a flange's edge, a bore's rim and an eye's rim taken; the shank filled
/// into the big eye. The pin at the origin, the rod along z, the front pocket at negative y.
#[test]
fn the_rod_is_rounded_where_its_comments_say() {
    let e = solved(ROD);
    let field = MaterialField::read(&e.sketch, ent(&e, "rod").i(), 1e-10).unwrap();
    let side = |p: [f64; 3]| field.side(p);
    // the web, and the pocket each side of it
    assert!(side([0.0, 0.0, 70.0]) < 0.0);
    assert!(side([0.0, -5.0, 70.0]) > 0.0);
    assert!(side([0.0, 5.0, 70.0]) > 0.0);
    // the pocket's floor (y = -2.5) into its wall (x = 6.5 at z = 70, square to the side's offset)
    assert!(side([6.2, -2.8, 70.0]) < 0.0);
    // the flange's edge where the shank's near face (y = -9) meets its right side (x = 10.5)
    assert!(side([10.45, -8.95, 70.0]) > 0.0);
    assert!(side([10.0, -8.5, 70.0]) < 0.0);
    // the big bore's rim on its cap (y = -14) and the big eye's
    assert!(side([0.0, -13.8, -23.7]) > 0.0);
    assert!(side([0.0, -13.95, -29.95]) > 0.0);
    assert!(side([0.0, -12.0, -27.0]) < 0.0);
    // the shank's right side into the big eye's wall: just outside both, filled
    assert!(side([11.7, 0.0, 28.0]) < 0.0);
    assert!(side([16.5, 0.0, 26.0]) > 0.0);
}

/// The bores' chamfers and the eyes' rims, each a whole circle: what each takes is its section
/// turned (Pappus) — a chamfer's triangle `d²/2` at `d/3` beyond the bore, a round's spandrel
/// `(1 − π/4) r²` at `r (10 − 3π) / (12 − 3π)` within the eye — both caps of each.
#[test]
fn the_bores_chamfers_and_the_eyes_rims_are_pappus() {
    let (ch, re) = (1.0, 1.0);
    let chamfer = |r: f64| 2.0 * 2.0 * PI * (r + ch / 3.0) * ch * ch / 2.0;
    let spandrel = re * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    let rim = |r: f64| 2.0 * 2.0 * PI * (r - spandrel) * (1.0 - PI / 4.0) * re * re;
    close("the bores' chamfers and the eyes' rims",
          taken(&["rod.big_end chamfer(d: ch) rod.big_hole",
                  "rod.small_end chamfer(d: ch) rod.small_hole",
                  "round(r: re) rod.big_end", "round(r: re) rod.small_end"]),
          chamfer(23.5) + chamfer(11.0) + rim(30.0) + rim(16.0));
}

/// A flange's long edge, run from one eye to the other and cut off by each: the rod is its own
/// mirror across the plane of swing and across the rod's axis, so each of the four runs takes the
/// same. (The right side's traced meeting with the small eye's wall arrives tangent to the edge
/// the side and the wall share, and was read as three crossings: `query::crossings_in`.)
#[test]
fn the_flanges_runs_are_alike_on_every_side() {
    let near_left = taken(&["rod.shank.near round(r: re) rod.shank.left"]);
    assert!(near_left > 0.0);
    for other in ["rod.shank.near round(r: re) rod.shank.right",
                  "rod.shank.far round(r: re) rod.shank.left"] {
        close(other, taken(&[other]), near_left);
    }
}

/// A pocket's rim, alone: at each corner of the pocket its straight run and its ring's stretch
/// each end on the wall the other runs along, so the two touch along the corner's line and stand
/// side by side, a shell each (`recipe::beside`). Front and back alike.
#[test]
fn a_pockets_rim_stands_beside_itself_at_its_corners() {
    let front = taken(&["rod.front_pocket round(r: re) rod.shank"]);
    assert!(front > 0.0);
    close("the back pocket's rim", taken(&["rod.back_pocket round(r: re) rod.shank"]), front);
}

/// The edges the rod leaves sharp, and why: a pocket's rim rounded beside its corners' fills, each
/// read off the rod before the other, leaves material touching itself along each corner's line.
/// The field reads it; the exact build is refused, saying so and where.
#[test]
fn a_rim_beside_a_filled_corner_is_refused_by_name() {
    let e = solved(&sharp(&["round(r: rp) rod.front_pocket",
                            "rod.front_pocket round(r: re) rod.shank"]));
    let why = built(&e).unwrap_err();
    assert!(why.contains("touching along a line") && why.contains("not a manifold"), "{why}");
    let field = MaterialField::read(&e.sketch, ent(&e, "rod").i(), 1e-10).unwrap();
    assert!(field.side([0.0, 0.0, 70.0]) < 0.0);
}

/// The rod as shipped, built and written as STEP and STL by this kernel (each read back and
/// checked as `solventc` checks it).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier: the rod built, written and checked")]
fn the_rod_is_built_exactly() {
    let e = solved(ROD);
    let body = ent(&e, "rod").i();
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = export::exact(&e.sketch, body, None, None, &say)
        .unwrap_or_else(|r| panic!("{r:?}"));
    export::step(&exact, "rod", None, &say).unwrap();
    let stl = export::stl(&e.sketch, body, &exact, None, &say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
    // the sharp rod less what each kind of round and chamfer takes, built apart: none takes what
    // another does (the flanges' runs end in the eyes' walls before the sides' fills begin)
    let v = props::volume(&exact.solid);
    let sharp = built(&solved(&sharp(&[]))).unwrap();
    let apart: f64 = [
        &["rod.shank.left round(r: rf) rod.big_end", "rod.shank.right round(r: rf) rod.big_end",
          "rod.shank.left round(r: rf) rod.small_end",
          "rod.shank.right round(r: rf) rod.small_end"][..],
        &["round(r: rp) rod.front_pocket", "round(r: rp) rod.back_pocket"],
        &["rod.big_end chamfer(d: ch) rod.big_hole", "rod.small_end chamfer(d: ch) rod.small_hole",
          "round(r: re) rod.big_end", "round(r: re) rod.small_end"],
        &["round(r: re) rod.shank"],
    ].iter().map(|more| taken(more)).sum();
    eprintln!("the rod: {v:.6}; sharp {sharp:.6}, less each kind apart {:.6}", sharp - apart);
    close("the rod, against each kind apart", v, sharp - apart);
}

/// The field's bar: a round's volume read off the field within this of the kernel's.
const BAR: f64 = 2e-3;

/// What `more` takes from the sharp rod read off the field within the box `lo`–`hi`, by cells of
/// `h`.
fn read_taken(more: &[&str], lo: [f64; 3], hi: [f64; 3], h: f64) -> f64 {
    let v = |src: &str| {
        let e = solved(src);
        field_volume(&MaterialField::read(&e.sketch, ent(&e, "rod").i(), 1e-10).unwrap(), lo, hi, h)
    };
    v(&sharp(&[])) - v(&sharp(more))
}

/// The field's reading of the rod's rounds is the kernel's construction: the shank's side filled
/// into the big eye (a plane against a cylinder), a pocket rounded inside (its floor's edges, its
/// upright corners and the balls where three meet) and a flange's edge run between the eyes.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, two minutes: three rounds, by the field")]
fn the_fields_rounds_are_the_kernels() {
    for (what, statement, lo, hi, h) in [
        ("the side into the big eye", "rod.shank.right round(r: rf) rod.big_end",
         [10.5, -9.5, 25.0], [16.5, 9.5, 33.0], 6.0 / 64.0),
        ("the front pocket, inside", "round(r: rp) rod.front_pocket",
         [-9.0, -9.5, 31.0], [9.0, -2.0, 123.0], 2.0 / 16.0),
        ("the flange's near right edge", "rod.shank.near round(r: re) rod.shank.right",
         [8.0, -9.5, 26.5], [12.5, -7.5, 128.0], 1.0 / 16.0),
    ] {
        let (field, exact) = (read_taken(&[statement], lo, hi, h), taken(&[statement]));
        eprintln!("{what}: {field:.6} against {exact:.6}, {:+.2e}", (field - exact) / exact);
        assert!((field - exact).abs() <= BAR * exact.abs(), "{what}: {field} against {exact}");
    }
}
