//! **Rounds and chamfers written in Solvent** (#183, step 6): `round` and `chamfer` are words of
//! `rust/lib/std.sv` over the language's own features — faces as sets, the distance from a set,
//! a bound on measurements, where an edge ends, the material's angle, repetition over faces and
//! words that make solids — evaluated by the field, and held to X2's closed forms.
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use gcs_core::solve::{solve, SolveOpts};
use std::f64::consts::PI;

use crate::common::{ent, read};
use crate::region_general::field_volume;

fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}\n{src}", r.message);
    e
}

/// A block 40 × 24 × 12 drawn on the front plane (x, z), its near face at y = 0, standing 12 deep.
const BLOCK: &str = "unit mm\nuse std (round, chamfer)\nin std.front {\n\
    b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
    fix((0, 0)) b0\nfix((40, 0)) b1\nfix((40, 24)) b2\nfix((0, 24)) b3\n\
    (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> \
    (west := line(b3, b0)) -> close\n\
    outline := face(south, east, north, west)\n}\n\
    block := solid(outline, depth: 12mm)\n";

/// The volume a statement takes from (or gives to) solid `name`, within a box: the field's before
/// less after.
fn change(stmt: &str, name: &str, lo: [f64; 3], hi: [f64; 3], h: f64) -> f64 {
    let before = solved(BLOCK);
    let after = solved(&format!("{BLOCK}{stmt}\n"));
    let volume = |e: &Elaborated| {
        field_volume(&MaterialField::read(&e.sketch, ent(e, name).i(), 1e-10).unwrap(), lo, hi, h)
    };
    volume(&before) - volume(&after)
}

/// The stack's mitre: a convex polygon's prism with one cap's rim rounded alone loses `∫δ` along
/// its perimeter less `∫δ²` at each corner (`cots` the sum of `cot(θ/2)` over its angles).
fn mitred(perimeter: f64, cots: f64, r: f64) -> f64 {
    (1.0 - PI / 4.0) * r * r * perimeter - (5.0 / 3.0 - PI / 2.0) * r * r * r * cots
}

/// X2's lid: the near face's rim rounded alone, the other edges sharp — runs crossing at each
/// corner in a mitre.
#[test]
fn a_rim_rounded_alone_is_mitred() {
    let r = 2.0;
    let got = change("block.near round(2mm) block", "block", [-1.0, -3.0, -1.0], [41.0, 3.0, 25.0], r / 32.0);
    let want = mitred(2.0 * (40.0 + 24.0), 4.0, r);
    assert!((got - want).abs() < 2e-3 * want, "{got} against {want}");
}

/// A box rounded on every edge, its corners spherical (Steiner).
fn rounded_box([l, w, h]: [f64; 3], r: f64) -> f64 {
    let (a, b, c) = (l - 2.0 * r, w - 2.0 * r, h - 2.0 * r);
    a * b * c + 2.0 * r * (a * b + b * c + a * c) + PI * r * r * (a + b + c)
        + 4.0 * PI * r * r * r / 3.0
}

/// X2's block rounded all over: twelve runs, and a corner term where three meet.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier: every edge of a block, by the field")]
fn a_block_rounded_all_over_is_steiners() {
    let r = 2.0;
    let got = change("round(2mm) block", "block", [-1.0, -13.0, -1.0], [41.0, 13.0, 25.0], r / 16.0);
    let want = 40.0 * 24.0 * 12.0 - rounded_box([40.0, 24.0, 12.0], r);
    assert!((got - want).abs() < 2e-3 * want, "{got} against {want}");
}
