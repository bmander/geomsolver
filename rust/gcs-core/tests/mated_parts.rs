//! **A part mated by what it made starts where its own solve, placed whole, puts it** (#904 A5,
//! `program::mated`).  A part placed by relations on its members — a gear's axis along a shaft,
//! a pinion's square to an axis, an ordinate for the shaft's plane — rather than on its datums
//! has nothing to seed its frame from before it is solved, and its frame is read through every
//! point it draws: the document was one block of the part's layout and its placement, and the
//! hypoid pair, its face mill balanced on envelopes, ran minutes and did not solve.  Solved alone
//! where its frame stands, placed by its frame, and carried, it is seeded at its answer.
use crate::common::at;
use gcs_core::program::Elaborated;
use gcs_core::solve::{bounds_hold, solve};
use gcs_core::space::{dot, sub};
use gcs_core::system::System;

/// How far the elaborated drawing's rows are from holding, each over its own units: where the
/// seeds stand.
fn off(e: &Elaborated) -> f64 {
    let mut sys = System::new(&e.sketch);
    let z = sys.z0(&e.sketch);
    sys.max_relative_residual(&z)
}

/// Two shafts at a right angle 17.5 apart, the second's axis found by the part's own rows
/// (the primer's 2.13), placed by the first's axis and the second's square to a held direction.
const SKEW: &str = "\
unit mm
use std
use std.place (horizontal)
component Skew(offset: Length) {
  in std.front {
    gax := line
    fix((0, 0)) gax.p1
    fix((0, 50)) gax.p2
  }
  t := axis hint(dir: (0.87, 0, 0.5))
  t perpendicular std.y
  side := plane(u: t, v: std.y)
  side.origin coincident std.front
  in side {
    pax := line(hint((0, 10)), hint((60, 12)))
    pax.p1 level(u) side
    pax.p2 distance(60, along: u) side
    pax.p1 horizontal pax.p2
  }
  gax angle(90deg) pax
  gax distance(offset) pax
}
flip := axis
fix(dir == (0.6, 0, 0.8)) flip
fix(origin == (0mm, 0mm, 0mm)) flip
k := Skew(offset: 17.5mm)
k.std.origin coincident std.x
k.gax parallel(sense: opposite) std.x
k.pax perpendicular flip
std.origin distance(30mm, along: std.x) k.pax.p1
";

/// A small part, placed by its members: elaborated, it already stands where its mate puts it,
/// its axis back along the world's x, its own found axis square to it and to `flip` and 30 out —
/// its rows and the mate's holding at the seeds — and the solve has nothing left to do.  (Which
/// way the found axis runs is its own root, which its seeds choose.)
#[test]
fn a_part_mated_by_what_it_made_starts_placed() {
    let mut e = fixtures::unsolved_in_examples(SKEW);
    assert!(off(&e) < 1e-9, "the seeds stand {} off", off(&e));
    let (g1, g2) = (at(&e, "k.gax.p1"), at(&e, "k.gax.p2"));
    let (p1, p2) = (at(&e, "k.pax.p1"), at(&e, "k.pax.p2"));
    let g = sub(g2, g1);
    assert!((g[1].abs() + g[2].abs()) < 1e-9 && g[0] < 0.0, "{g:?}");
    assert!(g1[1].abs() < 1e-9 && g1[2].abs() < 1e-9, "{g1:?}");
    let p = sub(p2, p1);
    assert!(dot(p, [0.6, 0.0, 0.8]).abs() < 1e-9 && p[0].abs() < 1e-9, "{p:?}");
    assert!((p1[0] - 30.0).abs() < 1e-9, "{p1:?}");
    let r = solve(&mut e.sketch, fixtures::accurate());
    assert!(r.settled(), "{r:?}");
}

/// The issue's mate (#904 A5): the pair stood on the world's y by its frame's origin, its gear's
/// axis along y, its pinion's square to `flip` and running back, the pinion's foot in the
/// shaft's plane 40 out on the drive side.
const HEAD: &str = "\
unit mm
use std
use spiral_bevel.hypoid (HypoidPair)
flip := axis
fix(dir == (0.162790697674, 0, 0.986660377)) flip
fix(origin == (0mm, -40mm, 0mm)) flip
";
const MATE: &str = "\
pair.std.origin coincident std.y
pair.gear_member.ax parallel(sense: same) std.y
pair.pinion_member.ax perpendicular flip
pair.pinion_member.ax.p1 distance(<= 0mm, along: std.x) pair.pinion_member.ax.p2
std.origin distance(-40mm, along: std.y) pair.pinion_member.foot
";

/// What the mate says of the solved pair: the gear's apex and foot on y, the pinion's axis square
/// to `flip` and back along x from its apex, its foot at -40 along y — and every bound held.
fn placed(e: &Elaborated) {
    for n in ["pair.gear_cone.O", "pair.gear_cone.foot"] {
        let w = at(e, n);
        assert!(w[0].abs() < 1e-6 && w[2].abs() < 1e-6, "{n} {w:?}");
    }
    let (a, foot) = (at(e, "pair.pinion_cone.A"), at(e, "pair.pinion_cone.foot"));
    let ax = sub(foot, a);
    assert!(dot(ax, [0.162790697674, 0.0, 0.986660377]).abs() < 1e-6 && ax[0] < 0.0, "{ax:?}");
    assert!((foot[1] + 40.0).abs() < 1e-6, "{foot:?}");
    assert!(bounds_hold(&e.sketch));
}

/// The configured pair, its face mill stated: seeded where the mate places it, and solved there.
/// Before, the document came out of its one block in some fifteen seconds; now at once.
#[test]
fn the_hypoid_pair_is_mated_by_its_axes() {
    let [shift, cutter] = fixtures::gear::FOUND_MILL;
    let pair = format!("\
pair := HypoidPair(pinion_teeth: 24, gear_teeth: 48, mean_module: 2mm, shaft_angle: 90deg,
  offset: 25mm, spiral_angle: 25deg, backlash: 0.05mm, tip_relief: 0.2mm, end_relief: 0.2mm,
  pressure_shift: {shift}deg, cutter_radius: {cutter}mm)
");
    let mut e = fixtures::unsolved_in_examples(&format!("{HEAD}{pair}{MATE}"));
    assert!(off(&e) < 1e-6, "the seeds stand {} off", off(&e));
    placed(&e);
    let r = solve(&mut e.sketch, fixtures::accurate());
    assert!(r.settled(), "{r:?}");
    placed(&e);
}

/// The issue's case: the face mill's shift found by the balance its caller states of the pair's
/// margins — two flanks held to one unknown margin, as `hypoid.sv` holds them — which the pair
/// solved alone finds, and mated as above.  Before, the solve ran minutes and did not come out.
#[test]
fn the_hypoid_pair_balanced_by_its_caller_is_mated_by_its_axes() {
    let cutter = fixtures::gear::FOUND_MILL[1];
    let pair = format!("\
pair := HypoidPair(pinion_teeth: 24, gear_teeth: 48, mean_module: 2mm, shaft_angle: 90deg,
  offset: 25mm, spiral_angle: 25deg, backlash: 0.05mm, tip_relief: 0.2mm, end_relief: 0.2mm,
  pressure_shift: hint(10deg), cutter_radius: {cutter}mm, least: hint(0.5))
pair.margins.pinion_inner regular(pair.least, by: pair.margins.pinion_cut) within pair.margins.pinion
pair.margins.gear_outer regular(pair.least, by: pair.margins.gear_outer_cut) within pair.margins.gear
");
    let mut e = fixtures::unsolved_in_examples(&format!("{HEAD}{pair}{MATE}"));
    assert!(off(&e) < 1e-6, "the seeds stand {} off", off(&e));
    let r = solve(&mut e.sketch, fixtures::accurate());
    assert!(r.settled(), "{r:?}");
    placed(&e);
    // the shift the pair finds standing alone (`fixtures::gear::FOUND_MILL`)
    let shift = e.sketch.params[e.sketch.free_vars["pair.pressure_shift"] as usize].value;
    assert!((shift - fixtures::gear::FOUND_MILL[0]).abs() < 1e-4, "the pressure shift {shift}°");
}

/// What a frame carries rigidly is derived in a drawing with no image (#904 A5).  `mated` places
/// a part by making its places in space follow its frame (`Sketch::rigid`), and `derived` gave
/// those back only past its mirrors' images: in a drawing with none, as every one here is,
/// nothing was carried, the part's places stood free where its frame was to place them, and the
/// whole solve after did the placing — the balanced pair's in half a minute, now two seconds.
#[test]
fn what_a_frame_carries_rigidly_is_derived_with_no_image() {
    use gcs_core::model::Derivation;
    let mut sk = fixtures::unsolved_in_examples(SKEW).sketch;
    assert!(sk.mirrors.is_empty() && sk.rigid.is_empty());
    assert!(sk.derived().is_empty() && sk.derived_mask().is_empty());
    sk.rigid.push(Derivation { param: 1, terms: vec![(0, 1.0)] });
    let derived = sk.derived();
    let carried = derived.iter().any(|d| d.param == 1 && d.terms == [(0, 1.0)]);
    assert!(carried, "{} derived", derived.len());
    assert!(sk.derived_mask()[1]);
}
