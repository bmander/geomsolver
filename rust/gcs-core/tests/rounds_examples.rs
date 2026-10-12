//! **The rounds series** (`rust/examples/rounds/`, #183): each example elaborates, solves and is
//! read by the field, with material where its comments say and none where they say not.  The
//! fillet stack's examples (#195: the pad, the knob, the rail, the tee, the bore, the slot and the
//! run-outs) are also held, in the slow tier, to the stack's closed forms and #85's quadrature:
//! each round's volume, the field's reading of the example less its reading with the rounds left
//! out, within the field's bar.
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use gcs_core::solve::{solve, SolveOpts};
use std::f64::consts::PI;

use crate::common::{ent, read};
use crate::fillet_definition::{bore_rim, root_ring, sloped, sloped_root, tee_crotch, RIM, ROOT};
use crate::fillet_meridian::knob_neck;
use crate::region_general::field_volume;

fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    e
}

/// Where solid `name` of `src` has material and where it has none.
fn holds(src: &str, name: &str, inside: &[[f64; 3]], outside: &[[f64; 3]]) -> Elaborated {
    let e = solved(src);
    let field = MaterialField::read(&e.sketch, ent(&e, name).i(), 1e-10).unwrap();
    for p in inside {
        assert!(field.side(*p) < 0.0, "{name}: {p:?} is material");
    }
    for p in outside {
        assert!(field.side(*p) > 0.0, "{name}: {p:?} is not");
    }
    e
}

/// `block.sv`: the near rim's corner rounded away, the far rim's chamfered, the middle kept.  The
/// block stands from y = 0 (its near face) to 12.
#[test]
fn the_block_is_rounded_and_chamfered() {
    holds(include_str!("../../examples/rounds/block.sv"), "block",
          &[[20.0, 6.0, 6.0], [20.0, 1.0, 1.0], [20.0, 11.0, 2.0]],
          &[[20.0, 0.2, 0.2], [0.2, 0.2, 12.0], [20.0, 11.8, 0.2], [39.8, 11.8, 12.0]]);
}

/// `pocket.sv`: the pocket's floor (6 deep) filled along its walls, its middle still empty.
#[test]
fn the_pocket_is_filled_inside() {
    holds(include_str!("../../examples/rounds/pocket.sv"), "dish",
          &[[30.0, 5.8, 10.2], [15.2, 5.8, 20.0], [15.2, 5.8, 10.2], [5.0, 5.0, 5.0]],
          &[[30.0, 3.0, 15.0], [30.0, 5.0, 20.0], [30.0, 5.8, 12.0]]);
}

/// `boss.sv`: the boss (in front of the plate, y < 0) filleted at its root, the ball's own place
/// beside it still empty.
#[test]
fn the_bosss_root_is_filleted() {
    holds(include_str!("../../examples/rounds/boss.sv"), "pad",
          &[[30.0, -0.3, 28.3], [30.0, -10.0, 20.0], [5.0, 5.0, 5.0]],
          &[[30.0, -2.5, 30.5], [30.0, -10.0, 29.0], [5.0, -5.0, 5.0]]);
}

/// `by_hand.sv`: the one edge the word written in the file names rounded, its neighbours sharp.
#[test]
fn the_round_written_by_hand_rounds_one_edge() {
    holds(include_str!("../../examples/rounds/by_hand.sv"), "block",
          &[[20.0, 6.0, 6.0], [20.0, 0.2, 23.8], [0.2, 0.2, 12.0]],
          &[[20.0, 0.2, 0.2], [20.0, 0.5, 0.5]]);
}

/// `disc.sv`: the top rim rounded and the bottom chamfered, built exactly in the meridian.
#[test]
fn the_disc_is_finished_exactly() {
    let e = holds(include_str!("../../examples/rounds/disc.sv"), "disc",
                  &[[0.0, 0.0, 2.0], [9.9, 0.0, 2.0], [9.0, 0.0, 3.5]],
                  &[[9.9, 0.0, 3.9], [9.9, 0.0, 0.1], [10.1, 0.0, 2.0]]);
    let disc = ent(&e, "disc").i();
    assert!(!gcs_core::solid::region::in_general(&e.sketch, disc));
    assert!(e.sketch.exact_solid(disc).is_ok(), "lowered into the meridian");
}

/// `regions.sv`: the egg by its foci, the bicylinder and the Voronoi cell, each read point by point.
#[test]
fn the_regions_are_read_point_by_point() {
    let src = include_str!("../../examples/rounds/regions.sv");
    let e = holds(src, "egg", &[[4.9, 0.0, 0.0], [0.0, 3.9, 0.0]], &[[5.1, 0.0, 0.0], [0.0, 0.0, 4.1]]);
    holds(src, "bicylinder", &[[20.0, 2.9, 0.0], [21.5, 0.0, 1.5]], &[[20.0, 2.0, 2.5], [20.0, 3.1, 0.0]]);
    holds(src, "cell", &[[39.0, 0.5, 0.5], [36.0, 0.0, 0.0]], &[[41.0, 0.5, 0.5], [34.0, 0.0, 0.0]]);
    for name in ["egg", "bicylinder", "cell"] {
        assert!(gcs_core::solid::region::in_general(&e.sketch, ent(&e, name).i()), "{name}");
    }
}

/// The edits the examples' comments suggest do what they say: the pocket's rim rounded instead of
/// its inside, the boss's top rim beside its root, the hand-written word over a whole rim, and the
/// disc's wall tilted out of the planes' axis, read point by point and still rounded.
#[test]
fn the_suggested_edits_do_what_they_say() {
    let pocket = include_str!("../../examples/rounds/pocket.sv")
        .replace("round(r: r) dish.pocket", "dish.pocket round(r: r) dish.tray.near");
    holds(&pocket, "dish", &[[14.0, 1.0, 20.0], [5.0, 5.0, 5.0]], &[[15.2, 0.2, 20.0], [30.0, 5.8, 10.2]]);
    let boss = include_str!("../../examples/rounds/boss.sv").to_string() + "round(r: 1mm) pad.boss\n";
    holds(&boss, "pad", &[[30.0, -0.3, 28.3], [30.0, -19.0, 20.0]], &[[30.0, -19.8, 27.8]]);
    let hand = include_str!("../../examples/rounds/by_hand.sv")
        .replace("block.near rolled(r: r) block.south", "block.near rolled(r: r) block");
    holds(&hand, "block", &[[20.0, 6.0, 6.0]], &[[20.0, 0.2, 23.8], [0.2, 0.2, 12.0], [20.0, 0.2, 0.2]]);
    let disc = include_str!("../../examples/rounds/disc.sv")
        .replace("fix((0, 0, 1)) ze", "fix((0.2, 0, 1)) ze").replace("hint((0, 0, 1))", "hint((0.2, 0, 1))");
    let e = holds(&disc, "disc", &[[0.4, 0.0, 2.0]], &[[10.6, 0.0, 3.9]]);
    assert!(gcs_core::solid::region::in_general(&e.sketch, ent(&e, "disc").i()));
}

/// A sketch on the block's far face (a plane on the surface it lies on) leaves its rounds and
/// chamfers as they were: the face is one thing to both.
#[test]
fn a_sketch_on_a_rounded_face_leaves_the_round() {
    let src = format!("{}v1 := plane\nv1.origin coincident block.far.surface\n\
                       v1 tangent(at: v1.origin) block.far.surface\n",
                      include_str!("../../examples/rounds/block.sv"));
    holds(&src, "block", &[[20.0, 6.0, 6.0], [20.0, 11.0, 2.0]],
          &[[20.0, 0.2, 0.2], [20.0, 11.8, 0.2]]);
}

// -- the fillet stack's examples (#195) ----------------------------------------------------------

const PAD: &str = include_str!("../../examples/rounds/pad.sv");
const KNOB: &str = include_str!("../../examples/rounds/knob.sv");
const RAIL: &str = include_str!("../../examples/rounds/rail.sv");
const TEE: &str = include_str!("../../examples/rounds/tee.sv");
const BORE: &str = include_str!("../../examples/rounds/bore.sv");
const SLOT: &str = include_str!("../../examples/rounds/slot.sv");
const RUNOUT: &str = include_str!("../../examples/rounds/runout.sv");

/// The field's bar: a round's volume read off the field within this of the closed form's.
const BAR: f64 = 2e-3;

/// `src` solved with its rounds left out (every line saying ` round(`), and as it stands.
fn rounding(src: &str) -> [Elaborated; 2] {
    let unrounded: String = src.lines().filter(|l| !l.contains(" round("))
        .map(|l| format!("{l}\n")).collect();
    [solved(&unrounded), solved(src)]
}

/// The volume solid `name` loses to its rounds within the box `lo`–`hi` (negative where they
/// add), each reading by the field down to cells of `h`.
fn taken(both: &[Elaborated; 2], name: &str, lo: [f64; 3], hi: [f64; 3], h: f64) -> f64 {
    let [before, after] = both.each_ref().map(|e| {
        field_volume(&MaterialField::read(&e.sketch, ent(e, name).i(), 1e-10).unwrap(), lo, hi, h)
    });
    before - after
}

fn agrees(what: &str, got: f64, want: f64) {
    eprintln!("{what}: {got:.6} against {want:.6}, {:+.2e}", (got - want) / want);
    assert!((got - want).abs() <= BAR * want.abs(), "{what}: {got} against {want}");
}

/// A straight run's section: the corner a ball of radius `r` leaves between two faces square.
fn square_run(r: f64) -> f64 { (1.0 - PI / 4.0) * r * r }

/// `pad.sv`: the boss's root (concave, Pappus's ring) and the plate's two side edges (convex, each
/// a straight run cut off flush by the faces it ends on); the bore cut after, by neither.
#[test]
fn the_pad_is_filled_at_its_root_and_rounded_at_its_sides() {
    holds(PAD, "pad",
          &[[9.0, -0.5, 0.0], [0.0, 5.0, 10.0], [0.0, 0.2, 19.8]],
          &[[-29.8, 0.2, 0.0], [29.8, 0.2, 0.0], [-29.8, 0.2, 19.8], [10.5, -2.5, 0.0],
            [0.0, -5.0, 0.0], [0.0, 5.0, 0.0]]);
    // the suggested edit: the top edge too, mitred into the side's at the corner
    let top = format!("{PAD}pad.plate.near round(r: edge_radius) pad.plate.cd\n");
    holds(&top, "pad", &[[0.0, 5.0, 10.0], [-29.0, 1.5, 19.0]],
          &[[0.0, 0.2, 19.8], [-29.8, 0.2, 19.8]]);
}

#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier: the pad's three rounds, by the field")]
fn the_pads_rounds_are_the_stacks() {
    let pad = rounding(PAD);
    let root = -taken(&pad, "pad", [-12.0, -4.0, -12.0], [12.0, 1.0, 12.0], 3.0 / 32.0);
    agrees("the pad's root", root, root_ring(8.0, 3.0));
    for (side, x) in [("left", -31.0), ("right", 27.0)] {
        agrees(&format!("the pad's {side} edge"),
               taken(&pad, "pad", [x, -1.0, -21.0], [x + 4.0, 3.0, 21.0], 1.0 / 32.0),
               square_run(2.0) * 40.0);
    }
}

/// `knob.sv`: the rod's root on the disc (plane against cylinder, Pappus's ring) and its neck where
/// it enters the ball (cylinder against sphere: the section between the rod, the rolled ball and
/// the sphere, turned).
#[test]
fn the_knobs_root_and_neck_are_filled() {
    holds(KNOB, "knob",
          &[[4.2, 0.0, 8.2], [0.0, 4.1, 36.95], [0.0, 0.0, 20.0], [0.0, 20.0, 4.0],
            [29.9, 0.0, 7.9]],
          &[[0.0, 5.0, 9.5], [4.4, 0.0, 36.6]]);
    // the suggested edit: the disc's own rims, convex, cut
    let rims = format!("{KNOB}round(r: 1mm) knob.disc\n");
    holds(&rims, "knob", &[[4.2, 0.0, 8.2], [0.0, 25.0, 4.0]],
          &[[29.9, 0.0, 7.9], [0.0, 29.9, 0.1]]);
}

#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier: the knob's two rounds, by the field")]
fn the_knobs_rounds_are_the_stacks() {
    let knob = rounding(KNOB);
    let root = -taken(&knob, "knob", [-9.0, -9.0, 7.0], [9.0, 9.0, 15.0], 2.0 / 32.0);
    agrees("the knob's root", root, root_ring(4.0, 2.0));
    let neck = -taken(&knob, "knob", [-7.0, -7.0, 34.0], [7.0, 7.0, 39.0], 1.5 / 32.0);
    agrees("the knob's neck", neck, knob_neck([0.0, 44.0], 8.0, 4.0, 1.5));
}

/// `rail.sv`: both sides of the rod filled, each run's section the corner between the rod's
/// circle and the plate's top a ball of radius `r` cannot reach, the plate's whole length.
#[test]
fn the_rails_sides_are_filled() {
    holds(RAIL, "rail", &[[23.9, -20.0, 10.2], [36.1, -0.1, 10.2], [30.0, -20.0, 15.0]],
          &[[23.0, -20.0, 10.2], [22.0, -20.0, 10.5], [23.9, 0.1, 10.2], [23.9, -40.1, 10.2]]);
    // the suggested edit: the plate's top alone, the same two runs
    let top = RAIL.replace("rail.rod round(r: r) rail.plate", "rail.rod round(r: r) rail.plate.cd");
    holds(&top, "rail", &[[23.9, -20.0, 10.2], [36.1, -0.1, 10.2]], &[[23.0, -20.0, 10.2]]);
}

#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier: the rail's two runs, by the field")]
fn the_rails_runs_are_the_stacks() {
    let (big, r): (f64, f64) = (6.0, 2.0);
    // the ball touches the plate `r` up and the rod `big + r` from its axis: the section is the
    // triangle of its centre, its foot and the axis, less the rod's sector and the ball's
    let theta = (r / (big + r)).asin();
    let section = ((big + r).powi(2) - r * r).sqrt() * r / 2.0 - big * big * theta / 2.0
        - r * r * (PI / 2.0 - theta) / 2.0;
    let got = -taken(&rounding(RAIL), "rail", [20.0, -41.0, 9.0], [40.0, 1.0, 15.0], r / 32.0);
    agrees("the rail's runs", got, 2.0 * 40.0 * section);
}

/// `tee.sv`: the crotch filled round the saddle, nothing where the cylinders' surfaces meet below
/// the main pipe's axis.
#[test]
fn the_tees_crotch_is_filled() {
    holds(TEE, "tee",
          &[[6.1, 0.0, 10.1], [0.0, 6.1, 8.1], [0.0, 0.0, 20.0]],
          &[[7.0, 0.0, 11.0], [0.0, 7.0, 9.0], [0.0, 6.1, -8.1], [6.5, 0.0, -10.1]]);
}

#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier: the tee's crotch, by the field")]
fn the_tees_crotch_is_the_stacks() {
    let got = -taken(&rounding(TEE), "tee", [-10.0, -10.0, 4.0], [10.0, 10.0, 14.0], 2.0 / 64.0);
    agrees("the tee's crotch", got, tee_crotch(10.0, 6.0, 2.0));
}

/// `bore.sv`: the bore's rim rounded off where it meets the pipe's wall, its floor sharp.
#[test]
fn the_bores_rim_is_rounded() {
    holds(BORE, "pipe", &[[6.6, 0.0, 7.0], [0.0, 6.5, 5.0], [5.8, 0.0, -0.2]],
          &[[6.1, 0.0, 9.9], [0.0, 6.1, 7.9], [5.8, 0.0, 0.2]]);
    // the suggested edit: the floor's edge too, concave, filled
    let floor = format!("{BORE}round(r: 1mm) pipe.bore\n");
    holds(&floor, "pipe", &[[5.8, 0.0, 0.2]], &[[6.1, 0.0, 9.9], [5.0, 0.0, 0.5]]);
}

#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier: the bore's rim, by the field")]
fn the_bores_rim_is_the_stacks() {
    let got = taken(&rounding(BORE), "pipe", [-10.0, -10.0, 4.0], [10.0, 10.0, 14.0], 1.0 / 32.0);
    agrees("the bore's rim", got, bore_rim(10.0, 6.0, 1.0));
}

/// `slot.sv`: the boss's foot filled along its sides and round its ends with no step at the
/// seams, the pocket's rim cut along its sides and round its corners, the pocket's floor sharp.
#[test]
fn the_slots_chains_run_through_their_seams() {
    // at the seams and round the ends, 0.3 from the plate's face and the boss's wall
    let seam = |deg: f64| {
        let t = deg.to_radians();
        [29.0 + 6.3 * t.cos(), -0.3, 25.0 + 6.3 * t.sin()]
    };
    holds(SLOT, "part",
          &[[20.0, -0.3, 18.7], [28.9, -0.3, 18.7], seam(-89.0), seam(-80.0), seam(0.0),
            seam(85.0)],
          &[[20.0, -1.5, 17.5], [47.9, 0.1, 25.0], [58.0, 0.1, 16.9], [66.9, 0.1, 31.9],
            [50.0, 0.5, 25.0], [48.1, 5.9, 25.0]]);
}

#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier: the slot's two chains, by the field")]
fn the_slots_chains_are_the_stacks() {
    let slot = rounding(SLOT);
    // the foot: two straight sides 18 long and the two half rings, one ring in all
    let foot = -taken(&slot, "part", [2.0, -2.5, 14.0], [38.0, 0.5, 36.0], 2.0 / 32.0);
    agrees("the boss's foot", foot, square_run(2.0) * 36.0 + root_ring(6.0, 2.0));
    // the rim: four sides, 12 and 8 long, and four quarter rings about the pocket's corners
    let rim = taken(&slot, "part", [46.0, -0.5, 15.0], [70.0, 2.0, 35.0], 1.0 / 32.0);
    agrees("the pocket's rim", rim, square_run(1.0) * 40.0 + root_ring(4.0, 1.0));
}

/// `runout.sv`: the boss's root filled on the slope and cut off by the near face where it runs
/// out, the boss's own overhang sharp; the vent's rim rounded and cut off by the end face.
#[test]
fn the_runouts_end_on_the_faces_they_run_onto() {
    holds(RUNOUT, "lug",
          &[[26.2, -3.0, 13.6], [20.0, -9.2, 15.2], [25.4, -0.1, 13.75], [26.1, -1.0, 13.6]],
          &[[26.1, 0.5, 13.6], [25.4, 0.1, 13.75], [27.0, -3.0, 14.5]]);
    holds(RUNOUT, "vent",
          &[[93.5, -12.0, 11.0], [99.5, -17.5, 8.0]],
          &[[93.9, -12.0, 11.4], [98.0, -16.1, 10.4], [99.9, -15.54, 9.34]]);
}

#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier: four run-outs, by the field")]
fn the_runouts_are_the_stacks() {
    let lug = ([10.0, -14.0, 8.0], [30.0, 3.0, 22.0]);
    let vent = ([84.0, -18.0, 7.0], [102.0, -6.0, 15.0]);
    let runout = rounding(RUNOUT);
    // the root run past the near face (`y = 0`) and cut off there, at a slant
    agrees("the lug's root", -taken(&runout, "lug", lug.0, lug.1, 2.0 / 32.0),
           sloped(6.0, [20.0, -3.0], [20.0, 0.25], 2.0, ROOT, Some(([0.0, 1.0, 0.0], 0.0))));
    // the rim run past the end face (`x = 100`) and cut off there
    agrees("the vent's rim", taken(&runout, "vent", vent.0, vent.1, 1.0 / 64.0),
           sloped(4.0, [98.0, -12.0], [35.0, 0.25], 1.0, RIM, Some(([1.0, 0.0, 0.0], 100.0))));
    // the suggested edits: the boss on the near face, its root cut in half; the hole in from the
    // end, its rim the whole way round
    let halved = rounding(&RUNOUT.replace("fix((20, -3)) bc", "fix((20, 0)) bc"));
    agrees("the lug's root, halved", -taken(&halved, "lug", lug.0, lug.1, 2.0 / 32.0),
           sloped_root(6.0, [20.0, 0.0], [20.0, 0.25], 2.0) / 2.0);
    let whole = rounding(&RUNOUT.replace("fix((98, -12)) hc", "fix((90, -12)) hc"));
    agrees("the vent's rim, whole", taken(&whole, "vent", vent.0, vent.1, 1.0 / 64.0),
           sloped(4.0, [90.0, -12.0], [35.0, 0.25], 1.0, RIM, None));
}
