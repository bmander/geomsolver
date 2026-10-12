//! **The rounds series** (`rust/examples/rounds/`, #183): each example elaborates, solves and is
//! read by the field, with material where its comments say and none where they say not.
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use gcs_core::solve::{solve, SolveOpts};

use crate::common::{ent, read};

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

/// `slot.sv`: the boss's foot (in front of the plate, y < 0) filled along a flat side, round a half
/// circle, and across the seam between them, where the flat side goes on tangent into the half
/// circle and nothing ends either round; the pocket's rim taken away, its void kept.
#[test]
fn the_slots_rounds_go_on_across_its_seams() {
    holds(include_str!("../../examples/rounds/slot.sv"), "part",
          &[[20.0, -0.3, 18.5], [29.5, -0.3, 18.5], [35.5, -0.3, 25.0], [40.0, 5.0, 10.0]],
          &[[36.5, -0.3, 25.0], [58.0, 3.0, 25.0], [58.0, 0.1, 16.9], [67.6, 0.1, 31.6]]);
}

/// `rail.sv`: the rod (in front of the plane, y < 0, along it) filled beside both its sides where
/// it stands on the plate, the ball's own place beyond still empty.
#[test]
fn the_rods_sides_are_filled() {
    holds(include_str!("../../examples/rounds/rail.sv"), "rail",
          &[[36.2, -20.0, 10.2], [23.8, -20.0, 10.2], [30.0, -20.0, 15.0], [10.0, -20.0, 5.0]],
          &[[38.5, -20.0, 10.5], [21.5, -20.0, 10.5], [30.0, -20.0, 16.5]]);
}

/// `knob.sv`: turned about the z axis — the rod's root filled on the disc, its neck filled where
/// it enters the ball, the rolled balls' own places beside each still empty.
#[test]
fn the_knobs_root_and_neck_are_filled() {
    holds(include_str!("../../examples/rounds/knob.sv"), "knob",
          &[[4.2, 0.0, 8.2], [4.1, 0.0, 37.0], [0.0, 0.0, 20.0], [20.0, 0.0, 4.0], [0.0, 0.0, 51.0]],
          &[[5.5, 0.0, 9.5], [4.6, 0.0, 36.6], [10.0, 0.0, 20.0]]);
}

/// `tee.sv`: the crotch filled where the branch (about z) stands on the main pipe (about x), the
/// ball's own place still empty; `bore.sv`: the bore's rim taken away, the ball's place kept.
#[test]
fn the_tees_crotch_is_filled_and_the_bores_rim_rounded() {
    holds(include_str!("../../examples/rounds/tee.sv"), "tee",
          &[[6.2, 0.0, 10.2], [0.0, 0.0, 20.0], [20.0, 0.0, 0.0]],
          &[[7.5, 0.0, 11.5], [10.0, 0.0, 15.0]]);
    holds(include_str!("../../examples/rounds/bore.sv"), "pipe",
          &[[7.0, 0.0, 9.5], [0.0, 0.0, -5.0], [20.0, 0.0, 9.5]],
          &[[6.2, 0.0, 9.8], [0.0, 0.0, 5.0]]);
}

/// `runout.sv`: the boss's root filled behind it on the slope, the hole's rim taken away where it
/// meets the slope, each wedge whole elsewhere.
#[test]
fn the_runouts_root_is_filled_and_rim_rounded() {
    let src = include_str!("../../examples/rounds/runout.sv");
    holds(src, "lug", &[[20.0, -9.2, 15.2], [10.0, -12.0, 5.0], [20.0, -3.0, 25.0]],
          &[[20.0, -10.5, 16.5], [10.0, -12.0, 19.0]]);
    holds(src, "vent", &[[90.0, -12.0, 5.0], [93.0, -12.0, 10.0]],
          &[[93.85, -12.0, 11.4], [98.0, -12.0, 5.0]]);
}

/// `pad.sv`: the boss's root filled, two of the plate's near edges rounded and the other two
/// sharp, the bore through it all.
#[test]
fn the_pad_is_filled_rounded_and_bored() {
    holds(include_str!("../../examples/rounds/pad.sv"), "pad",
          &[[8.3, -0.3, 0.0], [0.0, -10.0, 6.0], [-29.0, 1.0, 0.0], [0.0, 0.2, -19.8]],
          &[[10.5, -0.3, 0.0], [-29.8, 0.2, 0.0], [29.8, 0.2, 0.0], [0.0, 5.0, 0.0], [0.0, -10.0, 2.0]]);
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
