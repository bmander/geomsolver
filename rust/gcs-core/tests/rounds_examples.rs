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
