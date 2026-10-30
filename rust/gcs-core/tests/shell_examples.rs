//! **The shells series** (`rust/examples/shells/`): each document elaborates and solves, and each
//! body its comments say is built exactly is, to the closed form they give to 1e-9; each they say
//! is read by the field is, and an exact export refuses it, saying why.  Between them the corner
//! cases of `std.rounds`' `shell`: open through a cap and through a side, sealed; corners convex at
//! 90°, 120° and acute, reflex; arcs of a section, a whole circle, a turned band; walls leaning in
//! over an opening and out of one; a pocket and a bore cut from the body; bosses square and round
//! put on it, open at a top and sealed; a body added in the hollow; and the refusals.
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use std::f64::consts::PI;

use crate::common::{close, ent, text::solved};
use crate::shell::exact_volume;

/// Document `file` of the series, solved.
fn example(file: &str) -> Elaborated {
    let path = format!("{}/../examples/shells/{file}", env!("CARGO_MANIFEST_DIR"));
    solved(&std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}")))
}

/// The area of a polygon by its corners in order.
fn area(p: &[[f64; 2]]) -> f64 {
    (0..p.len()).map(|k| { let (a, b) = (p[k], p[(k + 1) % p.len()]); a[0] * b[1] - a[1] * b[0] }).sum::<f64>() / 2.0
}

/// `tray.sv`, `float.sv` and `drawer.sv`: a box open through its top, sealed, and open through a
/// side.
#[test]
fn boxes_open_and_sealed() {
    close(exact_volume(&example("tray.sv"), "tray"), 48000.0 - 56.0 * 36.0 * 18.0);
    close(exact_volume(&example("float.sv"), "float"), 30000.0 - 47.0 * 27.0 * 17.0);
    close(exact_volume(&example("drawer.sv"), "drawer"), 11520.0 - 36.0 * 8.0 * 22.0);
}

/// `enclosure.sv`: arcs moved in about their centres, and a post added to the body shelled, named.
#[test]
fn a_rounded_enclosure_and_its_post() {
    let e = example("enclosure.sv");
    let walls = 22835.0 + 1088.125 * PI;
    close(exact_volume(&e, "enclosure"), walls);
    close(exact_volume(&e, "case"), walls + 9.0 * PI * 17.5);
}

/// `cup.sv` and `can.sv`: a section that is one whole circle, and a turned body built by its walls.
#[test]
fn round_bodies() {
    close(exact_volume(&example("cup.sv"), "cup"), 2928.0 * PI);
    close(exact_volume(&example("can.sv"), "can"), 3576.0 * PI);
}

/// `step.sv` and `polygons.sv`: a reflex corner rounded within; corners of 120° and acute ones.
#[test]
fn corners_of_every_angle() {
    close(exact_volume(&example("step.sv"), "step"), 3600.0 - 8.0 * (160.0 - PI));
    let e = example("polygons.sv");
    let hexagon = |r: f64| 1.5 * 3f64.sqrt() * r * r;
    close(exact_volume(&e, "box"), hexagon(20.0) * 15.0 - hexagon(20.0 - 4.0 / 3f64.sqrt()) * 13.0);
    let inradius = 800.0 / (60.0 + 2000f64.sqrt());
    close(exact_volume(&e, "wedge"), 8000.0 - 400.0 * ((inradius - 2.0) / inradius).powi(2) * 16.0);
}

/// `hopper.sv`: a wall leaning in over the opening ends in a rounded lip; walls leaning out run up
/// through it.
#[test]
fn walls_leaning_in_and_out() {
    let e = example("hopper.sv");
    let s = 2f64.sqrt();
    let section = area(&[[0.0, 0.0], [30.0, 0.0], [30.0, 20.0], [10.0, 20.0], [0.0, 10.0]]);
    let moved = area(&[[2.0, 2.0], [28.0, 2.0], [28.0, 20.0], [10.0 + 2.0 * s, 20.0], [2.0, 12.0 - 2.0 * s]]);
    close(exact_volume(&e, "hopper"), 12.0 * section - 8.0 * (moved + 2.0 - PI / 2.0));
    // each slanted wall moved in by 2 is 2 √250 / 15 across
    let k = 2.0 * 250f64.sqrt() / 15.0;
    close(exact_volume(&e, "flared"), 20.0 * 525.0 - 16.0 * (13.0 * (30.0 - 2.0 * k) + 221.0 / 3.0));
}

/// `pocketed.sv`: a pocket and a bore cut from the body take their growth from the cavity.
#[test]
fn what_is_cut_is_shelled_with_the_body() {
    let e = example("pocketed.sv");
    let floor = 320.0 + 18.0 * PI + 2.0 * PI / 3.0;
    close(exact_volume(&e, "tray"), 14720.0 - (38.0 * 38.0 * 9.0 - (392.0 + PI) * 4.0 - floor));
    close(exact_volume(&e, "plate"), 8000.0 - 180.0 * PI - (38.0 * 38.0 * 3.0 - 147.0 * PI));
}

/// `plinth.sv`, `chimney.sv` and `bosses.sv`: bosses put on the body, built by its walls, each
/// foot rounded within — along a square's edges, turned round a circle.
#[test]
fn bosses_are_shelled_with_their_body() {
    let foot = 100.0 - 10.0 * PI + 8.0 / 3.0;
    close(exact_volume(&example("plinth.sv"), "plinth"), 16500.0 - (11552.0 + foot + 64.0 * 4.0));
    close(exact_volume(&example("chimney.sv"), "plinth"), 16500.0 - (11552.0 + foot + 64.0 * 5.0));
    let round_foot = PI * (25.0 + 2.0 / 3.0) - 2.5 * PI * PI;
    let cavity = 17632.0 + foot + 64.0 * 5.0 + round_foot + 16.0 * PI * 7.0;
    close(exact_volume(&example("bosses.sv"), "plate"), 24600.0 + 200.0 * PI - cavity);
}

/// `refused.sv`: each body read by the field, its exact export refused saying why.
#[test]
fn what_is_not_built_says_why() {
    let e = example("refused.sv");
    for (name, says) in [("plinth", "a wall standing on a face left open"),
                         ("step", "narrower than twice the thickness"),
                         ("tray", "none of the stock's")] {
        let Err(why) = e.sketch.exact_solid(ent(&e, name).i()) else { panic!("`{name}` built") };
        assert!(why.contains(says), "`{name}`: {why}");
        MaterialField::read(&e.sketch, ent(&e, name).i(), 1e-10).unwrap_or_else(|m| panic!("`{name}`: {m:?}"));
    }
}
