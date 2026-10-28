//! **A ball swept, built exactly** (#238): a sweep whose tool is a ball and whose motion is one
//! turn about one axis is the tube about its centre's path — the ball's disc square to the path
//! carried along it (a torus's stretch under a rotation, a loft along a helix under a screw) and
//! the ball at each end (`solid::canal`). So a cutter may stop inside the material (the groove)
//! and a sweep may stand alone (the spring, `removal.body`): each is built and exported by this
//! kernel, never the field's surface, and its volume is Pappus's to 1e-9.
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::brep::topo::Brep;
use gcs_core::program::Elaborated;
use std::f64::consts::{PI, TAU};

use crate::common::{close, ent, text::solved};

/// Solid `name` of `e` built exactly, written as STEP and as STL by this kernel (each read back and
/// checked as `solventc` checks it), and its B-rep — a static solid's, no sweep's field.
fn exported(e: &Elaborated, name: &str) -> Brep {
    let s = ent(e, name).i();
    assert!(!e.sketch.is_swept(s), "{name} is read by the field");
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = export::exact(&e.sketch, s, None, None, &say).unwrap_or_else(|r| panic!("{name}: {r:?}"));
    export::step(&exact, name, None, &say).unwrap();
    let stl = export::stl(&e.sketch, s, &exact, None, &say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
    assert!(e.sketch.exact_solid(s).is_ok(), "{name}: the app's exact solid");
    exact.solid
}

/// The tube about an arc or a helix of `length`, of radius `r`, with the ball at each end: the disc
/// carried along the path (Pappus) and the two half balls beyond its ends.
pub(crate) fn tube(r: f64, length: f64) -> f64 {
    PI * r * r * length + 4.0 / 3.0 * PI * r.powi(3)
}

/// `swept_spring.sv`: a ball of 2 on a helix of radius 12 climbing 8 a turn, two turns — a sweep
/// standing alone, its tool clear of nothing.
#[test]
fn a_spring_is_a_ball_carried_along_a_helix() {
    let e = solved(include_str!("../../examples/swept_spring.sv"));
    let b = exported(&e, "spring");
    let length = 2.0 * (TAU * 12.0f64).hypot(8.0);
    close(props::volume(&b), tube(2.0, length));
    // its faces are the tube and the two half balls, each the ball's face that made it
    let kinds: Vec<_> = b.faces.iter().map(|f| f.surface.kind()).collect();
    assert_eq!(kinds.iter().filter(|k| **k == "sphere").count(), 2, "{kinds:?}");
    assert!(b.faces.iter().all(|f| gcs_core::brep::recipe::named_parts(&f.name).1 == "meridian"),
            "{:?}", b.faces.iter().map(|f| &f.name).collect::<Vec<_>>());
}

/// `solid_generating_sweep.sv`'s `removal.body`: a ball of 1 three from the spindle, turned from
/// −60° to 60° — a torus's third and the two end balls.
#[test]
fn a_ball_turned_alone_is_a_torus_stretch_and_its_ends() {
    let e = solved(include_str!("../../examples/solid_generating_sweep.sv"));
    let b = exported(&e, "removal.body");
    close(props::volume(&b), tube(1.0, 3.0 * TAU / 3.0));
}

/// `swept_groove.sv`: the ball of 4 swung 150° at 14 from the spindle, its centre on the block's
/// top face, so the groove is half the tube: the block less it, its cutter ending inside the block
/// (once refused at admission, E1).
#[test]
fn a_groove_ends_in_the_block_it_cuts() {
    let e = solved(include_str!("../../examples/swept_groove.sv"));
    let b = exported(&e, "part");
    let groove = tube(4.0, 14.0 * 150f64.to_radians()) / 2.0;
    close(props::volume(&b), 40.0 * 12.0 * 40.0 - groove);
}

/// `solid_sweeps/ball.sv` with a ball of 2.9 swung 50° at 5 from its axis: its start's half ball
/// had its pole read a rounding past the sphere's chart, whose normal turns over there, and the
/// mesher refined into the pole without end — triangles 1e-7 across, which float32 wrote as none,
/// so the STL was refused. A revolution's sphere holds its `v` to its poles (`build::revolve`).
#[test]
fn a_half_balls_pole_is_meshed_as_any_point() {
    let doc = include_str!("../../examples/solid_sweeps/ball.sv").replace("swing_r := 12mm", "swing_r := 5mm")
        .replace("ball := 3mm", "ball := 2.9mm").replace("swing := 90deg", "swing := 50deg");
    let e = solved(&doc);
    let groove = tube(2.9, 5.0 * 50f64.to_radians()) / 2.0;
    close(props::volume(&exported(&e, "block")), 24.0 * 24.0 * 8.0 - groove);
}

/// A ball turned about an axis it stands across (the tube would meet itself there), and a helix
/// climbing less than the ball's diameter a turn: no tube, still the field's.
#[test]
fn a_tube_that_meets_itself_stays_the_fields() {
    let close_coil = include_str!("../../examples/swept_spring.sv").replace("pitch := 8mm", "pitch := 3mm");
    let e = solved(&close_coil);
    let s = ent(&e, "spring").i();
    assert!(gcs_core::solid::canal::built(&e.sketch, s).is_none());
    assert!(e.sketch.is_swept(s));
    let tight = include_str!("../../examples/swept_spring.sv").replace("coil_r := 12mm", "coil_r := 1.5mm");
    let e = solved(&tight);
    assert!(gcs_core::solid::canal::built(&e.sketch, ent(&e, "spring").i()).is_none());
}
