//! **A drawing shown at an instant of its march** (#398 L): what an app's timeline asks of the
//! core.  `follow::stand` stands the drawing at `t` on the branch the march carried and remembers
//! home; `follow::home` puts it back, numbers and all.  Standing is presentation: the march and a
//! trace over it are of the drawing at home, so standing re-marches nothing, and no seed is written
//! back from an instant.  `follow::Scrub` drags a point along its track, the instant found where it
//! comes nearest the pointer, walked to from the one shown.
use gcs_core::follow::{followed, home, stand, Scrub};
use gcs_core::json::Json;
use gcs_core::program::Elaborated;
use gcs_core::report::{parameter_json, parameters_json};

use crate::common::parsed::determined;
use crate::continuation::closed_c;

const FOUR_BAR: &str = include_str!("../../examples/mechanisms/four_bar.sv");
const TRIPLE_ROCKER: &str = include_str!("../../examples/mechanisms/triple_rocker.sv");
/// The four-bar's ground, crank, coupler and rocker.
const G: f64 = 40.0;
const R: f64 = 10.0;
const L: f64 = 35.0;
const K: f64 = 25.0;

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

fn apart(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

fn field<'a>(j: &'a Json, k: &str) -> &'a Json {
    j.get(k).unwrap_or_else(|| panic!("no `{k}` in {j:?}"))
}

/// Stood at an instant, the four-bar is where the closed form puts it; its march and its trace are
/// the ones it had at home, not marched again; and home is where it was, to the bit.
#[test]
fn a_drawing_stood_at_an_instant_is_there_and_goes_home() {
    let mut e = determined(FOUR_BAR);
    let (c, m) = (index(&e, "c"), index(&e, "m"));
    let coupler = index(&e, "coupler");
    let at_home = e.sketch.get_x();
    let march = followed(&e.sketch, "t");
    let trace = e.sketch.curve_polyline(coupler);
    for t in [90.0, 200.0, 359.0, 0.0] {
        stand(&mut e.sketch, "t", t).expect("reached");
        let want = closed_c(G, R, L, K, t, 1.0).expect("a crank-rocker closes");
        assert!(apart(e.sketch.point_xy(c), want) < 1e-8, "at {t}");
        let (s, co) = t.to_radians().sin_cos();
        let mid = ((R * co + want.0) / 2.0, (R * s + want.1) / 2.0);
        assert!(apart(e.sketch.point_xy(m), mid) < 1e-8, "at {t}");
        assert_eq!(e.sketch.standing.as_ref().map(|s| s.at), Some(t));
        // the same march, remembered, and the same trace
        assert!(std::rc::Rc::ptr_eq(&march, &followed(&e.sketch, "t")), "re-marched at {t}");
        assert_eq!(e.sketch.curve_polyline(coupler), trace, "the trace moved at {t}");
    }
    home(&mut e.sketch);
    assert!(e.sketch.standing.is_none());
    assert_eq!(e.sketch.get_x(), at_home);
    // the driver's number is its home value again, so a solve from here solves at `t = 0`
    let r = gcs_core::solve::solve(&mut e.sketch, Default::default());
    assert!(r.success);
    let want = closed_c(G, R, L, K, 0.0, 1.0).expect("closes");
    assert!(apart(e.sketch.point_xy(c), want) < 1e-8);
}

/// An instant past where the march stopped is refused, said why, and the drawing left at home.
#[test]
fn an_instant_not_reached_is_refused_at_home() {
    let mut e = determined(TRIPLE_ROCKER);
    let at_home = e.sketch.get_x();
    stand(&mut e.sketch, "t", 100.0).expect("reached");
    let err = stand(&mut e.sketch, "t", 200.0).expect_err("past the limit");
    assert_eq!(err, "200 is not reached");
    assert!(e.sketch.standing.is_none());
    assert_eq!(e.sketch.get_x(), at_home);
    let err = stand(&mut e.sketch, "s", 1.0).expect_err("no such parameter");
    assert_eq!(err, "`s` is no parameter the drawing declares");
}

/// No seed is written back from an instant: the source says where the drawing starts.
#[test]
fn a_drawing_standing_at_an_instant_writes_no_seeds() {
    let mut e = determined(FOUR_BAR);
    stand(&mut e.sketch, "t", 90.0).expect("reached");
    let out = gcs_core::edit::commit_seeds(&e, &e.sketch, &e.program);
    assert_eq!(out.kind, gcs_core::edit::Kind::None);
    assert_eq!(out.text, FOUR_BAR);
}

/// The crank's pin dragged round: the instant is where the pin comes nearest the pointer (to a
/// thousandth of a degree: a parabola through the stations, no solve), on past a whole turn, and
/// the coupler's midpoint dragged finds the instant its closed form gives.  A fixed point does not
/// move as `t` runs and is not dragged along the march.
#[test]
fn a_point_dragged_along_its_track_finds_its_instant() {
    let e = determined(FOUR_BAR);
    let b = index(&e, "b");
    let mut scrub = Scrub::new(&e.sketch, "t", b, None).expect("the pin moves");
    // round, the pointer a little outside the pin's circle: the turn goes on past 360°
    let mut last = 0.0;
    for t in [30.0, 80.0, 150.0, 240.0, 330.0, 10.0, 60.0] {
        let (s, c) = (t as f64).to_radians().sin_cos();
        let got = scrub.to(1.2 * R * c, 1.2 * R * s);
        assert!((got - t).abs() < 1e-3, "{got} for {t}");
        last = got;
    }
    assert!((last - 60.0).abs() < 1e-3);
    // the coupler's midpoint, from a drawing already standing elsewhere
    let mut sk = e.sketch.clone();
    stand(&mut sk, "t", 100.0).expect("reached");
    let m = index(&e, "m");
    let mut scrub = Scrub::new(&sk, "t", m, None).expect("the midpoint moves");
    for t in [110.0, 140.0, 120.0] {
        let want = closed_c(G, R, L, K, t, 1.0).expect("closes");
        let (s, c) = (t as f64).to_radians().sin_cos();
        let mid = ((R * c + want.0) / 2.0, (R * s + want.1) / 2.0);
        let got = scrub.to(mid.0, mid.1);
        assert!((got - t).abs() < 1e-3, "{got} for {t}");
    }
    let a = index(&e, "a");
    let err = Scrub::new(&e.sketch, "t", a, None).expect_err("the ground pivot is fixed");
    assert_eq!(err, "it does not move as `t` runs");
}

/// A pointer past where the march stopped finds the instant it stopped at: the drag goes no
/// further than the drawing does.
#[test]
fn a_drag_stops_where_the_march_stopped() {
    let e = determined(TRIPLE_ROCKER);
    let b = index(&e, "b");
    let mut scrub = Scrub::new(&e.sketch, "t", b, None).expect("the pin moves");
    let reached = followed(&e.sketch, "t").as_ref().as_ref().expect("followed").reached();
    let mut got = 0.0;
    for t in [60.0, 120.0, 170.0, 220.0] {
        let (s, c) = (t as f64).to_radians().sin_cos();
        got = scrub.to(20.0 * c, 20.0 * s);
    }
    assert!((got - reached).abs() < 1e-9, "{got} against {reached}");
}

/// What a timeline is told: the parameters, and the march in the core's words.
#[test]
fn a_timeline_is_told_the_march_in_the_cores_words() {
    let e = determined(TRIPLE_ROCKER);
    let ps = parameters_json(&e.sketch);
    let Json::Arr(ps) = ps else { panic!("{ps:?}") };
    let p = &ps[0];
    assert_eq!(field(p, "name"), &Json::from("t"));
    assert_eq!(field(p, "dim"), &Json::from("Angle"));
    assert_eq!(field(p, "fromText"), &Json::from("0deg"));
    assert_eq!(field(p, "toText"), &Json::from("360deg"));
    let j = parameter_json(&e.sketch, "t");
    assert_eq!(field(&j, "followed"), &Json::from(true));
    assert_eq!(field(&j, "from"), &Json::from(0.0));
    assert_eq!(field(&j, "reachedText"), &Json::from("129.838deg"));
    let s = field(&j, "singular");
    assert_eq!(field(s, "kind"), &Json::from("limit"));
    assert_eq!(field(s, "text"), &Json::from("singular at t = 129.838deg: no drawing stands past \
                                              it, the motion turns back"));
    let j = parameter_json(&e.sketch, "nope");
    assert_eq!(field(&j, "followed"), &Json::from(false));
    assert_eq!(field(&j, "why"), &Json::from("`nope` is no parameter the drawing declares"));
}
