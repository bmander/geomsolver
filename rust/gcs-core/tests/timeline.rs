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

/// The arm of #633 drawn in a frame whose value is `t`, and a post it swings into: nothing the
/// drawing states reads `t` but the placement (#697).
const SWUNG: &str = "\
unit mm
use std
use std.place (vertical, coords, on_u)
use std.motion (turns)
use std.shapes
param t: Angle over (0deg, 90deg)
in std.front {
  construction pivot := vertical line(std.origin, hint((0, 10)))
  length(10mm) pivot
}
in std.top {
  ac := point hint((20, 0))
  ac on_u(d: 20mm) std.top
  arm_r := std.shapes.CenteredRectangle(ac, w: 40mm, h: 6mm)
  pc := point hint((25, 25))
  pc coords(du: 25mm, dv: 25mm) std.top
  post_r := radius(4mm) circle(center: pc)
}
arm := solid(face(arm_r.loop), from: 0mm, to: 5mm)
post := solid(face(post_r), from: 0mm, to: 5mm)
swing := frame
swing turns(1) pivot
fix(value == t) swing
shown := arm in swing
claim shown clear(0.5mm) post
";

/// How far `shown` stands from the post where `sk` stands: the lone claim's measure.
fn room(sk: &gcs_core::model::Sketch) -> f64 {
    let v = gcs_core::diagnose::judge_solids(sk);
    v[0].measured().unwrap_or_else(|| panic!("{:?}", v[0]))
}

/// **A solid drawn in a valued frame stands where the value puts it** (#697): a drawing whose only
/// reader of `t` is such a placement is followed, a march that moves the held number alone; stood
/// at a quarter of the swing's way the arm is in the post, as the claim over `t` found (#633), and
/// home it is clear again.  Stood anywhere, the arm is the solid built at home and the swung arm
/// that one moved — never built again — and the move puts it where a fresh build does.
#[test]
fn a_solid_in_a_valued_frame_stands_where_its_value_puts_it() {
    let mut e = crate::common::read(SWUNG);
    let r = gcs_core::solve::solve(&mut e.sketch, Default::default());
    assert!(r.success, "{}", r.message);
    assert_eq!(gcs_core::report::parameter_text(&e.sketch, "t"), "t: followed from 0deg to 90deg");
    let (arm, shown) = (index(&e, "arm"), index(&e, "shown"));
    let mesh = gcs_core::solid::ApproximationPolicy::Mesh;
    let built = |sk: &gcs_core::model::Sketch, i: usize| sk.evaluated_solid(i, mesh).unwrap();
    let (arm_home, shown_home) = (built(&e.sketch, arm), built(&e.sketch, shown));
    assert!((room(&e.sketch) - 18.0).abs() < 0.01, "at rest the arm is clear");
    for t in [45.0, 10.0, 90.0, 60.0] {
        stand(&mut e.sketch, "t", t).expect("reached");
        let now = built(&e.sketch, shown);
        assert!(std::rc::Rc::ptr_eq(&built(&e.sketch, arm), &arm_home), "`arm` built again at {t}");
        assert!(now.same_shape(&shown_home) && !std::rc::Rc::ptr_eq(&now, &shown_home),
                "`shown` at {t}: moved, not built again");
        let fresh = e.sketch.clone();
        fresh.caches.clear();
        let fresh = built(&fresh, shown);
        let (a, b) = (now.world_bounds(), fresh.world_bounds());
        for k in 0..3 {
            let off = (a.lo[k] - b.lo[k]).abs().max((a.hi[k] - b.hi[k]).abs());
            assert!(off < 1e-9, "`shown` at {t}: {a:?} against a fresh build's {b:?}");
        }
    }
    stand(&mut e.sketch, "t", 45.0).expect("reached");
    assert!((room(&e.sketch) + 5.0).abs() < 0.01, "the arm in the post: {}", room(&e.sketch));
    home(&mut e.sketch);
    assert!((room(&e.sketch) - 18.0).abs() < 0.01, "home, the arm is clear again");
    assert!(std::rc::Rc::ptr_eq(&built(&e.sketch, shown), &shown_home), "home is where it was");
    // a sheet of the model at an instant draws the arm there
    let compile = |at: &str| gcs_core::drawing::compile(&format!("model m from \"swung.sv\"{at}
        sheet s {{
          view top(m.shown) from top at (60mm, 60mm)
        }}"), "drawing.svd", None,
        &mut |path, _| (path == "swung.sv").then(|| ("swung.sv".into(), SWUNG.into())));
    let (at_home, swung) = (compile("").unwrap(), compile(" at t = 45deg").unwrap());
    assert_ne!(at_home, swung, "the arm is drawn where it stands at 45deg");
    assert_eq!(at_home, compile(" at t = 0deg").unwrap(), "and at home at 0deg");
}
