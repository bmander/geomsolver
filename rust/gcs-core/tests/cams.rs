//! **Cams cut from their laws** (#483, the corpus of #398 G): each document of
//! `rust/examples/` whose cam is the envelope of a follower driven by a law of the cam's turn,
//! held against the closed form of that law — the pitch curve and the profile at every degree,
//! never against the core's own reading of the curve.

use std::f64::consts::PI;

use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};

use crate::common::build;

fn solved(src: &str) -> Elaborated {
    let mut e = build(src);
    assert!(e.ok(), "{:?}", e.diags);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    e
}

fn dof(e: &mut Elaborated) -> i64 {
    diagnose(&mut e.sketch, DiagnoseOptions::default()).dof
}

fn curve(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

/// `p` turned by `deg` degrees about the origin.
fn turn(p: [f64; 2], deg: f64) -> [f64; 2] {
    let (s, c) = deg.to_radians().sin_cos();
    [c * p[0] - s * p[1], s * p[0] + c * p[1]]
}

fn apart(a: [f64; 2], b: (f64, f64)) -> f64 {
    (a[0] - b.0).hypot(a[1] - b.1)
}

/// The point a roller's radius `rr` off a pitch curve at `p`, its rate `dp`, square to it on the
/// side toward the cam's centre (the origin): the profile a roller cuts, its closed form.
fn inward(p: [f64; 2], dp: [f64; 2], rr: f64) -> [f64; 2] {
    let n = dp[0].hypot(dp[1]);
    let mut w = [dp[1] / n, -dp[0] / n];
    if w[0] * p[0] + w[1] * p[1] > 0.0 {
        w = [-w[0], -w[1]];
    }
    [p[0] + rr * w[0], p[1] + rr * w[1]]
}

/// The cycloidal rise of `h` over `beta` degrees from `from`, at `t` degrees: its lift and its
/// rate a radian, held at either end (a dwell before it, the whole lift after).
fn cycloidal(h: f64, from: f64, beta: f64, t: f64) -> (f64, f64) {
    let x = ((t - from) / beta).clamp(0.0, 1.0);
    let rate = if (0.0..1.0).contains(&((t - from) / beta)) {
        h * (1.0 - (2.0 * PI * x).cos()) / beta.to_radians()
    } else {
        0.0
    };
    (h * (x - (2.0 * PI * x).sin() / (2.0 * PI)), rate)
}

/// **`dwell_cam.sv`**: the follower dwells to 60°, rises 12 on the cycloidal law to 180°, dwells
/// to 240° and returns to 360°.  Each stretch of the pitch curve is the roller centre at that
/// lift, `base + s` out along the follower's line turned back by the cam (1e-9), and each stretch
/// of the profile the roller's radius off it, square to it (1e-9); the dwells are arcs about the
/// cam's centre, and the stretches meet where the law turns its corners.
#[test]
fn the_dwell_cam_is_its_cycloidal_law() {
    let mut e = solved(include_str!("../../examples/generation/dwell_cam.sv"));
    assert_eq!(dof(&mut e), 0);
    let (base, h, rr) = (30.0, 12.0, 8.0);
    let law = |t: f64| {
        let (up, rate_up) = cycloidal(h, 60.0, 120.0, t);
        let (down, rate_down) = cycloidal(h, 240.0, 120.0, t);
        (up - down, rate_up - rate_down)
    };
    // the follower's line along the view's `u`, turned back by the cam's turn
    let pitch = |t: f64| {
        let (s, ds) = law(t);
        let r = base + s;
        let along = turn([1.0, 0.0], -t);
        let across = turn([0.0, -1.0], -t);
        ([r * along[0], r * along[1]],
         [ds * along[0] + r * across[0], ds * along[1] + r * across[1]])
    };
    let stretches = [("pitch_low", "low", 0.0, 60.0), ("pitch_rise", "rise", 60.0, 180.0),
                     ("pitch_high", "high", 180.0, 240.0),
                     ("pitch_return", "return", 240.0, 360.0)];
    let (mut worst_p, mut worst_c) = (0.0_f64, 0.0_f64);
    for (p_name, c_name, a, b) in stretches {
        let (pc, cc) = (curve(&e, p_name), curve(&e, c_name));
        assert_eq!(e.sketch.curve_domain(pc), (a, b), "{p_name}");
        for k in 0..=(b - a) as usize {
            let t = a + k as f64;
            let (p, dp) = pitch(t);
            worst_p = worst_p.max(apart(p, e.sketch.curve_point(pc, t)));
            worst_c = worst_c.max(apart(inward(p, dp, rr), e.sketch.curve_point(cc, t)));
        }
    }
    assert!(worst_p < 1e-9, "the pitch curve is off by {worst_p}");
    assert!(worst_c < 1e-9, "the profile is off by {worst_c}");
    // the dwells: arcs of `base - rr` and `base + h - rr` about the cam's centre
    for (name, r) in [("low", base - rr), ("high", base + h - rr)] {
        let c = curve(&e, name);
        let (a, b) = e.sketch.curve_domain(c);
        for k in 0..=12 {
            let q = e.sketch.curve_point(c, a + (b - a) * k as f64 / 12.0);
            assert!((q.0.hypot(q.1) - r).abs() < 1e-9, "{name} is no arc of {r}");
        }
    }
    // the stretches meet: the cycloidal law joins its dwells with no step in lift or speed
    for (end, start, t) in [("low", "rise", 60.0), ("rise", "high", 180.0),
                            ("high", "return", 240.0)] {
        let (p, q) = (e.sketch.curve_point(curve(&e, end), t),
                      e.sketch.curve_point(curve(&e, start), t));
        assert!((p.0 - q.0).hypot(p.1 - q.1) < 1e-9, "{end} and {start} part at {t}");
    }
}

/// **`rocker_cam.sv`**: the first driven *turn*.  The arm swings about its pivot by the simple
/// harmonic law `10° (1 - cos t)`, so the roller's centre stands on the pivot's arc at the law's
/// angle, and seen from the cam that point turned back by the cam's turn is the pitch curve
/// (1e-9); the profile is the roller's radius off it, square to it (1e-9).
#[test]
fn the_rocker_cam_is_its_arms_swing() {
    let mut e = solved(include_str!("../../examples/generation/rocker_cam.sv"));
    assert_eq!(dof(&mut e), 0);
    let (pivot, c0, rr) = ([30.0, 40.0], [30.0, 0.0], 7.0);
    // the roller's centre in the page at the turn `t` and its rate a radian, then seen from the
    // cam: turned back by `t`
    let pitch = |t: f64| {
        let phi = 10.0 * (1.0 - t.to_radians().cos());
        let dphi = 10f64.to_radians() * t.to_radians().sin();
        let arm = turn([c0[0] - pivot[0], c0[1] - pivot[1]], phi);
        let c = [pivot[0] + arm[0], pivot[1] + arm[1]];
        let dc = [-arm[1] * dphi, arm[0] * dphi];
        let p = turn(c, -t);
        // d/dt R(-t) c = R(-t) (dc - J c), J the quarter turn
        let dp = turn([dc[0] + c[1], dc[1] - c[0]], -t);
        (p, dp)
    };
    let (pc, cc) = (curve(&e, "pitch"), curve(&e, "profile"));
    assert_eq!(e.sketch.curve_domain(pc), (0.0, 360.0));
    let (mut worst_p, mut worst_c) = (0.0_f64, 0.0_f64);
    for k in 0..=360 {
        let t = k as f64;
        let (p, dp) = pitch(t);
        worst_p = worst_p.max(apart(p, e.sketch.curve_point(pc, t)));
        worst_c = worst_c.max(apart(inward(p, dp, rr), e.sketch.curve_point(cc, t)));
    }
    assert!(worst_p < 1e-9, "the pitch curve is off by {worst_p}");
    assert!(worst_c < 1e-9, "the profile is off by {worst_c}");
    // at the top of the swing the roller is out 40·sin 20° and up 40·(1 - cos 20°)
    let p = e.sketch.curve_point(pc, 180.0);
    let (s, c) = 20f64.to_radians().sin_cos();
    let top = (30.0 + 40.0 * s).hypot(40.0 * (1.0 - c));
    assert!((p.0.hypot(p.1) - top).abs() < 1e-9, "the swing tops out at {}", p.0.hypot(p.1));
}

/// The cycloidal rise of `h` over the first half turn and return over the second, at `t`
/// degrees: the lift and its first two rates a radian (`flat_cam.sv`'s law).
fn rise_and_return(h: f64, t: f64) -> [f64; 3] {
    // a half turn's fraction, `x`, and the rates of `h (x - sin 2πx / 2π)` a radian of the turn
    let half = |x: f64| [h * (x - (2.0 * PI * x).sin() / (2.0 * PI)),
                         h * (1.0 - (2.0 * PI * x).cos()) / PI,
                         2.0 * h * (2.0 * PI * x).sin() / PI];
    if t <= 180.0 {
        half(t / 180.0)
    } else {
        let [s, ds, dds] = half((t - 180.0) / 180.0);
        [h - s, -ds, -dds]
    }
}

/// **`flat_cam.sv`**: a flat face cuts the cam its support function says.  With `p = base + s`,
/// the cam touches the face at `(p, -p')` in the follower's frame, turned back by the cam (1e-9),
/// and bends with radius `p + p''`, by differences of the core's own points; that is positive
/// all round, so the cam is convex, and the circle held osculating it three quarters through the
/// rise has that radius (1e-6), its centre on the face's normal there.
#[test]
fn the_flat_faced_cam_is_its_support_function() {
    let mut e = solved(include_str!("../../examples/generation/flat_cam.sv"));
    assert_eq!(dof(&mut e), 0);
    let (base, h) = (25.0, 15.0);
    let contact = |t: f64| {
        let [s, ds, _] = rise_and_return(h, t);
        turn([base + s, -ds], -t)
    };
    let radius = |t: f64| {
        let [s, _, dds] = rise_and_return(h, t);
        base + s + dds
    };
    let (rise, ret) = (curve(&e, "rise"), curve(&e, "return"));
    let mut worst = 0.0_f64;
    for k in 0..=360 {
        let t = k as f64;
        let c = if t <= 180.0 { rise } else { ret };
        worst = worst.max(apart(contact(t), e.sketch.curve_point(c, t)));
        // convex all round: the closed form's radius, and the core's bend by differences
        assert!(radius(t) > 0.0, "concave at {t}");
        if k % 15 == 0 && !matches!(k, 0 | 180 | 360) {
            let r = crate::common::radius_by_differences(|u| e.sketch.curve_point(c, u), t, 1e-3);
            assert!((r - radius(t)).abs() < 1e-4 * radius(t), "at {t}: {r} against {}", radius(t));
        }
    }
    assert!(worst < 1e-9, "the cam is off its support function by {worst}");
    // the circle osculating the rise at 135°, three quarters through it
    let want = radius(135.0);
    assert!((want - (25.0 + 15.0 * (0.75 + 1.0 / (2.0 * PI)) - 30.0 / PI)).abs() < 1e-12);
    let osc = curve(&e, "osc");
    let got = e.sketch.params[e.sketch.circles[osc].radius as usize].value;
    assert!((got - want).abs() < 1e-6, "the cam bends at {got}, its support function says {want}");
    let k = e.sketch.point_xy(e.map.ent_named("k").unwrap().i());
    let q = contact(135.0);
    let n = turn([1.0, 0.0], -135.0);
    assert!(apart([q[0] - want * n[0], q[1] - want * n[1]], k) < 1e-6,
            "the centre is off the normal");
}

/// **`sized_cam.sv`**: the base circle a flat face needs for the cam's tightest bend to be 15.
/// The cam bends with radius `base + s + s''`; on a cycloidal rise of `h` over 120° the least of
/// `s + s''` is where `cos 2πx = -1/8`, so the base is `15 - (s + s'')` there, against which the
/// solved unknown (1e-6).  Sampled at every tenth of a degree, the closed form bends no tighter
/// anywhere, and the core's own cam, by differences, bends at 15 there.
#[test]
fn the_sized_cam_solves_its_base_circle_through_its_law() {
    let mut e = solved(include_str!("../../examples/generation/sized_cam.sv"));
    assert_eq!(dof(&mut e), 0);
    let (h, least, beta) = (12.0, 15.0, 120f64.to_radians());
    // `s + s''` a fraction `x` through a rise, and the place the rise and return put it
    let bend = |x: f64| h * (x - (2.0 * PI * x).sin() / (2.0 * PI))
        + 2.0 * PI * h / (beta * beta) * (2.0 * PI * x).sin();
    let x = 1.0 - (-1.0_f64 / 8.0).acos() / (2.0 * PI);
    let want = least - bend(x);
    let base = e.sketch.params[e.sketch.free_vars["base"] as usize].value;
    assert!((base - want).abs() < 1e-6, "the base solved to {base}, the law says {want}");
    assert!((want - 21.3984).abs() < 1e-4);
    // nowhere tighter: the return is the rise a half turn on, taken away, and `bend` is 0 and `h`
    // at a rise's ends, so the dwells bend at `base` and `base + h`
    let radius = |t: f64| {
        let [up, down] = [60.0, 240.0].map(|a: f64| ((t - a) / 120.0).clamp(0.0, 1.0));
        base + bend(up) - bend(down)
    };
    let tau = 180.0 - (-1.0_f64 / 8.0).acos().to_degrees() / 3.0;
    assert!((tau - (60.0 + 120.0 * x)).abs() < 1e-12);
    let tightest = (0..3600).map(|k| radius(k as f64 / 10.0)).fold(f64::INFINITY, f64::min);
    assert!(tightest > least - 1e-6, "the cam bends at {tightest} somewhere");
    assert!((radius(tau) - least).abs() < 1e-9);
    let rise = curve(&e, "rise");
    let r = crate::common::radius_by_differences(|u| e.sketch.curve_point(rise, u), tau, 1e-3);
    assert!((r - least).abs() < 1e-4, "the cam bends at {r} where it should bend at {least}");
}

/// The point named `name` where the sketch `sk` stands, in the front view's `(u, v)`: the world's
/// `x` and `z`.
fn in_front(e: &Elaborated, sk: &gcs_core::model::Sketch, name: &str) -> [f64; 2] {
    let p = sk.world_point(e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i());
    assert!(p[1].abs() < 1e-12, "`{name}` left the front view: {p:?}");
    [p[0], p[2]]
}

/// **`mechanisms/cam_follower.sv`**: the cam cut from the follower's law, and the follower drawn
/// in its frame, followed round the turn.  At every station of the march the roller drawn in the
/// follower stands where the law lifts it, the cam's pin is turned by `t`, and turned back by the
/// cam the roller's centre is the pitch curve's point at that roll, a roller's radius from the
/// profile there and no nearer to any of it (1e-9): the roller rides on the cam cut for it.
#[test]
fn the_cams_roller_rides_on_the_profile_cut_for_it() {
    let mut e = solved(include_str!("../../examples/mechanisms/cam_follower.sv"));
    assert_eq!(dof(&mut e), 0);
    let (base, h, rr) = (25.0, 10.0, 5.0);
    let f = gcs_core::follow::follow(&e.sketch, "t").expect("followed");
    assert!(f.singular.is_none() && f.reached() == 360.0, "{:?}", f.singular);
    assert!(f.stations.len() > 12, "{} stations", f.stations.len());
    let (pc, cc) = (curve(&e, "pitch"), curve(&e, "profile"));
    let profile: Vec<(f64, f64)> =
        (0..=7200).map(|k| e.sketch.curve_point(cc, k as f64 / 20.0)).collect();
    let mut sk = e.sketch.clone();
    for s in &f.stations {
        sk.set_x(&s.x);
        let lift = h / 2.0 * (1.0 - s.at.to_radians().cos());
        let riding = in_front(&e, &sk, "riding");
        assert!(apart(riding, (base + lift, 0.0)) < 1e-9, "at {}: {riding:?}", s.at);
        assert!(apart(in_front(&e, &sk, "pin"), {
            let q = turn([0.0, 12.0], s.at);
            (q[0], q[1])
        }) < 1e-9, "the pin at {}", s.at);
        // the roller turned back into the cam's frame
        let seen = turn(riding, -s.at);
        assert!(apart(seen, e.sketch.curve_point(pc, s.at)) < 1e-9, "off the pitch curve at {}",
                s.at);
        let touch = apart(seen, e.sketch.curve_point(cc, s.at));
        assert!((touch - rr).abs() < 1e-9, "at {} the roller stands {touch} from its contact",
                s.at);
        let nearest = profile.iter().map(|&q| apart(seen, q)).fold(f64::INFINITY, f64::min);
        assert!(nearest > rr - 1e-9, "at {} the roller cuts the cam: {nearest}", s.at);
    }
}

/// **`generation/cam_plate.sv`**: the plate is the blank less the tappet swept under its law, and
/// its rising flank the tappet face's side generated under the drive.  The gauge, held on that
/// side from a view square to the page, stands 20 below the cam's centre and 3 off the page,
/// where `flat_cam.sv`'s support function puts the rising flank at that level (1e-9) — and there
/// the plate's material field changes sign across the flank's normal: the plate is cut to it.
#[test]
fn the_cam_plate_is_cut_to_its_flank() {
    let mut e = solved(include_str!("../../examples/generation/cam_plate.sv"));
    assert_eq!(dof(&mut e), 0);
    let (base, h) = (25.0, 15.0);
    // the flank's contact in the front view at the roll `t`
    let contact = |t: f64| {
        let [s, ds, _] = rise_and_return(h, t);
        turn([base + s, -ds], -t)
    };
    let g = e.sketch.world_point(e.map.ent_named("gauge").unwrap().i());
    assert!((g[1] - 3.0).abs() < 1e-9 && (g[2] + 20.0).abs() < 1e-9, "the gauge stands at {g:?}");
    // the rolls of the rise where the flank crosses the level, bisected on the closed form
    let level = |t: f64| contact(t)[1] + 20.0;
    let crossings: Vec<f64> = (0..180).filter(|&k| level(k as f64) * level(k as f64 + 1.0) <= 0.0)
        .map(|k| {
            let (mut a, mut b) = (k as f64, k as f64 + 1.0);
            for _ in 0..80 {
                let m = 0.5 * (a + b);
                if level(a) * level(m) <= 0.0 { b = m } else { a = m }
            }
            a
        }).collect();
    let tau = *crossings.iter().find(|&&t| (contact(t)[0] - g[0]).abs() < 1e-9)
        .unwrap_or_else(|| panic!("the gauge at {} is on no crossing of {crossings:?}", g[0]));
    // the plate's field either side of the gauge, along the flank's normal there (the follower's
    // line turned back by the cam)
    let i = e.map.ent_named("plate").unwrap().i();
    let field = gcs_core::solid::MaterialField::read(&e.sketch, i, 1e-10).unwrap();
    let n = turn([1.0, 0.0], -tau);
    let side = |d: f64| field.side([g[0] + d * n[0], g[1], g[2] + d * n[1]]);
    let (inside, outside) = (side(-1e-3), side(1e-3));
    assert!(inside < 0.0 && outside > 0.0, "the plate reads {inside} and {outside} at its flank");
}
