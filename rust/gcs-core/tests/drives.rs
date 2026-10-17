//! **General drives** (#391 stage 5): a jointed frame whose value is written over a declared
//! parameter, `fix(value == 5deg * (1 - cos(t))) follower`, is *driven* by it — its value is a
//! law, any expression of the parameter, and every reading of the frame's motion over the roll
//! (a sweep, a placement, a roll through a body) stands the joint at the law's value of the roll,
//! the roll being that parameter (one clock).  The law is differentiated by `taylor::Jet` and
//! enclosed by interval arithmetic, so `MotionBounds`, the speed bound and with them the roll
//! search and a swept field stay conservative under it.  The gate: a cam follower's lift against
//! its law, followed and swept.

use gcs_core::interval::minimum::{Options, Status};
use gcs_core::interval::Interval as I;
use gcs_core::model::{EntKind, SolidDef};
use gcs_core::motion::{Family, Law};
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;

use crate::common::text::solved;

/// The base circle the roller's centre rides on, and the follower's rise.
const BASE: f64 = 20.0;
const RISE: f64 = 5.0;

/// The law the follower is driven by: its lift (mm) at the cam's turn `t` (degrees).
fn lift(t: f64) -> f64 {
    RISE * (1.0 - t.to_radians().cos())
}

/// A radial cam: the cam turns about the origin in the front view with `t`, and the follower
/// slides on a stem along `y` (360 mm a turn of its value, so its value in degrees is its lift in
/// millimetres), driven by the law `lift`.  `ball` is a unit ball centred where the roller is,
/// `R` that centre drawn in the follower, and `cut` the ball swept over `t` as the cam sees it:
/// what the cam must not hold.  `extra` adds statements.
fn cam(extra: &str) -> String {
    format!("unit mm\nuse std\nparam t: Angle over (0deg, 360deg)\n\
        in std.front {{\n\
        \x20 construction stem := line(std.origin, hint((0, 1)))\n\
        \x20 fix((0, 1)) stem.p2\n\
        \x20 r0 := point\n\
        \x20 fix((0, {BASE})) r0\n\
        \x20 bottom := point\n\
        \x20 top := point\n\
        \x20 fix((0, {b})) bottom\n\
        \x20 fix((0, {t})) top\n\
        \x20 diameter := line(bottom, top)\n\
        \x20 meridian := arc(center: r0, start: bottom, end: top)\n\
        \x20 radius(1mm) meridian\n\
        }}\n\
        ball := solid(face(meridian, diameter), about: diameter)\n\
        cam := frame\n\
        std.Spin(cam, about: std.origin, on: std.front, ratio: 1)\n\
        fix(value == t) cam\n\
        follower := frame\n\
        std.Slide(follower, along: stem, advance: 360mm)\n\
        fix(value == {RISE}deg * (1 - cos(t))) follower\n\
        R := r0 in follower\n\
        cut := sweep(ball over t, under: follower, seen_from: cam)\n{extra}",
        b = BASE - 1.0, t = BASE + 1.0)
}

/// The roller's centre at the roll `t` (degrees), seen from the cam, in the world: the
/// follower's lift up the stem (the front view's `v`, the world's `z`), turned back by the cam's
/// turn (the view's `u` toward its `v` is the cam's sense).
fn seen(t: f64) -> [f64; 3] {
    let (s, c) = t.to_radians().sin_cos();
    let v = BASE + lift(t);
    [v * s, 0.0, v * c]
}

fn point(e: &Elaborated, sk: &gcs_core::model::Sketch, n: &str) -> [f64; 3] {
    let p = e.map.ent_named(n).unwrap_or_else(|| panic!("no `{n}`"));
    assert_eq!(p.kind, EntKind::Point);
    sk.world_point(p.i())
}

/// The swept solid's motion: the follower's drive seen from the cam.
fn swept(e: &Elaborated) -> Family {
    let i = e.map.ent_named("cut").unwrap().i();
    let SolidDef::Swept { motion, .. } = e.sketch.solids[i].def else { panic!("not swept") };
    Family::read(&e.sketch, motion as usize).expect("the drive reads")
}

#[test]
fn a_followers_lift_is_its_law_at_every_instant() {
    let e = solved(&cam(""));
    let at_rest = point(&e, &e.sketch, "R");
    assert!(at_rest.iter().zip([0.0, 0.0, BASE]).all(|(a, b)| (a - b).abs() < 1e-9), "{at_rest:?}");
    let f = gcs_core::follow::follow(&e.sketch, "t").expect("followed");
    assert!(f.singular.is_none() && f.reached() == 360.0, "{:?}", f.singular);
    let mut sk = e.sketch.clone();
    for s in &f.stations {
        sk.set_x(&s.x);
        let r = point(&e, &sk, "R");
        let want = BASE + lift(s.at);
        assert!(r[0].abs() < 1e-9 && (r[2] - want).abs() < 1e-9, "at {}: {r:?} against {want}",
                s.at);
    }
}

#[test]
fn a_drive_seen_from_its_cam_is_its_law_turned_back() {
    let e = solved(&cam(""));
    let family = swept(&e);
    let r0 = [0.0, 0.0, BASE];
    for k in 0..=72 {
        let t = 5.0 * k as f64;
        let m = family.at(t.to_radians()).unwrap();
        let p = m.point(r0);
        let want = seen(t);
        assert!((0..3).all(|i| (p[i] - want[i]).abs() < 1e-9), "at {t}: {p:?} against {want:?}");
        // the rate is the law's, by the chain rule through its jet, against a difference of the
        // closed form
        let v = m.velocity(r0);
        let h: f64 = 1e-5;
        let d: Vec<f64> = (0..3).map(|i| (seen(t + h.to_degrees())[i]
            - seen(t - h.to_degrees())[i]) / (2.0 * h)).collect();
        assert!((0..3).all(|i| (v[i] - d[i]).abs() < 1e-6), "at {t}: {v:?} against {d:?}");
        // the pose alone agrees
        let q = family.pose_at(t.to_radians()).unwrap().point(r0);
        assert!((0..3).all(|i| (q[i] - want[i]).abs() < 1e-9));
    }
    // a drive is no constant twist
    assert!(family.screw().is_none() && family.turning().is_none() && family.sliding().is_none());
    assert!(family.instantaneous().is_err());
}

#[test]
fn a_drives_bounds_enclose_its_poses_and_its_speed() {
    let e = solved(&cam(""));
    let family = swept(&e);
    let r0 = [0.0, 0.0, BASE];
    let boxed = r0.map(|v| I::point(v).unwrap());
    for k in 0..24 {
        let [a, b] = [k, k + 1].map(|j| (15.0 * j as f64).to_radians());
        let span = I::new(a, b).unwrap();
        let bounds = family.bounds(span).unwrap();
        let enclosed = bounds.point(boxed).unwrap();
        let speed = family.inverse_point_speed_bound_over(boxed, span).unwrap();
        for j in 0..=20 {
            let theta = a + (b - a) * j as f64 / 20.0;
            let p = family.at(theta).unwrap().point(r0);
            assert!((0..3).all(|i| enclosed[i].contains(p[i])), "{p:?} outside {enclosed:?}");
            // the inverse pose carries the cam's point back to the follower's frame at the speed
            // the bound holds
            let inverse = |th: f64| family.at(th).unwrap().inverse().point(r0);
            let h = 1e-6;
            let (p1, p0) = (inverse(theta + h), inverse(theta - h));
            let v = ((0..3).map(|i| (p1[i] - p0[i]).powi(2)).sum::<f64>()).sqrt() / (2.0 * h);
            assert!(v <= speed * (1.0 + 1e-6), "speed {v} past the bound {speed}");
        }
    }
}

/// The bounds a swept field's distance from solid `name`'s material at `p` is found within.
fn material(e: &Elaborated, name: &str, p: [f64; 3]) -> [f64; 2] {
    let i = e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i();
    let mut query = MaterialField::read(&e.sketch, i, 1e-10).unwrap().evaluator(256);
    let answer = query.bounds(p.map(|v| I::point(v).unwrap()),
        Options { value_tolerance: 1e-6, max_evaluations: 200_000 }).unwrap();
    assert!(answer.sweeps.iter().all(|s| s.minimum.status == Status::Converged));
    answer.value.bounds()
}

#[test]
fn a_cam_cut_is_the_ball_swept_along_its_law() {
    let e = solved(&cam(""));
    // the distance from a point to the ball's path: the nearest of a fine grid of the closed
    // form, refined by halving steps either side
    let distance = |p: [f64; 3]| {
        let d = |t: f64| {
            let c = seen(t);
            (0..3).map(|k| (p[k] - c[k]).powi(2)).sum::<f64>().sqrt()
        };
        let mut t = (0..36000).map(|k| k as f64 / 100.0).min_by(|a, b| d(*a).total_cmp(&d(*b)))
            .unwrap();
        let mut h = 0.01;
        while h > 1e-9 {
            for s in [t - h, t + h] {
                if d(s) < d(t) { t = s }
            }
            h *= 0.5;
        }
        d(t) - 1.0
    };
    // in the front view's (u, v), off it by `y`: on the path, inside and outside its envelope
    for [u, v, y] in [[0.0, 20.0, 0.0], [7.0, 22.5, 0.0], [-24.0, -9.0, 0.3], [0.0, -30.0, 0.5],
                      [12.0, 12.0, -0.4], [0.0, 17.5, 0.0], [3.0, -31.5, 0.0]] {
        let p = [u, y, v];
        let [lo, hi] = material(&e, "cut", p);
        let want = distance(p);
        assert!(lo - 1e-6 <= want && want <= hi + 1e-6, "at {p:?}: [{lo}, {hi}] against {want}");
    }
}

#[test]
fn a_tool_placed_under_a_drive_stands_where_its_law_puts_it() {
    // read at a roll, the joint stands at the law's value of it; drawn in the frame, at its value
    // where the drawing stands, `t` at its start
    let e = solved(&cam("placed := ball at(under: follower) 90deg\ndrawn := ball in follower\n"));
    let up = BASE + lift(90.0);
    assert!(material(&e, "placed", [0.0, 0.0, up])[1] < -0.999);
    assert!(material(&e, "placed", [0.0, 0.0, BASE])[0] > 0.5);
    assert!(material(&e, "drawn", [0.0, 0.0, BASE])[1] < -0.999);
}

#[test]
fn a_roll_through_a_body_under_a_drive_is_conservative() {
    // a blank ball of radius 3 where the roller starts: the roll runs from where the roller, on
    // its path seen from the cam, first parts from it to where it last does — never short of
    // either, and a step or so past
    let blank = format!("in std.front {{\n  k0 := point\n  fix((0, {BASE})) k0\n  \
        k1 := point\n  k2 := point\n  fix((0, {a})) k1\n  fix((0, {b})) k2\n  \
        kd := line(k1, k2)\n  km := arc(center: k0, start: k1, end: k2)\n  radius(3mm) km\n}}\n\
        blank := solid(face(km, kd), about: kd)\n\
        rolled := sweep(ball, through: blank, under: follower, seen_from: cam)\n",
        a = BASE - 3.0, b = BASE + 3.0);
    let e = solved(&cam(&blank));
    let i = e.map.ent_named("rolled").unwrap().i();
    let [lo, hi] = gcs_core::solid::roll::interval(&e.sketch, i).expect("a roll is found");
    // where the roller's centre is 4 from the blank's: bisected on the closed form
    let gap = |t: f64| { let c = seen(t); (c[0].powi(2) + (c[2] - BASE).powi(2)).sqrt() - 4.0 };
    let (mut a, mut b) = (0.0, 90.0);
    for _ in 0..80 {
        let m = 0.5 * (a + b);
        if gap(m) < 0.0 { a = m } else { b = m }
    }
    let parts = b.to_radians();
    let step = gcs_core::solid::roll::STEP;
    assert!(hi >= parts && hi <= parts + 3.0 * step, "{hi} against {parts}");
    assert!(lo <= -parts && lo >= -parts - 3.0 * step, "{lo} against {}", -parts);
}

#[test]
fn a_drive_refuses_what_it_cannot_mean() {
    let said = |extra: &str| crate::common::lenient::read(&cam(extra)).1;
    // a relation the solve holds reads no law
    let s = said("X := point\nX coincident r0 at(under: follower) 30deg\n");
    assert!(s.len() == 1 && s[0].starts_with("E080") && s[0].contains("driven by `t`"), "{s:?}");
    // one parameter drives a joint
    let s = said("param u: Angle over (0deg, 90deg)\nlift := frame\n\
        std.Slide(lift, along: stem, advance: 360mm)\nfix(value == t + u) lift\n");
    assert!(s.len() == 1 && s[0].starts_with("E103") && s[0].contains("`t` and `u`"), "{s:?}");
    // the roll is an angle: a drive over a number of another kind is followed, not swept
    let s = said("param s: Scalar over (0, 1)\nlift := frame\n\
        std.Slide(lift, along: stem, advance: 360mm)\nfix(value == 90deg * s) lift\n\
        q := sweep(ball over t, under: lift)\n");
    assert!(s.len() == 1 && s[0].starts_with("E080") && s[0].contains("a Scalar"), "{s:?}");
    // nor does a relation held to an envelope the drive generates
    let s = said("wall := surface(ball, meridian)\n\
        g := envelope(wall over t in (0deg, 30deg), under: follower, seen_from: cam)\n\
        p := point hint((3, 0, 21))\np coincident g\n");
    assert!(s.len() == 1 && s[0].starts_with("E061") && s[0].contains("`follower` is driven"),
            "{s:?}");
    // a profile in the sheet is generated under joints at a constant rate
    let s = said("in std.front {\n  k := circle(center: r0)\n  radius(2mm) k\n}\n\
        p := envelope(k over t in (0deg, 90deg), under: follower, seen_from: cam)\n");
    assert!(s.iter().any(|m| m.contains("driven by `t`")), "{s:?}");
}

/// **A law three ways** (`motion::Law`): its rate from its Taylor jet against a difference of its
/// value, and its interval enclosure over every stretch holding both at every roll in it — each
/// function the interval walker carries — and failing closed for one it does not.
#[test]
fn a_laws_jet_and_enclosure_agree_with_its_value() {
    let e = solved("unit mm\nparam t: Angle over (-90deg, 400deg)\nuse std\nin std.front {\n  \
        o := point\n  fix((0, 0)) o\n}\n");
    let laws = ["5deg * (1 - cos(t))", "t + 20deg * sin(2 * t) - 3deg", "(t / 90deg) ^ 3 * 10deg",
                "3 * sqrt(t * t + 1deg * 1deg)", "abs(t - 100deg) / 2",
                "max(t, 30deg) - min(t, 0deg)", "hypot(t, 45deg)", "t / (2 + cos(t))",
                "(t / 10deg) ^ -2 * 1deg + t"];
    // where `abs`, `max` and `min` turn a corner, and their rate is one-sided
    let kinks = [0.0, 30.0, 100.0].map(f64::to_radians);
    for text in laws {
        let law = Law::read(&e.sketch, text, "t").expect(text);
        for k in 0..49 {
            let [a, b] = [k, k + 1].map(|j| (-90.0 + 10.0 * j as f64).to_radians());
            // a stretch the walker refuses (a power of one reaching zero) is said, not guessed
            let Ok([v, r]) = law.enclose(I::new(a, b).unwrap()) else {
                assert!(text.contains("^ -2") && a <= 0.0 && b >= 0.0, "{text} over [{a}, {b}]");
                continue;
            };
            for j in 0..=10 {
                let theta = a + (b - a) * j as f64 / 10.0;
                let [x, dx] = law.jet(theta).unwrap();
                assert!((x - law.value(theta).unwrap()).abs() < 1e-12);
                assert!(v.contains(x), "{text} at {theta}: {x} outside {v:?}");
                let h = 1e-6;
                let fd = (law.value(theta + h).unwrap() - law.value(theta - h).unwrap())
                    / (2.0 * h);
                let kinked = kinks.iter().any(|k| (theta - k).abs() < 2.0 * h);
                assert!(kinked || (dx - fd).abs() < 1e-6 * (1.0 + dx.abs()), "{text}: {dx} {fd}");
                assert!(r.contains(dx), "{text} at {theta}: rate {dx} outside {r:?}");
            }
        }
    }
    // no enclosure is carried of a tangent: the bound is refused, never guessed
    let law = Law::read(&e.sketch, "tan(t / 8)", "t").unwrap();
    assert!(law.enclose(I::new(0.0, 0.1).unwrap()).is_err());
    // a drive over the roll runs over an angle
    let scalar = solved("unit mm\nparam s: Scalar over (0, 1)\nuse std\nin std.front {\n  \
        o := point\n  fix((0, 0)) o\n}\n");
    assert!(Law::read(&scalar.sketch, "90deg * s", "s").unwrap_err().contains("an angle"));
}

