//! **General drives** (#391 stage 5): a jointed frame whose value is written over a declared
//! parameter, `fix(value == 5deg * (1 - cos(t))) follower`, is *driven* by it — its value is a
//! law, any expression of the parameter, and every reading of the frame's motion over the roll
//! (a sweep, a placement, a roll through a body) stands the joint at the law's value of the roll,
//! the roll being that parameter (one clock).  The law is differentiated by `taylor::Jet` and
//! enclosed by interval arithmetic, so `MotionBounds`, the speed bound and with them the roll
//! search and a swept field stay conservative under it.  The gate: a cam follower's lift against
//! its law, followed and swept; and (#398 G) the cam itself, a profile in the plane generated
//! under the drive, against its closed form, with a circle held osculating it; a point carried
//! under the drive held by a relation, at a stated roll, at one the solve finds, and its rate; and
//! an envelope in space generated under the drive held to, a ball's radius solving through it.

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

/// How fast `seen` runs a radian of the roll at `t` (degrees): a central difference of it.
fn seen_rate(t: f64) -> [f64; 3] {
    let h: f64 = 1e-5;
    let (a, b) = (seen(t + h.to_degrees()), seen(t - h.to_degrees()));
    std::array::from_fn(|i| (a[i] - b[i]) / (2.0 * h))
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
        let d = seen_rate(t);
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
    // a law is read over stated numbers, not an unknown the solve finds (#398 G)
    let s = said("param k: Scalar\nlift := frame\n\
        std.Slide(lift, along: stem, advance: 360mm)\nfix(value == 5deg * k * (1 - cos(t))) lift\n\
        X := point\nfix((0, 0, 22)) X\nX coincident r0 at(under: lift) 30deg\n");
    assert!(s.len() == 1 && s[0].starts_with("E080")
            && s[0].contains("reads `k`, which the drawing does not state"), "{s:?}");
    // one parameter drives a joint
    let s = said("param u: Angle over (0deg, 90deg)\nlift := frame\n\
        std.Slide(lift, along: stem, advance: 360mm)\nfix(value == t + u) lift\n");
    assert!(s.len() == 1 && s[0].starts_with("E103") && s[0].contains("`t` and `u`"), "{s:?}");
    // the roll is an angle: a drive over a number of another kind is followed, not swept
    let s = said("param s: Scalar over (0, 1)\nlift := frame\n\
        std.Slide(lift, along: stem, advance: 360mm)\nfix(value == 90deg * s) lift\n\
        q := sweep(ball over t, under: lift)\n");
    assert!(s.len() == 1 && s[0].starts_with("E080") && s[0].contains("a Scalar"), "{s:?}");
    // nor does a relation held to a meshing under the drive, which reads its mesh's turns at a
    // constant ratio (an envelope in space generated under one is held, #398 G)
    let s = said("wall := surface(ball, meridian)\n\
        contact := meshing(wall, wall over t in (0deg, 30deg), under: follower, seen_from: cam)\n\
        p := point hint((3, 0, 21))\np coincident contact\n");
    assert!(s.len() == 1 && s[0].starts_with("E061") && s[0].contains("`follower` is driven"),
            "{s:?}");
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


/// The roller of the planar cam (`profile`): its radius, and its centre where the drive starts.
const RR: f64 = 4.0;

/// The pitch point at the roll `t` (degrees) in the front view's `(u, v)`, its rate a degree,
/// and the cam's profile there: `RR` from the pitch point, square to its path, toward the cam's
/// centre — a radial roller cam's closed form.
fn pitch(t: f64) -> ([f64; 2], [f64; 2], [f64; 2]) {
    let (s, c) = t.to_radians().sin_cos();
    let rho = BASE + lift(t);
    let drho = RISE * t.to_radians().sin() * std::f64::consts::PI / 180.0;
    let k = std::f64::consts::PI / 180.0;
    let p = [rho * s, rho * c];
    let dp = [drho * s + rho * c * k, drho * c - rho * s * k];
    let n = dp[0].hypot(dp[1]);
    // the normal toward the centre
    let mut w = [dp[1] / n, -dp[0] / n];
    if w[0] * p[0] + w[1] * p[1] > 0.0 {
        w = [-w[0], -w[1]];
    }
    (p, dp, [p[0] + RR * w[0], p[1] + RR * w[1]])
}

/// **A profile in the plane generated under a drive** (#398 G): the cam the roller cuts as the
/// follower rises by its law, seen from the turning cam, against the closed form at every
/// degree — and its pitch curve, the roller's centre, against its own.
fn profile() -> Elaborated {
    solved(&cam(&format!("in std.front {{\n  roller := circle(center: r0)\n  \
        radius({RR}mm) roller\n}}\n\
        pitch := envelope(r0 over t, under: follower, seen_from: cam)\n\
        profile := envelope(roller over t, under: follower, seen_from: cam, side: near)\n")))
}

#[test]
fn a_cam_generated_under_a_drive_is_its_closed_form() {
    let e = profile();
    assert!(e.ok(), "{:?}", e.diags);
    let curve = |n: &str| e.map.ent_named(n).unwrap_or_else(|| panic!("no `{n}`")).i();
    let (pc, cc) = (curve("pitch"), curve("profile"));
    let (mut worst_p, mut worst_c) = (0.0_f64, 0.0_f64);
    for j in 0..=360 {
        let t = j as f64;
        let (p, _, c) = pitch(t);
        let (x, y) = e.sketch.curve_point(pc, t);
        worst_p = worst_p.max((x - p[0]).hypot(y - p[1]));
        let (x, y) = e.sketch.curve_point(cc, t);
        worst_c = worst_c.max((x - c[0]).hypot(y - c[1]));
    }
    assert!(worst_p < 1e-9, "the pitch curve is off by {worst_p}");
    assert!(worst_c < 1e-9, "the profile is off by {worst_c}");
}

#[test]
fn a_relation_holds_to_a_profile_generated_under_a_drive() {
    // a circle osculating the cam at 60° into the turn: its radius is the pitch curve's radius of
    // curvature less the roller's, the pitch curve the polar curve `ρ = BASE + lift`
    let e = solved(&cam(&format!("in std.front {{\n  roller := circle(center: r0)\n  \
        radius({RR}mm) roller\n}}\n\
        profile := envelope(roller over t, under: follower, seen_from: cam, side: near)\n\
        in std.front {{\n  k := point hint((0, 0))\n  osc := circle(center: k) hint(r: 20)\n  \
        profile curvature(t == 60) osc\n}}\n")));
    assert!(e.ok(), "{:?}", e.diags);
    let t = 60_f64.to_radians();
    let (r, dr, ddr) = (BASE + lift(60.0), RISE * t.sin(), RISE * t.cos());
    let want = (r * r + dr * dr).powf(1.5) / (r * r + 2.0 * dr * dr - r * ddr) - RR;
    let osc = e.map.ent_named("osc").unwrap().i();
    let got = e.sketch.params[e.sketch.circles[osc].radius as usize].value;
    assert!((got - want).abs() < 1e-6, "the cam bends at {got}, the closed form says {want}");
    // and its centre is the centre of curvature: on the profile's normal there
    let (_, _, c) = pitch(60.0);
    let k = e.sketch.point_xy(e.map.ent_named("k").unwrap().i());
    let d = (k.0 - c[0]).hypot(k.1 - c[1]);
    assert!((d - want).abs() < 1e-6, "the centre stands {d} from the profile");
    // the curvature contact's Jacobian, the law read through it, is the system's derivative
    crate::common::fd_jacobian(&e.sketch, 1e-5);
}

/// **A point carried under a drive, held by a relation** (#398 G): `X coincident r0 at(under:
/// follower) τ` holds `X` where the follower stands at its law's value of `τ`, the law carried in
/// the row's constants — against the closed form under the follower alone and seen from the cam,
/// at a roll stated and at one the solve finds.
#[test]
fn a_point_carried_under_a_drive_is_its_law() {
    let rolls = [0.0, 30.0, 90.0, 135.0, 200.0, 315.0];
    let mut extra = String::new();
    for (k, t) in rolls.iter().enumerate() {
        extra += &format!("X{k} := point\nX{k} coincident r0 at(under: follower) {t}deg\n\
            Y{k} := point\nY{k} coincident r0 at(under: follower, seen_from: cam) {t}deg\n");
    }
    let e = solved(&cam(&extra));
    assert!(e.ok(), "{:?}", e.diags);
    for (k, &t) in rolls.iter().enumerate() {
        let x = point(&e, &e.sketch, &format!("X{k}"));
        let want = [0.0, 0.0, BASE + lift(t)];
        assert!((0..3).all(|i| (x[i] - want[i]).abs() < 1e-9), "at {t}: {x:?} against {want:?}");
        let y = point(&e, &e.sketch, &format!("Y{k}"));
        let want = seen(t);
        assert!((0..3).all(|i| (y[i] - want[i]).abs() < 1e-9), "at {t}: {y:?} against {want:?}");
    }
    crate::common::fd_jacobian(&e.sketch, 1e-5);
}

/// **The instant a driven follower has risen 3 mm** (#398 G): the roll a relation owns, left to
/// the solve, reads the law's rate through its column — the roller's centre seen from the cam held
/// `BASE + 3` from the cam's centre is where `5 (1 − cos t) = 3`, `t = acos(0.4)`.
#[test]
fn the_roll_a_driven_point_is_carried_to_is_solved_through_its_law() {
    let want = (0.4_f64).acos().to_degrees();
    let s = seen(want);
    let e = solved(&cam(&format!("Z := point hint(({}, {}, {}))\n\
        Z coincident(under: follower, seen_from: cam) r0\n\
        Z distance({}mm) std.origin\n", s[0] + 0.5, s[1], s[2] - 0.5, BASE + 3.0)));
    assert!(e.ok(), "{:?}", e.diags);
    let c = e.sketch.constraints.iter()
        .find(|c| c.kind == gcs_core::constraints::CKind::CarriedPoint && !c.stated_roll())
        .expect("held");
    let got = e.sketch.params[c.args[3].param() as usize].value;
    assert!((got - want).abs() < 1e-8, "the roll is {got}°, the closed form says {want}°");
    let z = point(&e, &e.sketch, "Z");
    assert!((0..3).all(|i| (z[i] - s[i]).abs() < 1e-8), "{z:?} against {s:?}");
    crate::common::fd_jacobian(&e.sketch, 1e-5);
}

/// **A driven point's rate** (#398 G): `rate(X over t)` of a point carried under a drive is the
/// roller's velocity as the cam sees it, the law's rate by the chain rule — against a difference
/// of the closed form, held along a line through the point.
#[test]
fn a_driven_points_rate_is_its_laws() {
    let t = 50.0;
    let (s, d) = (seen(t), seen_rate(t));
    let e = solved(&cam(&format!("W := point\n\
        W coincident(under: follower, seen_from: cam, t == {t}) r0\n\
        a := point\nfix(({}, {}, {})) a\nb := point hint(({}, {}, {}))\nv := line(a, b)\n\
        rate(W over t) parallel v\nlength(10mm) v\n\
        b level(std.y) a\n",
        s[0], s[1], s[2], s[0] + d[0], s[1] + d[1], s[2] + d[2])));
    assert!(e.ok(), "{:?}", e.diags);
    let [a, b] = ["a", "b"].map(|n| point(&e, &e.sketch, n));
    let n = d.iter().map(|x| x * x).sum::<f64>().sqrt();
    let got: Vec<f64> = (0..3).map(|i| (b[i] - a[i]) / 10.0).collect();
    assert!((0..3).all(|i| (got[i] - d[i] / n).abs() < 1e-7), "{got:?} against {d:?}");
}

/// A ball whose radius the solve finds — its meridian `sleeve` about `r0`, its ends on the stem —
/// and `tube`, the envelope in space of its side swept under the follower as the cam sees it: the
/// canal surface about the path `seen`, at the ball's radius.  `extra` adds statements.
fn bulb(extra: &str) -> String {
    cam(&format!("in std.front {{\n  lo := point hint((0, 17))\n  hi := point hint((0, 23))\n  \
        lo coincident stem\n  hi coincident stem\n  \
        sleeve := arc(center: r0, start: lo, end: hi)\n  axle := line(lo, hi)\n}}\n\
        bulb := solid(face(sleeve, axle), about: axle)\n\
        skin := surface(bulb, sleeve)\n\
        tube := envelope(skin over t, under: follower, seen_from: cam)\n{extra}"))
}

/// Where `bulb`'s tests stand their point: off the path at 60° by 2 mm outward in the cam's plane
/// and 2 mm square to it, so the path's nearest point is `seen(60)`, `2√2` away.
fn off_the_path() -> [f64; 3] {
    let (s, d) = (seen(60.0), seen_rate(60.0));
    let n = d[0].hypot(d[2]);
    let out = [-d[2] / n, 0.0, d[0] / n];
    let out = if out[0] * s[0] + out[2] * s[2] > 0.0 { out } else { out.map(|x| -x) };
    [s[0] + 2.0 * out[0], 2.0, s[2] + 2.0 * out[2]]
}

/// **A relation held to an envelope in space generated under a drive** (#398 G): the ball's
/// radius solves so the tube its side sweeps under the follower, as the cam sees it, passes
/// through a point stated off the path — the law carried in the contact's constants and read in
/// Taylor arithmetic, so the rows and their Jacobian are exact through it.  The closed form: the
/// radius is the point's distance from the path `seen`, met at the roll whose point is nearest.
#[test]
fn a_relation_holds_to_an_envelope_generated_under_a_drive() {
    let p = off_the_path();
    let e = solved(&bulb(&format!("P := point\nfix(({}, {}, {})) P\nP coincident tube\n",
                                  p[0], p[1], p[2])));
    assert!(e.ok(), "{:?}", e.diags);
    let [r0, lo] = ["r0", "lo"].map(|n| point(&e, &e.sketch, n));
    let got = (0..3).map(|i| (lo[i] - r0[i]).powi(2)).sum::<f64>().sqrt();
    let want = 8_f64.sqrt();
    assert!((got - want).abs() < 1e-9, "the ball's radius is {got}, the closed form says {want}");
    let c = e.sketch.constraints.iter()
        .find(|c| c.kind == gcs_core::constraints::CKind::PointOnEnvelope).expect("held");
    // the place's roll, in radians: where the path's point is nearest
    let roll = e.sketch.params[c.args[4].param() as usize].value.to_degrees();
    assert!((roll - 60.0).abs() < 1e-7, "met at {roll}°, the closed form says 60°");
    crate::common::fd_jacobian(&e.sketch, 1e-5);
}

/// **The normal of an envelope generated under a drive** (#398 G): a line held along the tube's
/// normal where a point is held to it runs from the path's nearest point through the point.
#[test]
fn an_envelopes_normal_under_a_drive_is_its_closed_forms() {
    let p = off_the_path();
    let s = seen(60.0);
    let e = solved(&bulb(&format!("P := point\nfix(({}, {}, {})) P\nP coincident tube\n\
        Q := point hint(({}, {}, {}))\nnl := line(P, Q)\nlength(5mm) nl\n\
        tube perpendicular(at: P) nl\n",
        p[0], p[1], p[2], p[0] + (p[0] - s[0]) * 1.5, p[1] + (p[1] - s[1]) * 1.5,
        p[2] + (p[2] - s[2]) * 1.5)));
    assert!(e.ok(), "{:?}", e.diags);
    let q = point(&e, &e.sketch, "Q");
    let r = 8_f64.sqrt();
    let want: Vec<f64> = (0..3).map(|i| p[i] + (p[i] - s[i]) * 5.0 / r).collect();
    assert!((0..3).all(|i| (q[i] - want[i]).abs() < 1e-8), "{q:?} against {want:?}");
    crate::common::fd_jacobian(&e.sketch, 1e-5);
}
