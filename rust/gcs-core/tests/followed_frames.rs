//! **A motion read off a followed drawing** (#398 F): a frame with no joint of its own, placed on
//! geometry the drawing moves as a declared parameter runs — a four-bar's coupler, `std.Turned(b,
//! c)` — is read over that parameter as a motion (`MotionDef::Followed`, `motion::Track`): the
//! frame where the march stands it, fitted between stations.  So a sweep, a placement and an
//! envelope read it as they read a joint.  The gate: a ball on the coupler swept along the coupler
//! curve against the closed form, the envelope of a circle the coupler carries against its own,
//! and a spatial linkage's trace against its closed form.

use gcs_core::interval::minimum::{Options, Status};
use gcs_core::interval::Interval as I;
use gcs_core::model::{EntKind, SolidDef};
use gcs_core::motion::Family;
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;

use crate::common::text::solved;

/// The four-bar of `mechanisms/four_bar.sv`: ground `a`–`d`, crank `a`–`b` driven by `t`,
/// coupler `b`–`c`, rocker `d`–`c`.
const G: f64 = 40.0;
const R: f64 = 10.0;
const L: f64 = 35.0;
const K: f64 = 25.0;

/// The four-bar with a frame on its coupler, `coupler.axes` (its origin at `b`, `u` toward `c`),
/// and a unit ball centred on the coupler's midpoint `m`; `extra` adds statements.
fn four_bar(extra: &str) -> String {
    format!("unit mm\nuse std\nparam t: Angle over (0deg, 360deg)\n\
        in std.front {{\n\
        \x20 a := point\n  d := point\n  b := point hint(({R}, 0))\n  c := point hint(({G}, {K}))\n\
        \x20 fix((0, 0)) a\n  fix(({G}, 0)) d\n\
        \x20 ground := line(a, d)\n  crank := line(a, b)\n  coupler_bar := line(b, c)\n\
        \x20 rocker := line(d, c)\n\
        \x20 length({R}) crank\n  length({L}) coupler_bar\n  length({K}) rocker\n\
        \x20 ground angle(t) crank\n\
        \x20 m := point\n  m midpoint coupler_bar\n\
        \x20 bottom := point hint((22, 11))\n  top := point hint((22, 13))\n\
        \x20 diameter := line(bottom, top)\n  vertical diameter\n\
        \x20 m midpoint diameter\n\
        \x20 meridian := arc(center: m, start: bottom, end: top)\n  radius(1mm) meridian\n\
        }}\n\
        coupler := std.Turned(b, c) in std.front\n\
        ball := solid(face(meridian, diameter), about: diameter)\n{extra}")
}

/// The crank pin and the rocker pin at the crank's angle `t` (degrees), the coupler above the
/// ground (the branch the seed of `c` picks).
fn pins(t: f64) -> ([f64; 2], [f64; 2]) {
    let (b, c) = (|| {
        let (s, co) = t.to_radians().sin_cos();
        let b = (R * co, R * s);
        let c = crate::continuation::closed_c(G, R, L, K, t, 1.0)?;
        Some((b, c))
    })().expect("a crank-rocker closes everywhere");
    ([b.0, b.1], [c.0, c.1])
}

/// The coupler's midpoint at `t`, in the world (the front view's `u` is `x`, its `v` is `z`).
fn midpoint(t: f64) -> [f64; 3] {
    let (b, c) = pins(t);
    [(b[0] + c[0]) / 2.0, 0.0, (b[1] + c[1]) / 2.0]
}

/// The motion the solid `name` is swept under.
fn swept(e: &Elaborated, name: &str) -> Family {
    let i = e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i();
    let SolidDef::Swept { motion, .. } = e.sketch.solids[i].def else { panic!("not swept") };
    Family::read(&e.sketch, motion as usize).expect("the coupler's motion reads")
}

fn point(e: &Elaborated, n: &str) -> [f64; 3] {
    let p = e.map.ent_named(n).unwrap_or_else(|| panic!("no `{n}`"));
    assert_eq!(p.kind, EntKind::Point);
    e.sketch.world_point(p.i())
}

#[test]
fn a_frame_on_a_coupler_is_carried_where_the_coupler_goes() {
    let e = solved(&four_bar("groove := sweep(ball over t, under: coupler.axes)\n"));
    let family = swept(&e, "groove");
    // the midpoint where it is drawn, at `t = 0`, carried to where the coupler puts it
    let m0 = point(&e, "m");
    let want0 = midpoint(0.0);
    assert!((0..3).all(|k| (m0[k] - want0[k]).abs() < 1e-9), "{m0:?} against {want0:?}");
    let mut worst = 0.0_f64;
    let mut worst_rate = 0.0_f64;
    for j in 0..=720 {
        let t = 0.5 * j as f64;
        let m = family.at(t.to_radians()).unwrap();
        let p = m.point(m0);
        let want = midpoint(t);
        worst = worst.max((0..3).map(|k| (p[k] - want[k]).abs()).fold(0.0, f64::max));
        // its rate a radian of the crank, against a difference of the closed form
        let h = 1e-4_f64;
        let (fwd, back) = (midpoint(t + h.to_degrees()), midpoint(t - h.to_degrees()));
        let v = m.velocity(m0);
        worst_rate = worst_rate.max((0..3).map(|k| (v[k] - (fwd[k] - back[k]) / (2.0 * h)).abs())
            .fold(0.0, f64::max));
        // the pose alone agrees with the pose and its rate
        let q = family.pose_at(t.to_radians()).unwrap().point(m0);
        assert!((0..3).all(|k| (q[k] - p[k]).abs() < 1e-12), "at {t}: {q:?} against {p:?}");
        // and the coupler's direction turns as the coupler does
        let (b, c) = pins(t);
        let (b0, c0) = pins(0.0);
        let dir = |b: [f64; 2], c: [f64; 2]| {
            let (x, z) = (c[0] - b[0], c[1] - b[1]);
            let n = x.hypot(z);
            [x / n, 0.0, z / n]
        };
        let turned = m.vector(dir(b0, c0));
        let want = dir(b, c);
        assert!((0..3).all(|k| (turned[k] - want[k]).abs() < 1e-8), "at {t}: {turned:?}");
    }
    assert!(worst < 1e-8, "{worst}");
    assert!(worst_rate < 1e-5, "{worst_rate}");
    // a frame the drawing moves is no constant twist
    assert!(family.screw().is_none() && family.turning().is_none());
    assert!(family.instantaneous().is_err());
}

#[test]
fn a_coupler_frames_bounds_enclose_its_poses_and_its_speed() {
    let e = solved(&four_bar("groove := sweep(ball over t, under: coupler.axes)\n"));
    let family = swept(&e, "groove");
    let m0 = point(&e, "m");
    let boxed = m0.map(|v| I::point(v).unwrap());
    for k in 0..24 {
        let [a, b] = [k, k + 1].map(|j| (15.0 * j as f64).to_radians());
        let span = I::new(a, b).unwrap();
        let bounds = family.bounds(span).unwrap();
        let enclosed = bounds.point(boxed).unwrap();
        let speed = family.inverse_point_speed_bound_over(boxed, span).unwrap();
        for j in 0..=40 {
            let theta = a + (b - a) * j as f64 / 40.0;
            let p = family.at(theta).unwrap().point(m0);
            assert!((0..3).all(|i| enclosed[i].contains(p[i])), "{p:?} outside {enclosed:?}");
            let inverse = |th: f64| family.at(th).unwrap().inverse().point(m0);
            let h = 1e-6;
            let (p1, p0) = (inverse((theta + h).min(b)), inverse((theta - h).max(a)));
            let dt = (theta + h).min(b) - (theta - h).max(a);
            let v = ((0..3).map(|i| (p1[i] - p0[i]).powi(2)).sum::<f64>()).sqrt() / dt;
            assert!(v <= speed * (1.0 + 1e-6), "speed {v} past the bound {speed}");
        }
    }
    // past what the march reached, nothing is claimed
    assert!(family.at(361f64.to_radians()).is_err());
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

/// How far `p` stands from the coupler curve `midpoint` runs on: the nearest of a fine grid of
/// the closed form, refined by halving steps either side.
fn from_coupler_curve(p: [f64; 3]) -> f64 {
    let d = |t: f64| {
        let c = midpoint(t);
        (0..3).map(|k| (p[k] - c[k]).powi(2)).sum::<f64>().sqrt()
    };
    let mut t = (0..36000).map(|k| k as f64 / 100.0).min_by(|a, b| d(*a).total_cmp(&d(*b)))
        .unwrap();
    let mut h = 0.01;
    while h > 1e-9 {
        for s in [t - h, t + h] {
            if d(s) < d(t) {
                t = s;
            }
        }
        h *= 0.5;
    }
    d(t)
}

#[test]
fn a_ball_on_a_coupler_sweeps_the_tube_about_its_coupler_curve() {
    // a plate in the front view, the groove the ball sweeps cut from it
    let blank = "in std.front {\n  p1 := point\n  p2 := point\n  p3 := point\n  p4 := point\n  \
        fix((-20, -20)) p1\n  fix((60, -20)) p2\n  fix((60, 50)) p3\n  fix((-20, 50)) p4\n  \
        rim := line(p1, p2) -> line(p2, p3) -> line(p3, p4) -> line(p4, p1) -> close\n}\n\
        plate := solid(face(rim), from: -3mm, to: 3mm)\n\
        groove := sweep(ball over t, under: coupler.axes)\n\
        groove cut plate\n";
    let e = solved(&four_bar(blank));
    // in the front view's (u, v), off it by `y`: on the curve, beside it, inside and outside it
    let near: Vec<[f64; 3]> = [0.0, 100.0, 200.0, 300.0].iter().flat_map(|&t| {
        let c = midpoint(t);
        [[c[0], 0.0, c[2]], [c[0] + 0.6, 0.3, c[2] - 0.2], [c[0] - 1.5, 0.0, c[2] + 0.4]]
    }).collect();
    let far = [[20.0, 0.0, 0.0], [5.0, 0.4, 30.0], [45.0, -0.5, 40.0]];
    for &p in near.iter().chain(&far) {
        let [lo, hi] = material(&e, "groove", p);
        let want = from_coupler_curve(p) - 1.0;
        assert!(lo - 1e-6 <= want && want <= hi + 1e-6, "at {p:?}: [{lo}, {hi}] against {want}");
        // the plate keeps what the groove does not reach, and loses what it does
        let [lo, hi] = material(&e, "plate", p);
        if want < -1e-3 {
            assert!(lo > 0.0, "at {p:?} the groove cuts: [{lo}, {hi}]");
        } else if want > 1e-3 {
            assert!(hi < 0.0, "at {p:?} the plate stands: [{lo}, {hi}]");
        }
    }
}


/// The coupler's instant centre at `t`: where the crank's line meets the rocker's.
fn instant_centre(t: f64) -> [f64; 2] {
    let (b, c) = pins(t);
    // a + s b = d + r (c − d)
    let (ux, uy) = (b[0], b[1]);
    let (vx, vy) = (c[0] - G, c[1]);
    let den = ux * vy - uy * vx;
    let s = (G * vy) / den;
    [s * ux, s * uy]
}

#[test]
fn the_envelope_of_a_circle_a_coupler_carries_is_its_closed_form() {
    let r = 3.0;
    let e = solved(&four_bar(&format!("in std.front {{\n  roller := circle(center: m)\n  \
        radius({r}mm) roller\n}}\ncut := envelope(roller over t, under: coupler.axes)\n")));
    assert!(e.ok(), "{:?}", e.diags);
    let i = e.map.ent_named("cut").expect("the envelope is a curve").i();
    let (mut worst, mut worst_side) = (0.0_f64, 0.0_f64);
    for j in 0..=720 {
        let t = 0.5 * j as f64;
        let (x, z) = e.sketch.curve_point(i, t);
        // the roller cuts where it touches the curve its centre runs on: `r` from the centre,
        // square to the centre's velocity
        let m = midpoint(t);
        let h = 1e-4_f64;
        let (fwd, back) = (midpoint(t + h.to_degrees()), midpoint(t - h.to_degrees()));
        let v = [fwd[0] - back[0], fwd[2] - back[2]];
        let (m, off) = ([m[0], m[2]], [x - m[0], z - m[2]]);
        let along = (off[0] * v[0] + off[1] * v[1]) / v[0].hypot(v[1]);
        worst = worst.max((off[0].hypot(off[1]) - r).abs()).max(along.abs());
        // and on the near side of the instant centre, wherever that is a place
        let ic = instant_centre(t);
        let toward = [ic[0] - m[0], ic[1] - m[1]];
        let n = toward[0].hypot(toward[1]);
        if n < 1e3 {
            let want = [m[0] + r * toward[0] / n, m[1] + r * toward[1] / n];
            worst_side = worst_side.max((x - want[0]).hypot(z - want[1]));
        }
    }
    assert!(worst_side < 1e-6, "{worst_side}");
    assert!(worst < 1e-6, "{worst}");
    // drawn as a polyline round the whole turn
    let poly = e.sketch.curve_polyline(i);
    assert!(poly.len() > 100, "{}", poly.len());
}

#[test]
fn a_frame_the_drawing_moves_refuses_what_it_cannot_mean() {
    let said = |extra: &str| crate::common::lenient::read(&four_bar(extra)).1;
    // the solve holds no relation to a profile read off the march
    let s = said("in std.front {\n  roller := circle(center: m)\n  radius(3mm) roller\n}\n\
        cut := envelope(roller over t, under: coupler.axes)\n\
        in std.front {\n  p := point hint((30, 20))\n  p coincident cut\n}\n");
    assert!(s.len() == 1 && s[0].contains("reads no march"), "{s:?}");
    // a reading over an interval of the roll that names no parameter finds the frame at rest
    let s = said("q := sweep(ball over r in (0deg, 90deg), under: coupler.axes)\n");
    assert!(s.len() == 1 && s[0].contains("does not move") && s[0].contains("over t"), "{s:?}");
    // a point carried is a row, and a row reads no march
    let s = said("X := point\nX coincident m at(under: coupler.axes) 30deg\n");
    assert!(s.len() == 1 && s[0].starts_with("E080") && s[0].contains("reads no march"), "{s:?}");
    // the roll is an angle
    let s = said("param s: Scalar over (0, 1)\nq := sweep(ball over s, under: coupler.axes)\n");
    assert!(s.len() == 1 && s[0].starts_with("E103") && s[0].contains("no angle"), "{s:?}");
}

#[test]
fn a_tool_placed_under_a_coupler_stands_where_the_coupler_puts_it() {
    // at an instant, the one parameter the document declares
    let e = solved(&four_bar("placed := ball at(under: coupler.axes) 90deg\n"));
    let c = midpoint(90.0);
    assert!(material(&e, "placed", c)[1] < -0.999);
    assert!(material(&e, "placed", midpoint(0.0))[0] > 0.5);
}

/// An RSSR linkage — a spatial four-bar: the crank `a`–`b` turns about `z` in the top view, driven
/// by `t`; the rocker `d`–`c` turns about an axis along `x` through `(X0, 0, H)`, in a plane square
/// to it; and the coupler between `b` and `c`, ball-jointed at both, keeps its length in space.
/// `m` is the coupler's midpoint, in space, traced as `t` runs.
const R1: f64 = 10.0;
const X0: f64 = 30.0;
const H: f64 = 30.0;
const R2: f64 = 35.0;
const L2: f64 = 55.0;

fn rssr() -> String {
    format!("unit mm\nuse std\nparam t: Angle over (0deg, 360deg)\n\
        in std.top {{\n  a := point\n  e := point\n  b := point hint(({R1}, 0))\n  \
        fix((0, 0)) a\n  fix((1, 0)) e\n  ground := line(a, e)\n  crank := line(a, b)\n  \
        length({R1}) crank\n  ground angle(t) crank\n}}\n\
        swing := plane\nfix(origin == ({X0}, 0, {H})) swing\nfix(dir == (0, 1, 0)) swing.u\n\
        fix(dir == (0, 0, 1)) swing.v\n\
        in swing {{\n  d := point\n  fix((0, 0)) d\n  c := point hint({c0:?})\n  \
        rocker := line(d, c)\n  length({R2}) rocker\n}}\n\
        b distance({L2}) c\n\
        m := point hint({m0:?})\nm midpoint line(b, c)\n\
        path := trace(m over t)\n",
        c0 = { let p = rocker_at(0.0, None).0; (p[1], p[2] - H) },
        m0 = { let m = coupler_mid(0.0, None).0; (m[0], m[1], m[2]) })
}

/// The rocker pin at the crank's angle `t` (degrees): `|b − c| = L2` is `A cos φ + B sin φ = C`
/// in the rocker's angle `φ` from `y`, its root nearer `near` (or the one with `z` above the
/// rocker's axis at the start).
fn rocker_at(t: f64, near: Option<f64>) -> ([f64; 3], f64) {
    let (s, c) = t.to_radians().sin_cos();
    let b = [R1 * c, R1 * s, 0.0];
    let (a_, b_) = (-2.0 * b[1] * R2, 2.0 * H * R2);
    let cc = L2 * L2 - (b[0] - X0).powi(2) - b[1] * b[1] - H * H - R2 * R2;
    let base = b_.atan2(a_);
    let spread = (cc / a_.hypot(b_)).acos();
    let roots = [base + spread, base - spread];
    let pin = |phi: f64| [X0, R2 * phi.cos(), H + R2 * phi.sin()];
    let phi = match near {
        Some(n) => *roots.iter().min_by(|x, y| {
            let d = |r: f64| (r - n).sin().abs() + (1.0 - (r - n).cos());
            d(**x).total_cmp(&d(**y))
        }).unwrap(),
        None => *roots.iter().max_by(|x, y| x.sin().total_cmp(&y.sin())).unwrap(),
    };
    (pin(phi), phi)
}

/// The coupler's midpoint at `t`, and the rocker's angle there, carried from `near`.
fn coupler_mid(t: f64, near: Option<f64>) -> ([f64; 3], f64) {
    let (s, c) = t.to_radians().sin_cos();
    let (pin, phi) = rocker_at(t, near);
    ([(R1 * c + pin[0]) / 2.0, (R1 * s + pin[1]) / 2.0, pin[2] / 2.0], phi)
}

#[test]
fn a_spatial_linkages_trace_is_its_closed_form() {
    let e = solved(&rssr());
    assert!(e.ok(), "{:?}", e.diags);
    let i = e.map.ent_named("path").expect("a trace in space").i();
    // in space and in no view: nothing on a sheet, its points in the world
    assert!(e.sketch.curve_polyline(i).is_empty());
    let pts = e.sketch.followed_world(i);
    assert_eq!(pts.len(), gcs_core::model::CURVE_STEPS + 1);
    let mut phi = coupler_mid(0.0, None).1;
    let mut worst = 0.0_f64;
    for (k, p) in pts.iter().enumerate() {
        let t = 360.0 * k as f64 / gcs_core::model::CURVE_STEPS as f64;
        // the rocker carried by continuity from the branch the seed picks
        let (want, next) = coupler_mid(t, Some(phi));
        phi = next;
        worst = worst.max((0..3).map(|j| (p[j] - want[j]).abs()).fold(0.0, f64::max));
    }
    assert!(worst < 1e-8, "{worst}");
    // it leaves every plane: a curve in space
    let z: Vec<f64> = pts.iter().map(|p| p[2]).collect();
    let y: Vec<f64> = pts.iter().map(|p| p[1]).collect();
    let spread = |v: &[f64]| v.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
        - v.iter().cloned().fold(f64::INFINITY, f64::min);
    assert!(spread(&z) > 1.0 && spread(&y) > 1.0);
    // and the scene in space draws it, where it stands
    let scene = gcs_core::overview::scene3d(&e.sketch, 1.0);
    assert!(scene.iter().any(|it| it.of.is_some_and(|o| o.kind == EntKind::Curve && o.i() == i)
        && it.pts.len() == pts.len()));
}
