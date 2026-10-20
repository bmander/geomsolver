//! **A frame the drawing carries** (#398 F): a frame with no joint, placed on geometry that moves
//! as the drawing is followed along a declared parameter — a four-bar's coupler — is a motion,
//! its pose wherever the march puts it, fitted between the stations (`motion::Track`).  What is
//! drawn in it is swept over the parameter, `sweep(tool over t, under: c)`, its swept field
//! conservative by the motion's bounds.  The gate: the frame against the four-bar's closed form,
//! its bounds against its samples, and a ball on the coupler swept through a blank against the
//! tube about the closed-form coupler curve.

use gcs_core::interval::minimum::{Options, Status};
use gcs_core::interval::Interval as I;
use gcs_core::model::{EntKind, SolidDef};
use gcs_core::motion::Family;
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;

use crate::common::text::solved;
use crate::continuation::closed_c;

const G: f64 = 40.0;
const R: f64 = 10.0;
const L: f64 = 35.0;
const K: f64 = 25.0;

/// A four-bar (ground `g`, crank `r`, coupler `l`, rocker `k`, the rocker pin seeded at `seed`)
/// with a frame on its coupler (`cpl.axes`, its `u` from the crank pin `b` toward the rocker pin
/// `c`), and in it a unit ball `ball` centred at the coupler's midpoint; `swept` the ball swept as
/// the crank turns over `over`.  `extra` adds statements.
fn four_bar((g, r, l, k): (f64, f64, f64, f64), seed: (f64, f64), over: &str, extra: &str)
    -> String {
    format!("unit mm\nuse std\nparam t: Angle over {over}\n\
        in std.front {{\n\
        \x20 a := point\n  d := point\n  b := point hint(({r}, 0))\n\
        \x20 c := point hint(({x}, {y}))\n  fix((0, 0)) a\n  fix(({g}, 0)) d\n\
        \x20 ground := line(a, d)\n  crank := line(a, b)\n  coupler_bar := line(b, c)\n\
        \x20 rocker := line(d, c)\n\
        \x20 length({r}) crank\n  length({l}) coupler_bar\n  length({k}) rocker\n\
        \x20 ground angle(t) crank\n\
        \x20 cpl := std.Turned(b, c)\n\
        }}\n\
        in cpl.axes {{\n\
        \x20 r0 := point\n  bottom := point\n  top := point\n\
        \x20 fix(({h}, 0)) r0\n  fix(({h}, -1)) bottom\n  fix(({h}, 1)) top\n\
        \x20 diameter := line(bottom, top)\n\
        \x20 meridian := arc(center: r0, start: bottom, end: top)\n\
        \x20 radius(1mm) meridian\n\
        }}\n\
        ball := solid(face(meridian, diameter), about: diameter)\n\
        swept := sweep(ball over t, under: cpl.axes)\n{extra}", x = seed.0, y = seed.1,
        h = l / 2.0)
}

/// `mechanisms/four_bar.sv`'s crank-rocker, so carried, its crank turned round.
fn linkage(extra: &str) -> String {
    four_bar((G, R, L, K), (G, K), "(0deg, 360deg)", extra)
}

/// The coupler's pin `b`, its unit direction toward `c` and its midpoint at the crank's angle `t`
/// (degrees), in the world: the front view's `u` is the world's `x` and its `v` the world's `z`.
fn coupler(t: f64) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let (s, co) = t.to_radians().sin_cos();
    let b = (R * co, R * s);
    let c = closed_c(G, R, L, K, t, 1.0).expect("a crank-rocker closes everywhere");
    let u = ((c.0 - b.0) / L, (c.1 - b.1) / L);
    let world = |x: f64, y: f64| [x, 0.0, y];
    (world(b.0, b.1), world(u.0, u.1), world((b.0 + c.0) / 2.0, (b.1 + c.1) / 2.0))
}

/// The swept solid's motion: the coupler's frame as the drawing carries it.
fn carried(e: &Elaborated) -> Family {
    let i = e.map.ent_named("swept").unwrap().i();
    let SolidDef::Swept { motion, .. } = e.sketch.solids[i].def else { panic!("not swept") };
    Family::read(&e.sketch, motion as usize).expect("the frame is carried")
}

fn close(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() <= tol)
}

#[test]
fn a_frame_on_a_coupler_is_carried_where_the_closed_form_puts_it() {
    let e = solved(&linkage(""));
    let family = carried(&e);
    let (b0, u0, m0) = coupler(0.0);
    let p = e.map.ent_named("r0").unwrap();
    assert_eq!(p.kind, EntKind::Point);
    assert!(close(e.sketch.world_point(p.i()), m0, 1e-9), "the ball is drawn at the midpoint");
    let mut worst = [0.0f64; 3];
    for k in 0..=720 {
        let t = 0.5 * k as f64;
        let (b, u, m) = coupler(t);
        let pose = family.at(t.to_radians()).unwrap();
        // the pin, the midpoint and the coupler's direction carried from home
        for (got, want, w) in [(pose.point(b0), b, 0), (pose.point(m0), m, 0)] {
            worst[w] = worst[w].max((0..3).map(|i| (got[i] - want[i]).abs()).fold(0.0, f64::max));
        }
        let turned = vector(&pose, u0);
        worst[1] = worst[1].max((0..3).map(|i| (turned[i] - u[i]).abs()).fold(0.0, f64::max));
        // the rate against a difference of the closed form
        let v = pose.velocity(m0);
        let h = 1e-4;
        let (_, _, ma) = coupler(t + h);
        let (_, _, mb) = coupler(t - h);
        let d: Vec<f64> = (0..3).map(|i| (ma[i] - mb[i]) / (2.0 * h.to_radians())).collect();
        worst[2] = worst[2].max((0..3).map(|i| (v[i] - d[i]).abs()).fold(0.0, f64::max));
        // the pose alone agrees
        let q = family.pose_at(t.to_radians()).unwrap().point(m0);
        assert!(close(q, pose.point(m0), 1e-12), "at {t}: {q:?}");
    }
    assert!(worst[0] < 1e-9 && worst[1] < 1e-10 && worst[2] < 1e-6, "{worst:?}");
    // a frame the drawing carries is no constant twist
    assert!(family.screw().is_none() && family.turning().is_none() && family.sliding().is_none());
    assert!(family.instantaneous().is_err());
}

/// A direction turned by `pose`.
fn vector(pose: &gcs_core::envelope::Motion, v: [f64; 3]) -> [f64; 3] {
    let (o, p) = (pose.point([0.0; 3]), pose.point(v));
    std::array::from_fn(|k| p[k] - o[k])
}

#[test]
fn a_carried_frames_bounds_enclose_its_poses_and_its_speed() {
    let e = solved(&linkage(""));
    let family = carried(&e);
    let (_, _, m0) = coupler(0.0);
    // the midpoint and a point off the coupler, out of the view
    for x in [m0, [m0[0] + 3.0, 2.0, m0[2] - 1.0]] {
        let boxed = x.map(|v| I::point(v).unwrap());
        for k in 0..24 {
            let [a, b] = [k, k + 1].map(|j| (15.0 * j as f64).to_radians());
            let span = I::new(a, b).unwrap();
            let bounds = family.bounds(span).unwrap();
            let enclosed = bounds.point(boxed).unwrap();
            let speed = family.inverse_point_speed_bound_over(boxed, span).unwrap();
            for j in 0..=40 {
                let theta = a + (b - a) * j as f64 / 40.0;
                let p = family.at(theta).unwrap().point(x);
                assert!((0..3).all(|i| enclosed[i].contains(p[i])), "{p:?} outside {enclosed:?}");
                // the inverse pose carries a point of the world back to the coupler's frame at
                // the speed the bound holds
                let inverse = |th: f64| family.at(th).unwrap().inverse().point(x);
                let h = 1e-6;
                let (hi, lo) = ((theta + h).min(b), (theta - h).max(a));
                let (p1, p0) = (inverse(hi), inverse(lo));
                let v = (0..3).map(|i| (p1[i] - p0[i]).powi(2)).sum::<f64>().sqrt() / (hi - lo);
                assert!(v <= speed * (1.0 + 1e-6), "speed {v} past the bound {speed}");
            }
        }
    }
}

/// The bounds a field's distance from solid `name`'s material at `p` is found within.
fn material(e: &Elaborated, name: &str, p: [f64; 3]) -> [f64; 2] {
    let i = e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i();
    let mut query = MaterialField::read(&e.sketch, i, 1e-10).unwrap().evaluator(256);
    let answer = query.bounds(p.map(|v| I::point(v).unwrap()),
        Options { value_tolerance: 1e-6, max_evaluations: 400_000 }).unwrap();
    assert!(answer.sweeps.iter().all(|s| s.minimum.status == Status::Converged));
    answer.value.bounds()
}

/// How far `p` stands from the tube of radius 1 about the coupler curve: the nearest of a fine
/// grid of the closed form, refined by halving steps either side.
fn from_tube(p: [f64; 3]) -> f64 {
    let d = |t: f64| {
        let c = coupler(t).2;
        (0..3).map(|k| (p[k] - c[k]).powi(2)).sum::<f64>().sqrt()
    };
    let mut t = (0..36000).map(|k| k as f64 / 100.0).min_by(|a, b| d(*a).total_cmp(&d(*b)))
        .unwrap();
    let mut h = 0.01;
    while h > 1e-10 {
        for s in [t - h, t + h] {
            if d(s) < d(t) {
                t = s
            }
        }
        h *= 0.5;
    }
    d(t) - 1.0
}

#[test]
fn a_ball_on_the_coupler_sweeps_the_tube_about_its_curve() {
    let e = solved(&linkage(""));
    // on the curve, inside the tube and outside it, in the view and off it
    let on = |t: f64, off: [f64; 3]| {
        let c = coupler(t).2;
        std::array::from_fn(|k| c[k] + off[k])
    };
    for p in [on(0.0, [0.0; 3]), on(75.0, [0.3, 0.2, -0.4]), on(140.0, [0.0, 0.0, 1.5]),
              on(210.0, [-2.0, 0.5, 0.0]), on(300.0, [0.0, 0.9, 0.0]), [20.0, 0.0, 12.0],
              [5.0, -0.4, -6.0]] {
        let [lo, hi] = material(&e, "swept", p);
        let want = from_tube(p);
        assert!(lo - 1e-6 <= want && want <= hi + 1e-6, "at {p:?}: [{lo}, {hi}] against {want}");
    }
}

#[test]
fn a_ball_on_the_coupler_cuts_its_curve_through_a_blank() {
    // a plate a millimetre thick in the front view, the ball's path cut through it
    let e = solved(&linkage("in std.front {\n  o := point\n  fix((20, 12)) o\n  \
        plate := std.CenteredRectangle(o, w: 70mm, h: 60mm)\n}\n\
        blank := solid(plate.loop, from: -0.5mm, to: 0.5mm)\nswept cut blank\n"));
    for t in [0.0, 50.0, 120.0, 200.0, 290.0] {
        // the path itself is cut away, and the plate a millimetre and a half off it is not
        let c = coupler(t).2;
        let [lo, _] = material(&e, "blank", c);
        assert!(lo > 0.0, "at {t}: the path is not cut ({lo})");
        let (_, u, _) = coupler(t);
        let side = [c[0] - 1.5 * u[2], 0.0, c[2] + 1.5 * u[0]];
        let [_, hi] = material(&e, "blank", side);
        if from_tube(side) > 0.25 {
            assert!(hi < 0.0, "at {t}: {side:?} is cut ({hi})");
        }
    }
}

#[test]
fn a_carried_frame_refuses_what_it_cannot_mean() {
    let said = |src: &str| crate::common::lenient::read(src).1;
    // what is read after the solve, said once the drawing stands
    let after = |src: &str| {
        let e = solved(src);
        gcs_core::program::solid_diagnostics(&e.sketch, &e.map).iter()
            .map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect::<Vec<_>>()
    };
    // a frame no part of the march moves
    let s = after(&linkage("in std.front {\n  still := std.Turned(a, d)\n}\n\
        idle := sweep(ball over t, under: still.axes)\n"));
    assert!(s.iter().any(|m| m.starts_with("E080") && m.contains("does not move as `t` runs")),
            "{s:?}");
    // an interval of the roll no parameter declares moves no frame with no joint
    let s = said(&linkage("loose := sweep(ball over s in (0deg, 90deg), under: cpl.axes)\n"));
    assert!(s.len() == 1 && s[0].starts_with("E080") && s[0].contains("does not move"), "{s:?}");
    // a triple rocker's march stops at its limit, and its sweep runs past it
    let s = after(&four_bar((40.0, 20.0, 25.0, 30.0), (23.0, 25.0), "(0deg, 360deg)", ""));
    assert!(s.iter().any(|m| m.starts_with("E080") && m.contains("runs from 0deg to 129.8")
                         && m.contains("where its march stops, and 360deg is past it")), "{s:?}");
    // an envelope, built with the drawing, reads no frame the drawing carries
    let s = said(&linkage("in cpl.axes {\n  k := circle(center: r0)\n  radius(2mm) k\n}\n\
        p := envelope(k over t, under: cpl.axes)\n"));
    assert!(s.iter().any(|m| m.contains("`cpl.axes` does not move")), "{s:?}");
    // nor what has no march to be carried by
    let s = after(&linkage("w := sweep(ball over u, under: cpl.axes)\n\
        param u: Angle over (0deg, 90deg)\n"));
    assert!(s.iter().any(|m| m.starts_with("E080") && m.contains("followed along `u`")
                         && m.contains("nothing the drawing states reads `u`")), "{s:?}");
}

#[test]
fn a_carried_frame_seen_from_another_is_their_relative_pose() {
    // the coupler seen from the crank: the crank pin stays where it is, and the coupler turns by
    // its own angle less the crank's
    let e = solved(&linkage("in std.front {\n  crk := std.Turned(a, b)\n  \
        still := std.Turned(a, d)\n}\n\
        relative := sweep(ball over t, under: cpl.axes, seen_from: crk.axes)\n\
        from_still := sweep(ball over t, under: cpl.axes, seen_from: still.axes)\n"));
    let i = e.map.ent_named("relative").unwrap().i();
    let SolidDef::Swept { motion, .. } = e.sketch.solids[i].def else { panic!("not swept") };
    let family = Family::read(&e.sketch, motion as usize).expect("the relative reads");
    let (b0, u0, _) = coupler(0.0);
    let bearing = |u: [f64; 3]| u[2].atan2(u[0]);
    for k in 0..=36 {
        let t = 10.0 * k as f64;
        let pose = family.at(t.to_radians()).unwrap();
        assert!(close(pose.point(b0), b0, 1e-9), "at {t}: {:?}", pose.point(b0));
        let (_, u, _) = coupler(t);
        let want = bearing(u) - t.to_radians() - bearing(u0);
        let got = bearing(vector(&pose, u0)) - bearing(u0);
        let off = (got - want).sin().abs();
        assert!(off < 1e-10, "at {t}: turned {got} against {want}");
    }
    // seen from a frame the march leaves still, it is seen from the world
    let i = e.map.ent_named("from_still").unwrap().i();
    let SolidDef::Swept { motion, .. } = e.sketch.solids[i].def else { panic!("not swept") };
    let from_still = Family::read(&e.sketch, motion as usize).expect("seen from the world");
    let alone = carried(&e);
    for k in 0..=36 {
        let t = (10.0 * k as f64).to_radians();
        let (p, q) = (from_still.at(t).unwrap().point(b0), alone.at(t).unwrap().point(b0));
        assert!(close(p, q, 1e-12), "{p:?} against {q:?}");
    }
}

#[test]
fn a_carried_frame_is_read_from_where_the_drawing_stands() {
    // stood at a quarter turn, the frame's home is there: the motion is the identity at 90deg
    // and carries what stands there back to where the crank started
    let mut e = solved(&linkage(""));
    gcs_core::follow::stand(&mut e.sketch, "t", 90.0).expect("stood");
    let family = carried(&e);
    let (_, _, m90) = coupler(90.0);
    let (_, _, m0) = coupler(0.0);
    let p = e.map.ent_named("r0").unwrap();
    assert!(close(e.sketch.world_point(p.i()), m90, 1e-8));
    assert!(close(family.at(90f64.to_radians()).unwrap().point(m90), m90, 1e-12));
    assert!(close(family.at(0.0).unwrap().point(m90), m0, 1e-9));
    gcs_core::follow::home(&mut e.sketch);
    assert!(close(carried(&e).at(0.0).unwrap().point(m0), m0, 1e-12));
}
