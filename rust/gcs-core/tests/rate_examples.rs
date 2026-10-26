//! **The rates series** (`rust/examples/rates/`, issue #339 D2): each document solved to DOF 0,
//! each curve held to the closed form its comments give, and each claim a theorem — a tractrix
//! drawn and written in place, a clothoid easing a straight into a bend, a hanging chain shot to
//! its far end beside the free curve that hangs the same, a dog's pursuit refused where it catches
//! the hare, and a geodesic wound on a cone.  And fronts marched by the method of lines (#370):
//! geodesic circles on a sphere, refused where they focus, and on a cone, and a Wankel bore as the
//! trochoid's parallel curve, refused at its waist.  And wavefronts in a graded medium (#378): a
//! front whose speed reads its own point, in Maxwell's fish-eye, refocused at the source's
//! conjugate and square to `fisheye.sv`'s free-curve ray.
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::program::Elaborated;

use crate::common::parsed::determined;
use crate::traced_rates::{fresnel, geodesic, tractrix};

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

fn xy(e: &Elaborated, name: &str) -> (f64, f64) {
    e.sketch.point_xy(index(e, name))
}

/// Every claim of the document a theorem, and there are `n` of them.
fn theorems(e: &mut Elaborated, n: usize) {
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert!(d.claims_violated.is_empty() && d.claims_consuming.is_empty(),
            "violated {:?}, consuming {:?}", d.claims_violated, d.claims_consuming);
    assert_eq!(d.claims_theorem.len(), n);
}

/// **`tractrix.sv`**: the drawn curve from its cusp, and the same body written in place marched
/// both ways, are the tractrix; where a drawn copy of the body stands on it, the string is `a`
/// long and the curve's tangent runs along it.
#[test]
fn the_tractrix_is_dragged_both_ways() {
    let mut e = determined(include_str!("../../examples/rates/tractrix.sv"));
    for (name, lo) in [("tractrix", 0.0), ("both_ways", -3.0)] {
        let k = index(&e, name);
        for i in 0..=30 {
            let u = lo + (3.0 - lo) * i as f64 / 30.0;
            let (x, y) = e.sketch.curve_point(k, u);
            let (cx, cy) = tractrix(30.0, u);
            assert!((x - cx).abs() < 1e-11 && (y - cy).abs() < 1e-11, "{name} at {u}: ({x}, {y})");
        }
    }
    let (m, want) = (xy(&e, "m"), tractrix(30.0, 1.2));
    assert!((m.0 - want.0).abs() < 1e-11 && (m.1 - want.1).abs() < 1e-11, "{m:?}");
    theorems(&mut e, 2);
}

/// **`clothoid.sv`**: the transition is the Fresnel spiral, and the bend osculating its end has
/// radius `A²/L`, its centre on the normal there.
#[test]
fn the_transition_eases_into_its_bend() {
    let mut e = determined(include_str!("../../examples/rates/clothoid.sv"));
    let k = index(&e, "transition");
    for i in 0..=16 {
        let s = 5.0 * i as f64;
        let (x, y) = e.sketch.curve_point(k, s);
        let (cx, cy) = fresnel(50.0, s);
        assert!((x - cx).abs() < 1e-10 && (y - cy).abs() < 1e-10, "at {s}: ({x}, {y})");
    }
    let bend = &e.sketch.circles[index(&e, "bend")];
    let r = e.sketch.params[bend.radius as usize].value;
    assert!((r - 2500.0 / 80.0).abs() < 1e-9, "radius {r}");
    let (p, turn) = (fresnel(50.0, 80.0), 80.0f64 * 80.0 / 5000.0);
    let c = e.sketch.point_xy(bend.center as usize);
    let want = (p.0 - r * turn.sin(), p.1 + r * turn.cos());
    assert!((c.0 - want.0).abs() < 1e-8 && (c.1 - want.1).abs() < 1e-8, "{c:?} against {want:?}");
    theorems(&mut e, 1);
}

/// **`chain.sv`**: the flow is shot to `b` — `c` the root of `c sinh(50/c) = 75`, `v = −75` —
/// and is the catenary, and the free curve of the same length between the same points is the
/// same curve.
#[test]
fn the_chain_as_a_flow_hangs_as_the_free_curve_does() {
    let e = determined(include_str!("../../examples/rates/chain.sv"));
    let value = |n: &str| e.sketch.params[*e.sketch.free_vars.get(n).unwrap() as usize].value;
    let (mut lo, mut hi) = (20.0f64, 40.0f64);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if mid * (50.0 / mid).sinh() > 75.0 { lo = mid } else { hi = mid }
    }
    let c = 0.5 * (lo + hi);
    assert!((value("hang.c") - c).abs() < 1e-9, "c {}", value("hang.c"));
    assert!((value("hang.v") + 75.0).abs() < 1e-9, "v {}", value("hang.v"));
    let (chain, rope) = (index(&e, "chain"), index(&e, "rope"));
    for i in 0..=15 {
        let s = 10.0 * i as f64;
        let (x, y) = e.sketch.curve_point(chain, s);
        let (cx, cy) = (50.0 + c * ((s - 75.0) / c).asinh(),
                        (c * c + (s - 75.0).powi(2)).sqrt() - (c * c + 75.0f64.powi(2)).sqrt());
        assert!((x - cx).abs() < 1e-9 && (y - cy).abs() < 1e-9, "chain at {s}: ({x}, {y})");
        let (rx, ry) = e.sketch.curve_point(rope, s / 150.0);
        assert!((x - rx).abs() < 1e-8 && (y - ry).abs() < 1e-8, "rope at {s}: ({rx}, {ry})");
    }
}

/// **`pursuit.sv`**: the dog's path is the pursuit curve `x(y)` of a speed ratio one half, and
/// the march is refused where it catches the hare, at `t = 20`, though the interval runs to 25.
#[test]
fn the_dog_catches_the_hare_where_its_rates_lose_rank() {
    let e = determined(include_str!("../../examples/rates/pursuit.sv"));
    let k = index(&e, "dog");
    let (a, n) = (30.0f64, 0.5f64);
    let x_of = |y: f64| {
        let r = y / a;
        let rise = r.powf(1.0 + n) / (1.0 + n) - r.powf(1.0 - n) / (1.0 - n);
        0.5 * a * rise + a * n / (1.0 - n * n)
    };
    for i in 0..=38 {
        let t = 0.5 * i as f64;
        let (x, y) = e.sketch.curve_point(k, t);
        assert!((x - x_of(y)).abs() < 1e-9, "at {t}: ({x}, {y}) against x = {}", x_of(y));
    }
    assert!(e.sketch.curve_point(k, 22.0).0.is_nan());
    let said = gcs_core::report::rates_text(&e.sketch, k).unwrap();
    assert!(said.starts_with("integrated from its home at 0, refused at 20")
        && said.ends_with("its rates lose rank there"), "{said}");
}

/// **`geodesic.sv`**: the path on the cone is the unrolled straight line, the point held at the
/// height where `ρ = 50` is the closed form's at `s = 30`, and the line held along the tangent
/// there is the closed form's tangent.
#[test]
fn the_filament_on_the_cone_unrolls_straight() {
    let e = determined(include_str!("../../examples/rates/geodesic.sv"));
    let path = index(&e, "path");
    for i in 0..=12 {
        let s = -60.0 + 10.0 * i as f64;
        let need = gcs_core::traced::Need::Point;
        let got = gcs_core::traced::reading(&e.sketch, path, [s, 0.0], need)
            .unwrap_or_else(|| panic!("no reading at {s}")).position;
        let want = geodesic(s);
        assert!((0..3).all(|k| (got[k] - want[k]).abs() < 1e-10), "at {s}: {got:?}");
    }
    let at = |n: &str| e.sketch.world_point(index(&e, n));
    let (h, h2, want) = (at("h"), at("h2"), geodesic(30.0));
    assert!((0..3).all(|k| (h[k] - want[k]).abs() < 1e-9), "{h:?} against {want:?}");
    let (a, b) = (geodesic(30.0 - 1e-4), geodesic(30.0 + 1e-4));
    let cos: f64 = (0..3).map(|k| (b[k] - a[k]) / 2e-4 * (h2[k] - h[k]) / 10.0).sum();
    assert!((cos.abs() - 1.0).abs() < 1e-7, "the tangent at {cos}");
}

/// A geodesic circle of `geodesic_circles.sv` at `(d, u)` (`u` in degrees): on the sphere of
/// radius 30 the circle of colatitude `30° + d/R`; on the cone of half angle 30° the circle
/// `30 + d` from the apex.
fn on_sphere(d: f64, u: f64) -> [f64; 3] {
    let (t, u) = (30f64.to_radians() + d / 30.0, u.to_radians());
    [30.0 * t.sin() * u.cos(), 30.0 * t.sin() * u.sin(), 30.0 * t.cos()]
}

fn on_cone(d: f64, u: f64) -> [f64; 3] {
    let (alpha, u, rho) = (30f64.to_radians(), u.to_radians(), 30.0 + d);
    [rho * alpha.sin() * u.cos(), rho * alpha.sin() * u.sin(), rho * alpha.cos()]
}

/// The greatest distance, over a turn of `u` every 5°, between surface `i` at `d` and `want`.
fn worst(sk: &gcs_core::model::Sketch, i: usize, d: f64, lo: f64, hi: f64,
         want: impl Fn(f64, f64) -> [f64; 3]) -> f64 {
    (0..=72).map(|k| {
        let u = lo + (hi - lo) * k as f64 / 72.0;
        let got = gcs_core::traced::reading(sk, i, [d, u], gcs_core::traced::Need::Point)
            .unwrap_or_else(|| panic!("no reading at ({d}, {u})")).position;
        let w = want(d, u);
        (0..3).map(|k| (got[k] - w[k]).abs()).fold(0.0, f64::max)
    }).fold(0.0, f64::max)
}

/// **`geodesic_circles.sv`**: a front on a sphere is the circle of latitude it has run to, and
/// is refused where it focuses on the antipode, short of where the closed form's area falls to a
/// hundredth; a front on a cone, unrolled, is a circle about the apex.  On a whole turn of
/// 32 pieces, the error growing as each front runs.
#[test]
fn fronts_on_a_surface_are_geodesic_circles() {
    let e = determined(include_str!("../../examples/rates/geodesic_circles.sv"));
    let (sphere, cone) = (index(&e, "on_sphere"), index(&e, "on_cone"));
    for d in [0.0, 10.0, 20.0, 40.0, 60.0, 70.0] {
        let far = worst(&e.sketch, sphere, d, 0.0, 360.0, on_sphere);
        assert!(far < 2e-7, "on the sphere at {d}: {far:.1e}");
    }
    // the area R sin θ falls to a hundredth of its home's at θ = π − asin(sin θ₀ / 100); the
    // march, its own error growing as the circle closes on a point, is refused a little short
    let (t0, pi) = (30f64.to_radians(), std::f64::consts::PI);
    let fold = 30.0 * (pi - (gcs_core::lines::FOLD * t0.sin()).asin() - t0);
    let at = match gcs_core::traced::lines_reached(&e.sketch, sphere, 32, 30.0 * pi) {
        Some(Err(Some((refusal, gcs_core::lines::Why::Fold { .. })))) => refusal.at(),
        other => panic!("{other:?}"),
    };
    assert!(at > fold - 0.3 && at < fold, "refused at {at}; the closed form's {fold}");
    let said = gcs_core::report::traced_text(&e.sketch, sphere);
    assert!(said.starts_with("on_sphere: a surface traced in space, marched from its home at 0, \
        refused at 78.2") && said.contains(": it folds there"), "{said}");
    for d in [0.0, 10.0, 20.0, 30.0] {
        let far = worst(&e.sketch, cone, d, 0.0, 360.0, on_cone);
        assert!(far < 1e-7, "on the cone at {d}: {far:.1e}");
    }
    assert_eq!(gcs_core::report::traced_text(&e.sketch, cone),
               "on_cone: a surface traced in space, marched from its home at 0 across (0, 30)");
}

/// The epitrochoid `housing.sv` writes (R 75, e 15) at `u` degrees, moved `d` along its outward
/// normal.
fn housing(d: f64, u: f64) -> [f64; 3] {
    let (r, e, u) = (75.0, 15.0, u.to_radians());
    let (x, y) = (e * (3.0 * u).cos() + r * u.cos(), e * (3.0 * u).sin() + r * u.sin());
    let tx = -3.0 * e * (3.0 * u).sin() - r * u.sin();
    let ty = 3.0 * e * (3.0 * u).cos() + r * u.cos();
    let l = tx.hypot(ty);
    [x + d * ty / l, y - d * tx / l, 0.0]
}

/// **`housing.sv`**: the bore is the epitrochoid's parallel curve at the seal's radius, its
/// error falling more than tenfold each time the pieces double; run past the waist's radius of
/// curvature, the front is refused at the waist, short of where the closed form's area falls to
/// a hundredth — on twice the pieces too, its own error growing as the waist focuses.
#[test]
fn the_housing_is_the_trochoids_parallel_curve() {
    let e = determined(include_str!("../../examples/rates/housing.sv"));
    let (bore, past) = (index(&e, "bore"), index(&e, "past"));
    for d in [0.0, 1.0, 2.0, 3.0] {
        let far = worst(&e.sketch, bore, d, 0.0, 180.0, housing);
        assert!(far < 5e-6, "the bore at {d}: {far:.1e}");
    }
    let error = |pieces: usize| -> f64 {
        (0..=72).map(|k| {
            let u = 2.5 * k as f64;
            let r = gcs_core::traced::lines_at(&e.sketch, bore, pieces, 3.0, [3.0, u],
                                               gcs_core::traced::Need::Point).unwrap();
            let w = housing(3.0, u);
            (0..3).map(|k| (r.x[k] - w[k]).abs()).fold(0.0, f64::max)
        }).fold(0.0, f64::max)
    };
    let table = [error(16), error(32), error(64)];
    assert!(table[0] / table[1] > 10.0 && table[1] / table[2] > 10.0, "{table:?}");
    // the waist's radius of curvature, (R − 3e)²/(9e − R), and a hundredth of the area short of it
    let fold = 30.0f64.powi(2) / 60.0 * (1.0 - gcs_core::lines::FOLD);
    for pieces in [32, 64] {
        match gcs_core::traced::lines_reached(&e.sketch, past, pieces, 20.0) {
            Some(Err(Some((refusal, gcs_core::lines::Why::Fold { sigma })))) => {
                assert!((sigma - 90.0).abs() < 1e-9, "{pieces} pieces: folds at u = {sigma}");
                let at = refusal.at();
                assert!(at > fold - 1.5 && at < fold, "{pieces} pieces: refused at {at}");
            }
            other => panic!("{pieces} pieces: {other:?}"),
        }
    }
    assert_eq!(gcs_core::report::traced_text(&e.sketch, bore),
               "bore: a surface traced in space, marched from its home at 0 across (0, 3)");
    let said = gcs_core::report::traced_text(&e.sketch, past);
    assert!(said.starts_with("past: a surface traced in space, marched from its home at 0, \
        refused at 13.7") && said.ends_with(": it folds there, near 90 along its second formal"),
            "{said}");
}


/// How far `(x, z)` is from the fish-eye front at optical length `s` from `a = (−R, 0)`, R 50: the
/// circle `cos(s/R)(x² + z²) + 2R x + R² cos(s/R) = 0`, its residual over its gradient.
fn off_the_front(s: f64, x: f64, z: f64) -> f64 {
    let (r, c) = (50.0, (s / 50.0).cos());
    let f = c * (x * x + z * z) + 2.0 * r * x + r * r * c;
    f / (2.0 * c * x + 2.0 * r).hypot(2.0 * c * z)
}

/// **`fisheye_fronts.sv`**: a front whose speed is 1/n read at its own point (#378).  In Maxwell's
/// fish-eye each front is the circle the lens's sphere has at that optical length from the
/// source, out to past the rim and through the line `x = 0`, where the circle turns inside out;
/// the fronts close on the source's conjugate, `(R, 0)`, where the march is refused, short of
/// `πR`.  And `fisheye.sv`'s free-curve ray from the same source crosses every front square to it,
/// at the optical length the closed form gives its point: the march and the extremal agree.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 13 s: the fish-eye's fronts marched")]
fn fish_eye_fronts_refocus_square_to_the_rays() {
    let e = determined(include_str!("../../examples/rates/fisheye_fronts.sv"));
    let wave = index(&e, "wave");
    let pi = std::f64::consts::PI;
    let at = |s: f64, u: f64| {
        gcs_core::traced::reading(&e.sketch, wave, [s, u], gcs_core::traced::Need::Point)
            .unwrap_or_else(|| panic!("no reading at ({s}, {u})")).position
    };
    for s in [2.0, 20.0, 40.0, 60.0, 25.0 * pi, 100.0, 120.0, 140.0, 150.0] {
        let far = (0..=48).map(|k| {
            let x = at(s, -120.0 + 5.0 * k as f64);
            off_the_front(s, x[0], x[2]).abs().max(x[1].abs())
        }).fold(0.0, f64::max);
        assert!(far < 1e-7, "at {s}: {far:.1e}");
    }
    // nearly there, every node within the front's radius R tan(π − s/R) of the conjugate
    let image = [50.0, 0.0, 0.0];
    let rho = 50.0 * (pi - 155.0 / 50.0).tan();
    for k in 0..=48 {
        let u = -120.0 + 5.0 * k as f64;
        let x = at(155.0, u);
        let d = (0..3).map(|i| (x[i] - image[i]).powi(2)).sum::<f64>().sqrt();
        // the error grows as the front focuses, at the arc's ends first
        assert!(d < rho + 0.05 && off_the_front(155.0, x[0], x[2]).abs() < 5e-5, "{u}: {x:?}");
    }
    let stop = match gcs_core::traced::lines_reached(&e.sketch, wave, 32, 160.0) {
        Some(Err(Some((refusal, gcs_core::lines::Why::Fold { .. })))) => refusal.at(),
        other => panic!("{other:?}"),
    };
    assert!(stop > 154.5 && stop < 50.0 * pi, "refused at {stop}");
    let said = gcs_core::report::traced_text(&e.sketch, wave);
    assert!(said.starts_with("wave: a surface traced in space, marched from its home at 2, \
        refused at 155.") && said.contains(": it folds there"), "{said}");
    // `fisheye.sv`'s ray, from the same source, drawn in the same plane
    let f = crate::common::lenient::solved(gcs_core::examples::source("fisheye").unwrap());
    let ray = |t: f64| -> [f64; 3] {
        let (x, z) = f.sketch.curve_point(0, t);
        [x, 0.0, z]
    };
    let dot = |p: &[f64], q: &[f64]| (0..3).map(|i| p[i] * q[i]).sum::<f64>();
    for k in 1..16 {
        let t = k as f64 / 16.0;
        let x = ray(t);
        // the optical length to it, a great circle's on the lens's sphere
        let sigma = 50.0 * (-100.0 * x[0] / (2500.0 + x[0] * x[0] + x[2] * x[2])).acos();
        // its place on the fronts, by Gauss–Newton from that length and the ray's own direction
        let mut place = [sigma, 90.0];
        let mut r = gcs_core::traced::reading(&e.sketch, wave, place,
                                              gcs_core::traced::Need::Tangent).unwrap();
        for _ in 0..12 {
            let f: Vec<f64> = (0..3).map(|i| r.position[i] - x[i]).collect();
            let (a, b) = (r.d[0], r.d[1]);
            let (aa, ab, bb) = (dot(&a, &a), dot(&a, &b), dot(&b, &b));
            let (fa, fb) = (dot(&f, &a), dot(&f, &b));
            let det = aa * bb - ab * ab;
            let step = [(bb * fa - ab * fb) / det, (aa * fb - ab * fa) / det];
            place = [place[0] - step[0], place[1] - step[1]];
            r = gcs_core::traced::reading(&e.sketch, wave, place, gcs_core::traced::Need::Tangent)
                .unwrap();
            if step[0].abs() + step[1].abs() < 1e-12 {
                break;
            }
        }
        let gap = (0..3).map(|i| (r.position[i] - x[i]).abs()).fold(0.0, f64::max);
        assert!(gap < 1e-9, "the ray's point at {t} is {gap:.1e} off the fronts");
        assert!((place[0] - sigma).abs() < 1e-6, "at {t}: s = {} against {sigma}", place[0]);
        let (a, b) = (ray(t - 1e-6), ray(t + 1e-6));
        let tangent: Vec<f64> = (0..3).map(|i| b[i] - a[i]).collect();
        let xu = r.d[1];
        let cos = dot(&xu, &tangent) / (dot(&xu, &xu) * dot(&tangent, &tangent)).sqrt();
        assert!(cos.abs() < 1e-6, "at {t}: the front is {cos:.1e} off square to the ray");
    }
}
