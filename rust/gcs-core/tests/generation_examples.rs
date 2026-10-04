//! **The generation series** (`rust/examples/generation/`, issue #61): each example held to what
//! its comments say, against closed forms or a linkage assembled here from circle intersections —
//! never against the core's own reading of the curve.

use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};

use crate::common::{build, radius_by_differences};

fn solved(src: &str) -> Elaborated {
    let mut e = build(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    e
}

fn dof(e: &mut Elaborated) -> (i64, State) {
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    (d.dof, d.status)
}

fn at(e: &Elaborated, name: &str) -> (f64, f64) {
    e.sketch.point_xy(e.map.ent_named(name).unwrap().i())
}

fn curve(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap().i()
}

fn unknown(e: &Elaborated, name: &str) -> f64 {
    e.sketch.params[e.sketch.free_vars[name] as usize].value
}

/// The unit tangent of curve `i` at `t`, by central differences of its points.
fn tangent(e: &Elaborated, i: usize, t: f64) -> (f64, f64) {
    let (h, (a, b)) = (1e-4, (e.sketch.curve_point(i, t - 1e-4), e.sketch.curve_point(i, t + 1e-4)));
    let (x, y) = ((b.0 - a.0) / (2. * h), (b.1 - a.1) / (2. * h));
    (x / x.hypot(y), y / x.hypot(y))
}

/// The distance from `q` to the line through `p` square to the unit `t`.
fn normal_miss(p: (f64, f64), t: (f64, f64), q: (f64, f64)) -> f64 {
    ((q.0 - p.0) * t.0 + (q.1 - p.1) * t.1).abs()
}

/// `p` turned by `deg` degrees about `c`.
fn turn(p: (f64, f64), c: (f64, f64), deg: f64) -> (f64, f64) {
    let (s, k) = deg.to_radians().sin_cos();
    (c.0 + k * (p.0 - c.0) - s * (p.1 - c.1), c.1 + s * (p.0 - c.0) + k * (p.1 - c.1))
}

/// **`stride.sv`**: the toe rod solves so the ground stands on both dips, to Jansen's number within
/// a twentieth (the bisection reference is `jansen.rs`'s `the_stride_stands_on_the_ground_twice`),
/// the drawn leg's rod as long, the crank still free.
#[test]
fn the_stride_stands_on_level_ground() {
    let mut e = solved(include_str!("../../examples/generation/stride.sv"));
    assert_eq!(dof(&mut e), (1, State::Under), "the crank is the one freedom");
    let h = unknown(&e, "leg.h");
    assert!((h - 65.684).abs() < 1e-3, "the rod solved to {h}");
    let ((hx, hy), (tx, ty)) = (at(&e, "leg.heel"), at(&e, "leg.toe"));
    assert!(((hx - tx).hypot(hy - ty) - h).abs() < 1e-6);
}

/// The four-bar of `dwell.sv`, assembled from circle intersections: the coupler point at crank
/// angle `deg` with the point `ap` along the coupler.
fn coupler_point(deg: f64, ap: f64) -> (f64, f64) {
    let (r2, r3, r4, beta) = (10., 30., 25., 50f64.to_radians());
    let a = (r2 * deg.to_radians().cos(), r2 * deg.to_radians().sin());
    let (dx, dy) = (30. - a.0, -a.1);
    let d = dx.hypot(dy);
    let m = (r3 * r3 - r4 * r4 + d * d) / (2. * d);
    let h = (r3 * r3 - m * m).sqrt();
    // B left of the line from A to the rocker's pivot
    let b = (a.0 + m * dx / d - h * dy / d, a.1 + m * dy / d + h * dx / d);
    let w = (b.1 - a.1).atan2(b.0 - a.0) + beta;
    (a.0 + ap * w.cos(), a.1 + ap * w.sin())
}

/// **`dwell.sv`**: the osculating circle at 170° has the stated radius by the coupler curve's own
/// differences, its centre the curve's centre of curvature, the coupler point where the linkage
/// assembled here puts it; the crank still free.
#[test]
fn the_dwell_arc_has_its_stated_radius() {
    let mut e = solved(include_str!("../../examples/generation/dwell.sv"));
    assert_eq!(dof(&mut e), (1, State::Under), "the crank is the one freedom");
    let ap = unknown(&e, "bar.ap");
    assert!((ap - 25.5).abs() < 0.1, "the coupler point solved to {ap}");
    let r = radius_by_differences(|d| coupler_point(d, ap), 170., 1e-2);
    assert!((r - 25.).abs() < 1e-5, "the coupler curve bends at {r} there");
    let (p, k) = (coupler_point(170., ap), at(&e, "k"));
    assert!(((p.0 - k.0).hypot(p.1 - k.1) - 25.).abs() < 1e-6, "the centre is not the centre of curvature");
}

/// **`heart_cam.sv`**: the pitch curve is the uniform-rise spiral, its halves meet, the profile's
/// halves touch the roller there, and the osculating circle's radius is the spiral's radius of
/// curvature less the roller's.
#[test]
fn the_heart_cam_rises_and_falls_uniformly() {
    let mut e = solved(include_str!("../../examples/generation/heart_cam.sv"));
    assert_eq!(dof(&mut e).0, 0);
    let (rise, fall) = (curve(&e, "pitch_rise"), curve(&e, "pitch_fall"));
    for k in 0..=12 {
        let t = 15. * k as f64;
        let (p, q) = (e.sketch.curve_point(rise, t), e.sketch.curve_point(fall, t + 180.));
        assert!((p.0.hypot(p.1) - (25. + 15. * t / 180.)).abs() < 1e-9, "rising at {t}");
        assert!((q.0.hypot(q.1) - (40. - 15. * t / 180.)).abs() < 1e-9, "falling at {}", t + 180.);
    }
    // the pitch curve's halves meet at the heart's point, where it turns a corner, so the profile's
    // halves leave the roller there along different normals, each a roller's radius off the point
    let point = e.sketch.curve_point(rise, 180.);
    let q = e.sketch.curve_point(fall, 180.);
    assert!((point.0 - q.0).hypot(point.1 - q.1) < 1e-9, "the pitch curve parts at the heart's point");
    for half in ["cam_rise", "cam_fall"] {
        let a = e.sketch.curve_point(curve(&e, half), 180.);
        assert!(((a.0 - point.0).hypot(a.1 - point.1) - 6.).abs() < 1e-9, "{half} leaves the roller");
    }
    // r = 25 + b·θ with b = 15/π a radian; at a quarter turn r = 32.5
    let (r, b) = (32.5_f64, 15. / std::f64::consts::PI);
    let rho = (r * r + b * b).powf(1.5) / (r * r + 2. * b * b) - 6.;
    let osc = e.map.ent_named("osc").unwrap().i();
    let got = e.sketch.params[e.sketch.circles[osc].radius as usize].value;
    assert!((got - rho).abs() < 1e-6, "the profile bends at {got}, the spiral says {rho}");
}

/// **`conjugate_cams.sv`**: at every roll the two touch with their common normal through the
/// pitch point, halfway between the shafts.
#[test]
fn the_conjugate_cams_mesh_through_the_pitch_point() {
    let mut e = solved(include_str!("../../examples/generation/conjugate_cams.sv"));
    assert_eq!(dof(&mut e).0, 0);
    let partner = curve(&e, "partner");
    for k in 0..12 {
        let t = 30. * k as f64 + 7.;
        // the partner's point at roll t, and its tangent, turned by its shaft into the fixed frame
        let q = turn(e.sketch.curve_point(partner, t), (40., 0.), -t);
        let (tx, ty) = tangent(&e, partner, t);
        let tq = turn((tx, ty), (0., 0.), -t);
        assert!(normal_miss(q, tq, (20., 0.)) < 1e-5, "at roll {t} the normal misses the pitch point");
    }
}

/// **`pin_wheel.sv`**: the pin's centre runs the epicycloid of the pinion's pitch circle rolling
/// round the wheel's, worked out here, and the flank is that path set in by the pin's radius.
#[test]
fn the_pin_cuts_an_epicycloidal_flank() {
    let mut e = solved(include_str!("../../examples/generation/pin_wheel.sv"));
    assert_eq!(dof(&mut e).0, 0);
    let (path, flank) = (curve(&e, "epicycloid"), curve(&e, "flank"));
    for k in 0..=8 {
        let t = -20. + 5. * k as f64;
        // the pinion turns three times as fast the other way; seen from the wheel, turned back
        let want = turn(turn((10., 0.), (0., 0.), -3. * t), (40., 0.), -t);
        let p = e.sketch.curve_point(path, t);
        assert!((p.0 - want.0).hypot(p.1 - want.1) < 1e-9, "the pin's centre at {t}: {p:?} against {want:?}");
        let q = e.sketch.curve_point(flank, t);
        assert!(((q.0 - p.0).hypot(q.1 - p.1) - 2.).abs() < 1e-9, "the flank at {t} is off the pin");
        assert!(normal_miss(q, tangent(&e, flank, t), p) < 1e-6, "the flank at {t} does not touch the pin");
    }
}
