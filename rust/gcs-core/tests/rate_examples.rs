//! **The rates series** (`rust/examples/rates/`, issue #339 D2): each document solved to DOF 0,
//! each curve held to the closed form its comments give, and each claim a theorem — a tractrix
//! drawn and written in place, a clothoid easing a straight into a bend, a hanging chain shot to
//! its far end beside the free curve that hangs the same, a dog's pursuit refused where it catches
//! the hare, and a geodesic wound on a cone.
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::program::Elaborated;

use crate::common::parsed::determined;
use crate::rates::{fresnel, geodesic, tractrix};

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
