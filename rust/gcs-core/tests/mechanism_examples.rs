//! **The mechanisms series** (`rust/examples/mechanisms/`, #391): linkages written over a declared
//! parameter, `param t: Angle over (0deg, 360deg)`, the crank's angle, and followed along it by
//! continuation from where they are drawn.  Each document is held, its one freedom driven by `t`,
//! and each motion is the closed form its comments give: a four-bar's coupler curve round a whole
//! turn, a slider-crank through both dead centres, a triple rocker stopped where its motion turns
//! back, and a four-bar at its change point stopped where its two assembly modes cross.
use gcs_core::diagnose::{diagnose, summary, DiagnoseOptions, State};
use gcs_core::follow::{follow, Followed, Kind};
use gcs_core::program::Elaborated;
use gcs_core::report::parameter_text;

use crate::common::parsed::determined;
use crate::continuation::closed_c;

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

/// `src` solved and held, its one freedom the crank's, driven by `t`; and followed along `t`.
fn driven(src: &str) -> (Elaborated, Followed) {
    let mut e = determined(src);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!(d.status, State::Well);
    assert_eq!((d.parameters.len(), d.parameters[0].dof), (1, Some(1)));
    assert!(summary(&d).contains("DOF 0; 1 DOF over t"), "{}", summary(&d));
    let f = follow(&e.sketch, "t").expect("followed");
    (e, f)
}

/// The largest distance, over every station of `f`, of the point `n` from where `want` puts it at
/// that station's `t` (degrees).
fn worst(e: &Elaborated, f: &Followed, n: &str, want: impl Fn(f64) -> (f64, f64)) -> f64 {
    let mut sk = e.sketch.clone();
    let i = index(e, n);
    f.stations.iter().map(|s| {
        sk.set_x(&s.x);
        let (got, want) = (sk.point_xy(i), want(s.at));
        (got.0 - want.0).hypot(got.1 - want.1)
    }).fold(0.0, f64::max)
}

/// **`four_bar.sv`**: the rocker pin and the coupler's midpoint at every station where the
/// closed form puts them, round the whole turn, and `coupler` closed on itself.
#[test]
fn the_four_bar_runs_round_on_its_closed_form() {
    let (g, r, l, k) = (40.0, 10.0, 35.0, 25.0);
    let (e, f) = driven(include_str!("../../examples/mechanisms/four_bar.sv"));
    assert!(f.singular.is_none(), "{:?}", f.singular);
    assert_eq!(f.reached(), 360.0);
    assert_eq!(parameter_text(&e.sketch, "t"), "t: followed from 0deg to 360deg");
    let c = |t: f64| closed_c(g, r, l, k, t, 1.0).expect("a crank-rocker closes everywhere");
    assert!(worst(&e, &f, "c", c) < 1e-8);
    let mid = |t: f64| {
        let ((s, co), c) = (t.to_radians().sin_cos(), c(t));
        ((r * co + c.0) / 2.0, (r * s + c.1) / 2.0)
    };
    assert!(worst(&e, &f, "m", mid) < 1e-8);
    let i = index(&e, "coupler");
    assert!(e.sketch.curve_closed(i));
    for t in [45.0, 135.0, 225.0, 315.0] {
        let (got, want) = (e.sketch.curve_point(i, t), mid(t));
        assert!((got.0 - want.0).hypot(got.1 - want.1) < 1e-8, "at {t}: {got:?}, {want:?}");
    }
}

/// **`slider_crank.sv`**: the slider at `r·cos t + sqrt(l² − r²·sin² t)` on the rail at every
/// station, through both dead centres, and `stroke` the rail between them, `2r` long.
#[test]
fn the_slider_crank_runs_through_both_dead_centres() {
    let (r, l): (f64, f64) = (10.0, 30.0);
    let (e, f) = driven(include_str!("../../examples/mechanisms/slider_crank.sv"));
    assert!(f.singular.is_none(), "{:?}", f.singular);
    assert_eq!(f.reached(), 360.0);
    let s = |t: f64| {
        let (sn, cs) = t.to_radians().sin_cos();
        (r * cs + (l * l - r * r * sn * sn).sqrt(), 0.0)
    };
    assert!(worst(&e, &f, "s", s) < 1e-8);
    let stroke = e.sketch.curve_polyline(index(&e, "stroke"));
    let (lo, hi) = stroke.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(p.0), hi.max(p.0)));
    assert!((lo - (l - r)).abs() < 1e-6 && (hi - (l + r)).abs() < 1e-6, "{lo} to {hi}");
    assert!(stroke.iter().all(|p| p.1.abs() < 1e-8));
    assert!(e.sketch.curve_closed(index(&e, "rod_path")));
}

/// **`triple_rocker.sv`**: followed to the closed form's limit and stopped there, the motion
/// turning back, said in the words its comments quote; `path` ends there.
#[test]
fn the_triple_rocker_stops_where_its_motion_turns_back() {
    let (g, r, l, k): (f64, f64, f64, f64) = (40.0, 20.0, 25.0, 30.0);
    let limit = ((r * r + g * g - (l + k) * (l + k)) / (2.0 * r * g)).acos().to_degrees();
    let (e, f) = driven(include_str!("../../examples/mechanisms/triple_rocker.sv"));
    let s = f.singular.expect("the input stops");
    assert_eq!(s.kind, Kind::Limit);
    assert!((s.at - limit).abs() < 1e-3, "{} against {limit}", s.at);
    assert_eq!(parameter_text(&e.sketch, "t"), "t: followed from 0deg to 129.838deg, singular at \
               t = 129.838deg: no drawing stands past it, the motion turns back");
    // the stations short of the limit on their closed form; at the limit the rocker pin moves as
    // the root of the distance to it, so the last stations, within the march's tolerance of it
    // (perhaps a hair past the closed form's), only to its square root
    let c = |t: f64| closed_c(g, r, l, k, t.min(limit), 1.0).expect("closes up to the limit");
    let mut short = f.clone();
    short.stations.retain(|s| s.at < limit - 1e-2);
    assert!(worst(&e, &short, "c", c) < 1e-8);
    assert!(worst(&e, &f, "c", c) < 1e-4);
    let i = index(&e, "path");
    assert!(!e.sketch.curve_closed(i));
    let end = *e.sketch.curve_polyline(i).last().expect("drawn");
    let want = closed_c(g, r, l, k, limit, 1.0).expect("closes at its limit");
    assert!((end.0 - want.0).hypot(end.1 - want.1) < 1e-2, "{end:?} against {want:?}");
}

/// **`change_point.sv`**: stopped at the fold, `t = 180deg`, where two branches cross; and with
/// the rocker a hair longer, as its comments say, carried through the near-fold round the turn.
#[test]
fn the_change_point_stops_where_two_branches_cross() {
    let src = include_str!("../../examples/mechanisms/change_point.sv");
    let (e, f) = driven(src);
    let s = f.singular.expect("the four-bar folds");
    assert_eq!(s.kind, Kind::Branch, "{s:?}");
    assert!((s.at - 180.0).abs() < 1e-2, "{s:?}");
    assert!(parameter_text(&e.sketch, "t").ends_with("two of its branches cross there"));

    let (g, r, l, k) = (40.0, 10.0, 30.0, 20.0001);
    let (e, f) = driven(&src.replace("k := 20 ", "k := 20.0001 "));
    assert!(f.singular.is_none(), "{:?}", f.singular);
    assert_eq!(f.reached(), 360.0);
    let c = |t: f64| closed_c(g, r, l, k, t, 1.0).expect("closes");
    assert!(worst(&e, &f, "c", c) < 1e-6);
}
