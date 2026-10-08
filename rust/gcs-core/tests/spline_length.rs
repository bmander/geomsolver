//! A spline's whole length, `length(L) s` (#121): one row over every control point, by
//! Gauss–Legendre per span.  Held to closed forms the document never states (a straight cubic,
//! the weighted quarter circle), to a fine polyline of the solved curve, through a free length, a
//! claim and the printed spelling.

use gcs_core::constraints::CKind;
use gcs_core::curve;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::io;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use crate::common::parse;
use std::f64::consts::PI;

fn read(src: &str) -> (Elaborated, Vec<String>) {
    let (prog, errs) = parse(src);
    let e = elaborate(&prog);
    let mut all: Vec<String> = errs.iter().map(|x| format!("syntax: {}", x.message)).collect();
    all.extend(e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)));
    (e, all)
}

fn solved(src: &str) -> Elaborated {
    let (mut e, d) = read(src);
    assert!(e.ok(), "{d:?}");
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}: {src}", r.message);
    e
}

/// The length of spline `i` as a polyline of 20 000 chords — worked out here, not asked of the
/// core's quadrature.
fn polyline_length(e: &Elaborated, i: usize) -> f64 {
    let (t0, t1) = curve::domain(&e.sketch, i);
    let n = 20_000;
    let mut last = curve::point_at(&e.sketch, i, t0);
    let mut sum = 0.0;
    for k in 1..=n {
        let p = curve::point_at(&e.sketch, i, t0 + (t1 - t0) * k as f64 / n as f64);
        sum += (p.0 - last.0).hypot(p.1 - last.1);
        last = p;
    }
    sum
}

/// The length row's own reading of spline `i`: its residual with nothing stated.
fn stated_length(e: &Elaborated) -> f64 {
    let c = e.sketch.constraints.iter().find(|c| c.kind == CKind::SplineLength).unwrap();
    c.args[1].num() + c.residual(&e.sketch, &c.local_values(&e.sketch))[0]
}

/// Four control points evenly along a line are a straight cubic run at constant speed, so its
/// length is the distance between its ends, exactly.
#[test]
fn a_straight_cubic_is_as_long_as_its_chord() {
    let e = solved(
        "use std\nin std.front {\n\
         k0 := point\nk1 := point\nk2 := point\nk3 := point\n\
         fix((0, 0)) k0\nfix((10, 0)) k1\nfix((20, 0)) k2\nfix((30, 0)) k3\n\
         s := spline(k0, k1, k2, k3)\nclaim length(30) s\n}\n",
    );
    assert!((stated_length(&e) - 30.0).abs() < 1e-12, "{}", stated_length(&e));
}

/// The weighted cubic quarter of radius 30 is a circle exactly, so its length is 15π — to the
/// rule's order on a curve whose speed is no polynomial.
#[test]
fn the_weighted_quarter_is_a_quarter_of_the_circumference() {
    let inner = (2.0 - 2f64.sqrt()) * 30.0;
    let heavy = (1.0 + 2f64.sqrt()) / 3.0;
    let e = solved(&format!(
        "use std\nin std.front {{\n\
         k0 := point\nk1 := point\nk2 := point\nk3 := point\n\
         fix((30, 0)) k0\nfix((30, {inner})) k1\nfix(({inner}, 30)) k2\nfix((0, 30)) k3\n\
         s := spline(k0, k1, k2, k3) weights [1, {heavy}, {heavy}, 1]\nclaim length(1) s\n}}\n"
    ));
    let l = stated_length(&e);
    assert!((l - 15.0 * PI).abs() < 1e-7 * 15.0 * PI, "{l} against {}", 15.0 * PI);
}

/// A length stated on a spline whose middle is free is solved for: the curve the solve leaves is
/// that long, measured as a fine polyline, and the statement prints as it was written.
#[test]
fn a_stated_length_bends_the_curve_to_it() {
    let src = "use std\nin std.front {\n\
               k0 := point\nk1 := point hint((10, 5))\nk2 := point hint((20, 8))\n\
               k3 := point hint((30, 4))\nk4 := point\n\
               fix((0, 0)) k0\nfix((40, 0)) k4\n\
               k1 distance(12) k0\nk3 distance(12) k4\nk2 level(x) k1\n\
               s := spline(k0, k1, k2, k3, k4)\nlength(55) s\n}\n";
    let e = solved(src);
    let l = polyline_length(&e, 0);
    assert!((l - 55.0).abs() < 1e-4, "{l}");
    let c = e.sketch.constraints.iter().find(|c| c.kind == CKind::SplineLength).unwrap();
    assert_eq!(io::describe_with(c, &|r| e.map.name_of(r).cloned()), "length(55) s");
    crate::common::fd_jacobian(&e.sketch, 1e-5);
}

/// A length the drawing has to find: two splines `L` long, one of them fixed, give `L` the fixed
/// one's length, and the other is bent to match.
#[test]
fn a_free_length_is_read_off_the_fixed_curve() {
    let src = "use std\nin std.front {\n\
               k0 := point\nk1 := point\nk2 := point\nk3 := point\n\
               fix((0, 0)) k0\nfix((10, 10)) k1\nfix((20, -10)) k2\nfix((30, 0)) k3\n\
               s := spline(k0, k1, k2, k3)\n\
               m0 := point\nm1 := point hint((10, 30))\nm2 := point hint((20, 30))\nm3 := point\n\
               fix((0, 20)) m0\nfix((30, 20)) m3\n\
               m1 level(y) m2\nm2 distance(10, along: x) m1\n\
               t := spline(m0, m1, m2, m3)\n}\n\
               param L: Length hint(40)\nlength(L) s\nlength(L) t\n";
    let e = solved(src);
    let (ls, lt) = (polyline_length(&e, 0), polyline_length(&e, 1));
    assert!((ls - lt).abs() < 1e-4, "{ls} against {lt}");
    crate::common::fd_jacobian(&e.sketch, 1e-5);
}

/// A claim of a length is judged, never solved for: the straight cubic is proved 30 long and
/// refuted 31 long.
#[test]
fn a_claimed_length_is_judged() {
    let doc = |l: f64| {
        format!(
            "use std\nin std.front {{\n\
             k0 := point\nk1 := point\nk2 := point\nk3 := point\n\
             fix((0, 0)) k0\nfix((10, 0)) k1\nfix((20, 0)) k2\nfix((30, 0)) k3\n\
             s := spline(k0, k1, k2, k3)\nclaim length({l}) s\n}}\n"
        )
    };
    for (l, held) in [(30.0, true), (31.0, false)] {
        let mut e = solved(&doc(l));
        let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
        let id = e.sketch.constraints.iter().find(|c| c.kind == CKind::SplineLength).unwrap().id;
        assert_eq!(d.claims_violated.contains(&id), !held, "{l}: {:?}", d.claims_violated);
    }
}
