//! Rational splines in the sketch: a weight per control point (`spline(…) weights […]`), document
//! data like the knots, so the curve stays linear in its control points over the rational basis
//! (`curve::weigh`). Held to closed forms: a weighted cubic quarter is a circle exactly, a knot put
//! in or a control point taken away keeps it what it was, the weights travel through a save, a
//! printed program and an expression, bad ones are refused where written, and `nurbs.sv` — the
//! case — proves its bead round, refutes its polynomial twin's and turns a hemisphere of ⅔πr³.
use gcs_core::curve;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::model::{EntRef, Sketch};
use gcs_core::program::{elaborate, to_program};
use std::f64::consts::PI;

const R: f64 = 30.0;

/// The weight of a cubic quarter circle's two inner control points, `(1 + √2) / 3`.
fn heavy() -> f64 { (1.0 + 2f64.sqrt()) / 3.0 }

/// The quarter of radius `R` about the origin as a weighted cubic Bézier, its control points fixed.
fn quarter(sk: &mut Sketch, weighted: bool) -> usize {
    let inner = (2.0 - 2f64.sqrt()) * R;
    let ctrl: Vec<usize> = [(R, 0.0), (R, inner), (inner, R), (0.0, R)]
        .iter()
        .enumerate()
        .map(|(i, &(x, y))| sk.point(x, y, true, &format!("k{i}")))
        .collect();
    let w = weighted.then(|| vec![1.0, heavy(), heavy(), 1.0]);
    sk.spline_weighted(&ctrl, None, w).unwrap()
}

fn radius_at(sk: &Sketch, s: usize, t: f64) -> f64 {
    let (x, y) = curve::point_at(sk, s, t);
    x.hypot(y)
}

#[test]
fn a_weighted_cubic_quarter_is_a_circle() {
    let mut sk = Sketch::new();
    let s = quarter(&mut sk, true);
    let p = quarter(&mut sk, false);
    let (t0, t1) = curve::domain(&sk, s);
    let mut plain_off: f64 = 0.0;
    for k in 0..=200 {
        let t = t0 + (t1 - t0) * k as f64 / 200.0;
        assert!((radius_at(&sk, s, t) - R).abs() <= 1e-13 * R, "off the circle at {t}");
        plain_off = plain_off.max((radius_at(&sk, p, t) - R).abs());
        // the tangent square to the radius, as a circle's is
        let f = curve::eval(&sk, s, t);
        let dot = f.p.0 * f.d1.0 + f.p.1 * f.d1.1;
        assert!(dot.abs() <= 1e-12 * R * f.d1.0.hypot(f.d1.1));
        // and the derivatives the points' differences
        let h = 1e-6;
        if t - h >= t0 && t + h <= t1 {
            let (a, b) = (curve::eval(&sk, s, t - h), curve::eval(&sk, s, t + h));
            assert!(((b.p.0 - a.p.0) / (2.0 * h) - f.d1.0).abs() <= 1e-6 * R);
            assert!(((b.p.1 - a.p.1) / (2.0 * h) - f.d1.1).abs() <= 1e-6 * R);
            assert!(((b.d1.0 - a.d1.0) / (2.0 * h) - f.d2.0).abs() <= 1e-5 * R);
            assert!(((b.d1.1 - a.d1.1) / (2.0 * h) - f.d2.1).abs() <= 1e-5 * R);
        }
    }
    // the same control points unweighted swell past it, by half a millimetre in thirty
    assert!(plain_off > 0.4, "the polynomial cubic is off by only {plain_off}");
}

#[test]
fn a_contact_with_a_weighted_curve_solves_onto_the_circle() {
    // a bead coincident with the weighted quarter, pulled toward a far point: it settles on the
    // curve, so on the circle
    let mut sk = Sketch::new();
    let s = quarter(&mut sk, true);
    let bead = sk.point(25.0, 21.0, false, "bead");
    let c = gcs_core::constraints::Constraint::point_on_spline(&sk, EntRef::point(bead), EntRef::spline(s));
    sk.add(c);
    let r = gcs_core::solve::solve(&mut sk, Default::default());
    assert!(r.success, "{}", r.message);
    let (x, y) = sk.point_xy(bead);
    assert!((x.hypot(y) - R).abs() <= 1e-9, "the bead at radius {}", x.hypot(y));
}

/// Six control points weighted unevenly: three spans, so an insertion lands inside the curve.
fn wavy(sk: &mut Sketch) -> usize {
    let ctrl: Vec<usize> = [(0.0, 0.0), (10.0, 14.0), (22.0, -3.0), (31.0, 12.0), (45.0, 4.0), (52.0, 18.0)]
        .iter()
        .enumerate()
        .map(|(i, &(x, y))| sk.point(x, y, false, &format!("w{i}")))
        .collect();
    sk.spline_weighted(&ctrl, None, Some(vec![1.0, 2.5, 0.6, 1.8, 0.9, 1.3])).unwrap()
}

#[test]
fn a_knot_put_into_a_weighted_curve_keeps_its_shape() {
    let mut sk = Sketch::new();
    let s = wavy(&mut sk);
    let (t0, t1) = curve::domain(&sk, s);
    let before: Vec<(f64, f64)> = (0..=60).map(|k| curve::point_at(&sk, s, t0 + (t1 - t0) * k as f64 / 60.0)).collect();
    for t in [0.37, 1.5, 2.81] {
        curve::insert_control(&mut sk, s, t).unwrap();
    }
    let sp = &sk.splines[s];
    assert_eq!(sp.ctrl.len(), 9);
    assert_eq!(sp.weights.as_ref().unwrap().len(), 9);
    assert!(curve::weights_valid(sp.weights.as_ref().unwrap(), 9));
    for (k, &(x, y)) in before.iter().enumerate() {
        let (u, v) = curve::point_at(&sk, s, t0 + (t1 - t0) * k as f64 / 60.0);
        assert!((u - x).hypot(v - y) <= 1e-11, "moved at sample {k}: ({u}, {v}) against ({x}, {y})");
    }
}

#[test]
fn weights_go_with_their_control_points() {
    let mut sk = Sketch::new();
    let s = wavy(&mut sk);
    // saved and read back
    let back = gcs_core::io::loads(&gcs_core::io::dumps(&sk, None)).unwrap();
    assert_eq!(back.splines[s].weights, sk.splines[s].weights);
    // a control point deleted: its weight goes with it, the rest stay with theirs
    let gone = sk.splines[s].ctrl[2] as usize;
    let less = gcs_core::io::without(&sk, &[EntRef::point(gone)], &[]);
    assert_eq!(less.splines[0].weights.as_deref(), Some(&[1.0, 2.5, 1.8, 0.9, 1.3][..]));
    // weights all 1 are the polynomial curve, stored without them
    let mut plain = Sketch::new();
    let ctrl: Vec<usize> = (0..4).map(|i| plain.point(i as f64, (i * i) as f64, false, "c")).collect();
    let p = plain.spline_weighted(&ctrl, None, Some(vec![1.0; 4])).unwrap();
    assert!(plain.splines[p].weights.is_none());
    // and a weight at or below zero, or a count that does not match, is no curve
    for bad in [vec![1.0, 0.0, 1.0, 1.0], vec![1.0, -2.0, 1.0, 1.0], vec![1.0, 1.0, 1.0], vec![1.0, f64::NAN, 1.0, 1.0]] {
        assert!(plain.spline_weighted(&ctrl, None, Some(bad.clone())).is_none(), "{bad:?}");
    }
}

fn elaborated(src: &str) -> gcs_core::program::Elaborated {
    let (prog, errs) = crate::common::parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    elaborate(&prog)
}

const WRITTEN: &str = "unit mm
use std
r := 30mm
in std.front {
  k0 := point hint((30mm, 0mm))
  k1 := point hint((30mm, 17mm))
  k2 := point hint((17mm, 30mm))
  k3 := point hint((0mm, 30mm))
  s := spline(k0, k1, k2, k3) weights [1, (1 + sqrt(2)) / 3, (1 + sqrt(2)) / 3, 1]
}
";

#[test]
fn weights_are_written_read_and_printed() {
    let e = elaborated(WRITTEN);
    assert!(e.ok(), "{:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    let w = e.sketch.splines[0].weights.clone().unwrap();
    assert_eq!(w, vec![1.0, heavy(), heavy(), 1.0]);
    // a printed program says them, and reads back to the same curve
    let text = to_program(&e.sketch).text().to_string();
    assert!(text.contains(" weights [1, "), "{text}");
    let again = elaborated(&text);
    assert!(again.ok());
    assert_eq!(again.sketch.splines[0].weights, Some(w));
    // over a component's own number, through an instance
    let comp = "unit mm
use std
component Quarter(w: Scalar) {
  k0 := point hint((30mm, 0mm))
  k1 := point hint((30mm, 17mm))
  k2 := point hint((17mm, 30mm))
  k3 := point hint((0mm, 30mm))
  s := spline(k0, k1, k2, k3) weights [1, w, w, 1]
}
q := Quarter(w: 2) in std.front
";
    let e = elaborated(comp);
    assert!(e.ok(), "{:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    assert_eq!(e.sketch.splines[0].weights.as_deref(), Some(&[1.0, 2.0, 2.0, 1.0][..]));
}

#[test]
fn weights_that_are_no_weights_are_refused_where_written() {
    for (weights, what) in [
        ("[1, -1, 1, 1]", "a negative weight"),
        ("[1, 0, 1, 1]", "a zero weight"),
        ("[1, 1, 1]", "too few"),
        ("[1, 2mm, 1, 1]", "a length"),
        ("[1, nowhere, 1, 1]", "a name nothing declares"),
    ] {
        let src = WRITTEN.replace("[1, (1 + sqrt(2)) / 3, (1 + sqrt(2)) / 3, 1]", weights);
        let e = elaborated(&src);
        assert!(
            e.errors().any(|d| d.code.as_str() == "E103"),
            "{what}: {:?}",
            e.errors().map(|d| (d.code.as_str(), d.message.clone())).collect::<Vec<_>>()
        );
    }
}

/// `nurbs.sv`, the case: the weighted bead's claim a theorem and the polynomial one's violated,
/// the dome exactly a hemisphere, and its STEP rational.
#[test]
fn the_case_proves_its_circle_and_turns_a_hemisphere() {
    let (prog, perr, lerr) = gcs_core::library::parse_linked(gcs_core::examples::source("nurbs").unwrap());
    assert!(perr.is_empty() && lerr.is_empty());
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    let mut sk = e.sketch;
    assert!(gcs_core::solve::solve(&mut sk, Default::default()).success);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    let reads = |ids: &[u32], name: &str| {
        let p = e.map.ent_named(name).unwrap();
        ids.iter().any(|&id| sk.constraints.iter().any(|c| c.id == id && c.entities().contains(&p)))
    };
    assert!(reads(&d.claims_theorem, "bead"), "the weighted bead's claim is not a theorem: {d:?}");
    assert!(reads(&d.claims_violated, "slider"), "the polynomial bead's claim is not refuted");
    assert_eq!((d.claims_theorem.len(), d.claims_violated.len()), (1, 1));
    // the dome, by the B-rep kernel: a hemisphere to rounding
    let dome = e.map.ent_named("dome").unwrap().i();
    let recipe = gcs_core::solid::cad::recipe(&sk, dome).unwrap();
    let b = gcs_core::brep::recipe::build(&recipe).unwrap();
    b.check(1e-9).unwrap();
    let v = gcs_core::brep::props::volume(&b);
    let whole = 2.0 / 3.0 * PI * R.powi(3);
    assert!((v - whole).abs() <= 1e-9 * whole, "{v} against {whole}");
    let text = gcs_core::brep::step::write(&b, "dome", 1e-4).unwrap();
    assert!(text.contains("RATIONAL_B_SPLINE_CURVE"));
    gcs_core::brep::step_check::verify(&text, &gcs_core::brep::step_check::Solid::of(&b)).unwrap();
}

#[test]
fn a_drag_on_a_weighted_curve_rides_the_circle() {
    // issue #134: the drag's part of the document is grafted, and its entities paired with the
    // document's by kind and order; the weighted quarter must come out as what went in
    let (prog, perr, lerr) = gcs_core::library::parse_linked(gcs_core::examples::source("nurbs").unwrap());
    assert!(perr.is_empty() && lerr.is_empty());
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    let mut sk = e.sketch;
    assert!(gcs_core::solve::solve(&mut sk, Default::default()).success);
    let bead = e.map.ent_named("bead").unwrap().i();
    let (x, y) = sk.point_xy(bead);
    let mut d = gcs_core::decompose::PlanDrag::new(&sk, bead, x, y, None, 0.05);
    assert!(d.move_to(&mut sk, None, 5.0, 40.0).success);
    let (x, y) = sk.point_xy(bead);
    assert!((x.hypot(y) - R).abs() < 1e-6, "the bead left the circle: ({x}, {y})");
}
