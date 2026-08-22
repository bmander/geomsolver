//! The B-spline basis, a spline's geometry, and what a contact with one costs.
use gcs_core::constraints::{Arg, CKind, Constraint};
use gcs_core::curve::{self, DEGREE, SPAN_N};
use gcs_core::kernels;
use gcs_core::model::{EntRef, Sketch};
use gcs_core::system::System;

fn bernstein(t: f64) -> [f64; 4] {
    let u = 1.0 - t;
    [u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t]
}

fn at(t: f64, u: &[f64], n: usize) -> ([f64; SPAN_N], [f64; SPAN_N], [f64; SPAN_N]) {
    let span = curve::span_index(u, n, t);
    let lk = curve::local_knots(u, span);
    let (mut b, mut d, mut dd) = ([0.0; SPAN_N], [0.0; SPAN_N], [0.0; SPAN_N]);
    curve::basis(DEGREE, t, &lk, &mut b, &mut d, &mut dd);
    (b, d, dd)
}

#[test]
fn a_four_point_clamped_spline_is_a_bezier() {
    let u = curve::clamped_uniform(4);
    assert_eq!(u, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
    for k in 0..=20 {
        let t = k as f64 / 20.0;
        let (b, _, _) = at(t, &u, 4);
        for (got, want) in b.iter().zip(&bernstein(t)) {
            assert!((got - want).abs() < 1e-12, "t={t}: {got} vs {want}");
        }
    }
}

#[test]
fn the_basis_is_a_partition_of_unity() {
    for n in [4usize, 5, 7, 12] {
        let u = curve::clamped_uniform(n);
        let (t0, t1) = (u[DEGREE], u[n]);
        for k in 0..=40 {
            let t = t0 + (t1 - t0) * k as f64 / 40.0;
            let (b, d, dd) = at(t, &u, n);
            let s: f64 = b.iter().sum();
            assert!((s - 1.0).abs() < 1e-12, "n={n} t={t}: sum {s}");
            // a constant curve has no velocity, whatever the control points are
            assert!(d.iter().sum::<f64>().abs() < 1e-9);
            assert!(dd.iter().sum::<f64>().abs() < 1e-8);
        }
    }
}

#[test]
fn basis_derivatives_agree_with_finite_differences() {
    for n in [4usize, 6, 9] {
        let u = curve::clamped_uniform(n);
        let (t0, t1) = (u[DEGREE], u[n]);
        let h = 1e-5;
        for k in 1..40 {
            let t = t0 + (t1 - t0) * k as f64 / 40.0;
            // stay inside one span, so the comparison is of one polynomial piece
            if (t - t.round()).abs() < 2.0 * h {
                continue;
            }
            let (_, d, dd) = at(t, &u, n);
            let (bp, dp, _) = at(t + h, &u, n);
            let (bm, dm, _) = at(t - h, &u, n);
            for a in 0..SPAN_N {
                assert!(((bp[a] - bm[a]) / (2.0 * h) - d[a]).abs() < 1e-6, "n={n} t={t} a={a}");
                assert!(((dp[a] - dm[a]) / (2.0 * h) - dd[a]).abs() < 1e-5, "n={n} t={t} a={a}");
            }
        }
    }
}

#[test]
fn a_clamped_spline_runs_from_its_first_control_point_to_its_last() {
    let mut sk = Sketch::new();
    let pts: Vec<usize> = [(0.0, 0.0), (1.0, 3.0), (4.0, 3.0), (5.0, 0.0), (8.0, -2.0)]
        .iter()
        .enumerate()
        .map(|(i, &(x, y))| sk.point(x, y, false, &format!("c{i}")))
        .collect();
    let s = sk.spline(&pts).unwrap();
    let (t0, t1) = curve::domain(&sk, s);
    let a = curve::point_at(&sk, s, t0);
    let b = curve::point_at(&sk, s, t1);
    assert!((a.0 - 0.0).abs() < 1e-12 && (a.1 - 0.0).abs() < 1e-12);
    assert!((b.0 - 8.0).abs() < 1e-12 && (b.1 + 2.0).abs() < 1e-12);
    assert_eq!(t1 - t0, 2.0); // five control points, two spans
}

#[test]
fn tessellation_lands_on_the_curve_and_gets_finer_with_the_zoom() {
    let mut sk = Sketch::new();
    let pts: Vec<usize> = [(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]
        .iter()
        .enumerate()
        .map(|(i, &(x, y))| sk.point(x, y, false, &format!("c{i}")))
        .collect();
    let s = sk.spline(&pts).unwrap();
    let coarse = curve::tessellate(&sk, s, 1.0);
    let fine = curve::tessellate(&sk, s, 0.01);
    assert!(fine.len() > coarse.len());
    for &(x, y) in &fine {
        assert!(curve::distance_to(&sk, s, x, y) < 1e-6);
    }
}

/// The point of the whole design: a contact addresses one span, so its column count does not
/// grow with the spline.
#[test]
fn a_contact_costs_the_span_not_the_spline() {
    let mut widths = Vec::new();
    for n in [4usize, 8, 40] {
        let mut sk = Sketch::new();
        let pts: Vec<usize> =
            (0..n).map(|i| sk.point(i as f64, (i % 3) as f64, false, "c")).collect();
        let s = sk.spline(&pts).unwrap();
        let p = sk.point(3.0, 5.0, false, "p");
        let c = Constraint::point_on_spline(&sk, EntRef::point(p), EntRef::spline(s));
        let id = sk.add(c);
        widths.push(sk.constraint(id).unwrap().params(&sk).len());
    }
    assert_eq!(widths, vec![3 + 2 * SPAN_N; 3]);
}

#[test]
fn a_contact_owns_one_unknown_and_removes_one_degree_of_freedom() {
    let mut sk = Sketch::new();
    let pts: Vec<usize> = (0..4).map(|i| sk.point(i as f64, 0.0, false, "c")).collect();
    let s = sk.spline(&pts).unwrap();
    let p = sk.point(1.5, 2.0, false, "p");
    let before = sk.params.len();
    let id = sk.add(Constraint::point_on_spline(&sk, EntRef::point(p), EntRef::spline(s)));
    assert_eq!(sk.params.len(), before + 1, "one hidden unknown");
    assert_eq!(sk.constraint(id).unwrap().aux_params().len(), 1);
    let sys = System::new(&sk);
    assert_eq!(sys.n_res, 2, "two residuals against the one new unknown");
    // net: 10 free params - 2 equations = 8, where the point alone would have been 10 - 0
    assert_eq!(sys.n_free, before + 1);
}

#[test]
fn the_span_a_contact_sits_on_is_part_of_the_topology() {
    let mut sk = Sketch::new();
    let pts: Vec<usize> = (0..6).map(|i| sk.point(i as f64, 0.0, false, "c")).collect();
    let s = sk.spline(&pts).unwrap();
    let p = sk.point(0.5, 1.0, false, "p");
    let id = sk.add(Constraint::point_on_spline(&sk, EntRef::point(p), EntRef::spline(s)));
    let key = sk.topology_key();
    let t = sk.constraint(id).unwrap().aux_params()[0] as usize;
    assert_eq!(curve::span_of(&sk, s, sk.params[t].value), DEGREE);
    sk.params[t].value = 2.5; // walk the contact into the last span
    assert_ne!(curve::span_of(&sk, s, sk.params[t].value), DEGREE);
    assert_ne!(sk.topology_key(), key, "a contact past a knot is a recompile");
}

/* -- solving ---------------------------------------------------------------- */

fn wave(sk: &mut Sketch, n: usize) -> usize {
    let pts: Vec<usize> = (0..n)
        .map(|i| {
            let x = i as f64 * 10.0;
            let y = if i % 2 == 0 { 0.0 } else { 12.0 };
            sk.point(x, y, false, &format!("c{i}"))
        })
        .collect();
    sk.spline(&pts).unwrap()
}

fn solved(sk: &mut Sketch) -> bool {
    let r = gcs_core::solve::solve(sk, gcs_core::solve::SolveOpts::default());
    let mut sys = System::new(sk);
    let z = sys.z0(sk);
    r.success && sys.max_relative_residual(&z) <= 1e-6
}

#[test]
fn a_point_is_pulled_onto_the_curve() {
    let mut sk = Sketch::new();
    let s = wave(&mut sk, 6);
    for c in 0..sk.points.len() {
        sk.fix_point(c, true); // hold the curve still: only the point may move
    }
    let p = sk.point(21.0, 30.0, false, "p");
    sk.add(Constraint::point_on_spline(&sk.clone(), EntRef::point(p), EntRef::spline(s)));
    assert!(solved(&mut sk));
    let (x, y) = sk.point_xy(p);
    assert!(curve::distance_to(&sk, s, x, y) < 1e-9, "point is off the curve");
}

#[test]
fn the_curve_comes_to_the_point_when_the_point_is_the_fixed_one() {
    let mut sk = Sketch::new();
    let s = wave(&mut sk, 5);
    let p = sk.point(21.0, 30.0, true, "p");
    sk.add(Constraint::point_on_spline(&sk.clone(), EntRef::point(p), EntRef::spline(s)));
    assert!(solved(&mut sk));
    assert!(curve::distance_to(&sk, s, 21.0, 30.0) < 1e-9);
}

#[test]
fn a_line_is_made_tangent_to_a_curve() {
    let mut sk = Sketch::new();
    let s = wave(&mut sk, 6);
    for c in 0..sk.points.len() {
        sk.fix_point(c, true);
    }
    let a = sk.point(0.0, -20.0, false, "a");
    let b = sk.point(50.0, -20.0, false, "b");
    let l = sk.line(a, b);
    let id = sk.add(Constraint::spline_tangent_line(&sk.clone(), EntRef::spline(s),
                                                    EntRef::line(l)));
    assert!(solved(&mut sk));
    let t = sk.params[sk.constraint(id).unwrap().aux_params()[0] as usize].value;
    let f = curve::eval(&sk, s, t);
    // the contact point is on the line, and the curve's direction there is the line's
    let (ax, ay) = sk.point_xy(a);
    let (bx, by) = sk.point_xy(b);
    let (dx, dy) = (bx - ax, by - ay);
    let off = (dx * (f.p.1 - ay) - dy * (f.p.0 - ax)) / dx.hypot(dy);
    assert!(off.abs() < 1e-6, "contact is {off} off the line");
    let cross = (f.d1.0 * dy - f.d1.1 * dx) / (f.d1.0.hypot(f.d1.1) * dx.hypot(dy));
    assert!(cross.abs() < 1e-6, "directions differ by {cross}");
}

#[test]
fn a_tangency_costs_one_degree_of_freedom() {
    let mut sk = Sketch::new();
    let s = wave(&mut sk, 4);
    let a = sk.point(0.0, -20.0, false, "a");
    let b = sk.point(50.0, -20.0, false, "b");
    let l = sk.line(a, b);
    let before = gcs_core::diagnose::diagnose(&mut sk.clone(), Default::default()).dof;
    sk.add(Constraint::spline_tangent_line(&sk.clone(), EntRef::spline(s), EntRef::line(l)));
    // the constraint brings one unknown of its own and two equations: net one
    assert_eq!(gcs_core::diagnose::diagnose(&mut sk, Default::default()).dof, before - 1 + 1 - 2 + 1);
}

#[test]
fn a_contact_survives_save_and_load() {
    let mut sk = Sketch::new();
    let s = wave(&mut sk, 7);
    let p = sk.point(21.0, 30.0, false, "p");
    let id = sk.add(Constraint::point_on_spline(&sk.clone(), EntRef::point(p), EntRef::spline(s)));
    assert!(solved(&mut sk));
    let t = sk.params[sk.constraint(id).unwrap().aux_params()[0] as usize].value;
    let text = gcs_core::io::dumps(&sk, None);
    let back = gcs_core::io::loads(&text).unwrap();
    assert_eq!(back.splines.len(), 1);
    assert_eq!(back.splines[0].ctrl, sk.splines[0].ctrl);
    assert_eq!(back.splines[0].knots, sk.splines[0].knots);
    let c = back.constraints.iter().find(|c| c.kind == CKind::PointOnSpline).unwrap();
    let t2 = back.params[c.aux_params()[0] as usize].value;
    assert!((t - t2).abs() < 1e-12, "the curve parameter came back as {t2}, not {t}");
}

#[test]
fn copying_a_spline_takes_its_contacts_and_gives_them_their_own_unknowns() {
    let mut sk = Sketch::new();
    let s = wave(&mut sk, 5);
    let p = sk.point(21.0, 30.0, false, "p");
    sk.add(Constraint::point_on_spline(&sk.clone(), EntRef::point(p), EntRef::spline(s)));
    let sel: Vec<EntRef> = vec![EntRef::spline(s), EntRef::point(p)];
    let clip = gcs_core::io::copy(&sk, &sel);
    assert_eq!(clip.splines.len(), 1);
    let c = clip.constraints.iter().find(|c| c.kind == CKind::PointOnSpline).unwrap();
    let owned = c.aux_params();
    assert_eq!(owned.len(), 1);
    // the copy's unknown is its own Param, not a stale index into the sketch it came from
    assert!((owned[0] as usize) < clip.params.len());
    assert!(!clip.params[owned[0] as usize].fixed);
}

#[test]
fn deleting_a_contact_takes_its_unknown_out_of_the_count() {
    let mut sk = Sketch::new();
    let s = wave(&mut sk, 5);
    let p = sk.point(21.0, 30.0, false, "p");
    let id = sk.add(Constraint::point_on_spline(&sk.clone(), EntRef::point(p), EntRef::spline(s)));
    let free = sk.free_indices().len();
    sk.remove(id);
    assert_eq!(sk.free_indices().len(), free - 1, "an orphaned unknown is not a spare DOF");
    // and the rebuild walk reclaims the slot outright
    let rebuilt = gcs_core::io::without(&sk, &[], &[]);
    assert_eq!(rebuilt.params.len(), sk.params.len() - 1);
}

#[test]
fn a_duplicate_contact_is_recognised_however_far_apart_the_parameters_started() {
    let mut sk = Sketch::new();
    let s = wave(&mut sk, 5);
    let p = sk.point(21.0, 30.0, false, "p");
    let a = Constraint::point_on_spline(&sk, EntRef::point(p), EntRef::spline(s));
    let mut b = a.clone();
    b.args[2] = Arg::Num(1.7);
    assert!(gcs_core::constraints::same_constraint(&a, &b));
}

/* -- staying on the curve --------------------------------------------------- */

#[test]
fn a_contact_will_not_settle_on_the_phantom_extension_of_a_curve() {
    // the basis is a polynomial and evaluates happily outside the knot vector; a tangency there
    // is a perfectly good solution of the equations and a nonsense answer
    let mut sk = Sketch::new();
    let pts: Vec<usize> = [(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]
        .iter()
        .map(|&(x, y)| sk.point(x, y, false, "c"))
        .collect();
    let s = sk.spline(&pts).unwrap();
    for c in 0..sk.points.len() {
        sk.fix_point(c, true);
    }
    let a = sk.point(-6.0, 0.0, false, "a");
    let b = sk.point(-6.0, 10.0, false, "b");
    let l = sk.line(a, b);
    let id = sk.add(Constraint::spline_tangent_line(&sk.clone(), EntRef::spline(s),
                                                    EntRef::line(l)));
    assert!(solved(&mut sk));
    let t = sk.params[sk.constraint(id).unwrap().aux_params()[0] as usize].value;
    let (t0, t1) = curve::domain(&sk, s);
    assert!(t >= t0 - 1e-12 && t <= t1 + 1e-12, "contact settled at t={t}, off {t0}..{t1}");
}

#[test]
fn dragging_a_point_along_a_curve_carries_it_across_a_knot() {
    let mut sk = Sketch::new();
    let s = wave(&mut sk, 7);
    for c in 0..sk.points.len() {
        sk.fix_point(c, true);
    }
    let p = sk.point(2.0, 4.0, false, "p");
    let id = sk.add(Constraint::point_on_spline(&sk.clone(), EntRef::point(p),
                                                EntRef::spline(s)));
    assert!(solved(&mut sk));
    let tp = sk.constraint(id).unwrap().aux_params()[0] as usize;
    let first = curve::span_of(&sk, s, sk.params[tp].value);
    let mut drag = gcs_core::solve::Drag::new(&mut sk, p, 2.0, 4.0,
        gcs_core::newton::Method::DogLeg, 1.0, Vec::new(), 0.05);
    // pull it most of the way along the curve, which is several spans away
    let far = curve::point_at(&sk, s, curve::domain(&sk, s).1 - 0.2);
    drag.move_to(&mut sk, far.0, far.1);
    drag.end(&mut sk);
    let last = curve::span_of(&sk, s, sk.params[tp].value);
    assert!(last > first, "the contact never left span {first}");
    let (x, y) = sk.point_xy(p);
    assert!(curve::distance_to(&sk, s, x, y) < 1e-6, "the dragged point left the curve");
    let (t0, t1) = curve::domain(&sk, s);
    let t = sk.params[tp].value;
    assert!(t >= t0 - 1e-9 && t <= t1 + 1e-9);
}

/// A geometry problem does not know how big it is, and neither should the solver: the same
/// tangency at ten times the size is the same number of iterations.  It is `Param::scale` that
/// makes that true — a curve parameter is dimensionless, so without it a contact's column is
/// wrong by a factor of the curve's length and the conditioning follows the drawing's size.
#[test]
fn a_tangency_converges_the_same_at_any_size() {
    let shape = [(0.0, 0.0), (10.0, 12.0), (20.0, 0.0), (30.0, 12.0), (40.0, 0.0), (50.0, 12.0)];
    let mut answers = Vec::new();
    for k in [1.0f64, 10.0, 100.0] {
        let mut sk = Sketch::new();
        let pts: Vec<usize> =
            shape.iter().map(|&(x, y)| sk.point(x * k, y * k, false, "c")).collect();
        let s = sk.spline(&pts).unwrap();
        for c in 0..sk.points.len() {
            sk.fix_point(c, true);
        }
        let a = sk.point(0.0, -20.0 * k, false, "a");
        let b = sk.point(50.0 * k, -20.0 * k, false, "b");
        let l = sk.line(a, b);
        let id = sk.add(Constraint::spline_tangent_line(&sk.clone(), EntRef::spline(s),
                                                        EntRef::line(l)));
        assert!(solved(&mut sk), "failed at size {k}");
        answers.push(sk.params[sk.constraint(id).unwrap().aux_params()[0] as usize].value);
    }
    for w in answers.windows(2) {
        assert!((w[0] - w[1]).abs() < 1e-6, "different contact per size: {answers:?}");
    }
}

#[test]
fn a_sketch_with_a_curve_in_it_still_diagnoses_and_reports() {
    let mut sk = Sketch::new();
    let s = wave(&mut sk, 5);
    let p = sk.point(21.0, 30.0, false, "p");
    sk.add(Constraint::point_on_spline(&sk.clone(), EntRef::point(p), EntRef::spline(s)));
    let d = gcs_core::diagnose::diagnose(&mut sk, Default::default());
    assert!(d.conflicts.unwrap_or_default().is_empty());
    assert!(d.entity_state.contains_key(&EntRef::spline(s)));
    // the whole reporting surface is exercised, including the registry both bindings build from
    let _ = gcs_core::report::constraints_json(&sk);
    let _ = gcs_core::report::registry_json();
    let _ = gcs_core::report::callouts_json(&sk, 0.1);
    assert!(gcs_core::io::describe(sk.constraints.last().unwrap()).starts_with("PointOnSpline("));
}

#[test]
fn a_compiled_system_keeps_the_span_it_was_built_from() {
    // A block's columns name one span's control points and its constants are that span's knots.
    // Refreshing the constants from a parameter that has since moved to another span would put
    // one span's knots against another's columns, and the residual would be quietly wrong.
    let mut sk = Sketch::new();
    let s = wave(&mut sk, 7);
    let p = sk.point(2.0, 4.0, false, "p");
    let id = sk.add(Constraint::point_on_spline(&sk.clone(), EntRef::point(p),
                                                EntRef::spline(s)));
    let mut sys = System::new(&sk);
    let tp = sk.constraint(id).unwrap().aux_params()[0] as usize;
    let compiled = curve::span_of(&sk, s, sk.params[tp].value);

    sk.params[tp].value = curve::domain(&sk, s).1 - 0.25;      // walk it to the far span
    assert_ne!(curve::span_of(&sk, s, sk.params[tp].value), compiled);
    sys.refresh_consts(&sk);

    // the columns the block still names: the point, the parameter, and the compiled span's
    // control points — `local_values` would give the span the parameter has moved to
    let c = sk.constraint(id).unwrap();
    let cols: Vec<u32> = [sk.point_params(p).to_vec(), vec![c.aux_params()[0]],
                          sk.spline_span_params(s, compiled)].concat();
    let v: Vec<f64> = cols.iter().map(|&i| sk.params[i as usize].value).collect();
    let want = kernels::eval_one(c.kernel_id(), &v, &c.consts_on(&sk, Some(compiled))).0;
    let moved = kernels::eval_one(c.kernel_id(), &v, &c.consts(&sk)).0;
    assert!(want != moved, "the two spans have to disagree for this to be a test");

    let row = sys.row_of(id).unwrap();
    let z = sys.z0(&sk);
    let got = sys.residuals(&z);
    for i in 0..want.len() {
        assert!((got[row + i] - want[i]).abs() < 1e-12,
                "refreshed constants came from the parameter's new span, not the compiled one");
    }
}
