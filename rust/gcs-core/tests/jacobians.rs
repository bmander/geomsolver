//! Every constraint's analytic Jacobian must agree with finite differences at random points.
use gcs_core::constraints::{Arg, CKind, Constraint};
use gcs_core::examples;
use gcs_core::fdcheck::{check_constraint, check_sketch};
use gcs_core::model::{EntRef, Sketch};
use gcs_core::rng::Rng;
use gcs_core::system::System;

/// A sketch carrying one of every kind of entity, plus one of every constraint type on them.
fn all_constraints(seed: u32) -> Sketch {
    let mut rng = Rng::new(seed + 1);
    let mut sk = Sketch::new();
    let r = |rng: &mut Rng| rng.uniform(-10.0, 10.0);
    let (x, y) = (r(&mut rng), r(&mut rng));
    let p = sk.point(x, y, false, "p");
    let (x, y) = (r(&mut rng), r(&mut rng));
    let q = sk.point(x, y, false, "q");
    let pt = |sk: &mut Sketch, rng: &mut Rng| {
        let (x, y) = (rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0));
        sk.point(x, y, false, "z")
    };
    let (a1, b1) = (pt(&mut sk, &mut rng), pt(&mut sk, &mut rng));
    let l1 = sk.line(a1, b1);
    let (a2, b2) = (pt(&mut sk, &mut rng), pt(&mut sk, &mut rng));
    let l2 = sk.line(a2, b2);
    let cc1 = pt(&mut sk, &mut rng);
    let c1 = sk.circle(cc1, rng.uniform(1.0, 11.0), "c1");
    let cc2 = pt(&mut sk, &mut rng);
    let c2 = sk.circle(cc2, rng.uniform(1.0, 11.0), "c2");
    let (ac, as_, ae) = (pt(&mut sk, &mut rng), pt(&mut sk, &mut rng), pt(&mut sk, &mut rng));
    let arc = sk.arc(ac, as_, ae, "a");
    // six control points: three spans, so a contact is checked on an interior span too
    let ctrl: Vec<usize> = (0..6).map(|_| pt(&mut sk, &mut rng)).collect();
    let sp = sk.spline(&ctrl).unwrap();

    let (pe, qe) = (EntRef::point(p), EntRef::point(q));
    let (le1, le2) = (EntRef::line(l1), EntRef::line(l2));
    let (ce1, ce2, ae) = (EntRef::circle(c1), EntRef::circle(c2), EntRef::arc(arc));
    let spe = EntRef::spline(sp);
    let e = |x: EntRef| Arg::Ent(x);
    let cs = vec![
        Constraint::coincident(pe, qe),
        Constraint::distance(pe, qe, 3.0),
        Constraint::new(CKind::Midpoint, vec![e(pe), e(le1)]),
        Constraint::drag_target(pe, 1.0, 2.0, 0.3),
        Constraint::one_line(CKind::Horizontal, le1),
        Constraint::one_line(CKind::Vertical, le1),
        Constraint::new(CKind::HorizontalPoints, vec![e(pe), e(qe)]),
        Constraint::new(CKind::VerticalPoints, vec![e(pe), e(qe)]),
        Constraint::two_line(CKind::Parallel, le1, le2),
        Constraint::two_line(CKind::Perpendicular, le1, le2),
        Constraint::new(CKind::Angle, vec![e(le1), e(le2), Arg::Num(0.7)]),
        Constraint::two_line(CKind::EqualLength, le1, le2),
        Constraint::new(CKind::PointOnLine, vec![e(pe), e(le1)]),
        Constraint::point_on_circle(pe, ce1, false),
        Constraint::point_on_circle(pe, ae, false),
        Constraint::radius(ce1, 2.0),
        Constraint::new(CKind::EqualRadius, vec![e(ce1), e(ae)]),
        Constraint::tangent_line_circle(&sk, le1, ce1, None),
        Constraint::tangent_line_circle(&sk, le1, ce1, Some(-1)),
        Constraint::new(CKind::TangentCircleCircle, vec![e(ce1), e(ce2), Arg::Bool(true)]),
        Constraint::new(CKind::TangentCircleCircle, vec![e(ce1), e(ce2), Arg::Bool(false)]),
        Constraint::new(CKind::TangentArcLine, vec![e(ae), e(le1), Arg::Str("start".into())]),
        Constraint::new(CKind::TangentArcLine, vec![e(ae), e(le2), Arg::Str("end".into())]),
        Constraint::new(CKind::Symmetric, vec![e(pe), e(qe), e(le1)]),
        Constraint::new(CKind::ParallelDistance, vec![e(le1), e(le2), Arg::Num(4.0)]),
        Constraint::new(CKind::PointLineDistance, vec![e(pe), e(le1), Arg::Num(4.0)]),
        Constraint::new(CKind::AnnularDistance, vec![e(ce1), e(ae), Arg::Num(1.5)]),
        Constraint::point_on_spline(&sk, pe, spe),
        Constraint::point_on_spline(&sk, qe, spe),
        Constraint::spline_tangent_line(&sk, spe, le1),
        Constraint::spline_tangent_line(&sk, spe, le2),
        Constraint::spline_curvature(&sk, spe, ce1),
        Constraint::spline_curvature(&sk, spe, ae),
    ];
    // the two intrinsic PointOnCircle constraints the arc brought with it stay in the sketch
    sk.constraints.clear();
    for c in cs {
        sk.add(c);
    }
    sk
}

#[test]
fn every_constraint_jacobian_agrees_with_finite_differences() {
    for seed in 0..5u32 {
        let sk = all_constraints(seed);
        for c in &sk.constraints {
            let err = check_constraint(&sk, c, 1e-6, 1e-7)
                .unwrap_or_else(|e| panic!("seed {seed}: {e}"));
            assert!(err.is_finite());
        }
    }
}

#[test]
fn every_constraint_type_has_a_kernel_and_all_are_covered() {
    let sk = all_constraints(0);
    let used: std::collections::BTreeSet<usize> =
        sk.constraints.iter().map(|c| c.kernel_id()).collect();
    assert_eq!(used.len(), gcs_core::kernels::N_KERNELS);
}

#[test]
fn a_param_used_twice_in_one_constraint_sums_its_contributions() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "a");
    let b = sk.point(1.0, 0.2, false, "b");
    let c = sk.point(2.0, 1.0, false, "c");
    let l1 = sk.line(a, b);
    let l2 = sk.line(b, c);
    sk.add(Constraint::two_line(CKind::Perpendicular, EntRef::line(l1), EntRef::line(l2)));
    sk.add(Constraint::two_line(CKind::EqualLength, EntRef::line(l1), EntRef::line(l2)));
    sk.add(Constraint::new(
        CKind::Angle,
        vec![Arg::Ent(EntRef::line(l1)), Arg::Ent(EntRef::line(l2)), Arg::Num(0.3)],
    ));
    check_sketch(&sk, 1e-6, 1e-6).unwrap();
}

#[test]
fn example_sketch_jacobians() {
    for name in examples::EXAMPLES {
        check_sketch(&examples::example(name).unwrap(), 1e-6, 1e-6).unwrap();
    }
}

#[test]
fn fixed_params_are_dropped_from_the_jacobian() {
    let sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let mut sys = System::new(&sk);
    assert_eq!(sys.n_free, sk.params.len() - 2);
    let z = sys.z0(&sk);
    let j = sys.jacobian_dense(&z);
    assert_eq!((j.rows, j.cols), (sk.n_residuals(), sys.n_free));
}

#[test]
fn system_blocks_cover_every_constraint_once() {
    let sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let mut s = System::new(&sk);
    assert_eq!(s.blocks.iter().map(|b| b.count).sum::<usize>(), sk.constraints.len());
    assert_eq!(s.n_res, sk.n_residuals());
    let z = s.z0(&sk);
    let r = s.residuals(&z);
    for c in &sk.constraints {
        let off = s.row_of(c.id).expect("compiled constraint has a row");
        let expect = c.residual(&sk, &c.local_values(&sk));
        for (i, v) in expect.iter().enumerate() {
            assert!((r[off + i] - v).abs() < 1e-12);
        }
    }
}
