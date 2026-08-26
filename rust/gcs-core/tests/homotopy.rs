use gcs_core::constraints::Constraint;
use gcs_core::decompose::PlanSolver;
use gcs_core::examples;
use gcs_core::homotopy::{apply_alternative, enumerate_step, EnumerateOptions};
use gcs_core::model::{EntRef, Sketch};
use gcs_core::newton::Method;

#[test]
fn a_triangle_has_two_roots_and_the_other_can_be_applied() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "a");
    let b = sk.point(10.0, 0.0, true, "b");
    let c = sk.point(5.0, 4.0, false, "c");
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(c), 6.0));
    sk.add(Constraint::distance(EntRef::point(b), EntRef::point(c), 6.0));
    let mut ps = PlanSolver::new(&sk, true);
    ps.solve(&mut sk, 1e-9, true, Method::DogLeg);
    let idx = ps.plan.steps.iter().position(|s| s.ppp.is_some()).unwrap();
    let alts = enumerate_step(&mut ps.plan, &mut sk, idx, EnumerateOptions::default());
    assert_eq!(alts.len(), 2);
    assert!(alts[0].is_current() && !alts[1].is_current());
    let other = alts[1].clone();
    apply_alternative(&mut ps.plan, &mut sk, idx, &other);
    assert!(sk.point_xy(c).1 < 0.0);
    ps.solve(&mut sk, 1e-9, true, Method::DogLeg);
    assert!(sk.point_xy(c).1 < 0.0);
    for k in sk.hard_constraints() {
        assert!(k.error(&sk) < 1e-6);
    }
}

#[test]
fn under_determined_merges_are_skipped() {
    let mut sk = examples::rect_fillets_under();
    let mut ps = PlanSolver::new(&sk, true);
    ps.solve(&mut sk, 1e-9, true, Method::DogLeg);
    for i in 0..ps.plan.steps.len() {
        // whatever the step, enumeration must return a list and never panic
        let _ = enumerate_step(
            &mut ps.plan,
            &mut sk,
            i,
            EnumerateOptions { max_paths: 16, ..Default::default() },
        );
    }
}

#[test]
#[ignore = "~20 s (256 homotopy paths)"]
fn the_k33_core_has_several_real_realizations() {
    let mut sk = examples::k33();
    let mut ps = PlanSolver::new(&sk, true);
    ps.solve(&mut sk, 1e-9, true, Method::DogLeg);
    let idx = (0..ps.plan.steps.len()).max_by_key(|&i| ps.plan.steps[i].ids.len()).unwrap();
    assert_eq!(ps.plan.steps[idx].ids.len(), 9);
    let alts = enumerate_step(&mut ps.plan, &mut sk, idx, EnumerateOptions::default());
    assert!(alts.len() >= 2 && alts.iter().any(|a| a.is_current()));
    let x0 = sk.get_x();
    let other = alts.iter().find(|a| !a.is_current()).unwrap().clone();
    apply_alternative(&mut ps.plan, &mut sk, idx, &other);
    let r = ps.solve(&mut sk, 1e-9, false, Method::DogLeg);
    assert!(r.success);
    assert!(sk.get_x().iter().zip(&x0).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max) > 1.0);
}
