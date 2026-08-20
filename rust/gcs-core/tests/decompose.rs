use gcs_core::cgraph::{build, El};
use gcs_core::constraints::CKind;
use gcs_core::decompose::PlanSolver;
use gcs_core::examples;
use gcs_core::newton::Method;

#[test]
fn the_graph_maps_the_examples() {
    let g = build(&examples::rect_fillets(100.0, 60.0, 10.0, 0.0));
    assert_eq!(g.n_points(), 12);
    assert_eq!(g.lines.len(), 4);
    assert!(g.unsupported.is_empty());
    assert_eq!(g.virtuals.len(), 8); // one radius line per arc-endpoint tangency
    assert_eq!(g.dirs.len(), 4 + 8); // H/V + tangency perpendiculars

    let g = build(&examples::truss(8, 20.0, 15.0, true));
    assert_eq!((g.passive.len(), g.lines.len()), (30, 1)); // only the horizontal member

    let g = build(&examples::polygon_chain(6, 50.0));
    assert_eq!(g.unsupported.len(), 6); // EqualLength is not an F–H constraint
}

#[test]
fn examples_fully_decompose_and_replay_exactly() {
    for name in ["rect_fillets", "slotted_link", "truss"] {
        let mut sk = examples::example(name).unwrap();
        let mut ps = PlanSolver::new(&sk, false);
        assert!(ps.plan.fully_decomposed(), "{name}: {}", ps.plan.summary());
        for seed in 0..3u32 {
            sk.perturb(2.0, seed);
            let r = ps.solve(&mut sk, 1e-9, false, Method::DogLeg);
            assert!(r.success && !r.fell_back && r.max_residual < 1e-8, "{name}: {r:?}");
        }
    }
    // constraint values are read live: change a dimension, replay without recompiling
    let mut sk = examples::example("rect_fillets").unwrap();
    let mut ps = PlanSolver::new(&sk, false);
    let id = sk
        .constraints
        .iter()
        .find(|c| c.kind == CKind::Distance && (c.args[2].num() - 80.0).abs() < 1e-9)
        .unwrap()
        .id;
    sk.constraint_mut(id).unwrap().set_num("d", 120.0);
    let r = ps.solve(&mut sk, 1e-9, false, Method::DogLeg);
    assert!(r.success, "{r:?}");
    let max_x = (0..sk.points.len()).map(|i| sk.point_xy(i).0).fold(f64::MIN, f64::max);
    assert!((max_x - 140.0).abs() < 1e-6, "{max_x}");
}

#[test]
fn unsupported_constraints_fall_back_to_numeric() {
    let mut sk = examples::polygon_chain(8, 50.0);
    let mut ps = PlanSolver::new(&sk, false);
    assert!(!ps.plan.fully_decomposed());
    sk.perturb(2.0, 0);
    let r = ps.solve(&mut sk, 1e-9, true, Method::DogLeg);
    assert!(r.success && r.fell_back, "{r:?}");
}

#[test]
fn chirality_flags_follow_the_current_geometry() {
    use gcs_core::constraints::Constraint;
    use gcs_core::model::{EntRef, Sketch};
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "a");
    let b = sk.point(10.0, 0.0, true, "b");
    let c = sk.point(5.0, 4.0, false, "c");
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(c), 6.0));
    sk.add(Constraint::distance(EntRef::point(b), EntRef::point(c), 6.0));
    let mut ps = PlanSolver::new(&sk, false);
    assert!(ps.plan.fully_decomposed());
    let r = ps.solve(&mut sk, 1e-9, false, Method::DogLeg);
    assert!(r.success && sk.point_xy(c).1 > 0.0);
    let up: Vec<i32> =
        ps.plan.steps.iter().filter(|s| s.ppp.is_some()).map(|s| s.branch.unwrap()).collect();
    let cy = sk.points[c].y as usize;
    sk.params[cy].value = -4.0; // flip the sketch to the other root
    let r = ps.solve(&mut sk, 1e-9, false, Method::DogLeg);
    assert!(r.success && sk.point_xy(c).1 < 0.0);
    let down: Vec<i32> =
        ps.plan.steps.iter().filter(|s| s.ppp.is_some()).map(|s| s.branch.unwrap()).collect();
    assert!(!up.is_empty() && down == up.iter().map(|s| -s).collect::<Vec<_>>());
    // sticky branches: the recorded root wins even if the sketch moved
    ps.plan.sticky_branches = true;
    sk.params[cy].value = 4.0;
    ps.solve(&mut sk, 1e-9, false, Method::DogLeg);
    assert!(sk.point_xy(c).1 < 0.0);
}

#[test]
fn k33_needs_a_core_and_decomposes() {
    let mut sk = examples::k33(3);
    let mut ps = PlanSolver::new(&sk, false);
    assert!(ps.plan.fully_decomposed(), "{}", ps.plan.summary());
    assert!(ps.plan.steps.iter().map(|s| s.ids.len()).max().unwrap() >= 4); // a core merge
    sk.perturb(1.0, 1);
    let r = ps.solve(&mut sk, 1e-9, false, Method::DogLeg);
    assert!(r.success && r.max_residual < 1e-8, "{r:?}");
}

#[test]
fn laman_frameworks_decompose_fully() {
    for seed in 0..8u32 {
        let mut sk = examples::laman(6 + (seed as usize % 7), 500 + seed, true);
        let mut ps = PlanSolver::new(&sk, false);
        assert!(ps.plan.fully_decomposed(), "seed {seed}: {}", ps.plan.summary());
        sk.perturb(1.0, seed);
        let r = ps.solve(&mut sk, 1e-9, false, Method::DogLeg);
        assert!(r.success && r.max_residual < 1e-8, "seed {seed}: {r:?}");
    }
}

#[test]
fn the_plan_and_the_numeric_path_agree() {
    use gcs_core::solve::{solve, SolveOpts};
    for name in examples::EXAMPLES {
        for seed in 0..3u32 {
            let mut a = examples::example(name).unwrap();
            let mut b = examples::example(name).unwrap();
            a.perturb(1.0, seed);
            b.perturb(1.0, seed);
            let ra = PlanSolver::new(&a, false).solve(&mut a, 1e-9, true, Method::DogLeg);
            let rb = solve(&mut b, SolveOpts::default());
            assert!(ra.success && rb.success, "{name}");
            if name != "polygon_chain" {
                for (x, y) in a.get_x().iter().zip(b.get_x()) {
                    assert!((x - y).abs() < 1e-5, "{name}: {x} vs {y}");
                }
            }
        }
    }
}

#[test]
fn flipping_a_construction_survives_later_solves() {
    use gcs_core::constraints::Constraint;
    use gcs_core::model::{EntRef, Sketch};
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "a");
    let b = sk.point(10.0, 0.0, true, "b");
    let c = sk.point(5.0, 4.0, false, "c");
    sk.add(Constraint::distance(EntRef::point(a), EntRef::point(c), 6.0));
    sk.add(Constraint::distance(EntRef::point(b), EntRef::point(c), 6.0));
    let mut ps = PlanSolver::new(&sk, true);
    ps.solve(&mut sk, 1e-9, true, Method::DogLeg);
    assert!(sk.point_xy(c).1 > 0.0);
    let el = El::p(ps.plan.graph.point_of[c]);
    assert_eq!(ps.flip(&mut sk, el), 1);
    ps.solve(&mut sk, 1e-9, true, Method::DogLeg);
    assert!(sk.point_xy(c).1 < 0.0 && !sk.branches.is_empty());
    for _ in 0..3 {
        ps.solve(&mut sk, 1e-9, true, Method::DogLeg);
        assert!(sk.point_xy(c).1 < 0.0);
    }
}
