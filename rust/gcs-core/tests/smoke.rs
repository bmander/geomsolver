use gcs_core::constraints::{Arg, CKind, Constraint};
use gcs_core::examples;
use gcs_core::model::{EntRef, Sketch};
use gcs_core::newton::Method;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::system::System;

#[test]
fn examples_solve_from_a_perturbed_start() {
    for name in examples::EXAMPLES {
        for method in [Method::DogLeg, Method::Lm] {
            let mut sk = examples::example(name).unwrap();
            sk.perturb(2.0, 0);
            let r = solve(&mut sk, SolveOpts { method, ..SolveOpts::default() });
            assert!(r.success, "{name} {:?}: {r:?}", method);
            assert!(r.max_residual < 1e-8, "{name}: {r:?}");
        }
    }
}

#[test]
fn under_constrained_moves_minimally() {
    use gcs_core::constraints::Constraint;
    use gcs_core::model::{EntRef, Sketch};
    let mut sk = Sketch::new();
    let p = sk.point(0.0, 0.0, false, "p");
    let q = sk.point(12.0, 0.0, false, "q");
    sk.add(Constraint::distance(EntRef::point(p), EntRef::point(q), 10.0));
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success);
    assert!((sk.point_xy(p).0 - 1.0).abs() < 1e-6, "{:?}", sk.point_xy(p));
    assert!((sk.point_xy(q).0 - 11.0).abs() < 1e-6);
}

#[test]
fn sparse_and_dense_paths_agree() {
    let mut a = examples::truss(30, 20.0, 15.0, true);
    let mut b = examples::truss(30, 20.0, 15.0, true);
    a.perturb(1.0, 3);
    b.perturb(1.0, 3);
    let ra = System::new(&a).solve(&mut a, SolveOpts { dense: Some(true), ..SolveOpts::default() });
    let rb = System::new(&b).solve(&mut b, SolveOpts { dense: Some(false), ..SolveOpts::default() });
    assert!(ra.success && rb.success, "{ra:?} {rb:?}");
    for (x, y) in a.get_x().iter().zip(b.get_x()) {
        assert!((x - y).abs() < 1e-6);
    }
}

#[test]
fn rank_is_reported_on_the_dense_path() {
    let mut sk = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    sk.perturb(1.0, 0);
    let n_res = sk.n_residuals();
    let r = solve(&mut sk, SolveOpts::default());
    assert_eq!(r.rank, Some(n_res as i32));
    let mut sys = System::new(&sk);
    let z = sys.z0(&sk);
    assert_eq!(sys.rank(&z, 1e-10, false), n_res);
}

#[test]
fn graph_algorithms_agree_with_the_reference_cases() {
    use gcs_core::graph;
    let m = graph::hopcroft_karp(&[vec![0, 1], vec![1, 2], vec![2, 0]], 3).0;
    assert_eq!(m.iter().filter(|&&x| x >= 0).count(), 3);
    let m = graph::hopcroft_karp(&[vec![0], vec![0], vec![0]], 1).0;
    assert_eq!(m.iter().filter(|&&x| x >= 0).count(), 1);

    let dm = graph::dulmage_mendelsohn(&[vec![0], vec![0], vec![1, 2]], 3);
    assert_eq!(dm.over_rows, vec![0, 1]);
    assert_eq!(dm.over_cols, vec![0]);
    assert_eq!(dm.under_rows, vec![2]);
    assert_eq!(dm.under_cols, vec![1, 2]);
    assert_eq!((dm.n_redundant, dm.n_free, dm.rank), (1, 1, 2));

    assert!(graph::pebble_game(3, &[(0, 1), (1, 2), (2, 0)]).is_rigid());
    assert_eq!(graph::pebble_game(4, &[(0, 1), (1, 2), (2, 3), (3, 0)]).dof, 1);
    let k4 = graph::pebble_game(4, &[(0, 1), (1, 2), (2, 3), (3, 0), (0, 2), (1, 3)]);
    assert!(k4.is_rigid() && k4.redundant == vec![5]);
    let bow = graph::pebble_game(5, &[(0, 1), (1, 2), (2, 0), (2, 3), (3, 4), (4, 2)]);
    assert_eq!(bow.dof, 1);
    assert_eq!(bow.components, vec![vec![0, 1, 2], vec![2, 3, 4]]);
}

#[test]
fn pebble_game_recognises_laman_graphs() {
    use gcs_core::graph;
    use gcs_core::rng::Rng;
    for seed in 0..6u32 {
        let mut rng = Rng::new(seed + 1);
        let n = 4 + rng.int(11);
        let edges = gcs_core::examples::henneberg_edges(n, &mut rng);
        assert_eq!(edges.len(), 2 * n - 3);
        let res = graph::pebble_game(n, &edges);
        assert!(res.is_rigid() && res.redundant.is_empty());
        assert_eq!(res.components, vec![(0..n).collect::<Vec<_>>()]);
        assert_eq!(graph::pebble_game(n, &edges[1..]).dof, 1);
    }
}

/// A line whose endpoints have collapsed has no direction.  Dividing by its length put a NaN in
/// the residual vector, and every max we take skipped it — the solver stopped on iteration zero
/// and called an unsatisfiable sketch solved.
#[test]
fn a_degenerate_line_does_not_read_as_solved() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, true, "a");
    let b = sk.point(0.0, 0.0, true, "b"); // coincident with a: the line has no direction
    let p = sk.point(1.0, 1.0, true, "p");
    let ln = sk.line(a, b);
    let c = sk.add(Constraint::new(
        CKind::PointLineDistance,
        vec![Arg::Ent(EntRef::point(p)), Arg::Ent(EntRef::line(ln)), Arg::Num(3.0)],
    ));

    let mut sys = System::new(&sk);
    let z = sys.z0(&sk);
    let r = sys.residuals(&z).to_vec();
    assert!(r.iter().all(|v| v.is_finite()), "residuals went non-finite: {r:?}");
    assert!(sys.max_hard_residual(&z) > 1.0, "the unmet 3-unit offset vanished");
    let errs = sys.constraint_errors(&z);
    let i = sys.cids.iter().position(|&id| id == c).unwrap();
    assert!(errs[i] > 1.0, "constraint_errors reported {}", errs[i]);

    let res = solve(&mut sk, SolveOpts::default());
    assert!(!res.success, "an unsatisfiable sketch reported solved: {res:?}");
}

/// Defence in depth for the same thing: a NaN anywhere in a residual vector has to win the max,
/// not be skipped by `x > m`.
#[test]
fn a_nan_residual_wins_the_max() {
    assert!(gcs_core::linalg::absmax(&[1.0, f64::NAN, 2.0]).is_nan());
    assert_eq!(gcs_core::linalg::absmax(&[1.0, -3.0, 2.0]), 3.0);
}

/// `gr_svd` gives up after a fixed number of QR sweeps.  Its `false` return used to be discarded,
/// so a non-finite Jacobian came back as rank 0 with a full null space — read everywhere above as
/// "every constraint is redundant and every parameter is free".
#[test]
fn a_non_convergent_svd_is_reported_rather_than_read_as_rank_zero() {
    use gcs_core::linalg::{rank_and_nullspace, svd, Mat};

    let good = Mat::from_vec(2, 2, vec![1.0, 0.0, 0.0, 1.0]);
    let rn = rank_and_nullspace(&good, 1e-10);
    assert!(rn.converged && rn.rank == 2);
    assert!(svd(&good, false).converged);

    let bad = Mat::from_vec(2, 2, vec![1.0, f64::NAN, 0.0, 1.0]);
    let rn = rank_and_nullspace(&bad, 1e-10);
    assert!(!rn.converged, "a NaN matrix was reported as a converged rank {}", rn.rank);
    assert!(!svd(&bad, false).converged);
}
