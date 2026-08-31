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
        let edges = gcs_core::fixtures::henneberg_edges(n, &mut rng);
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

/// DogLeg's first full Gauss–Newton step can overshoot clean out of the solution's basin and
/// stall in a residual minimum that solves nothing — this rectangle, carrying its redundant
/// perpendiculars and asked to widen from 7.6 to 25, used to do exactly that on raw residuals,
/// and the one-shot solve retried the same start on the damped path.  With every row in its
/// own units DogLeg takes the trap itself, so the pinned failure has retired as its comment
/// said it would; what is held is that the figure solves, whichever path takes it.
#[test]
fn a_rectangle_widened_past_its_gauss_newton_step_solves() {
    let build = || {
        let mut sk = Sketch::new();
        let a = sk.point(167.7, 4.83, false, "a");
        let b = sk.point(175.3, 4.83, false, "b");
        let c = sk.point(175.3, -35.2, false, "c");
        let d = sk.point(167.7, -35.2, false, "d");
        let (top, right, bottom, left) =
            (sk.line(a, b), sk.line(b, c), sk.line(c, d), sk.line(d, a));
        for (l1, l2) in [(top, right), (right, bottom), (bottom, left)] {
            sk.add(Constraint::two_line(CKind::Perpendicular, EntRef::line(l1), EntRef::line(l2)));
        }
        for l in [left, right] {
            sk.add(Constraint::one_line(CKind::Vertical, EntRef::line(l)));
        }
        for l in [bottom, top] {
            sk.add(Constraint::one_line(CKind::Horizontal, EntRef::line(l)));
        }
        sk.add(Constraint::distance(EntRef::point(b), EntRef::point(c), 40.0));
        sk.add(Constraint::distance(EntRef::point(a), EntRef::point(b), 25.0));
        sk
    };
    let res = solve(&mut build(), SolveOpts::default());
    assert!(res.success, "{}: max residual {}", res.message, res.max_residual);
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

/// The trust-region loop is written once and used twice: for the sketch's compiled `System` and
/// for the tiny rigid-motion systems a cluster merge produces.  This exercises it directly on a
/// third — Rosenbrock as a two-residual least-squares problem, whose curved valley is exactly what
/// a globalised method is for (plain Gauss–Newton from here wanders).
#[test]
fn the_shared_dogleg_loop_solves_a_system_of_its_own() {
    use gcs_core::linalg::Mat;
    use gcs_core::newton::{dogleg, Tol, TrustRegion};

    struct Rosenbrock {
        j: Mat,
    }
    // r = (10(y - x²), 1 - x)
    impl TrustRegion for Rosenbrock {
        fn n(&self) -> usize {
            2
        }
        fn m(&self) -> usize {
            2
        }
        fn residuals_into(&mut self, z: &[f64], out: &mut [f64]) {
            out[0] = 10.0 * (z[1] - z[0] * z[0]);
            out[1] = 1.0 - z[0];
        }
        fn jacobian_at(&mut self, z: &[f64]) {
            self.j = Mat::from_vec(2, 2, vec![-20.0 * z[0], 10.0, -1.0, 0.0]);
        }
        fn jt_mul(&mut self, v: &[f64], out: &mut [f64]) {
            out.copy_from_slice(&self.j.mul_t_vec(v));
        }
        fn j_mul(&mut self, v: &[f64], out: &mut [f64]) {
            out.copy_from_slice(&self.j.mul_vec(v));
        }
        fn gn_step(&mut self, r: &[f64], _g: &[f64], p: &mut [f64]) {
            let neg: Vec<f64> = r.iter().map(|v| -v).collect();
            let b = Mat::from_vec(2, 1, neg);
            let (x, _) = gcs_core::linalg::min_norm_lstsq(&self.j, &b, 1e-12);
            p.copy_from_slice(&x.data);
        }
    }

    let mut t = Rosenbrock { j: Mat::zeros(0, 0) };
    let mut z = vec![-1.2, 1.0];
    let mut r = vec![0.0; 2];
    t.residuals_into(&z, &mut r);
    let info = dogleg(&mut t, &mut z, &mut r, Tol { ftol: 1e-12, xtol: 1e-14, gtol: 1e-16 },
                      200, 800);
    assert_eq!(info.status, 0, "{info:?}");
    assert!((z[0] - 1.0).abs() < 1e-6 && (z[1] - 1.0).abs() < 1e-6, "{z:?}");
}

#[test]
fn the_conditioned_jacobian_is_dimensionless() {
    // each row of `conditioned` is the raw row over `extent^(degree - 1)`: a degree-1 row is a
    // unit-free gradient already, a degree-2 row carries one power of length, a degree-0 row (an
    // angle, whose residual is radians) carries one fewer and is multiplied by the extent.  Every
    // row then sits within a couple of orders of 1 — which is what lets one absolute tolerance
    // judge them all, and what a kernel declaring the wrong `degree` would break.
    //
    // The band is a claim about *well-proportioned* drawings, not a law: a row's units come from
    // the drawing's extent while its gradient comes from the lengths that row actually reads, so
    // any row leaves the band once those two part company — a degree-0 angle on a corner far
    // smaller than the drawing on the high side, a degree-2 distance in the same pose on the low.
    // (`locus::assemble` takes a row's units from the lengths the row itself reads, which is the
    // sharper rule; this one is the cheap global.)  No document below carries an `angle`, so the
    // degree-0 case is not exercised here — `an_angle_is_directed` is where that kernel is held.
    use gcs_core::kernels::KERNELS;
    for name in examples::EXAMPLES {
        let sk = examples::example(name).unwrap();
        let mut sys = System::new(&sk);
        let z = sys.z0(&sk);
        let raw = sys.jacobian_dense(&z);
        let hard = sys.hard_rows();
        let c = sys.conditioned(&z);
        let (_, row_c) = sys.structure();
        assert_eq!((c.rows(), c.cols()), (hard.len(), sys.n_free), "{name}");
        let extent = sk.extent().max(1.0);
        for (i, &r) in hard.iter().enumerate() {
            let degree = KERNELS[sk.constraint(row_c[i]).unwrap().kernel_id()].degree;
            let unit = extent.powi(degree as i32 - 1);
            let mut norm = 0.0f64;
            for j in 0..sys.n_free {
                // `jacobian_dense` hands a row out over `row_scale` (`extent^degree`) already
                let want = raw.at(r, j) * sys.row_scale[r] / unit;
                assert!((c.as_mat().at(i, j) - want).abs() <= 1e-12 * want.abs().max(1.0));
                norm += want * want;
            }
            let norm = norm.sqrt();
            assert!((1e-3..=1e2).contains(&norm), "{name} row {i}: |row| = {norm}");
        }
    }
}

/// An `angle` is *directed*: stated mod half a turn, as it once was, the opposite side satisfies
/// the statement just as exactly, and the solve settles for whichever is nearer (issue #11).
///
/// The datum is grounded along the page's x-axis and the swinging point is held ten from its
/// pivot, so a bearing of ±90° names one place and no other — and the pose starts on the side the
/// half-turn reading would have been content with.
#[test]
fn an_angle_is_directed() {
    let pose = |theta: f64, start: (f64, f64)| {
        let mut sk = Sketch::new();
        let o = sk.point(0.0, 0.0, true, "o");
        let a = sk.point(10.0, 0.0, true, "a");
        let p = sk.point(start.0, start.1, false, "p");
        let datum = sk.line(o, a);
        let swing = sk.line(o, p);
        sk.add(Constraint::distance(EntRef::point(o), EntRef::point(p), 10.0));
        sk.add(Constraint::new(
            CKind::Angle,
            vec![
                Arg::Ent(EntRef::line(datum)),
                Arg::Ent(EntRef::line(swing)),
                Arg::Num(theta),
            ],
        ));
        let r = solve(&mut sk, SolveOpts::default());
        assert!(r.success, "theta {theta}: {r:?}");
        sk.point_xy(p)
    };
    let at = |got: (f64, f64), want: (f64, f64)| {
        assert!((got.0 - want.0).abs() < 1e-6 && (got.1 - want.1).abs() < 1e-6, "{got:?}");
    };
    // +90° posed below comes all the way round to straight up; −90° posed above, the mirror
    at(pose(std::f64::consts::FRAC_PI_2, (-3.0, -10.0)), (0.0, 10.0));
    at(pose(-std::f64::consts::FRAC_PI_2, (-3.0, 10.0)), (0.0, -10.0));
}
