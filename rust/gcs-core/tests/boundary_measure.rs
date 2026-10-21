//! What #459's benchmark found, held at sizes a test can afford (`boundary::measure`, `bench
//! boundaries`): the counts that do not depend on the machine, for #454's prototype against the
//! flat solve.  The timings are the bench's; the results table is `docs/solve-boundaries-plan.md`
//! §18.
use gcs_core::boundary::measure::{self, Document, Drive};
use gcs_core::model::Sketch;
use gcs_core::solve::{solve_through, Boundaries, SolveOpts, Through};

/// `doc` solved flat and through its boundaries from the same start, the two poses held to agree
/// to `tol`: both solved, and each solve's answer.
fn both(doc: &Document, tol: f64) -> (measure::Flat, Through, Sketch) {
    let e = measure::read(&doc.src).unwrap_or_else(|why| panic!("{why}\n{}", doc.src));
    let bs = measure::bounds(&e, &doc.boundaries).unwrap();
    let mut flat = e.sketch.clone();
    let f = measure::flat(&mut flat, SolveOpts::default());
    assert!(f.result.success, "flat: {}", f.result.message);
    let mut through = e.sketch.clone();
    let t = solve_through(&mut through, &bs, SolveOpts::default()).unwrap();
    assert!(t.result.success, "through: {}", t.result.message);
    for (a, b) in through.params.iter().zip(&flat.params) {
        assert!((a.value - b.value).abs() <= tol * (1.0 + b.value.abs()),
                "`{}`: {} through, {} flat", b.name, a.value, b.value);
    }
    (f, t, through)
}

fn solves(t: &Through) -> usize {
    t.evaluations.iter().map(|e| e.1).sum()
}

/// N identical drives cost N times one drive, inner solve for inner solve: nothing is shared
/// between instances, though every copy is asked the same question at every step (an exact
/// placement-independent cache would answer all but one, #456).  The flat solve's largest block
/// stays a run's four unknowns, while the parent through the runs is one dense matrix that grows
/// with N: eight unknowns a drive.
#[test]
fn identical_drives_cost_n_times_one_and_the_parent_is_dense() {
    let (f1, one, _) = both(&measure::belts(&[Drive::PAPER], false), 1e-9);
    let (f3, three, _) = both(&measure::belts(&[Drive::PAPER; 3], false), 1e-9);
    assert_eq!(three.result.iterations, one.result.iterations);
    assert_eq!(solves(&three), 3 * solves(&one));
    assert_eq!(three.inner_iterations, 3 * one.inner_iterations);
    let per_run: Vec<usize> = three.evaluations.iter().map(|e| e.1).collect();
    assert!(per_run.iter().all(|&n| n == one.evaluations[0].1), "{per_run:?}");
    assert_eq!((f1.largest_block, f3.largest_block), (4, 4));
    assert_eq!(three.largest_boundary, 4);
    assert_eq!((one.parent_unknowns, three.parent_unknowns), (8, 24));
}

/// Distinct drives sharing one unknown centre distance: every step of the parent moves every
/// drive's inputs, so every run is solved again at every residual the parent asks for, and asked
/// again for the Jacobian there, answered from the last.  No two runs are asked the same
/// question, so no cache could answer one.
#[test]
fn a_shared_unknown_solves_every_child_at_every_step() {
    let drives: Vec<Drive> =
        (0..3).map(|k| Drive { r_large: 40.0 + 10.0 * (k as f64 + 0.5) / 3.0, ..Drive::PAPER })
            .collect();
    let (f, t, _) = both(&measure::belts(&drives, true), 1e-8);
    for (name, n) in &t.evaluations {
        assert_eq!(*n as i32, t.result.nfev, "`{name}`");
    }
    // each Jacobian, and the pose the solve ended on, read where the last residual was
    assert_eq!(t.answered_again as i32, 6 * (t.result.njev + 1));
    // flat, the shared unknown welds the first drive's crossing to every drive's shaft
    assert!(f.largest_block > 4, "{}", f.largest_block);
}

/// A closed loop through `k` two-root points is one coupled block flat, every joint in it, and a
/// chain of 2 × 2 solves through boundaries under a parent of the crank's two unknowns, whatever
/// `k` is.
#[test]
fn a_loop_is_one_block_flat_and_a_chain_of_small_solves_through() {
    for k in [2, 8] {
        let (f, t, _) = both(&measure::dyads(k), 1e-9);
        assert_eq!((f.blocks, f.largest_block, f.unknowns), (1, 2 * k + 2, 2 * k + 2));
        assert_eq!((t.parent_unknowns, t.largest_boundary, t.boundary_unknowns), (2, 2, 2 * k));
        assert_eq!(t.evaluations.iter().map(|e| e.0.clone()).collect::<Vec<_>>(),
                   (1..=k).map(|i| format!("d{i}")).collect::<Vec<_>>(), "in the order they read");
    }
}

/// An edit to one drive of three, solved again from the pose the solve had reached: the prototype
/// keeps nothing between solves, so every run is solved afresh at least once, and the parent's
/// dense step moves the drives the edit did not touch by their residual's last bits.  What
/// per-output invalidation (#457) would save is every run but the edited drive's two.
#[test]
fn an_edit_to_one_drive_solves_every_run_again() {
    let (_, solved) = measure::solved(&measure::belts(&[Drive::PAPER; 3], false).src).unwrap();
    let mut drives = [Drive::PAPER; 3];
    drives[0].centres = 125.0;
    let doc = measure::belts(&drives, false);
    let e = measure::read(&doc.src).unwrap();
    let bs = measure::bounds(&e, &doc.boundaries).unwrap();
    let mut sk = e.sketch.clone();
    measure::carry(&solved, &mut sk);
    let t = solve_through(&mut sk, &bs, SolveOpts::default()).unwrap();
    assert!(t.result.success, "{}", t.result.message);
    assert!(t.evaluations.iter().all(|e| e.1 >= 1), "{:?}", t.evaluations);
    // the untouched drives end where they were, to the inner solve's tolerance
    let listed = Boundaries::new(&sk, &bs).unwrap();
    for b in &bs[2..] {
        for p in listed.unknowns(&b.name).unwrap() {
            let (a, w) = (sk.params[p as usize].value, solved.params[p as usize].value);
            assert!((a - w).abs() <= 1e-9 * (1.0 + w.abs()), "`{}` moved: {a} from {w}", b.name);
        }
    }
}

/// Towards a fold of the boundary (a line onto the circle it cuts), its derivative grows as the
/// inverse square root of the distance to the fold, and a central difference at a fixed step
/// loses it before the implicit function theorem does.  Past the fold, flat and through both fail,
/// and through names the boundary.
#[test]
fn the_derivative_grows_towards_a_fold_and_past_it_the_boundary_is_named() {
    let at = |reach: f64, from: Option<&Sketch>| {
        let doc = measure::meet(reach);
        let e = measure::read(&doc.src).unwrap();
        let bs = measure::bounds(&e, &doc.boundaries).unwrap();
        let mut sk = e.sketch.clone();
        if let Some(s) = from {
            measure::carry(s, &mut sk);
        }
        let t = solve_through(&mut sk, &bs, SolveOpts::default()).unwrap();
        (t, sk, bs)
    };
    let (t1, near, bs) = at(39.9, None);
    assert!(t1.result.success);
    let d1 = measure::derivative(&near, &bs[0]).unwrap();
    let (t2, nearer, bs) = at(39.99, Some(&near));
    assert!(t2.result.success);
    let d2 = measure::derivative(&nearer, &bs[0]).unwrap();
    let ratio = d2.largest / d1.largest;
    assert!((ratio - 10f64.sqrt()).abs() < 0.1, "{ratio}");
    let (past, _, _) = at(40.1, Some(&nearer));
    assert!(!past.result.success);
    assert_eq!(past.failure.as_ref().map(|f| f.boundary.as_str()), Some("one"));
}
