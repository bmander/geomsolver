//! What #459's benchmark found, held at sizes a test can afford (`boundary::measure`, `bench
//! boundaries`): the counts that do not depend on the machine, for #454's prototype against the
//! flat solve.  The timings are the bench's; the results table is `docs/solve-boundaries-plan.md`
//! §18.
use gcs_core::boundary::measure::{self, Document, Drive};
use gcs_core::model::Sketch;
use gcs_core::solve::{solve_through, Boundaries, SolveOpts, Through};
use gcs_core::system::{System, DENSE_MAX, RANK_TOL};

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
/// stays a run's four unknowns, and the parent through the runs has eight unknowns a drive.
#[test]
fn identical_drives_cost_n_times_one() {
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

/// The flat document's rank at `sk`'s pose, as the diagnosis reads it.
fn flat_rank(sk: &Sketch) -> usize {
    let mut sys = System::new(sk);
    let z = sys.z0(sk);
    sys.rank(&z, RANK_TOL, true)
}

/// The parent's Jacobian through the boundaries is sparse: a drive's rows read only its own
/// drive's unknowns, so its entries grow as N, not N², and past `DENSE_MAX` unknowns the parent
/// steps on its normal equations, as the flat solve does (the dense parent was #459's cost).
/// Stepped either way it comes to one pose, and its rank, found part by part, is what is left of
/// the flat rank when the boundaries' own unknowns are taken out (FL3).
#[test]
fn the_parent_is_sparse_and_steps_as_the_dense_one_does() {
    let (_, one, _) = both(&measure::belts(&[Drive::PAPER], false), 1e-9);
    let n = 20;
    let doc = measure::belts(&vec![Drive::PAPER; n], false);
    let (_, t, sparse) = both(&doc, 1e-7);
    assert!(t.parent_unknowns > DENSE_MAX, "{}", t.parent_unknowns);
    assert_eq!(t.parent_entries, n * one.parent_entries);
    assert_eq!(flat_rank(&sparse), t.boundary_unknowns + t.reduced_rank);
    let e = measure::read(&doc.src).unwrap();
    let bs = measure::bounds(&e, &doc.boundaries).unwrap();
    let mut dense = e.sketch.clone();
    let d = solve_through(&mut dense, &bs, SolveOpts { dense: Some(true), ..SolveOpts::default() })
        .unwrap();
    assert!(d.result.success, "{}", d.result.message);
    assert_eq!((d.result.iterations, d.reduced_rank), (t.result.iterations, t.reduced_rank));
    for (a, b) in sparse.params.iter().zip(&dense.params) {
        assert!((a.value - b.value).abs() <= 1e-9 * (1.0 + b.value.abs()),
                "`{}`: {} sparse, {} dense", b.name, a.value, b.value);
    }
}

/// An unknown every drive reads joins the whole parent into one part, over `DENSE_MAX` columns:
/// its rank is found with that column held out, each drive's part factored on its own and the
/// held column's rank added for what no part spans, and it is still the flat rank less the
/// boundaries' own unknowns (FL3).
#[test]
fn a_shared_unknown_is_held_out_of_the_rank_and_the_rank_adds_up() {
    let n = 20;
    let drives: Vec<Drive> =
        (0..n).map(|k| Drive { r_large: 40.0 + 10.0 * (k as f64 + 0.5) / n as f64, ..Drive::PAPER })
            .collect();
    let (_, t, got) = both(&measure::belts(&drives, true), 1e-7);
    assert!(t.parent_unknowns > DENSE_MAX, "{}", t.parent_unknowns);
    assert_eq!(flat_rank(&got), t.boundary_unknowns + t.reduced_rank);
}

/// A solve through boundaries says where its time went (`Through::times`), phase by phase: the
/// phases do not overlap, so they and what is left over add up to the whole, and the inner
/// solves, the most a cache of evaluations (#456) could save, are one of them.
#[test]
fn a_solve_through_says_where_its_time_went() {
    let (_, t, _) = both(&measure::belts(&[Drive::PAPER; 3], false), 1e-9);
    let s = t.times;
    let phases = [s.compile, s.setup, s.inner, s.rows, s.jacobian, s.step, s.rank];
    assert!(phases.iter().all(|&p| p >= 0.0), "{s:?}");
    assert!(s.other() >= -1e-9 * s.total, "the phases overlap: {s:?}");
    assert!(s.compile > 0.0 && s.inner > 0.0 && s.total >= t.result.time_s, "{s:?}");
}
