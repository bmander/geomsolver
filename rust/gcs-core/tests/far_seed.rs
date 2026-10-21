//! **A point far out stretches no row's measure** (#492).
//!
//! `unsettled.rs`'s belt — two circles, their two external tangents, and a point stated on both —
//! with the point seeded a kilometre and ten kilometres out.  Every row used to be judged over
//! the drawing's extent, which that one point makes 1e6 mm: a contact a micrometre off its 20 mm
//! circle read as 5e-14, a DogLeg stopped on its gradient after one step, and the belt over equal
//! circles, which has no solution, was *solved* with its claim a theorem.  A row is now judged
//! over its own size (`system::Sizes`, `System::row_size`), and a failed solve whose rows hold
//! only by the extent is solved on at its rows' own sizes (`System::at_own_sizes`).  The gate
//! for "solved" in `docs/contract/solver.md`.
use gcs_core::boundary::measure::{self, Drive};
use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts, SolveResult};
use gcs_core::system::System;

use crate::common::{at, ent, read};
use crate::unsettled::belt;

fn solved(rb: f64, seed: f64) -> (Elaborated, SolveResult) {
    let mut e = read(&belt(rb, Some(seed)));
    let r = solve(&mut e.sketch, SolveOpts::default());
    (e, r)
}

fn claimed(e: &Elaborated) -> u32 {
    e.sketch.constraints.iter().find(|c| c.claim).expect("the claim").id
}

/// The largest hard row over the drawing's extent, as the iteration reads it, and over its own
/// size, as "solved" does.
fn measures(sk: &Sketch) -> (f64, f64) {
    let mut sys = System::new(sk);
    let z = sys.z0(sk);
    let r = sys.residuals(&z);
    let by_extent = sys.hard_rows().iter().map(|&i| r[i].abs()).fold(0.0, f64::max);
    (by_extent, sys.max_relative_residual(&z))
}

/// Equal circles: the runs are parallel and no point is on both.  Seeded far out, the pose the
/// solve stops at holds every row by the extent's measure — and not by the rows' own sizes,
/// so it is no answer, and no claim is judged on it.
#[test]
fn equal_circles_seeded_far_out_do_not_solve_and_judge_no_claim() {
    for seed in [-1e6, -1e7] {
        let (mut e, r) = solved(20.0, seed);
        assert!(!r.settled() && r.outcome() != "solved", "seeded at {seed}: {r:?}");
        let (by_extent, own) = measures(&e.sketch);
        assert!(by_extent < 1e-6 && own > 1e-6, "seeded at {seed}: {by_extent:e} {own:e}");
        let opts = DiagnoseOptions { settled: r.settled(), ..Default::default() };
        let d = diagnose(&mut e.sketch, opts);
        let c = claimed(&e);
        assert!(
            !d.claims_theorem.contains(&c) && !d.claims_consuming.contains(&c),
            "seeded at {seed}: a claim judged on a pose that is no answer: {:?}",
            d.claims_theorem
        );
    }
}

/// Unequal by a micrometre: the runs cross at the external centre of similitude,
/// `−r_a·|ab|/(r_b − r_a)` = −2.4e6 from `a`.  From either seed the solve stops on the extent's
/// measure where `x` was seeded, and goes on at the rows' own sizes to the crossing.
#[test]
fn circles_a_micrometre_apart_solve_where_their_runs_cross() {
    for seed in [-1e6, -1e7] {
        let (e, r) = solved(20.001, seed);
        assert!(r.settled() && r.outcome() == "solved", "seeded at {seed}: {r:?}");
        let x = at(&e, "x");
        let within = 1e-6 * e.sketch.extent();
        assert!((x[0] + 2.4e6).abs() < within && x[1].abs() < within, "seeded at {seed}: {x:?}");
    }
}

/// A belt with a solution, seeded ten kilometres out: solved where its runs meet, at −96.
#[test]
fn a_belt_seeded_far_out_solves_where_its_runs_meet() {
    let (e, r) = solved(45.0, -1e7);
    assert!(r.settled(), "{r:?}");
    let x = at(&e, "x");
    assert!((x[0] + 96.0).abs() < 1e-9 && x[1].abs() < 1e-9, "{x:?}");
}

/// What each row is judged at where the solve seeded a kilometre out stops: a contact on a
/// circle at the circle's size, and `x` on a run at the run's — the far point left out of the
/// box round what it reads (`system::FAR_OUT`) — never at the extent the point stretches.
#[test]
fn a_row_is_judged_at_the_size_of_what_it_reads() {
    let (e, _) = solved(20.0, -1e6);
    let sys = System::new(&e.sketch);
    assert!(sys.extent >= 1e6);
    let (mut contacts, mut on_runs) = (0, 0);
    for row in sys.hard_rows() {
        let (b, i) = sys.instance_of(row);
        let c = e.sketch.constraint(sys.blocks[b].cids[i]).expect("a constraint");
        let size = sys.row_size[row];
        match c.kind {
            CKind::PointOnCircle => {
                contacts += 1;
                assert!((size - 20.0).abs() < 0.1, "{:?} at {size}", c.kind);
            }
            CKind::PointOnLine => {
                on_runs += 1;
                assert!(size > 100.0 && size < 200.0, "{:?} at {size}", c.kind);
            }
            _ => assert!(size < 1e3, "{:?} at {size}", c.kind),
        }
    }
    assert_eq!((contacts, on_runs), (4, 2));
}

/// `bench boundaries`' sweep of the radii to equal (#459), each solve started where the last
/// ended: the flat solve is solved nowhere the runs do not cross, and where it is solved, at
/// the crossing.  At r₂ = 20.001 (a crossing 2.4 km out) it holds every row to 2e-7 of its own
/// size and stands 2e-3 of the extent off the crossing, whose place moves 2.4e9 times what a
/// contact is off its circle: no residual measure tells that pose from the crossing, so it is
/// not held to it here (docs/iteration-limit-rescue-plan.md, #492).
#[test]
fn radii_swept_equal_solve_only_where_the_runs_cross() {
    let mut last: Option<Sketch> = None;
    for rb in [45.0, 30.0, 25.0, 22.0, 21.0, 20.5, 20.1, 20.01, 20.001, 20.0001, 20.0] {
        let doc = measure::belts(&[Drive { r_large: rb, ..Drive::PAPER }], false);
        let e = measure::read(&doc.src).expect("reads");
        let mut sk = e.sketch.clone();
        if let Some(s) = &last {
            measure::carry(s, &mut sk);
        }
        let f = measure::flat(&mut sk, SolveOpts::default());
        let x = sk.world_point(ent(&e, "meet_0.x").i());
        if rb <= 20.0001 {
            assert!(!f.result.success, "r₂ = {rb}: {:?} at {x:?}", f.result);
        } else if rb != 20.001 {
            let crossing = -20.0 * 120.0 / (rb - 20.0);
            assert!(f.result.settled(), "r₂ = {rb}: {:?}", f.result);
            assert!((x[0] - crossing).abs() < 1e-6 * sk.extent(), "r₂ = {rb}: {x:?}");
        }
        last = Some(sk);
    }
}
