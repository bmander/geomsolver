//! **An orientation the document states is held** (§9.6, #157): `ccw(a, b, c)` and `cw(a, b, c)`
//! add no row, as a bound adds none, and are read on the solution as one is — the seeds carried
//! to the side stated, a solve landing on the other root steered back by reflect and re-solve,
//! and one no root keeps failed and named.  What the document states is its own record
//! (`Sketch::orientations`), apart from the root choices a plan's replay writes back
//! (`Sketch::branches`), which are no statement and hold nothing.
use gcs_core::decompose::PlanSolver;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::newton::Method;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts, BOUND_BROKEN};
use gcs_core::system::System;

use crate::common::{ent, read, text::solved};

/// A point on two circles about `a` and `b`, five from each: a root above the line through them
/// and a root below, seeded below.
const CROSSING: &str = "unit mm\nuse std\nin std.front {\n  a := point hint((0, 0))\n  \
                        b := point hint((6, 0))\n  p := point hint((3, -2))\n}\nfix((0, 0)) a\n\
                        fix((6, 0)) b\np distance(5) a\np distance(5) b\n";

/// Where `p` stands in its plane.
fn p_at(e: &Elaborated) -> (f64, f64) {
    e.sketch.point_xy(ent(e, "p").i())
}

/// Unstated, the seed picks the root below; stated, the solve comes to the root the statement
/// names, whichever way round its points are written.
#[test]
fn an_orientation_chooses_the_root_the_seed_did_not() {
    let below = solved(CROSSING);
    assert!((p_at(&below).1 + 4.0).abs() < 1e-9, "{:?}", p_at(&below));
    let stated = [("ccw(a, b, p)", 4.0), ("ccw(b, p, a)", 4.0), ("cw(a, p, b)", 4.0),
                  ("cw(a, b, p)", -4.0), ("ccw(b, a, p)", -4.0)];
    for (stated, want) in stated {
        let e = solved(&format!("{CROSSING}{stated}\n"));
        let (x, y) = p_at(&e);
        assert!((x - 3.0).abs() < 1e-9 && (y - want).abs() < 1e-9, "{stated}: {:?}", (x, y));
    }
}

/// It is read on the seeds first: a point seeded on the wrong side is carried across the line
/// of the other two, as if it had been seeded there, and a seed read from it follows.
#[test]
fn an_orientation_chooses_the_seeds_side() {
    let src = format!("{CROSSING}ccw(a, b, p)\nin std.front {{\n  q := point hint(at: p)\n}}\n\
                       q distance(1) p\n");
    let e = read(&src);
    let (p, q) = (p_at(&e), e.sketch.point_xy(ent(&e, "q").i()));
    assert!((p.0 - 3.0).abs() < 1e-9 && (p.1 - 2.0).abs() < 1e-9, "carried to its mirror: {p:?}");
    assert!((q.1 - p.1).abs() < 1e-9, "and what is seeded from it with it: {q:?}");
    // a seed on the side stated is where it was written
    let e = read(&format!("{CROSSING}cw(a, b, p)\n"));
    assert_eq!(p_at(&e), (3.0, -2.0));
}

/// And on the solution: a pose put on the wrong side after the seeds were read is steered back,
/// reflected and solved again — and a drag, which keeps the branch it is on, is not.
#[test]
fn a_solve_is_steered_where_the_seeds_were_not() {
    let below = |e: &mut Elaborated| {
        let own = e.sketch.own_params(ent(e, "p"));
        e.sketch.params[own[1] as usize].value = -2.0;
    };
    let mut e = read(&format!("{CROSSING}ccw(a, b, p)\n"));
    below(&mut e);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success && (p_at(&e).1 - 4.0).abs() < 1e-9, "{r:?} {:?}", p_at(&e));
    let mut e = read(&format!("{CROSSING}ccw(a, b, p)\n"));
    below(&mut e);
    let r = solve(&mut e.sketch, SolveOpts { retry: false, ..SolveOpts::default() });
    assert!(r.success && (p_at(&e).1 + 4.0).abs() < 1e-9, "{r:?} {:?}", p_at(&e));
}

/// An orientation adds no row, and a drawing whose seeds already keep it solves to the same
/// bits as the drawing without it.
#[test]
fn an_orientation_adds_no_row() {
    let above = CROSSING.replace("hint((3, -2))", "hint((3, 2))");
    let plain = read(&above);
    let stated = read(&format!("{above}ccw(a, b, p)\n"));
    let count = |e: &Elaborated| {
        let sys = System::new(&e.sketch);
        (sys.hard_rows().len(), sys.n_free)
    };
    assert_eq!(count(&plain), count(&stated));
    let (mut a, mut b) = (plain, stated);
    solve(&mut a.sketch, SolveOpts::default());
    solve(&mut b.sketch, SolveOpts::default());
    let bits =
        |e: &Elaborated| e.sketch.params.iter().map(|p| p.value.to_bits()).collect::<Vec<_>>();
    assert_eq!(bits(&a), bits(&b));
}

/// One no root keeps fails the solve — a solver may not report a solution violating it — and the
/// diagnosis names it, in the words it is written in.
#[test]
fn an_orientation_no_root_keeps_fails_the_solve() {
    let mut e = read(&format!("{CROSSING}fix((3, -4)) p\nccw(a, b, p)\n"));
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(!r.success && r.status == BOUND_BROKEN, "{r:?}");
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!(d.orientations_violated.len(), 1, "{d:?}");
    let k = &d.orientations_violated[0];
    let v = e.sketch.orientations[k];
    let said = gcs_core::report::orientation_text(k, v, &|r| e.map.name_of(r).cloned());
    assert_eq!(said, "ccw(a, b, p)");
    let e = solved(&format!("{CROSSING}ccw(a, b, p)\n"));
    let mut sk = e.sketch;
    assert!(diagnose(&mut sk, DiagnoseOptions::default()).orientations_violated.is_empty());
}

/// **Stated apart from recorded** (#157): the statement is its own record, and the plan's replay
/// — which writes the root it took back into `branches` — neither makes one nor overrides one.
/// A plan that is not sticky replays a stated triangle as stated, whatever side the pose is on.
#[test]
fn the_plan_replays_what_is_stated_and_records_nothing_stated() {
    let mut e = read(&format!("{CROSSING}ccw(a, b, p)\n"));
    assert_eq!(e.sketch.orientations.len(), 1);
    assert_eq!(e.sketch.orientations, e.sketch.branches);
    let own = e.sketch.own_params(ent(&e, "p"));
    e.sketch.params[own[1] as usize].value = -2.0;
    let mut ps = PlanSolver::new(&e.sketch, false);
    let r = ps.solve(&mut e.sketch, 1e-9, true, Method::DogLeg);
    assert!(r.success && (p_at(&e).1 - 4.0).abs() < 1e-9, "{r:?} {:?}", p_at(&e));
    assert_eq!(e.sketch.orientations, e.sketch.branches, "the replay wrote back what is stated");
    // unstated, the replay records the root it took and nothing holds it
    let mut e = read(CROSSING);
    let mut ps = PlanSolver::new(&e.sketch, false);
    assert!(ps.solve(&mut e.sketch, 1e-9, true, Method::DogLeg).success);
    assert!(e.sketch.orientations.is_empty() && !e.sketch.branches.is_empty(), "{:?}",
            e.sketch.branches);
}

/// The document keeps what it states: saved and loaded, lifted to source and read back, grafted.
#[test]
fn an_orientation_is_kept() {
    let e = read(&format!("{CROSSING}cw(a, p, b)\n"));
    assert_eq!(e.sketch.orientations.len(), 1);
    let back = gcs_core::io::loads(&gcs_core::io::dumps(&e.sketch, None)).expect("loads");
    assert_eq!(back.orientations, e.sketch.orientations);
    // a document stating none says nothing of them
    assert!(!gcs_core::io::dumps(&read(CROSSING).sketch, None).contains("orientations"));
    let mut p = gcs_core::program::to_program(&e.sketch);
    let text = gcs_core::syntax::render_flat(&mut p).unwrap().to_string();
    let again = gcs_core::program::elaborate(&crate::common::parse_legacy(&text).0);
    assert_eq!(again.sketch.orientations, e.sketch.orientations, "{text}");
    let copied = gcs_core::graft::copy(&e.sketch, &e.sketch.primitives());
    assert_eq!(copied.orientations, e.sketch.orientations);
}
