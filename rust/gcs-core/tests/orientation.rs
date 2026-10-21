//! **A stated orientation chooses the root** (§9.6, #157): `ccw(a, b, c)` / `cw(a, b, c)` written
//! in a document adds no row, and a document's solve holds the drawing to it as it holds it to a
//! bound — the seeds' side chosen at elaboration, the solution checked and steered by carrying a
//! point of the triangle across the line through the other two, and a solve no root of which
//! keeps it failing (`ORIENTATION_BROKEN`).  What the document states is recorded apart from what
//! the plan records (`Sketch::orientations` beside `Sketch::branches`), and only it is enforced.
use gcs_core::decompose::{branch_record, PlanSolver};
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::newton::Method;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts, ORIENTATION_BROKEN};

use crate::common::{ent, read};

/// The review's two-root point (docs/solvent-direction.md, N2): `x` five from `a` and from `b`,
/// six apart, a root above the line through them and one below, seeded at `seed`, the orientation
/// `turn` stated of it (or none).
fn apex(seed: (f64, f64), turn: &str) -> String {
    format!("\
unit mm
use std
in std.front {{
  a := point
  b := point
  fix((0, 0)) a
  fix((6, 0)) b
  x := point hint(({}, {}))
  x distance(5mm) a
  x distance(5mm) b
  {turn}
}}
", seed.0, seed.1)
}

/// Where `x` stands in its plane.
fn x_at(e: &Elaborated) -> (f64, f64) {
    e.sketch.point_xy(ent(e, "x").i())
}

fn close(at: (f64, f64), want: (f64, f64)) -> bool {
    (at.0 - want.0).abs() < 1e-9 && (at.1 - want.1).abs() < 1e-9
}

/// `src` read and solved by the document's solve, which must succeed.
fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{r:?}\n{src}");
    e
}

/// The case the issue was opened on: from a seed below the line, or one nearer the root below,
/// `ccw(a, b, x)` comes to the root above, and `cw` to the one below from seeds above.
#[test]
fn a_stated_orientation_chooses_the_root_the_seed_did_not() {
    for seed in [(3.0, -4.0), (1.0, -1.0)] {
        let plain = solved(&apex(seed, ""));
        assert!(close(x_at(&plain), (3.0, -4.0)), "unstated, the seed chooses: {:?}", x_at(&plain));
        let e = solved(&apex(seed, "ccw(a, b, x)"));
        assert!(close(x_at(&e), (3.0, 4.0)), "ccw from {seed:?}: {:?}", x_at(&e));
        // the same triangle named the other way round is the same choice
        let e = solved(&apex(seed, "ccw(b, x, a)"));
        assert!(close(x_at(&e), (3.0, 4.0)), "ccw(b, x, a) from {seed:?}: {:?}", x_at(&e));
        let e = solved(&apex(seed, "cw(a, b, x)"));
        assert!(close(x_at(&e), (3.0, -4.0)), "cw from {seed:?}: {:?}", x_at(&e));
    }
    for seed in [(3.0, 4.0), (1.0, 1.0)] {
        let e = solved(&apex(seed, "cw(a, b, x)"));
        assert!(close(x_at(&e), (3.0, -4.0)), "cw from {seed:?}: {:?}", x_at(&e));
    }
}

/// It is read on the seeds first, as a bound is: a point seeded on the other side is carried
/// across the line through the other two before anything is solved, and a seed read from it
/// follows.
#[test]
fn a_stated_orientation_chooses_the_seeds_side() {
    let e = read(&apex((1.0, -1.0), "ccw(a, b, x)"));
    assert!(close(x_at(&e), (1.0, 1.0)), "carried to the mirror of its seed: {:?}", x_at(&e));
    let src = apex((1.0, -1.0), "ccw(a, b, x)\n  q := point hint(at: x)\n  q distance(1mm) x");
    let e = read(&src);
    let q = e.sketch.point_xy(ent(&e, "q").i());
    assert!(close(q, (1.0, 1.0)), "and what is seeded from it with it: {q:?}");
}

/// And on the solution: a pose put on the other side after the seeds were read is steered back,
/// reflected and solved again — and a drag, which keeps the branch it is on, is not.
#[test]
fn a_solve_is_steered_where_the_seeds_were_not() {
    let below = |e: &mut Elaborated| {
        let own = e.sketch.own_params(ent(e, "x"));
        e.sketch.params[own[1] as usize].value = -2.0;
    };
    let mut e = read(&apex((3.0, 4.0), "ccw(a, b, x)"));
    below(&mut e);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success && close(x_at(&e), (3.0, 4.0)), "{r:?} {:?}", x_at(&e));
    let mut e = read(&apex((3.0, 4.0), "ccw(a, b, x)"));
    below(&mut e);
    let r = solve(&mut e.sketch, SolveOpts { retry: false, ..SolveOpts::default() });
    assert!(r.success && close(x_at(&e), (3.0, -4.0)), "{r:?} {:?}", x_at(&e));
}

/// An orientation adds no row, and one the drawing keeps changes nothing: the same bits as the
/// drawing that does not state it.
#[test]
fn an_orientation_kept_solves_to_the_same_bits() {
    let plain = solved(&apex((3.0, 4.0), ""));
    let stated = solved(&apex((3.0, 4.0), "ccw(a, b, x)"));
    let bits = |e: &Elaborated| e.sketch.params.iter().map(|p| p.value.to_bits()).collect::<Vec<_>>();
    assert_eq!(bits(&plain), bits(&stated));
    assert!(plain.sketch.orientations.is_empty());
    assert_eq!(stated.sketch.orientations.len(), 1);
    let (mut a, mut b) = (plain, stated);
    assert_eq!(diagnose(&mut a.sketch, DiagnoseOptions::default()).dof,
               diagnose(&mut b.sketch, DiagnoseOptions::default()).dof);
}

/// No root on the stated side: the drawing puts `x` on the line through `a` and `b`, where the
/// triangle turns neither way.  The solve does not say solved, and the diagnosis names the
/// orientation, in the document's words.
#[test]
fn an_orientation_no_root_keeps_fails_the_solve() {
    let src = "\
unit mm
use std
in std.front {
  a := point
  b := point
  fix((0, 0)) a
  fix((6, 0)) b
  l := line(a, b)
  x := point hint((2, -1))
  x coincident l
  x distance(2mm) a
  ccw(a, b, x)
}
";
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(!r.success && r.status == ORIENTATION_BROKEN, "{r:?}");
    assert_eq!(r.message, "a stated orientation does not hold");
    assert!(r.max_residual < 1e-9, "every row holds: {r:?}");
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!(d.orientations_violated.len(), 1, "{d:?}");
    let name = |r| e.map.name_of(r).cloned();
    let said = gcs_core::report::orientation_text(&e.sketch, &d.orientations_violated[0], &name);
    assert_eq!(said.as_deref(), Some("ccw(a, b, x)"));
    // stated the way the drawing allows, nothing is broken
    let mut e = read(&src.replace("x coincident l\n", ""));
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{r:?}");
    assert!(diagnose(&mut e.sketch, DiagnoseOptions::default()).orientations_violated.is_empty());
}

/// **A recorded choice is not enforced.**  `Sketch::branches` is also the plan's cache, where a
/// replay writes the turn its seeds had: the document's solve never reads it, so a drawing
/// whose cache says otherwise solves where its seeds lead, as it always did.
#[test]
fn a_recorded_choice_is_not_enforced() {
    let mut e = read(&apex((3.0, -4.0), ""));
    let pts = ["a", "b", "x"].map(|n| ent(&e, n).i());
    let (key, v) = branch_record(pts, true);
    e.sketch.branches.insert(key, v);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success && close(x_at(&e), (3.0, -4.0)), "{r:?} {:?}", x_at(&e));
}

/// The plan replays a stated choice as stated, whatever its cache says and whatever turn its
/// seeds had, and writes the stated one back to the cache.
#[test]
fn the_plan_replays_what_is_stated() {
    for sticky in [false, true] {
        let mut e = read(&apex((3.0, 4.0), "ccw(a, b, x)"));
        let own = e.sketch.own_params(ent(&e, "x"));
        e.sketch.params[own[1] as usize].value = -4.0;
        let pts = ["a", "b", "x"].map(|n| ent(&e, n).i());
        let (key, v) = branch_record(pts, true);
        e.sketch.branches.insert(key.clone(), -v);
        let mut ps = PlanSolver::new(&e.sketch, sticky);
        let r = ps.solve(&mut e.sketch, 1e-9, true, Method::DogLeg);
        assert!(r.success && close(x_at(&e), (3.0, 4.0)), "sticky {sticky}: {r:?} {:?}", x_at(&e));
        assert_eq!(e.sketch.branches.get(&key), Some(&v), "sticky {sticky}");
    }
}

/// What the document states travels with it: saved and read back, and copied with its points.
#[test]
fn a_stated_orientation_is_saved_and_grafted() {
    let e = read(&apex((3.0, 4.0), "ccw(a, b, x)"));
    let back = gcs_core::io::from_json(&gcs_core::io::to_json(&e.sketch)).expect("reads back");
    assert_eq!(back.orientations, e.sketch.orientations);
    let all = e.sketch.primitives();
    let copied = gcs_core::graft::copy(&e.sketch, &all);
    assert_eq!(copied.orientations.len(), 1, "{:?}", copied.orientations);
    let mut sk = copied;
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{r:?}");
    assert!(gcs_core::solve::orientations_hold(&sk));
}

/// **A traced body is as it was** (§6.5.1): its `ccw`s are enforced at its anchor by the locus
/// (reflect and solve again) and recorded of its drawn instance, which stands where they hold —
/// so the linkages that state them solve to the same bits whether the sheet holds them or not.
#[test]
fn a_traced_bodys_drawing_is_unchanged() {
    for (name, mut sk) in [("jansen", gcs_core::examples::jansen()),
                           ("peaucellier", gcs_core::examples::peaucellier(100.0, 60.0, 40.0))] {
        assert!(!sk.orientations.is_empty(), "{name} states its orientations");
        let mut free = sk.clone();
        free.orientations.clear();
        let r = solve(&mut sk, SolveOpts::default());
        let f = solve(&mut free, SolveOpts::default());
        assert!(r.success && f.success, "{name}: {r:?} {f:?}");
        assert!(gcs_core::solve::orientations_hold(&sk), "{name}");
        let bits = |sk: &gcs_core::model::Sketch| {
            sk.params.iter().map(|p| p.value.to_bits()).collect::<Vec<_>>()
        };
        assert_eq!(bits(&sk), bits(&free), "{name}");
    }
}
