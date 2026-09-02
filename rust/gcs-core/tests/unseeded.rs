//! Where a point nobody seeded starts (`program::scatter`).
//!
//! Issue #43.6 and #43.7: `point a` / `point b` / `a distance(30) b` is the shape of the first
//! document anybody writes, and with both points at the origin its one residual is at a
//! stationary point — the solver reported a conflict on a figure with an obvious answer.  A
//! `point tip` had the same start and, taking no `hint(…)` clause, no way out of it.  Now
//! a declared point with no clause and no place starts where a minted child does, and the
//! declaring form of a port takes the clause every other declaration takes.

use gcs_core::edit;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::parse;

fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    let e = elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}",
        e.errors().map(|d| (d.code.as_str(), d.message.clone())).collect::<Vec<_>>()
    );
    e
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

#[test]
fn two_unseeded_points_do_not_start_on_top_of_each_other() {
    let e = read("point a\npoint b\na distance(30) b\nground a\n");
    let (a, b) = (e.sketch.point_xy(0), e.sketch.point_xy(1));
    assert!(dist(a, b) > 0.5, "b starts apart from a: {a:?} {b:?}");
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{r:?}");
    assert!((dist(sk.point_xy(0), sk.point_xy(1)) - 30.0).abs() < 1e-6);
}

#[test]
fn a_port_with_no_seed_starts_off_the_origin_and_solves() {
    let e = read(
        "component Hook(len: Length) {
           point tip
           point base hint(x: 0, y: 0)
           base distance(len) tip
         }
         h: Hook(len: 30)
         ground h.tip",
    );
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{r:?}");
    assert!((dist(sk.point_xy(0), sk.point_xy(1)) - 30.0).abs() < 1e-6);
}

#[test]
fn a_port_takes_a_hint_clause() {
    let e = read(
        "component Hook(len: Length) {
           point tip hint(x: 3, y: 4)
           point base hint(x: 0, y: 0)
           base distance(len) tip
         }
         h: Hook(len: 30)",
    );
    let tip = e.map.ent_named("h.tip").expect("the port is named");
    assert_eq!(e.sketch.point_xy(tip.i()), (3.0, 4.0), "seeded where the clause says");

    // an expression over the component's parameters, as any seed may be
    let e = read(
        "component Hook(len: Length) {
           point tip hint(x: len, y: 0)
           point base hint(x: 0, y: 0)
           base distance(len) tip
         }
         h: Hook(len: 30)",
    );
    let tip = e.map.ent_named("h.tip").unwrap();
    assert_eq!(e.sketch.point_xy(tip.i()), (30.0, 0.0));

    // and a key the kind has no scalar for is refused as it is on a declaration
    let (_, errs) = parse("component H() { point tip hint(z: 1) }\n");
    assert!(errs.iter().any(|x| x.message.contains("no scalar `z`")), "{errs:?}");
}

/// A solve moves the scattered point, and the source it came from has no clause to record the
/// pose in — so one is written, exactly as an omitted radius is (`edit::commit_seeds`), and the
/// written document reads back to the same drawing.
#[test]
fn an_unseeded_point_gets_its_pose_written_back() {
    let e = read("point a\npoint b\na distance(30) b\nground a\n");
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let edit = edit::commit_seeds(&e, &sk, &e.program);
    assert!(edit.text.contains("point a hint(x: "), "{}", edit.text);
    assert!(edit.text.contains("point b hint(x: "), "{}", edit.text);
    let back = read(&edit.text);
    assert_eq!(back.sketch.point_xy(1), sk.point_xy(1));
    let (prog, _) = parse(&edit.text);
    assert_eq!(prog.text(), edit.text);
}

/// The scatter is a function of creation order and nothing else, so the same document starts
/// the same way on every run — and a minted child still starts apart from its siblings.
#[test]
fn a_minted_line_still_has_length() {
    let a = read("line l\n");
    let b = read("line l\n");
    assert_eq!(a.sketch.point_xy(0), b.sketch.point_xy(0));
    assert_eq!(a.sketch.point_xy(1), b.sketch.point_xy(1));
    assert!(dist(a.sketch.point_xy(0), a.sketch.point_xy(1)) > 0.5);
}
