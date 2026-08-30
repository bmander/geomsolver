//! A block body may end mid-joint: the chain threads to the next copy (issue #38).
//!
//! `cycle N { distance(d) line -> angle(a) }` is N sides, each welded to the next at a corner
//! that may also state relations, the last wrapping to the first — the polygon, with no
//! `close`, no names and no written points.  What is worth testing is the two halves of the
//! bargain: the parser records the open joint on the block (and refuses it everywhere it can
//! mean nothing), and the flattener states it per pair of copies — the weld a shared point,
//! never a coincidence, and every worded corner the regular At-form an in-chain joint gets.

use gcs_core::constraints::CKind;
use gcs_core::edit::{self, Kind};
use gcs_core::model::EntRef;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::{parse, Chained, StmtKind};

fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    let e = elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}",
        e.errors().map(|d| d.message.clone()).collect::<Vec<_>>()
    );
    e
}

fn refuses(src: &str, needle: &str) {
    let (_, errs) = parse(src);
    let msgs: Vec<String> = errs.into_iter().map(|e| e.message).collect();
    assert!(msgs.iter().any(|m| m.contains(needle)), "expected `{needle}`\n{src}\n{msgs:?}");
}

fn kinds(sk: &gcs_core::model::Sketch, k: CKind) -> usize {
    sk.user_constraints().iter().filter(|c| c.kind == k).count()
}

/// The polygon: one link and one open joint make N sides welded into a loop — N lines over
/// exactly N points, head to tail round the cycle, and the drawing solves.
#[test]
fn a_cycle_of_one_link_is_a_polygon() {
    let e = read("cycle 4 {\n  distance(50) line -> angle(90)\n}\n");
    let mut sk = e.sketch;
    assert_eq!((sk.lines.len(), sk.points.len()), (4, 4), "welds, not coincident pairs");
    assert_eq!((kinds(&sk, CKind::Distance), kinds(&sk, CKind::Angle)), (4, 4));
    // head to tail: each side leaves at the point the next arrives by, the last at the first's
    for i in 0..4 {
        assert_eq!(sk.lines[i].p2, sk.lines[(i + 1) % 4].p1, "side {i} is not welded on");
    }
    let r = gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "the polygon does not solve");
}

/// `repeat` does not wrap, so the last copy's trailing joint is simply not stated: an open
/// polyline of N sides — N+1 points, N−1 corners — rather than an error.
#[test]
fn a_repeat_ends_open() {
    let e = read("repeat 4 {\n  distance(50) line -> angle(90)\n}\n");
    let mut sk = e.sketch;
    assert_eq!((sk.lines.len(), sk.points.len()), (4, 5), "an open polyline");
    assert_eq!(kinds(&sk, CKind::Distance), 4, "the prefix is each copy's own");
    assert_eq!(kinds(&sk, CKind::Angle), 3, "N-1 corners: the last is not stated");
    for i in 0..3 {
        assert_eq!(sk.lines[i].p2, sk.lines[i + 1].p1, "side {i} is not welded on");
    }
    let r = gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
    assert!(r.success);
    // one copy is no pairs at all, not an error
    let e = read("repeat 1 {\n  line -> angle(90)\n}\n");
    assert_eq!((e.sketch.lines.len(), e.sketch.points.len()), (1, 2));
    assert_eq!(kinds(&e.sketch, CKind::Angle), 0);
}

/// `ring` closes exactly as `cycle` does.
#[test]
fn a_ring_wraps_like_a_cycle() {
    let e = read("ring 4 {\n  distance(50) line -> angle(90)\n}\n");
    assert_eq!((e.sketch.lines.len(), e.sketch.points.len()), (4, 4));
    assert_eq!(kinds(&e.sketch, CKind::Angle), 4);
}

/// A worded open joint whose word is a tangency gets the regular At-form across the copy seam
/// — never the bare pair that is rank-deficient at every solution.
#[test]
fn a_tangent_corner_is_the_at_form() {
    let e = read(
        "cycle 2 {\n  point p hint(x: 0, y: 0)\n  point c hint(x: 10, y: 10)\n  \
         line a(p) -> tangent arc k(center: c) hint(r: 10) -> tangent\n}\n",
    );
    let sk = e.sketch;
    let tangents: Vec<String> = sk
        .user_constraints()
        .iter()
        .filter(|c| c.kind == CKind::TangentArcLine)
        .map(|c| gcs_core::io::describe(c))
        .collect();
    assert_eq!(tangents.len(), 4, "two in-chain corners and two across the seam");
    assert!(
        tangents.iter().all(|t| t.contains("tangent(at:")),
        "a corner tangency must name its end: {tangents:?}"
    );
}

/// A joint may state several relations, and each is stated per pair of copies.
#[test]
fn a_joint_states_several_relations_per_pair() {
    let e = read("cycle 4 {\n  line s -> perpendicular equal\n}\n");
    assert_eq!(kinds(&e.sketch, CKind::Perpendicular), 4);
    assert_eq!(kinds(&e.sketch, CKind::EqualLength), 4);
}

/// A boundary slot the body declares names the shared point, so the weld runs through the
/// declared points and mints nothing: N sides over the N points the body wrote.
#[test]
fn a_declared_entry_is_the_shared_point() {
    let e = read("cycle 3 {\n  point p hint(x: 0, y: 0)\n  line s(p) ->\n}\n");
    let sk = e.sketch;
    assert_eq!((sk.lines.len(), sk.points.len()), (3, 3), "the weld mints no point");
    for i in 0..3 {
        assert_eq!(sk.lines[i].p2, sk.lines[(i + 1) % 3].p1);
    }
}

/// How the joint's statements are spelled is recorded: one word steps down to the corner
/// (`Joint`), several are members of one written joint (`Member`).
#[test]
fn the_spelling_is_recorded() {
    let (prog, errs) = parse("cycle 4 {\n  line s -> perpendicular equal\n}\n");
    assert!(errs.is_empty(), "{errs:?}");
    let spellings: Vec<Chained> = prog
        .stmts()
        .filter(|s| matches!(s.kind, StmtKind::Relation(_)))
        .map(|s| s.chained)
        .collect();
    assert_eq!(spellings.len(), 2, "two words, one statement each");
    assert!(
        spellings.iter().all(|c| matches!(c, Chained::Member { out_of: 2, .. })),
        "{spellings:?}"
    );
    let (prog, _) = parse("cycle 4 {\n  line s -> angle(90)\n}\n");
    let spellings: Vec<Chained> = prog
        .stmts()
        .filter(|s| matches!(s.kind, StmtKind::Relation(_)))
        .map(|s| s.chained)
        .collect();
    assert_eq!(spellings, vec![Chained::Joint]);
}

/// Every entity and constraint the expansion makes names a statement the source has — the
/// joint's statements included, which is `Program::stmt` walking into `Block::joint`.
#[test]
fn every_stated_corner_names_its_statement() {
    let src = gcs_core::examples::source("square").expect("the shipped case");
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    assert!(e.ok());
    for (r, site) in e.map.of_entity.iter() {
        assert!(prog.stmt(site.stmt).is_some(), "{r:?} names a statement no source has");
    }
    for (id, site) in e.map.of_constraint.iter() {
        assert!(prog.stmt(site.stmt).is_some(), "constraint {id} names a statement no source has");
    }
}

/// The corner's statements are one written joint however many copies state it, so a gesture on
/// one stated corner cannot splice the block's line: removing the constraint is no edit, and
/// removing a welded side is refused with its reason.
#[test]
fn removing_one_stated_corner_edits_nothing() {
    let src = gcs_core::examples::source("square").expect("the shipped case");
    let e = read(src);
    let equal = e
        .sketch
        .user_constraints()
        .iter()
        .find(|c| c.kind == CKind::EqualLength)
        .expect("a stated corner")
        .id;
    let out = edit::remove(&e, &e.program, &[], &[equal]);
    assert_eq!(out.kind, Kind::None, "one line, four corners: nothing to splice");
    assert_eq!(out.text, src);
    let out = edit::remove(&e, &e.program, &[EntRef::line(0)], &[]);
    assert!(out.refused.is_some(), "deleting one copy's side went through");
}

/// A pose moved inside the block stays unwritten: one statement, N poses, no seed to record —
/// the bargain every expanded statement strikes, unchanged by the joint.
#[test]
fn a_seed_inside_the_block_stays_unwritten() {
    let src = "cycle 4 {\n  distance(50) line -> angle(90)\n}\n";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty());
    let mut e = elaborate(&prog);
    assert!(e.ok());
    for i in 0..e.sketch.points.len() {
        let [x, y] = e.sketch.point_params(i);
        e.sketch.params[x as usize].value = 3.0 * i as f64;
        e.sketch.params[y as usize].value = 7.0 * i as f64;
    }
    let out = edit::commit_seeds(&e, &e.sketch, &prog);
    assert_eq!(out.kind, Kind::None, "four poses, one statement, nothing to record");
    assert_eq!(out.text, src);
}

/// Where the open joint can mean nothing it is refused, each refusal its own message.
#[test]
fn the_refusals() {
    // an unthreaded trailing word still wants its right operand
    refuses("cycle 2 {\n  line a angle(30)\n}\n", "expected an element");
    // the top level has no next copy, and keeps the error it had
    refuses("line ->\n", "expected an element");
    // neither does a component
    refuses(
        "component C() {\n  line ->\n}\nc: C()\n",
        "a chain ends mid-joint only in a `repeat`, `cycle` or `ring`",
    );
    // a name-link's boundary is elaboration's to read (issue #35), so the body must declare
    refuses(
        "point q hint(x: 0, y: 0)\npoint w hint(x: 9, y: 0)\nline s(q, w)\ncycle 2 {\n  s -> angle(30)\n}\n",
        "declared elsewhere",
    );
    // both boundary slots declared name two different points across the seam
    refuses(
        "cycle 2 {\n  point a hint(x: 0, y: 0)\n  point b hint(x: 9, y: 0)\n  line l(a, b) ->\n}\n",
        "the joint names two points",
    );
    // a circle has no ends, at an open joint as at any other
    refuses("cycle 2 {\n  circle c hint(r: 5) ->\n}\n", "has no ends to thread");
}

/// One copy welds the line to itself, which build refuses as the self-reference it is —
/// an error with a cause, never a panic.
#[test]
fn a_cycle_of_one_copy_is_refused_at_build() {
    let (prog, errs) = parse("cycle 1 {\n  line ->\n}\n");
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    assert!(!e.ok(), "a line welded to itself elaborated");
}
