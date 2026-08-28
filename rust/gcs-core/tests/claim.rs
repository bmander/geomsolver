//! `claim` (Solvent §9.7): a relation stated as *expected to add no rank*.
//!
//! The contract has two halves and this file holds both.  A claim never acts: the solve, the
//! status, the degrees of freedom and every diagnostic set are exactly what they would be with
//! the claim deleted — a false claim cannot bend the drawing to itself, and cannot paint the
//! sketch Over or Conflict.  And a claim is always judged: the diagnosis reads it against the
//! drawing the rest of the document made and files it as a *theorem* (holds, adds no rank),
//! *violated* (does not hold), or *consuming* (holds only by the pose — enforcing it would have
//! taken a freedom, so the claim claims too much).
//!
//! `peaucellier.sv` is the case the feature was built for, and the last section here holds the
//! diagnosis to reading its straight line as a theorem — at any numbers, since a dependency that
//! held only at the shipped ones would be a coincidence.

use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::model::Sketch;

fn drawn(src: &str) -> Sketch {
    let (prog, errs) = gcs_core::syntax::parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}",
        e.errors().map(|d| (d.code.as_str(), d.message.clone())).collect::<Vec<_>>()
    );
    let mut sk = e.sketch;
    let r = gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "does not solve: {}", r.message);
    sk
}

/// §2.2's rectangle, with room after it for one claim.
const RECT: &str = "
point p0 hint(x: 0, y: 0)
point p1 hint(x: 60, y: 0)
point p2 hint(x: 60, y: 40)
point p3 hint(x: 0, y: 40)
horizontal line bottom(p0, p1) to
vertical   line right(p1, p2) to
horizontal line top(p2, p3) to
vertical   line left(p3, p0) to close
distance(p0, p1) == 60
distance(p1, p2) == 40
ground(p0)
";

#[test]
fn a_true_claim_is_a_theorem() {
    // two levelled sides are parallel — the drawing already says it, so the claim adds no rank
    let mut sk = drawn(&format!("{RECT}claim parallel(bottom, top)\n"));
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (0, State::Well), "a claim joins no count");
    let c = sk.constraints.iter().find(|c| c.claim).unwrap().id;
    assert_eq!(d.claims_theorem, vec![c]);
    assert!(d.claims_violated.is_empty() && d.claims_consuming.is_empty());
    assert!(d.over.is_empty() && d.implied.is_empty(), "a claim is its own report");
}

#[test]
fn a_false_claim_is_violated_and_moves_nothing() {
    // the bottom is horizontal by statement; claiming it vertical is simply untrue — and must
    // neither pull the rectangle out of shape nor read as a conflict
    let mut sk = drawn(&format!("{RECT}claim vertical(bottom)\n"));
    let (x1, y1) = sk.point_xy(1);
    assert!((x1 - 60.0).abs() < 1e-9 && y1.abs() < 1e-9, "the claim pulled the drawing");
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (0, State::Well), "a false claim is not a conflict");
    let c = sk.constraints.iter().find(|c| c.claim).unwrap().id;
    assert_eq!(d.claims_violated, vec![c]);
    assert!(d.violated.is_empty() && d.conflicts.is_none());
}

#[test]
fn a_claim_the_pose_happens_to_satisfy_is_consuming() {
    // nothing holds `c` above `a` — the seed just put it there.  The claim holds at this
    // solution, but enforcing it would have taken one of the two freedoms `c` keeps, so it is
    // not a theorem and the diagnosis says which kind of not
    let mut sk = drawn(
        "
point a hint(x: 0, y: 0)
point c hint(x: 0, y: 9)
line ac(a, c)
ground(a)
claim vertical(ac)
",
    );
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (2, State::Under), "the claim consumed no freedom in fact");
    let c = sk.constraints.iter().find(|c| c.claim).unwrap().id;
    assert_eq!(d.claims_consuming, vec![c]);
    assert!(d.claims_theorem.is_empty() && d.claims_violated.is_empty());
}

#[test]
fn a_claim_may_not_own_an_unknown() {
    let src = "
point o hint(x: 0, y: 0)
circle k(center: o) hint(r: 20)
point p hint(x: 20, y: 0)
claim point_on_circle(p, k)
point q hint(x: 25, y: 8)
spline s(o, p, q, o, p, q, o)
claim point_on_spline(q, s)
";
    let (prog, errs) = gcs_core::syntax::parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    // `point_on_circle` owns nothing and is a fine claim; a spline contact carries its own
    // curve parameter, which a claim may not
    let msgs: Vec<String> = e.errors().map(|d| d.message.clone()).collect();
    assert_eq!(msgs.len(), 1, "{msgs:?}");
    assert!(msgs[0].contains("a claim may add none"), "{msgs:?}");
}

#[test]
fn a_claim_travels_like_any_flag() {
    let sk = drawn(&format!("{RECT}claim parallel(bottom, top)\n"));
    // the JSON export and back
    let json = gcs_core::io::dumps(&sk, None);
    assert!(json.contains("\"claim\""), "the flag is document state");
    let back = gcs_core::io::loads(&json).unwrap();
    assert_eq!(back.constraints.iter().filter(|c| c.claim).count(), 1);
    // the rebuild walk (deletion, copy, paste are all this walk)
    let kept = gcs_core::io::without(&sk, &[], &[]);
    assert_eq!(kept.constraints.iter().filter(|c| c.claim).count(), 1);
    // and the lifted program prints the word back
    let mut p = gcs_core::program::to_program(&back);
    let text = gcs_core::syntax::render(&mut p).to_string();
    assert!(text.contains("claim parallel("), "{text}");
    // a claim-free document dumps exactly as it always has
    let plain = drawn(RECT);
    assert!(!gcs_core::io::dumps(&plain, None).contains("\"claim\""));
}

#[test]
fn a_claim_does_not_weld_drag_parts() {
    // two separate dimensioned lines, and a claim spanning them: the claim must not make them
    // one part, or dragging either would cost both
    let sk = drawn(
        "
point a hint(x: 0, y: 0)
point b hint(x: 30, y: 0)
line ab(a, b)
horizontal(ab)
distance(a, b) == 30
ground(a)
point c hint(x: 0, y: 20)
point d hint(x: 30, y: 20)
line cd(c, d)
horizontal(cd)
distance(c, d) == 30
ground(c)
claim equal_length(ab, cd)
",
    );
    let part = gcs_core::io::Part::around(&sk, gcs_core::model::EntRef::point(1));
    assert_eq!(part.sketch.points.len(), 2, "the claim welded two figures into one part");
}

#[test]
fn a_claim_restating_a_relation_is_not_a_duplicate_of_it() {
    // `same_constraint` compares what is said, not whether it is said, so a claim matches the
    // relation it restates exactly.  Counted as a duplicate it moves that relation out of
    // `implied` and into `over` — a claim making the sketch over-constrained, which is the one
    // thing §9.7 promises cannot happen.  `altitudes` is the case, since it is the sketch whose
    // reading turns on `implied`.
    let mut plain = gcs_core::examples::altitudes();
    assert!(gcs_core::solve::solve(&mut plain, gcs_core::solve::SolveOpts::default()).success);
    let before = diagnose(&mut plain, DiagnoseOptions::default());

    let mut sk = gcs_core::examples::altitudes();
    let first = &sk.constraints[0];
    let mut c = gcs_core::constraints::Constraint::new(first.kind, first.args.clone());
    c.claim = true;
    sk.add(c);
    assert!(gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default()).success);
    let after = diagnose(&mut sk, DiagnoseOptions::default());

    assert_eq!((after.dof, after.status), (before.dof, before.status), "the claim moved the count");
    assert_eq!(after.over, before.over, "a claim painted the sketch Over");
    assert_eq!(after.implied, before.implied, "a claim moved a relation out of `implied`");
}

#[test]
fn a_claim_is_not_a_number_the_decomposition_reads() {
    // `cgraph::known_radii` takes its working set from `Sketch::hard_constraints`, which is where
    // "everything that must be satisfied" is written down once.  A claimed radius states no
    // radius, so the decomposition must not be able to tell it from an absent one.
    const CIRCLE: &str = "
point o hint(x: 0, y: 0)
circle k(center: o) hint(r: 20)
ground(o)
";
    let plain = drawn(CIRCLE);
    let claimed = drawn(&format!("{CIRCLE}claim radius(k) == 20\n"));
    let stated = drawn(&format!("{CIRCLE}radius(k) == 20\n"));
    assert_eq!(gcs_core::cgraph::known_radii(&claimed), gcs_core::cgraph::known_radii(&plain));
    assert_ne!(gcs_core::cgraph::known_radii(&stated), gcs_core::cgraph::known_radii(&plain));
}

#[test]
fn a_document_may_not_smuggle_a_claim_onto_an_unknown() {
    // elaboration refuses it with a span (above); a document is untrusted input and arrives by
    // another road entirely, so the flag is dropped there rather than honoured
    let sk = drawn(
        "
point o hint(x: 0, y: 0)
point p hint(x: 20, y: 0)
point q hint(x: 25, y: 8)
spline s(o, p, q, o, p, q, o)
point_on_spline(q, s)
ground(o)
",
    );
    let json = gcs_core::io::dumps(&sk, None)
        .replace("\"type\":\"PointOnSpline\"", "\"type\":\"PointOnSpline\",\"claim\":true");
    let back = gcs_core::io::loads(&json).unwrap();
    assert_eq!(back.constraints.iter().filter(|c| c.claim).count(), 0);
}

#[test]
fn a_claims_rows_are_the_rows_the_compiler_would_have_built() {
    // The verdict rests on rows the diagnosis assembles by hand (`System::conditioned_with`)
    // rather than on a compiled block, so the two must agree.  The check that cannot be fooled by
    // a wrong scale or a mismapped column: a claim is *consuming* exactly when stating it for
    // real would have cost the drawing a freedom, so make it real and count.
    for (src, tail) in [
        (RECT, "parallel(bottom, top)\n"),   // a theorem: adds nothing
        (RECT, "horizontal(top)\n"),         // a duplicate: adds nothing either
        ("
point a hint(x: 0, y: 0)
point c hint(x: 0, y: 9)
line ac(a, c)
ground(a)
", "vertical(ac)\n"),                        // consuming: the pose alone satisfies it
        ("
point a hint(x: 0, y: 0)
point b hint(x: 30, y: 0)
point c hint(x: 30, y: 40)
line ab(a, b)
line bc(b, c)
horizontal(ab)
ground(a)
", "vertical(bc)\n"),                        // consuming as well, with a bigger base
    ] {
        let mut claimed = drawn(&format!("{src}claim {tail}"));
        let d = diagnose(&mut claimed, DiagnoseOptions::default());
        let id = claimed.constraints.iter().find(|c| c.claim).unwrap().id;
        let consuming = d.claims_consuming.contains(&id);
        assert!(consuming || d.claims_theorem.contains(&id), "neither, for `{tail}`");

        // the same statement, stated
        let mut stated = drawn(&format!("{src}{tail}"));
        let s = diagnose(&mut stated, DiagnoseOptions::default());
        let cost = d.dof - s.dof;
        assert_eq!(
            consuming,
            cost > 0,
            "`{tail}`: judged consuming={consuming}, but stating it cost {cost} DOF",
        );
    }
}

#[test]
fn a_claim_reads_as_one_wherever_it_is_read_out() {
    // the constraint list, the banner and both bindings all ask `describe` what a constraint is,
    // so the word the document spells it with is the word they get
    let sk = drawn(&format!("{RECT}claim parallel(bottom, top)\n"));
    let c = sk.constraints.iter().find(|c| c.claim).unwrap();
    assert_eq!(gcs_core::io::describe(c), "claim Parallel(L0, L2)");
    let plain = sk.constraints.iter().find(|c| c.kind == CKind::Distance).unwrap();
    assert!(!gcs_core::io::describe(plain).starts_with("claim "));
}

#[test]
fn a_claimed_dimension_is_drawn_as_a_reference_dimension() {
    // parentheses are the drafting convention for a dimension that is measured rather than
    // controlling, which is a claim exactly — and they go round the whole label, so a claimed
    // radius reads `(R20)` and never `R(20)`
    let src = "
point o hint(x: 0, y: 0)
point p hint(x: 60, y: 0)
circle k(center: o) hint(r: 20)
ground(o)
horizontal line l(o, p)
distance(o, p) == 60
";
    let label = |sk: &Sketch, kind: CKind| -> String {
        let id = sk.constraints.iter().find(|c| c.kind == kind).unwrap().id;
        gcs_core::callout::layout(sk, 1.0)
            .into_iter()
            .find(|k| k.id == id)
            .expect("a dimension is drawn")
            .text
    };
    let stated = drawn(&format!("{src}radius(k) == 20\n"));
    assert_eq!(label(&stated, CKind::Radius), "R20");
    assert_eq!(label(&stated, CKind::Distance), "60");

    let claimed = drawn(&format!("{src}claim radius(k) == 20\n"));
    assert_eq!(label(&claimed, CKind::Radius), "(R20)", "the parentheses go round the whole label");
    assert_eq!(label(&claimed, CKind::Distance), "60", "an ordinary dimension is untouched");
}

// -- the case: the Peaucellier cell's straight line ---------------------------------------
//
// `peaucellier.sv` ends on `claim vertical(rail)`.  What the diagnosis must say is *theorem* —
// not `violated`, not `consuming`, and not `over` — and it must keep saying it when the rods
// change length.  (`examples_sv.rs` pins what the library advertises: dof 1, Under.)

/// Solve a cell built from the document's own three lengths, and hold the diagnosis to the
/// theorem: dof 1 (the crank), state Under, and the claim judged a theorem.
fn straight_line_holds(arm: f64, side: f64, crank: f64, rail_x: f64) {
    let mut sk = gcs_core::examples::peaucellier(arm, side, crank);
    let r = gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "does not solve: {}", r.message);

    // both pinned trace points sit on the rail the closed form predicts — the straight line,
    // measured off the drawing
    for g in [6, 7] {
        let (px, _) = sk.point_xy(g);
        assert!((px - rail_x).abs() < 1e-6, "point {g} at x = {px}, the rail at {rail_x}");
    }

    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (1, State::Under), "over={:?} violated={:?}", d.over, d.violated);
    assert!(d.over.is_empty(), "a theorem painted as a surplus: {:?}", d.over);

    let claim = sk.constraints.iter().find(|c| c.kind == CKind::Vertical).unwrap();
    assert!(claim.claim, "the vertical is the claim");
    assert_eq!(d.claims_theorem, vec![claim.id], "the claim is a theorem");
    assert!(d.claims_violated.is_empty() && d.claims_consuming.is_empty());
}

#[test]
fn the_straight_line_is_implied() {
    straight_line_holds(100.0, 60.0, 40.0, 80.0);
}

/// The theorem survives every number in the file: rewrite a rod length, and the rail moves to
/// (arm² − side²) / (2 · crank) — but the claim stays implied.  A dependency that held only at
/// the shipped numbers would be a coincidence, not a theorem.
#[test]
fn the_theorem_does_not_care_what_the_rods_measure() {
    straight_line_holds(90.0, 60.0, 40.0, 56.25);
    straight_line_holds(100.0, 70.0, 40.0, 63.75);
    straight_line_holds(100.0, 60.0, 50.0, 64.0);
}

/// The same theorem, proved without a curve — `peaucellier_rail.sv`.  The rail joins the pen to
/// a grounded point, and what makes it a proof rather than an observation is the *rank* test: a
/// claim is a theorem only when stating it would cost the mechanism no freedom, so a theorem
/// here says the pen's x cannot change as the crank turns.
#[test]
fn the_rail_proves_the_line_without_tracing_it() {
    let mut sk = gcs_core::examples::example("peaucellier_rail").expect("a registered case");
    let r = gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "does not solve: {}", r.message);

    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (1, State::Under), "the crank is still free");
    assert!(d.over.is_empty(), "a claim must never paint the drawing Over: {:?}", d.over);
    let claim = sk.constraints.iter().find(|c| c.claim).expect("the file ends on a claim");
    assert_eq!(claim.kind, CKind::Vertical);
    assert_eq!(d.claims_theorem, vec![claim.id], "the straight line is a theorem");
}

/// And it is a test rather than a decoration: move the anchor off the line the pen actually
/// draws and the same claim comes back *refuted*.  Without this the theorem above could be
/// passing for any reason at all.
#[test]
fn the_rail_is_refuted_when_it_is_not_where_the_pen_goes() {
    let src = gcs_core::examples::source("peaucellier_rail").unwrap()
        .replace("point anchor hint(x: 80, y: 0)", "point anchor hint(x: 70, y: 0)");
    let (prog, errs) = gcs_core::syntax::parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let mut e = gcs_core::program::elaborate(&prog);
    assert!(e.ok());
    assert!(gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default()).success);

    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    let claim = e.sketch.constraints.iter().find(|c| c.claim).unwrap();
    assert_eq!(d.claims_violated, vec![claim.id], "a rail in the wrong place is a counterexample");
    assert!(d.claims_theorem.is_empty());
}
