//! **Claims about solids** (Solvent §9.8) — issue #48, items 6 and 7.
//!
//! Claims were the best thing in the tool: a test suite for a drawing, with the diagnosis as its
//! runner.  What they could not say was anything about the *object*, and every claim there was
//! got judged at one pose.  These are both halves of that, and the property under test is the one
//! §9.7 already bought for a 2D claim, one stratum out: **a claim about a solid is judged and can
//! never act.**

use gcs_core::program::{elaborate, Code, Elaborated};
use crate::common::{parse, parsed::read};

fn verdicts(e: &Elaborated) -> Vec<gcs_core::diagnose::SolidVerdict> {
    let mut sk = e.sketch.clone();
    let _ = gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
    gcs_core::diagnose::judge_solids(&sk)
}

/// A square `w` on a side with its lower-left corner at `(x, y)` in the front, grounded, standing
/// between `lo` and `hi` along the front's own normal — face `f{tag}` and solid `s{tag}_`.
fn block(tag: &str, x: f64, y: f64, w: f64, lo: f64, hi: f64) -> String {
    format!(
        "in std.front {{\na{tag} := point\nb{tag} := point hint(({}, {y}))\n\
         c{tag} := point hint(({}, {}))\nd{tag} := point hint(({x}, {}))\n\
         (p{tag} := line(a{tag}, b{tag})) -> (q{tag} := line(b{tag}, c{tag})) -> \
         (r{tag} := line(c{tag}, d{tag})) -> (s{tag} := line(d{tag}, a{tag})) -> close\n\
         horizontal p{tag}\nvertical q{tag}\nhorizontal r{tag}\nvertical s{tag}\n\
         a{tag} distance({w}) b{tag}\na{tag} distance({w}) d{tag}\nfix(({x}, {y})) a{tag}\n}}\n\
         f{tag} := face(p{tag}, q{tag}, r{tag}, s{tag})\n\
         s{tag}_ := solid(f{tag}, from: {lo}mm, to: {hi}mm)\n",
        x + w,
        x + w,
        y + w,
        y + w
    )
}

#[test]
fn a_clearance_is_measured_and_reported() {
    let src = format!("unit mm\nuse std (turns)\n{}{}claim sA_ clear(2mm) sB_\nclaim sA_ clear(4mm) sB_\n",
                      block("A", 0.0, 0.0, 10.0, -5.0, 0.0),
                      block("B", 13.0, 0.0, 10.0, -5.0, 0.0));
    let v = verdicts(&read(&src));
    assert_eq!(v.len(), 2);
    assert!((v[0].measured().unwrap() - 3.0).abs() < 1e-6, "three apart, and got {}", v[0].measured().unwrap());
    assert_eq!(v[0].holds(), Some(true), "two of clearance holds");
    assert_eq!(v[1].holds(), Some(false), "four does not");
    // **what a reader is owed is the measurement**, not a yes or no: `clear(4mm)` failing by a
    // millimetre and by a metre are different drawings
    assert!((v[1].measured().unwrap() - 3.0).abs() < 1e-6);
}

#[test]
fn containment_is_a_question_about_the_object() {
    // a small block inside a big one — the statement the pocket that held nothing would have
    // failed (issue #48, item 7)
    // B stands *strictly* inside A — three clear on every side, depth included.  Sharing a face
    // would make `inside` true and `fits` false, which is right and is not what this is about
    let src = format!(
        "unit mm\nuse std (turns)\n{}{}claim sB_ inside sA_\nclaim sA_ inside sB_\nclaim sB_ fits(1mm) sA_\n\
         claim sB_ fits(4mm) sA_\n",
        block("A", 0.0, 0.0, 20.0, -10.0, 0.0),
        block("B", 5.0, 5.0, 10.0, -7.0, -3.0)
    );
    let v = verdicts(&read(&src));
    assert_eq!(v[0].holds(), Some(true), "the small one is inside the big one");
    assert_eq!(v[1].holds(), Some(false), "and the big one is not inside the small one");
    assert_eq!(v[2].holds(), Some(true), "with a millimetre to spare all round");
    assert_eq!(v[3].holds(), Some(false), "but not four");
}

/// A block that slides along `reach`, an unknown the solver answers for, and a wall it must
/// clear: one claim over the travel and one at the pose.
const SLIDING: &str = "\
unit mm
use std (horizontal, vertical, turns)
in std.front {
o := point
fix((0, 0)) o
p := point hint((10, 0))
}
param reach: Length
in std.front {
o distance(reach, along: u) p
o horizontal p
q := point hint((14, 0))
p distance(4, along: u) q
p horizontal q
r := point hint((14, 4))
q vertical r
q distance(4, along: v) r
s := point hint((10, 4))
p vertical s
p distance(4, along: v) s
(e0 := line(p, q)) -> (e1 := line(q, r)) -> (e2 := line(r, s)) -> (e3 := line(s, p)) -> close
arm_f := face(e0, e1, e2, e3)
arm := solid(arm_f, depth: 3mm)
w0 := point
w1 := point hint((56, 0))
w2 := point hint((56, 4))
w3 := point hint((50, 4))
(g0 := line(w0, w1)) -> (g1 := line(w1, w2)) -> (g2 := line(w2, w3)) -> (g3 := line(w3, w0)) -> close
fix((50, 0)) w0
horizontal g0
vertical g1
w0 distance(6) w1
w1 distance(4) w2
horizontal g2
vertical g3
wall_f := face(g0, g1, g2, g3)
wall := solid(wall_f, depth: 3mm)
claim over reach in (10mm, 40mm) { arm clear(1mm) wall }
claim arm clear(1mm) wall
}
";

#[test]
fn a_swept_claim_finds_the_worst_pose() {
    // A block that slides along an unknown the solver answers for, and a wall it must clear.
    // This is the thing every claim in the language could not say: a fact about the whole travel
    // rather than about the one pose the drawing happens to stand in (issue #48, item 6).
    //
    // The gap is `50 - (reach + 4)`, so it is least at the far end of the sweep — and the claim
    // is written over the travel rather than checked at three angles by hand, which is what the
    // V-twin's port timing and disc clearance were checked by.
    let src = SLIDING;
    let e = read(src);
    let v = verdicts(&e);
    assert_eq!(v.len(), 2, "the swept claim and the one judged at the pose");
    let swept = &v[0];
    let worst = swept.worst().expect("a swept claim reports its worst pose");
    assert!((worst - 40.0).abs() < 1.0, "least room at the far end of the travel: {worst}");
    // at `reach = 40` the arm's far face is at 44 and the wall starts at 50
    assert!((swept.measured().unwrap() - 6.0).abs() < 0.2, "six of room at the worst pose: {}", swept.measured().unwrap());
    assert_eq!(swept.holds(), Some(true), "which is still a millimetre clear");
    // **the sweep is what makes it a different question**: judged at the pose alone the same
    // claim measures the travel it happens to stand at, which is not what a reader asked
    assert!(
        v[1].measured().unwrap() > swept.measured().unwrap() + 1.0,
        "at rest it reads {} and over the travel {}",
        v[1].measured().unwrap(),
        swept.measured().unwrap()
    );
}

#[test]
fn a_claim_about_a_solid_can_never_act() {
    // §9.7's property, one stratum out: adding one changes no equation, no rank, no DOF and no
    // parameter.  It is the whole reason a claim is safe to write.
    let plain = format!(
        "unit mm\nuse std (turns)\n{}{}",
        block("A", 0.0, 0.0, 10.0, -5.0, 0.0),
        block("B", 13.0, 0.0, 10.0, -5.0, 0.0)
    );
    let claimed = format!("{plain}claim sA_ clear(2mm) sB_\nclaim sB_ inside sA_\n");
    let (a, b) = (read(&plain), read(&claimed));
    assert_eq!(a.sketch.params.len(), b.sketch.params.len(), "no unknown");
    assert_eq!(a.sketch.constraints.len(), b.sketch.constraints.len(), "no equation");
    let dof = |e: &Elaborated| {
        let mut sk = e.sketch.clone();
        let _ = gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
        gcs_core::diagnose::diagnose(&mut sk, Default::default()).dof
    };
    assert_eq!(dof(&a), dof(&b), "and no freedom taken");
}

#[test]
fn what_is_written_wrong_is_refused() {
    let two = format!(
        "unit mm\nuse std (turns)\n{}{}",
        block("A", 0.0, 0.0, 10.0, -5.0, 0.0),
        block("B", 13.0, 0.0, 10.0, -5.0, 0.0)
    );
    let refused = |src: String, code: Code, needle: &str| {
        let (prog, errs) = parse(&src);
        assert!(errs.is_empty(), "{errs:?}");
        let e = elaborate(&prog);
        let saw: Vec<String> =
            e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect();
        assert!(
            e.diags.iter().any(|d| d.code == code && d.message.contains(needle)),
            "expected {} `{needle}`\n{src}\n{saw:#?}",
            code.as_str()
        );
    };
    // a clearance of nothing in particular
    refused(format!("{two}claim sA_ clear sB_\n"), Code::E040, "asks for room");
    // a word that relates solids, given something else
    refused(format!("{two}claim aA clear(2mm) sB_\n"), Code::E040, "relates solids");
    // a sweep along something that is not a free variable
    refused(
        format!("{two}claim over aA in (0deg, 90deg) {{ sA_ clear(2mm) sB_ }}\n"),
        Code::E040,
        "geometry, not a free variable",
    );
    refused(
        format!("{two}claim over nope in (0deg, 90deg) {{ sA_ clear(2mm) sB_ }}\n"),
        Code::E040,
        "not a free variable or a motion of this drawing",
    );
}

/// A ball swung about the z axis past a post: 10 out on a 2 mm ball, the post 15.5 out on the
/// other side on a 3 mm one, so at rest they stand 13.4 apart and three quarters of a turn round
/// half a millimetre, within the millimetre the claim asks for from 260° to 280°.
/// Each is an instance's solid, `swing.arm.body` and `swing.post.body`, and the claim is written
/// once at the top and once inside the component that makes them.
const SWUNG: &str = "\
unit mm
use std (turns)
component Ball(c: point, r: Length) {
  ball := { p | p distance(<= r) c }
  body := solid(ball)
}
component Swing(ax: line, a: point, b: point) {
  spin := frame
  spin turns(1) ax
  arm := Ball(a, r: 2mm)
  post := Ball(b, r: 3mm)
  swung := arm.body at(under: spin) 0deg
  claim over spin in (0deg, 360deg) { swung clear(1mm) post.body }
}
o := point
fix((0, 0, 0)) o
t := point
fix((0, 0, 1)) t
a := point
fix((10, 0, 0)) a
b := point
fix((0, -15.5, 0)) b
spindle := line(o, t)
swing := Swing(spindle, a, b)
claim over swing.spin in (0deg, 360deg) { swing.swung clear(1mm) swing.post.body }
claim swing.arm.body clear(1mm) swing.post.body
";

#[test]
fn a_claim_over_a_motion_reads_the_solids_of_instances() {
    // The body of a swept claim names its solids as any claim does: through an instance, and
    // from inside a component under the instance's prefix.  Each sweep finds the poses at which
    // the ball passes too near the post; at rest the same two stand well apart.
    let e = read(SWUNG);
    assert!(e.diags.is_empty(), "{:#?}", e.diags);
    let v = verdicts(&e);
    assert_eq!(v.len(), 3, "the claim in the component, the one beside it and the one at rest");
    for swept in &v[..2] {
        assert_eq!(swept.holds(), Some(false), "the ball passes too near the post");
        assert_eq!(swept.valid_samples().count(), 37, "every pose of the turn judged");
        let refuted: Vec<f64> = swept.poses().iter().filter(|p| p.holds() == Some(false))
            .map(|p| p.parameter().unwrap()).collect();
        assert_eq!(refuted.len(), 3, "{refuted:?}");
        assert!(refuted.iter().all(|t| (t - 270.0).abs() <= 10.0 + 1e-9), "{refuted:?}");
    }
    assert_eq!(v[2].holds(), Some(true), "at rest they are well apart");
}

/// **Judged a pose at a time, a claim comes to what it comes to judged at once** — the page's
/// worker steps a `ClaimJudge` between looks at its messages, and `judge_solids` is the same judge
/// run to the end, so the two cannot disagree.  Between steps it says which claim of how many and
/// which pose, in words a page shows as they are.
#[test]
fn a_judge_stepped_a_pose_at_a_time_comes_to_the_verdicts() {
    use gcs_core::diagnose::{ClaimJudge, SWEEP_STEPS};
    let e = read(SLIDING);
    let mut sk = e.sketch.clone();
    let _ = gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
    let whole = gcs_core::diagnose::judge_solids(&sk);
    let mut judge = ClaimJudge::new(&sk);
    let first = judge.progress(&sk);
    assert_eq!((first.claim, first.claims, first.pose, first.poses), (0, 2, 0, SWEEP_STEPS + 1));
    assert_eq!(first.fraction(), 0.0);
    assert!(first.doing().starts_with("judging claim over reach"), "{}", first.doing());
    assert!(first.doing().ends_with("(1 of 2), pose 1 of 37"), "{}", first.doing());
    let (mut steps, mut last) = (0, 0.0);
    while !judge.step(&sk) {
        steps += 1;
        let p = judge.progress(&sk);
        assert!(p.fraction() > last, "the judging moves on at every step");
        last = p.fraction();
    }
    assert_eq!(steps + 1, SWEEP_STEPS + 2, "a pose a step: the travel's 37 and the pose's one");
    let done = judge.progress(&sk);
    assert_eq!((done.fraction(), done.doing().as_str()), (1.0, "judged 2 claims"));
    let stepped = judge.verdicts();
    assert_eq!(stepped.len(), whole.len());
    for (s, w) in stepped.iter().zip(&whole) {
        assert_eq!(gcs_core::report::solid_claim_text(s), gcs_core::report::solid_claim_text(w));
        assert_eq!((s.stmt(), s.samples(), s.worst()), (w.stmt(), w.samples(), w.worst()));
    }
    // a step past the end is no step
    assert!(judge.step(&sk));
    assert_eq!(judge.verdicts().len(), 2);
}

/// **A page's diagnosis leaves the claims about solids to its worker**: asked not to judge them it
/// says nothing of them, and everything else it says is what it says judging them — a solid claim
/// reads the solved pose and enters nothing above it.
#[test]
fn a_diagnosis_asked_not_to_judge_solids_says_the_rest_unchanged() {
    use gcs_core::diagnose::{diagnose, summary, DiagnoseOptions};
    let e = read(SLIDING);
    let mut sk = e.sketch.clone();
    let _ = gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
    let judged = diagnose(&mut sk.clone(), DiagnoseOptions::default());
    let left = diagnose(&mut sk, DiagnoseOptions { solid_claims: false, ..Default::default() });
    assert_eq!(judged.solid_claims.len(), 2);
    assert!(left.solid_claims.is_empty());
    assert_eq!(summary(&left), summary(&judged));
    assert_eq!(left.status, judged.status);
    assert_eq!((left.dof, left.n_redundant), (judged.dof, judged.n_redundant));
}
