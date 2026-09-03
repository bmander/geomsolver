//! **Claims about solids** (Solvent §9.8) — issue #48, items 6 and 7.
//!
//! Claims were the best thing in the tool: a test suite for a drawing, with the diagnosis as its
//! runner.  What they could not say was anything about the *object*, and every claim there was
//! got judged at one pose.  These are both halves of that, and the property under test is the one
//! §9.7 already bought for a 2D claim, one stratum out: **a claim about a solid is judged and can
//! never act.**

use gcs_core::program::{elaborate, Code, Elaborated};
use gcs_core::syntax::parse;

fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let e = elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}\n{src}",
        e.errors().map(|d| d.message.clone()).collect::<Vec<_>>()
    );
    e
}

fn verdicts(e: &Elaborated) -> Vec<gcs_core::diagnose::SolidVerdict> {
    let mut sk = e.sketch.clone();
    let _ = gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
    gcs_core::diagnose::judge_solids(&sk)
}

/// A square `w` on a side with its lower-left corner at `(x, y)`, grounded, standing between
/// `lo` and `hi` along the page's own normal — face `f{tag}` and solid `s{tag}_`.
fn block(tag: &str, x: f64, y: f64, w: f64, lo: f64, hi: f64) -> String {
    format!(
        "point a{tag} hint(x: {x}, y: {y})\npoint b{tag} hint(x: {}, y: {y})\n\
         point c{tag} hint(x: {}, y: {})\npoint d{tag} hint(x: {x}, y: {})\n\
         line p{tag}(a{tag}, b{tag}) -> line q{tag}(b{tag}, c{tag}) -> \
         line r{tag}(c{tag}, d{tag}) -> line s{tag}(d{tag}, a{tag}) -> close\n\
         horizontal p{tag}\nvertical q{tag}\nhorizontal r{tag}\nvertical s{tag}\n\
         a{tag} distance({w}) b{tag}\na{tag} distance({w}) d{tag}\nground a{tag}\n\
         face f{tag}(p{tag}, q{tag}, r{tag}, s{tag})\n\
         solid s{tag}_(f{tag}, from: {lo}mm, to: {hi}mm)\n",
        x + w,
        x + w,
        y + w,
        y + w
    )
}

#[test]
fn a_clearance_is_measured_and_reported() {
    let src = format!("unit mm\n{}{}claim sA_ clear(2mm) sB_\nclaim sA_ clear(4mm) sB_\n",
                      block("A", 0.0, 0.0, 10.0, -5.0, 0.0),
                      block("B", 13.0, 0.0, 10.0, -5.0, 0.0));
    let v = verdicts(&read(&src));
    assert_eq!(v.len(), 2);
    assert!((v[0].measured - 3.0).abs() < 1e-6, "three apart, and got {}", v[0].measured);
    assert_eq!(v[0].holds, Some(true), "two of clearance holds");
    assert_eq!(v[1].holds, Some(false), "four does not");
    // **what a reader is owed is the measurement**, not a yes or no: `clear(4mm)` failing by a
    // millimetre and by a metre are different drawings
    assert!((v[1].measured - 3.0).abs() < 1e-6);
}

#[test]
fn containment_is_a_question_about_the_object() {
    // a small block inside a big one — the statement the pocket that held nothing would have
    // failed (issue #48, item 7)
    // B stands *strictly* inside A — three clear on every side, depth included.  Sharing a face
    // would make `inside` true and `fits` false, which is right and is not what this is about
    let src = format!(
        "unit mm\n{}{}claim sB_ inside sA_\nclaim sA_ inside sB_\nclaim sB_ fits(1mm) sA_\n\
         claim sB_ fits(4mm) sA_\n",
        block("A", 0.0, 0.0, 20.0, -10.0, 0.0),
        block("B", 5.0, 5.0, 10.0, -7.0, -3.0)
    );
    let v = verdicts(&read(&src));
    assert_eq!(v[0].holds, Some(true), "the small one is inside the big one");
    assert_eq!(v[1].holds, Some(false), "and the big one is not inside the small one");
    assert_eq!(v[2].holds, Some(true), "with a millimetre to spare all round");
    assert_eq!(v[3].holds, Some(false), "but not four");
}

#[test]
fn a_swept_claim_finds_the_worst_pose() {
    // A block that slides along an unknown the solver answers for, and a wall it must clear.
    // This is the thing every claim in the language could not say: a fact about the whole travel
    // rather than about the one pose the drawing happens to stand in (issue #48, item 6).
    //
    // The gap is `50 - (reach + 4)`, so it is least at the far end of the sweep — and the claim
    // is written over the travel rather than checked at three angles by hand, which is what the
    // V-twin's port timing and disc clearance were checked by.
    let src = "\
unit mm
point o hint(x: 0, y: 0)
ground o
point p hint(x: 10, y: 0)
o distance(reach, along: x) p
o distance(0, along: y) p
point q hint(x: 14, y: 0)
p distance(4, along: x) q
p distance(0, along: y) q
point r hint(x: 14, y: 4)
q distance(0, along: x) r
q distance(4, along: y) r
point s hint(x: 10, y: 4)
p distance(0, along: x) s
p distance(4, along: y) s
line e0(p, q) -> line e1(q, r) -> line e2(r, s) -> line e3(s, p) -> close
face arm_f(e0, e1, e2, e3)
solid arm(arm_f, depth: 3mm)
point w0 hint(x: 50, y: 0)
point w1 hint(x: 56, y: 0)
point w2 hint(x: 56, y: 4)
point w3 hint(x: 50, y: 4)
line g0(w0, w1) -> line g1(w1, w2) -> line g2(w2, w3) -> line g3(w3, w0) -> close
ground w0
horizontal g0
vertical g1
w0 distance(6) w1
w1 distance(4) w2
horizontal g2
vertical g3
face wall_f(g0, g1, g2, g3)
solid wall(wall_f, depth: 3mm)
claim over reach in (10mm, 40mm) { arm clear(1mm) wall }
claim arm clear(1mm) wall
";
    let e = read(src);
    let v = verdicts(&e);
    assert_eq!(v.len(), 2, "the swept claim and the one judged at the pose");
    let swept = &v[0];
    let worst = swept.worst.expect("a swept claim reports its worst pose");
    assert!((worst - 40.0).abs() < 1.0, "least room at the far end of the travel: {worst}");
    // at `reach = 40` the arm's far face is at 44 and the wall starts at 50
    assert!((swept.measured - 6.0).abs() < 0.2, "six of room at the worst pose: {}", swept.measured);
    assert_eq!(swept.holds, Some(true), "which is still a millimetre clear");
    // **the sweep is what makes it a different question**: judged at the pose alone the same
    // claim measures the travel it happens to stand at, which is not what a reader asked
    assert!(
        v[1].measured > swept.measured + 1.0,
        "at rest it reads {} and over the travel {}",
        v[1].measured,
        swept.measured
    );
}

#[test]
fn a_claim_about_a_solid_can_never_act() {
    // §9.7's property, one stratum out: adding one changes no equation, no rank, no DOF and no
    // parameter.  It is the whole reason a claim is safe to write.
    let plain = format!(
        "unit mm\n{}{}",
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
        "unit mm\n{}{}",
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
        "not a free variable of this drawing",
    );
}
