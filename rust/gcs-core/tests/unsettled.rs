//! **A solve that did not settle is no answer, and nothing is judged on its pose** (#449).
//!
//! Two equal circles, their two external tangents, and a point stated on both: the tangents are
//! parallel, so no finite point is on both and the document has no solution.  The solve runs the
//! point off towards infinity, and a drawing ten kilometres across measures every row against
//! that extent: a contact 0.13 mm inside its 20 mm circle read as 3e-8, and a recompile at the
//! runaway's size (`System::resized`) accepted it on the iteration limit as *solved* — whereupon
//! the diagnosis judged the claim a theorem on a pose that does not hold its own premises.  A
//! recovery is now kept only where it settles (`SolveResult::settled`), the solve's outcome says
//! so (`SolveResult::outcome`), and a diagnosis told the pose is no settled answer
//! (`DiagnoseOptions::settled`) judges no claim on it.  The controls, unequal radii and equal ones
//! without the point, solve and settle where they always did.
use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts, SolveResult};

use crate::common::{at, read};

/// The belt: two runs tangent to both circles, `upper` and `lower`, with `rb` the second radius;
/// `meet` the point stated on both runs and claimed on the line of centres.
fn belt(rb: f64, meet: bool) -> String {
    let mut src = format!(
        "unit mm\nuse std\n\
         component Run(view: plane, a: circle, b: circle, up: Scalar) {{\n\
           in view {{\n\
             s := point hint((a.center.x, a.center.y + up * a.r))\n\
             e := point hint((b.center.x, b.center.y + up * b.r))\n\
             span := line(s, e)\n\
             s coincident a\n\
             e coincident b\n\
             span tangent(at: p1) a\n\
             span tangent(at: p2) b\n\
           }}\n\
         }}\n\
         in std.front {{\n\
           a := point\nb := point\nfix((0, 0)) a\nfix((120, 0)) b\n\
           ca := circle(center: a)\ncb := circle(center: b)\n\
           radius(20mm) ca\nradius({rb}mm) cb\n\
           centres := line(a, b)\n\
         }}\n\
         upper := Run(std.front, ca, cb, up: 1)\n\
         lower := Run(std.front, ca, cb, up: -1)\n"
    );
    if meet {
        src.push_str(
            "in std.front {\n\
               x := point hint((-96, 0))\n\
               x coincident upper.span\n\
               x coincident lower.span\n\
               claim x coincident centres\n\
             }\n",
        );
    }
    src
}

fn solved(src: &str) -> (Elaborated, SolveResult) {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    (e, r)
}

fn claimed(e: &Elaborated) -> u32 {
    e.sketch.constraints.iter().find(|c| c.claim).expect("the claim").id
}

#[test]
fn parallel_tangents_held_to_meet_do_not_solve_and_judge_no_claim() {
    let (mut e, r) = solved(&belt(20.0, true));
    assert!(!r.settled(), "no point is on two parallel lines: {r:?}");
    assert_ne!(r.outcome(), "solved", "{r:?}");
    let d = diagnose(&mut e.sketch, DiagnoseOptions { settled: r.settled(), ..Default::default() });
    let c = claimed(&e);
    assert!(
        !d.claims_theorem.contains(&c) && !d.claims_consuming.contains(&c),
        "a claim judged on a pose that holds none of its premises: {:?}",
        d.claims_theorem
    );
    assert!(matches!(d.status, State::Unsolved | State::Conflict), "{:?}", d.status);
    assert!(
        gcs_core::diagnose::summary(&d).contains("UNSOLVED")
            || gcs_core::diagnose::summary(&d).contains("CONFLICT"),
        "{}",
        gcs_core::diagnose::summary(&d)
    );
}

/// Unequal radii: the external tangents meet on the line of centres at the external centre of
/// similitude, `−r_a·|ab|/(r_b − r_a)` = −96 from `a`, and the claim is a theorem.
#[test]
fn unequal_circles_tangents_meet_where_they_did() {
    let (mut e, r) = solved(&belt(45.0, true));
    assert!(r.settled() && r.outcome() == "solved", "{r:?}");
    let x = at(&e, "x");
    assert!((x[0] + 96.0).abs() < 1e-9 && x[1].abs() < 1e-9, "{x:?}");
    let d = diagnose(&mut e.sketch, DiagnoseOptions { settled: r.settled(), ..Default::default() });
    assert_eq!(d.claims_theorem, vec![claimed(&e)]);
    assert_eq!(d.status, State::Well);
}

/// Equal radii without the point: the runs lie along the circles' common tangents at ±20.
#[test]
fn equal_circles_belt_alone_solves_where_it_did() {
    let (e, r) = solved(&belt(20.0, false));
    assert!(r.settled(), "{r:?}");
    for (run, up) in [("upper", 1.0), ("lower", -1.0)] {
        let (s, t) = (at(&e, &format!("{run}.s")), at(&e, &format!("{run}.e")));
        assert!(s[0].abs() < 1e-9 && (s[1] - 20.0 * up).abs() < 1e-9, "{run}.s {s:?}");
        assert!((t[0] - 120.0).abs() < 1e-9 && (t[1] - 20.0 * up).abs() < 1e-9, "{run}.e {t:?}");
    }
}

/// A diagnosis told its pose is no settled answer judges no claim on it, even where the pose is
/// a solution, and a drawing whose rows all read as holding is unsolved rather than well: the
/// verdict rests on the solve, which the caller holds and the pose does not.
#[test]
fn a_pose_no_solve_settled_at_is_judged_for_nothing() {
    let (mut e, r) = solved(&belt(45.0, true));
    assert!(r.settled());
    let d = diagnose(&mut e.sketch, DiagnoseOptions { settled: false, ..Default::default() });
    assert!(d.claims_theorem.is_empty() && d.claims_violated.is_empty());
    assert!(d.claims_consuming.is_empty());
    assert_eq!((d.status, d.dof), (State::Unsolved, 0));
}

/// What a solve came to, in the words every surface says it in.
#[test]
fn a_stop_on_the_iteration_limit_is_said_as_one() {
    let mut r = SolveResult::plain("dogleg", true, 1e-8, 100);
    assert_eq!(r.outcome(), "solved");
    r.status = 4;
    assert!(!r.settled());
    assert_eq!(r.outcome(), "stopped at the iteration limit");
    r.success = false;
    assert_eq!(r.outcome(), "did not solve");
}
