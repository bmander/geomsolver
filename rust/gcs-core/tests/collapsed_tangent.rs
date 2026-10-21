//! **A tangency at a contact is no tangency along a line of no length** (#479): `tangent(at:)`
//! is the `tangent_arc_line` kernel's `(p − c)·(b − a)`, which holds for every contact once the
//! line's ends meet.  A run tangent at both its ends, from a point on each of two circles that
//! cross, then has a root with both its ends at the circles' crossing: isolated, of full rank,
//! residual nil.  A solve ending there fails (`solve::COLLAPSED`), and the diagnosis names the
//! tangencies (`Diagnosis::collapsed`); where it reaches the tangent, nothing is changed.
use gcs_core::decompose::PlanSolver;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::newton::Method;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts, COLLAPSED};

use crate::common::{ent, read};

/// The common tangent of two circles as `engine.parts.Span` writes it, the circles crossing
/// (`d = 4 < 3 + 2`), seeded poorly but on the upper side.
const CROSS: &str = "unit mm\nuse std\nin std.front {\n  a := point\n  b := point\n  \
                     fix((0, 0)) a\n  fix((4, 0)) b\n  ca := circle(center: a)\n  \
                     cb := circle(center: b)\n  radius(3mm) ca\n  radius(2mm) cb\n  \
                     s := point hint((2.4, 1.2))\n  e := point hint((2.8, 1.6))\n  \
                     run := line(s, e)\n  s coincident ca\n  e coincident cb\n  \
                     run tangent(at: p1) ca\n  run tangent(at: p2) cb\n}\n";

/// Where the run's ends are drawn.
fn ends(e: &Elaborated) -> [(f64, f64); 2] {
    ["s", "e"].map(|n| e.sketch.point_xy(ent(e, n).i()))
}

fn near(p: (f64, f64), q: (f64, f64)) -> bool {
    (p.0 - q.0).abs() < 1e-4 && (p.1 - q.1).abs() < 1e-4
}

/// From the poor seeds the solve comes to the crossing, `s = e`: not solved, the two tangencies
/// named, and never reported solved by the plan solver either.
#[test]
fn a_run_collapsed_where_its_circles_cross_is_not_solved() {
    let mut e = read(CROSS);
    let r = solve(&mut e.sketch, SolveOpts::default());
    let [s, t] = ends(&e);
    let crossing = (21.0 / 8.0, (9.0f64 - (21.0f64 / 8.0).powi(2)).sqrt());
    if near(s, t) {
        assert!(near(s, crossing), "{s:?}");
        assert!(!r.success && r.status == COLLAPSED, "{r:?}");
        let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
        assert_eq!(d.collapsed.len(), 2, "{d:?}");
    } else {
        // or the true tangent, which is as good an answer
        assert!(r.success && near(s, (0.75, 2.90474)) && near(t, (4.5, 1.93649)), "{s:?} {t:?}");
    }
    let mut e = read(CROSS);
    let mut ps = PlanSolver::new(&e.sketch, false);
    let r = ps.solve(&mut e.sketch, 1e-9, true, Method::DogLeg);
    let [s, t] = ends(&e);
    assert!(!(r.success && near(s, t)), "{r:?}: {s:?} {t:?}");
}

/// The controls: from good seeds the true external tangent, as before; with the circles clear
/// there is no crossing to collapse to; and the radii stated parallel in place of the second
/// tangency reach the same tangent.
#[test]
fn the_tangent_itself_is_still_solved() {
    let good = CROSS.replace("hint((2.4, 1.2))", "hint((0.6, 2.9))")
        .replace("hint((2.8, 1.6))", "hint((4.4, 1.95))");
    let clear = CROSS.replace("fix((4, 0)) b", "fix((10, 0)) b")
        .replace("hint((2.4, 1.2))", "hint((0, 3))").replace("hint((2.8, 1.6))", "hint((10, 2))");
    let parallel = good.replace("run tangent(at: p2) cb",
                                "ra := line(a, s)\n  rb := line(b, e)\n  ra parallel rb");
    let cases = [
        (&good, (0.75, 2.90474), (4.5, 1.93649)),
        (&clear, (0.3, 2.98496), (10.2, 1.98997)),
        (&parallel, (0.75, 2.90474), (4.5, 1.93649)),
    ];
    for (src, s, t) in cases {
        let mut e = read(src);
        let r = solve(&mut e.sketch, SolveOpts::default());
        let [p, q] = ends(&e);
        assert!(r.success, "{r:?}\n{src}");
        assert!(near(p, s) && near(q, t), "{p:?} {q:?}\n{src}");
        let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
        assert!(d.collapsed.is_empty(), "{d:?}");
    }
}
