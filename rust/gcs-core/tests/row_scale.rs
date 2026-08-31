//! The solver iterates on residuals in each row's own units (`System::row_scale`), not raw.
//!
//! Issue #43.4: a four-bar linkage — three lengths and one `angle` — did not solve.  An angle
//! row is a bearing gap in radians, degree 0 and O(1); a distance row is a squared length,
//! degree 2 and O(L²).  Minimised raw, the merit function, the trust-region ratio and the
//! Cauchy step all belonged to the length rows, and the crank turned about a degree an
//! iteration — worse the bigger the drawing, since the ratio between the rows is the square of
//! its size.  "Solved" was already judged relative to each row's units; the iteration now sees
//! the same vector, and the two agree.

use gcs_core::solve::{solve, SolveOpts};

fn solved(src: &str) -> (bool, i32, f64) {
    let (prog, errs) = gcs_core::syntax::parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(), "does not elaborate");
    let mut sk = e.sketch;
    let r = solve(&mut sk, SolveOpts::default());
    (r.success, r.iterations, r.max_residual)
}

#[test]
fn a_four_bar_with_its_crank_angle_stated_solves() {
    let (ok, it, res) = solved(
        "point a hint(x: 0, y: 0)
         point d hint(x: 60, y: 0)
         point b hint(x: 8, y: 24)
         point c hint(x: 52, y: 30)
         line ground_link(a, d)
         line crank(a, b)
         line coupler(b, c)
         line rocker(d, c)
         a distance(25) b
         b distance(45) c
         d distance(30) c
         ground a
         ground d
         crank angle(70) ground_link",
    );
    assert!(ok, "did not solve: residual {res:.3e} after {it} iterations");
    assert!(it < 40, "{it} iterations for four unknowns");
}

/// Two dimensioned lines from a grounded corner, seeded square and asked for 60°, at every
/// size: the iteration count must not grow with the drawing.
#[test]
fn an_angle_beside_lengths_solves_at_every_size() {
    for side in [1.0, 10.0, 40.0, 400.0, 4000.0] {
        let (ok, it, res) = solved(&format!(
            "point o hint(x: 0, y: 0)
             point a hint(x: {side}, y: 0)
             point b hint(x: 0, y: {side})
             line oa(o, a)
             line ob(o, b)
             horizontal oa
             o distance({side}) a
             o distance({side}) b
             ground o
             oa angle(60) ob"
        ));
        assert!(ok, "side {side}: did not solve, residual {res:.3e} after {it} iterations");
        assert!(it < 30, "side {side}: {it} iterations");
    }
}

/// The residuals a system hands out are dimensionless, so a distance row and an angle row that
/// are each off by the same fraction of their own units read the same size.
#[test]
fn residuals_are_in_row_units() {
    let (prog, _) = gcs_core::syntax::parse(
        "point o hint(x: 0, y: 0)
         point a hint(x: 1000, y: 0)
         point b hint(x: 0, y: 1000)
         line oa(o, a)
         line ob(o, b)
         o distance(1000) a
         oa angle(90) ob
         ground o",
    );
    let mut sk = gcs_core::program::elaborate(&prog).sketch;
    let mut sys = gcs_core::system::System::new(&sk);
    let z = sys.z0(&sk);
    assert!(sys.max_relative_residual(&z) < 1e-9, "seeded at the answer");
    // move `a` out by 1% of the extent: the distance row is off by ~2% of L² raw, the angle by
    // nothing; both readings are O(1e-2) or less, and neither is O(1e4)
    let px = sk.points[1].x as usize;
    sk.params[px].value += 10.0;
    let z = sys.z0(&sk);
    let r = sys.residuals(&z);
    let peak = r.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    assert!(peak > 1e-3 && peak < 1.0, "peak residual {peak} is not in row units");
}
