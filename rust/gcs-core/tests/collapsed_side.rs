//! **A side of no length is no perpendicular side** (#479): `parallel` and `perpendicular` are the
//! cross and the dot of the two lines' `b − a`, which hold whatever the other line once one has
//! no length.  A rectangle of four lines threaded `perpendicular`, two lengths stated and every
//! corner unseeded, starts about the origin at the size of a unit and grew its stated sides
//! first: its third side ended a point and the fourth closed across the diagonal, residual nil,
//! and that was reported solved.  It is solved again from its start at the size it ended at
//! (`System::rescaled`), which reaches the rectangle; a collapse no start explains fails
//! (`solve::COLLAPSED`), the relations named (`Diagnosis::collapsed`).
use gcs_core::decompose::PlanSolver;
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::newton::Method;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts, COLLAPSED};

use crate::common::{ent, read};

/// The document as it was drawn: the four corners unseeded, the circle a bystander.
const RECT: &str = "use std\n\nc0 := circle(center: std.front.origin) hint(r: 26.8) in std.front\n\n\
                    component Rectangle(w: Length, h: Length) {\n  length(w) (l1 := line) -> \
                    perpendicular length(h) (l2 := line) -> perpendicular (l3 := line) -> \
                    perpendicular (l4 := line) -> close\n}\n\n\
                    r0 := Rectangle(w: 30, h: 26.9) in std.front\n";

/// The four sides' lengths, in order.
fn sides(e: &Elaborated) -> [f64; 4] {
    ["r0.l1", "r0.l2", "r0.l3", "r0.l4"].map(|n| e.sketch.line_length(ent(e, n).i()))
}

fn rectangle(e: &Elaborated) -> bool {
    let s = sides(e);
    [30.0, 26.9, 30.0, 26.9].iter().zip(s).all(|(w, l)| (w - l).abs() < 1e-6)
}

#[test]
fn an_unseeded_rectangle_solves_to_a_rectangle() {
    let mut e = read(RECT);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{r:?}");
    assert!(rectangle(&e), "{:?}", sides(&e));
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert!(d.collapsed.is_empty(), "{d:?}");

    let mut e = read(RECT);
    let mut ps = PlanSolver::new(&e.sketch, false);
    let r = ps.solve(&mut e.sketch, 1e-9, true, Method::DogLeg);
    assert!(r.success && rectangle(&e), "{r:?}: {:?}", sides(&e));
}

/// Held on the collapsed root itself, at its size, the solve has nowhere to go: it fails,
/// never solved, and the two perpendiculars on the third side are named.
#[test]
fn a_side_seeded_as_a_point_is_not_solved() {
    let seeded = RECT.replace(
        "r0 := Rectangle(w: 30, h: 26.9) in std.front\n",
        "r0 := Rectangle(w: 30, h: 26.9) in std.front\n\
         fix((0, 0)) r0.l1.p1\nfix((30, 0)) r0.l2.p1\n\
         fix((30, 26.9)) r0.l3.p1\nfix((30, 26.9)) r0.l4.p1\n",
    );
    let mut e = read(&seeded);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(!r.success && r.status == COLLAPSED, "{r:?}: {:?}", sides(&e));
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!(d.collapsed.len(), 2, "{d:?}");
}
