//! The case library's documents.
//!
//! A case written as a Solvent document *is* that document: `examples::source` hands back the
//! text somebody wrote and `examples::case` hands back what elaborating it produces, and the two
//! must be the same drawing or the Program panel is showing a different sketch from the canvas.
//! Each entry below also pins what the library advertises about the case — how many degrees of
//! freedom are left and what the diagnosis calls it — since that is the whole reason the case is
//! in the library, and a document is very easy to edit into a drawing that no longer shows it.

use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::examples;

/// What the drawing is made of: points, lines, circles, arcs.
type Shape = (usize, usize, usize, usize);

/// (key, entities, dof, status)
const DOCS: &[(&str, Shape, i64, State)] = &[
    ("impossible_triangle", (3, 1, 0, 0), 0, State::Conflict),
    ("altitudes", (7, 6, 0, 0), 3, State::Under),
    ("parallels", (8, 4, 0, 0), 1, State::Under),
    ("belt_tangency", (4, 1, 2, 0), 0, State::Well),
    ("rect_fillets", (12, 4, 0, 4), 0, State::Well),
    ("rect_fillets_under", (12, 4, 0, 4), 1, State::Under),
    ("rect_fillets_conflict", (12, 4, 0, 4), 0, State::Conflict),
    ("slotted_link", (6, 2, 2, 2), 0, State::Well),
    ("polygon_chain", (24, 12, 0, 0), 11, State::Under),
    ("truss", (17, 31, 0, 0), 0, State::Well),
    ("truss_redundant", (13, 23, 0, 0), 0, State::Over),
    ("truss_conflict", (13, 23, 0, 0), 0, State::Conflict),
    ("truss_floating", (17, 31, 0, 0), 3, State::Under),
    ("zigzag", (96, 93, 0, 0), 99, State::Under),
    ("k33", (6, 1, 0, 0), 0, State::Well),
    ("pythagoras", (8, 8, 0, 0), 0, State::Over),
    ("spline_follower", (11, 1, 0, 0), 14, State::Under),
];

#[test]
fn every_document_case_is_its_document() {
    for &(key, shape, dof, status) in DOCS {
        let src = examples::source(key).unwrap_or_else(|| panic!("{key} has no source"));
        let (prog, errs) = gcs_core::syntax::parse(src);
        assert!(errs.is_empty(), "{key} does not parse: {errs:?}");
        let e = gcs_core::program::elaborate(&prog);
        assert!(
            e.ok(),
            "{key} does not elaborate: {:?}",
            e.errors().map(|d| (d.code.as_str(), d.message.clone())).collect::<Vec<_>>()
        );

        let mut sk = examples::case(key).unwrap_or_else(|| panic!("{key} is not a case"));
        let got = (sk.points.len(), sk.lines.len(), sk.circles.len(), sk.arcs.len());
        assert_eq!(got, shape, "{key}: entities");
        // the case and its source are one drawing, not two
        let from_src = (
            e.sketch.points.len(),
            e.sketch.lines.len(),
            e.sketch.circles.len(),
            e.sketch.arcs.len(),
        );
        assert_eq!(from_src, shape, "{key}: the source draws something else");
        assert_eq!(gcs_core::io::dumps(&e.sketch, None), gcs_core::io::dumps(&sk, None), "{key}: not one drawing");

        // what the library advertises is true of the drawing, which is the solved pose: before
        // the solve a case like `altitudes` merely has its incidences unmet
        let _ = gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
        let d = diagnose(&mut sk, DiagnoseOptions::default());
        assert_eq!((d.dof, d.status), (dof, status), "{key}: what the library advertises");
    }
}

/// The arguments a case takes are its document's own `param` lines, so asking for another size
/// gives another drawing rather than the default one.
#[test]
fn a_cases_arguments_reach_its_document() {
    let wide = examples::rect_fillets(140.0, 60.0, 10.0, 0.0);
    let base = examples::rect_fillets(100.0, 60.0, 10.0, 0.0);
    let span = |sk: &gcs_core::model::Sketch| {
        let xs: Vec<f64> = (0..sk.points.len()).map(|i| sk.point_xy(i).0).collect();
        xs.iter().cloned().fold(f64::MIN, f64::max) - xs.iter().cloned().fold(f64::MAX, f64::min)
    };
    assert!((span(&wide) - 140.0).abs() < 1e-9, "width reached the drawing: {}", span(&wide));
    assert!((span(&base) - 100.0).abs() < 1e-9, "{}", span(&base));
}
