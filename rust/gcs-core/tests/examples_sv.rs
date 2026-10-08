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
use gcs_core::model::{EntKind, Field};

/// What the drawing is made of: points, lines, circles, arcs.
type Shape = (usize, usize, usize, usize);

/// (key, entities, dof, status)
const DOCS: &[(&str, Shape, i64, State)] = &[
    ("impossible_triangle", (8, 1, 0, 0), 0, State::Conflict),
    ("altitudes", (12, 6, 0, 0), 3, State::Under),
    ("parallels", (11, 4, 0, 0), 1, State::Under),
    ("belt_tangency", (9, 1, 2, 0), 0, State::Well),
    ("belt_wrap", (11, 2, 0, 2), 0, State::Well),
    ("reflection", (11, 4, 0, 0), 0, State::Well),
    ("rect_fillets", (17, 4, 0, 4), 0, State::Well),
    ("rect_fillets_under", (17, 4, 0, 4), 1, State::Under),
    ("rect_fillets_conflict", (17, 4, 0, 4), 0, State::Conflict),
    ("slotted_link", (11, 2, 2, 2), 0, State::Well),
    ("square", (9, 4, 0, 0), 1, State::Under),
    ("ngon", (11, 5, 1, 0), 1, State::Under),
    ("edge_tabs", (13, 12, 0, 0), 0, State::Well),
    ("polygon_chain", (29, 12, 0, 0), 11, State::Under),
    ("truss", (22, 31, 0, 0), 0, State::Well),
    ("truss_redundant", (18, 23, 0, 0), 0, State::Over),
    ("truss_conflict", (18, 23, 0, 0), 0, State::Conflict),
    ("truss_floating", (22, 31, 0, 0), 3, State::Under),
    ("zigzag", (101, 93, 0, 0), 99, State::Under),
    ("k33", (11, 1, 0, 0), 0, State::Well),
    ("pythagoras", (13, 8, 0, 0), 0, State::Well),
    ("spline_follower", (16, 1, 0, 0), 14, State::Under),
    ("nurbs", (17, 2, 0, 0), 2, State::Under),
    ("catenary", (21, 0, 0, 0), 2, State::Under),
    ("dido", (21, 1, 0, 0), 0, State::Well),
    ("peaucellier", (13, 9, 1, 0), 1, State::Under),
    ("peaucellier_rail", (12, 9, 1, 0), 1, State::Under),
    ("jansen", (13, 12, 1, 0), 1, State::Under),
    ("bracket", (32, 21, 0, 0), 0, State::Well),
    ("engine", (633, 467, 43, 11), 0, State::Well),
    ("vtwin", (325, 321, 27, 4), 1, State::Under),
    ("vtwin_cylinder", (30, 23, 2, 0), 0, State::Well),
    ("vtwin_plate", (122, 124, 13, 4), 0, State::Well),
    ("vtwin_piston", (25, 22, 2, 0), 0, State::Well),
    ("vtwin_disc", (21, 18, 4, 0), 0, State::Well),
    ("vtwin_flywheel", (21, 18, 2, 0), 0, State::Well),
    ("vtwin_throttle", (45, 41, 3, 0), 0, State::Well),
];

#[test]
fn every_document_case_is_its_document() {
    for &(key, shape, dof, status) in DOCS {
        let src = examples::source(key).unwrap_or_else(|| panic!("{key} has no source"));
        // linked against the library, as the app links it: a document's modules are its own
        let (prog, errs, linked) = gcs_core::library::parse_linked(src);
        assert!(errs.is_empty(), "{key} does not parse: {errs:?}");
        assert!(linked.is_empty(), "{key} does not link: {linked:?}");
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

/// **Every seed in the library is written in a `hint(…)` clause** (Solvent §4.3).
///
/// The rule is lexical, so the check is too: a scalar the kind owns must not appear as a
/// constructor argument, and a place must be keyed too — no `hint at REF` (#47.2).  Grepping
/// the shipped documents is the only way to catch a spelling that still *parses* somewhere but
/// says the wrong thing about which numbers a solve may rewrite.
///
/// Which names are scalars is asked of `EntKind::fields()`, the one table that says so — a new
/// entity kind is then held to the rule by having a field, and not by anybody remembering to
/// add its letter to a list here.
#[test]
fn no_document_writes_a_seed_the_old_way() {
    for &(key, ..) in DOCS {
        let src = examples::source(key).unwrap_or_else(|| panic!("{key} has no source"));
        for (n, line) in src.lines().enumerate() {
            let code = line.split("//").next().unwrap_or("");
            let where_ = format!("{key}:{}: {line}", n + 1);
            assert!(!code.contains("hint at"), "`hint at` is `hint(at: …)` now — {where_}");
            // a declaration leads with its kind, and the scalars at issue are that kind's own
            let kind = code.split_whitespace().next().and_then(EntKind::parse);
            for (name, field) in kind.iter().flat_map(|k| k.fields()) {
                if *field != Field::Scalar {
                    continue;
                }
                // a scalar inside the brackets that say what the entity is made of
                if let Some(i) = code.find(&format!(" {name}: ")) {
                    let head = &code[..i];
                    assert!(
                        !head.contains('(') || head.contains("hint("),
                        "a seed in a constructor argument list — {where_}"
                    );
                }
            }
        }
    }
}

/// **The spatial demos** — documents the app opens as files rather than as cases, since they are
/// looked at in the glass box and have no sheet — hold what their headers say: the unknowns, the
/// equations and the freedom left.
#[test]
fn the_spatial_demos_are_what_they_say() {
    let demos: &[(&str, &str, (usize, usize), i64, State)] = &[
        ("skew_axes", include_str!("../../examples/skew_axes.sv"), (28, 28), 0, State::Well),
        (
            "hypoid_pitch_cones",
            include_str!("../../examples/hypoid_pitch_cones.sv"),
            (44, 44),
            0,
            State::Well,
        ),
        (
            "sphere_cone_cylinder",
            include_str!("../../examples/sphere_cone_cylinder.sv"),
            (47, 46),
            1,
            State::Under,
        ),
    ];
    for &(key, src, counts, dof, status) in demos {
        let (prog, errs, linked) = gcs_core::library::parse_linked(src);
        assert!(errs.is_empty() && linked.is_empty(), "{key}: {errs:?} {linked:?}");
        let mut e = gcs_core::program::elaborate(&prog);
        assert!(e.ok(), "{key}: {:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
        let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
        assert!(r.success, "{key} solves");
        let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
        assert_eq!(((d.n_params, d.n_equations), d.dof, d.status), (counts, dof, status), "{key}");
    }
}

