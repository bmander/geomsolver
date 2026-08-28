//! Presentation, and the line between it and what the drawing *is*.
//!
//! A class on a declaration and a `style` block that says what the class looks like are the
//! whole of it, and the property under test is mostly a *negative* one: changing how a drawing
//! is presented changes nothing the core computes.  Same solve, same DOF, same diagnosis.

use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::model::{EntKind, EntRef, Sketch};
use gcs_core::program::elaborate;
use gcs_core::style::Classes;
use gcs_core::syntax::parse;

fn read(src: &str) -> Sketch {
    let (p, errs) = parse(src);
    assert!(errs.is_empty(), "{src}: {errs:?}");
    let e = elaborate(&p);
    assert!(e.ok(), "{src}: {:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    e.sketch
}

const PLAIN: &str = "\
point a hint(x: 0, y: 0)
point b hint(x: 60, y: 0)
line ab(a, b) class construction
distance(a, b) == 60
horizontal(ab)
ground(a)
";

/// `class construction` draws exactly as the retired keyword did — from the base sheet, which is
/// the one rule the implementation ships and a document may override.
#[test]
fn the_base_sheet_is_what_makes_construction_dashed() {
    let sk = read(PLAIN);
    let l = EntRef::new(EntKind::Line, 0);
    assert!(sk.class_of(l).has("construction"));
    assert_eq!(sk.style_of(l).dash, Some(vec![7.0, 4.0]));
    assert_eq!(sk.style_of(l).width, None, "the base sheet says nothing else");
    assert_eq!(sk.style_of(l).color, None);
}

/// **A document that overrides `.construction` changes how it draws and nothing else.**
///
/// Same drawing, same freedoms, same verdict — and a JSON export that differs in nothing, since
/// a sheet is not a fact about any entity.
#[test]
fn overriding_a_class_changes_only_how_it_draws() {
    let styled =
        format!("style .construction {{ dash: 2 2; width: 0.5; color: #888888 }}\n{PLAIN}");
    let mut a = read(PLAIN);
    let mut b = read(&styled);
    assert_eq!(gcs_core::io::dumps(&a, Some(1)), gcs_core::io::dumps(&b, Some(1)));
    let opts = DiagnoseOptions::default();
    let (da, db) = (diagnose(&mut a, opts), diagnose(&mut b, opts));
    assert_eq!(da.dof, db.dof);
    assert_eq!(da.status, db.status);
    assert_eq!(da.status, State::Well);

    let l = EntRef::new(EntKind::Line, 0);
    let s = b.style_of(l);
    assert_eq!(s.dash, Some(vec![2.0, 2.0]), "the document's rule wins over the base sheet");
    assert_eq!(s.width, Some(0.5));
    assert_eq!(s.color.as_deref(), Some("#888888"));
}

/// Several classes cascade, later over earlier, and only on the properties the later one states.
#[test]
fn a_later_class_wins_only_what_it_says() {
    let src = "\
style .centerline { dash: 12 3 2 3; width: 0.5; color: #888888 }
style .heavy      { width: 2.5 }
point a hint(x: 0, y: 0)
point b hint(x: 60, y: 0)
line ab(a, b) class centerline heavy
";
    let sk = read(src);
    let s = sk.style_of(EntRef::new(EntKind::Line, 0));
    assert_eq!(s.width, Some(2.5), "a centreline drawn thick");
    assert_eq!(s.dash, Some(vec![12.0, 3.0, 2.0, 3.0]), "and still a centreline");
    assert_eq!(s.color.as_deref(), Some("#888888"));
}

/// An unmatched class draws as plain geometry and is not a diagnostic — exactly as in CSS, which
/// is also what makes paste work: a figure keeps its class names and picks up whatever the
/// destination says about them, or nothing.
#[test]
fn an_unmatched_class_is_not_an_error() {
    let sk = read("point a hint(x: 0, y: 0)\npoint b hint(x: 1, y: 0)\nline ab(a, b) class nobody\n");
    let l = EntRef::new(EntKind::Line, 0);
    assert!(sk.class_of(l).has("nobody"));
    assert_eq!(sk.style_of(l), gcs_core::style::Style::default());
}

/// Copy and paste carry classes: a figure copied out of a document brings its presentation.
#[test]
fn copy_and_paste_carry_classes() {
    let sk = read(PLAIN);
    let clip = gcs_core::io::copy(&sk, &[EntRef::new(EntKind::Line, 0)]);
    assert!(clip.lines[0].class.has("construction"));
    let mut dst = Sketch::new();
    gcs_core::io::paste(&mut dst, &clip, 10.0, 10.0);
    assert!(dst.lines[0].class.has("construction"));
}

/// The one key JSON reads and never writes: an export from before there were classes still opens.
#[test]
fn an_old_export_still_loads() {
    let src = r#"{"version":1,"points":[{"x":0,"y":0},{"x":10,"y":0}],
                  "lines":[{"p1":0,"p2":1,"construction":true}],"constraints":[]}"#;
    let sk = gcs_core::io::loads(src).expect("an old export loads");
    assert_eq!(sk.lines[0].class, Classes::one("construction"));
    let out = gcs_core::io::dumps(&sk, None);
    assert!(!out.contains("\"construction\":"), "the old key is never written back: {out}");
    assert!(out.contains("\"class\""));
}

/// A style block round-trips through the printer, which is what keeps a hand-written sheet from
/// being reformatted into something else on the first edit.
#[test]
fn a_style_block_prints_back() {
    let src = "style .centerline { dash: 12 3 2 3; width: 0.5; color: #888888 }\n";
    let (p, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let mut out = String::new();
    gcs_core::syntax::write_stmt_to(&mut out, &p.root().body[0].kind);
    assert_eq!(out, "style .centerline { dash: 12 3 2 3; width: 0.5; color: #888888 }");
}
