//! **An ordinate along any direction, and `level`** (`docs/ordinate-plan.md`): one statement,
//! `(q − p)·t̂ = d`, whatever the direction is — a view's own axis, a drawn line, an axis in
//! space, a plane's normal — and its zero, `a level(t) b`, of which `horizontal` and `vertical`
//! between points are aliases.  Held here against closed forms: where the points stand once
//! solved, the form each reading takes, the freedoms each removes, the words it is refused in,
//! and the spelling it is read back with.
use gcs_core::constraints::{CKind, OrdinateForm};
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::io;
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{dot, sub};

use crate::common::{ent, read, refused};

fn solved(e: &Elaborated) -> Sketch {
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    sk
}

fn dof(sk: &mut Sketch) -> i64 {
    diagnose(sk, DiagnoseOptions::default()).dof
}

/// Where the point the document calls `n` stands in space, as solved.
fn at(e: &Elaborated, sk: &Sketch, n: &str) -> [f64; 3] {
    sk.world_point(ent(e, n).i())
}

/// The form of every ordinate and level the document states, in statement order.
fn forms(e: &Elaborated) -> Vec<(CKind, OrdinateForm)> {
    e.sketch.user_constraints().iter().filter_map(|c| Some((c.kind, c.form()?))).collect()
}

/// Each statement as `describe` reads it back, in the document's own names.
fn said(e: &Elaborated) -> Vec<String> {
    e.sketch
        .user_constraints()
        .iter()
        .map(|c| io::describe_with(c, &|r| e.map.name_of(r).cloned()))
        .collect()
}

const VIEWS: &str = "\
unit mm
use std
in std.front {
  a := point hint((10, 20))
  a2 := point hint((30, 5))
}
in std.top {
  b := point hint((3, 4))
}
in std.side {
  c := point hint((7, 9))
}
";

/// Along an axis in space between points of two views: one row, `(B − A)·x̂`.
#[test]
fn an_ordinate_along_an_axis_relates_two_views() {
    let e = read(&format!("{VIEWS}a distance(25, along: std.x) b\n"));
    assert_eq!(forms(&e), vec![(CKind::Ordinate, OrdinateForm::Space)]);
    let sk = solved(&e);
    let (a, b) = (at(&e, &sk, "a"), at(&e, &sk, "b"));
    assert!((b[0] - a[0] - 25.0).abs() < 1e-9, "{a:?} {b:?}");
    let mut sk = sk;
    // four points drawn in three views, one number stated
    assert_eq!(dof(&mut sk), 4 * 2 - 1);
    assert_eq!(said(&e), vec!["a distance(25, along: std.x) b".to_string()]);
}

/// Along an axis turned in space, held: the ordinate is measured on its own direction.
#[test]
fn an_ordinate_along_a_turned_axis_is_measured_on_it() {
    let e = read(
        "unit mm\nt := axis hint(dir: (0.6, 0.8, 0))\nfix(dir == (0.6, 0.8, 0)) t\n\
         p := point hint((1, 2, 3))\nfix((1, 2, 3)) p\nq := point hint((5, 1, 0))\n\
         r := point hint((2, -4, 1))\np distance(4, along: t) q\np level(t) r\n",
    );
    assert_eq!(
        forms(&e),
        vec![(CKind::Ordinate, OrdinateForm::Space), (CKind::Level, OrdinateForm::Space)]
    );
    let mut sk = solved(&e);
    let (p, q, r) = (at(&e, &sk, "p"), at(&e, &sk, "q"), at(&e, &sk, "r"));
    let t = [0.6, 0.8, 0.0];
    assert!((dot(sub(q, p), t) - 4.0).abs() < 1e-9, "{q:?}");
    assert!(dot(sub(r, p), t).abs() < 1e-9, "{r:?}");
    // a direction read is no place read: the axis is not placed, and adds nothing to the count
    assert!(!sk.axes[ent(&e, "t").i()].placed, "the axis was given a place");
    assert_eq!(dof(&mut sk), 3 + 3 - 2);
    assert_eq!(
        said(&e),
        vec!["p distance(4, along: t) q".to_string(), "p level(t) r".to_string()]
    );
}

/// Along a line drawn where the points are, it is read in their view; along one in another
/// view, in space.
#[test]
fn an_ordinate_along_a_line_is_read_in_the_view_it_shares() {
    let e = read(
        "unit mm\nuse std\nin std.front {\no := point\nfix((0, 0)) o\nk := point\n\
         fix((3, 4)) k\nl := line(o, k)\np := point hint((1, 7))\nq := point hint((6, 2))\n\
         p distance(5, along: l) q\n}\nin std.top {\nm := line(hint((0, 0)), hint((0, 10)))\n\
         fix((0, 0)) m.p1\nfix((0, 10)) m.p2\n}\np level(m) q\n",
    );
    assert_eq!(
        forms(&e),
        vec![(CKind::Ordinate, OrdinateForm::InView), (CKind::Level, OrdinateForm::Space)]
    );
    let sk = solved(&e);
    let (p, q) = (sk.point_xy(ent(&e, "p").i()), sk.point_xy(ent(&e, "q").i()));
    assert!(((q.0 - p.0) * 0.6 + (q.1 - p.1) * 0.8 - 5.0).abs() < 1e-9, "{p:?} {q:?}");
    // the line in the top view runs along the world's y, which the front view sees edge on: the
    // two stand level along it in space, whatever they do in their own view
    let (pw, qw) = (at(&e, &sk, "p"), at(&e, &sk, "q"));
    assert!((pw[1] - qw[1]).abs() < 1e-9, "{pw:?} {qw:?}");
    assert_eq!(said(&e)[0], "p distance(5, along: l) q");
}

/// Level along an axis across two views: the same height in space.
#[test]
fn level_across_views_is_the_same_height() {
    let e = read(&format!("{VIEWS}a level(std.z) c\n"));
    let sk = solved(&e);
    let (a, c) = (at(&e, &sk, "a"), at(&e, &sk, "c"));
    assert!((a[2] - c[2]).abs() < 1e-9, "{a:?} {c:?}");
}

/// Within a view, the view's own words: the run and the rise, `horizontal` and `vertical` as
/// `level(up)` and `level(right)`, and the plane's words from its origin.  Each prints back as
/// the spelling of its case.
#[test]
fn a_views_words_are_its_own_axes() {
    let e = read(&format!(
        "{VIEWS}in std.front {{\nd := point hint((1, 1))\n}}\n\
         a distance(20, along: x) a2\na distance(-15, along: y) a2\nd level(up) a\n\
         d vertical a2\nb distance(5, along: u) std.top\nc level(v) std.side\n"
    ));
    use OrdinateForm::*;
    assert_eq!(
        forms(&e),
        vec![
            (CKind::Ordinate, PageU),
            (CKind::Ordinate, PageV),
            (CKind::Level, PageV),
            (CKind::Level, PageU),
            (CKind::Ordinate, CoordU),
            (CKind::Level, CoordV),
        ]
    );
    let sk = solved(&e);
    let xy = |n: &str| sk.point_xy(ent(&e, n).i());
    let (a, a2, d) = (xy("a"), xy("a2"), xy("d"));
    assert!((a2.0 - a.0 - 20.0).abs() < 1e-9 && (a2.1 - a.1 + 15.0).abs() < 1e-9);
    assert!((d.1 - a.1).abs() < 1e-9 && (d.0 - a2.0).abs() < 1e-9);
    assert!((xy("b").0 - 5.0).abs() < 1e-9 && xy("c").1.abs() < 1e-9);
    assert_eq!(
        said(&e),
        vec![
            "a distance(20, along: x) a2",
            "a distance(-15, along: y) a2",
            "d horizontal a",
            "d vertical a2",
            "b distance(5, along: u) std.top",
            "c level(v) std.side",
        ]
    );
    // the run said the other way is the same statement as its minus
    let e = read(&format!("{VIEWS}a distance(20, along: left) a2\n"));
    let sk = solved(&e);
    let (a, a2) = (sk.point_xy(ent(&e, "a").i()), sk.point_xy(ent(&e, "a2").i()));
    assert!((a.0 - a2.0 - 20.0).abs() < 1e-9);
}

/// A plane's normal from its origin, for a point drawn elsewhere: its height off the plane.
#[test]
fn along_a_planes_normal_is_a_height_off_it() {
    let e = read(&format!("{VIEWS}a distance(7, along: n) std.top\n"));
    assert_eq!(forms(&e), vec![(CKind::Ordinate, OrdinateForm::FrameN)]);
    let sk = solved(&e);
    assert!((at(&e, &sk, "a")[2] - 7.0).abs() < 1e-9);
    assert_eq!(said(&e), vec!["a distance(7, along: n) std.top".to_string()]);
}

/// An ordinate travels through JSON and through the lifted program, its form read again.
#[test]
fn an_ordinate_survives_a_document_and_a_lift() {
    let e = read(&format!(
        "{VIEWS}a distance(25, along: std.x) b\na distance(20, along: x) a2\nb level(std.y) c\n"
    ));
    let back = io::loads(&io::dumps(&e.sketch, None)).unwrap();
    let f = |sk: &Sketch| -> Vec<Option<OrdinateForm>> {
        sk.user_constraints().iter().map(|c| c.form()).collect()
    };
    assert_eq!(f(&back), f(&e.sketch));
    let text = gcs_core::program::dumps(&e.sketch);
    let again = read(&text);
    assert_eq!(f(&again.sketch), f(&e.sketch), "{text}");
}

#[test]
fn a_page_word_needs_both_points_in_one_view() {
    refused(&format!("{VIEWS}a horizontal b\n"), "E062", "no meaning in space", "a horizontal b");
    refused(
        &format!("{VIEWS}a distance(5, along: x) b\n"),
        "E062",
        "no meaning in space",
        "a distance(5, along: x) b",
    );
}

#[test]
fn a_direction_is_an_axis_or_a_line_never_a_plane() {
    refused(&format!("{VIEWS}a distance(5, along: std.top) b\n"), "E040", "within it", "along");
    refused(&format!("{VIEWS}a level(std.top) b\n"), "E040", "within it", "a level(std.top) b");
    refused(&format!("{VIEWS}a level(n) std.side\n"), "E040", "is on the plane", "a level(n) std.side");
}

#[test]
fn a_planes_own_words_are_against_the_plane() {
    refused(&format!("{VIEWS}a distance(5, along: u) a2\n"), "E040", "own direction", "along");
    refused(&format!("{VIEWS}a distance(5, along: std.x) std.top\n"), "E040", "one of its own", "along");
}

#[test]
fn a_zero_written_so_is_a_level() {
    refused(&format!("{VIEWS}a distance(0mm, along: y) a2\n"), "E040", "`a horizontal a2`", "0mm");
    refused(&format!("{VIEWS}b distance(0, along: v) std.top\n"), "E040", "`b level(v) std.top`", "0");
    refused(&format!("{VIEWS}a distance(0, along: std.x) b\n"), "E040", "`a level(std.x) b`", "0");
    // a zero a name stands for is a dimension like any other
    let e = read(&format!("{VIEWS}w := 0mm\na distance(w, along: x) a2\n"));
    assert_eq!(forms(&e), vec![(CKind::Ordinate, OrdinateForm::PageU)]);
}

#[test]
fn level_takes_its_direction_in_its_parentheses() {
    refused(&format!("{VIEWS}a level(along: up) a2\n"), "E040", "in its parentheses", "along");
}

/// A word of the table is a word: a direction by the same name is refused as the two readings
/// it would have, and anything else by that name — a point — is no direction to confuse.
#[test]
fn a_direction_word_that_names_a_direction_is_refused() {
    refused(
        &format!("{VIEWS}up := axis hint(dir: (0, 0, 1))\na level(up) a2\n"),
        "E040",
        "also the name of a direction",
        "a level(up) a2",
    );
    let e = read(&format!("{VIEWS}in std.front {{\nx := point hint((2, 2))\n}}\nx distance(4, along: x) a\n"));
    assert_eq!(forms(&e), vec![(CKind::Ordinate, OrdinateForm::PageU)]);
}

/// Two points of one view along its normal: identically nothing, refused by value.
#[test]
fn along_a_views_normal_between_its_own_points_is_refused() {
    refused(
        &format!("{VIEWS}a distance(5, along: std.y) a2\n"),
        "E061",
        "along its normal is not a question",
        "a distance(5, along: std.y) a2",
    );
}

/// Every form is a dimension with a free twin, one column wider.
#[test]
fn every_form_can_be_written_free() {
    let e = read(&format!(
        "{VIEWS}param s: Length hint(5)\nin std.front {{\nl := line(hint((0, 0)), hint((3, 4)))\n}}\n\
         a distance(s, along: x) a2\na distance(s, along: y) a2\na distance(s, along: l) a2\n\
         a distance(s, along: std.x) b\na distance(s, along: n) std.top\n"
    ));
    let mut seen = Vec::new();
    for c in e.sketch.user_constraints() {
        let k = &gcs_core::kernels::KERNELS[c.kernel_id()];
        assert!(c.free.is_some(), "{c:?}");
        seen.push((c.form(), k.n_par, k.n_const));
    }
    use OrdinateForm::*;
    assert_eq!(
        seen,
        vec![
            (Some(PageU), 5, 2),
            (Some(PageV), 5, 2),
            (Some(InView), 9, 2),
            (Some(Space), 13, 2),
            (Some(FrameN), 13, 2),
        ]
    );
}
