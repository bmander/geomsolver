//! **An ordinate along any direction, and `level`** (`docs/ordinate-plan.md`): one statement,
//! `(q − p)·t̂ = d`, whatever the direction is — a view's own axis, a drawn line, an axis in
//! space, a plane's normal — and its zero, `a level(t) b`, of which `horizontal` and `vertical`
//! between points are aliases.  Held here against closed forms: where the points stand once
//! solved, the form each reading takes, the freedoms each removes, the words it is refused in,
//! and the spelling it is read back with.
use gcs_core::constraints::{CKind, OrdinateForm};
use gcs_core::io;
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;
use gcs_core::space::{dot, sub};

use crate::common::{dof, ent, read, refused, said, solved};

/// Where the point the document calls `n` stands in space, as solved.
fn at(e: &Elaborated, sk: &Sketch, n: &str) -> [f64; 3] {
    sk.world_point(ent(e, n).i())
}

/// The form of every ordinate and level the document states, in statement order.
fn forms(e: &Elaborated) -> Vec<(CKind, OrdinateForm)> {
    e.sketch.user_constraints().iter().filter_map(|c| Some((c.kind, c.form()?))).collect()
}

const VIEWS: &str = "\
unit mm
use std
use std.place (horizontal, vertical)
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
         a distance(20, along: u) a2\na distance(-15, along: v) a2\nd level(up) a\n\
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
            "a distance(20, along: u) a2",
            "a distance(-15, along: v) a2",
            "d level(up) a",
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
        "{VIEWS}a distance(25, along: std.x) b\na distance(20, along: u) a2\nb level(std.y) c\n"
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
        &format!("{VIEWS}a distance(5, along: u) b\n"),
        "E062",
        "no meaning in space",
        "a distance(5, along: u) b",
    );
}

#[test]
fn a_direction_is_an_axis_or_a_line_never_a_plane() {
    refused(&format!("{VIEWS}a distance(5, along: std.top) b\n"), "E040", "within it", "along");
    refused(&format!("{VIEWS}a level(std.top) b\n"), "E040", "within it", "a level(std.top) b");
    refused(&format!("{VIEWS}a level(n) std.side\n"), "E040", "is on the plane", "a level(n) std.side");
}

/// A plane's normal is said against the plane; between two points it is no direction.  (Its
/// `u` and `v` between two points are the view's, which `u_and_v_are_the_plane_at_hands` holds.)
#[test]
fn a_planes_own_words_are_against_the_plane() {
    refused(&format!("{VIEWS}a distance(5, along: n) a2\n"), "E040", "own direction", "along");
    refused(&format!("{VIEWS}a distance(5, along: std.x) std.top\n"), "E040", "one of its own", "along");
    let page_word = format!("{VIEWS}b distance(5, along: right) std.top\n");
    refused(&page_word, "E040", "one of its own", "along");
}

/// **One vocabulary for a view's directions** (#268): a view is a plane, so `u` and `v` are the
/// plane at hand's — the plane operand's against one, else the view both points are drawn in —
/// and `right` and `up` are the same two with the sign said.  Between two points of
/// `std.front`, `along: v` is the front's `v`, world z — what `along: up` measures — and never
/// the world's y.
#[test]
fn u_and_v_are_the_plane_at_hands() {
    let e = read(&format!(
        "{VIEWS}a distance(20, along: u) a2\na distance(15, along: v) a2\n\
         a2 level(u) std.front.origin\nb distance(5, along: u) std.top\n"
    ));
    use OrdinateForm::*;
    assert_eq!(
        forms(&e),
        vec![
            (CKind::Ordinate, PageU),
            (CKind::Ordinate, PageV),
            (CKind::Level, PageU),
            (CKind::Ordinate, CoordU),
        ]
    );
    let sk = solved(&e);
    let (a, a2) = (at(&e, &sk, "a"), at(&e, &sk, "a2"));
    // the run along world x, the rise along world z (the front's `v`), nothing along world y
    assert!((a2[0] - a[0] - 20.0).abs() < 1e-9, "{a:?} {a2:?}");
    assert!((a2[2] - a[2] - 15.0).abs() < 1e-9, "{a:?} {a2:?}");
    assert!((a2[1] - a[1]).abs() < 1e-9, "{a:?} {a2:?}");
    // and the word reads back as written, between points and against a plane alike
    assert_eq!(
        said(&e),
        vec![
            "a distance(20, along: u) a2",
            "a distance(15, along: v) a2",
            "a2 level(u) std.front.origin",
            "b distance(5, along: u) std.top",
        ]
    );
    // `u` is `right` and `v` is `up`, the sign unsaid: the same ordinate, the same solution
    let signed = read(&format!(
        "{VIEWS}a distance(20, along: right) a2\na distance(15, along: up) a2\n"
    ));
    let sk2 = solved(&signed);
    let (b, b2) = (at(&signed, &sk2, "a"), at(&signed, &sk2, "a2"));
    for k in 0..3 {
        assert!(((a2[k] - a[k]) - (b2[k] - b[k])).abs() < 1e-9, "{k}: {a:?} {a2:?} {b:?} {b2:?}");
    }
}

/// `x` and `y` are retired (#268): between two points of `std.front` they read as the world's
/// letters and meant the view's, so `along: y` was world z.  Each is refused at the key, naming
/// what it became — and, no longer a word, is free to be a name.
#[test]
fn x_and_y_are_no_longer_direction_words() {
    for (old, now) in [("x", "u"), ("y", "v")] {
        let want = format!(
            "`{old}` is no longer a direction word: a view's own directions are `u` and `v`, as a \
             plane's are, and `right`, `left`, `up` and `down` say the sign — write `{now}`"
        );
        refused(&format!("{VIEWS}a distance(5, along: {old}) a2\n"), "E040", &want, "along");
        refused(&format!("{VIEWS}a level({old}) a2\n"), "E040", &want, old);
    }
    // a direction by that name is a reference like any other
    let e = read(&format!(
        "{VIEWS}x := axis hint(dir: (1, 0, 0))\nfix(dir == (1, 0, 0)) x\n\
         a distance(5, along: x) a2\n"
    ));
    assert_eq!(said(&e), vec!["a distance(5, along: x) a2".to_string()]);
}

/// A zero is a number like any other: written so, a name for it, or what an expression comes to,
/// it is the ordinate's dimension — so a box centred on an axis needs no case of its own.
#[test]
fn a_zero_written_so_is_a_number() {
    for src in [
        "a distance(0mm, along: v) a2\n",
        "w := 0mm\na distance(w, along: v) a2\n",
        "w := 2mm\na distance(w - 2mm, along: v) a2\n",
    ] {
        let e = read(&format!("{VIEWS}{src}"));
        assert_eq!(forms(&e), vec![(CKind::Ordinate, OrdinateForm::PageV)], "{src}");
        let sk = solved(&e);
        let (a, a2) = (at(&e, &sk, "a"), at(&e, &sk, "a2"));
        assert!((a2[2] - a[2]).abs() < 1e-9, "{src}: {a:?} {a2:?}");
    }
    // against a plane, along a reference, and through `std`'s `coords`, where it was refused
    let e = read(&format!("{VIEWS}b distance(0, along: v) std.top\n"));
    assert_eq!(forms(&e), vec![(CKind::Ordinate, OrdinateForm::CoordV)]);
    let e = read(&format!("{VIEWS}a distance(0, along: std.x) b\n"));
    assert_eq!(forms(&e), vec![(CKind::Ordinate, OrdinateForm::Space)]);
    let e = read(&format!("{VIEWS}use std\nuse std.place (coords)\nb coords(du: 0mm, dv: 5mm) std.top\n"));
    let sk = solved(&e);
    let b = at(&e, &sk, "b");
    assert!(b[0].abs() < 1e-9 && (b[1] - 5.0).abs() < 1e-9, "{b:?}");
    let ordinates = (CKind::Ordinate, OrdinateForm::CoordU);
    assert_eq!(forms(&e), vec![ordinates, (CKind::Ordinate, OrdinateForm::CoordV)]);
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
    let e = read(&format!("{VIEWS}in std.front {{\nu := point hint((2, 2))\n}}\nu distance(4, along: u) a\n"));
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
         a distance(s, along: u) a2\na distance(s, along: v) a2\na distance(s, along: l) a2\n\
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
