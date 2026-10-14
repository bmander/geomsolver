//! **Steps** (#330): `step(u: s, dv: b) l` says where a line's end stands, one ordinate at a
//! time — `du`, `dv` from its start, `u`, `v` from a frame's origin — and keeps the ordinate it
//! does not name.  On a chain's links it is the compact outline: one number per run, the runs
//! named, the closing edge whatever is left.  What is held here is what it is short for — the
//! ordinates and levels a person would otherwise write, measured in the view the line is drawn
//! in or the frame `from:` names — that each number is a dimension of its own, kept where it is
//! written, that the corners start where the walk puts them, and that what a step cannot mean is
//! said at the word.
use gcs_core::constraints::CKind;
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;

use crate::common::{dof, ent, read, refused, said, solved};

/// A turned flange's half-section: radii from the axis and heights from the base, from the
/// front's origin where the two meet, and the hub's height from where its run begins.
const FLANGE: &str = "\
unit mm
use std
in std.front {
  profile := step(u: 32mm) (ab := line) -> step(v: 6mm) (bc := line) ->
             step(u: 15mm) (cd := line) -> step(dv: 10mm) (de := line) ->
             step(u: 8mm) (ef := line) -> step(v: 0mm) (fa := line) -> close
}
";

/// Where the point the document calls `n` stands in its view.
fn xy(sk: &Sketch, e: &Elaborated, n: &str) -> (f64, f64) {
    sk.point_xy(ent(e, n).i())
}

fn near(got: (f64, f64), want: (f64, f64)) {
    let ok = (got.0 - want.0).abs() < 1e-9 && (got.1 - want.1).abs() < 1e-9;
    assert!(ok, "{got:?} against {want:?}");
}

/// The corners of the flange's section, in walk order from `ab`'s start.
const CORNERS: [(&str, (f64, f64)); 6] = [
    ("ab.p1", (8.0, 0.0)),
    ("ab.p2", (32.0, 0.0)),
    ("bc.p2", (32.0, 6.0)),
    ("cd.p2", (15.0, 6.0)),
    ("de.p2", (15.0, 16.0)),
    ("ef.p2", (8.0, 16.0)),
];

/// Six runs, one number each, are twelve rows for six corners: the section is determined, and
/// each corner is where the dimensions put it.
#[test]
fn a_stepped_section_is_determined_by_its_runs() {
    let e = read(FLANGE);
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    for (n, want) in CORNERS {
        near(xy(&sk, &e, n), want);
    }
    // a station or a radius from the origin, a run from the start, a level per ordinate kept,
    // and `v: 0mm` — back onto the axis — a level, since a zero ordinate states no number
    let k = |c: CKind| e.sketch.user_constraints().iter().filter(|x| x.kind == c).count();
    assert_eq!((k(CKind::Ordinate), k(CKind::Level)), (5, 7));
}

/// **The corners start where the walk puts them**: nothing restates a number in a `hint`, and
/// the seeds are the outline before any solve — twice round, so the first corner, which the last
/// step places, is where it closes.
#[test]
fn the_corners_start_where_the_steps_put_them() {
    let e = read(FLANGE);
    for (n, want) in CORNERS {
        near(xy(&e.sketch, &e, n), want);
    }
    // and a written seed is the source's: the walk leaves it where it was written
    let e = read(&FLANGE.replace("(ab := line)", "(ab := line(hint((9mm, 1mm))))"));
    near(xy(&e.sketch, &e, "ab.p1"), (9.0, 1.0));
    near(xy(&e.sketch, &e, "ab.p2"), (32.0, 1.0));
}

/// From the line's start, both ordinates may be named, and a closed walk of relative steps
/// leaves its last edge to close it: three steps and a held start are the whole rectangle.
#[test]
fn relative_steps_and_a_closing_edge() {
    let e = read("\
unit mm
use std
w := 60mm
in std.front {
  a := point
  fix((10, 5)) a
  outline := step(du: w) (bottom := line(a)) -> step(dv: 40mm) (side := line) ->
             step(du: -w, dv: 0mm) (top := line) -> (back := line) -> close
  slant := step(du: 3mm, dv: 4mm) line(a)
}
");
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    near(xy(&sk, &e, "side.p2"), (70.0, 45.0));
    near(xy(&sk, &e, "top.p2"), (10.0, 45.0));
    near(xy(&sk, &e, "slant.p2"), (13.0, 9.0));
}

/// **`from:` names the frame**: a step measured in turned axes runs along them, from their
/// origin or from the line's start, however the view they are drawn in stands.
#[test]
fn a_step_is_measured_in_the_frame_it_names() {
    let e = read("\
unit mm
use std
in std.front {
  o := point
  t := point hint((40, 20))
  fix((10, 5)) o
  tilted := std.Turned(o, t)
  tilted.u angle(30deg) std.x
  o distance(40mm) t
  path := step(u: 20mm, v: 0mm, from: tilted.axes) (a := line(o)) ->
          step(dv: 10mm, from: tilted.axes) (b := line)
}
");
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
    near(xy(&sk, &e, "a.p2"), (10.0 + 20.0 * c, 5.0 + 20.0 * s));
    near(xy(&sk, &e, "b.p2"), (10.0 + 20.0 * c - 10.0 * s, 5.0 + 20.0 * s + 10.0 * c));
}

/// A step is a statement of its own, written before any line — and an unknown its number reads
/// is solved for like any other.
#[test]
fn a_step_on_its_own_and_over_an_unknown() {
    let e = read("\
unit mm
use std
param w: Length
in std.front {
  a := point
  fix((0, 0)) a
  l := line(a)
  m := line(a)
  step(du: w) l
  step(dv: 7mm) m
  l equal m
}
");
    let mut sk = solved(&e);
    assert_eq!(dof(&mut sk), 0);
    near(xy(&sk, &e, "l.p2"), (7.0, 0.0));
}

/// **Each number is a dimension of the statement**: described as written, and edited where it
/// is written — the splice lands on the number in the step's parentheses.
#[test]
fn each_number_is_a_dimension_where_it_is_written() {
    let e = read(FLANGE);
    let s = said(&e);
    assert!(s.contains(&"step(u: 32mm) ab".to_string()), "{s:?}");
    assert!(s.contains(&"step(dv: 10mm) de".to_string()), "{s:?}");
    let c = e.sketch.user_constraints().into_iter()
        .find(|c| c.kind == CKind::Ordinate && c.word.as_ref().is_some_and(|w| w.args == "u: 32mm"))
        .expect("the outer radius");
    let edit = gcs_core::edit::set_dimension(&e, &e.program, c.id, "d", "40mm");
    assert!(edit.refused.is_none(), "{:?}", edit.refused);
    assert!(edit.text.contains("step(u: 40mm) (ab := line)"), "{}", edit.text);
    let e = read(&edit.text);
    near(xy(&solved(&e), &e, "ab.p2"), (40.0, 0.0));
}

/// What a step cannot mean is said once, at what was written.
#[test]
fn what_a_step_cannot_mean_is_said_at_the_word() {
    let doc =
        |link: &str| format!("unit mm\nuse std\nin std.front {{\n  p := {link} -> line\n}}\n");
    refused(&doc("step (l := line)"), "E040", "says where the line's end stands", "step");
    refused(&doc("step(5mm) (l := line)"), "E004", "names each ordinate", "5mm");
    refused(&doc("step(w: 5mm) (l := line)"), "E040", "not `w`", "w");
    refused(&doc("step(u: 5mm, du: 3mm) (l := line)"), "E040", "says again", "du");
    refused(&doc("step(u: 5mm, from: 3mm) (l := line)"), "E040", "names the plane", "3mm");
    refused(
        "unit mm\nuse std\np := step(u: 5mm) (l := line) -> line\n",
        "E040",
        "stands in no `in` block",
        "step(u: 5mm)",
    );
}

/// The colouring reads a step as the relation word it is, before a link or a line, and a name
/// spelled `step` — a formal, an argument — as a name still.
#[test]
fn a_step_is_tinted_as_a_relation() {
    use gcs_core::syntax::{highlight, Tint};
    let tints = |src: &str| -> Vec<Tint> {
        highlight(src)
            .into_iter()
            .filter(|(_, s)| src[s.lo as usize..].starts_with("step"))
            .map(|(t, _)| t)
            .collect()
    };
    assert_eq!(tints(FLANGE).len(), 6);
    assert!(tints(FLANGE).iter().all(|&t| t == Tint::Relation), "{:?}", tints(FLANGE));
    let formal = "component C(step: Length) {\n  w := step\n}\n";
    assert!(!tints(formal).contains(&Tint::Relation), "{:?}", tints(formal));
}
