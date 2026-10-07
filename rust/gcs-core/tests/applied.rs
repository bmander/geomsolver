//! **Predicates, applied** (#103): a relation word and a set are one thing — a body over
//! parameters, given where it is called, and bound variables, filled where it is used — applied
//! by one routine (`flatten::apply`).  So a word's body may hold several statements and the
//! declarations they need, a set may be written where it is used, a word reads inside a set and a
//! set inside a word, and a tangency to a set linearises through the words its body writes.
use gcs_core::io;
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{cross, dot, norm, sub};

use crate::common::{ends, ent, off_line, read, refused, unit};

fn solved(e: &Elaborated) -> Sketch {
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    sk
}

fn at(sk: &Sketch, e: &Elaborated, n: &str) -> [f64; 3] {
    sk.world_point(ent(e, n).i())
}

/// Each statement as `describe` reads it back, in the document's names.
fn said(e: &Elaborated) -> Vec<String> {
    e.sketch
        .user_constraints()
        .iter()
        .map(|c| io::describe_with(c, &|r| e.map.name_of(r).cloned()))
        .collect()
}

/// Two lines in the front view, one held.
const LINES: &str = "\
unit mm
use std
in std.front {
  a := line(hint((0, 0)), hint((40, 0)))
  fix((0, 0)) a.p1
  fix((40, 0)) a.p2
  b := line(hint((3, 12)), hint((35, 15)))
}
";

/// **A word whose body is several statements** states each, all read as the statement: two
/// lines flush — parallel, and `d` apart.
#[test]
fn a_word_states_several_relations() {
    let e = read(&format!(
        "l1 flush(d) l2 := {{\n  l1 parallel l2\n  l2.p1 distance(d) l1\n}}\n{LINES}\
         a flush(d: 12) b\n"
    ));
    let sk = solved(&e);
    let (p, q) = ends(&sk, ent(&e, "a"));
    let (r, s) = ends(&sk, ent(&e, "b"));
    for x in [r, s] {
        assert!((off_line(x, p, sub(q, p)) - 12.0).abs() < 1e-9, "{x:?}");
    }
    let s = said(&e);
    assert_eq!(s, ["a flush(d: 12) b", "a flush(d: 12) b"], "{s:?}");
}

/// A word's body may declare what its relations need, made once for each use under the use's
/// own name, and a dimension that is a parameter reads, and edits, the argument it was given.
#[test]
fn a_word_declares_what_it_needs_once_per_use() {
    let src = format!(
        "a apart(d) b := {{\n  private ab := line(a, b)\n  distance(d) ab\n}}\n\
         w := 20\n{LINES}in std.front {{\n  p := point hint((5, 5))\n  q := point hint((9, 9))\n}}\n\
         a.p1 apart(d: w) p\na.p2 apart(d: 7) q\n"
    );
    let e = read(&src);
    let sk = solved(&e);
    assert!((norm(sub(at(&sk, &e, "p"), at(&sk, &e, "a.p1"))) - 20.0).abs() < 1e-9);
    assert!((norm(sub(at(&sk, &e, "q"), at(&sk, &e, "a.p2"))) - 7.0).abs() < 1e-9);
    // two uses, two lines of their own
    assert_eq!(e.sketch.lines.len(), read(&format!("{LINES}in std.front {{\n  p := point\n  q := point\n}}\n"))
        .sketch.lines.len() + 2);
    let c = e.sketch.user_constraints().into_iter()
        .find(|c| c.word.as_ref().is_some_and(|w| w.args == "d: w")).unwrap();
    assert_eq!(c.written.as_deref(), Some("w"));
    let c = e.sketch.user_constraints().into_iter()
        .find(|c| c.word.as_ref().is_some_and(|w| w.args == "d: 7")).unwrap();
    let edit = gcs_core::edit::set_dimension(&e, &e.program, c.id, "d", "9");
    assert!(edit.refused.is_none(), "{:?}", edit.refused);
    assert!(edit.text.contains("a.p2 apart(d: 9) q"), "{}", edit.text);
}

/// What a word's body declares is its own: a body reading anything else is refused at the
/// definition, and a fault in what it states is said where the word is used.
#[test]
fn a_words_body_is_closed_and_said_at_the_use() {
    refused(
        &format!("a stray b := {{\n  private m := point\n  m coincident elsewhere\n}}\n{LINES}"),
        "E101",
        "not an operand or a parameter of `stray`",
        "elsewhere",
    );
    refused(
        &format!("l1 flush(d) l2 := {{\n  l1 parallel l2\n  l2.p1 distance(d) l1\n}}\n{LINES}\
                  in std.front {{\n  c := circle hint(r: 3)\n}}\na flush(d: 5) c\n"),
        "E040",
        "",
        "flush",
    );
}

/// The ball, the post and the tube used as their predicates: a sphere written in place, a set
/// whose body reads a word, a word whose body puts a point on a set.
const SPACE: &str = "\
unit mm
use std
side := plane(u: std.z, v: std.y)
fix(origin == (0, 0, 0)) side
a near(d) b := a distance(d) b
in std.front {
  c := point
  fix((0, 30)) c
  bx := line
  fix((0, 20)) bx.p1
  fix((50, 20)) bx.p2
}
q := point hint((10, 20)) in side
l := line(hint((5, 3)), hint((30, 20))) in side
fix((5, 3)) l.p1
";

/// **A set written where it is used** — a literal, or a family's call — reads as written.
#[test]
fn a_set_is_written_where_it_is_used() {
    for (set, r) in [("{ p | p distance(9) c }", 9.0), ("std.Sphere(c, r: 12)", 12.0)] {
        let e = read(&format!("{SPACE}q coincident {set}\n"));
        let sk = solved(&e);
        let got = norm(sub(at(&sk, &e, "q"), at(&sk, &e, "c")));
        assert!((got - r).abs() < 1e-9, "{set}: {got}");
        let s = said(&e);
        assert!(s.contains(&format!("q coincident {set}")), "{s:?}");
    }
    // and a tangency to a family written in place
    let e = read(&format!("{SPACE}l tangent std.Cylinder(bx, r: 12)\n"));
    let sk = solved(&e);
    let (p, d) = ends(&sk, ent(&e, "bx"));
    let (a, b) = ends(&sk, ent(&e, "l"));
    let gap = dot(unit(cross(sub(d, p), sub(b, a))), sub(a, p)).abs();
    assert!((gap - 12.0).abs() < 1e-9, "{gap}");
}

/// **A word inside a set, and a set inside a word**: a set whose body writes `near`, and a word
/// whose body puts its operand on a sphere — both applied by the one routine, read as the
/// statement the drawing wrote.
#[test]
fn words_and_sets_apply_inside_each_other() {
    let e = read(&format!("{SPACE}shell := {{ p | p near(d: 11) c }}\nq coincident shell\n"));
    let sk = solved(&e);
    assert!((norm(sub(at(&sk, &e, "q"), at(&sk, &e, "c"))) - 11.0).abs() < 1e-9);
    assert!(said(&e).contains(&"q coincident shell".to_string()), "{:?}", said(&e));
    let e = read(&format!(
        "a orbits(r) o := a coincident std.Sphere(o, r: r)\n{SPACE}q orbits(r: 13) c\n"
    ));
    let sk = solved(&e);
    assert!((norm(sub(at(&sk, &e, "q"), at(&sk, &e, "c"))) - 13.0).abs() < 1e-9);
    assert!(said(&e).contains(&"q orbits(r: 13) c".to_string()), "{:?}", said(&e));
}

/// **A tangency linearises through a word**: a set whose body is `near` is a sphere, and a line
/// tangent to it passes the centre at the radius.
#[test]
fn a_tangency_linearises_through_a_word() {
    let e = read(&format!("{SPACE}shell := {{ p | p near(d: 12) c }}\nl tangent shell\n"));
    assert!(e.sketch.constraints.iter().any(|c| c.along.is_some()));
    let sk = solved(&e);
    let (a, b) = ends(&sk, ent(&e, "l"));
    let got = off_line(at(&sk, &e, "c"), a, sub(b, a));
    assert!((got - 12.0).abs() < 1e-9, "{got}");
}
