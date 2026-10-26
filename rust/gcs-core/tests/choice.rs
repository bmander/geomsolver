//! **A choice of number**, `if(c, a, b)`: `a` where the condition holds and `b` where it does not
//! — a value, where `when` is a block of statements, so a number that differs from copy to copy
//! of a block is written as the one number it is.  The condition is `when`'s, decided from
//! numbers alone and never over an unknown of the drawing; both sides are worked out and agree
//! in dimension.  A curve's tape runs no branch, so there a choice is decided as it compiles.

use crate::common::linked::build as elaborated;
use gcs_core::constraints::CKind;
use gcs_core::expr::{self, eval, parse, Aff};
use gcs_core::program::{Code, Elaborated};
use gcs_core::tape::Tape;
use gcs_core::units::Dim;
use std::collections::BTreeMap;

fn ev(text: &str) -> Result<Aff, String> {
    let env: BTreeMap<String, Aff> =
        [("n".to_string(), Aff::num(3.0)), ("w".to_string(), Aff::of_dim(40.0, Dim::LENGTH))]
            .into_iter()
            .collect();
    eval(&parse(text)?.body, &env).map_err(String::from)
}

fn num(text: &str) -> f64 {
    ev(text).unwrap_or_else(|e| panic!("{text}: {e}")).number().expect("a number")
}

/// **The side the condition picks**, under every connective a `when` reads, nested, and inside
/// arithmetic like any other call.
#[test]
fn an_if_is_the_side_its_condition_picks() {
    assert_eq!(num("if(n > 2, 10, 20)"), 10.0);
    assert_eq!(num("if(n < 2, 10, 20)"), 20.0);
    assert_eq!(num("if(n == 2 or n == 3, 1, 0)"), 1.0);
    assert_eq!(num("if(n > 0 and not n == 3, 1, 0)"), 0.0);
    assert_eq!(num("if((n + 1) * 2 == 8, 1, 0)"), 1.0);
    assert_eq!(num("if(n > 2, if(n > 5, 1, 2), 3)"), 2.0);
    assert_eq!(num("1 + 2 * if(n != 3, 4, 5)"), 11.0);
    // its dimension is the one its sides agree in
    let v = ev("if(w > 30mm, w, 10mm)").unwrap();
    assert_eq!((v.number(), v.dim), (Some(40.0), Dim::LENGTH));
}

/// **Both sides are worked out**, as both sides of an `or` are: a mistake on the side not taken
/// is still said, and two dimensions are refused whichever is chosen.
#[test]
fn an_if_is_checked_on_both_sides() {
    let e = ev("if(n > 2, 40mm, 20deg)").unwrap_err();
    assert!(e.contains("`if` chooses between Length and Angle"), "{e}");
    let e = ev("if(n > 2, 1, 2mm + 3deg)").unwrap_err();
    assert!(e.contains("cannot be added"), "{e}");
    // two operands, as `+`'s and `max`'s are: a bare side does not take the other's
    let e = ev("if(w > 30mm, w, 10)").unwrap_err();
    assert!(e.contains("between Length and Scalar"), "{e}");
    let e = ev("if(w > 30, 1, 2)").unwrap_err();
    assert!(e.contains("Length cannot be compared with Scalar"), "{e}");
}

/// **The grammar**: a condition first — a number alone is none — then two numbers; `in` and
/// `meets` ask what a name stands for, which is a `when`'s to decide.
#[test]
fn an_if_takes_a_condition_and_two_numbers() {
    for (text, words) in [
        ("if(n, 1, 2)", "expected a comparison"),
        ("if(n > 2, 1)", "`if` takes a condition and two numbers"),
        ("if(n > 2, 1, 2, 3)", "`if` takes a condition and two numbers"),
        ("if(n > 2)", "`if` takes a condition and two numbers"),
        ("if(a in B, 1, 2)", "decided by a `when`"),
        ("if(a meets b, 1, 2)", "decided by a `when`"),
    ] {
        let e = parse(text).unwrap_err();
        assert!(e.contains(words), "{text}: {e}");
    }
    assert_eq!(expr::builtin("if"), Some("a built-in function"));
}

/// **What it reads**: every name of the condition and of both sides is a dependency, and an
/// `if` is affine in a name only where its condition does not read it.
#[test]
fn an_if_reads_its_condition_and_both_sides() {
    let p = parse("if(n > k, a, 2 * b)").unwrap();
    let deps: Vec<String> = p.body.deps().into_iter().collect();
    assert_eq!(deps, ["a", "b", "k", "n"]);
    assert!(p.body.affine_in("a"));
    assert!(!p.body.affine_in("n"));
    // a name a side reads is written in by `map_vars` there and in the condition alike
    let n = |v: &str| (v == "n").then(|| expr::Ast::Num(1.0, Dim::SCALAR));
    assert!(!p.body.map_vars(&n).deps().contains("n"));
}

/// **Never over an unknown**: which side is taken is known before the solve, so a condition
/// reading an unknown is refused, saying which; a side may still be one, scaled and offset.
#[test]
fn an_if_chooses_before_the_solve() {
    let unknown: BTreeMap<String, Aff> = [
        ("t".to_string(), Aff::free("t", Dim::ANGLE)),
        ("k".to_string(), Aff::free("t", Dim::ANGLE).times(2.0)),
    ]
    .into_iter()
    .collect();
    for (text, words) in [
        ("if(t > 1deg, 1, 2)", "`t` is an unknown of the drawing"),
        ("if(k > 1deg, 1, 2)", "`k` reads `t`, an unknown of the drawing"),
    ] {
        let e = eval(&parse(text).unwrap().body, &unknown).unwrap_err();
        assert!(e.message.contains(words), "{text}: {}", e.message);
    }
    let v = eval(&parse("if(1 > 0, 2 * t, t)").unwrap().body, &unknown).unwrap();
    assert_eq!((v.free.as_deref(), v.m, v.c), (Some("t"), 2.0, 0.0));
}

/// **A curve's choice is made as it compiles**: over the numbers the text was written with it is
/// the side it picks, the gradient that side's; over a number the tape is evaluated over it is
/// refused, saying to choose where the component is called.
#[test]
fn a_curve_decides_its_if_as_it_compiles() {
    let vars = vec!["u".to_string()];
    let p = parse("if(pi > 3, 2 * u, u * u)").unwrap();
    let t = Tape::compile(&p.body, &vars).unwrap();
    let mut s = gcs_core::tape::Scratch::new();
    let v = t.eval(&[5.0], &mut s);
    assert_eq!((v.v, v.d[0]), (10.0, 2.0));
    let p = parse("if(u > 180, 1, 2)").unwrap();
    let e = Tape::compile(&p.body, &vars).unwrap_err();
    assert!(e.contains("decided once, as the curve is compiled") && e.contains("`u`"), "{e}");
}

/// What the distances a document states come to, in order.
fn distances(e: &Elaborated) -> Vec<f64> {
    let mut d: Vec<f64> = e.sketch.user_constraints().iter()
        .filter(|c| matches!(c.kind, CKind::Distance | CKind::Distance3))
        .map(|c| c.args.last().expect("a distance has its number").num())
        .collect();
    d.sort_by(f64::total_cmp);
    d
}

/// **A number per copy**, the request it answers: a block's index chooses each copy's number in
/// the one statement, where a `when` per case or arithmetic over the index (`5 + j * 2`) did; a
/// value, a component's formal, a fix's member and a block's count choose the same way.
#[test]
fn a_block_chooses_a_number_per_copy() {
    let e = elaborated("\
use std
n := 3
w := if(n > 2, 40mm, 20mm)
in std.front {
  a := point
  fix((0, if(n >= 3, 0, 1))) a
  b := point
  a distance(w) b
}
repeat if(n > 2, 3, 5) as k {
  q := point
  a distance(if(k == 0, 5, 7 * k)) q
}
component Rim(r: Length, wide: Scalar) {
  edge := if(wide > 0, r + 2mm, r)
  claim edge == 12mm
}
rim := Rim(r: 10mm, wide: 1)
claim w == 40mm
");
    assert!(e.ok(), "{:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    assert_eq!(distances(&e), [5.0, 7.0, 14.0, 40.0]);
    assert_eq!(e.values["w"].value, 40.0);
    assert_eq!(e.value_claims.len(), 2);
    assert!(e.value_claims.iter().all(|c| c.holds), "{:?}", e.value_claims);
}

/// **In a document**, a choice over an unknown is refused where the value is defined, as any
/// value that cannot be worked out is (E103), and a dimension over one keeps its last number.
#[test]
fn a_documents_if_over_an_unknown_is_refused() {
    let e = elaborated("\
use std
param t: Angle hint(30deg)
x := if(t > 1deg, 1, 2)
in std.front {
  a := point
  fix((0, 0)) a
}
");
    let d = e.errors().find(|d| d.code == Code::E103)
        .unwrap_or_else(|| panic!("{:?}", e.errors().collect::<Vec<_>>()));
    assert!(d.message.contains("`t` is an unknown of the drawing, and an `if` chooses before the \
                                solve"), "{}", d.message);
}
