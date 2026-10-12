//! **`when COND { … }`** (#210, #145 F10): a block made once or not at all, by a comparison of
//! numbers decided at flattening.  It repeats nothing, so it has no index and no prefix: what it
//! declares is named as if written beside it (`q`, `t.tip`, never `#7.0.q`).  The condition reads
//! the numbers in scope, compares only numbers of one dimension (E103), and may not read an
//! unknown of the drawing (E040), since the block is made or not before there is a solve.

use std::collections::BTreeMap;

use crate::common::parse;
use gcs_core::edit::{self, Kind};
use gcs_core::expr::{self, Aff, Fault};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::units::{Dim, Units};

fn read(src: &str) -> (Elaborated, Vec<String>) {
    let (prog, errs) = parse(src);
    let e = elaborate(&prog);
    let mut all: Vec<String> = errs.iter().map(|x| format!("syntax: {}", x.message)).collect();
    all.extend(e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)));
    (e, all)
}

fn point(e: &Elaborated, name: &str) -> Option<usize> {
    e.map.ent_named(name).map(|r| r.i())
}

/// A drawing with one point made where the relief is more than nothing and another where it is
/// not.
fn switched(relief: &str) -> String {
    format!(
        "unit mm\nuse std\nrelief := {relief}\nin std.front {{\n\
         a := point\n\
         fix((0, 0)) a\n\
         when relief > 0mm {{\n\
           q := point hint((8, 3))\n\
           a distance(10mm) q\n\
         }}\n\
         when not relief > 0mm {{\n\
           r := point hint((3, 8))\n\
         }}\n\
         }}\n"
    )
}

/// **Both branches.**  Where the condition holds the body is made once, under the names it
/// wrote; where it does not, nothing of it is made, and nothing is said.
#[test]
fn a_when_is_made_once_or_not_at_all() {
    let (e, d) = read(&switched("0.2mm"));
    assert!(d.is_empty(), "{d:?}");
    let a = point(&e, "a").unwrap();
    let q = point(&e, "q").expect("`q` is made where the relief is more than zero");
    assert!(point(&e, "r").is_none(), "`r` is made only where it is not");
    assert_eq!(e.map.name_of(e.map.ent_named("q").unwrap()).map(String::as_str), Some("q"));
    let mut sk = e.sketch;
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let ((ax, ay), (qx, qy)) = (sk.point_xy(a), sk.point_xy(q));
    assert!(((qx - ax).hypot(qy - ay) - 10.0).abs() < 1e-9);

    let (e, d) = read(&switched("0mm"));
    assert!(d.is_empty(), "{d:?}");
    assert!(point(&e, "q").is_none(), "no relief, no `q`");
    assert!(point(&e, "r").is_some(), "and `r` in its place");
    // nothing the false block holds is a statement of the drawing
    assert!(e.sketch.user_constraints().is_empty(), "{:?}", e.sketch.user_constraints());
}

/// **The names inside are the root's**: a statement outside the block reads what it made by
/// its own name, and a component's block makes `t.tip` — no copy index anywhere.
#[test]
fn the_names_inside_a_when_are_the_enclosing_scopes() {
    let outside = "in std.front {\nb := point hint((0, 12))\nq distance(5mm) b\n}\n";
    let src = format!("{}{outside}", switched("1mm"));
    let (e, d) = read(&src);
    assert!(d.is_empty(), "{d:?}");
    let mut sk = e.sketch;
    assert!(solve(&mut sk, SolveOpts::default()).success);

    let doc = "unit mm\nuse std\n\
        component Tab(p: point, d: Length) {\n\
          when d > 0mm {\n\
            tip := point hint((5, 5))\n\
            p distance(d) tip\n\
          }\n\
        }\n\
        in std.front {\n\
          a := point\n\
          fix((0, 0)) a\n\
          b := point hint((0, 9))\n\
          t := Tab(a, d: 4mm)\n\
          u := Tab(a, d: 0mm)\n\
          t.tip distance(3mm) b\n\
        }\n";
    let (e, d) = read(doc);
    assert!(d.is_empty(), "{d:?}");
    let tip = e.map.ent_named("t.tip").expect("the block in `t` makes `t.tip`");
    assert_eq!(e.map.name_of(tip).map(String::as_str), Some("t.tip"));
    assert!(e.map.ent_named("u.tip").is_none(), "`u`'s depth is zero, so it has no tip");
    let names: Vec<&String> = (0..e.sketch.points.len())
        .filter_map(|i| e.map.name_of(gcs_core::model::EntRef::point(i)))
        .collect();
    assert!(names.iter().all(|n| !n.contains('#')), "a `when` makes no copy: {names:?}");
}

/// A solve writes a point's pose back into a `when` at the root, as into any root statement:
/// the block is made once, so the statement has one pose to record.
#[test]
fn a_solve_writes_back_inside_a_when() {
    let (mut e, d) = read(&switched("0.2mm"));
    assert!(d.is_empty(), "{d:?}");
    let q = point(&e, "q").unwrap();
    let [x, y] = e.sketch.point_params(q);
    e.sketch.params[x as usize].value = 6.0;
    e.sketch.params[y as usize].value = 8.0;
    let edit = edit::commit_seeds(&e, &e.sketch, &e.program);
    assert_eq!(edit.kind, Kind::Numeric);
    assert!(edit.text.contains("q := point hint((6, 8))"), "{}", edit.text);
}

/// **Refused, each with its code**: a condition across dimensions (E103), one reading an unknown
/// of the drawing — a declared one, or a formal no call binds — (E040), one reading a name
/// nothing declares (E101), a measurement of the solved drawing (E107), and a number that is no
/// comparison.  None makes its block.
#[test]
fn a_condition_is_decided_before_the_solve_or_refused() {
    let refused = |src: &str, code: &str, words: &str| {
        let (e, d) = read(src);
        assert!(
            d.iter().any(|m| m.starts_with(code) && m.contains(words)),
            "expected {code} saying {words:?}: {d:?}"
        );
        assert!(point(&e, "q").is_none(), "a refused block makes nothing");
    };
    let block = |cond: &str| {
        format!(
            "unit mm\nuse std\nrelief := 0.2mm\nparam s: Length hint(3mm)\nin std.front {{\n\
             a := point\nfix((0, 0)) a\nl := line(a, hint((5, 0)))\n\
             a distance(s) l.p2\n\
             when {cond} {{\nq := point hint((8, 3))\n}}\n}}\n"
        )
    };
    refused(&block("relief > 1deg"), "E103", "cannot be compared");
    refused(&block("relief > 0mm and 1 > 1deg"), "E103", "cannot be compared");
    refused(&block("s > 0mm"), "E040", "`s` is an unknown of the drawing");
    refused(&block("nothing > 0mm"), "E101", "`nothing` is not a number here");
    refused(&block("length(l) > 0mm"), "E107", "measures the solved drawing");
    refused(&block("relief"), "E103", "expected a comparison");
    // a component's formal left unbound is the instance's unknown
    let doc = "unit mm\nuse std\n\
        component Tab(p: point, d: Length) {\n\
          when d > 0mm {\n\
            tip := point hint((5, 5))\n\
          }\n\
          p distance(d) p2\n\
          p2 := point hint((1, 1))\n\
        }\n\
        in std.front {\n\
          a := point\n\
          fix((0, 0)) a\n\
          t := Tab(a)\n\
        }\n";
    let (e, d) = read(doc);
    assert!(d.iter().any(|m| m.starts_with("E040") && m.contains("`d` reads `t.d`")), "{d:?}");
    assert!(e.map.ent_named("t.tip").is_none());
}

/// What the header may not say: an index, a joint left open for a next copy there is none of,
/// nothing at all, or a chain to run over.
#[test]
fn a_when_has_no_index_and_no_next_copy() {
    let syntax = |src: &str, words: &str| {
        let (_, d) = read(src);
        assert!(
            d.iter().any(|m| m.starts_with("syntax") && m.contains(words)),
            "expected a syntax error saying {words:?}: {d:?}"
        );
    };
    let head = "unit mm\nuse std\nw := 2mm\n";
    syntax(&format!("{head}when w > 0mm as i {{\n}}\n"), "no index");
    syntax(&format!("{head}when {{\n}}\n"), "decided by comparing numbers");
    syntax(&format!("{head}when e in w {{\n}}\n"), "decided by comparing numbers");
    syntax(
        &format!("{head}in std.front {{\nwhen w > 0mm {{\n(s := line) -> perpendicular\n}}\n}}\n"),
        "a `when` has none",
    );
}

/// **The comparisons** (`expr`): six, between numbers of one dimension — two operands, as under
/// `+`, so a length is compared with `0mm` and not with `0` — joined by `and`, `or` and `not`,
/// `or` loosest; a parenthesis may hold a number or a condition.  Exact, no tolerance.
#[test]
fn conditions_compare_numbers_of_one_dimension() {
    let mm = Units::with_length("mm").unwrap();
    let env: BTreeMap<String, Aff> = [
        ("a", Aff::of_dim(2.0, Dim::LENGTH)),
        ("b", Aff::of_dim(3.0, Dim::LENGTH)),
        ("t", Aff::of_dim(30.0, Dim::ANGLE)),
        ("n", Aff::num(4.0)),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_string(), v))
    .collect();
    let decide = |text: &str| {
        let c = expr::parse_condition_in(text, mm).unwrap_or_else(|e| panic!("{text}: {e}"));
        expr::decide(&c, &env)
    };
    for (text, want) in [
        ("a < b", true),
        ("a <= 2mm", true),
        ("a > b", false),
        ("b >= 3mm", true),
        ("a == 2mm", true),
        ("a != 2mm", false),
        ("t > 0deg and t < 45deg", true),
        ("t > 45deg or n == 4", true),
        ("not a < b", false),
        ("not not a < b", true),
        ("a > b or a < b and n > 5", false),
        ("(a > b or a < b) and not n > 5", true),
        ("(a + b) / 2 > 2mm", true),
        ("2 * n - 1 == 7", true),
        ("0.1mm + 0.2mm == 0.3mm", false),
    ] {
        assert_eq!(decide(text), Ok(want), "{text}");
    }
    let err = decide("a < 1deg").unwrap_err();
    assert_eq!(err.fault, Fault::Dimension, "{err:?}");
    assert!(err.message.contains("Length cannot be compared with Angle"), "{err:?}");
    let bare = decide("a > 0").map_err(|e| e.fault);
    assert_eq!(bare, Err(Fault::Dimension), "a bare 0 is no length");
    for bad in ["a", "a < b < 3mm", "a = b", "a < b and", "(a < b", "a ! b"] {
        assert!(expr::parse_condition_in(bad, mm).is_err(), "{bad} is no condition");
    }
    // a comparison is no number: an ordinary expression stops at one
    assert!(expr::parse_in("a < b", mm).is_err());
}
