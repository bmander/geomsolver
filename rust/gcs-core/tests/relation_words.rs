//! **Relation words, and `use M (names)`** (Solvent §9.9, §14.4 [0.48]; issue #94): a word a
//! file defines is its body with the operands and parameters put in, closed over them, reported
//! where it is written and described as written; a `use` reads the names it lists bare.  The
//! standard library's `horizontal` and `vertical` between points are such words.
use gcs_core::constraints::CKind;
use gcs_core::io;
use gcs_core::modules::link;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::parse;
use std::collections::BTreeMap;

use crate::common::{ent, read, refused};

/// Where the point the document calls `n` stands once solved.
fn solved_at(e: &Elaborated, names: &[&str]) -> Vec<[f64; 3]> {
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    names.iter().map(|n| sk.world_point(ent(e, n).i())).collect()
}

/// Each statement as `describe` reads it back, in the document's names.
fn said(e: &Elaborated) -> Vec<String> {
    e.sketch
        .user_constraints()
        .iter()
        .map(|c| io::describe_with(c, &|r| e.map.name_of(r).cloned()))
        .collect()
}

fn near(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() < 1e-9)
}

const POINTS: &str = "\
in std.front {
  p := point hint((0, 0))
  q := point hint((10, 3))
  r := point hint((12, 9))
  fix((0, 0)) p
  p distance(10) q
";

#[test]
fn std_horizontal_and_vertical_are_imported_words() {
    let e = read(&format!("use std (horizontal, vertical)\n{POINTS}  p horizontal q\n  q vertical r\n  p distance(14) r\n}}\n"));
    let at = solved_at(&e, &["q", "r"]);
    assert!((at[0][2]).abs() < 1e-9 && (at[0][0] - 10.0).abs() < 1e-9, "{at:?}");
    assert!((at[1][0] - 10.0).abs() < 1e-9, "{at:?}");
    // the constraint is the level its body states, and it is described as written
    let levels: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
    assert_eq!(levels.iter().filter(|k| **k == CKind::Level).count(), 2);
    let s = said(&e);
    assert!(s.contains(&"p horizontal q".to_string()), "{s:?}");
    assert!(s.contains(&"q vertical r".to_string()), "{s:?}");
    // the same statement written in the core's own words reads back in them
    let e = read(&format!("use std\n{POINTS}  p level(up) q\n}}\n"));
    assert!(said(&e).contains(&"p level(up) q".to_string()), "{:?}", said(&e));
}

#[test]
fn a_word_not_imported_is_refused_naming_the_import() {
    refused(
        &format!("use std\n{POINTS}  p horizontal q\n}}\n"),
        "E102",
        "use std (horizontal)",
        "horizontal",
    );
    // `horizontal` of a line is the language's own and needs nothing
    let e = read("use std\nin std.front {\n  a := point\n  l := line(a, hint((5, 1)))\n  horizontal l\n}\n");
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::Horizontal));
    // and a word nothing defines anywhere
    refused(&format!("use std\n{POINTS}  p sideways q\n}}\n"), "E102", "no relation word `sideways`", "sideways");
}

#[test]
fn a_defined_word_with_a_number_reads_its_argument() {
    let src = format!("\
use std (horizontal)
a above(d) b := b distance(d, along: up) a
h := 7
{POINTS}  p horizontal q
  r above(d: h) q
  r level(right) q
}}
");
    let e = read(&src);
    let at = solved_at(&e, &["r"]);
    assert!(near(at[0], [10.0, 0.0, 7.0]), "{at:?}");
    // the dimension is the statement's argument: its callout draws `h`, and `describe` the word
    let c = e.sketch.user_constraints().into_iter().find(|c| c.kind == CKind::Ordinate).unwrap();
    assert_eq!(c.written.as_deref(), Some("h"));
    assert!(said(&e).contains(&"r above(d: h) q".to_string()), "{:?}", said(&e));
    // and editing it edits the argument: a number over a param is the param's line
    let lit = src.replace("r above(d: h) q", "r above(d: 7) q");
    let e = read(&lit);
    let c = e.sketch.user_constraints().into_iter().find(|c| c.kind == CKind::Ordinate).unwrap();
    let edit = gcs_core::edit::set_dimension(&e, &e.program, c.id, "d", "9");
    assert!(edit.refused.is_none(), "{:?}", edit.refused);
    assert!(edit.text.contains("r above(d: 9) q"), "{}", edit.text);
}

#[test]
fn a_word_parameter_stands_where_a_word_does() {
    let e = read(&format!("\
use std
a toward(w) b := a level(w) b
{POINTS}  p toward(w: up) q
  q toward(w: right) r
  p distance(13) r
}}
"));
    let at = solved_at(&e, &["q", "r"]);
    assert!(at[0][2].abs() < 1e-9 && (at[1][0] - 10.0).abs() < 1e-9, "{at:?}");
}

#[test]
fn a_prefix_word_and_a_word_in_terms_of_another() {
    let e = read("\
use std
long(d) l := distance(d) l
a above(d) b := b distance(d, along: up) a
a twice(d) b := a above(d: d * 2) b
in std.front {
  p := point hint((0, 0))
  q := point hint((2, 9))
  fix((0, 0)) p
  l := line(p, q)
  long(d: 10) l
  q twice(d: 3) p
}
");
    let at = solved_at(&e, &["q"]);
    assert!((at[0][2] - 6.0).abs() < 1e-9 && (at[0][0] - 8.0).abs() < 1e-9, "{at:?}");
    let s = said(&e);
    assert!(s.contains(&"long(d: 10) l".to_string()) && s.contains(&"q twice(d: 3) p".to_string()), "{s:?}");
}

#[test]
fn a_claim_keeps_its_word() {
    let e = read(&format!("use std (horizontal)\n{POINTS}  p distance(0, along: y) q\n}}\n")
        .replace("p distance(0, along: y) q", "p level(up) q\n  claim p horizontal q"));
    assert!(said(&e).contains(&"claim p horizontal q".to_string()), "{:?}", said(&e));
}

#[test]
fn the_arguments_are_given_by_label() {
    let src = |stmt: &str| format!("use std\na above(d) b := b distance(d, along: up) a\n{POINTS}  {stmt}\n}}\n");
    refused(&src("r above(5) q"), "E004", "by label: `above(d: 5)`", "5");
    refused(&src("r above(e: 5) q"), "E040", "has no parameter `e`", "e");
    refused(&src("r above q"), "E040", "needs `d`", "above");
}

#[test]
fn an_error_inside_the_expansion_is_said_at_the_word() {
    // the body's relation does not relate a point and a line: said where `above` is written
    refused(
        "use std\na above(d) b := b distance(d, along: up) a\nin std.front {\n  p := point\n  l := line\n  p above(d: 3) l\n}\n",
        "E040",
        "",
        "above",
    );
}

#[test]
fn an_operand_that_names_nothing_is_said_once() {
    let (prog, _, _) = gcs_core::library::parse_linked(&format!("use std (horizontal)\n{POINTS}  p horizontal nobody\n}}\n"));
    let e = elaborate(&prog);
    let said: Vec<&str> = e.errors().filter(|d| d.code.as_str() == "E101").map(|d| d.span.slice(prog.text())).collect();
    assert_eq!(said, ["nobody"], "{:?}", e.diags);
}

#[test]
fn a_definition_is_checked_where_it_is_written() {
    let src = |def: &str| format!("use std\n{def}\n{POINTS}}}\n");
    // closed over its operands and parameters
    refused(&src("flat l := l perpendicular z"), "E101", "not an operand or a parameter of `flat`", "z");
    refused(&src("a far b := a distance(w) b"), "E101", "`w` is not an operand", "w");
    // a word of the language's own, of the same fixity; a keyword whatever the fixity
    refused(&src("a level b := a coincident b"), "E071", "already a constraint word", "level");
    refused(&src("radius l := l coincident l"), "E071", "already a constraint word", "radius");
    refused(&src("a line b := a coincident b"), "E071", "a word of the language's own", "line");
    // twice in one file
    refused(&src("a same b := a coincident b\na same b := a coincident b"), "E071", "defined twice", "same");
    // a parameter read as a number and as a word
    refused(&src("a odd(s) b := a distance(s, along: s) b"), "E040", "as a number and as a word", "s");
    // defined in terms of itself: said where it is written
    refused(
        &format!("use std\na loop b := a loop b\n{POINTS}  p loop q\n}}\n"),
        "E003",
        "in terms of itself",
        "loop",
    );
}

#[test]
fn a_word_is_defined_at_the_top_of_a_file() {
    let (_, errs) = parse("component C(a: point, b: point) {\n  a same b := a coincident b\n}\n");
    assert!(errs.iter().any(|e| e.message.contains("at the top of a file")), "{errs:?}");
    let (_, errs) = parse("a same b := a coincident b class shown\n");
    assert!(errs.iter().any(|e| e.message.contains("the relation alone")), "{errs:?}");
}

/* -- imports ------------------------------------------------------------------------------- */

fn shelf() -> BTreeMap<&'static str, &'static str> {
    let mut m = BTreeMap::new();
    m.insert("lib.words", "\
a above(d) b := b distance(d, along: up) a
gap := 4
dims := {w: 6, h: 2}
component Rung(a: point, b: point, w: Length) {
  a above(d: w) b
}
");
    m.insert("lib.other", "gap := 9\ncomponent Rung(a: point) { }\n");
    // a module importing a word for itself: its importer has not
    m.insert("lib.uses", "use std (horizontal)\ncomponent Flat(a: point, b: point) {\n  a horizontal b\n}\n");
    m
}

fn linked(src: &str) -> (Elaborated, Vec<gcs_core::program::Diag>) {
    let (mut prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let shelf = shelf();
    let diags = link(&mut prog, &mut |name| {
        shelf.get(name).map(|t| t.to_string()).or_else(|| gcs_core::library::resolve(name))
    });
    (elaborate(&prog), diags)
}

fn ok(src: &str) -> Elaborated {
    let (e, diags) = linked(src);
    assert!(diags.is_empty(), "{diags:?}");
    let saw: Vec<String> = e.errors().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect();
    assert!(e.ok(), "{saw:?}\n{src}");
    e
}

fn errs(src: &str) -> Vec<(String, String, String)> {
    let (e, diags) = linked(src);
    e.diags
        .iter()
        .chain(&diags)
        .filter(|d| d.code.as_str().starts_with('E'))
        .map(|d| (d.code.as_str().to_string(), d.message.clone(), d.span.slice(e.program.text()).to_string()))
        .collect()
}

fn has(src: &str, code: &str, needle: &str, at: &str) {
    let saw = errs(src);
    assert!(
        saw.iter().any(|(c, m, s)| c == code && m.contains(needle) && s == at),
        "expected {code} `{needle}` at `{at}`\n{src}\n{saw:#?}"
    );
}

const BODY: &str = "in std.front {\n  p := point hint((0, 0))\n  q := point hint((3, 5))\n  fix((0, 0)) p\n";

#[test]
fn any_name_a_module_defines_may_be_imported() {
    // a word, a component, a value and a group, bare beside their full paths
    let e = ok(&format!("\
use std
use lib.words (above, Rung, gap, dims)
w := gap + dims.w + lib.words.dims.h
{BODY}  q above(d: lib.words.gap) p
  q distance(w, along: x) p
}}
"));
    let at = solved_at(&e, &["q"]);
    assert!(near(at[0], [-12.0, 0.0, 4.0]), "{at:?}");
    let e = ok(&format!("use std\nuse lib.words (Rung)\n{BODY}  r := Rung(q, p, w: 2)\n  q level(right) p\n}}\n"));
    let at = solved_at(&e, &["q"]);
    assert!(near(at[0], [0.0, 0.0, 2.0]), "{at:?}");
    // the full path still works with nothing imported, for a component and a value
    ok(&format!("use std\nuse lib.words\n{BODY}  r := lib.words.Rung(q, p, w: lib.words.gap)\n}}\n"));
}

#[test]
fn an_import_is_per_file_and_there_is_no_prelude() {
    // `lib.uses` imports `horizontal` for its own body; the document has not
    ok(&format!("use std\nuse lib.uses\n{BODY}  f := lib.uses.Flat(p, q)\n}}\n"));
    has(&format!("use std\nuse lib.uses\n{BODY}  p horizontal q\n}}\n"), "E102", "use std (horizontal)", "horizontal");
    // a word is only ever bare: imported, or not written
    has(&format!("use std\nuse lib.words\n{BODY}  q above(d: 1) p\n}}\n"), "E102", "use lib.words (above)", "above");
}

#[test]
fn an_import_is_refused_where_it_is_written() {
    has(&format!("use std\nuse lib.words (nothing)\n{BODY}}}\n"), "E101", "`lib.words` defines no `nothing`", "nothing");
    has(&format!("use std\nuse lib.words (gap)\nuse lib.other (gap)\n{BODY}}}\n"), "E071", "from `lib.words` and from `lib.other`", "gap");
    has(&format!("use std\nuse lib.words (gap, gap)\n{BODY}}}\n"), "E071", "imported twice", "gap");
    has(&format!("use std\nuse lib.words (gap)\ngap := 3\n{BODY}}}\n"), "E071", "defined in this file", "gap");
    has(&format!("use std\nuse lib.words (Rung)\ncomponent Rung(a: point) {{ }}\n{BODY}}}\n"), "E071", "defined in this file", "Rung");
    has(&format!("use std (horizontal)\na horizontal b := a coincident b\n{BODY}}}\n"), "E071", "defined in this file", "horizontal");
}

#[test]
fn a_built_in_word_is_imported_by_no_one() {
    let mut shelf = shelf();
    shelf.insert("lib.pi", "pi := 3\n");
    let (mut prog, _) = parse("use lib.pi (pi)\n");
    link(&mut prog, &mut |n| shelf.get(n).map(|t| t.to_string()));
    let e = elaborate(&prog);
    assert!(
        e.errors().any(|d| d.code.as_str() == "E071" && d.message.contains("word of the language's own")),
        "{:?}",
        e.diags
    );
}

#[test]
fn a_module_reads_its_own_words_bare() {
    // `std` writes `horizontal` in its own components with no import of its own
    let std = gcs_core::library::module("std").unwrap();
    assert!(std.contains("a horizontal b := a level(up) b"));
    assert!(std.contains("k.center horizontal q"));
    // `lib.words`' `Rung` writes `above` bare, and a file importing only `Rung` draws it
    ok(&format!("use std\nuse lib.words (Rung)\n{BODY}  r := Rung(q, p, w: 2)\n}}\n"));
}
