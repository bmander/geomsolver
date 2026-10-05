//! One way to define a name: `NAME := VALUE` (Solvent §5, [0.29]; `docs/definitions-plan.md`).
//!
//! The value says what the name is — a number, an element, a chain joined by `->`, an instance, a
//! group, a curve — and `(NAME := VALUE)` is the value, so a chain link is named where it stands.
//! The old spellings (`param w = 100`, `NAME := Comp(…)`, `NAME := CHAIN`, a name after the
//! keyword) do not parse, and a lone `=` is no token.  `param` before a definition marks one of the
//! document's inputs (§6.3), and a dimension's number defines nothing.

use gcs_core::constraints::{CKind, Constraint};
use gcs_core::edit;
use gcs_core::model::EntRef;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::{write_stmt_to, StmtKind};
use crate::common::parse;

fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}\n{src}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    e
}

fn refuses(src: &str, needle: &str) {
    let (_, errs) = parse(src);
    assert!(
        errs.iter().any(|e| e.message.contains(needle)),
        "expected `{needle}`\n{src}\n{:?}",
        errs.iter().map(|e| &e.message).collect::<Vec<_>>()
    );
}

const PTS: &str = "\
use std
in std.front {
a := point hint(x: 0, y: 0)
b := point hint(x: 10, y: 0)
c := point hint(x: 10, y: 10)
}
";

/// Every kind of value a definition can name, each lowered to the statement it always was.
#[test]
fn the_value_says_what_the_name_is() {
    let src = format!(
        "{PTS}w := 3 * 4\nl := line(a, b)\nk := circle(center: a) hint(r: w)\n\
         dims := {{size: w}}\n\
         component Tick(o: point, dims: group) {{ t := point hint(x: dims.size) }}\n\
         t := Tick(a, dims: dims)\n"
    );
    let e = read(&src);
    let kinds: Vec<&str> = e
        .program
        .root()
        .body
        .iter()
        .map(|s| match &s.kind {
            StmtKind::Param(_) => "param",
            StmtKind::Decl(_) => "decl",
            StmtKind::Group(_) => "group",
            StmtKind::Instance(_) => "instance",
            _ => "other",
        })
        .collect();
    assert_eq!(kinds, ["decl", "decl", "decl", "param", "decl", "decl", "group", "instance"]);
    assert!(e.map.ent_named("l").is_some() && e.map.ent_named("k").is_some());
    assert!(e.map.ent_named("t.t").is_some(), "the instance's members are reached by its name");
}

/// Without a `->` the name goes to the one declaration, prefix words and all — the value of
/// `horizontal line(a, b)` is the line; with one it names the traversal, and a link is named in
/// parentheses because `:=` binds looser than `->`.
#[test]
fn a_chain_names_its_traversal_and_a_link_is_named_in_parentheses() {
    let e = read(&format!("{PTS}in std.front {{\nl := horizontal line(a, b)\n}}\n"));
    assert_eq!(e.map.ent_named("l"), Some(EntRef::line(0)));
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::Horizontal));

    let e = read(&format!("{PTS}in std.front {{\nprofile := (ab := line(a, b)) -> line -> line -> close\n}}\n"));
    assert!(e.map.ent_named("profile").is_some(), "the chain is named");
    assert_eq!(e.map.ent_named("ab"), Some(EntRef::line(0)), "and so is its first link");

    // a prefix word stands outside the parentheses or inside the definition, alike
    let e = read(&format!("{PTS}in std.front {{\nhorizontal (ab := line(a, b)) -> vertical (bc := line) -> close\n}}\n"));
    assert_eq!(e.sketch.lines.len(), 2, "two links, closed at a shared corner");
    assert!(e.map.ent_named("bc").is_some());
}

/// The parentheses of a named link are told from an operator's own by the element keyword after
/// `:=`; anything else there is a dimension's number, which defines nothing.
#[test]
fn a_dimension_defines_no_name() {
    let e = read(&format!("{PTS}w := 10\na distance(w) b\nb distance(w) c\n"));
    assert_eq!(e.sketch.user_constraints().len(), 2);
    let e = read(&format!("{PTS}distance(10) (ab := line(a, b))\n"));
    assert!(e.map.ent_named("ab").is_some());
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::Distance));
    let (prog, errs) = parse(&format!("{PTS}a distance(w := 10) b\n"));
    assert!(!errs.is_empty() || !elaborate(&prog).ok(), "a dimension's number defined a name");
}

/// What is not a definition any more does not parse, and the errors say where.
#[test]
fn the_old_spellings_and_the_misuses_are_refused() {
    // the retired spellings are simply not the grammar: refused, by whatever error comes first
    for old in ["param w = 100\n", "line l(a, b)\n", "w = 3\n", "t: Tooth(a)\n",
                "profile = line -> line\n", "group g(s: 1)\n", "curve k = t.p over u in (0, 1)\n"]
    {
        refuses(old, "");
    }
    refuses("x := line(a, b) perpendicular m\n", "every pair");
    refuses("face := face(a, b)\n", "element keyword");
    refuses("p := point(x: 1)\n", "both `x:` and `y:`");
    refuses("point(x: 1, y: 2)\n", "a computed point is named");
    refuses("w :=\n", "says what the name is");
    // a param may still bear a word a sweep's brackets read as a label
    read("face := 3\nback := face + 1\n");
}

/// A name stated where a number is read is the dimension's name and nothing deeper.
#[test]
fn only_the_outermost_number_is_named() {
    let (p, errs) = parse(&format!("{PTS}a distance(2 * (w := 30)) b\n"));
    let e = elaborate(&p);
    assert!(!errs.is_empty() || !e.ok(), "an inner definition is refused");
}

/// The printers spell what the parser reads, so a printed statement parses back to itself.
#[test]
fn a_printed_definition_reads_back() {
    let src = format!(
        "{PTS}w := 5\nin std.front {{\nl := line(a, b)\nk := circle(center: c) hint(r: 3)\n}}\nd := {{s: w}}\n"
    );
    let e = read(&src);
    let mut out = String::new();
    for st in &e.program.root().body {
        write_stmt_to(&mut out, &st.kind).unwrap();
        out.push('\n');
    }
    for line in ["w := 5", "l := line(a, b)", "k := circle(center: c) hint(r: 3)", "d := {s: w}"]
    {
        assert!(out.contains(line), "`{line}` in\n{out}");
    }
    read(&crate::common::front(&out));
}

/// A name minted for an anonymous link wraps it: `(l0 := line(…))`, so the chain still parses
/// and still threads; a lone declaration takes `l0 := ` in front.
#[test]
fn a_minted_name_wraps_a_link_and_prefixes_a_statement() {
    let mut e = read(&crate::common::front("line -> line\n"));
    e.sketch.add(Constraint::one_line(CKind::Horizontal, EntRef::line(1)));
    let sk = std::mem::take(&mut e.sketch);
    let out = edit::reconcile(&mut e, &sk);
    assert!(out.text.contains(") -> (l0 := line("), "{}", out.text);
    let back = read(&out.text);
    assert_eq!(back.sketch.points.len(), 3 + crate::common::STD_POINTS, "still threaded");

    let mut e = read(&crate::common::front("horizontal line\n"));
    e.sketch.add(Constraint::one_line(CKind::Vertical, EntRef::line(0)));
    let sk = std::mem::take(&mut e.sketch);
    let out = edit::reconcile(&mut e, &sk);
    assert!(out.text.contains("{\nl0 := horizontal line("), "{}", out.text);
}

/// Deleting a definition that a prefix word qualifies takes the whole statement: the word stands
/// inside the declaration's text once the name is written before both.
#[test]
fn deleting_a_prefixed_definition_takes_its_line() {
    let e = read(&format!("{PTS}l := horizontal line(a, b)\nfix(x == 0, y == 0) a\n"));
    let out = edit::remove(&e, &e.program, &e.sketch, &[EntRef::line(0)], &[]);
    assert!(out.refused.is_none(), "{:?}", out.refused);
    assert!(!out.text.contains("horizontal") && !out.text.contains("l :="), "{}", out.text);
    read(&out.text);
}
