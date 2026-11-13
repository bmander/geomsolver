//! **A relation word a document names an element with is that element at a line's end** (#904,
//! shaft_drive README finding 21).  A line ending in a joint word continues its chain onto the next
//! (§6.6), so `std.origin distance(>= 0mm, along: u) level`, `level` a point the document declared,
//! read `level` as a second word and the next line's first name as its operand: "more on this line
//! than the statement wanted" there, and "no such entity" for what that line declared.  Bare and
//! last on its line after another joint's word, a declared name is the joint's right operand, and
//! the colouring reads it so; a word the document does not declare still continues its chain.
use crate::common::{ent, read};
use gcs_core::constraints::CKind;
use gcs_core::syntax::{highlight, Tint};

const LEVEL: &str = "unit mm
use std
in std.front {
  level := point
  std.origin distance(>= 0mm, along: u) level
  a := point
  std.origin distance(10mm, along: u) a
}
";

#[test]
fn a_point_named_level_ends_the_line_that_relates_it() {
    let e = read(LEVEL);
    // the point the next line declares is there, and each line states its one relation
    let _ = (ent(&e, "a"), ent(&e, "level"));
    assert_eq!(e.sketch.user_constraints().len(), 2);
}

#[test]
fn the_colouring_reads_the_name_as_the_parser_does() {
    // the second `level` (the operand) is no relation; the word between two names still is one
    let runs = highlight(LEVEL);
    let at = LEVEL.find("u) level").unwrap() + 3;
    let tint = runs.iter().find(|(_, s)| s.lo as usize == at).map(|(t, _)| *t);
    assert_ne!(tint, Some(Tint::Relation));
    let src = "unit mm\nuse std\nin std.front {\n  a := point\n  b := point\n  a level b\n}\n";
    let at = src.find("level b").unwrap();
    let tint = highlight(src).into_iter().find(|(_, s)| s.lo as usize == at).map(|(t, _)| t);
    assert_eq!(tint, Some(Tint::Relation));
}

#[test]
fn a_word_no_name_takes_still_continues_its_chain() {
    // `equal` ends the first line, after `parallel`, and names nothing: the chain runs on to `m`,
    // one statement stating both
    let e = read("unit mm\nuse std\nin std.front {\n  l := line\n  m := line\n  l parallel equal\n    m\n}\n");
    let kinds: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
    assert_eq!(kinds.len(), 2, "{kinds:?}");
}
