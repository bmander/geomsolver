//! **The body words are words of `std.solids`** (#783 P3): `x cut B`, `x union B` and `x common
//! B` are defined there over the language's primitive — more of an open body's set said where
//! the body is not declared, a conjunct (`B and { p | p not in x }`) or a disjunct (`B or { p | p
//! in x }`) — and imported bare where a file uses them (`use std.solids (cut)`).  Each is the body
//! rule's term (`SolidDef::Body`) as the built-in statement was, so the word, the primitive and
//! the Boolean written as a set build the same solid to the bit.  One solid within another is
//! claimed as what holds of every point of it, `claim every q in a: q in b`.
use gcs_core::edit;
use gcs_core::model::SolidDef;
use gcs_core::program::Elaborated;
use gcs_core::syntax::BodyWord;

use crate::common::{ent, lenient, parsed::prog_of, text::solved};

/// A 60 × 40 rectangle `sec` in the front plane with a hole of radius 5 at its middle, a stock
/// 30 deep behind it, a bore through it and a boss standing 10 in front of it.
const PARTS: &str = "unit mm\nuse std\nuse std.solids (cut, union, common)\nin std.front {\n\
    a := point\n  b := point hint((60, 0))\n  c := point hint((60, 40))\n\
    d := point hint((0, 40))\n\
    horizontal (ab := line(a, b)) ->\n  vertical (bc := line(b, c)) ->\n\
    horizontal (cd := line(c, d)) ->\n  vertical (da := line(d, a)) -> close\n\
    a distance(60) b\n  b distance(40) c\n  fix((0, 0)) a\n  sec := face(ab, bc, cd, da)\n\
    o := point hint((30, 20))\n  a distance(30, along: u) o\n  a distance(20, along: v) o\n\
    hole := circle(center: o) hint(r: 5)\n  radius(5) hole\n}\n\
    stock := solid(std.solids.Prism(sec, from: -30mm, to: 0mm))\n\
    bore := solid(std.solids.Prism(face(hole), from: -40mm, to: 10mm))\n\
    boss := solid(std.solids.Prism(face(hole), from: 0mm, to: 10mm))\n";

fn volume(e: &Elaborated, name: &str) -> f64 {
    let x = e.sketch.exact_solid(ent(e, name).i()).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    gcs_core::brep::props::volume(&x.brep)
}

/// Each word is the primitive it is defined as, and both are the body rule's term: the same
/// definition, so the same solid to the bit.
#[test]
fn a_body_word_is_the_primitive_it_names() {
    let e = solved(&format!("{PARTS}by_word := solid(stock)\nbore cut by_word\nboss union by_word\n\
        joined := solid(stock)\njoined and {{ p | p not in bore }}\njoined or {{ p | p in boss }}\n\
        kept := solid(bore)\nstock common kept\nkept_joined := solid(bore)\n\
        kept_joined and {{ p | p in stock }}\n"));
    for name in ["by_word", "joined"] {
        let SolidDef::Body { on, through, bound, .. } = &e.sketch.solids[ent(&e, name).i()].def
        else { panic!("`{name}` is a body") };
        assert_eq!((on.len(), through.len(), bound.len()), (1, 1, 0), "{name}");
    }
    assert_eq!(volume(&e, "by_word").to_bits(), volume(&e, "joined").to_bits());
    assert_eq!(volume(&e, "kept").to_bits(), volume(&e, "kept_joined").to_bits());
    // stock with the boss in union and the bore cut: the bore eats the boss, union first
    let want = 72_000.0 - std::f64::consts::PI * 25.0 * 30.0;
    assert!((volume(&e, "by_word") - want).abs() < 1e-9 * want, "{}", volume(&e, "by_word"));
}

/// A word is a file's where its `use` names it: unnamed, the use is refused naming the import,
/// and a document may name a thing `cut`, the word no longer being the grammar's.
#[test]
fn a_body_word_is_imported_by_name() {
    let unnamed = PARTS.replace("use std.solids (cut, union, common)", "use std.solids");
    let (_, said) = lenient::read(&format!("{unnamed}body := solid(stock)\nbore cut body\n"));
    assert!(said.iter().any(|m| m.contains("`use std.solids (cut)`")), "{said:?}");
    // the word is a name like any other where the file says nothing of it
    let (e, said) = lenient::read("unit mm\nuse std\ncut := point\nfix((1, 2, 3)) cut\n");
    assert!(e.ok(), "{said:?}");
}

/// The primitive takes one membership of a solid in the set for now, and says so otherwise.
#[test]
fn the_primitive_reads_one_membership() {
    for (statement, why) in [
        ("body and { p | p distance(<= 1mm) o }", "one membership of a solid"),
        ("body or { p | p not in bore }", "one membership of a solid"),
        ("body and { p | p in bore; p in boss }", "one membership of a solid"),
    ] {
        let (_, said) = lenient::read(&format!("{PARTS}body := solid(stock)\n{statement}\n"));
        assert!(said.iter().any(|m| m.contains(why)), "`{statement}`: {said:?}");
    }
}

/// The Boolean gesture writes the word and names it in the file's `use std.solids`: added to a
/// list the use has, or a list given to one with none.
#[test]
fn the_gesture_imports_the_word_it_writes() {
    let bare = PARTS.replace("use std.solids (cut, union, common)", "use std.solids");
    let src = format!("{bare}body := solid(stock)\n");
    let out = edit::add_body_word(&prog_of(&src), BodyWord::Cut, &["bore"], "body");
    assert_eq!(out.refused, None);
    assert!(out.text.contains("use std.solids (cut)\n") && out.text.contains("bore cut body"),
            "{}", out.text);
    let listed = PARTS.replace("(cut, union, common)", "(union)");
    let src = format!("{listed}body := solid(stock)\n");
    let out = edit::add_body_word(&prog_of(&src), BodyWord::Common, &["bore"], "body");
    assert_eq!(out.refused, None);
    assert!(out.text.contains("use std.solids (union, common)\n"), "{}", out.text);
}

/// One solid within another is what holds of every point of it: `claim every q in a: q in b`,
/// judged as the solid claims are, and the claim word it was refused naming it.
#[test]
fn every_point_of_a_solid_within_another_is_claimed() {
    let (mut e, said) = lenient::read(&format!("{PARTS}claim every q in boss: q in bore\n\
        claim every q in bore: q in boss\n"));
    assert!(e.ok(), "{said:?}");
    // and the report says it as it is written
    gcs_core::solve::solve(&mut e.sketch, Default::default());
    let v = gcs_core::diagnose::judge_solids(&e.sketch);
    let text = gcs_core::report::solid_claim_text(&v[0]);
    assert!(text.starts_with("claim every q in boss: q in bore — "), "{text}");
    // the printer writes it back as written, so a statement the app writes parses to what it read
    let (prog, _) = crate::common::parse("claim every q in boss: q in bore\n");
    let mut text = String::new();
    gcs_core::syntax::write_stmt_to(&mut text, &prog.root().body[0].kind).unwrap();
    assert_eq!(text, "claim every q in boss: q in bore");
    let (_, said) = lenient::read(&format!("{PARTS}claim boss inside bore\n"));
    assert!(said.iter().any(|m| m.contains("`claim every q in a: q in b`")), "{said:?}");
    let (_, said) = lenient::read(&format!("{PARTS}every q in boss: q in bore\n"));
    assert!(said.iter().any(|m| m.contains("in a claim")), "{said:?}");
}
