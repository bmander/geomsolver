//! `@` is `coincident` and `~` is `hint`: the lexer reads each as the word it stands for, so a
//! document written either way elaborates to the same drawing, colours alike, and takes its
//! solved seeds back into the clause as written.

use gcs_core::edit::{self, Kind};
use gcs_core::io;
use gcs_core::program::elaborate;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::{highlight, Tint};
use crate::common::parse;

const LONG: &str = "\
use std
in std.front {
o := point hint((0, 0))
c := circle(center: o) hint(r: 20)
p := point hint((18, 5))
p coincident c
q := point hint((3, 1))
q coincident o
fix((0, 0)) o
}
";

const SHORT: &str = "\
use std
in std.front {
o := point ~((0, 0))
c := circle(center: o) ~(r: 20)
p := point ~((18, 5))
p @ c
q := point ~((3, 1))
q@o
fix((0, 0)) o
}
";

fn drawing(src: &str) -> String {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.diags.iter().map(|d| &d.message).collect::<Vec<_>>());
    io::dumps(&e.sketch, None)
}

#[test]
fn the_shorthand_is_the_word() {
    assert_eq!(drawing(SHORT), drawing(LONG));
}

#[test]
fn the_shorthand_is_coloured_as_its_word() {
    let tint = |what: &str| {
        let at = SHORT.find(what).unwrap();
        highlight(SHORT).into_iter().find(|(_, s)| s.lo as usize == at).map(|(t, _)| t)
    };
    assert_eq!(tint("@ c"), Some(Tint::Relation));
    assert_eq!(tint("~((18"), Some(Tint::Word));
    assert_eq!(tint("18, 5"), Some(Tint::Seed), "a number inside `~(…)` is a seed");
}

/// A solve writes its seeds inside `~(…)` and leaves the `~` and the `@` as they were written.
#[test]
fn a_solve_writes_back_into_the_shorthand() {
    let (prog, _) = parse(SHORT);
    let mut e = elaborate(&prog);
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    let edit = edit::commit_seeds(&e, &e.sketch, &prog);
    assert_eq!(edit.kind, Kind::Numeric);
    assert!(!edit.text.contains("hint"), "{}", edit.text);
    assert!(edit.text.contains("p @ c\n") && edit.text.contains("q@o\n"), "{}", edit.text);
    assert!(edit.text.contains("q := point ~(("), "{}", edit.text);
    drawing(&edit.text);
}

/// The shorthand stands wherever its word does: `@` takes a pin in its parentheses and `~` seeds
/// the contact's own slot.
#[test]
fn a_pin_and_a_slot_seed_ride_on_the_shorthand() {
    const SPLINE: &str = "\
use std
in std.front {
s0 := point hint((0, 0))
s1 := point hint((20, 10))
s2 := point hint((40, 10))
s3 := point hint((60, 0))
s := spline(s0, s1, s2, s3)
a := point hint((30, 8))
b := point hint((10, 4))
";
    let long = format!("{SPLINE}a coincident(t == 0.4) s\nb coincident s hint(t: 0.2)\n}}\n");
    let short = format!("{SPLINE}a @(t == 0.4) s\nb @ s ~(t: 0.2)\n}}\n");
    assert_eq!(drawing(&short), drawing(&long));
}
