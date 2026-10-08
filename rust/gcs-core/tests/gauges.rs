//! The gauges and the orientation predicates are entries of the operator table (issue #47,
//! item 5): `fix((0, 0)) p`, `fix(r == 25) c`, `ccw(a, b, c)` are read by the one
//! relation parser and settled by the one table, so a class, a placement and a `claim` reach
//! them syntactically — and they are *applied* rather than added, holding parameters or
//! recording a root choice, with no constraint the sketch holds to show for it.
//!
//! **A `fix` states what it holds**: each number pinned under its field's name, the way any pin
//! is written in a relation's parentheses (`t == 0.4`), so the held numbers are never seeds in a
//! `hint(…)` clause, which is what a solve revises.

use gcs_core::constraints::{gauge_op, is_operator, CKind, Fixity, ALL_KINDS};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::{highlight, StmtKind, Tint};
use crate::common::parse_legacy as parse;

fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    elaborate(&prog)
}

fn messages(e: &Elaborated) -> Vec<String> {
    e.diags.iter().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect()
}

fn xy(e: &Elaborated, name: &str) -> (f64, f64) {
    let p = e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`"));
    e.sketch.point_xy(p.i())
}

const TRI: &str = "\
use std

in std.front {
a := point
b := point hint((10, 0))
c := point hint((0, 10))
k := circle(center: a) hint(r: 5)
}
";

/// The three are operator words, written as relations, and none of them is a constraint the
/// sketch holds: the registry never learns them.  `ground` is no word at all.
#[test]
fn the_three_are_operators_and_none_is_in_the_registry() {
    for w in ["fix", "ccw", "cw"] {
        assert!(is_operator(w), "{w}");
        let k = gauge_op(w).expect(w);
        assert!(k.gauge());
        assert!(!ALL_KINDS.contains(&k), "{w} is applied, never published");
        assert!(!k.claimable());
    }
    assert!(!is_operator("ground") && gauge_op("ground").is_none());
    assert_eq!(CKind::Ccw.operator(), Some(("ccw", Fixity::Call)));
    assert_eq!(CKind::Fix.operator(), Some(("fix", Fixity::Prefix)));
    let (prog, errs) = parse(&format!("{TRI}fix((0, 0)) a\nfix(r == 5) k\nccw(a, b, c)\n"));
    assert!(errs.is_empty(), "{errs:?}");
    let rels = prog.root().body.iter().filter(|s| matches!(s.kind, StmtKind::Relation(_))).count();
    assert_eq!(rels, 3, "each is an ordinary relation statement");
}

/// Applied, not added: the point is held where the statement says, the radius at what it says,
/// the root choice is recorded, and the sketch holds no constraint for any of it.
#[test]
fn applied_at_the_numbers_stated_and_not_added() {
    let e = read(&format!("{TRI}fix((3, -4)) a\nfix(r == 7) k\ncw(a, b, c)\n"));
    assert!(e.ok(), "{:?}", messages(&e));
    let sk = &e.sketch;
    assert!(sk.point_fixed(0));
    assert_eq!(xy(&e, "a"), (3., -4.));
    let r = &sk.params[sk.circles[0].radius as usize];
    assert!(r.fixed && r.value == 7.);
    assert_eq!(sk.branches.len(), 1);
    assert_eq!(sk.branches.values().next().copied(), Some(-1));
    assert!(sk.user_constraints().is_empty(), "{:?}", sk.user_constraints());
}

/// A fix may hold one coordinate and leave the other free, and what it holds is the number it
/// states, whatever a hint on the declaration says: a hint is only where a solve begins.
#[test]
fn a_fix_holds_what_it_names_and_beats_a_hint() {
    let e = read("unit mm\nuse std\nhalf := 10mm\nin std.front {\na := point hint((3, 7))\nfix(x == -half) a\n}\n");
    assert!(e.ok(), "{:?}", messages(&e));
    let p = &e.sketch.points[0];
    assert!(e.sketch.params[p.x as usize].fixed && !e.sketch.params[p.y as usize].fixed);
    assert_eq!(xy(&e, "a"), (-10., 7.), "x held at the expression, y left at its seed");
}

/// Held before the seeds that read geometry are worked out: a place reading a held point reads
/// where it is held, and a seed that reads geometry never moves a held number.
#[test]
fn a_seed_reads_a_held_point_and_never_moves_one() {
    let e = read("\
use std
in std.front {
a := point
fix((4, 5)) a
q := point hint(at: a)
b := point hint(at: q)
fix(y == -1) b
}
");
    assert!(e.ok(), "{:?}", messages(&e));
    assert_eq!(xy(&e, "q"), (4., 5.), "q seeded from a's held place");
    assert_eq!(xy(&e, "b"), (4., -1.), "b takes q's x, and its own held y");
}

/// The trailing clauses every relation takes are read on a gauge too — a class is inert, since
/// nothing is drawn for it — and a claim is refused with the reason.
#[test]
fn a_class_is_read_and_a_claim_is_refused() {
    let e = read(&format!("{TRI}fix((0, 0)) a class held\nccw(a, b, c) class chosen\n"));
    assert!(e.ok(), "{:?}", messages(&e));
    assert!(e.sketch.point_fixed(0));
    let e = read(&format!("{TRI}claim fix((0, 0)) a\n"));
    let m = messages(&e);
    assert!(m.iter().any(|m| m.starts_with("E040") && m.contains("fix")), "{m:?}");
    assert!(!e.sketch.point_fixed(0), "a refused claim holds nothing");
}

/// A fix says what it holds, by name, pinned — and every other spelling is refused where it
/// stands, saying how it is written.
#[test]
fn a_fix_that_does_not_state_its_numbers_is_refused() {
    for (stmt, says) in [
        ("fix a", "`fix` states the numbers it holds"),
        ("fix(x: 0) a", "`fix` pins a number with `==`"),
        ("fix(5) k", "`fix(r == 5) k`"),
        ("fix(w == 1) a", "or one component, `dir.x` — not `w`"),
        ("fix(px == 1) a", "not `px`"),
        ("fix((1, 2, 3)) a", "a point in a plane has two coordinates: `(x, y)`"),
        ("fix(dir == (1, 0, 0)) a", "a point has x and y, not `dir`"),
        ("fix(dir.x == 1) a", "a point has x and y, not `dir.x`"),
        ("fix((1, 2)) k", "a circle is no vector: it has r"),
        ("fix(z == 1) a", "a point has x and y, not `z`"),
        ("fix(x == 1) k", "a circle has r, not `x`"),
        ("fix(r == 1) a", "a point has x and y, not `r`"),
    ] {
        let e = read(&format!("{TRI}{stmt}\n"));
        let m = messages(&e);
        assert!(m.iter().any(|m| m.contains(says)), "{stmt}: {m:?}");
    }
    let e = read(&format!("{TRI}ccw(a, b)\n"));
    assert!(messages(&e).iter().any(|m| m.contains("three points")), "{:?}", messages(&e));
    let e = read(&format!("{TRI}ccw(a, b, k)\n"));
    assert!(messages(&e).iter().any(|m| m.contains("no such point")), "{:?}", messages(&e));
}

/// The lift writes them back as the same operators, holding the same numbers — a partial hold
/// as partial — and the colouring reads them as the relation words they are.
#[test]
fn lifted_with_their_numbers_and_coloured_as_relations() {
    let src = format!("{TRI}fix((2, 3)) a\nfix(y == 0) b\nfix(r == 5) k\nccw(a, b, c)\n");
    let e = read(&src);
    assert!(e.ok(), "{:?}", messages(&e));
    let mut p = gcs_core::program::to_program(&e.sketch);
    let text = gcs_core::syntax::render_flat(&mut p).unwrap().to_string();
    assert!(text.contains("fix((2, 3)) p0"), "{text}");
    assert!(text.contains("fix(y == 0) p1"), "{text}");
    assert!(text.contains("fix(r == 5) c0"), "{text}");
    // the key canonicalises the triple's order and keeps its sense — and the call is printed
    // in that order: printed back to front, `ccw(p2, p1, p0)` was the other turn, and the lifted
    // text read back chose the other root
    assert!(text.contains("ccw(p0, p1, p2)"), "{text}");
    let again = read(&text);
    assert!(again.ok(), "{:?}", messages(&again));
    assert_eq!(again.sketch.branches, e.sketch.branches, "{text}");
    let held = |e: &Elaborated| {
        e.sketch.params.iter().map(|p| (p.fixed, p.fixed.then_some(p.value))).collect::<Vec<_>>()
    };
    assert_eq!(held(&again), held(&e), "{text}");
    let src = format!("{TRI}fix((0, 0)) a\nccw(a, b, c)\n");
    let runs = highlight(&src);
    for w in ["fix", "ccw"] {
        let at = src.find(&format!("\n{w}")).unwrap() + 1;
        let tint = runs.iter().find(|(_, s)| s.lo as usize == at).map(|(t, _)| *t);
        assert_eq!(tint, Some(Tint::Relation), "{w}");
    }
}

/// A solve's writeback leaves a held number where the `fix` states it: no hint grows back on a
/// held point, a partly held one is seeded only in its free coordinate, and a held child's slot
/// stays empty.
#[test]
fn a_solve_writes_no_seed_for_a_held_number() {
    let src = "\
use std
in std.front {
b := point
fix((30, 0)) b
c := point
fix(x == 3) c
c distance(30) b
l := line
fix((0, 0)) l.p1
l.p1 distance(10) l.p2
}
";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let mut e = elaborate(&prog);
    assert!(e.ok(), "{:?}", messages(&e));
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let edit = gcs_core::edit::commit_seeds(&e, &e.sketch, &prog);
    let text = &edit.text;
    assert!(text.contains("b := point\n"), "{text}");
    assert!(text.contains("c := point hint(y: ") && !text.contains("hint(x: 3"), "{text}");
    assert!(text.contains("l := line(p2: hint("), "{text}");
    let again = read(text);
    assert!(again.ok(), "{:?}", messages(&again));
    assert_eq!(xy(&again, "b"), (30., 0.));
}

/// The app's hold is written as a `fix` with the numbers it holds — a coordinate on its own as a
/// partial one — and taken away again when let go.
#[test]
fn a_hold_made_in_the_app_is_written_with_its_numbers() {
    let src = "use std\nin std.front {\na := point hint((2, 3))\nb := point hint((7, 1))\nfix((7, 1)) b\n}\n";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let mut e = elaborate(&prog);
    let (a, b) = (e.map.ent_named("a").unwrap(), e.map.ent_named("b").unwrap());
    e.sketch.fix_point(b.i(), false);
    let ax = e.sketch.points[a.i()].x as usize;
    e.sketch.params[ax].fixed = true;
    let sk = std::mem::take(&mut e.sketch);
    let edit = gcs_core::edit::reconcile(&mut e, &sk);
    assert!(edit.text.contains("fix(x == 2) a"), "{}", edit.text);
    assert!(!edit.text.contains(") b"), "the hold let go is gone:\n{}", edit.text);
}

/// A number is held once in one `fix` (issue #112): assembled, the slot took the first argument
/// naming it, and a second vector, or a member a vector already gave, went nowhere and said
/// nothing.  Refused at the second, holding nothing.
#[test]
fn a_fix_gives_each_number_once() {
    for (twice, said) in [
        ("fix((3, 4), (1, 1)) a", "`fix` is given its place twice: a point is one vector"),
        ("fix((3, 4), x == 1) a", "`x` is given twice"),
        ("fix(y == 4, (3, 4)) a", "`y` is given twice"),
        ("fix(x == 3, x == 3) a", "`x` is given twice"),
        ("fix(r == 5, r == 6) k", "`r` is given twice"),
    ] {
        let e = read(&format!("{TRI}{twice}\n"));
        let m = messages(&e);
        assert!(m.iter().any(|m| m.starts_with("E040") && m.contains(said)), "{twice}: {m:?}");
        let held = ["a", "k"].iter().flat_map(|n| e.sketch.own_params(e.map.ent_named(n).unwrap()))
            .any(|i| e.sketch.params[i as usize].fixed);
        assert!(!held, "{twice} holds nothing");
    }
}
