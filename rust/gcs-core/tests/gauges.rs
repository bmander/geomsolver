//! The gauges and the orientation predicates are entries of the operator table (issue #47,
//! item 5): `ground p`, `fix c.r`, `ccw(a, b, c)` are read by the one relation parser and
//! settled by the one table, so a class, a placement and a `claim` reach them syntactically —
//! and they are *applied* rather than added, holding parameters or recording a root choice,
//! with no constraint the sketch holds to show for it.

use gcs_core::constraints::{gauge_op, is_operator, CKind, Fixity, ALL_KINDS};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::{highlight, parse, StmtKind, Tint};

fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    elaborate(&prog)
}

fn messages(e: &Elaborated) -> Vec<String> {
    e.diags.iter().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect()
}

const TRI: &str = "
point a hint(x: 0, y: 0)
point b hint(x: 10, y: 0)
point c hint(x: 0, y: 10)
circle k(center: a) hint(r: 5)
";

/// The four are operator words, written as relations, and none of them is a constraint the
/// sketch holds: the registry never learns them.
#[test]
fn the_four_are_operators_and_none_is_in_the_registry() {
    for w in ["ground", "fix", "ccw", "cw"] {
        assert!(is_operator(w), "{w}");
        let k = gauge_op(w).expect(w);
        assert!(k.gauge());
        assert!(!ALL_KINDS.contains(&k), "{w} is applied, never published");
        assert!(!k.claimable());
    }
    assert_eq!(CKind::Ccw.operator(), Some(("ccw", Fixity::Call)));
    assert_eq!(CKind::Ground.operator(), Some(("ground", Fixity::Prefix)));
    let (prog, errs) = parse(&format!("{TRI}ground a\nfix k.r\nccw(a, b, c)\n"));
    assert!(errs.is_empty(), "{errs:?}");
    let rels = prog.root().body.iter().filter(|s| matches!(s.kind, StmtKind::Relation(_))).count();
    assert_eq!(rels, 3, "each is an ordinary relation statement");
}

/// Applied, not added: the point is held, the radius is held, the root choice is recorded, and
/// the sketch holds no constraint for any of it.
#[test]
fn applied_and_not_added() {
    let e = read(&format!("{TRI}ground a\nfix k.r\ncw(a, b, c)\n"));
    assert!(e.ok(), "{:?}", messages(&e));
    let sk = &e.sketch;
    assert!(sk.point_fixed(0));
    assert!(sk.params[sk.circles[0].radius as usize].fixed);
    assert_eq!(sk.branches.len(), 1);
    assert_eq!(sk.branches.values().next().copied(), Some(-1));
    assert!(sk.user_constraints().is_empty(), "{:?}", sk.user_constraints());
}

/// The trailing clauses every relation takes are read on a gauge too — a class is inert, since
/// nothing is drawn for it — and a claim is refused with the reason.
#[test]
fn a_class_is_read_and_a_claim_is_refused() {
    let e = read(&format!("{TRI}ground a class held\nccw(a, b, c) class chosen\n"));
    assert!(e.ok(), "{:?}", messages(&e));
    assert!(e.sketch.point_fixed(0));
    let e = read(&format!("{TRI}claim ground a\n"));
    let m = messages(&e);
    assert!(m.iter().any(|m| m.starts_with("E040") && m.contains("ground")), "{m:?}");
    assert!(!e.sketch.point_fixed(0), "a refused claim holds nothing");
}

/// The words the gauges always used for what they refuse.
#[test]
fn the_refusals_keep_their_words() {
    let e = read(&format!("{TRI}ground k\n"));
    assert!(messages(&e).iter().any(|m| m.contains("ground pins a point")), "{:?}", messages(&e));
    let e = read(&format!("{TRI}fix k.q\n"));
    assert!(messages(&e).iter().any(|m| m.contains("has r, not `q`")), "{:?}", messages(&e));
    let e = read(&format!("{TRI}ccw(a, b)\n"));
    assert!(messages(&e).iter().any(|m| m.contains("three points")), "{:?}", messages(&e));
    let e = read(&format!("{TRI}ccw(a, b, k)\n"));
    assert!(messages(&e).iter().any(|m| m.contains("no such point")), "{:?}", messages(&e));
}

/// The lift writes them back as the same operators, and the colouring reads all four as the
/// relation words they are.
#[test]
fn lifted_and_coloured_as_relations() {
    let e = read(&format!("{TRI}ground a\nfix k.r\nccw(a, b, c)\n"));
    let mut p = gcs_core::program::to_program(&e.sketch);
    let text = gcs_core::syntax::render_flat(&mut p).unwrap().to_string();
    assert!(text.contains("ground p0"), "{text}");
    assert!(text.contains("fix c0.r"), "{text}");
    // the key canonicalises the triple's order and keeps its sense
    assert!(text.contains("ccw(p2, p1, p0)"), "{text}");
    let src = format!("{TRI}ground a\nccw(a, b, c)\n");
    let runs = highlight(&src);
    for w in ["ground", "ccw"] {
        let at = src.find(&format!("\n{w}")).unwrap() + 1;
        let tint = runs.iter().find(|(_, s)| s.lo as usize == at).map(|(t, _)| *t);
        assert_eq!(tint, Some(Tint::Relation), "{w}");
    }
}
