//! `ring` (Solvent §12.3–12.6) is **not yet a construct of this implementation** (issue #47,
//! item 3).  Before, the word unrolled into the `cycle` it stood over and a document that wrote
//! it was told on every run (W112) that it had not got one, with two errors (E021, E022) and a
//! mandatory `about` guarding a symmetry nothing held.  Now the parser refuses the word, once,
//! naming the spelling that does what the unrolling did — and this file is the gate for the
//! construct itself, to be rewritten when a ring solves a fundamental domain (§12.4).

use gcs_core::syntax::parse;

const SPOKES: &str = "
point hub hint(x: 0, y: 0)
circle rim(center: hub) hint(r: 40)
ground hub
ring 4 about hub as i {
  point tip hint(x: 40 * cos(90 * i), y: 40 * sin(90 * i))
  hub distance(40) tip
  tip on rim
}
point after hint(x: 1, y: 2)
";

#[test]
fn a_ring_is_refused_once_and_told_the_cycle_it_would_have_been() {
    let (prog, errs) = parse(SPOKES);
    assert_eq!(errs.len(), 1, "one mistake, said once: {errs:?}");
    assert!(errs[0].message.contains("`ring`"), "{}", errs[0].message);
    assert!(errs[0].message.contains("cycle"), "{}", errs[0].message);
    assert_eq!(&SPOKES[errs[0].span.lo as usize..errs[0].span.hi as usize], "ring");
    // the block's body is consumed with the word: nothing inside it leaks out as a loose
    // statement, and the statement after the block is read as usual
    let names: Vec<_> = prog.root().body.iter().filter_map(|s| match &s.kind {
        gcs_core::syntax::StmtKind::Decl(d) => d.name.written().map(|n| n.text.clone()),
        _ => None,
    }).collect();
    assert_eq!(names, ["hub", "rim", "after"], "{names:?}");
}

#[test]
fn the_same_drawing_as_a_cycle_is_a_drawing() {
    let (prog, errs) = parse(&SPOKES.replace("ring 4 about hub", "cycle 4"));
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(), "{:?}", e.diags.iter().map(|d| &d.message).collect::<Vec<_>>());
    assert!(e.diags.is_empty(), "nothing to say about a cycle: {:?}", e.diags);
}
