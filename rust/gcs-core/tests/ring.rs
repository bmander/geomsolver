//! `ring` (Solvent §12.3–12.6), as this implementation has it: **unrolled** into the cycle it
//! stands over, and *said* to be — W112 wherever the DOF ledger is, since the copies are
//! congruent by the numbers each was given and not held so.  The clause that makes it a ring
//! is mandatory, a ring inside a ring is refused (§12.6: may be refused, must not be
//! mis-solved), and a statement inside may reach outside only for what the ring's turn leaves
//! where it is — the axis point, and a circle or an arc centred on it (E021, §12.5).  Before
//! issue #43 the word `ring` had no effect on the drawing or on anything said about it.

use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::parse;

fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    elaborate(&prog)
}

fn codes(e: &Elaborated) -> Vec<(&str, String)> {
    e.diags.iter().map(|d| (d.code.as_str(), d.message.clone())).collect()
}

const SPOKES: &str = "
point hub hint(x: 0, y: 0)
circle rim(center: hub) hint(r: 40)
ground hub
ring 4 about hub as i {
  point tip hint(x: 40 * cos(90 * i), y: 40 * sin(90 * i))
  hub distance(40) tip
  tip on rim
}
";

#[test]
fn a_ring_is_unrolled_and_says_so() {
    let e = read(SPOKES);
    assert!(e.ok(), "{:?}", codes(&e));
    let c = codes(&e);
    assert_eq!(c.len(), 1, "{c:?}");
    assert_eq!(c[0].0, "W112");
    assert!(c[0].1.contains("unrolled"), "{}", c[0].1);
    // the drawing is the cycle's: four spokes, four freedoms
    let cyc = read(&SPOKES.replace("ring 4 about hub", "cycle 4"));
    assert_eq!(e.sketch.points.len(), cyc.sketch.points.len());
    assert_eq!(e.sketch.user_constraints().len(), cyc.sketch.user_constraints().len());
}

#[test]
fn the_axis_is_mandatory() {
    let (_, errs) = parse(&SPOKES.replace("ring 4 about hub", "ring 4"));
    assert_eq!(errs.len(), 1, "one mistake, said once: {errs:?}");
    assert!(errs[0].message.contains("names its axis"), "{}", errs[0].message);
}

#[test]
fn the_axis_and_what_is_centred_on_it_may_be_referenced_and_nothing_else_may() {
    // the axis point and the rim centred on it: fine (above).  A stray point is not.
    let e = read(&SPOKES.replace("  tip on rim\n", "  tip on rim\n  stray distance(30) tip\n")
        .replace("ground hub\n", "ground hub\npoint stray hint(x: 100, y: 5)\n"));
    let c = codes(&e);
    assert!(c.iter().any(|(k, m)| *k == "E021" && m.starts_with("`stray`")), "{c:?}");
    assert!(!e.ok());
    // a circle centred elsewhere is not invariant either
    let e = read(&SPOKES.replace("  tip on rim\n", "  tip on rim\n  tip on other\n")
        .replace("ground hub\n", "ground hub\npoint o2 hint(x: 100, y: 0)\ncircle other(center: o2) hint(r: 90)\n"));
    let c = codes(&e);
    assert!(c.iter().any(|(k, m)| *k == "E021" && m.starts_with("`other`")), "{c:?}");
    // an instance inside the ring reaching the axis through its formal: still the axis
    let e = read(
        "component Spoke(c: point, t: point) { c distance(40) t }
         point hub hint(x: 0, y: 0)
         ground hub
         ring 4 about hub as i {
           point tip hint(x: 40 * cos(90 * i), y: 40 * sin(90 * i))
           s: Spoke(hub, tip)
         }",
    );
    assert!(e.ok(), "{:?}", codes(&e));
}

#[test]
fn a_ring_inside_a_ring_is_refused() {
    let e = read(
        "point hub hint(x: 0, y: 0)
         ground hub
         ring 3 about hub as i {
           point tip hint(x: 40 * cos(120 * i), y: 40 * sin(120 * i))
           hub distance(40) tip
           ring 2 about tip as j {
             point q hint(x: 40 * cos(120 * i) + 5 * cos(180 * j), y: 40 * sin(120 * i) + 5 * sin(180 * j))
             tip distance(5) q
           }
         }",
    );
    let c = codes(&e);
    assert!(c.iter().any(|(k, _)| *k == "E022"), "{c:?}");
    assert!(!e.ok());
}
