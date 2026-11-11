//! **Some points of sets** ([0.212], docs/contract/sets.md): `some q in A, r in B: p in X` in a
//! set's body, the points some point of each set puts the set's point in — the join of sets (the
//! segments between their points: of convex sets their hull) and a set grown (a ball about one).
use gcs_core::solid::region::route;

use crate::common::{ent, text::solved};

/// The regular tetrahedron on (3, 3, 3), (3, −3, −3), (−3, 3, −3), (−3, −3, 3), as the join of two
/// of its opposite edges.
const TETRAHEDRON: &str = "unit mm\nuse std\nuse std.sets\n\
    a := point\nb := point\nc := point\nd := point\n\
    fix((3, 3, 3)) a\nfix((3, -3, -3)) b\nfix((-3, 3, -3)) c\nfix((-3, -3, 3)) d\n\
    tetrahedron := solid({ p | some q in std.sets.Segment(a, b), r in std.sets.Segment(c, d): \
    p in std.sets.Segment(q, r) })\n";

#[test]
fn probe_the_tetrahedron() {
    let e = solved(TETRAHEDRON);
    let s = ent(&e, "tetrahedron").i();
    match route(&e.sketch, s) {
        Ok(r) => eprintln!("route refusal: {:?}", r.refusal()),
        Err(m) => eprintln!("route error: {m}"),
    }
    let f = gcs_core::solid::MaterialField::read(&e.sketch, s, 1e-10).unwrap();
    for p in [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [3.0, 3.0, 3.0], [3.1, 3.1, 3.1], [0.0, 0.0, 3.0]] {
        eprintln!("{p:?} {}", f.side(p));
    }
    let x = e.sketch.exact_solid(s);
    eprintln!("exact: {:?}", x.as_ref().map(|x| gcs_core::brep::props::volume(&x.brep)).map_err(|m| m.to_string()));
}
