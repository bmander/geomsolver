//! **Some points of sets** ([0.212], docs/contract/sets.md): `some q in A, r in B: p in X` in a
//! set's body, the points some point of each set puts the set's point in — of a segment between
//! them, the join of the sets (of convex sets their hull, built exactly by the kernel's hull), of
//! a ball about one, the set grown.  Held to closed forms: a polytope's volume and area, Steiner's
//! for one grown, Pappus's for a capsule.
use gcs_core::brep::geom::Surface;
use gcs_core::brep::props::volume;
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use gcs_core::solid::region::route;
use std::f64::consts::PI;

use crate::common::{ent, refused, text::solved};

fn close(what: &str, a: f64, b: f64) { assert!((a - b).abs() <= 1e-9 * b.abs().max(1.0), "{what}: {a} against {b}"); }

fn area(b: &gcs_core::brep::topo::Brep) -> f64 { crate::brep::total_area(b) }

/// `name`, built exactly: its boundary, by the route that builds it.
fn exact(e: &Elaborated, name: &str) -> std::rc::Rc<gcs_core::solid::Exact> {
    let s = ent(e, name).i();
    let r = route(&e.sketch, s).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    assert!(r.refusal().is_none(), "`{name}`: {:?}", r.refusal());
    e.sketch.exact_solid(s).unwrap_or_else(|m| panic!("`{name}`: {m}"))
}

/// The regular tetrahedron on (3, 3, 3), (3, −3, −3), (−3, 3, −3), (−3, −3, 3), as the join of two
/// of its opposite edges; and grown by 1, brep-bench's offset/tetrahedron-grown.
const TETRAHEDRON: &str = "unit mm\nuse std\nuse std.sets\nuse std.rounds\n\
    a := point\nb := point\nc := point\nd := point\n\
    fix((3, 3, 3)) a\nfix((3, -3, -3)) b\nfix((-3, 3, -3)) c\nfix((-3, -3, 3)) d\n\
    tetrahedron := solid({ p | some q in std.sets.Segment(a, b), r in std.sets.Segment(c, d): \
    p in std.sets.Segment(q, r) })\n\
    grown := std.rounds.Grown(tetrahedron, d: 1mm)\n";

/// **A tetrahedron is the join of two opposite edges**: some point of each, and the segment
/// between them — the hull of its four corners, read point by point as its planes and built
/// exactly as the kernel's hull; grown by 1, Steiner's, as the bench gives it.
#[test]
fn a_tetrahedron_is_the_join_of_two_of_its_edges() {
    let e = solved(TETRAHEDRON);
    let x = exact(&e, "tetrahedron");
    assert_eq!(x.brep.faces.len(), 4);
    assert!(x.brep.faces.iter().all(|f| matches!(f.surface, Surface::Plane(_))));
    close("volume", volume(&x.brep), 72.0);
    close("area", area(&x.brep), 72.0 * 3f64.sqrt());
    // its field: its inradius below nothing at its middle, nothing on a face and at a corner
    let f = MaterialField::read(&e.sketch, ent(&e, "tetrahedron").i(), 1e-10).unwrap();
    close("middle", f.side([0.0; 3]), -3f64.sqrt());
    assert!(f.side([3.0, 3.0, 3.0]).abs() < 1e-12 && f.side([0.0, 0.0, 3.0]).abs() < 1e-12);
    assert!(f.side([3.1, 3.1, 3.1]) > 0.0 && f.side([0.0, 0.0, -2.9]) < 0.0);
    let g = exact(&e, "grown.body");
    close("grown", volume(&g.brep), 249.53323018738044);
}

/// A square's corners and an octagon's 30 above it: brep-bench's loft/square-to-octagon.
const LOFT: &str = "unit mm\nuse std\nuse std.sets\n\
    s0 := point\ns1 := point\ns2 := point\ns3 := point\n\
    fix((20, -20, 0)) s0\nfix((20, 20, 0)) s1\nfix((-20, 20, 0)) s2\nfix((-20, -20, 0)) s3\n\
    o0 := point\no1 := point\no2 := point\no3 := point\no4 := point\no5 := point\no6 := point\no7 := point\n\
    fix((6, -14, 30)) o0\nfix((14, -6, 30)) o1\nfix((14, 6, 30)) o2\nfix((6, 14, 30)) o3\n\
    fix((-6, 14, 30)) o4\nfix((-14, 6, 30)) o5\nfix((-14, -6, 30)) o6\nfix((-6, -14, 30)) o7\n\
    square := { x | some u in std.sets.Segment(s0, s1), v in std.sets.Segment(s2, s3): x in std.sets.Segment(u, v) }\n\
    near := { x | some u in std.sets.Segment(o0, o1), v in std.sets.Segment(o2, o3): x in std.sets.Segment(u, v) }\n\
    far := { x | some u in std.sets.Segment(o4, o5), v in std.sets.Segment(o6, o7): x in std.sets.Segment(u, v) }\n\
    octagon := { x | some u in near, v in far: x in std.sets.Segment(u, v) }\n\
    loft := solid({ p | some q in square, r in octagon: p in std.sets.Segment(q, r) })\n";

/// **Joins nest**: a square the join of two of its sides, an octagon the join of two joins, and
/// the transition between them the join of both — the hull of all twelve corners, its faces the
/// floor, the top, four trapezoids and four triangles, to the bench's closed forms.
#[test]
fn a_square_to_octagon_transition_is_a_join_of_joins() {
    let e = solved(LOFT);
    let x = exact(&e, "loft");
    assert_eq!(x.brep.faces.len(), 10);
    close("volume", volume(&x.brep), 33760.0);
    close("area", area(&x.brep), 2256.0 + 624.0 * 26f64.sqrt() + 160.0 * 22f64.sqrt());
}

/// **A pyramid is the join of its apex and its base**, a face drawn on the top plane: the pyramid
/// on the rectangle ±10 × ±6 with its apex 12 above, a third of its base times its height.
#[test]
fn a_pyramid_is_the_join_of_its_apex_and_its_base() {
    let src = "unit mm\nuse std\nuse std.sets\nin std.top {\n\
        b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
        fix((-10, -6)) b0\nfix((10, -6)) b1\nfix((10, 6)) b2\nfix((-10, 6)) b3\n\
        (s := line(b0, b1)) -> (e := line(b1, b2)) -> (n := line(b2, b3)) -> (w := line(b3, b0)) -> close\n\
        base := face(s, e, n, w)\n}\n\
        apex := point\nfix((0, 0, 12)) apex\n\
        pyramid := solid({ p | some q in base: p in std.sets.Segment(apex, q) })\n";
    let e = solved(src);
    let x = exact(&e, "pyramid");
    assert_eq!(x.brep.faces.len(), 5);
    close("volume", volume(&x.brep), 20.0 * 12.0 * 12.0 / 3.0);
}

/// **A ball about some point of a set is the set grown**: about some point of a segment, a capsule
/// — read as the distance from the segment, its field the distance less the radius.
#[test]
fn a_ball_about_some_point_of_a_set_is_the_set_grown() {
    let src = "unit mm\nuse std\nuse std.sets\n\
        a := point\nb := point\nfix((0, 0, 0)) a\nfix((10, 0, 0)) b\n\
        capsule := solid({ p | some q in std.sets.Segment(a, b): p distance(<= 2mm) q })\n";
    let e = solved(src);
    let f = MaterialField::read(&e.sketch, ent(&e, "capsule").i(), 1e-10).unwrap();
    close("on the axis", f.side([5.0, 0.0, 0.0]), -2.0);
    close("beside it", f.side([5.0, 3.0, 0.0]), 1.0);
    close("past an end", f.side([13.0, 0.0, 0.0]), 1.0);
    let _ = PI;
}

/// **What is not read is refused by name**: a relation that is neither a segment between the
/// points nor a ball about one, a `some` whose set's point is a drawn one, a join of a polygon
/// turning both ways, and `not some` of points.
#[test]
fn what_some_points_of_sets_cannot_read_is_refused() {
    let base = "unit mm\nuse std\nuse std.sets\na := point\nb := point\nfix((0, 0, 0)) a\nfix((10, 0, 0)) b\n";
    refused(&format!("{base}far := solid({{ p | some q in std.sets.Segment(a, b): p distance(>= 2mm) q }})\n"),
        "E040", "is not read yet", "p distance(>= 2mm) q");
    refused(&format!("{base}x := point\nsome q in std.sets.Segment(a, b): x in std.sets.Segment(a, q)\n"),
        "E040", "read where a solid reads its set's point", "in");
    let (_, errs, _) = gcs_core::library::parse_linked(&format!(
        "{base}s := solid({{ p | not some q in std.sets.Segment(a, b): p in std.sets.Segment(a, q) }})\n"));
    assert!(errs.iter().any(|e| e.message.contains("`not some` reads the complement")), "{errs:?}");
    // an L's corners turn both ways: the hull of them is not its join with a point
    let src = "unit mm\nuse std\nuse std.sets\nin std.top {\n\
        l0 := point\nl1 := point\nl2 := point\nl3 := point\nl4 := point\nl5 := point\n\
        fix((0, 0)) l0\nfix((20, 0)) l1\nfix((20, 10)) l2\nfix((10, 10)) l3\nfix((10, 20)) l4\nfix((0, 20)) l5\n\
        (k0 := line(l0, l1)) -> (k1 := line(l1, l2)) -> (k2 := line(l2, l3)) -> (k3 := line(l3, l4)) -> \
        (k4 := line(l4, l5)) -> (k5 := line(l5, l0)) -> close\n\
        ell := face(k0, k1, k2, k3, k4, k5)\n}\n\
        apex := point\nfix((5, 5, 12)) apex\n\
        spire := solid({ p | some q in ell: p in std.sets.Segment(apex, q) })\n";
    let e = solved(src);
    let s = ent(&e, "spire").i();
    let why = MaterialField::read(&e.sketch, s, 1e-10).err().unwrap_or_default();
    assert!(why.contains("convex polygon"), "{why}");
}
