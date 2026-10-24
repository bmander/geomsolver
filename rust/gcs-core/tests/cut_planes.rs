//! **A solid cut by a plane** (`mesh::section`, `EvaluatedSolid::section`): the loops where the
//! plane meets the surface it is drawn as, which is a hole, and the face the cut leaves.
use crate::common::{build, solid_named, solved};
use gcs_core::mesh::Section;
use gcs_core::plane::Basis;
use gcs_core::solid::{ApproximationPolicy, EvaluatedSolid, LocalPoint};
use gcs_core::space::{cross, dot, sub};

/// A 40 × 30 plate, 10 deep, a hole of radius 5 through its middle.
const PLATE: &str = "\
unit mm
use std
in std.top {
  a := point
  b := point
  c := point
  d := point
  o := point
  fix((0mm, 0mm)) a
  fix((40mm, 0mm)) b
  fix((40mm, 30mm)) c
  fix((0mm, 30mm)) d
  fix((20mm, 15mm)) o
  ab := line(a, b)
  bc := line(b, c)
  cd := line(c, d)
  da := line(d, a)
  hole := radius(5mm) circle(center: o)
  plate := solid(face(ab, bc, cd, da, holes: hole), from: 0mm, to: 10mm)
}
";

/// The plane through `o` spanned by `u` and `v`, its normal `u × v` the side taken away.
fn cut(o: [f64; 3], u: [f64; 3], v: [f64; 3]) -> Basis {
    Basis { o, ..Basis::explicit(u, v).unwrap() }
}

/// `solid` cut by `by`, its section stood in the world as the solid is.
fn cut_in_world(solid: &EvaluatedSolid, by: &Basis) -> Section {
    let mut s = solid.section(by);
    let world = |p: &mut [f64; 3]| *p = solid.to_world(LocalPoint(*p)).0;
    s.loops.iter_mut().flat_map(|l| &mut l.pts).for_each(world);
    for t in s.triangles.as_chunks_mut::<3>().0 {
        world(t);
    }
    s
}

fn plate_cut(by: Basis) -> Section {
    let e = build(PLATE);
    let sk = solved(&e);
    let solid = sk.evaluated_solid(solid_named(&e, "plate"), ApproximationPolicy::Mesh).unwrap();
    cut_in_world(&solid, &by)
}

/// Shoelace area of a loop read in the cut's plane: positive counter-clockwise about its normal.
fn signed(pts: &[[f64; 3]], n: [f64; 3]) -> f64 {
    (0..pts.len()).map(|i| dot(cross(pts[i], pts[(i + 1) % pts.len()]), n)).sum::<f64>() / 2.0
}

/// What every section owes, whatever it cuts: every loop closed and on the plane, an outer loop
/// counter-clockwise about the normal and a hole clockwise, the face's triangles on the plane and
/// facing the side taken away, and the face exactly the loops' area — the outer loops' less the
/// holes', so the holes were left out of it and nothing else was.
fn holds(s: &Section, by: &Basis, scale: f64) {
    let n = gcs_core::space::unit(by.normal());
    let on = |p: [f64; 3]| dot(sub(p, by.o), n).abs() <= 1e-9 * scale;
    assert!(s.open.is_empty(), "a closed surface cuts into closed loops: {} open", s.open.len());
    let mut enclosed = 0.0;
    for l in &s.loops {
        assert!(l.pts.len() >= 3 && l.pts.iter().all(|&p| on(p)));
        let a = signed(&l.pts, n);
        assert_eq!(a < 0.0, l.hole, "a hole turns clockwise and an outer loop counter-clockwise");
        enclosed += a;
    }
    for t in s.triangles.as_chunks::<9>().0 {
        let (a, b, c) = ([t[0], t[1], t[2]], [t[3], t[4], t[5]], [t[6], t[7], t[8]]);
        assert!(on(a) && on(b) && on(c));
        let facing = dot(cross(sub(b, a), sub(c, a)), n);
        assert!(facing > 0.0, "the face faces what is taken away: {facing} at {a:?} {b:?} {c:?}");
    }
    assert!((s.area() - enclosed).abs() <= 1e-9 * scale * scale,
            "the face is the loops' area: {} against {enclosed}", s.area());
}

/// Across the plate's depth: the plate's outline counter-clockwise and the hole clockwise, the
/// face the plate's area less the hole's polygon.
#[test]
fn a_cut_across_a_plate_is_its_outline_less_its_hole() {
    let by = cut([0.0, 0.0, 4.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
    let s = plate_cut(by);
    holds(&s, &by, 40.0);
    assert_eq!(s.loops.len(), 2);
    assert_eq!(s.loops.iter().filter(|l| l.hole).count(), 1);
    let hole = std::f64::consts::PI * 25.0;
    // the hole is a polygon inside its circle: a little less taken out than the circle would
    assert!(s.area() > 1200.0 - hole && s.area() < 1200.0 - 0.97 * hole, "{}", s.area());
}

/// Down through the hole: two rectangles side by side, neither a hole, and the face turned the
/// way the cut's normal points whichever way that is.
#[test]
fn a_cut_through_a_hole_is_two_pieces() {
    for sense in [1.0, -1.0] {
        let by = cut([20.0, 0.0, 0.0], [0.0, sense, 0.0], [0.0, 0.0, 1.0]);
        let s = plate_cut(by);
        holds(&s, &by, 40.0);
        assert_eq!(s.loops.len(), 2);
        assert!(s.loops.iter().all(|l| !l.hole));
        assert!((s.area() - 200.0).abs() < 1e-6, "{}", s.area());
    }
}

/// A plane past the plate cuts nothing; one on its top face leaves that face whole (the plate is
/// all behind it); one on its bottom face, looking up into it, takes all of it away.
#[test]
fn a_plane_on_a_face_or_past_the_solid() {
    let up = |z: f64| cut([0.0, 0.0, z], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
    assert!(plate_cut(up(20.0)).is_empty());
    let top = plate_cut(up(10.0));
    holds(&top, &up(10.0), 40.0);
    assert_eq!(top.loops.len(), 2);
    assert!(plate_cut(up(0.0)).is_empty(), "everything is on or in front of the plane");
    // what is on the plane is taken away with what is in front of it: the top's rim, and the top
    // itself, give way to the face the cut leaves there
    let (rim, inside) = ([0.0, 0.0, 10.0], [0.0, 0.0, 9.0]);
    assert!(!top.keeps(rim) && top.keeps(inside));
    assert_eq!(top.clip(rim, [40.0, 0.0, 10.0]), None);
    assert_eq!(top.clip(rim, inside), Some((rim, inside)));
    assert_eq!(top.clip([0.0, 0.0, 12.0], [0.0, 0.0, 8.0]), Some((rim, [0.0, 0.0, 8.0])));
    let down = cut([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]);
    let bottom = plate_cut(down);
    holds(&bottom, &down, 40.0);
    assert!((bottom.area() - top.area()).abs() < 1e-9, "the same face, seen from below");
}

/// Library parts cut through the middle of their boxes along each axis, each way: whatever they
/// are — turned, drilled through a bolt circle, hollowed, swept round a bend, patterned — the cut
/// holds what every cut owes.
#[test]
fn library_parts_cut_through_their_middles() {
    for (doc, src) in [
        ("solid_flange", include_str!("../../examples/solid_flange.sv")),
        ("solid_pulley", include_str!("../../examples/solid_pulley.sv")),
        ("solid_tray", include_str!("../../examples/solid_tray.sv")),
        ("solid_elbow", include_str!("../../examples/solid_elbow.sv")),
        ("solid_indexed_pattern", include_str!("../../examples/solid_indexed_pattern.sv")),
    ] {
        let e = build(src);
        let sk = solved(&e);
        let mut cut_any = 0;
        for i in gcs_core::overview::objects(&sk) {
            let solid = sk.evaluated_solid(i, ApproximationPolicy::Mesh).unwrap();
            let b = solid.world_bounds();
            let mid: [f64; 3] = std::array::from_fn(|k| (b.lo[k] + b.hi[k]) / 2.0);
            let scale = (0..3).fold(0.0f64, |m, k| m.max(b.hi[k] - b.lo[k]));
            for k in 0..3 {
                let axis = std::array::from_fn(|j| if j == k { 1.0 } else { 0.0 });
                let (u, v) = gcs_core::space::across(axis);
                for (u, v) in [(u, v), (v, u)] {
                    let by = Basis { o: mid, u, v };
                    let s = cut_in_world(&solid, &by);
                    holds(&s, &by, scale);
                    assert!(s.area() > 0.0, "{doc}: a cut through the middle of {} meets it",
                            sk.solids[i].name);
                    cut_any += 1;
                }
            }
        }
        assert!(cut_any > 0, "{doc} has an object");
    }
}

/// A solid placed by a rotation is cut where it stands in the world: the pulley's body, its axis
/// the world's z, cut square across that axis at its middle, is a ring — the rim less the bore —
/// whatever its own coordinates are.
#[test]
fn a_turned_part_cut_across_its_axis_is_a_ring() {
    let e = build(include_str!("../../examples/solid_pulley.sv"));
    let sk = solved(&e);
    let i = gcs_core::overview::objects(&sk)[0];
    let solid = sk.evaluated_solid(i, ApproximationPolicy::Mesh).unwrap();
    let b = solid.world_bounds();
    let by = cut([0.0, 0.0, (b.lo[2] + b.hi[2]) / 2.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
    let s = cut_in_world(&solid, &by);
    holds(&s, &by, 60.0);
    assert_eq!((s.loops.len(), s.loops.iter().filter(|l| l.hole).count()), (2, 1));
    let pi = std::f64::consts::PI;
    // between the bore and the groove's root, and the bore and the rim
    assert!(s.area() > pi * (21.0f64.powi(2) - 36.0) && s.area() < pi * (28.0f64.powi(2) - 36.0),
            "{}", s.area());
}
