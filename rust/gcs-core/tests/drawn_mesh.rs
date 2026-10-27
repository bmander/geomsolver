//! **A solid's mesh as a renderer draws it** (`mesh::drawn`, #674/#675): `grouped`'s triangles
//! over shared vertices, a vertex written twice only where two faces meet, and the edges a
//! viewer inks — sharp where faces meet at a corner, tangent where one runs smoothly into the
//! next.  Held by the properties: every triangle is the mesh's own, corner for corner and
//! normal for normal; a vertex is shared by every triangle of its face that has it; the sharp
//! edges of a box are its twelve edges, whole; the edges of a rounded plate where its rounds run
//! into its sides are tangent, and the rest are sharp.

use crate::common::read_solved;
use gcs_core::mesh::{self, Mesh};
use gcs_core::model::Sketch;
use gcs_core::solid::ApproximationPolicy;

/// A plate `w` by `d` and `h` deep, its corners fixed.
fn plate(w: f64, d: f64, h: f64) -> String {
    format!("
unit mm
use std
use std.place (offset)
in std.front {{
  a := point hint((0, 0))
  b := point hint(({w}, 0))
  c := point hint(({w}, {d}))
  e := point hint((0, {d}))
  a offset(dx: 0mm, dy: 0mm) std.origin
  b offset(dx: {w}mm, dy: 0mm) std.origin
  c offset(dx: {w}mm, dy: {d}mm) std.origin
  e offset(dx: 0mm, dy: {d}mm) std.origin
  ab := line(a, b)
  bc := line(b, c)
  ce := line(c, e)
  ea := line(e, a)
  body := solid(face(ab, bc, ce, ea), from: 0mm, to: {h}mm)
}}
")
}

/// The rounded plate of `rect_fillets.sv`, 10 deep.
fn rounded() -> String {
    include_str!("../../examples/rect_fillets.sv").to_string()
        + "\nin std.front {\n  body := solid(face(bottom, a_br, right, a_tr, top, a_tl, left, a_bl), from: 0, to: 10)\n}\n"
}

fn only_object(sk: &Sketch) -> usize {
    let objects = gcs_core::overview::objects(sk);
    assert_eq!(objects.len(), 1, "one object");
    objects[0]
}

/// Every triangle of `d` is the mesh's, corner for corner and normal for normal (to a billionth),
/// in its order, and every vertex is of the face whose run of triangles uses it.
fn faithful(m: &Mesh, d: &mesh::Drawn) {
    assert_eq!(d.triangles.len() * 9, m.positions.len(), "the triangles are the mesh's, all of them");
    assert_eq!(d.positions.len(), d.normals.len());
    assert_eq!(d.positions.len(), d.face.len());
    for (gi, g) in m.groups.iter().enumerate() {
        for t in g.start..g.start + g.count {
            for k in 0..3 {
                let v = d.triangles[t][k] as usize;
                assert_eq!(d.face[v] as usize, gi, "a vertex of {} is that face's", g.path);
                for c in 0..3 {
                    assert_eq!(d.positions[v][c], m.positions[9 * t + 3 * k + c]);
                    assert!((d.normals[v][c] - m.normals[9 * t + 3 * k + c]).abs() < 1e-9);
                }
            }
        }
    }
}

fn length(segs: &[[[f64; 3]; 2]]) -> f64 {
    segs.iter().map(|[a, b]| gcs_core::space::distance(*a, *b)).sum()
}

#[test]
fn a_box_is_drawn_over_its_corners_and_inked_round_its_twelve_edges() {
    let (_, sk) = read_solved(&plate(30., 20., 5.));
    let i = only_object(&sk);
    let solid = sk.evaluated_solid(i, ApproximationPolicy::Mesh).unwrap();
    let (m, d) = (solid.mesh(), solid.drawn());
    faithful(m, d);
    // six flat faces, each a triangulated polygon over its own vertices: a corner is written once
    // a face, never once a triangle — a disc of `t` triangles has at most `t + 2` vertices
    assert_eq!(m.groups.len(), 6);
    for (gi, g) in m.groups.iter().enumerate() {
        let used = d.face.iter().filter(|&&f| f as usize == gi).count();
        assert!(used >= 4 && used <= g.count + 2, "{} has {used} vertices for {} triangles", g.path, g.count);
    }
    assert!(d.positions.len() < 3 * d.triangles.len());
    // the sharp edges are the box's twelve, whole: as long as they are, and each between two
    // corners of the box
    assert!((length(&d.sharp) - 4. * (30. + 20. + 5.)).abs() < 1e-9, "{}", length(&d.sharp));
    assert!(d.tangent.is_empty(), "a box has no tangent edge");
    let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
    for p in &d.positions {
        for c in 0..3 { (lo[c], hi[c]) = (lo[c].min(p[c]), hi[c].max(p[c])); }
    }
    for seg in &d.sharp {
        for p in seg {
            let on = (0..3).filter(|&c| (p[c] - lo[c]).abs() < 1e-9 || (p[c] - hi[c]).abs() < 1e-9).count();
            assert!(on >= 2, "{p:?} is on an edge of the box");
        }
    }
}

#[test]
fn a_round_face_shares_its_vertices_and_its_rounds_meet_its_sides_tangent() {
    let (_, sk) = read_solved(&rounded());
    let i = only_object(&sk);
    let solid = sk.evaluated_solid(i, ApproximationPolicy::Mesh).unwrap();
    let (m, d) = (solid.mesh(), solid.drawn());
    faithful(m, d);
    // a round face shades smooth: its facets share their vertices, so it has fewer of them than
    // three a triangle
    let round = m.groups.iter().position(|g| g.smooth).expect("a round face");
    let g = &m.groups[round];
    let used = d.face.iter().filter(|&&f| f as usize == round).count();
    assert!(used < g.count * 2, "{} has {used} vertices for {} triangles", g.path, g.count);
    // its rounds meet its sides tangent: a stretch the depth of the plate on each side of each
    // round, square to the front plane it is drawn in (x across, z up), and no other
    assert!(!d.tangent.is_empty(), "a rounded plate has tangent edges");
    for [a, b] in &d.tangent {
        assert!((a[0] - b[0]).abs() < 1e-9 && (a[2] - b[2]).abs() < 1e-9, "a run-out is square to the plate: {a:?} {b:?}");
    }
    assert!((length(&d.tangent) - 8. * 10.).abs() < 1e-6, "eight run-outs, each 10 deep: {}", length(&d.tangent));
    // and the sharp edges are its two outlines, each in one of its two faces
    let depth: Vec<f64> = d.sharp.iter().flatten().map(|p| p[1]).collect();
    let (near, far) = depth.iter().fold((f64::MAX, f64::MIN), |(a, b), &y| (a.min(y), b.max(y)));
    assert!((far - near - 10.).abs() < 1e-9);
    for seg in &d.sharp {
        assert!(seg.iter().all(|p| (p[1] - near).abs() < 1e-9 || (p[1] - far).abs() < 1e-9),
                "a sharp edge of the rounded plate is on one of its faces: {seg:?}");
    }
}

#[test]
fn the_solids_of_one_binding_draw_from_one_geometry() {
    // two instances of one plate: one evaluation, renamed for the second, its triangles the first's
    let src = "
unit mm
use std
use std.place (offset)
component Plate(w: Length) {
  in std.front {
    a := point hint((0, 0))
    b := point hint((w, 0))
    c := point hint((w, 10))
    e := point hint((0, 10))
    a offset(dx: 0mm, dy: 0mm) std.origin
    b offset(dx: w, dy: 0mm) std.origin
    c offset(dx: w, dy: 10mm) std.origin
    e offset(dx: 0mm, dy: 10mm) std.origin
    ab := line(a, b)
    bc := line(b, c)
    ce := line(c, e)
    ea := line(e, a)
    body := solid(face(ab, bc, ce, ea), from: 0mm, to: 3mm)
  }
}
in (f := frame) { one := Plate(w: 20mm) }
fix(origin == (0, 0, 0)) f
in (g := frame) { two := Plate(w: 20mm) }
fix(origin == (50, 0, 0)) g
";
    let (_, sk) = read_solved(src);
    let objects = gcs_core::overview::objects(&sk);
    assert_eq!(objects.len(), 2);
    let [a, b] = [0, 1].map(|k| sk.evaluated_solid(objects[k], ApproximationPolicy::Mesh).unwrap());
    assert_ne!(a.shape_serial(), b.shape_serial(), "each is its own evaluation, by its own names");
    assert_eq!(a.geometry_serial(), b.geometry_serial(), "and one geometry");
    assert_eq!(a.drawn(), b.drawn(), "the same triangles");
}
