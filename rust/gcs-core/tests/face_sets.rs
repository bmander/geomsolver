//! **A face of a solid is a set** (§6.9, §6.21, #162 F5): `b0.near`, `b0.far`, a side `b0.ab`
//! and `body.b0.near` name the surface the face lies on, made from the solid's own text, so a
//! point is put on one as on any set and a plane lies on one by touching it at its own origin —
//! sketching on a face.  The carrier follows the extents and the profile wherever they move.
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{cross, dot};

use crate::common::{ent, read, refused};

/// A 40 × 30 rectangle on the front plane (normal −y), a corner held at the origin.
const PROFILE: &str = "use std\n\
    in std.front {\n  a := point hint((0, 0))\n  b := point hint((40, 0))\n  \
    c := point hint((40, 30))\n  d := point hint((0, 30))\n  ab := line(a, b)\n  \
    bc := line(b, c)\n  cd := line(c, d)\n  da := line(d, a)\n}\n\
    fix((0, 0)) a\nhorizontal ab\nvertical bc\nhorizontal cd\nvertical da\n\
    a distance(40) b\nb distance(30) c\nf0 := face(ab, bc, cd, da)\n";

fn solved(src: &str) -> Elaborated {
    let mut e = read(&format!("{PROFILE}{src}"));
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    e
}

fn at(e: &Elaborated, n: &str) -> [f64; 3] {
    e.sketch.world_point(ent(e, n).i())
}

/// A point put on a cap stands at the cap's ordinate along the profile's normal (−y here):
/// `depth: d` runs from −d to 0, so `far` is behind the profile and `near` is on it.
#[test]
fn a_point_on_a_cap_stands_at_its_ordinate() {
    let y = |sweep: &str, face: &str| {
        let e = solved(&format!("b0 := solid(f0, {sweep})\nm := point hint((5, 3, 5))\n\
                                 m coincident b0.{face}\n"));
        at(&e, "m")[1]
    };
    assert!((y("depth: 20", "far") - 20.0).abs() < 1e-9);
    assert!(y("depth: 20", "near").abs() < 1e-9);
    assert!((y("from: 5, to: 25", "near") + 25.0).abs() < 1e-9);
    assert!((y("from: 5, to: 25", "far") + 5.0).abs() < 1e-9);
    // the ends written the other way round are still the nearer and the farther
    assert!((y("from: 25, to: 5", "near") + 25.0).abs() < 1e-9);
}

/// The carrier is written over the solid's own extent, so editing it moves what stands on it.
#[test]
fn the_carrier_follows_the_extent() {
    let doc = "h := 20\nb0 := solid(f0, depth: h)\nm := point hint((5, 3, 5))\n\
               m coincident b0.far\n";
    assert!((at(&solved(doc), "m")[1] - 20.0).abs() < 1e-9);
    let edited = doc.replace("h := 20", "h := 35");
    assert!((at(&solved(&edited), "m")[1] - 35.0).abs() < 1e-9);
}

/// **Sketching on a face**: a plane touching a cap at its own origin lies on it — its normal
/// the profile's, its origin at the cap — and keeps the freedom to slide and turn there, four,
/// with nothing implied or over.
#[test]
fn a_plane_touching_a_cap_at_its_origin_lies_on_it() {
    let mut e = solved("b0 := solid(f0, depth: 20)\n\
        v1 := plane(u: hint(dir: (1, 0, 0)), v: hint(dir: (0, 0, 1))) hint(origin: (10, 18, 10))\n\
        v1.origin coincident b0.far\nv1 tangent(at: v1.origin) b0.far\n");
    let n = e.sketch.basis(ent(&e, "v1").i()).normal();
    assert!(n[0].abs() < 1e-9 && n[2].abs() < 1e-9, "{n:?}");
    assert!((at(&e, "v1.origin")[1] - 20.0).abs() < 1e-9);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert!(d.implied.is_empty() && d.over.is_empty(), "{:?} {:?}", d.implied, d.over);
    assert_eq!(d.dof, 4, "{:?}", d.warnings);
}

/// A side over a straight edge is the plane through the edge along the profile's normal: the
/// bottom edge `ab` runs along x at z = 0, so a plane on `b0.ab` faces along z.
#[test]
fn a_plane_on_a_side_holds_the_edge_and_the_normal() {
    let mut e = solved("b0 := solid(f0, depth: 20)\n\
        v1 := plane(u: hint(dir: (1, 0, 0)), v: hint(dir: (0, 1, 0))) hint(origin: (10, 8, 1))\n\
        v1.origin coincident b0.ab\nv1 tangent(at: v1.origin) b0.ab\n");
    let n = e.sketch.basis(ent(&e, "v1").i()).normal();
    assert!(n[0].abs() < 1e-9 && n[1].abs() < 1e-9, "{n:?}");
    assert!(at(&e, "v1.origin")[2].abs() < 1e-9);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert!(d.implied.is_empty() && d.over.is_empty(), "{:?} {:?}", d.implied, d.over);
    assert_eq!(d.dof, 4, "{:?}", d.warnings);
    // a slanted edge: the side's normal is square to it and to the profile's normal
    let slant = PROFILE.replace("vertical bc\n", "").replace("b distance(30) c\n", "");
    let mut e = read(&format!("{slant}b0 := solid(f0, depth: 20)\nfix((50, 30)) c\n\
        v1 := plane(u: hint(dir: (0, 0, 1)), v: hint(dir: (0, 1, 0))) hint(origin: (45, 8, 15))\n\
        v1.origin coincident b0.bc\nv1 tangent(at: v1.origin) b0.bc\n"));
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    let n = e.sketch.basis(ent(&e, "v1").i()).normal();
    let edge = gcs_core::space::sub(at(&e, "c"), at(&e, "b"));
    assert!(dot(n, edge).abs() < 1e-9 && n[1].abs() < 1e-9, "{n:?}");
    assert!(dot(cross(n, edge), cross(n, edge)) > 1.0);
}

/// Through a body the operand keeps its own name (§6.9 [0.18]), and a solid a feature made a
/// body keeps its own sweep's faces.
#[test]
fn a_face_is_reached_through_bodies() {
    let y = |src: &str| at(&solved(src), "m")[1];
    let stock = "b0 := solid(f0, depth: 20)\nbody := solid(b0)\nm := point hint((5, 3, 5))\n";
    assert!((y(&format!("{stock}m coincident body.b0.far\n")) - 20.0).abs() < 1e-9);
    let cut = "b0 := solid(f0, depth: 20)\n\
               in std.front {\n  e := point hint((10, 10))\n  r := circle(center: e) hint(r: 3)\n}\n\
               fix((10, 10)) e\nradius(3) r\ng0 := face(r)\nbore := solid(g0, depth: 30)\n\
               bore cut b0\nm := point hint((5, 3, 5))\nm coincident b0.far\n";
    assert!((y(cut) - 20.0).abs() < 1e-9);
}

/// What has no carrier yet is refused at the use, in its words.
#[test]
fn a_face_with_no_carrier_is_refused_where_it_is_named() {
    let src = |s: &str| format!("{PROFILE}b0 := solid(f0, depth: 20)\nm := point hint((5, 3, 5))\n{s}\n");
    refused(&src("m coincident b0.top"), "E040", "names no face", "b0.top");
    refused(&src("t := solid(f0, through: b0)\nm coincident t.near"), "E040",
            "known only once", "t.near");
    refused(&src("r0 := solid(f0, about: da)\nm coincident r0.end"), "E040",
            "only a prism's faces", "r0.end");
    refused(&src("body := solid(b0)\nm coincident body.near"), "E040", "is a body", "body.near");
}

/// A curved side is no set yet.
#[test]
fn a_curved_side_is_refused() {
    let src = "use std\nin std.front {\n  o := point hint((0, 0))\n  k := circle(center: o) hint(r: 5)\n}\n\
               fix((0, 0)) o\nradius(5) k\ng0 := face(k)\nb0 := solid(g0, depth: 10)\n\
               m := point hint((5, 3, 5))\nm coincident b0.k\n";
    refused(src, "E040", "curved side", "b0.k");
}

/// A solid swept from a face written in place has the same faces: `solid(face(ab, …), …)`.
#[test]
fn a_face_written_in_place_is_a_set_too() {
    let inline = PROFILE.replace("f0 := face(ab, bc, cd, da)\n", "");
    let mut e = read(&format!("{inline}b0 := solid(face(ab, bc, cd, da), depth: 20)\n\
        m := point hint((5, 3, 5))\nm coincident b0.far\n\
        q := point hint((5, 3, 25))\nq coincident b0.cd\n"));
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    assert!((at(&e, "m")[1] - 20.0).abs() < 1e-9);
    assert!((at(&e, "q")[2] - 30.0).abs() < 1e-9);
}
