//! A point no `in` reaches stands in space (`docs/planes-plan.md`, #81): three coordinates, its
//! seed and its gauge, through JSON; what it may not make (a curve, a face); and planes stacked
//! by a distance, placed by an axis through both origins.
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::io;
use gcs_core::solve::solve;
use gcs_core::space::{dot, sub};

use crate::common::{ent, read, refused};

#[test]
fn a_point_outside_every_plane_has_three_coordinates() {
    let mut e = read("use std\np := point hint((10, 20, 30))\n\
        q := point\nfix((1, 2, 3)) q\nq distance(40) p\n");
    let (p, q) = (ent(&e, "p").i(), ent(&e, "q").i());
    assert!(e.sketch.points[p].plane.is_none() && e.sketch.points[p].z.is_some());
    assert_eq!(e.sketch.world_point(q), [1.0, 2.0, 3.0], "the gauge holds all three");
    assert_eq!(diagnose(&mut e.sketch, DiagnoseOptions::default()).dof, 2, "a sphere about q");
    assert!(solve(&mut e.sketch, Default::default()).success);
    let d = sub(e.sketch.world_point(p), e.sketch.world_point(q));
    assert!((dot(d, d).sqrt() - 40.0).abs() < 1e-9, "the distance is in space");
}

#[test]
fn a_point_in_space_round_trips_through_json() {
    let e = read("use std\np := point hint((10, 20, 30))\nin std.front {\n  c := point\n}\n");
    let text = io::dumps(&e.sketch, None);
    let back = io::loads(&text).unwrap();
    let (p, c) = (ent(&e, "p").i(), ent(&e, "c").i());
    assert_eq!(back.world_point(p), [10.0, 20.0, 30.0]);
    assert!(back.points[p].z.is_some() && back.points[p].plane.is_none());
    assert_eq!(back.points[c].plane, e.sketch.points[c].plane, "a drawn point keeps its plane");
    assert_eq!(io::dumps(&back, None), text, "and is written again, byte for byte");
}

#[test]
fn a_curve_over_a_point_in_space_is_refused() {
    refused("use std\no := point\nk := circle(center: o) hint(r: 5)\n", "E060",
        "a circle is drawn in a plane, and `o` stands in space", "k := circle(center: o) hint(r: 5)");
    refused("use std\no := point\na := arc(center: o) hint(r: 5)\n", "E060",
        "`o` stands in space: draw it `in` one", "a := arc(center: o) hint(r: 5)");
}

#[test]
fn a_face_over_a_point_in_space_is_refused() {
    refused("use std\na := point\nb := point hint((10, 0, 0))\n\
        c := point hint((0, 10, 0))\nf := face(a, b, c, -> close)\n", "E080",
        "a face lies in one plane, and `a` stands in space", "f := face(a, b, c, -> close)");
}

#[test]
fn planes_stacked_by_a_distance_stand_apart_along_their_normal() {
    let mut e = read("use std\nunit mm\na := plane(u: std.x, v: std.z)\nb := plane hint(origin: (0, -10, 0))\n\
        b.u parallel std.x\nb.v parallel std.z\na distance(12) b\nn := axis hint(dir: (0, -1, 0))\n\
        n perpendicular a\nn coincident a.origin\nn coincident b.origin\n");
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
    assert!(solve(&mut e.sketch, Default::default()).success);
    let (a, b) = (ent(&e, "a").i(), ent(&e, "b").i());
    let (oa, ob) = (e.sketch.basis(a).o, e.sketch.basis(b).o);
    assert!(oa.iter().all(|x| x.abs() < 1e-9), "a stands at the origin: {oa:?}");
    // straight along the shared normal, 12 off, and nowhere else
    assert!(ob[0].abs() < 1e-9 && ob[2].abs() < 1e-9 && (ob[1].abs() - 12.0).abs() < 1e-9,
        "{ob:?}");
}

#[test]
fn a_point_in_space_is_seeded_where_its_place_stands() {
    let e = read("use std\nin std.top {\n  a := point hint((10, 20))\n}\n\
        p := point hint((1, 2, 3))\nr := point hint((5, 6, 7))\n\
        q := point hint(at: a)\nm := point hint(at: p, toward: r, by: 0.5)\n\
        s := point hint(at: std.top, (3, 4))\n");
    let at = |n: &str| e.sketch.world_point(ent(&e, n).i());
    assert_eq!(at("q"), [10.0, 20.0, 0.0], "a point drawn in the top, read where it stands");
    assert_eq!(at("m"), [3.0, 4.0, 5.0], "half way from p to r, in space");
    assert_eq!(at("s"), [3.0, 4.0, 0.0], "a place in the top's own coordinates");
}

/// A seed reads a point in space's third coordinate as it reads the other two.
#[test]
fn a_seed_reads_the_height_of_a_point_in_space() {
    let e = read("use std\nq := point hint((1, 2, 7))\nr := point hint((q.x, q.y, q.z + 1))\n");
    let r = ent(&e, "r").i();
    assert_eq!(e.sketch.world_point(r), [1.0, 2.0, 8.0]);
    // and a point drawn in a plane still has two
    refused("use std\nin std.front {\n  p := point hint((1, 2))\n  s := point hint(x: p.z)\n}\n",
            "E103", "a point has no `z`", "p.z");
}

/// `project` refused names its points and planes as the source does, never by a minted label.
#[test]
fn a_projection_refused_names_what_the_source_calls_things() {
    refused("use std\nin std.front {\n  a := point\n  b := point hint(x: 1)\n}\na project b\n",
            "E061", "both points are on std.front", "a project b");
    refused("use std\nin std.front {\n  a := point\n}\nq := point hint((1, 2, 3))\na project q\n",
            "E061", "q is on no plane", "a project q");
}

/// A kind is said with its article: an axis, an arc.
#[test]
fn a_kind_is_said_with_its_article() {
    refused("use std\nt := axis\nc := circle(center: t)\n",
            "E040", "`t` is an axis, and a circle is built from points", "t");
}
