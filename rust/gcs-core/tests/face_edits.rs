//! **A face deleted, said as a set** (docs/contract/sets.md): what is left when a face is taken out
//! of a solid and the faces round it run on along their own surfaces until they meet is the points
//! on the material's side of each of those faces' surfaces, `{ p | p in B.near.side; … }`.  Where
//! the lowering table builds none of it, it is built exactly as the common of the solids those
//! surfaces bound (`solid::bounded`): a plane's half-space, a cylinder, a ball, a cone or a torus,
//! kept within or taken away by the side the material is on, a union (`p in a or p in b`) as the
//! region of its turned terms taken away.  Held to closed forms, brep-bench's `edit/` cases; and a
//! traced intersection written to STEP as a cubic.
use gcs_core::brep::geom::Surface;
use gcs_core::brep::props::volume;
use gcs_core::program::Elaborated;
use gcs_core::solid::region::{route, Route};
use std::f64::consts::PI;

use crate::common::{ent, text::solved};

fn close(what: &str, a: f64, b: f64) { assert!((a - b).abs() <= 1e-9 * b.abs().max(1.0), "{what}: {a} against {b}"); }

/// `name`, built exactly as the common of its terms' solids.
fn bounded(e: &Elaborated, name: &str) -> std::rc::Rc<gcs_core::solid::Exact> {
    let s = ent(e, name).i();
    let r = route(&e.sketch, s).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    assert!(matches!(r, Route::Bounded(Ok(_))), "`{name}`: {:?}", r.refusal());
    e.sketch.exact_solid(s).unwrap_or_else(|m| panic!("`{name}`: {m}"))
}

/// A block 20 × 10 × 10 drilled through by a hole of radius 3: its six faces' sides, the hole's
/// wall left out, are the block whole again — six planes, the hole's ends closed over.
#[test]
fn a_holes_wall_deleted_closes_the_block() {
    let e = solved("unit mm\nuse std\nuse std.solids\nuse std.solids (cut)\n\
        in std.top {\n  a := point\n  b := point hint((20, 0))\n  c := point hint((20, 10))\n  d := point hint((0, 10))\n\
        horizontal (south := line(a, b)) -> vertical (east := line(b, c)) ->\n\
        horizontal (north := line(c, d)) -> vertical (west := line(d, a)) -> close\n\
        fix((0, 0)) a\n  a distance(20) b\n  b distance(10) c\n\
        o := point hint((10, 5))\n  a distance(10mm, along: u) o\n  a distance(5mm, along: v) o\n\
        hole := circle(center: o) hint(r: 3)\n  radius(3) hole\n\
        drill := solid(std.solids.Through(face(hole)))\n  drill cut block\n}\n\
        block := solid(std.solids.Prism(face(south, east, north, west), from: 0mm, to: 10mm))\n\
        part := solid({ p | p in block.near.side; p in block.far.side; p in block.south.side\n\
        p in block.east.side; p in block.north.side; p in block.west.side })\n");
    let x = bounded(&e, "part");
    assert_eq!(x.brep.faces.len(), 6);
    assert!(x.brep.faces.iter().all(|f| matches!(f.surface, Surface::Plane(_))));
    close("volume", volume(&x.brep), 2000.0);
    close("area", crate::brep::total_area(&x.brep), 1000.0);
}

/// A ball of radius 5 cut flat 3 from its centre: its sphere's side alone is the whole ball, its
/// support the box about the sphere.
#[test]
fn a_balls_flat_deleted_is_the_ball() {
    let e = solved("unit mm\nuse std\nuse std.sets\no := point\nfix((0, 0, 0)) o\nup := point\n\
        fix((0, 0, 1)) up\nz_axis := line(o, up)\nball := std.sets.Ball(o, r: 5mm)\n\
        flat := std.sets.HalfSpace(o, z_axis, d: 3mm)\ncut_ball := solid({ p | p in ball; p in flat })\n\
        part := solid({ p | p in cut_ball.ball.side })\n");
    let x = bounded(&e, "part");
    close("volume", volume(&x.brep), 500.0 * PI / 3.0);
    close("area", crate::brep::total_area(&x.brep), 100.0 * PI);
}

/// A post of radius 6 and height 10, its near rim rounded to 2: its two ends' and its wall's
/// sides, the round left out, are the post with a sharp rim — a cylinder kept within between two
/// planes, the four planes about its axis closing its support.
#[test]
fn a_posts_rim_round_deleted_sharpens_the_rim() {
    let e = solved("unit mm\nuse std\nuse std.solids\nuse std.rounds (round)\n\
        in std.top {\n  o := point\n  fix((0, 0)) o\n  rim := circle(center: o) hint(r: 6)\n  radius(6) rim\n}\n\
        post := solid(std.solids.Prism(face(rim), from: 0mm, to: 10mm))\npost.near round(r: 2mm) post\n\
        part := solid({ p | p in post.near.side; p in post.far.side; p in post.rim.side })\n");
    let x = bounded(&e, "part");
    close("volume", volume(&x.brep), 360.0 * PI);
    close("area", crate::brep::total_area(&x.brep), 2.0 * 36.0 * PI + 120.0 * PI);
}

/// A frustum turned about z, from radius 6 at z = 0 to 3 at z = 6, its top rim rounded to 1: its
/// base's, side's and top's sides are the frustum — a cone kept within, its apex at z = 12.
#[test]
fn a_frustums_rim_round_deleted_is_the_frustum() {
    let e = solved("unit mm\nuse std\nuse std.solids\nuse std.rounds (round)\n\
        in std.front {\n  a := point\n  b := point hint((6, 0))\n  c := point hint((3, 6))\n  d := point hint((0, 6))\n\
        horizontal (base := line(a, b)) -> (side := line(b, c)) -> horizontal (top := line(c, d)) ->\n\
        vertical (core := line(d, a)) -> close\n  fix((0, 0)) a\n  a distance(6) b\n  a distance(6) d\n\
        d distance(3) c\n}\n\
        frustum := solid(std.solids.Revolution(face(base, side, top, core), about: std.z))\n\
        frustum.top round(r: 1mm) frustum.side\n\
        part := solid({ p | p in frustum.base.side; p in frustum.side.side; p in frustum.top.side })\n");
    let x = bounded(&e, "part");
    assert!(x.brep.faces.iter().any(|f| matches!(f.surface, Surface::Cone(..))));
    close("volume", volume(&x.brep), PI * 6.0 * (36.0 + 18.0 + 9.0) / 3.0);
}

/// An L-block whose inside corner is rounded: the foot's faces' sides or the upright's, each a box,
/// are the L with its corner sharp — a union, the region of its turned terms taken away from a
/// block holding them.
#[test]
fn a_union_of_face_sides_is_their_regions_together() {
    let e = solved("unit mm\nuse std\nuse std.solids\nuse std.rounds (round)\n\
        in std.top {\n  a := point\n  b := point hint((20, 0))\n  c := point hint((20, 5))\n  d := point hint((5, 5))\n\
        e := point hint((5, 15))\n  f := point hint((0, 15))\n\
        horizontal (south := line(a, b)) -> vertical (east := line(b, c)) ->\n\
        horizontal (tread := line(c, d)) -> vertical (riser := line(d, e)) ->\n\
        horizontal (north := line(e, f)) -> vertical (west := line(f, a)) -> close\n\
        fix((0, 0)) a\n  a distance(20) b\n  b distance(5) c\n  a distance(15) f\n  f distance(5) e\n}\n\
        block := solid(std.solids.Prism(face(south, east, tread, riser, north, west), from: 0mm, to: 8mm))\n\
        block.tread round(r: 2mm) block.riser\n\
        foot := { q | q in block.near.side; q in block.far.side; q in block.south.side\n\
        q in block.east.side; q in block.tread.side; q in block.west.side }\n\
        upright := { q | q in block.near.side; q in block.far.side; q in block.south.side\n\
        q in block.riser.side; q in block.north.side; q in block.west.side }\n\
        part := solid({ p | p in foot or p in upright })\n");
    let x = bounded(&e, "part");
    assert!(x.brep.faces.iter().all(|f| matches!(f.surface, Surface::Plane(_))));
    close("volume", volume(&x.brep), (20.0 * 5.0 + 5.0 * 10.0) * 8.0);
}

/// **A traced intersection is written as a cubic**: a ball of radius 20 pierced off its centre by
/// a hole of radius 6 meets the hole's wall in two closed curves no entity of the standard writes,
/// and the file carries each as a cubic B-spline, never the polyline through it.
#[test]
fn a_traced_curve_is_written_as_a_cubic() {
    let e = solved("unit mm\nuse std\nuse std.solids\nuse std.solids (cut)\nuse std.sets\n\
        use std.place (right_of)\n\
        in std.front {\n  ax := point hint((5, 0))\n  ax right_of(d: 5mm) std.origin\n\
        rim := circle(center: ax) hint(r: 6)\n  radius(6mm) rim\n}\n\
        drill := solid(std.solids.Through(face(rim)))\n\
        part := solid(std.sets.Ball(std.origin, r: 20mm))\ndrill cut part\n");
    let s = ent(&e, "part").i();
    let x = e.sketch.exact_solid(s).unwrap();
    let text = gcs_core::brep::step::write(&x.brep, "part", 1e-4, gcs_core::brep::step::Fit::Exact).unwrap();
    let curves: Vec<&str> = text.lines().filter(|l| l.contains("B_SPLINE_CURVE_WITH_KNOTS('',")).collect();
    assert!(!curves.is_empty(), "no spline curve written");
    // a degree-one spline in space would be a traced curve's polyline; its pcurves are cubics too
    assert!(curves.iter().all(|l| !l.contains("B_SPLINE_CURVE_WITH_KNOTS('',1,")), "a polyline written");
}
