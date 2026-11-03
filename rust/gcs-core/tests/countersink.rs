//! **A countersink** (#765): `solid(std.solids.Countersink(face(rim), angle: θ))` is the cone
//! under a circle `rim` whose sides meet at the included angle `θ` — `Drafted`'s set with no far
//! ordinate, the lean closing the rim at the cone's point, so nothing says how deep it reaches.
//! The sides lean from the rim's own plane, so the rim is drawn where the sink opens, on a plane
//! lying on the face it is cut from.  The region's meridian reads the lean about a whole circle as
//! two straight lines (`solid::region::meridian`), so every evaluator builds the cone: the exact
//! kernel a revolve of its triangle, the field and the facets its meridian turned.  Held here: the
//! cone's volume to its closed form, its faces named as a prism's (`near`, and its side by the
//! rim's edge), the same set written by hand built alike, a strap countersunk on its top face to
//! its closed form, and a draft taken to nothing within a prism's ordinates said where it comes
//! to nothing and why.
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::program::Elaborated;
use gcs_core::solid::{ApproximationPolicy::Report, MaterialField};
use std::f64::consts::PI;

use crate::common::{ent, text::solved};

/// A circle `rim` of radius 3 about the origin of the top plane, its solid the set `sink` over
/// `face(rim)`.
fn rim(sink: &str) -> String {
    format!("unit mm
use std
use std.solids
in std.top {{
  o := point
  fix((0, 0)) o
  rim := circle(center: o) hint(r: 3)
  radius(3mm) rim
}}
sink := solid({sink})
")
}

/// The exact kernel's solid `name`, or why it is not built.
fn exact(e: &Elaborated, name: &str) -> Result<gcs_core::brep::topo::Brep, String> {
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    export::exact(&e.sketch, ent(e, name).i(), None, None, &say).map(|x| x.solid).map_err(|r| r.message)
}

fn close(what: &str, got: f64, want: f64, rel: f64) {
    eprintln!("{what}: {got:.12} against {want:.12}");
    assert!((got - want).abs() <= rel * want.abs(), "{what}: {got} against {want}");
}

/// The cone under a rim of radius `r` whose sides meet at `angle` degrees: `π r² h / 3`, `h` the
/// depth to its point, `r / tan(angle / 2)`.
fn cone(r: f64, angle: f64) -> f64 {
    let h = r / (angle / 2.0).to_radians().tan();
    PI * r * r * h / 3.0
}

/// A countersink is the cone to its point, whatever its angle: the exact kernel's volume is the
/// closed form's to 1e-9, its faces are the rim's disc `near` and the side named by the rim's
/// edge, and the field and the facets hold its side where it stands, below the rim's plane only.
#[test]
fn a_countersink_is_its_cone_to_its_point() {
    for angle in [90.0, 82.0, 120.0, 60.0] {
        let e = solved(&rim(&format!("std.solids.Countersink(face(rim), angle: {angle}deg)")));
        let want = cone(3.0, angle);
        let b = exact(&e, "sink").unwrap();
        close(&format!("{angle}°, exactly"), props::volume(&b), want, 1e-9);
        let names: Vec<&str> = b.faces.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names.len(), 2, "{names:?}");
        for face in ["near", "rim"] {
            assert_eq!(names.iter().filter(|n| n.ends_with(face)).count(), 1, "{face} in {names:?}");
        }
        let read = e.sketch.evaluated_solid(ent(&e, "sink").i(), Report).unwrap();
        close(&format!("{angle}°, as the app reads it"), read.volume(), want, 2e-3);
        for face in ["near", "rim"] {
            assert!(read.surviving_faces().contains(&format!("sink.{face}")), "{face}");
        }
        // a millimetre down the side stands `tan(angle / 2)` in from the rim, and nothing of the
        // cone is above the rim's plane
        let t = (angle / 2.0f64).to_radians().tan();
        let field = MaterialField::read(&e.sketch, ent(&e, "sink").i(), 1e-10).unwrap();
        let facets = gcs_core::solid::resolve(&e.sketch, ent(&e, "sink").i(), gcs_core::solid::REPORT_UNIT);
        let wall = 3.0 - t;
        for (x, z, inside) in [(wall - 0.02, -1.0, true), (wall + 0.02, -1.0, false),
                               (0.0, -3.0 / t + 0.02, true), (0.0, -3.0 / t - 0.02, false),
                               (1.0, 0.02, false), (1.0, -0.02, true)] {
            assert_eq!(field.side([x, 0.0, z]) < 0.0, inside, "{angle}°: the field at ({x}, {z})");
            assert_eq!(facets.inside([x, 0.0, z]), inside, "{angle}°: the facets at ({x}, {z})");
        }
    }
}

/// The evaluator reads a set by what it says, never by the word's name: the countersink's set
/// written by hand is the same cone, its faces named alike.
#[test]
fn a_countersink_written_by_hand_is_built_alike() {
    let e = solved(&rim("{ p |
  p distance(<= 0mm, along: n) face(rim)
  beyond := { q | q outside face(rim) }
  within := { q | q inside face(rim) }
  distance(p, beyond) - distance(p, within) >= tan(45deg) * distance(p, face(rim))
}"));
    let b = exact(&e, "sink").unwrap();
    close("by hand, exactly", props::volume(&b), cone(3.0, 90.0), 1e-9);
    let mut names: Vec<&str> = b.faces.iter().map(|f| f.name.rsplit('.').next().unwrap()).collect();
    names.sort_unstable();
    assert_eq!(names, ["near", "rim"]);
}

/// The issue's strap (#765): a hole through a strap whose top stands at 33.05, countersunk 90° to
/// a rim of radius 3 drawn on a plane lying on that top — so the sink's sides lean from where it
/// opens, and the cone, cut with the hole, sinks it to wherever it meets the hole.  The strap is
/// its closed form: the block, less the hole, less the frustum the cone leaves outside it.
#[test]
fn a_hole_is_countersunk_on_the_face_it_opens_on() {
    let e = solved("unit mm
use std
use std.solids
in std.top {
  a := point
  b := point
  c := point
  d := point
  fix((-10, -10)) a
  fix((10, -10)) b
  fix((10, 10)) c
  fix((-10, 10)) d
  ab := line(a, b)
  bc := line(b, c)
  cd := line(c, d)
  da := line(d, a)
  f := face(ab, bc, cd, da)
  o := point
  fix((3, 2)) o
  bore := circle(center: o) hint(r: 1.3)
  radius(1.3mm) bore
}
strap := solid(std.solids.Prism(f, from: 30mm, to: 33.05mm))
hole := solid(std.solids.Through(face(bore)))
hole cut strap
up := axis
up coincident o
up perpendicular std.top
deck := plane(u: hint(dir: (1, 0, 0)), v: hint(dir: (0, 1, 0))) hint(origin: (3, 2, 33))
deck.origin coincident strap.near.surface
deck tangent(at: deck.origin) strap.near.surface
deck.origin coincident up
in deck {
  rim := circle(center: deck.origin) hint(r: 3)
  radius(3mm) rim
}
sink := solid(std.solids.Countersink(face(rim), angle: 90deg))
sink cut strap
");
    let (r, h, depth) = (1.3, 33.05 - 30.0, 3.0 - 1.3);
    let frustum = PI * depth * (9.0 + 3.0 * r + r * r) / 3.0;
    let want = 400.0 * h - PI * r * r * h - (frustum - PI * r * r * depth);
    let b = exact(&e, "strap").unwrap();
    close("the countersunk strap, exactly", props::volume(&b), want, 1e-9);
    let read = e.sketch.evaluated_solid(ent(&e, "strap").i(), Report).unwrap();
    for face in ["near", "far", "hole.bore", "sink.rim"] {
        assert!(read.surviving_faces().contains(&format!("strap.{face}")), "{face}: {:?}",
                read.surviving_faces());
    }
    // the sink opens on the top: a rim of radius 3 there, the hole's 1.3 at 1.7 below it
    let facets = gcs_core::solid::resolve(&e.sketch, ent(&e, "strap").i(), gcs_core::solid::REPORT_UNIT);
    for (x, z, material) in [(3.0 + 2.9, 33.0, false), (3.0 + 3.1, 33.0, true),
                             (3.0 + 1.25, 31.0, false), (3.0 + 1.5, 31.0, true)] {
        assert_eq!(facets.inside([x, 2.0, z]), material, "the strap at ({x}, {z})");
    }
}

/// A countersink about a rim that is no whole circle is a pyramid, which no exact construction
/// builds yet: refused with that reason, never built as something else.
#[test]
fn a_countersink_about_a_polygon_is_refused_exactly() {
    let e = solved("unit mm
use std
use std.solids
in std.top {
  a := point
  b := point
  c := point
  fix((0, 0)) a
  fix((6, 0)) b
  fix((0, 6)) c
  ab := line(a, b)
  bc := line(b, c)
  ca := line(c, a)
  f := face(ab, bc, ca)
}
sink := solid(std.solids.Countersink(f, angle: 90deg))
");
    let why = exact(&e, "sink").unwrap_err();
    assert!(why.contains("no exact construction"), "{why}");
}

/// The issue's first drawing (#765): the rim drawn in the top plane, z = 0, and the drafted prism
/// asked for between 31.35 and 33.05 — its sections the rim moved in `|h| tan 45°` from that
/// plane, gone 3 from it.  Refused, saying where the side comes to nothing, measured from what.
#[test]
fn a_draft_taken_to_nothing_says_where_from() {
    let e = solved(&rim("std.solids.Drafted(face(rim), from: 31.35mm, to: 33.05mm, draft: 45deg)"));
    let why = e.sketch.evaluated_solid(ent(&e, "sink").i(), Report).unwrap_err();
    for said in ["takes the side `rim` to nothing 3mm from the face's own plane",
                 "(31.35mm to 33.05mm)", "the parting plane"] {
        assert!(why.contains(said), "{said}: {why}");
    }
}
