//! **A countersink** (#765): `solid(std.solids.Countersink(face(rim), angle: θ))` is the cone
//! under a circle `rim` whose sides meet at the included angle `θ` — `Drafted`'s set with no far
//! ordinate, the lean closing the rim at the cone's point, so nothing says how deep it reaches.
//! The sides lean from the rim's own plane, so the rim is drawn where the sink opens, on a plane
//! lying on the face it is cut from.  The region's meridian reads the lean about a whole circle as
//! two straight lines (`solid::region::meridian`), so every evaluator builds the cone: the exact
//! kernel a revolve of its triangle, the field and the facets its meridian turned.  Held here: the
//! cone's volume to its closed form, its faces named as a prism's (`near`, and its side by the
//! rim's edge), the same set written by hand built alike, a strap countersunk on its top face to
//! its closed form, a draft taken to nothing within a prism's ordinates said where it comes to
//! nothing and why, and a countersink beside a counterbore built though the surfaces run on past
//! both faces come near there (#961), a near-contact no face clears refused still.
use gcs_core::brep::affine::{Affine, Mapped};
use gcs_core::brep::geom::{tolerance, Frame, Surface, V};
use gcs_core::brep::query::Located;
use gcs_core::brep::topo::Brep;
use gcs_core::brep::{export, props, ssi, sweep::Say};
use gcs_core::program::Elaborated;
use gcs_core::rng::Rng;
use gcs_core::space::{add, distance, dot, norm, scale, sub};
use gcs_core::solid::faces::{face_paths, FacePath};
use gcs_core::solid::{ApproximationPolicy::Report, MaterialField};
use std::f64::consts::PI;

use crate::common::{ent, text::solved};

/// A circle `rim` of radius 3 about the origin of the top plane, its face `f`, and the solid the
/// set `sink`.
fn rim(sink: &str) -> String {
    format!("unit mm
use std
use std.solids (cut)
in std.top {{
  o := point
  fix((0, 0)) o
  rim := circle(center: o) hint(r: 3)
  radius(3mm) rim
  f := face(rim)
}}
sink := solid({sink})
")
}

/// The exact kernel's solid `name` (as a name picks one, `Sketch::solid_called`: `sink[1]` too),
/// or why it is not built.
fn exact(e: &Elaborated, name: &str) -> Result<gcs_core::brep::topo::Brep, String> {
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let i = e.sketch.solid_called(name)?;
    export::exact(&e.sketch, i, None, None, &say).map(|x| x.solid).map_err(|r| r.message)
}

/// The exact solid's faces by their last names, in order.
fn names(b: &gcs_core::brep::topo::Brep) -> Vec<&str> {
    let mut names: Vec<&str> = b.faces.iter().map(|f| f.name.rsplit(['.', '\u{1}']).next().unwrap()).collect();
    names.sort_unstable();
    names
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
        assert_eq!(names(&b), ["near", "rim"]);
        // and the paths a drawing reaches them by (`repeat a in sink`, `when a == sink.rim`)
        let mut paths: Vec<String> = face_paths(&e.sketch, ent(&e, "sink").i()).iter()
            .map(FacePath::text).collect();
        paths.sort();
        assert_eq!(paths, ["near", "rim"]);
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
  p distance(<= 0mm, along: n) f
  beyond := { q | not some h: q in f at(along: n) h }
  within := { q | some h: q in f at(along: n) h }
  distance(p, beyond) - distance(p, within) >= tan(45deg) * distance(p, f)
}"));
    let b = exact(&e, "sink").unwrap();
    close("by hand, exactly", props::volume(&b), cone(3.0, 90.0), 1e-9);
    assert_eq!(names(&b), ["near", "rim"]);
}

/// The issue's strap (#765): a hole through a strap whose top stands at 33.05, countersunk 90° to
/// a rim of radius 3 drawn on a plane lying on that top — so the sink's sides lean from where it
/// opens, and the cone, cut with the hole, sinks it to wherever it meets the hole.  The strap is
/// its closed form: the block, less the hole, less the frustum the cone leaves outside it.
#[test]
fn a_hole_is_countersunk_on_the_face_it_opens_on() {
    let e = solved("unit mm
use std
use std.solids (cut)
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
deck.origin in boundary(strap.near.side)
deck tangent(at: deck.origin) boundary(strap.near.side)
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
    // an operand's faces by its own path
    for face in ["strap.near", "strap.far", "hole.bore", "sink.rim"] {
        assert!(read.surviving_faces().contains(face), "{face}: {:?}", read.surviving_faces());
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
use std.solids (cut)
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

/// A countersink drawn on a plane of the drawing's own, after more faces than there are planes
/// (#948): its set's first term measures from the rim's *face* (`p distance(<= 0mm, along: n)
/// rim`), which the key read as though the face's number named a plane — out of bounds, a panic
/// in the solid's check, where faces outnumber planes, and another plane's pose read in its key
/// where they do not.  Now the face is read as a face: the solid is checked and built, its key
/// moves with the plane its rim is drawn on and stands still when an unrelated plane moves.
#[test]
fn a_countersink_on_a_plane_of_its_own_reads_its_rim_as_a_face() {
    let mut circles = String::new();
    for k in 1..=6 {
        circles += &format!("  c{k} := circle(center: a) hint(r: {k})\n  radius({k}mm) c{k}\n");
        circles += &format!("  f{k} := face(c{k})\n");
    }
    let mut e = solved(&format!("unit mm
use std
use std.solids (cut)
in std.top {{
  a := point
  fix((0, 0)) a
{circles}}}
other := plane
fix(origin == (0mm, 0mm, -20mm)) other
fix(dir == (1, 0, 0)) other.u
fix(dir == (0, 1, 0)) other.v
deck := plane
fix(origin == (0mm, 0mm, 10mm)) deck
fix(dir == (1, 0, 0)) deck.u
fix(dir == (0, 1, 0)) deck.v
in deck {{
  pc := point
  fix((0, 0)) pc
  rim := circle(center: pc) hint(r: 3)
  radius(3mm) rim
}}
sink := solid(std.solids.Countersink(face(rim), angle: 90deg))
"));
    let refused = gcs_core::program::solid_diagnostics(&e.sketch, &e.map, &e.program);
    assert!(refused.is_empty(), "{:?}", refused.iter().map(|d| &d.message).collect::<Vec<_>>());
    let b = exact(&e, "sink").unwrap();
    close("the sink on its own plane, exactly", props::volume(&b), cone(3.0, 90.0), 1e-9);
    let sink = ent(&e, "sink").i();
    let key = gcs_core::solid::reads(&e.sketch, sink, 0.0);
    let origin = |e: &Elaborated, name: &str| {
        e.sketch.frames[ent(e, name).i()].chart.as_ref().unwrap().o
    };
    for q in origin(&e, "other") { e.sketch.params[q as usize].value += 7.0; }
    assert_eq!(gcs_core::solid::reads(&e.sketch, sink, 0.0), key, "an unrelated plane moved");
    let z = origin(&e, "deck")[2] as usize;
    e.sketch.params[z].value += 1.0;
    assert_ne!(gcs_core::solid::reads(&e.sketch, sink, 0.0), key, "the rim's plane moved");
}

/// Countersinks in a `ring` (#948): the set's distances from `beyond` and `within` each read the
/// set at a hidden point of their own, held in space, which the ring took for a point of the copy
/// drawn out of its view (E023) — a prism in the same ring passed, having none.  Those points are
/// the set's, turned with nothing; the sinks are turned with their rims, each the cone to its
/// point, opening where its rim is.
#[test]
fn countersinks_in_a_ring_are_turned_with_their_rims() {
    let e = solved("unit mm
use std
use std.solids (cut)
use std.place (vertical)
deck := plane
fix(origin == (0mm, 0mm, 22.5mm)) deck
fix(dir == (1, 0, 0)) deck.u
fix(dir == (0, 1, 0)) deck.v
in deck {
  pc := point
  fix((40, 0)) pc
  ring 3 about pc {
    hc := point hint((40, 10))
    pc distance(10mm) hc
    rim := circle(center: hc) hint(r: 3)
    radius(3mm) rim
    sink := solid(std.solids.Countersink(face(rim), angle: 90deg))
  }
  pc vertical hc[0]
}
");
    for k in 0..3 {
        let sink = format!("sink[{k}]");
        let (i, b) = (e.sketch.solid_called(&sink).unwrap(), exact(&e, &sink).unwrap());
        close(&sink, props::volume(&b), cone(3.0, 90.0), 1e-9);
        // under its rim's centre, a turn of 120° about (40, 0) from (40, 10): its point 3 below
        let a = (90.0 + 120.0 * k as f64).to_radians();
        let (x, y) = (40.0 + 10.0 * a.cos(), 10.0 * a.sin());
        let apex = [x, y, 22.5 - 3.0];
        assert!(b.vertices.iter().any(|v| (0..3).all(|j| (v.p[j] - apex[j]).abs() < 1e-9)),
                "{sink}'s point at {apex:?}: {:?}", b.vertices.iter().map(|v| v.p).collect::<Vec<_>>());
        let facets = gcs_core::solid::resolve(&e.sketch, i, gcs_core::solid::REPORT_UNIT);
        assert!(facets.inside([x, y, 21.5]), "{sink} under its rim at ({x:.3}, {y:.3})");
        assert!(!facets.inside([x + 2.5, y, 21.5]), "{sink} narrows below its rim");
    }
}

/// The issue's part (#961): a disc of radius 100 from −20 to 20, a counterbore of radius 62.5
/// down to its floor at 0 — its wall a cylinder unless `draft` leans it (a cone) — a hole of
/// radius 3.5 from −16 to 1 at `(x, y)`, and a Ø8 × 90° countersink on it, opening on the
/// floor (where `sink`). `flipped`: the same part turned over, the counterbore below the floor
/// and the countersink drawn on a plane facing down, so it sinks up.
fn beside(x: f64, y: f64, draft: f64, flipped: bool, sink: bool) -> String {
    let (cb, hole, deck) = if flipped { ("-21mm, to: 0mm", "-1mm, to: 16mm", "under") }
        else { ("0mm, to: 21mm", "-16mm, to: 1mm", "std.top") };
    let bore = if draft == 0.0 { format!("std.solids.Prism(face(cbore), from: {cb})") }
        else { format!("std.solids.Drafted(face(cbore), from: {cb}, draft: {draft}deg)") };
    let ty = if flipped { -y } else { y };
    let sink = if sink {
        "csk := solid(std.solids.Countersink(face(sink), angle: 90deg))\ncsk cut part\n"
    } else { "" };
    format!("unit mm
use std
use std.solids
use std.solids (cut)
under := plane
fix(origin == (0mm, 0mm, 0mm)) under
fix(dir == (1, 0, 0)) under.u
fix(dir == (0, -1, 0)) under.v
in std.top {{
  o := point
  fix((0, 0)) o
  rim := circle(center: o) hint(r: 100)
  radius(100mm) rim
  cbore := circle(center: o) hint(r: 62.5)
  radius(62.5mm) cbore
  h := point
  fix(({x}, {y})) h
  drill := circle(center: h) hint(r: 3.5)
  radius(3.5mm) drill
}}
in {deck} {{
  c := point
  fix(({x}, {ty})) c
  sink := circle(center: c) hint(r: 4)
  radius(4mm) sink
}}
part := solid(std.solids.Prism(face(rim), from: -20mm, to: 20mm))
cb := solid({bore})
cb cut part
hole := solid(std.solids.Prism(face(drill), from: {hole}))
hole cut part
{sink}")
}

/// What the countersink takes from the drilled part, whose hole it is coaxial with: the cone of
/// radius 4 to its point 4 deep, less what of it the hole of radius 3.5 already took — the cone's
/// last 3.5 and the cylinder of the top half millimetre.
fn sunk() -> f64 { PI / 3.0 * (64.0 - 3.5f64.powi(3)) - PI * 3.5 * 3.5 * 0.5 }

/// The issue's part to its closed form, the hole centred `d` from the axis: the disc, less the
/// counterbore, the hole below the floor, the hole's top millimetre where it stands outside the
/// counterbore (the circles' lens subtracted), and the countersink.
fn beside_volume(d: f64) -> f64 {
    let (r, big) = (3.5f64, 62.5f64);
    let lens = r * r * ((d * d + r * r - big * big) / (2.0 * d * r)).acos()
        + big * big * ((d * d + big * big - r * r) / (2.0 * d * big)).acos()
        - 0.5 * ((-d + r + big) * (d + r - big) * (d - r + big) * (d + r + big)).sqrt();
    PI * 1e4 * 40.0 - PI * big * big * 20.0 - PI * r * r * 16.0 - (PI * r * r - lens) - sunk()
}

/// The exact part, checked a valid closed shell, and its volume.
fn built(src: &str, what: &str) -> f64 {
    let e = solved(src);
    let b = exact(&e, "part").unwrap_or_else(|m| panic!("{what}: {m}"));
    b.check(gcs_core::brep::geom::tolerance(b.size())).unwrap_or_else(|m| panic!("{what}: {m}"));
    props::volume(&b)
}

/// **A countersink beside a counterbore** (#961): its cone's box of parameters, grown past the
/// apex onto the other nappe, and the counterbore wall's surface, run on below the floor, came
/// within 4e-7 of one another 4.25 below the floor — on neither face — and the search for where
/// they meet refused that as unresolved.  A near-contact off either face is no part of what is
/// built: the part is its closed form wherever the hole stands by the wall (centred 0.08 inside
/// it, 0.1 inside, on it, and 1.5 inside, which always built), turned about the axis and turned
/// over, a valid closed shell each time.
#[test]
fn a_countersink_beside_a_counterbore_is_built() {
    for (d, turn, flipped) in [(62.42, 0.0, false), (62.4, 0.0, false), (62.5, 0.0, false),
                               (61.0, 0.0, false), (62.42, 37.0, false), (62.4, 200.0, false),
                               (62.42, 0.0, true), (62.5, 200.0, true)] {
        let a = f64::to_radians(turn);
        let (x, y) = ((d * a.cos() * 1e9).round() / 1e9, (d * a.sin() * 1e9).round() / 1e9);
        let over = if flipped { ", turned over" } else { "" };
        let what = format!("centred {d} out at {turn}°{over}");
        close(&what, built(&beside(x, y, 0.0, flipped, true), &what), beside_volume(d), 1e-9);
    }
}

/// The same near-contact between two cones (#961): the counterbore drafted, its wall a cone run on
/// below the floor, leaning either way, beside the countersink's apex.  Each part is the drilled
/// part less what the countersink takes from it (the cone below the floor, the drafted counterbore
/// above it, nothing of the one in the other).
#[test]
fn a_countersink_beside_a_drafted_counterbore_is_built() {
    for (draft, d) in [(2.0, 62.5), (5.0, 62.8), (-5.0, 62.1)] {
        let what = format!("drafted {draft}°, centred {d} out");
        let drilled = built(&beside(d, 0.0, draft, false, false), &format!("{what}, drilled"));
        close(&what, built(&beside(d, 0.0, draft, false, true), &what), drilled - sunk(), 1e-9);
    }
}

/// The countersink's cone and the counterbore's wall of the issue's part, the cone's box of its
/// face's parameters grown as a Boolean grows it (a sixteenth a side, past the apex), and its
/// face; the wall's face.
fn cone_and_wall() -> (Brep, usize, [[f64; 2]; 2], Brep, usize) {
    let e = solved(&beside(62.42, 0.0, 0.0, false, true));
    let (cb, csk) = (exact(&e, "cb").unwrap(), exact(&e, "csk").unwrap());
    let wall = cb.faces.iter().position(|f| matches!(f.surface, Surface::Cylinder(..))).unwrap();
    let cone = csk.faces.iter().position(|f| matches!(f.surface, Surface::Cone(..))).unwrap();
    let d = gcs_core::brep::boolean::face_parameters(&csk, cone);
    let periods = csk.faces[cone].surface.periods();
    let grown = std::array::from_fn(|k| {
        let g = (d[k][1] - d[k][0]) / 16.0 + 1e-9 * (1.0 + d[k][0].abs().max(d[k][1].abs()));
        match periods[k] {
            Some(p) if d[k][1] - d[k][0] + 2.0 * g >= p => [d[k][0], d[k][0] + p],
            _ => [d[k][0] - g, d[k][1] + g],
        }
    });
    (csk, cone, grown, cb, wall)
}

/// **A near-contact within both patches is refused still** (#961): the same cone and cylinder met
/// over boxes of their parameters — the cone's grown past its apex, the wall's run on 30 below
/// the floor and above the top, so the issue's point lies in both — and searched with no faces
/// to clear it, after tracing the meeting the search finds, come within a tolerance at the
/// issue's point and are refused there, as before: only a face's trim clears a near-contact.
#[test]
fn a_near_contact_within_both_patches_is_still_refused() {
    let (csk, cone, da, cb, wall) = cone_and_wall();
    let (sc, sw) = (&csk.faces[cone].surface, &cb.faces[wall].surface);
    let dw = gcs_core::brep::boolean::face_parameters(&cb, wall);
    let db = [dw[0], [dw[1][0] - 30.0, dw[1][1] + 30.0]];
    let tol = 1e-7;
    let at = "[62.4996, -0.2370, -4.2500] and no meeting was found there: \
              whether they touch is not resolved";
    let met = gcs_core::brep::meeting::meet(sc, da, sw, db, tol);
    let why = met.map(|m| m.branches.len()).unwrap_err();
    assert!(why.contains(at), "met over boxes: {why}");
    let (lo, hi) = ([-200.0, -200.0, -30.0], [200.0, 200.0, 30.0]);
    let mut known = Vec::new();
    let why = loop {
        match ssi::unseen(sc, da, sw, &known, lo, hi, tol) {
            Ok(Some(q)) => {
                let more = ssi::trace(sc, sw, &[q], lo, hi, tol).unwrap_or_default();
                assert!(!more.is_empty() && known.len() < 8, "a meeting at {q:?} not traced");
                known.extend(more);
            }
            Ok(None) => panic!("searched with no faces, the near-contact was read as none"),
            Err(why) => break why,
        }
    };
    assert!(why.contains(at), "searched with no faces: {why}");
}

/// **What is clear of a face** (#961, `Located::face_clear`): a point of its surface outside it
/// with no edge — a seam, an apex — within the reach along the surface of the ball asked about.
/// In the face, or beside its edge by less than that reach, is never clear; a ball too large for
/// its surface to be read as one disc about the point (across the cylinder, or at the cone's
/// apex) is not either.
#[test]
fn a_face_is_clear_only_off_it_and_away_from_its_edges() {
    let (csk, cone, _, cb, wall) = cone_and_wall();
    let lc = Located::new(&csk, tolerance(csk.size()));
    let lw = Located::new(&cb, tolerance(cb.size()));
    let r = 1e-6;
    // the wall, of radius 62.5 from 0 to 21
    let on_wall = |z: f64| [62.5 * 0.3f64.cos(), 62.5 * 0.3f64.sin(), z];
    for (z, clear) in [(10.0, false), (-4.25, true), (-3e-7, false), (-1e-3, true),
                       (21.0 + 3e-7, false), (21.001, true)] {
        assert_eq!(lw.face_clear(wall, on_wall(z), r), clear, "the wall at {z}");
    }
    assert!(!lw.face_clear(wall, on_wall(-30.0), 40.0), "a ball across the cylinder");
    // the countersink's cone from its point at (62.42, 0, -4) up to its rim at 0: below the point
    // is its other nappe
    let on_cone = |z: f64| {
        let rho = (z + 4.0).abs();
        [62.42 + rho * 1.1f64.cos(), rho * 1.1f64.sin(), z]
    };
    for (z, clear) in [(-2.0, false), (-4.25, true), (-4.0 - 1e-7, false), (-4.0 - 1e-3, true),
                       (2e-7, false), (0.25, true)] {
        assert_eq!(lc.face_clear(cone, on_cone(z), r), clear, "the cone at {z}");
    }
    // the issue's near-contact is clear of both: each face's foot of it
    let p = [62.4996, -0.2370, -4.2500];
    for (l, b, f) in [(&lc, &csk, cone), (&lw, &cb, wall)] {
        let s = &b.faces[f].surface;
        assert!(l.face_clear(f, s.point(s.inverse(p)), r), "{}", s.kind());
    }
    // how far a ball's points reach along a surface: as far as the ball on a cylinder, twice it
    // on a cone away from its apex, and not read where the ball reaches the apex
    assert_eq!(ssi::disc_reach(&cb.faces[wall].surface, on_wall(5.0), r), Some(r));
    assert_eq!(ssi::disc_reach(&csk.faces[cone].surface, on_cone(-2.0), r), Some(2.0 * r));
    assert_eq!(ssi::disc_reach(&csk.faces[cone].surface, on_cone(-4.0 - 1e-7), r), None);
}

/// A path on `s` from `p` to `q`, both on it: the one `ssi::disc_reach`'s bound is argued by — a
/// plane's segment, a sphere's great-circle arc, a cylinder's helix the shorter way round, a cone's
/// or a torus's parallel to `q`'s azimuth and then its meridian the shorter way, an image's its
/// preimage's mapped.
fn path_on(s: &Surface, p: V, q: V) -> Vec<V> {
    const N: usize = 256;
    let t = |j: usize| j as f64 / N as f64;
    let shorter = |a: f64, b: f64, period: Option<f64>| {
        let d = b - a;
        period.map_or(d, |w| d - (d / w).round() * w)
    };
    match s {
        Surface::Plane(_) => (0..=N).map(|j| add(p, scale(sub(q, p), t(j)))).collect(),
        Surface::Sphere(f, _) => {
            let (a, b) = (sub(p, f.o), sub(q, f.o));
            let w = (dot(a, b) / (norm(a) * norm(b))).clamp(-1.0, 1.0).acos();
            if w < 1e-9 { return vec![p, q] }
            (0..=N).map(|j| add(f.o, add(scale(a, ((1.0 - t(j)) * w).sin() / w.sin()),
                                         scale(b, (t(j) * w).sin() / w.sin())))).collect()
        }
        Surface::Cylinder(..) => {
            let (a, b) = (s.inverse(p), s.inverse(q));
            let du = shorter(a[0], b[0], s.periods()[0]);
            (0..=N).map(|j| s.point([a[0] + t(j) * du, a[1] + t(j) * (b[1] - a[1])])).collect()
        }
        Surface::Cone(..) | Surface::Torus(..) => {
            let (a, b, periods) = (s.inverse(p), s.inverse(q), s.periods());
            let (du, dv) = (shorter(a[0], b[0], periods[0]), shorter(a[1], b[1], periods[1]));
            (0..=N).map(|j| s.point([a[0] + t(j) * du, a[1]]))
                .chain((0..=N).map(|j| s.point([a[0] + du, a[1] + t(j) * dv]))).collect()
        }
        Surface::Affine(m) => path_on(&m.of, m.back.point(p), m.back.point(q)).into_iter()
            .map(|x| m.map.point(x)).collect(),
        _ => unreachable!("not a searchable surface"),
    }
}

/// **`ssi::disc_reach` holds by every path it is argued by** (#961), whatever the argument: over
/// each searchable kind — a plane, a sphere, a cylinder, cones from narrow to wide (past 45°, where
/// the other nappe is nearest), a ring torus with its circle either side of the axis, an
/// ellipsoid and an elliptic cylinder as images — seeded points `p` of the surface, the largest
/// ball the bound admits about each and a smaller one, and many points `q` of the surface within
/// it (the feet of seeded points of the ball, so any sheet within the ball is reached): the path
/// from `p` to `q` ends at `q` and keeps within the reach given. A spindle or a horn torus, a cone
/// at its apex and a ball across a cylinder are given none.
#[test]
fn a_surfaces_points_within_a_ball_reach_no_farther_along_it_than_said() {
    let tilted = Frame::new([1.0, -2.0, 0.5], [0.3, 0.4, 1.0], [1.0, 0.0, 0.0]);
    let mapped = |of: Surface| {
        let axes = Frame::new([0.5, 0.0, 1.0], [0.0, 0.6, 1.0], [1.0, 1.0, 0.0]);
        let map = Affine::stretch(&axes, [1.0, 2.5, 0.6]);
        Surface::Affine(std::sync::Arc::new(Mapped::new(map.unwrap(), of)))
    };
    let (round, along, across) = ([0.0, 6.3], [-10.0, 10.0], [-1.5, 1.5]);
    let mut surfaces = vec![
        ("a plane", Surface::Plane(tilted), [along, along]),
        ("a sphere", Surface::Sphere(tilted, 5.0), [round, across]),
        ("a cylinder", Surface::Cylinder(tilted, 4.0), [round, along]),
        ("a ring torus", Surface::Torus(tilted, 6.0, 2.0), [round, round]),
        ("a ring torus drawn across", Surface::Torus(tilted, -5.0, 3.0), [round, round]),
        ("an ellipsoid", mapped(Surface::Sphere(tilted, 3.0)), [round, across]),
        ("an elliptic cylinder", mapped(Surface::Cylinder(tilted, 2.0)), [round, along]),
    ];
    for alpha in [10.0, 30.0, 45.0, 60.0, 80.0] {
        let cone = Surface::Cone(tilted, 2.0, f64::to_radians(alpha));
        surfaces.push(("a cone", cone, [round, [-12.0, 12.0]]));
    }
    let mut rng = Rng::new(961);
    for (what, s, [[u0, u1], [v0, v1]]) in &surfaces {
        let mut asked = 0;
        for _ in 0..24 {
            let p = s.point([rng.uniform(*u0, *u1), rng.uniform(*v0, *v1)]);
            // the largest ball the bound admits about `p`, by halving (it admits a smaller one too)
            let admits = |r: f64| ssi::disc_reach(s, p, r).is_some();
            if !admits(1e-9) { continue }
            let (mut lo, mut hi) = (1e-9, 1e3);
            for _ in 0..80 { let m = 0.5 * (lo + hi); if admits(m) { lo = m } else { hi = m } }
            for r in [lo, rng.uniform(0.1, 1.0) * lo] {
                let reach = ssi::disc_reach(s, p, r).unwrap();
                for _ in 0..200 {
                    let x = loop {
                        let d = [rng.uniform(-r, r), rng.uniform(-r, r), rng.uniform(-r, r)];
                        if norm(d) <= r { break add(p, d) }
                    };
                    let q = s.point(s.inverse(x));
                    if distance(p, q) > r { continue }
                    asked += 1;
                    let path = path_on(s, p, q);
                    let end = *path.last().unwrap();
                    assert!(distance(end, q) <= 1e-9 * (1.0 + r),
                            "{what}: the path from {p:?} ends at {end:?}, not {q:?}");
                    let far = path.iter().map(|&x| distance(x, p)).fold(0.0, f64::max);
                    assert!(far <= reach * (1.0 + 1e-9),
                            "{what}: from {p:?} to {q:?} within {r}: {far} past the reach {reach}");
                }
            }
        }
        assert!(asked > 1000, "{what}: {asked} points asked");
    }
    // where the surface's points within the ball are not one disc about `p`, no reach is said
    for (big, minor) in [(1.0, 3.0), (2.0, 2.0), (-1.0, 3.0)] {
        let s = Surface::Torus(tilted, big, minor);
        let p = s.point([0.7, 0.4]);
        assert_eq!(ssi::disc_reach(&s, p, 1e-6), None, "a spindle or horn torus, {big}, {minor}");
    }
    let cone = Surface::Cone(Frame::WORLD, 2.0, f64::to_radians(30.0));
    let apex = [0.0, 0.0, -2.0 / f64::to_radians(30.0).tan()];
    assert_eq!(ssi::disc_reach(&cone, apex, 1e-6), None, "at the apex");
    let beside = cone.point([1.0, -4.0 + 1e-7]);
    assert_eq!(ssi::disc_reach(&cone, beside, 1e-6), None, "beside the apex");
    let tube = Surface::Cylinder(Frame::WORLD, 4.0);
    assert_eq!(ssi::disc_reach(&tube, [4.0, 0.0, 0.0], 3.0), None, "across a cylinder");
}
