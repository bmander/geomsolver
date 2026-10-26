//! **Helical sweeps** (#243 L6): a section carried along a screw is a face family whose station is
//! `std.Helix` — `base` turned `360deg · turns · s` about its normal and risen `pitch · turns · s`
//! along it, every direction an ordinate, so all constraints and no motion — or `std.OnHelix`,
//! the plane square to the helix of radius `r`.  A section drawn in `Helix`'s `axial` plane sweeps
//! a thread, one drawn in its `n` a twisted prism, one drawn in `OnHelix`'s `n` a coil.
//!
//! Gated by closed forms off the exact B-rep: an axial section under a screw sweeps its area times
//! its centroid's circle a turn (Pappus, A · 2πr̄ · turns: the rise lies in the section's plane
//! and adds nothing); a section square to the axis sweeps its area times the rise (Cavalieri); a
//! wire square to its helix with its centre on it sweeps its area times the helix's length.  And
//! the fast path: a section the screw carries rigidly lies on the surface its first rim sweeps
//! (`M(s)·C(u)`, a `Blend` of the rim with itself), read off the stations' numbers and held to the
//! general wall before it is kept — never off the word that wrote it.
use gcs_core::brep::geom::V;
use gcs_core::program::Elaborated;
use gcs_core::solid::ApproximationPolicy::Report;
use std::collections::BTreeMap;
use std::f64::consts::PI;

use crate::common::parsed::determined;

/// A thread's square and a V thread's triangle drawn in `std.Helix`'s axial plane, a flight's
/// rectangle drawn in its plane square to the axis, and a coil's wire square to `std.OnHelix`'s
/// helix — each about `std.top`'s normal, `std.z`.
const HELIX: &str = "unit mm
use std (coords, on_u)

component Thread(base: plane, r: Length, size: Length, pitch: Length, turns: Scalar, s: Scalar) {
  st := std.Helix(base, pitch: pitch, turns: turns, s: s)
  in st.axial {
    c := point
    c on_u(d: r) st.axial
  }
  sq := std.CenteredRectangle(c, w: size, h: size) in st.axial
  f := face(sq.loop)
}

component VThread(base: plane, root: Length, crest: Length, half: Length, pitch: Length,
                  turns: Scalar, s: Scalar) {
  st := std.Helix(base, pitch: pitch, turns: turns, s: s)
  in st.axial {
    a := point
    b := point
    c := point
    a coords(du: root, dv: -half) st.axial
    b on_u(d: crest) st.axial
    c coords(du: root, dv: half) st.axial
  }
  f := face(a, b, c, -> close)
}

component Flight(base: plane, off: Length, w: Length, h: Length, pitch: Length, turns: Scalar,
                 s: Scalar) {
  st := std.Helix(base, pitch: pitch, turns: turns, s: s)
  in st.n {
    a := point
    b := point
    c := point
    d := point
    a coords(du: off - w / 2, dv: -h / 2) st.n
    b coords(du: off + w / 2, dv: -h / 2) st.n
    c coords(du: off + w / 2, dv: h / 2) st.n
    d coords(du: off - w / 2, dv: h / 2) st.n
  }
  f := face(a, b, c, d, -> close)
}

component Coil(base: plane, r: Length, wire: Length, pitch: Length, turns: Scalar, s: Scalar) {
  st := std.OnHelix(base, r: r, pitch: pitch, turns: turns, s: s)
  in st.n {
    w := circle(center: st.n.origin)
    radius(wire) w
  }
  f := face(w)
}
";

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no solid `{name}`")).i()
}

/// `HELIX` with the solids `families` written below it, determined.
fn with(families: &str) -> Elaborated {
    determined(&format!("{HELIX}{families}\n"))
}

/// The exact B-rep of the solid `name`, its boundary checked.
fn brep(e: &Elaborated, name: &str) -> gcs_core::brep::topo::Brep {
    let x = e.sketch.exact_solid(index(e, name)).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    x.brep.check(1e-9 * x.brep.size()).unwrap_or_else(|k| panic!("`{name}`: {}", k.msg));
    assert_eq!(x.mm, 1.0);
    x.brep.clone()
}

/// The volume of the solid `name` off its exact B-rep.
fn exact(e: &Elaborated, name: &str) -> f64 {
    gcs_core::brep::props::volume(&brep(e, name))
}

/// The kinds of the surfaces of the solid `name`'s walls (its caps left out), by face.
fn walls(e: &Elaborated, name: &str) -> BTreeMap<String, &'static str> {
    brep(e, name).faces.iter()
        .filter(|f| !f.name.ends_with(".start") && !f.name.ends_with(".end"))
        .map(|f| (f.name.clone(), f.surface.kind()))
        .collect()
}

/// `got` within `rel` of `want`, said with both.
fn near(got: f64, want: f64, rel: f64, what: &str) {
    assert!((got - want).abs() <= rel * want.abs(), "{what}: {got} against {want} (±{rel})");
}

/// **A square thread is A · 2πr̄ · turns** (Pappus): the 2 mm square centred 10 from the axis,
/// three turns at a pitch of 4, is 4 · 20π · 3 = 240π mm³.  **A V thread** likewise about its
/// centroid, which is not its middle: the triangle with its base on the root at 8, 2 wide, and its
/// crest at 10.5 has area 2.5 and centroid (8 + 8 + 10.5)/3 from the axis; two turns at a pitch of
/// 3 are 2.5 · 2π · 26.5/3 · 2.
#[test]
fn a_thread_is_pappus() {
    let e = with("
param s: Scalar over (0, 1)
thread := sweep(Thread(std.top, r: 10mm, size: 2mm, pitch: 4mm, turns: 3).f over s)
vee := sweep(VThread(std.top, root: 8mm, crest: 10.5mm, half: 1mm, pitch: 3mm, turns: 2).f over s)");
    near(exact(&e, "thread"), 240.0 * PI, 1e-9, "a square thread");
    near(exact(&e, "vee"), 2.5 * 2.0 * PI * 26.5 / 3.0 * 2.0, 1e-9, "a V thread");
}

/// **A section square to the axis sweeps a twisted prism**, A × the rise whatever it turns
/// (Cavalieri): a 4 × 2 flight standing 4 to 8 from the axis, half a turn at a pitch of 30 and two
/// and a half at a pitch of 12, is 8 · 15 and 8 · 30 mm³.
#[test]
fn a_flight_is_cavalieri() {
    let e = with("
param s: Scalar over (0, 1)
half := sweep(Flight(std.top, off: 6mm, w: 4mm, h: 2mm, pitch: 30mm, turns: 0.5).f over s)
auger := sweep(Flight(std.top, off: 6mm, w: 4mm, h: 2mm, pitch: 12mm, turns: 2.5).f over s)");
    near(exact(&e, "half"), 8.0 * 15.0, 1e-9, "half a turn");
    near(exact(&e, "auger"), 8.0 * 30.0, 1e-9, "two and a half turns");
}

/// **A spring is A × the helix's length**: a wire of radius 1.5 square to the helix of radius 10,
/// pitch 5, four turns, is π · 1.5² · 4 · √((20π)² + 5²) mm³ — its centre on the helix, the wire
/// square to it, so the tube's volume is its section's area along its length.
#[test]
fn a_spring_is_its_wire_along_the_helix() {
    let e = with("
param s: Scalar over (0, 1)
spring := sweep(Coil(std.top, r: 10mm, wire: 1.5mm, pitch: 5mm, turns: 4).f over s)");
    let helix = 4.0 * (20.0 * PI).hypot(5.0);
    near(exact(&e, "spring"), PI * 1.5 * 1.5 * helix, 1e-9, "the spring");
}

/// **A negative pitch winds left-handed**: the screw still turns from `base.u` toward `base.v`, and
/// falls as it does, so the left-handed thread and spring are the right-handed ones mirrored in
/// the base plane — their corners each a right-handed corner with its height negated — at the same
/// volumes.
#[test]
fn a_negative_pitch_is_the_mirror_image() {
    let e = with("
param s: Scalar over (0, 1)
right := sweep(Thread(std.top, r: 10mm, size: 2mm, pitch: 4mm, turns: 3.25).f over s)
left := sweep(Thread(std.top, r: 10mm, size: 2mm, pitch: -4mm, turns: 3.25).f over s)
coil := sweep(Coil(std.top, r: 10mm, wire: 1.5mm, pitch: -5mm, turns: 4).f over s)");
    near(exact(&e, "left"), exact(&e, "right"), 1e-9, "the mirror thread");
    near(exact(&e, "left"), 4.0 * 20.0 * PI * 3.25, 1e-9, "a left-handed thread");
    let helix = 4.0 * (20.0 * PI).hypot(5.0);
    near(exact(&e, "coil"), PI * 1.5 * 1.5 * helix, 1e-9, "a left-handed spring");
    let corners = |name: &str, flip: f64| -> Vec<V> {
        let mut out: Vec<V> =
            brep(&e, name).vertices.iter().map(|v| [v.p[0], v.p[1], flip * v.p[2]]).collect();
        out.sort_by(|a, b| a.partial_cmp(b).unwrap());
        out
    };
    let (right, left) = (corners("right", 1.0), corners("left", -1.0));
    assert_eq!(right.len(), left.len());
    // each corner of one is a corner of the other: matched by place, since a coordinate of
    // nothing rounds either way and would reorder a sorted list
    for a in &right {
        let near = |b: &&V| (0..3).all(|k| (a[k] - b[k]).abs() <= 1e-9);
        assert!(left.iter().any(|b| near(&b)), "{a:?} in {left:?}");
    }
    // the end section a quarter turn on: toward `base.v`, above the base for the right hand
    assert!(right.iter().any(|p| p[1] > 8.0 && p[2] > 12.0), "{right:?}");
}

/// **The fast path is what the numbers say**: where the screw carries the whole section rigidly,
/// every wall that is not a plane, a cylinder, a cone or a torus lies on the surface its first rim
/// sweeps (`blend`) — the thread's four, the flight's, the coil's wire — each held to the general
/// wall (its points at every node, and its area) before it was kept.  A section that changes as
/// it runs is no screw of its first station: the thread swelling from 2 to 3 keeps its walls on
/// the family's sheets, at its own Pappus, 2π · 10 · 3 · ∫(2 + s)² ds = 380π.
#[test]
fn a_rigid_screw_lies_on_its_swept_rim() {
    let e = with("
param s: Scalar over (0, 1)
thread := sweep(Thread(std.top, r: 10mm, size: 2mm, pitch: 4mm, turns: 3).f over s)
auger := sweep(Flight(std.top, off: 6mm, w: 4mm, h: 2mm, pitch: 12mm, turns: 2.5).f over s)
spring := sweep(Coil(std.top, r: 10mm, wire: 1.5mm, pitch: 5mm, turns: 4).f over s)
component Swelling(base: plane, r: Length, size: Length, pitch: Length, turns: Scalar, s: Scalar) {
  st := std.Helix(base, pitch: pitch, turns: turns, s: s)
  in st.axial {
    c := point
    c on_u(d: r) st.axial
  }
  sq := std.CenteredRectangle(c, w: size * (1 + 0.5 * s), h: size * (1 + 0.5 * s)) in st.axial
  f := face(sq.loop)
}
swelling := sweep(Swelling(std.top, r: 10mm, size: 2mm, pitch: 4mm, turns: 3).f over s)");
    for name in ["thread", "auger", "spring"] {
        let k = walls(&e, name);
        assert!(!k.is_empty() && k.values().all(|&v| v == "blend"), "{name}: {k:?}");
    }
    let k = walls(&e, "swelling");
    assert!(!k.is_empty() && k.values().all(|&v| v == "family"), "{k:?}");
    near(exact(&e, "swelling"), 380.0 * PI, 1e-9, "a swelling thread");
}

/// **And a screwed wall is written as STEP**: the net through it fitted within the file's
/// tolerance, quintic along a screw of many turns, and the file read back describes the solid.
/// The evaluated solid is the B-rep meshed, at its volume.  **And its mesh follows the screw**:
/// within the bar, no facet turned against its surface, its volume the solid's within what
/// chords inscribed in the wire lose.  A rail of a whole number of turns — the spring's four, the
/// swelling coil's — has its ends, middle and quarter points in one line along the axis, their
/// tangents alike, and was sampled as one chord: the wall's mesh fanned across the coil from a
/// seam of two points, 800 000 triangles off it by 17 mm.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 11 s: a screwed wall meshed")]
fn a_screwed_wall_is_written_and_meshed() {
    let e = with("
param s: Scalar over (0, 1)
thread := sweep(Thread(std.top, r: 10mm, size: 2mm, pitch: 4mm, turns: 3).f over s)
spring := sweep(Coil(std.top, r: 10mm, wire: 1.5mm, pitch: 5mm, turns: 4).f over s)
component Swelling(base: plane, r: Length, wire: Length, pitch: Length, turns: Scalar, s: Scalar) {
  st := std.OnHelix(base, r: r, pitch: pitch, turns: turns, s: s)
  in st.n {
    w := circle(center: st.n.origin)
    radius(wire * (1 + 0.5 * s)) w
  }
  f := face(w)
}
swelling := sweep(Swelling(std.top, r: 10mm, wire: 1mm, pitch: 5mm, turns: 4).f over s)");
    for name in ["thread", "spring"] {
        let b = brep(&e, name);
        let text = gcs_core::brep::step::write(&b, name, 1e-3).unwrap();
        let solid = gcs_core::brep::step_check::Solid::of(&b);
        gcs_core::brep::step_check::verify(&text, &solid).unwrap_or_else(|m| panic!("{name}: {m}"));
    }
    let s = e.sketch.evaluated_solid(index(&e, "thread"), Report).unwrap();
    assert_eq!(s.volume(), exact(&e, "thread"));
    let k = walls(&e, "swelling");
    assert!(!k.is_empty() && k.values().all(|&v| v == "family"), "{k:?}");
    let bar = 0.01;
    for name in ["thread", "spring", "swelling"] {
        let b = brep(&e, name);
        let m = gcs_core::brep::mesh::mesh(&b, bar, 0.2).unwrap();
        assert_eq!(m.turned, 0, "{name}: triangles facing against their surfaces");
        assert!(m.sag <= bar, "{name}: the mesh sags {} against {bar}", m.sag);
        let volume: f64 = m.tris.iter().map(|t| {
            let [a, b, c] = t.map(|i| m.pts[i as usize]);
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
        }).sum();
        near(volume, gcs_core::brep::props::volume(&b), 1e-2, name);
    }
}
