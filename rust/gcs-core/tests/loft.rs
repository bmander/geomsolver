//! **Today's loft is library words** (#243 L3): the reducer, the elbow, the narrowing bend and the
//! square-to-diamond duct, each one section drawn at a station of its guide (`std.stations.OnLine`,
//! `std.stations.OnArc`) and swept as the station runs — `sweep(I(…).f over s)` — at the volume
//! `solid(a[, b], along: g)` gave them, read off the exact B-rep to a part in 10⁹, and with its
//! caps and its walls by name.  `along:` is retired, refused saying what to write; the station
//! words stand on any path the family runs over (`std.stations.On`, `std.stations.OnSpline`).
use gcs_core::program::Elaborated;
use gcs_core::solid::ApproximationPolicy::Report;
use std::collections::BTreeMap;
use std::f64::consts::PI;

use crate::common::parsed::determined;

const REDUCER: &str = include_str!("../../examples/solid_loft.sv");
const ELBOW: &str = include_str!("../../examples/solid_elbow.sv");
const BEND: &str = include_str!("../../examples/solid_bend.sv");
const TWIST: &str = include_str!("../../examples/solid_twist.sv");

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no solid `{name}`")).i()
}

/// The volume of the solid `name` off its exact B-rep, its boundary checked first.
fn exact(e: &Elaborated, name: &str) -> f64 {
    let x = e.sketch.exact_solid(index(e, name)).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    x.brep.check(1e-9 * x.brep.size()).unwrap_or_else(|k| panic!("`{name}`: {}", k.msg));
    gcs_core::brep::props::volume(&x.brep) / x.mm.powi(3)
}

/// The kinds of the exact solid `name`'s surfaces, by face name.
fn kinds(e: &Elaborated, name: &str) -> BTreeMap<String, Vec<&'static str>> {
    let x = e.sketch.exact_solid(index(e, name)).unwrap();
    let mut out: BTreeMap<String, Vec<&'static str>> = BTreeMap::new();
    for f in &x.brep.faces { out.entry(f.name.clone()).or_default().push(f.surface.kind()); }
    out
}

/// `got` within `rel` of `want`, said with both.
fn near(got: f64, want: f64, rel: f64, what: &str) {
    assert!((got - want).abs() <= rel * want.abs(), "{what}: {got} against {want} (±{rel})");
}

/// A hollow square's walls by name, under the section's path in the instance: the outer
/// rectangle's four edges, as the face's own boundary names them, and its hole's.
fn hollow_walls(body: &str) -> Vec<String> {
    let mut v = Vec::new();
    for ring in ["", "inner.loop."] {
        for edge in ["ab", "bc", "cd", "da"] { v.push(format!("{body}.sec.{ring}{edge}")); }
    }
    v
}

/// **Each of today's four lofts is its closed form**, solved at DOF 0: the reducer its prismatoid
/// (its area linear in `s`), the elbow Pappus, the narrowing bend Pappus differentially (its area
/// linear in the turn, its centroid on the arc), and the square run to a diamond its prismatoid
/// (its corners linear in `s`, so its area quadratic and Simpson exact).
#[test]
fn todays_lofts_are_running_sections_at_their_closed_forms() {
    // the twisted duct's section halfway: the square's corners and the diamond's averaged
    let middle = 4.0 * (5.0 * 5.0 + 11.0 * 11.0) / 2.0;
    for (what, src, want) in [
        ("reducer", REDUCER, 40.0 * (8.0 * 18.0 - 16.0)),
        ("elbow", ELBOW, (18.0 * 18.0 - 14.0 * 14.0) * 30.0 * PI / 2.0),
        // A(s) = 16 h − 16 with h = 9 − 3s, so ∫A = 104
        ("bend", BEND, 104.0 * 30.0 * PI / 2.0),
        ("twist", TWIST, 40.0 / 6.0 * (400.0 + 4.0 * middle + 288.0)),
    ] {
        let e = determined(src);
        near(exact(&e, "body"), want, 1e-9, what);
        // and the faceted evaluation, within its chords
        let s = e.sketch.evaluated_solid(index(&e, "body"), Report).unwrap();
        near(s.volume(), want, 1e-3, what);
        assert_eq!(super::solid::unpaired(s.mesh()), 0, "{what}");
        let faces = s.surviving_faces();
        let has = |cap: &str| faces.iter().any(|f| f == cap);
        assert!(has("body.start") && has("body.end"), "{what}: {faces:?}");
    }
}

/// **The walls are the section's edges, in the kinds a loft's were**: the reducer's planes, the
/// elbow's planes and cylinders, and the bend's and the twisted duct's sheets, one wall each edge.
#[test]
fn the_walls_are_named_by_the_section_and_keep_their_kinds() {
    // narrowing as it turns, none of the bend's walls stays on one surface
    for (what, src, flat, round, sheets) in
        [("reducer", REDUCER, 8, 0, 0), ("elbow", ELBOW, 4, 4, 0), ("bend", BEND, 0, 0, 8)] {
        let k = kinds(&determined(src), "body");
        let walls: Vec<&'static str> = hollow_walls("body").iter()
            .map(|w| match k.get(w).map(Vec::as_slice) {
                Some([kind]) => *kind,
                other => panic!("{what}: wall `{w}` is {other:?}: {k:?}"),
            })
            .collect();
        let count = |kind: &str| walls.iter().filter(|w| **w == kind).count();
        let counts = (count("plane"), count("cylinder"), count("family"));
        assert_eq!(counts, (flat, round, sheets), "{what}: {k:?}");
        assert_eq!(k.len(), 10, "{what}: the walls and the two caps: {k:?}");
    }
    let k = kinds(&determined(TWIST), "body");
    assert_eq!(k.len(), 6, "{k:?}");
    assert_eq!(k.values().filter(|v| **v == ["family"]).count(), 4, "{k:?}");
}

/// **The words follow their guide**: a longer reducer is longer by its area, a bend through half
/// the angle half the elbow, a bigger radius Pappus again.
#[test]
fn the_sections_follow_their_guide() {
    let e = determined(&REDUCER.replace("length := 40mm", "length := 60mm"));
    near(exact(&e, "body"), 60.0 * 128.0, 1e-9, "a longer reducer");
    let e = determined(&ELBOW.replace("bend_angle := 90deg", "bend_angle := 45deg"));
    near(exact(&e, "body"), 128.0 * 30.0 * PI / 4.0, 1e-9, "half the bend");
    let e = determined(&ELBOW.replace("bend_radius := 30mm", "bend_radius := 40mm"));
    near(exact(&e, "body"), 128.0 * 40.0 * PI / 2.0, 1e-9, "a wider bend");
}

/// **A running section is a Boolean operand**: the reducer cut across its middle keeps the half
/// where it is widest, 40 · ∫₀^½ (176 − 96 s) ds = 3040 mm³.
#[test]
fn a_running_section_is_a_boolean_operand() {
    let src = format!("{REDUCER}
in std.front {{
  q0 := point
  q1 := point
  q2 := point
  q3 := point
  fix((-50mm, -50mm)) q0
  fix((50mm, -50mm)) q1
  fix((50mm, 50mm)) q2
  fix((-50mm, 50mm)) q3
}}
cutter := solid(face(q0, q1, q2, q3, -> close), from: -60mm, to: -20mm)
kept := solid(body)
cutter cut kept
");
    let e = determined(&src);
    near(exact(&e, "kept"), 3040.0, 1e-9, "the reducer cut");
    let s = e.sketch.evaluated_solid(index(&e, "kept"), Report).unwrap();
    near(s.volume(), 3040.0, 1e-6, "the reducer cut, faceted");
}

/// **`along:` is retired**: refused where it is written, saying what a loft is now.
#[test]
fn along_is_retired_saying_what_to_write() {
    for src in ["body := solid(section, along: guide)\n", "body := solid(a, b, along: guide)\n"] {
        let (_, errs) = crate::common::parse(src);
        assert!(errs.iter().any(|e| e.message.contains("`along:` is retired")
                && e.message.contains("over s)") && e.message.contains("std.stations.OnLine")),
                "{src}: {errs:?}");
    }
}

/// A tube of radius 3 along `guide` through the station word `word` of its kind, the whole
/// drawing: the guides are a line 40 long, a quarter arc of radius 30 and a cubic Bézier, in plan.
fn tube(word: &str, kind: &str, guide: &str) -> String {
    format!("unit mm
use std
use std.stations

component Tube(g: {kind}, up: axis, s: Scalar) {{
  st := std.stations.{word}(g, up: up, s: s)
  in st.n {{
    k := circle(center: st.n.origin)
    radius(3mm) k
  }}
  f := face(k)
}}

in std.top {{
  l0 := point
  l1 := point
  fix((-60mm, 0mm)) l0
  fix((-60mm, 40mm)) l1
  straight := line(l0, l1)
  c := point
  e := point
  x := point
  fix((0mm, 0mm)) c
  fix((30mm, 0mm)) e
  fix((0mm, 30mm)) x
  bend := arc(center: c, start: e, end: x)
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((60mm, 0mm)) k0
  fix((60mm, 20mm)) k1
  fix((80mm, 30mm)) k2
  fix((100mm, 30mm)) k3
  curvy := spline(k0, k1, k2, k3)
}}
param s: Scalar over (0, 1)
tube := sweep(Tube({guide}, std.z).f over s)
")
}

/// **One word over any path**: `std.stations.On` stands where `std.stations.OnLine` and
/// `std.stations.OnSpline` do, so a tube along a line is the same solid by either, π·3²·40, and
/// along a spline too; `std.stations.OnArc`'s tube round a quarter of radius 30 is Pappus,
/// π·3²·30·π/2.
#[test]
fn on_stands_where_each_kind_s_word_does() {
    let vol = |word: &str, kind: &str, guide: &str| {
        exact(&determined(&tube(word, kind, guide)), "tube")
    };
    near(vol("On", "path", "straight"), 9.0 * PI * 40.0, 1e-9, "On along a line");
    near(vol("OnLine", "line", "straight"), 9.0 * PI * 40.0, 1e-9, "OnLine");
    near(vol("OnArc", "arc", "bend"), 9.0 * PI * 30.0 * PI / 2.0, 1e-9, "OnArc");
    let (on, by) = (vol("On", "path", "curvy"), vol("OnSpline", "spline", "curvy"));
    near(on, by, 1e-9, "On and OnSpline along a spline");
    // a tube's section square to its path, its centroid on it: longer than the chord, shorter
    // than the control polygon
    let chord = (40.0f64 * 40.0 + 30.0 * 30.0).sqrt();
    assert!(on > 9.0 * PI * chord && on < 9.0 * PI * (20.0 + 500.0f64.sqrt() + 20.0), "{on}");
}
