//! **Rates in a face family** (#339 D3): a station that states a rate along its running number —
//! `rate(f.u over s) perpendicular f.v`, `std.stations.Minimal`'s rotation-minimising frame —
//! integrated from its drawn station, its home, to each end of the interval, instead of solved
//! again.
//!
//! Gated by closed forms off the exact B-rep: the minimal frame along a helix turns against the
//! helix's Frenet frame (`std.stations.OnHelix`'s, which turns with it) at the torsion, `−2πb/√(a²
//! + b²)` a turn; a tube whose section's centroid stays on the helix is its section's area along
//! the helix's length, whatever the frame does; and along a guide in one plane the word is
//! `std.stations.On`'s frame, the same solid.  And the refusals: a station short of rates, with the
//! freedom it leaves; rows of the home in a station that states no rate; a rate read where no
//! direction is.
use gcs_core::brep::geom::V;
use gcs_core::program::Elaborated;
use std::f64::consts::PI;

use crate::common::parse;
use crate::common::parsed::{build, determined};

/// A coil whose 2 × 1 rectangle is drawn in `frame`'s plane (`n` for `std.stations.OnHelix`, `rm.f`
/// for `std.stations.Minimal` over it), centred on the helix of radius `r` about `std.top`'s
/// normal; and a tube along a guide `g` drawn in `std.top`, its section the same rectangle off the
/// guide, in `std.stations.On`'s plane or `std.stations.Minimal`'s over it.
const RATES: &str = "unit mm
use std
use std.place (coords)
use std.stations

component Coil(base: plane, r: Length, pitch: Length, turns: Scalar, s: Scalar) {
  st := std.stations.OnHelix(base, r: r, pitch: pitch, turns: turns, s: s)
  rm := std.stations.Minimal(st.n, up: base.normal, s: s)
  sec := Rect(rm.f, off: 0mm, w: 2mm, h: 1mm)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

component Frenet(base: plane, r: Length, pitch: Length, turns: Scalar, s: Scalar) {
  st := std.stations.OnHelix(base, r: r, pitch: pitch, turns: turns, s: s)
  sec := Rect(st.n, off: 0mm, w: 2mm, h: 1mm)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

component Tube(g: path, up: axis, s: Scalar) {
  st := std.stations.On(g, up: up, s: s)
  rm := std.stations.Minimal(st.n, up: up, s: s)
  sec := Rect(rm.f, off: 2mm, w: 3mm, h: 1mm)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

component Held(g: path, up: axis, s: Scalar) {
  st := std.stations.On(g, up: up, s: s)
  sec := Rect(st.n, off: 2mm, w: 3mm, h: 1mm)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

component Algebraic(g: spline, up: axis, s: Scalar) {
  st := std.stations.OnSpline(g, up: up, s: s)
  sec := Rect(st.n, off: 2mm, w: 3mm, h: 1mm)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

component Rect(n: plane, off: Length, w: Length, h: Length) {
  in n {
    a := point
    b := point
    c := point
    d := point
  }
  a coords(du: off - w / 2, dv: -h / 2) n
  b coords(du: off + w / 2, dv: -h / 2) n
  c coords(du: off + w / 2, dv: h / 2) n
  d coords(du: off - w / 2, dv: h / 2) n
}

in std.top {
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((60mm, 0mm)) k0
  fix((60mm, 20mm)) k1
  fix((80mm, 30mm)) k2
  fix((100mm, 30mm)) k3
  curvy := spline(k0, k1, k2, k3)
}
";

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no solid `{name}`")).i()
}

/// `RATES` with the solids `families` written below it, determined.
fn with(families: &str) -> Elaborated {
    determined(&format!("{RATES}{families}\n"))
}

/// The exact B-rep of the solid `name`, its boundary checked.
fn brep(e: &Elaborated, name: &str) -> gcs_core::brep::topo::Brep {
    let x = e.sketch.exact_solid(index(e, name)).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    x.brep.check(1e-9 * x.brep.size()).unwrap_or_else(|k| panic!("`{name}`: {}", k.msg));
    x.brep.clone()
}

fn exact(e: &Elaborated, name: &str) -> f64 {
    gcs_core::brep::props::volume(&brep(e, name))
}

/// `got` within `rel` of `want`, said with both.
fn near(got: f64, want: f64, rel: f64, what: &str) {
    assert!((got - want).abs() <= rel * want.abs(), "{what}: {got} against {want} (±{rel})");
}

fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn unit(a: V) -> V {
    let n = dot(a, a).sqrt();
    [a[0] / n, a[1] / n, a[2] / n]
}

/// The four corners of `b` nearest `at`: the corners of its cap there.
fn cap(b: &gcs_core::brep::topo::Brep, at: V) -> Vec<V> {
    let mut ps: Vec<V> = b.vertices.iter().map(|v| v.p).collect();
    ps.sort_by(|p, q| dot(sub(*p, at), sub(*p, at)).total_cmp(&dot(sub(*q, at), sub(*q, at))));
    ps.truncate(4);
    ps
}

/// The turn of a coil's rectangle at the cap at `s` against the helix's Frenet frame there —
/// radius `r`, `pitch` a turn, `turns` of them, about `z` from `x` — as an angle from the principal
/// normal toward the binormal, read off its long side (2 mm, so only to within a half turn).
fn turn(b: &gcs_core::brep::topo::Brep, r: f64, pitch: f64, turns: f64, s: f64) -> f64 {
    let th = 2.0 * PI * turns * s;
    let m = [r * th.cos(), r * th.sin(), pitch * turns * s];
    let corners = cap(b, m);
    let long = corners.iter().flat_map(|p| corners.iter().map(move |q| sub(*q, *p)))
        .find(|d| (dot(*d, *d).sqrt() - 2.0).abs() < 1e-6)
        .expect("a 2 mm side");
    let t = unit([-2.0 * PI * r * th.sin(), 2.0 * PI * r * th.cos(), pitch]);
    let n = [-th.cos(), -th.sin(), 0.0];
    let bn = cross(t, n);
    dot(long, bn).atan2(dot(long, n))
}

/// An angle brought within a quarter turn either way of zero: a side's direction is read only to
/// within a half turn.
fn wrapped(a: f64) -> f64 {
    let a = a.rem_euclid(PI);
    if a > PI / 2.0 { a - PI } else { a }
}

/// **The rotation-minimising frame turns against Frenet's at the torsion** (#243 L9): along a
/// helix of radius `a` rising `b` a radian, `τ = b/(a² + b²)`, so over a turn the minimal frame
/// falls behind the Frenet frame by `2πb/√(a² + b²)` — 28.558° for a radius of 10 and a pitch of
/// 5 — while `std.stations.OnHelix`'s frame, which is Frenet's, does not turn against it at all.
/// Read off the two caps of a coil drawn in each, the home at the middle and the frame integrated
/// to both.
#[test]
fn a_minimal_frame_turns_against_frenet_at_the_torsion() {
    let e = with("
param s: Scalar over (0, 1)
minimal := sweep(Coil(std.top, r: 10mm, pitch: 5mm, turns: 1).f over s)
frenet := sweep(Frenet(std.top, r: 10mm, pitch: 5mm, turns: 1).f over s)
three_quarters := sweep(Coil(std.top, r: 10mm, pitch: 5mm, turns: 0.75).f over s)");
    let b = 5.0 / (2.0 * PI);
    let per_turn = -2.0 * PI * b / (100.0 + b * b).sqrt();
    assert!((per_turn.to_degrees() + 28.558).abs() < 1e-3, "{}", per_turn.to_degrees());
    let swept = |name: &str, turns: f64| {
        let k = brep(&e, name);
        wrapped(turn(&k, 10.0, 5.0, turns, 1.0) - turn(&k, 10.0, 5.0, turns, 0.0))
    };
    let got = swept("minimal", 1.0);
    assert!((got - per_turn).abs() <= 1e-9, "a turn: {got} against {per_turn}");
    let got = swept("three_quarters", 0.75);
    let want = 0.75 * per_turn;
    assert!((got - want).abs() <= 1e-9, "three quarters: {got} against {want}");
    let got = swept("frenet", 1.0);
    assert!(got.abs() <= 1e-9, "Frenet's own frame: {got}");
    // the home is the middle of the interval, where the frame's `u` is square to the axis: the
    // rectangle's long side along the radius there
    let k = brep(&e, "minimal");
    let ends = turn(&k, 10.0, 5.0, 1.0, 0.0);
    assert!((wrapped(ends) - wrapped(-0.5 * per_turn)).abs() <= 1e-9, "the start: {ends}");
}

/// **A tube is its section along its path** whatever its frame does: a section square to the path
/// with its centroid on it sweeps its area times the path's length (Pappus–Guldin's first moment
/// is zero) — a 2 × 1 rectangle along four turns of a helix of radius 10 and pitch 5, in the
/// minimal frame, is 2 · 4√((20π)² + 5²) mm³, as in `std.stations.OnHelix`'s.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 10 s: a tube along a helix")]
fn a_tube_on_a_helix_is_its_section_along_its_length() {
    let e = with("
param s: Scalar over (0, 1)
minimal := sweep(Coil(std.top, r: 10mm, pitch: 5mm, turns: 4).f over s)
frenet := sweep(Frenet(std.top, r: 10mm, pitch: 5mm, turns: 4).f over s)");
    let helix = 4.0 * (20.0 * PI).hypot(5.0);
    near(exact(&e, "minimal"), 2.0 * helix, 1e-9, "the minimal coil");
    near(exact(&e, "frenet"), 2.0 * helix, 1e-9, "the Frenet coil");
}

/// **Along a guide in one plane the word is `std.stations.On`'s frame**: with `up` the plane's
/// normal, the frame that holds its `v` along `up` turns only as the guide does, so it is the
/// minimal one, and the solid swept in either is the same — every corner of it, and its volume.
#[test]
fn on_a_planar_guide_the_word_is_ons_frame() {
    let e = with("
param s: Scalar over (0, 1)
minimal := sweep(Tube(curvy, std.z).f over s)
held := sweep(Held(curvy, std.z).f over s)
algebraic := sweep(Algebraic(curvy, std.z).f over s)");
    let corners = |name: &str| -> Vec<V> { brep(&e, name).vertices.iter().map(|v| v.p).collect() };
    // `std.stations.On`'s frame is integrated too (#363 S2): over a planar guide it is the frame
    // that holds `v` along `up` at every station, `OnSpline`'s.  Corners paired by nearness: two
    // may share a coordinate to the last bits, which an order would not pair
    let a = corners("minimal");
    for other in ["held", "algebraic"] {
        near(exact(&e, "minimal"), exact(&e, other), 1e-9, other);
        let b = corners(other);
        assert_eq!(a.len(), b.len());
        for (from, to) in [(&a, &b), (&b, &a)] {
            for p in from {
                let d = to.iter().map(|q| dot(sub(*p, *q), sub(*p, *q)).sqrt())
                    .fold(f64::MAX, f64::min);
                assert!(d <= 1e-9, "{other}: {p:?} is {d} from the nearest corner");
            }
        }
    }
}

/// The words the solid `name` of `src` is refused with.
fn refused(src: &str, name: &str) -> String {
    let e = build(src);
    if !e.ok() {
        return format!("{:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    }
    e.sketch.exact_solid(index(&e, name)).err().unwrap_or_else(|| panic!("`{name}` was built"))
}

/// **A station short of rates is refused at its home, with the freedom it leaves**: a plane held
/// at its home square to the path and level, but away from it only standing at its place with
/// its axes square, and one rate of its turn stated, leaves two turns free.
/// **Rows of the home in a station that states no rate** are refused, since only a station
/// integrated from its home has one.  And a rate is read where a direction is: never of a circle,
/// and never by a word that does not read one.
#[test]
fn what_is_refused() {
    let short = format!("{RATES}
component Loose(g: path, up: axis, s: Scalar) {{
  st := std.stations.On(g, up: up, s: s)
  f := plane(u: hint(dir: up, turn: -90deg, about: st.n.normal), v: hint(dir: up))
  f.origin coincident st.n.origin
  f.u perpendicular f.v
  rate(f.u over s) perpendicular f.v
  initially {{
    f.u perpendicular st.n.normal
    f.v perpendicular st.n.normal
    f.u perpendicular up
  }}
  sec := Rect(f, off: 0mm, w: 2mm, h: 1mm)
  k := face(sec.a, sec.b, sec.c, sec.d, -> close)
}}
param s: Scalar over (0, 1)
loose := sweep(Loose(curvy, std.z).k over s)
");
    let said = refused(&short, "loose");
    assert!(said.contains("is short of rates: 2 degrees of freedom along its running number"),
            "{said}");
    let home = format!("{RATES}
component Homely(g: spline, up: axis, s: Scalar) {{
  m := point
  m coincident(t == s) g
  q := point hint(at: m, along: g, by: 0.1)
  tl := line(m, q)
  g tangent(at: m) tl
  length(10mm) tl
  n := plane(u: hint(dir: up, turn: -90deg, about: tl), v: hint(dir: up))
  n.origin coincident m
  n.u perpendicular tl
  n.v perpendicular tl
  initially {{
    n.u perpendicular up
  }}
  sec := Rect(n, off: 0mm, w: 2mm, h: 1mm)
  k := face(sec.a, sec.b, sec.c, sec.d, -> close)
}}
param s: Scalar over (0, 1)
homely := sweep(Homely(curvy, std.z).k over s)
");
    // away from its home the `initially` row is not stated and nothing turns the plane: the
    // definition leaves it free there (E108, #562)
    let said = refused(&home, "homely");
    assert!(said.contains("`Homely` leaves 1 of its own unknowns free"), "{said}");
    let circle = format!("{RATES}
component Round(g: path, up: axis, s: Scalar) {{
  st := std.stations.On(g, up: up, s: s)
  in st.n {{
    c := circle(center: st.n.origin)
    radius(1mm) c
  }}
  rate(c over s) perpendicular up
  k := face(c)
}}
param s: Scalar over (0, 1)
round := sweep(Round(curvy, std.z).k over s)
");
    let said = refused(&circle, "round");
    assert!(said.contains("a rate is of an axis's direction or of a point"), "{said}");
    let (_, errs) = parse("unit mm\nf := plane\nrate(f.u over s) coincident f.v\n");
    assert!(errs.iter().any(|e| e.message.contains("a rate stands where a direction is read")),
            "{errs:?}");
    let (_, errs) = parse("unit mm\nf := plane\nrate(f.u) perpendicular f.v\n");
    assert!(errs.iter().any(|e| e.message.contains("a rate names the formal it runs over")),
            "{errs:?}");
}

/// **The new statements print as they were written**: a relation over a rate, and a row of the
/// home, read back to the same statement.
#[test]
fn rates_and_rows_of_the_home_print_as_written() {
    let (p, errs) = parse("unit mm\nf := plane\nrate(f.u over s) perpendicular f.v\n\
                           initially {\n  f.u parallel f.v\n}\n");
    assert!(errs.is_empty(), "{errs:?}");
    let mut said = Vec::new();
    for st in &p.root().body {
        if let gcs_core::syntax::StmtKind::Relation(_) = &st.kind {
            let mut out = String::new();
            gcs_core::syntax::write_stmt_to(&mut out, &st.kind).unwrap();
            said.push(out);
        }
    }
    assert_eq!(said, ["rate(f.u over s) perpendicular f.v", "initially { f.u parallel f.v }"]);
}
