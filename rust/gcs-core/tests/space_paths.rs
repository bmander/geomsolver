//! **A curve traced in space is a path** (#363 S1): a station stands on a body traced over one
//! formal as it stands on a spline — `m coincident(t == s) g` pins the contact's place, the trace's
//! own formal, to the family's running number, `hint(at: m, along: g)` steps along its `d[0]`, and
//! a `path` formal takes it — so a family is swept along a guide drawn in space, its frame the
//! rotation-minimising one `std.Minimal` states as a rate.
//!
//! Gated by closed forms off the exact B-rep: a square tube along two turns of a helix traced by
//! its ordinates is its area along the helix's length; the same family over `std.OnHelix`'s
//! station, whose helix is written into its body, is the same solid corner for corner; and the
//! frame integrated along a traced helix falls behind the helix's Frenet frame at its torsion.
//! And the refusals: a surface traced over two formals is no path, and a curve's place is said
//! once, `t` and `u` being one number.
use gcs_core::brep::geom::V;
use gcs_core::program::Elaborated;
use std::f64::consts::PI;

use crate::common::parsed::{build, determined, read, solved};

/// Three axes through the origin drawn in space; a helix of radius `r` rising `pitch` a turn over
/// `turns` of them about `z` from `x`, traced by its point's ordinates as `u` runs over `(0, 1)`;
/// a station on any path whose plane stands square to it and turns by `std.Minimal`'s rate, a
/// `w` × `h` rectangle centred in it; and the same rectangle in `std.Minimal`'s plane over
/// `std.OnHelix`'s station.
const PATHS: &str = "unit mm
use std (coords, horizontal, vertical)

o := point hint((0, 0, 0))
fix((0, 0, 0)) o
X1 := point hint((10, 0, 0))
fix((10, 0, 0)) X1
Y1 := point hint((0, 10, 0))
fix((0, 10, 0)) Y1
Z1 := point hint((0, 0, 10))
fix((0, 0, 10)) Z1
xl := line(o, X1)
yl := line(o, Y1)
zl := line(o, Z1)

component Helix(o: point, x: line, y: line, z: line, r: Length, pitch: Length, turns: Scalar,
                u: Scalar) {
  p := point hint((r * cos(360deg * turns * u), r * sin(360deg * turns * u), pitch * turns * u))
  o distance(r * cos(360deg * turns * u), along: x) p
  o distance(r * sin(360deg * turns * u), along: y) p
  o distance(pitch * turns * u, along: z) p
}

component Station(g: path, up: axis, w: Length, h: Length, s: Scalar) {
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
  n.u perpendicular n.v
  rate(n.u over s) perpendicular n.v
  initially {
    n.u perpendicular up
  }
  sec := Rect(n, w: w, h: h)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

component Coil(base: plane, r: Length, pitch: Length, turns: Scalar, w: Length, h: Length,
               s: Scalar) {
  st := std.OnHelix(base, r: r, pitch: pitch, turns: turns, s: s)
  rm := std.Minimal(st.n, up: base.normal, s: s)
  sec := Rect(rm.f, w: w, h: h)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

component Rider(g: path, up: axis, w: Length, h: Length, s: Scalar) {
  st := std.On(g, up: up, s: s)
  sec := Rect(st.n, w: w, h: h)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

component Rect(n: plane, w: Length, h: Length) {
  in n {
    a := point
    b := point
    c := point
    d := point
  }
  a coords(du: -w / 2, dv: -h / 2) n
  b coords(du: w / 2, dv: -h / 2) n
  c coords(du: w / 2, dv: h / 2) n
  d coords(du: -w / 2, dv: h / 2) n
}
";

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no solid `{name}`")).i()
}

/// `PATHS` with the statements `below` written after it, determined.
fn with(below: &str) -> Elaborated {
    determined(&format!("{PATHS}{below}\n"))
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

/// The solids `a` and `b` have the same corners, each of one within 1e-9 of one of the other's
/// (paired by nearness: two caps may share a coordinate to the last bits, which an order would
/// not).
fn same_corners(e: &Elaborated, a: &str, b: &str) {
    let corners = |name: &str| -> Vec<V> { brep(e, name).vertices.iter().map(|v| v.p).collect() };
    let (pa, pb) = (corners(a), corners(b));
    assert_eq!(pa.len(), pb.len());
    for (from, to) in [(&pa, &pb), (&pb, &pa)] {
        for p in from {
            let d = to.iter().map(|q| dot(sub(*p, *q), sub(*p, *q)).sqrt()).fold(f64::MAX, f64::min);
            assert!(d <= 1e-9, "{p:?} is {d} from the other's nearest corner");
        }
    }
}

/// **A place on a traced curve is its formal**: `m coincident(t == 0.25) guide` stands `m` a
/// quarter of the way along a turn of a helix of radius 10 and pitch 5, at (0, 10, 1.25), from a
/// seed off it; and a seed's step along the curve from there is `by` times its rate along its
/// formal, (−20π, 0, 5) — read where the seed is placed, before any solve.
#[test]
fn a_place_on_a_traced_curve_is_its_formal() {
    let doc = |m: &str| format!("{PATHS}
guide := Helix(o, xl, yl, zl, r: 10mm, pitch: 5mm, turns: 1).p over u in (0, 1)
m := point hint({m})
m coincident(t == 0.25) guide
q := point hint(at: m, along: guide, by: 0.01)
");
    let at = |e: &Elaborated, n: &str| e.sketch.world_point(e.map.ent_named(n).unwrap().i());
    let close = |p: V, q: V| sub(p, q).iter().all(|d| d.abs() <= 1e-9);
    let e = solved(&doc("(9, 1, 0)"));
    assert!(close(at(&e, "m"), [0.0, 10.0, 1.25]), "{:?}", at(&e, "m"));
    let e = read(&doc("(0, 10, 1.25)"));
    let want = [-0.2 * PI, 10.0, 1.3];
    assert!(close(at(&e, "q"), want), "{:?} against {want:?}", at(&e, "q"));
}

/// **A square tube along a traced helix is its section along its length**: a 2 × 2 square square
/// to two turns of a helix of radius 10 rising 10, its centroid on it, sweeps 4·√((40π)² + 10²) =
/// 504.2439 mm³ — and the station the traced guide is handed to is `std.OnHelix`'s, under
/// `std.Minimal`'s frame, corner for corner: the guide drawn in space and the one written into the
/// station's body are one helix, and both frames are integrated from a home square to the axis.
#[test]
fn a_tube_along_a_traced_helix_is_its_section_along_its_length() {
    let e = with("
guide := Helix(o, xl, yl, zl, r: 10mm, pitch: 5mm, turns: 2).p over u in (0, 1)
traced := solid(Station(guide, std.z, w: 2mm, h: 2mm).f over s in (0, 1))
written := solid(Coil(std.top, r: 10mm, pitch: 5mm, turns: 2, w: 2mm, h: 2mm).f
                 over s in (0, 1))");
    let want = 4.0 * (40.0 * PI).hypot(10.0);
    assert!((want - 504.2439).abs() < 1e-4, "{want}");
    near(exact(&e, "traced"), want, 1e-9, "the tube along the traced helix");
    near(exact(&e, "written"), want, 1e-9, "the tube along `std.OnHelix`");
    same_corners(&e, "traced", "written");
}

/// **`std.On` stands square to a twisted guide** (#363 S2): its frame is integrated from its
/// home, `up` read there alone, so along a traced helix it is the station above — the same solid as
/// `std.OnHelix`'s under `std.Minimal`, corner for corner, at its area along the helix's length —
/// where a frame holding `v` along `up` everywhere would lean the section off square to it.
#[test]
fn on_stands_square_to_a_twisted_guide() {
    let e = with("
guide := Helix(o, xl, yl, zl, r: 10mm, pitch: 5mm, turns: 2).p over u in (0, 1)
on := solid(Rider(guide, std.z, w: 2mm, h: 2mm).f over s in (0, 1))
written := solid(Coil(std.top, r: 10mm, pitch: 5mm, turns: 2, w: 2mm, h: 2mm).f
                 over s in (0, 1))");
    near(exact(&e, "on"), 4.0 * (40.0 * PI).hypot(10.0), 1e-9, "the tube in `std.On`'s frame");
    same_corners(&e, "on", "written");
}

/// **The frame is carried across a chain's joints** (#363 S2, S4): a 2 × 2 square standing
/// 1 to 3 off a U of three lines 20 long and two left quarter turns of R25, toward their centres —
/// its frame integrated over each link from where the last one ended — sweeps its area along its
/// centroid's path, 4·(60 + 2·(π/2)·23) mm³.
#[test]
fn the_frame_is_carried_across_a_chains_joints() {
    let e = with("
component Box(g: path, up: axis, s: Scalar) {
  st := std.On(g, up: up, s: s)
  in st.n {
    a := point
    b := point
    c := point
    d := point
  }
  a coords(du: 1mm, dv: -1mm) st.n
  b coords(du: 3mm, dv: -1mm) st.n
  c coords(du: 3mm, dv: 1mm) st.n
  d coords(du: 1mm, dv: 1mm) st.n
  f := face(a, b, c, d, -> close)
}
in std.top {
  a := point
  b := point hint((20, 0))
  c1 := point hint((20, 25))
  d := point hint((45, 25))
  e := point hint((45, 45))
  c2 := point hint((20, 45))
  f := point hint((20, 70))
  g := point hint((0, 70))
  fix((0mm, 0mm)) a
  route := horizontal (l0 := line(a, b)) -> tangent (k1 := arc(center: c1)) ->
    tangent vertical (l2 := line(d, e)) -> tangent (k3 := arc(center: c2)) ->
    tangent horizontal (l4 := line(f, g))
  length(20mm) l0
  length(20mm) l2
  length(20mm) l4
  radius(25mm) k1
  radius(25mm) k3
}
box := solid(Box(route, std.z).f over s in (0, 5))");
    near(exact(&e, "box"), 4.0 * (60.0 + 23.0 * PI), 1e-9, "the box round the U");
}

/// The four corners of `b` nearest `at`: the corners of its cap there.
fn cap(b: &gcs_core::brep::topo::Brep, at: V) -> Vec<V> {
    let mut ps: Vec<V> = b.vertices.iter().map(|v| v.p).collect();
    ps.sort_by(|p, q| dot(sub(*p, at), sub(*p, at)).total_cmp(&dot(sub(*q, at), sub(*q, at))));
    ps.truncate(4);
    ps
}

/// The turn of a 2 × 1 rectangle at the cap at `s` against the Frenet frame of the helix of
/// radius `r` rising `pitch` a turn over `turns`, from the principal normal toward the binormal,
/// read off its long side (so only to within a half turn).
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

/// An angle brought within a quarter turn either way of zero.
fn wrapped(a: f64) -> f64 {
    let a = a.rem_euclid(PI);
    if a > PI / 2.0 { a - PI } else { a }
}

/// **The frame along a traced helix falls behind Frenet's at the torsion**: over a turn of a
/// helix of radius 10 and pitch 5 the rotation-minimising frame turns 2πb/√(r² + b²) = 28.5576°
/// against the helix's Frenet frame, b = pitch/2π — read off the caps of a 2 × 1 rectangle swept
/// along the guide traced in space, as `tests/rates.rs` reads it off `std.OnHelix`'s.
#[test]
fn the_frame_along_a_traced_helix_turns_against_frenet_at_the_torsion() {
    let e = with("
guide := Helix(o, xl, yl, zl, r: 10mm, pitch: 5mm, turns: 1).p over u in (0, 1)
ribbon := solid(Station(guide, std.z, w: 2mm, h: 1mm).f over s in (0, 1))");
    let b = 5.0 / (2.0 * PI);
    let per_turn = -2.0 * PI * b / (100.0 + b * b).sqrt();
    assert!((per_turn.to_degrees() + 28.5576).abs() < 1e-4, "{}", per_turn.to_degrees());
    let k = brep(&e, "ribbon");
    let got = wrapped(turn(&k, 10.0, 5.0, 1.0, 1.0) - turn(&k, 10.0, 5.0, 1.0, 0.0));
    assert!((got - per_turn).abs() <= 1e-9, "a turn: {got} against {per_turn}");
    near(exact(&e, "ribbon"), 2.0 * (20.0 * PI).hypot(5.0), 1e-9, "the ribbon");
}

/// The errors the document `src` is refused with, each with its code.
fn refusals(src: &str) -> Vec<String> {
    build(src).errors().map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect()
}

/// **A surface traced in space is no path**: a body over two formals has no one place to stand a
/// station at, so handed to a `path` formal it is refused at the argument, saying what a path is.
/// And a curve's place is said once: `t` is its formal, so `t` beside `u` is the number twice.
#[test]
fn what_is_refused() {
    let said = refusals(&format!("{PATHS}
component Sheet(o: point, x: line, y: line, z: line, a: Scalar, b: Scalar) {{
  p := point hint((a, b, 0))
  o distance(a * 1mm, along: x) p
  o distance(b * 1mm, along: y) p
  o distance(0mm, along: z) p
}}
F := Sheet(o, xl, yl, zl).p over (a in (-6, 6), b in (-6, 6))
st := Station(F, std.z, w: 2mm, h: 1mm, s: 0.5)
"));
    assert!(said.iter().any(|m| m.starts_with("E103") && m.contains("`g` is a path")
                                && m.contains("not a surface traced in space")), "{said:?}");
    let said = refusals(&format!("{PATHS}
guide := Helix(o, xl, yl, zl, r: 10mm, pitch: 5mm, turns: 1).p over u in (0, 1)
m := point hint((10, 0, 0))
m coincident(t == 0.25, u == 0.5) guide
"));
    assert!(said.iter().any(|m| m.contains("`u` is given twice")), "{said:?}");
}
