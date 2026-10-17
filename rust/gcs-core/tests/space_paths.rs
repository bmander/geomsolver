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
/// `w` × `h` rectangle centred in it; the same rectangle in `std.Minimal`'s plane over
/// `std.OnHelix`'s station; and a round section of radius `r` in `std.On`'s plane on any path.
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

component Facing(base: plane, r: Length, pitch: Length, turns: Scalar, w: Length, h: Length,
                 s: Scalar) {
  st := std.OnHelix(base, r: r, pitch: pitch, turns: turns, s: s)
  sec := Rect(st.n, w: w, h: h)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

component Frenet(g: path, w: Length, h: Length, s: Scalar) {
  st := std.Frenet(g, s: s)
  sec := Rect(st.n, w: w, h: h)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

component Rider(g: path, up: axis, w: Length, h: Length, s: Scalar) {
  st := std.On(g, up: up, s: s)
  sec := Rect(st.n, w: w, h: h)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

component Round(g: path, up: axis, r: Length, s: Scalar) {
  st := std.On(g, up: up, s: s)
  in st.n {
    k := circle(center: st.n.origin)
    radius(r) k
  }
  f := face(k)
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
guide := trace(Helix(o, xl, yl, zl, r: 10mm, pitch: 5mm, turns: 1).p over u in (0, 1))
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
guide := trace(Helix(o, xl, yl, zl, r: 10mm, pitch: 5mm, turns: 2).p over u in (0, 1))
param s: Scalar over (0, 1)
traced := sweep(Station(guide, std.z, w: 2mm, h: 2mm).f over s)
written := sweep(Coil(std.top, r: 10mm, pitch: 5mm, turns: 2, w: 2mm, h: 2mm).f
                 over s)");
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
guide := trace(Helix(o, xl, yl, zl, r: 10mm, pitch: 5mm, turns: 2).p over u in (0, 1))
param s: Scalar over (0, 1)
on := sweep(Rider(guide, std.z, w: 2mm, h: 2mm).f over s)
written := sweep(Coil(std.top, r: 10mm, pitch: 5mm, turns: 2, w: 2mm, h: 2mm).f
                 over s)");
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
param s: Scalar over (0, 5)
box := sweep(Box(route, std.z).f over s)");
    near(exact(&e, "box"), 4.0 * (60.0 + 23.0 * PI), 1e-9, "the box round the U");
}

/// **Frenet's frame along a traced helix is `std.OnHelix`'s** (#363 S5): `std.Frenet` stands its
/// plane square to the curve with `u` toward the centre of curvature (`g curvature(at: m) c`, no
/// seed saying which way) and `v` the binormal, so along the helix of `lofts/ribbon.sv` — radius
/// 12, pitch 6, three turns — a 4 × 0.5 ribbon in it is that example's `facing` solid, corner for
/// corner (`OnHelix`'s `u` runs out from the axis, the ribbon centred on it either way), at its
/// area along the helix's length.  And the centre where it stands is the closed form's, the
/// radius of curvature (r² + b²)/r in from the curve toward the axis.
#[test]
fn frenets_frame_along_a_traced_helix_is_on_helixs() {
    let e = with("
guide := trace(Helix(o, xl, yl, zl, r: 12mm, pitch: 6mm, turns: 3).p over u in (0, 1))
param s: Scalar over (0, 1)
frenet := sweep(Frenet(guide, w: 4mm, h: 0.5mm).f over s)
facing := sweep(Facing(std.top, r: 12mm, pitch: 6mm, turns: 3, w: 4mm, h: 0.5mm).f
                over s)
st := std.Frenet(guide, s: 0.3)");
    near(exact(&e, "frenet"), 2.0 * 3.0 * (24.0 * PI).hypot(6.0), 1e-9, "the ribbon");
    same_corners(&e, "frenet", "facing");
    let at = |n: &str| e.sketch.world_point(e.map.ent_named(n).unwrap().i());
    let (m, c) = (at("st.m"), at("st.c"));
    let b = 6.0 / (2.0 * PI);
    let rho = (144.0 + b * b) / 12.0;
    let th = 2.0 * PI * 3.0 * 0.3;
    let want = [12.0 * th.cos() - rho * th.cos(), 12.0 * th.sin() - rho * th.sin(), 5.4];
    assert!(sub(m, [12.0 * th.cos(), 12.0 * th.sin(), 5.4]).iter().all(|d| d.abs() <= 1e-9));
    assert!(sub(c, want).iter().all(|d| d.abs() <= 1e-9), "{c:?} against {want:?}");
}

/// **Frenet's frame is refused where the curve has no curvature**: a traced curve straight up to
/// a third of its run and bending after it has no centre of curvature there, so the station run
/// down from the middle loses its rank where the bend ends and is refused at that station, never
/// followed past it.  And a surface has no one centre at a point.
#[test]
fn frenets_frame_is_refused_on_a_straight_run() {
    let e = build(&format!("{PATHS}
component Bent(o: point, x: line, y: line, z: line, u: Scalar) {{
  p := point hint((20 * u, 10 * max(u - 0.3, 0)^3, 0))
  o distance(20mm * u, along: x) p
  o distance(10mm * max(u - 0.3, 0)^3, along: y) p
  o distance(0mm, along: z) p
}}
bent := trace(Bent(o, xl, yl, zl).p over u in (0, 1))
param s: Scalar over (0, 1)
ribbon := sweep(Frenet(bent, w: 2mm, h: 0.5mm).f over s)
"));
    assert!(e.ok(), "{:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    let why = e.sketch.exact_solid(index(&e, "ribbon")).err().expect("the ribbon was built");
    assert!(why.contains("the station at"), "{why}");
    let said = refusals(&format!("{PATHS}
component Sheet(o: point, x: line, y: line, z: line, a: Scalar, b: Scalar) {{
  p := point hint((a, b, 0))
  o distance(a * 1mm, along: x) p
  o distance(b * 1mm, along: y) p
  o distance(0mm, along: z) p
}}
F := trace(Sheet(o, xl, yl, zl).p over (a in (-6, 6), b in (-6, 6)))
m := point hint((1, 1, 0))
m coincident F
c := point
F curvature(at: m) c
"));
    assert!(said.iter().any(|m| m.contains("no one centre of curvature")), "{said:?}");
}

/// The cubic Bézier over `ctrl` at `t`: its point and first two derivatives, Bernstein's.
fn bezier(ctrl: &[V; 4], t: f64) -> [V; 3] {
    let u = 1.0 - t;
    let mix = |w: [f64; 4]| -> V {
        std::array::from_fn(|k| (0..4).map(|i| w[i] * ctrl[i][k]).sum())
    };
    [
        mix([u * u * u, 3.0 * t * u * u, 3.0 * t * t * u, t * t * t]),
        mix([-3.0 * u * u, 3.0 * u * u - 6.0 * t * u, 6.0 * t * u - 3.0 * t * t, 3.0 * t * t]),
        mix([6.0 * u, 6.0 * t - 12.0 * u, 6.0 * u - 12.0 * t, 6.0 * t]),
    ]
}

/// `∫ f` over `(a, b)` by five-point Gauss–Legendre over 400 panels: to the last digit for a
/// curve's speed.
pub fn integral(a: f64, b: f64, f: impl Fn(f64) -> f64) -> f64 {
    let (x, w) = ([-0.906179845938664, -0.538469310105683, 0.0, 0.538469310105683,
                   0.906179845938664],
                  [0.236926885056189, 0.478628670499366, 0.568888888888889, 0.478628670499366,
                   0.236926885056189]);
    let n = 400;
    (0..n).map(|k| {
        let (lo, hi) = (a + (b - a) * k as f64 / n as f64, a + (b - a) * (k + 1) as f64 / n as f64);
        let half = 0.5 * (hi - lo);
        (0..5).map(|i| w[i] * f(0.5 * (lo + hi) + half * x[i])).sum::<f64>() * half
    }).sum()
}

/// Four control points in space, a Bézier over them, and points held on it.
const CUBIC: &str = "
ka := point hint((0, 0, 0))
kb := point hint((10, 20, 5))
kc := point hint((30, 20, 15))
kd := point hint((40, 0, 30))
fix((0mm, 0mm, 0mm)) ka
fix((10mm, 20mm, 5mm)) kb
fix((30mm, 20mm, 15mm)) kc
fix((40mm, 0mm, 30mm)) kd
cubic := spline(ka, kb, kc, kd)
";

const CUBIC_CTRL: [V; 4] = [[0.0, 0.0, 0.0], [10.0, 20.0, 5.0], [30.0, 20.0, 15.0],
                            [40.0, 0.0, 30.0]];

/// **A spline over points in space is a curve traced in space** (#363 S3): `spline(a, b, c, d)`
/// with its control points standing in space is the body `Σ Bᵢ(u)·Pᵢ` over their lifts, so its
/// name reads a traced curve (the spline kept, no primitive) whose point, derivatives and length
/// are Bernstein's — to the last digits — and whose contacts are a traced curve's: a point
/// pinned at a place of it where Bernstein puts it, a tangent along `C′`, each against finite
/// differences.
#[test]
fn a_spline_over_points_in_space_is_its_bernstein_form() {
    let e = with(&format!("{CUBIC}
m1 := point hint((5, 5, 1))
m1 coincident(t == 0.25) cubic
m2 := point hint((20, 15, 10))
m2 coincident(t == 0.6) cubic
q := point hint((25, 10, 12))
tl := line(m2, q)
cubic tangent(at: m2) tl
length(10mm) tl"));
    let k = e.map.ent_named("cubic").unwrap();
    assert_eq!(k.kind, gcs_core::model::EntKind::Traced);
    assert!(!e.sketch.primitives().iter().any(|p| p.kind == gcs_core::model::EntKind::Spline));
    let at = |n: &str| e.sketch.world_point(e.map.ent_named(n).unwrap().i());
    let close = |p: V, q: V, tol: f64| sub(p, q).iter().all(|d| d.abs() <= tol);
    assert!(close(at("m1"), bezier(&CUBIC_CTRL, 0.25)[0], 1e-9), "{:?}", at("m1"));
    assert!(close(at("m2"), bezier(&CUBIC_CTRL, 0.6)[0], 1e-9), "{:?}", at("m2"));
    let along = unit(sub(at("q"), at("m2")));
    let want = unit(bezier(&CUBIC_CTRL, 0.6)[1]);
    assert!((dot(along, want).abs() - 1.0).abs() <= 1e-12, "{along:?} against {want:?}");
    // the reading, every order the contacts read
    use gcs_core::traced::{reading, Need};
    for t in [0.0, 0.1, 0.37, 0.5, 0.9, 1.0] {
        let r = reading(&e.sketch, k.i(), [t, 0.0], Need::Curvature).expect("a reading");
        let [x, d1, d2] = bezier(&CUBIC_CTRL, t);
        assert!(close(r.position, x, 1e-12) && close(r.d[0], d1, 1e-11)
                && close(r.dd[0][0], d2, 1e-10), "at {t}: {r:?}");
    }
    let length = integral(0.0, 1.0, |t| {
        let r = reading(&e.sketch, k.i(), [t, 0.0], Need::Tangent).unwrap();
        dot(r.d[0], r.d[0]).sqrt()
    });
    let want = integral(0.0, 1.0, |t| {
        let d = bezier(&CUBIC_CTRL, t)[1];
        dot(d, d).sqrt()
    });
    assert!((length - want).abs() <= 1e-12 * want, "{length} against {want}");
    // its contacts against finite differences, the control points free
    let mut sk = e.sketch.clone();
    for p in sk.params.iter_mut() {
        p.fixed = false;
    }
    // the tangent a rate of the contact holding `m2` (#365)
    let traced: Vec<_> = sk.constraints.iter()
        .filter(|c| c.kind.traced_contact().is_some() || c.rate_source().is_some())
        .cloned().collect();
    assert_eq!(traced.len(), 3, "{:?}", traced.iter().map(|c| c.kind).collect::<Vec<_>>());
    for c in &traced {
        let err = gcs_core::fdcheck::check_constraint(&sk, c, 1e-6, 1e-6)
            .unwrap_or_else(|m| panic!("{:?}: {m}", c.kind));
        assert!(err.is_finite());
    }
}

/// **A spline in space is drawn in the box** (#363 S3): a curve traced in space stands in no view,
/// so the scene in space carries it as an axis is carried — a polyline from its first control
/// point to its last, every vertex on Bernstein's curve, no chord straying from it by more than
/// the flatness it was refined to.
#[test]
fn a_spline_in_space_is_drawn_in_the_box() {
    let e = with(CUBIC);
    let k = e.map.ent_named("cubic").unwrap();
    let unit = 0.01;
    let items = gcs_core::overview::scene3d(&e.sketch, unit);
    let drawn: Vec<_> = items.iter().filter(|it| it.of == Some(k)).collect();
    assert_eq!(drawn.len(), 1, "the curve is drawn once");
    let pts = &drawn[0].pts;
    assert!(pts.len() > 16, "{}", pts.len());
    assert!(sub(pts[0], CUBIC_CTRL[0]).iter().all(|d| d.abs() <= 1e-9));
    assert!(sub(*pts.last().unwrap(), CUBIC_CTRL[3]).iter().all(|d| d.abs() <= 1e-9));
    // on the curve: each vertex where Bernstein puts the place nearest it
    let on = |p: V| -> f64 {
        let t = gcs_core::traced::place_of(&e.sketch, k.i(), p).unwrap()[0];
        dot(sub(bezier(&CUBIC_CTRL, t)[0], p), sub(bezier(&CUBIC_CTRL, t)[0], p)).sqrt()
    };
    let flat = gcs_core::curve::flatness(unit);
    for w in pts.windows(2) {
        assert!(on(w[0]) <= 1e-9, "{:?} is off the curve", w[0]);
        let mid = [0.5 * (w[0][0] + w[1][0]), 0.5 * (w[0][1] + w[1][1]), 0.5 * (w[0][2] + w[1][2])];
        assert!(on(mid) <= 1.01 * flat, "a chord strays {} past {flat}", on(mid));
    }
}

/// **A tube along a spline in space is its area along its length**: a round section of radius
/// 3 square to the Bézier above, in `std.On`'s frame, sweeps 9π times the curve's length
/// (Gauss–Legendre on Bernstein's derivative).
#[test]
fn a_tube_along_a_spline_in_space_is_its_area_along_its_length() {
    let e = with(&format!("{CUBIC}
param s: Scalar over (0, 1)
tube := sweep(Round(cubic, std.z, r: 3mm).f over s)"));
    let length = integral(0.0, 1.0, |t| {
        let d = bezier(&CUBIC_CTRL, t)[1];
        dot(d, d).sqrt()
    });
    near(exact(&e, "tube"), 9.0 * PI * length, 1e-9, "the tube");
}

/// The clamped uniform cubic B-spline basis of `m` control points at `t`, its spans of unit
/// length, and its first two derivatives — read off four samples a quarter apart in the span at
/// `t`, toward the inside (`dir`): exact for a cubic.
fn basis3(m: usize, t: f64, dir: f64) -> [Vec<f64>; 3] {
    let spans = m - 3;
    let mut u = vec![0.0; 4];
    u.extend((1..spans).map(|i| i as f64));
    u.extend([spans as f64; 4]);
    fn n(i: usize, p: usize, t: f64, u: &[f64]) -> f64 {
        if p == 0 {
            return if u[i] <= t && t < u[i + 1] { 1.0 } else { 0.0 };
        }
        let a = if u[i + p] > u[i] { (t - u[i]) / (u[i + p] - u[i]) * n(i, p - 1, t, u) }
            else { 0.0 };
        let b = if u[i + p + 1] > u[i + 1] {
            (u[i + p + 1] - t) / (u[i + p + 1] - u[i + 1]) * n(i + 1, p - 1, t, u)
        } else { 0.0 };
        a + b
    }
    let at = |t: f64| -> Vec<f64> {
        let t = t.min(spans as f64 - 1e-12);
        (0..m).map(|i| n(i, 3, t, &u)).collect()
    };
    let h = 0.25;
    let b: Vec<Vec<f64>> = (0..4).map(|i| at(t + dir * h * i as f64)).collect();
    let d1 = (0..m).map(|i| dir * (-11.0 * b[0][i] + 18.0 * b[1][i] - 9.0 * b[2][i]
                                   + 2.0 * b[3][i]) / (6.0 * h)).collect();
    let d2 = (0..m).map(|i| (2.0 * b[0][i] - 5.0 * b[1][i] + 4.0 * b[2][i] - b[3][i]) / (h * h))
        .collect();
    [b[0].clone(), d1, d2]
}

/// The clamped uniform cubic through the corners `pts` in space, each at its index, its ends
/// along `start` and `end` where given — `C′ × T = 0`, `C″ · T = 0` — and else natural, `C″ = 0`:
/// stated on the curve's own derivatives and solved here, apart from the core's construction (L7's
/// method, in space).  Its control points.
pub fn through3(pts: &[V], start: Option<V>, end: Option<V>) -> Vec<V> {
    let n = pts.len();
    let m = n + 2;
    let mut rows: Vec<(Vec<f64>, f64)> = Vec::new();
    let row = |w: &[f64], d: V| {
        let mut r = vec![0.0; 3 * m];
        for (i, wi) in w.iter().enumerate() {
            for c in 0..3 {
                r[3 * i + c] = wi * d[c];
            }
        }
        r
    };
    let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for (k, p) in pts.iter().enumerate() {
        let [b, _, _] = basis3(m, k as f64, 1.0);
        for c in 0..3 {
            rows.push((row(&b, axes[c]), p[c]));
        }
    }
    for (tangent, at, dir) in [(start, 0.0, 1.0), (end, (n - 1) as f64, -1.0)] {
        let [_, d1, d2] = basis3(m, at, dir);
        match tangent {
            None => {
                for a in axes {
                    rows.push((row(&d2, a), 0.0));
                }
            }
            Some(t) => {
                let t = unit(t);
                let k = (0..3).min_by(|&a, &b| t[a].abs().total_cmp(&t[b].abs())).unwrap();
                let e1 = unit(cross(t, axes[k]));
                let e2 = cross(t, e1);
                rows.push((row(&d1, e1), 0.0));
                rows.push((row(&d1, e2), 0.0));
                rows.push((row(&d2, t), 0.0));
            }
        }
    }
    let x = gauss_solve(rows);
    (0..m).map(|i| [x[3 * i], x[3 * i + 1], x[3 * i + 2]]).collect()
}

/// Gaussian elimination with partial pivoting of a square system.
fn gauss_solve(rows: Vec<(Vec<f64>, f64)>) -> Vec<f64> {
    let n = rows.len();
    let mut a: Vec<Vec<f64>> = rows.iter().map(|(r, b)| { let mut r = r.clone(); r.push(*b); r })
        .collect();
    assert_eq!(a[0].len(), n + 1, "a square system");
    for col in 0..n {
        let p = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs())).unwrap();
        a.swap(col, p);
        for i in 0..n {
            if i != col {
                let f = a[i][col] / a[col][col];
                for j in col..=n {
                    a[i][j] -= f * a[col][j];
                }
            }
        }
    }
    (0..n).map(|i| a[i][n] / a[i][i]).collect()
}

/// **A spline through corners in space** (#363 S3, L7's smooth rail out of its plane): over four
/// corners standing in space, natural at both ends, and over three, along lines in space at its
/// ends — each the cubic solved apart from the core, control point for control point, its ends
/// as the conditions say.
#[test]
fn a_spline_through_corners_in_space_is_the_cubic_through_them() {
    let e = with("
r0 := point
r1 := point
r2 := point
r3 := point
fix((0mm, 0mm, 0mm)) r0
fix((20mm, 10mm, 5mm)) r1
fix((40mm, 0mm, 20mm)) r2
fix((50mm, -10mm, 40mm)) r3
rims := line(r0, r1) -> line(r1, r2) -> line(r2, r3)
rail := spline(through: rims)
s0 := point
s1 := point
fix((0mm, -10mm, 5mm)) s0
fix((5mm, -3mm, 9mm)) s1
e0 := point
e1 := point
fix((40mm, 0mm, 20mm)) e0
fix((40mm, 6mm, 30mm)) e1
heads := line(s1, s0)
tails := line(e0, e1)
pair := line(r0, r1) -> line(r1, r2)
held := spline(through: pair, start: heads, end: tails)");
    let sk = &e.sketch;
    let ctrl = |name: &str| -> Vec<V> {
        let k = e.map.ent_named(name).unwrap();
        let (&s, _) = sk.spline_traced.iter().find(|(_, &t)| t as usize == k.i()).unwrap();
        sk.splines[s as usize].ctrl.iter().map(|&p| sk.world_point(p as usize)).collect()
    };
    let corners = [[0.0, 0.0, 0.0], [20.0, 10.0, 5.0], [40.0, 0.0, 20.0], [50.0, -10.0, 40.0]];
    let (start, end) = (Some([-5.0, -7.0, -4.0]), Some([0.0, 6.0, 10.0]));
    let cases = [("rail", through3(&corners, None, None)),
                 ("held", through3(&corners[..3], start, end))];
    for (name, want) in cases {
        let got = ctrl(name);
        assert_eq!(got.len(), want.len(), "`{name}`");
        for (g, w) in got.iter().zip(&want) {
            assert!(sub(*g, *w).iter().all(|d| d.abs() <= 1e-9),
                    "`{name}`: {got:?} against {want:?}");
        }
    }
    use gcs_core::traced::{reading, Need};
    let k = e.map.ent_named("rail").unwrap();
    for t in [0.0, 3.0] {
        let r = reading(sk, k.i(), [t, 0.0], Need::Curvature).unwrap();
        assert!(dot(r.dd[0][0], r.dd[0][0]).sqrt() <= 1e-9, "`rail` bends at {t}: {r:?}");
    }
    for (t, p) in corners.iter().enumerate() {
        let r = reading(sk, k.i(), [t as f64, 0.0], Need::Point).unwrap();
        assert!(sub(r.position, *p).iter().all(|d| d.abs() <= 1e-9), "{t}: {r:?}");
    }
}

/// **A spline through corners in space solves from its seeds whatever its ends' tangents**: the
/// foot each tangent end is built over may cross its corner on the way to the answer, which a line
/// from the corner to it could not follow (#363 S3) — so over three corners in space, each end
/// given each of a spread of directions — over three corners and over four — the cubic is the one
/// solved apart, every time.
#[test]
fn a_spline_through_corners_in_space_solves_for_any_tangents() {
    let dirs: [V; 6] = [[0.0, 6.0, 10.0], [0.0, 6.0, -10.0], [-5.0, -7.0, -4.0], [1.0, 0.0, 0.0],
                        [3.0, -2.0, 7.0], [-4.0, 9.0, 1.0]];
    let all = [[0.0, 0.0, 0.0], [20.0, 10.0, 5.0], [40.0, 0.0, 20.0], [50.0, -10.0, 40.0]];
    // over two links and over three, an interior knot or two between the ends
    for n in [3usize, 4] {
        let corners = &all[..n];
        let (last, chain) = if n == 3 {
            (corners[2], "pair := line(r0, r1) -> line(r1, r2)")
        } else {
            (corners[3], "pair := line(r0, r1) -> line(r1, r2) -> line(r2, r3)")
        };
        for (i, ds) in dirs.iter().enumerate() {
            for de in &dirs[i..] {
                let e = with(&format!("
r0 := point
r1 := point
r2 := point
r3 := point
fix((0mm, 0mm, 0mm)) r0
fix((20mm, 10mm, 5mm)) r1
fix((40mm, 0mm, 20mm)) r2
fix((50mm, -10mm, 40mm)) r3
s1 := point
fix(({}mm, {}mm, {}mm)) s1
e1 := point
fix(({}mm, {}mm, {}mm)) e1
heads := line(r0, s1)
tails := line(r{}, e1)
{chain}
held := spline(through: pair, start: heads, end: tails)",
                    ds[0], ds[1], ds[2], last[0] + de[0], last[1] + de[1], last[2] + de[2],
                    n - 1));
                let sk = &e.sketch;
                let k = e.map.ent_named("held").unwrap();
                let (&sp, _) = sk.spline_traced.iter().find(|(_, &t)| t as usize == k.i())
                    .unwrap();
                let got: Vec<V> = sk.splines[sp as usize].ctrl.iter()
                    .map(|&p| sk.world_point(p as usize)).collect();
                let want = through3(corners, Some(*ds), Some(*de));
                assert_eq!(got.len(), want.len());
                for (g, w) in got.iter().zip(&want) {
                    assert!(sub(*g, *w).iter().all(|d| d.abs() <= 1e-9),
                            "{n} corners, start {ds:?}, end {de:?}: {got:?} against {want:?}");
                }
            }
        }
    }
}

/// The 3D pipe of #363 S4: a line 20 long, a quarter bend of R20 in `std.top`, a line 20 long,
/// a quarter bend of R20 in the plane x = 20, and a line 20 long up `z` — one chain, its
/// straight between the bends touching each in its own plane — and stations on it.
const PIPE: &str = "
P2 := plane
fix(origin == (20mm, 0mm, 0mm)) P2
fix(dir == (0, 1, 0)) P2.u
fix(dir == (0, 0, 1)) P2.v
a := point hint((-20, 0, 0))
b := point in std.top hint((0, 0))
fix((0mm, 0mm)) b
c1 := point in std.top hint((0, 20))
fix((0mm, 20mm)) c1
d := point in std.top hint((20, 20))
e := point in P2 hint((40, 0))
c2 := point in P2 hint((40, 20))
f := point in P2 hint((60, 20))
g := point hint((20, 60, 40))
route := (l0 := line(a, b)) -> tangent (k1 := arc(center: c1)) ->
  tangent (l2 := line(d, e)) -> tangent (k3 := arc(center: c2)) ->
  tangent (l4 := line(f, g))
length(20mm) l0
length(20mm) l2
length(20mm) l4
radius(20mm) k3
l4 parallel std.z
component Bar(g: path, up: axis, lo: Length, hi: Length, s: Scalar) {
  st := std.On(g, up: up, s: s)
  sec := Span(st.n, lo: lo, hi: hi)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}
component Span(n: plane, lo: Length, hi: Length) {
  in n {
    a := point
    b := point
    c := point
    d := point
  }
  a coords(du: lo, dv: -1mm) n
  b coords(du: hi, dv: -1mm) n
  c coords(du: hi, dv: 1mm) n
  d coords(du: lo, dv: 1mm) n
}
";

/// **A pipe bent in two planes** (#363 S4): the straight between the bends touches each in its own
/// plane (`TangentCircleAt3`), so the five are one chain and a family runs it.  A 4 mm round tube
/// along it is its area along its length, 16π·(60 + 20π) = 6174.20 mm³; a 2 × 2 square under
/// `std.On`'s frame is 4·(60 + 20π) = 491.33 mm³.  And the frame has a closed form: it starts
/// with `u` square to `z` on the first straight (+y, `v` +z), turns a quarter about `z` round the
/// first bend and a quarter about `x` round the second, and ends with the tangent +z, `u` −x and
/// `v` −y — read off the end cap of a bar 3 wide along `u` and 2 along `v`.
#[test]
fn a_pipe_bent_in_two_planes_is_one_chain() {
    let e = with(&format!("{PIPE}
param s: Scalar over (0, 5)
tube := sweep(Round(route, std.z, r: 4mm).f over s)
square := sweep(Bar(route, std.z, lo: -1mm, hi: 1mm).f over s)
bar := sweep(Bar(route, std.z, lo: -1mm, hi: 2mm).f over s)"));
    let at = |n: &str| e.sketch.world_point(e.map.ent_named(n).unwrap().i());
    for (n, want) in [("a", [-20.0, 0.0, 0.0]), ("d", [20.0, 20.0, 0.0]), ("e", [20.0, 40.0, 0.0]),
                      ("f", [20.0, 60.0, 20.0]), ("g", [20.0, 60.0, 40.0])] {
        assert!(sub(at(n), want).iter().all(|d| d.abs() <= 1e-9), "{n}: {:?}", at(n));
    }
    let length = 60.0 + 20.0 * PI;
    near(exact(&e, "tube"), 16.0 * PI * length, 1e-9, "the round tube");
    near(exact(&e, "square"), 4.0 * length, 1e-9, "the square tube");
    // the bar's end cap: g + du·u + dv·v with u = −x, v = −y
    let cap: Vec<V> = brep(&e, "bar").vertices.iter().map(|v| v.p)
        .filter(|p| (p[2] - 40.0).abs() <= 1e-6).collect();
    assert_eq!(cap.len(), 4, "{cap:?}");
    for (du, dv) in [(-1.0, -1.0), (2.0, -1.0), (2.0, 1.0), (-1.0, 1.0)] {
        let want = [20.0 - du, 60.0 - dv, 40.0];
        let d = cap.iter().map(|p| dot(sub(*p, want), sub(*p, want)).sqrt())
            .fold(f64::MAX, f64::min);
        assert!(d <= 1e-9, "the corner at ({du}, {dv}) is {d} from {want:?}: {cap:?}");
    }
    // the bar's centroid stands ½ toward the first bend's centre and square to the second's plane
    near(exact(&e, "bar"), 6.0 * (length - 0.5 * PI / 2.0), 1e-9, "the bar");
}

/// **An arc over points in space is drawn in the plane through them** (#363 S6): the bent pipe of
/// S4 written with no plane at all — every point in space, each bend's arc over its centre and
/// the ends its chain threads — mints each bend's plane through its centre and ends, so the chain
/// is the one the planes made, and its tube is Pappus's, 16π(60 + 20π).  The arc is an arc there:
/// its radius 20, its plane square to `z` for the first bend and to `x` for the second.
#[test]
fn an_arc_over_points_in_space_is_drawn_in_the_plane_through_them() {
    let e = with("
a := point hint((-20, 0, 0))
b := point hint((0, 0, 0))
fix((0mm, 0mm, 0mm)) b
c1 := point hint((0, 20, 0))
fix((0mm, 20mm, 0mm)) c1
d := point hint((20, 20, 0))
e := point hint((20, 40, 0))
c2 := point hint((20, 40, 20))
f := point hint((20, 60, 20))
g := point hint((20, 60, 40))
route := (l0 := line(a, b)) -> tangent (k1 := arc(center: c1)) ->
  tangent (l2 := line(d, e)) -> tangent (k3 := arc(center: c2)) ->
  tangent (l4 := line(f, g))
length(20mm) l0
length(20mm) l2
length(20mm) l4
radius(20mm) k3
l4 parallel std.z
l2 parallel std.y
d coincident std.top
param s: Scalar over (0, 5)
tube := sweep(Round(route, std.z, r: 4mm).f over s)");
    near(exact(&e, "tube"), 16.0 * PI * (60.0 + 20.0 * PI), 1e-9, "the tube");
    let sk = &e.sketch;
    for (name, normal) in [("k1", [0.0, 0.0, 1.0]), ("k3", [1.0, 0.0, 0.0])] {
        let k = e.map.ent_named(name).unwrap();
        assert!((sk.radius_value(k).abs() - 20.0).abs() <= 1e-9, "`{name}`'s radius");
        let pl = sk.plane_of(sk.round_center(k)).expect("drawn in a plane");
        let n = sk.basis(pl).normal();
        assert!((dot(unit(n), normal).abs() - 1.0).abs() <= 1e-9, "`{name}`'s plane: {n:?}");
    }
}

/// L0's S-bend drawn in space, no plane written: a straight, a left turn, a straight, a right
/// turn, a straight, each 20 and each bend of radius 20, its third link `k3` as given — and a
/// round tube along it.
fn s_bend_in_space(k3: &str) -> String {
    "
a := point hint((-20, 0, 0))
b := point hint((0, 0, 0))
fix((0mm, 0mm, 0mm)) b
c1 := point hint((0, 20, 0))
fix((0mm, 20mm, 0mm)) c1
d := point hint((20, 20, 0))
e := point hint((20, 40, 0))
c2 := point hint((40, 40, 0))
f := point hint((40, 60, 0))
g := point hint((60, 60, 0))
route := (l0 := line(a, b)) -> tangent (k1 := arc(center: c1)) ->
  tangent (l2 := line(d, e)) -> tangent K3 ->
  tangent (l4 := line(f, g))
length(20mm) l0
length(20mm) l2
length(20mm) l4
radius(20mm) k3
d coincident std.top
l2 parallel std.y
l4 parallel std.x
param s: Scalar over (0, 5)
tube := sweep(Round(route, std.z, r: 4mm).f over s)"
        .replace("K3", k3)
}

/// The same S-bend drawn in `std.top`'s block, `before` written in the block ahead of the chain.
fn s_bend_in_top(before: &str, k3: &str) -> String {
    "
in std.top {
  a := point
  b := point hint((20, 0))
  c1 := point hint((20, 20))
  d := point hint((40, 20))
  e := point hint((40, 40))
  c2 := point hint((60, 40))
  f := point hint((60, 60))
  g := point hint((80, 60))
  fix((0mm, 0mm)) a
  BEFORE
  route := horizontal (l0 := line(a, b)) -> tangent (k1 := arc(center: c1)) ->
    tangent vertical (l2 := line(d, e)) -> tangent K3 ->
    tangent horizontal (l4 := line(f, g))
  length(20mm) l0
  length(20mm) l2
  length(20mm) l4
  radius(20mm) k1
  radius(20mm) k3
}
param s: Scalar over (0, 5)
tube := sweep(Round(route, std.z, r: 4mm).f over s)"
        .replace("BEFORE", before).replace("K3", k3)
}

/// **In space a chain turns either way at an arc** (#363 S4, by S6): L4's rule — an arc runs
/// counter-clockwise from its start, so a chain drawn in a plane turns left at every one — holds
/// of an arc drawn in a plane; one over points in space faces its plane as its points do.  So
/// L0's S-bend, a left turn then a right, is one chain drawn in space: its tube is Pappus's, and
/// its second bend's plane faces down.
#[test]
fn in_space_a_chain_turns_either_way_at_an_arc() {
    let e = with(&s_bend_in_space("(k3 := arc(center: c2))"));
    near(exact(&e, "tube"), 16.0 * PI * (60.0 + 20.0 * PI), 1e-9, "the S-bend");
    let at = |n: &str| e.sketch.world_point(e.map.ent_named(n).unwrap().i());
    assert!(sub(at("g"), [60.0, 60.0, 0.0]).iter().all(|d| d.abs() <= 1e-9), "{:?}", at("g"));
    let sk = &e.sketch;
    for (name, up) in [("k1", 1.0), ("k3", -1.0)] {
        let k = e.map.ent_named(name).unwrap();
        let n = sk.basis(sk.plane_of(sk.round_center(k)).unwrap()).normal();
        assert!((unit(n)[2] - up).abs() <= 1e-9, "`{name}`'s plane faces {n:?}");
    }
}

/// **A chain walks an arc it enters at its end from there** (#363 S4): L4's rule — a chain enters
/// each link at its start — read an arc entered at its end as no link at all; walked end to start
/// it is drawn from there, its turn reversed, the same arc.  So the S-bend in `std.top` is one
/// chain with its right turn written as the arc from `f` to `e`, counter-clockwise — declared in
/// the chain, or named there — its tube Pappus's, and a name still reads the points as written
/// (`claim k3.start coincident f`).  The bent pipe's second bend written from `f`, clockwise, is
/// the quarter it was: walked back it turns counter-clockwise from `e`, and the square tube and
/// the round one are the pipe's.  In space the arc from `f` to `e` faces its plane the other way
/// once walked, so L0's S-bend there is the one before.  And an arc two chains walk opposite ways
/// is refused: it is drawn one way.
#[test]
fn a_chain_walks_an_arc_it_enters_at_its_end_from_there() {
    let length = 60.0 + 20.0 * PI;
    for (before, link) in [("", "(k3 := arc(center: c2, start: f, end: e))"),
                           ("k3 := arc(center: c2, start: f, end: e)", "k3")] {
        let mut e = with(&format!("{}
claim k3.start coincident f
claim k3.end coincident e", s_bend_in_top(before, link)));
        near(exact(&e, "tube"), 16.0 * PI * length, 1e-9, "the S-bend in a plane");
        let at = |n: &str| e.sketch.world_point(e.map.ent_named(n).unwrap().i());
        assert!(sub(at("g"), [80.0, 60.0, 0.0]).iter().all(|d| d.abs() <= 1e-9), "{:?}", at("g"));
        assert_eq!(e.sketch.walked_back.len(), 1);
        let opts = gcs_core::diagnose::DiagnoseOptions::default();
        let d = gcs_core::diagnose::diagnose(&mut e.sketch, opts);
        assert!(d.claims_violated.is_empty() && d.claims_theorem.len() == 2, "{d:?}");
    }
    let pipe = PIPE.replace("tangent (k3 := arc(center: c2)) ->",
                            "tangent (k3 := arc(center: c2, start: f, end: e, sense: cw)) ->");
    let e = with(&format!("{pipe}
param s: Scalar over (0, 5)
tube := sweep(Round(route, std.z, r: 4mm).f over s)
square := sweep(Bar(route, std.z, lo: -1mm, hi: 1mm).f over s)"));
    near(exact(&e, "tube"), 16.0 * PI * length, 1e-9, "the bent pipe's round tube");
    near(exact(&e, "square"), 4.0 * length, 1e-9, "the bent pipe's square tube");
    let at = |n: &str| e.sketch.world_point(e.map.ent_named(n).unwrap().i());
    assert!(sub(at("g"), [20.0, 60.0, 40.0]).iter().all(|d| d.abs() <= 1e-9), "{:?}", at("g"));
    let e = with(&s_bend_in_space("(k3 := arc(center: c2, start: f, end: e))"));
    near(exact(&e, "tube"), 16.0 * PI * length, 1e-9, "the S-bend in space");
    let sk = &e.sketch;
    let k = e.map.ent_named("k3").unwrap();
    let n = sk.basis(sk.plane_of(sk.round_center(k)).unwrap()).normal();
    assert!((unit(n)[2] + 1.0).abs() <= 1e-9, "`k3`'s plane faces {n:?}");
    let said = refusals(&format!("{PATHS}
in std.top {{
  p0 := point hint((0, 0))
  p1 := point hint((10, 0))
  p2 := point hint((20, 10))
  p3 := point hint((20, 20))
  p4 := point hint((10, 30))
  q := point hint((10, 10))
  k := arc(center: q, start: p1, end: p2)
  one := (m0 := line(p0, p1)) -> k -> (m1 := line(p2, p3))
  two := (m2 := line(p3, p2)) -> k -> (m3 := line(p1, p4))
}}
"));
    assert!(said.iter().any(|m| m.starts_with("E080") && m.contains("drawn one way")), "{said:?}");
}

/// **A chain in a plane turns right at an arc said to turn clockwise** (#363 S4): `arc(center: c,
/// sense: cw)` drawn in `std.top` runs clockwise from its start — drawn counter-clockwise in the
/// plane turned over — so the S-bend drawn in the plane's block is one chain too, a family entering
/// every link at its start: its tube is Pappus's.  `sense:` of an arc over points in space, which
/// turns as its points stand, is refused, and so is a clockwise half turn.
#[test]
fn a_chain_in_a_plane_turns_right_at_an_arc_said_to_turn_clockwise() {
    let e = with(&s_bend_in_top("", "(k3 := arc(center: c2, sense: cw))"));
    near(exact(&e, "tube"), 16.0 * PI * (60.0 + 20.0 * PI), 1e-9, "the S-bend in a plane");
    let at = |n: &str| e.sketch.world_point(e.map.ent_named(n).unwrap().i());
    assert!(sub(at("g"), [80.0, 60.0, 0.0]).iter().all(|d| d.abs() <= 1e-9), "{:?}", at("g"));
    let said = refusals(&format!("{PATHS}
p0 := point hint((0, 0, 0))
p1 := point hint((5, 0, 0))
p2 := point hint((5, 5, 3))
k := arc(center: p1, start: p0, end: p2, sense: cw)
"));
    assert!(said.iter().any(|m| m.starts_with("E040") && m.contains("`sense:` says which way")),
            "{said:?}");
    let said = refusals(&format!("{PATHS}
in std.top {{
  q0 := point hint((0, 0))
  q1 := point hint((5, 0))
  q2 := point hint((10, 0))
  k := arc(center: q1, start: q0, end: q2, sense: cw)
}}
"));
    assert!(said.iter().any(|m| m.starts_with("E060") && m.contains("a half turn")), "{said:?}");
}

/// **The new words print as they were written**: a circle's `normal:` and an arc's `sense:`.
#[test]
fn normal_and_sense_print_as_written() {
    let (p, errs) = crate::common::parse("unit mm\nk := circle(center: o, normal: ax)\n\
                                          a := arc(center: c, start: s, end: e, sense: cw)\n");
    assert!(errs.is_empty(), "{errs:?}");
    let mut said = Vec::new();
    for st in &p.root().body {
        if let gcs_core::syntax::StmtKind::Decl(_) = &st.kind {
            let mut out = String::new();
            gcs_core::syntax::write_stmt_to(&mut out, &st.kind).unwrap();
            said.push(out);
        }
    }
    assert_eq!(said, ["k := circle(center: o, normal: ax)",
                      "a := arc(center: c, start: s, end: e, sense: cw)"]);
}

/// **A circle about a point in space stands square to its normal** (#363 S6): `circle(center: o,
/// normal: ax)` is drawn in the plane through `o` square to `ax`, so a disc swept from it is
/// π·r²·h, and a point held on it stands `r` from `o`, square to `ax`.  And what is refused: a
/// `normal:` of a circle drawn in a plane, a circle in space with none (saying to give one), and
/// an arc over three points in a line.
#[test]
fn a_circle_about_a_point_in_space_stands_square_to_its_normal() {
    let e = with("
oc := point hint((5, 5, 5))
fix((5mm, 5mm, 5mm)) oc
tc := point hint((6, 7, 8))
fix((6mm, 7mm, 8mm)) tc
ax := line(oc, tc)
k := circle(center: oc, normal: ax)
radius(4mm) k
q := point hint((9, 5, 5))
q coincident k
q distance(3mm, along: std.x) oc
disc := solid(face(k), depth: 10mm)");
    near(exact(&e, "disc"), PI * 16.0 * 10.0, 1e-9, "the disc");
    let at = |n: &str| e.sketch.world_point(e.map.ent_named(n).unwrap().i());
    let r = sub(at("q"), at("oc"));
    assert!((dot(r, r).sqrt() - 4.0).abs() <= 1e-9, "{r:?}");
    assert!(dot(r, [1.0, 2.0, 3.0]).abs() <= 1e-9, "{r:?}");
    let said = refusals(&format!("{PATHS}
in std.top {{
  c := point
}}
k := circle(center: c, normal: std.z)
"));
    assert!(said.iter().any(|m| m.starts_with("E040") && m.contains("drawn in a plane")),
            "{said:?}");
    let said = refusals(&format!("{PATHS}
c := point hint((1, 2, 3))
k := circle(center: c)
"));
    assert!(said.iter().any(|m| m.starts_with("E060")
                            && m.contains("`circle(center: o, normal: ax)`")), "{said:?}");
    let said = refusals(&format!("{PATHS}
p0 := point hint((0, 0, 0))
p1 := point hint((5, 5, 5))
p2 := point hint((10, 10, 10))
k := arc(center: p1, start: p0, end: p2)
"));
    assert!(said.iter().any(|m| m.starts_with("E060") && m.contains("stand in a line")),
            "{said:?}");
}

/// **A circle's gauge is taken again where a solve ends** (#363 S6): its turn about its normal
/// is held square to an axis chosen off the normal as the circle was drawn, and a normal that
/// swings onto that axis in the solve — seeded along `z`, so held square to `x`, and solved along
/// `x`; seeded along `x` and solved along `y` — would leave the turn unheld there, a freedom the drawing does not have.  So the solved
/// circle is still determined, its gauge square to its normal, and a second solve from there
/// keeps it.
#[test]
fn a_circles_gauge_follows_a_normal_that_swings_onto_it() {
    for (seed, [dx, dy, dz]) in [("(5, 5, 15)", [10, 0, 0]), ("(15, 5, 5)", [0, 10, 0]),
                                 ("(5, 5, 15)", [0, 10, 0])] {
        let place = [(dx, "x"), (dy, "y"), (dz, "z")].map(|(d, a)| match d {
            0 => format!("oc level(std.{a}) tc"),
            _ => format!("oc distance({d}mm, along: std.{a}) tc"),
        }).join("\n");
        let mut e = with(&format!("
oc := point hint((5, 5, 5))
fix((5mm, 5mm, 5mm)) oc
tc := point hint({seed})
{}
ax := line(oc, tc)
k := circle(center: oc, normal: ax)
radius(4mm) k
disc := solid(face(k), depth: 10mm)", place));
        near(exact(&e, "disc"), PI * 16.0 * 10.0, 1e-9, "the disc");
        let n = unit(sub(e.sketch.world_point(index(&e, "tc")),
                         e.sketch.world_point(index(&e, "oc"))));
        let (&g, _) = e.sketch.round_gauges.iter().next().expect("a gauge");
        let d = e.sketch.axes[g as usize].d.map(|p| e.sketch.params[p as usize].value);
        assert!(dot(unit(d), n).abs() <= 1e-9, "the gauge {d:?} against the normal {n:?}");
        let again = gcs_core::solve::solve(&mut e.sketch, Default::default());
        assert!(again.success);
        let opts = gcs_core::diagnose::DiagnoseOptions::default();
        assert_eq!(gcs_core::diagnose::diagnose(&mut e.sketch, opts).dof, 0);
    }
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
guide := trace(Helix(o, xl, yl, zl, r: 10mm, pitch: 5mm, turns: 1).p over u in (0, 1))
param s: Scalar over (0, 1)
ribbon := sweep(Station(guide, std.z, w: 2mm, h: 1mm).f over s)");
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
F := trace(Sheet(o, xl, yl, zl).p over (a in (-6, 6), b in (-6, 6)))
st := Station(F, std.z, w: 2mm, h: 1mm, s: 0.5)
"));
    assert!(said.iter().any(|m| m.starts_with("E103") && m.contains("`g` is a path")
                                && m.contains("not a surface traced in space")), "{said:?}");
    let said = refusals(&format!("{PATHS}
guide := trace(Helix(o, xl, yl, zl, r: 10mm, pitch: 5mm, turns: 1).p over u in (0, 1))
m := point hint((10, 0, 0))
m coincident(t == 0.25, u == 0.5) guide
"));
    assert!(said.iter().any(|m| m.contains("`u` is given twice")), "{said:?}");
    // a spline over points in space is a curve in space, and bounds no face (#363 S3)
    let said = refusals(&format!("{PATHS}{CUBIC}
closing := line(kd, ka)
f := face(cubic, closing)
"));
    assert!(said.iter().any(|m| m.starts_with("E080") && m.contains("`cubic` is a curve in space")),
            "{said:?}");
}
