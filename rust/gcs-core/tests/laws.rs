//! **Twist and scale laws** (#243 L5): a twist or a scale is a dimension of the section reading
//! the family's running number `s`, and any expression in it is a law.  The library's words take
//! a total and run it linearly (`std.Twisted`, `scaled`, `scaled_radius`); any other law is the
//! same word given another fraction, or the writer's own expression (finding 10: Solvent has no
//! function values, so no word takes a `law:`).  Gated by closed forms off the exact B-rep: a
//! twisted extrusion is A·L at any twist (Cavalieri), and a section turned about its centroid on
//! a bend is Pappus's whatever its twist; a linear scale is a frustum; a nonlinear scale is
//! L·∫A(s) ds, worked out in closed form or by a Gauss–Legendre rule exact for its degree.
use gcs_core::program::Elaborated;
use std::f64::consts::PI;

use crate::common::parsed::determined;

/// A square of half side `half` and a disc of radius `r`, each drawn square to the path `g` at
/// station `s`; the square turned by `std.Twisted` and its corners scaled by `scaled`, the disc's
/// radius by `scaled_radius`.  The guides: a line 40 long and a quarter arc of radius 30, in plan.
const LAWS: &str = "unit mm
use std (scaled, scaled_radius)

component Square(g: path, up: axis, half: Length, twist: Angle, scale: Scalar, s: Scalar) {
  st := std.On(g, up: up, s: s)
  tw := std.Twisted(st.n, twist: twist, s: s)
  in st.n {
    a := point
    b := point
    c := point
    d := point
    a scaled(du: -half, dv: -half, scale: scale, s: s) tw.axes
    b scaled(du: half, dv: -half, scale: scale, s: s) tw.axes
    c scaled(du: half, dv: half, scale: scale, s: s) tw.axes
    d scaled(du: -half, dv: half, scale: scale, s: s) tw.axes
  }
  f := face(a, b, c, d, -> close)
}

component Disc(g: path, up: axis, r: Length, scale: Scalar, s: Scalar) {
  st := std.On(g, up: up, s: s)
  in st.n {
    k := circle(center: st.n.origin)
    scaled_radius(r: r, scale: scale, s: s) k
  }
  f := face(k)
}

in std.top {
  l0 := point
  l1 := point
  fix((-60mm, 0mm)) l0
  fix((-60mm, 40mm)) l1
  straight := line(l0, l1)
  c := point
  e := point
  fix((0mm, 0mm)) c
  fix((30mm, 0mm)) e
  x := point hint((0, 30))
  bend := arc(center: c, start: e, end: x)
  vertical line(c, x)
}
";

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no solid `{name}`")).i()
}

/// The volume of the solid `name` off its exact B-rep, its boundary checked first.
fn exact(e: &Elaborated, name: &str) -> f64 {
    let x = e.sketch.exact_solid(index(e, name)).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    x.brep.check(1e-9 * x.brep.size()).unwrap_or_else(|k| panic!("`{name}`: {}", k.msg));
    gcs_core::brep::props::volume(&x.brep) / x.mm.powi(3)
}

/// `got` within `rel` of `want`, said with both.
fn near(got: f64, want: f64, rel: f64, what: &str) {
    assert!((got - want).abs() <= rel * want.abs(), "{what}: {got} against {want} (±{rel})");
}

/// `LAWS` with the solids `families` (`name := solid(…)` lines) written below it, determined.
fn with(families: &str) -> Elaborated {
    determined(&format!("{LAWS}{families}\n"))
}

/// The words the program is refused with.
fn refused(src: &str) -> Vec<String> {
    let (mut prog, errs) = crate::common::parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    gcs_core::modules::link(&mut prog, &mut gcs_core::library::resolve);
    let e = gcs_core::program::elaborate(&prog);
    e.errors().map(|d| d.message.clone()).collect()
}

/// `∫₀¹ f` by the five-point Gauss–Legendre rule, exact for a polynomial of degree nine.
fn gauss5(f: impl Fn(f64) -> f64) -> f64 {
    let (a, b) = (0.538_469_310_105_683_1, 0.906_179_845_938_664);
    let (wa, wb) = (0.478_628_670_499_366_5, 0.236_926_885_056_189_1);
    let w0 = 128.0 / 225.0;
    let at = |x: f64| f(0.5 + 0.5 * x);
    0.5 * (w0 * at(0.0) + wa * (at(-a) + at(a)) + wb * (at(-b) + at(b)))
}

/// **A twisted extrusion is A·L at any twist** (Cavalieri): the 20 mm square turned by
/// `std.Twisted` as it runs 40 mm is 20² · 40 = 16000 mm³, untwisted, a quarter turn, a turn and
/// a quarter the other way — and eased in and out, the word given the fraction `3s² − 2s³`.
#[test]
fn a_twisted_extrusion_is_al_at_any_twist() {
    let e = with("
none := solid(Square(straight, std.z, half: 10mm, twist: 0deg, scale: 1).f over s in (0, 1))
quarter := solid(Square(straight, std.z, half: 10mm, twist: 90deg, scale: 1).f over s in (0, 1))
back := solid(Square(straight, std.z, half: 10mm, twist: -450deg, scale: 1).f over s in (0, 1))");
    for name in ["none", "quarter", "back"] {
        near(exact(&e, name), 16000.0, 1e-9, name);
    }
    let src = format!("{LAWS}
eased := solid(Square(straight, std.z, half: 10mm, twist: 90deg, scale: 1).f over s in (0, 1))
")
    .replace("std.Twisted(st.n, twist: twist, s: s)",
             "std.Twisted(st.n, twist: twist, s: 3 * s^2 - 2 * s^3)");
    near(exact(&determined(&src), "eased"), 16000.0, 1e-9, "an eased quarter turn");
}

/// **And round a bend**: a 10 mm square turned about its centre, which stays on a quarter arc of
/// radius 30, sweeps its area times the arc's length whatever it turns by (Pappus):
/// 100 · 30 · π/2 = 1500π mm³, at a half turn as untwisted.
#[test]
fn a_twisted_bend_is_pappus() {
    let e = with("
bent := solid(Square(bend, std.z, half: 5mm, twist: 0deg, scale: 1).f over s in (0, 1))
turned := solid(Square(bend, std.z, half: 5mm, twist: 180deg, scale: 1).f over s in (0, 1))");
    near(exact(&e, "bent"), 1500.0 * PI, 1e-9, "untwisted round the bend");
    near(exact(&e, "turned"), 1500.0 * PI, 1e-9, "a half turn round the bend");
}

/// **A linear scale is a frustum**: the square 20 to 10 over 40 is 40/3 · (400 + 100 + 200)
/// mm³, at any twist as well (Cavalieri again: each section is the untwisted one turned), and the
/// disc of radius 10 to 5 is π · 40/3 · (100 + 50 + 25) — a cone.
#[test]
fn a_linear_scale_is_a_frustum() {
    let e = with("
frustum := solid(Square(straight, std.z, half: 10mm, twist: 0deg, scale: 0.5).f over s in (0, 1))
twisted := solid(Square(straight, std.z, half: 10mm, twist: -270deg, scale: 0.5).f over s in (0, 1))
cone := solid(Disc(straight, std.z, r: 10mm, scale: 0.5).f over s in (0, 1))");
    let square = 40.0 / 3.0 * (400.0 + 100.0 + 200.0);
    near(exact(&e, "frustum"), square, 1e-9, "a square frustum");
    near(exact(&e, "twisted"), square, 1e-9, "a twisted square frustum");
    near(exact(&e, "cone"), PI * 40.0 / 3.0 * 175.0, 1e-9, "a cone's frustum");
    let x = e.sketch.exact_solid(index(&e, "cone")).unwrap();
    assert!(x.brep.faces.iter().any(|f| f.surface.kind() == "cone"), "the disc sweeps a cone");
}

/// **A nonlinear scale is L·∫A(s) ds**: the disc swelling as `1 + 0.3·sin(180°·s)` — the word
/// given the fraction `sin(180 * s)` — is 40π · 100 · ∫(1 + 0.3 sin πs)² ds = 4000π(1 + 0.6/π·2
/// + 0.045); the square shrinking by the eased law `1 − 0.5(3s² − 2s³)` — the writer's own
/// expression — is 40 · 400 · ∫k(s)² ds, by a quadrature exact for its degree (six).
#[test]
fn a_nonlinear_scale_is_its_quadrature() {
    let src = format!("{LAWS}
swell := solid(Disc(straight, std.z, r: 10mm, scale: 1.3).f over s in (0, 1))
")
    .replace("scaled_radius(r: r, scale: scale, s: s) k", "scaled_radius(r: r, scale: scale, s: sin(180 * s)) k");
    let e = determined(&src);
    near(exact(&e, "swell"), 4000.0 * PI * (1.0 + 1.2 / PI + 0.045), 1e-9, "a sine swell");

    let law = "(1 - 0.5 * (3 * s^2 - 2 * s^3))";
    let src = format!("{LAWS}
eased := solid(Square(straight, std.z, half: 10mm, twist: 0deg, scale: 1).f over s in (0, 1))
")
    .replace("du: -half, dv: -half,", &format!("du: -half * {law}, dv: -half * {law},"))
    .replace("du: half, dv: -half,", &format!("du: half * {law}, dv: -half * {law},"))
    .replace("du: half, dv: half,", &format!("du: half * {law}, dv: half * {law},"))
    .replace("du: -half, dv: half,", &format!("du: -half * {law}, dv: half * {law},"));
    let e = determined(&src);
    let k = |s: f64| 1.0 - 0.5 * (3.0 * s * s - 2.0 * s * s * s);
    near(exact(&e, "eased"), 40.0 * 400.0 * gauss5(|s| k(s).powi(2)), 1e-9, "an eased scale");
}

/// **A law in pieces** is one expression: `max(1 − s, 0.5)` shrinks the disc to half its radius
/// by the middle and holds it there, a cone's frustum and a cylinder:
/// 4000π · (∫₀^½ (1 − s)² ds + ½ · ¼) = 4000π · 5/12.
#[test]
fn a_law_in_pieces_is_one_expression() {
    let src = format!("{LAWS}
piecewise := solid(Disc(straight, std.z, r: 10mm, scale: 1).f over s in (0, 1))
")
    .replace("scaled_radius(r: r, scale: scale, s: s) k", "radius(r * max(1 - s, 0.5)) k");
    let e = determined(&src);
    near(exact(&e, "piecewise"), 4000.0 * PI * 5.0 / 12.0, 1e-9, "a cone, then a cylinder");
}

/// **A `when` cannot say a law**: it is made once, for the whole run, so one over the running
/// number is refused saying what to write instead.
#[test]
fn a_when_over_the_running_number_is_refused() {
    let src = format!("{LAWS}
w := solid(Disc(straight, std.z, r: 10mm, scale: 1).f over s in (0, 1))
")
    .replace("    scaled_radius(r: r, scale: scale, s: s) k\n  }",
             "  }\n  when s < 0.5 {\n    radius(r) k\n  }\n  when s >= 0.5 {\n    radius(r / 2) k\n  }");
    let said = refused(&src);
    assert!(!said.is_empty(), "a `when` over `s` was not refused");
    assert!(said.iter().all(|m| m.contains("`s` is a face family's running number")
        && m.contains("`min`, `max` and `abs`") && !m.contains("#f")), "{said:?}");
}

/// **A station with two faces sweeps either**: the square's closing edges are the face's own,
/// named by nothing under the instance, and sweeping the disc beside it must still move the
/// square's corners as their laws say (they were held where the drawing stood, and no station
/// past the drawn one solved).
#[test]
fn a_station_with_two_faces_sweeps_either() {
    let src = LAWS.replace("    k := circle(center: st.n.origin)\n",
                           "    k := circle(center: st.n.origin)\n    p := point\n    q := point\n    \
                            r0 := point\n    p scaled(du: 12mm, dv: 0mm, scale: scale, s: s) st.n\n    \
                            q scaled(du: 14mm, dv: 0mm, scale: scale, s: s) st.n\n    \
                            r0 scaled(du: 13mm, dv: 1mm, scale: scale, s: s) st.n\n")
        .replace("  f := face(k)\n}", "  f := face(k)\n  tri := face(p, q, r0, -> close)\n}")
        + "cone := solid(Disc(straight, std.z, r: 10mm, scale: 0.5).f over s in (0, 1))\n\
           tri := solid(Disc(straight, std.z, r: 10mm, scale: 0.5).tri over s in (0, 1))\n";
    let e = determined(&src);
    near(exact(&e, "cone"), PI * 40.0 / 3.0 * 175.0, 1e-9, "the disc beside a triangle");
    // the triangle, half of 2 × 1 at the start, scaled to half: its area 1 · k², k = 1 − s/2
    near(exact(&e, "tri"), 40.0 * 1.0 * (1.0 - 0.5 + 0.25 / 3.0), 1e-9, "the triangle");
}
