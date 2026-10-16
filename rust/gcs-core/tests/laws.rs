//! **Twist and scale laws** (#243 L5): a twist or a scale is a dimension of the section reading
//! the family's running number `s`, and any expression in it is a law.  The library's words take
//! a total and run it linearly (`std.Twisted`'s turned point, `scaled`, `scaled_radius`); any
//! other law is the
//! same word given another fraction, or the writer's own expression (finding 10: Solvent has no
//! function values, so no word takes a `law:`).  Gated by closed forms off the exact B-rep: a
//! twisted extrusion is A·L at any twist (Cavalieri), and a section turned about its centroid on
//! a bend is Pappus's whatever its twist; a linear scale is a frustum; a nonlinear scale is
//! L·∫A(s) ds, worked out in closed form or by a Gauss–Legendre rule exact for its degree.
use gcs_core::program::Elaborated;
use std::f64::consts::PI;

use crate::common::parsed::determined;

/// A square of half side `half` and a disc of radius `r`, each drawn square to the path `g` at
/// station `s`: the square `sq` drawn once and carried whole by the similarity taking its centre
/// `po` and `px`, 10 along, to the station's origin and `std.Twisted`'s point, turned `twist · s`
/// and `half · (1 + (scale − 1)·s)` out (#357); the disc's radius scaled by `scaled_radius`.  The
/// guides: a line 40 long and a quarter arc of radius 30, in plan.
const LAWS: &str = "unit mm
use std (scaled, scaled_radius)

component Square(g: path, up: axis, sq: face, po: point, px: point, half: Length, twist: Angle,
                 scale: Scalar, s: Scalar) {
  st := std.On(g, up: up, s: s)
  tw := std.Twisted(st.n, twist: twist, reach: half * (1 + (scale - 1) * s), s: s)
  f := sq mapped(po -> st.n.origin, px -> tw.t) in st.n
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
in std.front {
  po := point
  px := point
  fix((0mm, -50mm)) po
  fix((10mm, -50mm)) px
  q0 := point
  q1 := point
  q2 := point
  q3 := point
  fix((-10mm, -60mm)) q0
  fix((10mm, -60mm)) q1
  fix((10mm, -40mm)) q2
  fix((-10mm, -40mm)) q3
}
sq := face(q0, q1, q2, q3, -> close)
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

/// `LAWS` with `edits` made (each must find its text) and the solids `families` written below
/// it, determined.
fn with(families: &str, edits: &[(&str, &str)]) -> Elaborated {
    determined(&edited(families, edits))
}

/// `LAWS` with `edits` made and `families` written below it.
fn edited(families: &str, edits: &[(&str, &str)]) -> String {
    let mut src = format!("{LAWS}{families}\n");
    for (from, to) in edits {
        assert!(src.contains(from), "no `{from}` to edit");
        src = src.replace(from, to);
    }
    src
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
/// `std.Twisted`'s point as it runs 40 mm is 20² · 40 = 16000 mm³, untwisted, a quarter turn, a turn and
/// a quarter the other way — and eased in and out, the word given the fraction `3s² − 2s³`.
#[test]
fn a_twisted_extrusion_is_al_at_any_twist() {
    let e = with("
none := sweep(Square(straight, std.z, sq, po, px,
    half: 10mm, twist: 0deg, scale: 1).f over s in (0, 1))
quarter := sweep(Square(straight, std.z, sq, po, px,
    half: 10mm, twist: 90deg, scale: 1).f over s in (0, 1))
back := sweep(Square(straight, std.z, sq, po, px,
    half: 10mm, twist: -450deg, scale: 1).f over s in (0, 1))",
        &[]);
    for name in ["none", "quarter", "back"] {
        near(exact(&e, name), 16000.0, 1e-9, name);
    }
    let e = with("
eased := sweep(Square(straight, std.z, sq, po, px,
    half: 10mm, twist: 90deg, scale: 1).f over s in (0, 1))",
        &[("* (1 + (scale - 1) * s), s: s)", "* (1 + (scale - 1) * s), s: 3 * s^2 - 2 * s^3)")]);
    near(exact(&e, "eased"), 16000.0, 1e-9, "an eased quarter turn");
}

/// **And round a bend**: a 10 mm square turned about its centre, which stays on a quarter arc of
/// radius 30, sweeps its area times the arc's length whatever it turns by (Pappus):
/// 100 · 30 · π/2 = 1500π mm³, at a half turn as untwisted.
#[test]
fn a_twisted_bend_is_pappus() {
    let e = with("
bent := sweep(Square(bend, std.z, sq, po, px,
    half: 5mm, twist: 0deg, scale: 1).f over s in (0, 1))
turned := sweep(Square(bend, std.z, sq, po, px,
    half: 5mm, twist: 180deg, scale: 1).f over s in (0, 1))",
        &[]);
    near(exact(&e, "bent"), 1500.0 * PI, 1e-9, "untwisted round the bend");
    near(exact(&e, "turned"), 1500.0 * PI, 1e-9, "a half turn round the bend");
}

/// **A linear scale is a frustum**: the square 20 to 10 over 40 is 40/3 · (400 + 100 + 200)
/// mm³, at any twist as well (Cavalieri again: each section is the untwisted one turned), and the
/// disc of radius 10 to 5 is π · 40/3 · (100 + 50 + 25), its wall a cone.
#[test]
fn a_linear_scale_is_a_frustum() {
    let e = with("
frustum := sweep(Square(straight, std.z, sq, po, px,
    half: 10mm, twist: 0deg, scale: 0.5).f over s in (0, 1))
twisted := sweep(Square(straight, std.z, sq, po, px,
    half: 10mm, twist: -270deg, scale: 0.5).f over s in (0, 1))
cone := sweep(Disc(straight, std.z, r: 10mm, scale: 0.5).f over s in (0, 1))",
        &[]);
    let square = 40.0 / 3.0 * (400.0 + 100.0 + 200.0);
    near(exact(&e, "frustum"), square, 1e-9, "a square frustum");
    near(exact(&e, "twisted"), square, 1e-9, "a twisted square frustum");
    near(exact(&e, "cone"), PI * 40.0 / 3.0 * 175.0, 1e-9, "a cone's frustum");
    let x = e.sketch.exact_solid(index(&e, "cone")).unwrap();
    assert!(x.brep.faces.iter().any(|f| f.surface.kind() == "cone"), "the disc sweeps a cone");
}

/// **A nonlinear scale is L·∫A(s) ds**.  The disc swelling as `1 + 0.3 sin(180°·s)` — the word
/// given the fraction `sin(180 * s)`, a number no affine form holds — is
/// 40 · 100π · ∫(1 + 0.3 sin πs)² ds = 4000π(1 + 1.2/π + 0.045).  The square shrinking to half
/// eased in and out, its reach given `3s² − 2s³`, is 40 · 400 · ∫k² ds with k = 1 − 0.5(3s² −
/// 2s³), by a quadrature exact for its degree (six).
#[test]
fn a_nonlinear_scale_is_its_quadrature() {
    let e = with("
swell := sweep(Disc(straight, std.z, r: 10mm, scale: 1.3).f over s in (0, 1))",
        &[("scale: scale, s: s) k", "scale: scale, s: sin(180 * s)) k")]);
    near(exact(&e, "swell"), 4000.0 * PI * (1.0 + 1.2 / PI + 0.045), 1e-9, "a sine swell");

    let e = with("
eased := sweep(Square(straight, std.z, sq, po, px,
    half: 10mm, twist: 0deg, scale: 0.5).f over s in (0, 1))",
        &[("reach: half * (1 + (scale - 1) * s)",
           "reach: half * (1 + (scale - 1) * (3 * s^2 - 2 * s^3))")]);
    let k = |s: f64| 1.0 - 0.5 * (3.0 * s * s - 2.0 * s * s * s);
    near(exact(&e, "eased"), 40.0 * 400.0 * gauss5(|s| k(s).powi(2)), 1e-9, "an eased scale");
}

/// **A law in pieces** is one expression: `max(1 − s, 0.7)` shrinks the disc to 0.7 of its
/// radius by `s = 0.3` and holds it there, a cone's frustum and a cylinder:
/// 4000π · (∫₀^0.3 (1 − s)² ds + 0.7 · 0.7²) = 4000π · ((1 − 0.7³)/3 + 0.7³).  The kink is off
/// the middle station, so inside a piece of the exact walls.
#[test]
fn a_law_in_pieces_is_one_expression() {
    let e = with("
piecewise := sweep(Disc(straight, std.z, r: 10mm, scale: 1).f over s in (0, 1))",
        &[("scaled_radius(r: r, scale: scale, s: s) k", "radius(r * max(1 - s, 0.7)) k")]);
    let k3 = 0.7f64.powi(3);
    near(exact(&e, "piecewise"), 4000.0 * PI * ((1.0 - k3) / 3.0 + k3), 1e-9,
         "a cone, then a cylinder");
}

/// **A `when` cannot say a law**: it is made once, for the whole run, so one over the running
/// number is refused saying what to write instead, in the names the document wrote.
#[test]
fn a_when_over_the_running_number_is_refused() {
    let src = edited("
w := sweep(Disc(straight, std.z, r: 10mm, scale: 1).f over s in (0, 1))",
        &[("    scaled_radius(r: r, scale: scale, s: s) k\n  }",
           "  }\n  when s < 0.5 {\n    radius(r) k\n  }\n  \
            when s >= 0.5 {\n    radius(r / 2) k\n  }")]);
    let said = refused(&src);
    assert!(!said.is_empty(), "a `when` over `s` was not refused");
    assert!(said.iter().all(|m| m.contains("`s` is a face family's running number")
        && m.contains("`min`, `max` and `abs`") && !m.contains("#f")), "{said:?}");
}

/// A disc and, beside it, a triangle closed by the face itself: two faces of one station.
const PAIR: &str = "
component Pair(g: path, up: axis, scale: Scalar, s: Scalar) {
  st := std.On(g, up: up, s: s)
  in st.n {
    k := circle(center: st.n.origin)
    scaled_radius(r: 10mm, scale: scale, s: s) k
    p := point
    q := point
    t := point
    p scaled(du: 12mm, dv: 0mm, scale: scale, s: s) st.n
    q scaled(du: 14mm, dv: 0mm, scale: scale, s: s) st.n
    t scaled(du: 13mm, dv: 1mm, scale: scale, s: s) st.n
  }
  disc := face(k)
  tri := face(p, q, t, -> close)
}
";

/// **A station with two faces sweeps either**: the triangle's closing edges are the face's own,
/// named by nothing under the instance, and they once held the corners they join where the
/// drawing stood while the disc beside them was swept, so no station past the drawn one solved.
/// The disc is its cone's frustum, and the triangle, of area 1 scaled to half, 40 · ∫(1 − s/2)².
#[test]
fn a_station_with_two_faces_sweeps_either() {
    let e = with(&format!("{PAIR}
cone := sweep(Pair(straight, std.z, scale: 0.5).disc over s in (0, 1))
prism := sweep(Pair(straight, std.z, scale: 0.5).tri over s in (0, 1))"), &[]);
    near(exact(&e, "cone"), PI * 40.0 / 3.0 * 175.0, 1e-9, "the disc beside a triangle");
    near(exact(&e, "prism"), 40.0 * (1.0 - 0.5 + 1.0 / 12.0), 1e-9, "the triangle");
}
