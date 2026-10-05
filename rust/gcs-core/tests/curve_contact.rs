//! A line tangent to, and a circle osculating, a curve written in the language (Solvent §6.5).
//!
//! The involute of a circle has closed forms for both: its tangent at roll `u` is the string
//! itself, at bearing `u` from the base radius, and its radius of curvature is the length of
//! string unwound, `Rb · u`.  Neither appears in the document; the constraints are stated and
//! the solver's answers are checked against them.  A traced curve gives an exact tangent and,
//! where its block's kernels have Taylor forms, an exact curvature; both are checked too.

use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use crate::common::parse;

const INVOLUTE: &str = "\
use std
component Involute(c: circle, phase: Angle, u: Angle) {
  p := point(x: c.center.x + c.r * (cos(u + phase) + u * pi / 180 * sin(u + phase)), y: c.center.y + c.r * (sin(u + phase) - u * pi / 180 * cos(u + phase)))
}
in std.front {
o := point
base := circle(center: o) hint(r: 20)
}
inv := Involute(base, phase: 0).p over u in (10, 90)
in std.front {
radius(20) base
fix(x == 0, y == 0) o
}
";

const UNWIND: &str = "\
use std
component Unwind(c: circle, datum: line, phase: Angle, u: Angle) {
  t := point hint(x: c.center.x + c.r * cos(u + phase), y: c.center.y + c.r * sin(u + phase))
  p := point hint(x: c.center.x + c.r * (cos(u + phase) + u * pi / 180 * sin(u + phase)), y: c.center.y + c.r * (sin(u + phase) - u * pi / 180 * cos(u + phase)))
  rad := line(c.center, t)
  s := line(t, p)
  t coincident c
  rad perpendicular s
  datum angle(u + phase) rad
  t distance(c.r * u * pi / 180) p
}
in std.front {
o := point
ax := point
datum := line(o, ax)
base := circle(center: o) hint(r: 20)
}
inv := Unwind(base, datum, phase: 0).p over u in (10, 90)
in std.front {
radius(20) base
fix(x == 0, y == 0) o
fix(x == 1, y == 0) ax
}
";

use crate::common::{build, fd_jacobian, involute_at};

/// The contact's parameter, read off the one constraint that owns one.
fn param_of(e: &Elaborated) -> f64 {
    let c = e.sketch.constraints.iter().find(|c| !c.aux_params().is_empty()).unwrap();
    e.sketch.params[c.aux_params()[0] as usize].value
}

/// **A line tangent to an involute is the string.**  Stated with the line free at both ends
/// but for one grounded end, the line solves onto the tangent at the roll the contact finds,
/// whose direction is `(cos u, sin u)`.
#[test]
fn a_line_solves_tangent_to_a_curve() {
    let src = format!(
        "{INVOLUTE}in std.front {{\na := point\nb := point hint(x: 45, y: 25)\nl := line(a, b)\n\
         fix(x == 30, y == -5) a\na distance(30) b\ninv tangent l hint(t: 45)\n}}\n"
    );
    let mut e = build(&src);
    fd_jacobian(&e.sketch, 1e-5);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let u = param_of(&e);
    let at = |n: &str| e.sketch.point_xy(e.map.ent_named(n).unwrap().i());
    let (a, b) = (at("a"), at("b"));
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    // parallel to the string, and through the contact point
    let (tx, ty) = (u.to_radians().cos(), u.to_radians().sin());
    assert!((dx * ty - dy * tx).abs() < 1e-6 * dx.hypot(dy), "direction at u = {u}");
    let c = involute_at(0.0, 0.0, 20.0, u);
    let off = ((c.0 - a.0) * dy - (c.1 - a.1) * dx).abs() / dx.hypot(dy);
    assert!(off < 1e-6, "the contact point is {off} off the line");
}

/// **An osculating circle's radius is the string unwound.**  `ρ = Rb · u` for an involute, and
/// the circle's centre is the point the string leaves the base circle — so a run of 12 from the
/// base centre to the circle's puts the contact at `cos u = 0.6`, inside the curve's interval.
#[test]
fn a_circle_solves_osculating_a_curve() {
    let src = format!(
        "{INVOLUTE}in std.front {{\nk := point hint(x: 5, y: 20)\nosc := circle(center: k) hint(r: 15)\n\
         inv curvature osc hint(t: 60)\no distance(12, along: x) k\n}}\n"
    );
    let mut e = build(&src);
    fd_jacobian(&e.sketch, 1e-5);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let u = param_of(&e);
    let rho = 20.0 * u.to_radians();
    let osc = e.map.ent_named("osc").unwrap();
    let radius = e.sketch.params[e.sketch.circles[osc.i()].radius as usize].value;
    assert!((radius - rho).abs() < 1e-6, "radius {radius} against Rb·u = {rho} at u = {u}");
    let k = e.sketch.point_xy(e.map.ent_named("k").unwrap().i());
    let c = involute_at(0.0, 0.0, 20.0, u);
    // the centre of curvature of an involute is the point where the string leaves the circle
    let t = (20.0 * u.to_radians().cos(), 20.0 * u.to_radians().sin());
    assert!((k.0 - t.0).abs() < 1e-6 && (k.1 - t.1).abs() < 1e-6, "centre {k:?} against {t:?}");
    assert!(((k.0 - c.0).hypot(k.1 - c.1) - rho).abs() < 1e-6);
}

/// **A tangency to a traced curve is exact in its residual** — the velocity is the implicit
/// function theorem's — and its Jacobian, a forward difference of that velocity, is close
/// enough for the solver to reach the same tangent the formula's does.
#[test]
fn a_line_solves_tangent_to_a_traced_curve() {
    let src = format!(
        "{UNWIND}in std.front {{\na := point\nb := point hint(x: 45, y: 25)\nl := line(a, b)\n\
         fix(x == 30, y == -5) a\na distance(30) b\ninv tangent l hint(t: 45)\n}}\n"
    );
    let mut e = build(&src);
    // the solve first: the frame's difference reads the pose the contact last reached
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    fd_jacobian(&e.sketch, 1e-3);
    let u = param_of(&e);
    let at = |n: &str| e.sketch.point_xy(e.map.ent_named(n).unwrap().i());
    let (a, b) = (at("a"), at("b"));
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let (tx, ty) = (u.to_radians().cos(), u.to_radians().sin());
    assert!((dx * ty - dy * tx).abs() < 1e-6 * dx.hypot(dy), "direction at u = {u}");
    let c = involute_at(0.0, 0.0, 20.0, u);
    let off = ((c.0 - a.0) * dy - (c.1 - a.1) * dx).abs() / dx.hypot(dy);
    assert!(off < 1e-6, "the contact point is {off} off the line");
}

/// **A traced curve has a curvature, exactly.**  The taut string states no formula, yet the
/// circle solved onto its osculating circle is the involute's — centre where the string leaves
/// the base circle, radius the string unwound — since `C''` and `C'''` are the block's own
/// Taylor orders (`locus::higher_orders`), not differences.
#[test]
fn a_circle_solves_osculating_a_traced_curve() {
    let src = format!(
        "{UNWIND}in std.front {{\nk := point hint(x: 5, y: 20)\nosc := circle(center: k) hint(r: 15)\n\
         inv curvature osc hint(t: 60)\no distance(12, along: x) k\n}}\n"
    );
    let mut e = build(&src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    fd_jacobian(&e.sketch, 1e-3);
    let u = param_of(&e);
    let rho = 20.0 * u.to_radians();
    let osc = e.map.ent_named("osc").unwrap();
    let radius = e.sketch.params[e.sketch.circles[osc.i()].radius as usize].value;
    assert!((radius - rho).abs() < 1e-6, "radius {radius} against Rb·u = {rho} at u = {u}");
    let k = e.sketch.point_xy(e.map.ent_named("k").unwrap().i());
    let t = (20.0 * u.to_radians().cos(), 20.0 * u.to_radians().sin());
    assert!((k.0 - t.0).abs() < 1e-6 && (k.1 - t.1).abs() < 1e-6, "centre {k:?} against {t:?}");
}

/// The trace's `C''` and `C'''` against the involute's closed form, at rolls across the
/// interval: `C = R(cos φ + φ sin φ, sin φ − φ cos φ)` with `φ = u°`, so in `u`,
/// `C'' = k²R((cos φ, sin φ) + φ(−sin φ, cos φ))` and
/// `C''' = k³R(2(−sin φ, cos φ) − φ(cos φ, sin φ))`, `k = π/180`.
#[test]
fn a_traced_curve_gives_its_higher_orders_exactly() {
    let e = build(UNWIND);
    let gcs_core::model::CurveBody::Trace(l) = &e.sketch.curve_defs[0].body else { panic!() };
    let mut s = gcs_core::locus::Scratch::new();
    let k = std::f64::consts::PI / 180.0;
    for u in [10.0, 25.0, 47.5, 70.0, 90.0] {
        let outer = e.sketch.curve_vars(0, u);
        let anchor = gcs_core::locus::Anchor { u: e.sketch.curve_home(0), pose: None };
        let v = gcs_core::locus::eval_flat_to(&l.flat, &outer, anchor, 3, &mut s);
        assert!(v.ok && v.orders == 3, "at {u}");
        let f = u * k;
        let (c, sn) = (f.cos(), f.sin());
        let d2 = [k * k * 20.0 * (c - f * sn), k * k * 20.0 * (sn + f * c)];
        let d3 = [k * k * k * 20.0 * (-2.0 * sn - f * c), k * k * k * 20.0 * (2.0 * c - f * sn)];
        for i in 0..2 {
            assert!((v.d2[i] - d2[i]).abs() < 1e-12, "C'' at {u}: {:?} against {d2:?}", v.d2);
            assert!((v.d3[i] - d3[i]).abs() < 1e-12, "C''' at {u}: {:?} against {d3:?}", v.d3);
        }
    }
}

/// A trace is lowered in one plane, so a block reading a relation in space is refused, naming
/// the word, rather than traced through rows that have no place to stand.
#[test]
fn a_trace_through_a_relation_in_space_is_refused() {
    let src = "\
use std
component Slide(f: plane, g: plane, u: Length) {
  in f {
    p := point hint(x: 1, y: 1)
    p distance(2, along: u) f
  }
  p distance(u, along: n) g
}
s := Slide(std.front, std.top).p over u in (0, 10)
";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    let said: Vec<&String> = e.errors().map(|d| &d.message).collect();
    assert!(said.iter().any(|m| m.contains("`distance` relates its operands in space")), "{said:?}");
}

/// The statements print back as they were written, and describe themselves the same way.
#[test]
fn the_contacts_are_operators() {
    let src = format!(
        "{INVOLUTE}in std.front {{\na := point hint(x: 30, y: -5)\nb := point hint(x: 45, y: 25)\nl := line(a, b)\n\
         k := point hint(x: 5, y: 20)\nosc := circle(center: k) hint(r: 15)\n\
         inv tangent l\ninv curvature osc\n}}\n"
    );
    let e = build(&src);
    let said: Vec<String> = e
        .sketch
        .user_constraints()
        .iter()
        .map(|c| gcs_core::io::describe_with(c, &|x| e.map.name_of(x).cloned()))
        .collect();
    assert!(said.iter().any(|s| s == "inv tangent l"), "{said:?}");
    assert!(said.iter().any(|s| s == "inv curvature osc"), "{said:?}");
}
