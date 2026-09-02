//! A line tangent to, and a circle osculating, a curve written in the language (Solvent §6.5).
//!
//! The involute of a circle has closed forms for both: its tangent at roll `u` is the string
//! itself, at bearing `u` from the base radius, and its radius of curvature is the length of
//! string unwound, `Rb · u`.  Neither appears in the document; the constraints are stated and
//! the solver's answers are checked against them.  A traced curve gives an exact tangent and no
//! curvature, and both halves of that are checked too.

use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::parse;

const INVOLUTE: &str = "\
component Involute(c: circle, phase: Angle, u: Angle) {
  port p = ( c.center.x + c.r * (cos(u + phase) + u * pi / 180 * sin(u + phase)),
             c.center.y + c.r * (sin(u + phase) - u * pi / 180 * cos(u + phase)) )
}
point  o hint(x: 0, y: 0)
circle base(center: o) hint(r: 20) class construction
curve  inv = Involute(base, phase: 0).p over u in (10, 90)
radius(20) base
ground o
";

const UNWIND: &str = "\
component Unwind(c: circle, datum: line, phase: Angle, u: Angle) {
  point t hint(x: c.center.x + c.r * cos(u + phase), y: c.center.y + c.r * sin(u + phase))
  point p hint(x: c.center.x + c.r * (cos(u + phase) + u * pi / 180 * sin(u + phase)), \
               y: c.center.y + c.r * (sin(u + phase) - u * pi / 180 * cos(u + phase)))
  line rad(c.center, t)
  line s(t, p)
  t on c
  rad perpendicular s
  datum angle(u + phase) rad
  t distance(c.r * u * pi / 180) p
}
point  o hint(x: 0, y: 0)
point  ax hint(x: 1, y: 0)
line   datum(o, ax) class construction
circle base(center: o) hint(r: 20) class construction
curve  inv = Unwind(base, datum, phase: 0).p over u in (10, 90)
radius(20) base
ground o
ground ax
";

mod common;
use common::{build, fd_jacobian, involute_at};

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
        "{INVOLUTE}point a hint(x: 30, y: -5)\npoint b hint(x: 45, y: 25)\nline l(a, b)\n\
         ground a\na distance(30) b\ninv tangent l hint(u: 45)\n"
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
        "{INVOLUTE}point k hint(x: 5, y: 20)\ncircle osc(center: k) hint(r: 15)\n\
         inv curvature osc hint(u: 60)\no distance(12, along: x) k\n"
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
/// function theorem's — and its Jacobian, a central difference of that velocity, is close
/// enough for the solver to reach the same tangent the formula's does.
#[test]
fn a_line_solves_tangent_to_a_traced_curve() {
    let src = format!(
        "{UNWIND}point a hint(x: 30, y: -5)\npoint b hint(x: 45, y: 25)\nline l(a, b)\n\
         ground a\na distance(30) b\ninv tangent l hint(u: 45)\n"
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

/// A curvature against a traced curve is refused, and says why.
#[test]
fn a_curvature_against_a_traced_curve_is_refused() {
    let src = format!("{UNWIND}point k hint(x: 5, y: 20)\ncircle osc(center: k) hint(r: 15)\ninv curvature osc\n");
    let (prog, errs) = parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    assert!(!e.ok());
    assert!(
        e.errors().any(|d| d.message.contains("no curvature")),
        "{:?}",
        e.errors().map(|d| &d.message).collect::<Vec<_>>()
    );
}

/// The statements print back as they were written, and describe themselves the same way.
#[test]
fn the_contacts_are_operators() {
    let src = format!(
        "{INVOLUTE}point a hint(x: 30, y: -5)\npoint b hint(x: 45, y: 25)\nline l(a, b)\n\
         point k hint(x: 5, y: 20)\ncircle osc(center: k) hint(r: 15)\n\
         inv tangent l\ninv curvature osc\n"
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
