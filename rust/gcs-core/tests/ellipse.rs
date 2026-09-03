//! The ellipse as a library component (issue #47, item 4): `Ellipse` in `std`, a computed point
//! at eccentric angle `u` on a datum, traced as a curve — so `on`, `tangent` and `curvature`
//! are the curve contacts, exact to third order, and there is no entity kind, no kernel and no
//! `CKind` of its own.  Nothing below states the ellipse's equation to the solver; each answer
//! is checked against the closed form it never saw.

use gcs_core::constraints::CKind;
use gcs_core::model::{pick, EntKind, EntRef};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};

use crate::common::fd_jacobian;

/// An ellipse of semi-axes 8 and 3 about (10, 5), its major axis along the page's x.
const ELLIPSE: &str = "\
use std
point o hint(x: 10, y: 5)
point q hint(x: 18, y: 5)
plane f(origin: o, toward: q) class construction
curve e = Ellipse(f, a: 8, b: 3).p over u in (0, 360)
ground o
ground q
";

fn build(src: &str) -> Elaborated {
    let (prog, errs, linked) = gcs_core::library::parse_linked(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    assert!(linked.is_empty(), "{linked:?}");
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    e
}

/// The contact's parameter — the one curve contact's, not the datum's alignment, which owns a
/// parameter of its own (the chord's length).
fn param_of(e: &Elaborated) -> f64 {
    let c = e
        .sketch
        .constraints
        .iter()
        .find(|c| matches!(c.kind, CKind::PointOnCurve | CKind::CurveTangentLine | CKind::CurveCurvature))
        .unwrap();
    e.sketch.params[c.aux_params()[0] as usize].value
}

fn at(e: &Elaborated, name: &str) -> (f64, f64) {
    e.sketch.point_xy(e.map.ent_named(name).unwrap().i())
}

/// (x/a)² + (y/b)² − 1 in the datum's own frame, at bearing `th` from the page.
fn on_rim(x: f64, y: f64, cx: f64, cy: f64, th: f64, a: f64, b: f64) -> f64 {
    let (wx, wy) = (x - cx, y - cy);
    let xx = wx * th.cos() + wy * th.sin();
    let yy = -wx * th.sin() + wy * th.cos();
    (xx / a) * (xx / a) + (yy / b) * (yy / b) - 1.0
}

/// The point at eccentric angle `u` (degrees), and the rim's radius of curvature there.
fn rim_at(u: f64, a: f64, b: f64) -> ((f64, f64), f64) {
    let (s, c) = u.to_radians().sin_cos();
    let rho = (a * a * s * s + b * b * c * c).powf(1.5) / (a * b);
    ((10.0 + a * c, 5.0 + b * s), rho)
}

#[test]
fn a_point_solves_onto_the_rim_at_its_eccentric_angle() {
    let mut e = build(&format!("{ELLIPSE}point p hint(x: 11, y: 9)\np on e hint(t: 80)\n"));
    fd_jacobian(&e.sketch, 1e-5);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let (x, y) = at(&e, "p");
    assert!(on_rim(x, y, 10.0, 5.0, 0.0, 8.0, 3.0).abs() < 1e-6, "left the rim: {x}, {y}");
    // the parameter is the eccentric angle, and the contact is the point at it
    let t = param_of(&e);
    let (want, _) = rim_at(t, 8.0, 3.0);
    assert!((x - want.0).abs() < 1e-6 && (y - want.1).abs() < 1e-6, "{t}: {x}, {y}");
}

#[test]
fn a_line_solves_tangent_to_the_rim() {
    // a level line above the ellipse, one end grounded, the other 12 away and free to fall
    let mut e = build(&format!(
        "{ELLIPSE}point a hint(x: 4, y: 10)\npoint b hint(x: 16, y: 10)\nline l(a, b)\n\
         ground a\na distance(12) b\ne tangent l hint(t: 90)\n"
    ));
    fd_jacobian(&e.sketch, 1e-5);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let (a, b) = (at(&e, "a"), at(&e, "b"));
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len = dx.hypot(dy);
    // through the contact and along the rim's tangent there, in the closed form
    let t = param_of(&e);
    let ((px, py), _) = rim_at(t, 8.0, 3.0);
    let (s, co) = t.to_radians().sin_cos();
    let (tx, ty) = (-8.0 * s, 3.0 * co);
    assert!((dx * ty - dy * tx).abs() < 1e-6 * len * tx.hypot(ty), "direction at u = {t}");
    let off = ((px - a.0) * dy - (py - a.1) * dx).abs() / len;
    assert!(off < 1e-6, "the contact point is {off} off the line");
    // and touching from one side: every rim point's signed distance shares a sign
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for k in 0..720 {
        let ((x, y), _) = rim_at(0.5 * k as f64, 8.0, 3.0);
        let d = (dx * (y - a.1) - dy * (x - a.0)) / len;
        lo = lo.min(d);
        hi = hi.max(d);
    }
    assert!(lo * hi >= -1e-9, "the line crosses the rim: {lo} .. {hi}");
}

#[test]
fn a_circle_solves_onto_the_osculating_circle() {
    // a circle near the major end, centre and radius free: a computed point's frame is exact
    // to third order, so the curvature is not refused as a traced curve's is
    let mut e = build(&format!(
        "{ELLIPSE}point k hint(x: 16, y: 5.5)\ncircle c(center: k) hint(r: 2)\n\
         e curvature c hint(t: 10)\n"
    ));
    fd_jacobian(&e.sketch, 1e-5);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let t = param_of(&e);
    let ((px, py), rho) = rim_at(t, 8.0, 3.0);
    let c = e.map.ent_named("c").unwrap();
    let radius = e.sketch.params[e.sketch.circles[c.i()].radius as usize].value.abs();
    assert!((radius - rho).abs() < 1e-6, "radius {radius} against ρ = {rho} at u = {t}");
    // the centre of curvature: inward along the normal by ρ
    let (s, co) = t.to_radians().sin_cos();
    let (d1x, d1y) = (-8.0 * s, 3.0 * co);
    let n = d1x.hypot(d1y);
    let want = (px - rho * d1y / n, py + rho * d1x / n);
    let k = at(&e, "k");
    assert!((k.0 - want.0).abs() < 1e-6 && (k.1 - want.1).abs() < 1e-6, "{k:?} against {want:?}");
}

/// The rim stands on its datum: turn the datum and the ellipse turns with it, its contact
/// solved back onto the turned rim.
#[test]
fn the_rim_turns_with_its_datum() {
    let mut e = build(&format!("{ELLIPSE}point p hint(x: 11, y: 9)\np on e hint(t: 80)\n"));
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    // the datum swings to 90°: q goes from (18, 5) to (10, 13)
    let q = e.map.ent_named("q").unwrap();
    let ps = e.sketch.point_params(q.i());
    e.sketch.params[ps[0] as usize].value = 10.0;
    e.sketch.params[ps[1] as usize].value = 13.0;
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let (x, y) = at(&e, "p");
    let th = std::f64::consts::FRAC_PI_2;
    assert!(on_rim(x, y, 10.0, 5.0, th, 8.0, 3.0).abs() < 1e-6, "off the turned rim: {x}, {y}");
    assert!(on_rim(x, y, 10.0, 5.0, 0.0, 8.0, 3.0).abs() > 1e-3, "the rim did not turn");
}

#[test]
fn the_rim_is_picked_and_bounded() {
    let e = build(ELLIPSE);
    let cv = e.map.ent_named("e").unwrap();
    assert_eq!(cv.kind, EntKind::Curve);
    // the top of the rim is a minor radius above the centre
    assert_eq!(pick(&e.sketch, 10.0, 8.02, 0.2), Some(cv));
    assert!(pick(&e.sketch, 12.0, 5.8, 0.2).is_none(), "the inside of an ellipse is empty space");
    let (x0, y0, x1, y1) = e.sketch.bounds(EntRef::new(EntKind::Curve, cv.i()));
    assert!((x0 - 2.0).abs() < 1e-3 && (x1 - 18.0).abs() < 1e-3, "{x0} .. {x1}");
    assert!((y0 - 2.0).abs() < 1e-3 && (y1 - 8.0).abs() < 1e-3, "{y0} .. {y1}");
}

/// The axes are values the curve takes, stated or read from a `param` — a curve written in
/// place is given every value, and a component of one computed point cannot be drawn as an
/// instance whose formal is left free (nothing on the sheet holds a point to a formula).  So
/// an axis left out is refused where the curve is written, naming the formal.
#[test]
fn an_axis_left_out_is_refused_by_name() {
    let (prog, errs, _) = gcs_core::library::parse_linked(
        "use std\npoint o hint(x: 10, y: 5)\npoint q hint(x: 18, y: 5)\n\
         plane f(origin: o, toward: q)\ncurve e = Ellipse(f, a: 8).p over u in (0, 360)\n",
    );
    assert!(errs.is_empty());
    let e = elaborate(&prog);
    let m: Vec<String> = e.errors().map(|d| d.message.clone()).collect();
    assert!(m.iter().any(|m| m.contains("`Ellipse` was not given `b`")), "{m:?}");
}
