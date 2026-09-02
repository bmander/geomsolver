//! A curve is a point of a component as one of its formals runs (Solvent §6.5): the cases at
//! the seams — a formal left unbound is one unknown of the drawing wherever it is read, a
//! nested instance's own unbound formal is its own, the instance a point belongs to is the one
//! with the formal, and a computed point stands alone.

use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::parse;

fn build(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    elaborate(&prog)
}

fn messages(e: &Elaborated) -> Vec<String> {
    e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect()
}

/// **An unbound formal is one unknown, wherever it is read.**  A `param` over it, an argument
/// handed on to a nested instance, and a dimension all read the same `c.theta` — the crank
/// turns once, not three times — and the drawing has exactly that one freedom.
#[test]
fn an_unbound_formal_is_one_unknown_everywhere_it_is_read() {
    let src = "\
component Arm(o: point, p: point, len: Length, theta: Angle) {
  line l(o, p)
  o distance(len) p
  point x hint(x: 1, y: 0)
  line ax(o, x)
  horizontal ax
  o distance(1) x
  ax angle(theta) l
}
component Crank(o: point, theta: Angle) {
  param twice = theta * 2
  point p hint(x: 20, y: 10)
  point q hint(x: 10, y: 20)
  a: Arm(o, p, len: 30, theta: theta)
  b: Arm(o, q, len: 30, theta: twice)
}
point o hint(x: 0, y: 0)
ground o
c: Crank(o)
";
    let mut e = build(src);
    assert!(e.ok(), "{:?}", messages(&e));
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let free: Vec<&String> = e.sketch.free_vars.keys().collect();
    assert_eq!(free, vec!["c.theta"], "one unknown, named under the instance");
    let d = gcs_core::diagnose::diagnose(&mut e.sketch, gcs_core::diagnose::DiagnoseOptions::default());
    assert_eq!(d.dof, 1, "the crank is the one freedom");
    // and the second arm really is at twice the angle of the first
    let theta = e.sketch.params[e.sketch.free_vars["c.theta"] as usize].value;
    let at = |n: &str| e.sketch.point_xy(e.map.ent_named(n).unwrap().i());
    let bearing = |(x, y): (f64, f64)| y.atan2(x).to_degrees();
    let (p, q) = (at("c.p"), at("c.q"));
    let wrap = |a: f64| (a % 360.0 + 540.0) % 360.0 - 180.0;
    assert!((wrap(bearing(p) - theta)).abs() < 1e-6, "p at theta: {} vs {theta}", bearing(p));
    assert!((wrap(bearing(q) - 2.0 * theta)).abs() < 1e-6, "q at 2 theta: {} vs {theta}", bearing(q));
}

/// **Inside a traced component a nested instance's unbound formal is its own**, not the outer
/// formal of the same name: it is no column of the curve, and is reported as such rather than
/// silently swept with the outer one.
#[test]
fn a_nested_unbound_formal_is_not_captured_by_the_outer_one() {
    let src = "\
component Inner(o: point, u: Angle) {
  point q hint(x: 5, y: 0)
  point x hint(x: 1, y: 0)
  line ax(o, x)
  line l(o, q)
  horizontal ax
  o distance(1) x
  o distance(5) q
  ax angle(u) l
}
component Outer(o: point, u: Angle) {
  i: Inner(o)
}
point o hint(x: 0, y: 0)
circle base(center: o) hint(r: 7)
ground o
curve w = Outer(o).i.q over u in (0, 90)
";
    let e = build(src);
    assert!(!e.ok(), "the inner `u` swept with the outer one");
    assert!(
        e.diags.iter().any(|d| d.message.contains("i.u")),
        "the inner formal is named: {:?}",
        messages(&e)
    );
}

/// **The instance a point belongs to is the innermost one whose component has the formal.**
/// `o.i.t over u`: `Inner` has no `u`, so the curve is `Outer`'s point `i.t` over `Outer`'s `u`.
#[test]
fn the_owner_is_the_instance_with_the_formal() {
    let src = "\
component Inner(o: point, a: Angle) {
  point t hint(x: 5, y: 0)
  point x hint(x: 1, y: 0)
  line ax(o, x)
  line l(o, t)
  horizontal ax
  o distance(1) x
  o distance(5) t
  ax angle(a) l
}
component Outer(o: point, u: Angle) {
  i: Inner(o, a: u)
}
point o hint(x: 0, y: 0)
ground o
d: Outer(o, u: 30)
curve k = d.i.t over u in (0, 90)
";
    let e = build(src);
    assert!(e.ok(), "{:?}", messages(&e));
    for u in [0.0f64, 30.0, 60.0, 90.0] {
        let want = (5.0 * u.to_radians().cos(), 5.0 * u.to_radians().sin());
        let got = e.sketch.curve_point(0, u);
        assert!(
            (got.0 - want.0).abs() < 1e-8 && (got.1 - want.1).abs() < 1e-8,
            "at u = {u}: {got:?} against {want:?}"
        );
    }
}

/// A computed point is the whole of what its component says: one beside placed geometry is
/// refused rather than the geometry being quietly dropped.
#[test]
fn a_computed_point_stands_alone() {
    let src = "\
component Both(o: point, u: Angle) {
  port p = ( o.x + cos(u), o.y + sin(u) )
  point q hint(x: 3, y: 4)
  o distance(5) q
}
point o hint(x: 0, y: 0)
curve w = Both(o).p over u in (0, 90)
";
    let e = build(src);
    assert!(!e.ok());
    assert!(
        e.diags.iter().any(|d| d.message.contains("may hold nothing else")),
        "{:?}",
        messages(&e)
    );
}
