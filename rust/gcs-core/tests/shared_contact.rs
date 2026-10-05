//! A contact parameter two contacts share (issue #70, part 2): `t == s` pins a contact's place
//! along its curve to an unknown (`param s: Angle hint(50)`, issue #77), so that place is the
//! unknown — and every contact pinned to it owns the same one, as every dimension reading an
//! unknown shares it.  The seed is the declaration's.  A tangency and a curvature stated at one
//! place are then one contact, and regular, where two contacts tied through other geometry are a
//! degenerate root.
//!
//! Checked against the involute's closed forms (`curve_contact.rs`): its tangent at roll `u` is
//! square to the string, its centre of curvature is where the string leaves the base circle, and
//! its radius of curvature is the string unwound, `Rb · u`.

use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::parse;

use crate::common::{build, involute_at};

const INVOLUTE: &str = "\
component Involute(c: circle, phase: Angle, u: Angle) {
  p := point(x: c.center.x + c.r * (cos(u + phase) + u * pi / 180 * sin(u + phase)), y: c.center.y + c.r * (sin(u + phase) - u * pi / 180 * cos(u + phase)))
}
o := point
base := circle(center: o) hint(r: 20)
inv := Involute(base, phase: 0).p over u in (10, 90)
radius(20) base
fix(x == 0, y == 0) o
";

/// How many constraints own the shared unknown `name`.
fn owners(e: &Elaborated, name: &str) -> usize {
    let p = e.sketch.shared[name].param;
    e.sketch.constraints.iter().filter(|c| c.owns(p)).count()
}

fn errors(src: &str) -> Vec<String> {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    elaborate(&prog).errors().map(|d| d.message.clone()).collect()
}

/// **One place is one unknown.**  A point on the involute and a line tangent to it, pinned to
/// the same name, own one parameter between them: the drawing has one freedom fewer than the
/// same two contacts each owning its own, and the line solves through the point.
#[test]
fn two_contacts_pinned_to_one_name_own_one_unknown() {
    let line = "a := point hint(x: 30, y: 10)\nb := point hint(x: 40, y: 35)\nl := line(a, b)\n\
                p := point hint(x: 20, y: 20)\n";
    let mut shared = build(&format!(
        "{INVOLUTE}{line}param s: Angle hint(50)\np coincident(t == s) inv\ninv tangent(t == s) l\n"
    ));
    let mut apart =
        build(&format!("{INVOLUTE}{line}p coincident inv hint(t: 50)\ninv tangent l hint(t: 50)\n"));
    assert_eq!(owners(&shared, "s"), 2);
    assert!(apart.sketch.shared.is_empty());
    let r = solve(&mut shared.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    assert!(solve(&mut apart.sketch, SolveOpts::default()).success);
    let dof = |e: &mut Elaborated| diagnose(&mut e.sketch, DiagnoseOptions::default()).dof;
    assert_eq!(dof(&mut apart) - dof(&mut shared), 1, "sharing the place takes one freedom");
    // the point is on the tangent line: the contact point is the tangency's
    let at = |n: &str| shared.sketch.point_xy(shared.map.ent_named(n).unwrap().i());
    let (a, b, p) = (at("a"), at("b"), at("p"));
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let off = ((p.0 - a.0) * dy - (p.1 - a.1) * dx).abs() / dx.hypot(dy);
    assert!(off < 1e-6, "the point is {off} off the tangent");
    let u = shared.sketch.params[shared.sketch.shared["s"].param as usize].value;
    let c = involute_at(0.0, 0.0, 20.0, u);
    assert!((c.0 - p.0).abs() < 1e-6 && (c.1 - p.1).abs() < 1e-6, "{p:?} against {c:?} at {u}");
}

/// **The osculating circle touches the tangent where the curve does.**  A line tangent to the
/// involute and a circle osculating it, at one shared place, with the circle's radius stated:
/// the place is the roll at which the string unwound is that long (`Rb · u = 5π`, so 45°), the
/// circle's centre is where the string leaves the base, and the line is tangent to the circle —
/// the condition that, stated through two contacts, holds only to third order.
#[test]
fn a_tangency_and_a_curvature_at_one_place() {
    let src = format!(
        "{INVOLUTE}a := point hint(x: 0, y: 10)\nb := point hint(x: 30, y: 40)\nl := line(a, b)\n\
         a distance(40) b\nfix(x == 0) a\n\
         k := point hint(x: 10, y: 10)\nosc := circle(center: k) hint(r: 12)\n\
         param s: Angle hint(40)\ninv tangent(t == s) l\ninv curvature(t == s) osc\n\
         radius(5 * pi) osc\n"
    );
    let mut e = build(&src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let u = e.sketch.params[e.sketch.shared["s"].param as usize].value;
    assert!((u - 45.0).abs() < 1e-6, "touches at {u}");
    let at = |n: &str| e.sketch.point_xy(e.map.ent_named(n).unwrap().i());
    let k = at("k");
    let t = (20.0 * 45f64.to_radians().cos(), 20.0 * 45f64.to_radians().sin());
    assert!((k.0 - t.0).abs() < 1e-6 && (k.1 - t.1).abs() < 1e-6, "centre {k:?} against {t:?}");
    let (a, b) = (at("a"), at("b"));
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let gap = ((k.0 - a.0) * dy - (k.1 - a.1) * dx).abs() / dx.hypot(dy);
    let rho = 5.0 * std::f64::consts::PI;
    assert!((gap - rho).abs() < 1e-6, "the line is {gap} from the centre, the radius {rho}");
}

/// A shared place is a place along **one** curve: a contact on another curve pinned to the
/// same name is refused, naming both.  And it is seeded where it is declared: a `hint(t: …)`
/// beside a pin to it is a second seed, refused.
#[test]
fn a_place_is_shared_along_one_curve_only() {
    let src = format!(
        "{INVOLUTE}inv2 := Involute(base, phase: 90).p over u in (10, 90)\n\
         p := point hint(x: 20, y: 20)\nq := point hint(x: -20, y: 20)\n\
         param s: Angle hint(50)\np coincident(t == s) inv\nq coincident(t == s) inv2\n"
    );
    let said = errors(&src);
    assert!(said.iter().any(|m| m.contains("cannot share it")), "{said:?}");
    let src = format!(
        "{INVOLUTE}p := point hint(x: 20, y: 20)\nparam s: Angle hint(50)\n\
         p coincident(t == s) inv hint(t: 50)\n"
    );
    let said = errors(&src);
    assert!(said.iter().any(|m| m.contains("seeded where it is declared")), "{said:?}");
    // and undeclared, the name is no unknown at all
    let said = errors(&format!("{INVOLUTE}p := point hint(x: 20, y: 20)\np coincident(t == s) inv\n"));
    assert!(said.iter().any(|m| m.contains("`s` is not defined")), "{said:?}");
}

/// A `fix` holds a number, so an unknown is refused there, and a pin to a number the document
/// defines is the pin it always was: the contact is held at that number.
#[test]
fn a_pin_to_a_number_still_pins() {
    let said = errors("param s: Length\np := point\nfix(x == s) p\n");
    assert!(said.iter().any(|m| m.contains("pinned to an unknown")), "{said:?}");
    let mut e = build(&format!(
        "{INVOLUTE}w := 30\np := point hint(x: 20, y: 20)\np coincident(t == w) inv\n"
    ));
    assert!(e.sketch.shared.is_empty());
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    let p = e.sketch.point_xy(e.map.ent_named("p").unwrap().i());
    let c = involute_at(0.0, 0.0, 20.0, 30.0);
    assert!((c.0 - p.0).abs() < 1e-9 && (c.1 - p.1).abs() < 1e-9, "{p:?} against {c:?}");
}

/// One name is one unknown: a place shared along a curve is no number a dimension can be written
/// in, so `s` read both ways is refused rather than made two unknowns told apart nowhere.
#[test]
fn a_place_is_not_a_free_variable() {
    let src = format!(
        "{INVOLUTE}p := point hint(x: 20, y: 20)\nparam s: Length hint(50)\np coincident(t == s) inv\n\
         q := point hint(x: 5, y: 5)\nq distance(s) o\n"
    );
    let said = errors(&src);
    assert!(said.iter().any(|m| m.contains("is a place along a curve")), "{said:?}");
}

/// A copy and a document both keep the sharing: the rebuild walk (`io::graft`) and the JSON
/// carry the name, so the contacts that owned one unknown own one again.
#[test]
fn a_copy_and_a_document_keep_the_place_shared() {
    let src = "a := point hint(x: 0, y: 0)\nb := point hint(x: 10, y: 20)\n\
               c := point hint(x: 30, y: 20)\nd := point hint(x: 40, y: 0)\n\
               spl := spline(a, b, c, d)\n\
               p := point hint(x: 15, y: 20)\nq := point hint(x: 15, y: 25)\nl := line(q, p)\n\
               param s: Scalar hint(0.4)\np coincident(t == s) spl\nspl tangent(t == s) l\n";
    let e = build(src);
    let check = |sk: &gcs_core::model::Sketch, how: &str| {
        let p = sk.shared.get("s").unwrap_or_else(|| panic!("{how}: no shared `s`")).param;
        let n = sk.constraints.iter().filter(|c| c.owns(p)).count();
        assert_eq!(n, 2, "{how}: {n} contacts own `s`");
        assert!((sk.params[p as usize].value - 0.4).abs() < 1e-12, "{how}: the seed came along");
    };
    check(&e.sketch, "elaborated");
    check(&gcs_core::io::copy(&e.sketch, &e.sketch.primitives()), "copied");
    let doc = gcs_core::io::dumps(&e.sketch, None);
    assert!(doc.contains("\"shared\""), "{doc}");
    check(&gcs_core::io::loads(&doc).unwrap(), "reloaded");
    // and the one unknown outlives one of its owners, retired only with the last
    let mut sk = e.sketch.clone();
    let ids: Vec<u32> = sk.constraints.iter().filter(|c| !c.aux_params().is_empty())
        .map(|c| c.id).collect();
    let p = sk.shared["s"].param as usize;
    sk.remove(ids[0]);
    assert!(!sk.params[p].fixed, "still owned by the tangency");
    sk.remove(ids[1]);
    assert!(sk.params[p].fixed, "owned by nothing, so no freedom");
}
