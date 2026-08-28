//! A curve written in the language, solved against.
//!
//! This is the first curve family the core does not know: no basis, no kernel of its own, no
//! entity kind of its own — an involute is two expressions over a circle, and the solver reaches
//! it through the same compiled tape the panel would.
//!
//! What is under test is the seam, not the arithmetic.  `tests/tape.rs` already checks that a
//! tape means what the language says and differentiates itself correctly; here the questions are
//! whether the *Jacobian the kernel writes* matches a finite difference of the *system*, and
//! whether a point actually converges onto the curve and stays there when the circle moves.

use gcs_core::constraints::{Arg, CKind, Constraint};
use gcs_core::expr;
use gcs_core::model::{CurveBody, CurveDef, CurveE, EntKind, EntRef, Sketch};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::system::System;
use gcs_core::tape::Tape;

/// `C(u) = O + Rb (cos u + u sin u, sin u - u cos u)`, with `u` in degrees like every angle a
/// person writes here — so the arc unwound is `Rb u π/180`.
fn involute_def() -> CurveDef {
    let vars: Vec<String> = ["u", "c.center.x", "c.center.y", "c.r"]
        .iter()
        .map(|s| s.to_string())
        .collect();
    let x = "c.center.x + c.r * (cos(u) + u * pi / 180 * sin(u))";
    let y = "c.center.y + c.r * (sin(u) - u * pi / 180 * cos(u))";
    CurveDef {
        name: "involute".to_string(),
        formals: vec![("c".to_string(), EntKind::Circle)],
        values: Vec::new(),
        param: "u".to_string(),
        body: CurveBody::Exprs {
            x: Tape::compile(&expr::parse(x).unwrap().body, &vars).unwrap(),
            y: Tape::compile(&expr::parse(y).unwrap().body, &vars).unwrap(),
        },
        vars,
        domain: (0.0, 90.0),
    }
}

/// A circle, an involute of it, and a point on that involute.
fn involute_sketch() -> (Sketch, usize) {
    let mut sk = Sketch::new();
    let o = sk.point(3.0, -2.0, false, "o");
    let c = sk.circle(o, 5.0, "base");
    sk.curve_defs.push(involute_def());
    sk.curves.push(CurveE {
        def: 0,
        args: vec![EntRef::circle(c)],
        values: Vec::new(),
        domain: (0.0, 90.0),
        class: gcs_core::style::Classes::one("construction"),
    });
    let p = sk.point(20.0, 20.0, false, "p");
    sk.add(Constraint::new(
        CKind::PointOnCurve,
        vec![
            Arg::Ent(EntRef::point(p)),
            Arg::Ent(EntRef::new(EntKind::Curve, 0)),
            Arg::Seed { value: 40.0, pinned: false },
        ],
    ));
    (sk, p)
}

/// Where the involute is at `u`, worked out here rather than asked of the core — so the test and
/// the thing it tests do not share an implementation.
fn involute_at(cx: f64, cy: f64, rb: f64, u_deg: f64) -> (f64, f64) {
    let (u, r) = (u_deg, u_deg.to_radians());
    (
        cx + rb * (u.to_radians().cos() + r * u.to_radians().sin()),
        cy + rb * (u.to_radians().sin() - r * u.to_radians().cos()),
    )
}

/// **A point solves onto a curve nothing in the core knows.**
#[test]
fn a_point_lands_on_a_curve_written_in_the_language() {
    let (mut sk, p) = involute_sketch();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);

    // At whatever parameter it settled, the curve is where the point is — read off the *solved*
    // circle, because nothing here is fixed and the curve was free to come to the point as much
    // as the point was to come to the curve.  That is the whole difference between a curve the
    // solver can see and a polyline somebody sampled.
    let u = sk.params[sk.constraints[0].aux_params()[0] as usize].value;
    let (cx, cy) = sk.point_xy(0);
    let rb = sk.params[sk.circles[0].radius as usize].value;
    let want = involute_at(cx, cy, rb, u);
    let got = sk.point_xy(p);
    assert!(
        (got.0 - want.0).abs() < 1e-8 && (got.1 - want.1).abs() < 1e-8,
        "at u = {u} the curve is at {want:?} and the point is at {got:?}",
    );
    // and that really is a point of the involute: the string from the tangent point is
    // perpendicular to the radius there, and as long as the arc it unwound
    let t = (cx + rb * u.to_radians().cos(), cy + rb * u.to_radians().sin());
    let radius = (t.0 - cx, t.1 - cy);
    let string = (got.0 - t.0, got.1 - t.1);
    assert!(
        (radius.0 * string.0 + radius.1 * string.1).abs() < 1e-6,
        "the string is perpendicular to the radius",
    );
    let arc = rb * u.to_radians();
    assert!((string.0.hypot(string.1) - arc).abs() < 1e-6, "the string is as long as the arc");
}

/// **The Jacobian the kernel writes is the system's own derivative.**
///
/// A finite difference of the assembled residual vector, against the assembled Jacobian — so the
/// tape's gradient, the kernel's column order and `params_on`'s are all checked at once.  Get any
/// one of the three wrong and this fails; the drawing would still look right.
#[test]
fn the_kernels_jacobian_matches_a_finite_difference() {
    let (sk, _) = involute_sketch();
    let mut sys = System::new(&sk);
    let z = sys.z0(&sk);
    let dense = sys.jacobian_dense(&z);
    let m = sys.n_res;
    let n = z.len();
    assert_eq!(m, 2, "one contact, two residuals");
    for j in 0..n {
        let h = 1e-6 * z[j].abs().max(1.0);
        let (mut lo, mut hi) = (z.clone(), z.clone());
        lo[j] -= h;
        hi[j] += h;
        let (a, b) = (sys.residuals(&lo), sys.residuals(&hi));
        for i in 0..m {
            let fd = (b[i] - a[i]) / (2.0 * h);
            let got = dense.at(i, j);
            assert!(
                (got - fd).abs() <= 1e-5 * fd.abs().max(1.0),
                "d r{i} / d z{j}: kernel {got}, finite difference {fd}",
            );
        }
    }
}

/// The curve moves when the geometry it is written over does, and the point comes with it —
/// which is what `∂C/∂θ` is for.  A curve that only knew `∂C/∂u` would solve here and then let
/// the point fall off the moment the circle was dragged.
#[test]
fn moving_the_circle_carries_the_curve_and_the_point() {
    let (mut sk, p) = involute_sketch();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    // move the base circle's centre and grow it, then re-solve
    let o = sk.points[0].clone();
    sk.params[o.x as usize].value = -10.0;
    sk.params[o.y as usize].value = 7.5;
    sk.params[sk.circles[0].radius as usize].value = 9.0;
    sk.params[o.x as usize].fixed = true;
    sk.params[o.y as usize].fixed = true;
    sk.params[sk.circles[0].radius as usize].fixed = true;
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let u = sk.params[sk.constraints[0].aux_params()[0] as usize].value;
    let want = involute_at(-10.0, 7.5, 9.0, u);
    let got = sk.point_xy(p);
    assert!(
        (got.0 - want.0).abs() < 1e-8 && (got.1 - want.1).abs() < 1e-8,
        "the point is still on the curve: {got:?} against {want:?}",
    );
}

/// A contact owns one unknown and states one equation, so a point on a curve keeps one degree of
/// freedom — it slides along.  That is the arithmetic every parametric contact in this core is
/// written to give, and a curve family from the language is no exception.
#[test]
fn a_contact_is_worth_one_equation() {
    let (sk, _) = involute_sketch();
    let sys = System::new(&sk);
    // 2 (centre) + 1 (radius) + 2 (the point) + 1 (the parameter) free, against 2 residuals
    assert_eq!(sys.n_free, 6);
    assert_eq!(sys.n_res, 2);
}

/// Two different families in one document do not share a block, because they do not have the
/// same columns — which is the whole reason a curve's kernel belongs to its definition.
#[test]
fn two_curve_families_get_their_own_kernels() {
    let (mut sk, _) = involute_sketch();
    // a second family over the same circle: a plain circle, written out longhand
    let vars: Vec<String> = ["u", "c.center.x", "c.center.y", "c.r"]
        .iter()
        .map(|s| s.to_string())
        .collect();
    sk.curve_defs.push(CurveDef {
        name: "rim".to_string(),
        formals: vec![("c".to_string(), EntKind::Circle)],
        values: Vec::new(),
        param: "u".to_string(),
        body: CurveBody::Exprs {
            x: Tape::compile(&expr::parse("c.center.x + c.r * cos(u)").unwrap().body, &vars)
                .unwrap(),
            y: Tape::compile(&expr::parse("c.center.y + c.r * sin(u)").unwrap().body, &vars)
                .unwrap(),
        },
        vars,
        domain: (0.0, 360.0),
    });
    sk.curves.push(CurveE {
        def: 1,
        args: vec![EntRef::circle(0)],
        values: Vec::new(),
        domain: (0.0, 90.0),
        class: gcs_core::style::Classes::one("construction"),
    });
    let q = sk.point(0.0, 30.0, false, "q");
    sk.add(Constraint::new(
        CKind::PointOnCurve,
        vec![
            Arg::Ent(EntRef::point(q)),
            Arg::Ent(EntRef::new(EntKind::Curve, 1)),
            Arg::Seed { value: 100.0, pinned: false },
        ],
    ));
    let ids: Vec<usize> =
        sk.constraints.iter().map(|c| c.kernel_id_in(&sk)).collect();
    assert_ne!(ids[0], ids[1], "one kernel per definition");
    let mut sk2 = sk;
    let r = solve(&mut sk2, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    // the second point is on the rim, which is what that family says
    let (x, y) = sk2.point_xy(q);
    let (cx, cy) = sk2.point_xy(0);
    let rr = sk2.params[sk2.circles[0].radius as usize].value;
    assert!(((x - cx).hypot(y - cy) - rr).abs() < 1e-8);
}

/* -- the language ------------------------------------------------------------------- */

/// **A curve family written in Solvent, drawn and solved against.**
///
/// Nothing in the core knows what an involute is.  The family is four lines of the document, the
/// instance is one, and the contact is one — and the solver reaches all of it through the same
/// compiled tape the panel would.
#[test]
fn a_curve_written_in_the_language_draws() {
    let src = "\
curve involute(c: circle, phase: Angle)(u) over (0, 90) =
  ( c.center.x + c.r * (cos(u + phase) + u * pi / 180 * sin(u + phase)),
    c.center.y + c.r * (sin(u + phase) - u * pi / 180 * cos(u + phase)) )

point  o hint(x: 0, y: 0)
circle base(center: o) hint(r: 20) class construction
curve  flank = involute(base, phase: 0) over (0, 60)

point  p hint(x: 40, y: 40)
point_on_curve(p, flank)
radius(base) == 20
ground(o)
";
    let (prog, errs) = gcs_core::syntax::parse(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    let mut e = gcs_core::program::elaborate(&prog);
    assert!(
        e.ok(),
        "{:?}",
        e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>()
    );
    assert_eq!(e.sketch.curve_defs.len(), 1, "one family");
    assert_eq!(e.sketch.curves.len(), 1, "one curve drawn from it");
    assert_eq!(e.sketch.curve_domain(0), (0.0, 60.0), "the instance narrowed the interval");

    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);

    // the point is on the involute of a circle of radius 20 about the origin
    let u = e.sketch.params[e.sketch.constraints[0].aux_params()[0] as usize].value;
    let want = involute_at(0.0, 0.0, 20.0, u);
    let got = e.sketch.point_xy(1);
    assert!(
        (got.0 - want.0).abs() < 1e-8 && (got.1 - want.1).abs() < 1e-8,
        "at u = {u}: curve {want:?}, point {got:?}",
    );
    // and the drawn polyline runs along it, start to finish
    let poly = e.sketch.curve_polyline(0);
    assert!(poly.len() > 32);
    assert_eq!(poly[0], involute_at(0.0, 0.0, 20.0, 0.0));
    let last = poly[poly.len() - 1];
    let end = involute_at(0.0, 0.0, 20.0, 60.0);
    assert!((last.0 - end.0).abs() < 1e-9 && (last.1 - end.1).abs() < 1e-9);
}

/// The names a curve reads its arguments' coordinates by are the parameters those arguments
/// actually have, in the same order.  Get this wrong and the Jacobian's columns name the wrong
/// numbers — the drawing would still look right until something moved.
#[test]
fn the_names_match_the_parameters() {
    let mut sk = Sketch::new();
    let a = sk.point(0.0, 0.0, false, "a");
    let b = sk.point(3.0, 4.0, false, "b");
    let c = sk.point(1.0, 9.0, false, "c");
    let l = sk.line(a, b);
    let ci = sk.circle(a, 2.0, "ci");
    let ar = sk.arc(a, b, c, "ar");
    let el = sk.ellipse(a, b, 1.5, "el");
    let fr = sk.frame(a, b, "fr");
    for e in [
        EntRef::point(a),
        EntRef::line(l),
        EntRef::circle(ci),
        EntRef::arc(ar),
        EntRef::ellipse(el),
        EntRef::frame(fr),
    ] {
        let names = e.kind.scalar_names("e").expect("a fixed number of coordinates");
        assert_eq!(
            names.len(),
            sk.entity_params(e).len(),
            "{}: {names:?}",
            e.kind.as_str(),
        );
    }
    assert!(EntKind::Spline.scalar_names("e").is_none(), "a control polygon has no fixed count");
}

/// A family written over something with no fixed number of coordinates is refused, and says why.
#[test]
fn a_curve_over_a_spline_is_refused() {
    let (prog, _) = gcs_core::syntax::parse(
        "curve bad(s: spline)(u) = ( u, u )\npoint p hint(x: 0, y: 0)\n",
    );
    let e = gcs_core::program::elaborate(&prog);
    assert!(!e.ok());
    assert!(
        e.errors().any(|d| d.message.contains("no fixed number")),
        "{:?}",
        e.errors().map(|d| &d.message).collect::<Vec<_>>()
    );
}
