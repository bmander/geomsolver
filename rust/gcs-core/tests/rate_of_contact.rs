//! **The rate of a contact** (#365): `rate(p over t)` where `t` is the slot of the contact holding
//! `p` — a curve's parameter, a meshing's phase, a carried point's roll — is the derivative of the
//! point the contact holds in that slot, every other column held.  One kind (`CKind::Rate`, and
//! `CKind::RateDistance` for a rate measured along a direction), its kernel the source's.  Held
//! here to closed forms: a rolling circle's point, carried by the roll, runs along the epicycloid
//! at its derivative; a spline's tangent written as a rate is the tangent; a meshing's path of
//! contact is the rate along its phase.  And refused where the slot is not one, is ambiguous, or
//! is a source's kernel does not read.
use crate::common::{build, read};
use gcs_core::constraints::CKind;
use gcs_core::program::Elaborated;
use gcs_core::solve;
use gcs_core::space::{norm, sub};

fn solved(src: &str) -> Elaborated {
    let mut e = build(src);
    let r = solve::solve(&mut e.sketch, Default::default());
    assert!(r.success, "{r:?}");
    e
}

fn errors(src: &str) -> Vec<String> {
    let (prog, errs) = crate::common::parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    e.errors().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect()
}

fn refused(src: &str, want: &str) {
    let errs = errors(src);
    assert!(errs.iter().any(|e| e.contains(want)), "{want}: {errs:?}");
}

/// A circle of radius 20 rolling round the outside of one of radius 40, in the top view, as
/// `tests/carried.rs` draws it: `X` is the rolling circle's point `x` seen from the fixed circle at
/// the roll `τ`.
fn rolling(tau: f64) -> String {
    format!("\
unit mm
use std
use std.motion (spins)
in std.top {{
  o := point
  fix((0, 0)) o
  c := point
  fix((60, 0)) c
  x := point
  fix((40, 0)) x
  X := point hint((50, 0))
  Q := point hint((55, 5))
  R := point hint((45, 5))
}}
fixed_turn := frame
fixed_turn spins(1) o
rolling_turn := frame
rolling_turn spins(-2) c
X coincident(under: rolling_turn, seen_from: fixed_turn, t == {tau}) x
param vx: Length hint(1)
param vy: Length hint(1)
rate(X over t) distance(vx, along: std.x)
rate(X over t) distance(vy, along: std.y)
along := line(X, Q)
X distance(10mm) Q
rate(X over t) parallel along
square := line(X, R)
X distance(10mm) R
rate(X over t) perpendicular square
")
}

/// **A carried point's rate is the motion's twist there.**  `X`, the rolling circle's point seen
/// by the fixed one at the roll `τ`, is the epicycloid at `θ = −τ`, so its rate a radian of the
/// roll is `−E′(−τ)`: its components along the world's axes, a line along it and a line square to
/// it all solve to that.
#[test]
fn a_carried_points_rate_is_the_epicycloids_derivative() {
    let (a, b) = (40.0f64, 20.0f64);
    let k = (a + b) / b;
    for tau in [-120.0f64, -35.0, 25.0, 80.0] {
        let e = solved(&rolling(tau));
        let th = -tau.to_radians();
        // E′(θ), and the rate is its opposite: the roll turns the other way
        let de = [-(a + b) * th.sin() + b * k * (k * th).sin(), (a + b) * th.cos() - b * k * (k * th).cos()];
        let want = [-de[0], -de[1]];
        let value = |n: &str| {
            let v = e.sketch.free_vars.get(n).copied().unwrap_or_else(|| panic!("no `{n}`"));
            e.sketch.params[v as usize].value
        };
        let got = [value("vx"), value("vy")];
        assert!(norm(sub([got[0], got[1], 0.0], [want[0], want[1], 0.0])) < 1e-8,
                "τ = {tau}: {got:?} against {want:?}");
        let at = |n: &str| e.sketch.world_point(e.map.ent_named(n).unwrap().i());
        let (x, q, r) = (at("X"), at("Q"), at("R"));
        let unit = |v: [f64; 2]| { let n = v[0].hypot(v[1]); [v[0] / n, v[1] / n] };
        let (w, d, s) = (unit(want), unit([q[0] - x[0], q[1] - x[1]]), unit([r[0] - x[0], r[1] - x[1]]));
        assert!((w[0] * d[1] - w[1] * d[0]).abs() < 1e-9, "τ = {tau}: along {d:?} against {w:?}");
        assert!((w[0] * s[0] + w[1] * s[1]).abs() < 1e-9, "τ = {tau}: square {s:?} against {w:?}");
    }
}

/// The spline the tangent tests draw on, and a point held on it.
const SPLINE: &str = "\
unit mm
use std
use std.motion (spins)
in std.top {
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((0mm, 0mm)) k0
  fix((0mm, 20mm)) k1
  fix((20mm, 30mm)) k2
  fix((40mm, 30mm)) k3
  g := spline(k0, k1, k2, k3)
  m := point
  m coincident(t == 0.5) g
  q := point hint((20, 30))
  tl := line(m, q)
  length(10mm) tl
}
";

/// **A spline's tangent at a point is its rate there**: `g tangent(at: m) tl` and `rate(m over t)
/// parallel tl` are one kind over the contact's own place, solve alike (the Bézier's tangent at
/// `t = ½` is `(60, 40)`), and each prints back as it was written.
#[test]
fn a_tangent_at_a_point_is_the_rate_of_its_contact() {
    for stated in ["g tangent(at: m) tl", "rate(m over t) parallel tl", "tl parallel rate(m over t)"] {
        let src = SPLINE.replace("  length(10mm) tl\n", &format!("  length(10mm) tl\n  {stated}\n"));
        let e = solved(&src);
        let sk = &e.sketch;
        let rate = sk.constraints.iter().find(|c| c.kind == CKind::Rate).expect("a rate");
        let contact = sk.constraints.iter().find(|c| c.kind == CKind::PointOnSpline).unwrap();
        assert_eq!(rate.args[3].param(), contact.args[2].param(), "{stated}: the contact's place");
        let at = |n: &str| e.sketch.world_point(e.map.ent_named(n).unwrap().i());
        let d = sub(at("q"), at("m"));
        assert!((d[0] * 40.0 - d[1] * 60.0).abs() < 1e-8, "{stated}: {d:?}");
        let named = gcs_core::io::describe_with(rate, &|x| e.map.name_of(x).cloned());
        let want = if stated.starts_with('g') { "g tangent(at: m) tl" } else { "rate(m over t) parallel tl" };
        assert_eq!(named, want);
    }
}

/// **A meshing's path of contact is the rate along its phase**, its lag and its places following
/// by the implicit function theorem: `bearing tangent(at: Q) path` and `rate(Q over t) parallel
/// path` are one statement, read alike.
#[test]
fn a_meshings_path_of_contact_is_its_rate_along_the_phase() {
    let text = include_str!("../../examples/spiral_bevel/contact.sv");
    let with = |line: &str| format!("{text}ahead := point hint((5, 48, 3))\npath := line(Q, ahead)\n\
                                     Q distance(10mm) ahead\n{line}\n");
    let tangent = read(&with("bearing tangent(at: Q) path"));
    let rate = read(&with("rate(Q over t) parallel path"));
    let one = |e: &Elaborated| {
        let c = e.sketch.constraints.iter().find(|c| c.kind == CKind::Rate).expect("a rate").clone();
        (c.params(&e.sketch), c.consts(&e.sketch), c.n_residuals(),
         gcs_core::io::describe_with(&c, &|x| e.map.name_of(x).cloned()))
    };
    let (a, b) = (one(&tangent), one(&rate));
    assert_eq!((&a.0, &a.1, a.2), (&b.0, &b.1, b.2));
    assert_eq!(a.2, 2);
    assert_eq!(a.3, "bearing tangent(at: Q) path");
    assert_eq!(b.3, "rate(Q over t) parallel path");
    // and measured along a line: the path's speed a radian of the phase, read by the same kernel
    let speed = solved(&format!("{text}ahead := point hint((5, 48, 3))\npath := line(Q, ahead)\n\
        Q distance(10mm) ahead\nrate(Q over t) parallel path\nparam v: Length hint(1)\n\
        rate(Q over t) distance(v, along: path)\n"));
    let v = speed.sketch.params[speed.sketch.free_vars["v"] as usize].value;
    assert!(v.is_finite() && v.abs() > 1e-3, "{v}");
}

/// What a rate of a contact refuses: a slot no contact holding the point owns, a point held by two
/// contacts with that slot, a declared unknown (a rate is the partial derivative in the contact's
/// own slot, never along an unknown several contacts may read), and what a source's kernel does
/// not read — a curve's rate in its view square to a line, a distance along a direction of
/// anything but a point a motion carries.
#[test]
fn a_rate_of_a_contact_is_refused_where_its_slot_is_not_one() {
    let with = |line: &str| SPLINE.replace("  length(10mm) tl\n", &format!("  length(10mm) tl\n  {line}\n"));
    refused(&with("rate(m over s) parallel tl"), "`s` is no formal here, and no contact holding");
    refused(&with("rate(q over t) parallel tl"), "no contact holding");
    refused(&with("g perpendicular(at: m) tl"), "`perpendicular` does not relate a spline to a line");
    refused(&with("rate(m over t) angle(30deg) tl"), "not `angle`");
    // two splines through one point
    let two = SPLINE.replace("  m coincident(t == 0.5) g\n", "  m coincident(t == 0.5) g\n  \
        h := spline(k3, k2, k1, k0)\n  m coincident h\n");
    refused(&two.replace("  length(10mm) tl\n", "  length(10mm) tl\n  rate(m over t) parallel tl\n"),
            "held by 2 contacts");
    // a declared unknown is no slot
    let shared = SPLINE.replace("  m coincident(t == 0.5) g\n", "  m coincident(t == s) g\n")
        .replace("in std.top {\n", "param s: Scalar hint(0.5)\nin std.top {\n");
    refused(&shared.replace("  length(10mm) tl\n", "  length(10mm) tl\n  rate(m over s) parallel tl\n"),
            "`s` is an unknown of the drawing");
}

/// **One kernel reads every word of every source** (#365): a spline's rate, which its tangent
/// kernel held only along a line, is held square to one and measured along one by the same rows a
/// rate in space is — the Bézier's `C′(½) = (45, 30)`, square to `nl` and `|C′|` along `tl`.
#[test]
fn a_splines_rate_is_held_square_and_measured_as_any_rate_is() {
    let src = SPLINE.replace("  length(10mm) tl\n", "  length(10mm) tl\n  g tangent(at: m) tl\n  \
        r := point hint((10, 30))\n  nl := line(m, r)\n  length(10mm) nl\n  \
        rate(m over t) perpendicular nl\n}\nparam v: Length hint(50)\nin std.top {\n  \
        rate(m over t) distance(v, along: tl)\n");
    let e = solved(&src);
    let at = |n: &str| e.sketch.world_point(e.map.ent_named(n).unwrap().i());
    let d = sub(at("r"), at("m"));
    assert!((d[0] * 45.0 + d[1] * 30.0).abs() < 1e-8, "{d:?}");
    let v = e.sketch.params[e.sketch.free_vars["v"] as usize].value;
    assert!((v.abs() - 45f64.hypot(30.0)).abs() < 1e-8, "{v}");
}
