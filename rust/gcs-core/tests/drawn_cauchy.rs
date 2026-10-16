//! **Cauchy data on a drawn curve** (#376): a marched body's `initially` holds its moved point on
//! a curve the drawing has at the sampled formal, `p coincident(t == u) k` — a spline, a curve
//! drawn in a held plane (a formula, a trace, an envelope) or a curve traced in space — and the
//! march's sensitivities to the body's columns run through the curve's.
//!
//! The gate: `rates/bore.sv`, the Wankel bore read off the apex's envelope, is `housing.sv`'s
//! bore from the apex the motion carries, and refused at its waist where `housing.sv`'s is;
//! `rates/pitch.sv`'s pitch curve marched from the heart cam's profile is the envelope of the
//! roller's centre, and a drawn cam's roller larger than its hollow's radius of curvature is
//! refused there.  Beside them a spline and a curve traced in space as Cauchy data, a contact's
//! Jacobian through a curve's columns against central differences, and the refusals.

use crate::common::{build, parse, solved};
use gcs_core::model::Sketch;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::traced::Need;

const BORE: &str = include_str!("../../examples/rates/bore.sv");
const HOUSING: &str = include_str!("../../examples/rates/housing.sv");
const PITCH: &str = include_str!("../../examples/rates/pitch.sv");

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

fn far(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|k| (a[k] - b[k]).abs()).fold(0.0, f64::max)
}

fn at(sk: &Sketch, i: usize, d: f64, u: f64) -> [f64; 3] {
    gcs_core::traced::reading(sk, i, [d, u], Need::Point)
        .unwrap_or_else(|| panic!("no reading at ({d}, {u})")).position
}

/// The epitrochoid (R 75, e 15) at eccentric angle `u` degrees, moved `d` along its outward
/// normal — `housing.sv`'s closed form.
fn housing(d: f64, u: f64) -> [f64; 3] {
    let (r, e, u) = (75.0, 15.0, u.to_radians());
    let (x, y) = (e * (3.0 * u).cos() + r * u.cos(), e * (3.0 * u).sin() + r * u.sin());
    let tx = -3.0 * e * (3.0 * u).sin() - r * u.sin();
    let ty = 3.0 * e * (3.0 * u).cos() + r * u.cos();
    let l = tx.hypot(ty);
    [x + d * ty / l, y - d * tx / l, 0.0]
}

/// The refusal a marched surface reaches on `pieces` toward `to`: where along its first formal,
/// and the node along its second where it folds.
fn folded(sk: &Sketch, i: usize, pieces: usize, to: f64) -> (f64, f64) {
    match gcs_core::traced::lines_reached(sk, i, pieces, to) {
        Some(Err(Some((refusal, gcs_core::lines::Why::Fold { sigma })))) => (refusal.at(), sigma),
        other => panic!("{pieces} pieces: {other:?}"),
    }
}

/// **The bore read off the apex's path is `housing.sv`'s.**  The envelope's parameter is the
/// shaft's roll, three times the eccentric angle, and at every place read the front marched from
/// it stands where the front marched from the apex the motion carries does (#377) — the two
/// marches the same to rounding (5e-12), far inside either's own error from the closed form — and
/// run past the waist's radius of curvature both are refused at the waist at the same place, on 16
/// pieces and on 32.
#[test]
fn the_bore_read_off_the_apex_path_is_the_carried_apexs() {
    let (b, h) = (build(BORE), build(HOUSING));
    let (sb, sh) = (solved(&b), solved(&h));
    let (bore, written) = (index(&b, "bore"), index(&h, "bore"));
    for d in [0.0, 1.0, 2.0, 3.0] {
        let (mut apart, mut off) = (0.0f64, 0.0f64);
        for k in 0..=72 {
            let u = 2.5 * k as f64;
            let x = at(&sb, bore, d, u);
            apart = apart.max(far(x, at(&sh, written, d, u)));
            off = off.max(far(x, housing(d, u)));
        }
        assert!(apart < 1e-10, "at {d}: {apart:.1e} from `housing.sv`'s");
        assert!(off < 5e-6, "at {d}: {off:.1e} from the closed form");
    }
    let (past, written) = (index(&b, "past"), index(&h, "past"));
    for pieces in [16, 32] {
        let (x, sx) = folded(&sb, past, pieces, 20.0);
        let (y, sy) = folded(&sh, written, pieces, 20.0);
        // at the waist's node, on 32 pieces the waist itself
        assert!(sx == sy && (sx - 90.0).abs() < 2.0, "{pieces} pieces: at {sx}, {sy}");
        assert!((x - y).abs() < 1e-3 && x < 15.0, "{pieces} pieces: {x} against {y}");
    }
    assert_eq!(gcs_core::report::traced_text(&sb, bore),
               "bore: a surface traced in space, marched from its home at 0 across (0, 3)");
    let said = gcs_core::report::traced_text(&sb, past);
    assert!(said.starts_with("past: a surface traced in space, marched from its home at 0, \
        refused at 13.7") && said.ends_with(": it folds there, near 90 along its second formal"),
            "{said}");
}

/// The basis of the plane curve `i` is drawn in, and a place in it lifted into space.
fn lift(sk: &Sketch, i: usize, q: (f64, f64)) -> [f64; 3] {
    let b = sk.basis(sk.curve_view(i).expect("a curve drawn in a plane"));
    std::array::from_fn(|k| b.o[k] + q.0 * b.u[k] + q.1 * b.v[k])
}

/// **A cam's pitch curve is marched from its profile.**  At the home the front is the heart cam's
/// profile, the envelope of the roller; marched the roller's radius out, it is the envelope of the
/// roller's centre at the same roll, everywhere along the rise but its ends.
#[test]
fn a_pitch_curve_is_marched_from_its_profile() {
    let e = build(PITCH);
    let sk = solved(&e);
    let (i, profile, centre) = (index(&e, "pitch"), index(&e, "cam_rise"), index(&e, "pitch_rise"));
    for (d, curve, tol) in [(0.0, profile, 1e-10), (6.0, centre, 1e-8)] {
        let worst = (0..=64).map(|k| {
            let u = 10.0 + 2.5 * k as f64;
            far(at(&sk, i, d, u), lift(&sk, curve, sk.curve_point(curve, u)))
        }).fold(0.0, f64::max);
        assert!(worst < tol, "at {d}: {worst:.1e}");
    }
    assert_eq!(gcs_core::report::traced_text(&sk, i),
               "pitch: a surface traced in space, marched from its home at 0 across (0, 6)");
}

/// The three-lobed cam `pitch.sv` draws, r = 30 + 6 cos 3u, moved `d` out along its normal.
fn lobed(d: f64, u: f64) -> [f64; 3] {
    let u = u.to_radians();
    let (r, dr) = (30.0 + 6.0 * (3.0 * u).cos(), -18.0 * (3.0 * u).sin());
    let (tx, ty) = (dr * u.cos() - r * u.sin(), dr * u.sin() + r * u.cos());
    let l = tx.hypot(ty);
    // the front plane: its first axis along x, its second along z
    [r * u.cos() + d * ty / l, 0.0, r * u.sin() - d * tx / l]
}

/// **The undercut is the fold.**  A roller of 15 follows the drawn cam: its centre's path is the
/// profile's parallel curve.  One of 22 cannot reach into the hollows, whose radius of curvature
/// is 19.2: the march is refused at the hollow (u = 60°), short of 19.2 where a node's area falls
/// to a hundredth of its home's — on 16 pieces and nearer it on 32, as `solventc` says.
#[test]
fn the_undercut_is_the_fold() {
    let e = build(PITCH);
    let sk = solved(&e);
    let (follows, undercut) = (index(&e, "follows"), index(&e, "undercut"));
    for (d, tol) in [(0.0, 1e-10), (5.0, 1e-7), (15.0, 5e-6)] {
        let worst = (0..=60).map(|k| {
            let u = 30.0 + k as f64;
            far(at(&sk, follows, d, u), lobed(d, u))
        }).fold(0.0, f64::max);
        assert!(worst < tol, "at {d}: {worst:.1e}");
    }
    let rho = 24.0 * 24.0 / (54.0 - 24.0);
    let mut last = 0.0;
    for pieces in [16, 32] {
        let (x, sigma) = folded(&sk, undercut, pieces, 22.0);
        assert!((sigma - 60.0).abs() < 1e-9, "{pieces} pieces: folds at u = {sigma}");
        assert!(x > rho - 1.5 && x < rho * (1.0 - gcs_core::lines::FOLD) && x > last,
                "{pieces} pieces: refused at {x}");
        last = x;
    }
    assert_eq!(gcs_core::report::traced_text(&sk, follows),
               "follows: a surface traced in space, marched from its home at 0 across (0, 15)");
    let said = gcs_core::report::traced_text(&sk, undercut);
    assert!(said.starts_with("undercut: a surface traced in space, marched from its home at 0, \
        refused at 18.") && said.ends_with(": it folds there, near 60 along its second formal"),
            "{said}");
}

/// A front in the top view's plane, `front.sv`'s with its Cauchy data read off `k` at `t == u`:
/// moving at `speed` along its normal, whose far end is seeded at `toward` (which picks its side).
fn front_on(k: &str, speed: f64, toward: &str, over: &str) -> String {
    format!("
component Front(c: point, z1: point, k: curve, d: Length, u: Angle) {{
  za := line(c, z1)
  p := point hint((0, 0, 0))
  m := point hint({toward})
  nl := line(p, m)
  length(10mm) nl
  c distance(0mm, along: za) m
  nl perpendicular rate(p over u)
  rate(p over d) parallel nl
  rate(p over d) distance({speed}, along: nl)
  initially {{
    p coincident(t == u) k
  }}
}}
c := point hint((0, 0, 0))
fix((0, 0, 0)) c
z1 := point hint((0, 0, 10))
fix((0, 0, 10)) z1
front := Front(c, z1, {k}).p over {over}
")
}

/// **A spline is Cauchy data, over its control points' lifts.**  A front set off from a spline in
/// the top view moves square to it at unit speed: at every place read it stands `d` from the
/// spline's point at its parameter, square to the spline's tangent there — the parallel curve,
/// whatever the curve.
#[test]
fn a_spline_is_cauchy_data() {
    let src = format!("unit mm
use std
in std.top {{
  k0 := point
  fix((-40, 0)) k0
  k1 := point
  fix((-20, 25)) k1
  k2 := point
  fix((10, 30)) k2
  k3 := point
  fix((30, 15)) k3
  k4 := point
  fix((40, -5)) k4
  rim := spline(k0, k1, k2, k3, k4)
}}
{}", front_on("rim", 1.0, "(0, 0, 0)", "(d in (0mm, 4mm), u in (0.1, 0.9))"));
    let e = build(&src);
    let sk = solved(&e);
    let (i, s) = (index(&e, "front"), index(&e, "rim"));
    for d in [0.0, 2.0, 4.0] {
        let (mut off, mut square) = (0.0f64, 0.0f64);
        for k in 0..=40 {
            let u = 0.1 + 0.02 * k as f64;
            let f = gcs_core::curve::eval(&sk, s, u);
            let x = at(&sk, i, d, u);
            let w = [x[0] - f.p.0, x[1] - f.p.1];
            off = off.max((w[0].hypot(w[1]) - d).abs()).max(x[2].abs());
            square = square.max((w[0] * f.d1.0 + w[1] * f.d1.1).abs() / f.d1.0.hypot(f.d1.1));
        }
        assert!(off < 1e-7 && square < 1e-7, "at {d}: {off:.1e} off, {square:.1e} along");
    }
}

/// **A curve traced in space is Cauchy data, through its contact.**  A circle of 20 about the
/// origin, written in place over its angle and traced in space; the front set off from it inward
/// is the circle of 20 − d, at every place read.
#[test]
fn a_curve_traced_in_space_is_cauchy_data() {
    let src = format!("unit mm
use std
component Circle(o: point, x1: point, y1: point, z1: point, r: Length, u: Angle) {{
  xa := line(o, x1)
  ya := line(o, y1)
  za := line(o, z1)
  q := point hint((r * cos(u), r * sin(u), 0mm))
  o distance(r * cos(u), along: xa) q
  o distance(r * sin(u), along: ya) q
  o distance(0mm, along: za) q
}}
o := point hint((0, 0, 0))
fix((0, 0, 0)) o
x1 := point hint((10, 0, 0))
fix((10, 0, 0)) x1
y1 := point hint((0, 10, 0))
fix((0, 10, 0)) y1
o1 := point hint((0, 0, 10))
fix((0, 0, 10)) o1
ring := Circle(o, x1, y1, o1, r: 20mm).q over u in (0deg, 360deg)
{}", front_on("ring", -1.0, "(50 * cos(u), 50 * sin(u), 0)",
                 "(d in (0mm, 10mm), u in (20deg, 140deg))"));
    let e = build(&src);
    assert_eq!(e.map.ent_named("ring").unwrap().kind, gcs_core::model::EntKind::Traced);
    let sk = solved(&e);
    let i = index(&e, "front");
    for d in [0.0, 5.0, 10.0] {
        let worst = (0..=48).map(|k| {
            let u = 20.0 + 2.5 * k as f64;
            let (r, a) = (20.0 - d, u.to_radians());
            far(at(&sk, i, d, u), [r * a.cos(), r * a.sin(), 0.0])
        }).fold(0.0, f64::max);
        assert!(worst < 1e-7, "at {d}: {worst:.1e}");
    }
}

/// **The march's sensitivities run through the curve's columns.**  A point held on the pitch
/// curve marched from the heart cam's profile: the contact's Jacobian in every column — the
/// point, its place, and the geometry the profile is generated from (the cam's centre, the
/// follower's line, the roller: the three it moves with most), the last through the curve's own
/// gradient at the home and the variational equations after — is its residual's own, against
/// central differences.
#[test]
fn the_march_reads_the_curves_columns() {
    let e = build(PITCH);
    let sk = solved(&e);
    let (i, centre) = (index(&e, "pitch"), index(&e, "pitch_rise"));
    let w = lift(&sk, centre, sk.curve_point(centre, 70.0));
    let src = format!("{PITCH}
h := point hint(({}, {}, {}))
fix(({}, {}, {})) h
h coincident pitch
", w[0] + 0.2, w[1], w[2] - 0.1, w[0], w[1], w[2]);
    let e = build(&src);
    let mut sk = solved(&e);
    assert_eq!(index(&e, "pitch"), i);
    let contact = sk.constraints.iter().rev()
        .find(|c| c.kind == gcs_core::constraints::CKind::PointOnTraced).unwrap().clone();
    let v = contact.local_values(&sk);
    assert!((v[3] - 6.0).abs() < 1e-7 && (v[4] - 70.0).abs() < 1e-5, "{:?}", &v[3..5]);
    for p in sk.params.iter_mut() {
        p.fixed = false;
    }
    let mut rng = gcs_core::rng::Rng::new(11);
    for p in sk.params.iter_mut() {
        p.value += rng.uniform(-0.01, 0.01) * (1.0 + p.value.abs().min(10.0));
    }
    let v = contact.local_values(&sk);
    let rows = contact.n_residuals();
    let got = contact.jacobian(&sk, &v);
    // the point and the place, then the body's columns: the two fixed points' lifts, and past
    // them the profile's — of which the three the contact moves with most, each a march of its own
    let (lead, profile_from) = (3 + 2, 3 + 2 + 6);
    assert!(v.len() > profile_from, "{} columns", v.len());
    let mut profile: Vec<usize> = (profile_from..v.len()).collect();
    let most = |k: &usize| (0..rows).map(|r| got[r * v.len() + k].abs()).fold(0.0, f64::max);
    profile.sort_by(|a, b| most(b).total_cmp(&most(a)));
    assert!(most(&profile[0]) > 1e-2, "no column of the profile moves the pitch curve");
    for k in (0..lead).chain(profile.into_iter().take(3)) {
        let h = 1e-4 * v[k].abs().max(1.0);
        let (mut lo, mut hi) = (v.clone(), v.clone());
        lo[k] -= h;
        hi[k] += h;
        let (a, b) = (contact.residual(&sk, &lo), contact.residual(&sk, &hi));
        for r in 0..rows {
            let fd = (b[r] - a[r]) / (2.0 * h);
            let j = got[r * v.len() + k];
            assert!((fd - j).abs() < 1e-5 * (1.0 + j.abs()),
                    "row {r}, column {k}: {j} against {fd}");
        }
    }
}

/// The errors a document elaborates to.
fn errors(src: &str) -> Vec<String> {
    let (prog, errs) = parse(src);
    if !errs.is_empty() {
        return errs.into_iter().map(|e| e.message).collect();
    }
    elaborate(&prog).errors().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect()
}

/// **What is refused, saying why**: a point on the curve stated outside `initially` (Cauchy data
/// alone), the curve read by any other relation, a surface handed as the curve, and a curve given
/// to a body over one formal (a curve of a plane, written over no curve).
#[test]
fn what_is_refused() {
    let initially = "  initially {\n    p coincident(t == 3 * u) k               // the Cauchy \
                     data: the apex's path\n  }\n";
    assert!(BORE.contains(initially));
    let cases = [
        (initially, "  p coincident(t == 3 * u) k\n",
         "a point on `k` is Cauchy data: it is said in `initially { … }` of a body marched"),
        ("  nl perpendicular rate(p over u)", "  nl perpendicular rate(p over u)\n  nl tangent k",
         "a marched body reads `k` as its Cauchy data alone"),
        ("past := Bore(c, z1, trochoid)", "past := Bore(c, z1, bore)",
         "`k`: a surface is no curve"),
    ];
    for (from, to, said) in cases {
        let errs = errors(&BORE.replace(from, to));
        assert!(errs.iter().any(|e| e.contains(said)), "{to}: {errs:?}");
    }
    // over one formal a body is a curve, and no curve is written over another
    let one = BORE
        .replace("bore := Bore(c, z1, trochoid).p over (d in (0mm, s), u in (0deg, 180deg))\n", "")
        .replace("past := Bore(c, z1, trochoid).p over (d in (0mm, 20mm), u in (0deg, 180deg))\n",
                 "")
        .replace("  nl perpendicular rate(p over u)            // ...square to the front\n", "")
        + "line_of := Bore(c, z1, trochoid, u: 30deg).p over d in (0mm, 3mm)\n";
    let errs = errors(&one);
    assert!(errs.iter().any(|e| e.contains("a curve cannot be written over a curve")), "{errs:?}");
}
