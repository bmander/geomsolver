//! **Rates along a running formal, in a traced body** (#339 D2): a body that states a condition
//! on a derivative along the formal it runs over — `rate(p over s) parallel tl` — integrated from
//! its home.  Each curve here is held to a closed form it never states: the tractrix (drawn, and
//! written in place from `initially`, both ways), the clothoid (the Fresnel integrals, by their
//! series), the hanging chain shot to its far end (today's catenary), and a geodesic on a cone in
//! space (a straight line unrolled).  Contacts on them are held to the closed forms too, their
//! Jacobians to differences; a body short of a rate row is refused naming the freedom it leaves,
//! a rate read as anything but a direction is refused, and a march is refused where its rates
//! lose rank.  A face family's rates (D3) are `rates.rs`'s.

use crate::common::{build, fd_jacobian, parse, solved};
use gcs_core::model::Sketch;
use gcs_core::program::elaborate;

const TRACTRIX: &str = "\
unit mm
use std
use std.motion (turns)

component Drag(o: point, a: Length, u: Scalar) {
  q := point
  p := point hint((0, 30))
  q level(up) o
  o distance(a * u, along: right) q
  string := line(p, q)
  length(a) string
  rate(p over u) parallel string
  initially {
    p level(right) o
  }
}

in std.front {
  o := point
  fix((0mm, 0mm)) o
  drag := Drag(o, a: 30mm, u: 0)
}
tractrix := trace(drag.p over u in (0, 3))
written := trace(Drag(o, a: 30mm, u: 0).p over u in (-3, 3))
";

/// The tractrix's closed form, the string's end at `(a u, 0)`: `x = a(u − tanh u)`, `y = a sech u`.
pub(crate) fn tractrix(a: f64, u: f64) -> (f64, f64) {
    (a * (u - u.tanh()), a / u.cosh())
}

fn curve(e: &gcs_core::program::Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

/// **A point dragged by a string traces the tractrix.**  The body says what holds everywhere —
/// the string's end on the line, `a u` along it, the string `a` long — and one rate row: the
/// point moves only along the string.  Drawn, the home is the instance on the sheet, at the
/// cusp, where the point's speed is zero and its rates' rows have full rank all the same;
/// written in place, `initially` says it, and the march runs from it both ways.
#[test]
fn a_dragged_point_traces_the_tractrix() {
    let e = build(TRACTRIX);
    let sk = solved(&e);
    for (name, from) in [("tractrix", 0.0), ("written", -3.0)] {
        let k = curve(&e, name);
        for i in 0..=30 {
            let u = from + 0.1 * i as f64 * if from < 0.0 { 2.0 } else { 1.0 };
            let (x, y) = sk.curve_point(k, u);
            let (cx, cy) = tractrix(30.0, u);
            assert!((x - cx).abs() < 1e-11 && (y - cy).abs() < 1e-11,
                    "{name} at {u}: ({x}, {y}) against ({cx}, {cy})");
        }
        assert_eq!(sk.curve_polyline(k).len(), gcs_core::model::CURVE_STEPS + 1);
    }
    assert_eq!(gcs_core::report::rates_text(&sk, curve(&e, "tractrix")).as_deref(),
               Some("integrated from its home at 0 across (0, 3)"));
}

const CLOTHOID: &str = "\
unit mm
use std
use std.motion (turns)

component Clothoid(start: point, heading: line, A: Length, s: Length) {
  p := point
  q := point
  k := point
  tl := line(p, q)
  nl := line(p, k)
  length(10mm) tl
  length(10mm) nl
  tl angle(90deg) nl
  rate(p over s) parallel tl
  rate(p over s) distance(1, along: tl)
  rate(q over s) distance(10mm * s / A^2, along: nl)
  initially {
    p coincident start
    heading angle(0deg) tl
  }
}

in std.front {
  o := point
  x := point
  fix((0mm, 0mm)) o
  fix((10mm, 0mm)) x
  datum := line(o, x)
  cl := Clothoid(o, datum, A: 50mm, s: 0mm)
}
spiral := trace(cl.p over s in (-100mm, 100mm))
";

/// The Fresnel integrals `∫₀ˢ (cos, sin)(σ²/2A²) dσ`, by their series — worked out here.
pub(crate) fn fresnel(a: f64, s: f64) -> (f64, f64) {
    let w = s * s / (2.0 * a * a);
    let (mut x, mut y) = (0.0, 0.0);
    // the term w^k / k!, its sign and its integral's factor
    let mut term = 1.0;
    for k in 0..60 {
        let sign = if (k / 2) % 2 == 0 { 1.0 } else { -1.0 };
        let part = sign * term * s / (2 * k + 1) as f64;
        if k % 2 == 0 { x += part } else { y += part }
        term *= w / (k + 1) as f64;
    }
    (x, y)
}

/// **A curvature linear in arc length is the clothoid.**  The body carries the curve's point, its
/// tangent and its normal, each 10 long and square; its three rate rows say the point runs along
/// the tangent at unit speed (so `s` is arc length) and the tangent turns at `κ = s/A²` — a
/// component of a rate along a drawn direction, linear in the rate.  Integrated both ways from
/// the drawn instance, it is the Fresnel spiral.
#[test]
fn a_curvature_linear_in_arc_length_is_the_clothoid() {
    let e = build(CLOTHOID);
    let sk = solved(&e);
    let k = curve(&e, "spiral");
    for i in 0..=40 {
        let s = -100.0 + 5.0 * i as f64;
        let (x, y) = sk.curve_point(k, s);
        let (cx, cy) = fresnel(50.0, s);
        assert!((x - cx).abs() < 1e-10 && (y - cy).abs() < 1e-10,
                "at {s}: ({x}, {y}) against ({cx}, {cy})");
    }
}

/// **A contact on a curve that states a rate owns its place, with `C′` the rate and `C″`, `C‴`
/// its Taylor orders.**  On the clothoid: a point at `s = 60` is the Fresnel point there, a line
/// along its tangent there turns `s²/2A²` from the start, and the circle osculating it at
/// `s = 80` has radius `A²/s` and its centre on the normal — every one a closed form.  Before the
/// solve, with the drawn instance's frame not yet its body's (the curve then the flow from the
/// nearest consistent frame), the assembled Jacobian is the residuals' own, column by column.
#[test]
fn contacts_on_a_rate_curve_hold_its_closed_forms() {
    let src = format!("{CLOTHOID}in std.front {{
m := point hint((55, 15))
m coincident(t == 60mm) spiral
b := point hint((70, 25))
l := line(m, b)
spiral tangent(at: m) l
m distance(20) b
osc := circle hint(r: 40)
spiral curvature(t == 80mm) osc
}}
");
    let e = build(&src);
    fd_jacobian(&e.sketch, 1e-5);
    let sk = solved(&e);
    let at = |n: &str| sk.point_xy(e.map.ent_named(n).unwrap().i());
    let (m, b) = (at("m"), at("b"));
    let want = fresnel(50.0, 60.0);
    assert!((m.0 - want.0).abs() < 1e-10 && (m.1 - want.1).abs() < 1e-10, "{m:?} against {want:?}");
    let turn = 60.0f64 * 60.0 / (2.0 * 50.0 * 50.0);
    let (dx, dy) = ((b.0 - m.0) / 20.0, (b.1 - m.1) / 20.0);
    assert!((dx * turn.sin() - dy * turn.cos()).abs() < 1e-10, "({dx}, {dy}) at {turn} rad");
    let osc = e.map.ent_named("osc").unwrap().i();
    let r = sk.params[sk.circles[osc].radius as usize].value;
    assert!((r - 2500.0 / 80.0).abs() < 1e-9, "radius {r}");
    let c = sk.point_xy(sk.circles[osc].center as usize);
    let (p, turn) = (fresnel(50.0, 80.0), 80.0f64 * 80.0 / 5000.0);
    let want = (p.0 - r * turn.sin(), p.1 + r * turn.cos());
    assert!((c.0 - want.0).abs() < 1e-8 && (c.1 - want.1).abs() < 1e-8, "{c:?} against {want:?}");
}

const CATENARY: &str = "\
unit mm
use std
use std.motion (turns)

component Chain(start: point, c: Length, v: Length, s: Length) {
  p := point
  q := point
  tl := line(p, q)
  p distance(c, along: right) q
  p distance(v + s, along: up) q
  rate(p over s) parallel tl
  rate(p over s) distance(1, along: tl)
  initially {
    p coincident start
  }
}

in std.front {
  a := point
  b := point
  fix((0mm, 0mm)) a
  fix((100mm, 0mm)) b
  hang := Chain(a, c: hint(30mm), v: hint(-75mm), s: 0mm)
  b coincident(t == 150mm) chain
}
chain := trace(hang.p over s in (0mm, 150mm))
";

/// **A two-point problem is a drawing.**  The chain's forces — a horizontal pull `c` and a
/// vertical one grown by the weight below, `v + s` — are unknowns of the drawing; the chain runs
/// along the pull at unit speed, and its far end lands on `b` after its length.  The contact's
/// gradient in `c` and `v` is the flow's sensitivity, so the solve shoots: `c` is the root of
/// `c sinh(50/c) = 75`, `v = −75`, and the curve is today's catenary.
#[test]
fn a_two_point_problem_is_shot_through_the_flow() {
    let e = build(CATENARY);
    fd_jacobian(&e.sketch, 1e-5);
    let sk = solved(&e);
    let value = |n: &str| sk.params[*sk.free_vars.get(n).unwrap() as usize].value;
    let (mut lo, mut hi) = (20.0f64, 40.0f64);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if mid * (50.0 / mid).sinh() > 75.0 { lo = mid } else { hi = mid }
    }
    let c = 0.5 * (lo + hi);
    assert!((value("hang.c") - c).abs() < 1e-9, "c {} against {c}", value("hang.c"));
    assert!((value("hang.v") + 75.0).abs() < 1e-9, "v {}", value("hang.v"));
    let k = curve(&e, "chain");
    for i in 0..=15 {
        let s = 10.0 * i as f64;
        let (x, y) = sk.curve_point(k, s);
        let (cx, cy) = (50.0 + c * ((s - 75.0) / c).asinh(),
                        (c * c + (s - 75.0).powi(2)).sqrt() - (c * c + 75.0f64.powi(2)).sqrt());
        assert!((x - cx).abs() < 1e-9 && (y - cy).abs() < 1e-9,
                "at {s}: ({x}, {y}) against ({cx}, {cy})");
    }
}

const GEODESIC: &str = "\
unit mm
use std
use std.motion (turns)

component Geodesic(o: point, top: point, half: Angle, s: Length) {
  ax := line(o, top)
  p := point hint((20, 0, 35))
  q := point hint((20, 10, 35))
  w := point hint((0, 0, 46))
  m := point hint((25, 0, 44))
  gen := line(o, p)
  tl := line(p, q)
  nl := line(p, w)
  bl := line(p, m)
  gen angle(half) ax
  w coincident ax
  nl perpendicular gen
  length(10mm) tl
  tl perpendicular nl
  length(10mm) bl
  bl perpendicular tl
  bl perpendicular nl
  rate(p over s) parallel tl
  rate(p over s) distance(1, along: tl)
  rate(q over s) perpendicular bl
}

component Started(o: point, top: point, ya: line, half: Angle, r0: Length, s: Length) {
  g := Geodesic(o, top, half: half, s: s)
  initially {
    o distance(0mm, along: ya) g.p
    o distance(r0) g.p
    g.tl perpendicular g.ax
  }
}

o := point hint((0, 0, 0))
fix((0, 0, 0)) o
top := point hint((0, 0, 10))
fix((0, 0, 10)) top
y1 := point hint((0, 10, 0))
fix((0, 10, 0)) y1
ya := line(o, y1)
zl := line(o, top)
path := trace(Started(o, top, ya, half: 30deg, r0: 40mm, s: 0mm).g.p over s in (-60mm, 60mm))
";

/// A geodesic of the cone of half angle 30° from 40 along its generator, setting off square to
/// it: unrolled, the straight line `ρ cos ψ = 40` — `ρ = √(40² + s²)`, turned `atan(s/40)/sin α`
/// round the axis.
pub(crate) fn geodesic(s: f64) -> [f64; 3] {
    let alpha = 30f64.to_radians();
    let rho = (40.0f64 * 40.0 + s * s).sqrt();
    let phi = (s / 40.0).atan() / alpha.sin();
    [rho * alpha.sin() * phi.cos(), rho * alpha.sin() * phi.sin(), rho * alpha.cos()]
}

/// **A curve on a cone whose curvature lies along the cone's normal is a geodesic** — in space.
/// The body draws the point on the cone, the cone's normal there, the tangent square to it and
/// the binormal in the surface; its rate rows say the point runs along the tangent at unit speed
/// and the tangent turns only toward the normal.  In space a `parallel` is two rows, one of
/// which the point on the cone already implies, so its rates' system has more rows than rates
/// and full rank: the index check is a rank.  Written in place, `initially` sets it off; the
/// curve is the unrolled straight line, a point held at a height is where the closed form is
/// that high, a line along its tangent there is the closed form's tangent, and both contacts'
/// Jacobians are their residuals' own.
#[test]
fn a_geodesic_on_a_cone_unrolls_straight() {
    let src = format!("{GEODESIC}\
h := point hint((7, 24, 43))
h coincident path
o distance(43.30127018922194mm, along: zl) h
h2 := point hint((0, 30, 50))
tl := line(h, h2)
path tangent(at: h) tl
h distance(10mm) h2
");
    let e = build(&src);
    let mut sk = solved(&e);
    let r = e.map.ent_named("path").unwrap();
    assert_eq!(r.kind, gcs_core::model::EntKind::Traced);
    for i in 0..=24 {
        let s = -60.0 + 5.0 * i as f64;
        let got = gcs_core::traced::reading(&sk, r.i(), [s, 0.0], gcs_core::traced::Need::Point)
            .unwrap_or_else(|| panic!("no reading at {s}")).position;
        let want = geodesic(s);
        assert!((0..3).all(|k| (got[k] - want[k]).abs() < 1e-10),
                "at {s}: {got:?} against {want:?}");
    }
    let at = |sk: &Sketch, n: &str| sk.world_point(e.map.ent_named(n).unwrap().i());
    let (h, h2) = (at(&sk, "h"), at(&sk, "h2"));
    let want = geodesic(30.0);
    assert!((0..3).all(|k| (h[k] - want[k]).abs() < 1e-9), "{h:?} against {want:?}");
    let (a, b) = (geodesic(30.0 - 1e-4), geodesic(30.0 + 1e-4));
    let t: Vec<f64> = (0..3).map(|k| (b[k] - a[k]) / 2e-4).collect();
    let d: Vec<f64> = (0..3).map(|k| (h2[k] - h[k]) / 10.0).collect();
    let cos = (0..3).map(|k| t[k] * d[k]).sum::<f64>();
    assert!((cos - 1.0).abs() < 1e-7, "{d:?} along {t:?}");
    assert_eq!(gcs_core::report::traced_text(&sk, r.i()),
               "path: a curve traced in space, integrated from its home at 0 across (-60, 60)");
    // the contacts' Jacobians, every column free and moved off the solution
    for p in sk.params.iter_mut() {
        p.fixed = false;
    }
    let mut rng = gcs_core::rng::Rng::new(7);
    for p in sk.params.iter_mut() {
        p.value += rng.uniform(-0.02, 0.02) * (1.0 + p.value.abs().min(10.0));
    }
    // a rate of the contact too, the normal and the tangent being rates (#365)
    for c in sk.constraints.iter()
        .filter(|c| c.kind.traced_contact().is_some() || c.rate_source().is_some()) {
        gcs_core::fdcheck::check_constraint(&sk, c, 1e-6, 1e-6)
            .unwrap_or_else(|why| panic!("{:?}: {why}", c.kind));
    }
}

const HELIX: &str = "\
unit mm
use std
use std.motion (turns)

component Wound(x0: point, vert: line, spin: frame, s: Length) {
  x := point hint((20, 0, 0))
  X := point hint((20, 0, 0))
  rate(x over s) parallel vert
  rate(x over s) distance(1, along: vert)
  X coincident(under: spin, t == 3deg * s / 1mm) x
  initially {
    x coincident(under: spin, t == 30deg) x0
  }
}

o := point hint((0, 0, 0))
fix((0, 0, 0)) o
top := point hint((0, 0, 10))
fix((0, 0, 10)) top
za := line(o, top)
spin := frame
spin turns(1) za
x0 := point hint((20, 0, 0))
fix((20, 0, 0)) x0
helix := trace(Wound(x0, za, spin, s: 0mm).X over s in (-60mm, 60mm))
";

/// The helix of radius 20 about the axis, a turn each 120 up it, from 30° round at `s = 0`.
fn helix(s: f64) -> [f64; 3] {
    let a = (3.0 * s + 30.0).to_radians();
    [20.0 * a.cos(), 20.0 * a.sin(), s]
}

/// **A point a motion carries, in a body that states a rate** (#377).  `x` runs up the axis at
/// unit speed, and `X` is `x` turned about it three degrees a millimetre: a row of the body,
/// integrated with the rest — its rate the carried row differentiated along `s` — and, in
/// `initially`, the home's: `x` starts as `x0` turned 30°.  The curve is the helix; a point held
/// at a height is where the helix is that high, and its Jacobian, through the carried rows'
/// sensitivities to the axis and to `x0`, is its residual's own.
#[test]
fn a_point_a_motion_carries_is_a_row_of_a_rate_body() {
    let src = format!("{HELIX}\
h := point hint((1, 19, 31))
h coincident helix
o distance(20mm, along: za) h
");
    let e = build(&src);
    let mut sk = solved(&e);
    let r = e.map.ent_named("helix").unwrap();
    for i in 0..=24 {
        let s = -60.0 + 5.0 * i as f64;
        let got = gcs_core::traced::reading(&sk, r.i(), [s, 0.0], gcs_core::traced::Need::Point)
            .unwrap_or_else(|| panic!("no reading at {s}")).position;
        let want = helix(s);
        assert!((0..3).all(|k| (got[k] - want[k]).abs() < 1e-10),
                "at {s}: {got:?} against {want:?}");
    }
    let h = sk.world_point(e.map.ent_named("h").unwrap().i());
    let want = helix(20.0);
    assert!((0..3).all(|k| (h[k] - want[k]).abs() < 1e-9), "{h:?} against {want:?}");
    assert_eq!(gcs_core::report::traced_text(&sk, r.i()),
               "helix: a curve traced in space, integrated from its home at 0 across (-60, 60)");
    for p in sk.params.iter_mut() {
        p.fixed = false;
    }
    let mut rng = gcs_core::rng::Rng::new(7);
    for p in sk.params.iter_mut() {
        p.value += rng.uniform(-0.02, 0.02) * (1.0 + p.value.abs().min(10.0));
    }
    for c in sk.constraints.iter().filter(|c| c.kind.traced_contact().is_some()) {
        gcs_core::fdcheck::check_constraint(&sk, c, 1e-6, 1e-6)
            .unwrap_or_else(|why| panic!("{:?}: {why}", c.kind));
    }
}

/// The errors a document elaborates to, or those parsing it finds: a rate read by a word that
/// reads no direction is refused as it is parsed.
fn errors(src: &str) -> Vec<String> {
    let (prog, errs) = parse(src);
    if !errs.is_empty() {
        return errs.into_iter().map(|e| e.message).collect();
    }
    elaborate(&prog).errors().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect()
}

/// **A body one rate row short is refused, naming the freedom it leaves.**  The clothoid without
/// its curvature row: six coordinates against three rows and two rate rows, and the freedom is
/// how its tangent turns — the rates of `q` and `k`, the structure's under-determined block.
#[test]
fn a_body_short_of_a_rate_row_names_its_freedom() {
    let src = CLOTHOID.replace("  rate(q over s) distance(10mm * s / A^2, along: nl)\n", "");
    let errs = errors(&src);
    assert!(errs.iter().any(|e| e.starts_with("E103 a body that states a rate must determine its \
        rates along `s`: 6 unknowns against 3 rows and 2 rate rows — its rates leave 1 freedom \
        among those of `q`, `k`")), "{errs:?}");
}

/// **A rate stands where a direction is read** (#339 D0, decision 1): never as a place, never as
/// a magnitude (a row quadratic in the rate), only along the formal the trace runs over, only of
/// a point the body moves, one to a relation, and not at the home.
#[test]
fn a_rate_is_read_only_as_a_direction() {
    let cases = [
        ("  length(1) rate(p over u)\n", "a rate stands where a direction is read: `rate(x over \
            s) perpendicular y`, `parallel y`, `angle(…) y` or `distance(…, along: u)`, not \
            `length`"),
        ("  rate(p over u) coincident q\n", "not `coincident`"),
        ("  rate(p over a) parallel string\n", "is read along `a`, and the curve runs over `u`"),
        ("  rate(o over u) parallel string\n", "does not move with `u`"),
        ("  rate(p over u) perpendicular rate(q over u)\n", "a relation reads one rate"),
        ("  initially {\n    rate(p over u) parallel string\n  }\n",
         "a rate is no row at the home"),
    ];
    for (row, said) in cases {
        let src = TRACTRIX.replace("  rate(p over u) parallel string\n", row);
        let errs = errors(&src);
        assert!(errs.iter().any(|e| e.contains(said)), "{row}: {errs:?}");
    }
    // along a turn a rate has no unit yet
    let errs = errors("unit mm\nuse std\nuse std.motion (turns)\ncomponent Turn(o: point, x: line, u: Angle) {\n  \
        p := point hint((10, 0))\n  t := line(o, p)\n  o distance(10mm) p\n  \
        x angle(u) t\n  rate(p over u) perpendicular t\n}\nin std.front {\n  o := point\n  \
        e := point\n  x := line(o, e)\n  fix((0, 0)) o\n  fix((10, 0)) e\n}\n\
        k := trace(Turn(o, x, u: 0deg).p over u in (0deg, 90deg))\n");
    assert!(errs.iter().any(|e| e.contains("a rate along an angle is not yet read")), "{errs:?}");
    // outside a component nothing runs
    let errs = errors("unit mm\nuse std\nuse std.motion (turns)\nin std.front {\n  p := point\n  q := point\n  \
                       l := line(p, q)\n  rate(p over s) parallel l\n}\n");
    assert!(errs.iter().any(|e| e.contains("`s` is no formal here")), "{errs:?}");
}

/// **A march is refused where its rates lose rank, at that place.**  A point on a circle whose
/// rate along `right` is 1: where the circle's tangent is square to `right` the rates have no
/// solution, and the march stops there, each way.  Short of it the curve is the circle.
#[test]
fn a_march_is_refused_where_its_rates_lose_rank() {
    let src = "\
unit mm
use std
use std.motion (turns)

component Round(o: point, u: Scalar) {
  p := point hint((0, 30))
  o distance(30mm) p
  rate(p over u) distance(1, along: right)
  initially {
    p level(right) o
  }
}

in std.front {
  o := point
  fix((0mm, 0mm)) o
  r := Round(o, u: 0)
}
arc := trace(r.p over u in (-40, 40))
";
    let e = build(src);
    let sk = solved(&e);
    let k = curve(&e, "arc");
    for u in [-29.0, -10.0, 0.0, 15.0, 29.0] {
        let (x, y) = sk.curve_point(k, u);
        assert!((x - u).abs() < 1e-9 && (y - (900.0 - u * u).sqrt()).abs() < 1e-9,
                "at {u}: ({x}, {y})");
    }
    assert!(sk.curve_point(k, 35.0).0.is_nan());
    // where each march stopped: at the rank loss, or past it by at most the substep that found
    // it there (`ode::Refusal::at`)
    let gcs_core::model::CurveBody::Trace(l) = &sk.curve_defs[sk.curves[k].def as usize].body else {
        panic!("a trace")
    };
    let reach =
        gcs_core::locus::rate_reach(&l.flat, &sk.curve_vars(k, 0.0), 0.0, (-40.0, 40.0)).unwrap();
    for (r, end) in reach.iter().zip([-30.0, 30.0]) {
        match r {
            Err(Some(gcs_core::ode::Refusal::Undefined { at })) => {
                assert!((at - end).abs() < 1e-2 && at.abs() >= 30.0 - 1e-9, "refused at {at}")
            }
            other => panic!("{other:?}"),
        }
    }
    let said = gcs_core::report::rates_text(&sk, k).unwrap();
    assert!(said.starts_with("integrated from its home at 0, refused at -30.0")
        && said.ends_with("its rates lose rank there"), "{said}");
}

/// The spelling prints as it is written: a rate, and a rate alone before its word.
#[test]
fn a_rate_prints_as_written() {
    let src = "component C(p: point, l: line, s: Length) {\n  rate(p over s) parallel l\n  \
               rate(p over s) distance(1, along: l)\n}\n";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let comp = &prog.components[0];
    let mut out = Vec::new();
    for st in &comp.body {
        let mut s = String::new();
        gcs_core::syntax::write_stmt_to(&mut s, &st.kind).unwrap();
        out.push(s);
    }
    assert_eq!(out, ["rate(p over s) parallel l", "rate(p over s) distance(1, along: l)"]);
}
