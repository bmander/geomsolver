//! **Rates along a running formal** (#339 D2): a traced body that states a rate, integrated from
//! its home, held to closed forms it never states.

use crate::common::build;

const TRACTRIX: &str = "\
unit mm
use std

component Drag(o: point, a: Length, u: Scalar) {
  q := point
  p := point hint((0, 30))
  q level(up) o
  o distance(a * u, along: right) q
  string := line(p, q)
  length(a) string
  rate(p over u) parallel string
}

in std.front {
  o := point
  fix((0mm, 0mm)) o
  drag := Drag(o, a: 30mm, u: 0)
  drag.p level(right) o
}
tractrix := drag.p over u in (0, 3)
";

/// The tractrix's closed form: `x = a(u − tanh u)`, `y = a sech u`.
fn tractrix(a: f64, u: f64) -> (f64, f64) {
    (a * (u - u.tanh()), a / u.cosh())
}

#[test]
fn the_dragged_point_traces_the_tractrix() {
    let e = build(TRACTRIX);
    let sk = crate::common::solved(&e);
    let k = e.map.ent_named("tractrix").unwrap().i();
    for u in [0.0, 0.4, 1.2, 2.0, 3.0] {
        let (x, y) = sk.curve_point(k, u);
        let (cx, cy) = tractrix(30.0, u);
        assert!((x - cx).abs() < 1e-9 && (y - cy).abs() < 1e-9, "u {u}: ({x}, {y}) against ({cx}, {cy})");
    }
}

#[test]
fn probe_errors() {
    let src = TRACTRIX.to_string() + "
component Dragged(o: point, a: Length, u: Scalar) {
  d := Drag(o, a: a, u: u)
  initially {
    d.p level(right) o
  }
}
written := Dragged(o, a: 30mm, u: 0).d.p over u in (-3, 3)
";
    let e = build(&src);
    let sk = crate::common::solved(&e);
    for name in ["tractrix", "written"] {
        let k = e.map.ent_named(name).unwrap().i();
        let mut worst = 0.0f64;
        for i in 0..=30 {
            let u = if name == "written" { -3.0 + 0.2 * i as f64 } else { 0.1 * i as f64 };
            let (x, y) = sk.curve_point(k, u);
            let (cx, cy) = tractrix(30.0, u);
            worst = worst.max((x - cx).abs().max((y - cy).abs()));
        }
        eprintln!("{name}: worst {worst:e}; polyline {}", sk.curve_polyline(k).len());
    }
}

const CLOTHOID: &str = "\
unit mm
use std

component Clothoid(A: Length, s: Length) {
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
}

in std.front {
  o := point
  x := point
  fix((0mm, 0mm)) o
  fix((10mm, 0mm)) x
  datum := line(o, x)
  cl := Clothoid(A: 50mm, s: 0mm)
  cl.p coincident o
  datum angle(0deg) cl.tl
}
spiral := cl.p over s in (-100mm, 100mm)
";

/// The Fresnel integrals `∫₀ˢ (cos, sin)(σ²/2A²) dσ`, by Simpson's rule on a fine grid — a
/// reference worked out here.
fn fresnel(a: f64, s: f64) -> (f64, f64) {
    let n = 20000;
    let h = s / n as f64;
    let (mut x, mut y) = (0.0, 0.0);
    for i in 0..=n {
        let t = i as f64 * h;
        let w = if i == 0 || i == n { 1.0 } else if i % 2 == 1 { 4.0 } else { 2.0 };
        let ph = t * t / (2.0 * a * a);
        x += w * ph.cos();
        y += w * ph.sin();
    }
    (x * h / 3.0, y * h / 3.0)
}

#[test]
fn probe_clothoid() {
    let e = build(CLOTHOID);
    let sk = crate::common::solved(&e);
    let k = e.map.ent_named("spiral").unwrap().i();
    let mut worst = 0.0f64;
    for i in 0..=20 {
        let s = -100.0 + 10.0 * i as f64;
        let (x, y) = sk.curve_point(k, s);
        let (cx, cy) = fresnel(50.0, s);
        worst = worst.max((x - cx).abs().max((y - cy).abs()));
        if !(x.is_finite()) { eprintln!("nan at {s}"); }
    }
    eprintln!("clothoid worst {worst:e}");
}

#[test]
fn probe_contacts() {
    let src = format!("{CLOTHOID}in std.front {{\nm := point hint((55, 15))\nm coincident(t == 60mm) spiral\n\
        b := point hint((70, 25))\nl := line(m, b)\nspiral tangent(at: m) l\nm distance(20) b\n\
        osc := circle hint(r: 40)\nspiral curvature(t == 80mm) osc\n}}\n");
    let t = std::time::Instant::now();
    let e = build(&src);
    eprintln!("build {:?}", t.elapsed());
    crate::common::fd_jacobian(&e.sketch, 1e-5);
    eprintln!("fd {:?}", t.elapsed());
    let sk = crate::common::solved(&e);
    eprintln!("solve {:?}", t.elapsed());
    let at = |n: &str| sk.point_xy(e.map.ent_named(n).unwrap().i());
    let osc = e.map.ent_named("osc").unwrap().i();
    eprintln!("m {:?} b {:?} osc r {}", at("m"), at("b"), sk.params[sk.circles[osc].radius as usize].value);
    eprintln!("fresnel 60 {:?}", fresnel(50.0, 60.0));
}

#[test]
fn probe_speed() {
    let e = build(CLOTHOID);
    let mut sk = crate::common::solved(&e);
    let k = e.map.ent_named("spiral").unwrap().i();
    let t = std::time::Instant::now();
    for i in 0..10 {
        // a fresh march each time: move the pose a hair
        let p = e.map.ent_named("cl.q").unwrap().i();
        let pp = sk.point_params(p)[0] as usize;
        sk.params[pp].value += 1e-9 * i as f64;
        let _ = sk.curve_point(k, 80.0);
    }
    eprintln!("10 marches+reads: {:?}", t.elapsed());
}

const CATENARY: &str = "\
unit mm
use std

component Chain(c: Length, v: Length, s: Length) {
  p := point
  q := point
  tl := line(p, q)
  p distance(c, along: right) q
  p distance(v + s, along: up) q
  rate(p over s) parallel tl
  rate(p over s) distance(1, along: tl)
}

in std.front {
  a := point
  b := point
  fix((0mm, 0mm)) a
  fix((100mm, 0mm)) b
  hang := Chain(c: hint(30mm), v: hint(-75mm), s: 0mm)
  hang.p coincident a
  b coincident(t == 150mm) chain
}
chain := hang.p over s in (0mm, 150mm)
";

#[test]
fn probe_catenary() {
    let t = std::time::Instant::now();
    let e = build(CATENARY);
    crate::common::fd_jacobian(&e.sketch, 1e-5);
    let sk = crate::common::solved(&e);
    eprintln!("took {:?}", t.elapsed());
    for n in ["hang.c", "hang.v"] {
        let p = sk.free_vars.get(n).copied();
        eprintln!("{n} = {:?}", p.map(|p| sk.params[p as usize].value));
    }
    let k = e.map.ent_named("chain").unwrap().i();
    for s in [0.0, 40.0, 75.0, 150.0] {
        let (x, y) = sk.curve_point(k, s);
        let c: f64 = 30.823647;
        eprintln!("{s}: ({x}, {y}) closed ({}, {})", 50.0 + c * ((s - 75.0) / c).asinh(), (c * c + (s - 75.0f64).powi(2)).sqrt() - (c * c + 75.0f64 * 75.0).sqrt());
    }
}

#[test]
fn probe_short() {
    let src = CLOTHOID.replace("  rate(q over s) distance(10mm * s / A^2, along: nl)\n", "");
    let (prog, errs) = crate::common::parse(&src);
    assert!(errs.is_empty());
    let e = gcs_core::program::elaborate(&prog);
    for d in e.errors() { eprintln!("{} {}", d.code.as_str(), d.message); }
    let src = TRACTRIX.replace("  rate(p over u) parallel string\n", "  rate(p over u) parallel string\n  rate(q over u) parallel string\n");
    let (prog, _) = crate::common::parse(&src);
    let e = gcs_core::program::elaborate(&prog);
    for d in e.errors() { eprintln!("{} {}", d.code.as_str(), d.message); }
    for bad in ["  length(1) rate(p over u)\n", "  rate(p over u) coincident q\n", "  rate(p over a) parallel string\n", "  rate(o over u) parallel string\n"] {
        let src = TRACTRIX.replace("  rate(p over u) parallel string\n", bad);
        let (prog, errs) = crate::common::parse(&src);
        for e in errs { eprintln!("syn {}", e.message); }
        let e = gcs_core::program::elaborate(&prog);
        for d in e.errors() { eprintln!("{} {}", d.code.as_str(), d.message); }
    }
}
