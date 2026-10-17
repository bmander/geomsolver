//! **Paths and frames** (#243 L4): a station word stands on any guide drawn in a plane — a line,
//! an arc, a spline, a traced curve, a named chain of lines and arcs — and its frame is held to
//! an `up`: along a guide in one plane with `up` its normal, the rotation-minimising frame.  A
//! chain is one path, its station run over it link by link (`s` from 0 to the chain's length in
//! links), each link's first station started where the last one's ended.  Gated by closed forms
//! off the exact B-rep: a section square to its path with its centroid on it sweeps its area
//! times the path's length (Pappus, piece by piece along a chain), and under a law in `s` the
//! integral of its area along the path.
use gcs_core::program::Elaborated;
use std::f64::consts::PI;

use crate::common::parsed::determined;

const PIPE: &str = include_str!("../../../docs/lofts/l0/pipe.sv");

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no solid `{name}`")).i()
}

/// The exact solid `name`, its boundary checked.
fn exact_of(e: &Elaborated, name: &str) -> std::rc::Rc<gcs_core::solid::Exact> {
    let x = e.sketch.exact_solid(index(e, name)).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    x.brep.check(1e-9 * x.brep.size()).unwrap_or_else(|k| panic!("`{name}`: {}", k.msg));
    x
}

/// The volume of the solid `name` off its exact B-rep, its boundary checked first.
fn exact(e: &Elaborated, name: &str) -> f64 {
    let x = exact_of(e, name);
    gcs_core::brep::props::volume(&x.brep) / x.mm.powi(3)
}

/// `got` within `rel` of `want`, said with both.
fn near(got: f64, want: f64, rel: f64, what: &str) {
    assert!((got - want).abs() <= rel * want.abs(), "{what}: {got} against {want} (±{rel})");
}

/// The words a family is refused with, where it elaborates at all.
fn refused(src: &str) -> Vec<String> {
    let (mut prog, errs) = crate::common::parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    gcs_core::modules::link(&mut prog, &mut gcs_core::library::resolve);
    let e = gcs_core::program::elaborate(&prog);
    e.errors().map(|d| d.message.clone()).collect()
}

/// A tube's section, of radius 3 and its centre `above` the station's origin along its plane's
/// `v`, swept along `guide` by `word`, over the guides below: a line 40 long, a quarter arc of
/// radius 30 and a cubic Bézier, in plan.
fn tube(word: &str, kind: &str, guide: &str, above: f64) -> String {
    let centre = if above == 0.0 {
        "k := circle(center: st.n.origin)".to_string()
    } else {
        format!("c := point\n    c on_v(d: {above}mm) st.n\n    k := circle(center: c)")
    };
    format!("unit mm
use std (on_v)

component Tube(g: {kind}, up: axis, s: Scalar) {{
  st := std.{word}(g, up: up, s: s)
  in st.n {{
    {centre}
    radius(3mm) k
  }}
  f := face(k)
}}

in std.top {{
  l0 := point
  l1 := point
  fix((-60mm, 0mm)) l0
  fix((-60mm, 40mm)) l1
  straight := line(l0, l1)
  c := point
  e := point
  fix((0mm, 0mm)) c
  fix((30mm, 0mm)) e
  x := point hint((0, 30))
  bend := arc(center: c, start: e, end: x)
  horizontal line(c, e)
  vertical line(c, x)
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((60mm, 0mm)) k0
  fix((60mm, 20mm)) k1
  fix((80mm, 30mm)) k2
  fix((100mm, 30mm)) k3
  curvy := spline(k0, k1, k2, k3)
}}
param s: Scalar over (0, 1)
tube := sweep(Tube({guide}, std.z).f over s)
")
}

/// The length of the Bézier through `ctrl`, by Gauss–Legendre over many panels: to the last digit.
fn bezier_length(ctrl: [(f64, f64); 4]) -> f64 {
    let speed = |t: f64| {
        let u = 1.0 - t;
        let d = |k: usize| -> f64 {
            let c: Vec<f64> = ctrl.iter().map(|p| if k == 0 { p.0 } else { p.1 }).collect();
            3.0 * (u * u * (c[1] - c[0]) + 2.0 * u * t * (c[2] - c[1]) + t * t * (c[3] - c[2]))
        };
        d(0).hypot(d(1))
    };
    let (x, w) = ([-0.906179845938664, -0.538469310105683, 0.0, 0.538469310105683,
                   0.906179845938664],
                  [0.236926885056189, 0.478628670499366, 0.568888888888889, 0.478628670499366,
                   0.236926885056189]);
    let n = 400;
    (0..n).map(|k| {
        let (a, b) = (k as f64 / n as f64, (k + 1) as f64 / n as f64);
        let half = 0.5 * (b - a);
        (0..5).map(|i| w[i] * speed(0.5 * (a + b) + half * x[i])).sum::<f64>() * half
    }).sum()
}

/// **One word over every guide drawn in a plane**: `std.On` round an arc — E062 until L4, its
/// tangency at a point of an arc read in space — is Pappus, π·3²·30·π/2, and the same solid
/// `std.OnArc` sweeps; along a spline a tube is its area times the spline's length.
#[test]
fn on_stands_on_an_arc_and_a_spline_at_pappus() {
    let vol = |word: &str, kind: &str, guide: &str| {
        exact(&determined(&tube(word, kind, guide, 0.0)), "tube")
    };
    near(vol("On", "path", "bend"), 9.0 * PI * 30.0 * PI / 2.0, 1e-9, "On round an arc");
    near(vol("OnArc", "arc", "bend"), 9.0 * PI * 30.0 * PI / 2.0, 1e-9, "OnArc");
    let length = bezier_length([(60.0, 0.0), (60.0, 20.0), (80.0, 30.0), (100.0, 30.0)]);
    near(vol("On", "path", "curvy"), 9.0 * PI * length, 1e-9, "On along a spline");
    near(vol("OnSpline", "spline", "curvy"), 9.0 * PI * length, 1e-9, "OnSpline");
}

/// **The frame is the one written, on every guide**: `v` along `up` and the normal along the
/// path, so a section drawn 5 above its station's origin rides 5 above the guide — never
/// mirrored below it, whichever way the solve might have started.
#[test]
fn the_frame_keeps_up_up_on_every_guide() {
    for (word, kind, guide) in [("On", "path", "straight"), ("OnLine", "line", "straight"),
                                ("On", "path", "bend"), ("On", "path", "curvy"),
                                ("OnSpline", "spline", "curvy")] {
        let e = determined(&tube(word, kind, guide, 5.0));
        let x = exact_of(&e, "tube");
        let (lo, hi) = x.brep.bounds();
        let z = |v: f64, k: usize| v / x.mm + x.origin[k];
        let (bottom, top) = (z(lo[2], 2), z(hi[2], 2));
        assert!((bottom - 2.0).abs() < 1e-6 && (top - 8.0).abs() < 1e-6,
                "{word} along {guide}: z from {bottom} to {top}");
    }
}

/// **A pipe along a chain is the sum of its pieces' Pappus**: three straight links of 20 and two
/// quarter turns of R25, one station word over all five, the joints never glued.
#[test]
fn a_pipe_along_a_chain_is_its_pieces_pappus() {
    let e = determined(PIPE);
    near(exact(&e, "pipe"), 16.0 * PI * (60.0 + 25.0 * PI), 1e-9, "the pipe");
    // one wall, the circle's, across all five links, and the two caps
    let x = exact_of(&e, "pipe");
    let mut names: Vec<&str> = x.brep.faces.iter().map(|f| f.name.as_str()).collect();
    names.sort();
    names.dedup();
    assert_eq!(names, ["pipe.end", "pipe.k", "pipe.start"], "{names:?}");
}

/// **A law reads the chain's `s`**: a pipe whose radius grows with `s` along the chain, link by
/// link — link `k`'s station reading `k + ` its own fraction — sweeps ∫ π r(s)² along the path,
/// each link `π ∫ₖ^{k+1} r(s)² ds` times its length.
#[test]
fn a_law_reads_the_chain_s_running_number() {
    let src = PIPE.replace("radius(r) k", "radius(r + 0.5mm * s) k");
    let e = determined(&src);
    let lengths = [20.0, 25.0 * PI / 2.0, 20.0, 25.0 * PI / 2.0, 20.0];
    // ∫ₖ^{k+1} (4 + s/2)² ds, exactly
    let cube = |s: f64| (4.0 + 0.5 * s).powi(3) / 1.5;
    let want: f64 = lengths.iter().enumerate()
        .map(|(k, l)| PI * l * (cube(k as f64 + 1.0) - cube(k as f64))).sum();
    near(exact(&e, "pipe"), want, 1e-9, "the growing pipe");
}

/// **Part of a chain** is the links its interval reaches, each its own part: from halfway along
/// the first bend to halfway along the second, half a bend, a straight and half a bend.
#[test]
fn part_of_a_chain_is_the_links_it_reaches() {
    let e = determined(&PIPE.replace("param s: Scalar over (0, 5)", "param s: Scalar over (1.5, 3.5)"));
    near(exact(&e, "pipe"), 16.0 * PI * (20.0 + 25.0 * PI / 2.0), 1e-9, "half the U");
}

/// **What a chain cannot be run over is said**: a closed chain, whose ends meet; an interval past
/// its links; and a contact on another curve pinned to `s`, which on a link past the first is
/// that link's number plus its index.
#[test]
fn a_chain_family_refuses_what_it_cannot_run() {
    let said = refused(&PIPE.replace("param s: Scalar over (0, 5)", "param s: Scalar over (0, 6)"));
    assert!(said.iter().any(|m| m.contains("has 5 links") && m.contains("within (0, 5)")),
            "{said:?}");
    let closed = "unit mm
use std

component Tube(g: path, up: axis, s: Scalar) {
  st := std.On(g, up: up, s: s)
  in st.n {
    k := circle(center: st.n.origin)
    radius(1mm) k
  }
  f := face(k)
}

in std.top {
  a := point
  b := point hint((20, 0))
  c := point hint((20, 20))
  d := point hint((0, 20))
  fix((0, 0)) a
  ring := length(20mm) horizontal (ab := line(a, b)) -> vertical (bc := line(b, c)) ->
    horizontal (cd := line(c, d)) -> length(20mm) vertical (da := line(d, a)) -> close
}
param s: Scalar over (0, 4)
body := sweep(Tube(ring, std.z).f over s)
";
    let said = refused(closed);
    assert!(said.iter().any(|m| m.contains("closed chain")), "{said:?}");
    let pinned = "  f := face(k)\n  w := point\n  w coincident(t == s) rail\n";
    let other = PIPE.replace("  f := face(k)\n", pinned)
        .replace("component Tube(g: path, up: axis", "component Tube(g: path, rail: line, up: axis")
        .replace("Tube(route, std.z", "Tube(route, l0, std.z");
    let said = refused(&other);
    assert!(said.iter().any(|m| m.contains("runs over a chain")), "{said:?}");
}
