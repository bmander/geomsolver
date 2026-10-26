//! **Several sections, ruled or smooth** (#243 L7): a loft through several sections is drawn as
//! its sections' corners and a **rail** for each corresponding corner — the corner's places in
//! order, one a section — and a station whose corners stand where their rails meet its plane.
//! Two general features say it, each with its use outside lofts:
//!
//! * **chains run in step**: a family given several named open chains runs them link by link
//!   together, so a guide cut at each section and a chain of lines through each corner make the
//!   **ruled** loft, piecewise in the running number, an edge at every section between;
//! * **a spline through a chain's corners**, `spline(through: rims, start: l, end: m)`: the cubic
//!   through them, a span a link, its ends along the tangents written or else running straight —
//!   the **smooth** loft's rail.
//!
//! A loft with **no guide** stands its station on its rails, the plane through three of them at
//! `s` (`std.stations.Across`).  Gated off the exact B-rep to a part in 10⁹: a ruled loft is the
//! sum of its frusta and prismatoids, a smooth one its area's integral by a quadrature exact for
//! its rails' degree on every span, guided or not, and its ends meet the face beside them square
//! (the cap) or tangent (a neck).
use gcs_core::brep::geom::Surface;
use gcs_core::program::Elaborated;
use std::f64::consts::PI;

use crate::common::parsed::determined;

/// The bottle of `docs/lofts/l0/bottle.sv`, drawn to be lofted all three ways: its axis whole
/// (`whole`) and cut at the shoulder (`axis`), its rims in order (`rims`, the ruled rail) and the
/// spline through them square to the base and the neck (`rail`), and a ring standing on any guide
/// whose rim reaches any rail.
const BOTTLE: &str = "unit mm
use std
use std.stations

component Ring(g: path, up: axis, front: plane, rail: path, s: Scalar) {
  st := std.stations.On(g, up: up, s: s)
  rim := point in st.n, front
  rim coincident rail
  in st.n {
    k := circle(center: st.n.origin)
    rim coincident k
  }
  f := face(k)
}

in std.front {
  base := point
  shoulder := point
  top := point
  fix((0mm, 0mm)) base
  fix((0mm, 60mm)) shoulder
  fix((0mm, 90mm)) top
  whole := line(base, top)
  axis := line(base, shoulder) -> line(shoulder, top)
  r0 := point
  r1 := point
  r2 := point
  fix((30mm, 0mm)) r0
  fix((28mm, 60mm)) r1
  fix((12mm, 90mm)) r2
  rims := line(r0, r1) -> line(r1, r2)
  rail := spline(through: rims, start: whole, end: whole)
}
";

/// A square loft with no guide: three squares in space — 20 at height 0, 12 at 40 moved off the
/// axis by (10, 5), 16 at 60 back on it — and a chain of lines through each corner, the station
/// the plane through three of them (`std.stations.Across`).
const SQUARES: &str = "unit mm
use std
use std.stations

component Quad(ra: path, rb: path, rc: path, rd: path, s: Scalar) {
  st := std.stations.Across(ra, rb, rc, s: s)
  a := point in st.n
  b := point in st.n
  c := point in st.n
  d := point in st.n
  a coincident ra
  b coincident rb
  c coincident rc
  d coincident rd
  f := face(a, b, c, d, -> close)
}

a0 := point
b0 := point
c0 := point
d0 := point
fix((-10mm, -10mm, 0mm)) a0
fix((10mm, -10mm, 0mm)) b0
fix((10mm, 10mm, 0mm)) c0
fix((-10mm, 10mm, 0mm)) d0
a1 := point
b1 := point
c1 := point
d1 := point
fix((4mm, -1mm, 40mm)) a1
fix((16mm, -1mm, 40mm)) b1
fix((16mm, 11mm, 40mm)) c1
fix((4mm, 11mm, 40mm)) d1
a2 := point
b2 := point
c2 := point
d2 := point
fix((-8mm, -8mm, 60mm)) a2
fix((8mm, -8mm, 60mm)) b2
fix((8mm, 8mm, 60mm)) c2
fix((-8mm, 8mm, 60mm)) d2
ra := line(a0, a1) -> line(a1, a2)
rb := line(b0, b1) -> line(b1, b2)
rc := line(c0, c1) -> line(c1, c2)
rd := line(d0, d1) -> line(d1, d2)
";

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

/// The exact solid `name`, its boundary checked.
fn exact(e: &Elaborated, name: &str) -> std::rc::Rc<gcs_core::solid::Exact> {
    let x = e.sketch.exact_solid(index(e, name)).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    x.brep.check(1e-9 * x.brep.size()).unwrap_or_else(|k| panic!("`{name}`: {}", k.msg));
    x
}

/// The volume of the solid `name` off its exact B-rep.
fn volume(e: &Elaborated, name: &str) -> f64 {
    let x = exact(e, name);
    gcs_core::brep::props::volume(&x.brep) / x.mm.powi(3)
}

/// `got` within `rel` of `want`, said with both.
fn near(got: f64, want: f64, rel: f64, what: &str) {
    assert!((got - want).abs() <= rel * want.abs(), "{what}: {got} against {want} (±{rel})");
}

/// `src` with `solids` written below it, solved at DOF 0.
fn with(src: &str, solids: &str) -> Elaborated {
    determined(&format!("{src}{solids}\n"))
}

/// The words the program is refused with.
fn refused(src: &str) -> Vec<String> {
    let (mut prog, errs) = crate::common::parse(src);
    if !errs.is_empty() {
        return errs.iter().map(|e| e.message.clone()).collect();
    }
    gcs_core::modules::link(&mut prog, &mut gcs_core::library::resolve);
    let e = gcs_core::program::elaborate(&prog);
    e.errors().map(|d| d.message.clone()).collect()
}

/// A frustum of height `h` between radii `a` and `b`.
fn frustum(h: f64, a: f64, b: f64) -> f64 {
    PI * h / 3.0 * (a * a + a * b + b * b)
}

/// A square frustum's prismatoid, height `h` between sides `a` and `b` — whatever it leans.
fn prismatoid(h: f64, a: f64, b: f64) -> f64 {
    h / 6.0 * (a * a + (a + b) * (a + b) + b * b)
}

/// `∫₀¹ f` by the five-point Gauss–Legendre rule, exact for a polynomial of degree nine.
fn gauss5(f: impl Fn(f64) -> f64) -> f64 {
    let (a, b) = (0.538_469_310_105_683_1, 0.906_179_845_938_664);
    let (wa, wb) = (0.478_628_670_499_366_5, 0.236_926_885_056_189_1);
    let at = |x: f64| f(0.5 + 0.5 * x);
    0.5 * (128.0 / 225.0 * at(0.0) + wa * (at(-a) + at(a)) + wb * (at(-b) + at(b)))
}

/// The clamped uniform cubic through `pts`, a span between neighbours, its ends held as the
/// conditions say — along `start` and `end` where given, with its second derivative square to
/// them, and else with none — stated on the curve's own derivatives and solved here, apart from
/// the core and its construction of them: its control points.
fn through(pts: &[(f64, f64)], start: Option<(f64, f64)>, end: Option<(f64, f64)>)
    -> Vec<(f64, f64)> {
    let n = pts.len();
    let m = n + 2;
    let spans = (n - 1) as f64;
    // one row a condition over both coordinates' control points, `x` then `y` of each
    let mut rows: Vec<(Vec<f64>, f64)> = Vec::new();
    let row = |w: &[f64], c: usize| {
        let mut r = vec![0.0; 2 * m];
        for (i, wi) in w.iter().enumerate() {
            r[2 * i + c] = *wi;
        }
        r
    };
    // the curve through each corner where its parameter is its index
    for (k, &p) in pts.iter().enumerate() {
        let w = basis(m, k as f64);
        rows.push((row(&w, 0), p.0));
        rows.push((row(&w, 1), p.1));
    }
    // at each end, the weights of the first and second derivatives: the basis is a cubic over
    // the end span, so four samples a quarter apart give them exactly
    for (tangent, at, dir) in [(start, 0.0, 1.0), (end, spans, -1.0)] {
        let h = 0.25;
        let b: Vec<Vec<f64>> = (0..4).map(|i| basis(m, at + dir * h * i as f64)).collect();
        let d1: Vec<f64> = (0..m).map(|i| {
            dir * (-11.0 * b[0][i] + 18.0 * b[1][i] - 9.0 * b[2][i] + 2.0 * b[3][i]) / (6.0 * h)
        }).collect();
        let d2: Vec<f64> = (0..m).map(|i| {
            (2.0 * b[0][i] - 5.0 * b[1][i] + 4.0 * b[2][i] - b[3][i]) / (h * h)
        }).collect();
        match tangent {
            // C'' = 0
            None => {
                rows.push((row(&d2, 0), 0.0));
                rows.push((row(&d2, 1), 0.0));
            }
            // C' × T = 0 and C'' · T = 0
            Some((tx, ty)) => {
                let (a, b) = (row(&d1, 0), row(&d1, 1));
                rows.push((a.iter().zip(&b).map(|(a, b)| a * ty - b * tx).collect(), 0.0));
                let (a, b) = (row(&d2, 0), row(&d2, 1));
                rows.push((a.iter().zip(&b).map(|(a, b)| a * tx + b * ty).collect(), 0.0));
            }
        }
    }
    let x = solve(rows);
    (0..m).map(|i| (x[2 * i], x[2 * i + 1])).collect()
}

/// The clamped uniform cubic B-spline basis of `m` control points at `t` (Cox–de Boor), its
/// spans of unit length.
fn basis(m: usize, t: f64) -> Vec<f64> {
    let spans = m - 3;
    let mut u = vec![0.0; 4];
    u.extend((1..spans).map(|i| i as f64));
    u.extend([spans as f64; 4]);
    let t = t.min(spans as f64 - 1e-12);
    fn n(i: usize, p: usize, t: f64, u: &[f64]) -> f64 {
        if p == 0 {
            return if u[i] <= t && t < u[i + 1] { 1.0 } else { 0.0 };
        }
        let a = if u[i + p] > u[i] {
            (t - u[i]) / (u[i + p] - u[i]) * n(i, p - 1, t, u)
        } else {
            0.0
        };
        let b = if u[i + p + 1] > u[i + 1] {
            (u[i + p + 1] - t) / (u[i + p + 1] - u[i + 1]) * n(i + 1, p - 1, t, u)
        } else {
            0.0
        };
        a + b
    }
    (0..m).map(|i| n(i, 3, t, &u)).collect()
}

/// Gaussian elimination with partial pivoting of a square system.
fn solve(rows: Vec<(Vec<f64>, f64)>) -> Vec<f64> {
    let n = rows.len();
    let mut a: Vec<Vec<f64>> = rows.iter().map(|(r, b)| { let mut r = r.clone(); r.push(*b); r })
        .collect();
    assert_eq!(a[0].len(), n + 1, "a square system");
    for col in 0..n {
        let p = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs())).unwrap();
        a.swap(col, p);
        for i in 0..n {
            if i != col {
                let f = a[i][col] / a[col][col];
                for j in col..=n {
                    a[i][j] -= f * a[col][j];
                }
            }
        }
    }
    (0..n).map(|i| a[i][n] / a[i][i]).collect()
}

/// `spline`'s control points on the page of its plane.
fn ctrl(e: &Elaborated, spline: &str) -> Vec<(f64, f64)> {
    let s = &e.sketch.splines[index(e, spline)];
    s.ctrl.iter().map(|&p| e.sketch.point_xy(p as usize)).collect()
}

/// **A spline through a chain's corners is the cubic through them**: a span a link, each corner at
/// its index, its ends along the tangents written — or, where none is, natural — each held to
/// control points solved apart from the core, from those conditions on the curve's derivatives.
/// The chain may be written after the spline.
#[test]
fn a_spline_through_a_chain_is_the_cubic_through_its_corners() {
    let e = with(BOTTLE, "");
    let got = ctrl(&e, "rail");
    let want = through(&[(30.0, 0.0), (28.0, 60.0), (12.0, 90.0)], Some((0.0, 1.0)),
                       Some((0.0, 1.0)));
    assert_eq!(got.len(), 5, "{got:?}");
    for (g, w) in got.iter().zip(&want) {
        assert!((g.0 - w.0).abs() < 1e-9 && (g.1 - w.1).abs() < 1e-9, "{got:?} against {want:?}");
    }
    let sk = &e.sketch;
    let i = index(&e, "rail");
    for (t, p) in [(0.0, (30.0, 0.0)), (1.0, (28.0, 60.0)), (2.0, (12.0, 90.0))] {
        let f = gcs_core::curve::eval(sk, i, t);
        assert!((f.p.0 - p.0).abs() < 1e-9 && (f.p.1 - p.1).abs() < 1e-9, "at {t}: {:?}", f.p);
    }
    // square to the base and the neck, and bent there no more than the tangent asks
    for t in [0.0, 2.0] {
        let f = gcs_core::curve::eval(sk, i, t);
        assert!(f.d1.0.abs() < 1e-9 * f.d1.1.abs(), "the tangent at {t}: {:?}", f.d1);
        assert!(f.d2.1.abs() < 1e-9 * f.d2.0.abs().max(1.0), "the bend along it at {t}: {:?}",
                f.d2);
    }
    // natural ends, over four corners and over two; the chain written after the spline
    let e = determined("unit mm
use std
use std.stations
in std.front {
  a := point
  b := point
  c := point
  d := point
  fix((0mm, 0mm)) a
  fix((10mm, 10mm)) b
  fix((20mm, 0mm)) c
  fix((30mm, 10mm)) d
  wave := spline(through: zig)
  zig := line(a, b) -> line(b, c) -> line(c, d)
  bar := spline(through: one)
  one := line(a, d) -> line(d, c)
}
");
    let corners = [(0.0, 0.0), (10.0, 10.0), (20.0, 0.0), (30.0, 10.0)];
    for (name, pts) in [("wave", &corners[..]), ("bar", &[(0.0, 0.0), (30.0, 10.0), (20.0, 0.0)])] {
        let (got, want) = (ctrl(&e, name), through(pts, None, None));
        for (g, w) in got.iter().zip(&want) {
            assert!((g.0 - w.0).abs() < 1e-9 && (g.1 - w.1).abs() < 1e-9,
                    "`{name}`: {got:?} against {want:?}");
        }
        let i = index(&e, name);
        for t in [0.0, (pts.len() - 1) as f64] {
            let f = gcs_core::curve::eval(&e.sketch, i, t);
            assert!(f.d2.0.hypot(f.d2.1) < 1e-9, "`{name}` bends at its end {t}: {:?}", f.d2);
        }
    }
}

/// **A ruled loft through several sections is its pieces**: the bottle's rims joined by straight
/// rails, the axis cut at the shoulder, so the family runs both chains in step — a frustum a
/// link, each wall a cone, one name over both, an edge at the shoulder where the two meet at an
/// angle.  The smooth bottle over the same cut axis is the one over the whole: a span a link.
#[test]
fn a_ruled_loft_through_three_sections_is_its_frusta() {
    let e = with(BOTTLE, "
param s: Scalar over (0, 2)
ruled := sweep(Ring(axis, std.y, std.front, rims).f over s)
smooth := sweep(Ring(axis, std.y, std.front, rail).f over s)
param s2: Scalar over (0, 1)
single := sweep(Ring(whole, std.y, std.front, rail, s: s2).f over s2)");
    near(volume(&e, "ruled"), frustum(60.0, 30.0, 28.0) + frustum(30.0, 28.0, 12.0), 1e-9,
         "the ruled bottle");
    near(volume(&e, "smooth"), volume(&e, "single"), 1e-9, "the smooth bottle, cut or whole");
    let x = exact(&e, "ruled");
    let walls: Vec<_> = x.brep.faces.iter().filter(|f| f.name == "ruled.k").collect();
    assert_eq!(walls.len(), 2, "a wall a link: {:?}",
               x.brep.faces.iter().map(|f| &f.name).collect::<Vec<_>>());
    assert!(walls.iter().all(|f| f.surface.kind() == "cone"),
            "{:?}", walls.iter().map(|f| f.surface.kind()).collect::<Vec<_>>());
    // at the shoulder each cone meets the other at an angle: the section is an edge
    let at = brep_point(&x, [28.0, 0.0, 60.0]);
    let normals: Vec<[f64; 3]> = walls.iter().map(|f| normal(&f.surface, at)).collect();
    let cos: f64 = (0..3).map(|k| normals[0][k] * normals[1][k]).sum();
    assert!(cos < 1.0 - 1e-3, "the cones meet tangent at the shoulder: {normals:?}");
}

/// **A loft with no guide is its prismatoids**: three squares in space, the middle one moved
/// off the axis, joined corner to corner by straight rails; its stations the planes through three
/// of them, level wherever the rails are, so each piece is the prismatoid of its two squares.
#[test]
fn a_loft_with_no_guide_is_its_prismatoids() {
    let e = with(SQUARES, "param s: Scalar over (0, 2)\nbody := sweep(Quad(ra, rb, rc, rd).f over s)");
    near(volume(&e, "body"), prismatoid(40.0, 20.0, 12.0) + prismatoid(20.0, 12.0, 16.0), 1e-9,
         "the squares");
    let x = exact(&e, "body");
    // a wall a link for each of the section's four edges, the section between an edge
    let walls = x.brep.faces.iter().filter(|f| f.name != "body.start" && f.name != "body.end");
    assert_eq!(walls.count(), 8, "{:?}", x.brep.faces.iter().map(|f| &f.name).collect::<Vec<_>>());
}

/// **A smooth loft is its quadrature on every span**, and **its ends meet the faces beside them
/// as written**: square to the base, its wall standing straight up from the cap; and at the neck
/// along a cone's generator, the cone a neck turned about the axis beside it — the loft's wall
/// tangent to the cone where they meet.
#[test]
fn a_smooth_loft_meets_the_faces_at_its_ends() {
    let neck = BOTTLE.replace("rail := spline(through: rims, start: whole, end: whole)",
        "rail := spline(through: rims, start: whole, end: slope)
  r3 := point
  t3 := point
  fix((8mm, 110mm)) r3
  fix((0mm, 110mm)) t3
  slope := line(r2, r3)
  lid := line(r3, t3)
  shaft := line(t3, top)
  seat := line(top, r2)
  neck := solid(face(slope, lid, shaft, seat), about: whole)");
    let e = with(&neck, "param s: Scalar over (0, 1)\nbottle := sweep(Ring(whole, std.y, std.front, rail).f over s)");
    // π∫ x² dy over each span of the rail, five Gauss–Legendre points exact for degree 8
    let (sk, i) = (&e.sketch, index(&e, "rail"));
    let want: f64 = (0..2).map(|span| gauss5(|t| {
        let f = gcs_core::curve::eval(sk, i, span as f64 + t);
        PI * f.p.0.powi(2) * f.d1.1
    })).sum();
    near(volume(&e, "bottle"), want, 1e-9, "the bottle");
    let x = exact(&e, "bottle");
    let face = |name: &str| &x.brep.faces.iter().find(|f| f.name == name)
        .unwrap_or_else(|| panic!("no `{name}`")).surface;
    // at the base, square to the cap
    let base = brep_point(&x, [30.0, 0.0, 0.0]);
    let (wall, cap) = (normal(face("bottle.k"), base), normal(face("bottle.start"), base));
    let dot: f64 = (0..3).map(|k| wall[k] * cap[k]).sum();
    assert!(dot.abs() < 1e-9, "the wall at the base against the cap: {wall:?}, {cap:?}");
    // at the neck, tangent to the cone
    let n = exact(&e, "neck");
    let top = brep_point(&x, [12.0, 0.0, 90.0]);
    let names: Vec<&String> = n.brep.faces.iter().map(|f| &f.name).collect();
    let cone = n.brep.faces.iter().find(|f| f.name == "neck.slope")
        .unwrap_or_else(|| panic!("no `neck.slope` in {names:?}"));
    let a = normal(face("bottle.k"), top);
    let b = normal(&cone.surface, brep_point(&n, [12.0, 0.0, 90.0]));
    let cross = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    // the wall's normal by its sheet's rate along `s`, the derivative of the polynomials through
    // its stations, which carries their 1e-10 to a few parts in 10⁹
    assert!(cross.iter().map(|c| c * c).sum::<f64>().sqrt() < 1e-7,
            "the wall at the neck against the cone: {a:?}, {b:?}");
}

/// **A smooth loft with no guide is its quadrature**: a triangle whose corners ride three splines
/// through their places at 0, 30 and 60, each in the plane through the axis it stands in, its
/// stations the planes through the three at `s` (`std.stations.Across`) — level, the three heights
/// alike at every `t` — and the triangle `r²` wherever the rails stand `r` out, so V = ∫ r² dz over
/// the rail, five Gauss–Legendre points a span.
#[test]
fn a_smooth_loft_with_no_guide_is_its_quadrature() {
    let rail = |name: &str, plane: &str, x: f64| format!("in std.{plane} {{
  {name}0 := point
  {name}1 := point
  {name}2 := point
  fix(({}mm, 0mm)) {name}0
  fix(({}mm, 30mm)) {name}1
  fix(({}mm, 60mm)) {name}2
  {name}c := line({name}0, {name}1) -> line({name}1, {name}2)
  r{name} := spline(through: {name}c)
}}
", 10.0 * x, 14.0 * x, 8.0 * x);
    let src = format!("unit mm
use std
use std.stations

component Tri(ra: path, rb: path, rc: path, side: plane, front: plane, s: Scalar) {{
  st := std.stations.Across(ra, rb, rc, s: s)
  a := point in st.n, side
  b := point in st.n, front
  c := point in st.n, front
  a coincident ra
  b coincident rb
  c coincident rc
  f := face(a, b, c, -> close)
}}
{}{}{}
param s: Scalar over (0, 2)
body := sweep(Tri(ra, rb, rc, std.side, std.front).f over s)
", rail("a", "side", 1.0), rail("b", "front", -1.0), rail("c", "front", 1.0));
    let e = determined(&src);
    let (sk, i) = (&e.sketch, index(&e, "ra"));
    let want: f64 = (0..2).map(|span| gauss5(|t| {
        let f = gcs_core::curve::eval(sk, i, span as f64 + t);
        f.p.0.powi(2) * f.d1.1
    })).sum();
    near(volume(&e, "body"), want, 1e-9, "the triangle");
}

/// A point in model units where the exact solid `x` has it.
fn brep_point(x: &gcs_core::solid::Exact, p: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|k| (p[k] - x.origin[k]) * x.mm)
}

/// The unit normal of `s` at its point nearest `p`.
fn normal(s: &Surface, p: [f64; 3]) -> [f64; 3] {
    s.normal(s.inverse(p)).expect("a normal there")
}

/// **What cannot be said is said where it is written**: chains run in step of as many links, a
/// spline through an open chain of lines, with no control points of its own, and its ends only
/// beside `through:`.
#[test]
fn what_cannot_be_lofted_is_refused_where_it_is_written() {
    let src = format!("{BOTTLE}
in std.front {{
  r4 := point
  fix((10mm, 100mm)) r4
  long := line(r0, r1) -> line(r1, r2) -> line(r2, r4)
  ring := line(r1, r2) -> line(r2, r4) -> line(r4, r1) -> close
}}
");
    let bad = "param s: Scalar over (0, 2)\nbad := sweep(Ring(axis, std.y, std.front, long).f over s)";
    let said = refused(&format!("{src}{bad}"));
    assert!(said.iter().any(|m| m.contains("runs the chains it is given in step")
        && m.contains("`long` has 3 links")), "{said:?}");
    let rail = "rail := spline(through: rims,";
    let said = refused(&src.replace(rail, "rail := spline(through: ring,"));
    assert!(said.iter().any(|m| m.contains("closed chain")), "{said:?}");
    let said = refused(&src.replace(rail, "rail := spline(through: r0,"));
    assert!(said.iter().any(|m| m.contains("is not one")), "{said:?}");
    let said = refused(&src.replace("rail := spline(through: rims, start: whole, end: whole)",
                                    "rail := spline(r0, r1, through: rims)"));
    assert!(said.iter().any(|m| m.contains("not both")), "{said:?}");
    let said = refused(&src.replace("rail := spline(through: rims, start: whole, end: whole)",
                                    "rail := spline(r0, r1, r2, r4, start: whole)"));
    assert!(said.iter().any(|m| m.contains("`start:` and `end:` hold the ends")), "{said:?}");
}

/// **It prints as it is written**: the spline through a chain keeps its words in the source a
/// program is printed back to.
#[test]
fn a_spline_through_a_chain_prints_as_written() {
    let (prog, errs) = crate::common::parse(BOTTLE);
    assert!(errs.is_empty(), "{errs:?}");
    let rail = prog.components.iter().flat_map(|c| &c.body).find(|st| matches!(&st.kind,
        gcs_core::syntax::StmtKind::Decl(d) if d.name.key().text == "rail")).expect("the rail");
    let mut text = String::new();
    gcs_core::syntax::write_stmt_to(&mut text, &rail.kind).unwrap();
    assert!(text.contains("rail := spline(through: rims, start: whole, end: whole)"), "{text}");
}
