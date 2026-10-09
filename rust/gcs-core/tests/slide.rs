//! **A free curve pressed onto a held line where it chooses** (#149): `rope touches floor`, a
//! corner on the line whose place along it is the curve's problem's — one more unknown, and one
//! more row, no force along the line (frictionless).  Held against closed forms: a rope of an
//! integrand of the point alone pulls with one tension both sides of a frictionless corner, and
//! the line pushes square to itself, so the rope leaves the line at the angle it met it (the law
//! of reflection) and, against a level line, its two arcs are catenaries of one directrix.

use crate::catenary::{bisect, read, samples, solved, ROPE};
use gcs_core::constraints::CKind;
use gcs_core::extremal::{self, shoot, Ends, Lagrangian, Stop};
use gcs_core::model::{EntKind, EntRef};
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::units::Units;
use gcs_core::variational::Extremum;

fn hanging() -> Lagrangian {
    Lagrangian::compile(&[(1.0, "p.y")], false, Units::default()).unwrap()
}

/// The catenary `y = y0 + a cosh((x − x0)/a)` through `p` and `q` of length `len`: `(a, x0, y0)`.
fn catenary_of(p: [f64; 2], q: [f64; 2], len: f64) -> (f64, f64, f64) {
    let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
    let flat = (len * len - dy * dy).sqrt();
    let a = bisect(1e-6, 1e6, |a| 2.0 * a * (dx / (2.0 * a)).sinh() - flat);
    let x0 = 0.5 * (p[0] + q[0]) - a * (dy / len).atanh();
    (a, x0, p[1] - a * ((p[0] - x0) / a).cosh())
}

/// The shape's corner at its one stop: where it is, its place along the curve, and the
/// direction the curve meets it in and leaves it in.
fn corner(l: &Lagrangian, sh: &shoot::Shape) -> ([f64; 2], f64, f64, f64) {
    let u = sh.places[1] / sh.ends.len;
    let (a, b) = (shoot::at(l, sh, u - 1e-12, false).unwrap(), shoot::at(l, sh, u + 1e-12, false).unwrap());
    ([b.z[0], b.z[1]], sh.places[1], a.theta, b.theta)
}

/// The issue's case: a rope 150 long hung from ends level and 100 apart, pressed down onto a
/// level line just below where it would hang.  It rests symmetric about the corner, the corner
/// half way along it, each half the catenary through its ends of half the length — and a minimum.
#[test]
fn a_rope_pressed_onto_a_level_line_rests_symmetric_about_the_corner() {
    let l = hanging();
    let ends = Ends { a: [0.0, 0.0], b: [100.0, 0.0], len: 150.0 };
    let line = Stop::Slide { o: [20.0, -52.0], d: [1.0, 0.0] };
    let sh = shoot::solve(&l, &ends, &[line], None).expect("a shape");
    let (p, s, into, out) = corner(&l, &sh);
    assert!((p[0] - 50.0).abs() < 1e-9 && (p[1] + 52.0).abs() < 1e-9, "the corner at {p:?}");
    assert!((s - 75.0).abs() < 1e-9, "the corner halves the rope: {s}");
    assert!((sh.sigmas[0] - 30.0).abs() < 1e-9, "30 along the line from its start: {}", sh.sigmas[0]);
    assert!((into + out).abs() < 1e-9 && (into - out).abs() > 0.1, "a corner mirrored: {into} {out}");
    let (la, lx, ly) = catenary_of([0.0, 0.0], p, 75.0);
    let (ra, rx, ry) = catenary_of(p, [100.0, 0.0], 75.0);
    let e = (0..=200)
        .map(|i| {
            let z = shoot::at(&l, &sh, i as f64 / 200.0, false).unwrap().z;
            let (a, x0, y0) = if z[0] <= 50.0 { (la, lx, ly) } else { (ra, rx, ry) };
            (z[1] - y0 - a * ((z[0] - x0) / a).cosh()).abs()
        })
        .fold(0.0, f64::max);
    assert!(e < 1e-9, "off the two catenaries by {e}");
    assert_eq!(extremal::verdict(&l, &sh, false), Extremum::Minimum);
}

/// Ends at different heights: the corner is no longer half way, but the frictionless line still
/// pushes only square to itself, so the rope's tension is one both sides and its pull along the
/// line balances — the two arcs are catenaries of **one directrix**, the same `a` and the same
/// `y0`, their corner mirrored about the line's normal.
#[test]
fn the_two_arcs_are_catenaries_of_one_directrix() {
    let l = hanging();
    let ends = Ends { a: [0.0, 0.0], b: [100.0, 15.0], len: 150.0 };
    let line = Stop::Slide { o: [0.0, -47.0], d: [1.0, 0.0] };
    let sh = shoot::solve(&l, &ends, &[line], None).expect("a shape");
    let (p, s, into, out) = corner(&l, &sh);
    assert!((p[1] + 47.0).abs() < 1e-9, "the corner on the line: {p:?}");
    assert!(p[0] > 40.0 && p[0] < 50.0, "nearer the lower end: {p:?}");
    let (la, lx, ly) = catenary_of([0.0, 0.0], p, s);
    let (ra, rx, ry) = catenary_of(p, [100.0, 15.0], 150.0 - s);
    assert!((la - ra).abs() < 1e-7 * la, "one horizontal tension: a {la} and {ra}");
    assert!((ly - ry).abs() < 1e-7 * la, "one directrix: {ly} and {ry}");
    assert!((lx - rx).abs() > 1.0, "two arcs of it, not one: x0 {lx} and {rx}");
    assert!((into + out).abs() < 1e-8, "mirrored about the normal: {into} {out}");
    let e = (0..=200)
        .map(|i| {
            let z = shoot::at(&l, &sh, i as f64 / 200.0, false).unwrap().z;
            let (a, x0, y0) = if z[0] <= p[0] { (la, lx, ly) } else { (ra, rx, ry) };
            (z[1] - y0 - a * ((z[0] - x0) / a).cosh()).abs()
        })
        .fold(0.0, f64::max);
    assert!(e < 1e-9, "off the catenaries by {e}");
    assert_eq!(extremal::verdict(&l, &sh, false), Extremum::Minimum);
}

/// A sloping line: the corner slides along it to where the rope reflects off it, the angle it
/// leaves at the angle it met — `θ_in + θ_out = 2φ`, the line at `φ`.
#[test]
fn on_a_sloping_line_the_rope_leaves_at_the_angle_it_met() {
    let l = hanging();
    let ends = Ends { a: [0.0, 0.0], b: [100.0, 0.0], len: 150.0 };
    let phi = 12f64.to_radians();
    let line = Stop::Slide { o: [0.0, -64.0], d: [phi.cos(), phi.sin()] };
    let sh = shoot::solve(&l, &ends, &[line], None).expect("a shape");
    let (p, _, into, out) = corner(&l, &sh);
    let gap = (p[0] - 0.0) * phi.sin() - (p[1] + 64.0) * phi.cos();
    assert!(gap.abs() < 1e-9, "the corner on the line: {gap}");
    assert!((into + out - 2.0 * phi).abs() < 1e-8, "reflected: {into} + {out} against {}", 2.0 * phi);
    assert!((into - out).abs() > 0.1, "a corner: {into} {out}");
    assert_eq!(extremal::verdict(&l, &sh, false), Extremum::Minimum);
}

/// A line the rope already crosses presses nothing: where it crosses, the corner bears no force,
/// so the rope hangs as it would, through the crossing.
#[test]
fn a_line_the_rope_crosses_presses_nothing() {
    let l = hanging();
    let ends = Ends { a: [0.0, 0.0], b: [100.0, 0.0], len: 150.0 };
    let free = shoot::solve(&l, &ends, &[], None).unwrap();
    let sh = shoot::solve(&l, &ends, &[Stop::Slide { o: [0.0, -40.0], d: [1.0, 0.0] }], None).unwrap();
    let (p, _, into, out) = corner(&l, &sh);
    assert!((p[1] + 40.0).abs() < 1e-9 && (into - out).abs() < 1e-8, "no corner: {p:?} {into} {out}");
    let e = (0..=100)
        .map(|i| {
            let u = i as f64 / 100.0;
            let (a, b) = (shoot::position(&l, &sh, u).unwrap(), shoot::position(&l, &free, u).unwrap());
            (a[0] - b[0]).hypot(a[1] - b[1])
        })
        .fold(0.0, f64::max);
    assert!(e < 1e-9, "the rope moved by {e}");
}

/// The derivatives a drawing reads off the shape — in the ends and the length — against central
/// differences of solves, the corner sliding along its line between them.
#[test]
fn a_touched_shapes_derivatives_are_its_differences() {
    let l = hanging();
    let ends = Ends { a: [0.0, 0.0], b: [100.0, 5.0], len: 150.0 };
    let stops = [Stop::Slide { o: [0.0, -56.0], d: [0.98f64.sqrt(), 0.02f64.sqrt()] }];
    let sh = shoot::solve(&l, &ends, &stops, None).expect("a shape");
    for u in [0.2, 0.7] {
        let at = shoot::at(&l, &sh, u, true).unwrap();
        let o = ends.outer();
        for c in 0..5 {
            let h = 1e-5;
            let z = |sign: f64| {
                let mut oo = o;
                oo[c] += sign * h;
                let s = shoot::solve(&l, &Ends::of(&oo), &stops, Some(&sh)).unwrap();
                shoot::at(&l, &s, u, false).unwrap().z
            };
            let (p, m) = (z(1.0), z(-1.0));
            for i in 0..4 {
                let fd = (p[i] - m[i]) / (2.0 * h);
                let got = at.dz[i * 6 + 1 + c];
                assert!((fd - got).abs() <= 1e-5 * (1.0 + fd.abs()), "u {u} z{i} / o{c}: {got} against {fd}");
            }
        }
    }
}

/* -- the language ------------------------------------------------------------------------- */

const RAIL: &str = "  length(150) rope\n  r0 := point\n  r1 := point\n  fix((20, -52)) r0\n  fix((90, -52)) r1\n  \
                    rail := line(r0, r1)\n  rope touches rail\n";

fn touched() -> String {
    ROPE.replace("  length(150) rope\n", RAIL)
}

fn dof(e: &mut Elaborated) -> (i64, Vec<&'static str>) {
    let d = gcs_core::diagnose::diagnose(&mut e.sketch, gcs_core::diagnose::DiagnoseOptions::default());
    (d.dof, d.extrema.iter().map(|(_, v)| v.name()).collect())
}

/// `rope touches rail`: the rope is pressed onto the held rail, its corner where it chooses — the
/// middle — and the statement compiles no row of the drawing's: the rope's problem has it.
#[test]
fn rope_touches_rail_drapes_it_on_the_rail() {
    let mut e = solved(&touched());
    let touch = e.sketch.constraints.iter().find(|c| c.kind == CKind::CurveTouchesLine).expect("a touch");
    assert_eq!(touch.rows_in(&e.sketch), 0);
    assert_eq!(gcs_core::io::describe_with(touch, &|r| e.map.name_of(r).cloned()), "rope touches rail");
    let (_, sh) = e.sketch.curve_shape(0).expect("a shape");
    assert!((sh.places[1] - 75.0).abs() < 1e-9, "the corner half way: {}", sh.places[1]);
    let low = samples(&e.sketch, 0).into_iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    assert!((low + 52.0).abs() < 1e-9, "the rope's lowest point on the rail: {low}");
    assert_eq!(dof(&mut e), (0, vec!["minimum"]));
}

/// With an end free, the drawing's freedoms are the end's alone, and dragging it slides the
/// corner along the rail: every frame the rope still reflects off it.
#[test]
fn dragging_an_end_slides_the_corner_along_the_rail() {
    let src = touched()
        .replace("  fix((100, 0)) b\n", "")
        .replace("  b := point\n", "  b := point hint((100, 0))\n");
    let mut e = solved(&src);
    assert_eq!(dof(&mut e).0, 2);
    let b = (0..e.sketch.points.len())
        .find(|&i| {
            let (x, y) = e.sketch.point_xy(i);
            (x - 100.0).abs() < 1e-9 && y.abs() < 1e-9
        })
        .unwrap();
    let dogleg = gcs_core::newton::Method::DogLeg;
    let mut d = gcs_core::solve::Drag::new(&mut e.sketch, b, 100.0, 0.0, dogleg, 1.0, Vec::new(), 0.05);
    let mut xs = Vec::new();
    for k in 1..=6 {
        let r = d.move_to(&mut e.sketch, 100.0 - 2.0 * k as f64, 2.0 * k as f64);
        assert!(r.success, "frame {k}: {}", r.message);
        let l = hanging();
        let (_, sh) = e.sketch.curve_shape(0).unwrap();
        let (p, _, into, out) = corner(&l, &sh);
        assert!((p[1] + 52.0).abs() < 1e-9, "frame {k}: the corner on the rail, {p:?}");
        assert!((into + out).abs() < 1e-8, "frame {k}: reflected, {into} {out}");
        xs.push(p[0]);
    }
    d.end(&mut e.sketch);
    assert!(xs.windows(2).all(|w| w[1] < w[0]), "the corner slides after the end: {xs:?}");
}

/// A copy (a paste, a deletion's rebuild) carries the touch, and the rope drapes the same.
#[test]
fn a_copied_touch_drapes_again() {
    let e = solved(&touched());
    let all: Vec<EntRef> = (0..e.sketch.points.len())
        .map(EntRef::point)
        .chain((0..e.sketch.lines.len()).map(|i| EntRef::new(EntKind::Line, i)))
        .chain((0..e.sketch.curves.len()).map(|i| EntRef::new(EntKind::Curve, i)))
        .collect();
    let mut copied = gcs_core::io::copy(&e.sketch, &all);
    let r = solve(&mut copied, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    for (p, q) in samples(&copied, 0).into_iter().zip(samples(&e.sketch, 0)) {
        assert!((p.0 - q.0).hypot(p.1 - q.1) < 1e-9, "{p:?} against {q:?}");
    }
}

/// What a touch is not: a rail the drawing does not hold (the rope would push it anywhere), and
/// a curve that is not free (its shape is its formula's, not pressed into a corner).  And the
/// tangency a held line refuses says what to write instead.
#[test]
fn a_touch_is_refused_where_it_cannot_press() {
    let loose = touched().replace("  fix((90, -52)) r1\n", "");
    let (_, d) = read(&loose);
    assert!(d.iter().any(|m| m.starts_with("E040") && m.contains("hold the line")), "{d:?}");
    let src = "use std\n\
               component Wheel(c: point, u: Angle) {\n  p := point(x: c.x + 50 * cos(u), y: c.y + 50 * sin(u))\n}\n\
               in std.front {\n  o := point\n  fix((0, 0)) o\n  r0 := point\n  r1 := point\n  \
               fix((0, 10)) r0\n  fix((10, 10)) r1\n  rail := line(r0, r1)\n}\n\
               k := Wheel(o).p over u in (0, 90)\nin std.front {\n  k touches rail\n}\n";
    let (_, d) = read(src);
    assert!(d.iter().any(|m| m.starts_with("E040") && m.contains("free curve")), "{d:?}");
    let tangent = touched().replace("rope touches rail", "rope tangent rail");
    let (_, d) = read(&tangent);
    assert!(d.iter().any(|m| m.starts_with("E040") && m.contains("touches")), "{d:?}");
}

/// The length nothing holds: the touch holds none, so the rope's length is the energy's — and a
/// hanging rope has none it is stationary in, said so as it is without the touch.
#[test]
fn a_touch_holds_no_length() {
    let src = touched().replace("  length(150) rope\n", "");
    let (e, d) = read(&src);
    assert!(e.ok(), "{d:?}");
    assert!(d.iter().any(|m| m.starts_with("W114")), "{d:?}");
}


/// `ring.sv`: a rope 160 long held to a sloping rod below where it would hang — the corner on
/// the rod, the rope reflected off it, a minimum, and the drawn rope bending at the corner itself
/// rather than cutting across it.
#[test]
fn the_library_ring_reflects_off_its_rod() {
    let mut sk = gcs_core::examples::example("ring").expect("the ring");
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let (l, sh) = sk.curve_shape(0).expect("a shape");
    let (p, _, into, out) = corner(&l, &sh);
    let phi = (16.0f64 / 100.0).atan();
    assert!((p[1] + 68.0 - p[0] * 0.16).abs() < 1e-9, "the corner on the rod: {p:?}");
    assert!((into + out - 2.0 * phi).abs() < 1e-8, "reflected: {into} + {out} against {}", 2.0 * phi);
    assert!((into - out).abs() > 0.1, "a corner: {into} {out}");
    assert!(sk.curve_polyline(0).iter().any(|q| (q.0 - p[0]).hypot(q.1 - p[1]) < 1e-8), "the corner drawn");
    let d = gcs_core::diagnose::diagnose(&mut sk, gcs_core::diagnose::DiagnoseOptions::default());
    assert_eq!(d.extrema.iter().map(|(_, v)| v.name()).collect::<Vec<_>>(), ["minimum"]);
}
