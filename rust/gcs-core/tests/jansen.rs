//! Jansen's linkage (`jansen.sv`): the leg assembles on the pose the machine is built to, and
//! the toe's traced stride is the leg itself posed round the crank.
//!
//! The reference is worked out here and not asked of the core: each joint is where two rods of
//! stated length meet, so it is one of the two intersections of two circles, and which one is
//! what the document's `ccw`/`cw` lines state.  The document never writes an intersection down.

use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::examples;
use gcs_core::roots::{bisect, brent};
use gcs_core::solve::{solve, SolveOpts};

use crate::common::{build, radius_by_differences};

// the holy numbers, as the document names them
const A: f64 = 38.0;
const B: f64 = 41.5;
const C: f64 = 39.3;
const D: f64 = 40.1;
const E: f64 = 55.8;
const F: f64 = 39.4;
const G: f64 = 36.7;
const H: f64 = 65.7;
const I: f64 = 49.0;
const J: f64 = 50.0;
const K: f64 = 61.9;
const L: f64 = 7.8;
const M: f64 = 15.0;

type P = (f64, f64);

/// The intersection of the circle of radius `rp` about `p` with that of `rq` about `q`, on the
/// left of p→q for `ccw` and the right otherwise — the reading `ccw(p, q, x)` takes.
fn meet(p: P, rp: f64, q: P, rq: f64, ccw: bool) -> P {
    let (dx, dy) = (q.0 - p.0, q.1 - p.1);
    let d = dx.hypot(dy);
    let x = (rp * rp - rq * rq + d * d) / (2.0 * d);
    let y = (rp * rp - x * x).max(0.0).sqrt() * if ccw { 1.0 } else { -1.0 };
    let (ux, uy) = (dx / d, dy / d);
    (p.0 + x * ux - y * uy, p.1 + x * uy + y * ux)
}

/// The leg at a crank pin at page bearing `deg`, axle at the origin.
fn leg(deg: f64) -> [(&'static str, P); 8] {
    leg_with(deg, L, H)
}

/// `leg` with the pivot `l` below the axle and the heel-to-toe rod `h` long — the two numbers
/// the tangency tests solve for.
fn leg_with(deg: f64, l: f64, h: f64) -> [(&'static str, P); 8] {
    let axle = (0.0, 0.0);
    let pivot = (-A, -l);
    let r = deg.to_radians();
    let pin = (M * r.cos(), M * r.sin());
    let top = meet(pin, J, pivot, B, false);
    let knee = meet(pin, K, pivot, C, true);
    let back = meet(pivot, D, top, E, true);
    let heel = meet(back, F, knee, G, false);
    let toe = meet(heel, h, knee, I, false);
    [("axle", axle), ("pivot", pivot), ("pin", pin), ("top", top), ("back", back),
     ("knee", knee), ("heel", heel), ("toe", toe)]
}

fn close(a: P, b: P, tol: f64) -> bool {
    (a.0 - b.0).abs() < tol && (a.1 - b.1).abs() < tol
}

/// The drawing solves onto the built pose, the crank is its one freedom, and the pose is the
/// one every `ccw`/`cw` line states.
#[test]
fn the_leg_assembles_on_the_stated_pose() {
    let (prog, errs) = gcs_core::syntax::parse(examples::JANSEN);
    assert!(errs.is_empty(), "{errs:?}");
    let mut e = gcs_core::program::elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>());
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (1, State::Under));

    let pin = e.map.ent_named("leg.pin").unwrap();
    let (px, py) = e.sketch.point_xy(pin.i());
    let want = leg(py.atan2(px).to_degrees());
    for (name, at) in want {
        let full = match name {
            "axle" | "pivot" => name.to_string(),
            _ => format!("leg.{name}"),
        };
        let p = e.map.ent_named(&full).unwrap();
        let got = e.sketch.point_xy(p.i());
        assert!(close(got, at, 1e-6), "{name}: drawn {got:?}, built {at:?}");
    }
}

/// **The traced stride is the leg.**  `path` is a locus over a scratch copy of the linkage, and
/// at every crank angle it puts the toe where the drawing's own rods would.
#[test]
fn the_stride_is_the_toe_round_the_crank() {
    let (prog, _) = gcs_core::syntax::parse(examples::JANSEN);
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok());
    assert_eq!(e.sketch.curves.len(), 1);
    // `u` is measured from the pivot-to-axle line, so the pin's page bearing is u + that
    let datum = L.atan2(A).to_degrees();
    for k in 0..24 {
        let u = 15.0 * k as f64;
        let got = e.sketch.curve_point(0, u);
        let want = leg(u + datum)[7].1;
        assert!(close(got, want, 1e-6), "at u = {u}: trace {got:?}, leg {want:?}");
    }
}

/// The toe at a crank pin at page bearing `deg` — `leg_with`'s last joint.
fn toe_with(deg: f64, l: f64, h: f64) -> P {
    leg_with(deg, l, h)[7].1
}

/// The lowest the toe goes within 15° of crank angle `around` (a dip of the stride — near the
/// holy numbers there are two), by Brent's method: the angle and the height.
fn dip(l: f64, h: f64, around: f64) -> (f64, f64) {
    let (y, at) = brent(&|d: f64| toe_with(d, l, h).1, around - 15.0, around + 15.0, 1e-11, 200, |_, _| false);
    (at, y)
}

/// The lowest the toe goes in a whole turn: sampled, then the dip about the lowest sample (the
/// stride's bottom is one smooth minimum).
fn lowest(l: f64, h: f64) -> f64 {
    let y = |d: f64| toe_with(d, l, h).1;
    let k = (0..720).min_by(|&a, &b| y(a as f64 / 2.0).total_cmp(&y(b as f64 / 2.0))).unwrap();
    brent(&y, k as f64 / 2.0 - 0.5, k as f64 / 2.0 + 0.5, 1e-11, 200, |_, _| false).0
}

/// The number in `(lo, hi)` at which `keep` stops holding, by bisection to 1e-12.
fn crossing(lo: f64, hi: f64, keep: impl FnMut(f64) -> bool) -> f64 {
    let (lo, hi) = bisect(lo, hi, |a, b| (a + b) / 2.0, |a, b| b - a > 1e-12, keep);
    (lo + hi) / 2.0
}

/// The number `x` in `(lo, hi)` at which the stride's bottom reaches `ground`.
fn reaching(ground: f64, lo: f64, hi: f64, bottom: impl Fn(f64) -> f64) -> f64 {
    let up = bottom(lo) > ground;
    crossing(lo, hi, |m| (bottom(m) > ground) == up)
}

/// The jansen document with a ground line under the stride, held tangent to it with the contact
/// seeded at `seed`, and `edit` applied to the text first.
fn grounded_at(seed: f64, edit: impl Fn(String) -> String) -> gcs_core::program::Elaborated {
    let doc = edit(examples::JANSEN.to_string())
        + "\ng0 := point hint(x: -60, y: -95)\ng1 := point hint(x: 0, y: -95)\n\
           ground := horizontal line(g0, g1)\ng0 distance(60, along: x) g1\n\
           fix(x == -60, y == -95) g0\n"
        + &format!("path tangent ground hint(t: {seed})\n");
    build(&doc)
}

/// `grounded_at` with the contact seeded near the stride's bottom.
fn grounded(edit: impl Fn(String) -> String) -> gcs_core::program::Elaborated {
    grounded_at(270.0, edit)
}

/// **The stride stands on the ground, and the frame is what moves.**  The ground is fixed and
/// held tangent to the traced toe path, and the pivot's height under the axle — no longer
/// stated — is what solves: the height at which the stride's lowest point just reaches it.
#[test]
fn the_frame_solves_for_the_stride_to_touch_the_ground() {
    let mut e = grounded(|d| d.replace("pivot distance(l, along: y) axle\n", ""));
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (1, State::Under), "the crank is still the one freedom");
    let pivot = e.map.ent_named("pivot").unwrap();
    let got = -e.sketch.point_xy(pivot.i()).1;
    let want = reaching(-95.0, 5.0, 15.0, |l| lowest(l, H));
    assert!((got - want).abs() < 1e-6, "pivot {got} under the axle, the reference {want}");
}

/// **A rod's length is what solves.**  The leg takes the heel-to-toe rod's length as a formal
/// and the drawn leg leaves it unbound, so it is an unknown of the drawing (`leg.h`) and a
/// column of the traced stride: it solves so the stride's lowest point reaches the ground.
#[test]
fn a_rod_solves_for_the_stride_to_touch_the_ground() {
    let mut e = grounded(|d| {
        d.replace(
            "component Leg(axle: point, pivot: point, theta: Angle) {",
            "component Leg(axle: point, pivot: point, theta: Angle, h: Length) {",
        )
        .replace("  h := 65.7    // heel to toe\n", "")
    });
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (1, State::Under), "the crank is still the one freedom");
    let got = e.sketch.params[e.sketch.free_vars["leg.h"] as usize].value;
    let want = reaching(-95.0, 60.0, 75.0, |h| lowest(L, h));
    assert!((got - want).abs() < 1e-6, "the rod solved to {got}, the reference {want}");
    // the drawn leg is the traced one: its toe rod is as long as the trace says
    let (heel, toe) = (e.map.ent_named("leg.heel").unwrap(), e.map.ent_named("leg.toe").unwrap());
    let ((hx, hy), (tx, ty)) = (e.sketch.point_xy(heel.i()), e.sketch.point_xy(toe.i()));
    assert!(((hx - tx).hypot(hy - ty) - got).abs() < 1e-6);
}

/// A number a drawn instance leaves unknown is a column of its curve only as itself: given an
/// expression in another unknown (the enclosing instance's own `k`), the curve says so rather
/// than carry a map no kernel reads.
#[test]
fn a_curve_column_is_an_unknown_number_as_itself() {
    let doc = examples::JANSEN
        .replace(
            "component Leg(axle: point, pivot: point, theta: Angle) {",
            "component Leg(axle: point, pivot: point, theta: Angle, h: Length) {",
        )
        .replace("  h := 65.7    // heel to toe\n", "")
        .replace(
            "leg := Leg(axle, pivot)\n",
            "component Pair(axle: point, pivot: point, k: Length) { leg := Leg(axle, pivot, h: 2 * k) }\n\
             pair := Pair(axle, pivot)\n",
        )
        .replace("path := leg.toe", "path := pair.leg.toe");
    let (prog, errs) = gcs_core::syntax::parse(&doc);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    let said: Vec<String> = e.errors().map(|d| d.message.clone()).collect();
    assert!(said.iter().any(|m| m.contains("only as the number itself")), "{said:?}");
}

/// **The stride is closed, so a contact has no end to stop at.**  `over theta in (0, 360)` runs
/// the crank a whole turn and the toe comes back where it began; the tangency's seed may sit
/// either side of where the interval was written to start, and a solve walking past it wraps
/// round rather than being pinned at the seam (where the drawing was once called a conflict).
#[test]
fn a_contact_on_a_closed_stride_wraps_round_the_seam() {
    let want = reaching(-95.0, 5.0, 15.0, |l| lowest(l, H));
    for seed in [0.0, 45.0, 90.0, 135.0, 225.0, 270.0, 340.0] {
        let mut e = grounded_at(seed, |d| d.replace("pivot distance(l, along: y) axle\n", ""));
        assert!(e.sketch.curve_closed(0), "the stride closes on itself");
        let r = solve(&mut e.sketch, SolveOpts::default());
        assert!(r.success, "seeded at {seed}: {}", r.message);
        let pivot = e.map.ent_named("pivot").unwrap();
        let got = -e.sketch.point_xy(pivot.i()).1;
        assert!((got - want).abs() < 1e-6, "seeded at {seed}: pivot {got}, the reference {want}");
    }
}

/// The stride's radius of curvature at crank angle `at`, by central differences in the angle.
/// A step of 1e-2° keeps it to about 1e-6 of itself.
fn radius_at(l: f64, h: f64, at: f64) -> f64 {
    radius_by_differences(|d| toe_with(d, l, h), at, 1e-2)
}

/// The jansen document with the heel-to-toe rod a formal the drawn leg leaves unknown (`leg.h`),
/// a level ground of unstated height under the stride, and `extra` stated about them.
fn on_level_ground(extra: &str) -> gcs_core::program::Elaborated {
    let doc = examples::JANSEN
        .replace(
            "component Leg(axle: point, pivot: point, theta: Angle) {",
            "component Leg(axle: point, pivot: point, theta: Angle, h: Length) {",
        )
        .replace("  h := 65.7    // heel to toe\n", "")
        + "\ng0 := point hint(x: -60, y: -92)\ng1 := point hint(x: 0, y: -92)\n\
           ground := horizontal line(g0, g1)\ng0 distance(60, along: x) g1\nfix(x == -60) g0\n"
        + extra;
    build(&doc)
}

fn rod(e: &gcs_core::program::Elaborated) -> f64 {
    e.sketch.params[e.sketch.free_vars["leg.h"] as usize].value
}

/// **A flat stride is one that stands on the ground twice.**  Near the holy numbers the toe's
/// path has two shallow dips, and the ground held tangent to it at both — two contacts, seeded
/// one at each — is a line the stride rolls along rather than rocks on.  The toe rod solves so the
/// two dips are level, and lands within a twentieth of Jansen's 65.7: the holy numbers are, very
/// nearly, the bitangent leg.
#[test]
fn the_stride_stands_on_the_ground_twice() {
    let mut e = on_level_ground("path tangent ground hint(t: 25)\npath tangent ground hint(t: 315)\n");
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (1, State::Under), "the crank is still the one freedom");
    // the reference: the rod at which the two dips are level, by bisection
    let want = crossing(65.6, 65.7, |m| dip(L, m, 35.0).1 < dip(L, m, 329.5).1);
    assert!((rod(&e) - want).abs() < 1e-6, "the rod solved to {}, the reference {want}", rod(&e));
    assert!((rod(&e) - H).abs() < 0.05, "and it is the holy number, nearly");
    let g0 = e.sketch.point_xy(e.map.ent_named("g0").unwrap().i());
    assert!((g0.1 - dip(L, want, 35.0).1).abs() < 1e-6, "the ground is where both dips are");
}

/// **How flat, as a statement.**  A circle of radius 150 osculates the stride with the crank at
/// 314.5° — near where the stride bottoms out — so the stride bends exactly that tightly there,
/// and the toe rod solves for it.  The curvature is the traced stride's own `C''`
/// (`locus::higher_orders`): nothing in the document or the core states the coupler curve, and
/// the reference finds the rod by bisection on the leg built from circle intersections.
///
/// The angle is pinned rather than left to the bottom of the stride: a curvature contact and a
/// tangency are two contacts with two parameters, and near the bottom "the osculating circle
/// touches the ground" holds to third order in the distance between them, so tying them through
/// the ground is a degenerate root (it solved 1e-4 of the rod off).
#[test]
fn the_stride_bends_at_a_stated_radius() {
    let mut e = on_level_ground(
        "k := point hint(x: -45, y: 50)\nosc := circle(center: k) hint(r: 140)\n\
         path curvature(t == 314.5) osc\nradius(150) osc\nfix(y == -92) g0\n",
    );
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (1, State::Under), "the crank is still the one freedom");
    // the reference, at the crank's page bearing: `u` is measured from the pivot-to-axle line
    let at = 314.5 + L.atan2(A).to_degrees();
    let want = crossing(66.0, 68.0, |m| radius_at(L, m, at) > 150.0);
    assert!((rod(&e) - want).abs() < 1e-5, "the rod solved to {}, the reference {want}", rod(&e));
    // and the circle's centre is the stride's centre of curvature there
    let toe = toe_with(at, L, rod(&e));
    let k = e.sketch.point_xy(e.map.ent_named("k").unwrap().i());
    assert!(((k.0 - toe.0).hypot(k.1 - toe.1) - 150.0).abs() < 1e-6);
}
