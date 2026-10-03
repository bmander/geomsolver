//! **Generation by motion, in the plane** (issue #61, proposal 2): a profile *cut* by a tool as
//! the tool and the blank move together is the envelope of the tool, and in the plane the
//! envelope condition `n · v = 0` is a statement about geometry, not a derivative.  Every
//! relative velocity of two bodies in the plane is a turn about their instant centre, so the
//! tool's normal at the point it is cutting passes through that centre (the law of gearing,
//! Willis): the cut point is where the normal from the instant centre meets the tool.
//!
//! So a generated profile is a **trace** — a component whose statements place the tool at roll
//! `u` and the cut point on it with its normal through the instant centre — and every contact
//! a trace takes, a generated profile takes: a point on it, a line tangent to it, a circle
//! osculating it (its curvature is the block's own `C''`, `locus::higher_orders`).  Nothing here
//! is a new construct; the gear's frame is the one the trace is written in.
//!
//! The rack and pinion: in the gear's frame the rack's pitch line rolls without slipping on the
//! pitch circle, and the instant centre is the pitch point where they touch.  A straight rack
//! flank inclined at the pressure angle `α` cuts the involute of the base circle `rp·cos α` —
//! checked here against that closed form, which the document never states.

use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};

use crate::common::build;

/// The rack's flank, carried round the pitch circle `pitch` by the roll `u` (a bearing from the
/// datum, in degrees): `P` is the pitch point, the instant centre; `tl` the rack's pitch line,
/// tangent there; `T` the flank's foot on the pitch line, `s0` along it at no roll and rolled
/// back by the arc `rp·u`; `fl` the flank, at the pressure angle from the radius; and `p`, the
/// cut point, the foot of the normal from `P` on the flank.  The normal is a line of its own
/// length (`P` to `N`) so the block stays regular where the flank passes through `P`.
pub const RACK: &str = "\
component RackFlank(pitch: circle, datum: line, alpha: Angle, s0: Length, u: Angle) {
  P := point hint(x: pitch.center.x + pitch.r * cos(u), y: pitch.center.y + pitch.r * sin(u))
  Q := point hint(x: pitch.center.x + pitch.r * cos(u) - 10 * sin(u), \
                  y: pitch.center.y + pitch.r * sin(u) + 10 * cos(u))
  T := point hint(x: pitch.center.x + pitch.r * cos(u) - (s0 - pitch.r * u * pi / 180) * sin(u), \
                  y: pitch.center.y + pitch.r * sin(u) + (s0 - pitch.r * u * pi / 180) * cos(u))
  q := point hint(x: pitch.center.x + pitch.r * cos(u) - (s0 - pitch.r * u * pi / 180) * sin(u) + 10 * cos(u + alpha), \
                  y: pitch.center.y + pitch.r * sin(u) + (s0 - pitch.r * u * pi / 180) * cos(u) + 10 * sin(u + alpha))
  N := point hint(x: pitch.center.x + pitch.r * cos(u) - 10 * sin(u + alpha), \
                  y: pitch.center.y + pitch.r * sin(u) + 10 * cos(u + alpha))
  p := point hint(x: pitch.center.x + pitch.r * cos(u), y: pitch.center.y + pitch.r * sin(u))
  rad := line(pitch.center, P)
  tl := line(P, Q)
  fl := line(T, q)
  nl := line(P, N)
  P on pitch                                  // the pitch point, on the pitch circle...
  datum angle(u) rad                          // ...at the roll
  P distance(10) Q
  rad perpendicular tl                        // the rack's pitch line, tangent there
  T on tl
  T distance(s0 - pitch.r * u * pi / 180) rad // rolled without slipping
  T distance(10) q
  rad angle(alpha) fl                         // the flank, at the pressure angle
  P distance(10) N
  nl perpendicular fl                         // the normal from the instant centre...
  p on nl
  p on fl                                     // ...meets the flank where it cuts
}
o := point
x := point
datum := line(o, x)
pitch := circle(center: o) hint(r: 30)
radius(30) pitch
fix(x == 0, y == 0) o
fix(x == 1, y == 0) x
";

fn flank(src_tail: &str) -> Elaborated {
    build(&format!("{RACK}flank := RackFlank(pitch, datum, alpha: 20, s0: 5).p over u in (-25, 25)\n{src_tail}"))
}

/// **A straight rack cuts an involute.**  At every roll the line from the instant centre to the
/// cut point is tangent to the base circle `rp·cos α` — which is what an involute's normal is —
/// and the cut point's tangent is square to that line.
#[test]
fn a_rack_cuts_the_involute_of_its_base_circle() {
    let e = flank("");
    let rb = 30.0 * 20f64.to_radians().cos();
    for k in 0..=10 {
        let u = -25.0 + 5.0 * k as f64;
        let (px, py) = e.sketch.curve_point(0, u);
        let (cx, cy) = (30.0 * u.to_radians().cos(), 30.0 * u.to_radians().sin());
        // the normal through the pitch point, its distance from the centre
        let (nx, ny) = (px - cx, py - cy);
        let len = nx.hypot(ny);
        if len < 1e-6 {
            continue; // the cut point is the pitch point: the normal has no direction here
        }
        let off = (cx * ny - cy * nx).abs() / len;
        assert!((off - rb).abs() < 1e-9, "at roll {u}: the line of action is {off} from the centre, not {rb}");
        // and the profile there runs square to it
        let h = 1e-4;
        let (a, b) = (e.sketch.curve_point(0, u - h), e.sketch.curve_point(0, u + h));
        let (tx, ty) = (b.0 - a.0, b.1 - a.1);
        assert!((tx * nx + ty * ny).abs() < 1e-6 * len * tx.hypot(ty), "at roll {u}: tangent not square");
    }
}

/// **A generated profile takes every contact a trace takes.**  A circle osculating the cut
/// flank solves onto the involute's centre of curvature — the point where the line of action
/// touches the base circle — and its radius is the length of that line: the base-circle
/// construction, which nothing in the document states.
#[test]
fn a_circle_osculates_the_cut_flank() {
    let mut e = flank(
        "k := point hint(x: 25, y: 5)\nosc := circle(center: k) hint(r: 8)\n\
         flank curvature osc hint(t: 10)\no distance(26, along: x) k\n",
    );
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    crate::common::fd_jacobian(&e.sketch, 1e-3);
    let rb = 30.0 * 20f64.to_radians().cos();
    let k = e.sketch.point_xy(e.map.ent_named("k").unwrap().i());
    let osc = e.map.ent_named("osc").unwrap();
    let rho = e.sketch.params[e.sketch.circles[osc.i()].radius as usize].value;
    // the centre of curvature is on the base circle...
    assert!((k.0.hypot(k.1) - rb).abs() < 1e-6, "centre {k:?} is {} from o, not {rb}", k.0.hypot(k.1));
    // ...and the radius is the involute's: the distance along the tangent from where the
    // curve point's normal touches the base circle, `sqrt(|p|² − rb²)`
    let c = e.sketch.constraints.iter().find(|c| !c.aux_params().is_empty()).unwrap();
    let u = e.sketch.params[c.aux_params()[0] as usize].value;
    let (px, py) = e.sketch.curve_point(0, u);
    let want = (px * px + py * py - rb * rb).sqrt();
    assert!((rho - want).abs() < 1e-6, "radius {rho}, the involute's {want}");
}

/// The rack's tip round, carried round the pitch circle as the flank is: `C` its centre, `ht`
/// below the pitch line (toward the gear's centre) and `st` along it, rolled back by `rp·u`; and
/// `p` the cut point — on the round where the normal from the instant centre `P` meets it, which
/// for a circle is the line from `P` through its centre.  The fillet this cuts is the envelope of
/// the tip round, the trochoid's offset, which has no closed form worth writing.
pub const TIP: &str = "\
component RackTip(pitch: circle, datum: line, st: Length, ht: Length, rf: Length, u: Angle) {
  P := point hint(x: pitch.center.x + pitch.r * cos(u), y: pitch.center.y + pitch.r * sin(u))
  Q := point hint(x: pitch.center.x + pitch.r * cos(u) - 10 * sin(u), \
                  y: pitch.center.y + pitch.r * sin(u) + 10 * cos(u))
  C := point hint(x: pitch.center.x + (pitch.r - ht) * cos(u) - (st - pitch.r * u * pi / 180) * sin(u), \
                  y: pitch.center.y + (pitch.r - ht) * sin(u) + (st - pitch.r * u * pi / 180) * cos(u))
  p := point hint(x: pitch.center.x + (pitch.r - ht - rf) * cos(u) - (st - pitch.r * u * pi / 180) * sin(u), \
                  y: pitch.center.y + (pitch.r - ht - rf) * sin(u) + (st - pitch.r * u * pi / 180) * cos(u))
  rad := line(pitch.center, P)
  tl := line(P, Q)
  round := circle(center: C) hint(r: rf)
  cut := line(P, C)
  P on pitch
  datum angle(u) rad
  P distance(10) Q
  rad perpendicular tl
  C distance(st - pitch.r * u * pi / 180) rad // along the pitch line, rolled without slipping
  C distance(ht) tl                           // and below it: the left of P to Q is the centre
  radius(rf) round
  p on round
  p on cut                                    // the normal from the instant centre
}
";

/// **The tip round cuts the fillet, and the fillet is its envelope.**  At every roll the cut
/// curve touches the tool where they meet: it runs square to the round's radius there.  That is
/// what an envelope is, and the block states only that the normal passes through the instant
/// centre — the law of gearing is the theorem connecting the two.
#[test]
fn the_tip_round_cuts_a_fillet_tangent_to_it() {
    let e = build(&format!(
        "{RACK}{TIP}fillet := RackTip(pitch, datum, st: 9, ht: 7.5, rf: 1.5).p over u in (-30, 30)\n"
    ));
    let i = e.sketch.curves.len() - 1;
    for k in 0..=12 {
        let u = -30.0 + 5.0 * k as f64;
        let ur = u.to_radians();
        // the round's centre at this roll, worked out here from the rack's own numbers
        let (along, depth) = (9.0 - 30.0 * ur, 30.0 - 7.5);
        let c = (depth * ur.cos() - along * ur.sin(), depth * ur.sin() + along * ur.cos());
        let p = e.sketch.curve_point(i, u);
        let (rx, ry) = (p.0 - c.0, p.1 - c.1);
        assert!((rx.hypot(ry) - 1.5).abs() < 1e-9, "at roll {u}: the cut point is off the round");
        let h = 1e-4;
        let (a, b) = (e.sketch.curve_point(i, u - h), e.sketch.curve_point(i, u + h));
        let (tx, ty) = (b.0 - a.0, b.1 - a.1);
        assert!(
            (tx * rx + ty * ry).abs() < 1e-6 * tx.hypot(ty) * 1.5,
            "at roll {u}: the fillet crosses the round instead of touching it"
        );
    }
}
