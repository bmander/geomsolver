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

/* -- the construct: `envelope(tool, under: motion)` in the plane (§6.15.1) ------------------- */

/// A rack and a blank as motions (over `RACK`'s `o`): the blank turns about `o`, the rack slides along a line
/// through the pitch point at the pitch circle's speed (`2π·30` a turn), and `rel` is the rack as
/// the blank sees it.  The flank is drawn where it stands at roll 0: through `(30, 5)` on the
/// pitch line, at 20° from the radius.
const RACK_MOTION: &str = "\
rk0 := point hint(x: 30, y: 0)
rk1 := point hint(x: 30, y: 10)
fix(x == 30, y == 0) rk0
fix(x == 30, y == 10) rk1
slide := line(rk0, rk1)
blank := motion(about: o, ratio: 1)
rack := motion(along: slide, advance: 2 * pi * 30)
rel := motion(of: rack, relative_to: blank)
f0 := point hint(x: 30, y: 5)
f1 := point hint(x: 40, y: 8)
flank := line(f0, f1)
fix(x == 30, y == 5) f0
fix(x == 39.396926207859084, y == 8.420201433256687) f1
";

/// **The construct is the trace.**  `envelope(flank, under: rel)` — a straight flank carried by
/// the rack relative to the blank — is the curve `RackFlank` states by hand: point for point,
/// at the roll the blank has turned by, the same cut.  (The trace's roll is the pitch point's
/// bearing in the blank, which runs the other way: `u = −t`.)
#[test]
fn an_envelope_of_a_rack_flank_is_the_traced_cut() {
    let src = format!(
        "{RACK}{RACK_MOTION}cut := envelope(flank, under: rel, from: -25deg, to: 25deg)\n\
         traced := RackFlank(pitch, datum, alpha: 20, s0: 5).p over u in (-25, 25)\n"
    );
    let e = build(&src);
    let cut = e.map.ent_named("cut").unwrap();
    let traced = e.map.ent_named("traced").unwrap();
    assert_eq!(cut.kind, gcs_core::model::EntKind::Curve);
    for k in 0..=10 {
        let t = -25.0 + 5.0 * k as f64;
        let a = e.sketch.curve_point(cut.i(), t);
        let b = e.sketch.curve_point(traced.i(), -t);
        assert!((a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9, "at roll {t}: {a:?} against {b:?}");
    }
}

/// **An envelope takes every contact a curve takes**, exact in its residual: a circle solves onto
/// the cut flank's osculating circle — centre on the base circle, radius the involute's — with
/// the Jacobian (gradients along the columns by difference) good enough to reach it.
#[test]
fn a_circle_osculates_an_envelope() {
    let mut e = build(&format!(
        "{RACK}{RACK_MOTION}cut := envelope(flank, under: rel, from: -25deg, to: 25deg)\n\
         k := point hint(x: 25, y: -5)\nosc := circle(center: k) hint(r: 8)\n\
         cut curvature osc hint(t: -10)\no distance(26, along: x) k\n"
    ));
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    crate::common::fd_jacobian(&e.sketch, 1e-3);
    let rb = 30.0 * 20f64.to_radians().cos();
    let k = e.sketch.point_xy(e.map.ent_named("k").unwrap().i());
    let osc = e.map.ent_named("osc").unwrap();
    let rho = e.sketch.params[e.sketch.circles[osc.i()].radius as usize].value;
    assert!((k.0.hypot(k.1) - rb).abs() < 1e-6, "centre {k:?} is {} from o, not {rb}", k.0.hypot(k.1));
    let c = e.sketch.constraints.iter().find(|c| !c.aux_params().is_empty()).unwrap();
    let t = e.sketch.params[c.aux_params()[0] as usize].value;
    let cut = e.map.ent_named("cut").unwrap();
    let (px, py) = e.sketch.curve_point(cut.i(), t);
    let want = (px * px + py * py - rb * rb).sqrt();
    assert!((rho - want).abs() < 1e-6, "radius {rho}, the involute's {want}");
}

/// **The tool is a column of what it cuts.**  The flank's angle is left to the solve — its far
/// end free to swing round its foot — and a point the cut must pass through decides it: the
/// flank solves to the pressure angle whose involute reaches that point, and the line of action
/// is then tangent to that angle's base circle.
#[test]
fn a_contact_on_the_cut_solves_the_tool() {
    let src = format!("{RACK}{RACK_MOTION}")
        .replace("fix(x == 39.396926207859084, y == 8.420201433256687) f1\n", "f0 distance(10) f1\n")
        + "cut := envelope(flank, under: rel, from: -25deg, to: 25deg)\n\
           g := point hint(x: 31, y: -3)\nfix(x == 31.5, y == -3) g\ng on cut hint(t: 5)\n";
    let mut e = build(&src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    crate::common::fd_jacobian(&e.sketch, 1e-3);
    let at = |n: &str| e.sketch.point_xy(e.map.ent_named(n).unwrap().i());
    let (f0, f1) = (at("f0"), at("f1"));
    let alpha = (f1.1 - f0.1).atan2(f1.0 - f0.0);
    // the cut point at the contact: its normal through the pitch point touches rp·cos α
    let c = e.sketch.constraints.iter().find(|c| !c.aux_params().is_empty()).unwrap();
    let t = e.sketch.params[c.aux_params()[0] as usize].value;
    let (px, py) = e.sketch.curve_point(e.map.ent_named("cut").unwrap().i(), t);
    assert!((px - 31.5).abs() < 1e-6 && (py + 3.0).abs() < 1e-6);
    let tr = (-t).to_radians();
    let (cx, cy) = (30.0 * tr.cos(), 30.0 * tr.sin());
    let (nx, ny) = (px - cx, py - cy);
    let off = (cx * ny - cy * nx).abs() / nx.hypot(ny);
    // the flank is a line: which way its far end swung does not matter, only its slope
    assert!((off - 30.0 * alpha.cos().abs()).abs() < 1e-6, "line of action {off}, rp·cos α {}", 30.0 * alpha.cos());
    assert!((alpha.cos().abs() - 20f64.to_radians().cos()).abs() > 1e-3, "the flank moved off 20°");
}

/// **A roller cuts its cam.**  A roller of radius 4 on a follower sliding out along a line
/// through the cam's centre at 20 a turn, the cam turning under it: in the cam's frame the
/// roller's centre runs on an Archimedean spiral, and the cam profile — the envelope of the
/// roller, its near side — stays exactly the roller's radius from that spiral, touching the
/// roller at every roll.
#[test]
fn a_roller_cuts_its_cam() {
    let src = "\
o := point
fix(x == 0, y == 0) o
s0 := point hint(x: 0, y: 0)
s1 := point hint(x: 10, y: 0)
fix(x == 0, y == 0) s0
fix(x == 10, y == 0) s1
path := line(s0, s1)
cam := motion(about: o, ratio: 1)
follower := motion(along: path, advance: 20)
rel := motion(of: follower, relative_to: cam)
rc := point hint(x: 30, y: 0)
fix(x == 30, y == 0) rc
roller := circle(center: rc) hint(r: 4)
radius(4) roller
pitch := envelope(rc, under: rel, from: 0deg, to: 270deg)
profile := envelope(roller, under: rel, from: 0deg, to: 270deg, side: near)
";
    let e = build(src);
    let (pitch, profile) = (e.map.ent_named("pitch").unwrap(), e.map.ent_named("profile").unwrap());
    for k in 0..=9 {
        let t = 30.0 * k as f64;
        // the roller's centre, worked out here: out 30 + 20·t/360 along the path, turned back by t
        let rho = 30.0 + 20.0 * t / 360.0;
        let tr = (-t).to_radians();
        let c = (rho * tr.cos(), rho * tr.sin());
        let q = e.sketch.curve_point(pitch.i(), t);
        assert!((q.0 - c.0).abs() < 1e-9 && (q.1 - c.1).abs() < 1e-9, "pitch at {t}: {q:?} against {c:?}");
        let p = e.sketch.curve_point(profile.i(), t);
        let (rx, ry) = (p.0 - c.0, p.1 - c.1);
        assert!((rx.hypot(ry) - 4.0).abs() < 1e-9, "at {t}: the profile is {} from the roller", rx.hypot(ry));
        let h = 1e-4;
        let (a, b) = (e.sketch.curve_point(profile.i(), t - h), e.sketch.curve_point(profile.i(), t + h));
        let (tx, ty) = (b.0 - a.0, b.1 - a.1);
        assert!((tx * rx + ty * ry).abs() < 1e-6 * tx.hypot(ty) * 4.0, "at {t}: not touching the roller");
        // the near side faces the cam's centre more than the roller's centre does
        assert!(p.0.hypot(p.1) < c.0.hypot(c.1), "at {t}: the far side");
    }
}

/// **A tooth cuts its conjugate.**  An involute flank of the pinion's base circle (a formula
/// curve) turning about `o1` meshes a gear turning about `o2` at the ratio of their pitch
/// radii: the envelope of the pinion's flank in the gear's frame is the involute of the gear's
/// base circle, so its normal at every roll is tangent to that circle.  The conjugate is
/// generated, not stated: no formula for the gear's tooth is in the document.
#[test]
fn a_tooth_cuts_its_conjugate() {
    // pitch radii 20 and 40, 20° pressure angle: base radii 20 cos 20° and 40 cos 20°
    let src = "\
component Involute(c: circle, phase: Angle, u: Angle) {
  p := point(x: c.center.x + c.r * (cos(u + phase) + u * pi / 180 * sin(u + phase)), \
             y: c.center.y + c.r * (sin(u + phase) - u * pi / 180 * cos(u + phase)))
}
o1 := point hint(x: 0, y: 0)
o2 := point hint(x: 60, y: 0)
fix(x == 0, y == 0) o1
fix(x == 60, y == 0) o2
base1 := circle(center: o1) hint(r: 18.79385241571817)
radius(18.79385241571817) base1
flank := Involute(base1, phase: -20).p over u in (0, 60)
pinion := motion(about: o1, ratio: -2)
gear := motion(about: o2, ratio: 1)
rel := motion(of: pinion, relative_to: gear)
conj := envelope(flank, under: rel, from: -12deg, to: 12deg)
";
    let e = build(src);
    let conj = e.map.ent_named("conj").unwrap();
    let rb2 = 40.0 * 20f64.to_radians().cos();
    let mut checked = 0;
    for k in 0..=12 {
        let t = -12.0 + 2.0 * k as f64;
        let p = e.sketch.curve_point(conj.i(), t);
        if !(p.0.is_finite() && p.1.is_finite()) {
            continue;
        }
        let h = 1e-4;
        let (a, b) = (e.sketch.curve_point(conj.i(), t - h), e.sketch.curve_point(conj.i(), t + h));
        let (tx, ty) = (b.0 - a.0, b.1 - a.1);
        // the normal through p, its distance from the gear's centre
        let (nx, ny) = (-ty, tx);
        let (wx, wy) = (60.0 - p.0, 0.0 - p.1);
        let off = (wx * ny - wy * nx).abs() / nx.hypot(ny);
        assert!((off - rb2).abs() < 1e-6, "at roll {t}: the conjugate's normal is {off} from o2, not {rb2}");
        checked += 1;
    }
    assert!(checked >= 10, "only {checked} rolls cut");
}

/// A planar envelope says what is wrong with it: a side that is neither word, a tool a traced
/// curve (not yet a cutter), a spatial rotation carrying a tool of the sheet.
#[test]
fn a_planar_envelope_refuses_what_it_cannot_cut() {
    let base = format!("{RACK}{RACK_MOTION}");
    for (tail, want) in [
        ("cut := envelope(flank, under: rel, from: 0deg, to: 10deg, side: left)", "`near` or `far`"),
        ("st := RackFlank(pitch, datum, alpha: 20, s0: 5).p over u in (-25, 25)\n\
          cut := envelope(st, under: rel, from: 0deg, to: 10deg)", "a traced curve cannot cut yet"),
        ("spin := motion(about: slide)\ncut := envelope(flank, under: spin, from: 0deg, to: 10deg)",
         "turns about a line in space"),
        ("cut := envelope(flank, under: rel, from: 10deg, to: 0deg)", "increasing"),
    ] {
        let (prog, errs) = gcs_core::syntax::parse(&format!("{base}{tail}\n"));
        assert!(errs.is_empty(), "{errs:?}");
        let e = gcs_core::program::elaborate(&prog);
        let said: Vec<&String> = e.errors().map(|d| &d.message).collect();
        assert!(said.iter().any(|m| m.contains(want)), "{tail}: {said:?}");
    }
}

/// A line tangent to an envelope: the rack's cut flank.  A line pinned at a point on the
/// involute's tangent at roll −15 (read off the curve itself, then fixed) and free to swing
/// solves onto that tangent — square to the line of action.  The involute's cusp on the base
/// circle is the other answer, where `C' = 0` and every line through it is "tangent": the
/// contact must not end there.
#[test]
fn a_line_is_tangent_to_an_envelope() {
    let base = format!("{RACK}{RACK_MOTION}cut := envelope(flank, under: rel, from: -25deg, to: 25deg)\n");
    let probe = build(&base);
    let ci = probe.map.ent_named("cut").unwrap().i();
    let (p0, h) = (probe.sketch.curve_point(ci, -15.0), 1e-5);
    let (lo, hi) = (probe.sketch.curve_point(ci, -15.0 - h), probe.sketch.curve_point(ci, -15.0 + h));
    let (tx, ty) = (hi.0 - lo.0, hi.1 - lo.1);
    let n = tx.hypot(ty);
    let a = (p0.0 + 12.0 * tx / n, p0.1 + 12.0 * ty / n);
    let mut e = build(&format!(
        "{base}a := point hint(x: {}, y: {})\nb := point hint(x: {}, y: {})\nl := line(a, b)\n\
         fix(x == {}, y == {}) a\na distance(30) b\ncut tangent l hint(t: -13)\n",
        a.0, a.1, a.0 - 25.0 * tx / n + 3.0, a.1 - 25.0 * ty / n - 3.0, a.0, a.1
    ));
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    crate::common::fd_jacobian(&e.sketch, 1e-3);
    let c = e.sketch.constraints.iter().find(|c| !c.aux_params().is_empty()).unwrap();
    let t = e.sketch.params[c.aux_params()[0] as usize].value;
    assert!((t + 15.0).abs() < 1e-6, "the contact is at roll {t}, not −15");
    let (px, py) = e.sketch.curve_point(e.map.ent_named("cut").unwrap().i(), t);
    let at = |n: &str| e.sketch.point_xy(e.map.ent_named(n).unwrap().i());
    let (a, b) = (at("a"), at("b"));
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    // on the line, and square to the normal through the pitch point
    assert!(((px - a.0) * dy - (py - a.1) * dx).abs() < 1e-6 * dx.hypot(dy));
    let tr = (-t).to_radians();
    let (nx, ny) = (px - 30.0 * tr.cos(), py - 30.0 * tr.sin());
    assert!((nx * dx + ny * dy).abs() < 1e-6 * dx.hypot(dy) * nx.hypot(ny));
}
