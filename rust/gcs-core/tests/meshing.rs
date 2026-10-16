//! The meshing of two flanks (#283, step 3): tooth contact analysis as a locus.  A point held to
//! `meshing(pinion_flank, gear_flank over t in (a, b), under: gear_turn, seen_from: pinion_turn)`
//! owns the pinion's phase, the gear's lag and a place on each flank; after the solve the contact
//! is traced over the phase.  Each is held here to something worked out apart from
//! `envelope::meshing`: closed forms (a conjugate pair transmits without error, a pair conjugate at
//! another ratio at that ratio), and a gap between the two members closed by turning the gear,
//! found with no normal and no row of the meshing.
use crate::common::build;
use gcs_core::constraints::CKind;
use gcs_core::envelope::meshing::{Meshing, Sample, Trace};
use gcs_core::model::Sketch;
use gcs_core::solve;
use gcs_core::space::{add, cross, dot, norm, scale, sub, unit};

pub(super) fn solved(src: &str) -> gcs_core::program::Elaborated {
    let mut e = build(src);
    let r = solve::solve(&mut e.sketch, Default::default());
    assert!(r.success, "{r:?}");
    e
}

/// The mean point's mesh, reduced to its kinematics as `spiral_bevel/mean_point.sv` reduces it —
/// crossed shafts 72 apart, the gear at half the pinion's speed — and the gear's flank, formed: a
/// cone about the face mill's axis through M, its blades leaning 15°.
pub(super) const MESH: &str = "\
unit mm
use std
g0 := point hint((0, 0, 0))
g1 := point hint((0, 0, 10))
fix((0, 0, 0)) g0
fix((0mm, 0mm, 10mm)) g1
gear_ax := line(g0, g1)
p0 := point hint((0, 72, 0))
p1 := point hint((10, 72, 0))
fix((0mm, 72mm, 0mm)) p0
fix((10mm, 72mm, 0mm)) p1
pinion_ax := line(p0, p1)
gear_turn := frame
std.Turn(gear_turn, about: gear_ax, ratio: -0.5)
pinion_turn := frame
std.Turn(pinion_turn, about: pinion_ax, ratio: 1)
M := point hint((0, 48, 0))
fix((0mm, 48mm, 0mm)) M
C := point hint((28.28, 48, -28.28))
fix((28.2842712mm, 48mm, -28.2842712mm)) C
top := point hint((27, 58, -27))
fix((27.6679mm, 57.9619mm, -27.6679mm)) top
mill_ax := line(C, top)
apex := point hint((35, -60, -35))
apex coincident mill_ax
spine := line(apex, C)
flank := std.Cone(spine, half: 15deg)
M coincident flank
";

pub(super) const HALF: f64 = 15.0;

pub(super) fn at(e: &gcs_core::program::Elaborated, name: &str) -> [f64; 3] {
    e.sketch.world_point(e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i())
}

fn traced(sk: &Sketch, name: &str, n: usize) -> Trace {
    let i = sk.meshings.iter().position(|m| m.name == name)
        .unwrap_or_else(|| panic!("no meshing `{name}`"));
    Meshing::named(sk, i).unwrap().traced(n).unwrap()
}

/// `x` turned `angle` about the line through `o` along the unit `d`.
pub(super) fn turned(x: [f64; 3], o: [f64; 3], d: [f64; 3], angle: f64) -> [f64; 3] {
    let w = sub(x, o);
    let (s, c) = angle.sin_cos();
    add(o, add(add(scale(w, c), scale(cross(d, w), s)), scale(d, dot(d, w) * (1.0 - c))))
}

/// The pinion's turn at phase `deg`, and the gear's at roll `deg` of the mesh: about x through
/// (0, 72, 0), and about z at half the speed the other way.
pub(super) fn pinion_pose(x: [f64; 3], deg: f64) -> [f64; 3] {
    turned(x, [0.0, 72.0, 0.0], [1.0, 0.0, 0.0], deg.to_radians())
}
pub(super) fn gear_pose(x: [f64; 3], deg: f64) -> [f64; 3] {
    turned(x, [0.0; 3], [0.0, 0.0, 1.0], -0.5 * deg.to_radians())
}

/// How far `x` stands outside the cone with apex `a`, unit axis `d` and half angle `half`: its
/// distance from the generator in its meridian, signed.
pub(super) fn off_cone(x: [f64; 3], a: [f64; 3], d: [f64; 3], half: f64) -> f64 {
    let w = sub(x, a);
    let h = dot(w, d);
    let rho = norm(sub(w, scale(d, h)));
    rho * half.to_radians().cos() - h * half.to_radians().sin()
}

/// Where a sample says the two touch, read against the poses and the gear's cone apart from the
/// meshing: the pinion's point at the phase and the gear's at the phase and lag are the contact,
/// the gear's on its cone.
pub(super) fn holds_together(s: &Sample, apex: [f64; 3], axis: [f64; 3]) {
    let (phase, lag) = (s.place.phase, s.place.lag);
    assert!(norm(sub(pinion_pose(s.on_pinion, phase), s.position)) < 1e-9, "{s:?}");
    assert!(norm(sub(gear_pose(s.on_gear, phase + lag), s.position)) < 1e-9, "{s:?}");
    assert!(off_cone(s.on_gear, apex, axis, HALF).abs() < 1e-9, "{s:?}");
    assert!(s.residual < 1e-9, "{s:?}");
}

/// **A conjugate pair transmits without error.** The pinion's flank is the envelope of the gear's
/// under the mesh, so at every phase the two touch with the gear exactly where the ratio puts it,
/// and along a line: the rows lose a rank. And the same flank generated with the gear turning
/// 1 % faster is conjugate to it at that ratio, so meshed at the drawn one it lags by the
/// difference: `(q − 1)·φ`, exactly.
#[test]
fn a_conjugate_pair_transmits_without_error_and_one_cut_at_another_ratio_at_that_ratio() {
    let src = format!("{MESH}\
pinion_flank := envelope(flank over t in (-40deg, 40deg), under: gear_turn, seen_from: pinion_turn)
contact := meshing(pinion_flank, flank over t in (-20deg, 20deg), under: gear_turn, seen_from: pinion_turn)
Q := point hint((0, 48, 0))
Q coincident(t == 0deg) contact
");
    let e = solved(&src);
    let (apex, axis) = (at(&e, "apex"), unit(sub(at(&e, "C"), at(&e, "apex"))));
    let t = traced(&e.sketch, "contact", 24);
    assert_eq!(t.found().count(), 25);
    for s in t.found() {
        assert!(s.place.lag.abs() < 1e-10, "{s:?}");
        assert!(s.line, "a conjugate pair touches along a line: {s:?}");
        holds_together(s, apex, axis);
    }
    let text = gcs_core::report::meshing_text(&e.sketch, 0);
    assert!(text.starts_with("contact: line contact; transmission error"), "{text}");

    let q = 1.01;
    let cut = src.replace("pinion_flank := envelope(flank over t in (-40deg, 40deg), under: gear_turn,",
        &format!("cutting_turn := frame\n\
                  std.Turn(cutting_turn, about: gear_ax, ratio: -0.5 * {q})\n\
                  pinion_flank := envelope(flank over t in (-40deg, 40deg), under: cutting_turn,"));
    let e = solved(&cut);
    let t = traced(&e.sketch, "contact", 24);
    assert_eq!(t.found().count(), 25);
    for s in t.found() {
        let want = (q - 1.0) * s.place.phase;
        assert!((s.place.lag - want).abs() < 1e-10 * (1.0 + want.abs()), "{s:?}");
        assert!((s.gear_error.unwrap() + 0.5 * want).abs() < 1e-10, "{s:?}");
        assert!(s.line, "{s:?}");
        holds_together(s, apex, axis);
    }
}

/// **A mismatched pair touches at a point.** The pinion's cutter is the gear's flank grown about M
/// (its radius 25 % larger, its blades as lean): the two cones share the generator through M and
/// their normal along it and nothing else, so the bearing is that line and the contact a point on
/// it — the rows regular — and the pair, generated at a modified roll, transmits at that roll's
/// ratio exactly, as the conjugate one does.
#[test]
fn a_cutter_of_another_radius_at_a_modified_roll_touches_at_a_point_on_the_common_generator() {
    let q = 0.995;
    let src = format!("{MESH}\
generator := line(M, apex)
apex2 := point hint((44, -87, -44))
apex2 coincident(t == 1.25) generator
top2 := point hint((44, -77, -44))
spine2 := line(apex2, top2)
spine2 parallel spine
apex2 distance(10mm) top2
cutter := std.Cone(spine2, half: 15deg)
cutting_turn := frame
std.Turn(cutting_turn, about: gear_ax, ratio: -0.5 * {q})
pinion_flank := envelope(cutter over t in (-40deg, 40deg), under: cutting_turn, seen_from: pinion_turn)
contact := meshing(pinion_flank, flank over t in (-20deg, 20deg), under: gear_turn, seen_from: pinion_turn)
Q := point hint((1, 47, 1))
Q coincident(t == 0deg) contact
");
    let e = solved(&src);
    let (apex, m) = (at(&e, "apex"), at(&e, "M"));
    let axis = unit(sub(at(&e, "C"), apex));
    let along = unit(sub(m, apex));
    // where the pair is drawn the contact is on that generator — near M, not at it, the cutting
    // roll's meshing equation not the mesh's
    let w = sub(at(&e, "Q"), m);
    assert!(norm(sub(w, scale(along, dot(w, along)))) < 1e-8, "{:?}", at(&e, "Q"));
    let t = traced(&e.sketch, "contact", 24);
    assert_eq!(t.found().count(), 25);
    for s in t.found() {
        assert!(!s.line, "a mismatched pair touches at a point: {s:?}");
        let want = (q - 1.0) * s.place.phase;
        assert!((s.place.lag - want).abs() < 1e-9 * (1.0 + want.abs()), "{s:?}");
        holds_together(s, apex, axis);
        // on the gear, the bearing is the generator through M
        let w = sub(s.on_gear, m);
        assert!(norm(sub(w, scale(along, dot(w, along)))) < 1e-8, "{s:?}");
    }
    let text = gcs_core::report::meshing_text(&e.sketch, 0);
    let said = "contact: point contact; transmission error (the gear's angle)";
    assert!(text.starts_with(said), "{text}");
}

/* -- a gap closed by turning the gear ---------------------------------------------------------- */

/// The pinion's flank generated by the cone with apex `a`, unit axis `d` and half `HALF`, under
/// the mesh, read with nothing of `envelope::held`: the cone's point at `(u, v)`, carried by the
/// gear's turn and back by the pinion's at roll `t`, its roll `v` the one where the cone's normal
/// is square to the point's velocity there (both by central differences, the root by secants).
pub(super) struct Generated {
    a: [f64; 3],
    d: [f64; 3],
    e1: [f64; 3],
    e2: [f64; 3],
}

impl Generated {
    pub(super) fn new(a: [f64; 3], d: [f64; 3]) -> Generated {
        let e1 = unit(cross(d, [0.0, 1.0, 0.0]));
        Generated { a, d, e1, e2: cross(d, e1) }
    }

    fn cone(&self, u: f64, v: f64) -> [f64; 3] {
        let (sh, ch) = HALF.to_radians().sin_cos();
        let radial = add(scale(self.e1, v.cos()), scale(self.e2, v.sin()));
        add(self.a, scale(add(scale(self.d, ch), scale(radial, sh)), u))
    }

    /// The source's point at `(u, v)` in the pinion's frame at roll `t` (degrees).
    fn moved(&self, u: f64, v: f64, t: f64) -> [f64; 3] {
        pinion_pose(gear_pose(self.cone(u, v), t), -t)
    }

    /// The moved source's unit normal `∂u × ∂v` at `(u, v)` and roll `t`.
    pub(super) fn normal(&self, u: f64, v: f64, t: f64) -> [f64; 3] {
        let (h, k) = (1e-5, 1e-6);
        let du = scale(sub(self.moved(u + h, v, t), self.moved(u - h, v, t)), 0.5 / h);
        let dv = scale(sub(self.moved(u, v + k, t), self.moved(u, v - k, t)), 0.5 / k);
        unit(cross(du, dv))
    }

    fn meshing(&self, u: f64, v: f64, t: f64) -> f64 {
        let k = 1e-6;
        let dt = scale(sub(self.moved(u, v, t + k), self.moved(u, v, t - k)), 0.5 / k);
        dot(self.normal(u, v, t), dt)
    }

    /// The envelope's point at `(u, t)`, in the pinion's frame, its `v` from `v0`.
    pub(super) fn point(&self, u: f64, t: f64, v0: f64) -> ([f64; 3], f64) {
        let (mut a, mut b) = (v0, v0 + 1e-4);
        let (mut fa, mut fb) = (self.meshing(u, a, t), self.meshing(u, b, t));
        for _ in 0..60 {
            if fb == fa || fb.abs() < 1e-15 {
                break;
            }
            let c = b - fb * (b - a) / (fb - fa);
            (a, fa) = (b, fb);
            b = c;
            fb = self.meshing(u, b, t);
        }
        (self.moved(u, b, t), b)
    }
}

/// Where a sample's point on the pinion's flank starts on `g`'s place `(u, t, v)`: its roll, and
/// the point carried back by it to the cone.
pub(super) fn start(g: &Generated, s: &Sample) -> (f64, f64, f64) {
    let roll = s.place.pinion[2].to_degrees();
    let w = sub(gear_pose(pinion_pose(s.on_pinion, roll), -roll), g.a);
    (dot(w, g.d) / HALF.to_radians().cos(), roll, dot(w, g.e2).atan2(dot(w, g.e1)))
}

/// How far a pinion's flank — `point` its point at `(u, t)` in the pinion's frame, its `v` from
/// the one given — stands outside the gear's cone at its nearest, the pinion at `phase` and the
/// gear at `phase + lag` (degrees): the stationary point of the signed distance over the flank's
/// `(u, t)`, by Newton on central differences from `start`.  The gap and the point in the fixed
/// frame.
fn gap(point: &impl Fn(f64, f64, f64) -> ([f64; 3], f64), cone: ([f64; 3], [f64; 3]), phase: f64,
       lag: f64, start: (f64, f64, f64)) -> (f64, [f64; 3]) {
    let (mut u, mut t, mut v) = start;
    let f = |u: f64, t: f64, v: f64| -> (f64, f64, [f64; 3]) {
        let (x, v) = point(u, t, v);
        let y = pinion_pose(x, phase);
        (off_cone(gear_pose(y, -(phase + lag)), cone.0, cone.1, HALF), v, y)
    };
    let (hu, ht) = (1e-3, 1e-3);
    for _ in 0..30 {
        let s = |du: f64, dt: f64| f(u + du, t + dt, v).0;
        let f0 = s(0.0, 0.0);
        let gu = (s(hu, 0.0) - s(-hu, 0.0)) / (2.0 * hu);
        let gt = (s(0.0, ht) - s(0.0, -ht)) / (2.0 * ht);
        let huu = (s(hu, 0.0) - 2.0 * f0 + s(-hu, 0.0)) / (hu * hu);
        let htt = (s(0.0, ht) - 2.0 * f0 + s(0.0, -ht)) / (ht * ht);
        let hut = (s(hu, ht) - s(hu, -ht) - s(-hu, ht) + s(-hu, -ht)) / (4.0 * hu * ht);
        let det = huu * htt - hut * hut;
        let (du, dt) = ((htt * gu - hut * gt) / det, (huu * gt - hut * gu) / det);
        u -= du;
        t -= dt;
        v = f(u, t, v).1;
        if du.abs() < 1e-11 && dt.abs() < 1e-11 {
            break;
        }
    }
    let (d, _, y) = f(u, t, v);
    (d, y)
}

/// The lag that closes the gap between a pinion's flank (`point`, as `gap` reads it) and the
/// gear's cone at a sample's phase — by secants from either side of the sample's, the flank's
/// point started from where the sample says the pinion's is (`start`) — and where they close, in
/// the fixed frame.
pub(super) fn closing(point: &impl Fn(f64, f64, f64) -> ([f64; 3], f64),
                      cone: ([f64; 3], [f64; 3]), s: &Sample, start: (f64, f64, f64))
                      -> (f64, [f64; 3]) {
    let close = |lag: f64| gap(point, cone, s.place.phase, lag, start).0;
    let (mut a, mut b) = (s.place.lag - 1e-4, s.place.lag + 1e-4);
    let (mut fa, mut fb) = (close(a), close(b));
    assert!(fa * fb < 0.0, "the gap closes between: {fa} {fb}");
    for _ in 0..40 {
        let c = b - fb * (b - a) / (fb - fa);
        (a, fa) = (b, fb);
        b = c;
        fb = close(b);
        if fb.abs() < 1e-13 {
            break;
        }
    }
    (b, gap(point, cone, s.place.phase, b, start).1)
}

/// Step 3's turned cutter: the pinion's cutter the gear's cone turned 2° about its normal at M,
/// generating under the mesh, and Q held to the meshing where the pair is drawn.  The document, M,
/// the gear's cone's apex and axis, and the cutter's apex and axis point.
fn turned_cutter() -> (String, [[f64; 3]; 3], [[f64; 3]; 2]) {
    let base = solved(&format!("{MESH}gear_flank := envelope(flank over t in (-40deg, 40deg), under: gear_turn, \
                                seen_from: pinion_turn)\n"));
    let (m, apex, c) = (at(&base, "M"), at(&base, "apex"), at(&base, "C"));
    let axis = unit(sub(c, apex));
    let g = sub(m, apex);
    let normal = unit(cross(cross(axis, g), g));
    let turn = 2f64.to_radians();
    let (a2, c2) = (turned(apex, m, normal, turn), turned(c, m, normal, turn));
    let pt = |p: [f64; 3]| format!("({}mm, {}mm, {}mm)", p[0], p[1], p[2]);
    let src = format!("{MESH}\
cut_apex := point hint(({}, {}, {}))
fix({}) cut_apex
cut_c := point hint(({}, {}, {}))
fix({}) cut_c
cut_spine := line(cut_apex, cut_c)
cutter := std.Cone(cut_spine, half: 15deg)
pinion_flank := envelope(cutter over t in (-40deg, 40deg), under: gear_turn, seen_from: pinion_turn)
contact := meshing(pinion_flank, flank over t in (-15deg, 15deg), under: gear_turn, seen_from: pinion_turn)
Q := point hint((0, 48, 0))
Q coincident(t == 0deg) contact
", a2[0], a2[1], a2[2], pt(a2), c2[0], c2[1], c2[2], pt(c2));
    (src, [m, apex, axis], [a2, c2])
}

/// **A mismatched pair, checked against the gap it closes.** The pinion's cutter is the gear's
/// cone turned 2° about its normal at M: the two touch at M and nowhere else, so the bearing
/// localises and the gear lags by a little that grows with the phase.  At each phase the lag the
/// meshing finds is the turn of the gear that closes the gap between the two members — the
/// pinion's flank's nearest approach to the gear's cone, worked out from the cone's equation and
/// the motions' poses alone — and the contact is where they close.
#[test]
fn a_mismatched_pair_lags_by_the_turn_that_closes_the_gap() {
    let (src, [m, apex, axis], [a2, c2]) = turned_cutter();
    let e = solved(&src);
    assert!(norm(sub(at(&e, "Q"), m)) < 1e-9, "the contact is at M where the pair is drawn");
    let t = traced(&e.sketch, "contact", 12);
    assert_eq!(t.found().count(), 13);
    let generated = Generated::new(a2, unit(sub(c2, a2)));
    let [lo, hi] = t.error_range().unwrap();
    assert!(hi - lo > 1e-6, "the gear lags by something: {lo} … {hi}");
    for s in t.found() {
        assert!(!s.line, "{s:?}");
        holds_together(s, apex, axis);
        if s.place.phase == 0.0 {
            assert!(s.place.lag.abs() < 1e-12, "{s:?}");
            continue;
        }
        // the lag that closes the gap, and where
        let point = |u, t, v| generated.point(u, t, v);
        let (lag, y) = closing(&point, (apex, axis), s, start(&generated, s));
        assert!((lag - s.place.lag).abs() < 1e-9, "phase {}: closed at {lag}, meshed at {}",
                s.place.phase, s.place.lag);
        assert!(norm(sub(y, s.position)) < 1e-5, "phase {}: {y:?} {:?}", s.place.phase, s.position);
    }
}

/// **The lag is a number a document states.** The pair cut by a larger cutter at a modified roll
/// lags in proportion to the phase; stated a lag and left its phase, a point on the contact solves
/// to the phase that lag is at, `lag / (q − 1)`, and to where the two touch there.
#[test]
fn a_stated_lag_solves_the_phase_it_is_at() {
    let q = 1.01;
    let lag = 0.07;
    let src = format!("{MESH}\
generator := line(M, apex)
apex2 := point hint((44, -87, -44))
apex2 coincident(t == 1.25) generator
top2 := point hint((44, -77, -44))
spine2 := line(apex2, top2)
spine2 parallel spine
apex2 distance(10mm) top2
cutter := std.Cone(spine2, half: 15deg)
cutting_turn := frame
std.Turn(cutting_turn, about: gear_ax, ratio: -0.5 * {q})
pinion_flank := envelope(cutter over t in (-40deg, 40deg), under: cutting_turn, seen_from: pinion_turn)
contact := meshing(pinion_flank, flank over t in (-20deg, 20deg), under: gear_turn, seen_from: pinion_turn)
Q := point hint((0, 48, 0))
Q coincident(t == 0deg) contact
R := point hint((1, 48, 1))
R coincident(lag == {lag}) contact
");
    let e = solved(&src);
    let c = e.sketch.constraints.iter().filter(|c| c.kind == CKind::PointOnMeshing).nth(1).unwrap();
    let phase = e.sketch.params[c.args[2].param() as usize].value;
    assert!((phase - lag / (q - 1.0)).abs() < 1e-8, "{phase}");
    // R is where the two touch at that phase, walked to from Q's
    let m = Meshing::named(&e.sketch, 0).unwrap();
    let t = m.trace(&m.anchor.unwrap(), &[phase]);
    let there = t[0].unwrap();
    assert!(!there.line);
    let r = at(&e, "R");
    assert!(norm(sub(there.position, r)) < 1e-7, "{:?} {r:?}", there.position);
}

/// A revolution's side meshed with its envelope under the mesh of skew shafts, one a screw, and a
/// point held to the contact where the pair is drawn.  Its joints are written out as `std.Turn`
/// writes them, so the flat form prints it.
pub(super) const SIDE_MESHING: &str = "\
unit mm
use std
in std.front {
  o := point
  q := point
  fix((0, 0)) o
  fix((0, 10)) q
  ax := line(o, q)
  a := point
  b := point
  c := point
  d := point
  fix((6, 2)) a
  fix((9, 7)) b
  fix((12, 7)) c
  fix((12, 2)) d
  l1 := line(a, b)
  l2 := line(b, c)
  l3 := line(c, d)
  l4 := line(d, a)
}
blade := solid(face(l1, l2, l3, l4), about: ax)
side := surface(blade, l1)
g1 := point hint((5, 40, 3))
g2 := point hint((9, 41, 13))
fix((5, 40, 3)) g1
fix((9, 41, 13)) g2
gear_ax := line(g1, g2)
spin := frame
ax coincident spin.axis
fix(ratio == 1, advance == 0mm) spin
gear_turn := frame
gear_ax coincident gear_turn.axis
fix(ratio == -0.4, advance == 3mm) gear_turn
f1 := envelope(side over t in (-60deg, 60deg), under: spin, seen_from: gear_turn)
contact := meshing(f1, side over t in (-20deg, 20deg), under: spin, seen_from: gear_turn)
p := point hint((3, 1, 8))
p coincident(t == 0deg, lag == 0) contact
";

/// A meshing prints as it is written and reads back the same, a pinned phase and lag with it —
/// here over a revolution's side, formed, and its envelope under the mesh; one read off a frame
/// seen from nothing, or over a flank the solve cannot read, is refused saying why.
#[test]
fn a_meshing_prints_back_and_refuses_what_it_cannot_turn() {
    let src = SIDE_MESHING;
    let (mut prog, errs) = crate::common::parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let first = gcs_core::program::elaborate(&prog);
    assert!(first.ok(), "{:?}", first.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(first.sketch.meshings.len(), 1);
    let printed = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    let said = "contact := meshing(first: f1, second: side over t in (-20deg, 20deg), motion: spin, seen_from: gear_turn)";
    assert!(printed.contains(said), "{printed}");
    assert!(printed.contains("p coincident(t == 0deg, lag == 0) contact"), "{printed}");
    let again = build(&printed);
    let flanks = |e: &gcs_core::program::Elaborated| e.sketch.meshings.iter()
        .map(|m| (m.name.clone(), m.flanks.clone(), m.roll.clone())).collect::<Vec<_>>();
    assert_eq!(flanks(&again), flanks(&first));
    let points = again.sketch.constraints.iter().filter(|c| c.kind == CKind::PointOnMeshing);
    assert_eq!(points.count(), 1);

    let refused = |src: &str, want: &str| {
        let (prog, errs) = crate::common::parse(src);
        assert!(errs.is_empty(), "{errs:?}");
        let e = gcs_core::program::elaborate(&prog);
        assert!(e.errors().any(|d| d.message.contains(want)),
                "{want}: {:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    };
    refused(&src.replace("under: spin, seen_from: gear_turn)", "under: gear_turn)"),
            "`gear_turn` is no relative motion");
    refused(&src.replace("meshing(f1, side over", "meshing(f1, ax over"),
            "`ax` is no flank the solve reads");
    refused(&src.replace("in (-20deg, 20deg)", "in (20deg, -20deg)"),
            "a meshing needs increasing roll bounds");
}

/// `spiral_bevel/contact.sv` (#283 step 3): the mean point's pair, the gear formed by its face
/// mill and the pinion cut by a larger cutter at a modified roll.  At each design the bearing runs
/// along the generator the two cones share through M, a point contact, and the gear's angle is
/// off the ratio's by `(roll − 1)` times the ratio times the phase — and the report says so.
#[test]
fn the_mean_points_bearing_runs_along_the_common_generator() {
    let text = include_str!("../../examples/spiral_bevel/contact.sv");
    for (grown, roll) in [(1.25, 1.005), (0.8, 0.99), (1.5, 1.0)] {
        let src = fixtures::gear::configure(text, &[("grown", grown), ("roll", roll)]);
        let e = solved(&src);
        let (m, apex) = (at(&e, "M"), at(&e, "apex"));
        let along = unit(sub(m, apex));
        let on_generator = |x: [f64; 3]| {
            let w = sub(x, m);
            norm(sub(w, scale(along, dot(w, along))))
        };
        assert!(on_generator(at(&e, "Q")) < 1e-8, "{grown} {roll}: {:?}", at(&e, "Q"));
        let t = traced(&e.sketch, "bearing", 24);
        assert_eq!(t.found().count(), 25, "{grown} {roll}");
        for s in t.found() {
            assert!(!s.line, "{grown} {roll}: {s:?}");
            let want = (roll - 1.0) * s.place.phase;
            let off = (s.place.lag - want).abs();
            assert!(off < 1e-9 * (1.0 + want.abs()), "{grown} {roll}: {s:?}");
            assert!(on_generator(s.on_gear) < 1e-8, "{grown} {roll}: {s:?}");
        }
        let text = gcs_core::report::meshing_text(&e.sketch, 0);
        let scalar = gcs_core::constraints::SpecKind::Scalar;
        let edge = gcs_core::io::reading(scalar, 0.5 * (roll - 1.0).abs() * 15.0);
        let said = "bearing: point contact; transmission error (the gear's angle)";
        assert!(text.starts_with(said), "{text}");
        if roll != 1.0 {
            let range = format!("-{edge}° … {edge}° over the phase -15° … 15°");
            assert!(text.contains(&range), "{text}");
        }
    }
}

/* -- the common normal and the path of contact at a point (#345) ------------------------------- */

/// `contact.sv` with a line along the common normal and one along the path of contact at Q (the
/// phase the pair is drawn at) and at R (pinned at 10°).
fn contact_with_lines(text: &str) -> String {
    format!("{text}
R := point hint((0, 48, 0))
R coincident(t == 10deg) bearing
nq := point hint((0, 58, 0))
normal_q := line(Q, nq)
bearing perpendicular(at: Q) normal_q
Q distance(10mm) nq
pq := point hint((10, 48, 0))
path_q := line(Q, pq)
bearing tangent(at: Q) path_q
Q distance(10mm) pq
nr := point hint((0, 58, 0))
normal_r := line(R, nr)
bearing perpendicular(at: R) normal_r
R distance(10mm) nr
pr := point hint((10, 48, 0))
path_r := line(R, pr)
bearing tangent(at: R) path_r
R distance(10mm) pr
")
}

/// How far apart two directions are, either way round: the sine of the angle between them.
fn skew(a: [f64; 3], b: [f64; 3]) -> f64 {
    norm(cross(unit(a), unit(b)))
}

/// `contact.sv`'s pair apart from the meshing: at phase `phase` (degrees) the gear has turned as
/// the cutting turn has (`−roll/2` the pinion's: the pair lags `(roll − 1)` times the phase), and
/// the two touch on the generator the gear's cone and the cutter's share through M, turned with
/// the gear, where the cutting motion's relative velocity is square to the cones' normal — one
/// normal all along a cone's generator.  The point there, and that normal.
fn closed_contact(m: [f64; 3], apex: [f64; 3], spine: [f64; 3], roll: f64, phase: f64)
    -> ([f64; 3], [f64; 3]) {
    let z = [0.0, 0.0, 1.0];
    let theta = -0.5 * roll * phase.to_radians();
    let gear = |x: [f64; 3]| turned(x, [0.0; 3], z, theta);
    let (m1, along) = (gear(m), unit(sub(gear(m), gear(apex))));
    let w = sub(m, apex);
    let n = gear(unit(cross(cross(spine, w), w)));
    // per radian of the pinion: the gear's turn about z less the pinion's about x through
    // (0, 72, 0)
    let v = |x: [f64; 3]| {
        sub(scale(cross(z, x), -0.5 * roll), cross([1.0, 0.0, 0.0], sub(x, [0.0, 72.0, 0.0])))
    };
    let (v0, v1) = (dot(n, v(m1)), dot(n, sub(v(add(m1, along)), v(m1))));
    (add(m1, scale(along, -v0 / v1)), n)
}

/// **The common normal and the path of contact are relations on a meshing** (#345 S1): on
/// `contact.sv`'s pair, at the phase it is drawn at and at 10°, the point held there is on the
/// generator the two cones share, turned with the gear; a line along the common normal there runs
/// along the gear's cone's normal, and a line along the path of contact along the derivative of
/// that point over the phase.  (Not along the generator, the bearing's line: the contact stands
/// still on the gear's flank, and the gear's turn carries it.)
#[test]
fn the_normal_and_the_path_of_contact_at_a_point_are_the_pairs_own() {
    let text = include_str!("../../examples/spiral_bevel/contact.sv");
    let roll = 1.005; // the document's
    let e = solved(&contact_with_lines(text));
    let (m, apex) = (at(&e, "M"), at(&e, "apex"));
    let spine = unit(sub(at(&e, "C"), apex));
    for (p, phase, normal, path) in [("Q", 0.0, "nq", "pq"), ("R", 10.0, "nr", "pr")] {
        let (x, n) = closed_contact(m, apex, spine, roll, phase);
        assert!(norm(sub(at(&e, p), x)) < 1e-8, "{p}: {:?} {x:?}", at(&e, p));
        let d = sub(at(&e, normal), at(&e, p));
        assert!(skew(d, n) < 1e-12, "{p}: the normal {d:?} against {n:?}");
        let h = 1e-3;
        let tangent = sub(closed_contact(m, apex, spine, roll, phase + h).0,
                          closed_contact(m, apex, spine, roll, phase - h).0);
        let d = sub(at(&e, path), at(&e, p));
        assert!(skew(d, tangent) < 1e-9, "{p}: the path {d:?} against {tangent:?}: {}",
                skew(d, tangent));
        assert!(skew(d, sub(m, apex)) > 0.9, "{p}: the path is no bearing");
    }
}

/// The direction of the path of contact at phase `phase`: a central difference of the contact
/// traced from the meshing's anchor either side of it, read with nothing of the path's own rows.
fn traced_path(sk: &Sketch, phase: f64) -> [f64; 3] {
    let m = Meshing::named(sk, 0).unwrap();
    let h = 1e-3;
    let t = m.trace(&m.anchor.unwrap(), &[phase - h, phase + h]);
    let (a, b) = (t[0].expect("traced below"), t[1].expect("traced above"));
    assert!(!a.line && !b.line, "a point contact");
    sub(b.position, a.position)
}

/// On step 3's turned cutter the path of contact at Q runs along the path the meshing traces: a
/// central difference of the contact's positions either side of the phase.
#[test]
fn the_path_of_contact_is_the_traced_paths_tangent() {
    let (src, _, _) = turned_cutter();
    let e = solved(&format!("{src}\
pq := point hint((10, 48, 0))
path_q := line(Q, pq)
contact tangent(at: Q) path_q
Q distance(10mm) pq
"));
    let d = sub(at(&e, "pq"), at(&e, "Q"));
    let traced = traced_path(&e.sketch, 0.0);
    assert!(skew(d, traced) < 1e-7, "{d:?} against {traced:?}: {}", skew(d, traced));
}

/// A point R held to `contact.sv`'s contact (`PIN` its pin, or none) and a line along the path of
/// contact there.
const PATH_AT_R: &str = "\
R := point hint((0, 48, 0))
R coincident PIN bearing
pr := point hint((10, 48, 0))
path_r := line(R, pr)
bearing tangent(at: R) path_r
R distance(10mm) pr
";

/// How far `d` turns from the x axis, in degrees.
fn from_x(d: [f64; 3]) -> f64 {
    dot(unit(d), [1.0, 0.0, 0.0]).acos().to_degrees()
}

/// **A number solves from a stated path angle** (#345 S1): on `contact.sv`'s pair the contact
/// stands still on the gear's flank, so the path of contact is the gear's turn carrying it, and
/// turns with the gear.  A point R held to the contact with no pin is free along the path; stated
/// the path's angle from the x axis there that the pair has at 10° of the pinion, R's phase,
/// seeded at 0°, solves to 10° — and the pair traces a path at that angle there.
#[test]
fn a_phase_solves_from_the_path_of_contacts_angle() {
    let text = include_str!("../../examples/spiral_bevel/contact.sv");
    let at10 = solved(&format!("{text}{}", PATH_AT_R.replace("PIN", "(t == 10deg)")));
    let stated = from_x(sub(at(&at10, "pr"), at(&at10, "R")));
    let at0 = from_x(traced_path(&at10.sketch, 0.0));
    assert!((stated - at0).abs() > 1.0, "the path turns with the gear: {at0} {stated}");
    let free = PATH_AT_R.replace("PIN ", "");
    let e = solved(&format!("{text}{free}path_r angle({stated}deg) std.x\n"));
    let c = e.sketch.constraints.iter().filter(|c| c.kind == CKind::PointOnMeshing).nth(1).unwrap();
    let phase = c.args[2].value(&e.sketch);
    assert!((phase - 10.0).abs() < 1e-8, "{phase}");
    let traced = from_x(traced_path(&e.sketch, phase));
    assert!((traced - stated).abs() < 1e-5, "{traced} {stated}");
}

/// The common normal and the path at a point print as they are written and read back the same,
/// sharing the place of the contact holding the point — written either way round, before the
/// contact or after it; at a point no contact holds on the meshing they are refused, naming the
/// statement they want.
#[test]
fn lines_at_a_point_on_a_meshing_print_back_and_share_its_place() {
    let src = format!("{SIDE_MESHING}\
m := point hint((15, 5, 4))
nl := line(p, m)
t := axis hint(dir: (1, 2, 3))
contact perpendicular(at: p) nl
contact tangent(at: p) t
");
    let (mut prog, errs) = crate::common::parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let printing = gcs_core::program::elaborate(&prog);
    assert!(printing.ok(), "{:?}", printing.errors().map(|d| &d.message).collect::<Vec<_>>());
    let printed = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    for want in ["contact perpendicular(at: p) nl", "contact tangent(at: p) t"] {
        assert!(printed.contains(want), "{want}\n{printed}");
    }
    let kinds = |e: &gcs_core::program::Elaborated| {
        let mut k: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
        k.sort_by_key(|k| k.name());
        k
    };
    let again = build(&printed);
    assert_eq!(kinds(&again), kinds(&printing));
    assert!(kinds(&again).contains(&CKind::Rate));
    let text = include_str!("../../examples/spiral_bevel/contact.sv");
    let src = contact_with_lines(text);
    let first = solved(&src);
    // each line reads the place of the contact holding its point, whichever came first
    let shared = |e: &gcs_core::program::Elaborated| {
        let sk = &e.sketch;
        let held = |c: &gcs_core::constraints::Constraint| -> Vec<u32> {
            c.args[2..10].iter().map(|a| a.param()).collect()
        };
        let contacts: Vec<_> =
            sk.constraints.iter().filter(|c| c.kind == CKind::PointOnMeshing).collect();
        let lines: Vec<_> = sk.constraints.iter()
            .filter(|c| c.kind == CKind::Rate).collect();
        assert_eq!(lines.len(), 4);
        for l in lines {
            let at = l.args[2].ent();
            let holding = contacts.iter().find(|c| c.args[0].ent() == at).unwrap();
            // the common normal and the path of contact are the contact's frame, read by a rate
            // sharing its phase, its first unknown, the rest read as columns (#365)
            assert_eq!(l.args[3].param(), held(holding)[0]);
            assert_eq!(l.params(sk)[6..14], held(holding)[..], "the contact's unknowns read");
        }
    };
    shared(&first);
    // Q's contact stated after the lines at Q, which are written the other way round
    let held = "Q coincident(t == 0deg) bearing\n";
    let late = contact_with_lines(&text.replace(held, ""))
        .replace("bearing tangent(at: Q) path_q", "path_q tangent(at: Q) bearing")
        .replace("bearing perpendicular(at: Q) normal_q", "normal_q perpendicular(at: Q) bearing");
    assert!(late.contains("path_q tangent(at: Q) bearing") && !late.contains(held));
    shared(&solved(&format!("{late}{held}")));
    let refused = |src: &str, want: &str| {
        let (prog, errs) = crate::common::parse(src);
        assert!(errs.is_empty(), "{errs:?}");
        let e = gcs_core::program::elaborate(&prog);
        assert!(e.errors().any(|d| d.message.contains(want)),
                "{want}: {:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    };
    let unheld = src.replace("R coincident(t == 10deg) bearing\n", "");
    refused(&unheld, "the common normal at `R` is taken where a contact holds `R` on `bearing`");
    refused(&unheld, "the path of contact at `R` is taken where a contact holds `R` on `bearing`");
}
