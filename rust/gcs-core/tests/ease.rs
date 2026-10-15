//! Ease-off (#345 S2): a meshing's first flank relieved along its normal by a parabola about a
//! point, `½ κₗ s² + ½ κₚ r²` — `s` how far along a line, `r` how far across it.  A conjugate
//! pair touches along a line; eased, it touches at a point, and the gear lags by the turn that
//! closes the relief.  Each is held here to something worked out apart from `envelope::meshing`:
//! the lag and the bearing against the motions' poses, the gear's cone and the eased flank's own
//! equation, found with no row of the meshing.
use crate::common::build;
use gcs_core::envelope::meshing::{Meshing, Sample};
use gcs_core::model::Sketch;
use gcs_core::solve;
use gcs_core::space::{add, cross, dot, norm, scale, sub, unit};

fn solved(src: &str) -> gcs_core::program::Elaborated {
    let mut e = build(src);
    let r = solve::solve(&mut e.sketch, Default::default());
    assert!(r.success, "{r:?}");
    e
}

/// `tests/meshing.rs`'s mesh — crossed shafts 72 apart, the gear at half the pinion's speed, the
/// gear's flank formed, a cone about the face mill's axis through M — and the pinion's flank its
/// envelope under the mesh: a conjugate pair.  `lengthwise` runs from M along the face mill's
/// circle there.
const MESH: &str = "\
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
gear_turn := motion(about: gear_ax, ratio: -0.5)
pinion_turn := motion(about: pinion_ax)
mesh := motion(gear_turn, relative_to: pinion_turn)
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
L := point hint((10, 48, 10))
fix((10mm, 48mm, 10mm)) L
lengthwise := line(M, L)
pinion_flank := envelope(flank, under: mesh, from: -40deg, to: 40deg)
";

const HALF: f64 = 15.0;
/// The ease: per millimetre, across `lengthwise` (the profile) and along it.
const PROFILE: f64 = 0.002;
const LENGTHWISE: f64 = 0.0005;

/// The conjugate pair, the pinion's flank eased by `profile` and `lengthwise` (texts), and Q held
/// to the contact where the pair is drawn.
fn eased(profile: &str, lengthwise: &str) -> String {
    format!("{MESH}\
contact := meshing(pinion_flank, flank, under: mesh, from: -20deg, to: 20deg, ease: M,
                   along: lengthwise, profile: {profile}, lengthwise: {lengthwise})
Q := point hint((0, 48, 0))
Q coincident(t == 0deg) contact
")
}

fn at(e: &gcs_core::program::Elaborated, name: &str) -> [f64; 3] {
    e.sketch.world_point(e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i())
}

fn meshing(sk: &Sketch) -> Meshing {
    Meshing::named(sk, 0).unwrap()
}

/// `x` turned `angle` about the line through `o` along the unit `d`.
fn turned(x: [f64; 3], o: [f64; 3], d: [f64; 3], angle: f64) -> [f64; 3] {
    let w = sub(x, o);
    let (s, c) = angle.sin_cos();
    add(o, add(add(scale(w, c), scale(cross(d, w), s)), scale(d, dot(d, w) * (1.0 - c))))
}

const X: [f64; 3] = [1.0, 0.0, 0.0];
const Z: [f64; 3] = [0.0, 0.0, 1.0];
const PINION: [f64; 3] = [0.0, 72.0, 0.0];

/// The pinion's turn at phase `deg`: about x through (0, 72, 0).
fn pinion_pose(x: [f64; 3], deg: f64) -> [f64; 3] {
    turned(x, PINION, X, deg.to_radians())
}

/// How far `x` stands outside the cone with apex `a`, unit axis `d` and half angle `HALF`: its
/// distance from the generator in its meridian, signed.
fn off_cone(x: [f64; 3], a: [f64; 3], d: [f64; 3]) -> f64 {
    let w = sub(x, a);
    let h = dot(w, d);
    let rho = norm(sub(w, scale(d, h)));
    rho * HALF.to_radians().cos() - h * HALF.to_radians().sin()
}

/// How far the ease relieves a point `y` of the pinion as drawn: the parabola about M, `PROFILE`
/// across `lengthwise` and `LENGTHWISE` along it.
fn relief(y: [f64; 3], kp: f64, kl: f64) -> f64 {
    let a = unit([1.0, 0.0, 1.0]);
    let w = sub(y, [0.0, 48.0, 0.0]);
    let s = dot(w, a);
    let r = sub(w, scale(a, s));
    0.5 * kl * s * s + 0.5 * kp * dot(r, r)
}

/// The pinion's flank — the gear's cone's envelope under the mesh — eased, read with nothing of
/// `envelope::held` or `envelope::meshing`: the cone's point at `(u, v)` and its normal `∂u × ∂v`,
/// carried by the gear's turn and back by the pinion's at roll `t`, its `v` the one where that
/// normal is square to the point's velocity (central differences, the root by secants); then
/// moved along the normal by the relief there.
struct Eased {
    a: [f64; 3],
    d: [f64; 3],
    e1: [f64; 3],
    e2: [f64; 3],
    kp: f64,
    kl: f64,
}

impl Eased {
    fn new(a: [f64; 3], d: [f64; 3], kp: f64, kl: f64) -> Eased {
        let e1 = unit(cross(d, [0.0, 1.0, 0.0]));
        Eased { a, d, e1, e2: cross(d, e1), kp, kl }
    }

    /// The cone's point at `(u, v)`, and its two derivatives.
    fn cone(&self, u: f64, v: f64) -> ([f64; 3], [f64; 3], [f64; 3]) {
        let (sh, ch) = HALF.to_radians().sin_cos();
        let radial = add(scale(self.e1, v.cos()), scale(self.e2, v.sin()));
        let g = add(scale(self.d, ch), scale(radial, sh));
        let dv = scale(add(scale(self.e1, -v.sin()), scale(self.e2, v.cos())), sh * u);
        (add(self.a, scale(g, u)), g, dv)
    }

    /// A direction of the gear as drawn in the pinion's frame at roll `t` (degrees).
    fn carried(&self, x: [f64; 3], t: f64) -> [f64; 3] {
        turned(turned(x, [0.0; 3], Z, -0.5 * t.to_radians()), [0.0; 3], X, -t.to_radians())
    }

    /// The cone's point at `(u, v)` in the pinion's frame at roll `t` (degrees).
    fn moved(&self, u: f64, v: f64, t: f64) -> [f64; 3] {
        let gear = turned(self.cone(u, v).0, [0.0; 3], Z, -0.5 * t.to_radians());
        turned(gear, PINION, X, -t.to_radians())
    }

    fn normal(&self, u: f64, v: f64, t: f64) -> [f64; 3] {
        let (_, du, dv) = self.cone(u, v);
        self.carried(unit(cross(du, dv)), t)
    }

    fn meshing(&self, u: f64, v: f64, t: f64) -> f64 {
        let k = 1e-6;
        let dt = scale(sub(self.moved(u, v, t + k), self.moved(u, v, t - k)), 0.5 / k);
        dot(self.normal(u, v, t), dt)
    }

    /// The eased flank's point at `(u, t)`, in the pinion's frame, its `v` from `v0`.
    fn point(&self, u: f64, t: f64, v0: f64) -> ([f64; 3], f64) {
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
        let y = self.moved(u, b, t);
        (add(y, scale(self.normal(u, b, t), relief(y, self.kp, self.kl))), b)
    }
}

/// How far the eased pinion's flank stands outside the gear's cone at its nearest, the pinion at
/// `phase` and the gear at `phase + lag` (degrees): the stationary point of the signed distance
/// over the flank's `(u, t)`, by Newton on central differences from `start`.  The gap, the point
/// in the fixed frame and on the pinion as drawn, and where it is.
fn gap(g: &Eased, cone: ([f64; 3], [f64; 3]), phase: f64, lag: f64, start: (f64, f64, f64))
    -> (f64, [f64; 3], [f64; 3], (f64, f64, f64)) {
    let (mut u, mut t, mut v) = start;
    let f = |u: f64, t: f64, v: f64| -> (f64, f64, [f64; 3], [f64; 3]) {
        let (x, v) = g.point(u, t, v);
        let y = pinion_pose(x, phase);
        let back = turned(y, [0.0; 3], Z, 0.5 * (phase + lag).to_radians());
        (off_cone(back, cone.0, cone.1), v, y, x)
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
    let (d, v, y, x) = f(u, t, v);
    (d, y, x, (u, t, v))
}

/// Where a sample says the two touch, read against the poses and the gear's cone apart from the
/// meshing: the pinion's point at the phase and the gear's at the phase and lag are the contact,
/// the gear's on its cone.
fn holds_together(s: &Sample, apex: [f64; 3], axis: [f64; 3]) {
    let (phase, lag) = (s.place.phase, s.place.lag);
    assert!(norm(sub(pinion_pose(s.on_pinion, phase), s.position)) < 1e-9, "{s:?}");
    let gear = turned(s.on_gear, [0.0; 3], Z, -0.5 * (phase + lag).to_radians());
    assert!(norm(sub(gear, s.position)) < 1e-9, "{s:?}");
    assert!(off_cone(s.on_gear, apex, axis).abs() < 1e-9, "{s:?}");
    assert!(s.residual < 1e-9, "{s:?}");
}

/// **An eased conjugate pair touches at a point, and the gear lags by the relief.** Uneased, the
/// pinion's flank touches the gear's along a line at every phase with no lag (`tests/meshing.rs`).
/// Eased about M, it touches at M where the pair is drawn — the relief and its slope nothing there
/// — and elsewhere at a point, the rows regular, the gear turned by the lag that closes the relief
/// along the line it would have touched on.  At each phase the lag is the turn of the gear that
/// closes the gap between the eased flank and the gear's cone — the flank's nearest approach to the
/// cone, worked out from the cone's equation, the eased flank's (the envelope's point moved along
/// its normal by the parabola) and the motions' poses alone — and the contact is where they close,
/// on the eased flank: the bearing.
#[test]
fn an_eased_conjugate_pair_touches_at_a_point_and_lags_by_the_turn_that_closes_the_relief() {
    let e = solved(&eased(&format!("{PROFILE} / 1mm"), &format!("{LENGTHWISE} / 1mm")));
    let (m, apex) = (at(&e, "M"), at(&e, "apex"));
    let axis = unit(sub(at(&e, "C"), apex));
    assert!(norm(sub(at(&e, "Q"), m)) < 1e-9, "the contact is at M where the pair is drawn");
    let t = meshing(&e.sketch).traced(8).unwrap();
    assert_eq!(t.found().count(), 9);
    let [lo, hi] = t.error_range().unwrap();
    assert!(hi - lo > 1e-3, "the gear lags by something: {lo} … {hi}");
    assert!(t.path_length() > 10.0, "the contact moves: {}", t.path_length());
    let flank = Eased::new(apex, axis, PROFILE, LENGTHWISE);
    for s in t.found() {
        assert!(!s.line, "an eased pair touches at a point: {s:?}");
        holds_together(s, apex, axis);
        if s.place.phase == 0.0 {
            assert!(s.place.lag.abs() < 1e-12, "{s:?}");
            assert!(norm(sub(s.on_pinion, m)) < 1e-9, "{s:?}");
            continue;
        }
        // the lag that closes the gap, by secants from either side of the meshing's — the eased
        // flank's point started from where the sample says the pinion's is, carried back to the
        // gear's cone
        let roll = s.place.pinion[2].to_degrees();
        let source = turned(turned(s.on_pinion, PINION, X, roll.to_radians()), [0.0; 3], Z,
                            0.5 * roll.to_radians());
        let w = sub(source, flank.a);
        let start = (dot(w, flank.d) / HALF.to_radians().cos(), roll,
                     dot(w, flank.e2).atan2(dot(w, flank.e1)));
        let close = |lag: f64| gap(&flank, (apex, axis), s.place.phase, lag, start).0;
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
        assert!((b - s.place.lag).abs() < 1e-9, "phase {}: closed at {b}, meshed at {}",
                s.place.phase, s.place.lag);
        let (_, y, x, _) = gap(&flank, (apex, axis), s.place.phase, b, start);
        assert!(norm(sub(y, s.position)) < 1e-5, "phase {}: {y:?} {:?}", s.place.phase, s.position);
        assert!(norm(sub(x, s.on_pinion)) < 1e-5, "phase {}: {x:?} {:?}", s.place.phase,
                s.on_pinion);
    }
}

/// **The lag is a number a document states, and the ease solves from it.** The forward pair's
/// lags at −15° and 15°, stated of two points held to a pair whose curvatures are unknowns,
/// solve both curvatures back: the transmission error at the two ends of the phase sizes the
/// ease, the two ends' difference saying how it divides between across and along.  And the
/// solved pair, traced, lags as stated there.
#[test]
fn lags_stated_at_two_phases_solve_the_ease() {
    let e = solved(&eased(&format!("{PROFILE} / 1mm"), &format!("{LENGTHWISE} / 1mm")));
    let m = meshing(&e.sketch);
    let t = m.trace(&m.anchor.unwrap(), &[-15.0, 15.0]);
    let ends = [t[0].unwrap(), t[1].unwrap()];
    let lags = ends.map(|s| s.place.lag);
    let unknowns = "use std\nparam kp: Scalar hint(0.001)\nparam kl: Scalar hint(0.001)\n";
    let held = |name: &str, s: &Sample| {
        let [x, y, z] = s.position;
        format!("{name} := point hint(({x}, {y}, {z}))\n\
                 {name} coincident(t == {}deg, lag == {}) contact\n", s.place.phase, s.place.lag)
    };
    let src = format!("{}{}{}", eased("kp / 1mm", "kl / 1mm").replacen("use std\n", unknowns, 1),
                      held("R", &ends[0]), held("S", &ends[1]));
    let e = solved(&src);
    let read = |name: &str| e.sketch.params[e.sketch.free_vars[name] as usize].value;
    let (kp, kl) = (read("kp"), read("kl"));
    assert!((kp - PROFILE).abs() < 1e-9, "{kp}");
    assert!((kl - LENGTHWISE).abs() < 1e-9, "{kl}");
    let m = meshing(&e.sketch);
    let t = m.trace(&m.anchor.unwrap(), &[-15.0, 15.0]);
    for (s, (phase, lag)) in t.iter().zip([(-15.0, lags[0]), (15.0, lags[1])]) {
        let s = s.unwrap();
        assert_eq!(s.place.phase, phase);
        assert!((s.place.lag - lag).abs() < 1e-9, "{s:?}");
    }
    // and R and S are where the two touch there
    for (name, s) in [("R", ends[0]), ("S", ends[1])] {
        assert!(norm(sub(at(&e, name), s.position)) < 1e-6, "{name}: {:?}", at(&e, name));
    }
}

/// `tests/meshing.rs`'s revolution's side meshed with its envelope under the mesh of skew
/// shafts, the envelope eased about a point of the gear's axis.
const SIDE_MESHING: &str = "\
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
spin := motion(about: ax)
gear_turn := motion(about: gear_ax, ratio: -0.4, advance: 3mm)
mesh := motion(spin, relative_to: gear_turn)
f1 := envelope(side, under: mesh, from: -60deg, to: 60deg)
contact := meshing(f1, side, under: mesh, from: -20deg, to: 20deg, ease: g1, along: gear_ax,
                   profile: 0.002 / 1mm, lengthwise: -0.0005 / 1mm)
p := point hint((3, 1, 8))
p coincident(t == 0deg, lag == 0) contact
";

/// An ease prints as it is written and reads back the same; one with a part missing, one about
/// no point or along no line, one whose curvature is no number per length, and one on a surface
/// traced in space, are refused saying why.
#[test]
fn an_ease_prints_back_and_refuses_what_it_cannot_read() {
    let (mut prog, errs) = crate::common::parse(SIDE_MESHING);
    assert!(errs.is_empty(), "{errs:?}");
    let first = gcs_core::program::elaborate(&prog);
    assert!(first.ok(), "{:?}", first.errors().map(|d| &d.message).collect::<Vec<_>>());
    let ease = |e: &gcs_core::program::Elaborated| e.sketch.meshings[0].ease.clone().unwrap();
    let printed = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    let said = "to: 20deg, ease: g1, along: gear_ax, profile: 0.002 / 1mm, \
                lengthwise: -0.0005 / 1mm)";
    assert!(printed.contains(said), "{printed}");
    let again = build(&printed);
    assert_eq!(ease(&again), ease(&first));
    let numbers = ease(&first).numbers.map(|n| n.c);
    assert_eq!(numbers, [0.002, -0.0005]);

    let src = eased("0.002 / 1mm", "0.0005 / 1mm");

    let parse_refused = |src: &str, want: &str| {
        let (_, errs) = crate::common::parse(src);
        assert!(errs.iter().any(|e| format!("{e:?}").contains(want)), "{want}: {errs:?}");
    };
    parse_refused(&src.replace("profile: 0.002 / 1mm, ", ""), "a meshing's ease needs");
    let refused = |src: &str, want: &str| {
        let (prog, errs) = crate::common::parse(src);
        assert!(errs.is_empty(), "{errs:?}");
        let e = gcs_core::program::elaborate(&prog);
        assert!(e.errors().any(|d| d.message.contains(want)),
                "{want}: {:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    };
    refused(&src.replace("ease: M,", "ease: lengthwise,"), "`lengthwise` is no point");
    refused(&src.replace("along: lengthwise,", "along: M,"), "`M` is no line");
    refused(&src.replace("profile: 0.002 / 1mm", "profile: 2mm"), "profile curvature");
    let traced = include_str!("../../examples/spiral_bevel/phillips_pure.sv").replace(
        "bearing := meshing(pinion_flank, gear_flank, under: mesh, from: -15deg, to: 15deg)",
        "bearing := meshing(pinion_flank, gear_flank, under: mesh, from: -15deg, to: 15deg, \
         ease: P, along: L, profile: 0.001 / 1mm, lengthwise: 0.001 / 1mm)");
    refused(&traced, "a surface traced in space, which is not eased");
}
