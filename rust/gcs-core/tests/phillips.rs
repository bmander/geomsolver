//! **Phillips' spatial involute gearing** (#326, step 4a of #283): a pair on skew shafts that is
//! one line of action `L`.  Each member's flank is the involute helicoid a plane square to `L`
//! sweeps sliding along it at the members' common speed along it, seen from the member; the two
//! touch at a point of `L`, with `L` for their normal, at the ratio's roll.  Each is held here to
//! what is worked out apart from `envelope::held` and `envelope::meshing`: the helicoid's own
//! equations (a point of member `i`'s flank is one the member carries, at some roll `t`, onto the
//! rack at `t` where its speed along `L` is the rack's) and the members' poses.
use crate::common::build;
use gcs_core::envelope::held::{self, Contact, Parts};
use gcs_core::envelope::meshing::{Meshing, Sample, Trace};
use gcs_core::model::Sketch;
use gcs_core::rng::Rng;
use gcs_core::solve;
use gcs_core::space::{add, cross, dot, norm, scale, sub, unit};

/// `spiral_bevel/phillips.sv`'s pair: the gear about z, the pinion about x through `(0, E, 0)`,
/// the gear at half the pinion's speed the other way; `L` through the pitch point `P` on the
/// common perpendicular, leaning `ALPHA` from the pitch plane.
const RG: f64 = 48.0;
const RP: f64 = 24.0;
const Q: f64 = 0.5;
const ALPHA: f64 = 20.0;

/// The helix angle where the members' speeds along `L` match, `L`'s direction, and that speed per
/// radian of the pinion's phase.
fn helix() -> f64 {
    (-Q * RG / RP).atan()
}
fn m() -> [f64; 3] {
    let (a, b) = (ALPHA.to_radians(), helix());
    [a.cos() * b.cos(), a.sin(), a.cos() * b.sin()]
}
fn sigma() -> f64 {
    Q * RG * ALPHA.to_radians().cos() * helix().cos()
}
fn p() -> [f64; 3] {
    [0.0, RG, 0.0]
}

/// A member: a turn about the line through `c` along the unit `a`, `w` radians per radian of the
/// pinion's phase.
#[derive(Clone, Copy, Debug)]
struct Member {
    c: [f64; 3],
    a: [f64; 3],
    w: f64,
}

const PINION: Member = Member { c: [0.0, RG + RP, 0.0], a: [1.0, 0.0, 0.0], w: 1.0 };
const GEAR: Member = Member { c: [0.0; 3], a: [0.0, 0.0, 1.0], w: -Q };

/// `x` turned `angle` about the line through `o` along the unit `d`.
fn turned(x: [f64; 3], o: [f64; 3], d: [f64; 3], angle: f64) -> [f64; 3] {
    let w = sub(x, o);
    let (s, c) = angle.sin_cos();
    add(o, add(add(scale(w, c), scale(cross(d, w), s)), scale(d, dot(d, w) * (1.0 - c))))
}

impl Member {
    /// Where the member carries its own point `x` at roll `t` (radians of the pinion's phase).
    fn pose(&self, x: [f64; 3], t: f64) -> [f64; 3] {
        turned(x, self.c, self.a, self.w * t)
    }
    /// A direction carried likewise.
    fn turn(&self, d: [f64; 3], t: f64) -> [f64; 3] {
        turned(d, [0.0; 3], self.a, self.w * t)
    }
    /// The member's speed along `m` at the fixed point `y`, per radian of phase.
    fn speed(&self, y: [f64; 3], m: [f64; 3]) -> f64 {
        dot(m, scale(cross(self.a, sub(y, self.c)), self.w))
    }
    /// The same member moved rigidly: its axis turned `angle` about the line through `o` along
    /// `d`, then shifted by `shift`.
    fn moved(&self, o: [f64; 3], d: [f64; 3], angle: f64, shift: [f64; 3]) -> Member {
        Member { c: add(turned(self.c, o, d, angle), shift), a: turned(self.a, [0.0; 3], d, angle), w: self.w }
    }
}

/// **The involute helicoid's own equations.**  `x`, a point of the member, is on the flank its
/// rack (the plane through `p` square to `m`, sliding along `m` at `sigma`) sweeps where at some
/// roll `t` the member carries `x` onto the rack, `(y − p)·m = σt`, at a place moving along `m`
/// as fast as the rack does, `m·v(y) = σ`.  Returns that roll and the gap left in the first
/// equation there (zero on the flank), the roll solved by Newton on the second from `t0`.
fn on_helicoid(mem: &Member, p: [f64; 3], m: [f64; 3], sigma: f64, x: [f64; 3], t0: f64) -> (f64, f64) {
    let speed = |t: f64| mem.speed(mem.pose(x, t), m) - sigma;
    let mut t = t0;
    for _ in 0..60 {
        let h = 1e-6;
        let d = (speed(t + h) - speed(t - h)) / (2.0 * h);
        let step = speed(t) / d;
        t -= step;
        if step.abs() < 1e-15 {
            break;
        }
    }
    (t, dot(sub(mem.pose(x, t), p), m) - sigma * t)
}

fn solved(src: &str) -> gcs_core::program::Elaborated {
    let mut e = build(src);
    let r = solve::solve(&mut e.sketch, Default::default());
    assert!(r.success, "{r:?}");
    e
}

fn envelope(sk: &Sketch, name: &str) -> usize {
    sk.envelopes.iter().position(|e| e.name == name).unwrap_or_else(|| panic!("no envelope `{name}`"))
}

fn traced(sk: &Sketch, name: &str, n: usize) -> Trace {
    let i = sk.meshings.iter().position(|m| m.name == name)
        .unwrap_or_else(|| panic!("no meshing `{name}`"));
    Meshing::named(sk, i).unwrap().traced(n).unwrap()
}

/// The document: `spiral_bevel/phillips.sv` itself.
fn document() -> &'static str {
    include_str!("../../examples/spiral_bevel/phillips.sv")
}

/// A point of an envelope at a place: its roll and one direction of the plane given, the other
/// solved by secants until the meshing equation is zero.
fn on_envelope(k: &[f64], values: &[f64], mut place: [f64; 3], free: usize) -> held::Reading {
    let g = |place: [f64; 3]| held::read(k, values, place).unwrap().normal_velocity;
    for _ in 0..60 {
        let h = 1e-4;
        let mut up = place;
        up[free] += h;
        let step = g(place) * h / (g(up) - g(place));
        place[free] -= step;
        if step.abs() < 1e-14 {
            break;
        }
    }
    held::read(k, values, place).unwrap()
}

/// **The plane's envelope is the involute helicoid.**  Over each member's flank, points of the
/// envelope the solve reads — a roll and a place on the rack drawn at random, the meshing equation
/// solved for the rest — are on the helicoid by its own equations, and the envelope's normal there,
/// carried by the member to that roll, is `L`'s: a developable, its normal constant along each
/// generator, every generator the rack's at some roll.
#[test]
fn the_racks_envelope_is_each_members_involute_helicoid() {
    let e = solved(document());
    let sk = &e.sketch;
    let mut rng = Rng::new(326);
    for (name, mem) in [("pinion_flank", PINION), ("gear_flank", GEAR)] {
        let i = envelope(sk, name);
        let parts = Parts::of(sk, i).unwrap();
        assert_eq!(parts.source, held::Source::Plane, "{name}");
        let k = parts.consts(Contact::Point);
        let values = parts.values(sk);
        for _ in 0..40 {
            let t = rng.uniform(-0.6, 0.6);
            let place = [rng.uniform(-8.0, 8.0), rng.uniform(-8.0, 8.0), t];
            let r = on_envelope(&k, &values, place, 0);
            assert!(r.normal_velocity.abs() < 1e-10, "{name}: {r:?}");
            let (roll, gap) = on_helicoid(&mem, p(), m(), sigma(), r.position, t);
            assert!((roll - t).abs() < 1e-9, "{name}: the envelope's roll {t}, the helicoid's {roll}");
            assert!(gap.abs() < 1e-9, "{name}: {gap} off the helicoid at {:?}", r.position);
            let n = mem.turn(r.normal, t);
            assert!((dot(n, m()).abs() - 1.0).abs() < 1e-12, "{name}: {n:?}");
        }
    }
}

/// Where a sample's flanks touch, read against the helicoids' equations and the poses apart from
/// the meshing: the pinion's point at the phase and the gear's at the phase and lag are the
/// contact, each on its own helicoid (`sigma` each), and each flank's normal there, carried by its
/// member, is `m`.  Returns the contact's normal in the fixed frame.
fn holds_together(sk: &Sketch, s: &Sample, pinion: &Member, gear: &Member, m: [f64; 3],
                  sigmas: [f64; 2]) -> [f64; 3] {
    let (phase, lag) = (s.place.phase.to_radians(), s.place.lag.to_radians());
    assert!(norm(sub(pinion.pose(s.on_pinion, phase), s.position)) < 1e-9, "{s:?}");
    assert!(norm(sub(gear.pose(s.on_gear, phase + lag), s.position)) < 1e-9, "{s:?}");
    assert!(s.residual < 1e-9, "{s:?}");
    let (_, gap) = on_helicoid(pinion, p(), m, sigmas[0], s.on_pinion, phase);
    assert!(gap.abs() < 1e-9, "the pinion's point is {gap} off its helicoid: {s:?}");
    let (_, gap) = on_helicoid(gear, p(), m, sigmas[1], s.on_gear, phase + lag);
    assert!(gap.abs() < 1e-9, "the gear's point is {gap} off its helicoid: {s:?}");
    let normal = |name: &str, place: [f64; 3]| {
        let parts = Parts::of(sk, envelope(sk, name)).unwrap();
        held::read(&parts.consts(Contact::Point), &parts.values(sk), place).unwrap().normal
    };
    let n1 = pinion.turn(normal("pinion_flank", s.place.pinion), phase);
    let n2 = gear.turn(normal("gear_flank", s.place.gear), phase + lag);
    assert!((dot(n1, n2).abs() - 1.0).abs() < 1e-10, "{s:?}");
    n1
}

/// **Phillips' pair transmits without error, touching at a point of `L`.**  At every phase of the
/// trace the gear is where the ratio puts it — lag below 1e-10° — the two touch at a point (the
/// rows regular: each member's plane of action is its own), that point is where the rack meets
/// `L`, `P + σφ·m`, so the path of contact is `L` itself, and the common normal is `L`'s.
#[test]
fn phillips_pair_touches_at_a_point_running_along_l_with_no_lag() {
    let e = solved(document());
    let t = traced(&e.sketch, "bearing", 24);
    assert_eq!(t.found().count(), 25);
    for s in t.found() {
        assert!(s.place.lag.abs() < 1e-10, "{s:?}");
        assert!(!s.line, "Phillips' pair touches at a point: {s:?}");
        let n = holds_together(&e.sketch, s, &PINION, &GEAR, m(), [sigma(); 2]);
        assert!((dot(n, m()).abs() - 1.0).abs() < 1e-12, "the normal is L's: {n:?}");
        let want = add(p(), scale(m(), sigma() * s.place.phase.to_radians()));
        assert!(norm(sub(s.position, want)) < 1e-9, "{s:?} against {want:?}");
    }
    let text = gcs_core::report::meshing_text(&e.sketch, 0);
    assert!(text.starts_with("bearing: point contact; transmission error"), "{text}");
}

/* -- a pair stated by its numbers --------------------------------------------------------------- */

/// A pair of the same shape stated point by point: the pinion's rack sliding at `sigmas[0]`, the
/// gear's at `sigmas[1]`, and the gear — its axis and its rack, so its flank — moved rigidly by
/// `moved` (its axis turned about a line, then shifted), the pinion and its rack as drawn.
fn pair(sigmas: [f64; 2], moved: impl Fn([f64; 3]) -> [f64; 3]) -> String {
    let (pt, m) = (p(), m());
    let ahead = add(pt, scale(m, 10.0));
    let (gp, ga) = (moved(pt), moved(ahead));
    let (g0, g1) = (moved([0.0; 3]), moved([0.0, 0.0, 10.0]));
    let v = |x: [f64; 3]| format!("({:.15}mm, {:.15}mm, {:.15}mm)", x[0], x[1], x[2]);
    let h = |x: [f64; 3]| format!("({:.6}, {:.6}, {:.6})", x[0], x[1], x[2]);
    format!("\
unit mm
use std
g0 := point hint({hg0})
g1 := point hint({hg1})
fix({g0}) g0
fix({g1}) g1
gear_ax := line(g0, g1)
p0 := point hint((0, 72, 0))
p1 := point hint((10, 72, 0))
fix((0mm, 72mm, 0mm)) p0
fix((10mm, 72mm, 0mm)) p1
pinion_ax := line(p0, p1)
gear_turn := motion(about: gear_ax, ratio: -{Q})
pinion_turn := motion(about: pinion_ax)
mesh := motion(gear_turn, relative_to: pinion_turn)
P := point hint({hp})
fix({p}) P
ahead := point hint({ha})
fix({a}) ahead
L := line(P, ahead)
GP := point hint({hgp})
fix({gp}) GP
gahead := point hint({hga})
fix({ga}) gahead
GL := line(GP, gahead)
rack := std.HalfSpace(P, L, d: 0mm)
gear_rack := std.HalfSpace(GP, GL, d: 0mm)
slide := motion(along: L, advance: {s1}mm)
gear_slide := motion(along: GL, advance: {s2}mm)
pinion_cutting := motion(slide, relative_to: pinion_turn)
gear_cutting := motion(gear_slide, relative_to: gear_turn)
pinion_flank := envelope(rack, under: pinion_cutting, from: -40deg, to: 40deg)
gear_flank := envelope(gear_rack, under: gear_cutting, from: -40deg, to: 40deg)
bearing := meshing(pinion_flank, gear_flank, under: mesh, from: -15deg, to: 15deg)
Q := point hint({hp})
Q coincident(t == 0deg) bearing
",
        hg0 = h(g0), hg1 = h(g1), g0 = v(g0), g1 = v(g1), hp = h(pt), p = v(pt), ha = h(ahead),
        a = v(ahead), hgp = h(gp), gp = v(gp), hga = h(ga), ga = v(ga),
        s1 = std::f64::consts::TAU * sigmas[0], s2 = std::f64::consts::TAU * sigmas[1])
}

/// **A pair whose speeds along `L` differ transmits at another ratio.**  The gear's rack slides 2 %
/// faster than the pinion's, so the gear's flank is the helicoid of a plane of action parallel to
/// its own and off `L`.  The contact is a point all the same, the normal still `L`'s, and its path
/// a line along `L`, but the gear lags by what closes the gap between the racks' places: at phase
/// `φ` the pinion touches the rack `σφ` along `L` and the gear the one `σ'(φ + δ)` along, so
/// `δ = (σ/σ' − 1)·φ`.  Each sample is held to that, and to both helicoids' own equations at the
/// poses (`holds_together`), with nothing read of the meshing but the place it reports.
#[test]
fn a_pair_whose_speeds_along_l_differ_lags_by_the_racks_gap() {
    let ratio = 1.02;
    let sigmas = [sigma(), sigma() * ratio];
    let e = solved(&pair(sigmas, |x| x));
    let t = traced(&e.sketch, "bearing", 24);
    assert_eq!(t.found().count(), 25);
    let mut path = Vec::new();
    for s in t.found() {
        assert!(!s.line, "{s:?}");
        let want = (1.0 / ratio - 1.0) * s.place.phase;
        assert!((s.place.lag - want).abs() < 1e-9 * (1.0 + want.abs()), "{s:?} against {want}");
        if s.place.phase != 0.0 {
            assert!(s.place.lag.abs() > 1e-3, "the lag is no rounding: {s:?}");
        }
        let n = holds_together(&e.sketch, s, &PINION, &GEAR, m(), sigmas);
        assert!((dot(n, m()).abs() - 1.0).abs() < 1e-12, "{n:?}");
        // the pinion touches the rack at its place at the phase
        let along = dot(sub(s.position, p()), m());
        assert!((along - sigma() * s.place.phase.to_radians()).abs() < 1e-9, "{s:?}");
        path.push(s.position);
    }
    // a straight path along `L`
    for x in &path {
        let w = sub(*x, path[0]);
        assert!(norm(sub(w, scale(m(), dot(w, m())))) < 1e-9, "{x:?}");
    }
}

/// **Phillips' pair keeps its ratio when it is misaligned** (#326 R1: checked, not assumed).  The
/// flanks made, the gear is moved — half a millimetre further off along the common perpendicular,
/// or its shaft turned half a degree about it — and meshed with the pinion as made.  The gear's
/// plane of action moves with it, parallel to itself under the shift: the contact's normal is
/// still `L`'s and both members still move as fast along it, so the lag is a constant — the gear
/// set back a fixed angle — and the transmission error zero.  Turned, the normal is wherever the
/// members' cones of normals meet, the same at every phase, and again both move as fast along it:
/// a constant lag.  This is Phillips' claim, crossed helical gears' insensitivity generalised, and
/// it holds — so the meshing is held to a constant lag on a pair whose flanks were made for
/// another placing.
#[test]
fn phillips_pair_keeps_its_ratio_when_the_offset_or_the_shaft_angle_is_moved() {
    let y = [0.0, 1.0, 0.0];
    let cases: [(&str, Member, Box<dyn Fn([f64; 3]) -> [f64; 3]>); 2] = [
        ("offset", GEAR.moved([0.0; 3], y, 0.0, [0.0, -0.5, 0.0]),
         Box::new(|x| add(x, [0.0, -0.5, 0.0]))),
        ("shaft angle", GEAR.moved([0.0; 3], y, 0.5f64.to_radians(), [0.0; 3]),
         Box::new(|x| turned(x, [0.0; 3], y, 0.5f64.to_radians()))),
    ];
    for (what, gear, moved) in cases {
        let e = solved(&pair([sigma(); 2], moved));
        let t = traced(&e.sketch, "bearing", 24);
        assert_eq!(t.found().count(), 25, "{what}");
        let lags: Vec<f64> = t.found().map(|s| s.place.lag).collect();
        let [lo, hi] = t.error_range().unwrap();
        assert!(hi - lo < 1e-9, "{what}: the lag runs {lo}° … {hi}°: {lags:?}");
        assert!(lags[0].abs() > 1e-6, "{what}: the gear is set back: {lags:?}");
        let mut normals = Vec::new();
        for s in t.found() {
            assert!(!s.line, "{what}: {s:?}");
            normals.push(holds_together_moved(&e.sketch, s, &gear));
        }
        for n in &normals {
            assert!((dot(*n, normals[0]).abs() - 1.0).abs() < 1e-10, "{what}: the normal turns: {n:?}");
        }
    }
}

/// `holds_together` for a moved gear, whose helicoid's equations are its own as made: the poses
/// alone, each flank's normal carried by its member, and the two agreeing.
fn holds_together_moved(sk: &Sketch, s: &Sample, gear: &Member) -> [f64; 3] {
    let (phase, lag) = (s.place.phase.to_radians(), s.place.lag.to_radians());
    assert!(norm(sub(PINION.pose(s.on_pinion, phase), s.position)) < 1e-9, "{s:?}");
    assert!(norm(sub(gear.pose(s.on_gear, phase + lag), s.position)) < 1e-9, "{s:?}");
    assert!(s.residual < 1e-9, "{s:?}");
    let normal = |name: &str, place: [f64; 3]| {
        let parts = Parts::of(sk, envelope(sk, name)).unwrap();
        held::read(&parts.consts(Contact::Point), &parts.values(sk), place).unwrap().normal
    };
    let n1 = PINION.turn(normal("pinion_flank", s.place.pinion), phase);
    let n2 = gear.turn(normal("gear_flank", s.place.gear), phase + lag);
    assert!((dot(n1, n2).abs() - 1.0).abs() < 1e-10, "{s:?}");
    unit(n1)
}
