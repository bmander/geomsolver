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
        Member { c: add(turned(self.c, o, d, angle), shift), a: turned(self.a, [0.0; 3], d, angle),
                 w: self.w }
    }
}

/// **The involute helicoid's own equations.**  `x`, a point of the member, is on the flank its
/// rack (the plane through `p` square to `m`, sliding along `m` at `sigma`) sweeps where at some
/// roll `t` the member carries `x` onto the rack, `(y − p)·m = σt`, at a place moving along `m`
/// as fast as the rack does, `m·v(y) = σ`.  Returns that roll and the gap left in the first
/// equation there (zero on the flank), the roll solved by Newton on the second from `t0`.
fn on_helicoid(mem: &Member, p: [f64; 3], m: [f64; 3], sigma: f64, x: [f64; 3], t0: f64)
                -> (f64, f64) {
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
    sk.envelopes.iter().position(|e| e.name == name)
        .unwrap_or_else(|| panic!("no envelope `{name}`"))
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
            assert!((roll - t).abs() < 1e-9,
                    "{name}: the envelope's roll {t}, the helicoid's {roll}");
            assert!(gap.abs() < 1e-9, "{name}: {gap} off the helicoid at {:?}", r.position);
            let n = mem.turn(r.normal, t);
            assert!((dot(n, m()).abs() - 1.0).abs() < 1e-12, "{name}: {n:?}");
        }
    }
}

/// Where a sample's flanks touch, read against the poses apart from the meshing: the pinion's
/// point at the phase and the gear's at the phase and lag are the contact, and each flank's normal
/// there, carried by its member, is the other's.  With `sigmas`, each point is also on its own
/// helicoid (a gear as made, its rack sliding at `sigmas[1]`).  Returns the contact's unit normal
/// in the fixed frame.
fn holds_together(sk: &Sketch, s: &Sample, gear: &Member, sigmas: Option<[f64; 2]>) -> [f64; 3] {
    let (phase, lag) = (s.place.phase.to_radians(), s.place.lag.to_radians());
    assert!(norm(sub(PINION.pose(s.on_pinion, phase), s.position)) < 1e-9, "{s:?}");
    assert!(norm(sub(gear.pose(s.on_gear, phase + lag), s.position)) < 1e-9, "{s:?}");
    assert!(s.residual < 1e-9, "{s:?}");
    if let Some(sigmas) = sigmas {
        let (_, gap) = on_helicoid(&PINION, p(), m(), sigmas[0], s.on_pinion, phase);
        assert!(gap.abs() < 1e-9, "the pinion's point is {gap} off its helicoid: {s:?}");
        let (_, gap) = on_helicoid(gear, p(), m(), sigmas[1], s.on_gear, phase + lag);
        assert!(gap.abs() < 1e-9, "the gear's point is {gap} off its helicoid: {s:?}");
    }
    let normal = |name: &str, place: [f64; 3]| {
        let parts = Parts::of(sk, envelope(sk, name)).unwrap();
        held::read(&parts.consts(Contact::Point), &parts.values(sk), place).unwrap().normal
    };
    let n1 = PINION.turn(normal("pinion_flank", s.place.pinion), phase);
    let n2 = gear.turn(normal("gear_flank", s.place.gear), phase + lag);
    assert!((dot(n1, n2).abs() - 1.0).abs() < 1e-10, "{s:?}");
    unit(n1)
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
        let n = holds_together(&e.sketch, s, &GEAR, Some([sigma(); 2]));
        assert!((dot(n, m()).abs() - 1.0).abs() < 1e-12, "the normal is L's: {n:?}");
        let want = add(p(), scale(m(), sigma() * s.place.phase.to_radians()));
        assert!(norm(sub(s.position, want)) < 1e-9, "{s:?} against {want:?}");
    }
    let text = gcs_core::report::meshing_text(&e.sketch, 0);
    assert!(text.starts_with("bearing: point contact; transmission error"), "{text}");
}

/* -- a pair stated by its numbers ------------------------------------------------------------ */

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
        let n = holds_together(&e.sketch, s, &GEAR, Some(sigmas));
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
            normals.push(holds_together(&e.sketch, s, &gear, None));
        }
        for n in &normals {
            assert!((dot(*n, normals[0]).abs() - 1.0).abs() < 1e-10,
                    "{what}: the normal turns: {n:?}");
        }
    }
}

/* -- the screw and the meshing law (R2) ------------------------------------------------------ */

/// The axis `isa screw mesh` solves to and its pitch: the axis's point nearest the origin, its
/// direction, and the relation's own unknown.
fn screw_of(sk: &Sketch) -> ([f64; 3], [f64; 3], f64) {
    use gcs_core::constraints::{Arg, CKind};
    let c = sk.constraints.iter().find(|c| c.kind == CKind::ScrewAxis).expect("a screw");
    let (Arg::Ent(ax), Arg::Param(h)) = (&c.args[0], &c.args[2]) else { panic!("{c:?}") };
    let x = &sk.axes[ax.i()];
    let v = |q: [u32; 3]| q.map(|p| sk.params[p as usize].value);
    (v(x.a), v(x.d), sk.params[*h as usize].value)
}

/// **The screw of two turns is Ball's.**  Two members turning about skew shafts at a ratio — the
/// pinion about `a₁` through `c₁` at 1, the gear about `a₂` through `c₂` at `w` — and the gear seen
/// from the pinion: the screw `isa screw mesh` solves to crosses the shafts' common perpendicular
/// square, where a point of the perpendicular `s` from the pinion's shaft moves along `ω = ω₂ − ω₁`
/// (`s = E ω₂·ω / |ω|²`, `E` the offset), and its pitch is that point's speed along `ω` a radian of
/// turn about it.  Worked out on the perpendicular itself, apart from `envelope::screw`, and read
/// both off the drawing and off `motion::Family`.  With no offset the screw passes through the
/// apex where the shafts meet, and its pitch is zero: bevel gears roll.
#[test]
fn the_screw_of_two_turns_is_on_the_common_perpendicular_with_balls_pitch() {
    use gcs_core::motion::Family;
    // (shaft angle, offset, the gear's ratio)
    for (shaft, offset, w) in [(90.0f64, 72.0, -0.5), (60.0, 25.0, -0.4), (110.0, 0.0, -0.75)] {
        let s = shaft.to_radians();
        let (a1, c1) = ([1.0, 0.0, 0.0], [0.0, offset, 0.0]);
        let (a2, c2) = ([s.cos(), 0.0, s.sin()], [0.0; 3]);
        let v = |x: [f64; 3]| format!("({:.15}mm, {:.15}mm, {:.15}mm)", x[0], x[1], x[2]);
        let src = format!("\
unit mm
use std
g0 := point hint((0, 0, 0))
g1 := point hint((0, 0, 10))
fix({g0}) g0
fix({g1}) g1
gear_ax := line(g0, g1)
p0 := point hint((0, {offset}, 0))
p1 := point hint((10, {offset}, 0))
fix({p0}) p0
fix({p1}) p1
pinion_ax := line(p0, p1)
gear_turn := motion(about: gear_ax, ratio: {w})
pinion_turn := motion(about: pinion_ax)
mesh := motion(gear_turn, relative_to: pinion_turn)
isa := axis hint(dir: (-0.9, 0.1, -0.4))
isa screw mesh
", g0 = v(c2), g1 = v(add(c2, scale(a2, 10.0))), p0 = v(c1), p1 = v(add(c1, scale(a1, 10.0))));
        let e = solved(&src);
        let (point, dir, pitch) = screw_of(&e.sketch);
        // on the perpendicular, the pinion's foot `c₁`, toward the gear's `c₂`
        let (w1, w2) = (a1, scale(a2, w));
        let omega = sub(w2, w1);
        let ww = dot(omega, omega);
        let s_at = offset * dot(w2, omega) / ww;
        let e_dir = if offset > 0.0 { unit(sub(c2, c1)) } else { [0.0, -1.0, 0.0] };
        let foot = add(c1, scale(e_dir, s_at));
        let u = sub(scale(w2, s_at - offset), scale(w1, s_at));
        let want_pitch = dot(cross(u, e_dir), omega) / ww;
        let what = format!("{shaft}° {offset} {w}");
        assert!(norm(sub(point, foot)) < 1e-9, "{what}: {point:?} against {foot:?}");
        assert!((dot(dir, unit(omega)).abs() - 1.0).abs() < 1e-12, "{what}: {dir:?}");
        assert!(dot(dir, e_dir).abs() < 1e-12, "{what}: square to the perpendicular");
        assert!((pitch - want_pitch).abs() < 1e-9, "{what}: pitch {pitch} against {want_pitch}");
        if offset == 0.0 {
            assert!(norm(point) < 1e-9 && pitch.abs() < 1e-12,
                    "{what}: through the apex, no pitch");
        }
        // and the velocity of the point on the axis is along it, by the pitch
        let speed = sub(scale(cross(w2, sub(foot, c2)), 1.0), cross(w1, sub(foot, c1)));
        assert!(norm(sub(speed, scale(omega, pitch))) < 1e-9, "{what}: {speed:?}");
        let mesh = e.sketch.motions.iter().position(|m| m.name == "mesh").unwrap();
        let isa = Family::read(&e.sketch, mesh).unwrap().instantaneous().unwrap();
        assert!(norm(sub(isa.point, foot)) < 1e-9 && (isa.pitch - want_pitch).abs() < 1e-9,
                "{what}: {isa:?}");
    }
}

/// **`L reciprocal mesh` is the meshing law.**  `phillips.sv` states `L`'s pressure angle and that
/// it is reciprocal to the mesh: the helix angle it solves to is the closed form's, and along it
/// the two members move as fast at every point — checked from the members' turns alone.  The
/// rack's slide is reciprocal to each member's cutting in turn (the document's claims hold), and
/// the report says the mesh's screw.
#[test]
fn l_reciprocal_to_the_mesh_is_where_the_members_move_as_fast_along_it() {
    use gcs_core::diagnose::{diagnose, DiagnoseOptions};
    let e = solved(document());
    let sk = &e.sketch;
    let at = |n: &str| sk.world_point(e.map.ent_named(n).unwrap().i());
    let d = unit(sub(at("ahead"), at("P")));
    assert!(norm(sub(d, m())) < 1e-10, "{d:?} against {:?}", m());
    for lambda in [-30.0, 0.0, 12.5, 40.0] {
        let x = add(p(), scale(d, lambda));
        let (v1, v2) = (PINION.speed(x, d), GEAR.speed(x, d));
        assert!((v1 - v2).abs() < 1e-9, "{lambda}: {v1} {v2}");
        assert!((v1 - sigma()).abs() < 1e-9, "{lambda}: {v1} against σ {}", sigma());
    }
    let (point, _, pitch) = screw_of(sk);
    assert!(norm(sub(point, [0.0, 57.6, 0.0])) < 1e-9 && (pitch + 28.8).abs() < 1e-9,
            "{point:?} {pitch}");
    let mut sk = e.sketch.clone();
    let dg = diagnose(&mut sk, DiagnoseOptions::default());
    assert!(dg.claims_violated.is_empty(), "{:?}", dg.claims_violated);
    let said = gcs_core::report::screws_text(&e.sketch);
    assert_eq!(said.len(), 1);
    assert!(said[0].starts_with("mesh: screws about the axis through (0, 57.6, 0)")
            && said[0].ends_with("pitch -28.8 a radian"), "{said:?}");
}

/// `reciprocal` and `screw` print as they are written and read back the same, a pinned pitch with
/// them; a twist that is not constant — a motion relative to a relative one — is refused where it
/// is read.
#[test]
fn reciprocal_and_screw_print_back_and_refuse_what_has_no_constant_screw() {
    use gcs_core::constraints::CKind;
    let src = "\
unit mm
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
P := point hint((0, 48, 0))
fix((0mm, 48mm, 0mm)) P
ahead := point hint((7, 51, -7))
L := line(P, ahead)
P distance(10mm) ahead
P distance(3.42mm, along: gear_ax) ahead
L reciprocal mesh
claim L reciprocal gear_turn
isa := axis hint(dir: (-0.9, 0, -0.45))
isa screw(pitch == -28.8mm) mesh
";
    let (mut prog, errs) = crate::common::parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let first = gcs_core::program::elaborate(&prog);
    assert!(first.ok(), "{:?}", first.errors().map(|d| &d.message).collect::<Vec<_>>());
    let printed = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    let stated = ["L reciprocal mesh", "isa screw(pitch == -28.8mm) mesh",
                  "claim L reciprocal gear_turn"];
    for said in stated {
        assert!(printed.contains(said), "{said}: {printed}");
    }
    let again = solved(&printed);
    let kinds = |sk: &Sketch| sk.constraints.iter().map(|c| c.kind)
        .filter(|k| matches!(k, CKind::Reciprocal | CKind::ScrewAxis)).collect::<Vec<_>>();
    assert_eq!(kinds(&again.sketch), kinds(&first.sketch));
    assert_eq!(kinds(&again.sketch).len(), 3);
    // a relative of a relative has no constant twist, and is refused where it is read
    let nested = src.replace("isa screw(pitch == -28.8mm) mesh",
        "again := motion(mesh, relative_to: gear_turn)\nisa screw again");
    let (prog, errs) = crate::common::parse(&nested);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.errors().any(|d| d.message.contains("whose twist changes as it runs")),
            "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
}

/* -- the pure form (R6) ------------------------------------------------------------------------ */

/// `spiral_bevel/phillips_pure.sv`: the same pair with each flank the contact element as its member
/// sees it, a surface traced in space, and no rack anywhere.
fn pure() -> &'static str {
    include_str!("../../examples/spiral_bevel/phillips_pure.sv")
}

fn traced_index(e: &gcs_core::program::Elaborated, name: &str) -> usize {
    let r = e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`"));
    assert_eq!(r.kind, gcs_core::model::EntKind::Traced, "`{name}` is traced in space");
    r.i()
}

/// **Each pure-form flank is the rack's helicoid.**  Read at places across it, the flank traced
/// from the contact alone is on the member's involute helicoid by its own equations, and on the
/// envelope `phillips.sv` sweeps with its rack — at the element's phase, the rack's place through
/// that point is on the envelope there, to 1e-9 — and its normal, carried by its member to the
/// place's phase, is `L`'s.
#[test]
fn each_pure_form_flank_is_the_racks_envelope() {
    use gcs_core::traced::{reading, Need};
    let traced = solved(pure());
    let racked = solved(document());
    let mut rng = Rng::new(6);
    for (name, mem) in [("pinion_flank", PINION), ("gear_flank", GEAR)] {
        let i = traced_index(&traced, name);
        let env = envelope(&racked.sketch, name);
        let parts = Parts::of(&racked.sketch, env).unwrap();
        let k = parts.consts(Contact::Point);
        let values = parts.values(&racked.sketch);
        for _ in 0..12 {
            let (phi, s) = (rng.uniform(-35.0, 35.0), rng.uniform(-12.0, 12.0));
            let r = reading(&traced.sketch, i, [phi, s], Need::Tangent).unwrap();
            let t = phi.to_radians();
            let (roll, gap) = on_helicoid(&mem, p(), m(), sigma(), r.position, t);
            assert!((roll - t).abs() < 1e-9 && gap.abs() < 1e-9,
                    "{name} ({phi}, {s}): {roll} {gap}");
            let n = mem.turn(r.normal.unwrap(), t);
            assert!((dot(n, m()).abs() - 1.0).abs() < 1e-10, "{name}: {n:?}");
            // the rack's envelope at the roll the element is at: the rack's place there is
            // affine in the plane's two lengths, so three readings find it
            let at = |u: f64, v: f64| held::read(&k, &values, [u, v, t]).unwrap().position;
            let o = at(0.0, 0.0);
            let (a, b, c) = (o, sub(at(1.0, 0.0), o), sub(at(0.0, 1.0), o));
            let w = sub(r.position, a);
            let (bb, bc, cc, wb, wc) = (dot(b, b), dot(b, c), dot(c, c), dot(w, b), dot(w, c));
            let det = bb * cc - bc * bc;
            let place = [(cc * wb - bc * wc) / det, (bb * wc - bc * wb) / det, t];
            let on = held::read(&k, &values, place).unwrap();
            assert!(norm(sub(on.position, r.position)) < 1e-9 && on.normal_velocity.abs() < 1e-9,
                    "{name} ({phi}, {s}): the rack's envelope there is {:?}, the traced flank {:?}",
                    on.position, r.position);
        }
    }
}

/// **Phillips' pair in the pure form meshes as the rack-cut one does**: a point contact on `L` at
/// every phase, with no lag, the common normal `L`'s, each flank's point there its member's own
/// point carried to the phase — the traced flanks read through `meshing` as the envelopes are.
#[test]
fn phillips_pair_in_the_pure_form_touches_along_l_with_no_lag() {
    use gcs_core::traced::{reading, Need};
    let e = solved(pure());
    let sk = &e.sketch;
    let t = traced(sk, "bearing", 24);
    assert_eq!(t.found().count(), 25);
    let (pi, gi) = (traced_index(&e, "pinion_flank"), traced_index(&e, "gear_flank"));
    for s in t.found() {
        assert!(s.place.lag.abs() < 1e-10, "{s:?}");
        assert!(!s.line, "{s:?}");
        let (phase, lag) = (s.place.phase.to_radians(), s.place.lag.to_radians());
        assert!(norm(sub(PINION.pose(s.on_pinion, phase), s.position)) < 1e-9, "{s:?}");
        assert!(norm(sub(GEAR.pose(s.on_gear, phase + lag), s.position)) < 1e-9, "{s:?}");
        let want = add(p(), scale(m(), sigma() * phase));
        assert!(norm(sub(s.position, want)) < 1e-9, "{s:?} against {want:?}");
        let n1 = reading(sk, pi, [s.place.pinion[0], s.place.pinion[1]], Need::Tangent).unwrap();
        let n2 = reading(sk, gi, [s.place.gear[0], s.place.gear[1]], Need::Tangent).unwrap();
        let n1 = PINION.turn(n1.normal.unwrap(), phase);
        let n2 = GEAR.turn(n2.normal.unwrap(), phase + lag);
        for n in [n1, n2] {
            assert!((dot(n, m()).abs() - 1.0).abs() < 1e-10, "{n1:?} {n2:?}");
        }
    }
    let text = gcs_core::report::meshing_text(sk, 0);
    assert!(text.starts_with("bearing: point contact; transmission error"), "{text}");
}

/// The Jacobian of a point on a meshing of two traced flanks against finite differences: at the
/// solve and knocked off it, every column — the phase, the lag, both places, the bodies' geometry
/// and the mesh's axes.
#[test]
fn a_meshing_of_traced_flanks_agrees_with_finite_differences() {
    use gcs_core::constraints::CKind;
    use gcs_core::fdcheck::check_constraint;
    let mut sk = solved(pure()).sketch;
    let mut rng = Rng::new(62);
    for round in 0..3 {
        if round > 0 {
            for p in sk.params.iter_mut().filter(|p| !p.fixed) {
                p.value += rng.uniform(-0.01, 0.01) * (1.0 + p.value.abs().min(10.0));
            }
        }
        let c = sk.constraints.iter().find(|c| c.kind == CKind::PointOnMeshing).unwrap();
        let err = check_constraint(&sk, c, 1e-6, 1e-6)
            .unwrap_or_else(|e| panic!("round {round}: {e}"));
        assert!(err.is_finite());
    }
}
