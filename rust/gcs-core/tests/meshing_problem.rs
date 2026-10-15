//! The hypoid pair stated as its meshing problem (#345 S4): `spiral_bevel/meshing_problem.sv`.
//! `formate.sv`'s pair — the gear formed by a face mill, the pinion its blank less the gear's
//! tooth swept under the mesh, so conjugate to the mill's cones — with the gear's concave flank
//! eased off the inner cone: the document states the contact (where it is drawn, the pressure
//! angle of its normal, the angle of its path, the lag at the start of the phase, no undercut)
//! and the ease's two curvatures are found, and with them the gear's concave blade that cuts it.
//! Here the document solves from its seeds and from others, its numbers are recorded, each
//! stated condition is read again on the solved pair apart from the relation that states it, and
//! the blade the gear is cut by is held to the ease the meshing reads.
use crate::common::{at, ends, ent};
use crate::meshing::turned;
use crate::regular::{admitted, reading};
use gcs_core::envelope::meshing::{Meshing, Sample};
use gcs_core::program::Elaborated;
use gcs_core::space::{add, cross, dot, norm, scale, sub, unit};

type V = [f64; 3];

/// The document, its ease seeded at `profile` and `lengthwise` (texts).
fn pair(profile: &str, lengthwise: &str) -> Elaborated {
    let call = "profile_ease: hint(-0.01), lengthwise_ease: hint(-0.001)";
    let text = fixtures::gear::meshing_problem_source();
    assert!(text.contains(call), "the pair is called with its ease seeded");
    fixtures::gear::read(&text.replace(call, &format!("profile_ease: hint({profile}), \
        lengthwise_ease: hint({lengthwise})")))
}

/// The pair solved, its ease seeded at what the document's own seeds solve it to (`PROFILE`,
/// `LENGTHWISE`): the same pair, found without the search from the document's seeds.
fn solved() -> Elaborated {
    pair(&PROFILE.to_string(), &LENGTHWISE.to_string())
}

/// An unknown number as solved.
fn number(e: &Elaborated, name: &str) -> f64 {
    e.sketch.params[e.sketch.free_vars[name] as usize].value
}

/// A line's direction, from its start to its end.
fn direction(e: &Elaborated, name: &str) -> V {
    let (a, b) = ends(&e.sketch, ent(e, name));
    unit(sub(b, a))
}

/// The contact traced at `phases`, every one found.
fn trace(e: &Elaborated, phases: &[f64]) -> Vec<Sample> {
    let m = Meshing::named(&e.sketch, 0).unwrap();
    m.trace(&m.anchor.expect("a point held to the contact"), phases).into_iter()
        .map(|s| s.expect("traced at every phase"))
        .collect()
}

/// The ease, per millimetre, as solved: the profile's curvature and the lengthwise one.
const PROFILE: f64 = -0.018004871;
const LENGTHWISE: f64 = -0.00285435006;

/// The document solves from its own seeds — and from seeds on either side of what it comes to,
/// to the same ease. Each is a number the drawing found, no `param` a designer tunes.
#[test]
fn the_pair_solves_to_its_ease_from_its_seeds() {
    for seeds in [("-0.01", "-0.001"), ("-0.03", "-0.005"), ("-0.005", "-0.0005")] {
        let e = pair(seeds.0, seeds.1);
        let (kp, kl) = (number(&e, "pair.profile_ease"), number(&e, "pair.lengthwise_ease"));
        eprintln!("{seeds:?}: profile {kp:.9}, lengthwise {kl:.11} per mm");
        assert!((kp - PROFILE).abs() < 1e-8 && (kl - LENGTHWISE).abs() < 1e-10, "{seeds:?}");
    }
}

/// The stated conditions, read again on the solved pair apart from the relations that state
/// them: the contact at the phase it is drawn is T, where the face mill's inner cone leans its
/// normal the stated pressure angle from the plane square to the mill's axis; the path of
/// contact there, by differences of the traced contact, makes the stated angle with the trace's
/// heading; the pinion lags the stated amount at the start of the phase; the pair touches at a
/// point at every phase; and the pinion's flank does not fold within its blank — the least area
/// factor the bound reads, the one the sweep's admission samples of the pinion as it is cut.
#[test]
fn the_stated_conditions_read_again_on_the_solved_pair() {
    let e = solved();
    let t = at(&e, "pair.mean.T");
    let ax = direction(&e, "pair.mill.ax");
    let phases: Vec<f64> = (0..=12).map(|k| -7.5 + 15.0 * k as f64 / 12.0).collect();
    let samples = trace(&e, &phases);

    // the contact at the drawn phase, and its normal: the cone's at T, toward the mill's axis
    let drawn = &samples[6];
    assert!(drawn.place.phase.abs() < 1e-12);
    assert!(norm(sub(drawn.position, t)) < 1e-9, "{:?} {t:?}", drawn.position);
    assert!(drawn.place.lag.abs() < 1e-9);
    let normal = unit(sub(at(&e, "pair.mean.to_axis"), t));
    let pressure = dot(normal, ax).abs().asin().to_degrees();
    eprintln!("pressure angle at T {pressure:.9}°");
    assert!((pressure - 30.0).abs() < 1e-9, "{pressure}");

    // the path, by central differences of the traced contact
    let h = 1e-3;
    let ends = trace(&e, &[-h, h]);
    let path = unit(sub(ends[1].position, ends[0].position));
    let outward = direction(&e, "pair.trace.normal");
    let path = if dot(path, outward) < 0.0 { scale(path, -1.0) } else { path };
    let angle = dot(path, direction(&e, "pair.trace.heading")).acos().to_degrees();
    eprintln!("path angle to the heading {angle:.7}°");
    assert!((angle - 70.0).abs() < 1e-5, "{angle}");

    // the transmission error over the phase, a point contact throughout
    for s in &samples {
        eprintln!("{:+6.2}° lag {:+.7}° {}", s.place.phase, s.place.lag,
            if s.line { "line" } else { "point" });
        assert!(!s.line && s.place.lag <= 1e-12, "{s:?}");
    }
    assert!((samples[0].place.lag + 0.03).abs() < 1e-9, "{:?}", samples[0]);
    assert!((samples[12].place.lag + 0.0281899).abs() < 1e-6, "{:?}", samples[12]);

    // no undercut
    let ours = reading(&e);
    let theirs = admitted(&e).expect("the pinion is admitted");
    eprintln!("least area factor {ours:.6}, sampled {theirs:.6}");
    assert!(ours > 0.0 && theirs - ours >= -1e-9 && theirs - ours < 5e-3, "{ours} {theirs}");
}

/// The gear is cut to the ease: its concave blade, a circle about `centre` in the normal section
/// turned about the axis through `turn_centre`, passes within a tenth of a micrometre of the
/// eased flank — the face mill's cone moved along its normal (toward the axis) by `½ κₗ s² +
/// ½ κₚ r²` about T, worked out here from the cone alone — along the whole bearing, the meshing's
/// contact on the gear over the phase. Away from it the two agree to second order about T and
/// part as the cube of the distance: a micrometre and a half 1.25 mm up the profile and 4 mm
/// along the tooth, 13 µm at its corners, where the pair never touches. The ease relieves the
/// flank: it moves away from the mill's axis, into the gear's tooth.
#[test]
fn the_gear_is_cut_to_the_ease() {
    let e = solved();
    let (kp, kl) = (number(&e, "pair.profile_ease"), number(&e, "pair.lengthwise_ease"));
    let t = at(&e, "pair.mean.T");
    let c = at(&e, "pair.trace.C");
    let ax = direction(&e, "pair.mill.ax");
    let along = direction(&e, "pair.trace.heading");
    let apex = at(&e, "pair.mill.inner_flank.apex");
    let to_axis = unit(sub(at(&e, "pair.mean.to_axis"), t));
    // the blade: its circle's centre and radius, and the axis it is turned about
    let centre = at(&e, "pair.blade.centre");
    let start = at(&e, "pair.blade.turn_centre");
    let about = unit(sub(at(&e, "pair.blade.finish"), start));
    let rho = norm(sub(centre, t));
    let cutter = norm(cross(sub(t, start), about));
    let mill = norm(cross(sub(t, c), ax));
    eprintln!("profile radius {rho:.6} mm (1/κₚ {:.6}), cutter radius {cutter:.6} mm (the mill's \
        at T {mill:.6})", -1.0 / kp);
    assert!(norm(cross(about, ax)) < 1e-9, "the blade turns about an axis parallel to the mill's");
    // how far a point is off the blade's surface: the torus about `about` through the circle
    let polar = |y: V| {
        let w = sub(y, start);
        let h = dot(w, about);
        (norm(sub(w, scale(about, h))), h)
    };
    let (rc, hc) = polar(centre);
    let off = |y: V| {
        let (r, h) = polar(y);
        (r - rc).hypot(h - hc) - rho
    };
    // the cone's point `s` along its generator from T and `l` round it, its normal there, and
    // the eased flank's point
    let generator = unit(sub(t, apex));
    let eased = |s: f64, l: f64| {
        let x = turned(add(t, scale(generator, s)), c, ax, l / mill);
        let n = turned(to_axis, [0.0; 3], ax, l / mill);
        let w = sub(x, t);
        let a = dot(w, along);
        let across = sub(w, scale(along, a));
        let d = 0.5 * kl * a * a + 0.5 * kp * dot(across, across);
        (add(x, scale(n, d)), d)
    };
    // about T, a parabola of the distance; at the tooth's ends, its cube
    let (mut near, mut far) = (0f64, 0f64);
    for k in -4..=4 {
        for j in -4..=4 {
            let (s, l) = (0.625 * k as f64, 2.0 * j as f64);
            let (y, d) = eased(s, l);
            assert!(d <= 0.0, "the ease moves the flank off the axis: {d}");
            let o = off(y).abs();
            if s.abs() <= 1.25 && l.abs() <= 4.0 { near = near.max(o) } else { far = far.max(o) }
        }
    }
    eprintln!("off the blade: {:.3} µm within 1.25 mm and 4 mm of T, {:.3} µm to the tooth's ends",
        near * 1e3, far * 1e3);
    assert!(near < 2e-3 && far < 2e-2, "{near} {far}");
    // along the bearing, the meshing's own eased point
    let phases: Vec<f64> = (0..=30).map(|k| -7.5 + 0.5 * k as f64).collect();
    let bearing = trace(&e, &phases).iter().map(|s| off(s.on_pinion).abs()).fold(0f64, f64::max);
    eprintln!("the bearing at most {:.3} µm off the blade", bearing * 1e3);
    assert!(bearing < 1e-4, "{bearing}");
}
