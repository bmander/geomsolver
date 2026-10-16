//! Ease-off (#345 S2): a meshing's first flank relieved along its normal by a parabola about a
//! point, `½ κₗ s² + ½ κₚ r²` — `s` how far along a line, `r` how far across it.  A conjugate
//! pair touches along a line; eased, it touches at a point, and the gear lags by the turn that
//! closes the relief.  Each is held here to something worked out apart from `envelope::meshing`:
//! the lag and the bearing against the motions' poses, the gear's cone and the eased flank's own
//! equation, found with no row of the meshing.
use crate::common::build;
use crate::meshing::{at, closing, holds_together, solved, start, Generated};
use gcs_core::envelope::meshing::{Meshing, Sample};
use gcs_core::model::Sketch;
use gcs_core::space::{add, dot, norm, scale, sub, unit};

/// The ease: per millimetre, across `lengthwise` (the profile) and along it.
const PROFILE: f64 = 0.002;
const LENGTHWISE: f64 = 0.0005;

/// `tests/meshing.rs`'s mesh — crossed shafts 72 apart, the gear at half the pinion's speed, the
/// gear's flank formed, a cone about the face mill's axis through M — and the pinion's flank its
/// envelope under the gear's turn seen from the pinion's, a conjugate pair; the pinion's flank
/// eased by `profile` and `lengthwise` (texts) about M, `lengthwise` running along the face mill's
/// circle there, and Q held to the contact where the pair is drawn.
fn eased(profile: &str, lengthwise: &str) -> String {
    format!("{}\
L := point hint((10, 48, 10))
fix((10mm, 48mm, 10mm)) L
lengthwise := line(M, L)
pinion_flank := envelope(flank, under: gear_turn, seen_from: pinion_turn, from: -40deg,
                         to: 40deg)
contact := meshing(pinion_flank, flank, under: gear_turn, seen_from: pinion_turn, from: -20deg,
                   to: 20deg, ease: M,
                   along: lengthwise, profile: {profile}, lengthwise: {lengthwise})
Q := point hint((0, 48, 0))
Q coincident(t == 0deg) contact
", crate::meshing::MESH)
}

/// The pair eased by `PROFILE` and `LENGTHWISE`.
fn stated() -> String {
    eased(&format!("{PROFILE} / 1mm"), &format!("{LENGTHWISE} / 1mm"))
}

fn meshing(sk: &Sketch) -> Meshing {
    Meshing::named(sk, 0).unwrap()
}

/// How far the ease relieves a point `y` of the pinion as drawn: the parabola about M, `PROFILE`
/// across `lengthwise` and `LENGTHWISE` along it.
fn relief(y: [f64; 3]) -> f64 {
    let a = unit([1.0, 0.0, 1.0]);
    let w = sub(y, [0.0, 48.0, 0.0]);
    let s = dot(w, a);
    let r = sub(w, scale(a, s));
    0.5 * LENGTHWISE * s * s + 0.5 * PROFILE * dot(r, r)
}

/// The pinion's flank — the gear's cone's envelope as the pinion sees it — eased, read with
/// nothing of `envelope::held` or `envelope::meshing`: the envelope's point at `(u, t)` as
/// `tests/meshing.rs` reads it (`Generated`), moved along the moved cone's normal `∂u × ∂v` there
/// by the relief at the point.
fn eased_point(g: &Generated, u: f64, t: f64, v0: f64) -> ([f64; 3], f64) {
    let (y, v) = g.point(u, t, v0);
    (add(y, scale(g.normal(u, v, t), relief(y))), v)
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
    let e = solved(&stated());
    let (m, apex) = (at(&e, "M"), at(&e, "apex"));
    let axis = unit(sub(at(&e, "C"), apex));
    assert!(norm(sub(at(&e, "Q"), m)) < 1e-9, "the contact is at M where the pair is drawn");
    let t = meshing(&e.sketch).traced(8).unwrap();
    assert_eq!(t.found().count(), 9);
    let [lo, hi] = t.error_range().unwrap();
    assert!(hi - lo > 1e-3, "the gear lags by something: {lo} … {hi}");
    assert!(t.path_length() > 10.0, "the contact moves: {}", t.path_length());
    let flank = Generated::new(apex, axis);
    let point = |u, t, v| eased_point(&flank, u, t, v);
    for s in t.found() {
        assert!(!s.line, "an eased pair touches at a point: {s:?}");
        holds_together(s, apex, axis);
        if s.place.phase == 0.0 {
            assert!(s.place.lag.abs() < 1e-12, "{s:?}");
            assert!(norm(sub(s.on_pinion, m)) < 1e-9, "{s:?}");
            continue;
        }
        // the lag that closes the gap between the eased flank and the gear's cone, and where
        let (lag, y) = closing(&point, (apex, axis), s, start(&flank, s));
        assert!((lag - s.place.lag).abs() < 1e-9, "phase {}: closed at {lag}, meshed at {}",
                s.place.phase, s.place.lag);
        assert!(norm(sub(y, s.position)) < 1e-5, "phase {}: {y:?} {:?}", s.place.phase, s.position);
    }
}

/// **The lag is a number a document states, and the ease solves from it.** The forward pair's
/// lags at −15° and 15°, stated of two points held to a pair whose curvatures are unknowns,
/// solve both curvatures back: the transmission error at the two ends of the phase sizes the
/// ease, the two ends' difference saying how it divides between across and along.  And the
/// solved pair, traced, lags as stated there.
#[test]
fn lags_stated_at_two_phases_solve_the_ease() {
    let e = solved(&stated());
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

/// An ease prints as it is written and reads back the same; one with a part missing, one about
/// no point or along no line, one whose curvature is no number per length, and one on a surface
/// traced in space, are refused saying why.
#[test]
fn an_ease_prints_back_and_refuses_what_it_cannot_read() {
    // `tests/meshing.rs`'s revolution's side meshed with its envelope, the envelope eased about a
    // point of the gear's axis
    let src = crate::meshing::SIDE_MESHING.replace("from: -20deg, to: 20deg)",
        "from: -20deg, to: 20deg, ease: g1, along: gear_ax, profile: 0.002 / 1mm, \
         lengthwise: -0.0005 / 1mm)");
    let (mut prog, errs) = crate::common::parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let first = gcs_core::program::elaborate(&prog);
    assert!(first.ok(), "{:?}", first.errors().map(|d| &d.message).collect::<Vec<_>>());
    let ease = |e: &gcs_core::program::Elaborated| e.sketch.meshings[0].ease.clone().unwrap();
    let printed = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    let said = "to: 20deg, ease: g1, along: gear_ax, profile: 0.002 / 1mm, \
                lengthwise: -0.0005 / 1mm";
    assert!(printed.contains(said), "{printed}");
    let again = build(&printed);
    assert_eq!(ease(&again), ease(&first));
    let numbers = ease(&first).numbers.map(|n| n.c);
    assert_eq!(numbers, [0.002, -0.0005]);

    let src = stated();
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
    // the bearing's bounds, the one place the document writes them
    let traced = include_str!("../../examples/spiral_bevel/phillips_pure.sv").replace(
        "from: -15deg, to: 15deg)",
        "from: -15deg, to: 15deg, ease: P, along: L, profile: 0.001 / 1mm, \
         lengthwise: 0.001 / 1mm)");
    refused(&traced, "a surface traced in space, which is not eased");
}
