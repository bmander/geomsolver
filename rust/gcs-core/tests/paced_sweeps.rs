//! **A sweep is its path's, not its pace's** (#684 L, `solid::paced`): a joint driven by a law of
//! the roll stands the tool at the joint's own poses over the values the law takes, so what it
//! sweeps is the joint's sweep over the least to the greatest of them — a turn, a slide or a
//! screw, built exactly by the constant-twist class (or a ball's tube by the canal), however the
//! law rises, dwells and returns. The gate: a law's reach against its closed form (`Law::range`,
//! 1e-12); a slide that rises, dwells and returns, held to the slide's lens and its STL to the
//! driven sweep's own field; a ball rocked out and back, Pappus; and a drive seen from another
//! motion, still refused. `solid_sweeps/rocked.sv` is held in `solid_sweeps_examples.rs`.
use gcs_core::motion::Law;
use gcs_core::program::Elaborated;
use gcs_core::solid::admission::{self,Asked,Class,Condition,Options};
use std::f64::consts::{PI,TAU};

use crate::common::text::solved;
use crate::round_lowering::lens;
use crate::solid_sweeps_examples::{class,text};
use crate::turns_and_slides::{exported,near,refused};

/// A law's reach over a roll is its closed form's: monotone, kinked, dwelling and turning back in
/// the middle of the roll, each least and greatest within 1e-12 of the reach's magnitude (of a
/// radian, where it is less); a law no enclosure is carried of is refused, never guessed.
#[test]
fn a_laws_reach_is_the_least_and_greatest_it_takes() {
    let e = solved("unit mm\nparam t: Angle over (0deg, 360deg)\nuse std\nin std.front {\n  \
        o := point\n  fix((0, 0)) o\n}\n");
    let rad = f64::to_radians;
    // `60deg * sin(2t)` added to the roll turns back where 1 + 2k cos 2t = 0, k its amplitude
    let k = rad(60.);
    let peak = (-1./(2.*k)).acos()/2.;
    let wave = |t: f64| t+k*(2.*t).sin();
    let cases: [(&str,[f64;2],[f64;2]);8] = [
        ("5deg * (1 - cos(t))",[0.,360.],[0.,rad(10.)]),
        ("5deg * (1 - cos(t))",[30.,120.],
         [rad(5.*(1.-rad(30.).cos())),rad(5.*(1.-rad(120.).cos()))]),
        ("30deg * sin(3 * t)",[0.,360.],[rad(-30.),rad(30.)]),
        ("30deg * sin(3 * t)",[10.,50.],[rad(15.),rad(30.)]),
        ("90deg - abs(t - 100deg)",[0.,360.],[rad(-170.),rad(90.)]),
        ("360deg * min(max(min(t / 90deg, (360deg - t) / 90deg), 0), 1)",[0.,360.],[0.,TAU]),
        ("360deg * min(max(min(t / 90deg, (360deg - t) / 90deg), 0), 1)",[100.,200.],[TAU,TAU]),
        ("t + 60deg * sin(2 * t)",[30.,150.],[wave(PI-peak),wave(peak)]),
    ];
    for (text,roll,[lo,hi]) in cases {
        let law = Law::read(&e.sketch,text,"t").expect(text);
        let [a,b] = law.range(roll.map(rad)).unwrap_or_else(|m| panic!("{text}: {m}"));
        eprintln!("{text} over {roll:?}: [{a}, {b}] against [{lo}, {hi}]");
        // `RANGE` of a radian, or of the reach's magnitude where it is more
        let bar = 1e-12*lo.abs().max(hi.abs()).max(1.);
        assert!((a-lo).abs() <= bar && (b-hi).abs() <= bar,
            "{text} over {roll:?}: [{a}, {b}] against [{lo}, {hi}]");
    }
    let tangent = Law::read(&e.sketch,"tan(t / 8)","t").unwrap();
    let refused = tangent.range([0.,1.]).unwrap_err();
    assert!(refused.contains("no enclosure"),"{refused}");
}

/// The line of `slide.sv` that sweeps its wheel, which the cases here write under a drive.
const SLID: &str = "construction groove := sweep(wheel over t in (0deg, 360deg), under: feed)";

/// The example `name` with each `(from, to)` written in, every `from` found in it first.
fn edited(name: &str,edits: &[(&str,&str)]) -> Elaborated {
    let mut src = text(name);
    for (from,to) in edits {
        assert!(src.contains(from),"`{name}` writes `{from}`");
        src = src.replacen(from,to,1);
    }
    fixtures::read(&src)
}

/// `slide.sv`'s wheel driven up the rod and back: it rises a quarter of the roll, dwells at the
/// top for half and returns, so it stands at every height of the slide's whole turn and at no
/// other, and the groove is the slide's — the lens the rod's disc and the tube's share, carried
/// the rod's 20 mm — built as a slide's and held to the driven sweep's own field.
#[test]
fn a_slide_that_rises_dwells_and_returns_cuts_what_its_rise_does() {
    let e = edited("slide.sv",&[(SLID,"param t: Angle over (0deg, 360deg)\n\
        fix(value == 360deg * min(max(min(t / 90deg, (360deg - t) / 90deg), 0), 1)) feed\n\
        construction groove := sweep(wheel over t, under: feed)")]);
    let groove = fixtures::solid(&e,"groove");
    let (_,[lo,hi]) = gcs_core::solid::paced::restated(&e.sketch,groove)
        .expect("read as its joint's slide");
    assert!(lo.abs() < 1e-12 && (hi-TAU).abs() < 1e-12,"[{lo}, {hi}]");
    let Class::ConstantTwist(found) = class(&e,"rod") else { panic!("the constant-twist class") };
    assert!(found.screw.is_slide(),"read as a slide");
    near(exported(&e,"rod"),PI*25.*20.-lens(5.,2.,5.5)*20.,1e-6);
}

/// `ball.sv`'s ball rocked out its quarter turn and back by the simple harmonic law: it stops at
/// each end, where a tube fitted along its path would stop too (B1), but a sweep is its path's,
/// so the groove is the quarter turn's tube with half a ball at each end, as `ball.sv`'s (Pappus).
#[test]
fn a_ball_rocked_out_and_back_leaves_its_tube_once() {
    let e = edited("ball.sv",&[(
        "construction groove := sweep(cutter over t in (0deg, swing), under: swinging)",
        "param t: Angle over (0deg, 360deg)\n\
         fix(value == swing * (1 - cos(t)) / 2) swinging\n\
         construction groove := sweep(cutter over t, under: swinging)")]);
    let groove = (PI*9.*12.*PI/2.+4./3.*PI*27.)/2.;
    near(exported(&e,"block"),24.*24.*8.-groove,1e-9);
}

/// A drive seen from another motion is a path of its own: the wheel slid up the rod by a law while
/// the rod turns under it moves along no constant twist, so the drive is not read as its joint, and
/// the sweep is refused by the motion's row, saying so.
#[test]
fn a_drive_seen_from_another_motion_is_still_refused() {
    let e = edited("slide.sv",&[("use std.motion (slides)","use std.motion (slides, turns)"),
        (SLID,"param t: Angle over (0deg, 360deg)\n\
         fix(value == 180deg * (1 - cos(t))) feed\n\
         spin := frame\nspin turns(1) ax\n\
         construction groove := sweep(wheel over t, under: feed, seen_from: spin)")]);
    assert!(gcs_core::solid::paced::restated(&e.sketch,fixtures::solid(&e,"groove")).is_none());
    let r = refused(admission::admit_body(&e.sketch,fixtures::solid(&e,"rod"),&Options::default()));
    assert!(r.condition == Condition::Motion && r.asked == Asked::Generating,"{r}");
    assert!(r.message.contains("seen from another motion"),"{r}");
}
