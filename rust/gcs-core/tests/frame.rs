//! The frame as a datum: an origin, a point it is pointed at, and a unit rotor slaved to the
//! chord between them by its two intrinsic constraints — so the attitude is a first-class
//! unknown that adds no freedom beyond the two points.
use gcs_core::constraints::CKind;
use gcs_core::model::{distance_between, pick, EntRef, Sketch};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::{diagnose, io};

/// A frame whose chord is (4, 3) — length 5, so the rotor is the 3-4-5 triangle's.
fn with_frame() -> (Sketch, usize) {
    let mut sk = Sketch::new();
    let o = sk.point(10.0, 5.0, false, "o");
    let t = sk.point(14.0, 8.0, false, "t");
    let f = sk.frame(o, t, "f");
    (sk, f)
}

#[test]
fn the_rotor_is_seeded_from_the_chord() {
    let (sk, f) = with_frame();
    let fr = &sk.frames[f];
    assert!((sk.params[fr.c as usize].value - 0.8).abs() < 1e-12);
    assert!((sk.params[fr.s as usize].value - 0.6).abs() < 1e-12);
    // one rotor unit is worth the chord's length — what `Param::scale` asks of a
    // dimensionless unknown
    assert!((sk.params[fr.c as usize].scale - 5.0).abs() < 1e-12);
    assert!((sk.params[fr.s as usize].scale - 5.0).abs() < 1e-12);
    // the two intrinsics came with it, and they are the whole constraint list
    assert_eq!(sk.constraints.len(), 2);
    assert!(sk.constraints.iter().all(|c| c.intrinsic));
    assert_eq!(sk.constraints[0].kind, CKind::FrameUnit);
    assert_eq!(sk.constraints[1].kind, CKind::FrameAlign);
    assert!(sk.user_constraints().is_empty(), "an intrinsic is not something the user said");
    // the alignment's own unknown seeds at the chord's length
    let rp = sk.constraints[1].args[1].param() as usize;
    assert!((sk.params[rp].value - 5.0).abs() < 1e-12);
}

#[test]
fn a_frame_adds_no_freedom() {
    let (mut sk, _) = with_frame();
    let d = diagnose::diagnose(&mut sk, Default::default());
    assert_eq!(d.dof, 4, "two free points, and the rotor slaved to them");
}

#[test]
fn the_rotor_tracks_the_chord_through_a_solve() {
    let (mut sk, f) = with_frame();
    let (o, t) = (sk.frames[f].origin as usize, sk.frames[f].toward as usize);
    sk.fix_point(o, true);
    sk.fix_point(t, true);
    // start the rotor a quarter-turn wrong: the intrinsics are equations, not decoration
    let (cp, sp) = (sk.frames[f].c as usize, sk.frames[f].s as usize);
    sk.params[cp].value = 0.0;
    sk.params[sp].value = 1.0;
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    assert!((sk.params[cp].value - 0.8).abs() < 1e-8);
    assert!((sk.params[sp].value - 0.6).abs() < 1e-8);
}

#[test]
fn round_trips_through_json() {
    let (mut sk, f) = with_frame();
    sk.frames[f].class = gcs_core::style::Classes::one("construction");
    let (cp, sp) = (sk.frames[f].c as usize, sk.frames[f].s as usize);
    sk.params[cp].fixed = true;
    // an unsolved pose survives: the saved rotor wins over the recomputed one
    sk.params[sp].value = 0.25;
    let back = io::loads(&io::dumps(&sk, None)).unwrap();
    assert_eq!(back.frames.len(), 1);
    let bf = &back.frames[0];
    assert!(bf.class.has("construction"));
    assert!(back.params[bf.c as usize].fixed);
    assert!(!back.params[bf.s as usize].fixed);
    assert_eq!(back.params[bf.c as usize].value, 0.8);
    assert_eq!(back.params[bf.s as usize].value, 0.25);
    // the intrinsics are not stored — the primitive recreates them
    assert!(io::dumps(&sk, None).find("FrameUnit").is_none());
    assert_eq!(back.constraints.len(), 2);
    assert!(back.user_constraints().is_empty());
}

#[test]
fn a_copy_keeps_the_frame_and_deletion_follows_its_points() {
    let (sk, f) = with_frame();
    let all = [
        EntRef::point(sk.frames[f].origin as usize),
        EntRef::point(sk.frames[f].toward as usize),
        EntRef::frame(f),
    ];
    let clip = io::copy(&sk, &all);
    assert_eq!(clip.frames.len(), 1);
    assert_eq!(clip.constraints.len(), 2, "the graft's constructor re-mints the intrinsics");
    let bf = &clip.frames[0];
    assert_eq!(clip.params[bf.c as usize].value, 0.8);
    // a frame that lost a defining point is deleted whole
    let cut = io::without(&sk, &[EntRef::point(sk.frames[f].origin as usize)], &[]);
    assert_eq!(cut.frames.len(), 0);
    assert!(cut.constraints.is_empty());
}

/// A frame sorts last in `measure_order`, so every pair holding one reaches the swept arm with
/// the frame as `b` — above the arms that would ask it for a centre and a radius it does not
/// have.  Measured at its origin, being a datum rather than a figure.
#[test]
fn every_pair_with_a_frame_measures_from_its_origin() {
    let (mut sk, f) = with_frame();          // origin (10, 5), toward (14, 8)
    let p = sk.point(10.0, 0.0, false, "p");
    let l = sk.line_xy(0.0, 0.0, 100.0, 0.0, "l");
    let ci = sk.circle(p, 2.0, "ci");
    let f2 = {
        let a = sk.point(10.0, 25.0, false, "o2");
        let b = sk.point(11.0, 25.0, false, "t2");
        sk.frame(a, b, "f2")
    };
    let fr = EntRef::frame(f);
    // each is the distance from (10, 5) to the thing, and none of them panics
    assert!((distance_between(&sk, fr, EntRef::point(p)) - 5.0).abs() < 1e-12);
    assert!((distance_between(&sk, fr, EntRef::line(l)) - 5.0).abs() < 1e-12);
    assert!((distance_between(&sk, fr, EntRef::circle(ci)) - 3.0).abs() < 1e-12);
    assert!((distance_between(&sk, fr, EntRef::frame(f2)) - 20.0).abs() < 1e-12);
    // and the pair reads the same measured either way round
    assert_eq!(
        distance_between(&sk, EntRef::line(l), fr),
        distance_between(&sk, fr, EntRef::line(l))
    );
}

#[test]
fn a_frame_is_never_picked() {
    let (sk, f) = with_frame();
    let (ox, oy) = sk.point_xy(sk.frames[f].origin as usize);
    // a click at the origin picks the point, not the frame standing on it
    assert_eq!(pick(&sk, ox, oy, 0.5), Some(EntRef::point(sk.frames[f].origin as usize)));
    // and the chord's midpoint is empty space: a frame draws nothing of its own
    assert_eq!(pick(&sk, 12.0, 6.5, 0.5), None);
}

/* -- the frame in a trace block: bearings measured from the drawing, not the page ---------- */

use gcs_core::program::elaborate;
use gcs_core::syntax::parse;

/// A two-bar elbow posed at crank angle `u` *from the datum*: the crank end `t` is pinned by a
/// directed angle, and the elbow `p` is the intersection of two circles — a mirror pair about
/// the crank, with **no predicate and no signed row to choose between them**.  The seed alone
/// picks the side, which is exactly the job issue #10 found it doing with page-fixed numbers:
/// written `u + 53` the choice is right only while the datum is horizontal.  `f.angle` is the
/// datum's own bearing, read off the frame's rotor, so the seed follows the drawing.
const ELBOW: &str = "\
curve elbow(o: point, datum: line, f: frame)(u) over (10, 80) =
  trace p from (30) where {
    point t hint(x: o.x + 60 * cos(u + f.angle), y: o.y + 60 * sin(u + f.angle))
    point p hint(x: o.x + 50 * cos(u + f.angle + 53), y: o.y + 50 * sin(u + f.angle + 53))
    line swing(o, t)
    datum angle(u) swing
    o distance(60) t
    t distance(50) p
    o distance(50) p
  }

point o hint(x: 0, y: 0)
point q hint(x: -30, y: 51.9615242270663)
line  datum(o, q) class construction
frame f(origin: o, toward: q) class construction

curve path = elbow(o, datum, f)

ground o
ground q
";

/// Where the elbow is at `u`, worked out here: 50 out of `o` at bearing `datum + u + acos(0.6)`,
/// the isoceles triangle 60-50-50 opened on the counter-clockwise side.
fn elbow_at(datum_deg: f64, u_deg: f64) -> (f64, f64) {
    let beta = 0.6f64.acos();
    let th = (datum_deg + u_deg).to_radians() + beta;
    (50.0 * th.cos(), 50.0 * th.sin())
}

fn build(src: &str) -> gcs_core::program::Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    elaborate(&prog)
}

#[test]
fn a_frame_relative_seed_follows_a_tilted_datum() {
    let mut e = build(ELBOW);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let path = e.map.ent_named("path").unwrap();
    for u in [15.0, 30.0, 55.0, 75.0] {
        let want = elbow_at(120.0, u);
        let got = e.sketch.curve_point(path.i(), u);
        assert!(
            (got.0 - want.0).abs() < 1e-6 && (got.1 - want.1).abs() < 1e-6,
            "u = {u}: {got:?} against {want:?}",
        );
    }
}

/// The negative control: the same document with *one* seed page-fixed — the elbow's, with the
/// crank's still following the frame, which is issue #10's actual shape (a bearing seed reading
/// the geometry beside coordinate seeds that do not).  Striking `f.angle` from both would rotate
/// the seeds rigidly and change nothing; struck from one, the seeded chirality of (o, t, p)
/// flips, everything still elaborates and still solves — nothing *breaks* — and the trace comes
/// back with the joint folded the other way.  This is the quiet failure the issue describes,
/// and the reason a seed has to be able to name the frame.
#[test]
fn a_page_fixed_seed_picks_the_mirror_elbow() {
    let mut e = build(&ELBOW.replace("u + f.angle + 53", "u + 53"));
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let path = e.map.ent_named("path").unwrap();
    let want = elbow_at(120.0, 30.0);
    let got = e.sketch.curve_point(path.i(), 30.0);
    let miss = (got.0 - want.0).hypot(got.1 - want.1);
    assert!(miss > 10.0, "the page-fixed seed happened to find the right elbow ({miss})");
    // and it found the *mirror*, not garbage: the triangle closed on the other side
    let beta = 0.6f64.acos();
    let th = (150.0f64).to_radians() - beta;
    let mirror = (50.0 * th.cos(), 50.0 * th.sin());
    assert!(
        (got.0 - mirror.0).abs() < 1e-6 && (got.1 - mirror.1).abs() < 1e-6,
        "{got:?} against the mirror {mirror:?}",
    );
}

/// Rotating the datum carries the curve: the rotor is *solved* back into step — it is an
/// unknown with equations, not a number something recomputes — and every contact's ∂C/∂θ
/// includes the frame's columns, so the traced figure follows the drawing it is written over.
#[test]
fn turning_the_frame_carries_the_curve() {
    let mut e = build(ELBOW);
    assert!(e.ok());
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let q = e.map.ent_named("q").unwrap();
    let ps = e.sketch.point_params(q.i());
    e.sketch.params[ps[0] as usize].value = 0.0;   // the datum swings 120° -> 90°
    e.sketch.params[ps[1] as usize].value = 60.0;
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let path = e.map.ent_named("path").unwrap();
    let want = elbow_at(90.0, 40.0);
    let got = e.sketch.curve_point(path.i(), 40.0);
    assert!(
        (got.0 - want.0).abs() < 1e-6 && (got.1 - want.1).abs() < 1e-6,
        "{got:?} against {want:?}",
    );
}

/// A misspelling is still a misspelling: only `.angle` over a rotor the table holds is derived.
#[test]
fn a_wrong_name_is_still_refused() {
    let e = build(&ELBOW.replace("u + f.angle", "u + f.angel"));
    assert!(!e.ok(), "`f.angel` elaborated cleanly");
    assert!(
        e.errors().any(|d| d.message.contains("f.angel")),
        "{:?}",
        e.errors().map(|d| &d.message).collect::<Vec<_>>()
    );
}
