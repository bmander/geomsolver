//! **A ball along any path** (#684 D, `solid::canal`): a ball looks the same however it is turned,
//! so whatever a motion does to it, everything it passes through is the tube about its centre's
//! path with the ball at each end. A slide carries the centre along a line, built exactly (a
//! cylinder and two half balls, its ends inside what it cuts once refused at E1); a roll, a frame
//! along a spline, in a plane or in space, carry it along a curve no closed form gives, and the
//! tube's wall is fitted to it, a face family's one round wall; a frame along an arc carries it
//! along a circle, and the fitted wall lies on its torus. Each solid is held to the field — the
//! tube by its gate, the body that cuts it by its STL — and to a volume made apart from the code
//! under test: π r² L + 4/3 π r³, L the path's length integrated here from its closed form or its
//! control points. Where the tube would meet itself it is refused by its own rows, B1 (the path
//! turns tighter than the radius) and B2 (it comes back within the diameter), each with a point of
//! the path, and its sweep stays the field's.
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::program::Elaborated;
use gcs_core::solid::admission::{self, Condition, Error, Options};
use gcs_core::solid::canal;
use std::f64::consts::PI;

use crate::solid_sweeps_examples::text;
use crate::turns_and_slides::near;

/// The arc length of the cubic Bézier over `p`, by Gauss–Legendre quadrature over 64 stretches.
pub(crate) fn bezier_length<const N: usize>(p: [[f64; N]; 4]) -> f64 {
    let speed = |t: f64| -> f64 {
        let s = 1.0 - t;
        (0..N).map(|k| {
            let d = 3.0 * s * s * (p[1][k] - p[0][k]) + 6.0 * s * t * (p[2][k] - p[1][k]) + 3.0 * t * t * (p[3][k] - p[2][k]);
            d * d
        }).sum::<f64>().sqrt()
    };
    integral(&speed, 0.0, 1.0)
}

/// `f` integrated over `[a, b]` by five-point Gauss–Legendre over 64 stretches.
fn integral(f: &dyn Fn(f64) -> f64, a: f64, b: f64) -> f64 {
    let x = [0.0, 0.5384693101056831, -0.5384693101056831, 0.906179845938664, -0.906179845938664];
    let w = [0.5688888888888889, 0.47862867049936647, 0.47862867049936647, 0.23692688505618908, 0.23692688505618908];
    let h = (b - a) / 64.0;
    (0..64).map(|i| {
        let m = a + h * (i as f64 + 0.5);
        (0..5).map(|k| w[k] * f(m + h / 2.0 * x[k])).sum::<f64>() * h / 2.0
    }).sum()
}

/// The tube of radius `r` about a path of `length` with the ball at each end.
fn tube(r: f64, length: f64) -> f64 {
    PI * r * r * length + 4.0 / 3.0 * PI * r.powi(3)
}

/// The solid `name` built exactly within 0.01 mm by this kernel, written as STEP and STL, and held
/// to its own material field with no disagreement; its B-rep's volume and the kinds of its faces.
pub(crate) fn held(e: &Elaborated, name: &str) -> f64 {
    held_faces(e, name).0
}

fn held_faces(e: &Elaborated, name: &str) -> (f64, Vec<&'static str>) {
    let (sk, body) = (&e.sketch, fixtures::solid(e, name));
    assert!(!sk.is_swept(body), "`{name}` is read by the field");
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let tolerance = Some(gcs_core::solid::export::Tolerance::new(0.01).unwrap());
    let exact = export::exact(sk, body, None, tolerance, &say).unwrap_or_else(|r| panic!("{name}: {r:?}"));
    exact.solid.check(1e-6).unwrap();
    assert!(export::step(&exact, name, tolerance, &say).unwrap().starts_with("ISO-10303-21;"));
    gcs_core::mesh::stl_shells(&export::stl(sk, body, &exact, tolerance, &say).unwrap()).unwrap();
    let field = gcs_core::solid::MaterialField::read(sk, body, gcs_core::solid::cad::AXIS_TOLERANCE).unwrap();
    gcs_core::solid::agreement::gate(&field, &exact.solid, None, name).unwrap();
    (props::volume(&exact.solid), exact.solid.faces.iter().map(|f| f.surface.kind()).collect())
}

/// A ball of `ball` drawn about the origin, a half disc turned about its upright diameter.
const BALL: &str = "
in std.front {
  private bottom := point
  private top := point
  bottom coords(du: 0mm, dv: -ball) std.front
  top coords(du: 0mm, dv: ball) std.front
  private diameter := line(bottom, top)
  private meridian := arc(center: std.origin, start: bottom, end: top) hint(r: 1)
  radius(ball) meridian
}
construction cutter := solid(face(meridian, diameter), about: diameter)
";

/// A block 40 by 20 by 8 below the top face, from (-10, -10).
const BLOCK: &str = "
in std.top {
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  fix((-10, -10)) b0
  fix((30, -10)) b1
  fix((30, 10)) b2
  fix((-10, 10)) b3
  private s0 := line(b0, b1)
  private s1 := line(b1, b2)
  private s2 := line(b2, b3)
  private s3 := line(b3, b0)
}
block := solid(face(s0, s1, s2, s3), from: -8mm, to: 0mm)
";

/// **A slide of a ball is a capsule** (#684 B's slide reader, D): its centre's path a line, so the
/// tube is a cylinder, built exactly, with a half ball at each end — and its ends may stand inside
/// what it cuts, which the constant-twist class refuses at E1. Half sunk in the block, slid 20
/// along x from the origin: the groove is half the capsule.
#[test]
fn a_ball_slid_is_a_capsule_its_ends_in_the_blank() {
    let e = fixtures::read(&format!("unit mm\nuse std\nuse std.place (coords)\nuse std.motion (slides)\nball := 3mm\n{BALL}\
        in std.front {{\n  construction centerline rail := line(std.origin, hint((1, 0)))\n  fix((1, 0)) rail.p2\n}}\n{BLOCK}\
        feed := frame\nfeed slides(40mm) rail\nconstruction groove := sweep(cutter over t in (0deg, 180deg), under: feed)\n\
        groove cut block\n"));
    let c = canal::built(&e.sketch, fixtures::solid(&e, "groove")).unwrap().unwrap();
    assert!(!c.fitted(), "a line is a closed form");
    let (volume, kinds) = held_faces(&e, "block");
    assert!(kinds.contains(&"cylinder"), "{kinds:?}");
    near(volume, 40.0 * 20.0 * 8.0 - tube(3.0, 20.0) / 2.0, 1e-9);
}

/// **A ball along a spline in space** (#684 D, #398 C): a ball riding a frame along a cubic that
/// leaves every plane, swept standing alone over the cubic's whole parameter — a tube that twists
/// with the cubic's torsion, fitted to it, its length read off the control points.
#[test]
fn a_ball_along_a_spline_in_space_is_its_tube() {
    let e = fixtures::read(&format!("unit mm\nuse std\nuse std.place (coords)\nuse std.stations\nball := 2mm\n{BALL}\
        ka := point hint((0, 0, 0))\nkb := point hint((10, 20, 5))\nkc := point hint((30, 20, 15))\nkd := point hint((40, 0, 30))\n\
        fix((0mm, 0mm, 0mm)) ka\nfix((10mm, 20mm, 5mm)) kb\nfix((30mm, 20mm, 15mm)) kc\nfix((40mm, 0mm, 30mm)) kd\n\
        cubic := spline(ka, kb, kc, kd)\nalong := std.stations.Along(cubic, up: std.z)\n\
        wire := sweep(cutter over t in (0rad, 1rad), under: along.f)\n"));
    assert!(canal::built(&e.sketch, fixtures::solid(&e, "wire")).unwrap().unwrap().fitted());
    let length = bezier_length([[0.0, 0.0, 0.0], [10.0, 20.0, 5.0], [30.0, 20.0, 15.0], [40.0, 0.0, 30.0]]);
    near(held(&e, "wire"), tube(2.0, length), 1e-9);
}

/// **A ball rolled**: spun a quarter as fast as the roll about its own upright axis through
/// (3, 0, 0), from which it stands 2, and seen from a turn about the spindle — a generator's motion,
/// whose contact rows a ball's pole fails (M2). Its centre runs the trochoid `3 e^{-it} +
/// 2 e^{-3it/4}` over a third of a turn, its length integrated from the closed form.
#[test]
fn a_ball_rolled_is_its_tube_along_the_trochoid() {
    let e = fixtures::read(&format!("unit mm\nuse std\nuse std.place (coords)\nuse std.motion (turns)\nball := 1mm\n\
        in std.front {{\n  private bottom := point\n  private top := point\n  bottom coords(du: 5mm, dv: -ball) std.front\n\
          top coords(du: 5mm, dv: ball) std.front\n  private diameter := line(bottom, top)\n  private c := point\n\
          c coords(du: 5mm, dv: 0mm) std.front\n  private meridian := arc(center: c, start: bottom, end: top) hint(r: 1)\n\
          radius(ball) meridian\n  construction centerline spindle := line(std.origin, hint((0, 1)))\n  fix((0, 1)) spindle.p2\n\
          private h0 := point\n  private h1 := point\n  h0 coords(du: 3mm, dv: 0mm) std.front\n\
          h1 coords(du: 3mm, dv: 5mm) std.front\n  construction centerline own := line(h0, h1)\n}}\n\
        construction cutter := solid(face(meridian, diameter), about: diameter)\n\
        spin := frame\nspin turns(0.25) own\nobserver := frame\nobserver turns(1) spindle\n\
        rolled := sweep(cutter over t in (-60deg, 60deg), under: spin, seen_from: observer)\n"));
    assert!(canal::built(&e.sketch, fixtures::solid(&e, "rolled")).unwrap().unwrap().fitted());
    // |q'(t)| = |3 e^{-it} + 3/2 e^{-3it/4}|, for q(t) = 3 e^{-it} + 2 e^{-3it/4}
    let speed = |t: f64| (9.0 + 2.25 + 9.0 * (t / 4.0).cos()).sqrt();
    let length = integral(&speed, -PI / 3.0, PI / 3.0);
    near(held(&e, "rolled"), tube(1.0, length), 1e-9);
}

/// **A ball along an arc**: a frame along an arc of radius 15 carries the centre round a circle,
/// so the fitted wall lies on a torus (`brep::family` reads it so by its numbers); Pappus over the
/// arc's quarter turn.
#[test]
fn a_ball_along_an_arc_lies_on_its_torus() {
    let e = fixtures::read(&format!("unit mm\nuse std\nuse std.place (coords)\nuse std.stations\nball := 2mm\n{BALL}\
        in std.top {{\n  private a := point\n  private o := point\n  private e := point\n  fix((0mm, 0mm)) a\n\
          fix((0mm, 15mm)) o\n  fix((15mm, 15mm)) e\n  bend := arc(center: o, start: a, end: e)\n}}\nalong := std.stations.Along(bend, up: std.z)\n\
        bent := sweep(cutter over t in (0rad, 1rad), under: along.f)\n"));
    let (volume, kinds) = held_faces(&e, "bent");
    assert!(kinds.contains(&"torus"), "{kinds:?}");
    near(volume, tube(2.0, 15.0 * PI / 2.0), 1e-9);
}

fn refused(e: &Elaborated) -> admission::Refusal {
    match admission::admit_body(&e.sketch, fixtures::solid(e, "block"), &Options::default()) {
        Err(Error::Refused(r)) => { eprintln!("{r}"); r }
        Err(Error::Unreadable(m)) => panic!("unreadable: {m}"),
        Ok(a) => panic!("admitted: {:?}", a.sweeps().iter().map(|s| &s.name).collect::<Vec<_>>()),
    }
}

/// **B1**: `solid_sweeps/path.sv`'s spline turns on a radius of 7.5 at its tightest, so a ball of
/// 8 would fold its tube there: refused by name, a point of the path its witness, its sweep the
/// field's.
#[test]
fn a_path_tighter_than_the_ball_is_refused() {
    let e = fixtures::read(&text("path.sv").replace("ball := 3mm", "ball := 8mm"));
    let r = refused(&e);
    assert_eq!((r.condition, r.class(), r.sweep.as_str()), (Condition::Curvature, "canal class", "groove"));
    assert!(r.message.contains("turns on a radius of") && r.message.contains("read at"), "{}", r.message);
    let w = r.witness.unwrap();
    assert!(w[2].abs() < 1e-9 && (-1e-9..=30.0).contains(&w[0]), "a point of the path: {w:?}");
    assert!(e.sketch.is_swept(fixtures::solid(&e, "block")));
}

/// **B2**: a spline that loops round and ends 4 from where it starts turns no tighter than 8.5,
/// but a ball of 3 would cross its own tube where the ends meet: refused by name, the witness
/// between them.
#[test]
fn a_path_that_comes_back_within_the_diameter_is_refused() {
    let looped = text("path.sv").replace("fix((10mm, 0mm)) k1", "fix((40mm, -30mm)) k1")
        .replace("fix((20mm, 20mm)) k2", "fix((40mm, 30mm)) k2").replace("fix((30mm, 20mm)) k3", "fix((0mm, 4mm)) k3");
    let e = fixtures::read(&looped);
    let r = refused(&e);
    assert_eq!((r.condition, r.class()), (Condition::Return, "canal class"));
    assert!(r.message.contains("comes back"), "{}", r.message);
    let w = r.witness.unwrap();
    assert!(w[0].hypot(w[1] - 2.0) < 4.0 && w[2].abs() < 1e-9, "between the ends: {w:?}");
    assert!(e.sketch.is_swept(fixtures::solid(&e, "block")));
}
