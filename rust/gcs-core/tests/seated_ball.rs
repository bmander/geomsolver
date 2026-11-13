//! **A ball seated in an assembly is written as the ball it is** (shaft_drive README, finding 19):
//! a bearing's balls, each a half disc turned about its diameter, seated where a hub's bearings stand,
//! were refused or written as an STL of no triangles.  Two faults, each held here.
//!
//! *A radius solves to either sign, and a solid reads its magnitude* (contract: solids.md).  An
//! arc's and a circle's rows read their radius squared, or as a point's distance from the centre,
//! so a solve may leave it negative — the seated component's did, its half disc written as a
//! circle of radius −2 between its own ends' angles, the other half of the circle.  The tests flip
//! a solved radius's sign, which is still a solution, and hold the solid to the one at the positive
//! radius.
//!
//! *A triangle with no plane stands off the surface by its distance from its sides* (`brep::mesh`,
//! `off_at`).  A sphere's first triangulation joins its seam's samples either side of the seam,
//! two corners of each triangle one point; read as its plane's distance it stood off by nothing,
//! was never refined, and was dropped as having no area, the whole face with it — wherever the
//! seam's samples came out the same to the bit, as they did for the seated ball.
use crate::common::{ent, parsed::determined};
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::program::Elaborated;
use std::f64::consts::PI;

/// Solid `name` of `e` built exactly and written as STL (its shells checked); its volume.
fn volume(e: &Elaborated, name: &str) -> f64 {
    let s = ent(e, name).i();
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = export::exact(&e.sketch, s, None, None, &say).unwrap_or_else(|r| panic!("{name}: {r:?}"));
    let stl = export::stl(&e.sketch, s, &exact, None, &say).unwrap_or_else(|r| panic!("{name}: {r:?}"));
    gcs_core::mesh::stl_shells(&stl).unwrap();
    props::volume(&exact.solid)
}

/// The radius parameter of the arc or circle `name`, set to its negative.
fn flip(e: &mut Elaborated, name: &str) {
    let r = ent(e, name);
    let p = match r.kind {
        gcs_core::model::EntKind::Arc => e.sketch.arcs[r.i()].radius,
        gcs_core::model::EntKind::Circle => e.sketch.circles[r.i()].radius,
        k => panic!("`{name}` is a {k:?}"),
    } as usize;
    assert!(e.sketch.params[p].value > 0.);
    e.sketch.params[p].value = -e.sketch.params[p].value;
}

const BALL: &str = "unit mm
use std
use std.solids
in std.front {
  c := point
  std.origin distance(9mm, along: u) c
  std.origin distance(70mm, along: v) c
  n := point
  s_ := point
  c distance(2mm, along: v) n
  c distance(0mm, along: u) n
  c distance(0mm, along: u) s_
  c distance(<= 0mm, along: v) s_
  rim := arc(center: c, start: s_, end: n)
  construction across := line(n, s_)
}
ball := solid(std.solids.Revolution(face(rim, across), about: across))
";

#[test]
fn a_half_disc_whose_radius_solved_negative_turns_into_the_same_ball() {
    let ball = 4. / 3. * PI * 8.;
    let mut e = determined(BALL);
    let at = volume(&e, "ball");
    assert!((at - ball).abs() < 1e-6 * ball, "{at} against {ball}");
    flip(&mut e, "rim");
    let flipped = volume(&e, "ball");
    assert!((flipped - ball).abs() < 1e-6 * ball, "{flipped} against {ball}");
}

#[test]
fn a_disc_whose_radius_solved_negative_stands_as_the_same_cylinder() {
    let mut e = determined("unit mm
use std
use std.solids
in std.top {
  c := point
  c coincident std.top.origin
  rim := circle(center: c)
  radius(3mm) rim
}
post := solid(std.solids.Prism(face(rim), from: 0mm, to: 10mm))
");
    let post = PI * 9. * 10.;
    flip(&mut e, "rim");
    let flipped = volume(&e, "post");
    assert!((flipped - post).abs() < 1e-6 * post, "{flipped} against {post}");
}

#[test]
fn a_ball_seated_in_another_frame_is_meshed_whole() {
    let e = determined("unit mm
use std
use std.solids
use std.place (seats)
component Ball() {
  in std.front {
    c := point
    std.origin distance(9mm, along: u) c
    std.origin distance(0mm, along: v) c
    n := point
    s_ := point
    c distance(2mm, along: v) n
    c distance(0mm, along: u) n
    c distance(0mm, along: u) s_
    c distance(<= 0mm, along: v) s_
    rim := arc(center: c, start: s_, end: n)
    construction across := line(n, s_)
  }
  ball := solid(std.solids.Revolution(face(rim, across), about: across))
}
in std.front {
  at := point
  std.origin distance(0mm, along: u) at
  std.origin distance(70mm, along: v) at
}
b := Ball()
b seats(turn: 0deg, up: std.y, east: std.x) at
");
    let ball = 4. / 3. * PI * 8.;
    let s = ent(&e, "b.ball").i();
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = export::exact(&e.sketch, s, None, None, &say).unwrap();
    let m = gcs_core::brep::mesh::mesh(&exact.solid, 0.01, 0.3).unwrap();
    // the mesh encloses the ball, short of it by no more than its sag allows
    let enclosed: f64 = m.tris.iter().map(|t| {
        let [a, b, c] = t.map(|i| m.pts[i as usize]);
        a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])
    }).sum::<f64>() / 6.;
    assert!(enclosed < ball && enclosed > ball * (1. - 3. * 0.01 / 2.), "{enclosed} against {ball}");
    assert!((volume(&e, "b.ball") - ball).abs() < 1e-6 * ball);
}
