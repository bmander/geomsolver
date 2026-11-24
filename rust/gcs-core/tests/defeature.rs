//! **Faces deleted** (`brep::defeature::deleted`, docs/contract/brep.md): faces taken out of a solid
//! and the gap closed by the faces round them run on along their own surfaces. Held to closed forms,
//! as brep-bench's `edit/` cases are: a convex round and a concave one, a hole and a boss, a flat on
//! a ball, an L-block's step, a notch whose side was two faces — each the solid it was cut or added
//! from, its faces the plain ones it had, valid; and a face nothing closes over refused, saying so.
use gcs_core::brep::boolean::{combined, Op};
use gcs_core::brep::build::{block, disc, kept, prism, sphere, Profile, ProfileEdge};
use gcs_core::brep::defeature::deleted;
use gcs_core::brep::fillet::roll;
use gcs_core::brep::geom::{tolerance, Surface, V};
use gcs_core::brep::props::{area, volume};
use gcs_core::brep::query::{Located, Place};
use gcs_core::brep::topo::Brep;
use std::f64::consts::PI;

fn close(what: &str, a: f64, b: f64) { assert!((a - b).abs() <= 1e-9 * b.abs().max(1.), "{what}: {a} against {b}"); }

/// The faces of `b` each point lies inside.
fn through(b: &Brep, on: &[V]) -> Vec<usize> {
    let tol = tolerance(b.size());
    let l = Located::new(b, tol);
    on.iter().map(|&p| (0..b.faces.len()).find(|&f| l.face_gap(f, p) <= tol && l.face_place(f, p) == Place::In)
        .unwrap_or_else(|| panic!("no face through {p:?}"))).collect()
}

/// `b` with `faces` deleted, valid, of volume `v`, area `a` and `n` faces.
fn deletes_to(b: &Brep, faces: &[usize], v: f64, a: f64, n: usize) -> Brep {
    let tol = tolerance(b.size());
    let out = deleted(b, faces, tol).unwrap_or_else(|e| panic!("{e}"));
    out.check(tol).unwrap_or_else(|e| panic!("{e}"));
    close("volume", volume(&out), v);
    close("area", out.faces.iter().map(|f| area(&out, f)).sum(), a);
    assert_eq!(out.faces.len(), n, "faces: {:?}", out.faces.iter().map(|f| f.surface.kind()).collect::<Vec<_>>());
    out
}

/// `b` with the edge through `on` rounded to `r`, its piece cut from a convex edge and filled into
/// a concave one; its faces named first, each its own.
fn rounded(b: &Brep, on: V, r: f64) -> Brep {
    let mut b = b.clone();
    for (i, f) in b.faces.iter_mut().enumerate() { f.name = format!("face {i}") }
    let round = roll(&b, on, r, tolerance(b.size())).unwrap();
    combined(&b, &round.piece, if round.concave { Op::Union } else { Op::Cut }, 0.).unwrap()
}

fn cylinder(c: V, r: f64, h: f64) -> Brep { prism(&disc(c, [0., 0., 1.], r), 0., h).unwrap() }

/// A block's convex edge rounded, the round deleted: the box again, its six planes, the top run on
/// over the corner it had lost and joined to its own piece.
#[test]
fn a_convex_rounds_gap_is_taken_in() {
    let b = rounded(&block([0.; 3], [20., 10., 10.]).unwrap(), [20., 5., 10.], 2.);
    let h = 2. * std::f64::consts::FRAC_1_SQRT_2;
    let out = deletes_to(&b, &through(&b, &[[18. + h, 5., 8. + h]]), 2000., 1000., 6);
    assert!(out.faces.iter().all(|f| matches!(f.surface, Surface::Plane(..))));
}

/// A boss's root rounded, the round deleted: what the round filled in given up, the plate's top run
/// in and the boss's side run down until they meet at its foot.
#[test]
fn a_concave_rounds_gap_is_given_up() {
    let b = combined(&cylinder([0., 0., -4.], 8., 4.), &cylinder([0.; 3], 3., 8.), Op::Union, 0.).unwrap();
    let b = rounded(&b, [3., 0., 0.], 1.);
    let round: Vec<usize> = (0..b.faces.len()).filter(|&f| matches!(b.faces[f].surface, Surface::Torus(..))).collect();
    assert_eq!(round.len(), 1);
    deletes_to(&b, &round, 328. * PI, 240. * PI, 5);
}

/// A hole's wall deleted: the top and the bottom close over it; a boss deleted: the plate's top
/// closes over its foot.
#[test]
fn a_hole_and_a_boss_close_over() {
    let drilled = combined(&block([0.; 3], [20., 10., 10.]).unwrap(), &cylinder([10., 5., -1.], 3., 12.), Op::Cut, 0.).unwrap();
    deletes_to(&drilled, &through(&drilled, &[[13., 5., 5.]]), 2000., 1000., 6);
    let bossed = combined(&cylinder([0., 0., -4.], 8., 4.), &cylinder([0.; 3], 3., 8.), Op::Union, 0.).unwrap();
    deletes_to(&bossed, &through(&bossed, &[[3., 0., 4.], [1., 0., 8.]]), 256. * PI, 192. * PI, 3);
}

/// A ball's flat deleted: the ball whole, one face.
#[test]
fn a_flat_on_a_ball_closes_it() {
    let b = kept(&sphere([0.; 3], [0., 0., 1.], 5.).unwrap(), [0., 0., 3.], [0., 0., -1.], 0.).unwrap();
    deletes_to(&b, &through(&b, &[[1., 0., 3.]]), 500. * PI / 3., 100. * PI, 1);
}

/// An L-block's step, riser and tread, deleted: the faces beside them, which never met, run on
/// until they do, the box again.
#[test]
fn faces_that_never_met_run_on_to_meet() {
    let l = [[0., 0., 0.], [20., 0., 0.], [20., 5., 0.], [5., 5., 0.], [5., 15., 0.], [0., 15., 0.]];
    let edges = (0..6).map(|k| ProfileEdge::Line { a: l[k], b: l[(k + 1) % 6] }).collect();
    let b = prism(&Profile { origin: [0.; 3], normal: [0., 0., 1.], loops: vec![edges], names: Vec::new() }, 0., 8.).unwrap();
    deletes_to(&b, &through(&b, &[[5., 10., 4.], [12., 5., 4.]]), 2400., 2. * 300. + 2. * 160. + 2. * 120., 6);
}

/// A half-round notch in a block's side, which parts the side in two: the notch deleted, the two
/// pieces of the side, on one plane, one rectangle again.
#[test]
fn faces_a_gap_parted_are_one_again() {
    let b = combined(&block([0.; 3], [20., 10., 10.]).unwrap(), &cylinder([10., 0., -1.], 3., 12.), Op::Cut, 0.).unwrap();
    deletes_to(&b, &through(&b, &[[10., 3., 5.]]), 2000., 1000., 6);
}

/// A box's top deleted: its four sides run on up and never meet past it, so no gap closes, and
/// that is said.
#[test]
fn a_face_nothing_closes_over_is_refused() {
    let b = block([0.; 3], [10., 10., 10.]).unwrap();
    let e = deleted(&b, &through(&b, &[[5., 5., 10.]]), tolerance(b.size())).unwrap_err();
    assert!(e.msg.contains("closes no gap"), "{e}");
}
