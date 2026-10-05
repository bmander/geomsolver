//! What is left of the datum a plane once was (issue #47, item 6 folded `frame` into `plane`;
//! `docs/planes-plan.md` made a plane two rays and a place): a plane is measured where it
//! stands, at its own origin, and the word `frame` is refused with the spelling it became.
use gcs_core::model::{distance_between, EntRef, Sketch};
use gcs_core::plane::Basis;
use crate::common::parse;

/// A plane sorts last in `measure_order`, so every pair holding one reaches the swept arm with
/// the plane as `b` — above the arms that would ask it for a centre and a radius it does not
/// have.  Measured at its origin, `(0, 0)` in its own coordinates.
#[test]
fn every_pair_with_a_plane_measures_from_its_origin() {
    let mut sk = Sketch::new();
    let f = sk.fixed_plane(Basis::page(), "f");
    let p = sk.point(3.0, 4.0, false, "p");
    let l = sk.line_xy(-50.0, 5.0, 100.0, 5.0, "l");
    let ci = sk.circle(p, 2.0, "ci");
    let f2 = sk.fixed_plane(Basis { o: [0.0, 0.0, 9.0], ..Basis::page() }, "f2");
    let fr = EntRef::plane(f);
    // each is the distance from (0, 0) to the thing, and none of them panics
    assert!((distance_between(&sk, fr, EntRef::point(p)) - 5.0).abs() < 1e-12);
    assert!((distance_between(&sk, fr, EntRef::line(l)) - 5.0).abs() < 1e-12);
    assert!((distance_between(&sk, fr, EntRef::circle(ci)) - 3.0).abs() < 1e-12);
    assert!(distance_between(&sk, fr, EntRef::plane(f2)).abs() < 1e-12);
    // and the pair reads the same measured either way round
    assert_eq!(
        distance_between(&sk, EntRef::line(l), fr),
        distance_between(&sk, fr, EntRef::line(l))
    );
}

/// The word itself is gone: a document written with it is told what to write instead, at the
/// declaration and at a component's formal alike.
#[test]
fn the_word_frame_is_refused_with_the_spelling_it_became() {
    let (_, errs) = parse("o := point hint(x: 0, y: 0)\nq := point hint(x: 4, y: 0)\nframe f(origin: o, toward: q)\n");
    assert_eq!(errs.len(), 1, "{errs:?}");
    assert!(errs[0].message.contains("folded into `plane`"), "{}", errs[0].message);
    let (_, errs) = parse("component c(f: frame) {\n  p := point\n}\n");
    assert!(errs.iter().any(|e| e.message.contains("`f: plane`")), "{errs:?}");
}
