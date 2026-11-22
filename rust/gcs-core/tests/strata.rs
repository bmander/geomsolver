//! **A body's written terms are built once** (#996): the stratum a body's rounds read — its stock
//! and what its statements join to it and cut from it — is made once for the drawing, and the
//! body's own build goes on from it to make the rounds, as each round that rolls a ball against it
//! does.  Made again for the body and for each such round, a round on a body of many terms cost
//! that body's Booleans over again, and a part's dozen rounds a dozen of its builds.  Held by
//! count, never by time: the Booleans a round adds do not grow with the terms of the body it
//! rounds (`brep::boolean::combined_so_far`, this thread's).
use gcs_core::brep::boolean::combined_so_far;
use gcs_core::brep::props;

use crate::common::{ent, text::solved};

/// A plate with a post standing on it and a bar through the post, `bosses` small blocks joined to
/// the plate well away from both, and — where `rounded` — a round where the post meets the bar
/// (a ball rolled along their saddle) and one where it stands on the plate (a ring).
fn part(bosses: usize, rounded: bool) -> String {
    let mut s = String::from("unit mm\nuse std\nuse std.solids\nuse std.solids (union)\nuse std.rounds (round)\n\
        in std.top {\n  a := point\n  b := point\n  c := point\n  d := point\n\
        fix((-30, -20)) a\n  fix((30, -20)) b\n  fix((30, 20)) c\n  fix((-30, 20)) d\n\
        plate_f := face(a, b, c, d, -> close)\n  pc := point\n  fix((0, 0)) pc\n\
        post_c := circle(center: pc) hint(r: 6)\n  radius(6mm) post_c\n}\n\
        plate := solid(std.solids.Prism(plate_f, from: 0mm, to: 10mm))\n\
        post := solid(std.solids.Prism(face(post_c), from: 10mm, to: 40mm))\n\
        in std.side {\n  bc := point\n  fix((0, 28)) bc\n  bar_c := circle(center: bc) hint(r: 4)\n  radius(4mm) bar_c\n}\n\
        bar := solid(std.solids.Prism(face(bar_c), from: -15mm, to: 15mm))\n");
    for i in 0..bosses {
        let x = -25 + 6 * i as i64;
        s += &format!("in std.top {{\n  q{i}0 := point\n  q{i}1 := point\n  q{i}2 := point\n  q{i}3 := point\n\
            fix(({x}, -18)) q{i}0\n  fix(({}, -18)) q{i}1\n  fix(({}, -15)) q{i}2\n  fix(({x}, -15)) q{i}3\n\
            boss{i}_f := face(q{i}0, q{i}1, q{i}2, q{i}3, -> close)\n}}\n\
            boss{i} := solid(std.solids.Prism(boss{i}_f, from: 10mm, to: 13mm))\n", x + 3, x + 3);
    }
    s += "part := solid(plate)\npost union part\nbar union part\n";
    for i in 0..bosses {
        s += &format!("boss{i} union part\n");
    }
    if rounded {
        s += "saddle := part.post round(r: 1mm) part.bar\nfoot := part.post round(r: 1mm) part.plate\n";
    }
    s
}

/// `part`'s exact B-rep built from nothing remembered, and the Booleans that took, on this thread.
fn built(bosses: usize, rounded: bool) -> (usize, f64, usize) {
    let e = solved(&part(bosses, rounded));
    let i = ent(&e, "part").i();
    e.sketch.caches.clear();
    let before = combined_so_far();
    let x = e.sketch.exact_solid(i).unwrap_or_else(|why| panic!("{bosses} bosses, rounded {rounded}: {why}"));
    (combined_so_far() - before, props::volume(&x.brep), x.brep.faces.len())
}

#[test]
fn the_booleans_a_round_adds_do_not_grow_with_the_body_it_rounds() {
    let (plain1, v1, f1) = built(1, false);
    let (plain5, v5, f5) = built(5, false);
    let (round1, w1, g1) = built(1, true);
    let (round5, w5, g5) = built(5, true);
    eprintln!("Booleans: plain {plain1} and {plain5}, rounded {round1} and {round5}");
    // the four bosses more are four Booleans more, and the same four more where the body is rounded:
    // its written terms made once, not once more for the body and for the round rolled against them
    assert_eq!(plain5 - plain1, 4, "a boss is one Boolean");
    assert_eq!(round5 - plain5, round1 - plain1, "the rounds made the body's terms again: {round1} and {round5}");
    assert!(round1 > plain1, "the rounds were made");
    // and what is built is the body with its rounds: each boss its 3 × 3 × 3 block and five faces
    // more, each round material of its own
    assert!((v5 - v1 - 4. * 27.).abs() < 1e-6 * v1, "{v1} {v5}");
    assert!((w5 - w1 - 4. * 27.).abs() < 1e-6 * w1, "{w1} {w5}");
    assert_eq!((f5 - f1, g5 - g1), (20, 20));
    assert!(w1 > v1 && g1 > f1, "{v1} {w1} {f1} {g1}");
}
