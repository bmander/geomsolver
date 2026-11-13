//! **A region solid in a `ring` is one a copy** (#904, shaft_drive README finding 20).  A ball
//! written as a set, `ball := solid({ p | p distance(<= 2mm) c })` or `solid(std.sets.Ball(c, r:
//! 2mm))`, inside a `ring`, was refused twice: E023, "a point is not drawn in `std.top.origin`'s
//! view", the region's hidden probe (`{solid}#probe`, held in space) taken for a point the ring
//! turns; and E080, "a solid is a region: a set whose body bounds a number (`std.sets.Ball`, …)",
//! for every copy past the first, whose relations are its representative's turned and so stated
//! none of the bounds its set puts on its own probe.  A probe is the solid's own, turned with
//! nothing; and a copy's relation bounding its own probe is stated, a term of the copy's solid
//! read over its turned points and never a row.
use crate::common::read;
use gcs_core::brep::{export, props, sweep::Say};
use std::f64::consts::PI;

fn ring(set: &str) -> String {
    format!("unit mm
use std
use std.sets
use std.place (right_of)
in std.top {{
  ring 6 about std.top.origin {{
    c := point
    c right_of(d: 9mm) std.top.origin
    ball := solid({set})
  }}
}}
")
}

/// Each copy of the ring's ball built exactly: a ball 2 mm across its radius, its centre 9 mm out
/// and turned a sixth of a turn a copy.
fn each_copy_is_a_ball_turned(src: &str) {
    let mut e = read(src);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let ball = 4. / 3. * PI * 8.;
    let mut seen = Vec::new();
    for k in 0..6 {
        let names: Vec<String> = (0..e.sketch.solids.len()).map(|i| e.sketch.solid_name(i)).collect();
        let s = names.iter().position(|n| *n == format!("ball[{k}]")).unwrap_or_else(|| panic!("{names:?}"));
        let x = export::exact(&e.sketch, s, None, None, &say).unwrap_or_else(|r| panic!("ball[{k}]: {r:?}"));
        let v = props::volume(&x.solid);
        assert!((v - ball).abs() < 1e-6 * ball, "ball[{k}]: {v} against {ball}");
        let (lo, hi) = x.solid.bounds();
        let c = [(lo[0] + hi[0]) / 2., (lo[1] + hi[1]) / 2.];
        assert!(((c[0] * c[0] + c[1] * c[1]).sqrt() - 9.).abs() < 1e-6, "ball[{k}] centred at {c:?}");
        seen.push(c[1].atan2(c[0]).to_degrees().rem_euclid(360.));
    }
    seen.sort_by(f64::total_cmp);
    for (k, a) in seen.iter().enumerate() {
        let want = seen[0] + 60. * k as f64;
        assert!((a - want).abs() < 1e-6, "{seen:?}");
    }
}

#[test]
fn a_ball_written_as_a_set_is_one_ball_a_copy() {
    each_copy_is_a_ball_turned(&ring("{ p | p distance(<= 2mm) c }"));
}

#[test]
fn a_ball_of_std_sets_is_one_ball_a_copy() {
    each_copy_is_a_ball_turned(&ring("std.sets.Ball(c, r: 2mm)"));
}
