//! **Relations held to a followed trace** (#398 A): `trace(m over t)`, a point of the drawing
//! followed along a declared parameter, is a curve the solve works through — a point held on it
//! at an instant, a line tangent to it, a circle osculating it — and a number of the drawing that
//! does not run with `t` solves through it (`followed::lower`).  The gate beside this file is the
//! corpus rewritten onto `param t`: `jansen.rs`, `generation_examples.rs` and `examples_sv.rs`
//! hold `stride.sv`, `osculating_stride.sv`, `dwell.sv`, `peaucellier.sv` and `jansen.sv` to the
//! numbers their traced components gave.  Here: a rod solved so the coupler curve passes a stated
//! point, against the closed form; the contacts' Jacobians against differences of their residuals;
//! the march holding the relation at every station; printing; and the refusals.

use gcs_core::follow::followed;
use gcs_core::program::Elaborated;

use crate::common::text::solved;
use crate::continuation::closed_c;

/// A four-bar whose rocker's length is a number the document declares unknown, `k`: ground `a`–`d`
/// of 40, crank `a`–`b` of 10 driven by `t`, coupler `b`–`c` of 35, `m` the coupler's midpoint
/// traced as `coupler`; `extra` more statements.
fn four_bar(extra: &str) -> String {
    format!("unit mm\nuse std\nparam t: Angle over (0deg, 360deg)\nparam k: Length hint(24)\n\
        in std.front {{\n  a := point\n  d := point\n  b := point hint((10, 0))\n  \
        c := point hint((40, 25))\n  m := point\n  fix((0, 0)) a\n  fix((40, 0)) d\n  \
        a distance(10) b\n  b distance(35) c\n  c distance(k) d\n  bc := line(b, c)\n  \
        m midpoint bc\n  ground := line(a, d)\n  crank := line(a, b)\n  ground angle(t) crank\n}}\n\
        coupler := trace(m over t)\n{extra}")
}

/// The coupler's midpoint of the closed form at crank angle `t`, the rocker `k` long.
fn midpoint(k: f64, t: f64) -> (f64, f64) {
    let c = closed_c(40.0, 10.0, 35.0, k, t, 1.0).expect("the four-bar closes");
    let (s, co) = t.to_radians().sin_cos();
    ((10.0 * co + c.0) / 2.0, (10.0 * s + c.1) / 2.0)
}

fn unknown(e: &Elaborated, name: &str) -> f64 {
    e.sketch.params[e.sketch.free_vars[name] as usize].value
}

/// Where the coupler's midpoint stands at 90° when the rocker is 25, as a statement: a point
/// fixed there and held on the trace at that instant.
fn passing() -> String {
    let (x, y) = midpoint(25.0, 90.0);
    four_bar(&format!("in std.front {{\n  q := point\n  fix(({x}, {y})) q\n  \
        q coincident(t == 90) coupler\n}}\n"))
}

/// **A rod solves so the coupler curve passes a stated point.**  The rocker's length is no number
/// the march moves, so it is a column of the trace, and the point held on the curve at 90° fixes
/// it: to the length the closed form passes there.  The drawing is DOF 0, its one freedom the
/// crank `t` drives.
#[test]
fn a_rod_solves_so_the_coupler_curve_passes_a_point() {
    let mut e = solved(&passing());
    assert!((unknown(&e, "k") - 25.0).abs() < 1e-8, "the rocker solved to {}", unknown(&e, "k"));
    let d = gcs_core::diagnose::diagnose(&mut e.sketch, Default::default());
    assert_eq!(d.dof, 0);
    assert_eq!(d.parameters.iter().map(|p| p.dof).collect::<Vec<_>>(), vec![Some(1)]);
    // the drawing stands at the start, the rocker's pin where the closed form puts it
    let c = e.sketch.point_xy(e.map.ent_named("c").unwrap().i());
    let want = closed_c(40.0, 10.0, 35.0, 25.0, 0.0, 1.0).unwrap();
    assert!((c.0 - want.0).hypot(c.1 - want.1) < 1e-8, "{c:?} against {want:?}");
}

/// **The march holds it at every station.**  Followed round the turn, the point stays on the
/// curve's instant and the rod where it was solved; the midpoint is the closed form's.
#[test]
fn the_march_holds_the_relation_at_every_station() {
    let e = solved(&passing());
    let k = unknown(&e, "k");
    let f = followed(&e.sketch, "t");
    let f = f.as_ref().as_ref().expect("followed");
    assert!(f.singular.is_none() && f.reached() == 360.0);
    let m = e.map.ent_named("m").unwrap().i();
    let mut sk = e.sketch.clone();
    for s in f.stations.iter().step_by(7) {
        sk.set_x(&s.x);
        assert!((sk.params[sk.free_vars["k"] as usize].value - k).abs() < 1e-8, "at {}", s.at);
        let (got, want) = (sk.point_xy(m), midpoint(25.0, s.at));
        assert!((got.0 - want.0).hypot(got.1 - want.1) < 1e-8, "at {}: {got:?} against {want:?}",
                s.at);
    }
}

/// **The contacts' Jacobians are their derivatives**: a point held on the curve, a line tangent to
/// it and a circle osculating it, each column against a central difference of the residuals —
/// the place, the rod, and what the contact holds.
#[test]
fn a_followed_contacts_jacobian_is_its_derivative() {
    let e = solved(&passing());
    crate::common::fd_jacobian(&e.sketch, 1e-5);
    let src = four_bar("in std.front {\n  g0 := point hint((0, 30))\n  \
        g1 := point hint((10, 30))\n  \
        top := horizontal line(g0, g1)\n  fix(x == 0) g0\n  g0 distance(10, along: u) g1\n  \
        coupler tangent top hint(t: 100)\n  o := point hint((20, 0))\n  \
        osc := circle(center: o) hint(r: 20)\n  coupler curvature(t == 200) osc\n}\n");
    let src = src.replace("param k: Length hint(24)\n", "k := 25\n");
    let e = solved(&src);
    crate::common::fd_jacobian(&e.sketch, 1e-4);
    // the circle's centre is the curve's centre of curvature at 200°
    let (o, r) = (e.map.ent_named("o").unwrap().i(), e.map.ent_named("osc").unwrap());
    let o = e.sketch.point_xy(o);
    let p = midpoint(25.0, 200.0);
    let rr = crate::common::radius_by_differences(|t| midpoint(25.0, t), 200.0, 1e-2);
    assert!(((o.0 - p.0).hypot(o.1 - p.1) - rr).abs() < 1e-5, "the centre is off the curve's");
    assert!((e.sketch.radius_value(r).abs() - rr).abs() < 1e-5);
}

/// A relation held to a followed trace prints back as written and solves to the same numbers.
#[test]
fn a_relation_held_to_a_followed_trace_prints_back() {
    let src = passing();
    let (mut prog, errs) = crate::common::parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let printed = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    assert!(printed.contains("coincident(t == 90) coupler"), "{printed}");
    assert_eq!(unknown(&solved(&printed), "k"), unknown(&solved(&src), "k"));
}

/// **What cannot be lowered is refused, naming why** (E061): what moves with `t` must place every
/// number it moves — a coordinate it leaves free is no number solved through the trace, which is
/// a free value — and something must read the parameter for the drawing to move at all.
#[test]
fn a_trace_that_cannot_be_lowered_is_refused_naming_why() {
    use crate::common::lenient::read;
    let refused = |text: &str, want: &str| {
        let (_, said) = read(text);
        assert!(said.iter().any(|m| m.starts_with("E061") && m.contains(want)), "{text}\n{said:?}");
    };
    // the ground's far end free along a circle: one number the rows leave free
    refused(&passing().replace("fix((40, 0)) d", "a distance(40) d"),
            "leave 1 of the numbers they place free: a number solved through the trace is a free \
             value");
    // nothing reads `t`: the drawing does not move with it
    refused(&passing().replace("ground angle(t) crank", "ground angle(30deg) crank"),
            "nothing the drawing states reads `t`");
}
