//! **An arc of a half turn placed outright** (#961, the primer's 1.4): its ends a diameter apart,
//! so the half turn is one fact and the arc four numbers.  The two spellings the primer gives —
//! the centre on the chord, and the radii running opposite — solve to the one figure from any
//! seed, regular and with nothing said twice; each spelling it warns off is read as it says: the
//! three points held is said twice, the ends held alone leave the centre its slide, the
//! `midpoint` says the radius twice, and the ends with a radius, or an end held by the coordinate
//! the circle runs square to, are a double root — rigid, blocked at second order, never a freedom
//! and a surplus radius.
use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions, Diagnosis, State};
use gcs_core::model::Sketch;

use crate::common::{ent, read, solved};

/// The places a test draws over: a chord's midpoint, its radius and the bearing of its start.
const PLACES: &[((f64, f64), f64, f64)] = &[
    ((0.0, 0.0), 10.0, 0.0),
    ((37.5, -12.0), 0.4, 30.0),
    ((-140.0, 65.0), 250.0, 137.0),
    ((5.0, 5.0), 3.0, 270.0),
];

/// `m + r·(cos θ, sin θ)`, θ in degrees.
fn on(m: (f64, f64), r: f64, deg: f64) -> (f64, f64) {
    let t = deg.to_radians();
    (m.0 + r * t.cos(), m.1 + r * t.sin())
}

fn v(p: (f64, f64)) -> String {
    format!("({}, {})", p.0, p.1)
}

/// `src` read and solved, its diagnosis, and the sketch.
fn read_diagnosed(src: &str) -> (gcs_core::program::Elaborated, Sketch, Diagnosis) {
    let e = read(src);
    let mut sk = solved(&e);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    (e, sk, d)
}

fn at(e: &gcs_core::program::Elaborated, sk: &Sketch, n: &str) -> (f64, f64) {
    sk.point_xy(ent(e, n).i())
}

fn near(a: (f64, f64), b: (f64, f64), tol: f64) -> bool {
    (a.0 - b.0).hypot(a.1 - b.1) <= tol
}

/// Determined, regular, and nothing in it said twice or blocked.
fn well(d: &Diagnosis, src: &str) {
    assert_eq!((d.dof, d.status), (0, State::Well), "{}\n{src}", gcs_core::diagnose::summary(d));
    assert_eq!(d.numeric_rank, Some(d.structural_rank), "{src}");
    assert_eq!(d.shaky, 0, "{src}");
    assert!(d.over.is_empty() && d.implied.is_empty() && d.said_twice.is_empty(), "{src}");
}

/// The arc's ends a diameter apart through its centre, the start at `s`, the end at `e`: a half
/// turn, counter-clockwise from the start.
fn half_turn(sk: &Sketch, c: (f64, f64), s: (f64, f64), e: (f64, f64), r: f64) {
    let tol = 1e-9 * r.max(1.0);
    assert!(near(e, (2.0 * c.0 - s.0, 2.0 * c.1 - s.1), tol), "c {c:?} s {s:?} e {e:?}");
    assert!((sk.params[sk.arcs[0].radius as usize].value - r).abs() <= tol);
}

/// Both ends held, the centre stated on the chord: the centre is the chord's midpoint, from a
/// seed on either side of the chord or none, at every place, size and bearing.
#[test]
fn the_centre_on_the_chord_places_a_half_turn() {
    for &(m, r, deg) in PLACES {
        let (s, e) = (on(m, r, deg), on(m, r, deg + 180.0));
        for seed in [Some(on(m, r / 3.0, deg + 90.0)), Some(on(m, r / 3.0, deg - 90.0)), None] {
            let seeded = seed.map_or(String::new(), |p| format!(" hint({})", v(p)));
            let src = format!(
                "use std\nin std.front {{\n  c := point{seeded}\n  s := point\n  e := point\n  \
                 fix({}) s\n  fix({}) e\n  k := arc(center: c, start: s, end: e)\n  \
                 construction chord := line(s, e)\n  c coincident chord\n}}\n",
                v(s),
                v(e)
            );
            let (el, sk, d) = read_diagnosed(&src);
            well(&d, &src);
            let c = at(&el, &sk, "c");
            assert!(near(c, m, 1e-9 * r.max(1.0)), "{c:?} {m:?}\n{src}");
            half_turn(&sk, c, at(&el, &sk, "s"), at(&el, &sk, "e"), r);
        }
    }
}

/// The centre and the start held, the radii stated to run opposite: the end is the start's
/// reflection through the centre, from a seed beside the start as from one across from it — the
/// one root, where the end held by a coordinate has two.
#[test]
fn the_radii_opposite_place_a_half_turn_from_the_centre() {
    for &(m, r, deg) in PLACES {
        let s = on(m, r, deg);
        for seed in [on(m, r, deg + 10.0), on(m, r, deg + 170.0), on(m, 0.5 * r, deg + 90.0)] {
            let src = format!(
                "use std\nin std.front {{\n  c := point\n  s := point\n  e := point hint({})\n  \
                 fix({}) c\n  fix({}) s\n  k := arc(center: c, start: s, end: e)\n  \
                 construction r0 := line(c, s)\n  construction r1 := line(c, e)\n  \
                 r0 parallel(sense: opposite) r1\n}}\n",
                v(seed),
                v(m),
                v(s)
            );
            let (el, sk, d) = read_diagnosed(&src);
            well(&d, &src);
            half_turn(&sk, at(&el, &sk, "c"), at(&el, &sk, "s"), at(&el, &sk, "e"), r);
        }
    }
}

/// A half turn's ends held, then one more statement or none.  `seed` is the centre's.
fn ends(m: (f64, f64), r: f64, deg: f64, seed: (f64, f64), more: &str) -> String {
    format!(
        "use std\nin std.front {{\n  c := point hint({})\n  s := point\n  e := point\n  \
         fix({}) s\n  fix({}) e\n  k := arc(center: c, start: s, end: e)\n{more}}}\n",
        v(seed),
        v(on(m, r, deg)),
        v(on(m, r, deg + 180.0))
    )
}

/// The three points held say the radius twice, as of any arc.
#[test]
fn the_three_points_held_say_the_radius_twice() {
    for &(m, r, deg) in PLACES {
        let src = ends(m, r, deg, m, &format!("  fix({}) c\n", v(m)));
        let (_, _, d) = read_diagnosed(&src);
        assert_eq!(d.dof, 0, "{src}");
        assert_eq!(d.n_said_twice, 1, "{}\n{src}", gcs_core::diagnose::summary(&d));
    }
}

/// The ends held alone leave the centre its slide along their bisector: the half turn is the one
/// fact not yet said.
#[test]
fn the_ends_held_alone_leave_the_centre_free() {
    for &(m, r, deg) in PLACES {
        let src = ends(m, r, deg, on(m, r / 3.0, deg + 90.0), "");
        let (_, _, d) = read_diagnosed(&src);
        assert_eq!((d.dof, d.status), (1, State::Under), "{src}");
    }
}

/// The centre stated the chord's midpoint says the radius twice: the arc's equal radii already
/// put it on the bisector, so of the midpoint's two rows only the one along the chord is news.
#[test]
fn the_midpoint_says_the_radius_twice() {
    for &(m, r, deg) in PLACES {
        let more = "  construction chord := line(s, e)\n  c midpoint chord\n";
        let src = ends(m, r, deg, on(m, r / 3.0, deg + 90.0), more);
        let (_, sk, d) = read_diagnosed(&src);
        assert_eq!(d.dof, 0, "{src}");
        assert_eq!(d.n_said_twice, 1, "{src}");
        let kinds: Vec<CKind> =
            d.said_twice.iter().filter_map(|&c| sk.constraint(c)).map(|c| c.kind).collect();
        assert!(kinds.contains(&CKind::Midpoint), "{kinds:?}\n{src}");
    }
}

/// The ends held and the radius stated: the radius is the least a circle through ends a diameter
/// apart can have, so the circles about the ends touch rather than cross — a double root.  Solved
/// from on it, the centre's slide along the bisector is blocked at second order: DOF 0 and
/// nothing over (where the screen asked for a tangency, this read DOF 1 with the radius over).
/// From off it the solve stops a square root of its tolerance short and reads DOF 0 too.
#[test]
fn the_ends_and_a_radius_are_a_double_root() {
    for &(m, r, deg) in PLACES {
        let more = format!("  radius({r}) k\n");
        let src = ends(m, r, deg, m, &more);
        let (el, sk, d) = read_diagnosed(&src);
        assert_eq!((d.dof, d.status), (0, State::Well), "{}\n{src}", gcs_core::diagnose::summary(&d));
        assert_eq!(d.shaky, 1, "{src}");
        assert!(d.over.is_empty() && d.implied.is_empty(), "{src}");
        assert!(near(at(&el, &sk, "c"), m, 1e-9 * r.max(1.0)), "{src}");

        let src = ends(m, r, deg, on(m, r / 3.0, deg + 90.0), &more);
        let (el, sk, d) = read_diagnosed(&src);
        assert_eq!(d.dof, 0, "{src}");
        assert!(near(at(&el, &sk, "c"), m, 1e-4 * r), "{:?}\n{src}", at(&el, &sk, "c"));
    }
}

/// The end held by the coordinate the circle runs square to there is the circle touching that
/// line: from on it, blocked at second order, not a freedom.  Held by the one it crosses, it
/// solves, with two roots — seeded beside the start, it lands on the start, a whole turn.
#[test]
fn an_end_held_by_a_coordinate() {
    let held = |hold: &str, seed: &str| {
        format!(
            "use std\nin std.front {{\n  c := point\n  s := point\n  e := point hint({seed})\n  \
             fix((0, 0)) c\n  fix((10, 0)) s\n  {hold} e\n  k := arc(center: c, start: s, end: e)\n}}\n"
        )
    };
    let src = held("fix(x == -10mm)", "(-10, 0)");
    let (_, _, d) = read_diagnosed(&src);
    assert_eq!((d.dof, d.shaky), (0, 1), "{src}");
    assert!(d.over.is_empty(), "{src}");

    let src = held("fix(y == 0mm)", "(-10, 1)");
    let (el, sk, d) = read_diagnosed(&src);
    well(&d, &src);
    assert!(near(at(&el, &sk, "e"), (-10.0, 0.0), 1e-9), "{src}");
    let src = held("fix(y == 0mm)", "(9, 1)");
    let (el, sk, _) = read_diagnosed(&src);
    assert!(near(at(&el, &sk, "e"), (10.0, 0.0), 1e-9), "the other root\n{src}");
}

/// The screen asks for no tangency, and still eats no real freedom: a rectangle's fourth
/// right angle is a theorem, and its four genuine motions — two of place, width and height —
/// all settle where they step.
#[test]
fn the_screen_blocks_no_real_freedom() {
    let src = "use std\nin std.front {\n  a := point\n  b := point hint((30, 1))\n  \
               c := point hint((31, 20))\n  d := point hint((1, 21))\n  ab := line(a, b)\n  \
               bc := line(b, c)\n  cd := line(c, d)\n  da := line(d, a)\n  horizontal ab\n  \
               ab perpendicular bc\n  bc perpendicular cd\n  cd perpendicular da\n  \
               da perpendicular ab\n}\n";
    let (_, _, d) = read_diagnosed(src);
    assert_eq!((d.dof, d.shaky), (4, 0), "{}", gcs_core::diagnose::summary(&d));
    assert!(!d.implied.is_empty());
}
