//! Two words the spiral-bevel layout asked for (`docs/spiral-bevel-layout-plan.md`, language
//! changes 4 and 6): an angle stated as another angle, `l1 angle(l3, l4) l2`, and an arc's length
//! along itself, `length(L) a`.  Each is checked against a closed form the document never
//! states, with its free twin and its printed spelling; their refusals are in `refusals.rs`.

use gcs_core::constraints::CKind;
use gcs_core::program::{to_program, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use crate::common::{lenient::read_legacy as read, parse_legacy as parse, said};
use std::f64::consts::PI;

/// Elaborated cleanly and solved.
fn solved(src: &str) -> Elaborated {
    let (mut e, d) = read(src);
    assert!(e.ok(), "{d:?}");
    assert!(solve(&mut e.sketch, SolveOpts::default()).success, "{src}");
    e
}

fn at(e: &Elaborated, name: &str) -> (f64, f64) {
    let r = e.map.ent_named(name).unwrap_or_else(|| panic!("no {name}"));
    e.sketch.point_xy(r.i())
}

fn near(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).hypot(a.1 - b.1) < 1e-7
}

/* -- arc length ---------------------------------------------------------------------------- */

const QUARTER: &str = "\
use std
use std.place (horizontal)
in std.front {
o := point
s := point hint((10, 0))
e := point hint((3, 9))
a := arc(o, s, e)
radius(10) a
o horizontal s
fix((0, 0)) o
}
";

/// An arc of radius 10 whose length is 5π sweeps a quarter turn: its end lands straight above
/// the centre, from a seed that is not there.
#[test]
fn an_arc_of_radius_ten_and_length_five_pi_sweeps_a_quarter_turn() {
    let e = solved(&format!("{QUARTER}length(5 * pi) a\n"));
    assert!(near(at(&e, "e"), (0.0, 10.0)), "{:?}", at(&e, "e"));
    let (a0, a1) = e.sketch.arc_angles(0);
    assert!((a1 - a0 - PI / 2.0).abs() < 1e-9);
    // past half a turn is the same statement, read the way the arc runs
    let e = solved(&format!("{QUARTER}length(15 * pi) a\n"));
    assert!(near(at(&e, "e"), (0.0, -10.0)), "{:?}", at(&e, "e"));
    let (a0, a1) = e.sketch.arc_angles(0);
    assert!((a1 - a0 - 1.5 * PI).abs() < 1e-9);
}

/// The length is the radius times the sweep, so a stated length on a free radius and a stated
/// sweep make the radius: a quarter turn 5π long is a radius of 10.
#[test]
fn a_length_and_a_sweep_make_the_radius() {
    let src = "\
use std
in std.front {
o := point
s := point hint((14, 0))
e := point hint((0, 14))
a := arc(o, s, e)
l1 := line(o, s)
l2 := line(o, e)
horizontal l1
l1 angle(90deg) l2
length(5 * pi) a
fix((0, 0)) o
}
";
    let e = solved(src);
    assert!((e.sketch.radius_value(e.map.ent_named("a").unwrap()) - 10.0).abs() < 1e-9);
}

/// Written in terms of one unknown, two arcs' lengths are tied: the first is fixed by its
/// grounded end, and the second — twice the radius — sweeps half as far.
#[test]
fn two_arc_lengths_tied_by_a_free_variable() {
    let src = "\
use std
use std.place (horizontal)
in std.front {
o1 := point
s1 := point hint((10, 0))
e1 := point
a1 := arc(o1, s1, e1)
o1 horizontal s1
fix((0, 0)) o1
fix((0, 10)) e1
o2 := point
s2 := point hint((70, 0))
e2 := point hint((60, 15))
a2 := arc(o2, s2, e2)
radius(20) a2
o2 horizontal s2
fix((50, 0)) o2
}
param s: Length
in std.front {
length(s) a1
length(s) a2
}
";
    let (mut e, d) = read(src);
    assert!(e.ok(), "{d:?}");
    assert!(d.is_empty(), "{d:?}");
    assert_eq!(gcs_core::diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    let h = 20.0 * (PI / 4.0).cos();
    assert!(near(at(&e, "e2"), (50.0 + h, h)), "{:?}", at(&e, "e2"));
    assert!(e.sketch.user_constraints().iter().all(|c| c.kind != CKind::ArcLength || c.free.is_some()));
}

/// Drawn, it is an arc concentric with the one it measures, swept exactly as far, the number
/// marked apart from a chord.
#[test]
fn an_arc_length_is_drawn_as_a_concentric_arc() {
    let e = solved(&format!("{QUARTER}length(5 * pi) a\n"));
    let id = e.sketch.user_constraints().iter().find(|c| c.kind == CKind::ArcLength).unwrap().id;
    let k = gcs_core::callout::layout(&e.sketch, 0.1).into_iter().find(|k| k.id == id)
        .expect("an arc length has a figure");
    assert!(k.text.starts_with('⌒'), "{}", k.text);
    assert_eq!(k.arcs.len(), 1);
    let arc = k.arcs[0];
    assert!(near(arc.c, (0.0, 0.0)) && arc.r > 10.0);
    assert!((arc.a1 - arc.a0 - PI / 2.0).abs() < 1e-9);
    assert!(gcs_core::callout::frame(&e.sketch, e.sketch.constraint(id).unwrap()).is_some());
}

/* -- an angle as another angle ------------------------------------------------------------- */

const FAN: &str = "\
use std
in std.front {
a := point
b := point hint((40, 0))
c := point hint((10, 30))
d := point hint((25, 10))
ab := line(a, b)
ac := line(a, c)
ad := line(a, d)
horizontal ab
a distance(40) b
a distance(30) c
a distance(20) d
ab angle(60deg) ac
fix((0, 0)) a
}
";

/// The angle from `ab` to `ad` stated as the angle from `ad` to `ac` makes `ad` the bisector:
/// 30° up from a line at 0° to one at 60°.
#[test]
fn equal_angles_make_a_bisector() {
    let e = solved(&format!("{FAN}ab angle(ad, ac) ad\n"));
    let r = 30f64.to_radians();
    assert!(near(at(&e, "d"), (20.0 * r.cos(), 20.0 * r.sin())), "{:?}", at(&e, "d"));
    let (mut e, _) = read(&format!("{FAN}ab angle(ad, ac) ad\n"));
    assert_eq!(gcs_core::diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::EqualAngle));
}

/// Directed like `angle`: stated the other way round, the equal angle is on the other side.
/// `sense: cw` turns the second pair's angle, so `ad` is `ac`'s mirror image in `ab`.
#[test]
fn equal_angles_are_directed_and_sense_cw_is_the_mirror_image() {
    let e = solved(&format!("{FAN}ab angle(ab, ac, sense: cw) ad\n"));
    let r = (-60f64).to_radians();
    assert!(near(at(&e, "d"), (20.0 * r.cos(), 20.0 * r.sin())), "{:?}", at(&e, "d"));
    // stated ccw it is the same angle again: `ad` on `ac`
    let e = solved(&format!("{FAN}ab angle(ab, ac) ad\n"));
    let r = 60f64.to_radians();
    assert!(near(at(&e, "d"), (20.0 * r.cos(), 20.0 * r.sin())), "{:?}", at(&e, "d"));
}

/// It replaces the shared unknown the same statement used to need, and says what that did: the
/// two drawings land in one place.
#[test]
fn equal_angles_say_what_a_shared_free_variable_said() {
    let word = solved(&format!("{FAN}ab angle(ad, ac) ad\n"));
    let (mut free, d) =
        read(&format!("{FAN}param beta: Angle\nab angle(beta) ad\nad angle(beta) ac\n"));
    assert!(d.is_empty(), "{d:?}");
    assert!(solve(&mut free.sketch, SolveOpts::default()).success);
    assert!(near(at(&word, "d"), at(&free, "d")));
}

/* -- the spellings ------------------------------------------------------------------------- */

/// Each prints the way it is written — the written form and the one lifted from the sketch —
/// and the printed text elaborates to the same statements.
#[test]
fn both_words_print_back_as_written() {
    let src = format!(
        "{FAN}ab angle(ad, ac) ad\nab angle(ab, ac, sense: cw) ad\n\
         in std.front {{\no := point hint((100, 0))\ns := point hint((110, 0))\n\
         e := point hint((100, 10))\nk := arc(o, s, e)\n}}\nlength(5 * pi) k\n"
    );
    let (e, d) = read(&src);
    assert!(e.ok(), "{d:?}");
    let words = said(&e);
    for w in ["ab angle(ad, ac) ad", "ab angle(ab, ac, sense: cw) ad"] {
        assert!(words.iter().any(|s| s == w), "{w}: {words:?}");
    }
    assert!(words.iter().any(|s| s.starts_with("length(5 * pi") && s.ends_with(") k")), "{words:?}");
    // the written form
    let (mut p, _) = parse(&src);
    let printed = gcs_core::syntax::render_flat(&mut p).unwrap().to_string();
    assert!(printed.contains("ab angle(ad, ac) ad"), "{printed}");
    assert!(printed.contains("ab angle(ab, ac, sense: cw) ad"), "{printed}");
    assert!(printed.contains("length(5 * pi) k"), "{printed}");
    // and the one lifted from the sketch, read back
    let mut lifted = to_program(&e.sketch);
    let text = gcs_core::syntax::render_flat(&mut lifted).unwrap().to_string();
    let (again, d) = read(&text);
    assert!(again.ok(), "{d:?}\n{text}");
    let kinds = |e: &Elaborated| {
        let mut k: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
        k.sort();
        k
    };
    assert_eq!(kinds(&again), kinds(&e), "{text}");
    // the registry publishes the words the sense slot takes
    let reg = gcs_core::report::registry_json();
    let eq = reg.get("types").unwrap().arr().iter()
        .find(|t| t.get("name").unwrap().as_str() == "EqualAngle")
        .expect("equal angles in the registry")
        .clone();
    let words = eq.get("words").unwrap().arr()[4].arr().iter()
        .map(|w| w.as_str().to_string())
        .collect::<Vec<_>>();
    assert_eq!(words, ["ccw", "cw"]);
    assert!(reg.get("types").unwrap().arr().iter()
        .any(|t| t.get("name").unwrap().as_str() == "ArcLength"));
}

/* -- a perpendicular that says which way --------------------------------------------------- */

/// A horizontal line and a second from its start, the second's end seeded `d` (#497).
fn square(seed: (f64, f64), stated: &str) -> String {
    format!("use std\nuse std.place (horizontal)\nin std.front {{\na := point\nb := point hint((40, 0))\n\
        d := point hint(({}, {}))\nab := line(a, b)\nad := line(a, d)\nhorizontal ab\n\
        a distance(40) b\na distance(20) d\nfix((0, 0)) a\n{stated}\n}}\n", seed.0, seed.1)
}

/// `sense:` says which quarter turn: `ccw` puts the second line anticlockwise of the first, as
/// `angle` counts, and `cw` the other way — from a seed on the wrong side, and from one along
/// the first line, where square-either-way has two roots equally near (#497).
#[test]
fn a_sided_perpendicular_turns_the_way_it_says_from_any_seed() {
    for seed in [(25.0, -10.0), (25.0, 10.0), (20.0, 0.0), (-20.0, 0.1)] {
        let e = solved(&square(seed, "ab perpendicular(sense: ccw) ad"));
        assert!(near(at(&e, "d"), (0.0, 20.0)), "{seed:?}: {:?}", at(&e, "d"));
        let e = solved(&square(seed, "ab perpendicular(sense: cw) ad"));
        assert!(near(at(&e, "d"), (0.0, -20.0)), "{seed:?}: {:?}", at(&e, "d"));
    }
    // unsaid, it is square either way, and the seed's side is kept
    let e = solved(&square((25.0, -10.0), "ab perpendicular ad"));
    assert!(near(at(&e, "d"), (0.0, -20.0)), "{:?}", at(&e, "d"));
}

/// A sided perpendicular is still a relation, not a dimension: no number, no callout, one row,
/// and it prints back as written and reads back from the sketch.
#[test]
fn a_sided_perpendicular_is_drawn_as_none_and_prints_as_written() {
    let src = square((25.0, 10.0), "ab perpendicular(sense: cw) ad");
    let (mut e, d) = read(&src);
    assert!(e.ok(), "{d:?}");
    assert!(gcs_core::callout::layout(&e.sketch, 1.0).iter().all(|k| k.text != "90°"));
    assert_eq!(gcs_core::diagnose::diagnose(&mut e.sketch, Default::default()).dof, 0);
    assert!(said(&e).iter().any(|s| s == "ab perpendicular(sense: cw) ad"), "{:?}", said(&e));
    let mut lifted = to_program(&e.sketch);
    let text = gcs_core::syntax::render_flat(&mut lifted).unwrap().to_string();
    assert!(text.contains("perpendicular(sense: cw)"), "{text}");
    let (mut again, d) = read(&text);
    assert!(again.ok(), "{d:?}\n{text}");
    assert!(solve(&mut again.sketch, SolveOpts::default()).success);
    // the lifted text names its points by number: `d` is `p2`
    assert!(near(at(&again, "p2"), (0.0, -20.0)), "{:?}\n{text}", at(&again, "p2"));
    // the same statement written swapped is not a duplicate of its mirror
    let (e, d) = read(&square((25.0, 10.0),
        "ab perpendicular(sense: ccw) ad\nad perpendicular(sense: ccw) ab"));
    assert!(e.ok(), "{d:?}");
    let sided = |e: &Elaborated| e.sketch.user_constraints().iter()
        .filter(|c| c.kind == CKind::Perpendicular).count();
    assert_eq!(sided(&e), 2);
}

/// Two lines in space turn neither way, so a sense there is refused at its key.
#[test]
fn a_sided_perpendicular_in_space_is_refused() {
    let (e, d) = read("a := point hint((0, 0, 0))\nb := point hint((1, 0, 0))\n\
        c := point hint((0, 1, 1))\nab := line(a, b)\nac := line(a, c)\n\
        ab perpendicular(sense: ccw) ac\n");
    assert!(!e.ok());
    assert!(d.iter().any(|x| format!("{x:?}").contains("turn neither way")), "{d:?}");
}
