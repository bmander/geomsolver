//! Two words the spiral-bevel layout asked for (`docs/spiral-bevel-layout-plan.md`, language
//! changes 4 and 6): an angle stated as another angle, `l1 angle(l3, l4) l2`, and an arc's length
//! along itself, `length(L) a`.  Each is checked against a closed form the document never
//! states, with its free twin and its printed spelling; their refusals are in `refusals.rs`.

use gcs_core::constraints::CKind;
use gcs_core::io;
use gcs_core::program::{elaborate, to_program, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::parse_legacy as parse;
use std::f64::consts::PI;

fn read(src: &str) -> (Elaborated, Vec<String>) {
    let (prog, errs) = parse(src);
    let e = elaborate(&prog);
    let mut all: Vec<String> = errs.iter().map(|x| format!("syntax: {}", x.message)).collect();
    all.extend(e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)));
    (e, all)
}

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

/// The statements a sketch holds, in the words a document writes them with.
fn said(e: &Elaborated) -> Vec<String> {
    e.sketch.user_constraints().iter().map(|c| io::describe_with(c, &|r| e.map.name_of(r).cloned()))
        .collect()
}

/* -- arc length ---------------------------------------------------------------------------- */

const QUARTER: &str = "\
o := point hint(x: 0, y: 0)
s := point hint(x: 10, y: 0)
e := point hint(x: 3, y: 9)
a := arc(o, s, e)
radius(10) a
o horizontal s
ground o
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
o := point hint(x: 0, y: 0)
s := point hint(x: 14, y: 0)
e := point hint(x: 0, y: 14)
a := arc(o, s, e)
l1 := line(o, s)
l2 := line(o, e)
horizontal l1
l1 angle(90deg) l2
length(5 * pi) a
ground o
";
    let e = solved(src);
    assert!((e.sketch.radius_value(e.map.ent_named("a").unwrap()) - 10.0).abs() < 1e-9);
}

/// Written in terms of a free variable, two arcs' lengths are tied: the first is fixed by its
/// grounded end, and the second — twice the radius — sweeps half as far.
#[test]
fn two_arc_lengths_tied_by_a_free_variable() {
    let src = "\
o1 := point hint(x: 0, y: 0)
s1 := point hint(x: 10, y: 0)
e1 := point hint(x: 0, y: 10)
a1 := arc(o1, s1, e1)
o1 horizontal s1
ground o1
ground e1
o2 := point hint(x: 50, y: 0)
s2 := point hint(x: 70, y: 0)
e2 := point hint(x: 60, y: 15)
a2 := arc(o2, s2, e2)
radius(20) a2
o2 horizontal s2
ground o2
length(s) a1
length(s) a2
";
    let (mut e, d) = read(src);
    assert!(e.ok(), "{d:?}");
    assert!(d.iter().any(|m| m.starts_with("W111")), "{d:?}");
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
a := point hint(x: 0, y: 0)
b := point hint(x: 40, y: 0)
c := point hint(x: 10, y: 30)
d := point hint(x: 25, y: 10)
ab := line(a, b)
ac := line(a, c)
ad := line(a, d)
horizontal ab
a distance(40) b
a distance(30) c
a distance(20) d
ab angle(60deg) ac
ground a
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

/// It replaces the shared free variable the same statement used to need, and says what that
/// did: the two drawings land in one place.
#[test]
fn equal_angles_say_what_a_shared_free_variable_said() {
    let word = solved(&format!("{FAN}ab angle(ad, ac) ad\n"));
    let (mut free, d) = read(&format!("{FAN}ab angle(beta) ad\nad angle(beta) ac\n"));
    assert!(d.iter().any(|m| m.starts_with("W111")), "{d:?}");
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
         o := point hint(x: 100, y: 0)\ns := point hint(x: 110, y: 0)\ne := point hint(x: 100, y: 10)\n\
         k := arc(o, s, e)\nlength(5 * pi) k\n"
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
