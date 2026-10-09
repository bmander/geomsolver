//! **A gesture writes a solid** (#162 F0): faces, sweeps and the body rule appended as statements,
//! each held to the elaborator before it is handed back, so a gesture the language refuses is
//! refused at the gesture, in the language's words.

use gcs_core::edit::{self, Kind, SolidSweep};
use gcs_core::model::EntRef;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solid::ApproximationPolicy::Report;
use gcs_core::syntax::{BodyWord, Program, Sense};
use crate::common::parse;

/// A 60 × 40 rectangle of named lines on the front, a circle of radius 5 at (12, 20) and a loose
/// 10 × 10 square at (35, 15) inside it.
const PLATE: &str = "\
unit mm
use std
in std.front {
a := point hint((0, 0))
b := point hint((60, 0))
c := point hint((60, 40))
d := point hint((0, 40))
ab := line(a, b)
bc := line(b, c)
cd := line(c, d)
da := line(d, a)
o := point hint((12, 20))
hole := circle(center: o) hint(r: 5)
p := point hint((35, 15))
q := point hint((45, 15))
r := point hint((45, 25))
s := point hint((35, 25))
pq := line(p, q)
qr := line(q, r)
rs := line(r, s)
sp := line(s, p)
}
";

fn prog_of(src: &str) -> Program {
    let (p, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}\n{src}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    p
}

fn read(src: &str) -> Elaborated {
    let e = elaborate(&prog_of(src));
    assert!(e.ok(), "{:?}\n{src}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    e
}

fn taken(e: edit::Edit) -> String {
    assert_eq!(e.refused, None);
    assert_eq!(e.kind, Kind::Structural);
    e.text
}

fn volume(e: &Elaborated, name: &str) -> f64 {
    e.sketch.evaluated_solid(e.map.ent_named(name).unwrap().i(), Report).unwrap().volume()
}

fn edges(names: &[&str]) -> Vec<String> {
    names.iter().map(|n| n.to_string()).collect()
}

fn depth(d: &str) -> SolidSweep {
    SolidSweep { depth: Some(d.into()), ..Default::default() }
}

const OUTER: [&str; 4] = ["ab", "bc", "cd", "da"];

#[test]
fn a_face_and_its_depth_are_two_statements() {
    let out = edit::add_face(&prog_of(PLATE), &edges(&OUTER), &[], None);
    assert_eq!(out.names, vec!["f0"]);
    let text = taken(out);
    assert!(text.ends_with("}\nf0 := face(ab, bc, cd, da)\n"), "{text}");
    for (written, d) in [("8", 8.0), ("8mm", 8.0), ("1cm", 10.0)] {
        let out = edit::add_solid(&prog_of(&text), "f0", &depth(written), None);
        assert_eq!(out.names, vec!["b0"]);
        let solid = taken(out);
        assert!(solid.ends_with(&format!("f0 := face(ab, bc, cd, da)\nb0 := solid(f0, depth: {written})\n")),
            "{solid}");
        assert!((volume(&read(&solid), "b0") - 2400.0 * d).abs() < 1e-7, "{written}");
    }
}

#[test]
fn a_hole_is_its_circle_or_a_face_written_first() {
    let holes = vec![edges(&["hole"]), edges(&["pq", "qr", "rs", "sp"])];
    let out = edit::add_face(&prog_of(PLATE), &edges(&OUTER), &holes, Some("plate"));
    assert_eq!(out.names, vec!["plate", "f0"]);
    let text = taken(out);
    assert!(text.ends_with("f0 := face(pq, qr, rs, sp)\nplate := face(ab, bc, cd, da, holes: hole, f0)\n"),
        "{text}");
    let solid = taken(edit::add_solid(&prog_of(&text), "plate", &depth("2"), None));
    let expected = 2.0 * (2400.0 - 100.0 - 25.0 * std::f64::consts::PI);
    // the circle is faceted to the report's unit, so within its sagitta
    assert!((volume(&read(&solid), "b0") / expected - 1.0).abs() < 1e-3, "{}", volume(&read(&solid), "b0"));
}

#[test]
fn every_sweep_a_gesture_makes_is_written_as_the_language_spells_it() {
    let with_face = format!("{PLATE}sec := face(ab, bc, cd, da)\nbody := solid(sec, depth: 4)\n");
    let cases = [
        (SolidSweep { from: Some("-2".into()), to: Some("3".into()), ..Default::default() },
            "b0 := solid(sec, from: -2, to: 3)"),
        (SolidSweep { through: Some("body".into()), ..Default::default() },
            "b0 := solid(sec, through: body)"),
        (SolidSweep { about: Some("da".into()), ..Default::default() }, "b0 := solid(sec, about: da)"),
        (SolidSweep { about: Some("da".into()), sweep: Some("90deg".into()), sense: Some(Sense::Cw),
            ..Default::default() }, "b0 := solid(sec, about: da, sweep: 90deg, sense: cw)"),
    ];
    for (how, line) in cases {
        let text = taken(edit::add_solid(&prog_of(&with_face), "sec", &how, None));
        assert!(text.ends_with(&format!("{line}\n")), "{text}");
        read(&text);
    }
}

#[test]
fn a_fresh_name_passes_every_name_the_document_binds() {
    // `b0` is a value, not a declaration: a name minted over it would be defined twice
    let src = format!("{PLATE}b0 := 5\nsec := face(ab, bc, cd, da)\n");
    let out = edit::add_solid(&prog_of(&src), "sec", &depth("b0"), None);
    assert_eq!(out.names, vec!["b1"]);
    assert!(taken(out).ends_with("b1 := solid(sec, depth: b0)\n"));
    for (name, why) in [("sec", "already a name"), ("in", "not a name")] {
        let out = edit::add_face(&prog_of(&src), &edges(&OUTER), &[], Some(name));
        assert!(out.refused.as_deref().is_some_and(|m| m.contains(why)), "{name}: {:?}", out.refused);
        assert_eq!(out.text, src);
    }
}

#[test]
fn a_mixture_of_sweeps_is_refused_in_the_parsers_words() {
    let src = format!("{PLATE}sec := face(ab, bc, cd, da)\n");
    let cases = [
        (SolidSweep { depth: Some("4".into()), about: Some("da".into()), ..Default::default() },
            "not both"),
        (SolidSweep { sense: Some(Sense::Cw), depth: Some("4".into()), ..Default::default() },
            "turn a face about an axis"),
        (SolidSweep { from: Some("4".into()), ..Default::default() }, "`from:` one ordinate"),
        (SolidSweep::default(), "says how"),
    ];
    for (how, why) in cases {
        let out = edit::add_solid(&prog_of(&src), "sec", &how, None);
        assert!(out.refused.as_deref().is_some_and(|m| m.contains(why)), "{why}: {:?}", out.refused);
        assert_eq!(out.text, src);
        assert_eq!(out.kind, Kind::None);
    }
}

#[test]
fn what_the_elaborator_refuses_is_refused_at_the_gesture() {
    let src = format!("{PLATE}sec := face(ab, bc, cd, da)\nbody := solid(sec, depth: 4)\n");
    let prog = prog_of(&src);
    let refusals = [
        edit::add_solid(&prog, "nothing", &depth("4"), None),
        edit::add_face(&prog, &edges(&["ab", "cd"]), &[], None),
        edit::add_body_word(&prog, BodyWord::Cut, "body", "body"),
    ];
    for out in refusals {
        assert!(out.refused.is_some(), "{}", out.text);
        assert_eq!(out.text, src);
    }
}

#[test]
fn an_error_already_in_the_document_is_not_the_gestures() {
    let src = format!("{PLATE}a distance(5) nowhere\n");
    assert!(elaborate(&prog_of(&src)).errors().next().is_some());
    let text = taken(edit::add_face(&prog_of(&src), &edges(&OUTER), &[], None));
    assert!(text.ends_with("f0 := face(ab, bc, cd, da)\n"), "{text}");
}

#[test]
fn a_body_word_cuts_and_a_removed_solid_takes_its_word_with_it() {
    let src = format!("{PLATE}sec := face(ab, bc, cd, da)\nstock := solid(sec, depth: 4)\n\
        body := solid(stock)\nbore := solid(face(hole), through: body)\n");
    let text = taken(edit::add_body_word(&prog_of(&src), BodyWord::Cut, "bore", "body"));
    assert!(text.ends_with("bore cut body\n"), "{text}");
    let e = read(&text);
    let expected = 4.0 * (2400.0 - 25.0 * std::f64::consts::PI);
    assert!((volume(&e, "body") / expected - 1.0).abs() < 1e-3, "{}", volume(&e, "body"));
    let bore = e.map.ent_named("bore").unwrap();
    let out = edit::remove(&e, &e.program, &e.sketch, &[EntRef::solid(bore.i())], &[]);
    assert_eq!(out.refused, None);
    assert!(!out.text.contains("bore"), "{}", out.text);
    assert!((volume(&read(&out.text), "body") - 9600.0).abs() < 1e-7);
}
