//! **Union, Cut, Intersect** (#162 F4): a target body and one or more tool bodies picked in the
//! workspace, written `tool union|cut|bound target` as one splice — and shown before it is written
//! by `Sketch::preview_body`, which evaluates what the statements would make of the target and
//! leaves the drawing as it was.

use gcs_core::edit::{self, Kind};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solid::ApproximationPolicy::Report;
use gcs_core::syntax::{BodyWord, Program};
use crate::common::parse;
use std::f64::consts::PI;

/// A 60 × 40 plate 4 deep behind the front, a 10 × 10 boss standing through it from 8 behind to
/// 4 in front, a bore of radius 5 through it the same way, and a plug inside the bore's reach.
const PARTS: &str = "\
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
fix((0, 0)) a
fix((60, 0)) b
fix((60, 40)) c
fix((0, 40)) d
fix((12, 20)) o
fix(r == 5) hole
fix((35, 15)) p
fix((45, 15)) q
fix((45, 25)) r
fix((35, 25)) s
}
plate := solid(face(ab, bc, cd, da), depth: 4)
boss := solid(face(pq, qr, rs, sp), from: -8, to: 4)
bore := solid(face(hole), from: -8, to: 4)
";

/// The plate's volume, the boss's share of it, and the bore's.
const PLATE: f64 = 60.0 * 40.0 * 4.0;
const BOSS_IN: f64 = 10.0 * 10.0 * 4.0;
const BOSS: f64 = 10.0 * 10.0 * 12.0;
const BORE_IN: f64 = 25.0 * PI * 4.0;

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

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

fn volume(e: &Elaborated, name: &str) -> f64 {
    e.sketch.evaluated_solid(index(e, name), Report).unwrap().volume()
}

fn near(got: f64, want: f64, what: &str) {
    // a bore's wall is faceted: its volume is good to the faceting, and no better
    assert!((got / want - 1.0).abs() < 2e-3, "{what}: {got} against {want}");
}

/// The preview of `tools` joined to `target` by `word`, by volume.
fn previewed(e: &mut Elaborated, target: &str, word: BodyWord, tools: &[&str]) -> f64 {
    let tools: Vec<usize> = tools.iter().map(|t| index(e, t)).collect();
    let target = index(e, target);
    e.sketch.preview_body(target, word, &tools).unwrap().volume()
}

/// The text the gesture writes.
fn written(src: &str, word: BodyWord, tools: &[&str], target: &str) -> String {
    let out = edit::add_body_word(&prog_of(src), word, tools, target);
    assert_eq!(out.refused, None);
    assert_eq!(out.kind, Kind::Structural);
    out.text
}

#[test]
fn the_preview_is_what_the_statements_make() {
    let cases = [
        (BodyWord::Union, vec!["boss"], PLATE + BOSS - BOSS_IN),
        (BodyWord::Cut, vec!["boss"], PLATE - BOSS_IN),
        (BodyWord::Bound, vec!["boss"], BOSS_IN),
        (BodyWord::Cut, vec!["boss", "bore"], PLATE - BOSS_IN - BORE_IN),
    ];
    for (word, tools, want) in cases {
        let what = format!("{tools:?} {}", word.as_str());
        let mut e = read(PARTS);
        near(previewed(&mut e, "plate", word, &tools), want, &format!("{what}, previewed"));
        let text = written(PARTS, word, &tools, "plate");
        let said: String = tools.iter().map(|t| format!("{t} {} plate\n", word.as_str())).collect();
        assert!(text.ends_with(&said), "{what}: one splice, a statement a tool\n{text}");
        near(volume(&read(&text), "plate"), want, &format!("{what}, written"));
    }
}

#[test]
fn a_preview_leaves_the_drawing_as_it_was() {
    let mut e = read(PARTS);
    let (n, objects) = (e.sketch.solids.len(), gcs_core::overview::objects(&e.sketch));
    let before = volume(&e, "plate");
    previewed(&mut e, "plate", BodyWord::Cut, &["boss", "bore"]);
    assert_eq!(e.sketch.solids.len(), n, "the scratch solid is taken out again");
    assert_eq!(gcs_core::overview::objects(&e.sketch), objects);
    assert_eq!(volume(&e, "plate"), before, "the target is untouched");
    // and a refusal leaves it so too
    let plate = index(&e, "plate");
    assert!(e.sketch.preview_body(plate, BodyWord::Cut, &[plate]).is_err());
    assert_eq!(e.sketch.solids.len(), n);
}

#[test]
fn a_body_that_already_cuts_takes_the_tool_into_its_own_sets() {
    // union comes first: a plug joined inside the bore is cut away with it, as the statement says
    let src = format!("{PARTS}bore cut plate\nplug := solid(face(hole), from: -3, to: -1)\n");
    let mut e = read(&src);
    let want = PLATE - BORE_IN;
    near(previewed(&mut e, "plate", BodyWord::Union, &["plug"]), want, "previewed");
    let text = written(&src, BodyWord::Union, &["plug"], "plate");
    near(volume(&read(&text), "plate"), want, "written");
}

#[test]
fn a_tool_through_the_body_reads_its_material_and_closes_no_cycle() {
    // `through: plate` reads the plate's sweep, which the statement moves to a stock: no cycle
    let src = format!("{PARTS}drill := solid(face(hole), through: plate)\n");
    let mut e = read(&src);
    let want = PLATE - BORE_IN;
    near(previewed(&mut e, "plate", BodyWord::Cut, &["drill"]), want, "previewed");
    let text = written(&src, BodyWord::Cut, &["drill"], "plate");
    near(volume(&read(&text), "plate"), want, "written");
}

#[test]
fn a_solid_made_of_itself_is_refused_at_the_gesture() {
    let src = format!("{PARTS}wrap := solid(plate)\n");
    let mut e = read(&src);
    let (plate, wrap) = (index(&e, "plate"), index(&e, "wrap"));
    for tools in [vec![plate], vec![wrap], vec![]] {
        let why = e.sketch.preview_body(plate, BodyWord::Cut, &tools).err();
        assert!(why.is_some(), "{tools:?} previewed");
    }
    let prog = prog_of(&src);
    for tools in [&["plate"][..], &["wrap"], &["boss", "wrap"], &[]] {
        let out = edit::add_body_word(&prog, BodyWord::Cut, tools, "plate");
        assert!(out.refused.is_some(), "{tools:?} written:\n{}", out.text);
        assert_eq!(out.text, src);
        assert_eq!(out.kind, Kind::None);
    }
}
