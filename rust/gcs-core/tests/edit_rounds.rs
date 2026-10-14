//! **Round and chamfer, by gesture**: a solid's edges read off its mesh as the pairs of faces
//! that meet there (`mesh::face_edges`), picked where the eye sees them (`workspace::pick_edge`),
//! and written as the library's words over face paths (`edit::add_rounds`).

use gcs_core::edit::{self, EdgeWord};
use gcs_core::overview::workspace::{pick_edge, solid_edges, Projection};
use gcs_core::solid::ApproximationPolicy::Report;
use gcs_core::program;
use crate::common::gestures::{build, seen};

const BLOCK: &str = "\
unit mm
use std

in std.front {
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  fix((0, 0)) b0
  fix((40, 0)) b1
  fix((40, 24)) b2
  fix((0, 24)) b3
  (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) ->
    (west := line(b3, b0)) -> close
  outline := face(south, east, north, west)
}
block := solid(outline, depth: 12mm)
";

fn objects(e: &program::Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap().i()
}

fn pair(a: &str, b: &str) -> (String, String) {
    (a.to_string(), b.to_string())
}

fn volume(e: &program::Elaborated, name: &str) -> f64 {
    e.sketch.evaluated_solid(objects(e, name), Report).unwrap().volume()
}

/// `add_rounds` taken, and the text it wrote elaborated.
fn rounded(e: &program::Elaborated, solid: &str, word: EdgeWord, faces: &[&str],
           edges: &[(String, String)], size: &str) -> (String, program::Elaborated) {
    let faces: Vec<String> = faces.iter().map(|f| f.to_string()).collect();
    let out = edit::add_rounds(&e.program, &e.sketch, objects(e, solid), word, &faces, edges, size);
    assert_eq!(out.refused, None);
    assert_eq!(out.kind, edit::Kind::Structural);
    let next = build(&out.text);
    (out.text, next)
}

fn refused(e: &program::Elaborated, solid: &str, faces: &[&str], edges: &[(String, String)],
           size: &str) -> String {
    let faces: Vec<String> = faces.iter().map(|f| f.to_string()).collect();
    let out = edit::add_rounds(&e.program, &e.sketch, objects(e, solid), EdgeWord::Round, &faces,
                               edges, size);
    assert_eq!(out.kind, edit::Kind::None, "{faces:?} {edges:?} {size}\n{}", out.text);
    assert_eq!(out.text, e.program.text());
    out.refused.expect("refused, saying why")
}

#[test]
fn an_edge_is_where_two_faces_meet_and_a_tangent_seam_is_none() {
    let e = build(BLOCK);
    let edges = solid_edges(&e.sketch, objects(&e, "block"));
    // a box's twelve, each named by its two faces
    assert_eq!(edges.len(), 12);
    let near_north = edges.iter().find(|f| f.faces == pair("block.near", "block.north")).unwrap();
    for [a, b] in &near_north.segs {
        let on = |p: &[f64; 3]| p[1].abs() < 1e-9 && (p[2] - 24.0).abs() < 1e-9;
        assert!(on(a) && on(b), "{a:?} {b:?}");
    }
    // an obround's sides go on tangent into its ends: no edge between them, one where each meets
    // the plate it stands on
    let e = build(include_str!("../../examples/rounds/slot.sv"));
    let edges = solid_edges(&e.sketch, objects(&e, "part"));
    let has = |a: &str, b: &str| edges.iter().any(|f| f.faces == pair(a, b));
    assert!(!has("boss.east", "boss.near_side") && !has("boss.far_side", "boss.west"));
    assert!(has("boss.east", "boss.near") && has("boss.near", "boss.near_side"));
}

#[test]
fn an_edge_is_picked_where_the_eye_sees_it_and_never_through_the_material() {
    let e = build(BLOCK);
    let sk = &e.sketch;
    // from in front, to the right and above: the near, east and north faces are seen
    let (az, el) = (-1.0, 0.5);
    let proj = Projection::new(sk, az, el);
    let at = |x: [f64; 3]| pick_edge(sk, &proj, seen(x, az, el), 0.5);
    let hit = at([20.0, 0.0, 24.2]).expect("the near face's top edge");
    assert_eq!((hit.solid, hit.faces), (objects(&e, "block"), pair("block.near", "block.north")));
    let [x, y, z] = hit.at;
    assert!((x - 20.0).abs() < 0.5 && y.abs() < 1e-9 && (z - 24.0).abs() < 1e-9, "{:?}", hit.at);
    assert_eq!(at([40.0, 6.0, 24.0]).map(|h| h.faces), Some(pair("block.east", "block.north")));
    // the far face's bottom edge is behind the block: where it is seen, nothing is picked
    assert_eq!(at([20.0, 12.0, 0.0]), None);
    // and from behind and below it is seen
    let (az, el) = (2.0, -0.5);
    let proj = Projection::new(sk, az, el);
    let hit = pick_edge(sk, &proj, seen([20.0, 12.0, 0.0], az, el), 0.5).expect("seen from behind");
    assert_eq!(hit.faces, pair("block.far", "block.south"));
    // far from every edge, nothing
    assert_eq!(pick_edge(sk, &proj, seen([20.0, 6.0, 12.0], az, el), 0.5), None);
}

#[test]
fn an_edge_picked_is_rounded_by_the_two_faces_meeting_there() {
    let e = build(BLOCK);
    let whole = 40.0 * 24.0 * 12.0;
    assert!((volume(&e, "block") - whole).abs() < 1e-6);
    let (text, next) = rounded(&e, "block", EdgeWord::Round, &[],
                               &[pair("block.near", "block.north")], "2");
    assert!(text.contains("\nblock.near round(r: 2) block.north"), "{text}");
    // the word is brought in by name, beside what the document already used
    assert!(text.contains("use std (round)\n"), "{text}");
    // a ball of radius 2 rolled 40 along the edge takes the corner it cannot reach
    let taken = 4.0 * (1.0 - std::f64::consts::FRAC_PI_4) * 40.0;
    assert!((volume(&next, "block") - (whole - taken)).abs() < 1e-6, "{}", volume(&next, "block"));
    // a chamfer the same way, by its leg
    let (text, next) = rounded(&e, "block", EdgeWord::Chamfer, &[],
                               &[pair("block.east", "block.north")], "1mm");
    assert!(text.contains("\nblock.east chamfer(d: 1mm) block.north"), "{text}");
    assert!(text.contains("use std (chamfer)\n"), "{text}");
    assert!((volume(&next, "block") - (whole - 0.5 * 12.0)).abs() < 1e-6);
}

#[test]
fn a_chain_of_edges_is_a_statement_a_link_and_a_face_its_every_edge() {
    let e = build(BLOCK);
    let rim = [pair("block.near", "block.north"), pair("block.east", "block.near"),
               pair("block.near", "block.south"), pair("block.near", "block.west")];
    let (text, chained) = rounded(&e, "block", EdgeWord::Round, &[], &rim, "2");
    assert_eq!(text.matches(" round(r: 2) ").count(), 4, "{text}");
    // the near face picked is its rim: one statement, the same solid
    let (text, faced) = rounded(&e, "block", EdgeWord::Round, &["block.near"], &rim, "2");
    assert!(text.trim_end().ends_with("\nblock.near round(r: 2) block"), "{text}");
    assert_eq!(text.matches(" round(r: 2) ").count(), 1, "{text}");
    assert!((volume(&chained, "block") - volume(&faced, "block")).abs() < 1e-6);
    // every face picked is every edge, its corners a ball's
    let all = ["block.near", "block.far", "block.north", "block.south", "block.east", "block.west"];
    let (text, _) = rounded(&e, "block", EdgeWord::Round, &all, &[], "2");
    assert!(text.contains("\nround(r: 2) block"), "{text}");
}

#[test]
fn a_face_of_a_body_is_named_through_the_operand_that_made_it() {
    let src = include_str!("../../examples/rounds/pad.sv");
    // the pad without its rounds
    let src: String =
        src.lines().filter(|l| !l.contains(" round(r: ")).collect::<Vec<_>>().join("\n");
    let e = build(&src);
    let edge = [pair("plate.cd", "plate.near")];
    let (text, _) = rounded(&e, "pad", EdgeWord::Round, &[], &edge, "edge_radius");
    assert!(text.contains("\npad.plate.cd round(r: edge_radius) pad.plate.near"), "{text}");
    // the document already imported the word: nothing more is said of the `use`
    assert_eq!(text.matches("use std").count(), 1, "{text}");
}

#[test]
fn the_import_names_the_word_beside_what_it_named() {
    let e = build(&BLOCK.replace("use std\n", "use std (horizontal)\n"));
    let (text, next) = rounded(&e, "block", EdgeWord::Round, &["block.far"], &[], "2");
    assert!(text.contains("use std (horizontal, round)\n"), "{text}");
    let (text, _) = rounded(&next, "block", EdgeWord::Chamfer, &["block.near"], &[], "1");
    assert!(text.contains("use std (horizontal, round, chamfer)\n"), "{text}");
}

#[test]
fn what_the_language_would_refuse_is_refused_at_the_gesture() {
    let e = build(BLOCK);
    let why = refused(&e, "block", &[], &[pair("block.near", "block.far")], "2");
    assert!(why.contains("do not meet"), "{why}");
    let why = refused(&e, "block", &["plate.near"], &[], "2");
    assert!(why.contains("no face of `block`"), "{why}");
    let why = refused(&e, "block", &["block.near"], &[], " ");
    assert!(why.contains("radius"), "{why}");
    let why = refused(&e, "block", &[], &[], "2");
    assert!(why.contains("nothing to round"), "{why}");
    // a size that is no length, in the language's words
    let why = refused(&e, "block", &["block.near"], &[], "2deg");
    assert!(!why.is_empty());
    // a face a round made has no name the source can write
    let edge = [pair("block.near", "block.north")];
    let (_, next) = rounded(&e, "block", EdgeWord::Round, &[], &edge, "2");
    let edges = solid_edges(&next.sketch, objects(&next, "block"));
    let made = edges.iter().find(|f| f.faces.0.contains('#') || f.faces.1.contains('#'))
        .expect("an edge of the round's face");
    let why = refused(&next, "block", &[], &[made.faces.clone()], "1");
    assert!(why.contains("no name the document can write"), "{why}");
}
