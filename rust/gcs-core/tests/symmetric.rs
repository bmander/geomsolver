//! **A family that is the same set in any order** (#415): `component Run(a: face, b: face, r:
//! Length) symmetric(a, b) := { p | … }`.  An instance given what one before it was given, in an
//! order the family allows, is that instance; a copy's region solid of it is that solid, and a body
//! statement over it saying what was said is said once (`flatten::same`).  So `round(r) X`, which
//! reaches every edge of `X` in both orders and every corner in six, states each once — and makes
//! the same body as the words written without `symmetric`, which state each every time.
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::model::SolidDef;
use gcs_core::program::Elaborated;

use crate::common::{close, ent, parse, read, text::solved};

/// A block 40 × 24 × 12 drawn on the front plane (x, z), its near face at y = 0, standing 12 deep.
const BLOCK: &str = "unit mm\nuse std (round, chamfer)\nin std.front {\n\
    b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
    fix((0, 0)) b0\nfix((40, 0)) b1\nfix((40, 24)) b2\nfix((0, 24)) b3\n\
    (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> \
    (west := line(b3, b0)) -> close\n\
    outline := face(south, east, north, west)\n}\n\
    block := solid(outline, depth: 12mm)\n";

/// `std.sv`'s rounds and chamfers — the sets and the four words — said again with no `symmetric`
/// and under names of their own (`ordered_round`, `ORun`, …): the words as they were before #415,
/// stating each edge once for each order of its faces.
fn ordered_words() -> String {
    let std = include_str!("../../lib/std.sv");
    let from = std.find("component Run(").expect("std's Run");
    let to = std.find("chamfer(d) X := X chamfer(d: d) X").expect("std's chamfer") + 33;
    let mut words = std[from..to].to_string();
    // each `symmetric(…)` clause, its tuples' brackets included
    while let Some(at) = words.find(" symmetric(") {
        let open = at + " symmetric".len();
        let mut depth = 0;
        let mut end = open;
        for (k, ch) in words[open..].char_indices() {
            match ch {
                '(' => depth += 1,
                ')' => depth -= 1,
                _ => {}
            }
            if depth == 0 {
                end = open + k + 1;
                break;
            }
        }
        words.replace_range(at..end, "");
    }
    for (a, b) in [("Run(", "ORun("), ("Fill(", "OFill("), ("Corner(", "OCorner("), ("Flat(", "OFlat("),
                   ("round(", "ordered_round("), ("chamfer(", "ordered_chamfer(")] {
        words = words.replace(a, b);
    }
    assert!(!words.contains("symmetric"), "{words}");
    words
}

/// `src` with the library's `round` and `chamfer` in it replaced by `ordered_words`', defined
/// where it says `use std`.
fn ordered(src: &str) -> String {
    let uses = src.lines().find(|l| l.starts_with("use std")).expect("a `use std`");
    let src = src.replace("round(r: ", "ordered_round(r: ").replace("chamfer(d: ", "ordered_chamfer(d: ");
    src.replacen(uses, &format!("use std\n{}\n", ordered_words()), 1)
}

/// Solid `name` of `e` built exactly, and its volume.
fn volume(e: &Elaborated, name: &str) -> f64 {
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = export::exact(&e.sketch, ent(e, name).i(), None, None, &say)
        .unwrap_or_else(|r| panic!("{name}: {r:?}"));
    props::volume(&exact.solid)
}

/// **Each edge and corner once** (#415's table): `bores.sv` has nine faces, so `round(r) X` reached
/// 72 ordered pairs and 504 ordered triples, a run and a fill for each — 1152 region solids, where
/// 36 pairs and 84 triples are 240.
#[test]
fn a_body_rounded_all_over_states_each_edge_and_corner_once() {
    let src = include_str!("../../examples/rounds/bores.sv");
    let once = read(src).sketch.solids.len();
    let every = read(&ordered(src)).sketch.solids.len();
    assert_eq!(every - once, 1152 - 240, "{every} solids written ordered, {once} so");
}

/// The block rounded all over and chamfered all over, each by the library's words and by the
/// same words written without `symmetric`: one body, to the closed form's tolerance — so no edge or
/// corner stated once is one dropped.
#[test]
fn stating_each_edge_once_makes_the_same_body() {
    for word in ["round(r: 2mm) block", "chamfer(d: 1mm) block"] {
        let src = format!("{BLOCK}{word}\n");
        let once = solved(&src);
        let every = solved(&ordered(&src));
        assert!(once.sketch.solids.len() < every.sketch.solids.len(), "{word}");
        close(volume(&once, "block"), volume(&every, "block"));
    }
}

/// A word over the block's faces making `std.Flat(a, b, la: d, lb: e)` for every two of them.
fn flats(d: &str, e: &str, other: bool) -> String {
    let both = if other { "other := std.Flat(b, a, la: e, lb: d)\n        made := solid(other)\n        made cut A.body\n" } else { "" };
    let src = BLOCK.replace("use std (round, chamfer)", "use std");
    format!("{src}A flats(d, e) B := {{\n  repeat a in A {{\n    repeat b in B {{\n      when b != a {{\n        \
        one := std.Flat(a, b, la: d, lb: e)\n        taken := solid(one)\n        taken cut A.body\n        \
        {both}      }}\n    }}\n  }}\n}}\nblock flats(d: {d}, e: {e}) block\n")
}

/// **A tuple changes places whole**: `Flat` is `symmetric((a, la), (b, lb))`, each face with the
/// leg along it.  A flat 1 along `a` and 3 along `b` is the flat 3 along `b` and 1 along `a` — and
/// not the flat 1 along `b` and 3 along `a`, which the copy over the faces the other way round makes.
#[test]
fn a_tuple_of_formals_changes_places_together() {
    let regions = |src: &str| read(src).sketch.solids.iter().filter(|s| s.name != "block").count();
    // six faces, thirty ordered pairs: legs alike, each pair's flat once
    assert_eq!(regions(&flats("1mm", "1mm", false)), 15);
    // legs unlike, each order of a pair its own flat
    assert_eq!(regions(&flats("1mm", "3mm", false)), 30);
    // and the same flat written with its faces and its legs both swapped is that flat
    assert_eq!(regions(&flats("1mm", "3mm", true)), 30);
}

/// Two instances of a symmetric family written in the document are one set, but two solids the
/// document names are two statements it made: neither is taken for the other.
#[test]
fn a_documents_own_solids_of_one_set_stay_two() {
    let src = "unit mm\nuse std\n\
        component Lens(a: point, b: point, r: Length) symmetric(a, b) := { p | p distance(<= r) a; p distance(<= r) b }\n\
        in std.front {\n  p := point\n  q := point\n  fix((0, 0)) p\n  fix((6, 0)) q\n}\n\
        x := Lens(p, q, r: 5mm)\ny := Lens(q, p, r: 5mm)\nsx := solid(x)\nsy := solid(y)\n";
    let e = solved(src);
    let named: Vec<&str> = e.sketch.solids.iter().map(|s| s.name.as_str()).collect();
    assert!(named.contains(&"sx") && named.contains(&"sy"), "{named:?}");
}

/// What body `name` of `e` takes away: its `cut` terms, by name.
fn cut_terms(e: &Elaborated, name: &str) -> Vec<String> {
    let SolidDef::Body { through, .. } = &e.sketch.solids[ent(e, name).i()].def else {
        panic!("`{name}` is no body");
    };
    through.iter().map(|&t| e.sketch.solids[t as usize].name.clone()).collect()
}

/// A solid the document names is never taken for a copy's: `sx`, the document's own solid of the
/// edge's set, stands apart while the round makes its own of that edge — it is no term of the
/// block.
#[test]
fn a_documents_solid_is_never_a_copys() {
    let block = BLOCK.replace("use std (round, chamfer)", "use std\nuse std (round)");
    let src = format!("{block}x := std.Run(block.near, block.east, r: 2mm)\nsx := solid(x)\n\
        round(r: 2mm) block\n");
    let cut = cut_terms(&read(&src), "block");
    assert!(!cut.is_empty() && !cut.iter().any(|t| t == "sx"), "{cut:?}");
}

/// A body reached by two words over one edge (the block rounded all over, then its near face's
/// edges again) takes each solid away once: the second word's solids are the first's, and a cut
/// already made is not made again.
#[test]
fn an_edge_rounded_twice_is_cut_once() {
    let once = cut_terms(&read(&format!("{BLOCK}round(r: 2mm) block\n")), "block");
    let twice = read(&format!("{BLOCK}round(r: 2mm) block\nblock.near round(r: 2mm) block\n"));
    let twice = cut_terms(&twice, "block");
    let mut distinct = twice.clone();
    distinct.sort();
    distinct.dedup();
    assert_eq!(distinct.len(), twice.len(), "{twice:?}");
    assert_eq!(once.len(), twice.len(), "{once:?} / {twice:?}");
}

/// The block's regions, past the block, of a word `body` written over every two of its faces
/// `a` and `b` and stated over the block, under `head` (a family or nothing).
fn over_pairs(head: &str, body: &str) -> usize {
    let src = BLOCK.replace("use std (round, chamfer)", "use std");
    let src = format!("{src}{head}A pairs() B := {{\n  repeat a in A {{\n    repeat b in B {{\n      \
        when b != a {{\n{body}      }}\n    }}\n  }}\n}}\nblock pairs() block\n");
    read(&src).sketch.solids.iter().filter(|s| s.name != "block").count()
}

/// Two instances given the same faces are one only where they are stamped alike: a flat over a
/// pair's faces each way round, one of them `class phantom`, is two flats per pair — the plain
/// one each way one, and the phantom one each way one.
#[test]
fn instances_stamped_otherwise_are_their_own() {
    let body = "        one := std.Flat(a, b, la: 1mm, lb: 1mm)\n        taken := solid(one)\n        \
        taken cut A.body\n        other := std.Flat(b, a, la: 1mm, lb: 1mm) class phantom\n        \
        made := solid(other)\n        made cut A.body\n";
    assert_eq!(over_pairs("", body), 30);
}

/// A family may list as many items as it has formals: eight faces that change places, each pair
/// of the block's faces reached both ways one set.
#[test]
fn a_family_of_many_items_is_one_set_in_any_order() {
    let head = "component Eight(a: face, b: face, c: face, d: face, e: face, f: face, g: face, \
        h: face) symmetric(a, b, c, d, e, f, g, h) := { p |\n  p inside a\n  p inside b\n  \
        p inside c\n  p inside d\n  p inside e\n  p inside f\n  p inside g\n  p inside h\n}\n";
    let body = "        one := Eight(a, b, a, b, a, b, a, b)\n        taken := solid(one)\n        \
        taken cut A.body\n";
    assert_eq!(over_pairs(head, body), 15);
}

/// What `symmetric` may say, and what it may not, each refused where it is written.
#[test]
fn symmetric_names_formals_that_can_change_places() {
    let refused = |head: &str, says: &str| {
        let src = format!("unit mm\ncomponent S(a: point, b: point, c: line, r: Length, s: Length) {head}\n");
        let (_, errs) = parse(&src);
        assert!(errs.iter().any(|e| e.message.contains(says)), "`{head}`: {errs:?}");
    };
    refused("symmetric(a) := { p | p distance(r) a }", "two or more formals");
    refused("symmetric(a, z) := { p | p distance(r) a }", "`z` is no formal");
    refused("symmetric(a, a) := { p | p distance(r) a }", "`a` is listed twice");
    refused("symmetric(a, c) := { p | p distance(r) a }", "of one type");
    refused("symmetric((a, r), b) := { p | p distance(r) a }", "as many formals");
    refused("symmetric(a, b) { x := point }", "only a set family");
    let (_, errs) = parse("unit mm\ncomponent S(a: point, b: point, r: Length, s: Length) \
        symmetric((a, r), (b, s)) := { p | p distance(r) a; p distance(s) b }\n");
    assert!(errs.is_empty(), "{errs:?}");
}

