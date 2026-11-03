//! **Faces that meet** (#625): `when a meets b` holds where two faces of a solid share a point of
//! its boundary, read where the drawing is seeded, so `A round(r) B` pairs and threes only the
//! faces that meet — not every two and three of the faces a body lists, which for a web opened by
//! four turned spokes was past the expansion's limit.  A pair read apart that the solve brings
//! together is read again where the drawing solves (#761); an expansion past its limit is one
//! E103 at the document's statement, and nothing built.
use std::collections::BTreeMap;

use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::program::{self, Elaborated};
use gcs_core::units::Units;

use crate::common::lenient::read;
use crate::common::{ent, text::solved};

/// A block 40 × 24 × 12 on the front plane: six faces, each meeting four.
const BLOCK: &str = "unit mm\nuse std\nuse std.solids\nin std.front {\n\
    b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
    fix((0, 0)) b0\nfix((40, 0)) b1\nfix((40, 24)) b2\nfix((0, 24)) b3\n\
    (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> \
    (west := line(b3, b0)) -> close\n\
    outline := face(south, east, north, west)\n}\n\
    block := solid(std.solids.Prism(outline, from: -12mm, to: 0))\n";

/// How many points the elaboration named ending `.{last}`.
fn made(e: &Elaborated, last: &str) -> usize {
    (0..e.sketch.points.len())
        .filter_map(|i| e.map.name_of(gcs_core::model::EntRef::point(i)))
        .filter(|n| n.ends_with(&format!(".{last}")))
        .count()
}

/// Over the block's faces `meets` keeps the twelve edges each way round (6 × 4, the opposite
/// faces left out) and the eight corners in their six orders.
#[test]
fn faces_meet_where_they_share_an_edge_or_a_corner() {
    let count = |body: &str, last: &str| {
        let (e, d) = read(&format!("{BLOCK}{body}"));
        assert!(d.is_empty(), "{body}: {d:?}");
        made(&e, last)
    };
    let pairs = "repeat a in block {\nrepeat b in block {\nwhen b != a and b meets a {\n\
                 m := point\n}\n}\n}\n";
    assert_eq!(count(pairs, "m"), 24);
    let threes = "repeat a in block {\nrepeat b in block {\nrepeat c in block {\n\
                  when b != a and c != a and c != b and b meets a and c meets a and c meets b {\n\
                  n := point\n}\n}\n}\n}\n";
    assert_eq!(count(threes, "n"), 48);
    let opposite = "repeat a in block.near {\nrepeat b in block.far {\nwhen a meets b {\n\
                    o := point\n}\n}\n}\n";
    assert_eq!(count(opposite, "o"), 0);
}

/// `meets` asks of faces: a name that is none is E103, one that stands for nothing E101, and
/// neither makes its block.
#[test]
fn meets_is_between_two_faces() {
    let refused = |cond: &str, code: &str, words: &str| {
        let src = format!("unit mm\nuse std\nuse std.solids\nin std.front {{\n\
                           a := point\nfix((0, 0)) a\nb := point hint((3, 0))\n\
                           when {cond} {{\nq := point hint((8, 3))\n}}\n}}\n");
        let (e, d) = read(&src);
        assert!(
            d.iter().any(|m| m.starts_with(code) && m.contains(words)),
            "{cond}: expected {code} saying {words:?}: {d:?}"
        );
        assert!(e.map.ent_named("q").is_none(), "a refused block makes nothing");
    };
    refused("a meets b", "E103", "`a` is no face");
    refused("a meets nothing", "E101", "`nothing` is not a number here");
    // and the condition reads it as two names, which `decide` refuses
    let mm = Units::with_length("mm").unwrap();
    let c = gcs_core::expr::parse_condition_in("b meets a", mm).unwrap();
    assert_eq!(c, gcs_core::expr::Cond::Meets("b".into(), "a".into()));
    assert!(gcs_core::expr::parse_condition_in("b meets 3", mm).is_err(), "a name, not a number");
    assert!(gcs_core::expr::decide(&c, &BTreeMap::new()).is_err());
}

/// The disc of #625: its web opened between r 8 and r 15 but for spokes, every edge among the
/// opening's faces rounded.  `SPOKES` stands for how the spokes are drawn.
const DISC: &str = "unit mm\nuse std\nuse std.solids\nuse std.place (vertical, coords)\nuse std.rounds (round)\nuse std.motion (turns)\n\
    in std.front {\n  spindle := vertical line(std.origin, hint((0, 10)))\n  length(10mm) spindle\n}\n\
    in std.top {\n\
      edge := circle(center: std.top.origin) hint(r: 20)\n  radius(20mm) edge\n\
      rim := circle(center: std.top.origin) hint(r: 15)\n  radius(15mm) rim\n\
      hub := circle(center: std.top.origin) hint(r: 8)\n  radius(8mm) hub\n\
      e := point hint((-1.2, V0))\n  f := point hint((1.2, V0))\n\
      g := point hint((1.2, 20))\n  h := point hint((-1.2, 20))\n\
      e coords(du: -1.2mm, dv: V0mm) std.top\n  f coords(du: 1.2mm, dv: V0mm) std.top\n\
      g coords(du: 1.2mm, dv: 20mm) std.top\n  h coords(du: -1.2mm, dv: 20mm) std.top\n\
      ACROSS\n}\n\
    index := frame\nindex turns(1) spindle\n\
    disc := solid(std.solids.Prism(face(edge), from: 0mm, to: 5mm))\n\
    opening := solid(std.solids.Prism(face(rim, holes: hub), from: -1mm, to: 6mm))\n\
    spoke := solid(std.solids.Prism(face(e, f, g, h, -> close), from: -2mm, to: 7mm))\n\
    SPOKES\n\
    opening cut disc\nround(r: 1.2mm) disc.opening\n";

/// Solid `name` of `e` built exactly, and its volume.
fn volume(e: &Elaborated, name: &str) -> f64 {
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = export::exact(&e.sketch, ent(e, name).i(), None, None, &say)
        .unwrap_or_else(|r| panic!("{name}: {r:?}"));
    props::volume(&exact.solid)
}

/// **Four turned spokes** (#625): built, and as the same opening drawn with two crossed bars —
/// the faces to round alike — where before the round's corners over every three of its 28 faces
/// were past the expansion's limit.
#[test]
fn a_round_over_four_turned_spokes_builds() {
    let turned = DISC.replace("V0", "-1").replace("ACROSS", "")
        .replace("SPOKES", "repeat 4 as i {\n  turned_spoke := spoke at(under: index) (i * 90deg)\n  \
                            turned_spoke cut opening\n}");
    let across = "p := point hint((-20, -1.2))\n  q := point hint((20, -1.2))\n  \
        s := point hint((20, 1.2))\n  t := point hint((-20, 1.2))\n  \
        p coords(du: -20mm, dv: -1.2mm) std.top\n  q coords(du: 20mm, dv: -1.2mm) std.top\n  \
        s coords(du: 20mm, dv: 1.2mm) std.top\n  t coords(du: -20mm, dv: 1.2mm) std.top";
    let crossed = DISC.replace("V0", "-20").replace("ACROSS", across)
        .replace("SPOKES", "bar := solid(std.solids.Prism(face(p, q, s, t, -> close), from: -2mm, to: 7mm))\n\
                            spoke cut opening\nbar cut opening");
    let (a, b) = (solved(&turned), solved(&crossed));
    let (va, vb) = (volume(&a, "disc"), volume(&b, "disc"));
    assert!((va - vb).abs() <= 1e-6 * vb, "{va} turned, {vb} crossed");
    // the corners left are none: no three of the opening's faces share a point of the disc
    assert!(a.sketch.solids.len() < 120, "{} solids", a.sketch.solids.len());
}

/// A plate 20 × 20 on the top plane, five thick, and `PIN` standing on it, the two one solid
/// `whole` rounded all over: `PIN` is drawn in `std.top`, its prism called `pin` and joined to
/// `whole`.
fn pinned(pin: &str) -> String {
    format!("unit mm\nuse std\nuse std.solids\nuse std.place (coords, seats)\nuse std.rounds (round)\n\
        in std.top {{\n\
        a := point hint((0, 0))\nb := point hint((20, 0))\nc := point hint((20, 20))\n\
        d := point hint((0, 20))\n\
        a coords(du: 0mm, dv: 0mm) std.top\nb coords(du: 20mm, dv: 0mm) std.top\n\
        c coords(du: 20mm, dv: 20mm) std.top\nd coords(du: 0mm, dv: 20mm) std.top\n}}\n\
        plate := solid(std.solids.Prism(face(a, b, c, d, -> close), from: 0mm, to: 5mm))\n\
        whole := solid(plate)\n{pin}\nround(r: 1mm) whole\n")
}

/// `pinned`'s pin of radius 3, its centre held at `(u, v)` and seeded at `at`, its radius
/// `radius(3mm)` and seeded `seed` (nothing where empty).
fn pin_at((u, v): (u32, u32), at: &str, seed: &str) -> String {
    pinned(&format!("in std.top {{\no := point hint(({at}))\no coords(du: {u}mm, dv: {v}mm) std.top\n\
        boss := circle(center: o){seed}\nradius(3mm) boss\n}}\n\
        pin := solid(std.solids.Prism(face(boss), from: 0mm, to: 10mm))\npin union whole"))
}

/// `src` solved, its solids said nothing of.
fn quiet(src: &str) -> Elaborated {
    let e = solved(src);
    let said = program::solid_diagnostics(&e.sketch, &e.map, &e.program);
    assert!(said.is_empty(), "{said:?}\n{src}");
    e
}

/// `src` solved quietly, and `whole`'s volume.
fn built(src: &str) -> f64 {
    volume(&quiet(src), "whole")
}

/// **Faces read apart where the drawing is seeded are read again where it solves** (#625,
/// #761): a pin seeded off the plate it stands on, held onto it by its centre, meets none of the
/// plate's faces where it is seeded, so `meets` reads its root apart from the plate's top; where
/// the drawing solves they meet, and there the faces are read again and the root is rounded — the
/// same solid as the pin seeded where it stands.  So is a pin whose radius only `radius(3mm)`
/// says, seeded at none (#761's circle): at the unwritten radius's start it stands clear of the
/// plate's side it reaches across.
#[test]
fn faces_read_apart_that_the_solve_brings_together_are_read_there() {
    let near = built(&pin_at((10, 10), "10, 10", " hint(r: 3)"));
    // the root's round fills: the pin and its round add to the plate's rounded volume
    let plate = built(&pinned(""));
    assert!(near > plate + std::f64::consts::PI * 9.0 * 5.0, "{near} with the pin, {plate} without");
    let far = built(&pin_at((10, 10), "60, 60", " hint(r: 3)"));
    assert!((far - near).abs() <= 1e-9 * near, "{far} seeded far, {near} seeded near");
    // and one across the plate's side, its radius left to its dimension: the same rounds made
    // (the solid they make is past the kernel's export, a round's end meeting another's on the
    // side, seeded or not)
    let across = |seed| quiet(&pin_at((20, 10), "20, 10", seed)).sketch.solids.len();
    assert_eq!(across(""), across(" hint(r: 3)"));
}

/// **A part placed by a mate rounds where it is placed** (#761): a post drawn about its own
/// origin and seated on the plate by `seats` stands at the world's origin where it is seeded,
/// over the plate's corner — so the corner's two sides read apart there — and where the drawing
/// solves it stands at the plate's middle.  The same solid as the post drawn there.
#[test]
fn a_part_seated_by_a_mate_rounds_where_it_is_seated() {
    let seated = built(&pinned("component Post() {\nin std.top {\no := point\nfix((0, 0)) o\n\
        k := circle(center: o) hint(r: 3)\nradius(3mm) k\n}\n\
        body := solid(std.solids.Prism(face(k), from: 0mm, to: 10mm))\n}\n\
        in std.top {\nst := point hint((10, 10))\nst coords(du: 10mm, dv: 10mm) std.top\n}\n\
        post := Post()\npost seats(turn: 90deg, up: std.z, east: std.x) st\npost.body union whole"));
    let drawn = built(&pin_at((10, 10), "10, 10", " hint(r: 3)"));
    assert!((seated - drawn).abs() <= 1e-9 * drawn, "{seated} seated, {drawn} drawn in place");
}

/// **An expansion past its limit is one E103** (#625), at the document's statement that expands
/// to it, and nothing is built of what was made: no E101 for a name the rest would have made.
#[test]
fn an_expansion_past_its_limit_is_said_once() {
    let src = "unit mm\nuse std\nuse std.solids\nrepeat 1000 as i {\n  repeat 300 as j {\n    \
               p := point hint((i, j))\n  }\n}\nq := point\n";
    let (e, d) = read(src);
    assert_eq!(d.len(), 1, "{d:?}");
    assert!(d[0].starts_with("E103: more than 200000 statements once expanded"), "{d:?}");
    assert_eq!(e.diags[0].span.lo as usize, src.find("repeat 1000").unwrap());
    assert!(e.sketch.points.is_empty(), "{} points built", e.sketch.points.len());
}
