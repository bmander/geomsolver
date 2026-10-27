//! **A colour for each part** (`colours`, #676/#677): every object of one binding one colour, as
//! the placements of one part are in a CAD assembly; the colour the palette's, drawn from what
//! the part is and not from its numbers, so it survives a reload and a numeric edit, and parts
//! told apart while there are colours to go round; or the drawing's, where a `.svd` rule outside
//! its sheets states a `fill` for a path naming the part.

use crate::common::read_solved;
use gcs_core::colours::{self, PALETTE};
use gcs_core::drawing;
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;
use std::collections::BTreeMap;

/// Two plates of one binding, a third of other numbers, and a peg.
fn model(w: &str) -> String {
    format!("
unit mm
use std
use std.place (offset, right_of)
component Plate(w: Length) {{
  in std.front {{
    a := point hint((0, 0))
    b := point hint((20, 0))
    c := point hint((20, 10))
    e := point hint((0, 10))
    a offset(dx: 0mm, dy: 0mm) std.origin
    b offset(dx: w, dy: 0mm) std.origin
    c offset(dx: w, dy: 10mm) std.origin
    e offset(dx: 0mm, dy: 10mm) std.origin
    ab := line(a, b)
    bc := line(b, c)
    ce := line(c, e)
    ea := line(e, a)
    body := solid(face(ab, bc, ce, ea), from: 0mm, to: 3mm)
  }}
}}
component Peg(r: Length) {{
  in std.front {{
    c := point hint((5, 0))
    c right_of(d: 5mm) std.origin
    rim := circle(center: c) hint(r: r)
    radius(r) rim
    body := solid(face(rim), from: 0mm, to: 8mm)
  }}
}}
in (f := frame) {{ one := Plate(w: {w}) }}
fix(origin == (0, 0, 0)) f
in (g := frame) {{ two := Plate(w: {w}) }}
fix(origin == (50, 0, 0)) g
in (h := frame) {{ three := Plate(w: 35mm) }}
fix(origin == (0, 50, 0)) h
in (k := frame) {{ peg := Peg(r: 2mm) }}
fix(origin == (0, 0, 40)) k
")
}

fn object(e: &Elaborated, sk: &Sketch, name: &str) -> usize {
    let i = sk.solids.iter().position(|s| s.name == name).unwrap_or_else(|| panic!("no solid `{name}`"));
    assert!(gcs_core::overview::objects(sk).contains(&i), "`{name}` is an object");
    let _ = e;
    i
}

/// The colour of each object by its name.
fn by_name(sk: &Sketch, colours: &BTreeMap<usize, [u8; 3]>) -> BTreeMap<String, [u8; 3]> {
    colours.iter().map(|(&i, &c)| (sk.solids[i].name.clone(), c)).collect()
}

#[test]
fn every_placement_of_a_part_is_one_colour_and_parts_differ() {
    let (e, sk) = read_solved(&model("20mm"));
    let p = colours::palette(&sk);
    assert_eq!(p.len(), 4, "a colour for every object: {:?}", by_name(&sk, &p));
    let [one, two, three, peg] = ["one.body", "two.body", "three.body", "peg.body"].map(|n| object(&e, &sk, n));
    assert_eq!(p[&one], p[&two], "two placements of one part, one colour");
    assert_ne!(p[&one], p[&three], "a plate of other numbers is another part");
    assert_ne!(p[&one], p[&peg]);
    assert_ne!(p[&three], p[&peg]);
    assert!(p.values().all(|c| PALETTE.contains(c)), "every colour the palette's");
}

#[test]
fn a_part_keeps_its_colour_across_a_reload_and_a_numeric_edit() {
    let (_, a) = read_solved(&model("20mm"));
    let (_, again) = read_solved(&model("20mm"));
    let (_, edited) = read_solved(&model("24mm"));
    let [a, again, edited] = [&a, &again, &edited].map(|sk| by_name(sk, &colours::palette(sk)));
    assert_eq!(a, again, "read again, the same colours");
    assert_eq!(a, edited, "the plates' width edited, the same colours");
}

/// A drawing beside the model, its bundle and the loader a host gives `drawing::colours`.
fn coloured(sk: &Sketch, e: &Elaborated, svd: &str) -> Result<BTreeMap<String, [u8; 3]>, String> {
    let files: BTreeMap<String, String> = [
        ("parts/model.sv".to_string(), String::new()),
        ("parts/model.svd".to_string(), svd.to_string()),
        ("parts/inks.svd".to_string(), "style m.peg { fill: #123456 }\n".to_string()),
    ].into();
    let model = drawing::Model { sketch: sk, names: &e.map };
    drawing::colours(svd, "parts/model.svd", "parts/model.sv", model, &mut |path, from| {
        let name = drawing::relative_path(path, from);
        files.get(&name).map(|t| (name, t.clone()))
    }).map(|c| by_name(sk, &c)).map_err(|e| e.message)
}

#[test]
fn a_drawing_names_a_parts_colour_for_every_placement_of_it() {
    let (e, sk) = read_solved(&model("20mm"));
    // a rule on one placement's solid colours the part, both placements; a later rule over an
    // earlier; an imported rule before the drawing's own; a scope's rule its objects
    let c = coloured(&sk, &e, "
model m from \"model.sv\"
use \"inks.svd\"
style m.one.body { fill: #e8a33d }
style m.three { fill: #abc }
style m.three.body { fill: #b5a642 }
").unwrap();
    assert_eq!(c.get("one.body"), Some(&[0xe8, 0xa3, 0x3d]));
    assert_eq!(c.get("two.body"), Some(&[0xe8, 0xa3, 0x3d]), "the other placement of the part named");
    assert_eq!(c.get("three.body"), Some(&[0xb5, 0xa6, 0x42]), "the later rule");
    assert_eq!(c.get("peg.body"), Some(&[0x12, 0x34, 0x56]), "the imported rule");
    // what the drawing leaves keeps the palette's, said as the palette's
    let parts = colours::parts(&sk, &BTreeMap::new());
    assert!(parts.iter().all(|p| !p.drawn));
    // a drawing of another model colours nothing of this one
    assert!(coloured(&sk, &e, "model m from \"other.sv\"\nstyle m.peg { fill: #fff }\n").unwrap().is_empty());
}

#[test]
fn a_fill_names_a_part_and_stands_outside_every_sheet() {
    let (e, sk) = read_solved(&model("20mm"));
    let refused = |svd: &str| coloured(&sk, &e, svd).expect_err("refused");
    assert!(refused("model m from \"model.sv\"\nstyle m.one.a { fill: #fff }\n").contains("names no part"));
    assert!(drawing::parse("style .visible { fill: #fff }").unwrap_err().message.contains("model path"));
    assert!(drawing::parse("model m from \"x.sv\"\nsheet s {\n  style m.one { fill: #fff }\n}\n")
        .unwrap_err().message.contains("outside every sheet"));
    assert!(drawing::parse("style m.one { fill: #ffffff80 }").unwrap_err().message.contains("#RGB or #RRGGBB"));
    assert!(drawing::parse("style m.one { fill: red }").is_err());
    // and stated beside a sheet's ink, a rule draws both
    let rule = &drawing::parse("style m.one { color: #000; fill: #fff }").unwrap().styles[0];
    assert_eq!(rule.fill, Some([255, 255, 255]));
    assert_eq!(rule.style.color.as_deref(), Some("#000"));
}

#[test]
fn a_colour_is_written_as_a_drawing_writes_it() {
    assert_eq!(colours::parse("#e8a33d"), Some([0xe8, 0xa3, 0x3d]));
    assert_eq!(colours::parse("#fa3"), Some([0xff, 0xaa, 0x33]));
    assert_eq!(colours::hex([0xe8, 0xa3, 0x3d]), "#e8a33d");
    assert_eq!(colours::parse("e8a33d"), None);
    assert_eq!(colours::unit([255, 0, 51]), [1.0, 0.0, 0.2]);
}
