//! **A material as a model value** (#510 P3): `steel := material(E: 200GPa, nu: 0.3, rho:
//! 7850kg / 1m^3, yield: 250MPa)` and `arm := solid(sec, depth: 4mm) of steel`.  The gates: a
//! material is held in SI whatever the document's unit, each property is checked for its
//! dimension and its range where it is written, a solid's `of` names a material and nothing
//! else, the members read as numbers as any group's do, and a material survives the printer, a
//! lifted program, the document JSON, a rebuild and a paste.

use gcs_core::io;
use gcs_core::model::{EntKind, EntRef, Material, Property, Sketch};
use gcs_core::program::{to_program, Elaborated};

use crate::common::lenient::read;
use crate::common::parse;

const STEEL: &str = "steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)";

/// A disc of radius `r` and depth `depth`, `of` what `of` says, in a document in `unit`, with
/// `extra` beside it.
fn disc(unit: &str, r: &str, depth: &str, of: &str, extra: &str) -> String {
    format!("unit {unit}\nuse std\n{extra}\nin std.top {{\n  \
             rim := radius({r}) circle(center: std.top.origin)\n  \
             disc := solid(face(rim), depth: {depth}){of}\n}}\n")
}

/// `src` read with no diagnostic, or the test fails saying which.
fn built(src: &str) -> Elaborated {
    let (e, d) = read(src);
    assert!(e.ok() && d.is_empty(), "{d:?}\n{src}");
    e
}

/// The diagnostics `src` is read with, each `CODE: message` or `syntax: message`.
fn said(src: &str) -> Vec<String> {
    read(src).1
}

fn solid(sk: &Sketch, name: &str) -> usize {
    sk.solids.iter().position(|s| s.name == name).unwrap_or_else(|| panic!("no solid `{name}`"))
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-12 * b.abs().max(1.0)
}

#[test]
fn a_material_is_a_model_value_in_si() {
    let e = built(&disc("mm", "20mm", "5mm", " of steel", STEEL));
    let m = &e.sketch.materials["steel"];
    assert_eq!(m.name, "steel");
    // pascals and kilograms a cubic metre, not the document's kilograms a millimetre-second²
    assert!(close(m.get(Property::E).unwrap(), 200e9), "{m:?}");
    assert!(close(m.get(Property::Nu).unwrap(), 0.3), "{m:?}");
    assert!(close(m.get(Property::Rho).unwrap(), 7850.0), "{m:?}");
    assert!(close(m.get(Property::Yield).unwrap(), 250e6), "{m:?}");
    assert_eq!(m.text(), "steel: E 200GPa, nu 0.3, rho 7850kg/m^3, yield 250MPa");
    // the solid names it, and nothing else does
    let sk = &e.sketch;
    assert_eq!(sk.solids[solid(sk, "disc")].material.as_deref(), Some("steel"));
    assert_eq!(sk.solids.iter().filter(|s| s.material.is_some()).count(), 1);
    assert_eq!(gcs_core::report::material_text(sk, "steel"),
               "steel: E 200GPa, nu 0.3, rho 7850kg/m^3, yield 250MPa, of disc");
}

/// **What a material is does not change with the unit a drawing is dimensioned in**: the same
/// text in an inch document states the same pascals.
#[test]
fn a_material_is_the_same_in_any_unit() {
    let mm = built(&disc("mm", "20mm", "5mm", " of steel", STEEL));
    let inch = built(&disc("in", "1in", "0.25in", " of steel", STEEL));
    let (a, b) = (&mm.sketch.materials["steel"], &inch.sketch.materials["steel"]);
    for p in Property::ALL {
        assert!(close(a.get(p).unwrap(), b.get(p).unwrap()), "{p:?}: {a:?} against {b:?}");
    }
    // and an aluminium written in grams a cubic centimetre is 2700kg/m^3
    let al = "al := material(E: 69GPa, rho: 2.7g / 1cm^3)";
    let al = built(&disc("in", "1in", "0.25in", " of al", al));
    let al = &al.sketch.materials["al"];
    assert!(close(al.get(Property::Rho).unwrap(), 2700.0), "{al:?}");
    assert_eq!(al.get(Property::Nu), None, "a material states what it states");
}

/// A material's members are a group's: `steel.E` is a number with its dimension, read where any
/// number is.
#[test]
fn its_properties_read_as_numbers() {
    let src = disc("mm", "steel.E / 10GPa * 1mm", "steel.rho / (1000kg / 1m^3) * 1mm", " of steel",
                   STEEL);
    let mut e = built(&src);
    let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let sk = &e.sketch;
    assert!(close(sk.params[sk.circles[0].radius as usize].value, 20.0));
    // and the dimension travels: a pressure is no length
    let d = said(&disc("mm", "steel.E", "5mm", "", STEEL));
    assert!(d.iter().any(|s| s.contains("Length") && s.contains("(Pa)")), "{d:?}");
}

/// **Each property is checked where it is written**: a property a material has, of its
/// dimension — a bare number refused where it has one, since the document's unit of pressure is
/// nothing anyone writes — in its range, never an unknown of the drawing, never given twice; and
/// a material states at least one.
#[test]
fn a_property_is_checked_where_it_is_written() {
    let cases: [(&str, &str); 8] = [
        ("m := material(E: 200000)", "E103: `E` is Young's modulus, Length^-1·Mass·Time^-2 (Pa): a \
          bare number says no unit, so write one (`200GPa`)"),
        ("m := material(rho: 7850kg)", "E103: `rho` is Length^-3·Mass (kg/m^3), and this is \
          Mass (kg)"),
        ("m := material(nu: 0.5)", "E103: `nu` is Poisson's ratio, which lies between -1 and 0.5, \
          and this is 0.5"),
        ("m := material(E: -1GPa)", "E103: `E` is Young's modulus, which is positive, and this is \
          -1GPa"),
        ("m := material(strength: 3MPa)", "E103: `strength` is no property of a material: `E`, \
          `nu`, `rho`, `yield`"),
        ("m := material()", "E103: `m` states no property: a material says what it is made of, \
          `E`, `nu`, `rho`, `yield`"),
        ("param k: Scalar hint(2)\nm := material(E: k * 1GPa)", "E103: `E` is a number the \
          drawing is made of, and this reads an unknown of the drawing"),
        ("m := material(E: 1GPa, E: 2GPa)", "E001: `E` is given twice"),
    ];
    for (decl, want) in cases {
        let d = said(&disc("mm", "20mm", "5mm", " of m", decl));
        assert!(d.iter().any(|s| s == want), "{decl}: {d:?}");
        // a refused material is said once, where it is written: the solid made of it says nothing
        assert!(!d.iter().any(|s| s.contains("no material")), "{decl}: {d:?}");
    }
    // the properties are named, and only a solid is made of one
    let d = said("m := material(200GPa)\n");
    assert!(d.iter().any(|s| s.contains("a material's properties are named")), "{d:?}");
    let d = said(&format!("{STEEL}\nuse std\nin std.top {{\n  c := circle(center: std.top.origin) \
                           of steel\n}}\n"));
    assert!(d.iter().any(|s| s.contains("`of` says what a solid is made of")), "{d:?}");
}

/// **A solid's `of` names a material**: another name of the scope is refused, and one that names
/// nothing is said once, as no such name.
#[test]
fn of_names_a_material() {
    let d = said(&disc("mm", "20mm", "5mm", " of rim", STEEL));
    assert!(d.iter().any(|s| s.starts_with("E103: `rim` is no material")), "{d:?}");
    let d = said(&disc("mm", "20mm", "5mm", " of dims", "dims := {bore: 16mm}"));
    assert!(d.iter().any(|s| s.starts_with("E103: `dims` is no material")), "{d:?}");
    let d = said(&disc("mm", "20mm", "5mm", " of nothing", STEEL));
    assert_eq!(d.iter().filter(|s| s.contains("nothing")).count(), 1, "{d:?}");
    assert!(d.iter().any(|s| s.starts_with("E101")), "{d:?}");
    // `of` closes a class list as `in` does, where a class list is still read
    let (e, d) = crate::common::lenient::read_legacy(&disc("mm", "20mm", "5mm",
                                                           " class heavy of steel", STEEL));
    assert!(e.ok() && d.is_empty(), "{d:?}");
    let sk = &e.sketch;
    let i = solid(sk, "disc");
    assert_eq!(sk.solids[i].material.as_deref(), Some("steel"));
    assert!(sk.solids[i].class.has("heavy"));
}

/// **The material is the body's**: a solid a cut makes a body keeps it under its name, and the
/// sweep moved to its stock carries none — what is made of steel is what the document named.
/// A material declared in a component is its instance's (`o.al`), and one handed to a component
/// as a group is the caller's.
#[test]
fn a_body_and_an_instance_name_their_material() {
    let e = built(&format!("unit mm\nuse std\n{STEEL}\nin std.top {{\n  \
        rim := radius(20mm) circle(center: std.top.origin)\n  \
        hole := radius(5mm) circle(center: std.top.origin)\n  \
        disc := solid(face(rim), depth: 5mm) of steel\n  \
        bore := solid(face(hole), through: disc)\n}}\nbore cut disc\n"));
    let sk = &e.sketch;
    let made: Vec<&str> = sk.solids.iter().filter(|s| s.material.is_some())
        .map(|s| s.name.as_str()).collect();
    assert_eq!(made, ["disc"]);
    assert!(matches!(sk.solids[solid(sk, "disc")].def, gcs_core::model::SolidDef::Body { .. }));
    let e = built(&format!("unit mm\nuse std\n\
        component Plate(m: group, c: point, r: Length) {{\n  \
          rim := radius(r) circle(center: c)\n  \
          body := solid(face(rim), depth: 5mm) of m\n}}\n\
        component Own(c: point) {{\n  \
          al := material(E: 69GPa)\n  \
          rim := radius(10mm) circle(center: c)\n  \
          body := solid(face(rim), depth: 5mm) of al\n}}\n\
        {STEEL}\np := Plate(steel, std.top.origin, r: 20mm) in std.top\n\
        o := Own(std.front.origin) in std.front\n"));
    let sk = &e.sketch;
    assert_eq!(sk.materials.keys().collect::<Vec<_>>(), ["o.al", "steel"]);
    assert_eq!(sk.solids[solid(sk, "p.body")].material.as_deref(), Some("steel"));
    assert_eq!(sk.solids[solid(sk, "o.body")].material.as_deref(), Some("o.al"));
}

/// **The printer writes what it read**: `material(…)` and `of` come back as written, and the
/// printed program reads to the same materials.
#[test]
fn a_material_prints_as_written() {
    let src = disc("mm", "20mm", "5mm", " of steel", STEEL);
    let (mut p, errs) = parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let printed = gcs_core::syntax::render_flat(&mut p).unwrap().to_string();
    assert!(printed.contains(STEEL), "{printed}");
    assert!(printed.contains("depth: 5mm) of steel"), "{printed}");
    let again = built(&printed);
    let before = built(&src);
    assert_eq!(again.sketch.materials, before.sketch.materials);
    let sk = &again.sketch;
    assert_eq!(sk.solids[solid(sk, "disc")].material.as_deref(), Some("steel"));
}

/// **The document JSON carries the materials**, in SI under each property's own name, and reads
/// them back the same — refusing, as the language does, a property no material has or a value
/// none takes.  A program lifted from the sketch states them again.
#[test]
fn a_material_round_trips_through_json() {
    let e = built(&disc("in", "1in", "0.25in", " of steel", STEEL));
    let json = io::to_json(&e.sketch);
    let ms = json.get("materials").expect("written").arr();
    assert_eq!(ms.len(), 1);
    assert_eq!(ms[0].get("name").map(|v| v.as_str()), Some("steel"));
    assert!(close(ms[0].get("E").unwrap().as_f64(), 200e9));
    let back = io::loads(&io::dumps(&e.sketch, None)).expect("reads back");
    assert_eq!(back.materials, e.sketch.materials);
    // a document with none writes none
    let plain = built(&disc("mm", "20mm", "5mm", "", ""));
    assert!(io::to_json(&plain.sketch).get("materials").is_none());
    // what the reader refuses
    let read = |j: &str| io::material_from_json(&gcs_core::json::parse(j).unwrap());
    assert_eq!(read(r#"{"name": "x", "nu": 0.7}"#).unwrap_err(),
               "`nu` is Poisson's ratio, which lies between -1 and 0.5, and this is 0.7");
    assert!(read(r#"{"name": "x", "G": 1}"#).unwrap_err().contains("no property"));
    assert!(read(r#"{"name": "x", "E": "stiff"}"#).is_err());
    assert!(read(r#"{"E": 1}"#).is_err());
    assert_eq!(read(r#"{"name": "x", "E": 1e9}"#).unwrap(),
               Material::new("x", [(Property::E, 1e9)].into()).unwrap());
    // lifted to a program and read again: the same materials, in an inch document as in mm
    let mut lifted = to_program(&back);
    let text = gcs_core::syntax::render_flat(&mut lifted).unwrap().to_string();
    assert!(text.contains("steel := material(E: 200GPa, nu: 0.3, rho: "), "{text}");
    let again = built(&text);
    for p in Property::ALL {
        let (a, b) = (again.sketch.materials["steel"].get(p), back.materials["steel"].get(p));
        assert!(close(a.unwrap(), b.unwrap()), "{p:?}: {a:?} against {b:?}\n{text}");
    }
}

/// **A rebuild and a paste keep them**: a deletion keeps the document's materials and every
/// solid's name of one; a paste brings the clipboard's where the document lacks them.
#[test]
fn a_material_survives_a_rebuild_and_a_paste() {
    let e = built(&disc("mm", "20mm", "5mm", " of steel", STEEL));
    let sk = &e.sketch;
    let point = (0..sk.points.len()).find(|&i| !sk.point_fixed(i)).map(EntRef::point);
    let rebuilt = gcs_core::graft::without(sk, &point.into_iter().collect::<Vec<_>>(), &[]);
    assert_eq!(rebuilt.materials, sk.materials);
    let all: Vec<EntRef> = sk.primitives().into_iter()
        .chain((0..sk.solids.len()).map(|i| EntRef::new(EntKind::Solid, i))).collect();
    let clip = gcs_core::graft::copy(sk, &all);
    assert_eq!(clip.materials, sk.materials);
    let mut other = Sketch::new();
    gcs_core::graft::paste(&mut other, &clip, 0.0, 0.0);
    assert_eq!(other.materials, sk.materials);
    let i = solid(&clip, "disc");
    assert_eq!(clip.solids[i].material.as_deref(), Some("steel"));
}

/// **The report says what each is and what is made of it** (`report::materials_json`, what
/// `gcs_materials_json` hands a binding): the document JSON's record, its words and its solids.
#[test]
fn the_report_lists_materials_and_their_solids() {
    let both = format!("{STEEL}\nal := material(E: 69GPa)");
    let e = built(&disc("mm", "20mm", "5mm", " of steel", &both));
    let sk = &e.sketch;
    let j = gcs_core::report::materials_json(sk);
    let ms = j.arr();
    assert_eq!(ms.len(), 2);
    let by = |n: &str| ms.iter().find(|m| m.get("name").map(|v| v.as_str()) == Some(n)).unwrap();
    let solids = |n: &str| by(n).get("solids").unwrap().arr().iter().map(|v| v.as_i64())
        .collect::<Vec<_>>();
    assert_eq!(solids("steel"), [solid(sk, "disc") as i64]);
    assert!(solids("al").is_empty());
    assert_eq!(by("al").get("text").map(|v| v.as_str()), Some("al: E 69GPa, of no solid"));
}
