//! **A drawing's objects as one STEP assembly** (`brep::export::assembly`, `step::write_assembly`):
//! a part per object, the objects of one binding one part built once and used where each
//! instance stands, a mirrored one its own part, a sub-assembly per step of their names; the file
//! parsed back against that structure (`step_check::verify_assembly`).  Held by the property: each
//! object's part, carried down the tree by its uses' placements, is the object's own exact build
//! where it stands; and each part's colour, written as AP214's presentation on its solid, is read
//! back as the colour asked for it (#677).

use crate::common::read_solved as solved;
use gcs_core::brep::geom::Frame;
use gcs_core::brep::query::{Located, Place};
use gcs_core::brep::topo::Brep;
use gcs_core::brep::{export, props, step, step_check, sweep::Say};
use gcs_core::model::Sketch;

/// Two plates of one binding in turned frames, the first mirrored; a plate of another; and a
/// ring of three pegs, cylinders turned about a point.
const DRAWING: &str = "
unit mm
use std
use std.place (offset, right_of)
component Plate(w: Length, d: Length, h: Length) {
  in std.front {
    a := point hint((1, 1))
    b := point hint((w, 1))
    c := point hint((w, d))
    e := point hint((1, d))
    a offset(dx: 1mm, dy: 1mm) std.origin
    b offset(dx: w, dy: 1mm) std.origin
    c offset(dx: w, dy: d) std.origin
    e offset(dx: 1mm, dy: d) std.origin
    ab := line(a, b)
    bc := line(b, c)
    ce := line(c, e)
    ea := line(e, a)
    body := solid(face(ab, bc, ce, ea), from: 0mm, to: h)
  }
}
component Peg(r: Length) {
  in std.front {
    c := point hint((20, 0))
    c right_of(d: 20mm) std.origin
    rim := circle(center: c) hint(r: r)
    radius(r) rim
    body := solid(face(rim), from: 0mm, to: 5mm)
  }
}
in (f := frame) { one := Plate(w: 30mm, d: 20mm, h: 5mm) mirrored(front) }
fix(dir == (0, 1, 0)) f.u
fix(dir == (0, 0, 1)) f.v
fix(origin == (10, 20, 30)) f
in (g := frame) { two := Plate(w: 30mm, d: 20mm, h: 5mm) }
fix(dir == (0, 0, 1)) g.u
fix(dir == (1, 0, 0)) g.v
fix(origin == (-40, 5, 12)) g
in (h := frame) { three := Plate(w: 30mm, d: 20mm, h: 5mm) }
fix(dir == (0, 1, 0)) h.u
fix(dir == (-1, 0, 0)) h.v
fix(origin == (0, 0, -50)) h
in (k := frame) { four := Plate(w: 40mm, d: 20mm, h: 5mm) }
fix(origin == (60, 0, 0)) k
in std.front {
  o := point hint((50, 0))
  fix((50, 0)) o
  ring 3 about o {
    p := Peg(r: 3mm)
  }
}
";

fn quiet() -> Say<'static> {
    fn nothing(_: &str) {}
    fn unmarked(_: gcs_core::solid::export::Stage) {}
    Say { stage: &nothing, mark: &unmarked }
}

/// The drawing's objects as an assembly, each part built once and coloured `colour` of its first
/// object, and the objects.
fn built(sk: &Sketch, colour: &dyn Fn(usize) -> Option<[f64; 3]>) -> (step::Assembly, Vec<usize>) {
    let objects = gcs_core::overview::objects(sk);
    let mut parts = Vec::new();
    let a = export::assembly(sk, &objects, "drawing", None, colour, &quiet(), &mut |i, _| {
        parts.push(i)
    })
    .unwrap_or_else(|(i, e)| panic!("{}: {e}", sk.solids[i].name));
    assert_eq!(parts.len(), a.products.iter().filter(|p| p.solid.is_some()).count());
    (a, objects)
}

/// The part a use of `name` places, and that part where it stands in the world: carried by its
/// placement and each placement up the tree to the root.
fn placed(a: &step::Assembly, name: &str) -> (usize, Brep) {
    let u = a.uses.iter().find(|u| u.name == name).unwrap_or_else(|| panic!("no use `{name}`"));
    let part = a.products[u.child].solid.as_ref().expect("a use of an object places a part");
    let mut solid = part.moved(&u.placement.placing());
    let mut parent = u.parent;
    while parent != 0 {
        let up = a.uses.iter().find(|w| w.child == parent).expect("a sub-assembly is used");
        solid = solid.moved(&up.placement.placing());
        parent = up.parent;
    }
    (u.child, solid)
}

/// Whether `a` and `b` are one solid, wherever their seams fall: the same volume and bounds, and
/// every vertex of each on the other's boundary.
fn same(a: &Brep, b: &Brep, what: &str) {
    let (va, vb) = (props::volume(a), props::volume(b));
    assert!((va - vb).abs() <= 1e-9 * vb.abs(), "{what}: {va} against {vb}");
    let (x, y) = (a.bounds(), b.bounds());
    for k in 0..3 {
        assert!((x.0[k] - y.0[k]).abs() <= 1e-9 && (x.1[k] - y.1[k]).abs() <= 1e-9,
                "{what}: {x:?} against {y:?}");
    }
    for (one, other) in [(a, b), (b, a)] {
        let located = Located::new(other, 1e-9);
        for v in &one.vertices {
            assert_eq!(located.solid_place(v.p), Place::On, "{what}: {:?} is off the other's", v.p);
        }
    }
}

#[test]
fn every_object_is_its_part_carried_where_it_stands() {
    let (_, sk) = solved(DRAWING);
    let (a, objects) = built(&sk, &|_| None);
    let say = quiet();
    for &i in &objects {
        let name = gcs_core::vocab::public_path(&sk.solids[i].name);
        let own = export::exact(&sk, i, None, None, &say).unwrap().solid;
        same(&placed(&a, &name).1, &own, &name);
    }
}

#[test]
fn the_objects_of_one_binding_are_one_part_and_a_mirrored_one_its_own() {
    let (_, sk) = solved(DRAWING);
    let (a, _) = built(&sk, &|_| None);
    let part = |name: &str| placed(&a, name).0;
    // the representative is the mirrored plate, whose build is reflected: the first unmirrored
    // object of the binding is the part the others use
    assert_ne!(part("one.body"), part("two.body"));
    assert_eq!(part("two.body"), part("three.body"));
    assert_ne!(part("two.body"), part("four.body"));
    assert_eq!(a.products[part("three.body")].name, "two.body");
    // a ring's copies, named as a reader writes them, are one part
    assert_eq!(part("p[0].body"), part("p[1].body"));
    assert_eq!(part("p[0].body"), part("p[2].body"));
    assert_eq!(a.products.iter().filter(|p| p.solid.is_some()).count(), 4);
    // a sub-assembly per instance, each holding its object
    let groups: Vec<&str> =
        a.products.iter().filter(|p| p.solid.is_none()).map(|p| p.name.as_str()).collect();
    assert_eq!(groups, ["drawing", "one", "two", "three", "four", "p[0]", "p[1]", "p[2]"]);
    // the part of an instance is written in its instance's frame: the plate's corner drawn at
    // (1, 1) off its frame's origin in its front view, the frame's (x, z) there
    let two = a.products[part("two.body")].solid.as_ref().unwrap();
    assert!(two.vertices.iter().any(|v| (0..3).all(|k| (v.p[k] - [1., 0., 1.][k]).abs() < 1e-9)),
            "{:?}", two.vertices.iter().map(|v| v.p).collect::<Vec<_>>());
}

#[test]
fn the_file_is_parsed_back_against_the_assembly() {
    let (_, sk) = solved(DRAWING);
    let (a, _) = built(&sk, &|_| None);
    let text = export::assembly_step(&a, "drawing", None, &quiet()).unwrap();
    let structure = step_check::Structure::of(&a);
    let verified = step_check::verify_assembly(&text, &structure).unwrap();
    let faces: usize =
        a.products.iter().filter_map(|p| p.solid.as_ref()).map(|s| s.faces.len()).sum();
    assert_eq!(verified.faces, faces);
    assert_eq!(text.matches("NEXT_ASSEMBLY_USAGE_OCCURRENCE(").count(), a.uses.len());
    assert_eq!(text.matches("MANIFOLD_SOLID_BREP(").count(), 4, "a part written once, used often");
    // a use placed elsewhere is not the assembly's
    let item = text.find("ITEM_DEFINED_TRANSFORMATION('','',#").unwrap();
    let to = text[item..].split(['#', ')']).nth(2).unwrap();
    let axes = text.find(&format!("\n#{to} = AXIS2_PLACEMENT_3D('',#")).unwrap();
    let point = text[axes..].split(['#', ',']).nth(3).unwrap();
    let line = text.find(&format!("\n#{point} = CARTESIAN_POINT('',(")).unwrap() + 1;
    let end = line + text[line..].find('\n').unwrap();
    let moved =
        format!("{}#{point} = CARTESIAN_POINT('',(0.5,0.5,0.5));{}", &text[..line], &text[end..]);
    let e = step_check::verify_assembly(&moved, &structure).expect_err("a use moved");
    assert!(e.contains("is not a use the assembly has"), "{e}");
    // and a product renamed is not
    let renamed = text.replacen("PRODUCT('p[0].body','p[0].body'", "PRODUCT('peg','peg'", 1);
    let e = step_check::verify_assembly(&renamed, &structure).expect_err("a product renamed");
    assert!(e.contains("the file's is named"), "{e}");
}

#[test]
fn an_assembly_that_is_not_a_tree_laid_out_top_down_is_refused() {
    let (_, sk) = solved(DRAWING);
    let (a, _) = built(&sk, &|_| None);
    let part = a.products.iter().find_map(|p| p.solid.clone()).unwrap();
    let products = || vec![
        step::Product { name: "root".into(), solid: None, colour: None },
        step::Product { name: "part".into(), solid: Some(part.clone()), colour: None },
        step::Product { name: "more".into(), solid: Some(part.clone()), colour: None },
    ];
    let usage = |parent, child| {
        step::Usage { parent, child, name: format!("{parent}/{child}"), placement: Frame::WORLD }
    };
    let refused =
        |uses: Vec<step::Usage>| step::Assembly { products: products(), uses }.check().unwrap_err();
    assert!(refused(vec![usage(0, 1), usage(1, 2)]).contains("is used as an assembly"));
    assert!(refused(vec![usage(0, 1)]).contains("`more` is in no assembly"));
    assert!(refused(vec![usage(0, 1), usage(2, 2)]).contains("does not come before it"));
    let fine =
        step::Assembly { products: products(), uses: vec![usage(0, 1), usage(0, 2), usage(0, 1)] };
    assert!(fine.check().is_ok());
    assert!(step::write_assembly(&fine, "fine", 1e-4).is_ok());
}

/// Three colours dealt to the objects by their solids' indices, so the four parts are not all
/// one colour and two of them share one.
fn dealt(i: usize) -> Option<[f64; 3]> {
    Some([[0.8, 0.25, 0.125], [0.1, 0.6, 0.3], [0.2, 0.4, 1.0]][i % 3])
}

/// Held by the property: each part's solid is styled once, in the colour of the first object it is
/// built for, which `step_check::colours` reads back from the file; every use of a part, wherever
/// placed, is that one part and so that colour; and a colour two parts share is written once.
#[test]
fn each_part_is_written_in_its_colour_and_read_back_so() {
    let (_, sk) = solved(DRAWING);
    let (a, objects) = built(&sk, &dealt);
    let text = export::assembly_step(&a, "drawing", None, &quiet()).unwrap();
    step_check::verify_assembly(&text, &step_check::Structure::of(&a)).unwrap();
    // each part is the colour of the first object it is built for, the one it is named for
    let read = step_check::colours(&text).unwrap();
    let parts: Vec<&step::Product> = a.products.iter().filter(|p| p.solid.is_some()).collect();
    assert_eq!(read.len(), parts.len());
    for p in &parts {
        let first = objects.iter().copied()
            .find(|&i| gcs_core::vocab::public_path(&sk.solids[i].name) == p.name)
            .expect("a part is named for an object");
        assert_eq!(p.colour, dealt(first), "{}", p.name);
        let colours: Vec<Option<[f64; 3]>> =
            read.iter().filter(|(n, _)| *n == p.name).map(|(_, c)| Some(*c)).collect();
        assert_eq!(colours, [p.colour], "{}", p.name);
    }
    // every object placed is its part, in its part's colour
    for &i in &objects {
        let name = gcs_core::vocab::public_path(&sk.solids[i].name);
        let part = &a.products[placed(&a, &name).0];
        assert!(read.iter().any(|(n, c)| *n == part.name && Some(*c) == part.colour), "{name}");
    }
    // a solid styled once per part, and a colour's styles written once
    let mut distinct: Vec<[u64; 3]> =
        parts.iter().map(|p| p.colour.unwrap().map(f64::to_bits)).collect();
    distinct.sort();
    distinct.dedup();
    assert!(distinct.len() < parts.len(), "two parts share a colour");
    assert_eq!(text.matches("STYLED_ITEM('color',").count(), parts.len());
    assert_eq!(text.matches("COLOUR_RGB(").count(), distinct.len());
    assert_eq!(text.matches("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION(").count(), 1);
}

/// Held by the property: the file is the assembly's only where every part's solid is styled in
/// the part's own colour and every style is presented — one component of one colour edited, or
/// a styled item left off the presentation, is refused; and only a part takes a colour, three
/// numbers in 0..=1.
#[test]
fn a_colour_not_the_parts_is_refused() {
    let (_, sk) = solved(DRAWING);
    let (a, _) = built(&sk, &dealt);
    let text = export::assembly_step(&a, "drawing", None, &quiet()).unwrap();
    let structure = step_check::Structure::of(&a);
    // one component of one colour edited
    let at = text.find("COLOUR_RGB('',").unwrap() + "COLOUR_RGB('',".len();
    let end = at + text[at..].find(',').unwrap();
    assert_ne!(&text[at..end], "0.5");
    let edited = format!("{}0.5{}", &text[..at], &text[end..]);
    let e = step_check::verify_assembly(&edited, &structure).expect_err("a colour edited");
    assert!(e.contains("in the colour"), "{e}");
    // a part's styled item left off the presentation
    let styled = text.find("STYLED_ITEM('color',").unwrap();
    let id = text[..styled].rsplit('#').next().unwrap().split(' ').next().unwrap();
    let unlisted = text.replace(&format!("#{id},"), "").replace(&format!(",#{id})"), ")");
    let e = step_check::verify_assembly(&unlisted, &structure).expect_err("a style unlisted");
    assert!(e.contains("listed by no presentation representation"), "{e}");
    // an assembly, or a colour out of 0..=1, is no part's colour
    let mut wrong = built(&sk, &dealt).0;
    wrong.products[0].colour = Some([0.5, 0.5, 0.5]);
    assert!(wrong.check().unwrap_err().contains("is coloured, and only a part is"));
    let mut wrong = built(&sk, &dealt).0;
    let part = wrong.products.iter().position(|p| p.solid.is_some()).unwrap();
    wrong.products[part].colour = Some([0.5, 1.5, 0.5]);
    assert!(wrong.check().unwrap_err().contains("is not three numbers in 0..=1"));
}

/// Held by the property: an assembly with no colour writes no presentation at all (no styled
/// item, nothing read back as a colour), and that file is refused as the same assembly coloured.
#[test]
fn an_uncoloured_assembly_styles_nothing() {
    let (_, sk) = solved(DRAWING);
    let (a, _) = built(&sk, &|_| None);
    let text = export::assembly_step(&a, "drawing", None, &quiet()).unwrap();
    assert!(!text.contains("STYLED_ITEM("));
    assert!(!text.contains("PRESENTATION_REPRESENTATION("));
    assert_eq!(step_check::colours(&text).unwrap(), []);
    let (coloured, _) = built(&sk, &dealt);
    let e = step_check::verify_assembly(&text, &step_check::Structure::of(&coloured))
        .expect_err("the colours left out");
    assert!(e.contains("is styled by nothing, the part is coloured"), "{e}");
}
