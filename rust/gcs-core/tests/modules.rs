//! Modules (§14.4): `use NAME` at the top of a document, resolved by the host and linked by
//! `modules::link` — a module's components join the document's, its top-level `param`s are read
//! by its components and by the files that `use` it, and its own drawing is its own.

use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::modules::{link, relink};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::parse;
use std::collections::BTreeMap;

const RUNG: &str = "\
// a module: a param its component reads, and a drawing of its own that is not the document's
param len = 50
component Rung(a: point, b: point) {
  line e(a, b)
  horizontal e
  a distance(len) b
}
point stray hint(x: 999, y: 999)
";

const LADDER: &str = "\
use lib.rung
point l0 hint(x: 0, y: 0)
point r0 hint(x: 50, y: 0)
point l1 hint(x: 0, y: 20)
point r1 hint(x: 50, y: 20)
t0: Rung(l0, r0)
t1: Rung(l1, r1)
line stile(l0, l1)
vertical stile
l0 distance(len) l1
ground l0
";

fn shelf() -> BTreeMap<&'static str, &'static str> {
    let mut m = BTreeMap::new();
    m.insert("lib.rung", RUNG);
    m.insert("lib.bad", "component Bad(a: point) {\n  line l(a,\n}\n");
    m.insert("lib.rung2", "use lib.rung\ncomponent Rung(a: point) { }\n");
    m.insert("lib.diamond", "use lib.rung\ncomponent Step(a: point, b: point) { r: Rung(a, b) }\n");
    m.insert("lib.loop_a", "use lib.loop_b\nparam pa = 1\ncomponent A(p: point) { }\n");
    m.insert("lib.loop_b", "use lib.loop_a\nparam pb = 2\ncomponent B(p: point) { }\n");
    m
}

fn read(src: &str) -> (Elaborated, Vec<gcs_core::program::Diag>) {
    let (mut prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let shelf = shelf();
    let linked = link(&mut prog, &mut |name| shelf.get(name).map(|t| t.to_string()));
    (elaborate(&prog), linked)
}

#[test]
fn a_module_contributes_its_components_and_its_params() {
    let (e, linked) = read(LADDER);
    assert!(linked.is_empty(), "{linked:?}");
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    // two rungs and a stile: the module's own stray point is not drawn
    assert_eq!(e.sketch.points.len(), 4);
    assert_eq!(e.sketch.lines.len(), 3);
    let mut sk = e.sketch.clone();
    gcs_core::solve::solve(&mut sk, Default::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    // `l0 distance(len) l1` read the module's `len` from the document: 50 up as well as across
    assert_eq!((d.dof, d.status), (0, State::Well));
    let (x, y) = sk.point_xy(2);
    assert!((x - 0.0).abs() < 1e-6 && (y - 50.0).abs() < 1e-6, "{x} {y}");
}

#[test]
fn a_module_nothing_resolves_is_said_at_the_use() {
    let (e, linked) = read("use lib.nothing\npoint p\n");
    assert_eq!(linked.len(), 1);
    assert_eq!(linked[0].code.as_str(), "E070");
    assert!(linked[0].message.contains("lib.nothing"));
    assert_eq!(linked[0].span.lo, 0, "at the `use`");
    assert!(e.ok(), "the rest of the document still elaborates");
}

#[test]
fn a_component_defined_twice_is_refused_at_the_documents_own() {
    let src = "use lib.rung\ncomponent Rung(a: point) { }\npoint p\n";
    let (_, linked) = read(src);
    assert_eq!(linked.len(), 1, "{linked:?}");
    assert_eq!(linked[0].code.as_str(), "E071");
    assert_eq!(linked[0].span.lo as usize, src.find("component").unwrap());
    // and across two modules, at the `use` that brought the later one in
    let src2 = "use lib.rung\nuse lib.rung2\npoint p\n";
    let (_, linked) = read(src2);
    assert_eq!(linked.len(), 1, "{linked:?}");
    assert_eq!(linked[0].code.as_str(), "E071");
    assert_eq!(linked[0].span.lo as usize, src2.find("use lib.rung2").unwrap());
    assert!(linked[0].message.starts_with("lib.rung2:"), "{}", linked[0].message);
}

#[test]
fn a_modules_parse_error_is_shown_at_the_use_with_its_own_place() {
    let src = "point p\nuse lib.bad\n";
    let (_, linked) = read(src);
    assert_eq!(linked.len(), 1, "{linked:?}");
    assert_eq!(linked[0].code.as_str(), "E100");
    assert_eq!(linked[0].span.lo as usize, src.find("use").unwrap());
    assert!(linked[0].message.starts_with("lib.bad:2:") || linked[0].message.starts_with("lib.bad:3:"), "{}", linked[0].message);
}

#[test]
fn a_diamond_links_once_and_a_cycle_ends() {
    let (e, linked) = read("use lib.rung\nuse lib.diamond\npoint a\npoint b\ns: Step(a, b)\n");
    assert!(linked.is_empty(), "{linked:?}");
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(e.sketch.lines.len(), 1);
    let (e, linked) = read("use lib.loop_a\npoint p\nq: A(p)\nr: B(p)\n");
    assert!(linked.is_empty(), "{linked:?}");
    assert!(e.ok());
    assert_eq!(e.program.modules.len(), 2);
}

#[test]
fn a_reparse_links_again_from_the_texts_in_hand() {
    let (e, _) = read(LADDER);
    let (mut again, errs) = parse(LADDER);
    assert!(errs.is_empty());
    let d = relink(&mut again, &e.program);
    assert!(d.is_empty());
    assert_eq!(again.modules.len(), 1);
    assert!(elaborate(&again).ok());
}

#[test]
fn a_use_inside_a_body_is_a_syntax_error() {
    let (_, errs) = parse("component C(a: point) {\n  use lib.rung\n}\n");
    assert!(errs.iter().any(|e| e.message.contains("top of a document")), "{errs:?}");
}

#[test]
fn every_span_is_one_integer_into_one_virtual_text() {
    let (e, _) = read(LADDER);
    let p = &e.program;
    let m = &p.modules[0];
    assert_eq!(m.base, LADDER.len() + 1);
    // the module's component sits past the document, and the map says which text it is in
    let rung = gcs_core::modules::component(p, "Rung").expect("linked");
    assert_eq!(rung.module, Some(0));
    assert!(rung.span.lo as usize >= m.base);
    assert_eq!(p.source_at(rung.span.lo as usize).0, Some(0));
    assert_eq!(p.source_at(3).0, None);
    assert!(!p.owns(rung.span));
}

#[test]
fn the_library_resolves_the_shipped_engine() {
    let (prog, errs, linked) =
        gcs_core::library::parse_linked(gcs_core::examples::source("engine").unwrap());
    assert!(errs.is_empty() && linked.is_empty(), "{errs:?} {linked:?}");
    assert!(prog.modules.len() >= 6, "{:?}", prog.modules.iter().map(|m| &m.name).collect::<Vec<_>>());
    assert!(elaborate(&prog).ok());
}
