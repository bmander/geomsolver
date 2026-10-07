//! Modules (§14.4): `use NAME` at the top of a document, resolved by the host and linked by
//! `modules::link` — a module's components are called, and its top-level params and groups read
//! by the root bodies of files that `use` it, **by the module's full path** (`lib.rung.Rung`,
//! `lib.rung.len`) or bare where the `use` names it (`use lib.rung (Rung, len)`, [0.48]); and
//! its own drawing is its own.  Relation words and the import errors are `relation_words.rs`'.

use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::modules::{link, relink};
use gcs_core::program::{elaborate, Elaborated};
use crate::common::STD_POINTS;
use gcs_core::syntax::parse;
use std::collections::BTreeMap;

const RUNG: &str = "\
use std
// a module: a param the caller can pass, and a drawing of its own that is not the document's
len := 50
component Rung(a: point, b: point, len: Length) {
  e := line(a, b)
  horizontal e
  a distance(len) b
}
in std.front {
stray := point hint((999, 999))
}
";

const LADDER: &str = "\
use std
use lib.rung
in std.front {
l0 := point
r0 := point hint((50, 0))
l1 := point hint((0, 20))
r1 := point hint((50, 20))
t0 := lib.rung.Rung(l0, r0, len: lib.rung.len)
t1 := lib.rung.Rung(l1, r1, len: lib.rung.len)
stile := line(l0, l1)
vertical stile
l0 distance(lib.rung.len) l1
fix((0, 0)) l0
}
";

fn shelf() -> BTreeMap<&'static str, &'static str> {
    let mut m = BTreeMap::new();
    m.insert("lib.rung", RUNG);
    m.insert("lib.bad", "component Bad(a: point) {\n  l := line(a,\n}\n");
    m.insert("lib.rung2", "use lib.rung\ncomponent Rung(a: point) { }\n");
    m.insert("lib.diamond", "use lib.rung\ncomponent Step(a: point, b: point, len: Length) { r := lib.rung.Rung(a, b, len: len) }\n");
    m.insert("lib.loop_a", "use lib.loop_b\npa := 1\ncomponent A(p: point) { }\n");
    m.insert("lib.loop_b", "use lib.loop_a\npb := 2\ncomponent B(p: point) { }\n");
    m.insert("lib.over", "use lib.rung\ntwice := 2 * lib.rung.len\ncomponent Long(a: point, b: point, twice: Length) { a distance(twice) b }\n");
    m.insert("lib.units", "size := 10mm\ncomponent UnitLink(a: point, b: point, size: Length) { a distance(size) b }\n");
    m
}

fn read(src: &str) -> (Elaborated, Vec<gcs_core::program::Diag>) {
    let (mut prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let shelf = shelf();
    // the shelf, then the library, as a host resolves: `std` is the library's
    let linked = link(&mut prog, &mut |name| {
        shelf.get(name).map(|t| t.to_string()).or_else(|| gcs_core::library::resolve(name))
    });
    (elaborate(&prog), linked)
}

#[test]
fn a_component_only_file_does_not_instantiate_its_last_definition() {
    let src = "use lib.units\ncomponent Link(a: point, b: point, length: Length) { a distance(length) b }\n";
    let (e, linked) = read(src);
    assert!(linked.is_empty());
    assert!(e.ok(), "{:?}", e.diags);
    assert!(e.sketch.points.is_empty());
    assert!(e.sketch.constraints.is_empty());
    assert!(e.program.root().name.is_none());
    assert_eq!(e.program.text(), src);
}

#[test]
fn preview_solves_only_when_its_file_is_opened() {
    let src = "use std\nshared := 3mm\ncomponent Sample(size: Length) {\n\
        c := circle hint(r: size)\nradius(size) c\nfix((0, 0)) c.center\n}\n\
        preview {\nunit cm\npreview_size := 7cm\n\
        sample := Sample(size: preview_size) in std.front\n}\n";
    let (mut p, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    assert!(link(&mut p, &mut gcs_core::library::resolve).is_empty());
    assert!(p.preview.is_some());
    let mut e = elaborate(&p);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    assert_eq!(e.sketch.circles.len(), 1);
    assert_eq!(e.program.text(), src);

    // The caller can use the same setup names and different units. Neither the preview's
    // geometry nor its params/named dimensions may become part of the importing document.
    let (mut caller, errs) = parse("unit mm\nuse std\nuse part\npreview_size := 11mm\n\
        sample := part.Sample(size: preview_size + part.shared) in std.front\n");
    assert!(errs.is_empty());
    let linked = link(&mut caller, &mut |name| {
        (name == "part").then(|| src.to_string()).or_else(|| gcs_core::library::resolve(name))
    });
    assert!(linked.is_empty(), "{linked:?}");
    let mut imported = elaborate(&caller);
    assert!(imported.ok(), "{:?}", imported.diags);
    assert!(gcs_core::solve::solve(&mut imported.sketch, Default::default()).success);
    assert_eq!(imported.sketch.circles.len(), 1);
    assert!(caller.modules.iter().find(|m| m.name == "part").unwrap().root.body.iter().all(|st| !matches!(st.kind,
        gcs_core::syntax::StmtKind::Unit(_))));
    let positions = gcs_core::report::positions(&imported.sketch, &imported.map);
    assert!(positions.iter().any(|(n, v)| n == "sample.c.r" && (*v - 14.0).abs() < 1e-8));
}

#[test]
fn preview_is_a_single_top_level_block() {
    let runs = gcs_core::syntax::highlight("preview { p := point }");
    assert!(runs.iter().any(|(t, s)| s.lo == 0 && *t == gcs_core::syntax::Tint::Word));
    for src in [
        "preview {}\npreview {}",
        "component Part() { preview {} }",
        "preview { preview {} }",
        "repeat 2 { preview {} }",
        "preview { p := point",
        "preview { line -> }",
    ] {
        let (_, errs) = parse(src);
        assert!(!errs.is_empty(), "accepted {src}");
    }
    let (mut p, errs) = parse("\
use std
preview {
f := plane(u: std.x, v: std.y)
in f { p := point }
}
");
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(p.in_blocks.len(), 1);
    let before = p.text().to_string();
    assert!(gcs_core::syntax::render_flat(&mut p).is_err());
    assert_eq!(p.text(), before);
}

#[test]
fn a_module_contributes_its_components_and_its_params() {
    let (e, linked) = read(LADDER);
    assert!(linked.is_empty(), "{linked:?}");
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    // two rungs and a stile: the module's own stray point is not drawn
    assert_eq!(e.sketch.points.len(), 4 + STD_POINTS);
    assert_eq!(e.sketch.lines.len(), 3);
    let mut sk = e.sketch.clone();
    gcs_core::solve::solve(&mut sk, Default::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    // `l0 distance(lib.rung.len) l1` read the module's `len`: 50 up as well as across
    assert_eq!((d.dof, d.status), (0, State::Well));
    let (x, y) = sk.point_xy(e.map.ent_named("l1").unwrap().i());
    assert!((x - 0.0).abs() < 1e-6 && (y - 50.0).abs() < 1e-6, "{x} {y}");
}

#[test]
fn a_use_names_what_it_reads_bare() {
    // the ladder again, its rungs and their length imported by name beside the full path
    let src = LADDER
        .replace("use lib.rung\n", "use lib.rung (Rung, len)\n")
        .replace("lib.rung.Rung(l0, r0, len: lib.rung.len)", "Rung(l0, r0, len: len)");
    assert!(src.contains("Rung(l0, r0, len: len)") && src.contains("lib.rung.Rung(l1"));
    let (e, linked) = read(&src);
    assert!(linked.is_empty(), "{linked:?}");
    assert!(e.ok(), "{:?}", e.diags);
    let (full, _) = read(LADDER);
    assert_eq!(e.sketch.constraints.len(), full.sketch.constraints.len());
    // nothing listed is nothing imported: no prelude
    let (e, _) = read(&LADDER.replace("lib.rung.Rung(l0, r0, len: lib.rung.len)", "Rung(l0, r0, len: 50)"));
    assert!(e.errors().any(|d| d.message.contains("use lib.rung (Rung)")), "{:?}", e.diags);
}

#[test]
fn a_module_nothing_resolves_is_said_at_the_use() {
    let (e, linked) = read("use lib.nothing\np := point\n");
    assert_eq!(linked.len(), 1);
    assert_eq!(linked[0].code.as_str(), "E070");
    assert!(linked[0].message.contains("lib.nothing"));
    assert_eq!(linked[0].span.lo, 0, "at the `use`");
    assert!(e.ok(), "the rest of the document still elaborates");
}

/// A module's names are its own: the document may define a `Rung` beside `lib.rung`'s, and two
/// modules may each define one.  Only two definitions in one file clash.
#[test]
fn two_files_may_define_one_name_and_one_file_may_not() {
    let src = "\
use std
use lib.rung
use lib.rung2
component Rung(a: point) { b := point }
in std.front {
p := point
q := point
mine := Rung(p)
theirs := lib.rung2.Rung(p)
used := lib.rung.Rung(p, q, len: 10)
}
";
    let (e, linked) = read(src);
    assert!(linked.is_empty(), "{linked:?}");
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert!(e.map.ent_named("mine.b").is_some(), "the document's own Rung");
    assert!(e.map.ent_named("used.e").is_some(), "lib.rung's");
    let (mut p, _) = parse("use lib.twice\n");
    let twice = "component A(p: point) { }\ncomponent A(p: point) { }\n";
    let linked = link(&mut p, &mut |_| Some(twice.into()));
    assert_eq!(linked.len(), 1, "{linked:?}");
    assert_eq!(linked[0].code.as_str(), "E071");
    // and so is one defined twice in the document itself: the first stands, the second is said
    let (p, errs) = parse(&format!("{twice}use std\nin std.front {{\n  o := point\n  a := A(o)\n}}\n"));
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&p);
    let d: Vec<_> = e.diags.iter().filter(|d| d.code.as_str() == "E071").collect();
    assert_eq!(d.len(), 1, "{:?}", e.diags.iter().map(|d| &d.message).collect::<Vec<_>>());
    assert!(d[0].message.contains("`A` is defined twice; the first is at line 1"), "{}", d[0].message);
    assert_eq!(d[0].span.lo as usize, twice.rfind("A(").unwrap());
}

/// **Nothing is imported bare** (§14.4): a module's component, param or group is written with
/// the module's full path, and only through a `use` the file itself wrote.
#[test]
fn a_module_name_is_reached_only_by_its_full_path() {
    // bare: refused, with the spelling that works
    let (e, _) = read("use std\nuse lib.rung\nin std.front {\na := point\nb := point\nr := Rung(a, b, len: 10)\n}\n");
    let said: Vec<&String> = e.errors().map(|d| &d.message).collect();
    assert!(said.iter().any(|m| m.contains("written `lib.rung.Rung`")), "{said:?}");
    let (e, _) = read("use lib.rung\nw := len * 2\n");
    assert!(!e.ok() || e.diags.iter().any(|d| d.message.contains("len")), "{:?}", e.diags);
    // through a module the file did not `use` itself: refused, naming the `use` to write
    let (e, _) = read("use std\nuse lib.diamond\nin std.front {\na := point\nb := point\nr := lib.rung.Rung(a, b, len: 10)\n}\n");
    let said: Vec<&String> = e.errors().map(|d| &d.message).collect();
    assert!(said.iter().any(|m| m.contains("write `use lib.rung`")), "{said:?}");
    // the standard datums too: `hardware` uses `std`, and that is `hardware`'s business
    let (prog, _, linked) = gcs_core::library::parse_linked(
        "use std\nuse hardware\nin std.front {\na := point\na distance(1, along: u) std.front\n}\n");
    assert!(linked.is_empty(), "{linked:?}");
    assert!(!elaborate(&prog).ok(), "`std.front` without a `use std` of the file's own");
    let (prog, _, _) = gcs_core::library::parse_linked(
        "use std\nin std.front {\na := point\na distance(1, along: u) std.front\n}\n");
    assert!(elaborate(&prog).ok());
    // and a name the module does not define
    let (e, _) = read("use std\nuse lib.rung\nin std.front {\na := point\nr := lib.rung.Nope(a)\n}\n");
    assert!(e.errors().any(|d| d.message.contains("defines no component `Nope`")));
}

#[test]
fn a_modules_parse_error_is_shown_at_the_use_with_its_own_place() {
    let src = "p := point\nuse lib.bad\n";
    let (_, linked) = read(src);
    assert_eq!(linked.len(), 1, "{linked:?}");
    assert_eq!(linked[0].code.as_str(), "E100");
    assert_eq!(linked[0].span.lo as usize, src.find("use").unwrap());
    assert!(linked[0].message.starts_with("lib.bad:2:") || linked[0].message.starts_with("lib.bad:3:"), "{}", linked[0].message);
}

#[test]
fn a_diamond_links_once_and_a_cycle_ends() {
    let (e, linked) = read("\
use std
use lib.rung
use lib.diamond
in std.front {
a := point
b := point
s := lib.diamond.Step(a, b, len: lib.rung.len)
}
");
    assert!(linked.is_empty(), "{linked:?}");
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(e.sketch.lines.len(), 1);
    let (e, linked) = read("use lib.loop_a\nuse lib.loop_b\np := point\nq := lib.loop_a.A(p)\n\
                            r := lib.loop_b.B(p)\n");
    assert!(linked.is_empty(), "{linked:?}");
    assert!(e.ok());
    assert_eq!(e.program.modules.len(), 2);
}

/// A module's own params may read the params of the modules it uses, by their path, and the
/// document's read the module's: `twice := 2 * lib.rung.len` is 100 and not free.
#[test]
fn a_files_params_read_the_modules_it_uses() {
    let (e, linked) = read("\
use std
use lib.over
in std.front {
a := point
b := point hint((100, 0))
l := lib.over.Long(a, b, twice: lib.over.twice)
fix((0, 0)) a
b distance(lib.over.twice, along: y) a
}
");
    assert!(linked.is_empty(), "{linked:?}");
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let mut sk = e.sketch.clone();
    gcs_core::solve::solve(&mut sk, Default::default());
    let (x, y) = sk.point_xy(1);
    assert!((x.hypot(y) - 100.0).abs() < 1e-6 && (y + 100.0).abs() < 1e-6, "{x} {y}");
}

#[test]
fn a_reparse_links_again_from_the_texts_in_hand() {
    let (e, _) = read(LADDER);
    let (mut again, errs) = parse(LADDER);
    assert!(errs.is_empty());
    let d = relink(&mut again, &e.program);
    assert!(d.is_empty());
    assert_eq!(again.modules.len(), 2, "lib.rung, and std");
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
    assert_eq!(p.modules[0].base, LADDER.len() + 1, "the first module starts past the document");
    let k = p.modules.iter().position(|m| m.name == "lib.rung").expect("linked");
    let m = &p.modules[k];
    // the module's component sits past the document, and the map says which text it is in
    let rung = &p.components[p.resolve_component("lib.rung.Rung", None).expect("linked")];
    assert_eq!(rung.module, Some(k));
    assert!(rung.span.lo as usize >= m.base);
    assert_eq!(p.source_at(rung.span.lo as usize).0, Some(k));
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
