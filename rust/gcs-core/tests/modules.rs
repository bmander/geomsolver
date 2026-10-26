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
component Rung(a: point, len: Length) {
  b := point
  e := line(a, b)
  horizontal e
  a distance(len, along: u) b
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
l1 := point hint((0, 20))
t0 := lib.rung.Rung(l0, len: lib.rung.len)
t1 := lib.rung.Rung(l1, len: lib.rung.len)
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
    m.insert("lib.diamond", "use lib.rung\ncomponent Step(a: point, len: Length) { r := lib.rung.Rung(a, len: len) }\n");
    m.insert("lib.loop_a", "use lib.loop_b\npa := 1\ncomponent A(p: point) { }\n");
    m.insert("lib.loop_b", "use lib.loop_a\npb := 2\ncomponent B(p: point) { }\n");
    m.insert("lib.over", "use lib.rung\ntwice := 2 * lib.rung.len\ncomponent Long(a: point, twice: Length) {\n  b := point\n  vertical (e := line(a, b))\n  b distance(twice, along: v) a\n}\n");
    m.insert("lib.units", "size := 10mm\ncomponent UnitLink(a: point, b: point, size: Length) { a distance(size) b }\n");
    m.insert("lib.table", "unit mm\nL := 150mm\nhalf := L / 2\nsizes := {gap: 5mm}\n");
    m.insert("lib.arm", "unit mm\nuse lib.table (half)\nreach := lib.table.L - half\n\
        a near b := a distance(lib.table.sizes.gap) b\n\
        component Arm(a: point) {\n  b := point\n  horizontal (e := line(a, b))\n  a distance(reach + lib.table.sizes.gap, along: u) b\n}\n\
        component Theirs(a: point, b: point) { a distance(mine) b }\n");
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

/// **A module's parameter is its drawing's** (#593, [0.180]): a part declaring the parameter its
/// own drawing runs over, opened on its own to scrub its joint, is `use`d as it stands — the
/// parameter left out with the drawing, as the preview is — and nothing outside names it.
#[test]
fn a_modules_parameter_is_left_out_with_its_drawing() {
    let part = "unit mm\nuse std\nsizes := {r: 10mm, h: 5mm}\ncomponent Disc(s: group) {\n  \
        in std.top {\n    rim := radius(s.r) circle(center: std.top.origin)\n  }\n  \
        body := solid(face(rim), from: 0mm, to: s.h)\n}\n\
        param spin: Angle over (0deg, 360deg)\nd := Disc(sizes)\n";
    let elaborated = |src: &str| {
        let (mut prog, errs) = parse(src);
        assert!(errs.is_empty(), "{errs:?}");
        let linked = link(&mut prog, &mut |name| {
            (name == "part").then(|| part.to_string()).or_else(|| gcs_core::library::resolve(name))
        });
        (elaborate(&prog), linked)
    };
    // opened, the part's own parameter is the document's
    let (opened, linked) = elaborated(part);
    assert!(linked.is_empty(), "{linked:?}");
    assert!(opened.ok(), "{:?}", opened.diags);
    assert!(opened.sketch.parameters.contains_key("spin"), "{:?}", opened.sketch.parameters);
    // used, it is not: the using document has none of its own
    let (used, linked) = elaborated("unit mm\nuse std\nuse part (Disc, sizes)\nd := Disc(sizes)\n");
    assert!(linked.is_empty(), "{linked:?}");
    assert!(used.ok(), "{:?}", used.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert!(used.sketch.parameters.is_empty(), "{:?}", used.sketch.parameters);
    assert_eq!(used.sketch.solids.len(), opened.sketch.solids.len());
    // and nothing outside the part names it, bare or by its path
    let (e, _) = elaborated("unit mm\nuse std\nuse part (spin)\n");
    assert!(e.errors().any(|d| d.code.as_str() == "E101" && d.message.contains("spin")),
        "{:?}", e.diags);
    let (e, _) = elaborated("unit mm\nuse std\nuse part\nw := 2 * part.spin\n");
    assert!(e.errors().any(|d| d.message.contains("part.spin")), "{:?}", e.diags);
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
        .replace("lib.rung.Rung(l0, len: lib.rung.len)", "Rung(l0, len: len)");
    assert!(src.contains("Rung(l0, len: len)") && src.contains("lib.rung.Rung(l1"));
    let (e, linked) = read(&src);
    assert!(linked.is_empty(), "{linked:?}");
    assert!(e.ok(), "{:?}", e.diags);
    let (full, _) = read(LADDER);
    assert_eq!(e.sketch.constraints.len(), full.sketch.constraints.len());
    // nothing listed is nothing imported: no prelude
    let (e, _) = read(&LADDER.replace("lib.rung.Rung(l0, len: lib.rung.len)", "Rung(l0, len: 50)"));
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
component Rung(a: point) { b := point\n a level(v) b\n a distance(5, along: u) b }
in std.front {
p := point
q := point
mine := Rung(p)
theirs := lib.rung2.Rung(p)
used := lib.rung.Rung(p, len: 10)
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
        "use hardware\nin std.front {\na := point\na distance(1, along: u) std.front\n}\n");
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
s := lib.diamond.Step(a, len: lib.rung.len)
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
l := lib.over.Long(a, twice: lib.over.twice)
fix((0, 0)) a
}
");
    assert!(linked.is_empty(), "{linked:?}");
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let mut sk = e.sketch.clone();
    gcs_core::solve::solve(&mut sk, Default::default());
    let (x, y) = sk.point_xy(e.map.ent_named("l.b").unwrap().i());
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

/// **The standard library is modules under `std`** (§14.4 [0.195]): each links on its own and
/// clean, no name is defined in two of them, `use std.sets` reaches `std.sets.Sphere` and brings
/// no datum the file may name, and a name written as it was when `std` was one flat namespace is
/// refused saying where it is now.
#[test]
fn the_standard_library_is_modules_under_std() {
    let under: Vec<&str> = gcs_core::library::MODULES.iter().map(|(n, _)| *n)
        .filter(|n| n.starts_with("std.")).collect();
    assert_eq!(under, ["std.place", "std.shapes", "std.sets", "std.rounds", "std.panels",
                       "std.stations", "std.motion"]);
    let mut named: BTreeMap<String, &str> = BTreeMap::new();
    for m in &under {
        let (prog, errs, linked) = gcs_core::library::parse_linked(&format!("use {m}\n"));
        assert!(errs.is_empty() && linked.is_empty(), "{m}: {errs:?} {linked:?}");
        assert!(elaborate(&prog).ok(), "{m}");
        let k = prog.module_named(m).unwrap();
        let comps = prog.components.iter().filter(|c| c.module == Some(k))
            .filter_map(|c| c.name.as_ref().map(|n| n.text.clone()));
        let words = prog.words.iter().filter(|d| d.module == Some(k)).map(|d| d.word.text.clone());
        for n in comps.chain(words).collect::<std::collections::BTreeSet<_>>() {
            assert_eq!(gcs_core::library::home("std", &n), Some(*m), "`{n}`");
            assert!(named.insert(n.clone(), m).is_none(), "`{n}` in {m} and {}", named[&n]);
        }
    }
    let sphere = "in std.front {\n  c := point\n}\nball := std.sets.Sphere(c, r: 5)\n";
    let (prog, _, linked) = gcs_core::library::parse_linked(&format!("use std\nuse std.sets\n{sphere}"));
    assert!(linked.is_empty() && elaborate(&prog).ok(), "{linked:?}");
    // a module under `std` is not `std`: the file names no datum through it
    let (prog, _, _) = gcs_core::library::parse_linked(&format!("use std.sets\n{sphere}"));
    assert!(!elaborate(&prog).ok());
    // the old spellings, each refused naming the new
    let errs = |src: &str| {
        let (prog, _, _) = gcs_core::library::parse_linked(src);
        elaborate(&prog).errors().map(|d| d.message.clone()).collect::<Vec<_>>()
    };
    let said = errs("use std\nin std.front {\n  c := point\n}\nball := std.Sphere(c, r: 5)\n");
    assert!(said.iter().any(|m| m.contains("it is `std.sets.Sphere`, under `use std.sets`")),
            "{said:?}");
    let said = errs("use std (horizontal)\n");
    assert!(said.iter().any(|m| m.contains("`std.place` does, `use std.place (horizontal)`")),
            "{said:?}");
    let said = errs("use std\nin std.front {\n  a := point\n  b := point\n  a horizontal b\n}\n");
    assert!(said.iter().any(|m| m.contains("import it, `use std.place (horizontal)`")), "{said:?}");
}

/// **A definition reads the constants of its file** (#461 PC1): a module's component reads its
/// own file's values and those of the modules its file `use`s, by the names the file reads them
/// by (`lib.table.L`, `half` where the `use` lists it) — with nothing handed in at the call, and
/// to the number one handed in would have given.
#[test]
fn a_definition_reads_the_constants_of_the_modules_its_file_uses() {
    let (e, linked) = read("\
unit mm
use std
use lib.arm
in std.front {
  a := point
  fix((0, 0)) a
}
arm := lib.arm.Arm(a) in std.front
");
    assert!(linked.is_empty() && e.ok(), "{linked:?} {:?}", e.diags);
    let mut sk = e.sketch.clone();
    assert!(gcs_core::solve::solve(&mut sk, Default::default()).success);
    let b = sk.point_xy(e.map.ent_named("arm.b").unwrap().i());
    assert!((b.0 - 80.0).abs() < 1e-9, "{b:?}");
}

/// **Only its own file's** (#461 PC1): the document's numbers are no module definition's — a
/// module's component means the same whoever uses it — and a module's word reads its own file's
/// as its components do.
#[test]
fn a_definition_reads_no_number_of_its_caller() {
    let (e, _) = read("\
unit mm
use std
use lib.arm
mine := 3mm
in std.front {
  a := point
  b := point
}
c := lib.arm.Theirs(a, b)
");
    assert!(e.errors().any(|d| d.message.contains("mine")), "{:?}", e.diags);
    let (e, linked) = read("\
unit mm
use std
use lib.arm (near)
in std.front {
  a := point
  b := point hint((4, 0))
  fix((0, 0)) a
  fix(y == 0) b
  a near b
}
");
    assert!(linked.is_empty() && e.ok(), "{linked:?} {:?}", e.diags);
    let mut sk = e.sketch.clone();
    assert!(gcs_core::solve::solve(&mut sk, Default::default()).success);
    assert!((sk.point_xy(e.map.ent_named("b").unwrap().i()).0 - 5.0).abs() < 1e-9);
}
