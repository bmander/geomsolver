//! **A given body** (#993 stage 2): `use bracket`, where the host resolves `bracket` to a STEP
//! file's text, makes `bracket` a *given module* (`modules::is_step`), never parsed as Solvent, and
//! the one thing it contributes is the body the file gives, named by the module's whole path and
//! read wherever a solid's name is read: `part := solid(bracket)`.  Linking keeps the file beside
//! the module, read once (`modules::Given`); an edit never reads it again.  The documents here are
//! linked against an in-memory shelf, as a host's resolver would answer.

use gcs_core::modules::{self, link, relink};
use gcs_core::program::{elaborate, Code, Diag, Elaborated};
use gcs_core::syntax::{parse, Program};
use std::collections::BTreeMap;
use std::sync::Arc;

const DRILLED: &str = include_str!("step_file/drilled_ap214.step");
const INCH: &str = include_str!("step_file/drilled_inch.step");
const VOID: &str = include_str!("step_file/void_ap214.step");
const ASSEMBLY: &str = include_str!("step_file/assembly_ap214.step");
const SHELL: &str = include_str!("step_file/shell_ap214.step");
const NAMED: &str = include_str!("step_file/named_ap214.step");

/// The modules a host would find: the fixtures under the names a document `use`s.
fn shelf() -> BTreeMap<&'static str, String> {
    let mut m = BTreeMap::new();
    m.insert("bracket", DRILLED.to_string());
    m.insert("vendor.bracket", DRILLED.to_string());
    m.insert("inch", INCH.to_string());
    m.insert("void", VOID.to_string());
    m.insert("assembly", ASSEMBLY.to_string());
    m.insert("shell", SHELL.to_string());
    m.insert("named", NAMED.to_string());
    m.insert("bom", format!("\u{feff}  \n{DRILLED}"));
    // cut short half way through its data
    m.insert("truncated", DRILLED[..DRILLED.len() / 2].to_string());
    m.insert("garbage", "ISO-10303-21;\nthis is no exchange structure at all ((( #1 = ;\n".into());
    // a Solvent module that uses a given one in turn
    m.insert("lib.holder", "use bracket\ncomponent Holder() {\n  part := solid(bracket)\n}\n".into());
    m
}

/// `src` parsed and linked against the shelf (then the library), as a host does.
fn linked(src: &str) -> (Program, Vec<Diag>) {
    let (mut prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let shelf = shelf();
    let diags = link(&mut prog, &mut |name| {
        shelf.get(name).cloned().or_else(|| gcs_core::library::resolve(name))
    });
    (prog, diags)
}

/// `src` linked and elaborated, every diagnostic of both.
fn read(src: &str) -> (Elaborated, Vec<Diag>) {
    let (prog, mut diags) = linked(src);
    let e = elaborate(&prog);
    diags.extend(e.diags.iter().cloned());
    (e, diags)
}

#[test]
fn a_step_text_is_known_by_its_first_token() {
    assert!(modules::is_step(DRILLED));
    assert!(modules::is_step("\u{feff}\n  \tISO-10303-21;\nHEADER;"));
    assert!(!modules::is_step("use std\n// ISO-10303-21;\n"));
    assert!(!modules::is_step("ISO-10303-21\n"));
    assert!(!modules::is_step(""));
}

/// The digest is a function of the bytes alone, fixed on every target: its value is held here,
/// so a change to it is a change every worker's key would see.
/// `digest("ISO-10303-21;")`, as every target computes it.
const DIGEST: u64 = 0x1251_ceaa_192e_bdc9;

#[test]
fn a_digest_is_the_same_everywhere() {
    assert_eq!(modules::digest(DRILLED), modules::digest(&DRILLED.to_string()));
    assert_ne!(modules::digest(DRILLED), modules::digest(INCH));
    assert_ne!(modules::digest("a"), modules::digest("b"));
    assert_ne!(modules::digest(""), modules::digest("\0"));
    assert_eq!(modules::digest("ISO-10303-21;"), DIGEST, "{:#x}", modules::digest("ISO-10303-21;"));
}

/// A given module is linked with no text in the virtual text — one byte — and its file beside
/// it, read once: a second module linked after it starts one byte on.
#[test]
fn a_given_module_holds_its_file_beside_it() {
    let (p, diags) = linked("unit mm\nuse bracket\nuse std\n");
    assert!(diags.is_empty(), "{diags:?}");
    let k = p.module_named("bracket").expect("linked");
    let m = &p.modules[k];
    let g = m.given.as_ref().expect("a given module");
    assert!(m.text.is_empty() && m.root.body.is_empty() && m.uses.is_empty());
    assert_eq!(g.text, DRILLED);
    assert_eq!(g.digest, modules::digest(DRILLED));
    assert!(g.body.is_ok(), "{:?}", g.body);
    assert_eq!(p.module_text("bracket").as_deref(), Some(DRILLED));
    let s = p.module_named("std").unwrap();
    assert_eq!(p.modules[s].base, m.base + 1);
    assert_eq!(p.source_at(m.base), (Some(k), 0));
}

/// **An edit never reads the file again**: a re-parse relinks from the program in hand, and the
/// reading it hands on is the same allocation.  A fresh link of the same text on the same thread
/// finds it in the memo too.
#[test]
fn an_edit_hands_the_reading_on() {
    let (p, _) = linked("unit mm\nuse bracket\npart := solid(bracket)\n");
    let was = p.modules[p.module_named("bracket").unwrap()].given.clone().unwrap();
    let (mut next, errs) = parse("unit mm\nuse bracket\npart := solid(bracket)\nx := 2\n");
    assert!(errs.is_empty());
    let diags = relink(&mut next, &p);
    assert!(diags.is_empty(), "{diags:?}");
    let now = next.modules[next.module_named("bracket").unwrap()].given.clone().unwrap();
    assert!(Arc::ptr_eq(&was, &now));
    // and through the program's own re-parse, the one a numeric edit makes (`retext`)
    let mut e = elaborate(&p);
    assert!(e.retext("unit mm\nuse bracket\npart := solid(bracket) \n"));
    let again = &e.program.modules[e.program.module_named("bracket").unwrap()].given;
    assert!(Arc::ptr_eq(&was, again.as_ref().unwrap()));
    // a fresh link of the same text: the memo's reading
    let (fresh, _) = linked("unit mm\nuse bracket\n");
    let memo = fresh.modules[fresh.module_named("bracket").unwrap()].given.clone().unwrap();
    assert!(Arc::ptr_eq(&was, &memo));
}

/// Every text the kernel refuses is refused once, at the `use`, in the kernel's words with the
/// module's name in front — no panic, whatever follows the first token.
#[test]
fn a_file_the_kernel_refuses_is_one_diagnostic_at_the_use() {
    for name in ["assembly", "shell", "truncated", "garbage"] {
        let src = format!("unit mm\nuse {name}\n");
        let (e, diags) = read(&src);
        let at = src.find(&format!("use {name}")).unwrap() as u32;
        assert_eq!(diags.len(), 1, "{name}: {diags:?}");
        let d = &diags[0];
        assert_eq!(d.code, Code::E072, "{name}: {d:?}");
        assert_eq!(d.span.lo, at, "{name}");
        assert!(d.message.starts_with(&format!("`{name}`")), "{name}: {}", d.message);
        assert!(!e.ok());
    }
}

/// The drilled block's volume: a 40 × 30 × 20 block, drilled through by a hole of radius 6.
fn drilled_volume() -> f64 {
    40.0 * 30.0 * 20.0 - std::f64::consts::PI * 36.0 * 20.0
}

/// `src` read, every diagnostic none, and solved.
fn solved(src: &str) -> Elaborated {
    let (mut e, diags) = read(src);
    assert!(diags.is_empty() && e.ok(), "{diags:?}\n{src}");
    let r = gcs_core::solve::solve(&mut e.sketch, Default::default());
    assert!(r.success, "{}", r.message);
    e
}

/// The solid the document calls `name`, by index.
fn solid(e: &Elaborated, name: &str) -> usize {
    e.sketch.solids.iter().position(|s| s.name == name).unwrap_or_else(|| panic!("no `{name}`"))
}

/// Its exact volume.
fn volume(e: &Elaborated, name: &str) -> f64 {
    let x = e.sketch.exact_solid(solid(e, name)).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    gcs_core::brep::props::volume(&x.brep)
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * b.abs().max(1.0)
}

/// What is not read is not there: `use bracket` alone makes no solid, and `part :=
/// solid(bracket)` one object, `part` — the given body its stock, an operand and no part.
#[test]
fn the_body_is_read_by_the_modules_path_and_only_where_read() {
    let e = solved("unit mm\nuse bracket\n");
    assert!(e.sketch.solids.is_empty(), "{:?}", e.sketch.solids);
    let e = solved("unit mm\nuse bracket\npart := solid(bracket)\n");
    assert_eq!(e.sketch.solids.len(), 2, "{:?}", e.sketch.solids);
    assert!(matches!(e.sketch.solids[solid(&e, "bracket")].def,
                     gcs_core::model::SolidDef::Given(_)));
    let part = solid(&e, "part");
    assert_eq!(gcs_core::model::objects(&e.sketch), vec![part]);
    assert!(close(volume(&e, "part"), drilled_volume()), "{}", volume(&e, "part"));
    // one object wherever objects are read: the workspace, the cut list, an assembly's export
    assert_eq!(gcs_core::overview::shown_objects(&e.sketch), vec![part]);
    let cut = gcs_core::stock::cut_list(&e.sketch).unwrap();
    assert_eq!(cut.len(), 1);
    assert_eq!(cut[0].objects, vec!["part".to_string()]);
    let say = gcs_core::brep::sweep::Say { stage: &|_: &str| {}, mark: &|_| {} };
    let a = gcs_core::brep::export::assembly(&e.sketch, &[part], "drawing", None, &|_| None, &say,
                                             &mut |_, _| {})
        .unwrap_or_else(|(i, m)| panic!("{i}: {m}"));
    assert_eq!(a.products.iter().filter(|p| p.solid.is_some()).count(), 1);
    // by its whole path, where a module under a directory is used
    let e = solved("unit mm\nuse vendor.bracket\npart := solid(vendor.bracket)\n");
    assert!(close(volume(&e, "part"), drilled_volume()));
    // a file reads only what it `use`s
    let (_, diags) = read("unit mm\nuse vendor.bracket\npart := solid(bracket)\n");
    assert!(diags.iter().any(|d| d.code == Code::E101), "{diags:?}");
}

/// **Two spellings of one set**: `solid(bracket)` and `solid({ p | p in bracket })` are one
/// solid.  The claim of containment is judged both ways: between the given body and the body
/// made of it, whose boundaries are one, neither way is refuted; a block within the material is
/// within it, and one standing out of it is not.
#[test]
fn a_given_body_is_a_set() {
    let e = solved("unit mm\nuse bracket\nuse std\nwhole := solid(bracket)\n\
                    set := solid({ p | p in bracket })\n\
                    claim every q in whole: q in bracket\nclaim every q in bracket: q in whole\n");
    assert!(close(volume(&e, "set"), drilled_volume()), "{}", volume(&e, "set"));
    let faces = |n: &str| e.sketch.exact_solid(solid(&e, n)).unwrap().brep.faces.len();
    assert_eq!(faces("set"), faces("whole"));
    let v = gcs_core::diagnose::judge_solids(&e.sketch);
    assert_eq!(v.len(), 2);
    assert!(v.iter().all(|v| v.holds() != Some(false)), "{v:?}");
    // a pin within the material, and one above the block
    let pins = |from: i32, to: i32| DRILL.replace("from: -30mm, to: 30mm",
        &format!("from: {from}mm, to: {to}mm"));
    for (from, to, holds) in [(5, 15, true), (25, 35, false)] {
        let e = solved(&(pins(from, to) + "claim every q in drill: q in bracket\n"));
        let v = gcs_core::diagnose::judge_solids(&e.sketch);
        assert_eq!(v[0].holds(), Some(holds), "{from}..{to}: {v:?}");
    }
}

/// A body word takes a given body as its tool, and a body made of one as its body: `drill cut
/// part` is the block drilled twice, to its closed form.
const DRILL: &str = "unit mm\nuse bracket\nuse std\nuse std.solids (cut)\nin std.top {\n\
    c := point hint((10, 10))\n  fix((10, 10)) c\n  hole := circle(center: c) hint(r: 3)\n\
    radius(3) hole\n}\ndrill := solid(std.solids.Prism(face(hole), from: -30mm, to: 30mm))\n";

#[test]
fn a_given_body_is_an_operand() {
    let e = solved(&format!("{DRILL}part := solid(bracket)\ndrill cut part\n"));
    let want = drilled_volume() - std::f64::consts::PI * 9.0 * 20.0;
    assert!(close(volume(&e, "part"), want), "{} against {want}", volume(&e, "part"));
    assert_eq!(gcs_core::model::objects(&e.sketch), vec![solid(&e, "part")]);
    // the tool of a body word: the block taken from a pin through it leaves its two ends
    let e = solved(&format!("{DRILL}pin := solid(drill)\nbracket cut pin\n"));
    let ends = std::f64::consts::PI * 9.0 * 40.0;
    assert!(close(volume(&e, "pin"), ends), "{} against {ends}", volume(&e, "pin"));
}

/// **A given body is never extended**: a statement that would add to it, or take from it, is
/// refused where it is written, naming the body to make instead.
#[test]
fn a_given_body_is_never_extended() {
    for word in ["cut", "union", "common"] {
        let src = DRILL.replace("(cut)", &format!("({word})")) + &format!("drill {word} bracket\n");
        let (_, diags) = read(&src);
        let at = src.find(&format!("drill {word} bracket")).unwrap() as u32;
        assert_eq!(diags.len(), 1, "{word}: {diags:?}");
        let d = &diags[0];
        assert_eq!((d.code, d.span.lo), (Code::E080, at), "{word}: {d:?}");
        assert!(d.message.contains(&format!("`part := solid(bracket)`, then `drill {word} \
                                              part`")), "{}", d.message);
    }
    // and the primitive the words are written in, said of it outside it
    for set in ["bracket and { p | p not in drill }", "bracket or { p | p in drill }"] {
        let src = format!("{DRILL}{set}\n");
        let (_, diags) = read(&src);
        assert_eq!(diags.len(), 1, "{set}: {diags:?}");
        assert_eq!(diags[0].code, Code::E080, "{set}: {diags:?}");
        assert!(diags[0].message.contains("`part := solid(bracket)`"), "{}", diags[0].message);
    }
}

/// `use bracket (x)` names nothing the module defines, and a file's own name over the module's
/// is two things by one name.
#[test]
fn a_given_module_defines_nothing_but_its_body() {
    let (_, diags) = read("unit mm\nuse bracket (x)\n");
    assert!(diags.iter().any(|d| d.code == Code::E101 && d.message.contains("defines no `x`")),
            "{diags:?}");
    let (_, diags) = read("unit mm\nuse std\nuse bracket\nbracket := point in std.front\n");
    assert!(diags.iter().any(|d| d.code == Code::E071), "{diags:?}");
}

/// **Where it stands**: the document's own space, whatever plane of the world a statement is
/// written in; refused where it would have to be placed — a definition's body, a frame not the
/// world's — at the read.
#[test]
fn a_given_body_stands_in_the_documents_space() {
    let e = solved("unit mm\nuse bracket\nuse std\nin std.top {\n  part := solid(bracket)\n}\n");
    assert!(close(volume(&e, "part"), drilled_volume()));
    let x = e.sketch.exact_solid(solid(&e, "part")).unwrap();
    // the file's origin and axes are the world's: its corners from (0, 0, 0) to (40, 30, 20)
    let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
    for v in &x.brep.vertices {
        for k in 0..3 {
            lo[k] = lo[k].min(v.p[k]);
            hi[k] = hi[k].max(v.p[k]);
        }
    }
    let near = |a: [f64; 3], b: [f64; 3]| a.iter().zip(b).all(|(a, b)| (a - b).abs() < 1e-6);
    assert!(near(lo, [0.0; 3]) && near(hi, [40.0, 30.0, 20.0]), "{lo:?} {hi:?}");
    // in a definition's body
    let src = "unit mm\nuse bracket\nuse std\ncomponent Holder() {\n  part := solid(bracket)\n}\n\
               h := Holder()\n";
    let (e, diags) = read(src);
    let at = src.find("solid(bracket)").unwrap() as u32 + 6;
    assert_eq!(diags.len(), 1, "{diags:?}");
    assert_eq!((diags[0].code, diags[0].span.lo), (Code::E106, at), "{diags:?}");
    assert!(diags[0].message.contains("a definition's body"), "{}", diags[0].message);
    assert!(!e.ok());
    // and in a module's, the module `use`ing it in turn
    let (_, diags) = read("unit mm\nuse lib.holder\nh := lib.holder.Holder()\n");
    assert!(diags.iter().any(|d| d.code == Code::E106), "{diags:?}");
    // under a frame that is not the world's
    let src = "unit mm\nuse bracket\nuse std\nf := frame\nin f {\n  part := solid(bracket)\n}\n";
    let (_, diags) = read(src);
    assert!(diags.iter().any(|d| d.code == Code::E106 && d.message.contains("`in f`")),
            "{diags:?}");
}

/// **Units**: the file's lengths are the file's, carried to the document's unit.  A `unit in`
/// document reading the millimetre file gets the same body measured in inches; the file written
/// in inches gives the same block.
#[test]
fn a_given_body_is_measured_in_the_documents_unit() {
    // the volume the model measures, in the document's unit
    let measured = |e: &Elaborated| e.sketch
        .evaluated_solid(solid(e, "part"), gcs_core::solid::ApproximationPolicy::Report)
        .unwrap_or_else(|m| panic!("{m}")).volume();
    let e = solved("unit in\nuse bracket\npart := solid(bracket)\n");
    let inch3 = 25.4f64.powi(3);
    assert!(close(measured(&e), drilled_volume() / inch3), "{}", measured(&e));
    let e = solved("unit mm\nuse inch\npart := solid(inch)\n");
    assert!((measured(&e) - drilled_volume()).abs() < 1e-6 * drilled_volume(), "{}", measured(&e));
}

/// A file of several solids is refused at the `use`, saying how many; a read of a refused
/// file's name says nothing more.
#[test]
fn a_refused_file_is_said_once_however_it_is_read() {
    let src = "unit mm\nuse assembly\nuse std\nuse std.solids (cut)\npart := solid(assembly)\n\
               set := solid({ p | p in assembly })\nother := solid(part)\nassembly cut other\n\
               claim every q in part: q in assembly\n";
    let (_, diags) = read(src);
    assert_eq!(diags.len(), 1, "{diags:?}");
    assert_eq!(diags[0].code, Code::E072);
}

/// **The stage gate** (#993 stage 2): a document that only names a file exports the same solid —
/// written by the kernel's writer (`export::exact`, `export::step`, which holds what it writes to
/// `step_check::verify`), read back, every face on its original surface in its order, every edge
/// and vertex where the file had it (`step_roundtrip::same`, against the reading of the file).
#[test]
fn a_document_naming_a_file_exports_the_same_solid() {
    use crate::step_roundtrip::{measured, same, stages, Found};
    use gcs_core::brep::{export, step::Fit, sweep::Say};
    let e = solved("unit mm\nuse bracket\npart := solid(bracket)\n");
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let x = export::exact(&e.sketch, solid(&e, "part"), None, None, &say)
        .unwrap_or_else(|m| panic!("{m:?}"));
    let text = export::step(&x, "part", None, &say).unwrap_or_else(|m| panic!("{m:?}"));
    let original = stages(DRILLED).unwrap_or_else(|(by, r, _)| panic!("{by:?}: {r}"));
    let back = stages(&text).unwrap_or_else(|(by, r, _)| panic!("read back, {by:?}: {r}"));
    assert_eq!(back.len(), 1);
    let b = &original[0].brep;
    let tol = original[0].context.uncertainty.unwrap_or(1e-7);
    same(b, &measured(b), &text, &back[0], Fit::Exact, tol, &mut Found::default())
        .unwrap_or_else(|d| panic!("not the same solid: {d}"));
}
