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

/// `src` linked against the shelf and `more` before it (a host's resolver over other files),
/// then elaborated: every diagnostic of both.
fn read_with(src: &str, more: &[(&str, &str)]) -> (Elaborated, Vec<Diag>) {
    let (mut prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let shelf = shelf();
    let mut diags = link(&mut prog, &mut |name| {
        more.iter().find(|(n, _)| *n == name).map(|(_, t)| t.to_string())
            .or_else(|| shelf.get(name).cloned()).or_else(|| gcs_core::library::resolve(name))
    });
    let e = elaborate(&prog);
    diags.extend(e.diags.iter().cloned());
    (e, diags)
}

/// Two solids in one product's shape: the drilled block, and the void file's cube beside it
/// (its outer shell renumbered into the block's file), written by hand.
const TWO: &str = include_str!("step_file/two_solids_ap214.step");

/// **A file that gives several solids is refused at the `use`, saying how many** (E072): the
/// reader does give two, and the document reading the name hears one diagnostic, at the `use`.
#[test]
fn a_file_of_several_solids_is_refused_saying_how_many() {
    let solids = gcs_core::brep::step_read::read(TWO).unwrap_or_else(|r| panic!("{r}"));
    assert_eq!(solids.len(), 2);
    let src = "unit mm\nuse two\npart := solid(two)\nset := solid({ p | p in two })\n";
    let (e, diags) = read_with(src, &[("two", TWO)]);
    assert_eq!(diags.len(), 1, "{diags:?}");
    let d = &diags[0];
    assert_eq!((d.code, d.span.lo), (Code::E072, src.find("use two").unwrap() as u32), "{d:?}");
    assert_eq!(d.message, "`two`: the file gives 2 solids, and a given body is one solid");
    assert!(!e.ok());
}

/// The volume and world box of the solid the document calls `name`, in the document's unit.
fn measure(e: &Elaborated, name: &str) -> (f64, [f64; 3], [f64; 3]) {
    let s = e.sketch.evaluated_solid(solid(e, name), gcs_core::solid::ApproximationPolicy::Report)
        .unwrap_or_else(|m| panic!("`{name}`: {m}"));
    let b = s.world_bounds();
    (s.volume(), b.lo, b.hi)
}

fn boxed(got: ([f64; 3], [f64; 3]), lo: [f64; 3], hi: [f64; 3], tol: f64) -> bool {
    (0..3).all(|k| (got.0[k] - lo[k]).abs() <= tol && (got.1[k] - hi[k]).abs() <= tol)
}

/// A frame `f` standing at (100, 0, 0), square to the world, and a 10-cube `box` and `part :=
/// solid(bracket)` at the root: what a read under the frame is compared with.
const POSED: &str = "unit mm\nuse bracket\nuse std\nuse std.solids\nin std.top {\n  c := point\n\
    fix((100, 0)) c\n\
    k0 := point\n  k1 := point\n  k2 := point\n  k3 := point\n  fix((0, 0)) k0\n  fix((10, 0)) k1\n\
    fix((10, 10)) k2\n  fix((0, 10)) k3\n  (s0 := line(k0, k1)) -> (s1 := line(k1, k2)) -> \
    (s2 := line(k2, k3)) -> (s3 := line(k3, k0)) -> close\n  sq := face(s0, s1, s2, s3)\n}\n\
    box := solid(std.solids.Prism(sq, from: 0mm, to: 10mm))\npart := solid(bracket)\n\
    in (f := frame) {\n}\nf.origin coincident c\nf.u parallel std.x\nf.v parallel std.y\n\
    f.u perpendicular f.v\n";

/// **Wherever a read would have to place the given body it is refused, never left at the
/// world** (E106), and wherever the language reads it in place it is right.  Refused: a `ring`'s
/// copies, a mirrored instance, two closed instances of one binding (said once), a frame with a
/// joint, a claim under a posed frame, a component's formal given it in a posed instance.  Right:
/// a body over a body made of it, written under a posed frame, stands where a body over any root
/// solid does (no root solid's body rides a frame); a body made of it turned at an instant of a
/// line; and a `repeat`'s copies, each where the file put it.  (The solid it sweeps about a line:
/// `the_solid_a_given_body_sweeps_is_swept`, in the slow tier.)
#[test]
fn a_given_body_is_refused_wherever_it_would_have_to_be_placed() {
    let e106 = |src: &str, at: &str, words: &str| {
        let (e, diags) = read(src);
        assert!(!e.ok(), "{src}");
        let lo = src.find(at).unwrap_or_else(|| panic!("no `{at}`")) as u32;
        assert!(diags.iter().any(|d| d.code == Code::E106 && d.span.lo == lo
                                     && d.message.contains(words)), "{at}: {diags:?}");
        diags
    };
    let holder = "unit mm\nuse bracket\nuse std\ncomponent Holder() {\n  \
                  part := solid(bracket)\n}\n";
    // a ring turns each copy (the ring's own E021 beside it is the ring's)
    e106("unit mm\nuse bracket\nuse std\nin std.top {\n  hub := point\n  fix((0, 0)) hub\n}\n\
          ring 3 about hub {\n  part := solid(bracket)\n}\n", "bracket)\n}", "a `ring`");
    // a mirrored instance, and two instances of one binding: the read in the definition's body,
    // said once
    for calls in ["left := Holder() mirrored(front)\n", "a := Holder()\nb := Holder()\n"] {
        let src = format!("{holder}{calls}");
        let diags = e106(&src, "bracket)\n}", "a definition's body");
        assert_eq!(diags.len(), 1, "{calls}: {diags:?}");
    }
    // a frame with a joint
    let src = "unit mm\nuse bracket\nuse std\nuse std.motion (spins)\nin std.top {\n  o := point\n\
               fix((0, 0)) o\n}\nin (arm := frame) {\n  part := solid(bracket)\n}\n\
               arm spins(1) o\n";
    let diags = e106(src, "bracket)\n}", "`in arm`");
    assert_eq!(diags.len(), 1, "{diags:?}");
    // a claim under a posed frame
    e106(&format!("{POSED}in f {{\n  claim every q in part: q in bracket\n}}\n"), "bracket\n}",
         "`in f`");
    // a component's formal given it, the instance posed: the body reads it in place of the formal
    let (e, diags) = read(&format!("{POSED}component Copy(b: solid) {{\n  o := solid(b)\n}}\n\
                                    hg := Copy(b: bracket) in f\n"));
    assert!(!e.ok() && diags.iter().any(|d| d.code == Code::E106), "{diags:?}");

    // a body over a body made of it, under the posed frame: where `solid(box)` there stands
    let e = solved(&format!("{POSED}in f {{\n  obox := solid(box)\n  opart := solid(part)\n}}\n"));
    let (v, lo, hi) = measure(&e, "obox");
    assert!(close(v, 1000.0) && boxed((lo, hi), [0.0; 3], [10.0; 3], 1e-9), "{lo:?} {hi:?}");
    let (v, lo, hi) = measure(&e, "opart");
    assert!(close(v, drilled_volume()) && boxed((lo, hi), [0.0; 3], [40.0, 30.0, 20.0], 1e-9),
            "{v} {lo:?} {hi:?}");
    // turned a quarter about the world's z: x from −30 to 0, y from 0 to 40
    let e = solved("unit mm\nuse bracket\nuse std\npart := solid(bracket)\n\
                    turned := part at(under: std.z) 90deg\n");
    let (v, lo, hi) = measure(&e, "turned");
    assert!(close(v, drilled_volume()) && boxed((lo, hi), [-30.0, 0.0, 0.0], [0.0, 40.0, 20.0],
                                                 1e-9), "{v} {lo:?} {hi:?}");
    // a `repeat` turns nothing: each copy is the file's body where it stands
    let e = solved("unit mm\nuse bracket\nuse std\nrepeat 2 {\n  part := solid(bracket)\n}\n");
    for copy in ["#3.0.part", "#3.1.part"] {
        let (v, lo, hi) = measure(&e, copy);
        assert!(close(v, drilled_volume()) && boxed((lo, hi), [0.0; 3], [40.0, 30.0, 20.0], 1e-9),
                "{copy}: {v} {lo:?} {hi:?}");
    }
}

/// **One body per given module, however many files `use` it** (rule 6): the document and a
/// module of its own both `use bracket`, the document reads it in three statements, and the
/// model holds one given body, read by both bodies made of it.
#[test]
fn one_body_per_module_however_many_files_use_it() {
    let src = "unit mm\nuse bracket\nuse lib.holder\nuse std\na := solid(bracket)\n\
               b := solid(bracket)\nclaim every q in a: q in bracket\n";
    let (mut e, diags) = read(src);
    assert!(diags.is_empty() && e.ok(), "{diags:?}");
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let given: Vec<usize> = (0..e.sketch.solids.len())
        .filter(|&i| matches!(e.sketch.solids[i].def, gcs_core::model::SolidDef::Given(_)))
        .collect();
    assert_eq!(given, vec![solid(&e, "bracket")]);
    for name in ["a", "b"] {
        assert!(matches!(e.sketch.solids[solid(&e, name)].def,
                         gcs_core::model::SolidDef::Body { stock, .. }
                             if stock as usize == given[0]));
    }
    // the module's file `use`s it too, and its component reading it is refused where it reads
    let (_, diags) = read("unit mm\nuse bracket\nuse lib.holder\na := solid(bracket)\n\
                           h := lib.holder.Holder()\n");
    assert_eq!(diags.iter().filter(|d| d.code == Code::E106).count(), 1, "{diags:?}");
}

/// **A body with a void, through the language**: its volume `20³ − 4/3·π·4³`; a cube standing in
/// the hollow is not in it, and one in its material is (claims); and its export, by the kernel's
/// writer, passes `step_check` and reads back the same solid as the file gave, void and all.
#[test]
fn a_body_with_a_void_is_read_claimed_and_written_again() {
    use crate::step_roundtrip::{measured, same, stages, Found};
    use gcs_core::brep::{export, step::Fit, sweep::Say};
    let cube = |at: f64| format!("in std.top {{\n  k0 := point\n  k1 := point\n  k2 := point\n\
        k3 := point\n  fix(({a}, {a})) k0\n  fix(({b}, {a})) k1\n  fix(({b}, {b})) k2\n\
        fix(({a}, {b})) k3\n  (s0 := line(k0, k1)) -> (s1 := line(k1, k2)) -> (s2 := line(k2, k3)) \
        -> (s3 := line(k3, k0)) -> close\n  sq := face(s0, s1, s2, s3)\n}}\n\
        cube := solid(std.solids.Prism(sq, from: {a}mm, to: {b}mm))\n", a = at - 0.5, b = at + 0.5);
    let want = 8000.0 - 4.0 / 3.0 * std::f64::consts::PI * 64.0;
    for (at, holds) in [(0.0, false), (7.0, true)] {
        let e = solved(&format!("unit mm\nuse void\nuse std\nuse std.solids\n\
                                 part := solid(void)\n{}\
                                 claim every q in cube: q in part\n", cube(at)));
        assert!((volume(&e, "part") - want).abs() < 1e-9 * want, "{}", volume(&e, "part"));
        let v = gcs_core::diagnose::judge_solids(&e.sketch);
        assert_eq!(v.len(), 1);
        assert_eq!(v[0].holds(), Some(holds), "a cube at {at}: {v:?}");
    }
    let e = solved("unit mm\nuse void\npart := solid(void)\n");
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let x = export::exact(&e.sketch, solid(&e, "part"), None, None, &say)
        .unwrap_or_else(|m| panic!("{m:?}"));
    let text = export::step(&x, "part", None, &say).unwrap_or_else(|m| panic!("{m:?}"));
    assert!(text.contains("BREP_WITH_VOIDS"));
    let original = stages(VOID).unwrap_or_else(|(by, r, _)| panic!("{by:?}: {r}"));
    let back = stages(&text).unwrap_or_else(|(by, r, _)| panic!("read back, {by:?}: {r}"));
    assert_eq!(back.len(), 1);
    let b = &original[0].brep;
    let tol = original[0].context.uncertainty.unwrap_or(1e-7);
    same(b, &measured(b), &text, &back[0], Fit::Exact, tol, &mut Found::default())
        .unwrap_or_else(|d| panic!("not the same solid: {d}"));
}

/// **Edits with a given body in the document**: the gestures the app makes over a solid, and the
/// source map's own re-texts, none panicking.  The given body has no statement of its own (its
/// `use` makes it): deleting it, rounding it, resizing its sweep, drawing on its faces or taking
/// from it is refused in words, the document untouched; the body made of it is rounded whole,
/// taken from, deleted, or its edge refused for faces with no name to write; a statement appended
/// and a re-text are adopted in place, and the outline shows the one object.
#[test]
fn edits_over_a_given_body_are_made_or_refused_in_words() {
    use gcs_core::edit::{self, EdgeWord, Kind, SolidSweep};
    use gcs_core::model::EntRef;
    let src = "unit mm\nuse bracket\npart := solid(bracket)\n";
    let e = solved(src);
    let (g, p) = (solid(&e, "bracket"), solid(&e, "part"));
    let untouched = |x: &edit::Edit| x.kind == Kind::None && x.text == src;
    let refused = |what: &str, x: edit::Edit, words: &str| {
        assert!(untouched(&x), "{what}: {:?}", x.text);
        let why = x.refused.unwrap_or_else(|| panic!("{what}: refused without a word"));
        assert!(why.contains(words), "{what}: {why}");
    };
    let rebuilt = |text: &str| {
        let (mut next, errs) = parse(text);
        assert!(errs.is_empty(), "{errs:?}");
        assert!(relink(&mut next, &e.program).is_empty());
        let mut e = elaborate(&next);
        assert!(e.ok(), "{:?}\n{text}", e.diags);
        assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
        e
    };
    // the given body itself
    refused("delete", edit::remove(&e, &e.program, &e.sketch, &[EntRef::solid(g)], &[]),
            "`bracket` is the body a file gives, made by its `use`");
    let edges = gcs_core::overview::workspace::solid_edges(&e.sketch, g);
    assert_eq!(edges.len(), 14);
    let pair = edges[0].faces.clone();
    for (what, faces, edges, whole) in [("whole", vec![], vec![], true),
                                         ("an edge", vec![], vec![pair.clone()], false),
                                         ("a face", vec![pair.0.clone()], vec![], false)] {
        refused(&format!("round {what}"),
                edit::add_rounds(&e.program, &e.sketch, g, EdgeWord::Round, &faces, &edges, whole,
                                 "1"), "never extended");
    }
    let depth = SolidSweep { depth: Some("5".into()), ..Default::default() };
    refused("resweep", edit::set_sweep(&e, &e.program, g, &depth), "no statement makes");
    refused("turn", edit::set_turn(&e, &e.program, g, "90"), "no statement makes");
    refused("draw on a face", edit::sketch_on_face(&e, &e.sketch, g, &pair.0, None, None), "");
    assert_eq!(edit::face_plane(&e, &e.sketch, g, &pair.0), None);
    refused("cut from it", edit::add_body_word(&e.program, gcs_core::syntax::BodyWord::Cut,
                                               &["part"], "bracket"), "never extended");
    // the body made of it
    let x = edit::add_rounds(&e.program, &e.sketch, p, EdgeWord::Round, &[], &[], true, "1");
    assert_eq!((x.kind, x.refused.as_deref()), (Kind::Structural, None));
    assert!(x.text.ends_with("round(r: 1) part\n"), "{}", x.text);
    let rounded = rebuilt(&x.text);
    // built, or refused in the solid's own words: never a panic
    let _ = rounded.sketch.evaluated_solid(solid(&rounded, "part"),
                                           gcs_core::solid::ApproximationPolicy::Report);
    let pair = gcs_core::overview::workspace::solid_edges(&e.sketch, p)[0].faces.clone();
    refused("round an edge of the body", edit::add_rounds(&e.program, &e.sketch, p,
            EdgeWord::Round, &[], &[pair.clone()], false, "1"), "no name the document can write");
    refused("draw on the body's face", edit::sketch_on_face(&e, &e.sketch, p, &pair.0, None, None),
            "");
    let x = edit::add_body_word(&e.program, gcs_core::syntax::BodyWord::Cut, &["bracket"], "part");
    assert_eq!(x.kind, Kind::Structural, "{:?}", x.refused);
    rebuilt(&x.text);
    // both asked for: the body's statement goes, the given body is kept, and counted
    let x = edit::remove(&e, &e.program, &e.sketch, &[EntRef::solid(g), EntRef::solid(p)], &[]);
    assert_eq!((x.kind, x.text.as_str(), x.kept), (Kind::Structural, "unit mm\nuse bracket\n", 1));
    // a statement appended, adopted; a re-text; and the outline: one row, `part`
    let x = edit::add_point(&e.program, 1.0, 2.0);
    let mut held = solved(src);
    assert!(held.retext("unit mm\nuse bracket\npart := solid(bracket)  \n"));
    assert!(held.adopt(&x.text, &[]));
    rebuilt(&x.text);
    let rows = gcs_core::overview::outline::outline(&held);
    assert_eq!(rows.iter().map(|n| n.label.as_str()).collect::<Vec<_>>(), vec!["part"]);
    assert_eq!(rows[0].solid, Some(p));
    let (mut fresh, _) = read(src);
    let sk = fresh.sketch.clone();
    assert!(untouched(&edit::reconcile(&mut fresh, &sk)));
}

/// **A body whose stock is the given body, untouched, is the same set** (`part :=
/// solid(bracket)`): each holds the other exactly, whatever the bore's faceting — the claim's
/// judge reads the two as one set (`clear::one_set`) and so answers, where comparing their
/// facets alone left it within the faceting's uncertainty; a body made of that body too.  A
/// body with anything taken from it is judged by its boundary as before: never wrongly, though
/// the bore it shares with the file's body leaves both ways within the faceting's uncertainty.
#[test]
fn a_body_and_its_untouched_stock_hold_each_other() {
    let e = solved("unit mm\nuse bracket\nwhole := solid(bracket)\nagain := solid(whole)\n\
                    claim every q in whole: q in bracket\nclaim every q in bracket: q in whole\n\
                    claim every q in again: q in bracket\nclaim every q in bracket: q in again\n");
    let v = gcs_core::diagnose::judge_solids(&e.sketch);
    assert_eq!(v.len(), 4);
    assert!(v.iter().all(|v| v.holds() == Some(true)), "{v:?}");
    // the block drilled again is within the file's body, and the file's body not within it:
    // neither judged the wrong way
    let e = solved(&format!("{DRILL}part := solid(bracket)\ndrill cut part\n\
                             claim every q in part: q in bracket\n\
                             claim every q in bracket: q in part\n"));
    let v = gcs_core::diagnose::judge_solids(&e.sketch);
    assert!(v.len() == 2 && v[0].holds() != Some(false) && v[1].holds() != Some(true), "{v:?}");
}

/// **The library's case** (`rust/examples/given/bought_block.sv`, its file `drilled_block.step`
/// beside it): a block bought in, named by its file, and the one hole a designer drills in it.
/// Read as `solventc` reads it, it is one object, the block less a 5 mm hole through its 20 mm,
/// and its claim on the volume is kept, judged by `std.measures`; the bound brought under the
/// volume, it is refuted.
#[test]
fn the_librarys_case_drills_a_bought_block() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/given");
    let src = std::fs::read_to_string(dir.join("bought_block.sv")).unwrap();
    let judged = |src: &str| {
        let (mut prog, errs) = parse(src);
        assert!(errs.is_empty(), "{errs:?}");
        let linked = link(&mut prog,
                          &mut |m| fixtures::module_beside(&dir, "given/bought_block.sv", m));
        assert!(linked.is_empty(), "{linked:?}");
        let mut e = elaborate(&prog);
        assert!(e.ok(), "{:?}", e.diags);
        assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
        let answers = gcs_core::report::measure_answers(&e.sketch, &e.measures);
        assert_eq!(answers.len(), 1);
        (answers[0].holds, e)
    };
    let (holds, e) = judged(&src);
    assert_eq!(holds, Some(true));
    let block = solid(&e, "block");
    assert_eq!(gcs_core::model::objects(&e.sketch), vec![block]);
    let want = drilled_volume() - std::f64::consts::PI * 2.5 * 2.5 * 20.0;
    assert!(close(volume(&e, "block"), want), "{} against {want}", volume(&e, "block"));
    let (holds, _) = judged(&src.replace("21400 * 1mm^3", "21300 * 1mm^3"));
    assert_eq!(holds, Some(false));
}

/// **The solid a given body sweeps turning about a line is swept**: read at the document's root
/// under a line, a quarter turn about the world's z, it reaches out to the far corner's radius, 50,
/// and holds more than the body.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 20 s: a given body's sweep meshed")]
fn the_solid_a_given_body_sweeps_is_swept() {
    let e = solved("unit mm\nuse bracket\nuse std\nswept := solid({ p | some t in (0deg, 90deg): \
                    p in bracket at(under: std.z) t })\n");
    let (v, lo, hi) = measure(&e, "swept");
    assert!(v > drilled_volume() && boxed((lo, hi), [-30.0, 0.0, 0.0], [40.0, 50.0, 20.0], 1e-2),
            "{v} {lo:?} {hi:?}");
}

/// **A given body's build key reads the document's unit** (#993 stage 2, the review's R1): the
/// file's numbers are millimetres whatever the document says, and a display is in the
/// document's unit, so the same file read in millimetres and in inches is two builds — never the
/// millimetre mesh drawn in the inch drawing.  The same document twice is one.
#[test]
fn a_given_bodys_key_reads_the_documents_unit() {
    let key = |unit: &str| {
        let e = solved(&format!("unit {unit}\nuse bracket\npart := solid(bracket)\n"));
        (gcs_core::solid::field_key(&e.sketch, solid(&e, "part")),
         gcs_core::solid::field_key(&e.sketch, solid(&e, "bracket")))
    };
    let (mm, inch) = (key("mm"), key("in"));
    assert_eq!(mm, key("mm"));
    assert_ne!(mm.0, inch.0);
    assert_ne!(mm.1, inch.1);
}

/// **A given body is never an object, however it is read** (#993 stage 2, the review's R2): read
/// through a set, by a claim, as a body's stock or as a tool, the objects are what the document
/// makes of it — never the given body beside them in the report, the workspace, the cut list or an
/// assembly's export, which all read `model::objects`.
#[test]
fn a_given_body_is_never_an_object() {
    let names = |e: &Elaborated, of: Vec<usize>| -> Vec<String> {
        of.into_iter().map(|i| e.sketch.solids[i].name.clone()).collect()
    };
    let cases: [(&str, &[&str]); 5] = [
        ("unit mm\nuse bracket\nuse std\nb := solid({ p | p in bracket })\n", &["b"]),
        ("unit mm\nuse bracket\nuse std\nb := solid({ p | p in bracket; p distance(>= 10mm, \
          along: n) std.top })\n", &["b"]),
        (&format!("{DRILL}claim every q in drill: q in bracket\n"), &["drill"]),
        (&format!("{DRILL}part := solid(bracket)\ndrill cut part\n"), &["part"]),
        (&format!("{DRILL}pin := solid(drill)\nbracket cut pin\n"), &["pin"]),
    ];
    for (src, want) in cases {
        let e = solved(src);
        assert!(e.sketch.solids.iter().any(|s| s.name == "bracket"), "{src}");
        assert_eq!(names(&e, gcs_core::model::objects(&e.sketch)), want.to_vec(), "{src}");
        assert_eq!(names(&e, gcs_core::overview::shown_objects(&e.sketch)), want.to_vec(), "{src}");
        let cut = gcs_core::stock::cut_list(&e.sketch).unwrap();
        let listed: Vec<String> = cut.iter().flat_map(|c| c.objects.clone()).collect();
        assert_eq!(listed, want.to_vec(), "{src}");
    }
}

/// **Two spellings of one set, by identity** (#993's review, R3): `solid({ p | p in bracket })`
/// is the body whose stock is the given body, as `solid(bracket)` is — the same face names and
/// the same STEP to the byte, and that STEP read back is the file's solid, every face on its own
/// surface (`step_roundtrip::same`).
#[test]
fn two_spellings_of_a_given_body_are_one_solid() {
    use crate::step_roundtrip::{measured, same, stages, Found};
    use gcs_core::brep::{export, step::Fit, sweep::Say};
    let e = solved("unit mm\nuse bracket\nwhole := solid(bracket)\nset := solid({ p | p in bracket })\n");
    assert!(matches!(e.sketch.solids[solid(&e, "set")].def,
                     gcs_core::model::SolidDef::Body { ref on, ref through, ref bound, .. }
                     if on.is_empty() && through.is_empty() && bound.is_empty()));
    // the built B-rep's names, which the report reads (no written path reaches a given face)
    let names = |n: &str| -> Vec<String> {
        let x = e.sketch.exact_solid(solid(&e, n)).unwrap_or_else(|m| panic!("`{n}`: {m}"));
        x.brep.faces.iter().map(|f| f.name.clone()).collect()
    };
    assert_eq!(names("whole"), names("set"));
    assert_eq!(names("set").len(), 7);
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let step = |n: &str| {
        let x = export::exact(&e.sketch, solid(&e, n), None, None, &say)
            .unwrap_or_else(|m| panic!("{m:?}"));
        export::step(&x, "part", None, &say).unwrap_or_else(|m| panic!("{m:?}"))
    };
    let text = step("set");
    assert!(text == step("whole"), "the two spellings' STEP alike");
    let original = stages(DRILLED).unwrap_or_else(|(by, r, _)| panic!("{by:?}: {r}"));
    let back = stages(&text).unwrap_or_else(|(by, r, _)| panic!("read back, {by:?}: {r}"));
    let b = &original[0].brep;
    let tol = original[0].context.uncertainty.unwrap_or(1e-7);
    same(b, &measured(b), &text, &back[0], Fit::Exact, tol, &mut Found::default())
        .unwrap_or_else(|d| panic!("not the same solid: {d}"));
}

/// **A refused file's name still resolves, and only what it leaves unmade is quiet** (#993's
/// review, R4): what is made of the refused body — a body over it, a set of its points, a body
/// over that, a claim over it, a reading of it at an instant — is not made and says nothing, by
/// what the statements' names resolve to; a fault of the statement's own, or a name a scope
/// cannot see, is said as it is with a readable file.
#[test]
fn a_refused_file_leaves_unmade_only_what_is_made_of_it() {
    let codes = |diags: &[Diag]| -> Vec<(Code, String)> {
        diags.iter().map(|d| (d.code, d.message.clone())).collect()
    };
    let src = |file: &str, rest: &str| format!("unit mm\nuse {file}\nuse std\nuse std.solids (cut)\n\
                                                part := solid({file})\n{rest}");
    // an unrelated fault in a statement that reads the body made of it
    let rest = "x := part.volume + nosuch\n";
    let (_, readable) = read(&src("bracket", rest));
    let (_, refused) = read(&src("assembly", rest));
    let nosuch = (Code::E101, "`x`: no such entity: `nosuch`".to_string());
    assert_eq!(codes(&readable), vec![nosuch.clone()], "{readable:?}");
    assert_eq!(refused.len(), 2, "{refused:?}");
    assert_eq!(refused[0].code, Code::E072);
    assert_eq!(codes(&refused[1..]), vec![nosuch]);
    // a definition's body cannot see the document's `part`, whatever its file
    let rest = "component C() {\n  q := point in std.top\n  z := point in part\n}\nc := C()\n";
    for file in ["bracket", "assembly"] {
        let at = src(file, rest).find("z := point in part").unwrap() as u32 + 14;
        let (_, diags) = read(&src(file, rest));
        assert!(diags.iter().any(|d| d.code == Code::E101 && d.span.lo == at
                                 && d.message == "no such entity: `part`"), "{file}: {diags:?}");
    }
    // and everything made of the refused body, however it is read, says nothing more
    let rest = "set := solid({ p | p in assembly })\nother := solid(part)\nmore := solid(set)\n\
                drill := solid(other)\nassembly cut other\nclaim every q in part: q in assembly\n\
                cap := solid({ p | p in assembly; p distance(>= 10mm, along: n) std.top })\n\
                moved := assembly at(under: std.z) 90deg\n";
    let (e, diags) = read(&src("assembly", rest));
    assert_eq!(codes(&diags).len(), 1, "{diags:?}");
    assert_eq!(diags[0].code, Code::E072);
    assert!(e.sketch.solids.is_empty(), "{:?}", e.sketch.solids);
}

/// **A given body read at an instant** (#993's review, R6): `bracket at(under: std.z) 90deg` is
/// the body placed by the turn, as `part at(under: std.z) 90deg` is — a solid, never a frame.
#[test]
fn a_given_body_is_read_at_an_instant() {
    let e = solved("unit mm\nuse bracket\nuse std\nmoved := bracket at(under: std.z) 90deg\n\
                    part := solid(bracket)\nturned := part at(under: std.z) 90deg\n");
    for name in ["moved", "turned"] {
        assert!(close(volume(&e, name), drilled_volume()), "{name}: {}", volume(&e, name));
        let x = e.sketch.exact_solid(solid(&e, name)).unwrap();
        let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
        for v in &x.brep.vertices {
            for k in 0..3 {
                lo[k] = lo[k].min(v.p[k]);
                hi[k] = hi[k].max(v.p[k]);
            }
        }
        // the block from (0, 0, 0) to (40, 30, 20) turned a quarter about the world's z
        let near = |a: [f64; 3], b: [f64; 3]| a.iter().zip(b).all(|(a, b)| (a - b).abs() < 1e-6);
        assert!(near(lo, [-30.0, 0.0, 0.0]) && near(hi, [0.0, 40.0, 20.0]), "{name}: {lo:?} {hi:?}");
    }
    assert_eq!(gcs_core::model::objects(&e.sketch).len(), 3);
}

/// **One cause, one diagnostic, the fix in the author's words** (#993's review, R8): a given body
/// read in a `ring` is refused as one (E106), not also as a name outside the ring (E021); a body
/// word extending it through a definition's formal is refused as extended (E080), not also as
/// read in a definition's body; and the fix an E080 names is the author's own statement, its word
/// as written (`drill bore part`).
#[test]
fn a_given_body_refused_is_said_once_in_the_authors_words() {
    let (_, diags) = read("unit mm\nuse bracket\nuse std\nring 3 about std.origin {\n  \
                           part := solid(bracket)\n}\n");
    assert_eq!(diags.iter().map(|d| d.code).collect::<Vec<_>>(), vec![Code::E106], "{diags:?}");
    let src = format!("{DRILL}component Adds(B: solid, T: solid) {{\n  T cut B\n}}\n\
                       a := Adds(bracket, drill)\n");
    let (_, diags) = read(&src);
    assert_eq!(diags.len(), 1, "{diags:?}");
    assert_eq!(diags[0].code, Code::E080, "{diags:?}");
    assert!(diags[0].message.ends_with("`part := solid(bracket)`, and give `part` where it is \
                                        given"), "{}", diags[0].message);
    let src = format!("{DRILL}x bore B := B and {{ p | p not in x }}\ndrill bore bracket\n");
    let (_, diags) = read(&src);
    let at = src.find("drill bore bracket").unwrap() as u32;
    assert_eq!(diags.len(), 1, "{diags:?}");
    assert_eq!((diags[0].code, diags[0].span.lo), (Code::E080, at), "{diags:?}");
    assert!(diags[0].message.ends_with("`part := solid(bracket)`, then `drill bore part`"),
            "{}", diags[0].message);
}

/// **No name a reader cannot write reaches them** (#993's review, R9): a given body's faces are
/// named by no path until a face is chosen by what it is (stage 3), so the report lists none of
/// them by its key — the body's area holds theirs — and a claim's witness says "a face of
/// `bracket`".
#[test]
fn a_given_bodys_faces_are_said_as_its_faces() {
    let e = solved(&format!("{DRILL}part := solid(bracket)\nclaim every q in drill: q in bracket\n"));
    let rows = gcs_core::report::positions_where(&e.sketch, &e.map, &|n| n == "part"
                                                 || n == "bracket");
    assert!(rows.iter().any(|(n, _)| n == "part.area"), "{rows:?}");
    assert!(rows.iter().all(|(n, _)| !n.contains("#given")), "{rows:?}");
    let v = gcs_core::diagnose::judge_solids(&e.sketch);
    let text = gcs_core::report::solid_claim_text(&v[0]);
    assert!(text.contains("a face of `bracket`") && !text.contains("#given"), "{text}");
    assert_eq!(gcs_core::model::Given::face_said("part.bracket.#given0"),
               "a face of `part.bracket`");
    assert_eq!(gcs_core::model::Given::face_said("drill.hole"), "drill.hole");
}

/// **Statements no text holds are numbered apart** (#993's review, R10): a given body's
/// declaration past every module's statements, a distance's foot's at the very top, so neither is
/// ever another's, nor a statement of the document or a module.
#[test]
fn statements_no_text_holds_are_numbered_apart() {
    use gcs_core::modules::{FOOT_STMTS, GIVEN_STMTS, MODULE_STMTS};
    assert!(MODULE_STMTS < GIVEN_STMTS);
    // a given module's index is far below the room between the two ranges
    assert!(GIVEN_STMTS.checked_add(1 << 20).is_some_and(|top| top < FOOT_STMTS - 2));
}

/// **The corpus through a three-line document** (a tool; `$STEP_CORPUS`, as
/// `step_rewrite::the_corpus_is_written_again` reads it, or `STEP_DOCUMENT_ONLY`, a path within
/// it; `STEP_DOCUMENT_FILES`, more files by path, `:` between): every `.step` and `.stp` file made
/// `input` of `unit mm / use input / part := solid(input)`, linked and elaborated as a host does;
/// `part` built and exported as `solventc --step` does (`brep::export::exact`, then `step`:
/// `step_check` and the kernel's reader on what it wrote); the export made `again` of the same
/// document, built and exported in its turn; and the export read the same solid as the file's
/// (`step_roundtrip::same`), every face on its original surface. One line a file — its outcome,
/// seconds and why — and a tally, on `STEP_DOCUMENT_THREADS` threads (two by default):
/// `STEP_CORPUS=/home/user/step-corpus cargo test -p gcs-core --test core
/// the_corpus_through_a_document -- --ignored --nocapture`.
#[test]
#[ignore = "a tool: STEP_CORPUS names the corpus outside the repository"]
fn the_corpus_through_a_document() {
    use crate::step_roundtrip::{joined, measured, same, stages, Found};
    use gcs_core::brep::export;
    use gcs_core::brep::sweep::Say;
    const QUIET: Say = Say { stage: &|_| {}, mark: &|_| {} };
    let mut files = Vec::new();
    if let Ok(root) = std::env::var("STEP_CORPUS") {
        let base = std::path::Path::new(&root).join(std::env::var("STEP_DOCUMENT_ONLY").unwrap_or_default());
        fn walk(p: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
            let Ok(dir) = std::fs::read_dir(p) else { return };
            let mut entries: Vec<_> = dir.map(|e| e.unwrap().path()).collect();
            entries.sort();
            for e in entries {
                if e.is_dir() { if !e.ends_with("build") { walk(&e, out) } }
                else if e.extension().is_some_and(|x| x == "step" || x == "stp") { out.push(e) }
            }
        }
        walk(&base, &mut files);
    }
    if let Ok(more) = std::env::var("STEP_DOCUMENT_FILES") {
        files.extend(more.split(':').filter(|p| !p.is_empty()).map(std::path::PathBuf::from));
    }
    if files.is_empty() { eprintln!("no STEP_CORPUS nor STEP_DOCUMENT_FILES"); return }
    // `name` made `input` of the three-line document, its `part` built and exported: the
    // document's solid and the file written, or the stage that refused it and why
    fn through(text: &str, name: &str) -> Result<(gcs_core::brep::topo::Brep, String), (&'static str, String)> {
        let src = format!("unit mm\nuse {name}\npart := solid({name})\n");
        let (mut prog, errs) = parse(&src);
        assert!(errs.is_empty(), "{errs:?}");
        let mut diags = link(&mut prog, &mut |n| (n == name).then(|| text.to_string()).or_else(|| gcs_core::library::resolve(n)));
        let mut e = elaborate(&prog);
        diags.extend(e.diags.iter().cloned());
        if let Some(d) = diags.first() { return Err(("refused at the use", d.message.clone())) }
        let r = gcs_core::solve::solve(&mut e.sketch, Default::default());
        if !r.success { return Err(("not solved", r.message)) }
        let i = solid(&e, "part");
        let built = export::exact(&e.sketch, i, None, None, &QUIET).map_err(|r| ("not built", r.message))?;
        let written = export::step(&built, "part", None, &QUIET).map_err(|r| ("not written", r.message))?;
        Ok((built.solid, written))
    }
    let threads: usize = std::env::var("STEP_DOCUMENT_THREADS").ok().and_then(|s| s.parse().ok()).unwrap_or(2);
    let next = std::sync::atomic::AtomicUsize::new(0);
    let rows = std::sync::Mutex::new(Vec::new());
    std::thread::scope(|s| for _ in 0..threads { s.spawn(|| loop {
        let k = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let Some(p) = files.get(k) else { break };
        let Ok(text) = std::fs::read_to_string(p) else { continue };
        let started = std::time::Instant::now();
        let got = std::panic::catch_unwind(|| -> Result<String, (&'static str, String)> {
            let (b, written) = through(&text, "input")?;
            // the export a given body of the same document in its turn, and the file's solid again
            let (_, twice) = through(&written, "again").map_err(|(stage, why)| (stage, format!("the export, read again: {why}")))?;
            let again = joined(stages(&written).map_err(|(by, r, _)| ("not the same", format!("the export read by {by:?}: {r}")))?);
            let tol = again.context.uncertainty.unwrap_or(1e-7);
            same(&b, &measured(&b), &written, &again, gcs_core::brep::step::Fit::Exact, tol, &mut Found::default())
                .map_err(|d| ("not the same", d))?;
            Ok(format!("{} faces; its export {} bytes, again {} bytes", b.faces.len(), written.len(), twice.len()))
        }).unwrap_or_else(|_| Err(("panicked", String::new())));
        let (outcome, why) = match got { Ok(said) => ("built, exported, read again the same", said), Err((stage, why)) => (stage, why) };
        eprintln!("DOCUMENT\t{}\t{outcome}\t{:.1}\t{why}", p.display(), started.elapsed().as_secs_f64());
        rows.lock().unwrap().push(outcome);
    }); });
    let mut tally: BTreeMap<&str, usize> = BTreeMap::new();
    for outcome in rows.into_inner().unwrap() { *tally.entry(outcome).or_default() += 1; }
    for (k, n) in &tally { eprintln!("DOCUMENT SUMMARY {n:4}  {k}"); }
}

/// **A given body's volume and area are its B-rep's, read there** (`report::positions_where`,
/// #993 stage 2): `--where part.volume` on a body a file gives, untouched, meshes nothing — its
/// evaluation meshed it at the report's fineness for a volume its B-rep holds exactly (CADGenBench
/// 201, half a minute against a build of a second) — while a question of its box still evaluates
/// it, the one measure its facets give.
#[test]
fn a_given_bodys_measures_are_read_off_its_b_rep() {
    let e = solved("unit mm\nuse bracket\npart := solid(bracket)\n");
    let before = gcs_core::solid::brep_meshes();
    let rows = gcs_core::report::positions_where(&e.sketch, &e.map, &|n| n == "part" || n == "part.volume");
    assert_eq!(gcs_core::solid::brep_meshes(), before, "the body was meshed for its volume");
    let row = |name: &str| rows.iter().find(|(n, _)| n == name).map(|r| r.1);
    assert!(row("part.volume").is_some_and(|v| close(v, drilled_volume())), "{rows:?}");
    assert!(row("part.area").is_some(), "{rows:?}");
    assert!(row("part.bounds.x0").is_none(), "{rows:?}");
    // asked of its box, it is evaluated: its box the block's
    let rows = gcs_core::report::positions_where(&e.sketch, &e.map, &|n| n.starts_with("part"));
    let x1 = rows.iter().find(|(n, _)| n == "part.bounds.x1").map(|r| r.1);
    assert!(x1.is_some_and(|x| (x - 40.0).abs() < 1e-6), "{rows:?}");
}
