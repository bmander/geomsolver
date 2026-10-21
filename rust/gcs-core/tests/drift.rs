//! **The drift lint** (#462, `program::drift`): six patterns that are drift from Solvent's style
//! by definition, each a warning of its own code, asked for and never raised by elaboration.  A
//! document that trips each pattern and one beside it that does not, and the corpus held to a
//! list of what it already has that can only shrink.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use gcs_core::program::{drift, elaborate, DRIFT};
use crate::common::{parse, parsed::prog_of};

/// What the lint says of `src`, each finding `CODE: the text it is at`.
fn lint(src: &str) -> Vec<String> {
    let p = prog_of(src);
    drift(&p).iter().map(|d| format!("{}: {}", d.code.as_str(), d.span.slice(src))).collect()
}

/// The findings of one code.
fn of(code: &str, src: &str) -> Vec<String> {
    lint(src).into_iter().filter(|f| f.starts_with(code)).collect()
}

const GEAR: &str = "\
component Gear(o: point, teeth: Int, m: Length) {
  pitch := circle(center: o) hint(r: m * teeth / 2)
  radius(m * teeth / 2) pitch
}
";

#[test]
fn a_label_given_the_name_it_labels_is_a_pun() {
    let src = format!("{GEAR}teeth := 12
in std.front {{
  o := point
  u := line
  v := line
  g := Gear(o, teeth: teeth, m: 2mm)
  h := Gear(o, teeth: 12, m: 2mm)
  f := plane(u: u, v: v)
}}
");
    assert_eq!(of("W201", &src), ["W201: teeth"], "{:?}", lint(&src));
}

#[test]
fn a_pun_to_a_defined_word_is_said_and_a_built_ins_slot_is_not() {
    let src = "\
a apart(d) b := a distance(d) b
component Pair(a: point, b: point, d: Length, along: line) {
  a apart(d: d) b
  a distance(0mm, along: along) b
}
";
    assert_eq!(of("W201", src), ["W201: d"], "{:?}", lint(src));
}

#[test]
fn a_component_call_standing_as_a_statement_is_said() {
    let src = "\
use std (horizontal)
component Level(a: point, b: point) {
  a horizontal b
}
in std.front {
  a := point
  b := point
  Level(a, b)
  l := Level(a, b)
}
ax := axis
q := point
q inside std.Cylinder(ax, r: 8mm)
";
    assert_eq!(of("W202", src), ["W202: Level"], "{:?}", lint(src));
}

#[test]
fn a_component_that_makes_nothing_a_parent_reads_is_a_verb() {
    let src = "\
use std (horizontal)
component Level(a: point, b: point) {
  a horizontal b
}
component Apart(a: point, b: point, d: Length) {
  private m := point
  m distance(d) a
  m distance(d) b
}
component Mid(a: point, b: point) {
  m := point
  m midpoint line(a, b)
}
component Nothing(a: point) {}
";
    assert_eq!(of("W203", src), ["W203: Level", "W203: Apart"], "{:?}", lint(src));
    let said = drift(&prog_of(src));
    assert!(said.iter().any(|d| d.message.contains("declares nothing")), "{said:?}");
    assert!(said.iter().any(|d| d.message.contains("all it declares is private")), "{said:?}");
}

const ARM: &str = "\
component Arm(o: point) {
  tip := point
  rod := line(o, tip)
}
";

#[test]
fn a_path_through_one_instance_into_another_is_deep() {
    // what the instance names as its own — a group of aliases — is read past it, not through it
    let src = format!("{ARM}component Pair(o: point) {{
  arm := Arm(o)
  end := point
  ends := {{tip: arm.tip}}
}}
in std.front {{
  o := point
  pair := Pair(o)
  pair.arm.tip distance(10mm) o
  pair.end distance(10mm) o
  pair.ends.tip distance(10mm) o
}}
");
    assert_eq!(of("W204", &src), ["W204: pair.arm.tip"], "{:?}", lint(&src));
    // a line's end is the line's, not another level's
    let src = format!("{ARM}in std.front {{
  o := point
  arm := Arm(o)
  arm.rod.p2 distance(10mm) o
}}
");
    assert!(of("W204", &src).is_empty(), "{:?}", lint(&src));
    // a block's next copy is the same body's: its instance's instance is as deep
    let src = format!("{ARM}component Pair(o: point) {{
  arm := Arm(o)
}}
in std.front {{
  o := point
  cycle 3 {{
    pair := Pair(o)
    span := line(pair.arm.tip, next.pair.arm.tip)
  }}
}}
");
    assert_eq!(of("W204", &src), ["W204: pair.arm.tip", "W204: next.pair.arm.tip"], "{:?}",
        lint(&src));
}

#[test]
fn an_int_read_only_by_when_is_a_presentation_flag() {
    let src = "\
component Part(o: point, draw: Int, n: Int, keep: Int) {
  when draw == 1 {
    c := circle(center: o)
  }
  repeat n {
    p := point
  }
  when keep == 1 {
    q := point hint((keep, 0))
  }
}
";
    assert_eq!(of("W205", src), ["W205: draw"], "{:?}", lint(src));
}

#[test]
fn a_group_handed_on_whole_and_never_read_is_threaded() {
    let src = "\
component Inner(o: point, dims: group) {
  c := circle(center: o) hint(r: dims.r)
}
component Outer(o: point, dims: group) {
  i := Inner(o, dims)
}
component Reads(o: point, dims: group) {
  i := Inner(o, dims)
  q := point hint((dims.r, 0))
}
";
    assert_eq!(of("W206", src), ["W206: dims"], "{:?}", lint(src));
    let said = drift(&prog_of(src));
    let line = |d: &gcs_core::program::Diag| src[..d.span.lo as usize].lines().count();
    assert_eq!(said.iter().map(line).collect::<Vec<_>>(), [4], "{said:?}");
}

/// A group a solid is made of is read whole where it is used, as a material (#510 P3): no W206.
#[test]
fn a_group_a_solid_is_made_of_is_read() {
    let src = "\
component Plate(c: point, m: group) {
  rim := circle(center: c) hint(r: 5)
  body := solid(face(rim), depth: 2mm) of m
}
";
    assert!(of("W206", src).is_empty(), "{:?}", lint(src));
}

/// Advisory: elaboration never raises a pattern, so a document that trips every one elaborates as
/// it would without the lint; and a module the document reads is not the document's text.
#[test]
fn the_lint_is_asked_for_and_reads_only_the_documents_own_text() {
    let src = format!("{GEAR}use std (horizontal)
teeth := 12
in std.front {{
  o := point
  g := Gear(o, teeth: teeth, m: 2mm)
  f := frame
  std.Spin(f, about: o, on: std.front, ratio: -1)
}}
");
    let (p, errs) = parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&p);
    assert!(e.diags.iter().all(|d| !DRIFT.contains(&d.code)), "{:?}", e.diags);
    // `std.Spin` is itself an asserting component, but std's, not this document's
    assert_eq!(lint(&src), ["W201: teeth", "W202: std.Spin"]);
    assert!(DRIFT.iter().all(|c| c.as_str().starts_with("W2")));
}

/// Every `.sv` under `rust/examples` and `rust/lib`, by its path from `rust/`, its modules
/// resolved as `solventc` resolves them, and what the lint says of it by code.
fn corpus() -> BTreeMap<String, BTreeMap<&'static str, usize>> {
    fn sources(dir: &Path, out: &mut Vec<PathBuf>) {
        let mut entries: Vec<_> =
            std::fs::read_dir(dir).unwrap().flatten().map(|e| e.path()).collect();
        entries.sort();
        for p in entries {
            if p.is_dir() { sources(&p, out) }
            else if p.extension().is_some_and(|e| e == "sv") { out.push(p) }
        }
    }
    let rust = Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
    let mut files = Vec::new();
    sources(&rust.join("examples"), &mut files);
    sources(&rust.join("lib"), &mut files);
    let mut found = BTreeMap::new();
    for path in files {
        let name = path.strip_prefix(&rust).unwrap().to_string_lossy().to_string();
        let text = std::fs::read_to_string(&path).unwrap();
        let (mut prog, _) = gcs_core::syntax::parse(&text);
        let dir = path.parent().unwrap().to_path_buf();
        let mut resolve = |m: &str| -> Option<String> {
            gcs_core::modules::search_paths(m, &name).iter()
                .find_map(|rel| std::fs::read_to_string(dir.join(rel)).ok())
                .or_else(|| gcs_core::library::resolve(m))
        };
        let _ = gcs_core::modules::link(&mut prog, &mut resolve);
        let mut by: BTreeMap<&'static str, usize> = BTreeMap::new();
        for d in drift(&prog) {
            *by.entry(d.code.as_str()).or_default() += 1;
        }
        found.insert(name, by);
    }
    found
}

/// The patterns CI holds the corpus to, each because it is unambiguous and the language has
/// another way to say it today: what a parent reads of an instance's instance, the first one names
/// as its own (a group of aliases, `ends := {tip: arm.tip}`, or an interface), and a group nothing
/// reads is not handed on.  The others are said and not held.  A pun (W201): a defined word's
/// measure is labelled until #463.  A bare call (W202): the motion components are words since #464,
/// and `std.Rolls` and `Standoff` are not yet.  An asserting component (W203): a word's every parameter is given, so a verb that leaves
/// a number to the solve (`std.Screw`'s pitch, a joint's ratio) is a component until one may be
/// left out.  A flag read by `when` (W205): a model's choice as often as a picture's.
const GATED: [&str; 2] = ["W204", "W206"];

/// The findings standing of the gated patterns, as (file under `rust/`, code, how many).  A
/// ratchet: a new finding fails the gate, and so does one removed but still listed, so the list
/// only shrinks.  None since the corpus was brought to it: each instance read through names what
/// is read of its own (`HypoidPair`'s `gear_member`, `Tooth`'s `left` and `right`).
const ALLOWED: &[(&str, &str, usize)] = &[];

#[test]
fn the_corpus_drifts_no_further() {
    let found = corpus();
    let mut wrong = Vec::new();
    for (name, by) in &found {
        for code in GATED {
            let have = by.get(code).copied().unwrap_or(0);
            let listed = ALLOWED.iter().find(|(f, c, _)| f == name && *c == code);
            let allowed = listed.map_or(0, |a| a.2);
            if have > allowed {
                wrong.push(format!("{name}: {code} {have}, {} new", have - allowed));
            }
        }
    }
    for &(name, code, n) in ALLOWED {
        let have = found.get(name).and_then(|by| by.get(code)).copied().unwrap_or(0);
        if have < n {
            let fix = if have == 0 { "delete it from ALLOWED" } else { "lower ALLOWED" };
            wrong.push(format!("{name}: {code} {have}, listed {n}: {fix}"));
        }
    }
    let said = wrong.join("\n");
    assert!(wrong.is_empty(), "the drift lint (`solventc --lint` says where):\n{said}");
}

/// The corpus, counted: each code's findings and the files they are in, then every file with any.
/// A report, not a gate (`cargo test drift::report -- --ignored --nocapture`).
#[test]
#[ignore = "a report, not a gate"]
fn report() {
    let found = corpus();
    println!("{} files", found.len());
    for code in DRIFT.map(|c| c.as_str()) {
        let n: usize = found.values().filter_map(|by| by.get(code)).sum();
        let files = found.values().filter(|by| by.contains_key(code)).count();
        println!("{code}: {n} in {files} files");
    }
    for (name, by) in found.iter().filter(|(_, by)| !by.is_empty()) {
        let counts: Vec<String> = by.iter().map(|(c, n)| format!("{c} {n}")).collect();
        println!("{name}: {}", counts.join(", "));
    }
}
