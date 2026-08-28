//! `solventc` end to end: what a document's exit code says, and what the report reads like.
//!
//! The binary is the point of the tests — a `Diag` carrying a code and a span has existed since
//! the app's banner, and this is the first consumer of it that a CI job can run.

use std::path::PathBuf;
use std::process::{Command, Output};

fn examples() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../examples")
}

fn run(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_solventc")).args(args).output().expect("solventc runs")
}

fn doc(name: &str) -> String {
    examples().join(name).to_string_lossy().into_owned()
}

/// **The library, checked from a terminal.**  Every document reports; the three deliberately
/// unsatisfiable ones are the only nonzero exits, and `--allow-unsolved` makes those zero too.
/// The under-constrained cases exit 0: they solve, they just have freedoms left.
#[test]
fn the_whole_library_reports() {
    let all: Vec<String> = std::fs::read_dir(examples())
        .expect("the example documents")
        .filter_map(|e| e.ok())
        .map(|e| e.path().to_string_lossy().into_owned())
        .filter(|p| p.ends_with(".sv"))
        .collect();
    let args: Vec<&str> = all.iter().map(String::as_str).collect();
    let out = run(&args);
    let text = String::from_utf8_lossy(&out.stdout);
    assert_eq!(out.status.code(), Some(2), "the unsatisfiable ones are a failure");
    assert_eq!(text.lines().filter(|l| l.contains(": solved")).count(), all.len() - 3);

    let mut ok = args.clone();
    ok.push("--allow-unsolved");
    assert_eq!(run(&ok).status.code(), Some(0), "and told not to be, they are not");

    for under in ["rect_fillets_under.sv", "truss_floating.sv"] {
        assert_eq!(run(&[&doc(under)]).status.code(), Some(0), "{under}: solved, with freedoms");
    }
    for bad in ["impossible_triangle.sv", "truss_conflict.sv", "rect_fillets_conflict.sv"] {
        assert_eq!(run(&[&doc(bad)]).status.code(), Some(2), "{bad}");
    }
}

/// A document that does not elaborate exits 1, and says where — `file:line:col`, with the column
/// counting **characters**.  Offsets cross from the core in UTF-8 bytes, and a document with a
/// non-ASCII character before the offending token is the ordinary case (`gear.sv` has an em dash
/// in its second line), not a corner one.
#[test]
fn a_broken_document_says_where() {
    let dir = std::env::temp_dir().join("solventc-test");
    std::fs::create_dir_all(&dir).expect("a place to write");
    let path = dir.join("uni.sv");
    std::fs::write(&path, "point é hint(x: 0, y: 0)   // — an em dash\nline l(é, zzz)\n")
        .expect("write");
    let out = run(&[&path.to_string_lossy()]);
    let err = String::from_utf8_lossy(&out.stderr);
    assert_eq!(out.status.code(), Some(1));
    assert!(err.contains("uni.sv:2:11: error[E101]:"), "{err}");
    // `é` is two bytes: a byte column would say 12
    assert!(!err.contains(":2:12:"), "the column counts characters, not bytes: {err}");
    // and one finding is said once
    assert_eq!(err.matches("no such entity").count(), 1, "{err}");
}

/// `--json` parses, and carries the same numbers the text report does.
#[test]
fn the_json_report_carries_the_same_numbers() {
    let out = run(&["--json", &doc("rect_fillets.sv")]);
    assert_eq!(out.status.code(), Some(0));
    let v = gcs_core::json::parse(&String::from_utf8_lossy(&out.stdout)).expect("valid JSON");
    let docs = v.get("documents").expect("documents").arr().to_vec();
    assert_eq!(docs.len(), 1);
    let d = &docs[0];
    assert!(d.get("name").expect("name").as_str().ends_with("rect_fillets.sv"));
    assert!(d.get("solve").and_then(|s| s.get("success")).expect("success").as_bool());
    let dg = d.get("diagnosis").expect("diagnosis");
    assert_eq!(dg.get("dof").expect("dof").as_i64(), 0);
    // the text report says the same, in the core's own words
    let text = String::from_utf8_lossy(&run(&[&doc("rect_fillets.sv")]).stdout).into_owned();
    assert!(text.contains("DOF 0"), "{text}");
}

/// `--no-diagnose` solves and stops there.
#[test]
fn no_diagnose_solves_only() {
    let text = String::from_utf8_lossy(&run(&["--no-diagnose", &doc("rect_fillets.sv")]).stdout)
        .into_owned();
    assert!(text.contains(": solved"));
    assert!(!text.contains("DOF"), "{text}");
}

/// `--output` writes an SVG, and the writer is the **core's** — an "export SVG" button in the
/// web app must not be a second implementation.
#[test]
fn output_writes_an_svg() {
    let dir = std::env::temp_dir().join("solventc-test");
    std::fs::create_dir_all(&dir).expect("a place to write");
    let out = dir.join("rect.svg");
    let r = run(&["--output", &out.to_string_lossy(), &doc("rect_fillets.sv")]);
    assert_eq!(r.status.code(), Some(0));
    let svg = std::fs::read_to_string(&out).expect("an SVG");
    assert!(svg.starts_with("<svg xmlns=\"http://www.w3.org/2000/svg\""), "{}", &svg[..80]);
    assert!(svg.ends_with("</svg>\n"));
    assert!(!svg.contains("NaN") && !svg.contains("inf"), "every number is a number");
    // the four fillets, as arcs; the four sides, as lines; the two dimensions, as text
    assert_eq!(svg.matches("<path d=\"M").count(), 4);
    assert_eq!(svg.matches("<line ").count(), 4);
    assert!(svg.contains(">100</text>") && svg.contains(">60</text>"), "the numbers are drawn");

    // one file, so one document
    let two = run(&["--output", &out.to_string_lossy(), &doc("rect_fillets.sv"), &doc("truss.sv")]);
    assert_eq!(two.status.code(), Some(2));
}
