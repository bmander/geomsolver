//! The core's whole suite as one test binary.  Cargo would make each file in this directory a
//! binary of its own — `autotests = false` in Cargo.toml turns that off and names this file
//! instead — and every one of those binaries linked the engine again, was assessed by macOS on
//! its first launch (a third of a second each, syspolicyd looking a fresh executable over) and
//! ran its tests after the last binary's had finished.  As one crate the engine links once,
//! the launch is paid once, and `diagnose`'s eight seconds overlap `program`'s seven instead of
//! following them.  Nothing moved: a file is the module of the same name, `tests/chain.rs` is
//! still the gate it was, and a test is filtered as `cargo test chain::` where it was
//! `--test chain`.  A file added here must be listed here too, or it is not run —
//! `every_file_is_a_module` fails the suite when one is forgotten.

mod common;

mod anonymous;
mod callout;
mod chain;
mod claim;
mod components;
mod computed_point;
mod copies;
mod curve_contact;
mod curve_of;
mod curve;
mod curvedef;
mod decompose;
mod describe;
mod diagnose;
mod drag;
mod edit;
mod ellipse;
mod examples_sv;
mod expr;
mod frame;
mod gauges;
mod highlight;
mod homotopy;
mod io;
mod jacobians;
mod jansen;
mod linalg;
mod modules;
mod names;
mod open_joint;
mod order;
mod overview;
mod pick;
mod plane_lang;
mod plane;
mod program;
mod refusals;
mod ring;
mod row_scale;
mod seeds;
mod smoke;
mod solid;
mod style;
mod tape;
mod trace;
mod units;
mod unseeded;
mod witness;

/// Every `tests/*.rs` beside this file is declared above: with `autotests = false` a file
/// nobody lists is a test nobody runs, silently.
#[test]
fn every_file_is_a_module() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests");
    let listed = include_str!("main.rs");
    let mut missing = Vec::new();
    for entry in std::fs::read_dir(&dir).unwrap() {
        let name = entry.unwrap().file_name().into_string().unwrap();
        let Some(stem) = name.strip_suffix(".rs") else { continue };
        if stem != "main" && !listed.contains(&format!("\nmod {stem};\n")) {
            missing.push(name);
        }
    }
    assert!(missing.is_empty(), "tests/main.rs does not declare {missing:?}");
}
