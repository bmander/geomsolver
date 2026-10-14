//! A document read whatever it says: elaborated even where it does not parse, with every
//! diagnostic beside it for a test to look through — what the refusal and the variational tests
//! read a document by.
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::{Program, SynErr};

/// `src` parsed (`common::parse`) and elaborated, and every diagnostic: a syntax error as
/// `syntax: …`, the rest as `CODE: message`.
pub fn read(src: &str) -> (Elaborated, Vec<String>) {
    read_with(src, super::parse)
}

/// `read`, parsed by `common::parse_legacy`.
pub fn read_legacy(src: &str) -> (Elaborated, Vec<String>) {
    read_with(src, super::parse_legacy)
}

fn read_with(src: &str, parse: fn(&str) -> (Program, Vec<SynErr>)) -> (Elaborated, Vec<String>) {
    let (prog, errs) = parse(src);
    let e = elaborate(&prog);
    let mut all: Vec<String> = errs.iter().map(|x| format!("syntax: {}", x.message)).collect();
    all.extend(e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)));
    (e, all)
}

/// `src` read with no diagnostic and solved, or the test fails saying why.
pub fn solved(src: &str) -> Elaborated {
    let (mut e, d) = read(src);
    assert!(e.ok(), "{d:?}");
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}: {src}", r.message);
    e
}
