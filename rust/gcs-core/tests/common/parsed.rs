//! A document parsed by `common::parse` — linked against the library, a link diagnostic left to
//! the elaboration rather than asserted, as against `common::read` — and elaborated: built, read
//! with no error, solved, and solved with no freedom left.
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::Program;

use super::parse;

/// `src` parsed, or the test fails with every syntax error.
pub fn prog_of(src: &str) -> Program {
    let (p, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}\n{src}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    p
}

/// `src` parsed with no syntax error and elaborated, whatever the elaboration says.
pub fn build(src: &str) -> Elaborated {
    let (p, errors) = parse(src);
    assert!(errors.is_empty(), "{errors:?}");
    elaborate(&p)
}

/// `src` parsed and elaborated with no error, or the test fails with every diagnostic.
pub fn read(src: &str) -> Elaborated {
    read_but(src, |_| false)
}

/// `read`, past the promise every definition is held to (`common::promise`): for a test of another
/// rule whose bodies only relate what they are given, on purpose.
pub fn read_past_promise(src: &str) -> Elaborated {
    read_but(src, super::promise)
}

/// `src` parsed and elaborated with no error but those `pass` lets through.
fn read_but(src: &str, pass: fn(&gcs_core::program::Diag) -> bool) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let e = elaborate(&prog);
    let said: Vec<&String> = e.errors().filter(|d| !pass(d)).map(|d| &d.message).collect();
    assert!(said.is_empty(), "does not elaborate: {said:?}\n{src}");
    e
}

/// `src` built with no error and solved.
pub fn solved(src: &str) -> Elaborated {
    let mut e = build(src);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    e
}

/// `src` solved, with no freedom left.
pub fn determined(src: &str) -> Elaborated {
    let mut e = solved(src);
    assert_eq!(diagnose(&mut e.sketch, DiagnoseOptions::default()).dof, 0);
    e
}

