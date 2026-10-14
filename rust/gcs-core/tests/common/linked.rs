//! A document parsed and linked as a host does (`library::parse_linked`), a syntax or a link
//! error failing the test, and elaborated whatever the elaboration says — or solved.
use gcs_core::library;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};

/// `src` parsed and linked with no error, and elaborated.
pub fn build(src: &str) -> Elaborated {
    let (p, errors, linked) = library::parse_linked(src);
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}");
    elaborate(&p)
}

/// `src` built with no error and solved.
pub fn solved(src: &str) -> Elaborated {
    let mut e = build(src);
    assert!(e.ok(), "{:?}", e.diags);
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    e
}
