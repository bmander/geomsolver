//! A document parsed and linked as a host does (`library::parse_linked`), a syntax or a link
//! error failing the test, and elaborated whatever the elaboration says — or solved.
use gcs_core::library;
use gcs_core::modules::link;
use gcs_core::program::{elaborate, Diag, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::{parse, Program};

/// `src` parsed and linked with no error, and elaborated.
pub fn build(src: &str) -> Elaborated {
    let (p, errors, linked) = library::parse_linked(src);
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}");
    elaborate(&p)
}

/// `src` built with no error and solved.
pub fn solved(src: &str) -> Elaborated {
    solving(build(src))
}

/// `e`, elaborated with no error, solved.
pub fn solving(mut e: Elaborated) -> Elaborated {
    assert!(e.ok(), "{:?}", e.diags);
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    e
}

/// `src` parsed with no error and linked as a host links it, `found` answering a module before
/// the library does (a host's resolver over its files): the program, and what linking said.
pub fn linked_with(src: &str, found: &dyn Fn(&str) -> Option<String>) -> (Program, Vec<Diag>) {
    let (mut prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let diags = link(&mut prog, &mut |name| found(name).or_else(|| library::resolve(name)));
    (prog, diags)
}
