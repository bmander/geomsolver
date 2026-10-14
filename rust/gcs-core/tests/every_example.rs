//! **Every example elaborates**: each `.sv` under `rust/examples/` parsed, linked as `solventc`
//! links it (a module beside the document, then beside each of its ancestors up to the examples'
//! root, then the library: `modules::search_paths`) and elaborated with no error — and each
//! library module (`rust/lib/`) on its own.  `common::corpus` and `fixtures::examples` skip a
//! document that does not elaborate and the tools reading them are ignored, so without this gate
//! an example could stop elaborating unseen; one referenced by no other test (`parametric_torus`,
//! `solid_bend`, `solid_twist`, `swept_tumble`) is held here alone.
//!
//! A document that does not elaborate today is listed in `KNOWN`, with what it says, so the gate
//! passes and catches any other: one that starts failing fails here, and one on the list that
//! elaborates again fails here too until it is taken off.
use std::path::{Path, PathBuf};

use gcs_core::{library, modules, par, program, syntax};

/// What does not elaborate today, by path (under `rust/examples/`, or `lib/` for the library),
/// each with a diagnostic it gives.  All three are dimension tables, modules written to be read
/// through a `use` from a document naming its `unit`: on their own they name none, so every
/// `80mm` among them is E103 (and each group member over one names nothing).  They elaborate
/// where they are used, which this gate checks with the documents using them.
const KNOWN: &[(&str, &str)] = &[
    ("engine/dims.sv", "E103: `D`: this document names no unit"),
    ("vtwin/components/dims.sv", "this document names no unit"),
    ("lib/hardware.sv", "E103: `hexbolt14_d`: this document names no unit"),
];

/// Every `.sv` under `dir`, in order.
fn sources(dir: &Path, out: &mut Vec<PathBuf>) {
    let mut entries: Vec<PathBuf> =
        std::fs::read_dir(dir).unwrap().flatten().map(|e| e.path()).collect();
    entries.sort();
    for p in entries {
        if p.is_dir() {
            sources(&p, out);
        } else if p.extension().is_some_and(|e| e == "sv") {
            out.push(p);
        }
    }
}

/// Every error `text` gives, parsed, linked through `resolve` and elaborated: the syntax's, the
/// linker's and the elaborator's, each as `CODE: message`.
fn errors(text: &str, resolve: &mut dyn FnMut(&str) -> Option<String>) -> Vec<String> {
    let (mut prog, errs) = syntax::parse(text);
    if !errs.is_empty() {
        return errs.iter().map(|e| format!("E100: {}", e.message)).collect();
    }
    let linked = modules::link(&mut prog, resolve);
    let e = program::elaborate(&prog);
    linked.iter().chain(e.errors())
        .filter(|d| d.severity() == program::Severity::Error)
        .map(|d| format!("{}: {}", d.code.as_str(), d.message))
        .collect()
}

/// Each document under `rust/examples/` and each library module that does not elaborate, by its
/// path, with its errors: read on every core (`par::map`), a few documents taking seconds each.
fn failing() -> Vec<(String, Vec<String>)> {
    let rust = Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
    let root = rust.join("examples");
    let mut files = Vec::new();
    sources(&root, &mut files);
    assert!(files.len() > 100, "only {} examples under {}", files.len(), root.display());
    let mut libs = Vec::new();
    sources(&rust.join("lib"), &mut libs);
    assert!(!libs.is_empty(), "no library module under {}", rust.join("lib").display());
    // (its name, its path, whether it is a library module)
    let jobs: Vec<(String, PathBuf, bool)> = files.into_iter()
        .map(|p| (p.strip_prefix(&root).unwrap().to_string_lossy().replace('\\', "/"), p, false))
        .chain(libs.into_iter()
            .map(|p| (format!("lib/{}", p.file_name().unwrap().to_string_lossy()), p, true)))
        .collect();
    let read = par::map(&jobs, |(name, path, in_lib)| {
        let text = std::fs::read_to_string(path).unwrap();
        let dir = path.parent().unwrap().to_path_buf();
        let mut beside = |m: &str| -> Option<String> {
            modules::search_paths(m, name).iter()
                .find_map(|rel| std::fs::read_to_string(dir.join(rel)).ok())
                .or_else(|| library::resolve(m))
        };
        let errs = if *in_lib {
            errors(&text, &mut library::resolve)
        } else {
            errors(&text, &mut beside)
        };
        (name.clone(), errs)
    });
    read.into_iter().filter(|(_, errs)| !errs.is_empty()).collect()
}

#[test]
fn every_example_and_library_module_elaborates() {
    let failing = failing();
    let fresh: Vec<_> =
        failing.iter().filter(|(n, _)| !KNOWN.iter().any(|(k, _)| k == n)).collect();
    let mended: Vec<_> =
        KNOWN.iter().filter(|(k, _)| !failing.iter().any(|(n, _)| n == k)).collect();
    let otherwise: Vec<_> = failing.iter()
        .filter(|(n, errs)| KNOWN.iter()
            .any(|(k, says)| k == n && !errs.iter().any(|e| e.contains(says))))
        .collect();
    assert!(fresh.is_empty(), "examples that no longer elaborate: {fresh:#?}");
    assert!(mended.is_empty(), "examples that elaborate again, to take off `KNOWN`: {mended:#?}");
    assert!(otherwise.is_empty(), "examples in `KNOWN` failing otherwise: {otherwise:#?}");
}
