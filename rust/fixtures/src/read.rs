//! Reading a fixture: parse, link, elaborate and solve, asserting each step — and reading the
//! example corpus as solventc reads it.
use gcs_core::{program,syntax,solve};
use std::path::{Path,PathBuf};

/// The accuracy the recorded numbers were taken at: every hard row to 1e-12 of its units.
pub fn accurate() -> solve::SolveOpts {
    solve::SolveOpts {tol:1e-16,acceptance_tol:1e-12,..Default::default()}
}

/// Parse one source, link it through `resolver`, elaborate and solve it `accurate`ly.
pub fn elaborate(source: &str,resolver: &mut dyn FnMut(&str) -> Option<String>) -> program::Elaborated {
    let mut e = unsolved(source,resolver);
    let result = solve::solve(&mut e.sketch,accurate());
    assert!(result.success,"{result:?}");
    e
}

/// Parse one source, link it through `resolver` and elaborate it, left where its seeds put it.
pub fn unsolved(source: &str,resolver: &mut dyn FnMut(&str) -> Option<String>) -> program::Elaborated {
    let (mut p,errors) = syntax::parse(source);
    // each error beside the line it is on: a generated fixture has no file to look in
    let lines = errors.iter().map(|e| line_at(source,e.span.lo as usize)).collect::<Vec<_>>();
    assert!(errors.is_empty(),"{errors:?}\n{}",lines.join("\n"));
    let errors = gcs_core::modules::link(&mut p,resolver);
    assert!(errors.is_empty(),"{errors:?}");
    let e = program::elaborate(&p); assert!(e.ok(),"{:?}",e.diags);
    e
}

/// The whole line of `source` an offset stands on.
fn line_at(source: &str,at: usize) -> &str {
    let lo = source[..at].rfind('\n').map_or(0,|k| k+1);
    let hi = source[at..].find('\n').map_or(source.len(),|k| at+k);
    &source[lo..hi]
}

/// A small fixture against the standard library, which is written in mm at scale 1.
pub fn read(source: &str) -> program::Elaborated {
    let e = elaborate(source,&mut |name| gcs_core::library::resolve(name));
    assert_eq!(e.sketch.units.length.map(|u| u.1),Some(1.),"fixtures are written in mm at scale 1");
    e
}

/// The module `name` as the CLI resolves it: `<name>.sv` beside `base` (a dotted name in a
/// subdirectory, `engine.parts` as `engine/parts.sv`), then the library.
pub fn module(base: &Path,name: &str) -> Option<String> {
    std::fs::read_to_string(base.join(format!("{}.sv",name.replace('.',"/")))).ok()
        .or_else(|| gcs_core::library::resolve(name))
}

/// A resolver reading each `module` beside `base`, with a rewrite of each module's text.
pub fn beside<'a>(base: &'a Path,rewrite: &'a mut dyn FnMut(&str,String) -> String)
    -> impl FnMut(&str) -> Option<String>+'a {
    move |name: &str| module(base,name).map(|text| rewrite(name,text))
}

/// A document whose modules are read beside `base`, each rewritten.
pub fn read_beside(source: &str,base: &Path,rewrite: &mut dyn FnMut(&str,String) -> String) -> program::Elaborated {
    elaborate(source,&mut beside(base,rewrite))
}

/// The index of the named solid.
pub fn solid(e: &program::Elaborated,name: &str) -> usize { e.map.ent_named(name).unwrap().i() }

/// Every `.sv` in `rust/examples`, by its path there, elaborated and not solved, its modules
/// read as solventc reads them (beside the document, then in its ancestors up to the examples,
/// then the library) — `None` for one that does not parse.  Nothing is asserted: a document
/// that does not elaborate is returned with its diagnostics.
pub fn examples() -> Vec<(String,Option<program::Elaborated>)> {
    fn sources(dir: &Path,out: &mut Vec<PathBuf>) {
        let mut entries: Vec<_> = std::fs::read_dir(dir).unwrap().flatten().map(|e| e.path()).collect();
        entries.sort();
        for p in entries {
            if p.is_dir() { sources(&p,out) }
            else if p.extension().is_some_and(|e| e == "sv") { out.push(p) }
        }
    }
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let mut files = Vec::new();
    sources(&root,&mut files);
    files.into_iter().map(|path| {
        let name = path.strip_prefix(&root).unwrap().to_string_lossy().to_string();
        let text = std::fs::read_to_string(&path).unwrap();
        let (mut prog,errs) = syntax::parse(&text);
        if !errs.is_empty() { return (name,None); }
        let dir = path.parent().unwrap().to_path_buf();
        let mut resolve = |m: &str| -> Option<String> {
            gcs_core::modules::search_paths(m,&name).iter()
                .find_map(|rel| std::fs::read_to_string(dir.join(rel)).ok())
                .or_else(|| gcs_core::library::resolve(m))
        };
        let _ = gcs_core::modules::link(&mut prog,&mut resolve);
        let e = program::elaborate(&prog);
        (name,Some(e))
    }).collect()
}
