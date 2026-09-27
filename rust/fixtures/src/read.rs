//! Reading a fixture: parse, link, elaborate and solve, asserting each step.
use gcs_core::{program,syntax,solve};
use std::path::Path;

/// Parse one source, link it through `resolver`, elaborate and solve it to the accuracy the
/// recorded numbers were taken at.
pub fn elaborate(source: &str,resolver: &mut dyn FnMut(&str) -> Option<String>) -> program::Elaborated {
    let (mut p,errors) = syntax::parse(source); assert!(errors.is_empty(),"{errors:?}");
    let errors = gcs_core::modules::link(&mut p,resolver);
    assert!(errors.is_empty(),"{errors:?}");
    let mut e = program::elaborate(&p); assert!(e.ok(),"{:?}",e.diags);
    let result = solve::solve(&mut e.sketch,solve::SolveOpts {tol:1e-16,acceptance_tol:1e-12,..Default::default()});
    assert!(result.success,"{result:?}");
    e
}

/// A small fixture against the standard library, which is written in mm at scale 1.
pub fn read(source: &str) -> program::Elaborated {
    let e = elaborate(source,&mut |name| gcs_core::library::resolve(name));
    assert_eq!(e.sketch.units.length.map(|u| u.1),Some(1.),"fixtures are written in mm at scale 1");
    e
}

/// A resolver reading `<name>.sv` beside `base` (a dotted name in a subdirectory, as the CLI
/// reads `engine.parts` as `engine/parts.sv`), then the library, with a rewrite of each module's
/// text.
pub fn beside<'a>(base: &'a Path,rewrite: &'a mut dyn FnMut(&str,String) -> String)
    -> impl FnMut(&str) -> Option<String>+'a {
    move |name: &str| {
        std::fs::read_to_string(base.join(format!("{}.sv",name.replace('.',"/")))).ok()
            .or_else(|| gcs_core::library::resolve(name)).map(|text| rewrite(name,text))
    }
}

/// A document whose modules are read beside `base`, each rewritten.
pub fn read_beside(source: &str,base: &Path,rewrite: &mut dyn FnMut(&str,String) -> String) -> program::Elaborated {
    elaborate(source,&mut beside(base,rewrite))
}

/// The index of the named solid.
pub fn solid(e: &program::Elaborated,name: &str) -> usize { e.map.ent_named(name).unwrap().i() }
