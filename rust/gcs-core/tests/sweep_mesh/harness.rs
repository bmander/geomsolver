//! Reading a fixture and the small geometry every check shares.
use gcs_core::{program,syntax,solve};
use std::path::Path;

pub type V3 = [f64;3];

/// Parse, link against the standard library, elaborate and solve one source.
pub fn read(source: &str) -> program::Elaborated {
    read_resolving(source,&mut |name| gcs_core::library::resolve(name))
}

/// The same, resolving modules through the caller first (a directory example).
pub fn read_resolving(source: &str,resolver: &mut dyn FnMut(&str) -> Option<String>) -> program::Elaborated {
    let (mut p,errors) = syntax::parse(source); assert!(errors.is_empty(),"{errors:?}");
    let errors = gcs_core::modules::link(&mut p,resolver);
    assert!(errors.is_empty(),"{errors:?}");
    let mut e = program::elaborate(&p); assert!(e.ok(),"{:?}",e.diags);
    let result = solve::solve(&mut e.sketch,solve::SolveOpts {tol:1e-16,acceptance_tol:1e-12,..Default::default()});
    assert!(result.success,"{result:?}");
    assert_eq!(e.sketch.units.length.map(|u| u.1),Some(1.),"fixtures are written in mm at scale 1");
    e
}

/// A resolver reading `<name>.sv` beside `base`, then the library, with a
/// rewrite of each module's text.
pub fn directory_resolver<'a>(base: &'a Path,rewrite: &'a mut dyn FnMut(&str,String) -> String)
    -> impl FnMut(&str) -> Option<String>+'a {
    move |name: &str| {
        std::fs::read_to_string(base.join(format!("{name}.sv"))).ok()
            .or_else(|| gcs_core::library::resolve(name)).map(|text| rewrite(name,text))
    }
}

pub fn distance(a: V3,b: V3) -> f64 { (0..3).map(|k| (a[k]-b[k]).powi(2)).sum::<f64>().sqrt() }

/// The index of the named solid.
pub fn solid(e: &program::Elaborated,name: &str) -> usize { e.map.ent_named(name).unwrap().i() }
