use gcs_core::{program,syntax,solve};
use std::path::Path;

pub fn read(source: &str,base: &Path) -> program::Elaborated {
    let (mut p,errors) = syntax::parse(source); assert!(errors.is_empty(),"{errors:?}");
    let errors = gcs_core::modules::link(&mut p,&mut |name| {
        std::fs::read_to_string(base.join(format!("{name}.sv"))).ok()
            .or_else(|| gcs_core::library::resolve(name))
    });
    assert!(errors.is_empty(),"{errors:?}");
    let mut e = program::elaborate(&p); assert!(e.ok(),"{:?}",e.diags);
    let result = solve::solve(&mut e.sketch,solve::SolveOpts {
        tol:1e-16,acceptance_tol:1e-12,..Default::default()});
    assert!(result.success,"{result:?}"); e
}
