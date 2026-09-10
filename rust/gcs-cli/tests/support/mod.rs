use gcs_core::{program,syntax,solve};
use std::path::Path;

/// The bevel pair: every recorded number in these suites was taken with the
/// pinion axis through the common apex, so the configured axis offset reads as
/// zero here.
pub fn read(source: &str,base: &Path) -> program::Elaborated {
    read_with(source,base,&mut |_,text| text)
}

/// The pair as configured, offset and all.
#[allow(dead_code)]
pub fn read_as_configured(source: &str,base: &Path) -> program::Elaborated {
    let (mut p,errors) = syntax::parse(source); assert!(errors.is_empty(),"{errors:?}");
    let errors = gcs_core::modules::link(&mut p,&mut |name| {
        std::fs::read_to_string(base.join(format!("{name}.sv"))).ok().or_else(|| gcs_core::library::resolve(name))
    });
    assert!(errors.is_empty(),"{errors:?}");
    let mut e = program::elaborate(&p); assert!(e.ok(),"{:?}",e.diags);
    let result = solve::solve(&mut e.sketch,solve::SolveOpts {tol:1e-16,acceptance_tol:1e-12,..Default::default()});
    assert!(result.success,"{result:?}"); e
}

/// The module rewrite that zeroes the configured axis offset.
pub fn bevel(name: &str,text: String) -> String {
    if name != "configuration" { return text; }
    text.lines().filter(|l| !l.trim_start().starts_with("param axis_offset")).map(|l| format!("{l}\n")).collect::<String>()
        + "param axis_offset = 0mm\n"
}

/// Read with a module text rewrite, for tests recording evidence about a
/// declaration the project has since changed (the gear's 35-degree roll).
pub fn read_with(source: &str,base: &Path,rewrite: &mut dyn FnMut(&str,String) -> String) -> program::Elaborated {
    let (mut p,errors) = syntax::parse(source); assert!(errors.is_empty(),"{errors:?}");
    let errors = gcs_core::modules::link(&mut p,&mut |name| {
        std::fs::read_to_string(base.join(format!("{name}.sv"))).ok()
            .or_else(|| gcs_core::library::resolve(name)).map(|text| rewrite(name,bevel(name,text)))
    });
    assert!(errors.is_empty(),"{errors:?}");
    let mut e = program::elaborate(&p); assert!(e.ok(),"{:?}",e.diags);
    let result = solve::solve(&mut e.sketch,solve::SolveOpts {
        tol:1e-16,acceptance_tol:1e-12,..Default::default()});
    assert!(result.success,"{result:?}"); e
}
