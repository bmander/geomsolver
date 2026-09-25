//! Readers shared by the CLI's suites; each suite uses some of them.
#![allow(dead_code)]
use gcs_core::{program,syntax,solve};
use std::path::Path;

/// The bevel pair: every recorded number in these suites was taken with the
/// pinion axis through the common apex, so the configured offset angle reads as
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

/// The module rewrite that zeroes the configured offset angle and restores the
/// bevel pair's pressure angles and spiral, whatever the configured design is.
pub fn bevel(name: &str,text: String) -> String { design(name,text,0.,0.,35.) }

/// The module rewrite that sets the gear design: the offset angle, the pressure
/// shift and the crown's spiral angle, in degrees.
pub fn design(name: &str,text: String,offset: f64,shift: f64,spiral: f64) -> String {
    if name != "configuration" { return text; }
    let set = ["param offset_angle","param pressure_shift","param spiral_angle"];
    text.lines().filter(|l| !set.iter().any(|s| l.trim_start().starts_with(s))).map(|l| format!("{l}\n")).collect::<String>()
        + &format!("param offset_angle = {offset}deg\nparam pressure_shift = {shift}deg\nparam spiral_angle = {spiral}deg\n")
}

/// The hypoid the mesh-path records were taken at: 6 degrees, with the bevel
/// pair's pressure angles and spiral.
#[allow(dead_code)]
pub fn hypoid6(name: &str,text: String) -> String { design(name,text,6.,0.,35.) }

/// The pair as configured, with a module text rewrite.
#[allow(dead_code)]
pub fn read_configured_with(source: &str,base: &Path,rewrite: &mut dyn FnMut(&str,String) -> String) -> program::Elaborated {
    let (mut p,errors) = syntax::parse(source); assert!(errors.is_empty(),"{errors:?}");
    let errors = gcs_core::modules::link(&mut p,&mut |name| {
        std::fs::read_to_string(base.join(format!("{name}.sv"))).ok()
            .or_else(|| gcs_core::library::resolve(name)).map(|text| rewrite(name,text))
    });
    assert!(errors.is_empty(),"{errors:?}");
    let mut e = program::elaborate(&p); assert!(e.ok(),"{:?}",e.diags);
    let result = solve::solve(&mut e.sketch,solve::SolveOpts {tol:1e-16,acceptance_tol:1e-12,..Default::default()});
    assert!(result.success,"{result:?}"); e
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
