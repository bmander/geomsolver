//! The spiral-bevel project (`rust/examples/spiral_bevel`) at a design: the module rewrites
//! every gear suite reads it through, and the readers.
use gcs_core::program;
use std::path::PathBuf;

/// The project's directory.
pub fn project() -> PathBuf { PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel") }

/// The pair's entry document, `gears.sv`.
pub fn source() -> String { std::fs::read_to_string(project().join("gears.sv")).unwrap() }

/// The `configuration` module with each named parameter set to a number of degrees: its line
/// dropped wherever it stands and the value written at the end.  Every other module unchanged.
pub fn configure(name: &str,text: String,params: &[(&str,f64)]) -> String {
    if name != "configuration" { return text; }
    let lines: Vec<String> = params.iter().map(|(p,_)| format!("param {p}")).collect();
    // `param offset` must not take `param offset_angle`: a name ends at a space or an `=`.
    let names = |l: &str| lines.iter().any(|p| l.trim_start().strip_prefix(p.as_str())
        .is_some_and(|rest| rest.starts_with([' ','='])));
    text.lines().filter(|l| !names(l)).map(|l| format!("{l}\n")).collect::<String>()
        + &params.iter().map(|(p,v)| format!("param {p} = {v}deg\n")).collect::<String>()
}

/// The gear design: the offset angle, the pressure shift and the crown's spiral angle, in degrees.
pub fn design(name: &str,text: String,offset: f64,shift: f64,spiral: f64) -> String {
    configure(name,text,&[("offset_angle",offset),("pressure_shift",shift),("spiral_angle",spiral)])
}

/// The bevel pair: every recorded number in the suites was taken with the pinion axis through
/// the common apex and the bevel pair's pressure angles and spiral, whatever is configured.
pub fn bevel(name: &str,text: String) -> String { design(name,text,0.,0.,35.) }

/// The hypoid the mesh-path records were taken at: 6 degrees, with the bevel pair's pressure
/// angles and spiral.
pub fn hypoid6(name: &str,text: String) -> String { design(name,text,6.,0.,35.) }

/// The `matched_pair` module with a single tooth space per member, as a one-space export has.
pub fn one_space(text: &str) -> String { text.replace("repeat teeth as i {","repeat 1 as i {") }

/// The `matched_pair` module with each member also publishing its blank (`blank`: the heel
/// bounded by the tip, less the toe and the back) and `extra` after it.
pub fn publish_blank(text: &str,extra: &str) -> String {
    text.replace("  solid body(design.heel)\n",&format!("  solid body(design.heel)\n  construction solid blank(design.heel)\n  \
        design.tip bound blank\n  design.toe cut blank\n  design.back cut blank\n{extra}"))
}

/// The pair as configured, each module rewritten.
pub fn read_configured_with(rewrite: &mut dyn FnMut(&str,String) -> String) -> program::Elaborated {
    crate::read_beside(&source(),&project(),rewrite)
}

/// The pair as configured, offset and all.
pub fn read_as_configured() -> program::Elaborated { read_configured_with(&mut |_,text| text) }

/// `source` with its modules read beside `base` as the bevel pair, then rewritten.
pub fn read_with(source: &str,base: &std::path::Path,rewrite: &mut dyn FnMut(&str,String) -> String) -> program::Elaborated {
    crate::read_beside(source,base,&mut |name,text| rewrite(name,bevel(name,text)))
}

/// `source` with its modules read beside `base` as the bevel pair.
pub fn read(source: &str,base: &std::path::Path) -> program::Elaborated { read_with(source,base,&mut |_,text| text) }
