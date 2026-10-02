//! The spiral-bevel project (`rust/examples/spiral_bevel`) at a design: the module rewrites
//! every gear suite reads it through, and the readers.
use gcs_core::program;
use std::path::PathBuf;

/// The project's directory.
pub fn project() -> PathBuf { PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel") }

/// The pair's entry document, `gears.sv`.
pub fn source() -> String { std::fs::read_to_string(project().join("gears.sv")).unwrap() }

/// One of the project's modules (a dotted name in a subdirectory), or the library's.
pub fn module(name: &str) -> Option<String> { crate::module(&project(),name) }

/// A `configuration` module of its own: the tooth counts, the mean module and the offset between
/// the shafts in millimetres, the pressure shift and the crown's spiral in degrees, the shafts
/// square, with no backlash, no tip relief and no end relief.
pub fn configuration(teeth: [u32;2],mean_module: f64,offset: f64,shift: f64,spiral: f64) -> String {
    format!("pinion_teeth := {}\ngear_teeth := {}\nmean_module := {mean_module}mm\n\
        shaft_angle := 90deg\noffset := {offset}mm\npressure_shift := {shift}deg\n\
        spiral_angle := {spiral}deg\nbacklash := 0mm\ntip_relief := 0mm\n\
        end_relief := 0mm\n",teeth[0],teeth[1])
}

/// The parameters `configure` reads in millimetres; every other is in degrees.
const LENGTHS: [&str;4] = ["offset","backlash","tip_relief","end_relief"];

/// The `configuration` module with each named parameter set to a number: the offset, the
/// backlash and the tip and end reliefs in millimetres, every other in degrees. Its line is
/// dropped wherever it stands and the value written at the end. Every other module unchanged.
pub fn configure(name: &str,text: String,params: &[(&str,f64)]) -> String {
    if name != "configuration" { return text; }
    // `offset` must not take an `offset_…`: a name ends at a space or its `:=`.
    let names = |l: &str| params.iter().any(|(p,_)| l.trim_start().strip_prefix(p)
        .is_some_and(|rest| rest.starts_with([' ',':'])));
    let unit = |p: &str| if LENGTHS.contains(&p) { "mm" } else { "deg" };
    text.lines().filter(|l| !names(l)).map(|l| format!("{l}\n")).collect::<String>()
        + &params.iter().map(|(p,v)| format!("{p} := {v}{}\n",unit(p))).collect::<String>()
}

/// The gear design: the offset between the shafts in millimetres, the pressure shift and the
/// crown's spiral angle in degrees; with no backlash and no tip or end relief, the conjugate pair
/// every number the suites recorded was taken of (`fabricated` adds them).
pub fn design(name: &str,text: String,offset: f64,shift: f64,spiral: f64) -> String {
    fabricated(name,text,[offset,shift,spiral],0.,0.,0.)
}

/// `design` with the fabrication allowances: the normal backlash, the tip relief and the end
/// relief in millimetres.
pub fn fabricated(name: &str,text: String,design: [f64;3],backlash: f64,tip_relief: f64,end_relief: f64) -> String {
    configure(name,text,&[("offset",design[0]),("pressure_shift",design[1]),("spiral_angle",design[2]),
        ("backlash",backlash),("tip_relief",tip_relief),("end_relief",end_relief)])
}

/// The bevel pair: every recorded number in the suites was taken with the pinion axis through
/// the common apex and the bevel pair's pressure angles and spiral, whatever is configured.
pub fn bevel(name: &str,text: String) -> String { design(name,text,0.,0.,35.) }

/// A small hypoid: 5.7 mm between the shafts, the axis offset of the turned layout's six-degree
/// hypoid it replaced (5.705 mm), with the bevel pair's pressure angles and spiral.
pub fn hypoid6(name: &str,text: String) -> String { design(name,text,5.7,0.,35.) }

/// The `members` module with a single tooth space per member, as a one-space export has.
pub fn one_space(text: &str) -> String {
    assert!(text.contains("repeat teeth as i {"),"the member indexes its cut by `repeat teeth as i`");
    text.replace("repeat teeth as i {","repeat 1 as i {")
}

/// The `members` module with each member also publishing its blank (`blank`: the heel bounded
/// by the tip, less the toe and the back) and `extra` after it. The member's own blank term
/// gives up its instance name to the solid.
pub fn publish_blank(text: &str,extra: &str) -> String {
    let member = "  body := solid(design.heel)\n  blank := blank.member.MemberBlank(body, design)\n";
    assert!(text.contains(member),"the member's body is the heel under its blank term");
    text.replace(member,&format!("  body := solid(design.heel)\n  \
        body_blank := blank.member.MemberBlank(body, design)\n  construction blank := solid(design.heel)\n  \
        published := blank.member.MemberBlank(blank, design)\n{extra}"))
}

/// The `design` module with each member's roll limit in degrees (`pinion_roll`, `gear_roll`).
pub fn roll(name: &str,text: String,member: &str,degrees: f64) -> String {
    if name != "design" { return text; }
    let key = format!("{member}_roll: ");
    let at = text.find(&key).unwrap_or_else(|| panic!("the design states no `{key}`"))+key.len();
    let end = at+text[at..].find([',',')']).unwrap();
    format!("{}{degrees}deg{}",&text[..at],&text[end..])
}

/// Copy the project's sources into `dir`, subdirectories and all, its configuration at a design
/// (`design`: the offset in millimetres, the shift and the spiral in degrees; no fabrication
/// allowances).
pub fn copy_design(dir: &std::path::Path,offset: f64,shift: f64,spiral: f64) {
    fn walk(from: &std::path::Path,to: &std::path::Path) {
        std::fs::create_dir_all(to).unwrap();
        for entry in std::fs::read_dir(from).unwrap() {
            let path = entry.unwrap().path();
            let target = to.join(path.file_name().unwrap());
            if path.is_dir() { walk(&path,&target); }
            else if path.extension().map_or(false,|e| e == "sv") { std::fs::copy(&path,&target).unwrap(); }
        }
    }
    walk(&project(),dir);
    let path = dir.join("configuration.sv");
    let text = std::fs::read_to_string(&path).unwrap();
    std::fs::write(&path,design("configuration",text,offset,shift,spiral)).unwrap();
}

/// The pair as configured, each module rewritten.
pub fn read_configured_with(rewrite: &mut dyn FnMut(&str,String) -> String) -> program::Elaborated {
    crate::read_beside(&source(),&project(),rewrite)
}

/// The pair as configured, offset and fabrication allowances and all.
pub fn read_as_configured() -> program::Elaborated { read_configured_with(&mut |_,text| text) }

/// `source` with its modules read beside `base` as the bevel pair, then rewritten.
pub fn read_with(source: &str,base: &std::path::Path,rewrite: &mut dyn FnMut(&str,String) -> String) -> program::Elaborated {
    crate::read_beside(source,base,&mut |name,text| rewrite(name,bevel(name,text)))
}

/// `source` with its modules read beside `base` as the bevel pair.
pub fn read(source: &str,base: &std::path::Path) -> program::Elaborated { read_with(source,base,&mut |_,text| text) }

/// The twelve designs the layout regression records (`tests/hypoid_layout.rs`), each a label and
/// its `configuration` module: the configured hypoid (with no fabrication allowances, as it was
/// recorded), the bevel pair, the six-millimetre hypoid, and the bevel pair at three tooth pairs
/// and three modules — the sizes and ratios the paired-envelope checks read.
pub fn designs() -> Vec<(String,String)> {
    let text = std::fs::read_to_string(project().join("configuration.sv")).unwrap();
    let mut all = vec![
        ("configured".to_string(),design("configuration",text.clone(),25.,12.5,25.)),
        ("bevel".to_string(),bevel("configuration",text.clone())),
        ("hypoid6".to_string(),hypoid6("configuration",text)),
    ];
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            all.push((format!("{}x{} m{module}",teeth[0],teeth[1]),configuration(teeth,module,0.,0.,35.)));
        }
    }
    all
}

/// The pair (`gears.sv`) at a `configuration` module, elaborated and not solved: where its seeds
/// put it.
pub fn unsolved(configuration: &str) -> program::Elaborated {
    crate::unsolved(&source(),&mut crate::beside(&project(),
        &mut |name,text| if name == "configuration" { configuration.to_string() } else { text }))
}
