//! The spiral-bevel pair (`rust/examples/spiral_bevel/hypoid.sv`, one file) at a design: the
//! rewrites every gear suite reads it through, and the readers. A design is the document's own
//! inputs given numbers (`param pinion_teeth := 24`, …), each rewritten where the parser says
//! its value stands (`configure`); nothing else of the text is touched but the member's tooth
//! count for a one-space build (`one_space`) and a published blank (`publish_blank`).
use gcs_core::{program,syntax};
use std::path::PathBuf;

/// The project's directory.
pub fn project() -> PathBuf { PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel") }

/// The pair's document, `hypoid.sv`.
pub fn source() -> String { std::fs::read_to_string(project().join("hypoid.sv")).unwrap() }

/// `text` with each named input (`param NAME := …` at the top level) given a number, in the
/// unit its literal is written in there (`2mm`, `25deg`, a count bare). Asserts each is an input.
pub fn configure(text: &str,params: &[(&str,f64)]) -> String {
    let (prog,_) = syntax::parse(text);
    let mut values: Vec<(syntax::Span,String)> = params.iter().map(|&(name,v)| {
        let span = prog.root().body.iter().find_map(|st| match &st.kind {
            syntax::StmtKind::Param(p) if p.input.is_some() && p.bound() && p.name.text == name => Some(p.span),
            _ => None,
        }).unwrap_or_else(|| panic!("the pair has no input `{name}`"));
        let written = &text[span.lo as usize..span.hi as usize];
        let unit = written.trim_start_matches(|c: char| !c.is_ascii_alphabetic());
        (span,format!("{v}{unit}"))
    }).collect();
    values.sort_by_key(|(at,_)| std::cmp::Reverse(at.lo));
    let mut out = text.to_string();
    for (at,v) in values {
        out.replace_range(at.lo as usize..at.hi as usize,&v);
    }
    out
}

/// A design's inputs: the tooth counts, the mean module and the offset between the shafts in
/// millimetres, the pressure shift and the crown's spiral in degrees, the shafts square, with no
/// backlash, no tip relief and no end relief.
pub fn configuration(teeth: [u32;2],mean_module: f64,offset: f64,shift: f64,spiral: f64) -> Vec<(&'static str,f64)> {
    vec![("pinion_teeth",teeth[0].into()),("gear_teeth",teeth[1].into()),("mean_module",mean_module),
        ("shaft_angle",90.),("offset",offset),("pressure_shift",shift),("spiral_angle",spiral),
        ("backlash",0.),("tip_relief",0.),("end_relief",0.)]
}

/// The pair at a design of its own (`configuration`).
pub fn at(teeth: [u32;2],mean_module: f64,offset: f64,shift: f64,spiral: f64) -> String {
    configure(&source(),&configuration(teeth,mean_module,offset,shift,spiral))
}

/// `text` at the gear design: the offset between the shafts in millimetres, the pressure shift and
/// the crown's spiral angle in degrees; with no backlash and no tip or end relief, the conjugate
/// pair every number the suites recorded was taken of (`fabricated` adds them).
pub fn design(text: &str,offset: f64,shift: f64,spiral: f64) -> String {
    fabricated(text,[offset,shift,spiral],0.,0.,0.)
}

/// `design` with the fabrication allowances: the normal backlash, the tip relief and the end
/// relief in millimetres.
pub fn fabricated(text: &str,design: [f64;3],backlash: f64,tip_relief: f64,end_relief: f64) -> String {
    configure(text,&[("offset",design[0]),("pressure_shift",design[1]),("spiral_angle",design[2]),
        ("backlash",backlash),("tip_relief",tip_relief),("end_relief",end_relief)])
}

/// The bevel pair: every recorded number in the suites was taken with the pinion axis through
/// the common apex and the bevel pair's pressure angles and spiral, whatever is configured.
pub fn bevel(text: &str) -> String { design(text,0.,0.,35.) }

/// A small hypoid: 5.7 mm between the shafts, the axis offset of the turned layout's six-degree
/// hypoid it replaced (5.705 mm), with the bevel pair's pressure angles and spiral.
pub fn hypoid6(text: &str) -> String { design(text,5.7,0.,35.) }

/// The pair with a single tooth space per member, as a one-space export has: `Generated`'s
/// indexing at its first tooth alone.
pub fn one_space(text: &str) -> String {
    assert!(text.contains("repeat n as i {"),"a cut is indexed at every tooth by `repeat n as i`");
    text.replace("repeat n as i {","repeat 1 as i {")
}

/// The pair with each member also publishing its blank (`blank`: a body over the blank's region)
/// and `extra` after it, past the member's generated `cut`.
pub fn publish_blank(text: &str,extra: &str) -> String {
    let cut = "  body := solid(blank)\n  cut := Generated(";
    let at = text.find(cut).expect("the member's body is its blank, less its generated cut")+cut.len();
    let end = at+text[at..].find('\n').unwrap()+1;
    format!("{}  construction blank := solid(blank)\n{extra}{}",&text[..end],&text[end..])
}

/// A member's axis (`pinion`, `gear`): each is its pitch cone's, under the pair
/// (`pair.gear_cone.ax`, `pair.pinion_cone.ax`).
pub fn axis(member: &str) -> String { format!("pair.{member}_cone.ax") }

/// `text` with a member's cut (`pinion`, `gear`; not its tip relief's) rolled from `-degrees` to
/// `degrees`, stated, in place of the roll its tool takes through its blank (`through:`, #271):
/// the negative controls, a roll too short or too long. The member is made by copies of `Member`
/// and `Generated` as `text` has them, taking the roll.
pub fn roll(text: &str,member: &str,degrees: f64) -> String {
    let component = |name: &str| -> String {
        let head = format!("\ncomponent {name}(");
        let at = text.find(&head).unwrap_or_else(|| panic!("the pair has a component `{name}`"))+1;
        let end = at+text[at..].find("\n}\n").expect("a component ends at a closing brace")+3;
        text[at..end].to_string()
    };
    let formal = "n: Int) {";
    let member_copy = component("Member").replacen("component Member(","component RolledMember(",1)
        .replacen(formal,"n: Int, roll: Angle) {",1).replacen("Generated(","RolledGenerated(",1)
        .replacen("n: n)","n: n, roll: roll)",1);
    let generated = component("Generated");
    assert!(generated.contains("through: body)"),"the cut rolls through its body");
    let generated_copy = generated.replacen("component Generated(","component RolledGenerated(",1)
        .replacen(formal,"n: Int, roll: Angle) {",1).replacen("through: body)","from: -roll, to: roll)",1);
    let call = format!("{member} := Member(");
    let at = text.find(&call).unwrap_or_else(|| panic!("the pair makes `{member}` by `Member`"));
    let count = format!("n: {member}_teeth)");
    let end = at+text[at..].find(&count).expect("the member's call ends at its tooth count")+count.len();
    let rolled = text[at..end].replacen("Member(","RolledMember(",1)
        .replacen(&count,&format!("n: {member}_teeth, roll: {degrees}deg)"),1);
    format!("{}{rolled}{}\n{member_copy}{generated_copy}",&text[..at],&text[end..])
}

/// Write the pair into `dir` as `hypoid.sv` at a design (`design`: the offset in millimetres, the
/// shift and the spiral in degrees; no fabrication allowances), and say where.
pub fn copy_design(dir: &std::path::Path,offset: f64,shift: f64,spiral: f64) -> PathBuf {
    std::fs::create_dir_all(dir).unwrap();
    let path = dir.join("hypoid.sv");
    std::fs::write(&path,design(&source(),offset,shift,spiral)).unwrap();
    path
}

/// `text`, the pair at some design, elaborated and solved accurately.
pub fn read(text: &str) -> program::Elaborated {
    crate::elaborate(text,&mut |name| gcs_core::library::resolve(name))
}

/// `text` elaborated and not solved: where its words and the elaborator put it.
pub fn unsolved(text: &str) -> program::Elaborated {
    crate::unsolved(text,&mut |name| gcs_core::library::resolve(name))
}

/// The pair as configured, offset and fabrication allowances and all.
pub fn read_as_configured() -> program::Elaborated { read(&source()) }

/// The twelve designs the layout regression records (`tests/hypoid_layout.rs`), each a label and
/// the document at it: the configured hypoid (with no fabrication allowances, as it was
/// recorded), the bevel pair, the six-millimetre hypoid, and the bevel pair at three tooth pairs
/// and three modules — the sizes and ratios the paired-envelope checks read.
pub fn designs() -> Vec<(String,String)> {
    let text = source();
    let mut all = vec![
        ("configured".to_string(),design(&text,25.,12.5,25.)),
        ("bevel".to_string(),bevel(&text)),
        ("hypoid6".to_string(),hypoid6(&text)),
    ];
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            all.push((format!("{}x{} m{module}",teeth[0],teeth[1]),configure(&text,&configuration(teeth,module,0.,0.,35.))));
        }
    }
    all
}
