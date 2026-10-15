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
/// The face mill's pressure shift, which `hypoid.sv` finds rather than states (#283), is set as
/// `recorded_machine` sets it, with the cutter the suites recorded: the Formate pair, which still
/// states it, takes it as an input.
pub fn configure(text: &str,params: &[(&str,f64)]) -> String {
    let shift = params.iter().find(|(name,_)| *name == "pressure_shift").map(|&(_,v)| v);
    let states_shift = text.contains("param pressure_shift :=");
    let rest: Vec<(&str,f64)> = params.iter().copied()
        .filter(|(name,_)| states_shift || *name != "pressure_shift").collect();
    let out = configure_inputs(text,&rest);
    match shift {
        Some(v) if !states_shift => recorded_machine(&out,v),
        _ => out,
    }
}

/// `text`, the pair, with the face mill's settings stated as the suites recorded the pair at
/// before `hypoid.sv` found them (#283): the pressure shift `shift` in degrees and the cutter's
/// radius 0.8 R, R the mean cone distance — and the balance that finds them dropped. Every
/// number the suites recorded was taken of a pair so cut.
pub fn recorded_machine(text: &str,shift: f64) -> String {
    let seeds = "pressure_shift: hint(10deg), cutter_radius: hint(80mm))";
    assert!(text.contains(seeds),"the pair leaves its face mill's settings to the solve, seeded");
    let stated = format!("pressure_shift: {shift}deg, \
        cutter_radius: 0.8 * mean_module * sqrt(pinion_teeth^2 + gear_teeth^2) / 2)");
    let mut out = text.replacen(seeds,&stated,1);
    let call = "  balance := Balanced(";
    let at = out.find(call).expect("the pair finds its face mill's settings by `Balanced`");
    let mut depth = 0;
    let end = at+out[at..].char_indices().find_map(|(k,c)| {
        match c { '(' => depth += 1, ')' => { depth -= 1; if depth == 0 { return Some(k+1); } } _ => {} }
        None
    }).expect("the balance's call closes");
    let end = end+out[end..].find('\n').map_or(0,|k| k+1);
    out.replace_range(at..end,"");
    out
}

/// `configure` over the document's own inputs alone.
fn configure_inputs(text: &str,params: &[(&str,f64)]) -> String {
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
/// the negative controls, a roll too short or too long.
pub fn roll(text: &str,member: &str,degrees: f64) -> String { rolled(text,member,[-degrees,degrees]) }

/// `text` with a member's cut rolled over `degrees`, stated (`roll`): the member is made by copies
/// of `Member` and `Generated` as `text` has them, taking the roll, made once for every member
/// so rolled.
pub fn rolled(text: &str,member: &str,degrees: [f64;2]) -> String {
    let component = |name: &str| -> String {
        let head = format!("\ncomponent {name}(");
        let at = text.find(&head).unwrap_or_else(|| panic!("the pair has a component `{name}`"))+1;
        let end = at+text[at..].find("\n}\n").expect("a component ends at a closing brace")+3;
        text[at..end].to_string()
    };
    let formal = "n: Int) {";
    let copies = if text.contains("\ncomponent RolledMember(") { String::new() } else {
        let member_copy = component("Member").replacen("component Member(","component RolledMember(",1)
            .replacen(formal,"n: Int, from: Angle, to: Angle) {",1)
            .replacen("Generated(","RolledGenerated(",1).replacen("n: n)","n: n, from: from, to: to)",1);
        let generated = component("Generated");
        assert!(generated.contains("through: body)"),"the cut rolls through its body");
        let generated_copy = generated.replacen("component Generated(","component RolledGenerated(",1)
            .replacen(formal,"n: Int, from: Angle, to: Angle) {",1)
            .replacen("through: body)","from: from, to: to)",1);
        format!("\n{member_copy}{generated_copy}")
    };
    let call = format!("{member} := Member(");
    let at = text.find(&call).unwrap_or_else(|| panic!("the pair makes `{member}` by `Member`"));
    let count = format!("n: {member}_teeth)");
    let end = at+text[at..].find(&count).expect("the member's call ends at its tooth count")+count.len();
    let rolled = text[at..end].replacen("Member(","RolledMember(",1).replacen(&count,
        &format!("n: {member}_teeth, from: {}deg, to: {}deg)",degrees[0],degrees[1]),1);
    format!("{}{rolled}{}{copies}",&text[..at],&text[end..])
}

/// Write the pair into `dir` as `hypoid.sv` at a design (`design`: the offset in millimetres, the
/// shift and the spiral in degrees; no fabrication allowances), and say where.
pub fn copy_design(dir: &std::path::Path,offset: f64,shift: f64,spiral: f64) -> PathBuf {
    std::fs::create_dir_all(dir).unwrap();
    let path = dir.join("hypoid.sv");
    std::fs::write(&path,design(&source(),offset,shift,spiral)).unwrap();
    path
}

/// `text`, the pair at some design, elaborated and solved accurately, its modules read beside the
/// project (the Formate pair imports `hypoid`), then from the library.
pub fn read(text: &str) -> program::Elaborated { crate::read_beside(text,&project(),&mut |_,t| t) }

/// `text` elaborated and not solved: where its words and the elaborator put it.
pub fn unsolved(text: &str) -> program::Elaborated { crate::unsolved(text,&mut crate::beside(&project(),&mut |_,t| t)) }

/// The pair as configured, offset and fabrication allowances and all.
pub fn read_as_configured() -> program::Elaborated { read(&source()) }

/// The Formate pair's document, `formate.sv` (#283, step 1): the gear formed, the pinion its blank
/// less the gear's tooth swept under the mesh. It states the same inputs as `hypoid.sv`, so
/// `configure`, `design` and `fabricated` set them, and `read` reads it, `hypoid` beside it.
pub fn formate_source() -> String { std::fs::read_to_string(project().join("formate.sv")).unwrap() }

/// The pair stated as its meshing problem, `meshing_problem.sv` (#345, S4): `formate.sv`'s pair
/// with the gear's concave flanks eased off the face mill's cone, the ease found from the contact
/// the document states. `read` reads it, `hypoid` and `formate` beside it.
pub fn meshing_problem_source() -> String {
    std::fs::read_to_string(project().join("meshing_problem.sv")).unwrap()
}

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
