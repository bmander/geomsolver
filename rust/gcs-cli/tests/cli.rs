//! `solventc` end to end: what a document's exit code says, and what the report reads like.
//!
//! The binary is the point of the tests — a `Diag` carrying a code and a span has existed since
//! the app's banner, and this is the first consumer of it that a CI job can run.

use std::path::PathBuf;
use std::process::{Command, Output};

fn examples() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../examples")
}

fn run(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_solventc")).args(args).output().expect("solventc runs")
}

fn doc(name: &str) -> String {
    examples().join(name).to_string_lossy().into_owned()
}

#[test]
fn version_is_said_and_succeeds() {
    let output = run(&["--version"]);
    assert_eq!(output.status.code(),Some(0));
    assert!(String::from_utf8_lossy(&output.stdout).starts_with("solventc "));
}

/// A command line that cannot be read says what is wrong first, then the synopsis and where the
/// options are — not every option, which would push the one line that matters off the screen —
/// and exits 64, apart from what a document is.
#[test]
fn a_wrong_command_line_says_why_and_points_at_help() {
    for (args,why) in [(&[][..],"no document given"),(&["--bogus","x.sv"][..],"unknown option `--bogus`"),
        (&["--glb","x.glb","x.sv"][..],"unknown option `--glb`"),(&["x.sv","--where"][..],"--where needs a name")] {
        let output = run(args);
        assert_eq!(output.status.code(),Some(64),"{args:?}");
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(stderr.starts_with(&format!("solventc: {why}\n")),"{args:?}: {stderr}");
        assert!(stderr.contains("usage: solventc [OPTIONS] FILE...") && stderr.contains("solventc --help"),"{stderr}");
        assert!(!stderr.contains("--measure-samples"),"the whole of the help is not repeated: {stderr}");
        assert!(output.stdout.is_empty());
    }
}

/// `--help` is the whole of it: every option the parser reads, examples, the exit codes and where
/// the language is written down.
#[test]
fn help_lists_every_option_examples_and_the_primer() {
    let output = run(&["--help"]);
    assert_eq!(output.status.code(),Some(0));
    let help = String::from_utf8_lossy(&output.stdout);
    let source = include_str!("../src/main.rs");
    let parser = &source[source.find("fn main()").unwrap()..];
    let mut read = 0;
    for option in parser.split('"').skip(1).step_by(2)
        .filter(|s| s.starts_with('-') && s.len() > 1 && !s.contains(char::is_whitespace)) {
        assert!(help.contains(&format!(" {option} ")) || help.contains(&format!(" {option},")),
            "{option} is read but not in --help");
        read += 1;
    }
    assert!(read > 20,"the parser's options were found ({read})");
    assert!(help.contains("Examples:") && help.contains("solvent-primer.md") && help.contains("64 the command line"));
    assert!(help.lines().all(|l| l.chars().count() <= 100),"--help fits 100 columns");
}

#[test]
fn unknown_stl_backend_is_refused() {
    let output = run(&["--stl-backend","unknown"]);
    assert_eq!(output.status.code(),Some(64));
    assert!(String::from_utf8_lossy(&output.stderr).contains("occt, mesh or refine"));
}

#[test]
fn a_continuous_boundary_exports_through_the_field_mesh() {
    let dir = std::env::temp_dir().join(format!("solventc-sweep-mesh-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let output = dir.join("swept.stl");
    // the ball slid along the spindle, not turned about it: a turned ball is its tube, built
    // exactly (#238, `solid::canal`), and a slid one is still the field's
    let source = dir.join("slid.sv");
    let turned = include_str!("../../examples/solid_generating_sweep.sv");
    let (turn,slide) = ("generating turns(1) spindle","generating slides(6mm) spindle");
    assert!(turned.contains(turn),"the sweep's turn moved");
    let slid = turned.replace("use std (on_u, turns)","use std (on_u, slides)").replace(turn,slide);
    std::fs::write(&source,slid).unwrap();
    let result = run(&[source.to_str().unwrap(),"--stl",output.to_str().unwrap(),
        "--solid","removal.body","--stl-backend","mesh","--no-diagnose"]);
    assert_eq!(result.status.code(),Some(0),"{}",String::from_utf8_lossy(&result.stderr));
    let bytes = std::fs::read(&output).unwrap();
    gcs_core::mesh::stl_shells(&bytes).unwrap();
    std::fs::remove_dir_all(dir).unwrap();
}

/// A body whose sweep is outside the generating class is refused before any construction,
/// with the row it fails and a point, and an earlier output is left as it was: the gear pair
/// 30 mm off the bevel on a symmetric rack, whose pinion touches the blank twice.
#[test]
fn a_sweep_outside_the_generating_class_is_refused_with_its_row() {
    let dir = std::env::temp_dir().join(format!("solventc-class-refusal-{}",std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let document = fixtures::gear::copy_design(&dir,30.,0.,35.);
    let output = dir.join("pinion.stl");
    std::fs::write(&output,"old STL").unwrap();
    let result = run(&[document.to_str().unwrap(),"--stl",output.to_str().unwrap(),
        "--solid","pair.pinion.body","--no-diagnose"]);
    let stderr = String::from_utf8_lossy(&result.stderr);
    assert_eq!(result.status.code(),Some(1),"{stderr}");
    assert!(stderr.contains("outside the generating-sweep class (E2"),"{stderr}");
    assert_eq!(std::fs::read_to_string(output).unwrap(),"old STL");
    std::fs::remove_dir_all(dir).unwrap();
}

/// A refined mesh the field-agreement probe refuses is never written: the swept torus meshed
/// so coarsely its facets leave the field, and the earlier output is left as it was.
#[cfg(feature="occt")]
#[test]
fn a_refined_mesh_the_field_refuses_leaves_the_old_output() {
    let dir = std::env::temp_dir().join(format!("solventc-refine-refusal-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let output = dir.join("part.stl");
    std::fs::write(&output,"old STL").unwrap();
    let result = Command::new(env!("CARGO_BIN_EXE_solventc"))
        .args([doc("swept_torus.sv").as_str(),"--stl",output.to_str().unwrap(),"--stl-backend","refine","--no-diagnose"])
        .env("SOLVENT_REFINE_FACET","3").env("SOLVENT_REFINE_DISTANCE","1")
        .env_remove("SOLVENT_FEATURES").env_remove("SOLVENT_KEEP_REJECTED")
        .output().expect("solventc runs");
    let stderr = String::from_utf8_lossy(&result.stderr);
    assert_eq!(result.status.code(),Some(1),"{stderr}");
    assert!(stderr.contains("disagrees with the material field"),"{stderr}");
    assert_eq!(std::fs::read_to_string(&output).unwrap(),"old STL");
    assert_eq!(std::fs::read_dir(&dir).unwrap().count(),1,"nothing staged is left beside it");
    std::fs::remove_dir_all(dir).unwrap();
}

/// The swept torus held to 0.1 µm (docs/native-hypoid-plan.md): the first sheet
/// (24x24, 0.11 µm from its withheld contacts at best against a 0.05 µm bar) misses, is refined
/// where it misses and fitted again until it holds, and the meter reads both files within the
/// tolerance.
#[cfg(feature="occt")]
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 16 s: the swept torus exported at 0.1 µm and measured")]
fn a_swept_export_is_refined_into_its_tolerance() {
    let dir = std::env::temp_dir().join(format!("solventc-tolerance-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let (stl,step) = (dir.join("part.stl"),dir.join("part.step"));
    let (stl,step) = (stl.to_str().unwrap(),step.to_str().unwrap());
    // (the slow tier verifies its files in full: the kernel reads them back as well)
    let result = run(&[&doc("swept_torus.sv"),"--stl",stl,"--step",step,"--tolerance","0.1um","--verify-step","full","--no-diagnose"]);
    let stderr = String::from_utf8_lossy(&result.stderr);
    assert_eq!(result.status.code(),Some(0),"{stderr}");
    let fits: Vec<&str> = stderr.lines().filter(|l| l.contains("withheld contacts at the blank (bar 0.05 µm)")).collect();
    assert!(fits.len() >= 4,"{stderr}");
    assert!(stderr.contains("each the solid's, and read back by the kernel as"),"{stderr}");
    // Both parametrizations miss at first; the last fitted holds.
    assert!(fits[..2].iter().all(|l| !l.contains("; 0 miss by distance, 0 by normal")),"{stderr}");
    assert!(fits.last().unwrap().contains("; 0 miss by distance, 0 by normal"),"{stderr}");
    assert!(stderr.contains("refining the sheet where it misses") && stderr.contains("fit holds"),"{stderr}");
    assert!(stderr.contains("probed 0.0020 mm off each side") && stderr.contains("0 disagree"),"{stderr}");
    for file in [step,stl] {
        let measured = run(&[&doc("swept_torus.sv"),"--measure",file,"--tolerance","0.1um","--no-diagnose"]);
        let stdout = String::from_utf8_lossy(&measured.stdout);
        assert_eq!(measured.status.code(),Some(0),"{stdout}{}",String::from_utf8_lossy(&measured.stderr));
        assert!(stdout.contains("tolerance 0.100 µm: every exact face within it"),"{stdout}");
    }
    // Without a tolerance the same sheet passes the gross bars as it is (fitted by centripetal
    // parameters there, as this kernel fits a gross sheet; OCCT's chord-length fit was 2.19e-4 mm).
    let gross = run(&[&doc("swept_torus.sv"),"--step",step,"--no-diagnose"]);
    let stderr = String::from_utf8_lossy(&gross.stderr);
    assert_eq!(gross.status.code(),Some(0),"{stderr}");
    assert!(!stderr.contains("refining") && stderr.contains("fitted sheet within 6.60e-4 mm"),"{stderr}");
    std::fs::remove_dir_all(dir).unwrap();
}

/// The twist drill (`twist_drill/drill.sv`, issue #64): its flutes ground under a screw, its point,
/// its shank, written by the core's kernel within 10 µm and measured against its exact faces —
/// and, in a build with OCCT, read back by it as a valid solid of the same volume.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about two minutes: the twist drill exported and measured")]
fn the_twist_drill_exports_and_measures_within_its_tolerance() {
    let dir = std::env::temp_dir().join(format!("solventc-drill-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let (stl,step) = (dir.join("drill.stl"),dir.join("drill.step"));
    let (stl,step) = (stl.to_str().unwrap(),step.to_str().unwrap());
    let drill = examples().join("twist_drill/drill.sv");
    let drill = drill.to_str().unwrap();
    let result = run(&[drill,"--stl",stl,"--step",step,"--tolerance","0.01mm","--verify-step","full","--no-diagnose"]);
    let stderr = String::from_utf8_lossy(&result.stderr);
    assert_eq!(result.status.code(),Some(0),"{stderr}");
    assert!(stderr.contains("`drill` built from its swept material by this kernel") && stderr.contains("0 disagree"),"{stderr}");
    #[cfg(feature="occt")]
    assert!(stderr.contains("each the solid's, and read back by the kernel as a valid solid of 14 faces"),"{stderr}");
    let files: &[&str] = if cfg!(feature = "occt") { &[stl,step] } else { &[stl] };
    for file in files {
        let measured = run(&[drill,"--measure",file,"--tolerance","0.01mm","--no-diagnose"]);
        let stdout = String::from_utf8_lossy(&measured.stdout);
        assert_eq!(measured.status.code(),Some(0),"{stdout}{}",String::from_utf8_lossy(&measured.stderr));
        assert!(stdout.contains("tolerance 10.000 µm: every exact face within it"),"{stdout}");
    }
    std::fs::remove_dir_all(dir).unwrap();
}

/// The Wankel engine (`wankel/wankel.sv`, issue #65): its rotor admitted to the planar generating
/// class and built as its blank within the bore's inner envelope, its housing a disc less the bore,
/// both written by the core's kernel within 10 µm, the rotor's mesh held to its material field, and
/// each measured against its exact faces (the rotor's by the envelope through its slab).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about half a minute: the Wankel's rotor and housing exported and measured")]
fn the_wankel_rotor_and_housing_export_and_measure_within_their_tolerance() {
    let dir = std::env::temp_dir().join(format!("solventc-wankel-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let engine = examples().join("wankel/wankel.sv");
    let engine = engine.to_str().unwrap();
    for part in ["rotor","housing"] {
        let (stl,step) = (dir.join(format!("{part}.stl")),dir.join(format!("{part}.step")));
        let (stl,step) = (stl.to_str().unwrap(),step.to_str().unwrap());
        let result = run(&[engine,"--solid",part,"--stl",stl,"--step",step,"--tolerance","0.01mm","--no-diagnose"]);
        let stderr = String::from_utf8_lossy(&result.stderr);
        assert_eq!(result.status.code(),Some(0),"{stderr}");
        if part == "rotor" {
            assert!(stderr.contains("`swept` is in the planar generating class") && stderr.contains("0 disagree"),"{stderr}");
        }
        let measured = run(&[engine,"--solid",part,"--measure",stl,"--tolerance","0.01mm","--no-diagnose"]);
        let stdout = String::from_utf8_lossy(&measured.stdout);
        assert_eq!(measured.status.code(),Some(0),"{stdout}{}",String::from_utf8_lossy(&measured.stderr));
        assert!(stdout.contains("tolerance 10.000 µm: every exact face within it"),"{stdout}");
    }
    std::fs::remove_dir_all(dir).unwrap();
}

/// The rounds series (`rounds/`, #194): each round and chamfer the lowering table builds is written
/// by the Rust kernel as STEP and STL within 10 µm and measured so against its exact faces —
/// `boss.sv`'s ring, `pocket.sv`'s filled corners, the block's mitred rims, the round written by
/// hand, `blank.sv`'s rounds on an offset cone built in its meridian (#207), `slot.sv`'s rounds round
/// tangent chains, `pad.sv`'s, `rail.sv`'s beside its rod, `knob.sv`'s rings and the balls rolled
/// along traced runs of `tee.sv`, `bore.sv` and both of `runout.sv`'s solids, and `twin.sv`'s seams
/// between two cylinders, `post.sv`'s runs cut off by a cylinder and `dshaft.sv` rounded all over,
/// its corners each a ball's cell with a patch of a cylinder (#218); `groove.sv`'s runs mitred
/// against rings on one cylinder, `bores.sv`'s corners over a plane and two cylinders,
/// `frustum.sv`'s rolled generators glued to its corners, `crotch.sv`'s flat along a traced
/// saddle, `bullnose.sv`'s root rolled across a tangent seam onto a ring's stretch and `spill.sv`'s
/// chamfer spilled onto the plate's rim round, a ring glued to the flat beyond the seam, and
/// `ledge.sv`'s along a straight edge, a prism of a line and a conic (#422).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about a minute: the rounds exported and measured")]
fn the_rounds_export_and_measure_within_their_tolerance() {
    let dir = std::env::temp_dir().join(format!("solventc-rounds-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let files = ["block","by_hand","pocket","boss","blank","slot","pad","rail","knob","tee","bore","twin","post","dshaft"].map(|f| (f,None));
    let more = [("runout",Some("lug")),("runout",Some("vent")),("groove",Some("part")),("bores",Some("slab")),
        ("frustum",Some("half")),("crotch",Some("tee")),("bullnose",Some("part")),("spill",Some("boss")),
        ("ledge",Some("part"))];
    for (file,solid) in files.into_iter().chain(more) {
        let name = solid.unwrap_or(file);
        let source = doc(&format!("rounds/{file}.sv"));
        let (stl,step) = (dir.join(format!("{name}.stl")),dir.join(format!("{name}.step")));
        let (stl,step) = (stl.to_str().unwrap(),step.to_str().unwrap());
        let which: Vec<&str> = solid.iter().flat_map(|s| ["--solid",*s]).collect();
        // one run writes both and measures what it wrote
        let measured = run(&[&[source.as_str(),"--stl",stl,"--step",step,"--measure",stl,"--tolerance","0.01mm","--no-diagnose"],
            &which[..]].concat());
        let stdout = String::from_utf8_lossy(&measured.stdout);
        assert_eq!(measured.status.code(),Some(0),"{name}: {stdout}{}",String::from_utf8_lossy(&measured.stderr));
        assert!(stdout.contains("tolerance 10.000 µm: every exact face within it"),"{name}: {stdout}");
    }
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn a_tolerance_is_a_positive_length_for_a_native_export() {
    let output = run(&[&doc("swept_torus.sv"),"--tolerance","3furlongs","--step","x.step"]);
    assert_eq!(output.status.code(),Some(64));
    assert!(String::from_utf8_lossy(&output.stderr).contains("`furlongs` is not a length unit"));
    let output = run(&[&doc("swept_torus.sv"),"--tolerance","0mm","--step","x.step"]);
    assert_eq!(output.status.code(),Some(64));
    assert!(String::from_utf8_lossy(&output.stderr).contains("a tolerance is a positive length"));
    // It holds an export or a measurement to it, and means nothing to a document only checked.
    let output = run(&[&doc("swept_torus.sv"),"--tolerance","--no-diagnose"]);
    assert_eq!(output.status.code(),Some(64));
    assert!(String::from_utf8_lossy(&output.stderr).contains("--tolerance holds a native export"));
}

/// One member of the configured hypoid pair exported natively for fabrication, STEP and STL at
/// 10 µm, the field gate passed, and both files measured within 10 µm on every exact face.
#[cfg(feature="occt")]
fn fabricated(member: &str) {
    let dir = std::env::temp_dir().join(format!("solventc-native-{member}-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let (stl,step) = (dir.join(format!("{member}.stl")),dir.join(format!("{member}.step")));
    let (stl,step) = (stl.to_str().unwrap(),step.to_str().unwrap());
    let body = format!("pair.{member}.body");
    let result = run(&[&doc("spiral_bevel/hypoid.sv"),"--solid",&body,"--stl",stl,"--step",step,"--tolerance","0.01mm",
        "--verify-step","full","--no-diagnose"]);
    let stderr = String::from_utf8_lossy(&result.stderr);
    assert_eq!(result.status.code(),Some(0),"{stderr}");
    assert!(stderr.contains("each the solid's, and read back by the kernel as"),"{stderr}");
    assert!(stderr.contains("probed 0.0200 mm off each side") && stderr.contains("0 disagree"),"{stderr}");
    gcs_core::mesh::stl_shells(&std::fs::read(stl).unwrap()).unwrap();
    for file in [step,stl] {
        let measured = run(&[&doc("spiral_bevel/hypoid.sv"),"--solid",&body,"--measure",file,"--tolerance","0.01mm","--no-diagnose"]);
        let stdout = String::from_utf8_lossy(&measured.stdout);
        assert_eq!(measured.status.code(),Some(0),"{stdout}{}",String::from_utf8_lossy(&measured.stderr));
        assert!(stdout.contains("tolerance 10.000 µm: every exact face within it"),"{stdout}");
    }
    std::fs::remove_dir_all(dir).unwrap();
}

/// The configured hypoid gear exports natively, STEP and STL, through the swept construction
/// and its field-agreement gate, held to 10 µm. It was refused at the fit until the sheet's rows
/// stopped before a margin column's leap into a far corner's fan (docs/native-hypoid-plan.md).
#[cfg(feature="occt")]
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about six minutes: the configured gear exported at 10 µm and measured")]
fn the_configured_gear_exports_natively() { fabricated("gear"); }

/// The configured hypoid pinion at 10 µm: its fillets were 11.6 and 16.9 µm off at the gross bars
/// (docs/native-hypoid-plan.md) and are refined into the tolerance.
#[cfg(feature="occt")]
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about five minutes: the configured pinion exported at 10 µm and measured")]
fn the_configured_pinion_exports_natively_within_its_tolerance() { fabricated("pinion"); }

#[cfg(not(feature="occt"))]
#[test]
fn explicit_native_stl_never_falls_back_when_occt_is_unavailable() {
    let dir = std::env::temp_dir().join(format!("solventc-no-occt-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let model = dir.join("model.sv");
    let output = dir.join("model.stl");
    std::fs::write(&model,"\
unit mm
use std
in std.front {
o := point
fix((0, 0)) o
c := circle(center: o)
radius(2) c
}
body := solid(face(c), depth: 3)
").unwrap();
    std::fs::write(&output,"old STL").unwrap();
    let result = run(&[model.to_str().unwrap(),"--stl",output.to_str().unwrap(),
        "--stl-backend","occt","--no-diagnose"]);
    assert_eq!(result.status.code(),Some(1));
    assert!(String::from_utf8_lossy(&result.stderr).contains("requires native OCCT support"));
    assert_eq!(std::fs::read_to_string(output).unwrap(),"old STL");
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn component_previews_resolve_project_imports_for_models_and_drawings() {
    let dir = std::env::temp_dir().join(format!("solventc-preview-{}", std::process::id()));
    std::fs::create_dir_all(dir.join("parts")).unwrap();
    std::fs::write(dir.join("parts/dims.sv"), "width := 20mm\n").unwrap();
    let part = dir.join("parts/bar.sv");
    std::fs::write(&part, "use parts.dims\ncomponent Bar(w: Length) {\n\
        a := point\nfix(x == 0, y == 0, z == 0) a\nb := point\nfix(x == w, y == 0, z == 0) b\n\
        border := line(a, b)\n}\npreview {\nunit mm\ndemo := Bar(w: parts.dims.width)\n}\n").unwrap();
    let args = [part.to_str().unwrap(), "--json", "--where", "demo.b.x"];
    let result = run(&args);
    assert!(result.status.success(), "{}", String::from_utf8_lossy(&result.stdout));
    assert!(String::from_utf8_lossy(&result.stdout).contains("\"demo.b.x\": 20"));

    let sheet = dir.join("bar.svd");
    let output = dir.join("bar.svg");
    std::fs::write(&sheet, "model m from \"parts/bar.sv\"\n\
        sheet part { sketch v(m) at (30,40) measure distance(m.demo.a,m.demo.b) in v }").unwrap();
    let result = run(&[sheet.to_str().unwrap(), "--output", output.to_str().unwrap()]);
    assert!(result.status.success(), "{}", String::from_utf8_lossy(&result.stderr));
    assert!(std::fs::read_to_string(output).unwrap().contains(">20</text>"));

    // A nearer project definition wins over the ancestor's copy.
    std::fs::create_dir_all(dir.join("parts/parts")).unwrap();
    std::fs::write(dir.join("parts/parts/dims.sv"), "width := 30mm\n").unwrap();
    let result = run(&args);
    assert!(result.status.success());
    assert!(String::from_utf8_lossy(&result.stdout).contains("\"demo.b.x\": 30"));
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn drawings_load_relative_files_select_sheets_and_refuse_broken_references() {
    let dir = std::env::temp_dir().join(format!("solventc-drawing-{}", std::process::id()));
    std::fs::create_dir_all(dir.join("sheets")).unwrap();
    std::fs::write(dir.join("part.sv"), "\
unit mm
use std
in std.front {
a := point
fix((0, 0)) a
b := point
fix((20, 0)) b
bar := line(a,b)
}
").unwrap();
    std::fs::write(dir.join("ink.svd"), "style m.bar { color: #123456 }").unwrap();
    let path = dir.join("sheets/part.svd");
    let source = "model m from \"../part.sv\" use \"../ink.svd\"\n\
        sheet a { sketch v(m) at (30,40) measure distance(m.bar.p1,m.b) in v }\n\
        sheet b { size (100mm,80mm) sketch v(m) at (30,40) }";
    std::fs::write(&path, source).unwrap();
    let output = dir.join("part.svg");
    let args = [path.to_str().unwrap(), "--sheet", "a", "--output", output.to_str().unwrap()];
    let r = run(&args);
    assert!(r.status.success(), "{}", String::from_utf8_lossy(&r.stderr));
    let svg = std::fs::read_to_string(&output).unwrap();
    assert!(svg.contains("#123456") && svg.contains(">20</text>"));
    assert_eq!(run(&[path.to_str().unwrap()]).status.code(), Some(1));
    std::fs::write(&path, source.replace("m.b)", "m.missing)")).unwrap();
    let r = run(&args);
    assert_eq!(r.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&r.stderr).contains("not a point"));
    assert_eq!(std::fs::read_to_string(&output).unwrap(), svg, "failed compile preserves export");
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn issue_50_reductions_report_or_diagnose_instead_of_exporting_invalid_solids() {
    let fixtures = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../gcs-core/tests/fixtures/solid_issue50");
    let temp = std::env::temp_dir().join(format!("solventc-issue50-{}", std::process::id()));
    std::fs::create_dir_all(&temp).unwrap();
    for entry in std::fs::read_dir(fixtures).unwrap() {
        let path = entry.unwrap().path();
        let filename = path.file_name().unwrap().to_str().unwrap();
        let item: usize = filename[..2].parse().unwrap();
        let output = temp.join(format!("{item}.stl"));
        let out = run(&[path.to_str().unwrap(), "--json", "--where", "result", "--solid", "result", "--stl-backend", "mesh", "--stl", output.to_str().unwrap()]);
        let json = gcs_core::json::parse(&String::from_utf8_lossy(&out.stdout)).unwrap();
        let doc = &json.get("documents").unwrap().arr()[0];
        let invalid = [2, 8, 9, 10, 12].contains(&item);
        assert_eq!(out.status.code(), Some(if invalid { 1 } else { 0 }), "item {item}: {}", String::from_utf8_lossy(&out.stderr));
        if invalid {
            assert!(!doc.get("diagnostics").unwrap().arr().is_empty(), "item {item}");
            assert!(!output.exists(), "item {item}: no misleading export");
        } else {
            assert!(output.metadata().unwrap().len() > 84, "item {item}: a nonempty STL");
        }
        if item == 2 {
            assert!(String::from_utf8_lossy(&out.stdout).contains("float32 STL"));
            let out = run(&[path.to_str().unwrap(), "--json", "--where", "result"]);
            assert_eq!(out.status.code(), Some(0), "the f64 report succeeds without an STL export");
            assert!(String::from_utf8_lossy(&out.stdout).contains("result.volume"));
        }
    }
    std::fs::remove_dir_all(temp).unwrap();
}

/// **The library, checked from a terminal.**  Every document reports; the three deliberately
/// unsatisfiable ones are the only nonzero exits, and `--allow-unsolved` makes those zero too.
/// The under-constrained cases exit 0: they solve, they just have freedoms left.
#[test]
fn the_whole_library_reports() {
    let all: Vec<String> = [examples(), examples().join("vtwin"), examples().join("vtwin/components")].into_iter()
        .flat_map(|dir| std::fs::read_dir(dir).expect("the example documents"))
        .filter_map(|e| e.ok())
        .map(|e| e.path().to_string_lossy().into_owned())
        .filter(|p| p.ends_with(".sv"))
        .filter(|p| !p.contains("/vtwin/components/")
            || std::fs::read_to_string(p).unwrap().contains("\npreview {"))
        .collect();
    let args: Vec<&str> = all.iter().map(String::as_str).collect();
    let out = run(&args);
    let text = String::from_utf8_lossy(&out.stdout);
    assert_eq!(out.status.code(), Some(2), "the unsatisfiable ones are a failure");
    assert_eq!(text.lines().filter(|l| l.contains(": solved")).count(), all.len() - 3);

    // told not to be, they are not — asked over the three that fail rather than the whole
    // library again, which is the same assertion for a second parse, solve and diagnosis of
    // every document (1.2 s of `make test` in the profile the tests build)
    let bad: Vec<String> =
        ["impossible_triangle.sv", "truss_conflict.sv", "rect_fillets_conflict.sv"]
            .iter()
            .map(|f| doc(f))
            .collect();
    let mut ok: Vec<&str> = bad.iter().map(String::as_str).collect();
    ok.push("--allow-unsolved");
    assert_eq!(run(&ok).status.code(), Some(0), "and told not to be, they are not");

    for under in ["rect_fillets_under.sv", "truss_floating.sv"] {
        assert_eq!(run(&[&doc(under)]).status.code(), Some(0), "{under}: solved, with freedoms");
    }
    for b in &bad {
        assert_eq!(run(&[b]).status.code(), Some(2), "{b}");
    }
}

/// **A solid read at a pose the solve did not settle on does not hide the solve** (#571).  Two
/// statements about `p1` conflict, and where the solve stops the rectangle straddles the axis it is
/// turned about.  The run says it did not solve, with its diagnosis, and exits 2; what the solid
/// says of that pose is a note under the core's `NOT_BUILT`, not an error of the document — and
/// under `--json`, `diagnosis` is filled and `diagnostics` does not hold it.
#[test]
fn a_solid_at_an_unsolved_pose_does_not_hide_the_diagnosis() {
    let dir = std::env::temp_dir().join(format!("solventc-unbuilt-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("a place to write");
    let path = dir.join("hidden.sv");
    std::fs::write(&path, "unit mm\nuse std\nin std.front {\n  p0 := point\n\
        \x20 p1 := point hint((12, 0))\n  p2 := point hint((12, 5))\n  p3 := point hint((2, 5))\n\
        \x20 fix((2, 0)) p0\n\
        \x20 sec := horizontal (e0 := line(p0, p1)) -> vertical (e1 := line(p1, p2)) ->\n\
        \x20        horizontal (e2 := line(p2, p3)) -> vertical (e3 := line(p3, p0)) -> close\n\
        \x20 p0 distance(10) p1\n  p1 distance(5) p2\n  std.origin distance(-6mm, along: u) p1\n}\n\
        ring := solid(sec, about: std.z)\n").expect("write");
    let path = path.to_string_lossy().into_owned();
    let out = run(&[&path]);
    let (text, err) = (String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    assert_eq!(out.status.code(), Some(2), "did not solve, whatever the solid does: {text}{err}");
    assert!(text.contains("hidden.sv: did not solve"), "{text}");
    assert!(text.contains("  conflict: std.origin distance(-6mm, along: u) p1"), "{text}");
    let not_built = text.find(gcs_core::program::NOT_BUILT).expect("the solid said as unbuilt");
    assert!(text.find("conflict:").unwrap() < not_built, "under the diagnosis: {text}");
    assert!(text.contains("hidden.sv:15:1: note[E080]: `ring`:"), "{text}");
    assert_eq!(text.matches("the revolution axis crosses the profile").count(), 1, "once: {text}");
    assert!(!err.contains("error["), "no error of the document: {err}");
    // told an unsolved drawing is no failure, its solids are none either
    assert_eq!(run(&[&path, "--allow-unsolved"]).status.code(), Some(0));

    let out = run(&["--json", &path]);
    assert_eq!(out.status.code(), Some(2));
    let v = gcs_core::json::parse(&String::from_utf8_lossy(&out.stdout)).expect("valid JSON");
    let d = &v.get("documents").expect("documents").arr()[0];
    let conflicts = d.get("diagnosis").and_then(|g| g.get("conflicts")).expect("the diagnosis");
    assert_eq!(conflicts.arr().len(), 3, "the conflict, as the text says it");
    assert!(d.get("diagnostics").unwrap().arr().is_empty(), "no error of the document");
    let unbuilt = d.get("notBuilt").expect("what was not built");
    assert_eq!(unbuilt.get("why").unwrap().as_str(), gcs_core::program::NOT_BUILT);
    let notes = unbuilt.get("diagnostics").unwrap().arr();
    assert_eq!(notes.len(), 1);
    assert_eq!(notes[0].get("code").unwrap().as_str(), "E080");
    assert_eq!(notes[0].get("severity").unwrap().as_str(), "note");
    std::fs::remove_dir_all(dir).unwrap();
}

/// A document that does not elaborate exits 1, and says where — `file:line:col`, with the column
/// counting **characters**.  Offsets cross from the core in UTF-8 bytes, and a document with a
/// non-ASCII character before the offending token is the ordinary case (`gear.sv` has an em dash
/// in its second line), not a corner one.
#[test]
fn a_broken_document_says_where() {
    let dir = std::env::temp_dir().join("solventc-test");
    std::fs::create_dir_all(&dir).expect("a place to write");
    let path = dir.join("uni.sv");
    std::fs::write(&path, "use std\nin std.front {\né := point hint((0, 0))   // — an em dash\nl := line(é, zzz)\n}\n")
        .expect("write");
    let out = run(&[&path.to_string_lossy()]);
    let err = String::from_utf8_lossy(&out.stderr);
    assert_eq!(out.status.code(), Some(1));
    assert!(err.contains("uni.sv:4:14: error[E101]:"), "{err}");
    // `é` is two bytes: a byte column would say 15
    assert!(!err.contains(":4:15:"), "the column counts characters, not bytes: {err}");
    // and one finding is said once
    assert_eq!(err.matches("no such entity").count(), 1, "{err}");
}

/// `--json` parses, and carries the same numbers the text report does.
#[test]
fn the_json_report_carries_the_same_numbers() {
    let out = run(&["--json", &doc("rect_fillets.sv")]);
    assert_eq!(out.status.code(), Some(0));
    let v = gcs_core::json::parse(&String::from_utf8_lossy(&out.stdout)).expect("valid JSON");
    let docs = v.get("documents").expect("documents").arr().to_vec();
    assert_eq!(docs.len(), 1);
    let d = &docs[0];
    assert!(d.get("name").expect("name").as_str().ends_with("rect_fillets.sv"));
    assert!(d.get("solve").and_then(|s| s.get("success")).expect("success").as_bool());
    let dg = d.get("diagnosis").expect("diagnosis");
    assert_eq!(dg.get("dof").expect("dof").as_i64(), 0);
    // the text report says the same, in the core's own words
    let text = String::from_utf8_lossy(&run(&[&doc("rect_fillets.sv")]).stdout).into_owned();
    assert!(text.contains("DOF 0"), "{text}");
}

/// `--no-diagnose` solves and stops there.
#[test]
fn no_diagnose_solves_only() {
    let text = String::from_utf8_lossy(&run(&["--no-diagnose", &doc("rect_fillets.sv")]).stdout)
        .into_owned();
    assert!(text.contains(": solved"));
    assert!(!text.contains("DOF"), "{text}");
}

/// `--output` writes an SVG, and the writer is the **core's** — an "export SVG" button in the
/// web app must not be a second implementation.
#[test]
fn output_writes_an_svg() {
    let dir = std::env::temp_dir().join("solventc-test");
    std::fs::create_dir_all(&dir).expect("a place to write");
    let out = dir.join("rect.svg");
    let r = run(&["--output", &out.to_string_lossy(), &doc("rect_fillets.sv")]);
    assert_eq!(r.status.code(), Some(0));
    let svg = std::fs::read_to_string(&out).expect("an SVG");
    assert!(svg.starts_with("<svg xmlns=\"http://www.w3.org/2000/svg\""), "{}", &svg[..80]);
    assert!(svg.ends_with("</svg>\n"));
    assert!(!svg.contains("NaN") && !svg.contains("inf"), "every number is a number");
    // the four fillets, as arcs; the four sides, as lines; the dimensions, as text — each drawn
    // with the `param` it was written over, not the number the flattener settled it to
    assert_eq!(svg.matches("<path d=\"M").count(), 4);
    assert_eq!(svg.matches("<line ").count(), 4);
    assert!(svg.contains(">w</text>") && svg.contains(">h</text>") && svg.contains(">R r</text>"),
            "the dimensions are drawn as written");

    // one file, so one document
    let two = run(&["--output", &out.to_string_lossy(), &doc("rect_fillets.sv"), &doc("truss.sv")]);
    assert_eq!(two.status.code(), Some(64));
}

#[test]
fn issue51_sweep_reports_distinguish_sampling_from_failed_poses() {
    let fixtures =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../gcs-core/tests/fixtures/solid_issue51");
    for (case, failed, verdict) in
        [("sampling_disclosure", 0, "sampled-success"), ("sweep_impossible", 37, "undecided")]
    {
        let path = fixtures.join(format!("{case}.sv"));
        let text = run(&[path.to_str().unwrap()]);
        assert!(text.status.success());
        let text = String::from_utf8(text.stdout).unwrap();
        assert!(text.contains("sampling 37 poses"), "{text}");
        assert!(text.contains(&format!("({failed} failed)")), "{text}");
        if failed > 0 {
            assert!(text.contains("no solved valid poses"));
            assert_eq!(text.matches("unresolved at").count(), failed);
            assert_eq!(text.matches("solve failed:").count(), failed);
        }
        let out = run(&[path.to_str().unwrap(), "--json"]);
        assert!(out.status.success());
        let doc = gcs_core::json::parse(&String::from_utf8(out.stdout).unwrap()).unwrap();
        let claim = &doc.get("documents").unwrap().arr()[0]
            .get("diagnosis")
            .unwrap()
            .get("solidClaims")
            .unwrap()
            .arr()[0];
        assert_eq!(claim.get("verdict").unwrap().as_str(), verdict);
        assert_eq!(claim.get("samples").unwrap().as_i64(), 37);
        assert_eq!(claim.get("failedSamples").unwrap().arr().len(), failed);
        if failed > 0 {
            assert_eq!(claim.get("measured"), Some(&gcs_core::json::Json::Null));
            assert_eq!(claim.get("tolerance"), Some(&gcs_core::json::Json::Null));
            assert_eq!(claim.get("counterexample"), Some(&gcs_core::json::Json::Null));
            assert!(claim.get("worst").is_none());
            let poses = claim.get("poses").unwrap().arr();
            assert_eq!(poses.len(), failed);
            assert!(poses.iter().all(|p| p.get("status").unwrap().as_str() == "failed"
                && p.get("reason").unwrap().as_str().starts_with("solve failed:")));
        }
    }
}

#[test]
fn issue51_invalid_claims_and_colliding_cap_names_are_diagnosed() {
    let fixtures =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../gcs-core/tests/fixtures/solid_issue51");
    for case in [
        "arguments_inside(1mm)",
        "arguments_clear(1mm,2mm)",
        "sweep_dimensional_error",
        "cap_collision",
    ] {
        let path = fixtures.join(format!("{case}.sv"));
        let out = run(&[path.to_str().unwrap(), "--json"]);
        assert_eq!(out.status.code(), Some(1), "{case}");
        let doc = gcs_core::json::parse(&String::from_utf8(out.stdout).unwrap()).unwrap();
        assert!(!doc.get("documents").unwrap().arr()[0]
            .get("diagnostics")
            .unwrap()
            .arr()
            .is_empty());
    }
}

/// The core's own kernel exports a static solid in any build (`--kernel rust`): a bored block's
/// STEP, parsed back against it, and its STL, closed and within the bar.
#[test]
fn the_rust_kernel_exports_a_static_solid() {
    let dir = std::env::temp_dir().join(format!("solventc-rust-kernel-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let model = dir.join("model.sv");
    let (stl,step) = (dir.join("model.stl"),dir.join("model.step"));
    std::fs::write(&model,BORED).unwrap();
    let result = run(&[model.to_str().unwrap(),"--solid","body","--kernel","rust","--stl",stl.to_str().unwrap(),
        "--step",step.to_str().unwrap(),"--no-diagnose"]);
    let stderr = String::from_utf8_lossy(&result.stderr);
    assert_eq!(result.status.code(),Some(0),"{stderr}");
    assert!(stderr.contains("built the solid by the Rust kernel") && stderr.contains("each the solid's"),"{stderr}");
    gcs_core::mesh::stl_shells(&std::fs::read(&stl).unwrap()).unwrap();
    assert!(std::fs::read_to_string(&step).unwrap().starts_with("ISO-10303-21;"));
    let _ = std::fs::remove_dir_all(&dir);
}

/// A block with a bore through it, `body`, built of `block`: two solids to write.
const BORED: &str = "\
unit mm
use std
in std.front {
o := point
b := point
a := point
d := point
fix((0, 0)) o
fix((10, 0)) b
fix((10, 6)) a
fix((0, 6)) d
m := point
fix((5, 3)) m
c := circle(center: m) hint(r: 1)
radius(1) c
}
block := solid(face(o, b, a, d, -> close), depth: 3)
bore := solid(face(c), from: -5, to: 5)
body := solid(block)
bore cut body
";

/// **Several outputs, one build** (#592): two pictures, each with its own eye and size, and two
/// exports, each with its own solid — `body`'s STL and STEP beside each other — come out of one
/// run, and the body is built once for all of them: the pictures read the export's build, and
/// are what each draws alone, byte for byte.
#[test]
fn several_outputs_of_one_run_build_each_solid_once() {
    let dir = std::env::temp_dir().join(format!("solventc-outputs-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let model = dir.join("bored.sv");
    std::fs::write(&model,BORED).unwrap();
    let file = |name: &str| dir.join(name).to_string_lossy().into_owned();
    let (top,home,body_stl,body_step,block_stl) =
        (file("top.png"),file("home.png"),file("body.stl"),file("body.step"),file("block.stl"));
    let result = run(&[model.to_str().unwrap(),"--no-diagnose","--progress","--size","200",
        "--png",&top,"--view","top","--png",&home,"--size","120",
        "--stl",&body_stl,"--step",&body_step,"--solid","body","--stl",&block_stl,"--solid","block"]);
    let stderr = String::from_utf8_lossy(&result.stderr);
    assert_eq!(result.status.code(),Some(0),"{stderr}");
    let builds = stderr.lines().filter(|l| l.contains("built the solid by the Rust kernel")
        || l.contains("] built body in")).count();
    assert_eq!(builds,2,"the body and the block built once each for every output:\n{stderr}");
    assert!(!stderr.contains("] built body in"),"the pictures read the export's build:\n{stderr}");
    let alone = |name: &str,extra: &[&str]| {
        let png = file(name);
        let out = run(&[&[model.to_str().unwrap(),"--no-diagnose","--png",&png][..],extra].concat());
        assert_eq!(out.status.code(),Some(0),"{}",String::from_utf8_lossy(&out.stderr));
        std::fs::read(png).unwrap()
    };
    assert_eq!(std::fs::read(&top).unwrap(),alone("top_alone.png",&["--view","top","--size","200"]));
    assert_eq!(std::fs::read(&home).unwrap(),alone("home_alone.png",&["--size","120"]));
    let (top,home) = (read_png(&std::fs::read(&top).unwrap()),read_png(&std::fs::read(&home).unwrap()));
    assert_eq!(top.0.max(top.1),200,"the run's size where a picture is told none");
    assert_eq!(home.0.max(home.1),120,"a picture's own size");
    assert_ne!((top.0,top.1),(home.0,home.1),"each picture from its own eye");
    let (body,block) = (std::fs::read(&body_stl).unwrap(),std::fs::read(&block_stl).unwrap());
    gcs_core::mesh::stl_shells(&body).unwrap();
    gcs_core::mesh::stl_shells(&block).unwrap();
    assert_ne!(body,block,"each export its own solid");
    assert!(std::fs::read_to_string(&body_step).unwrap().starts_with("ISO-10303-21;"));
    std::fs::remove_dir_all(&dir).unwrap();
}

/// **A repeated flag is never silently dropped** (#592): an option that takes one value a run,
/// a qualifier given twice to one output, two outputs to one file and a qualifier with nothing to
/// qualify are each refused as a command line, before anything is read or written.
#[test]
fn a_repeated_option_or_output_is_refused() {
    for (args,why) in [
        (&["x.sv","--width","400","--width","800"][..],"`--width` given twice; it takes one value a run"),
        (&["x.sv","-o","a.svg","--output","b.svg"][..],"`--output` given twice; it takes one value a run"),
        (&["x.sv","--png","a.png","--png","a.png"][..],"two outputs write `a.png`"),
        (&["x.sv","--stl","a.stl","--solid","p","--stl","a.stl","--solid","q"][..],"two outputs write `a.stl`"),
        (&["x.sv","--png","a.png","--view","top","--view","left"][..],"`--view` given twice for `a.png`"),
        (&["x.sv","--stl","a.stl","--solid","p","--solid","q"][..],"`--solid` given twice for `a.stl`"),
        (&["x.sv","--solid","p","--solid","q","--stl","a.stl"][..],"`--solid` given twice before every output"),
        (&["x.sv","--stl","a.stl","--view","top"][..],"`--view` qualifies a --png, and none is given"),
        (&["x.sv","--solid","p"][..],"`--solid` says which solid an output writes, and none is given"),
    ] {
        let output = run(args);
        assert_eq!(output.status.code(),Some(64),"{args:?}");
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(stderr.starts_with(&format!("solventc: {why}\n")),"{args:?}: {stderr}");
    }
}

/// Two blocks three apart and a claim about their clearance, written to a file of its own.
fn clearance_document(tag: &str) -> (PathBuf,String) {
    let dir = std::env::temp_dir().join(format!("solventc-progress-{tag}-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let block = |t: &str,x: f64| format!(
        "in std.front {{\na{t} := point\nb{t} := point hint(({}, 0))\nc{t} := point hint(({}, 10))\n\
         d{t} := point hint(({x}, 10))\n(p{t} := line(a{t}, b{t})) -> (q{t} := line(b{t}, c{t})) -> \
         (r{t} := line(c{t}, d{t})) -> (s{t} := line(d{t}, a{t})) -> close\nhorizontal p{t}\nvertical q{t}\n\
         horizontal r{t}\nvertical s{t}\na{t} distance(10) b{t}\na{t} distance(10) d{t}\nfix(({x}, 0)) a{t}\n}}\n\
         f{t} := face(p{t}, q{t}, r{t}, s{t})\ns{t}_ := solid(f{t}, from: -5mm, to: 0mm)\n",x+10.,x+10.);
    let text = format!("unit mm\nuse std\n{}{}claim sA_ clear(2mm) sB_\n",block("A",0.),block("B",13.));
    let path = dir.join("blocks.sv");
    std::fs::write(&path,&text).unwrap();
    (path,text)
}

/// `--progress` says each stage on stderr as it is done, and `--timings` lists the solids and
/// claims after the report, each with the statement it came from (#320); unasked, a run whose
/// stderr is no terminal says nothing of the kind.
#[test]
fn a_run_says_its_stages_and_their_times() {
    let (path,text) = clearance_document("said");
    let file = path.to_str().unwrap();
    let result = run(&[file,"--progress","--timings"]);
    let (stdout,stderr) = (String::from_utf8_lossy(&result.stdout),String::from_utf8_lossy(&result.stderr));
    assert_eq!(result.status.code(),Some(0),"{stderr}");
    for stage in [format!("elaborated {file} in"),"solved for".into(),"built sA_ in".into(),
                  "judged claim sA_ clear(2mm) sB_ in".into(),format!("diagnosed {file} in")] {
        assert!(stderr.lines().any(|l| l.starts_with("solventc: [") && l.contains(&stage)),
            "`{stage}` said:\n{stderr}");
    }
    assert!(stdout.contains(&format!("{file}: solids and claims, slowest first")),"{stdout}");
    let claim = text.lines().position(|l| l.starts_with("claim")).unwrap()+1;
    assert!(stdout.lines().any(|l| l.contains("judged claim sA_ clear(2mm) sB_") && l.ends_with(&format!("{file}:{claim}:1"))),
        "the claim's line points at its statement:\n{stdout}");
    let quiet = run(&[file]);
    assert!(!String::from_utf8_lossy(&quiet.stderr).contains("solventc: ["),"unasked, a pipe hears nothing");
    std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
}

/// `--lint` says the drift lint's warnings in the core's words, and only when asked: advisory, so
/// a document that solves still exits 0.
#[test]
fn the_drift_lint_is_said_when_asked_and_fails_nothing() {
    let dir = std::env::temp_dir().join(format!("solventc-lint-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let source = dir.join("level.sv");
    std::fs::write(&source,"\
use std
component Level(a: point, b: point, run: Length) {
  a distance(run) b
}
in std.front {
  a := point
  b := point hint((10, 3))
  fix((0, 0)) a
  Level(a, b)
}
").unwrap();
    let path = source.to_str().unwrap();
    let plain = run(&[path,"--no-diagnose","--allow-unsolved"]);
    assert_eq!(plain.status.code(),Some(0),"{}",String::from_utf8_lossy(&plain.stderr));
    assert!(!String::from_utf8_lossy(&plain.stderr).contains("[W2"));
    let linted = run(&[path,"--no-diagnose","--allow-unsolved","--lint"]);
    assert_eq!(linted.status.code(),Some(0),"{}",String::from_utf8_lossy(&linted.stderr));
    let said = String::from_utf8_lossy(&linted.stderr);
    assert!(said.contains(":2:11: warning[W203]: `Level` declares nothing"),"{said}");
    assert!(said.contains(":9:3: warning[W202]: `Level(…)` stands as a statement"),"{said}");
    let json = run(&[path,"--no-diagnose","--allow-unsolved","--lint","--json"]);
    assert!(String::from_utf8_lossy(&json.stdout).contains("\"code\": \"W202\""));
    std::fs::remove_dir_all(dir).unwrap();
}

/// A scenario's diagnosis words (#510 P7) are the core's, said under the document's result: what
/// its supports leave free, and where a `safety` it states has no answer.  Words, not errors: the
/// document still exits 0.
#[test]
fn a_scenarios_words_are_said() {
    let dir = std::env::temp_dir().join(format!("solventc-scenario-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let source = dir.join("ell.sv");
    std::fs::write(&source,"\
unit mm
use std
in std.front {
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  b4 := point
  b5 := point
  fix((0, 0)) b0
  fix((60, 0)) b1
  fix((60, 20)) b2
  fix((20, 20)) b3
  fix((20, 40)) b4
  fix((0, 40)) b5
  (south := line(b0, b1)) -> (east := line(b1, b2)) -> (ledge := line(b2, b3)) -> \
(riser := line(b3, b4)) -> (top := line(b4, b5)) -> (west := line(b5, b0)) -> close
  outline := face(south, east, ledge, riser, top, west)
}
plate := solid(outline, depth: 4mm)
scenario lift {
  pinned b0
  claim safety(plate) >= 2
}
").unwrap();
    let path = source.to_str().unwrap();
    let out = run(&[path]);
    assert_eq!(out.status.code(),Some(0),"{}",String::from_utf8_lossy(&out.stderr));
    let said = String::from_utf8_lossy(&out.stdout);
    let free = "  scenario `lift` leaves `plate` free to turn about x through (0, 0, 0), turn \
about y through (0, 0, 0) and turn about z through (0, 0, 0)\n";
    assert!(said.contains(free),"{said}");
    let unbounded = "  scenario `lift`: `safety(plate)` has no answer: stress is unbounded at the \
edge of `ledge` and `riser`";
    assert!(said.contains(unbounded),"{said}");
    let json = String::from_utf8_lossy(&run(&[path,"--json"]).stdout).into_owned();
    assert!(json.contains("\"scenarios\"") && json.contains("leaves `plate` free"),"{json}");
    std::fs::remove_dir_all(dir).unwrap();
}

/// **A requirement out of reach is said with why** (#510 P7): the library's drilled plate loaded
/// ten times over sinks least with no hole, far more than its `sag`, and the drawing does not
/// solve; the summary says a requirement is out of reach, the row is said with the core's words
/// for why in place of a `conflict:` line, and `--json` gives them as `outOfReach`.
#[test]
fn a_requirement_out_of_reach_is_said() {
    let dir = std::env::temp_dir().join(format!("solventc-reach-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let source = dir.join("heavy.sv");
    let plate = std::fs::read_to_string(examples().join("drilled_plate.sv")).unwrap();
    assert!(plate.contains("P := 2kN"));
    std::fs::write(&source,plate.replace("P := 2kN","P := 20kN")).unwrap();
    let path = source.to_str().unwrap();
    let out = run(&[path,"--allow-unsolved"]);
    assert_eq!(out.status.code(),Some(0),"{}",String::from_utf8_lossy(&out.stderr));
    let said = String::from_utf8_lossy(&out.stdout);
    assert!(said.contains("did not solve"),"{said}");
    assert!(said.contains("CONFLICT — a requirement out of reach of what drives it"),"{said}");
    assert!(said.contains("  deflection(0.6, at: end_low, scenario: 0) down: out of reach of the \
dimensions that drive it: 4.79"),"{said}");
    assert!(!said.contains("  conflict: "),"{said}");
    let json = String::from_utf8_lossy(&run(&[path,"--json","--allow-unsolved"]).stdout).into_owned();
    assert!(json.contains("\"outOfReach\"") && json.contains("the least they reach here"),"{json}");
    std::fs::remove_dir_all(dir).unwrap();
}

/// **A frame's answers are said** (#510 S0): under each scenario, every requirement on a frame
/// with its value in the document's units, the error estimated beside it (#510 S3: nothing, at a
/// joint a cubic is exact for) and whether it keeps what it states — a refuted claim is a
/// report, so the document still exits 0 — and `--json` gives them as the scenario's
/// `answers`.
#[test]
fn a_frames_answers_are_said() {
    let dir = std::env::temp_dir().join(format!("solventc-frame-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let source = dir.join("arm.sv");
    std::fs::write(&source,"\
unit mm
use std
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
b := 25mm
h := 50mm
in std.front {
  foot := point
  end := point
  fix((0mm, 0mm)) foot
  fix((1000mm, 0mm)) end
  span := line(foot, end)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
arm := beam(span, A: b * h, Iy: h * b^3 / 12, Iz: b * h^3 / 12, J: 0.229 * h * b^3) of steel
scenario tip_load {
  fixed foot
  load end 1kN along: down
  claim deflection(end, along: down) <= 7mm
  claim deflection(end, along: down) <= 6mm
  deflection(end, along: down) <= 5mm
}
").unwrap();
    let path = source.to_str().unwrap();
    let out = run(&[path]);
    assert_eq!(out.status.code(),Some(0),"{}",String::from_utf8_lossy(&out.stderr));
    let said = String::from_utf8_lossy(&out.stdout);
    for line in [
        "  material steel: E 200GPa, nu 0.3, rho 7850kg/m^3, yield 250MPa, of arm\n",
        "  scenario `tip_load`: deflection(end, along: down) is 6.4mm ± 0mm: claimed <= 7mm, \
kept\n",
        "  scenario `tip_load`: deflection(end, along: down) is 6.4mm ± 0mm: claimed <= 6mm, \
refuted\n",
        "  scenario `tip_load`: deflection(end, along: down) is 6.4mm ± 0mm: stated <= 5mm, which \
does not hold; nothing drives it yet\n",
    ] {
        assert!(said.contains(line),"{line}\n{said}");
    }
    let json = String::from_utf8_lossy(&run(&[path,"--json"]).stdout).into_owned();
    assert!(json.contains("\"answers\"") && json.contains("\"grade\": \"claim\"")
            && json.contains("\"holds\": false"),"{json}");
    std::fs::remove_dir_all(dir).unwrap();
}

/// **A document's values are read** (#596): `--where` on a value says it in its unit, a claim
/// over values is judged before the solve and said with what it reads — a refuted one a report,
/// so the document still exits 0 — and `--json` gives both, as `values` and `valueClaims`.
#[test]
fn a_documents_values_are_read_and_claimed() {
    let dir = std::env::temp_dir().join(format!("solventc-values-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let source = dir.join("values.sv");
    std::fs::write(&source,"\
unit mm
use std
teeth := 48
ratio := teeth / 12 * 4
torque := 0.006Nm * ratio * 0.9
claim torque >= 2Nm
claim ratio == 16
in std.front {
  a := point
  fix((0, 0)) a
}
").unwrap();
    let path = source.to_str().unwrap();
    let out = run(&[path, "--where", "ratio", "--where", "torque"]);
    assert_eq!(out.status.code(),Some(0),"{}",String::from_utf8_lossy(&out.stderr));
    let said = String::from_utf8_lossy(&out.stdout);
    for line in [
        "  ratio = 16\n",
        "  torque = 0.0864 N·m\n",
        "  claim torque >= 2Nm — refuted: torque = 0.0864 N·m\n",
        "  claim ratio == 16 — theorem: ratio = 16\n",
    ] {
        assert!(said.contains(line),"{line}\n{said}");
    }
    assert!(!said.contains("names nothing there"),"{said}");
    let json = run(&[path,"--json","--where","torque"]).stdout;
    let json = String::from_utf8_lossy(&json).into_owned();
    assert!(json.contains("\"values\"") && json.contains("\"unit\": \"N·m\"")
            && !json.contains("\"teeth\"") && json.contains("\"valueClaims\"")
            && json.contains("\"holds\": false"),"{json}");
    std::fs::remove_dir_all(dir).unwrap();
}

/// **A driving deflection is said driven** (#510 S1): the library's sized cantilever solves to
/// the depth its tip sinks 4mm at, and `solventc` says the requirement is driven and held, and
/// the unknown it sized.
#[test]
fn a_driven_deflection_is_said() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../examples/sized_cantilever.sv");
    let out = run(&[path, "--where", "h"]);
    assert_eq!(out.status.code(),Some(0),"{}",String::from_utf8_lossy(&out.stderr));
    let said = String::from_utf8_lossy(&out.stdout);
    for line in [
        "  scenario `tip_load`: deflection(end, along: down) is 4mm ± 0mm: driven == 4mm, held\n",
        "DOF 0\n",
        "  h = 58.4804\n",
    ] {
        assert!(said.contains(line),"{line}\n{said}");
    }
}

/// A PNG `solventc --png` wrote, read back: its width, its height and its pixels, three bytes
/// each.  Only what the writer makes is read (8-bit truecolour, filter 0, fixed-code deflate
/// blocks), and the stream's own check is held to.
fn read_png(bytes: &[u8]) -> (usize, usize, Vec<u8>) {
    assert_eq!(&bytes[..8], b"\x89PNG\r\n\x1a\n", "a PNG's signature");
    let (mut at, mut size, mut idat) = (8, (0, 0), Vec::new());
    while at < bytes.len() {
        let n = u32::from_be_bytes(bytes[at..at + 4].try_into().unwrap()) as usize;
        let (kind, data) = (&bytes[at + 4..at + 8], &bytes[at + 8..at + 8 + n]);
        match kind {
            b"IHDR" => {
                let word = |k: usize| u32::from_be_bytes(data[k..k + 4].try_into().unwrap()) as usize;
                size = (word(0), word(4));
                assert_eq!(&data[8..], &[8, 2, 0, 0, 0], "8-bit truecolour, not interlaced");
            }
            b"IDAT" => idat.extend_from_slice(data),
            _ => {}
        }
        at += 12 + n;
    }
    let raw = inflate(&idat[2..idat.len() - 4]);
    let (mut a, mut b) = (1u32, 0u32);
    for &x in &raw {
        a = (a + x as u32) % 65521;
        b = (b + a) % 65521;
    }
    assert_eq!(((b << 16) | a).to_be_bytes(), idat[idat.len() - 4..], "the zlib stream's Adler-32");
    let (w, h) = size;
    assert_eq!(raw.len(), h * (3 * w + 1), "a scanline a row, each led by its filter");
    let mut rgb = Vec::with_capacity(3 * w * h);
    for row in raw.chunks_exact(3 * w + 1) {
        assert_eq!(row[0], 0, "filter 0");
        rgb.extend_from_slice(&row[1..]);
    }
    (w, h, rgb)
}

/// Deflate's fixed-code blocks (RFC 1951), which are all `--png` writes.
fn inflate(data: &[u8]) -> Vec<u8> {
    let mut bit = 0usize;
    let mut take = |n: usize| -> usize {
        let mut v = 0;
        for k in 0..n {
            v |= ((data[bit / 8] >> (bit % 8)) as usize & 1) << k;
            bit += 1;
        }
        v
    };
    const LENGTH: [(usize, usize); 29] = [(3, 0), (4, 0), (5, 0), (6, 0), (7, 0), (8, 0), (9, 0),
        (10, 0), (11, 1), (13, 1), (15, 1), (17, 1), (19, 2), (23, 2), (27, 2), (31, 2), (35, 3),
        (43, 3), (51, 3), (59, 3), (67, 4), (83, 4), (99, 4), (115, 4), (131, 5), (163, 5),
        (195, 5), (227, 5), (258, 0)];
    let mut out = Vec::new();
    loop {
        let last = take(1) == 1;
        match take(2) {
            1 => loop {
                // a code is read its most significant bit first
                let mut code = 0;
                let mut symbol = None;
                for len in 1..=9 {
                    code = code << 1 | take(1);
                    symbol = match (len, code) {
                        (7, 0..=23) => Some(256 + code),
                        (8, 0x30..=0xbf) => Some(code - 0x30),
                        (8, 0xc0..=0xc7) => Some(280 + code - 0xc0),
                        (9, 0x190..=0x1ff) => Some(144 + code - 0x190),
                        _ => None,
                    };
                    if symbol.is_some() { break; }
                }
                match symbol.expect("a fixed code") {
                    s @ 0..=255 => out.push(s as u8),
                    256 => break,
                    s => {
                        let (base, extra) = LENGTH[s - 257];
                        let len = base + take(extra);
                        let k = (0..5).fold(0, |c, _| c << 1 | take(1));
                        let (base, extra) = if k < 4 { (k + 1, 0) } else {
                            let extra = k / 2 - 1;
                            ((2 + (k & 1)) << extra | 1, extra)
                        };
                        let d = base + take(extra);
                        for _ in 0..len {
                            out.push(out[out.len() - d]);
                        }
                    }
                }
            },
            t => panic!("--png writes no deflate block of type {t}"),
        }
        if last { return out; }
    }
}

/// The colour at the middle of a picture.
fn middle((w, h, rgb): &(usize, usize, Vec<u8>)) -> [u8; 3] {
    let k = 3 * (h / 2 * w + w / 2);
    [rgb[k], rgb[k + 1], rgb[k + 2]]
}

/// Two prisms on one axis, the narrow one standing out of the wide one's top.
const NESTED: &str = "\
unit mm
use std
in std.top {
  hub := point
  fix((0mm, 0mm)) hub
  wide_rim := radius(30mm) circle(center: hub)
  narrow_rim := radius(8mm) circle(center: hub)
  wide := solid(face(wide_rim), from: -5mm, to: 5mm)
  narrow := solid(face(narrow_rim), from: 0mm, to: 20mm)
}
";

/// `--png` keeps, at each pixel, what is nearest the eye (#541): from above, the narrow prism at
/// the middle, drawn as it is drawn alone; from below, the wide one.  Which is drawn first does
/// not decide it, so a painter's order cannot pass both.  A part keeps its instance's colour in
/// a cutaway, and the picture is fitted to what it draws, its longer side `--size`.
#[test]
fn a_png_keeps_what_is_nearest_the_eye() {
    let dir = std::env::temp_dir().join(format!("solventc-png-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let source = dir.join("nested.sv");
    std::fs::write(&source, NESTED).unwrap();
    let draw = |name: &str, extra: &[&str]| {
        let png = dir.join(format!("{name}.png"));
        let mut args = vec![source.to_str().unwrap(), "--no-diagnose", "--size", "300", "--png",
                            png.to_str().unwrap()];
        args.extend_from_slice(extra);
        let out = run(&args);
        assert_eq!(out.status.code(), Some(0), "{}", String::from_utf8_lossy(&out.stderr));
        read_png(&std::fs::read(png).unwrap())
    };
    let above = draw("above", &["--view", "top"]);
    assert_eq!((above.0, above.1), (300, 300), "a round part seen square on is a square picture");
    let below = draw("below", &["--view", "-90,-90"]);
    let narrow = middle(&draw("narrow", &["--view", "top", "--solid", "narrow"]));
    let wide_below = middle(&draw("wide_below", &["--view", "-90,-90", "--solid", "wide"]));
    let wide_above = middle(&draw("wide_above", &["--view", "top", "--solid", "wide"]));
    let paper = [250, 250, 250];
    assert!(narrow != paper && wide_below != paper && narrow != wide_below);
    assert_eq!(middle(&above), narrow, "from above, the narrow prism is nearest");
    assert_eq!(middle(&below), wide_below, "from below, the wide prism is nearest");
    assert_eq!(middle(&draw("cutaway", &["--view", "top", "--hide", "narrow"])), wide_above,
               "hidden, the narrow prism lets the wide one through, its colour its own");
    // an edge is drawn round the wide prism's rim: dark ink across a row through the middle
    let (w, h, rgb) = &above;
    let row = &rgb[3 * w * (h / 2)..3 * w * (h / 2 + 1)];
    assert!(row.as_chunks::<3>().0.iter().any(|p| p.iter().all(|&c| c < 120)), "an edge at the rim");
    std::fs::remove_dir_all(dir).unwrap();
}

/// A part file draws its `preview` (#541): the engine's head opened alone, about the world's
/// origin, as the workspace opens it.
#[test]
fn a_part_file_draws_its_preview() {
    let dir = std::env::temp_dir().join(format!("solventc-png-part-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let png = dir.join("head.png");
    let out = run(&[&doc("engine/head.sv"), "--no-diagnose", "--size", "240", "--png",
                    png.to_str().unwrap()]);
    assert_eq!(out.status.code(), Some(0), "{}", String::from_utf8_lossy(&out.stderr));
    let (w, h, rgb) = read_png(&std::fs::read(&png).unwrap());
    assert_eq!(w.max(h), 240);
    let inked = rgb.as_chunks::<3>().0.iter().filter(|p| **p != [250, 250, 250]).count();
    assert!(inked > w * h / 4, "the head fills its picture: {inked} of {} pixels", w * h);
    std::fs::remove_dir_all(dir).unwrap();
}

/// What `--png` refuses: an eye it cannot read, a name to hide that names nothing, and a drawing
/// rather than a model.
#[test]
fn a_png_is_refused_what_it_cannot_draw() {
    let out = run(&[&doc("solid_pulley.sv"), "--png", "x.png", "--view", "askew"]);
    assert_eq!(out.status.code(), Some(64));
    assert!(String::from_utf8_lossy(&out.stderr).contains("--view needs front, back, right, left, top, bottom, isometric"));
    let dir = std::env::temp_dir().join(format!("solventc-png-refused-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let png = dir.join("p.png");
    let out = run(&[&doc("solid_pulley.sv"), "--no-diagnose", "--png", png.to_str().unwrap(),
                    "--hide", "nothing_here"]);
    assert_eq!(out.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&out.stderr).contains("no solid called `nothing_here` to hide"));
    assert!(!png.exists(), "nothing is written");
    let out = run(&[&doc("solid_pulley.svd"), "--png", png.to_str().unwrap()]);
    assert_eq!(out.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&out.stderr).contains("PNG export selects geometry from a .sv model"));
    std::fs::remove_dir_all(dir).unwrap();
}

/// A star handle swept through a wheel's place, and a block beside it (#614): the star's round
/// tips and fillets leave the sweep's field a crease the mesher cannot protect.
const STAR_WHEEL: &str = "\
unit mm
use std
use std (vertical, turns)

interface StarSizes {
  span: Length, waist: Length, tip_r: Length, fillet_r: Length
}

component StarOutline(top: plane, s: StarSizes) {
  in top {
    hub := point
    hub coincident top.origin
    ring 4 about hub {
      tip_c := point hint((s.span / 2 - s.tip_r, 0))
      fillet_c := point hint(((s.waist / 2 + s.fillet_r) / sqrt(2),
                              (s.waist / 2 + s.fillet_r) / sqrt(2)))
      apex := point hint((s.span / 2, 0))
      hub distance(s.waist / 2 + s.fillet_r) fillet_c
      line(fillet_c, tip_c) equal line(fillet_c, next.tip_c)
      ccw(hub, tip_c, fillet_c)
      apex coincident(t == s.span / (s.span - 2 * s.tip_r)) line(hub, tip_c)
      p1 := point hint((tip_c.x, s.tip_r))
      p2 := point hint((fillet_c.x - 0.2 * s.fillet_r, fillet_c.y - 0.98 * s.fillet_r))
      p3 := point hint((fillet_c.x - 0.98 * s.fillet_r, fillet_c.y - 0.2 * s.fillet_r))
      p4 := point hint((s.tip_r, tip_c.x))
      tip_half := arc(center: tip_c, start: apex, end: p1)
      side := line(p1, p2)
      fillet := arc(center: fillet_c, start: p3, end: p2)
      next_side := line(p3, p4)
      next_tip_half := arc(center: next.tip_c, start: p4, end: next.apex)
      radius(s.fillet_r) fillet
      side tangent(at: p1) tip_half
      side tangent(at: p2) fillet
      next_side tangent(at: p1) fillet
      next_side tangent(at: p2) next_tip_half
      ccw(p1, p2, tip_c)
      cw(p1, p2, fillet_c)
      ccw(p3, p4, next.tip_c)
      cw(p3, p4, fillet_c)
    }
    hub distance(s.span / 2 - s.tip_r, along: u) tip_c[0]
    hub level(v) tip_c[0]
    outline := face(tip_half[0], side[0], fillet[0], next_side[0], next_tip_half[0],
                    tip_half[1], side[1], fillet[1], next_side[1], next_tip_half[1],
                    tip_half[2], side[2], fillet[2], next_side[2], next_tip_half[2],
                    tip_half[3], side[3], fillet[3], next_side[3], next_tip_half[3])
  }
}

in std.front {
  construction stem := vertical line(std.origin, hint((0, 10)))
  length(10mm) stem
  wc := point hint((-60, 0))
  std.origin distance(-60mm, along: u) wc
  construction spindle := vertical line(wc, hint((-60, 10)))
  length(10mm) spindle
}
top := plane
fix(origin == (0mm, 0mm, -1.1mm)) top
fix(dir == (1, 0, 0)) top.u
fix(dir == (0, 1, 0)) top.v
plan := StarOutline(top, s: {span: 74.2mm, waist: 25.8mm, tip_r: 3.25mm, fillet_r: 4mm})
star := solid(plan.outline, from: -16mm, to: 0mm)
handle_turn := frame
handle_turn turns(1) stem
wheel_turn := frame
wheel_turn turns(-4 / 6) spindle
space := sweep(star over t in (0deg, 540deg), under: handle_turn, seen_from: wheel_turn)
in std.top {
  bc := point hint((-60, 70))
  fix((-60mm, 70mm)) bc
  bsq := std.CenteredRectangle(bc, w: 10mm, h: 10mm)
}
block := solid(face(bsq.loop), from: 0mm, to: 10mm)
";

/// The wheel the star's sweep cuts, of the planar subtraction class: its field fails the mesher
/// as the sweep's does, and the kernel builds it exactly in a tenth of a second.
const STAR_WHEEL_CUT: &str = "\
in std.top {
  wo := point
  wo coincident wc
  rim := radius(46.1mm) circle(center: wo)
}
wheel := solid(face(rim), from: -15.6mm, to: -0.6mm)
space cut wheel
";

/// **A swept object is pictured from its exact build** (#614), as the workspace replaces its
/// field's preview with it: the star wheel, whose field the mesher cannot close, is drawn alone
/// and beside the block, each in its own colour, and the run exits 0.
#[test]
fn a_swept_object_is_pictured_from_its_exact_build() {
    let dir = std::env::temp_dir().join(format!("solventc-png-swept-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let source = dir.join("star_wheel.sv");
    std::fs::write(&source, format!("{STAR_WHEEL}{STAR_WHEEL_CUT}")).unwrap();
    let draw = |name: &str, extra: &[&str]| {
        let png = dir.join(format!("{name}.png"));
        let mut args = vec![source.to_str().unwrap(), "--no-diagnose", "--no-progress", "--size",
                            "240", "--view", "top", "--png", png.to_str().unwrap()];
        args.extend_from_slice(extra);
        let out = run(&args);
        let stderr = String::from_utf8_lossy(&out.stderr);
        assert_eq!(out.status.code(), Some(0), "{name}: {stderr}");
        assert!(!stderr.contains("could not be protected"), "{name}: {stderr}");
        read_png(&std::fs::read(png).unwrap())
    };
    let (wheel, block) = (middle(&draw("wheel", &["--solid", "wheel"])),
                          middle(&draw("block", &["--solid", "block"])));
    let paper = [250, 250, 250];
    assert!(wheel != paper && block != paper && wheel != block, "{wheel:?} {block:?}");
    let (_, _, rgb) = draw("both", &[]);
    let count = |c: [u8; 3]| rgb.as_chunks::<3>().0.iter().filter(|p| **p == c).count();
    assert!(count(wheel) > 4 * count(block) && count(block) > 0,
            "the wheel drawn beside the block: {} of the wheel's pixels, {} of the block's",
            count(wheel), count(block));
    std::fs::remove_dir_all(dir).unwrap();
}

/// **A picture missing a solid fails, naming it** (#614): the star's sweep standing as an object
/// of its own, which no kernel builds exactly and whose field the mesher cannot close, is left
/// out of the picture — said against the picture with its solid's name, the rest still drawn and
/// written, and the run exits 1, so a picture with a part missing is never taken for the whole.
#[test]
fn a_picture_missing_a_solid_fails_naming_it() {
    let dir = std::env::temp_dir().join(format!("solventc-png-missing-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let source = dir.join("star.sv");
    std::fs::write(&source, STAR_WHEEL).unwrap();
    let png = dir.join("p.png");
    let path = png.to_str().unwrap();
    let out = run(&[source.to_str().unwrap(), "--no-diagnose", "--no-progress", "--size", "120",
                    "--png", path]);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert_eq!(out.status.code(), Some(1), "{stderr}");
    assert!(stderr.contains(&format!("solventc: {path}: `space`: ")), "{stderr}");
    let (_, _, rgb) = read_png(&std::fs::read(&png).unwrap());
    assert!(rgb.as_chunks::<3>().0.iter().any(|p| *p != [250, 250, 250]), "the block is drawn");
    std::fs::remove_dir_all(dir).unwrap();
}

/// **A solid read in its plane is said** (#510 S2): the corpus's Kirsch plate, `plane stress
/// plate` in its scenario, has its factor of safety said under the scenario — kept — and a
/// buckling asked of it says why it has none; the document still exits 0.
#[test]
fn a_plane_reading_is_said() {
    let dir = std::env::temp_dir().join(format!("solventc-plane-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let source = dir.join("kirsch.sv");
    let kirsch = include_str!("../../examples/analysis/kirsch.sv")
        .replace("  claim safety(plate) <= 1.03 * steel.yield / (kt * sigma)\n",
                 "  claim safety(plate) <= 1.03 * steel.yield / (kt * sigma)\n  \
                  claim buckling(plate) >= 1\n");
    std::fs::write(&source,kirsch).unwrap();
    let path = source.to_str().unwrap();
    let out = run(&[path]);
    assert_eq!(out.status.code(),Some(0),"{}",String::from_utf8_lossy(&out.stderr));
    let said = String::from_utf8_lossy(&out.stdout);
    let safety = said.lines().find(|l| l.starts_with("  scenario `pull`: safety(plate) is "));
    assert!(safety.is_some_and(|l| l.ends_with(", kept")), "{said}");
    assert!(said.contains("  scenario `pull`: buckling(plate) has no answer: `plate` is read in \
its plane, and a plate buckles out of its plane, which it has no move for\n"),"{said}");
    std::fs::remove_dir_all(dir).unwrap();
}

/// **A solid read in three dimensions is said** (#510 S3): Euler's column of the corpus, its
/// buckling factor with the error estimated beside it, each claim kept; `--json` gives each
/// answer's `error`.
#[test]
fn a_solid_reading_is_said() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../examples/analysis/column.sv");
    let out = run(&[path]);
    assert_eq!(out.status.code(),Some(0),"{}",String::from_utf8_lossy(&out.stderr));
    let said = String::from_utf8_lossy(&out.stdout);
    let lines: Vec<&str> =
        said.lines().filter(|l| l.starts_with("  scenario `press`: buckling(strut) is ")).collect();
    assert_eq!(lines.len(), 2, "{said}");
    assert!(lines.iter().all(|l| l.contains(" ± ") && l.ends_with(", kept")), "{said}");
    let json = String::from_utf8_lossy(&run(&[path,"--json"]).stdout).into_owned();
    assert!(json.contains("\"error\": ") && !json.contains("\"error\": null"),"{json}");
}
