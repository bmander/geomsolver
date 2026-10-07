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
fn unknown_stl_backend_is_refused() {
    let output = run(&["--stl-backend","unknown"]);
    assert_eq!(output.status.code(),Some(2));
    assert!(String::from_utf8_lossy(&output.stderr).contains("occt, mesh or refine"));
}

#[test]
fn a_continuous_boundary_exports_through_the_field_mesh() {
    let dir = std::env::temp_dir().join(format!("solventc-sweep-mesh-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let output = dir.join("swept.stl");
    let result = run(&[&doc("solid_generating_sweep.sv"),"--stl",output.to_str().unwrap(),
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
    fixtures::gear::copy_design(&dir,30.,0.,35.);
    let output = dir.join("pinion.stl");
    std::fs::write(&output,"old STL").unwrap();
    let result = run(&[dir.join("gears.sv").to_str().unwrap(),"--stl",output.to_str().unwrap(),
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

/// The fillet examples (issue #66): a cast pad (a boss's root filleted, two plate edges rounded, a
/// bore drilled after), a knob (a rod's root and its neck in a ball), a rail (a rod half sunk in a
/// plate, filleted along both sides), a pipe tee's crotch and a bore's rim (balls rolled round traced
/// loops, their faces fitted), each written by the core's kernel within 10 µm and its STL measured
/// against its exact faces, the fillets' among them.
#[test]
fn the_filleted_examples_export_and_measure_within_their_tolerance() {
    let dir = std::env::temp_dir().join(format!("solventc-fillet-{}",std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    for (file,solid,built) in [("solid_fillet.sv","pad","26549.817129 mm³, 12 faces"),
        ("solid_fillet_knob.sv","knob","26224.398265 mm³, 7 faces"),("solid_fillet_rail.sv","rail","26306.865907 mm³, 18 faces"),
        ("solid_fillet_tee.sv","tee","20616.666786 mm³, 6 faces"),("solid_fillet_bore.sv","pipe","17747.246919 mm³, 6 faces")] {
        let (stl,step) = (dir.join(format!("{solid}.stl")),dir.join(format!("{solid}.step")));
        let (stl,step) = (stl.to_str().unwrap(),step.to_str().unwrap());
        let source = doc(file);
        let result = run(&[&source,"--solid",solid,"--stl",stl,"--step",step,"--tolerance","0.01mm","--no-diagnose"]);
        let stderr = String::from_utf8_lossy(&result.stderr);
        assert_eq!(result.status.code(),Some(0),"{file}: {stderr}");
        assert!(stderr.contains(built),"{file}: {stderr}");
        let measured = run(&[&source,"--solid",solid,"--measure",stl,"--tolerance","0.01mm","--no-diagnose"]);
        let stdout = String::from_utf8_lossy(&measured.stdout);
        assert_eq!(measured.status.code(),Some(0),"{file}: {stdout}{}",String::from_utf8_lossy(&measured.stderr));
        assert!(stdout.contains("tolerance 10.000 µm: every exact face within it"),"{file}: {stdout}");
        assert!(stdout.contains("  round "),"{file}: the fillets' faces are measured: {stdout}");
    }
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn a_tolerance_is_a_positive_length_for_a_native_export() {
    let output = run(&[&doc("swept_torus.sv"),"--tolerance","3furlongs","--step","x.step"]);
    assert_eq!(output.status.code(),Some(2));
    assert!(String::from_utf8_lossy(&output.stderr).contains("`furlongs` is not a length unit"));
    let output = run(&[&doc("swept_torus.sv"),"--tolerance","0mm","--step","x.step"]);
    assert_eq!(output.status.code(),Some(2));
    assert!(String::from_utf8_lossy(&output.stderr).contains("a tolerance is a positive length"));
    // It holds an export or a measurement to it, and means nothing to a document only checked.
    let output = run(&[&doc("swept_torus.sv"),"--tolerance","--no-diagnose"]);
    assert_eq!(output.status.code(),Some(2));
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
    let result = run(&[&doc("spiral_bevel/gears.sv"),"--solid",&body,"--stl",stl,"--step",step,"--tolerance","0.01mm",
        "--verify-step","full","--no-diagnose"]);
    let stderr = String::from_utf8_lossy(&result.stderr);
    assert_eq!(result.status.code(),Some(0),"{stderr}");
    assert!(stderr.contains("each the solid's, and read back by the kernel as"),"{stderr}");
    assert!(stderr.contains("probed 0.0200 mm off each side") && stderr.contains("0 disagree"),"{stderr}");
    gcs_core::mesh::stl_shells(&std::fs::read(stl).unwrap()).unwrap();
    for file in [step,stl] {
        let measured = run(&[&doc("spiral_bevel/gears.sv"),"--solid",&body,"--measure",file,"--tolerance","0.01mm","--no-diagnose"]);
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
        a := point\nfix(x == 0, y == 0) a\nb := point\nfix(x == w, y == 0) b\n\
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
    assert_eq!(two.status.code(), Some(2));
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
    std::fs::write(&model,"\
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
").unwrap();
    let result = run(&[model.to_str().unwrap(),"--solid","body","--kernel","rust","--stl",stl.to_str().unwrap(),
        "--step",step.to_str().unwrap(),"--no-diagnose"]);
    let stderr = String::from_utf8_lossy(&result.stderr);
    assert_eq!(result.status.code(),Some(0),"{stderr}");
    assert!(stderr.contains("built the solid by the Rust kernel") && stderr.contains("each the solid's"),"{stderr}");
    gcs_core::mesh::stl_shells(&std::fs::read(&stl).unwrap()).unwrap();
    assert!(std::fs::read_to_string(&step).unwrap().starts_with("ISO-10303-21;"));
    let _ = std::fs::remove_dir_all(&dir);
}
