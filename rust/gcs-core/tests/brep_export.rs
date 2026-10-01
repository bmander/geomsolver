//! A solid's exact export by this kernel (`brep::export`, phase 5 of docs/rust-kernel-plan.md), the
//! functions `solventc` and the browser's worker both call: a body whose swept cut is placed once is
//! no sector, and is built whole — the blank split by the placed sheet, its cells judged by the
//! material field — then written, parsed back, meshed and held to the field.
use gcs_core::brep::{export,props,sweep::Say};
use gcs_core::model::Sketch;

fn example(name: &str) -> Sketch {
    let (_,e) = fixtures::examples().into_iter().find(|(n,_)| n == name).unwrap();
    let mut sk = e.unwrap().sketch;
    gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
    sk
}

#[test]
fn a_sweep_placed_once_is_built_whole_and_exported() {
    let sk = example("swept_torus.sv");
    let body = (0..sk.solids.len()).find(|&i| sk.solids[i].name == "part").unwrap();
    let lines = std::sync::Mutex::new(Vec::new());
    let say = Say {stage:&|l: &str| lines.lock().unwrap().push(l.to_string()),mark:&|_| {}};
    let exact = export::exact(&sk,body,None,None,&say).unwrap();
    let said = lines.lock().unwrap().join("\n");
    assert!(exact.swept && exact.pattern.is_none() && exact.indexed.is_none(),"{said}");
    assert!(said.contains("built whole, not as one sector: the sweep is placed 1 times"),"{said}");
    assert!(said.contains("classified 1 material and 1 removed cells"),"{said}");
    exact.solid.check(1e-6).unwrap();
    // the blank (0.753982 mm³) less the removal, which OCCT's reading of the file measures as
    // 0.447407 mm³ at 0.1 µm
    let volume = props::volume(&exact.solid);
    assert!((volume-0.447407).abs() < 2e-5,"{volume}");
    let step = export::step(&exact,"part",None,&say).unwrap();
    assert!(step.starts_with("ISO-10303-21;"));
    let stl = export::stl(&sk,body,&exact,None,&say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
    let said = lines.lock().unwrap().join("\n");
    assert!(said.contains("0 disagree"),"{said}");
}
