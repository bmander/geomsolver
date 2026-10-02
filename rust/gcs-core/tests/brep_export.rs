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

/// The page's exact surface of a swept body (`export::Builder`), built a stage at a time — each
/// stage saying what the next does, as a worker shows it — and supplied to a sketch in place of
/// the field's surface: its volume the B-rep's, its faces meeting at creases.
#[test]
fn a_swept_bodys_exact_surface_is_built_by_stages_and_supplied() {
    let sk = example("swept_torus.sv");
    let body = (0..sk.solids.len()).find(|&i| sk.solids[i].name == "part").unwrap();
    let mut builder = export::Builder::new(&sk,body).unwrap();
    let mut doing = vec![builder.doing()];
    let (_,total) = builder.stages();
    while !builder.step().unwrap() { doing.push(builder.doing()); }
    assert_eq!(builder.stages(),(total,total),"{doing:?}");
    assert_eq!(doing[0],"admitting its sweeps to the generating class");
    assert!(doing.iter().any(|d| d.starts_with("fitting the sheet of `removal`")),"{doing:?}");
    assert!(doing.iter().any(|d| d == "cutting the blank by its sheets"),"{doing:?}");
    assert!(builder.said().is_some());
    let d = builder.display().unwrap().clone();
    assert_eq!(d.of.len(),d.triangles.len());
    assert!((d.volume-0.447407).abs() < 2e-5,"{}",d.volume);
    // supplied in place of the field's surface, the sketch draws it and never meshes the field
    sk.field_meshing.set(gcs_core::solid::FieldMeshing::Deferred);
    let exact = gcs_core::solid::ExactFaces {of:d.of.clone(),smooth:d.smooth.clone(),volume:d.volume};
    let surface = gcs_core::solid::FieldSurface {vertices:d.vertices.clone(),triangles:d.triangles.clone(),provisional:false,exact:Some(exact)};
    sk.supply_field(body,surface).unwrap();
    let solid = sk.evaluated_solid(body,gcs_core::solid::ApproximationPolicy::Mesh).unwrap();
    assert!(!solid.provisional());
    assert_eq!(solid.volume(),d.volume);
    assert!(solid.surviving_faces().contains("part.surface"));
    // its faces meet at creases (the cut's rim on the post), and a curved face's seams are smooth
    assert!(solid.edges().iter().any(|e| !e.smooth));
    assert!(solid.edges().iter().any(|e| e.smooth));
    // a provisional exact surface is refused
    let bad = gcs_core::solid::FieldSurface {provisional:true,..sk.supplied_field(body).unwrap().as_ref().clone()};
    assert!(sk.supply_field(body,bad).unwrap_err().contains("never provisional"));
}
