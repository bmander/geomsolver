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

/// A 2.03 × 2 × 2 mm box whose near face stands at x = `x0` mm (issue #60).
fn far_box(x0: f64) -> Sketch {
    let x1 = x0+2.03;
    let src = format!("unit mm
p0 := point
fix(x == {x0}, y == 0) p0
p1 := point
fix(x == {x1}, y == 0) p1
p2 := point
fix(x == {x1}, y == 2) p2
p3 := point
fix(x == {x0}, y == 2) p3
section := face(p0, p1, p2, p3, -> close)
body := solid(section, from: 0mm, to: 2mm)
");
    let (p,errors) = gcs_core::syntax::parse(&src);
    assert!(errors.is_empty(),"{errors:?}");
    let e = gcs_core::program::elaborate(&p);
    assert!(e.ok(),"{:?}",e.diags);
    e.sketch
}

/// The x extent of a binary STL's written coordinates, decoded from its bytes.
fn written_x(stl: &[u8]) -> (f64,f64) {
    let n = u32::from_le_bytes(stl[80..84].try_into().unwrap()) as usize;
    let xs = (0..n).flat_map(|i| (0..3).map(move |k| {
        let at = 84+50*i+12+12*k;
        f32::from_le_bytes(stl[at..at+4].try_into().unwrap()) as f64
    }));
    xs.fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),x| (lo.min(x),hi.max(x)))
}

/// The tolerance is about the surface written, and binary STL writes float32: a box far from the
/// origin, whose mesh sags nothing, is moved by the encoding alone (30 µm at x = 10⁶ mm). The
/// rounding is counted with the sag, so an STL is written within its share of the tolerance —
/// its faces read back from the bytes where the model has them — or refused, and never given
/// back moved further.
#[test]
fn float32_rounding_is_spent_from_the_tolerance_of_the_written_stl() {
    let say = Say {stage:&|_| {},mark:&|_| {}};
    for (x0,tolerance,written) in [(0.03,0.001,true),(100_000.003,0.001,false),(100_000.003,0.01,true),
        (1_000_000.03,0.001,false),(1_000_000.03,0.01,false)] {
        let sk = far_box(x0);
        let tolerance = gcs_core::solid::export::Tolerance::new(tolerance).unwrap();
        let exact = export::exact(&sk,0,None,Some(tolerance),&say).unwrap();
        let what = format!("x = {x0} mm within {} µm",tolerance.millimetres*1e3);
        match export::stl(&sk,0,&exact,Some(tolerance),&say) {
            Ok(stl) => {
                assert!(written,"{what}: written");
                let rounding = gcs_core::mesh::stl_rounding(&stl).unwrap();
                assert!(rounding <= tolerance.deflection(),"{what}: {rounding}");
                let (lo,hi) = written_x(&stl);
                let off = (lo-x0).abs().max((hi-(x0+2.03)).abs());
                assert!(off <= tolerance.deflection(),"{what}: the faces written at {lo} and {hi}, {off} mm off");
            }
            Err(e) => {
                assert!(!written,"{what}: {e}");
                assert_eq!(e.stage,gcs_core::solid::export::Stage::Stl,"{what}: {e}");
                assert!(e.message.contains("float32"),"{what}: {e}");
            }
        }
    }
    // the bound is half a float32 step: 1/32 mm on one axis at 10⁶ mm, nothing at the origin
    let one = |p: [f64;3]| gcs_core::mesh::stl_rounding(&gcs_core::mesh::stl_of(&[p,[p[0]+1.,p[1],p[2]],[p[0],p[1]+1.,p[2]]]
        .concat(),"t")).unwrap();
    assert!((one([1_000_000.03,0.,0.])-1./32.).abs() < 1e-9);
    assert!(one([0.,0.,0.]) < 1e-7);
}
