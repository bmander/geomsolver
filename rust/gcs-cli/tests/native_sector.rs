//! The sector construction (docs/native-speed-plan.md) against the whole one, on a small indexed
//! fixture: a ring about the spindle cut by a unit sphere rolled about a parallel cradle, turned to
//! several places — a gear's shape in miniature, its cuts leaving a gap between neighbours. Built
//! as one sector patterned, the body is the body built whole: its volume, its faces, a closed mesh.
#[path="../src/cad/native.rs"]
#[allow(dead_code)]
mod native;
#[path="../src/cad/progress.rs"]
#[allow(dead_code)]
mod progress;
use native::sweep_boundary::{construct_built,construct_solid_as,Construction};
use fixtures::{tools::{indexed_ring,sphere},motions::{Observer,cradle_roll}};
use gcs_core::solid::{admission,cad};

fn ring(count: usize) -> gcs_core::program::Elaborated {
    fixtures::read(&format!("{}{}construction removal := solid(tool, under: turn, from: -35deg, to: 35deg)\n{}",sphere(2.),
        cradle_roll(2.5,Observer::Parallel),indexed_ring(count,3.5,4.2,1.7,2.3)))
}

/// The body built as `asked`: how it was built, its volume, its faces, and its mesh checked closed.
fn built(e: &gcs_core::program::Elaborated,asked: Construction) -> (Construction,f64,usize) {
    let body = fixtures::solid(e,"part");
    let recipe = cad::recipe_static(&e.sketch,body).unwrap();
    let admitted = admission::admit_body(&e.sketch,body,&admission::Options::default()).unwrap();
    let session = native::Session::new().unwrap();
    let (part,how) = construct_solid_as(&session,&e.sketch,body,&recipe,Some(&admitted),None,asked).unwrap();
    let stl = std::env::temp_dir().join(format!("solvent-sector-{}-{asked:?}.stl",std::process::id()));
    session.stl(part,stl.to_str().unwrap()).unwrap();
    gcs_core::mesh::stl_shells(&std::fs::read(&stl).unwrap()).unwrap();
    let _ = std::fs::remove_file(&stl);
    (how,session.volume(part).unwrap(),session.faces(part).unwrap().len())
}

#[test]
fn an_indexed_ring_built_as_one_sector_is_the_ring_built_whole() {
    let e = ring(6);
    let (how,volume,faces) = built(&e,Construction::Sector);
    let (whole,expected,expected_faces) = built(&e,Construction::Whole);
    eprintln!("sector: {volume:.9} mm³, {faces} faces; whole: {expected:.9} mm³, {expected_faces} faces");
    assert_eq!((how,whole),(Construction::Sector,Construction::Whole));
    assert!((volume-expected).abs() <= 1e-6*expected,"{volume} against {expected}");
    assert_eq!(faces,expected_faces);
}

/// Cut twice, the ring has no sector worth building (its halves could not be told apart by their
/// volume): it is built whole, and is what the whole construction builds.
#[test]
fn a_ring_cut_twice_is_built_whole() {
    let e = ring(2);
    let (how,..) = built(&e,Construction::Sector);
    assert_eq!(how,Construction::Whole);
}

/// Meshed as its sector turned into every copy (`Session::sector_mesh`, `sector_stl`), the ring is
/// one closed shell, whose volume is the mesh of the whole solid's to the chords' sag.
#[test]
fn an_indexed_ring_meshed_as_one_sector_is_one_closed_shell() {
    let e = ring(6);
    let body = fixtures::solid(&e,"part");
    let recipe = cad::recipe_static(&e.sketch,body).unwrap();
    let admitted = admission::admit_body(&e.sketch,body,&admission::Options::default()).unwrap();
    let session = native::Session::new().unwrap();
    let built = construct_built(&session,&e.sketch,body,&recipe,Some(&admitted),None).unwrap();
    let sector = built.sector.expect("the ring is built as one sector");
    let path = |what: &str| std::env::temp_dir().join(format!("solvent-sector-{}-meshed-{what}.stl",std::process::id()));
    session.sector_mesh(&sector,0.01,0.2,false).unwrap();
    let (triangles,moved) = session.sector_stl(&sector,0.01,path("sector").to_str().unwrap()).unwrap();
    session.stl(built.solid,path("whole").to_str().unwrap()).unwrap();
    let read = |what: &str| { let bytes = std::fs::read(path(what)).unwrap(); let _ = std::fs::remove_file(path(what)); bytes };
    let (sectored,whole) = (read("sector"),read("whole"));
    let shells = gcs_core::mesh::stl_shells(&sectored).unwrap();
    assert_eq!(shells.len(),1);
    // the volume a triangle soup encloses, from its binary STL
    let volume = |bytes: &[u8]| -> f64 {
        let f = |at: usize| f32::from_le_bytes(bytes[at..at+4].try_into().unwrap()) as f64;
        let n = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
        (0..n).map(|t| {
            let v: [[f64;3];3] = std::array::from_fn(|k| std::array::from_fn(|i| f(84+50*t+12+12*k+4*i)));
            gcs_core::space::dot(v[0],gcs_core::space::cross(v[1],v[2]))/6.
        }).sum()
    };
    let (mine,theirs) = (volume(&sectored),volume(&whole));
    eprintln!("{triangles} triangles, seam points moved {moved:.2e} mm; {mine:.6} mm³ against the whole mesh's {theirs:.6}");
    assert_eq!(triangles,u32::from_le_bytes(sectored[80..84].try_into().unwrap()) as usize);
    assert!((mine-theirs).abs() <= 1e-3*theirs,"{mine} against {theirs}");
}

/// The ring's blank, every operand a revolution about the spindle, built as its meridian section
/// turned once is the blank its Booleans build — its volume and its faces; the same ring bounded by
/// a prism is no revolution, and its blank is left to its Booleans, with the reason.
#[test]
fn a_blank_of_revolutions_is_its_meridian_section_turned_and_any_other_its_booleans() {
    let e = ring(6);
    let body = fixtures::solid(&e,"part");
    let recipe = cad::recipe_static(&e.sketch,body).unwrap();
    let session = native::Session::new().unwrap();
    let (turned,meridian) = session.construct_meridian(&recipe.recipe).unwrap().expect("the ring's blank is a revolution");
    let booleans = session.construct(&recipe.recipe).unwrap();
    let [a,b] = [turned,booleans].map(|s| session.volume(s).unwrap());
    eprintln!("turned {a:.9} mm³, {} faces; Booleans {b:.9} mm³, {} faces; about {:?} along {:?}",session.faces(turned).unwrap().len(),
        session.faces(booleans).unwrap().len(),meridian.origin,meridian.axis);
    assert!((a-b).abs() <= 1e-9*b,"{a} against {b}");
    assert_eq!(session.faces(turned).unwrap().len(),session.faces(booleans).unwrap().len());
    let boxed = fixtures::read(&format!("{}{}construction removal := solid(tool, under: turn, from: -35deg, to: 35deg)\n{}\
private b0 := point
private b1 := point
private b2 := point
private b3 := point
fix(x == -5, y == 1) b0
fix(x == 5, y == 1) b1
fix(x == 5, y == 3) b2
fix(x == -5, y == 3) b3
private bb := line(b0, b1)
private bw := line(b1, b2)
private bt := line(b2, b3)
private ba := line(b3, b0)
construction holder := solid(face(bb, bw, bt, ba), from: -5mm, to: 5mm)
holder bound part
",sphere(2.),cradle_roll(2.5,Observer::Parallel),indexed_ring(6,3.5,4.2,1.7,2.3)));
    let recipe = cad::recipe_static(&boxed.sketch,fixtures::solid(&boxed,"part")).unwrap();
    let why = session.construct_meridian(&recipe.recipe).unwrap().expect_err("a prism bounds the ring");
    eprintln!("{why}");
    assert!(why.contains("prism"),"{why}");
}

/// A STEP model's entities formatted side by side (`step_text`) are the kernel's text to the byte:
/// the export writes the ring as the kernel's writer would, the check (`SOLVENT_STEP_TEXT_CHECK`)
/// formatting it both ways and refusing a difference; and the file reads back as the solid.
#[test]
fn a_step_formatted_side_by_side_is_the_kernels_text() {
    std::env::set_var("SOLVENT_STEP_TEXT_CHECK","1");
    let e = ring(6);
    let body = fixtures::solid(&e,"part");
    let recipe = cad::recipe_static(&e.sketch,body).unwrap();
    let admitted = admission::admit_body(&e.sketch,body,&admission::Options::default()).unwrap();
    let session = native::Session::new().unwrap();
    let built = construct_built(&session,&e.sketch,body,&recipe,Some(&admitted),None).unwrap();
    let path = std::env::temp_dir().join(format!("solvent-sector-{}-text.step",std::process::id()));
    let said = session.step(built.solid,path.to_str().unwrap()).unwrap();
    eprintln!("{said}");
    let text = std::fs::read_to_string(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    assert!(text.starts_with("ISO-10303-21;\nHEADER;") && text.trim_end().ends_with("END-ISO-10303-21;"),"{}",&text[..80.min(text.len())]);
    // verified against the solid, its sheets' B-spline faces too; a sheet's pole moved a micron is refused
    let solid = session.brep_summary(built.solid).unwrap();
    let verified = native::step_check::verify(&text,&solid).unwrap();
    assert!(verified.splines > 0 && verified.faces == solid.faces.len(),"{verified:?}");
    let spline = text.find("B_SPLINE_SURFACE_WITH_KNOTS('',").unwrap();
    let pole = spline+text[spline..].find("(#").unwrap()+1;
    let digits = text[pole+1..].find(|c: char| !c.is_ascii_digit()).unwrap();
    let point = text.find(&format!("\n{} = CARTESIAN_POINT('',(",&text[pole..pole+1+digits])).unwrap();
    let number = text[point..].find("',(").unwrap()+point+3;
    let comma = number+text[number..].find(',').unwrap();
    let x: f64 = text[number..comma].parse().unwrap();
    let moved = format!("{}{}{}",&text[..number],x+1e-3,&text[comma..]);
    let e = native::step_check::verify(&moved,&solid).expect_err("a moved pole");
    eprintln!("{e}");
    assert!(e.contains("poles"),"{e}");
}
