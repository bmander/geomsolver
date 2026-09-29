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
    fixtures::read(&format!("{}{}construction solid removal(tool, under: turn, from: -35deg, to: 35deg)\n{}",sphere(2.),
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
