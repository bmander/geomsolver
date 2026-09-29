//! The accuracy meter's harness (`cad::measure`) on native exports of turned parts — a pulley
//! (planes, cylinders and cones, a groove cut), a sphere and a torus: their STEP faces are the
//! exact surfaces and read as nothing on both routes, their STL corners read as nothing but the
//! float32 rounding, and the same STL with every corner moved 20 µm along its normal reads 20 µm.
use gcs_core::solid::{accuracy::{Meter,Options,SampleKind},agreement,cad};
use std::collections::BTreeMap;
#[path="../src/cad/native.rs"]
#[allow(dead_code)]
mod native;
#[path="../src/cad/progress.rs"]
#[allow(dead_code)]
mod progress;
#[path="../src/cad/measure.rs"]
#[allow(dead_code)]
mod measure;

const OFFSET: f64 = 0.02;

/// A binary STL of these triangles, each carrying its winding's normal.
fn stl(vertices: &[[f64;3]],triangles: &[[u32;3]]) -> Vec<u8> {
    let mut out = vec![0u8;80];
    out.extend((triangles.len() as u32).to_le_bytes());
    for t in triangles {
        let [a,b,c] = t.map(|i| vertices[i as usize]);
        let n = gcs_core::space::triangle_normal(a,b,c).unwrap_or([0.;3]);
        for v in [n,a,b,c] { for x in v { out.extend((x as f32).to_le_bytes()); } }
        out.extend([0u8;2]);
    }
    out
}

/// Every corner of an STL moved `offset` along its normal (its triangles' area-weighted normal,
/// corners welded by their float32 bits), and the corners whose triangles all agree with that
/// normal within 25 degrees: those on a face, not on an edge (faceting turns a triangle of a
/// curved face a tenth of a radian or so, a crease far more).
fn offset_corners(bytes: &[u8],offset: f64) -> (Vec<u8>,Vec<[f64;3]>) {
    let (vertices,triangles) = agreement::stl_triangles(bytes,1.).unwrap();
    let key = |p: [f64;3]| p.map(|x| (x as f32).to_bits());
    let mut normals: BTreeMap<[u32;3],[f64;3]> = BTreeMap::new();
    for t in &triangles {
        let [a,b,c] = t.map(|i| vertices[i as usize]);
        let n = gcs_core::space::cross(gcs_core::space::sub(b,a),gcs_core::space::sub(c,a));
        for p in [a,b,c] { let s = normals.entry(key(p)).or_insert([0.;3]); for k in 0..3 { s[k] += n[k]; } }
    }
    let unit = |p: [f64;3]| gcs_core::space::normalised(normals[&key(p)]).unwrap();
    let mut smooth: BTreeMap<[u32;3],bool> = BTreeMap::new();
    for t in &triangles {
        let [a,b,c] = t.map(|i| vertices[i as usize]);
        let Some(n) = gcs_core::space::triangle_normal(a,b,c) else { continue };
        for p in [a,b,c] {
            let agrees = gcs_core::space::dot(n,unit(p)) >= 25_f64.to_radians().cos();
            *smooth.entry(key(p)).or_insert(true) &= agrees;
        }
    }
    let moved: Vec<[f64;3]> = vertices.iter().map(|&p| gcs_core::space::add(p,gcs_core::space::scale(unit(p),offset))).collect();
    let faces = vertices.iter().zip(&moved).filter(|(p,_)| smooth[&key(**p)]).map(|(_,m)| m.map(|x| x as f32 as f64)).collect();
    (stl(&moved,&triangles),faces)
}

fn check(name: &str,source: &str,solid: &str) {
    let e = fixtures::read(source);
    let sk = &e.sketch;
    let body = fixtures::solid(&e,solid);
    let session = native::Session::new().unwrap();
    let shape = session.construct(&cad::recipe(sk,body).unwrap()).unwrap();
    let out = std::env::temp_dir().join(format!("solvent-measure-{name}-{}",std::process::id()));
    std::fs::create_dir_all(&out).unwrap();
    let (step,stl) = (out.join("part.step"),out.join("part.stl"));
    session.step(shape,step.to_str().unwrap()).unwrap();
    session.stl(shape,stl.to_str().unwrap()).unwrap();
    let meter = Meter::read(sk,body,Options::in_units(1.)).unwrap();
    let worst = |samples: &[gcs_core::solid::accuracy::Sample],which: &dyn Fn(&gcs_core::solid::accuracy::Sample) -> bool,expected: f64| {
        let chosen: Vec<_> = samples.iter().filter(|s| which(s)).copied().collect();
        let read = measure::measure(&meter,&chosen);
        let mut w = [0_f64;2];
        for m in &read {
            let a = m.analytic.expect("every sample within reach of an exact face");
            w[0] = w[0].max((a.distance-expected).abs());
            w[1] = w[1].max((m.field-expected).abs());
        }
        (chosen.len(),w)
    };
    // the STEP file's faces are the exact surfaces
    let from_step = measure::step_samples(step.to_str().unwrap(),1.,4000).unwrap();
    let (n,w) = worst(&from_step.samples,&|_| true,0.);
    eprintln!("{name}: {n} STEP face points off by at most {:.2e} mm (analytic), {:.2e} mm (field); faces {:?}",w[0],w[1],from_step.classes);
    assert!(n > 100 && w[0] < 1e-6 && w[1] < 1e-6,"{name}: STEP faces read {w:?}");
    // the STL's corners lie on them to float32 rounding, its chords within the mesher's deflection
    let bytes = std::fs::read(&stl).unwrap();
    let from_stl = measure::stl_samples(&bytes,1.,70000).unwrap();
    let (n,w) = worst(&from_stl.samples,&|s| s.kind == SampleKind::Vertex,0.);
    eprintln!("{name}: {n} STL corners off by at most {:.2e} mm (analytic), {:.2e} mm (field)",w[0],w[1]);
    assert!(n > 100 && w[0] < 1e-5 && w[1] < 1e-5,"{name}: STL corners read {w:?}");
    let (n,w) = worst(&from_stl.samples,&|s| s.kind == SampleKind::Centroid,0.);
    eprintln!("{name}: {n} STL centroids off by at most {:.2e} mm (analytic), {:.2e} mm (field)",w[0],w[1]);
    assert!(w[0] < 0.011,"{name}: STL centroids read {w:?}");
    // every corner moved 20 µm along its normal reads 20 µm where it is on a face
    let (moved,on_faces) = offset_corners(&bytes,OFFSET);
    let from_moved = measure::stl_samples(&moved,1.,70000).unwrap();
    let key = |p: [f64;3]| p.map(f64::to_bits);
    let on: std::collections::BTreeSet<_> = on_faces.iter().map(|&p| key(p)).collect();
    let (n,w) = worst(&from_moved.samples,&|s| s.kind == SampleKind::Vertex && on.contains(&key(s.position)),OFFSET);
    eprintln!("{name}: {n} STL corners moved {OFFSET} mm read off by at most {:.2e} mm (analytic), {:.2e} mm (field)",w[0],w[1]);
    assert!(n > 50 && w[0] < 1e-3 && w[1] < 1e-3,"{name}: moved corners read {w:?}");
    let _ = std::fs::remove_dir_all(&out);
}

#[test]
fn a_pulleys_export_reads_as_exact_and_an_offset_as_the_offset() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/solid_pulley.sv");
    check("pulley",&std::fs::read_to_string(path).unwrap(),"body");
}

#[test]
fn a_spheres_export_reads_as_exact_and_an_offset_as_the_offset() { check("sphere",&fixtures::tools::sphere(0.),"tool"); }

#[test]
fn a_torus_export_reads_as_exact_and_an_offset_as_the_offset() { check("torus",&fixtures::tools::torus(0.),"tool"); }
