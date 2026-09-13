//! Milestone 2: every kept triangle certified by the field a probe distance
//! inside and outside it, and the closure audit of what the labels leave.
use super::{harness,motions,tools};
use gcs_core::solid::{MaterialField,swept_boundary::{FieldJudge,boundary_loops,certify,grazing_seeds,kept_triangles,label_sheets,seeds}};

const SPACING: f64 = 0.5;
const SAGITTA: f64 = 0.02;
const EPSILON: f64 = SAGITTA/4.;
const REACH: f64 = SAGITTA;
const PROBE: f64 = 2.*SAGITTA;

/// Seeds judged, kept triangles certified, boundary loops counted. The seeds are the traced
/// sheets and the regions of the faces the motion carries within their own planes, which is
/// what covers the sides of a box sliding along one of them.
fn certified(source: &str) -> (gcs_core::solid::swept_boundary::KeptMesh,gcs_core::solid::swept_boundary::Certificate,Vec<Vec<u32>>) {
    let e = harness::read(source);
    let swept = harness::solid(&e,"swept");
    let (sweep,mut sheets,faces) = seeds(&e.sketch,swept,SPACING,SAGITTA,&|_| {}).unwrap();
    let mut times: Vec<f64> = sheets.iter().flat_map(|s| s.times.iter().copied()).collect();
    times.sort_by(f64::total_cmp); times.dedup();
    if times.is_empty() { times = sweep.domain().to_vec(); }
    sheets.extend(grazing_seeds(&faces,&times,sweep.domain(),SAGITTA,SPACING).unwrap().into_iter().map(|g| g.patch));
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,EPSILON/2.,4000,1000,4096);
    let labelled = label_sheets(&mut judge,&sheets,EPSILON,REACH).unwrap();
    let mesh = kept_triangles(&sheets,&labelled);
    let started = std::time::Instant::now();
    let certificate = certify(&mut judge,&mesh.vertices,&mesh.triangles,PROBE,2.*EPSILON).unwrap();
    let loops = boundary_loops(&mesh.triangles);
    eprintln!("{} triangles: {} certified ({} at a halved probe, least {:.4}), {} thin, {} failed in {:?}; {} boundary loops of {:?} vertices; {}",mesh.triangles.len(),certificate.certified,
        certificate.halved,certificate.least_used,certificate.unresolved.len(),certificate.failures.len(),started.elapsed(),loops.len(),loops.iter().map(|l| l.len()).collect::<Vec<_>>(),judge.stats.report());
    for (i,c,f) in certificate.unresolved.iter().take(3) { eprintln!("  thin triangle {i} at {c:?}: {f:?}"); }
    for (i,c,f) in certificate.failures.iter().take(5) { eprintln!("  triangle {i} at {c:?}: {f:?}"); }
    (mesh,certificate,loops)
}

#[test]
fn a_turned_spheres_tube_is_certified_with_two_open_ends() {
    let source = format!("{}{}{}",tools::SPHERE,motions::TURN_SPINDLE,motions::swept("turn",-60.,60.));
    let (mesh,certificate,loops) = certified(&source);
    assert!(certificate.is_complete(),"{} of {} triangles failed",certificate.failures.len(),mesh.triangles.len());
    assert_eq!(loops.len(),2,"a tube has two open ends");
}

#[test]
fn a_translated_boxs_sides_are_certified() {
    let source = format!("{}{}{}",tools::BOX,motions::slide_x(10.),motions::swept("feed",0.,360.));
    let (mesh,certificate,_) = certified(&source);
    assert!(mesh.triangles.len() > 0);
    assert!(certificate.is_complete(),"{} of {} triangles failed",certificate.failures.len(),mesh.triangles.len());
}

#[test]
fn a_tumbling_cylinders_unresolved_centroids_prevent_completion() {
    let source = format!("{}{}{}",tools::CYLINDER,motions::TUMBLE,motions::swept("turn",-30.,30.));
    let (mesh,certificate,_) = certified(&source);
    assert!(!certificate.is_complete());
    assert!(!certificate.unresolved.is_empty());
    assert_eq!(certificate.certified+certificate.failures.len()+certificate.unresolved.len(),mesh.triangles.len());
}
