//! Milestone 3: sheets and caps stitched into a closed shell, certified, and
//! its volume against a closed form nothing in the construction knows.
use super::{harness::{self,V3},motions,tools};
use gcs_core::solid::{MaterialField,swept_boundary::{FieldJudge,KeptMesh,boundary_loops,caps,certify,kept_triangles,label_sheets,retained,rim_zip,seeds,weld,without_overlaps}};
use gcs_core::topology::ClosedShell;
use std::f64::consts::PI;

const SPACING: f64 = 0.5;
const SAGITTA: f64 = 0.02;
const EPSILON: f64 = SAGITTA/4.;
const REACH: f64 = SAGITTA;
const PROBE: f64 = 2.*SAGITTA;

/// Divergence-theorem volume of an outward-wound indexed mesh.
fn volume(mesh: &KeptMesh) -> f64 {
    let mut six = 0.;
    for t in &mesh.triangles {
        let [a,b,c] = t.map(|i| mesh.vertices[i as usize]);
        six += a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]);
    }
    six/6.
}

/// Seeds and caps judged, kept, welded, rims zipped, certified: the shell.
fn closed_shell(source: &str) -> (KeptMesh,gcs_core::solid::swept_boundary::Certificate) {
    let e = harness::read(source);
    let swept = harness::solid(&e,"swept");
    let started = std::time::Instant::now();
    let (_,mut sheets) = seeds(&e.sketch,swept,SPACING,SAGITTA,&|_| {}).unwrap();
    let [start,end] = caps(&e.sketch,swept,SAGITTA).unwrap();
    eprintln!("{} sheets of {} points, caps of {} and {} triangles ({:?})",sheets.len(),sheets.iter().map(|s| s.points.len()).sum::<usize>(),
        start.triangles.len(),end.triangles.len(),started.elapsed());
    sheets.push(start); sheets.push(end);
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,EPSILON/2.,4000,1000,4096);
    let labelled = label_sheets(&mut judge,&sheets,EPSILON,REACH).unwrap();
    let mesh = kept_triangles(&sheets,&labelled);
    let keep = without_overlaps(&mesh,2.*EPSILON);
    let dropped = keep.iter().filter(|k| !**k).count();
    let mut mesh = retained(&mesh,&keep);
    weld(&mut mesh,1e-9);
    eprintln!("{dropped} overlapping triangles dropped");
    let loops = boundary_loops(&mesh.triangles);
    for (i,l) in loops.iter().enumerate() {
        let mut sorted = l.clone(); sorted.sort(); sorted.dedup();
        eprintln!("  loop {i}: {} vertices, {} distinct, first {:?}",l.len(),sorted.len(),mesh.vertices[l[0] as usize]);
    }
    if loops.len() >= 2 {
        let pts = |l: &Vec<u32>| l.iter().map(|&v| mesh.vertices[v as usize]).collect::<Vec<_>>();
        let (a,b) = (pts(&loops[0]),pts(&loops[1]));
        let zip = gcs_core::solid::zip_polylines(&a,&b,true);
        let mut uses: std::collections::BTreeMap<(bool,u32,u32),usize> = Default::default();
        for t in &zip { for k in 0..3 { let (p,q) = (t[k],t[(k+1)%3]); if p.0 == q.0 { *uses.entry((p.0,p.1.min(q.1),p.1.max(q.1))).or_default() += 1; } } }
        let multi: Vec<_> = uses.iter().filter(|(_,n)| **n > 1).collect();
        eprintln!("  zip of loops 0 and 1: {} triangles, loop edges used more than once: {:?}",zip.len(),multi);
    }
    let before = loops.len();
    let (pairs,unpaired) = rim_zip(&mut mesh,4.*SPACING,2.*SAGITTA,EPSILON);
    eprintln!("welded: {} vertices, {} triangles, {} boundary loops; {pairs} rims zipped, {} loops left",mesh.vertices.len(),mesh.triangles.len(),before,unpaired.len());
    for (k,l) in unpaired.iter().enumerate() {
        let pts: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
        let (lo,hi) = pts.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),p| (std::array::from_fn(|k| lo[k].min(p[k])),std::array::from_fn(|k| hi[k].max(p[k]))));
        eprintln!("  unpaired loop {k}: {} vertices, box {:?}..{:?}",l.len(),lo.map(|x| (x*1e3).round()/1e3),hi.map(|x| (x*1e3).round()/1e3));
    }
    let certificate = certify(&mut judge,&mesh.vertices,&mesh.triangles,PROBE,2.*EPSILON).unwrap();
    eprintln!("certified {} of {} triangles, {} thin, {} failed; volume {:.5}; {} ({:?})",certificate.certified,mesh.triangles.len(),
        certificate.thin.len(),certificate.failures.len(),volume(&mesh),judge.stats.report(),started.elapsed());
    let sampler = super::labels::Sampler::new(&e,swept);
    for (i,c,f) in certificate.failures.iter().take(5) {
        let [a,b,cc] = mesh.triangles[*i].map(|v| mesh.vertices[v as usize]);
        let n = gcs_core::solid::swept_boundary::certify::triangle_normal(a,b,cc).unwrap_or([0.;3]);
        let at = |r: f64| -> V3 { std::array::from_fn(|k| c[k]+r*n[k]) };
        eprintln!("  triangle {i} at {c:?} normal {:?} from {}: {f:?}; sampled field inside {:+.4} outside {:+.4}",n.map(|x| (x*1e3).round()/1e3),
            if mesh.sheet[*i] == u32::MAX { "a zip".into() } else { format!("sheet {}",mesh.sheet[*i]) },sampler.least(at(-PROBE)).0,sampler.least(at(PROBE)).0);
    }
    if let Err(e) = closed(&mesh) {
        let mut uses: std::collections::BTreeMap<(u32,u32),Vec<usize>> = Default::default();
        for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); uses.entry((a.min(b),a.max(b))).or_default().push(i); } }
        let mut histogram: std::collections::BTreeMap<usize,usize> = Default::default();
        for v in uses.values() { *histogram.entry(v.len()).or_default() += 1; }
        eprintln!("  shell error {e}; edge use histogram {histogram:?}");
        for (edge,ts) in uses.iter().filter(|(_,v)| v.len() != 2).take(2) {
            eprintln!("    edge at {:?}-{:?} used by {} triangles from sheets {:?}",mesh.vertices[edge.0 as usize].map(|x| (x*1e3).round()/1e3),mesh.vertices[edge.1 as usize].map(|x| (x*1e3).round()/1e3),ts.len(),ts.iter().map(|&i| mesh.sheet[i]).collect::<Vec<_>>());
            for (i,t) in mesh.triangles.iter().enumerate().filter(|(_,t)| t.contains(&edge.0)) {
                eprintln!("      triangle {i} (sheet {}) at {:?}",mesh.sheet[i],t.map(|v| mesh.vertices[v as usize].map(|x| (x*1e3).round()/1e3)));
            }
        }
    }
    let mut kinds: std::collections::BTreeMap<String,usize> = Default::default();
    for (i,_,f) in &certificate.thin { *kinds.entry(format!("{f:?} from {}",if mesh.sheet[*i] == u32::MAX { "a zip".to_string() } else { format!("sheet {}",mesh.sheet[*i]) })).or_default() += 1; }
    if !kinds.is_empty() { eprintln!("  thin: {kinds:?}"); }
    (mesh,certificate)
}

fn closed(mesh: &KeptMesh) -> Result<(),String> {
    let mesh = mesh.compact();
    let triangles: Vec<[usize;3]> = mesh.triangles.iter().map(|t| t.map(|v| v as usize)).collect();
    ClosedShell::from_triangles(mesh.vertices.len(),&triangles).map(|_| ()).map_err(|e| format!("{e:?}"))
}

#[test]
fn a_cylinder_plunged_along_its_axis_sweeps_a_longer_cylinder() {
    // radius 1, height 2, advanced 6 along its axis over one turn of the parameter
    let source = format!("{}{}{}",tools::CYLINDER,"motion plunge(along: axis, advance: 6mm)\n",motions::swept("plunge",0.,360.));
    let (mesh,certificate) = closed_shell(&source);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    let expected = PI*8.;
    let v = volume(&mesh);
    assert!(v <= expected*(1.+1e-3) && v >= expected*(1.-3.*SAGITTA/1.),"volume {v} against {expected}");
}

#[test]
fn a_box_translated_along_x_sweeps_a_longer_box() {
    // 2 x 3 x 2 box advanced 10 along x: 12 + 6 * 10
    let source = format!("{}{}{}",tools::BOX,motions::slide_x(10.),motions::swept("feed",0.,360.));
    let (mesh,certificate) = closed_shell(&source);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    let expected = 72.;
    let v = volume(&mesh);
    assert!((v-expected).abs() <= 1e-9*expected,"volume {v} against {expected}");
}

#[test]
fn a_sphere_turned_about_the_spindle_sweeps_a_torus_segment_with_spherical_ends() {
    let source = format!("{}{}{}",tools::SPHERE,motions::TURN_SPINDLE,motions::swept("turn",-60.,60.));
    let (mesh,certificate) = closed_shell(&source);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    // a tube of radius 1 along an arc of radius 3 through 120°, plus the two
    // hemispherical ends that make one sphere
    let expected = PI*3.*(2.*PI/3.)+4.*PI/3.;
    let v = volume(&mesh);
    // an inscribed mesh falls short of a round solid: its columns are
    // inscribed polygons (about 1.3 sagittas per unit of the smallest radius,
    // the tube's, 1) and its seams chamfer a column's width; it never exceeds it
    assert!(v <= expected*(1.+1e-3) && v >= expected*(1.-3.*SAGITTA/1.),"volume {v} against {expected}");
}
