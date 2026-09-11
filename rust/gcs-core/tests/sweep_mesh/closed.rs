//! Milestone 3: sheets and caps stitched into a closed shell, certified, and
//! its volume against a closed form nothing in the construction knows.
use super::{harness::{self,V3},motions,tools};
use gcs_core::solid::{MaterialField,swept_boundary::{FieldJudge,KeptMesh,boundary_loops,caps,certify,clip_overlaps,clip_sheets,merge_creases,planar_union,retained,rim_zip,seeds,weld}};
use gcs_core::topology::ClosedShell;
use std::f64::consts::PI;

const SPACING: f64 = 0.5;
const SAGITTA: f64 = 0.02;

/// Divergence-theorem volume of an outward-wound indexed mesh.
pub(super) fn volume(mesh: &KeptMesh) -> f64 {
    let mut six = 0.;
    for t in &mesh.triangles {
        let [a,b,c] = t.map(|i| mesh.vertices[i as usize]);
        six += a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]);
    }
    six/6.
}

/// How many edges two triangles walk the same way (an orientation flip)
/// and how many are used once or more than twice, printed at a stage.
fn stage(mesh: &KeptMesh,label: &str) {
    if let Ok(path) = std::env::var("SOLVENT_DUMP") { dump(mesh,&format!("{path}.{label}")); }
    if std::env::var("SOLVENT_SHEETS").is_err() { return; }
    let mut uses: std::collections::BTreeMap<(u32,u32),(usize,usize)> = Default::default();
    for t in &mesh.triangles { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); let e = uses.entry((a.min(b),a.max(b))).or_default(); if a < b { e.0 += 1; } else { e.1 += 1; } } }
    let same = uses.values().filter(|(f,b)| *f >= 2 || *b >= 2).count();
    let odd = uses.values().filter(|(f,b)| f+b != 2).count();
    let mut per: std::collections::BTreeMap<u32,usize> = Default::default();
    for s in &mesh.sheet { *per.entry(*s).or_default() += 1; }
    // triangles sharing an undirected edge whose normals oppose: a fold
    let normal = |t: &[u32;3]| { let [a,b,c] = t.map(|v| mesh.vertices[v as usize]); gcs_core::solid::swept_boundary::triangle_normal(a,b,c) };
    let mut by_edge: std::collections::BTreeMap<(u32,u32),Vec<usize>> = Default::default();
    for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); by_edge.entry((a.min(b),a.max(b))).or_default().push(i); } }
    let mut folded = 0; let mut example = None;
    for ts in by_edge.values() { for i in 0..ts.len() { for j in i+1..ts.len() {
        if let (Some(n0),Some(n1)) = (normal(&mesh.triangles[ts[i]]),normal(&mesh.triangles[ts[j]])) { if n0[0]*n1[0]+n0[1]*n1[1]+n0[2]*n1[2] < -0.9 { folded += 1; if example.is_none() { example = Some((ts[i],ts[j])); } } }
    } } }
    eprintln!("  {label}: {} triangles {:?}, {} edges walked twice the same way, {} edges not used twice, {folded} folded pairs",mesh.triangles.len(),per.values().collect::<Vec<_>>(),same,odd);
    if let Some((i,j)) = example { for i in [i,j] { eprintln!("      folded triangle {i} (sheet {}) {:?}",mesh.sheet[i],mesh.triangles[i].map(|v| mesh.vertices[v as usize].map(|x| (x*1e4).round()/1e4))); } }
    if same > 0 {
        let mut by_sheet: std::collections::BTreeMap<(u32,u32),usize> = Default::default();
        let mut owners: std::collections::BTreeMap<(u32,u32),Vec<usize>> = Default::default();
        for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); owners.entry((a,b)).or_default().push(i); } }
        for ts in owners.values() { if ts.len() >= 2 { *by_sheet.entry((mesh.sheet[ts[0]],mesh.sheet[ts[1]])).or_default() += 1; } }
        eprintln!("    same-way edges by sheet pair {by_sheet:?}");
        if let Some((e,ts)) = owners.iter().find(|(_,ts)| ts.len() >= 2) {
            for &i in ts.iter().take(2) { eprintln!("      triangle {i} (sheet {}) {:?}",mesh.sheet[i],mesh.triangles[i].map(|v| mesh.vertices[v as usize].map(|x| (x*1e3).round()/1e3))); }
            let _ = e;
        }
    }
}

/// The mesh as text (`v x y z`, `t a b c sheet`), for a script to read.
fn dump(mesh: &KeptMesh,path: &str) {
    let mut text = String::new();
    for v in &mesh.vertices { text += &format!("v {} {} {}\n",v[0],v[1],v[2]); }
    for (t,s) in mesh.triangles.iter().zip(&mesh.sheet) { text += &format!("t {} {} {} {s}\n",t[0],t[1],t[2]); }
    std::fs::write(path,text).unwrap();
}

/// Seeds and caps judged, kept, welded, rims zipped, certified: the shell.
fn closed_shell(source: &str) -> (KeptMesh,gcs_core::solid::swept_boundary::Certificate) { closed_shell_at(source,SAGITTA) }

/// The shell at a sagitta of the case's own (a coarser one for a tool whose
/// field never depends on the roll, where every query refines the whole
/// interval).
pub(super) fn closed_shell_at(source: &str,sagitta: f64) -> (KeptMesh,gcs_core::solid::swept_boundary::Certificate) {
    let e = harness::read(source);
    let swept = harness::solid(&e,"swept");
    let started = std::time::Instant::now();
    let (epsilon,reach,probe) = (sagitta/4.,sagitta,2.*sagitta);
    let (_,mut sheets) = seeds(&e.sketch,swept,SPACING,sagitta,&|_| {}).unwrap();
    let ([start,end],components) = caps(&e.sketch,swept,&sheets,sagitta,sagitta/4.,SPACING).unwrap();
    if std::env::var("SOLVENT_SHEETS").is_ok() { for (k,c) in components.iter().enumerate() { eprintln!("  cap {k} components: {:?}",c.iter().map(|c| format!("{} facets {:+.2}{}",c.facets,c.extreme,if c.kept { " kept" } else { "" })).collect::<Vec<_>>()); } }
    eprintln!("{} sheets of {} points, caps of {} and {} triangles ({:?})",sheets.len(),sheets.iter().map(|s| s.points.len()).sum::<usize>(),
        start.triangles.len(),end.triangles.len(),started.elapsed());
    if std::env::var("SOLVENT_SHEETS").is_ok() {
        for (i,s) in sheets.iter().enumerate() {
            let (lo,hi) = s.points.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),p| (std::array::from_fn(|k| lo[k].min(p[k])),std::array::from_fn(|k| hi[k].max(p[k]))));
            eprintln!("  sheet {i}: {} points, {} columns, closed {}, box {:?}..{:?}",s.points.len(),s.times.len(),s.closed,lo.map(|x| (x*1e3).round()/1e3),hi.map(|x| (x*1e3).round()/1e3));
            if std::env::var("SOLVENT_COLUMNS").is_ok() {
                for c in [0,s.times.len()/2] {
                    let vs = s.column_vertices(c as u32);
                    eprintln!("    column {c} at t {:.3}: {:?}",s.times[c],vs.iter().map(|&v| (s.points[v as usize].map(|x| (x*1e2).round()/1e2),s.normals[v as usize].map(|x| (x*1e2).round()/1e2))).collect::<Vec<_>>());
                }
            }
        }
    }
    sheets.push(start); sheets.push(end);
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,epsilon/2.,4000,1000,4096);
    let labelled = gcs_core::solid::swept_boundary::label_sheets_from(&mut judge,&sheets,epsilon,reach,sheets.len()-2).unwrap();
    let (mut mesh,rims) = clip_sheets(&mut judge,&sheets,&labelled,epsilon,reach,sheets.len()-2).unwrap();
    let (welded_rims,split_rims) = merge_creases(&mut mesh,&rims,1.5*sagitta,4.*epsilon);
    eprintln!("{} rims clipped; {welded_rims} rim vertices welded, {split_rims} rim edges split",rims.len());
    if std::env::var("SOLVENT_SHEETS").is_ok() {
        for r in &rims { let p: Vec<V3> = r.vertices.iter().map(|&v| mesh.vertices[v as usize]).collect(); let (lo,hi) = p.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),p| (std::array::from_fn(|k| lo[k].min(p[k])),std::array::from_fn(|k| hi[k].max(p[k])))); eprintln!("  rim on sheet {}: {} vertices, closed {}, box {:?}..{:?}",r.sheet,r.vertices.len(),r.closed,lo.map(|x| (x*1e3).round()/1e3),hi.map(|x| (x*1e3).round()/1e3));
            if std::env::var("SOLVENT_COLUMNS").is_ok() { eprintln!("    {:?}",p.iter().map(|p| p.map(|x| (x*1e2).round()/1e2)).collect::<Vec<_>>()); } }
    }
    if std::env::var("SOLVENT_SHEETS").is_ok() {
        use gcs_core::solid::swept_boundary::Label;
        for (i,(l,s)) in labelled.iter().zip(&sheets).enumerate() {
            let kept = mesh.sheet.iter().filter(|x| **x == i as u32).count();
            eprintln!("  sheet {i}: {} of {} triangles kept; labels kept {} moved {} inner {} positive {} unresolved {}",kept,s.triangles.len(),l.count(Label::Kept),l.count(Label::Moved),l.count(Label::Inner),l.count(Label::Positive),l.count(Label::Unresolved));
            for (v,r,enc) in l.unresolved.iter().take(3) { eprintln!("    unresolved at {:?} along {:?} offset {r:.4} enclosure {enc:?}",l.points[*v].map(|x| (x*1e3).round()/1e3),l.directions[*v].map(|x| (x*1e3).round()/1e3)); }
            for (v,lab) in l.labels.iter().enumerate().filter(|(_,l)| matches!(l,Label::Inner | Label::Positive)).take(3) { eprintln!("    {lab:?} at {:?} along {:?}",l.points[v].map(|x| (x*1e3).round()/1e3),l.directions[v].map(|x| (x*1e3).round()/1e3)); }
        }
    }
    let (keep,inside,outside) = gcs_core::solid::swept_boundary::centroid_kept(&mut judge,&mesh,sagitta).unwrap();
    if std::env::var("SOLVENT_SHEETS").is_ok() {
        let mut by_sheet: std::collections::BTreeMap<u32,Vec<V3>> = Default::default();
        for (i,t) in mesh.triangles.iter().enumerate().filter(|(i,_)| !keep[*i]) {
            let [a,b,c] = t.map(|v| mesh.vertices[v as usize]);
            by_sheet.entry(mesh.sheet[i]).or_default().push(std::array::from_fn(|k| ((a[k]+b[k]+c[k])/3.*1e3).round()/1e3));
        }
        for (s,cs) in &by_sheet { eprintln!("  dropped from sheet {s}: {} triangles, e.g. {:?}",cs.len(),&cs[..cs.len().min(6)]); }
    }
    let mesh = retained(&mesh,&keep);
    stage(&mesh,"kept");
    let (mesh,replaced) = planar_union(&mesh,1e-7);
    stage(&mesh,"unioned");
    let (mut mesh,dropped) = clip_overlaps(&mesh,2.*sagitta);
    stage(&mesh,"without overlaps");
    weld(&mut mesh,1e-7);
    let collapsed = gcs_core::solid::swept_boundary::collapse_short_edges(&mut mesh,epsilon/2.);
    let doubled = gcs_core::solid::swept_boundary::drop_doubled_slivers(&mut mesh,2.*epsilon);
    if std::env::var("SOLVENT_SHEETS").is_ok() { eprintln!("  {collapsed} vertex pairs within half a sagitta merged, {doubled} doubled slivers dropped"); }
    stage(&mesh,"welded");
    eprintln!("{inside} triangles dropped as interior, {outside} as exterior, {replaced} unioned in their planes, {dropped} as overlapping");
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
    gcs_core::solid::swept_boundary::split_at_vertices(&mut mesh,2.*sagitta);
    stage(&mesh,"split");
    if std::env::var("SOLVENT_SHEETS").is_ok() {
        for (i,l) in boundary_loops(&mesh.triangles).iter().enumerate() {
            let pts: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
            let (lo,hi) = pts.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),p| (std::array::from_fn(|k| lo[k].min(p[k])),std::array::from_fn(|k| hi[k].max(p[k]))));
            let mut sheets: std::collections::BTreeSet<u32> = Default::default();
            for (i,t) in mesh.triangles.iter().enumerate() { if t.iter().filter(|v| l.contains(v)).count() >= 2 { sheets.insert(mesh.sheet[i]); } }
            eprintln!("  after split, loop {i}: {} vertices, box {:?}..{:?}, bordered by sheets {:?}",l.len(),lo.map(|x| (x*1e3).round()/1e3),hi.map(|x| (x*1e3).round()/1e3),sheets);
            if l.len() <= 40 {
                eprintln!("    {:?}",pts.iter().map(|p| p.map(|x| (x*1e3).round()/1e3)).collect::<Vec<_>>());
                for (i,t) in mesh.triangles.iter().enumerate() { if t.iter().filter(|v| l.contains(v)).count() >= 2 { eprintln!("      triangle {i} (sheet {}) {:?}",mesh.sheet[i],t.map(|v| mesh.vertices[v as usize].map(|x| (x*1e3).round()/1e3))); } }
            }
        }
    }
    let (pairs,unpaired) = rim_zip(&mut mesh,SPACING,2.*sagitta);
    stage(&mesh,"zipped");
    eprintln!("welded: {} vertices, {} triangles, {} boundary loops; {pairs} rims zipped, {} loops left",mesh.vertices.len(),mesh.triangles.len(),before,unpaired.len());
    for (k,l) in unpaired.iter().enumerate() {
        let pts: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
        let (lo,hi) = pts.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),p| (std::array::from_fn(|k| lo[k].min(p[k])),std::array::from_fn(|k| hi[k].max(p[k]))));
        let mut sheets: std::collections::BTreeSet<u32> = Default::default();
        for (i,t) in mesh.triangles.iter().enumerate() { if t.iter().filter(|v| l.contains(v)).count() >= 2 { sheets.insert(mesh.sheet[i]); } }
        eprintln!("  unpaired loop {k}: {} vertices, box {:?}..{:?}, bordered by sheets {:?}",l.len(),lo.map(|x| (x*1e3).round()/1e3),hi.map(|x| (x*1e3).round()/1e3),sheets);
        if std::env::var("SOLVENT_SHEETS").is_ok() {
            eprintln!("    {:?}",pts.iter().map(|p| p.map(|x| (x*1e3).round()/1e3)).collect::<Vec<_>>());
            // the triangles on the first few loop edges
            for k in 0..l.len().min(8) {
                let (a,b) = (l[k],l[(k+1)%l.len()]);
                for (i,t) in mesh.triangles.iter().enumerate() { if t.contains(&a) && t.contains(&b) { eprintln!("      edge {k} on triangle {i} (sheet {}) {:?}",mesh.sheet[i],t.map(|v| mesh.vertices[v as usize].map(|x| (x*1e4).round()/1e4))); } }
            }
        }
    }
    if let Ok(path) = std::env::var("SOLVENT_DUMP") { dump(&mesh,&path); }
    let certificate = certify(&mut judge,&mesh.vertices,&mesh.triangles,probe,2.*epsilon).unwrap();
    eprintln!("certified {} of {} triangles ({} slivers by their centroid), {} thin, {} failed; volume {:.5}; {} ({:?})",certificate.certified,mesh.triangles.len(),certificate.slivers,
        certificate.thin.len(),certificate.failures.len(),volume(&mesh),judge.stats.report(),started.elapsed());
    let sampler = super::labels::Sampler::new(&e,swept);
    for (i,c,f) in certificate.failures.iter().take(5) {
        let [a,b,cc] = mesh.triangles[*i].map(|v| mesh.vertices[v as usize]);
        if std::env::var("SOLVENT_SHEETS").is_ok() { eprintln!("    corners {:?}",[a,b,cc].map(|p| p.map(|x| (x*1e6).round()/1e6))); }
        let n = gcs_core::solid::swept_boundary::certify::triangle_normal(a,b,cc).unwrap_or([0.;3]);
        let at = |r: f64| -> V3 { std::array::from_fn(|k| c[k]+r*n[k]) };
        eprintln!("  triangle {i} at {c:?} normal {:?} from {}: {f:?}; sampled field inside {:+.4} outside {:+.4}",n.map(|x| (x*1e3).round()/1e3),
            if mesh.sheet[*i] == u32::MAX { "a zip".into() } else { format!("sheet {}",mesh.sheet[*i]) },sampler.least(at(-probe)).0,sampler.least(at(probe)).0);
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

pub(super) fn closed(mesh: &KeptMesh) -> Result<(),String> {
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

/// A round case: at most three sagittas per unit of the least radius below
/// its closed form, never above.
fn round(v: f64,expected: f64,least_radius: f64) {
    assert!(v <= expected*(1.+1e-3) && v >= expected*(1.-3.*SAGITTA/least_radius),"volume {v} against {expected}");
}

#[test]
fn a_sphere_turned_about_its_own_centre_sweeps_only_itself() {
    // the field never depends on the roll, so every query refines the whole
    // interval: a coarser sagitta keeps the case to seconds
    let sagitta = 0.05;
    let source = format!("{}{}{}",tools::SPHERE,motions::TURN_OWN_AXIS,motions::swept("turn",-60.,60.));
    let (mesh,certificate) = closed_shell_at(&source,sagitta);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    let (v,expected) = (volume(&mesh),4.*PI/3.);
    assert!(v <= expected*(1.+1e-3) && v >= expected*(1.-3.*sagitta),"volume {v} against {expected}");
}

#[test]
fn a_cylinder_turned_about_the_spindle_sweeps_a_ring_sector_with_round_ends() {
    // radius 1 about x = 3, height 2, turned 120° about world z: the tool
    // plus the sector swept by every circle about the axis, r from 2 to 4
    let source = format!("{}{}{}",tools::CYLINDER,motions::TURN_SPINDLE,motions::swept("turn",-60.,60.));
    let (mesh,certificate) = closed_shell(&source);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    round(volume(&mesh),2.*PI+(2.*PI/3.)*2.*(16.-4.)/2.,1.);
}

#[test]
fn a_triangular_prism_translated_along_x_sweeps_exactly() {
    // the prism's section across x: y in [-0.8, 0.8], z in [-1.5, 1.5]
    let source = format!("{}{}{}",tools::TRIANGLE_PRISM,motions::slide_x(10.),motions::swept("feed",0.,360.));
    let (mesh,certificate) = closed_shell(&source);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    let expected = 0.5*1.5*1.6*3.+10.*1.6*3.;
    let v = volume(&mesh);
    assert!((v-expected).abs() <= 1e-9*expected,"volume {v} against {expected}");
}

#[test]
fn a_sphere_translated_along_the_spindle_sweeps_a_capsule() {
    let source = format!("{}{}{}",tools::SPHERE,motions::slide_z(10.),motions::swept("feed",0.,360.));
    let (mesh,certificate) = closed_shell(&source);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    round(volume(&mesh),4.*PI/3.+10.*PI,1.);
}

#[test]
fn a_negative_advance_sweeps_the_other_way() {
    // the capsule again, travelling down: its centre line runs from z = 0 to z = -10
    let source = format!("{}{}{}",tools::SPHERE,motions::slide_z(-10.),motions::swept("feed",0.,360.));
    let (mesh,certificate) = closed_shell(&source);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    round(volume(&mesh),4.*PI/3.+10.*PI,1.);
    let (lo,hi) = mesh.vertices.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),p| (lo.min(p[2]),hi.max(p[2])));
    assert!((lo+11.).abs() < 0.05 && (hi-1.).abs() < 0.05,"z extent {lo}..{hi}");
}
