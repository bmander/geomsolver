//! Milestone 3: sheets and caps stitched into a closed shell, certified, and
//! its volume against a closed form nothing in the construction knows.
use super::{harness::{self,V3},motions,tools};
use gcs_core::solid::swept_boundary::{Certificate,ConstructError,KeptMesh,Label,QueryStats,Stage,SweptBoundaryOptions,boundary_loops,construct};
use gcs_core::topology::ClosedShell;
use std::collections::{BTreeMap,BTreeSet};
use std::f64::consts::PI;
use std::time::{Duration,Instant};

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

/// What a case prints beyond its summary, read from the environment once:
/// `SOLVENT_SHEETS` every stage's detail, `SOLVENT_COLUMNS` the points of
/// columns and rims as well, `SOLVENT_DUMP=path` each stage's mesh as text.
struct Diagnostics { sheets: bool,columns: bool,dump: Option<String> }

impl Diagnostics {
    fn from_env() -> Diagnostics {
        Diagnostics {sheets:std::env::var("SOLVENT_SHEETS").is_ok(),columns:std::env::var("SOLVENT_COLUMNS").is_ok(),dump:std::env::var("SOLVENT_DUMP").ok()}
    }

    /// How many edges two triangles walk the same way (an orientation flip)
    /// and how many are used once or more than twice, printed at a stage.
    fn stage(&self,mesh: &KeptMesh,label: &str) {
        if let Some(path) = &self.dump { dump(mesh,&format!("{path}.{label}")); }
        if !self.sheets { return; }
        let mut uses: BTreeMap<(u32,u32),(usize,usize)> = Default::default();
        for t in &mesh.triangles { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); let e = uses.entry((a.min(b),a.max(b))).or_default(); if a < b { e.0 += 1; } else { e.1 += 1; } } }
        let same = uses.values().filter(|(f,b)| *f >= 2 || *b >= 2).count();
        let odd = uses.values().filter(|(f,b)| f+b != 2).count();
        let mut per: BTreeMap<u32,usize> = Default::default();
        for s in &mesh.sheet { *per.entry(*s).or_default() += 1; }
        // triangles sharing an undirected edge whose normals oppose: a fold
        let normal = |t: &[u32;3]| { let [a,b,c] = t.map(|v| mesh.vertices[v as usize]); gcs_core::solid::swept_boundary::triangle_normal(a,b,c) };
        let mut by_edge: BTreeMap<(u32,u32),Vec<usize>> = Default::default();
        for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); by_edge.entry((a.min(b),a.max(b))).or_default().push(i); } }
        let mut folded = 0; let mut example = None;
        for ts in by_edge.values() { for i in 0..ts.len() { for j in i+1..ts.len() {
            if let (Some(n0),Some(n1)) = (normal(&mesh.triangles[ts[i]]),normal(&mesh.triangles[ts[j]])) { if n0[0]*n1[0]+n0[1]*n1[1]+n0[2]*n1[2] < -0.9 { folded += 1; if example.is_none() { example = Some((ts[i],ts[j])); } } }
        } } }
        eprintln!("  {label}: {} triangles {:?}, {} edges walked twice the same way, {} edges not used twice, {folded} folded pairs",mesh.triangles.len(),per.values().collect::<Vec<_>>(),same,odd);
        if let Some((i,j)) = example { for i in [i,j] { eprintln!("      folded triangle {i} (sheet {}) {:?}",mesh.sheet[i],mesh.triangles[i].map(|v| mesh.vertices[v as usize].map(|x| (x*1e4).round()/1e4))); } }
        if same > 0 {
            let mut by_sheet: BTreeMap<(u32,u32),usize> = Default::default();
            let mut owners: BTreeMap<(u32,u32),Vec<usize>> = Default::default();
            for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); owners.entry((a,b)).or_default().push(i); } }
            for ts in owners.values() { if ts.len() >= 2 { *by_sheet.entry((mesh.sheet[ts[0]],mesh.sheet[ts[1]])).or_default() += 1; } }
            eprintln!("    same-way edges by sheet pair {by_sheet:?}");
            if let Some((_,ts)) = owners.iter().find(|(_,ts)| ts.len() >= 2) {
                for &i in ts.iter().take(2) { eprintln!("      triangle {i} (sheet {}) {:?}",mesh.sheet[i],mesh.triangles[i].map(|v| mesh.vertices[v as usize].map(|x| (x*1e3).round()/1e3))); }
            }
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

fn rounded(p: V3,scale: f64) -> V3 { p.map(|x| (x*scale).round()/scale) }

fn bounds(points: impl Iterator<Item = V3>) -> (V3,V3) {
    points.fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),p| (std::array::from_fn(|k| lo[k].min(p[k])),std::array::from_fn(|k| hi[k].max(p[k]))))
}

/// The sheets bordering a loop: those with a triangle on two of its vertices.
fn bordering(mesh: &KeptMesh,l: &[u32]) -> BTreeSet<u32> {
    mesh.triangles.iter().zip(&mesh.sheet).filter(|(t,_)| t.iter().filter(|v| l.contains(v)).count() >= 2).map(|(_,s)| *s).collect()
}

/// Wall time per pipeline stage, printed as one line at the end of a case;
/// the time spent printing is no stage's.
struct Clock { last: Instant,laps: Vec<(&'static str,Duration,String)>,queries: [usize;3] }
impl Clock {
    fn new() -> Clock { Clock {last:Instant::now(),laps:Vec::new(),queries:[0;3]} }
    fn lap(&mut self,stage: &'static str) { let now = Instant::now(); self.laps.push((stage,now-self.last,String::new())); self.last = now; }
    /// A lap that asked the field, with the near and far queries and roll
    /// evaluations it made.
    fn judged(&mut self,stage: &'static str,stats: &QueryStats) {
        self.lap(stage);
        let now = [stats.near,stats.far,stats.roll_evaluations];
        let d: [usize;3] = std::array::from_fn(|k| now[k]-self.queries[k]);
        self.queries = now;
        self.laps.last_mut().unwrap().2 = format!(" ({} near, {} far, {} rolls)",d[0],d[1],d[2]);
    }
    /// Printing is done: the next lap starts now.
    fn resume(&mut self) { self.last = Instant::now(); }
    fn report(&self) -> String { self.laps.iter().map(|(s,d,q)| format!("{s} {:.0} ms{q}",d.as_secs_f64()*1e3)).collect::<Vec<_>>().join(", ") }
}

/// The construction's stages as a case prints them: a summary always, the
/// detail `Diagnostics` asks for, and the time each stage took.
struct Report<'d> {
    diagnostics: &'d Diagnostics,
    clock: Clock,
    started: Instant,
    sheets: usize,
    points: usize,
    /// per seed: its triangles and its label counts, printed once its kept triangles are known
    labels: Vec<(usize,String)>,
    dropped: [usize;4],
    loops: usize,
}

impl Report<'_> {
    fn see(&mut self,stage: Stage<'_>,stats: &QueryStats) {
        let d = self.diagnostics;
        match stage {
            Stage::Seeded {sheets} => {
                self.clock.lap("seeds");
                self.sheets = sheets.len();
                self.points = sheets.iter().map(|s| s.points.len()).sum();
                if d.sheets { for (i,s) in sheets.iter().enumerate() {
                    let (lo,hi) = bounds(s.points.iter().copied());
                    eprintln!("  sheet {i}: {} points, {} columns, closed {}, box {:?}..{:?}",s.points.len(),s.times.len(),s.closed,rounded(lo,1e3),rounded(hi,1e3));
                    if d.columns { for c in [0,s.times.len()/2] {
                        let vs = s.column_vertices(c as u32);
                        eprintln!("    column {c} at t {:.3}: {:?}",s.times[c],vs.iter().map(|&v| (rounded(s.points[v as usize],1e2),rounded(s.normals[v as usize],1e2))).collect::<Vec<_>>());
                    } }
                } }
            }
            Stage::Grazed {regions} => {
                self.clock.lap("grazing");
                if !regions.is_empty() {
                    eprintln!("{} grazing faces swept in their own planes: {:?}",regions.len(),regions.iter().map(|g| format!("face {} of {} triangles",g.face.face,g.patch.triangles.len())).collect::<Vec<_>>());
                }
            }
            Stage::Capped {caps} => {
                self.clock.lap("caps");
                if d.sheets { for (k,c) in caps.iter().enumerate() { eprintln!("  cap {k} components: {:?}",c.components.iter().map(|c| format!("{} facets {:+.2}{}",c.facets,c.extreme,if c.kept { " kept" } else { "" })).collect::<Vec<_>>()); } }
                eprintln!("{} sheets of {} points, caps of {} and {} triangles ({:?})",self.sheets,self.points,caps[0].patch.triangles.len(),caps[1].patch.triangles.len(),self.started.elapsed());
            }
            Stage::Labelled {seeds,labelled} => {
                self.clock.judged("labels",stats);
                if d.sheets { self.labels = seeds.iter().zip(labelled).map(|(s,l)| {
                    let mut text = format!("labels kept {} moved {} inner {} positive {} unresolved {}",l.count(Label::Kept),l.count(Label::Moved),l.count(Label::Inner),l.count(Label::Positive),l.count(Label::Unresolved));
                    for (v,r,enc) in l.unresolved.iter().take(3) { text += &format!("\n    unresolved at {:?} along {:?} offset {r:.4} enclosure {enc:?}",rounded(l.points[*v],1e3),rounded(l.directions[*v],1e3)); }
                    for (v,lab) in l.labels.iter().enumerate().filter(|(_,l)| matches!(l,Label::Inner | Label::Positive)).take(3) { text += &format!("\n    {lab:?} at {:?} along {:?}",rounded(l.points[v],1e3),rounded(l.directions[v],1e3)); }
                    (s.patch().triangles.len(),text)
                }).collect(); }
            }
            Stage::Clipped {mesh,..} => {
                self.clock.judged("clip",stats);
                for (i,(triangles,text)) in self.labels.iter().enumerate() {
                    let kept = mesh.sheet.iter().filter(|x| **x == i as u32).count();
                    eprintln!("  sheet {i}: {kept} of {triangles} triangles kept; {text}");
                }
            }
            Stage::Merged {mesh,rims,welded,split} => {
                self.clock.lap("merge");
                eprintln!("{} rims clipped; {welded} rim vertices welded, {split} rim edges split",rims.len());
                if d.sheets { for r in rims {
                    let p: Vec<V3> = r.vertices.iter().map(|&v| mesh.vertices[v as usize]).collect();
                    let (lo,hi) = bounds(p.iter().copied());
                    eprintln!("  rim on sheet {}: {} vertices, closed {}, box {:?}..{:?}",r.sheet,r.vertices.len(),r.closed,rounded(lo,1e3),rounded(hi,1e3));
                    if d.columns { eprintln!("    {:?}",p.iter().map(|&p| rounded(p,1e2)).collect::<Vec<_>>()); }
                } }
            }
            Stage::Kept {before,keep,kept,inside,outside} => {
                self.clock.judged("kept",stats);
                self.dropped[0] = inside; self.dropped[1] = outside;
                if d.sheets {
                    let mut by_sheet: BTreeMap<u32,Vec<V3>> = Default::default();
                    for (i,t) in before.triangles.iter().enumerate().filter(|(i,_)| !keep[*i]) {
                        let [a,b,c] = t.map(|v| before.vertices[v as usize]);
                        by_sheet.entry(before.sheet[i]).or_default().push(rounded(std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.),1e3));
                    }
                    for (s,cs) in &by_sheet { eprintln!("  dropped from sheet {s}: {} triangles, e.g. {:?}",cs.len(),&cs[..cs.len().min(6)]); }
                }
                d.stage(kept,"kept");
            }
            Stage::Unioned {mesh,replaced} => { self.clock.lap("union"); self.dropped[2] = replaced; d.stage(mesh,"unioned"); }
            Stage::Uncovered {mesh,dropped} => { self.clock.lap("overlaps"); self.dropped[3] = dropped; d.stage(mesh,"without overlaps"); }
            Stage::Welded {mesh,collapsed,doubled} => {
                self.clock.lap("weld");
                if d.sheets { eprintln!("  {collapsed} vertex pairs within half a sagitta merged, {doubled} doubled slivers dropped"); }
                d.stage(mesh,"welded");
                let [inside,outside,replaced,dropped] = self.dropped;
                eprintln!("{inside} triangles dropped as interior, {outside} as exterior, {replaced} unioned in their planes, {dropped} as overlapping");
                let loops = boundary_loops(&mesh.triangles);
                for (i,l) in loops.iter().enumerate() {
                    let distinct: BTreeSet<_> = l.iter().collect();
                    eprintln!("  loop {i}: {} vertices, {} distinct, first {:?}",l.len(),distinct.len(),mesh.vertices[l[0] as usize]);
                }
                if loops.len() >= 2 && d.sheets {
                    let pts = |l: &Vec<u32>| l.iter().map(|&v| mesh.vertices[v as usize]).collect::<Vec<_>>();
                    let zip = gcs_core::solid::zip_polylines(&pts(&loops[0]),&pts(&loops[1]),true);
                    let mut uses: BTreeMap<(bool,u32,u32),usize> = Default::default();
                    for t in &zip { for k in 0..3 { let (p,q) = (t[k],t[(k+1)%3]); if p.0 == q.0 { *uses.entry((p.0,p.1.min(q.1),p.1.max(q.1))).or_default() += 1; } } }
                    let multi: Vec<_> = uses.iter().filter(|(_,n)| **n > 1).collect();
                    eprintln!("  zip of loops 0 and 1: {} triangles, loop edges used more than once: {:?}",zip.len(),multi);
                }
                self.loops = loops.len();
            }
            Stage::Split {mesh} => {
                self.clock.lap("split");
                d.stage(mesh,"split");
                if d.sheets { for (i,l) in boundary_loops(&mesh.triangles).iter().enumerate() {
                    let pts: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
                    let (lo,hi) = bounds(pts.iter().copied());
                    eprintln!("  after split, loop {i}: {} vertices, box {:?}..{:?}, bordered by sheets {:?}",l.len(),rounded(lo,1e3),rounded(hi,1e3),bordering(mesh,l));
                    if l.len() <= 40 {
                        eprintln!("    {:?}",pts.iter().map(|&p| rounded(p,1e3)).collect::<Vec<_>>());
                        for (i,t) in mesh.triangles.iter().enumerate() { if t.iter().filter(|v| l.contains(v)).count() >= 2 { eprintln!("      triangle {i} (sheet {}) {:?}",mesh.sheet[i],t.map(|v| rounded(mesh.vertices[v as usize],1e3))); } }
                    }
                } }
            }
            Stage::Zipped {mesh,pairs,unpaired} => {
                self.clock.lap("zip");
                d.stage(mesh,"zipped");
                eprintln!("welded: {} vertices, {} triangles, {} boundary loops; {pairs} rims zipped, {} loops left",mesh.vertices.len(),mesh.triangles.len(),self.loops,unpaired.len());
                for (k,l) in unpaired.iter().enumerate() {
                    let pts: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
                    let (lo,hi) = bounds(pts.iter().copied());
                    eprintln!("  unpaired loop {k}: {} vertices, box {:?}..{:?}, bordered by sheets {:?}",l.len(),rounded(lo,1e3),rounded(hi,1e3),bordering(mesh,l));
                    if d.sheets {
                        eprintln!("    {:?}",pts.iter().map(|&p| rounded(p,1e3)).collect::<Vec<_>>());
                        // the triangles on the first few loop edges
                        for k in 0..l.len().min(8) {
                            let (a,b) = (l[k],l[(k+1)%l.len()]);
                            for (i,t) in mesh.triangles.iter().enumerate() { if t.contains(&a) && t.contains(&b) { eprintln!("      edge {k} on triangle {i} (sheet {}) {:?}",mesh.sheet[i],t.map(|v| rounded(mesh.vertices[v as usize],1e4))); } }
                        }
                    }
                }
                if let Some(path) = &d.dump { dump(mesh,path); }
            }
            Stage::Certified {mesh,certificate} => {
                self.clock.judged("certify",stats);
                eprintln!("stage times: {}",self.clock.report());
                eprintln!("certified {} of {} triangles ({} slivers by their centroid), {} thin, {} failed; volume {:.5}; {} ({:?})",certificate.certified,mesh.triangles.len(),certificate.slivers,
                    certificate.thin.len(),certificate.failures.len(),volume(mesh),stats.report(),self.started.elapsed());
            }
        }
        self.clock.resume();
    }
}

/// Seeds and caps judged, kept, welded, rims zipped, certified: the shell.
fn closed_shell(source: &str) -> (KeptMesh,Certificate) { closed_shell_at(source,SAGITTA) }

/// The shell at a sagitta of the case's own (a coarser one for a tool whose
/// field never depends on the roll, where every query refines the whole
/// interval).
pub(super) fn closed_shell_at(source: &str,sagitta: f64) -> (KeptMesh,Certificate) {
    let (mesh,certificate) = shell_at(source,sagitta);
    (mesh,certificate.unwrap_or_else(|e| panic!("construction refused: {e:?}")))
}

/// The shell, or the construction's refusal with the mesh as the zip left it (empty when it
/// refused before the zip), for a case that does not close yet.
pub(super) fn shell_at(source: &str,sagitta: f64) -> (KeptMesh,Result<Certificate,ConstructError>) {
    let diagnostics = Diagnostics::from_env();
    let mut clock = Clock::new();
    let e = harness::read(source);
    let swept = harness::solid(&e,"swept");
    clock.lap("read");
    let options = SweptBoundaryOptions {sagitta,spacing:SPACING,..Default::default()};
    let mut report = Report {diagnostics:&diagnostics,clock,started:Instant::now(),sheets:0,points:0,labels:Vec::new(),dropped:[0;4],loops:0};
    let mut zipped: Option<KeptMesh> = None;
    let built = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,stats| {
        if let Stage::Zipped {mesh,..} = &stage { zipped = Some((*mesh).clone()); }
        report.see(stage,stats)
    });
    let built = match built {
        Ok(built) => built,
        Err(e) => { eprintln!("refused: {e:?}"); return (zipped.unwrap_or_default(),Err(e)); }
    };
    let (mesh,certificate) = (built.mesh,built.certificate);
    let probe = options.probe_distance();
    let sampler = super::labels::Sampler::new(&e,swept);
    for (i,c,f) in certificate.failures.iter().take(5) {
        let [a,b,cc] = mesh.triangles[*i].map(|v| mesh.vertices[v as usize]);
        if diagnostics.sheets { eprintln!("    corners {:?}",[a,b,cc].map(|p| rounded(p,1e6))); }
        let n = gcs_core::solid::swept_boundary::certify::triangle_normal(a,b,cc).unwrap_or([0.;3]);
        let at = |r: f64| -> V3 { std::array::from_fn(|k| c[k]+r*n[k]) };
        eprintln!("  triangle {i} at {c:?} normal {:?} from {}: {f:?}; sampled field inside {:+.4} outside {:+.4}",rounded(n,1e3),
            if mesh.sheet[*i] == u32::MAX { "a zip".into() } else { format!("sheet {}",mesh.sheet[*i]) },sampler.least(at(-probe)).0,sampler.least(at(probe)).0);
    }
    if let Err(e) = closed(&mesh) {
        let mut uses: BTreeMap<(u32,u32),Vec<usize>> = Default::default();
        for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); uses.entry((a.min(b),a.max(b))).or_default().push(i); } }
        let mut histogram: BTreeMap<usize,usize> = Default::default();
        for v in uses.values() { *histogram.entry(v.len()).or_default() += 1; }
        eprintln!("  shell error {e}; edge use histogram {histogram:?}");
        for (edge,ts) in uses.iter().filter(|(_,v)| v.len() != 2).take(2) {
            eprintln!("    edge at {:?}-{:?} used by {} triangles from sheets {:?}",rounded(mesh.vertices[edge.0 as usize],1e3),rounded(mesh.vertices[edge.1 as usize],1e3),ts.len(),ts.iter().map(|&i| mesh.sheet[i]).collect::<Vec<_>>());
            for (i,t) in mesh.triangles.iter().enumerate().filter(|(_,t)| t.contains(&edge.0)) {
                eprintln!("      triangle {i} (sheet {}) at {:?}",mesh.sheet[i],t.map(|v| rounded(mesh.vertices[v as usize],1e3)));
            }
        }
    }
    let mut kinds: BTreeMap<String,usize> = Default::default();
    for (i,_,f) in &certificate.thin { *kinds.entry(format!("{f:?} from {}",if mesh.sheet[*i] == u32::MAX { "a zip".to_string() } else { format!("sheet {}",mesh.sheet[*i]) })).or_default() += 1; }
    if !kinds.is_empty() { eprintln!("  thin: {kinds:?}"); }
    (mesh,Ok(certificate))
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
