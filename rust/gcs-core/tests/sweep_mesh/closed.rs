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
/// columns and rims as well, `SOLVENT_DUMP=path` each stage's mesh as text,
/// `SOLVENT_WINDOW=x,y,z,r` what stands in that ball at every stage, which is
/// how a defect is watched forming rather than inferred backwards from the
/// stage that refused over it.
/// A stage whose mesh is not clean says so whatever the environment says.
struct Diagnostics { sheets: bool,columns: bool,dump: Option<String>,window: Option<(V3,f64)>,coincidence: f64,snap: f64 }

impl Diagnostics {
    fn from_env(coincidence: f64,snap: f64) -> Diagnostics {
        let window = std::env::var("SOLVENT_WINDOW").ok().and_then(|s| {
            let n: Vec<f64> = s.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            (n.len() == 4).then(|| ([n[0],n[1],n[2]],n[3]))
        });
        Diagnostics {sheets:std::env::var("SOLVENT_SHEETS").is_ok(),columns:std::env::var("SOLVENT_COLUMNS").is_ok(),dump:std::env::var("SOLVENT_DUMP").ok(),window,coincidence,snap}
    }

    /// How many edges two triangles walk the same way (an orientation flip)
    /// and how many are used once or more than twice, printed at a stage.
    fn stage(&self,mesh: &KeptMesh,label: &str) {
        if let Some(path) = &self.dump { dump(mesh,&format!("{path}.{label}")); }
        // The checks every stage's output should pass, said whenever one fails: the stage that
        // first reports a defect is the stage that made it, where the stage that refuses over it
        // may be many stages later.
        let h = gcs_core::solid::swept_boundary::hygiene(mesh,self.coincidence,self.snap);
        if !h.clean() || self.sheets { eprintln!("  {label} hygiene: {}",h.report(self.snap)); }
        // the same ball at every stage, so a defect is watched forming
        if let Some((at,r)) = self.window {
            let w = gcs_core::solid::swept_boundary::window(mesh,at,r);
            if !w.vertices.is_empty() || self.sheets { eprintln!("  {label} {}",w.report()); }
        }
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
                // How near two sheets come to one another, which decides what kind of defect an
                // unpaired loop between them is. Two sheets that abut share a seam, and the weld
                // makes one vertex of two positions within the coincidence (1e-7): so a nearest
                // approach under that is already shared and the loop is something else, one a
                // little above it is two traces of one seam and wants identity at the source, and
                // one at a thousandth or more means the sheets genuinely end apart and it is the
                // traced geometry, not the bookkeeping. Point against point, since for two grids
                // that abut the nearest approach is the seam however the boundary rows run.
                if d.sheets {
                    let mut pairs: Vec<(f64,usize,usize,usize)> = Vec::new();
                    for i in 0..sheets.len() { for j in i+1..sheets.len() {
                        let (mut least,mut near) = (f64::INFINITY,0);
                        for q in &sheets[j].points {
                            let e = sheets[i].points.iter().map(|p| harness::distance(*p,*q)).fold(f64::INFINITY,f64::min);
                            least = least.min(e);
                            if e <= 1e-7 { near += 1; }
                        }
                        if least.is_finite() { pairs.push((least,i,j,near)); }
                    } }
                    pairs.sort_by(|a,b| a.0.total_cmp(&b.0));
                    for (least,i,j,near) in pairs.iter().take(8) {
                        eprintln!("    sheets {i} and {j}: nearest {least:.9}, {near} of the second's points already coincident (within 1e-7)",);
                    }
                }
            }
            Stage::Grazed {regions} => {
                self.clock.lap("grazing");
                if !regions.is_empty() {
                    eprintln!("{} grazing faces swept in their own planes: {:?}",regions.len(),regions.iter().map(|g| format!("face {} of {} triangles",g.face.face,g.patch.triangles.len())).collect::<Vec<_>>());
                }
            }
            Stage::Capped {caps} => {
                self.clock.lap("caps");
                // which operation each cap's vertices came from: a defect in a cap is a vertex
                // that should not be there, and this names the code that put it there
                if d.sheets { for (k,c) in caps.iter().enumerate() {
                    let mut by: BTreeMap<String,usize> = Default::default();
                    for o in &c.origin { *by.entry(format!("{o:?}")).or_default() += 1; }
                    eprintln!("  cap {k} vertices by origin: {by:?}");
                } }
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
            Stage::Clipped {mesh,rims} => {
                self.clock.judged("clip",stats);
                // the earliest stage holding a mesh: what the seeds and the caps handed over
                d.stage(mesh,"clipped");
                for (i,(triangles,text)) in self.labels.iter().enumerate() {
                    let kept = mesh.sheet.iter().filter(|x| **x == i as u32).count();
                    eprintln!("  sheet {i}: {kept} of {triangles} triangles kept; {text}");
                }
                // How near the rims of different sheets come to one another, which is the whole of
                // what `merge_creases` can act on: it welds a vertex of one rim onto a vertex of
                // another within the snap, and splits a rim edge only at a rim vertex. A pair that
                // never comes that close cannot be merged at all, and the two sheets keep separate
                // boundaries however the crease runs between them — so this says whether a crease
                // went unmerged because the gate refused it or because the rims never met.
                if d.sheets {
                    let pts: Vec<Vec<V3>> = rims.iter().map(|r| r.vertices.iter().map(|&v| mesh.vertices[v as usize]).collect()).collect();
                    // per pair: the nearest approach, and how many of the second's vertices lie
                    // within the snap of one of the first's
                    let mut pairs: Vec<(f64,usize,usize,usize)> = Vec::new();
                    for i in 0..rims.len() { for j in i+1..rims.len() {
                        if rims[i].sheet == rims[j].sheet { continue; }
                        let (mut least,mut near) = (f64::INFINITY,0);
                        for q in &pts[j] {
                            let e = pts[i].iter().map(|p| harness::distance(*p,*q)).fold(f64::INFINITY,f64::min);
                            least = least.min(e);
                            if e <= 0.02 { near += 1; }
                        }
                        if least.is_finite() { pairs.push((least,i,j,near)); }
                    } }
                    pairs.sort_by(|a,b| a.0.total_cmp(&b.0));
                    let count = |t: f64| pairs.iter().filter(|p| p.0 <= t).count();
                    eprintln!("  rim pairs of different sheets: {}; nearest within 0.005: {}, within 0.02 (the snap): {}, within 0.03: {}, within 0.08: {}",
                        pairs.len(),count(0.005),count(0.02),count(0.03),count(0.08));
                    for (least,i,j,near) in pairs.iter().take(8) {
                        eprintln!("    rim {i} (sheet {}, {} vertices) and rim {j} (sheet {}, {} vertices): nearest {least:.6}, {near} of the second's within the snap",
                            rims[*i].sheet,rims[*i].vertices.len(),rims[*j].sheet,rims[*j].vertices.len());
                    }
                }
            }
            Stage::Merged {mesh,rims,welded,split} => {
                self.clock.lap("merge");
                eprintln!("{} rims clipped; {welded} rim vertices welded, {split} rim edges split",rims.len());
                d.stage(mesh,"merged");
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
                // How many boundary edges meet at each vertex. Where several sheets meet at one
                // point — which `chain` leaves separate on purpose, threading them would carry a
                // sheet through itself — each contributes two boundary edges there, so three
                // sheets make six. The loop walk rotates round a vertex fan until it finds a
                // boundary edge, so with several to choose from it pairs them as it meets them,
                // and a walk closed through such a vertex need not bound a hole at all.
                let boundary_degree: BTreeMap<u32,usize> = {
                    let mut uses: BTreeMap<(u32,u32),usize> = Default::default();
                    for t in &mesh.triangles { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); *uses.entry((a.min(b),a.max(b))).or_insert(0) += 1; } }
                    let mut degree: BTreeMap<u32,usize> = Default::default();
                    for ((a,b),n) in &uses { if *n == 1 { *degree.entry(*a).or_insert(0) += 1; *degree.entry(*b).or_insert(0) += 1; } }
                    degree
                };
                for (k,l) in unpaired.iter().enumerate() {
                    let pts: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
                    let (lo,hi) = bounds(pts.iter().copied());
                    // what `rim_zip` itself judged the loop on: a loop visiting a vertex twice is
                    // pinched and is never paired or filled, and the slit pass takes only a loop
                    // turning back at exactly two corners. A rounded picture of the points says
                    // neither, and both are read off the loop alone.
                    let distinct: BTreeSet<u32> = l.iter().copied().collect();
                    let n = l.len();
                    let corners = (0..n).filter(|&k| {
                        let (a,b,c) = (pts[(k+n-1)%n],pts[k],pts[(k+1)%n]);
                        let (u,w) = ([b[0]-a[0],b[1]-a[1],b[2]-a[2]],[c[0]-b[0],c[1]-b[1],c[2]-b[2]]);
                        let (lu,lw) = ((u[0]*u[0]+u[1]*u[1]+u[2]*u[2]).sqrt(),(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]).sqrt());
                        lu > 0. && lw > 0. && (u[0]*w[0]+u[1]*w[1]+u[2]*w[2])/(lu*lw) < -0.5
                    }).count();
                    eprintln!("  unpaired loop {k}: {} vertices ({} distinct, {corners} reflex corners), box {:?}..{:?}, bordered by sheets {:?}",
                        l.len(),distinct.len(),rounded(lo,1e3),rounded(hi,1e3),bordering(mesh,l));
                    // How thin the loop is: a loop whose least side is far under the sagitta is a
                    // sliver, and no triangle laid across it has area to take a normal from, so
                    // the fill declines it and it can never be closed by adding a triangle. Its
                    // two sides have to be identified instead. The least side is the tolerance
                    // such a weld would need, so it is printed for every loop that survives.
                    {
                        let mut ext = [hi[0]-lo[0],hi[1]-lo[1],hi[2]-lo[2]];
                        ext.sort_by(f64::total_cmp);
                        eprintln!("    sides: least {:.6}, middle {:.6}, longest {:.6}",ext[0],ext[1],ext[2]);
                    }
                    // Whether the loop folds back on itself, which is what decides the treatment:
                    // a slit, or a bundle of them, has a counterpart close to nearly every vertex
                    // and can be zipped arc against arc; a seam running across surface that was
                    // never laid has none, and no band may be invented there. Measured as the
                    // nearest other vertex of this loop four or more steps away along the walk, so
                    // a vertex's own neighbours cannot answer for it, and counted at three
                    // distances since what the zip admits is a whole arc pair lying together.
                    if n >= 9 {
                        let dist = |a: V3,b: V3| ((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt();
                        let far = |i: usize,j: usize| { let d = (i as isize-j as isize).unsigned_abs(); d.min(n-d) >= 4 };
                        let counterpart: Vec<f64> = (0..n).map(|i| (0..n).filter(|&j| far(i,j))
                            .map(|j| dist(pts[i],pts[j])).fold(f64::INFINITY,f64::min)).collect();
                        let near = |t: f64| counterpart.iter().filter(|&&d| d <= t).count();
                        let worst = counterpart.iter().copied().fold(0_f64,f64::max);
                        eprintln!("    folds back: of {n} vertices, {} have a counterpart 4+ steps away within 0.05, {} within 0.2, {} within 0.5; farthest {:.4}",
                            near(0.05),near(0.2),near(0.5),worst);
                    }
                    // Whether this walk runs through a vertex where several boundary edges meet: a
                    // loop closed through one of those may be an artefact of how the walk paired
                    // them rather than a hole. Two is the ordinary count for a boundary vertex.
                    {
                        let crowded: Vec<(u32,usize)> = l.iter().map(|&v| (v,boundary_degree.get(&v).copied().unwrap_or(0)))
                            .filter(|(_,deg)| *deg > 2).collect();
                        eprintln!("    boundary degree: {} of {} vertices carry more than two boundary edges {:?}",crowded.len(),l.len(),crowded);
                    }
                    // Whether the surface about the loop is laid twice. Triangles of different
                    // sheets whose centroids all but coincide and whose normals agree are two
                    // coverings of one patch, which the coverage clip should have taken to one —
                    // and a loop bounded by doubled surface is no hole, which is what the zip's
                    // declined bands (each duplicating a facet already there) suggest. Only the
                    // triangles about the loop are walked: that is where the question is, and the
                    // whole mesh would be quadratic for nothing. Measured: the surface about these
                    // loops is **not** doubled — the 30° cylinder reports none at all, and the
                    // tumbling cylinder a few pairs in a neighbourhood of hundreds, which is
                    // coincidence between sheets rather than a second covering.
                    if d.sheets {
                        let centroid = |t: &[u32;3]| -> V3 { let [a,b,c] = t.map(|v| mesh.vertices[v as usize]); std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.) };
                        let about: Vec<usize> = (0..mesh.triangles.len()).filter(|&i| {
                            let c = centroid(&mesh.triangles[i]);
                            (0..3).all(|k| c[k] >= lo[k]-0.05 && c[k] <= hi[k]+0.05)
                        }).collect();
                        let normal = |t: &[u32;3]| { let [a,b,c] = t.map(|v| mesh.vertices[v as usize]); gcs_core::solid::swept_boundary::triangle_normal(a,b,c) };
                        let whose: BTreeSet<u32> = about.iter().map(|&i| mesh.sheet[i]).collect();
                        let mut doubled = 0;
                        for (x,&i) in about.iter().enumerate() { for &j in &about[x+1..] {
                            if mesh.sheet[i] == mesh.sheet[j] { continue; }
                            let (ci,cj) = (centroid(&mesh.triangles[i]),centroid(&mesh.triangles[j]));
                            if (0..3).map(|k| (ci[k]-cj[k]).powi(2)).sum::<f64>().sqrt() > 0.01 { continue; }
                            if let (Some(u),Some(w)) = (normal(&mesh.triangles[i]),normal(&mesh.triangles[j])) {
                                if u[0]*w[0]+u[1]*w[1]+u[2]*w[2] > 0.99 { doubled += 1; }
                            }
                        } }
                        eprintln!("    about it: {} triangles from sheets {:?}; {doubled} pairs of different sheets doubled (centroids within 0.01, normals agreeing)",about.len(),whose);
                    }
                    if d.sheets {
                        // the walk itself, each vertex with the seeds whose triangles use it: a
                        // loop bounded by one seed is a hole in it, one alternating between two
                        // is the gap between their rims, and a vertex the walk meets twice is
                        // where the two touch without sharing an edge
                        for (k,&v) in l.iter().enumerate() {
                            let on: BTreeSet<u32> = mesh.triangles.iter().zip(&mesh.sheet)
                                .filter(|(t,_)| t.contains(&v)).map(|(_,s)| *s).collect();
                            let twice = l.iter().filter(|&&u| u == v).count() > 1;
                            eprintln!("      {k}: v{v} at {:?}, seeds {:?}{}",rounded(mesh.vertices[v as usize],1e4),on,
                                if twice { "   <- the walk meets this one twice" } else { "" });
                        }
                        eprintln!("    {:?}",pts.iter().map(|&p| rounded(p,1e3)).collect::<Vec<_>>());
                        // the triangles on the first few loop edges
                        for k in 0..l.len().min(8) {
                            let (a,b) = (l[k],l[(k+1)%l.len()]);
                            for (i,t) in mesh.triangles.iter().enumerate() { if t.contains(&a) && t.contains(&b) { eprintln!("      edge {k} on triangle {i} (sheet {}) {:?}",mesh.sheet[i],t.map(|v| rounded(mesh.vertices[v as usize],1e4))); } }
                        }
                    }
                }
                // Which boundary vertices no boundary edge could ever be split at. `split_where`
                // visits every boundary edge, so a vertex past one edge's end is normally picked
                // up by the next edge along the same line; the span skip can only bite for a
                // vertex that **no** boundary edge contains, and then the edge that does contain
                // it is interior, which no tolerance reaches. This counts the two apart.
                {
                    let tol = 0.04_f64; // the junction tolerance the split works to
                    let mut uses: BTreeMap<(u32,u32),usize> = Default::default();
                    for t in &mesh.triangles { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); *uses.entry((a.min(b),a.max(b))).or_insert(0) += 1; } }
                    let edges: Vec<(u32,u32)> = uses.iter().filter(|(_,n)| **n == 1).map(|(e,_)| *e).collect();
                    let verts: BTreeSet<u32> = edges.iter().flat_map(|&(a,b)| [a,b]).collect();
                    let (mut contained,mut orphan) = (0,0);
                    for &v in &verts {
                        let p = mesh.vertices[v as usize];
                        let (mut inside,mut on_line) = (false,false);
                        for &(a,b) in &edges {
                            if v == a || v == b { continue; }
                            let (pa,pb) = (mesh.vertices[a as usize],mesh.vertices[b as usize]);
                            let e = [pb[0]-pa[0],pb[1]-pa[1],pb[2]-pa[2]];
                            let l2 = e[0]*e[0]+e[1]*e[1]+e[2]*e[2];
                            if l2 <= 0. { continue; }
                            let w = [p[0]-pa[0],p[1]-pa[1],p[2]-pa[2]];
                            let f = (w[0]*e[0]+w[1]*e[1]+w[2]*e[2])/l2;
                            let foot = [pa[0]+f*e[0],pa[1]+f*e[1],pa[2]+f*e[2]];
                            if harness::distance(p,foot) > tol { continue; }
                            if f > 0. && f < 1. { inside = true; } else { on_line = true; }
                        }
                        if inside { contained += 1; } else if on_line { orphan += 1; }
                    }
                    eprintln!("boundary vertices: {} in all; {contained} lie inside some boundary edge's span (the split can take them), {orphan} only on a line past an end (no boundary edge can)",verts.len());
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
    let mut clock = Clock::new();
    let e = harness::read(source);
    let swept = harness::solid(&e,"swept");
    clock.lap("read");
    let options = SweptBoundaryOptions {sagitta,spacing:SPACING,..Default::default()};
    let diagnostics = Diagnostics::from_env(options.coincidence(),options.vertex_tolerance());
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
