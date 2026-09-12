//! Milestone 4: sheets that cross. Where one sheet turns inner across
//! another, the kept parts must meet along the crease between them, and the
//! shell must still close and certify.
use super::{closed::{closed,closed_shell_at,shell_at,volume},harness::{self,V3},motions,tools};
use gcs_core::{model::SolidDef,motion::Family};

const SAGITTA: f64 = 0.02;

// Each case's document: the tool, its motion and the sweep, one source
// shared by the case's test and the export below.
pub(super) fn turning_prism() -> String { format!("{}{}{}",tools::TRIANGLE_PRISM,motions::TURN_OFFSET,motions::swept("turn",-50.,50.)) }
pub(super) fn tumbling_cylinder() -> String { format!("{}{}{}",tools::CYLINDER,motions::TUMBLE,motions::swept("turn",-30.,30.)) }
pub(super) fn turned_box() -> String { format!("{}{}{}",tools::BOX,motions::turn_about(4.,-1.5,5.,-1.5),motions::swept("turn",0.,30.)) }
pub(super) fn turned_lens() -> String { format!("{}{}{}",tools::LENS,motions::TURN_SPINDLE,motions::swept("turn",-60.,60.)) }
fn sliding_dumbbell() -> String { format!("{}{}{}",tools::DUMBBELL,motions::slide_x(4.),motions::swept("feed",0.,360.)) }

/// The swept volume by sampled membership: a midpoint grid over the box,
/// each point material when the tool's closed-form membership holds at any
/// of `poses` parameters. Uses only `Family::at`, never the field.
fn sampled_volume(source: &str,member: &dyn Fn(V3) -> bool,lo: V3,hi: V3,grid: usize,poses: usize) -> f64 {
    let e = harness::read(source);
    let swept = harness::solid(&e,"swept");
    let SolidDef::Swept {motion,from,to,..} = &e.sketch.solids[swept].def else { panic!("not a sweep") };
    let family = Family::read(&e.sketch,*motion as usize).unwrap();
    let inverses: Vec<_> = (0..=poses).map(|k| family.at(from.value+(to.value-from.value)*k as f64/poses as f64).unwrap().inverse()).collect();
    let step: V3 = std::array::from_fn(|k| (hi[k]-lo[k])/grid as f64);
    let mut count = 0usize;
    for i in 0..grid { for j in 0..grid { for k in 0..grid {
        let p = [lo[0]+(i as f64+0.5)*step[0],lo[1]+(j as f64+0.5)*step[1],lo[2]+(k as f64+0.5)*step[2]];
        if inverses.iter().any(|m| member(m.point(p))) { count += 1; }
    } } }
    count as f64*step[0]*step[1]*step[2]
}

/// Whether the crowded junctions really are fixed points of the motion, which is the premise the
/// 5c design rests on and was inferred rather than measured. The loops the construction refuses
/// run through vertices carrying six and eight boundary edges — four surface sectors meeting at a
/// point — and the plan predicts those are where a generator sweeps a bowtie through a fixed
/// point. So: how far does the motion actually move them? A fixed point does not move at all.
/// Nothing is built or changed here; it only measures, and either way the answer decides whether
/// the design in `docs/swept-boundary.md` is the one to build.
#[test]
#[ignore]
fn whether_the_crowded_junctions_are_fixed_points_of_the_motion() {
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct};
    use std::collections::BTreeMap;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let SolidDef::Swept {motion,from,to,..} = &e.sketch.solids[swept].def else { panic!("not a sweep") };
    let family = Family::read(&e.sketch,*motion as usize).unwrap();
    let (from,to) = (from.value,to.value);
    // the finished mesh, and which of its vertices carry more than two boundary edges
    let mut crowded: Vec<(u32,V3,usize)> = Vec::new();
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,unpaired,..} = stage {
            let mut uses: BTreeMap<(u32,u32),usize> = Default::default();
            for t in &mesh.triangles { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); *uses.entry((a.min(b),a.max(b))).or_insert(0) += 1; } }
            let mut degree: BTreeMap<u32,usize> = Default::default();
            for ((a,b),n) in &uses { if *n == 1 { *degree.entry(*a).or_insert(0) += 1; *degree.entry(*b).or_insert(0) += 1; } }
            let on_a_loop: std::collections::BTreeSet<u32> = unpaired.iter().flatten().copied().collect();
            for (&v,&deg) in &degree {
                if deg > 2 && on_a_loop.contains(&v) { crowded.push((v,mesh.vertices[v as usize],deg)); }
            }
        }
    });
    crowded.sort_by(|a,b| b.2.cmp(&a.2).then(a.0.cmp(&b.0)));
    eprintln!("== tumbling cylinder: {} crowded vertices on refused loops",crowded.len());
    // how far the motion carries each of them over the declared interval
    let steps = 64;
    for (v,p,deg) in crowded.iter().take(10) {
        let moved = (0..=steps).map(|i| {
            let t = from+(to-from)*i as f64/steps as f64;
            let q = family.at(t).unwrap().point(*p);
            harness::distance(*p,q)
        }).fold(0_f64,f64::max);
        eprintln!("   v{v} (degree {deg}) at {:?}: the motion moves it at most {moved:.6}",
            p.map(|x| (x*1e4).round()/1e4));
    }
}

/// How much surface each refused loop is actually missing. A loop enclosing no area is not a
/// hole: it is one segment sampled two ways, walked out along one chain and back along the other,
/// and closing it welds rather than invents — the only repair the certificate cannot object to,
/// since it adds no triangle. A loop enclosing area is a genuine open seam and needs surface that
/// only the field may supply. One measured slit is not a population, so this asks of all of them.
/// Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn whether_the_refused_loops_enclose_any_area() {
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct};
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let sub = |a: V3,b: V3| -> V3 { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] };
    let cross = |a: V3,b: V3| -> V3 { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] };
    let norm = |a: V3| -> f64 { (a[0]*a[0]+a[1]*a[1]+a[2]*a[2]).sqrt() };
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,unpaired,..} = stage {
            let mut slits = 0;
            eprintln!("== tumbling cylinder: {} refused loops, by the area each encloses",unpaired.len());
            for (k,l) in unpaired.iter().enumerate() {
                let p: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
                let n = p.len();
                let c = { let mut c = [0.;3]; for q in &p { for i in 0..3 { c[i] += q[i]/n as f64; } } c };
                // fanned from the centroid: the absolute sum is zero only if every fan triangle is
                // flat, which is what a collinear out-and-back walk gives
                let area: f64 = (0..n).map(|i| {
                    let (a,b) = (sub(p[i],c),sub(p[(i+1)%n],c));
                    norm(cross(a,b))/2.
                }).sum();
                let perimeter: f64 = (0..n).map(|i| harness::distance(p[i],p[(i+1)%n])).sum();
                // the two farthest apart, and how far the rest stand off the line between them
                let mut far = (0.,0,0);
                for i in 0..n { for j in i+1..n {
                    let d = harness::distance(p[i],p[j]);
                    if d > far.0 { far = (d,i,j); }
                } }
                let (span,i,j) = far;
                let axis = sub(p[j],p[i]);
                let off = p.iter().map(|q| if span > 0. { norm(cross(sub(*q,p[i]),axis))/span } else { 0. })
                    .fold(0_f64,f64::max);
                let distinct: std::collections::BTreeSet<u32> = l.iter().copied().collect();
                let slit = off <= 1e-9 && area <= 1e-12;
                if slit { slits += 1; }
                eprintln!("  loop {k}: {n} vertices ({} distinct), area {area:.3e}, perimeter {perimeter:.6}, \
                    span {span:.6}, off the line at most {off:.3e}{}",
                    distinct.len(),if slit { "  <- zero-area slit" } else { "" });
            }
            eprintln!("== {slits} of {} loops enclose no area", unpaired.len());
        }
    });
}

/// Which gate refuses each loop the fill could not close — the standing puzzle the note in
/// `zip_round`'s field pass records. It narrowed the causes to two: a fan triangle the
/// construction reads degenerate, or a loop edge already walked the way the fan needs. Both are
/// asked here with the library's own `space::degenerate`, not a copy of it, and the use counts are
/// reported in both directions per loop edge, so the answer does not depend on what
/// `boundary_loops` counts as boundary. Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn which_gate_refuses_each_unfilled_loop() {
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct};
    use gcs_core::space::{altitude,degenerate};
    use std::collections::BTreeMap;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,unpaired,..} = stage {
            // every directed edge the finished mesh walks, and how often
            let mut uses: BTreeMap<(u32,u32),usize> = Default::default();
            for t in &mesh.triangles { for k in 0..3 { *uses.entry((t[k],t[(k+1)%3])).or_insert(0) += 1; } }
            let count = |a: u32,b: u32| -> usize { uses.get(&(a,b)).copied().unwrap_or(0) };
            eprintln!("== tumbling cylinder: which gate refuses each of {} loops",unpaired.len());
            let (mut by_edge,mut by_flat,mut unexplained) = (0,0,0);
            for (k,l) in unpaired.iter().enumerate() {
                let n = l.len();
                let p: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
                let apex: V3 = std::array::from_fn(|j| p.iter().map(|q| q[j]).sum::<f64>()/n as f64);
                // the field pass's own triangulation: a new apex, so only the loop edge itself can
                // already be walked
                let flat = (0..n).filter(|&i| degenerate(apex,p[i],p[(i+1)%n])).count();
                // the fill must walk each loop edge against the way the mesh walks it
                let taken: Vec<(usize,usize,usize)> = (0..n).map(|i| {
                    let (a,b) = (l[i],l[(i+1)%n]);
                    (i,count(a,b),count(b,a))
                }).filter(|&(_,_,back)| back > 0).collect();
                let least = (0..n).map(|i| altitude(apex,p[i],p[(i+1)%n])).fold(f64::INFINITY,f64::min);
                let cause = if !taken.is_empty() { by_edge += 1; "a loop edge is already walked the fill's way" }
                    else if flat > 0 { by_flat += 1; "a fan triangle reads degenerate" }
                    else { unexplained += 1; "NEITHER GATE — unexplained" };
                eprintln!("  loop {k}: {n} vertices, {flat} flat fan triangles, least fan altitude {least:.3e}, \
                    {} loop edges already walked the fill's way -> {cause}",taken.len());
                for &(i,f,b) in taken.iter().take(3) {
                    eprintln!("      edge {i} ({},{}) : walked {f} forward, {b} backward",l[i],l[(i+1)%n]);
                }
            }
            eprintln!("== {by_edge} refused by an already-walked edge, {by_flat} by a flat fan triangle, \
                {unexplained} by neither");
        }
    });
}

/// What the field says of every loop the zip left, which is the gate `which_gate_refuses_each_
/// unfilled_loop` showed must be the one refusing all but the slits: 30 of the 33 reach neither
/// of the two causes `zip_round`'s note names, so they are declined one gate earlier, by
/// `closeable`. Asked with the construction's own tolerances (`judge_tolerance`,
/// `vertex_tolerance`, `reach`), so this is the pipeline's question and not a similar one.
/// `Open` means the field found no boundary across the span: the surface is missing rather than
/// the stitching wrong, and only the tracer can supply it. Nothing is built or changed here.
#[test]
#[ignore]
fn what_the_field_says_of_each_unfilled_loop() {
    use gcs_core::solid::MaterialField;
    use gcs_core::solid::swept_boundary::{FieldJudge,Span,Stage,SweptBoundaryOptions,construct,loop_span};
    use gcs_core::space::stable_normal;
    use std::collections::BTreeMap;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    // the loops as the construction left them, with the per-edge normals `zip_round` reads off the
    // triangle owning each loop edge; copied out so the judge may be asked after the run
    let mut loops: Vec<(Vec<V3>,Vec<V3>)> = Vec::new();
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,unpaired,..} = stage {
            let mut owner: BTreeMap<(u32,u32),usize> = Default::default();
            for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { owner.insert((t[k],t[(k+1)%3]),i); } }
            for l in unpaired.iter() {
                let n = l.len();
                let points: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
                let normals: Vec<V3> = (0..n).map(|k| {
                    let (a,b) = (l[k],l[(k+1)%n]);
                    owner.get(&(a,b)).and_then(|&t| {
                        let [p,q,r] = mesh.triangles[t].map(|v| mesh.vertices[v as usize]);
                        stable_normal(p,q,r)
                    }).unwrap_or([0.;3])
                }).collect();
                loops.push((points,normals));
            }
        }
    });
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,options.judge_tolerance(),4000,1000,4096);
    let (epsilon,reach) = (options.vertex_tolerance(),options.reach());
    let (mut spanned,mut open,mut unresolved,mut refused) = (0,0,0,0);
    eprintln!("== tumbling cylinder: what the field says of each of {} refused loops",loops.len());
    for (k,(points,normals)) in loops.iter().enumerate() {
        let round = |p: &V3| p.map(|x| (x*1e4).round()/1e4);
        let said = match loop_span(&mut judge,points,normals,epsilon,reach) {
            Ok(Span::Spanned) => { spanned += 1; "Spanned — a hole the fill is authorised to close".to_string() }
            Ok(Span::Open {at}) => { open += 1; format!("Open at {:?} — no boundary there, surface genuinely missing",round(&at)) }
            Ok(Span::Unresolved {at}) => { unresolved += 1; format!("Unresolved at {:?} — no sign at the full budget",round(&at)) }
            Err(e) => { refused += 1; format!("the judge refused: {e:?}") }
        };
        eprintln!("  loop {k}: {} vertices -> {said}",points.len());
    }
    eprintln!("== {spanned} Spanned, {open} Open, {unresolved} Unresolved, {refused} refused by the judge");
}

/// Which of `zip_round`'s passes claims each loop, and how much of the band it claimed it with is
/// actually laid. The fill gates its fan all-or-nothing through `takes`; the pairing and the slit
/// pass mark both loops paired and only then call `lay`, which declines a triangle silently. A
/// loop claimed by a band that lays nothing is consumed: it never reaches the field pass, in this
/// round or any later one, since the pairing is re-decided the same way each time. That would
/// explain the nine loops the field calls `Spanned` and neither recorded cause refuses.
///
/// The three passes are replicated here in the library's own order, with `apart` and the winding
/// copied from `stitch.rs` and `zip_loops`/`zip_polylines`/`degenerate` called rather than copied.
/// Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn which_pass_consumes_each_loop_the_field_would_fill() {
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct,zip_loops};
    use gcs_core::solid::zip_polylines;
    use gcs_core::space::degenerate;
    use std::collections::BTreeSet;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let within = 0.5; // `rim_zip` is given the spacing as its reach
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,unpaired,..} = stage {
            let loops: Vec<Vec<u32>> = unpaired.to_vec();
            let vertices = &mesh.vertices;
            let points: Vec<Vec<V3>> = loops.iter().map(|l| l.iter().map(|&v| vertices[v as usize]).collect()).collect();
            let mut walked: BTreeSet<(u32,u32)> = Default::default();
            for t in &mesh.triangles { for k in 0..3 { walked.insert((t[k],t[(k+1)%3])); } }
            let mut boundary: BTreeSet<(u32,u32)> = Default::default();
            for l in &loops { for k in 0..l.len() { boundary.insert((l[k],l[(k+1)%l.len()])); } }
            let wind = |tri: [u32;3]| -> [u32;3] {
                let flip = (0..3).find_map(|k| {
                    let (p,q) = (tri[k],tri[(k+1)%3]);
                    if boundary.contains(&(p,q)) { Some(true) } else if boundary.contains(&(q,p)) { Some(false) } else { None }
                }).unwrap_or(false);
                let mut tri = tri; if flip { tri.swap(1,2); } tri
            };
            let flat_or_used = |walked: &BTreeSet<(u32,u32)>,tri: [u32;3]| -> bool {
                let tri = wind(tri);
                if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] { return true; }
                let [p,q,r] = tri.map(|v| vertices[v as usize]);
                degenerate(p,q,r) || (0..3).any(|k| walked.contains(&(tri[k],tri[(k+1)%3])))
            };
            // The corner sets the mesh already has. `dedupe` runs after every round and drops a
            // triangle repeating one, and a band wound the other way round an existing triangle's
            // three vertices passes `lay`'s directed-edge test (none of its directed edges is
            // walked) only to be dropped again — so such a band is laid and undone every round,
            // and the loop it closed comes back.
            let existing: BTreeSet<[u32;3]> = mesh.triangles.iter().map(|t| { let mut k = *t; k.sort(); k }).collect();
            // `lay` as the library lays: how much of the band it takes, and how much of that
            // `dedupe` would then drop
            // `wind` decides a band triangle's sense from the first of its edges that lies on a
            // boundary loop, and falls through to **no flip at all** when none does
            // (`unwrap_or(false)`), which leaves that triangle's winding arbitrary. Measured: 243
            // of the zip's 471 triangles in the finished mesh are wound inwards — 52%, against
            // 1–3% on every traced sheet — so this counts how often it has nothing to key on.
            let unkeyed = |tri: [u32;3]| -> bool {
                !(0..3).any(|k| {
                    let (p,q) = (tri[k],tri[(k+1)%3]);
                    boundary.contains(&(p,q)) || boundary.contains(&(q,p))
                })
            };
            let lay = |walked: &mut BTreeSet<(u32,u32)>,band: &[[u32;3]]| -> (usize,usize) {
                let (mut laid,mut doubled) = (0,0);
                for &tri in band {
                    if flat_or_used(walked,tri) { continue; }
                    if unkeyed(tri) { eprintln!("      a band triangle with no boundary edge to \
                        wind by: {:?}",tri.map(|v| vertices[v as usize].map(|x| (x*1e3).round()/1e3))); }
                    let tri = wind(tri);
                    for k in 0..3 { walked.insert((tri[k],tri[(k+1)%3])); }
                    let mut key = tri; key.sort();
                    if existing.contains(&key) { doubled += 1; }
                    laid += 1;
                }
                (laid,doubled)
            };
            let segment = |p: V3,x: V3,y: V3| -> f64 {
                let d = [y[0]-x[0],y[1]-x[1],y[2]-x[2]]; let w = [p[0]-x[0],p[1]-x[1],p[2]-x[2]];
                let l = d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
                let f = if l > 0. { ((w[0]*d[0]+w[1]*d[1]+w[2]*d[2])/l).clamp(0.,1.) } else { 0. };
                harness::distance(p,[x[0]+f*d[0],x[1]+f*d[1],x[2]+f*d[2]])
            };
            let apart = |a: &[V3],b: &[V3]| -> f64 {
                let to = |from: &[V3],onto: &[V3]| from.iter().map(|p| (0..onto.len())
                    .map(|k| segment(*p,onto[k],onto[(k+1)%onto.len()])).fold(f64::INFINITY,f64::min)).fold(0_f64,f64::max);
                to(a,b).max(to(b,a))
            };
            let boxes: Vec<(V3,V3)> = points.iter().map(|p| p.iter()
                .fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),q|
                    (std::array::from_fn(|k| lo[k].min(q[k])),std::array::from_fn(|k| hi[k].max(q[k]))))).collect();
            let gap = |i: usize,j: usize| -> f64 { let ((a0,a1),(b0,b1)) = (boxes[i],boxes[j]);
                (0..3).map(|k| (b0[k]-a1[k]).max(a0[k]-b1[k]).max(0.).powi(2)).sum::<f64>().sqrt() };
            let simple = |l: &Vec<u32>| l.len() >= 3 && { let mut s = l.clone(); s.sort_unstable(); s.windows(2).all(|w| w[0] != w[1]) };
            let mut paired = vec![false;loops.len()];
            let mut claimed: Vec<Option<(&str,usize,usize,usize)>> = vec![None;loops.len()];
            // the small-hole fill, gated all-or-nothing
            for i in 0..loops.len() {
                if paired[i] || !simple(&loops[i]) || loops[i].len() > 4 { continue; }
                let (l,n) = (&loops[i],loops[i].len());
                let fan: Vec<[u32;3]> = (1..n-1).map(|k| [l[0],l[k+1],l[k]]).collect();
                if fan.iter().any(|&t| flat_or_used(&walked,t)) { continue; }
                paired[i] = true;
                let (laid,doubled) = lay(&mut walked,&fan);
                claimed[i] = Some(("fill",fan.len(),laid,doubled));
            }
            // the pairing, which commits before it lays
            for i in 0..loops.len() {
                if paired[i] || !simple(&loops[i]) { continue; }
                let candidates: Vec<(f64,usize)> = (0..loops.len())
                    .filter(|&j| j != i && !paired[j] && simple(&loops[j]) && gap(i,j) <= within*(1.+1e-9))
                    .map(|j| (apart(&points[i],&points[j]),j)).filter(|(d,_)| *d <= within).collect();
                let least = candidates.iter().map(|c| c.0).fold(f64::INFINITY,f64::min);
                let Some((_,j)) = candidates.into_iter().find(|c| c.0 <= least+1e-9*within.max(1.)) else { continue };
                let band = zip_loops(&loops[i],&loops[j],vertices);
                let (laid,doubled) = lay(&mut walked,&band);
                // as the library now commits: only on a band that laid something
                if laid == 0 { continue; }
                paired[i] = true; paired[j] = true;
                claimed[i] = Some(("pairing",band.len(),laid,doubled));
                claimed[j] = Some(("pairing",band.len(),laid,doubled));
            }
            // the slit pass, which commits before it lays too
            let corners_of = |p: &[V3]| -> Vec<usize> {
                let n = p.len();
                (0..n).filter(|&k| {
                    let (a,b,c) = (p[(k+n-1)%n],p[k],p[(k+1)%n]);
                    let (u,w) = ([b[0]-a[0],b[1]-a[1],b[2]-a[2]],[c[0]-b[0],c[1]-b[1],c[2]-b[2]]);
                    let (lu,lw) = (harness::distance(a,b),harness::distance(b,c));
                    lu > 0. && lw > 0. && (u[0]*w[0]+u[1]*w[1]+u[2]*w[2])/(lu*lw) < -0.5
                }).collect()
            };
            for i in 0..loops.len() {
                if paired[i] || loops[i].len() < 4 || !simple(&loops[i]) { continue; }
                let (l,p,n) = (&loops[i],&points[i],loops[i].len());
                let corners = corners_of(p);
                let (c0,c1) = if corners.len() == 2 { (corners[0],corners[1]) } else {
                    let mut best = (f64::NEG_INFINITY,0,0);
                    for a in 0..n { for b in a+1..n { let d = harness::distance(p[a],p[b]); if d > best.0 { best = (d,a,b); } } }
                    (best.1,best.2)
                };
                if c0 == c1 { continue; }
                let arc_a: Vec<u32> = (c0..=c1).map(|k| l[k]).collect();
                let arc_b: Vec<u32> = (c1..=c0+n).map(|k| l[k%n]).rev().collect();
                let pts = |arc: &[u32]| arc.iter().map(|&v| vertices[v as usize]).collect::<Vec<_>>();
                let (pa,pb) = (pts(&arc_a),pts(&arc_b));
                if apart(&pa,&pb) > within { continue; }
                let band: Vec<[u32;3]> = zip_polylines(&pa,&pb,false).into_iter()
                    .map(|t| t.map(|(on_b,k)| if on_b { arc_b[k as usize] } else { arc_a[k as usize] })).collect();
                let (laid,doubled) = lay(&mut walked,&band);
                if laid == 0 { continue; }
                paired[i] = true;
                claimed[i] = Some(("slit",band.len(),laid,doubled));
            }
            eprintln!("== tumbling cylinder: which pass claims each of {} loops",loops.len());
            let mut undone = 0;
            for (k,c) in claimed.iter().enumerate() {
                match c {
                    Some((pass,band,laid,doubled)) => {
                        let all = *doubled == *laid && *laid > 0;
                        if all { undone += 1; }
                        eprintln!("  loop {k}: {} vertices -> the {pass} pass lays {laid} of a band of {band}, \
                            {doubled} repeating a triangle the mesh has{}",loops[k].len(),
                            if all { "  <- UNDONE by dedupe before the next round" } else { "" });
                    }
                    None => eprintln!("  loop {k}: {} vertices -> no pass takes it; left for the field",loops[k].len()),
                }
            }
            eprintln!("== {undone} loops whose whole band dedupe would drop; {} taken by no pass at all",
                claimed.iter().filter(|c| c.is_none()).count());
        }
    });
}

/// The gate on two defects in the stitch, each measured before it was fixed. The pairing and the
/// slit pass marked their loops paired *before* laying the band, so a band `lay` declined whole
/// consumed those loops for every later pass and — the pairing being decided the same way each
/// round — for every later round too; and `lay` took a facet the mesh already had, which `dedupe`
/// then dropped, so the loop it had closed came back and only the count of rounds ended it.
/// Together they left the tumbling cylinder with ten loops the field itself calls `Spanned` —
/// holes it authorises the fill to close — refused all the same, which is the "standing puzzle"
/// the note in `zip_round`'s field pass used to record.
///
/// A third defect was then found by the same route and fixed: `collapse_needles` had no guard
/// against *stretching* an edge, and was rewriting an existing sheet's triangle out to 0.9859 —
/// twice the spacing — which the stage walk pinned by reporting the longest edge *and the sheet
/// that laid it* (a rewritten triangle keeps its own sheet id; a band the zip lays carries
/// `u32::MAX`). Disabling each pass in turn settled it: 0.5330 with the collapse off, 0.9964 with
/// the weld off.
///
/// The property is the gate, not a count: no loop the field says is a hole may be left open. Of
/// the tumbling cylinder's loops, 10 of 33 were before the stitch fixes and 0 of 7 after; the count
/// bound below comes down with each fix so a regression cannot hide behind a stale limit.
///
/// **It went up once, to 21, and that was not a regression.** Cutting each cap along its own tool
/// edges (`caps`, 2026-09-12) recovered the `z = 0` band the cap had been dropping wholesale: the
/// volume went 6.6540 to 9.2929 against the reference's 9.4786 — from 30% short to 2.0% — the mesh
/// 3288 triangles to 6661, and the reference's uncovered area 6.445 to about 2.0, its two peaks at
/// angle 0° and 180° becoming its smallest bins. New surface brings new boundary for the zip to
/// close, and it closed less of it: 21 small loops (3 to 7 vertices) where there were 7. So the
/// bound records recovered surface, not a worse stitch of the same surface, and it must resume
/// coming down as the stitch is fixed.
///
/// **With the count no longer short-circuiting it, the property then failed — and fixing that is
/// what the paragraph below records.** Loops 13 and 14 — both 3-vertex — came back `Spanned`: holes
/// the field itself calls holes and the zip left open. So the cap fix recovered the band and
/// exposed two zero-area triangles the fill declines. `which_gate_refuses_each_unfilled_loop` names the gate: six of the
/// 21 are 3-vertex and refused because *a fan triangle reads degenerate*, least altitudes 2.575e-17,
/// 4.189e-17, 9.181e-11, 5.623e-12, 1.202e-10 and 4.446e-12, and the two the field calls `Spanned`
/// are the two at ~1e-17. `lay` refusing an already-walked edge is **not** the cause — 0 of 21 are
/// refused that way. A zero-area hole must not be filled with a degenerate triangle, so the fix is
/// upstream: a needle to weld, a T-junction to split, or a sliver to drop. This is the gate doing
/// its job, not a limit to raise again.
///
/// **The fix is one last split over the finished mesh, and *where* it goes was settled by
/// measurement rather than by argument.** Applied to the finished mesh, a second `split_where` made
/// 4 splits, took the three-vertex loops 6 to 2, and left both survivors `Open` — 0 `Spanned`.
/// Moved *inside* `rim_zip`'s round, where it interleaves with the zip and changes what the zip
/// then pairs, it was worse on every count: loops 21 to 23, triangles 6661 to 6701, volume 9.2929
/// to 9.2581, and one loop the field still called a hole. Run once **after** the rounds it
/// reproduces the experiment exactly: **17 loops, 6665 triangles, volume 9.2929 against the
/// reference's 9.4786, and no loop the field calls a hole.** The property passes, the suite is
/// green at 1209, and the prism, lens, turned box and dumbbell stay byte-identical. The in-round
/// placement is recorded with its numbers so it is not tried again.
#[test]
fn no_loop_the_field_calls_a_hole_is_left_unfilled() {
    use gcs_core::solid::MaterialField;
    use gcs_core::solid::swept_boundary::{FieldJudge,Span,Stage,SweptBoundaryOptions,construct,loop_span};
    use gcs_core::space::stable_normal;
    use std::collections::BTreeMap;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    // the loops the zip left, with the per-edge normals `zip_round` reads off each loop edge's
    // owning triangle, copied out so the judge may be asked after the run
    let mut loops: Vec<(Vec<V3>,Vec<V3>)> = Vec::new();
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,unpaired,..} = stage {
            let mut owner: BTreeMap<(u32,u32),usize> = Default::default();
            for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { owner.insert((t[k],t[(k+1)%3]),i); } }
            for l in unpaired.iter() {
                let n = l.len();
                let points: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
                let normals: Vec<V3> = (0..n).map(|k| {
                    let (a,b) = (l[k],l[(k+1)%n]);
                    owner.get(&(a,b)).and_then(|&t| {
                        let [p,q,r] = mesh.triangles[t].map(|v| mesh.vertices[v as usize]);
                        stable_normal(p,q,r)
                    }).unwrap_or([0.;3])
                }).collect();
                loops.push((points,normals));
            }
        }
    });
    assert!(loops.len() <= 17,"{} loops refused, 17 when measured",loops.len());
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,options.judge_tolerance(),4000,1000,4096);
    let (epsilon,reach) = (options.vertex_tolerance(),options.reach());
    let mut holes: Vec<(usize,usize)> = Vec::new();
    for (k,(points,normals)) in loops.iter().enumerate() {
        if let Ok(Span::Spanned) = loop_span(&mut judge,points,normals,epsilon,reach) { holes.push((k,points.len())); }
    }
    assert!(holes.is_empty(),"the field calls these loops holes and the zip left them open \
        (index, vertices): {holes:?}");
}

/// Where the field actually puts each refused loop's span, which decides whether the remaining
/// work is the tracer's at all. `loop_span` probes along the *owner triangle's* normal over one
/// sagitta, so its `Open` says only "no sign change along that direction within 0.02" — which an
/// absent surface gives, and so does a probe direction running near-tangent to a boundary that is
/// present. These loops sit at radius 1 from the tumble axis, the fold locus, where exactly that
/// can happen. So this asks two things the verdict cannot: the field's plain sign at every fan
/// centre (all `Material` puts the mesh's boundary *inside* material, so the missing surface is
/// somewhere else; all `Exterior` puts it in open air; `Near` puts the boundary right there), and
/// whether the first `Open` centre finds a boundary along the loop's own plane normal or its
/// tangents, at one sagitta and at four. Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn where_the_field_puts_each_open_loops_span() {
    use gcs_core::solid::MaterialField;
    use gcs_core::solid::swept_boundary::{FieldJudge,Projection,Sign,Stage,SweptBoundaryOptions,construct};
    use gcs_core::space::stable_normal;
    use std::collections::{BTreeMap,BTreeSet};
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let mut loops: Vec<(Vec<V3>,Vec<V3>)> = Vec::new();
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,unpaired,..} = stage {
            let mut owner: BTreeMap<(u32,u32),usize> = Default::default();
            for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { owner.insert((t[k],t[(k+1)%3]),i); } }
            for l in unpaired.iter() {
                let n = l.len();
                let points: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
                let normals: Vec<V3> = (0..n).map(|k| {
                    let (a,b) = (l[k],l[(k+1)%n]);
                    owner.get(&(a,b)).and_then(|&t| {
                        let [p,q,r] = mesh.triangles[t].map(|v| mesh.vertices[v as usize]);
                        stable_normal(p,q,r)
                    }).unwrap_or([0.;3])
                }).collect();
                loops.push((points,normals));
            }
        }
    });
    let sub = |a: V3,b: V3| -> V3 { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] };
    let cross = |a: V3,b: V3| -> V3 { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] };
    let norm = |a: V3| -> f64 { (a[0]*a[0]+a[1]*a[1]+a[2]*a[2]).sqrt() };
    let unit = |a: V3| -> Option<V3> { let l = norm(a); (l > 0.).then(|| [a[0]/l,a[1]/l,a[2]/l]) };
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,options.judge_tolerance(),4000,1000,4096);
    let (epsilon,reach) = (options.vertex_tolerance(),options.reach());
    let word = |s: Sign| match s {
        Sign::Material => "material",Sign::Exterior => "exterior",
        Sign::Near {..} => "near",Sign::Unresolved => "unresolved" };
    eprintln!("== tumbling cylinder: where the field puts each of {} refused loops",loops.len());
    for (k,(points,normals)) in loops.iter().enumerate() {
        let n = points.len();
        let centroid: V3 = std::array::from_fn(|j| points.iter().map(|p| p[j]).sum::<f64>()/n as f64);
        // the loop's own plane normal, from the fan's area vector
        let area: V3 = (0..n).fold([0.;3],|acc,i| {
            let c = cross(sub(points[i],centroid),sub(points[(i+1)%n],centroid));
            [acc[0]+c[0],acc[1]+c[1],acc[2]+c[2]] });
        let plane = unit(area);
        let mut tally: BTreeMap<&str,usize> = Default::default();
        let mut open_at: Option<(V3,V3)> = None;
        for i in 0..n {
            let (a,b) = (points[i],points[(i+1)%n]);
            let at: V3 = std::array::from_fn(|j| (centroid[j]+a[j]+b[j])/3.);
            if let Ok((s,_)) = judge.sign(at) { *tally.entry(word(s)).or_insert(0) += 1; }
            let m = normals[i];
            if open_at.is_none() && norm(m) > 0. {
                if matches!(judge.project(at,m,epsilon,reach),Ok(Projection::Inner)|Ok(Projection::Positive)) {
                    open_at = Some((at,m));
                }
            }
        }
        // The fan centre is pulled toward the loop's centroid, which for a curved loop dips inside
        // a convex surface whatever the loop sits on — so the loop's own vertices and edge
        // midpoints are asked too. They lie on the mesh. Vertices `near` with centres `material`
        // means the loop is on the boundary and the centroid dips in, which would make
        // `loop_span`'s verdict an artefact of its own construction; vertices `material` too means
        // the mesh's boundary has genuinely run inside the solid.
        // `Material` means only "deeper than the judge's near band", which is a quarter-sagitta
        // here — and a chord across a curved boundary lies up to a **whole sagitta** inside it. So
        // the sign alone says nothing: what matters is the depth, and only a depth past the
        // sagitta is deeper than the mesh's own chords can explain.
        let (mut at_v,mut at_m): (BTreeMap<&str,usize>,BTreeMap<&str,usize>) = Default::default();
        let (mut deep_v,mut deep_m) = (0_f64,0_f64);
        let (mut past_v,mut past_m) = (0,0);
        for i in 0..n {
            if let Ok((s,enc)) = judge.sign(points[i]) {
                *at_v.entry(word(s)).or_insert(0) += 1;
                if matches!(s,Sign::Material) {
                    let d = -enc[1]; deep_v = deep_v.max(d); if d > SAGITTA { past_v += 1; }
                }
            }
            let mid: V3 = std::array::from_fn(|j| 0.5*(points[i][j]+points[(i+1)%n][j]));
            if let Ok((s,enc)) = judge.sign(mid) {
                *at_m.entry(word(s)).or_insert(0) += 1;
                if matches!(s,Sign::Material) {
                    let d = -enc[1]; deep_m = deep_m.max(d); if d > SAGITTA { past_m += 1; }
                }
            }
        }
        eprintln!("      depths: vertices at most {deep_v:.4} inside ({past_v} past the sagitta), \
            midpoints at most {deep_m:.4} ({past_m} past)");
        let show = |t: &BTreeMap<&str,usize>| t.iter().map(|(w,c)| format!("{c} {w}")).collect::<Vec<_>>().join(", ");
        let said: Vec<String> = tally.iter().map(|(w,c)| format!("{c} {w}")).collect();
        eprintln!("  loop {k}: {n} vertices; fan centres [{}]; the loop's own vertices [{}]; edge midpoints [{}]",
            said.join(", "),show(&at_v),show(&at_m));
        // What each fill triangle's *own* normal says. This is the question `certify` will ask of
        // the laid triangle, and it is **not** the one `loop_span` asks: that probes each fan
        // centre along the *owner* triangle's normal, which at these rims runs along the boundary
        // rather than across it, so it cannot bracket however near the boundary is.
        {
            let (mut bracketed,mut inner,mut other) = (0,0,0);
            for i in 0..n {
                let (a,b) = (points[i],points[(i+1)%n]);
                let at: V3 = std::array::from_fn(|j| (centroid[j]+a[j]+b[j])/3.);
                let Some(own) = stable_normal(centroid,a,b) else { other += 1; continue };
                match judge.project(at,own,epsilon,reach) {
                    Ok(Projection::Kept {..}) | Ok(Projection::Moved {..}) => bracketed += 1,
                    Ok(Projection::Inner) | Ok(Projection::Positive) => inner += 1,
                    _ => other += 1,
                }
            }
            eprintln!("      along each fan triangle's OWN normal: {bracketed} of {n} bracket the \
                boundary, {inner} inner or positive, {other} else");
        }
        // Would a fan from an apex *projected onto the boundary* do? The fill lays exactly the fan
        // `loop_span` judges, so this says whether moving the apex onto the surface is enough or a
        // wide loop's fan must be subdivided to follow it.
        if let Some(p) = plane {
            let apex = match judge.project(centroid,p,epsilon,reach*8.) {
                Ok(Projection::Kept {..}) => Some(centroid),
                Ok(Projection::Moved {point,..}) => Some(point),
                _ => None,
            };
            match apex {
                None => eprintln!("      an apex on the boundary: the centroid finds none within 8 sagittas"),
                Some(a) => {
                    let (mut near,mut deep,mut worst) = (0,0,0_f64);
                    for i in 0..n {
                        let c: V3 = std::array::from_fn(|j| (a[j]+points[i][j]+points[(i+1)%n][j])/3.);
                        if let Ok((s,enc)) = judge.sign(c) {
                            match s {
                                Sign::Near {..} => near += 1,
                                Sign::Material => { deep += 1; worst = worst.max(-enc[1]); }
                                _ => {}
                            }
                        }
                    }
                    eprintln!("      an apex on the boundary (moved {:.4}): {near} of {n} fan centres near, \
                        {deep} material (deepest {worst:.4})",harness::distance(centroid,a));
                }
            }
        }
        // and whether some other direction finds the boundary the owner's normal missed
        let Some((at,m)) = open_at else { continue };
        let mut dirs: Vec<(&str,V3)> = vec![("owner normal",m)];
        if let Some(p) = plane { dirs.push(("loop plane normal",p));
            if let Some(t) = unit(cross(p,m)) { dirs.push(("tangent",t)); }
            if let Some(t) = unit(cross(m,cross(p,m))) { dirs.push(("in-plane",t)); } }
        let mut found: BTreeSet<String> = Default::default();
        for (name,d) in dirs {
            for (mult,label) in [(1.,"1 sagitta"),(4.,"4 sagittas")] {
                match judge.project(at,d,epsilon,reach*mult) {
                    Ok(Projection::Kept {radius}) => { found.insert(format!("{name} at {label}: KEPT within {radius:.4}")); }
                    Ok(Projection::Moved {by,radius,..}) => { found.insert(format!("{name} at {label}: MOVED by {by:.4} within {radius:.4}")); }
                    _ => {}
                }
            }
        }
        if found.is_empty() { eprintln!("      the span at {:?}: no direction finds a boundary",at.map(|x| (x*1e4).round()/1e4)); }
        else { for f in &found { eprintln!("      the span at {:?}: {f}",at.map(|x| (x*1e4).round()/1e4)); } }
    }
}

/// Which stage first hands on a boundary edge that lies inside the material. Eight of the
/// tumbling cylinder's thirteen refused loops sit within the solid — their own vertices and every
/// edge midpoint read `material` — and a boundary edge should lie *on* the boundary at every stage
/// from `Clipped` onward, a rim being a crease and a crease being on the surface. So the first
/// stage whose boundary-edge midpoints read `material` is where the defect enters, which is what
/// `hygiene` does for the manifold faults it knows and cannot do for this one.
///
/// Its own judge, since `construct` owns the one it uses, and at most `PROBES` edges a stage taken
/// on a stride, so the run stays near the seconds the case already costs. Nothing is built or
/// changed here; it only measures.
#[test]
#[ignore]
fn which_stage_first_leaves_a_boundary_edge_inside_the_material() {
    use gcs_core::solid::MaterialField;
    use gcs_core::solid::swept_boundary::{FieldJudge,KeptMesh,Sign,Stage,SweptBoundaryOptions,construct};
    use std::collections::BTreeMap;
    const PROBES: usize = 48;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,options.judge_tolerance(),4000,1000,4096);
    eprintln!("== tumbling cylinder: boundary-edge midpoints by stage (a rim lies on the boundary)");
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        let named: Option<(&str,&KeptMesh)> = match &stage {
            Stage::Clipped {mesh,..} => Some(("clipped",mesh)),
            Stage::Merged {mesh,..} => Some(("merged",mesh)),
            Stage::Kept {kept,..} => Some(("kept",kept)),
            Stage::Unioned {mesh,..} => Some(("unioned",mesh)),
            Stage::Uncovered {mesh,..} => Some(("uncovered",mesh)),
            Stage::Welded {mesh,..} => Some(("welded",mesh)),
            Stage::Split {mesh} => Some(("split",mesh)),
            Stage::Zipped {mesh,..} => Some(("zipped",mesh)),
            _ => None,
        };
        let Some((name,mesh)) = named else { return };
        // the undirected edges exactly one triangle walks
        // Folds (one directed edge walked twice) and over-used edges. `hygiene` reports these, but
        // the harness prints it for only five stages — clipped, merged, kept, unioned and without
        // overlaps — and never for welded, split or zipped, so whether a fold reaches the boundary
        // the zip must close has gone unmeasured. Measured at those five: 15 folds at clipped,
        // merged and kept, falling to 4 at unioned, where the planar union retriangulates the
        // coplanar fragments and rebuilds eleven of them.
        let mut directed: BTreeMap<(u32,u32),usize> = Default::default();
        for t in &mesh.triangles { for k in 0..3 { *directed.entry((t[k],t[(k+1)%3])).or_insert(0) += 1; } }
        let folds = directed.values().filter(|n| **n >= 2).count();
        let over = {
            let mut u: BTreeMap<(u32,u32),usize> = Default::default();
            for (&(a,b),n) in &directed { *u.entry((a.min(b),a.max(b))).or_insert(0) += n; }
            u.values().filter(|n| **n > 2).count()
        };
        let mut uses: BTreeMap<(u32,u32),usize> = Default::default();
        for t in &mesh.triangles { for k in 0..3 {
            let (a,b) = (t[k],t[(k+1)%3]);
            *uses.entry((a.min(b),a.max(b))).or_insert(0) += 1;
        } }
        let edges: Vec<(u32,u32)> = uses.into_iter().filter(|(_,n)| *n == 1).map(|(e,_)| e).collect();
        if edges.is_empty() { eprintln!("  {name}: no boundary at all"); return; }
        let stride = (edges.len()/PROBES).max(1);
        let mut tally: BTreeMap<&str,usize> = Default::default();
        let mut deepest: Option<(f64,V3)> = None;
        // how many lie deeper than a chord across a curved boundary could
        let mut past = 0;
        for &(a,b) in edges.iter().step_by(stride).take(PROBES) {
            let (p,q) = (mesh.vertices[a as usize],mesh.vertices[b as usize]);
            let mid: V3 = std::array::from_fn(|j| 0.5*(p[j]+q[j]));
            let Ok((s,enclosure)) = judge.sign(mid) else { continue };
            *tally.entry(match s {
                Sign::Material => "material",Sign::Exterior => "exterior",
                Sign::Near {..} => "near",Sign::Unresolved => "unresolved" }).or_insert(0) += 1;
            // how far inside the deepest one reads, by its own enclosure's upper end
            if matches!(s,Sign::Material) {
                let depth = -enclosure[1];
                // only a depth past the sagitta is deeper than a chord across a curved boundary
                if depth > SAGITTA { past += 1; }
                if deepest.map_or(true,|(d,_)| depth > d) { deepest = Some((depth,mid)); }
            }
        }
        // And the longest edge this stage's mesh walks, with the sheet that laid it. The finished
        // mesh's three longest are all sheet 6's, in the plane x = 2.0 where that sheet is wholly
        // flat — a plane `planar_union` retriangulates — so the stage at which the longest edge
        // jumps says who made it. The tracer is excluded: sheets 6 and 7 are structurally
        // identical, 54 points in 9 columns of 6, with no jump between neighbouring columns.
        let mut long = (0_f64,u32::MAX);
        for (i,t) in mesh.triangles.iter().enumerate() {
            let p = t.map(|v| mesh.vertices[v as usize]);
            for k in 0..3 {
                let d = harness::distance(p[k],p[(k+1)%3]);
                if d > long.0 { long = (d,mesh.sheet[i]); }
            }
        }
        let by = if long.1 == u32::MAX { "the zip".to_string() } else { format!("sheet {}",long.1) };
        let said = tally.iter().map(|(w,c)| format!("{c} {w}")).collect::<Vec<_>>().join(", ");
        let worst = match deepest {
            Some((d,p)) => format!("; deepest inside {d:.4} at {:?}",p.map(|x| (x*1e4).round()/1e4)),
            None => String::new() };
        eprintln!("  {name}: {} boundary edges, {} probed -> [{said}]{worst}; {past} past the sagitta; \
            longest edge anywhere {:.4} by {by}; {folds} folds, {over} over-used",
            edges.len(),edges.len().min(PROBES),long.0);
    });
}

/// How far apart the two sides of each wide loop run. Loops 0 and 1 carry 14.20 of the 14.26
/// refused area and are the one reading that survived every refutation: their rims lie **on** the
/// boundary (41 of 51 and 30 of 35 vertices read `near`) while no fan triangle brackets along any
/// direction tried, and loop 0's perimeter is 13.6 against a span of 2.81 where a circle of that
/// span would measure about 8.8. That is the shape of a **seam folded back on itself**, not a
/// hole — and the distinction is decided by one number: how far the other side lies.
///
/// Within the snap (0.005) the two sides should be welded into one. At a tenth or more apart,
/// joining them invents area, which is how the side-to-side sewing tried earlier failed — the
/// volume rose by 0.036. So this reports, for every vertex, the distance to the nearest stretch of
/// its own walk at least four steps away, against the snap, the sagitta, the junction tolerance
/// and the spacing. Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn how_the_wide_seams_fold_back_on_themselves() {
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct};
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let segment = |p: V3,x: V3,y: V3| -> f64 {
        let d = [y[0]-x[0],y[1]-x[1],y[2]-x[2]]; let w = [p[0]-x[0],p[1]-x[1],p[2]-x[2]];
        let l = d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
        let f = if l > 0. { ((w[0]*d[0]+w[1]*d[1]+w[2]*d[2])/l).clamp(0.,1.) } else { 0. };
        harness::distance(p,[x[0]+f*d[0],x[1]+f*d[1],x[2]+f*d[2]])
    };
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,unpaired,..} = stage {
            eprintln!("== tumbling cylinder: how far the other side of each wide loop lies");
            for (k,l) in unpaired.iter().enumerate() {
                let n = l.len();
                if n < 20 { continue; }
                let p: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
                let step = |a: usize,b: usize| -> usize { let d = (a as i64-b as i64).unsigned_abs() as usize; d.min(n-d) };
                let mut d: Vec<f64> = Vec::with_capacity(n);
                for i in 0..n {
                    let mut least = f64::INFINITY;
                    for j in 0..n {
                        // a stretch of the same walk far enough along it to be the other side
                        if step(i,j) < 4 || step(i,(j+1)%n) < 4 { continue; }
                        least = least.min(segment(p[i],p[j],p[(j+1)%n]));
                    }
                    if least.is_finite() { d.push(least); }
                }
                if d.is_empty() { eprintln!("  loop {k}: {n} vertices; no stretch four steps away"); continue; }
                let mut sorted = d.clone(); sorted.sort_by(f64::total_cmp);
                let count = |t: f64| d.iter().filter(|x| **x <= t).count();
                eprintln!("  loop {k}: {n} vertices; the other side lies within the snap 0.005 for {}, \
                    within a sagitta {SAGITTA} for {}, within the junction 0.04 for {}, within the spacing 0.5 for {} \
                    -> least {:.4}, median {:.4}, most {:.4}",
                    count(0.005),count(SAGITTA),count(0.04),count(0.5),sorted[0],sorted[d.len()/2],sorted[d.len()-1]);
            }
        }
    });
}

/// The longest edges the finished mesh walks, and who laid each. At a spacing of 0.5 and a sagitta
/// of 0.02 no legitimate triangle here spans much more than the spacing, so anything far above it
/// would be a band laid across the solid rather than a piece of its surface.
///
/// It is a positive check, not a lead: the harness's `sides: 2.000000 … 2.732051` on the two wide
/// loops are their **bounding-box extents** (`closed.rs` sorts `hi − lo` over the three axes), not
/// edge lengths — loop 0's box gives exactly 2.000, 2.732 and 1.725 — so nothing there implies a
/// giant triangle. Kept because the question is worth an answer on the record rather than an
/// inference. Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn the_longest_edges_the_mesh_walks() {
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct};
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,..} = stage {
            let who = |s: u32| if s == u32::MAX { "zip".to_string() } else { format!("sheet {s}") };
            let mut longest: Vec<(f64,usize,u32)> = mesh.triangles.iter().enumerate().map(|(i,t)| {
                let e = (0..3).map(|k| harness::distance(mesh.vertices[t[k] as usize],
                    mesh.vertices[t[(k+1)%3] as usize])).fold(0_f64,f64::max);
                (e,i,mesh.sheet[i])
            }).collect();
            longest.sort_by(|a,b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
            eprintln!("== tumbling cylinder: the longest edges the finished mesh walks (spacing 0.5)");
            for (len,i,s) in longest.iter().take(10) {
                eprintln!("  triangle {i} by the {}: longest edge {len:.4}, corners {:?}",
                    who(*s),mesh.triangles[*i].map(|v| mesh.vertices[v as usize].map(|x| (x*1e3).round()/1e3)));
            }
            let over = |t: f64| longest.iter().filter(|(len,_,_)| *len > t).count();
            let zipped = |t: f64| longest.iter().filter(|(len,_,s)| *len > t && *s == u32::MAX).count();
            eprintln!("== of {} triangles: {} have an edge past the spacing (0.5), of which {} are the zip's; \
                {} past twice it, of which {} are the zip's",mesh.triangles.len(),
                over(0.5),zipped(0.5),over(1.0),zipped(1.0));
        }
    });
}

/// Each sheet's coarseness and extent, and how near it comes to the motion's fixed points. The
/// three longest edges in the finished mesh (0.9859, 0.9810, 0.9797 — twice the spacing) all
/// belong to sheet 6, all lie in the plane x = 2.0, and all fan out from (2, 0, 0), which is on
/// the cylinder's wall ((2−3)² = 1) **and** on the tumble axis, so the motion holds it still. The
/// sheets bordering both wide loops are 6, 7, 9 and 10. If those are coarse fans across a disc of
/// radius 1 about the fixed points (2,0,0) and (4,0,0), that explains the shallow material
/// readings: the sweep is second-order deep near a fixed point, so an under-refined fan there has
/// chords lying just inside the surface. Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn how_coarse_each_sheet_is_and_how_near_the_fixed_points() {
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct};
    use std::collections::BTreeMap;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    // where the tool's wall crosses the tumble axis (the world x-axis): held still by the motion
    let fixed: [V3;2] = [[2.,0.,0.],[4.,0.,0.]];
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,..} = stage {
            let mut per: BTreeMap<u32,(usize,f64,f64,V3,V3,f64)> = Default::default();
            for (i,t) in mesh.triangles.iter().enumerate() {
                let p = t.map(|v| mesh.vertices[v as usize]);
                let (mut most,mut sum) = (0_f64,0.);
                for k in 0..3 { let d = harness::distance(p[k],p[(k+1)%3]); most = most.max(d); sum += d/3.; }
                let near = p.iter().map(|q| fixed.iter().map(|f| harness::distance(*q,*f))
                    .fold(f64::INFINITY,f64::min)).fold(f64::INFINITY,f64::min);
                let entry = per.entry(mesh.sheet[i]).or_insert((0,0.,0.,[f64::INFINITY;3],[f64::NEG_INFINITY;3],f64::INFINITY));
                entry.0 += 1; entry.1 = entry.1.max(most); entry.2 += sum;
                for k in 0..3 { for j in 0..3 { entry.3[j] = entry.3[j].min(p[k][j]); entry.4[j] = entry.4[j].max(p[k][j]); } }
                entry.5 = entry.5.min(near);
                let _ = i;
            }
            eprintln!("== tumbling cylinder: each sheet's coarseness (spacing 0.5, sagitta 0.02)");
            for (s,(n,most,total,lo,hi,near)) in &per {
                let who = if *s == u32::MAX { "zip".to_string() } else { format!("sheet {s}") };
                eprintln!("  {who}: {n} triangles, longest edge {most:.4}, mean {:.4}, box {:?}..{:?}, \
                    nearest a fixed point {near:.4}",total/(*n as f64),
                    lo.map(|x| (x*1e3).round()/1e3),hi.map(|x| (x*1e3).round()/1e3));
            }
        }
    });
}

/// Sheets 6 and 7 column by column, which is where the one asymmetry in a symmetric fixture is.
/// They are mirror silhouette patches at the motion's two fixed points, (2,0,0) and (4,0,0), with
/// the same geometry and the same motion — and sheet 6's longest edge is 0.9859, twice the
/// spacing, while sheet 7's is 0.5085. The 9 traced sheets are seed indices 0 to 8; 9 and 10 are
/// the two caps, so this is a question about the tracer's own columns. A strip whose consecutive
/// columns carry very different numbers of points zips into long rungs, which is what `zip_pieces`
/// gives when the simplification thins one end differently. Nothing is built or changed here.
#[test]
#[ignore]
fn sheets_six_and_seven_column_by_column() {
    use gcs_core::solid::swept_boundary::seeds;
    use std::collections::BTreeMap;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let (_,sheets,_) = seeds(&e.sketch,swept,0.5,SAGITTA,&|_| {}).unwrap();
    eprintln!("== tumbling cylinder: {} traced sheets, column by column",sheets.len());
    for (i,s) in sheets.iter().enumerate() {
        let mut per: BTreeMap<u32,usize> = Default::default();
        for &c in &s.column { *per.entry(c).or_insert(0) += 1; }
        let counts: Vec<usize> = per.values().copied().collect();
        let (least,most) = (counts.iter().copied().min().unwrap_or(0),counts.iter().copied().max().unwrap_or(0));
        let (lo,hi) = s.points.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),
            |(lo,hi),q| (std::array::from_fn(|k| lo[k].min(q[k])),std::array::from_fn(|k| hi[k].max(q[k]))));
        eprintln!("  sheet {i}: {} points in {} columns ({least} to {most} each), times {:.4}..{:.4}, \
            closed {}, box {:?}..{:?}",s.points.len(),per.len(),
            s.times.first().copied().unwrap_or(f64::NAN),s.times.last().copied().unwrap_or(f64::NAN),
            s.closed,lo.map(|x| (x*1e3).round()/1e3),hi.map(|x| (x*1e3).round()/1e3));
        // the pair with the asymmetry, spelled out: a jump between neighbours is where the rungs
        // stretch, since a zip must pair a column of many points against one of few
        if i == 6 || i == 7 {
            let line: Vec<String> = per.values().map(|n| n.to_string()).collect();
            eprintln!("      points per column in order: {}",line.join(" "));
            let worst = per.values().copied().collect::<Vec<_>>().windows(2)
                .map(|w| w[0].abs_diff(w[1])).max().unwrap_or(0);
            eprintln!("      the largest jump between neighbouring columns: {worst} points");
        }
    }
}

/// Which triangles are wound inwards, and which sheet owns each. The one loop the field refuses
/// outright does so with `ReversedNormal` at [2.000337, 0.0887, −0.1432] along a direction that is
/// very nearly **+x** — and at the plane x = 2 the solid lies at x > 2, so outward there is −x. A
/// normal pointing +x on that plane is wound into the material, which is a defect and not a probe
/// subtlety: `ReversedNormal` is the judge reporting material outside and exterior inside.
///
/// `certify` would name such a triangle `Reversed`, but this case refuses at `UnpairedRim` before
/// the certificate runs, so nothing has ever reported it. This asks the certificate's own question
/// of every triangle instead — `sides(centroid, n, probe)` is `(Material, Exterior)` for an
/// outward triangle and `(Exterior, Material)` for a reversed one. Nothing is built or changed
/// here; it only measures.
#[test]
#[ignore]
fn which_triangles_are_wound_inwards() {
    use gcs_core::solid::MaterialField;
    use gcs_core::solid::swept_boundary::{FieldJudge,Sign,Stage,SweptBoundaryOptions,construct};
    use gcs_core::space::stable_normal;
    use std::collections::BTreeMap;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,options.judge_tolerance(),4000,1000,4096);
    let probe = options.probe_distance();
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,..} = stage {
            // sheet -> (outward, reversed, undecided), and where the first reversed one sits
            // The certificate does **not** call a triangle reversed on one probe at the full
            // distance: it halves the probe down to the least distance for thin material and
            // reports those as `thin`. Where material is thinner than twice the probe, a probe
            // inside exits the far side and one outside enters other material, so a perfectly
            // outward triangle reads (Exterior, Material). An earlier version of this test used
            // the full probe alone and called 243 of the zip's 471 triangles reversed — with 235
            // of them sharing an edge, walked oppositely, with an outward triangle, which is
            // impossible for a consistently wound surface. That contradiction is what says the
            // test was wrong, not the mesh, so the rule here follows `certify`'s.
            let least = options.least_probe();
            let mut per: BTreeMap<u32,(usize,usize,usize,usize)> = Default::default();
            let mut examples: Vec<(u32,V3,V3)> = Vec::new();
            let mut flipped = 0;
            for (i,t) in mesh.triangles.iter().enumerate() {
                let [a,b,c] = t.map(|v| mesh.vertices[v as usize]);
                let Some(n) = stable_normal(a,b,c) else { continue };
                let centroid: V3 = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
                let entry = per.entry(mesh.sheet[i]).or_insert((0,0,0,0));
                // outward at any distance from the full probe down to the least is outward; one
                // that only stops reading reversed as the probe shrinks is thin, not reversed
                let (mut verdict,mut d,mut narrowed) = (2,probe,false);
                while d >= least {
                    match judge.sides(centroid,n,d) {
                        Ok((Sign::Material,Sign::Exterior)) => { verdict = 0; break; }
                        Ok((Sign::Exterior,Sign::Material)) => { verdict = 1; }
                        _ => { verdict = 2; }
                    }
                    d /= 2.; narrowed = true;
                }
                match verdict {
                    0 => { entry.0 += 1; if narrowed && d < probe { flipped += 1; } }
                    1 => {
                        entry.2 += 1;
                        if examples.len() < 6 { examples.push((mesh.sheet[i],centroid,n)); }
                    }
                    _ => entry.3 += 1,
                }
            }
            eprintln!("== tumbling cylinder: the winding of every triangle, by sheet \
                (certify's rule: halve the probe from {probe} to {least} before calling one reversed)");
            let (mut out,mut rev,mut und) = (0,0,0);
            for (s,(o,_,r,u)) in &per {
                let who = if *s == u32::MAX { "the zip".to_string() } else { format!("sheet {s}") };
                out += o; rev += r; und += u;
                eprintln!("  {who}: {o} outward, {r} REVERSED, {u} undecided");
            }
            // Measured: 278 reversed at the full probe becomes 101 under this rule, with outward
            // unchanged at 2920 and undecided rising 90 to 267 — the same 177. So what stops
            // reading reversed becomes **undecided at the shortest probe**, which is what thin
            // material gives, and `flipped` (reversed at the full probe, outward at a shorter one)
            // is legitimately 0 rather than miscounted.
            eprintln!("== {out} outward, {rev} reversed, {und} undecided; {flipped} read reversed \
                at the full probe and outward at a shorter one");
            for (s,c,n) in &examples {
                eprintln!("  reversed on {}: centroid {:?}, its normal {:?}",
                    if *s == u32::MAX { "the zip".to_string() } else { format!("sheet {s}") },
                    c.map(|x| (x*1e4).round()/1e4),n.map(|x| (x*1e3).round()/1e3));
            }
        }
    });
}

/// How each of the zip's reversed triangles sits against its neighbours. 243 of the 471 triangles
/// `rim_zip` lays are wound inwards while every traced sheet is 1–3% reversed, and the
/// `unwrap_or(false)` suspect in `wind` could not be measured by replaying the passes — on the
/// finished mesh no pass takes any loop, so `lay` never runs. This needs no replay.
///
/// Consistency plus an outward neighbour *forces* outwardness, so for a reversed triangle exactly
/// one of four things must hold, and these counts separate them: it shares an edge walked
/// oppositely with an **outward** triangle (a contradiction — the winding is not consistent
/// there); its only such neighbours are themselves reversed (propagation from one bad seed); some
/// edge is walked the **same** way by another triangle (a fold); or some edge is walked by nothing
/// else (it sits on the boundary and inherited nothing). Nothing is built or changed here.
#[test]
#[ignore]
fn how_the_zips_reversed_triangles_sit_against_their_neighbours() {
    use gcs_core::solid::MaterialField;
    use gcs_core::solid::swept_boundary::{FieldJudge,Sign,Stage,SweptBoundaryOptions,construct};
    use gcs_core::space::stable_normal;
    use std::collections::BTreeMap;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,options.judge_tolerance(),4000,1000,4096);
    let probe = options.probe_distance();
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,..} = stage {
            let mut walkers: BTreeMap<(u32,u32),Vec<usize>> = Default::default();
            for (i,t) in mesh.triangles.iter().enumerate() {
                for k in 0..3 { walkers.entry((t[k],t[(k+1)%3])).or_default().push(i); }
            }
            // +1 outward, -1 reversed, 0 undecided — the certificate's own question
            let mut sense: Vec<i8> = vec![0;mesh.triangles.len()];
            for (i,t) in mesh.triangles.iter().enumerate() {
                let [a,b,c] = t.map(|v| mesh.vertices[v as usize]);
                let Some(n) = stable_normal(a,b,c) else { continue };
                let centroid: V3 = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
                // `certify`'s rule, not one probe at the full distance: halve down to the least
                // before calling a triangle reversed. With the full probe alone this test said 235
                // of 243 reversed triangles share an edge, walked oppositely, with an outward one,
                // which is impossible for a consistently wound surface — the sign of a wrong rule.
                let (mut verdict,mut d) = (0_i8,probe);
                while d >= options.least_probe() {
                    match judge.sides(centroid,n,d) {
                        Ok((Sign::Material,Sign::Exterior)) => { verdict = 1; break; }
                        Ok((Sign::Exterior,Sign::Material)) => verdict = -1,
                        _ => verdict = 0,
                    }
                    d /= 2.;
                }
                sense[i] = verdict;
            }
            let (mut contradiction,mut propagated,mut folded,mut free_edge,mut total) = (0,0,0,0,0);
            for (i,t) in mesh.triangles.iter().enumerate() {
                if sense[i] != -1 || mesh.sheet[i] != u32::MAX { continue; }
                total += 1;
                let (mut outward_nb,mut reversed_nb,mut fold,mut free) = (0,0,0,0);
                for k in 0..3 {
                    let (a,b) = (t[k],t[(k+1)%3]);
                    let opposite: Vec<usize> = walkers.get(&(b,a)).cloned().unwrap_or_default();
                    let same: Vec<usize> = walkers.get(&(a,b)).map(|v| v.iter().copied()
                        .filter(|&j| j != i).collect()).unwrap_or_default();
                    if !same.is_empty() { fold += 1; }
                    if opposite.is_empty() && same.is_empty() { free += 1; }
                    for j in opposite {
                        if sense[j] == 1 { outward_nb += 1; } else if sense[j] == -1 { reversed_nb += 1; }
                    }
                }
                if outward_nb > 0 { contradiction += 1; } else if reversed_nb > 0 { propagated += 1; }
                if fold > 0 { folded += 1; }
                if free > 0 { free_edge += 1; }
            }
            eprintln!("== tumbling cylinder: {total} reversed triangles laid by the zip");
            eprintln!("   {contradiction} share an edge walked oppositely with an OUTWARD triangle \
                (consistency would force them outward, so the winding is not consistent there)");
            eprintln!("   {propagated} have only reversed such neighbours (propagation from one seed)");
            eprintln!("   {folded} have an edge another triangle walks the SAME way (a fold)");
            eprintln!("   {free_edge} have an edge nothing else walks (on the boundary, inheriting nothing)");
        }
    });
}

/// Whether the refused loops lie on the envelope of the cap planes. Closed form says that surface
/// must exist and says exactly what it is: a cap is a planar face whose plane runs **parallel** to
/// the tumble axis at distance 1, and the envelope of a plane rotating about a parallel axis is a
/// **cylinder of that radius** — here `y² + z² = 1` about the x-axis. The contact condition agrees:
/// for a cap normal `n` and a point `p`, the velocity about x̂ is `ω(0, −z, y)`, so `n·v = 0` picks
/// out a **diameter line** on the cap, and sweeping that line traces exactly that cylinder.
///
/// The tracer does produce it — sheets 5 and 8, boxes `[2.0,−0.5,0.866]..[4.0,0.5,1.0]` and
/// `[2.0,−0.5,−1.0]..[4.0,0.5,−0.866]`, which is `(x, −sin t, cos t)` over t ∈ [−30°,30°], a 60°
/// arc. So if the refused loops lie **on** that cylinder, clustered at the arc's ends, then the
/// missing piece is the junction between that patch and its neighbours (the wall sheets and the
/// silhouettes) rather than absent surface, and `merge_creases` is the machinery that should join
/// them. Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn whether_the_refused_loops_lie_on_the_cap_planes_envelope() {
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct};
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,unpaired,..} = stage {
            eprintln!("== tumbling cylinder: distance from the envelope cylinder y²+z²=1 about the x-axis");
            for (k,l) in unpaired.iter().enumerate() {
                let p: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
                // how far off that cylinder each vertex stands, and where it sits along the arc
                let mut off: Vec<f64> = p.iter().map(|q| ((q[1]*q[1]+q[2]*q[2]).sqrt()-1.).abs()).collect();
                let on = off.iter().filter(|d| **d <= SAGITTA).count();
                // How near the loop comes to the tumble axis (the world x-axis). Rendered, the two
                // wide loops are chevrons whose apexes meet at y = z = 0 — the axis itself, which
                // the motion holds still and which pierces the tool's wall at (2,0,0) and (4,0,0).
                // This is the number that says so rather than a projection: `|r-1|` came back as
                // "most 0.9836", which reads as radius 1.9836 **or** 0.0164, and only the picture
                // told the two apart. A radius near zero puts the apex on the axis.
                let (nearest,at_axis) = p.iter().fold((f64::INFINITY,[0.;3]),|(least,q),v| {
                    let r = (v[1]*v[1]+v[2]*v[2]).sqrt();
                    if r < least { (r,*v) } else { (least,q) }
                });
                off.sort_by(f64::total_cmp);
                let (lo,hi) = p.iter().fold((f64::INFINITY,f64::NEG_INFINITY),
                    |(lo,hi),q| (lo.min(q[2]),hi.max(q[2])));
                // the 60° arc the roll sweeps reaches |z| >= cos 30° = 0.866
                let in_band = p.iter().filter(|q| q[2].abs() >= 0.866-SAGITTA).count();
                eprintln!("  loop {k}: {} vertices, {on} within a sagitta of the cylinder \
                    (least {:.4}, median {:.4}, most {:.4}); z from {lo:.3} to {hi:.3}, \
                    {in_band} at |z| >= cos 30; nearest the tumble axis {nearest:.4} at {:?}",
                    p.len(),off[0],off[off.len()/2],off[off.len()-1],at_axis.map(|x| (x*1e3).round()/1e3));
            }
        }
    });
}

/// How far apart the sheets bordering each refused loop actually stand. Four of the seven loops sit
/// on the cap planes' envelope cylinder at the edge of its 60° arc, so they ought to be a crease
/// between that traced patch (sheets 5 and 8) and its neighbours — and `merge_creases` is plainly
/// active on this case (45 rims clipped, 136 rim vertices welded, 27 rim edges split) yet does not
/// join them. The number that decides why: the least distance between the vertices of two
/// different bordering sheets near the loop. Above the crease merge's reach the patches genuinely
/// do not abut, which is a coverage gap in the tracer and not a tolerance to widen; just above it,
/// the fix is within reach. Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn how_far_apart_the_sheets_bordering_each_loop_stand() {
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct};
    use std::collections::{BTreeMap,BTreeSet};
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,unpaired,..} = stage {
            // which sheets own a triangle at each vertex
            let mut owners: BTreeMap<u32,BTreeSet<u32>> = Default::default();
            for (i,t) in mesh.triangles.iter().enumerate() {
                for v in t { owners.entry(*v).or_default().insert(mesh.sheet[i]); }
            }
            eprintln!("== tumbling cylinder: the least gap between two bordering sheets, per loop \
                (weld snap 0.005, crease merge 0.03, junction 0.04, spacing 0.5)");
            for (k,l) in unpaired.iter().enumerate() {
                let p: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
                let centre: V3 = std::array::from_fn(|j| p.iter().map(|q| q[j]).sum::<f64>()/p.len() as f64);
                let reach = p.iter().map(|q| harness::distance(*q,centre)).fold(0_f64,f64::max)+0.2;
                // the vertices of each sheet near this loop, the zip's own left out
                let mut near: BTreeMap<u32,Vec<V3>> = Default::default();
                for (v,sheets) in &owners {
                    let q = mesh.vertices[*v as usize];
                    if harness::distance(q,centre) > reach { continue; }
                    // A vertex counts for a sheet only when that sheet is its sole owner. Counting
                    // a **shared** vertex for both sheets made every gap read exactly 0.0000 —
                    // `weld` has already merged the coincident rims, so a shared vertex always
                    // exists near a loop and the first version of this asked only "do these two
                    // sheets share one", which answers itself. Excluding shared vertices makes the
                    // number the real distance between the two patches' own material.
                    let mine: Vec<u32> = sheets.iter().copied().filter(|s| *s != u32::MAX).collect();
                    if mine.len() != 1 { continue; }
                    near.entry(mine[0]).or_default().push(q);
                }
                let sheets: Vec<u32> = near.keys().copied().collect();
                let mut gaps: Vec<(f64,u32,u32)> = Vec::new();
                for i in 0..sheets.len() { for j in i+1..sheets.len() {
                    let (a,b) = (&near[&sheets[i]],&near[&sheets[j]]);
                    let least = a.iter().map(|x| b.iter().map(|y| harness::distance(*x,*y))
                        .fold(f64::INFINITY,f64::min)).fold(f64::INFINITY,f64::min);
                    if least.is_finite() { gaps.push((least,sheets[i],sheets[j])); }
                } }
                gaps.sort_by(|x,y| x.0.total_cmp(&y.0));
                let said: Vec<String> = gaps.iter().take(4)
                    .map(|(d,a,b)| format!("{a}-{b} {d:.4}")).collect();
                eprintln!("  loop {k}: {} vertices, sheets {:?} near it -> least gaps {}",
                    l.len(),sheets,said.join(", "));
            }
        }
    });
}

/// Where the boundary has no mesh at all. Every other measurement in this file asks what is wrong
/// with the mesh that exists; none asks where the surface is that was never laid. The field is the
/// truth here, so the question is answerable directly: walk a grid, take every adjacent pair whose
/// signs differ, bisect to land on the boundary, and measure how far that boundary point stands
/// from the nearest mesh vertex. A covered point lies within half an edge (about 0.1 here); a
/// cluster standing far off is missing surface, **located** rather than inferred.
///
/// This is what would settle what the two wide seams bound — they hold 13.63 of the 13.67 refused
/// area, span radius 1 out to ≈2, and every structural explanation for them has been measured and
/// excluded. Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn where_the_boundary_has_no_mesh() {
    use gcs_core::solid::MaterialField;
    use gcs_core::solid::swept_boundary::{FieldJudge,Sign,Stage,SweptBoundaryOptions,construct};
    const GRID: usize = 20;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,options.judge_tolerance(),4000,1000,4096);
    let probe = options.probe_distance();
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,..} = stage {
            let (lo,hi) = mesh.vertices.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),
                |(lo,hi),q| (std::array::from_fn(|k| lo[k].min(q[k])),std::array::from_fn(|k| hi[k].max(q[k]))));
            let (lo,hi): (V3,V3) = (std::array::from_fn(|k| lo[k]-0.1),std::array::from_fn(|k| hi[k]+0.1));
            let at = |i: usize,j: usize,k: usize| -> V3 {
                std::array::from_fn(|d| { let (a,b) = (lo[d],hi[d]); let n = [i,j,k][d] as f64;
                    a+(b-a)*n/(GRID as f64-1.) })
            };
            // material or exterior at every grid point, by far queries
            let mut inside = vec![false;GRID*GRID*GRID];
            let mut known = vec![false;GRID*GRID*GRID];
            // How much of the grid this instrument cannot use. A point inside the judge's band
            // reads `Near` and a budget-exhausted one `Unresolved`, and either makes every pair it
            // belongs to unusable. Measured: only **80** of about 22 800 adjacent pairs straddled
            // the boundary with both ends known, so a negative result here rests on a thin sample
            // and these counts are what say how thin. Without them the audit looks conclusive when
            // it is not.
            let (mut band,mut skipped) = (0,0);
            let index = |i: usize,j: usize,k: usize| (i*GRID+j)*GRID+k;
            for i in 0..GRID { for j in 0..GRID { for k in 0..GRID {
                if let Ok((s,_)) = judge.deep_sign(at(i,j,k),probe) {
                    match s {
                        Sign::Material => { inside[index(i,j,k)] = true; known[index(i,j,k)] = true; }
                        Sign::Exterior => { known[index(i,j,k)] = true; }
                        _ => band += 1,
                    }
                }
            } } }
            // every adjacent pair that straddles the boundary, bisected onto it
            let mut far: Vec<(f64,V3)> = Vec::new();
            let mut found = 0;
            for i in 0..GRID { for j in 0..GRID { for k in 0..GRID {
                for (di,dj,dk) in [(1,0,0),(0,1,0),(0,0,1)] {
                    let (i2,j2,k2) = (i+di,j+dj,k+dk);
                    if i2 >= GRID || j2 >= GRID || k2 >= GRID { continue; }
                    let (a,b) = (index(i,j,k),index(i2,j2,k2));
                    if !known[a] || !known[b] { skipped += 1; continue; }
                    if inside[a] == inside[b] { continue; }
                    let (mut p,mut q) = (at(i,j,k),at(i2,j2,k2));
                    if !inside[a] { std::mem::swap(&mut p,&mut q); }
                    for _ in 0..8 {
                        let mid: V3 = std::array::from_fn(|d| 0.5*(p[d]+q[d]));
                        match judge.deep_sign(mid,probe) {
                            Ok((Sign::Material,_)) => p = mid,
                            Ok((Sign::Exterior,_)) => q = mid,
                            _ => break,
                        }
                    }
                    let on: V3 = std::array::from_fn(|d| 0.5*(p[d]+q[d]));
                    let near = mesh.vertices.iter().map(|v| harness::distance(on,*v)).fold(f64::INFINITY,f64::min);
                    found += 1;
                    far.push((near,on));
                }
            } } }
            far.sort_by(|x,y| y.0.total_cmp(&x.0));
            let over = |t: f64| far.iter().filter(|(d,_)| *d > t).count();
            eprintln!("== tumbling cylinder: {found} boundary points from a {GRID}³ grid, by distance \
                to the nearest mesh vertex ({band} grid points in the judge's band or unresolved, \
                {skipped} pairs discarded for an unknown end — the sample's thinness)");
            eprintln!("   {} beyond 0.15, {} beyond 0.3, {} beyond 0.5",over(0.15),over(0.3),over(0.5));
            if !far.is_empty() {
                eprintln!("   median {:.4}, most {:.4}",far[far.len()/2].0,far[0].0);
                for (d,p) in far.iter().take(5) {
                    eprintln!("   uncovered: {:.4} from any vertex at {:?}",d,p.map(|x| (x*1e3).round()/1e3));
                }
            }
        }
    });
}

/// Whether the cap planes' envelope cylinder is meshed all the way round its arc. This drops the
/// grid the coverage audit used, and with it that audit's bias: `deep_sign` gives a strict sign
/// only 0.04 clear of the surface while a 20³ cell is 0.12, so the pairs that survived were those
/// where the surface passes near a cell's middle — 80 crossings where a surface of this area
/// should cross some 1400. The candidate surface is known in closed form instead, so it is sampled
/// directly: `y² + z² = 1` about the tumble axis, for x across the tool and θ right round.
///
/// The coverage audit's five farthest points were [2.36, 0.926, ±0.077] and [3.64, −0.926, ±0.077]
/// — symmetric, all at radius ≈0.929, so on this cylinder, but at |z| ≈ 0.08, the middle of the
/// arc, while sheets 5 and 8 cover it only at |z| ≥ 0.866. An arc of θ where the field says
/// boundary and the mesh stands far off is the missing surface, located analytically. Nothing is
/// built or changed here; it only measures.
#[test]
#[ignore]
fn whether_the_envelope_cylinder_is_meshed_round_its_arc() {
    use gcs_core::solid::MaterialField;
    use gcs_core::solid::swept_boundary::{FieldJudge,Sign,Stage,SweptBoundaryOptions,construct};
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,options.judge_tolerance(),4000,1000,4096);
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,..} = stage {
            eprintln!("== tumbling cylinder: the envelope cylinder y²+z²=1, θ measured from +y");
            eprintln!("   (θ; then per x = 2.5, 3.0, 3.5: the field's sign and the distance to the \
                nearest mesh vertex)");
            for step in 0..24 {
                let theta = step as f64*15.;
                let (c,s) = (theta.to_radians().cos(),theta.to_radians().sin());
                let mut said: Vec<String> = Vec::new();
                for x in [2.5,3.0,3.5] {
                    let p: V3 = [x,c,s];
                    // The **depth**, not the word. `sign` says `Near` only inside a band of
                    // 0.0025, so a point half a thousandth inside reads `Material` — and the first
                    // version of this printed that as "in" at θ = 60°–120°, where a mesh vertex
                    // sits 0.000 away, i.e. on the surface. Reading a sign as a position is the
                    // same mistake that made 278 thin triangles look reversed.
                    let word = match judge.sign(p) {
                        Ok((Sign::Near {..},_)) => "on      ".to_string(),
                        Ok((Sign::Material,e)) => format!("in {:6.4}",-e[1]),
                        Ok((Sign::Exterior,e)) => format!("out{:6.4}",e[0]),
                        _ => "?       ".to_string(),
                    };
                    let near = mesh.vertices.iter().map(|v| harness::distance(p,*v)).fold(f64::INFINITY,f64::min);
                    said.push(format!("{word} mesh {near:.3}"));
                }
                eprintln!("   θ={theta:5.1}°: {}",said.join(" | "));
            }
        }
    });
}

/// What the field reports at the mesh's **own** vertices. This settles a contradiction that three
/// misreadings of the field's output finally produced: sampling the analytic cylinder printed a
/// field value of −0.1340 at θ = 60°–120°, where a mesh vertex sits 0.000 away. The field is
/// one-Lipschitz, so |f| ≤ distance to the boundary, and −0.134 would put that vertex at least
/// 0.134 from the true surface — gross, not chordal, and at odds with the certificate passing on
/// the closing cases.
///
/// Either the enclosure is not what it is being read as, or those vertices really do stand off the
/// boundary. Asking the vertices directly separates the two: every vertex the construction keeps
/// was judged `Kept` or `Moved` within the vertex tolerance, so their values should be small. A
/// distribution with a long tail says otherwise. Nothing is built or changed here; it only
/// measures.
#[test]
#[ignore]
fn what_the_field_reports_at_the_meshs_own_vertices() {
    use gcs_core::solid::MaterialField;
    use gcs_core::solid::swept_boundary::{FieldJudge,Sign,Stage,SweptBoundaryOptions,construct};
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,options.judge_tolerance(),4000,1000,4096);
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,..} = stage {
            // a stride, since every vertex would be thousands of queries
            let stride = (mesh.vertices.len()/400).max(1);
            let (mut near,mut material,mut exterior,mut other) = (0,0,0,0);
            let mut worst: Vec<(f64,V3)> = Vec::new();
            for (i,v) in mesh.vertices.iter().enumerate() {
                if i%stride != 0 { continue; }
                match judge.sign(*v) {
                    Ok((Sign::Near {..},_)) => near += 1,
                    Ok((Sign::Material,enc)) => { material += 1; worst.push((-enc[1],*v)); }
                    Ok((Sign::Exterior,enc)) => { exterior += 1; worst.push((enc[0],*v)); }
                    _ => other += 1,
                }
            }
            worst.sort_by(|a,b| b.0.total_cmp(&a.0));
            let probed = near+material+exterior+other;
            eprintln!("== tumbling cylinder: the field at {probed} of the mesh's own vertices \
                (every {stride}th): {near} near, {material} material, {exterior} exterior, {other} else");
            eprintln!("   |f| is a LOWER BOUND on the distance to the boundary, the field being \
                one-Lipschitz — so a large value means the vertex is genuinely off the surface");
            for (d,p) in worst.iter().take(6) {
                eprintln!("   |f| >= {d:.4} at {:?}",p.map(|x| (x*1e3).round()/1e3));
            }
            if let Some((d,_)) = worst.get(worst.len()/2) { eprintln!("   median of those: {d:.4}"); }
        }
    });
}

/// Draw the open mesh with its refused loops, so somebody can **look** at it. Every other
/// instrument in this file computes statistics about this geometry; none of them shows it. Three
/// orthographic views side by side, the mesh's edges faint, each unpaired loop in its own colour
/// with its vertices marked. Writes an SVG to `SOLVENT_DRAW` (default `/tmp/loops.svg`), which
/// headless Chrome turns into a picture.
#[test]
#[ignore]
fn draw_the_refused_loops() {
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct};
    use std::collections::BTreeSet;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let mut svg = String::new();
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,unpaired,..} = stage {
            let views: [(&str,usize,usize);3] = [("down z: x right, y up",0,1),
                ("down y: x right, z up",0,2),("down x: y right, z up",1,2)];
            let (lo,hi) = mesh.vertices.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),
                |(lo,hi),q| (std::array::from_fn(|k| lo[k].min(q[k])),std::array::from_fn(|k| hi[k].max(q[k]))));
            let span = (0..3).map(|k| hi[k]-lo[k]).fold(1e-9,f64::max);
            let (pad,size) = (24.,440.);
            let (w,h) = ((size+pad)*3.+pad,size+2.*pad+26.);
            svg.push_str(&format!("<svg xmlns='http://www.w3.org/2000/svg' width='{w}' height='{h}' \
                viewBox='0 0 {w} {h}'><rect width='{w}' height='{h}' fill='white'/>"));
            let hues = ["#e6194b","#3cb44b","#4363d8","#f58231","#911eb4","#008080","#9a6324"];
            for (i,(label,a,b)) in views.iter().enumerate() {
                let ox = pad+(size+pad)*i as f64;
                let at = |p: &V3| -> (f64,f64) {
                    let u = (p[*a]-lo[*a])/span*size*0.9+size*0.05;
                    let v = (p[*b]-lo[*b])/span*size*0.9+size*0.05;
                    (ox+u,pad+size-v)
                };
                svg.push_str(&format!("<text x='{ox:.0}' y='{:.0}' font-family='sans-serif' \
                    font-size='13' fill='#333'>{label}</text>",h-8.));
                // the mesh, each undirected edge once and faint
                let mut drawn: BTreeSet<(u32,u32)> = Default::default();
                for t in &mesh.triangles { for k in 0..3 {
                    let (p,q) = (t[k],t[(k+1)%3]);
                    if !drawn.insert((p.min(q),p.max(q))) { continue; }
                    let (x1,y1) = at(&mesh.vertices[p as usize]);
                    let (x2,y2) = at(&mesh.vertices[q as usize]);
                    svg.push_str(&format!("<line x1='{x1:.1}' y1='{y1:.1}' x2='{x2:.1}' y2='{y2:.1}' \
                        stroke='#d8d8d8' stroke-width='0.3'/>"));
                } }
                // and every refused loop over it, in its own colour
                for (k,l) in unpaired.iter().enumerate() {
                    let colour = hues[k%hues.len()];
                    let pts: Vec<String> = l.iter().map(|&v| {
                        let (x,y) = at(&mesh.vertices[v as usize]); format!("{x:.1},{y:.1}") }).collect();
                    svg.push_str(&format!("<polygon points='{}' fill='none' stroke='{colour}' \
                        stroke-width='2.2' stroke-linejoin='round'/>",pts.join(" ")));
                    for &v in l.iter() {
                        let (x,y) = at(&mesh.vertices[v as usize]);
                        svg.push_str(&format!("<circle cx='{x:.1}' cy='{y:.1}' r='1.8' fill='{colour}'/>"));
                    }
                }
            }
            svg.push_str("</svg>");
            eprintln!("== drew {} triangles and {} refused loops",mesh.triangles.len(),unpaired.len());
            for (k,l) in unpaired.iter().enumerate() {
                eprintln!("   loop {k}: {} vertices, drawn {}",l.len(),hues[k%hues.len()]);
            }
        }
    });
    let path = std::env::var("SOLVENT_DRAW").unwrap_or_else(|_| "/tmp/loops.svg".into());
    std::fs::write(&path,&svg).unwrap();
    eprintln!("== wrote {} bytes to {path}",svg.len());
}

/// Not closed yet (milestone 5, the tracer): the sphere the stationary
/// ring sweeps is inner everywhere but at second order (within a quarter
/// sagitta of the boundary over a patch a fifth wide about each fixed
/// point of the axis, which the labels keep), the generators through the
/// fixed points sweep bowtie sectors whose halves face opposite ways, and
/// the rims' sweeps fold where a rim's tangent runs along its velocity.
/// Runs in four seconds and leaves nine loops and a few dozen refused
/// triangles; `SOLVENT_SHEETS=1` prints the stages.
#[test]
#[ignore]
fn a_tumbling_cylinder_closes_along_its_creases() {
    // radius 1, height 2 about the vertical through (3, 0), tumbled ±30°
    // about the horizontal line through its centre
    let source = tumbling_cylinder();
    let (mesh,certificate) = closed_shell_at(&source,SAGITTA);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    let member = |p: V3| (p[0]-3.).powi(2)+p[1]*p[1] <= 1. && p[2].abs() <= 1.;
    let expected = sampled_volume(&source,&member,[1.5,-1.5,-1.5],[4.5,1.5,1.5],64,256);
    let v = volume(&mesh);
    eprintln!("tumbling cylinder: volume {v:.4}, sampled {expected:.4}");
    assert!((v-expected).abs() <= 0.03*expected,"volume {v} against sampled {expected}");
}

/// Milestone 5a, and now a row of the case table (`cases.rs`): the box's end faces are square to
/// the turn's axis, so the motion carries them within their own planes and `n·v` is zero over the
/// whole of them. Each is swept exactly in its plane (`swept_boundary::grazing`) rather than
/// traced: the tracer emitted the bands their edges sweep in that plane, folded where a segment
/// turning in its plane crosses its earlier positions, and the slivers those left along the
/// envelope arc pinched the rims the zip then closed wrongly.

/// Not closed yet (project 2, the Boolean meshes): the union's mesh is
/// forty-five thousand facets of the bar's wall shredded by the balls'
/// facet planes, so the caps take most of a minute and their slivers leave
/// four loops at the crease junctions and twenty-odd refused triangles.
#[test]
#[ignore]
fn a_dumbbell_translated_along_x_closes_along_its_creases() {
    // balls of radius 0.5 on a bar of radius 0.25, advanced 4 along x: the
    // two concave crease circles sweep, and every line along x meets the
    // tool in one segment
    let source = sliding_dumbbell();
    let (mesh,certificate) = closed_shell_at(&source,SAGITTA);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    let member = |p: V3| ((p[0]-3.).powi(2)+p[1]*p[1] <= 0.0625 && p[2].abs() <= 1.) || (p[0]-3.).powi(2)+p[1]*p[1]+(p[2]-1.).powi(2) <= 0.25 || (p[0]-3.).powi(2)+p[1]*p[1]+(p[2]+1.).powi(2) <= 0.25;
    let expected = sampled_volume(&source,&member,[2.4,-0.6,-1.6],[7.6,0.6,1.6],64,256);
    let v = volume(&mesh);
    eprintln!("dumbbell: volume {v:.4}, sampled {expected:.4}");
    assert!((v-expected).abs() <= 0.03*expected,"volume {v} against sampled {expected}");
}

/// Every milestone-4 case written out for a person to look at: its Solvent
/// document and the swept boundary the pipeline makes of it, as binary STL,
/// into `SOLVENT_EXPORT` (default `rust/examples/swept_boundary`). The mesh is written whether or not it closed; a line per case says
/// which did. The dumbbell takes most of a minute, the rest seconds each.
#[test]
#[ignore]
fn export_milestone_4_cases() {
    let dir = std::path::PathBuf::from(std::env::var("SOLVENT_EXPORT").unwrap_or_else(|_| concat!(env!("CARGO_MANIFEST_DIR"),"/../examples/swept_boundary").into()));
    std::fs::create_dir_all(&dir).unwrap();
    let cases: [(&str,String); 5] = [
        ("turning_prism",turning_prism()),
        ("turned_lens",turned_lens()),
        ("tumbling_cylinder",tumbling_cylinder()),
        ("turned_box",turned_box()),
        ("sliding_dumbbell",sliding_dumbbell()),
    ];
    let mut summary = Vec::new();
    for (name,source) in &cases {
        eprintln!("== {name}");
        std::fs::write(dir.join(format!("{name}.sv")),source).unwrap();
        let (mesh,certificate) = shell_at(source,SAGITTA);
        let mesh = mesh.compact();
        let verdict = match &certificate { Ok(c) => format!("{} failed certificate",c.failures.len()),Err(e) => format!("refused {e:?}") };
        // written unchecked: an open case's slivers may collapse in float32,
        // and a person looking at it wants them there, counted
        let (bytes,collapsed) = stl(&mesh.vertices,&mesh.triangles,name);
        std::fs::write(dir.join(format!("{name}.stl")),bytes).unwrap();
        let shell = match closed(&mesh) { Ok(()) => "closed".to_string(),Err(e) => format!("open ({e})") };
        summary.push(format!("{name}: {} triangles ({collapsed} collapse in float32), {shell}, {verdict}, volume {:.4}",mesh.triangles.len(),volume(&mesh)));
    }
    eprintln!("written to {}",dir.display());
    for line in &summary { eprintln!("  {line}"); }
}

/// Binary STL of an indexed mesh as it stands, with how many of its
/// triangles float32 coordinates reduce to no area.
pub(super) fn stl(vertices: &[V3],triangles: &[[u32;3]],name: &str) -> (Vec<u8>,usize) {
    let mut out = Vec::with_capacity(84+triangles.len()*50);
    let mut header = [0u8;80];
    for (i,b) in format!("solvent {name}").bytes().take(79).enumerate() { header[i] = b; }
    out.extend(header);
    out.extend((triangles.len() as u32).to_le_bytes());
    let mut collapsed = 0;
    for t in triangles {
        let [a,b,c] = t.map(|v| vertices[v as usize].map(|x| x as f32));
        let (u,w) = ([b[0]-a[0],b[1]-a[1],b[2]-a[2]],[c[0]-a[0],c[1]-a[1],c[2]-a[2]]);
        let n = [u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]];
        let len = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
        if !(len > 0.) { collapsed += 1; }
        for x in if len > 0. { n.map(|x| x/len) } else { [0.;3] } { out.extend(x.to_le_bytes()); }
        for p in [a,b,c] { for x in p { out.extend(x.to_le_bytes()); } }
        out.extend(0u16.to_le_bytes());
    }
    (out,collapsed)
}

/// The certified cases do not stand on knife edges: every seed point moved by about 1e-12 (far
/// below every tolerance the construction uses) leaves the mesh as it was, triangle for
/// triangle.
#[test]
fn seeds_moved_below_every_tolerance_leave_the_mesh_as_it_was() {
    use gcs_core::solid::swept_boundary::{SweptBoundaryOptions,construct_from,seeds};
    // The turned box is a grazing region under a turn; the two slides are one under a slide,
    // which is the other branch `Family::in_plane` reads and had no gate of its own — 5a made
    // both exact (the box to 1e-9, the prism exactly) and nothing held them to it under a jitter.
    // The dumbbell is left to the ignored all-case variant below: it alone costs twice this test.
    let slid_box = format!("{}{}{}",tools::BOX,motions::slide_x(10.),motions::swept("feed",0.,360.));
    let slid_prism = format!("{}{}{}",tools::TRIANGLE_PRISM,motions::slide_x(10.),motions::swept("feed",0.,360.));
    for source in [turning_prism(),turned_lens(),turned_box(),slid_box,slid_prism] {
        let e = harness::read(&source);
        let swept = harness::solid(&e,"swept");
        let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
        let (_,sheets,_) = seeds(&e.sketch,swept,options.spacing,options.sagitta,&|_| {}).unwrap();
        let mut moved = sheets.clone();
        let mut k = 0_f64;
        for s in &mut moved { for p in &mut s.points { k += 1.; *p = [p[0]+1e-12*(1.7*k).sin(),p[1]+1e-12*(2.3*k).cos(),p[2]+1e-12*(3.1*k+1.).sin()]; } }
        let build = |sheets| construct_from(&e.sketch,swept,&options,sheets,&mut |_,_| {}).unwrap().mesh;
        let (a,b) = (build(sheets),build(moved));
        assert_eq!(a.triangles,b.triangles);
        assert_eq!(a.sheet,b.sheet);
        let worst = a.vertices.iter().zip(&b.vertices).map(|(p,q)| (0..3).map(|k| (p[k]-q[k]).abs()).fold(0.,f64::max)).fold(0.,f64::max);
        assert!(a.vertices.len() == b.vertices.len() && worst <= 1e-9,"{} vertices against {}, one moved {worst:e}",a.vertices.len(),b.vertices.len());
    }
}

/// The same of all five milestone-4 cases, the three deferred ones included: every stage's
/// labels, keep flags, triangles and vertices as they were. About a minute (the dumbbell); prints
/// a line per case.
#[test]
#[ignore]
fn seeds_moved_below_every_tolerance_leave_every_milestone_4_case_as_it_was() {
    use gcs_core::solid::swept_boundary::{KeptMesh,Stage,SweptBoundaryOptions,construct_from,seeds};
    #[derive(Clone)]
    enum Print { Codes(Vec<u64>),Mesh(Vec<[u32;3]>,Vec<u32>,Vec<V3>) }
    let print = |stage: &Stage<'_>| -> (&'static str,Print) {
        let mesh = |m: &KeptMesh| Print::Mesh(m.triangles.clone(),m.sheet.clone(),m.vertices.clone());
        match stage {
            Stage::Seeded {..} => ("seeded",Print::Codes(vec![])),
            Stage::Grazed {regions} => ("grazed",Print::Mesh(regions.iter().flat_map(|g| g.patch.triangles.clone()).collect(),vec![],regions.iter().flat_map(|g| g.patch.points.clone()).collect())),
            Stage::Capped {caps} => ("capped",Print::Mesh(caps.iter().flat_map(|c| c.patch.triangles.clone()).collect(),vec![],caps.iter().flat_map(|c| c.patch.points.clone()).collect())),
            Stage::Labelled {labelled,..} => ("labelled",Print::Codes(labelled.iter().flat_map(|l| l.labels.iter().map(|x| *x as u64)).collect())),
            Stage::Clipped {mesh:m,..} => ("clipped",mesh(m)),
            Stage::Merged {mesh:m,..} => ("merged",mesh(m)),
            Stage::Kept {keep,..} => ("kept",Print::Codes(keep.iter().map(|k| *k as u64).collect())),
            Stage::Unioned {mesh:m,..} => ("unioned",mesh(m)),
            Stage::Uncovered {mesh:m,..} => ("uncovered",mesh(m)),
            Stage::Welded {mesh:m,..} => ("welded",mesh(m)),
            Stage::Split {mesh:m} => ("split",mesh(m)),
            Stage::Zipped {mesh:m,..} => ("zipped",mesh(m)),
            Stage::Certified {mesh:m,..} => ("certified",mesh(m)),
        }
    };
    // None when the two agree: the same codes, or the same triangles on vertices within 1e-9
    let differ = |a: &Print,b: &Print| -> Option<String> {
        match (a,b) {
            (Print::Codes(x),Print::Codes(y)) => (x != y).then(|| format!("{} of {} decisions",x.iter().zip(y).filter(|(p,q)| p != q).count()+x.len().abs_diff(y.len()),x.len())),
            (Print::Mesh(ta,sa,va),Print::Mesh(tb,sb,vb)) => {
                // within the construction's coincidence (the weld's 1e-7): an intersection of two
                // nearly parallel lines turns a 1e-12 move into a larger, still negligible one
                let moved = va.len() != vb.len() || va.iter().zip(vb).any(|(p,q)| (0..3).any(|k| (p[k]-q[k]).abs() > 1e-7));
                if ta == tb && sa == sb && !moved { return None; }
                // the triangles by sheet and centroid, each matched within 1e-9 of one in the other
                let cs = |t: &Vec<[u32;3]>,s: &Vec<u32>,v: &Vec<V3>| -> Vec<(u32,V3)> { t.iter().enumerate().map(|(i,tri)| (s.get(i).copied().unwrap_or(0),std::array::from_fn(|k| tri.iter().map(|&x| v[x as usize][k]).sum::<f64>()/3.))).collect() };
                let (ca,cb) = (cs(ta,sa,va),cs(tb,sb,vb));
                let unmatched = |p: &Vec<(u32,V3)>,q: &Vec<(u32,V3)>| -> Vec<(u32,V3)> {
                    let mut grid: std::collections::BTreeMap<[i64;3],Vec<usize>> = Default::default();
                    for (j,(_,c)) in q.iter().enumerate() { grid.entry(c.map(|x| (x*1e4).floor() as i64)).or_default().push(j); }
                    p.iter().filter(|(s,c)| {
                        let k = c.map(|x| (x*1e4).floor() as i64);
                        !(-1..=1).any(|dx| (-1..=1).any(|dy| (-1..=1).any(|dz| grid.get(&[k[0]+dx,k[1]+dy,k[2]+dz]).is_some_and(|l| l.iter().any(|&j| q[j].0 == *s && (0..3).all(|d| (q[j].1[d]-c[d]).abs() <= 1e-7))))))
                    }).copied().collect()
                };
                let (oa,ob) = (unmatched(&ca,&cb),unmatched(&cb,&ca));
                Some(if oa.is_empty() && ob.is_empty() {
                    let at = (0..ta.len().min(tb.len())).find(|&i| ta[i] != tb[i] || sa.get(i) != sb.get(i));
                    let show = |t: [u32;3],v: &Vec<V3>| t.map(|x| (x,v[x as usize].map(|y| (y*1e4).round()/1e4)));
                    match at {
                        Some(i) => {
                            let kind = { let (mut x,mut y) = (ta[i],tb[i]); x.sort(); y.sort(); if x != y { "other corners" } else if (0..3).any(|r| (0..3).all(|k| ta[i][k] == tb[i][(k+r)%3])) { "rotated" } else { "reversed" } };
                            format!("the same {} triangles, ordered otherwise: first at {i} ({kind}) {:?} against {:?}",ta.len(),show(ta[i],va),show(tb[i],vb))
                        }
                        None => format!("the same {} triangles and indices, vertices moved",ta.len()),
                    }
                }
                    else { format!("{} against {} triangles; traced only {} e.g. {:?}; moved only {} e.g. {:?}",ta.len(),tb.len(),oa.len(),&oa[..oa.len().min(3)],ob.len(),&ob[..ob.len().min(3)]) })
            }
            _ => Some("different kinds".into()),
        }
    };
    let mut failed = Vec::new();
    for (name,source) in [("turning_prism",turning_prism()),("turned_lens",turned_lens()),("turned_box",turned_box()),("tumbling_cylinder",tumbling_cylinder()),("sliding_dumbbell",sliding_dumbbell())] {
        let e = harness::read(&source);
        let swept = harness::solid(&e,"swept");
        let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
        let (_,sheets,_) = seeds(&e.sketch,swept,options.spacing,options.sagitta,&|_| {}).unwrap();
        // every stage's print up to the end or the refusal, and the refusal
        let run = |sheets| -> Result<(Vec<(&'static str,Print)>,Option<String>),String> {
            let mut prints = Vec::new();
            let refused = construct_from(&e.sketch,swept,&options,sheets,&mut |s,_| prints.push(print(&s))).err().map(|e| format!("{e:?}"));
            Ok((prints,refused))
        };
        let traced = run(sheets.clone());
        for phase in [0.,1.] {
            let mut moved = sheets.clone();
            let mut k = 0_f64;
            for s in &mut moved { for p in &mut s.points { k += 1.; *p = [p[0]+1e-12*(1.7*k+phase).sin(),p[1]+1e-12*(2.3*k+phase).cos(),p[2]+1e-12*(3.1*k+1.+phase).sin()]; } }
            let verdict = match (&traced,run(moved)) {
                (Ok((a,ra)),Ok((b,rb))) => a.iter().zip(&b).find_map(|((stage,x),(_,y))| differ(x,y).map(|d| format!("first differs at {stage}: {d}")))
                    .or_else(|| (a.len() != b.len() || *ra != rb).then(|| format!("refused otherwise: {ra:?} against {rb:?}"))),
                (a,b) => Some(format!("refused: traced {:?}, moved {:?}",a.as_ref().err(),b.err())),
            };
            eprintln!("{name} (phase {phase}): {}",verdict.as_deref().unwrap_or("as it was"));
            if let Some(v) = verdict { failed.push(format!("{name}: {v}")); }
        }
    }
    assert!(failed.is_empty(),"{failed:?}");
}

/// Milestone 5b: the box turned a whole turn about the axis through its end face's centre. Its
/// end faces graze, so each is swept in its own plane, and the sweep is the solid of revolution
/// the section makes: every circle about the axis within the section's own reach.
#[test]
#[ignore]
fn a_box_turned_a_whole_turn_sweeps_a_ring() {
    let source = format!("{}{}{}",tools::BOX,motions::turn_about(4.,-1.5,5.,-1.5),motions::swept("turn",0.,360.));
    let (mesh,certificate) = closed_shell_at(&source,SAGITTA);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    // the 2 x 3 section turned right round its centre: a disc of the section's own half-diagonal,
    // less the disc its nearest edge leaves, 2 deep along the axis
    let (half_width,half_height) = (1_f64,1.5_f64);
    let expected = std::f64::consts::PI*(half_width*half_width+half_height*half_height)*2.;
    let v = volume(&mesh);
    eprintln!("box turned a whole turn: volume {v:.4}, expected {expected:.4}");
    assert!(v <= expected*(1.+1e-3) && v >= expected*(1.-3.*SAGITTA/half_width),"volume {v} against {expected}");
}

/// Which pass destroys the z = 0 band. The missing wedge **is** traced: sheet 0 carries 78 of its
/// 153 seed points in it, over x 2.00..4.00 and angle −172.5°..180.0°, while every other sheet
/// carries none (`reference::whether_the_missing_wedge_is_ever_traced`). Yet the finished mesh has
/// eight triangles of sheet 0, in a box 0.05 across beside the pierce point (4,0,0). The surface is
/// generated and the pipeline annihilates it — a **trim** defect, which is what retires the plan to
/// build a bowtie generator: a generator would add surface the tracer already makes and leave the
/// mechanism that deletes it running.
///
/// Two counts per stage, and the second is the one to trust. Triangles whose **sheet id** is 0; and
/// triangles of **any** sheet whose centroid lies in the wedge, which no renumbering can affect.
/// The id alone would mislead: `planar_union` relabels a merged coplanar group to one member's id
/// and the zip pushes `u32::MAX`, so an id can vanish where no surface did. Position is the only
/// handle that survives between stages, which is `window`'s own lesson.
///
/// Nothing is built or changed here; it only measures. The wedge predicate is copied from
/// `reference.rs` deliberately unchanged — two spellings of it would be two rules.
#[test]
#[ignore]
fn which_pass_destroys_the_wedge() {
    use gcs_core::solid::swept_boundary::{KeptMesh,Stage,SweptBoundaryOptions,construct};
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let angle = |p: &V3| p[2].atan2(p[1]).to_degrees();
    let in_wedge = |p: &V3| { let a = angle(p).abs(); a < 15. || a > 165. };
    eprintln!("== tumbling cylinder: sheet 0 and the z = 0 wedge, stage by stage");
    eprintln!("   (the wedge is |angle| < 15° or > 165° about the tumble axis, i.e. z near 0)");
    let mut last: Option<usize> = None;
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        let named: Option<(&str,&KeptMesh)> = match &stage {
            Stage::Clipped {mesh,..} => Some(("clipped",mesh)),
            Stage::Merged {mesh,..} => Some(("merged",mesh)),
            Stage::Kept {kept,..} => Some(("kept",kept)),
            Stage::Unioned {mesh,..} => Some(("unioned",mesh)),
            Stage::Uncovered {mesh,..} => Some(("uncovered",mesh)),
            Stage::Welded {mesh,..} => Some(("welded",mesh)),
            Stage::Split {mesh} => Some(("split",mesh)),
            Stage::Zipped {mesh,..} => Some(("zipped",mesh)),
            _ => None,
        };
        let Some((name,mesh)) = named else { return };
        let (mut own,mut wedge) = (0usize,0usize);
        // Area, not triangle count, is what compares against the reference's uncovered 6.445 of
        // 23.493: a count only says how finely the band is cut, and the shortfall argument must not
        // rest on an assumed density.
        let (mut own_area,mut wedge_area,mut all_area) = (0.,0.,0.);
        let (mut xlo,mut xhi) = (f64::INFINITY,f64::NEG_INFINITY);
        let (mut alo,mut ahi) = (f64::INFINITY,f64::NEG_INFINITY);
        for (i,t) in mesh.triangles.iter().enumerate() {
            let [a,b,c] = t.map(|v| mesh.vertices[v as usize]);
            let mid: V3 = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
            let (u,v): (V3,V3) = (std::array::from_fn(|k| b[k]-a[k]),std::array::from_fn(|k| c[k]-a[k]));
            let n: V3 = [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
            let area = 0.5*(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
            all_area += area;
            if mesh.sheet.get(i).copied() == Some(0) {
                own += 1; own_area += area;
                xlo = xlo.min(mid[0]); xhi = xhi.max(mid[0]);
                let d = angle(&mid); alo = alo.min(d); ahi = ahi.max(d);
            }
            if in_wedge(&mid) { wedge += 1; wedge_area += area; }
        }
        let step = match last { None => "  (baseline)".to_string(),
            Some(p) => format!("{:+}",wedge as i64 - p as i64) };
        last = Some(wedge);
        let where_it_is = if own > 0 {
            format!(" (x {xlo:.3}..{xhi:.3}, angle {alo:.1}°..{ahi:.1}°)") } else { String::new() };
        eprintln!("  {name:>10}: {:>5} triangles, area {all_area:7.4}; sheet 0 {own:>4}{where_it_is}, \
            area {own_area:6.4}; in the wedge {wedge:>4} {step}, area {wedge_area:6.4}",
            mesh.triangles.len());
    });
    eprintln!("   the stage where the wedge count collapses is the pass that destroys the band");
}

/// What sheet 0 actually traces, and on which branch of the characteristic set. The `z = 0` band is
/// a genuine generation gap (wedge area 0.0088 against the reference's 6.445 uncovered), so the
/// question is no longer which pass destroys it but why the tracer's own sheet 0 — whose points
/// span the whole band — carries no area.
///
/// The geometry, read off the fixture rather than assumed. `CYLINDER` revolves its 1×2 profile
/// **about `axis`**, the segment x = 3 — not about `spindle` — and page u is world x while page v is
/// world z, so the tool is a cylinder of radius 1 about the line x = 3 parallel to ẑ, with
/// z ∈ [−1,1]. `TUMBLE` turns about x̂ through (3,0,0), so v = (0,−z,y); the wall's normal is
/// (x−3,y,0), and **n·v = −yz**. The characteristic set on the wall is {yz = 0}: the two lines
/// y = 0 (meeting the wall at x = 2 and x = 4) and the **circle z = 0**.
///
/// The key structural fact: a turn about x̂ leaves x̂ fixed in the tool's *own* frame, so that set is
/// the same tool-frame curve at every time. And the chart decides the rest. A revolved surface's
/// normalised **u follows the source edge** (here the meridian, so u ↔ z) and **v the revolution**
/// (the angle); stations are the 96 lines of constant `u = i/96`, each solved for `v`. So the two
/// lines `y = 0` are `v = const` spanning every u and are crossed by all 96 stations — which is
/// exactly why sheets 1–4 cover the angular complement — while the **circle `z = 0` is `u = 0.5`
/// spanning every v**: it lies *along* a single station, and `i/96` at `i = 48` lands on u = 0.5
/// dead on, where the equation in v is degenerate and every v is a root rather than two being
/// isolated.
///
/// That is the dual-stationing defect the plan already names, with the pinion's missing branch as
/// its evidence. A funnel would not touch it: a funnel needs two faces and a cone of normals, and a
/// smooth wall has one normal.
///
/// Branches are classified in the **tool** frame, because the patch's points are posed. A
/// tool-frame point (x,y,0) of the circle lands at (x, y cos θ, y sin θ), so its world angle
/// atan2(z,y) is θ itself; a line-branch point (x,0,z) lands at world angle θ ± 90°. So the offset
/// of each point's world angle from its own column's θ says which branch it came from.
///
/// Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn what_sheet_zero_actually_traces() {
    use gcs_core::solid::swept_boundary::seeds;
    use std::collections::BTreeMap;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let (_,sheets,_) = seeds(&e.sketch,swept,0.5,SAGITTA,&|_| {}).unwrap();
    let s = &sheets[0];
    let angle = |p: &V3| p[2].atan2(p[1]).to_degrees();
    // the signed offset from theta, folded into (-90, 90]: 0 is the circle branch z = 0, +/-90 the
    // lines y = 0 (a branch and its opposite side are the same branch)
    let offset = |p: &V3,theta: f64| {
        let mut d = angle(p) - theta.to_degrees();
        while d <= -90. { d += 180.; }
        while d > 90. { d -= 180.; }
        d
    };
    let mut per: BTreeMap<u32,Vec<usize>> = Default::default();
    for (i,&c) in s.column.iter().enumerate() { per.entry(c).or_default().push(i); }
    eprintln!("== sheet 0: {} points in {} columns, on the wall's {{yz = 0}}",s.points.len(),per.len());
    eprintln!("   offset 0° = the circle branch z = 0 (the missing band); ±90° = the lines y = 0");
    let (mut circle,mut line) = (0usize,0usize);
    for (c,idx) in &per {
        let theta = s.times.get(*c as usize).copied().unwrap_or(f64::NAN);
        let mut near = 0;
        for &i in idx { let d = offset(&s.points[i],theta).abs();
            if d < 20. { near += 1; circle += 1; } else if d > 70. { line += 1; } }
        let shown: Vec<String> = idx.iter().take(6)
            .map(|&i| format!("{:>6.1}",offset(&s.points[i],theta))).collect();
        eprintln!("  column {c:>3} theta {:>7.2}°: {:>3} points, {near:>2} on the circle branch; \
            offsets{}{}",theta.to_degrees(),idx.len(),shown.join(""),if idx.len() > 6 { " …" } else { "" });
    }
    eprintln!("== {circle} points on the circle branch (z = 0), {line} on the line branches, of {}",
        s.points.len());
    eprintln!("   MEASURED: 17 points to a column over 9 columns, 149 of 153 on the circle branch. \
        The band is FULLY sampled, so dual stationing is refuted too — the loss is downstream of \
        the tracer's points, and `where_sheet_zeros_area_goes` asks where.");
}

/// Where sheet 0's area goes, from the tracer's own patch onward. Three findings stand behind this:
/// the `z = 0` band is a genuine gap (wedge area 0.0088 against the reference's 6.445 uncovered);
/// the circle branch is **fully** traced (17 points to a column over 9 columns, 149 of 153 points
/// on it), so neither a funnel nor dual stationing is the answer; and the surface it ought to sweep
/// has real area — the tool-frame circle (x−3)² + y² = 1 turned about x̂ through ±30° sweeps
/// ∮|y| ds × Δθ = 4 × π/3 ≈ **4.19**, the right order for the 6.445 that is missing.
///
/// So 4.19 becomes 0.0102 somewhere, and `which_pass_destroys_the_wedge` cannot say where because
/// it began at `Clipped`: **`Seeded` and `Labelled` run before it and were never measured.** This
/// asks three things in order — the area of sheet 0's **raw patch** triangles, the histogram of its
/// vertex **labels**, and the area of those patch triangles surviving the all-`Kept`/`Moved` rule
/// that `kept_triangles` applies. A raw area near 4.19 mostly dropped at labelling is a defect at
/// the judge; a raw area already near 0.01 is a degenerate zip inside the tracer, with the points
/// present and the triangles carrying nothing.
///
/// Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn where_sheet_zeros_area_goes() {
    use gcs_core::solid::swept_boundary::{Label,Stage,SweptBoundaryOptions,construct,seeds};
    use std::collections::BTreeMap;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let (_,sheets,_) = seeds(&e.sketch,swept,0.5,SAGITTA,&|_| {}).unwrap();
    let area_of = |pts: &[V3],tris: &[[u32;3]]| -> f64 {
        tris.iter().map(|t| {
            let [a,b,c] = t.map(|v| pts[v as usize]);
            let (p,q): (V3,V3) = (std::array::from_fn(|k| b[k]-a[k]),std::array::from_fn(|k| c[k]-a[k]));
            let n: V3 = [p[1]*q[2]-p[2]*q[1],p[2]*q[0]-p[0]*q[2],p[0]*q[1]-p[1]*q[0]];
            0.5*(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt()
        }).sum()
    };
    let s0 = &sheets[0];
    eprintln!("== sheet 0 raw from the tracer: {} points, {} triangles, area {:.4}",
        s0.points.len(),s0.triangles.len(),area_of(&s0.points,&s0.triangles));
    eprintln!("   (the swept circle wants about 4.19; the reference is short by 6.445)");
    // the largest and smallest raw triangle, to tell a degenerate zip from a small one
    let mut each: Vec<f64> = s0.triangles.iter()
        .map(|t| area_of(&s0.points,std::slice::from_ref(t))).collect();
    each.sort_by(|a,b| a.partial_cmp(b).unwrap());
    if !each.is_empty() {
        eprintln!("   raw triangle areas: least {:.3e}, median {:.3e}, greatest {:.3e}",
            each[0],each[each.len()/2],each[each.len()-1]);
    }
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        let Stage::Labelled {labelled,..} = &stage else { return };
        let l = &labelled[0];
        let mut hist: BTreeMap<String,usize> = Default::default();
        for lab in &l.labels { *hist.entry(format!("{lab:?}")).or_insert(0) += 1; }
        let keeps: Vec<[u32;3]> = s0.triangles.iter().copied()
            .filter(|t| t.iter().all(|&v| matches!(l.labels[v as usize],Label::Kept | Label::Moved)))
            .collect();
        eprintln!("   sheet 0 labels over {} vertices: {hist:?}",l.labels.len());
        eprintln!("   at the judged positions: all {} triangles area {:.4}; the {} all-Kept/Moved {:.4}",
            s0.triangles.len(),area_of(&l.points,&s0.triangles),keeps.len(),area_of(&l.points,&keeps));
        // how far the judge moved the points, which is what could collapse a strip
        let moved = l.moved_by.iter().fold(0_f64,|m,d| m.max(d.abs()));
        eprintln!("   the judge moved sheet 0's vertices by at most {moved:.4}");
    });
}

/// Whether the `z = 0` band is cap surface, and whether anything produces it. Sheet 0 is the
/// equator's sweep; the judge correctly calls it interior (`where_sheet_zeros_area_goes`); and the
/// closed form says the true boundary near z = 0 is attained at the roll's **ends**, since in the
/// plane z = 0 the swept region is ∪_θ {(x−3)² + y² cos²θ ≤ 1}, largest at |θ| = 30°, so at x = 3
/// it reaches |y| = 1/cos 30° = **1.1547** against the tool's own equator radius of 1.
///
/// Two questions, the first being the negative control for the second. **Where is the field's own
/// boundary at x = 3, z = 0?** Probed straight along y with no pipeline involved: 1.1547 confirms
/// the boundary there is end-pose surface, while 1.00 would mean the closed form is wrong and it is
/// merely the tool's equator — in which case everything built on the prediction goes with it.
/// **And which seed carries anything in the band?** Per seed — traced, cap or region — its raw
/// area, its raw area in the wedge, and the area surviving the all-`Kept`/`Moved` rule. A cap with
/// raw wedge area that labelling empties is a defect in how caps are labelled; a cap with no raw
/// area there at all is a defect in `caps` itself, whose divider `n·v = 0` holds *identically*
/// along the equator and so cannot pick a side.
///
/// Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn whether_the_z_zero_band_is_cap_surface() {
    use gcs_core::solid::MaterialField;
    use gcs_core::solid::swept_boundary::{FieldJudge,Label,Seed,Sign,Stage,SweptBoundaryOptions,construct};
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};

    // (1) the field alone, along y at x = 3, z = 0
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let mut judge = FieldJudge::new(field,options.judge_tolerance(),4000,1000,4096);
    eprintln!("== the field along y at x = 3, z = 0 (closed form: material out to |y| = 1.1547)");
    let (mut last_material,mut first_exterior) = (f64::NAN,f64::NAN);
    for k in 0..=28 {
        let y = k as f64*0.05;
        let word = match judge.sign([3.,y,0.]) {
            Ok((Sign::Material,_)) => "material",Ok((Sign::Exterior,_)) => "exterior",
            Ok((Sign::Near {..},_)) => "near",Ok((Sign::Unresolved,_)) => "unresolved",Err(_) => "error" };
        if word == "material" { last_material = y; }
        if word == "exterior" && first_exterior.is_nan() { first_exterior = y; }
        if (0.90..=1.30).contains(&y) { eprintln!("   y {y:.2}: {word}"); }
    }
    eprintln!("   last material at y {last_material:.2}, first exterior at y {first_exterior:.2}");
    eprintln!("   1.15 confirms the boundary there is END-POSE surface; 1.00 refutes the closed form");

    // (2) every seed against the band
    let angle = |p: &V3| p[2].atan2(p[1]).to_degrees();
    let in_wedge = |p: &V3| { let a = angle(p).abs(); a < 15. || a > 165. };
    let area_of = |pts: &[V3],tris: &[[u32;3]]| -> f64 {
        tris.iter().map(|t| {
            let [a,b,c] = t.map(|v| pts[v as usize]);
            let (p,q): (V3,V3) = (std::array::from_fn(|k| b[k]-a[k]),std::array::from_fn(|k| c[k]-a[k]));
            let n: V3 = [p[1]*q[2]-p[2]*q[1],p[2]*q[0]-p[0]*q[2],p[0]*q[1]-p[1]*q[0]];
            0.5*(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt()
        }).sum()
    };
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        let Stage::Labelled {seeds,labelled} = &stage else { return };
        eprintln!("== every seed: raw area, raw area in the wedge, and what survives labelling");
        let (mut raw_band,mut kept_band) = (0.,0.);
        for (i,(seed,l)) in seeds.iter().zip(labelled.iter()).enumerate() {
            let p = seed.patch();
            let kind = match seed {
                Seed::Traced(_) => "traced",Seed::Cap(_) => "cap",Seed::Grazing(_) => "region" };
            let wedge: Vec<[u32;3]> = p.triangles.iter().copied().filter(|t| {
                let [a,b,c] = t.map(|v| p.points[v as usize]);
                let m: V3 = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
                in_wedge(&m)
            }).collect();
            let kept: Vec<[u32;3]> = wedge.iter().copied()
                .filter(|t| t.iter().all(|&v| matches!(l.labels[v as usize],Label::Kept | Label::Moved)))
                .collect();
            let (w,k) = (area_of(&p.points,&wedge),area_of(&p.points,&kept));
            raw_band += w; kept_band += k;
            eprintln!("  seed {i:>2} {kind:>6}: raw {:8.4}, in the wedge {w:7.4} ({} tris), \
                surviving labelling {k:7.4} ({} tris)",
                area_of(&p.points,&p.triangles),wedge.len(),kept.len());
        }
        eprintln!("== the band over all seeds: raw {raw_band:.4}, surviving labelling {kept_band:.4}");
        eprintln!("   the reference says 6.445 of area is missing there");
    });
}

/// Where each seed's surface actually lies, because the per-seed band areas came back anomalous and
/// the anomaly must be resolved before anything is built on it. Both caps keep 9.4172 of area —
/// exactly half the wall (2π·1·2 / 2 = 6.283) plus half the two end discs (6.283 / 2 = 3.14), so a
/// cap does contain wall — yet **neither has a single triangle in the wedge, raw**.
///
/// That contradicts the closed form, which the field has just confirmed (material to y = 1.15 at
/// x = 3, z = 0; the boundary at 1/cos 30° = 1.1547). The world boundary point (3, 1.1547, 0) is
/// the θ = 30° image of the tool-frame wall point (3, 1, −0.577): R_θ(x,y,z) =
/// (x, y cos θ − z sin θ, y sin θ + z cos θ) sends it to (3, 1·0.866 + 0.577·0.5, 1·0.5 −
/// 0.577·0.866) = (3, 1.1547, 0). That facet's n·v = −yz = +0.577 > 0 — advancing, which `End::To`
/// is meant to keep. So it should be present and in the wedge, and it is not.
///
/// Three possibilities, and this separates them rather than guessing: the cap has no surface near
/// z = 0 at all (my derivation is wrong), it has some at angles the wedge predicate misses (the
/// predicate is wrong), or it has some and the component keep rule drops it (the defect). The
/// measurement is the distribution of world |z| and angle over each seed's own triangle centroids,
/// with no wedge predicate involved in the histogram.
///
/// Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn where_the_caps_surface_lies() {
    use gcs_core::solid::swept_boundary::{Seed,Stage,SweptBoundaryOptions,construct};
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        let Stage::Labelled {seeds,..} = &stage else { return };
        eprintln!("== every seed: where its triangles' centroids lie, in world coordinates");
        eprintln!("   angle = atan2(z,y) about the tumble axis; the band in question is z near 0");
        for (i,seed) in seeds.iter().enumerate() {
            let p = seed.patch();
            let kind = match seed {
                Seed::Traced(_) => "traced",Seed::Cap(_) => "cap",Seed::Grazing(_) => "region" };
            let mids: Vec<V3> = p.triangles.iter().map(|t| {
                let [a,b,c] = t.map(|v| p.points[v as usize]);
                std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.)
            }).collect();
            if mids.is_empty() { eprintln!("  seed {i:>2} {kind:>6}: no triangles"); continue; }
            let near_zero = mids.iter().filter(|m| m[2].abs() < 0.1).count();
            let (zlo,zhi) = mids.iter().fold((f64::INFINITY,f64::NEG_INFINITY),
                |(lo,hi),m| (lo.min(m[2]),hi.max(m[2])));
            // the angles, in twelve 30° bins, so nothing depends on the wedge's own cut-off
            let mut bins = [0usize;12];
            for m in &mids {
                let a = m[2].atan2(m[1]).to_degrees()+180.;
                bins[((a/30.).floor() as usize).min(11)] += 1;
            }
            eprintln!("  seed {i:>2} {kind:>6}: {:>4} tris, z {zlo:>6.3}..{zhi:>6.3}, \
                {near_zero:>3} with |z| < 0.1; angle bins (-180°..180° by 30°) {bins:?}",mids.len());
        }
    });
}

/// What each cap kept and what it dropped. `where_the_caps_surface_lies` showed both caps avoiding
/// the missing band exactly: a 60°-wide hole centred on angle 0 (the bins for [−30°,30°) are zero
/// for both), and only 6 of 2161 triangles with |z| < 0.1, while spanning z −1.362..1.362 — which
/// also confirms the patches are posed into world, since max |z| at θ = 30° is 0.5·1 + 0.866·1 =
/// 1.366.
///
/// But `Cap::patch` holds the **kept** facets only, so that cannot tell "never there" from
/// "dropped", and deriving which quadrant each cap keeps has tangled twice already. `components` is
/// public and answers it outright: per component its facet count, its most decisive relative normal
/// velocity, and whether the cap kept it. A large **dropped** component is where the band went, and
/// confirms or kills the swallowing mechanism — that `caps` keeps on `e.abs() < decisive ||
/// e.signum() == wanted` with `e` the extreme over a whole connected component, components joining
/// across every non-cut edge, and cuts running only along the sheets' end columns.
///
/// It also pins which cap is `From` and which `To`. Every sign argument made so far has *guessed*
/// that pairing, which is reason enough to measure it rather than assume it again.
///
/// Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn what_each_cap_kept_and_dropped() {
    use gcs_core::solid::swept_boundary::{CapComponent,Seed,Stage,SweptBoundaryOptions,construct};
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        let Stage::Labelled {seeds,..} = &stage else { return };
        eprintln!("== each cap: which end it stands at, and every component it cut");
        for (i,seed) in seeds.iter().enumerate() {
            let Seed::Cap(c) = seed else { continue };
            let kept: usize = c.components.iter().filter(|k| k.kept).map(|k| k.facets).sum();
            let dropped: usize = c.components.iter().filter(|k| !k.kept).map(|k| k.facets).sum();
            eprintln!("  seed {i:>2}: end {:?}, {} components, {kept} facets kept, {dropped} dropped",
                c.end,c.components.len());
            let mut rows: Vec<&CapComponent> = c.components.iter().collect();
            rows.sort_by(|a,b| b.facets.cmp(&a.facets));
            for k in rows.iter().take(14) {
                eprintln!("       {:>6} facets, extreme {:>10.6}, {}",
                    k.facets,k.extreme,if k.kept { "kept" } else { "DROPPED" });
            }
            if rows.len() > 14 { eprintln!("       … {} more components",rows.len()-14); }
        }
    });
}

/// Whether a cap facet's own sign disagrees with its component's — the one link still inferred, and
/// the fifth diagnosis of this band, the four before it having each been refuted by the next
/// measurement. So it is written to be falsifiable in one line.
///
/// `caps` reduces a cap to whole connected components, each judged by its **most decisive** facet's
/// relative normal velocity: kept when `e.abs() < decisive || e.signum() == wanted`. The measured
/// components are four large (~850 facets, the wall's quadrants) and four small (~240, the discs),
/// every extreme a clean ±0.704322 or ±0.673057 — so the grazing escape `e.abs() < 0.25` never
/// fires here and nothing is near a knife edge. The claim under test is that the band at world
/// z = 0 lies along the curve z_t = −y_t tan θ, which runs through a quadrant's **interior** while
/// the cuts run only along the quadrant boundaries (the end columns, z_t = 0 and y_t = 0) — so a
/// facet whose own sign is wanted is dropped with the component around it.
///
/// The cap mesh is rebuilt **identically** to `caps`, with every constant read from the library and
/// none guessed: `sagitta` 0.02, `snap()` = `vertex_tolerance()` = 0.005, `longest` = `spacing` 0.5,
/// `decisive` 0.25, `wanted` +1 at `To`, and no grazing planes (this tool has none — its wall is
/// curved and its discs' normals are ±ẑ against a tumble axis of x̂, and the seed listing shows no
/// region seed). Any mismatch in those would measure a mesh `caps` never built.
///
/// **The proof is a facet with `own > 0` in a component whose `extreme < 0`.** None, and the
/// diagnosis is dead.
///
/// Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn whether_a_cap_facets_own_sign_disagrees_with_its_components() {
    use gcs_core::model::SolidDef;
    use gcs_core::motion::Family;
    use gcs_core::solid::swept_boundary::{CutMesh,seeds};
    use gcs_core::solid::{indexed_faces,static_solid_at_unit};
    use std::collections::BTreeMap;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let (sagitta,spacing,snap,decisive) = (SAGITTA,0.5_f64,SAGITTA/4.,0.25_f64);
    let (_,sheets,_) = seeds(&e.sketch,swept,spacing,sagitta,&|_| {}).unwrap();
    let SolidDef::Swept {source: src,motion,from,to} = &e.sketch.solids[swept].def else { panic!("not a sweep") };
    let unit = sagitta/gcs_core::curve::FLATNESS_PX;
    let tool = static_solid_at_unit(&e.sketch,*src as usize,unit,0).unwrap();
    let (vertices,triangles,faces) = indexed_faces(&tool.boundary().unwrap());
    let family = Family::read(&e.sketch,*motion as usize).unwrap();
    let tol = 1e-9*(to.value-from.value).abs().max(1.);
    // the To cap, where `wanted` is +1
    let t = to.value;
    let pose = family.at(t).unwrap();
    let mut mesh = CutMesh::new(vertices.iter().map(|p| pose.point(*p)).collect(),triangles,faces,snap,sagitta);
    mesh.refine(spacing);
    for s in &sheets {
        let c = s.times.len().saturating_sub(1) as u32;
        let Some(at) = s.times.last().copied() else { continue };
        if (at-t).abs() > tol { continue; }
        let line: Vec<(V3,V3)> = s.column_vertices(c).into_iter()
            .map(|v| (s.points[v as usize],s.normals[v as usize])).collect();
        if line.len() < 2 { continue; }
        mesh.cut_along(&line,s.closed).unwrap();
    }
    let inverse = pose.inverse();
    let facet_normal = |m: &CutMesh,i: usize| -> Option<V3> {
        let [a,b,c] = m.triangles[i].map(|v| m.vertices[v as usize]);
        let (p,q): (V3,V3) = (std::array::from_fn(|k| b[k]-a[k]),std::array::from_fn(|k| c[k]-a[k]));
        let n: V3 = [p[1]*q[2]-p[2]*q[1],p[2]*q[0]-p[0]*q[2],p[0]*q[1]-p[1]*q[0]];
        let len = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
        (len > 0.).then(|| n.map(|x| x/len))
    };
    let relative: Vec<f64> = (0..mesh.triangles.len()).map(|i| {
        let Some(n) = facet_normal(&mesh,i) else { return 0. };
        let centroid: V3 = std::array::from_fn(|k|
            mesh.triangles[i].iter().map(|&v| mesh.vertices[v as usize][k]).sum::<f64>()/3.);
        let v = pose.velocity(inverse.point(centroid));
        let speed = (v[0]*v[0]+v[1]*v[1]+v[2]*v[2]).sqrt();
        if speed > 0. { (n[0]*v[0]+n[1]*v[1]+n[2]*v[2])/speed } else { 0. }
    }).collect();
    let component = mesh.components();
    let mut extreme: BTreeMap<usize,f64> = Default::default();
    for i in 0..mesh.triangles.len() {
        let en = extreme.entry(component[i]).or_insert(0.);
        if relative[i].abs() > en.abs() { *en = relative[i]; }
    }
    eprintln!("== the To cap rebuilt as `caps` builds it: {} facets, {} components, {} cuts",
        mesh.triangles.len(),extreme.len(),mesh.cuts.len());
    // the facets near the band, and whether each one's own sign matches the component deciding it
    let mut disagree = 0usize;
    let mut near_band = 0usize;
    let mut worst: Option<(f64,f64,V3)> = None;
    for i in 0..mesh.triangles.len() {
        let centroid: V3 = std::array::from_fn(|k|
            mesh.triangles[i].iter().map(|&v| mesh.vertices[v as usize][k]).sum::<f64>()/3.);
        if centroid[2].abs() >= 0.1 { continue; }
        near_band += 1;
        let en = extreme[&component[i]];
        let kept = en.abs() < decisive || en.signum() == 1.;
        // its own sign says advancing (wanted at To) while the component it belongs to is dropped
        if relative[i] > 0. && !kept {
            disagree += 1;
            if worst.map_or(true,|(r,_,_)| relative[i] > r) { worst = Some((relative[i],en,centroid)); }
        }
    }
    eprintln!("   {near_band} facets with |z| < 0.1; {disagree} of them advance (own sign > 0) \
        inside a DROPPED component");
    if let Some((own,en,at)) = worst {
        eprintln!("   the clearest: own {own:+.6}, its component's extreme {en:+.6}, at \
            [{:.3},{:.3},{:.3}]",at[0],at[1],at[2]);
    }
    eprintln!("   nonzero PROVES the band is lost inside a component; zero REFUTES this diagnosis");
    // The fix this once named — cutting each cap along its own n·v = 0 divider — was refuted:
    // measured, every branch of that divider is ALREADY cut by the sheets' end columns, so it
    // would have been a no-op (`why_the_caps_cuts_fail_to_separate_the_sign_regions`). What
    // actually straddles is the tool's own sharp edge, where the normal jumps, and the fix is to
    // cut every tool-face boundary.
    eprintln!("   (the fix is cutting every TOOL-FACE boundary — the divider cut was refuted as a \
        no-op, every divider branch being cut already)");
}

/// Which cases a change to the cap's cutting would move, asked **before** writing it. The tumbling
/// cylinder loses its band because a cap component carries both signs and one is dropped with the
/// other (`whether_a_cap_facets_own_sign_disagrees_with_its_components`: 364 of 370 band facets
/// advance inside a dropped component). Any such change touches `caps`, and `caps` runs for
/// **every** case — so this counts the straddling facets of all five, at both ends.
///
/// The change it was written for was a divider cut, and that was refuted: every branch of the
/// `n·v = 0` divider is already cut by the sheets' end columns, so it would have been a no-op. The
/// fix that landed cuts every **tool-face** boundary instead. This instrument outlived the
/// hypothesis because what it measures — which components carry both signs decisively — is the
/// right question either way, and its prediction is what the export gate then confirmed: 0
/// straddling for prism, lens, turned box and dumbbell, all four byte-identical; the tumbling
/// cylinder alone straddling, and alone moved.
///
/// A case with **no** straddling facet must come out byte-identical, and saying so in advance makes
/// the export diff a prediction rather than an explanation invented after the fact. That matters
/// here: four diagnoses of this band have already been refuted, each time by a quantity that merely
/// looked like the right one.
///
/// Straddling is the honest measure, not a facet count: a component straddles when its facets carry
/// **both** signs decisively, so one sign must be dropped with the other. Reported per cap as the
/// number of components that straddle, the facets on the wrong side of the keep, and their area.
///
/// Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn which_cases_a_divider_cut_would_move() {
    use gcs_core::model::SolidDef;
    use gcs_core::motion::Family;
    use gcs_core::solid::swept_boundary::{CutMesh,seeds};
    use gcs_core::solid::{indexed_faces,static_solid_at_unit};
    use std::collections::BTreeMap;
    let (sagitta,spacing,decisive) = (SAGITTA,0.5_f64,0.25_f64);
    let snap = sagitta/4.;
    let cases: [(&str,String); 5] = [
        ("turning_prism",turning_prism()),
        ("turned_lens",turned_lens()),
        ("tumbling_cylinder",tumbling_cylinder()),
        ("turned_box",turned_box()),
        ("sliding_dumbbell",sliding_dumbbell()),
    ];
    eprintln!("== straddling cap components per case: where one sign must be dropped with the other");
    for (name,source) in &cases {
        let e = harness::read(source);
        let swept = harness::solid(&e,"swept");
        let (_,sheets,_) = seeds(&e.sketch,swept,spacing,sagitta,&|_| {}).unwrap();
        let SolidDef::Swept {source: src,motion,from,to} = &e.sketch.solids[swept].def
            else { eprintln!("  {name}: not a sweep"); continue };
        let unit = sagitta/gcs_core::curve::FLATNESS_PX;
        let tool = static_solid_at_unit(&e.sketch,*src as usize,unit,0).unwrap();
        let (vertices,triangles,faces) = indexed_faces(&tool.boundary().unwrap());
        let family = Family::read(&e.sketch,*motion as usize).unwrap();
        let tol = 1e-9*(to.value-from.value).abs().max(1.);
        // the grazing planes exactly as `construct_from` hands them to `caps`: without these the
        // turned box reads as one uncut component and the instrument invents a defect it has not got
        let sweep = gcs_core::solid::SweepContacts::read(&e.sketch,swept,1e-10).unwrap();
        let grazing: Vec<(V3,V3)> = gcs_core::solid::swept_boundary::grazing_faces(&sweep).unwrap()
            .iter().map(|f| (f.origin,f.outward)).collect();
        for (label,t,wanted) in [("From",from.value,-1.),("To",to.value,1.)] {
            let pose = family.at(t).unwrap();
            let mut mesh = CutMesh::new(vertices.iter().map(|p| pose.point(*p)).collect(),
                triangles.clone(),faces.clone(),snap,sagitta);
            mesh.refine(spacing);
            for s in &sheets {
                let (c,at) = if label == "From" { (0,s.times.first().copied()) }
                    else { ((s.times.len().saturating_sub(1)) as u32,s.times.last().copied()) };
                let Some(at) = at else { continue };
                if (at-t).abs() > tol { continue; }
                let line: Vec<(V3,V3)> = s.column_vertices(c).into_iter()
                    .map(|v| (s.points[v as usize],s.normals[v as usize])).collect();
                if line.len() < 2 { continue; }
                if mesh.cut_along(&line,s.closed).is_err() { continue; }
            }
            // `caps`' own grazing treatment: a facet lying in a grazing face's plane is `left`
            // (the region sweeps it instead), and the edges between left and the rest are cuts, so
            // no component reaches through them. `facet_normal` is not public, hence inline.
            let normal_at = |m: &CutMesh,i: usize| -> Option<V3> {
                let [a,b,c] = m.triangles[i].map(|v| m.vertices[v as usize]);
                let (p,q): (V3,V3) = (std::array::from_fn(|k| b[k]-a[k]),std::array::from_fn(|k| c[k]-a[k]));
                let n: V3 = [p[1]*q[2]-p[2]*q[1],p[2]*q[0]-p[0]*q[2],p[0]*q[1]-p[1]*q[0]];
                let len = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
                (len > 0.).then(|| n.map(|x| x/len))
            };
            let dot3 = |a: V3,b: V3| a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
            let left: Vec<bool> = (0..mesh.triangles.len()).map(|i| {
                let Some(facet) = normal_at(&mesh,i) else { return false };
                grazing.iter().any(|&(o,n)| dot3(facet,n).abs() > 1.-1e-6
                    && mesh.triangles[i].iter().all(|&v| {
                        let p = mesh.vertices[v as usize];
                        dot3([p[0]-o[0],p[1]-o[1],p[2]-o[2]],n).abs() <= snap
                    }))
            }).collect();
            if left.iter().any(|x| *x) {
                let mut by_edge: BTreeMap<(u32,u32),Vec<usize>> = Default::default();
                for (i,t) in mesh.triangles.iter().enumerate() {
                    for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]);
                        by_edge.entry((a.min(b),a.max(b))).or_default().push(i); }
                }
                let edges: Vec<(u32,u32)> = by_edge.into_iter()
                    .filter(|(_,f)| f.len() == 2 && left[f[0]] != left[f[1]]).map(|(e,_)| e).collect();
                for e in edges { mesh.cuts.insert(e); }
            }
            let inverse = pose.inverse();
            let measure = |m: &CutMesh,i: usize| -> Option<(f64,f64)> {
                let [a,b,c] = m.triangles[i].map(|v| m.vertices[v as usize]);
                let (p,q): (V3,V3) = (std::array::from_fn(|k| b[k]-a[k]),std::array::from_fn(|k| c[k]-a[k]));
                let n: V3 = [p[1]*q[2]-p[2]*q[1],p[2]*q[0]-p[0]*q[2],p[0]*q[1]-p[1]*q[0]];
                let len = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
                if !(len > 0.) { return None; }
                let n = n.map(|x| x/len);
                let centroid: V3 = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
                let v = pose.velocity(inverse.point(centroid));
                let speed = (v[0]*v[0]+v[1]*v[1]+v[2]*v[2]).sqrt();
                let rel = if speed > 0. { (n[0]*v[0]+n[1]*v[1]+n[2]*v[2])/speed } else { 0. };
                Some((rel,0.5*len))
            };
            let component = mesh.components();
            let mut extreme: BTreeMap<usize,f64> = Default::default();
            for i in 0..mesh.triangles.len() {
                let Some((rel,_)) = measure(&mesh,i) else { continue };
                let en = extreme.entry(component[i]).or_insert(0.);
                if rel.abs() > en.abs() { *en = rel; }
            }
            // A component straddles when it holds facets decisively signed BOTH ways. `left`
            // facets are excluded: the region sweeps them, and `caps` keeps them out of the patch
            // whatever their sign. `extreme` above is over every facet, left included, as `caps`
            // computes it.
            let mut both: BTreeMap<usize,(bool,bool)> = Default::default();
            for i in 0..mesh.triangles.len() {
                if left[i] { continue; }
                let Some((rel,_)) = measure(&mesh,i) else { continue };
                if rel.abs() < decisive { continue; }
                let s = both.entry(component[i]).or_insert((false,false));
                if rel > 0. { s.0 = true; } else { s.1 = true; }
            }
            let straddling: Vec<usize> = both.iter().filter(|(_,(p,n))| *p && *n).map(|(c,_)| *c).collect();
            // the facets a keep would drop although their own sign is wanted
            let (mut lost,mut lost_area) = (0usize,0.);
            for i in 0..mesh.triangles.len() {
                if left[i] { continue; }
                let Some((rel,area)) = measure(&mesh,i) else { continue };
                let en = extreme[&component[i]];
                let kept = en.abs() < decisive || en.signum() == wanted;
                if !kept && rel.signum() == wanted && rel.abs() >= decisive { lost += 1; lost_area += area; }
            }
            eprintln!("  {name:>18} {label:>4}: {} components, {} straddling, {} grazing facets; \
                {lost} facets lost against their own sign, area {lost_area:.4}",
                extreme.len(),straddling.len(),left.iter().filter(|x| **x).count());
        }
    }
    eprintln!("   a case with 0 straddling must stay byte-identical when the divider cut lands");
}

/// Why the cap's cuts fail to separate the sign regions — the hole in "cut each cap along its own
/// `n·v = 0` divider". On the wall `n·v = −z·sin φ`, so the divider is `{z = 0} ∪ {sin φ = 0}`: the
/// equator and the two lines meeting the wall at x = 2 and x = 4. It is **pose-independent**, the
/// turn about x̂ leaving x̂ fixed in the tool's own frame, and those three branches are exactly the
/// contact curves at the end pose — which is what `caps` already cuts along. So that fix is
/// nominally already done, and stating it as the fix risks changing nothing.
///
/// Yet components demonstrably straddle (4 of 8, 853 facets and 3.6463 of area lost against their
/// own sign). Two different causes want two different fixes:
///
/// 1. **Branch coverage.** No traced sheet's end column covers `sin φ = 0` — the branch
///    classification found sheet 0 carrying 149 of 153 points on the *equator* branch and **zero**
///    on the line branches — so quadrants stay joined across φ = 0 and φ = π.
/// 2. **Topological failure.** The columns lie on the right curves but do not separate: an open
///    column short of the mesh boundary, or a gap where `cut_along` declined, leaves the flood
///    connected around the end.
///
/// The per-component histogram over the four tool-frame quadrants (signs of `z_t`, `y_t`) tells
/// them apart. A component holding both (+z,+y) and (+z,−y) means the **line** branch is uncut
/// there; one holding (+z,+y) and (−z,+y) means the **equator** is. Each contributing column is
/// reported too: its size, its `closed` flag, and its divider residual `max|y·z|` in tool frame,
/// which must be ≈ 0 if it really lies on the divider.
///
/// Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn why_the_caps_cuts_fail_to_separate_the_sign_regions() {
    use gcs_core::model::SolidDef;
    use gcs_core::motion::Family;
    use gcs_core::solid::swept_boundary::{CutMesh,seeds};
    use gcs_core::solid::{indexed_faces,static_solid_at_unit};
    use std::collections::BTreeMap;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let (sagitta,spacing,decisive) = (SAGITTA,0.5_f64,0.25_f64);
    let snap = sagitta/4.;
    let (_,sheets,_) = seeds(&e.sketch,swept,spacing,sagitta,&|_| {}).unwrap();
    let SolidDef::Swept {source: src,motion,from,to} = &e.sketch.solids[swept].def
        else { panic!("not a sweep") };
    let unit = sagitta/gcs_core::curve::FLATNESS_PX;
    let tool = static_solid_at_unit(&e.sketch,*src as usize,unit,0).unwrap();
    let (vertices,triangles,faces) = indexed_faces(&tool.boundary().unwrap());
    let family = Family::read(&e.sketch,*motion as usize).unwrap();
    let tol = 1e-9*(to.value-from.value).abs().max(1.);
    let t = to.value;
    let pose = family.at(t).unwrap();
    let inverse = pose.inverse();
    let mut mesh = CutMesh::new(vertices.iter().map(|p| pose.point(*p)).collect(),
        triangles,faces,snap,sagitta);
    mesh.refine(spacing);
    eprintln!("== the To cap at t {t:.4}: which end columns cut it, and on which divider branch");
    eprintln!("   the wall's divider is n·v = -z·sin(phi) = 0: the equator z = 0 and the lines y = 0");
    let mut cut = 0usize;
    for (i,s) in sheets.iter().enumerate() {
        let c = s.times.len().saturating_sub(1) as u32;
        let Some(at) = s.times.last().copied() else { continue };
        if (at-t).abs() > tol {
            eprintln!("  sheet {i:>2}: ends at {at:.4}, not the cap's {t:.4} — contributes no cut");
            continue;
        }
        let ids = s.column_vertices(c);
        let (mut worst,mut zlo,mut zhi,mut ylo,mut yhi) = (0_f64,f64::INFINITY,f64::NEG_INFINITY,f64::INFINITY,f64::NEG_INFINITY);
        for &v in &ids {
            let p = inverse.point(s.points[v as usize]);
            worst = worst.max((p[1]*p[2]).abs());
            zlo = zlo.min(p[2]); zhi = zhi.max(p[2]);
            ylo = ylo.min(p[1]); yhi = yhi.max(p[1]);
        }
        let line: Vec<(V3,V3)> = ids.iter()
            .map(|&v| (s.points[v as usize],s.normals[v as usize])).collect();
        let outcome = if line.len() < 2 { "too short to cut".to_string() }
            else { match mesh.cut_along(&line,s.closed) { Ok(()) => { cut += 1; "cut".into() },
                Err(m) => format!("DECLINED: {m}") } };
        eprintln!("  sheet {i:>2}: {:>3} points, closed {:>5}, tool-frame y {ylo:>6.3}..{yhi:>6.3} \
            z {zlo:>6.3}..{zhi:>6.3}, max |y·z| {worst:.4} — {outcome}",ids.len(),s.closed);
    }
    eprintln!("   {cut} columns cut in, {} cut edges", mesh.cuts.len());
    // The same tally before and after, so the two are strictly comparable. Each component's facets
    // by tool-frame quadrant, which tool faces it spans, its relative-normal-velocity range, and
    // the facets a keep would drop although their own sign is wanted.
    let normal_at = |m: &CutMesh,i: usize| -> Option<V3> {
        let [a,b,c] = m.triangles[i].map(|v| m.vertices[v as usize]);
        let (p,q): (V3,V3) = (std::array::from_fn(|k| b[k]-a[k]),std::array::from_fn(|k| c[k]-a[k]));
        let n: V3 = [p[1]*q[2]-p[2]*q[1],p[2]*q[0]-p[0]*q[2],p[0]*q[1]-p[1]*q[0]];
        let len = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
        (len > 0.).then(|| n.map(|x| x/len))
    };
    let tally = |mesh: &CutMesh,label: &str| {
        let component = mesh.components();
        let mut rows: BTreeMap<usize,([usize;4],std::collections::BTreeSet<u32>,f64,f64,usize)> = Default::default();
        let mut rel_of = vec![0_f64;mesh.triangles.len()];
        for i in 0..mesh.triangles.len() {
            let Some(n) = normal_at(mesh,i) else { continue };
            let centroid: V3 = std::array::from_fn(|k|
                mesh.triangles[i].iter().map(|&v| mesh.vertices[v as usize][k]).sum::<f64>()/3.);
            let v = pose.velocity(inverse.point(centroid));
            let speed = (v[0]*v[0]+v[1]*v[1]+v[2]*v[2]).sqrt();
            let rel = if speed > 0. { (n[0]*v[0]+n[1]*v[1]+n[2]*v[2])/speed } else { 0. };
            rel_of[i] = rel;
            let local = inverse.point(centroid);
            let q = (if local[2] > 0. { 0 } else { 2 })+(if local[1] > 0. { 0 } else { 1 });
            let row = rows.entry(component[i]).or_insert(([0;4],Default::default(),f64::INFINITY,f64::NEG_INFINITY,0));
            row.0[q] += 1; row.1.insert(mesh.faces[i]);
            row.2 = row.2.min(rel); row.3 = row.3.max(rel); row.4 += 1;
        }
        let mut extreme: BTreeMap<usize,f64> = Default::default();
        for i in 0..mesh.triangles.len() {
            let en = extreme.entry(component[i]).or_insert(0.);
            if rel_of[i].abs() > en.abs() { *en = rel_of[i]; }
        }
        let (mut lost,mut straddling) = (0usize,0usize);
        for i in 0..mesh.triangles.len() {
            let en = extreme[&component[i]];
            let kept = en.abs() < decisive || en.signum() == 1.;
            if !kept && rel_of[i] > decisive { lost += 1; }
        }
        let mut ordered: Vec<_> = rows.into_iter().collect();
        ordered.sort_by(|a,b| b.1.4.cmp(&a.1.4));
        eprintln!("== {label}: {} components", ordered.len());
        eprintln!("   facets, quadrants [+z+y, +z-y, -z+y, -z-y], tool faces spanned, relative range");
        for (_,(quad,faces_in,lo,hi,n)) in &ordered {
            let s = *lo < -decisive && *hi > decisive;
            if s { straddling += 1; }
            eprintln!("   {n:>5} {quad:?} faces {:?} relative {lo:>+.4}..{hi:>+.4}{}",
                faces_in.iter().collect::<Vec<_>>(),if s { "  STRADDLES" } else { "" });
        }
        eprintln!("   {straddling} components straddle; {lost} facets would be dropped against \
            their own sign");
    };
    tally(&mesh,"with only the contact columns cut, as `caps` does today");
    // Every edge between facets of DIFFERENT tool faces, as a cut. This is the rule `caps` already
    // applies to a grazing face's boundary — "a component must not reach through them to be judged
    // by a normal velocity that is not its own" — applied to all of the tool's own edges. Across a
    // sharp edge the normal jumps, so n·v changes sign discontinuously and one extreme cannot
    // speak for both sides.
    let mut by_edge: BTreeMap<(u32,u32),Vec<usize>> = Default::default();
    for (i,t) in mesh.triangles.iter().enumerate() {
        for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]);
            by_edge.entry((a.min(b),a.max(b))).or_default().push(i); }
    }
    let across: Vec<(u32,u32)> = by_edge.into_iter()
        .filter(|(_,f)| f.len() == 2 && mesh.faces[f[0]] != mesh.faces[f[1]])
        .map(|(e,_)| e).collect();
    eprintln!("== adding {} edges between different tool faces as cuts",across.len());
    for e in across { mesh.cuts.insert(e); }
    tally(&mesh,"with the tool's own face boundaries cut as well");
    eprintln!("   0 straddling and 0 lost ⇒ the defect is components spanning a sharp tool edge, \
        and the fix is to cut every tool-face boundary — not a new divider marcher");
}

/// What the degenerate three-vertex loops are made of. After the cap fix, six of the 21 refused
/// loops are 3-vertex and declined because *a fan triangle reads degenerate*
/// (`which_gate_refuses_each_unfilled_loop`), with least altitudes 2.575e-17, 4.189e-17, 9.181e-11,
/// 5.623e-12, 1.202e-10 and 4.446e-12 — and the two the field calls `Spanned`, 13 and 14, are the
/// two at ~1e-17. `lay` refusing a corner set the mesh already carries is **not** the cause: that
/// instrument reports 0 of 21 refused by an already-walked edge.
///
/// A zero-area hole must not be filled with a degenerate triangle — flatness is `space::degenerate`
/// and never exactly zero area — so the treatment depends on what the three vertices actually are,
/// and there are three cases with three different fixes:
///
///   * **two coincide** — a needle `weld`/`collapse_short_edges` should have merged;
///   * **three distinct and collinear, one projecting inside the opposite edge's span** — a
///     T-junction `split_at_vertices` should have resolved, its tolerance being `junction` (2
///     sagittas);
///   * **three distinct, collinear, none inside the others' span** — a genuine zero-area sliver,
///     whose honest treatment is to drop it rather than to span it.
///
/// The projection *fraction* along the opposite edge is what separates the second from the third,
/// so it is reported beside each altitude; an altitude alone cannot tell them apart. Nothing is
/// built or changed here; it only measures.
#[test]
#[ignore]
fn what_the_degenerate_three_loops_are_made_of() {
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct};
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    eprintln!("== the 3-vertex refused loops (junction tolerance {:.4}, shortest edge {:.5})",
        options.junction(),options.shortest_edge());
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        let Stage::Zipped {mesh,unpaired,..} = stage else { return };
        let d = |a: V3,b: V3| ((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt();
        for (k,l) in unpaired.iter().enumerate() {
            if l.len() != 3 { continue; }
            let p: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
            let round = |q: V3| q.map(|x| (x*1e5).round()/1e5);
            eprintln!("  loop {k:>2}: ids {:?}", l);
            eprintln!("      at {:?} {:?} {:?}",round(p[0]),round(p[1]),round(p[2]));
            eprintln!("      edge lengths {:.3e} {:.3e} {:.3e}",d(p[0],p[1]),d(p[1],p[2]),d(p[2],p[0]));
            for i in 0..3 {
                let (v,a,b) = (p[i],p[(i+1)%3],p[(i+2)%3]);
                let ab: V3 = std::array::from_fn(|c| b[c]-a[c]);
                let av: V3 = std::array::from_fn(|c| v[c]-a[c]);
                let ll = ab[0]*ab[0]+ab[1]*ab[1]+ab[2]*ab[2];
                let f = if ll > 0. { (av[0]*ab[0]+av[1]*ab[1]+av[2]*ab[2])/ll } else { f64::NAN };
                let foot: V3 = std::array::from_fn(|c| a[c]+f*ab[c]);
                let h = d(v,foot);
                let where_it_sits = if !(0. ..=1.).contains(&f) { "past an end" }
                    else if h <= options.junction() { "INSIDE the span, within the junction — a T-junction split should reach it" }
                    else { "inside the span but beyond the junction" };
                eprintln!("      vertex {i} (id {:>5}) over the opposite edge: altitude {h:.3e}, \
                    projects at {f:+.4} — {where_it_sits}",l[i]);
            }
        }
    });
    eprintln!("   two coincident ⇒ weld; collinear with one inside the span ⇒ split_at_vertices; \
        otherwise a zero-area sliver to drop, never to span");
}

/// What makes the finished mesh look rough, measured rather than guessed. The tumbling cylinder
/// slices, so the surface is broadly right; the question is what shape its facets are and where
/// they come from. `the_longest_edges_the_mesh_walks` already rules out one answer: 154 of 6665
/// triangles have an edge past the spacing (0.5), the longest 0.5713, and none past twice it — so
/// this is not a handful of stretched giants. It also shows every one of the ten longest edges is
/// the **zip's**, clustered at x ≈ 2.0 and x ≈ 3.96, which are the two ends of the tumble axis.
///
/// Two candidates remain, wanting opposite work. A mesh that is **coarse but well shaped** is a
/// tessellation setting: the sagitta is 0.02 on a part two units across, and `caps` refines the
/// tool's own mesh only to `longest` = the column spacing, 0.5, so a cap facet may be half a unit
/// across — and the band this session recovered is cap surface, inheriting exactly that. A mesh
/// **full of needles** is a stitching artefact instead, which wrecks shading at any density.
///
/// So: over the finished mesh, split by where each triangle came from, the distribution of longest
/// edge, area and least altitude, and how many fall under the certificate's own least probe
/// (0.01), which is its definition of a sliver. The source split is what makes it actionable —
/// coarse caps and needly zip bands are different repairs.
///
/// The seed order is the fixture's: this tool traces nine sheets, so 9 and 10 are its two caps and
/// `u32::MAX` is a band the zip laid.
///
/// Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn what_makes_the_finished_mesh_rough() {
    use gcs_core::solid::swept_boundary::{Seed,Stage,SweptBoundaryOptions,construct};
    use std::collections::BTreeMap;
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let least = options.least_probe();
    let cases: [(&str,String); 5] = [
        ("turning_prism",turning_prism()),
        ("turned_lens",turned_lens()),
        ("tumbling_cylinder",tumbling_cylinder()),
        ("turned_box",turned_box()),
        ("sliding_dumbbell",sliding_dumbbell()),
    ];
    eprintln!("== facet shape by source, every case (least probe {least:.4}, sagitta {SAGITTA}, \
        spacing {})",options.spacing);
    eprintln!("   a well-shaped triangle's altitude is a good fraction of its longest edge, so a \
        median edge:altitude near 10 means the mesh is slivers whatever its density");
    for (name,source) in &cases {
    let e = harness::read(source);
    let swept = harness::solid(&e,"swept");
    // each seed's class, kept from the stage where the seeds are still in hand; a band the zip
    // laid carries `u32::MAX` instead of a seed index
    let mut class: Vec<&'static str> = Vec::new();
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Labelled {seeds,..} = &stage {
            class = seeds.iter().map(|s| match s {
                Seed::Traced(_) => "sheet",Seed::Cap(_) => "cap",Seed::Grazing(_) => "region" }).collect();
        }
        let Stage::Zipped {mesh,..} = stage else { return };
        let class_of = |s: u32| -> &'static str {
            if s == u32::MAX { "zip" } else { class.get(s as usize).copied().unwrap_or("?") } };
        let mut rows: BTreeMap<&str,(Vec<f64>,Vec<f64>,Vec<f64>)> = Default::default();
        for (i,t) in mesh.triangles.iter().enumerate() {
            let [a,b,c] = t.map(|v| mesh.vertices[v as usize]);
            let d = |p: V3,q: V3| ((p[0]-q[0]).powi(2)+(p[1]-q[1]).powi(2)+(p[2]-q[2]).powi(2)).sqrt();
            let longest = d(a,b).max(d(b,c)).max(d(c,a));
            let (u,v): (V3,V3) = (std::array::from_fn(|k| b[k]-a[k]),std::array::from_fn(|k| c[k]-a[k]));
            let n: V3 = [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
            let area = 0.5*(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
            let alt = gcs_core::space::altitude(a,b,c);
            let row = rows.entry(class_of(mesh.sheet.get(i).copied().unwrap_or(u32::MAX))).or_default();
            row.0.push(longest); row.1.push(area); row.2.push(alt);
        }
        let pct = |v: &[f64],p: f64| -> f64 {
            if v.is_empty() { return f64::NAN; }
            v[((((v.len()-1) as f64)*p).round() as usize).min(v.len()-1)]
        };
        eprintln!("  {name}: {} triangles",mesh.triangles.len());
        for (name,(edges,areas,alts)) in rows.iter_mut() {
            let n = edges.len();
            let slivers = alts.iter().filter(|h| **h < least).count();
            let flat = alts.iter().filter(|h| **h < SAGITTA).count();
            edges.sort_by(f64::total_cmp); areas.sort_by(f64::total_cmp); alts.sort_by(f64::total_cmp);
            eprintln!("  {name:>5}: {n:>5} triangles, {slivers} under the least probe \
                ({:.1}%), {flat} under the sagitta ({:.1}%)",
                100.*slivers as f64/n.max(1) as f64,100.*flat as f64/n.max(1) as f64);
            eprintln!("         longest edge   p10 {:.4}  median {:.4}  p90 {:.4}  max {:.4}",
                pct(edges,0.1),pct(edges,0.5),pct(edges,0.9),pct(edges,1.0));
            eprintln!("         area           p10 {:.3e}  median {:.3e}  p90 {:.3e}",
                pct(areas,0.1),pct(areas,0.5),pct(areas,0.9));
            eprintln!("         least altitude p10 {:.3e}  median {:.3e}  p90 {:.3e}",
                pct(alts,0.1),pct(alts,0.5),pct(alts,0.9));
        }
    });
    }
    eprintln!("   a case that is coarse but well shaped wants a tessellation setting; one whose \
        median edge:altitude is near 10 is built of slivers, which is a different repair — and \
        whether that is peculiar to one case or how the pipeline meshes everywhere is the point \
        of running all five");
}

/// Whether the open loops sit in slivery neighbourhoods — the test of whether facet shape is
/// *upstream* of the closure failure or merely cosmetic. Across the five cases, both that fail to
/// close carry a major source at edge:altitude 11 or worse while no closing case does
/// (`what_makes_the_finished_mesh_rough`), and every stitch failure chased in this work was a sliver
/// of some kind. That is a correlation over five cases, not a mechanism, and this asks the
/// within-case question the correlation implies.
///
/// **The control is the whole design.** "The open loops are surrounded by slivers" says nothing when
/// 30% of the entire mesh is slivers — it would confirm itself by construction. So the comparison is
/// against the seams the zip **closed**: it laid 1105 band triangles where it succeeded and left 17
/// loops where it did not, and both are seam regions. The question is then sharp: was the surface
/// the stitch was *given* slivery-er where it failed than where it succeeded?
///
/// **The zip's own output is excluded from both sides**, or the measurement is circular — a zip band
/// is itself 38.7% slivers, so counting bands near a loop would inflate precisely the number under
/// test. Only the sheet and cap triangles are measured: the input the stitch had, never the output
/// it made. Two radii are reported so the answer does not hinge on one choice that happens to
/// produce it.
///
/// **A limitation to keep in view:** within one mesh all boundary is *failed* boundary, so "near an
/// open loop" and "on the boundary at all" cannot be fully separated here. The zip-seam control is
/// the best available inside a single case; comparing against a closing case's seams would be
/// stronger and is more work.
///
/// Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn whether_the_open_loops_sit_in_slivery_neighbourhoods() {
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct};
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let least = options.least_probe();
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        let Stage::Zipped {mesh,unpaired,..} = stage else { return };
        let d = |p: V3,q: V3| ((p[0]-q[0]).powi(2)+(p[1]-q[1]).powi(2)+(p[2]-q[2]).powi(2)).sqrt();
        // each triangle's centroid, its edge:altitude ratio, its altitude, and whether the zip laid it
        let facets: Vec<(V3,f64,f64,bool)> = mesh.triangles.iter().enumerate().map(|(i,t)| {
            let [a,b,c] = t.map(|v| mesh.vertices[v as usize]);
            let longest = d(a,b).max(d(b,c)).max(d(c,a));
            let altitude = gcs_core::space::altitude(a,b,c);
            let at: V3 = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
            (at,if altitude > 0. { longest/altitude } else { f64::INFINITY },altitude,
                mesh.sheet.get(i).copied() == Some(u32::MAX))
        }).collect();
        // the non-zip surface within `radius` of any of `seeds`: how many, what fraction are
        // slivers, and the median ratio
        let around = |seeds: &[V3],radius: f64| -> (usize,f64,f64) {
            let (mut ratios,mut slivers) = (Vec::new(),0usize);
            for (at,ratio,altitude,zip) in facets.iter() {
                if *zip { continue; }
                if !seeds.iter().any(|s| d(*s,*at) <= radius) { continue; }
                if *altitude < least { slivers += 1; }
                ratios.push(*ratio);
            }
            ratios.sort_by(f64::total_cmp);
            let n = ratios.len();
            (n,100.*slivers as f64/n.max(1) as f64,if n == 0 { f64::NAN } else { ratios[n/2] })
        };
        // where the stitch failed, and where it succeeded
        let failed: Vec<V3> = unpaired.iter().flatten().map(|&v| mesh.vertices[v as usize]).collect();
        let closed: Vec<V3> = facets.iter().filter(|(_,_,_,zip)| *zip).map(|(at,..)| *at).collect();
        // the whole non-zip surface, as the flat baseline
        let (all_n,all_sliver,all_ratio) = around(&facets.iter().map(|(at,..)| *at).collect::<Vec<_>>(),f64::INFINITY);
        eprintln!("== does the stitch fail where the surface it was given is slivery?");
        eprintln!("   measured over sheet and cap triangles only — the zip's own bands are excluded \
            from both sides, or the test is circular");
        eprintln!("   the whole non-zip surface: {all_n} triangles, {all_sliver:.1}% slivers, \
            median edge:altitude {all_ratio:.1}");
        eprintln!("   {} open-loop vertices against {} closed-seam band centroids",failed.len(),closed.len());
        for radius in [0.05,0.15] {
            let (fn_,fs,fr) = around(&failed,radius);
            let (cn,cs,cr) = around(&closed,radius);
            eprintln!("   radius {radius:.2}:");
            eprintln!("      near the 17 loops the zip left OPEN: {fn_:>5} triangles, {fs:>5.1}% \
                slivers, median ratio {fr:>5.1}");
            eprintln!("      near the seams the zip CLOSED:       {cn:>5} triangles, {cs:>5.1}% \
                slivers, median ratio {cr:>5.1}");
        }
        // and the spread, since one bad loop could carry an aggregate
        eprintln!("   per loop at radius 0.15 (triangles, % slivers, median ratio):");
        for (k,l) in unpaired.iter().enumerate() {
            let seeds: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect();
            let (n,s,r) = around(&seeds,0.15);
            eprintln!("      loop {k:>2} ({} vertices): {n:>4}, {s:>5.1}%, {r:>5.1}",l.len());
        }
        eprintln!("   failed ≈ closed ⇒ sliveriness does NOT mark where the stitch fails, and facet \
            shape is cosmetic after all; failed much worse ⇒ it is upstream and belongs first");
    });
}

/// Whether each degenerate three-loop bounds a **lone** zero-area facet. `zip_round`'s own note says
/// this is what the `Spanned` loop is — "three boundary edges used once each with a triangle already
/// on those three vertices, so that triangle is their only user and the loop bounds a lone facet
/// with no neighbour" — and `split_where`'s note names the gap: "What wants removing is the needle…
/// A lone needle has nothing to remove it." `split_where` is right to refuse these: its
/// normal-agreement guard needs `|n1|` and `|n2|` over `least`, and splitting a collinear triangle
/// at a collinear point makes two zero-area triangles.
///
/// So the candidate fix is a pass that drops a degenerate facet no other triangle neighbours — a
/// zero-volume flap carrying no surface, removed rather than filled. **That is only safe if the
/// facet really is lone.** If any of its three edges has a second user, dropping it opens a new
/// boundary and tears the mesh, which would be worse than the hole. So this measures, per 3-loop:
/// how many triangles carry exactly those three corners, that triangle's area, and the use count of
/// each of its three edges. Three edges each used once by one degenerate triangle is the licence;
/// anything else refuses the fix.
///
/// Nothing is built or changed here; it only measures.
#[test]
#[ignore]
fn whether_each_degenerate_three_loop_bounds_a_lone_facet() {
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct};
    use std::collections::BTreeMap;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    eprintln!("== each 3-vertex refused loop: is the facet on its corners lone and degenerate?");
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        let Stage::Zipped {mesh,unpaired,..} = stage else { return };
        // every undirected edge's use count, and every corner set's triangles
        let mut uses: BTreeMap<(u32,u32),usize> = Default::default();
        for t in &mesh.triangles { for k in 0..3 {
            let (a,b) = (t[k],t[(k+1)%3]); *uses.entry((a.min(b),a.max(b))).or_insert(0) += 1; } }
        let mut by_corners: BTreeMap<[u32;3],Vec<usize>> = Default::default();
        for (i,t) in mesh.triangles.iter().enumerate() {
            let mut k = *t; k.sort(); by_corners.entry(k).or_default().push(i);
        }
        let area = |t: [u32;3]| -> f64 {
            let [a,b,c] = t.map(|v| mesh.vertices[v as usize]);
            let (p,q): (V3,V3) = (std::array::from_fn(|k| b[k]-a[k]),std::array::from_fn(|k| c[k]-a[k]));
            let n: V3 = [p[1]*q[2]-p[2]*q[1],p[2]*q[0]-p[0]*q[2],p[0]*q[1]-p[1]*q[0]];
            0.5*(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt()
        };
        for (k,l) in unpaired.iter().enumerate() {
            if l.len() != 3 { continue; }
            let mut key = [l[0],l[1],l[2]]; key.sort();
            let owners = by_corners.get(&key).cloned().unwrap_or_default();
            let edge_uses: Vec<usize> = (0..3)
                .map(|i| { let (a,b) = (l[i],l[(i+1)%3]); uses.get(&(a.min(b),a.max(b))).copied().unwrap_or(0) })
                .collect();
            let areas: Vec<String> = owners.iter().map(|&i| format!("{:.3e}",area(mesh.triangles[i]))).collect();
            // the facet's own sheet says where it came from: u32::MAX is one the zip laid
            let sheets: Vec<String> = owners.iter()
                .map(|&i| match mesh.sheet.get(i).copied() { Some(u32::MAX) => "zip".into(),
                    Some(s) => s.to_string(),None => "?".into() }).collect();
            let verdict = if owners.is_empty() {
                "NO facet on those corners at all — nothing to drop; a zero-area GAP between three \
                 triangles, one boundary edge each"
            } else if owners.len() == 1 && edge_uses.iter().all(|&n| n == 1) {
                "LONE and droppable"
            } else { "a facet with neighbours — dropping would tear the mesh" };
            eprintln!("  loop {k:>2}: {} facet(s) on its corners {:?}, area {:?}, sheet {:?}, \
                edge uses {edge_uses:?} — {verdict}",owners.len(),owners,areas,sheets);
        }
    });
    eprintln!("   one degenerate facet with all three edges used once ⇒ a lone zero-volume flap, \
        and dropping it closes the loop without adding surface");
    eprintln!("   NO facet on the corners ⇒ a zero-area gap between three triangles, and the fix \
        is upstream: split the edge that spans the third vertex");
}

/// Whether a second split pass closes the degenerate loops. The six 3-vertex loops bound **no
/// facet at all** — zero-area gaps between three triangles, one boundary edge each, whose three
/// vertices are collinear with one projecting *inside* the opposite edge
/// (`what_the_degenerate_three_loops_are_made_of`: fractions +0.3333, +0.6603, +0.2731, +0.3408,
/// +0.4342). That is a T-junction, which `split_at_vertices` exists to resolve.
///
/// And one structural fact makes it plausible that it simply never got the chance:
/// **`rim_zip` calls `split_at_vertices` exactly once, before its 32-round loop, never inside it.**
/// The rounds re-walk the boundary every time precisely because the passes change it — the note
/// there says so — but the split runs once. So a T-junction the zip's own laying creates cannot be
/// resolved, which fits loops 17 to 20 being built from zip-created vertices (ids 3921 to 5312).
///
/// The experiment is cheap and decisive: take the finished mesh, run `split_where` over it with the
/// junction tolerance, and re-walk the boundary. Splits made and the 3-loops gone means the fix is
/// to run the split inside the round loop and count it in the "laid nothing" break. No splits, or
/// the loops surviving, and something else refuses them and this is not the answer either.
///
/// The mesh is copied through its public fields rather than assuming `KeptMesh: Clone`. Nothing in
/// the library is changed here; it only measures.
#[test]
#[ignore]
fn whether_a_second_split_pass_closes_the_degenerate_loops() {
    use gcs_core::solid::MaterialField;
    use gcs_core::solid::swept_boundary::{FieldJudge,KeptMesh,Span,Stage,SweptBoundaryOptions,
        boundary_loops,construct,loop_span,rim_zip,split_where,unpinch};
    use std::collections::BTreeMap;
    let source = tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        let Stage::Zipped {mesh,unpaired,..} = stage else { return };
        let mut sizes: Vec<usize> = unpaired.iter().map(|l| l.len()).collect();
        sizes.sort_unstable();
        eprintln!("== the finished mesh: {} triangles, {} unpaired loops, sizes {sizes:?}",
            mesh.triangles.len(),unpaired.len());
        let mut m = KeptMesh {vertices:mesh.vertices.clone(),triangles:mesh.triangles.clone(),
            sheet:mesh.sheet.clone()};
        let splits = split_where(&mut m,options.junction(),&|_,_| true,&|_| true);
        let after: Vec<Vec<u32>> = boundary_loops(&m.triangles).into_iter().flat_map(unpinch).collect();
        let mut after_sizes: Vec<usize> = after.iter().map(|l| l.len()).collect();
        after_sizes.sort_unstable();
        eprintln!("== a second split pass at the junction tolerance {:.4}: {splits} splits, \
            triangles {} -> {}, loops {} -> {}",options.junction(),
            mesh.triangles.len(),m.triangles.len(),unpaired.len(),after.len());
        eprintln!("   sizes after {after_sizes:?}");
        let threes = after.iter().filter(|l| l.len() == 3).count();
        eprintln!("   {threes} three-vertex loops remain (6 before)");
        // Which ones survive is the whole question: the gate fails on the two the field calls
        // `Spanned`, so a split that takes four others and leaves those two improves the count and
        // leaves the test red. Reported by position, the only handle that survives renumbering.
        eprintln!("   the survivors, and what the field says of each:");
        let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
        let mut judge = FieldJudge::new(field,options.judge_tolerance(),4000,1000,4096);
        let (epsilon,reach) = (options.vertex_tolerance(),options.reach());
        let mut owner: BTreeMap<(u32,u32),usize> = Default::default();
        for (i,t) in m.triangles.iter().enumerate() { for k in 0..3 { owner.insert((t[k],t[(k+1)%3]),i); } }
        let mut still_spanned = 0;
        for (k,l) in after.iter().enumerate() {
            if l.len() != 3 { continue; }
            let n = l.len();
            let points: Vec<V3> = l.iter().map(|&v| m.vertices[v as usize]).collect();
            let normals: Vec<V3> = (0..n).map(|j| {
                let (a,b) = (l[j],l[(j+1)%n]);
                owner.get(&(a,b)).and_then(|&t| {
                    let [p,q,r] = m.triangles[t].map(|v| m.vertices[v as usize]);
                    gcs_core::space::stable_normal(p,q,r)
                }).unwrap_or([0.;3])
            }).collect();
            let verdict = match loop_span(&mut judge,&points,&normals,epsilon,reach) {
                Ok(Span::Spanned) => { still_spanned += 1; "SPANNED — the field calls it a hole".to_string() }
                Ok(Span::Open {..}) => "Open".to_string(),
                Ok(Span::Unresolved {..}) => "Unresolved".to_string(),
                Err(err) => format!("{err:?}"),
            };
            eprintln!("     loop {k:>2} at {:?}: {verdict}",points[0].map(|x| (x*1e5).round()/1e5));
        }
        eprintln!("   {still_spanned} of the survivors are still `Spanned` — the gate passes only at 0");
        // And the real candidate fix alternates split with zip, since each changes what the other
        // can take. So: run the zip's own rounds again over the split mesh and see what is left.
        // the tolerances `construct_from` actually passes: `within` is `spacing`, not the crease
        // merge — 0.5 against 0.03, a sixteenfold difference in the pairing's reach
        let (pairs,left) = rim_zip(&mut m,options.spacing,options.junction(),
            options.least_probe(),&mut |points: &[V3],normals: &[V3]|
                matches!(loop_span(&mut judge,points,normals,epsilon,reach),Ok(Span::Spanned)));
        let mut left_sizes: Vec<usize> = left.iter().map(|l| l.len()).collect();
        left_sizes.sort_unstable();
        eprintln!("== then a full `rim_zip` over the split mesh: {pairs} paired, {} loops left, \
            sizes {left_sizes:?}, triangles now {}",left.len(),m.triangles.len());
        // What this experiment showed on the finished mesh — 4 splits, 21 loops to 17, 0 `Spanned`
        // — is what `rim_zip` now does, once after its rounds. Putting the same split *inside* the
        // round was tried and is worse (23 loops, volume 9.2581, still 1 `Spanned`), because it
        // interleaves with the zip and changes what the zip then pairs.
        eprintln!("   this is the placement that landed: one split AFTER the rounds, not inside \
            them — inside gave 23 loops and 9.2581 against 17 and 9.2929");
    });
}
