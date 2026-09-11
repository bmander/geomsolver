//! Closing the pieces into one shell: coincident vertices of different
//! pieces welded, and every remaining boundary loop zipped to the loop that
//! lies alongside it (a cap's ragged rim to a sheet's end column, a dying
//! column to the column born beside it). A zip joins two loops of judged
//! boundary points, so its band lies along the boundary between them; the
//! certificate still probes every one of its triangles.
use super::trim::{KeptMesh,boundary_loops};
use crate::solid::zip_polylines;

type V3 = [f64;3];

fn distance(a: V3,b: V3) -> f64 { ((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt() }

/// Identify vertices within `tolerance` of one another and drop the
/// triangles that collapse. The first vertex of a group keeps its position.
pub fn weld(mesh: &mut KeptMesh,tolerance: f64) {
    let cell = tolerance.max(f64::MIN_POSITIVE)*4.;
    let key = |p: V3| p.map(|x| (x/cell).floor() as i64);
    let mut cells: std::collections::BTreeMap<[i64;3],Vec<u32>> = Default::default();
    let mut remap: Vec<u32> = Vec::with_capacity(mesh.vertices.len());
    let mut kept: Vec<V3> = Vec::new();
    for p in &mesh.vertices {
        let k = key(*p);
        let mut found = None;
        'search: for x in k[0]-1..=k[0]+1 { for y in k[1]-1..=k[1]+1 { for z in k[2]-1..=k[2]+1 {
            if let Some(list) = cells.get(&[x,y,z]) { for &i in list { if distance(kept[i as usize],*p) <= tolerance { found = Some(i); break 'search; } } }
        } } }
        let i = match found { Some(i) => i,None => { kept.push(*p); let i = (kept.len()-1) as u32; cells.entry(k).or_default().push(i); i } };
        remap.push(i);
    }
    mesh.vertices = kept;
    let mut triangles = Vec::with_capacity(mesh.triangles.len());
    let mut sheet = Vec::with_capacity(mesh.sheet.len());
    for (t,s) in mesh.triangles.iter().zip(&mesh.sheet) {
        let t = t.map(|v| remap[v as usize]);
        if t[0] == t[1] || t[1] == t[2] || t[2] == t[0] { continue; }
        triangles.push(t); sheet.push(*s);
    }
    mesh.triangles = triangles; mesh.sheet = sheet;
}

/// Merge every pair of vertices closer than `shortest`, nearest first,
/// each into the one of the two with more triangles on it, refusing a merge that would
/// turn any triangle round either over or flatten one that does not vanish
/// with the pair. Two samplings of one crease, a
/// fragment's corner beside its neighbour's, a cut's crossing a whisker
/// from a column point, a cap's rim vertex beside a plane's corner: all
/// land within a sagitta of each other, joined by an edge or not, and
/// nothing thinner than a sagitta is a feature of the boundary. Returns
/// how many pairs were merged.
pub fn collapse_short_edges(mesh: &mut KeptMesh,shortest: f64) -> usize {
    use super::certify::triangle_normal;
    let mut collapsed = 0;
    let cell = shortest.max(f64::MIN_POSITIVE);
    let key = |p: V3| p.map(|x| (x/cell).floor() as i64);
    loop {
        // the vertices in use, and every pair of them within the distance
        let mut used = vec![false;mesh.vertices.len()];
        for t in &mesh.triangles { for &v in t { used[v as usize] = true; } }
        let mut cells: std::collections::BTreeMap<[i64;3],Vec<u32>> = Default::default();
        for (v,p) in mesh.vertices.iter().enumerate() { if used[v] { cells.entry(key(*p)).or_default().push(v as u32); } }
        let mut pairs: Vec<(f64,u32,u32)> = Vec::new();
        for (k,list) in &cells {
            for x in k[0]-1..=k[0]+1 { for y in k[1]-1..=k[1]+1 { for z in k[2]-1..=k[2]+1 {
                let Some(other) = cells.get(&[x,y,z]) else { continue };
                for &a in list { for &b in other {
                    if b <= a { continue; }
                    let l = distance(mesh.vertices[a as usize],mesh.vertices[b as usize]);
                    if l < shortest { pairs.push((l,a,b)); }
                } }
            } } }
        }
        if pairs.is_empty() { break; }
        pairs.sort_by(|p,q| p.0.total_cmp(&q.0).then(p.1.cmp(&q.1)).then(p.2.cmp(&q.2)));
        pairs.dedup();
        // Which triangles each vertex is on, at the round's start. A merge relabels only its
        // absorbed vertex's triangles, and a vertex a merge has touched takes no further part
        // in the round, so an untouched vertex's triangles are still these (the degenerate
        // ones a merge makes go only at the round's end, and are counted as before).
        let mut on: Vec<Vec<usize>> = vec![Vec::new();mesh.vertices.len()];
        for (t,tri) in mesh.triangles.iter().enumerate() {
            for k in 0..3 { if !tri[..k].contains(&tri[k]) { on[tri[k] as usize].push(t); } }
        }
        let mut done_any = false;
        let mut touched: std::collections::BTreeSet<u32> = Default::default();
        for (_,a,b) in pairs {
            // one merge per vertex per round, so the distances stay true
            if touched.contains(&a) || touched.contains(&b) { continue; }
            // the pair meets at the vertex with more triangles on it, the
            // more constrained of the two: a polyhedral case keeps its
            // exact corners, and a crease point moves onto the column
            let count = |v: u32| on[v as usize].len();
            let (a,b) = if count(b) > count(a) { (b,a) } else { (a,b) };
            let mid: V3 = mesh.vertices[a as usize];
            let mut ok = true;
            for &i in on[a as usize].iter().chain(&on[b as usize]) {
                let t = &mesh.triangles[i];
                let on_a = t.contains(&a); let on_b = t.contains(&b);
                if !(on_a || on_b) || (on_a && on_b) { continue; }
                let before = t.map(|v| mesh.vertices[v as usize]);
                let after = t.map(|v| if v == a || v == b { mid } else { mesh.vertices[v as usize] });
                match (triangle_normal(before[0],before[1],before[2]),triangle_normal(after[0],after[1],after[2])) {
                    (Some(n0),Some(n1)) if n0[0]*n1[0]+n0[1]*n1[1]+n0[2]*n1[2] > 0.5 => {}
                    _ => { ok = false; break; }
                }
            }
            if !ok { continue; }
            mesh.vertices[a as usize] = mid;
            for &i in &on[b as usize] { for v in mesh.triangles[i].iter_mut() { if *v == b { *v = a; } } }
            touched.insert(a); touched.insert(b);
            collapsed += 1; done_any = true;
        }
        let mut triangles = Vec::with_capacity(mesh.triangles.len());
        let mut sheet = Vec::with_capacity(mesh.sheet.len());
        for (t,s) in mesh.triangles.iter().zip(&mesh.sheet) { if t[0] != t[1] && t[1] != t[2] && t[2] != t[0] { triangles.push(*t); sheet.push(*s); } }
        mesh.triangles = triangles; mesh.sheet = sheet;
        dedupe(mesh);
        if !done_any { break; }
    }
    collapsed
}

/// The farthest any vertex of one loop is from the other loop's polyline.
fn apart(a: &[V3],b: &[V3]) -> f64 {
    let segment = |p: V3,x: V3,y: V3| -> f64 {
        let d = [y[0]-x[0],y[1]-x[1],y[2]-x[2]]; let w = [p[0]-x[0],p[1]-x[1],p[2]-x[2]];
        let l = d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
        let f = if l > 0. { ((w[0]*d[0]+w[1]*d[1]+w[2]*d[2])/l).clamp(0.,1.) } else { 0. };
        distance(p,[x[0]+f*d[0],x[1]+f*d[1],x[2]+f*d[2]])
    };
    let to = |from: &[V3],onto: &[V3]| from.iter().map(|p| (0..onto.len()).map(|k| segment(*p,onto[k],onto[(k+1)%onto.len()])).fold(f64::INFINITY,f64::min)).fold(0_f64,f64::max);
    to(a,b).max(to(b,a))
}

/// Fill every boundary loop of three or four vertices (a hole the size of
/// a triangle, never a seam), zip every other loop to the nearest other
/// loop within `within` of it (both ways), each pair once, the new
/// triangles wound against the edges they close; then every loop left that
/// doubles back on itself (a cap's ragged rim walked out and the sheet's
/// column walked back, joined at their ends) is zipped across, arc to arc. Two runs along one curve
/// sampled twice (a cap's rim on the tool's edge and the sheet's column on
/// the same edge) are first made one by `split_at_vertices` within `split`,
/// so no band is ever laid between them. Returns the pairs zipped and the
/// loops left unpaired.
pub fn rim_zip(mesh: &mut KeptMesh,within: f64,split: f64) -> (usize,Vec<Vec<u32>>) {
    split_at_vertices(mesh,split);
    let loops = boundary_loops(&mesh.triangles);
    // the directed boundary edges as the mesh walks them
    let mut boundary: std::collections::BTreeSet<(u32,u32)> = Default::default();
    for l in &loops { for k in 0..l.len() { boundary.insert((l[k],l[(k+1)%l.len()])); } }
    let points: Vec<Vec<V3>> = loops.iter().map(|l| l.iter().map(|&v| mesh.vertices[v as usize]).collect()).collect();
    let mut paired = vec![false;loops.len()];
    let mut pairs = 0;
    let lay = |mesh: &mut KeptMesh,_on_a: &[u32],band: Vec<[u32;3]>| {
        // every band triangle has an edge along a loop, and a seam must walk
        // it against the way the mesh already does; each triangle is turned
        // by its own loop edge, since a zip of arcs winds each arc for itself
        for mut tri in band {
            let flip = (0..3).find_map(|k| {
                let (p,q) = (tri[k],tri[(k+1)%3]);
                if boundary.contains(&(p,q)) { Some(true) } else if boundary.contains(&(q,p)) { Some(false) } else { None }
            }).unwrap_or(false);
            if flip { tri.swap(1,2); }
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] { continue; }
            // a zero-area triangle (rim vertices collinear with the column
            // along a straight edge) closes nothing
            let [p,q,r] = tri.map(|v| mesh.vertices[v as usize]);
            let (u,w) = ([q[0]-p[0],q[1]-p[1],q[2]-p[2]],[r[0]-p[0],r[1]-p[1],r[2]-p[2]]);
            let area2 = { let c = [u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]]; (c[0]*c[0]+c[1]*c[1]+c[2]*c[2]).sqrt() };
            if area2 <= 1e-12*(distance(p,q)*distance(p,r)).max(f64::MIN_POSITIVE) { continue; }
            mesh.triangles.push(tri); mesh.sheet.push(u32::MAX);
        }
    };
    // a loop of three or four vertices is a hole the size of a triangle,
    // never a seam: it is filled outright, fanned against its own walk
    for i in 0..loops.len() {
        if paired[i] || loops[i].len() < 3 || loops[i].len() > 4 { continue; }
        let l = &loops[i];
        let fan: Vec<[u32;3]> = (1..l.len()-1).map(|k| [l[0],l[k+1],l[k]]).collect();
        paired[i] = true; pairs += 1;
        lay(mesh,l,fan);
    }
    for i in 0..loops.len() {
        if paired[i] || loops[i].len() < 2 { continue; }
        let best = (0..loops.len()).filter(|&j| j != i && !paired[j] && loops[j].len() >= 2)
            .map(|j| (apart(&points[i],&points[j]),j)).filter(|(d,_)| *d <= within).min_by(|a,b| a.0.total_cmp(&b.0));
        let Some((_,j)) = best else { continue };
        paired[i] = true; paired[j] = true; pairs += 1;
        let band = zip_loops(&loops[i],&loops[j],&mesh.vertices);
        lay(mesh,&loops[i],band);
    }
    // slits: a loop turning back on itself at exactly two corners, its two
    // arcs alongside each other
    for i in 0..loops.len() {
        if paired[i] || loops[i].len() < 4 { continue; }
        let (l,p) = (&loops[i],&points[i]);
        let n = l.len();
        let corners: Vec<usize> = (0..n).filter(|&k| {
            let (a,b,c) = (p[(k+n-1)%n],p[k],p[(k+1)%n]);
            let (u,w) = ([b[0]-a[0],b[1]-a[1],b[2]-a[2]],[c[0]-b[0],c[1]-b[1],c[2]-b[2]]);
            let (lu,lw) = (distance(a,b),distance(b,c));
            lu > 0. && lw > 0. && (u[0]*w[0]+u[1]*w[1]+u[2]*w[2])/(lu*lw) < -0.5
        }).collect();
        if corners.len() != 2 { continue; }
        let (c0,c1) = (corners[0],corners[1]);
        let arc_a: Vec<u32> = (c0..=c1).map(|k| l[k]).collect();
        let arc_b: Vec<u32> = (c1..=c0+n).map(|k| l[k%n]).rev().collect();
        let pts = |arc: &[u32]| arc.iter().map(|&v| mesh.vertices[v as usize]).collect::<Vec<_>>();
        let (pa,pb) = (pts(&arc_a),pts(&arc_b));
        if apart(&pa,&pb) > within { continue; }
        let band: Vec<[u32;3]> = zip_polylines(&pa,&pb,false).into_iter().map(|t| t.map(|(on_b,k)| if on_b { arc_b[k as usize] } else { arc_a[k as usize] })).collect();
        paired[i] = true; pairs += 1;
        lay(mesh,&arc_a,band);
    }
    let unpaired: Vec<Vec<u32>> = loops.into_iter().zip(paired).filter(|(_,p)| !p).map(|(l,_)| l).collect();
    dedupe(mesh);
    (pairs,unpaired)
}

/// The triangles zipping two boundary loops of one mesh, wound as a strip
/// from the first loop to the second walked the first loop's way. The two
/// loops of a seam between outward pieces run opposite ways, so the second
/// is reversed before the strip zip pairs points along it; zipped as given
/// the rungs cross and the band dips deep into the material. Loops that
/// share no vertex are zipped closed; loops that share vertices (a
/// tessellation vertex on the contact curve, a fan meeting a face on a tool
/// edge) are zipped arc by arc between the shared vertices, each arc pair
/// open and each arc of the second taken start to end like the first's, so a
/// shared vertex is never made a rung's end and no loop edge is doubled.
pub fn zip_loops(a: &[u32],b: &[u32],vertices: &[V3]) -> Vec<[u32;3]> {
    let pts = |l: &[u32]| l.iter().map(|&v| vertices[v as usize]).collect::<Vec<_>>();
    let shared: Vec<usize> = (0..a.len()).filter(|&i| b.contains(&a[i])).collect();
    if shared.is_empty() {
        // the second loop the way whose rungs are shortest: two loops of one
        // seam run opposite ways when both pieces face outward, and zipped
        // the wrong way round the rungs cross and the band dips into the material
        let rungs = |b: &[u32]| -> (f64,Vec<[u32;3]>) {
            let tris: Vec<[u32;3]> = zip_polylines(&pts(a),&pts(b),true).into_iter().map(|t| t.map(|(on_b,k)| if on_b { b[k as usize] } else { a[k as usize] })).collect();
            let cost: f64 = tris.iter().map(|t| { let [p,q,r] = t.map(|v| vertices[v as usize]); distance(p,q)+distance(q,r)+distance(r,p) }).sum();
            (cost,tris)
        };
        let reversed: Vec<u32> = b.iter().rev().copied().collect();
        let (forward,backward) = (rungs(b),rungs(&reversed));
        return if backward.0 <= forward.0 { backward.1 } else { forward.1 };
    }
    // a from its first shared vertex, cut at every shared vertex; each arc
    // is paired with whichever of b's two arcs between the same vertices
    // lies alongside it
    let a: Vec<u32> = a[shared[0]..].iter().chain(&a[..shared[0]]).copied().collect();
    let shared_a: Vec<usize> = (0..a.len()).filter(|&i| b.contains(&a[i])).collect();
    let nb = b.len();
    let along = |i0: usize,i1: usize,forward: bool| -> Vec<u32> {
        // from b[i0] to b[i1] stepping forward or backward round the loop,
        // the whole loop when the two coincide
        let mut arc = vec![b[i0]];
        let mut i = i0;
        loop {
            i = if forward { (i+1)%nb } else { (i+nb-1)%nb };
            arc.push(b[i]);
            if i == i1 { break; }
        }
        arc
    };
    let mut out = Vec::new();
    let cuts_a: Vec<usize> = shared_a.iter().copied().chain(std::iter::once(a.len())).collect();
    for w in cuts_a.windows(2) {
        let arc_a: Vec<u32> = (w[0]..=w[1]).map(|i| a[i%a.len()]).collect();
        let (start,end) = (arc_a[0],arc_a[arc_a.len()-1]);
        let (i0,i1) = (b.iter().position(|&x| x == start).unwrap(),b.iter().position(|&x| x == end).unwrap());
        let (forward,backward) = (along(i0,i1,true),along(i0,i1,false));
        let pa = pts(&arc_a);
        let arc_b = if apart(&pa,&pts(&forward)) <= apart(&pa,&pts(&backward)) { forward } else { backward };
        for t in zip_polylines(&pa,&pts(&arc_b),false) {
            out.push(t.map(|(on_b,k)| if on_b { arc_b[k as usize] } else { arc_a[k as usize] }));
        }
    }
    out
}

/// Resolve the T-junctions between boundary edges: a boundary vertex lying
/// within `tolerance` of the interior of a boundary edge it does not end
/// splits the triangle on that edge in two at the vertex (which stays where
/// it is), so two runs along one curve sampled twice (a cap's rim on the
/// tool's edge and a sheet's column on the same edge, a planar fragment's
/// corner on its neighbour's edge) come to share every vertex and every edge
/// between them, where a band between them would be zero-area or a fold.
/// With the tolerance a sagitta or so, either sampling's chords hold the
/// other's vertices.
pub fn split_at_vertices(mesh: &mut KeptMesh,tolerance: f64) { split_where(mesh,tolerance,&|_,_| true,&|_| true); }

/// `split_at_vertices` over the boundary edges `edge_ok` admits, at the
/// boundary vertices `vertex_ok` admits. Returns how many splits were made.
pub fn split_where(mesh: &mut KeptMesh,tolerance: f64,edge_ok: &dyn Fn(u32,u32) -> bool,vertex_ok: &dyn Fn(u32) -> bool) -> usize {
    let mut made = 0;
    for _ in 0..64 {
        let loops = boundary_loops(&mesh.triangles);
        let mut owner: std::collections::BTreeMap<(u32,u32),usize> = Default::default();
        for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { owner.insert((t[k],t[(k+1)%3]),i); } }
        let boundary_vertices: Vec<u32> = loops.iter().flatten().copied().filter(|&v| vertex_ok(v)).collect();
        let mut incident: std::collections::BTreeMap<u32,Vec<usize>> = Default::default();
        for (i,t) in mesh.triangles.iter().enumerate() { for &v in t { incident.entry(v).or_default().push(i); } }
        let mut split: Vec<(usize,u32,u32,u32)> = Vec::new(); // triangle, edge a, edge b, vertex
        let mut used: std::collections::BTreeSet<usize> = Default::default();
        for l in &loops { for k in 0..l.len() {
            let (a,b) = (l[k],l[(k+1)%l.len()]);
            if !edge_ok(a,b) { continue; }
            let Some(&t) = owner.get(&(a,b)) else { continue };
            if used.contains(&t) { continue; }
            let own = mesh.triangles[t];
            let (pa,pb) = (mesh.vertices[a as usize],mesh.vertices[b as usize]);
            let d = [pb[0]-pa[0],pb[1]-pa[1],pb[2]-pa[2]];
            let l2 = d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
            if l2 <= 0. { continue; }
            // the triangle's height over this edge: a vertex farther off the
            // edge than a quarter of it would fold the split (a pole's fan of
            // slivers, each vertex a whisker from its neighbours' edges)
            let height = {
                let c = own.iter().copied().find(|&v| v != a && v != b).map(|v| mesh.vertices[v as usize]).unwrap_or(pa);
                let w = [c[0]-pa[0],c[1]-pa[1],c[2]-pa[2]];
                let f = (w[0]*d[0]+w[1]*d[1]+w[2]*d[2])/l2;
                distance(c,[pa[0]+f*d[0],pa[1]+f*d[1],pa[2]+f*d[2]])
            };
            // the vertex nearest the middle of the edge, of another loop
            let mut best: Option<(f64,u32)> = None;
            for &v in &boundary_vertices {
                if own.contains(&v) { continue; }
                let p = mesh.vertices[v as usize];
                let w = [p[0]-pa[0],p[1]-pa[1],p[2]-pa[2]];
                let f = (w[0]*d[0]+w[1]*d[1]+w[2]*d[2])/l2;
                if f <= 0. || f >= 1. { continue; }
                let foot = [pa[0]+f*d[0],pa[1]+f*d[1],pa[2]+f*d[2]];
                let off = distance(p,foot);
                // a vertex at an end is that end (welded already, or as
                // near as makes no triangle); one merely near it splits
                // and it must lie along the edge, not beside an end: a
                // vertex a whisker from the edge's start but as far from
                // its line folds the split's first triangle
                let along = f.min(1.-f)*l2.sqrt();
                if off <= tolerance && off <= height/4. && off <= along/4. && best.map_or(true,|(g,_)| (f-0.5).abs() < (g-0.5).abs()) { best = Some((f,v)); }
            }
            // the split must leave two triangles facing the way the one did,
            // along edges no triangle already walks that way
            if let Some((_,v)) = best {
                if owner.contains_key(&(a,v)) || owner.contains_key(&(v,b)) { continue; }
                let c = own.iter().copied().find(|&x| x != a && x != b).unwrap_or(a);
                let (pc,pv) = (mesh.vertices[c as usize],mesh.vertices[v as usize]);
                let normal = |p: V3,q: V3,r: V3| -> V3 { let (u,w) = ([q[0]-p[0],q[1]-p[1],q[2]-p[2]],[r[0]-p[0],r[1]-p[1],r[2]-p[2]]); [u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]] };
                let (n0,n1,n2) = (normal(pa,pb,pc),normal(pa,pv,pc),normal(pv,pb,pc));
                let dot = |x: V3,y: V3| x[0]*y[0]+x[1]*y[1]+x[2]*y[2];
                let least = 1e-6*dot(n0,n0).sqrt();
                if !(dot(n0,n1) > least*dot(n1,n1).sqrt() && dot(n0,n2) > least*dot(n2,n2).sqrt() && dot(n1,n1).sqrt() > least && dot(n2,n2).sqrt() > least) { continue; }
                // a vertex beside the edge with a triangle of its own in
                // the edge's plane, between it and the edge: the split
                // would lay a new triangle over that one (two sources'
                // fragments of one plane, a vertex of one a whisker past
                // the other's edge, whose own sliver reaches back to it)
                if lies_over(mesh,&incident,v,[a,v,c],[v,b,c],n0) { continue; }
                split.push((t,a,b,v)); used.insert(t);
            }
        } }
        if split.is_empty() { break; }
        made += split.len();
        for (t,a,b,v) in split {
            let tri = mesh.triangles[t];
            let k = (0..3).find(|&k| tri[k] == a && tri[(k+1)%3] == b).unwrap();
            let c = tri[(k+2)%3];
            mesh.triangles[t] = [a,v,c];
            mesh.triangles.push([v,b,c]);
            let s = mesh.sheet[t]; mesh.sheet.push(s);
        }
        // two splits of one round can make one triangle twice
        dedupe(mesh);
    }
    made
}

/// Whether either of two triangles to be made at `v` overlaps, in area, a
/// triangle already on `v` that lies in their plane (normal `n`, not unit)
/// facing their way.
fn lies_over(mesh: &KeptMesh,incident: &std::collections::BTreeMap<u32,Vec<usize>>,v: u32,first: [u32;3],second: [u32;3],n: V3) -> bool {
    use super::planar::{P2,area2,overlap};
    let len = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
    if !(len > 0.) { return false; }
    let n = n.map(|x| x/len);
    let least = (0..3).min_by(|&a,&b| n[a].abs().total_cmp(&n[b].abs())).unwrap();
    let mut axis = [0.;3]; axis[least] = 1.;
    let u = { let c = [n[1]*axis[2]-n[2]*axis[1],n[2]*axis[0]-n[0]*axis[2],n[0]*axis[1]-n[1]*axis[0]]; let l = (c[0]*c[0]+c[1]*c[1]+c[2]*c[2]).sqrt(); c.map(|x| x/l) };
    let w = [n[1]*u[2]-n[2]*u[1],n[2]*u[0]-n[0]*u[2],n[0]*u[1]-n[1]*u[0]];
    let origin = mesh.vertices[v as usize];
    let to2 = |p: V3| -> P2 { let r = [p[0]-origin[0],p[1]-origin[1],p[2]-origin[2]]; (None,r[0]*u[0]+r[1]*u[1]+r[2]*u[2],r[0]*w[0]+r[1]*w[1]+r[2]*w[2]) };
    let polygon = |t: [u32;3]| -> Vec<P2> { let mut p: Vec<P2> = t.iter().map(|&x| to2(mesh.vertices[x as usize])).collect(); if area2(&p) < 0. { p.reverse(); } p };
    let scale = [first,second].iter().flatten().map(|&x| distance(mesh.vertices[x as usize],origin)).fold(0_f64,f64::max).max(f64::MIN_POSITIVE);
    let eps = 1e-9*scale;
    let news = [polygon(first),polygon(second)];
    let Some(list) = incident.get(&v) else { return false };
    for &i in list {
        let t = mesh.triangles[i];
        let [p,q,r] = t.map(|x| mesh.vertices[x as usize]);
        let Some(m) = super::certify::triangle_normal(p,q,r) else { continue };
        if m[0]*n[0]+m[1]*n[1]+m[2]*n[2] < 0.999 { continue; }
        if [p,q,r].iter().any(|x| ((x[0]-origin[0])*n[0]+(x[1]-origin[1])*n[1]+(x[2]-origin[2])*n[2]).abs() > 1e-6*scale) { continue; }
        let old = polygon(t);
        if news.iter().any(|new| overlap(new,&old,eps)) { return true; }
    }
    false
}

/// Drop every sliver (no altitude above `thin`) that walks an edge the way
/// another triangle already does: a piece of one source lying along a
/// tool edge over another source's region, which the merge of their
/// vertices has laid on the same edges. Returns how many went.
pub fn drop_doubled_slivers(mesh: &mut KeptMesh,thin: f64) -> usize {
    use super::certify::triangle_normal;
    let mut owners: std::collections::BTreeMap<(u32,u32),Vec<usize>> = Default::default();
    for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { owners.entry((t[k],t[(k+1)%3])).or_default().push(i); } }
    let altitude = |t: &[u32;3]| -> f64 {
        let [a,b,c] = t.map(|v| mesh.vertices[v as usize]);
        let Some(_) = triangle_normal(a,b,c) else { return 0. };
        let (u,w) = ([b[0]-a[0],b[1]-a[1],b[2]-a[2]],[c[0]-a[0],c[1]-a[1],c[2]-a[2]]);
        let cr = [u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]];
        let area2 = (cr[0]*cr[0]+cr[1]*cr[1]+cr[2]*cr[2]).sqrt();
        let longest = distance(a,b).max(distance(b,c)).max(distance(c,a));
        if longest > 0. { area2/longest } else { 0. }
    };
    let doomed: std::collections::BTreeSet<usize> = owners.values().filter(|v| v.len() > 1).flatten().copied().filter(|&i| altitude(&mesh.triangles[i]) < thin).collect();
    let mut triangles = Vec::with_capacity(mesh.triangles.len());
    let mut sheet = Vec::with_capacity(mesh.sheet.len());
    for (i,(t,s)) in mesh.triangles.iter().zip(&mesh.sheet).enumerate() { if !doomed.contains(&i) { triangles.push(*t); sheet.push(*s); } }
    mesh.triangles = triangles; mesh.sheet = sheet;
    doomed.len()
}

/// Drop every triangle that repeats another's three vertices.
pub fn dedupe(mesh: &mut KeptMesh) -> usize {
    let mut seen: std::collections::BTreeSet<[u32;3]> = Default::default();
    let mut triangles = Vec::with_capacity(mesh.triangles.len());
    let mut sheet = Vec::with_capacity(mesh.sheet.len());
    let mut dropped = 0;
    for (t,s) in mesh.triangles.iter().zip(&mesh.sheet) {
        let mut key = *t; key.sort();
        if seen.insert(key) { triangles.push(*t); sheet.push(*s); } else { dropped += 1; }
    }
    mesh.triangles = triangles; mesh.sheet = sheet;
    dropped
}
