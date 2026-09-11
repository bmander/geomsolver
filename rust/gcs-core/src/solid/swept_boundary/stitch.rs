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

/// Zip every boundary loop to the nearest other loop within `within` of it
/// (both ways), each pair once, the new triangles wound against the edges
/// they close. Two loops within `coincident` of each other are one curve
/// sampled twice (a cap's rim on the tool's sharp edge, and the sheet's end
/// column on the same edge): a band between them would double the surface
/// beside it, so the band of triangles at the second loop is peeled off and
/// the loop it leaves is zipped instead. Returns the pairs zipped and the
/// loops left unpaired.
pub fn rim_zip(mesh: &mut KeptMesh,within: f64,coincident: f64,vertex: f64) -> (usize,Vec<Vec<u32>>) {
    // a T-junction is exact coincidence (two runs along one tool edge), a
    // thousandth of the vertex tolerance; a vertex merely near another loop's
    // chord is the coincident-rim case below, not a junction
    split_at_vertices(mesh,vertex*1e-3);
    let mut loops = boundary_loops(&mesh.triangles);
    // the directed boundary edges as the mesh walks them
    let mut boundary: std::collections::BTreeSet<(u32,u32)> = Default::default();
    for l in &loops { for k in 0..l.len() { boundary.insert((l[k],l[(k+1)%l.len()])); } }
    let mut points: Vec<Vec<V3>> = loops.iter().map(|l| l.iter().map(|&v| mesh.vertices[v as usize]).collect()).collect();
    let mut paired = vec![false;loops.len()];
    let mut pairs = 0;
    for i in 0..loops.len() {
        if paired[i] || loops[i].len() < 2 { continue; }
        let best = (0..loops.len()).filter(|&j| j != i && !paired[j] && loops[j].len() >= 2)
            .map(|j| (apart(&points[i],&points[j]),j)).filter(|(d,_)| *d <= within).min_by(|a,b| a.0.total_cmp(&b.0));
        let Some((d,j)) = best else { continue };
        paired[i] = true; paired[j] = true; pairs += 1;
        // the coincident pair's sparser loop is the one peeled, the denser
        // kept: a sheet's column against a cap's rim on the same edge
        let (i,j) = if d <= coincident && loops[i].len() < loops[j].len() { (j,i) } else { (i,j) };
        if d <= coincident {
            // peel the band at loop j: every triangle of the loop's own sheet
            // on one of its vertices (a vertex the two loops share also
            // carries the other sheet's triangles, which stay)
            let on: std::collections::BTreeSet<u32> = loops[j].iter().copied().collect();
            let mut owner: std::collections::BTreeMap<(u32,u32),usize> = Default::default();
            for (k,t) in mesh.triangles.iter().enumerate() { for e in 0..3 { owner.insert((t[e],t[(e+1)%3]),k); } }
            let mut sheets_of_j: std::collections::BTreeMap<u32,usize> = Default::default();
            for k in 0..loops[j].len() { if let Some(&t) = owner.get(&(loops[j][k],loops[j][(k+1)%loops[j].len()])) { *sheets_of_j.entry(mesh.sheet[t]).or_default() += 1; } }
            let peeled = sheets_of_j.into_iter().max_by_key(|(_,n)| *n).map(|(s,_)| s).unwrap_or(u32::MAX);
            let mut keep = Vec::with_capacity(mesh.triangles.len());
            let mut sheet = Vec::with_capacity(mesh.sheet.len());
            for (t,s) in mesh.triangles.iter().zip(&mesh.sheet) { if !(*s == peeled && t.iter().any(|v| on.contains(v))) { keep.push(*t); sheet.push(*s); } }
            mesh.triangles = keep; mesh.sheet = sheet;
            // the loop the peel leaves: the new boundary loop nearest loop i
            let fresh = boundary_loops(&mesh.triangles);
            let known: std::collections::BTreeSet<Vec<u32>> = loops.iter().enumerate().filter(|(k,_)| *k != j).map(|(_,l)| l.clone()).collect();
            let candidates: Vec<Vec<u32>> = fresh.into_iter().filter(|l| !known.contains(l)).collect();
            let Some(next) = candidates.into_iter().map(|l| { let p: Vec<V3> = l.iter().map(|&v| mesh.vertices[v as usize]).collect(); (apart(&points[i],&p),l) })
                .min_by(|a,b| a.0.total_cmp(&b.0)).map(|(_,l)| l) else { paired[j] = false; continue };
            for k in 0..next.len() { boundary.insert((next[k],next[(k+1)%next.len()])); }
            points[j] = next.iter().map(|&v| mesh.vertices[v as usize]).collect();
            loops[j] = next;
        }
        let band = zip_loops(&loops[i],&loops[j],&mesh.vertices);
        // the band is one strip, so one of its loop edges says whether it
        // walks the mesh's edges the way they already run, which a seam
        // must not
        let on_a: std::collections::BTreeSet<u32> = loops[i].iter().copied().collect();
        let flip = band.iter().find_map(|t| (0..3).find_map(|k| {
            let (p,q) = (t[k],t[(k+1)%3]);
            if on_a.contains(&p) && on_a.contains(&q) { Some(boundary.contains(&(p,q))) } else { None }
        })).unwrap_or(false);
        for mut tri in band {
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
    }
    let unpaired: Vec<Vec<u32>> = loops.into_iter().zip(paired).filter(|(_,p)| !p).map(|(l,_)| l).collect();
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
/// within `tolerance` (the vertex tolerance: this is exact coincidence, not
/// proximity) of the interior of a boundary edge it does not end splits the
/// triangle on that edge in two at the vertex, so two runs along one line (a
/// translated box's trailing column and its leading band's cut, both on the
/// tool's edge, walked out along one and back along the other) come to
/// share exact edges there, where a band between them would be zero-area.
pub fn split_at_vertices(mesh: &mut KeptMesh,tolerance: f64) {
    for _ in 0..8 {
        let loops = boundary_loops(&mesh.triangles);
        let mut owner: std::collections::BTreeMap<(u32,u32),usize> = Default::default();
        for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { owner.insert((t[k],t[(k+1)%3]),i); } }
        let boundary_vertices: Vec<u32> = loops.iter().flatten().copied().collect();
        let mut split: Vec<(usize,u32,u32,u32)> = Vec::new(); // triangle, edge a, edge b, vertex
        let mut used: std::collections::BTreeSet<usize> = Default::default();
        for l in &loops { for k in 0..l.len() {
            let (a,b) = (l[k],l[(k+1)%l.len()]);
            let Some(&t) = owner.get(&(a,b)) else { continue };
            if used.contains(&t) { continue; }
            let (pa,pb) = (mesh.vertices[a as usize],mesh.vertices[b as usize]);
            let d = [pb[0]-pa[0],pb[1]-pa[1],pb[2]-pa[2]];
            let l2 = d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
            if l2 <= 0. { continue; }
            // the vertex nearest the middle of the edge, of another loop
            let mut best: Option<(f64,u32)> = None;
            for &v in &boundary_vertices {
                if v == a || v == b { continue; }
                let p = mesh.vertices[v as usize];
                let w = [p[0]-pa[0],p[1]-pa[1],p[2]-pa[2]];
                let f = (w[0]*d[0]+w[1]*d[1]+w[2]*d[2])/l2;
                if f <= 0. || f >= 1. { continue; }
                let foot = [pa[0]+f*d[0],pa[1]+f*d[1],pa[2]+f*d[2]];
                let off = distance(p,foot);
                if off <= tolerance && distance(p,pa) > tolerance && distance(p,pb) > tolerance && best.map_or(true,|(g,_)| (f-0.5).abs() < (g-0.5).abs()) { best = Some((f,v)); }
            }
            if let Some((_,v)) = best { split.push((t,a,b,v)); used.insert(t); }
        } }
        if split.is_empty() { break; }
        for (t,a,b,v) in split {
            let tri = mesh.triangles[t];
            let k = (0..3).find(|&k| tri[k] == a && tri[(k+1)%3] == b).unwrap();
            let c = tri[(k+2)%3];
            mesh.triangles[t] = [a,v,c];
            mesh.triangles.push([v,b,c]);
            let s = mesh.sheet[t]; mesh.sheet.push(s);
        }
    }
}
