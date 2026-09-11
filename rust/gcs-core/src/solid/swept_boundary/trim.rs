//! What the labels leave of the seed sheets: the triangles all of whose
//! vertices the field kept or moved, wound outward, and the boundary loops
//! of that mesh, which the closure audit reads. (Bisecting mixed edges to
//! rim points and building creases comes with milestone 4.)
use super::certify::triangle_normal;
use super::project::{Label,Labelled};
use crate::solid::SweepPatch;

type V3 = [f64;3];

/// An indexed mesh with, per triangle, the sheet it came from.
#[derive(Clone,Debug,Default)]
pub struct KeptMesh {
    pub vertices: Vec<V3>,
    pub triangles: Vec<[u32;3]>,
    pub sheet: Vec<u32>,
}

/// The kept triangles of every sheet, each wound so that its normal agrees
/// with the direction its vertices were judged along (outward). Vertices are
/// the judged positions; sheets keep separate vertices here (welding is the
/// stitch's).
pub fn kept_triangles(sheets: &[SweepPatch],labelled: &[Labelled]) -> KeptMesh {
    let mut out = KeptMesh::default();
    for (s,(sheet,l)) in sheets.iter().zip(labelled).enumerate() {
        let base = out.vertices.len() as u32;
        out.vertices.extend_from_slice(&l.points);
        for t in &sheet.triangles {
            if !t.iter().all(|&v| matches!(l.labels[v as usize],Label::Kept | Label::Moved)) { continue; }
            let [a,b,c] = t.map(|v| l.points[v as usize]);
            let Some(n) = triangle_normal(a,b,c) else { continue };
            let outward: V3 = std::array::from_fn(|k| t.iter().map(|&v| l.directions[v as usize][k]).sum());
            let agree = n[0]*outward[0]+n[1]*outward[1]+n[2]*outward[2] >= 0.;
            out.triangles.push(if agree { [base+t[0],base+t[1],base+t[2]] } else { [base+t[0],base+t[2],base+t[1]] });
            out.sheet.push(s as u32);
        }
    }
    out
}

/// Kept triangles of a later sheet lying on an earlier sheet's kept surface
/// are dropped: two sources can generate one piece of the boundary (a tool's
/// leading and trailing edges sweeping the same face of the sweep, the two
/// rims of a plunged cylinder sweeping one wall), and the field, rightly,
/// keeps both. A triangle is on another sheet where its three vertices and
/// its centroid each lie within `tolerance` of that sheet's kept triangles.
/// Returns, per triangle of the mesh, whether it stays.
pub fn without_overlaps(mesh: &KeptMesh,tolerance: f64) -> Vec<bool> {
    use super::certify::triangle_normal;
    let cell = (tolerance*8.).max(f64::MIN_POSITIVE);
    let key = |p: V3| p.map(|x| (x/cell).floor() as i64);
    // triangles by sheet, bucketed by their boxes
    let mut buckets: std::collections::BTreeMap<(u32,[i64;3]),Vec<usize>> = Default::default();
    for (i,t) in mesh.triangles.iter().enumerate() {
        let corners = t.map(|v| mesh.vertices[v as usize]);
        let (lo,hi) = (key(std::array::from_fn(|k| corners.iter().map(|q| q[k]).fold(f64::INFINITY,f64::min))),
            key(std::array::from_fn(|k| corners.iter().map(|q| q[k]).fold(f64::NEG_INFINITY,f64::max))));
        for x in lo[0]..=hi[0] { for y in lo[1]..=hi[1] { for z in lo[2]..=hi[2] { buckets.entry((mesh.sheet[i],[x,y,z])).or_default().push(i); } } }
    }
    let distance_to = |p: V3,i: usize| -> f64 {
        let [a,b,c] = mesh.triangles[i].map(|v| mesh.vertices[v as usize]);
        // Ericson's closest point on a triangle
        let sub = |x: V3,y: V3| -> V3 { [x[0]-y[0],x[1]-y[1],x[2]-y[2]] };
        let dot = |x: V3,y: V3| x[0]*y[0]+x[1]*y[1]+x[2]*y[2];
        let len = |x: V3| dot(x,x).sqrt();
        let (ab,ac,ap) = (sub(b,a),sub(c,a),sub(p,a));
        let (d1,d2) = (dot(ab,ap),dot(ac,ap));
        if d1 <= 0. && d2 <= 0. { return len(ap); }
        let bp = sub(p,b); let (d3,d4) = (dot(ab,bp),dot(ac,bp));
        if d3 >= 0. && d4 <= d3 { return len(bp); }
        let vc = d1*d4-d3*d2;
        if vc <= 0. && d1 >= 0. && d3 <= 0. { let v = d1/(d1-d3); return len(sub(p,[a[0]+v*ab[0],a[1]+v*ab[1],a[2]+v*ab[2]])); }
        let cp = sub(p,c); let (d5,d6) = (dot(ab,cp),dot(ac,cp));
        if d6 >= 0. && d5 <= d6 { return len(cp); }
        let vb = d5*d2-d1*d6;
        if vb <= 0. && d2 >= 0. && d6 <= 0. { let w = d2/(d2-d6); return len(sub(p,[a[0]+w*ac[0],a[1]+w*ac[1],a[2]+w*ac[2]])); }
        let va = d3*d6-d5*d4;
        if va <= 0. && d4-d3 >= 0. && d5-d6 >= 0. { let w = (d4-d3)/((d4-d3)+(d5-d6)); return len(sub(p,[b[0]+w*(c[0]-b[0]),b[1]+w*(c[1]-b[1]),b[2]+w*(c[2]-b[2])])); }
        let denom = 1./(va+vb+vc); let (v,w) = (vb*denom,vc*denom);
        len(sub(p,[a[0]+v*ab[0]+w*ac[0],a[1]+v*ab[1]+w*ac[1],a[2]+v*ab[2]+w*ac[2]]))
    };
    let near_sheet = |p: V3,sheet: u32| -> bool {
        let k = key(p);
        for x in k[0]-1..=k[0]+1 { for y in k[1]-1..=k[1]+1 { for z in k[2]-1..=k[2]+1 {
            if let Some(list) = buckets.get(&(sheet,[x,y,z])) { for &i in list { if distance_to(p,i) <= tolerance { return true; } } }
        } } }
        false
    };
    let sheets: std::collections::BTreeSet<u32> = mesh.sheet.iter().copied().collect();
    let mut keep = vec![true;mesh.triangles.len()];
    for (i,t) in mesh.triangles.iter().enumerate() {
        let mine = mesh.sheet[i];
        let corners = t.map(|v| mesh.vertices[v as usize]);
        let centroid: V3 = std::array::from_fn(|k| (corners[0][k]+corners[1][k]+corners[2][k])/3.);
        if triangle_normal(corners[0],corners[1],corners[2]).is_none() { keep[i] = false; continue; }
        for &earlier in sheets.iter().filter(|&&s| s < mine) {
            if corners.iter().chain(std::iter::once(&centroid)).all(|p| near_sheet(*p,earlier)) { keep[i] = false; break; }
        }
    }
    keep
}

impl KeptMesh {
    /// The mesh with its unused vertices dropped and the rest renumbered.
    pub fn compact(&self) -> KeptMesh {
        let mut remap: Vec<u32> = vec![u32::MAX;self.vertices.len()];
        let mut vertices = Vec::new();
        for t in &self.triangles { for &v in t { if remap[v as usize] == u32::MAX { remap[v as usize] = vertices.len() as u32; vertices.push(self.vertices[v as usize]); } } }
        KeptMesh {vertices,triangles:self.triangles.iter().map(|t| t.map(|v| remap[v as usize])).collect(),sheet:self.sheet.clone()}
    }
}

/// The mesh with only the triangles `keep` says.
pub fn retained(mesh: &KeptMesh,keep: &[bool]) -> KeptMesh {
    let mut out = KeptMesh {vertices:mesh.vertices.clone(),triangles:Vec::new(),sheet:Vec::new()};
    for (i,t) in mesh.triangles.iter().enumerate() { if keep[i] { out.triangles.push(*t); out.sheet.push(mesh.sheet[i]); } }
    out
}

/// The boundary of an indexed mesh: its directed edges used by exactly one
/// triangle, chained into loops by walking round each vertex's fan, so two
/// loops that share a vertex (a tessellation vertex on the contact curve
/// where a cap's rim meets a sheet's column) stay two loops rather than
/// crossing over at it. An edge used more than twice is returned as a loop
/// of two, so the audit sees it.
pub fn boundary_loops(triangles: &[[u32;3]]) -> Vec<Vec<u32>> {
    let mut uses: std::collections::BTreeMap<(u32,u32),usize> = Default::default();
    let mut owner: std::collections::BTreeMap<(u32,u32),usize> = Default::default();
    for (i,t) in triangles.iter().enumerate() { for k in 0..3 {
        let (a,b) = (t[k],t[(k+1)%3]);
        *uses.entry((a.min(b),a.max(b))).or_default() += 1;
        owner.insert((a,b),i);
    } }
    let boundary = |a: u32,b: u32| uses.get(&(a.min(b),a.max(b))).copied() == Some(1);
    // the boundary edge leaving b after arriving along a -> b: rotate round
    // b's fan from the triangle on a -> b until an edge out of b is boundary
    let next_after = |a: u32,b: u32| -> Option<u32> {
        let mut t = *owner.get(&(a,b))?;
        for _ in 0..triangles.len()+1 {
            let tri = triangles[t];
            let k = (0..3).find(|&k| tri[k] == b)?;
            let c = tri[(k+1)%3];
            if boundary(b,c) { return Some(c); }
            // across edge b -> c to the triangle that walks c -> b
            t = *owner.get(&(c,b))?;
        }
        None
    };
    let mut pending: std::collections::BTreeSet<(u32,u32)> = owner.keys().copied().filter(|&(a,b)| boundary(a,b)).collect();
    let mut loops = Vec::new();
    while let Some(&(start_a,start_b)) = pending.iter().next() {
        let mut walk = vec![start_a];
        let (mut a,mut b) = (start_a,start_b);
        loop {
            pending.remove(&(a,b));
            if b == start_a { break; }
            walk.push(b);
            match next_after(a,b) {
                Some(c) if pending.contains(&(b,c)) => { a = b; b = c; }
                _ => break,
            }
        }
        loops.push(walk);
    }
    loops
}
