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

/// Drop every triangle whose centroid the field reads material or exterior
/// by more than twice `sagitta` (an inscribed triangle's centroid is inside
/// by up to a sagitta and is boundary; facets cut at the sagitta sit on it). A triangle's three vertices can all lie on
/// the boundary while its interior does not: a box's front face at the
/// start of its translation has its corners on the sweep's side faces and
/// its interior millimetres inside the sweep. Returns, per triangle, whether
/// it stays, and how many were dropped inside and outside.
pub fn centroid_kept(judge: &mut super::judge::FieldJudge,mesh: &KeptMesh,sagitta: f64) -> Result<(Vec<bool>,usize,usize),super::judge::JudgeError> {
    use super::judge::Sign;
    let mut keep = vec![true;mesh.triangles.len()];
    let (mut inside,mut outside) = (0,0);
    for (i,t) in mesh.triangles.iter().enumerate() {
        let [a,b,c] = t.map(|v| mesh.vertices[v as usize]);
        let centroid: V3 = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
        match judge.deep_sign(centroid,2.*sagitta)?.0 {
            Sign::Material => { keep[i] = false; inside += 1; }
            Sign::Exterior => { keep[i] = false; outside += 1; }
            _ => {}
        }
    }
    Ok((keep,inside,outside))
}

/// Whether the triangle `t` is covered by `tiles` lying on the same surface:
/// the tiles facing its way (within about twenty-five degrees) whose planes
/// pass within `tolerance` of its centroid are projected onto its plane and
/// clipped from it, and it is covered when nothing is left. Tiles of one
/// tessellation share their edges, so their footprints leave no cracks
/// between them; but the triangle's own corners lie on the surface, beyond
/// the chords of a coarser tessellation, and land a whisker outside their
/// footprints (the tile's sagitta times the sine of the angle between the
/// two normals), so every footprint is grown by an eighth of the tolerance.
/// A tile that merely touches the triangle's edge takes nothing from it.
pub fn covered_by(t: [V3;3],tiles: impl Iterator<Item = [V3;3]>,tolerance: f64) -> bool {
    use super::certify::triangle_normal;
    use super::planar::{P2,area2,clip,overlap};
    let Some(n) = triangle_normal(t[0],t[1],t[2]) else { return true };
    let least = (0..3).min_by(|&a,&b| n[a].abs().total_cmp(&n[b].abs())).unwrap();
    let mut axis = [0.;3]; axis[least] = 1.;
    let u = { let c = [n[1]*axis[2]-n[2]*axis[1],n[2]*axis[0]-n[0]*axis[2],n[0]*axis[1]-n[1]*axis[0]]; let l = (c[0]*c[0]+c[1]*c[1]+c[2]*c[2]).sqrt(); c.map(|x| x/l) };
    let v = [n[1]*u[2]-n[2]*u[1],n[2]*u[0]-n[0]*u[2],n[0]*u[1]-n[1]*u[0]];
    let origin = t[0];
    let to2 = |p: V3| -> P2 { let r = [p[0]-origin[0],p[1]-origin[1],p[2]-origin[2]]; (None,r[0]*u[0]+r[1]*u[1]+r[2]*u[2],r[0]*v[0]+r[1]*v[1]+r[2]*v[2]) };
    let mut own: Vec<P2> = t.iter().map(|p| to2(*p)).collect();
    if area2(&own) < 0. { own.reverse(); }
    let scale = own.iter().map(|p| p.1.abs().max(p.2.abs())).fold(0_f64,f64::max).max(f64::MIN_POSITIVE);
    let (eps,area_eps) = (1e-9*scale,1e-14*scale*scale);
    let centroid: V3 = std::array::from_fn(|k| (t[0][k]+t[1][k]+t[2][k])/3.);
    let mut pieces = vec![own];
    for tile in tiles {
        let Some(m) = triangle_normal(tile[0],tile[1],tile[2]) else { continue };
        if m[0]*n[0]+m[1]*n[1]+m[2]*n[2] < 0.9 { continue; }
        if ((centroid[0]-tile[0][0])*m[0]+(centroid[1]-tile[0][1])*m[1]+(centroid[2]-tile[0][2])*m[2]).abs() > tolerance { continue; }
        let mut footprint: Vec<P2> = tile.iter().map(|p| to2(*p)).collect();
        if area2(&footprint) < 0. { footprint.reverse(); }
        if area2(&footprint) <= area_eps { continue; }
        let mut next = Vec::new();
        for piece in pieces { if overlap(&piece,&footprint,eps) { clip(piece,&footprint,eps,tolerance/8.,area_eps,&mut next); } else { next.push(piece); } }
        pieces = next;
        if pieces.is_empty() { return true; }
    }
    false
}

/// Kept triangles of a later sheet that earlier sheets cover are dropped:
/// two sources can generate one piece of the boundary (a tool's leading
/// and trailing edges sweeping the same face of the sweep, the two rims of
/// a plunged cylinder sweeping one wall, a grazing face of the tool at an
/// end pose on the sheet its own edges sweep), and the field, rightly, keeps
/// both. Covered is `covered_by` over every earlier sheet's triangles within
/// the tolerance of the triangle's box; nearness alone would eat a cap's
/// facets beside the seam where its sheet is tangent to them, and a planar
/// sliver beside a perpendicular wall. Returns, per triangle, whether it
/// stays.
pub fn without_overlaps(mesh: &KeptMesh,tolerance: f64) -> Vec<bool> {
    let cell = (tolerance*8.).max(f64::MIN_POSITIVE);
    let key = |p: V3| p.map(|x| (x/cell).floor() as i64);
    // triangles by sheet, bucketed by their boxes
    let mut buckets: std::collections::BTreeMap<(u32,[i64;3]),Vec<usize>> = Default::default();
    let boxes: Vec<([i64;3],[i64;3])> = mesh.triangles.iter().map(|t| {
        let corners = t.map(|v| mesh.vertices[v as usize]);
        (key(std::array::from_fn(|k| corners.iter().map(|q| q[k]).fold(f64::INFINITY,f64::min))),
            key(std::array::from_fn(|k| corners.iter().map(|q| q[k]).fold(f64::NEG_INFINITY,f64::max))))
    }).collect();
    for (i,(lo,hi)) in boxes.iter().enumerate() {
        for x in lo[0]..=hi[0] { for y in lo[1]..=hi[1] { for z in lo[2]..=hi[2] { buckets.entry((mesh.sheet[i],[x,y,z])).or_default().push(i); } } }
    }
    let sheets: std::collections::BTreeSet<u32> = mesh.sheet.iter().copied().collect();
    let mut keep = vec![true;mesh.triangles.len()];
    for (i,t) in mesh.triangles.iter().enumerate() {
        let mine = mesh.sheet[i];
        let (lo,hi) = boxes[i];
        let mut seen: std::collections::BTreeSet<usize> = Default::default();
        let mut tiles: Vec<[V3;3]> = Vec::new();
        for &earlier in sheets.iter().filter(|&&s| s < mine) {
            for x in lo[0]-1..=hi[0]+1 { for y in lo[1]-1..=hi[1]+1 { for z in lo[2]-1..=hi[2]+1 {
                let Some(list) = buckets.get(&(earlier,[x,y,z])) else { continue };
                for &j in list { if seen.insert(j) { tiles.push(mesh.triangles[j].map(|w| mesh.vertices[w as usize])); } }
            } } }
        }
        if !tiles.is_empty() && covered_by(t.map(|v| mesh.vertices[v as usize]),tiles.into_iter(),tolerance) { keep[i] = false; }
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
