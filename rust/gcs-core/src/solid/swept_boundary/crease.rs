//! Creases: where a sheet turns inner across another, the kept part is
//! clipped at the crossing and the two sheets' clipped rims are made one
//! polyline, so the shell folds along the crease with no band laid across
//! the concave corner (which the certificate would refuse, its chord lying
//! in the air). The crease point on a mixed edge (one end on the boundary,
//! the other inner or off the material) is bisected by the judge along the
//! edge, to the vertex tolerance; the two rims are then two samplings of the
//! same curve within that tolerance, and each is put into the other's last
//! row of triangles in the order of the curve, never by a distance rule.
use super::judge::{FieldJudge,JudgeError,Projection};
use super::project::{Label,Labelled};
use super::trim::KeptMesh;

type V3 = [f64;3];

use crate::space::{dot,lerp,norm,sub};

/// One clipped rim: the sheet it lies on and its vertices in order.
#[derive(Clone,Debug)]
pub struct Rim { pub sheet: usize,pub vertices: Vec<u32>,pub closed: bool }

/// The kept triangles of every sheet, each triangle with an inner corner
/// clipped at the crease points bisected on its mixed edges (the last
/// boundary point found along the edge, within `epsilon`), wound outward.
/// A traced sheet's vertex labels decide; a cap's do not at the vertices on an edge of the
/// tool (`Cap::tool_edges`). Returns the mesh and the rims the clipping made, chained per sheet.
pub fn clip_sheets(judge: &mut FieldJudge,seeds: &[super::Seed],labelled: &[Labelled],epsilon: f64,reach: f64) -> Result<(KeptMesh,Vec<Rim>),JudgeError> {
    use crate::space::stable_normal;
    let mut out = KeptMesh::default();
    let mut rims = Vec::new();
    for (s,(seed,l)) in seeds.iter().zip(labelled).enumerate() {
        let sheet = seed.patch();
        let base = out.vertices.len() as u32;
        out.vertices.extend_from_slice(&l.points);
        let edge = seed.tool_edges().unwrap_or(&[]);
        let per_corner = seed.tool_edges().is_some();
        // A cap's vertex on a tool edge is on the boundary by one face or
        // the other, and its label says nothing about which: the face beside
        // a crease that crosses the edge reads on at both ends of the
        // crossing. So a cap's corners are judged a sagitta in from the
        // corner along the triangle's own plane, where the face itself
        // speaks, and a crease is bisected along the edge the same way in.
        // Inside a face the vertex's own label speaks for the face: a
        // triangle there with every vertex kept is kept whole, unasked, so
        // the corners judged go with the tool's edges and the creases
        // crossing the cap, not with the cap's area.
        let pull = 8.*epsilon;
        let mut corner_on: std::collections::BTreeMap<(usize,usize),bool> = Default::default();
        let faces = super::project::orientation(sheet);
        if per_corner {
            for (i,t) in sheet.triangles.iter().enumerate() {
                if t.iter().all(|&v| !edge[v as usize] && l.labels[v as usize] == Label::Kept) {
                    for k in 0..3 { corner_on.insert((i,k),true); }
                    continue;
                }
                let [a,b,c] = t.map(|v| l.points[v as usize]);
                let Some(mut n) = stable_normal(a,b,c) else { continue };
                if !faces[i] { n = n.map(|x| -x); }
                let centroid: V3 = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
                for (k,p) in [a,b,c].into_iter().enumerate() {
                    let toward = sub(centroid,p);
                    let len = norm(toward);
                    let q = if len > pull { lerp(p,centroid,pull/len) } else { centroid };
                    // on means kept: a corner the judge would move is one
                    // whose bracket straddles the other face at the edge
                    let on = match judge.project(q,n,epsilon,reach) {
                        Ok(p) => matches!(p,Projection::Kept {..}),
                        Err(e) => return Err(e),
                    };
                    corner_on.insert((i,k),on);
                }
            }
        }
        let on = |v: u32| matches!(l.labels[v as usize],Label::Kept | Label::Moved);
        // the crease point of each mixed edge, once
        let mut crease: std::collections::BTreeMap<(u32,u32),u32> = Default::default();
        let mut rim_edges: Vec<(u32,u32)> = Vec::new();
        // the last on point along the edge a -> b, the test points pulled
        // off the edge by `pull` along `d` (within the triangle) and judged
        // along `m`; returns the parameter and the point
        let bisect = |judge: &mut FieldJudge,pa: V3,pb: V3,ma: V3,mb: V3,pull: f64,d: Option<V3>,m: Option<V3>| -> Result<(f64,V3),JudgeError> {
            let length = norm(sub(pb,pa));
            let (mut t_on,mut t_off) = (0_f64,1_f64);
            let mut found = pa;
            while length*(t_off-t_on) > epsilon {
                let mid = 0.5*(t_on+t_off);
                let p0 = lerp(pa,pb,mid);
                let p = match d { Some(d) => [p0[0]+pull*d[0],p0[1]+pull*d[1],p0[2]+pull*d[2]],None => p0 };
                let m = match m { Some(n) => n,None => lerp(ma,mb,mid) };
                let len = norm(m);
                if !(len > 0.) { break; }
                // on the boundary means the sheet's own point brackets it
                // within epsilon; a point the judge would move is one the
                // sheet has already left, onto the other sheet of the crease
                match judge.project(p,m.map(|x| x/len),epsilon,reach)? {
                    Projection::Kept {..} => { t_on = mid; found = p0; }
                    _ => t_off = mid,
                }
            }
            Ok((t_on,found))
        };
        let mut point_on = |judge: &mut FieldJudge,out: &mut KeptMesh,a: u32,b: u32,inward: Option<(V3,V3,f64)>| -> Result<u32,JudgeError> {
            // a on the boundary, b off it
            let key = (a.min(b),a.max(b));
            if let Some(&v) = crease.get(&key) { return Ok(v); }
            let (pa,pb) = (l.points[a as usize],l.points[b as usize]);
            let (ma,mb) = (l.directions[a as usize],l.directions[b as usize]);
            let found = match inward {
                None => bisect(judge,pa,pb,ma,mb,0.,None,None)?.1,
                Some((d,n,height)) => {
                    // A cap's edge may lie on a tool edge, where the face's
                    // own state is invisible on the edge itself; so the test
                    // points are pulled into the triangle, twice, and the
                    // crossing extrapolated to the edge: a crease is straight
                    // over a sagitta, so the crossing moves linearly with the
                    // pull. The pulls stay inside the triangle.
                    let (p1,p2) = ((5.*epsilon).min(height/3.),(3.*epsilon).min(height/6.));
                    let (t1,_) = bisect(judge,pa,pb,ma,mb,p1,Some(d),Some(n))?;
                    let (t2,_) = bisect(judge,pa,pb,ma,mb,p2,Some(d),Some(n))?;
                    let t0 = (t2-(t1-t2)*p2/(p1-p2)).clamp(0.,1.);
                    lerp(pa,pb,t0)
                }
            };
            // a crease point within a sagitta of the on vertex is that
            // vertex: the sheet leaves the boundary at its own edge (a face
            // beside a convex edge reads on the boundary for an epsilon or
            // so, its bracket straddling the other face), and a sliver
            // between them would be a rim to nothing
            let v = if norm(sub(found,pa)) <= 4.*epsilon { base+a } else { out.vertices.push(found); (out.vertices.len()-1) as u32 };
            crease.insert(key,v);
            Ok(v)
        };
        for (i,t) in sheet.triangles.iter().enumerate() {
            let ons: Vec<bool> = (0..3).map(|k| if per_corner { corner_on.get(&(i,k)).copied().unwrap_or(false) } else { on(t[k]) }).collect();
            let count = ons.iter().filter(|x| **x).count();
            if count == 0 { continue; }
            let [a,b,c] = t.map(|v| l.points[v as usize]);
            let Some(n) = stable_normal(a,b,c) else { continue };
            let agree = faces[i];
            let n_out = if agree { n } else { n.map(|x| -x) };
            let centroid: V3 = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
            // the pull for an edge: in the plane, perpendicular to the edge,
            // toward the centroid, at most the triangle's height over it
            let inward = |x: u32,y: u32| -> Option<(V3,V3,f64)> {
                if !per_corner { return None; }
                let (px,py) = (l.points[x as usize],l.points[y as usize]);
                let e = sub(py,px);
                let mid = lerp(px,py,0.5);
                let w = sub(centroid,mid);
                let along = dot(w,e)/dot(e,e).max(f64::MIN_POSITIVE);
                let d = [w[0]-along*e[0],w[1]-along*e[1],w[2]-along*e[2]];
                let len = norm(d);
                (len > 0.).then(|| (d.map(|v| v/len),n_out,3.*len))
            };
            // the triangle's corners in outward order
            let order: [usize;3] = if agree { [0,1,2] } else { [0,2,1] };
            let corners: [u32;3] = order.map(|k| t[k]);
            let on_c: [bool;3] = order.map(|k| ons[k]);
            let mut made: Vec<[u32;3]> = Vec::new();
            match count {
                3 => made.push(corners.map(|v| base+v)),
                1 => {
                    let k = (0..3).find(|&k| on_c[k]).unwrap();
                    let (a,b,c) = (corners[k],corners[(k+1)%3],corners[(k+2)%3]);
                    let (x_ab,x_ca) = (point_on(judge,&mut out,a,b,inward(a,b))?,point_on(judge,&mut out,a,c,inward(a,c))?);
                    made.push([base+a,x_ab,x_ca]);
                    rim_edges.push((x_ab,x_ca));
                }
                _ => {
                    let k = (0..3).find(|&k| !on_c[k]).unwrap();
                    let (c,a,b) = (corners[k],corners[(k+1)%3],corners[(k+2)%3]);
                    let (x_bc,x_ca) = (point_on(judge,&mut out,b,c,inward(b,c))?,point_on(judge,&mut out,a,c,inward(a,c))?);
                    made.push([base+a,base+b,x_bc]);
                    made.push([base+a,x_bc,x_ca]);
                    rim_edges.push((x_bc,x_ca));
                }
            }
            for tri in made {
                if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] { continue; }
                out.triangles.push(tri); out.sheet.push(s as u32);
            }
        }
        // a rim edge between two of the sheet's own vertices is the sheet's
        // own edge: the boundary leaves it there, along an edge its
        // neighbour already shares, and there is nothing to merge
        rim_edges.retain(|(a,b)| a != b && !(*a < base+l.points.len() as u32 && *b < base+l.points.len() as u32));
        for (vertices,closed) in chains(&rim_edges) { rims.push(Rim {sheet:s,vertices,closed}); }
    }
    Ok((out,rims))
}

/// Undirected edges chained into polylines by shared vertices: open chains
/// from each end of degree one, then whatever closed loops remain.
pub fn chains(edges: &[(u32,u32)]) -> Vec<(Vec<u32>,bool)> {
    let mut adjacent: std::collections::BTreeMap<u32,Vec<u32>> = Default::default();
    for &(a,b) in edges { adjacent.entry(a).or_default().push(b); adjacent.entry(b).or_default().push(a); }
    let mut used: std::collections::BTreeSet<(u32,u32)> = Default::default();
    let mut out = Vec::new();
    let walk = |start: u32,used: &mut std::collections::BTreeSet<(u32,u32)>| -> Vec<u32> {
        let mut chain = vec![start];
        let mut at = start;
        loop {
            let Some(next) = adjacent[&at].iter().copied().find(|&n| !used.contains(&(at.min(n),at.max(n)))) else { break };
            used.insert((at.min(next),at.max(next)));
            chain.push(next);
            at = next;
            if at == start { break; }
        }
        chain
    };
    let ends: Vec<u32> = adjacent.iter().filter(|(_,n)| n.len() == 1).map(|(v,_)| *v).collect();
    for start in ends {
        if adjacent[&start].iter().all(|&n| used.contains(&(start.min(n),start.max(n)))) { continue; }
        let chain = walk(start,&mut used);
        if chain.len() >= 2 { out.push((chain,false)); }
    }
    for start in adjacent.keys().copied().collect::<Vec<_>>() {
        if adjacent[&start].iter().all(|&n| used.contains(&(start.min(n),start.max(n)))) { continue; }
        let mut chain = walk(start,&mut used);
        let closed = chain.len() > 2 && chain.first() == chain.last();
        if closed { chain.pop(); }
        if chain.len() >= 2 { out.push((chain,closed)); }
    }
    out
}

/// Make the rims of different sheets one along every crease they share:
/// a rim vertex within `snap` of another sheet's rim vertex is that vertex,
/// and every rim edge is split at the other sheets' rim vertices within
/// `tolerance` of its interior (`split_at_vertices`' rule, kept to rims).
/// Two rims sample one crease within the vertex tolerance each, so where
/// they run together they come to share every vertex and edge; where a rim
/// runs alone (along a sheet's own column, which its neighbour already
/// shares by identity) nothing is done. Returns the vertices welded and the
/// edges split.
/// Identities used by the actual crease merge, before aliasing and after removal.
/// `source_triangles` indexes the pre-alias mesh independently of its sheet array.
pub enum MergeEvent<'a> {
    Aliases { mesh: &'a KeptMesh, aliases: &'a std::collections::BTreeMap<u32,u32> },
    Retained { mesh: &'a KeptMesh, source_triangles: &'a [usize] },
}

pub fn merge_creases(mesh: &mut KeptMesh,rims: &[Rim],tolerance: f64,snap: f64) -> (usize,usize) {
    merge_creases_observed(mesh,rims,tolerance,snap,&mut |_| {})
}

pub fn merge_creases_observed(mesh: &mut KeptMesh,rims: &[Rim],tolerance: f64,snap: f64,
    observe: &mut dyn FnMut(MergeEvent<'_>)) -> (usize,usize) {
    // coincident rim vertices of different sheets are one
    let mut welded = 0;
    let mut alias: std::collections::BTreeMap<u32,u32> = Default::default();
    for i in 0..rims.len() { for j in i+1..rims.len() {
        if rims[i].sheet == rims[j].sheet { continue; }
        for &vb in &rims[j].vertices {
            if alias.contains_key(&vb) { continue; }
            let pb = mesh.vertices[vb as usize];
            if let Some(&va) = rims[i].vertices.iter().find(|&&va| va != vb && norm(sub(mesh.vertices[va as usize],pb)) <= snap) {
                let va = *alias.get(&va).unwrap_or(&va);
                alias.insert(vb,va); welded += 1;
            }
        }
    } }
    observe(MergeEvent::Aliases {mesh,aliases:&alias});
    for t in mesh.triangles.iter_mut() { for v in t.iter_mut() { if let Some(&a) = alias.get(v) { *v = a; } } }
    let mut source_triangles = Vec::new(); let mut source_triangle = 0;
    mesh.triangles.retain(|t| {
        let keep = t[0] != t[1] && t[1] != t[2] && t[2] != t[0];
        if keep { source_triangles.push(source_triangle); }
        source_triangle += 1; keep
    });
    observe(MergeEvent::Retained {mesh,source_triangles:&source_triangles});
    let resolve = |v: u32| *alias.get(&v).unwrap_or(&v);
    let rim_vertices: std::collections::BTreeSet<u32> = rims.iter().flat_map(|r| r.vertices.iter().map(|&v| resolve(v))).collect();
    let mut rim_edges: std::collections::BTreeSet<(u32,u32)> = Default::default();
    for r in rims {
        let n = r.vertices.len();
        let segments = if r.closed { n } else { n.saturating_sub(1) };
        for k in 0..segments { let (a,b) = (resolve(r.vertices[k]),resolve(r.vertices[(k+1)%n])); if a != b { rim_edges.insert((a.min(b),a.max(b))); } }
    }
    let split = super::stitch::split_where(mesh,tolerance,&|a,b| rim_edges.contains(&(a.min(b),a.max(b))),&|v| rim_vertices.contains(&v));
    (welded,split)
}
