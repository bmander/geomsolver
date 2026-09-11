//! What the labels leave of the seed sheets: the triangles all of whose
//! vertices the field kept or moved, wound outward, and the boundary loops
//! of that mesh, which the closure audit reads. (Bisecting mixed edges to
//! rim points and building creases comes with milestone 4.)
use crate::space::Grid;
use crate::space::stable_normal;
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
            let Some(n) = stable_normal(a,b,c) else { continue };
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
/// `probe` and `least` are the certificate's probe distance and least distance.
pub fn centroid_kept(judge: &mut super::judge::FieldJudge,mesh: &KeptMesh,probe: f64,least: f64) -> Result<(Vec<bool>,usize,usize),super::judge::JudgeError> {
    use super::judge::Sign;
    let mut keep = vec![true;mesh.triangles.len()];
    let (mut inside,mut outside) = (0,0);
    for (i,t) in mesh.triangles.iter().enumerate() {
        let [a,b,c] = t.map(|v| mesh.vertices[v as usize]);
        let centroid: V3 = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
        // Material a probe distance inside and exterior as far outside, along the triangle's own
        // outward normal (the certificate's first probes, which the judge keeps for it): the
        // field being one-Lipschitz, the centroid is then neither material nor exterior deeper
        // than that, which is all the deep sign could have said against the triangle. Asked
        // only of a triangle the certificate will probe so: one with an altitude of at least
        // its least distance (a sliver it judges at the centroid).
        if let Some(n) = stable_normal(a,b,c) {
            if crate::space::altitude(a,b,c) >= least && judge.sides(centroid,n,probe)? == (Sign::Material,Sign::Exterior) { continue; }
        }
        match judge.deep_sign(centroid,probe)?.0 {
            Sign::Material => { keep[i] = false; inside += 1; }
            Sign::Exterior => { keep[i] = false; outside += 1; }
            _ => {}
        }
    }
    Ok((keep,inside,outside))
}

/// What a covering sheet leaves of the triangle `t`: `tiles` are the
/// sheet's triangles near it and `outline` the sheet's boundary edges near
/// it (each with the tile on it). The tiles facing its way (within about
/// twenty-five degrees) whose planes pass within `tolerance` of it where
/// the two overlap (at the middle of the tile's footprint on it: a coarse
/// chord of a curved surface is a sagitta off it at its own middle and
/// further at its ends, and the overlap with a neighbour is at an end)
/// are the sheet where it lies on this one; the triangle is cut,
/// in its own plane, along every outline edge of such a tile that crosses
/// it, and of the pieces those whose centroid some such tile's footprint
/// holds are covered and go. `None` when no tile was tangent to it (it
/// stays as it is), otherwise the convex pieces left, possibly none. The
/// pieces' new corners are put onto the outline's edges themselves (they
/// lie on their lines in this plane, within the two sheets' distance of
/// them), so the seam is the covering sheet's own edges, for the stitch's
/// split to share; no tile is grown and nothing is cut where no edge runs.
pub fn uncovered(t: [V3;3],tiles: &[[V3;3]],outline: &[([V3;3],V3,V3)],tolerance: f64) -> Option<Vec<[V3;3]>> {
    use crate::space::stable_normal;
    use super::planar::{P2,area2,cut};
    let Some(n) = stable_normal(t[0],t[1],t[2]) else { return Some(Vec::new()) };
    let least = (0..3).min_by(|&a,&b| n[a].abs().total_cmp(&n[b].abs())).unwrap();
    let mut axis = [0.;3]; axis[least] = 1.;
    let u = { let c = [n[1]*axis[2]-n[2]*axis[1],n[2]*axis[0]-n[0]*axis[2],n[0]*axis[1]-n[1]*axis[0]]; let l = (c[0]*c[0]+c[1]*c[1]+c[2]*c[2]).sqrt(); c.map(|x| x/l) };
    let v = [n[1]*u[2]-n[2]*u[1],n[2]*u[0]-n[0]*u[2],n[0]*u[1]-n[1]*u[0]];
    let origin = t[0];
    let to2 = |p: V3| -> P2 { let r = [p[0]-origin[0],p[1]-origin[1],p[2]-origin[2]]; (None,r[0]*u[0]+r[1]*u[1]+r[2]*u[2],r[0]*v[0]+r[1]*v[1]+r[2]*v[2]) };
    let mut own: Vec<P2> = t.iter().enumerate().map(|(k,p)| { let q = to2(*p); (Some(k as u32),q.1,q.2) }).collect();
    if area2(&own) < 0. { own.reverse(); }
    let scale = own.iter().map(|p| p.1.abs().max(p.2.abs())).fold(0_f64,f64::max).max(f64::MIN_POSITIVE);
    // Where two sheets' edges coincide they do so to the tracer's noise, about 1e-12 at unit
    // coordinates, whatever the triangle's size: a width decided below that (a millionth of a
    // millionth of a sliver) is decided by rounding. A billionth of the coordinates is well
    // above it, and an area is that width along the triangle.
    let magnitude = t.iter().flatten().fold(0_f64,|m,x| m.max(x.abs()));
    let eps = 1e-9*scale.max(magnitude);
    let area_eps = eps*scale;
    let tangent = |tile: &[V3;3]| -> bool {
        let Some(m) = stable_normal(tile[0],tile[1],tile[2]) else { return false };
        if m[0]*n[0]+m[1]*n[1]+m[2]*n[2] < 0.9 { return false; }
        // the tile's footprint on this plane, clipped to the triangle
        let mut f: Vec<P2> = tile.iter().map(|p| to2(*p)).collect();
        if area2(&f) < 0. { f.reverse(); }
        if area2(&f) <= area_eps { return false; }
        let mut rest = f;
        for k in 0..3 {
            let (a,b) = (own[k],own[(k+1)%3]);
            let (ex,ey) = (b.1-a.1,b.2-a.2);
            let len = (ex*ex+ey*ey).sqrt();
            if !(len > 0.) { continue; }
            let d: Vec<f64> = rest.iter().map(|v| ((v.1-a.1)*(-ey)+(v.2-a.2)*ex)/len).collect();
            let (inside,_) = cut(&rest,&d,eps);
            if inside.len() < 3 || area2(&inside) <= area_eps { return false; }
            rest = inside;
        }
        let c = (rest.iter().map(|p| p.1).sum::<f64>()/rest.len() as f64,rest.iter().map(|p| p.2).sum::<f64>()/rest.len() as f64);
        let at: V3 = std::array::from_fn(|k| origin[k]+c.0*u[k]+c.1*v[k]);
        ((at[0]-tile[0][0])*m[0]+(at[1]-tile[0][1])*m[1]+(at[2]-tile[0][2])*m[2]).abs() <= tolerance
    };
    let footprints: Vec<Vec<P2>> = tiles.iter().filter(|tile| tangent(tile)).map(|tile| {
        let mut f: Vec<P2> = tile.iter().map(|p| to2(*p)).collect();
        if area2(&f) < 0. { f.reverse(); }
        f
    }).filter(|f| area2(f) > area_eps).collect();
    if footprints.is_empty() { return None; }
    // cut along every outline edge of a tangent tile that crosses the triangle
    let mut pieces = vec![own.clone()];
    for (tile,a,b) in outline {
        if !tangent(tile) { continue; }
        let (pa,pb) = (to2(*a),to2(*b));
        let (ex,ey) = (pb.1-pa.1,pb.2-pa.2);
        let len = (ex*ex+ey*ey).sqrt();
        if !(len > 0.) { continue; }
        let (nx,ny) = (-ey/len,ex/len);
        // does the segment cross the triangle: some corner on each side of
        // its line, and the crossing within the segment's own span
        let d: Vec<f64> = pieces.iter().flatten().map(|p| (p.1-pa.1)*nx+(p.2-pa.2)*ny).collect();
        if !(d.iter().any(|x| *x > eps) && d.iter().any(|x| *x < -eps)) { continue; }
        let along: Vec<f64> = pieces.iter().flatten().map(|p| ((p.1-pa.1)*ex+(p.2-pa.2)*ey)/len).collect();
        if along.iter().all(|s| *s < -eps) || along.iter().all(|s| *s > len+eps) { continue; }
        let mut next = Vec::new();
        for piece in pieces {
            let d: Vec<f64> = piece.iter().map(|p| (p.1-pa.1)*nx+(p.2-pa.2)*ny).collect();
            let (left,right) = cut(&piece,&d,eps);
            for side in [left,right] { if side.len() >= 3 && area2(&side) > area_eps { next.push(side); } }
        }
        pieces = next;
    }
    // a piece whose centroid a footprint holds is covered
    let inside = |c: (f64,f64),f: &Vec<P2>| -> bool {
        (0..f.len()).all(|k| { let (a,b) = (f[k],f[(k+1)%f.len()]); (b.1-a.1)*(c.1-a.2)-(b.2-a.2)*(c.0-a.1) >= -eps })
    };
    // a corner the cuts made lies on an outline edge's line in this plane;
    // it is put onto the edge itself, so the seam is that edge exactly
    let lift = |p: &P2| -> V3 {
        let q: V3 = std::array::from_fn(|k| origin[k]+p.1*u[k]+p.2*v[k]);
        if p.0.is_some() { return q; }
        let mut best: Option<(f64,V3)> = None;
        for (tile,a,b) in outline {
            if !tangent(tile) { continue; }
            let d = [b[0]-a[0],b[1]-a[1],b[2]-a[2]]; let w = [q[0]-a[0],q[1]-a[1],q[2]-a[2]];
            let l = d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
            let f = if l > 0. { ((w[0]*d[0]+w[1]*d[1]+w[2]*d[2])/l).clamp(0.,1.) } else { 0. };
            let on = [a[0]+f*d[0],a[1]+f*d[1],a[2]+f*d[2]];
            let dist = (0..3).map(|k| (q[k]-on[k]).powi(2)).sum::<f64>().sqrt();
            if dist <= tolerance && best.map_or(true,|(x,_)| dist < x) { best = Some((dist,on)); }
        }
        best.map_or(q,|(_,on)| on)
    };
    let mut out = Vec::new();
    let mut left_any = false;
    for piece in pieces {
        let c = (piece.iter().map(|p| p.1).sum::<f64>()/piece.len() as f64,piece.iter().map(|p| p.2).sum::<f64>()/piece.len() as f64);
        if footprints.iter().any(|f| inside(c,f)) { continue; }
        left_any = true;
        // the corners lifted, and a corner the cuts made put within an
        // eighth of the tolerance of another corner is that corner; the
        // triangle's own corners are never merged (a sliver of the sheet is
        // still the sheet), an own corner taking the place of a made one
        let mut lifted: Vec<(V3,bool)> = Vec::new();
        for p in &piece {
            let q = lift(p);
            let own = p.0.is_some();
            let near = lifted.iter().position(|(r,_)| (0..3).map(|k| (r[k]-q[k]).powi(2)).sum::<f64>().sqrt() <= tolerance/8.);
            match near {
                Some(k) if !lifted[k].1 => { if own { lifted[k] = (q,true); } }
                Some(_) if !own => {}
                _ => lifted.push((q,own)),
            }
        }
        let lifted: Vec<V3> = lifted.into_iter().map(|(q,_)| q).collect();
        for k in 1..lifted.len().saturating_sub(1) {
            let (a,b,c) = (lifted[0],lifted[k],lifted[k+1]);
            // a piece turned over or stood up by the snap was a sliver
            // thinner than the snap, and goes; the hole it leaves is a
            // triangle's, filled later
            let Some(m) = stable_normal(a,b,c) else { continue };
            if m[0]*n[0]+m[1]*n[1]+m[2]*n[2] < 0.5 { continue; }
            out.push([a,b,c]);
        }
    }
    let _ = left_any;
    Some(out)
}

/// Whether the triangle `t` is covered by the sheet of `tiles` with the
/// boundary `outline`: `uncovered` leaves nothing of it.
pub fn covered_by(t: [V3;3],tiles: &[[V3;3]],outline: &[([V3;3],V3,V3)],tolerance: f64) -> bool {
    uncovered(t,tiles,outline,tolerance).is_some_and(|left| left.is_empty())
}

/// Kept triangles of a later sheet that earlier sheets cover are clipped to
/// what they leave: two sources can generate one piece of the boundary (a
/// tool's leading and trailing edges sweeping the same face of the sweep,
/// the two rims of a plunged cylinder sweeping one wall, a grazing face of
/// the tool at an end pose on the sheet its own edges sweep, the two corner
/// paths of a turning prism's caps sweeping one surface of revolution over
/// overlapping arcs), and the field, rightly, keeps both. Each earlier
/// sheet in turn is applied through `uncovered`, with its triangles and its
/// own boundary edges near the triangle: a triangle wholly covered goes,
/// one partly covered is replaced by the pieces left, in its own plane.
/// Nearness alone would eat a cap's facets beside the seam where its sheet
/// is tangent to them, and a planar sliver beside a perpendicular wall.
/// Returns the mesh and how many triangles were clipped or dropped.
pub fn clip_overlaps(mesh: &KeptMesh,tolerance: f64) -> (KeptMesh,usize) {
    let cell = (tolerance*8.).max(f64::MIN_POSITIVE);
    let box_of = |corners: &[V3]| -> (V3,V3) {
        (std::array::from_fn(|k| corners.iter().map(|q| q[k]).fold(f64::INFINITY,f64::min)),
            std::array::from_fn(|k| corners.iter().map(|q| q[k]).fold(f64::NEG_INFINITY,f64::max)))
    };
    // each sheet's triangles and boundary edges, filed by their boxes (an edge by its place in
    // `edges`)
    let mut tile_grids: std::collections::BTreeMap<u32,Grid> = Default::default();
    let mut edge_grids: std::collections::BTreeMap<u32,Grid> = Default::default();
    let mut edges: Vec<(usize,u32,u32)> = Vec::new();
    let mut uses: std::collections::BTreeMap<(u32,u32,u32),(usize,usize)> = Default::default(); // (sheet, a, b) -> (count, a triangle)
    for (i,t) in mesh.triangles.iter().enumerate() {
        let corners = t.map(|v| mesh.vertices[v as usize]);
        let (lo,hi) = box_of(&corners);
        tile_grids.entry(mesh.sheet[i]).or_insert_with(|| Grid::new(cell)).insert_box(lo,hi,i as u32);
        for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); let e = uses.entry((mesh.sheet[i],a.min(b),a.max(b))).or_insert((0,i)); e.0 += 1; }
    }
    for (&(s,a,b),&(count,i)) in &uses {
        if count != 1 { continue; }
        let (lo,hi) = box_of(&[mesh.vertices[a as usize],mesh.vertices[b as usize]]);
        edge_grids.entry(s).or_insert_with(|| Grid::new(cell)).insert_box(lo,hi,edges.len() as u32);
        edges.push((i,a,b));
    }
    let pack = |grids: std::collections::BTreeMap<u32,Grid>| -> std::collections::BTreeMap<u32,crate::space::PackedGrid> { grids.into_iter().map(|(s,g)| (s,g.pack())).collect() };
    let (tile_grids,edge_grids) = (pack(tile_grids),pack(edge_grids));
    let sheets: std::collections::BTreeSet<u32> = mesh.sheet.iter().copied().collect();
    let mut out = KeptMesh {vertices:mesh.vertices.clone(),triangles:Vec::new(),sheet:Vec::new()};
    let mut changed = 0;
    for (i,t) in mesh.triangles.iter().enumerate() {
        let mine = mesh.sheet[i];
        let corners = t.map(|v| mesh.vertices[v as usize]);
        // the triangle's pieces so far, applied to by each earlier sheet in turn
        let mut pieces: Vec<[V3;3]> = vec![corners];
        let mut touched = false;
        for &earlier in sheets.iter().filter(|&&s| s < mine) {
            let mut next = Vec::new();
            for piece in pieces {
                let (lo,hi) = box_of(&piece);
                let mut seen: std::collections::BTreeSet<u32> = Default::default();
                let mut tiles: Vec<[V3;3]> = Vec::new();
                let mut outline: Vec<([V3;3],V3,V3)> = Vec::new();
                let mut seen_edges: std::collections::BTreeSet<(u32,u32)> = Default::default();
                // the cells of the piece's box and one more round it
                if let Some(grid) = tile_grids.get(&earlier) {
                    grid.in_keys(grid.key(lo).map(|x| x-1),grid.key(hi).map(|x| x+1),|j| if seen.insert(j) { tiles.push(mesh.triangles[j as usize].map(|w| mesh.vertices[w as usize])); });
                }
                if let Some(grid) = edge_grids.get(&earlier) {
                    grid.in_keys(grid.key(lo).map(|x| x-1),grid.key(hi).map(|x| x+1),|e| {
                        let (j,a,b) = edges[e as usize];
                        if seen_edges.insert((a,b)) { outline.push((mesh.triangles[j].map(|w| mesh.vertices[w as usize]),mesh.vertices[a as usize],mesh.vertices[b as usize])); }
                    });
                }
                match if tiles.is_empty() { None } else { uncovered(piece,&tiles,&outline,tolerance) } {
                    None => next.push(piece),
                    Some(left) => { touched = true; next.extend(left); }
                }
            }
            pieces = next;
        }
        if !touched { out.triangles.push(*t); out.sheet.push(mine); continue; }
        changed += 1;
        for tri in pieces {
            let idx: [u32;3] = std::array::from_fn(|k| {
                // a piece's corner at the triangle's own corner is that corner
                if let Some(&v) = t.iter().find(|&&v| { let q = mesh.vertices[v as usize]; (0..3).map(|d| (q[d]-tri[k][d]).powi(2)).sum::<f64>().sqrt() <= 1e-9*(1.+q.iter().map(|x| x.abs()).fold(0.,f64::max)) }) { v }
                else { out.vertices.push(tri[k]); (out.vertices.len()-1) as u32 }
            });
            if idx[0] == idx[1] || idx[1] == idx[2] || idx[2] == idx[0] { continue; }
            out.triangles.push(idx); out.sheet.push(mine);
        }
    }
    (out,changed)
}

/// `clip_overlaps` as a keep list: whether each triangle stays whole. A
/// triangle clipped or dropped reads as not kept.
pub fn without_overlaps(mesh: &KeptMesh,tolerance: f64) -> Vec<bool> {
    let (out,_) = clip_overlaps(mesh,tolerance);
    let kept: std::collections::BTreeSet<[u32;3]> = out.triangles.iter().copied().collect();
    mesh.triangles.iter().map(|t| kept.contains(t)).collect()
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
    let edges = super::adjacency::Edges::new(triangles);
    walk_loops(triangles,edges.boundary(),|a,b| edges.uses(a,b) == 1,|a,b| edges.owner(a,b))
}

/// The loops of `boundary_loops` walked from the boundary's directed edges (`pending`), with
/// whether an edge is boundary and which triangle (the last by index) walks a directed edge.
pub(crate) fn walk_loops(triangles: &[[u32;3]],mut pending: std::collections::BTreeSet<(u32,u32)>,boundary: impl Fn(u32,u32) -> bool,owner: impl Fn(u32,u32) -> Option<usize>) -> Vec<Vec<u32>> {
    // the boundary edge leaving b after arriving along a -> b: rotate round
    // b's fan from the triangle on a -> b until an edge out of b is boundary
    let next_after = |a: u32,b: u32| -> Option<u32> {
        let mut t = owner(a,b)?;
        for _ in 0..triangles.len()+1 {
            let tri = triangles[t];
            let k = (0..3).find(|&k| tri[k] == b)?;
            let c = tri[(k+1)%3];
            if boundary(b,c) { return Some(c); }
            // across edge b -> c to the triangle that walks c -> b
            t = owner(c,b)?;
        }
        None
    };
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
