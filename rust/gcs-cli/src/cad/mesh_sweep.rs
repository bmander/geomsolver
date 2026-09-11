//! A body with continuous swept cuts as a mesh, by arrangement of candidate
//! sheets and classification by the declared material field.
//!
//! The static remainder of the body is meshed by the core's own facet evaluator
//! and is the blank. Each sweep's candidate sheet (its contact grid) becomes a
//! thin closed slab, offset either side of the sheet along the contact normals;
//! the tool at both end poses is an exact cap solid. Manifold removes the caps
//! from the blank outright (the tool occupies them), then splits the remainder
//! by every slab. Each cell is judged by one material probe a few slab widths
//! inside its largest triangle, and each slab piece by the field at its
//! mid-surface: strictly material means a hidden wall, kept; otherwise it is the
//! skin of an exposed boundary and is dropped. Material cells and kept walls are
//! united. No trimming or visibility is computed here; an unresolved or mixed
//! cell refuses the build.
use super::manifold::Solid;
use gcs_core::{envelope,interval::{Interval,minimum::Options},mesh,model::{Sketch,SolidDef},
    motion::Family,solid::{self,cad,MaterialField,ProbeState}};

fn stage(message: &str) { eprintln!("solventc: {message}"); }

/// A detail under `--verbose=2`: what a construction is made of.
/// How much the export says as it goes: 0 the stages, 1 a drip every few
/// seconds, 2 every sheet and column. Set by `solventc --verbose`.
pub static VERBOSITY: std::sync::atomic::AtomicU8 = std::sync::atomic::AtomicU8::new(0);
pub fn verbosity() -> u8 { VERBOSITY.load(std::sync::atomic::Ordering::Relaxed) }
fn detail(message: &str) { if verbosity() >= 2 { eprintln!("solventc:   {message}"); } }

/// A steady drip under `--verbose`: a line at most every few seconds, so a
/// stage that runs for minutes can be told from one that is wedged.
fn tick(message: &str) {
    use std::sync::Mutex;
    static LAST: Mutex<Option<std::time::Instant>> = Mutex::new(None);
    if verbosity() < 1 { return; }
    let mut last = LAST.lock().unwrap();
    if last.is_some_and(|t| t.elapsed().as_secs_f64() < 3.) { return; }
    *last = Some(std::time::Instant::now());
    eprintln!("solventc:   {message}");
}
fn sub(a: [f64;3],b: [f64;3]) -> [f64;3] { std::array::from_fn(|k| a[k]-b[k]) }
fn cross(a: [f64;3],b: [f64;3]) -> [f64;3] { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: [f64;3]) -> f64 { (a[0]*a[0]+a[1]*a[1]+a[2]*a[2]).sqrt() }
fn dot(a: [f64;3],b: [f64;3]) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }

/// A candidate sheet in model units: a row-major grid of contact positions with
/// the cutter's outward normal at each. `closed_rows` joins the last row back
/// to the first, a tube.
#[derive(Clone)]
pub struct SheetGrid { pub points: Vec<[f64;3]>,pub normals: Vec<[f64;3]>,pub times: Vec<f64>,pub rows: usize,pub columns: usize,pub closed_rows: bool }

impl SheetGrid {
    fn at(&self,r: usize,c: usize) -> [f64;3] { self.points[r*self.columns+c] }


}

/// A coarse occupancy of a solid: the cells of a cubic grid over its box whose
/// centres are inside it, or that a boundary triangle touches, or that lie
/// within the reach of one that is, so a query answers whether a point is
/// near the material.
struct Occupancy { lo: [f64;3], cell: f64, dims: [usize;3], cells: Vec<bool> }

impl Occupancy {
    /// `margin` is how far beyond the solid a point still reaches: at least
    /// two cells, and never less than the length asked, since a sheet is kept
    /// or cropped column by column and a column must not step from beyond
    /// reach into the solid in one stride.
    fn of(mesh: &Solid,resolution: usize,margin: f64) -> Result<Self,String> {
        let (vertices,triangles) = mesh.triangles()?;
        let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
        for p in &vertices { for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); } }
        let cell = (0..3).map(|k| hi[k]-lo[k]).fold(0_f64,f64::max)/resolution as f64;
        if !(cell > 0.) { return Err("an empty blank".into()); }
        let reach = ((margin/cell).ceil() as i64).max(2);
        // The grid extends past the blank by the dilation, so a point kept for
        // being near the material is never cut off by the grid's own edge.
        let lo: [f64;3] = lo.map(|v| v-(reach as f64+1.)*cell);
        let hi: [f64;3] = hi.map(|v| v+(reach as f64+1.)*cell);
        let dims: [usize;3] = std::array::from_fn(|k| ((hi[k]-lo[k])/cell).ceil() as usize+1);
        let index = |x: usize,y: usize,z: usize| (x*dims[1]+y)*dims[2]+z;
        // Every cell a boundary triangle's box touches is occupied, and the
        // rest of the blank is what a flood from the grid's own edge cannot
        // reach: a closed surface's cells separate the outside from the inside
        // for a six-connected walk, since a step between neighbouring centres
        // that crosses the surface crosses it inside one of the two cells.
        let mut occupied = vec![false;dims[0]*dims[1]*dims[2]];
        for t in &triangles {
            let corners = t.map(|i| vertices[i as usize]);
            let range = |k: usize| {
                let (a,b) = (corners.iter().map(|p| p[k]).fold(f64::INFINITY,f64::min),corners.iter().map(|p| p[k]).fold(f64::NEG_INFINITY,f64::max));
                (((a-lo[k])/cell).floor().max(0.) as usize)..=(((b-lo[k])/cell).floor().max(0.) as usize).min(dims[k]-1)
            };
            for x in range(0) { for y in range(1) { for z in range(2) { occupied[index(x,y,z)] = true; } } }
        }
        let mut outside = vec![false;occupied.len()];
        let mut pending = vec![(0usize,0usize,0usize)];
        outside[0] = true;
        while let Some((x,y,z)) = pending.pop() {
            for (dx,dy,dz) in [(1i64,0i64,0i64),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)] {
                let (nx,ny,nz) = (x as i64+dx,y as i64+dy,z as i64+dz);
                if nx < 0 || ny < 0 || nz < 0 { continue; }
                let (nx,ny,nz) = (nx as usize,ny as usize,nz as usize);
                if nx >= dims[0] || ny >= dims[1] || nz >= dims[2] { continue; }
                let i = index(nx,ny,nz);
                if occupied[i] || outside[i] { continue; }
                outside[i] = true; pending.push((nx,ny,nz));
            }
        }
        for i in 0..occupied.len() { if !outside[i] { occupied[i] = true; } }
        // dilate by the reach, so a sheet is kept wherever it comes near: a
        // box dilation, one sliding window per axis in turn, since the cube
        // of neighbours per cell was the whole cost of a small blank with a
        // margin many cells wide
        let mut cells = occupied;
        let reach = reach as usize;
        for axis in 0..3 {
            let stride = match axis { 0 => dims[1]*dims[2],1 => dims[2],_ => 1 };
            let length = dims[axis];
            let mut line = vec![false;length];
            let mut prefix = vec![0usize;length+1];
            for x in 0..if axis == 0 { 1 } else { dims[0] } { for y in 0..if axis == 1 { 1 } else { dims[1] } { for z in 0..if axis == 2 { 1 } else { dims[2] } {
                let base = index(x,y,z);
                for i in 0..length { line[i] = cells[base+i*stride]; prefix[i+1] = prefix[i]+line[i] as usize; }
                for i in 0..length {
                    let (a,b) = (i.saturating_sub(reach),(i+reach+1).min(length));
                    cells[base+i*stride] = prefix[b] > prefix[a];
                }
            } } }
        }
        Ok(Self {lo,cell,dims,cells})
    }

    fn reaches(&self,p: [f64;3]) -> bool {
        let mut i = [0usize;3];
        for k in 0..3 {
            let v = ((p[k]-self.lo[k])/self.cell).floor();
            if v < 0. || v >= self.dims[k] as f64 { return false; }
            i[k] = v as usize;
        }
        self.cells[(i[0]*self.dims[1]+i[1])*self.dims[2]+i[2]]
    }
}

fn point_triangle_distance(p: [f64;3],a: [f64;3],b: [f64;3],c: [f64;3]) -> f64 {
    // Ericson's closest point on a triangle
    let (ab,ac,ap) = (sub(b,a),sub(c,a),sub(p,a));
    let dot = |x: [f64;3],y: [f64;3]| x[0]*y[0]+x[1]*y[1]+x[2]*y[2];
    let (d1,d2) = (dot(ab,ap),dot(ac,ap));
    if d1 <= 0. && d2 <= 0. { return norm(ap); }
    let bp = sub(p,b); let (d3,d4) = (dot(ab,bp),dot(ac,bp));
    if d3 >= 0. && d4 <= d3 { return norm(bp); }
    let vc = d1*d4-d3*d2;
    if vc <= 0. && d1 >= 0. && d3 <= 0. { let v = d1/(d1-d3); return norm(sub(p,std::array::from_fn(|k| a[k]+v*ab[k]))); }
    let cp = sub(p,c); let (d5,d6) = (dot(ab,cp),dot(ac,cp));
    if d6 >= 0. && d5 <= d6 { return norm(cp); }
    let vb = d5*d2-d1*d6;
    if vb <= 0. && d2 >= 0. && d6 <= 0. { let w = d2/(d2-d6); return norm(sub(p,std::array::from_fn(|k| a[k]+w*ac[k]))); }
    let va = d3*d6-d5*d4;
    if va <= 0. && d4-d3 >= 0. && d5-d6 >= 0. { let w = (d4-d3)/((d4-d3)+(d5-d6)); return norm(sub(p,std::array::from_fn(|k| b[k]+w*(c[k]-b[k])))); }
    let denom = 1./(va+vb+vc); let (v,w) = (vb*denom,vc*denom);
    norm(sub(p,std::array::from_fn(|k| a[k]+ab[k]*v+ac[k]*w)))
}


/// A candidate sheet as triangles with a normal at each vertex, in model
/// units: what the core's zipped patches, a grid's quads and a seam ribbon
/// all are. Vertices come column by column in order along each column's
/// curve; `column[v]` says which, and `times` the motion parameter of each
/// column, so the seams between sheets can be found and closed.
#[derive(Clone)]
pub struct Patch { pub vertices: Vec<[f64;3]>, pub normals: Vec<[f64;3]>, pub triangles: Vec<[u32;3]>, pub column: Vec<u32>, pub times: Vec<f64> }

impl SheetGrid {
    /// The grid as triangles, two per quad, its columns kept.
    pub fn patch(&self) -> Patch {
        let (rows,columns) = (self.rows,self.columns);
        // vertices column by column
        let mut vertices = Vec::with_capacity(rows*columns); let mut normals = Vec::with_capacity(rows*columns); let mut column = Vec::with_capacity(rows*columns);
        for c in 0..columns { for r in 0..rows { vertices.push(self.at(r,c)); normals.push(self.normals[r*columns+c]); column.push(c as u32); } }
        let at = |r: usize,c: usize| (c*rows+r%rows) as u32;
        let row_spans = if self.closed_rows { rows } else { rows-1 };
        let mut triangles = Vec::with_capacity(2*row_spans*(columns-1));
        for r in 0..row_spans { for c in 0..columns-1 {
            let (a,b,d,e) = (at(r,c),at(r,c+1),at(r+1,c),at(r+1,c+1));
            triangles.push([a,b,e]); triangles.push([a,e,d]);
        } }
        Patch {vertices,normals,triangles,column,times:self.times.clone()}
    }
}

impl From<solid::SweepPatch> for Patch {
    fn from(s: solid::SweepPatch) -> Self { Patch {vertices:s.points,normals:s.normals,triangles:s.triangles,column:s.column,times:s.times} }
}

/// Triangles of a patch by the cells of a cubic grid their boxes reach.
pub struct PatchIndex { cell: f64, buckets: std::collections::HashMap<[i64;3],Vec<usize>> }

impl Patch {
    /// The patch carried by a rigid pose.
    pub fn placed(&self,pose: envelope::Motion) -> Self {
        Patch {vertices:self.vertices.iter().map(|p| pose.point(*p)).collect(),
            normals:self.normals.iter().map(|n| pose.vector(*n)).collect(),triangles:self.triangles.clone(),column:self.column.clone(),times:self.times.clone()}
    }

    /// The vertices of one column, in order along its curve.
    fn column_vertices(&self,c: u32) -> Vec<u32> { (0..self.vertices.len() as u32).filter(|&v| self.column[v as usize] == c).collect() }

    /// The first and last columns present.
    fn column_range(&self) -> (u32,u32) {
        self.column.iter().fold((u32::MAX,0),|(lo,hi),&c| (lo.min(c),hi.max(c)))
    }

    /// A pair of non-adjacent triangles that cross, if any: a slab of such a
    /// patch overlaps itself, which the kernel's Booleans cannot take.
    fn self_intersection(&self) -> Option<(usize,usize)> {
        let index = self.index(COLUMN_SPACING);
        let corners = |t: usize| self.triangles[t].map(|v| self.vertices[v as usize]);
        for a in 0..self.triangles.len() {
            let ca = corners(a);
            let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
            for p in &ca { for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); } }
            let mut candidates: Vec<usize> = Vec::new();
            let key = |p: [f64;3]| p.map(|x| (x/index.cell).floor() as i64);
            let (klo,khi) = (key(lo),key(hi));
            for x in klo[0]..=khi[0] { for y in klo[1]..=khi[1] { for z in klo[2]..=khi[2] {
                if let Some(ts) = index.buckets.get(&[x,y,z]) { candidates.extend(ts.iter().copied()); }
            } } }
            candidates.sort(); candidates.dedup();
            for b in candidates {
                if b <= a || self.triangles[a].iter().any(|v| self.triangles[b].contains(v)) { continue; }
                if triangles_cross(ca,corners(b)) { return Some((a,b)); }
            }
        }
        None
    }

    /// The directed edges used by exactly one triangle: the patch's rim.
    fn boundary_edges(&self) -> Vec<(u32,u32)> {
        let mut directed: std::collections::BTreeSet<(u32,u32)> = Default::default();
        for t in &self.triangles { for k in 0..3 { directed.insert((t[k],t[(k+1)%3])); } }
        directed.iter().copied().filter(|&(p,q)| !directed.contains(&(q,p))).collect()
    }

    /// The part of the patch that reaches `keep`: every triangle with a vertex
    /// that does, and the ring of triangles round those, so what the crop
    /// leaves as a rim lies beyond `keep`, away from the body. Vertices no
    /// kept triangle uses are dropped.
    pub fn cropped(&self,keep: &dyn Fn(&[f64;3]) -> bool) -> Option<Patch> {
        let reaching: Vec<bool> = self.vertices.iter().map(|p| keep(p)).collect();
        let mut kept: Vec<bool> = self.triangles.iter().map(|t| t.iter().any(|&v| reaching[v as usize])).collect();
        if !kept.iter().any(|k| *k) { return None; }
        let mut touched = vec![false;self.vertices.len()];
        for (t,k) in self.triangles.iter().zip(&kept) { if *k { for &v in t { touched[v as usize] = true; } } }
        for (t,k) in self.triangles.iter().zip(kept.iter_mut()) { if t.iter().any(|&v| touched[v as usize]) { *k = true; } }
        let mut used = vec![false;self.vertices.len()];
        for (t,k) in self.triangles.iter().zip(&kept) { if *k { for &v in t { used[v as usize] = true; } } }
        let mut remap = vec![u32::MAX;self.vertices.len()];
        let mut vertices = Vec::new(); let mut normals = Vec::new(); let mut column = Vec::new();
        for (v,p) in self.vertices.iter().enumerate() {
            if used[v] { remap[v] = vertices.len() as u32; vertices.push(*p); normals.push(self.normals[v]); column.push(self.column[v]); }
        }
        let triangles = self.triangles.iter().zip(&kept).filter(|(_,k)| **k).map(|(t,_)| t.map(|v| remap[v as usize])).collect();
        Some(Patch {vertices,normals,triangles,column,times:self.times.clone()})
    }

    pub fn index(&self,cell: f64) -> PatchIndex {
        let mut buckets: std::collections::HashMap<[i64;3],Vec<usize>> = Default::default();
        let key = |p: [f64;3]| p.map(|v| (v/cell).floor() as i64);
        for (i,t) in self.triangles.iter().enumerate() {
            let corners = t.map(|v| self.vertices[v as usize]);
            let (lo,hi) = (key(std::array::from_fn(|k| corners.iter().map(|q| q[k]).fold(f64::INFINITY,f64::min))),
                key(std::array::from_fn(|k| corners.iter().map(|q| q[k]).fold(f64::NEG_INFINITY,f64::max))));
            for x in lo[0]..=hi[0] { for y in lo[1]..=hi[1] { for z in lo[2]..=hi[2] { buckets.entry([x,y,z]).or_default().push(i); } } }
        }
        PatchIndex {cell,buckets}
    }

    /// The distance from `p` to the patch within `within`, or more than that
    /// if none of it is that close.
    pub fn distance_within(&self,index: &PatchIndex,p: [f64;3],within: f64) -> f64 {
        let key = |v: f64| (v/index.cell).floor() as i64;
        let (lo,hi) = (p.map(|v| key(v-within)),p.map(|v| key(v+within)));
        let mut best = f64::INFINITY;
        for x in lo[0]..=hi[0] { for y in lo[1]..=hi[1] { for z in lo[2]..=hi[2] {
            let Some(triangles) = index.buckets.get(&[x,y,z]) else { continue; };
            for &i in triangles {
                let [a,b,c] = self.triangles[i].map(|v| self.vertices[v as usize]);
                best = best.min(point_triangle_distance(p,a,b,c));
            }
        } } }
        best
    }
}

/// A patch cut along its folds into pieces: where two triangles on one edge
/// face opposite ways the sheet doubles back on itself (the two branches of
/// a contact curve meeting at a fold), and one slab of it would overlap
/// itself, which no Boolean can take; two slabs that share the fold line
/// are an ordinary union. Every piece keeps the fold's vertices.
fn split_at_folds(patch: &Patch) -> Vec<Patch> {
    let normal = |t: &[u32;3]| { let [a,b,c] = t.map(|v| patch.vertices[v as usize]); cross(sub(b,a),sub(c,a)) };
    let normals: Vec<[f64;3]> = patch.triangles.iter().map(normal).collect();
    let mut by_edge: std::collections::BTreeMap<(u32,u32),Vec<usize>> = Default::default();
    for (i,t) in patch.triangles.iter().enumerate() { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); by_edge.entry((a.min(b),a.max(b))).or_default().push(i); } }
    // union-find over triangles across unfolded edges
    let mut parent: Vec<usize> = (0..patch.triangles.len()).collect();
    fn find(parent: &mut Vec<usize>,i: usize) -> usize { let mut r = i; while parent[r] != r { r = parent[r]; } let mut j = i; while parent[j] != r { let n = parent[j]; parent[j] = r; j = n; } r }
    for ts in by_edge.values() {
        if ts.len() != 2 { continue; }
        let (n1,n2) = (normals[ts[0]],normals[ts[1]]);
        let folded = n1[0]*n2[0]+n1[1]*n2[1]+n1[2]*n2[2] < 0. && norm(n1)*norm(n2) > 1e-24;
        if !folded { let (a,b) = (find(&mut parent,ts[0]),find(&mut parent,ts[1])); parent[a] = b; }
    }
    let mut groups: std::collections::BTreeMap<usize,Vec<usize>> = Default::default();
    for i in 0..patch.triangles.len() { let r = find(&mut parent,i); groups.entry(r).or_default().push(i); }
    if groups.len() == 1 { return vec![patch.clone()]; }
    groups.values().map(|ts| {
        let mut remap: std::collections::BTreeMap<u32,u32> = Default::default();
        let mut vertices = Vec::new(); let mut normals = Vec::new(); let mut column = Vec::new();
        let triangles = ts.iter().map(|&i| patch.triangles[i].map(|v| *remap.entry(v).or_insert_with(|| {
            vertices.push(patch.vertices[v as usize]); normals.push(patch.normals[v as usize]); column.push(patch.column[v as usize]); (vertices.len()-1) as u32 }))).collect();
        Patch {vertices,normals,triangles,column,times:patch.times.clone()}
    }).collect()
}

/// The ribbons that close the seams between sheets. Where one strip ends and
/// another begins at one parameter, their end columns are one curve (or a
/// curve and the pieces it became) traced twice, and differ by up to the
/// tracer's own resolution: far more than a slab's thickness. Each end
/// column is zipped to the columns that meet it there: every vertex of it is
/// projected onto the nearest of them, and the run of vertices landing on one
/// partner is triangulated against the partner's own vertices between the
/// first and last projection, so the ribbon's edges are exactly segments of
/// both polylines and it closes the seam without a chord of its own.
fn seam_ribbons(sheets: &[Patch],within: f64) -> Vec<Patch> {
    let column = |g: &Patch,c: u32| -> (Vec<[f64;3]>,Vec<[f64;3]>) {
        let vs = g.column_vertices(c);
        (vs.iter().map(|&v| g.vertices[v as usize]).collect(),vs.iter().map(|&v| g.normals[v as usize]).collect())
    };
    // A death and the birth beside it are each bisected to their own event,
    // and where the curve between them matches neither side the two lie up to
    // a column step apart: a partner is a sheet ending within one step before
    // this one begins, or beginning within one step after it ends.
    let step = |g: &Patch| g.times.windows(2).map(|w| w[1]-w[0]).fold(0_f64,f64::max).max(f64::MIN_POSITIVE);
    let mut ribbons = Vec::new();
    for (i,g) in sheets.iter().enumerate() {
        let (first,last) = g.column_range();
        for (c,partner_end) in [(first,true),(last,false)] {
            let t = g.times[c as usize];
            let partners: Vec<(Vec<[f64;3]>,Vec<[f64;3]>)> = sheets.iter().enumerate().filter(|(j,_)| *j != i)
                .filter_map(|(_,h)| {
                    let (hf,hl) = h.column_range();
                    let pc = if partner_end { hl } else { hf };
                    let tp = h.times[pc as usize];
                    let within = step(g).max(step(h))*1.001;
                    let meets = if partner_end { tp <= t+1e-9 && t-tp <= within } else { tp >= t-1e-9 && tp-t <= within };
                    meets.then(|| column(h,pc))
                }).filter(|(p,_)| p.len() >= 2).collect();
            if partners.is_empty() { continue; }
            let (points,normals) = column(g,c);
            if points.len() < 2 { continue; }
            // (partner, segment, fraction) of the nearest point of any partner
            let nearest = |p: [f64;3]| -> Option<(usize,usize,f64)> {
                let mut best: Option<(f64,(usize,usize,f64))> = None;
                for (pi,(line,_)) in partners.iter().enumerate() {
                    for k in 0..line.len()-1 {
                        let (a,b) = (line[k],line[k+1]);
                        let ab = sub(b,a); let ap = sub(p,a);
                        let l2 = norm(ab).powi(2);
                        let f = if l2 > 0. { (ab[0]*ap[0]+ab[1]*ap[1]+ab[2]*ap[2])/l2 } else { 0. }.clamp(0.,1.);
                        let q: [f64;3] = std::array::from_fn(|k| a[k]+f*ab[k]);
                        let d = norm(sub(p,q));
                        if best.map_or(true,|(bd,_)| d < bd) { best = Some((d,(pi,k,f))); }
                    }
                }
                best.filter(|(d,_)| *d <= within).map(|(_,q)| q)
            };
            let projected: Vec<Option<(usize,usize,f64)>> = points.iter().map(|p| nearest(*p)).collect();
            // runs of consecutive rows landing on one partner
            let mut start = 0;
            while start < points.len() {
                let Some((pi,_,_)) = projected[start] else { start += 1; continue; };
                let mut end = start;
                while end+1 < points.len() && projected[end+1].is_some_and(|(q,_,_)| q == pi) { end += 1; }
                if end > start {
                    let (line,line_normals) = &partners[pi];
                    let mut s: Vec<f64> = (start..=end).map(|r| { let (_,k,f) = projected[r].unwrap(); k as f64+f }).collect();
                    let reversed = s[s.len()-1] < s[0];
                    let mut d_points: Vec<[f64;3]> = points[start..=end].to_vec();
                    let mut d_normals: Vec<[f64;3]> = normals[start..=end].to_vec();
                    if reversed { d_points.reverse(); d_normals.reverse(); s.reverse(); }
                    for r in 1..s.len() { if s[r] < s[r-1] { s[r] = s[r-1]; } }
                    let at = |x: f64| -> ([f64;3],[f64;3]) {
                        let k = (x.floor() as usize).min(line.len()-2); let f = x-k as f64;
                        (std::array::from_fn(|j| line[k][j]+f*(line[k+1][j]-line[k][j])),std::array::from_fn(|j| line_normals[k][j]+f*(line_normals[k+1][j]-line_normals[k][j])))
                    };
                    // the partner's vertices strictly between the two end projections, with those ends
                    let mut b_params: Vec<f64> = vec![s[0]];
                    let mut k = s[0].floor()+1.;
                    while k < s[s.len()-1] { b_params.push(k); k += 1.; }
                    b_params.push(s[s.len()-1]);
                    let b_points: Vec<([f64;3],[f64;3])> = b_params.iter().map(|&x| at(x)).collect();
                    let mut vertices: Vec<[f64;3]> = d_points.clone(); let mut vnormals = d_normals.clone();
                    let db = vertices.len() as u32;
                    for (p,nrm) in &b_points { vertices.push(*p); vnormals.push(*nrm); }
                    let b_only: Vec<[f64;3]> = b_points.iter().map(|q| q.0).collect();
                    let mut triangles: Vec<[u32;3]> = solid::zip_polylines(&d_points,&b_only,false).into_iter()
                        .map(|t| t.map(|(on_b,k)| if on_b { db+k } else { k })).collect();
                    let degenerate = |t: &[u32;3]| { let [a,b,c] = t.map(|k| vertices[k as usize]); norm(cross(sub(b,a),sub(c,a))) < 1e-18 };
                    triangles.retain(|t| !degenerate(t));
                    if !triangles.is_empty() {
                        let column = vec![0;vertices.len()];
                        ribbons.push(Patch {vertices,normals:vnormals,triangles,column,times:vec![t]});
                    }
                }
                start = end+1;
            }
        }
    }
    ribbons
}

/// A closed slab of thickness `epsilon` centred on the sheet.
#[allow(dead_code)]
pub fn slab(sheet: &SheetGrid,epsilon: f64) -> Result<Solid,String> { slab_of(&sheet.patch(),epsilon) }

/// A closed slab of thickness `epsilon` centred on a patch: the patch offset
/// either way along its normals, and a wall along every boundary edge.
pub fn slab_of(patch: &Patch,epsilon: f64) -> Result<Solid,String> {
    let n = patch.vertices.len();
    let mut vertices = Vec::with_capacity(2*n);
    for side in [1.,-1.] {
        for (p,normal) in patch.vertices.iter().zip(&patch.normals) {
            vertices.push(std::array::from_fn(|k| p[k]+side*0.5*epsilon*normal[k]));
        }
    }
    let below = |i: u32| i+n as u32;
    let mut triangles = Vec::with_capacity(4*patch.triangles.len());
    let mut directed: std::collections::BTreeSet<(u32,u32)> = Default::default();
    for t in &patch.triangles {
        triangles.push(*t);
        triangles.push([below(t[0]),below(t[2]),below(t[1])]);
        for k in 0..3 { directed.insert((t[k],t[(k+1)%3])); }
    }
    // Side walls: for each directed boundary edge p->q of the top surface (one
    // with no opposite), the wall quad carries q->p on top and p'->q' below,
    // so every edge pairs.
    for &(p,q) in &directed {
        if directed.contains(&(q,p)) { continue; }
        triangles.push([q,p,below(p)]); triangles.push([q,below(p),below(q)]);
    }
    let solid = Solid::from_triangles(&vertices,&triangles)?;
    if solid.volume() < 0. {
        // The patch's orientation is the sheet's; wind the other way if the
        // offset side turned out to be inward.
        let flipped: Vec<[u32;3]> = triangles.iter().map(|t| [t[0],t[2],t[1]]).collect();
        return Solid::from_triangles(&vertices,&flipped);
    }
    Ok(solid)
}

/// Whether two triangles with no shared vertex cross: an edge of either
/// passes through the other's interior.
fn triangles_cross(a: [[f64;3];3],b: [[f64;3];3]) -> bool {
    let through = |p: [f64;3],q: [f64;3],t: [[f64;3];3]| -> bool {
        let n = cross(sub(t[1],t[0]),sub(t[2],t[0]));
        let (dp,dq) = (dot(sub(p,t[0]),n),dot(sub(q,t[0]),n));
        if dp*dq >= 0. { return false; }
        let f = dp/(dp-dq);
        let x: [f64;3] = std::array::from_fn(|k| p[k]+f*(q[k]-p[k]));
        (0..3).all(|k| dot(cross(sub(t[(k+1)%3],t[k]),sub(x,t[k])),n) > 0.)
    };
    (0..3).any(|k| through(a[k],a[(k+1)%3],b) || through(b[k],b[(k+1)%3],a))
}

/// Whether a triangle overlaps an axis-aligned box (Akenine-Möller).
fn triangle_meets_box(tri: [[f64;3];3],centre: [f64;3],half: f64) -> bool {
    let v: Vec<[f64;3]> = tri.iter().map(|p| sub(*p,centre)).collect();
    for k in 0..3 {
        let (lo,hi) = v.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),p| (lo.min(p[k]),hi.max(p[k])));
        if lo > half || hi < -half { return false; }
    }
    let edges = [sub(v[1],v[0]),sub(v[2],v[1]),sub(v[0],v[2])];
    let axes = [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]];
    for e in &edges { for a in &axes {
        let axis = cross(*a,*e);
        let r = half*(axis[0].abs()+axis[1].abs()+axis[2].abs());
        let ps: Vec<f64> = v.iter().map(|p| p[0]*axis[0]+p[1]*axis[1]+p[2]*axis[2]).collect();
        let (lo,hi) = ps.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),x| (lo.min(*x),hi.max(*x)));
        if lo > r || hi < -r { return false; }
    } }
    let n = cross(edges[0],edges[1]);
    let r = half*(n[0].abs()+n[1].abs()+n[2].abs());
    let d = n[0]*v[0][0]+n[1]*v[0][1]+n[2]*v[0][2];
    d.abs() <= r
}

/// A leak search, `SOLVENT_FIND_LEAK=<voxel mm>`: over the box of one
/// placement's slab union, a six-connected flood on a voxel grid from the
/// voxels the tool removes at mid roll, blocked by every triangle of the union,
/// the caps and the blank, until it reaches a voxel the field calls material:
/// the path there is where the sheets fail to enclose the cut. Nothing is
/// found on a sound placement; the report is the whole result.
fn find_leak(blockers: &[&Solid],removed: &dyn Fn([f64;3]) -> bool,material: &mut dyn FnMut([f64;3]) -> Result<bool,String>,
    voxel: f64) -> Result<(),String> {
    let (uv,ut) = blockers[0].triangles()?;
    let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
    for p in &uv { for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); } }
    let lo = lo.map(|x| x-2.*voxel); let hi = hi.map(|x| x+2.*voxel);
    let dims: [usize;3] = std::array::from_fn(|k| ((hi[k]-lo[k])/voxel).ceil() as usize+1);
    let index = |i: [usize;3]| (i[0]*dims[1]+i[1])*dims[2]+i[2];
    let centre = |i: [usize;3]| -> [f64;3] { std::array::from_fn(|k| lo[k]+voxel*(i[k] as f64+0.5)) };
    let mut blocked = vec![false;dims[0]*dims[1]*dims[2]];
    let mut count = 0;
    for (b,solid) in blockers.iter().enumerate() {
        let (v,t) = if b == 0 { (uv.clone(),ut.clone()) } else { solid.triangles()? };
        for tri in &t {
            let corners = tri.map(|i| v[i as usize]);
            let range = |k: usize| {
                let (a,b) = (corners.iter().map(|p| p[k]).fold(f64::INFINITY,f64::min),corners.iter().map(|p| p[k]).fold(f64::NEG_INFINITY,f64::max));
                if b < lo[k] || a > hi[k] { return 1..=0; }
                (((a-lo[k])/voxel).floor().max(0.) as usize)..=(((b-lo[k])/voxel).floor().max(0.) as usize).min(dims[k]-1)
            };
            for x in range(0) { for y in range(1) { for z in range(2) {
                let i = [x,y,z];
                if !blocked[index(i)] && triangle_meets_box(corners,centre(i),0.5*voxel) { blocked[index(i)] = true; count += 1; }
            } } }
        }
    }
    // Only the blank's inside is searched: ray parity against the blank's
    // own triangles along every z column of the grid.
    let (bv,bt) = blockers[1].triangles()?;
    let mut inside = vec![false;blocked.len()];
    for x in 0..dims[0] { for y in 0..dims[1] {
        let (px,py) = (lo[0]+voxel*(x as f64+0.5),lo[1]+voxel*(y as f64+0.5));
        let mut crossings: Vec<f64> = Vec::new();
        for tri in &bt {
            let [a,b,c] = tri.map(|i| bv[i as usize]);
            // 2D barycentric test in xy, then the z of the plane there
            let d = (b[0]-a[0])*(c[1]-a[1])-(c[0]-a[0])*(b[1]-a[1]);
            if d.abs() < 1e-300 { continue; }
            let u = ((px-a[0])*(c[1]-a[1])-(c[0]-a[0])*(py-a[1]))/d;
            let v = ((b[0]-a[0])*(py-a[1])-(px-a[0])*(b[1]-a[1]))/d;
            if u < 0. || v < 0. || u+v > 1. { continue; }
            crossings.push(a[2]+u*(b[2]-a[2])+v*(c[2]-a[2]));
        }
        crossings.sort_by(f64::total_cmp);
        for z in 0..dims[2] {
            let pz = lo[2]+voxel*(z as f64+0.5);
            let below = crossings.iter().filter(|&&c| c < pz).count();
            inside[index([x,y,z])] = below % 2 == 1;
        }
    } }
    stage(&format!("leak search: {:?} voxels of {voxel} mm, {count} blocked, {} inside the blank",dims,inside.iter().filter(|i| **i).count()));
    let mut parent: Vec<u32> = vec![u32::MAX;blocked.len()];
    let mut pending: std::collections::VecDeque<[usize;3]> = Default::default();
    let mut seeds = 0;
    for x in 0..dims[0] { for y in 0..dims[1] { for z in 0..dims[2] {
        let i = [x,y,z];
        if !blocked[index(i)] && inside[index(i)] && removed(centre(i)) { parent[index(i)] = index(i) as u32; pending.push_back(i); seeds += 1; }
    } } }
    stage(&format!("leak search: {seeds} seed voxels removed at mid roll"));
    let mut reached = 0usize;
    while let Some(i) = pending.pop_front() {
        reached += 1;
        if reached % 500 == 0 || pending.is_empty() {
            let p = centre(i);
            if material(p)? {
                let mut path = vec![p];
                let mut j = index(i);
                while parent[j] as usize != j { j = parent[j] as usize; let c = [j/(dims[1]*dims[2]),(j/dims[2])%dims[1],j%dims[2]]; path.push(centre(c)); }
                stage(&format!("LEAK: the flood reached material at {p:?} after {reached} voxels, {} steps from a removed seed at {:?}",path.len(),path[path.len()-1]));
                let step = (path.len()/20).max(1);
                for (k,q) in path.iter().enumerate() { if k % step == 0 || k+10 >= path.len() || k < 10 { stage(&format!("  path {k}: {q:?}")); } }
                return Ok(());
            }
        }
        for (dx,dy,dz) in [(1i64,0i64,0i64),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)] {
            let n = [i[0] as i64+dx,i[1] as i64+dy,i[2] as i64+dz];
            if n.iter().any(|&c| c < 0) || (0..3).any(|k| n[k] as usize >= dims[k]) { continue; }
            let n = n.map(|c| c as usize);
            if blocked[index(n)] || !inside[index(n)] || parent[index(n)] != u32::MAX { continue; }
            parent[index(n)] = index(i) as u32; pending.push_back(n);
        }
    }
    stage(&format!("leak search: the flood stayed enclosed, {reached} voxels"));
    Ok(())
}

/// Evaluate a static solid's Boolean term with Manifold over its primitives.
pub fn solid_of(fixed: &solid::StaticSolid) -> Result<Solid,String> {
    let mut prims: Vec<Option<Solid>> = Vec::with_capacity(fixed.csg.prims.len());
    for prim in &fixed.csg.prims {
        let (vertices,triangles) = solid::primitive_triangles(prim,fixed.origin);
        prims.push(Some(Solid::from_triangles(&vertices,&triangles).map_err(|e| format!("`{}`: {e}",prim.of))?));
    }
    fn evaluate(term: &solid::Term,prims: &[Option<Solid>]) -> Result<Solid,String> {
        Ok(match term {
            solid::Term::Prim(i) => {
                let (vertices,triangles) = prims[*i].as_ref().unwrap().triangles()?;
                Solid::from_triangles(&vertices,&triangles)?
            }
            solid::Term::Union(a,b) => evaluate(a,prims)?.union(&evaluate(b,prims)?)?,
            solid::Term::Diff(a,b) => evaluate(a,prims)?.difference(&evaluate(b,prims)?)?,
            solid::Term::Inter(a,b) => evaluate(a,prims)?.intersection(&evaluate(b,prims)?)?,
            solid::Term::Empty => return Err("an empty operand cannot be meshed".into()),
        })
    }
    evaluate(&fixed.csg.term,&prims)
}

/// A point inside the solid, `depth` behind its largest triangle, with that
/// triangle's outward normal.
pub fn interior_point(solid: &Solid,depth: f64) -> Result<[f64;3],String> {
    let (vertices,triangles) = solid.triangles()?;
    let mut best: Option<(f64,[f64;3],[f64;3])> = None;
    for t in &triangles {
        let [a,b,c] = t.map(|i| vertices[i as usize]);
        let n = cross(sub(b,a),sub(c,a));
        let area = norm(n);
        if area <= 0. { continue; }
        if best.as_ref().is_none_or(|(a,_,_)| area > *a) {
            best = Some((area,std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.),n.map(|v| v/area)));
        }
    }
    let (_,centroid,normal) = best.ok_or("a cell has no triangles")?;
    Ok(std::array::from_fn(|k| centroid[k]-depth*normal[k]))
}

impl From<solid::SweepSheet> for SheetGrid {
    fn from(s: solid::SweepSheet) -> Self {
        SheetGrid {points:s.points,normals:s.normals,times:s.times,rows:s.rows,columns:s.columns,closed_rows:s.closed_rows}
    }
}

/// Sheets of a sweep whose tool-frame twist is constant: the core carries each
/// characteristic along the motion, with two slabs' worth of overrun into the
/// caps at both ends.
pub fn constant_twist_sheets(sk: &Sketch,swept: usize,epsilon: f64,sagitta: f64) -> Result<Vec<Patch>,String> {
    let sweep = solid::SweepContacts::read(sk,swept,1e-10)?;
    let sheets = sweep.carried_sheets(COLUMN_SPACING,sagitta,2.*epsilon,1e-9)
        .map_err(|e| format!("`{}`: {e}",sk.solids[swept].name))?;
    Ok(sheets.into_iter().map(|s| SheetGrid::from(s).patch()).collect())
}

/// Sheets of a sweep whose tool-frame twist varies: the core's contact curves
/// at a sequence of motion parameters, linked into strips and zipped.
pub fn traced_sheets(sk: &Sketch,swept: usize,epsilon: f64,sagitta: f64,reach: &dyn Fn([f64;3]) -> bool) -> Result<Vec<Patch>,String> {
    let started = std::time::Instant::now();
    let sweep = solid::SweepContacts::read(sk,swept,1e-10)?;
    let name = &sk.solids[swept].name;
    stage(&format!("`{name}`: tool read, {} faces, {} creases ({:?})",sweep.faces().len(),sweep.creases().len(),started.elapsed()));
    let sheets = sweep.characteristic_sheets(COLUMN_SPACING,sagitta,2.*epsilon,1e-9,Some(reach),&|step| tick(&format!("`{name}`: tracing, {step}")))?;
    stage(&format!("`{}`: traced {} sheets of {} points ({:?})",sk.solids[swept].name,sheets.len(),
        sheets.iter().map(|s| s.points.len()).sum::<usize>(),started.elapsed()));
    for (i,s) in sheets.iter().enumerate() {
        detail(&format!("sheet {i}: {} points, {} columns over {:.6}..{:.6}, {}",s.points.len(),s.times.len(),s.times[0],s.times[s.times.len()-1],
            if s.closed { "closed" } else { "open" }));
    }
    Ok(sheets.into_iter().map(Patch::from).collect())
}

/// Target node spacing along a sheet's band, model millimetres.
const COLUMN_SPACING: f64 = 0.5;


/// The body as indexed triangles in model units. `sheets` supplies the
/// candidate sheets of a sweep that is not constant-twist, given a containment
/// oracle for the blank in native millimetres.
pub fn construct(sk: &Sketch,body: usize,sheets: &dyn Fn(usize,&dyn Fn(&[[f64;3]]) -> Result<Vec<bool>,String>) -> Result<Vec<SheetGrid>,String>)
    -> Result<(Vec<[f64;3]>,Vec<[u32;3]>),String> {
    let recipe = cad::recipe_static(sk,body)?;
    let scale = sk.units.length.ok_or("CAD construction requires an explicit length unit")?.1;
    let name = sk.solids[body].name.clone();
    let started = std::time::Instant::now();
    let blank = solid::static_solid(sk,body,recipe.sweeps.len())?;
    let blank_mesh = solid_of(&blank)?;
    stage(&format!("`{name}`: blank meshed, {} triangles, {:.6} mm³ ({:?})",blank_mesh.triangle_count(),
        blank_mesh.volume()*scale.powi(3),started.elapsed()));
    let started = std::time::Instant::now();
    let occupancy = Occupancy::of(&blank_mesh,64,2.*COLUMN_SPACING)?;
    stage(&format!("`{name}`: occupancy of the blank ({:?})",started.elapsed()));
    // Slab thickness: a micrometre in native millimetres, expressed in model units.
    let epsilon = 1e-3/scale;
    // Chordal facets of a subtracted round face lie outside the true surface by
    // up to the tessellation sagitta, so a cell is judged deeper than that.
    let sagitta = blank.unit*gcs_core::curve::FLATNESS_PX;
    let inside = |points: &[[f64;3]]| -> Result<Vec<bool>,String> { Ok(points.iter().map(|p| blank.contains(p.map(|v| v/scale))).collect()) };
    let mut distinct: Vec<usize> = recipe.sweeps.iter().map(|s| s.swept).collect();
    distinct.sort(); distinct.dedup();
    let (mut slabs,mut caps) = (Vec::new(),Vec::new());
    // the slabs of each placement, split from the blank one placement at a time
    let mut groups: Vec<std::ops::Range<usize>> = Vec::new();
    // what each placement removes at mid roll, for the leak search
    let mut removed_by: Vec<Box<dyn Fn([f64;3]) -> bool>> = Vec::new();
    for swept in distinct {
        let SolidDef::Swept {source,motion,from,to} = &sk.solids[swept].def else { unreachable!() };
        let started = std::time::Instant::now();
        let family = Family::read(sk,*motion as usize)?;
        let twist = solid::constant_twist(&family,[from.value,to.value],1e-9)?;
        let patches = if twist { constant_twist_sheets(sk,swept,epsilon,sagitta)? } else {
            // What matters is what any placement of the sweep carries into the
            // blank: the blank's neighbourhood pulled back through every placement.
            let placed_reach = |p: [f64;3]| recipe.sweeps.iter().filter(|c| c.swept == swept)
                .any(|c| occupancy.reaches(c.pose.point(p)));
            match traced_sheets(sk,swept,epsilon,sagitta,&placed_reach) {
                Ok(patches) => patches,
                Err(e) => { stage(&format!("`{}`: tracer declined ({e}); sectioning the cutter instead",sk.solids[swept].name)); sheets(swept,&inside)?.iter().map(SheetGrid::patch).collect() }
            }
        };
        let tool = solid_of(&solid::static_solid(sk,*source as usize,0)?)?;
        let ends = [family.at(from.value)?,family.at(to.value)?];
        let source_field = solid::SpatialField::read(sk,*source as usize,1e-10)?;
        // A sheet is cropped to what reaches the blank's neighbourhood and one
        // ring of triangles more, so what a crop leaves as a rim lies outside
        // the blank. The neighbourhood is a coarse occupancy of the blank,
        // dilated by two cells.
        let reach = |p: &[f64;3]| occupancy.reaches(*p);
        let placements = recipe.sweeps.iter().filter(|c| c.swept == swept).count();
        let (mut sheet_count,mut sheet_triangles) = (0,0);
        for (k,cut) in recipe.sweeps.iter().filter(|c| c.swept == swept).enumerate() {
            let mut posed: Vec<Patch> = patches.iter().filter_map(|g| g.placed(cut.pose).cropped(&reach)).collect();
            let near = 4.*sagitta*scale+COLUMN_SPACING/20.;
            let ribbons = seam_ribbons(&posed,2.*near);
            for (i,g) in posed.iter().enumerate() {
                let (c0,c1) = g.column_range();
                detail(&format!("placement {k}: sheet {i} kept columns {c0}..{c1} ({:.6}..{:.6}), {} vertices, {} triangles",g.times[c0 as usize],g.times[c1 as usize],g.vertices.len(),g.triangles.len()));
            }
            detail(&format!("placement {k}: {} seam ribbons of {} triangles",ribbons.len(),ribbons.iter().map(|r| r.triangles.len()).sum::<usize>()));
            posed.extend(ribbons);
            if std::env::var("SOLVENT_SELF_INTERSECT").is_ok() {
                for (i,g) in posed.iter().enumerate() {
                    if let Some((a,b)) = g.self_intersection() {
                        stage(&format!("placement {k}: sheet {i} crosses itself: triangles {a} (columns {:?}) and {b} (columns {:?}) at {:?}",
                            g.triangles[a].map(|v| g.column[v as usize]),g.triangles[b].map(|v| g.column[v as usize]),g.vertices[g.triangles[a][0] as usize]));
                        let (c0,c1) = g.column_range();
                        stage(&format!("  columns {c0}..{c1}, times {:?}",&g.times));
                        let mut cs: Vec<u32> = g.triangles[a].iter().chain(&g.triangles[b]).map(|&v| g.column[v as usize]).collect();
                        cs.sort(); cs.dedup();
                        for c in cs {
                            let vs = g.column_vertices(c);
                            stage(&format!("  column {c}: {} vertices",vs.len()));
                            for v in vs { stage(&format!("    {v}: {:?} n {:?}",g.vertices[v as usize],g.normals[v as usize])); }
                        }
                        for t in [a,b] { stage(&format!("  triangle {t}: {:?}",g.triangles[t])); }
                    }
                }
            }
            // Every rim point of a sheet must be outside the blank, inside an end
            // pose of the tool (removed outright), or on another candidate of the
            // same sweep; otherwise the arrangement would leave the sheet's edge
            // loose inside the material and the cut incomplete.
            let cap_poses: Vec<envelope::Motion> = ends.iter().map(|end| end.then(cut.pose).inverse()).collect();
            // A sheet runs two slabs' worth past each end pose, tangent to the
            // tool there, so its rim lies on the cap to within the slab.
            let in_cap = |p: [f64;3]| -> Result<bool,String> {
                for inverse in &cap_poses {
                    let q = inverse.point(p.map(|v| v/scale));
                    let value = source_field.bounds(q.map(|x| Interval::point(x).unwrap())).map_err(|e| format!("{e:?}"))?;
                    if value.bounds()[0] <= epsilon { return Ok(true); }
                }
                Ok(false)
            };
            let indices: Vec<PatchIndex> = posed.iter().map(|g| g.index(COLUMN_SPACING)).collect();
            for (i,patch) in posed.iter().enumerate() {
                let mut rim: Vec<u32> = patch.boundary_edges().into_iter().flat_map(|(p,q)| [p,q]).collect();
                rim.sort(); rim.dedup();
                let points: Vec<[f64;3]> = rim.iter().map(|&v| patch.vertices[v as usize]).collect();
                let inside_blank = inside(&points)?;
                for ((&v,p),&in_blank) in rim.iter().zip(&points).zip(&inside_blank) {
                    if !in_blank || in_cap(*p)? { continue; }
                    let attached = posed.iter().zip(&indices).enumerate().filter(|(j,_)| *j != i)
                        .any(|(_,(other,index))| other.distance_within(index,*p,near) <= near);
                    if !attached {
                        let (c0,c1) = patch.column_range();
                        let c = patch.column[v as usize];
                        return Err(format!("`{}`: placement {k} of `{}` leaves a sheet's edge loose inside the blank at {p:?} \
                            (sheet {i}, column {c} of {c0}..{c1}, parameter {:.6}), attached to no end pose or other candidate; \
                            its cut would be incomplete",name,sk.solids[swept].name,patch.times[c as usize]));
                    }
                }
            }
            let group_start = slabs.len();
            let posed: Vec<Patch> = posed.iter().flat_map(split_at_folds).collect();
            for patch in &posed {
                let sheet = slab_of(patch,epsilon)?;
                sheet_count += 1; sheet_triangles += sheet.triangle_count();
                slabs.push(sheet);
            }
            groups.push(group_start..slabs.len());
            let mid_inverse = family.at(0.5*(from.value+to.value))?.then(cut.pose).inverse();
            let (field,margin) = (source_field.clone(),0.05/scale);
            removed_by.push(Box::new(move |p: [f64;3]| field.value(mid_inverse.point(p.map(|v| v/scale))) < -margin));
            for end in ends {
                let placed = tool.placed(&cad::placement_matrix(end.then(cut.pose),1.))?;
                caps.push(placed);
            }
            tick(&format!("`{}`: placement {} of {placements}: {} sheets, {} triangles ({:?})",sk.solids[swept].name,k+1,
                posed.len(),slabs[group_start..].iter().map(|s| s.triangle_count()).sum::<usize>(),started.elapsed()));
        }
        stage(&format!("`{}`: {sheet_count} sheets of {sheet_triangles} triangles over {placements} placements ({:?})",
            sk.solids[swept].name,started.elapsed()));
    }
    let started = std::time::Instant::now();
    // Only the caps whose box meets the blank's are worth a Boolean: a roll
    // that carries the cutter clear of the blank at both limits leaves none.
    let bounds = |solid: &Solid| -> Result<([f64;3],[f64;3]),String> {
        let (v,_) = solid.triangles()?;
        Ok(v.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),p| (std::array::from_fn(|k| lo[k].min(p[k])),std::array::from_fn(|k| hi[k].max(p[k])))))
    };
    let (blank_lo,blank_hi) = bounds(&blank_mesh)?;
    let mut near_caps: Vec<&Solid> = Vec::new();
    for cap in &caps {
        let (lo,hi) = bounds(cap)?;
        if (0..3).all(|k| lo[k] <= blank_hi[k] && hi[k] >= blank_lo[k]) { near_caps.push(cap); }
    }
    let body_mesh = if near_caps.is_empty() { Solid::from_triangles(&blank_mesh.triangles()?.0,&blank_mesh.triangles()?.1)? } else {
        blank_mesh.difference(&Solid::batch(&near_caps,true)?)?
    };
    let removed = blank_mesh.volume()-body_mesh.volume();
    // One placement's slabs at a time: a union of every placement's slabs at
    // once is a Boolean over millions of overlapping triangles, where each
    // placement's union is small and the remainder only grows by its cut.
    let (mut walls,mut remainder): (Vec<Solid>,Solid) = (Vec::new(),body_mesh);
    for (g,group) in groups.iter().enumerate() {
        if group.is_empty() { continue; }
        let lap = std::time::Instant::now();
        let union = Solid::batch(&slabs[group.clone()].iter().collect::<Vec<_>>(),true)?;
        let united = lap.elapsed();
        if let Some(voxel) = std::env::var("SOLVENT_FIND_LEAK").ok().and_then(|v| v.parse::<f64>().ok()) {
            let mut material = MaterialField::read(sk,body,1e-10)?.evaluator(4096);
            let mut is_material = |p: [f64;3]| -> Result<bool,String> {
                let d = 0.3/scale;
                let probe = material.probe(p.map(|x| Interval::point(x/scale).unwrap()),[1.,0.,0.],d,
                    Options {value_tolerance:d/4.,max_evaluations:40000}).map_err(|e| format!("{e:?}"))?;
                Ok(matches!(probe.state,ProbeState::InteriorBall))
            };
            let mut blockers: Vec<&Solid> = vec![&union,&blank_mesh];
            blockers.extend(near_caps.iter().copied());
            find_leak(&blockers,&*removed_by[g],&mut is_material,voxel)?;
        }
        let (w,r) = remainder.split(&union)?;
        detail(&format!("placement {}: {} slabs of {} triangles united to {} ({united:?}), split of {} triangles ({:?})",g+1,group.len(),
            slabs[group.clone()].iter().map(|s| s.triangle_count()).sum::<usize>(),union.triangle_count(),remainder.triangle_count(),lap.elapsed()-united));
        walls.push(w); remainder = r;
        tick(&format!("split by placement {} of {}: remainder {} triangles ({:?})",g+1,groups.len(),remainder.triangle_count(),started.elapsed()));
    }
    let cells = remainder.components()?;
    tick(&format!("{} cells ({:?})",cells.len(),started.elapsed()));
    // each placement's walls are their own pieces; a union across placements
    // would only merge slabs that overlap, which the field judges alike
    let mut wall_pieces: Vec<Solid> = Vec::new();
    for w in &walls { wall_pieces.extend(w.components()?); }
    stage(&format!("caps removed {:.6} mm³; {} cells and {} wall pieces ({:?})",removed*scale.powi(3),cells.len(),wall_pieces.len(),started.elapsed()));
    let started = std::time::Instant::now();
    let mut material = MaterialField::read(sk,body,1e-10)?.evaluator(4096);
    let mut kept: Vec<Solid> = Vec::new();
    let (mut removed_cells,mut hidden_walls) = (0,0);
    let (mut slivers,mut sliver_volume) = (0,0.);
    for cell in cells {
        // A sliver between two nearly coincident candidates holds no material
        // worth a probe and offers nowhere to put one.
        if cell.volume() < 1e-4/scale.powi(3) { slivers += 1; sliver_volume += cell.volume(); continue; }
        // A point deep inside the cell: a quarter of its mean thickness, but at
        // least past the tessellation sagitta and the slab. The swept field
        // resolves far faster away from the envelope than beside it.
        let thickness = 2.*cell.volume()/cell.area().max(f64::MIN_POSITIVE);
        let depth = (0.25*thickness).clamp(3.*sagitta+3.*epsilon,0.5/scale);
        let p = interior_point(&cell,depth)?;
        let distance = (depth/3.).max(1.5*epsilon);
        let probe = material.probe(p.map(|x| Interval::point(x).unwrap()),[1.,0.,0.],distance,
            Options {value_tolerance:distance/4.,max_evaluations:40000}).map_err(|e| format!("{e:?}"))?;
        match probe.state {
            ProbeState::InteriorBall => kept.push(cell),
            ProbeState::ExteriorBall => removed_cells += 1,
            state => return Err(format!("the material at {p:?}, inside a cell of {:.6} mm³, is {state:?}",cell.volume()*scale.powi(3))),
        }
    }
    let cells_time = started.elapsed();
    let hidden_depth = (0.02/scale).max(3.*sagitta);
    for wall in wall_pieces {
        let p = interior_point(&wall,0.5*epsilon)?;
        // A hidden wall is well inside material; an exposed one reads near zero.
        // Only that distinction is needed, so the field is refined coarsely and
        // may stop as soon as a sweep is clear of the band.
        let bounds = material.bounds_outside(p.map(|x| Interval::point(x).unwrap()),
            Interval::new(-hidden_depth,hidden_depth).unwrap(),
            Options {value_tolerance:hidden_depth/2.,max_evaluations:40000}).map_err(|e| format!("{e:?}"))?;
        if bounds.value.bounds()[1] < -hidden_depth { kept.push(wall); hidden_walls += 1; }
    }
    if slivers > 0 { stage(&format!("dropped {slivers} sliver cells of {:.9} mm³",sliver_volume*scale.powi(3))); }
    stage(&format!("classified {} material cells, {removed_cells} removed cells ({cells_time:?}), {hidden_walls} hidden walls ({:?})",
        kept.len(),started.elapsed()));
    if kept.is_empty() { return Err("no cell of the blank is material".into()); }
    let started = std::time::Instant::now();
    let part = Solid::batch(&kept.iter().collect::<Vec<_>>(),true)?;
    stage(&format!("united the material: {:.6} mm³, {} triangles ({:?})",part.volume()*scale.powi(3),part.triangle_count(),started.elapsed()));
    // STL carries float32 coordinates, and the Boolean between nearly
    // coincident slabs leaves features far below them: edges a few nanometres
    // long, needles whose altitude is under a float32 step. Encoded, those are
    // edges of no length and triangles of no area, which the kernel's own
    // simplification takes out while keeping the mesh manifold; so the mesh
    // is rounded to float32 first and simplified after, and what comes back
    // is what the file will hold.
    let (vertices,triangles) = part.triangles()?;
    let rounded: Vec<[f64;3]> = vertices.iter().map(|v| v.map(|x| (x as f32) as f64)).collect();
    let encoded = Solid::from_triangles(&rounded,&triangles)?.simplified(0.)?;
    stage(&format!("encoded for float32: {:.6} mm³, {} triangles",encoded.volume()*scale.powi(3),encoded.triangle_count()));
    encoded.triangles()
}

/// The mesh prepared for float32 encoding: STL carries no vertex identity, so
/// two vertices nearer than float32 resolves are one vertex once written, and a
/// triangle collinear once written is no triangle. The Boolean between nearly
/// coincident slabs leaves both, edges a few nanometres long and needles whose
/// altitude is under a float32 step. Each is removed by a move that keeps a
/// closed manifold closed: an edge shorter than the weld is collapsed only
/// under the link condition (its ends share exactly the two vertices across
/// the edge's two triangles), a needle is dropped and its neighbour across the
/// long edge split at the needle's middle vertex (a T-junction resolved), and
/// two vertices the encoding identifies that no edge joins are nudged apart
/// along their own fans' normals, since a pinch is what the encoding would
/// otherwise make of them. Welding by proximity alone folded strips into edges
/// used three times.
fn merged(vertices: &[[f64;3]],triangles: &[[u32;3]]) -> (Vec<[f64;3]>,Vec<[u32;3]>) {
    let magnitude = vertices.iter().flatten().fold(1_f64,|m,x| m.max(x.abs()));
    let weld = 4.*(magnitude as f32).abs() as f64*f32::EPSILON as f64;
    let mut points: Vec<[f64;3]> = vertices.to_vec();
    let mut kept: Vec<[u32;3]> = triangles.to_vec();
    // Short edges, collapsed under the link condition. One pass collapses
    // edges whose neighbourhoods are untouched by an earlier collapse of the
    // same pass, and passes repeat until nothing collapses.
    loop {
        let n = points.len();
        let mut neighbours: Vec<Vec<u32>> = vec![Vec::new();n];
        let mut incident: Vec<Vec<usize>> = vec![Vec::new();n];
        for (i,t) in kept.iter().enumerate() { for k in 0..3 {
            let (a,b) = (t[k],t[(k+1)%3]);
            neighbours[a as usize].push(b); neighbours[b as usize].push(a); incident[a as usize].push(i);
        } }
        for list in neighbours.iter_mut() { list.sort(); list.dedup(); }
        let mut remap: Vec<u32> = (0..n as u32).collect();
        let mut dead = vec![false;kept.len()];
        let mut touched = vec![false;n];
        let mut collapsed = false;
        for a in 0..n {
            for &b in &neighbours[a] {
                let b = b as usize;
                if b <= a || touched[a] || touched[b] || norm(sub(points[a],points[b])) >= weld { continue; }
                let shared: Vec<usize> = incident[a].iter().copied().filter(|&i| kept[i].contains(&(b as u32))).collect();
                let common: Vec<u32> = neighbours[a].iter().copied().filter(|v| neighbours[b].contains(v)).collect();
                let across = |i: usize| kept[i].iter().copied().find(|&v| v != a as u32 && v != b as u32).unwrap();
                if shared.len() != 2 || common.len() != 2 || !shared.iter().all(|&i| common.contains(&across(i))) { continue; }
                for &i in &shared { dead[i] = true; }
                remap[b] = a as u32;
                collapsed = true;
                for &v in neighbours[a].iter().chain(&neighbours[b]) { touched[v as usize] = true; }
                touched[a] = true; touched[b] = true;
            }
        }
        if !collapsed { break; }
        kept = kept.iter().enumerate().filter(|(i,_)| !dead[*i]).map(|(_,t)| t.map(|v| remap[v as usize])).collect();
    }
    // Needles: three vertices collinear once encoded, the middle one on the
    // long edge, a T-junction with the neighbour across that edge.
    for _ in 0..8 {
        let encoded = |v: u32| points[v as usize].map(|x| (x as f32) as f64);
        let needle = |t: &[u32;3]| -> Option<(u32,u32,u32)> {
            let [a,b,c] = t.map(encoded);
            if !mesh::degenerate(a,b,c) { return None; }
            let mid = (0..3).max_by(|&i,&j| { let d = |k: usize| norm(sub([a,b,c][(k+1)%3],[a,b,c][(k+2)%3])); d(i).total_cmp(&d(j)) }).unwrap();
            Some((t[(mid+1)%3],t[mid],t[(mid+2)%3]))
        };
        let needles: Vec<(usize,(u32,u32,u32))> = kept.iter().enumerate().filter_map(|(i,t)| needle(t).map(|n| (i,n))).collect();
        if needles.is_empty() { break; }
        let mut drop = vec![false;kept.len()];
        let mut added: Vec<[u32;3]> = Vec::new();
        for (i,(p,m,q)) in needles {
            if drop[i] { continue; }
            // the neighbour across the long edge q->p (the needle runs p->m->q on the other side)
            let Some(j) = kept.iter().enumerate().position(|(j,t)| j != i && !drop[j] && (0..3).any(|k| t[k] == q && t[(k+1)%3] == p)) else { continue };
            let t = kept[j];
            let k = (0..3).find(|&k| t[k] == q && t[(k+1)%3] == p).unwrap();
            let x = t[(k+2)%3];
            drop[i] = true; drop[j] = true;
            added.push([q,m,x]); added.push([m,p,x]);
        }
        kept = kept.into_iter().enumerate().filter(|(i,_)| !drop[*i]).map(|(_,t)| t).chain(added).collect();
    }
    // Vertices the encoding identifies that no edge joins: each but the first
    // moves a few float32 steps along its own fan's normal.
    let mut used = vec![false;points.len()];
    for t in &kept { for &v in t { used[v as usize] = true; } }
    let mut groups: std::collections::BTreeMap<[u32;3],Vec<u32>> = Default::default();
    for (i,p) in points.iter().enumerate() { if used[i] { groups.entry(p.map(|x| (x as f32).to_bits())).or_default().push(i as u32); } }
    let mut fan_normal: Vec<[f64;3]> = vec![[0.;3];points.len()];
    for t in &kept {
        let [a,b,c] = t.map(|v| points[v as usize]);
        let n = cross(sub(b,a),sub(c,a));
        for &v in t { for k in 0..3 { fan_normal[v as usize][k] += n[k]; } }
    }
    for group in groups.values().filter(|g| g.len() > 1) {
        for (j,&v) in group.iter().enumerate().skip(1) {
            let (n,p) = (fan_normal[v as usize],points[v as usize]);
            let len = norm(n); if len == 0. { continue; }
            points[v as usize] = std::array::from_fn(|k| {
                let step = ((p[k] as f32).abs().max(1e-30) as f64)*f32::EPSILON as f64*2.*j as f64;
                p[k]+step*n[k]/len
            });
        }
    }
    // unused vertices dropped
    let mut compact: Vec<u32> = vec![u32::MAX;points.len()];
    let mut vertices_out: Vec<[f64;3]> = Vec::new();
    for (i,&u) in used.iter().enumerate() { if u { compact[i] = vertices_out.len() as u32; vertices_out.push(points[i]); } }
    (vertices_out,kept.iter().map(|t| t.map(|v| compact[v as usize])).collect())
}

/// Binary STL of the indexed triangles through the core's checked writer, with
/// the encoded shell topology verified. STL carries no vertex identity, so
/// vertices the encoding identifies are merged first and the triangles that
/// collapse under the merge (a sliver between two coincident vertices) dropped;
/// a genuine pinch, three faces on one segment, still refuses.
pub fn stl(vertices: &[[f64;3]],triangles: &[[u32;3]],name: &str) -> Result<Vec<u8>,String> {
    let (vertices,triangles) = merged(vertices,triangles);
    let (vertices,triangles) = (&vertices[..],&triangles[..]);
    let bytes = mesh::indexed_stl(vertices,triangles,name)?;
    if let Err(e) = mesh::stl_shells(&bytes) {
        // Distinct vertices that float32 identifies, and the shortest edges.
        let mut by_key: std::collections::HashMap<[u32;3],Vec<usize>> = Default::default();
        for (i,v) in vertices.iter().enumerate() { by_key.entry(v.map(|x| (x as f32).to_bits())).or_default().push(i); }
        let merged: Vec<_> = by_key.values().filter(|v| v.len() > 1).take(3)
            .map(|v| format!("{:?}",v.iter().map(|&i| vertices[i]).collect::<Vec<_>>())).collect();
        let mut shortest = f64::INFINITY;
        for t in triangles { for k in 0..3 {
            let (a,b) = (vertices[t[k] as usize],vertices[t[(k+1)%3] as usize]);
            shortest = shortest.min(((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt());
        } }
        let mut uses: std::collections::HashMap<(u32,u32),Vec<usize>> = Default::default();
        for (i,t) in triangles.iter().enumerate() { for k in 0..3 {
            let (a,b) = (t[k],t[(k+1)%3]); uses.entry((a.min(b),a.max(b))).or_default().push(i);
        } }
        let mut bad: Vec<String> = uses.iter().filter(|(_,v)| v.len() != 2).take(4).map(|((a,b),v)| format!("edge {:?}-{:?} used by {}",
            vertices[*a as usize],vertices[*b as usize],v.iter().map(|&i| format!("{:?}",triangles[i].map(|j| vertices[j as usize]))).collect::<Vec<_>>().join(", "))).collect();
        bad.sort();
        // vertices whose incident triangles fall into more than one fan: pinches
        let mut incident: Vec<Vec<usize>> = vec![Vec::new();vertices.len()];
        for (i,t) in triangles.iter().enumerate() { for &v in t { incident[v as usize].push(i); } }
        let mut pinches: Vec<String> = Vec::new();
        for (v,ts) in incident.iter().enumerate() {
            if ts.len() < 2 { continue; }
            // fans: triangles linked when they share an edge at v
            let mut group: Vec<usize> = (0..ts.len()).collect();
            let other = |i: usize| -> Vec<u32> { triangles[ts[i]].iter().copied().filter(|&w| w != v as u32).collect() };
            for a in 0..ts.len() { for b in a+1..ts.len() {
                if other(a).iter().any(|w| other(b).contains(w)) { let (ga,gb) = (group[a],group[b]); for g in group.iter_mut() { if *g == gb { *g = ga; } } }
            } }
            let mut fans: Vec<usize> = group.clone(); fans.sort(); fans.dedup();
            if fans.len() > 1 && pinches.len() < 4 { pinches.push(format!("vertex {:?} with {} fans over {} triangles",vertices[v],fans.len(),ts.len())); }
        }
        return Err(format!("mesh STL validation failed: {e}; {} float32-identified vertex groups, e.g. {}; shortest edge {shortest:e}; {}; {}",
            by_key.values().filter(|v| v.len() > 1).count(),merged.join(" | "),bad.join("\n"),pinches.join("; ")));
    }
    Ok(bytes)
}
