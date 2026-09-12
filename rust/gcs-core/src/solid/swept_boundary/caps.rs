//! The caps: the tool's own boundary at the two end poses, cut along the
//! sheets' end columns (the contact curves on the tool at that instant) and
//! kept where the tool recedes from the sweep (at the start) or advances into
//! it (at the end). The field cannot make this cut: the sweep is only
//! quadratically deep just past a contact curve, so a facet a chord past it
//! reads boundary to any tolerance the judge can afford, while the tracer's
//! column is the curve itself. The cut is geometric: every column point is
//! put into the tool's mesh as a vertex (on the facet, edge or vertex it
//! lies on), consecutive points are joined through the mesh by splitting
//! every facet edge the chord between them crosses, and the chain of edges
//! so made separates the mesh into components. Each component is judged by
//! its most decisively signed facet's normal velocity; one signed neither way
//! (a face parallel to a translation, a face square to a turn's axis: grazing
//! the sweep) is kept, and where it doubles a sheet the planar union or the
//! overlap drop takes one copy. A cap's rim is then the column, chord for
//! chord, save the crossing vertices the cut put on the chords, which the
//! stitch's tolerant split puts into the sheet's edges too.
use crate::model::{Sketch,SolidDef};
use crate::motion::Family;
use crate::solid::{SweepPatch,indexed_faces,static_solid_at_unit};

type V3 = [f64;3];

/// Which end of the roll a cap stands at.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum End { From, To }

use crate::space::{Grid,Region,add,closest_on_triangle,cross,dot,norm,normalised,scale,sub};

/// How a vertex of a cut mesh came to be. A defect in a cap is nearly always a vertex that
/// should not be where it is, and which operation put it there is the question that names the
/// code to look at: the twins the tilted cylinder's whisker is made of are all `Crossed`, minted
/// a fraction of a snap from one another while a chord walked from one column point to the next.
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub enum Origin {
    /// The tool's own mesh, as it was read.
    Tool,
    /// A midpoint `refine` put in, so no facet edge is longer than the column spacing.
    Refined,
    /// A point of a contact curve, put into the mesh where the curve runs.
    Inserted,
    /// Where the chord between two such points crossed a facet's far edge.
    Crossed,
}

/// A triangle mesh being cut along polylines on it.
pub struct CutMesh {
    pub vertices: Vec<V3>,
    pub triangles: Vec<[u32;3]>,
    /// The tool face each facet lies on; the pieces of a split facet keep it.
    pub faces: Vec<u32>,
    /// How each vertex came to be, beside `vertices`.
    pub origin: Vec<Origin>,
    /// Vertex pairs the cuts run along.
    pub cuts: std::collections::BTreeSet<(u32,u32)>,
    /// The vertices a cut runs through. A later point within the snap of one
    /// takes it where it stands: moved onto that point, it would drag the
    /// chain already walking through it off the curve that chain was cut
    /// along. The point is then at most a snap away, which is well inside the
    /// tolerance the stitch splits an edge at.
    pinned: std::collections::BTreeSet<u32>,
    /// Snap: a point this close to a vertex is that vertex (which moves to
    /// it), and a crossing this close to an edge's end takes the end, so no
    /// sliver thinner than this is ever made.
    pub vertex_tolerance: f64,
    /// How far off a facet's plane a point on the surface may lie.
    pub sagitta: f64,
    /// The facets on each vertex, each once, in index order.
    incident: Vec<Vec<u32>>,
    /// The facets filed under every cell their boxes (grown by `locate`'s reach) meet, made at
    /// the first cut.
    index: Option<Grid>,
}

/// Put `t` in an ascending list of distinct facets.
fn attach(list: &mut Vec<u32>,t: usize) { let t = t as u32; if let Err(k) = list.binary_search(&t) { list.insert(k,t); } }
fn detach(list: &mut Vec<u32>,t: usize) { if let Ok(k) = list.binary_search(&(t as u32)) { list.remove(k); } }

#[derive(Clone,Copy,Debug)]
enum Place { Vertex(u32),Edge(u32,u32),Face(usize) }

impl CutMesh {
    pub fn new(vertices: Vec<V3>,triangles: Vec<[u32;3]>,faces: Vec<u32>,vertex_tolerance: f64,sagitta: f64) -> CutMesh {
        let mut incident = vec![Vec::new();vertices.len()];
        for (t,tri) in triangles.iter().enumerate() { for &v in tri { attach(&mut incident[v as usize],t); } }
        let origin = vec![Origin::Tool;vertices.len()];
        CutMesh {vertices,triangles,faces,origin,cuts:Default::default(),pinned:Default::default(),vertex_tolerance,sagitta,incident,index:None}
    }

    /// How far from a facet `locate` looks for a point.
    fn reach(&self) -> f64 { 2.*self.sagitta }

    fn file(&mut self,t: usize) {
        let reach = self.reach();
        let [a,b,c] = self.triangles[t].map(|v| self.vertices[v as usize]);
        let lo: V3 = std::array::from_fn(|k| a[k].min(b[k]).min(c[k])-reach);
        let hi: V3 = std::array::from_fn(|k| a[k].max(b[k]).max(c[k])+reach);
        if let Some(grid) = &mut self.index { grid.insert_box(lo,hi,t as u32); }
    }

    /// File every facet by place, in cells the size of a facet's box on average.
    fn build_index(&mut self) {
        let n = self.triangles.len().max(1) as f64;
        let size = self.triangles.iter().map(|t| {
            let [a,b,c] = t.map(|v| self.vertices[v as usize]);
            (0..3).map(|k| a[k].max(b[k]).max(c[k])-a[k].min(b[k]).min(c[k])).fold(0.,f64::max)
        }).sum::<f64>()/n+2.*self.reach();
        self.index = Some(Grid::new(size.max(f64::MIN_POSITIVE)));
        for t in 0..self.triangles.len() { self.file(t); }
    }

    fn push_vertex(&mut self,p: V3,origin: Origin) -> u32 {
        self.vertices.push(p); self.origin.push(origin); self.incident.push(Vec::new());
        (self.vertices.len()-1) as u32
    }

    /// A facet's corners changed to `now`: the lists of the vertices it left and joined follow.
    fn set_triangle(&mut self,t: usize,now: [u32;3]) {
        let before = self.triangles[t];
        self.triangles[t] = now;
        for v in before { if !now.contains(&v) { detach(&mut self.incident[v as usize],t); } }
        for v in now { attach(&mut self.incident[v as usize],t); }
    }

    fn push_triangle(&mut self,tri: [u32;3],face: u32) {
        let t = self.triangles.len();
        self.triangles.push(tri); self.faces.push(face);
        for v in tri { attach(&mut self.incident[v as usize],t); }
        self.file(t);
    }

    fn facet_normal(&self,t: usize) -> Option<V3> { let [a,b,c] = self.triangles[t].map(|v| self.vertices[v as usize]); normalised(cross(sub(b,a),sub(c,a))) }

    /// The summed normal of the facets at a vertex.
    fn vertex_normal(&self,v: u32) -> V3 {
        let mut n = [0.;3];
        for &t in &self.incident[v as usize] { if let Some(f) = self.facet_normal(t as usize) { n = add(n,f); } }
        n
    }

    /// Where a point of the surface lies on the mesh: on a vertex, on an
    /// edge, or in a facet, by the nearest point of the nearest facet. The
    /// perpendicular projections of surface points onto a convex facet
    /// mesh leave gaps at the edges (a point over a rim edge projects
    /// short of both facets), so nearness, not containment, decides.
    fn locate(&self,p: V3) -> Option<Place> {
        let mut best: Option<(f64,Place)> = None;
        // the facets filed under p's cell, in index order (a facet whose vertex moved is filed
        // again, so twice)
        let mut candidates: Vec<u32> = match &self.index {
            Some(grid) => grid.at(grid.key(p)).to_vec(),
            None => (0..self.triangles.len() as u32).collect(),
        };
        candidates.sort_unstable(); candidates.dedup();
        for t in candidates.into_iter().map(|t| t as usize) {
            let tri = &self.triangles[t];
            let [a,b,c] = tri.map(|v| self.vertices[v as usize]);
            let lo: V3 = std::array::from_fn(|k| a[k].min(b[k]).min(c[k])-2.*self.sagitta);
            let hi: V3 = std::array::from_fn(|k| a[k].max(b[k]).max(c[k])+2.*self.sagitta);
            if (0..3).any(|k| p[k] < lo[k] || p[k] > hi[k]) { continue; }
            let (q,region) = closest_on_triangle(p,a,b,c);
            let dist = norm(sub(p,q));
            if dist > 2.*self.sagitta || best.is_some_and(|(x,_)| dist >= x) { continue; }
            // the nearest corner within the snap: the first found would be decided by which of
            // the facets on a shared edge the point happened to land nearest
            let corner = [a,b,c].iter().enumerate().map(|(k,x)| (norm(sub(q,*x)),k)).filter(|(d,_)| *d <= self.vertex_tolerance).min_by(|x,y| x.0.total_cmp(&y.0));
            let place = if let Some((_,k)) = corner { Place::Vertex(tri[k]) }
                else {
                    // on an edge when the nearest point is, or lies within the
                    // vertex tolerance of one
                    let edge = |x: V3,y: V3| -> f64 { let d = sub(y,x); let l = dot(d,d); let f = if l > 0. { (dot(sub(q,x),d)/l).clamp(0.,1.) } else { 0. }; norm(sub(q,add(x,scale(d,f)))) };
                    let ea = edge(a,b); let eb = edge(b,c); let ec = edge(c,a);
                    let m = ea.min(eb).min(ec);
                    if region == Region::Face && m > self.vertex_tolerance { Place::Face(t) }
                    else if m == ea { Place::Edge(tri[0],tri[1]) } else if m == eb { Place::Edge(tri[1],tri[2]) } else { Place::Edge(tri[2],tri[0]) }
                };
            best = Some((dist,place));
        }
        best.map(|(_,p)| p)
    }

    /// A new vertex at `p` on the edge `a`-`b`, both facets on it split.
    fn split_edge(&mut self,a: u32,b: u32,p: V3,origin: Origin) -> u32 {
        let v = self.push_vertex(p,origin);
        // a cut running along this edge runs along its two halves now. Left
        // naming the edge that is gone, it separates nothing and `components`
        // floods straight through the seam.
        if self.cuts.remove(&(a.min(b),a.max(b))) {
            self.cuts.insert((a.min(v),a.max(v)));
            self.cuts.insert((b.min(v),b.max(v)));
            self.pinned.insert(v);
        }
        let mut fresh = Vec::new();
        for t in self.incident[a as usize].clone().into_iter().map(|t| t as usize) {
            let tri = self.triangles[t];
            let Some(k) = (0..3).find(|&k| (tri[k] == a && tri[(k+1)%3] == b) || (tri[k] == b && tri[(k+1)%3] == a)) else { continue };
            let (x,y,c) = (tri[k],tri[(k+1)%3],tri[(k+2)%3]);
            self.set_triangle(t,[x,v,c]);
            fresh.push(([v,y,c],self.faces[t]));
        }
        for (tri,face) in fresh { self.push_triangle(tri,face); }
        v
    }

    /// A new vertex at `p` inside facet `t`, the facet split in three.
    fn split_face(&mut self,t: usize,p: V3,origin: Origin) -> u32 {
        let v = self.push_vertex(p,origin);
        let [a,b,c] = self.triangles[t];
        self.set_triangle(t,[a,b,v]);
        let face = self.faces[t];
        self.push_triangle([b,c,v],face);
        self.push_triangle([c,a,v],face);
        v
    }

    /// The vertex a surface point is or becomes: a vertex within the snap
    /// of it moves onto it, so the seam is the sheet's own point exactly.
    fn insert(&mut self,p: V3) -> Result<u32,String> {
        let v = match self.locate(p).ok_or_else(|| format!("column point {p:?} is off the tool's mesh"))? {
            Place::Vertex(v) => {
                if !self.pinned.contains(&v) {
                    self.vertices[v as usize] = p;
                    // its facets moved with it: filed again where they now reach
                    for t in self.incident[v as usize].clone() { self.file(t as usize); }
                }
                v
            }
            Place::Edge(a,b) => self.split_edge(a,b,p,Origin::Inserted),
            Place::Face(t) => self.split_face(t,p,Origin::Inserted),
        };
        self.pinned.insert(v);
        Ok(v)
    }

    fn share_facet(&self,u: u32,w: u32) -> bool { self.incident[u as usize].iter().any(|&t| self.triangles[t as usize].contains(&w)) }

    /// Join two vertices by a chain of edges along the chord between them,
    /// walked facet by facet: from the vertex reached so far, the facet on it
    /// whose far edge the chord's projection (in the plane of the two ends'
    /// normals) crosses ahead is the next; the crossing splits that edge, or
    /// takes its end when within the snap of it. Only facets the walk enters
    /// are looked at, so a face round a corner, folded into the projection,
    /// cannot offer a crossing of its own. Records every chain edge as a cut.
    fn connect(&mut self,u: u32,w: u32,nu: V3,nw: V3) -> Result<(),String> {
        if u == w { return Ok(()); }
        if self.share_facet(u,w) { self.cuts.insert((u.min(w),u.max(w))); return Ok(()); }
        let (pu,pw) = (self.vertices[u as usize],self.vertices[w as usize]);
        // the chord's own normals give the projection: the mesh's at a
        // column point on a crease of the tool lean into the other face
        let n = normalised(add(normalised(nu).unwrap_or([0.;3]),normalised(nw).unwrap_or([0.;3]))).ok_or("no normal at a column point")?;
        let d = sub(pw,pu);
        let du = normalised(sub(d,scale(n,dot(d,n)))).ok_or("column points coincide in projection")?;
        let dv = cross(n,du);
        let to2 = |p: V3| -> (f64,f64) { let r = sub(p,pu); (dot(r,du),dot(r,dv)) };
        let length = to2(pw).0;
        let mut chain = vec![u];
        let mut current = u;
        let mut along = 0_f64;
        for _ in 0..self.triangles.len()+1 {
            // the facets on the current vertex, the one holding the far end first
            let here: Vec<usize> = self.incident[current as usize].iter().map(|&t| t as usize).collect();
            if here.iter().any(|&t| self.triangles[t].contains(&w)) { chain.push(w); break; }
            let mut step: Option<(f64,u32,u32,f64)> = None; // t along the chord, edge, s along it
            for &t in &here {
                // a facet the projection sees edge-on (a face square to the ends' normals, its
                // edge along the chord) offers no crossing: its far edge projects onto the
                // chord's own line, where a crossing is decided by rounding
                if self.facet_normal(t).is_none_or(|m| dot(m,n).abs() <= 1e-6) { continue; }
                let tri = self.triangles[t];
                let k = (0..3).find(|&k| tri[k] == current).unwrap();
                let (a,b) = (tri[(k+1)%3],tri[(k+2)%3]);
                let (pa,pb) = (to2(self.vertices[a as usize]),to2(self.vertices[b as usize]));
                let (ex,ey) = (pb.0-pa.0,pb.1-pa.1);
                if ey.abs() <= 1e-15*(ex.abs()+ey.abs()).max(f64::MIN_POSITIVE) { continue; }
                let s = -pa.1/ey;
                if s < -1e-9 || s > 1.+1e-9 { continue; }
                let x = pa.0+s*ex;
                let t = x/length;
                if t <= along+1e-9 { continue; }
                if step.map_or(true,|(t0,_,_,_)| t < t0) { step = Some((t,a,b,s.clamp(0.,1.))); }
            }
            let Some((t,a,b,s)) = step else {
                let facets: Vec<_> = here.iter().map(|&t| self.triangles[t].map(|v| { let p = self.vertices[v as usize]; (p,to2(p)) })).collect();
                return Err(format!("the cut from {pu:?} to {pw:?} leaves the tool's mesh at {:?}; normals {nu:?} {nw:?} plane {n:?}; walked {:?}; facets here {facets:?}",self.vertices[current as usize],chain.iter().map(|&v| self.vertices[v as usize]).collect::<Vec<_>>()))
            };
            let (pa,pb) = (self.vertices[a as usize],self.vertices[b as usize]);
            let p = add(pa,scale(sub(pb,pa),s));
            // the nearer end within the snap, or a new vertex: taking the
            // farther of two near ends walked off the plane onto a vertex
            // beside the true one, with nothing ahead of it
            let (da,db) = (norm(sub(p,pa)),norm(sub(p,pb)));
            let next = if da.min(db) <= self.vertex_tolerance { if da <= db { a } else { b } } else { self.split_edge(a,b,p,Origin::Crossed) };
            if next == w { chain.push(w); break; }
            chain.push(next); current = next; along = t;
        }
        if chain.last() != Some(&w) { return Err(format!("the cut from {pu:?} to {pw:?} does not reach its end")); }
        for k in 0..chain.len()-1 {
            let (a,b) = (chain[k],chain[k+1]);
            if !self.share_facet(a,b) { return Err(format!("the cut from {pu:?} to {pw:?} does not follow the tool's mesh between {:?} and {:?}",self.vertices[a as usize],self.vertices[b as usize])); }
            self.cuts.insert((a.min(b),a.max(b)));
            self.pinned.insert(a); self.pinned.insert(b);
        }
        Ok(())
    }

    /// Split every edge longer than `longest` at its midpoint, longest
    /// first, until none is: a planar face is two facets however large, and
    /// a crease crossing it would be invisible to the labels at its corners.
    pub fn refine(&mut self,longest: f64) {
        loop {
            let mut worst: Option<(f64,u32,u32)> = None;
            for t in &self.triangles { for k in 0..3 {
                let (a,b) = (t[k],t[(k+1)%3]);
                let l = norm(sub(self.vertices[a as usize],self.vertices[b as usize]));
                if l > longest && worst.map_or(true,|(w,_,_)| l > w) { worst = Some((l,a.min(b),a.max(b))); }
            } }
            let Some((_,a,b)) = worst else { break };
            let mid = scale(add(self.vertices[a as usize],self.vertices[b as usize]),0.5);
            self.split_edge(a,b,mid,Origin::Refined);
        }
    }

    /// Cut along a polyline on the surface.
    pub fn cut_along(&mut self,line: &[(V3,V3)],closed: bool) -> Result<(),String> {
        if self.index.is_none() { self.build_index(); }
        let ids: Vec<u32> = line.iter().map(|(p,_)| self.insert(*p)).collect::<Result<_,_>>()?;
        let mut normals: Vec<V3> = line.iter().map(|(_,n)| *n).collect();
        // a point given no normal takes the mesh's
        for (k,n) in normals.iter_mut().enumerate() { if normalised(*n).is_none() { *n = self.vertex_normal(ids[k]); } }
        for k in 0..ids.len().saturating_sub(1) { self.connect(ids[k],ids[k+1],normals[k],normals[k+1])?; }
        if closed && ids.len() > 2 { self.connect(ids[ids.len()-1],ids[0],normals[ids.len()-1],normals[0])?; }
        Ok(())
    }

    /// The component of each facet, joined across every edge that is not a cut.
    pub fn components(&self) -> Vec<usize> {
        let mut parent: Vec<usize> = (0..self.triangles.len()).collect();
        fn find(parent: &mut Vec<usize>,i: usize) -> usize { let mut r = i; while parent[r] != r { r = parent[r]; } let mut j = i; while parent[j] != r { let next = parent[j]; parent[j] = r; j = next; } r }
        // every edge's facets, the edges in order and each edge's facets in index order
        let mut by_edge: Vec<((u32,u32),u32)> = Vec::with_capacity(3*self.triangles.len());
        for (i,t) in self.triangles.iter().enumerate() { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); by_edge.push(((a.min(b),a.max(b)),i as u32)); } }
        by_edge.sort_unstable();
        let mut start = 0;
        while start < by_edge.len() {
            let e = by_edge[start].0;
            let end = start+by_edge[start..].partition_point(|x| x.0 == e);
            if !self.cuts.contains(&e) {
                for w in by_edge[start..end].windows(2) { let (ri,rj) = (find(&mut parent,w[0].1 as usize),find(&mut parent,w[1].1 as usize)); if ri != rj { parent[ri] = rj; } }
            }
            start = end;
        }
        (0..self.triangles.len()).map(|i| find(&mut parent,i)).collect()
    }
}

/// One component of a cut cap: how many facets, its most decisive relative
/// normal velocity (signed, negative receding), and whether the cap kept it.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct CapComponent { pub facets: usize,pub extreme: f64,pub kept: bool }

/// The tool's boundary at one end of the roll, cut along the sheets' end columns and reduced
/// to what bounds the sweep there.
#[derive(Clone,Debug)]
pub struct Cap {
    pub end: End,
    /// The kept facets, their vertices with the tool's summed facet normals.
    pub patch: SweepPatch,
    /// Every component the cut left, kept or not.
    pub components: Vec<CapComponent>,
    /// Per vertex of `patch`: whether it lies on an edge of the tool (its facets lie on more
    /// than one of the tool's faces), where its own label cannot say which face it speaks for.
    pub tool_edges: Vec<bool>,
    /// Per vertex of `patch`: how the cut mesh came to have it.
    pub origin: Vec<Origin>,
}

/// The tool's boundary at each end pose, cut along the end columns of
/// `sheets` and reduced to the components that recede from the sweep (at
/// the start) or advance into it (at the end), plus every grazing
/// component. `sagitta` is the construction's chord error, which cuts the
/// tool's mesh; `snap` is how near a column point or a crossing must come to
/// a mesh vertex to be it (a quarter of the sagitta serves: what it moves is
/// well within the certificate's probe, and no sliver thinner than it is
/// left for the stitch); no facet edge is left longer than `longest` (the
/// column spacing serves), so the labels at a cap's corners see what the
/// labels at a sheet's corners see. The caps are the start's, then the end's.
/// `grazing` are the planes (a point and the outward normal) the regions cover, whose facets are
/// the regions' to place.
pub fn caps(sk: &Sketch,swept: usize,sheets: &[SweepPatch],sagitta: f64,snap: f64,longest: f64,grazing: &[(V3,V3)])
    -> Result<[Cap;2],String> {
    let SolidDef::Swept {source,motion,from,to} = &sk.solids[swept].def else { return Err("not a continuous sweep".into()) };
    let unit = sagitta/crate::curve::FLATNESS_PX;
    let tool = static_solid_at_unit(sk,*source as usize,unit,0)?;
    let (vertices,triangles,faces) = indexed_faces(&tool.boundary()?);
    let family = Family::read(sk,*motion as usize)?;
    let time_tolerance = 1e-9*(to.value-from.value).abs().max(1.);
    let mut out = Vec::with_capacity(2);
    for (end,t) in [(End::From,from.value),(End::To,to.value)] {
        let pose = family.at(t).map_err(|e| format!("{e:?}"))?;
        let mut mesh = CutMesh::new(vertices.iter().map(|p| pose.point(*p)).collect(),triangles.clone(),faces.clone(),snap,sagitta);
        mesh.refine(longest);
        // the contact curves on the tool at this instant
        for s in sheets {
            let (c,at) = match end { End::From => (0,s.times.first().copied()),End::To => ((s.times.len().saturating_sub(1)) as u32,s.times.last().copied()) };
            let Some(at) = at else { continue };
            if (at-t).abs() > time_tolerance { continue; }
            let line: Vec<(V3,V3)> = s.column_vertices(c).into_iter().map(|v| (s.points[v as usize],s.normals[v as usize])).collect();
            if line.len() < 2 { continue; }
            mesh.cut_along(&line,s.closed)?;
        }
        // Every edge between facets of two different tool faces, as a cut. Across a sharp edge the
        // normal jumps, so `n·v` changes sign discontinuously and a component's most decisive facet
        // cannot speak for both sides of it. Measured on the tumbling cylinder: the wall's quadrants
        // reached through the rim into a disc, whose sign there is the other way, so the disc's
        // facet was the most decisive one and the whole quadrant was dropped with it — 364 of the
        // 370 cap facets beside the z = 0 band were advancing inside a dropped component, which is
        // the surface that band was missing. This is the rule the grazing block below already
        // states, applied to the tool's own edges and not only to a grazing face's boundary.
        {
            let mut by_edge: std::collections::BTreeMap<(u32,u32),Vec<usize>> = Default::default();
            for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); by_edge.entry((a.min(b),a.max(b))).or_default().push(i); } }
            let edges: Vec<(u32,u32)> = by_edge.into_iter().filter(|(_,f)| f.len() == 2 && mesh.faces[f[0]] != mesh.faces[f[1]]).map(|(e,_)| e).collect();
            for e in edges { mesh.cuts.insert(e); }
        }
        // The facets of a grazing face, and the edges between them and the rest as cuts: the
        // region that face sweeps holds them at both ends of the roll, and a component must not
        // reach through them to be judged by a normal velocity that is not its own.
        let in_plane = |i: usize| -> bool {
            let Some(facet) = mesh.facet_normal(i) else { return false };
            grazing.iter().any(|&(o,n)| dot(facet,n).abs() > 1.-1e-6
                && mesh.triangles[i].iter().all(|&v| dot(sub(mesh.vertices[v as usize],o),n).abs() <= snap))
        };
        let left: Vec<bool> = (0..mesh.triangles.len()).map(in_plane).collect();
        if left.iter().any(|x| *x) {
            let mut by_edge: std::collections::BTreeMap<(u32,u32),Vec<usize>> = Default::default();
            for (i,t) in mesh.triangles.iter().enumerate() { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); by_edge.entry((a.min(b),a.max(b))).or_default().push(i); } }
            let edges: Vec<(u32,u32)> = by_edge.into_iter().filter(|(_,f)| f.len() == 2 && left[f[0]] != left[f[1]]).map(|(e,_)| e).collect();
            for e in edges { mesh.cuts.insert(e); }
        }
        let inverse = pose.inverse();
        // the normal velocity of each facet, relative to its speed
        let relative: Vec<f64> = (0..mesh.triangles.len()).map(|i| {
            let Some(n) = mesh.facet_normal(i) else { return 0. };
            let centroid: V3 = std::array::from_fn(|k| mesh.triangles[i].iter().map(|&v| mesh.vertices[v as usize][k]).sum::<f64>()/3.);
            let v = pose.velocity(inverse.point(centroid));
            let speed = norm(v);
            if speed > 0. { dot(n,v)/speed } else { 0. }
        }).collect();
        let component = mesh.components();
        let mut extreme: std::collections::BTreeMap<usize,f64> = Default::default();
        for i in 0..mesh.triangles.len() { let e = extreme.entry(component[i]).or_insert(0.); if relative[i].abs() > e.abs() { *e = relative[i]; } }
        let decisive = 0.25;
        let wanted = match end { End::From => -1.,End::To => 1. };
        let mut report: std::collections::BTreeMap<usize,CapComponent> = Default::default();
        for i in 0..mesh.triangles.len() {
            let e = extreme[&component[i]];
            let entry = report.entry(component[i]).or_insert(CapComponent {facets:0,extreme:e,kept:e.abs() < decisive || e.signum() == wanted});
            entry.facets += 1;
        }
        let components = report.into_values().collect::<Vec<_>>();
        let mut patch = SweepPatch {points:Vec::new(),normals:Vec::new(),triangles:Vec::new(),column:Vec::new(),times:vec![t],closed:false};
        // a vertex on a tool edge: its facets lie on more than one face
        let mut first = vec![u32::MAX;mesh.vertices.len()];
        let mut on_edge = vec![false;mesh.vertices.len()];
        for (tri,&face) in mesh.triangles.iter().zip(&mesh.faces) {
            for &v in tri {
                let f = &mut first[v as usize];
                if *f == u32::MAX { *f = face; } else if *f != face { on_edge[v as usize] = true; }
            }
        }
        let mut edge = Vec::new();
        let mut origin = Vec::new();
        let mut remap: Vec<u32> = vec![u32::MAX;mesh.vertices.len()];
        // every vertex's summed facet normal in one pass, each facet's added in the order
        // `vertex_normal` adds them
        let mut normals: Vec<V3> = vec![[0.;3];mesh.vertices.len()];
        for (t,tri) in mesh.triangles.iter().enumerate() {
            let Some(f) = mesh.facet_normal(t) else { continue };
            for &v in tri { normals[v as usize] = add(normals[v as usize],f); }
        }
        for i in 0..mesh.triangles.len() {
            let Some(_) = mesh.facet_normal(i) else { continue };
            if left[i] { continue; }
            let e = extreme[&component[i]];
            if !(e.abs() < decisive || e.signum() == wanted) { continue; }
            patch.triangles.push(mesh.triangles[i].map(|v| {
                if remap[v as usize] == u32::MAX {
                    remap[v as usize] = patch.points.len() as u32;
                    patch.points.push(mesh.vertices[v as usize]);
                    patch.normals.push(normalised(normals[v as usize]).unwrap_or([0.;3]));
                    patch.column.push(0);
                    edge.push(on_edge[v as usize]);
                    origin.push(mesh.origin[v as usize]);
                }
                remap[v as usize]
            }));
        }
        out.push(Cap {end,patch,components,tool_edges:edge,origin});
    }
    Ok([out.remove(0),out.remove(0)])
}
