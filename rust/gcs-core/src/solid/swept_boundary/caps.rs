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
use crate::solid::{SweepPatch,indexed,static_solid_at_unit};

type V3 = [f64;3];

/// Which end of the roll a cap stands at.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum End { From, To }

fn sub(a: V3,b: V3) -> V3 { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }
fn add(a: V3,b: V3) -> V3 { [a[0]+b[0],a[1]+b[1],a[2]+b[2]] }
fn scale(a: V3,s: f64) -> V3 { a.map(|x| x*s) }
fn dot(a: V3,b: V3) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn cross(a: V3,b: V3) -> V3 { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: V3) -> f64 { dot(a,a).sqrt() }
fn normalised(a: V3) -> Option<V3> { let l = norm(a); (l > 0.).then(|| a.map(|x| x/l)) }

/// Which feature of a triangle its nearest point to a query lies on.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Region { Vertex, Edge, Face }

/// The nearest point of a triangle to `p` and the feature it lies on
/// (Ericson's Voronoi-region walk).
pub fn closest_on_triangle(p: V3,a: V3,b: V3,c: V3) -> (V3,Region) {
    let (ab,ac,ap) = (sub(b,a),sub(c,a),sub(p,a));
    let (d1,d2) = (dot(ab,ap),dot(ac,ap));
    if d1 <= 0. && d2 <= 0. { return (a,Region::Vertex); }
    let bp = sub(p,b); let (d3,d4) = (dot(ab,bp),dot(ac,bp));
    if d3 >= 0. && d4 <= d3 { return (b,Region::Vertex); }
    let vc = d1*d4-d3*d2;
    if vc <= 0. && d1 >= 0. && d3 <= 0. { let v = d1/(d1-d3); return (add(a,scale(ab,v)),Region::Edge); }
    let cp = sub(p,c); let (d5,d6) = (dot(ab,cp),dot(ac,cp));
    if d6 >= 0. && d5 <= d6 { return (c,Region::Vertex); }
    let vb = d5*d2-d1*d6;
    if vb <= 0. && d2 >= 0. && d6 <= 0. { let w = d2/(d2-d6); return (add(a,scale(ac,w)),Region::Edge); }
    let va = d3*d6-d5*d4;
    if va <= 0. && d4-d3 >= 0. && d5-d6 >= 0. { let w = (d4-d3)/((d4-d3)+(d5-d6)); return (add(b,scale(sub(c,b),w)),Region::Edge); }
    let denom = 1./(va+vb+vc); let (v,w) = (vb*denom,vc*denom);
    (add(a,add(scale(ab,v),scale(ac,w))),Region::Face)
}

/// A triangle mesh being cut along polylines on it.
pub struct CutMesh {
    pub vertices: Vec<V3>,
    pub triangles: Vec<[u32;3]>,
    /// Vertex pairs the cuts run along.
    pub cuts: std::collections::BTreeSet<(u32,u32)>,
    /// Snap: a point this close to a vertex is that vertex (which moves to
    /// it), and a crossing this close to an edge's end takes the end, so no
    /// sliver thinner than this is ever made.
    pub vertex_tolerance: f64,
    /// How far off a facet's plane a point on the surface may lie.
    pub sagitta: f64,
}

#[derive(Clone,Copy,Debug)]
enum Place { Vertex(u32),Edge(u32,u32),Face(usize) }

impl CutMesh {
    fn facet_normal(&self,t: usize) -> Option<V3> { let [a,b,c] = self.triangles[t].map(|v| self.vertices[v as usize]); normalised(cross(sub(b,a),sub(c,a))) }

    /// The summed normal of the facets at a vertex.
    fn vertex_normal(&self,v: u32) -> V3 {
        let mut n = [0.;3];
        for t in 0..self.triangles.len() { if self.triangles[t].contains(&v) { if let Some(f) = self.facet_normal(t) { n = add(n,f); } } }
        n
    }

    /// Where a point of the surface lies on the mesh: on a vertex, on an
    /// edge, or in a facet, by the nearest point of the nearest facet. The
    /// perpendicular projections of surface points onto a convex facet
    /// mesh leave gaps at the edges (a point over a rim edge projects
    /// short of both facets), so nearness, not containment, decides.
    fn locate(&self,p: V3) -> Option<Place> {
        let mut best: Option<(f64,Place)> = None;
        for (t,tri) in self.triangles.iter().enumerate() {
            let [a,b,c] = tri.map(|v| self.vertices[v as usize]);
            let lo: V3 = std::array::from_fn(|k| a[k].min(b[k]).min(c[k])-2.*self.sagitta);
            let hi: V3 = std::array::from_fn(|k| a[k].max(b[k]).max(c[k])+2.*self.sagitta);
            if (0..3).any(|k| p[k] < lo[k] || p[k] > hi[k]) { continue; }
            let (q,region) = closest_on_triangle(p,a,b,c);
            let dist = norm(sub(p,q));
            if dist > 2.*self.sagitta || best.is_some_and(|(x,_)| dist >= x) { continue; }
            let near = |x: V3| norm(sub(q,x)) <= self.vertex_tolerance;
            let place = if near(a) { Place::Vertex(tri[0]) } else if near(b) { Place::Vertex(tri[1]) } else if near(c) { Place::Vertex(tri[2]) }
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
    fn split_edge(&mut self,a: u32,b: u32,p: V3) -> u32 {
        let v = self.vertices.len() as u32;
        self.vertices.push(p);
        let mut fresh = Vec::new();
        for t in 0..self.triangles.len() {
            let tri = self.triangles[t];
            let Some(k) = (0..3).find(|&k| (tri[k] == a && tri[(k+1)%3] == b) || (tri[k] == b && tri[(k+1)%3] == a)) else { continue };
            let (x,y,c) = (tri[k],tri[(k+1)%3],tri[(k+2)%3]);
            self.triangles[t] = [x,v,c];
            fresh.push([v,y,c]);
        }
        self.triangles.extend(fresh);
        v
    }

    /// A new vertex at `p` inside facet `t`, the facet split in three.
    fn split_face(&mut self,t: usize,p: V3) -> u32 {
        let v = self.vertices.len() as u32;
        self.vertices.push(p);
        let [a,b,c] = self.triangles[t];
        self.triangles[t] = [a,b,v];
        self.triangles.push([b,c,v]);
        self.triangles.push([c,a,v]);
        v
    }

    /// The vertex a surface point is or becomes: a vertex within the snap
    /// of it moves onto it, so the seam is the sheet's own point exactly.
    fn insert(&mut self,p: V3) -> Result<u32,String> {
        match self.locate(p).ok_or_else(|| format!("column point {p:?} is off the tool's mesh"))? {
            Place::Vertex(v) => { self.vertices[v as usize] = p; Ok(v) }
            Place::Edge(a,b) => Ok(self.split_edge(a,b,p)),
            Place::Face(t) => Ok(self.split_face(t,p)),
        }
    }

    fn share_facet(&self,u: u32,w: u32) -> bool { self.triangles.iter().any(|t| t.contains(&u) && t.contains(&w)) }

    /// Join two vertices by a chain of edges along the chord between them,
    /// walked facet by facet: from the vertex reached so far, the facet on it
    /// whose far edge the chord's projection (in the plane of the two ends'
    /// normals) crosses ahead is the next; the crossing splits that edge, or
    /// takes its end when within the snap of it. Only facets the walk enters
    /// are looked at, so a face round a corner, folded into the projection,
    /// cannot offer a crossing of its own. Records every chain edge as a cut.
    fn connect(&mut self,u: u32,w: u32) -> Result<(),String> {
        if u == w { return Ok(()); }
        if self.share_facet(u,w) { self.cuts.insert((u.min(w),u.max(w))); return Ok(()); }
        let (pu,pw) = (self.vertices[u as usize],self.vertices[w as usize]);
        let n = normalised(add(normalised(self.vertex_normal(u)).unwrap_or([0.;3]),normalised(self.vertex_normal(w)).unwrap_or([0.;3]))).ok_or("no normal at a column point")?;
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
            let here: Vec<usize> = (0..self.triangles.len()).filter(|&t| self.triangles[t].contains(&current)).collect();
            if here.iter().any(|&t| self.triangles[t].contains(&w)) { chain.push(w); break; }
            let mut step: Option<(f64,u32,u32,f64)> = None; // t along the chord, edge, s along it
            for &t in &here {
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
            let Some((t,a,b,s)) = step else { return Err(format!("the cut from {pu:?} to {pw:?} leaves the tool's mesh at {:?}",self.vertices[current as usize])) };
            let (pa,pb) = (self.vertices[a as usize],self.vertices[b as usize]);
            let p = add(pa,scale(sub(pb,pa),s));
            let next = if norm(sub(p,pa)) <= self.vertex_tolerance { a } else if norm(sub(p,pb)) <= self.vertex_tolerance { b } else { self.split_edge(a,b,p) };
            if next == w { chain.push(w); break; }
            chain.push(next); current = next; along = t;
        }
        if chain.last() != Some(&w) { return Err(format!("the cut from {pu:?} to {pw:?} does not reach its end")); }
        for k in 0..chain.len()-1 {
            let (a,b) = (chain[k],chain[k+1]);
            if !self.share_facet(a,b) { return Err(format!("the cut from {pu:?} to {pw:?} does not follow the tool's mesh between {:?} and {:?}",self.vertices[a as usize],self.vertices[b as usize])); }
            self.cuts.insert((a.min(b),a.max(b)));
        }
        Ok(())
    }

    /// Cut along a polyline on the surface.
    pub fn cut_along(&mut self,line: &[V3],closed: bool) -> Result<(),String> {
        let ids: Vec<u32> = line.iter().map(|p| self.insert(*p)).collect::<Result<_,_>>()?;
        for k in 0..ids.len().saturating_sub(1) { self.connect(ids[k],ids[k+1])?; }
        if closed && ids.len() > 2 { self.connect(ids[ids.len()-1],ids[0])?; }
        Ok(())
    }

    /// The component of each facet, joined across every edge that is not a cut.
    pub fn components(&self) -> Vec<usize> {
        let mut parent: Vec<usize> = (0..self.triangles.len()).collect();
        fn find(parent: &mut Vec<usize>,i: usize) -> usize { let mut r = i; while parent[r] != r { r = parent[r]; } let mut j = i; while parent[j] != r { let next = parent[j]; parent[j] = r; j = next; } r }
        let mut by_edge: std::collections::BTreeMap<(u32,u32),Vec<usize>> = Default::default();
        for (i,t) in self.triangles.iter().enumerate() { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); by_edge.entry((a.min(b),a.max(b))).or_default().push(i); } }
        for (e,ts) in &by_edge {
            if self.cuts.contains(e) { continue; }
            for w in ts.windows(2) { let (ri,rj) = (find(&mut parent,w[0]),find(&mut parent,w[1])); if ri != rj { parent[ri] = rj; } }
        }
        (0..self.triangles.len()).map(|i| find(&mut parent,i)).collect()
    }
}

/// One component of a cut cap: how many facets, its most decisive relative
/// normal velocity (signed, negative receding), and whether the cap kept it.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct CapComponent { pub facets: usize,pub extreme: f64,pub kept: bool }

/// The tool's boundary at each end pose, cut along the end columns of
/// `sheets` and reduced to the components that recede from the sweep (at
/// the start) or advance into it (at the end), plus every grazing
/// component. `sagitta` is the construction's chord error, which cuts the
/// tool's mesh; `snap` is how near a column point or a crossing must come to
/// a mesh vertex to be it (a quarter of the sagitta serves: what it moves is
/// well within the certificate's probe, and no sliver thinner than it is
/// left for the stitch).
pub fn caps(sk: &Sketch,swept: usize,sheets: &[SweepPatch],sagitta: f64,snap: f64) -> Result<([SweepPatch;2],[Vec<CapComponent>;2]),String> {
    let SolidDef::Swept {source,motion,from,to} = &sk.solids[swept].def else { return Err("not a continuous sweep".into()) };
    let unit = sagitta/crate::curve::FLATNESS_PX;
    let tool = static_solid_at_unit(sk,*source as usize,unit,0)?;
    let (vertices,triangles) = indexed(&tool.boundary()?);
    let family = Family::read(sk,*motion as usize)?;
    let time_tolerance = 1e-9*(to.value-from.value).abs().max(1.);
    let mut out = Vec::with_capacity(2);
    let mut reports = Vec::with_capacity(2);
    for (end,t) in [(End::From,from.value),(End::To,to.value)] {
        let pose = family.at(t).map_err(|e| format!("{e:?}"))?;
        let mut mesh = CutMesh {vertices:vertices.iter().map(|p| pose.point(*p)).collect(),triangles:triangles.clone(),cuts:Default::default(),vertex_tolerance:snap,sagitta};
        // the contact curves on the tool at this instant
        for s in sheets {
            let (c,at) = match end { End::From => (0,s.times.first().copied()),End::To => ((s.times.len().saturating_sub(1)) as u32,s.times.last().copied()) };
            let Some(at) = at else { continue };
            if (at-t).abs() > time_tolerance { continue; }
            let line: Vec<V3> = s.column_vertices(c).into_iter().map(|v| s.points[v as usize]).collect();
            if line.len() < 2 { continue; }
            mesh.cut_along(&line,s.closed)?;
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
        reports.push(report.into_values().collect::<Vec<_>>());
        let mut patch = SweepPatch {points:Vec::new(),normals:Vec::new(),triangles:Vec::new(),column:Vec::new(),times:vec![t],closed:false};
        let mut remap: Vec<u32> = vec![u32::MAX;mesh.vertices.len()];
        let normals: Vec<V3> = (0..mesh.vertices.len() as u32).map(|v| mesh.vertex_normal(v)).collect();
        for i in 0..mesh.triangles.len() {
            let Some(_) = mesh.facet_normal(i) else { continue };
            let e = extreme[&component[i]];
            if !(e.abs() < decisive || e.signum() == wanted) { continue; }
            patch.triangles.push(mesh.triangles[i].map(|v| {
                if remap[v as usize] == u32::MAX {
                    remap[v as usize] = patch.points.len() as u32;
                    patch.points.push(mesh.vertices[v as usize]);
                    patch.normals.push(normalised(normals[v as usize]).unwrap_or([0.;3]));
                    patch.column.push(0);
                }
                remap[v as usize]
            }));
        }
        out.push(patch);
    }
    Ok(([out.remove(0),out.remove(0)],[reports.remove(0),reports.remove(0)]))
}
