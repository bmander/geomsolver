//! An indexed solid built from one sector by identity (phase 2 of docs/rust-kernel-plan.md): the
//! sector turned into every copy about its axis, its two sides — one face, and the same face turned
//! a pitch on — left out, and the pieces of one surface that meet across a side made one face. A face
//! of revolution about the axis keeps the sector's surface in every copy, its pcurves moved along `u`
//! by the turn, so the pieces of a blank face continue one another's parameters; a piece meeting both
//! sides closes round the axis into a ring, whose last junction is kept as its seam (one edge, used
//! twice, its pcurves a period apart). Nothing is intersected or sewn: every vertex on the far side is
//! the near side's turned, matched once in the sector, and every other point is the sector's turned.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::geom::{Rigid,Surface,Uv,V};
use super::topo::{Brep,Coedge,EdgeCurve,Face,Pcurve};
use crate::space::{cross,distance,norm,sub};
use std::f64::consts::TAU;
use std::sync::Arc;

/// What the pattern found in the sector and built.
#[derive(Clone,Debug)]
pub struct Built {
    pub solid: Brep,
    /// The sector's faces that are its sides (the near one, then the far: the near turned by `step`).
    pub sides: [usize;2],
    /// The turn from one copy to the next (radians, about `axis`).
    pub step: f64,
    /// The furthest a far side's vertex is from its near partner turned.
    pub matched: f64,
    /// What the copies are made of, for their mesh: the sector, the axis, the copies, and each far
    /// vertex's and far edge's near partner.
    sector: Brep,
    o: V,
    a: V,
    count: usize,
    partner: Vec<Option<u32>>,
    edge_partner: Vec<Option<(u32,bool)>>,
}

impl Built {
    /// The solid's mesh within `bar` (`mesh::mesh`), made as the sector's — its sides left out, its far
    /// side sampled as its near side turned — turned into every copy, the copies sharing their seam
    /// points by index: the same in every copy to the turn's rounding, as the solid is.
    pub fn mesh(&self,bar: f64,angular: f64) -> Result<super::mesh::Mesh,String> {
        let turned = super::mesh::Turned {turn:Rigid::turn(self.o,self.a,self.step),vertex:&self.partner,edge:&self.edge_partner};
        let (one,alias) = super::mesh::mesh_with(&self.sector,bar,angular,&|f| f != self.sides[0] && f != self.sides[1],Some(&turned))?;
        let (n,l) = (self.count,one.pts.len());
        // a point of copy k is copy k's, or (standing for a near point) copy k + 1's
        let id = |k: usize,x: u32| match alias[x as usize] { Some(w) => ((k+1)%n)*l+w as usize,None => k*l+x as usize };
        let mut used = vec![false;n*l];
        let mut tris = Vec::with_capacity(n*one.tris.len());
        for k in 0..n { for t in &one.tris { let g = t.map(|x| id(k,x)); for &x in &g { used[x] = true; } tris.push(g); } }
        let mut renum = vec![u32::MAX;n*l];
        let mut pts = Vec::new();
        for k in 0..n {
            let turn = Rigid::turn(self.o,self.a,self.step*k as f64);
            for x in 0..l { if used[k*l+x] { renum[k*l+x] = pts.len() as u32; pts.push(turn.point(one.pts[x])); } }
        }
        let of = (0..n).flat_map(|_| one.of.iter().copied()).collect();
        Ok(super::mesh::Mesh {pts,tris:tris.into_iter().map(|t| t.map(|x| renum[x])).collect(),of,sag:one.sag,turned:one.turned*n})
    }
}

/// Whether `s` is a surface of revolution about the line through `o` along the unit `a`.
fn about(s: &Surface,o: V,a: V,tol: f64) -> bool {
    if !matches!(s,Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) | Surface::Torus(..)) { return false }
    let f = s.frame();
    let off = sub(f.o,o);
    cross(f.z,a).iter().all(|c| c.abs() <= 1e-12) && norm(cross(off,a)) <= tol
}

/// The shift along `u` that turning surface `s` (of revolution about the axis) by `angle` makes:
/// `+angle` or `-angle`, read off one of its points.
fn shift(s: &Surface,turn: &Rigid,angle: f64,uv: Uv,tol: f64) -> Result<f64,String> {
    let p = turn.point(s.point(uv));
    for sign in [1.,-1.] {
        if distance(s.point([uv[0]+sign*angle,uv[1]]),p) <= tol { return Ok(sign*angle) }
    }
    Err(format!("a {} about the axis does not turn along its own u",s.kind()))
}

/// A pcurve moved along `u` by `du`.
fn moved_pcurve(p: &Pcurve,du: f64) -> Pcurve {
    if du == 0. { return p.clone() }
    match p {
        Pcurve::Line {a,b} => Pcurve::Line {a:[a[0]+du,a[1]],b:[b[0]+du,b[1]]},
        Pcurve::Inverse {a,b} => Pcurve::Inverse {a:[a[0]+du,a[1]],b:[b[0]+du,b[1]]},
        Pcurve::Curve(c) => Pcurve::Curve(Arc::new(c.moved(&Rigid {t:[du,0.,0.],..Rigid::identity()}))),
    }
}

/// Whether two faces lie on one surface: the same kind, frame and numbers.
fn same_surface(a: &Surface,b: &Surface) -> bool {
    a.kind() == b.kind() && super::step::written(a) == super::step::written(b)
}

/// The indexed solid of `count` copies of `sector` turned about the line through `o` along `axis`,
/// its two sides found as the one pair of faces (not of revolution about the axis) one of which,
/// turned a pitch either way, lies on the other within `tol`; vertices matched across a side within
/// `tol` too.
pub fn pattern(sector: &Brep,o: V,axis: V,count: usize,tol: f64) -> Result<Built,String> {
    if count < 2 { return Err("a pattern needs at least two copies".into()) }
    let a = { let l = norm(axis); [axis[0]/l,axis[1]/l,axis[2]/l] };
    let pitch = TAU/count as f64;
    let faces = &sector.faces;
    let revolved: Vec<bool> = faces.iter().map(|f| about(&f.surface,o,a,tol)).collect();
    // the sides: a face whose points, turned a pitch, lie on another face's surface
    let samples = |f: &Face| -> Vec<V> {
        let uvs: Vec<Uv> = f.loops.iter().flatten().map(|c| { let e = &sector.edges[c.edge as usize];
            c.pcurve.at((e.t[0]+e.t[1])/2.,e,&f.surface,&sector.vertices) }).collect();
        uvs.into_iter().map(|uv| f.surface.point(uv)).collect()
    };
    let mut sides = None;
    'find: for i in 0..faces.len() { if revolved[i] { continue }
        for j in 0..faces.len() { if j == i || revolved[j] { continue }
            for sign in [1.,-1.] {
                let turn = Rigid::turn(o,a,sign*pitch);
                let lands = samples(&faces[i]).iter().all(|&p| { let q = turn.point(p);
                    distance(faces[j].surface.point(faces[j].surface.inverse(q)),q) <= tol });
                if lands { sides = Some(([i,j],sign*pitch)); break 'find }
            }
        }
    }
    let Some((sides,step)) = sides else { return Err("the sector has no pair of sides, one the other turned a pitch".into()) };
    let turn = |k: usize| Rigid::turn(o,a,step*k as f64);
    // each side's edges and vertices
    let side_edges = |s: usize| -> Vec<u32> { let mut v: Vec<u32> = faces[s].loops.iter().flatten().map(|c| c.edge).collect(); v.sort(); v.dedup(); v };
    let (near_edges,far_edges) = (side_edges(sides[0]),side_edges(sides[1]));
    let vertices_of = |es: &[u32]| -> Vec<u32> { let mut v: Vec<u32> = es.iter().flat_map(|&e| sector.edges[e as usize].v).collect(); v.sort(); v.dedup(); v };
    let (near_vs,far_vs) = (vertices_of(&near_edges),vertices_of(&far_edges));
    // a far vertex is a near one turned a step
    let one = turn(1);
    let mut partner = vec![None;sector.vertices.len()];
    let mut matched: f64 = 0.;
    for &v in &far_vs {
        let p = sector.vertices[v as usize].p;
        let (d,w) = near_vs.iter().map(|&w| (distance(one.point(sector.vertices[w as usize].p),p),w)).min_by(|x,y| x.0.total_cmp(&y.0))
            .ok_or("a side without vertices")?;
        if d > tol { return Err(format!("a vertex of the far side is {d:e} from every near one turned")) }
        partner[v as usize] = Some(w);
        matched = matched.max(d);
    }
    // a far edge is a near one turned: by its ends, and its middle where two share them
    let mut edge_partner = vec![None;sector.edges.len()];
    for &e in &far_edges {
        let ef = &sector.edges[e as usize];
        let m = ef.point((ef.t[0]+ef.t[1])/2.,&sector.vertices);
        let found = near_edges.iter().copied().filter_map(|n| {
            let en = &sector.edges[n as usize];
            let ends = en.v.map(Some);
            let same = ends == [partner[ef.v[0] as usize],partner[ef.v[1] as usize]];
            let other = ends == [partner[ef.v[1] as usize],partner[ef.v[0] as usize]];
            if !same && !other { return None }
            let d = distance(one.point(en.point((en.t[0]+en.t[1])/2.,&sector.vertices)),m);
            Some((d,n,same))
        }).min_by(|x,y| x.0.total_cmp(&y.0)).ok_or("an edge of the far side is no near one turned")?;
        edge_partner[e as usize] = Some((found.1,found.2));
    }
    // the face across each side edge, and which pieces are one face: copy k's piece across the far
    // side and copy k + 1's across the near, where both lie on one surface of revolution
    let across = |e: u32,side: usize| -> Result<usize,String> {
        let mut by = faces.iter().enumerate().filter(|&(fi,f)| fi != side && f.loops.iter().flatten().any(|c| c.edge == e)).map(|(fi,_)| fi);
        by.next().ok_or_else(|| "a side edge with no face across it".to_string())
    };
    let node = |k: usize,f: usize| k*faces.len()+f;
    let mut parent: Vec<usize> = (0..count*faces.len()).collect();
    fn root(p: &mut [usize],mut x: usize) -> usize { while p[x] != x { p[x] = p[p[x]]; x = p[x]; } x }
    let mut ring = vec![false;faces.len()];
    // the joins between pieces of a face that does not close round the axis (copy k's to copy k + 1's)
    let mut links: Vec<((usize,usize),(usize,usize))> = Vec::new();
    for &e in &far_edges {
        let (n,_) = edge_partner[e as usize].unwrap();
        let (fa,fb) = (across(e,sides[1])?,across(n,sides[0])?);
        if !(revolved[fa] && revolved[fb] && same_surface(&faces[fa].surface,&faces[fb].surface) && faces[fa].reversed == faces[fb].reversed) {
            return Err(format!("across a side, a {} meets a {}: only pieces of one surface of revolution are joined yet",
                faces[fa].surface.kind(),faces[fb].surface.kind()))
        }
        if fa == fb { ring[fa] = true; }
        for k in 0..count {
            let (x,y) = (root(&mut parent,node(k,fa)),root(&mut parent,node((k+1)%count,fb)));
            parent[x] = y;
            if fa != fb { links.push(((k,fa),((k+1)%count,fb))); }
        }
    }
    // each face of revolution's parameters within half a turn of the sector's middle (a kernel keeps
    // each face's in a period of its own choosing): the pieces of one face in neighbouring copies
    // then continue one another's, where a piece a period away would meet its neighbour on pcurves a
    // period apart
    let (lo,hi) = sector.bounds();
    let middle = [(lo[0]+hi[0])/2.,(lo[1]+hi[1])/2.,(lo[2]+hi[2])/2.];
    let home: Vec<f64> = faces.iter().enumerate().map(|(fi,f)| {
        if !revolved[fi] { return 0. }
        let uc = f.surface.inverse(middle)[0];
        let us: Vec<f64> = f.loops.iter().flatten().map(|c| { let e = &sector.edges[c.edge as usize];
            c.pcurve.at((e.t[0]+e.t[1])/2.,e,&f.surface,&sector.vertices)[0] }).collect();
        let (u0,u1) = us.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),&u| (l.min(u),h.max(u)));
        -((((u0+u1)/2.)-uc)/TAU).round()*TAU
    }).collect();
    // the turn's shift along u of each face of revolution
    let du: Vec<f64> = faces.iter().enumerate().map(|(fi,f)| -> Result<f64,String> {
        if !revolved[fi] { return Ok(0.) }
        let c = &f.loops[0][0];
        let e = &sector.edges[c.edge as usize];
        shift(&f.surface,&one,step,c.pcurve.at(e.t[0],e,&f.surface,&sector.vertices),tol.max(1e-9*(1.+sector.size())))
    }).collect::<Result<_,_>>()?;

    // each sheet cut to its face's parameter box (a hundredth of it about it): the sector's sheet is
    // fitted wider than the face, and every copy would carry all of it into a file
    let surfaces: Vec<Surface> = faces.iter().map(|f| match &f.surface {
        Surface::BSpline(fr,net) => {
            let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
            for c in f.loops.iter().flatten() {
                let e = &sector.edges[c.edge as usize];
                let mut ts: Vec<f64> = (0..=64).map(|i| e.t[0]+(e.t[1]-e.t[0])*i as f64/64.).collect();
                if let Pcurve::Curve(pc) = &c.pcurve {
                    let mut cuts = vec![e.t[0]]; cuts.extend(pc.breaks(e.t)); cuts.push(e.t[1]);
                    for w in cuts.windows(2) { ts.extend((1..8).map(|i| w[0]+(w[1]-w[0])*i as f64/8.)); }
                }
                for t in ts { let p = c.pcurve.at(t,e,&f.surface,&sector.vertices); for k in 0..2 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); } }
            }
            let m = [(hi[0]-lo[0])*0.01,(hi[1]-lo[1])*0.01];
            Surface::BSpline(*fr,Arc::new(net.segment([lo[0]-m[0],hi[0]+m[0]],[lo[1]-m[1],hi[1]+m[1]])))
        }
        s => s.clone(),
    }).collect();

    let mut out = Brep::default();
    // the vertices: every copy's, a far side's being the next copy's near one
    let mut vid = vec![u32::MAX;count*sector.vertices.len()];
    for k in 0..count { for (v,p) in sector.vertices.iter().enumerate() {
        if partner[v].is_none() { vid[k*sector.vertices.len()+v] = out.vertex(turn(k).point(p.p)); }
    } }
    for k in 0..count { for v in 0..sector.vertices.len() {
        if let Some(w) = partner[v] { vid[k*sector.vertices.len()+v] = vid[((k+1)%count)*sector.vertices.len()+w as usize]; }
    } }
    let vertex = |k: usize,v: u32| vid[k*sector.vertices.len()+v as usize];
    // the edges: every copy's but the sides', and a ring's seam: its junction between the last copy
    // and the first, copy 0's near side edge
    let side_of = |e: u32| near_edges.binary_search(&e).is_ok() || far_edges.binary_search(&e).is_ok();
    let mut eid = vec![u32::MAX;count*sector.edges.len()];
    for k in 0..count { for (e,edge) in sector.edges.iter().enumerate() {
        let near = near_edges.binary_search(&(e as u32)).is_ok();
        if side_of(e as u32) && !(near && k == 0 && ring[across(e as u32,sides[0])?]) { continue }
        let curve = match &edge.curve { EdgeCurve::Curve(c) => EdgeCurve::Curve(c.moved(&turn(k))),EdgeCurve::Degenerate => EdgeCurve::Degenerate };
        eid[k*sector.edges.len()+e] = out.edge(curve,edge.t,edge.v.map(|v| vertex(k,v)));
    } }
    // the faces: one a group of pieces
    let mut groups: std::collections::BTreeMap<usize,Vec<(usize,usize)>> = std::collections::BTreeMap::new();
    for k in 0..count { for f in 0..faces.len() { if f == sides[0] || f == sides[1] { continue }
        let r = root(&mut parent,node(k,f));
        groups.entry(r).or_default().push((k,f));
    } }
    for members in groups.values() {
        let (k0,f0) = members[0];
        let first = &faces[f0];
        let surface = if revolved[f0] { surfaces[f0].clone() } else { surfaces[f0].moved(&turn(k0)) };
        // each piece's place along the face: its copy, but for a face that does not close round the
        // axis, one more than the piece it continues (so a face across the last copy and the first
        // reads its first copy's piece a period on)
        let mut place: std::collections::BTreeMap<(usize,usize),i64> = std::collections::BTreeMap::new();
        place.insert(members[0],members[0].0 as i64);
        if !ring[f0] {
            loop {
                let mut grew = false;
                for &(x,y) in &links {
                    match (place.get(&x).copied(),place.get(&y).copied()) {
                        (Some(i),None) => { place.insert(y,i+1); grew = true; }
                        (None,Some(i)) => { place.insert(x,i-1); grew = true; }
                        _ => {}
                    }
                }
                if !grew { break }
            }
        }
        let mut uses = Vec::new();
        for &(k,f) in members {
            for c in faces[f].loops.iter().flatten() {
                let at = home[f]+(if ring[f] { k as i64 } else { place[&(k,f)] }) as f64*du[f];
                let (edge,reversed,pcurve) = if far_edges.binary_search(&c.edge).is_ok() {
                    // the far side: a ring's last junction is its seam, copy 0's near edge, read a period on
                    if !(ring[f] && k == count-1) { continue }
                    let (n,same) = edge_partner[c.edge as usize].unwrap();
                    let near_use = faces[f].loops.iter().flatten().find(|u| u.edge == n).ok_or("a ring's piece without its near edge")?;
                    (eid[n as usize],if same { c.reversed } else { !c.reversed },moved_pcurve(&near_use.pcurve,home[f]+count as f64*du[f]))
                } else if near_edges.binary_search(&c.edge).is_ok() {
                    if !(ring[f] && k == 0) { continue }
                    (eid[c.edge as usize],c.reversed,moved_pcurve(&c.pcurve,home[f]))
                } else {
                    (eid[k*sector.edges.len()+c.edge as usize],c.reversed,moved_pcurve(&c.pcurve,at))
                };
                uses.push(Coedge {edge,reversed,pcurve});
            }
        }
        let loops = loops_of(uses,&out,&surface)?;
        out.faces.push(Face {surface,reversed:first.reversed,loops,name:first.name.clone()});
    }
    // vertices no edge uses (a side's own) left out
    let mut used = vec![false;out.vertices.len()];
    for e in &out.edges { for v in e.v { used[v as usize] = true; } }
    let mut renum = vec![u32::MAX;out.vertices.len()];
    let mut kept = Vec::new();
    for (i,v) in out.vertices.iter().enumerate() { if used[i] { renum[i] = kept.len() as u32; kept.push(v.clone()); } }
    out.vertices = kept;
    for e in &mut out.edges { e.v = e.v.map(|v| renum[v as usize]); }
    super::json::measure(&mut out);
    Ok(Built {solid:out,sides,step,matched,sector:sector.clone(),o,a,count,partner,edge_partner})
}

/// A face's uses walked into closed loops: each the next whose start is the last one's end vertex,
/// nearest it in the face's parameters, a loop closing where its first use is the nearest; the loop
/// enclosing the most of the face's parameters first.
fn loops_of(mut left: Vec<Coedge>,b: &Brep,s: &Surface) -> Result<Vec<Vec<Coedge>>,String> {
    let ends = |c: &Coedge| {
        let e = &b.edges[c.edge as usize];
        let (t0,t1) = if c.reversed { (e.t[1],e.t[0]) } else { (e.t[0],e.t[1]) };
        let (v0,v1) = if c.reversed { (e.v[1],e.v[0]) } else { (e.v[0],e.v[1]) };
        ((v0,c.pcurve.at(t0,e,s,&b.vertices)),(v1,c.pcurve.at(t1,e,s,&b.vertices)))
    };
    let gap = |p: Uv,q: Uv| (p[0]-q[0]).dhypot(p[1]-q[1]);
    let mut loops = Vec::new();
    while !left.is_empty() {
        let mut walk = vec![left.remove(0)];
        let start = ends(&walk[0]).0;
        loop {
            let (v,uv) = ends(walk.last().unwrap()).1;
            let best = (0..left.len()).filter(|&i| ends(&left[i]).0.0 == v).min_by(|&i,&j| gap(ends(&left[i]).0.1,uv).total_cmp(&gap(ends(&left[j]).0.1,uv)));
            let closes = v == start.0;
            match best {
                Some(i) if !closes || gap(ends(&left[i]).0.1,uv) < gap(start.1,uv) => walk.push(left.remove(i)),
                _ if closes => break,
                _ => return Err("a face's uses do not walk into closed loops".into()),
            }
        }
        loops.push(walk);
    }
    // the outer loop first: the one with the greatest area in the parameters
    let area = |l: &Vec<Coedge>| -> f64 {
        let mut sum = 0.;
        for c in l {
            let e = &b.edges[c.edge as usize];
            let mut ts: Vec<f64> = (0..=8).map(|i| e.t[0]+(e.t[1]-e.t[0])*i as f64/8.).collect();
            if c.reversed { ts.reverse(); }
            for w in ts.windows(2) {
                let (p,q) = (c.pcurve.at(w[0],e,s,&b.vertices),c.pcurve.at(w[1],e,s,&b.vertices));
                sum += p[0]*q[1]-p[1]*q[0];
            }
        }
        (sum/2.).abs()
    };
    loops.sort_by(|x,y| area(y).total_cmp(&area(x)));
    Ok(loops)
}
