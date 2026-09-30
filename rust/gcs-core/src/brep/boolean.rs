//! Booleans of two solids (docs/rust-kernel-plan.md, rung 1): every edge of each split where it
//! crosses the other's faces, every pair of faces intersected and the curves kept where they lie in
//! both, every face split by what now lies on it — traced in its own parameters, the material on
//! the left — each piece placed against the other solid, and the pieces the operation keeps
//! assembled, their edges shared by construction.
use super::geom::{Uv,V};
use super::query::{curve_surface,Located,Meets,Place};
use super::ssi::{intersect,Ssi};
use super::topo::{unwrap,Brep,Coedge,EdgeCurve,Face,Pcurve};
use crate::space::{distance,norm};
use std::collections::BTreeMap;

const TAU: f64 = std::f64::consts::TAU;

/// What a Boolean keeps.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Op { Union,Cut,Common }

/// Points merged where they come within the tolerance of one another.
struct Pool { pts: Vec<V>,tol: f64,grid: crate::space::Grid }

impl Pool {
    fn new(tol: f64) -> Pool { Pool {pts:Vec::new(),tol,grid:crate::space::Grid::new(4.*tol)} }
    fn at(&mut self,p: V) -> u32 {
        let mut found = None;
        let pts = &self.pts;
        let tol = self.tol;
        self.grid.around(p,|i| if found.is_none() && distance(pts[i as usize],p) <= tol { found = Some(i) });
        if let Some(i) = found { return i }
        self.pts.push(p);
        let i = (self.pts.len()-1) as u32;
        self.grid.insert(p,i);
        i
    }
}

/// An edge of the working set: a stretch of a curve (or a pole) between two pooled vertices.
#[derive(Clone,Copy,Debug)]
struct WEdge { curve: EdgeCurve,t: [f64;2],v: [u32;2] }

impl WEdge {
    fn point(&self,t: f64,pool: &Pool) -> V {
        match self.curve { EdgeCurve::Curve(c) => c.point(t),EdgeCurve::Degenerate => pool.pts[self.v[0] as usize] }
    }
}

/// One way along an edge on a face, with its curve in the face's parameters.
#[derive(Clone,Debug)]
struct Half { edge: u32,along: bool,pcurve: Pcurve,from: Uv,to: Uv }

/// A face's box in space, from its edges and a grid across its parameters.
fn face_box(b: &Brep,fi: usize) -> ([f64;3],[f64;3]) {
    let f = &b.faces[fi];
    let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
    let mut grow = |p: V| for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); };
    let (mut ulo,mut uhi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
    for l in &f.loops { for c in l {
        let e = &b.edges[c.edge as usize];
        for j in 0..=16 {
            let t = e.t[0]+(e.t[1]-e.t[0])*j as f64/16.;
            grow(e.point(t,&b.vertices));
            let uv = c.pcurve.at(t,e,&f.surface,&b.vertices);
            for k in 0..2 { ulo[k] = ulo[k].min(uv[k]); uhi[k] = uhi[k].max(uv[k]); }
        }
    } }
    for i in 0..=8 { for j in 0..=8 {
        grow(f.surface.point([ulo[0]+(uhi[0]-ulo[0])*i as f64/8.,ulo[1]+(uhi[1]-ulo[1])*j as f64/8.]));
    } }
    (lo,hi)
}

fn overlap(a: &([f64;3],[f64;3]),b: &([f64;3],[f64;3]),pad: f64) -> bool {
    (0..3).all(|k| a.0[k] <= b.1[k]+pad && b.0[k] <= a.1[k]+pad)
}

/// A closed curve's parameters `t` moved by whole periods into `[lo, lo + period)`.
fn around(t: f64,lo: f64) -> f64 { lo+(t-lo).rem_euclid(TAU) }

/// `a` combined with `b` by `op`, to `tol` (a length).
pub fn boolean(a: &Brep,b: &Brep,op: Op,tol: f64) -> Result<Brep,String> {
    let solids = [a,b];
    let located = [Located::new(a,tol),Located::new(b,tol)];
    let mut pool = Pool::new(tol);
    // the working edges: A's, then B's, their vertices pooled
    let mut edges: Vec<WEdge> = Vec::new();
    let mut first_edge = [0usize;2];
    for (s,sol) in solids.iter().enumerate() {
        first_edge[s] = edges.len();
        let ids: Vec<u32> = sol.vertices.iter().map(|v| pool.at(v.p)).collect();
        for e in &sol.edges { edges.push(WEdge {curve:e.curve,t:e.t,v:[ids[e.v[0] as usize],ids[e.v[1] as usize]]}); }
    }
    let boxes: [Vec<_>;2] = [(0..a.faces.len()).map(|i| face_box(a,i)).collect(),(0..b.faces.len()).map(|i| face_box(b,i)).collect()];
    let pad = 4.*tol;
    // 1. every edge against every face of the other solid: where it crosses, a vertex on both
    let mut cuts: Vec<Vec<(f64,u32)>> = vec![Vec::new();edges.len()];
    let mut on_face: [Vec<Vec<u32>>;2] = [vec![Vec::new();a.faces.len()],vec![Vec::new();b.faces.len()]];
    for s in 0..2 {
        let other = 1-s;
        for (i,e) in solids[s].edges.iter().enumerate() {
            let EdgeCurve::Curve(c) = e.curve else { continue };
            let we = first_edge[s]+i;
            let mut ebox = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
            for j in 0..=32 {
                let p = c.point(e.t[0]+(e.t[1]-e.t[0])*j as f64/32.);
                for k in 0..3 { ebox.0[k] = ebox.0[k].min(p[k]); ebox.1[k] = ebox.1[k].max(p[k]); }
            }
            // a sampled chord may cut inside its curve's bulge: pad by the sagitta's bound
            let sag = c.speed()*(e.t[1]-e.t[0])/32.;
            for (fi,f) in solids[other].faces.iter().enumerate() {
                if !overlap(&ebox,&boxes[other][fi],pad+sag) { continue }
                match curve_surface(&c,e.t,&f.surface,tol) {
                    Meets::Along => {}
                    Meets::At(roots) => for (t,_) in roots {
                        let q = c.point(t);
                        if located[other].face_place(fi,q) == Place::Out { continue }
                        let v = pool.at(q);
                        cuts[we].push((t,v));
                        on_face[other][fi].push(v);
                    },
                }
            }
        }
    }
    // 2. every pair of faces: their surfaces' curves, kept where they lie in both faces
    let mut ssi_edges: Vec<(usize,usize,usize)> = Vec::new(); // (edge, face of A, face of B)
    for fa in 0..a.faces.len() {
        for fb in 0..b.faces.len() {
            if !overlap(&boxes[0][fa],&boxes[1][fb],pad) { continue }
            let (sa,sb) = (&a.faces[fa].surface,&b.faces[fb].surface);
            let curves = match intersect(sa,sb,tol) {
                Ssi::Curves(cs) => cs,
                Ssi::Same => return Err(format!("faces on one surface ({} and {}) are not combined yet",sa.kind(),sb.kind())),
                Ssi::Traced => return Err(format!("a {} and a {} meet in a curve with no closed form, not traced yet",sa.kind(),sb.kind())),
            };
            for c in curves {
                let mut ts: Vec<(f64,u32)> = Vec::new();
                for &v in on_face[1][fb].iter().chain(&on_face[0][fa]) {
                    let p = pool.pts[v as usize];
                    let t = c.inverse(p);
                    if distance(c.point(t),p) <= 8.*tol && !ts.iter().any(|&(_,w)| w == v) { ts.push((t,v)); }
                }
                let closed = c.period().is_some();
                let mut spans: Vec<([f64;2],[u32;2])> = Vec::new();
                if closed {
                    for t in &mut ts { t.0 = around(t.0,0.); }
                    ts.sort_by(|x,y| x.0.total_cmp(&y.0));
                    if ts.is_empty() {
                        let v = pool.at(c.point(0.));
                        spans.push(([0.,TAU],[v,v]));
                    } else {
                        for k in 0..ts.len() {
                            let (t0,v0) = ts[k];
                            let (mut t1,v1) = ts[(k+1)%ts.len()];
                            if k+1 == ts.len() { t1 += TAU; }
                            spans.push(([t0,t1],[v0,v1]));
                        }
                    }
                } else {
                    ts.sort_by(|x,y| x.0.total_cmp(&y.0));
                    for w in ts.windows(2) { spans.push(([w[0].0,w[1].0],[w[0].1,w[1].1])); }
                }
                for (t,v) in spans {
                    if (t[1]-t[0])*c.speed() <= tol || (v[0] == v[1] && !closed) { continue }
                    let m = c.point((t[0]+t[1])/2.);
                    if located[0].face_place(fa,m) == Place::Out || located[1].face_place(fb,m) == Place::Out { continue }
                    edges.push(WEdge {curve:EdgeCurve::Curve(c),t,v});
                    cuts.push(Vec::new());
                    ssi_edges.push((edges.len()-1,fa,fb));
                }
            }
        }
    }
    // 3. every original edge split where it was cut: its pieces, in order along it
    let mut pieces: Vec<Vec<(u32,[f64;2])>> = Vec::new();
    let mut out: Vec<WEdge> = Vec::new();
    for (i,e) in edges.iter().enumerate() {
        let mut cs: Vec<(f64,u32)> = cuts[i].iter().copied()
            .filter(|&(t,v)| v != e.v[0] && v != e.v[1] && t > e.t[0] && t < e.t[1]).collect();
        cs.sort_by(|x,y| x.0.total_cmp(&y.0));
        cs.dedup_by(|x,y| x.1 == y.1);
        let mut run = vec![(e.t[0],e.v[0])];
        run.extend(cs);
        run.push((e.t[1],e.v[1]));
        let mut ps = Vec::new();
        for w in run.windows(2) {
            if w[0].1 == w[1].1 && w.len() == 2 && run.len() > 2 { continue }
            out.push(WEdge {curve:e.curve,t:[w[0].0,w[1].0],v:[w[0].1,w[1].1]});
            ps.push(((out.len()-1) as u32,[w[0].0,w[1].0]));
        }
        pieces.push(ps);
    }
    // 4. pieces that are one edge merged: the same ends and the same curve between
    let mut rep: Vec<(u32,bool)> = (0..out.len() as u32).map(|i| (i,false)).collect();
    let mut by_ends: BTreeMap<(u32,u32),Vec<u32>> = BTreeMap::new();
    for (i,e) in out.iter().enumerate() { by_ends.entry((e.v[0].min(e.v[1]),e.v[0].max(e.v[1]))).or_default().push(i as u32); }
    for group in by_ends.values() {
        for (k,&i) in group.iter().enumerate() {
            let ei = out[i as usize];
            let mid = ei.point((ei.t[0]+ei.t[1])/2.,&pool);
            for &j in &group[..k] {
                if rep[j as usize].0 != j { continue }
                let ej = out[j as usize];
                let (EdgeCurve::Curve(ci),EdgeCurve::Curve(cj)) = (ei.curve,ej.curve) else { continue };
                let mut t = cj.inverse(mid);
                if cj.period().is_some() { t = around(t,ej.t[0]); }
                if t < ej.t[0]-1e-9 || t > ej.t[1]+1e-9 || distance(cj.point(t),mid) > 8.*tol { continue }
                // the same way round: the same first vertex, or (closed) the same tangent
                let along = if ei.v[0] != ei.v[1] { ei.v[0] == ej.v[0] } else {
                    crate::space::dot(ci.tangent((ei.t[0]+ei.t[1])/2.),cj.tangent(t)) > 0.
                };
                rep[i as usize] = (j,!along);
                break
            }
        }
    }
    // 5. each face split by what lies on it, traced in its parameters
    let mut kept: Vec<Face> = Vec::new();
    let mut used = vec![false;out.len()];
    for s in 0..2 {
        for (fi,f) in solids[s].faces.iter().enumerate() {
            let loc = &located[s];
            let mut halves: Vec<Half> = Vec::new();
            let mut on: Vec<u32> = Vec::new();
            for l in &f.loops { for c in l {
                let we = first_edge[s]+c.edge as usize;
                let orig = &solids[s].edges[c.edge as usize];
                let mut ps = pieces[we].clone();
                if c.reversed { ps.reverse(); }
                for (piece,[ta,tb]) in ps {
                    let (r,flip) = rep[piece as usize];
                    let at = |t: f64| c.pcurve.at(t,orig,&f.surface,&solids[s].vertices);
                    let (ua,ub) = (at(ta),at(tb));
                    let pcurve = if r == piece { match c.pcurve {
                        Pcurve::Line {..} => Pcurve::Line {a:ua,b:ub},
                        Pcurve::Inverse {..} => Pcurve::Inverse {a:ua,b:ub},
                    } } else if flip { Pcurve::Inverse {a:ub,b:ua} } else { Pcurve::Inverse {a:ua,b:ub} };
                    let along = !c.reversed != flip;
                    let (from,to) = if c.reversed { (ub,ua) } else { (ua,ub) };
                    halves.push(Half {edge:r,along,pcurve,from,to});
                    on.push(r);
                }
            } }
            // the curves the other solid's faces leave on this one, both ways round
            for &(e,x,y) in &ssi_edges {
                if (if s == 0 { x } else { y }) != fi { continue }
                for &(piece,_) in &pieces[e] {
                    let (r,_) = rep[piece as usize];
                    if on.contains(&r) { continue }
                    on.push(r);
                    let w = out[r as usize];
                    let EdgeCurve::Curve(c) = w.curve else { continue };
                    // the parameters from the middle out, each step unwrapped from the last
                    let n = 32;
                    let mid = (w.t[0]+w.t[1])/2.;
                    let start = loc.uv(fi,c.point(mid));
                    let walk = |to: f64| {
                        let mut uv = start;
                        for j in 1..=n {
                            let t = mid+(to-mid)*j as f64/n as f64;
                            uv = unwrap(f.surface.inverse(c.point(t)),uv,f.surface.periods());
                        }
                        uv
                    };
                    let (ua,ub) = (walk(w.t[0]),walk(w.t[1]));
                    let pcurve = Pcurve::Inverse {a:ua,b:ub};
                    halves.push(Half {edge:r,along:true,pcurve:pcurve.clone(),from:ua,to:ub});
                    halves.push(Half {edge:r,along:false,pcurve,from:ub,to:ua});
                }
            }
            for piece in split(f,&halves,&out,&pool)? {
                // place a point inside the piece against the other solid
                let p = f.surface.point(inside(&piece,f,&out,&pool)?);
                let place = located[1-s].solid_place(p);
                if place == Place::On {
                    return Err(format!("a piece of a {} lies on the other solid's boundary",f.surface.kind()))
                }
                let keep = match (op,s,place) {
                    (Op::Union,_,Place::Out) | (Op::Cut,0,Place::Out) | (Op::Cut,1,Place::In) | (Op::Common,_,Place::In) => true,
                    _ => false,
                };
                if !keep { continue }
                let flip = op == Op::Cut && s == 1;
                let loops: Vec<Vec<Coedge>> = piece.iter().map(|cycle| {
                    let mut l: Vec<Coedge> = cycle.iter().map(|h| Coedge {edge:h.edge,reversed:!h.along,pcurve:h.pcurve.clone()}).collect();
                    if flip { l = l.into_iter().rev().map(|c| Coedge {reversed:!c.reversed,..c}).collect(); }
                    l
                }).collect();
                for l in &loops { for c in l { used[c.edge as usize] = true; } }
                kept.push(Face {surface:f.surface,reversed:f.reversed != flip,loops,name:f.name.clone()});
            }
        }
    }
    // 6. the result: the vertices and edges its faces use, renumbered
    let mut result = Brep::default();
    let mut vmap: BTreeMap<u32,u32> = BTreeMap::new();
    let mut emap: BTreeMap<u32,u32> = BTreeMap::new();
    for (i,e) in out.iter().enumerate() {
        if !used[i] { continue }
        let mut v = [0;2];
        for k in 0..2 { v[k] = *vmap.entry(e.v[k]).or_insert_with(|| result.vertex(pool.pts[e.v[k] as usize])); }
        emap.insert(i as u32,result.edge(e.curve,e.t,v));
    }
    for mut f in kept {
        for l in &mut f.loops { for c in l { c.edge = emap[&c.edge]; } }
        result.faces.push(f);
    }
    Ok(result)
}

/// The parameters along a half, at `n + 1` points from its start.
fn samples(h: &Half,f: &Face,out: &[WEdge],pool: &Pool,n: usize) -> Vec<Uv> {
    (0..=n).map(|j| at(h,f,out,pool,j as f64/n as f64)).collect()
}

/// The parameters a fraction `s` of the way along a half.
fn at(h: &Half,f: &Face,out: &[WEdge],pool: &Pool,s: f64) -> Uv {
    let e = &out[h.edge as usize];
    {
        let t = if h.along { e.t[0]+(e.t[1]-e.t[0])*s } else { e.t[1]-(e.t[1]-e.t[0])*s };
        let a = if h.along { h.from } else { h.to };
        let b = if h.along { h.to } else { h.from };
        let frac = (t-e.t[0])/(e.t[1]-e.t[0]);
        let near = [a[0]+frac*(b[0]-a[0]),a[1]+frac*(b[1]-a[1])];
        match h.pcurve {
            Pcurve::Line {..} => near,
            Pcurve::Inverse {..} => unwrap(f.surface.inverse(e.point(t,pool)),near,f.surface.periods()),
        }
    }
}

/// The face's parameters mirrored where it is reversed, so the material is always on the left.
fn oriented(f: &Face,uv: Uv) -> Uv { if f.reversed { [-uv[0],uv[1]] } else { uv } }

/// The face split into pieces by its halves: each piece its outer cycle and its holes.
fn split(f: &Face,halves: &[Half],out: &[WEdge],pool: &Pool) -> Result<Vec<Vec<Vec<Half>>>,String> {
    // nodes: a vertex at one place in the parameters (a seam's vertex is at two)
    let mut nodes: Vec<(u32,Uv)> = Vec::new();
    let node_of = |v: u32,uv: Uv,nodes: &mut Vec<(u32,Uv)>| -> usize {
        if let Some(i) = nodes.iter().position(|&(w,p)| w == v && (p[0]-uv[0]).abs() < 1e-7 && (p[1]-uv[1]).abs() < 1e-7) { return i }
        nodes.push((v,uv));
        nodes.len()-1
    };
    let ends: Vec<(usize,usize)> = halves.iter().map(|h| {
        let e = &out[h.edge as usize];
        let (v0,v1) = if h.along { (e.v[0],e.v[1]) } else { (e.v[1],e.v[0]) };
        (node_of(v0,h.from,&mut nodes),node_of(v1,h.to,&mut nodes))
    }).collect();
    // each half's direction leaving its start and arriving at its end, as lengths, oriented
    let dir = |h: &Half,at_start: bool| -> f64 {
        let (p,q) = if at_start { (h.from,at(h,f,out,pool,1e-4)) } else { (h.to,at(h,f,out,pool,1.-1e-4)) };
        let (_,su,sv) = f.surface.d1(p);
        let (ku,kv) = (norm(su).max(1e-12),norm(sv).max(1e-12));
        let (p,q) = (oriented(f,p),oriented(f,q));
        ((q[1]-p[1])*kv).atan2((q[0]-p[0])*ku)
    };
    let leave: Vec<f64> = halves.iter().map(|h| dir(h,true)).collect();
    let arrive_back: Vec<f64> = halves.iter().map(|h| dir(h,false)).collect();
    let mut outgoing: Vec<Vec<usize>> = vec![Vec::new();nodes.len()];
    for (i,&(s,_)) in ends.iter().enumerate() { outgoing[s].push(i); }
    let mut seen = vec![false;halves.len()];
    let mut cycles: Vec<Vec<usize>> = Vec::new();
    for start in 0..halves.len() {
        if seen[start] { continue }
        let mut cycle = Vec::new();
        let mut h = start;
        loop {
            if seen[h] {
                if h == start { break }
                return Err(format!("a {}'s pieces do not close",f.surface.kind()))
            }
            seen[h] = true;
            cycle.push(h);
            // at the end node, the next half clockwise from the way back along this one
            let node = ends[h].1;
            let back = arrive_back[h];
            let twin = |g: usize| halves[g].edge == halves[h].edge && halves[g].along != halves[h].along;
            let mut best: Option<(f64,usize)> = None;
            for &g in &outgoing[node] {
                if twin(g) && outgoing[node].len() > 1 { continue }
                // the clockwise turn from `back` to g's leaving direction, in (0, 2π]
                let mut turn = back-leave[g];
                while turn <= 1e-12 { turn += TAU; }
                while turn > TAU+1e-12 { turn -= TAU; }
                if best.is_none_or(|(b,_)| turn < b) { best = Some((turn,g)); }
            }
            let Some((_,g)) = best else { return Err(format!("a {}'s pieces end nowhere",f.surface.kind())) };
            h = g;
        }
        cycles.push(cycle);
    }
    // outer cycles turn counter-clockwise (the material on their left), holes clockwise
    let polys: Vec<Vec<Uv>> = cycles.iter().map(|c| c.iter().flat_map(|&h| {
        let pts = samples(&halves[h],f,out,pool,24);
        pts[..pts.len()-1].iter().map(|&p| oriented(f,p)).collect::<Vec<_>>()
    }).collect()).collect();
    let area = |p: &[Uv]| (0..p.len()).map(|i| { let (a,b) = (p[i],p[(i+1)%p.len()]); a[0]*b[1]-a[1]*b[0] }).sum::<f64>()/2.;
    let areas: Vec<f64> = polys.iter().map(|p| area(p)).collect();
    let mut pieces: Vec<(usize,Vec<usize>)> = (0..cycles.len()).filter(|&i| areas[i] > 0.).map(|i| (i,vec![i])).collect();
    for i in (0..cycles.len()).filter(|&i| areas[i] <= 0.) {
        // a hole goes in the smallest outer cycle about it
        let probe = polys[i][0];
        let mut host: Option<usize> = None;
        for (k,(o,_)) in pieces.iter().enumerate() {
            if winding(&polys[*o],probe) != 0 && host.is_none_or(|h| areas[*o] < areas[pieces[h].0]) { host = Some(k); }
        }
        match host {
            Some(k) => pieces[k].1.push(i),
            None => return Err(format!("a {} has a hole in no piece",f.surface.kind())),
        }
    }
    Ok(pieces.into_iter().map(|(_,cs)| cs.iter().map(|&c| cycles[c].iter().map(|&h| halves[h].clone()).collect()).collect()).collect())
}

fn winding(poly: &[Uv],p: Uv) -> i32 {
    let mut w = 0;
    let n = poly.len();
    for i in 0..n {
        let (a,b) = (poly[i],poly[(i+1)%n]);
        let cross = (b[0]-a[0])*(p[1]-a[1])-(p[0]-a[0])*(b[1]-a[1]);
        if a[1] <= p[1] { if b[1] > p[1] && cross > 0. { w += 1; } }
        else if b[1] <= p[1] && cross < 0. { w -= 1; }
    }
    w
}

/// A point well inside a piece, in its face's parameters: the middle of the widest span a level
/// line across it has inside it.
fn inside(piece: &[Vec<Half>],f: &Face,out: &[WEdge],pool: &Pool) -> Result<Uv,String> {
    let polys: Vec<Vec<Uv>> = piece.iter().map(|cycle| cycle.iter().flat_map(|h| {
        let pts = samples(h,f,out,pool,24);
        pts[..pts.len()-1].to_vec()
    }).collect()).collect();
    let (mut vlo,mut vhi) = (f64::INFINITY,f64::NEG_INFINITY);
    for p in polys.iter().flatten() { vlo = vlo.min(p[1]); vhi = vhi.max(p[1]); }
    let mut best: Option<(f64,Uv)> = None;
    for k in 1..16 {
        let v = vlo+(vhi-vlo)*(k as f64+0.37)/16.;
        let mut xs: Vec<f64> = Vec::new();
        for p in &polys {
            for i in 0..p.len() {
                let (a,b) = (p[i],p[(i+1)%p.len()]);
                if (a[1] <= v) != (b[1] <= v) { xs.push(a[0]+(v-a[1])/(b[1]-a[1])*(b[0]-a[0])); }
            }
        }
        xs.sort_by(|a,b| a.total_cmp(b));
        for w in xs.chunks(2) {
            if w.len() == 2 && best.is_none_or(|(width,_)| w[1]-w[0] > width) { best = Some((w[1]-w[0],[(w[0]+w[1])/2.,v])); }
        }
    }
    best.map(|b| b.1).ok_or(format!("a piece of a {} has no inside",f.surface.kind()))
}
