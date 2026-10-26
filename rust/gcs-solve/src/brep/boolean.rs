//! Booleans of two solids (docs/rust-kernel-plan.md, rung 1): every edge of each split where it
//! crosses the other's faces, every pair of faces intersected and the curves kept where they lie in
//! both, every face split by what now lies on it — traced in its own parameters, the material on
//! the left — each piece placed against the other solid, and the pieces the operation keeps
//! assembled, their edges shared by construction.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::geom::{around,Box3,Curve,Surface,Uv,V};
use super::query::{crossings_in,curve_surface_within,Located,Meets,Outline,Place};
use super::ssi::{intersect,Ssi};
use super::topo::{reverse_loop,unwrap,Brep,Coedge,EdgeCurve,Face,Pcurve};
use crate::space::{distance,norm};
use std::cell::Cell;
use std::collections::BTreeMap;

const TAU: f64 = std::f64::consts::TAU;

/// What a Boolean keeps.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Op { Union,Cut,Common }

/// One of a Boolean's two solids: `A`, the first (the solid a run of Booleans hands on), or `B`.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
enum Side { A,B }

impl Side {
    const BOTH: [Side;2] = [Side::A,Side::B];
    fn other(self) -> Side { match self { Side::A => Side::B,Side::B => Side::A } }
    fn name(self) -> &'static str { match self { Side::A => "A",Side::B => "B" } }
}

/// A value for each of a Boolean's two solids, read by its `Side`.
#[derive(Clone,Copy,Debug)]
struct Pair<T>([T;2]);

impl<T> Pair<T> {
    /// Each side's value, A's made first.
    fn new(mut f: impl FnMut(Side) -> T) -> Pair<T> { Pair([f(Side::A),f(Side::B)]) }
}

impl<T> std::ops::Index<Side> for Pair<T> {
    type Output = T;
    fn index(&self,s: Side) -> &T { &self.0[s as usize] }
}

impl<T> std::ops::IndexMut<Side> for Pair<T> {
    fn index_mut(&mut self,s: Side) -> &mut T { &mut self.0[s as usize] }
}

/// Points merged where they come within the tolerance of one another.
struct Pool { pts: Vec<V>,tol: f64,grid: crate::space::Grid }

impl Pool {
    fn new(tol: f64) -> Pool { Pool {pts:Vec::new(),tol,grid:crate::space::Grid::new(4.*tol)} }
    fn at(&mut self,p: V) -> u32 {
        assert!(p.iter().all(|x| x.is_finite() && x.abs() < 1e12),"a vertex at {p:?}");
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

/// A root an edge meets a face of the other solid at, by its distance from the face's surface:
/// the working edge, the vertex the root minted, its parameter, the edge's solid and index there,
/// and the face met.
struct Loose { edge: usize,vertex: u32,t: f64,side: Side,index: usize,face: usize }

/// An edge of the working set: a stretch of a curve (or a pole) between two pooled vertices.
#[derive(Clone,Debug)]
struct WEdge { curve: EdgeCurve,t: [f64;2],v: [u32;2] }

impl WEdge {
    fn point(&self,t: f64,pool: &Pool) -> V {
        match &self.curve { EdgeCurve::Curve(c) => c.point(t),EdgeCurve::Degenerate => pool.pts[self.v[0] as usize] }
    }
}

/// One way along an edge on a face, with its curve in the face's parameters.
#[derive(Clone,Debug)]
struct Half { edge: u32,along: bool,pcurve: Pcurve,from: Uv,to: Uv }

/// Whether two boxes meet, each grown by `pad`.
pub(crate) fn overlap(a: &([f64;3],[f64;3]),b: &([f64;3],[f64;3]),pad: f64) -> bool {
    (0..3).all(|k| a.0[k] <= b.1[k]+pad && b.0[k] <= a.1[k]+pad)
}

/// A closed curve's parameter `t` moved by whole periods into `[lo, lo + period)`.

/// An edge lying along an extrusion's own curve at one height — the same B-spline moved along
/// the sweep, as a plane square to it cuts it (`ssi`) — is the iso-line `v = h` of the face, its
/// parameter the face's `u`: its curve in the face's parameters is that straight line, exactly,
/// where inverting each point onto the surface is a Newton search per reading.
fn iso_pcurve(surface: &super::geom::Surface,curve: &super::geom::Curve,t: [f64;2]) -> Option<(Uv,Uv)> {
    use super::geom::{Curve,Surface};
    let (Surface::Extrusion(f,c),Curve::BSpline(e)) = (surface,curve) else { return None };
    let Curve::BSpline(c) = &**c else { return None };
    if c.degree != e.degree || c.knots != e.knots || c.weights != e.weights || c.poles.len() != e.poles.len() { return None }
    let lift = crate::space::dot(crate::space::sub(e.poles[0],c.poles[0]),f.z);
    let scale = 1.+c.poles.iter().map(|&p| norm(p)).fold(0.,f64::max);
    c.poles.iter().zip(&e.poles).all(|(&p,&q)| norm(crate::space::sub(q,crate::space::add(p,crate::space::scale(f.z,lift)))) <= 1e-12*scale)
        .then(|| ([t[0],lift],[t[1],lift]))
}

/// `a` combined with `b` by `op`, to `tol` (a length).
/// Two B-reps' faces split by everything the other lays on them, nothing yet kept or dropped: the
/// working edges and their pooled vertices, and each face's pieces (which solid, which face, its loops).
struct Arranged<'a> {
    solids: Pair<&'a Brep>,
    located: Pair<Located<'a>>,
    pool: Pool,
    out: Vec<WEdge>,
    pieces: Vec<(Side,usize,Vec<Vec<Half>>)>,
    /// Each solid's faces clear of the other: whose boxes miss the box the other's faces fill, so
    /// nothing of the other meets them and all of each lies outside it (#589).
    clear: Pair<Vec<bool>>,
}

/// **Two solids sharing a face, joined there** (the pieces of a chain of fillets, each ending in
/// the section the next begins in): where a face of `a` and one of `b` are the same face — one
/// surface, material either side of it, their boundaries one curve within `tol` — the two faces go
/// and their edges become one, used by the faces beside them. The boundaries are matched run by
/// run, a run going from one vertex both have to the next: one edge each side (the common case),
/// or edges one side holds in pieces, or both, cut at different points (a fitted net seamed where
/// its loop starts, #424) — each side's edges lying along the other's, end for end and through the
/// middle. Each side is cut where the other has a vertex within a run, so the two sides' edges
/// pair one to one whichever of `a` and `b` holds the pieces. Nothing is intersected, so `a` and
/// `b` must meet nowhere else: the caller's to know. `None` where they share no such face.
pub fn glued(a: &Brep,b: &Brep,tol: f64) -> Option<Brep> {
    use super::geom::Surface;
    // a run of a face's boundary: its edges in the order it walks them, each with whether it walks
    // it along its curve
    type Walk = Vec<(u32,bool)>;
    let near = |p: V,q: V| distance(p,q) <= tol;
    let middle = |s: &Brep,e: usize| s.edges[e].point(0.5*(s.edges[e].t[0]+s.edges[e].t[1]),&s.vertices);
    let start = |s: &Brep,(e,along): (u32,bool)| { let v = s.edges[e as usize].v; s.vertices[v[usize::from(!along)] as usize].p };
    let end = |s: &Brep,(e,along): (u32,bool)| start(s,(e,!along));
    // a face's outward normal at `p` (on its surface): a plane's everywhere; a curved one's (the
    // seam's normal cylinder two pieces of a chain's section share, #422) where it is read
    let outward = |f: &Face,p: V| -> V {
        let n = match f.surface { Surface::Plane(fr) => fr.z,_ => f.surface.gradient(p) };
        if f.reversed { n.map(|x| -x) } else { n }
    };
    // a point of the face to read both faces' normals at: its first edge's middle
    let at = |s: &Brep,f: &Face| f.loops.first().and_then(|l| l.first()).map(|c| middle(s,c.edge as usize));
    // whether `p` lies on `s`'s edge `e`, within its stretch: its curve's nearest point (a traced
    // meeting and a fitted one parametrise alike only at their ends), or its middle where the two
    // are parametrised alike (a pole's)
    let on_edge = |s: &Brep,e: usize,p: V| {
        if near(middle(s,e),p) { return true }
        let e = &s.edges[e];
        let EdgeCurve::Curve(c) = &e.curve else { return false };
        let mut t = c.inverse(p);
        if let Some(period) = c.period() { t = around(t,e.t[0],period); }
        let slack = 1e-9*(e.t[1]-e.t[0]).abs().max(1.);
        t >= e.t[0]-slack && t <= e.t[1]+slack && near(c.point(t),p)
    };
    let plane = |f: &Face| matches!(f.surface,Surface::Plane(_));
    let corners = |s: &Brep,f: &Face| -> Vec<V> { f.loops.iter().flatten().map(|c| start(s,(c.edge,!c.reversed))).collect() };
    // a face's loops cut into runs at the vertices the other face has too; `None` where a loop has none
    let runs = |s: &Brep,f: &Face,other: &[V]| -> Option<Vec<Walk>> {
        let mut out = Vec::new();
        for l in &f.loops {
            let walk: Walk = l.iter().map(|c| (c.edge,!c.reversed)).collect();
            let cut: Vec<bool> = walk.iter().map(|&w| other.iter().any(|&q| near(start(s,w),q))).collect();
            let first = cut.iter().position(|&c| c)?;
            let mut run = Vec::new();
            for k in (first..walk.len()).chain(0..first) {
                if cut[k] && !run.is_empty() { out.push(std::mem::take(&mut run)); }
                run.push(walk[k]);
            }
            out.push(run);
        }
        Some(out)
    };
    // the way a closed run sets out from its vertex
    let heading = |s: &Brep,(e,along): (u32,bool)| -> Option<V> {
        let e = &s.edges[e as usize];
        let EdgeCurve::Curve(c) = &e.curve else { return None };
        let d = c.tangent(e.t[usize::from(!along)]);
        Some(if along { d } else { d.map(|x| -x) })
    };
    // `a`'s run `ra` and `b`'s `rb` (walked the same way) one curve: from one end to the other,
    // each one's vertices within it on an edge of the other's, where that one is cut (whose, which
    // edge, the point), and each one's edges through the middle on the other's
    let along = |ra: &Walk,rb: &Walk| -> Option<Vec<(usize,u32,V)>> {
        let (a0,b0) = (start(a,ra[0]),start(b,rb[0]));
        if !near(a0,b0) || !near(end(a,ra[ra.len()-1]),end(b,rb[rb.len()-1])) { return None }
        if near(a0,end(a,ra[ra.len()-1])) && heading(a,ra[0]).zip(heading(b,rb[0])).is_some_and(|(p,q)| crate::space::dot(p,q) < 0.) {
            return None
        }
        if let ([(ea,_)],[(eb,_)]) = (&ra[..],&rb[..]) { return on_edge(a,*ea as usize,middle(b,*eb as usize)).then(Vec::new) }
        let mut cuts = Vec::new();
        for (side,(s,r),(o,ro)) in [(0,(b,rb),(a,ra)),(1,(a,ra),(b,rb))] {
            for &w in &r[1..] {
                let p = start(s,w);
                let &(e,_) = ro.iter().find(|&&(e,_)| on_edge(o,e as usize,p))?;
                cuts.push((side,e,p));
            }
            if !r.iter().all(|&(e,_)| { let m = middle(s,e as usize); ro.iter().any(|&(f,_)| on_edge(o,f as usize,m)) }) { return None }
        }
        Some(cuts)
    };
    let reversed = |w: &Walk| -> Walk { w.iter().rev().map(|&(e,x)| (e,!x)).collect() };
    // each face of `b` the same as one of `a`: its runs paired with that one's, each walked as `a`'s
    // is, and where either side is to be cut
    let (mut shared_a,mut shared_b) = (std::collections::BTreeSet::new(),std::collections::BTreeSet::new());
    let mut pairs: Vec<(Walk,Walk)> = Vec::new();
    let mut cuts: [BTreeMap<u32,Vec<V>>;2] = Default::default();
    for (i,fa) in a.faces.iter().enumerate() {
        let Some(p) = at(a,fa) else { continue };
        let na = outward(fa,p);
        for (j,fb) in b.faces.iter().enumerate() {
            if shared_b.contains(&j) || plane(fa) != plane(fb) || !super::ssi::same(&fa.surface,&fb.surface,tol) { continue }
            if crate::space::dot(na,outward(fb,p)) > if plane(fa) { -1.+1e-9 } else { 0. } { continue }
            let (Some(ra),Some(rb)) = (runs(a,fa,&corners(b,fb)),runs(b,fb,&corners(a,fa))) else { continue };
            if ra.len() != rb.len() { continue }
            let mut used = vec![false;ra.len()];
            let mut found = Vec::with_capacity(rb.len());
            for r in &rb {
                // the faces facing apart, `b`'s run is most often walked the other way round
                let hit = [reversed(r),r.clone()].into_iter().find_map(|w|
                    (0..ra.len()).filter(|&k| !used[k]).find_map(|k| along(&ra[k],&w).map(|c| (k,c))).map(|(k,c)| (k,w,c)));
                let Some((k,w,c)) = hit else { break };
                used[k] = true;
                found.push((k,w,c));
            }
            if found.len() != rb.len() { continue }
            shared_a.insert(i);
            shared_b.insert(j);
            for (k,w,c) in found {
                for (side,e,p) in c { cuts[side].entry(e).or_default().push(p); }
                pairs.push((ra[k].clone(),w));
            }
            break;
        }
    }
    if shared_a.is_empty() { return None }
    // each side cut where the other has a vertex within a run: then the runs pair edge for edge
    let mut out = a.clone();
    let pieces_a = cut_at(&mut out,&cuts[0],tol)?;
    let mut cut_b = std::borrow::Cow::Borrowed(b);
    let pieces_b = if cuts[1].is_empty() { BTreeMap::new() } else { cut_at(cut_b.to_mut(),&cuts[1],tol)? };
    let b = &*cut_b;
    let expand = |w: &Walk,pieces: &BTreeMap<u32,Vec<u32>>| -> Walk {
        w.iter().flat_map(|&(e,x)| {
            let p = pieces.get(&e).map_or_else(|| vec![e],Vec::clone);
            if x { p.into_iter().map(|e| (e,true)).collect::<Vec<_>>() } else { p.into_iter().rev().map(|e| (e,false)).collect() }
        }).collect()
    };
    // each of `b`'s shared edges: `a`'s it is, whether it runs the other way, and whether either was
    // in pieces
    let mut edge_of: BTreeMap<usize,(u32,bool,bool)> = BTreeMap::new();
    for (ra,rb) in &pairs {
        let pieced = ra.len() > 1 || rb.len() > 1;
        let (ra,rb) = (expand(ra,&pieces_a),expand(rb,&pieces_b));
        if ra.len() != rb.len() { return None }
        for (&(ea,xa),&(eb,xb)) in ra.iter().zip(&rb) {
            if !near(end(&out,(ea,xa)),end(b,(eb,xb))) { return None }
            edge_of.insert(eb as usize,(ea,xa != xb,pieced));
        }
    }
    // `b`'s vertices: those of a shared edge are `a`'s, the rest its own. A vertex of `b` met by
    // two of `a`'s (two pieces `b` joins, each with its own copy of the point it ends at) makes
    // them one
    let mut vertex_of: BTreeMap<u32,u32> = BTreeMap::new();
    let mut same = crate::graph::UnionFind::new(out.vertices.len());
    let mut merged = false;
    for (&eb,&(ea,flip,_)) in &edge_of {
        let (vb,va) = (b.edges[eb].v,out.edges[ea as usize].v);
        for k in 0..2 {
            let va = va[if flip { 1-k } else { k }];
            match vertex_of.get(&vb[k]) {
                Some(&w) if same.find(w as usize) != same.find(va as usize) => { same.union(w as usize,va as usize); merged = true; }
                Some(_) => {}
                None => { vertex_of.insert(vb[k],va); }
            }
        }
    }
    if merged {
        for e in &mut out.edges { e.v = e.v.map(|v| same.find(v as usize) as u32); }
        for v in vertex_of.values_mut() { *v = same.find(*v as usize) as u32; }
    }
    for (i,v) in b.vertices.iter().enumerate() {
        if !vertex_of.contains_key(&(i as u32)) { vertex_of.insert(i as u32,out.vertices.len() as u32); out.vertices.push(*v); }
    }
    // `b`'s edges: a shared one is `a`'s (its tolerance the larger), the rest its own
    let mut new_edge: Vec<u32> = Vec::with_capacity(b.edges.len());
    for (i,e) in b.edges.iter().enumerate() {
        if let Some(&(ea,..)) = edge_of.get(&i) {
            let t = &mut out.edges[ea as usize].tol;
            *t = t.max(e.tol);
            new_edge.push(ea);
        } else {
            let mut e = e.clone();
            e.v = e.v.map(|v| vertex_of[&v]);
            out.edges.push(e);
            new_edge.push((out.edges.len()-1) as u32);
        }
    }
    out.faces = std::mem::take(&mut out.faces).into_iter().enumerate().filter(|(i,_)| !shared_a.contains(i)).map(|(_,f)| f).collect();
    for (j,f) in b.faces.iter().enumerate() {
        if shared_b.contains(&j) { continue }
        let mut f = f.clone();
        for c in f.loops.iter_mut().flatten() {
            // a use of an edge now `a`'s: its pcurve by its ends (a pcurve read at its own edge's
            // parameter reads another's wrongly), from the other end where that runs the other
            // way, and inverted between them where either was in pieces (a line in the face's
            // parameters along one curve's parameter is none along the other's)
            if let Some(&(_,flip,pieced)) = edge_of.get(&(c.edge as usize)) {
                if flip || pieced || matches!(c.pcurve,Pcurve::Curve(_)) {
                    let old = &b.edges[c.edge as usize];
                    let (u0,u1) = (c.pcurve.at(old.t[0],old,&f.surface,&b.vertices),c.pcurve.at(old.t[1],old,&f.surface,&b.vertices));
                    let (a,z) = if flip { (u1,u0) } else { (u0,u1) };
                    c.pcurve = match c.pcurve { Pcurve::Line {..} if !pieced => Pcurve::Line {a,b:z},_ => Pcurve::Inverse {a,b:z} };
                    c.reversed ^= flip;
                }
            }
            c.edge = new_edge[c.edge as usize];
        }
        out.faces.push(f);
    }
    Some(out)
}

/// `s`'s edges cut at the points listed against each (on its curve, within its stretch; those
/// within `tol` of one another one): each edge's pieces in order along it, the first keeping its
/// index, and each use of it a use of each piece, its pcurve cut where the edge is. `None` where an
/// edge so listed is a pole.
fn cut_at(s: &mut Brep,cuts: &BTreeMap<u32,Vec<V>>,tol: f64) -> Option<BTreeMap<u32,Vec<u32>>> {
    let mut pieces = BTreeMap::new();
    for (&e,ps) in cuts {
        let edge = s.edges[e as usize].clone();
        let EdgeCurve::Curve(c) = &edge.curve else { return None };
        let mut ts: Vec<f64> = ps.iter().map(|&p| { let t = c.inverse(p); c.period().map_or(t,|period| around(t,edge.t[0],period)) })
            .filter(|&t| t > edge.t[0] && t < edge.t[1]).collect();
        ts.sort_by(f64::total_cmp);
        ts.dedup_by(|x,y| distance(c.point(*x),c.point(*y)) <= tol);
        if ts.is_empty() { continue }
        let t: Vec<f64> = std::iter::once(edge.t[0]).chain(ts.iter().copied()).chain([edge.t[1]]).collect();
        let v: Vec<u32> = std::iter::once(edge.v[0]).chain(ts.iter().map(|&t| s.vertex(c.point(t)))).chain([edge.v[1]]).collect();
        let mut ids = vec![e];
        s.edges[e as usize].t = [t[0],t[1]];
        s.edges[e as usize].v = [v[0],v[1]];
        for k in 1..t.len()-1 {
            s.edges.push(super::topo::Edge {curve:edge.curve.clone(),t:[t[k],t[k+1]],v:[v[k],v[k+1]],tol:edge.tol});
            ids.push((s.edges.len()-1) as u32);
        }
        let Brep {vertices,faces,..} = s;
        for f in faces.iter_mut() {
            for l in f.loops.iter_mut() {
                if !l.iter().any(|c| c.edge == e) { continue }
                let mut walk = Vec::with_capacity(l.len()+ids.len());
                for c in l.drain(..) {
                    if c.edge != e { walk.push(c); continue }
                    let uv: Vec<Uv> = match c.pcurve {
                        Pcurve::Curve(_) => Vec::new(),
                        _ => t.iter().map(|&t| c.pcurve.at(t,&edge,&f.surface,vertices)).collect(),
                    };
                    let piece = |k: usize| Coedge {edge:ids[k],reversed:c.reversed,pcurve:match &c.pcurve {
                        Pcurve::Line {..} => Pcurve::Line {a:uv[k],b:uv[k+1]},
                        Pcurve::Inverse {..} => Pcurve::Inverse {a:uv[k],b:uv[k+1]},
                        Pcurve::Curve(p) => Pcurve::Curve(p.clone()),
                    }};
                    if c.reversed { walk.extend((0..ids.len()).rev().map(piece)) } else { walk.extend((0..ids.len()).map(piece)) }
                }
                *l = walk;
            }
        }
        pieces.insert(e,ids);
    }
    Some(pieces)
}

/// One step of the body rule: `solid` combined with `operand` by `op`, to a tolerance of the two's
/// size (never under `floor`, mm), and checked.
pub fn combined(solid: &Brep,operand: &Brep,op: Op,floor: f64) -> Result<Brep,super::KernelError> {
    Ok(combined_known(solid,None,operand,op,floor)?.0)
}

/// **What a Boolean knows of the faces it hands on** (#589), beside its result's faces: each one's
/// box in space where it is a face of an operand carried whole (`Arranged::clear`: nothing of the
/// other met it), and whether it was checked whole as a face of a result before (`Brep::check`).
/// A run of Booleans on one solid (a body's operands, one after another) hands it from each to the
/// next, so a face far from every operand is boxed and checked once, not once an operand.
#[derive(Clone,Debug,Default)]
pub struct Known { boxes: Vec<Option<Box3>>,checked: Vec<bool> }

impl Known {
    /// Face `fi`'s box, where it is known.
    fn face_box(&self,fi: usize) -> Option<Box3> { self.boxes.get(fi).copied().flatten() }
}

/// `combined`, `solid`'s faces known as `known` says (the last step of a run handed on), and what
/// the result's are.
pub fn combined_known(solid: &Brep,known: Option<&Known>,operand: &Brep,op: Op,floor: f64)
    -> Result<(Brep,Known),super::KernelError> {
    let tol = super::geom::tolerance_over(solid.size().max(operand.size()),floor);
    let (mut out,mut known) = boolean_known(solid,known,operand,op,tol)?;
    // material touching itself along a line (two rounds of separate statements meeting where the
    // body they each read leaves them so, #218): each such edge held twice, a solid's boundary
    // pinched there
    if let Err(e) = out.check_settled(10.*tol,&known.checked) {
        if e.kind != super::Kind::NotManifold || !matches!(out.unpinched(),Ok(n) if n > 0) { return Err(e) }
        out.check(10.*tol)?;
        // its edges split: nothing of it is handed on
        known.boxes.clear();
    }
    known.checked = vec![true;out.faces.len()];
    Ok((out,known))
}

/// `a` combined with `b` by `op`, to `tol` (a length).
pub fn boolean(a: &Brep,b: &Brep,op: Op,tol: f64) -> Result<Brep,String> { Ok(boolean_known(a,None,b,op,tol)?.0) }

/// `boolean`, `a`'s faces known as `known` says, and what the result's are (`Known`): the boxes
/// of those carried whole, and which of them were checked (`a`'s that were).
fn boolean_known(a: &Brep,known: Option<&Known>,b: &Brep,op: Op,tol: f64) -> Result<(Brep,Known),String> {
    let Arranged {solids,located,pool,out,pieces,clear} = arrange(a,known,b,tol)?;
    let mut kept: Vec<Face> = Vec::new();
    let mut handed = Known::default();
    for (s,fi,piece) in pieces {
        let f = &solids[s].faces[fi];
        // a face clear of the other solid is one piece, whole and outside it: no point of it need
        // be placed (a large solid's faces far from a small one, the most of them, #589)
        if clear[s][fi] {
            if op == Op::Union || op == Op::Cut && s == Side::A {
                kept.push(piece_face(f,&piece,false));
                handed.boxes.push(Some(located[s].face_box(fi)));
                handed.checked.push(s == Side::A && known.is_some_and(|k| k.checked.get(fi) == Some(&true)));
            }
            continue
        }
        // place a point inside the piece against the other solid: the first of its inner
        // points not on the other's boundary (a piece may touch it tangentially), or one on
        // it where the other has a face on this surface
        let mut inner = reading::inside(&piece,f,&out,&pool).map(|q| q.point()).peekable();
        let Some(&first) = inner.peek() else { return Err(format!("a piece of a {} has no inside",f.surface.kind())) };
        let other = s.other();
        let (p,place) = inner.map(|p| (p,located[other].solid_place(p)))
            .find(|&(p,place)| place != Place::On || (0..solids[other].faces.len()).any(|g|
                super::ssi::same(&f.surface,&solids[other].faces[g].surface,tol) && located[other].face_place(g,p) == Place::In))
            .unwrap_or((first,Place::On));
        let keep = if place == Place::On {
            // on a face of the other on the same surface: kept once, from A, where the two
            // face the same way and the operation keeps that side
            let partner = (0..solids[other].faces.len()).find(|&g| {
                super::ssi::same(&f.surface,&solids[other].faces[g].surface,tol)
                    && located[other].face_place(g,p) == Place::In
            });
            let Some(g) = partner else {
                // every point read inside the piece on the other's boundary, and on no face of
                // the other's on its surface: a contact the arrangement did not cut it along
                let named = |f: &Face| match f.name.is_empty() {
                    true => format!("a {}",f.surface.kind()),
                    false => format!("`{}` (a {})",f.name,f.surface.kind()),
                };
                let near: Vec<String> = solids[other].faces.iter().enumerate()
                    .filter(|&(g,h)| h.surface.implicit(p).abs() <= tol && located[other].face_place(g,p) != Place::Out)
                    .map(|(_,h)| named(h)).collect();
                let near = if near.is_empty() { "the other solid's boundary".to_string() } else { near.join(" and ") };
                return Err(format!("a piece of {} touches {near} at {p:?} along no face of either: \
                    move the one off the other's face, or into it",named(f)))
            };
            let normal = |face: &Face| {
                let n = face.surface.normal_raw(face.surface.inverse(p));
                if face.reversed { crate::space::scale(n,-1.) } else { n }
            };
            let alike = crate::space::dot(normal(f),normal(&solids[other].faces[g])) > 0.;
            s == Side::A && match op { Op::Union | Op::Common => alike,Op::Cut => !alike }
        } else { match (op,s,place) {
            (Op::Union,_,Place::Out) | (Op::Cut,Side::A,Place::Out) | (Op::Cut,Side::B,Place::In) | (Op::Common,_,Place::In) => true,
            _ => false,
        } };
        if !keep { continue }
        let flip = op == Op::Cut && s == Side::B;
        kept.push(piece_face(f,&piece,flip));
        handed.boxes.push(None);
        handed.checked.push(false);
    }
    Ok((assemble(&out,&pool,kept),handed))
}

/// A piece of face `f` as a face of its own: its cycles as loops, each turned the other way round
/// with the face's sense where `flip` (a piece of the solid a cut takes away).
fn piece_face(f: &Face,piece: &[Vec<Half>],flip: bool) -> Face {
    let loops: Vec<Vec<Coedge>> = piece.iter().map(|cycle| {
        let mut l: Vec<Coedge> = cycle.iter().map(|h| Coedge {edge:h.edge,reversed:!h.along,pcurve:h.pcurve.clone()}).collect();
        if flip { l = reverse_loop(l); }
        l
    }).collect();
    Face {surface:f.surface.clone(),reversed:f.reversed != flip,loops,name:f.name.clone()}
}

/// The faces of `a` and `b` split where the other crosses them (`Arranged`), phase by phase over
/// the state they share (`Arrangement`).
/// `a`'s faces' boxes are taken from `known` where it has them (`Known`).
fn arrange<'a>(a: &'a Brep,known: Option<&Known>,b: &'a Brep,tol: f64) -> Result<Arranged<'a>,String> {
    let mut arr = Arrangement::new(a,known,b,tol);
    // 1. every edge against every face of the other solid: where it crosses, a vertex on both
    let mut crossings = arr.crossings();
    // a root met flatly that is a crossing found besides is that crossing (`flat_roots`)
    let gone = arr.flat_roots(&mut crossings);
    arr.say.lap();
    // 2. every pair of faces: their surfaces' curves, kept where they lie in both faces
    let meetings = arr.meetings(&crossings)?;
    // 2b. every vertex, either solid's or a crossing's, cuts an edge it lies inside, the faces'
    // meetings among them: two edges running along one another (a shared circle, each solid
    // seamed at its own angle; a meeting traced along an edge already there) then share their
    // vertices, and step 4 takes them for one. Each edge boxed by its curve's enclosure, on every core.
    let inside = vertices_inside(&arr.edges,&arr.cuts,&arr.pool,&gone,arr.pad,tol);
    for (we,found) in inside.into_iter().enumerate() { arr.cuts[we].extend(found); }
    arr.say.lap();
    // 3. every original edge split where it was cut: its pieces, in order along it
    let (pieces_of,out) = split_edges(&arr.edges,&arr.cuts);
    // 4. pieces that are one edge merged: the same ends and the same curve between
    let rep = merged(&out,&arr.pool,tol);
    let split = Split {pieces_of,out,rep};
    // 5. each face split by what lies on it, traced in its parameters
    let pieces = arr.faces_split(&meetings,&split)?;
    arr.say.arranged();
    let Arrangement {solids,located,pool,clear,..} = arr;
    Ok(Arranged {solids,located,pool,out:split.out,pieces,clear})
}

/// **The working state `arrange`'s phases share**: the two solids and where each face lies, the
/// vertices pooled, and the working edges — A's, then B's, then the meetings phase 2 lays on the
/// faces — each with the points it is cut at, the two growing together (`add_edge`).
struct Arrangement<'a> {
    solids: Pair<&'a Brep>,
    located: Pair<Located<'a>>,
    pool: Pool,
    edges: Vec<WEdge>,
    /// where each working edge is cut, in step with `edges`: the parameter, and the vertex there
    cuts: Vec<Vec<(f64,u32)>>,
    /// where each solid's own edges begin among `edges`
    first_edge: Pair<usize>,
    /// each face's box
    boxes: Pair<Vec<Box3>>,
    /// the box each solid's faces fill
    reach: Pair<Box3>,
    /// each solid's faces clear of the other (`Arranged::clear`)
    clear: Pair<Vec<bool>>,
    tol: f64,
    /// how far apart two boxes may stand and still be taken to meet
    pad: f64,
    say: Narration,
}

/// Phase 1's findings: every edge against every face of the other solid.
struct Crossings {
    /// each face's vertices where an edge of the other crosses it, or crosses its boundary
    on_face: Pair<Vec<Vec<u32>>>,
    /// each face's edges of the other solid lying in its surface (`Meets::Along`), working indices
    along_face: Pair<Vec<Vec<usize>>>,
    /// each root an edge meets a face at by its distance from the surface, at a vertex of its own
    loose: Vec<Loose>,
}

/// Phase 2's findings: every pair of faces.
struct Meetings {
    /// the working edges laid where two faces meet, each with the two faces
    ssi_edges: Vec<(usize,Pair<usize>)>,
    /// faces of the two on one surface: each splits the other with its boundary
    same_pairs: Vec<Pair<usize>>,
}

/// Phases 3 and 4: every working edge split where it was cut (`split_edges`), and the pieces that
/// are one edge merged (`merged`).
struct Split {
    /// each working edge's pieces in order along it: indices into `out`, and their parameters
    pieces_of: Vec<Vec<(u32,[f64;2])>>,
    out: Vec<WEdge>,
    /// each piece's representative among `out`, and whether it runs the other way round
    rep: Vec<(u32,bool)>,
}

impl<'a> Arrangement<'a> {
    /// Both solids' faces boxed and their edges pooled, nothing yet cut.
    fn new(a: &'a Brep,known: Option<&Known>,b: &'a Brep,tol: f64) -> Arrangement<'a> {
        let solids = Pair([a,b]);
        let located = Pair([Located::boxed(a,tol,&|fi| known.and_then(|k| k.face_box(fi))),Located::new(b,tol)]);
        let mut pool = Pool::new(tol);
        // the working edges: A's, then B's, their vertices pooled
        let mut edges: Vec<WEdge> = Vec::new();
        let mut first_edge = Pair([0usize;2]);
        for s in Side::BOTH {
            first_edge[s] = edges.len();
            let ids: Vec<u32> = solids[s].vertices.iter().map(|v| pool.at(v.p)).collect();
            for e in &solids[s].edges { edges.push(WEdge {curve:e.curve.clone(),t:e.t,v:[ids[e.v[0] as usize],ids[e.v[1] as usize]]}); }
        }
        let cuts = vec![Vec::new();edges.len()];
        let boxes = Pair::new(|s| (0..solids[s].faces.len()).map(|i| located[s].face_box(i)).collect::<Vec<_>>());
        let pad = 4.*tol;
        // the box each solid's faces fill, and the faces of each clear of the other's (`Arranged::clear`)
        let reach = Pair::new(|s| boxes[s].iter().fold(super::geom::EMPTY,|mut r,b| { super::geom::widen(&mut r,*b); r }));
        let clear = Pair::new(|s| boxes[s].iter().map(|bx| !overlap(bx,&reach[s.other()],pad)).collect::<Vec<_>>());
        Arrangement {solids,located,pool,edges,cuts,first_edge,boxes,reach,clear,tol,pad,say:Narration::new()}
    }

    /// Edge `i` of solid `s`'s index among the working edges.
    fn working(&self,s: Side,i: usize) -> usize { self.first_edge[s]+i }

    /// A working edge added, cut nowhere yet: its index.
    fn add_edge(&mut self,curve: EdgeCurve,t: [f64;2],v: [u32;2]) -> usize {
        self.edges.push(WEdge {curve,t,v});
        self.cuts.push(Vec::new());
        self.edges.len()-1
    }

    /// The box two faces' boxes share, one of each solid's.
    fn shared(&self,faces: Pair<usize>) -> Box3 {
        let (a,b) = (self.boxes[Side::A][faces[Side::A]],self.boxes[Side::B][faces[Side::B]]);
        (std::array::from_fn(|k| a.0[k].max(b.0[k])),std::array::from_fn(|k| a.1[k].min(b.1[k])))
    }

    /// **Phase 1**: every edge against every face of the other solid. Where it crosses, a vertex
    /// on both, the edge cut there; where it lies in the face's surface, cut only where it crosses
    /// the face's own edges.
    fn crossings(&mut self) -> Crossings {
        let (tol,pad) = (self.tol,self.pad);
        let mut found = Crossings {
            on_face: Pair::new(|s| vec![Vec::new();self.solids[s].faces.len()]),
            along_face: Pair::new(|s| vec![Vec::new();self.solids[s].faces.len()]),
            loose: Vec::new(),
        };
        // the faces each edge bounds
        let bounded: Pair<Vec<Vec<usize>>> = Pair::new(|s| {
            let sol = self.solids[s];
            let mut out = vec![Vec::new();sol.edges.len()];
            for (fi,f) in sol.faces.iter().enumerate() {
                for c in f.loops.iter().flatten() { if !out[c.edge as usize].contains(&fi) { out[c.edge as usize].push(fi); } }
            }
            out
        });
        for s in Side::BOTH {
            let other = s.other();
            let theirs = self.solids[other];
            for (i,e) in self.solids[s].edges.iter().enumerate() {
                let EdgeCurve::Curve(c) = &e.curve else { continue };
                let we = self.working(s,i);
                // a sampled chord may cut inside its curve's bulge: pad by the sagitta's bound
                let sag = c.speed()*(e.t[1]-e.t[0])/32.;
                // an edge whose enclosure misses the box the other's faces fill meets none of them
                // (its samples lie within it), and is not sampled (#589); a trace's enclosure is
                // measured, not proved, so a trace is sampled whatever it says
                if !matches!(c,Curve::Traced(_)) && !overlap(&c.bounds(e.t),&self.reach[other],pad+sag) { continue }
                let mut ebox = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
                for j in 0..=32 {
                    let p = c.point(e.t[0]+(e.t[1]-e.t[0])*j as f64/32.);
                    for k in 0..3 { ebox.0[k] = ebox.0[k].min(p[k]); ebox.1[k] = ebox.1[k].max(p[k]); }
                }
                for (fi,f) in theirs.faces.iter().enumerate() {
                    if !overlap(&ebox,&self.boxes[other][fi],pad+sag) { continue }
                    let started = crate::clock::Instant::now();
                    // an edge measured off its surfaces by more than the tolerance (a fitted contact's
                    // `tol`, `Brep::measure_tolerances`) is read at its own: it lies in a face it is
                    // that near
                    let meets = curve_surface_within(c,e.t,&f.surface,tol.max(e.tol),self.boxes[other][fi],pad);
                    self.say.edge_against_face(s,i,e,fi,f,started.elapsed().as_secs_f64());
                    match meets {
                        // lying in the face's surface, it is split only where it crosses the face's
                        // own edges, which no surface tells: each crossing a vertex on both (a
                        // fillet's contact circle laid on a cylinder across the cylinder's seam)
                        Meets::Along => {
                            found.along_face[other][fi].push(we);
                            let edges: std::collections::BTreeSet<u32> = f.loops.iter().flatten().map(|g| g.edge).collect();
                            for g in edges {
                                let ge = &theirs.edges[g as usize];
                                let EdgeCurve::Curve(gc) = &ge.curve else { continue };
                                // (where neither is a line or a circle — a spline sweep's rail along a
                                // side — it is left unsplit, as every edge along a face was before; that
                                // crossing wants a general curve–curve root, not yet written)
                                for (t,q) in crossings_in(&f.surface,c,e.t,gc,ge.t,tol).unwrap_or_default() {
                                    self.say.along_crosses(s,i,c,fi,g,q);
                                    let v = self.pool.at(q);
                                    self.cuts[we].push((t,v));
                                    // on the boundary of every face that edge bounds: a meeting of the
                                    // face beside it may end there (two fillets' faces and the face both
                                    // roll on, touching one another at one point)
                                    for &h in &bounded[other][g as usize] {
                                        if !found.on_face[other][h].contains(&v) { found.on_face[other][h].push(v); }
                                    }
                                }
                            }
                        }
                        Meets::At(roots) => for (t,touch) in roots {
                            let q = c.point(t);
                            let place = self.located[other].face_place(fi,q);
                            self.say.meets(s,i,c,fi,f,q,touch,place);
                            if place == Place::Out { continue }
                            let fresh = self.pool.pts.len();
                            let v = self.pool.at(q);
                            self.cuts[we].push((t,v));
                            found.on_face[other][fi].push(v);
                            if v as usize >= fresh { found.loose.push(Loose {edge:we,vertex:v,t,side:s,index:i,face:fi}); }
                        },
                    }
                }
            }
        }
        found
    }

    /// **A root found where an edge meets a surface flatly is known only as the stretch of the edge
    /// within the tolerance of it**: a crossing of the same edge found exactly inside that stretch (a
    /// fillet's contact, where its canal lies tangent to the face the edge lies in) is that root, and
    /// the vertex the root minted is no vertex at all. Returns those vertices: each such root's
    /// crossing takes its place in `cuts` and `on_face`.
    fn flat_roots(&mut self,crossings: &mut Crossings) -> std::collections::BTreeSet<u32> {
        let tol = self.tol;
        let loose = &crossings.loose;
        let rooted: std::collections::BTreeSet<(usize,u32)> = loose.iter().map(|l| (l.edge,l.vertex)).collect();
        let mut gone = std::collections::BTreeSet::new();
        for l in loose {
            if self.cuts[l.edge].len() < 2 { continue }
            let e = &self.solids[l.side].edges[l.index];
            let EdgeCurve::Curve(c) = &e.curve else { continue };
            let s = &self.solids[l.side.other()].faces[l.face].surface;
            let near = |t: f64| s.implicit(c.point(t)).abs() <= 16.*tol.max(e.tol);
            // out from the root each way, doubling, as far as the edge stays that near
            let reach = |dir: f64| {
                let (mut d,mut last) = (1e-12*(e.t[1]-e.t[0]).abs().max(1e-300),l.t);
                loop {
                    let t = (l.t+dir*d).clamp(e.t[0],e.t[1]);
                    if !near(t) { return last }
                    last = t;
                    if t == e.t[0] || t == e.t[1] { return last }
                    d *= 2.;
                }
            };
            let (lo,hi) = (reach(-1.),reach(1.));
            let exact = self.cuts[l.edge].iter().filter(|&&(t,w)| w != l.vertex && !rooted.contains(&(l.edge,w)) && t >= lo && t <= hi)
                .min_by(|x,y| (x.0-l.t).abs().total_cmp(&(y.0-l.t).abs())).map(|x| x.1);
            let Some(w) = exact else { continue };
            self.say.flat_root(self.pool.pts[l.vertex as usize],self.pool.pts[w as usize]);
            self.cuts[l.edge].retain(|&(_,u)| u != l.vertex);
            for u in crossings.on_face[l.side.other()][l.face].iter_mut() { if *u == l.vertex { *u = w } }
            gone.insert(l.vertex);
        }
        gone
    }

    /// **Phase 2**: every pair of faces whose boxes meet, their surfaces' meeting — in closed
    /// form, or traced (`traced_meeting`) — cut at the points both faces put on it, its spans
    /// laid as working edges where they lie in both faces.
    fn meetings(&mut self,crossings: &Crossings) -> Result<Meetings,String> {
        let (a,b) = (self.solids[Side::A],self.solids[Side::B]);
        let mut found = Meetings {ssi_edges:Vec::new(),same_pairs:Vec::new()};
        for fa in 0..a.faces.len() {
            for fb in 0..b.faces.len() {
                let faces = Pair([fa,fb]);
                if !overlap(&self.boxes[Side::A][fa],&self.boxes[Side::B][fb],self.pad) { continue }
                let surfaces = Pair([&a.faces[fa].surface,&b.faces[fb].surface]);
                // two curves of one meeting in closed form may cross one another (`crossings_among`)
                let (curves,crossed) = match intersect(surfaces[Side::A],surfaces[Side::B],self.tol) {
                    Ssi::Curves(cs) => { let x = crossings_among(surfaces[Side::A],&cs,self.tol); (cs,x) }
                    Ssi::Same => { found.same_pairs.push(faces); continue }
                    Ssi::Traced => {
                        let traced = self.traced_meeting(faces,crossings)?;
                        let none = vec![Vec::new();traced.len()];
                        (traced,none)
                    }
                };
                self.say.curves(faces,surfaces,curves.len());
                let (lo,hi) = self.shared(faces);
                let (lo,hi): (V,V) = (lo.map(|x| x-self.pad),hi.map(|x| x+self.pad));
                for (c,crossed) in curves.into_iter().zip(crossed) {
                    // a curve that comes nowhere near both faces (a far crossing of two nearly parallel
                    // meridians) leaves nothing on them
                    if far(&c,lo,hi) { continue }
                    let mut ts = self.points_on(&c,surfaces,&crossings.on_face[Side::B][fb],&crossings.on_face[Side::A][fa],crossed);
                    let closed = c.period().is_some();
                    let spans = spans(&c,&mut ts,&mut self.pool);
                    self.say.points_on(&c,&ts,&self.pool);
                    for (t,v) in spans {
                        if (t[1]-t[0])*c.speed() <= self.tol || (v[0] == v[1] && !closed) { continue }
                        if !self.kept(&c,t,v,faces,surfaces) { continue }
                        let we = self.add_edge(EdgeCurve::Curve(c.clone()),t,v);
                        found.ssi_edges.push((we,faces));
                    }
                }
            }
        }
        Ok(found)
    }

    /// The vertices on curve `c`, where two faces' surfaces (`surfaces`) meet, each with its
    /// parameter along it: those on B's face and on A's (`on_b`, `on_a`: where the other solid's
    /// edges cross it) within reach of the curve, then those of `crossed`, where it crosses
    /// another curve of the same meeting, each pooled.
    fn points_on(&mut self,c: &Curve,surfaces: Pair<&Surface>,on_b: &[u32],on_a: &[u32],crossed: Vec<V>) -> Vec<(f64,u32)> {
        let (sa,sb) = (surfaces[Side::A],surfaces[Side::B]);
        let mut ts: Vec<(f64,u32)> = Vec::new();
        for &v in on_b.iter().chain(on_a) {
            let p = self.pool.pts[v as usize];
            let t = c.inverse(p);
            // a point within the tolerance of both surfaces is that over the sine of the angle
            // they meet at off their meeting
            let sine = norm(crate::space::cross(sa.gradient(p),sb.gradient(p))).max(1e-4);
            if distance(c.point(t),p) <= 8.*self.tol/sine && !ts.iter().any(|&(_,w)| w == v) { ts.push((t,v)); }
        }
        for q in crossed {
            let v = self.pool.at(q);
            if !ts.iter().any(|&(_,w)| w == v) { ts.push((c.inverse(q),v)); }
        }
        ts
    }

    /// Whether the span `t` of curve `c`, from vertex `v[0]` to `v[1]`, is kept as an edge where
    /// two faces meet: its middle in both, where the two cross rather than touch.
    fn kept(&self,c: &Curve,t: [f64;2],v: [u32;2],faces: Pair<usize>,surfaces: Pair<&Surface>) -> bool {
        let m = c.point((t[0]+t[1])/2.);
        let places = Pair::new(|s| self.located[s].face_place(faces[s],m));
        self.say.span(t,v,m,places,&self.pool);
        if places[Side::A] == Place::Out || places[Side::B] == Place::Out { return false }
        // where the two only touch (their normals parallel) nothing crosses from one
        // side to the other: no face is split there
        if norm(crate::space::cross(surfaces[Side::A].gradient(m),surfaces[Side::B].gradient(m))) < 1e-7 { return false }
        true
    }

    /// **Where two faces meet in curves no closed form gives**, traced: through the points where
    /// either face's boundary crosses the other, and where a grid across a sheet's face crosses
    /// the other surface — a curve lying inside both faces, or ending on a sheet's boundary
    /// beyond them, crosses no edge — then, between analytic surfaces, the smaller face searched
    /// for what no curve yet passes, each found traced in turn, until the search shows nothing
    /// else is there. Where the two only touch is a meeting already known (`Touches`): no trace
    /// starts or passes there.
    fn traced_meeting(&self,faces: Pair<usize>,crossings: &Crossings) -> Result<Vec<Curve>,String> {
        let (tol,pad) = (self.tol,self.pad);
        let (fa,fb) = (faces[Side::A],faces[Side::B]);
        let (a,b) = (self.solids[Side::A],self.solids[Side::B]);
        let surfaces = Pair([&a.faces[fa].surface,&b.faces[fb].surface]);
        let (sa,sb) = (surfaces[Side::A],surfaces[Side::B]);
        let on = &crossings.on_face;
        let mut seeds: Vec<V> = on[Side::B][fb].iter().chain(&on[Side::A][fa]).map(|&v| self.pool.pts[v as usize])
            .filter(|&p| sa.implicit(p).abs() <= 8.*tol && sb.implicit(p).abs() <= 8.*tol).collect();
        // the edges either lies in that the other's face holds (`Meets::Along`), over their stretches
        let along = &crossings.along_face;
        let stretches: Vec<(&Curve,[f64;2])> = along[Side::B][fb].iter().chain(&along[Side::A][fa])
            .filter_map(|&we| match &self.edges[we].curve {
                EdgeCurve::Curve(c) => Some((c,self.edges[we].t)),
                EdgeCurve::Degenerate => None,
            }).collect();
        let mut touches = Touches::new(self,faces,&stretches);
        seeds.retain(|&p| !touches.touching(p));
        let (lo,hi) = self.shared(faces);
        let started = crate::clock::Instant::now();
        // a sheet's or a face family's wall: no closed form says where a meeting may lie
        let patch = |s: &Surface| matches!(s,Surface::BSpline(..) | Surface::Family(..));
        let grid = if patch(sb) { grid_seeds(b,fb,sa,tol) }
            else if patch(sa) { grid_seeds(a,fa,sb,tol) } else { Vec::new() };
        self.say.seeded(started.elapsed().as_secs_f64());
        // (only where both faces may be: a seed beyond their shared box traces nothing on them)
        seeds.extend(grid.into_iter().filter(|p| (0..3).all(|k| p[k] >= lo[k]-pad && p[k] <= hi[k]+pad))
            .filter(|&p| !touches.touching(p)));
        self.say.tracing(faces,surfaces,&seeds);
        let started = crate::clock::Instant::now();
        let beside = &mut |p: V| touches.beside(p);
        let mut traced = super::ssi::trace_beside(sa,sb,&seeds,beside,lo,hi,tol)?;
        // a closed curve inside both faces crosses no edge and has no seed
        if super::ssi::searchable(sa) && super::ssi::searchable(sb) {
            let size = |bx: &Box3| distance(bx.0,bx.1);
            let (sol,fi,mine,theirs) = if size(&self.boxes[Side::B][fb]) < size(&self.boxes[Side::A][fa]) { (b,fb,sb,sa) } else { (a,fa,sa,sb) };
            let domain = parameters(sol,fi,true);
            // more separate loops than any face pair this kernel builds meets in
            const LOOPS: usize = 64;
            let mut found = 0;
            // the edges they touch along are meetings already known, over their own stretch
            while let Some(p) = super::ssi::unseen_beside(mine,domain,theirs,&traced,&stretches,beside,lo,hi,tol)? {
                self.say.unseen(faces,surfaces,p);
                found += 1;
                if found > LOOPS {
                    return Err(format!("a {} and a {} meet in more than {LOOPS} curves no edge crosses, not built yet",
                        sa.kind(),sb.kind()))
                }
                let more = super::ssi::trace_beside(sa,sb,&[p],beside,lo,hi,tol)?;
                if more.is_empty() {
                    return Err(format!("a {} and a {} meet at {p:?}, where no edge crosses, in a curve that could not be \
                        traced, not built yet",sa.kind(),sb.kind()))
                }
                traced.extend(more);
            }
        }
        self.say.traced(faces,surfaces,&traced,&seeds,started.elapsed().as_secs_f64());
        Ok(traced)
    }

    /// **Phase 5**: each face split by what lies on it — its boundary's pieces, the meetings laid
    /// across it and a coincident face's boundary within it — traced in its parameters, each
    /// piece with its solid and face.
    fn faces_split(&self,meetings: &Meetings,split: &Split) -> Result<Vec<(Side,usize,Vec<Vec<Half>>)>,String> {
        let mut pieces: Vec<(Side,usize,Vec<Vec<Half>>)> = Vec::new();
        for s in Side::BOTH {
            for (fi,f) in self.solids[s].faces.iter().enumerate() {
                let (halves,loop_ends) = self.boundary_halves(s,fi,split);
                // a face clear of the other is nothing but its loops, its edges' pieces in their order
                // round them (`Arranged::clear`): one piece, not traced again (#589)
                if self.clear[s][fi] {
                    let mut halves = halves.into_iter();
                    let mut start = 0;
                    let piece = loop_ends.iter().map(|&end| { let l = halves.by_ref().take(end-start).collect(); start = end; l }).collect();
                    pieces.push((s,fi,piece));
                    continue
                }
                let mut cut = FaceCut::new(f,fi,&self.located[s],&split.out,halves);
                for &(e,on) in &meetings.ssi_edges {
                    if on[s] != fi { continue }
                    for &(piece,_) in &split.pieces_of[e] { cut.internal(split.rep[piece as usize].0); }
                }
                // a face of the other on this one's surface: its boundary's pieces inside this face
                for same in &meetings.same_pairs {
                    if same[s] != fi { continue }
                    let other = s.other();
                    for l in &self.solids[other].faces[same[other]].loops { for c in l {
                        for &(piece,_) in &split.pieces_of[self.working(other,c.edge as usize)] {
                            let r = split.rep[piece as usize].0;
                            let w = &split.out[r as usize];
                            if matches!(w.curve,EdgeCurve::Degenerate) { continue }
                            if self.located[s].face_place(fi,w.point((w.t[0]+w.t[1])/2.,&self.pool)) == Place::In { cut.internal(r); }
                        }
                    } }
                }
                if s == Side::B { self.say.cut_network(fi,f,meetings,split,&self.pool); }
                for piece in pieces_of_face(f,&cut.halves,&split.out,&self.pool)? { pieces.push((s,fi,piece)); }
            }
        }
        Ok(pieces)
    }

    /// Face `fi` of solid `s`'s boundary as halves — each of its edges' pieces, the way its loop
    /// walks it, with its curve in the face's parameters (`piece_pcurve`) — and where each loop's
    /// halves end.
    fn boundary_halves(&self,s: Side,fi: usize,split: &Split) -> (Vec<Half>,Vec<usize>) {
        let sol = self.solids[s];
        let f = &sol.faces[fi];
        let mut halves: Vec<Half> = Vec::new();
        let mut loop_ends: Vec<usize> = Vec::new();
        for l in &f.loops {
            for c in l {
                let orig = &sol.edges[c.edge as usize];
                let mut ps = split.pieces_of[self.working(s,c.edge as usize)].clone();
                if c.reversed { ps.reverse(); }
                for (piece,[ta,tb]) in ps {
                    let (r,flip) = split.rep[piece as usize];
                    let at = |t: f64| c.pcurve.at(t,orig,&f.surface,&sol.vertices);
                    let (ua,ub) = (at(ta),at(tb));
                    let merged = (r != piece).then(|| (&split.out[r as usize],flip));
                    let pcurve = piece_pcurve(&c.pcurve,&f.surface,merged,ua,ub);
                    let along = !c.reversed != flip;
                    let (from,to) = if c.reversed { (ub,ua) } else { (ua,ub) };
                    halves.push(Half {edge:r,along,pcurve,from,to});
                }
            }
            loop_ends.push(halves.len());
        }
        (halves,loop_ends)
    }
}

/// Whether a curve comes nowhere near the box `[lo, hi]` (a far crossing of two nearly parallel
/// meridians): a line clipped by the box's slabs to nothing, or a closed curve far larger than
/// the box with none of its samples in it.
fn far(c: &Curve,lo: V,hi: V) -> bool {
    match *c {
        Curve::Line {p,d} => {
            let (mut t0,mut t1) = (f64::NEG_INFINITY,f64::INFINITY);
            for k in 0..3 {
                if d[k].abs() < 1e-300 { if p[k] < lo[k] || p[k] > hi[k] { t0 = 1.; t1 = 0.; } continue }
                let (a,z) = ((lo[k]-p[k])/d[k],(hi[k]-p[k])/d[k]);
                t0 = t0.max(a.min(z)); t1 = t1.min(a.max(z));
            }
            t0 > t1
        }
        _ => c.speed() > 1e3*(1.+norm(crate::space::sub(hi,lo)))
            && !(0..256).any(|j| { let q = c.point(TAU*j as f64/256.); (0..3).all(|k| q[k] >= lo[k] && q[k] <= hi[k]) }),
    }
}

/// A curve's stretches between the points on it (`ts`, each a parameter and a vertex), which are
/// sorted along it: a closed curve's from each to the next and from the last round to the first,
/// or — with none — the whole curve from a vertex pooled at its start; an open curve's between
/// each two in turn.
fn spans(c: &Curve,ts: &mut [(f64,u32)],pool: &mut Pool) -> Vec<([f64;2],[u32;2])> {
    let mut spans: Vec<([f64;2],[u32;2])> = Vec::new();
    match c.period() {
        Some(period) => {
            for t in ts.iter_mut() { t.0 = around(t.0,0.,period); }
            ts.sort_by(|x,y| x.0.total_cmp(&y.0));
            if ts.is_empty() {
                let v = pool.at(c.point(0.));
                spans.push(([0.,period],[v,v]));
            } else {
                for k in 0..ts.len() {
                    let (t0,v0) = ts[k];
                    let (mut t1,v1) = ts[(k+1)%ts.len()];
                    if k+1 == ts.len() { t1 += period; }
                    spans.push(([t0,t1],[v0,v1]));
                }
            }
        }
        None => {
            ts.sort_by(|x,y| x.0.total_cmp(&y.0));
            for w in ts.windows(2) { spans.push(([w[0].0,w[1].0],[w[0].1,w[1].1])); }
        }
    }
    spans
}

/// The curve in a face's parameters, from `ua` to `ub`, of a piece of one of its edges, whose
/// own is `own`: the same kind, where the piece stands for itself; where it is `merged` into
/// another edge (that edge, and whether it runs the other way round), the straight line that
/// edge is in the face's parameters (`iso_pcurve`), else the surface's inverse of it.
fn piece_pcurve(own: &Pcurve,surface: &Surface,merged: Option<(&WEdge,bool)>,ua: Uv,ub: Uv) -> Pcurve {
    let Some((w,flip)) = merged else {
        return match own {
            Pcurve::Line {..} => Pcurve::Line {a:ua,b:ub},
            Pcurve::Inverse {..} => Pcurve::Inverse {a:ua,b:ub},
            // keyed to the curve's own parameter, which a piece of the edge keeps
            Pcurve::Curve(c) => Pcurve::Curve(c.clone()),
        }
    };
    let iso = match &w.curve { EdgeCurve::Curve(c) => iso_pcurve(surface,c,w.t),EdgeCurve::Degenerate => None };
    if let Some((a,b)) = iso { Pcurve::Line {a,b} }
    else if flip { Pcurve::Inverse {a:ub,b:ua} } else { Pcurve::Inverse {a:ua,b:ub} }
}

/// **Where two faces touch rather than cross**, their normals parallel: no trace can follow such
/// a meeting, and none need pass it. Along an edge one of them lies in (a fillet's face against
/// the face it rolls on), that edge is their meeting there; at a point where they meet alone
/// (`ssi::touch`), that point is; outside either face, nothing built is. Each is a meeting known,
/// beside which a trace ends (`ssi::trace_beside`).
struct Touches<'s,'a> {
    arr: &'s Arrangement<'a>,
    faces: Pair<usize>,
    surfaces: Pair<&'s Surface>,
    /// each edge they touch along, over its stretch
    stretches: &'s [(&'s Curve,[f64;2])],
    /// how far off such an edge a point near both may stand: a point within `tol` of two surfaces
    /// touching may stand the square root of `tol` times their size off the curve they touch along
    spread: f64,
    /// the smaller face's size, which a touch read alone is small beside
    least: f64,
    /// the touches read, as they alone are met there or not, each within its radius
    read: Vec<(super::ssi::Touch,bool)>,
}

impl<'s,'a> Touches<'s,'a> {
    fn new(arr: &'s Arrangement<'a>,faces: Pair<usize>,stretches: &'s [(&'s Curve,[f64;2])]) -> Touches<'s,'a> {
        let surfaces = Pair::new(|s| &arr.solids[s].faces[faces[s]].surface);
        let size = |s: Side| { let bx = arr.boxes[s][faces[s]]; distance(bx.0,bx.1) };
        let spread = 8.*(arr.tol*size(Side::A).max(size(Side::B))).sqrt();
        let least = size(Side::A).min(size(Side::B));
        Touches {arr,faces,surfaces,stretches,spread,least,read:Vec::new()}
    }

    /// Whether the two meet at `p` nearly tangent: too shallowly for a trace to follow.
    fn shallow(&self,p: V) -> bool {
        norm(crate::space::cross(self.surfaces[Side::A].gradient(p),self.surfaces[Side::B].gradient(p))) < super::ssi::SHALLOW
    }

    /// Whether `p` lies on an edge they touch along, within its stretch.
    fn on_contact(&self,p: V) -> bool {
        self.stretches.iter().any(|&(c,span)| {
            let mut t = c.inverse(p);
            if let Some(period) = c.period() { t = around(t,span[0],period); }
            distance(c.point(t.clamp(span[0],span[1])),p) <= self.spread.max(8.*self.arr.tol)
        })
    }

    /// **Where the two touch along an edge one of them lies in**: its points seed nothing, and
    /// any other meeting is traced from the rest.
    fn touching(&self,p: V) -> bool { self.shallow(p) && self.on_contact(p) }

    /// **Where the two touch at a point**: to second order their meeting there is the point alone,
    /// or two branches crossing at it (`ssi::touch`); where none of the branches runs into both
    /// faces the two meet there alone, and nothing is traced through it. A touch with a branch
    /// into both is left to the trace, which refuses it. Each read is kept, as alone or not,
    /// within its radius.
    fn alone(&mut self,p: V) -> bool {
        let (tol,(sa,sb)) = (self.arr.tol,(self.surfaces[Side::A],self.surfaces[Side::B]));
        let Some(at) = super::ssi::touch_near(sa,sb,p,tol) else { return false };
        let Some(t) = super::ssi::touch(sa,sb,at) else { return false };
        if t.radius > 0.05*self.least || distance(at,p) > t.radius { return false }
        let place = |side: Side,d: V| {
            let s = self.surfaces[side];
            let q = crate::space::add(at,crate::space::scale(d,2.*t.radius));
            self.arr.located[side].face_place(self.faces[side],s.point(s.inverse(q)))
        };
        let lone = !t.branches.iter().any(|&d| place(Side::A,d) != Place::Out && place(Side::B,d) != Place::Out);
        if lone { self.arr.say.touch_alone(self.faces,self.surfaces,&t); }
        self.read.push((t,lone));
        lone
    }

    /// **Where they touch outside a face**: no part of what is built, so no trace need pass it (a
    /// fillet's canal run on past the face that cuts it off lies tangent to the face it rolls on
    /// beyond the contact's end).
    fn outside(&self,p: V) -> bool {
        Side::BOTH.into_iter().any(|side| {
            let s = self.surfaces[side];
            self.arr.located[side].face_place(self.faces[side],s.point(s.inverse(p))) == Place::Out
        })
    }

    /// Whether `p` is beside a meeting already known: about a point they touch at, read alone, or
    /// where they touch beside an edge they touch along, or outside a face — the cheapest asked
    /// first.
    fn beside(&mut self,p: V) -> bool {
        let known = self.read.iter().find(|(t,_)| distance(t.at,p) <= t.radius).map(|&(_,lone)| lone);
        known == Some(true) || self.shallow(p) && (self.on_contact(p) || self.outside(p) || known.is_none() && self.alone(p))
    }
}

/// A face being split: its halves so far — its boundary's, then each edge laid across it, both
/// ways round — and the edges they use.
struct FaceCut<'s,'a> {
    face: &'s Face,
    fi: usize,
    located: &'s Located<'a>,
    out: &'s [WEdge],
    halves: Vec<Half>,
    on: Vec<u32>,
}

impl<'s,'a> FaceCut<'s,'a> {
    fn new(face: &'s Face,fi: usize,located: &'s Located<'a>,out: &'s [WEdge],halves: Vec<Half>) -> FaceCut<'s,'a> {
        let on = halves.iter().map(|h| h.edge).collect();
        FaceCut {face,fi,located,out,halves,on}
    }

    /// An edge the other solid leaves across the face, unless it is on it already: both ways
    /// round, its parameters walked from its middle out, each step unwrapped from the last.
    fn internal(&mut self,r: u32) {
        if self.on.contains(&r) { return }
        self.on.push(r);
        let (f,out) = (self.face,self.out);
        let w = &out[r as usize];
        let EdgeCurve::Curve(c) = &w.curve else { return };
        let n = 32;
        let mid = (w.t[0]+w.t[1])/2.;
        let start = self.located.uv(self.fi,c.point(mid));
        let walk = |to: f64| {
            let mut uv = start;
            for j in 1..=n {
                let t = mid+(to-mid)*j as f64/n as f64;
                uv = unwrap(f.surface.inverse_near(c.point(t),uv),uv,f.surface.periods());
            }
            uv
        };
        let (pcurve,(ua,ub)) = match iso_pcurve(&f.surface,c,w.t) {
            Some((a,b)) => (Pcurve::Line {a,b},(a,b)),
            None => { let (a,b) = (walk(w.t[0]),walk(w.t[1])); (Pcurve::Inverse {a,b},(a,b)) }
        };
        self.halves.push(Half {edge:r,along:true,pcurve:pcurve.clone(),from:ua,to:ub});
        self.halves.push(Half {edge:r,along:false,pcurve,from:ub,to:ua});
    }
}

/// **What `SOLVENT_BREP_DEBUG` and `SOLVENT_BREP_TIME` say of an arrangement**, one method a
/// message, each silent unless asked; and the clocks the timing reads.
struct Narration {
    /// `SOLVENT_BREP_DEBUG`: every pair of faces narrated, the curves and the spans kept and dropped
    debug: bool,
    /// `SOLVENT_BREP_TIME`: each phase's time, and any step slow enough to name
    timing: bool,
    clock: crate::clock::Instant,
    /// the clock at the end of each phase timed: the edges against the faces, the pairs of faces
    laps: Vec<f64>,
    /// the time spent laying grids of seeds across sheets' faces, and tracing
    grid: Cell<f64>,
    trace: Cell<f64>,
}

/// Two faces of a pair, one of each solid, by their indices and surfaces' kinds.
fn named(faces: Pair<usize>,surfaces: Pair<&Surface>) -> String {
    format!("A{} {} × B{} {}",faces[Side::A],surfaces[Side::A].kind(),faces[Side::B],surfaces[Side::B].kind())
}

impl Narration {
    fn new() -> Narration {
        Narration {debug:super::debug(),timing:super::timing(),clock:crate::clock::Instant::now(),laps:Vec::new(),
            grid:Cell::new(0.),trace:Cell::new(0.)}
    }

    /// The end of a phase timed.
    fn lap(&mut self) { self.laps.push(self.clock.elapsed().as_secs_f64()); }

    /// Every phase done: how long each took.
    fn arranged(&self) {
        if !self.timing { return }
        let [t1,t2] = [0,1].map(|k| self.laps.get(k).copied().unwrap_or(0.));
        eprintln!("time: arrange: edges against faces {t1:.2} s; face pairs {:.2} s (grid seeds {:.2}, traces {:.2}); \
            the rest {:.2} s",t2-t1,self.grid.get(),self.trace.get(),self.clock.elapsed().as_secs_f64()-t2);
    }

    /// Edge `i` of solid `s` read against face `fi` of the other, which took `took` seconds.
    fn edge_against_face(&self,s: Side,i: usize,e: &super::topo::Edge,fi: usize,f: &Face,took: f64) {
        if !(took > 0.1 && self.timing) { return }
        let EdgeCurve::Curve(c) = &e.curve else { return };
        eprintln!("time: edge {i} of {} ({}, {:.3} long) against face {fi} ({}, feature {:.3e}): {took:.2} s",s.name(),c.kind(),
            c.speed()*(e.t[1]-e.t[0]),f.surface.kind(),f.surface.feature());
    }

    /// Edge `i` of solid `s`, lying along face `fi` of the other, crossing that face's edge `g` at `q`.
    fn along_crosses(&self,s: Side,i: usize,c: &Curve,fi: usize,g: u32,q: V) {
        if self.debug { eprintln!("brep: edge {i} of {} ({}) along face {fi} crosses its edge {g} at {q:?}",s.name(),c.kind()); }
    }

    /// Edge `i` of solid `s` meeting face `fi` of the other at `q`, which lies `place` the face.
    #[allow(clippy::too_many_arguments)]
    fn meets(&self,s: Side,i: usize,c: &Curve,fi: usize,f: &Face,q: V,touch: bool,place: Place) {
        if self.debug { eprintln!("brep: edge {i} of {} ({}) meets face {fi} ({}) at {q:?}{}: {place:?}",s.name(),c.kind(),
            f.surface.kind(),if touch { ", touching" } else { "" }); }
    }

    /// A root met flatly found to be a crossing found besides (`flat_roots`).
    fn flat_root(&self,root: V,crossing: V) {
        if self.debug { eprintln!("brep: a root at {root:?} met flatly is the crossing at {crossing:?}"); }
    }

    /// Two faces found to touch alone at a point.
    fn touch_alone(&self,faces: Pair<usize>,surfaces: Pair<&Surface>,t: &super::ssi::Touch) {
        if self.debug { eprintln!("brep: {}: touch alone at {:?} ({} branches, radius {:.1e})",named(faces,surfaces),
            t.at,t.branches.len(),t.radius); }
    }

    /// Grids of seeds laid across a sheet's face, in `took` seconds.
    fn seeded(&self,took: f64) { self.grid.set(self.grid.get()+took); }

    /// Two faces' meeting about to be traced from `seeds`.
    fn tracing(&self,faces: Pair<usize>,surfaces: Pair<&Surface>,seeds: &[V]) {
        if self.debug { eprintln!("brep: {}: tracing from {} seed(s) {seeds:?}",named(faces,surfaces),seeds.len()); }
    }

    /// The search finding a meeting of two faces no edge crosses, at `p`.
    fn unseen(&self,faces: Pair<usize>,surfaces: Pair<&Surface>,p: V) {
        if self.debug { eprintln!("brep: {}: a meeting no edge crosses, at {p:?}",named(faces,surfaces)); }
    }

    /// Two faces' meeting traced, as `traced`, from `seeds`, in `took` seconds.
    fn traced(&self,faces: Pair<usize>,surfaces: Pair<&Surface>,traced: &[Curve],seeds: &[V],took: f64) {
        self.trace.set(self.trace.get()+took);
        if took > 0.2 && self.timing {
            eprintln!("time: {}: traced {} curves from {} seeds in {took:.2} s",named(faces,surfaces),traced.len(),seeds.len());
        }
        if !self.debug { return }
        for c in traced {
            if let Curve::Traced(t) = c {
                eprintln!("brep:   traced {} points, closed {}, from {:?} to {:?}; seeds off it by {:?}",t.pts.len(),t.closed,
                    t.pts[0],t.pts[t.pts.len()-1],seeds.iter().map(|&p| distance(c.point(c.inverse(p)),p)).collect::<Vec<_>>());
            }
        }
    }

    /// Two faces' surfaces meeting in `n` curves.
    fn curves(&self,faces: Pair<usize>,surfaces: Pair<&Surface>,n: usize) {
        if self.debug && n > 0 { eprintln!("brep: {}: {n} curve(s)",named(faces,surfaces)); }
    }

    /// A curve of a meeting and the points on it (`ts`).
    fn points_on(&self,c: &Curve,ts: &[(f64,u32)],pool: &Pool) {
        if self.debug { eprintln!("brep:   {} with {} point(s) on it: {:?}",c.kind(),ts.len(),ts.iter().map(|x| pool.pts[x.1 as usize]).collect::<Vec<_>>()); }
    }

    /// A span `t` of a meeting's curve, from vertex `v[0]` to `v[1]`, its middle `m` where `places` says in each face.
    fn span(&self,t: [f64;2],v: [u32;2],m: V,places: Pair<Place>,pool: &Pool) {
        if self.debug { eprintln!("brep:     span {t:?} from {:?} to {:?}: middle {m:?} {:?}",pool.pts[v[0] as usize],pool.pts[v[1] as usize],
            (places[Side::A],places[Side::B])); }
    }

    /// The cut network on face `fi` of B: each vertex's count of cut edges (an odd one is an open end).
    fn cut_network(&self,fi: usize,f: &Face,meetings: &Meetings,split: &Split,pool: &Pool) {
        if !self.debug { return }
        let mut degree: BTreeMap<u32,usize> = BTreeMap::new();
        let cut: Vec<u32> = meetings.ssi_edges.iter().filter(|e| e.1[Side::B] == fi)
            .flat_map(|&(e,_)| split.pieces_of[e].iter().map(|&(p,_)| split.rep[p as usize].0)).collect();
        for &e in &cut { for v in split.out[e as usize].v { *degree.entry(v).or_insert(0) += 1; } }
        let open: Vec<V> = degree.iter().filter(|(_,&d)| d % 2 == 1).map(|(&v,_)| pool.pts[v as usize]).collect();
        eprintln!("arrange: face {fi} of B ({}): {} cut edges, open ends {:?}",f.surface.kind(),cut.len(),open);
    }
}

/// Step 2b's vertices: for each working edge, the pooled vertices (none of `gone`) lying inside
/// it, off its ends, and where along it; each edge boxed by its curve's enclosure, on every core.
fn vertices_inside(edges: &[WEdge],cuts: &[Vec<(f64,u32)>],pool: &Pool,gone: &std::collections::BTreeSet<u32>,
    pad: f64,tol: f64) -> Vec<Vec<(f64,u32)>> {
    // the vertices in order along x: an edge reads those within its box's stretch of x, not every
    // vertex (#589), each in the pool's order
    let mut along: Vec<u32> = (0..pool.pts.len() as u32).collect();
    along.sort_by(|&v,&w| pool.pts[v as usize][0].total_cmp(&pool.pts[w as usize][0]).then(v.cmp(&w)));
    crate::par::indices(edges.len(),|we| -> Vec<(f64,u32)> {
        let e = &edges[we];
        let EdgeCurve::Curve(c) = &e.curve else { return Vec::new() };
        let (lo,hi) = c.bounds(e.t);
        let first = along.partition_point(|&v| pool.pts[v as usize][0] < lo[0]-pad);
        let last = along.partition_point(|&v| pool.pts[v as usize][0] <= hi[0]+pad);
        let mut near = along[first..last.max(first)].to_vec();
        near.sort_unstable();
        near.into_iter().filter_map(|v| {
            if v == e.v[0] || v == e.v[1] || gone.contains(&v) || cuts[we].iter().any(|&(_,w)| w == v) { return None }
            let p = pool.pts[v as usize];
            if (0..3).any(|k| p[k] < lo[k]-pad || p[k] > hi[k]+pad) { return None }
            let mut t = c.inverse(p);
            if let Some(period) = c.period() { t = around(t,e.t[0],period); }
            // a vertex at the edge's end in all but the pool's labelling is its end
            let near = |q: V| distance(q,p) <= 8.*tol;
            (t > e.t[0] && t < e.t[1] && near(c.point(t)) && !near(c.point(e.t[0])) && !near(c.point(e.t[1]))).then_some((t,v))
        }).collect()
    })
}

/// Step 3: every working edge split where `cuts` cut it, as pieces in order along it (indices
/// into the edges returned, and each piece's parameters).
fn split_edges(edges: &[WEdge],cuts: &[Vec<(f64,u32)>]) -> (Vec<Vec<(u32,[f64;2])>>,Vec<WEdge>) {
    let mut pieces_of: Vec<Vec<(u32,[f64;2])>> = Vec::new();
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
            out.push(WEdge {curve:e.curve.clone(),t:[w[0].0,w[1].0],v:[w[0].1,w[1].1]});
            ps.push(((out.len()-1) as u32,[w[0].0,w[1].0]));
        }
        pieces_of.push(ps);
    }
    (pieces_of,out)
}

/// Step 4: each piece's representative among `out` (the first piece with the same ends and the
/// same curve between), and whether it runs the other way round.
fn merged(out: &[WEdge],pool: &Pool,tol: f64) -> Vec<(u32,bool)> {
    let mut rep: Vec<(u32,bool)> = (0..out.len() as u32).map(|i| (i,false)).collect();
    let mut by_ends: BTreeMap<(u32,u32),Vec<u32>> = BTreeMap::new();
    for (i,e) in out.iter().enumerate() { by_ends.entry((e.v[0].min(e.v[1]),e.v[0].max(e.v[1]))).or_default().push(i as u32); }
    for group in by_ends.values() {
        for (k,&i) in group.iter().enumerate() {
            let ei = &out[i as usize];
            let mid = ei.point((ei.t[0]+ei.t[1])/2.,&pool);
            for &j in &group[..k] {
                if rep[j as usize].0 != j { continue }
                let ej = &out[j as usize];
                let (EdgeCurve::Curve(ci),EdgeCurve::Curve(cj)) = (&ei.curve,&ej.curve) else { continue };
                let mut t = cj.inverse(mid);
                if let Some(period) = cj.period() { t = around(t,ej.t[0],period); }
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
    rep
}

/// Where the closed curves two surfaces meet in (`curves`, in closed form) cross one another,
/// each crossing listed against both: two curves of one meeting cross only where the surfaces
/// touch (two cylinders of one radius whose axes cross meet in two ellipses, crossing where both
/// touch), and there each is split as at an edge.
fn crossings_among(surface: &super::geom::Surface,curves: &[super::geom::Curve],tol: f64) -> Vec<Vec<V>> {
    let mut crossed = vec![Vec::new();curves.len()];
    for i in 0..curves.len() {
        for j in i+1..curves.len() {
            let (Some(pi),Some(pj)) = (curves[i].period(),curves[j].period()) else { continue };
            for (_,q) in crossings_in(surface,&curves[i],[0.,pi],&curves[j],[0.,pj],tol).unwrap_or_default() {
                crossed[i].push(q);
                crossed[j].push(q);
            }
        }
    }
    crossed
}

/// The B-rep of `faces`, their edges among `out` (vertices in `pool`), with the vertices and edges
/// they use renumbered.
fn assemble(out: &[WEdge],pool: &Pool,kept: Vec<Face>) -> Brep {
    let mut used = vec![false;out.len()];
    for f in &kept { for c in f.loops.iter().flatten() { used[c.edge as usize] = true; } }
    let mut result = Brep::default();
    let mut vmap: BTreeMap<u32,u32> = BTreeMap::new();
    let mut emap: BTreeMap<u32,u32> = BTreeMap::new();
    for (i,e) in out.iter().enumerate() {
        if !used[i] { continue }
        let mut v = [0;2];
        for k in 0..2 { v[k] = *vmap.entry(e.v[k]).or_insert_with(|| result.vertex(pool.pts[e.v[k] as usize])); }
        emap.insert(i as u32,result.edge(e.curve.clone(),e.t,v));
    }
    for mut f in kept {
        for l in &mut f.loops { for c in l { c.edge = emap[&c.edge]; } }
        result.faces.push(f);
    }
    result
}

/// The parameter of a half's edge a fraction `s` of the way along the half.
fn param(h: &Half,out: &[WEdge],s: f64) -> f64 {
    let e = &out[h.edge as usize];
    if h.along { e.t[0]+(e.t[1]-e.t[0])*s } else { e.t[1]-(e.t[1]-e.t[0])*s }
}

/// The parameters a fraction `s` of the way along a half.
fn at(h: &Half,f: &Face,out: &[WEdge],pool: &Pool,s: f64) -> Uv {
    let (e,t) = (&out[h.edge as usize],param(h,out,s));
    uv_at(h,f,e,t,|| e.point(t,pool))
}

/// The parameters a fraction `s` of the way along a half, and its point in space.
fn at_point(h: &Half,f: &Face,out: &[WEdge],pool: &Pool,s: f64) -> (Uv,V) {
    let (e,t) = (&out[h.edge as usize],param(h,out,s));
    let p = e.point(t,pool);
    (uv_at(h,f,e,t,|| p),p)
}

/// The parameters of a half at its edge's parameter `t`, whose point in space is `p()` (asked
/// only where the half's curve in the parameters is the surface's inverse of it).
fn uv_at(h: &Half,f: &Face,e: &WEdge,t: f64,p: impl FnOnce() -> V) -> Uv {
    let a = if h.along { h.from } else { h.to };
    let b = if h.along { h.to } else { h.from };
    let frac = (t-e.t[0])/(e.t[1]-e.t[0]);
    let near = [a[0]+frac*(b[0]-a[0]),a[1]+frac*(b[1]-a[1])];
    match h.pcurve {
        Pcurve::Line {..} => near,
        Pcurve::Curve(ref c) => { let p = c.point(t); [p[0],p[1]] }
        Pcurve::Inverse {..} => unwrap(f.surface.inverse_near(p(),near),near,f.surface.periods()),
    }
}

/// The face's parameters mirrored where it is reversed, so the material is always on the left.
fn oriented(f: &Face,uv: Uv) -> Uv { if f.reversed { [-uv[0],uv[1]] } else { uv } }

/// The face split into pieces by its halves: each piece its outer cycle and its holes.
fn pieces_of_face(f: &Face,halves: &[Half],out: &[WEdge],pool: &Pool) -> Result<Vec<Vec<Vec<Half>>>,String> {
    // nodes: a vertex at one place in the parameters (a seam's vertex is at two, a period apart; the
    // uses of one vertex otherwise agree only as well as their pcurves do, a fitted curve's end to
    // within its fit)
    let mut nodes: Vec<(u32,Uv)> = Vec::new();
    // each vertex's nodes, so a face of many edges finds one without reading every node (#589)
    let mut of_vertex: BTreeMap<u32,Vec<usize>> = BTreeMap::new();
    let mut node_of = |v: u32,uv: Uv| -> usize {
        let mine = of_vertex.entry(v).or_default();
        if let Some(&i) = mine.iter().find(|&&i| { let p = nodes[i].1; (p[0]-uv[0]).abs() < 1e-3 && (p[1]-uv[1]).abs() < 1e-3 }) { return i }
        nodes.push((v,uv));
        mine.push(nodes.len()-1);
        nodes.len()-1
    };
    let ends: Vec<(usize,usize)> = halves.iter().map(|h| {
        let e = &out[h.edge as usize];
        let (v0,v1) = if h.along { (e.v[0],e.v[1]) } else { (e.v[1],e.v[0]) };
        (node_of(v0,h.from),node_of(v1,h.to))
    }).collect();
    // each half's direction leaving its start and arriving at its end, as lengths, oriented
    let dir = |h: &Half,at_start: bool| -> f64 {
        let (p,q) = if at_start { (h.from,at(h,f,out,pool,1e-4)) } else { (h.to,at(h,f,out,pool,1.-1e-4)) };
        let (_,su,sv) = f.surface.d1(p);
        let (ku,kv) = (norm(su).max(1e-12),norm(sv).max(1e-12));
        let (p,q) = (oriented(f,p),oriented(f,q));
        ((q[1]-p[1])*kv).datan2((q[0]-p[0])*ku)
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
                if super::debug() {
                    for (k,x) in halves.iter().enumerate() {
                        let e = &out[x.edge as usize];
                        eprintln!("pieces: half {k} edge {} ({}, vertices {:?} at {:?}) along {} nodes {:?} from {:?} to {:?} leave {:.4} back {:.4}{}",
                            x.edge,match &e.curve { EdgeCurve::Curve(c) => c.kind(),_ => "degenerate" },e.v,e.v.map(|v| pool.pts[v as usize]),
                            x.along,ends[k],x.from,x.to,leave[k],arrive_back[k],if cycle.contains(&k) { " (this cycle)" } else { "" });
                    }
                }
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
    let mut cycles: Vec<Vec<Half>> = cycles.iter().map(|c| c.iter().map(|&h| halves[h].clone()).collect()).collect();
    let bar = reading::bar(&cycles,out,pool);
    let drawn: Vec<Outline> = cycles.iter().map(|c| reading::outline(std::slice::from_ref(c),f,out,pool,bar)).collect();
    let areas: Vec<f64> = drawn.iter().map(|o| if f.reversed { -o.area() } else { o.area() }).collect();
    let mut pieces: Vec<(usize,Vec<usize>)> = (0..cycles.len()).filter(|&i| areas[i] > 0.).map(|i| (i,vec![i])).collect();
    for i in (0..cycles.len()).filter(|&i| areas[i] <= 0.) {
        // a hole goes in the smallest outer cycle holding a point of it, read against that cycle
        // (`reading::holds`). An outer cycle sharing one of its edges is on that edge's other
        // side, and is never its host.
        let Some(first) = cycles[i].iter().find(|h| matches!(out[h.edge as usize].curve,EdgeCurve::Curve(_))) else {
            return Err(format!("a {} has a hole of no curve",f.surface.kind()))
        };
        let probe = at_point(first,f,out,pool,0.5);
        let mut host: Option<usize> = None;
        for (k,(o,_)) in pieces.iter().enumerate() {
            if cycles[*o].iter().any(|h| cycles[i].iter().any(|g| g.edge == h.edge)) { continue }
            let held = reading::holds(std::slice::from_ref(&cycles[*o]),&drawn[*o],bar,f,out,pool,probe);
            if held == Some(true) && host.is_none_or(|h| areas[*o] < areas[pieces[h].0]) { host = Some(k); }
        }
        match host {
            Some(k) => pieces[k].1.push(i),
            None => return Err(format!("a {} has a hole in no piece",f.surface.kind())),
        }
    }
    // each cycle is in one piece
    Ok(pieces.into_iter().map(|(_,cs)| cs.iter().map(|&c| std::mem::take(&mut cycles[c])).collect()).collect())
}

/// **Reading a point against cycles of halves**: the one way this module tells inside a face's
/// pieces from outside them. An outline (`query::Outline`) cuts a curved half's chords short of it
/// — a full circle in 24 chords by 0.86% of its radius, wider than a narrow strip beside it (a
/// turned rim chamfered by half a millimetre from 32 out, whose top's strip a middle of the outline
/// once put in the hole beside it) — so its winding is the cycles' only clear of every half by
/// twice its sag; nearer, the side of the nearest half's edge says, and nearer a vertex than that
/// nothing does. `Inner`, a point read inside a piece, is made nowhere else.
mod reading {
    use super::{at,at_point,param,Half,Pool,WEdge};
    use super::super::geom::{hull,Uv,V};
    use super::super::query::{beside,nearest_on,Outline};
    use super::super::topo::Face;
    use crate::space::distance;

    /// How many times an outline is drawn again, each to a quarter of the last bar, while what it
    /// reads leaves the question open.
    const FINER: usize = 5;

    /// A point of a piece's face, in space, read inside the piece (`inside`).
    #[derive(Clone,Copy,Debug)]
    pub(super) struct Inner(V);

    impl Inner {
        pub(super) fn point(self) -> V { self.0 }
    }

    /// The bar cycles of halves are first drawn to: a hundredth of how far across they stand in
    /// space, read at their ends and middles (never under the pool's tolerance).
    pub(super) fn bar(cycles: &[Vec<Half>],out: &[WEdge],pool: &Pool) -> f64 {
        let (lo,hi) = hull(cycles.iter().flatten().flat_map(|h| [0.,0.5].map(|s| out[h.edge as usize].point(param(h,out,s),pool))));
        (if lo[0] <= hi[0] { distance(lo,hi)/100. } else { 0. }).max(pool.tol)
    }

    /// `bar`, then each a quarter of the last, `FINER` times, down to the pool's tolerance.
    fn finer(bar: f64,tol: f64) -> impl Iterator<Item=f64> {
        std::iter::successors(Some(bar),move |&b| (b > tol).then(|| (b/4.).max(tol))).take(FINER+1)
    }

    /// The outline of cycles of halves on `f`, drawn to `bar` (`Outline::of`).
    pub(super) fn outline(cycles: &[Vec<Half>],f: &Face,out: &[WEdge],pool: &Pool,bar: f64) -> Outline {
        Outline::of(&f.surface,bar,cycles.iter().map(|c| c.iter().map(|h| move |s: f64| at_point(h,f,out,pool,s))))
    }

    /// Where `p`, at `uv` on `f`, stands against cycles of halves drawn as `outline`: `Some(true)`
    /// inside, by the outline's winding where the point is clear of every half by twice its sag,
    /// else by the side of the nearest half's edge (inside on the material's side of any use of
    /// it: a seam is used once each side); `None` within the pool's tolerance of a half, or where
    /// the nearest is a vertex or a pole nearer than that.
    pub(super) fn judged(cycles: &[Vec<Half>],outline: &Outline,f: &Face,out: &[WEdge],pool: &Pool,uv: Uv,p: V) -> Option<bool> {
        let mut near: Option<(f64,u32,f64)> = None;
        for h in cycles.iter().flatten() {
            let e = &out[h.edge as usize];
            let (t,d) = nearest_on(&e.curve,e.t,pool.pts[e.v[0] as usize],p);
            if near.is_none_or(|(best,_,_)| d < best) { near = Some((d,h.edge,t)); }
        }
        let (d,edge,t) = near?;
        if d <= pool.tol { return None }
        if d > 2.*outline.sag() { return Some(outline.winding(uv) != 0) }
        let e = &out[edge as usize];
        if t <= e.t[0] || t >= e.t[1] { return None }
        Some(cycles.iter().flatten().filter(|u| u.edge == edge).any(|u| {
            let s = if u.along { (t-e.t[0])/(e.t[1]-e.t[0]) } else { (e.t[1]-t)/(e.t[1]-e.t[0]) };
            let (a,b) = (at(u,f,out,pool,(s-1e-6).max(0.)),at(u,f,out,pool,(s+1e-6).min(1.)));
            beside(f,at(u,f,out,pool,s),[b[0]-a[0],b[1]-a[1]],uv)
        }))
    }

    /// Whether cycles drawn as `drawn`, to `bar`, hold `(uv, p)` (`judged`), drawn finer while
    /// they leave it unread; `None` where they still do.
    pub(super) fn holds(cycles: &[Vec<Half>],drawn: &Outline,bar: f64,f: &Face,out: &[WEdge],pool: &Pool,(uv,p): (Uv,V)) -> Option<bool> {
        finer(bar,pool.tol).enumerate().find_map(|(k,bar)| match k {
            0 => judged(cycles,drawn,f,out,pool,uv,p),
            _ => judged(cycles,&outline(cycles,f,out,pool,bar),f,out,pool,uv,p),
        })
    }

    /// Where along a level line's span across a piece its points are read (`inside`): its middle,
    /// then a point each side of it, for a piece the other solid touches along a line through
    /// every middle (a boss tangent to the face at its middle, #597).
    const ALONG: [f64;3] = [0.5,0.3,0.7];

    /// Points inside a piece, the best first: the middles of the spans level lines across its
    /// outline have inside it (`Outline::spans`), widest first, each read against the piece
    /// (`judged`) and given only where it is inside; drawn finer while none is. Then, read alike,
    /// the points off the middles (`ALONG`).
    pub(super) fn inside<'a>(piece: &'a [Vec<Half>],f: &'a Face,out: &'a [WEdge],pool: &'a Pool) -> impl Iterator<Item=Inner>+'a {
        ALONG.into_iter().flat_map(move |at| along(piece,f,out,pool,at))
    }

    /// The points `at` the way along the spans (`inside`).
    fn along<'a>(piece: &'a [Vec<Half>],f: &'a Face,out: &'a [WEdge],pool: &'a Pool,at: f64) -> impl Iterator<Item=Inner>+'a {
        let mut bars = finer(bar(piece,out,pool),pool.tol);
        let mut round: Option<(Outline,std::vec::IntoIter<Uv>)> = None;
        let mut any = false;
        std::iter::from_fn(move || loop {
            if let Some((drawn,queue)) = &mut round {
                for uv in queue.by_ref() {
                    let p = f.surface.point(uv);
                    if judged(piece,drawn,f,out,pool,uv,p) == Some(true) { any = true; return Some(Inner(p)) }
                }
                if any { return None }
            }
            let drawn = outline(piece,f,out,pool,bars.next()?);
            let queue = drawn.spans(at).into_iter();
            round = Some((drawn,queue));
        })
    }
}

/// `solid` split by `sheets` (one or more open faces reaching past it on every side) to `tol`: its
/// cells, each a connected closed shell. The faces are split where they cross; the sheets' pieces
/// inside the solid are kept, both ways round; and at every edge the pieces meeting there are taken
/// in turn about it, each two neighbours bounding one cell by the sides that face the gap between
/// them. A cell holding a solid face's outside is the outside, left out.
pub fn split(solid: &Brep,sheets: &Brep,tol: f64) -> Result<Vec<Brep>,String> {
    // a sheet has no material side, but its faces are split as any face is, the material left of its
    // outer loop in its parameters: one handed over wound the other way is turned round
    let mut sheets = sheets.clone();
    for f in &mut sheets.faces {
        let Some(outer) = f.loops.first() else { continue };
        let mut area = 0.;
        for c in outer {
            let e = &sheets.edges[c.edge as usize];
            let mut ts: Vec<f64> = (0..=16).map(|k| e.t[0]+(e.t[1]-e.t[0])*k as f64/16.).collect();
            if c.reversed { ts.reverse(); }
            for w in ts.windows(2) {
                let (p,q) = (oriented(f,c.pcurve.at(w[0],e,&f.surface,&sheets.vertices)),oriented(f,c.pcurve.at(w[1],e,&f.surface,&sheets.vertices)));
                area += p[0]*q[1]-p[1]*q[0];
            }
        }
        if area < 0. {
            for l in &mut f.loops { *l = reverse_loop(std::mem::take(l)); }
        }
    }
    let sheets = &sheets;
    let clock = crate::clock::Instant::now();
    let Arranged {solids,located,pool,out,pieces,..} = arrange(solid,None,sheets,tol)?;
    let arranged = clock.elapsed().as_secs_f64();
    let face_of = |i: usize,flip: bool| -> Face {
        let (s,fi,piece) = &pieces[i];
        piece_face(&solids[*s].faces[*fi],piece,flip)
    };
    // the pieces kept: all the solid's, and the sheets' inside it
    let mut kept: Vec<usize> = Vec::new();
    for i in 0..pieces.len() {
        let (s,fi,piece) = &pieces[i];
        if *s == Side::A { kept.push(i); continue }
        let f = &solids[Side::B].faces[*fi];
        let mut inner = reading::inside(piece,f,&out,&pool).map(|q| q.point()).peekable();
        let Some(&first) = inner.peek() else { return Err(format!("a piece of a {} has no inside",f.surface.kind())) };
        let place = inner.map(|p| located[Side::A].solid_place(p)).find(|&p| p != Place::On).unwrap_or(Place::On);
        if super::debug() {
            eprintln!("split: a piece of the sheet's {}: {place:?}, first read at {first:?} ({} loops)",f.surface.kind(),piece.len());
        }
        if place == Place::In { kept.push(i); }
    }
    if super::timing() {
        eprintln!("time: split: arranged {arranged:.2} s, the sheet's pieces placed {:.2} s",clock.elapsed().as_secs_f64()-arranged);
    }
    // a side of a piece: 2i as it is, 2i + 1 turned round; the cells are classes of sides
    let mut parent: Vec<usize> = (0..2*pieces.len()).collect();
    fn root(p: &mut [usize],mut x: usize) -> usize { while p[x] != x { p[x] = p[p[x]]; x = p[x]; } x }
    let mut at_edge: BTreeMap<u32,Vec<(usize,usize)>> = BTreeMap::new();
    for &i in &kept { for (k,h) in pieces[i].2.iter().flatten().enumerate() { at_edge.entry(h.edge).or_default().push((i,k)); } }
    if super::debug() {
        for (&e,uses) in &at_edge { if uses.len() == 1 {
            let (i,_) = uses[0];
            eprintln!("split: edge {e} meets only a piece of a {} (solid {}), at {:?}",solids[pieces[i].0].faces[pieces[i].1].surface.kind(),pieces[i].0.name(),
                out[e as usize].point((out[e as usize].t[0]+out[e as usize].t[1])/2.,&pool));
        } }
    }
    for (&e,uses) in &at_edge {
        let w = &out[e as usize];
        let EdgeCurve::Curve(c) = &w.curve else { continue };
        let tm = (w.t[0]+w.t[1])/2.;
        let t = crate::space::normalised(c.tangent(tm)).ok_or("an edge with no direction")?;
        let m = c.point(tm);
        let b1 = crate::space::normalised(crate::space::cross(t,if t[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] })).unwrap();
        let b2 = crate::space::cross(t,b1);
        // each use: the way into its piece, square to the edge, and its piece's normal as it is
        let mut around: Vec<(f64,usize,V,V)> = Vec::new();
        for &(i,k) in uses {
            let (s,fi,piece) = &pieces[i];
            let f = &solids[*s].faces[*fi];
            let h = piece.iter().flatten().nth(k).unwrap();
            let uv = at(h,f,&out,&pool,0.5);
            let mut n = crate::space::normalised(f.surface.normal_raw(uv)).ok_or_else(|| format!("a {} with no normal at {m:?}",f.surface.kind()))?;
            if f.reversed { n = crate::space::scale(n,-1.); }
            let walk = if h.along { t } else { crate::space::scale(t,-1.) };
            let d = crate::space::cross(n,walk);
            let d = crate::space::sub(d,crate::space::scale(t,crate::space::dot(d,t)));
            around.push((crate::space::dot(d,b2).datan2(crate::space::dot(d,b1)),i,d,n));
        }
        around.sort_by(|x,y| x.0.total_cmp(&y.0));
        for k in 0..around.len() {
            let (_,i,d,n) = around[k];
            let (_,j,dj,nj) = around[(k+1)%around.len()];
            // the gap from i's way in to j's, turning about the edge: i bounds it by the side facing
            // away from the turn, j by the side facing along it
            let r = crate::space::cross(t,d);
            let rj = crate::space::cross(t,dj);
            let side_i = 2*i+usize::from(crate::space::dot(n,r) > 0.);
            let side_j = 2*j+usize::from(crate::space::dot(nj,rj) < 0.);
            if super::debug() && pieces[i].0 == Side::A && pieces[j].0 == Side::A && (side_i%2) != (side_j%2) {
                eprintln!("split: at edge {e} ({} uses, at {m:?}) a {} piece's {} meets a {} piece's {}",around.len(),
                    solids[Side::A].faces[pieces[i].1].surface.kind(),["inside","outside"][side_i%2],solids[Side::A].faces[pieces[j].1].surface.kind(),["inside","outside"][side_j%2]);
            }
            let (x,y) = (root(&mut parent,side_i),root(&mut parent,side_j));
            parent[x] = y;
        }
    }
    // the classes of sides: a cell, unless it holds a solid face turned round (the outside)
    let outside: std::collections::BTreeSet<usize> = kept.iter().filter(|&&i| pieces[i].0 == Side::A).map(|&i| root(&mut parent,2*i+1)).collect();
    let mut classes: BTreeMap<usize,Vec<usize>> = BTreeMap::new();
    for &i in &kept { for side in [2*i,2*i+1] {
        if pieces[i].0 == Side::A && side == 2*i+1 { continue }
        let r = root(&mut parent,side);
        if !outside.contains(&r) { classes.entry(r).or_default().push(side); }
    } }
    if super::debug() {
        let mut all: BTreeMap<usize,(usize,usize,bool)> = BTreeMap::new();
        for &i in &kept { for side in [2*i,2*i+1] {
            let r = root(&mut parent,side);
            let e = all.entry(r).or_insert((0,0,false));
            if pieces[i].0 == Side::A { e.0 += 1 } else { e.1 += 1 }
            if pieces[i].0 == Side::A && side == 2*i+1 { e.2 = true; }
        } }
        eprintln!("split: {} solid pieces, {} sheet pieces kept; classes (solid sides, sheet sides, outside): {:?}",
            kept.iter().filter(|&&i| pieces[i].0 == Side::A).count(),kept.iter().filter(|&&i| pieces[i].0 == Side::B).count(),all.values().collect::<Vec<_>>());
    }
    let mut cells = Vec::new();
    for sides in classes.into_values() {
        let faces: Vec<Face> = sides.iter().map(|&side| face_of(side/2,side%2 == 1)).collect();
        cells.push(assemble(&out,&pool,faces));
    }
    Ok(cells)
}

/// The box of face `fi`'s parameters its loops reach, sampled along each edge. `whole`: grown
/// to hold the face though a pcurve bulges between samples — by a sixteenth of each side, or to a
/// whole period where that comes near one.
fn parameters(b: &Brep,fi: usize,whole: bool) -> [[f64;2];2] {
    let f = &b.faces[fi];
    let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
    for c in f.loops.iter().flatten() {
        let e = &b.edges[c.edge as usize];
        for j in 0..=16 {
            let uv = c.pcurve.at(e.t[0]+(e.t[1]-e.t[0])*j as f64/16.,e,&f.surface,&b.vertices);
            for k in 0..2 { lo[k] = lo[k].min(uv[k]); hi[k] = hi[k].max(uv[k]); }
        }
    }
    if whole {
        let periods = f.surface.periods();
        for k in 0..2 {
            let grow = (hi[k]-lo[k])/16.+1e-9*(1.+lo[k].abs().max(hi[k].abs()));
            match periods[k] {
                Some(p) if hi[k]-lo[k]+2.*grow >= p => hi[k] = lo[k]+p,
                _ => { lo[k] -= grow; hi[k] += grow; }
            }
        }
    }
    [[lo[0],hi[0]],[lo[1],hi[1]]]
}

/// Where a grid across face `fi` of `b`, in its parameters' box, crosses `other`: each grid line's
/// change of sign in `other`'s implicit found by bisection, then pulled onto both surfaces by the
/// trace. Seeds for curves no edge crosses.
fn grid_seeds(b: &Brep,fi: usize,other: &super::geom::Surface,tol: f64) -> Vec<V> {
    const N: usize = 24;
    let f = &b.faces[fi];
    let [[lo0,hi0],[lo1,hi1]] = parameters(b,fi,false);
    let (lo,hi) = ([lo0,lo1],[hi0,hi1]);
    let at = |i: usize,j: usize| [lo[0]+(hi[0]-lo[0])*i as f64/N as f64,lo[1]+(hi[1]-lo[1])*j as f64/N as f64];
    let value = |uv: Uv| other.implicit(f.surface.point(uv));
    let grid: Vec<Vec<f64>> = (0..=N).map(|i| (0..=N).map(|j| value(at(i,j))).collect()).collect();
    let mut seeds = Vec::new();
    let mut root = |a: Uv,b: Uv,fa: f64| {
        let (mut a,mut b,mut fa) = (a,b,fa);
        for _ in 0..40 {
            let m = [(a[0]+b[0])/2.,(a[1]+b[1])/2.];
            let fm = value(m);
            if fm.abs() <= tol { a = m; break }
            if (fm < 0.) == (fa < 0.) { a = m; fa = fm; } else { b = m; }
        }
        seeds.push(f.surface.point(a));
    };
    for i in 0..=N { for j in 0..=N {
        if i < N && (grid[i][j] < 0.) != (grid[i+1][j] < 0.) { root(at(i,j),at(i+1,j),grid[i][j]); }
        if j < N && (grid[i][j] < 0.) != (grid[i][j+1] < 0.) { root(at(i,j),at(i,j+1),grid[i][j]); }
    } }
    seeds
}
