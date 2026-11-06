//! Booleans of two solids (docs/rust-kernel-plan.md, rung 1): every edge of each split where it
//! crosses the other's faces, every pair of faces intersected and the curves kept where they lie in
//! both, every face split by what now lies on it — traced in its own parameters, the material on
//! the left — each piece placed against the other solid, and the pieces the operation keeps
//! assembled, their edges shared by construction.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::geom::{around,Box3,Curve,Surface,Uv,V};
use super::query::{crossings_in,curve_surface_within,Located,Meets,Outline,Place};
use super::ssi::Ssi;
use super::complex::{Complex,Origin};
use super::topo::{reverse_loop,unwrap,Brep,Coedge,EdgeCurve,Face,Pcurve};
use crate::util::space::{distance,norm};
use std::cell::Cell;
use std::collections::BTreeMap;

const TAU: f64 = std::f64::consts::TAU;

/// What a Boolean keeps.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Op { Union,Cut,Common }

/// One of an arrangement's operands, by its place among them: `A`, the first (the solid a run of
/// Booleans hands on), `B` the second, and so on (#735 A4: any number of them at once).
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
struct Side(usize);

impl Side {
    const A: Side = Side(0);
    const B: Side = Side(1);
    /// The other of a Boolean's two solids.
    fn other(self) -> Side { Side(1-self.0) }
    /// `A`, `B`, … as narrated.
    fn name(self) -> String { if self.0 < 26 { ((b'A'+self.0 as u8) as char).to_string() } else { format!("#{}",self.0) } }
}

/// A value for each operand, read by its `Side`.
#[derive(Clone,Debug)]
struct Sides<T>(Vec<T>);

impl<T> Sides<T> {
    /// Each operand's value, the first's made first.
    fn new(n: usize,mut f: impl FnMut(Side) -> T) -> Sides<T> { Sides((0..n).map(|i| f(Side(i))).collect()) }
    /// Every operand, in order.
    fn all(&self) -> impl Iterator<Item=Side> { (0..self.0.len()).map(Side) }
}

impl<T> std::ops::Index<Side> for Sides<T> {
    type Output = T;
    fn index(&self,s: Side) -> &T { &self.0[s.0] }
}

impl<T> std::ops::IndexMut<Side> for Sides<T> {
    fn index_mut(&mut self,s: Side) -> &mut T { &mut self.0[s.0] }
}

/// Two faces of two operands met: each's operand and face, the lower operand first.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
struct Two { s: [Side;2],f: [usize;2] }

impl Two {
    /// Where operand `s`'s face `fi` is one of the two: 0 or 1.
    fn holds(&self,s: Side,fi: usize) -> Option<usize> { (0..2).find(|&k| self.s[k] == s && self.f[k] == fi) }
}

use super::tolerant::{on_meeting,Pool,FLAT,ON_CURVE};

/// A root an edge meets a face of the other solid at, by its distance from the face's surface:
/// the working edge, the vertex the root minted, its parameter, the edge's solid and index there,
/// and the operand and face met.
struct Loose { edge: usize,vertex: u32,t: f64,side: Side,index: usize,against: Side,face: usize }

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
pub fn overlap(a: &([f64;3],[f64;3]),b: &([f64;3],[f64;3]),pad: f64) -> bool {
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
    let lift = crate::util::space::dot(crate::util::space::sub(e.poles[0],c.poles[0]),f.z);
    let scale = 1.+c.poles.iter().map(|&p| norm(p)).fold(0.,f64::max);
    c.poles.iter().zip(&e.poles).all(|(&p,&q)| norm(crate::util::space::sub(q,crate::util::space::add(p,crate::util::space::scale(f.z,lift)))) <= 1e-12*scale)
        .then(|| ([t[0],lift],[t[1],lift]))
}

/// `a` combined with `b` by `op`, to `tol` (a length).
/// Two B-reps' faces split by everything the other lays on them, nothing yet kept or dropped: the
/// working edges and their pooled vertices, and each face's pieces (which solid, which face, its loops).
struct Arranged<'a> {
    solids: Sides<&'a Brep>,
    located: Sides<Located<'a>>,
    pool: Pool,
    out: Vec<WEdge>,
    /// Each of `out`'s edges laid by a meeting (`heal`'s to take out where one face is kept on
    /// both sides of it), not one of an operand's own edges: no piece of an operand's edge merged
    /// into it.
    met: Vec<bool>,
    pieces: Vec<(Side,usize,Vec<Vec<Half>>)>,
    /// Each operand's faces clear of the others: whose boxes miss the box each other's faces fill,
    /// so nothing of the others meets them and all of each lies outside them (#589).
    clear: Sides<Vec<bool>>,
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
        if near(a0,end(a,ra[ra.len()-1])) && heading(a,ra[0]).zip(heading(b,rb[0])).is_some_and(|(p,q)| crate::util::space::dot(p,q) < 0.) {
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
            if crate::util::space::dot(na,outward(fb,p)) > if plane(fa) { -1.+1e-9 } else { 0. } { continue }
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
    let mut same = crate::util::union_find::UnionFind::new(out.vertices.len());
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

/// **Faces on one plane meeting edge to edge made one** (a `glued` node's feet, `solid::grown`):
/// where two faces of `b` lie on one plane with their material the same side and share an edge,
/// the edge goes and their loops are walked as one face's — the first's surface, each other's
/// uses read onto it, the outer loop first.  Edges and vertices left bounding nothing stay for
/// `Brep::compacted`.  A vertex the joined loops leave by more than one use is refused: no walk
/// round the face is one there.
pub fn planes_joined(b: &Brep,tol: f64) -> Result<Brep,String> {
    let n = b.faces.len();
    let outward = |f: &Face| -> Option<V> {
        let Surface::Plane(fr) = f.surface else { return None };
        Some(if f.reversed { fr.z.map(|x| -x) } else { fr.z })
    };
    let mut users: Vec<Vec<usize>> = vec![Vec::new();b.edges.len()];
    for (i,f) in b.faces.iter().enumerate() {
        for c in f.loops.iter().flatten() { users[c.edge as usize].push(i); }
    }
    // the faces joined, by the first of each
    fn root(of: &mut [usize],i: usize) -> usize { let mut r = i; while of[r] != r { r = of[r]; } of[i] = r; r }
    let mut of: Vec<usize> = (0..n).collect();
    let joined = |i: usize,j: usize| match (outward(&b.faces[i]),outward(&b.faces[j])) {
        (Some(p),Some(q)) => crate::util::space::dot(p,q) > 1.-1e-9 && super::ssi::same(&b.faces[i].surface,&b.faces[j].surface,tol),
        _ => false,
    };
    let mut inside = vec![false;b.edges.len()];
    for (e,u) in users.iter().enumerate() {
        let &[i,j] = &u[..] else { continue };
        if i != j && joined(i,j) {
            let (ri,rj) = (root(&mut of,i),root(&mut of,j));
            of[ri.max(rj)] = ri.min(rj);
            inside[e] = true;
        }
    }
    if !inside.iter().any(|&x| x) { return Ok(b.clone()) }
    let mut groups: BTreeMap<usize,Vec<usize>> = BTreeMap::new();
    for i in 0..n { let r = root(&mut of,i); groups.entry(r).or_default().push(i); }
    let mut out = b.clone();
    let mut gone = vec![false;n];
    for (&host,group) in groups.iter().filter(|(_,g)| g.len() > 1) {
        let surface = b.faces[host].surface.clone();
        let mut uses: Vec<Coedge> = Vec::new();
        for &fi in group {
            for c in b.faces[fi].loops.iter().flatten().filter(|c| !inside[c.edge as usize]) {
                let mut c = c.clone();
                if fi != host {
                    // read onto the first face's plane: its parameters are another frame's
                    let e = &b.edges[c.edge as usize];
                    let at = |t: f64| surface.inverse(e.point(t,&b.vertices));
                    c.pcurve = Pcurve::Inverse {a:at(e.t[0]),b:at(e.t[1])};
                }
                uses.push(c);
            }
            if fi != host { gone[fi] = true; }
        }
        // each use followed by the one leaving the vertex it reaches
        let mut leaving: BTreeMap<u32,usize> = BTreeMap::new();
        for (k,c) in uses.iter().enumerate() {
            if leaving.insert(b.ends(c)[0],k).is_some() {
                return Err(format!("faces joined on one plane ({}) pass a vertex twice",b.faces[host].name))
            }
        }
        let mut walked = vec![false;uses.len()];
        let mut loops = Vec::new();
        for k0 in 0..uses.len() {
            let mut l = Vec::new();
            let mut k = k0;
            while !walked[k] {
                walked[k] = true;
                l.push(uses[k].clone());
                k = *leaving.get(&b.ends(&uses[k])[1]).ok_or("faces joined on one plane leave a loop open")?;
            }
            if k != k0 && !l.is_empty() { return Err("faces joined on one plane leave a loop open".into()) }
            if !l.is_empty() { loops.push(l); }
        }
        let face = Face {loops:Vec::new(),..b.faces[host].clone()};
        loops.sort_by(|p,q| out.loop_area(&face,q).abs().total_cmp(&out.loop_area(&face,p).abs()));
        out.faces[host].loops = loops;
    }
    out.faces = out.faces.into_iter().enumerate().filter(|(i,_)| !gone[*i]).map(|(_,f)| f).collect();
    Ok(out)
}

/// `s`'s edges cut at the points listed against each (on its curve, within its stretch; those
/// within `tol` of one another one): each edge's pieces in order along it, the first keeping its
/// index, and each use of it a use of each piece, its pcurve cut where the edge is (`cut_with`).
/// `None` where an edge so listed is a pole.
fn cut_at(s: &mut Brep,cuts: &BTreeMap<u32,Vec<V>>,tol: f64) -> Option<BTreeMap<u32,Vec<u32>>> {
    let mut at = BTreeMap::new();
    for (&e,ps) in cuts {
        let edge = &s.edges[e as usize];
        let EdgeCurve::Curve(c) = &edge.curve else { return None };
        let mut ts: Vec<f64> = ps.iter().map(|&p| { let t = c.inverse(p); c.period().map_or(t,|period| around(t,edge.t[0],period)) })
            .filter(|&t| t > edge.t[0] && t < edge.t[1]).collect();
        ts.sort_by(f64::total_cmp);
        ts.dedup_by(|x,y| distance(c.point(*x),c.point(*y)) <= tol);
        if ts.is_empty() { continue }
        let points: Vec<(f64,V)> = ts.iter().map(|&t| (t,c.point(t))).collect();
        at.insert(e,points.into_iter().map(|(t,p)| (t,s.vertex(p))).collect());
    }
    Some(cut_with(s,&at))
}

/// `s`'s edges cut at the parameters listed against each, increasing and inside its stretch, each
/// at the vertex given with it: each edge's pieces in order along it, the first keeping its index,
/// the rest added, and each use of it a use of each piece in the use's direction, its pcurve cut
/// where the edge is.
pub(crate) fn cut_with(s: &mut Brep,cuts: &BTreeMap<u32,Vec<(f64,u32)>>) -> BTreeMap<u32,Vec<u32>> {
    let mut pieces = BTreeMap::new();
    for (&e,at) in cuts {
        let edge = s.edges[e as usize].clone();
        let t: Vec<f64> = std::iter::once(edge.t[0]).chain(at.iter().map(|c| c.0)).chain([edge.t[1]]).collect();
        let v: Vec<u32> = std::iter::once(edge.v[0]).chain(at.iter().map(|c| c.1)).chain([edge.v[1]]).collect();
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
    pieces
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
    // touches laid as edges, so the complex is split along them as any arrangement is, and every
    // meeting's edge taken back out of the result where a face is kept on both sides of it (`heal`)
    let Arranged {solids,located,pool,out,met,pieces,clear} = arrange(&[a,b],known,&[],tol)?;
    // the faces of the complex the two part space into (#735 A4): every piece but those with no
    // inside and those a coincident piece of A stands for, each with what it is known by — its box
    // and whether it was checked, for a face carried whole — and whether each of its sides, behind
    // it and in front of it, lies in A and in B
    let mut faces: Vec<Face> = Vec::new();
    let mut origins: Vec<Vec<Origin>> = Vec::new();
    let mut member: Vec<[[bool;2];2]> = Vec::new();
    let mut meta: Vec<(Option<Box3>,bool)> = Vec::new();
    for (s,fi,piece) in pieces {
        let f = &solids[s].faces[fi];
        let other = s.other();
        // the piece's own solid lies behind it; the other on both sides (`[true; 2]`), neither, or
        // — a piece on a face of the other — behind it where the two face the same way
        let mut add = |face: Face,theirs: [bool;2],box_: Option<Box3>,checked: bool,partner: Option<(usize,bool)>| {
            let mut m = [[false;2];2];
            for k in 0..2 { m[k][s.0] = k == 0; m[k][other.0] = theirs[k]; }
            let mut from = vec![Origin {operand:s.0,face:fi,alike:true}];
            if let Some((face,alike)) = partner { from.push(Origin {operand:other.0,face,alike}); }
            faces.push(face);
            origins.push(from);
            member.push(m);
            meta.push((box_,checked));
        };
        // a face clear of the other solid is one piece, whole and outside it: no point of it need
        // be placed (a large solid's faces far from a small one, the most of them, #589)
        if clear[s][fi] {
            add(piece_face(f,&piece,false),[false;2],Some(located[s].face_box(fi)),
                s == Side::A && known.is_some_and(|k| k.checked.get(fi) == Some(&true)),None);
            continue
        }
        // place a point inside the piece against the other solid: the first of its inner
        // points not on the other's boundary (a piece may touch it tangentially), or one on
        // it where the other has a face on this surface
        let mut inner = reading::inside(&piece,f,&out,&pool).map(|q| q.point()).peekable();
        // a piece with no inside bounds nothing: the sliver a surface touching the other solid
        // along an edge of it cuts off there (#721 X6), dropped — merged into its edge (#735 A3);
        // whether what is left closes is the result's check
        let Some(&first) = inner.peek() else { continue };
        // a face of the other on this one's surface holding `p`
        let partner = |p: V| (0..solids[other].faces.len()).find(|&g|
            super::ssi::same(&f.surface,&solids[other].faces[g].surface,tol) && located[other].face_holds(g,p));
        let (p,place) = inner.map(|p| (p,located[other].solid_place(p)))
            .find(|&(p,place)| place != Place::On || partner(p).is_some())
            .unwrap_or((first,Place::On));
        match place {
            Place::In => add(piece_face(f,&piece,false),[true;2],None,false,None),
            Place::Out => add(piece_face(f,&piece,false),[false;2],None,false,None),
            Place::On => {
                // on a face of the other on the same surface: one face of the complex, A's piece
                // standing for both, the other's material behind it where the two face the same way
                let Some(g) = partner(p) else {
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
                if s != Side::A { continue }
                let alike = facing_alike(f,&solids[other].faces[g],p);
                add(piece_face(f,&piece,false),[alike,!alike],None,false,Some((g,alike)));
            }
        }
    }
    let value = |m: [bool;2]| match op { Op::Union => m[0] || m[1],Op::Cut => m[0] && !m[1],Op::Common => m[0] && m[1] };
    // **the operation a selection of the complex's cells** (#735 A4): each shell kept as the
    // operation keeps its sides, which agree round it; its boundary the result, touching itself
    // where kept cells meet along an edge across cells not kept, each wedge there its own edge
    let n = faces.len();
    let origin: Vec<(usize,usize)> = origins.iter().map(|o| (o[0].operand,o[0].face)).collect();
    let c = Complex::new(working_brep(&out,&pool,faces),origins,tol)?;
    let mut keep: Vec<Option<bool>> = vec![None;c.shells.len()];
    for f in 0..n {
        for k in 0..2 {
            let v = value(member[f][k]);
            match keep[c.shell_sides[f][k]] {
                None => keep[c.shell_sides[f][k]] = Some(v),
                // a shell is one cell's side: kept and dropped at once, the complex has joined two
                // cells the operation tells apart, round an edge whose faces it ordered wrongly
                Some(w) if w != v => {
                    let f = &c.b.faces[f];
                    return Err(format!("a shell of the arrangement is kept on one side of a {}{} and dropped on another: \
                        two of its cells joined round an edge",f.surface.kind(),if f.name.is_empty() { String::new() } else { format!(" (`{}`)",f.name) }))
                }
                _ => {}
            }
        }
    }
    let parts = c.select_parts(&|k| keep[k] == Some(true));
    let mut handed = Known::default();
    let mut kept: Vec<Face> = Vec::new();
    let mut kept_origin: Vec<(usize,usize)> = Vec::new();
    // each edge a meeting laid, as the edge it is: itself, or for a twin the edge it is a twin of
    let mut laid: Vec<Option<u32>> = (0..out.len()).map(|e| met[e].then_some(e as u32)).collect();
    let twins: Vec<Option<u32>> = parts.twin_of.iter().map(|&e| laid[e]).collect();
    laid.extend(twins);
    let mut edges = out;
    edges.extend(parts.twins.into_iter().map(|e| WEdge {curve:e.curve,t:e.t,v:e.v}));
    // a face given an edge of its own where the result touches itself is not the face it was: its
    // box is read again
    for (f,face,touched) in parts.faces {
        handed.boxes.push(if touched { None } else { meta[f].0 });
        handed.checked.push(meta[f].1);
        kept.push(face);
        kept_origin.push(origin[f]);
    }
    Ok((healed(&edges,&pool,kept,&kept_origin,&laid,&mut handed),handed))
}

/// `faces` as one boundary over `edges` (`assemble`), the meetings that part nothing healed out
/// first (`heal`, each face's operand face `origin`, each edge's meeting `laid`), a loop a healed
/// meeting parted in two given the one enclosing the other as its outer, and a loop through a pole
/// closed along it (`Brep::close_poles`).
fn healed(edges: &[WEdge],pool: &Pool,mut faces: Vec<Face>,origin: &[(usize,usize)],laid: &[Option<u32>],handed: &mut Known) -> Brep {
    let parted = heal(&mut faces,handed,origin,laid);
    let mut result = assemble(edges,pool,faces);
    outer_first(&mut result,&parted);
    // a loop through a pole of its surface, a meeting arriving there, closed along it (#797)
    result.close_poles();
    result
}

/// **A selection's meetings taken back out of its result where they part nothing** (#735): an edge
/// a meeting laid (`laid`, each such edge as itself, a twin as the edge it twins) is no edge of the
/// result where one face of an operand (`origin`, each kept face's) is kept on both sides of it —
/// a touch along the face, or another operand's surface crossing it where none of that operand is
/// kept — the face's pieces either side joined again across it, and a slit it leaves inside the
/// face (both its sides one face's) closed: the result is the manifold B-rep it would be had the
/// meeting never been laid, its faces whole and touching where they touch. A meeting bounding a
/// face kept on one side only is an edge of the result, as any edge between two of its faces is;
/// an operand's own edge (a seam) is never taken out. The faces gone (joined into another) are
/// dropped with what is known of them; the faces whose loops were parted are returned (by their
/// places after the drop), each loop's outer one for the caller to read again.
fn heal(faces: &mut Vec<Face>,handed: &mut Known,origin: &[(usize,usize)],laid: &[Option<u32>]) -> Vec<usize> {
    let laid_by = |e: u32| laid.get(e as usize).copied().flatten();
    let mut gone = vec![false;faces.len()];
    let mut parted = vec![false;faces.len()];
    loop {
        // the uses of each laid edge by each operand's face: (face, loop, place)
        let mut uses: BTreeMap<(u32,(usize,usize)),Vec<(usize,usize,usize)>> = BTreeMap::new();
        for (f,face) in faces.iter().enumerate() {
            if gone[f] { continue }
            for (l,lp) in face.loops.iter().enumerate() {
                for (k,c) in lp.iter().enumerate() {
                    if let Some(t) = laid_by(c.edge) { uses.entry((t,origin[f])).or_default().push((f,l,k)); }
                }
            }
        }
        let Some(&[(fa,la,ia),(fb,lb,ib)]) = uses.values().find(|u| u.len() == 2 && faces[u[0].0].reversed == faces[u[1].0].reversed)
            .map(|u| &u[..]) else { break };
        // the two loops spliced where the edge was: one runs on into the other past it
        let la_loop = faces[fa].loops[la].clone();
        if (fa,la) == (fb,lb) {
            // one loop through both sides of it: parted into the stretch between and the rest
            let (i,j) = (ia.min(ib),ia.max(ib));
            let inner: Vec<Coedge> = la_loop[i+1..j].to_vec();
            let rest: Vec<Coedge> = la_loop[j+1..].iter().chain(&la_loop[..i]).cloned().collect();
            let face = &mut faces[fa];
            face.loops.remove(la);
            for l in [rest,inner] { if !l.is_empty() { face.loops.push(l) } }
            parted[fa] = true;
        } else {
            let lb_loop = faces[fb].loops[lb].clone();
            let joined: Vec<Coedge> = la_loop[..ia].iter().chain(&lb_loop[ib+1..]).chain(&lb_loop[..ib]).chain(&la_loop[ia+1..]).cloned().collect();
            if fa == fb {
                let face = &mut faces[fa];
                face.loops[la] = joined;
                face.loops.remove(lb);
            } else {
                let mut others: Vec<Vec<Coedge>> = std::mem::take(&mut faces[fb].loops);
                others.remove(lb);
                faces[fa].loops[la] = joined;
                faces[fa].loops.extend(others);
                gone[fb] = true;
                parted[fa] |= parted[fb];
            }
        }
        if let Some(b) = handed.boxes.get_mut(fa) { *b = None }
        if let Some(c) = handed.checked.get_mut(fa) { *c = false }
    }
    // a laid edge left between two faces of different operands: one edge, its twin given up
    let mut left: BTreeMap<u32,Vec<(usize,usize,usize)>> = BTreeMap::new();
    for (f,face) in faces.iter().enumerate() {
        if gone[f] { continue }
        for (l,lp) in face.loops.iter().enumerate() {
            for (k,c) in lp.iter().enumerate() { if let Some(t) = laid_by(c.edge) { left.entry(t).or_default().push((f,l,k)); } }
        }
    }
    for u in left.values() {
        if let [(fa,la,ia),(fb,lb,ib)] = u[..] {
            let e = faces[fa].loops[la][ia].edge.min(faces[fb].loops[lb][ib].edge);
            faces[fa].loops[la][ia].edge = e;
            faces[fb].loops[lb][ib].edge = e;
        }
    }
    let mut out = Vec::new();
    let mut k = 0;
    for f in 0..faces.len() {
        if gone[f] { continue }
        if parted[f] { out.push(k) }
        k += 1;
    }
    fn kept<T>(v: &mut Vec<T>,gone: &[bool]) { let mut i = 0; v.retain(|_| { i += 1; !gone[i-1] }); }
    kept(faces,&gone);
    if handed.boxes.len() == gone.len() { kept(&mut handed.boxes,&gone) }
    if handed.checked.len() == gone.len() { kept(&mut handed.checked,&gone) }
    out
}

/// `faces` as one boundary, their edges `out` and their vertices `pool`'s, numbered as there: what
/// a complex of the arrangement is built on.
fn working_brep(out: &[WEdge],pool: &Pool,faces: Vec<Face>) -> Brep {
    let edges = out.iter().map(|w| super::topo::Edge {curve:w.curve.clone(),t:w.t,v:w.v,tol:built_tol(w,pool)}).collect();
    Brep {vertices:pool.vertices(),edges,faces}
}

/// **The tolerance a working edge was built to** (#735 A3): how far its curve's ends stand from the
/// vertices a merge onto a meeting put there, where that is past the pool's own reach (0 within it).
fn built_tol(w: &WEdge,pool: &Pool) -> f64 {
    let EdgeCurve::Curve(c) = &w.curve else { return 0. };
    let gap = (0..2).map(|k| distance(c.point(w.t[k]),pool.pts[w.v[k] as usize])).fold(0.,f64::max);
    if gap > pool.tol { gap } else { 0. }
}

/// Whether two faces on one surface at `p` face the same way (their material normals agree).
fn facing_alike(f: &Face,g: &Face,p: V) -> bool {
    let normal = |face: &Face| {
        let n = face.surface.normal_raw(face.surface.inverse(p));
        if face.reversed { crate::util::space::scale(n,-1.) } else { n }
    };
    crate::util::space::dot(normal(f),normal(g)) > 0.
}


/// **The cell complex any number of solids and sheets part space into** (#735 A4): every face of
/// every operand split by all the others at once, through one set of pooled vertices and merged
/// edges (A3's one place), each piece a face of the complex — but a piece with no inside, the
/// sliver a surface touching another only along an edge of it cuts off, merged into that edge; and
/// where pieces of several operands lie on one surface over one region, the lowest operand's piece
/// standing for all, each named among its origins with whether it faces the same way. Nothing is
/// kept or dropped by any operation: that is a selection over it (`selected`, `Complex::select`).
pub fn arrangement(operands: &[&Brep],tol: f64) -> Result<Complex,String> { arranged(operands,&[],false,tol) }

/// **The cell complex solids part a domain into** (`arrangement`, its first operand the domain):
/// every piece of the others lying outside the domain left out, so the complex's bounded cells
/// are the domain's, each bounded by the domain's faces and the others' pieces within it (#721's
/// X6: a box holding a sweep, split by the sweep's candidate surfaces at once).
pub fn arrangement_within(domain: &Brep,operands: &[&Brep],tol: f64) -> Result<Complex,String> {
    arrangement_given(domain,operands,&[],tol)
}

/// **A meeting known before any is sought** (#810, #780 R5: a touch given by construction): curves
/// lying on both `surfaces`, surfaces of two of an arrangement's operands' faces — whole curves, as
/// a closed form's are (a line unbounded, a closed curve whole) — that are the two surfaces' whole
/// meeting, laid as an edge of both with no search for it. What a sweep knows of its candidates'
/// touches (a contact's path where it crosses a smooth seam into a neighbour's contact, along which
/// the two contacts' swept sheets touch) is handed in so, not sought where a search would read a
/// touch as a crossing or miss it.
#[derive(Clone,Debug)]
pub struct Given { pub surfaces: [Surface;2],pub curves: Vec<Curve> }

/// **`arrangement_within`, with meetings given** (#810): the domain split by the operands at once,
/// each pair of surfaces a `Given` names met in the curves it gives, with no search.
pub fn arrangement_given(domain: &Brep,operands: &[&Brep],given: &[Given],tol: f64) -> Result<Complex,String> {
    let all: Vec<&Brep> = std::iter::once(domain).chain(operands.iter().copied()).collect();
    arranged(&all,given,true,tol)
}

/// `arrangement`, the first operand's outside left out where `within`, the meetings `given` known.
fn arranged(operands: &[&Brep],given: &[Given],within: bool,tol: f64) -> Result<Complex,String> {
    let Arranged {solids,located,pool,out,met,pieces,..} = arrange(operands,None,given,tol)?;
    let mut faces: Vec<Face> = Vec::new();
    let mut origins: Vec<Vec<Origin>> = Vec::new();
    for (s,fi,piece) in pieces {
        let f = &solids[s].faces[fi];
        let Some(p) = reading::inside(&piece,f,&out,&pool).map(|q| q.point()).next() else {
            if super::debug() { eprintln!("arrangement: a piece of {}{fi} ({}) has no inside: merged into its edges",s.name(),f.surface.kind()); }
            continue
        };
        if within && s != Side::A && located[Side::A].solid_place(p) == Place::Out {
            if super::debug() { eprintln!("arrangement: a piece of {}{fi} ({}) at {p:?} outside the domain",s.name(),f.surface.kind()); }
            continue
        }
        let mut from = vec![Origin {operand:s.0,face:fi,alike:true}];
        let mut stood_for = false;
        for o in solids.all().filter(|&o| o != s) {
            for (g,h) in solids[o].faces.iter().enumerate() {
                if !super::ssi::same(&f.surface,&h.surface,tol) || !located[o].face_holds(g,p) { continue }
                if o < s { stood_for = true; }
                from.push(Origin {operand:o.0,face:g,alike:facing_alike(f,h,p)});
            }
        }
        if super::debug() { eprintln!("arrangement: a piece of {}{fi} ({}) at {p:?}: {from:?}{}",s.name(),f.surface.kind(),
            if stood_for { ", stood for by a lower operand's" } else { "" }); }
        if stood_for { continue }
        faces.push(piece_face(f,&piece,false));
        origins.push(from);
    }
    // each edge a meeting laid, as the complex's edges are kept: those the faces use, in order
    let mut used = vec![false;out.len()];
    for c in faces.iter().flat_map(|f| f.loops.iter().flatten()) { used[c.edge as usize] = true; }
    let met: Vec<bool> = met.into_iter().zip(used).filter(|&(_,u)| u).map(|(m,_)| m).collect();
    let mut complex = Complex::new(working_brep(&out,&pool,faces).compacted(),origins,tol)?;
    complex.met = met;
    Ok(complex)
}

/// **A selection's boundary, whole** (#735): the faces between the cells `keep` keeps and the rest
/// (`Complex::select_cells`), the meetings among them that part nothing taken back out (`heal`:
/// where one face of an operand is kept on both sides of one), so it is the manifold B-rep the
/// selection is wherever it is one — each face as whole as its operand's, crossed by no candidate
/// that bounds nothing there. What a swept solid built or cut on one arrangement returns.
pub fn selection(complex: &Complex,keep: &dyn Fn(usize) -> bool) -> Result<Brep,String> {
    let parts = complex.select_cells(keep)?;
    let mut laid: Vec<Option<u32>> = (0..complex.b.edges.len()).map(|e| complex.met.get(e).copied().unwrap_or(false).then_some(e as u32)).collect();
    let twins: Vec<Option<u32>> = parts.twin_of.iter().map(|&e| laid[e]).collect();
    laid.extend(twins);
    let origin: Vec<(usize,usize)> = parts.faces.iter().map(|(f,_,_)| (complex.origins[*f][0].operand,complex.origins[*f][0].face)).collect();
    let mut faces: Vec<Face> = parts.faces.into_iter().map(|(_,face,_)| face).collect();
    let parted = heal(&mut faces,&mut Known::default(),&origin,&laid);
    let edges = complex.b.edges.iter().cloned().chain(parts.twins).collect();
    let mut out = Brep {vertices:complex.b.vertices.clone(),edges,faces}.compacted();
    outer_first(&mut out,&parted);
    Ok(out)
}

/// Each face of `parted` given the loop enclosing the others as its outer: a loop a healed
/// meeting parted in two.
fn outer_first(b: &mut Brep,parted: &[usize]) {
    for &f in parted {
        let face = &b.faces[f];
        let area = |l: &[Coedge]| b.loop_area(face,l).abs();
        let outer = (0..face.loops.len()).max_by(|&i,&j| area(&face.loops[i]).total_cmp(&area(&face.loops[j]))).unwrap_or(0);
        b.faces[f].loops.swap(0,outer);
    }
}

/// **Whether each cell of an arrangement lies in each of its operands** (closed solids): read off a
/// face of the operand bounding the cell where one does — behind it where the face faces the way
/// the operand's did — else at a point deep inside the cell placed against the operand.
pub fn members(complex: &Complex,operands: &[&Brep],tol: f64) -> Result<Vec<Vec<bool>>,String> {
    let cells = complex.cells()?;
    let located: Vec<Located> = operands.iter().map(|b| Located::new(b,tol)).collect();
    let mut out = vec![vec![false;operands.len()];cells.len()];
    for c in 1..cells.len() {
        let sides = complex.cell_sides(cells,c);
        let mut deep: Option<Option<V>> = None;
        for k in 0..operands.len() {
            let by_face = sides.iter().find_map(|&(f,front)| complex.origins[f].iter().find(|o| o.operand == k)
                .map(|o| o.alike != front));
            out[c][k] = match by_face {
                Some(inside) => inside,
                None => {
                    let p = *deep.get_or_insert_with(|| complex.inside(c).ok().flatten());
                    let Some(p) = p else { return Err(format!("cell {c} of the arrangement has no inside to read")) };
                    located[k].solid_place(p) == Place::In
                }
            };
        }
    }
    Ok(out)
}

/// **A Boolean of any number of solids at once**: their arrangement, each cell kept where `keep`
/// says of whether it lies in each (`members`), and the boundary between kept cells and the rest
/// (`Complex::select`), touching itself where kept cells meet along an edge or at a point and
/// saying so.
pub fn selected(operands: &[&Brep],keep: &dyn Fn(&[bool]) -> bool,tol: f64) -> Result<super::complex::Selected,String> {
    let complex = arrangement(operands,tol)?;
    let inside = members(&complex,operands,tol)?;
    complex.select(&|c| c != 0 && keep(&inside[c]))
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

/// The faces of the operands split where the others cross them, and along the spans two faces
/// only touch along (#735: a touch represented, the faces split along it — a Boolean's and a
/// split's healed out of their results, `heal`) (`Arranged`), phase by phase over the state they
/// share (`Arrangement`). The first operand's faces' boxes are taken from `known` where it has
/// them (`Known`). From no warm start, whatever the thread asked before, and its caller's left as
/// they were: what an arrangement is depends on its operands alone (`util::warm`).
fn arrange<'a>(solids: &[&'a Brep],known: Option<&Known>,given: &[Given],tol: f64) -> Result<Arranged<'a>,String> {
    crate::util::warm::fresh(|| arrange_phases(solids,known,given,tol))
}

/// `arrange`'s phases, the meetings `given` known before any is sought (#810).
fn arrange_phases<'a>(solids: &[&'a Brep],known: Option<&Known>,given: &[Given],tol: f64) -> Result<Arranged<'a>,String> {
    let mut arr = Arrangement::new(solids,known,tol);
    for g in given { arr.shared.give(&g.surfaces[0],&g.surfaces[1],&g.curves,tol); }
    // 1. every edge against every face of the other solid: where it crosses, a vertex on both
    let mut crossings = arr.crossings()?;
    // a root met flatly that is a crossing found besides is that crossing (`flat_roots`)
    let gone = arr.flat_roots(&mut crossings);
    arr.say.lap();
    // (a build past its budget goes no further, #777 V3)
    crate::util::meter::go_on()?;
    // 2. every pair of faces: their surfaces' curves, kept where they lie in both faces
    let meetings = arr.meetings(&crossings)?;
    crate::util::meter::go_on()?;
    // 2a. where three faces of three operands meet at a point no edge passes: two meetings laid
    // on one face, with faces of two other operands, crossing there (#735 A4)
    if arr.solids.0.len() > 2 { arr.triple_points(&meetings)?; }
    // 2b. every vertex, either solid's or a crossing's, cuts an edge it lies inside, the faces'
    // meetings among them: two edges running along one another (a shared circle, each solid
    // seamed at its own angle; a meeting traced along an edge already there) then share their
    // vertices, and step 4 takes them for one. Each edge boxed by its curve's enclosure, on every core.
    let inside = vertices_inside(&arr.edges,&arr.cuts,&arr.pool,&gone,arr.pad,tol);
    for (we,found) in inside.into_iter().enumerate() { arr.cuts[we].extend(found); }
    arr.say.lap();
    // 3. every original edge split where it was cut: its pieces, in order along it
    let (pieces_of,mut out) = split_edges(&arr.edges,&arr.cuts);
    // 3b. a piece shorter than a vertex's reach onto a curve, its ends one vertex (#735 A3)
    let gone = collapsed(&mut out,&mut arr.pool,tol);
    // 4. pieces that are one edge merged: the same ends and the same curve between
    let rep = merged(&out,tol);
    let split = Split {pieces_of,out,rep,gone};
    crate::util::meter::go_on()?;
    // 5. each face split by what lies on it, traced in its parameters
    let pieces = arr.faces_split(&meetings,&split)?;
    arr.say.arranged();
    // each edge a meeting laid, where no piece of an operand's own edge is merged into it (the
    // operands' edges are the first working edges)
    let own = arr.solids.all().map(|s| arr.solids[s].edges.len()).sum::<usize>();
    let mut met = vec![true;split.out.len()];
    for pieces in split.pieces_of.iter().take(own) {
        for &(p,_) in pieces { if !split.gone[p as usize] { met[split.rep[p as usize].0 as usize] = false } }
    }
    let Arrangement {solids,located,pool,clear,..} = arr;
    Ok(Arranged {solids,located,pool,out:split.out,met,pieces,clear})
}

/// **The working state `arrange`'s phases share**: the two solids and where each face lies, the
/// vertices pooled, and the working edges — A's, then B's, then the meetings phase 2 lays on the
/// faces — each with the points it is cut at, the two growing together (`add_edge`).
struct Arrangement<'a> {
    solids: Sides<&'a Brep>,
    located: Sides<Located<'a>>,
    pool: Pool,
    edges: Vec<WEdge>,
    /// where each working edge is cut, in step with `edges`: the parameter, and the vertex there
    cuts: Vec<Vec<(f64,u32)>>,
    /// where each solid's own edges begin among `edges`
    first_edge: Sides<usize>,
    /// each face's box
    boxes: Sides<Vec<Box3>>,
    /// the box each solid's faces fill
    reach: Sides<Box3>,
    /// each solid's faces clear of the others (`Arranged::clear`)
    clear: Sides<Vec<bool>>,
    tol: f64,
    /// how far apart two boxes may stand and still be taken to meet
    pad: f64,
    /// each pair of surfaces' meeting, found once and shared by every two faces on them (#735 A2),
    /// and each face's surface's index there, given the first time the face meets another
    shared: super::meeting::Shared,
    surface_of: BTreeMap<(Side,usize),usize>,
    say: Narration,
}

/// Phase 1's findings: every edge against every face of the other solid.
struct Crossings {
    /// each face's vertices where an edge of another crosses it, or crosses its boundary
    on_face: Sides<Vec<Vec<u32>>>,
    /// each face's edges of another operand lying in its surface (`Meets::Along`), working indices
    along_face: Sides<Vec<Vec<usize>>>,
    /// each root an edge meets a face at by its distance from the surface, at a vertex of its own
    loose: Vec<Loose>,
}

/// Phase 2's findings: every pair of faces.
struct Meetings {
    /// the working edges laid where two faces meet, each with the two faces
    ssi_edges: Vec<(usize,Two)>,
    /// faces of two operands on one surface: each splits the other with its boundary
    same_pairs: Vec<Two>,
    /// a traced meeting dropped as an edge one face has (`along_an_edge`): that edge (working), and
    /// the operand and face it is laid across wherever it lies inside it
    along_edges: Vec<(usize,Side,usize)>,
}

/// Phases 3 and 4: every working edge split where it was cut (`split_edges`), and the pieces that
/// are one edge merged (`merged`).
struct Split {
    /// each working edge's pieces in order along it: indices into `out`, and their parameters
    pieces_of: Vec<Vec<(u32,[f64;2])>>,
    out: Vec<WEdge>,
    /// each piece's representative among `out`, and whether it runs the other way round
    rep: Vec<(u32,bool)>,
    /// each piece collapsed into one vertex, shorter than the tolerance's reach (`collapsed`)
    gone: Vec<bool>,
}

impl<'a> Arrangement<'a> {
    /// Every operand's faces boxed and their edges pooled, nothing yet cut; the first's faces'
    /// boxes taken from `known` where it has them.
    fn new(operands: &[&'a Brep],known: Option<&Known>,tol: f64) -> Arrangement<'a> {
        let n = operands.len();
        let solids = Sides(operands.to_vec());
        let located = Sides::new(n,|s| if s == Side::A { Located::boxed(solids[s],tol,&|fi| known.and_then(|k| k.face_box(fi))) }
            else { Located::new(solids[s],tol) });
        let mut pool = Pool::new(tol);
        // the working edges: A's, then B's, and so on, their vertices pooled
        let mut edges: Vec<WEdge> = Vec::new();
        let mut first_edge = Sides(vec![0usize;n]);
        for s in solids.all() {
            first_edge[s] = edges.len();
            // each vertex keeping the tolerance it was built to (#735 A3), through every Boolean after
            let ids: Vec<u32> = solids[s].vertices.iter().map(|v| {
                let i = pool.at(v.p);
                pool.tols[i as usize] = pool.tols[i as usize].max(v.tol);
                i
            }).collect();
            for e in &solids[s].edges { edges.push(WEdge {curve:e.curve.clone(),t:e.t,v:[ids[e.v[0] as usize],ids[e.v[1] as usize]]}); }
        }
        let cuts = vec![Vec::new();edges.len()];
        let boxes = Sides::new(n,|s| (0..solids[s].faces.len()).map(|i| located[s].face_box(i)).collect::<Vec<_>>());
        let pad = 4.*tol;
        // the box each solid's faces fill, and the faces of each clear of every other's (`Arranged::clear`)
        let reach = Sides::new(n,|s| boxes[s].iter().fold(super::geom::EMPTY,|mut r,b| { super::geom::widen(&mut r,*b); r }));
        let clear = Sides::new(n,|s| boxes[s].iter().map(|bx| solids.all().all(|o| o == s || !overlap(bx,&reach[o],pad))).collect::<Vec<_>>());
        Arrangement {solids,located,pool,edges,cuts,first_edge,boxes,reach,clear,tol,pad,
            shared:Default::default(),surface_of:BTreeMap::new(),say:Narration::new()}
    }

    /// Edge `i` of solid `s`'s index among the working edges.
    fn working(&self,s: Side,i: usize) -> usize { self.first_edge[s]+i }

    /// A working edge added, cut nowhere yet: its index.
    fn add_edge(&mut self,curve: EdgeCurve,t: [f64;2],v: [u32;2]) -> usize {
        self.edges.push(WEdge {curve,t,v});
        self.cuts.push(Vec::new());
        self.edges.len()-1
    }

    /// The box two faces' boxes share, of two operands.
    fn shared(&self,faces: Two) -> Box3 {
        let (a,b) = (self.boxes[faces.s[0]][faces.f[0]],self.boxes[faces.s[1]][faces.f[1]]);
        (std::array::from_fn(|k| a.0[k].max(b.0[k])),std::array::from_fn(|k| a.1[k].min(b.1[k])))
    }

    /// **Phase 1**: every edge against every face of the other solid. Where it crosses, a vertex
    /// on both, the edge cut there; where it lies in the face's surface, cut only where it crosses
    /// the face's own edges.
    fn crossings(&mut self) -> Result<Crossings,String> {
        let (tol,pad) = (self.tol,self.pad);
        let n = self.solids.0.len();
        let mut found = Crossings {
            on_face: Sides::new(n,|s| vec![Vec::new();self.solids[s].faces.len()]),
            along_face: Sides::new(n,|s| vec![Vec::new();self.solids[s].faces.len()]),
            loose: Vec::new(),
        };
        // the faces each edge bounds
        let bounded: Sides<Vec<Vec<usize>>> = Sides::new(n,|s| {
            let sol = self.solids[s];
            let mut out = vec![Vec::new();sol.edges.len()];
            for (fi,f) in sol.faces.iter().enumerate() {
                for c in f.loops.iter().flatten() { if !out[c.edge as usize].contains(&fi) { out[c.edge as usize].push(fi); } }
            }
            out
        });
        for s in self.solids.all() {
          for other in self.solids.all().filter(|&o| o != s) {
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
                    let started = crate::util::clock::Instant::now();
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
                                for (t,q) in crossings_in(&f.surface,c,e.t,gc,ge.t,tol).map_err(|r| r.to_string())?.unwrap_or_default() {
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
                        // a reading not proved refuses the Boolean, with its witness (#814 P3)
                        Meets::Refused(r) => return Err(format!("an edge met with a face: {r}")),
                        Meets::At(roots) => for (t,touch) in roots {
                            // a touch is a double root, placed only to the square root of the
                            // tolerance: where a face of the edge's own touches the surface along a
                            // curve, it is the edge's crossing with that curve, placed exactly
                            let own: Vec<&Surface> = bounded[s][i].iter().map(|&g| &self.solids[s].faces[g].surface).collect();
                            let t = if touch { touch_root(c,e.t,&own,&f.surface,t,tol).unwrap_or(t) } else { t };
                            let q = c.point(t);
                            let place = self.located[other].face_place(fi,q);
                            self.say.meets(s,i,c,fi,f,q,touch,place);
                            if place == Place::Out { continue }
                            let fresh = self.pool.pts.len();
                            let v = self.pool.at(q);
                            self.cuts[we].push((t,v));
                            found.on_face[other][fi].push(v);
                            if v as usize >= fresh { found.loose.push(Loose {edge:we,vertex:v,t,side:s,index:i,against:other,face:fi}); }
                        },
                    }
                }
            }
          }
        }
        Ok(found)
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
            let s = &self.solids[l.against].faces[l.face].surface;
            let near = |t: f64| s.implicit(c.point(t)).abs() <= FLAT*tol.max(e.tol);
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
            for u in crossings.on_face[l.against][l.face].iter_mut() { if *u == l.vertex { *u = w } }
            gone.insert(l.vertex);
        }
        gone
    }

    /// **Phase 2**: every pair of faces whose boxes meet, their surfaces' meeting — in closed
    /// form, or traced (`traced_meeting`) — cut at the points both faces put on it, its spans
    /// laid as working edges where they lie in both faces.
    fn meetings(&mut self,crossings: &Crossings) -> Result<Meetings,String> {
        let mut found = Meetings {ssi_edges:Vec::new(),same_pairs:Vec::new(),along_edges:Vec::new()};
        let sides: Vec<Side> = self.solids.all().collect();
        // every two faces of two operands whose boxes meet, the lower operand first
        let mut pairs: Vec<Two> = Vec::new();
        for (k,&sa) in sides.iter().enumerate() { for &sb in &sides[k+1..] {
            for fa in 0..self.solids[sa].faces.len() { for fb in 0..self.solids[sb].faces.len() {
                if overlap(&self.boxes[sa][fa],&self.boxes[sb][fb],self.pad) { pairs.push(Two {s:[sa,sb],f:[fa,fb]}) }
            } }
        } }
        // each face's surface's index, and the box the faces on each surface fill: where a meeting of
        // two surfaces with no closed form is traced, once for every two faces on them
        let mut filled: BTreeMap<usize,Box3> = BTreeMap::new();
        for faces in &pairs {
            for k in 0..2 {
                let key = (faces.s[k],faces.f[k]);
                let i = match self.surface_of.get(&key) {
                    Some(&i) => i,
                    None => {
                        let i = self.shared.index(&self.solids[key.0].faces[key.1].surface,self.tol);
                        self.surface_of.insert(key,i);
                        i
                    }
                };
                super::geom::widen(filled.entry(i).or_insert(super::geom::EMPTY),self.boxes[key.0][key.1]);
            }
        }
        for faces in pairs {
            let ([sa,sb],[fa,fb]) = (faces.s,faces.f);
            let (a,b) = (self.solids[sa],self.solids[sb]);
            let surfaces = [&a.faces[fa].surface,&b.faces[fb].surface];
            // each two faces met counted against the build's budget (#777 V3): a stopped build meets
            // no more
            if !crate::util::meter::charge(crate::util::meter::Stage::Arrangement,1,|| {
                let (lo,hi) = self.shared(faces);
                (std::array::from_fn(|k| 0.5*(lo[k]+hi[k])),format!("{} × {}",surfaces[0].kind(),surfaces[1].kind()))
            }) { break }
            // the two surfaces' meeting, found once for every two faces on them
            let ids = [0,1].map(|k| self.surface_of[&(faces.s[k],faces.f[k])]);
            // two curves of one meeting in closed form may cross one another (`crossings_among`)
            let (curves,crossed) = match self.shared.meeting(ids[0],ids[1],self.tol) {
                Ssi::Curves(cs) => {
                    let mut x = crossings_among(surfaces[0],&cs,self.shared(faces),self.pad,self.tol)?;
                    ends_on_one_another(&cs,&mut x,self.tol);
                    (cs,x)
                }
                Ssi::Same => { found.same_pairs.push(faces); continue }
                Ssi::Traced => {
                    let (fi,fj) = (filled[&ids[0]],filled[&ids[1]]);
                    let within = (std::array::from_fn(|k| fi.0[k].max(fj.0[k])),std::array::from_fn(|k| fi.1[k].min(fj.1[k])));
                    let known = self.shared.traced(ids[0],ids[1]).to_vec();
                    let mut traced = self.traced_meeting(faces,crossings,&known,within)?;
                    self.shared.add_traced(ids[0],ids[1],traced[known.len()..].iter().cloned());
                    // a traced meeting lying along an edge either face already has is that edge
                    // (#735 A2: "this meeting is that existing edge"), laid once, not twice
                    // a hair apart: two surfaces meeting along a circle both their caps end on;
                    // the other face is cut by that edge where it lies inside it (a torus's seam
                    // in a plane through its axis)
                    traced.retain(|c| match self.along_an_edge(c,faces,surfaces) {
                        Some((k,e)) => {
                            found.along_edges.push((self.working(faces.s[k],e),faces.s[1-k],faces.f[1-k]));
                            false
                        }
                        None => true,
                    });
                    let met = ends_met(&traced,self.tol);
                    (traced,met)
                }
            };
            self.say.curves(faces,surfaces,curves.len());
            let (lo,hi) = self.shared(faces);
            let (lo,hi): (V,V) = (lo.map(|x| x-self.pad),hi.map(|x| x+self.pad));
            for (c,crossed) in curves.into_iter().zip(crossed) {
                // a curve that comes nowhere near both faces (a far crossing of two nearly parallel
                // meridians) leaves nothing on them
                if far(&c,lo,hi) { continue }
                let mut ts = self.points_on(&c,surfaces,&crossings.on_face[sb][fb],&crossings.on_face[sa][fa],crossed);
                let domain = closed_domain(&c,self.tol);
                let closed = c.period().is_some() || domain.is_some();
                let spans = spans(&c,domain,&mut ts,&mut self.pool);
                self.say.points_on(&c,&ts,&self.pool);
                for (t,v) in spans {
                    if (t[1]-t[0])*c.speed() <= self.tol || (v[0] == v[1] && !closed) { continue }
                    if !self.kept(&c,t,v,faces,surfaces) { continue }
                    let we = self.add_edge(EdgeCurve::Curve(c.clone()),t,v);
                    found.ssi_edges.push((we,faces));
                }
            }
        }
        Ok(found)
    }

    /// The edge of either of the two faces the traced curve `c` lies along, over all of it — which
    /// face's (0 or 1) and the edge, its own solid's: each of its points within the reach a point
    /// of two surfaces meeting at its angle has (`on_meeting`) of the edge's curve, inside the
    /// edge's stretch.
    fn along_an_edge(&self,c: &Curve,faces: Two,surfaces: [&Surface;2]) -> Option<(usize,usize)> {
        let Curve::Traced(t) = c else { return None };
        // each point with the reach a point of the two surfaces meeting at its angle has
        let pts: Vec<(V,f64)> = (0..=32).map(|k| {
            let p = t.pts[(k*(t.pts.len()-1))/32];
            (p,on_meeting(self.tol,norm(crate::util::space::cross(surfaces[0].gradient(p),surfaces[1].gradient(p)))))
        }).collect();
        (0..2).find_map(|k| {
            let sol = self.solids[faces.s[k]];
            let f = &sol.faces[faces.f[k]];
            f.loops.iter().flatten().find(|co| {
                let e = &sol.edges[co.edge as usize];
                let EdgeCurve::Curve(ec) = &e.curve else { return false };
                pts.iter().all(|&(p,reach)| {
                    let mut u = ec.inverse(p);
                    if let Some(period) = ec.period() { u = around(u,e.t[0],period); }
                    u >= e.t[0]-1e-9 && u <= e.t[1]+1e-9 && distance(ec.point(u),p) <= reach
                })
            }).map(|co| (k,co.edge as usize))
        })
    }

    /// **Where three faces of three operands meet** (an arrangement of more than two): on each
    /// face, every two meetings laid across it with faces of two other operands, crossing — a
    /// point no edge of any operand passes, so no crossing of phase 1 made it — cut at a vertex
    /// they share.
    fn triple_points(&mut self,meetings: &Meetings) -> Result<(),String> {
        let tol = self.tol;
        // **each point three surfaces share computed once** (#711, #777 M2): found from each of the
        // three faces it lies on (each holds two of its three meetings), it is solved the first
        // time, by the three surfaces' indices among the meetings' (`meeting::Shared`), and every
        // later finding of it near there takes that vertex — never solved again and snapped
        let mut shared: BTreeMap<[usize;3],Vec<(V,u32)>> = BTreeMap::new();
        let mut on: BTreeMap<(Side,usize),Vec<(usize,Side,usize)>> = BTreeMap::new();
        for &(we,two) in &meetings.ssi_edges {
            for k in 0..2 { on.entry((two.s[k],two.f[k])).or_default().push((we,two.s[1-k],two.f[1-k])); }
        }
        for ((s,fi),list) in on {
            let surface = &self.solids[s].faces[fi].surface;
            for i in 0..list.len() { for j in i+1..list.len() {
                let ((ei,oi,gi),(ej,oj,gj)) = (list[i],list[j]);
                if oi == oj { continue }
                let (EdgeCurve::Curve(ci),EdgeCurve::Curve(cj)) = (&self.edges[ei].curve,&self.edges[ej].curve) else { continue };
                let (ti,tj) = (self.edges[ei].t,self.edges[ej].t);
                if !overlap(&ci.bounds(ti),&cj.bounds(tj),self.pad) { continue }
                // each meeting's reach at a point: a point of it within the tolerance of its two
                // surfaces stands that over the sine of their angle off it (`on_meeting`), so a
                // meeting traced between surfaces meeting shallowly is known only so well
                let (si,sj) = (&self.solids[oi].faces[gi].surface,&self.solids[oj].faces[gj].surface);
                let reach_of = |o: &Surface,p: V| on_meeting(tol,norm(crate::util::space::cross(
                    crate::util::space::normalised(surface.gradient(p)).unwrap_or([0.;3]),
                    crate::util::space::normalised(o.gradient(p)).unwrap_or([0.;3]))));
                // in closed form where `query` has one, else by search (`meeting::crossings`), as far
                // as the shallower of the two meetings anywhere along them is known, each candidate
                // then held to the reach of the two at it
                let widest = super::meeting::reach_along(&[(ci,ti,si),(cj,tj,sj)],&|o,p| reach_of(o,p));
                let found: Vec<(f64,V)> = crossings_in(surface,ci,ti,cj,tj,tol).map_err(|r| r.to_string())?.unwrap_or_else(||
                    super::meeting::crossings(ci,ti,cj,tj,widest).into_iter()
                        .filter(|&(t,u,_)| distance(ci.point(t),cj.point(u)) <= reach_of(si,ci.point(t)).max(reach_of(sj,cj.point(u))))
                        .map(|(t,_,q)| (t,q)).collect());
                // each the point the three surfaces share, by Newton on their signed distances: what
                // each of the three faces' searches finds again, so the pool takes it for one
                let three = [surface,&self.solids[oi].faces[gi].surface,&self.solids[oj].faces[gj].surface];
                let key = [(s,fi),(oi,gi),(oj,gj)].map(|k| self.surface_of.get(&k).copied());
                let key: Option<[usize;3]> = key.iter().all(Option::is_some).then(|| { let mut k = key.map(|x| x.unwrap_or(0)); k.sort(); k });
                for (t0,q) in found {
                    let q0 = q;
                    // (within the reach Newton settles it from, `shared_point`'s; a touch's crossing,
                    // known only along a stretch, is read again)
                    let known = key.and_then(|k| shared.get(&k)?.iter().find(|(p,_)| distance(*p,q0) <= 16.*ON_CURVE*tol).copied());
                    let q = match known {
                        Some((p,_)) => p,
                        None => match shared_point(three,q,tol).or_else(|| touch_point(three,q,tol)) {
                            Some(q) => q,
                            // two of the three touch there (their gradients parallel), so the two
                            // curves are tangent and their crossing is known only as the stretch
                            // along which they stay within reach of one another: a vertex either
                            // has in it already is that crossing, else their nearest approach is
                            // (#735 A3)
                            None => self.crossing_in_touch([ei,ej],t0,q),
                        },
                    };
                    let t = { let mut t = ci.inverse(q); if let Some(period) = ci.period() { t = around(t,ti[0],period); } t };
                    let near = |c: &Curve,x: f64,o: &Surface| distance(c.point(x),q) <= reach_of(o,q).max(distance(q,q0)+ON_CURVE*tol);
                    if !(t >= ti[0]-1e-12 && t <= ti[1]+1e-12) || !near(ci,t,si) { continue }
                    let mut u = cj.inverse(q);
                    if let Some(period) = cj.period() { u = around(u,tj[0],period); }
                    if !(u >= tj[0]-1e-12 && u <= tj[1]+1e-12) || !near(cj,u,sj) { continue }
                    let v = match known { Some((_,v)) => v,None => self.pool.at(q) };
                    if let (Some(k),None) = (key,known) { shared.entry(k).or_default().push((q,v)) }
                    if !self.cuts[ei].iter().any(|&(_,w)| w == v) { self.cuts[ei].push((t,v)); }
                    if !self.cuts[ej].iter().any(|&(_,w)| w == v) { self.cuts[ej].push((u,v)); }
                }
            } }
        }
        Ok(())
    }

    /// **Where two meetings tangent at a crossing cross** (`triple_points`, its three surfaces'
    /// Newton singular there): of working edges `e[0]` and `e[1]`, found near `q` at `t` along the
    /// first, the stretch of the first within a vertex's reach onto a curve of the second (out from
    /// `t` each way, doubling, as `flat_roots` reads a root met flatly); the vertex already on
    /// either inside the stretch nearest `q` — one of their ends, or a cut — else the point of the
    /// stretch nearest the second curve.
    fn crossing_in_touch(&self,e: [usize;2],t: f64,q: V) -> V {
        let reach = ON_CURVE*self.tol;
        let (EdgeCurve::Curve(ci),EdgeCurve::Curve(cj)) = (&self.edges[e[0]].curve,&self.edges[e[1]].curve) else { return q };
        let span = self.edges[e[0]].t;
        let off = |t: f64| { let p = ci.point(t); distance(cj.point(cj.inverse(p)),p) };
        let out = |dir: f64| {
            let (mut d,mut last) = (1e-12*(span[1]-span[0]).abs().max(1e-300),t);
            loop {
                let x = (t+dir*d).clamp(span[0],span[1]);
                if off(x) > reach { return last }
                last = x;
                if x == span[0] || x == span[1] { return last }
                d *= 2.;
            }
        };
        let (lo,hi) = (out(-1.),out(1.));
        let on_ci = |p: V| {
            let mut u = ci.inverse(p);
            if let Some(period) = ci.period() { u = around(u,span[0],period); }
            (u >= lo && u <= hi && distance(ci.point(u),p) <= reach).then_some(u)
        };
        let known = e.iter().flat_map(|&k| self.edges[k].v.into_iter().chain(self.cuts[k].iter().map(|&(_,v)| v)))
            .map(|v| self.pool.pts[v as usize]).filter(|&p| on_ci(p).is_some() && off(ci.inverse(p)) <= reach)
            .min_by(|a,b| distance(*a,q).total_cmp(&distance(*b,q)));
        if let Some(p) = known { return p }
        const N: usize = 32;
        let best = (0..=N).map(|k| lo+(hi-lo)*k as f64/N as f64).min_by(|&a,&b| off(a).total_cmp(&off(b))).unwrap_or(t);
        ci.point(best)
    }

    /// The vertices on curve `c`, where two faces' surfaces (`surfaces`) meet, each with its
    /// parameter along it: those on B's face and on A's (`on_b`, `on_a`: where the other solid's
    /// edges cross it) within reach of the curve, then those of `crossed`, where it crosses
    /// another curve of the same meeting, each pooled.
    fn points_on(&mut self,c: &Curve,surfaces: [&Surface;2],on_b: &[u32],on_a: &[u32],crossed: Vec<V>) -> Vec<(f64,u32)> {
        let (sa,sb) = (surfaces[0],surfaces[1]);
        let mut ts: Vec<(f64,u32)> = Vec::new();
        for &v in on_b.iter().chain(on_a) {
            let p = self.pool.pts[v as usize];
            let t = c.inverse(p);
            // a point within the tolerance of both surfaces is that over the sine of the angle
            // they meet at off their meeting
            let sine = norm(crate::util::space::cross(sa.gradient(p),sb.gradient(p)));
            if distance(c.point(t),p) <= on_meeting(self.tol,sine) && !ts.iter().any(|&(_,w)| w == v) { ts.push((t,v)); }
        }
        for q in crossed {
            let v = self.pool.at(q);
            if !ts.iter().any(|&(_,w)| w == v) { ts.push((c.inverse(q),v)); }
        }
        ts
    }

    /// Whether the span `t` of curve `c`, from vertex `v[0]` to `v[1]`, is kept as an edge where
    /// two faces meet: its middle in both and on each surface's patch, whether the two cross or touch.
    fn kept(&self,c: &Curve,t: [f64;2],v: [u32;2],faces: Two,surfaces: [&Surface;2]) -> bool {
        let m = c.point((t[0]+t[1])/2.);
        let places = [0,1].map(|k| self.located[faces.s[k]].face_place(faces.f[k],m));
        self.say.span(t,v,m,places,&self.pool);
        if places[0] == Place::Out || places[1] == Place::Out { return false }
        // on each surface's patch, not only its extension: a surface over a curve's domain (a
        // revolution's meridian, an extrusion's section, a sheet) inverts a point off it onto its
        // rim, which a face's place reads as on its boundary (a hyperboloid's rulings run on past
        // the heights it is built over, where a plane holding one meets it)
        // (planes, cylinders, cones, spheres and tori run on without end, and need no reading)
        if surfaces.iter().any(|s| s.bounded() && distance(s.point(s.inverse(m)),m) > 1e3*self.tol) { return false }
        // where the two only touch (their normals parallel) nothing crosses from one side to the
        // other, and the span is laid all the same: a touch represented (#735)
        true
    }

    /// **Where two faces meet in curves no closed form gives**, traced: through the points where
    /// either face's boundary crosses the other, and where a grid across a sheet's face crosses
    /// the other surface — a curve lying inside both faces, or ending on a sheet's boundary
    /// beyond them, crosses no edge — then, between analytic surfaces, the smaller face searched
    /// for what no curve yet passes, each found traced in turn, until the search shows nothing
    /// else is there. Where the two only touch is a meeting already known (`Touches`): no trace
    /// starts or passes there.
    fn traced_meeting(&self,faces: Two,crossings: &Crossings,known: &[Curve],bx: Box3) -> Result<Vec<Curve>,String> {
        // **a meeting the march finds nearly touching is read from its critical set** (#778 T3, T4;
        // `meeting::marched`, the one fallback): the ridges of one face's distance from the other
        // over its parameters — the touches along curves through the inside of both faces, and the
        // crossings beside them however shallow — and the crossings no ridge holds traced as
        // `f = 0` in either face's parameters through the points where the faces' edges cross the
        // other, known before the march, which ends where it comes to one
        // (each face's parameters and the seeds read only where the march is refused)
        let past_inputs = || {
            let ([oa,ob],[fa,fb]) = (faces.s,faces.f);
            let (a,b) = (self.solids[oa],self.solids[ob]);
            let side = |s: &'a Brep,f: usize| (&s.faces[f].surface,super::meeting::within_domain(&s.faces[f].surface,parameters(s,f,true)));
            let on = &crossings.on_face;
            ([side(a,fa),side(b,fb)],on[ob][fb].iter().chain(&on[oa][fa]).map(|&v| self.pool.pts[v as usize]).collect())
        };
        super::meeting::marched(None,self.tol,past_inputs,&mut |ridged,past| self.traced_meeting_with(faces,crossings,known,ridged,past,bx))
            .map(|(traced,_)| traced)
    }

    /// `traced_meeting`'s march, the branches a ridge holds (`ridged`) known before it: each a curve
    /// of the meeting, a touch among them a stretch a march ends at; `past` a refusal read past
    /// (`meeting::marched`).
    fn traced_meeting_with(&self,faces: Two,crossings: &Crossings,known: &[Curve],ridged: &super::meeting::Sampled,past: bool,
        (lo,hi): Box3) -> Result<Vec<Curve>,super::meeting::Unmarched> {
        let (tol,pad) = (self.tol,self.pad);
        let ([oa,ob],[fa,fb]) = (faces.s,faces.f);
        let (a,b) = (self.solids[oa],self.solids[ob]);
        let surfaces = [&a.faces[fa].surface,&b.faces[fb].surface];
        let (sa,sb) = (surfaces[0],surfaces[1]);
        let on = &crossings.on_face;
        let mut seeds: Vec<V> = on[ob][fb].iter().chain(&on[oa][fa]).map(|&v| self.pool.pts[v as usize])
            .filter(|&p| sa.implicit(p).abs() <= 8.*tol && sb.implicit(p).abs() <= 8.*tol).collect();
        // the edges either lies in that the other's face holds (`Meets::Along`), over their stretches
        let along = &crossings.along_face;
        let mut stretches: Vec<(&Curve,[f64;2])> = along[ob][fb].iter().chain(&along[oa][fa])
            .filter_map(|&we| match &self.edges[we].curve {
                EdgeCurve::Curve(c) => Some((c,self.edges[we].t)),
                EdgeCurve::Degenerate => None,
            }).collect();
        // a ridge's touches are stretches the two touch along, as an edge they lie along is
        for br in ridged.branches().iter().filter(|br| br.contact == super::meeting::Contact::Touching) {
            if let Curve::BSpline(s) = &br.curve { stretches.push((&br.curve,s.domain())) }
        }
        let mut touches = Touches::new(self,faces,&stretches);
        // a seed on a curve of the meeting traced for two other faces on these surfaces is on it, or
        // on a ridge's branch
        let on_known = |p: V| known.iter().any(|c| distance(c.point(c.inverse(p)),p) <= 8.*tol)
            || super::meeting::on_a_branch(&self.solids[faces.s[0]].faces[faces.f[0]].surface,&self.solids[faces.s[1]].faces[faces.f[1]].surface,ridged,p,tol);
        seeds.retain(|&p| !touches.touching(p) && !on_known(p));
        let started = crate::util::clock::Instant::now();
        // a sheet's or a face family's wall: no closed form says where a meeting may lie
        let grid = if sb.sheet() { grid_seeds(b,fb,sa,tol) }
            else if sa.sheet() { grid_seeds(a,fa,sb,tol) } else { Vec::new() };
        self.say.seeded(started.elapsed().as_secs_f64());
        // (only where both faces may be: a seed beyond their shared box traces nothing on them)
        seeds.extend(grid.into_iter().filter(|p| (0..3).all(|k| p[k] >= lo[k]-pad && p[k] <= hi[k]+pad))
            .filter(|&p| !touches.touching(p) && !on_known(p)));
        self.say.tracing(faces,surfaces,&seeds);
        let started = crate::util::clock::Instant::now();
        // beside a ridge's or a level curve's branch is beside a known meeting
        let beside = &mut |p: V| touches.beside(p) || super::meeting::on_a_branch(sa,sb,ridged,p,tol);
        let mut traced = known.to_vec();
        traced.extend(ridged.branches().iter().map(|br| br.curve.clone()));
        if past {
            // read from its ridges and level curves (#778): each seed left traced alone
            super::meeting::traced_alone(&seeds,&mut traced,&mut |p| super::ssi::trace_beside(sa,sb,&[p],beside,lo,hi,tol),tol);
        } else {
            traced.extend(super::ssi::trace_beside(sa,sb,&seeds,beside,lo,hi,tol)?);
        }
        // a closed curve inside both faces crosses no edge and has no seed
        if super::ssi::searchable(sa) && super::ssi::searchable(sb) {
            let size = |bx: &Box3| distance(bx.0,bx.1);
            let (sol,fi,mine,theirs) = if size(&self.boxes[ob][fb]) < size(&self.boxes[oa][fa]) { (b,fb,sb,sa) } else { (a,fa,sa,sb) };
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
                        sa.kind(),sb.kind()).into())
                }
                let more = match super::ssi::trace_beside(sa,sb,&[p],beside,lo,hi,tol) {
                    // past a refusal, one refused again stands where the branches read it
                    Err(_) if past => break,
                    other => other?,
                };
                if more.is_empty() {
                    return Err(format!("a {} and a {} meet at {p:?}, where no edge crosses, in a curve that could not be \
                        traced, not built yet",sa.kind(),sb.kind()).into())
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
        for s in self.solids.all() {
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
                    if on.holds(s,fi).is_none() { continue }
                    for &(piece,_) in &split.pieces_of[e] { if !split.gone[piece as usize] { cut.internal(split.rep[piece as usize].0); } }
                }
                // an edge's pieces that lie inside this face (on its patch too: a sheet lying in a
                // plane is that plane over its patch alone, `face_holds`)
                let inside = |we: usize,cut: &mut FaceCut| for &(piece,_) in &split.pieces_of[we] {
                    if split.gone[piece as usize] { continue }
                    let r = split.rep[piece as usize].0;
                    let w = &split.out[r as usize];
                    if matches!(w.curve,EdgeCurve::Degenerate) { continue }
                    if self.located[s].face_holds(fi,w.point((w.t[0]+w.t[1])/2.,&self.pool)) { cut.internal(r); }
                };
                // a face of the other on this one's surface: its boundary's pieces inside this face
                for same in &meetings.same_pairs {
                    let Some(k) = same.holds(s,fi) else { continue };
                    let (other,face) = (same.s[1-k],same.f[1-k]);
                    for c in self.solids[other].faces[face].loops.iter().flatten() { inside(self.working(other,c.edge as usize),&mut cut); }
                }
                // an edge of the other a traced meeting lay along
                for &(we,_,_) in meetings.along_edges.iter().filter(|&&(_,o,face)| o == s && face == fi) { inside(we,&mut cut); }
                if s != Side::A { self.say.cut_network(s,fi,f,meetings,split,&self.pool); }
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
                    if split.gone[piece as usize] { continue }
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
        _ => c.speed() > 1e3*(1.+norm(crate::util::space::sub(hi,lo)))
            && !(0..256).any(|j| { let q = c.point(TAU*j as f64/256.); (0..3).all(|k| q[k] >= lo[k] && q[k] <= hi[k]) }),
    }
}

/// A curve's stretches between the points on it (`ts`, each a parameter and a vertex), which are
/// sorted along it: a closed curve's from each to the next and from the last round to the first,
/// or — with none — the whole curve from a vertex pooled at its start; an open curve's between
/// each two in turn.
fn spans(c: &Curve,domain: Option<[f64;2]>,ts: &mut [(f64,u32)],pool: &mut Pool) -> Vec<([f64;2],[u32;2])> {
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
            // a clamped B-spline whose ends meet (`domain`: a rational ellipse's rim cut square to
            // its sweep) is closed with no period: round it as a closed curve, its stretches split
            // where its parameter starts again, at the vertex both its ends are
            if let Some([lo,hi]) = domain {
                let start = pool.at(c.point(lo));
                let mut at: Vec<(f64,u32)> = ts.iter().map(|&(t,v)| (if v == start { lo } else { t },v)).collect();
                if at.first().map_or(true,|x| x.1 != start) { at.insert(0,(lo,start)); }
                at.push((hi,start));
                for w in at.windows(2) { if w[1].0 > w[0].0 { spans.push(([w[0].0,w[1].0],[w[0].1,w[1].1])); } }
                return spans
            }
            for w in ts.windows(2) { spans.push(([w[0].0,w[1].0],[w[0].1,w[1].1])); }
        }
    }
    spans
}

/// A clamped B-spline's parameter domain where its two ends are one point (within `tol`) and it
/// runs round a loop far wider than that: a closed curve with no period, whose parameter does not
/// run round — not a fitted meeting shrunk to a point, all of it within a few tolerances of its ends.
fn closed_domain(c: &Curve,tol: f64) -> Option<[f64;2]> {
    let Curve::BSpline(b) = c else { return None };
    let [lo,hi] = b.domain();
    let start = c.point(lo);
    if distance(start,c.point(hi)) > tol { return None }
    (1..16).any(|k| distance(c.point(lo+(hi-lo)*k as f64/16.),start) > 64.*tol).then_some([lo,hi])
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
    faces: Two,
    surfaces: [&'s Surface;2],
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
    fn new(arr: &'s Arrangement<'a>,faces: Two,stretches: &'s [(&'s Curve,[f64;2])]) -> Touches<'s,'a> {
        let surfaces = [0,1].map(|k| &arr.solids[faces.s[k]].faces[faces.f[k]].surface);
        let size = |k: usize| { let bx = arr.boxes[faces.s[k]][faces.f[k]]; distance(bx.0,bx.1) };
        let spread = 8.*(arr.tol*size(0).max(size(1))).sqrt();
        let least = size(0).min(size(1));
        Touches {arr,faces,surfaces,stretches,spread,least,read:Vec::new()}
    }

    /// Whether the two meet at `p` nearly tangent: too shallowly for a trace to follow.
    fn shallow(&self,p: V) -> bool {
        norm(crate::util::space::cross(self.surfaces[0].gradient(p),self.surfaces[1].gradient(p))) < super::ssi::SHALLOW
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
        let (tol,(sa,sb)) = (self.arr.tol,(self.surfaces[0],self.surfaces[1]));
        let Some(at) = super::ssi::touch_near(sa,sb,p,tol) else { return false };
        let Some(t) = super::ssi::touch(sa,sb,at) else { return false };
        if t.radius > 0.05*self.least || distance(at,p) > t.radius { return false }
        let place = |k: usize,d: V| {
            let s = self.surfaces[k];
            let q = crate::util::space::add(at,crate::util::space::scale(d,2.*t.radius));
            self.arr.located[self.faces.s[k]].face_place(self.faces.f[k],s.point(s.inverse(q)))
        };
        let lone = !t.branches.iter().any(|&d| place(0,d) != Place::Out && place(1,d) != Place::Out);
        if lone { self.arr.say.touch_alone(self.faces,self.surfaces,&t); }
        self.read.push((t,lone));
        lone
    }

    /// **Where they touch outside a face**: no part of what is built, so no trace need pass it (a
    /// fillet's canal run on past the face that cuts it off lies tangent to the face it rolls on
    /// beyond the contact's end).
    fn outside(&self,p: V) -> bool {
        (0..2).any(|k| {
            let s = self.surfaces[k];
            self.arr.located[self.faces.s[k]].face_place(self.faces.f[k],s.point(s.inverse(p))) == Place::Out
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
    clock: crate::util::clock::Instant,
    /// the clock at the end of each phase timed: the edges against the faces, the pairs of faces
    laps: Vec<f64>,
    /// the time spent laying grids of seeds across sheets' faces, and tracing
    grid: Cell<f64>,
    trace: Cell<f64>,
}

/// Two faces of a pair, of two operands, by their operands, indices and surfaces' kinds.
fn named(faces: Two,surfaces: [&Surface;2]) -> String {
    format!("{}{} {} × {}{} {}",faces.s[0].name(),faces.f[0],surfaces[0].kind(),faces.s[1].name(),faces.f[1],surfaces[1].kind())
}

impl Narration {
    fn new() -> Narration {
        Narration {debug:super::debug(),timing:super::timing(),clock:crate::util::clock::Instant::now(),laps:Vec::new(),
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
    fn touch_alone(&self,faces: Two,surfaces: [&Surface;2],t: &super::ssi::Touch) {
        if self.debug { eprintln!("brep: {}: touch alone at {:?} ({} branches, radius {:.1e})",named(faces,surfaces),
            t.at,t.branches.len(),t.radius); }
    }

    /// Grids of seeds laid across a sheet's face, in `took` seconds.
    fn seeded(&self,took: f64) { self.grid.set(self.grid.get()+took); }

    /// Two faces' meeting about to be traced from `seeds`.
    fn tracing(&self,faces: Two,surfaces: [&Surface;2],seeds: &[V]) {
        if self.debug { eprintln!("brep: {}: tracing from {} seed(s) {seeds:?}",named(faces,surfaces),seeds.len()); }
    }

    /// The search finding a meeting of two faces no edge crosses, at `p`.
    fn unseen(&self,faces: Two,surfaces: [&Surface;2],p: V) {
        if self.debug { eprintln!("brep: {}: a meeting no edge crosses, at {p:?}",named(faces,surfaces)); }
    }

    /// Two faces' meeting traced, as `traced`, from `seeds`, in `took` seconds.
    fn traced(&self,faces: Two,surfaces: [&Surface;2],traced: &[Curve],seeds: &[V],took: f64) {
        self.trace.set(self.trace.get()+took);
        if took > 0.2 && self.timing {
            eprintln!("time: {}: traced {} curves from {} seeds in {took:.2} s",named(faces,surfaces),traced.len(),seeds.len());
        }
        if !self.debug { return }
        crate::util::warm::kept(|| for c in traced {
            if let Curve::Traced(t) = c {
                eprintln!("brep:   traced {} points, closed {}, from {:?} to {:?}; seeds off it by {:?}",t.pts.len(),t.closed,
                    t.pts[0],t.pts[t.pts.len()-1],seeds.iter().map(|&p| distance(c.point(c.inverse(p)),p)).collect::<Vec<_>>());
            }
        })
    }

    /// Two faces' surfaces meeting in `n` curves.
    fn curves(&self,faces: Two,surfaces: [&Surface;2],n: usize) {
        if self.debug && n > 0 { eprintln!("brep: {}: {n} curve(s)",named(faces,surfaces)); }
    }

    /// A curve of a meeting and the points on it (`ts`).
    fn points_on(&self,c: &Curve,ts: &[(f64,u32)],pool: &Pool) {
        if self.debug { eprintln!("brep:   {} with {} point(s) on it: {:?}",c.kind(),ts.len(),ts.iter().map(|x| pool.pts[x.1 as usize]).collect::<Vec<_>>()); }
    }

    /// A span `t` of a meeting's curve, from vertex `v[0]` to `v[1]`, its middle `m` where `places` says in each face.
    fn span(&self,t: [f64;2],v: [u32;2],m: V,places: [Place;2],pool: &Pool) {
        if self.debug { eprintln!("brep:     span {t:?} from {:?} to {:?}: middle {m:?} {:?}",pool.pts[v[0] as usize],pool.pts[v[1] as usize],
            (places[0],places[1])); }
    }

    /// The cut network on face `fi` of operand `s`: each vertex's count of cut edges (an odd one is
    /// an open end).
    fn cut_network(&self,s: Side,fi: usize,f: &Face,meetings: &Meetings,split: &Split,pool: &Pool) {
        if !self.debug { return }
        let mut degree: BTreeMap<u32,usize> = BTreeMap::new();
        let cut: Vec<u32> = meetings.ssi_edges.iter().filter(|e| e.1.holds(s,fi).is_some())
            .flat_map(|&(e,_)| split.pieces_of[e].iter().map(|&(p,_)| split.rep[p as usize].0)).collect();
        for &e in &cut { for v in split.out[e as usize].v { *degree.entry(v).or_insert(0) += 1; } }
        let open: Vec<V> = degree.iter().filter(|(_,&d)| d % 2 == 1).map(|(&v,_)| pool.pts[v as usize]).collect();
        eprintln!("arrange: face {fi} of {} ({}): {} cut edges, open ends {:?}",s.name(),f.surface.kind(),cut.len(),open);
    }
}

/// **The point three surfaces share near `q`**: Newton onto their three zero sets
/// (`roots::newton_onto`), no step longer than `1e3 tol`; `None` where that does not settle, or
/// where two of the three meet there shallower than a trace can follow (`ssi::SHALLOW`): touching,
/// every point near where they touch is within the tolerance of all three, and settling says
/// nothing of which.
fn shared_point(surfaces: [&Surface;3],q: V,tol: f64) -> Option<V> {
    let x = crate::util::roots::newton_onto::<3>(q,1e3*tol,|x| Some(surfaces.map(|s| (s.gradient(x),s.implicit(x)))),
        |at| at.iter().all(|r| r.1.abs() <= 1e-3*tol)).filter(|&x| distance(x,q) <= 1e3*tol)?;
    let g = surfaces.map(|s| crate::util::space::normalised(s.gradient(x)).unwrap_or([0.;3]));
    let steep = |a: V,b: V| norm(crate::util::space::cross(a,b)) >= super::ssi::SHALLOW;
    (steep(g[0],g[1]) && steep(g[0],g[2]) && steep(g[1],g[2])).then_some(x)
}

/// **Where three surfaces meet, two of them touching there**: a touching pair (their normals
/// nearer parallel at `q` than a trace can follow) meet along a curve their closed form gives, and
/// where the third crosses that curve steeply the point is the third's root along it, by Newton on
/// the curve's parameter from its foot of `q`: the same point whichever face's search came near it.
/// Each touching pair is tried, the nearest parallel first, and the first whose third crosses its
/// curve steeply (not, as where all three touch, at a double root placed only to the square root of
/// the tolerance) is taken. `None` where none does, or the root does not settle within `1e4 tol`
/// of `q`.
fn touch_point(surfaces: [&Surface;3],q: V,tol: f64) -> Option<V> {
    let unit = |s: &Surface,p: V| crate::util::space::normalised(s.gradient(p)).unwrap_or([0.;3]);
    let sine = |a: usize,b: usize| norm(crate::util::space::cross(unit(surfaces[a],q),unit(surfaces[b],q)));
    let mut pairs: Vec<(usize,usize,usize)> = [(0,1,2),(0,2,1),(1,2,0)].into_iter().filter(|&(a,b,_)| sine(a,b) < super::ssi::SHALLOW).collect();
    pairs.sort_by(|x,y| sine(x.0,x.1).total_cmp(&sine(y.0,y.1)));
    pairs.into_iter().find_map(|(a,b,c)| {
        let Ssi::Curves(curves) = super::ssi::intersect(surfaces[a],surfaces[b],tol) else { return None };
        let (curve,mut t) = curves.iter().map(|k| (k,k.inverse(q)))
            .min_by(|x,y| distance(x.0.point(x.1),q).total_cmp(&distance(y.0.point(y.1),q)))?;
        let third = surfaces[c];
        for _ in 0..40 {
            let (p,d) = curve.d1(t);
            let f = third.implicit(p);
            let slope = crate::util::space::dot(third.gradient(p),d);
            if f.abs() <= 1e-3*tol {
                let steep = slope.abs() >= super::ssi::SHALLOW*norm(d)*norm(third.gradient(p));
                return (steep && distance(p,q) <= 1e4*tol).then_some(p)
            }
            if !(slope.abs() > 0.) { return None }
            t -= f/slope;
        }
        None
    })
}

/// **Where an edge touching a surface touches it** (a root of its distance that is also a root of
/// its rate): where one of the surfaces of the edge's own faces (`own`) touches `other` there,
/// along a curve their closed form gives, the edge's crossing with that curve — a crossing on
/// the edge's face, steep where the edge leaves the touch, found by Gauss–Newton on the two
/// curves' parameters from `t` and the curve's foot of the edge's point. The edge's parameter
/// there; `None` where no face of its touches `other`, or the crossing does not settle within
/// `1e4 tol` of the root.
fn touch_root(c: &Curve,span: [f64;2],own: &[&Surface],other: &Surface,t: f64,tol: f64) -> Option<f64> {
    let q = c.point(t);
    let unit = |s: &Surface| crate::util::space::normalised(s.gradient(q)).unwrap_or([0.;3]);
    own.iter().find_map(|&face| {
        if norm(crate::util::space::cross(unit(face),unit(other))) >= super::ssi::SHALLOW { return None }
        let Ssi::Curves(curves) = super::ssi::intersect(face,other,tol) else { return None };
        let k = curves.iter().min_by(|a,b| distance(a.point(a.inverse(q)),q).total_cmp(&distance(b.point(b.inverse(q)),q)))?;
        let (mut x,mut u) = (t,k.inverse(q));
        for _ in 0..30 {
            let ((p,dp),(r,dr)) = (c.d1(x),k.d1(u));
            let res = crate::util::space::sub(p,r);
            if norm(res) <= 1e-3*tol {
                // the two crossing steeply, or the crossing is no better placed than the root
                let steep = norm(crate::util::space::cross(dp,dr)) >= super::ssi::SHALLOW*norm(dp)*norm(dr);
                return (steep && distance(p,q) <= 1e4*tol && x >= span[0]-1e-12 && x <= span[1]+1e-12).then_some(x)
            }
            // [dp, -dr] [dx, du] = -res, by its normal equations
            let (a11,a12,a22) = (crate::util::space::dot(dp,dp),-crate::util::space::dot(dp,dr),crate::util::space::dot(dr,dr));
            let (b1,b2) = (-crate::util::space::dot(dp,res),crate::util::space::dot(dr,res));
            let det = a11*a22-a12*a12;
            if !(det.abs() > 1e-300) { return None }
            x += (a22*b1-a12*b2)/det;
            u += (a11*b2-a12*b1)/det;
        }
        None
    })
}

/// Step 2b's vertices: for each working edge, the pooled vertices (none of `gone`) lying inside
/// it, off its ends, and where along it; each edge boxed by its curve's enclosure, on every core.
fn vertices_inside(edges: &[WEdge],cuts: &[Vec<(f64,u32)>],pool: &Pool,gone: &std::collections::BTreeSet<u32>,
    pad: f64,tol: f64) -> Vec<Vec<(f64,u32)>> {
    // the vertices in order along x: an edge reads those within its box's stretch of x, not every
    // vertex (#589), each in the pool's order
    let mut along: Vec<u32> = (0..pool.pts.len() as u32).collect();
    along.sort_by(|&v,&w| pool.pts[v as usize][0].total_cmp(&pool.pts[w as usize][0]).then(v.cmp(&w)));
    crate::util::par::indices(edges.len(),|we| -> Vec<(f64,u32)> {
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
            super::tolerant::on_stretch(c,e.t,p,tol).map(|t| (t,v))
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

/// **Step 3b** (#735 A3, `tolerant`): every piece shorter than a vertex's reach onto a curve
/// (`ON_CURVE` tolerances, along it) collapsed, its two ends one vertex — the lower, its tolerance
/// grown to the other's distance — so no edge is left whose length is noise beside what the other
/// rules merge (a crossing a hair from a vertex, which the pool's own reach left apart). Which
/// pieces went; every piece's ends renumbered.
fn collapsed(out: &mut [WEdge],pool: &mut Pool,tol: f64) -> Vec<bool> {
    let reach = ON_CURVE*tol;
    let mut alias = crate::util::union_find::UnionFind::new(pool.pts.len());
    let mut gone = vec![false;out.len()];
    for (i,e) in out.iter().enumerate() {
        if e.v[0] == e.v[1] { continue }
        let EdgeCurve::Curve(c) = &e.curve else { continue };
        // no shorter than its chord: most pieces are read no further than their ends
        if distance(pool.pts[e.v[0] as usize],pool.pts[e.v[1] as usize]) > reach { continue }
        let at = |k: usize| c.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/4.);
        if (0..4).map(|k| distance(at(k),at(k+1))).sum::<f64>() > reach { continue }
        gone[i] = true;
        // the lower vertex the class's (the union keeps its first argument's root)
        let (a,b) = (alias.find(e.v[0] as usize),alias.find(e.v[1] as usize));
        alias.union(a.min(b),a.max(b));
    }
    if !gone.iter().any(|&g| g) { return gone }
    for v in 0..pool.pts.len() {
        let r = alias.find(v);
        if r != v { let d = distance(pool.pts[r],pool.pts[v]); pool.tols[r] = pool.tols[r].max(d); }
    }
    for e in out.iter_mut() { e.v = e.v.map(|v| alias.find(v as usize) as u32); }
    gone
}

/// Step 4: each piece's representative among `out` (the first piece with the same ends and the
/// same curve between), and whether it runs the other way round.
fn merged(out: &[WEdge],tol: f64) -> Vec<(u32,bool)> {
    let mut rep: Vec<(u32,bool)> = (0..out.len() as u32).map(|i| (i,false)).collect();
    let mut by_ends: BTreeMap<(u32,u32),Vec<u32>> = BTreeMap::new();
    for (i,e) in out.iter().enumerate() { by_ends.entry((e.v[0].min(e.v[1]),e.v[0].max(e.v[1]))).or_default().push(i as u32); }
    for group in by_ends.values() {
        for (k,&i) in group.iter().enumerate() {
            let ei = &out[i as usize];
            for &j in &group[..k] {
                if rep[j as usize].0 != j { continue }
                let ej = &out[j as usize];
                let (EdgeCurve::Curve(ci),EdgeCurve::Curve(cj)) = (&ei.curve,&ej.curve) else { continue };
                let Some(along) = super::tolerant::same_edge(ci,ei.t,ei.v,cj,ej.t,ej.v,tol) else { continue };
                rep[i as usize] = (j,!along);
                break
            }
        }
    }
    rep
}

/// Where the curves two surfaces meet in (`curves`, in closed form) cross one another, over their
/// stretches within the faces' shared box `bx` (grown by `pad`), each crossing listed against both:
/// two curves of one meeting cross only where the surfaces touch — a singular point of the meeting
/// (#735 A2): two cylinders of one radius whose axes cross meet in two ellipses crossing where both
/// touch, a plane tangent to a hyperboloid in the two rulings through the point it touches at — and
/// there each is split as at an edge.
fn crossings_among(surface: &super::geom::Surface,curves: &[super::geom::Curve],bx: Box3,pad: f64,tol: f64) -> Result<Vec<Vec<V>>,String> {
    let mut crossed = vec![Vec::new();curves.len()];
    let stretches: Vec<Option<[f64;2]>> = curves.iter().map(|c| stretch_in(c,bx,pad)).collect();
    for i in 0..curves.len() {
        for j in i+1..curves.len() {
            let (Some(ti),Some(tj)) = (stretches[i],stretches[j]) else { continue };
            for (_,q) in crossings_in(surface,&curves[i],ti,&curves[j],tj,tol).map_err(|r| r.to_string())?.unwrap_or_default() {
                crossed[i].push(q);
                crossed[j].push(q);
            }
        }
    }
    Ok(crossed)
}

/// The stretch of a closed-form curve to read for crossings within the box `bx` grown by `pad`: a
/// closed one's period, a line's run inside the box's slabs, a spline's domain; none for a line
/// missing the box.
fn stretch_in(c: &Curve,(lo,hi): Box3,pad: f64) -> Option<[f64;2]> {
    if let Some(p) = c.period() { return Some([0.,p]) }
    match *c {
        Curve::Line {p,d} => {
            let (mut t0,mut t1) = (f64::NEG_INFINITY,f64::INFINITY);
            for k in 0..3 {
                if d[k].abs() < 1e-300 { if p[k] < lo[k]-pad || p[k] > hi[k]+pad { return None } continue }
                let (a,z) = ((lo[k]-pad-p[k])/d[k],(hi[k]+pad-p[k])/d[k]);
                t0 = t0.max(a.min(z)); t1 = t1.min(a.max(z));
            }
            (t0 < t1).then_some([t0,t1])
        }
        Curve::BSpline(ref b) => Some(b.domain()),
        _ => None,
    }
}

/// **Where an open curve of a meeting in closed form ends on another of it**: a meeting handed over
/// in pieces (a screwed circle's branches ending on the circle where it is a round side's section,
/// `ssi`'s `screwed_circle_meeting`) ends where its pieces meet, which no crossing of two curves
/// finds; each such end is laid as a point on the curve it ends and on the one it ends on.
fn ends_on_one_another(curves: &[super::geom::Curve],crossed: &mut [Vec<V>],tol: f64) {
    use super::geom::Curve;
    for i in 0..curves.len() {
        let Curve::Traced(t) = &curves[i] else { continue };
        if t.closed { continue }
        for p in [t.pts[0],t.pts[t.pts.len()-1]] {
            crossed[i].push(p);
            for (j,c) in curves.iter().enumerate() {
                if j == i { continue }
                if distance(c.point(c.inverse(p)),p) <= 8.*tol { crossed[j].push(p) }
            }
        }
    }
}

/// Where traced curves of one meeting (`curves`) end at one another, each such end listed against
/// its curve: the branches of a meeting through a point where the two surfaces touch end there
/// (`ssi::trace_beside`), each at a vertex they share, which no edge crossing need make.
fn ends_met(curves: &[super::geom::Curve],tol: f64) -> Vec<Vec<V>> {
    use super::geom::Curve;
    let ends = |c: &Curve| match c {
        Curve::BSpline(b) if c.period().is_none() => { let [t0,t1] = b.domain(); vec![c.point(t0),c.point(t1)] }
        Curve::Traced(t) if !t.closed => vec![t.pts[0],t.pts[t.pts.len()-1]],
        _ => Vec::new(),
    };
    let all: Vec<Vec<V>> = curves.iter().map(ends).collect();
    (0..curves.len()).map(|i| all[i].iter().copied().filter(|&p|
        all.iter().enumerate().any(|(j,es)| j != i && es.iter().any(|&q| distance(p,q) <= 8.*tol))).collect()).collect()
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
        for k in 0..2 { v[k] = *vmap.entry(e.v[k]).or_insert_with(|| {
            result.vertices.push(super::topo::Vertex {p:pool.pts[e.v[k] as usize],tol:pool.tols[e.v[k] as usize]});
            (result.vertices.len()-1) as u32
        }); }
        let id = result.edge(e.curve.clone(),e.t,v);
        result.edges[id as usize].tol = built_tol(e,pool);
        emap.insert(i as u32,id);
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

/// A cycle's signed area in its face's parameters, the material on its left: the trapezoid rule
/// along each half at 64 points, then twice as many, until two readings agree to a thousandth (or
/// 4096 a half).
fn fine_area(cycle: &[Half],f: &Face,out: &[WEdge],pool: &Pool) -> f64 {
    let read = |n: usize| cycle.iter().map(|h| (0..n).map(|k| {
        let (a,b) = (oriented(f,at(h,f,out,pool,k as f64/n as f64)),oriented(f,at(h,f,out,pool,(k+1) as f64/n as f64)));
        a[0]*b[1]-a[1]*b[0]
    }).sum::<f64>()).sum::<f64>()/2.;
    let (mut n,mut last) = (64,read(64));
    while n < 4096 {
        n *= 2;
        let next = read(n);
        if (next-last).abs() <= 1e-3*next.abs() { return next }
        last = next;
    }
    last
}

/// How near two ways at a node may be (radians, each read a ten-thousandth of its own half's length
/// along it) and be read again at one length by their curvature: two curves tangent there part by
/// as much as a step along an arc turns its chord, a ten-thousandth of the arc's angle.
const TANGENT_TIE: f64 = 1e-3;

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
    // each half's direction leaving its start and arriving at its end, as lengths, oriented, read a
    // part `s` of its way along it
    let dir_at = |h: &Half,at_start: bool,s: f64| -> f64 {
        let (p,q) = if at_start { (h.from,at(h,f,out,pool,s)) } else { (h.to,at(h,f,out,pool,1.-s)) };
        let (_,su,sv) = f.surface.d1(p);
        let (ku,kv) = (norm(su).max(1e-12),norm(sv).max(1e-12));
        let (p,q) = (oriented(f,p),oriented(f,q));
        ((q[1]-p[1])*kv).datan2((q[0]-p[0])*ku)
    };
    let leave: Vec<f64> = halves.iter().map(|h| dir_at(h,true,1e-4)).collect();
    let arrive_back: Vec<f64> = halves.iter().map(|h| dir_at(h,false,1e-4)).collect();
    // where two halves leave a node within rounding of one way (two curves tangent there), read
    // apart by how they turn: the chord's way at a step `δ` along a half is `θ₀ + κ δ/2`, so read at
    // `δ` and `2δ` of its own length it gives `κ`, and the two compared at one length (the shorter
    // step) part by their curvature. Read at a part of each one's own length alone, an arc five times
    // another's radius and five times its length leaves a point they touch at the same way to
    // rounding (#721: a drum at its end pose touching the cylinders its generators turn into)
    let length = |h: &Half| -> f64 {
        let e = &out[h.edge as usize];
        let pts: Vec<V> = (0..=8).map(|k| e.point(param(h,out,k as f64/8.),pool)).collect();
        pts.windows(2).map(|w| distance(w[0],w[1])).sum::<f64>().max(1e-300)
    };
    // a half's way leaving its start (or arriving back at its end) read at the length `delta` along
    // it: its chord to the point that far from the node, the part of its way there found by halving.
    // A curve's parameter runs at no one speed, so no part of it stands for a length: a trace's
    // counts its points, which crowd where it sets out, and two probes a ten-thousandth and two of
    // its parameter along it stood within a millionth of the node, where their ways, extrapolated to
    // `delta`, read 80 radians (#791: the ball-end mill's pipe meeting its side's foot plane at the
    // touch helix, tangent there to the foot's rim)
    let way_at = |g: usize,at_start: bool,delta: f64| -> f64 {
        let h = &halves[g];
        let e = &out[h.edge as usize];
        let node = e.point(param(h,out,if at_start { 0. } else { 1. }),pool);
        let reach = |s: f64| distance(node,e.point(param(h,out,if at_start { s } else { 1.-s }),pool));
        // bracket the part, halved until short of `delta` or doubled until past it, and halved to
        // a millionth of it: a way read that far off its length turns by a millionth of what two
        // tied curves part by there
        let (mut lo,mut hi) = (0.,1e-4);
        if reach(hi) > delta {
            while hi > 1e-300 && reach(0.5*hi) > delta { hi *= 0.5 }
            lo = 0.5*hi;
        } else {
            while hi < 1. && reach(hi) < delta { lo = hi; hi = (2.*hi).min(1.) }
        }
        let (_,hi) = crate::util::roots::bisect(lo,hi,|a,b| 0.5*(a+b),|a,b| b-a > 1e-6*b,|m| reach(m) < delta);
        dir_at(h,at_start,hi)
    };
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
            let twin = |g: usize| halves[g].edge == halves[h].edge && halves[g].along != halves[h].along;
            let candidates: Vec<usize> = outgoing[node].iter().copied().filter(|&g| !(twin(g) && outgoing[node].len() > 1)).collect();
            // where two of the ways here (the way back and those leaving) agree within rounding — two
            // curves tangent at the node — all are read at one length along each, by their curvature
            let near = |x: f64,y: f64| { let d = (x-y).rem_euclid(TAU); d <= TANGENT_TIE || TAU-d <= TANGENT_TIE };
            // (the halves tied, the way back's among them where it is: the length they are read at is a
            // ten-thousandth of the shortest of them, the scale they are drawn at — not of a short
            // half beside them, where a fitted curve's own small lean off tangent would rule)
            let back_tied = candidates.iter().any(|&g| near(leave[g],arrive_back[h]));
            let tied: Vec<usize> = candidates.iter().copied().filter(|&g| near(leave[g],arrive_back[h])
                || candidates.iter().any(|&g2| g2 != g && near(leave[g],leave[g2]))).collect();
            let tangent = !tied.is_empty();
            let (back,leaving): (f64,Vec<(usize,f64)>) = if tangent {
                let delta = tied.iter().chain(back_tied.then_some(&h)).map(|&g| 1e-4*length(&halves[g])).fold(f64::INFINITY,f64::min);
                (way_at(h,false,delta),candidates.iter().map(|&g| (g,way_at(g,true,delta))).collect())
            } else { (arrive_back[h],candidates.iter().map(|&g| (g,leave[g])).collect()) };
            if tangent && super::debug() { eprintln!("pieces tie at node {node} from half {h} (back {:.9} read {back:.9}): {:?}",arrive_back[h],
                leaving.iter().map(|&(g,w)| (g,leave[g],w,length(&halves[g]))).collect::<Vec<_>>()); }
            // the clockwise turn from `back` to each leaving way, in (0, 2π]: the least
            let best = leaving.iter().map(|&(g,w)| {
                let mut turn = back-w;
                while turn <= 1e-12 { turn += TAU; }
                while turn > TAU+1e-12 { turn -= TAU; }
                (turn,g)
            }).fold(None,|b: Option<(f64,usize)>,t| if b.is_none_or(|b| t.0 < b.0) { Some(t) } else { b });
            let Some((_,g)) = best else { return Err(format!("a {}'s pieces end nowhere",f.surface.kind())) };
            h = g;
        }
        cycles.push(cycle);
    }
    // outer cycles turn counter-clockwise (the material on their left), holes clockwise
    let mut cycles: Vec<Vec<Half>> = cycles.iter().map(|c| c.iter().map(|&h| halves[h].clone()).collect()).collect();
    let bar = reading::bar(&cycles,out,pool);
    let drawn: Vec<Outline> = cycles.iter().map(|c| reading::outline(std::slice::from_ref(c),f,out,pool,bar)).collect();
    // each cycle's signed area, the material on its left: its outline's, where that stands clear of
    // what the outline's chords may miss along its whole length; else, a piece thinner than its
    // outline's sag (a cusp-thin strip between two curves tangent at its end), along its curves
    // sampled ever finer until the area settles (#735 A3: a sliver's side read exactly, not by a
    // drawing coarser than it is wide)
    let areas: Vec<f64> = drawn.iter().zip(&cycles).map(|(o,c)| {
        let area = if f.reversed { -o.area() } else { o.area() };
        let (mut uv_len,mut space_len) = (0.,0.);
        for l in o.loops() {
            for k in 0..l.len() {
                let (a,b) = (l[k],l[(k+1)%l.len()]);
                uv_len += (b[0]-a[0]).hypot(b[1]-a[1]);
                space_len += distance(f.surface.point(a),f.surface.point(b));
            }
        }
        let sag_uv = o.sag()*if space_len > 0. { uv_len/space_len } else { 1. };
        if area.abs() > 4.*sag_uv*uv_len { area } else { fine_area(c,f,out,pool) }
    }).collect();
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
    use crate::util::space::distance;

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
/// them. A cell holding a solid face's outside is the outside, left out; so is a shell every face of
/// which it holds by both sides — a sheet's piece inside a cell that parts nothing, a fin with no
/// edge on the cell's boundary (one an earlier sheet cut off already reaching it, #812): it bounds
/// nothing, and is no cell. A piece slit into a cell from its boundary is of the cell's shell.
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
    let clock = crate::util::clock::Instant::now();
    // touches laid as a Boolean lays them, and healed out of each cell as out of its result
    let Arranged {solids,located,pool,out,met,pieces,..} = arrange(&[solid,sheets],None,&[],tol)?;
    let laid: Vec<Option<u32>> = (0..out.len()).map(|e| met[e].then_some(e as u32)).collect();
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
        // a piece with no inside bounds nothing: merged into its edge (#735 A3, as a Boolean's)
        let Some(&first) = inner.peek() else { continue };
        let place = inner.map(|p| located[Side::A].solid_place(p)).find(|&p| p != Place::On).unwrap_or(Place::On);
        if super::debug() {
            eprintln!("split: a piece of the sheet's {}: {place:?}, first read at {first:?} ({} loops)",f.surface.kind(),piece.len());
        }
        if place == Place::In { kept.push(i); }
    }
    if super::timing() {
        eprintln!("time: split: arranged {arranged:.2} s, the sheet's pieces placed {:.2} s",clock.elapsed().as_secs_f64()-arranged);
    }
    // the complex the kept pieces part the solid into (#735 A1): its shells, of which those a face
    // of the solid bounds from outside are the outside, left out, and each other a cell
    let faces: Vec<Face> = kept.iter().map(|&i| face_of(i,false)).collect();
    let origins: Vec<Vec<Origin>> = kept.iter().map(|&i| vec![Origin {operand:pieces[i].0.0,face:pieces[i].1,alike:true}]).collect();
    let complex = Complex::new(working_brep(&out,&pool,faces),origins,tol)?;
    let outside: std::collections::BTreeSet<usize> = kept.iter().enumerate().filter(|&(_,&i)| pieces[i].0 == Side::A)
        .map(|(f,_)| complex.shell_sides[f][1]).collect();
    if super::debug() {
        let mut all: Vec<(usize,usize,bool)> = vec![(0,0,false);complex.shells.len()];
        for (k,shell) in complex.shells.iter().enumerate() {
            for &(f,front) in shell {
                if pieces[kept[f]].0 == Side::A { all[k].0 += 1 } else { all[k].1 += 1 }
                if pieces[kept[f]].0 == Side::A && front { all[k].2 = true; }
            }
        }
        eprintln!("split: {} solid pieces, {} sheet pieces kept; classes (solid sides, sheet sides, outside): {:?}",
            kept.iter().filter(|&&i| pieces[i].0 == Side::A).count(),kept.iter().filter(|&&i| pieces[i].0 == Side::B).count(),all);
    }
    let mut cells = Vec::new();
    for (k,shell) in complex.shells.iter().enumerate() {
        if outside.contains(&k) || shell.iter().all(|&(f,front)| shell.contains(&(f,!front))) { continue }
        let own: Vec<usize> = shell.iter().filter(|&&(f,front)| !(front && pieces[kept[f]].0 == Side::A)).map(|&(f,_)| f).collect();
        let faces: Vec<Face> = shell.iter().filter(|&&(f,front)| !(front && pieces[kept[f]].0 == Side::A))
            .map(|&(f,front)| face_of(kept[f],front)).collect();
        let origin: Vec<(usize,usize)> = own.iter().map(|&f| (pieces[kept[f]].0.0,pieces[kept[f]].1)).collect();
        cells.push(healed(&out,&pool,faces,&origin,&laid,&mut Known::default()));
    }
    Ok(cells)
}

/// The box of parameters face `fi` of `b` reaches, sampled along its edges (`parameters`).
pub fn face_parameters(b: &Brep,fi: usize) -> [[f64;2];2] { parameters(b,fi,false) }

/// The box of face `fi`'s parameters its loops reach, sampled along each edge. `whole`: grown
/// to hold the face though a pcurve bulges between samples — by a sixteenth of each side, or to a
/// whole period where that comes near one.
pub(crate) fn parameters(b: &Brep,fi: usize,whole: bool) -> [[f64;2];2] {
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
