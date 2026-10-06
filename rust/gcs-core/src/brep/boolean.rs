//! Booleans of two solids (docs/rust-kernel-plan.md, rung 1): every edge of each split where it
//! crosses the other's faces, every pair of faces intersected and the curves kept where they lie in
//! both, every face split by what now lies on it — traced in its own parameters, the material on
//! the left — each piece placed against the other solid, and the pieces the operation keeps
//! assembled, their edges shared by construction.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::geom::{around,Uv,V};
use super::query::{crossings_in,face_box,curve_surface_within,Located,Meets,Place};
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

fn overlap(a: &([f64;3],[f64;3]),b: &([f64;3],[f64;3]),pad: f64) -> bool {
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
    solids: [&'a Brep;2],
    located: [Located<'a>;2],
    pool: Pool,
    out: Vec<WEdge>,
    pieces: Vec<(usize,usize,Vec<Vec<Half>>)>,
}

/// **Two solids sharing a face, joined there** (the pieces of a chain of fillets, each ending in
/// the section the next begins in): where a plane face of `a` and one of `b` are the same face —
/// one plane, material either side of it, every edge of the one an edge of the other, end for end
/// and through the middle, within `tol` — the two faces go and their edges become one, used by
/// the faces beside them. Nothing is intersected, so `a` and `b` must meet nowhere else: the
/// caller's to know. `None` where they share no such face.
pub fn glued(a: &Brep,b: &Brep,tol: f64) -> Option<Brep> {
    use super::geom::Surface;
    let near = |p: V,q: V| distance(p,q) <= tol;
    let outward = |f: &Face| match f.surface { Surface::Plane(fr) => Some(if f.reversed { fr.z.map(|x| -x) } else { fr.z }),_ => None };
    let middle = |s: &Brep,e: usize| s.edges[e].point(0.5*(s.edges[e].t[0]+s.edges[e].t[1]),&s.vertices);
    let ends = |s: &Brep,e: usize| s.edges[e].v.map(|v| s.vertices[v as usize].p);
    // each face of `b` the same as one of `a`: its edges matched to `a`'s, and whether each runs the
    // other way
    let (mut shared_a,mut shared_b) = (std::collections::BTreeSet::new(),std::collections::BTreeSet::new());
    let mut edge_of: BTreeMap<usize,(u32,bool)> = BTreeMap::new();
    for (i,fa) in a.faces.iter().enumerate() {
        let Some(na) = outward(fa) else { continue };
        for (j,fb) in b.faces.iter().enumerate() {
            let Some(nb) = outward(fb) else { continue };
            if shared_b.contains(&j) || crate::space::dot(na,nb) > -1.+1e-9 { continue }
            if !super::ssi::same(&fa.surface,&fb.surface,tol) { continue }
            let (la,lb): (Vec<usize>,Vec<usize>) = (fa.loops.iter().flatten().map(|c| c.edge as usize).collect(),
                fb.loops.iter().flatten().map(|c| c.edge as usize).collect());
            if la.len() != lb.len() { continue }
            let matched: Option<Vec<(usize,u32,bool)>> = lb.iter().map(|&eb| la.iter().find_map(|&ea| {
                let ([a0,a1],[b0,b1]) = (ends(a,ea),ends(b,eb));
                let flip = if near(a0,b0) && near(a1,b1) { false } else if near(a0,b1) && near(a1,b0) { true } else { return None };
                near(middle(a,ea),middle(b,eb)).then_some((eb,ea as u32,flip))
            })).collect();
            let Some(matched) = matched else { continue };
            shared_a.insert(i);
            shared_b.insert(j);
            for (eb,ea,flip) in matched { edge_of.insert(eb,(ea,flip)); }
        }
    }
    if shared_a.is_empty() { return None }
    let mut out = a.clone();
    // `b`'s vertices: those of a shared edge are `a`'s, the rest its own
    let mut vertex_of: BTreeMap<u32,u32> = BTreeMap::new();
    for (&eb,&(ea,flip)) in &edge_of {
        let (vb,va) = (b.edges[eb].v,a.edges[ea as usize].v);
        let va = if flip { [va[1],va[0]] } else { va };
        for k in 0..2 { vertex_of.insert(vb[k],va[k]); }
    }
    for (i,v) in b.vertices.iter().enumerate() {
        if !vertex_of.contains_key(&(i as u32)) { vertex_of.insert(i as u32,out.vertices.len() as u32); out.vertices.push(v.clone()); }
    }
    // `b`'s edges: a shared one is `a`'s (its tolerance the larger), the rest its own
    let mut new_edge: Vec<(u32,bool)> = Vec::with_capacity(b.edges.len());
    for (i,e) in b.edges.iter().enumerate() {
        if let Some(&(ea,flip)) = edge_of.get(&i) {
            let t = &mut out.edges[ea as usize].tol;
            *t = t.max(e.tol);
            new_edge.push((ea,flip));
        } else {
            let mut e = e.clone();
            e.v = e.v.map(|v| vertex_of[&v]);
            out.edges.push(e);
            new_edge.push(((out.edges.len()-1) as u32,false));
        }
    }
    out.faces = a.faces.iter().enumerate().filter(|(i,_)| !shared_a.contains(i)).map(|(_,f)| f.clone()).collect();
    for (j,f) in b.faces.iter().enumerate() {
        if shared_b.contains(&j) { continue }
        let mut f = f.clone();
        for c in f.loops.iter_mut().flatten() {
            let (e,flip) = new_edge[c.edge as usize];
            // a use of an edge now `a`'s: its pcurve by its ends (a pcurve read at its own edge's
            // parameter reads another's wrongly), and from the other end where that runs the
            // other way
            if edge_of.contains_key(&(c.edge as usize)) && (flip || matches!(c.pcurve,Pcurve::Curve(_))) {
                let old = &b.edges[c.edge as usize];
                let (u0,u1) = (c.pcurve.at(old.t[0],old,&f.surface,&b.vertices),c.pcurve.at(old.t[1],old,&f.surface,&b.vertices));
                let (a,z) = if flip { (u1,u0) } else { (u0,u1) };
                c.pcurve = match c.pcurve { Pcurve::Line {..} => Pcurve::Line {a,b:z},_ => Pcurve::Inverse {a,b:z} };
                c.reversed ^= flip;
            }
            c.edge = e;
        }
        out.faces.push(f);
    }
    Some(out)
}

/// `a` combined with `b` by `op`, to `tol` (a length).
pub fn boolean(a: &Brep,b: &Brep,op: Op,tol: f64) -> Result<Brep,String> {
    let Arranged {solids,located,pool,out,pieces,..} = arrange(a,b,tol)?;
    let mut kept: Vec<Face> = Vec::new();
    for (s,fi,piece) in pieces {
        let f = &solids[s].faces[fi];
        // place a point inside the piece against the other solid: the first of its inner
        // points not on the other's boundary (a piece may touch it tangentially), or one on
        // it where the other has a face on this surface
        let candidates = inside(&piece,f,&out,&pool)?;
        let (p,place) = candidates.iter().map(|&uv| { let p = f.surface.point(uv); (p,located[1-s].solid_place(p)) })
            .find(|&(p,place)| place != Place::On || (0..solids[1-s].faces.len()).any(|g|
                super::ssi::same(&f.surface,&solids[1-s].faces[g].surface,tol) && located[1-s].face_place(g,p) == Place::In))
            .unwrap_or_else(|| { let p = f.surface.point(candidates[0]); (p,Place::On) });
        let keep = if place == Place::On {
            // on a face of the other on the same surface: kept once, from A, where the two
            // face the same way and the operation keeps that side
            let other = 1-s;
            let partner = (0..solids[other].faces.len()).find(|&g| {
                super::ssi::same(&f.surface,&solids[other].faces[g].surface,tol)
                    && located[other].face_place(g,p) == Place::In
            });
            let Some(g) = partner else {
                let near: Vec<String> = (0..solids[other].faces.len()).filter(|&g| solids[other].faces[g].surface.implicit(p).abs() <= tol)
                    .map(|g| format!("{} {:?} same {}",solids[other].faces[g].surface.kind(),located[other].face_place(g,p),
                        super::ssi::same(&f.surface,&solids[other].faces[g].surface,tol))).collect();
                return Err(format!("a piece of a {} touches the other solid's boundary at {p:?} ({})",f.surface.kind(),near.join(", ")))
            };
            let normal = |face: &Face| {
                let n = face.surface.normal_raw(face.surface.inverse(p));
                if face.reversed { crate::space::scale(n,-1.) } else { n }
            };
            let alike = crate::space::dot(normal(f),normal(&solids[other].faces[g])) > 0.;
            s == 0 && match op { Op::Union | Op::Common => alike,Op::Cut => !alike }
        } else { match (op,s,place) {
            (Op::Union,_,Place::Out) | (Op::Cut,0,Place::Out) | (Op::Cut,1,Place::In) | (Op::Common,_,Place::In) => true,
            _ => false,
        } };
        if !keep { continue }
        let flip = op == Op::Cut && s == 1;
        let loops: Vec<Vec<Coedge>> = piece.iter().map(|cycle| {
            let mut l: Vec<Coedge> = cycle.iter().map(|h| Coedge {edge:h.edge,reversed:!h.along,pcurve:h.pcurve.clone()}).collect();
            if flip { l = l.into_iter().rev().map(|c| Coedge {reversed:!c.reversed,..c}).collect(); }
            l
        }).collect();
        kept.push(Face {surface:f.surface.clone(),reversed:f.reversed != flip,loops,name:f.name.clone()});
    }
    Ok(assemble(&out,&pool,kept))
}

/// The faces of `a` and `b` split where the other crosses them (`Arranged`).
fn arrange<'a>(a: &'a Brep,b: &'a Brep,tol: f64) -> Result<Arranged<'a>,String> {
    // `SOLVENT_BREP_DEBUG` narrates every pair of faces: the curves, the spans kept and dropped
    let debug = std::env::var_os("SOLVENT_BREP_DEBUG").is_some();
    let solids = [a,b];
    let located = [Located::new(a,tol),Located::new(b,tol)];
    let mut pool = Pool::new(tol);
    // the working edges: A's, then B's, their vertices pooled
    let mut edges: Vec<WEdge> = Vec::new();
    let mut first_edge = [0usize;2];
    for (s,sol) in solids.iter().enumerate() {
        first_edge[s] = edges.len();
        let ids: Vec<u32> = sol.vertices.iter().map(|v| pool.at(v.p)).collect();
        for e in &sol.edges { edges.push(WEdge {curve:e.curve.clone(),t:e.t,v:[ids[e.v[0] as usize],ids[e.v[1] as usize]]}); }
    }
    let boxes: [Vec<_>;2] = [(0..a.faces.len()).map(|i| face_box(a,i)).collect(),(0..b.faces.len()).map(|i| face_box(b,i)).collect()];
    let pad = 4.*tol;
    let clock = crate::clock::Instant::now();
    let (mut t_grid,mut t_trace) = (0f64,0f64);
    // 1. every edge against every face of the other solid: where it crosses, a vertex on both
    let mut cuts: Vec<Vec<(f64,u32)>> = vec![Vec::new();edges.len()];
    let mut on_face: [Vec<Vec<u32>>;2] = [vec![Vec::new();a.faces.len()],vec![Vec::new();b.faces.len()]];
    // each face's edges of the other solid lying in its surface (`Meets::Along`), working indices
    let mut along_face: [Vec<Vec<usize>>;2] = [vec![Vec::new();a.faces.len()],vec![Vec::new();b.faces.len()]];
    for s in 0..2 {
        let other = 1-s;
        for (i,e) in solids[s].edges.iter().enumerate() {
            let EdgeCurve::Curve(c) = &e.curve else { continue };
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
                let started = crate::clock::Instant::now();
                // an edge measured off its surfaces by more than the tolerance (a fitted contact's
                // `tol`, `json::measure`) is read at its own: it lies in a face it is that near
                let meets = curve_surface_within(&c,e.t,&f.surface,tol.max(e.tol),boxes[other][fi],pad);
                if started.elapsed().as_secs_f64() > 0.1 && std::env::var_os("SOLVENT_BREP_TIME").is_some() {
                    eprintln!("time: edge {i} of {} ({}, {:.3} long) against face {fi} ({}, feature {:.3e}): {:.2} s",["A","B"][s],c.kind(),
                        c.speed()*(e.t[1]-e.t[0]),f.surface.kind(),f.surface.feature(),started.elapsed().as_secs_f64());
                }
                match meets {
                    // lying in the face's surface, it is split only where it crosses the face's
                    // own edges, which no surface tells: each crossing a vertex on both (a
                    // fillet's contact circle laid on a cylinder across the cylinder's seam)
                    Meets::Along => {
                        along_face[other][fi].push(we);
                        let edges: std::collections::BTreeSet<u32> = f.loops.iter().flatten().map(|g| g.edge).collect();
                        for g in edges {
                            let ge = &solids[other].edges[g as usize];
                            let EdgeCurve::Curve(gc) = &ge.curve else { continue };
                            // (where neither is a line or a circle — a spline sweep's rail along a
                            // side — it is left unsplit, as every edge along a face was before; that
                            // crossing wants a general curve–curve root, not yet written)
                            for (t,q) in crossings_in(&f.surface,c,e.t,gc,ge.t,tol).unwrap_or_default() {
                                if debug { eprintln!("brep: edge {i} of {} ({}) along face {fi} crosses its edge {g} at {q:?}",
                                    ["A","B"][s],c.kind()); }
                                let v = pool.at(q);
                                cuts[we].push((t,v));
                                on_face[other][fi].push(v);
                            }
                        }
                    }
                    Meets::At(roots) => for (t,touch) in roots {
                        let q = c.point(t);
                        if debug { eprintln!("brep: edge {i} of {} ({}) meets face {fi} ({}) at {q:?}{}: {:?}",["A","B"][s],c.kind(),
                            f.surface.kind(),if touch { ", touching" } else { "" },located[other].face_place(fi,q)); }
                        if located[other].face_place(fi,q) == Place::Out { continue }
                        let v = pool.at(q);
                        cuts[we].push((t,v));
                        on_face[other][fi].push(v);
                    },
                }
            }
        }
    }
    let t1 = clock.elapsed().as_secs_f64();
    // 2. every pair of faces: their surfaces' curves, kept where they lie in both faces
    let mut ssi_edges: Vec<(usize,usize,usize)> = Vec::new(); // (edge, face of A, face of B)
    // faces of the two on one surface: each splits the other with its boundary
    let mut same_pairs: Vec<(usize,usize)> = Vec::new();
    for fa in 0..a.faces.len() {
        for fb in 0..b.faces.len() {
            if !overlap(&boxes[0][fa],&boxes[1][fb],pad) { continue }
            let (sa,sb) = (&a.faces[fa].surface,&b.faces[fb].surface);
            let curves = match intersect(sa,sb,tol) {
                Ssi::Curves(cs) => cs,
                Ssi::Same => { same_pairs.push((fa,fb)); continue }
                Ssi::Traced => {
                    // traced through the points where either face's boundary crosses the other, and
                    // where a grid across a sheet's face crosses the other surface: a curve lying
                    // inside both faces, or ending on a sheet's boundary beyond them, crosses no edge
                    let mut seeds: Vec<V> = on_face[1][fb].iter().chain(&on_face[0][fa]).map(|&v| pool.pts[v as usize])
                        .filter(|&p| sa.implicit(p).abs() <= 8.*tol && sb.implicit(p).abs() <= 8.*tol).collect();
                    // **where the two touch along an edge one of them lies in** (a fillet's face against
                    // the face it rolls on), that edge is their meeting there, which no trace can
                    // follow (they are tangent): its points seed nothing, and any other meeting is
                    // traced from the rest
                    // (a point within `tol` of two surfaces touching may stand the square root of
                    // `tol` times their size off the curve they touch along)
                    let contact: Vec<usize> = along_face[1][fb].iter().chain(&along_face[0][fa]).copied().collect();
                    let spread = 8.*(tol*distance(boxes[0][fa].0,boxes[0][fa].1).max(distance(boxes[1][fb].0,boxes[1][fb].1))).sqrt();
                    // each edge they touch along, over its stretch
                    let stretches: Vec<(&super::geom::Curve,[f64;2])> = contact.iter().filter_map(|&we| match &edges[we].curve {
                        EdgeCurve::Curve(c) => Some((c,edges[we].t)),
                        EdgeCurve::Degenerate => None,
                    }).collect();
                    let shallow = |p: V| norm(crate::space::cross(sa.gradient(p),sb.gradient(p))) < super::ssi::SHALLOW;
                    let on_contact = |p: V| stretches.iter().any(|&(c,span)| {
                        let mut t = c.inverse(p);
                        if let Some(period) = c.period() { t = around(t,span[0],period); }
                        distance(c.point(t.clamp(span[0],span[1])),p) <= spread.max(8.*tol)
                    });
                    let touching = |p: V| shallow(p) && on_contact(p);
                    seeds.retain(|&p| !touching(p));
                    // **where the two touch at a point**: to second order their meeting there is the
                    // point alone, or two branches crossing at it (`ssi::touch`); where none of the
                    // branches runs into both faces the two meet there alone — nothing is traced
                    // through it. Read wherever a seed, a march or the search comes where the two
                    // nearly touch; a touch with a branch into both is left to the trace, which
                    // refuses it
                    let least = distance(boxes[0][fa].0,boxes[0][fa].1).min(distance(boxes[1][fb].0,boxes[1][fb].1));
                    // the touches read, as they alone are met there or not, each within its radius
                    let read: std::cell::RefCell<Vec<(super::ssi::Touch,bool)>> = std::cell::RefCell::new(Vec::new());
                    let alone = |p: V| -> bool {
                        let Some(at) = super::ssi::touch_near(sa,sb,p,tol) else { return false };
                        let Some(t) = super::ssi::touch(sa,sb,at) else { return false };
                        if t.radius > 0.05*least || distance(at,p) > t.radius { return false }
                        let place = |side: usize,fi: usize,d: V| {
                            let s = &solids[side].faces[fi].surface;
                            let q = crate::space::add(at,crate::space::scale(d,2.*t.radius));
                            located[side].face_place(fi,s.point(s.inverse(q)))
                        };
                        let lone = !t.branches.iter().any(|&d| place(0,fa,d) != Place::Out && place(1,fb,d) != Place::Out);
                        if debug && lone { eprintln!("brep: A{fa} {} × B{fb} {}: touch alone at {:?} ({} branches, radius {:.1e})",
                            sa.kind(),sb.kind(),t.at,t.branches.len(),t.radius); }
                        read.borrow_mut().push((t,lone));
                        lone
                    };
                    // beside a meeting already known: about a point they touch at, read alone, or
                    // where they touch beside an edge they touch along — the cheapest asked first
                    let beside = |p: V| {
                        let known = read.borrow().iter().find(|(t,_)| distance(t.at,p) <= t.radius).map(|&(_,lone)| lone);
                        known == Some(true) || shallow(p) && (on_contact(p) || known.is_none() && alone(p))
                    };
                    let (lo,hi): (V,V) = (std::array::from_fn(|k| boxes[0][fa].0[k].max(boxes[1][fb].0[k])),
                        std::array::from_fn(|k| boxes[0][fa].1[k].min(boxes[1][fb].1[k])));
                    let started = crate::clock::Instant::now();
                    let grid = if matches!(sb,super::geom::Surface::BSpline(..)) { grid_seeds(b,fb,sa,tol) }
                        else if matches!(sa,super::geom::Surface::BSpline(..)) { grid_seeds(a,fa,sb,tol) } else { Vec::new() };
                    t_grid += started.elapsed().as_secs_f64();
                    // (only where both faces may be: a seed beyond their shared box traces nothing on them)
                    seeds.extend(grid.into_iter().filter(|p| (0..3).all(|k| p[k] >= lo[k]-pad && p[k] <= hi[k]+pad))
                        .filter(|&p| !touching(p)));
                    if debug { eprintln!("brep: A{fa} {} × B{fb} {}: tracing from {} seed(s) {seeds:?}",sa.kind(),sb.kind(),seeds.len()); }
                    let started = crate::clock::Instant::now();
                    let mut traced = super::ssi::trace_beside(sa,sb,&seeds,&beside,lo,hi,tol)?;
                    // a closed curve inside both faces crosses no edge and has no seed: between
                    // analytic surfaces the smaller face is searched for what no curve yet passes,
                    // each found traced in turn, until the search shows nothing else is there
                    if super::ssi::searchable(sa) && super::ssi::searchable(sb) {
                        let size = |bx: &(V,V)| distance(bx.0,bx.1);
                        let (sol,fi,mine,theirs) = if size(&boxes[1][fb]) < size(&boxes[0][fa]) { (b,fb,sb,sa) } else { (a,fa,sa,sb) };
                        let domain = parameters(sol,fi,true);
                        // more separate loops than any face pair this kernel builds meets in
                        const LOOPS: usize = 64;
                        let mut found = 0;
                        // the edges they touch along are meetings already known, over their own stretch
                        while let Some(p) = super::ssi::unseen_beside(mine,domain,theirs,&traced,&stretches,&beside,lo,hi,tol)? {
                            if debug { eprintln!("brep: A{fa} {} × B{fb} {}: a meeting no edge crosses, at {p:?}",sa.kind(),sb.kind()); }
                            found += 1;
                            if found > LOOPS {
                                return Err(format!("a {} and a {} meet in more than {LOOPS} curves no edge crosses, not built yet",
                                    sa.kind(),sb.kind()))
                            }
                            let more = super::ssi::trace_beside(sa,sb,&[p],&beside,lo,hi,tol)?;
                            if more.is_empty() {
                                return Err(format!("a {} and a {} meet at {p:?}, where no edge crosses, in a curve that could not be \
                                    traced, not built yet",sa.kind(),sb.kind()))
                            }
                            traced.extend(more);
                        }
                    }
                    let took = started.elapsed().as_secs_f64();
                    t_trace += took;
                    if took > 0.2 && std::env::var_os("SOLVENT_BREP_TIME").is_some() {
                        eprintln!("time: A{fa} {} × B{fb} {}: traced {} curves from {} seeds in {took:.2} s",sa.kind(),sb.kind(),traced.len(),seeds.len());
                    }
                    if debug { for c in &traced {
                        if let super::geom::Curve::Traced(t) = c {
                            eprintln!("brep:   traced {} points, closed {}, from {:?} to {:?}; seeds off it by {:?}",t.pts.len(),t.closed,
                                t.pts[0],t.pts[t.pts.len()-1],seeds.iter().map(|&p| distance(c.point(c.inverse(p)),p)).collect::<Vec<_>>());
                        }
                    } }
                    traced
                }
            };
            let reach = [boxes[0][fa].0,boxes[0][fa].1,boxes[1][fb].0,boxes[1][fb].1];
            if debug && !curves.is_empty() {
                eprintln!("brep: A{fa} {} × B{fb} {}: {} curve(s)",sa.kind(),sb.kind(),curves.len());
            }
            for c in curves {
                // a curve that comes nowhere near both faces (a far crossing of two nearly parallel
                // meridians) leaves nothing on them
                let (lo,hi): (V,V) = (std::array::from_fn(|k| reach[0][k].max(reach[2][k])-pad),
                    std::array::from_fn(|k| reach[1][k].min(reach[3][k])+pad));
                let far = match c {
                    // a line misses the box both faces share: clipped by its slabs to nothing
                    super::geom::Curve::Line {p,d} => {
                        let (mut t0,mut t1) = (f64::NEG_INFINITY,f64::INFINITY);
                        for k in 0..3 {
                            if d[k].abs() < 1e-300 { if p[k] < lo[k] || p[k] > hi[k] { t0 = 1.; t1 = 0.; } continue }
                            let (a,z) = ((lo[k]-p[k])/d[k],(hi[k]-p[k])/d[k]);
                            t0 = t0.max(a.min(z)); t1 = t1.min(a.max(z));
                        }
                        t0 > t1
                    }
                    // a closed curve far larger than the faces (a far crossing of nearly parallel meridians)
                    _ => c.speed() > 1e3*(1.+norm(crate::space::sub(hi,lo)))
                        && !(0..256).any(|j| { let q = c.point(TAU*j as f64/256.); (0..3).all(|k| q[k] >= lo[k] && q[k] <= hi[k]) }),
                };
                if far { continue }
                let mut ts: Vec<(f64,u32)> = Vec::new();
                for &v in on_face[1][fb].iter().chain(&on_face[0][fa]) {
                    let p = pool.pts[v as usize];
                    let t = c.inverse(p);
                    // a point within the tolerance of both surfaces is that over the sine of the angle
                    // they meet at off their meeting
                    let sine = norm(crate::space::cross(sa.gradient(p),sb.gradient(p))).max(1e-4);
                    if distance(c.point(t),p) <= 8.*tol/sine && !ts.iter().any(|&(_,w)| w == v) { ts.push((t,v)); }
                }
                let closed = c.period().is_some();
                let period = c.period().unwrap_or(TAU);
                let mut spans: Vec<([f64;2],[u32;2])> = Vec::new();
                if closed {
                    for t in &mut ts { t.0 = around(t.0,0.,period); }
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
                } else {
                    ts.sort_by(|x,y| x.0.total_cmp(&y.0));
                    for w in ts.windows(2) { spans.push(([w[0].0,w[1].0],[w[0].1,w[1].1])); }
                }
                if debug { eprintln!("brep:   {} with {} point(s) on it: {:?}",c.kind(),ts.len(),ts.iter().map(|x| pool.pts[x.1 as usize]).collect::<Vec<_>>()); }
                for (t,v) in spans {
                    if (t[1]-t[0])*c.speed() <= tol || (v[0] == v[1] && !closed) { continue }
                    let m = c.point((t[0]+t[1])/2.);
                    let places = (located[0].face_place(fa,m),located[1].face_place(fb,m));
                    if debug { eprintln!("brep:     span {t:?} from {:?} to {:?}: middle {m:?} {places:?}",pool.pts[v[0] as usize],pool.pts[v[1] as usize]); }
                    if places.0 == Place::Out || places.1 == Place::Out { continue }
                    // where the two only touch (their normals parallel) nothing crosses from one
                    // side to the other: no face is split there
                    if norm(crate::space::cross(sa.gradient(m),sb.gradient(m))) < 1e-7 { continue }
                    edges.push(WEdge {curve:EdgeCurve::Curve(c.clone()),t,v});
                    cuts.push(Vec::new());
                    ssi_edges.push((edges.len()-1,fa,fb));
                }
            }
        }
    }
    // 2b. every vertex, either solid's or a crossing's, cuts an edge it lies inside, the faces'
    // meetings among them: two edges running along one another (a shared circle, each solid
    // seamed at its own angle; a meeting traced along an edge already there) then share their
    // vertices, and step 4 takes them for one. Each edge boxed by its curve's enclosure, on every core.
    let inside = crate::par::indices(edges.len(),|we| -> Vec<(f64,u32)> {
        let e = &edges[we];
        let EdgeCurve::Curve(c) = &e.curve else { return Vec::new() };
        let (lo,hi) = c.bounds(e.t);
        (0..pool.pts.len() as u32).filter_map(|v| {
            if v == e.v[0] || v == e.v[1] || cuts[we].iter().any(|&(_,w)| w == v) { return None }
            let p = pool.pts[v as usize];
            if (0..3).any(|k| p[k] < lo[k]-pad || p[k] > hi[k]+pad) { return None }
            let mut t = c.inverse(p);
            if let Some(period) = c.period() { t = around(t,e.t[0],period); }
            // a vertex at the edge's end in all but the pool's labelling is its end
            let near = |q: V| distance(q,p) <= 8.*tol;
            (t > e.t[0] && t < e.t[1] && near(c.point(t)) && !near(c.point(e.t[0])) && !near(c.point(e.t[1]))).then_some((t,v))
        }).collect()
    });
    for (we,found) in inside.into_iter().enumerate() { cuts[we].extend(found); }
    let t2 = clock.elapsed().as_secs_f64();
    // 3. every original edge split where it was cut: its pieces, in order along it
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
    // 4. pieces that are one edge merged: the same ends and the same curve between
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
    // 5. each face split by what lies on it, traced in its parameters
    let mut pieces: Vec<(usize,usize,Vec<Vec<Half>>)> = Vec::new();
    for s in 0..2 {
        for (fi,f) in solids[s].faces.iter().enumerate() {
            let loc = &located[s];
            let mut halves: Vec<Half> = Vec::new();
            let mut on: Vec<u32> = Vec::new();
            for l in &f.loops { for c in l {
                let we = first_edge[s]+c.edge as usize;
                let orig = &solids[s].edges[c.edge as usize];
                let mut ps = pieces_of[we].clone();
                if c.reversed { ps.reverse(); }
                for (piece,[ta,tb]) in ps {
                    let (r,flip) = rep[piece as usize];
                    let at = |t: f64| c.pcurve.at(t,orig,&f.surface,&solids[s].vertices);
                    let (ua,ub) = (at(ta),at(tb));
                    let iso = match (&out[r as usize].curve,r == piece) {
                        (EdgeCurve::Curve(cr),false) => { let w = &out[r as usize]; iso_pcurve(&f.surface,cr,w.t) }
                        _ => None,
                    };
                    let pcurve = if r == piece { match c.pcurve {
                        Pcurve::Line {..} => Pcurve::Line {a:ua,b:ub},
                        Pcurve::Inverse {..} => Pcurve::Inverse {a:ua,b:ub},
                        // keyed to the curve's own parameter, which a piece of the edge keeps
                        Pcurve::Curve(ref c) => Pcurve::Curve(c.clone()),
                    } } else if let Some((a,b)) = iso { Pcurve::Line {a,b} }
                    else if flip { Pcurve::Inverse {a:ub,b:ua} } else { Pcurve::Inverse {a:ua,b:ub} };
                    let along = !c.reversed != flip;
                    let (from,to) = if c.reversed { (ub,ua) } else { (ua,ub) };
                    halves.push(Half {edge:r,along,pcurve,from,to});
                    on.push(r);
                }
            } }
            // an edge the other solid leaves across this face: both ways round, its parameters
            // walked from its middle out, each step unwrapped from the last
            let internal = |r: u32,halves: &mut Vec<Half>,on: &mut Vec<u32>| {
                if on.contains(&r) { return }
                on.push(r);
                let w = &out[r as usize];
                let EdgeCurve::Curve(c) = &w.curve else { return };
                let n = 32;
                let mid = (w.t[0]+w.t[1])/2.;
                let start = loc.uv(fi,c.point(mid));
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
                halves.push(Half {edge:r,along:true,pcurve:pcurve.clone(),from:ua,to:ub});
                halves.push(Half {edge:r,along:false,pcurve,from:ub,to:ua});
            };
            for &(e,x,y) in &ssi_edges {
                if (if s == 0 { x } else { y }) != fi { continue }
                for &(piece,_) in &pieces_of[e] { internal(rep[piece as usize].0,&mut halves,&mut on); }
            }
            // a face of the other on this one's surface: its boundary's pieces inside this face
            for &(x,y) in &same_pairs {
                let (mine,theirs) = if s == 0 { (x,y) } else { (y,x) };
                if mine != fi { continue }
                let other = 1-s;
                for l in &solids[other].faces[theirs].loops { for c in l {
                    for &(piece,_) in &pieces_of[first_edge[other]+c.edge as usize] {
                        let r = rep[piece as usize].0;
                        let w = &out[r as usize];
                        if matches!(w.curve,EdgeCurve::Degenerate) { continue }
                        if loc.face_place(fi,w.point((w.t[0]+w.t[1])/2.,&pool)) == Place::In { internal(r,&mut halves,&mut on); }
                    }
                } }
            }
            if std::env::var_os("SOLVENT_BREP_DEBUG").is_some() && s == 1 {
                // the cut network on this face: each vertex's count of cut edges (an odd one is an open end)
                let mut degree: BTreeMap<u32,usize> = BTreeMap::new();
                let cut: Vec<u32> = ssi_edges.iter().filter(|e| e.2 == fi).flat_map(|&(e,_,_)| pieces_of[e].iter().map(|&(p,_)| rep[p as usize].0)).collect();
                for &e in &cut { for v in out[e as usize].v { *degree.entry(v).or_insert(0) += 1; } }
                let open: Vec<V> = degree.iter().filter(|(_,&d)| d % 2 == 1).map(|(&v,_)| pool.pts[v as usize]).collect();
                eprintln!("arrange: face {fi} of B ({}): {} cut edges, open ends {:?}",f.surface.kind(),cut.len(),open);
            }
            for piece in pieces_of_face(f,&halves,&out,&pool)? { pieces.push((s,fi,piece)); }
        }
    }
    if std::env::var_os("SOLVENT_BREP_TIME").is_some() {
        eprintln!("time: arrange: edges against faces {t1:.2} s; face pairs {:.2} s (grid seeds {t_grid:.2}, traces {t_trace:.2}); \
            the rest {:.2} s",t2-t1,clock.elapsed().as_secs_f64()-t2);
    }
    Ok(Arranged {solids,located,pool,out,pieces})
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
            Pcurve::Curve(ref c) => { let p = c.point(t); [p[0],p[1]] }
            Pcurve::Inverse {..} => unwrap(f.surface.inverse_near(e.point(t,pool),near),near,f.surface.periods()),
        }
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
    let node_of = |v: u32,uv: Uv,nodes: &mut Vec<(u32,Uv)>| -> usize {
        if let Some(i) = nodes.iter().position(|&(w,p)| w == v && (p[0]-uv[0]).abs() < 1e-3 && (p[1]-uv[1]).abs() < 1e-3) { return i }
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
                if std::env::var_os("SOLVENT_BREP_DEBUG").is_some() {
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
    let polys: Vec<Vec<Uv>> = cycles.iter().map(|c| c.iter().flat_map(|&h| {
        let pts = samples(&halves[h],f,out,pool,24);
        pts[..pts.len()-1].iter().map(|&p| oriented(f,p)).collect::<Vec<_>>()
    }).collect()).collect();
    let area = |p: &[Uv]| (0..p.len()).map(|i| { let (a,b) = (p[i],p[(i+1)%p.len()]); a[0]*b[1]-a[1]*b[0] }).sum::<f64>()/2.;
    let areas: Vec<f64> = polys.iter().map(|p| area(p)).collect();
    let mut pieces: Vec<(usize,Vec<usize>)> = (0..cycles.len()).filter(|&i| areas[i] > 0.).map(|i| (i,vec![i])).collect();
    for i in (0..cycles.len()).filter(|&i| areas[i] <= 0.) {
        // a hole goes in the smallest outer cycle about the ground just to its left, where its
        // material is (a point on the hole itself may be on its twin, a piece it bounds).  An outer
        // cycle sharing one of its edges is on that edge's other side, and is never its host: the
        // probe is a hair off the hole, nearer it than the two walks' samples of one edge agree.
        let p = &polys[i];
        let j = (0..p.len()).max_by(|&a,&b| {
            let l = |k: usize| { let (x,y) = (p[k],p[(k+1)%p.len()]); (y[0]-x[0]).dhypot(y[1]-x[1]) };
            l(a).total_cmp(&l(b))
        }).unwrap_or(0);
        let (p0,p1) = (p[j],p[(j+1)%p.len()]);
        let d = [p1[0]-p0[0],p1[1]-p0[1]];
        let len = d[0].dhypot(d[1]).max(1e-300);
        let eps = 1e-6*len;
        let probe = [(p0[0]+p1[0])/2.-eps*d[1]/len,(p0[1]+p1[1])/2.+eps*d[0]/len];
        let edges: Vec<u32> = cycles[i].iter().map(|&h| halves[h].edge).collect();
        let mut host: Option<usize> = None;
        for (k,(o,_)) in pieces.iter().enumerate() {
            if cycles[*o].iter().any(|&h| edges.contains(&halves[h].edge)) { continue }
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

/// Points well inside a piece, in its face's parameters, the best first: the middles of the spans
/// level lines across it have inside it, widest first.
fn inside(piece: &[Vec<Half>],f: &Face,out: &[WEdge],pool: &Pool) -> Result<Vec<Uv>,String> {
    let polys: Vec<Vec<Uv>> = piece.iter().map(|cycle| cycle.iter().flat_map(|h| {
        let pts = samples(h,f,out,pool,24);
        pts[..pts.len()-1].to_vec()
    }).collect()).collect();
    let (mut vlo,mut vhi) = (f64::INFINITY,f64::NEG_INFINITY);
    for p in polys.iter().flatten() { vlo = vlo.min(p[1]); vhi = vhi.max(p[1]); }
    let mut found: Vec<(f64,Uv)> = Vec::new();
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
            if w.len() == 2 && w[1] > w[0] { found.push((w[1]-w[0],[(w[0]+w[1])/2.,v])); }
        }
    }
    if found.is_empty() { return Err(format!("a piece of a {} has no inside",f.surface.kind())) }
    found.sort_by(|a,b| b.0.total_cmp(&a.0));
    Ok(found.into_iter().map(|f| f.1).collect())
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
            for l in &mut f.loops { *l = l.iter().rev().map(|c| Coedge {reversed:!c.reversed,..c.clone()}).collect(); }
        }
    }
    let sheets = &sheets;
    let clock = crate::clock::Instant::now();
    let Arranged {solids,located,pool,out,pieces,..} = arrange(solid,sheets,tol)?;
    let arranged = clock.elapsed().as_secs_f64();
    let face_of = |i: usize,flip: bool| -> Face {
        let (s,fi,piece) = &pieces[i];
        let f = &solids[*s].faces[*fi];
        let loops: Vec<Vec<Coedge>> = piece.iter().map(|cycle| {
            let mut l: Vec<Coedge> = cycle.iter().map(|h| Coedge {edge:h.edge,reversed:!h.along,pcurve:h.pcurve.clone()}).collect();
            if flip { l = l.into_iter().rev().map(|c| Coedge {reversed:!c.reversed,..c}).collect(); }
            l
        }).collect();
        Face {surface:f.surface.clone(),reversed:f.reversed != flip,loops,name:f.name.clone()}
    };
    // the pieces kept: all the solid's, and the sheets' inside it
    let mut kept: Vec<usize> = Vec::new();
    for i in 0..pieces.len() {
        let (s,fi,piece) = &pieces[i];
        if *s == 0 { kept.push(i); continue }
        let f = &solids[1].faces[*fi];
        let candidates = inside(piece,f,&out,&pool)?;
        let place = candidates.iter().map(|&uv| located[0].solid_place(f.surface.point(uv))).find(|&p| p != Place::On).unwrap_or(Place::On);
        if std::env::var_os("SOLVENT_BREP_DEBUG").is_some() {
            let all: Vec<Place> = candidates.iter().map(|&uv| located[0].solid_place(f.surface.point(uv))).collect();
            eprintln!("split: a piece of the sheet's {}: {place:?} at {:?} ({} loops); at its {} inner points {} in, {} out, {} on",f.surface.kind(),
                f.surface.point(candidates[0]),piece.len(),all.len(),all.iter().filter(|&&p| p == Place::In).count(),
                all.iter().filter(|&&p| p == Place::Out).count(),all.iter().filter(|&&p| p == Place::On).count());
        }
        if place == Place::In { kept.push(i); }
    }
    if std::env::var_os("SOLVENT_BREP_TIME").is_some() {
        eprintln!("time: split: arranged {arranged:.2} s, the sheet's pieces placed {:.2} s",clock.elapsed().as_secs_f64()-arranged);
    }
    // a side of a piece: 2i as it is, 2i + 1 turned round; the cells are classes of sides
    let mut parent: Vec<usize> = (0..2*pieces.len()).collect();
    fn root(p: &mut [usize],mut x: usize) -> usize { while p[x] != x { p[x] = p[p[x]]; x = p[x]; } x }
    let mut at_edge: BTreeMap<u32,Vec<(usize,usize)>> = BTreeMap::new();
    for &i in &kept { for (k,h) in pieces[i].2.iter().flatten().enumerate() { at_edge.entry(h.edge).or_default().push((i,k)); } }
    if std::env::var_os("SOLVENT_BREP_DEBUG").is_some() {
        for (&e,uses) in &at_edge { if uses.len() == 1 {
            let (i,_) = uses[0];
            eprintln!("split: edge {e} meets only a piece of a {} (solid {}), at {:?}",solids[pieces[i].0].faces[pieces[i].1].surface.kind(),pieces[i].0,
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
            if std::env::var_os("SOLVENT_BREP_DEBUG").is_some() && pieces[i].0 == 0 && pieces[j].0 == 0 && (side_i%2) != (side_j%2) {
                eprintln!("split: at edge {e} ({} uses, at {m:?}) a {} piece's {} meets a {} piece's {}",around.len(),
                    solids[0].faces[pieces[i].1].surface.kind(),["inside","outside"][side_i%2],solids[0].faces[pieces[j].1].surface.kind(),["inside","outside"][side_j%2]);
            }
            let (x,y) = (root(&mut parent,side_i),root(&mut parent,side_j));
            parent[x] = y;
        }
    }
    // the classes of sides: a cell, unless it holds a solid face turned round (the outside)
    let outside: std::collections::BTreeSet<usize> = kept.iter().filter(|&&i| pieces[i].0 == 0).map(|&i| root(&mut parent,2*i+1)).collect();
    let mut classes: BTreeMap<usize,Vec<usize>> = BTreeMap::new();
    for &i in &kept { for side in [2*i,2*i+1] {
        if pieces[i].0 == 0 && side == 2*i+1 { continue }
        let r = root(&mut parent,side);
        if !outside.contains(&r) { classes.entry(r).or_default().push(side); }
    } }
    if std::env::var_os("SOLVENT_BREP_DEBUG").is_some() {
        let mut all: BTreeMap<usize,(usize,usize,bool)> = BTreeMap::new();
        for &i in &kept { for side in [2*i,2*i+1] {
            let r = root(&mut parent,side);
            let e = all.entry(r).or_insert((0,0,false));
            if pieces[i].0 == 0 { e.0 += 1 } else { e.1 += 1 }
            if pieces[i].0 == 0 && side == 2*i+1 { e.2 = true; }
        } }
        eprintln!("split: {} solid pieces, {} sheet pieces kept; classes (solid sides, sheet sides, outside): {:?}",
            kept.iter().filter(|&&i| pieces[i].0 == 0).count(),kept.iter().filter(|&&i| pieces[i].0 == 1).count(),all.values().collect::<Vec<_>>());
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
