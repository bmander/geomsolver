//! A drafted prism of any loop (#607, #588): its section at depth `w` is its profile moved
//! `|w| tan draft` into its material, each side along its own normal — a line square to itself, an
//! arc's radius less or more, each corner where its moved sides meet (`build::drafted`'s
//! `offset`, the same section the exact kernel lofts). A convex loop of lines and whole circles is
//! read exactly by its profile's field moved (`PlanarField::draftable`); every other loop is read
//! here, at a point, as the signed distance to its section at the point's depth, divided by the
//! most the section's boundary can move per unit of depth together with a unit across it — so the
//! field stays one-Lipschitz, its zero the drafted sides themselves.
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::solid::surface::Edge;
use std::f64::consts::{PI,TAU};

use crate::brep::plane2::{add,cross,dot,left,meet,norm as length,scale,sub,unit,Carrier,Touch};

type P = [f64;2];

/// The loops of a drafted prism's profile, each walked with its material on its left (the outer
/// counter-clockwise, a hole clockwise), and how its sections are read.
#[derive(Clone,Debug)]
pub(crate) struct Sections {
    loops: Vec<Vec<Edge>>,
    /// Each loop's corners as the move takes them, the one where step `k` starts at `k`: where both
    /// sides are lines (or go on tangent), its point and the way it moves per unit of offset;
    /// otherwise solved where the moved sides meet (`meet`).
    corners: Vec<Vec<Option<(P,P)>>>,
    /// The draft's tangent: positive moves every side into the material as the depth grows.
    taper: f64,
    /// The most the field changes per unit length in space: the section's boundary moving at most
    /// `mitre` per unit of offset, an offset `taper` per unit of depth.
    lipschitz: f64,
    /// How far any section within the prism's depths reaches from the profile's origin.
    reach: f64,
}

/// A step's unit tangent, the way it is walked, at fraction `t` of it.
fn tangent(e: &Edge,t: f64) -> P { unit(e.tangent(t)) }

/// The root of two carriers' meeting nearest `near`.
fn nearest(a: Carrier,b: Carrier,near: P) -> Option<P> {
    meet(a,b,Touch::Exact)?.into_iter().min_by(|x,y| length(sub(*x,near)).total_cmp(&length(sub(*y,near))))
}

/// A profile's loops (the first its outer loop, the rest holes, in any order and direction, each
/// closed) each walked with its material on its left: the outer counter-clockwise, a hole
/// clockwise.
pub(crate) fn oriented(loops: &[Vec<Edge>]) -> Result<Vec<Vec<Edge>>,String> {
    loops.iter().enumerate().map(|(k,l)| {
        let edges = super::profile::ordered(l,0.)?;
        let area: f64 = edges.iter().map(Edge::area_twice).sum();
        if !(area.abs() > 0.) { return Err("a profile's loop encloses nothing".to_string()) }
        let ccw = k == 0;
        Ok(if (area > 0.) == ccw { edges } else { edges.into_iter().rev().map(Edge::reversed).collect() })
    }).collect()
}

impl Sections {
    /// The loops of a profile (the first its outer loop, the rest holes, in any order and
    /// direction, each closed) drafted by `angle` (radians) over depths `range`: refused where an
    /// edge is no line or arc, or a section within the range loses an edge or a corner.
    pub(crate) fn new(loops: &[Vec<Edge>],angle: f64,range: [f64;2]) -> Result<Sections,String> {
        let taper = angle.dtan();
        if loops.iter().flatten().any(|e| matches!(e,Edge::Conic {..})) {
            return Err("the field reads a drafted prism whose sides are lines and arcs".into())
        }
        let out = oriented(loops)?;
        let deepest = range[0].abs().max(range[1].abs());
        let corners = out.iter().map(|l| {
            let n = l.len();
            (0..n).map(|k| {
                let (a,b) = (&l[(k+n-1)%n],&l[k]);
                let c = b.endpoints()[0];
                let (ta,tb) = (tangent(a,1.),tangent(b,0.));
                if cross(ta,tb).abs() <= 1e-9 && dot(ta,tb) > 0. { return Some((c,left(tb))) }
                let (Edge::Line {..},Edge::Line {..}) = (a,b) else { return None };
                // the moved lines cross a unit of offset along this from the corner
                nearest(Carrier::Line {p:left(ta),d:ta},Carrier::Line {p:left(tb),d:tb},add(left(ta),left(tb))).map(|m| (c,m))
            }).collect()
        }).collect();
        let mut s = Sections {loops:out,corners,taper,lipschitz:1.,reach:0.};
        let far = deepest*taper;
        // every section within the depths keeps its edges, so the farthest does
        let moved = s.section(far).map_err(|e| format!("the field reads a drafted prism's section at its far end: {e}"))?;
        let mut mitre: f64 = 1.;
        let mut reach: f64 = 0.;
        for (l,m) in s.loops.iter().zip(&moved) {
            for (e,f) in l.iter().zip(m) {
                if far != 0. { mitre = mitre.max(length(sub(f.endpoints()[0],e.endpoints()[0]))/far.abs()); }
                reach = reach.max(e.reach()).max(f.reach());
            }
        }
        // a corner between arcs moves along a curve; read at its far end it is a little slow
        s.lipschitz = 1_f64.dhypot(1.05*mitre*taper);
        s.reach = reach*1.01+far.abs()*mitre;
        Ok(s)
    }

    /// Each loop moved `d` into its material: every side along its own normal, each corner where
    /// its moved sides meet, refused where a side the move takes to nothing or turns round.
    pub(crate) fn section(&self,d: f64) -> Result<Vec<Vec<Edge>>,String> {
        (0..self.loops.len()).map(|l| (0..self.loops[l].len()).map(|k| self.moved(l,k,d)).collect()).collect()
    }

    /// Where loop `l`'s corner `k` (where its step `k` starts) is moved `d` into the material.
    fn corner(&self,l: usize,k: usize,d: f64) -> Result<P,String> {
        if let Some((c,m)) = self.corners[l][k] { return Ok(add(c,scale(m,d))) }
        let edges = &self.loops[l];
        let n = edges.len();
        let (a,b) = (&edges[(k+n-1)%n],&edges[k]);
        let c = b.endpoints()[0];
        let carrier = |e: &Edge| match *e {
            Edge::Line {a,b,..} => { let t = unit(sub(b,a)); Carrier::Line {p:add(a,scale(left(t),d)),d:t} }
            // the material is left of travel: toward the centre counter-clockwise
            Edge::Arc {center,radius,sweep,..} => Carrier::Circle {c:center,r:radius-d*sweep.signum()},
            Edge::Conic {..} => unreachable!("refused when read"),
        };
        let near = add(c,scale(add(left(tangent(a,1.)),left(tangent(b,0.))),d/2.));
        nearest(carrier(a),carrier(b),near).ok_or_else(|| "a draft parts two sides that meet at a corner".to_string())
    }

    /// Loop `l`'s step `k` moved `d` into the material, between its moved corners.
    fn moved(&self,l: usize,k: usize,d: f64) -> Result<Edge,String> {
        if d == 0. { return Ok(self.loops[l][k].clone()) }
        let n = self.loops[l].len();
        self.between(l,k,d,self.corner(l,k,d)?,self.corner(l,(k+1)%n,d)?)
    }

    /// Loop `l`'s step `k` moved `d` into the material, from its moved corner `p` to `q`.
    fn between(&self,l: usize,k: usize,d: f64,p: P,q: P) -> Result<Edge,String> {
        let edges = &self.loops[l];
        let e = &edges[k];
        if let [Edge::Arc {center,radius,start,sweep,..}] = edges[..] {
            let r = radius-d*sweep.signum();
            if !(r > 0.) { return Err("a draft takes a circle to nothing".into()) }
            let p = [center[0]+r*start.dcos(),center[1]+r*start.dsin()];
            return Ok(Edge::Arc {center,radius:r,start,sweep,ends:[p,p]})
        }
        match *e {
            Edge::Line {a,b,..} => {
                if (q[0]-p[0])*(b[0]-a[0])+(q[1]-p[1])*(b[1]-a[1]) <= 0. { return Err("a draft takes a side to nothing".into()) }
                Ok(Edge::Line {a:p,b:q,axis:false})
            }
            Edge::Arc {center,radius,start,sweep,..} => {
                let r = radius-d*sweep.signum();
                if !(r > 0.) { return Err("a draft takes an arc to nothing".into()) }
                let angle = |x: P| (x[1]-center[1]).datan2(x[0]-center[0]);
                // the moved arc's ends about the original's, its sweep the same way round
                let s0 = start+((angle(p)-start+PI).rem_euclid(TAU)-PI);
                let end = start+sweep;
                let s1 = end+((angle(q)-end+PI).rem_euclid(TAU)-PI);
                let turn = s1-s0;
                if turn*sweep <= 0. { return Err("a draft takes an arc to nothing".into()) }
                Ok(Edge::Arc {center,radius:r,start:s0,sweep:turn,ends:[p,q]})
            }
            Edge::Conic {..} => unreachable!("refused when read"),
        }
    }

    /// The field at a point of the profile's plane `p` and depth `w`: the signed distance to the
    /// section there (negative inside), over the Lipschitz bound; and the edge nearest, counted
    /// across the loops in order.
    pub(crate) fn value_piece(&self,p: P,w: f64) -> (f64,usize) {
        let d = w.abs()*self.taper;
        let (mut best,mut piece,mut crossings,mut i) = (f64::INFINITY,0,0,0);
        for l in 0..self.loops.len() {
            let n = self.loops[l].len();
            // each corner moved once, carried from the step it ends to the one it starts
            let first = self.corner(l,0,d);
            let mut at = first.clone();
            for k in 0..n {
                let next = if k+1 < n { self.corner(l,k+1,d) } else { first.clone() };
                // refused at construction within the prism's depths; beyond them, the profile
                let e = match (d == 0.,&at,&next) {
                    (false,Ok(a),Ok(b)) => self.between(l,k,d,*a,*b).ok(),
                    _ => None,
                }.unwrap_or_else(|| self.loops[l][k].clone());
                at = next;
                let x = e.distance(p);
                if x < best { best = x; piece = i; }
                crossings += e.crossings(p);
                i += 1;
            }
        }
        (if crossings%2 == 1 { -best } else { best }/self.lipschitz,piece)
    }

    pub(crate) fn value(&self,p: P,w: f64) -> f64 { self.value_piece(p,w).0 }

    /// Edge `piece`'s whole line or circle at depth `w`, negative on its material side.
    pub(crate) fn carrier(&self,p: P,w: f64,piece: usize) -> Option<f64> {
        let d = w.abs()*self.taper;
        let (mut l,mut k) = (0,piece);
        while k >= self.loops.get(l)?.len() { k -= self.loops[l].len(); l += 1; }
        let e = self.moved(l,k,d).unwrap_or_else(|_| self.loops[l][k].clone());
        let v = match e {
            Edge::Line {a,b,..} => -cross(unit(sub(b,a)),sub(p,a)),
            Edge::Arc {center,radius,sweep,..} => (length(sub(p,center))-radius)*sweep.signum(),
            Edge::Conic {..} => return None,
        };
        Some(v/self.lipschitz)
    }

    /// The section at depth `w`: each loop moved `|w| tan` into its material.
    pub(crate) fn section_at(&self,w: f64) -> Result<Vec<Vec<Edge>>,String> { self.section(w.abs()*self.taper) }

    /// How many edges the loops have.
    pub(crate) fn pieces(&self) -> usize { self.loops.iter().map(Vec::len).sum() }

    /// A radius about the profile's origin within which every section lies.
    pub(crate) fn reach(&self) -> f64 { self.reach }
}
