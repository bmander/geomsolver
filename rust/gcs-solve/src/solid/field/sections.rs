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

type P = [f64;2];
fn cross(a: P,b: P) -> f64 { a[0]*b[1]-a[1]*b[0] }
fn sub(a: P,b: P) -> P { [a[0]-b[0],a[1]-b[1]] }
fn add(a: P,b: P) -> P { [a[0]+b[0],a[1]+b[1]] }
fn scale(a: P,k: f64) -> P { [a[0]*k,a[1]*k] }
fn length(a: P) -> f64 { a[0].dhypot(a[1]) }
fn unit(a: P) -> P { let l = length(a); if l > 0. { scale(a,1./l) } else { a } }
fn left(a: P) -> P { [-a[1],a[0]] }

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

/// One step of a loop: a line or an arc, read for its unit tangent at either end.
fn tangent(e: &Edge,t: f64) -> P {
    match *e {
        Edge::Line {a,b,..} => unit(sub(b,a)),
        Edge::Arc {start,sweep,..} => {
            let (s,c) = (start+t*sweep).dsin_cos();
            [-s*sweep.signum(),c*sweep.signum()]
        }
        Edge::Conic {..} => unreachable!("refused when read"),
    }
}

fn start(e: &Edge) -> P {
    match *e {
        Edge::Line {a,..} => a,
        Edge::Arc {sweep,ends,..} if sweep.abs() < TAU => ends[0],
        _ => e.at(0.),
    }
}

/// Where two moved carriers meet nearest `near`: a line by a point and its direction, or a circle
/// by its centre and radius.
#[derive(Clone,Copy)]
enum Carrier { Line(P,P),Circle(P,f64) }

fn meet(a: Carrier,b: Carrier,near: P) -> Option<P> {
    let nearest = |xs: Vec<P>| xs.into_iter().min_by(|x,y| length(sub(*x,near)).total_cmp(&length(sub(*y,near))));
    match (a,b) {
        (Carrier::Line(p,d),Carrier::Line(q,e)) => {
            let w = cross(d,e);
            if w.abs() <= 1e-12 { return None }
            Some(add(p,scale(d,cross(sub(q,p),e)/w)))
        }
        (Carrier::Line(p,d),Carrier::Circle(c,r)) | (Carrier::Circle(c,r),Carrier::Line(p,d)) => {
            // |p + s d − c|² = r², d a unit
            let f = sub(p,c);
            let (b,k) = (2.*(f[0]*d[0]+f[1]*d[1]),f[0]*f[0]+f[1]*f[1]-r*r);
            let disc = b*b-4.*k;
            if disc < 0. { return None }
            let q = disc.sqrt();
            nearest(vec![add(p,scale(d,(-b-q)/2.)),add(p,scale(d,(-b+q)/2.))])
        }
        (Carrier::Circle(c0,r0),Carrier::Circle(c1,r1)) => {
            let d = sub(c1,c0);
            let l = length(d);
            if !(l > 0.) { return None }
            let a = (r0*r0-r1*r1+l*l)/(2.*l);
            let h2 = r0*r0-a*a;
            if h2 < 0. { return None }
            let (m,h) = (add(c0,scale(d,a/l)),h2.sqrt());
            let n = scale(left(d),h/l);
            nearest(vec![add(m,n),sub(m,n)])
        }
    }
}

/// A profile's loops (the first its outer loop, the rest holes, in any order and direction, each
/// closed) each walked with its material on its left: the outer counter-clockwise, a hole
/// clockwise.
pub(crate) fn oriented(loops: &[Vec<Edge>]) -> Result<Vec<Vec<Edge>>,String> {
    loops.iter().enumerate().map(|(k,l)| {
        let edges = super::profile::ordered_loop(l)?;
        let area: f64 = edges.iter().map(super::profile::area_twice).sum();
        if !(area.abs() > 0.) { return Err("a profile's loop encloses nothing".to_string()) }
        let ccw = k == 0;
        Ok(if (area > 0.) == ccw { edges } else { edges.into_iter().rev().map(super::profile::reversed).collect() })
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
                let c = start(b);
                let (ta,tb) = (tangent(a,1.),tangent(b,0.));
                if cross(ta,tb).abs() <= 1e-9 && ta[0]*tb[0]+ta[1]*tb[1] > 0. { return Some((c,left(tb))) }
                let (Edge::Line {..},Edge::Line {..}) = (a,b) else { return None };
                // the moved lines cross a unit of offset along this from the corner
                meet(Carrier::Line(left(ta),ta),Carrier::Line(left(tb),tb),add(left(ta),left(tb))).map(|m| (c,m))
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
                if far != 0. { mitre = mitre.max(length(sub(start(f),start(e)))/far.abs()); }
                for x in [e,f] {
                    reach = reach.max(match *x {
                        Edge::Line {a,b,..} => length(a).max(length(b)),
                        Edge::Arc {center,radius,..} => length(center)+radius,
                        Edge::Conic {..} => 0.,
                    });
                }
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
        let c = start(b);
        let carrier = |e: &Edge| match *e {
            Edge::Line {a,b,..} => { let t = unit(sub(b,a)); Carrier::Line(add(a,scale(left(t),d)),t) }
            // the material is left of travel: toward the centre counter-clockwise
            Edge::Arc {center,radius,sweep,..} => Carrier::Circle(center,radius-d*sweep.signum()),
            Edge::Conic {..} => unreachable!("refused when read"),
        };
        let near = add(c,scale(add(left(tangent(a,1.)),left(tangent(b,0.))),d/2.));
        meet(carrier(a),carrier(b),near).ok_or_else(|| "a draft parts two sides that meet at a corner".to_string())
    }

    /// Loop `l`'s step `k` moved `d` into the material, between its moved corners.
    fn moved(&self,l: usize,k: usize,d: f64) -> Result<Edge,String> {
        let edges = &self.loops[l];
        let n = edges.len();
        let e = &edges[k];
        if d == 0. { return Ok(e.clone()) }
        if let [Edge::Arc {center,radius,start,sweep,..}] = edges[..] {
            let r = radius-d*sweep.signum();
            if !(r > 0.) { return Err("a draft takes a circle to nothing".into()) }
            let p = [center[0]+r*start.dcos(),center[1]+r*start.dsin()];
            return Ok(Edge::Arc {center,radius:r,start,sweep,ends:[p,p]})
        }
        let (p,q) = (self.corner(l,k,d)?,self.corner(l,(k+1)%n,d)?);
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
            for k in 0..self.loops[l].len() {
                // refused at construction within the prism's depths; beyond them, the profile
                let e = self.moved(l,k,d).unwrap_or_else(|_| self.loops[l][k].clone());
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
