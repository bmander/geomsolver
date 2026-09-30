//! A boundary representation: vertices, edges on curves, faces on surfaces bounded by loops of
//! oriented edge uses, each use carrying its curve in the face's parameters (a pcurve). Indices,
//! never pointers; identity is shared by construction, never found by proximity.
//!
//! **Orientation.** A face's material normal is its surface's `S_u × S_v`, or the opposite where
//! `reversed`; walking any loop with that normal up, the face lies on the left. So in the face's
//! parameters an unreversed face's outer loop runs counter-clockwise and its holes clockwise, a
//! reversed face's the other way round — which is what lets the volume be read off the loops alone
//! (`props`). A closed shell uses every edge twice, once each way, except a degenerate edge (a
//! pole or apex), which its one face uses once.
use super::geom::{Curve,Rigid,Surface,Uv,V};
use crate::space::{distance,dot,norm,sub};

/// A point of the boundary.
#[derive(Clone,Copy,Debug)]
pub struct Vertex { pub p: V }

/// What an edge lies on: a curve, or — for a surface's pole or apex, a point its face's
/// parameters stretch into a line — nothing but its vertex.
#[derive(Clone,Copy,Debug)]
pub enum EdgeCurve { Curve(Curve),Degenerate }

/// A stretch of a curve from `t[0]` at `v[0]` to `t[1]` at `v[1]` (`t[0] < t[1]`; a closed edge
/// has one vertex at both ends).
#[derive(Clone,Copy,Debug)]
pub struct Edge { pub curve: EdgeCurve,pub t: [f64;2],pub v: [u32;2] }

impl Edge {
    pub fn point(&self,t: f64,vertices: &[Vertex]) -> V {
        match self.curve { EdgeCurve::Curve(c) => c.point(t),EdgeCurve::Degenerate => vertices[self.v[0] as usize].p }
    }
    pub fn closed(&self) -> bool { self.v[0] == self.v[1] }
}

/// An edge's curve in a face's parameters, as a function of the edge's own parameter.
#[derive(Clone,Debug)]
pub enum Pcurve {
    /// `a` at the edge's first parameter, `b` at its last, linear between.
    Line { a: Uv,b: Uv },
    /// The edge's curve inverted onto the surface; on a periodic surface the branch is the one
    /// nearest the line from `a` to `b` (each end's parameters, unwrapped).
    Inverse { a: Uv,b: Uv },
}

impl Pcurve {
    /// The parameters at the edge's parameter `t` of `edge`, on `surface`.
    pub fn at(&self,t: f64,edge: &Edge,surface: &Surface,vertices: &[Vertex]) -> Uv {
        let s = if edge.t[1] > edge.t[0] { (t-edge.t[0])/(edge.t[1]-edge.t[0]) } else { 0. };
        match *self {
            Pcurve::Line {a,b} => [a[0]+s*(b[0]-a[0]),a[1]+s*(b[1]-a[1])],
            Pcurve::Inverse {a,b} => {
                let near = [a[0]+s*(b[0]-a[0]),a[1]+s*(b[1]-a[1])];
                unwrap(surface.inverse(edge.point(t,vertices)),near,surface.periods())
            }
        }
    }
    /// `d(u, v)/dt` at `t`.
    pub fn derivative(&self,t: f64,edge: &Edge,surface: &Surface,vertices: &[Vertex]) -> Uv {
        match *self {
            Pcurve::Line {a,b} => {
                let l = edge.t[1]-edge.t[0];
                [(b[0]-a[0])/l,(b[1]-a[1])/l]
            }
            Pcurve::Inverse {..} => {
                let EdgeCurve::Curve(c) = edge.curve else { return [0.;2] };
                let uv = self.at(t,edge,surface,vertices);
                let (_,su,sv) = surface.d1(uv);
                let d = c.tangent(t);
                // least squares [S_u S_v] x = C'
                let (a11,a12,a22) = (dot(su,su),dot(su,sv),dot(sv,sv));
                let (b1,b2) = (dot(su,d),dot(sv,d));
                let det = a11*a22-a12*a12;
                [(a22*b1-a12*b2)/det,(a11*b2-a12*b1)/det]
            }
        }
    }
}

/// `uv` moved by whole periods to be nearest `near`.
pub fn unwrap(mut uv: Uv,near: Uv,periods: [Option<f64>;2]) -> Uv {
    for k in 0..2 {
        if let Some(p) = periods[k] { uv[k] += ((near[k]-uv[k])/p).round()*p; }
    }
    uv
}

/// One use of an edge by a face's loop: along the edge (`!reversed`) or against it.
#[derive(Clone,Debug)]
pub struct Coedge { pub edge: u32,pub reversed: bool,pub pcurve: Pcurve }

/// A face: a surface, which side is material, and its loops (the first outer).
#[derive(Clone,Debug)]
pub struct Face { pub surface: Surface,pub reversed: bool,pub loops: Vec<Vec<Coedge>>,pub name: String }

/// A solid's boundary: one or more closed shells, its faces in any order.
#[derive(Clone,Debug,Default)]
pub struct Brep { pub vertices: Vec<Vertex>,pub edges: Vec<Edge>,pub faces: Vec<Face> }

impl Brep {
    pub fn vertex(&mut self,p: V) -> u32 { self.vertices.push(Vertex {p}); (self.vertices.len()-1) as u32 }
    pub fn edge(&mut self,curve: EdgeCurve,t: [f64;2],v: [u32;2]) -> u32 {
        self.edges.push(Edge {curve,t,v});
        (self.edges.len()-1) as u32
    }
    /// The first and last point of a use of an edge, in the use's direction.
    pub fn ends(&self,c: &Coedge) -> [u32;2] {
        let e = &self.edges[c.edge as usize];
        if c.reversed { [e.v[1],e.v[0]] } else { e.v }
    }
    /// The parameters at the start and end of a use, in its direction.
    pub fn uv_ends(&self,face: &Face,c: &Coedge) -> [Uv;2] {
        let e = &self.edges[c.edge as usize];
        let ends = [c.pcurve.at(e.t[0],e,&face.surface,&self.vertices),c.pcurve.at(e.t[1],e,&face.surface,&self.vertices)];
        if c.reversed { [ends[1],ends[0]] } else { ends }
    }
    /// Every geometric position moved by `m`.
    pub fn moved(&self,m: &Rigid) -> Brep {
        let mut b = self.clone();
        for v in &mut b.vertices { v.p = m.point(v.p); }
        for e in &mut b.edges { if let EdgeCurve::Curve(c) = &mut e.curve { *c = c.moved(m); } }
        for f in &mut b.faces { f.surface = f.surface.moved(m); }
        b
    }
    /// A box about every point of the boundary: the corners of its faces' parameter boxes and a
    /// grid across each (for a face of revolution, the bulge between its edges), so it may be larger
    /// than the solid but is never smaller than its vertices and edges.
    pub fn bounds(&self) -> ([f64;3],[f64;3]) {
            let b = self;
        let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
        let mut grow = |p: V| for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); };
        for v in &b.vertices { grow(v.p); }
        for f in &b.faces {
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
            for i in 0..=16 { for j in 0..=16 {
                let uv = [ulo[0]+(uhi[0]-ulo[0])*i as f64/16.,ulo[1]+(uhi[1]-ulo[1])*j as f64/16.];
                grow(f.surface.point(uv));
            } }
        }
        (lo,hi)
    }
    /// The diagonal of `bounds`, the scale tolerances are taken against.
    pub fn size(&self) -> f64 {
        let (lo,hi) = self.bounds();
        if lo[0].is_finite() { norm(sub(hi,lo)) } else { 0. }
    }

    /// Everything the representation promises, checked, to `tol` (a length): every loop closed
    /// in space and in its face's parameters; every edge's ends on its curve; every use's pcurve
    /// on its surface along the edge; every face's loops turning the way its sense says; every
    /// non-degenerate edge used twice, once each way, and every degenerate one once.
    pub fn check(&self,tol: f64) -> Result<(),String> {
        let mut uses = vec![(0usize,0usize);self.edges.len()];
        for (i,e) in self.edges.iter().enumerate() {
            if let EdgeCurve::Curve(c) = e.curve {
                for k in 0..2 {
                    let d = distance(c.point(e.t[k]),self.vertices[e.v[k] as usize].p);
                    if d > tol { return Err(format!("edge {i}: its {} vertex is {d:e} off its curve",["first","last"][k])) }
                }
                if !(e.t[1] > e.t[0]) { return Err(format!("edge {i}: its parameters do not increase")) }
            }
        }
        for (fi,f) in self.faces.iter().enumerate() {
            if f.loops.is_empty() { return Err(format!("face {fi} ({}) has no loop",f.name)) }
            for (li,l) in f.loops.iter().enumerate() {
                if l.is_empty() { return Err(format!("face {fi} ({}): loop {li} is empty",f.name)) }
                for (k,c) in l.iter().enumerate() {
                    let next = &l[(k+1)%l.len()];
                    if self.ends(c)[1] != self.ends(next)[0] {
                        return Err(format!("face {fi} ({}): loop {li} breaks after use {k}",f.name))
                    }
                    // the same place, reached on the same branch of the surface's periods
                    let (a,b) = (self.uv_ends(f,c)[1],self.uv_ends(f,next)[0]);
                    if distance(f.surface.point(a),f.surface.point(b)) > tol || (a[0]-b[0]).abs().max((a[1]-b[1]).abs()) > 1e-3 {
                        return Err(format!("face {fi} ({}): loop {li} breaks in parameters after use {k}: {a:?} to {b:?}",f.name))
                    }
                    let e = &self.edges[c.edge as usize];
                    for j in 0..=8 {
                        let t = e.t[0]+(e.t[1]-e.t[0])*j as f64/8.;
                        let d = distance(f.surface.point(c.pcurve.at(t,e,&f.surface,&self.vertices)),e.point(t,&self.vertices));
                        if d > tol { return Err(format!("face {fi} ({}): edge {}'s pcurve is {d:e} off it",f.name,c.edge)) }
                    }
                    let u = &mut uses[c.edge as usize];
                    if c.reversed { u.1 += 1 } else { u.0 += 1 }
                }
                let area = self.loop_area(f,l);
                let ccw = area > 0.;
                if (li == 0) != (ccw != f.reversed) {
                    return Err(format!("face {fi} ({}): loop {li} turns the wrong way ({area:e} in its parameters)",f.name))
                }
            }
        }
        for (i,(e,u)) in self.edges.iter().zip(&uses).enumerate() {
            match e.curve {
                EdgeCurve::Degenerate => if u.0+u.1 != 1 { return Err(format!("degenerate edge {i} used {} times",u.0+u.1)) },
                EdgeCurve::Curve(_) => if *u != (1,1) {
                    return Err(format!("edge {i} ({}) used {} times along and {} against",
                        if let EdgeCurve::Curve(c) = e.curve { c.kind() } else { "" },u.0,u.1))
                }
            }
        }
        Ok(())
    }

    /// The signed area a loop encloses in its face's parameters (counter-clockwise positive),
    /// by the trapezoid rule over each use sampled finely enough to settle its sign.
    pub fn loop_area(&self,f: &Face,l: &[Coedge]) -> f64 {
        let mut pts: Vec<Uv> = Vec::new();
        for c in l {
            let e = &self.edges[c.edge as usize];
            let n = 32;
            let mut ts: Vec<f64> = (0..n).map(|j| e.t[0]+(e.t[1]-e.t[0])*j as f64/n as f64).collect();
            ts.push(e.t[1]);
            if c.reversed { ts.reverse(); }
            for &t in &ts[..ts.len()-1] { pts.push(c.pcurve.at(t,e,&f.surface,&self.vertices)); }
        }
        let n = pts.len();
        (0..n).map(|i| { let (a,b) = (pts[i],pts[(i+1)%n]); a[0]*b[1]-a[1]*b[0] }).sum::<f64>()/2.
    }
}
