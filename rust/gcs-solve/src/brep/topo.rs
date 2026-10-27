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
use super::geom::{include,widen,Box3,Curve,Rigid,Surface,Uv,V,EMPTY};
use crate::space::{distance,dot,norm,sub};

/// A point of the boundary.
#[derive(Clone,Copy,Debug)]
pub struct Vertex { pub p: V }

/// What an edge lies on: a curve, or — for a surface's pole or apex, a point its face's
/// parameters stretch into a line — nothing but its vertex.
#[derive(Clone,Debug)]
pub enum EdgeCurve { Curve(Curve),Degenerate }

/// A stretch of a curve from `t[0]` at `v[0]` to `t[1]` at `v[1]` (`t[0] < t[1]`; a closed edge
/// has one vertex at both ends).
#[derive(Clone,Debug)]
pub struct Edge {
    pub curve: EdgeCurve,
    pub t: [f64;2],
    pub v: [u32;2],
    /// The largest gap measured between the edge's curve and its pcurves' images, and its ends
    /// and its vertices (0 for an edge built exact here): what `check` holds it to, beside the
    /// tolerance it is asked for. Never widened to make a check pass.
    pub tol: f64,
}

impl Edge {
    pub fn point(&self,t: f64,vertices: &[Vertex]) -> V {
        match &self.curve { EdgeCurve::Curve(c) => c.point(t),EdgeCurve::Degenerate => vertices[self.v[0] as usize].p }
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
    /// A curve of its own in the face's parameters (its z is 0), read at the edge's parameter
    /// itself: what a native kernel hands over (`brep::json`).
    Curve(std::sync::Arc<Curve>),
}

impl Pcurve {
    /// Whether the edge's image in the face's parameters is a straight segment, each point of it
    /// an affine function of the edge's parameter: a line given so, or a line in space inverted
    /// onto a plane. A loop of such is the polygon of its corners (#589).
    pub fn straight(&self,edge: &Edge,surface: &Surface) -> bool {
        match self {
            Pcurve::Line {..} => true,
            Pcurve::Inverse {..} => matches!((surface,&edge.curve),(Surface::Plane(_),EdgeCurve::Curve(Curve::Line {..}))),
            Pcurve::Curve(_) => false,
        }
    }
    /// The parameters at the edge's parameter `t` of `edge`, on `surface`.
    pub fn at(&self,t: f64,edge: &Edge,surface: &Surface,vertices: &[Vertex]) -> Uv {
        let s = if edge.t[1] > edge.t[0] { (t-edge.t[0])/(edge.t[1]-edge.t[0]) } else { 0. };
        match *self {
            Pcurve::Curve(ref c) => { let p = c.point(t); [p[0],p[1]] }
            Pcurve::Line {a,b} => [a[0]+s*(b[0]-a[0]),a[1]+s*(b[1]-a[1])],
            Pcurve::Inverse {a,b} => {
                let near = [a[0]+s*(b[0]-a[0]),a[1]+s*(b[1]-a[1])];
                unwrap(surface.inverse_near(edge.point(t,vertices),near),near,surface.periods())
            }
        }
    }
    /// `d(u, v)/dt` at `t`.
    pub fn derivative(&self,t: f64,edge: &Edge,surface: &Surface,vertices: &[Vertex]) -> Uv {
        match *self {
            Pcurve::Curve(ref c) => { let d = c.tangent(t); [d[0],d[1]] }
            Pcurve::Line {a,b} => {
                let l = edge.t[1]-edge.t[0];
                [(b[0]-a[0])/l,(b[1]-a[1])/l]
            }
            Pcurve::Inverse {..} => inverse_derivative(self.at(t,edge,surface,vertices),t,edge,surface),
        }
    }
    /// `at` and `derivative` at `t`: an inverse curve's point found on the surface once for both.
    pub fn at_and_derivative(&self,t: f64,edge: &Edge,surface: &Surface,vertices: &[Vertex]) -> (Uv,Uv) {
        let uv = self.at(t,edge,surface,vertices);
        match self {
            Pcurve::Inverse {..} => (uv,inverse_derivative(uv,t,edge,surface)),
            _ => (uv,self.derivative(t,edge,surface,vertices)),
        }
    }
    /// The same curve with its `u` negated: what it is on its face's surface placed so that its
    /// `u` runs the other way (`Frame::rehanded`, a revolution's). Negating a coordinate is linear,
    /// so a rational curve's weights are unchanged.
    pub fn u_negated(&self) -> Pcurve {
        let neg = |p: Uv| [-p[0],p[1]];
        let neg3 = |p: V| [-p[0],p[1],p[2]];
        let frame = |f: super::geom::Frame| super::geom::Frame {o:neg3(f.o),x:neg3(f.x),y:neg3(f.y),z:f.z};
        match self {
            Pcurve::Line {a,b} => Pcurve::Line {a:neg(*a),b:neg(*b)},
            Pcurve::Inverse {a,b} => Pcurve::Inverse {a:neg(*a),b:neg(*b)},
            Pcurve::Curve(c) => Pcurve::Curve(std::sync::Arc::new(match &**c {
                Curve::Line {p,d} => Curve::Line {p:neg3(*p),d:neg3(*d)},
                Curve::BSpline(b) => Curve::BSpline(std::sync::Arc::new(super::geom::BSpline {
                    poles:b.poles.iter().map(|&p| neg3(p)).collect(),..(**b).clone()})),
                Curve::Circle(f,r) => Curve::Circle(frame(*f),*r),
                Curve::Ellipse(f,a,b) => Curve::Ellipse(frame(*f),*a,*b),
                other => other.clone(),
            })),
        }
    }
}

/// `d(u, v)/dt` of the curve of points of `surface` nearest the edge's, at its point's parameters
/// `uv`: the edge's tangent by least squares in `[S_u S_v]` there.
fn inverse_derivative(uv: Uv,t: f64,edge: &Edge,surface: &Surface) -> Uv {
    let EdgeCurve::Curve(c) = &edge.curve else { return [0.;2] };
    let (_,su,sv) = surface.d1(uv);
    let d = c.tangent(t);
    // least squares [S_u S_v] x = C'
    let (a11,a12,a22) = (dot(su,su),dot(su,sv),dot(sv,sv));
    let (b1,b2) = (dot(su,d),dot(sv,d));
    let det = a11*a22-a12*a12;
    [(a22*b1-a12*b2)/det,(a11*b2-a12*b1)/det]
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

/// A loop's uses walked the other way round: in reverse order, each against its edge.
pub fn reverse_loop(l: Vec<Coedge>) -> Vec<Coedge> {
    l.into_iter().rev().map(|c| Coedge {reversed:!c.reversed,..c}).collect()
}

/// A face: a surface, which side is material, and its loops (the first outer).
#[derive(Clone,Debug)]
pub struct Face { pub surface: Surface,pub reversed: bool,pub loops: Vec<Vec<Coedge>>,pub name: String }

impl Face {
    /// The face's outward normal at `p` (on its surface, or near it): its surface's, turned where
    /// the face is the reversed side; none at a point where the surface has none (a pole).
    pub fn outward(&self,p: V) -> Option<V> {
        let n = self.surface.normal(self.surface.inverse(p))?;
        Some(if self.reversed { n.map(|x| -x) } else { n })
    }
    /// The edges this face uses twice, once each side: its seams, which bound nothing.
    pub fn seams(&self) -> std::collections::BTreeSet<u32> {
        let mut seen = std::collections::BTreeSet::new();
        self.loops.iter().flatten().filter(|c| !seen.insert(c.edge)).map(|c| c.edge).collect()
    }
}

/// How `Brep::check` opens a refusal of an edge used as often each way, but more than once: the
/// boundary is no manifold there (two pieces touching along it, or a face kept twice).
const NOT_MANIFOLD: &str = "not a manifold: ";
/// And of one used more often one way than the other, or by no face: the boundary is open there
/// (a face of it missing, or one too many).
const OPEN: &str = "not closed: ";

/// A solid's boundary: one or more closed shells, its faces in any order.
#[derive(Clone,Debug,Default)]
pub struct Brep { pub vertices: Vec<Vertex>,pub edges: Vec<Edge>,pub faces: Vec<Face> }

impl Brep {
    pub fn vertex(&mut self,p: V) -> u32 { self.vertices.push(Vertex {p}); (self.vertices.len()-1) as u32 }
    /// The faces of `self` and of `other` as one B-rep, sharing nothing (two sheets to split by at once).
    pub fn joined(&self,other: &Brep) -> Brep {
        let mut c = self.clone();
        c.join(other);
        c
    }
    /// `other`'s faces added to these, sharing nothing (`joined`, in place).
    pub fn join(&mut self,other: &Brep) {
        let (nv,ne) = (self.vertices.len() as u32,self.edges.len() as u32);
        self.vertices.extend(other.vertices.iter().cloned());
        self.edges.extend(other.edges.iter().cloned().map(|mut e| { e.v = e.v.map(|v| v+nv); e }));
        self.faces.extend(other.faces.iter().cloned().map(|mut f| { for co in f.loops.iter_mut().flatten() { co.edge += ne; } f }));
    }
    pub fn edge(&mut self,curve: EdgeCurve,t: [f64;2],v: [u32;2]) -> u32 {
        self.edges.push(Edge {curve,t,v,tol:0.});
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
    /// **Every geometric position reflected by the improper motion `m`** (#578): what a mirrored
    /// instance's solid is, its body's own B-rep moved. Each face's surface is reflected and placed
    /// right-handed again (`Surface::reflected`); a reflection turns which way a loop walks about
    /// the material, so every loop is walked the other way round, and the sense a face's
    /// material normal has against its surface's `S_u × S_v` — which the reflection turns, and a
    /// surface of revolution's `u` running the other way turns back — follows, its pcurves'
    /// `u` negated where it runs so. Every edge is still used once each way.
    pub fn reflected(&self,m: &Rigid) -> Result<Brep,String> {
        let mut b = self.clone();
        for v in &mut b.vertices { v.p = m.point(v.p); }
        for e in &mut b.edges { if let EdgeCurve::Curve(c) = &mut e.curve { *c = c.reflected(m)?; } }
        for f in &mut b.faces {
            let (surface,flip) = f.surface.reflected(m)?;
            f.surface = surface;
            f.reversed = f.reversed == flip;
            for l in &mut f.loops {
                let mut walked = reverse_loop(std::mem::take(l));
                if flip {
                    for c in &mut walked { c.pcurve = c.pcurve.u_negated(); }
                }
                *l = walked;
            }
        }
        Ok(b)
    }
    /// A box about every point of the boundary, never smaller than it and exact where its geometry
    /// is analytic: its vertices, each edge's `Curve::bounds`, and where a face may bulge past its
    /// edges, its surface's `bounds_over` the parameters its loops reach. What a `through:` cutter
    /// spans and a tool is culled by, so a sample between a curve's extremes is never its edge.
    pub fn bounds(&self) -> Box3 {
        let mut b = EMPTY;
        for v in &self.vertices { include(&mut b,v.p); }
        for e in &self.edges { if let EdgeCurve::Curve(c) = &e.curve { widen(&mut b,c.bounds(e.t)); } }
        for f in self.faces.iter().filter(|f| !f.loops.is_empty()) { widen(&mut b,f.surface.bounds_over(|| self.parameter_range(f))); }
        b
    }
    /// The parameters a face's loops (one or more) reach: read at 17 points along each use and
    /// widened by the largest step between two of them, so a range read off an inverted pcurve is
    /// not short.
    fn parameter_range(&self,f: &Face) -> [[f64;2];2] {
        const N: usize = 16;
        let (mut lo,mut hi,mut step) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2],[0f64;2]);
        for c in f.loops.iter().flatten() {
            let e = &self.edges[c.edge as usize];
            let mut last: Option<Uv> = None;
            for j in 0..=N {
                let uv = c.pcurve.at(e.t[0]+(e.t[1]-e.t[0])*j as f64/N as f64,e,&f.surface,&self.vertices);
                for k in 0..2 {
                    lo[k] = lo[k].min(uv[k]); hi[k] = hi[k].max(uv[k]);
                    if let Some(l) = last { step[k] = step[k].max((uv[k]-l[k]).abs()); }
                }
                last = Some(uv);
            }
        }
        [[lo[0]-step[0],hi[0]+step[0]],[lo[1]-step[1],hi[1]+step[1]]]
    }
    /// The diagonal of `bounds`, the scale tolerances are taken against.
    pub fn size(&self) -> f64 {
        let (lo,hi) = self.bounds();
        if lo[0].is_finite() { norm(sub(hi,lo)) } else { 0. }
    }

    /// Everything the representation promises, checked, to `tol` (a length) or an edge's own
    /// measured tolerance where that is larger: every loop closed
    /// in space and in its face's parameters; every edge's ends on its curve; every use's pcurve
    /// on its surface along the edge; every face's loops turning the way its sense says; every
    /// non-degenerate edge used twice, once each way, and every degenerate one once. An edge used
    /// as often each way, more than once, is a `Kind::NotManifold` refusal (`NOT_MANIFOLD` opens
    /// what it says); one used more often one way, or never, is `Kind::Open` (`OPEN`).
    pub fn check(&self,tol: f64) -> Result<(),super::KernelError> { self.check_settled(tol,&[]) }

    /// The same boundary less every edge no face uses and every vertex no edge does, the rest
    /// numbered again in order: what faces glued away inside a solid leave (`boolean::glued`
    /// keeps every edge, and one whose faces all went bounds nothing).
    pub fn compacted(mut self) -> Brep {
        // the items `used` marks, and where each old one went
        fn kept<T>(items: Vec<T>,used: &[bool]) -> (Vec<T>,Vec<u32>) {
            let mut at = vec![u32::MAX;items.len()];
            let kept = items.into_iter().enumerate().filter(|(i,_)| used[*i]).enumerate()
                .map(|(k,(i,x))| { at[i] = k as u32; x }).collect();
            (kept,at)
        }
        let mut used = vec![false;self.edges.len()];
        for c in self.faces.iter().flat_map(|f| f.loops.iter().flatten()) { used[c.edge as usize] = true; }
        let (edges,edge_at) = kept(std::mem::take(&mut self.edges),&used);
        let mut held = vec![false;self.vertices.len()];
        for e in &edges { for v in e.v { held[v as usize] = true; } }
        let (vertices,vertex_at) = kept(std::mem::take(&mut self.vertices),&held);
        let edges = edges.into_iter().map(|e| Edge {v:e.v.map(|v| vertex_at[v as usize]),..e}).collect();
        for c in self.faces.iter_mut().flat_map(|f| f.loops.iter_mut().flatten()) { c.edge = edge_at[c.edge as usize]; }
        Brep {vertices,edges,faces:self.faces}
    }

    /// `check`, each face `settled` marks (one carried whole from a solid checked before, #589)
    /// read for its loops' ends and its edges' uses alone: its geometry is what was checked.
    pub fn check_settled(&self,tol: f64,settled: &[bool]) -> Result<(),super::KernelError> {
        let mut uses = vec![(0usize,0usize);self.edges.len()];
        for (i,e) in self.edges.iter().enumerate() {
            if let EdgeCurve::Curve(c) = &e.curve {
                for k in 0..2 {
                    let d = distance(c.point(e.t[k]),self.vertices[e.v[k] as usize].p);
                    if d > tol.max(e.tol) { return Err(format!("edge {i}: its {} vertex is {d:e} off its curve",["first","last"][k]).into()) }
                }
                if !(e.t[1] > e.t[0]) { return Err(format!("edge {i}: its parameters do not increase").into()) }
            }
        }
        for (fi,f) in self.faces.iter().enumerate() {
            if f.loops.is_empty() { return Err(format!("face {fi} ({}) has no loop",f.name).into()) }
            let settled = settled.get(fi) == Some(&true);
            for (li,l) in f.loops.iter().enumerate() {
                if l.is_empty() { return Err(format!("face {fi} ({}): loop {li} is empty",f.name).into()) }
                for (k,c) in l.iter().enumerate() {
                    let next = &l[(k+1)%l.len()];
                    if self.ends(c)[1] != self.ends(next)[0] {
                        return Err(format!("face {fi} ({}): loop {li} breaks after use {k}",f.name).into())
                    }
                    let u = &mut uses[c.edge as usize];
                    if c.reversed { u.1 += 1 } else { u.0 += 1 }
                    if settled { continue }
                    // the same place, reached on the same branch of the surface's periods
                    let (a,b) = (self.uv_ends(f,c)[1],self.uv_ends(f,next)[0]);
                    // each pcurve's end within twice its edge's tolerance of the vertex (its image to the
                    // curve, the curve's end to the vertex), so two ends within twice the sum
                    let corner = tol.max(2.*(self.edges[c.edge as usize].tol+self.edges[next.edge as usize].tol));
                    if distance(f.surface.point(a),f.surface.point(b)) > corner || (a[0]-b[0]).abs().max((a[1]-b[1]).abs()) > 1e-3 {
                        return Err(format!("face {fi} ({}): loop {li} breaks in parameters after use {k}: {a:?} to {b:?}",f.name).into())
                    }
                    let e = &self.edges[c.edge as usize];
                    // a line on a plane, straight in its parameters, is off it most at an end
                    let flat = matches!((&f.surface,&e.curve),(Surface::Plane(_),EdgeCurve::Curve(Curve::Line {..})))
                        && c.pcurve.straight(e,&f.surface);
                    for j in (0..=8).filter(|&j| !flat || j%8 == 0) {
                        let t = e.t[0]+(e.t[1]-e.t[0])*j as f64/8.;
                        let d = distance(f.surface.point(c.pcurve.at(t,e,&f.surface,&self.vertices)),e.point(t,&self.vertices));
                        if d > tol.max(e.tol) { return Err(format!("face {fi} ({}): edge {}'s pcurve is {d:e} off it",f.name,c.edge).into()) }
                    }
                }
                if settled { continue }
                let area = self.loop_area(f,l);
                let ccw = area > 0.;
                if (li == 0) != (ccw != f.reversed) {
                    return Err(format!("face {fi} ({}): loop {li} turns the wrong way ({area:e} in its parameters)",f.name).into())
                }
            }
        }
        for (i,(e,u)) in self.edges.iter().zip(&uses).enumerate() {
            match e.curve {
                EdgeCurve::Degenerate => if u.0+u.1 != 1 { return Err(format!("degenerate edge {i} used {} times",u.0+u.1).into()) },
                EdgeCurve::Curve(_) => if *u != (1,1) {
                    let (kind,says) = if u.0 == u.1 && u.0 > 1 { (super::Kind::NotManifold,NOT_MANIFOLD) } else { (super::Kind::Open,OPEN) };
                    return Err(super::KernelError::new(kind,
                        format!("{says}edge {i} ({}) near {:?} used {} times along and {} against",
                        if let EdgeCurve::Curve(c) = &e.curve { c.kind() } else { "" },
                        e.point(0.5*(e.t[0]+e.t[1]),&self.vertices).map(|x| (x*1e4).round()/1e4),u.0,u.1)))
                }
            }
        }
        Ok(())
    }

    /// The boundary of a solid touching itself along lines, each held as two edges on one curve:
    /// where an edge is used twice each way (two pieces of material meeting along it and nowhere
    /// else near), its four faces sorted round it, each paired with its neighbour across the
    /// material between them — one along, one against — and the second pair given an edge of its
    /// own. Its ends are then pinches (`pinches`). How many edges it split, or `Err` where four
    /// uses pair no way a solid can (faces meeting tangent there, or material on both sides of one).
    pub fn unpinched(&mut self) -> Result<usize,String> {
        let mut uses: Vec<Vec<(usize,usize,usize)>> = vec![Vec::new();self.edges.len()];
        for (fi,f) in self.faces.iter().enumerate() {
            for (li,l) in f.loops.iter().enumerate() {
                for (ci,c) in l.iter().enumerate() { uses[c.edge as usize].push((fi,li,ci)); }
            }
        }
        let mut split = 0;
        for e in 0..self.edges.len() {
            if uses[e].len() != 4 { continue }
            let EdgeCurve::Curve(curve) = &self.edges[e].curve else { continue };
            let along = uses[e].iter().filter(|&&(f,l,c)| !self.faces[f].loops[l][c].reversed).count();
            if along != 2 { continue }
            let t = 0.5*(self.edges[e].t[0]+self.edges[e].t[1]);
            let d = curve.tangent(t);
            let Some(d) = crate::space::normalised(d) else { return Err(format!("edge {e} has no direction")) };
            let (a,b) = crate::space::across(d);
            // each use's face: the way into it from the edge (left of the use, its material normal
            // up) as an angle about the edge, and whether its material lies on from there
            let mut round: Vec<(f64,bool,usize)> = Vec::new();
            for (k,&(f,l,c)) in uses[e].iter().enumerate() {
                let face = &self.faces[f];
                let co = &face.loops[l][c];
                let uv = co.pcurve.at(t,&self.edges[e],&face.surface,&self.vertices);
                let Some(n) = face.surface.normal(uv) else { return Err(format!("edge {e}: a face with no normal")) };
                let n = if face.reversed { crate::space::scale(n,-1.) } else { n };
                let way = if co.reversed { crate::space::scale(d,-1.) } else { d };
                let into = crate::space::cross(n,way);
                let angle = dot(into,b).atan2(dot(into,a));
                // turning on about `d` from `into`, the material (against `n`) lies ahead or behind
                let on = dot(crate::space::scale(n,-1.),crate::space::cross(d,into)) > 0.;
                round.push((angle,on,k));
            }
            round.sort_by(|x,y| x.0.total_cmp(&y.0));
            let gap = (0..4).map(|i| { let g = round[(i+1)%4].0-round[i].0; if i == 3 { g+std::f64::consts::TAU } else { g } })
                .fold(f64::INFINITY,f64::min);
            if gap < 1e-6 { return Err(format!("edge {e}: faces meeting tangent along it, used twice each way")) }
            // the material wedges: from a face whose material lies on to the next, which must have
            // it behind
            let start = (0..4).find(|&i| round[i].1).ok_or(format!("edge {e}: no material beside it"))?;
            let pairs = [[round[start].2,round[(start+1)%4].2],[round[(start+2)%4].2,round[(start+3)%4].2]];
            let alternate = (0..4).all(|i| round[(start+i)%4].1 == (i % 2 == 0));
            let opposed = pairs.iter().all(|&[x,y]| {
                let r = |k: usize| { let (f,l,c) = uses[e][k]; self.faces[f].loops[l][c].reversed };
                r(x) != r(y)
            });
            if !alternate || !opposed { return Err(format!("edge {e}: four uses that pair no way a solid can")) }
            let twin = self.edges[e].clone();
            self.edges.push(twin);
            let new = (self.edges.len()-1) as u32;
            for k in pairs[1] { let (f,l,c) = uses[e][k]; self.faces[f].loops[l][c].edge = new; }
            split += 1;
        }
        Ok(split)
    }

    /// Where the solid touches itself along a line (`unpinched`): two edges between the same
    /// vertices along one curve, within `tol` at their middles; each line's ends.
    pub fn self_touching(&self,tol: f64) -> Vec<[V;2]> {
        // the edges between each pair of vertices, then each against the ones after it there
        let mut between: std::collections::BTreeMap<[u32;2],Vec<usize>> = Default::default();
        for (i,e) in self.edges.iter().enumerate() {
            if !matches!(e.curve,EdgeCurve::Curve(_)) { continue }
            between.entry([e.v[0].min(e.v[1]),e.v[0].max(e.v[1])]).or_default().push(i);
        }
        let mid = |e: &Edge| e.point(0.5*(e.t[0]+e.t[1]),&self.vertices);
        let mut out = Vec::new();
        for (_,list) in between {
            if list.len() < 2 { continue }
            let mids: Vec<V> = list.iter().map(|&i| mid(&self.edges[i])).collect();
            for (j,&i) in list.iter().enumerate() {
                let a = &self.edges[i];
                let twin = (j+1..list.len()).any(|k| distance(mids[j],mids[k]) <= tol.max(a.tol).max(self.edges[list[k]].tol));
                if twin { out.push((i,a.v.map(|v| self.vertices[v as usize].p))); }
            }
        }
        // in the edges' order, as found before
        out.sort_by_key(|&(i,_)| i);
        out.into_iter().map(|(_,p)| p).collect()
    }

    /// Where the solid pinches: vertices whose faces fall into more than one fan — joined round
    /// the vertex only through the edges that end there — so the solid touches itself at a point
    /// (a bore whose end is tangent to another's wall). A valid boundary, but not a manifold one.
    pub fn pinches(&self) -> Vec<V> {
        let mut out = Vec::new();
        for (vi,v) in self.vertices.iter().enumerate() {
            let vi = vi as u32;
            let faces: Vec<usize> = (0..self.faces.len()).filter(|&f| self.faces[f].loops.iter().flatten()
                .any(|c| self.edges[c.edge as usize].v.contains(&vi))).collect();
            if faces.len() < 2 { continue }
            // union the faces sharing an edge that ends at the vertex
            let mut root: Vec<usize> = (0..faces.len()).collect();
            fn find(r: &mut Vec<usize>,i: usize) -> usize { if r[i] == i { i } else { let j = find(r,r[i]); r[i] = j; j } }
            for (ei,e) in self.edges.iter().enumerate() {
                if !e.v.contains(&vi) || matches!(e.curve,EdgeCurve::Degenerate) { continue }
                let using: Vec<usize> = faces.iter().enumerate().filter(|(_,&f)| self.faces[f].loops.iter().flatten()
                    .any(|c| c.edge as usize == ei)).map(|(k,_)| k).collect();
                for w in using.windows(2) { let (a,b) = (find(&mut root,w[0]),find(&mut root,w[1])); root[a] = b; }
            }
            let mut roots: Vec<usize> = (0..faces.len()).map(|i| find(&mut root,i)).collect();
            roots.sort(); roots.dedup();
            if roots.len() > 1 { out.push(v.p); }
        }
        out
    }

    /// The signed area a loop encloses in its face's parameters (counter-clockwise positive),
    /// by the trapezoid rule over each use sampled finely enough to settle its sign.
    pub fn loop_area(&self,f: &Face,l: &[Coedge]) -> f64 {
        let mut pts: Vec<Uv> = Vec::new();
        for c in l {
            let e = &self.edges[c.edge as usize];
            // a straight image is its first corner's (the next use's first is its last)
            let n = if c.pcurve.straight(e,&f.surface) { 1 } else { 32 };
            let mut ts: Vec<f64> = (0..n).map(|j| e.t[0]+(e.t[1]-e.t[0])*j as f64/n as f64).collect();
            ts.push(e.t[1]);
            if c.reversed { ts.reverse(); }
            for &t in &ts[..ts.len()-1] { pts.push(c.pcurve.at(t,e,&f.surface,&self.vertices)); }
        }
        let n = pts.len();
        (0..n).map(|i| { let (a,b) = (pts[i],pts[(i+1)%n]); a[0]*b[1]-a[1]*b[0] }).sum::<f64>()/2.
    }

    /// Each edge's tolerance: the largest gap between its curve and each use's pcurve image
    /// (sampled along it, its knots and ends included), and between its curve's ends and its
    /// vertices.
    pub fn measure_tolerances(&mut self) {
        let mut gaps = vec![0_f64;self.edges.len()];
        for f in &self.faces {
            for c in f.loops.iter().flatten() {
                let e = &self.edges[c.edge as usize];
                let gap = &mut gaps[c.edge as usize];
                let mut ts: Vec<f64> = (0..=32).map(|k| e.t[0]+(e.t[1]-e.t[0])*k as f64/32.).collect();
                if let EdgeCurve::Curve(curve) = &e.curve { ts.extend(curve.breaks(e.t)); }
                if let Pcurve::Curve(pc) = &c.pcurve { ts.extend(pc.breaks(e.t)); }
                for t in ts {
                    let d = distance(f.surface.point(c.pcurve.at(t,e,&f.surface,&self.vertices)),e.point(t,&self.vertices));
                    *gap = gap.max(d);
                }
            }
        }
        for (e,gap) in self.edges.iter_mut().zip(gaps) {
            let ends = match &e.curve {
                EdgeCurve::Curve(c) => (0..2).map(|k| distance(c.point(e.t[k]),self.vertices[e.v[k] as usize].p)).fold(0.,f64::max),
                EdgeCurve::Degenerate => 0.,
            };
            e.tol = gap.max(ends);
        }
    }
}
