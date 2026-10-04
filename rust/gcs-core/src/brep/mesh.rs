//! A B-rep meshed within a stated sag: every edge sampled once, in space, so the two faces that
//! share it share its points to the bit; every face triangulated in its own parameters (scaled
//! to lengths, mirrored where the face is reversed, so its triangles turn with its material
//! normal) as the constrained Delaunay triangulation of its loops' samples — each found by walking
//! to it, its cavity replaced, the loops' sides recovered by flips — and refined where a triangle's
//! measured sag — the surface's distance from it at its middle and at the middles of its inner sides
//! — exceeds the bar, its middle put in and the triangulation made Delaunay about it again — and
//! where a facet turns from its surface's outward normal, since a sag says nothing of which way a
//! facet faces. The faces are meshed side by side.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::geom::{Rigid,Uv,V};
use super::topo::{Brep,EdgeCurve,Face};
use crate::delaunay::expansion::Expansion;
use crate::space::{add,cross,distance,dot,norm,scale,sub};

/// Triangles over shared points, each turning with the solid's outward normal.
#[derive(Clone,Debug,Default)]
pub struct Mesh { pub pts: Vec<V>,pub tris: Vec<[u32;3]>,
    /// The face of the meshed B-rep each triangle is of (a pattern's: the sector's).
    pub of: Vec<u32>,
    /// The greatest sag measured on any face's triangles.
    pub sag: f64,
    /// Triangles facing against their surface's outward normal at their middle, of those thicker
    /// than a tenth of the bar (none, but where a face's refinement ran out of points).
    pub turned: usize }

impl Mesh {
    /// Nine coordinates a triangle, for STL.
    pub fn triangles(&self) -> Vec<f64> {
        self.tris.iter().flat_map(|t| t.iter().flat_map(|&i| self.pts[i as usize])).collect()
    }
}

/// `(b − a) × (c − a)`'s sign, exactly.
fn orient(a: [f64;2],b: [f64;2],c: [f64;2]) -> i8 {
    let det = (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
    let bound = 1e-14*((b[0]-a[0]).abs()*(c[1]-a[1]).abs()+(b[1]-a[1]).abs()*(c[0]-a[0]).abs());
    if det.abs() > bound { return det.signum() as i8 }
    let l = Expansion::diff(b[0],a[0]).mul(&Expansion::diff(c[1],a[1]));
    let r = Expansion::diff(b[1],a[1]).mul(&Expansion::diff(c[0],a[0]));
    l.sub(&r).sign()
}

/// Whether `d` is strictly inside the circle through the counter-clockwise `a`, `b`, `c`, exactly.
fn in_circle_exact(a: [f64;2],b: [f64;2],c: [f64;2],d: [f64;2]) -> bool {
    let e = |x: f64,y: f64| Expansion::diff(x,y);
    let (ax,ay,bx,by,cx,cy) = (e(a[0],d[0]),e(a[1],d[1]),e(b[0],d[0]),e(b[1],d[1]),e(c[0],d[0]),e(c[1],d[1]));
    let sq = |x: &Expansion,y: &Expansion| x.mul(x).add(&y.mul(y));
    let cr = |x1: &Expansion,y1: &Expansion,x2: &Expansion,y2: &Expansion| x1.mul(y2).sub(&x2.mul(y1));
    let det = sq(&ax,&ay).mul(&cr(&bx,&by,&cx,&cy)).sub(&sq(&bx,&by).mul(&cr(&ax,&ay,&cx,&cy))).add(&sq(&cx,&cy).mul(&cr(&ax,&ay,&bx,&by)));
    det.sign() > 0
}

/// Whether `d` is strictly inside the circle through the counter-clockwise `a`, `b`, `c`: a filtered
/// evaluation, exact where it is too near zero to decide.
fn in_circle(a: [f64;2],b: [f64;2],c: [f64;2],d: [f64;2]) -> bool {
    let (ax,ay,bx,by,cx,cy) = (a[0]-d[0],a[1]-d[1],b[0]-d[0],b[1]-d[1],c[0]-d[0],c[1]-d[1]);
    let det = (ax*ax+ay*ay)*(bx*cy-cx*by)-(bx*bx+by*by)*(ax*cy-cx*ay)+(cx*cx+cy*cy)*(ax*by-bx*ay);
    let scale = (ax*ax+ay*ay)*(bx.abs()*cy.abs()+cx.abs()*by.abs())+(bx*bx+by*by)*(ax.abs()*cy.abs()+cx.abs()*ay.abs())
        +(cx*cx+cy*cy)*(ax.abs()*by.abs()+bx.abs()*ay.abs());
    if det.abs() > 1e-12*scale { det > 0. } else { in_circle_exact(a,b,c,d) }
}

/// An edge's parameters, refined until each chord's middle is within `bar` of the curve and no
/// chord turns more than `angular` (a closed edge in at least three pieces).
fn edge_params(b: &Brep,e: usize,bar: f64,angular: f64) -> Vec<f64> {
    let edge = &b.edges[e];
    let EdgeCurve::Curve(c) = &edge.curve else { return vec![edge.t[0],edge.t[1]] };
    let mut out = vec![edge.t[0]];
    // the first pieces: a closed edge in three; a B-spline's every knot span (one inflection at most
    // in a cubic's, which the quarter points below see though a midpoint test is blind to it)
    let mut cuts = vec![edge.t[0]];
    let near = 1e-6*(edge.t[1]-edge.t[0]);
    if let super::geom::Curve::BSpline(s) = c {
        // (a knot is a cut only a bar or more along from the last one and from the end: a fit's
        // knots crowd where it bends, and a cut at each fans slivers there)
        let end = c.point(edge.t[1]);
        for k in s.breaks(edge.t).into_iter().filter(|&k| k-edge.t[0] > near && edge.t[1]-k > near) {
            let p = c.point(k);
            if distance(p,c.point(*cuts.last().unwrap())) >= bar && distance(p,end) >= bar { cuts.push(k); }
        }
    }
    cuts.push(edge.t[1]);
    // (cut in several pieces regardless, a short span fans slivers from the face's nearest point)
    let each = |_: f64,_: f64| if !matches!(c,super::geom::Curve::BSpline(_)) && edge.closed() { 3 } else { 1 };
    let mut stack: Vec<(f64,f64,u32)> = cuts.windows(2).flat_map(|w| { let n = each(w[0],w[1]); (0..n).map(move |k| {
        let h = (w[1]-w[0])/n as f64;
        (w[0]+h*k as f64,w[0]+h*(k+1) as f64,0)
    }) }).collect();
    stack.reverse();
    while let Some((a,z,depth)) = stack.pop() {
        let (pa,pz) = (c.point(a),c.point(z));
        let m = (a+z)/2.;
        let pm = c.point(m);
        let chord = sub(pz,pa);
        let l2 = dot(chord,chord);
        let s = if l2 > 0. { (dot(sub(pm,pa),chord)/l2).clamp(0.,1.) } else { 0. };
        let off = distance(pm,add(pa,scale(chord,s)));
        let (ta,tz) = (c.tangent(a),c.tangent(z));
        let turn = (dot(ta,tz)/(norm(ta)*norm(tz)).max(1e-300)).clamp(-1.,1.).dacos();
        // and the quarter points, which a stretch centred on an inflection cannot hide
        let strays = || [0.25,0.75].iter().any(|&f| {
            let q = c.point(a+(z-a)*f);
            let s = if l2 > 0. { (dot(sub(q,pa),chord)/l2).clamp(0.,1.) } else { 0. };
            distance(q,add(pa,scale(chord,s))) > bar
        });
        // (a traced or fitted curve's chord no longer than the bar turns as it likes: a tight bend of
        // an intersection near tangency, cut by its turning, fans slivers from the face's nearest
        // point; a circle's or an ellipse's tight bend is a small feature, and keeps its turns)
        let conic = matches!(c,super::geom::Curve::Circle(..) | super::geom::Curve::Ellipse(..));
        if depth < 40 && (off > bar || turn > angular && (conic || l2 > bar*bar) || strays()) { stack.push((m,z,depth+1)); stack.push((a,m,depth+1)); }
        else { out.push(z); }
    }
    out
}

const NONE: u32 = u32::MAX;

/// A triangulation of points in the plane, each triangle counter-clockwise with its three
/// neighbours (`nbr[t][k]` across the side from corner `k` to corner `k + 1`), some sides held (a
/// face's boundary), every point found by walking from the last triangle reached, so that each
/// insertion costs the neighbourhood it changes rather than the whole triangulation.
struct Tri2 {
    pts: Vec<[f64;2]>,
    tris: Vec<[u32;3]>,
    nbr: Vec<[u32;3]>,
    /// One triangle about each point.
    about: Vec<u32>,
    held: std::collections::BTreeSet<(u32,u32)>,
    last: usize,
    /// Triangles written since the refinement last looked (theirs to measure again).
    dirty: Vec<bool>,
}

impl Tri2 {
    fn held(&self,a: u32,b: u32) -> bool { self.held.contains(&(a.min(b),a.max(b))) }
    fn p(&self,v: u32) -> [f64;2] { self.pts[v as usize] }
    /// The side of `t` from `a` to `b`.
    fn side(&self,t: usize,a: u32,b: u32) -> Option<usize> { (0..3).find(|&k| self.tris[t][k] == a && self.tris[t][(k+1)%3] == b) }
    /// Writes triangle `t` and points its corners at it.
    fn write(&mut self,t: usize,tri: [u32;3],nbr: [u32;3]) {
        if t == self.tris.len() { self.tris.push(tri); self.nbr.push(nbr); self.dirty.push(true); }
        else { self.tris[t] = tri; self.nbr[t] = nbr; self.dirty[t] = true; }
        for v in tri { self.about[v as usize] = t as u32; }
    }
    /// Points `u`'s side from `b` to `a` at `t` (the far side of `t`'s side from `a` to `b`).
    fn link(&mut self,u: u32,a: u32,b: u32,t: usize) {
        if u == NONE { return }
        let k = self.side(u as usize,b,a).expect("a neighbour shares the side");
        self.nbr[u as usize][k] = t as u32;
    }
    /// The triangle holding `p` (on its sides included), walked to from the last one reached.
    fn locate(&mut self,p: [f64;2]) -> Result<usize,String> {
        let mut t = self.last.min(self.tris.len()-1);
        for step in 0..=self.tris.len() {
            let mut next = None;
            for j in 0..3 {
                // the sides in turn from a changing one, so a walk cannot circle
                let k = (j+step)%3;
                let (a,b) = (self.tris[t][k],self.tris[t][(k+1)%3]);
                if orient(self.p(a),self.p(b),p) < 0 { next = Some(self.nbr[t][k]); break }
            }
            match next {
                None => { self.last = t; return Ok(t) }
                Some(NONE) => return Err("a point lies outside its face's triangulation".into()),
                Some(u) => t = u as usize,
            }
        }
        // a walk that did not arrive: look at every triangle
        (0..self.tris.len()).find(|&t| { let [a,b,c] = self.tris[t].map(|v| self.p(v));
            orient(a,b,p) >= 0 && orient(b,c,p) >= 0 && orient(c,a,p) >= 0 }).ok_or("a point lies outside its face's triangulation".into())
    }
    /// Point `v` into the Delaunay triangulation (Bowyer–Watson): every triangle whose circle holds
    /// it, a connected cavity about the one it lies in, replaced by the fan from it to the cavity's
    /// boundary.
    fn insert(&mut self,v: u32,mark: &mut Vec<u32>,stamp: u32) -> Result<(),String> {
        let p = self.p(v);
        let t0 = self.locate(p)?;
        let in_circle_of = |s: &Tri2,t: usize| { let [a,b,c] = s.tris[t].map(|x| s.p(x)); in_circle(a,b,c,p) };
        if !in_circle_of(self,t0) {
            // on a corner of the triangle it lies in: split it, as the triangle about a pole holds
            self.split(t0,v);
            return Ok(())
        }
        mark.resize(self.tris.len(),0);
        let mut cavity = vec![t0];
        mark[t0] = stamp;
        let mut i = 0;
        while i < cavity.len() {
            let t = cavity[i];
            i += 1;
            for k in 0..3 {
                let u = self.nbr[t][k];
                if u != NONE && mark[u as usize] != stamp && in_circle_of(self,u as usize) { mark[u as usize] = stamp; cavity.push(u as usize); }
            }
        }
        let mut rim = Vec::new();
        for &t in &cavity { for k in 0..3 {
            let u = self.nbr[t][k];
            if u == NONE || mark[u as usize] != stamp {
                let (a,b) = (self.tris[t][k],self.tris[t][(k+1)%3]);
                if orient(self.p(a),self.p(b),p) <= 0 { return Err("a cavity is not star-shaped about its point".into()) }
                rim.push((a,b,u));
            }
        } }
        // the cavity's slots, then new ones; the fan's triangles each (a, b, v), linked round
        // a disc of k triangles has k + 2 sides round it
        if rim.len() != cavity.len()+2 { return Err("a cavity is not a disc".into()) }
        let mut slots = cavity;
        slots.extend([self.tris.len(),self.tris.len()+1]);
        for (&(a,b,u),&t) in rim.iter().zip(&slots) {
            self.write(t,[a,b,v],[u,NONE,NONE]);
            self.link(u,a,b,t);
        }
        for (i,&(a,b,_)) in rim.iter().enumerate() {
            let after = rim.iter().position(|r| r.0 == b).unwrap();
            let before = rim.iter().position(|r| r.1 == a).unwrap();
            self.nbr[slots[i]][1] = slots[after] as u32;
            self.nbr[slots[i]][2] = slots[before] as u32;
        }
        self.last = slots[0];
        Ok(())
    }
    /// Point `v` inside (or on a side of) triangle `t`, the triangle split three ways.
    fn split(&mut self,t: usize,v: u32) -> [usize;3] {
        let [a,b,c] = self.tris[t];
        let [nab,nbc,nca] = self.nbr[t];
        let (t1,t2) = (self.tris.len(),self.tris.len()+1);
        self.write(t1,[b,c,v],[nbc,t2 as u32,t as u32]);
        self.write(t2,[c,a,v],[nca,t as u32,t1 as u32]);
        self.write(t,[a,b,v],[nab,t1 as u32,t2 as u32]);
        self.link(nbc,b,c,t1);
        self.link(nca,c,a,t2);
        [t,t1,t2]
    }
    /// Flips side `k` of `t` — (a, b), with c opposite and d beyond — to (c, d), where the
    /// quadrilateral is convex; the two triangles then (c, a, d) and (d, b, c), in the same slots.
    fn flip(&mut self,t: usize,k: usize) -> Option<usize> {
        let u = self.nbr[t][k];
        if u == NONE { return None }
        let u = u as usize;
        let (a,b,c) = (self.tris[t][k],self.tris[t][(k+1)%3],self.tris[t][(k+2)%3]);
        let j = self.side(u,b,a)?;
        let d = self.tris[u][(j+2)%3];
        if orient(self.p(c),self.p(a),self.p(d)) <= 0 || orient(self.p(d),self.p(b),self.p(c)) <= 0 { return None }
        let (nbc,nca) = (self.nbr[t][(k+1)%3],self.nbr[t][(k+2)%3]);
        let (nad,ndb) = (self.nbr[u][(j+1)%3],self.nbr[u][(j+2)%3]);
        self.write(t,[c,a,d],[nca,nad,u as u32]);
        self.write(u,[d,b,c],[ndb,nbc,t as u32]);
        self.link(nad,a,d,t);
        self.link(nbc,b,c,u);
        Some(u)
    }
    /// Lawson's flips about a point just put in: each side opposite it that a held side is not and
    /// whose far corner lies in its circle, flipped, and the two sides that then face it looked at.
    fn legalize(&mut self,v: u32,start: &[usize]) {
        let mut stack: Vec<usize> = start.to_vec();
        let mut flips = 0usize;
        while let Some(t) = stack.pop() {
            let Some(i) = (0..3).find(|&i| self.tris[t][i] == v) else { continue };
            // the side opposite v
            let k = (i+1)%3;
            let (a,b) = (self.tris[t][k],self.tris[t][(k+1)%3]);
            let u = self.nbr[t][k];
            if u == NONE || self.held(a,b) { continue }
            let Some(j) = self.side(u as usize,b,a) else { continue };
            let d = self.tris[u as usize][(j+2)%3];
            if !in_circle(self.p(a),self.p(b),self.p(v),self.p(d)) { continue }
            if let Some(u) = self.flip(t,k) { stack.push(t); stack.push(u); flips += 1; }
            if flips > 10_000_000 { break }
        }
    }
    /// The triangles about `v`, in turn.
    fn fan(&self,v: u32) -> Vec<usize> {
        let first = self.about[v as usize] as usize;
        let mut out = vec![first];
        let mut t = first;
        loop {
            let i = (0..3).find(|&i| self.tris[t][i] == v).unwrap();
            let u = self.nbr[t][(i+2)%3];
            if u == NONE || u as usize == first { break }
            t = u as usize;
            out.push(t);
            if out.len() > self.tris.len() { break }
        }
        out
    }
    /// Whether the side from `a` to `b` (either way) is in the triangulation.
    fn has(&self,a: u32,b: u32) -> bool { self.fan(a).iter().any(|&t| self.tris[t].contains(&b)) }
    /// Side (a, b) recovered (Sloan): the sides crossing it, walked along it from `a`, each flipped
    /// once its quadrilateral is convex, its new diagonal looked at again while it still crosses.
    fn recover(&mut self,a: u32,b: u32) -> Result<(),String> {
        if self.has(a,b) { return Ok(()) }
        let (pa,pb) = (self.p(a),self.p(b));
        let crosses = |s: &Tri2,u: u32,w: u32| u != a && u != b && w != a && w != b
            && orient(pa,pb,s.p(u))*orient(pa,pb,s.p(w)) < 0 && orient(s.p(u),s.p(w),pa)*orient(s.p(u),s.p(w),pb) < 0;
        // the first side crossed: the side opposite `a` in the triangle about it that `ab` leaves by
        let mut queue: std::collections::VecDeque<(u32,u32)> = std::collections::VecDeque::new();
        let mut t = None;
        for f in self.fan(a) {
            let i = (0..3).find(|&i| self.tris[f][i] == a).unwrap();
            let (u,w) = (self.tris[f][(i+1)%3],self.tris[f][(i+2)%3]);
            if crosses(self,u,w) { t = Some((f,(i+1)%3)); break }
        }
        let Some((mut f,mut k)) = t else { return Err("a face's boundary passes through one of its points".into()) };
        loop {
            let (u,w) = (self.tris[f][k],self.tris[f][(k+1)%3]);
            queue.push_back((u,w));
            let g = self.nbr[f][k];
            if g == NONE { return Err("a face's boundary leaves its triangulation".into()) }
            let g = g as usize;
            let j = self.side(g,w,u).ok_or("a broken neighbour")?;
            let x = self.tris[g][(j+2)%3];
            if x == b { break }
            // on: the side of g that ab leaves by
            let (s1,s2) = ((j+1)%3,(j+2)%3);
            k = if crosses(self,self.tris[g][s1],self.tris[g][(s1+1)%3]) { s1 } else if crosses(self,self.tris[g][s2],self.tris[g][(s2+1)%3]) { s2 }
                else { return Err("a face's boundary passes through one of its points".into()) };
            f = g;
        }
        let mut guard = 0usize;
        while let Some((u,w)) = queue.pop_front() {
            guard += 1;
            if guard > 1_000_000 { return Err("a face's boundary could not be recovered in its triangulation".into()) }
            let Some(t) = self.fan(u).into_iter().find(|&t| self.side(t,u,w).is_some()) else { continue };
            let k = self.side(t,u,w).unwrap();
            match self.flip(t,k) {
                Some(_) => {
                    // the new diagonal: c (opposite in t) to d
                    let (c,d) = (self.tris[t][0],self.tris[t][2]);
                    if crosses(self,c,d) { queue.push_back((c,d)); }
                }
                None => queue.push_back((u,w)),
            }
        }
        if self.has(a,b) { Ok(()) } else { Err("a face's boundary could not be recovered in its triangulation".into()) }
    }
}

/// The constrained Delaunay triangulation of a face's loops: every corner inserted (Bowyer–Watson,
/// in a triangle about them all), every side of every loop recovered by flipping the sides that
/// cross it, and the triangles inside the loops marked — those left of a loop's side as it runs
/// (the outer loop counter-clockwise, its holes clockwise), and all they reach without crossing one.
fn cdt(pts: &[[f64;2]],loops: &[Vec<usize>]) -> Result<(Tri2,Vec<bool>),String> {
    let n = pts.len();
    let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
    for p in pts { for k in 0..2 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); } }
    let size = (hi[0]-lo[0]).max(hi[1]-lo[1]).max(1e-300);
    let mid = [(lo[0]+hi[0])/2.,(lo[1]+hi[1])/2.];
    let mut all = pts.to_vec();
    all.push([mid[0]-20.*size,mid[1]-20.*size]);
    all.push([mid[0]+20.*size,mid[1]-20.*size]);
    all.push([mid[0],mid[1]+20.*size]);
    let (s0,s1,s2) = (n as u32,n as u32+1,n as u32+2);
    let mut t = Tri2 {about:vec![0;all.len()],pts:all,tris:vec![[s0,s1,s2]],nbr:vec![[NONE;3]],
        held:std::collections::BTreeSet::new(),last:0,dirty:vec![true]};
    let mut mark = Vec::new();
    for i in 0..n { t.insert(i as u32,&mut mark,i as u32+1)?; }
    for l in loops { for k in 0..l.len() {
        let (a,b) = (l[k] as u32,l[(k+1)%l.len()] as u32);
        if a == b { continue }
        t.recover(a,b)?;
        t.held.insert((a.min(b),a.max(b)));
    } }
    // inside: left of each loop side as it runs, and what that reaches across sides not held
    let mut inside = vec![false;t.tris.len()];
    let mut stack = Vec::new();
    for l in loops { for k in 0..l.len() {
        let (a,b) = (l[k] as u32,l[(k+1)%l.len()] as u32);
        if a == b { continue }
        if let Some(f) = t.fan(a).into_iter().find(|&f| t.side(f,a,b).is_some()) { if !inside[f] { inside[f] = true; stack.push(f); } }
    } }
    while let Some(f) = stack.pop() {
        for k in 0..3 {
            let (a,b,u) = (t.tris[f][k],t.tris[f][(k+1)%3],t.nbr[f][k]);
            if u == NONE || t.held(a,b) || inside[u as usize] { continue }
            inside[u as usize] = true;
            stack.push(u as usize);
        }
    }
    if (0..t.tris.len()).any(|f| inside[f] && t.tris[f].iter().any(|&v| v >= s0)) {
        return Err("a face's loops do not close its triangulation".into())
    }
    Ok((t,inside))
}

/// Point `p` into triangle `t` of a face's triangulation, made Delaunay about it again; the new
/// triangles inside the face, their sag yet to be measured.
fn put(t2: &mut Tri2,inside: &mut Vec<bool>,sag: &mut Vec<f64>,t: usize,p: [f64;2]) {
    t2.pts.push(p);
    t2.about.push(t as u32);
    let v = (t2.pts.len()-1) as u32;
    let parts = t2.split(t,v);
    inside.resize(t2.tris.len(),true);
    sag.resize(t2.tris.len(),0.);
    t2.legalize(v,&parts);
}

/// Whether triangle `t` is thin in the scaled parameters: its height over its longest side under a
/// fifth of that side.
fn sliver(t2: &Tri2,t: [u32;3]) -> bool {
    let q = t.map(|v| t2.pts[v as usize]);
    let area2 = ((q[1][0]-q[0][0])*(q[2][1]-q[0][1])-(q[1][1]-q[0][1])*(q[2][0]-q[0][0])).abs();
    let longest = (0..3).map(|k| { let (a,b) = (q[k],q[(k+1)%3]); (b[0]-a[0]).dhypot(b[1]-a[1]) }).fold(0.,f64::max);
    area2 < 0.2*longest*longest
}

/// The distance from the surface at `uv` to the plane of the triangle `a`, `b`, `c` (space).
fn off(f: &Face,uv: Uv,a: V,b: V,c: V) -> f64 {
    let n = cross(sub(b,a),sub(c,a));
    let l = norm(n);
    if !(l > 0.) { return 0. }
    dot(sub(f.surface.point(uv),a),n).abs()/l
}

/// The mesh of the whole boundary within `bar` (a length) and `angular` (radians an edge's chords
/// may turn), and the greatest sag measured.
/// Where face `fi`'s loops, as their edges are sampled (`samples`: each edge's parameters first),
/// cross themselves in the face's parameters: for each pair of chords that cross and are not
/// neighbours along a loop, each one's edge and the parameter at its middle.
fn crossings(b: &Brep,fi: usize,samples: &[(Vec<f64>,Vec<u32>)]) -> Vec<(usize,f64)> {
    let f = &b.faces[fi];
    // every chord: its ends in the face's parameters, its edge and parameters, its loop and place
    let mut chords: Vec<([Uv;2],usize,[f64;2],usize,usize)> = Vec::new();
    let mut sizes = Vec::new();
    for (li,l) in f.loops.iter().enumerate() {
        let first = chords.len();
        for c in l {
            let e = c.edge as usize;
            if matches!(b.edges[e].curve,EdgeCurve::Degenerate) { continue }
            let ts = &samples[e].0;
            let at = |t: f64| c.pcurve.at(t,&b.edges[e],&f.surface,&b.vertices);
            let mut pairs: Vec<[f64;2]> = ts.windows(2).map(|w| [w[0],w[1]]).collect();
            if c.reversed { pairs.reverse(); }
            for [x,y] in pairs {
                let place = chords.len()-first;
                chords.push(([at(x),at(y)],e,[x,y],li,place));
            }
        }
        sizes.push(chords.len()-first);
    }
    // by their least u, each against those whose u reach overlaps it
    let lo = |c: &([Uv;2],usize,[f64;2],usize,usize)| c.0[0][0].min(c.0[1][0]);
    let hi = |c: &([Uv;2],usize,[f64;2],usize,usize)| c.0[0][0].max(c.0[1][0]);
    let mut order: Vec<usize> = (0..chords.len()).collect();
    order.sort_by(|&i,&j| lo(&chords[i]).total_cmp(&lo(&chords[j])));
    let neighbours = |i: usize,j: usize| {
        let (a,c) = (&chords[i],&chords[j]);
        if a.3 != c.3 { return false }
        let n = sizes[a.3];
        let d = a.4.abs_diff(c.4);
        d <= 1 || d == n-1
    };
    let mut out = Vec::new();
    for (k,&i) in order.iter().enumerate() {
        for &j in &order[k+1..] {
            if lo(&chords[j]) > hi(&chords[i]) { break }
            if neighbours(i,j) { continue }
            let ([p,q],[r,t]) = (chords[i].0,chords[j].0);
            if (p[1].max(q[1]) < r[1].min(t[1])) || (r[1].max(t[1]) < p[1].min(q[1])) { continue }
            if orient(p,q,r)*orient(p,q,t) < 0 && orient(r,t,p)*orient(r,t,q) < 0 {
                for c in [&chords[i],&chords[j]] { out.push((c.1,0.5*(c.2[0]+c.2[1]))); }
            }
        }
    }
    out
}

pub fn mesh(b: &Brep,bar: f64,angular: f64) -> Result<Mesh,String> { Ok(mesh_with(b,bar,angular,&|_| true,None)?.0) }

/// A sector's far side sampled as its near side turned (`brep::pattern`): the turn from one copy to
/// the next, each far vertex's near partner, and each far edge's (and whether it runs the same way).
pub(crate) struct Turned<'a> { pub turn: Rigid,pub vertex: &'a [Option<u32>],pub edge: &'a [Option<(u32,bool)>] }

/// The mesh of the faces `kept` says, and — where `turned` gives a far side — for each point, the
/// near side's point it stands for (the next copy's), every far edge's samples being its near
/// partner's turned, so neighbouring copies of a sector's mesh share their seam points by index.
pub(crate) fn mesh_with(b: &Brep,bar: f64,angular: f64,kept: &(dyn Fn(usize) -> bool + Sync),turned: Option<&Turned>)
    -> Result<(Mesh,Vec<Option<u32>>),String> {
    let mut m = Mesh::default();
    let mut alias: Vec<Option<u32>> = Vec::new();
    let vid: Vec<u32> = b.vertices.iter().map(|v| { m.pts.push(v.p); alias.push(None); (m.pts.len()-1) as u32 }).collect();
    let far_vertex = |v: usize| turned.and_then(|t| t.vertex[v]);
    for v in 0..b.vertices.len() { if let Some(w) = far_vertex(v) { alias[vid[v] as usize] = Some(vid[w as usize]); } }
    let far_edge = |e: usize| turned.and_then(|t| t.edge[e]);
    // every edge's samples, once: parameters and point ids (a far edge's after the rest)
    let mut samples: Vec<(Vec<f64>,Vec<u32>)> = vec![(Vec::new(),Vec::new());b.edges.len()];
    for e in (0..b.edges.len()).filter(|&e| far_edge(e).is_none()) {
        let ts = edge_params(b,e,bar,angular);
        let edge = &b.edges[e];
        let ids = ts.iter().enumerate().map(|(k,&t)| {
            if k == 0 { vid[edge.v[0] as usize] }
            else if k == ts.len()-1 { vid[edge.v[1] as usize] }
            else if matches!(edge.curve,EdgeCurve::Degenerate) { vid[edge.v[0] as usize] }
            else { m.pts.push(edge.point(t,&b.vertices)); alias.push(None); (m.pts.len()-1) as u32 }
        }).collect();
        samples[e] = (ts,ids);
    }
    if let Some(tn) = turned {
        for e in 0..b.edges.len() {
            let Some((near,same)) = far_edge(e) else { continue };
            let edge = &b.edges[e];
            let EdgeCurve::Curve(curve) = &edge.curve else { return Err("a far side's degenerate edge".into()) };
            let (_,nids) = samples[near as usize].clone();
            let mut order: Vec<u32> = nids;
            if !same { order.reverse(); }
            let last = order.len()-1;
            let (mut ts,mut ids) = (Vec::with_capacity(order.len()),Vec::with_capacity(order.len()));
            for (k,&nid) in order.iter().enumerate() {
                let p = tn.turn.point(m.pts[nid as usize]);
                if k == 0 { ts.push(edge.t[0]); ids.push(vid[edge.v[0] as usize]); continue }
                if k == last { ts.push(edge.t[1]); ids.push(vid[edge.v[1] as usize]); continue }
                ts.push(curve.inverse(p).clamp(edge.t[0],edge.t[1]));
                m.pts.push(p);
                alias.push(Some(nid));
                ids.push((m.pts.len()-1) as u32);
            }
            if ts.windows(2).any(|w| !(w[1] > w[0])) { return Err("a far side's edge, sampled as its near partner turned, does not run along it".into()) }
            samples[e] = (ts,ids);
        }
    }
    // a face's loops, sampled, must not cross themselves: a face thinner than its edges' chords sag
    // (a crescent between two arcs) would close no triangulation. The stretches that cross are split,
    // each edge's samples shared by its faces, until none do (a far side's are its partner's, and a
    // patterned sector's faces are left as they are)
    if turned.is_none() {
        for _ in 0..16 {
            let found: Vec<(usize,f64)> = crate::par::indices(b.faces.len(),|fi| if kept(fi) { crossings(b,fi,&samples) } else { Vec::new() })
                .into_iter().flatten().collect();
            if found.is_empty() { break }
            for (e,t) in found {
                let (ts,ids) = &mut samples[e];
                let k = ts.partition_point(|&x| x < t);
                if k == 0 || k >= ts.len() || ts[k] == t { continue }
                m.pts.push(b.edges[e].point(t,&b.vertices)); alias.push(None);
                ts.insert(k,t); ids.insert(k,(m.pts.len()-1) as u32);
            }
        }
    }
    // the least a parameter's scale is taken to be, where a surface's derivative vanishes
    let least = 1e-9*(1.+b.size());
    // points of a face nearer than this are one
    let weld = least;
    // each face side by side: its new points, its triangles (a new point's id marked `NEW`, its
    // place among them), its sag
    const NEW: u32 = 1<<31;
    let faces = crate::par::indices(b.faces.len(),|fi| -> Result<(Vec<V>,Vec<[u32;3]>,f64,usize),String> {
        if !kept(fi) { return Ok((Vec::new(),Vec::new(),0.,0)) }
        let f = &b.faces[fi];
        // the loops' corners: their places in the face's scaled, oriented parameters and their ids
        let mut pts: Vec<[f64;2]> = Vec::new();
        let mut uvs: Vec<Uv> = Vec::new();
        let mut ids: Vec<u32> = Vec::new();
        let mut loops: Vec<Vec<usize>> = Vec::new();
        let mut raw: Vec<Vec<(Uv,u32)>> = Vec::new();
        for l in &f.loops {
            let mut run = Vec::new();
            for c in l {
                let e = &b.edges[c.edge as usize];
                let (ts,is) = &samples[c.edge as usize];
                let mut these: Vec<(Uv,u32)> = ts.iter().zip(is).map(|(&t,&i)| (c.pcurve.at(t,e,&f.surface,&b.vertices),i)).collect();
                if c.reversed { these.reverse(); }
                run.extend_from_slice(&these[..these.len()-1]);
            }
            raw.push(run);
        }
        let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
        for p in raw.iter().flatten() { for k in 0..2 { lo[k] = lo[k].min(p.0[k]); hi[k] = hi[k].max(p.0[k]); } }
        let mid = [(lo[0]+hi[0])/2.,(lo[1]+hi[1])/2.];
        let (_,su,sv) = f.surface.d1(mid);
        let (ku,kv) = (norm(su).max(least),norm(sv).max(least));
        let place = |uv: Uv| [if f.reversed { -uv[0]*ku } else { uv[0]*ku },uv[1]*kv];
        for run in &raw {
            let mut l = Vec::new();
            for &(uv,i) in run {
                pts.push(place(uv));
                uvs.push(uv);
                ids.push(i);
                l.push(pts.len()-1);
            }
            loops.push(l);
        }
        let n = pts.len();
        let (mut t2,mut inside) = cdt(&pts,&loops).map_err(|e| format!("face {fi} ({}): {e}",f.surface.kind()))?;
        // refine where the surface stands off a triangle by more than the bar: its middle put in and
        // the triangulation made Delaunay about it again, looking only at triangles since written
        let unplace = |p: [f64;2]| [if f.reversed { -p[0]/ku } else { p[0]/ku },p[1]/kv];
        let curved = !matches!(f.surface,super::geom::Surface::Plane(_));
        let mut sag = vec![0_f64;t2.tris.len()];
        // points put in for turning alone, at most twice the triangles the loops made
        let (mut turned,budget) = (0usize,2*t2.tris.len()+64);
        if curved {
            for _round in 0..64 {
                let mut added = 0;
                for ti in 0..t2.tris.len() {
                    if !inside[ti] || !t2.dirty[ti] { continue }
                    t2.dirty[ti] = false;
                    let t = t2.tris[ti];
                    let corner = |k: usize| -> (Uv,V) {
                        let v = t[k] as usize;
                        let uv = if v < n { uvs[v] } else { unplace(t2.pts[v]) };
                        (uv,f.surface.point(uv))
                    };
                    let (a,b3,c) = (corner(0),corner(1),corner(2));
                    let centre = [(a.0[0]+b3.0[0]+c.0[0])/3.,(a.0[1]+b3.0[1]+c.0[1])/3.];
                    let mut s = off(f,centre,a.1,b3.1,c.1);
                    for (p,q,k) in [(&a,&b3,(0,1)),(&b3,&c,(1,2)),(&c,&a,(2,0))] {
                        if t2.held(t[k.0],t[k.1]) { continue }
                        s = s.max(off(f,[(p.0[0]+q.0[0])/2.,(p.0[1]+q.0[1])/2.],a.1,b3.1,c.1));
                    }
                    sag[ti] = s;
                    // and where the triangle turns from the surface's outward normal by more than
                    // `angular` at a corner, so a facet says which way the surface faces there: where
                    // the chart's metric changes fast across a triangle, one wound the right way in the
                    // scaled parameters can come out turned over in space, near the surface all the same.
                    // Not at a corner whose normal is degenerate (a pole's), and for a turn alone only
                    // while the triangle is longer than twice the bar (a turn finer than the sag is no
                    // shape), unless it is turned right over
                    let n = cross(sub(b3.1,a.1),sub(c.1,a.1));
                    let long = [distance(a.1,b3.1),distance(b3.1,c.1),distance(c.1,a.1)].into_iter().fold(0.,f64::max) > 2.*bar;
                    let out = if f.reversed { -1. } else { 1. };
                    let cosines: Vec<f64> = [a.0,b3.0,c.0,centre].iter().filter_map(|&q| {
                        let (_,su,sv) = f.surface.d1(q);
                        let m = cross(su,sv);
                        let l = norm(n)*norm(m);
                        (norm(m) > 1e-6*norm(su)*norm(sv) && l > 0.).then(|| out*dot(n,m)/l)
                    }).collect();
                    let over = cosines.iter().any(|&c| c < 0.);
                    let turns = over || long && cosines.iter().any(|&c| c.min(1.).dacos() > angular);
                    if t2.pts.len() >= 400_000 || !(s > bar || turns && turned < budget) { continue }
                    let middle = |q: [[f64;2];3]| [(q[0][0]+q[1][0]+q[2][0])/3.,(q[0][1]+q[1][1]+q[2][1])/3.];
                    let q = t.map(|v| t2.pts[v as usize]);
                    let (at,p) = if s > bar || !sliver(&t2,t) { (ti,middle(q)) } else {
                        // a sliver (its corners nearly in a line, often three samples of one boundary)
                        // only gets more from its middle: a point off its longest side not on the
                        // boundary, toward the neighbour across it, gives its corners a third to meet
                        let Some(k) = (0..3).filter(|&k| !t2.held(t[k],t[(k+1)%3]))
                            .max_by(|&i,&j| distance(corner(i).1,corner((i+1)%3).1).total_cmp(&distance(corner(j).1,corner((j+1)%3).1)))
                            else { continue };
                        let (a2,b2,c2) = (q[k],q[(k+1)%3],q[(k+2)%3]);
                        let (m,d) = ([(a2[0]+b2[0])/2.,(a2[1]+b2[1])/2.],[b2[0]-a2[0],b2[1]-a2[1]]);
                        let side = if (c2[0]-m[0])*(-d[1])+(c2[1]-m[1])*d[0] > 0. { -0.5 } else { 0.5 };
                        let p = [m[0]-d[1]*side,m[1]+d[0]*side];
                        match t2.locate(p).ok().filter(|&tp| inside[tp] && !t2.tris[tp].iter().any(|&w| t2.pts[w as usize] == p)) {
                            Some(tp) => (tp,p),
                            None => {
                                // no room beyond it (the boundary is there): a sliver turned right over
                                // is flipped across that side instead, where the quadrilateral allows
                                if over { t2.flip(ti,k); turned += 1; added += 1; }
                                continue
                            }
                        }
                    };
                    put(&mut t2,&mut inside,&mut sag,at,p);
                    if s <= bar { turned += 1; }
                    added += 1;
                }
                if added == 0 { break }
            }
        }
        let worst = (0..t2.tris.len()).filter(|&t| inside[t]).map(|t| sag[t]).fold(0.,f64::max);
        let facing = |v: u32| { let v = v as usize; if v < n { uvs[v] } else { unplace(t2.pts[v]) } };
        let turned = (0..t2.tris.len()).filter(|&ti| inside[ti] && {
            let [ua,ub,uc] = t2.tris[ti].map(facing);
            let (pa,pb,pc) = (f.surface.point(ua),f.surface.point(ub),f.surface.point(uc));
            let (_,su,sv) = f.surface.d1([(ua[0]+ub[0]+uc[0])/3.,(ua[1]+ub[1]+uc[1])/3.]);
            // (a triangle thinner than a tenth of the bar is flat within it, and its normal says
            // nothing of the surface's: a sliver a few nanometres off a boundary side, one with two
            // corners at a pole, which is dropped, or one whose inner corner a sheet's chart bends
            // past the straight side its parameters stand beside, 0.35 µm across on the pinion)
            let nf = cross(sub(pb,pa),sub(pc,pa));
            let longest = distance(pa,pb).max(distance(pb,pc)).max(distance(pc,pa));
            let o = dot(nf,cross(su,sv));
            norm(nf) > 0.1*bar*longest && if f.reversed { o > 0. } else { o < 0. }
        }).count();
        // the new points, and the triangles (the three corners of the triangle about them all come
        // after the face's own, and are never inside)
        let first_new = n+3;
        // a new point within `weld` of one already placed (a stretch of the surface's chart where
        // several parameters are one point) is that point, and the triangles it collapses dropped
        let cell = |p: V| p.map(|x| (x/weld).floor() as i64);
        let mut placed: std::collections::BTreeMap<[i64;3],Vec<(V,u32)>> = std::collections::BTreeMap::new();
        let near = |placed: &std::collections::BTreeMap<[i64;3],Vec<(V,u32)>>,p: V| -> Option<u32> {
            let c = cell(p);
            for dx in -1..=1 { for dy in -1..=1 { for dz in -1..=1 {
                if let Some(list) = placed.get(&[c[0]+dx,c[1]+dy,c[2]+dz]) {
                    if let Some(&(_,id)) = list.iter().find(|(q,_)| distance(*q,p) <= weld) { return Some(id) }
                }
            } } }
            None
        };
        for (k,&uv) in uvs.iter().enumerate() { let p = f.surface.point(uv); placed.entry(cell(p)).or_default().push((p,ids[k])); }
        let mut new_pts: Vec<V> = Vec::new();
        let mut renamed: Vec<u32> = Vec::with_capacity(t2.pts.len().saturating_sub(first_new));
        for v in first_new..t2.pts.len() {
            let p = f.surface.point(unplace(t2.pts[v]));
            let id = near(&placed,p).unwrap_or_else(|| {
                let id = NEW|new_pts.len() as u32;
                new_pts.push(p);
                placed.entry(cell(p)).or_default().push((p,id));
                id
            });
            renamed.push(id);
        }
        let id = |v: u32| if (v as usize) < n { ids[v as usize] } else { renamed[v as usize-first_new] };
        let tris = t2.tris.iter().zip(&inside).filter(|x| *x.1).map(|(t,_)| t.map(id)).collect();
        Ok((new_pts,tris,worst,turned))
    });
    for (fi,face) in faces.into_iter().enumerate() {
        let (pts,tris,sag,turned) = face?;
        let base = m.pts.len() as u32;
        alias.extend(pts.iter().map(|_| None));
        m.pts.extend(pts);
        m.sag = m.sag.max(sag);
        m.turned += turned;
        for t in tris {
            let tri = t.map(|v| if v & NEW != 0 { base+(v & !NEW) } else { v });
            // a triangle with two corners at one point (a pole) has no area
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] { continue }
            m.tris.push(tri);
            m.of.push(fi as u32);
        }
    }
    Ok((m,alias))
}
