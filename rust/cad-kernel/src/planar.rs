//! Regions of a plane bounded by lines and circular arcs, and their Booleans, exactly: every edge
//! split where the other region's edges cross or overlap it (lines and circles meet in closed
//! form), each piece kept or dropped by where its middle stands against the other region, and the
//! kept pieces chained into loops. A meridian of solids of revolution is such a region, so a blank
//! or a cutter made of revolutions about one line is its region turned once (`recipe::meridian`).
//!
//! A step may also be a line or arc of another plane carried into this one by a `Chart`: the
//! meridian of a solid of revolution about a line parallel to the one a half-plane turns about,
//! seen in that half-plane (`brep::section`). Such steps meet others by root-finding on the other's
//! implicit along them, and are tagged, as every step is, with the face each bounds.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use std::f64::consts::{PI,TAU};

pub use super::plane2::P;

use super::plane2::{self,Carrier,Touch,sub,dot,cross,norm,dist,left,scale,unit};

/// A step of a loop: a line from `a` to `b`, the arc about `c` of radius `r` from angle `a0`
/// through the signed `sweep`, or a conic's arc from `a` to `b`: the rational quadratic Bézier
/// over `a`, `m`, `b`, the middle weighed `w` and the ends one (a parabola where `w` is one, an
/// ellipse's arc below, a hyperbola's above) — what a chamfer's flat is between a line and a
/// circle (`corners::sum`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Seg { Line { a: P,b: P },Arc { c: P,r: f64,a0: f64,sweep: f64 },Conic { a: P,m: P,b: P,w: f64 } }

/// A rational quadratic Bézier over `p` (its middle weighed `w`, its ends one) at `t`: the point
/// and its derivative, by the quotient rule over `N = Σ kᵢ pᵢ` and `D = Σ kᵢ` — a conic's arc in
/// the plane (`Seg::Conic`) or turned into space (`solid::surface`'s meridian).
pub fn conic_at<const N: usize>(p: [[f64;N];3],w: f64,t: f64) -> ([f64;N],[f64;N]) {
    let s = 1.-t;
    let (k,dk) = ([s*s,2.*w*t*s,t*t],[-2.*s,2.*w*(1.-2.*t),2.*t]);
    let (d,dd) = (k[0]+k[1]+k[2],dk[0]+dk[1]+dk[2]);
    let n: [f64;N] = std::array::from_fn(|i| k[0]*p[0][i]+k[1]*p[1][i]+k[2]*p[2][i]);
    let dn: [f64;N] = std::array::from_fn(|i| dk[0]*p[0][i]+dk[1]*p[1][i]+dk[2]*p[2][i]);
    (n.map(|x| x/d),std::array::from_fn(|i| (dn[i]*d-n[i]*dd)/(d*d)))
}

/// A point's barycentric coordinates in the triangle `a`, `m`, `b`, and their gradients: affine in
/// the point, so a conic's implicit form over them is quadratic along any line.
fn barycentric(a: P,m: P,b: P,p: P) -> ([f64;3],[P;3]) {
    let det = cross(sub(m,a),sub(b,a));
    let grad = |u: P,v: P| [(u[1]-v[1])/det,(v[0]-u[0])/det];
    let (ga,gm,gb) = (grad(m,b),grad(b,a),grad(a,m));
    let at = |g: P,o: P| dot(g,sub(p,o));
    ([at(ga,m),at(gm,b),at(gb,a)],[ga,gm,gb])
}

impl Seg {
    pub fn point(&self,t: f64) -> P {
        match *self {
            Seg::Line {a,b} => [a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t],
            Seg::Arc {c,r,a0,sweep} => { let w = a0+sweep*t; [c[0]+r*w.dcos(),c[1]+r*w.dsin()] }
            // its ends exactly as written (the basis is 1, 0, 0 there): a vertex two steps share
            // stays one point
            Seg::Conic {a,m,b,w} => conic_at([a,m,b],w,t).0,
        }
    }
    pub fn start(&self) -> P { self.point(0.) }
    pub fn end(&self) -> P { self.point(1.) }
    /// The direction of travel at `t` (not of unit length).
    pub fn tangent(&self,t: f64) -> P {
        match *self {
            Seg::Line {a,b} => sub(b,a),
            Seg::Arc {r,a0,sweep,..} => { let w = a0+sweep*t; [-r*sweep*w.dsin(),r*sweep*w.dcos()] }
            Seg::Conic {a,m,b,w} => conic_at([a,m,b],w,t).1,
        }
    }
    pub fn length(&self) -> f64 {
        match *self {
            Seg::Line {a,b} => dist(a,b),
            Seg::Arc {r,sweep,..} => r*sweep.abs(),
            // eight-point Gauss–Legendre over sixteen stretches
            Seg::Conic {..} => (0..16).map(|k| crate::util::quadrature::GAUSS8.iter()
                .map(|&(x,w)| w*norm(self.tangent((k as f64+0.5+0.5*x)/16.))).sum::<f64>()/32.).sum(),
        }
    }
    pub fn reversed(&self) -> Seg {
        match *self {
            Seg::Line {a,b} => Seg::Line {a:b,b:a},
            Seg::Arc {c,r,a0,sweep} => Seg::Arc {c,r,a0:a0+sweep,sweep:-sweep},
            Seg::Conic {a,m,b,w} => Seg::Conic {a:b,m,b:a,w},
        }
    }
    /// The stretch from `t0` to `t1`.
    pub fn part(&self,t0: f64,t1: f64) -> Seg {
        match *self {
            Seg::Line {..} => Seg::Line {a:self.point(t0),b:self.point(t1)},
            Seg::Arc {c,r,a0,sweep} => Seg::Arc {c,r,a0:a0+sweep*t0,sweep:sweep*(t1-t0)},
            Seg::Conic {a,m,b,w} => {
                // the blossom of the homogeneous net at (t0, t0), (t0, t1), (t1, t1), back in
                // standard form (its ends weighed one, the middle the geometric mean's share)
                let h = [[a[0],a[1],1.],[w*m[0],w*m[1],w],[b[0],b[1],1.]];
                let blossom = |u: f64,v: f64| -> [f64;3] {
                    let k = [(1.-u)*(1.-v),(1.-u)*v+u*(1.-v),u*v];
                    std::array::from_fn(|i| k[0]*h[0][i]+k[1]*h[1][i]+k[2]*h[2][i])
                };
                let (p0,p1,p2) = (blossom(t0,t0),blossom(t0,t1),blossom(t1,t1));
                let at = |q: [f64;3]| [q[0]/q[2],q[1]/q[2]];
                let a = if t0 == 0. { a } else if t0 == 1. { b } else { at(p0) };
                let b2 = if t1 == 1. { b } else if t1 == 0. { a } else { at(p2) };
                Seg::Conic {a,m:at(p1),b:b2,w:p1[2]/(p0[2]*p2[2]).sqrt()}
            }
        }
    }
    /// Where along the step a point of its line or circle is (its fraction; past the ends outside
    /// `[0, 1]`, an arc's measured on the side its sweep runs).
    pub fn param(&self,p: P) -> f64 {
        match *self {
            Seg::Line {a,b} => { let d = sub(b,a); dot(sub(p,a),d)/dot(d,d) }
            Seg::Arc {c,a0,sweep,..} => {
                let phi = (p[1]-c[1]).datan2(p[0]-c[0]);
                let mut delta = (phi-a0)*sweep.signum();
                delta = delta.rem_euclid(TAU);
                // a point just before the start reads as a whole turn on: read it as just before
                if delta > PI+sweep.abs()/2. { delta -= TAU; }
                delta/sweep.abs()
            }
            Seg::Conic {a,m,b,..} => {
                // (1 − t)² : 2wt(1 − t) : t² are the point's barycentric coordinates in the
                // control triangle, so `t / (1 − t)` is the root of their ends' ratio, turned
                // negative off the arc (the middle's coordinate below nothing) — or, a hyperbola's
                // (`w > 1`), on its other branch, where the weights' sum is below nothing and so
                // are both ends' coordinates
                let ([l0,l1,l2],_) = barycentric(a,m,b,p);
                let (s0,s2) = (l0.abs().sqrt(),l2.abs().sqrt());
                if l1 >= 0. && !(l0 < 0. && l2 < 0.) { s2/(s0+s2).max(1e-300) } else { let k = s2/s0.max(1e-300); -k/(1.-k) }
            }
        }
    }
    /// The nearest distance from `p` to the step.
    fn distance(&self,p: P) -> f64 {
        let t = match self { Seg::Conic {..} => self.nearest(p),_ => self.param(p).clamp(0.,1.) };
        dist(self.point(t),p).min(dist(self.start(),p)).min(dist(self.end(),p))
    }
    /// The fraction of the step's nearest point to `p`: of 64 samples the nearest, refined by
    /// Brent's minimisation of the squared distance between its neighbours.
    pub fn nearest(&self,p: P) -> f64 {
        const N: usize = 64;
        let d2 = |t: f64| { let q = sub(self.point(t),p); dot(q,q) };
        let k = (0..=N).min_by(|&i,&j| d2(i as f64/N as f64).total_cmp(&d2(j as f64/N as f64))).unwrap();
        let (lo,hi) = ((k.max(1)-1) as f64/N as f64,(k+1).min(N) as f64/N as f64);
        crate::util::roots::brent(&d2,lo,hi,1e-15,200,|_,_| false).at
    }
}

impl Seg {
    /// A signed measure of how far `p` is off the step's line or circle (zero on it).
    pub fn implicit(&self,p: P) -> f64 {
        match *self {
            Seg::Line {a,b} => { let d = sub(b,a); cross(d,sub(p,a))/norm(d).max(1e-300) }
            Seg::Arc {c,r,..} => dist(p,c)-r,
            // `λ₁² − 4w²λ₀λ₂` over its gradient's length: zero on the conic, a length off it to
            // first order, positive on the side away from the middle control point
            Seg::Conic {a,m,b,w} => { let (f,g) = conic_form(a,m,b,w,p); -f/norm(g).max(1e-300) }
        }
    }
    /// The gradient of `implicit` at `p`: exact for a line and an arc; for a conic its form's
    /// gradient made unit, exact on the conic (where a touch is) and to first order off it.
    fn gradient(&self,p: P) -> P {
        match *self {
            Seg::Line {a,b} => { let d = sub(b,a); scale(left(d),1./norm(d).max(1e-300)) }
            Seg::Arc {c,..} => { let q = sub(p,c); if norm(q) > 0. { unit(q) } else { [1.,0.] } }
            Seg::Conic {a,m,b,w} => { let (_,g) = conic_form(a,m,b,w,p); scale(g,-1./norm(g).max(1e-300)) }
        }
    }
}

/// A conic's form `λ₁² − 4w²λ₀λ₂` at `p` over its control points' barycentric coordinates, and its
/// gradient (`Seg::implicit`, `Seg::gradient`).
fn conic_form(a: P,m: P,b: P,w: f64,p: P) -> (f64,P) {
    let ([l0,l1,l2],[g0,g1,g2]) = barycentric(a,m,b,p);
    (l1*l1-4.*w*w*l0*l2,std::array::from_fn(|i| 2.*l1*g1[i]-4.*w*w*(l2*g0[i]+l0*g2[i])))
}

/// A plane's coordinates `(r, z)` about a line parallel to the one a half-plane turns about, read
/// from the half-plane's `(s, z)`: `r² = s² + 2 β s + δ²` (`β`, `δ` the other line's offset along
/// the half-plane's direction and its whole offset, square to both lines) and `z` moved by `shift`
/// and turned by `sign`. One to one where `s > −β`, the side a meridian of the other line is seen on.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Chart { pub beta: f64,pub delta2: f64,pub shift: f64,pub sign: f64 }
impl Chart {
    pub fn to(&self,p: P) -> P { [(p[0]*p[0]+2.*self.beta*p[0]+self.delta2).max(0.).sqrt(),self.sign*(p[1]+self.shift)] }
    pub fn from(&self,q: P) -> P { [-self.beta+(q[0]*q[0]-self.delta2+self.beta*self.beta).max(0.).sqrt(),self.sign*q[1]-self.shift] }
    /// `ds/dr` at the chart's `r`.
    fn ds_dr(&self,r: f64) -> f64 { r/(r*r-self.delta2+self.beta*self.beta).max(1e-300).sqrt() }
}

/// A step of a region: a line or arc of this plane, or of a chart's plane carried into this one,
/// tagged with the face it bounds.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Step { pub seg: Seg,pub chart: Option<Chart>,pub tag: u32 }

impl From<Seg> for Step { fn from(seg: Seg) -> Step { Step {seg,chart:None,tag:0} } }

impl Step {
    pub fn point(&self,t: f64) -> P { let p = self.seg.point(t); match self.chart { None => p,Some(c) => c.from(p) } }
    pub fn start(&self) -> P { self.point(0.) }
    pub fn end(&self) -> P { self.point(1.) }
    pub fn tangent(&self,t: f64) -> P {
        let d = self.seg.tangent(t);
        match self.chart { None => d,Some(c) => [c.ds_dr(self.seg.point(t)[0])*d[0],c.sign*d[1]] }
    }
    pub fn length(&self) -> f64 {
        match self.chart { None => self.seg.length(),Some(_) => (0..16).map(|k| dist(self.point(k as f64/16.),self.point((k+1) as f64/16.))).sum() }
    }
    pub fn reversed(&self) -> Step { Step {seg:self.seg.reversed(),..*self} }
    fn part(&self,t0: f64,t1: f64) -> Step { Step {seg:self.seg.part(t0,t1),..*self} }
    pub(crate) fn param(&self,p: P) -> f64 { match self.chart { None => self.seg.param(p),Some(c) => self.seg.param(c.to(p)) } }
    fn distance(&self,p: P) -> f64 {
        match self.chart {
            None => self.seg.distance(p),
            Some(_) => { let t = self.param(p).clamp(0.,1.); dist(self.point(t),p).min(dist(self.start(),p)).min(dist(self.end(),p)) }
        }
    }
    /// This step continued by `next` as one, where `next` is the rest of the same line or arc of
    /// the same face (a step a Boolean split where the other region only touched it).
    pub fn joined(&self,next: &Step) -> Option<Step> {
        if self.tag != next.tag || self.chart != next.chart { return None }
        let close = |a: f64,b: f64| (a-b).abs() <= 1e-12*(1.+a.abs().max(b.abs()));
        let seg = match (self.seg,next.seg) {
            (Seg::Line {a,b},Seg::Line {a: c,b: d}) => {
                let (u,v) = (sub(b,a),sub(d,c));
                if !(close(b[0],c[0]) && close(b[1],c[1])) || cross(u,v).abs() > 1e-12*norm(u)*norm(v) || dot(u,v) <= 0. { return None }
                Seg::Line {a,b:d}
            }
            (Seg::Arc {c,r,a0,sweep},Seg::Arc {c: c2,r: r2,a0: b0,sweep: s2}) => {
                if !(close(c[0],c2[0]) && close(c[1],c2[1]) && close(r,r2)) || sweep.signum() != s2.signum() { return None }
                let end = a0+sweep;
                if ((b0-end+PI).rem_euclid(TAU)-PI).abs() > 1e-12 { return None }
                Seg::Arc {c,r,a0,sweep:sweep+s2}
            }
            _ => return None,
        };
        Some(Step {seg,..*self})
    }
    fn implicit(&self,p: P) -> f64 { match self.chart { None => self.seg.implicit(p),Some(c) => self.seg.implicit(c.to(p)) } }
    /// The gradient of `implicit` at `p`, through the chart's `(r, z)` where it has one.
    fn gradient(&self,p: P) -> P {
        match self.chart {
            None => self.seg.gradient(p),
            Some(c) => {
                let q = c.to(p);
                let g = self.seg.gradient(q);
                [if q[0] > 0. { g[0]*(p[0]+c.beta)/q[0] } else { 0. },g[1]*c.sign]
            }
        }
    }
    /// The outward normal at `t` in the step's own plane (`(r, z)` of its chart; its material on
    /// the left of its plain travel), unit.
    pub fn own_normal(&self,t: f64) -> P {
        let d = self.seg.tangent(t);
        let l = norm(d).max(1e-300);
        // (a chart turned by `sign` reverses the step's sense in its own plane)
        let f = self.chart.map_or(1.,|c| c.sign);
        [f*d[1]/l,-f*d[0]/l]
    }
}

/// Fractions along `s` and `q` where they meet: crossings and touches, and the ends of a stretch
/// they share. Plain steps meet in closed form; a charted one by roots of the other's implicit
/// along it (and of its implicit along the other).
fn meets_steps(s: &Step,q: &Step,tol: f64) -> (Vec<f64>,Vec<f64>) {
    let conic = |x: &Step| matches!(x.seg,Seg::Conic {..});
    if s.chart.is_none() && q.chart.is_none() && !conic(s) && !conic(q) { return meets(&s.seg,&q.seg,tol) }
    let (mut on_s,mut on_q) = (Vec::new(),Vec::new());
    let within = |t: f64,st: &Step| { let l = st.length().max(1e-300); t > -tol/l && t < 1.+tol/l };
    let keep = |p: P,on_s: &mut Vec<f64>,on_q: &mut Vec<f64>| {
        let (ts,tq) = (s.param(p),q.param(p));
        if within(ts,s) && within(tq,q) && dist(s.point(ts.clamp(0.,1.)),p) <= tol && dist(q.point(tq.clamp(0.,1.)),p) <= tol {
            on_s.push(ts.clamp(0.,1.)); on_q.push(tq.clamp(0.,1.));
        }
    };
    for p in [s.start(),s.end()] { if q.distance(p) <= tol { keep(p,&mut on_s,&mut on_q); } }
    for p in [q.start(),q.end()] { if s.distance(p) <= tol { keep(p,&mut on_s,&mut on_q); } }
    for (a,b) in [(s,q),(q,s)] {
        const N: usize = 96;
        let g = |t: f64| b.implicit(a.point(t));
        let rate = |t: f64| dot(b.gradient(a.point(t)),a.tangent(t));
        let ts: Vec<f64> = (0..=N).map(|i| i as f64/N as f64).collect();
        let gs: Vec<f64> = ts.iter().map(|&t| g(t)).collect();
        for r in crate::util::roots::sampled_roots(g,rate,&ts,&gs,tol,1e-15) { keep(a.point(r.at),&mut on_s,&mut on_q); }
    }
    (on_s,on_q)
}

/// Fractions along `s` and `q` where they meet: crossings and touches, and the ends of a stretch
/// they share.
pub fn meets(s: &Seg,q: &Seg,tol: f64) -> (Vec<f64>,Vec<f64>) {
    let (mut on_s,mut on_q) = (Vec::new(),Vec::new());
    let within = |t: f64,seg: &Seg| { let l = seg.length().max(1e-300); t > -tol/l && t < 1.+tol/l };
    let keep = |p: P,on_s: &mut Vec<f64>,on_q: &mut Vec<f64>| {
        let (ts,tq) = (s.param(p),q.param(p));
        if within(ts,s) && within(tq,q) && dist(s.point(ts.clamp(0.,1.)),p) <= tol && dist(q.point(tq.clamp(0.,1.)),p) <= tol {
            on_s.push(ts.clamp(0.,1.)); on_q.push(tq.clamp(0.,1.));
        }
    };
    // a shared stretch: each one's ends that lie on the other
    for p in [s.start(),s.end()] { if q.distance(p) <= tol { keep(p,&mut on_s,&mut on_q); } }
    for p in [q.start(),q.end()] { if s.distance(p) <= tol { keep(p,&mut on_s,&mut on_q); } }
    let carrier = |s: &Seg| match *s {
        Seg::Line {a,b} => Carrier::Line {p: a,d: plane2::unit(sub(b,a))},
        Seg::Arc {c,r,..} => Carrier::Circle {c,r},
        Seg::Conic {..} => unreachable!("a conic meets by `meets_steps`' roots"),
    };
    let points = plane2::meet(carrier(s),carrier(q),Touch::Gap(tol)).unwrap_or_default();
    for p in points { keep(p,&mut on_s,&mut on_q); }
    (on_s,on_q)
}

/// A region: closed loops, each with its material on its left (outer loops counter-clockwise,
/// holes clockwise).
#[derive(Clone,Debug,Default)]
pub struct Region { pub loops: Vec<Vec<Step>> }

/// What a Boolean does with two regions.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Op { Union,Cut,Common }

impl Region {
    /// A region of plain lines and arcs (`new`), tagged nothing.
    pub fn plain(loops: Vec<Vec<Seg>>) -> Region { Region::new(loops.into_iter().map(|l| l.into_iter().map(Step::from).collect()).collect()) }
    /// A region from closed loops in any orientation: each turned so that its material is on its
    /// left, by how many of the others hold it.
    pub fn new(loops: Vec<Vec<Step>>) -> Region {
        // each loop's steps walked end to end first (a profile lists its edges in any order and
        // direction)
        let loops: Vec<Vec<Step>> = loops.into_iter().map(|l| {
            if l.is_empty() { return l }
            let mut rest = l[1..].to_vec();
            let mut walk = vec![l[0]];
            while !rest.is_empty() {
                let end = walk.last().unwrap().end();
                let Some(k) = (0..rest.len()).min_by(|&i,&j| {
                    let d = |s: &Step| dist(s.start(),end).min(dist(s.end(),end));
                    d(&rest[i]).total_cmp(&d(&rest[j]))
                }) else { break };
                let s = rest.swap_remove(k);
                walk.push(if dist(s.start(),end) <= dist(s.end(),end) { s } else { s.reversed() });
            }
            walk
        }).collect();
        let polys: Vec<Vec<P>> = loops.iter().map(|l| polygon(l)).collect();
        let loops = loops.iter().enumerate().map(|(i,l)| {
            let area = signed_area(&polys[i]);
            let depth = (0..loops.len()).filter(|&j| j != i && winding(&polys[j],interior_point(&polys[i]))).count();
            let ccw = depth % 2 == 0;
            if (area > 0.) == ccw { l.clone() } else { l.iter().rev().map(Step::reversed).collect() }
        }).collect();
        Region {loops}
    }
    /// A plain region of a chart's plane carried into this one, each step tagged by `tag` (its
    /// loop and its place in it).
    pub fn charted(&self,chart: Chart,tag: &dyn Fn(usize,usize) -> u32) -> Region {
        Region::new(self.loops.iter().enumerate().map(|(li,l)| l.iter().enumerate().map(|(k,s)| Step {seg:s.seg,chart:Some(chart),tag:tag(li,k)}).collect()).collect())
    }
    /// The same region, each step tagged by `tag` (its loop and its place in it).
    pub fn tagged(&self,tag: &dyn Fn(usize,usize) -> u32) -> Region {
        Region {loops:self.loops.iter().enumerate().map(|(li,l)| l.iter().enumerate().map(|(k,s)| Step {tag:tag(li,k),..*s}).collect()).collect()}
    }
    /// Whether `p` is inside: the parity of the steps a ray from it crosses, each line and circle
    /// met exactly (the ray at a slope no step's end lies on but by accident), a charted step's
    /// chords 1/256 of it apart.
    pub fn contains(&self,p: P) -> bool {
        let d = [0.8939966636005579,0.4480736161291702];
        let ray_line = |a: P,b: P,half_open: bool| -> usize {
            let e = sub(b,a);
            let den = cross(d,e);
            if den == 0. { return 0 }
            let w = sub(a,p);
            let (along,t) = (cross(w,e)/den,cross(w,d)/den);
            usize::from(along > 0. && if half_open { (0. ..1.).contains(&t) } else { (0. ..=1.).contains(&t) })
        };
        let mut crossings = 0;
        for st in self.loops.iter().flatten() {
            if st.chart.is_some() {
                const N: usize = 256;
                crossings += (0..N).map(|k| ray_line(st.point(k as f64/N as f64),st.point((k+1) as f64/N as f64),true)).sum::<usize>();
                continue
            }
            let s = &st.seg;
            crossings += match *s {
                Seg::Line {a,b} => ray_line(a,b,true),
                Seg::Arc {c,r,..} => {
                    let f = sub(p,c);
                    let (qb,qc) = (dot(f,d),dot(f,f)-r*r);
                    let disc = qb*qb-qc;
                    if disc <= 0. { 0 } else {
                        let w = disc.sqrt();
                        [-qb-w,-qb+w].into_iter().filter(|&along| along > 0. && {
                            let t = s.param([p[0]+d[0]*along,p[1]+d[1]*along]);
                            (0. ..1.).contains(&t)
                        }).count()
                    }
                }
                // along the ray the barycentric coordinates are affine, so the implicit form is a
                // quadratic in the distance along it
                Seg::Conic {a,m,b,w} => {
                    let ([l0,l1,l2],[g0,g1,g2]) = barycentric(a,m,b,p);
                    let (d0,d1,d2) = (dot(g0,d),dot(g1,d),dot(g2,d));
                    let ww = 4.*w*w;
                    let roots = crate::util::roots::quadratic(d1*d1-ww*d0*d2,2.*l1*d1-ww*(l0*d2+l2*d0),
                        l1*l1-ww*l0*l2);
                    roots.into_iter().filter(|&along| along > 0. && {
                        let t = s.param([p[0]+d[0]*along,p[1]+d[1]*along]);
                        (0. ..1.).contains(&t)
                    }).count()
                }
            };
        }
        crossings % 2 == 1
    }
    /// The step of the boundary within `tol` of `p` and whether it runs along `d` there.
    fn along(&self,p: P,d: P,tol: f64) -> Option<bool> {
        let mut best: Option<(f64,bool)> = None;
        for s in self.loops.iter().flatten() {
            let e = s.distance(p);
            if e <= tol && best.is_none_or(|b| e < b.0) { best = Some((e,dot(s.tangent(s.param(p).clamp(0.,1.)),d) > 0.)); }
        }
        best.map(|b| b.1)
    }
    pub fn area(&self) -> f64 { self.loops.iter().map(|l| signed_area(&polygon(l))).sum() }
    /// The loops, the one enclosing most first (a profile's outer loop, then its holes).
    pub fn outer_first(&self) -> Vec<&[Step]> {
        let mut loops: Vec<(f64,&[Step])> = self.loops.iter().map(|l| (signed_area(&polygon(l)).abs(),&l[..])).collect();
        loops.sort_by(|a,b| b.0.total_cmp(&a.0));
        loops.into_iter().map(|l| l.1).collect()
    }
    /// A ball's meridian: the half-disc of radius `r` about `[0, z]`, from the axis round through
    /// the half-plane and back along it.
    pub fn half_disc(z: f64,r: f64) -> Region {
        Region::plain(vec![vec![Seg::Arc {c:[0.,z],r,a0:-std::f64::consts::FRAC_PI_2,sweep:std::f64::consts::PI},
            Seg::Line {a:[0.,z+r],b:[0.,z-r]}]])
    }
}

/// A loop's steps as a polygon, each arc in steps of at most a sixty-fourth of a turn, a charted
/// step in 32.
fn polygon(l: &[Step]) -> Vec<P> {
    l.iter().flat_map(|s| {
        let n = match (s.chart,s.seg) { (Some(_),_) => 32,(None,Seg::Line {..}) => 1,
            (None,Seg::Arc {sweep,..}) => ((sweep.abs()/(TAU/64.)).ceil() as usize).max(2),
            (None,Seg::Conic {..}) => 32 };
        (0..n).map(move |k| s.point(k as f64/n as f64))
    }).collect()
}
fn signed_area(p: &[P]) -> f64 { (0..p.len()).map(|i| cross(p[i],p[(i+1)%p.len()])).sum::<f64>()/2. }
fn winding(poly: &[P],q: P) -> bool {
    let mut inside = false;
    for i in 0..poly.len() {
        let (a,b) = (poly[i],poly[(i+1)%poly.len()]);
        if (a[1] <= q[1]) != (b[1] <= q[1]) && a[0]+(q[1]-a[1])/(b[1]-a[1])*(b[0]-a[0]) > q[0] { inside = !inside; }
    }
    inside
}
/// A point just inside a polygon's first edge (on its left if counter-clockwise, either way a
/// point of no other loop's boundary).
fn interior_point(p: &[P]) -> P {
    let (a,b) = (p[0],p[1]);
    let m = [(a[0]+b[0])/2.,(a[1]+b[1])/2.];
    let d = sub(b,a);
    let l = norm(d).max(1e-300);
    let e = 1e-7*l;
    [m[0]-d[1]/l*e,m[1]+d[0]/l*e]
}

/// `a` combined with `b` by `op`, within `tol` (a length).
pub fn boolean(a: &Region,b: &Region,op: Op,tol: f64) -> Result<Region,String> {
    let regions = [a,b];
    // every step split where the other region's steps meet it
    let mut cuts: [Vec<Vec<f64>>;2] = [a.loops.iter().flatten().map(|_| vec![0.,1.]).collect(),b.loops.iter().flatten().map(|_| vec![0.,1.]).collect()];
    let steps: [Vec<Step>;2] = [a.loops.iter().flatten().copied().collect(),b.loops.iter().flatten().copied().collect()];
    for (i,s) in steps[0].iter().enumerate() {
        for (j,q) in steps[1].iter().enumerate() {
            let (ts,tq) = meets_steps(s,q,tol);
            cuts[0][i].extend(ts);
            cuts[1][j].extend(tq);
        }
    }
    // the pieces, each with where it stands against the other region
    let mut kept: Vec<Step> = Vec::new();
    for k in 0..2 {
        let other = regions[1-k];
        for (i,s) in steps[k].iter().enumerate() {
            let mut ts = cuts[k][i].clone();
            ts.sort_by(f64::total_cmp);
            let l = s.length().max(1e-300);
            ts.dedup_by(|x,y| (*x-*y)*l <= tol);
            if let Some(last) = ts.last_mut() { *last = 1.; }
            for w in ts.windows(2) {
                let piece = s.part(w[0],w[1]);
                if piece.length() <= tol { continue }
                let (m,d) = (piece.point(0.5),piece.tangent(0.5));
                // on the other's boundary only if all of it is: its middle within `tol` of a step
                // the other's boundary is near tangent to, a stretch its ends are not, is no
                // stretch the two share
                let on = |q: P| other.loops.iter().flatten().any(|o| o.distance(q) <= tol);
                let along = other.along(m,d,tol).filter(|_| on(piece.start()) && on(piece.end()));
                let keep = match along {
                    // on the other's boundary: one copy of a stretch both keep, from `a`
                    Some(same) => k == 0 && match op { Op::Union | Op::Common => same,Op::Cut => !same },
                    None => {
                        let inside = other.contains(m);
                        match (op,k) { (Op::Union,_) => !inside,(Op::Common,_) => inside,(Op::Cut,0) => !inside,(Op::Cut,_) => inside }
                    }
                };
                if keep { kept.push(if op == Op::Cut && k == 1 { piece.reversed() } else { piece }); }
            }
        }
    }
    // chained into loops: at a point where several continue, the one turning least clockwise from
    // the way back along the step arriving
    let mut used = vec![false;kept.len()];
    let mut loops = Vec::new();
    for start in 0..kept.len() {
        if used[start] { continue }
        used[start] = true;
        let mut l = vec![kept[start]];
        loop {
            let last = *l.last().unwrap();
            if dist(last.end(),l[0].start()) <= tol { break }
            let back = last.tangent(1.);
            let back = [-back[0],-back[1]];
            let mut best: Option<(f64,usize)> = None;
            for (i,s) in kept.iter().enumerate() {
                if used[i] || dist(s.start(),last.end()) > tol { continue }
                let out = s.tangent(0.);
                let mut turn = cross(back,out).datan2(dot(back,out));
                if turn <= 0. { turn += TAU; }
                if best.is_none_or(|b| turn < b.0) { best = Some((turn,i)); }
            }
            let Some((_,i)) = best else {
                if dist(last.end(),l[0].start()) <= tol { break }
                return Err("a planar Boolean's pieces do not close".into())
            };
            used[i] = true;
            l.push(kept[i]);
        }
        // a step split where nothing changed, joined again
        let mut joined: Vec<Step> = Vec::new();
        for st in l {
            match joined.last().and_then(|last| last.joined(&st)) { Some(j) => *joined.last_mut().unwrap() = j,None => joined.push(st) }
        }
        while joined.len() > 1 {
            match joined.last().unwrap().joined(&joined[0]) { Some(j) => { joined[0] = j; joined.pop(); } None => break }
        }
        loops.push(joined);
    }
    Ok(Region {loops})
}
