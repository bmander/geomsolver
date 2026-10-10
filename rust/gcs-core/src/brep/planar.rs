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
use crate::fmath::Det;
use std::f64::consts::{PI,TAU};

pub type P = [f64;2];

fn sub(a: P,b: P) -> P { [a[0]-b[0],a[1]-b[1]] }
fn dot(a: P,b: P) -> f64 { a[0]*b[0]+a[1]*b[1] }
fn cross(a: P,b: P) -> f64 { a[0]*b[1]-a[1]*b[0] }
fn norm(a: P) -> f64 { a[0].dhypot(a[1]) }
fn dist(a: P,b: P) -> f64 { norm(sub(a,b)) }

/// A step of a loop: a line from `a` to `b`, or the arc about `c` of radius `r` from angle `a0`
/// through the signed `sweep`.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Seg { Line { a: P,b: P },Arc { c: P,r: f64,a0: f64,sweep: f64 } }

impl Seg {
    pub fn point(&self,t: f64) -> P {
        match *self {
            Seg::Line {a,b} => [a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t],
            Seg::Arc {c,r,a0,sweep} => { let w = a0+sweep*t; [c[0]+r*w.dcos(),c[1]+r*w.dsin()] }
        }
    }
    pub fn start(&self) -> P { self.point(0.) }
    pub fn end(&self) -> P { self.point(1.) }
    /// The direction of travel at `t` (not of unit length).
    pub fn tangent(&self,t: f64) -> P {
        match *self {
            Seg::Line {a,b} => sub(b,a),
            Seg::Arc {r,a0,sweep,..} => { let w = a0+sweep*t; [-r*sweep*w.dsin(),r*sweep*w.dcos()] }
        }
    }
    pub fn length(&self) -> f64 { match *self { Seg::Line {a,b} => dist(a,b),Seg::Arc {r,sweep,..} => r*sweep.abs() } }
    pub fn reversed(&self) -> Seg {
        match *self { Seg::Line {a,b} => Seg::Line {a:b,b:a},Seg::Arc {c,r,a0,sweep} => Seg::Arc {c,r,a0:a0+sweep,sweep:-sweep} }
    }
    /// The stretch from `t0` to `t1`.
    fn part(&self,t0: f64,t1: f64) -> Seg {
        match *self {
            Seg::Line {..} => Seg::Line {a:self.point(t0),b:self.point(t1)},
            Seg::Arc {c,r,a0,sweep} => Seg::Arc {c,r,a0:a0+sweep*t0,sweep:sweep*(t1-t0)},
        }
    }
    /// Where along the step a point of its line or circle is (its fraction; past the ends outside
    /// `[0, 1]`, an arc's measured on the side its sweep runs).
    fn param(&self,p: P) -> f64 {
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
        }
    }
    /// The nearest distance from `p` to the step.
    fn distance(&self,p: P) -> f64 {
        let t = self.param(p).clamp(0.,1.);
        dist(self.point(t),p).min(dist(self.start(),p)).min(dist(self.end(),p))
    }
}

impl Seg {
    /// A signed measure of how far `p` is off the step's line or circle (zero on it).
    fn implicit(&self,p: P) -> f64 {
        match *self {
            Seg::Line {a,b} => { let d = sub(b,a); cross(d,sub(p,a))/norm(d).max(1e-300) }
            Seg::Arc {c,r,..} => dist(p,c)-r,
        }
    }
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
    fn param(&self,p: P) -> f64 { match self.chart { None => self.seg.param(p),Some(c) => self.seg.param(c.to(p)) } }
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
    if s.chart.is_none() && q.chart.is_none() { return meets(&s.seg,&q.seg,tol) }
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
        let ts: Vec<f64> = (0..=N).map(|i| i as f64/N as f64).collect();
        let gs: Vec<f64> = ts.iter().map(|&t| g(t)).collect();
        for i in 0..N {
            let (ga,gb) = (gs[i],gs[i+1]);
            if (ga < 0.) != (gb < 0.) {
                let t = crate::roots::bracketed_root(|t| Some(g(t)),ts[i],ga,ts[i+1],gb,1e-15);
                keep(a.point(t),&mut on_s,&mut on_q);
            } else if i+1 < N && gb.abs() < ga.abs() && gb.abs() < gs[i+2].abs() {
                // a sampled minimum on one side: a touch where it comes down to the curve
                let (t,v) = crate::roots::brent(&|t| g(t).abs(),ts[i],ts[i+2],1e-15,200,|_,_| false);
                if v <= tol { keep(a.point(t),&mut on_s,&mut on_q); }
            }
        }
    }
    (on_s,on_q)
}

/// Fractions along `s` and `q` where they meet: crossings and touches, and the ends of a stretch
/// they share.
fn meets(s: &Seg,q: &Seg,tol: f64) -> (Vec<f64>,Vec<f64>) {
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
    let points: Vec<P> = match (*s,*q) {
        (Seg::Line {a,b},Seg::Line {a: c,b: d}) => {
            let (u,v) = (sub(b,a),sub(d,c));
            let den = cross(u,v);
            if den.abs() <= 1e-14*norm(u)*norm(v) { vec![] } else {
                let t = cross(sub(c,a),v)/den;
                vec![[a[0]+u[0]*t,a[1]+u[1]*t]]
            }
        }
        (Seg::Line {a,b},Seg::Arc {c,r,..}) | (Seg::Arc {c,r,..},Seg::Line {a,b}) => {
            let d = sub(b,a);
            let f = sub(a,c);
            let (qa,qb,qc) = (dot(d,d),2.*dot(f,d),dot(f,f)-r*r);
            let disc = qb*qb-4.*qa*qc;
            let scale = qb*qb+4.*qa*qc.abs();
            if disc < -1e-12*scale { vec![] }
            else if disc <= 1e-12*scale { let t = -qb/(2.*qa); vec![[a[0]+d[0]*t,a[1]+d[1]*t]] }
            else {
                let w = disc.sqrt();
                [(-qb-w)/(2.*qa),(-qb+w)/(2.*qa)].iter().map(|&t| [a[0]+d[0]*t,a[1]+d[1]*t]).collect()
            }
        }
        (Seg::Arc {c: c1,r: r1,..},Seg::Arc {c: c2,r: r2,..}) => {
            let d = dist(c1,c2);
            if d <= tol && (r1-r2).abs() <= tol { vec![] }
            else if d > r1+r2+tol || d < (r1-r2).abs()-tol || d == 0. { vec![] }
            else {
                let a = (r1*r1-r2*r2+d*d)/(2.*d);
                let h = (r1*r1-a*a).max(0.).sqrt();
                let e = [(c2[0]-c1[0])/d,(c2[1]-c1[1])/d];
                let m = [c1[0]+a*e[0],c1[1]+a*e[1]];
                if h <= tol { vec![m] } else { vec![[m[0]-h*e[1],m[1]+h*e[0]],[m[0]+h*e[1],m[1]-h*e[0]]] }
            }
        }
    };
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
            (None,Seg::Arc {sweep,..}) => ((sweep.abs()/(TAU/64.)).ceil() as usize).max(2) };
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
                let keep = match other.along(m,d,tol) {
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
