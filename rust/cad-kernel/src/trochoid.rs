//! **A point's path under a planar motion of turns and slides, exactly** (#721 F1d): the curve a
//! point carried by turns about points of a plane at steady rates and slides along it at steady
//! speeds, composed and seen from one another, traces in that plane — a trochoid, the Wankel's
//! epitrochoid bore among them. Each turn is a rotation `e^{iωt}`, each slide a translation `v t`,
//! and a pose seen from another is the first's undone by the second's, so the path is a sum of
//! terms `(a + b t) e^{iωt}` in the plane's complex coordinate, each `a` and `b` a complex number:
//! its points and their derivatives in closed form at every order, its speed bounded over any
//! stretch. A drawing hands one to the kernel where its envelope of a point is one (the solver's
//! `generate::point_path`), so a prism over it is read exactly from its B-rep where a fit would not
//! be (the planar class's pocket, `swept::pocket`).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::geom::{Box3,Frame,Rigid,V};
use crate::turned::{Exact,J3,Jet,Num};
use crate::util::space::{distance,dot,sub};

/// One term `(a + b t) e^{iωt}`: `a` and `b` as `[re, im]`, `w` its rate (radians a unit of `t`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Term { pub a: [f64;2],pub b: [f64;2],pub w: f64 }

/// The curve `x + iy` in `frame`'s plane (its `x` and `y`), the sum of `terms` at `t`, over `span`.
#[derive(Clone,Debug,PartialEq)]
pub struct Trochoid { pub frame: Frame,pub terms: Vec<Term>,pub span: [f64;2] }

/// `p` times `q`, complex.
fn mul(p: [f64;2],q: [f64;2]) -> [f64;2] { [p[0]*q[0]-p[1]*q[1],p[0]*q[1]+p[1]*q[0]] }

impl Trochoid {
    /// Its point in the plane at `t` and its first two derivatives there.
    pub fn planar(&self,t: f64) -> [[f64;2];3] {
        let mut out = [[0.;2];3];
        for &Term {a,b,w} in &self.terms {
            let (s,c) = (w*t).dsin_cos();
            let e = [c,s];
            let iw = [0.,w];
            // f = a + b t, f' = b; z = f e, z' = (f' + iw f) e, z'' = (2 iw f' + (iw)² f) e
            let f = [a[0]+b[0]*t,a[1]+b[1]*t];
            let z = mul(f,e);
            let z1 = mul(add2(b,mul(iw,f)),e);
            let z2 = mul(add2(scale2(mul(iw,b),2.),scale2(f,-w*w)),e);
            for (k,x) in [z,z1,z2].into_iter().enumerate() { out[k] = add2(out[k],x) }
        }
        out
    }
    /// Its point at `t`, and its first two derivatives.
    pub fn d2(&self,t: f64) -> (V,V,V) {
        let [p,d,dd] = self.planar(t);
        let f = &self.frame;
        (f.at([p[0],p[1],0.]),f.dir([d[0],d[1],0.]),f.dir([dd[0],dd[1],0.]))
    }
    pub fn point(&self,t: f64) -> V { self.d2(t).0 }
    pub fn span(&self) -> [f64;2] { self.span }
    /// A bound on its speed over `[t0, t1]`: each term's `|b| + |ω| |a + b t|`, the latter at its
    /// largest at an end of the stretch, summed — never below it, `u` and `v` unit and square.
    fn speed_over(&self,[t0,t1]: [f64;2]) -> f64 {
        let reach = t0.abs().max(t1.abs());
        self.terms.iter().map(|&Term {a,b,w}| norm2(b)+w.abs()*(norm2(a)+norm2(b)*reach)).sum()
    }
    /// A bound on its speed over its span.
    pub fn speed(&self) -> f64 { self.speed_over(self.span) }
    /// A box about it over `[t0, t1]`, never smaller: its point at the middle grown by its speed's
    /// bound over half the stretch, each way.
    pub fn bounds(&self,[t0,t1]: [f64;2]) -> Box3 {
        let (t0,t1) = (t0.min(t1),t0.max(t1));
        let c = self.point(0.5*(t0+t1));
        let r = self.speed_over([t0,t1])*0.5*(t1-t0);
        (c.map(|x| x-r),c.map(|x| x+r))
    }
    /// The parameter of its point nearest `p`: the nearest of samples across its span, refined.
    pub fn inverse(&self,p: V) -> f64 {
        let [a,z] = self.span;
        let n = 512;
        let best = (0..=n).map(|k| a+(z-a)*k as f64/n as f64).min_by(|&x,&y| distance(self.point(x),p).total_cmp(&distance(self.point(y),p)))
            .unwrap_or(a);
        self.newton(p,best)
    }
    /// Newton on `(C − p)·C'` from `t`, held to its span.
    pub fn newton(&self,p: V,mut t: f64) -> f64 {
        let [a,z] = self.span;
        for _ in 0..32 {
            let (c,d,dd) = self.d2(t);
            let r = sub(c,p);
            let (g,h) = (dot(r,d),dot(d,d)+dot(r,dd));
            if !(h.abs() > 1e-300) { break }
            let next = (t-g/h).clamp(a.min(z),a.max(z));
            if (next-t).abs() <= 1e-15*(1.+t.abs()) { t = next; break }
            t = next;
        }
        t
    }
    /// The same curve carried by `m`.
    pub fn moved(&self,m: &Rigid) -> Trochoid { Trochoid {frame:self.frame.moved(m),..self.clone()} }
    /// The same curve over `span`.
    pub fn over(&self,span: [f64;2]) -> Trochoid { Trochoid {span,..self.clone()} }
}

impl Exact for Trochoid {
    fn frame(&self) -> &Frame { &self.frame }
    fn span(&self) -> [f64;2] { self.span }
    /// Its point at the jet `t` in its frame: each term's `(a + b t)(cos ωt + i sin ωt)`.
    fn local<T: Num>(&self,t: Jet<T>) -> J3<T> {
        let (mut x,mut y) = (Jet::of(0.),Jet::of(0.));
        for &Term {a,b,w} in &self.terms {
            let (s,c) = t.times(w).sin_cos();
            let (fr,fi) = (t.times(b[0])+Jet::of(a[0]),t.times(b[1])+Jet::of(a[1]));
            x = x+fr*c-fi*s;
            y = y+fr*s+fi*c;
        }
        [x,y,Jet::of(0.)]
    }
}

fn add2(p: [f64;2],q: [f64;2]) -> [f64;2] { [p[0]+q[0],p[1]+q[1]] }
fn scale2(p: [f64;2],k: f64) -> [f64;2] { [p[0]*k,p[1]*k] }
fn norm2(p: [f64;2]) -> f64 { p[0].dhypot(p[1]) }
