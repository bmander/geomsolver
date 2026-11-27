//! **A curve on a surface carried along the surface's normal, exactly** (`Curve::Parallel`): every
//! point `q(t)` of a curve lying on a surface `base` moved `d` along its unit normal at its foot,
//! `q(t) + d N(u(t), v(t))`, onto `under`, the surface standing `d` off `base` over the same
//! parameters (`Surface::normal_moved`: a cone's, a torus's, a B-spline sheet's offset,
//! `Surface::Offset`) — whose edge moved with its face (`offset::carried_sweep`) is no curve of
//! the kernel's own kinds (a parabola's offset is of degree six and has no rational parameter, an
//! ellipse's on a cone is no conic). A face moved keeps its pcurves and the curve keeps its
//! parameter: `t` here is `t` on the curve it was carried from, and the ruling joining the two at
//! one `t` is the normal there — what a wall the edge sweeps is ruled along (`geom::Blend`). Its
//! point is `q + (P − B)` at the foot `(u, v)` of `q`, `P` the offset's point and `B` the base's,
//! so it stands exactly `d` off `q` along the normal however finely the foot settles; its rate
//! `q' + (P_u − B_u) u' + (P_v − B_v) v'`, `(u', v')` the curve's rate read in the parameters, the
//! offset's partials Weingarten's (`Surface::d1`); its second rate a central difference of the
//! first, as a loft's rail's. The foot and its rate are the face's pcurve's where it has one in
//! closed form (`Foot`, a line or a curve of the parameters), and otherwise `q` inverted onto
//! `base` and its rate solved there.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::geom::{Box3,Curve,Rigid,Surface,Uv,V};
use std::sync::Arc;
use crate::util::space::{add,norm,scale,sub};

/// Where a curve stands in its surface's parameters as its own parameter runs, as the face it
/// bounds says (`topo::Pcurve`): a line, `a` at `t[0]` and `b` at `t[1]` (read on past them along
/// it), or a curve of the parameters read at the curve's own parameter.
#[derive(Clone,Debug,PartialEq)]
pub enum Foot { Line { t: [f64;2],a: Uv,b: Uv },Curve(Arc<Curve>) }

impl Foot {
    /// The foot at `t` and its rate.
    fn d1(&self,t: f64) -> (Uv,Uv) {
        match *self {
            Foot::Line {t:[t0,t1],a,b} => {
                let rate = [(b[0]-a[0])/(t1-t0),(b[1]-a[1])/(t1-t0)];
                ([a[0]+rate[0]*(t-t0),a[1]+rate[1]*(t-t0)],rate)
            }
            Foot::Curve(ref c) => { let (p,d) = c.d1(t); ([p[0],p[1]],[d[0],d[1]]) }
        }
    }
    /// The foot at `t`.
    fn at(&self,t: f64) -> Uv {
        match *self { Foot::Line {..} => self.d1(t).0,Foot::Curve(ref c) => { let p = c.point(t); [p[0],p[1]] } }
    }
}

/// The curve `of`, on `base`, carried `d` along its normal onto `under`, `base`'s offset over the
/// same parameters; `speed` its greatest, read once; `foot` where `of` stands on `base`, where
/// its face gives it in closed form.
#[derive(Clone,Debug,PartialEq)]
pub struct Parallel { pub of: Curve,pub base: Surface,pub under: Surface,pub d: f64,pub speed: f64,pub foot: Option<Foot> }

impl Parallel {
    /// `of`, lying on `base` at `foot` (where it is known), carried `d` along its normal onto
    /// `under`, which stands `d` off `base` over its own parameters (`Surface::normal_moved`).
    pub fn new(of: Curve,base: Surface,under: Surface,d: f64,foot: Option<Foot>) -> Parallel {
        let mut p = Parallel {of,base,under,d,speed:0.,foot};
        // sampled along `of`'s domain where it has one, else about the origin
        let [a,z] = crate::query::curve_domain(&p.of).unwrap_or([-1.,1.]);
        p.speed = (0..=64).map(|k| norm(p.d1(a+(z-a)*k as f64/64.).1)).fold(0.,f64::max);
        p
    }
    /// `C`, `C'`.
    pub fn d1(&self,t: f64) -> (V,V) {
        let (q,dq) = self.of.d1(t);
        let (uv,rate) = match self.foot { Some(ref f) => { let (uv,r) = f.d1(t); (uv,Some(r)) },None => (self.base.continued_inverse(q),None) };
        let (b,bu,bv) = self.base.d1(uv);
        let (p,pu,pv) = self.under.d1(uv);
        let [du,dv] = rate.unwrap_or_else(|| crate::topo::solved(bu,bv,dq));
        (add(q,sub(p,b)),add(dq,add(scale(sub(pu,bu),du),scale(sub(pv,bv),dv))))
    }
    /// `C`, `C'`, `C''`: the last a central difference of the first, exact enough for the curvature
    /// a sampler reads (`Curve::Iso`'s).
    pub fn d2(&self,t: f64) -> (V,V,V) {
        const H: f64 = 1e-5;
        let (x,d) = self.d1(t);
        let h = H*(1.+t.abs());
        (x,d,scale(sub(self.d1(t+h).1,self.d1(t-h).1),0.5/h))
    }
    pub fn point(&self,t: f64) -> V {
        let q = self.of.point(t);
        let uv = match self.foot { Some(ref f) => f.at(t),None => self.base.continued_inverse(q) };
        add(q,sub(self.under.point(uv),self.base.point(uv)))
    }
    /// Where Newton seeks its point nearest `p` from (`Curve::inverse`): the foot of `p` on `under`
    /// is the foot on `base` of the point `of` is carried from, read on `of`.
    pub fn seed(&self,p: V) -> f64 { self.of.inverse(self.base.point(self.under.continued_inverse(p))) }
    /// A box about it over `[t0, t1]`, never smaller: `of`'s grown by `|d|`, as far as any point
    /// of it stands from the point it is carried from.
    pub fn bounds(&self,t: [f64;2]) -> Box3 {
        let (lo,hi) = self.of.bounds(t);
        let r = self.d.abs();
        (lo.map(|x| x-r),hi.map(|x| x+r))
    }
    /// Moved by `m`: its surfaces' parameters, and so its foot, are kept.
    pub fn moved(&self,m: &Rigid) -> Parallel {
        Parallel {of:self.of.moved(m),base:self.base.moved(m),under:self.under.moved(m),foot:self.foot.clone(),..*self}
    }
    /// Reflected by the improper motion `m`: the normal turns over with it, so the curve is the
    /// reflected one carried `−d` along the reflected base's normal (an offset's `−e`); a
    /// surface reflected may run its parameters the other way, so its foot is searched for.
    pub fn reflected(&self,m: &Rigid) -> Result<Parallel,String> {
        Ok(Parallel {of:self.of.reflected(m)?,base:self.base.reflected(m)?.0,under:self.under.reflected(m)?.0,d:-self.d,foot:None,
            ..*self})
    }
}
