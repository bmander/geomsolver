//! **A curve on a sheet carried along the sheet's normal, exactly** (`Curve::Parallel`): every
//! point `q(t)` of a curve lying on a surface `base` moved `d` along its unit normal at its foot,
//! `q(t) + d N(u(t), v(t))`, onto `under`, the surface standing `d` off `base` over the same
//! parameters — a B-spline sheet's offset (`Surface::Offset`, `Surface::parallel`), whose edge
//! moved with its face (`offset::carried_sweep`) no spline of any degree is (a parabola's offset is
//! of degree six and has no rational parameter). A face moved keeps its pcurves and the curve keeps
//! its parameter: `t` here is `t` on the curve it was carried from, and the ruling joining the two
//! at one `t` is the normal there — what a wall the edge sweeps is ruled along (`geom::Blend`). Its
//! point is `q + (P − B)` at the foot `(u, v)` of `q`, `P` the offset's point and `B` the base's,
//! so it stands exactly `d` off `q` along the normal however finely the foot settles; its rate
//! `q' + (P_u − B_u) u' + (P_v − B_v) v'`, `(u', v')` the curve's rate read in the parameters, the
//! offset's partials Weingarten's (`Surface::d1`); its second rate a central difference of the
//! first, as a loft's rail's.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::geom::{Box3,Curve,Rigid,Surface,V};
use crate::util::space::{add,norm,scale,sub};

/// The curve `of`, on `base`, carried `d` along its normal onto `under`, `base`'s offset over the
/// same parameters; `speed` its greatest, read once.
#[derive(Clone,Debug,PartialEq)]
pub struct Parallel { pub of: Curve,pub base: Surface,pub under: Surface,pub d: f64,pub speed: f64 }

impl Parallel {
    /// `of`, lying on `base`, carried `d` along its normal onto `under`, which stands `d` off `base`
    /// over its own parameters (a sheet's offset, `Surface::parallel`).
    pub fn new(of: Curve,base: Surface,under: Surface,d: f64) -> Parallel {
        let mut p = Parallel {of,base,under,d,speed:0.};
        // sampled along `of`'s domain where it has one, else about the origin
        let [a,z] = crate::query::curve_domain(&p.of).unwrap_or([-1.,1.]);
        p.speed = (0..=64).map(|k| norm(p.d1(a+(z-a)*k as f64/64.).1)).fold(0.,f64::max);
        p
    }
    /// `C`, `C'`.
    pub fn d1(&self,t: f64) -> (V,V) {
        let (q,dq) = self.of.d1(t);
        let uv = self.base.continued_inverse(q);
        let (b,bu,bv) = self.base.d1(uv);
        let (p,pu,pv) = self.under.d1(uv);
        let [du,dv] = crate::topo::solved(bu,bv,dq);
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
        let uv = self.base.continued_inverse(q);
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
    pub fn moved(&self,m: &Rigid) -> Parallel {
        Parallel {of:self.of.moved(m),base:self.base.moved(m),under:self.under.moved(m),..*self}
    }
    /// Reflected by the improper motion `m`: the normal turns over with it, so the curve is the
    /// reflected one carried `−d` along the reflected base's normal (an offset's `−e`).
    pub fn reflected(&self,m: &Rigid) -> Result<Parallel,String> {
        Ok(Parallel {of:self.of.reflected(m)?,base:self.base.reflected(m)?.0,under:self.under.reflected(m)?.0,d:-self.d,..*self})
    }
}
