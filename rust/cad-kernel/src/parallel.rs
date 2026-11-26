//! **A curve on a sheet carried along the sheet's normal, exactly** (`Curve::Parallel`): every
//! point `q(t)` of a curve lying on a B-spline sheet (or on an offset of one, `Surface::Offset`)
//! moved `d` along the sheet's unit normal at its foot, `q(t) + d N(u(t), v(t))` — the edge of a
//! face's offset (`offset::carried_sweep`), which no spline of any degree is (a parabola's offset
//! is of degree six and has no rational parameter). It lies on the sheet's offset `under`
//! (`Surface::parallel`), read over the sheet's own parameters, so a face moved keeps its pcurves
//! and the curve keeps its parameter: `t` here is `t` on the curve it was carried from, and the
//! ruling joining the two at one `t` is the sheet's normal there — what a wall the edge sweeps is
//! ruled along (`geom::Blend`). Its point is `q + (P − B)` at the foot `(u, v)` of `q`, `P` the
//! offset's point and `B` the sheet's (`under` read `d` nearer), so it stands exactly `d` off `q`
//! along the normal however finely the foot settles; its rate `q' + (P_u − B_u) u' + (P_v − B_v) v'`,
//! `(u', v')` the curve's rate read in the sheet's parameters, the offset's partials Weingarten's
//! (`Surface::d1`); its second rate a central difference of the first, as a loft's rail's.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::geom::{Box3,Curve,Rigid,Surface,Uv,V};
use crate::util::space::{add,dot,norm,scale,sub};

/// The curve `of` carried `d` along its sheet's normal onto `under`, the sheet's offset
/// (`Surface::Offset`) on which it lies.
#[derive(Clone,Debug,PartialEq)]
pub struct Parallel { pub of: Curve,pub under: Surface,pub d: f64 }

impl Parallel {
    /// `of` carried `d` along the normal of `sheet`, the B-spline sheet or sheet's offset it lies
    /// on: none for any other surface, whose offset is not read over its own parameters.
    pub fn new(of: Curve,sheet: &Surface,d: f64) -> Option<Parallel> {
        match sheet {
            Surface::BSpline(..) | Surface::Offset(..) => {}
            _ => return None,
        }
        let under = sheet.parallel(d,0.)?.surface()?;
        Some(Parallel {of,under,d})
    }
    /// The surface `of` lies on: `under` read `d` nearer the sheet.
    fn base(&self) -> Surface {
        let Surface::Offset(f,ref n,e) = self.under else { unreachable!("a parallel lies on a sheet's offset") };
        Surface::Offset(f,n.clone(),e-self.d)
    }
    /// The foot of `q`, a point of `of`, in the sheet's parameters.
    fn foot(&self,base: &Surface,q: V) -> Uv { base.continued_inverse(q) }
    /// `C`, `C'`.
    pub fn d1(&self,t: f64) -> (V,V) {
        let (q,dq) = self.of.d1(t);
        let base = self.base();
        let uv = self.foot(&base,q);
        let (b,bu,bv) = base.d1(uv);
        let (p,pu,pv) = self.under.d1(uv);
        // the curve's rate in the sheet's parameters: `dq = B_u u' + B_v v'`, by its normal equations
        let (e,f,g) = (dot(bu,bu),dot(bu,bv),dot(bv,bv));
        let (x,y) = (dot(bu,dq),dot(bv,dq));
        let det = e*g-f*f;
        let (du,dv) = if det.abs() > 0. { ((g*x-f*y)/det,(e*y-f*x)/det) } else { (0.,0.) };
        let turn = add(scale(sub(pu,bu),du),scale(sub(pv,bv),dv));
        (add(q,sub(p,b)),add(dq,turn))
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
        let base = self.base();
        let uv = self.foot(&base,q);
        add(q,sub(self.under.point(uv),base.point(uv)))
    }
    /// The parameter of its point nearest `p`: the foot of `p` on `under` is the foot on the sheet
    /// of the point `of` is carried from, read on `of`, then Newton on `(C − p)·C'`.
    pub fn inverse(&self,p: V) -> f64 {
        let uv = self.under.continued_inverse(p);
        let mut t = self.of.inverse(self.base().point(uv));
        let span = crate::query::curve_domain(&self.of);
        for _ in 0..20 {
            let (x,d,dd) = self.d2(t);
            let e = sub(x,p);
            let h = dot(d,d)+dot(e,dd);
            if !(h > 0.) { break }
            let mut next = t-dot(e,d)/h;
            if let (None,Some([a,z])) = (self.of.period(),span) { next = next.clamp(a,z); }
            if !next.is_finite() { break }
            if (next-t).abs() <= 1e-15*(1.+t.abs()) { t = next; break }
            t = next;
        }
        t
    }
    /// Its greatest speed: sampled along `of`'s domain where it has one, else about the origin.
    pub fn speed(&self) -> f64 {
        let [a,z] = crate::query::curve_domain(&self.of).unwrap_or([-1.,1.]);
        (0..=64).map(|k| norm(self.d1(a+(z-a)*k as f64/64.).1)).fold(0.,f64::max)
    }
    /// A box about it over `[t0, t1]`, never smaller: `of`'s grown by `|d|`, as far as any point
    /// of it stands from the point it is carried from.
    pub fn bounds(&self,t: [f64;2]) -> Box3 {
        let (lo,hi) = self.of.bounds(t);
        let r = self.d.abs();
        (lo.map(|x| x-r),hi.map(|x| x+r))
    }
    pub fn moved(&self,m: &Rigid) -> Parallel {
        Parallel {of:self.of.moved(m),under:self.under.moved(m),d:self.d}
    }
    /// Reflected by the improper motion `m`: the sheet's normal turns over with it, so the curve is
    /// the reflected one carried `−d` along the reflected sheet's normal (its offset's `−e`).
    pub fn reflected(&self,m: &Rigid) -> Result<Parallel,String> {
        Ok(Parallel {of:self.of.reflected(m)?,under:self.under.reflected(m)?.0,d:-self.d})
    }
}
