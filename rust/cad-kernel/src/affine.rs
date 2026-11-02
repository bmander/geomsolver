//! **Affine maps of the boundary**: `p ↦ M p + t`, `M` any invertible matrix. An affine map carries
//! every surface and curve onto one the kernel holds exactly, its parameters kept, so a solid
//! mapped keeps its topology and its pcurves (`Brep::mapped`). A kind whose image is its own kind is
//! written so (`Surface::mapped`, `Curve::mapped`); anything else is wrapped (`Mapped`,
//! `MappedCurve`), measured in space (its foot in the true metric) and met in its preimage, where
//! the closed forms are (`ssi::intersect`, `query::curve_surface`).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::geom::{conjugate_bounds,hull,Box3,Carry,Curve,Frame,Rigid,Surface,Traced,Uv,V};
use super::topo::{reverse_loop,Brep,EdgeCurve,Pcurve};
use crate::util::space::{add,cross,distance,dot,norm,scale,sub,unit};
use std::sync::Arc;

const TAU: f64 = std::f64::consts::TAU;
const PI: f64 = std::f64::consts::PI;

/// An invertible affine map `p ↦ m p + t`, `m` row-major.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Affine { pub m: [[f64;3];3],pub t: V }

/// How far a matrix may stray from a similarity, relative to its scale squared, and still be read
/// as one: the rounding a rotation's own entries carry.
const SIMILAR: f64 = 1e-13;

impl Affine {
    pub const IDENTITY: Affine = Affine {m:[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],t:[0.;3]};
    /// Refused where it is not finite or not invertible (its determinant nothing against its
    /// columns' sizes).
    pub fn new(m: [[f64;3];3],t: V) -> Result<Affine,String> {
        if !(m.iter().flatten().chain(&t).all(|x| x.is_finite())) { return Err("an affine map with an entry not finite".into()) }
        let a = Affine {m,t};
        let size = (0..3).map(|j| norm(a.column(j))).product::<f64>();
        if !(a.det().abs() > 1e-12*size) { return Err(format!("an affine map that is not invertible: {m:?}")) }
        Ok(a)
    }
    /// The rigid motion as an affine map.
    pub fn of_rigid(r: &Rigid) -> Affine { Affine {m:r.r,t:r.t} }
    /// The stretch by `s[k]` along each of `f`'s axes about its origin (a ball's frame stretched
    /// into an ellipsoid's).
    pub fn stretch(f: &Frame,s: [f64;3]) -> Result<Affine,String> {
        let m = outer(&[f.x,f.y,f.z],&s);
        let lin = Affine {m,t:[0.;3]};
        Affine::new(m,sub(f.o,lin.vector(f.o)))
    }
    pub fn point(&self,p: V) -> V { add(self.vector(p),self.t) }
    pub fn vector(&self,v: V) -> V { [dot(self.m[0],v),dot(self.m[1],v),dot(self.m[2],v)] }
    fn column(&self,j: usize) -> V { [self.m[0][j],self.m[1][j],self.m[2][j]] }
    pub fn det(&self) -> f64 { dot(self.m[0],cross(self.m[1],self.m[2])) }
    /// The rows of `M⁻¹`: the cofactors over the determinant.
    fn inverse_rows(&self) -> [V;3] {
        let d = self.det();
        [cross(self.column(1),self.column(2)),cross(self.column(2),self.column(0)),cross(self.column(0),self.column(1))]
            .map(|c| scale(c,1./d))
    }
    /// `M⁻ᵀ n`: where a normal goes (the covector a plane's normal is), not unit.
    pub fn normal(&self,n: V) -> V {
        let c = self.inverse_rows();
        std::array::from_fn(|i| (0..3).map(|k| c[k][i]*n[k]).sum())
    }
    /// A frame carried over and made orthonormal again: its origin and `x` mapped, its `z` where its
    /// plane's normal goes.
    pub fn frame(&self,f: &Frame) -> Frame { Frame::new(self.point(f.o),self.normal(f.z),self.vector(f.x)) }
    /// The inverse map.
    pub fn inverse(&self) -> Affine {
        let m = self.inverse_rows();
        let back = Affine {m,t:[0.;3]};
        Affine {m,t:scale(back.vector(self.t),-1.)}
    }
    /// This map, then `next`.
    pub fn then(&self,next: &Affine) -> Affine {
        let m = std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| next.m[i][k]*self.m[k][j]).sum()));
        Affine {m,t:next.point(self.t)}
    }
    /// The singular values of `M`, least first: the least and greatest a length is stretched by.
    pub fn stretches(&self) -> [f64;3] { eigen3(outer(&self.m,&[1.;3])).0.map(|x| x.max(0.).sqrt()) }
    /// `M = s R` with `R` orthogonal, to rounding: the scale and the rigid motion that, scaled by
    /// it about the origin, is the map (`p ↦ s (R p + t / s)`), and whether `R` is proper.
    pub fn similarity(&self) -> Option<(f64,Rigid,bool)> {
        let mtm = outer(&self.m,&[1.;3]);
        let s2 = (mtm[0][0]+mtm[1][1]+mtm[2][2])/3.;
        let off = (0..3).flat_map(|i| (0..3).map(move |j| (i,j)))
            .map(|(i,j)| (mtm[i][j]-if i == j { s2 } else { 0. }).abs()).fold(0.,f64::max);
        if !(off <= SIMILAR*s2) { return None }
        let s = s2.sqrt();
        Some((s,Rigid {r:self.m.map(|row| row.map(|x| x/s)),t:scale(self.t,1./s)},self.det() > 0.))
    }
    /// Whether it is the identity to rounding.
    pub fn is_identity(&self) -> bool {
        let size = 1.+norm(self.t);
        (0..3).all(|i| (0..3).all(|j| (self.m[i][j]-if i == j { 1. } else { 0. }).abs() <= 1e-14) && self.t[i].abs() <= 1e-14*size)
    }
}

/// `Σ wₖ vₖ vₖᵀ`: a stretch along axes (`w` the stretches), a Gram matrix (`MᵀM`, the rows of `M`
/// with weights one), a quadric's form (`w` the inverse squared semi-axes).
fn outer(vs: &[V],w: &[f64]) -> [V;3] {
    std::array::from_fn(|i| std::array::from_fn(|j| vs.iter().zip(w).map(|(v,w)| w*v[i]*v[j]).sum()))
}

/// The eigenvalues of the symmetric `a`, ascending, and a unit eigenvector for each
/// (`util::linalg::sym_eigen`).
fn eigen3(a: [V;3]) -> ([f64;3],[V;3]) {
    use crate::util::linalg::{sym_eigen,Mat};
    let (l,v) = sym_eigen(&Mat::from_vec(3,3,a.iter().flatten().copied().collect()));
    ([l[0],l[1],l[2]],std::array::from_fn(|k| [v.at(0,k),v.at(1,k),v.at(2,k)]))
}

/// A map of a face's parameters, `uv ↦ m uv + t`: what a surface's own parameters become where its
/// image is written as another surface's (a plane's frame made orthonormal again, a cone's
/// generator scaled, a reflected revolution's `u` run the other way).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct UvMap { pub m: [[f64;2];2],pub t: Uv }

impl UvMap {
    pub const IDENTITY: UvMap = UvMap {m:[[1.,0.],[0.,1.]],t:[0.;2]};
    pub fn apply(&self,uv: Uv) -> Uv {
        [self.m[0][0]*uv[0]+self.m[0][1]*uv[1]+self.t[0],self.m[1][0]*uv[0]+self.m[1][1]*uv[1]+self.t[1]]
    }
    pub fn det(&self) -> f64 { self.m[0][0]*self.m[1][1]-self.m[0][1]*self.m[1][0] }
    pub fn is_identity(&self) -> bool { *self == UvMap::IDENTITY }
    /// This map, then `next`.
    pub fn then(&self,next: &UvMap) -> UvMap {
        let m = std::array::from_fn(|i| std::array::from_fn(|j| next.m[i][0]*self.m[0][j]+next.m[i][1]*self.m[1][j]));
        UvMap {m,t:next.apply(self.t)}
    }
    /// Each parameter scaled about nothing.
    fn scaled(su: f64,sv: f64) -> UvMap { UvMap {m:[[su,0.],[0.,sv]],t:[0.;2]} }
    /// As a map of space about the plane `z = 0` (a pcurve's points), `z` kept.
    fn spatial(&self) -> Affine {
        let m = self.m;
        Affine {m:[[m[0][0],m[0][1],0.],[m[1][0],m[1][1],0.],[0.,0.,1.]],t:[self.t[0],self.t[1],0.]}
    }
}

/// **The image of a surface under an affine map** (`Surface::Affine`): `A(S(u, v))` over the
/// preimage's own parameters, its foot found in space's own metric.
#[derive(Clone,Debug,PartialEq)]
pub struct Mapped {
    /// The map, preimage to image, and its inverse.
    pub map: Affine,
    pub back: Affine,
    /// The preimage: never itself an image, a plane or a B-spline sheet, which map to their own kinds.
    pub of: Surface,
    /// The preimage's frame carried over, made orthonormal: what `Surface::frame` lends it.
    pub frame: Frame,
    /// The least and greatest stretch of the map.
    pub stretch: [f64;2],
    /// A quadric's image in its principal axes, where its foot is found in closed form.
    pub quadric: Option<Quadric>,
}

/// A sphere's or a cylinder's image about its principal axes: the ellipsoid of semi-axes `e` along
/// `f`'s axes, or the elliptic cylinder whose section square to `f.z` is the ellipse of semi-axes
/// `e` along `f.x` and `f.y`.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Quadric { Ellipsoid(Frame,[f64;3]),Cylinder(Frame,[f64;2]) }

impl Quadric {
    /// The point nearest `p` (Eberly's, `ellipsoid_foot`, `ellipse_foot`), the outward unit normal
    /// there (the form's gradient, `xᵢ / eᵢ²`) and whether `p` is outside.
    fn foot(&self,p: V) -> (V,V,bool) {
        match *self {
            Quadric::Ellipsoid(f,e) => {
                let l = f.local(p);
                let x = ellipsoid_foot(e,l);
                let level: f64 = (0..3).map(|k| (l[k]/e[k])*(l[k]/e[k])).sum();
                (f.at(x),unit(f.dir(std::array::from_fn(|k| x[k]/(e[k]*e[k])))),level >= 1.)
            }
            Quadric::Cylinder(f,e) => {
                let l = f.local(p);
                let [x,y] = ellipse_foot(e,[l[0],l[1]]);
                let level = (l[0]/e[0])*(l[0]/e[0])+(l[1]/e[1])*(l[1]/e[1]);
                (f.at([x,y,l[2]]),unit(f.dir([x/(e[0]*e[0]),y/(e[1]*e[1]),0.])),level >= 1.)
            }
        }
    }
    /// Whether two are one quadric, to `tol`: the same centre (a cylinder's axis the same line)
    /// and the same quadratic form `Σ (aᵢ · x)² / eᵢ²` (so axes along each other where the
    /// semi-axes differ, any across them where two are equal), whose change `δ` along a direction
    /// moves a radius `r` there by `r³ δ / 2`.
    pub fn same(&self,other: &Quadric,tol: f64) -> bool {
        let form = |axes: &[V],e: &[f64]| outer(axes,&e.iter().map(|e| 1./(e*e)).collect::<Vec<_>>());
        let near = |d: &[f64],e: &[f64],p: [V;3],q: [V;3]| {
            let far = d.iter().chain(e).fold(0.,|m: f64,&x| m.max(x));
            (0..3).all(|i| (0..3).all(|j| far*far*far*(p[i][j]-q[i][j]).abs() <= 2.*tol))
        };
        match (self,other) {
            (Quadric::Ellipsoid(f,d),Quadric::Ellipsoid(g,e)) => distance(f.o,g.o) <= tol
                && near(d,e,form(&[f.x,f.y,f.z],d),form(&[g.x,g.y,g.z],e)),
            (Quadric::Cylinder(f,d),Quadric::Cylinder(g,e)) => norm(cross(f.z,g.z)) <= 1e-12
                && norm(cross(sub(g.o,f.o),f.z)) <= tol
                && near(d,e,form(&[f.x,f.y],d),form(&[g.x,g.y],e)),
            _ => false,
        }
    }
}

impl Mapped {
    /// The image of `of` under `map`, `of` itself not an image.
    pub fn new(map: Affine,of: Surface) -> Mapped {
        debug_assert!(!matches!(of,Surface::Affine(_)),"an image of an image is the composite's");
        let frame = map.frame(of.frame());
        let o = frame.o;
        let s = map.stretches();
        let quadric = match of {
            Surface::Sphere(g,r) => {
                let l = [map.vector(scale(g.x,r)),map.vector(scale(g.y,r)),map.vector(scale(g.z,r))];
                let (lam,u) = eigen3(outer(&l,&[1.;3]));
                let z = if dot(cross(u[0],u[1]),u[2]) > 0. { u[2] } else { scale(u[2],-1.) };
                Some(Quadric::Ellipsoid(Frame {o,x:u[0],y:u[1],z},lam.map(|x| x.max(0.).sqrt())))
            }
            Surface::Cylinder(g,r) => {
                let d = unit(map.vector(g.z));
                let across = |w: V| { let w = map.vector(scale(w,r)); sub(w,scale(d,dot(w,d))) };
                let (p,q) = (across(g.x),across(g.y));
                // the two eigenvectors across the axis (the largest two), the larger first
                let (lam,u) = eigen3(outer(&[p,q],&[1.;2]));
                let x = unit(sub(u[2],scale(d,dot(u[2],d))));
                Some(Quadric::Cylinder(Frame {o,x,y:cross(d,x),z:d},[lam[2].max(0.).sqrt(),lam[1].max(0.).sqrt()]))
            }
            _ => None,
        };
        Mapped {back:map.inverse(),map,of,frame,stretch:[s[0],s[2]],quadric}
    }
    /// `S`, `S_u`, `S_v`.
    pub fn d1(&self,uv: Uv) -> (V,V,V) {
        let (p,su,sv) = self.of.d1(uv);
        (self.map.point(p),self.map.vector(su),self.map.vector(sv))
    }
    /// The parameters of the surface's point nearest `p`: a quadric's in closed form, any other's by
    /// Newton in space's metric from `start` (the preimage's foot, or a hint), asked only there.
    pub(crate) fn foot(&self,p: V,start: impl FnOnce() -> Uv) -> Uv {
        match self.quadric {
            Some(q) => self.of.inverse(self.back.point(q.foot(p).0)),
            None => self.newton(p,start()),
        }
    }
    /// Gauss–Newton on `|S(uv) − p|²` in space, from `uv`, never moving farther off: each accepted
    /// point's derivatives carried into the next step, which is taken whole once it is at rounding.
    fn newton(&self,p: V,mut uv: Uv) -> Uv {
        let periods = self.of.periods();
        let mut at = self.d1(uv);
        let mut gap = distance(at.0,p);
        for _ in 0..40 {
            let (x,su,sv) = at;
            let e = sub(x,p);
            let (a11,a12,a22) = (dot(su,su),dot(su,sv),dot(sv,sv));
            let (b1,b2) = (dot(su,e),dot(sv,e));
            let det = a11*a22-a12*a12;
            if !(det > 1e-300*(a11*a22).max(1e-300)) { break }
            let step = [(a22*b1-a12*b2)/det,(a11*b2-a12*b1)/det];
            let mut next = [uv[0]-step[0],uv[1]-step[1]];
            if step[0].abs().max(step[1].abs()) <= 1e-15*(1.+uv[0].abs().max(uv[1].abs())) { uv = next; break }
            let mut there = self.d1(next);
            let mut d = distance(there.0,p);
            for _ in 0..8 {
                if d <= gap { break }
                next = [(uv[0]+next[0])/2.,(uv[1]+next[1])/2.];
                there = self.d1(next);
                d = distance(there.0,p);
            }
            if d > gap { break }
            (uv,at,gap) = (next,there,d);
        }
        for k in 0..2 { if let Some(per) = periods[k] { uv[k] = uv[k].rem_euclid(per); if uv[k] >= per { uv[k] = 0. } } }
        uv
    }
    pub fn inverse(&self,p: V) -> Uv { self.foot(p,|| self.of.inverse(self.back.point(p))) }
    pub fn inverse_near(&self,p: V,hint: Uv) -> Uv { self.foot(p,|| self.of.inverse_near(self.back.point(p),hint)) }
    /// The signed distance and the unit normal at the foot, its gradient, positive on the side of
    /// `S_u × S_v` (outward from a quadric's image unless the map reverses space). A quadric's in
    /// closed form; any other's on the side of the normal the point stands along, or, where it
    /// stands along no normal (a foot Newton did not reach), the side the preimage's says.
    pub fn side(&self,p: V) -> (f64,V) {
        let flip = if self.map.det() < 0. { -1. } else { 1. };
        if let Some(q) = self.quadric {
            let (x,out,outside) = q.foot(p);
            let d = distance(p,x);
            return (if outside { flip*d } else { -flip*d },scale(out,flip))
        }
        let (x,su,sv) = self.d1(self.inverse(p));
        let q = self.back.point(p);
        let n = crate::util::space::normalised(cross(su,sv))
            .unwrap_or_else(|| unit(scale(self.map.normal(self.of.gradient(q)),flip)));
        let (e,d) = (sub(p,x),distance(p,x));
        let along = dot(e,n);
        let sign = if along.abs() >= 0.5*d { along } else { flip*self.of.implicit(q) };
        (if sign < 0. { -d } else { d },n)
    }
    /// How far `p` stands from the patch's own points, where its preimage has a domain's edge.
    pub fn off_patch(&self,p: V) -> Option<f64> {
        self.of.bounded().then(|| distance(p,self.d1(self.inverse(p)).0))
    }
    /// The length over which it turns appreciably: a quadric's least radius of curvature, else the
    /// preimage's shrunk by what a map can do to a curvature (`σ_min³ / σ_max²`).
    pub fn feature(&self) -> f64 {
        match self.quadric {
            Some(Quadric::Ellipsoid(_,e)) => { let lo = e[0].min(e[1]).min(e[2]); lo*lo/e[0].max(e[1]).max(e[2]) }
            Some(Quadric::Cylinder(_,e)) => e[1]*e[1]/e[0],
            None => {
                let [lo,hi] = self.stretch;
                (self.of.feature()*lo*lo*lo/(hi*hi)).max(f64::MIN_POSITIVE)
            }
        }
    }
}

/// A net's poles carried by `f`, its weights and knots kept (a rational net's image is the net of
/// its poles' images under an affine map).
pub(crate) fn net_mapped(n: &super::nurbs::Net,f: &dyn Fn(V) -> V) -> super::nurbs::Net {
    super::nurbs::Net {poles:n.poles.iter().map(|row| row.iter().map(|&p| f(p)).collect()).collect(),..n.clone()}
}

/// **The image of a curve under an affine map** (`Curve::Affine`): `A(C((t − β) / α))`, the
/// preimage's own parameter where `at = [α, β]` is `[1, 0]` (an edge's curve), or run so that a
/// pcurve keeps reading its edge's parameter where the edge's curve was written anew.
#[derive(Clone,Debug,PartialEq)]
pub struct MappedCurve { pub map: Affine,pub back: Affine,pub of: Curve,pub at: [f64;2],
    /// The greatest stretch of the map (`speed`).
    pub stretch: f64 }

impl MappedCurve {
    pub fn new(map: Affine,of: Curve,at: [f64;2]) -> MappedCurve {
        debug_assert!(!matches!(of,Curve::Affine(_)),"an image of an image is the composite's");
        MappedCurve {back:map.inverse(),stretch:map.stretches()[2],map,of,at}
    }
    fn local(&self,t: f64) -> f64 { (t-self.at[1])/self.at[0] }
    fn global(&self,s: f64) -> f64 { self.at[0]*s+self.at[1] }
    pub fn d2(&self,t: f64) -> (V,V,V) {
        let (x,d,dd) = self.of.d2(self.local(t));
        let a = self.at[0];
        (self.map.point(x),scale(self.map.vector(d),1./a),scale(self.map.vector(dd),1./(a*a)))
    }
    pub fn period(&self) -> Option<f64> { self.of.period().map(|p| p*self.at[0]) }
    pub fn breaks(&self,[a,b]: [f64;2]) -> Vec<f64> {
        self.of.breaks([self.local(a),self.local(b)]).into_iter().map(|s| self.global(s)).collect()
    }
    /// The parameter of the nearest point: the preimage's foot of the point carried back, refined
    /// by Newton on `(C − p) · C'` in space.
    pub fn inverse(&self,p: V) -> f64 {
        let mut t = self.global(self.of.inverse(self.back.point(p)));
        let mut at = self.d2(t);
        let mut gap = distance(at.0,p);
        for _ in 0..12 {
            let (x,d,dd) = at;
            let e = sub(x,p);
            let h = dot(d,d)+dot(e,dd);
            if !(h > 0.) { break }
            let next = t-dot(e,d)/h;
            if (next-t).abs() <= 1e-15*(1.+t.abs()) { t = next; break }
            let there = self.d2(next);
            let g = distance(there.0,p);
            if !(g < gap) { break }
            (t,at,gap) = (next,there,g);
        }
        match self.period() { Some(per) => t.rem_euclid(per),None => t }
    }
    /// A conic preimage's image boxed exactly over `span` of its own parameter.
    fn conic(&self,f: &Frame,a: f64,b: f64,span: [f64;2]) -> Box3 {
        conjugate_bounds(self.map.point(f.o),self.map.vector(scale(f.x,a)),self.map.vector(scale(f.y,b)),span)
    }
    pub fn speed(&self) -> f64 { self.of.speed()*self.stretch/self.at[0] }
    pub fn bounds(&self,[t0,t1]: [f64;2]) -> Box3 {
        let (s0,s1) = (self.local(t0),self.local(t1));
        match self.of {
            Curve::Line {p,d} => hull([s0,s1].map(|s| self.map.point(add(p,scale(d,s))))),
            Curve::Circle(f,r) => self.conic(&f,r,r,[s0,s1]),
            Curve::Ellipse(f,a,b) => self.conic(&f,a,b,[s0,s1]),
            Curve::BSpline(ref b) => hull(b.poles_over([s0.min(s1),s0.max(s1)]).iter().map(|&p| self.map.point(p))),
            _ => super::geom::boxed(&self.map,self.of.bounds([s0,s1])),
        }
    }
}

impl Surface {
    /// **The surface's image under `a`**, and what its parameters become there: `S'(φ(u, v)) =
    /// A(S(u, v))`. A kind whose image is its own kind is written so — moved or reflected where `a`
    /// is rigid, scaled where it is a similarity, a plane on its frame made orthonormal again, a
    /// B-spline's poles mapped, a blend carried along a line by its edges' images — and anything
    /// else is wrapped (`Surface::Affine`), its parameters its own.
    pub fn mapped(&self,a: &Affine) -> (Surface,UvMap) {
        if let Surface::Affine(m) = self { return m.of.mapped(&m.map.then(a)) }
        if a.is_identity() { return (self.clone(),UvMap::IDENTITY) }
        let wrapped = || (Surface::Affine(Arc::new(Mapped::new(*a,self.clone()))),UvMap::IDENTITY);
        if let Some((s,r,proper)) = a.similarity() {
            let (rigid,phi) = if proper { (self.moved(&r),UvMap::IDENTITY) } else {
                match self.reflected(&r) {
                    Ok((surface,flip)) => (surface,if flip { UvMap::scaled(-1.,1.) } else { UvMap::IDENTITY }),
                    Err(_) => return wrapped(),
                }
            };
            if (s-1.).abs() <= 1e-15 { return (rigid,phi) }
            return match rigid.scaled(s) { Some((surface,psi)) => (surface,phi.then(&psi)),None => wrapped() }
        }
        match self {
            Surface::Plane(f) => {
                let (o,x,y) = (a.point(f.o),a.vector(f.x),a.vector(f.y));
                let g = Frame::new(o,cross(x,y),x);
                (Surface::Plane(g),UvMap {m:[[dot(x,g.x),dot(y,g.x)],[dot(x,g.y),dot(y,g.y)]],t:[0.;2]})
            }
            Surface::BSpline(f,n) => (Surface::BSpline(*f,Arc::new(net_mapped(n,&|p| a.point(p)))),UvMap::IDENTITY),
            Surface::Blend(f,b) => match b.mapped(a) {
                Some(b) => (Surface::Blend(a.frame(f),Arc::new(b)),UvMap::IDENTITY),
                None => wrapped(),
            },
            _ => wrapped(),
        }
    }
    /// The surface scaled by `s > 0` about the origin, and what its parameters become: a radius and
    /// a length along it scale, an angle does not. None for a face family's wall.
    fn scaled(&self,s: f64) -> Option<(Surface,UvMap)> {
        let at = |f: &Frame| Frame {o:scale(f.o,s),..*f};
        let sc = Affine {m:[[s,0.,0.],[0.,s,0.],[0.,0.,s]],t:[0.;3]};
        Some(match *self {
            Surface::Plane(f) => (Surface::Plane(at(&f)),UvMap::scaled(s,s)),
            Surface::Cylinder(f,r) => (Surface::Cylinder(at(&f),s*r),UvMap::scaled(1.,s)),
            Surface::Cone(f,r,h) => (Surface::Cone(at(&f),s*r,h),UvMap::scaled(1.,s)),
            Surface::Sphere(f,r) => (Surface::Sphere(at(&f),s*r),UvMap::IDENTITY),
            Surface::Torus(f,big,r) => (Surface::Torus(at(&f),s*big,s*r),UvMap::IDENTITY),
            Surface::Extrusion(f,ref c) => {
                let (c,[al,be]) = c.mapped(&sc);
                (Surface::Extrusion(at(&f),Arc::new(c)),UvMap {m:[[al,0.],[0.,s]],t:[be,0.]})
            }
            Surface::Revolution(f,ref c) => {
                let (c,[al,be]) = c.mapped(&sc);
                (Surface::Revolution(at(&f),Arc::new(c)),UvMap {m:[[1.,0.],[0.,al]],t:[0.,be]})
            }
            Surface::BSpline(f,ref n) => (Surface::BSpline(f,Arc::new(net_mapped(n,&|p| scale(p,s)))),UvMap::IDENTITY),
            Surface::Blend(f,ref b) => (Surface::Blend(at(&f),Arc::new(b.mapped(&sc)?)),UvMap::IDENTITY),
            Surface::Family(..) | Surface::Affine(..) => return None,
        })
    }
}

impl super::geom::Blend {
    /// Its image under `a`, where a map carries it as one: both edges mapped and each run over its
    /// own image's parameters, a step along a line along the step's image, a screw scaled where `a`
    /// is a similarity. None for a screw under any other map.
    fn mapped(&self,a: &Affine) -> Option<super::geom::Blend> {
        let carry = match self.carry {
            Carry::Line {delta} => Carry::Line {delta:a.vector(delta)},
            Carry::Screw {center,axis,angle,advance} => {
                let (s,r,proper) = a.similarity()?;
                if !proper { return None }
                let axis = r.vector(axis);
                Carry::Screw {center:a.point(center),axis,angle,advance:s*advance}
            }
        };
        let (ca,[aa,ba]) = self.a.mapped(a);
        let (cb,[ab,bb]) = self.b.mapped(a);
        Some(super::geom::Blend {a:ca,ta:self.ta.map(|t| aa*t+ba),b:cb,tb:self.tb.map(|t| ab*t+bb),carry,closed:self.closed})
    }
}

impl Curve {
    /// **The curve's image under `a`**, and how its parameter runs there, `[α, β]` with `α > 0`:
    /// `C'(α t + β) = A(C(t))`. A line is a line at the speed the map gives its direction, a circle
    /// or an ellipse an ellipse about its principal axes (or a circle) started where the map takes
    /// its own start, a B-spline its poles mapped, a traced curve its points and surfaces; a rigid
    /// motion moves any curve, and anything else is wrapped (`Curve::Affine`).
    pub fn mapped(&self,a: &Affine) -> (Curve,[f64;2]) {
        if let Curve::Affine(m) = self {
            let (c,[al,be]) = m.of.mapped(&m.map.then(a));
            // `self(t) = map(of((t − β₀) / α₀))`
            let [a0,b0] = m.at;
            return (c,[al/a0,be-al*b0/a0])
        }
        if a.is_identity() { return (self.clone(),[1.,0.]) }
        let wrapped = || (Curve::Affine(Arc::new(MappedCurve::new(*a,self.clone(),[1.,0.]))),[1.,0.]);
        match *self {
            Curve::Line {p,d} => { let v = a.vector(d); let l = norm(v); (Curve::Line {p:a.point(p),d:scale(v,1./l)},[l,0.]) }
            Curve::Circle(f,r) => conic(a,f.o,scale(f.x,r),scale(f.y,r)),
            Curve::Ellipse(f,ea,eb) => conic(a,f.o,scale(f.x,ea),scale(f.y,eb)),
            Curve::BSpline(ref b) => {
                let poles = b.poles.iter().map(|&p| a.point(p)).collect();
                (Curve::BSpline(Arc::new(super::nurbs::BSpline {poles,..(**b).clone()})),[1.,0.])
            }
            Curve::Traced(ref c) => (Curve::Traced(Arc::new(Traced {a:c.a.mapped(a).0,b:c.b.mapped(a).0,
                pts:c.pts.iter().map(|&p| a.point(p)).collect(),closed:c.closed})),[1.,0.]),
            Curve::Iso(ref b,u) => match b.mapped(a) { Some(b) => (Curve::Iso(Arc::new(b),u),[1.,0.]),None => wrapped() },
            Curve::Rail(..) | Curve::Rim(..) | Curve::SheetCut(..) => match a.similarity() {
                Some((s,r,true)) if (s-1.).abs() <= 1e-15 => (self.moved(&r),[1.,0.]),
                _ => wrapped(),
            },
            Curve::Affine(_) => unreachable!("composed above"),
        }
    }
}

/// `o + cos t p + sin t q` carried by `a`: the ellipse about its principal axes, the longer first
/// (a circle where they are equal), and the turn `φ` its parameter starts from, `τ = t − φ`: with
/// `p', q'` the images, `tan 2φ = 2 p'·q' / (p'·p' − q'·q')` turns them square to each other.
fn conic(a: &Affine,o: V,p: V,q: V) -> (Curve,[f64;2]) {
    let (p,q) = (a.vector(p),a.vector(q));
    let mut phi = 0.5*(2.*dot(p,q)).datan2(dot(p,p)-dot(q,q));
    let axes = |phi: f64| { let (s,c) = phi.dsin_cos(); (add(scale(p,c),scale(q,s)),sub(scale(q,c),scale(p,s))) };
    let (mut major,mut minor) = axes(phi);
    if norm(major) < norm(minor) { phi += 0.5*PI; (major,minor) = axes(phi); }
    let (ea,eb) = (norm(major),norm(minor));
    let x = scale(major,1./ea);
    let y = scale(minor,1./eb);
    let f = Frame {o:a.point(o),x,y,z:cross(x,y)};
    let c = if ea-eb <= 1e-14*ea { Curve::Circle(f,ea) } else { Curve::Ellipse(f,ea,eb) };
    // `τ` held to a turn from where `t = 0` falls (a blend's edge's run; an edge's own is held to
    // its first period by `Brep::mapped`)
    (c,[1.,(-phi).rem_euclid(TAU)])
}

impl Brep {
    /// **Every geometric position carried by the affine map `a`** (`a` invertible): each vertex
    /// mapped, each edge's curve its image (`Curve::mapped`) run over its image's parameters, each
    /// face's surface its image (`Surface::mapped`) with its pcurves carried onto the parameters
    /// that image is read in. A map reversing space turns which way a loop walks about the material,
    /// so every loop is walked the other way round; and a face's sense against its surface's normal
    /// follows where the normal's image (`M⁻ᵀ`, times the sign of `det M` and of the parameters'
    /// map) points the other way. Every edge is still used once each way, and tolerances grow by
    /// the greatest stretch.
    pub fn mapped(&self,a: &Affine) -> Brep {
        let mut b = self.clone();
        let grow = a.stretches()[2];
        for v in &mut b.vertices { v.p = a.point(v.p); v.tol *= grow; }
        let mut runs = vec![[1.,0.];b.edges.len()];
        for (k,e) in b.edges.iter_mut().enumerate() {
            e.tol *= grow;
            if let EdgeCurve::Curve(c) = &e.curve {
                let (c,mut run) = c.mapped(a);
                // a closed curve's edge started within its first period, as the kernel reads one
                if let Some(per) = c.period() { run[1] -= per*((run[0]*e.t[0]+run[1])/per).floor(); }
                e.curve = EdgeCurve::Curve(c);
                e.t = e.t.map(|t| run[0]*t+run[1]);
                runs[k] = run;
            }
        }
        let reversing = a.det() < 0.;
        for f in &mut b.faces {
            let (surface,phi) = f.surface.mapped(a);
            f.surface = surface;
            if reversing != (phi.det() < 0.) { f.reversed = !f.reversed; }
            for l in &mut f.loops {
                for c in l.iter_mut() {
                    c.pcurve = match &c.pcurve {
                        Pcurve::Line {a,b} => Pcurve::Line {a:phi.apply(*a),b:phi.apply(*b)},
                        Pcurve::Inverse {a,b} => Pcurve::Inverse {a:phi.apply(*a),b:phi.apply(*b)},
                        Pcurve::Curve(pc) => Pcurve::Curve(Arc::new(rerun(pc,&phi,runs[c.edge as usize]))),
                    };
                }
                if reversing { *l = reverse_loop(std::mem::take(l)); }
            }
        }
        b
    }
}

/// A pcurve's curve with its points carried by `phi` and read at its edge's new parameter
/// `α t + β`: `C'(α t + β) = φ(C(t))`. The image of its points (`Curve::mapped`) run anew, exactly
/// where it is a spline (its knots moved) or needs no running anew, else wrapped.
fn rerun(pc: &Curve,phi: &UvMap,[al,be]: [f64;2]) -> Curve {
    let (c,[a1,b1]) = pc.mapped(&phi.spatial());
    // `C'(t') = c(r₀ t' + r₁)`
    let (r0,r1) = (a1/al,b1-a1*be/al);
    if (r0-1.).abs() <= 1e-15 && r1.abs() <= 1e-15*(1.+be.abs()) { return c }
    match c {
        Curve::BSpline(ref b) => Curve::BSpline(Arc::new(super::nurbs::BSpline {knots:b.knots.iter().map(|k| (k-r1)/r0).collect(),..(**b).clone()})),
        // `m.map(m.of((r₀ t' + r₁ − β₀) / α₀))`
        Curve::Affine(ref m) => Curve::Affine(Arc::new(MappedCurve::new(m.map,m.of.clone(),[m.at[0]/r0,(m.at[1]-r1)/r0]))),
        other => Curve::Affine(Arc::new(MappedCurve::new(Affine::IDENTITY,other,[1./r0,-r1/r0]))),
    }
}

/// **The point of an ellipsoid nearest `q`**: `q` in the ellipsoid's own frame, `e` its semi-axes
/// along that frame's axes, the foot in the same coordinates. Eberly's robust reduction ("Distance
/// from a point to an ellipse, an ellipsoid, or a hyperellipsoid", 2013): the axes ordered longest
/// first and the point folded into the first octant, where the foot is `xᵢ = rᵢ yᵢ / (s + rᵢ)` at
/// the one root `s` of a function that falls monotonically over a bracket it is bisected in to the
/// last bit; a point on a plane of symmetry the foot either in that plane or off it, as the root's
/// bracket says, and on an axis likewise. The global nearest point, inside the solid as outside it,
/// so the normal through it passes through `q`.
pub fn ellipsoid_foot(e: [f64;3],q: V) -> V {
    let mut order = [0usize,1,2];
    order.sort_by(|&i,&j| e[j].total_cmp(&e[i]));
    let x = foot_sorted(order.map(|k| e[k]),order.map(|k| q[k].abs()));
    let mut out = [0.;3];
    for (n,&k) in order.iter().enumerate() { out[k] = x[n].copysign(q[k]); }
    out
}

/// The point of the ellipse of semi-axes `e` along its frame's axes nearest `q`, as `ellipsoid_foot`.
pub fn ellipse_foot(e: [f64;2],q: [f64;2]) -> [f64;2] {
    let swap = e[1] > e[0];
    let (es,qs) = if swap { ([e[1],e[0]],[q[1],q[0]]) } else { (e,q) };
    let x = foot_planar(es,[qs[0].abs(),qs[1].abs()]);
    let x = [x[0].copysign(qs[0]),x[1].copysign(qs[1])];
    if swap { [x[1],x[0]] } else { x }
}

/// `ellipsoid_foot` for semi-axes `e₀ ≥ e₁ ≥ e₂` and a point `y` in the first octant.
fn foot_sorted([e0,e1,e2]: [f64;3],[y0,y1,y2]: [f64;3]) -> [f64;3] {
    if y2 > 0. {
        if y1 > 0. {
            if y0 > 0. {
                let z = [y0/e0,y1/e1,y2/e2];
                let g = z[0]*z[0]+z[1]*z[1]+z[2]*z[2]-1.;
                if g == 0. { return [y0,y1,y2] }
                let r = [(e0/e2)*(e0/e2),(e1/e2)*(e1/e2),1.];
                let s = foot_root(&r,&z,g);
                return [r[0]*y0/(s+r[0]),r[1]*y1/(s+r[1]),y2/(s+1.)]
            }
            let [x1,x2] = foot_planar([e1,e2],[y1,y2]);
            return [0.,x1,x2]
        }
        if y0 > 0. { let [x0,x2] = foot_planar([e0,e2],[y0,y2]); return [x0,0.,x2] }
        return [0.,0.,e2]
    }
    // on the plane of the two longer axes: the foot off it where the point is near enough the
    // middle that the normal from the shortest axis's side reaches it
    let (d0,d1) = (e0*e0-e2*e2,e1*e1-e2*e2);
    let (n0,n1) = (e0*y0,e1*y1);
    if n0 < d0 && n1 < d1 {
        let (a,b) = (n0/d0,n1/d1);
        let disc = 1.-a*a-b*b;
        if disc > 0. { return [e0*a,e1*b,e2*disc.sqrt()] }
    }
    let [x0,x1] = foot_planar([e0,e1],[y0,y1]);
    [x0,x1,0.]
}

/// The nearest point of the ellipse of semi-axes `e₀ ≥ e₁` to `y` in its first quadrant.
fn foot_planar([e0,e1]: [f64;2],[y0,y1]: [f64;2]) -> [f64;2] {
    if y1 > 0. {
        if y0 > 0. {
            let z = [y0/e0,y1/e1];
            let g = z[0]*z[0]+z[1]*z[1]-1.;
            if g == 0. { return [y0,y1] }
            let r = [(e0/e1)*(e0/e1),1.];
            let s = foot_root(&r,&z,g);
            return [r[0]*y0/(s+r[0]),y1/(s+1.)]
        }
        return [0.,e1]
    }
    let d0 = e0*e0-e1*e1;
    let n0 = e0*y0;
    if n0 < d0 { let a = n0/d0; return [e0*a,e1*(1.-a*a).max(0.).sqrt()] }
    [e0,0.]
}

/// The root `s > −1` of `G(s) = Σ (rᵢ zᵢ / (s + rᵢ))² − 1 = 0` (the last `r` is 1, every `z`
/// positive), `g` its value at `s = 0`: bracketed between `z_last − 1` and 0 or the norm of `rᵢ zᵢ`
/// less one, the side `g` says. `G` falls and is convex there, so Newton from the bracket's left
/// end climbs to the root without passing it, a step at a time; where a step stops climbing or
/// leaves the bracket, the bracket is bisected to the last bit instead.
fn foot_root(r: &[f64],z: &[f64],g: f64) -> f64 {
    let n = r.len();
    let mut num = [0.;3];
    for i in 0..n { num[i] = r[i]*z[i]; }
    let num = &num[..n];
    let (mut s0,mut s1) = (z[n-1]-1.,if g < 0. { 0. } else { num.iter().fold(0.,|m: f64,&v| m.dhypot(v))-1. });
    let at = |s: f64| -> (f64,f64) {
        let (mut h,mut dh) = (-1.,0.);
        for i in 0..n { let q = num[i]/(s+r[i]); h += q*q; dh -= 2.*q*q/(s+r[i]); }
        (h,dh)
    };
    let mut s = s0;
    for _ in 0..64 {
        let (h,dh) = at(s);
        // at the root, or a rounding past it
        if h <= 0. { return s }
        if !(dh < 0.) { break }
        let next = s-h/dh;
        if !(next < s1) { break }
        // settled: no step climbs further
        if !(next > s) { return s }
        s = next;
    }
    s0 = s;
    for _ in 0..2200 {
        let s = 0.5*(s0+s1);
        if s == s0 || s == s1 { return s }
        let h = at(s).0;
        if h > 0. { s0 = s } else if h < 0. { s1 = s } else { return s }
    }
    0.5*(s0+s1)
}
