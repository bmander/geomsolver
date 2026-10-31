//! The analytic geometry a B-rep face and edge lie on: frames, surfaces with their natural
//! parameterisations and closed-form inverses, and curves. Every surface is parameterised as
//! OCCT (and STEP) parameterise it, so a face written out is the face read in.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::util::space::{add,cross,dot,norm,scale,sub,unit};
use std::sync::Arc;
pub use super::nurbs::BSpline;

pub type V = [f64;3];
pub type Uv = [f64;2];

const TAU: f64 = std::f64::consts::TAU;
const PI: f64 = std::f64::consts::PI;

/// The working tolerance of a construction `size` across (mm): a part in 10⁹ of it.
pub fn tolerance(size: f64) -> f64 { 1e-9*size }
/// `tolerance(size)`, never under `floor` (mm).
pub fn tolerance_over(size: f64,floor: f64) -> f64 { tolerance(size).max(floor) }

/// An axis-aligned box: its least and its greatest corner.
pub type Box3 = (V,V);
/// The box holding nothing, which anything widens.
pub const EMPTY: Box3 = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
/// `t` moved by whole periods into `[lo, lo + period)`.
pub fn around(t: f64,lo: f64,period: f64) -> f64 { lo+(t-lo).rem_euclid(period) }
/// `b` widened to hold `p`.
pub fn include(b: &mut Box3,p: V) { for k in 0..3 { b.0[k] = b.0[k].min(p[k]); b.1[k] = b.1[k].max(p[k]); } }
/// `b` widened to hold `other` (unchanged by `EMPTY`).
pub fn widen(b: &mut Box3,other: Box3) { for k in 0..3 { b.0[k] = b.0[k].min(other.0[k]); b.1[k] = b.1[k].max(other.1[k]); } }
pub(super) fn hull(pts: impl IntoIterator<Item=V>) -> Box3 { let mut b = EMPTY; for p in pts { include(&mut b,p); } b }

/// The box a turn about the axis through `c` along `a`, however far, keeps `b` within: the heights
/// along the axis its corners reach and their greatest distance from it (one linear, the other
/// convex, so both are a corner's), the ring between.
fn swept(c: V,a: V,b: Box3) -> Box3 {
    let a = unit(a);
    let (mut h,mut rho) = ([f64::INFINITY,f64::NEG_INFINITY],0f64);
    for i in 0..8 {
        let d = sub(std::array::from_fn(|k| if (i>>k)&1 == 0 { b.0[k] } else { b.1[k] }),c);
        let along = dot(d,a);
        h = [h[0].min(along),h[1].max(along)];
        rho = rho.max(norm(sub(d,scale(a,along))));
    }
    let across = |k: usize| rho*(1.-a[k]*a[k]).max(0.).sqrt();
    (std::array::from_fn(|k| c[k]+(h[0]*a[k]).min(h[1]*a[k])-across(k)),std::array::from_fn(|k| c[k]+(h[0]*a[k]).max(h[1]*a[k])+across(k)))
}

/// `swept`, the box also stepped up to `advance` along the axis: every point a screw about the line
/// through `c` along `a` carries one of `b`'s, turned any way and stepped no further than that.
fn screwed(c: V,a: V,advance: f64,b: Box3) -> Box3 {
    let step = scale(unit(a),advance);
    let mut out = swept(c,a,b);
    widen(&mut out,swept(c,a,(add(b.0,step),add(b.1,step))));
    out
}

/// Where in `[t0, t1]` `p cos t + q sin t` turns, and whether to its greatest (`φ + 2nπ`, `φ` the
/// angle of `(p, q)`) or its least (`φ + (2n + 1)π`): none where it does not turn at all.
pub fn turns(p: f64,q: f64,[t0,t1]: [f64;2]) -> impl Iterator<Item=(f64,bool)> {
    let phi = q.datan2(p);
    let first = ((t0-phi)/PI).ceil();
    [first,first+1.].into_iter().filter(move |&m| p.dhypot(q) > 0. && phi+m*PI <= t1).map(move |m| (phi+m*PI,m.rem_euclid(2.) == 0.))
}

/// `o + a cos t x + b sin t y` over `[t0, t1]`: its ends, and in each coordinate the turning
/// points between (`o_k ± hypot(a x_k, b y_k)`), exactly.
fn conic_bounds(f: &Frame,a: f64,b: f64,[t0,t1]: [f64;2],ends: [V;2]) -> Box3 {
    let mut out = hull(ends);
    for k in 0..3 {
        let (p,q) = (a*f.x[k],b*f.y[k]);
        for (_,top) in turns(p,q,[t0,t1]) {
            if top { out.1[k] = out.1[k].max(f.o[k]+p.dhypot(q)) } else { out.0[k] = out.0[k].min(f.o[k]-p.dhypot(q)) }
        }
    }
    out
}

/// A right-handed orthonormal frame: an origin and three axes.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Frame { pub o: V,pub x: V,pub y: V,pub z: V }

impl Frame {
    /// The world's own axes.
    pub const WORLD: Frame = Frame {o:[0.;3],x:[1.,0.,0.],y:[0.,1.,0.],z:[0.,0.,1.]};
    /// The frame at `o` whose `z` is along `z` and whose `x` is `x` made square to it.
    pub fn new(o: V,z: V,x: V) -> Frame {
        let z = unit(z);
        let x = unit(sub(x,scale(z,dot(x,z))));
        Frame {o,x,y:cross(z,x),z}
    }
    /// A frame about `z` with some `x` square to it.
    pub fn about(o: V,z: V) -> Frame {
        let z = unit(z);
        let seed = if z[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
        Frame::new(o,z,seed)
    }
    /// The point with local coordinates `l`.
    pub fn at(&self,l: V) -> V {
        [0,1,2].map(|k| self.o[k]+l[0]*self.x[k]+l[1]*self.y[k]+l[2]*self.z[k])
    }
    /// A direction with local components `l`.
    pub fn dir(&self,l: V) -> V { [0,1,2].map(|k| l[0]*self.x[k]+l[1]*self.y[k]+l[2]*self.z[k]) }
    /// The local components of a direction.
    pub fn dir_local(&self,d: V) -> V { [dot(d,self.x),dot(d,self.y),dot(d,self.z)] }
    /// The local coordinates of `p`.
    pub fn local(&self,p: V) -> V { let d = sub(p,self.o); [dot(d,self.x),dot(d,self.y),dot(d,self.z)] }
    /// The motion carrying local coordinates to where they stand (`at`).
    pub fn placing(&self) -> Rigid { Rigid {r:[0,1,2].map(|k| [self.x[k],self.y[k],self.z[k]]),t:self.o} }
    /// The motion carrying a point to its local coordinates (`local`).
    pub fn unplacing(&self) -> Rigid { Rigid {r:[self.x,self.y,self.z],t:[self.x,self.y,self.z].map(|a| -dot(a,self.o))} }
    pub fn moved(&self,m: &Rigid) -> Frame {
        Frame {o:m.point(self.o),x:m.vector(self.x),y:m.vector(self.y),z:m.vector(self.z)}
    }
    /// Whether its axes are right-handed, `y` on the turn from `z` to `x`.
    pub fn right(&self) -> bool { dot(cross(self.z,self.x),self.y) > 0. }
    /// **Its axes made right-handed again** (#578), as a surface placed on them reads them: one
    /// read by its `x` and `y` (a plane, a circle) keeps those and takes `z = x × y`; one turned
    /// about its `z` (a surface of revolution) keeps its axis and `x` and takes `y = z × x`,
    /// which runs its turn the other way — whether it did is returned beside the frame.  What a
    /// native kernel's left-handed placement is read as (`brep::json`) and a reflected B-rep is
    /// placed by (`Brep::reflected`).
    pub fn rehanded(self,about_z: bool) -> (Frame,bool) {
        match (self.right(),about_z) {
            (true,_) => (self,false),
            (false,false) => (Frame {z:cross(self.x,self.y),..self},false),
            (false,true) => (Frame {y:cross(self.z,self.x),..self},true),
        }
    }
}

/// A rigid motion: `p ↦ R p + t`, R row-major.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Rigid { pub r: [[f64;3];3],pub t: V }

impl Rigid {
    pub fn identity() -> Rigid { Rigid {r:[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],t:[0.;3]} }
    /// From a row-major 3×4 matrix (the CAD recipe's placement).
    pub fn from_rows(m: &[f64]) -> Rigid {
        Rigid {r:[[m[0],m[1],m[2]],[m[4],m[5],m[6]],[m[8],m[9],m[10]]],t:[m[3],m[7],m[11]]}
    }
    /// The turn by `angle` about the line through `o` along the unit `a`.
    pub fn turn(o: V,a: V,angle: f64) -> Rigid {
        let a = unit(a);
        let (s,c) = angle.dsin_cos();
        let t = 1.-c;
        let [x,y,z] = a;
        let r = [[c+x*x*t,x*y*t-z*s,x*z*t+y*s],[y*x*t+z*s,c+y*y*t,y*z*t-x*s],[z*x*t-y*s,z*y*t+x*s,c+z*z*t]];
        let ro = [dot(r[0],o),dot(r[1],o),dot(r[2],o)];
        Rigid {r,t:sub(o,ro)}
    }
    pub fn vector(&self,v: V) -> V { [dot(self.r[0],v),dot(self.r[1],v),dot(self.r[2],v)] }
    pub fn point(&self,p: V) -> V { add(self.vector(p),self.t) }
    /// First `self`, then `next`.
    pub fn then(&self,next: &Rigid) -> Rigid {
        let r = std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| next.r[i][k]*self.r[k][j]).sum()));
        Rigid {r,t:next.point(self.t)}
    }
}

/// A surface and its natural parameterisation `S(u, v)`, with normal `S_u × S_v`.
#[derive(Clone,Debug,PartialEq)]
pub enum Surface {
    /// `o + u x + v y`.
    Plane(Frame),
    /// `o + r (cos u x + sin u y) + v z`.
    Cylinder(Frame,f64),
    /// `o + (r + v sin α)(cos u x + sin u y) + v cos α z`: `r` the radius at `o`, `α` the
    /// half-angle, `v` along the generator; the apex at `v = −r / sin α`.
    Cone(Frame,f64,f64),
    /// `o + r cos v (cos u x + sin u y) + r sin v z`, `v ∈ [−π/2, π/2]`.
    Sphere(Frame,f64),
    /// `o + (R + r cos v)(cos u x + sin u y) + r sin v z`.
    Torus(Frame,f64,f64),
    /// `C(u) + v z`: a curve lying in the plane through `o` square to `z`, swept along `z`.
    Extrusion(Frame,Arc<Curve>),
    /// `C(v)` turned by `u` about the axis through `o` along `z`: the curve lies in the half-plane
    /// of `x` (`y = 0`, `x ≥ 0` in the frame).
    Revolution(Frame,Arc<Curve>),
    /// A swept section's face between an edge and its image (`Blend`), its frame the section's.
    Blend(Frame,Arc<Blend>),
    /// A tensor B-spline over its knots' domain (a generating sheet as the native kernel fits it);
    /// its frame is the page's, carrying nothing.
    BSpline(Frame,Arc<super::nurbs::Net>),
    /// A face family's wall (`family::Sheet`, #243 L2): `S(u, s)`, `u` the fraction of its edge's
    /// run and `s` the family's running number; its frame the start section's.
    Family(Frame,Arc<super::family::Sheet>),
}

/// What carries a blended section as `v` runs from 0 to 1: a step along a line (a drafted prism's),
/// or a screw — a turn by `angle` about the axis through `center` along the unit `axis` and a step
/// of `advance` along it (#238: a ball's normal disc carried along a helix).
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Carry { Line { delta: V },Screw { center: V,axis: V,angle: f64,advance: f64 } }

impl Carry {
    /// `p` turned by `angle` about the unit `axis` through the origin, by Rodrigues' formula: the
    /// same turn as `Rigid::turn`'s matrix, but not its bits, so the two stay apart.
    fn turn(axis: V,angle: f64,p: V) -> V {
        let (s,c) = angle.dsin_cos();
        add(add(scale(p,c),scale(cross(axis,p),s)),scale(axis,dot(axis,p)*(1.-c)))
    }
    /// `p` carried to `v`.
    pub fn at(&self,p: V,v: f64) -> V {
        match *self {
            Carry::Line {delta} => add(p,scale(delta,v)),
            Carry::Screw {center,axis,angle,advance} =>
                add(add(center,Carry::turn(axis,angle*v,sub(p,center))),scale(axis,advance*v)),
        }
    }
    /// A direction carried to `v` (turned, never moved).
    pub fn dir(&self,d: V,v: f64) -> V {
        match *self {
            Carry::Line {..} => d,
            Carry::Screw {axis,angle,..} => Carry::turn(axis,angle*v,d),
        }
    }
    /// `d/dv` of `at(p, v)` for a fixed `p`, at the carried point `q = at(p, v)`.
    fn rate(&self,q: V) -> V {
        match *self {
            Carry::Line {delta} => delta,
            Carry::Screw {center,axis,angle,advance} => add(scale(cross(axis,sub(q,center)),angle),scale(axis,advance)),
        }
    }
    pub fn moved(&self,m: &Rigid) -> Carry {
        match *self {
            Carry::Line {delta} => Carry::Line {delta:m.vector(delta)},
            Carry::Screw {center,axis,angle,advance} => Carry::Screw {center:m.point(center),axis:m.vector(axis),angle,advance},
        }
    }
}

thread_local! {
    /// The feet each thread found last on B-spline curves, each with its curve (held, so that its
    /// address is no other's) and the point asked: a trace asks a swept surface its distance and
    /// then its gradient at one point, both through the foot, and a march asks again where it
    /// landed. The answer found once, as it would be found again.
    static HELD_CURVE_FEET: std::cell::RefCell<([Option<(Arc<BSpline>,V,f64)>;8],usize)> =
        const { std::cell::RefCell::new(([const { None };8],0)) };
}

/// `b.nearest(p)`, held for the next to ask of the same curve and point.
fn nearest_held(b: &Arc<BSpline>,p: V) -> f64 {
    let same = |q: &V| (0..3).all(|k| q[k].to_bits() == p[k].to_bits());
    let held = HELD_CURVE_FEET.with(|c| c.borrow().0.iter().flatten().find(|(c,q,_)| Arc::ptr_eq(c,b) && same(q))
        .map(|e| e.2));
    if let Some(t) = held { return t }
    let t = b.nearest(p);
    HELD_CURVE_FEET.with(|c| {
        let (slots,next) = &mut *c.borrow_mut();
        slots[*next] = Some((b.clone(),p,t));
        *next = (*next+1)%slots.len();
    });
    t
}

thread_local! {
    /// The last foot each thread found on each of the last few blends (by their addresses), as a
    /// face family's wall keeps its own: only ever a start, held to settling near the point.
    static LAST_BLEND_FOOT: std::cell::Cell<[(usize,Uv);4]> = const { std::cell::Cell::new([(0,[0.;2]);4]) };
}

/// A swept section's face between one of its edges and that edge's image in the section it is
/// carried to (`build::ruled`): `S(u, v) = carry_v((1 − v) A(a₀ + u Δa) + v B(b₀ + u Δb))`, each
/// edge at the same fraction `u` of its own parameter, `B` the image carried back to the start. A
/// whole-circle pair runs round once, periodic in `u`.
#[derive(Clone,Debug,PartialEq)]
pub struct Blend { pub a: Curve,pub ta: [f64;2],pub b: Curve,pub tb: [f64;2],pub carry: Carry,pub closed: bool }

impl Blend {
    fn ends(&self,u: f64) -> ((V,V,V),(V,V,V)) {
        (self.a.d2(self.ta[0]+u*(self.ta[1]-self.ta[0])),self.b.d2(self.tb[0]+u*(self.tb[1]-self.tb[0])))
    }
    /// `S`, `S_u`, `S_v`.
    pub fn d1(&self,[u,v]: Uv) -> (V,V,V) {
        let ((a,da,_),(b,db,_)) = self.ends(u);
        let p = add(scale(a,1.-v),scale(b,v));
        let pu = add(scale(da,(1.-v)*(self.ta[1]-self.ta[0])),scale(db,v*(self.tb[1]-self.tb[0])));
        let q = self.carry.at(p,v);
        (q,self.carry.dir(pu,v),add(self.carry.dir(sub(b,a),v),self.carry.rate(q)))
    }
    /// The parameters of the point of the patch nearest `p`: Gauss–Newton from the last foot this
    /// thread found on it where that settles near `p` (a mesh asks of one corner after the next, as
    /// of a face family's wall); else from the nearest of a grid, `v` held to `[0, 1]` and `u` too
    /// unless the patch runs round.  A section screwed through more than a turn has a sheet for
    /// each turn, so beside the grid each `u`'s point is tried where the screw turns it to face `p`
    /// (`turned_to`): a grid spaced along `v` alone would start on the next turn of a coil as
    /// readily as on its own.
    pub fn inverse(&self,p: V) -> Uv {
        let key = self as *const Blend as usize;
        let near = 1e-3*(1.+dot(p,p).sqrt());
        let last = LAST_BLEND_FOOT.with(|c| c.get());
        let warm = last.iter().find(|f| f.0 == key).map(|f| self.newton(p,f.1))
            .filter(|&uv| crate::util::space::distance(self.d1(uv).0,p) <= near);
        let uv = warm.unwrap_or_else(|| self.searched(p));
        let mut next = [(key,uv);4];
        let mut k = 1;
        for f in last { if f.0 != key && k < 4 { next[k] = f; k += 1; } }
        LAST_BLEND_FOOT.with(|c| c.set(next));
        uv
    }
    /// `inverse` from the nearest of the grid and the turned points.
    fn searched(&self,p: V) -> Uv {
        const N: usize = 16;
        let mut best = (f64::INFINITY,[0.;2]);
        let mut try_at = |uv: Uv| {
            let d = crate::util::space::distance(self.d1(uv).0,p);
            if d < best.0 { best = (d,uv); }
        };
        for i in 0..=N { for j in 0..=N { try_at([i as f64/N as f64,j as f64/N as f64]); } }
        for i in 0..=N {
            let u = i as f64/N as f64;
            for v in self.turned_to(u,p) { try_at([u,v]); }
        }
        self.newton(p,best.1)
    }
    /// Where along `v` a screw turns the section's point at `u` to face `p` about its axis: every
    /// such `v` in `[0, 1]` (one a turn), none for a step along a line or a point on the axis.
    fn turned_to(&self,u: f64,p: V) -> Vec<f64> {
        let Carry::Screw {center,axis,angle,..} = self.carry else { return Vec::new() };
        if !(angle.abs() > 0.) { return Vec::new() }
        // the angle about the axis read off the start's point (a seed only: where the section's
        // image differs from it, the turn of the point between them is near the screw's)
        let x = self.d1([u,0.]).0;
        let radial = |q: V| { let d = sub(q,center); sub(d,scale(axis,dot(d,axis))) };
        let (rx,rp) = (radial(x),radial(p));
        if !(norm(rx) > 0. && norm(rp) > 0.) { return Vec::new() }
        let phi = dot(axis,cross(rx,rp)).datan2(dot(rx,rp));
        let tau = std::f64::consts::TAU;
        let turns = (angle.abs()/tau).ceil() as i64+1;
        (-turns..=turns).map(|k| (phi+tau*k as f64)/angle).filter(|v| (0.0..=1.0).contains(v)).collect()
    }
    /// Gauss–Newton for the parameters of the patch's point nearest `p`, from `uv`.
    fn newton(&self,p: V,mut uv: Uv) -> Uv {
        for _ in 0..30 {
            let (x,su,sv) = self.d1(uv);
            let e = sub(x,p);
            let (a11,a12,a22) = (dot(su,su),dot(su,sv),dot(sv,sv));
            let (b1,b2) = (dot(su,e),dot(sv,e));
            let det = a11*a22-a12*a12;
            if !(det.abs() > 0.) { break }
            let (du,dv) = ((a22*b1-a12*b2)/det,(a11*b2-a12*b1)/det);
            let mut next = [uv[0]-du,(uv[1]-dv).clamp(0.,1.)];
            next[0] = if self.closed { next[0] } else { next[0].clamp(0.,1.) };
            let moved = (next[0]-uv[0]).abs().max((next[1]-uv[1]).abs());
            uv = next;
            if moved < 1e-15 { break }
        }
        if self.closed { uv[0] = uv[0].rem_euclid(1.); }
        uv
    }
    pub fn moved(&self,m: &Rigid) -> Blend {
        Blend {a:self.a.moved(m),b:self.b.moved(m),carry:self.carry.moved(m),..self.clone()}
    }
    /// The length over which the patch turns appreciably: its edges' and, carried round an
    /// axis, the least distance of its sections from it.
    fn feature(&self) -> f64 {
        let edge = |c: &Curve| match c { Curve::Circle(_,r) => *r,Curve::BSpline(_) => least_radius(c),_ => f64::INFINITY };
        let mut f = edge(&self.a).min(edge(&self.b));
        if let Carry::Screw {center,axis,..} = self.carry {
            for k in 0..=8 { for j in 0..=2 {
                let x = self.d1([k as f64/8.,j as f64/2.]).0;
                f = f.min(norm(cross(sub(x,center),axis)));
            } }
        }
        let size = crate::util::space::distance(self.d1([0.,0.]).0,self.d1([1.,1.]).0)+crate::util::space::distance(self.d1([0.,1.]).0,self.d1([1.,0.]).0);
        f.min(size.max(f64::MIN_POSITIVE))
    }
}

/// A patch's side of `p`: the distance along the normal at its nearest point `uv` — or, where the
/// normal is not defined there (a tangent vanishing), the normal a step toward `middle`, and failing
/// that the direction to the point itself.
fn patch_side(p: V,uv: Uv,middle: Uv,d1: &dyn Fn(Uv) -> (V,V,V)) -> (f64,V) {
    let x = d1(uv).0;
    let normal = |uv: Uv| { let (_,su,sv) = d1(uv); crate::util::space::normalised(cross(su,sv)) };
    let inward = [uv[0]+(middle[0]-uv[0])*1e-6,uv[1]+(middle[1]-uv[1])*1e-6];
    let d = sub(p,x);
    let n = normal(uv).or_else(|| normal(inward)).unwrap_or_else(|| crate::util::space::normalised(d).unwrap_or([0.,0.,1.]));
    (dot(d,n),n)
}

/// Gauss–Newton for the parameters of a B-spline sheet's point nearest `p`, from `uv` and held to
/// the domain: where it reaches a foot — on the sheet, off it along its normal (to a millionth of a
/// radian), or held at the domain's edge with the point beyond it — or `None` where it does not
/// settle.
fn net_newton(n: &super::nurbs::Net,p: V,mut uv: Uv) -> Option<Uv> {
    let [[u0,u1],[v0,v1]] = n.domain();
    let size = 1.+(u1-u0).abs().max((v1-v0).abs());
    let on = 1e-12*(1.+dot(p,p).sqrt());
    uv = [uv[0].clamp(u0,u1),uv[1].clamp(v0,v1)];
    for _ in 0..30 {
        let [x,xu,xv,xuu,xuv,xvv] = n.d2(uv[0],uv[1]);
        let e = sub(x,p);
        let gap = dot(e,e).sqrt();
        let square = |d: V| dot(e,d).abs() <= 1e-6*gap*dot(d,d).sqrt();
        if gap <= on || (square(xu) && square(xv)) { return Some(uv) }
        // Newton on the squared distance where its Hessian is positive definite (a point well off a
        // curved sheet, where Gauss–Newton's first-order model crawls), Gauss–Newton elsewhere
        let (g11,g12,g22) = (dot(xu,xu),dot(xu,xv),dot(xv,xv));
        let (h11,h12,h22) = (g11+dot(e,xuu),g12+dot(e,xuv),g22+dot(e,xvv));
        let (a11,a12,a22) = if h11 > 0. && h22 > 0. && h11*h22-h12*h12 > 0. { (h11,h12,h22) } else { (g11,g12,g22) };
        let (b1,b2) = (dot(xu,e),dot(xv,e));
        // a parameter at the domain's edge with the descent pushing it out is held there, and the
        // other found alone (the joint step would keep moving it along the edge)
        let held_u = (uv[0] <= u0 && b1 > 0.) || (uv[0] >= u1 && b1 < 0.);
        let held_v = (uv[1] <= v0 && b2 > 0.) || (uv[1] >= v1 && b2 < 0.);
        let next = match (held_u,held_v) {
            (true,true) => return Some(uv),
            (true,false) => { if !(a22 > 0.) { return None } [uv[0],(uv[1]-b2/a22).clamp(v0,v1)] }
            (false,true) => { if !(a11 > 0.) { return None } [(uv[0]-b1/a11).clamp(u0,u1),uv[1]] }
            (false,false) => {
                let det = a11*a22-a12*a12;
                if !(det.abs() > 0.) { return None }
                [(uv[0]-(a22*b1-a12*b2)/det).clamp(u0,u1),(uv[1]-(a11*b2-a12*b1)/det).clamp(v0,v1)]
            }
        };
        // a step that moves farther off is halved back toward where it began
        let mut next = next;
        for _ in 0..8 {
            let f = sub(n.point(next[0],next[1]),p);
            if dot(f,f) <= dot(e,e) { break }
            next = [(uv[0]+next[0])/2.,(uv[1]+next[1])/2.];
        }
        let moved = (next[0]-uv[0]).abs().max((next[1]-uv[1]).abs());
        uv = next;
        // still: at a foot, or at the domain's edge with the point beyond it (the nearest the domain has)
        if moved <= 1e-14*size { return Some(uv) }
        // settled along the edge: the free parameter's gap square to the surface
        if held_u || held_v {
            let (x,xu,xv) = n.d1(uv[0],uv[1]);
            let e = sub(x,p);
            let gap = dot(e,e).sqrt();
            let d = if held_u { xv } else { xu };
            if dot(e,d).abs() <= 1e-9*gap*dot(d,d).sqrt() { return Some(uv) }
        }
    }
    None
}

thread_local! {
    /// The last foot each thread found on each of the last few sheets (by their nets' addresses): a
    /// trace asks of one point after another a step apart, on two surfaces in turn, so the next foot is
    /// a Newton step or two from the last.
    static LAST_FOOT: std::cell::Cell<[(usize,Uv);4]> = const { std::cell::Cell::new([(0,[0.;2]);4]) };
}

/// The parameters of a B-spline sheet's point nearest `p`: Gauss–Newton from the last foot this
/// thread found on it where `p` is near the sheet there (within a thousandth of its own size — off
/// the surface a local foot need not be the nearest); else from the nearest pole's Greville
/// parameters (the net of a fitted sheet lies close to it); where that does not settle, from the
/// nearest of a grid across its knot spans.
fn net_inverse(n: &super::nurbs::Net,p: V) -> Uv {
    let key = n as *const _ as usize;
    let near = 1e-3*(1.+dot(p,p).sqrt());
    let last = LAST_FOOT.with(|c| c.get());
    let warm = last.iter().find(|s| s.0 == key).and_then(|s| net_newton(n,p,s.1))
        .filter(|&uv| crate::util::space::distance(n.point(uv[0],uv[1]),p) <= near);
    let uv = warm.unwrap_or_else(|| {
        let greville = |k: &[f64],d: usize,i: usize| k[i+1..=i+d].iter().sum::<f64>()/d as f64;
        // from the nearest poles' Greville points, the nearest first, then the grid
        let mut nearest = [(f64::INFINITY,(0,0));4];
        for (i,row) in n.poles.iter().enumerate() { for (j,&q) in row.iter().enumerate() {
            let e = sub(q,p);
            let d = dot(e,e);
            if d < nearest[3].0 {
                nearest[3] = (d,(i,j));
                nearest.sort_by(|a,b| a.0.total_cmp(&b.0));
            }
        } }
        nearest.iter().filter(|c| c.0.is_finite())
            .find_map(|&(_,at)| net_newton(n,p,[greville(&n.uknots,n.du,at.0),greville(&n.vknots,n.dv,at.1)]))
            .unwrap_or_else(|| net_grid_inverse(n,p))
    });
    // this sheet's foot first, the rest after it, the oldest dropped
    let mut next = [(key,uv);4];
    let mut k = 1;
    for s in last { if s.0 != key && k < 4 { next[k] = s; k += 1; } }
    LAST_FOOT.with(|c| c.set(next));
    uv
}

/// `net_inverse` by the nearest of a grid across the knot spans, then Gauss–Newton.
fn net_grid_inverse(n: &super::nurbs::Net,p: V) -> Uv {
    let [[u0,u1],[v0,v1]] = n.domain();
    // a sample or two per span, at least sixteen a side and at most sixty-four
    let (su,sv) = ((n.poles.len()-n.du).clamp(16,64),(n.poles[0].len()-n.dv).clamp(16,64));
    let mut best = (f64::INFINITY,[u0,v0]);
    for i in 0..=su { for j in 0..=sv {
        let uv = [u0+(u1-u0)*i as f64/su as f64,v0+(v1-v0)*j as f64/sv as f64];
        let d = crate::util::space::distance(n.point(uv[0],uv[1]),p);
        if d < best.0 { best = (d,uv); }
    } }
    let mut uv = best.1;
    for _ in 0..30 {
        let (x,xu,xv) = n.d1(uv[0],uv[1]);
        let e = sub(x,p);
        let (a11,a12,a22) = (dot(xu,xu),dot(xu,xv),dot(xv,xv));
        let (b1,b2) = (dot(xu,e),dot(xv,e));
        let det = a11*a22-a12*a12;
        if !(det.abs() > 0.) { break }
        let next = [(uv[0]-(a22*b1-a12*b2)/det).clamp(u0,u1),(uv[1]-(a11*b2-a12*b1)/det).clamp(v0,v1)];
        let moved = (next[0]-uv[0]).abs().max((next[1]-uv[1]).abs());
        uv = next;
        if moved <= 1e-15*(1.+(u1-u0).abs().max((v1-v0).abs())) { break }
    }
    uv
}

/// The signed distance from `(ρ, z)` to a curve drawn in the same half-plane (`meridian` its
/// `(ρ, z)` at its parameter), its normal the tangent turned clockwise; the foot's parameter too.
fn plane_side(c: &Curve,t: f64,p: [f64;2],meridian: &dyn Fn(V) -> [f64;2],along: &dyn Fn(V) -> [f64;2]) -> (f64,[f64;2]) {
    let (x,d) = c.d1(t);
    let [fr,fz] = meridian(x);
    let [dr,dz] = along(d);
    let l = dr.dhypot(dz);
    if !(l > 0.) { return ((p[0]-fr).dhypot(p[1]-fz),[1.,0.]) }
    let n = [dz/l,-dr/l];
    ((p[0]-fr)*n[0]+(p[1]-fz)*n[1],n)
}

/// An angle in `[0, 2π)`.
fn wrap(a: f64) -> f64 { let a = a.rem_euclid(TAU); if a >= TAU { 0. } else { a } }

/// The least radius of curvature a planar curve has over its domain, sampled (infinite where it
/// is straight), and never more than its extent.
fn least_radius(c: &Curve) -> f64 {
    let Curve::BSpline(b) = c else { return f64::INFINITY };
    let [a,z] = b.domain();
    let mut ts = vec![a];
    let mut cuts = vec![a];
    cuts.extend(b.breaks([a,z]));
    cuts.push(z);
    for w in cuts.windows(2) { for j in 1..=8 { ts.push(w[0]+(w[1]-w[0])*j as f64/8.); } }
    let mut r = b.hull_length();
    for t in ts {
        let (_,d,dd) = b.d2(t);
        let k = norm(cross(d,dd));
        let s = norm(d);
        if k > 0. { r = r.min(s*s*s/k); }
    }
    r.max(f64::MIN_POSITIVE)
}

impl Surface {
    pub fn frame(&self) -> &Frame {
        match self { Surface::Plane(f) | Surface::Cylinder(f,_) | Surface::Cone(f,_,_) | Surface::Sphere(f,_)
            | Surface::Torus(f,_,_) | Surface::Extrusion(f,_) | Surface::Revolution(f,_) | Surface::Blend(f,_) | Surface::BSpline(f,_) | Surface::Family(f,_) => f }
    }
    pub fn kind(&self) -> &'static str {
        match self { Surface::Plane(_) => "plane",Surface::Cylinder(..) => "cylinder",Surface::Cone(..) => "cone",
            Surface::Sphere(..) => "sphere",Surface::Torus(..) => "torus",Surface::Extrusion(..) => "extrusion",
            Surface::Revolution(..) => "revolution",Surface::Blend(..) => "blend",Surface::BSpline(..) => "bspline",Surface::Family(..) => "family" }
    }
    /// The periods in `u` and `v` (none where the parameter is not periodic).
    pub fn periods(&self) -> [Option<f64>;2] {
        match self {
            Surface::Plane(_) | Surface::Extrusion(..) => [None,None],
            Surface::Revolution(..) => [Some(TAU),None],
            Surface::Blend(_,b) => [b.closed.then_some(1.),None],
            Surface::Family(_,w) => [w.period(),None],
            Surface::BSpline(..) => [None,None],
            Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) => [Some(TAU),None],
            Surface::Torus(..) => [Some(TAU),Some(TAU)],
        }
    }
    /// Whether every coordinate is affine along `v` (a plane, cylinder, cone, extrusion, or a blend
    /// carried along a line), so that over any face its extremes lie on the face's boundary: the
    /// line in `v` through an inner point leaves the face across it.
    fn ruled(&self) -> bool {
        match self {
            Surface::Plane(_) | Surface::Cylinder(..) | Surface::Cone(..) | Surface::Extrusion(..) => true,
            Surface::Blend(_,b) => matches!(b.carry,Carry::Line {..}),
            Surface::Sphere(..) | Surface::Torus(..) | Surface::Revolution(..) | Surface::BSpline(..) | Surface::Family(..) => false,
        }
    }
    /// A box about where a face may reach beyond its boundary, never smaller; `range` the
    /// parameters its loops reach, asked only where it is read. Nothing for a `ruled` surface.
    /// About its axis a coordinate of a sphere, torus or revolution is sinusoidal in `u`, so a
    /// face's inner extremes lie on the meridians where it turns (or anywhere, where it does not
    /// turn with `u`, or on the axis: the middle one): each within the range is boxed over its `v`.
    /// A blend screwed about an axis is boxed by the ring its sections turn in; a B-spline by the
    /// poles reaching the range.
    pub fn bounds_over(&self,range: impl FnOnce() -> [[f64;2];2]) -> Box3 {
        if self.ruled() { return EMPTY }
        if let Surface::Blend(_,b) = self {
            let mut sections = b.a.bounds(b.ta);
            widen(&mut sections,b.b.bounds(b.tb));
            return match b.carry {
                Carry::Screw {center,axis,advance,..} => screwed(center,axis,advance,sections),
                Carry::Line {..} => unreachable!("a blend carried along a line is ruled"),
            }
        }
        let [u,v] = range();
        // a family's wall by the ranges its numbers keep to over the stations it reaches
        if let Surface::Family(_,w) = self { return w.bounds(v) }
        let meridians = |f: &Frame,at: &dyn Fn(f64) -> Box3| -> Box3 {
            let mut out = at((u[0]+u[1])/2.);
            for k in 0..3 { for (t,_) in turns(f.x[k],f.y[k],u) { widen(&mut out,at(t)); } }
            out
        };
        // a sphere is a torus about no circle: its meridian the circle `r` about `big` out from `o`
        let tube = |f: &Frame,big: f64,r: f64| meridians(f,&|t: f64| {
            let (s,c) = t.dsin_cos();
            let out = f.dir([c,s,0.]);
            Curve::Circle(Frame {o:add(f.o,scale(out,big)),x:out,y:f.z,z:cross(out,f.z)},r).bounds(v)
        });
        match self {
            Surface::Sphere(f,r) => tube(f,0.,*r),
            Surface::Torus(f,big,r) => tube(f,*big,*r),
            Surface::Revolution(f,c) => match &**c {
                // the poles reaching the range taken once, and turned to each meridian
                Curve::BSpline(b) => {
                    let poles = b.poles_over(v);
                    meridians(f,&|t| { let m = Rigid::turn(f.o,f.z,t); hull(poles.iter().map(|&p| m.point(p))) })
                }
                c => meridians(f,&|t| c.moved(&Rigid::turn(f.o,f.z,t)).bounds(v)),
            },
            Surface::BSpline(_,n) => hull(n.poles_over(u,v)),
            Surface::Plane(_) | Surface::Cylinder(..) | Surface::Cone(..) | Surface::Extrusion(..) | Surface::Blend(..)
                | Surface::Family(..) => unreachable!(),
        }
    }
    /// `S`, `S_u`, `S_v`.
    pub fn d1(&self,[u,v]: Uv) -> (V,V,V) {
        // a plane's point is its frame's, read before any angle (`inverse`)
        if let Surface::Plane(f) = self { return (f.at([u,v,0.]),f.x,f.y) }
        let (su,cu) = u.dsin_cos();
        match self {
            Surface::Extrusion(f,c) => {
                let (x,d) = c.d1(u);
                return (add(x,scale(f.z,v)),d,f.z)
            }
            Surface::Revolution(f,c) => {
                let (x,d) = c.d1(v);
                let (q,e) = (f.local(x),f.dir_local(d));
                let turn = |q: V| [q[0]*cu-q[1]*su,q[0]*su+q[1]*cu,q[2]];
                let r = turn(q);
                return (f.at(r),f.dir([-r[1],r[0],0.]),f.dir(turn(e)))
            }
            Surface::Blend(_,b) => return b.d1([u,v]),
            Surface::Family(_,w) => return w.d1([u,v]),
            Surface::BSpline(_,n) => return n.d1(u,v),
            _ => {}
        }
        match *self {
            Surface::Plane(f) => (f.at([u,v,0.]),f.x,f.y),
            Surface::Cylinder(f,r) => (f.at([r*cu,r*su,v]),f.dir([-r*su,r*cu,0.]),f.z),
            Surface::Cone(f,r,a) => {
                let (sa,ca) = a.dsin_cos();
                let q = r+v*sa;
                (f.at([q*cu,q*su,v*ca]),f.dir([-q*su,q*cu,0.]),f.dir([sa*cu,sa*su,ca]))
            }
            Surface::Sphere(f,r) => {
                let (sv,cv) = v.dsin_cos();
                (f.at([r*cv*cu,r*cv*su,r*sv]),f.dir([-r*cv*su,r*cv*cu,0.]),f.dir([-r*sv*cu,-r*sv*su,r*cv]))
            }
            Surface::Torus(f,big,r) => {
                let (sv,cv) = v.dsin_cos();
                let q = big+r*cv;
                (f.at([q*cu,q*su,r*sv]),f.dir([-q*su,q*cu,0.]),f.dir([-r*sv*cu,-r*sv*su,r*cv]))
            }
            Surface::Extrusion(..) | Surface::Revolution(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..) => unreachable!(),
        }
    }
    pub fn point(&self,uv: Uv) -> V { self.d1(uv).0 }
    /// `S_u × S_v` (not unit; zero at a pole or apex).
    pub fn normal_raw(&self,uv: Uv) -> V { let (_,su,sv) = self.d1(uv); cross(su,sv) }
    /// The unit normal, where it is defined.
    pub fn normal(&self,uv: Uv) -> Option<V> { crate::util::space::normalised(self.normal_raw(uv)) }
    /// The parameters of the surface point nearest `p`, `u` within its period where periodic
    /// (`[0, 2π)`, or `[0, 1)` for a loft face running round). For a
    /// point on the surface this is exact to rounding; off it, the foot of the normal through it
    /// (on the axis, where every `u` is nearest, `u = 0`).
    /// `inverse`, where the point's parameters are known to be near `hint`: a sheet's from there by
    /// Newton where that lands on the surface (within a millionth of the point's size: an edge's
    /// point on the face), and globally otherwise — off the surface a local foot need not be the
    /// nearest. Any other surface's in closed form as ever.
    /// A B-spline sheet's foot of `p` found by Newton from `hint` alone (a local search: the
    /// nearest foot where the hint is nearest it), or none where it does not settle.
    pub fn foot_from(&self,p: V,hint: Uv) -> Option<Uv> {
        match self { Surface::BSpline(_,n) => net_newton(n,p,hint),Surface::Family(_,w) => Some(w.newton(p,hint)),_ => Some(self.inverse_near(p,hint)) }
    }
    /// How far `p` stands from a patch's own points — a sheet's or a loft's, its foot held to its
    /// domain: past the domain's edge its signed distance, read along the normal at the edge, runs on
    /// along the tangent extension, and this does not. So for a revolution or an extrusion of a
    /// spline, past its curve's ends (#196: the end relief's band, a conic turned). `None` for a
    /// surface with no domain's edge.
    pub fn off_patch(&self,p: V) -> Option<f64> {
        match self {
            Surface::BSpline(_,n) => { let uv = net_inverse(n,p); Some(crate::util::space::distance(p,n.point(uv[0],uv[1]))) }
            Surface::Blend(_,b) => Some(crate::util::space::distance(p,b.d1(b.inverse(p)).0)),
            Surface::Family(_,w) => Some(crate::util::space::distance(p,w.d1(w.inverse(p)).0)),
            Surface::Revolution(..) | Surface::Extrusion(..) => Some(crate::util::space::distance(p,self.point(self.inverse(p)))),
            _ => None,
        }
    }
    pub fn inverse_near(&self,p: V,hint: Uv) -> Uv {
        match self {
            Surface::BSpline(_,n) => net_newton(n,p,hint)
                .filter(|&uv| crate::util::space::distance(n.point(uv[0],uv[1]),p) <= 1e-6*(1.+dot(p,p).sqrt()))
                .unwrap_or_else(|| net_inverse(n,p)),
            // from the hint where it settles on the wall near `p`, else the whole wall searched
            Surface::Family(_,w) => {
                let uv = w.newton(p,hint);
                if crate::util::space::distance(w.d1(uv).0,p) <= 1e-6*(1.+dot(p,p).sqrt()) { uv } else { w.inverse(p) }
            }
            // a swept section's patch likewise
            Surface::Blend(_,b) => {
                let uv = b.newton(p,hint);
                if crate::util::space::distance(b.d1(uv).0,p) <= 1e-6*(1.+dot(p,p).sqrt()) { uv } else { b.inverse(p) }
            }
            // a spline swept or turned likewise, its curve's foot from the hint's
            Surface::Extrusion(_,c) | Surface::Revolution(_,c) if matches!(**c,Curve::BSpline(_)) => {
                let Curve::BSpline(b) = &**c else { unreachable!() };
                let uv = self.swept_inverse(p,&|q,k| b.refined(q,hint[k])).expect("a swept curve");
                if crate::util::space::distance(self.point(uv),p) <= 1e-6*(1.+dot(p,p).sqrt()) { uv } else { self.inverse(p) }
            }
            _ => self.inverse(p),
        }
    }
    /// A swept or turned curve's parameters of `p`, its curve's at the foot `foot` finds of the
    /// point in the curve's plane (the curve's parameter the `k`th); `None` for any other surface.
    fn swept_inverse(&self,p: V,foot: &dyn Fn(V,usize) -> f64) -> Option<Uv> {
        let [x,y,z] = self.frame().local(p);
        match self {
            Surface::Extrusion(f,c) => {
                let t = foot(f.at([x,y,0.]),0);
                Some([t,dot(sub(p,c.point(t)),f.z)])
            }
            Surface::Revolution(f,_) => {
                let u = if x == 0. && y == 0. { 0. } else { wrap(y.datan2(x)) };
                Some([u,foot(f.at([x.dhypot(y),0.,z]),1)])
            }
            _ => None,
        }
    }
    pub fn inverse(&self,p: V) -> Uv {
        let f = self.frame();
        let [x,y,z] = f.local(p);
        // a plane's parameters are its frame's coordinates, read before any angle (#589: a large
        // solid's planar faces are inverted at every reading of their edges)
        if let Surface::Plane(_) = self { return [x,y] }
        let u = if x == 0. && y == 0. { 0. } else { wrap(y.datan2(x)) };
        let rho = x.dhypot(y);
        match self {
            Surface::Extrusion(_,c) | Surface::Revolution(_,c) => return self.swept_inverse(p,&|q,_| c.inverse(q)).expect("a swept curve"),
            Surface::Blend(_,b) => return b.inverse(p),
            Surface::BSpline(_,n) => return net_inverse(n,p),
            Surface::Family(_,w) => return w.inverse(p),
            _ => {}
        }
        match *self {
            Surface::Plane(_) => [x,y],
            Surface::Cylinder(..) => [u,z],
            Surface::Cone(_,r,a) => {
                // the generator through u, in the (ρ, z) half-plane: (r, 0) + v (sin α, cos α)
                let (sa,ca) = a.dsin_cos();
                [u,(rho-r)*sa+z*ca]
            }
            Surface::Sphere(..) => [u,z.datan2(rho)],
            Surface::Torus(_,big,_) => [u,wrap(z.datan2(rho-big))],
            Surface::Extrusion(..) | Surface::Revolution(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..) => unreachable!(),
        }
    }
    /// Whether the surface has an offset in closed form: the analytic kinds, not the swept and
    /// fitted ones.
    pub fn has_offset(&self) -> bool {
        !matches!(self,Surface::Extrusion(..) | Surface::Revolution(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..))
    }
    /// What stands `d` off this surface along its own normal (`S_u × S_v`), where its signed
    /// distance (`implicit`) is `d`, in closed form for the analytic kinds (`has_offset`): a
    /// surface — a plane moved, a radius grown or shrunk, a cone shrunk past its apex the cone of
    /// its own apex — or, where `d` takes a radius to nothing (within `tol`), what the surface is
    /// drawn about, which the offset closes onto (`Offset`): a cylinder's axis, a sphere's centre,
    /// a torus's middle circle.  `None` past that, where no point stands `d` off it, and for the
    /// kinds with no offset in closed form.
    pub fn offset(&self,d: f64,tol: f64) -> Option<Offset> {
        // a radius grown or shrunk by `d`: the surface of it, what it closes onto at nothing, or none
        let grown = |r: f64,made: &dyn Fn(f64) -> Surface,closed: Offset| {
            if (r+d).abs() <= tol { Some(closed) } else { (r+d > 0.).then(|| Offset::Surface(made(r+d))) }
        };
        match *self {
            Surface::Plane(f) => Some(Offset::Surface(Surface::Plane(Frame {o:add(f.o,scale(f.z,d)),..f}))),
            Surface::Cylinder(f,r) => grown(r,&|r| Surface::Cylinder(f,r),Offset::Curve(Curve::Line {p:f.o,d:f.z})),
            Surface::Cone(f,r,a) => {
                // implicit (ρ − r) cos α − z sin α = d is the cone of radius r + d / cos α at `o`;
                // where that is none or less, the same sheet placed at its own apex, where the
                // radius `r + z tan α` is nothing
                let r = r+d/a.dcos();
                let tan = a.dtan();
                if r > 0. { Some(Offset::Surface(Surface::Cone(f,r,a))) }
                else if tan.abs() > 1e-12 { Some(Offset::Surface(Surface::Cone(Frame {o:f.at([0.,0.,-r/tan]),..f},0.,a))) }
                else { None }
            }
            Surface::Sphere(f,r) => grown(r,&|r| Surface::Sphere(f,r),Offset::Point(f.o)),
            Surface::Torus(f,big,r) => grown(r,&|r| Surface::Torus(f,big,r),Offset::Curve(Curve::Circle(f,big))),
            Surface::Extrusion(..) | Surface::Revolution(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..) => None,
        }
    }
    /// `offset` where only a surface will do (a leg's meeting, a ball's spine against a wall),
    /// refused saying why: `what` is wider than `face` holds past where the offset closes, and as
    /// wide as it where the offset closes onto what the surface is drawn about, which nothing is
    /// built against yet.
    pub fn offset_surface(&self,d: f64,tol: f64,what: &str,face: &str) -> Result<Surface,String> {
        let onto = match self.offset(d,tol) {
            Some(Offset::Surface(s)) => return Ok(s),
            Some(Offset::Curve(Curve::Line {..})) => "its axis",
            Some(Offset::Curve(_)) => "its middle circle",
            Some(Offset::Point(_)) => "its centre",
            None => return Err(format!("{what} is wider than {face} holds")),
        };
        Err(format!("{what} is as wide as {face} holds: its offset closes onto {onto}, which nothing is built against yet"))
    }
    /// A signed distance whose zero set is the surface (for a cone, the one sheet `ρ ≥ 0` its
    /// parameters reach), positive on the side its normal points to: exact for the plane,
    /// cylinder, sphere and torus. A cone's is the distance to its generating line in the meridian
    /// half-plane; a swept curve's the distance to the tangent line at its foot, which runs on past
    /// the curve's ends; a loft face's the distance along the normal at its nearest point, clamped
    /// to the patch. Off a finite patch those extensions have zeros of their own, which is why
    /// `query::curve_surface` keeps a swept surface's root only where the surface is.
    pub fn implicit(&self,p: V) -> f64 {
        if let Some((d,_)) = self.swept_side(p) { return d }
        let [x,y,z] = self.frame().local(p);
        let rho = x.dhypot(y);
        match *self {
            Surface::Plane(_) => z,
            Surface::Cylinder(_,r) => rho-r,
            Surface::Cone(_,r,a) => { let (sa,ca) = a.dsin_cos(); (rho-r)*ca-z*sa }
            Surface::Sphere(_,r) => rho.dhypot(z)-r,
            Surface::Torus(_,big,r) => (rho-big).dhypot(z)-r,
            Surface::Extrusion(..) | Surface::Revolution(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..) => unreachable!(),
        }
    }
    /// For a swept curve, the signed distance from `p` to the curve in the section through it —
    /// the distance to the tangent line at the foot, as a cone's is to its generator — and that
    /// distance's gradient.
    fn swept_side(&self,p: V) -> Option<(f64,V)> {
        match self {
            Surface::Extrusion(f,c) => {
                let [x,y,_] = f.local(p);
                let t = c.inverse(f.at([x,y,0.]));
                let loc = |q: V| { let l = f.local(q); [l[0],l[1]] };
                let dir = |d: V| { let l = f.dir_local(d); [l[0],l[1]] };
                // the section's (x, y) is right-handed about z, so S_u × z is the tangent turned clockwise
                let (d,n) = plane_side(c,t,[x,y],&loc,&dir);
                Some((d,f.dir([n[0],n[1],0.])))
            }
            Surface::Revolution(f,c) => {
                let [x,y,z] = f.local(p);
                let rho = x.dhypot(y);
                let t = c.inverse(f.at([rho,0.,z]));
                let loc = |q: V| { let l = f.local(q); [l[0],l[2]] };
                let dir = |d: V| { let l = f.dir_local(d); [l[0],l[2]] };
                // in (ρ, z) the normal S_u × S_v is the tangent turned clockwise, as for a plane in (x, y)
                let (d,n) = plane_side(c,t,[rho,z],&loc,&dir);
                let radial = if rho > 0. { [x/rho,y/rho] } else { [1.,0.] };
                Some((d,f.dir([n[0]*radial[0],n[0]*radial[1],n[1]])))
            }
            // a loft's face: the distance along the normal at the nearest point of the patch — or,
            // where the normal is not defined there (an edge's tangent vanishing), the normal a
            // step toward the patch's middle, and failing that the direction to the point itself
            Surface::Blend(_,b) => Some(patch_side(p,b.inverse(p),[0.5,0.5],&|uv| b.d1(uv))),
            // a family's wall likewise, toward the middle of its domain
            Surface::Family(_,w) => {
                let [a,z] = w.span();
                Some(patch_side(p,w.inverse(p),[0.5,(a+z)/2.],&|uv| w.d1(uv)))
            }
            // a B-spline sheet likewise, toward the middle of its domain
            Surface::BSpline(_,n) => {
                let [[u0,u1],[v0,v1]] = n.domain();
                Some(patch_side(p,net_inverse(n,p),[(u0+u1)/2.,(v0+v1)/2.],&|[u,v]| n.d1(u,v)))
            }
            _ => None,
        }
    }
    /// The gradient of `implicit` (unit length wherever the surface's normal is defined).
    pub fn gradient(&self,p: V) -> V {
        if let Some((_,g)) = self.swept_side(p) { return g }
        let f = self.frame();
        let [x,y,z] = f.local(p);
        let rho = x.dhypot(y);
        let radial = if rho > 0. { [x/rho,y/rho,0.] } else { [1.,0.,0.] };
        let l = match *self {
            Surface::Plane(_) => [0.,0.,1.],
            Surface::Cylinder(..) => radial,
            Surface::Cone(_,_,a) => { let (sa,ca) = a.dsin_cos(); [radial[0]*ca,radial[1]*ca,-sa] }
            Surface::Sphere(..) => { let d = rho.dhypot(z); if d > 0. { [x/d,y/d,z/d] } else { [0.,0.,1.] } }
            Surface::Torus(_,big,_) => {
                let (dr,dz) = (rho-big,z);
                let d = dr.dhypot(dz);
                if d > 0. { [radial[0]*dr/d,radial[1]*dr/d,dz/d] } else { [0.,0.,1.] }
            }
            Surface::Extrusion(..) | Surface::Revolution(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..) => unreachable!(),
        };
        f.dir(l)
    }
    /// The length over which the surface turns appreciably, for sampling a curve against it
    /// (infinite for a plane).
    pub fn feature(&self) -> f64 {
        match *self {
            Surface::Plane(_) => f64::INFINITY,
            Surface::Cylinder(_,r) | Surface::Sphere(_,r) | Surface::Torus(_,_,r) => r,
            Surface::Cone(_,r,_) => r.max(f64::MIN_POSITIVE),
            Surface::Extrusion(_,ref c) | Surface::Revolution(_,ref c) => least_radius(c),
            Surface::Blend(_,ref b) => b.feature(),
            Surface::Family(_,ref w) => w.feature(),
            // a length a sheet turns over: a sixteenth of its poles' spread
            Surface::BSpline(_,ref n) => {
                let (lo,hi) = hull(n.poles.iter().flatten().copied());
                (crate::util::space::distance(lo,hi)/16.).max(f64::MIN_POSITIVE)
            }
        }
    }
    /// `feature` within the box `[lo, hi]`: a cone's least radius of curvature there, `ρ / cos α`
    /// at its least radius `ρ` in the box (the radius at its frame's origin says nothing of the
    /// box, and is nothing where that origin is the apex), never less than `feature`'s.
    pub fn feature_within(&self,lo: V,hi: V) -> f64 {
        let Surface::Cone(f,_,a) = *self else { return self.feature() };
        let c = sub(scale(add(lo,hi),0.5),f.o);
        let rho = norm(sub(c,scale(f.z,dot(c,f.z))))-0.5*crate::util::space::distance(lo,hi);
        self.feature().max(rho/a.dcos().abs().max(1e-12))
    }
    pub fn moved(&self,m: &Rigid) -> Surface {
        match *self {
            Surface::Plane(f) => Surface::Plane(f.moved(m)),
            Surface::Cylinder(f,r) => Surface::Cylinder(f.moved(m),r),
            Surface::Cone(f,r,a) => Surface::Cone(f.moved(m),r,a),
            Surface::Sphere(f,r) => Surface::Sphere(f.moved(m),r),
            Surface::Torus(f,big,r) => Surface::Torus(f.moved(m),big,r),
            Surface::Extrusion(f,ref c) => Surface::Extrusion(f.moved(m),Arc::new(c.moved(m))),
            Surface::Revolution(f,ref c) => Surface::Revolution(f.moved(m),Arc::new(c.moved(m))),
            Surface::Blend(f,ref b) => Surface::Blend(f.moved(m),Arc::new(b.moved(m))),
            Surface::Family(f,ref w) => Surface::Family(f.moved(m),Arc::new(w.moved(m))),
            Surface::BSpline(f,ref n) => Surface::BSpline(f.moved(m),Arc::new(super::nurbs::Net {
                poles:n.poles.iter().map(|row| row.iter().map(|&p| m.point(p)).collect()).collect(),..(**n).clone()})),
        }
    }
    /// **The surface reflected by the improper motion `m`** (#578), placed right-handed again
    /// (`Frame::rehanded`): a plane and a B-spline sheet by their normal, an extrusion by its `y`
    /// (its section's frame, which no parameter reads), and a surface of revolution by its `y`,
    /// which runs its `u` the other way — whether it does is returned beside it, so a face's
    /// pcurves follow.  Refused for a blend and a family's wall, whose sweeps a reflection
    /// would turn the other way.
    pub fn reflected(&self,m: &Rigid) -> Result<(Surface,bool),String> {
        let turned = |f: Frame| f.moved(m).rehanded(true);
        Ok(match *self {
            Surface::Plane(f) => (Surface::Plane(f.moved(m).rehanded(false).0),false),
            Surface::Cylinder(f,r) => { let (f,flip) = turned(f); (Surface::Cylinder(f,r),flip) }
            Surface::Cone(f,r,a) => { let (f,flip) = turned(f); (Surface::Cone(f,r,a),flip) }
            Surface::Sphere(f,r) => { let (f,flip) = turned(f); (Surface::Sphere(f,r),flip) }
            Surface::Torus(f,big,r) => { let (f,flip) = turned(f); (Surface::Torus(f,big,r),flip) }
            Surface::Revolution(f,ref c) => {
                let (f,flip) = turned(f);
                (Surface::Revolution(f,Arc::new(c.reflected(m)?)),flip)
            }
            Surface::Extrusion(f,ref c) => (Surface::Extrusion(turned(f).0,Arc::new(c.reflected(m)?)),false),
            Surface::BSpline(..) => {
                let Surface::BSpline(f,net) = self.moved(m) else { unreachable!() };
                (Surface::BSpline(f.rehanded(false).0,net),false)
            }
            Surface::Blend(..) | Surface::Family(..) => return Err(
                "a swept section's face is not reflected yet: a mirrored body is drawn of planes, \
                 surfaces of revolution, extrusions and B-spline sheets".to_string()),
        })
    }
    /// Where, strictly inside `[a, b]` of parameter `k` (0 for u, 1 for v), the surface stops
    /// being smooth: its swept curve's knots.
    pub fn breaks(&self,k: usize,span: [f64;2]) -> Vec<f64> {
        match (self,k) {
            (Surface::Extrusion(_,c),0) | (Surface::Revolution(_,c),1) => c.breaks(span),
            // a loft's edges break at their knots, each at its own fraction of `u`
            (Surface::BSpline(_,n),k) => n.breaks(k,span),
            // a family's wall at its edge's knots, and at the joints of its pieces
            (Surface::Family(_,w),k) => w.breaks(k,span),
            (Surface::Blend(_,b),0) => {
                let mut out = Vec::new();
                for (c,t) in [(&b.a,b.ta),(&b.b,b.tb)] {
                    let (lo,hi) = (t[0].min(t[1]),t[0].max(t[1]));
                    out.extend(c.breaks([lo,hi]).into_iter().map(|k| (k-t[0])/(t[1]-t[0]))
                        .filter(|&u| u > span[0].min(span[1]) && u < span[0].max(span[1])));
                }
                out.sort_by(f64::total_cmp);
                out.dedup();
                out
            }
            _ => vec![],
        }
    }
    /// Whether the surface is one of revolution about its frame's `z` axis — every one but a swept
    /// curve's extrusion, a loft face and the
    /// plane, which is one about its normal through any point.
    pub fn axis(&self) -> Option<(V,V)> {
        match self { Surface::Plane(_) | Surface::Extrusion(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..) => None,_ => { let f = self.frame(); Some((f.o,f.z)) } }
    }
}

/// Two surfaces' intersection with no closed form, traced: points on both, in order, and a
/// parameter that counts them — `C(t)` is the chord between points `⌊t⌋` and `⌊t⌋ + 1` at `t`'s
/// fraction, pulled onto both surfaces by Newton, so the curve is on both to rounding wherever it
/// is read. A closed trace ends where it began (its last point is its first).
#[derive(Clone,Debug,PartialEq)]
pub struct Traced { pub a: Surface,pub b: Surface,pub pts: Vec<V>,pub closed: bool }

impl Traced {
    fn segments(&self) -> usize { self.pts.len()-1 }
    /// The point of both surfaces nearest `q`, by minimum-norm Newton steps.
    pub fn project(&self,q: V) -> V {
        // settled once both distances are at the rounding of the coordinates, a step stops moving,
        // or the distances stop falling (a sheet's are read through a numerical foot, whose noise
        // is above the coordinates' rounding): the nearest iterate, never the unprojected point —
        // at the noise, not short of it, since `d1` differences two projections
        let settled = 4.*f64::EPSILON*(1.+crate::util::space::norm(q));
        let mut p = q;
        let mut best = (f64::INFINITY,q);
        for _ in 0..50 {
            let (fa,fb) = (self.a.implicit(p),self.b.implicit(p));
            let off = fa.abs().max(fb.abs());
            if off >= best.0 { break }
            best = (off,p);
            if off <= settled { break }
            let Some(step) = crate::util::roots::least_norm_step(&[self.a.gradient(p),self.b.gradient(p)],&[fa,fb]) else { break };
            p = sub(p,step);
            if crate::util::space::norm(step) <= settled { best.1 = p; break }
        }
        best.1
    }
    /// `C`: the chord at `t` pulled onto both surfaces.
    pub(crate) fn at(&self,t: f64) -> V {
        let m = self.segments();
        let t = if self.closed { t.rem_euclid(m as f64) } else { t };
        let i = (t.floor().max(0.) as usize).min(m-1);
        self.project(crate::util::space::lerp(self.pts[i],self.pts[i+1],t-i as f64))
    }
    /// `C` and `C'`, the derivative the central difference of `C` itself (the projection of a
    /// chord's point is not the curve's tangent times its speed off the curve): consistent with
    /// the points to ~1e-10 of the speed, which is what an integral along it needs.
    fn d1(&self,t: f64) -> (V,V) {
        const H: f64 = 1e-6;
        let (lo,hi) = if self.closed { (t-H,t+H) } else {
            let m = self.segments() as f64;
            ((t-H).max(0.),(t+H).min(m))
        };
        // never across a point, where the derivative steps
        let k = t.floor();
        let (lo,hi) = (if t-k >= H { lo } else { t },if k+1.-t >= H { hi } else { t });
        let (a,b) = (self.at(lo),self.at(hi));
        (self.at(t),scale(sub(b,a),1./(hi-lo)))
    }
}

/// What a surface offset by a distance is (`Surface::offset`): a surface, or — where the distance
/// takes the surface's radius to nothing — the curve or the point it closes onto: a cylinder's
/// axis, a torus's middle circle (each a `Curve`), a sphere's centre.  A ball as wide as a
/// cylinder rolled inside it has its centre on the axis, nowhere else.
#[derive(Clone,Debug)]
pub enum Offset { Surface(Surface),Curve(Curve),Point(V) }

impl Offset {
    /// The offset where it is a surface.
    pub fn surface(self) -> Option<Surface> { match self { Offset::Surface(s) => Some(s),_ => None } }
}

/// A curve and its parameterisation `C(t)`.
#[derive(Clone,Debug,PartialEq)]
pub enum Curve {
    /// `p + t d`, `d` a unit vector.
    Line { p: V,d: V },
    /// `o + r (cos t x + sin t y)`.
    Circle(Frame,f64),
    /// `o + a cos t x + b sin t y`.
    Ellipse(Frame,f64,f64),
    /// A traced intersection, its parameter counting its points.
    Traced(std::sync::Arc<Traced>),
    /// A non-rational B-spline over its knots' domain.
    BSpline(Arc<BSpline>),
    /// A loft face's rail: its blend at `u` as `v` runs, the curve's parameter `v`.
    Iso(Arc<Blend>,f64),
    /// A face family's rail (#243 L2): its wall at `u` as the family runs, the curve's parameter
    /// the running number `s`.
    Rail(Arc<super::family::Sheet>,f64),
    /// A face family's rim: its wall at the station `s`, the curve's parameter the fraction `u` of
    /// the edge's run.
    Rim(Arc<super::family::Sheet>,f64),
}

impl Curve {
    pub fn kind(&self) -> &'static str {
        match self { Curve::Line {..} => "line",Curve::Circle(..) => "circle",Curve::Ellipse(..) => "ellipse",
            Curve::Traced(..) => "traced",Curve::BSpline(..) => "bspline",Curve::Iso(..) => "iso",
            Curve::Rail(..) => "rail",Curve::Rim(..) => "rim" }
    }
    pub fn period(&self) -> Option<f64> {
        match self {
            Curve::Line {..} | Curve::BSpline(..) | Curve::Iso(..) | Curve::Rail(..) => None,
            Curve::Rim(ref w,_) => w.period(),
            Curve::Traced(c) => c.closed.then(|| c.segments() as f64),
            _ => Some(TAU),
        }
    }
    /// `C`, `C'`, `C''`.
    pub fn d2(&self,t: f64) -> (V,V,V) {
        match *self {
            Curve::Traced(ref c) => { let (x,d) = c.d1(t); (x,d,[0.;3]) }
            Curve::BSpline(ref b) => b.d2(t),
            Curve::Iso(ref b,u) => {
                // the second derivative by a central difference of the first, exact enough for
                // the curvature a sampler reads
                const H: f64 = 1e-5;
                let (x,_,d) = b.d1([u,t]);
                let (lo,hi) = ((t-H).max(0.),(t+H).min(1.));
                (x,d,scale(sub(b.d1([u,hi]).2,b.d1([u,lo]).2),1./(hi-lo)))
            }
            Curve::Rail(ref w,u) => w.rail(u,t),
            Curve::Rim(ref w,s) => w.rim(t,s),
            Curve::Line {p,d} => (add(p,scale(d,t)),d,[0.;3]),
            Curve::Circle(f,r) => {
                let (s,c) = t.dsin_cos();
                (f.at([r*c,r*s,0.]),f.dir([-r*s,r*c,0.]),f.dir([-r*c,-r*s,0.]))
            }
            Curve::Ellipse(f,a,b) => {
                let (s,c) = t.dsin_cos();
                (f.at([a*c,b*s,0.]),f.dir([-a*s,b*c,0.]),f.dir([-a*c,-b*s,0.]))
            }
        }
    }
    /// Where, strictly inside `[a, b]`, the curve stops being smooth: a B-spline's knots, a traced
    /// curve's points.
    pub fn breaks(&self,[a,b]: [f64;2]) -> Vec<f64> {
        let (a,b) = (a.min(b),a.max(b));
        match self {
            Curve::BSpline(s) => s.breaks([a,b]),
            Curve::Traced(_) => (a.floor() as i64+1..=b.ceil() as i64-1).map(|k| k as f64).filter(|&k| k > a && k < b).collect(),
            Curve::Rail(w,_) => w.breaks(1,[a,b]),
            Curve::Rim(w,_) => w.breaks(0,[a,b]),
            _ => vec![],
        }
    }
    pub fn point(&self,t: f64) -> V { if let Curve::BSpline(b) = self { b.point(t) } else { self.d2(t).0 } }
    pub fn tangent(&self,t: f64) -> V { self.d1(t).1 }
    /// `C`, `C'`: `d2`'s, a B-spline's second derivative not read.
    pub fn d1(&self,t: f64) -> (V,V) {
        match self { Curve::BSpline(b) => b.d1(t),_ => { let (x,d,_) = self.d2(t); (x,d) } }
    }
    /// The parameter of the point of the curve nearest `p` (`t ∈ [0, 2π)` on a closed curve);
    /// exact for a point on a line or circle, and for an ellipse refined by Newton from its
    /// eccentric angle.
    pub fn inverse(&self,p: V) -> f64 {
        match *self {
            Curve::Traced(ref c) => {
                // the nearest chord, then Newton on (C − p)·C'
                let mut best = (f64::INFINITY,0.);
                for i in 0..c.segments() {
                    let (a,b) = (c.pts[i],c.pts[i+1]);
                    let d = sub(b,a);
                    let s = (dot(sub(p,a),d)/dot(d,d).max(1e-300)).clamp(0.,1.);
                    let dist = crate::util::space::distance(p,crate::util::space::lerp(a,b,s));
                    if dist < best.0 { best = (dist,i as f64+s); }
                }
                let mut t = best.1;
                for _ in 0..8 {
                    let (x,dx) = c.d1(t);
                    let h = dot(dx,dx);
                    if !(h > 0.) { break }
                    let step = dot(sub(x,p),dx)/h;
                    let next = if c.closed { t-step } else { (t-step).clamp(0.,c.segments() as f64) };
                    if (next-t).abs() < 1e-14 { t = next; break }
                    t = next;
                }
                if c.closed { t.rem_euclid(c.segments() as f64) } else { t }
            }
            Curve::Rail(ref w,u) => w.rail_inverse(u,p),
            Curve::Rim(ref w,s) => w.rim_inverse(s,p),
            Curve::Iso(ref b,u) => {
                let mut best = (f64::INFINITY,0.);
                for j in 0..=32 {
                    let v = j as f64/32.;
                    let d = crate::util::space::distance(b.d1([u,v]).0,p);
                    if d < best.0 { best = (d,v); }
                }
                let mut v = best.1;
                for _ in 0..20 {
                    let (x,d,dd) = self.d2(v);
                    let e = sub(x,p);
                    let h = dot(d,d)+dot(e,dd);
                    if !(h > 0.) { break }
                    let next = (v-dot(e,d)/h).clamp(0.,1.);
                    if (next-v).abs() <= 1e-15 { v = next; break }
                    v = next;
                }
                v
            }
            // the nearest of samples across every knot span, then Newton: once a point
            Curve::BSpline(ref b) => nearest_held(b,p),
            Curve::Line {p: q,d} => dot(sub(p,q),d),
            Curve::Circle(f,_) => { let [x,y,_] = f.local(p); wrap(y.datan2(x)) }
            Curve::Ellipse(f,a,b) => {
                let [x,y,_] = f.local(p);
                let mut t = (y/b).datan2(x/a);
                for _ in 0..8 {
                    let (s,c) = t.dsin_cos();
                    // d/dt ½|C − p|² = (C − p)·C'
                    let g = (a*c-x)*(-a*s)+(b*s-y)*(b*c);
                    let h = a*a*s*s+b*b*c*c+(a*c-x)*(-a*c)+(b*s-y)*(-b*s);
                    if h.abs() < 1e-300 { break }
                    let step = g/h;
                    t -= step;
                    if step.abs() < 1e-15 { break }
                }
                wrap(t)
            }
        }
    }
    /// The curve's length per unit of parameter (its speed, where it is constant; an ellipse's
    /// greatest).
    pub fn speed(&self) -> f64 {
        match *self {
            Curve::Line {..} => 1.,Curve::Circle(_,r) => r,Curve::Ellipse(_,a,b) => a.max(b),
            Curve::Traced(ref c) => c.pts.windows(2).map(|w| crate::util::space::distance(w[0],w[1])).fold(0.,f64::max),
            Curve::BSpline(ref b) => {
                let [a,z] = b.domain();
                (0..=64).map(|j| norm(b.d2(a+(z-a)*j as f64/64.).1)).fold(0.,f64::max)
            }
            Curve::Iso(ref b,u) => (0..=64).map(|j| norm(b.d1([u,j as f64/64.]).2)).fold(0.,f64::max),
            Curve::Rail(ref w,u) => {
                let [a,z] = w.span();
                (0..=64).map(|j| norm(w.d1([u,a+(z-a)*j as f64/64.]).2)).fold(0.,f64::max)
            }
            Curve::Rim(ref w,s) => (0..=64).map(|j| norm(w.rim(j as f64/64.,s).1)).fold(0.,f64::max),
        }
    }
    /// A box about the curve over `[t0, t1]`, never smaller than it: exact for a line, circle and
    /// ellipse (their ends and the turning points between), the poles of a B-spline reaching the
    /// interval (their hull), a blend's rail by its ends where it is carried along a line and by the
    /// ring about the axis it is screwed round otherwise. A traced curve is its points' box grown by
    /// twice the largest sagitta measured at its chords' midpoints: the one bound measured rather
    /// than proved, since what bounds a trace's sag is the spacing it was traced at.
    pub fn bounds(&self,[t0,t1]: [f64;2]) -> Box3 {
        let (t0,t1) = (t0.min(t1),t0.max(t1));
        let ends = || [self.point(t0),self.point(t1)];
        match *self {
            Curve::Line {p,d} => hull([add(p,scale(d,t0)),add(p,scale(d,t1))]),
            Curve::Circle(ref f,r) => conic_bounds(f,r,r,[t0,t1],ends()),
            Curve::Ellipse(ref f,a,b) => conic_bounds(f,a,b,[t0,t1],ends()),
            Curve::BSpline(ref b) => hull(b.poles_over([t0,t1]).iter().copied()),
            Curve::Iso(ref b,u) => match b.carry {
                Carry::Line {..} => hull([t0,t1].map(|v| b.d1([u,v]).0)),
                // before it is carried, the rail is the straight line between its sections
                Carry::Screw {center,axis,advance,..} => {
                    let ((a,_,_),(z,_,_)) = b.ends(u);
                    screwed(center,axis,advance,hull([t0,t1].map(|v| add(scale(a,1.-v),scale(z,v)))))
                }
            },
            // a rail within its wall's box over the stretch of stations, a rim within its edge's
            Curve::Rail(ref w,_) => w.bounds([t0,t1]),
            Curve::Rim(ref w,s) => w.rim_bounds(s,[t0,t1]),
            Curve::Traced(ref c) => {
                // the chords reaching the interval, so its ends' too
                let m = c.segments();
                let mut out = EMPTY;
                let mut sag = 0f64;
                let first = t0.floor();
                let count = ((t1.ceil()-first).max(1.) as usize).min(m);
                for j in 0..count {
                    let i = first as i64+j as i64;
                    let i = if c.closed { i.rem_euclid(m as i64) as usize } else { i.clamp(0,m as i64-1) as usize };
                    let (a,b) = (c.pts[i],c.pts[i+1]);
                    include(&mut out,a);
                    include(&mut out,b);
                    sag = sag.max(crate::util::space::distance(c.at(i as f64+0.5),crate::util::space::lerp(a,b,0.5)));
                }
                (out.0.map(|x| x-2.*sag),out.1.map(|x| x+2.*sag))
            }
        }
    }
    pub fn moved(&self,m: &Rigid) -> Curve {
        match *self {
            Curve::Line {p,d} => Curve::Line {p:m.point(p),d:m.vector(d)},
            Curve::Circle(f,r) => Curve::Circle(f.moved(m),r),
            Curve::Ellipse(f,a,b) => Curve::Ellipse(f.moved(m),a,b),
            Curve::Traced(ref c) => Curve::Traced(std::sync::Arc::new(Traced {a:c.a.moved(m),b:c.b.moved(m),
                pts:c.pts.iter().map(|&p| m.point(p)).collect(),closed:c.closed})),
            Curve::BSpline(ref b) => Curve::BSpline(Arc::new(BSpline {poles:b.poles.iter().map(|&p| m.point(p)).collect(),..(**b).clone()})),
            Curve::Iso(ref b,u) => Curve::Iso(Arc::new(b.moved(m)),u),
            Curve::Rail(ref w,u) => Curve::Rail(Arc::new(w.moved(m)),u),
            Curve::Rim(ref w,s) => Curve::Rim(Arc::new(w.moved(m)),s),
        }
    }
    /// **The curve reflected by the improper motion `m`** (#578): its points the reflection's and
    /// its parameter its own — a circle's or an ellipse's frame made right-handed by its normal,
    /// which its points do not read, and a traced intersection between its two surfaces
    /// reflected.  Refused for a loft's or a family's rail and rim, as their surfaces are.
    pub fn reflected(&self,m: &Rigid) -> Result<Curve,String> {
        Ok(match *self {
            Curve::Line {..} | Curve::BSpline(_) => self.moved(m),
            Curve::Circle(f,r) => Curve::Circle(f.moved(m).rehanded(false).0,r),
            Curve::Ellipse(f,a,b) => Curve::Ellipse(f.moved(m).rehanded(false).0,a,b),
            Curve::Traced(ref c) => Curve::Traced(std::sync::Arc::new(Traced {a:c.a.reflected(m)?.0,
                b:c.b.reflected(m)?.0,pts:c.pts.iter().map(|&p| m.point(p)).collect(),closed:c.closed})),
            Curve::Iso(..) | Curve::Rail(..) | Curve::Rim(..) => return Err(
                "a swept section's edge is not reflected yet".to_string()),
        })
    }
}
