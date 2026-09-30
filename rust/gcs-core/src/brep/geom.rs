//! The analytic geometry a B-rep face and edge lie on: frames, surfaces with their natural
//! parameterisations and closed-form inverses, and curves. Every surface is parameterised as
//! OCCT (and STEP) parameterise it, so a face written out is the face read in.
use crate::space::{add,cross,dot,norm,scale,sub};
use std::sync::Arc;
pub use super::nurbs::BSpline;

pub type V = [f64;3];
pub type Uv = [f64;2];

const TAU: f64 = std::f64::consts::TAU;

fn unit(a: V) -> V { scale(a,1./norm(a)) }

/// A right-handed orthonormal frame: an origin and three axes.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Frame { pub o: V,pub x: V,pub y: V,pub z: V }

impl Frame {
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
    pub fn moved(&self,m: &Rigid) -> Frame {
        Frame {o:m.point(self.o),x:m.vector(self.x),y:m.vector(self.y),z:m.vector(self.z)}
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
        let (s,c) = angle.sin_cos();
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
}

/// The signed distance from `(ρ, z)` to a curve drawn in the same half-plane (`meridian` its
/// `(ρ, z)` at its parameter), its normal the tangent turned clockwise; the foot's parameter too.
fn plane_side(c: &Curve,t: f64,p: [f64;2],meridian: &dyn Fn(V) -> [f64;2],along: &dyn Fn(V) -> [f64;2]) -> (f64,[f64;2]) {
    let (x,d,_) = c.d2(t);
    let [fr,fz] = meridian(x);
    let [dr,dz] = along(d);
    let l = dr.hypot(dz);
    if !(l > 0.) { return ((p[0]-fr).hypot(p[1]-fz),[1.,0.]) }
    let n = [dz/l,-dr/l];
    ((p[0]-fr)*n[0]+(p[1]-fz)*n[1],n)
}

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
            | Surface::Torus(f,_,_) | Surface::Extrusion(f,_) | Surface::Revolution(f,_) => f }
    }
    pub fn kind(&self) -> &'static str {
        match self { Surface::Plane(_) => "plane",Surface::Cylinder(..) => "cylinder",Surface::Cone(..) => "cone",
            Surface::Sphere(..) => "sphere",Surface::Torus(..) => "torus",Surface::Extrusion(..) => "extrusion",
            Surface::Revolution(..) => "revolution" }
    }
    /// The periods in `u` and `v` (none where the parameter is not periodic).
    pub fn periods(&self) -> [Option<f64>;2] {
        match self {
            Surface::Plane(_) | Surface::Extrusion(..) => [None,None],
            Surface::Revolution(..) => [Some(TAU),None],
            Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) => [Some(TAU),None],
            Surface::Torus(..) => [Some(TAU),Some(TAU)],
        }
    }
    /// `S`, `S_u`, `S_v`.
    pub fn d1(&self,[u,v]: Uv) -> (V,V,V) {
        let (su,cu) = u.sin_cos();
        match self {
            Surface::Extrusion(f,c) => {
                let (x,d,_) = c.d2(u);
                return (add(x,scale(f.z,v)),d,f.z)
            }
            Surface::Revolution(f,c) => {
                let (x,d,_) = c.d2(v);
                let (q,e) = (f.local(x),f.dir_local(d));
                let turn = |q: V| [q[0]*cu-q[1]*su,q[0]*su+q[1]*cu,q[2]];
                let r = turn(q);
                return (f.at(r),f.dir([-r[1],r[0],0.]),f.dir(turn(e)))
            }
            _ => {}
        }
        match *self {
            Surface::Plane(f) => (f.at([u,v,0.]),f.x,f.y),
            Surface::Cylinder(f,r) => (f.at([r*cu,r*su,v]),f.dir([-r*su,r*cu,0.]),f.z),
            Surface::Cone(f,r,a) => {
                let (sa,ca) = a.sin_cos();
                let q = r+v*sa;
                (f.at([q*cu,q*su,v*ca]),f.dir([-q*su,q*cu,0.]),f.dir([sa*cu,sa*su,ca]))
            }
            Surface::Sphere(f,r) => {
                let (sv,cv) = v.sin_cos();
                (f.at([r*cv*cu,r*cv*su,r*sv]),f.dir([-r*cv*su,r*cv*cu,0.]),f.dir([-r*sv*cu,-r*sv*su,r*cv]))
            }
            Surface::Torus(f,big,r) => {
                let (sv,cv) = v.sin_cos();
                let q = big+r*cv;
                (f.at([q*cu,q*su,r*sv]),f.dir([-q*su,q*cu,0.]),f.dir([-r*sv*cu,-r*sv*su,r*cv]))
            }
            Surface::Extrusion(..) | Surface::Revolution(..) => unreachable!(),
        }
    }
    pub fn point(&self,uv: Uv) -> V { self.d1(uv).0 }
    /// `S_u × S_v` (not unit; zero at a pole or apex).
    pub fn normal_raw(&self,uv: Uv) -> V { let (_,su,sv) = self.d1(uv); cross(su,sv) }
    /// The unit normal, where it is defined.
    pub fn normal(&self,uv: Uv) -> Option<V> { crate::space::normalised(self.normal_raw(uv)) }
    /// The parameters of the surface point nearest `p`, `u` in `[0, 2π)` where periodic. For a
    /// point on the surface this is exact to rounding; off it, the foot of the normal through it
    /// (on the axis, where every `u` is nearest, `u = 0`).
    pub fn inverse(&self,p: V) -> Uv {
        let wrap = |a: f64| { let a = a.rem_euclid(TAU); if a >= TAU { 0. } else { a } };
        let f = self.frame();
        let [x,y,z] = f.local(p);
        let u = if x == 0. && y == 0. { 0. } else { wrap(y.atan2(x)) };
        let rho = x.hypot(y);
        match self {
            Surface::Extrusion(_,c) => {
                let t = c.inverse(f.at([x,y,0.]));
                return [t,dot(sub(p,c.point(t)),f.z)]
            }
            Surface::Revolution(_,c) => return [u,c.inverse(f.at([rho,0.,z]))],
            _ => {}
        }
        match *self {
            Surface::Plane(_) => [x,y],
            Surface::Cylinder(..) => [u,z],
            Surface::Cone(_,r,a) => {
                // the generator through u, in the (ρ, z) half-plane: (r, 0) + v (sin α, cos α)
                let (sa,ca) = a.sin_cos();
                [u,(rho-r)*sa+z*ca]
            }
            Surface::Sphere(..) => [u,z.atan2(rho)],
            Surface::Torus(_,big,_) => [u,wrap(z.atan2(rho-big))],
            Surface::Extrusion(..) | Surface::Revolution(..) => unreachable!(),
        }
    }
    /// A signed distance whose zero set is the surface (for a cone, the one sheet `ρ ≥ 0` its
    /// parameters reach), positive on the side its normal points to: exact for all but the cone,
    /// whose value is the distance to its generating line in the meridian half-plane.
    pub fn implicit(&self,p: V) -> f64 {
        if let Some((d,_)) = self.swept_side(p) { return d }
        let [x,y,z] = self.frame().local(p);
        let rho = x.hypot(y);
        match *self {
            Surface::Plane(_) => z,
            Surface::Cylinder(_,r) => rho-r,
            Surface::Cone(_,r,a) => { let (sa,ca) = a.sin_cos(); (rho-r)*ca-z*sa }
            Surface::Sphere(_,r) => rho.hypot(z)-r,
            Surface::Torus(_,big,r) => (rho-big).hypot(z)-r,
            Surface::Extrusion(..) | Surface::Revolution(..) => unreachable!(),
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
                let rho = x.hypot(y);
                let t = c.inverse(f.at([rho,0.,z]));
                let loc = |q: V| { let l = f.local(q); [l[0],l[2]] };
                let dir = |d: V| { let l = f.dir_local(d); [l[0],l[2]] };
                // in (ρ, z) the normal S_u × S_v is the tangent turned clockwise, as for a plane in (x, y)
                let (d,n) = plane_side(c,t,[rho,z],&loc,&dir);
                let radial = if rho > 0. { [x/rho,y/rho] } else { [1.,0.] };
                Some((d,f.dir([n[0]*radial[0],n[0]*radial[1],n[1]])))
            }
            _ => None,
        }
    }
    /// The gradient of `implicit` (unit length wherever the surface's normal is defined).
    pub fn gradient(&self,p: V) -> V {
        if let Some((_,g)) = self.swept_side(p) { return g }
        let f = self.frame();
        let [x,y,z] = f.local(p);
        let rho = x.hypot(y);
        let radial = if rho > 0. { [x/rho,y/rho,0.] } else { [1.,0.,0.] };
        let l = match *self {
            Surface::Plane(_) => [0.,0.,1.],
            Surface::Cylinder(..) => radial,
            Surface::Cone(_,_,a) => { let (sa,ca) = a.sin_cos(); [radial[0]*ca,radial[1]*ca,-sa] }
            Surface::Sphere(..) => { let d = rho.hypot(z); if d > 0. { [x/d,y/d,z/d] } else { [0.,0.,1.] } }
            Surface::Torus(_,big,_) => {
                let (dr,dz) = (rho-big,z);
                let d = dr.hypot(dz);
                if d > 0. { [radial[0]*dr/d,radial[1]*dr/d,dz/d] } else { [0.,0.,1.] }
            }
            Surface::Extrusion(..) | Surface::Revolution(..) => unreachable!(),
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
        }
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
        }
    }
    /// Where, strictly inside `[a, b]` of parameter `k` (0 for u, 1 for v), the surface stops
    /// being smooth: its swept curve's knots.
    pub fn breaks(&self,k: usize,span: [f64;2]) -> Vec<f64> {
        match (self,k) {
            (Surface::Extrusion(_,c),0) | (Surface::Revolution(_,c),1) => c.breaks(span),
            _ => vec![],
        }
    }
    /// Whether the surface is one of revolution about its frame's `z` axis — every one but the
    /// plane, which is one about its normal through any point.
    pub fn axis(&self) -> Option<(V,V)> {
        match self { Surface::Plane(_) | Surface::Extrusion(..) => None,_ => { let f = self.frame(); Some((f.o,f.z)) } }
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
        // settled once both distances are at the rounding of the coordinates, or a step stops
        // moving; the last iterate either way, never the unprojected point
        let settled = 4.*f64::EPSILON*(1.+crate::space::norm(q));
        let mut p = q;
        for _ in 0..50 {
            let (fa,fb) = (self.a.implicit(p),self.b.implicit(p));
            if fa.abs() <= settled && fb.abs() <= settled { break }
            let Some(step) = crate::roots::least_norm_step(&[self.a.gradient(p),self.b.gradient(p)],&[fa,fb]) else { break };
            p = sub(p,step);
            if crate::space::norm(step) <= settled { break }
        }
        p
    }
    /// `C`: the chord at `t` pulled onto both surfaces.
    fn at(&self,t: f64) -> V {
        let m = self.segments();
        let t = if self.closed { t.rem_euclid(m as f64) } else { t };
        let i = (t.floor().max(0.) as usize).min(m-1);
        self.project(crate::space::lerp(self.pts[i],self.pts[i+1],t-i as f64))
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
}

impl Curve {
    pub fn kind(&self) -> &'static str {
        match self { Curve::Line {..} => "line",Curve::Circle(..) => "circle",Curve::Ellipse(..) => "ellipse",
            Curve::Traced(..) => "traced",Curve::BSpline(..) => "bspline" }
    }
    pub fn period(&self) -> Option<f64> {
        match self {
            Curve::Line {..} | Curve::BSpline(..) => None,
            Curve::Traced(c) => c.closed.then(|| c.segments() as f64),
            _ => Some(TAU),
        }
    }
    /// `C`, `C'`, `C''`.
    pub fn d2(&self,t: f64) -> (V,V,V) {
        match *self {
            Curve::Traced(ref c) => { let (x,d) = c.d1(t); (x,d,[0.;3]) }
            Curve::BSpline(ref b) => b.d2(t),
            Curve::Line {p,d} => (add(p,scale(d,t)),d,[0.;3]),
            Curve::Circle(f,r) => {
                let (s,c) = t.sin_cos();
                (f.at([r*c,r*s,0.]),f.dir([-r*s,r*c,0.]),f.dir([-r*c,-r*s,0.]))
            }
            Curve::Ellipse(f,a,b) => {
                let (s,c) = t.sin_cos();
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
            _ => vec![],
        }
    }
    pub fn point(&self,t: f64) -> V { self.d2(t).0 }
    pub fn tangent(&self,t: f64) -> V { self.d2(t).1 }
    /// The parameter of the point of the curve nearest `p` (`t ∈ [0, 2π)` on a closed curve);
    /// exact for a point on a line or circle, and for an ellipse refined by Newton from its
    /// eccentric angle.
    pub fn inverse(&self,p: V) -> f64 {
        let wrap = |a: f64| { let a = a.rem_euclid(TAU); if a >= TAU { 0. } else { a } };
        match *self {
            Curve::Traced(ref c) => {
                // the nearest chord, then Newton on (C − p)·C'
                let mut best = (f64::INFINITY,0.);
                for i in 0..c.segments() {
                    let (a,b) = (c.pts[i],c.pts[i+1]);
                    let d = sub(b,a);
                    let s = (dot(sub(p,a),d)/dot(d,d).max(1e-300)).clamp(0.,1.);
                    let dist = crate::space::distance(p,crate::space::lerp(a,b,s));
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
            Curve::BSpline(ref b) => {
                // the nearest of samples across every knot span, then Newton on (C − p)·C'
                let [a,z] = b.domain();
                let mut cuts = vec![a];
                cuts.extend(b.breaks([a,z]));
                cuts.push(z);
                let mut best = (f64::INFINITY,a);
                for w in cuts.windows(2) { for j in 0..=16 {
                    let t = w[0]+(w[1]-w[0])*j as f64/16.;
                    let d = crate::space::distance(b.point(t),p);
                    if d < best.0 { best = (d,t); }
                } }
                let mut t = best.1;
                for _ in 0..20 {
                    let (x,d,dd) = b.d2(t);
                    let e = sub(x,p);
                    let g = dot(e,d);
                    let h = dot(d,d)+dot(e,dd);
                    if !(h > 0.) { break }
                    let next = (t-g/h).clamp(a,z);
                    if (next-t).abs() <= 1e-15*(z-a) { t = next; break }
                    t = next;
                }
                t
            }
            Curve::Line {p: q,d} => dot(sub(p,q),d),
            Curve::Circle(f,_) => { let [x,y,_] = f.local(p); wrap(y.atan2(x)) }
            Curve::Ellipse(f,a,b) => {
                let [x,y,_] = f.local(p);
                let mut t = (y/b).atan2(x/a);
                for _ in 0..8 {
                    let (s,c) = t.sin_cos();
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
            Curve::Traced(ref c) => c.pts.windows(2).map(|w| crate::space::distance(w[0],w[1])).fold(0.,f64::max),
            Curve::BSpline(ref b) => {
                let [a,z] = b.domain();
                (0..=64).map(|j| norm(b.d2(a+(z-a)*j as f64/64.).1)).fold(0.,f64::max)
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
        }
    }
}
