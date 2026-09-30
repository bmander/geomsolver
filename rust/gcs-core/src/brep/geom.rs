//! The analytic geometry a B-rep face and edge lie on: frames, surfaces with their natural
//! parameterisations and closed-form inverses, and curves. Every surface is parameterised as
//! OCCT (and STEP) parameterise it, so a face written out is the face read in.
use crate::space::{add,cross,dot,norm,scale,sub};

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
#[derive(Clone,Copy,Debug,PartialEq)]
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
}

impl Surface {
    pub fn frame(&self) -> &Frame {
        match self { Surface::Plane(f) | Surface::Cylinder(f,_) | Surface::Cone(f,_,_) | Surface::Sphere(f,_)
            | Surface::Torus(f,_,_) => f }
    }
    pub fn kind(&self) -> &'static str {
        match self { Surface::Plane(_) => "plane",Surface::Cylinder(..) => "cylinder",Surface::Cone(..) => "cone",
            Surface::Sphere(..) => "sphere",Surface::Torus(..) => "torus" }
    }
    /// The periods in `u` and `v` (none where the parameter is not periodic).
    pub fn periods(&self) -> [Option<f64>;2] {
        match self {
            Surface::Plane(_) => [None,None],
            Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) => [Some(TAU),None],
            Surface::Torus(..) => [Some(TAU),Some(TAU)],
        }
    }
    /// `S`, `S_u`, `S_v`.
    pub fn d1(&self,[u,v]: Uv) -> (V,V,V) {
        let (su,cu) = u.sin_cos();
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
        }
    }
    pub fn moved(&self,m: &Rigid) -> Surface {
        match *self {
            Surface::Plane(f) => Surface::Plane(f.moved(m)),
            Surface::Cylinder(f,r) => Surface::Cylinder(f.moved(m),r),
            Surface::Cone(f,r,a) => Surface::Cone(f.moved(m),r,a),
            Surface::Sphere(f,r) => Surface::Sphere(f.moved(m),r),
            Surface::Torus(f,big,r) => Surface::Torus(f.moved(m),big,r),
        }
    }
    /// Whether the surface is one of revolution about its frame's `z` axis — every one but the
    /// plane, which is one about its normal through any point.
    pub fn axis(&self) -> Option<(V,V)> {
        match self { Surface::Plane(_) => None,_ => { let f = self.frame(); Some((f.o,f.z)) } }
    }
}

/// A curve and its parameterisation `C(t)`.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Curve {
    /// `p + t d`, `d` a unit vector.
    Line { p: V,d: V },
    /// `o + r (cos t x + sin t y)`.
    Circle(Frame,f64),
    /// `o + a cos t x + b sin t y`.
    Ellipse(Frame,f64,f64),
}

impl Curve {
    pub fn kind(&self) -> &'static str {
        match self { Curve::Line {..} => "line",Curve::Circle(..) => "circle",Curve::Ellipse(..) => "ellipse" }
    }
    pub fn period(&self) -> Option<f64> { match self { Curve::Line {..} => None,_ => Some(TAU) } }
    /// `C`, `C'`, `C''`.
    pub fn d2(&self,t: f64) -> (V,V,V) {
        match *self {
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
    pub fn point(&self,t: f64) -> V { self.d2(t).0 }
    pub fn tangent(&self,t: f64) -> V { self.d2(t).1 }
    /// The parameter of the point of the curve nearest `p` (`t ∈ [0, 2π)` on a closed curve);
    /// exact for a point on a line or circle, and for an ellipse refined by Newton from its
    /// eccentric angle.
    pub fn inverse(&self,p: V) -> f64 {
        let wrap = |a: f64| { let a = a.rem_euclid(TAU); if a >= TAU { 0. } else { a } };
        match *self {
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
    pub fn moved(&self,m: &Rigid) -> Curve {
        match *self {
            Curve::Line {p,d} => Curve::Line {p:m.point(p),d:m.vector(d)},
            Curve::Circle(f,r) => Curve::Circle(f.moved(m),r),
            Curve::Ellipse(f,a,b) => Curve::Ellipse(f.moved(m),a,b),
        }
    }
}
