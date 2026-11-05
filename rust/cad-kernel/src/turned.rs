//! **A round face's contact under a turn, turned** (#780 C3): the meridian, read exactly, of the
//! curve along which a cylinder or a cone about an axis leaning to a turn's touches the surface it
//! sweeps.
//!
//! Under a turn about the axis `Z` through `c`, a point `x` of a face is on its contact where the
//! face's normal is square to the turn's velocity, `n · (Z × (x − c)) = 0`. On the face's ring at
//! slant `v` along its rulings — centre `m(v) = o + v cos α a`, radius `R(v) = r + v sin α`, its
//! points `m + R ρ̂(u)` with normal `cos α ρ̂ − sin α a` — that is
//!
//! ```text
//!     ρ̂ · W = C,   W = cos α (Z × q) − R sin α (a × Z) (its part in the ring's plane),
//!                  C = sin α a · (Z × q),   q = m − c,
//! ```
//!
//! `A cos u + B sin u = C` in the ring's own frame: two roots on each ring where `|W| > |C|`,
//! written without an angle as `ρ̂ = (C W ± √(|W|² − C²) a × W) / |W|²`. Each sign is a contact
//! curve `x(v) = m(v) + R(v) ρ̂(v)` along the face, and turned about `Z` it sweeps a surface of
//! revolution whose meridian is where its points go in `(ρ, z)`: `ρ = |x − c|` square to `Z`, `z`
//! along it. That meridian is held here as what it is — a closed form in `v` with its derivatives
//! — not fitted (`Turned`, `geom::Curve::Turned`).
//!
//! Its jets are one formula read two ways (`Num`): in floating point for its point and derivatives,
//! and in outward-rounded intervals (`util::interval`) for bounds over a stretch of `v` that hold —
//! what a certified reading of a line against the surface needs (`query::line_revolution`), and the
//! curve's box (`Turned::bounds`).
use crate::geom::{Box3,Frame,Rigid,V};
use crate::util::interval::Interval;
use crate::util::space::{dot,sub};
use std::ops::{Add,Div,Mul,Neg,Sub};

/// A number a jet is read in: a float, or an outward-rounded interval (`Iv`).
pub trait Num: Copy+Add<Output=Self>+Sub<Output=Self>+Mul<Output=Self>+Div<Output=Self>+Neg<Output=Self> {
    /// The number `x`.
    fn of(x: f64) -> Self;
    /// Its square root.
    fn sqrt(self) -> Self;
}

impl Num for f64 {
    fn of(x: f64) -> f64 { x }
    fn sqrt(self) -> f64 { f64::sqrt(self) }
}

/// An outward-rounded interval, or none where an operation left its domain (a division by an
/// interval holding nought, a root of one reaching below it): every reading through it then none.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Iv(pub Option<Interval>);

impl Iv {
    /// The interval `[lo, hi]`.
    pub fn new(lo: f64,hi: f64) -> Iv { Iv(Interval::new(lo,hi).ok()) }
    /// Its ends, where it is one.
    pub fn bounds(self) -> Option<[f64;2]> { self.0.map(|i| i.bounds()) }
    /// The most magnitude it holds, where it is one.
    pub fn most(self) -> Option<f64> { self.bounds().map(|[a,b]| a.abs().max(b.abs())) }
}

impl Add for Iv { type Output = Iv; fn add(self,b: Iv) -> Iv { Iv(self.0.zip(b.0).and_then(|(a,b)| a.add(b).ok())) } }
impl Sub for Iv { type Output = Iv; fn sub(self,b: Iv) -> Iv { Iv(self.0.zip(b.0).and_then(|(a,b)| a.sub(b).ok())) } }
impl Mul for Iv { type Output = Iv; fn mul(self,b: Iv) -> Iv { Iv(self.0.zip(b.0).and_then(|(a,b)| a.mul(b).ok())) } }
impl Div for Iv { type Output = Iv; fn div(self,b: Iv) -> Iv { Iv(self.0.zip(b.0).and_then(|(a,b)| a.div(b).ok())) } }
impl Neg for Iv { type Output = Iv; fn neg(self) -> Iv { Iv(self.0.map(|a| a.neg())) } }

impl Num for Iv {
    fn of(x: f64) -> Iv { Iv(Interval::point(x).ok()) }
    fn sqrt(self) -> Iv { Iv(self.0.and_then(|a| a.sqrt().ok())) }
}

/// A number and its first two derivatives along one parameter.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Jet<T> { pub v: T,pub d: T,pub dd: T }

impl<T: Num> Jet<T> {
    /// The constant `x`.
    pub fn of(x: f64) -> Jet<T> { Jet {v:T::of(x),d:T::of(0.),dd:T::of(0.)} }
    /// The parameter itself, at `t`.
    pub fn at(t: T) -> Jet<T> { Jet {v:t,d:T::of(1.),dd:T::of(0.)} }
    /// Scaled by the constant `k`.
    pub fn times(self,k: f64) -> Jet<T> { let k = T::of(k); Jet {v:self.v*k,d:self.d*k,dd:self.dd*k} }
    /// Its square root: `s' = x' / 2s`, `s'' = (x'' − 2 s'²) / 2s`.
    pub fn sqrt(self) -> Jet<T> {
        let s = self.v.sqrt();
        let two = T::of(2.);
        let d = self.d/(two*s);
        Jet {v:s,d,dd:(self.dd-two*d*d)/(two*s)}
    }
}

impl<T: Num> Add for Jet<T> { type Output = Jet<T>; fn add(self,b: Jet<T>) -> Jet<T> { Jet {v:self.v+b.v,d:self.d+b.d,dd:self.dd+b.dd} } }
impl<T: Num> Sub for Jet<T> { type Output = Jet<T>; fn sub(self,b: Jet<T>) -> Jet<T> { Jet {v:self.v-b.v,d:self.d-b.d,dd:self.dd-b.dd} } }
impl<T: Num> Neg for Jet<T> { type Output = Jet<T>; fn neg(self) -> Jet<T> { Jet {v:-self.v,d:-self.d,dd:-self.dd} } }
impl<T: Num> Mul for Jet<T> {
    type Output = Jet<T>;
    fn mul(self,b: Jet<T>) -> Jet<T> {
        Jet {v:self.v*b.v,d:self.d*b.v+self.v*b.d,dd:self.dd*b.v+T::of(2.)*self.d*b.d+self.v*b.dd}
    }
}
impl<T: Num> Div for Jet<T> {
    type Output = Jet<T>;
    /// `q = a / b`: `q' = (a' − q b') / b`, `q'' = (a'' − 2 q' b' − q b'') / b`.
    fn div(self,b: Jet<T>) -> Jet<T> {
        let v = self.v/b.v;
        let d = (self.d-v*b.d)/b.v;
        Jet {v,d,dd:(self.dd-T::of(2.)*d*b.d-v*b.dd)/b.v}
    }
}

type J3<T> = [Jet<T>;3];

fn dot3<T: Num>(a: J3<T>,b: J3<T>) -> Jet<T> { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn cross3<T: Num>(a: J3<T>,b: J3<T>) -> J3<T> { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn constant<T: Num>(x: V) -> J3<T> { x.map(Jet::of) }

/// **The contact of a cylinder or a cone with the surface it sweeps under a turn, on one side**
/// (the module's): the face's axis through `o` along the unit `a`, its ring frame's `x` and `y`
/// square to it, its radius `r` at `v = 0` and its half-angle's sine and cosine (`sa = 0`, `ca = 1`
/// a cylinder's), and which of the two roots on each ring (`side`, ±1) — all in the turn's own
/// frame, whose origin is on the turn's axis and whose `z` is along it.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Contact { pub o: V,pub a: V,pub r: f64,pub sa: f64,pub ca: f64,pub side: f64 }

impl Contact {
    /// The contact's point at the jet `v` of the face's slant, and the discriminant `|W|² − C²`
    /// whose root it takes (two contacts on the ring where it is above nought, none below).
    pub fn point<T: Num>(&self,v: Jet<T>) -> (J3<T>,Jet<T>) {
        let a: J3<T> = constant(self.a);
        let m: J3<T> = std::array::from_fn(|i| Jet::of(self.o[i])+v.times(self.ca*self.a[i]));
        let radius = Jet::of(self.r)+v.times(self.sa);
        // Z × q, and a × Z (constant)
        let zq: J3<T> = [-m[1],m[0],Jet::of(0.)];
        let az: J3<T> = constant([self.a[1],-self.a[0],0.]);
        let w3: J3<T> = std::array::from_fn(|i| zq[i].times(self.ca)-radius.times(self.sa)*az[i]);
        let along = dot3(w3,a);
        let w: J3<T> = std::array::from_fn(|i| w3[i]-along*a[i]);
        let c = dot3(a,zq).times(self.sa);
        let ww = dot3(w,w);
        let disc = ww-c*c;
        let across = cross3(a,w);
        let root = disc.sqrt().times(self.side);
        let x = std::array::from_fn(|i| m[i]+radius*(c*w[i]+root*across[i])/ww);
        (x,disc)
    }
    /// Where the contact at `v` goes in `(ρ, z)` about the turn's axis, each with its jet.
    pub fn meridian<T: Num>(&self,v: Jet<T>) -> [Jet<T>;2] {
        let (x,_) = self.point(v);
        [(x[0]*x[0]+x[1]*x[1]).sqrt(),x[2]]
    }
}

/// **The meridian of a contact turned** (`geom::Curve::Turned`): the curve `frame.at([ρ(v), 0,
/// z(v)])`, `v` the face's slant over `span`, in the half-plane of `frame`'s `x` and `z` — `frame`
/// the turn's axis (its origin on it, its `z` along it), the contact read in its coordinates. Or,
/// `posed` by an angle `θ`, the contact itself turned by `θ` about that axis: where the face at that
/// pose of the turn touches the meridian's surface of revolution (`ssi`'s `own_contact`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Turned { pub frame: Frame,pub contact: Contact,pub span: [f64;2],pub posed: Option<f64> }

impl Turned {
    /// The meridian of `contact` turned about `frame`'s axis, over `span`.
    pub fn meridian(frame: Frame,contact: Contact,span: [f64;2]) -> Turned { Turned {frame,contact,span,posed:None} }
    /// `M`, `M'`, `M''` at `v`.
    pub fn d2(&self,v: f64) -> (V,V,V) {
        let f = &self.frame;
        if let Some(theta) = self.posed {
            let (x,_) = self.contact.point(Jet::at(v));
            let (s,c) = (theta.sin(),theta.cos());
            let turn = |p: [f64;3]| [c*p[0]-s*p[1],s*p[0]+c*p[1],p[2]];
            return (f.at(turn(x.map(|j| j.v))),f.dir(turn(x.map(|j| j.d))),f.dir(turn(x.map(|j| j.dd))))
        }
        let [r,z] = self.contact.meridian(Jet::at(v));
        (f.at([r.v,0.,z.v]),f.dir([r.d,0.,z.d]),f.dir([r.dd,0.,z.dd]))
    }
    /// `ρ` and `z` with their first two derivatives over the stretch `[a, b]` of `v`, each held in
    /// an interval (none where a reading left its domain over it).
    pub fn bounded(&self,[a,b]: [f64;2]) -> Option<[[[f64;2];3];2]> {
        let [r,z] = self.contact.meridian(Jet::at(Iv::new(a.min(b),a.max(b))));
        let three = |j: Jet<Iv>| Some([j.v.bounds()?,j.d.bounds()?,j.dd.bounds()?]);
        Some([three(r)?,three(z)?])
    }
    /// Whether the contact is a curve over the stretch `[a, b]`, proved: its discriminant above
    /// nought (two contacts on every ring, apart), and it off the turn's axis, all along.
    pub fn defined(&self,[a,b]: [f64;2]) -> bool {
        const PIECES: usize = 64;
        (0..PIECES).all(|k| {
            let (s,t) = (a+(b-a)*k as f64/PIECES as f64,a+(b-a)*(k+1) as f64/PIECES as f64);
            let v = Jet::at(Iv::new(s.min(t),s.max(t)));
            let (x,disc) = self.contact.point(v);
            let rho2 = x[0].v*x[0].v+x[1].v*x[1].v;
            disc.v.bounds().is_some_and(|[lo,_]| lo > 0.) && rho2.bounds().is_some_and(|[lo,_]| lo > 0.)
        })
    }
    /// Whether its height runs one way over the stretch `[a, b]`, proved: `z'` held off nought.
    pub fn climbs(&self,[a,b]: [f64;2]) -> bool {
        const PIECES: usize = 64;
        let signs: Vec<Option<f64>> = (0..PIECES).map(|k| {
            let (s,t) = (a+(b-a)*k as f64/PIECES as f64,a+(b-a)*(k+1) as f64/PIECES as f64);
            let [lo,hi] = self.bounded([s,t])?[1][1];
            if lo > 0. { Some(1.) } else if hi < 0. { Some(-1.) } else { None }
        }).collect();
        signs.iter().all(|s| s.is_some()) && signs.windows(2).all(|w| w[0] == w[1])
    }
    /// The parameter of its point nearest `p`: the nearest of samples across the span, then
    /// Newton on `(M − p) · M'` held to the span.
    pub fn inverse(&self,p: V) -> f64 {
        let [a,b] = self.span;
        let d2 = |v: f64| { let q = sub(self.d2(v).0,p); dot(q,q) };
        let mut v = (0..=64).map(|k| a+(b-a)*k as f64/64.).min_by(|x,y| d2(*x).total_cmp(&d2(*y))).unwrap_or(a);
        for _ in 0..30 {
            let (x,d,dd) = self.d2(v);
            let e = sub(x,p);
            let h = dot(d,d)+dot(e,dd);
            if !(h > 0.) { break }
            let next = (v-dot(e,d)/h).clamp(a.min(b),a.max(b));
            if !next.is_finite() { break }
            if (next-v).abs() <= 1e-15*(1.+v.abs()) { v = next; break }
            v = next;
        }
        v
    }
    /// A box about it over `[t0, t1]`, never smaller than it: `ρ` and `z` held in intervals over
    /// each of sixteen pieces, their rectangles in the half-plane placed; where a reading leaves
    /// its domain, its samples grown by its speed across a piece.
    pub fn bounds(&self,[t0,t1]: [f64;2]) -> Box3 {
        const PIECES: usize = 16;
        let f = &self.frame;
        let mut out = (([f64::INFINITY;3]),[f64::NEG_INFINITY;3]);
        let mut take = |p: V| for k in 0..3 { out.0[k] = out.0[k].min(p[k]); out.1[k] = out.1[k].max(p[k]) };
        for k in 0..PIECES {
            let (s,t) = (t0+(t1-t0)*k as f64/PIECES as f64,t0+(t1-t0)*(k+1) as f64/PIECES as f64);
            // posed, the contact's own box turned: its corners', each turned (a turn of a box
            // about its axis is held by the box of its corners turned and its ring's reach)
            let held = match self.posed {
                None => self.bounded([s,t]).map(|[[r,..],[z,..]]| vec![[r[0],0.,z[0]],[r[1],0.,z[1]],[r[0],0.,z[1]],[r[1],0.,z[0]]]),
                Some(theta) => {
                    let (x,_) = self.contact.point(Jet::at(Iv::new(s.min(t),s.max(t))));
                    match (x[0].v.bounds(),x[1].v.bounds(),x[2].v.bounds()) {
                        (Some(a),Some(b),Some(z)) => {
                            let (sn,cs) = (theta.sin(),theta.cos());
                            // the turned box's corners, the box of them a little grown for rounding,
                            // and that box's corners
                            let mut pts: Vec<[f64;3]> = Vec::new();
                            for &x in &a { for &y in &b { for &zz in &z { pts.push([cs*x-sn*y,sn*x+cs*y,zz]) } } }
                            let (lo,hi) = pts.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(l,h),p|
                                (std::array::from_fn(|i| l[i].min(p[i])),std::array::from_fn(|i| h[i].max(p[i]))));
                            let pad = 1e-12*(1.+hi.iter().chain(lo.iter()).fold(0f64,|m,x| m.max(x.abs())));
                            Some((0..8).map(|k| std::array::from_fn(|i| if (k>>i)&1 == 0 { lo[i]-pad } else { hi[i]+pad })).collect())
                        }
                        _ => None,
                    }
                }
            };
            match held {
                Some(corners) => for l in corners { take(f.at(l)) },
                None => {
                    let reach = (t-s).abs()*self.speed();
                    for v in [s,t] { let p = self.d2(v).0; for k in 0..3 { let mut q = p; q[k] -= reach; take(q); q[k] += 2.*reach; take(q) } }
                }
            }
        }
        out
    }
    /// The most length per unit of `v`, read at 65 points of the span.
    pub fn speed(&self) -> f64 {
        let [a,b] = self.span;
        (0..=64).map(|k| crate::util::space::norm(self.d2(a+(b-a)*k as f64/64.).1)).fold(0.,f64::max)
    }
    /// The same curve carried by the rigid `m`, or a meridian reflected by an improper one: its
    /// frame carried (the half-plane's points only read its `x` and `z`), its contact read in it as
    /// before. A posed contact is reflected nowhere yet: none where `m` is improper.
    pub fn moved(&self,m: &Rigid) -> Option<Turned> {
        let f = self.frame.moved(m);
        if f.right() { return Some(Turned {frame:f,..*self}) }
        self.posed.is_none().then(|| Turned {frame:f.rehanded(true).0,..*self})
    }
}
