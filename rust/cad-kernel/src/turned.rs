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
//!
//! **A sharp circle turned** (#780 L2, a fan's contact as a patch): a circle edge where two faces
//! meet at an angle carries the fan of normals between theirs, and under a turn every point of it
//! sweeps its circle about the axis, so its fan's sheet lies on the surface of revolution the circle
//! sweeps (`Rim`, the circle `o + r (cos w x + sin w y)` read at its angle `w`). Measured from a
//! point `p` of the axis, `ρ² + z² = |x − p|²` is affine in `(cos w, sin w)`, as `z` is: in
//! `(ρ² + z², z)` the meridian is an ellipse, so in `(ρ, z)` a simple closed curve wherever that
//! ellipse is one and the circle stands clear of the axis (`Rim::simple`) — the surface a ring,
//! held as that closed form in `w`, by the same jets.
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
    /// Its sine and cosine.
    fn sin_cos(self) -> (Self,Self);
}

impl Num for f64 {
    fn of(x: f64) -> f64 { x }
    fn sqrt(self) -> f64 { f64::sqrt(self) }
    fn sin_cos(self) -> (f64,f64) { use crate::util::fmath::Det; self.dsin_cos() }
}

/// An outward-rounded interval, or none where an operation left its domain (a division by an
/// interval holding nought, a root of one reaching below it): every reading through it then none.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Iv(pub Option<Interval>);

impl Iv {
    /// The interval between `lo` and `hi`, either way round.
    pub fn new(lo: f64,hi: f64) -> Iv { Iv(Interval::new(lo.min(hi),lo.max(hi)).ok()) }
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
    fn sin_cos(self) -> (Iv,Iv) {
        match self.0.and_then(|a| a.sin_cos_periodic().ok()) { Some((s,c)) => (Iv(Some(s)),Iv(Some(c))),None => (Iv(None),Iv(None)) }
    }
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
    /// Its sine and cosine: `sin' = cos x'`, `sin'' = cos x'' − sin x'²`; `cos' = −sin x'`,
    /// `cos'' = −sin x'' − cos x'²`.
    pub fn sin_cos(self) -> (Jet<T>,Jet<T>) {
        let (s,c) = self.v.sin_cos();
        let d2 = self.d*self.d;
        (Jet {v:s,d:c*self.d,dd:c*self.dd-s*d2},Jet {v:c,d:-(s*self.d),dd:-(s*self.dd)-c*d2})
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
        let [r2,z] = self.meridian2(v);
        [r2.sqrt(),z]
    }
    /// `meridian` with its radius squared, `ρ² = x² + y²`, read with no root (an interval's root
    /// and square again only widen it).
    pub fn meridian2<T: Num>(&self,v: Jet<T>) -> [Jet<T>;2] {
        let (x,_) = self.point(v);
        [x[0]*x[0]+x[1]*x[1],x[2]]
    }
}

/// **A sharp circle of the tool, under a turn** (the module's): `o + r (cos w x + sin w y)` at its
/// angle `w`, `x` and `y` unit and square, all in the turn's own frame (its origin on the turn's
/// axis, its `z` along it).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Rim { pub o: V,pub x: V,pub y: V,pub r: f64 }

impl Rim {
    /// Its point at the jet `w` of its angle.
    pub fn point<T: Num>(&self,w: Jet<T>) -> J3<T> {
        let (s,c) = w.sin_cos();
        std::array::from_fn(|i| Jet::of(self.o[i])+c.times(self.r*self.x[i])+s.times(self.r*self.y[i]))
    }
    /// Whether it turns into a ring, proved: clear of the axis (its least distance from it above
    /// `eps`, read over 64 pieces of the turn in intervals) and its meridian a simple closed curve —
    /// in `(ρ² + z², z)` an ellipse, `(2 r o·x, r x_z)` and `(2 r o·y, r y_z)` its axes' images,
    /// which part (their determinant `2 r² o · (x y_z − y x_z)` at least `eps` of `|o| r`: the
    /// circle's normal off the plane through the axis and its centre, where the circle's two halves
    /// would turn into one surface).
    pub fn simple(&self,eps: f64) -> bool {
        let (o,x,y,r) = (self.o,self.x,self.y,self.r);
        let det = 2.*r*r*(dot(o,x)*y[2]-dot(o,y)*x[2]);
        det.abs() > eps*r*(1.+dot(o,o).sqrt()) && pieces([0.,std::f64::consts::TAU],64).all(|w| {
            let p = self.point(Jet::at(w));
            (p[0].v*p[0].v+p[1].v*p[1].v).bounds().is_some_and(|[lo,_]| lo > eps*eps)
        })
    }
}

/// What a turned meridian is the meridian of: a round face's contact (#780 C3), or a sharp circle
/// of the tool (#780 L2).
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Source { Contact(Contact),Rim(Rim) }

impl Source {
    /// Its point at the jet of its parameter, in the turn's frame: the contact's at the face's slant,
    /// the circle's at its angle.
    pub fn point<T: Num>(&self,v: Jet<T>) -> J3<T> {
        match self { Source::Contact(c) => c.point(v).0,Source::Rim(c) => c.point(v) }
    }
    /// Where its point at `v` goes in `(ρ, z)` about the turn's axis, each with its jet.
    pub fn meridian<T: Num>(&self,v: Jet<T>) -> [Jet<T>;2] {
        let [r2,z] = self.meridian2(v);
        [r2.sqrt(),z]
    }
    /// `meridian` with its radius squared, read with no root.
    pub fn meridian2<T: Num>(&self,v: Jet<T>) -> [Jet<T>;2] {
        let x = self.point(v);
        [x[0]*x[0]+x[1]*x[1],x[2]]
    }
}

/// **The meridian of a contact or a sharp circle turned** (`geom::Curve::Turned`): the curve
/// `frame.at([ρ(v), 0, z(v)])`, `v` the face's slant (or the circle's angle) over `span`, in the
/// half-plane of `frame`'s `x` and `z` — `frame` the turn's axis (its origin on it, its `z` along
/// it), the source read in its coordinates. Or, `posed`, the contact itself, `frame.at(x(v))`:
/// `frame` then the turn's turned by the pose's angle, where the face at that pose of the turn
/// touches the meridian's surface of revolution (`ssi`'s `own_contact`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Turned { pub frame: Frame,pub source: Source,pub span: [f64;2],pub posed: bool }

/// `[a, b]` in `n` pieces, each an interval.
fn pieces([a,b]: [f64;2],n: usize) -> impl Iterator<Item=Iv> {
    (0..n).map(move |k| Iv::new(a+(b-a)*k as f64/n as f64,a+(b-a)*(k+1) as f64/n as f64))
}

impl Turned {
    /// The meridian of `contact` turned about `frame`'s axis, over `span`.
    pub fn meridian(frame: Frame,contact: Contact,span: [f64;2]) -> Turned { Turned {frame,source:Source::Contact(contact),span,posed:false} }
    /// The meridian of the sharp circle `rim` turned about `frame`'s axis, over the angles `span`.
    pub fn rim(frame: Frame,rim: Rim,span: [f64;2]) -> Turned { Turned {frame,source:Source::Rim(rim),span,posed:false} }
    /// The round face's contact it is the meridian of, if it is one's.
    pub fn contact(&self) -> Option<Contact> { match self.source { Source::Contact(c) => Some(c),Source::Rim(_) => None } }
    /// Where its source's point at `v` goes in `(ρ, z)`, each with its jet.
    pub fn meridian_at<T: Num>(&self,v: Jet<T>) -> [Jet<T>;2] { self.source.meridian(v) }
    /// `meridian_at` with its radius squared, read with no root.
    pub fn meridian2<T: Num>(&self,v: Jet<T>) -> [Jet<T>;2] { self.source.meridian2(v) }
    /// Its point at the jet `v`, in its frame's coordinates: the source's own, posed, else
    /// `(ρ, 0, z)`.
    fn local<T: Num>(&self,v: Jet<T>) -> J3<T> {
        if self.posed { return self.source.point(v) }
        let [r,z] = self.source.meridian(v);
        [r,Jet::of(0.),z]
    }
    /// `M`, `M'`, `M''` at `v`.
    pub fn d2(&self,v: f64) -> (V,V,V) {
        let (f,l) = (&self.frame,self.local(Jet::at(v)));
        (f.at(l.map(|j| j.v)),f.dir(l.map(|j| j.d)),f.dir(l.map(|j| j.dd)))
    }
    /// `ρ` and `z` with their first two derivatives over the stretch `[a, b]` of `v`, each held in
    /// an interval (none where a reading left its domain over it).
    pub fn bounded(&self,[a,b]: [f64;2]) -> Option<[[[f64;2];3];2]> {
        let [r,z] = self.source.meridian(Jet::at(Iv::new(a,b)));
        let three = |j: Jet<Iv>| Some([j.v.bounds()?,j.d.bounds()?,j.dd.bounds()?]);
        Some([three(r)?,three(z)?])
    }
    /// Whether the contact is a curve over `span`, proved, and its height runs one way (a contact's
    /// only): over each of
    /// 64 pieces its discriminant above nought (two contacts on every ring, apart), it off the
    /// turn's axis, and `z'` held off nought, one sign all along.
    pub fn defined(&self,span: [f64;2]) -> bool {
        let Source::Contact(contact) = self.source else { return false };
        let mut sign = 0.;
        pieces(span,64).all(|v| {
            let (x,disc) = contact.point(Jet::at(v));
            let rho2 = x[0].v*x[0].v+x[1].v*x[1].v;
            let climb = match x[2].d.bounds() { Some([lo,_]) if lo > 0. => 1.,Some([_,hi]) if hi < 0. => -1.,_ => 0. };
            let held = climb != 0. && (sign == 0. || sign == climb);
            sign = climb;
            held && disc.v.bounds().is_some_and(|[lo,_]| lo > 0.) && rho2.bounds().is_some_and(|[lo,_]| lo > 0.)
        })
    }
    /// Newton on `(M − p) · M'` from `v`, held to the span.
    pub fn newton(&self,p: V,mut v: f64) -> f64 {
        let [a,b] = self.span;
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
    /// The parameter of its point nearest `p`: the nearest of samples across the span (each read
    /// once, its point alone), then `newton`.
    pub fn inverse(&self,p: V) -> f64 {
        let [a,b] = self.span;
        let off = |v: f64| { let l = self.local(Jet::<f64>::of(v)).map(|j| j.v); let q = sub(self.frame.at(l),p); dot(q,q) };
        let v = (0..=64).map(|k| a+(b-a)*k as f64/64.).map(|v| (off(v),v)).min_by(|x,y| x.0.total_cmp(&y.0)).map_or(a,|x| x.1);
        self.newton(p,v)
    }
    /// A box about it over `[t0, t1]`, never smaller than it: its point held in intervals over each
    /// of sixteen pieces, each box's corners placed; where a reading leaves its domain, its samples
    /// grown by its speed across a piece.
    pub fn bounds(&self,[t0,t1]: [f64;2]) -> Box3 {
        const PIECES: usize = 16;
        let mut out = crate::geom::EMPTY;
        let mut speed = None;
        for (k,v) in pieces([t0,t1],PIECES).enumerate() {
            let l = self.local(Jet::at(v));
            match (l[0].v.bounds(),l[1].v.bounds(),l[2].v.bounds()) {
                (Some(x),Some(y),Some(z)) => for c in 0..8 {
                    crate::geom::include(&mut out,self.frame.at([x[c&1],y[(c>>1)&1],z[c>>2]]))
                },
                _ => {
                    let (s,t) = (t0+(t1-t0)*k as f64/PIECES as f64,t0+(t1-t0)*(k+1) as f64/PIECES as f64);
                    let reach = (t-s).abs()**speed.get_or_insert_with(|| self.speed());
                    for v in [s,t] { let p = self.d2(v).0; for k in 0..3 { let mut q = p; q[k] -= reach; crate::geom::include(&mut out,q); q[k] += 2.*reach; crate::geom::include(&mut out,q) } }
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
        (!self.posed).then(|| Turned {frame:f.rehanded(true).0,..*self})
    }
}
