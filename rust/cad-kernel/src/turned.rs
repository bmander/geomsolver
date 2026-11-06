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
//!
//! **Where the tool at an end pose meets those surfaces** (#780 L2, posed curves read round an angle
//! `τ`): a cylinder holding a sharp circle crosses the circle's ring off the circle along one closed
//! loop (`Crossing`), and a plane not level cuts a contact's surface of revolution (`Section`), each
//! the root of a function that runs one way over its stretch set equal to `−cos τ`, so the curve is
//! as smooth through its turning points as anywhere; each read in floats, its derivatives by the
//! implicit function, its box grown from its samples.
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
    /// The float it is, where it is one (an interval is none).
    fn real(self) -> Option<f64>;
    /// A number read nowhere: a float's NaN, an interval's none.
    fn unknown() -> Self;
}

impl Num for f64 {
    fn of(x: f64) -> f64 { x }
    fn sqrt(self) -> f64 { f64::sqrt(self) }
    fn sin_cos(self) -> (f64,f64) { use crate::util::fmath::Det; self.dsin_cos() }
    fn real(self) -> Option<f64> { Some(self) }
    fn unknown() -> f64 { f64::NAN }
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
    fn real(self) -> Option<f64> { None }
    fn unknown() -> Iv { Iv(None) }
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

/// **Where a cylinder holding a sharp circle at a pose of the turn crosses the circle's ring, off
/// the circle** (#780 L2: a flat-ended tool's side at an end pose against its rim's ring). In the
/// turn's frame, the cylinder's points `c + h a + r (cos φ e₁ + sin φ e₂)` — `c`, `e₁`, `e₂` the posed
/// circle's centre and frame, `a` its normal, the cylinder's axis — have `(|x|², z) = m + r L (cos φ,
/// sin φ) + (2h c·a + h², h a_z)`, `L = [[2 c·e₁, 2 c·e₂], [e₁_z, e₂_z]]`, and the ring is `m + r L
/// (cos w, sin w)`: so the point is on the ring where `|r ρ + u|² = r²`, `u = h v(h)`, `v = M₀ + h k`,
/// `M₀ = L⁻¹ (2 c·a, a_z)`, `k = L⁻¹ (1, 0)` — `h = 0`, the circle, or `ρ · v̂ = −σ(h)`, `σ = h |v| /
/// 2r`. Where `σ` climbs from −1 to 1 over `[h₀, h₁]` (`h σ` is `h² |v|²/2r`, a quartic; its rate's
/// sign is that of the quadratic `2|M₀|² + 6h M₀·k + 4h² |k|²`, held above nought over the stretch)
/// that is one closed loop, read at the angle `τ` round it: `σ(h) = −cos τ`, `ρ = cos τ v̂ + sin τ
/// v̂⊥`. It crosses the circle at `τ = π/2, 3π/2` (`h = 0`, `ρ = ±v̂⊥`), where the side's contact
/// reaches the rim. `h(τ)` is a root found by bisection and Newton, its derivatives by the implicit
/// function (`σ' h' = sin τ`); read in floats only (an interval reading is none, and a box about the
/// curve is grown from its samples).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Crossing { pub c: V,pub e1: V,pub e2: V,pub a: V,pub r: f64,pub m0: [f64;2],pub k: [f64;2],pub h: [f64;2] }

impl Crossing {
    /// The loop on the cylinder about the line through `c` along `a`, of radius `r`, holding the circle
    /// of that radius about `c` in the frame `e₁, e₂` (all in the turn's frame), where it crosses
    /// that circle's ring; none where `L` is all but singular, or `σ` does not climb once from −1
    /// to 1 about `h = 0` (its rate's quadratic reaching nought over the stretch).
    pub fn new(c: V,e1: V,e2: V,a: V,r: f64) -> Option<Crossing> {
        let l = [[2.*dot(c,e1),2.*dot(c,e2)],[e1[2],e2[2]]];
        let det = l[0][0]*l[1][1]-l[0][1]*l[1][0];
        if !(det.abs() > 1e-12*(1.+dot(c,c).sqrt()) && r > 0.) { return None }
        let inv = |p: [f64;2]| [(l[1][1]*p[0]-l[0][1]*p[1])/det,(-l[1][0]*p[0]+l[0][0]*p[1])/det];
        let (m0,k) = (inv([2.*dot(c,a),a[2]]),inv([1.,0.]));
        let q0 = m0[0]*m0[0]+m0[1]*m0[1];
        let (b,kk) = (m0[0]*k[0]+m0[1]*k[1],k[0]*k[0]+k[1]*k[1]);
        if !(q0 > 0.) { return None }
        let sigma = |h: f64| { let v = [m0[0]+h*k[0],m0[1]+h*k[1]]; h*v[0].hypot(v[1])/(2.*r) };
        // `|σ|` grows at least as `|h| (|v| ≥ |M₀| − |h| |k|)`: reach out until it passes 1 each way
        let reach = |sense: f64| -> Option<f64> {
            let mut h = r/q0.sqrt();
            for _ in 0..200 { if sense*sigma(sense*h) >= 1. { return Some(sense*h) } h *= 2. }
            None
        };
        let (lo,hi) = (reach(-1.)?,reach(1.)?);
        // σ's rate's sign over [lo, hi]: the quadratic 2 q0 + 6 h b + 4 h² kk, its least there
        let quad = |h: f64| 2.*q0+6.*h*b+4.*h*h*kk;
        let vertex = (-6.*b/(8.*kk)).clamp(lo,hi);
        if !(quad(lo).min(quad(hi)).min(if kk > 0. { quad(vertex) } else { f64::INFINITY }) > 1e-12*q0) { return None }
        // the ends, σ = ∓1, by bisection (σ climbs over [lo, hi])
        let root = |target: f64| { let (mut x,mut y) = (lo,hi); for _ in 0..200 { let m = 0.5*(x+y); if sigma(m) < target { x = m } else { y = m } } 0.5*(x+y) };
        Some(Crossing {c,e1,e2,a,r,m0,k,h:[root(-1.),root(1.)]})
    }
    /// `σ(h)` and its first two derivatives.
    fn sigma(&self,h: f64) -> [f64;3] {
        let j = Jet::<f64>::at(h);
        let v0 = Jet::of(self.m0[0])+j.times(self.k[0]);
        let v1 = Jet::of(self.m0[1])+j.times(self.k[1]);
        let s = (j*(v0*v0+v1*v1).sqrt()).times(0.5/self.r);
        [s.v,s.d,s.dd]
    }
    /// The height along the axis at the angle `τ`: `σ(h) = −cos τ` over the loop's stretch, by
    /// bisection to a bracket and Newton.
    fn height(&self,tau: f64) -> f64 {
        use crate::util::fmath::Det;
        let target = -tau.dcos();
        let [mut x,mut y] = self.h;
        if target <= -1. { return x }
        if target >= 1. { return y }
        let mut h = 0.5*(x+y);
        for _ in 0..200 {
            let [s,ds,_] = self.sigma(h);
            let f = s-target;
            if f == 0. { return h }
            if f < 0. { x = h } else { y = h }
            // Newton within the bracket, else its middle; done where the step is rounding's
            let newton = h-f/ds;
            let next = if ds > 0. && newton >= x && newton <= y { newton } else { 0.5*(x+y) };
            if (next-h).abs() <= 1e-15*(1.+h.abs()) { return next }
            h = next;
        }
        h
    }
    /// Its point at the jet `t` of the angle round it, in the turn's frame (floats only).
    pub fn point<T: Num>(&self,t: Jet<T>) -> J3<T> {
        let (Some(tv),Some(td),Some(tdd)) = (t.v.real(),t.d.real(),t.dd.real()) else { return [Jet {v:T::unknown(),d:T::unknown(),dd:T::unknown()};3] };
        use crate::util::fmath::Det;
        let h0 = self.height(tv);
        let [_,ds,dds] = self.sigma(h0);
        let (st,ct) = tv.dsin_cos();
        // h(τ): σ(h) = −cos τ, so σ' h' = sin τ and σ' h'' + σ'' h'² = cos τ
        let (h1,h2) = { let h1 = st/ds; (h1,(ct-dds*h1*h1)/ds) };
        // the jets in τ's own jet, through the chain rule
        let h = Jet {v:h0,d:h1*td,dd:h2*td*td+h1*tdd};
        let (sn,cs) = Jet {v:tv,d:td,dd:tdd}.sin_cos();
        let v0 = Jet::of(self.m0[0])+h.times(self.k[0]);
        let v1 = Jet::of(self.m0[1])+h.times(self.k[1]);
        let len = (v0*v0+v1*v1).sqrt();
        let (u0,u1) = (v0/len,v1/len);
        // ρ = cos τ v̂ + sin τ v̂⊥, v̂⊥ = (−v̂₁, v̂₀)
        let rho = [cs*u0-sn*u1,cs*u1+sn*u0];
        let x: [Jet<f64>;3] = std::array::from_fn(|i| Jet::of(self.c[i])+h.times(self.a[i])+(rho[0].times(self.e1[i])+rho[1].times(self.e2[i])).times(self.r));
        x.map(|j| Jet {v:T::of(j.v),d:T::of(j.d),dd:T::of(j.dd)})
    }
}

/// **Where a plane cuts the surface of revolution of a contact's meridian** (#780 L2: a flat end at
/// an end pose against its side's contact turned, where the side's contact reaching its rim makes the
/// plane's cut tangent to the rim's ring's). In the turn's frame, at the meridian's height `z(v)` the
/// plane `n · x = d` meets the circle of radius `ρ(v)` where `cos(ψ − ψₙ) = C(v) = (d − n_z z(v)) /
/// (|n_h| ρ(v))`, `ψₙ` the way of the normal's level part `n_h`. Where `C` runs one way over the
/// meridian's span (its rate held off nought by the interval jets), each `C` is one height, and the
/// cut is read at the angle `τ` round it: `C(v) = −cos τ`, `ψ = ψₙ + π − τ` — through the points
/// where the plane is tangent to a level circle (`C = ±1`, `sin τ = 0`) as smoothly as anywhere.
/// `v(τ)` a root found by bisection and Newton, its derivatives by the implicit function (`C' v' =
/// sin τ`); read in floats only.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Section { pub of: Contact,pub way: [f64;2],pub nh: f64,pub nz: f64,pub d: f64,pub v: [f64;2] }

impl Section {
    /// `C` at the jet `v`.
    fn c<T: Num>(&self,v: Jet<T>) -> Jet<T> {
        let [r,z] = self.of.meridian(v);
        (Jet::of(self.d)-z.times(self.nz))/r.times(self.nh)
    }
    /// The cut of the plane `n · x = d` (unit `n`, in the turn's frame) through the surface of
    /// revolution of `of`'s meridian over `span`, as its curves' spans of `τ`: none where the plane is
    /// all but level (a level plane cuts circles, read as meridians are), or `C` is not proved to run
    /// one way over the span; no curve where it misses.
    pub fn new(of: Contact,n: V,d: f64,span: [f64;2]) -> Option<(Section,Vec<[f64;2]>)> {
        use std::f64::consts::{PI,TAU};
        let nh = n[0].hypot(n[1]);
        if !(nh > 1e-9) { return None }
        let k = Section {of,way:[n[0]/nh,n[1]/nh],nh,nz:n[2],d,v:[span[0].min(span[1]),span[0].max(span[1])]};
        let mut sign = 0.;
        let n = 64;
        for i in 0..n {
            let [a,b] = k.v;
            let piece = Iv::new(a+(b-a)*i as f64/n as f64,a+(b-a)*(i+1) as f64/n as f64);
            let rate = match k.c(Jet::at(piece)).d.bounds() { Some([lo,_]) if lo > 0. => 1.,Some([_,hi]) if hi < 0. => -1.,_ => return None };
            if sign != 0. && rate != sign { return None }
            sign = rate;
        }
        let (c0,c1) = (k.c(Jet::<f64>::of(k.v[0])).v,k.c(Jet::<f64>::of(k.v[1])).v);
        let (lo,hi) = (c0.min(c1),c0.max(c1));
        if lo >= 1. || hi <= -1. { return Some((k,vec![])) }
        let (ta,tb) = (if lo <= -1. { 0. } else { (-lo).acos() },if hi >= 1. { PI } else { (-hi).acos() });
        let spans = match (ta == 0.,tb == PI) {
            (true,true) => vec![[0.,TAU]],
            (true,false) => vec![[-tb,tb]],
            (false,true) => vec![[ta,TAU-ta]],
            (false,false) => vec![[ta,tb],[TAU-tb,TAU-ta]],
        };
        Some((k,spans.into_iter().filter(|s| s[1] > s[0]).collect()))
    }
    /// The slant at the angle `τ`: `C(v) = −cos τ` over the span (`C` runs one way), by bisection to
    /// a bracket and Newton.
    fn slant(&self,tau: f64) -> f64 {
        use crate::util::fmath::Det;
        let [a,b] = self.v;
        let up = self.c(Jet::<f64>::of(b)).v > self.c(Jet::<f64>::of(a)).v;
        let target = -tau.dcos();
        let (mut x,mut y) = (a,b);
        let mut v = 0.5*(a+b);
        for _ in 0..200 {
            let j = self.c(Jet::<f64>::at(v));
            let f = j.v-target;
            if f == 0. { return v }
            if (f < 0.) == up { x = v } else { y = v }
            let newton = v-f/j.d;
            let next = if newton >= x && newton <= y { newton } else { 0.5*(x+y) };
            if (next-v).abs() <= 1e-15*(1.+v.abs()) { return next }
            v = next;
        }
        v
    }
    /// Its point at the jet `t` of the angle round it, in the turn's frame (floats only).
    pub fn point<T: Num>(&self,t: Jet<T>) -> J3<T> {
        let (Some(tv),Some(td),Some(tdd)) = (t.v.real(),t.d.real(),t.dd.real()) else { return [Jet {v:T::unknown(),d:T::unknown(),dd:T::unknown()};3] };
        use crate::util::fmath::Det;
        let v0 = self.slant(tv);
        let j = self.c(Jet::<f64>::at(v0));
        let (st,ct) = tv.dsin_cos();
        // C(v(τ)) = −cos τ: C' v' = sin τ, C' v'' + C'' v'² = cos τ
        let (v1,v2) = { let v1 = st/j.d; (v1,(ct-j.dd*v1*v1)/j.d) };
        let v = Jet {v:v0,d:v1*td,dd:v2*td*td+v1*tdd};
        let [r,z] = self.of.meridian(v);
        let psi = Jet {v:self.way[1].datan2(self.way[0])+std::f64::consts::PI-tv,d:-td,dd:-tdd};
        let (sn,cs) = psi.sin_cos();
        [r*cs,r*sn,z].map(|j| Jet {v:T::of(j.v),d:T::of(j.d),dd:T::of(j.dd)})
    }
}

/// What a turned meridian is the meridian of: a round face's contact (#780 C3), or a sharp circle
/// of the tool (#780 L2); or, posed only, the loop along which a cylinder holding that circle at a
/// pose crosses its ring (`Crossing`), or where a plane cuts a contact's surface (`Section`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Source { Contact(Contact),Rim(Rim),Crossing(Crossing),Section(Section) }

impl Source {
    /// Its point at the jet of its parameter, in the turn's frame: the contact's at the face's slant,
    /// the circle's at its angle.
    pub fn point<T: Num>(&self,v: Jet<T>) -> J3<T> {
        match self { Source::Contact(c) => c.point(v).0,Source::Rim(c) => c.point(v),Source::Crossing(c) => c.point(v),Source::Section(c) => c.point(v) }
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
    /// Its period, where it is a closed loop read round an angle over a whole turn (#780 L2: a
    /// side's crossing with its rim's ring, a plane's cut of a contact's surface all round): `2π`,
    /// every reading of it periodic in its angle. A meridian has none (a sharp circle's, closed
    /// too, is read as a profile's edge over its span).
    pub fn period(&self) -> Option<f64> {
        let full = (self.span[1]-self.span[0]-std::f64::consts::TAU).abs() <= 1e-12;
        (self.posed && full && matches!(self.source,Source::Crossing(_) | Source::Section(_))).then_some(std::f64::consts::TAU)
    }
    /// The period of its meridian's parameter on its surface of revolution: a sharp circle's, read
    /// over a whole turn of its angle, `2π`; none for a contact's.
    pub fn meridian_period(&self) -> Option<f64> {
        let full = (self.span[1]-self.span[0]-std::f64::consts::TAU).abs() <= 1e-12;
        (!self.posed && full && matches!(self.source,Source::Rim(_))).then_some(std::f64::consts::TAU)
    }
    /// The round face's contact it is the meridian of, if it is one's.
    pub fn contact(&self) -> Option<Contact> { match self.source { Source::Contact(c) => Some(c),_ => None } }
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
    /// Newton on `(M − p) · M'` from `v`, held to the span — or, a closed loop or a closed meridian
    /// (#780 L2), read round its period, across the place its span starts.
    pub fn newton(&self,p: V,mut v: f64) -> f64 {
        let [a,b] = self.span;
        let period = self.period().or(self.meridian_period());
        for _ in 0..30 {
            let (x,d,dd) = self.d2(v);
            let e = sub(x,p);
            let h = dot(d,d)+dot(e,dd);
            if !(h > 0.) { break }
            let step = v-dot(e,d)/h;
            let next = match period { Some(per) => crate::geom::around(step,a.min(b),per),None => step.clamp(a.min(b),a.max(b)) };
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
