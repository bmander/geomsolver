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
use crate::util::space::{dot,sub};

pub use crate::util::interval::{Iv,Jet,Num};

/// A point's three coordinates, each with its jet.
pub type J3<T> = [Jet<T>;3];

/// `a · b`, its jet.
pub(crate) fn dot3<T: Num>(a: J3<T>,b: J3<T>) -> Jet<T> { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
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

/// **Where a circle's points go in `(ρ² + z², z)`**, measured from the turn's origin: about its
/// centre `c` in its frame `e₁, e₂`, its point `c + q` goes to `m + L q`, `L = [[2 c·e₁, 2 c·e₂],
/// [e₁_z, e₂_z]]` — the inverse of `L`, none where its determinant is not above `floor`. The ring
/// a sharp circle turns into, the loop where a cylinder holding it crosses that ring and a plane
/// holding it cuts it are each read through it.
pub fn rim_inverse(c: V,e1: V,e2: V,floor: f64) -> Option<[[f64;2];2]> {
    let l = [[2.*dot(c,e1),2.*dot(c,e2)],[e1[2],e2[2]]];
    let det = l[0][0]*l[1][1]-l[0][1]*l[1][0];
    (det.abs() > floor).then(|| [[l[1][1]/det,-l[0][1]/det],[-l[1][0]/det,l[0][0]/det]])
}

/// `m p`, `m` two by two.
fn times(m: [[f64;2];2],p: [f64;2]) -> [f64;2] { [m[0][0]*p[0]+m[0][1]*p[1],m[1][0]*p[0]+m[1][1]*p[1]] }

impl Rim {
    /// Its point at the jet `w` of its angle.
    pub fn point<T: Num>(&self,w: Jet<T>) -> J3<T> {
        let (s,c) = w.sin_cos();
        std::array::from_fn(|i| Jet::of(self.o[i])+c.times(self.r*self.x[i])+s.times(self.r*self.y[i]))
    }
    /// Whether it turns into a ring, proved: clear of the axis (its least distance from it above
    /// `eps`, read over 64 pieces of the turn in intervals) and its meridian a simple closed curve —
    /// in `(ρ² + z², z)` the ellipse `m + L (circle)`, `L` invertible (`rim_inverse`, its determinant
    /// `2 o · (x y_z − y x_z)` at least `eps` of `|o| / r`: the circle's normal off the plane through the
    /// axis and its centre, where the circle's two halves would turn into one surface).
    pub fn simple(&self,eps: f64) -> bool {
        rim_inverse(self.o,self.x,self.y,eps*(1.+dot(self.o,self.o).sqrt())/self.r).is_some()
            && pieces([0.,std::f64::consts::TAU],64).all(|[a,b]| {
                let p = self.point(Jet::at(Iv::new(a,b)));
                (p[0].v*p[0].v+p[1].v*p[1].v).bounds().is_some_and(|[lo,_]| lo > eps*eps)
            })
    }
    /// The turns about the axis carrying its normal onto `along` (a unit way in the turn's frame),
    /// either way round: the circle so carried, its centre and frame. None where its normal runs
    /// along the axis, or no turn does.
    pub fn posed(&self,along: V) -> Vec<(V,V,V)> {
        use crate::util::fmath::Det;
        let n = crate::util::space::unit(crate::util::space::cross(self.x,self.y));
        let (h1,h2) = (along[0].dhypot(along[1]),n[0].dhypot(n[1]));
        if h2 <= 1e-9 { return vec![] }
        [1.,-1.].iter().filter_map(|&sign| {
            let m = along.map(|x| x*sign);
            if (m[2]-n[2]).abs() > 1e-9 || (h1-h2).abs() > 1e-9 { return None }
            let turn = Rigid::turn([0.;3],[0.,0.,1.],m[1].datan2(m[0])-n[1].datan2(n[0]));
            Some((turn.point(self.o),turn.vector(self.x),turn.vector(self.y)))
        }).collect()
    }
}

/// **The root of a function running one way over `[a, b]` at `target`**: `f` its value and rate at
/// a float, `ends` its values at `a` and `b`; the end it reaches past, else Newton held to a
/// bracket, its middle where Newton leaves it, done where a step is rounding's.
fn monotone_root(f: &dyn Fn(f64) -> [f64;2],[a,b]: [f64;2],ends: [f64;2],target: f64) -> f64 {
    let up = ends[1] > ends[0];
    if (target <= ends[0]) == up { return a }
    if (target >= ends[1]) == up { return b }
    let (mut x,mut y) = (a,b);
    let mut v = 0.5*(a+b);
    for _ in 0..200 {
        let [fv,dv] = f(v);
        let g = fv-target;
        if g == 0. { return v }
        if (g < 0.) == up { x = v } else { y = v }
        let newton = v-g/dv;
        let next = if newton >= x && newton <= y { newton } else { 0.5*(x+y) };
        if (next-v).abs() <= 1e-15*(1.+v.abs()) { return next }
        v = next;
    }
    v
}

/// **A posed curve's point at the jet `t` of its angle**, its parameter `x(τ)` the root of
/// `f(x) = −cos τ` (`solve`, `f` with its first two rates at a float): `x`'s jet in `τ` by the
/// implicit function (`f' x' = sin τ`, `f' x'' + f'' x'² = cos τ`) and the chain rule, handed to `at`
/// with `τ`'s own. Read in intervals (#814: what a certified reading encloses the curve by), over a
/// stretch `T` of `τ`: `f` runs one way over `x`'s bracket, so the `x` it takes over `−cos T` lie
/// between the roots at that interval's ends — each found in floats, then widened until `f` read in
/// intervals just outside it is proved past those ends (`bracketed`) — and `x' = sin τ / f'(x)` is
/// enclosed over them; the point by `at` read in intervals (`at_iv`, `f_iv`: the same formulas over
/// `Num`). Where that cannot be proved, every number.
fn implicit<T: Num>(t: Jet<T>,solve: &dyn Fn(f64) -> f64,f: &dyn Fn(f64) -> [f64;3],at: &dyn Fn(Jet<f64>,Jet<f64>) -> J3<f64>,
    f_iv: &dyn Fn(Jet<Iv>) -> Jet<Iv>,at_iv: &dyn Fn(Jet<Iv>,Jet<Iv>) -> J3<Iv>) -> J3<T> {
    let (Some(tv),Some(td),Some(tdd)) = (t.v.real(),t.d.real(),t.dd.real()) else {
        let any = Jet {v:T::from_iv(Iv::ANY),d:T::from_iv(Iv::ANY),dd:T::from_iv(Iv::ANY)};
        let tj = Jet {v:t.v.iv(),d:t.d.iv(),dd:t.dd.iv()};
        let (st,ct) = tj.v.sin_cos();
        let Some(x) = bracketed(-ct,solve,f_iv) else { return [any;3] };
        let rate = f_iv(Jet::at(x)).d;
        if rate.has_zero() { return [any;3] }
        let xj = Jet {v:x,d:st/rate*tj.d,dd:Iv::ANY};
        return at_iv(xj,tj).map(|j| Jet {v:T::from_iv(j.v),d:T::from_iv(j.d),dd:T::from_iv(j.dd)})
    };
    use crate::util::fmath::Det;
    let x0 = solve(tv);
    let [_,d1,d2] = f(x0);
    let (st,ct) = tv.dsin_cos();
    let x1 = st/d1;
    let x2 = (ct-d2*x1*x1)/d1;
    at(Jet {v:x0,d:x1*td,dd:x2*td*td+x1*tdd},Jet {v:tv,d:td,dd:tdd}).map(|j| Jet {v:T::of(j.v),d:T::of(j.d),dd:T::of(j.dd)})
}

/// **Every root of `f(x) = c` for `c` in `target`, `f` running one way** (`implicit`'s): the roots at
/// `target`'s ends (`solve`: `solve(τ)` is the root at `−cos τ`, so it is asked at the angles those
/// ends stand for), widened by a few units of rounding, then more, until `f` read in intervals at
/// each end of the bracket is proved past the target's end there (or the bracket is the root's own
/// stretch, which `solve` holds it to). `None` where it cannot be proved.
fn bracketed(target: Iv,solve: &dyn Fn(f64) -> f64,f_iv: &dyn Fn(Jet<Iv>) -> Jet<Iv>) -> Option<Iv> {
    let (c0,c1) = (target.lo().max(-1.),target.hi().min(1.));
    if !(c0 <= c1) { return None }
    // `solve` is asked at an angle: `−cos τ = c` at `τ = acos(−c)`
    use crate::util::fmath::Det;
    let (r0,r1) = (solve((-c0).dacos()),solve((-c1).dacos()));
    let (lo,hi) = (r0.min(r1),r0.max(r1));
    let value = |x: f64| f_iv(Jet::at(Iv::point(x))).v;
    let up = { let (a,b) = (value(lo).mid(),value(hi).mid()); if a == b { value(hi+1e-9*(1.+hi.abs())).mid() >= a } else { b > a } };
    let mut grow = 4.*f64::EPSILON;
    for _ in 0..24 {
        let (x0,x1) = (lo-grow*(1.+lo.abs()),hi+grow*(1.+hi.abs()));
        // past each end: below the target's least there and above its most, the way `f` runs
        let (v0,v1) = (value(x0),value(x1));
        let held = if up { v0.hi() < c0 && v1.lo() > c1 } else { v0.lo() > c1 && v1.hi() < c0 };
        if held { return Some(Iv::new(x0,x1)) }
        grow *= 4.;
    }
    // (an end the bracket's own: the root clamped there; held where `f` there is past it no further)
    None
}

/// **Where a cylinder holding a sharp circle at a pose of the turn crosses the circle's ring, off
/// the circle** (#780 L2: a flat-ended tool's side at an end pose against its rim's ring). In the
/// turn's frame, the cylinder's points `c + h a + r (cos φ e₁ + sin φ e₂)` — `c`, `e₁`, `e₂` the posed
/// circle's centre and frame, `a` its normal, the cylinder's axis — have `(|x|², z) = m + r L (cos φ,
/// sin φ) + (2h c·a + h², h a_z)` (`rim_inverse`), and the ring is `m + r L (cos w, sin w)`: so the
/// point is on the ring where `|r ρ + u|² = r²`, `u = h v(h)`, `v = M₀ + h k`, `M₀ = L⁻¹ (2 c·a,
/// a_z)`, `k = L⁻¹ (1, 0)` — `h = 0`, the circle, or `ρ · v̂ = −σ(h)`, `σ = h |v| / 2r`. Where `σ`
/// climbs from −1 to 1 over `[h₀, h₁]` (`h σ` is `h² |v|²/2r`, a quartic; its rate's sign is that of
/// the quadratic `2|M₀|² + 6h M₀·k + 4h² |k|²`, held above nought over the stretch) that is one
/// closed loop, read at the angle `τ` round it: `σ(h) = −cos τ`, `ρ = cos τ v̂ + sin τ v̂⊥`
/// (`implicit`). It crosses the circle at `τ = π/2, 3π/2` (`h = 0`, `ρ = ±v̂⊥`), where the side's
/// contact reaches the rim.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Crossing { pub c: V,pub e1: V,pub e2: V,pub a: V,pub r: f64,m0: [f64;2],k: [f64;2],h: [f64;2] }

impl Crossing {
    /// The loop on the cylinder about the line through `c` along `a`, of radius `r`, holding the circle
    /// of that radius about `c` in the frame `e₁, e₂` (all in the turn's frame), where it crosses
    /// that circle's ring; none where `L` is all but singular, or `σ` does not climb once from −1
    /// to 1 about `h = 0` (its rate's quadratic reaching nought over the stretch).
    pub fn new(c: V,e1: V,e2: V,a: V,r: f64) -> Option<Crossing> {
        if !(r > 0.) { return None }
        let inv = rim_inverse(c,e1,e2,1e-12*(1.+dot(c,c).sqrt()))?;
        let (m0,k) = (times(inv,[2.*dot(c,a),a[2]]),times(inv,[1.,0.]));
        let q0 = m0[0]*m0[0]+m0[1]*m0[1];
        if !(q0 > 0.) { return None }
        let mut out = Crossing {c,e1,e2,a,r,m0,k,h:[0.;2]};
        let sigma = |h: f64| out.sigma(h)[0];
        // `|σ|` grows at least as `|h| (|v| ≥ |M₀| − |h| |k|)`: reach out until it passes 1 each way
        let reach = |sense: f64| -> Option<f64> {
            let mut h = r/q0.sqrt();
            for _ in 0..200 { if sense*sigma(sense*h) >= 1. { return Some(sense*h) } h *= 2. }
            None
        };
        let (lo,hi) = (reach(-1.)?,reach(1.)?);
        // σ's rate's sign over [lo, hi]: the quadratic 2 q0 + 6 h b + 4 h² kk, its least there
        let (b,kk) = (m0[0]*k[0]+m0[1]*k[1],k[0]*k[0]+k[1]*k[1]);
        let quad = |h: f64| 2.*q0+6.*h*b+4.*h*h*kk;
        let vertex = (-6.*b/(8.*kk)).clamp(lo,hi);
        if !(quad(lo).min(quad(hi)).min(if kk > 0. { quad(vertex) } else { f64::INFINITY }) > 1e-12*q0) { return None }
        // the ends, σ = ∓1 (σ climbs over [lo, hi])
        let (rate,ends) = (|h: f64| { let [s,d,_] = out.sigma(h); [s,d] },[sigma(lo),sigma(hi)]);
        out.h = [monotone_root(&rate,[lo,hi],ends,-1.),monotone_root(&rate,[lo,hi],ends,1.)];
        Some(out)
    }
    /// **The loop where a cylinder crosses the ring of the sharp circle `rim` it holds at some pose**
    /// (`Crossing::new`): the cylinder about the line through `through` along the unit `along`, of
    /// radius `radius` (all in the turn's frame), holding a turned copy of the circle — its normal the
    /// circle's turned (`Rim::posed`), its centre on the line within `tol`, its radius the circle's.
    pub fn held(rim: &Rim,along: V,through: V,radius: f64,tol: f64) -> Option<Crossing> {
        if (radius.abs()-rim.r).abs() > tol { return None }
        let (o,e1,e2) = rim.posed(along).into_iter()
            .find(|&(o,..)| crate::util::space::norm(crate::util::space::cross(sub(o,through),along)) <= tol)?;
        Crossing::new(o,e1,e2,crate::util::space::cross(e1,e2),rim.r)
    }
    /// The heights along the axis its loop spans.
    pub fn heights(&self) -> [f64;2] { self.h }
    /// `σ(h)` and its first two derivatives.
    fn sigma(&self,h: f64) -> [f64;3] {
        let j = Jet::<f64>::at(h);
        let v0 = Jet::of(self.m0[0])+j.times(self.k[0]);
        let v1 = Jet::of(self.m0[1])+j.times(self.k[1]);
        let s = (j*(v0*v0+v1*v1).sqrt()).times(0.5/self.r);
        [s.v,s.d,s.dd]
    }
    /// `σ` at the jet `h`, written once over `Num`.
    fn sigma_jet<T: Num>(&self,j: Jet<T>) -> Jet<T> {
        let v0 = Jet::of(self.m0[0])+j.times(self.k[0]);
        let v1 = Jet::of(self.m0[1])+j.times(self.k[1]);
        (j*(v0*v0+v1*v1).sqrt()).times(0.5/self.r)
    }
    /// The loop's point at the jets of its height `h` and its angle `τ`, written once over `Num`.
    fn at<T: Num>(&self,h: Jet<T>,tau: Jet<T>) -> J3<T> {
        let (sn,cs) = tau.sin_cos();
        let v0 = Jet::of(self.m0[0])+h.times(self.k[0]);
        let v1 = Jet::of(self.m0[1])+h.times(self.k[1]);
        let len = (v0*v0+v1*v1).sqrt();
        let (u0,u1) = (v0/len,v1/len);
        // ρ = cos τ v̂ + sin τ v̂⊥, v̂⊥ = (−v̂₁, v̂₀)
        let rho = [cs*u0-sn*u1,cs*u1+sn*u0];
        std::array::from_fn(|i| Jet::of(self.c[i])+h.times(self.a[i])+(rho[0].times(self.e1[i])+rho[1].times(self.e2[i])).times(self.r))
    }
    /// Its point at the jet `t` of the angle round it, in the turn's frame, in floats or intervals.
    pub fn point<T: Num>(&self,t: Jet<T>) -> J3<T> {
        use crate::util::fmath::Det;
        let rate = |h: f64| { let [s,d,_] = self.sigma(h); [s,d] };
        implicit(t,&|tau| monotone_root(&rate,self.h,[-1.,1.],-tau.dcos()),&|h| self.sigma(h),&|h,tau| self.at(h,tau),
            &|h| self.sigma_jet(h),&|h,tau| self.at(h,tau))
    }
}

/// **Where a plane cuts the surface of revolution of a contact's meridian** (#780 L2: a flat end at
/// an end pose against its side's contact turned, where the side's contact reaching its rim makes the
/// plane's cut tangent to the rim's ring's). In the turn's frame, at the meridian's height `z(v)` the
/// plane `n · x = d` meets the circle of radius `ρ(v)` where `cos(ψ − ψₙ) = C(v) = (d − n_z z(v)) /
/// (|n_h| ρ(v))`, `ψₙ` the way of the normal's level part `n_h`. Where `C` runs one way over the
/// meridian's span (its rate held off nought by the interval jets), each `C` is one height, and the
/// cut is read at the angle `τ` round it: `C(v) = −cos τ`, `ψ = ψₙ + π − τ` (`implicit`) — through the
/// points where the plane is tangent to a level circle (`C = ±1`, `sin τ = 0`) as smoothly as
/// anywhere.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Section { pub of: Contact,pub n: V,pub d: f64,pub v: [f64;2],ends: [f64;2] }

impl Section {
    /// `C` at the jet `v`.
    fn c<T: Num>(&self,v: Jet<T>) -> Jet<T> {
        let [r,z] = self.of.meridian(v);
        (Jet::of(self.d)-z.times(self.n[2]))/r.times(self.n[0].hypot(self.n[1]))
    }
    /// The cut of the plane `n · x = d` (unit `n`, in the turn's frame) through the surface of
    /// revolution of `of`'s meridian over `span`, as its curves' spans of `τ`: none where the plane is
    /// all but level (a level plane cuts circles, read as meridians are), or `C` is not proved to run
    /// one way over the span; no curve where it misses.
    pub fn new(of: Contact,n: V,d: f64,span: [f64;2]) -> Option<(Section,Vec<[f64;2]>)> {
        use std::f64::consts::{PI,TAU};
        if !(n[0].hypot(n[1]) > 1e-9) { return None }
        let mut k = Section {of,n,d,v:[span[0].min(span[1]),span[0].max(span[1])],ends:[0.;2]};
        let mut sign = 0.;
        let one_way = pieces(k.v,64).all(|[a,b]| {
            let rate = match k.c(Jet::at(Iv::new(a,b))).d.bounds() { Some([lo,_]) if lo > 0. => 1.,Some([_,hi]) if hi < 0. => -1.,_ => 0. };
            let held = rate != 0. && (sign == 0. || rate == sign);
            sign = rate;
            held
        });
        if !one_way { return None }
        k.ends = k.v.map(|v| k.c(Jet::<f64>::of(v)).v);
        let (lo,hi) = (k.ends[0].min(k.ends[1]),k.ends[0].max(k.ends[1]));
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
    /// The cut's point at the jets of the slant `v` and the angle `τ`, written once over `Num`.
    fn at<T: Num>(&self,v: Jet<T>,tau: Jet<T>) -> J3<T> {
        use crate::util::fmath::Det;
        let way = self.n[1].datan2(self.n[0]);
        let [r,z] = self.of.meridian(v);
        let (sn,cs) = (Jet::of(way+std::f64::consts::PI)-tau).sin_cos();
        [r*cs,r*sn,z]
    }
    /// Its point at the jet `t` of the angle round it, in the turn's frame, in floats or intervals.
    pub fn point<T: Num>(&self,t: Jet<T>) -> J3<T> {
        use crate::util::fmath::Det;
        let c = |v: f64| { let j = self.c(Jet::<f64>::at(v)); [j.v,j.d,j.dd] };
        let rate = |v: f64| { let [x,d,_] = c(v); [x,d] };
        implicit(t,&|tau| monotone_root(&rate,self.v,self.ends,-tau.dcos()),&c,&|v,tau| self.at(v,tau),
            &|v| self.c(v),&|v,tau| self.at(v,tau))
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
    /// The period of its parameter: an angle's, `2π`, for all but a contact's slant.
    pub fn period(&self) -> Option<f64> { (!matches!(self,Source::Contact(_))).then_some(std::f64::consts::TAU) }
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

impl Turned {
    /// The meridian of `contact` turned about `frame`'s axis, over `span`.
    pub fn meridian(frame: Frame,contact: Contact,span: [f64;2]) -> Turned { Turned {frame,source:Source::Contact(contact),span,posed:false} }
    /// The meridian of the sharp circle `rim` turned about `frame`'s axis, over the angles `span`.
    pub fn rim(frame: Frame,rim: Rim,span: [f64;2]) -> Turned { Turned {frame,source:Source::Rim(rim),span,posed:false} }
    /// The plane it lies in, where it is a plane's cut (`Section`): its unit normal and offset, in
    /// the world.
    pub fn plane(&self) -> Option<(V,f64)> {
        let Source::Section(k) = self.source else { return None };
        let n = self.frame.dir(k.n);
        Some((n,k.d+dot(n,self.frame.o)))
    }
    /// The round face's contact it is the meridian of, if it is one's.
    pub fn contact(&self) -> Option<Contact> { match self.source { Source::Contact(c) => Some(c),_ => None } }
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
        pieces(span,64).map(|[a,b]| Iv::new(a,b)).all(|v| {
            let (x,disc) = contact.point(Jet::at(v));
            let rho2 = x[0].v*x[0].v+x[1].v*x[1].v;
            let climb = match x[2].d.bounds() { Some([lo,_]) if lo > 0. => 1.,Some([_,hi]) if hi < 0. => -1.,_ => 0. };
            let held = climb != 0. && (sign == 0. || sign == climb);
            sign = climb;
            held && disc.v.bounds().is_some_and(|[lo,_]| lo > 0.) && rho2.bounds().is_some_and(|[lo,_]| lo > 0.)
        })
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

impl Exact for Turned {
    fn frame(&self) -> &Frame { &self.frame }
    fn span(&self) -> [f64;2] { self.span }
    /// Its point at the jet `v`, in its frame's coordinates: the source's own, posed, else
    /// `(ρ, 0, z)`.
    fn local<T: Num>(&self,v: Jet<T>) -> J3<T> {
        if self.posed { return self.source.point(v) }
        let [r,z] = self.source.meridian(v);
        [r,Jet::of(0.),z]
    }
    /// Its period, where it is closed (#780 L2): read over a whole period of its source's parameter
    /// — a sharp circle's meridian round the circle (a ring, periodic round its meridian as a torus
    /// is), a side's crossing with its rim's ring, a plane's cut of a contact's surface all round.
    fn period(&self) -> Option<f64> {
        self.source.period().filter(|&p| (self.span[1]-self.span[0]-p).abs() <= 1e-12)
    }
}

/// `[a, b]` in `n` pieces.
pub(crate) fn pieces([a,b]: [f64;2],n: usize) -> impl Iterator<Item=[f64;2]> {
    (0..n).map(move |k| [a+(b-a)*k as f64/n as f64,a+(b-a)*(k+1) as f64/n as f64])
}

/// **A curve read exactly in a frame** (#780 C3's turned contact, #805's slid one): its point a jet
/// of its parameter in its frame's coordinates (`local`), read in floats or in outward-rounded
/// intervals, over its span. What every such curve reads off that, written once.
pub trait Exact {
    /// The frame its points are read in.
    fn frame(&self) -> &Frame;
    /// The stretch of its parameter it runs over.
    fn span(&self) -> [f64;2];
    /// Its point at the jet `t`, in its frame's coordinates.
    fn local<T: Num>(&self,t: Jet<T>) -> J3<T>;
    /// The period of its parameter, where it is closed (#780 L2): none by default.
    fn period(&self) -> Option<f64> { None }
    /// `C`, `C'`, `C''` at `t`.
    fn d2(&self,t: f64) -> (V,V,V) {
        let (f,l) = (self.frame(),self.local(Jet::at(t)));
        (f.at(l.map(|j| j.v)),f.dir(l.map(|j| j.d)),f.dir(l.map(|j| j.dd)))
    }
    /// Newton on `(C − p) · C'` from `t`, held to the span — or, closed (#780 L2), read round its
    /// period, across the place its span starts.
    fn newton(&self,p: V,mut t: f64) -> f64 {
        let [a,b] = self.span();
        let period = self.period();
        for _ in 0..30 {
            let (x,d,dd) = self.d2(t);
            let e = sub(x,p);
            let h = dot(d,d)+dot(e,dd);
            if !(h > 0.) { break }
            let step = t-dot(e,d)/h;
            let next = match period { Some(per) => crate::geom::around(step,a.min(b),per),None => step.clamp(a.min(b),a.max(b)) };
            if !next.is_finite() { break }
            if (next-t).abs() <= 1e-15*(1.+t.abs()) { t = next; break }
            t = next;
        }
        t
    }
    /// The parameter of its point nearest `p`: the nearest of samples across the span (each read
    /// once, its point alone), then `newton`.
    fn inverse(&self,p: V) -> f64 {
        let [a,b] = self.span();
        let off = |t: f64| { let l = self.local(Jet::<f64>::of(t)).map(|j| j.v); let q = sub(self.frame().at(l),p); dot(q,q) };
        let t = (0..=64).map(|k| a+(b-a)*k as f64/64.).map(|t| (off(t),t)).min_by(|x,y| x.0.total_cmp(&y.0)).map_or(a,|x| x.1);
        self.newton(p,t)
    }
    /// A box about it over `[t0, t1]`, never smaller than it: its point held in intervals over each
    /// of sixteen pieces, each box's corners placed; where a reading leaves its domain, its samples
    /// grown by its speed across a piece.
    fn bounds(&self,[t0,t1]: [f64;2]) -> Box3 {
        let mut out = crate::geom::EMPTY;
        let mut speed = None;
        for [s,t] in pieces([t0,t1],16) {
            let l = self.local(Jet::at(Iv::new(s,t)));
            match (l[0].v.bounds(),l[1].v.bounds(),l[2].v.bounds()) {
                (Some(x),Some(y),Some(z)) => for c in 0..8 {
                    crate::geom::include(&mut out,self.frame().at([x[c&1],y[(c>>1)&1],z[c>>2]]))
                },
                _ => {
                    let reach = (t-s).abs()**speed.get_or_insert_with(|| self.speed());
                    for v in [s,t] { let p = self.d2(v).0; for k in 0..3 { let mut q = p; q[k] -= reach; crate::geom::include(&mut out,q); q[k] += 2.*reach; crate::geom::include(&mut out,q) } }
                }
            }
        }
        out
    }
    /// **Where it crosses the plane through `o` square to the unit `n`** (both in space), as its
    /// parameters: the roots of `(C(t) − o) · n` isolated (`util::isolate::roots`) from its value and
    /// rate enclosed over each stretch by its jets read in outward-rounded intervals, a stretch
    /// it cannot part a touch where `near` says so — what a level line meets a turned contact's
    /// surface at (`query::line_turned`), a line meets a slid contour's cylinder at
    /// (`query::line_slid`, the plane through the line along the slide), and a plane along the slide
    /// meets that cylinder in (`ssi`'s `slid_meeting`).
    fn crossings(&self,o: V,n: V,near: &dyn Fn(f64) -> bool) -> Result<Vec<crate::util::roots::Root>,crate::util::isolate::GaveUp> {
        let (o,n) = (self.frame().local(o),self.frame().dir_local(n));
        fn height<T: Num>(l: J3<T>,o: V,n: V) -> Jet<T> { dot3(std::array::from_fn(|i| l[i]-Jet::of(o[i])),constant(n)) }
        let at = |t: f64| { let j = height(self.local(Jet::at(t)),o,n); (j.v,j.d) };
        let value = |t: f64| height(self.local(Jet::at(Iv::point(t))),o,n).v;
        let slope = |a: f64,b: f64| height(self.local(Jet::at(Iv::new(a,b))),o,n).d;
        let [a,b] = self.span();
        let (a,b) = (a.min(b),a.max(b));
        let g = crate::util::isolate::Parts {at:&at,value:&value,slope:&slope};
        crate::util::isolate::roots(&g,[a,b],1e-12*(b-a).max(1e-300),near)
    }
    /// The most length per unit of its parameter, read at 65 points of the span.
    fn speed(&self) -> f64 {
        let [a,b] = self.span();
        (0..=64).map(|k| crate::util::space::norm(self.d2(a+(b-a)*k as f64/64.).1)).fold(0.,f64::max)
    }
}
