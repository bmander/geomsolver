//! **Enclosures, one per kind of geometry** (#814 P2): each `Curve` and `Surface` kind encloses,
//! once, its point and its first derivatives over a box of its parameters in outward-rounded
//! intervals (`util::interval::Iv`), so every curve–surface pair has a certified reading by
//! construction (`query`'s general reading, by the Krawczyk operator `util::isolate`), and no
//! reading needs a bound written by hand for a pair of types — each of which restated its
//! surface's geometry and could contradict it (#812).
//!
//! The boundary is `Enclosed<P>`: a point set of `P` parameters (1 a curve, 2 a surface) answering
//! `enclose(box)` with a `Patch` — its point and its `P` partial derivatives, each an interval
//! vector holding every value they take over the box. What encloses each kind:
//!
//! * **B-splines, rational too** (`BSpline`, `Net`, a face family's spline run): over each knot span
//!   the box reaches, de Boor's algorithm read in intervals — its blending ratios held to `[0, 1]`,
//!   each step in its coefficients' hull — for the homogeneous point and for the derivative, a
//!   B-spline whose coefficients are the differences; the hull over the spans. At a knot of any
//!   multiplicity the box straddling it gets the hull of both one-sided derivatives, which is what
//!   the mean-value inclusion needs (#812's kink costs nothing). A rational one's point and
//!   derivative are its homogeneous form's quotients (its weights are positive).
//! * **Lines, conics, quadrics, the torus, revolutions, extrusions, blends and their carries, a
//!   face family's line and round runs**: their own formulas in intervals, the angles through
//!   `Interval::sin_cos_periodic`.
//! * **Affine images**: the preimage's enclosure carried through the map.
//! * **Turned and slid contacts** (`turned::Exact`): their own jets read in intervals.
//! * **A plane's cut of a screwed sheet** (`SheetCut`): `u = −α/β` and its rate in intervals, and
//!   beside a ruling the plane holds — where both vanish — the divided differences `α/δ`, `β/δ`
//!   enclosed by the mean value of `α'`, `β'` (and their rates by `α''/2`, `β''/2`), so the curve is
//!   enclosed through that point as anywhere.
//! * **Traced curves** (`Traced`): a trace is points pulled onto two surfaces by Newton, with no
//!   closed form to read in intervals; its enclosure is its own error bound — its points over the
//!   stretch, grown by the chords between them — measured, not proved, and said so (`Patch::proved`
//!   false), so a reading through it is profiled as measured.
use crate::geom::{Blend,Carry,Curve,Frame,SheetCut,Surface,Traced,V};
use crate::nurbs::{BSpline,Net};
use crate::util::interval::{Iv,Jet,Num};

/// Three intervals: a point or a vector enclosed.
pub type I3 = [Iv;3];

/// **What a point set of `P` parameters holds over a box of them**: its point and its partial
/// derivatives, enclosed, and whether the enclosure is proved (`false` for a measured one, a
/// trace's).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Patch<const P: usize> { pub at: I3,pub d: [I3;P],pub proved: bool }

impl<const P: usize> Patch<P> {
    /// Nothing known: every number.
    pub const ANY: Patch<P> = Patch {at:[Iv::ANY;3],d:[[Iv::ANY;3];P],proved:true};
    /// Whether every interval of it is one (none left its domain).
    pub fn bounded(&self) -> bool { self.at.iter().chain(self.d.iter().flatten()).all(|x| x.0.is_some()) }
}

/// **The boundary** (#814): a point set of `P` parameters encloses its point and first derivatives
/// over any box of them.
pub trait Enclosed<const P: usize> {
    fn enclose(&self,over: [[f64;2];P]) -> Patch<P>;
}

// ---------------------------------------------------------------------------------------------
// Interval vectors

pub fn c3(x: V) -> I3 { x.map(Iv::point) }
pub fn add3(a: I3,b: I3) -> I3 { std::array::from_fn(|i| a[i]+b[i]) }
pub fn sub3(a: I3,b: I3) -> I3 { std::array::from_fn(|i| a[i]-b[i]) }
pub fn scale3(a: I3,k: Iv) -> I3 { a.map(|x| x*k) }
pub fn dot3(a: I3,b: I3) -> Iv { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
pub fn cross3(a: I3,b: I3) -> I3 { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
pub fn hull3(a: I3,b: I3) -> I3 { std::array::from_fn(|i| a[i].hull(b[i])) }
pub fn meet3(a: I3,b: I3) -> I3 { std::array::from_fn(|i| a[i].meet(b[i])) }
/// A frame's point at local coordinates `l`.
pub fn frame_at(f: &Frame,l: I3) -> I3 { add3(c3(f.o),frame_dir(f,l)) }
/// A frame's direction at local components `l`.
pub fn frame_dir(f: &Frame,l: I3) -> I3 { std::array::from_fn(|i| Iv::point(f.x[i])*l[0]+Iv::point(f.y[i])*l[1]+Iv::point(f.z[i])*l[2]) }
/// A point's local coordinates in a frame.
pub fn frame_local(f: &Frame,p: I3) -> I3 { let q = sub3(p,c3(f.o)); [dot3(q,c3(f.x)),dot3(q,c3(f.y)),dot3(q,c3(f.z))] }
/// A vector's local components in a frame.
pub fn frame_dir_local(f: &Frame,d: I3) -> I3 { [dot3(d,c3(f.x)),dot3(d,c3(f.y)),dot3(d,c3(f.z))] }
/// `w` turned about the unit `k` by the angle whose sine and cosine are `(s, c)` (Rodrigues).
pub fn turn3(k: V,(s,c): (Iv,Iv),w: I3) -> I3 {
    let k = c3(k);
    add3(add3(scale3(w,c),scale3(cross3(k,w),s)),scale3(k,dot3(k,w)*(Iv::point(1.)-c)))
}
/// The interval `[a, b]` either way round.
fn iv([a,b]: [f64;2]) -> Iv { Iv::new(a,b) }
/// An interval's ends, where it is one.
fn ends(x: Iv) -> Option<[f64;2]> { x.bounds() }

// ---------------------------------------------------------------------------------------------
// B-splines

/// The knot span holding `t`, for `count` coefficients of `degree`.
fn span(degree: usize,knots: &[f64],count: usize,t: f64) -> usize {
    let (p,n) = (degree,count);
    if t >= knots[n] { return n-1 }
    if t <= knots[p] { return p }
    let (mut lo,mut hi) = (p,n);
    while hi-lo > 1 { let mid = (lo+hi)/2; if t < knots[mid] { hi = mid } else { lo = mid } }
    lo
}

/// The hull of some interval vectors.
fn hull_k<const K: usize>(c: &[[Iv;K]]) -> [Iv;K] {
    std::array::from_fn(|k| c.iter().skip(1).fold(c[0][k],|h,x| h.hull(x[k])))
}

/// **De Boor's algorithm in intervals** over span `s` of `knots` (degree `p`), `coeffs` the `p + 1`
/// coefficients reaching it, at `t` within the span: each blending ratio held to `[0, 1]` (the
/// parameter is within the span, so every ratio is), each step's result within its two
/// coefficients' hull — so the whole within theirs.
fn deboor<const K: usize>(p: usize,knots: &[f64],s: usize,coeffs: &[[Iv;K]],t: Iv) -> [Iv;K] {
    let mut d = coeffs.to_vec();
    let unit = Iv::new(0.,1.);
    for r in 1..=p {
        for j in (r..=p).rev() {
            let (k0,k1) = (knots[s+j-p],knots[s+1+j-r]);
            let alpha = ((t-Iv::point(k0))/(Iv::point(k1)-Iv::point(k0))).meet(unit);
            let (a,b) = (d[j-1],d[j]);
            d[j] = std::array::from_fn(|c| (a[c]+alpha*(b[c]-a[c])).meet(a[c].hull(b[c])));
        }
    }
    d[p]
}

/// The coefficients of a spline's derivative reaching span `s`: `p (c_{i+1} − c_i) / (t_{i+p+1} −
/// t_{i+1})` for `i` in `s − p .. s` (the span is not empty, so neither is any gap).
fn differences<const K: usize>(p: usize,knots: &[f64],s: usize,c: &[[Iv;K]]) -> Vec<[Iv;K]> {
    (0..p).map(|j| {
        let i = s-p+j;
        let gap = Iv::point(knots[i+p+1])-Iv::point(knots[i+1]);
        std::array::from_fn(|k| (c[j+1][k]-c[j][k])*Iv::point(p as f64)/gap)
    }).collect()
}

/// **A spline's value and derivative over `[a, b]`, enclosed** (the module's): `coeff(i)` its
/// `i`th coefficient (a homogeneous point, say), `count` of them. Read span by span over the spans
/// the stretch reaches, by `deboor` (its derivative over the differences, a spline of a degree less
/// on the knots but the first and last), and the hull taken; over more than `WIDE` spans, the hull
/// of the coefficients and of their differences, which hold the spline and its derivative too. A
/// stretch past the domain is clamped, as the spline's own reading is, its derivative there the
/// hull with nought.
const WIDE: usize = 4;
fn spline<const K: usize>(p: usize,knots: &[f64],count: usize,coeff: &dyn Fn(usize) -> [Iv;K],[a,b]: [f64;2]) -> ([Iv;K],[Iv;K]) {
    let (lo,hi) = (knots[p],knots[count]);
    let past = a < lo || b > hi;
    let (a,b) = (a.clamp(lo,hi),b.clamp(lo,hi));
    let (s0,s1) = (span(p,knots,count,a),span(p,knots,count,b));
    let reach: Vec<usize> = (s0..=s1).filter(|&s| knots[s] < knots[s+1] && knots[s] <= b && knots[s+1] >= a).collect();
    let coeffs: Vec<[Iv;K]> = (s0-p..=s1).map(coeff).collect();
    let (mut value,mut rate): (Option<[Iv;K]>,Option<[Iv;K]>) = (None,None);
    let mut take = |v: [Iv;K],d: [Iv;K]| {
        value = Some(value.map_or(v,|h| std::array::from_fn(|k| h[k].hull(v[k]))));
        rate = Some(rate.map_or(d,|h| std::array::from_fn(|k| h[k].hull(d[k]))));
    };
    if reach.len() > WIDE {
        let diffs: Vec<[Iv;K]> = (s0..=s1).flat_map(|s| differences(p,knots,s,&coeffs[s-s0..=s-s0+p])).collect();
        take(hull_k(&coeffs),hull_k(&diffs));
    } else {
        for &s in &reach {
            let c = &coeffs[s-s0..=s-s0+p];
            let t = Iv::new(a.max(knots[s]),b.min(knots[s+1]));
            let v = deboor(p,knots,s,c,t);
            let v: [Iv;K] = std::array::from_fn(|k| v[k].meet(hull_k(c)[k]));
            let dc = differences(p,knots,s,c);
            let d = if p == 1 { dc[0] } else { deboor(p-1,&knots[1..],s-1,&dc,t) };
            let d: [Iv;K] = std::array::from_fn(|k| d[k].meet(hull_k(&dc)[k]));
            take(v,d);
        }
    }
    let (v,mut d) = (value.unwrap_or([Iv::ANY;K]),rate.unwrap_or([Iv::ANY;K]));
    if past { d = d.map(|x| x.hull(Iv::point(0.))) }
    (v,d)
}

/// A homogeneous point `[w x, w y, w z, w]` and its derivative carried to the point and its
/// derivative: `P = A / W`, `P' = (A' − P W') / W`.
fn quotient(a: [Iv;4],d: [Iv;4]) -> (I3,I3) {
    let w = a[3];
    let p: I3 = std::array::from_fn(|k| a[k]/w);
    (p,std::array::from_fn(|k| (d[k]-p[k]*d[3])/w))
}

/// A pole and its weight, homogeneous, enclosed.
fn lift(q: V,w: f64) -> [Iv;4] { let w = Iv::point(w); [Iv::point(q[0])*w,Iv::point(q[1])*w,Iv::point(q[2])*w,w] }

impl Enclosed<1> for BSpline {
    fn enclose(&self,[t]: [[f64;2];1]) -> Patch<1> {
        let w = |i: usize| self.weights.as_ref().map_or(1.,|w| w[i]);
        let (a,d) = spline(self.degree,&self.knots,self.poles.len(),&|i| lift(self.poles[i],w(i)),[t[0].min(t[1]),t[0].max(t[1])]);
        let (at,d) = quotient(a,d);
        // (a curve keeps within its poles' hull: its weights are positive)
        let poles = self.poles_over(t);
        let hull: I3 = std::array::from_fn(|k| Iv::of_all(poles.iter().map(|q| q[k])));
        Patch {at:meet3(at,hull),d:[d],proved:true}
    }
}

impl Enclosed<2> for Net {
    fn enclose(&self,[u,v]: [[f64;2];2]) -> Patch<2> {
        let (u,v) = ([u[0].min(u[1]),u[0].max(u[1])],[v[0].min(v[1]),v[0].max(v[1])]);
        let (nu,nv) = (self.poles.len(),self.poles[0].len());
        let w = |i: usize,j: usize| self.weights.as_ref().map_or(1.,|w| w[i][j]);
        // each row along `v` read over `v`, then the rows read along `u`: the point and `S_v` from
        // the rows' values, `S_u` from their values' differences
        let rows: Vec<([Iv;4],[Iv;4])> = (0..nu).map(|i| spline(self.dv,&self.vknots,nv,&|j| lift(self.poles[i][j],w(i,j)),v)).collect();
        let at_u = |k: usize| -> ([Iv;4],[Iv;4]) {
            let pick = |i: usize| if k == 0 { rows[i].0 } else { rows[i].1 };
            spline(self.du,&self.uknots,nu,&pick,u)
        };
        let ((a,au),(av,_)) = (at_u(0),at_u(1));
        let (s,su) = quotient(a,au);
        let (_,sv) = quotient(a,av);
        let hull: I3 = std::array::from_fn(|k| Iv::of_all(self.poles_over(u,v).map(|q| q[k])));
        Patch {at:meet3(s,hull),d:[su,sv],proved:true}
    }
}

// ---------------------------------------------------------------------------------------------
// Curves

impl Enclosed<1> for Curve {
    fn enclose(&self,[t]: [[f64;2];1]) -> Patch<1> {
        let tv = iv(t);
        let conic = |f: &Frame,a: f64,b: f64| {
            let (s,c) = tv.sin_cos();
            let (a,b) = (Iv::point(a),Iv::point(b));
            Patch {at:frame_at(f,[a*c,b*s,Iv::point(0.)]),d:[frame_dir(f,[-(a*s),b*c,Iv::point(0.)])],proved:true}
        };
        match self {
            Curve::Line {p,d} => Patch {at:add3(c3(*p),scale3(c3(*d),tv)),d:[c3(*d)],proved:true},
            Curve::Circle(f,r) => conic(f,*r,*r),
            Curve::Ellipse(f,a,b) => conic(f,*a,*b),
            Curve::BSpline(b) => b.enclose([t]),
            Curve::Traced(c) => c.enclose([t]),
            Curve::Iso(b,u) => { let p = b.enclose([[*u,*u],t]); Patch {at:p.at,d:[p.d[1]],proved:p.proved} }
            Curve::SheetCut(c) => c.enclose([t]),
            Curve::Rail(w,u) => { let p = w.enclose([[*u,*u],t]); Patch {at:p.at,d:[p.d[1]],proved:p.proved} }
            Curve::Rim(w,s) => { let p = w.enclose([t,[*s,*s]]); Patch {at:p.at,d:[p.d[0]],proved:p.proved} }
            Curve::Affine(m) => {
                // the preimage's parameter `(t − b)/a`, and the derivative over `a`
                let (a,b) = (Iv::point(m.at[0]),Iv::point(m.at[1]));
                let Some(local) = ends((tv-b)/a) else { return Patch::ANY };
                let p = m.of.enclose([local]);
                Patch {at:affine_point(&m.map,p.at),d:[scale3(affine_vector(&m.map,p.d[0]),Iv::point(1.)/a)],proved:p.proved}
            }
            Curve::Turned(m) => exact(&**m,t),
            Curve::Slid(m) => exact(&**m,t),
        }
    }
}

/// An affine map's image of a point, enclosed.
pub fn affine_point(a: &crate::affine::Affine,p: I3) -> I3 { add3(affine_vector(a,p),c3(a.t)) }
/// An affine map's image of a vector, enclosed.
pub fn affine_vector(a: &crate::affine::Affine,v: I3) -> I3 { std::array::from_fn(|i| dot3(c3(a.m[i]),v)) }

/// A curve read exactly in a frame (`turned::Exact`), its jets in intervals: over a stretch where
/// a reading leaves its domain (a root of an interval reaching below nought, a quotient by one
/// holding it — an interval's overestimate over a wide stretch), its halves, eight times at most,
/// and their hull.
fn exact(c: &dyn ExactJets,t: [f64;2]) -> Patch<1> { exact_within(c,[t[0].min(t[1]),t[0].max(t[1])],8) }

fn exact_within(c: &dyn ExactJets,[a,b]: [f64;2],depth: usize) -> Patch<1> {
    let (l,f) = c.jets(Iv::new(a,b));
    let p = Patch {at:frame_at(&f,l[0]),d:[frame_dir(&f,l[1])],proved:true};
    if p.bounded() || depth == 0 || !(b > a) { return p }
    let m = 0.5*(a+b);
    let (x,y) = (exact_within(c,[a,m],depth-1),exact_within(c,[m,b],depth-1));
    Patch {at:hull3(x.at,y.at),d:[hull3(x.d[0],y.d[0])],proved:true}
}

/// What `exact` reads of a `turned::Exact` curve (its generic `local` is not object safe).
trait ExactJets { fn jets(&self,t: Iv) -> ([I3;2],Frame); }
impl<E: crate::turned::Exact> ExactJets for E {
    fn jets(&self,t: Iv) -> ([I3;2],Frame) {
        let l = self.local(Jet::at(t));
        ([l.map(|j| j.v),l.map(|j| j.d)],*self.frame())
    }
}

impl Enclosed<1> for Traced {
    /// **A trace's own error bound** (the module's): its points at the stretch's ends, its own
    /// points within it and its middle, hulled and grown by the longest chord between them — what a
    /// curve between two points pulled onto both surfaces can stray by — and its derivative the
    /// hull of its central differences there, grown by their spread. Measured: `proved` false.
    fn enclose(&self,[t]: [[f64;2];1]) -> Patch<1> {
        let (a,b) = (t[0].min(t[1]),t[0].max(t[1]));
        let mut ts = vec![a,0.5*(a+b),b];
        ts.extend((a.ceil() as i64..=b.floor() as i64).map(|k| k as f64).filter(|&k| k > a && k < b));
        ts.sort_by(f64::total_cmp);
        let pts: Vec<(V,V)> = ts.iter().map(|&s| self.d1_at(s)).collect();
        let chord = pts.windows(2).map(|w| crate::util::space::distance(w[0].0,w[1].0)).fold(0.,f64::max);
        let grow = |x: Iv,by: f64| Iv::new(x.lo()-by,x.hi()+by);
        let at: I3 = std::array::from_fn(|k| grow(Iv::of_all(pts.iter().map(|p| p.0[k])),chord+1e-12*(1.+x_scale(&pts))));
        let d: I3 = std::array::from_fn(|k| {
            let h = Iv::of_all(pts.iter().map(|p| p.1[k]));
            let spread = (h.hi()-h.lo()).max(1e-6*h.mag());
            grow(h,spread)
        });
        Patch {at,d:[d],proved:false}
    }
}

/// The largest coordinate among some points.
fn x_scale(pts: &[(V,V)]) -> f64 { pts.iter().map(|p| p.0.iter().fold(0f64,|m,x| m.max(x.abs()))).fold(0.,f64::max) }

impl Enclosed<1> for SheetCut {
    /// `C(v) = S(u(v), v)`, `C' = S_u u' + S_v`, `u = −α/β` (the module's).
    fn enclose(&self,[t]: [[f64;2];1]) -> Patch<1> {
        let (a,b) = (t[0].min(t[1]),t[0].max(t[1]));
        let Some((u,du)) = self.u_enclosed([a,b]) else { return Patch::ANY };
        let Some(us) = ends(u) else { return Patch::ANY };
        let s = self.blend.enclose([us,[a,b]]);
        Patch {at:s.at,d:[add3(scale3(s.d[0],du),s.d[1])],proved:s.proved}
    }
}

impl SheetCut {
    /// `α` and `β` at the jet `v`: `α = n·S(0, v) − h`, `β = n·S_u(v)` (`coefficients`), written
    /// once over `Num`.
    pub fn alpha_beta<T: Num>(&self,v: Jet<T>) -> (Jet<T>,Jet<T>) {
        let l = self.line();
        let (k,n) = (l.axis,self.n);
        let (s,c) = v.times(l.angle).sin_cos();
        // `n · R(θ) w` for a fixed `w`: `n·w c + n·(k × w) s + (n·k)(k·w)(1 − c)`
        let turned = |w: V| -> Jet<T> {
            let (nw,nkw,nk,kw) = (crate::util::space::dot(n,w),crate::util::space::dot(n,crate::util::space::cross(k,w)),
                crate::util::space::dot(n,k),crate::util::space::dot(k,w));
            c.times(nw)+s.times(nkw)+(Jet::of(1.)-c).times(nk*kw)
        };
        let alpha = turned(crate::util::space::sub(l.p0,l.centre))+Jet::of(crate::util::space::dot(n,l.centre)-self.h)
            +v.times(crate::util::space::dot(n,k)*l.rise);
        (alpha,turned(l.run))
    }
    /// **`u` and its rate over `[a, b]`, enclosed**: `−α/β` and `−(α'β − αβ')/β²` read in intervals;
    /// beside a held ruling `r` — where `α` and `β` both vanish and the quotient is the limit `−α'/β'`
    /// — the divided differences `A = α/(v − r)` and `B = β/(v − r)` over the hull of the stretch
    /// and `r`: `A ∈ α'`, `A' ∈ α''/2` there (Taylor's theorem with the mean value), likewise `B`,
    /// so `u = −A/B` and `u' = −(A'B − AB')/B²`. `None` where `β` (or `B`) may vanish: the curve
    /// runs off to infinity there.
    pub fn u_enclosed(&self,[a,b]: [f64;2]) -> Option<(Iv,Iv)> { self.u_within([a,b],8) }
    /// `u_enclosed`, a stretch whose `β` is not proved off nought (an interval's overestimate over a
    /// wide one, or a pole) read as its halves, `depth` times at most, and their hull taken.
    fn u_within(&self,[a,b]: [f64;2],depth: usize) -> Option<(Iv,Iv)> {
        if let Some(e) = self.u_once([a,b]) { return Some(e) }
        if depth == 0 || !(b > a) { return None }
        let m = 0.5*(a+b);
        let ((u0,d0),(u1,d1)) = (self.u_within([a,m],depth-1)?,self.u_within([m,b],depth-1)?);
        Some((u0.hull(u1),d0.hull(d1)))
    }
    /// `u_enclosed` over the stretch at once, or `None`.
    fn u_once(&self,[a,b]: [f64;2]) -> Option<(Iv,Iv)> {
        let (al,be) = self.alpha_beta(Jet::at(Iv::new(a,b)));
        if !be.v.has_zero() {
            let u = -(al.v/be.v);
            let du = -((al.d*be.v-al.v*be.d)/(be.v*be.v));
            return (u.0.is_some() && du.0.is_some()).then_some((u,du))
        }
        // (only a held ruling within the stretch's reach is read so: beside a far one, halve)
        let r = self.held.iter().copied().min_by(|x,y| (x-0.5*(a+b)).abs().total_cmp(&(y-0.5*(a+b)).abs()))
            .filter(|&r| (r-0.5*(a+b)).abs() <= (b-a).max(1e-9))?;
        let (al,be) = self.alpha_beta(Jet::at(Iv::new(a.min(r),b.max(r))));
        let (big_a,big_b) = (al.d,be.d);
        let half = Iv::point(0.5);
        let (da,db) = (al.dd*half,be.dd*half);
        if big_b.has_zero() { return None }
        let u = -(big_a/big_b);
        let du = -((da*big_b-big_a*db)/(big_b*big_b));
        (u.0.is_some() && du.0.is_some()).then_some((u,du))
    }
}

// ---------------------------------------------------------------------------------------------
// Surfaces

impl Enclosed<2> for Surface {
    fn enclose(&self,[u,v]: [[f64;2];2]) -> Patch<2> {
        let (uv,vv) = (iv(u),iv(v));
        let zero = Iv::point(0.);
        let p = |f: &Frame,l: I3,du: I3,dv: I3| Patch {at:frame_at(f,l),d:[frame_dir(f,du),frame_dir(f,dv)],proved:true};
        match self {
            Surface::Plane(f) => p(f,[uv,vv,zero],[Iv::point(1.),zero,zero],[zero,Iv::point(1.),zero]),
            Surface::Cylinder(f,r) => {
                let (s,c) = uv.sin_cos();
                let r = Iv::point(*r);
                p(f,[r*c,r*s,vv],[-(r*s),r*c,zero],[zero,zero,Iv::point(1.)])
            }
            Surface::Cone(f,r,a) => {
                use crate::util::fmath::Det;
                let (sa,ca) = a.dsin_cos();
                let (s,c) = uv.sin_cos();
                let q = Iv::point(*r)+vv*Iv::point(sa);
                p(f,[q*c,q*s,vv*Iv::point(ca)],[-(q*s),q*c,zero],[Iv::point(sa)*c,Iv::point(sa)*s,Iv::point(ca)])
            }
            Surface::Sphere(f,r) => {
                let ((su,cu),(sv,cv)) = (uv.sin_cos(),vv.sin_cos());
                let r = Iv::point(*r);
                p(f,[r*cv*cu,r*cv*su,r*sv],[-(r*cv*su),r*cv*cu,zero],[-(r*sv*cu),-(r*sv*su),r*cv])
            }
            Surface::Torus(f,big,r) => {
                let ((su,cu),(sv,cv)) = (uv.sin_cos(),vv.sin_cos());
                let r = Iv::point(*r);
                let q = Iv::point(*big)+r*cv;
                p(f,[q*cu,q*su,r*sv],[-(q*su),q*cu,zero],[-(r*sv*cu),-(r*sv*su),r*cv])
            }
            Surface::Extrusion(f,c) => {
                let e = c.enclose([u]);
                Patch {at:add3(e.at,scale3(c3(f.z),vv)),d:[e.d[0],c3(f.z)],proved:e.proved}
            }
            Surface::Revolution(f,c) => {
                // the meridian's point and rate in the frame, turned by `u`
                let e = c.enclose([v]);
                let (q,dq) = (frame_local(f,e.at),frame_dir_local(f,e.d[0]));
                let (s,cs) = uv.sin_cos();
                let turn = |q: I3| [q[0]*cs-q[1]*s,q[0]*s+q[1]*cs,q[2]];
                let r = turn(q);
                Patch {at:frame_at(f,r),d:[frame_dir(f,[-r[1],r[0],zero]),frame_dir(f,turn(dq))],proved:e.proved}
            }
            Surface::Blend(_,b) => b.enclose([u,v]),
            Surface::BSpline(_,n) => n.enclose([u,v]),
            Surface::Family(_,w) => w.enclose([u,v]),
            Surface::Affine(m) => {
                let e = m.of.enclose([u,v]);
                Patch {at:affine_point(&m.map,e.at),d:e.d.map(|d| affine_vector(&m.map,d)),proved:e.proved}
            }
        }
    }
}

impl Enclosed<2> for Blend {
    /// `S = carry_v((1 − v) A(a₀ + u Δa) + v B(b₀ + u Δb))` and its partials, each edge enclosed
    /// over the stretch `u` reaches of it and the carry read in intervals.
    fn enclose(&self,[u,v]: [[f64;2];2]) -> Patch<2> {
        let (uv,vv) = (iv(u),iv(v));
        let edge = |c: &Curve,t: [f64;2]| -> Option<(Patch<1>,Iv)> {
            let (t0,dt) = (Iv::point(t[0]),Iv::point(t[1])-Iv::point(t[0]));
            let over = ends(t0+uv*dt)?;
            Some((c.enclose([over]),dt))
        };
        let (Some((a,da)),Some((b,db))) = (edge(&self.a,self.ta),edge(&self.b,self.tb)) else { return Patch::ANY };
        let one = Iv::point(1.);
        let p = add3(scale3(a.at,one-vv),scale3(b.at,vv));
        let pu = add3(scale3(a.d[0],da*(one-vv)),scale3(b.d[0],db*vv));
        let pv = sub3(b.at,a.at);
        let proved = a.proved && b.proved;
        match self.carry {
            Carry::Line {delta} => Patch {at:add3(p,scale3(c3(delta),vv)),d:[pu,add3(pv,c3(delta))],proved},
            Carry::Screw {center,axis,angle,advance} => {
                let sc = (vv*Iv::point(angle)).sin_cos();
                let q = add3(add3(c3(center),turn3(axis,sc,sub3(p,c3(center)))),scale3(c3(axis),vv*Iv::point(advance)));
                // the carry's rate at `q`: `angle axis × (q − c) + advance axis`
                let rate = add3(scale3(cross3(c3(axis),sub3(q,c3(center))),Iv::point(angle)),scale3(c3(axis),Iv::point(advance)));
                Patch {at:q,d:[turn3(axis,sc,pu),add3(turn3(axis,sc,pv),rate)],proved}
            }
        }
    }
}

impl Enclosed<2> for crate::family::Sheet {
    /// The edge's numbers over the stations' pieces the box reaches — each piece's polynomial in
    /// its own variable read by Horner's rule in intervals, with its rate — and the edge read with
    /// them (`Run`'s line, round or spline), the hull over the pieces.
    fn enclose(&self,[u,s]: [[f64;2];2]) -> Patch<2> {
        let (s0,s1) = (s[0].min(s[1]),s[0].max(s[1]));
        let uv = iv(u);
        let st = &self.stations;
        let mut out: Option<Patch<2>> = None;
        for (j,sp) in st.spans.iter().enumerate() {
            let last = j+1 == st.spans.len();
            // the end pieces take what lies past them
            let (lo,hi) = (if j == 0 { s0 } else { s0.max(sp[0]) },if last { s1 } else { s1.min(sp[1]) });
            if j > 0 && s1 < sp[0] || !last && s0 > sp[1] || lo > hi { continue }
            let (mid,half) = (Iv::point(0.5)*(Iv::point(sp[0])+Iv::point(sp[1])),Iv::point(0.5)*(Iv::point(sp[1])-Iv::point(sp[0])));
            let t = (Iv::new(lo,hi)-mid)/half;
            let inv = st.inverse(j);
            let nodes = &self.nodes[6*j..6*j+6];
            let len = nodes[0].len();
            let (n,dn): (Vec<Iv>,Vec<Iv>) = (0..len).map(|q| {
                // the number's coefficients in `t`, then Horner's rule for it and its rate
                let a: [Iv;6] = std::array::from_fn(|k| (0..6).fold(Iv::point(0.),|acc,i| acc+Iv::point(inv[k][i])*Iv::point(nodes[i][q])));
                let value = a.iter().rev().fold(Iv::point(0.),|acc,&x| acc*t+x);
                let rate = (1..6).rev().fold(Iv::point(0.),|acc,k| acc*t+a[k]*Iv::point(k as f64))/half;
                (value,rate)
            }).unzip();
            let p = run_enclosed(&self.run,&n,&dn,uv);
            out = Some(match out { None => p,Some(h) => Patch {at:hull3(h.at,p.at),d:[hull3(h.d[0],p.d[0]),hull3(h.d[1],p.d[1])],proved:true} });
        }
        out.unwrap_or(Patch::ANY)
    }
}

/// A face family's edge with numbers `n` (running at `dn`) at the fraction `u` of its run,
/// enclosed: its point, `∂/∂u` and `∂/∂s` (`family::Run::at` and `Sheet::rate`).
fn run_enclosed(run: &crate::family::Run,n: &[Iv],dn: &[Iv],u: Iv) -> Patch<2> {
    use crate::family::Run;
    let v3 = |n: &[Iv],i: usize| -> I3 { [n[i],n[i+1],n[i+2]] };
    match run {
        Run::Line => {
            let (a,b,da,db) = (v3(n,0),v3(n,3),v3(dn,0),v3(dn,3));
            Patch {at:add3(a,scale3(sub3(b,a),u)),d:[sub3(b,a),add3(da,scale3(sub3(db,da),u))],proved:true}
        }
        Run::Round => {
            let (c,r0,r9,sweep) = (v3(n,0),v3(n,3),v3(n,6),n[9]);
            let (sn,cs) = (u*sweep).sin_cos();
            let radial = add3(scale3(r0,cs),scale3(r9,sn));
            let along = add3(scale3(r0,-sn),scale3(r9,cs));
            let moved = add3(v3(dn,0),add3(scale3(v3(dn,3),cs),scale3(v3(dn,6),sn)));
            Patch {at:add3(c,radial),d:[scale3(along,sweep),add3(moved,scale3(along,u*dn[9]))],proved:true}
        }
        Run::Spline {degree,knots,weights} => {
            let k = n.len();
            let count = (k-2)/3;
            let w = |i: usize| weights.as_ref().map_or(1.,|w| w[i]);
            let (t0,t1,dt0,dt1) = (n[k-2],n[k-1],dn[k-2],dn[k-1]);
            let tt = t0+u*(t1-t0);
            let Some(over) = ends(tt) else { return Patch::ANY };
            let lifted = |src: &[Iv],i: usize| -> [Iv;4] { let wi = Iv::point(w(i)); [src[3*i]*wi,src[3*i+1]*wi,src[3*i+2]*wi,wi] };
            let (a,d) = spline(*degree,knots,count,&|i| lifted(n,i),over);
            let (x,xt) = quotient(a,d);
            // the poles' rates read by the same basis (the weights stand still)
            let (ra,_) = spline(*degree,knots,count,&|i| lifted(dn,i),over);
            let moved: I3 = std::array::from_fn(|c| ra[c]/ra[3]);
            let dt = dt0+u*(dt1-dt0);
            Patch {at:x,d:[scale3(xt,t1-t0),add3(moved,scale3(xt,dt))],proved:true}
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Carries

/// The angle a carry turns by at `v`, its sine and cosine enclosed (none for a step along a line).
fn carry_turn(c: &Carry,v: Iv) -> Option<(V,(Iv,Iv))> {
    match *c { Carry::Line {..} => None,Carry::Screw {axis,angle,..} => Some((axis,(v*Iv::point(angle)).sin_cos())) }
}

/// `p` carried to `v` (`Carry::at`), enclosed.
pub fn carry_at(c: &Carry,p: I3,v: Iv) -> I3 {
    match *c {
        Carry::Line {delta} => add3(p,scale3(c3(delta),v)),
        Carry::Screw {center,axis,advance,..} => {
            let (_,sc) = carry_turn(c,v).expect("a screw turns");
            add3(add3(c3(center),turn3(axis,sc,sub3(p,c3(center)))),scale3(c3(axis),v*Iv::point(advance)))
        }
    }
}

/// A direction carried to `v` (`Carry::dir`), enclosed.
pub fn carry_dir(c: &Carry,d: I3,v: Iv) -> I3 {
    match carry_turn(c,v) { None => d,Some((axis,sc)) => turn3(axis,sc,d) }
}

/// The carry's rate at the carried point `q` (`Carry::rate`), enclosed.
pub fn carry_rate(c: &Carry,q: I3) -> I3 {
    match *c {
        Carry::Line {delta} => c3(delta),
        Carry::Screw {center,axis,angle,advance} =>
            add3(scale3(cross3(c3(axis),sub3(q,c3(center))),Iv::point(angle)),scale3(c3(axis),Iv::point(advance))),
    }
}

/// A point of the circle `o + r (cos θ x + sin θ y)` in `f` over `θ`, and its rate, enclosed.
pub fn circle_at(f: &Frame,r: f64,th: Iv) -> (I3,I3) {
    let (s,c) = th.sin_cos();
    let r = Iv::point(r);
    (frame_at(f,[r*c,r*s,Iv::point(0.)]),frame_dir(f,[-(r*s),r*c,Iv::point(0.)]))
}

/// **A scalar spline's value and rate over `[a, b]`, enclosed** (`spline`, its coefficients `c`):
/// what a B-spline curve's heights over a plane are read by (`query::spline_plane`).
pub fn scalar(degree: usize,knots: &[f64],c: &[f64],t: [f64;2]) -> (Iv,Iv) {
    let (v,d) = spline::<1>(degree,knots,c.len(),&|i| [Iv::point(c[i])],[t[0].min(t[1]),t[0].max(t[1])]);
    (v[0],d[0])
}
