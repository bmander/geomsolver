//! **A torus's contact under a slide, and its apparent contour** (#805, #780 L1's X2 under a
//! constant twist): the curve along which a torus slid at a lean to its axis touches the solid it
//! sweeps, read exactly, and that curve seen along the slide.
//!
//! On the torus about its frame's `z`, its tube's middle radius `R` and the tube's `r`, the point at
//! angle `u` round the axis and `v` round the tube is `R ρ̂ + r n`, its normal `n = cos v ρ̂ + sin v
//! z`. Under the slide `D` (unit, in the frame) the point is on the contact where `n · D = 0`:
//!
//! ```text
//!     cos v g(u) + sin v D_z = 0,   g(u) = D_x cos u + D_y sin u.
//! ```
//!
//! Read in the height chart `v` (`sin v` against `cos v`) that is two roots on each latitude that meet
//! where `|v| = v*`, the silhouette's top and bottom: the branch locus `A² + B² = C²` on the face
//! (docs/swept-solid-design.md step 6). Read in the angle chart it is smooth: `n` lies in the plane of
//! `ρ̂` and `z` and square to `D`, so `n = σ (D_z ρ̂ − g z) / √(D_z² + g²)`, `σ = ±1` — two closed
//! curves, `tan v(u) = −g(u)/D_z` (`σ D_z > 0`, the tube's outer half) and that turned by `π` (the
//! inner half), each smooth wherever the slide leans off square to the axis (`D_z ≠ 0`). So the
//! patch is read across the locus, not cut there. Written rationally in `s = tan(u/2)`, `ρ̂ = ((1 −
//! s²) x + 2s y)/(1 + s²)`, its jets stay in `turned::Num` — floats, and outward-rounded intervals —
//! each closed curve in two halves, `s ∈ [−1, 1]` about the frame's `x` and about its `−x`.
//!
//! Swept along the slide each contact is a cylinder over its **apparent contour**, the contact seen
//! along `D` (its points less their part along `D`): the outer contour, the torus's outline, and the
//! inner, the outline of its hole. The inner contour **folds** — cusps where the contact runs along
//! `D`, an asymptotic direction of the tube's saddle — once the slide leans `β` from the axis with
//! `tan² β ≥ (R − r)/r` (first at `u = ±π/2`, across the lean): the fold locus (`folded`), beyond
//! which the sheet doubles back. Short of it the contour is a regular curve and every line along the
//! slide through a contact point meets the torus there alone (`meets_again`), so the cylinder touches
//! the torus at an end pose along the contact and nowhere else.
use crate::geom::{Box3,Frame,Rigid,V};
use crate::turned::{Iv,Jet,Num};
use crate::util::space::{dot,norm,sub};

type J3<T> = [Jet<T>;3];

/// **A torus's contact under a slide, on one side** (the module's): the tube's middle radius `big`
/// and its radius `small`, the slide's unit direction `d` in the torus's frame (its `z` the torus's
/// axis), and which half of the tube (`side`: `+1` the outer, where `n · ρ̂ > 0`, `−1` the inner).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Contact { pub big: f64,pub small: f64,pub d: V,pub side: f64 }

impl Contact {
    /// The contact's point at the jet `s = tan(u/2)` of the angle round the axis from the frame's `x`.
    pub fn point<T: Num>(&self,s: Jet<T>) -> J3<T> {
        let one = Jet::of(1.);
        let w = one+s*s;
        let (c,sn) = ((one-s*s)/w,s.times(2.)/w);
        let [dx,dy,dz] = self.d;
        let g = c.times(dx)+sn.times(dy);
        let q = (g*g+Jet::of(dz*dz)).sqrt();
        // n = σ (D_z ρ̂ − g z)/q, σ the side's sign against D_z's
        let sigma = self.side*dz.signum();
        let reach = Jet::of(self.big)+(Jet::of(sigma*self.small*dz)/q);
        [reach*c,reach*sn,(g/q).times(-sigma*self.small)]
    }
    /// The contact's apparent contour at `s`: its point less its part along the slide.
    pub fn contour<T: Num>(&self,s: Jet<T>) -> J3<T> {
        let x = self.point(s);
        let along = x[0].times(self.d[0])+x[1].times(self.d[1])+x[2].times(self.d[2]);
        std::array::from_fn(|i| x[i]-along.times(self.d[i]))
    }
    /// How far past meeting the torus again a line along the slide through the contact at `s`
    /// stands: the line `p + t D` meets the torus where `(|p + tD|² + R² − r²)² = 4R² |(p + tD)ₓᵧ|²`,
    /// a quartic with a double root at the contact (`t = 0`), the rest `t² + a t + b` whose
    /// discriminant is `8 (2 R² |Dₓᵧ|² − |p|² − R² + r²)`. Below nought it meets the torus there
    /// alone; the value is that discriminant over eight.
    pub fn again<T: Num>(&self,s: Jet<T>) -> Jet<T> {
        let x = self.point(s);
        let (big,small) = (self.big,self.small);
        let flat = self.d[0]*self.d[0]+self.d[1]*self.d[1];
        Jet::of(2.*big*big*flat-big*big+small*small)-(x[0]*x[0]+x[1]*x[1]+x[2]*x[2])
    }
}

/// **A torus's contact under a slide, as a curve** (`geom::Curve::Slid`): one half of the contact
/// (`s ∈ span`, within `[−1, 1]` about the frame's `x`), read in `frame` — the torus's own frame,
/// its origin the torus's centre and its `z` its axis, its `x` the half's middle — either the
/// contact itself (`contour` false: where the torus at a pose of the slide touches the cylinder the
/// contact sweeps, `ssi`'s `slid_touch`) or its apparent contour, in the plane through the frame's
/// origin square to the slide (`contour`: the cylinder's section, `Surface::Extrusion` of it).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Slid { pub frame: Frame,pub contact: Contact,pub span: [f64;2],pub contour: bool }

/// `[a, b]` in `n` pieces, each an interval.
fn pieces([a,b]: [f64;2],n: usize) -> impl Iterator<Item=[f64;2]> {
    (0..n).map(move |k| [a+(b-a)*k as f64/n as f64,a+(b-a)*(k+1) as f64/n as f64])
}

/// Whether `held` holds over `[a, b]`, read over it whole and, where it does not, over its halves,
/// to `depth` halvings: the first stretch it is not proved over, or none.
fn proved(held: &dyn Fn(Iv) -> bool,[a,b]: [f64;2],depth: usize) -> Option<f64> {
    if held(Iv::new(a,b)) { return None }
    if depth == 0 { return Some(0.5*(a+b)) }
    let m = 0.5*(a+b);
    proved(held,[a,m],depth-1).or_else(|| proved(held,[m,b],depth-1))
}

impl Slid {
    /// Its point at the jet `s`, in its frame's coordinates.
    fn local<T: Num>(&self,s: Jet<T>) -> J3<T> {
        if self.contour { self.contact.contour(s) } else { self.contact.point(s) }
    }
    /// `C`, `C'`, `C''` at `s`.
    pub fn d2(&self,s: f64) -> (V,V,V) {
        let (f,l) = (&self.frame,self.local(Jet::at(s)));
        (f.at(l.map(|j| j.v)),f.dir(l.map(|j| j.d)),f.dir(l.map(|j| j.dd)))
    }
    /// The slide's unit direction, in space.
    pub fn slide(&self) -> V { self.frame.dir(self.contact.d) }
    /// **Where its contour folds** (the fold locus, docs/swept-solid-design.md step 6): the first
    /// stretch of `span` over which the contour's tangent is not proved off nought — its derivative
    /// held in outward intervals over each of 64 pieces, halved where that does not prove it, to
    /// eight halvings — as the parameter there; none where it is regular all along.
    pub fn folded(&self) -> Option<f64> {
        let held = |v: Iv| {
            let p = self.contact.contour(Jet::at(v));
            (p[0].d*p[0].d+p[1].d*p[1].d+p[2].d*p[2].d).bounds().is_some_and(|[lo,_]| lo > 0.)
        };
        pieces(self.span,64).find_map(|w| proved(&held,w,8))
    }
    /// **Where a line along the slide through its contact meets the torus again** (`Contact::again`):
    /// the first stretch of `span` over which that is not proved nowhere — the discriminant held
    /// below nought in outward intervals over each of 64 pieces, halved where that does not prove it,
    /// to eight halvings — as the parameter there; none where each such line touches the torus at
    /// the contact alone.
    pub fn meets_again(&self) -> Option<f64> {
        let held = |v: Iv| self.contact.again(Jet::at(v)).v.bounds().is_some_and(|[_,hi]| hi < 0.);
        pieces(self.span,64).find_map(|w| proved(&held,w,8))
    }
    /// The contour's jet at `s` in its frame's coordinates, in floats or in intervals (what a
    /// certified reading of a line against the contour's cylinder reads, `query::line_slid`).
    pub fn jet<T: Num>(&self,s: Jet<T>) -> J3<T> { self.local(s) }
    /// Newton on `(C − p) · C'` from `s`, held to the span.
    pub fn newton(&self,p: V,mut s: f64) -> f64 {
        let [a,b] = self.span;
        for _ in 0..30 {
            let (x,d,dd) = self.d2(s);
            let e = sub(x,p);
            let h = dot(d,d)+dot(e,dd);
            if !(h > 0.) { break }
            let next = (s-dot(e,d)/h).clamp(a.min(b),a.max(b));
            if !next.is_finite() { break }
            if (next-s).abs() <= 1e-15*(1.+s.abs()) { s = next; break }
            s = next;
        }
        s
    }
    /// The parameter of its point nearest `p`: the nearest of samples across the span, then
    /// `newton`.
    pub fn inverse(&self,p: V) -> f64 {
        let [a,b] = self.span;
        let off = |s: f64| { let l = self.local(Jet::<f64>::of(s)).map(|j| j.v); let q = sub(self.frame.at(l),p); dot(q,q) };
        let s = (0..=64).map(|k| a+(b-a)*k as f64/64.).map(|s| (off(s),s)).min_by(|x,y| x.0.total_cmp(&y.0)).map_or(a,|x| x.1);
        self.newton(p,s)
    }
    /// A box about it over `[t0, t1]`, never smaller than it: its point held in intervals over each
    /// of sixteen pieces, each box's corners placed.
    pub fn bounds(&self,[t0,t1]: [f64;2]) -> Box3 {
        let mut out = crate::geom::EMPTY;
        for w in pieces([t0,t1],16) {
            let l = self.local(Jet::at(Iv::new(w[0],w[1])));
            match (l[0].v.bounds(),l[1].v.bounds(),l[2].v.bounds()) {
                (Some(x),Some(y),Some(z)) => for c in 0..8 {
                    crate::geom::include(&mut out,self.frame.at([x[c&1],y[(c>>1)&1],z[c>>2]]))
                },
                // (the contact's readings never leave their domain: `1 + s²` and `D_z² + g²` are
                // never nought; its point's own box, read in floats, all the same)
                _ => for s in w { crate::geom::include(&mut out,self.d2(s).0) },
            }
        }
        out
    }
    /// The most length per unit of `s`, read at 65 points of the span.
    pub fn speed(&self) -> f64 {
        let [a,b] = self.span;
        (0..=64).map(|k| norm(self.d2(a+(b-a)*k as f64/64.).1)).fold(0.,f64::max)
    }
    /// The same curve carried by `m`, rigid or a reflection: its frame carried (a frame carried by
    /// a reflection is left-handed, and its points, read off it, the reflection's).
    pub fn moved(&self,m: &Rigid) -> Slid { Slid {frame:self.frame.moved(m),..*self} }
    /// The same curve's posed contact: the contact itself where the torus about `centre` (on the
    /// cylinder's line through the frame's origin along the slide) touches the contour's cylinder.
    pub fn posed(&self,centre: V) -> Slid { Slid {frame:Frame {o:centre,..self.frame},contour:false,..*self} }
}

/// **The two halves of a torus's contact under a slide, on one side** (`side`), as curves: the torus
/// about `frame`'s `z`, its tube's middle radius `big` and radius `small`, the slide along the unit
/// `d` (in space); each half `s ∈ [−1, 1]` about the frame's `x` and about its `−x`, read as its
/// apparent contour where `contour`, else as the contact itself.
pub fn halves(frame: Frame,big: f64,small: f64,d: V,side: f64,contour: bool) -> [Slid;2] {
    [1.,-1.].map(|k| {
        let f = Frame {x:crate::util::space::scale(frame.x,k),y:crate::util::space::scale(frame.y,k),..frame};
        Slid {frame:f,contact:Contact {big,small,d:f.dir_local(d),side},span:[-1.,1.],contour}
    })
}
