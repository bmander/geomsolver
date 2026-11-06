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
use crate::geom::{Frame,Rigid,V};
use crate::turned::{Exact,Iv,J3,Jet,Num,dot3,pieces};
use crate::util::space::scale;

/// **A torus's contact under a slide, on one side** (the module's): the tube's middle radius `big`
/// and its radius `small`, the slide's unit direction `d` in the torus's frame (its `z` the torus's
/// axis), and which half of the tube (`side`: `+1` the outer, where `n · ρ̂ > 0`, `−1` the inner).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Contact { pub big: f64,pub small: f64,pub d: V,pub side: f64 }

impl Contact {
    /// The contact's point at the jet `s = tan(u/2)` of the angle round the axis from the frame's `x`.
    fn point<T: Num>(&self,s: Jet<T>) -> J3<T> {
        let (one,s2) = (Jet::of(1.),s*s);
        let w = one/(one+s2);
        let (c,sn) = ((one-s2)*w,s.times(2.)*w);
        let [dx,dy,dz] = self.d;
        let g = c.times(dx)+sn.times(dy);
        let q = one/(g*g+Jet::of(dz*dz)).sqrt();
        // n = σ (D_z ρ̂ − g z)/|·|, σ the side's sign against D_z's
        let sigma = self.side*dz.signum();
        let reach = Jet::of(self.big)+q.times(sigma*self.small*dz);
        [reach*c,reach*sn,(g*q).times(-sigma*self.small)]
    }
    /// The contact's apparent contour at `s`: its point less its part along the slide.
    fn contour<T: Num>(&self,s: Jet<T>) -> J3<T> {
        let x = self.point(s);
        let along = dot3(x,self.d.map(Jet::of));
        std::array::from_fn(|i| x[i]-along.times(self.d[i]))
    }
    /// How far past meeting the torus again a line along the slide through the contact at `s`
    /// stands: the line `p + t D` meets the torus where `(|p + tD|² + R² − r²)² = 4R² |(p + tD)ₓᵧ|²`,
    /// a quartic with a double root at the contact (`t = 0`), the rest `t² + a t + b` whose
    /// discriminant is `8 (2 R² |Dₓᵧ|² − |p|² − R² + r²)`. Below nought it meets the torus there
    /// alone; the value is that discriminant over eight.
    fn again<T: Num>(&self,s: Jet<T>) -> Jet<T> {
        let x = self.point(s);
        let (big,small) = (self.big,self.small);
        let flat = self.d[0]*self.d[0]+self.d[1]*self.d[1];
        Jet::of(2.*big*big*flat-big*big+small*small)-dot3(x,x)
    }
}

/// **A torus's contact under a slide, as a curve** (`geom::Curve::Slid`): one half of the contact
/// (`s ∈ span`, within `[−1, 1]` about the frame's `x`), read in `frame` — the torus's own frame,
/// its origin the torus's centre and its `z` its axis, its `x` the half's middle — either the
/// contact itself (`contour` false: where the torus at a pose of the slide touches the cylinder the
/// contact sweeps, `ssi`'s `slid_meeting`) or its apparent contour, in the plane through the frame's
/// origin square to the slide (`contour`: the cylinder's section, `Surface::Extrusion` of it). Its
/// point, box, inverse and speed are read off its jets (`turned::Exact`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Slid { pub frame: Frame,pub contact: Contact,pub span: [f64;2],pub contour: bool }

impl Exact for Slid {
    fn frame(&self) -> &Frame { &self.frame }
    fn span(&self) -> [f64;2] { self.span }
    fn local<T: Num>(&self,s: Jet<T>) -> J3<T> {
        if self.contour { self.contact.contour(s) } else { self.contact.point(s) }
    }
}

/// Where `held` is not proved over `span`: read over each of 64 pieces whole and, where it does not
/// hold, over its halves, to eight halvings — the middle of the first stretch it is not proved over,
/// or none.
fn unproved(held: &dyn Fn(Iv) -> bool,span: [f64;2]) -> Option<f64> {
    fn within(held: &dyn Fn(Iv) -> bool,[a,b]: [f64;2],depth: usize) -> Option<f64> {
        if held(Iv::new(a,b)) { return None }
        let m = 0.5*(a+b);
        if depth == 0 { return Some(m) }
        within(held,[a,m],depth-1).or_else(|| within(held,[m,b],depth-1))
    }
    pieces(span,64).find_map(|w| within(held,w,8))
}

impl Slid {
    /// The slide's unit direction, in space.
    pub fn slide(&self) -> V { self.frame.dir(self.contact.d) }
    /// **Where its contour folds** (the fold locus, docs/swept-solid-design.md step 6): the first
    /// stretch of `span` over which the contour's tangent is not proved off nought, its derivative
    /// held in outward intervals (`unproved`), as the parameter there; none where it is regular all
    /// along.
    pub fn folded(&self) -> Option<f64> {
        unproved(&|v| {
            let p = self.contact.contour(Jet::at(v));
            (p[0].d*p[0].d+p[1].d*p[1].d+p[2].d*p[2].d).bounds().is_some_and(|[lo,_]| lo > 0.)
        },self.span)
    }
    /// **Where a line along the slide through its contact meets the torus again** (`Contact::again`):
    /// the first stretch of `span` over which that is not proved nowhere, the discriminant held
    /// below nought in outward intervals (`unproved`), as the parameter there; none where each such
    /// line touches the torus at the contact alone.
    pub fn meets_again(&self) -> Option<f64> {
        unproved(&|v| self.contact.again(Jet::at(v)).v.bounds().is_some_and(|[_,hi]| hi < 0.),self.span)
    }
    /// The same curve carried by `m`, rigid or a reflection: its frame carried (a frame carried by
    /// a reflection is left-handed, and its points, read off it, the reflection's).
    pub fn moved(&self,m: &Rigid) -> Slid { Slid {frame:self.frame.moved(m),..*self} }
    /// The same curve standing about `centre` (on the cylinder's line through the frame's origin
    /// along the slide), read as its contour or as its contact (`contour`): the contour carried
    /// along the slide, or the contact where the torus about `centre` touches the contour's cylinder.
    pub fn at(&self,centre: V,contour: bool) -> Slid { Slid {frame:Frame {o:centre,..self.frame},contour,..*self} }
}

/// **The two halves of a torus's contact under a slide, on one side** (`side`), as curves: the torus
/// about `frame`'s `z`, its tube's middle radius `big` and radius `small`, the slide along the unit
/// `d` (in space); each half `s ∈ [−1, 1]` about the frame's `x` and about its `−x`, read as its
/// apparent contour where `contour`, else as the contact itself.
pub fn halves(frame: Frame,big: f64,small: f64,d: V,side: f64,contour: bool) -> [Slid;2] {
    [1.,-1.].map(|k| {
        let f = Frame {x:scale(frame.x,k),y:scale(frame.y,k),..frame};
        Slid {frame:f,contact:Contact {big,small,d:f.dir_local(d),side},span:[-1.,1.],contour}
    })
}
