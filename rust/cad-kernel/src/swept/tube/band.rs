//! **The band about a near miss of a whole-touch** (#880 T1f): where an instant all but touches a
//! meridian whole — the arc's terms `(A, B)` small without vanishing — the spine's normal turns half
//! round the tube across a line of `(θ, t)` within a hair of it: the sheets are surfaces still, but
//! no net laid out in `(θ, t)` follows them (the crossed reverse member's, `|(A, B)|` 3e-4 of the
//! terms' scale at its least, graded to 944 × 849 poles). Its own chart follows them.
//!
//! **The line.** `(A, B)` folds over `(θ, t)` there: along one way (`e_y`) it hardly moves, across
//! it (`e_x`) it passes by nought at the distance its least is, so the normal's turn about the least
//! is `atan(x / w)` across, `w` the least over the rate across — the turn's width. Along the line the
//! least grows and the turn widens (a valley, not a point: the reverse's 5.4e-4 wide at its least,
//! 1e-2 at 0.2 along, 3e-2 at 0.4), and the line's floor bends across by a little.
//!
//! **The chart.** `(θ, t) = c + (mid(η) + w sinh τ) e_x + η e_y`: `η` along the line, `mid` the
//! floor's bend, and `τ` across, the sinh spreading the turn — `atan(w sinh τ / W)` turns at most
//! one in `τ` for a turn `W` at least `w` wide — so a sheet read over `(τ, η)` is smooth on the
//! scale of the chart (the reverse's both sheets, over ±0.4 along, fitted quintic to a tenth of
//! the tolerance with 129 × 129 poles). Its `(τ, η)` turns as `(θ, t)` does: the map's Jacobian is
//! `w cosh τ` times `e_x × e_y`, one.
//!
//! **The seam** the sheets would end on and the band begin, nearer a box than an ellipse in
//! `(τ, η)` (`SEAM_POWER`): along the line to where the floor's turn is `END_TURN` of the roll wide,
//! across `BAND_ACROSS` times that turn — the chart reaching past it by `BAND_ROOM` of `τ` and
//! `ROOM_ALONG` of its length. **In place, not yet built into the road** (`faces::solid` reads no
//! band yet): the seam as the sheets' hole, the band's traces and its faces are to come, as the
//! collar's came (`Pinch`).
//!
//! (The peer kernel cadrs, MIT OR Apache-2.0, `native/sweep_tube.rs`'s `Band`, charts a near pinch
//! so — `w sinh τ` across a line, its seam a rounded box. Its idea is taken, none of its code: here
//! the width is the floor's own, read where the line is followed, and the bend a Chebyshev fit.)

use super::{Point,Tube};
use super::super::closed::Unbuilt;
use crate::util::fmath::{Det,HALF_READ,THIRD_READ};
use crate::util::space::{add,scale};

/// A near miss is banded where its turn is narrower than this part of the roll (structural: a
/// sheet's own net follows a turn a hundredth of the roll wide in a few dozen knots).
const NEAR_TURN: f64 = 1e-2;
/// The seam leaves the line where the floor's turn is this part of the roll wide (structural, as
/// `NEAR_TURN`: what the sheets beyond follow).
const END_TURN: f64 = 1e-2;
/// The seam stands this many times the turn's width at its ends across the line: the sheet beside it
/// bends as sharply as the tube over how far it stands from the line, so the farther the seam, the
/// fewer knots the sheets' nets carry across their windows (the reverse's, measured: at twice 776 ×
/// 907 poles, at four times 476 × 429, at eight 340 × 283, at sixteen 351 × 217).
const BAND_ACROSS: f64 = 8.;
/// The chart reaches this much of `τ` past its seam.
const BAND_ROOM: f64 = 0.5;
/// The chart reaches this part of its seam's length along the line past each end.
const ROOM_ALONG: f64 = 0.125;
/// The seam's shape: `|x/X|^p + |η/η_s|^p = 1` across and along the line, a box with its corners
/// rounded.
const SEAM_POWER: f64 = 4.;
/// How far inside the seam, as a part of its measure, a sheet is read past it (`Band::deep`).
const RIM_LAYER: f64 = 0.125;
/// The degree of the floor's bend, in Chebyshev polynomials of `η` over the chart.
const BEND_DEGREE: usize = 8;
/// Steps of the line's floor followed, at most (structural).
const MOST_STEPS: usize = 8192;
/// Golden sections a least is settled by (to the rounding of its bracket).
const SECTIONS: usize = 90;

/// **A near miss's band and its chart** (the module's).
#[derive(Clone,Debug)]
pub struct Band {
    /// The least of `|(A, B)|` over the terms' scale, `(θ, t)`, and the least.
    pub at: [f64;2],
    pub least: f64,
    /// Across the line and along it, in `(θ, t)`.
    pub ex: [f64;2],
    pub ey: [f64;2],
    /// The turn's width at the least.
    pub w: f64,
    /// The seam's reach across the line in `(θ, t)`, `X`, and in `τ` (`asinh(X / w)`), and its ends along
    /// the line in `η`.
    pub across: f64,
    pub tau: f64,
    pub eta: [f64;2],
    /// The floor of the stand-in's `h` (`Tube::net_point`): the least of `|(A, B)|` over the scale
    /// where the seam leaves the line, so the stand-in turns no faster than the sheets beyond it.
    pub delta: f64,
    /// The chart's `η` over which the bend is read: its middle and half its length.
    span: [f64;2],
    /// The floor's bend across, Chebyshev coefficients in `(η − span[0]) / span[1]`.
    bend: Vec<f64>,
}

/// `sinh` and `cosh` by `Det::dexp` (the same bits on every target), and `asinh` by `Det::dln`.
fn sinh(x: f64) -> f64 { let e = x.dexp(); 0.5*(e-1./e) }
fn cosh(x: f64) -> f64 { let e = x.dexp(); 0.5*(e+1./e) }
fn asinh(y: f64) -> f64 { let a = y.abs(); (a+(a*a+1.).sqrt()).dln().copysign(y) }

/// The Chebyshev polynomials `T_0 … T_n` at `s`, and their derivatives in `s`.
fn chebyshev(s: f64,n: usize) -> (Vec<f64>,Vec<f64>) {
    let (mut t,mut d) = (vec![1.,s],vec![0.,1.]);
    for k in 2..=n {
        t.push(2.*s*t[k-1]-t[k-2]);
        d.push(2.*t[k-1]+2.*s*d[k-1]-d[k-2]);
    }
    t.truncate(n+1);
    d.truncate(n+1);
    (t,d)
}

/// The least of `f` over `[lo, hi]` by golden sections (`f` unimodal there): where, and the least.
fn golden(f: &dyn Fn(f64) -> f64,mut lo: f64,mut hi: f64) -> (f64,f64) {
    const R: f64 = 0.381_966_011_250_105_1;
    let (mut x1,mut x2) = (lo+R*(hi-lo),hi-R*(hi-lo));
    let (mut f1,mut f2) = (f(x1),f(x2));
    for _ in 0..SECTIONS {
        if f1 <= f2 { hi = x2; x2 = x1; f2 = f1; x1 = lo+R*(hi-lo); f1 = f(x1) }
        else { lo = x1; x1 = x2; f1 = f2; x2 = hi-R*(hi-lo); f2 = f(x2) }
    }
    if f1 <= f2 { (x1,f1) } else { (x2,f2) }
}

impl Band {
    /// The floor's bend across at `η`, and its rate.
    pub fn mid(&self,eta: f64) -> (f64,f64) {
        // (`chebyshev`'s recurrence summed as it runs: the same terms in the same order, no list)
        let s = (eta-self.span[0])/self.span[1];
        let (mut t,mut d) = ([1.,s],[0.,1.]);
        let (mut v,mut r) = (0.,0.);
        for (k,c) in self.bend.iter().enumerate() {
            let (tk,dk) = match k {
                0|1 => (t[k],d[k]),
                _ => {
                    let (tk,dk) = (2.*s*t[1]-t[0],2.*t[1]+2.*s*d[1]-d[0]);
                    (t,d) = ([t[1],tk],[d[1],dk]);
                    (tk,dk)
                }
            };
            v += c*tk;
            r += c*dk;
        }
        (v,r/self.span[1])
    }

    /// **The band's `(τ, η)` at the place `(θ, t)` when its seam holds it**: none outside — at once
    /// where `η` lies past the seam's ends, which its measure cannot be below one beyond.
    pub fn holding(&self,uv: [f64;2]) -> Option<[f64;2]> {
        let th = self.at[0]+(uv[0]-self.at[0]+std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)-std::f64::consts::PI;
        let eta = (th-self.at[0])*self.ey[0]+(uv[1]-self.at[1])*self.ey[1];
        if !(eta > self.eta[0] && eta < self.eta[1]) { return None }
        let q = self.chart_of(uv);
        (self.seam(q) < 0.).then_some(q)
    }

    /// **The place `(θ, t)` of the chart's `(τ, η)`**.
    pub fn place(&self,[tau,eta]: [f64;2]) -> [f64;2] {
        let x = self.mid(eta).0+self.w*sinh(tau);
        [self.at[0]+x*self.ex[0]+eta*self.ey[0],self.at[1]+x*self.ex[1]+eta*self.ey[1]]
    }

    /// The place's rates by `τ` and by `η`, in `(θ, t)`.
    pub fn rates(&self,[tau,eta]: [f64;2]) -> [[f64;2];2] {
        let (c,m) = (self.w*cosh(tau),self.mid(eta).1);
        [[c*self.ex[0],c*self.ex[1]],[m*self.ex[0]+self.ey[0],m*self.ex[1]+self.ey[1]]]
    }

    /// **The chart's `(τ, η)` at the place `(θ, t)`**, `θ` read within half a turn of the band's
    /// least: `η` along the line, then `τ` from what lies across past the floor there.
    pub fn chart_of(&self,[th,t]: [f64;2]) -> [f64;2] {
        let th = self.at[0]+(th-self.at[0]+std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)-std::f64::consts::PI;
        let d = [th-self.at[0],t-self.at[1]];
        let eta = d[0]*self.ey[0]+d[1]*self.ey[1];
        let x = d[0]*self.ex[0]+d[1]*self.ex[1];
        [asinh((x-self.mid(eta).0)/self.w),eta]
    }

    /// The chart's rectangle: past the seam by `BAND_ROOM` of `τ` and `ROOM_ALONG` of its length.
    pub fn rect(&self) -> crate::zero_set::Rect {
        let room = ROOM_ALONG*(self.eta[1]-self.eta[0]);
        [[-self.tau-BAND_ROOM,self.tau+BAND_ROOM],[self.eta[0]-room,self.eta[1]+room]]
    }

    /// The seam's measure at `(τ, η)`: `S = |x/X|^p + |(η − m)/h|^p`, `x = w sinh τ` how far across the
    /// line, `m` and `h` the middle and half of the seam along it — below one inside.
    fn measure(&self,[tau,eta]: [f64;2]) -> f64 {
        let (m,h) = (0.5*(self.eta[0]+self.eta[1]),0.5*(self.eta[1]-self.eta[0]));
        (self.w*sinh(tau)/self.across).abs().dpowf(SEAM_POWER)+((eta-m)/h).abs().dpowf(SEAM_POWER)
    }

    /// **Which side of the seam `(τ, η)` lies**: positive outside (the sheets'), negative inside (the
    /// band's), nought on it — `S^(1/p) − 1`. A box with its corners rounded in how far across the line
    /// and along it, not in `τ`: so the seam stands its reach off the line all along but at its two
    /// ends, where the turn is as wide as the sheets beyond follow (a box in `(τ, η)` is a spindle in
    /// `(θ, t)`, hugging the line where the turn is still narrow, and the stand-in's blend falls from
    /// one to nought within a sliver of the seam).
    pub fn seam(&self,q: [f64;2]) -> f64 { self.measure(q).dpowf(1./SEAM_POWER)-1. }
}

impl Band {
    /// **Whether `(τ, η)` lies deep in the band**: farther inside the seam than `RIM_LAYER` of its
    /// measure (`seam` under −`RIM_LAYER`), where a sheet is not read (`Tube::side`) — the pinch's rule
    /// (`Pinch::deep`), a sheet read on past the rim only as far as a tracer's exit asks; but a thin
    /// layer, where the pinch's is half its hole: inside the seam the line runs on toward its ends with
    /// its turn still narrow, and the depth's search halved its cells over it without end.
    pub fn deep(&self,q: [f64;2]) -> bool { !(self.seam(q) >= -RIM_LAYER) }

    /// **The seam's point at the fraction `f` of a turn round it**, in `(τ, η)`: the rounded box in
    /// polar form, `r(φ) (cos φ, sin φ)` with `r = (|cos φ|^p + |sin φ|^p)^(−1/p)`, across by `X` and
    /// along by `h` about the middle `m`, `τ` its `asinh` — smooth in `f` all round for an even `p`
    /// (a box written as `(c^(2/p), s^(2/p))` has the slope of a square root wherever a cosine or a
    /// sine passes nought, and its arcs were fitted to 1e-3 only there).
    pub fn seam_at(&self,f: f64) -> [f64;2] {
        let (s,c) = (std::f64::consts::TAU*f).dsin_cos();
        let (m,h) = (0.5*(self.eta[0]+self.eta[1]),0.5*(self.eta[1]-self.eta[0]));
        let r = (c.abs().dpowf(SEAM_POWER)+s.abs().dpowf(SEAM_POWER)).dpowf(-1./SEAM_POWER);
        [asinh(self.across*r*c/self.w),m+h*r*s]
    }

    /// **Whether `(τ, η)` lies beyond the band**: outside its seam past the half digits of its measure
    /// (a point a sheet's trace left on the seam reads on either side of it by its rounding, and is
    /// the band's too), where the band's chart is not read (`Tube::side`): the sheets'. Its chart ends at its seam as the collar's at its rim — read a
    /// layer past it, the search met the sheets' own seams running on into that layer, which the
    /// traces, stopped at the seam, never reached, and halved its cells beside them to 2⁻¹⁷ without
    /// end (an edge of the layer's own as an inner edge stopped nothing the seam had not).
    pub fn beyond(&self,q: [f64;2]) -> bool { !(self.seam(q) <= HALF_READ) }

    /// **The fraction of a turn round the seam at `(τ, η)`** (`seam_at`'s inverse, read through any
    /// point by its way from the seam's middle): the polar angle of how far across and along, each
    /// over its reach, in `[0, 1)`.
    pub fn fraction(&self,[tau,eta]: [f64;2]) -> f64 {
        let (m,h) = (0.5*(self.eta[0]+self.eta[1]),0.5*(self.eta[1]-self.eta[0]));
        (((eta-m)/h).datan2(self.w*sinh(tau)/self.across)/std::f64::consts::TAU).rem_euclid(1.)
    }

    /// **The seam's `η` at `τ`, the way `dir` along the line** (±1): where the rounded box stands at
    /// that `τ`, `m ± h (1 − |x/X|^p)^(1/p)`; the middle where `τ` reaches past it.
    pub fn seam_eta(&self,tau: f64,dir: f64) -> f64 {
        let (m,h) = (0.5*(self.eta[0]+self.eta[1]),0.5*(self.eta[1]-self.eta[0]));
        let r = (1.-(self.w*sinh(tau)/self.across).abs().dpowf(SEAM_POWER)).max(0.).dpowf(1./SEAM_POWER);
        m+dir*h*r
    }

    /// **The seam as the band's own inner edge**: `n` chords of it in `(τ, η)`, counter-clockwise.
    pub fn seam_line(&self,n: usize) -> Vec<[f64;2]> { (0..=n).map(|i| self.seam_at(i as f64/n as f64)).collect() }

    /// **The seam as a sheet's inner edge**: `n` chords of it, each place `(θ, t)` (`θ` unwrapped
    /// along it), counter-clockwise in `(τ, η)` as in `(θ, t)`.
    pub fn rim(&self,n: usize) -> Vec<[f64;2]> {
        let mut line: Vec<[f64;2]> = (0..=n).map(|i| self.place(self.seam_at(i as f64/n as f64))).collect();
        for j in 1..line.len() {
            line[j][0] = line[j-1][0]+(line[j][0]-line[j-1][0]+std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)-std::f64::consts::PI
        }
        line
    }
}

impl Band {
    /// **The stand-in's blend at `(τ, η)`** (`Tube::net_point`): `e^(1 − 1/(1 − S))` inside the seam,
    /// nought outside, `S` the seam's measure — one on the line at its middle, flat to every order on
    /// the seam, and smooth through the line: `S` is a polynomial in how far across and along, where
    /// the seam's radius `S^(1/p)` has a cone point at the middle (a blend of the radius, the pinch's
    /// rule `β(ρ/ρ₀)`, kinked the stand-in along the very line the band is for).
    pub fn blend(&self,q: [f64;2]) -> f64 {
        let s = self.measure(q);
        if s < 1. { (1.-1./(1.-s)).dexp() } else { 0. }
    }
}

impl Tube<'_> {
    /// `(A, B)` of the tube's arc at `(θ, t)` over the terms' scale.
    fn terms_at(&self,th: f64,t: f64) -> Option<[f64;2]> {
        let (v,size) = self.ring.ab(self.path,0,th,t)?;
        Some([v[0]/size,v[1]/size])
    }

    /// `|(A, B)|` over the scale at `(θ, t)`, infinite where it is not read.
    fn least_at(&self,th: f64,t: f64) -> f64 { self.terms_at(th,t).map_or(f64::INFINITY,|v| v[0].dhypot(v[1])) }

    /// The way across the line at `(θ, t)`: the way `(A, B)` moves fastest (the larger right
    /// singular vector of its Jacobian, by differences a cube root of the rounding apart), its `t`
    /// forward; and the rate across.
    fn across(&self,th: f64,t: f64) -> Option<([f64;2],f64)> {
        let (ht,hs) = (THIRD_READ,THIRD_READ*(self.roll[1]-self.roll[0]));
        let (p,m) = (self.terms_at(th+ht,t)?,self.terms_at(th-ht,t)?);
        let (q,n) = (self.terms_at(th,t+hs)?,self.terms_at(th,t-hs)?);
        let d = [[(p[0]-m[0])/(2.*ht),(q[0]-n[0])/(2.*hs)],[(p[1]-m[1])/(2.*ht),(q[1]-n[1])/(2.*hs)]];
        let (a,b,c) = (d[0][0]*d[0][0]+d[1][0]*d[1][0],d[0][0]*d[0][1]+d[1][0]*d[1][1],d[0][1]*d[0][1]+d[1][1]*d[1][1]);
        let l1 = 0.5*(a+c)+(0.25*(a-c)*(a-c)+b*b).sqrt();
        let v = if b != 0. { [l1-c,b] } else if a >= c { [1.,0.] } else { [0.,1.] };
        let n = v[0].dhypot(v[1]);
        if !(n > 0. && l1 > 0.) { return None }
        let v = if v[1] < 0. || (v[1] == 0. && v[0] < 0.) { [-v[0]/n,-v[1]/n] } else { [v[0]/n,v[1]/n] };
        Some((v,l1.sqrt()))
    }

    /// **The near misses of a whole-touch over the roll, each charted by its band** (the module's):
    /// along each of the two angles `A` vanishes at, followed by continuity at `SHAPE_LOOKS` rolls as
    /// `Ring::touched_once` follows them, each least of `|B|` that keeps its sign; settled as the
    /// least of `|(A, B)|` in `(θ, t)` — the floor across the line, then along it, the line read again
    /// at each — and banded where its turn is narrower than `NEAR_TURN` of the roll; its floor
    /// followed along the line each way to where the turn is `END_TURN` of the roll wide, and on past
    /// that by `ROOM_ALONG`. Refused by name where the floor falls again along the line (another
    /// least, or the pinch's), where the chart would leave the roll or hold the pinch, or where the
    /// bend does not follow the floor.
    pub fn bands(&self) -> Result<Vec<Band>,Unbuilt> {
        let [a,b] = self.roll;
        let failed = |why: String| Unbuilt::Failed(format!("the tube road's band: {why}"));
        let looks = super::super::rings::SHAPE_LOOKS;
        let at = |i: usize| if i == looks { b } else { a+(b-a)*i as f64/looks as f64 };
        let read = |t: f64| self.ring.a_vanishes(self.path,0,t).ok_or_else(|| failed(format!("no terms at {t}")));
        let way = |x: f64,y: f64| { let d = (x-y).rem_euclid(std::f64::consts::TAU); d.min(std::f64::consts::TAU-d) };
        // (each branch's angle and `B` at every look)
        let mut misses: Vec<[f64;2]> = Vec::new();
        for j in 0..2 {
            let mut row: Vec<(f64,f64,f64)> = Vec::with_capacity(looks+1);
            let mut theta = read(a)?[j].0;
            for i in 0..=looks {
                let r = read(at(i))?;
                let here = if way(r[0].0,theta) <= way(r[1].0,theta) { r[0] } else { r[1] };
                theta = here.0;
                row.push((at(i),here.0,here.1));
            }
            for i in 1..looks {
                let (p,q,r) = (row[i-1].2,row[i].2,row[i+1].2);
                if q.abs() < p.abs() && q.abs() <= r.abs() && (p < 0.) == (r < 0.) && (q < 0.) == (p < 0.) { misses.push([row[i].1,row[i].0]) }
            }
        }
        let step = (b-a)/looks as f64;
        let mut out = Vec::new();
        for guess in misses {
            // settled: the floor across, then the least along, the line read again at each
            let mut c = guess;
            let mut reach = 64.*step;
            for _ in 0..4 {
                let Some((ex,_)) = self.across(c[0],c[1]) else { break };
                let ey = [-ex[1],ex[0]];
                let floor = |eta: f64| golden(&|x: f64| self.least_at(c[0]+x*ex[0]+eta*ey[0],c[1]+x*ex[1]+eta*ey[1]),-reach,reach);
                let (eta,_) = golden(&|eta: f64| floor(eta).1,-reach,reach);
                let (x,_) = floor(eta);
                c = [c[0]+x*ex[0]+eta*ey[0],c[1]+x*ex[1]+eta*ey[1]];
                reach *= 0.125;
            }
            let least = self.least_at(c[0],c[1]);
            let Some((ex,rate)) = self.across(c[0],c[1]) else { continue };
            let w = least/rate;
            if !(w < NEAR_TURN*(b-a)) || !(least > 0.) { continue }
            let ey = [-ex[1],ex[0]];
            // the floor followed along the line each way: (η, across, the turn's width)
            let mut floor: Vec<(f64,f64,f64,f64)> = vec![(0.,0.,w,least)];
            let mut ends = [0.;2];
            for (e,dir) in [-1.,1.].into_iter().enumerate() {
                let (mut eta,mut x,mut turn,mut low) = (0.,0.,w,least);
                let mut end: Option<f64> = None;
                let mut steps = 0;
                loop {
                    steps += 1;
                    if steps > MOST_STEPS { return Err(failed(format!("the line through {c:?} followed past {MOST_STEPS} steps"))) }
                    eta += dir*step;
                    let place = |x: f64| [c[0]+x*ex[0]+eta*ey[0],c[1]+x*ex[1]+eta*ey[1]];
                    let r = 8.*turn+8.*step;
                    let (nx,l) = golden(&|y: f64| { let p = place(y); self.least_at(p[0],p[1]) },x-r,x+r);
                    let p = place(nx);
                    if !(p[1] > a+step && p[1] < b-step) { return Err(failed(format!("the line through {c:?} reaches the roll's end at {p:?}"))) }
                    if l < low*(1.-1e-9) { return Err(failed(format!("the floor through {c:?} falls again at {p:?} ({l:e} under {low:e})"))) }
                    let Some((_,rate)) = self.across(p[0],p[1]) else { return Err(failed(format!("no rate across at {p:?}"))) };
                    (x,turn,low) = (nx,l/rate,l);
                    floor.push((eta,x,turn,l));
                    if end.is_none() && turn >= END_TURN*(b-a) { end = Some(eta) }
                    if let Some(z) = end { if eta.abs() >= z.abs()*(1.+2.*ROOM_ALONG) { break } }
                }
                ends[e] = end.expect("an end");
            }
            floor.sort_by(|p,q| p.0.total_cmp(&q.0));
            let nearest = |eta: f64| *floor.iter().min_by(|p,q| (p.0-eta).abs().total_cmp(&(q.0-eta).abs())).expect("a floor");
            let across = BAND_ACROSS*nearest(ends[0]).2.max(nearest(ends[1]).2);
            let tau = asinh(across/w);
            let delta = nearest(ends[0]).3.max(nearest(ends[1]).3);
            // the bend: a least-squares Chebyshev fit of the floor over what was followed
            let (lo,hi) = (floor[0].0,floor[floor.len()-1].0);
            let span = [0.5*(lo+hi),0.5*(hi-lo)];
            let mut m = crate::util::linalg::Mat::zeros(floor.len(),BEND_DEGREE+1);
            for (i,&(eta,..)) in floor.iter().enumerate() {
                let (t,_) = chebyshev((eta-span[0])/span[1],BEND_DEGREE);
                for (k,v) in t.into_iter().enumerate() { m.data[i*(BEND_DEGREE+1)+k] = v }
            }
            let rhs: Vec<f64> = floor.iter().map(|p| p.1).collect();
            let (bend,_) = crate::util::linalg::min_norm_solve(&m,&rhs,1e-14);
            let band = Band {at:c,least,ex,ey,w,across,tau,eta:ends,delta,span,bend};
            if let Some((eta,x,turn,_)) = floor.iter().copied().find(|&(eta,x,turn,_)| (band.mid(eta).0-x).abs() > 0.05*turn) {
                return Err(failed(format!("the bend through {c:?} misses the floor at η {eta} by {:e}, its turn {turn:e}",band.mid(eta).0-x)))
            }
            // the chart inside the roll, clear of the pinch
            let [[t0,t1],[e0,e1]] = band.rect();
            for i in 0..=64 {
                let f = i as f64/64.;
                for q in [[t0+(t1-t0)*f,e0],[t0+(t1-t0)*f,e1],[t0,e0+(e1-e0)*f],[t1,e0+(e1-e0)*f]] {
                    let p = band.place(q);
                    if !(p[1] > a && p[1] < b) { return Err(failed(format!("the band through {c:?} leaves the roll at {p:?}"))) }
                }
            }
            if let Some(p) = &self.pinch {
                let q = band.chart_of([p.collar.at.1,p.collar.at.0]);
                if q[0].abs() <= t1 && q[1] >= e0 && q[1] <= e1 {
                    return Err(failed(format!("the band through {c:?} holds the pinch at {:?}",p.collar.at)))
                }
            }
            out.push(band);
        }
        Ok(out)
    }

    /// **A sheet built round its near misses** (`Tube::pinched`, then `bands`): the sheets' hole is
    /// each band's seam too (`Tube::hole`), deep inside it no sheet is read (`Tube::side`), and its
    /// stand-in fills it for the nets (`Tube::net_point`). Not yet the road's: `faces::solid` builds
    /// no band's faces, and `closed::ring` calls `Tube::pinched`.
    pub fn banded<'a>(ring: &'a super::super::rings::Ring,path: &'a crate::trajectory::Trajectory,roll: [f64;2],tol: f64,feature: f64)
        -> Result<Tube<'a>,Unbuilt> {
        let mut tube = Tube::pinched(ring,path,roll,tol,feature)?;
        tube.bands = tube.bands()?;
        Ok(tube)
    }

    /// **Where a chart's point `q` stands on a band's seam**: the band and the fraction round its seam
    /// there — on a band's chart where its seam reads nought, on a sheet where a band's does, each to
    /// the half digits of the seam's measure (where a trace's exit settled it); none elsewhere.
    pub fn on_seam(&self,chart: super::Chart,q: [f64;2]) -> Option<(usize,f64)> {
        match chart {
            super::Chart::Band(k,_) => (self.bands[k].seam(q).abs() <= HALF_READ).then(|| (k,self.bands[k].fraction(q))),
            super::Chart::Sheet(_) => self.bands.iter().enumerate().find_map(|(k,b)| {
                let r = b.chart_of(q);
                (b.seam(r).abs() <= HALF_READ).then(|| (k,b.fraction(r)))
            }),
            _ => None,
        }
    }

    /// The band whose seam holds the place `(θ, t)`, and its `(τ, η)` there.
    pub fn band_holding(&self,uv: [f64;2]) -> Option<(&Band,[f64;2])> {
        self.bands.iter().find_map(|b| Some((b,b.holding(uv)?)))
    }

    /// **The sheet on `side` read on a band's chart at `(τ, η)`**: the sheet's point at the place, its
    /// rates by the chain rule through the place's (`Band::rates`), its normal and area factor the
    /// sheet's. None where the sheet is not read there.
    pub fn band_at(&self,band: &Band,side: f64,q: [f64;2]) -> Option<Point> {
        let [th,t] = band.place(q);
        let p = self.point(side,th,t)?;
        let [dt,de] = band.rates(q);
        Some(Point {x_th:add(scale(p.x_th,dt[0]),scale(p.x_t,dt[1])),x_t:add(scale(p.x_th,de[0]),scale(p.x_t,de[1])),..p})
    }

    /// Which side of a band's chart can bound the sweep: the sheet's `J` at the place, the map
    /// turning as `(θ, t)` does.
    pub fn band_side(&self,band: &Band,side: f64,q: [f64;2]) -> Option<f64> {
        let [th,t] = band.place(q);
        self.jacobian(side,th,t)
    }
}
