//! **The tube's own certified cover** (#880 T1f): how a point stands under the sweep's other
//! instants, read with the torus's depth in closed form and a bound on its second rate, where
//! `depth::cover` (the ring road's, for any meridian) has only the point's speed.
//!
//! **The bound.** At the roll `τ` the point `x` stands at `q(τ) = M(τ)⁻¹ x` in the tool's frame, and
//! the tool's depth there is `d(τ) = g(q(τ)) − r`, `g` the distance from the core circle. So
//! `d' = ∇g · q'` and `d'' = ∇g · q'' + q'ᵀ H q'`, `H` the Hessian of `g`. In the ring's cylindrical
//! frame `(ρ, φ, z)` `g` is the distance in the meridian half-plane from the circle's point there,
//! whose Hessian has the eigenvalues `0` and `1/g`, and `H`'s `φφ` entry is `g_ρ/ρ`, at most `1/ρ` —
//! no other entry — so `|H| ≤ max(1/g, 1/ρ)`, and with `|∇g| = 1`
//!
//! > `|d''| ≤ A + V² max(1/g, 1/ρ)`,
//!
//! `V` and `A` the motion's bounds on `q`'s speed and acceleration over the roll
//! (`Trajectory::inverse_point_speed_bound`, `…_acceleration_bound`), `g` and `ρ` their least over
//! the stretch read — each 1-Lipschitz in `q`, so within `V` a unit of the roll of their value at
//! its read. Where either may vanish over the stretch (the point beside the core circle or its
//! axis) the bound is the first-order one alone: `d` is `V`-Lipschitz in the roll.
//!
//! **The march and the search.** From the point's own instant each way its own contact climbs: it
//! is marched by steps over which the bound keeps `d` above the greatest read so far less `within`
//! (no dip deeper than that is stepped over), until `d` falls a rounding below that greatest (its
//! top, settled by golden sections) — or, first, its first depth clear of the rounding is below
//! nought (`Near`), or it climbs to the roll's end. Past each top the least of `d` is found
//! best-first over halved stretches, each stretch's floor `d(m) − |d'(m)| w − M w²/2` (its middle
//! `m`, half width `w`) or the Lipschitz one, until every floor stands within `within` of the least
//! read — then settled by golden sections about it. Each step at a basin's bottom is of order
//! `√(2 within / M)`, where the first-order march's was `within / V`.

use super::Tube;
use crate::swept::depth::{Cover,Reading};
use crate::util::fmath::{Det,HALF_READ};
use crate::util::space::{dot,norm};

type V = [f64;3];

/// The tool's depth at the point at one instant, its rate, and the distances its bound reads.
#[derive(Clone,Copy,Debug)]
struct Read { t: f64,d: f64,rate: f64,g: f64,rho: f64,speed: f64 }

/// What a cover of one point reads once: its place, its speed and acceleration bounds under the
/// motion over the roll.
struct Point<'a> { tube: &'a Tube<'a>,x: V,speed: f64,accel: f64 }

impl Point<'_> {
    /// The depth at `t` and its rate in closed form, with `g` and `ρ`.
    fn read(&self,t: f64) -> Option<Read> {
        let tb = self.tube;
        let m = tb.path.at(t).ok()?.inverse();
        let (q,v) = (m.point(self.x),m.velocity(self.x));
        let f = &tb.ring.frame;
        let l = f.local(q);
        let w = [dot(v,f.x),dot(v,f.y),dot(v,f.z)];
        let rho = l[0].dhypot(l[1]);
        if !(rho > 0.) { return None }
        let dv = [l[0]-tb.radius*l[0]/rho,l[1]-tb.radius*l[1]/rho,l[2]-tb.height];
        let g = norm(dv);
        if !(g > 0.) { return None }
        Some(Read {t,d:g-tb.r,rate:dot(dv,w)/g,g,rho,speed:norm(v)})
    }

    /// The bound on `|d''|` over the stretch within `w` of the read `r` (the module's); infinite
    /// where `g` or `ρ` may vanish over it.
    fn curvature(&self,r: &Read,w: f64) -> f64 {
        let v = self.speed.min(r.speed+self.accel*w);
        let (g,rho) = (r.g-v*w,r.rho-v*w);
        if !(g > 0. && rho > 0.) { return f64::INFINITY }
        self.accel+v*v*(1./g).max(1./rho)
    }

    /// The floor of `d` over the stretch within `w` of the read `r`: the second-order bound's or the
    /// Lipschitz one, the higher.
    fn floor(&self,r: &Read,w: f64) -> f64 {
        let lipschitz = r.d-self.speed*w;
        let m = self.curvature(r,w);
        if m.is_finite() { lipschitz.max(r.d-r.rate.abs()*w-0.5*m*w*w) } else { lipschitz }
    }

    /// The longest step from `r` in the way `dir`, at most `most`, over which `d` keeps above `level`
    /// (`r.d` at or above it): `d(t + s) ≥ d + k s − M s²/2` (`k` the rate along the way) is least at
    /// an end of the step, so the step is the quadratic's root; or the Lipschitz step, the longer.
    /// `M` is read over a stretch the step may take: from `most`, halved toward twice the step
    /// while that lengthens it (a stretch nearer its read bounds `g` and `ρ` higher).
    fn step(&self,r: &Read,dir: f64,level: f64,most: f64) -> f64 {
        let room = (r.d-level).max(0.);
        let lipschitz = if self.speed > 0. { room/self.speed } else { most };
        let k = dir*r.rate;
        let second = |over: f64| {
            let m = self.curvature(r,over);
            if !m.is_finite() { return 0. }
            let s = if m > 0. { (k+(k*k+2.*m*room).sqrt())/m } else if k < 0. { room/-k } else { over };
            s.min(over)
        };
        // (from `most`, halved while `M` is unbounded over it or the step falls short of half of
        // it, at most to twice the step — each stretch's step kept within it, so each bound holds;
        // a pass a halving of the stretch, as many as half the digits of the roll, structural)
        let (mut over,mut s) = (most,0f64);
        for _ in 0..f64::MANTISSA_DIGITS/2 {
            let t = second(over);
            s = s.max(t);
            let next = if t > 0. { (2.*t).max(0.5*over) } else { 0.5*over };
            if !(next < over) { break }
            over = next;
        }
        lipschitz.max(s).min(most)
    }

    /// **The least of `d` about the read `r`, within `w` of it inside `[lo, hi]`**: the rate's root
    /// beside it bracketed and settled by the secant with the Illinois rule (the rate in closed
    /// form), to the rounding of the roll `first`; else the least of the reads.
    fn settle(&self,r: Read,w: f64,[lo,hi]: [f64;2],first: f64) -> Option<Read> {
        if !(w > 0.) { return Some(r) }
        let (l,h) = (self.read((r.t-w).max(lo))?,self.read((r.t+w).min(hi))?);
        let least = |a: Read,b: Read| if b.d < a.d { b } else { a };
        let (mut a,mut b) = if r.rate > 0. { (l,r) } else { (r,h) };
        if !(a.rate < 0. && b.rate > 0.) { return Some(least(least(l,r),h)) }
        let (mut fa,mut fb) = (a.rate,b.rate);
        let mut side = 0;
        while b.t-a.t > first {
            let t = a.t-fa*(b.t-a.t)/(fb-fa);
            let t = if t > a.t && t < b.t { t } else { 0.5*(a.t+b.t) };
            let c = self.read(t)?;
            if c.rate == 0. { return Some(least(least(a,b),c)) }
            if c.rate < 0. { a = c; fa = c.rate; if side == -1 { fb *= 0.5 } side = -1 } else { b = c; fb = c.rate; if side == 1 { fa *= 0.5 } side = 1 }
        }
        Some(least(least(a,b),r))
    }
}

/// A stretch of the roll the search holds: its ends, its middle's read, and its floor.
struct Span { lo: f64,hi: f64,mid: Read,floor: f64 }

impl Tube<'_> {
    /// **How the point `x`, its own instant `own`, stands under the sweep** (`Reading`), its depth
    /// read to `within` (the module's march and search). None where the motion gives no bound on
    /// the point's speed or acceleration, or where a pose is not read.
    pub(super) fn certified(&self,x: V,own: f64,within: f64) -> Option<Reading> {
        let [a,b] = self.roll;
        let domain = crate::util::interval::Interval::new(a,b).ok()?;
        let speed = self.path.inverse_point_speed_bound(x,domain).ok()?;
        let accel = self.path.inverse_point_acceleration_bound(x,domain).ok()?;
        let p = Point {tube:self,x,speed,accel};
        // (the rounding's share of the roll, the first step; the whole roll the longest — the
        // bound alone keeps the march: no feature size enters)
        let most = b-a;
        let first = HALF_READ*(b-a);
        // (the pose's and the distance's few roundings of the point's place, read at the tool's size)
        let rounding = 16.*f64::EPSILON*(norm(x)+self.ring.size);
        // each way: the own contact marched to its top, or `None` where the neighbours hold it;
        // the top's instant and depth where it falls, none where it climbs to the roll's end
        let climb = |dir: f64| -> Option<Option<f64>> {
            let end = if dir > 0. { b } else { a };
            let (mut s,mut step) = (own,first);
            let mut clear = false;
            // (the greatest read so far)
            let mut top: Option<Read> = None;
            while (end-s)*dir > 0. {
                s = if (end-s-dir*step)*dir <= 0. { end } else { s+dir*step };
                let r = p.read(s)?;
                if !clear && r.d.abs() > rounding {
                    if r.d < 0. { return None }
                    clear = true;
                }
                match top {
                    // (a fall within the rounding is no fall: about its own instant `d` is nought
                    // to it — `depth::cover`'s rule. The top is its greatest read: between the reads
                    // `d` keeps within `within` of it, so the search past it may begin there)
                    Some(t) if t.d > rounding && r.d < t.d-rounding => return Some(Some(t.t)),
                    Some(t) if r.d <= t.d => {}
                    _ => top = Some(r),
                }
                let level = top.map_or(r.d,|t| t.d)-within;
                step = if clear { p.step(&r,dir,level,most) } else { (1.25*step).min(p.step(&r,dir,level,most)) };
            }
            Some(None)
        };
        let lower = climb(-1.)?;
        let upper = climb(1.)?;
        // past each top, the least of `d` best-first
        let stretches: Vec<[f64;2]> = [lower.map(|t| [a,t]),upper.map(|t| [t,b])].into_iter().flatten().collect();
        if stretches.is_empty() { return Some(Reading::Alone) }
        let mut best: Option<(Read,f64)> = None;
        let mut spans: Vec<Span> = Vec::new();
        let note = |r: Read,w: f64,best: &mut Option<(Read,f64)>| if best.is_none_or(|(q,_)| r.d < q.d) { *best = Some((r,w)) };
        for &[lo,hi] in &stretches {
            for t in [lo,hi] { note(p.read(t)?,0.,&mut best) }
            if hi > lo {
                let mid = p.read(0.5*(lo+hi))?;
                note(mid,0.5*(hi-lo),&mut best);
                spans.push(Span {lo,hi,mid,floor:p.floor(&mid,0.5*(hi-lo))});
            }
        }
        loop {
            let level = best.map_or(f64::INFINITY,|(q,_)| q.d)-within;
            let Some(k) = (0..spans.len()).filter(|&k| spans[k].floor < level).min_by(|&i,&j| spans[i].floor.total_cmp(&spans[j].floor)) else { break };
            let Span {lo,hi,mid,..} = spans.swap_remove(k);
            // (a stretch the rounding of the roll wide is read whole by its middle)
            if hi-lo <= first { continue }
            for (l,h) in [(lo,mid.t),(mid.t,hi)] {
                let r = p.read(0.5*(l+h))?;
                note(r,0.5*(h-l),&mut best);
                spans.push(Span {lo:l,hi:h,mid:r,floor:p.floor(&r,0.5*(h-l))});
            }
        }
        let (r,w) = best?;
        // settled about the least read, within the stretch it was read the middle of
        let s = *stretches.iter().find(|s| r.t >= s[0] && r.t <= s[1])?;
        let t = p.settle(r,w,s,first)?.t;
        let pose = self.path.pose(t).ok()?;
        let point = pose.inverse().point(x);
        let (depth,_) = self.depth_at(x,t)?;
        Some(Reading::Far(Cover {depth,at:t,point,piece:0}))
    }
}
