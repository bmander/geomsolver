//! **How deep another instant covers a point of a candidate sheet** (#942, the depth-based trim):
//! the swept solid is the union of the tool's poses over the roll, so a point of a candidate — a
//! tube's sheet, the tool at an end pose — is on its boundary exactly where no pose holds it. On a
//! sheet the pose of the point's own instant touches it (the depth there nought by construction),
//! and the instants beside it hold it or not by the sign of `s''` (the fold, which the ring road
//! reads in closed form); so what is read here is the least depth over the instants *outside* the
//! point's own contact — the stretch of the roll about its own instant over which its distance from
//! the tool climbs away from nought on each side — and the instant and the place of the tool where
//! that least depth is taken. Where that least is below nought another instant covers the point;
//! the curve where it passes nought is a trim curve, and the covering instant names the partner
//! face and its point there, so both faces' parameters of the curve come by construction.
//!
//! (The peer kernel cadrs, MIT OR Apache-2.0, `crates/cadrs_kernel/src/native/sweep_tube.rs`,
//! trims its sheets by the same reading; nothing of its code is ported here.)

use super::closed::Unbuilt;
use super::rings::{Label,Ring};
use crate::trajectory::{Trajectory,Twist};
use crate::util::fmath::HALF_READ;

type V = [f64;3];

/// **What covers a point beside its own contact**: the least depth of the tool at an instant
/// outside the point's own (`signed`, below nought inside), that instant, the point in the tool's
/// coordinates there and the piece of the meridian (the tool's face) nearest it.
#[derive(Clone,Copy,Debug)]
pub struct Cover { pub depth: f64,pub at: f64,pub point: V,pub piece: usize }

/// **How a point of a sheet stands under the sweep's other instants.**
#[derive(Clone,Copy,Debug)]
pub enum Reading {
    /// The instants beside its own hold it (the far side of a fold, or a pose at a roll's end
    /// moving into it): covered with no other instant needed.
    Near,
    /// Its own contact climbs to each end of the roll: no other instant to cover it.
    Alone,
    /// The least depth outside its own contact, and where.
    Far(Cover),
}

/// The tool's signed depth at the point `x` (in space) at the roll `t`, and the point in the tool's
/// coordinates and its nearest piece.
fn depth_at(ring: &Ring,path: &Trajectory,x: V,t: f64) -> Option<(f64,V,usize)> {
    let p = path.pose(t).ok()?.inverse().point(x);
    let (d,k) = ring.signed(p);
    Some((d,p,k))
}

/// The golden-section least of `f` over `[lo, hi]`, to the rounding of the roll.
fn least(f: &dyn Fn(f64) -> f64,mut lo: f64,mut hi: f64) -> f64 {
    let g = 0.5*(5f64.sqrt()-1.);
    loop {
        let (x1,x2) = (hi-g*(hi-lo),lo+g*(hi-lo));
        if !(x1 > lo && x2 < hi && x1 < x2) { break }
        if f(x1) < f(x2) { hi = x2 } else { lo = x1 }
    }
    0.5*(lo+hi)
}

/// **How the point `x` of a sheet, its own instant `own`, stands under the sweep over `[a, b]`**
/// (`Reading`), its depth read to `within` (the tolerance: a point held less deep than that is on
/// the boundary as the tolerance reads it). Its distance from the tool `d(τ)` is read along the roll
/// each way from `own`, the steps growing from the rounding's share of the roll by a quarter each,
/// and never longer than the stretch over which the point moves the larger of `|d|` and `within`
/// (`d` 1-Lipschitz in space: so no stretch held more than `within` deep is stepped over — a
/// certified march) nor a quarter of the ring's thickness (a feature size); the own contact runs
/// on each side while `d` climbs, its end (the greatest) settled by golden sections, and the least
/// of `d` past both ends settled the same way about the least sample.
pub fn cover(ring: &Ring,path: &Trajectory,[a,b]: [f64;2],x: V,own: f64,within: f64) -> Result<Reading,Unbuilt> {
    let domain = crate::util::interval::Interval::new(a,b).map_err(|_| Unbuilt::Failed("a roll with no extent".into()))?;
    let speed = path.inverse_point_speed_bound(x,domain).map_err(|_| Unbuilt::Failed("a point's speed under the motion unbounded".into()))?;
    let most = if speed > 0. { 0.25*ring.thickness/speed } else { b-a };
    let first = HALF_READ*(b-a);
    let d = |t: f64| depth_at(ring,path,x,t).map_or(f64::INFINITY,|r| r.0);
    // each side's samples: the roll and the depth
    let side = |dir: f64| -> Vec<(f64,f64)> {
        let end = if dir > 0. { b } else { a };
        let mut out = Vec::new();
        let (mut s,mut step) = (own,first);
        while (end-s)*dir > 0. {
            s = if (end-s-dir*step)*dir <= 0. { end } else { s+dir*step };
            let v = d(s);
            out.push((s,v));
            step = (1.25*step).min(most).min(v.abs().max(within)/speed);
        }
        out
    };
    let (lower,upper) = (side(-1.),side(1.));
    // (a side whose first depth clear of its rounding — the pose's and the distance's few roundings
    // of the point's place, read at the tool's size — is below nought: its neighbours cover the
    // point. Nearer, `d` is nought to its rounding either way)
    let rounding = 16.*f64::EPSILON*(crate::util::space::norm(x)+ring.size);
    let held = |s: &[(f64,f64)]| s.iter().find(|p| p.1.abs() > rounding).is_some_and(|p| p.1 < 0.);
    if held(&lower) || held(&upper) { return Ok(Reading::Near) }
    // each side's own contact's end: the first sample past which `d` falls
    let past = |s: &[(f64,f64)]| -> (usize,Option<f64>) {
        // (a fall within the rounding is no fall: about its own instant `d` is nought to it)
        let Some(k) = (1..s.len()).find(|&k| s[k-1].1 > rounding && s[k].1 < s[k-1].1-rounding) else { return (s.len(),None) };
        let lo = if k >= 2 { s[k-2].0 } else { own };
        let (l,h) = if lo < s[k].0 { (lo,s[k].0) } else { (s[k].0,lo) };
        (k,Some(least(&|t| -d(t),l,h)))
    };
    let ((kl,el),(ku,eu)) = (past(&lower),past(&upper));
    // the least sample past each end, settled between its neighbours and within the far stretch
    let mut best: Option<(f64,f64)> = None;
    for (s,k,e) in [(&lower,kl,el),(&upper,ku,eu)] {
        let Some(e) = e else { continue };
        let Some(i) = (k..s.len()).min_by(|&i,&j| s[i].1.total_cmp(&s[j].1)) else { continue };
        let ends = |j: usize| if j < k { e } else { s[j].0 };
        let (p,q) = (ends(i.saturating_sub(1).max(k-1)),if i+1 < s.len() { s[i+1].0 } else { s[i].0 });
        let (l,h) = if p < q { (p,q) } else { (q,p) };
        let t = if h > l { least(&d,l,h) } else { s[i].0 };
        let v = d(t).min(s[i].1);
        let t = if d(t) <= s[i].1 { t } else { s[i].0 };
        if best.is_none_or(|b| v < b.1) { best = Some((t,v)) }
    }
    let Some((t,_)) = best else { return Ok(Reading::Alone) };
    let Some((depth,point,piece)) = depth_at(ring,path,x,t) else { return Err(Unbuilt::Failed("a pose not read".into())) };
    Ok(Reading::Far(Cover {depth,at:t,point,piece}))
}

/// **The point of a ring's sheet at `(θ, t)` on the contact labelled `l`**, in space, and how its
/// sheet bends there (`s''`, the fold's sign): `None` where the contact has no such root.
pub fn sheet(ring: &Ring,path: &Trajectory,whole: [f64;2],theta: f64,t: f64,l: Label) -> Option<(V,f64)> {
    let (w,dw) = super::rings::rates(path,t,whole).ok()?;
    let th = theta.rem_euclid(std::f64::consts::TAU);
    let r = ring.root(&w,th,l,false)?;
    Some((path.pose(t).ok()?.point(r.x),ring.bend(&w,&dw,th,&r)))
}

/// The label of the contact at `(θ, t)` nearest the place `σ` round the meridian.
pub fn label_at(ring: &Ring,path: &Trajectory,theta: f64,t: f64,sigma: f64) -> Option<Label> {
    let w = Twist::body(path.at(t).ok()?);
    let nf = ring.elements.len() as f64;
    let apart = |x: f64| { let d = (x-sigma).rem_euclid(nf); d.min(nf-d) };
    ring.roots(&w,theta.rem_euclid(std::f64::consts::TAU)).into_iter().min_by(|a,b| apart(a.1.sigma).total_cmp(&apart(b.1.sigma))).map(|r| r.0)
}

/// The far cover's depth at the sheet's point `(θ, t)` on `l` (`cover`): below nought covered by
/// another instant; `None` where the sheet has no point there, the neighbours hold it, or nothing
/// else can.
pub fn far_depth(ring: &Ring,path: &Trajectory,roll: [f64;2],theta: f64,t: f64,l: Label,within: f64) -> Option<Cover> {
    let (x,_) = sheet(ring,path,roll,theta,t,l)?;
    match cover(ring,path,roll,x,t,within).ok()? { Reading::Far(c) => Some(c),_ => None }
}

/// The tool's depth at the point `x` at each of `ts` (a probe of `cover`).
pub fn depths(ring: &Ring,path: &Trajectory,x: V,ts: &[f64]) -> Vec<f64> {
    ts.iter().map(|&t| depth_at(ring,path,x,t).map_or(f64::NAN,|r| r.0)).collect()
}
