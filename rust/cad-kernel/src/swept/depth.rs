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
/// (`Reading`), the point standing off its sheet by as much as `off`, its depth read to `within` (the tolerance: a point held less deep than that is on
/// the boundary as the tolerance reads it). Its distance from the tool `d(τ)` is read along the roll
/// each way from `own`, the steps growing from the rounding's share of the roll by a quarter each,
/// and never longer than the stretch over which the point moves the larger of `|d|` and `within`
/// (`d` 1-Lipschitz in space: so no stretch held more than `within` deep is stepped over — a
/// certified march) nor a quarter of the ring's thickness (a feature size); the own contact runs
/// on each side while `d` climbs, its end (the greatest) settled by golden sections, and the least
/// of `d` past both ends settled the same way about the least sample.
pub fn cover(ring: &Ring,path: &Trajectory,[a,b]: [f64;2],x: V,own: f64,off: f64,within: f64) -> Result<Reading,Unbuilt> {
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
    // (or, where the point stands off its sheet — a fitted sheet's point, `off` its fit — that:
    // about its own instant `d` is then nought to it)
    let rounding = (16.*f64::EPSILON*(crate::util::space::norm(x)+ring.size)).max(off);
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
    match cover(ring,path,roll,x,t,0.,within).ok()? { Reading::Far(c) => Some(c),_ => None }
}

/// The tool's depth at the point `x` at each of `ts` (a probe of `cover`).
pub fn depths(ring: &Ring,path: &Trajectory,x: V,ts: &[f64]) -> Vec<f64> {
    ts.iter().map(|&t| depth_at(ring,path,x,t).map_or(f64::NAN,|r| r.0)).collect()
}

/// **The far cover as a field over a ring's sheet** (#942, the depth-based trim): a sheet's points
/// by `(θ, t)` on the contact `label`, each read by `cover` (`within` the tolerance) — the value the
/// far cover's depth, what it was read at the `Cover` (the covering instant and the tool's point
/// there: the partner). Where the sheet has no point, folds away (`s''` at or below nought: the far
/// side of a fold, which its neighbours hold), or nothing else covers it (`Alone`), the field is not
/// defined: `zero_set` reads that as outside its domain. Its gradient is the envelope theorem's
/// (`gradient`).
pub struct Covered<'a> { pub ring: &'a Ring,pub path: &'a Trajectory,pub roll: [f64;2],pub label: Label,pub within: f64 }

impl Covered<'_> {
    /// The sheet's point at `(θ, t)`, where it touches.
    fn point(&self,[th,t]: [f64;2]) -> Option<V> {
        let (x,bend) = sheet(self.ring,self.path,self.roll,th,t,self.label)?;
        (bend > 0.).then_some(x)
    }

    /// The covering instant's preimage on the ring's sheet: the angle of the tool's point round the
    /// ring's axis, the instant, and the place round the meridian of the root there nearest it.
    /// `None` at an end of the roll (the pose there is the tool's own face, met by the Boolean) or
    /// where no root stands within the tolerance of the point.
    pub fn partner(&self,c: &Cover) -> Option<(f64,f64,f64)> {
        let [a,b] = self.roll;
        if !(c.at > a && c.at < b) { return None }
        let l = self.ring.frame.local(c.point);
        let th = l[1].atan2(l[0]).rem_euclid(std::f64::consts::TAU);
        let w = Twist::body(self.path.at(c.at).ok()?);
        self.ring.roots(&w,th).into_iter().map(|(_,r)| (crate::util::space::distance(r.x,c.point),r.sigma))
            .filter(|p| p.0 <= self.within).min_by(|p,q| p.0.total_cmp(&q.0)).map(|p| (th,c.at,p.1))
    }

    /// **The far cover's gradient in `(θ, t)`** where its least is taken at the instant `at`: by
    /// the envelope theorem the tool's signed distance's gradient there (its normal, read in the
    /// tool by central differences a cube root of the rounding of its thickness apart, turned into
    /// space) against the sheet's two rates. Differences of the depth itself, at the half digits
    /// `zero_set` steps by, read the noise of the sheet's root (the half turn's chords accepted
    /// 3.6e-5 off its zero set, eleven tolerances).
    fn gradient(&self,uv: [f64;2],x: V,at: f64) -> Option<[f64;2]> {
        let pose = self.path.pose(at).ok()?;
        let p = pose.inverse().point(x);
        let h = f64::EPSILON.powf(1./3.)*self.ring.thickness;
        let n = [0,1,2].map(|k| {
            let (mut a,mut b) = (p,p);
            a[k] += h;
            b[k] -= h;
            (self.ring.signed(a).0-self.ring.signed(b).0)/(2.*h)
        });
        let n = pose.vector(n);
        let (_,su,sv) = crate::zero_set::Field::surface(self,uv);
        let g = [crate::util::space::dot(n,su),crate::util::space::dot(n,sv)];
        g.iter().all(|v| v.is_finite()).then_some(g)
    }

    /// The place round the meridian of the sheet's own root at `(θ, t)`.
    pub fn sigma(&self,[th,t]: [f64;2]) -> Option<f64> {
        let w = Twist::body(self.path.at(t).ok()?);
        Some(self.ring.root(&w,th.rem_euclid(std::f64::consts::TAU),self.label,false)?.sigma)
    }
}

impl crate::zero_set::Field for Covered<'_> {
    type At = Cover;
    fn least(&self,uv: [f64;2]) -> Option<crate::zero_set::Value<Cover>> {
        let x = self.point(uv)?;
        match cover(self.ring,self.path,self.roll,x,uv[1],0.,self.within).ok()? {
            Reading::Far(c) => Some(crate::zero_set::Value {f:c.depth,grad:self.gradient(uv,x,c.at),at:c}),
            _ => None,
        }
    }

    /// The least of the tool's depth at the point about the covering instant `from` names: over the
    /// stretch of the roll a quarter of the ring's thickness long in the point's motion either side
    /// of it (the feature `cover` steps by), within the roll and short of the point's own instant
    /// by half the way to it (its own contact, nought there, is no cover).
    fn along(&self,uv: [f64;2],from: &Cover) -> Option<crate::zero_set::Value<Cover>> {
        let x = self.point(uv)?;
        let [a,b] = self.roll;
        let domain = crate::util::interval::Interval::new(a,b).ok()?;
        let speed = self.path.inverse_point_speed_bound(x,domain).ok()?;
        let reach = if speed > 0. { 0.25*self.ring.thickness/speed } else { b-a };
        let (mut lo,mut hi) = ((from.at-reach).max(a),(from.at+reach).min(b));
        let own = uv[1];
        if own > lo && own < hi { if from.at > own { lo = 0.5*(own+from.at) } else { hi = 0.5*(own+from.at) } }
        let d = |t: f64| depth_at(self.ring,self.path,x,t).map_or(f64::INFINITY,|r| r.0);
        let t = if hi > lo { least(&d,lo,hi) } else { from.at };
        let (depth,point,piece) = depth_at(self.ring,self.path,x,t)?;
        Some(crate::zero_set::Value {f:depth,grad:self.gradient(uv,x,t),at:Cover {depth,at:t,point,piece}})
    }

    /// The sheet's point and its two rates, by central differences a cube root of the rounding of
    /// an angle and of the roll apart (where the difference's rounding and its truncation balance).
    fn surface(&self,[th,t]: [f64;2]) -> (V,V,V) {
        let h = f64::EPSILON.powf(1./3.);
        let ht = h*(self.roll[1]-self.roll[0]);
        let at = |th: f64,t: f64| sheet(self.ring,self.path,self.roll,th,t,self.label).map_or([f64::NAN;3],|s| s.0);
        let rate = |p: V,q: V,step: f64| crate::util::space::scale(crate::util::space::sub(p,q),0.5/step);
        (at(th,t),rate(at(th+h,t),at(th-h,t),h),rate(at(th,t+ht),at(th,t-ht),ht))
    }
}

/// **A boundary read from the depth, as a seam** (#942): one piece of the curve where another
/// instant's cover of a sheet begins — its points on the sheet `(θ, t, σ)`, how its two ends end
/// (`zero_set::End`), and its partner: the covering instant's preimages `(θ', τ, σ')`, run by run
/// where the covering instant lies inside the roll (where it is an end of the roll the partner is
/// the tool's own face posed there, which the Boolean meets).
#[derive(Clone,Debug)]
pub struct Seam { pub sheet: Vec<(f64,f64,f64)>,pub ends: [crate::zero_set::End;2],pub partner: Vec<Vec<(f64,f64,f64)>> }

impl Seam {
    /// Its curves as the ring road lays a double curve's (`rings::cut_along`): the sheet's, then
    /// each run of its partner's.
    pub fn curves(&self) -> Vec<Vec<(f64,f64,f64)>> {
        std::iter::once(self.sheet.clone()).chain(self.partner.iter().cloned()).filter(|c| c.len() >= 2).collect()
    }
}

/// **Where another instant's cover of a ring's sheet begins, traced as seams** (#942, the
/// depth-based trim): the zero set of the far cover's depth (`Covered`) over the box `rect` of the
/// sheet's `(θ, t)` on the contact `label`, by `zero_set::trace` to `within` (the tolerance) in space
/// with `feature` its longest step and the spacing it samples at, from `seeds` (events the caller
/// knows: a swallowtail point, a double curve's crossing of a stretch's end) — each piece with the
/// covering instant's preimages, so both faces' parameters come by construction. A cell the search
/// for closed curves left unresolved is refused by name, never dropped.
#[allow(clippy::too_many_arguments)]
pub fn seams(ring: &Ring,path: &Trajectory,roll: [f64;2],label: Label,rect: crate::zero_set::Rect,seeds: &[[f64;2]],within: f64,feature: f64)
    -> Result<Vec<Seam>,Unbuilt> {
    let field = Covered {ring,path,roll,label,within};
    let q = crate::zero_set::Question {rect,edges:&[],seeds,tol:within,feature};
    let traced = crate::zero_set::trace(&field,&q).map_err(|why| Unbuilt::Failed(format!("the cover's boundary on a sheet: {why}")))?;
    if let Some(cell) = traced.search.unresolved.first() {
        return Err(Unbuilt::Failed(format!("the cover's boundary on a sheet: the cell {cell:?} of its (θ, t) not resolved")))
    }
    let mut out = Vec::new();
    for p in &traced.pieces {
        let (uvs,ats) = refined(&field,&p.uv,&p.at)?;
        let mut sheet = Vec::with_capacity(uvs.len());
        for &uv in &uvs {
            let Some(sg) = field.sigma(uv) else { return Err(Unbuilt::Failed(format!("the cover's boundary at {uv:?}: no root of the sheet"))) };
            sheet.push((uv[0],uv[1],sg));
        }
        let mut partner: Vec<Vec<(f64,f64,f64)>> = vec![Vec::new()];
        for c in &ats {
            match field.partner(c) {
                Some(q) => partner.last_mut().unwrap().push(q),
                None => if !partner.last().unwrap().is_empty() { partner.push(Vec::new()) },
            }
        }
        partner.retain(|r| r.len() >= 2);
        // (a piece traced again from a seed on one already traced: its ends those of the earlier
        // within twice the tolerance — each within it of the one end — and every point of it within
        // the feature of the earlier's points, the spacing below which the trace tells nothing
        // apart. Two traces of one curve beside a swallowtail point stand some 1e-5 apart between
        // their points: along the kernel of the sheet's rates a chord's sagitta is not its depth's)
        let on = |q: (f64,f64,f64)| field.point([q.0,q.1]);
        let close = |a: (f64,f64,f64),b: (f64,f64,f64),r: f64| on(a).zip(on(b)).is_some_and(|(x,y)| crate::util::space::distance(x,y) <= r);
        let near = |a: &Seam| {
            let (n,m) = (sheet.len(),a.sheet.len());
            let ends = (close(sheet[0],a.sheet[0],2.*within) && close(sheet[n-1],a.sheet[m-1],2.*within))
                || (close(sheet[0],a.sheet[m-1],2.*within) && close(sheet[n-1],a.sheet[0],2.*within));
            ends && sheet.iter().all(|&q| a.sheet.iter().any(|&r| close(q,r,feature)))
        };
        if out.iter().any(near) { continue }
        out.push(Seam {sheet,ends:p.ends,partner});
    }
    Ok(out)
}

/// Newton's steps a point is pulled onto the zero set by: one started within a chord's sagitta
/// converges quadratically in three or four, one that has not in sixteen is not converging
/// (structural, as `zero_set`'s corrector).
const PULLS: usize = 16;

/// **A traced piece held to the tolerance in space** (#942): where a sheet's two rates are nearly
/// one way (beside a swallowtail point, along the kernel of its rates) a first-order distance to
/// the zero set through the inverse metric reads a point far off it as near — the half turn's
/// double curve 0.05 from its swallowtail point, its chords accepted 3.6e-5 off. So each chord's
/// middle in the parameters is pulled onto the zero set (Newton along the branch's gradient in the
/// parameters) and, where that moves it more than the tolerance in space, put in and both halves
/// read again; a middle that does not settle is refused by name.
fn refined(field: &Covered,uv: &[[f64;2]],at: &[Cover]) -> Result<(Vec<[f64;2]>,Vec<Cover>),Unbuilt> {
    use crate::zero_set::Field;
    let pulled = |m: [f64;2],from: &Cover| -> Option<([f64;2],Cover)> {
        let mut m = m;
        let mut c = *from;
        for _ in 0..PULLS {
            let v = field.along(m,&c)?;
            c = v.at;
            let g = v.grad?;
            let gg = g[0]*g[0]+g[1]*g[1];
            if !(gg > 0.) { return None }
            let step = [v.f*g[0]/gg,v.f*g[1]/gg];
            m = [m[0]-step[0],m[1]-step[1]];
            if step[0].abs() <= f64::EPSILON*m[0].abs().max(1.) && step[1].abs() <= f64::EPSILON*m[1].abs().max(1.) { break }
        }
        let v = field.along(m,&c)?;
        let x = field.point(m)?;
        // (settled where what is left of the depth is within the rounding of the point's place)
        (v.f.abs() <= 16.*f64::EPSILON*(crate::util::space::norm(x)+field.ring.size)).then_some((m,v.at))
    };
    let (mut uvs,mut ats) = (vec![uv[0]],vec![at[0]]);
    for k in 1..uv.len() {
        let mut stack = vec![(uv[k],at[k])];
        while let Some(&(q,cq)) = stack.last() {
            let p = *uvs.last().unwrap();
            let cp = *ats.last().unwrap();
            let m = [0.5*(p[0]+q[0]),0.5*(p[1]+q[1])];
            let Some((z,cz)) = pulled(m,&cp) else {
                return Err(Unbuilt::Failed(format!("the cover's boundary on a sheet: the middle {m:?} of a chord not pulled onto it")))
            };
            let (Some(xm),Some(xz)) = (field.point(m),field.point(z)) else {
                return Err(Unbuilt::Failed(format!("the cover's boundary on a sheet: no point of it at {m:?}")))
            };
            let _ = cq;
            if crate::util::space::distance(xm,xz) <= field.within { uvs.push(q); ats.push(cq); stack.pop(); }
            else if stack.len() > PULLS*4 { return Err(Unbuilt::Failed(format!("the cover's boundary on a sheet: a chord at {m:?} halved \
                {} times and still off it",PULLS*4))) }
            else { stack.push((z,cz)) }
        }
    }
    Ok((uvs,ats))
}

