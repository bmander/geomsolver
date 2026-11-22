//! **How deep a point lies in a ring's sweep, and under which instant** (#942, the depth trim): the
//! sweep is the union of the tool's poses over the roll, so a point `x` lies in it by as much as the
//! least, over the roll, of the tool's signed distance at `x` (`Ring::signed`, its own coordinates
//! `pose(t)⁻¹ x`) — below nought inside. A candidate sheet (a tube's, a meridian's strip, an end
//! pose's face) is on the sweep's boundary exactly where no other instant holds its point: where its
//! depth read over every instant but the point's own is not below nought. The instant that holds a
//! point names the tool's piece there and its point on it, so the face meeting a sheet along the
//! edge of what is kept, and that face's parameters, come with the reading.
//!
//! On an envelope sheet the point's own instant touches it, `s(t₀) = 0`, so the least over the
//! whole roll never changes sign there: what is read is the least over the instants outside the
//! point's own basin (`Depth::cover`). Cover by the instants beside `t₀` begins where `s''(t₀)`
//! changes sign — the fold, which the ring road reads exactly (`Ring::bend`) — and is read here by
//! the own basin's not being a least at all (`s''` below nought: the instants beside it go deeper).
use crate::trajectory::{Pose,Trajectory};
use crate::util::space::{distance,norm};

type V = [f64;3];
use super::closed::Unbuilt;
use super::rings::Ring;

/// **A ring carried over a roll, read for depth.** Its instants sampled at `n + 1` equal steps
/// (the poses' inverses kept), the least over each basin settled from there.
pub struct Depth<'a> { pub ring: &'a Ring,pub path: &'a Trajectory,pub roll: [f64;2],ts: Vec<f64>,inv: Vec<Pose> }

/// **What holds a point**: the instant `t`, the signed distance there (below nought inside), the
/// tool's meridian piece nearest it and the point in the tool's own coordinates.
#[derive(Clone,Copy,Debug)]
pub struct Cover { pub t: f64,pub depth: f64,pub piece: usize,pub at: V }

impl<'a> Depth<'a> {
    /// The ring over `roll`, its instants first read at `n + 1` equal steps.
    pub fn new(ring: &'a Ring,path: &'a Trajectory,roll: [f64;2],n: usize) -> Result<Depth<'a>,Unbuilt> {
        let ts: Vec<f64> = (0..=n).map(|i| roll[0]+(roll[1]-roll[0])*i as f64/n as f64).collect();
        let inv = ts.iter().map(|&t| path.pose(t).map(Pose::inverse)).collect::<Result<Vec<_>,_>>()
            .map_err(|e| Unbuilt::Failed(format!("a pose for depth: {e}")))?;
        Ok(Depth {ring,path,roll,ts,inv})
    }

    /// The tool's signed distance at `x` at the instant `t` (`Ring::signed`), and its piece there.
    pub fn signed(&self,x: V,t: f64) -> (f64,usize) {
        let Ok(p) = self.path.pose(t) else { return (f64::NAN,0) };
        self.ring.signed(p.inverse().point(x))
    }

    fn cover_at(&self,x: V,t: f64) -> Cover {
        let at = self.path.pose(t).map(|p| p.inverse().point(x)).unwrap_or([f64::NAN;3]);
        let (depth,piece) = self.ring.signed(at);
        Cover {t,depth,piece,at}
    }

    /// **Every least of the depth at `x` over the roll**: each basin the samples show (an end of the
    /// roll among them where the depth rises from it) settled by golden section within its two
    /// neighbouring samples, to where the depth no longer reads a difference.
    pub fn minima(&self,x: V) -> Vec<Cover> { self.minima_beside(x,None) }

    /// `minima`, with `own` taken in among the samples: a point's own instant on a sheet, so that its
    /// basin is read as its own (`cover`).
    fn minima_beside(&self,x: V,own: Option<f64>) -> Vec<Cover> {
        let mut s: Vec<(f64,f64)> = self.ts.iter().zip(&self.inv).map(|(&t,p)| (t,self.ring.signed(p.point(x)).0)).collect();
        if let Some(t0) = own {
            // (and samples halving their way in toward it from the steps either side, until the
            // roll no longer reads a difference: a partner basin closing on the own one — the wedge
            // beside a corner, held from an instant 0.11 of its angle on — is parted from it)
            let h = (self.roll[1]-self.roll[0])/(self.ts.len()-1).max(1) as f64;
            let mut near = vec![t0];
            let mut d = h;
            while t0+d != t0 && t0-d != t0 {
                near.extend([t0-d,t0+d]);
                d *= 0.5;
            }
            s.extend(near.into_iter().filter(|&t| t >= self.roll[0] && t <= self.roll[1]).map(|t| (t,self.signed(x,t).0)));
            s.sort_by(|a,b| a.0.total_cmp(&b.0));
            s.dedup_by(|a,b| a.0 == b.0);
        }
        let n = s.len();
        let rounding = 16.*f64::EPSILON*(norm(x)+self.ring.size);
        let mut out = Vec::new();
        for i in 0..n {
            let d = s[i].1;
            let below = i > 0 && s[i-1].1 < d;
            let above = i+1 < n && s[i+1].1 < d;
            if below || above || (i > 0 && s[i-1].1 == d) { continue }
            if Some(s[i].0) == own { out.push(self.cover_at(x,s[i].0)); continue }
            // (beside the own instant, a sample at nought to the rounding is its touch read again)
            if own.is_some() && d.abs() <= rounding { continue }
            let (lo,hi) = (s[i.saturating_sub(1)].0,s[(i+1).min(n-1)].0);
            out.push(self.settled(x,lo,hi,s[i].0));
        }
        out
    }

    /// The least of the depth at `x` over `[lo, hi]`, from the sample `best`, by golden section.
    fn settled(&self,x: V,mut lo: f64,mut hi: f64,best: f64) -> Cover {
        let g = 0.5*(5f64.sqrt()-1.);
        let f = |t: f64| self.signed(x,t).0;
        let mut b = self.cover_at(x,best);
        // (shrunk until the bracket is the roll's rounding: the depth is flat at its least to half the
        // digits, so the bracket, not the values, says when it is done)
        let (mut c,mut d) = (hi-g*(hi-lo),lo+g*(hi-lo));
        let (mut fc,mut fd) = (f(c),f(d));
        while hi-lo > 4.*f64::EPSILON*hi.abs().max(lo.abs()).max(1.) {
            if fc < fd { hi = d; d = c; fd = fc; c = hi-g*(hi-lo); fc = f(c) }
            else { lo = c; c = d; fc = fd; d = lo+g*(hi-lo); fd = f(d) }
        }
        for t in [c,d,0.5*(lo+hi)] { let k = self.cover_at(x,t); if k.depth < b.depth { b = k } }
        b
    }

    /// **What holds `x` from outside its own instant `t₀`**: the deepest least of the depth over the
    /// roll but `t₀`'s own basin — its own where `t₀` is itself a least among the samples (the sheet
    /// touching, not folded under), every basin otherwise. `None` where no other basin is.
    pub fn cover(&self,x: V,t0: f64) -> Option<Cover> {
        // (a least at nought to the rounding of the depth there — the point's coordinates' digits,
        // the pose carrying it back — is the point's own touch read again beside its instant, the
        // depth that flat there; and an instant touching it exactly from beyond keeps it all the same)
        let rounding = 16.*f64::EPSILON*(norm(x)+self.ring.size);
        self.minima_beside(x,Some(t0)).into_iter().filter(|c| c.t != t0 && c.depth.abs() > rounding).min_by(|a,b| a.depth.total_cmp(&b.depth))
    }

    /// The point in space the tool puts its own point `at` at the instant `t`.
    pub fn placed(&self,at: V,t: f64) -> V { self.path.pose(t).map_or([f64::NAN;3],|p| p.point(at)) }

    /// How far the sample is from the settled least: a check on `minima`.
    pub fn apart(&self,a: &Cover,x: V) -> f64 { distance(self.placed(a.at,a.t),x) }
}

/// **A point of a seam**: its parameters on the sheet `(e, s)`, the point, its own instant, and what
/// holds it from beyond — the partner instant, and the tool's point there, which is the partner
/// face's point by construction.
#[derive(Clone,Copy,Debug)]
pub struct SeamPoint { pub e: f64,pub s: f64,pub x: V,pub own: f64,pub cover: Cover }

impl Depth<'_> {
    /// The depth at a sheet's point from beyond its own instant: the deepest other least, `+∞` where
    /// none is.
    fn beyond(&self,(x,own): (V,f64)) -> f64 { self.cover(x,own).map_or(f64::INFINITY,|c| c.depth) }

    /// **Where cover from beyond begins on the line `e` of a sheet** (`sheet(e, s)`, its point and own
    /// instant): between `kept` and `held`, where the depth from beyond is not below nought at the
    /// one and below at the other, halved until the two points are within a quarter of `tol` in space.
    /// `None` where the ends do not read so. The kept end may be held by less than `tol`: a point
    /// held so is on the boundary to the tolerance — the rim beside a whole-touched meridian, where
    /// the tube's buried sliver stands 1e-10 off it, reads so.
    pub fn edge_on(&self,sheet: &dyn Fn(f64,f64) -> (V,f64),e: f64,[mut kept,mut held]: [f64;2],tol: f64) -> Option<SeamPoint> {
        if self.beyond(sheet(e,kept)) < -tol || self.beyond(sheet(e,held)) >= 0. { return None }
        // (each halving parts the stretch in two; it ends where the points are a quarter of the
        // tolerance apart, or where the parameter no longer halves)
        while distance(sheet(e,kept).0,sheet(e,held).0) > 0.25*tol {
            let mid = 0.5*(kept+held);
            if mid == kept || mid == held { break }
            if self.beyond(sheet(e,mid)) < 0. { held = mid } else { kept = mid }
        }
        let (x,own) = sheet(e,kept);
        let cover = self.cover(x,own).unwrap_or(Cover {t:f64::NAN,depth:f64::INFINITY,piece:0,at:[f64::NAN;3]});
        Some(SeamPoint {e,s:kept,x,own,cover})
    }

    /// **A seam traced across a sheet as a graph over `e`** (the depth trim's curve, #942): from `e0`
    /// toward `e1`, at each `e` the edge of what is kept between the ends `bracket(e)` gives
    /// (`edge_on`), stepped as far as the chord between two edges sags within `tol` of the edge
    /// found at its middle, halved where it sags more. It stops where the bracket no longer reads a
    /// change of cover, or where the step can no longer be halved — refused by name only then.
    /// Each point carries its partner instant and the tool's point there (`SeamPoint::cover`).
    pub fn seam_over(&self,sheet: &dyn Fn(f64,f64) -> (V,f64),[e0,e1]: [f64;2],bracket: &dyn Fn(f64) -> [f64;2],tol: f64)
        -> Result<Vec<SeamPoint>,Unbuilt> {
        let at = |e: f64| self.edge_on(sheet,e,bracket(e),tol);
        let Some(first) = at(e0) else { return Ok(Vec::new()) };
        let mut out = vec![first];
        // (the first step a sixteenth of the stretch: structural, the chord test refines it)
        let mut step = (e1-e0)/16.;
        while (e1-out.last().unwrap().e)*(e1-e0) > 0. {
            let last = *out.last().unwrap();
            let e = if (e1-last.e-step)*(e1-e0) < 0. { e1 } else { last.e+step };
            let (Some(p),Some(m)) = (at(e),at(0.5*(last.e+e))) else { break };
            let mid = [0.5*(last.x[0]+p.x[0]),0.5*(last.x[1]+p.x[1]),0.5*(last.x[2]+p.x[2])];
            if distance(mid,m.x) > tol {
                step *= 0.5;
                if last.e+step == last.e { return Err(Unbuilt::Failed("a seam of the depth trim not traced: its step no longer halves".into())) }
                continue
            }
            out.push(m);
            out.push(p);
            step *= 2.;
        }
        Ok(out)
    }
}
