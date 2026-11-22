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
use crate::util::space::distance;

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
            let i = s.partition_point(|&(t,_)| t < t0);
            if s.get(i).is_none_or(|&(t,_)| t != t0) { s.insert(i,(t0,self.signed(x,t0).0)) }
        }
        let n = s.len();
        let mut out = Vec::new();
        for i in 0..n {
            let d = s[i].1;
            let below = i > 0 && s[i-1].1 < d;
            let above = i+1 < n && s[i+1].1 < d;
            if below || above || (i > 0 && s[i-1].1 == d) { continue }
            if Some(s[i].0) == own { out.push(self.cover_at(x,s[i].0)); continue }
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
        self.minima_beside(x,Some(t0)).into_iter().filter(|c| c.t != t0).min_by(|a,b| a.depth.total_cmp(&b.depth))
    }

    /// The point in space the tool puts its own point `at` at the instant `t`.
    pub fn placed(&self,at: V,t: f64) -> V { self.path.pose(t).map_or([f64::NAN;3],|p| p.point(at)) }

    /// How far the sample is from the settled least: a check on `minima`.
    pub fn apart(&self,a: &Cover,x: V) -> f64 { distance(self.placed(a.at,a.t),x) }
}
