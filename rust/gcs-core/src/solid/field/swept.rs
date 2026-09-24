//! Continuous volume sweeps of explicit one-Lipschitz material fields.
use super::{SpatialField,Error,I,V};
use crate::{interval::minimum::{self,Minimum,Options,Stop},motion::{Family,MotionBounds}};
use std::collections::BTreeMap;

pub type SweepError = minimum::Error<Error>;

/// A sweep's least value at a point, the roll time it is least at, and whether a second contact
/// time reads nearly as low (`SweptField::minimum_at`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct SweptMinimum { pub value: f64,pub time: f64,pub tied: bool }

/// The field min_t source(inverse(motion(t)) x) over a finite closed interval.
/// Source, motion and domain are immutable solved snapshots. Its material is
/// closure({f<0}); the field is one-Lipschitz but need not be signed distance.
/// A point domain represents one fixed pose. Source-solve error is separate.
#[derive(Clone,Debug)]
pub struct SweptField {source:SpatialField,motion:Family,domain:I,
    /// The inverse poses at the first `SIDE_TABLE` dyadic divisions of the roll, which every
    /// `side` query walks through first, filled once.
    poses:std::sync::OnceLock<std::sync::Arc<Vec<Option<crate::motion::Pose>>>>,
    /// The box every pose of the source lies in, as plain numbers, found once: a point outside
    /// it is outside the material (`clear_of`).
    support:std::sync::OnceLock<Option<[[f64;2];3]>>}

/// Source evaluations made by every `side` query so far, for a caller measuring the oracle.
pub static SIDE_EVALUATIONS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Dyadic level at which `side` stops splitting and searches each basin left (see `side`).
const SIDE_BASIN: u32 = 6;

/// Dyadic levels of the roll whose inverse poses `side` keeps: 2¹⁰ + 1 of them.
const SIDE_LEVELS: u32 = 10;
/// Dyadic levels a stretch may be split to: its ends are integers on a grid this fine.
const SIDE_DEPTH: u32 = 40;

impl SweptField {
    pub fn new(source: SpatialField,motion: Family,domain: I) -> Self {
        Self {source,motion,domain,poses:std::sync::OnceLock::new(),support:std::sync::OnceLock::new()}
    }
    pub fn domain(&self) -> I { self.domain }

    /// Enclose every generating pose of the source's finite material support.
    /// The entire roll domain is passed through interval motion evaluation.
    pub fn support_bounds(&self) -> Result<Option<V>,Error> {
        self.source.support_bounds()?.map(|b| self.motion.bounds(self.domain)?.point(b)).transpose()
    }

    /// A lower bound at least `cap` on the sweep's value at `p`, when one is cheap to prove: the
    /// support box's distance, or `side`'s first-order bound — the source is Lipschitz in the roll
    /// by the motion's inverse-point speed bound, so a stretch between two readings is at least
    /// their mean less that bound times half its width — split lowest first until every stretch
    /// is at least `cap`. `None` as soon as a reading falls below `cap`, or after `budget` source
    /// evaluations: then the caller needs the minimum itself. A Boolean asks this of an operand
    /// that can only matter below `cap` (`MaterialField::reading_warm`).
    pub fn at_least(&self,p: [f64;3],cap: f64,budget: u64) -> Option<f64> {
        if let Some(d) = self.clear_of(p) { if d >= cap { return Some(d); } }
        let [a,b] = self.domain.bounds();
        let n = (1u64 << SIDE_DEPTH) as f64;
        let time = |i: u64| a+(b-a)*(i as f64/n);
        let table = self.poses.get_or_init(|| std::sync::Arc::new((0..=1u64 << SIDE_LEVELS)
            .map(|k| self.motion.pose_at(time(k << (SIDE_DEPTH-SIDE_LEVELS))).ok().map(|m| m.inverse())).collect()));
        let evaluations = std::cell::Cell::new(0u64);
        let at = |i: u64| {
            evaluations.set(evaluations.get()+1);
            let step = SIDE_DEPTH-SIDE_LEVELS;
            let pose = if i & ((1u64 << step)-1) == 0 { table[(i >> step) as usize] }
                else { self.motion.pose_at(time(i)).ok().map(|m| m.inverse()) };
            pose.map_or(f64::INFINITY,|m| self.source.value(m.point(p)))
        };
        let last = 1u64 << SIDE_DEPTH;
        let result = (|| {
            let (fa,fb) = (at(0),at(last));
            if !(fa.min(fb) >= cap) { return None; }
            if !(b > a) { return Some(fa.min(fb)); }
            let speed = self.motion.inverse_point_speed_bound(p,self.domain).ok()?;
            let per = (b-a)/n;
            let bound = |i0: u64,f0: f64,i1: u64,f1: f64| 0.5*(f0+f1)-0.5*speed*per*(i1-i0) as f64;
            // lowest bound first, as `side` orders them
            let mut stretches: Vec<(f64,u64,f64,u64,f64)> = vec![(bound(0,fa,last,fb),0,fa,last,fb)];
            loop {
                let k = (0..stretches.len()).min_by(|&x,&y| stretches[x].0.total_cmp(&stretches[y].0))?;
                let (low,i0,f0,i1,f1) = stretches.swap_remove(k);
                if low >= cap { return Some(low); }
                if evaluations.get() >= budget || i1-i0 < 2 { return None; }
                let m = i0+(i1-i0)/2;
                let fm = at(m);
                if !(fm >= cap) { return None; }
                stretches.push((bound(i0,f0,m,fm),i0,f0,m,fm));
                stretches.push((bound(m,fm,i1,f1),m,fm,i1,f1));
            }
        })();
        SIDE_EVALUATIONS.fetch_add(evaluations.get(),std::sync::atomic::Ordering::Relaxed);
        result
    }

    /// How far `p` stands outside the box the whole sweep lies in, when it does: then no pose of
    /// the source reaches it, and the distance is a lower bound on the field there (it is
    /// one-Lipschitz and the material is inside the box), positive, with the field's sign. A cut
    /// indexed round a blank is far from most points asked about, and reading this costs nothing.
    pub fn clear_of(&self,p: [f64;3]) -> Option<f64> {
        let support = (*self.support.get_or_init(|| self.support_bounds().ok().flatten().map(|b| b.map(|x| x.bounds()))))?;
        let d2: f64 = (0..3).map(|k| (support[k][0]-p[k]).max(p[k]-support[k][1]).max(0.).powi(2)).sum();
        (d2 > 0.).then(|| d2.sqrt())
    }

    /// A number with the field's sign at a point, in plain floating point, for a mesher that
    /// asks only which side a point is on: never an interval claim, and its magnitude only an
    /// upper bound on the field. Along the point's path in the tool's frame the source's `value`
    /// is Lipschitz in the roll by the motion's inverse-point speed bound, so a stretch of roll
    /// between two readings is at least their mean less that bound times half its width. The
    /// stretches are split lowest bound first until a reading is negative (inside), every bound
    /// is within a ten-billionth of the point's size of zero (a tie), or the stretch is too short
    /// to split: far from the boundary one
    /// bound decides, and near it the work grows as the logarithm of the distance.
    pub fn side(&self,p: [f64;3]) -> f64 {
        if let Some(d) = self.clear_of(p) { return d; }
        let [a,b] = self.domain.bounds();
        let n = (1u64 << SIDE_DEPTH) as f64;
        let time = |i: u64| a+(b-a)*(i as f64/n);
        let table = self.poses.get_or_init(|| std::sync::Arc::new((0..=1u64 << SIDE_LEVELS)
            .map(|k| self.motion.pose_at(time(k << (SIDE_DEPTH-SIDE_LEVELS))).ok().map(|m| m.inverse())).collect()));
        let evaluations = std::cell::Cell::new(0u64);
        let at_time = |t: f64| {
            evaluations.set(evaluations.get()+1);
            self.motion.pose_at(t).ok().map(|m| m.inverse()).map_or(f64::INFINITY,|m| self.source.value(m.point(p)))
        };
        let at = |i: u64| {
            evaluations.set(evaluations.get()+1);
            let step = SIDE_DEPTH-SIDE_LEVELS;
            let pose = if i & ((1u64 << step)-1) == 0 { table[(i >> step) as usize] }
                else { self.motion.pose_at(time(i)).ok().map(|m| m.inverse()) };
            pose.map_or(f64::INFINITY,|m| self.source.value(m.point(p)))
        };
        let last = 1u64 << SIDE_DEPTH;
        let (fa,fb) = (at(0),at(last));
        let mut best = fa.min(fb);
        let count = |n: u64| SIDE_EVALUATIONS.fetch_add(n,std::sync::atomic::Ordering::Relaxed);
        if !(b > a) || best < 0. { count(evaluations.get()); return best; }
        let Ok(speed) = self.motion.inverse_point_speed_bound(p,self.domain) else { count(evaluations.get()); return best };
        let per = (b-a)/n;
        // A bound this close to zero leaves the side a tie a mesher's bisection resolves by
        // position; refining it further buys a sign nothing downstream can use.
        let tolerance = 1e-10*(1.+(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]).sqrt());
        let bound = |i0: u64,f0: f64,i1: u64,f1: f64| 0.5*(f0+f1)-0.5*speed*per*(i1-i0) as f64;
        // Lowest bound first: a heap ordered by the bound reversed.
        #[derive(PartialEq)]
        struct Stretch(f64,u64,f64,u64,f64);
        impl Eq for Stretch {}
        impl PartialOrd for Stretch { fn partial_cmp(&self,o: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(o)) } }
        impl Ord for Stretch { fn cmp(&self,o: &Self) -> std::cmp::Ordering { o.0.total_cmp(&self.0) } }
        let mut stretches = std::collections::BinaryHeap::new();
        stretches.push(Stretch(bound(0,fa,last,fb),0,fa,last,fb));
        // Near a rolling contact the path runs along the tool, so the source rises slowly away
        // from its minimum: a long shallow basin that a first-order bound splits stretch by
        // stretch at every level. So splitting stops at `SIDE_BASIN`: the stretches that may
        // still hold a negative value are grouped into contiguous runs, each a basin, and each
        // basin's minimum is found by golden section around its lowest reading. That is a
        // reading, not a bound: a second dip inside one run, between readings, can be missed.
        let basin = last >> SIDE_BASIN;
        let result = 'search: {
        loop {
            let Some(Stretch(low,i0,f0,i1,f1)) = stretches.pop() else { break 'search best };
            if low > -tolerance { break 'search best; }
            if i1-i0 <= basin {
                let mut open: Vec<(u64,f64,u64,f64)> = vec![(i0,f0,i1,f1)];
                open.extend(stretches.drain().filter(|s| s.0 <= -tolerance).map(|s| (s.1,s.2,s.3,s.4)));
                open.sort_by_key(|s| s.0);
                let mut runs: Vec<Vec<(u64,f64,u64,f64)>> = Vec::new();
                for s in open {
                    match runs.last_mut() { Some(r) if r.last().unwrap().2 == s.0 => r.push(s), _ => runs.push(vec![s]) }
                }
                for run in runs {
                    // The lowest reading in the run and the readings either side bracket its minimum.
                    let mut readings: Vec<(u64,f64)> = run.iter().map(|s| (s.0,s.1)).collect();
                    let end = run[run.len()-1];
                    readings.push((end.2,end.3));
                    let k = (0..readings.len()).min_by(|&x,&y| readings[x].1.total_cmp(&readings[y].1)).unwrap();
                    let w = run[0].2-run[0].0;
                    let (w0,w1) = (readings[k.saturating_sub(1)].0.saturating_sub(if k == 0 { w } else { 0 }),
                        (if k+1 < readings.len() { readings[k+1].0 } else { readings[k].0+w }).min(last));
                    let (mut lo,mut hi) = (time(w0),time(w1));
                    let g = 0.5*(5f64.sqrt()-1.);
                    let (mut x1,mut x2) = (hi-g*(hi-lo),lo+g*(hi-lo));
                    let (mut g1,mut g2) = (at_time(x1),at_time(x2));
                    // 28 steps narrow the bracket by 10⁻⁶: the minimum is quadratic, so its value
                    // is then found to 10⁻¹² of the bracket's rise, far below any tolerance.
                    for _ in 0..28 {
                        if g1.min(g2) < 0. { break; }
                        if g1 <= g2 { hi = x2; x2 = x1; g2 = g1; x1 = hi-g*(hi-lo); g1 = at_time(x1); }
                        else { lo = x1; x1 = x2; g1 = g2; x2 = lo+g*(hi-lo); g2 = at_time(x2); }
                    }
                    let found = g1.min(g2);
                    best = best.min(found);
                    if found < 0. { break 'search found; }
                }
                break 'search best;
            }
            let im = i0+(i1-i0)/2;
            let fm = at(im);
            best = best.min(fm);
            if fm < 0. { break 'search fm; }
            stretches.push(Stretch(bound(i0,f0,im,fm),i0,f0,im,fm));
            stretches.push(Stretch(bound(im,fm,i1,f1),im,fm,i1,f1));
        }
        };
        count(evaluations.get());
        result
    }

    /// The field's value at a point and the roll time it is least at, in plain floating point —
    /// `side`'s search carried on until no stretch of roll can read more than `accuracy` below the
    /// best reading, where `side` stops at the first negative one. The value is a reading, not an
    /// interval claim: a stretch is bounded by the motion's inverse-point speed, and a basin is
    /// searched by golden section around its lowest reading, so a second dip inside one basin can
    /// be missed. `tied` is set when the minima of two separate basins are within `tie` of each
    /// other: two contact times, which is a crease of the swept surface.
    pub fn minimum_at(&self,p: [f64;3],accuracy: f64,tie: f64) -> SweptMinimum {
        self.minimum_relative(p,accuracy,0.,tie)
    }

    /// `minimum_at` found only to `relative` of its own size where that is coarser than
    /// `accuracy`: a Newton step from a point far from the boundary needs its value to a few
    /// digits, and the search stops as soon as no stretch can read that much lower.
    pub fn minimum_relative(&self,p: [f64;3],accuracy: f64,relative: f64,tie: f64) -> SweptMinimum {
        self.minimum_hinted(p,accuracy,relative,tie,None,false)
    }

    /// `minimum_relative` warm-started from `hint`, the contact time at a nearby point: the
    /// basin-wide window about it (a `SIDE_BASIN`th of the roll) is searched first, and its
    /// minimum is the best reading the bounded search over the whole roll starts from — so every
    /// stretch that cannot beat it is pruned as `side` prunes, the window itself is not searched
    /// again, and a deeper minimum anywhere else is still found. A hint far from the contact
    /// costs the window's search and nothing in correctness.
    ///
    /// `local` trusts the window: its minimum is returned without the search over the whole roll,
    /// unless it lies at the window's edge (the contact moved further than the window reaches),
    /// when the whole roll is searched after all. That is a continuation, not a minimum: a deeper
    /// contact elsewhere is not looked for, and a caller that takes it must check its conclusions
    /// another way (the refinement checks a crossing's bracket by `side`).
    pub fn minimum_hinted(&self,p: [f64;3],accuracy: f64,relative: f64,tie: f64,hint: Option<f64>,local: bool) -> SweptMinimum {
        let [a,b] = self.domain.bounds();
        let n = (1u64 << SIDE_DEPTH) as f64;
        let time = |i: u64| a+(b-a)*(i as f64/n);
        let table = self.poses.get_or_init(|| std::sync::Arc::new((0..=1u64 << SIDE_LEVELS)
            .map(|k| self.motion.pose_at(time(k << (SIDE_DEPTH-SIDE_LEVELS))).ok().map(|m| m.inverse())).collect()));
        let evaluations = std::cell::Cell::new(0u64);
        let at_time = |t: f64| {
            evaluations.set(evaluations.get()+1);
            self.motion.pose_at(t).ok().map(|m| m.inverse()).map_or(f64::INFINITY,|m| self.source.value(m.point(p)))
        };
        let at = |i: u64| {
            evaluations.set(evaluations.get()+1);
            let step = SIDE_DEPTH-SIDE_LEVELS;
            let pose = if i & ((1u64 << step)-1) == 0 { table[(i >> step) as usize] }
                else { self.motion.pose_at(time(i)).ok().map(|m| m.inverse()) };
            pose.map_or(f64::INFINITY,|m| self.source.value(m.point(p)))
        };
        let last = 1u64 << SIDE_DEPTH;
        let (fa,fb) = (at(0),at(last));
        let (mut best,mut best_t) = if fa <= fb { (fa,a) } else { (fb,b) };
        let count = |n: u64| SIDE_EVALUATIONS.fetch_add(n,std::sync::atomic::Ordering::Relaxed);
        let done = |value: f64,time: f64,tied: bool,n: u64| { count(n); SweptMinimum {value,time,tied} };
        if !(b > a) { return done(best,best_t,false,evaluations.get()); }
        let basin = last >> SIDE_BASIN;
        // The golden section about a bracket, stopping once its two readings agree to `stop`.
        let golden = |mut lo: f64,mut hi: f64,stop: f64| -> (f64,f64) {
            let g = 0.5*(5f64.sqrt()-1.);
            let (mut x1,mut x2) = (hi-g*(hi-lo),lo+g*(hi-lo));
            let (mut g1,mut g2) = (at_time(x1),at_time(x2));
            for _ in 0..40 {
                if hi-lo <= 1e-12*(1.+hi.abs()) { break; }
                // the minimum is quadratic: once the two readings agree to the accuracy asked,
                // the least of them is that close to it, however near zero it is
                if (g1-g2).abs() <= 0.25*stop { break; }
                if g1 <= g2 { hi = x2; x2 = x1; g2 = g1; x1 = hi-g*(hi-lo); g1 = at_time(x1); }
                else { lo = x1; x1 = x2; g1 = g2; x2 = lo+g*(hi-lo); g2 = at_time(x2); }
            }
            if g1 <= g2 { (g1,x1) } else { (g2,x2) }
        };
        // The hint's window, in grid indices, searched first; `None` when cold.
        let window = hint.filter(|t| t.is_finite() && *t >= a && *t <= b).map(|t| {
            let centre = ((t-a)/(b-a)*n).round().clamp(0.,last as f64) as u64;
            let (w0,w1) = (centre.saturating_sub(basin/2),(centre+basin/2).min(last));
            let found = golden(time(w0),time(w1),accuracy);
            if found.0 < best { best = found.0; best_t = found.1; }
            (w0,w1,found)
        });
        if local {
            if let Some((w0,w1,found)) = window {
                let margin = 0.02*(time(w1)-time(w0));
                let at_edge = (found.1-time(w0) < margin && w0 > 0) || (time(w1)-found.1 < margin && w1 < last);
                if !at_edge { return done(found.0,found.1,false,evaluations.get()); }
            }
        }
        let Ok(speed) = self.motion.inverse_point_speed_bound(p,self.domain) else { return done(best,best_t,false,evaluations.get()) };
        let per = (b-a)/n;
        let bound = |i0: u64,f0: f64,i1: u64,f1: f64| 0.5*(f0+f1)-0.5*speed*per*(i1-i0) as f64;
        #[derive(PartialEq)]
        struct Stretch(f64,u64,f64,u64,f64);
        impl Eq for Stretch {}
        impl PartialOrd for Stretch { fn partial_cmp(&self,o: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(o)) } }
        impl Ord for Stretch { fn cmp(&self,o: &Self) -> std::cmp::Ordering { o.0.total_cmp(&self.0) } }
        let mut stretches = std::collections::BinaryHeap::new();
        stretches.push(Stretch(bound(0,fa,last,fb),0,fa,last,fb));
        let mut tied = false;
        let searched = |i0: u64,i1: u64| window.is_some_and(|(w0,w1,_)| i0 >= w0 && i1 <= w1);
        loop {
            let Some(Stretch(low,i0,f0,i1,f1)) = stretches.pop() else { break };
            let accuracy = accuracy.max(relative*best.abs());
            if low > best-accuracy { break; }
            if searched(i0,i1) { continue; }
            if i1-i0 <= basin {
                // As in `side`: the stretches still able to beat the best reading, grouped into
                // contiguous runs, each a basin whose minimum golden section finds.
                let mut open: Vec<(u64,f64,u64,f64)> = vec![(i0,f0,i1,f1)];
                open.extend(stretches.drain().filter(|s| s.0 <= best-accuracy && !searched(s.1,s.3)).map(|s| (s.1,s.2,s.3,s.4)));
                open.sort_by_key(|s| s.0);
                let mut runs: Vec<Vec<(u64,f64,u64,f64)>> = Vec::new();
                for s in open {
                    match runs.last_mut() { Some(r) if r.last().unwrap().2 == s.0 => r.push(s), _ => runs.push(vec![s]) }
                }
                let stop = accuracy;
                let mut minima: Vec<(f64,f64)> = Vec::new();
                for run in runs {
                    let mut readings: Vec<(u64,f64)> = run.iter().map(|s| (s.0,s.1)).collect();
                    let end = run[run.len()-1];
                    readings.push((end.2,end.3));
                    let k = (0..readings.len()).min_by(|&x,&y| readings[x].1.total_cmp(&readings[y].1)).unwrap();
                    let w = run[0].2-run[0].0;
                    let (w0,w1) = (readings[k.saturating_sub(1)].0.saturating_sub(if k == 0 { w } else { 0 }),
                        (if k+1 < readings.len() { readings[k+1].0 } else { readings[k].0+w }).min(last));
                    minima.push(golden(time(w0),time(w1),stop));
                }
                // The window's minimum is a basin too. A run's beside it is the same basin carried
                // on past the window's edge (the contact moved further than the window reaches),
                // and the lower of the two is that basin's minimum.
                if let Some((w0,w1,found)) = window {
                    let reach = time(w1)-time(w0);
                    let mut basin_min = found;
                    let mut others = Vec::new();
                    for m in minima {
                        if (m.1-found.1).abs() <= reach { if m.0 < basin_min.0 { basin_min = m; } }
                        else { others.push(m); }
                    }
                    minima = others;
                    minima.push(basin_min);
                }
                minima.sort_by(|x,y| x.0.total_cmp(&y.0));
                if let Some(&(v,t)) = minima.first() { if v < best { best = v; best_t = t; } }
                tied = minima.len() > 1 && minima[1].0-minima[0].0 <= tie;
                break;
            }
            let im = i0+(i1-i0)/2;
            let fm = at(im);
            if fm < best { best = fm; best_t = time(im); }
            stretches.push(Stretch(bound(i0,f0,im,fm),i0,f0,im,fm));
            stretches.push(Stretch(bound(im,fm,i1,f1),im,fm,i1,f1));
        }
        done(best,best_t,tied,evaluations.get())
    }

    /// The source, for a reading that follows a minimum to the tool's own gradient.
    pub(crate) fn source(&self) -> &SpatialField { &self.source }
    pub(crate) fn motion(&self) -> &Family { &self.motion }

    /// Numerical storage is an evaluator control, not a geometry parameter.
    /// Zero disables caching. Reaching the cap merely recomputes later poses;
    /// it cannot discard motion intervals or change the resulting enclosure.
    pub fn evaluator(&self,max_cached_poses: usize) -> SweepEvaluator {
        SweepEvaluator {field:self.clone(),poses:BTreeMap::new(),max_cached_poses}
    }
}

/// Reusable query state for one immutable swept field. The pose cache belongs
/// to that exact field/motion snapshot and has an explicit capacity.
pub struct SweepEvaluator {
    field:SweptField,
    poses:BTreeMap<u64,MotionBounds>,
    max_cached_poses:usize,
}

impl SweepEvaluator {
    pub fn domain(&self) -> I { self.field.domain }
    pub fn cached_poses(&self) -> usize { self.poses.len() }
    pub fn clear_cache(&mut self) { self.poses.clear(); }

    /// Enclose the swept field for every point in the input box. Options bound
    /// the complete roll search; termination status and uncertainty are retained.
    /// The value tolerance is a field width, not a geometric export tolerance.
    pub fn bounds(&mut self,p: V,options: Options) -> Result<Minimum,SweepError> {
        self.bounds_with_observer(p,options,|_,_| {})
    }

    /// Stop when the full sweep enclosure is strictly outside the requested
    /// field-value band. `Separated` retains bounds and an attained witness;
    /// it does not claim convergence to the value-width tolerance.
    pub fn bounds_outside(&mut self,p: V,band: I,options: Options) -> Result<Minimum,SweepError> {
        self.evaluate(p,options,Stop::Outside(band),|_,_| {})
    }

    /// Observe raw interval-oracle enclosures, e.g. to extract independently
    /// checkable coverage evidence. The observer supplies no geometry, bounds
    /// or pruning decisions; it cannot change the oracle's mathematical result.
    pub fn bounds_with_observer(&mut self,p: V,options: Options,observe: impl FnMut(I,I))
        -> Result<Minimum,SweepError> {
        self.evaluate(p,options,Stop::Converged,observe)
    }

    /// The search stops as `stop` allows (see `minimum::Stop`).
    pub(super) fn evaluate(&mut self,p: V,options: Options,stop: Stop,mut observe: impl FnMut(I,I))
        -> Result<Minimum,SweepError> {
        // The motion's speed bound over the whole input box.
        let speed = self.field.motion.inverse_point_speed_bound_over(p,self.field.domain)
            .and_then(I::point).map_err(minimum::Error::Oracle)?;
        // The refiner bounds a new cell and then samples the cell's midpoint,
        // and both are the source at the pose of that one midpoint: the last
        // midpoint's value is kept, and only the travel differs.
        let mut last: Option<(u64,I)> = None;
        minimum::refine(self.field.domain,|t| {
            let [lo,hi] = t.bounds(); let mid = lo*0.5+hi*0.5;
            let key = mid.to_bits();
            let value = match last {
                Some((k,value)) if k == key => value,
                _ => {
                    let pose = if let Some(pose) = self.poses.get(&key) { *pose } else {
                        let pose = self.field.motion.bounds(I::point(mid)?)?;
                        if self.poses.len() < self.max_cached_poses { self.poses.insert(key,pose); }
                        pose
                    };
                    let value = self.field.source.bounds(pose.inverse_point(p)?)?;
                    last = Some((key,value));
                    value
                }
            };
            let dt = t.sub(I::point(mid)?)?.bounds();
            let travel = speed.mul(I::point(dt[0].abs().max(dt[1].abs()))?)?.bounds()[1];
            // SpatialField's constructors establish the one-Lipschitz contract.
            // No arbitrary value callback or assumed evaluation-error band enters.
            let bound = value.add(I::new(-travel,travel)?)?;
            observe(t,bound);
            Ok::<_,Error>(bound)
        },options,stop)
    }
}
