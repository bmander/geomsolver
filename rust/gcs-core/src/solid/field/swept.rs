//! Continuous volume sweeps of explicit one-Lipschitz material fields.
use super::{SpatialField,Error,I,V};
use crate::{interval::minimum::{self,Minimum,Options,Stop},motion::{Family,MotionBounds}};
use std::collections::BTreeMap;

pub type SweepError = minimum::Error<Error>;

/// The field min_t source(inverse(motion(t)) x) over a finite closed interval.
/// Source, motion and domain are immutable solved snapshots. Its material is
/// closure({f<0}); the field is one-Lipschitz but need not be signed distance.
/// A point domain represents one fixed pose. Source-solve error is separate.
#[derive(Clone,Debug)]
pub struct SweptField {source:SpatialField,motion:Family,domain:I,
    /// The inverse poses at the first `SIDE_TABLE` dyadic divisions of the roll, which every
    /// `side` query walks through first, filled once.
    poses:std::sync::OnceLock<std::sync::Arc<Vec<Option<crate::motion::Pose>>>>}

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
        Self {source,motion,domain,poses:std::sync::OnceLock::new()}
    }
    pub fn domain(&self) -> I { self.domain }

    /// Enclose every generating pose of the source's finite material support.
    /// The entire roll domain is passed through interval motion evaluation.
    pub fn support_bounds(&self) -> Result<Option<V>,Error> {
        self.source.support_bounds()?.map(|b| self.motion.bounds(self.domain)?.point(b)).transpose()
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
