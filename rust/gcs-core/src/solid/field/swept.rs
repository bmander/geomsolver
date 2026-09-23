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
    poses:std::sync::OnceLock<std::sync::Arc<Vec<Option<crate::envelope::Motion>>>>}

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
            .map(|k| self.motion.at(time(k << (SIDE_DEPTH-SIDE_LEVELS))).ok().map(|m| m.inverse())).collect()));
        let at = |i: u64| {
            let step = SIDE_DEPTH-SIDE_LEVELS;
            let pose = if i & ((1u64 << step)-1) == 0 { table[(i >> step) as usize] }
                else { self.motion.at(time(i)).ok().map(|m| m.inverse()) };
            pose.map_or(f64::INFINITY,|m| self.source.value(m.point(p)))
        };
        let last = 1u64 << SIDE_DEPTH;
        let (fa,fb) = (at(0),at(last));
        let mut best = fa.min(fb);
        if !(b > a) || best < 0. { return best; }
        let Ok(speed) = self.motion.inverse_point_speed_bound(p,self.domain) else { return best };
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
        for _ in 0..4096 {
            let Some(Stretch(low,i0,f0,i1,f1)) = stretches.pop() else { break };
            if low > -tolerance || i1-i0 < 2 { break; }
            let im = i0+(i1-i0)/2;
            let fm = at(im);
            best = best.min(fm);
            if fm < 0. { return fm; }
            stretches.push(Stretch(bound(i0,f0,im,fm),i0,f0,im,fm));
            stretches.push(Stretch(bound(im,fm,i1,f1),im,fm,i1,f1));
        }
        best
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
