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
pub struct SweptField {source:SpatialField,motion:Family,domain:I}

impl SweptField {
    pub fn new(source: SpatialField,motion: Family,domain: I) -> Self { Self {source,motion,domain} }
    pub fn domain(&self) -> I { self.domain }

    /// Enclose every generating pose of the source's finite material support.
    /// The entire roll domain is passed through interval motion evaluation.
    pub fn support_bounds(&self) -> Result<Option<V>,Error> {
        self.source.support_bounds()?.map(|b| self.motion.bounds(self.domain)?.point(b)).transpose()
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
