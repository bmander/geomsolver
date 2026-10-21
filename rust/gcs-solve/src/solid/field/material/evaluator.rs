//! Interval enclosures of a material field over point boxes: every operand bounded, each sweep by
//! its own capped pose cache (`SweepEvaluator`), with the evidence each sweep query leaves.
use super::{MaterialField,min,max,Error,I,V,SweepEvaluator,SweepError};
use super::plan::{Bool,Fold,OpId};
use super::super::{SpatialField,SweptField};
use crate::interval::minimum::{Minimum,Options,Stop};
use crate::motion::MotionBounds;
use std::collections::HashMap;

type QueryCache = HashMap<(OpId,[[u64;2];3],(u8,[u64;2])),Result<I,SweepError>>;

/// A stop as part of a memo key: which kind, and its band's bits.
fn stop_key(stop: Stop) -> (u8,[u64;2]) {
    let bits = |b: I| super::super::memo::bits(b.bounds());
    match stop { Stop::Converged => (0,[0;2]),Stop::Outside(b) => (1,bits(b)),Stop::Decided(b) => (2,bits(b)) }
}

impl MaterialField {
    /// Cloned swept operands share one capped pose cache, even under different
    /// fixed transforms. The cap applies per distinct swept node, not per copy.
    pub fn evaluator(&self,max_cached_poses_per_sweep: usize) -> MaterialEvaluator {
        MaterialEvaluator {field:self.clone(),sweeps:HashMap::new(),max_cached_poses_per_sweep}
    }
}

/// Evidence for one swept-node/input-box query, in first-visit traversal order.
/// Shared nodes at an identical box and requested band are evaluated once per query.
#[derive(Clone,Debug)]
pub struct MaterialSweepQuery {
    pub point_box: V,
    pub domain: I,
    pub minimum: Minimum,
    /// The requested leaf band, reflected when this operand is subtracted.
    pub separation_band: Option<I>,
}

#[derive(Clone,Debug)]
pub struct MaterialBounds {
    pub value: I,
    /// All contributing sweep queries, including exhausted or resolution-limited
    /// ones. Their tolerance is a field width, not an export distance bound.
    pub sweeps: Vec<MaterialSweepQuery>,
}

/// Query state belongs to one immutable root. Only motion poses persist across
/// calls; point-box results and their diagnostics are rebuilt for each query. Each sweep's pose
/// cache is keyed by its op in the root's plan, which a sweep placed many times has once.
pub struct MaterialEvaluator {
    field:MaterialField,
    sweeps:HashMap<OpId,SweepEvaluator>,
    max_cached_poses_per_sweep:usize,
}

impl MaterialEvaluator {
    pub fn support_bounds(&self) -> Result<Option<V>,Error> { self.field.support_bounds() }
    pub fn cached_poses(&self) -> usize { self.sweeps.values().map(SweepEvaluator::cached_poses).sum() }
    pub fn clear_cache(&mut self) { self.sweeps.values_mut().for_each(SweepEvaluator::clear_cache); }

    /// Enclose the field over a point box, stopping each swept operand as `stop` allows:
    /// `Stop::Converged` refines every sweep to `options`' value tolerance; `Stop::Outside(band)`
    /// lets a sweep stop once strictly outside the band (a subtracted operand's band reflected),
    /// and the final interval, not a leaf status, classifies; `Stop::Decided` reaches a sweep
    /// through fixed poses alone, whose enclosure is the root's, and across a Boolean becomes
    /// `Stop::Outside`, since an operand inside the band decides nothing about the composite.
    ///
    /// Options apply independently to each unique swept-node/input-box query. No Boolean branch is
    /// skipped, including on budget exhaustion: failures and unresolved bounds remain. Strict
    /// negative/positive result bounds establish material/exterior respectively; bounds containing
    /// zero remain unresolved. Static leaves need no roll budget.
    ///
    /// `observe` receives raw interval-oracle evidence: its first argument indexes the returned
    /// `sweeps`, the others are the roll-oracle domain and bound. It supplies no geometry and
    /// cannot affect stopping. Memoized DAG visits do not repeat observations; on error any
    /// partial observations must be discarded.
    pub fn query(&mut self,p: V,stop: Stop,options: Options,mut observe: Option<&mut dyn FnMut(usize,I,I)>)
        -> Result<MaterialBounds,SweepError> {
        let mut queries = vec![];
        let mut observe = |q: usize,d: I,b: I| if let Some(o) = observe.as_mut() { o(q,d,b) };
        let plan = self.field.plan();
        let mut fold = Enclose {sweeps:&mut self.sweeps,max_cached_poses:self.max_cached_poses_per_sweep,options,
            cache:HashMap::new(),queries:&mut queries,observe:&mut observe};
        let value = plan.fold(plan.root(),(p,stop),&mut fold)?;
        Ok(MaterialBounds {value,sweeps:queries})
    }

    /// `query` to convergence, unobserved.
    pub fn bounds(&mut self,p: V,options: Options) -> Result<MaterialBounds,SweepError> {
        self.query(p,Stop::Converged,options,None)
    }
}

/// `MaterialEvaluator::query`'s fold: every operand enclosed, each unique op, box and stop once.
struct Enclose<'a,O: FnMut(usize,I,I)> {
    sweeps: &'a mut HashMap<OpId,SweepEvaluator>,
    max_cached_poses: usize,
    options: Options,
    cache: QueryCache,
    queries: &'a mut Vec<MaterialSweepQuery>,
    observe: &'a mut O,
}

impl<O: FnMut(usize,I,I)> Fold for Enclose<'_,O> {
    type In = (V,Stop);
    type Out = Result<I,SweepError>;
    fn leaf(&mut self,source: &SpatialField,(p,_): (V,Stop)) -> Self::Out { source.bounds(p).map_err(SweepError::Oracle) }
    fn sweep(&mut self,op: OpId,source: &SweptField,(p,stop): (V,Stop)) -> Self::Out {
        let sweep = self.sweeps.entry(op).or_insert_with(|| source.evaluator(self.max_cached_poses));
        let query = self.queries.len();
        let observe = &mut *self.observe;
        let minimum = sweep.query(p,stop,self.options,Some(&mut |d,b| observe(query,d,b)))?;
        self.queries.push(MaterialSweepQuery {point_box:p,domain:source.domain(),minimum,separation_band:stop.band()});
        Ok(minimum.value)
    }
    // a fixed pose moves the point, not the value: containment survives it
    fn into(&mut self,pose: &MotionBounds,(p,stop): (V,Stop)) -> Result<(V,Stop),Self::Out> {
        pose.inverse_point(p).map(|q| (q,stop)).map_err(|e| Err(SweepError::Oracle(e)))
    }
    fn operand(&mut self,op: Bool,second: bool,(p,stop): (V,Stop)) -> (V,Stop) {
        (p,if op == Bool::Difference && second { stop.operand().negated() } else { stop.operand() })
    }
    fn combine(&mut self,op: Bool,a: Self::Out,b: Option<Self::Out>) -> Self::Out {
        let (a,b) = (a?,b.expect("every operand is enclosed")?);
        Ok(match op { Bool::Union => min(a,b),Bool::Intersection => max(a,b),Bool::Difference => max(a,b.neg()) })
    }
    fn settled(&self,v: &Self::Out) -> bool { v.is_err() }
    fn recall(&mut self,op: OpId,&(p,stop): &(V,Stop)) -> Option<Self::Out> {
        self.cache.get(&(op,super::super::memo::box_bits(&p),stop_key(stop))).cloned()
    }
    fn keep(&mut self,op: OpId,(p,stop): (V,Stop),v: Self::Out) {
        self.cache.insert((op,super::super::memo::box_bits(&p),stop_key(stop)),v);
    }
}
