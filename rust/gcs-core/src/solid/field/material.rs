//! Composition of static and continuously swept material fields.
use super::{min,max,union_support,intersection_support,Error,I,V,SpatialField,SweptField,SweepEvaluator,SweepError};
use crate::{interval::minimum::{Minimum,Options,Stop},motion::{Family,MotionBounds}};
use std::{collections::HashMap,sync::Arc};

type QueryCache = HashMap<(usize,[[u64;2];3],(u8,[u64;2])),I>;

/// A stop as part of a memo key: which kind, and its band's bits.
fn stop_key(stop: Stop) -> (u8,[u64;2]) {
    let bits = |b: I| b.bounds().map(f64::to_bits);
    match stop { Stop::Converged => (0,[0;2]),Stop::Outside(b) => (1,bits(b)),Stop::Decided(b) => (2,bits(b)) }
}

#[derive(Clone,Debug)]
enum Node {
    Static(SpatialField),
    Swept(SweptField),
    Transformed {source:MaterialField,pose:MotionBounds},
    Union(MaterialField,MaterialField),
    Intersection(MaterialField,MaterialField),
    Difference(MaterialField,MaterialField),
}

/// Immutable one-Lipschitz material field, including completed swept operands.
/// Material is closure({f<0}); zero alone does not certify a boundary. Fixed
/// poses and Booleans compose without discretizing the generating motion.
/// Expression depth is limited to 64, separately from each static source graph.
/// A sweep's source is still a SpatialField; nested continuous sweeps are not
/// represented by silently sampling or fixing an inner sweep.
#[derive(Clone,Debug)]
pub struct MaterialField {node:Arc<Node>,depth:u8}

impl From<SpatialField> for MaterialField {
    fn from(source: SpatialField) -> Self { Self {node:Arc::new(Node::Static(source)),depth:1} }
}
impl From<SweptField> for MaterialField {
    fn from(source: SweptField) -> Self { Self {node:Arc::new(Node::Swept(source)),depth:1} }
}

impl MaterialField {
    /// A construction-derived finite box containing all regularized material.
    /// `None` means this construction has no derived finite support, not that
    /// it is necessarily unbounded. No samples or caller-provided crop enter.
    pub fn support_bounds(&self) -> Result<Option<V>,Error> { self.support(&mut HashMap::new()) }
    fn support(&self,cache: &mut HashMap<usize,Option<V>>) -> Result<Option<V>,Error> {
        let key = Arc::as_ptr(&self.node) as usize;
        if let Some(value) = cache.get(&key) { return Ok(*value); }
        let value = match self.node.as_ref() {
            Node::Static(source) => source.support_bounds()?,
            Node::Swept(source) => source.support_bounds()?,
            Node::Transformed {source,pose} => source.support(cache)?.map(|b| pose.point(b)).transpose()?,
            Node::Union(a,b) => union_support(a.support(cache)?,b.support(cache)?),
            Node::Intersection(a,b) => intersection_support(a.support(cache)?,b.support(cache)?),
            Node::Difference(a,_) => a.support(cache)?,
        };
        cache.insert(key,value);
        Ok(value)
    }
    fn node(node: Node,depth: u8) -> Result<Self,Error> {
        if depth > 64 { return Err(Error::OutsideDomain); }
        Ok(Self {node:Arc::new(node),depth})
    }
    pub fn transformed(self,under: &Family,at: f64) -> Result<Self,Error> {
        let depth = self.depth+1;
        let pose = under.bounds(I::point(at)?)?;
        Self::node(Node::Transformed {source:self,pose},depth)
    }
    fn combine(self,other: Self,make: impl FnOnce(Self,Self)->Node) -> Result<Self,Error> {
        let depth = self.depth.max(other.depth)+1;
        Self::node(make(self,other),depth)
    }
    pub fn union(self,other: Self) -> Result<Self,Error> { self.combine(other,Node::Union) }
    pub fn intersection(self,other: Self) -> Result<Self,Error> { self.combine(other,Node::Intersection) }
    pub fn difference(self,other: Self) -> Result<Self,Error> { self.combine(other,Node::Difference) }

    /// A number with the field's sign at a point, in plain floating point: `SpatialField::value`
    /// for static material and `SweptField::side` for a sweep, composed by the Booleans (which
    /// keep signs). Which side a point is on, for a mesher; never an interval claim.
    pub fn side(&self,p: [f64;3]) -> f64 {
        match self.node.as_ref() {
            Node::Static(source) => source.value(p),
            Node::Swept(source) => source.side(p),
            Node::Transformed {source,pose} => source.side(pose.inverse_point_mid(p)),
            Node::Union(a,b) => a.side(p).min(b.side(p)),
            Node::Intersection(a,b) => a.side(p).max(b.side(p)),
            Node::Difference(a,b) => a.side(p).max(-b.side(p)),
        }
    }

    /// The value, gradient and deciding operand at a point (`reading.rs`), with tolerances
    /// scaled to the point.
    pub fn reading(&self,p: [f64;3]) -> super::Reading {
        self.reading_with(p,&super::ReadingOptions::at(p),&mut 0)
    }

    /// A reading with given tolerances, leaves numbered from `*next`.
    pub fn reading_with(&self,p: [f64;3],options: &super::ReadingOptions,next: &mut usize) -> super::Reading {
        self.reading_warm(p,options,next,&mut Vec::new())
    }

    /// A reading warm-started from a nearby point's: `hints[k]` is the contact time of the sweep
    /// numbered `k` there (`SweptField::minimum_hinted`), and is left holding this point's. An
    /// empty vector is a cold reading; a hint from far away costs time, never the answer.
    pub fn reading_warm(&self,p: [f64;3],options: &super::ReadingOptions,next: &mut usize,
        hints: &mut Vec<Option<f64>>) -> super::Reading {
        use super::reading::{higher,lower};
        match self.node.as_ref() {
            Node::Static(source) => source.reading(p,options,next),
            Node::Swept(source) => {
                // the sweep is keyed by the number its first source leaf will take
                let key = *next;
                if hints.len() <= key { hints.resize(key+1,None); }
                let m = source.minimum_hinted(p,options.accuracy,options.relative,options.tie,hints[key],options.local);
                hints[key] = Some(m.time);
                // The tool's own reading at the roll time the sweep is least, turned into the
                // world: its value is the minimum, and so is its gradient (the envelope theorem).
                let pose = source.motion().pose_at(m.time).ok().map(|x| x.inverse());
                let Some(inverse) = pose else {
                    *next += 1;
                    return super::Reading {value:m.value,gradient:[0.;3],leaf:*next-1,time:Some(m.time),ambiguous:true};
                };
                let r = source.source().reading(inverse.point(p),options,next);
                let g = r.gradient;
                let gradient = std::array::from_fn(|i| (0..3).map(|k| inverse.r[k][i]*g[k]).sum());
                super::Reading {value:m.value,gradient,time:Some(m.time),ambiguous:r.ambiguous || m.tied,..r}
            }
            Node::Transformed {source,pose} => {
                let r = source.reading_warm(pose.inverse_point_mid(p),options,next,hints);
                super::Reading {gradient:pose.gradient_mid(r.gradient),..r}
            }
            Node::Union(a,b) => {
                let (x,y) = (a.reading_warm(p,options,next,hints),b.reading_warm(p,options,next,hints));
                lower(x,y,options.tie)
            }
            Node::Intersection(a,b) => {
                let (x,y) = (a.reading_warm(p,options,next,hints),b.reading_warm(p,options,next,hints));
                higher(x,y,options.tie)
            }
            Node::Difference(a,b) => {
                let (x,y) = (a.reading_warm(p,options,next,hints),b.reading_warm(p,options,next,hints));
                higher(x,y.negated(),options.tie)
            }
        }
    }

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
/// calls; point-box results and their diagnostics are rebuilt for each query.
pub struct MaterialEvaluator {
    field:MaterialField,
    sweeps:HashMap<usize,SweepEvaluator>,
    max_cached_poses_per_sweep:usize,
}

impl MaterialEvaluator {
    pub fn support_bounds(&self) -> Result<Option<V>,Error> { self.field.support_bounds() }
    pub fn cached_poses(&self) -> usize { self.sweeps.values().map(SweepEvaluator::cached_poses).sum() }
    pub fn clear_cache(&mut self) { self.sweeps.values_mut().for_each(SweepEvaluator::clear_cache); }

    /// Options apply independently to each unique swept-node/input-box query.
    /// No Boolean branch is skipped, including on budget exhaustion. Strict
    /// negative/positive result bounds establish material/exterior respectively;
    /// bounds containing zero remain unresolved. Static leaves need no roll budget.
    pub fn bounds(&mut self,p: V,options: Options) -> Result<MaterialBounds,SweepError> {
        self.bounds_with_observer(p,options,|_,_,_| {})
    }

    /// Enclose every operand, permitting each swept leaf to stop when strictly
    /// outside the requested band. Difference reflects the right operand's
    /// band. The final interval, not a leaf status, determines classification.
    /// Every operand is still evaluated; failures and unresolved bounds remain.
    pub fn bounds_outside(&mut self,p: V,band: I,options: Options) -> Result<MaterialBounds,SweepError> {
        self.bounds_outside_with_observer(p,band,options,|_,_,_| {})
    }

    /// Band-limited refinement with the same raw evidence and query indexing
    /// contract as `bounds_with_observer`. The observer cannot affect stopping.
    pub fn bounds_outside_with_observer(&mut self,p: V,band: I,options: Options,
        observe: impl FnMut(usize,I,I)) -> Result<MaterialBounds,SweepError> {
        self.query(p,options,Stop::Outside(band),observe)
    }

    /// Enclose every operand, each swept leaf stopping as `stop` allows. `Stop::Decided`
    /// reaches a sweep through fixed poses alone, whose enclosure is the root's; across a
    /// Boolean an operand inside the band decides nothing about the composite, and there it
    /// becomes `Stop::Outside` (reflected for a subtracted operand).
    pub fn bounds_stopping(&mut self,p: V,stop: Stop,options: Options) -> Result<MaterialBounds,SweepError> {
        self.query(p,options,stop,|_,_,_| {})
    }

    /// The first observer argument indexes the returned `sweeps` vector. The
    /// other arguments are the raw roll-oracle domain and bound. Memoized DAG
    /// visits do not repeat observations. On error, any partial observations
    /// must be discarded; no successful material result is returned.
    pub fn bounds_with_observer(&mut self,p: V,options: Options,observe: impl FnMut(usize,I,I))
        -> Result<MaterialBounds,SweepError> {
        self.query(p,options,Stop::Converged,observe)
    }

    fn query(&mut self,p: V,options: Options,stop: Stop,mut observe: impl FnMut(usize,I,I))
        -> Result<MaterialBounds,SweepError> {
        let mut queries = vec![];
        let root = self.field.clone();
        let value = self.evaluate(&root,p,options,stop,&mut HashMap::new(),&mut queries,&mut observe)?;
        Ok(MaterialBounds {value,sweeps:queries})
    }

    fn evaluate(&mut self,field: &MaterialField,p: V,options: Options,stop: Stop,
        cache: &mut QueryCache,queries: &mut Vec<MaterialSweepQuery>,
        observe: &mut impl FnMut(usize,I,I)) -> Result<I,SweepError> {
        let id = Arc::as_ptr(&field.node) as usize;
        let key = (id,p.map(|v| v.bounds().map(f64::to_bits)),stop_key(stop));
        if let Some(value) = cache.get(&key) { return Ok(*value); }
        let value = match field.node.as_ref() {
            Node::Static(source) => source.bounds(p).map_err(SweepError::Oracle)?,
            Node::Swept(source) => {
                let sweep = self.sweeps.entry(id)
                    .or_insert_with(|| source.evaluator(self.max_cached_poses_per_sweep));
                let query = queries.len();
                let minimum = sweep.evaluate(p,options,stop,|d,b| observe(query,d,b))?;
                queries.push(MaterialSweepQuery {point_box:p,domain:source.domain(),minimum,separation_band:stop.band()});
                minimum.value
            },
            // a fixed pose moves the point, not the value: containment survives it
            Node::Transformed {source,pose} => self.evaluate(source,
                pose.inverse_point(p).map_err(SweepError::Oracle)?,options,stop,cache,queries,observe)?,
            Node::Union(a,b) => min(self.evaluate(a,p,options,stop.operand(),cache,queries,observe)?,
                self.evaluate(b,p,options,stop.operand(),cache,queries,observe)?),
            Node::Intersection(a,b) => max(self.evaluate(a,p,options,stop.operand(),cache,queries,observe)?,
                self.evaluate(b,p,options,stop.operand(),cache,queries,observe)?),
            Node::Difference(a,b) => max(self.evaluate(a,p,options,stop.operand(),cache,queries,observe)?,
                self.evaluate(b,p,options,stop.operand().negated(),cache,queries,observe)?.neg()),
        };
        cache.insert(key,value);
        Ok(value)
    }
}
