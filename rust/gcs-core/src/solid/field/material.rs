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
    Union(MaterialField,MaterialField,Arc<Spread>),
    Intersection(MaterialField,MaterialField),
    Difference(MaterialField,MaterialField),
}

/// A union's operands are sorted by sweeps' floors from cubes this many times the fine ones.
const SPREAD_COARSENING: u32 = 4;

/// A union's operands with the unions under it flattened, and, filled as cells are asked, the
/// operands that may decide it in each cell of a grid: a lower bound on each over the cell,
/// lowest first (`MaterialField::low`). An indexed cut is a union of as many placed copies as the
/// gear has teeth, and a point near one tooth space is decided by two or three of them; read in
/// turn, every copy costs a transform and a table lookup at every one of a mesher's queries.
#[derive(Debug,Default)]
struct Spread {
    operands: std::sync::OnceLock<Operands>,
    cells: std::sync::Mutex<super::adf::Map<[i32;3],Arc<[(f64,u32)]>>>,
}

/// A union's flattened operands, each with the number its first leaf takes counted from the
/// union's own first; the side of `Spread`'s cells; and the union's leaf count.
#[derive(Debug)]
struct Operands { list: Vec<(MaterialField,usize)>,cell: f64,leaves: usize }

/// Points spread over the support at which a symmetry is checked, beside the caller's.
const SYMMETRY_SAMPLES: usize = 6;

/// A rigid map the field reads the same under (`MaterialField::symmetries`): one placement's
/// inverse, then another's.
#[derive(Clone,Copy,Debug)]
pub struct Symmetry { from: MotionBounds,to: MotionBounds }

impl Symmetry {
    pub fn apply(&self,p: [f64;3]) -> [f64;3] { self.to.point_mid(self.from.inverse_point_mid(p)) }
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
            Node::Union(a,b,_) => union_support(a.support(cache)?,b.support(cache)?),
            Node::Intersection(a,b) => intersection_support(a.support(cache)?,b.support(cache)?),
            Node::Difference(a,_) => a.support(cache)?,
        };
        cache.insert(key,value);
        Ok(value)
    }
    /// A box holding all the material, tighter than `support_bounds`: that box split up to
    /// `depth` times, keeping the parts the field may be negative in, and their hull. A cut is
    /// left out of the test, since removing material cannot put any outside the rest, and so is a
    /// sweep standing for material (it may be anywhere its support says); only static operands
    /// prune. At most `cap` boxes are split a level; past that the hull stands as it is. Sizing,
    /// never a claim: the hull is a box the material is in, found by interval bounds.
    pub(crate) fn tight_support(&self,depth: usize,cap: usize) -> Result<Option<V>,Error> {
        let Some(support) = self.support_bounds()? else { return Ok(None) };
        let mut boxes = vec![support];
        for _ in 0..depth {
            if boxes.len()*8 > cap { break; }
            let mut next = Vec::new();
            for b in &boxes {
                let [lo,hi] = [0,1].map(|k| b.map(|x| x.bounds()[k]));
                let mid: [f64;3] = std::array::from_fn(|k| 0.5*(lo[k]+hi[k]));
                for corner in 0..8 {
                    let low = |k: usize| corner>>k & 1 == 0;
                    let part = boxed(std::array::from_fn(|k| if low(k) { lo[k] } else { mid[k] }),
                        std::array::from_fn(|k| if low(k) { mid[k] } else { hi[k] }))?;
                    if self.least(part) <= 0. { next.push(part); }
                }
            }
            if next.is_empty() { return Ok(None); }
            boxes = next;
        }
        let lo: [f64;3] = std::array::from_fn(|k| boxes.iter().map(|b| b[k].bounds()[0]).fold(f64::INFINITY,f64::min));
        let hi: [f64;3] = std::array::from_fn(|k| boxes.iter().map(|b| b[k].bounds()[1]).fold(f64::NEG_INFINITY,f64::max));
        Ok(Some(boxed(lo,hi)?))
    }

    /// A lower bound on the field over a box from its static material alone: minus infinity
    /// wherever a sweep or a failed bound leaves it unknown, and a cut's bound its minuend's.
    fn least(&self,b: V) -> f64 {
        match self.node.as_ref() {
            Node::Static(source) => source.bounds(b).map_or(f64::NEG_INFINITY,|x| x.bounds()[0]),
            Node::Swept(_) => f64::NEG_INFINITY,
            Node::Transformed {source,pose} => pose.inverse_point(b).map_or(f64::NEG_INFINITY,|q| source.least(q)),
            Node::Union(x,y,_) => x.least(b).min(y.least(b)),
            Node::Intersection(x,y) => x.least(b).max(y.least(b)),
            Node::Difference(x,_) => x.least(b),
        }
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
    pub fn union(self,other: Self) -> Result<Self,Error> { self.combine(other,|a,b| Node::Union(a,b,Default::default())) }
    pub fn intersection(self,other: Self) -> Result<Self,Error> { self.combine(other,Node::Intersection) }
    pub fn difference(self,other: Self) -> Result<Self,Error> { self.combine(other,Node::Difference) }

    /// A lower bound on the field at `p` from what is already known or cheap: a static operand's
    /// value, a sweep's distance outside its box or its coarse floor table's bound, composed by the
    /// Booleans (a cut's by its minuend's alone).
    fn low(&self,p: [f64;3]) -> f64 {
        match self.node.as_ref() {
            Node::Static(source) => source.value(p),
            Node::Swept(source) => source.clear_of(p).or_else(|| source.coarse_floor(p,SPREAD_COARSENING)).unwrap_or(f64::NEG_INFINITY),
            Node::Transformed {source,pose} => source.low(pose.inverse_point_mid(p)),
            Node::Union(a,b,_) => a.low(p).min(b.low(p)),
            Node::Intersection(a,b) => a.low(p).max(b.low(p)),
            Node::Difference(a,_) => a.low(p),
        }
    }

    /// The side of the least floor table cube of any sweep in the field (`SweptField::floor`), or
    /// infinity where there is none.
    fn floor_cube(&self) -> f64 {
        match self.node.as_ref() {
            Node::Static(_) => f64::INFINITY,
            Node::Swept(source) => source.cube_side(),
            Node::Transformed {source,..} => source.floor_cube(),
            Node::Union(a,b,_) | Node::Intersection(a,b) | Node::Difference(a,b) => a.floor_cube().min(b.floor_cube()),
        }
    }

    /// A union's flattened operands (`Operands`), worked out once.
    fn operands(&self) -> &Operands {
        let Node::Union(_,_,spread) = self.node.as_ref() else { unreachable!("the operands of a union") };
        spread.operands.get_or_init(|| {
            fn flatten(f: &MaterialField,first: usize,out: &mut Vec<(MaterialField,usize)>) {
                match f.node.as_ref() {
                    Node::Union(a,b,_) => { flatten(a,first,out); flatten(b,first+a.leaf_count(),out); }
                    _ => out.push((f.clone(),first)),
                }
            }
            let mut list = Vec::new();
            flatten(self,0,&mut list);
            // cells the side of the coarse floor cubes the bounds come from: a union's own
            // support is no measure, a sweep's reaching round the whole of its roll
            let cell = list.iter().map(|(f,_)| f.floor_cube()).fold(f64::INFINITY,f64::min)*SPREAD_COARSENING as f64;
            Operands {list,cell,leaves:self.leaf_count()}
        })
    }

    /// A union's operands that may decide it in the cell holding `p`, as indices into
    /// `operands().list` under a lower bound on each over the cell, lowest first (`Spread`).
    fn candidates(&self,p: [f64;3]) -> Arc<[(f64,u32)]> {
        let Node::Union(_,_,spread) = self.node.as_ref() else { unreachable!("the candidates of a union") };
        let Operands {list:operands,cell:h,..} = self.operands();
        let h = *h;
        if !(h > 0.) || !h.is_finite() {
            return (0..operands.len() as u32).map(|i| (f64::NEG_INFINITY,i)).collect();
        }
        let key = p.map(|x| (x/h).floor().clamp(i32::MIN as f64,i32::MAX as f64) as i32);
        if let Some(list) = spread.cells.lock().ok().and_then(|cells| cells.get(&key).cloned()) { return list; }
        let centre = key.map(|k| (k as f64+0.5)*h);
        let reach = 0.5*h*3f64.sqrt();
        let mut list: Vec<(f64,u32)> = operands.iter().enumerate().map(|(i,(f,_))| (f.low(centre)-reach,i as u32)).collect();
        list.sort_by(|a,b| a.0.total_cmp(&b.0));
        let list: Arc<[(f64,u32)]> = list.into();
        if let Ok(mut cells) = spread.cells.lock() { cells.insert(key,list.clone()); }
        list
    }

    /// Rigid maps the field reads the same under, from the first union of placed copies of one
    /// operand it holds (an indexed cut, `Spread`): the map carrying the first copy's placement to
    /// each other's. Kept only if the whole field reads alike at `samples` (a surface's vertices,
    /// say), at points spread over its support, and at all their images, since the rest of the
    /// body (a gear's blank) must be alike under it too: sampled, a reading and never a claim,
    /// and empty where anything disagrees.
    pub(crate) fn symmetries(&self,samples: &[[f64;3]]) -> Vec<Symmetry> {
        // the largest set of one operand's placements among a union's operands (the rest, if any,
        // must be alike under the maps too, which the samples check)
        fn copies(f: &MaterialField) -> Option<Vec<MotionBounds>> {
            match f.node.as_ref() {
                Node::Union(..) => {
                    let mut groups: Vec<(*const Node,Vec<MotionBounds>)> = Vec::new();
                    for (op,_) in &f.operands().list {
                        let Node::Transformed {source,pose} = op.node.as_ref() else { continue };
                        let key = Arc::as_ptr(&source.node);
                        match groups.iter_mut().find(|g| g.0 == key) {
                            Some(g) => g.1.push(*pose),
                            None => groups.push((key,vec![*pose])),
                        }
                    }
                    groups.into_iter().map(|g| g.1).max_by_key(Vec::len).filter(|g| g.len() > 1)
                }
                Node::Difference(a,b) | Node::Intersection(a,b) => copies(b).or_else(|| copies(a)),
                Node::Transformed {..} | Node::Static(_) | Node::Swept(_) => None,
            }
        }
        let Some(poses) = copies(self) else { return Vec::new() };
        let maps: Vec<Symmetry> = poses[1..].iter().map(|&to| Symmetry {from:poses[0],to}).collect();
        let Some(support) = self.tight_support(4,512).ok().flatten() else { return Vec::new() };
        let [lo,hi] = [0,1].map(|k| support.map(|x| x.bounds()[k]));
        let size = (0..3).map(|k| (hi[k]-lo[k]).powi(2)).sum::<f64>().sqrt();
        let mut rng = crate::rng::Rng::new(0x5e11);
        let samples: Vec<[f64;3]> = (0..SYMMETRY_SAMPLES).map(|_| std::array::from_fn(|k| rng.uniform(lo[k],hi[k])))
            .chain(samples.iter().copied()).collect();
        let options = |p: [f64;3]| super::ReadingOptions {relative:0.,..super::ReadingOptions::at(p)};
        // the first map at every sample, the rest at the points spread over the support and a few
        // of the caller's: a map carrying one tooth space onto the next already says most of it
        let alike = |(k,m): (usize,&Symmetry)| samples.iter().take(if k == 0 { usize::MAX } else { 2*SYMMETRY_SAMPLES }).all(|&p| {
            let q = m.apply(p);
            let (a,b) = (self.reading_with(p,&options(p),&mut 0).value,self.reading_with(q,&options(q),&mut 0).value);
            (a-b).abs() <= 1e-9*size
        });
        if maps.iter().enumerate().all(alike) { maps } else { Vec::new() }
    }

    /// A number with the field's sign at a point, in plain floating point: `SpatialField::value`
    /// for static material and `SweptField::side` for a sweep, composed by the Booleans (which
    /// keep signs). Which side a point is on, for a mesher; never an interval claim.
    pub fn side(&self,p: [f64;3]) -> f64 { self.side_with(p,None) }

    /// `side` with every sweep read from its adaptive distance field refined to `resolution`
    /// (`SweptField::cached`), where no bound settles it first: a mesher's sign, off the exact
    /// field's only within about the resolution's tolerance of the boundary.
    pub(crate) fn side_cached(&self,p: [f64;3],resolution: super::Resolution) -> f64 { self.side_with(p,Some(resolution)) }

    fn side_with(&self,p: [f64;3],cached: Option<super::Resolution>) -> f64 {
        match self.node.as_ref() {
            Node::Static(source) => source.value(p),
            Node::Swept(source) => match cached {
                None => source.side(p),
                Some(resolution) => {
                    // the box and the floor table settle most points before the octree is walked
                    if let Some(d) = source.clear_of(p) { return d; }
                    if let Some(low) = source.floor(p) { if low > 0. { return low; } }
                    source.cached(p,resolution).map_or_else(|| source.side(p),|(v,_)| v)
                }
            },
            Node::Transformed {source,pose} => source.side_with(pose.inverse_point_mid(p),cached),
            // One operand may settle the sign alone: then the other is not asked.
            Node::Union(..) => {
                // lowest bound first: past a bound at or above zero no operand can turn the sign
                let operands = &self.operands().list;
                let mut best = f64::INFINITY;
                for &(low,i) in self.candidates(p).iter() {
                    if low >= 0. { return best.min(low); }
                    let x = operands[i as usize].0.side_with(p,cached);
                    if x < 0. { return x; }
                    best = best.min(x);
                }
                best
            }
            Node::Intersection(a,b) => { let x = a.side_with(p,cached); if x >= 0. { x } else { x.max(b.side_with(p,cached)) } }
            Node::Difference(a,b) => { let x = a.side_with(p,cached); if x >= 0. { x } else { x.max(-b.side_with(p,cached)) } }
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
        self.reading_capped(p,options,next,hints,f64::INFINITY)
    }

    /// A reading exact wherever its value is below `cap`, and elsewhere a lower bound at least
    /// `cap` with no gradient worth reading — branch and bound over the term. A Boolean asks each
    /// operand only for what can decide it: a union's second operand below the first's value, a
    /// cut below the negated blank, both less twice the tie, so what is skipped neither decides
    /// nor ties. A sweep proves its bound by `SweptField::at_least`, a few evaluations, where its
    /// minimum is a whole search: a blank cut at twenty-four indices is decided by one cut.
    fn reading_capped(&self,p: [f64;3],options: &super::ReadingOptions,next: &mut usize,
        hints: &mut Vec<Option<f64>>,cap: f64) -> super::Reading {
        use super::reading::{higher,lower};
        // An operand left unread still takes its numbers.
        let skip = |node: &MaterialField,next: &mut usize,x: super::Reading| { *next += node.leaf_count(); x };
        match self.node.as_ref() {
            Node::Static(source) => source.reading(p,options,next),
            Node::Swept(source) => {
                // the sweep is keyed by the number its first source leaf will take
                let key = *next;
                if hints.len() <= key { hints.resize(key+1,None); }
                if let Some(resolution) = options.cached {
                    // a copy the floor table puts past the cap decides nothing, and is left unread
                    let low = source.clear_of(p).or_else(|| source.floor(p)).filter(|&low| low >= cap);
                    if let Some(value) = low {
                        *next += source.source().leaf_count();
                        return super::Reading::bound(value,key,hints[key]);
                    }
                    if let Some((value,gradient)) = source.cached(p,resolution) {
                        *next += source.source().leaf_count();
                        return super::Reading {gradient,..super::Reading::bound(value,key,hints[key])};
                    }
                }
                if cap.is_finite() {
                    if let Some(low) = source.at_least(p,cap,AT_LEAST_BUDGET) {
                        *next += source.source().leaf_count();
                        return super::Reading::bound(low,key,hints[key]);
                    }
                }
                let m = source.minimum_hinted(p,options.accuracy,options.relative,options.tie,hints[key],options.local);
                hints[key] = Some(m.time);
                // The tool's own reading at the roll time the sweep is least, turned into the
                // world: its value is the minimum, and so is its gradient (the envelope theorem).
                let pose = source.motion().pose_at(m.time).ok().map(|x| x.inverse());
                let Some(inverse) = pose else {
                    // numbered as every other exit numbers the sweep: all its source's leaves
                    *next += source.source().leaf_count();
                    return super::Reading {ambiguous:true,..super::Reading::bound(m.value,key,Some(m.time))};
                };
                let r = source.source().reading(inverse.point(p),options,next);
                super::Reading {value:m.value,gradient:inverse.gradient(r.gradient),time:Some(m.time),
                    ambiguous:r.ambiguous || m.tied,..r}
            }
            Node::Transformed {source,pose} => {
                let r = source.reading_capped(pose.inverse_point_mid(p),options,next,hints,cap);
                super::Reading {gradient:pose.gradient_mid(r.gradient),..r}
            }
            Node::Union(..) => {
                // operands in the order of their bounds, each read under the least reading so
                // far, and none once a bound passes it
                let Operands {list:operands,leaves,..} = self.operands();
                let first = *next;
                *next += leaves;
                let mut best: Option<super::Reading> = None;
                for &(low,i) in self.candidates(p).iter() {
                    let limit = cap.min(best.as_ref().map_or(f64::INFINITY,|r| r.value+2.*options.tie));
                    if low >= limit {
                        if best.is_none() { best = Some(super::Reading::bound(low,first,None)); }
                        break;
                    }
                    let (operand,start) = &operands[i as usize];
                    let r = operand.reading_capped(p,options,&mut (first+start),hints,limit);
                    best = Some(match best { None => r,Some(b) => lower(b,r,options.tie) });
                }
                best.unwrap_or(super::Reading::bound(f64::INFINITY,first,None))
            }
            Node::Intersection(a,b) => {
                let x = a.reading_capped(p,options,next,hints,cap);
                if x.value >= cap { return skip(b,next,x); }
                let y = b.reading_capped(p,options,next,hints,cap);
                higher(x,y,options.tie)
            }
            Node::Difference(a,b) => {
                let x = a.reading_capped(p,options,next,hints,cap);
                if x.value >= cap { return skip(b,next,x); }
                let y = b.reading_capped(p,options,next,hints,-x.value+2.*options.tie);
                higher(x,y.negated(),options.tie)
            }
        }
    }

    /// How many leaves the field has, as `reading` numbers them (a sweep's are its source's).
    pub fn leaf_count(&self) -> usize {
        match self.node.as_ref() {
            Node::Static(source) => source.leaf_count(),
            Node::Swept(source) => source.source().leaf_count(),
            Node::Transformed {source,..} => source.leaf_count(),
            Node::Union(a,b,_) | Node::Intersection(a,b) | Node::Difference(a,b) => a.leaf_count()+b.leaf_count(),
        }
    }

    /// The reading of one operand alone at a point: piece `piece` of leaf `target` (its smooth
    /// carrier, `SpatialField::leaf_reading`), with the transforms above it and the sign it
    /// enters the field with. With `piece` `WHOLE`, the leaf as the field sees it: a static leaf's
    /// own value, and for a leaf of a sweep's tool the whole tool's, whose reading names the leaf
    /// deciding it. A leaf of a sweep's source is that leaf swept, and
    /// near `time` — the least value over roll times about it, a window of a thousandth of the roll
    /// widened up to a sixty-fourth as it follows the minimum past an edge — so two contacts of one sweep are two
    /// operands, told apart by their times. Without a time the whole roll is searched coarsely
    /// first. `None` when `target` is not a leaf of this field.
    pub(crate) fn operand(&self,p: [f64;3],target: usize,piece: usize,time: Option<f64>,
        options: &super::ReadingOptions) -> Option<super::Reading> {
        self.operand_from(p,target,piece,time,options,&mut 0)
    }

    fn operand_from(&self,p: [f64;3],target: usize,piece: usize,time: Option<f64>,options: &super::ReadingOptions,
        next: &mut usize) -> Option<super::Reading> {
        match self.node.as_ref() {
            Node::Static(source) => source.leaf_reading(p,options,target,piece,next),
            Node::Swept(source) => {
                let first = *next;
                let count = source.source().leaf_count();
                *next += count;
                if target < first || target >= first+count { return None; }
                let [a,b] = source.domain().bounds();
                let pose = |t: f64| source.motion().pose_at(t).ok().map(|m| m.inverse());
                // A leaf's whole value inside a sweep is the tool's: the tool may be a Boolean, and
                // a leaf of it cuts only where the rest of the tool lets it. Its reading says
                // which leaf decides.
                let read = |q: [f64;3]| if piece == super::WHOLE { Some(source.source().reading(q,options,&mut first.clone())) }
                    else { source.source().leaf_reading(q,options,target,piece,&mut first.clone()) };
                // the search reads values alone, the gradient only where it ends
                let value = |t: f64| pose(t).and_then(|m| {
                    let q = m.point(p);
                    if piece == super::WHOLE { Some(source.source().value(q)) }
                    else { source.source().leaf_value(q,target,piece,&mut first.clone()) }
                }).unwrap_or(f64::INFINITY);
                // A time given is a continuation's, near where the least now is: a narrow window
                // about it, widened each time the least is found at its edge.
                let mut width = if time.is_some_and(f64::is_finite) { (b-a)/1024. } else { (b-a)/64. };
                // Where to look: about the time given, or about the least of a coarse sampling.
                let mut centre = time.filter(|t| t.is_finite()).unwrap_or_else(|| (0..=256)
                    .map(|k| a+(b-a)*k as f64/256.).min_by(|x,y| value(*x).total_cmp(&value(*y))).unwrap_or(a));
                let mut found = (value(centre),centre);
                for _ in 0..16 {
                    let (lo,hi) = ((centre-0.5*width).max(a),(centre+0.5*width).min(b));
                    // Brent to the accuracy asked, read off the parabola through its best points
                    found = super::swept::brent(&value,lo,hi,1e-12*(1.+hi.abs()),60,|_,slack| slack <= 0.25*options.accuracy);
                    // a least value at the roll's own end is no parabola's: read the end itself
                    for end in [a,b] {
                        if end >= lo && end <= hi { let v = value(end); if v < found.0 { found = (v,end); } }
                    }
                    // At the window's edge, and not the roll's: the minimum lies beyond; follow it.
                    let margin = 0.02*width;
                    let beyond = (found.1-lo < margin && lo > a) || (hi-found.1 < margin && hi < b);
                    if !beyond { break; }
                    centre = found.1+if found.1-lo < margin { -0.45*width } else { 0.45*width };
                    width = (2.*width).min((b-a)/64.);
                }
                let inverse = pose(found.1)?;
                let r = read(inverse.point(p))?;
                Some(super::Reading {gradient:inverse.gradient(r.gradient),time:Some(found.1),..r})
            }
            Node::Transformed {source,pose} => source.operand_from(pose.inverse_point_mid(p),target,piece,time,options,next)
                .map(|r| super::Reading {gradient:pose.gradient_mid(r.gradient),..r}),
            Node::Union(a,b,_) | Node::Intersection(a,b) => {
                let first = a.operand_from(p,target,piece,time,options,next);
                if first.is_some() { return first; }
                b.operand_from(p,target,piece,time,options,next)
            }
            Node::Difference(a,b) => {
                let first = a.operand_from(p,target,piece,time,options,next);
                if first.is_some() { return first; }
                b.operand_from(p,target,piece,time,options,next).map(super::Reading::negated)
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
            Node::Union(a,b,_) => min(self.evaluate(a,p,options,stop.operand(),cache,queries,observe)?,
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

/// Source evaluations a sweep may spend proving an operand cannot decide a reading before it is
/// read in full: about what a warm reading's local search costs.
const AT_LEAST_BUDGET: u64 = 16;

fn boxed(lo: [f64;3],hi: [f64;3]) -> Result<V,Error> {
    Ok([I::new(lo[0],hi[0])?,I::new(lo[1],hi[1])?,I::new(lo[2],hi[2])?])
}
