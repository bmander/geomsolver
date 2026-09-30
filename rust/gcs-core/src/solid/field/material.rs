//! Composition of static and continuously swept material fields.
use super::{min,max,union_support,intersection_support,Error,I,V,SpatialField,SweptField,SweepEvaluator,SweepError};
use super::{OperandId,Query,Reading,Source,Want};
use crate::motion::{Family,MotionBounds};
use std::sync::Arc;
mod evaluator;
mod plan;
mod symmetry;
use plan::{Bool,Fold,Kind,OpId,Plan};
pub use evaluator::{MaterialEvaluator,MaterialBounds,MaterialSweepQuery};
pub use symmetry::Symmetry;

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

/// Filled as cells are asked, the operands that may decide a union (its flattened operands,
/// `plan::Kind::Union`) in each cell of a grid: a lower bound on each over the cell, lowest first
/// (`MaterialField::low`). An indexed cut is a union of as many placed copies as the gear has
/// teeth, and a point near one tooth space is decided by two or three of them; read in turn, every
/// copy costs a transform and a table lookup at every one of a mesher's queries. Kept on the node,
/// so every field compiled over it shares the table; the side of its cells is the least coarse
/// floor cube of any sweep among the operands, worked out once.
#[derive(Debug,Default)]
struct Spread {
    cell: std::sync::OnceLock<f64>,
    cells: super::memo::Memo<[i32;3],Arc<[(f64,u32)]>>,
}

/// Immutable one-Lipschitz material field, including completed swept operands.
/// Material is closure({f<0}); zero alone does not certify a boundary. Fixed
/// poses and Booleans compose without discretizing the generating motion.
/// Expression depth is limited to 64, separately from each static source graph.
/// A sweep's source is still a SpatialField; nested continuous sweeps are not
/// represented by silently sampling or fixing an inner sweep. Read through its term compiled
/// once (`Plan`), which its clones share.
#[derive(Clone,Debug)]
pub struct MaterialField {node:Arc<Node>,depth:u8,plan:Arc<std::sync::OnceLock<Plan>>}

impl std::fmt::Debug for Plan {
    fn fmt(&self,f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { write!(f,"Plan({} ops)",self.ops.len()) }
}

impl From<SpatialField> for MaterialField {
    fn from(source: SpatialField) -> Self { Self::leaf(Node::Static(source)) }
}
impl From<SweptField> for MaterialField {
    fn from(source: SweptField) -> Self { Self::leaf(Node::Swept(source)) }
}

/// `MaterialField::support_bounds`: a box's image through a pose, memoised per op.
struct Support(Vec<Option<Result<Option<V>,Error>>>);
impl Fold for Support {
    type In = ();
    type Out = Result<Option<V>,Error>;
    fn leaf(&mut self,source: &SpatialField,_: ()) -> Self::Out { source.support_bounds() }
    fn sweep(&mut self,_: OpId,source: &SweptField,_: ()) -> Self::Out { source.support_bounds() }
    fn out(&mut self,pose: &MotionBounds,v: Self::Out) -> Self::Out { v?.map(|b| pose.point(b)).transpose() }
    fn combine(&mut self,op: Bool,a: Self::Out,b: Option<Self::Out>) -> Self::Out {
        Ok(match (op,b) {
            (Bool::Union,Some(b)) => union_support(a?,b?),
            (Bool::Intersection,Some(b)) => intersection_support(a?,b?),
            _ => a?,
        })
    }
    fn reads_cuts(&self) -> bool { false }
    fn settled(&self,v: &Self::Out) -> bool { v.is_err() }
    fn recall(&mut self,op: OpId,_: &()) -> Option<Self::Out> { self.0[op as usize] }
    fn keep(&mut self,op: OpId,_: (),v: Self::Out) { self.0[op as usize] = Some(v); }
}

/// `MaterialField::least`: a lower bound over a box from static material alone.
struct Least;
impl Fold for Least {
    type In = V;
    type Out = f64;
    fn leaf(&mut self,source: &SpatialField,b: V) -> f64 { source.bounds(b).map_or(f64::NEG_INFINITY,|x| x.bounds()[0]) }
    fn sweep(&mut self,_: OpId,_: &SweptField,_: V) -> f64 { f64::NEG_INFINITY }
    fn into(&mut self,pose: &MotionBounds,b: V) -> Result<V,f64> { pose.inverse_point(b).map_err(|_| f64::NEG_INFINITY) }
    fn combine(&mut self,op: Bool,a: f64,b: Option<f64>) -> f64 { bound(op,a,b) }
    fn reads_cuts(&self) -> bool { false }
}

/// `MaterialField::low`: a lower bound at a point from what is known or cheap.
struct Low;
impl Fold for Low {
    type In = [f64;3];
    type Out = f64;
    fn leaf(&mut self,source: &SpatialField,p: [f64;3]) -> f64 { source.value(p) }
    fn sweep(&mut self,_: OpId,source: &SweptField,p: [f64;3]) -> f64 {
        source.clear_of(p).or_else(|| source.coarse_floor(p,SPREAD_COARSENING)).unwrap_or(f64::NEG_INFINITY)
    }
    fn into(&mut self,pose: &MotionBounds,p: [f64;3]) -> Result<[f64;3],f64> { Ok(pose.inverse_point_mid(p)) }
    fn combine(&mut self,op: Bool,a: f64,b: Option<f64>) -> f64 { bound(op,a,b) }
    fn reads_cuts(&self) -> bool { false }
}

/// A lower bound's Booleans: a cut's is its minuend's.
fn bound(op: Bool,a: f64,b: Option<f64>) -> f64 {
    match (op,b) { (Bool::Union,Some(b)) => a.min(b),(Bool::Intersection,Some(b)) => a.max(b),_ => a }
}

/// `MaterialField`'s least floor-table cube: the side of any sweep's, or infinity.
struct FloorCube;
impl Fold for FloorCube {
    type In = ();
    type Out = f64;
    fn leaf(&mut self,_: &SpatialField,_: ()) -> f64 { f64::INFINITY }
    fn sweep(&mut self,_: OpId,source: &SweptField,_: ()) -> f64 { source.cube_side() }
    fn combine(&mut self,_: Bool,a: f64,b: Option<f64>) -> f64 { b.map_or(a,|b| a.min(b)) }
}

impl MaterialField {
    fn leaf(node: Node) -> Self { Self {node:Arc::new(node),depth:1,plan:Default::default()} }

    /// The term compiled, once for this field and its clones.
    fn plan(&self) -> &Plan { self.plan.get_or_init(|| Plan::compile(self)) }

    /// A construction-derived finite box containing all regularized material.
    /// `None` means this construction has no derived finite support, not that
    /// it is necessarily unbounded. No samples or caller-provided crop enter.
    pub fn support_bounds(&self) -> Result<Option<V>,Error> {
        let plan = self.plan();
        plan.fold(plan.root(),(),&mut Support(vec![None;plan.ops.len()]))
    }
    /// A box holding all the material, tighter than `support_bounds`: that box split up to
    /// `depth` times, keeping the parts the field may be negative in, and their hull. A cut is
    /// left out of the test, since removing material cannot put any outside the rest, and so is a
    /// sweep standing for material (it may be anywhere its support says); only static operands
    /// prune. At most `cap` boxes are split a level; past that the hull stands as it is. Sizing,
    /// never a claim: the hull is a box the material is in, found by interval bounds.
    pub(crate) fn tight_support(&self,depth: usize,cap: usize) -> Result<Option<V>,Error> {
        let Some(support) = self.support_bounds()? else { return Ok(None) };
        let plan = self.plan();
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
                    // a lower bound on the field over the box from its static material alone:
                    // minus infinity wherever a sweep or a failed bound leaves it unknown
                    if plan.fold(plan.root(),part,&mut Least) <= 0. { next.push(part); }
                }
            }
            if next.is_empty() { return Ok(None); }
            boxes = next;
        }
        let lo: [f64;3] = std::array::from_fn(|k| boxes.iter().map(|b| b[k].bounds()[0]).fold(f64::INFINITY,f64::min));
        let hi: [f64;3] = std::array::from_fn(|k| boxes.iter().map(|b| b[k].bounds()[1]).fold(f64::NEG_INFINITY,f64::max));
        Ok(Some(boxed(lo,hi)?))
    }

    fn node(node: Node,depth: u8) -> Result<Self,Error> {
        if depth > 64 { return Err(Error::OutsideDomain); }
        Ok(Self {node:Arc::new(node),depth,plan:Default::default()})
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

    /// A union's operands that may decide it in the cell holding `p`, as indices into its
    /// flattened operands under a lower bound on each over the cell, lowest first (`Spread`): a
    /// lower bound at the cell's centre from what is already known or cheap (`Low` — a static
    /// operand's value, a sweep's distance outside its box or its coarse floor table's bound,
    /// composed by the Booleans, a cut's by its minuend's alone), less the cell's reach.
    fn candidates(plan: &Plan,flat: &[(OpId,usize)],spread: &Spread,p: [f64;3]) -> Arc<[(f64,u32)]> {
        // cells the side of the coarse floor cubes the bounds come from: a union's own
        // support is no measure, a sweep's reaching round the whole of its roll
        let cell = *spread.cell.get_or_init(|| flat.iter().map(|&(i,_)| plan.fold(i,(),&mut FloorCube))
            .fold(f64::INFINITY,f64::min)*SPREAD_COARSENING as f64);
        let grid = super::memo::Grid(cell);
        if !grid.usable() {
            return (0..flat.len() as u32).map(|i| (f64::NEG_INFINITY,i)).collect();
        }
        let key = grid.key(p);
        if let Some(list) = spread.cells.lock().get(&key).cloned() { return list; }
        let centre = grid.centre(key);
        let reach = 0.5*grid.0*3f64.sqrt();
        let mut list: Vec<(f64,u32)> = flat.iter().enumerate().map(|(k,&(i,_))| (plan.fold(i,centre,&mut Low)-reach,k as u32)).collect();
        list.sort_by(|a,b| a.0.total_cmp(&b.0));
        let list: Arc<[(f64,u32)]> = list.into();
        spread.cells.lock().insert(key,list.clone());
        list
    }

    /// A point query (`Query`): the field's sign, or its value, gradient and deciding operand. The
    /// hints of a warm query (`Source::Warm`) are left holding this point's contact times.
    pub fn query(&self,p: [f64;3],q: &mut Query) -> Reading {
        let plan = self.plan();
        match q.want {
            Want::Sign => Reading::bound(Self::sign(plan,plan.root(),p,q)),
            Want::Reading => {
                let mut hints = match &mut q.source { Source::Warm {hints,..} => std::mem::take(hints),_ => Vec::new() };
                let r = Self::below(plan,plan.root(),p,q,0,&mut hints,f64::INFINITY);
                if let Source::Warm {hints:kept,..} = &mut q.source { *kept = hints; }
                r
            }
        }
    }

    /// The field's sign at a point, every sweep searched (`Query::sign`).
    pub fn side(&self,p: [f64;3]) -> f64 { self.query(p,&mut Query::sign()).value }

    /// The field's value, gradient and deciding operand at a point, to tolerances scaled to it
    /// (`Query::at`).
    pub fn reading(&self,p: [f64;3]) -> Reading { self.query(p,&mut Query::at(p)) }

    /// A number with the field's sign at a point, in plain floating point: `SpatialField::value`
    /// for static material and a sweep's sign (`SweptField::query`), composed by the Booleans
    /// (which keep signs). Which side a point is on, for a mesher; never an interval claim.
    fn sign(plan: &Plan,i: OpId,p: [f64;3],q: &Query) -> f64 {
        match &plan.op(i).kind {
            Kind::Static(source) => source.value(p),
            Kind::Swept(source) => source.query(p,q,f64::INFINITY,&mut None,0).value,
            Kind::Transformed {source,pose} => Self::sign(plan,*source,pose.inverse_point_mid(p),q),
            // One operand may settle the sign alone: then the other is not asked.
            Kind::Union {flat,spread,..} => {
                // lowest bound first: past a bound at or above zero no operand can turn the sign
                let mut best = f64::INFINITY;
                for &(low,k) in Self::candidates(plan,flat,spread,p).iter() {
                    if low >= 0. { return best.min(low); }
                    let x = Self::sign(plan,flat[k as usize].0,p,q);
                    if x < 0. { return x; }
                    best = best.min(x);
                }
                best
            }
            Kind::Intersection(a,b) => { let x = Self::sign(plan,*a,p,q); if x >= 0. { x } else { x.max(Self::sign(plan,*b,p,q)) } }
            Kind::Difference(a,b) => { let x = Self::sign(plan,*a,p,q); if x >= 0. { x } else { x.max(-Self::sign(plan,*b,p,q)) } }
        }
    }

    /// A reading exact wherever its value is below `cap`, and elsewhere a lower bound at least
    /// `cap` with no gradient worth reading and no operand — branch and bound over the term, the
    /// op's leaves numbered from `first`, `hints[k]` the contact time of the sweep whose first
    /// source leaf is numbered `k`. A Boolean asks each operand only for what can decide it: a
    /// union's second operand below the first's value, a cut below the negated blank, both less
    /// twice the tie, so what is skipped neither decides nor ties. A sweep proves such a bound
    /// cheaply (`SweptField::query`), where its minimum is a whole search: a blank cut at
    /// twenty-four indices is decided by one cut.
    fn below(plan: &Plan,i: OpId,p: [f64;3],q: &Query,first: usize,hints: &mut Vec<Option<f64>>,cap: f64) -> Reading {
        use super::reading::{higher,lower};
        match &plan.op(i).kind {
            Kind::Static(source) => source.reading(p,q,first),
            Kind::Swept(source) => {
                // the sweep is keyed by the number its first source leaf takes
                if hints.len() <= first { hints.resize(first+1,None); }
                source.query(p,q,cap,&mut hints[first],first)
            }
            Kind::Transformed {source,pose} => {
                let r = Self::below(plan,*source,pose.inverse_point_mid(p),q,first,hints,cap);
                Reading {gradient:pose.gradient_mid(r.gradient),..r}
            }
            Kind::Union {flat,spread,..} => {
                // operands in the order of their bounds, each read under the least reading so
                // far, and none once a bound passes it
                let mut best: Option<Reading> = None;
                for &(low,k) in Self::candidates(plan,flat,spread,p).iter() {
                    let limit = cap.min(best.as_ref().map_or(f64::INFINITY,|r| r.value+2.*q.tie));
                    if low >= limit {
                        if best.is_none() { best = Some(Reading::bound(low)); }
                        break;
                    }
                    let (operand,start) = flat[k as usize];
                    let r = Self::below(plan,operand,p,q,first+start,hints,limit);
                    best = Some(match best { None => r,Some(b) => lower(b,r,q.tie) });
                }
                best.unwrap_or(Reading::bound(f64::INFINITY))
            }
            // an operand left unread still takes its numbers: the second's are counted from the
            // first's whether it is read or not
            Kind::Intersection(a,b) => {
                let x = Self::below(plan,*a,p,q,first,hints,cap);
                if x.value >= cap { return x; }
                let y = Self::below(plan,*b,p,q,first+plan.op(*a).leaves,hints,cap);
                higher(x,y,q.tie)
            }
            Kind::Difference(a,b) => {
                let x = Self::below(plan,*a,p,q,first,hints,cap);
                if x.value >= cap { return x; }
                let y = Self::below(plan,*b,p,q,first+plan.op(*a).leaves,hints,-x.value+2.*q.tie);
                higher(x,y.negated(),q.tie)
            }
        }
    }

    /// The field with the cut operands `keep` refuses left out: every operand of a union that is
    /// subtracted (through the Booleans above it, a difference's minuend or either side of an
    /// intersection), asked in the order of the term, `keep` told each one. What is left out may
    /// only ever have added removal, so the field left can only be lower (more material), and it is
    /// the same wherever the operands left out are positive: a caller leaves out only operands it has
    /// proved positive over the region it reads (`solid::agreement`, over one sector of an indexed
    /// body). Where every operand of a cut is left out, the difference is its minuend.
    pub fn without_cuts(&self,keep: &mut dyn FnMut(&MaterialField) -> bool) -> Result<MaterialField,Error> {
        fn cut(f: &MaterialField,keep: &mut dyn FnMut(&MaterialField) -> bool) -> Result<Option<MaterialField>,Error> {
            match f.node.as_ref() {
                Node::Union(a,b,_) => Ok(match (cut(a,keep)?,cut(b,keep)?) {
                    (Some(a),Some(b)) => Some(a.union(b)?),
                    (Some(a),None) | (None,Some(a)) => Some(a),
                    (None,None) => None,
                }),
                _ => Ok(keep(f).then(|| f.clone())),
            }
        }
        match self.node.as_ref() {
            Node::Difference(a,b) => {
                let a = a.without_cuts(keep)?;
                match cut(b,keep)? { Some(b) => a.difference(b),None => Ok(a) }
            }
            Node::Intersection(a,b) => a.without_cuts(keep)?.intersection(b.without_cuts(keep)?),
            _ => Ok(self.clone()),
        }
    }

    /// The cut operands `without_cuts` asks about, in its order.
    pub fn cut_operands(&self) -> Vec<MaterialField> {
        let mut all = Vec::new();
        let _ = self.without_cuts(&mut |f| { all.push(f.clone()); true });
        all
    }

    /// How many leaves the field has, as a reading numbers them (a sweep's are its source's).
    pub fn leaf_count(&self) -> usize { self.plan().op(self.plan().root()).leaves }

    /// The reading of one operand alone at a point: its piece of its leaf (that piece's smooth
    /// carrier, `SpatialField::leaf_reading`), with the transforms above it and the sign it enters
    /// the field with. For a whole leaf (`OperandId::whole`), the leaf as the field sees it: a
    /// static leaf's own value, and for a leaf of a sweep's tool the whole tool's, whose reading
    /// names the leaf deciding it. A leaf of a sweep's source is that leaf swept, and near the
    /// operand's time — the least value over roll times about it, a window of a thousandth of the
    /// roll widened up to a sixty-fourth as it follows the minimum past an edge — so two contacts
    /// of one sweep are two operands, told apart by their times. Without a time the whole roll is
    /// searched coarsely first. Read to `q`'s accuracy and step, whatever it wants; `None` when the
    /// operand's leaf is not a leaf of this field. Found by going down to the op whose leaves hold
    /// its number.
    pub(crate) fn operand(&self,p: [f64;3],op: OperandId,q: &Query) -> Option<Reading> {
        let plan = self.plan();
        Self::operand_at(plan,plan.root(),p,op,q,0)
    }

    fn operand_at(plan: &Plan,i: OpId,p: [f64;3],op: OperandId,q: &Query,first: usize) -> Option<Reading> {
        let target = op.leaf();
        if target < first || target >= first+plan.op(i).leaves { return None; }
        // the second of two operands, its leaves numbered on from the first's
        let second = |a: OpId| target >= first+plan.op(a).leaves;
        match &plan.op(i).kind {
            Kind::Static(source) => source.leaf_reading(p,q,op,first),
            Kind::Swept(source) => {
                let [a,b] = source.domain().bounds();
                let pose = |t: f64| source.motion().pose_at(t).ok().map(|m| m.inverse());
                let whole = op.piece().is_none();
                // A leaf's whole value inside a sweep is the tool's: the tool may be a Boolean, and
                // a leaf of it cuts only where the rest of the tool lets it. Its reading says
                // which leaf decides.
                let read = |x: [f64;3]| if whole { Some(source.source().reading(x,q,first)) }
                    else { source.source().leaf_reading(x,q,op,first) };
                // the search reads values alone, the gradient only where it ends
                let value = |t: f64| pose(t).and_then(|m| {
                    let x = m.point(p);
                    if whole { Some(source.source().value(x)) } else { source.source().leaf_value(x,op,first) }
                }).unwrap_or(f64::INFINITY);
                // A time given is a continuation's, near where the least now is: a narrow window
                // about it, widened each time the least is found at its edge.
                let found = super::swept::follow(&value,a,b,op.time(),q.accuracy);
                let inverse = pose(found.1)?;
                let r = read(inverse.point(p))?;
                Some(Reading {gradient:inverse.gradient(r.gradient),operand:r.operand.map(|o| o.at_time(Some(found.1))),..r})
            }
            Kind::Transformed {source,pose} => Self::operand_at(plan,*source,pose.inverse_point_mid(p),op,q,first)
                .map(|r| Reading {gradient:pose.gradient_mid(r.gradient),..r}),
            Kind::Union {a,b,..} | Kind::Intersection(a,b) => if second(*a) {
                Self::operand_at(plan,*b,p,op,q,first+plan.op(*a).leaves)
            } else { Self::operand_at(plan,*a,p,op,q,first) },
            Kind::Difference(a,b) => if second(*a) {
                Self::operand_at(plan,*b,p,op,q,first+plan.op(*a).leaves).map(Reading::negated)
            } else { Self::operand_at(plan,*a,p,op,q,first) },
        }
    }
}

fn boxed(lo: [f64;3],hi: [f64;3]) -> Result<V,Error> {
    Ok([I::new(lo[0],hi[0])?,I::new(lo[1],hi[1])?,I::new(lo[2],hi[2])?])
}
