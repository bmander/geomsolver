//! A material field's term compiled for reading (`tape.rs`'s shape): every distinct node one op,
//! its operands named by earlier index, and each op knowing how many leaves a reading counts under
//! it — so the leaf a reading names is its op's first leaf plus an offset fixed here, and one
//! operand is found by going down to the op whose leaves hold its number, never by counting.
//!
//! Most readings of a term are one fold, bottom up: what a leaf reads, how a fixed pose carries the
//! input into its source's frame, how the Booleans combine (`Fold`). The support, the lower bounds
//! and the interval enclosure are each one; the sign and the reading, which skip what cannot
//! decide them, walk the ops themselves (`MaterialField::sign`, `MaterialField::below`).
use super::{MaterialField,Node,Spread,SpatialField,SweptField};
use crate::motion::MotionBounds;
use std::{collections::HashMap,sync::Arc};

/// An op's index in its plan.
pub(super) type OpId = u32;

pub(super) struct Plan { pub ops: Vec<Op> }

pub(super) struct Op { pub kind: Kind,pub leaves: usize }

pub(super) enum Kind {
    Static(SpatialField),
    Swept(SweptField),
    Transformed {source: OpId,pose: MotionBounds},
    /// A union: its two operands, and its operands with the unions under it flattened, each with
    /// the number its first leaf takes counted from the union's own first (`Spread`).
    Union {a: OpId,b: OpId,flat: Vec<(OpId,usize)>,spread: Arc<Spread>},
    Intersection(OpId,OpId),
    Difference(OpId,OpId),
}

#[derive(Clone,Copy,PartialEq)]
pub(super) enum Bool { Union,Intersection,Difference }

impl Plan {
    /// The field's term compiled: shared nodes once (a gear's indexed cut is one sweep placed many
    /// times), children before parents, the root last.
    pub fn compile(root: &MaterialField) -> Self {
        fn walk(f: &MaterialField,ops: &mut Vec<Op>,seen: &mut HashMap<usize,OpId>) -> OpId {
            // looked up by node identity, never iterated
            let key = Arc::as_ptr(&f.node) as usize;
            if let Some(&i) = seen.get(&key) { return i; }
            let kind = match f.node.as_ref() {
                Node::Static(source) => Kind::Static(source.clone()),
                Node::Swept(source) => Kind::Swept(source.clone()),
                Node::Transformed {source,pose} => Kind::Transformed {source:walk(source,ops,seen),pose:*pose},
                Node::Union(a,b,spread) => {
                    let (a,b) = (walk(a,ops,seen),walk(b,ops,seen));
                    let mut flat = Vec::new();
                    flatten(ops,a,0,&mut flat);
                    flatten(ops,b,ops[a as usize].leaves,&mut flat);
                    Kind::Union {a,b,flat,spread:spread.clone()}
                }
                Node::Intersection(a,b) => Kind::Intersection(walk(a,ops,seen),walk(b,ops,seen)),
                Node::Difference(a,b) => Kind::Difference(walk(a,ops,seen),walk(b,ops,seen)),
            };
            let leaves = match &kind {
                Kind::Static(source) => source.leaf_count(),
                Kind::Swept(source) => source.source().leaf_count(),
                Kind::Transformed {source,..} => ops[*source as usize].leaves,
                Kind::Union {a,b,..} | Kind::Intersection(a,b) | Kind::Difference(a,b) =>
                    ops[*a as usize].leaves+ops[*b as usize].leaves,
            };
            ops.push(Op {kind,leaves});
            let i = (ops.len()-1) as OpId;
            seen.insert(key,i);
            i
        }
        fn flatten(ops: &[Op],i: OpId,first: usize,out: &mut Vec<(OpId,usize)>) {
            match &ops[i as usize].kind {
                Kind::Union {a,b,..} => {
                    flatten(ops,*a,first,out);
                    flatten(ops,*b,first+ops[*a as usize].leaves,out);
                }
                _ => out.push((i,first)),
            }
        }
        let mut ops = Vec::new();
        walk(root,&mut ops,&mut HashMap::new());
        Self {ops}
    }

    pub fn root(&self) -> OpId { (self.ops.len()-1) as OpId }
    pub fn op(&self,i: OpId) -> &Op { &self.ops[i as usize] }

    /// A fold of the term from op `i` (`Fold`), operands in order, the second of a Boolean left
    /// unread where the first settles it or the fold does not read what a cut takes away.
    pub fn fold<F: Fold>(&self,i: OpId,x: F::In,f: &mut F) -> F::Out {
        if let Some(v) = f.recall(i,&x) { return v; }
        let v = match &self.op(i).kind {
            Kind::Static(source) => f.leaf(source,x),
            Kind::Swept(source) => f.sweep(i,source,x),
            Kind::Transformed {source,pose} => match f.into(pose,x) {
                Ok(y) => { let v = self.fold(*source,y,f); f.out(pose,v) }
                Err(v) => v,
            },
            Kind::Union {a,b,..} => self.pair(Bool::Union,*a,*b,x,f),
            Kind::Intersection(a,b) => self.pair(Bool::Intersection,*a,*b,x,f),
            Kind::Difference(a,b) => self.pair(Bool::Difference,*a,*b,x,f),
        };
        f.keep(i,x,v.clone());
        v
    }

    fn pair<F: Fold>(&self,op: Bool,a: OpId,b: OpId,x: F::In,f: &mut F) -> F::Out {
        let u = self.fold(a,f.operand(op,false,x),f);
        if f.settled(&u) || op == Bool::Difference && !f.reads_cuts() { return f.combine(op,u,None); }
        let v = self.fold(b,f.operand(op,true,x),f);
        f.combine(op,u,Some(v))
    }
}

/// One bottom-up reading of the term (`Plan::fold`).
pub(super) trait Fold {
    type In: Copy;
    type Out: Clone;
    fn leaf(&mut self,source: &SpatialField,x: Self::In) -> Self::Out;
    /// A sweep, op `op` of its plan.
    fn sweep(&mut self,op: OpId,source: &SweptField,x: Self::In) -> Self::Out;
    /// A fixed pose: the input carried into its source's frame, or the answer where it cannot be.
    fn into(&mut self,_pose: &MotionBounds,x: Self::In) -> Result<Self::In,Self::Out> { Ok(x) }
    /// A source's answer carried out through its pose; a value is left as it is.
    fn out(&mut self,_pose: &MotionBounds,v: Self::Out) -> Self::Out { v }
    /// What a Boolean's first or second operand is asked.
    fn operand(&mut self,_op: Bool,_second: bool,x: Self::In) -> Self::In { x }
    /// The Boolean of its operands' answers, the second none where it was left unread.
    fn combine(&mut self,op: Bool,a: Self::Out,b: Option<Self::Out>) -> Self::Out;
    /// Whether the fold reads a cut's subtrahend at all: a bound on material from the rest does not.
    fn reads_cuts(&self) -> bool { true }
    /// An answer that settles its Boolean alone: an error.
    fn settled(&self,_v: &Self::Out) -> bool { false }
    /// A memo, for folds that keep one: an answer already worked out for this op and input.
    fn recall(&mut self,_op: OpId,_x: &Self::In) -> Option<Self::Out> { None }
    fn keep(&mut self,_op: OpId,_x: Self::In,_v: Self::Out) {}
}
