//! Immutable spatial composition of analytic material fields.
use super::{min,max,union_support,intersection_support,Error,I,V,RevolvedField,ExtrudedField,CanalField};
use crate::motion::{Family,MotionBounds};
use std::{collections::HashMap,sync::Arc};

type Cache = HashMap<(usize,[[u64;2];3]),I>;

#[derive(Clone,Debug)]
enum Node {
    Revolved(RevolvedField),
    Extruded(ExtrudedField),
    Canal(CanalField),
    Transformed {source:SpatialField,pose:MotionBounds},
    Union(SpatialField,SpatialField),
    Intersection(SpatialField,SpatialField),
    Difference(SpatialField,SpatialField),
}

/// A continuous one-Lipschitz field in world coordinates. Material means
/// closure({f<0}), not the entire zero set. Booleans need not remain distances.
/// Clones share immutable geometry. Spatial expression depth is limited to 64;
/// each revolved or extruded leaf also enforces the planar field's own depth limit.
/// Whether any node is reached by more than one path is decided once, when the
/// node is made: `bounds` is asked it on every roll of every sweep query. So is how many
/// leaves it has as a reading numbers them (`leaves`): a Boolean's second operand's are
/// numbered on from its first's, whatever point is read.
#[derive(Clone,Debug)]
pub struct SpatialField {node:Arc<Node>,depth:u8,shares:bool,leaves:usize}

impl From<RevolvedField> for SpatialField {
    fn from(source: RevolvedField) -> Self { Self {node:Arc::new(Node::Revolved(source)),depth:1,shares:false,leaves:1} }
}
impl From<ExtrudedField> for SpatialField {
    fn from(source: ExtrudedField) -> Self { Self {node:Arc::new(Node::Extruded(source)),depth:1,shares:false,leaves:1} }
}
impl From<CanalField> for SpatialField {
    fn from(source: CanalField) -> Self { Self {node:Arc::new(Node::Canal(source)),depth:1,shares:false,leaves:1} }
}

impl SpatialField {
    /// Conservative support of all regularized material. No query domain is
    /// supplied by the caller; unknown supports remain explicit.
    pub fn support_bounds(&self) -> Result<Option<V>,Error> {
        self.support(&mut HashMap::new())
    }
    fn support(&self,cache: &mut HashMap<usize,Option<V>>) -> Result<Option<V>,Error> {
        let key = Arc::as_ptr(&self.node) as usize;
        if let Some(value) = cache.get(&key) { return Ok(*value); }
        let value = match self.node.as_ref() {
            Node::Revolved(source) => source.support_bounds()?,
            Node::Extruded(source) => source.support_bounds()?,
            Node::Canal(source) => source.support_bounds()?,
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
        let leaves = match &node {
            Node::Revolved(_) | Node::Extruded(_) | Node::Canal(_) => 1,
            Node::Transformed {source,..} => source.leaves,
            Node::Union(a,b) | Node::Intersection(a,b) | Node::Difference(a,b) => a.leaves+b.leaves,
        };
        let mut field = Self {node:Arc::new(node),depth,shares:false,leaves};
        field.shares = field.shares_nodes();
        Ok(field)
    }

    /// Apply one fixed pose of the supplied solved motion. A point parameter
    /// defines one mathematical rigid transform, even though its stored matrix
    /// is enclosed by intervals. A range of poses is a different operation.
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
    pub fn difference(self,other: Self) -> Result<Self,Error> {
        // Regularized CSG identity A - (A - B) = A intersect B. The raw min/max
        // expression leaves zeros on A's discarded boundary, arbitrarily far
        // from the intersection. A subsequent sweep cannot separate those zeros
        // from material. Only shared immutable nodes establish this identity;
        // no sampled geometry or approximate coefficient comparison is involved.
        if let Node::Difference(a,b) = other.node.as_ref() {
            if Arc::ptr_eq(&self.node,&a.node) { return self.intersection(b.clone()); }
        }
        self.combine(other,Node::Difference)
    }

    /// The field at a point in plain floating point, from the midpoints of
    /// every enclosed coefficient and pose: a reading for tests against a
    /// tolerance far wider than the enclosures' widths (is this point of a
    /// face on the tool's boundary, which side of it is material), never an
    /// interval claim. Shared nodes are simply evaluated again.
    pub fn value(&self,p: [f64;3]) -> f64 {
        match self.node.as_ref() {
            Node::Revolved(source) => source.value(p),
            Node::Extruded(source) => source.value(p),
            Node::Canal(source) => source.value(p),
            Node::Transformed {source,pose} => source.value(pose.inverse_point_mid(p)),
            Node::Union(a,b) => a.value(p).min(b.value(p)),
            Node::Intersection(a,b) => a.value(p).max(b.value(p)),
            Node::Difference(a,b) => a.value(p).max(-b.value(p)),
        }
    }

    /// The value, gradient and deciding leaf at a point (`reading.rs`), to `q`'s step and tie,
    /// leaves numbered depth-first from `first`.
    pub fn reading(&self,p: [f64;3],q: &super::Query,first: usize) -> super::Reading {
        use super::reading::{higher,leaf,lower};
        match self.node.as_ref() {
            Node::Revolved(_) | Node::Extruded(_) | Node::Canal(_) => {
                let l = self.leaf().unwrap();
                leaf(|x| l.value(x),p,q.step,first,l.value_piece(p).1)
            }
            Node::Transformed {source,pose} => {
                let r = source.reading(pose.inverse_point_mid(p),q,first);
                super::Reading {gradient:pose.gradient_mid(r.gradient),..r}
            }
            Node::Union(a,b) => { let (x,y) = (a.reading(p,q,first),b.reading(p,q,first+a.leaves)); lower(x,y,q.tie) }
            Node::Intersection(a,b) => { let (x,y) = (a.reading(p,q,first),b.reading(p,q,first+a.leaves)); higher(x,y,q.tie) }
            Node::Difference(a,b) => {
                let (x,y) = (a.reading(p,q,first),b.reading(p,q,first+a.leaves));
                higher(x,y.negated(),q.tie)
            }
        }
    }

    /// How many leaves the field has, as `reading` numbers them.
    pub fn leaf_count(&self) -> usize { self.leaves }

    /// This node as a leaf, of either kind.
    fn leaf(&self) -> Option<Leaf<'_>> {
        match self.node.as_ref() {
            Node::Revolved(source) => Some(Leaf::Revolved(source)),
            Node::Extruded(source) => Some(Leaf::Extruded(source)),
            Node::Canal(source) => Some(Leaf::Canal(source)),
            _ => None,
        }
    }

    /// Leaf `target`, numbered as `reading` numbers them from `first`, with the sign it enters the
    /// field with: the point turned into its frame, and whether it is subtracted. Found by going
    /// down to the operand whose leaves hold the number; `None` when no leaf here has it.
    fn find_leaf(&self,p: [f64;3],target: usize,first: usize) -> Option<(Leaf<'_>,[f64;3],Vec<&MotionBounds>,bool)> {
        if target < first || target >= first+self.leaves { return None; }
        match self.node.as_ref() {
            Node::Revolved(_) | Node::Extruded(_) | Node::Canal(_) => Some((self.leaf().unwrap(),p,Vec::new(),false)),
            Node::Transformed {source,pose} => source.find_leaf(pose.inverse_point_mid(p),target,first)
                .map(|(l,x,mut poses,negated)| { poses.push(pose); (l,x,poses,negated) }),
            Node::Union(a,b) | Node::Intersection(a,b) => if target < first+a.leaves { a.find_leaf(p,target,first) }
                else { b.find_leaf(p,target,first+a.leaves) },
            Node::Difference(a,b) => if target < first+a.leaves { a.find_leaf(p,target,first) }
                else { b.find_leaf(p,target,first+a.leaves).map(|(l,x,poses,negated)| (l,x,poses,!negated)) },
        }
    }

    /// `leaf_reading`'s value alone, without the gradient's three further evaluations.
    pub(crate) fn leaf_value(&self,p: [f64;3],op: super::OperandId,first: usize) -> Option<f64> {
        let (l,x,_,negated) = self.find_leaf(p,op.leaf(),first)?;
        let v = match op.piece() {
            None => l.value(x),
            Some(piece) if piece >= l.piece_count() => return None,
            Some(piece) => l.carrier(x,piece).unwrap_or(f64::NAN),
        };
        Some(if negated { -v } else { v })
    }

    /// The reading of one operand alone at a point — its piece's whole smooth carrier, or for a
    /// whole leaf (`OperandId::whole`) the leaf itself — to `q`'s step, with the transforms above
    /// it and the sign it enters the field with (turned where it is subtracted), leaves numbered
    /// from `first` as `reading` numbers them; `None` when its leaf is not among this field's or
    /// has no such piece.
    pub(crate) fn leaf_reading(&self,p: [f64;3],q: &super::Query,op: super::OperandId,first: usize)
        -> Option<super::Reading> {
        use super::reading::leaf;
        let target = op.leaf();
        let (l,x,poses,negated) = self.find_leaf(p,target,first)?;
        let r = match op.piece() {
            None => leaf(|y| l.value(y),x,q.step,target,l.value_piece(x).1),
            Some(piece) if piece >= l.piece_count() => return None,
            Some(piece) => leaf(|y| l.carrier(y,piece).unwrap_or(f64::NAN),x,q.step,target,piece),
        };
        // the gradient turned back out through each transform, innermost first
        let r = poses.iter().fold(r,|r,pose| super::Reading {gradient:pose.gradient_mid(r.gradient),..r});
        Some(if negated { r.negated() } else { r })
    }

    /// Enclose the field over the complete world-coordinate box. Transform
    /// nodes query their source through the inverse pose, including interval
    /// axis normalization, trigonometry and all coordinate arithmetic.
    pub fn bounds(&self,p: V) -> Result<I,Error> {
        // A tree that shares no node needs no memo: the memo is a hash of the
        // whole box per node, which costs about what a leaf does, and most
        // tools are a handful of leaves under a Boolean or two.
        if self.shares { self.evaluate(p,&mut Some(Cache::new())) } else { self.evaluate(p,&mut None) }
    }

    /// Whether any node is reached by more than one path.
    fn shares_nodes(&self) -> bool {
        fn walk(f: &SpatialField,seen: &mut Vec<usize>) -> bool {
            let key = Arc::as_ptr(&f.node) as usize;
            if seen.contains(&key) { return true; }
            seen.push(key);
            match f.node.as_ref() {
                Node::Revolved(_) | Node::Extruded(_) | Node::Canal(_) => false,
                Node::Transformed {source,..} => walk(source,seen),
                Node::Union(a,b) | Node::Intersection(a,b) | Node::Difference(a,b) => walk(a,seen) || walk(b,seen),
            }
        }
        walk(self,&mut Vec::new())
    }

    fn evaluate(&self,p: V,cache: &mut Option<Cache>) -> Result<I,Error> {
        // Shared Boolean subexpressions must not expand exponentially. Identity
        // is valid for this one query, whose immutable root owns all nodes.
        // The box is part of the key: transforms can query the same source at
        // different coordinates. No cache survives a bounds call or source edit.
        let key = (Arc::as_ptr(&self.node) as usize,super::memo::box_bits(&p));
        if let Some(cache) = cache { if let Some(value) = cache.get(&key) { return Ok(*value); } }
        let value = match self.node.as_ref() {
            Node::Revolved(source) => source.bounds(p),
            Node::Extruded(source) => source.bounds(p),
            Node::Canal(source) => source.bounds(p),
            Node::Transformed {source,pose} => source.evaluate(pose.inverse_point(p)?,cache),
            Node::Union(a,b) => Ok(min(a.evaluate(p,cache)?,b.evaluate(p,cache)?)),
            Node::Intersection(a,b) => Ok(max(a.evaluate(p,cache)?,b.evaluate(p,cache)?)),
            Node::Difference(a,b) => Ok(max(a.evaluate(p,cache)?,b.evaluate(p,cache)?.neg())),
        }?;
        if let Some(cache) = cache { cache.insert(key,value); }
        Ok(value)
    }
}

/// A leaf of either kind, read alike.
#[derive(Clone,Copy)]
enum Leaf<'a> { Revolved(&'a RevolvedField),Extruded(&'a ExtrudedField),Canal(&'a CanalField) }

impl Leaf<'_> {
    fn value(self,p: [f64;3]) -> f64 {
        match self { Leaf::Revolved(l) => l.value(p),Leaf::Extruded(l) => l.value(p),Leaf::Canal(l) => l.value(p) }
    }
    fn value_piece(self,p: [f64;3]) -> (f64,usize) {
        match self { Leaf::Revolved(l) => l.value_piece(p),Leaf::Extruded(l) => l.value_piece(p),Leaf::Canal(l) => (l.value(p),0) }
    }
    /// (a canal is one piece: its creases are not traced)
    fn piece_count(self) -> usize {
        match self { Leaf::Revolved(l) => l.piece_count(),Leaf::Extruded(l) => l.piece_count(),Leaf::Canal(_) => 1 }
    }
    fn carrier(self,p: [f64;3],piece: usize) -> Option<f64> {
        match self { Leaf::Revolved(l) => l.carrier(p,piece),Leaf::Extruded(l) => l.carrier(p,piece),Leaf::Canal(l) => Some(l.value(p)) }
    }
}
