//! Immutable spatial composition of analytic material fields.
use super::{min,max,union_support,intersection_support,Error,I,V,RevolvedField,ExtrudedField};
use crate::motion::{Family,MotionBounds};
use std::{collections::HashMap,sync::Arc};

type Cache = HashMap<(usize,[[u64;2];3]),I>;

#[derive(Clone,Debug)]
enum Node {
    Revolved(RevolvedField),
    Extruded(ExtrudedField),
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
/// node is made: `bounds` is asked it on every roll of every sweep query.
#[derive(Clone,Debug)]
pub struct SpatialField {node:Arc<Node>,depth:u8,shares:bool}

impl From<RevolvedField> for SpatialField {
    fn from(source: RevolvedField) -> Self { Self {node:Arc::new(Node::Revolved(source)),depth:1,shares:false} }
}
impl From<ExtrudedField> for SpatialField {
    fn from(source: ExtrudedField) -> Self { Self {node:Arc::new(Node::Extruded(source)),depth:1,shares:false} }
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
        let mut field = Self {node:Arc::new(node),depth,shares:false};
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

    /// Enclose the field over the complete world-coordinate box. Transform
    /// nodes query their source through the inverse pose, including interval
    /// axis normalization, trigonometry and all coordinate arithmetic.
    /// The field at a point in plain floating point, from the midpoints of
    /// every enclosed coefficient and pose: a reading for tests against a
    /// tolerance far wider than the enclosures' widths (is this point of a
    /// face on the tool's boundary, which side of it is material), never an
    /// interval claim. Shared nodes are simply evaluated again.
    pub fn value(&self,p: [f64;3]) -> f64 {
        match self.node.as_ref() {
            Node::Revolved(source) => source.value(p),
            Node::Extruded(source) => source.value(p),
            Node::Transformed {source,pose} => source.value(pose.inverse_point_mid(p)),
            Node::Union(a,b) => a.value(p).min(b.value(p)),
            Node::Intersection(a,b) => a.value(p).max(b.value(p)),
            Node::Difference(a,b) => a.value(p).max(-b.value(p)),
        }
    }

    /// The value, gradient and deciding leaf at a point (`reading.rs`), leaves numbered from
    /// `*next` depth-first; every leaf is visited, so the numbering does not depend on the point.
    pub fn reading(&self,p: [f64;3],options: &super::ReadingOptions,next: &mut usize) -> super::Reading {
        use super::reading::{higher,leaf,lower};
        match self.node.as_ref() {
            Node::Revolved(source) => { *next += 1; leaf(|q| source.value(q),p,options.step,*next-1,source.value_piece(p).1) }
            Node::Extruded(source) => { *next += 1; leaf(|q| source.value(q),p,options.step,*next-1,source.value_piece(p).1) }
            Node::Transformed {source,pose} => {
                let r = source.reading(pose.inverse_point_mid(p),options,next);
                super::Reading {gradient:pose.gradient_mid(r.gradient),..r}
            }
            Node::Union(a,b) => { let (x,y) = (a.reading(p,options,next),b.reading(p,options,next)); lower(x,y,options.tie) }
            Node::Intersection(a,b) => { let (x,y) = (a.reading(p,options,next),b.reading(p,options,next)); higher(x,y,options.tie) }
            Node::Difference(a,b) => {
                let (x,y) = (a.reading(p,options,next),b.reading(p,options,next));
                higher(x,y.negated(),options.tie)
            }
        }
    }

    /// How many leaves the field has, as `reading` numbers them.
    pub fn leaf_count(&self) -> usize {
        match self.node.as_ref() {
            Node::Revolved(_) | Node::Extruded(_) => 1,
            Node::Transformed {source,..} => source.leaf_count(),
            Node::Union(a,b) | Node::Intersection(a,b) | Node::Difference(a,b) => a.leaf_count()+b.leaf_count(),
        }
    }

    /// The reading of one operand alone at a point — piece `piece` of leaf `target`, its whole
    /// smooth carrier, or with `piece` `WHOLE` the leaf itself — with the transforms above it and the sign it enters the field with (turned
    /// where it is subtracted), leaves numbered from `*next` as `reading` numbers them; `None`
    /// when `target` is not among this field's leaves or has no such piece.
    pub fn leaf_reading(&self,p: [f64;3],options: &super::ReadingOptions,target: usize,piece: usize,
        next: &mut usize) -> Option<super::Reading> {
        use super::reading::leaf;
        match self.node.as_ref() {
            Node::Revolved(source) => {
                *next += 1;
                if *next-1 != target { return None; }
                if piece == super::WHOLE { return Some(leaf(|q| source.value(q),p,options.step,target,source.value_piece(p).1)); }
                if piece >= source.piece_count() { return None; }
                Some(leaf(|q| source.carrier(q,piece).unwrap_or(f64::NAN),p,options.step,target,piece))
            }
            Node::Extruded(source) => {
                *next += 1;
                if *next-1 != target { return None; }
                if piece == super::WHOLE { return Some(leaf(|q| source.value(q),p,options.step,target,source.value_piece(p).1)); }
                if piece >= source.piece_count() { return None; }
                Some(leaf(|q| source.carrier(q,piece).unwrap_or(f64::NAN),p,options.step,target,piece))
            }
            Node::Transformed {source,pose} => source.leaf_reading(pose.inverse_point_mid(p),options,target,piece,next)
                .map(|r| super::Reading {gradient:pose.gradient_mid(r.gradient),..r}),
            Node::Union(a,b) | Node::Intersection(a,b) => {
                let first = a.leaf_reading(p,options,target,piece,next);
                if first.is_some() { return first; }
                b.leaf_reading(p,options,target,piece,next)
            }
            Node::Difference(a,b) => {
                let first = a.leaf_reading(p,options,target,piece,next);
                if first.is_some() { return first; }
                b.leaf_reading(p,options,target,piece,next).map(super::Reading::negated)
            }
        }
    }

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
                Node::Revolved(_) | Node::Extruded(_) => false,
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
        let key = (Arc::as_ptr(&self.node) as usize,p.map(|v| v.bounds().map(f64::to_bits)));
        if let Some(cache) = cache { if let Some(value) = cache.get(&key) { return Ok(*value); } }
        let value = match self.node.as_ref() {
            Node::Revolved(source) => source.bounds(p),
            Node::Extruded(source) => source.bounds(p),
            Node::Transformed {source,pose} => source.evaluate(pose.inverse_point(p)?,cache),
            Node::Union(a,b) => Ok(min(a.evaluate(p,cache)?,b.evaluate(p,cache)?)),
            Node::Intersection(a,b) => Ok(max(a.evaluate(p,cache)?,b.evaluate(p,cache)?)),
            Node::Difference(a,b) => Ok(max(a.evaluate(p,cache)?,b.evaluate(p,cache)?.neg())),
        }?;
        if let Some(cache) = cache { cache.insert(key,value); }
        Ok(value)
    }
}
