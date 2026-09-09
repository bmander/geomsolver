//! Analytic material fields with interval evaluation. The regularized material
//! set is closure({x | f(x)<0}); a zero value alone is not a boundary certificate.
//! These are explicit field constructions, not implicit conversions of mesh CSG.
use crate::interval::{Error,Interval as I};
mod spatial;
mod document;
pub use spatial::SpatialField;
mod swept;
pub use swept::{SweptField,SweepEvaluator,SweepError};
mod material;
pub use material::{MaterialField,MaterialEvaluator,MaterialBounds,MaterialSweepQuery};
mod probe;
pub use probe::{MaterialProbe,ProbeState};
mod boundary;
pub use boundary::{BoundaryOptions,BoundaryError,BoundaryStage,FieldBoundary,BoundaryCell,BoundaryPoint,BoundaryCrossing};
pub use boundary::BoundaryComponent;
type P = [I;2];
type V = [I;3];

fn norm<const N: usize>(v: [I;N]) -> Result<I,Error> {
    let sum = v.into_iter().try_fold(I::ZERO,|s,x| s.add(x.square()?))?;
    I::new(sum.bounds()[0].max(0.),sum.bounds()[1])?.sqrt()
}
fn point<const N: usize>(p: [f64;N]) -> Result<[I;N],Error> {
    let mut result = [I::ZERO;N];
    for i in 0..N { result[i] = I::point(p[i])?; }
    Ok(result)
}
fn min(a: I,b: I) -> I {
    I::new(a.bounds()[0].min(b.bounds()[0]),a.bounds()[1].min(b.bounds()[1])).unwrap()
}
fn max(a: I,b: I) -> I {
    I::new(a.bounds()[0].max(b.bounds()[0]),a.bounds()[1].max(b.bounds()[1])).unwrap()
}

fn union_support(a: Option<V>,b: Option<V>) -> Option<V> {
    let (a,b) = (a?,b?);
    Some(std::array::from_fn(|k| {
        let a = a[k].bounds(); let b = b[k].bounds();
        I::new(a[0].min(b[0]),a[1].max(b[1])).unwrap()
    }))
}

fn intersection_support(a: Option<V>,b: Option<V>) -> Option<V> {
    match (a,b) {
        (Some(a),Some(b)) => {
            let mut result = a;
            for k in 0..3 {
                let lo = a[k].bounds()[0].max(b[k].bounds()[0]);
                let hi = a[k].bounds()[1].min(b[k].bounds()[1]);
                // Disjoint supports imply empty material. Any existing finite
                // box still encloses it; no finite box is claimed to be tight.
                if lo > hi { return Some(a); }
                result[k] = I::new(lo,hi).unwrap();
            }
            Some(result)
        },
        _ => a.or(b),
    }
}

#[derive(Clone,Debug)]
enum Node {
    HalfPlane {through:P,normal:P},
    Disk {center:P,radius:I},
    Union(Box<PlanarField>,Box<PlanarField>),
    Intersection(Box<PlanarField>,Box<PlanarField>),
    Difference(Box<PlanarField>,Box<PlanarField>),
}

/// A continuous, one-Lipschitz scalar field in planar coordinates. Negative
/// values denote material; positive values denote exterior. Boolean combinations
/// preserve the Lipschitz bound but need not preserve exact signed distance.
/// The coefficients enclose exact normalization of the supplied binary64 data.
#[derive(Clone,Debug)]
pub struct PlanarField { node:Node,depth:u8,uses:[bool;2] }

impl PlanarField {
    // A disk about planar zero enclosing closure({f<0}), if derivable from the
    // construction. Unknown does not mean unbounded; complementary half-planes
    // can bound a polygon without any finite primitive support.
    fn radius_bound(&self) -> Result<Option<f64>,Error> {
        Ok(match &self.node {
            Node::HalfPlane {..} => None,
            Node::Disk {center,radius} => Some(norm(*center)?.add(*radius)?.bounds()[1]),
            Node::Union(a,b) => match (a.radius_bound()?,b.radius_bound()?) {
                (Some(a),Some(b)) => Some(a.max(b)), _ => None,
            },
            Node::Intersection(a,b) => match (a.radius_bound()?,b.radius_bound()?) {
                (Some(a),Some(b)) => Some(a.min(b)), (a,b) => a.or(b),
            },
            Node::Difference(a,_) => a.radius_bound()?,
        })
    }
    /// Material is on the negative side of the supplied outward normal.
    pub fn half_plane(through: [f64;2],normal: [f64;2]) -> Result<Self,Error> {
        // Record exact source dependencies before interval normalization widens
        // zero coefficients. A tiny nonzero coefficient still uses its axis.
        let uses = normal.map(|n| n != 0.);
        let mut normal = point(normal)?; let length = norm(normal)?;
        for n in &mut normal { *n = n.div(length)?; }
        Ok(Self {node:Node::HalfPlane {through:point(through)?,normal},depth:1,uses})
    }

    pub fn disk(center: [f64;2],radius: f64) -> Result<Self,Error> {
        if radius <= 0. { return Err(Error::OutsideDomain); }
        Ok(Self {node:Node::Disk {center:point(center)?,radius:I::point(radius)?},depth:1,uses:[true;2]})
    }

    fn combine(self,other: Self,make: impl FnOnce(Box<Self>,Box<Self>)->Node) -> Result<Self,Error> {
        let depth = self.depth.max(other.depth)+1;
        if depth > 64 { return Err(Error::OutsideDomain); }
        let uses = std::array::from_fn(|i| self.uses[i] || other.uses[i]);
        Ok(Self {node:make(Box::new(self),Box::new(other)),depth,uses})
    }
    pub fn union(self,other: Self) -> Result<Self,Error> { self.combine(other,Node::Union) }
    pub fn intersection(self,other: Self) -> Result<Self,Error> { self.combine(other,Node::Intersection) }
    pub fn difference(self,other: Self) -> Result<Self,Error> { self.combine(other,Node::Difference) }

    /// Enclose the field over the complete point box; no sampling or libm calls.
    pub fn bounds(&self,p: P) -> Result<I,Error> {
        match &self.node {
            Node::HalfPlane {through,normal} => {
                let term = |k:usize| p[k].sub(through[k])?.mul(normal[k]);
                match self.uses {
                    [true,true] => term(0)?.add(term(1)?),
                    [true,false] => term(0),
                    [false,true] => term(1),
                    [false,false] => unreachable!("half-plane normalization rejects a zero normal"),
                }
            },
            Node::Disk {center,radius} => norm([p[0].sub(center[0])?,p[1].sub(center[1])?])?.sub(*radius),
            Node::Union(a,b) => Ok(min(a.bounds(p)?,b.bounds(p)?)),
            Node::Intersection(a,b) => Ok(max(a.bounds(p)?,b.bounds(p)?)),
            Node::Difference(a,b) => Ok(max(a.bounds(p)?,b.bounds(p)?.neg())),
        }
    }
}

/// Revolve an explicit planar field in (nonnegative radius, axial height).
/// The mathematical frame normalizes the supplied axis exactly. Its coordinate
/// map is nonexpansive, so the field remains one-Lipschitz in world coordinates.
/// Do not add a half-plane for an artificial meridian spine on the axis: radial
/// coordinates are already nonnegative, and a spine is not a revolved boundary.
#[derive(Clone,Debug)]
pub struct RevolvedField { profile:PlanarField,origin:V,axis:V }

impl RevolvedField {
    /// A finite world box enclosing all regularized material, when one can be
    /// derived from the profile construction. `None` is an unknown support,
    /// not a proof of unboundedness. This is independent of sampling or a crop.
    pub fn support_bounds(&self) -> Result<Option<V>,Error> {
        let Some(radius) = self.profile.radius_bound()? else { return Ok(None); };
        let mut result = self.origin;
        // Revolution preserves sqrt(r*r+z*z) about its normalized exact axis.
        for p in &mut result { *p = p.add(I::new(-radius,radius)?)?; }
        Ok(Some(result))
    }
    pub fn new(profile: PlanarField,origin: [f64;3],axis: [f64;3]) -> Result<Self,Error> {
        let mut axis = point(axis)?; let length = norm(axis)?;
        for a in &mut axis { *a = a.div(length)?; }
        Ok(Self {profile,origin:point(origin)?,axis})
    }

    pub fn bounds(&self,p: V) -> Result<I,Error> {
        let mut q = [I::ZERO;3]; let mut z = I::ZERO;
        for i in 0..3 { q[i] = p[i].sub(self.origin[i])?; z = z.add(q[i].mul(self.axis[i])?)?; }
        // Axial fields (including their Booleans) have no radial dependency.
        // Avoid the radial projection/norm and its irrelevant overflow risk.
        if !self.profile.uses[0] { return self.profile.bounds([I::ZERO,z]); }
        for i in 0..3 { q[i] = q[i].sub(z.mul(self.axis[i])?)?; }
        self.profile.bounds([norm(q)?,z])
    }
}
