//! Analytic material fields with interval evaluation. The regularized material
//! set is closure({x | f(x)<0}); a zero value alone is not a boundary certificate.
//! These are explicit field constructions, not implicit conversions of mesh CSG.
use crate::interval::{Error,Interval as I};
mod profile;
mod spatial;
mod document;
pub use spatial::SpatialField;
mod swept;
pub use swept::{SweptField,SweepEvaluator,SweepError,SweptMinimum,SIDE_EVALUATIONS,SWEEP_TALLY};
mod material;
pub use material::{MaterialField,MaterialEvaluator,MaterialBounds,MaterialSweepQuery};
mod reading;
pub use reading::{Reading,ReadingOptions,WHOLE};
pub mod crease;
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
    /// A simple loop of lines and arcs as its signed boundary distance.
    Profile(profile::Profile),
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
            Node::Profile(profile) => Some(profile.reach()),
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

    /// The field at a point in plain floating point, from the midpoints of
    /// the enclosed coefficients: a reading to compare against a tolerance
    /// far wider than the enclosure's own width, never an interval claim.
    pub fn value(&self,p: [f64;2]) -> f64 {
        let mid = |x: I| { let [lo,hi] = x.bounds(); 0.5*(lo+hi) };
        match &self.node {
            Node::HalfPlane {through,normal} => (p[0]-mid(through[0]))*mid(normal[0])+(p[1]-mid(through[1]))*mid(normal[1]),
            Node::Disk {center,radius} => (p[0]-mid(center[0])).hypot(p[1]-mid(center[1]))-mid(*radius),
            Node::Union(a,b) => a.value(p).min(b.value(p)),
            Node::Intersection(a,b) => a.value(p).max(b.value(p)),
            Node::Difference(a,b) => a.value(p).max(-b.value(p)),
            Node::Profile(profile) => profile.value(p),
        }
    }

    /// The pieces of the field's boundary: each half-plane and disk, and each edge of a loop,
    /// numbered depth-first. A piece lies on a smooth carrier — a line or a circle — and a crease
    /// of the solid swept from the field is where the piece deciding its value changes.
    pub fn piece_count(&self) -> usize {
        match &self.node {
            Node::HalfPlane {..} | Node::Disk {..} => 1,
            Node::Union(a,b) | Node::Intersection(a,b) | Node::Difference(a,b) => a.piece_count()+b.piece_count(),
            Node::Profile(profile) => profile.edge_count(),
        }
    }

    /// `value`, and the piece deciding it, pieces numbered from `*next`.
    pub fn value_piece(&self,p: [f64;2],next: &mut usize) -> (f64,usize) {
        match &self.node {
            Node::HalfPlane {..} | Node::Disk {..} => { *next += 1; (self.value(p),*next-1) }
            Node::Union(a,b) => { let (x,y) = (a.value_piece(p,next),b.value_piece(p,next)); if x.0 <= y.0 { x } else { y } }
            Node::Intersection(a,b) => { let (x,y) = (a.value_piece(p,next),b.value_piece(p,next)); if x.0 >= y.0 { x } else { y } }
            Node::Difference(a,b) => {
                let (x,y) = (a.value_piece(p,next),b.value_piece(p,next));
                if x.0 >= -y.0 { x } else { (-y.0,y.1) }
            }
            Node::Profile(profile) => {
                let (value,edge) = profile.value_edge(p);
                let first = *next;
                *next += profile.edge_count();
                (value,first+edge)
            }
        }
    }

    /// The carrier of piece `target` — its whole line or circle, untrimmed — as a field negative
    /// on the material's side of the piece, with the sign it enters the field with; `None` when
    /// `target` is not one of this field's pieces.
    pub fn carrier(&self,p: [f64;2],target: usize,next: &mut usize) -> Option<f64> {
        match &self.node {
            Node::HalfPlane {..} | Node::Disk {..} => { *next += 1; (*next-1 == target).then(|| self.value(p)) }
            Node::Union(a,b) | Node::Intersection(a,b) => {
                let first = a.carrier(p,target,next);
                if first.is_some() { return first; }
                b.carrier(p,target,next)
            }
            Node::Difference(a,b) => {
                let first = a.carrier(p,target,next);
                if first.is_some() { return first; }
                b.carrier(p,target,next).map(|v| -v)
            }
            Node::Profile(profile) => {
                let first = *next;
                *next += profile.edge_count();
                (target >= first && target < *next).then(|| profile.carrier(p,target-first))
            }
        }
    }

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
            Node::Profile(profile) => profile.bounds(p),
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

    /// The field at a point in plain floating point; see `PlanarField::value`.
    pub fn value(&self,p: [f64;3]) -> f64 { self.profile.value(self.meridian(p)) }

    /// A point in the profile's (radius, height).
    fn meridian(&self,p: [f64;3]) -> [f64;2] {
        let mid = |x: I| { let [lo,hi] = x.bounds(); 0.5*(lo+hi) };
        let q: [f64;3] = std::array::from_fn(|i| p[i]-mid(self.origin[i]));
        let axis: [f64;3] = self.axis.map(mid);
        let z = q[0]*axis[0]+q[1]*axis[1]+q[2]*axis[2];
        if !self.profile.uses[0] { return [0.,z]; }
        let radial: [f64;3] = std::array::from_fn(|i| q[i]-z*axis[i]);
        [(radial[0]*radial[0]+radial[1]*radial[1]+radial[2]*radial[2]).sqrt(),z]
    }

    /// The profile's pieces, each turned: a line to a cone, cylinder or plane, a circle to a
    /// torus or sphere.
    pub fn piece_count(&self) -> usize { self.profile.piece_count() }
    pub fn value_piece(&self,p: [f64;3]) -> (f64,usize) { self.profile.value_piece(self.meridian(p),&mut 0) }
    pub fn carrier(&self,p: [f64;3],piece: usize) -> Option<f64> { self.profile.carrier(self.meridian(p),piece,&mut 0) }

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

/// Extrude an explicit planar field between two ordinates along the normal of
/// an orthonormal frame `(u, v, u x v)` about an origin. Gram-Schmidt runs in
/// interval arithmetic, so the enclosed mathematical frame is exactly
/// orthonormal, its coordinate map an isometry, and the field one-Lipschitz in
/// world coordinates: the prism is the profile intersected with the slab.
#[derive(Clone,Debug)]
pub struct ExtrudedField { profile:PlanarField,origin:V,frame:[V;3],range:I }

impl ExtrudedField {
    /// `range` holds the two ordinates in either order; a zero thickness is refused.
    pub fn new(profile: PlanarField,origin: [f64;3],u: [f64;3],v: [f64;3],range: [f64;2])
        -> Result<Self,Error> {
        if range[0] == range[1] { return Err(Error::OutsideDomain); }
        let range = I::new(range[0].min(range[1]),range[0].max(range[1]))?;
        let dot = |a: V,b: V| (0..3).try_fold(I::ZERO,|s,i| a[i].mul(b[i]).and_then(|t| s.add(t)));
        let unit = |mut a: V| -> Result<V,Error> {
            let length = norm(a)?;
            for x in &mut a { *x = x.div(length)?; }
            Ok(a)
        };
        let u = unit(point(u)?)?;
        let mut v = point(v)?; let shear = dot(v,u)?;
        for i in 0..3 { v[i] = v[i].sub(shear.mul(u[i])?)?; }
        let v = unit(v)?;
        let n = [u[1].mul(v[2])?.sub(u[2].mul(v[1])?)?,u[2].mul(v[0])?.sub(u[0].mul(v[2])?)?,
            u[0].mul(v[1])?.sub(u[1].mul(v[0])?)?];
        Ok(Self {profile,origin:point(origin)?,frame:[u,v,n],range})
    }

    /// A finite world box enclosing all regularized material, when the profile
    /// construction gives one; `None` is an unknown support, not unboundedness.
    pub fn support_bounds(&self) -> Result<Option<V>,Error> {
        let Some(radius) = self.profile.radius_bound()? else { return Ok(None); };
        let disk = I::new(-radius,radius)?;
        let mut result = self.origin;
        // |a u_k + b v_k| <= sqrt(a^2 + b^2) for orthonormal u, v.
        for k in 0..3 { result[k] = result[k].add(disk)?.add(self.range.mul(self.frame[2][k])?)?; }
        Ok(Some(result))
    }

    /// The field at a point in plain floating point; see `PlanarField::value`.
    pub fn value(&self,p: [f64;3]) -> f64 {
        let c = self.frame_of(p);
        let [lo,hi] = self.range.bounds();
        self.profile.value([c[0],c[1]]).max((lo-c[2]).max(c[2]-hi))
    }

    /// A point in the frame's (u, v, depth).
    fn frame_of(&self,p: [f64;3]) -> [f64;3] {
        let mid = |x: I| { let [lo,hi] = x.bounds(); 0.5*(lo+hi) };
        let q: [f64;3] = std::array::from_fn(|i| p[i]-mid(self.origin[i]));
        std::array::from_fn(|k| (0..3).map(|i| q[i]*mid(self.frame[k][i])).sum())
    }

    /// The profile's pieces, each run along the depth, then the two caps (the near end, then the
    /// far).
    pub fn piece_count(&self) -> usize { self.profile.piece_count()+2 }
    pub fn value_piece(&self,p: [f64;3]) -> (f64,usize) {
        let c = self.frame_of(p);
        let [lo,hi] = self.range.bounds();
        let n = self.profile.piece_count();
        let side = self.profile.value_piece([c[0],c[1]],&mut 0);
        [side,(lo-c[2],n),(c[2]-hi,n+1)].into_iter().fold((f64::NEG_INFINITY,0),|a,b| if b.0 > a.0 { b } else { a })
    }
    pub fn carrier(&self,p: [f64;3],piece: usize) -> Option<f64> {
        let c = self.frame_of(p);
        let [lo,hi] = self.range.bounds();
        let n = self.profile.piece_count();
        if piece == n { Some(lo-c[2]) } else if piece == n+1 { Some(c[2]-hi) }
        else { self.profile.carrier([c[0],c[1]],piece,&mut 0) }
    }

    pub fn bounds(&self,p: V) -> Result<I,Error> {
        let mut q = [I::ZERO;3];
        for i in 0..3 { q[i] = p[i].sub(self.origin[i])?; }
        let mut c = [I::ZERO;3];
        for (k,axis) in self.frame.iter().enumerate() {
            for i in 0..3 { c[k] = c[k].add(q[i].mul(axis[i])?)?; }
        }
        let [lo,hi] = self.range.bounds();
        let slab = max(I::point(lo)?.sub(c[2])?,c[2].sub(I::point(hi)?)?);
        Ok(max(self.profile.bounds([c[0],c[1]])?,slab))
    }
}
