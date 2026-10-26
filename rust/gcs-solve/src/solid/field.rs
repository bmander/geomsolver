//! Analytic material fields with interval evaluation. The regularized material
//! set is closure({x | f(x)<0}); a zero value alone is not a boundary certificate.
//! These are explicit field constructions, not implicit conversions of mesh CSG.
use crate::interval::{Error,Interval as I};
#[allow(unused_imports)]
use crate::fmath::Det;
mod profile;
mod sections;
pub(crate) use sections::{oriented,Sections};
pub(crate) use document::prism_loops;
mod spatial;
mod document;
pub use document::CHORD_SLACK;
pub mod region;
pub use region::RegionField;
mod built;
pub use built::BuiltField;
pub use spatial::SpatialField;
mod swept;
pub use swept::{SweptField,SweepEvaluator,SweepError,SIDE_EVALUATIONS};
mod material;
pub use material::{MaterialField,MaterialEvaluator,MaterialBounds,MaterialSweepQuery,Symmetry};
mod reading;
pub use reading::{OperandId,Query,Reading,Source,Want};
pub use adf::Resolution;
pub mod crease;
mod adf;
mod memo;
mod probe;
pub use probe::{MaterialProbe,ProbeState};
mod millimetres;
pub use millimetres::{Millimetres,MillimetreEvaluator};
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

/// The least interval holding both.
fn hull(a: I,b: I) -> Result<I,Error> {
    let (a,b) = (a.bounds(),b.bounds());
    I::new(a[0].min(b[0]),a[1].max(b[1]))
}

/// What two intervals share, or the first where they share nothing: an enclosure either way.
fn common(a: I,b: I) -> Result<I,Error> {
    let (x,y) = (a.bounds(),b.bounds());
    let (lo,hi) = (x[0].max(y[0]),x[1].min(y[1]));
    if lo > hi { Ok(a) } else { I::new(lo,hi) }
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


fn mid(x: I) -> f64 { x.mid() }

/// Where a field's plain value stands against a level `t`, as comparing `value(p)` with `t` says
/// (`Nan` where the value is no number): read where only that is asked, a Boolean's second operand
/// left unread where its first decides, and a loop's nearest edge sought only within the level.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub(crate) enum Side { Below,At,Above,Nan }

impl Side {
    pub(crate) fn of(v: f64,t: f64) -> Side {
        if v < t { Side::Below } else if v > t { Side::Above } else if v == t { Side::At } else { Side::Nan }
    }
    /// The side of `-v` against `-t`, `v`'s against `t` being this.
    fn negated(self) -> Side { match self { Side::Below => Side::Above,Side::Above => Side::Below,s => s } }
    /// The side of `f64::min` of the two values (a number over no number), `other`'s read only
    /// where this one's does not decide it.
    fn min(self,other: impl FnOnce() -> Side) -> Side {
        if self == Side::Below { return Side::Below }
        match (self,other()) {
            (Side::Nan,o) => o,
            (s,Side::Nan) => s,
            (_,Side::Below) => Side::Below,
            (Side::At,_) | (_,Side::At) => Side::At,
            _ => Side::Above,
        }
    }
    /// The side of `f64::max` of the two values, likewise.
    fn max(self,other: impl FnOnce() -> Side) -> Side { self.negated().min(|| other().negated()).negated() }
}

#[derive(Clone,Debug)]
enum Node {
    /// `plain` is the midpoints in plain floating point, `value`'s: the through point, then the
    /// normal (the centre, then the radius), worked out once rather than at every evaluation.
    HalfPlane {through:P,normal:P,plain:[f64;4]},
    Disk {center:P,radius:I,plain:[f64;3]},
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
pub struct PlanarField { node:Node,depth:u8,uses:[bool;2],
    /// A box holding its material where its construction knows one (a loop's own, its low corner
    /// and its high), which a disk about planar zero may hold only far wider.
    hull:Option<[[f64;2];2]> }

impl PlanarField {
    // A disk about planar zero enclosing closure({f<0}), if derivable from the
    // construction. Unknown does not mean unbounded; complementary half-planes
    // can bound a polygon without any finite primitive support.
    fn radius_bound(&self) -> Result<Option<f64>,Error> {
        Ok(match &self.node {
            Node::HalfPlane {..} => None,
            Node::Disk {center,radius,..} => Some(norm(*center)?.add(*radius)?.bounds()[1]),
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
    // A box in planar coordinates enclosing closure({f<0}), where the construction gives one, as
    // `radius_bound`'s disk: a profile far from planar zero lies in a box of its own size, where
    // the disk about zero reaching it is that far across.
    fn box_bound(&self) -> Result<Option<[I;2]>,Error> {
        if let Some([lo,hi]) = self.hull { return Ok(Some([I::new(lo[0],hi[0])?,I::new(lo[1],hi[1])?])) }
        Ok(match &self.node {
            Node::HalfPlane {..} => None,
            Node::Disk {center,radius,..} => {
                let r = radius.bounds()[1];
                let about = |c: I| { let [lo,hi] = c.bounds(); I::new(lo-r,hi+r) };
                Some([about(center[0])?,about(center[1])?])
            }
            Node::Union(a,b) => match (a.box_bound()?,b.box_bound()?) {
                (Some(a),Some(b)) => Some([hull(a[0],b[0])?,hull(a[1],b[1])?]), _ => None,
            },
            Node::Intersection(a,b) => match (a.box_bound()?,b.box_bound()?) {
                (Some(a),Some(b)) => Some([common(a[0],b[0])?,common(a[1],b[1])?]), (a,b) => a.or(b),
            },
            Node::Difference(a,_) => a.box_bound()?,
            Node::Profile(profile) => {
                let [lo,hi] = profile.boxed();
                Some([I::new(lo[0],hi[0])?,I::new(lo[1],hi[1])?])
            }
        })
    }
    /// Material is on the negative side of the supplied outward normal.
    pub fn half_plane(through: [f64;2],normal: [f64;2]) -> Result<Self,Error> {
        // Record exact source dependencies before interval normalization widens
        // zero coefficients. A tiny nonzero coefficient still uses its axis.
        let uses = normal.map(|n| n != 0.);
        let mut normal = point(normal)?; let length = norm(normal)?;
        for n in &mut normal { *n = n.div(length)?; }
        let through = point(through)?;
        let plain = [mid(through[0]),mid(through[1]),mid(normal[0]),mid(normal[1])];
        Ok(Self {node:Node::HalfPlane {through,normal,plain},depth:1,uses,hull:None})
    }

    pub fn disk(center: [f64;2],radius: f64) -> Result<Self,Error> {
        if radius <= 0. { return Err(Error::OutsideDomain); }
        let (center,radius) = (point(center)?,I::point(radius)?);
        let plain = [mid(center[0]),mid(center[1]),mid(radius)];
        Ok(Self {node:Node::Disk {center,radius,plain},depth:1,uses:[true;2],hull:None})
    }

    fn combine(self,other: Self,make: impl FnOnce(Box<Self>,Box<Self>)->Node) -> Result<Self,Error> {
        let depth = self.depth.max(other.depth)+1;
        if depth > 64 { return Err(Error::OutsideDomain); }
        let uses = std::array::from_fn(|i| self.uses[i] || other.uses[i]);
        Ok(Self {node:make(Box::new(self),Box::new(other)),depth,uses,hull:None})
    }
    pub fn union(self,other: Self) -> Result<Self,Error> { self.combine(other,Node::Union) }
    pub fn intersection(self,other: Self) -> Result<Self,Error> { self.combine(other,Node::Intersection) }
    pub fn difference(self,other: Self) -> Result<Self,Error> { self.combine(other,Node::Difference) }

    /// The field at a point in plain floating point, from the midpoints of
    /// the enclosed coefficients: a reading to compare against a tolerance
    /// far wider than the enclosure's own width, never an interval claim.
    pub fn value(&self,p: [f64;2]) -> f64 {
        match &self.node {
            Node::HalfPlane {plain:[tx,ty,nx,ny],..} => (p[0]-tx)*nx+(p[1]-ty)*ny,
            Node::Disk {plain:[cx,cy,r],..} => { let (x,y) = (p[0]-cx,p[1]-cy); (x*x+y*y).sqrt()-r }
            Node::Union(a,b) => a.value(p).min(b.value(p)),
            Node::Intersection(a,b) => a.value(p).max(b.value(p)),
            Node::Difference(a,b) => a.value(p).max(-b.value(p)),
            Node::Profile(profile) => profile.value(p),
        }
    }

    /// `Side::of(self.value(p), t)`, its Booleans' second operands read only where needed.
    pub(crate) fn side(&self,p: [f64;2],t: f64) -> Side {
        match &self.node {
            Node::HalfPlane {..} | Node::Disk {..} => Side::of(self.value(p),t),
            Node::Union(a,b) => a.side(p,t).min(|| b.side(p,t)),
            Node::Intersection(a,b) => a.side(p,t).max(|| b.side(p,t)),
            Node::Difference(a,b) => a.side(p,t).max(|| b.side(p,-t).negated()),
            Node::Profile(profile) => profile.side(p,t),
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
            Node::HalfPlane {through,normal,..} => {
                let term = |k:usize| p[k].sub(through[k])?.mul(normal[k]);
                match self.uses {
                    [true,true] => term(0)?.add(term(1)?),
                    [true,false] => term(0),
                    [false,true] => term(1),
                    [false,false] => unreachable!("half-plane normalization rejects a zero normal"),
                }
            },
            Node::Disk {center,radius,..} => norm([p[0].sub(center[0])?,p[1].sub(center[1])?])?.sub(*radius),
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
pub struct RevolvedField { profile:PlanarField,origin:V,axis:V,
    /// The origin's and the axis's midpoints, `value`'s, worked out once.
    plain:[[f64;3];2] }

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
        let origin = point(origin)?;
        let plain = [origin.map(mid),axis.map(mid)];
        Ok(Self {profile,origin,axis,plain})
    }

    /// The field at a point in plain floating point; see `PlanarField::value`.
    pub fn value(&self,p: [f64;3]) -> f64 { self.profile.value(self.meridian(p)) }

    /// A point in the profile's (radius, height).
    fn meridian(&self,p: [f64;3]) -> [f64;2] {
        let [origin,axis] = self.plain;
        let q: [f64;3] = std::array::from_fn(|i| p[i]-origin[i]);
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
pub struct ExtrudedField { profile:PlanarField,origin:V,frame:[V;3],range:I,
    /// The origin's and the frame's midpoints, `frame_of`'s, worked out once.
    plain:[[f64;3];4],
    /// A draft's tangent and cosine (#245; 0 and 1 where there is none): the sides at depth `w`
    /// read as the profile's field plus `|w| tan`, times the cosine, so the field stays
    /// one-Lipschitz and is a tilted side's exact distance where the profile's is its line's.
    draft:[f64;2],
    /// A drafted loop the profile's field cannot move (#607): each depth's section read itself
    /// (`sections`), in place of the profile's field.
    sections:Option<std::sync::Arc<sections::Sections>> }

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
        let origin = point(origin)?;
        let plain = [origin.map(mid),u.map(mid),v.map(mid),n.map(mid)];
        Ok(Self {profile,origin,frame:[u,v,n],range,plain,draft:[0.,1.],sections:None})
    }

    /// Its sides tapered by `angle` (radians) from the profile's plane where the profile's own
    /// field does not move as a draft does (#607): every depth's section read as the profile's
    /// `loops` moved into their material (`sections::Sections`), refused where one within the
    /// prism's depths loses a side.
    pub(crate) fn drafted_sections(mut self,loops: &[Vec<crate::solid::surface::Edge>],angle: f64) -> Result<Self,String> {
        let [lo,hi] = self.range.bounds();
        self.sections = Some(std::sync::Arc::new(sections::Sections::new(loops,angle,[lo,hi])?));
        Ok(self)
    }

    /// Its sides tapered by `angle` (radians) from the profile's plane (#245): the section at
    /// depth `w` is where the profile's field is at most `-|w| tan angle`. Exactly the draft where
    /// the profile's field moves its sides so (`PlanarField::draftable`, its caller's to ask).
    pub fn drafted(mut self,angle: f64) -> Self {
        self.draft = [angle.dtan(),angle.dcos()];
        self
    }

    /// The sides' value at a frame point: the profile's, drafted.
    fn side(&self,value: f64,depth: f64) -> f64 {
        let [t,c] = self.draft;
        if t == 0. { value } else { (value+depth.abs()*t)*c }
    }

    /// A finite world box enclosing all regularized material, when the profile
    /// construction gives one; `None` is an unknown support, not unboundedness.
    pub fn support_bounds(&self) -> Result<Option<V>,Error> {
        let radius = match &self.sections {
            Some(s) => s.reach(),
            None => match self.profile.radius_bound()? { Some(r) => r,None => return Ok(None) },
        };
        // a draft leaning out grows the section, a sharp corner's furthest: not bounded here
        if self.draft[0] < 0. && self.sections.is_none() { return Ok(None); }
        let disk = I::new(-radius,radius)?;
        let mut result = self.origin;
        // |a u_k + b v_k| <= sqrt(a^2 + b^2) for orthonormal u, v.
        for k in 0..3 { result[k] = result[k].add(disk)?.add(self.range.mul(self.frame[2][k])?)?; }
        // and within the profile's own box carried by the frame, where every section is within
        // the profile (no draft, or one leaning in): a profile drawn far from its plane's origin is
        // in a box its own size, where the disk about the origin reaching it is twice that far across
        let within = self.sections.is_none() && self.draft[0] >= 0.;
        if let (true,Some([a,b])) = (within,self.profile.box_bound()?) {
            let mut boxed = self.origin;
            for k in 0..3 {
                boxed[k] = boxed[k].add(a.mul(self.frame[0][k])?)?.add(b.mul(self.frame[1][k])?)?
                    .add(self.range.mul(self.frame[2][k])?)?;
            }
            return Ok(intersection_support(Some(result),Some(boxed)));
        }
        Ok(Some(result))
    }

    /// The field at a point in plain floating point; see `PlanarField::value`.
    pub fn value(&self,p: [f64;3]) -> f64 {
        let c = self.frame_of(p);
        let [lo,hi] = self.range.bounds();
        let side = match &self.sections { Some(s) => s.value([c[0],c[1]],c[2]),None => self.side(self.profile.value([c[0],c[1]]),c[2]) };
        side.max((lo-c[2]).max(c[2]-hi))
    }

    /// `Side::of(self.value(p), t)`: the caps first, the profile only where they leave it open.
    pub(crate) fn side_of(&self,p: [f64;3],t: f64) -> Side {
        let c = self.frame_of(p);
        let [lo,hi] = self.range.bounds();
        let cap = Side::of((lo-c[2]).max(c[2]-hi),t);
        match &self.sections {
            None if self.draft[0] == 0. => cap.max(|| self.profile.side([c[0],c[1]],t)),
            _ => Side::of(self.value(p),t),
        }
    }

    /// A point in the frame's (u, v, depth).
    fn frame_of(&self,p: [f64;3]) -> [f64;3] {
        let [origin,u,v,n] = self.plain;
        let q: [f64;3] = std::array::from_fn(|i| p[i]-origin[i]);
        [u,v,n].map(|a| q[0]*a[0]+q[1]*a[1]+q[2]*a[2])
    }

    /// The profile's pieces, each run along the depth, then the two caps (the near end, then the
    /// far).
    pub fn piece_count(&self) -> usize { self.side_pieces()+2 }
    fn side_pieces(&self) -> usize { self.sections.as_ref().map_or_else(|| self.profile.piece_count(),|s| s.pieces()) }
    pub fn value_piece(&self,p: [f64;3]) -> (f64,usize) {
        let c = self.frame_of(p);
        let [lo,hi] = self.range.bounds();
        let n = self.side_pieces();
        let side = match &self.sections {
            Some(s) => s.value_piece([c[0],c[1]],c[2]),
            None => { let side = self.profile.value_piece([c[0],c[1]],&mut 0); (self.side(side.0,c[2]),side.1) }
        };
        [side,(lo-c[2],n),(c[2]-hi,n+1)].into_iter().fold((f64::NEG_INFINITY,0),|a,b| if b.0 > a.0 { b } else { a })
    }
    pub fn carrier(&self,p: [f64;3],piece: usize) -> Option<f64> {
        let c = self.frame_of(p);
        let [lo,hi] = self.range.bounds();
        let n = self.side_pieces();
        if piece == n { Some(lo-c[2]) } else if piece == n+1 { Some(c[2]-hi) }
        else if let Some(s) = &self.sections { s.carrier([c[0],c[1]],c[2],piece) }
        else { self.profile.carrier([c[0],c[1]],piece,&mut 0).map(|v| self.side(v,c[2])) }
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
        // a drafted section read at the box's middle, one-Lipschitz about it
        if let Some(s) = &self.sections {
            let mid = c.map(|x| { let [a,b] = x.bounds(); 0.5*(a+b) });
            let half = c.iter().map(|x| { let [a,b] = x.bounds(); 0.5*(b-a) }).fold(0.,|r: f64,h| r.dhypot(h));
            let v = s.value([mid[0],mid[1]],mid[2]);
            let slack = 1e-12*(1.+v.abs()+half);
            return Ok(max(I::new(v-half-slack,v+half+slack)?,slab))
        }
        let mut side = self.profile.bounds([c[0],c[1]])?;
        let [t,cos] = self.draft;
        if t != 0. {
            side = side.add(profile::magnitude(c[2]).mul(I::point(t)?)?)?.mul(I::point(cos)?)?;
        }
        Ok(max(side,slab))
    }
}
