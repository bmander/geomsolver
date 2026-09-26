//! Interval poses enclose the mathematical rigid family represented by a solved
//! snapshot. No rounded libm values or sampled poses enter the construction.
use super::{Family,Step};
use crate::interval::{Error,Interval as I};
type V = [I;3];
type M = [V;3];

fn vector(p: [f64;3]) -> Result<V,Error> {
    Ok([I::point(p[0])?,I::point(p[1])?,I::point(p[2])?])
}
fn sub(a: V,b: V) -> Result<V,Error> { Ok([a[0].sub(b[0])?,a[1].sub(b[1])?,a[2].sub(b[2])?]) }
fn dot(a: V,b: V) -> Result<I,Error> {
    a[0].mul(b[0])?.add(a[1].mul(b[1])?)?.add(a[2].mul(b[2])?)
}
fn mv(a: M,b: V) -> Result<V,Error> { Ok([dot(a[0],b)?,dot(a[1],b)?,dot(a[2],b)?]) }
fn transpose(a: M) -> M { std::array::from_fn(|i| std::array::from_fn(|j| a[j][i])) }
fn mm(a: M,b: M) -> Result<M,Error> {
    let bt = transpose(b);
    let mut result = [[I::ZERO;3];3];
    for i in 0..3 { for j in 0..3 { result[i][j] = dot(a[i],bt[j])?; } }
    Ok(result)
}

/// Enclosed poses over one finite roll interval, derived from one motion snapshot.
/// The private coefficients enclose rotations, but interval matrices are not
/// themselves orthogonal. Point queries include all poses and all points in the
/// input box; they do not claim every point of the output box is attainable.
#[derive(Clone,Copy,Debug)]
pub struct MotionBounds { r: M,p: V,
    /// The rotation's and the translation's midpoints in plain floating point, the `_mid`
    /// methods' — asked at every query through a placed copy — worked out once.
    plain: ([[f64;3];3],[f64;3]) }

impl MotionBounds {
    fn new(r: M,p: V) -> Self {
        Self {r,p,plain:(r.map(|row| row.map(I::mid)),p.map(I::mid))}
    }
    pub fn vector(&self,vector: V) -> Result<V,Error> { mv(self.r,vector) }
    pub fn point(&self,point: V) -> Result<V,Error> {
        let q = mv(self.r,point)?;
        Ok([q[0].add(self.p[0])?,q[1].add(self.p[1])?,q[2].add(self.p[2])?])
    }

    /// The inverse pose applied to a point in plain floating point, from
    /// the midpoints of the enclosed matrix: for evaluations whose answer is
    /// checked against a tolerance far wider than the enclosure, never for
    /// an interval claim.
    pub fn inverse_point_mid(&self,point: [f64;3]) -> [f64;3] {
        let (r,p) = &self.plain;
        let q: [f64;3] = std::array::from_fn(|k| point[k]-p[k]);
        std::array::from_fn(|k| (0..3).map(|i| r[i][k]*q[i]).sum())
    }

    /// The pose applied to a point in plain floating point, from the midpoints: `inverse_point_mid`'s
    /// converse, under the same terms.
    pub fn point_mid(&self,point: [f64;3]) -> [f64;3] {
        let (r,p) = &self.plain;
        std::array::from_fn(|i| (0..3).map(|k| r[i][k]*point[k]).sum::<f64>()+p[i])
    }

    /// A gradient read in the source's frame (at `inverse_point_mid`) turned into the world's: the
    /// rotation's midpoint applied, the transpose of the inverse map's linear part.
    pub fn gradient_mid(&self,g: [f64;3]) -> [f64;3] {
        let r = &self.plain.0;
        std::array::from_fn(|i| (0..3).map(|k| r[i][k]*g[k]).sum())
    }

    pub fn inverse_point(&self,point: V) -> Result<V,Error> {
        // Every underlying rotation is orthogonal. Transpose its enclosure; do
        // not numerically invert an interval matrix or assume independent entries.
        mv(transpose(self.r),sub(point,self.p)?)
    }

    pub(crate) fn rotation(origin: [f64;3],axis: [f64;3],angle: I) -> Result<Self,Error> {
        let mut axis = vector(axis)?;
        let norm2 = axis[0].square()?.add(axis[1].square()?)?.add(axis[2].square()?)?;
        let norm = I::new(norm2.bounds()[0].max(0.),norm2.bounds()[1])?.sqrt()?;
        for a in &mut axis { *a = a.div(norm)?; }
        let [x,y,z] = axis;
        let k = [[I::ZERO,z.neg(),y],[z,I::ZERO,x.neg()],[y.neg(),x,I::ZERO]];
        // a motion's angle runs as far as its sweep does: several turns for a coil
        let (s,c) = angle.sin_cos_periodic()?;
        let mut r = [[I::ZERO;3];3];
        for i in 0..3 { for j in 0..3 {
            // Group the coefficient of cos(theta). Using cos(theta) and
            // 1-cos(theta) independently loses cancellation along the axis.
            let parallel = axis[i].mul(axis[j])?;
            let transverse = (if i == j { I::ONE } else { I::ZERO }).sub(parallel)?;
            r[i][j] = parallel.add(c.mul(transverse)?)?.add(s.mul(k[i][j])?)?;
        } }
        let origin = vector(origin)?;
        Ok(Self::new(r,sub(origin,mv(r,origin)?)?))
    }

    /// Advance along a unit axis by `advance` per turn over an angle interval.
    fn advanced(self,axis: [f64;3],advance: f64,angle: I) -> Result<Self,Error> {
        if advance == 0. { return Ok(self); }
        let mut axis = vector(axis)?;
        let norm2 = axis[0].square()?.add(axis[1].square()?)?.add(axis[2].square()?)?;
        let norm = I::new(norm2.bounds()[0].max(0.),norm2.bounds()[1])?.sqrt()?;
        for a in &mut axis { *a = a.div(norm)?; }
        let distance = I::point(advance)?.mul(angle)?.div(I::point(std::f64::consts::TAU)?)?;
        let mut p = self.p;
        for k in 0..3 { p[k] = p[k].add(axis[k].mul(distance)?)?; }
        Ok(Self::new(self.r,p))
    }

    fn identity() -> Self {
        let mut r = [[I::ZERO;3];3];
        for i in 0..3 { r[i][i] = I::ONE; }
        Self::new(r,[I::ZERO;3])
    }

    fn relative(source: Self,observer: Self) -> Result<Self,Error> {
        let inverse_rotation = transpose(observer.r);
        Ok(Self::new(mm(inverse_rotation,source.r)?,mv(inverse_rotation,sub(source.p,observer.p)?)?))
    }
}

impl Family {
    /// Bound the mathematical solved motion for every roll value in `angle`.
    /// Axis normalization, phase/rate arithmetic and trigonometry all use outward
    /// intervals. This does not transfer errors from the sketch's nonlinear solve.
    /// Each primitive angle must lie within the interval sine/cosine domain [-8,8];
    /// unsupported argument reduction, overflow and unresolved normalization fail.
    pub fn bounds(&self,angle: I) -> Result<MotionBounds,Error> {
        let mut values = Vec::with_capacity(self.steps.len());
        for step in &self.steps {
            let value = match *step {
                Step::Rotation {origin,axis,ratio,phase,advance} => MotionBounds::rotation(origin,axis,
                    I::point(phase)?.add(I::point(ratio)?.mul(angle)?)?)?.advanced(axis,advance,angle)?,
                Step::Translation {axis,advance} => MotionBounds::identity().advanced(axis,advance,angle)?,
                Step::Relative {source,observer} => MotionBounds::relative(values[source],values[observer])?,
            };
            values.push(value);
        }
        Ok(*values.last().expect("a motion family has a root"))
    }
}
