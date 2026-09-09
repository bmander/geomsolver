//! Whole-source-box temporal contact bounds. No sampled coefficient error band.
use super::{Family,Step};
use crate::interval::{Interval as I,Error};
type V = [I;3];
fn point(p: [f64;3]) -> Result<V,Error> { Ok([I::point(p[0])?,I::point(p[1])?,I::point(p[2])?]) }
fn sub(a: V,b: V) -> Result<V,Error> { Ok([a[0].sub(b[0])?,a[1].sub(b[1])?,a[2].sub(b[2])?]) }
fn dot(a: V,b: V) -> Result<I,Error> { a[0].mul(b[0])?.add(a[1].mul(b[1])?)?.add(a[2].mul(b[2])?) }
fn scale(a: V,b: I) -> Result<V,Error> { Ok([a[0].mul(b)?,a[1].mul(b)?,a[2].mul(b)?]) }
fn cross(a: V,b: V) -> Result<V,Error> {
    Ok([a[1].mul(b[2])?.sub(a[2].mul(b[1])?)?,a[2].mul(b[0])?.sub(a[0].mul(b[2])?)?,
        a[0].mul(b[1])?.sub(a[1].mul(b[0])?)?])
}
fn unit(axis: [f64;3]) -> Result<V,Error> {
    let max = I::point(axis.iter().map(|x| x.abs()).fold(0.,f64::max))?;
    let mut axis = point(axis)?;
    for x in &mut axis { *x = x.div(max)?; }
    let length = axis[0].square()?.add(axis[1].square()?)?.add(axis[2].square()?)?.sqrt()?;
    for x in &mut axis { *x = x.div(length)?; }
    Ok(axis)
}

/// Coefficient enclosures for (du cross dv) dot velocity in the source frame.
/// This normal is NOT normalized: zeros at source poles remain possible contacts.
/// The one-sinusoid reduction supports a rotation or two relative rotations.
#[derive(Clone,Copy,Debug)]
pub struct NormalVelocityBounds { constant:I,cosine:I,sine:I,rate:I,phase:I }
impl NormalVelocityBounds {
    pub fn is_time_independent(&self) -> bool {
        self.rate == I::ZERO || (self.cosine == I::ZERO && self.sine == I::ZERO)
    }
    pub fn at(&self,time: I) -> Result<I,Error> {
        if self.cosine == I::ZERO && self.sine == I::ZERO { return Ok(self.constant); }
        let (s,c) = self.phase.add(self.rate.mul(time)?)?.sin_cos()?;
        self.constant.add(self.cosine.mul(c)?)?.add(self.sine.mul(s)?)
    }
    pub fn derivative(&self,time: I) -> Result<I,Error> {
        if self.is_time_independent() { return Ok(I::ZERO); }
        let (s,c) = self.phase.add(self.rate.mul(time)?)?.sin_cos()?;
        self.rate.mul(self.cosine.neg().mul(s)?.add(self.sine.mul(c)?)?)
    }
}
impl Family {
    /// Bound the same rigid-motion identity as `normal_velocity`, using the
    /// complete position/normal boxes and outward-rounded axis normalization and
    /// arithmetic. The normal need not be unit length; its enclosure is supplied
    /// by the source surface, including any correlations its construction retains.
    /// Unsupported nested relative motions return OutsideDomain.
    pub fn normal_velocity_bounds(&self,position: V,normal: V) -> Result<NormalVelocityBounds,Error> {
        let (source,observer) = match *self.steps.last().expect("motion root") {
            Step::Rotation {..} => (self.steps.last().unwrap(),None),
            Step::Relative {source,observer} => (&self.steps[source],Some(&self.steps[observer])),
        };
        let Step::Rotation {origin,axis,ratio,phase} = *source else { return Err(Error::OutsideDomain); };
        let axis = unit(axis)?;
        let (omega,offset) = match observer {
            None => ([I::ZERO;3],[I::ZERO;3]),
            Some(Step::Rotation {origin:other,axis,ratio,..}) =>
                (scale(unit(*axis)?,I::point(*ratio)?)?,sub(point(origin)?,point(*other)?)?),
            Some(_) => return Err(Error::OutsideDomain),
        };
        let n = normal;
        let x = sub(position,point(origin)?)?;
        let mut coefficients = [I::point(ratio)?.mul(dot(n,cross(axis,x)?)?)?,I::ZERO,I::ZERO];
        // A single fixed-axis rotation has a time-independent source contact
        // equation. Preserve that exact dependency instead of widening 0 terms.
        if observer.is_none() || matches!(observer,Some(Step::Rotation {ratio:0.,..})) {
            return Ok(NormalVelocityBounds {constant:coefficients[0],cosine:I::ZERO,sine:I::ZERO,
                rate:I::point(ratio)?,phase:I::point(phase)?});
        }
        for (v,w) in [(omega,cross(x,n)?),(cross(omega,offset)?,n)] {
            let parallel = scale(axis,dot(axis,w)?)?;
            coefficients[0] = coefficients[0].sub(dot(v,parallel)?)?;
            coefficients[1] = coefficients[1].sub(dot(v,sub(w,parallel)?)?)?;
            coefficients[2] = coefficients[2].sub(dot(v,cross(axis,w)?)?)?;
        }
        Ok(NormalVelocityBounds {constant:coefficients[0],cosine:coefficients[1],sine:coefficients[2],
            rate:I::point(ratio)?,phase:I::point(phase)?})
    }
}
