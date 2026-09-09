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
// Shared coefficient construction for the value and source directional derivatives.
struct Frame {origin:V,axis:V,omega:V,offset:V,rate:I,phase:I,fixed_observer:bool}
impl Frame {
    fn read(family: &Family) -> Result<Self,Error> {
        let (source,observer) = match *family.steps.last().expect("motion root") {
            Step::Rotation {..} => (family.steps.last().unwrap(),None),
            Step::Relative {source,observer} => (&family.steps[source],Some(&family.steps[observer])),
        };
        let Step::Rotation {origin,axis,ratio,phase} = *source else { return Err(Error::OutsideDomain); };
        let (omega,offset) = match observer {
            None => ([I::ZERO;3],[I::ZERO;3]),
            Some(Step::Rotation {origin:other,axis,ratio,..}) =>
                (scale(unit(*axis)?,I::point(*ratio)?)?,sub(point(origin)?,point(*other)?)?),
            Some(_) => return Err(Error::OutsideDomain),
        };
        Ok(Self {origin:point(origin)?,axis:unit(axis)?,omega,offset,rate:I::point(ratio)?,phase:I::point(phase)?,
            fixed_observer:observer.is_none() || matches!(observer,Some(Step::Rotation {ratio:0.,..}))})
    }
    fn coefficients(&self,position: V,n: V,vector: bool) -> Result<[I;3],Error> {
        // A position derivative is a vector: origin/offset terms disappear.
        let x = if vector { position } else { sub(position,self.origin)? };
        self.moment_coefficients(n,cross(x,n)?,vector)
    }
    fn moment_coefficients(&self,n: V,moment: V,vector: bool) -> Result<[I;3],Error> {
        let mut c = [self.rate.mul(dot(self.axis,moment)?)?,I::ZERO,I::ZERO];
        if self.fixed_observer { return Ok(c); }
        let mut subtract_rotation = |v,w| -> Result<(),Error> {
            let parallel = scale(self.axis,dot(self.axis,w)?)?;
            c[0] = c[0].sub(dot(v,parallel)?)?;
            c[1] = c[1].sub(dot(v,sub(w,parallel)?)?)?;
            c[2] = c[2].sub(dot(v,cross(self.axis,w)?)?)?;
            Ok(())
        };
        subtract_rotation(self.omega,moment)?;
        if !vector { subtract_rotation(cross(self.omega,self.offset)?,n)?; }
        Ok(c)
    }
    fn equation(&self,c: [I;3]) -> NormalVelocityBounds {
        NormalVelocityBounds {constant:c[0],cosine:c[1],sine:c[2],rate:self.rate,phase:self.phase}
    }
}
impl Family {
    /// The rigid-motion contact equation is linear in the source normal and
    /// position cross normal. A source can bound that moment before rotating,
    /// avoiding the dependency loss of crossing separate world-coordinate boxes.
    /// Passing both source derivatives gives the corresponding equation derivative.
    pub fn normal_velocity_moment_bounds(&self,normal: V,position_cross_normal: V) -> Result<NormalVelocityBounds,Error> {
        let frame = Frame::read(self)?;
        let moment = sub(position_cross_normal,cross(frame.origin,normal)?)?;
        Ok(frame.equation(frame.moment_coefficients(normal,moment,false)?))
    }

    /// Bound the same rigid-motion identity as `normal_velocity`, using the
    /// complete position/normal boxes and outward-rounded axis normalization and
    /// arithmetic. The normal need not be unit length; its enclosure is supplied
    /// by the source surface, including any correlations its construction retains.
    /// Unsupported nested relative motions return OutsideDomain.
    pub fn normal_velocity_bounds(&self,position: V,normal: V) -> Result<NormalVelocityBounds,Error> {
        let frame = Frame::read(self)?;
        Ok(frame.equation(frame.coefficients(position,normal,false)?))
    }

    /// Derivative of the contact equation along a source parameter. Applies the
    /// product rule to the position and unnormalized normal, including offset
    /// motion axes. The result is still a sinusoid in motion time.
    pub fn normal_velocity_directional_bounds(&self,position: V,normal: V,
        position_derivative: V,normal_derivative: V) -> Result<NormalVelocityBounds,Error> {
        let frame = Frame::read(self)?;
        let a = frame.coefficients(position,normal_derivative,false)?;
        let b = frame.coefficients(position_derivative,normal,true)?;
        // Keep exact time independence when both derivative terms have it.
        let add = |a: I,b: I| if a == I::ZERO && b == I::ZERO { Ok(I::ZERO) } else { a.add(b) };
        Ok(frame.equation([add(a[0],b[0])?,add(a[1],b[1])?,add(a[2],b[2])?]))
    }
}
