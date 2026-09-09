//! Enclosures over the original source chart, including its singular poles.
use super::RevolvedSurface;
use crate::{interval::{Interval as I,Error},motion::MotionBounds};

#[derive(Clone,Copy,Debug)]
pub struct SurfaceBounds {
    pub position: [I;3],
    pub du: [I;3],
    pub dv: [I;3],
    /// Unnormalized du cross dv over the underlying source points. Rotation
    /// covariance gives a tighter enclosure than crossing independent du/dv boxes.
    pub normal: [I;3],
    /// Derivative of the unnormalized normal with respect to source v.
    pub normal_dv: [I;3],
}
fn cross(a: [I;3],b: [I;3]) -> Result<[I;3],Error> {
    Ok([a[1].mul(b[2])?.sub(a[2].mul(b[1])?)?,
        a[2].mul(b[0])?.sub(a[0].mul(b[2])?)?,
        a[0].mul(b[1])?.sub(a[1].mul(b[0])?)?])
}
impl RevolvedSurface {
    /// Every position and first derivative over a complete source parameter box.
    /// This encloses the solved binary64 snapshot, not the source solve error.
    pub fn bounds(&self,u: I,v: I) -> Result<SurfaceBounds,Error> {
        let [lo,hi] = v.bounds();
        if lo < self.v_domain[0] || hi > self.v_domain[1] { return Err(Error::OutsideDomain); }
        let (p,du) = self.generating_profile_bounds(u)?;
        let rotation = MotionBounds::rotation(self.origin,self.axis,v.mul(I::point(self.sweep)?)?)?;
        let mut axis = self.axis.map(|x| I::point(x).unwrap());
        let length = axis[0].square()?.add(axis[1].square()?)?.add(axis[2].square()?)?.sqrt()?;
        for x in &mut axis { *x = x.div(length)?; }
        let mut radial = p;
        for k in 0..3 { radial[k] = radial[k].sub(I::point(self.origin[k])?)?; }
        let mut tangent = cross(axis,radial)?;
        for x in &mut tangent { *x = x.mul(I::point(self.sweep)?)?; }
        let normal = cross(du,tangent)?;
        let mut normal_dv = cross(axis,normal)?;
        for x in &mut normal_dv { *x = x.mul(I::point(self.sweep)?)?; }
        Ok(SurfaceBounds {position:rotation.point(p)?,du:rotation.vector(du)?,dv:rotation.vector(tangent)?,
            normal:rotation.vector(normal)?,normal_dv:rotation.vector(normal_dv)?})
    }
}
