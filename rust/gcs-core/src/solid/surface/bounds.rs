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
    /// Derivative of the unnormalized normal with respect to source u.
    pub normal_du: [I;3],
    /// Derivative of the unnormalized normal with respect to source v.
    pub normal_dv: [I;3],
    /// Position cross the unnormalized normal, retaining their common rotation.
    pub moment: [I;3],
    pub moment_du: [I;3],
    pub moment_dv: [I;3],
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
        let [p,du,duu] = self.generating_profile_jet_bounds(u)?;
        let rotation = MotionBounds::rotation([0.;3],self.axis,v.mul(I::point(self.sweep)?)?)?;
        let mut axis = self.axis.map(|x| I::point(x).unwrap());
        let length = axis[0].square()?.add(axis[1].square()?)?.add(axis[2].square()?)?.sqrt()?;
        for x in &mut axis { *x = x.div(length)?; }
        let mut radial = p;
        for k in 0..3 { radial[k] = radial[k].sub(I::point(self.origin[k])?)?; }
        // Keep the common origin outside the interval rotation. Expanding this
        // as R*p + (origin - R*origin) loses the two R terms' correlation and
        // makes the box grow with an arbitrary world-coordinate translation.
        let mut position = rotation.vector(radial)?;
        for k in 0..3 { position[k] = position[k].add(I::point(self.origin[k])?)?; }
        let mut tangent = cross(axis,radial)?;
        for x in &mut tangent { *x = x.mul(I::point(self.sweep)?)?; }
        let normal = cross(du,tangent)?;
        let mut tangent_du = cross(axis,du)?;
        for x in &mut tangent_du { *x = x.mul(I::point(self.sweep)?)?; }
        let mut normal_du = cross(duu,tangent)?;
        let second = cross(du,tangent_du)?;
        for k in 0..3 { normal_du[k] = normal_du[k].add(second[k])?; }
        let mut normal_dv = cross(axis,normal)?;
        for x in &mut normal_dv { *x = x.mul(I::point(self.sweep)?)?; }
        let moment = cross(radial,normal)?;
        let mut moment_du = cross(du,normal)?;
        let term = cross(radial,normal_du)?;
        for k in 0..3 { moment_du[k] = moment_du[k].add(term[k])?; }
        let mut moment_dv = cross(axis,moment)?;
        for x in &mut moment_dv { *x = x.mul(I::point(self.sweep)?)?; }
        let normal = rotation.vector(normal)?;
        let normal_du = rotation.vector(normal_du)?;
        let normal_dv = rotation.vector(normal_dv)?;
        let place_moment = |m,n| -> Result<[I;3],Error> {
            let mut m = rotation.vector(m)?;
            let offset = cross(self.origin.map(|x| I::point(x).unwrap()),n)?;
            for k in 0..3 { m[k] = m[k].add(offset[k])?; }
            Ok(m)
        };
        Ok(SurfaceBounds {position,du:rotation.vector(du)?,dv:rotation.vector(tangent)?,
            normal,normal_du,normal_dv,moment:place_moment(moment,normal)?,
            moment_du:place_moment(moment_du,normal_du)?,moment_dv:place_moment(moment_dv,normal_dv)?})
    }
}
