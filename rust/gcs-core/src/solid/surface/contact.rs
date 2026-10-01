//! Contact curves of a revolved surface under an instantaneous rigid motion.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;
use crate::envelope::{self,Contact};
use std::f64::consts::TAU;

/// One isolated root of the contact equation. Branch labels are the two analytic
/// roots, not a sort by wrapped v; that sorting would exchange sheets at a seam.
#[derive(Clone,Copy,Debug)]
pub struct RevolvedContact {
    pub branch: usize,
    pub v: f64,
    pub contact: Contact,
}

impl RevolvedSurface {
    /// Apply a fixed placement to this snapshot. Only the pose is used; a motion's
    /// derivative does not turn this placement into another generating motion.
    pub fn placed(&self,pose: Motion) -> Self {
        let meridian = match self.meridian {
            Meridian::Line {start,delta} => Meridian::Line {
                start:pose.point(start),delta:pose.vector(delta)},
            Meridian::Round {center,a,b,sweep} => Meridian::Round {
                center:pose.point(center),a:pose.vector(a),b:pose.vector(b),sweep},
        };
        Self {meridian,origin:pose.point(self.origin),axis:pose.vector(self.axis),..self.clone()}
    }

    /// Solve n dot velocity = 0 around one meridian station, without a seed or
    /// angular march. For a surface of revolution this is A cos(theta) +
    /// B sin(theta) + C = 0 under any instantaneous rigid motion.
    ///
    /// This produces candidate contact points, not a trimmed swept boundary.
    /// Poles, stationary rings and near-double roots return Degenerate, so topology
    /// events cannot silently become an empty curve. Tolerance is normal velocity
    /// in model length per motion parameter; this is not an interval root proof.
    pub fn contacts(&self,u: f64,motion: Motion,tolerance: f64)
        -> Result<Vec<RevolvedContact>,Error> {
        envelope::check_tolerance(tolerance)?;
        if tolerance == 0. { return Err(Error::InvalidOptions); }
        let (a,b,c) = self.contact_coefficients(u,motion)?;
        let mut roots = Vec::new();
        for (branch,v) in sinusoid_roots(a,b,c,self.sweep,self.v_domain,tolerance)? {
            let contact = envelope::contact(self.at(u,v)?,motion)?;
            if contact.normal_velocity.abs() > tolerance { return Err(Error::NotConverged); }
            roots.push(RevolvedContact {branch,v,contact});
        }
        Ok(roots)
    }

    /// The coefficients of the ring's contact equation `a cos + b sin + c = 0`
    /// at meridian station `u` under the motion's instantaneous twist. A zero
    /// amplitude with zero constant is a stationary ring, every point of which
    /// is in contact.
    pub fn contact_coefficients(&self,u: f64,motion: Motion) -> Result<(f64,f64,f64),Error> {
        // The coefficients use the original meridian even for a restricted span.
        let full = Self {v_domain:[0.,1.],..self.clone()};
        let s = full.at(u,0.)?;
        let normal = envelope::contact(s,Motion::identity())?.normal;
        let q = sub(s.position,self.origin);
        let axial = plane::dot(q,self.axis);
        let radial = sub(q,scale(self.axis,axial));
        let radius = radial[0].dhypot(radial[1]).dhypot(radial[2]);
        if radius == 0. || !radius.is_finite() { return Err(Error::Degenerate); }
        let e = scale(radial,1./radius);
        let f = plane::cross(self.axis,e);
        let nr = plane::dot(normal,e); let nz = plane::dot(normal,self.axis);
        // Source-frame angular velocity is the axial vector of R^T R'.
        let inverse = motion.inverse();
        let velocity = |p| inverse.vector(motion.velocity(p));
        let v0 = velocity([0.;3]);
        let columns: [V;3] = std::array::from_fn(|i| {
            let mut p = [0.;3]; p[i] = 1.; sub(velocity(p),v0)
        });
        let omega = [0.5*(columns[1][2]-columns[2][1]),
            0.5*(columns[2][0]-columns[0][2]),0.5*(columns[0][1]-columns[1][0])];
        let center_velocity = velocity(add(self.origin,scale(self.axis,axial)));
        let h = add(scale(center_velocity,nr),scale(plane::cross(self.axis,omega),radius*nz));
        Ok((plane::dot(h,e),plane::dot(h,f),nz*plane::dot(center_velocity,self.axis)))
    }

    /// The revolution's unit axis direction.
    pub fn axis_direction(&self) -> [f64;3] { self.axis }
}

/// Isolated roots on a normalized angular span.
fn sinusoid_roots(a: f64,b: f64,c: f64,sweep: f64,domain: [f64;2],tolerance: f64)
    -> Result<Vec<(usize,f64)>,Error> {
    let amplitude = a.dhypot(b);
    if ![a,b,c,amplitude].iter().all(|v| v.is_finite()) { return Err(Error::NonFinite); }
    if c.abs() > amplitude+tolerance { return Ok(Vec::new()); }
    if amplitude <= tolerance || (c.abs()-amplitude).abs() <= tolerance {
        return Err(Error::Degenerate);
    }
    let phase = b.datan2(a); let spread = (-c/amplitude).dacos();
    let mut roots = Vec::new();
    for (branch,theta) in [phase-spread,phase+spread].into_iter().enumerate() {
        let v = (theta*sweep.signum()).rem_euclid(TAU)/sweep.abs();
        // The endpoint at 1 also represents the seam at 0 for a full turn.
        let period = TAU/sweep.abs();
        let Some(v) = [v,v-period,v+period].into_iter().find(|v|
            *v >= domain[0] && *v <= domain[1]) else { continue };
        roots.push((branch,v));
    }
    Ok(roots)
}
