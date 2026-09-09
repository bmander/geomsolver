//! The alternate exact contact chart: fix revolution angle, solve profile position.
use super::*;
use crate::envelope::{self,Contact};

#[derive(Clone,Copy,Debug)]
pub struct MeridianContact {
    /// Algebraic meridian branch, independent of angular-chart branch labels.
    pub branch: usize,
    pub u: f64,
    pub contact: Contact,
}

impl RevolvedSurface {
    /// Solve contact along a meridian rather than around a ring. For a line the
    /// normal direction is constant and normal velocity is affine in u. For a
    /// circle its radial normal annihilates the rotational velocity about the
    /// circle center, leaving A cos(phi)+B sin(phi). This chart can cross a fold
    /// of `contacts` without stepping or fitting through a missing root.
    ///
    /// Poles and non-isolated contacts remain explicit errors. This locates roots
    /// on the source patch, not its Boolean trims or the globally exposed sweep.
    pub fn meridian_contacts(&self,v: f64,motion: Motion,tolerance: f64)
        -> Result<Vec<MeridianContact>,Error> {
        envelope::check_tolerance(tolerance)?;
        if tolerance == 0. { return Err(Error::InvalidOptions); }
        if !motion.is_finite() { return Err(Error::NonFinite); }
        let start = self.at(0.,v)?;
        let roots = match &self.meridian {
            Meridian::Line {..} => {
                let mut normal = None;
                for u in [0.,0.5,1.] {
                    match envelope::contact(self.at(u,v)?,motion) {
                        Ok(c) => { normal = Some(c.normal); break; },
                        Err(Error::Degenerate) => continue,
                        Err(error) => return Err(error),
                    }
                }
                let normal = normal.ok_or(Error::Degenerate)?;
                let a = plane::dot(normal,motion.velocity(start.position));
                let b = plane::dot(normal,motion.velocity(self.at(1.,v)?.position));
                let slope = b-a;
                if !a.is_finite() || !b.is_finite() || !slope.is_finite() { return Err(Error::NonFinite); }
                if slope.abs() <= tolerance {
                    return if a.abs() > tolerance+slope.abs() { Ok(Vec::new()) } else { Err(Error::Degenerate) };
                }
                let u = -a/slope;
                if (0. ..=1.).contains(&u) { vec![(0,u)] } else { Vec::new() }
            }
            Meridian::Round {center,a,b,sweep} => {
                let rotation = Motion::rotation(self.axis,v*self.sweep,0.)?;
                let center = add(self.origin,rotation.vector(sub(*center,self.origin)));
                let radius = a[0].hypot(a[1]).hypot(a[2]);
                let normal_a = motion.vector(rotation.vector(scale(*a,1./radius)));
                let normal_b = motion.vector(rotation.vector(scale(*b,1./radius)));
                let velocity = motion.velocity(center);
                contact::sinusoid_roots(plane::dot(normal_a,velocity),plane::dot(normal_b,velocity),
                    0.,*sweep,[0.,1.],tolerance)?
            }
        };
        roots.into_iter().map(|(branch,u)| {
            let contact = envelope::contact(self.at(u,v)?,motion)?;
            if contact.normal_velocity.abs() > tolerance { return Err(Error::NotConverged); }
            Ok(MeridianContact {branch,u,contact})
        }).collect()
    }
}
