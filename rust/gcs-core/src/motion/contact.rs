//! Temporal contact charts for a rotation viewed from another rotating frame.
use super::{Family,Step};
use crate::{envelope::{self,Error,Motion,SurfacePoint},plane::{cross,dot}};
use std::f64::consts::TAU;
mod bounds;
pub use bounds::NormalVelocityBounds;

/// Normal velocity at one fixed source point under a supported motion family.
/// f(t) = constant + cosine*cos(phase+rate*t) + sine*sin(phase+rate*t).
/// Coefficients describe the solved floating-point geometry, not interval bounds.
#[derive(Clone,Copy,Debug)]
pub struct NormalVelocity {
    constant: f64,
    cosine: f64,
    sine: f64,
    rate: f64,
    phase: f64,
}

/// One isolated temporal root. Branch and turn identify the algebraic root and
/// winding at this source point; they are not global surface-sheet identities.
#[derive(Clone,Copy,Debug)]
pub struct ContactTime {
    pub time: f64,
    pub branch: usize,
    pub turn: i64,
}

impl Family {
    /// For M = B^-1 A with fixed-axis rotations, the observer's rotation cancels
    /// from n dot velocity. In the shared world frame, with x=p-origin_A,
    /// f = rate_A*n dot (axis_A cross x)
    ///     - omega_B dot R_A(x cross n)
    ///     - (omega_B cross (origin_A-origin_B)) dot R_A n.
    /// Rodrigues' formula makes this a single sinusoid in A's angle. Different
    /// axis origins, signed rates and source phase are retained. Observer phase
    /// changes the contact position, but not its time. Nested relative motions
    /// can contain additional harmonics and are explicitly unsupported here.
    pub fn normal_velocity(&self,surface: SurfacePoint) -> Result<NormalVelocity,String> {
        let (source,observer) = match *self.steps.last().expect("motion root") {
            Step::Rotation {..} => (self.steps.last().unwrap(),None),
            Step::Relative {source,observer} => (&self.steps[source],Some(&self.steps[observer])),
        };
        let Step::Rotation {origin,axis,ratio,phase} = *source else {
            return Err("temporal contacts require a rotation or two relative rotations".into());
        };
        let length = |v: [f64;3]| v[0].hypot(v[1]).hypot(v[2]);
        let axis = axis.map(|v| v/length(axis));
        let (omega,offset) = match observer {
            None => ([0.;3],[0.;3]),
            Some(Step::Rotation {origin:other,axis,ratio,..}) =>
                (axis.map(|v| v/length(*axis)*ratio),std::array::from_fn(|k| origin[k]-other[k])),
            Some(_) => return Err("temporal contacts require a rotation or two relative rotations".into()),
        };
        let n = envelope::contact(surface,Motion::identity()).map_err(|e| format!("{e:?}"))?.normal;
        let x = std::array::from_fn(|k| surface.position[k]-origin[k]);
        let mut coefficients = [ratio*dot(n,cross(axis,x)),0.,0.];
        for (v,w) in [(omega,cross(x,n)),(cross(omega,offset),n)] {
            let parallel = axis.map(|a| a*dot(axis,w));
            let perpendicular = std::array::from_fn(|k| w[k]-parallel[k]);
            coefficients[0] -= dot(v,parallel);
            coefficients[1] -= dot(v,perpendicular);
            coefficients[2] -= dot(v,cross(axis,w));
        }
        if !coefficients.iter().all(|v| v.is_finite()) {
            return Err("normal-velocity coefficients overflowed".into());
        }
        Ok(NormalVelocity {constant:coefficients[0],cosine:coefficients[1],sine:coefficients[2],
            rate:ratio,phase})
    }
}

impl NormalVelocity {
    pub fn at(&self,time: f64) -> Result<f64,Error> {
        let angle = self.phase+self.rate*time;
        let value = self.constant+self.cosine*angle.cos()+self.sine*angle.sin();
        if time.is_finite() && value.is_finite() { Ok(value) } else { Err(Error::NonFinite) }
    }

    /// Enumerate isolated roots in a closed time interval. Constant-zero and
    /// near-double-root cases need another chart and return Degenerate. A root
    /// budget failure returns no partial result. Tolerance is normal velocity,
    /// not a root-position or end-to-end geometry error bound.
    pub fn roots(&self,domain: [f64;2],tolerance: f64,max_roots: usize)
        -> Result<Vec<ContactTime>,Error> {
        envelope::check_tolerance(tolerance)?;
        if tolerance == 0. || max_roots == 0 || max_roots > 1_000_000 || domain[0] > domain[1] {
            return Err(Error::InvalidOptions);
        }
        if !domain.iter().all(|v| v.is_finite()) { return Err(Error::NonFinite); }
        let amplitude = self.cosine.hypot(self.sine);
        if !amplitude.is_finite() { return Err(Error::NonFinite); }
        if self.rate == 0. || amplitude <= tolerance {
            return if self.at(domain[0])?.abs() <= tolerance { Err(Error::Degenerate) }
                else if self.rate == 0. || self.constant.abs() > amplitude+tolerance { Ok(Vec::new()) }
                else { Err(Error::Degenerate) };
        }
        if self.constant.abs() > amplitude+tolerance { return Ok(Vec::new()); }
        if (self.constant.abs()-amplitude).abs() <= tolerance { return Err(Error::Degenerate); }
        let phase = self.sine.atan2(self.cosine);
        let spread = (-self.constant/amplitude).acos();
        let angles = domain.map(|t| self.phase+self.rate*t);
        let lo = angles[0].min(angles[1]); let hi = angles[0].max(angles[1]);
        if !lo.is_finite() || !hi.is_finite() { return Err(Error::NonFinite); }
        let mut result = Vec::new();
        for (branch,base) in [phase-spread,phase+spread].into_iter().enumerate() {
            // Include neighboring windings so endpoint arithmetic cannot exclude
            // a root by one ulp. Only roundoff-sized endpoint overruns are clamped.
            let first = ((lo-base)/TAU).floor()-1.;
            let last = ((hi-base)/TAU).ceil()+1.;
            if first.abs().max(last.abs()) >= (1_u64<<52) as f64
                || last-first > max_roots as f64+8. { return Err(Error::NotConverged); }
            for turn in first as i64..=last as i64 {
                let angle = base+TAU*turn as f64;
                let angular_roundoff = 16.*f64::EPSILON*(angle.abs()+lo.abs()+hi.abs()+1.);
                if angle < lo-angular_roundoff || angle > hi+angular_roundoff { continue; }
                let time = (angle-self.phase)/self.rate;
                if !time.is_finite() { return Err(Error::NonFinite); }
                let clamped = time.clamp(domain[0],domain[1]);
                let roundoff = 16.*f64::EPSILON*(time.abs()+clamped.abs()+1.);
                if (time-clamped).abs() > roundoff { continue; }
                if self.at(clamped)?.abs() > tolerance { return Err(Error::NotConverged); }
                result.push(ContactTime {time:clamped,branch,turn});
                if result.len() > max_roots { return Err(Error::NotConverged); }
            }
        }
        result.sort_by(|a,b| a.time.total_cmp(&b.time));
        if result.windows(2).any(|p| p[0].time == p[1].time) { return Err(Error::Degenerate); }
        Ok(result)
    }
}
