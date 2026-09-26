//! Exact support equations and finite-patch incidence for line/circle revolutions.
use super::*;
use std::f64::consts::TAU;

use crate::space::{dot,length};

#[derive(Clone,Debug)]
enum Section {
    Line {start:[f64;2],delta:[f64;2],length:f64},
    Circle {center:[f64;2],a:[f64;2],b:[f64;2],radius:f64,sweep:f64,orientation:f64},
}

/// A support-surface projection and an actual point on the finite source patch.
#[derive(Clone,Copy,Debug)]
pub struct SurfaceProjection {
    /// Parameters on the continued supporting surface. They may be outside [0,1].
    pub parameters: [f64;2],
    /// Oriented meridian line/circle residual in length units. This is a local signed
    /// distance near a regular interior point, not a global closest-distance claim.
    pub signed_residual: f64,
    /// Gradient of signed_residual, following the source tangent orientation.
    pub normal: V,
    /// A point on the finite source patch, obtained by bounding the projected parameters.
    pub point: V,
    /// Distance from the query to `point`. This certifies incidence when small; it is
    /// not asserted to be the minimum distance to a finite patch with multiple edges.
    pub incidence_error: f64,
}

/// A solved snapshot for signed support equations and bounded incidence checks. The
/// meridian must lie on one side of the axis; an axis-crossing parameterization is refused.
#[derive(Clone,Debug)]
pub struct SurfaceProjector {
    source: RevolvedSurface,
    radial: V,
    section: Section,
}

// Select the periodic representative nearest the closed interval [0,sweep]. Do not
// silently turn an off-patch projection into an in-domain parameter by clamping it.
fn phase(angle: f64,sweep: f64) -> f64 {
    let a = angle.rem_euclid(TAU);
    let distance = |a: f64| (-a).max(a-sweep).max(0.);
    if distance(a-TAU) < distance(a) { a-TAU } else { a }
}

impl RevolvedSurface {
    pub fn projector(&self) -> Result<SurfaceProjector,String> {
        let perp = |p: V| sub(p,scale(self.axis,dot(p,self.axis)));
        let candidates = match self.meridian {
            Meridian::Line {start,delta} => vec![sub(start,self.origin),delta],
            Meridian::Round {center,a,b,..} => vec![sub(center,self.origin),a,b],
        };
        let radial = candidates.iter().map(|&p| perp(p))
            .find(|&p| length(p) > 1e-12*candidates.iter().map(|&p| length(p)).fold(0.,f64::max))
            .ok_or("a surface on its axis has no regular meridian")?;
        let radial = scale(radial,1./length(radial));
        let coords = |p: V| [dot(p,radial),dot(p,self.axis)];
        let section = match self.meridian {
            Meridian::Line {start,delta} => {
                let start = coords(sub(start,self.origin));
                let delta = coords(delta);
                let length = delta[0].hypot(delta[1]);
                if !length.is_finite() || length == 0. {
                    return Err("a boundary meridian needs a nondegenerate line".into());
                }
                let rmin = start[0].min(start[0]+delta[0]);
                if rmin < -1e-12*length || start[0].max(start[0]+delta[0]) <= 0. {
                    return Err("a boundary meridian must stay on one side of its axis".into());
                }
                Section::Line {start,delta,length}
            }
            Meridian::Round {center,a,b,sweep} => {
                let center = coords(sub(center,self.origin));
                let a = coords(a); let b = coords(b);
                let radius = a[0].hypot(a[1]);
                let det = a[0]*b[1]-a[1]*b[0];
                if !radius.is_finite() || radius <= 0. || det == 0. {
                    return Err("a boundary meridian needs a regular circle".into());
                }
                let r = |t: f64| center[0]+a[0]*t.cos()+b[0]*t.sin();
                let mut rmin = r(0.).min(r(sweep));
                let extreme = b[0].atan2(a[0]).rem_euclid(TAU);
                for t in [extreme,(extreme+std::f64::consts::PI).rem_euclid(TAU)] {
                    if t <= sweep { rmin = rmin.min(r(t)); }
                }
                if rmin < -1e-12*radius {
                    return Err("a boundary meridian must stay on one side of its axis".into());
                }
                Section::Circle {center,a,b,radius,sweep,orientation:-det.signum()}
            }
        };
        Ok(SurfaceProjector {source:self.clone(),radial,section})
    }
}

impl SurfaceProjector {
    pub(crate) fn surface(&self) -> &RevolvedSurface { &self.source }

    pub fn named(sk: &Sketch, index: usize) -> Result<Self,String> {
        RevolvedSurface::named(sk,index)?.projector()
    }

    pub fn project(&self, p: V) -> Result<SurfaceProjection,Error> {
        if !p.iter().all(|x| x.is_finite()) { return Err(Error::NonFinite); }
        let s = &self.source;
        let q = sub(p,s.origin);
        let z = dot(q,s.axis);
        let rvec = sub(q,scale(s.axis,z));
        let r = length(rvec);
        let direction = if r > 0. { scale(rvec,1./r) } else { self.radial };
        let sign = s.sweep.signum();
        let angle = dot(s.axis,plane::cross(self.radial,direction))
            .atan2(dot(self.radial,direction));
        let [from,to] = s.v_domain.map(|v| v*s.sweep.abs());
        let v = (from+phase(sign*angle-from,to-from))/s.sweep.abs();
        let (u,distance,n) = match self.section {
            Section::Line {start,delta,length} => {
                let q = [r-start[0],z-start[1]];
                let n = [-delta[1]/length,delta[0]/length];
                ((q[0]*delta[0]+q[1]*delta[1])/(length*length),
                    q[0]*n[0]+q[1]*n[1],n)
            }
            Section::Circle {center,a,b,radius,sweep,orientation} => {
                let q = [r-center[0],z-center[1]];
                let d = q[0].hypot(q[1]);
                if d == 0. { return Err(Error::Degenerate); }
                let angle = (q[0]*b[0]+q[1]*b[1]).atan2(q[0]*a[0]+q[1]*a[1]);
                (phase(angle,sweep)/sweep,orientation*(d-radius),
                    [orientation*q[0]/d,orientation*q[1]/d])
            }
        };
        // At an axis point a nonzero radial normal has no unique world direction.
        if r == 0. && n[0].abs() > 1e-12 { return Err(Error::Degenerate); }
        let point = s.at(u.clamp(0.,1.),v.clamp(s.v_domain[0],s.v_domain[1]))?.position;
        let result = SurfaceProjection {parameters:[u,v],signed_residual:sign*distance,
            normal:scale(add(scale(direction,n[0]),scale(s.axis,n[1])),sign),
            point,incidence_error:length(sub(p,point))};
        if !result.parameters.iter().chain(&result.normal).chain(&result.point)
            .chain([&result.signed_residual,&result.incidence_error]).all(|x| x.is_finite()) {
            return Err(Error::NonFinite);
        }
        Ok(result)
    }
}

impl RevolvedSurface {
    /// Intersect a fixed revolution meridian with a sphere. A line meridian gives a
    /// quadratic; return every root on the finite source edge, in parameter order.
    /// This is an exact geometric section, independent of any gear convention.
    pub fn line_on_sphere(&self, center: V, radius: f64, v: f64)
        -> Result<Vec<SurfacePoint>,String> {
        if !matches!(self.meridian,Meridian::Line {..}) {
            return Err("a line/sphere section requires a straight meridian".into());
        }
        if !radius.is_finite() || radius <= 0. || !center.iter().all(|x| x.is_finite()) {
            return Err("a sphere section needs a finite center and positive radius".into());
        }
        let start = self.at(0.,v).map_err(|e| format!("invalid section: {e:?}"))?;
        let end = self.at(1.,v).map_err(|e| format!("invalid section: {e:?}"))?;
        let q = sub(start.position,center);
        let d = sub(end.position,start.position);
        let size = length(q).max(length(d)).max(radius);
        let q = scale(q,1./size); let d = scale(d,1./size); let r = radius/size;
        let a = dot(d,d); let b = dot(q,d); let c = dot(q,q)-r*r;
        if !a.is_finite() || a <= 0. { return Err("a line section needs a regular meridian".into()); }
        let disc = b*b-a*c;
        if !disc.is_finite() { return Err("a sphere section overflowed".into()); }
        if disc < 0. { return Ok(vec![]); }
        let k = -b-disc.sqrt().copysign(b);
        let mut roots = if disc == 0. { vec![-b/a] } else { vec![k/a,c/k] };
        roots.sort_by(f64::total_cmp);
        roots.retain(|&u| (0. ..=1.).contains(&u));
        roots.into_iter().map(|u| self.at(u,v).map_err(|e| format!("invalid section: {e:?}"))).collect()
    }
}
