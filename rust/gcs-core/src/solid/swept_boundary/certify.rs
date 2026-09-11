//! The triangle certificate: every triangle probed a fixed distance inside
//! and outside along its own normal, the field asked to read material and
//! exterior there. A triangle that passes lies within that distance of the
//! boundary along its normal at its centroid, on the right side; the
//! chord and vertex tolerances of the construction make that the whole
//! triangle to within the sagitta. Nothing here is inferred from a value's
//! magnitude, and an enclosure containing zero is a failure to certify,
//! never a pass.
use super::judge::{FieldJudge,JudgeError,Sign};

type V3 = [f64;3];

/// Why a triangle failed its certificate.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Failure {
    /// Exterior inside and material outside at the least probe distance:
    /// the triangle faces the wrong way, or is off the boundary.
    Reversed,
    /// The field did not read material `d` inside, nor at any halving.
    InsideNotMaterial(Sign),
    /// The field did not read exterior `d` outside, nor at any halving.
    OutsideNotExterior(Sign),
    /// The triangle has no area to take a normal from.
    Degenerate,
}

#[derive(Clone,Debug,Default)]
pub struct Certificate {
    /// The probe distance every triangle was first tested at.
    pub probe_distance: f64,
    /// The least probe distance any triangle was certified at: material
    /// thinner than the probe distance is certified at a halved distance,
    /// down to `least_distance`, and the statement weakens with it.
    pub least_used: f64,
    /// How many triangles needed a halved distance.
    pub halved: usize,
    /// How many slivers were certified by the boundary passing within the
    /// least distance of their centroid.
    pub slivers: usize,
    pub certified: usize,
    /// Triangles on material or exterior thinner than the least probe
    /// distance: their vertices were bracketed, and at the least distance one
    /// side still reads the boundary or the material beyond a gap. They are
    /// not certified and not wrong; the certificate says so.
    pub thin: Vec<(usize,V3,Failure)>,
    /// Every failing triangle by index, with its centroid and the failure.
    pub failures: Vec<(usize,V3,Failure)>,
}

impl Certificate {
    /// Every triangle certified, thin ones excepted and stated.
    pub fn is_complete(&self) -> bool { self.failures.is_empty() }
}

pub use crate::space::triangle_normal;

/// Certify every triangle of an indexed mesh whose winding faces outward, at
/// probe distance `d`, halving the distance down to `least` for a triangle
/// whose material or exterior is thinner than that. A sliver, with no
/// altitude above `least`, has no normal worth probing along: it lies
/// within `least` of the segment its corners span, and is certified where
/// the field reads the boundary within `least` of its centroid.
pub fn certify(judge: &mut FieldJudge,vertices: &[V3],triangles: &[[u32;3]],d: f64,least: f64) -> Result<Certificate,JudgeError> {
    let mut out = Certificate {probe_distance:d,least_used:d,..Default::default()};
    for (i,t) in triangles.iter().enumerate() {
        let [a,b,c] = t.map(|v| vertices[v as usize]);
        let centroid: V3 = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
        // a triangle with no area is the flattest sliver: no normal, and within `least` of the
        // segment its corners span all the same
        let altitude = crate::space::altitude(a,b,c);
        if altitude < least {
            match judge.sign(centroid)?.0 {
                Sign::Near {within} if within <= least => { out.certified += 1; out.slivers += 1; }
                other => { out.thin.push((i,centroid,Failure::InsideNotMaterial(other))); }
            }
            continue;
        }
        let Some(n) = triangle_normal(a,b,c) else { out.failures.push((i,centroid,Failure::Degenerate)); continue };
        let mut probe = d;
        loop {
            let (inside,outside) = judge.sides(centroid,n,probe)?;
            let last = if inside != Sign::Material { Failure::InsideNotMaterial(inside) } else if outside != Sign::Exterior { Failure::OutsideNotExterior(outside) } else {
                out.certified += 1;
                if probe < d { out.halved += 1; }
                if probe < out.least_used { out.least_used = probe; }
                break;
            };
            if probe/2. < least {
                let reversed = matches!(last,Failure::InsideNotMaterial(Sign::Exterior))
                    || (matches!(last,Failure::OutsideNotExterior(Sign::Material)) && judge.sides(centroid,n,probe)?.0 == Sign::Exterior);
                if reversed { out.failures.push((i,centroid,Failure::Reversed)); } else { out.thin.push((i,centroid,last)); }
                break;
            }
            probe /= 2.;
        }
    }
    Ok(out)
}
