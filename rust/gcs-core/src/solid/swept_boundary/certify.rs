//! A report of strict centroid-side witnesses for candidate triangles. These
//! point checks are not whole-triangle or reverse-coverage certificates; accepted
//! boundaries additionally require the spatial audit. Unknown triangles never pass.
use super::judge::{BoundaryBracket,FieldJudge,JudgeError,Sign};

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
    pub triangles: usize,
    pub surface_area: f64,
    pub unresolved_area: f64,
    pub failed_area: f64,
    /// Actual strict spatial witnesses for each successful centroid check.
    pub brackets: Vec<(usize,BoundaryBracket)>,
    /// The probe distance every triangle was first tested at.
    pub probe_distance: f64,
    /// The least probe distance any triangle was certified at: material
    /// thinner than the probe distance is certified at a halved distance,
    /// down to `least_distance`, and the statement weakens with it.
    pub least_used: f64,
    /// How many triangles needed a halved distance.
    pub halved: usize,
    /// Slivers with successful strict centroid-side witnesses.
    pub slivers: usize,
    pub certified: usize,
    /// Unresolved centroid checks, including thin material, insufficient query
    /// budgets, and unsupported geometry. None is licensed by this report.
    pub unresolved: Vec<(usize,V3,Failure)>,
    /// Every failing triangle by index, with its centroid and the failure.
    pub failures: Vec<(usize,V3,Failure)>,
}

impl Certificate {
    /// Every triangle has a strict centroid bracket. This is only a point-check
    /// completion predicate; whole-surface acceptance also needs the spatial audit.
    pub fn is_complete(&self) -> bool {
        self.triangles > 0 && self.certified == self.triangles && self.failures.is_empty() && self.unresolved.is_empty()
    }
}

pub use crate::space::triangle_normal;

/// Probe triangle centroids, retaining strict witnesses and unresolved outcomes.
/// Halving may resolve thin material; exhaustion is never evidence of thinness.
pub fn certify(judge: &mut FieldJudge,vertices: &[V3],triangles: &[[u32;3]],d: f64,least: f64) -> Result<Certificate,JudgeError> {
    if !d.is_finite() || !least.is_finite() || least <= 0. || d < least {
        return Err(JudgeError::Field("invalid certificate probe distances".into()));
    }
    if vertices.iter().flatten().any(|x| !x.is_finite()) || triangles.iter().flatten().any(|&v| v as usize >= vertices.len()) {
        return Err(JudgeError::Field("invalid certificate mesh".into()));
    }
    let mut out = Certificate {triangles:triangles.len(),probe_distance:d,least_used:d,..Default::default()};
    for (i,t) in triangles.iter().enumerate() {
        let [a,b,c] = t.map(|v| vertices[v as usize]);
        let centroid: V3 = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
        let sliver = crate::space::altitude(a,b,c) < least;
        let Some(n) = triangle_normal(a,b,c) else { out.failures.push((i,centroid,Failure::Degenerate)); continue };
        let mut probe = d;
        loop {
            let (inside,outside) = judge.sides(centroid,n,probe)?;
            let last = if inside != Sign::Material { Failure::InsideNotMaterial(inside) } else if outside != Sign::Exterior { Failure::OutsideNotExterior(outside) } else {
                let inside = std::array::from_fn(|k| centroid[k]-probe*n[k]);
                let outside = std::array::from_fn(|k| centroid[k]+probe*n[k]);
                let Some(bracket) = judge.bracket(inside,outside)? else {
                    out.unresolved.push((i,centroid,Failure::InsideNotMaterial(Sign::Unresolved)));
                    break;
                };
                out.brackets.push((i,bracket));
                out.certified += 1;
                if sliver { out.slivers += 1; }
                if probe < d { out.halved += 1; }
                if probe < out.least_used { out.least_used = probe; }
                break;
            };
            if probe/2. < least {
                let reversed = inside == Sign::Exterior && outside == Sign::Material;
                if reversed { out.failures.push((i,centroid,Failure::Reversed)); } else { out.unresolved.push((i,centroid,last)); }
                break;
            }
            probe /= 2.;
        }
    }
    let area = |i: usize| {
        let [a,b,c] = triangles[i].map(|v| vertices[v as usize]);
        0.5*crate::space::norm(crate::space::cross(crate::space::sub(b,a),crate::space::sub(c,a)))
    };
    out.surface_area = (0..triangles.len()).map(area).sum();
    out.unresolved_area = out.unresolved.iter().map(|(i,_,_)| area(*i)).sum();
    out.failed_area = out.failures.iter().map(|(i,_,_)| area(*i)).sum();
    Ok(out)
}
