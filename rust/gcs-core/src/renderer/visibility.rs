//! Visibility against a prepared solid, shared by drawing projection and the 3D overview.
use crate::{plane, solid::LocalPoint};
use super::Renderer;

impl Renderer<'_> {
    /// Orthographic occlusion uses retained boundary crossings and the same material classifier.
    /// A section's limit excludes all material on the discarded side.
    pub fn occludes(&self, m: LocalPoint, eye: [f64; 3], limit: Option<f64>) -> bool {
        let solid = self.solid;
        let eps = solid.epsilon();
        let bounds = solid.bounds();
        if bounds.is_empty() {
            return false;
        }
        let m = m.0;
        let far = (0..3)
            .map(|k| {
                eye[k]
                    * (if eye[k] >= 0.0 {
                        bounds.hi[k]
                    } else {
                        bounds.lo[k]
                    } - m[k])
            })
            .sum::<f64>();
        let end = limit.unwrap_or(far + eps).min(far + eps);
        if end <= eps {
            return false;
        }
        let step = |t: f64| std::array::from_fn(|k| m[k] + t * eye[k]);
        let mut cuts = vec![eps, end];
        for f in solid.boundary() {
            let denominator = plane::dot(f.n, eye);
            if denominator.abs() <= 1e-12 {
                continue;
            }
            let t = plane::dot(f.n, std::array::from_fn(|k| f.pts[0][k] - m[k])) / denominator;
            if t <= eps || t >= end {
                continue;
            }
            let x = step(t);
            if f.pts.iter().enumerate().all(|(i, a)| {
                let b = f.pts[(i + 1) % f.pts.len()];
                let edge = std::array::from_fn(|k| b[k] - a[k]);
                plane::dot(
                    f.n,
                    plane::cross(edge, std::array::from_fn(|k| x[k] - a[k])),
                ) >= -eps * plane::norm(edge)
            }) {
                cuts.push(t);
            }
        }
        cuts.sort_by(f64::total_cmp);
        cuts.windows(2).any(|w| {
            w[1] - w[0] > eps * 1e-3 && solid.contains(LocalPoint(step((w[0] + w[1]) * 0.5)))
        })
    }
}
