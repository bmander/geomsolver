//! Visibility against a prepared solid, shared by drawing projection and the 3D overview.
use crate::{plane, solid::LocalPoint};
use super::{Renderer, spatial::{Bounds, Bvh}};

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
        let index = self.visibility.get_or_init(|| {
            Bvh::new(solid.boundary().iter().map(|f| face_bounds(f, eps)))
        });
        let mut candidates = 0;
        index.query_ray(m, eye, eps, end, |i| {
            candidates += 1;
            let f = &solid.boundary()[i];
            let denominator = plane::dot(f.n, eye);
            if denominator.abs() <= 1e-12 {
                return;
            }
            let t = plane::dot(f.n, std::array::from_fn(|k| f.pts[0][k] - m[k])) / denominator;
            if t <= eps || t >= end {
                return;
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
        });
        let mut stats = self.stats.get();
        stats.visibility_rays += 1;
        stats.boundary_candidates += candidates;
        stats.boundary_exhaustive += solid.boundary().len();
        self.stats.set(stats);
        cuts.sort_by(f64::total_cmp);
        cuts.windows(2).any(|w| {
            w[1] - w[0] > eps * 1e-3 && solid.contains(LocalPoint(step((w[0] + w[1]) * 0.5)))
        })
    }
}

/// Bound the polygon test's tolerance-expanded half-planes, including acute corners.
/// Growing by epsilon alone misses the long miter where two nearly opposing edges meet.
fn face_bounds(face: &crate::csg::Piece, eps: f64) -> Bounds<3> {
    let mut edges = Vec::with_capacity(face.pts.len());
    for i in 0..face.pts.len() {
        let a = face.pts[i];
        let b = face.pts[(i + 1) % face.pts.len()];
        let e: [f64; 3] = std::array::from_fn(|k| b[k] - a[k]);
        let length = e[0].hypot(e[1]).hypot(e[2]);
        if length > 0.0 { edges.push(e.map(|x| x / length)); }
    }
    if edges.len() < 3 {
        return Bounds { lo: [f64::NEG_INFINITY; 3], hi: [f64::INFINITY; 3] };
    }
    let mut miter = 1.0f64;
    for i in 0..edges.len() {
        let cosine = plane::dot(edges[i], edges[(i + 1) % edges.len()]);
        // A collapsed/sliver corner has no useful finite conservative bound.
        if 1.0 + cosine < 1e-12 {
            return Bounds { lo: [f64::NEG_INFINITY; 3], hi: [f64::INFINITY; 3] };
        }
        miter = miter.max((2.0 / (1.0 + cosine)).sqrt());
    }
    let b = face.bbox();
    let scale = b.lo.into_iter().chain(b.hi).fold(1.0f64, |m, x| m.max(x.abs()));
    let b = b.grown(eps / plane::norm(face.n) * miter * 1.000_001 + scale * f64::EPSILON * 16.0);
    Bounds { lo: b.lo, hi: b.hi }
}
