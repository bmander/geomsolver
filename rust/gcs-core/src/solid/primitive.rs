//! Faceted prism and revolution primitives.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;

// -- sweeping ----------------------------------------------------------------------------------

/// Ear clipping: a simple polygon as triangles, by index, preserving its winding.  The one
/// triangulation in the kernel, used for a prism's caps and a partial revolution's.
fn ears(pts: &[(f64, f64)]) -> Vec<[usize; 3]> {
    let n = pts.len();
    if n < 3 {
        return Vec::new();
    }
    let area2 = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| {
        (b.0 - a.0) * (c.1 - a.1) - (c.0 - a.0) * (b.1 - a.1)
    };
    // A revolution's meridian can have either winding, depending on which side of the axis
    // the face lies. Test convexity and containment in that winding; keep the original index
    // order so the caller can still orient each cap against the sweep.
    let signed_area: f64 = (1..n - 1).map(|i| area2(pts[0], pts[i], pts[i + 1])).sum();
    let winding = if signed_area < 0.0 { -1.0 } else { 1.0 };
    let oriented_area = |a, b, c| winding * area2(a, b, c);
    let mut idx: Vec<usize> = (0..n).collect();
    let mut out = Vec::with_capacity(n.saturating_sub(2));
    let mut guard = 0;
    while idx.len() > 3 && guard < 4 * n + 16 {
        let m = idx.len();
        let mut cut = false;
        for i in 0..m {
            let (ia, ib, ic) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
            let (a, b, c) = (pts[ia], pts[ib], pts[ic]);
            if oriented_area(a, b, c) <= 0.0 {
                continue;
            }
            let clear = idx.iter().all(|&k| {
                if k == ia || k == ib || k == ic {
                    return true;
                }
                let p = pts[k];
                !(oriented_area(a, b, p) >= 0.0
                    && oriented_area(b, c, p) >= 0.0
                    && oriented_area(c, a, p) >= 0.0)
            });
            if clear {
                out.push([ia, ib, ic]);
                idx.remove(i);
                cut = true;
                break;
            }
        }
        if !cut {
            break;
        }
        guard += 1;
    }
    if idx.len() == 3 {
        out.push([idx[0], idx[1], idx[2]]);
    }
    out
}

/// A face swept along its plane's normal, from `lo` to `hi` — a prism (§6.9).
pub fn prism(poly: &FacePoly, lo: f64, hi: f64, of: &str) -> Option<Prim> {
    let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
    if hi - lo <= 0.0 {
        return None;
    }
    let n = poly.pts.len();
    let nrm = poly.basis.normal();
    let at = |i: usize, k: f64| {
        let p = poly.lift(i);
        [p[0] + k * nrm[0], p[1] + k * nrm[1], p[2] + k * nrm[2]]
    };
    // face 0 is `far` (at lo), face 1 is `near` (at hi), then one per drawn edge
    let mut faces = vec!["far".to_string(), "near".to_string()];
    faces.extend(poly.names.iter().cloned());
    let mut facets = Vec::new();
    for t in ears(&poly.pts) {
        // The loop is CCW in view coordinates, so a triangle taken in its own order has the
        // plane's own normal by the right-hand rule.  The cap at `hi` is the far end along +n
        // and keeps that winding; the cap at `lo` faces the other way and is reversed.  The
        // winding and the declared normal must agree or the divergence sum reads the volume of
        // a surface that is inside out — which is exactly what a cap contributing nothing at
        // the origin will hide from every test that does not stand the solid away from it.
        facets.push(Facet {
            pts: vec![at(t[0], hi), at(t[1], hi), at(t[2], hi)],
            n: nrm,
            face: 1,
            smooth: false,
        });
        facets.push(Facet {
            pts: vec![at(t[2], lo), at(t[1], lo), at(t[0], lo)],
            n: plane::scaled(nrm, -1.0),
            face: 0,
            smooth: false,
        });
    }
    for i in 0..n {
        let j = (i + 1) % n;
        let (edge, smooth) = poly.of[i];
        let quad = vec![at(i, lo), at(j, lo), at(j, hi), at(i, hi)];
        let Some(nn) = facet_normal(&quad) else { continue };
        facets.push(Facet { pts: quad, n: nn, face: 2 + edge, smooth });
    }
    Some(finish(facets, faces, of))
}

/// A face swept about a line in its own plane — a revolution (§6.9, §17.1's `ring` about a line).
pub fn revolve(
    poly: &FacePoly,
    axis: ((f64, f64), (f64, f64)),
    sweep: f64,
    sense: Sense,
    unit: f64,
    of: &str,
) -> Option<Prim> {
    let full = sweep >= std::f64::consts::TAU - 1e-9;
    let sweep = if full { std::f64::consts::TAU } else { sweep };
    if sweep <= 0.0 {
        return None;
    }
    // the axis in the plane's own 2D coordinates, and the meridian frame off it
    let (a2, b2) = axis;
    let dir2 = (b2.0 - a2.0, b2.1 - a2.1);
    let len = dir2.0.dhypot(dir2.1);
    if len <= 0.0 {
        return None;
    }
    let dir2 = (dir2.0 / len, dir2.1 / len);
    // signed distance across the axis; the face must lie on one side of it
    let across = |p: (f64, f64)| -(p.0 - a2.0) * dir2.1 + (p.1 - a2.1) * dir2.0;
    let along = |p: (f64, f64)| (p.0 - a2.0) * dir2.0 + (p.1 - a2.1) * dir2.1;
    let s: Vec<f64> = poly.pts.iter().map(|p| across(*p)).collect();
    let scale = s.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    let tol = scale * SNAP;
    if s.iter().any(|v| *v > tol) && s.iter().any(|v| *v < -tol) {
        return None; // the axis crosses the face: a double cover, refused
    }
    let flip = s.iter().any(|v| *v < -tol);
    let sign = if flip { -1.0 } else { 1.0 };
    // the meridian: (r, z) per vertex, r ≥ 0
    // A solved point on the axis can miss it by roundoff. Collapse that radius using
    // the same tolerance as the axis-side check, so it cannot mint a microscopic tube.
    let mer: Vec<(f64, f64)> = poly.pts.iter().map(|p| {
        let r = across(*p);
        (if r.abs() <= tol { 0.0 } else { sign * r }, along(*p))
    }).collect();

    // the frame in space: W along the axis, P the in-plane perpendicular the face lies on, and
    // Q = W × P, which is ±n — right-handed about the axis's own p1 → p2
    let o3 = poly.basis.lift(a2.0, a2.1);
    let w = std::array::from_fn(|k| dir2.0 * poly.basis.u[k] + dir2.1 * poly.basis.v[k]);
    let pdir = plane::scaled(plane::unit(plane::cross(poly.basis.normal(), w))?, sign);
    let qdir = plane::cross(w, pdir);
    let turn = if sense == Sense::Cw { -1.0 } else { 1.0 };
    let at = |r: f64, z: f64, phi: f64| {
        let (sp, cp) = (turn * phi).dsin_cos();
        [
            o3[0] + z * w[0] + r * (cp * pdir[0] + sp * qdir[0]),
            o3[1] + z * w[1] + r * (cp * pdir[1] + sp * qdir[1]),
            o3[2] + z * w[2] + r * (cp * pdir[2] + sp * qdir[2]),
        ]
    };
    // faceted about the axis by the same sagitta rule an arc is drawn by, on the widest radius
    let rmax = mer.iter().fold(0.0f64, |m, p| m.max(p.0));
    let tolf = crate::curve::flatness(unit);
    let step = (if rmax > tolf { 2.0 * (1.0 - tolf / rmax).dacos() } else { std::f64::consts::TAU })
        .min(std::f64::consts::TAU / 64.0);
    let steps = ((sweep / step).ceil() as usize).clamp(3, 2048);

    let n = mer.len();
    // a revolution's faces: one per drawn edge, then `start` and `end` for a partial turn
    let mut faces: Vec<String> = poly.names.clone();
    let (fi_start, fi_end) = (faces.len(), faces.len() + 1);
    faces.push("start".into());
    faces.push("end".into());

    // the meridian loop must run so that the swept surface faces outward; with r ≥ 0 and the
    // frame above, a CCW loop in (r, z) gives an outward normal under the right-hand rule
    let mer_ccw = {
        let mut a = 0.0;
        for i in 0..n {
            let (x0, y0) = mer[i];
            let (x1, y1) = mer[(i + 1) % n];
            a += x0 * y1 - x1 * y0;
        }
        a * turn >= 0.0
    };

    let mut facets = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        let (edge, _) = poly.of[i];
        let (r0, z0) = mer[i];
        let (r1, z1) = mer[j];
        for k in 0..steps {
            let p0 = sweep * k as f64 / steps as f64;
            let p1 = sweep * (k + 1) as f64 / steps as f64;
            // A meridian running counter-clockwise in `(r, z)` has the outer wall going up, and
            // the quad that faces *away* from the axis there is the one taken against the sweep.
            // Wound the other way the whole solid is inside out, which the divergence sum reports
            // as a negative volume and nothing else notices.
            let mut quad = if mer_ccw {
                vec![at(r0, z0, p1), at(r1, z1, p1), at(r1, z1, p0), at(r0, z0, p0)]
            } else {
                vec![at(r0, z0, p0), at(r1, z1, p0), at(r1, z1, p1), at(r0, z0, p1)]
            };
            // The two angular chords are parallel, so a swept meridian segment is a
            // planar trapezoid. On the axis it collapses to a triangle (or nothing).
            quad.dedup();
            if quad.first() == quad.last() { quad.pop(); }
            let Some(nn) = facet_normal(&quad) else { continue };
            facets.push(Facet { pts: quad, n: nn, face: edge, smooth: true });
        }
    }
    if !full {
        for (phi, fi, rev) in [(0.0, fi_start, true), (sweep, fi_end, false)] {
            for t in ears(&mer) {
                let mut pts: Vec<[f64; 3]> =
                    t.iter().map(|&k| at(mer[k].0, mer[k].1, phi)).collect();
                // the start cap faces back along the sweep and the end cap forward, so exactly
                // one of the two keeps the meridian's own winding
                if rev != mer_ccw {
                    pts.reverse();
                }
                let Some(nn) = facet_normal(&pts) else { continue };
                facets.push(Facet { pts, n: nn, face: fi, smooth: false });
            }
        }
    }
    Some(finish(facets, faces, of))
}

pub(super) fn facet_normal(pts: &[[f64; 3]]) -> Option<[f64; 3]> {
    let acc = area_vector(pts);
    let length = plane::norm(acc);
    // This is an area vector, not a direction seed: a small valid face has a small vector.
    (length > 0.0 && length.is_finite()).then(|| plane::scaled(acc, 1.0 / length))
}

/// Twice the oriented polygon area, computed relative to a vertex to avoid cancellation.
pub(crate) fn area_vector(pts: &[[f64; 3]]) -> [f64; 3] {
    let mut acc = [0.0; 3];
    if pts.len() < 3 { return acc; }
    let local = |p: [f64; 3]| [p[0] - pts[0][0], p[1] - pts[0][1], p[2] - pts[0][2]];
    for i in 1..pts.len() - 1 {
        let cross = plane::cross(local(pts[i]), local(pts[i + 1]));
        for k in 0..3 { acc[k] += cross[k]; }
    }
    acc
}

pub(super) fn finish(facets: Vec<Facet>, faces: Vec<String>, of: &str) -> Prim {
    let mut bbox = Box3::empty();
    for f in &facets {
        for p in &f.pts {
            bbox.add(*p);
        }
    }
    Prim { facets, bbox, faces, of: of.to_string(), exact: false }
}
