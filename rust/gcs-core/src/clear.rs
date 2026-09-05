//! **Clearance and interference between two solids** (Solvent §9.8) — issue #48, item 7.
//!
//! Claims were the best thing in the tool: a test suite for a drawing, with the diagnosis as its
//! runner.  What they could not say was anything about the *object*: the pocket that held
//! nothing would have failed a claim that the bolt's tension is reacted by material on the nut's
//! side of the head, and that is a statement about solids and not about lines.  These three
//! words are that, and they are **judged and never solved** — a claim about a solid can no more
//! move geometry than a `project` claim can, because nothing three-dimensional is an unknown.
//!
//! Separation is the least boundary distance. Interference is measured as common-material
//! thickness: the diameter of the largest ball in the evaluated intersection. This remains
//! a meaningful length for identical, crossing and nonconvex bodies. Its bounded numerical
//! search reports the remaining error, in addition to any curved-faceting uncertainty.

use crate::csg::Piece;
use crate::model::Sketch;
use crate::plane;
use crate::solid::{self, Box3};

/// What a claim about two solids came to.
#[derive(Clone, Debug)]
pub struct Verdict {
    /// Separation distance, or negative common-material thickness when they overlap.
    pub measured: f64,
    /// Measurement uncertainty: curved-faceting error plus any remaining overlap-search error.
    pub tolerance: f64,
    /// True where it holds, false where this drawing is a counterexample, `None` where the
    /// faceting cannot tell.
    pub holds: Option<bool>,
}

/// The distance between two boundaries: the least any piece of one comes to any piece of the
/// other, and negative where the two overlap.
///
/// Culled by bounding box before anything is measured, which is what keeps a part with thirty
/// features from paying for every pair of its faces.
pub fn distance(a: &[Piece], b: &[Piece], acsg: &solid::Csg, bcsg: &solid::Csg, eps: f64) -> f64 {
    // Boundaries can be separated even when one solid contains the other.
    overlap(acsg, bcsg, eps).map_or_else(|| boundary_gap(a, b), |(d, _)| d)
}

/// Negative common-material thickness: the diameter of the largest ball in A ∩ B.
/// Unlike a pushed boundary sample, this is a geometric length, also for identical solids
/// and crossings with no contained vertices. A Lipschitz branch-and-bound search supplies
/// an explicit error interval when the common region is nonconvex or the work limit is hit.
fn overlap(
    acsg: &solid::Csg,
    bcsg: &solid::Csg,
    eps: f64,
) -> Option<(f64, f64)> {
    let common = crate::csg::common_boundary(acsg, bcsg, eps);
    if common.is_empty() || crate::mesh::volume(&common) <= crate::mesh::area(&common) * eps * 1e-3
    {
        return None;
    }
    let bounds = crate::mesh::bounds(&common);
    let mut upper =
        (0..3).map(|k| (bounds.hi[k] - bounds.lo[k]) * 0.5).fold(f64::INFINITY, f64::min);
    // Supporting slabs bound every inscribed ball, whether or not the region is convex.
    for piece in common.iter().step_by((common.len() / 48).max(1)) {
        let origin = piece.pts[0];
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for p in common.iter().flat_map(|p| &p.pts) {
            let h = plane::dot(piece.n, std::array::from_fn(|k| p[k] - origin[k]));
            lo = lo.min(h);
            hi = hi.max(h);
        }
        upper = upper.min((hi - lo) * 0.5);
    }
    let signed = |x: [f64; 3]| {
        let d = common.iter().map(|p| point_to_piece(x, p)).fold(f64::INFINITY, f64::min);
        if acsg.inside(x) && bcsg.inside(x) {
            d
        } else {
            -d
        }
    };
    struct Cell {
        lo: [f64; 3],
        hi: [f64; 3],
        upper: f64,
    }
    impl PartialEq for Cell {
        fn eq(&self, b: &Self) -> bool {
            self.upper == b.upper
        }
    }
    impl Eq for Cell {}
    impl PartialOrd for Cell {
        fn partial_cmp(&self, b: &Self) -> Option<std::cmp::Ordering> {
            Some(self.cmp(b))
        }
    }
    impl Ord for Cell {
        fn cmp(&self, b: &Self) -> std::cmp::Ordering {
            self.upper.total_cmp(&b.upper)
        }
    }
    let center =
        |lo: [f64; 3], hi: [f64; 3]| std::array::from_fn(|k| lo[k] + (hi[k] - lo[k]) * 0.5);
    let mut lower = signed(center(bounds.lo, bounds.hi)).max(0.0).min(upper);
    let tolerance = (upper * 1e-5).max(eps * 1e-3);
    let mut cells = std::collections::BinaryHeap::new();
    cells.push(Cell { lo: bounds.lo, hi: bounds.hi, upper });
    for _ in 0..4096 {
        let Some(cell) = cells.pop() else { break };
        if cell.upper - lower <= tolerance {
            cells.push(cell);
            break;
        }
        let k = (0..3)
            .max_by(|&a, &b| (cell.hi[a] - cell.lo[a]).total_cmp(&(cell.hi[b] - cell.lo[b])))
            .unwrap();
        let mid = center(cell.lo, cell.hi)[k];
        for side in [false, true] {
            let (mut lo, mut hi) = (cell.lo, cell.hi);
            if side {
                lo[k] = mid;
            } else {
                hi[k] = mid;
            }
            let value = signed(center(lo, hi));
            lower = lower.max(value).min(upper);
            let radius = plane::norm(std::array::from_fn(|k| (hi[k] - lo[k]) * 0.5));
            let bound = (value + radius).min(upper);
            if bound > lower {
                cells.push(Cell { lo, hi, upper: bound });
            }
        }
    }
    let remaining = cells.peek().map(|c| c.upper).unwrap_or(lower).max(lower);
    // Diameter lies in [2*lower, 2*remaining]; report its midpoint and half-width.
    Some((-(lower + remaining), remaining - lower))
}

/// Is every point of `a` a point of `b`? Evaluate A − B so an enclosed void is tested too.
pub fn contained(a: &[Piece], b: &solid::Csg, eps: f64) -> bool {
    crate::csg::contains_boundary(b, a, eps)
}

fn grow(b: &Box3, k: f64) -> Box3 {
    if k.is_finite() {
        b.grown(k)
    } else {
        Box3 { lo: [f64::NEG_INFINITY; 3], hi: [f64::INFINITY; 3] }
    }
}

/// The least distance between two convex planar pieces: every vertex against the other's plane
/// and outline, and every pair of edges.  The closed forms rather than an iteration, since a
/// piece has three or four sides and the whole thing is a handful of dot products.
fn piece_gap(p: &Piece, q: &Piece) -> f64 {
    let mut best = f64::INFINITY;
    for (a, b) in [(p, q), (q, p)] {
        for (i, v) in a.pts.iter().enumerate() {
            let w = a.pts[(i + 1) % a.pts.len()];
            let h = plane::dot(b.n, std::array::from_fn(|k| v[k] - b.pts[0][k]));
            let end = plane::dot(b.n, std::array::from_fn(|k| w[k] - b.pts[0][k]));
            if h * end < 0.0 {
                let t = h / (h - end);
                let x = std::array::from_fn(|k| v[k] + t * (w[k] - v[k]));
                if point_to_piece(x, b) <= plane::norm(std::array::from_fn(|k| w[k] - v[k])) * 1e-12
                {
                    return 0.0;
                }
            }
        }
        for v in &a.pts {
            best = best.min(point_to_piece(*v, b));
        }
    }
    for i in 0..p.pts.len() {
        let a = (p.pts[i], p.pts[(i + 1) % p.pts.len()]);
        for j in 0..q.pts.len() {
            let b = (q.pts[j], q.pts[(j + 1) % q.pts.len()]);
            best = best.min(seg_gap(a, b));
        }
    }
    best
}

fn point_to_piece(x: [f64; 3], p: &Piece) -> f64 {
    // the foot of the perpendicular, if it lands inside the outline
    let h = plane::dot(p.n, [x[0] - p.pts[0][0], x[1] - p.pts[0][1], x[2] - p.pts[0][2]]);
    let foot = [x[0] - h * p.n[0], x[1] - h * p.n[1], x[2] - h * p.n[2]];
    let mut inside = true;
    for i in 0..p.pts.len() {
        let a = p.pts[i];
        let b = p.pts[(i + 1) % p.pts.len()];
        let e = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let w = [foot[0] - a[0], foot[1] - a[1], foot[2] - a[2]];
        if plane::dot(p.n, plane::cross(e, w)) < 0.0 {
            inside = false;
            break;
        }
    }
    if inside {
        return h.abs();
    }
    let mut best = f64::INFINITY;
    for i in 0..p.pts.len() {
        let a = p.pts[i];
        let b = p.pts[(i + 1) % p.pts.len()];
        best = best.min(point_to_seg(x, a, b));
    }
    best
}

fn point_to_seg(x: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let dd = plane::dot(d, d);
    let t = if dd > 0.0 {
        (plane::dot([x[0] - a[0], x[1] - a[1], x[2] - a[2]], d) / dd).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let c = [a[0] + t * d[0], a[1] + t * d[1], a[2] + t * d[2]];
    plane::norm([x[0] - c[0], x[1] - c[1], x[2] - c[2]])
}

/// The least distance between two segments, by the clamped closed form.
fn seg_gap(a: ([f64; 3], [f64; 3]), b: ([f64; 3], [f64; 3])) -> f64 {
    let u = [a.1[0] - a.0[0], a.1[1] - a.0[1], a.1[2] - a.0[2]];
    let v = [b.1[0] - b.0[0], b.1[1] - b.0[1], b.1[2] - b.0[2]];
    let w = [a.0[0] - b.0[0], a.0[1] - b.0[1], a.0[2] - b.0[2]];
    let (uu, uv, vv) = (plane::dot(u, u), plane::dot(u, v), plane::dot(v, v));
    let (uw, vw) = (plane::dot(u, w), plane::dot(v, w));
    let den = uu * vv - uv * uv;
    let (mut s, mut t) = if den.abs() > 1e-14 {
        (((uv * vw - vv * uw) / den).clamp(0.0, 1.0), ((uu * vw - uv * uw) / den).clamp(0.0, 1.0))
    } else {
        (0.0, if vv > 0.0 { (vw / vv).clamp(0.0, 1.0) } else { 0.0 })
    };
    // one clamp can put the other off its own segment, so each is re-fitted against the answer
    if vv > 0.0 {
        t = ((vw + s * uv) / vv).clamp(0.0, 1.0);
    }
    if uu > 0.0 {
        s = ((t * uv - uw) / uu).clamp(0.0, 1.0);
    }
    let p = [a.0[0] + s * u[0], a.0[1] + s * u[1], a.0[2] + s * u[2]];
    let q = [b.0[0] + t * v[0], b.0[1] + t * v[1], b.0[2] + t * v[2]];
    plane::norm([p[0] - q[0], p[1] - q[1], p[2] - q[2]])
}

/// How far the faceting of a solid could be wrong: the sagitta the tessellation was refined to.
/// Quoted with every verdict, so a claim decided within it is reported as *undecided* rather than
/// answered by an artefact of the mesh.
pub fn sagitta(unit: f64) -> f64 {
    crate::curve::flatness(unit)
}

/// **Judge one claim about two solids at the pose the drawing stands in.**
pub fn judge(
    sk: &Sketch,
    word: crate::constraints::SolidWord,
    a: usize,
    b: usize,
    gap: f64,
    unit: f64,
) -> Verdict {
    use crate::constraints::SolidWord as W;
    let (pa, pb) = (sk.solid_boundary(a, unit), sk.solid_boundary(b, unit));
    let (ca, cb) = (solid::resolve(sk, a, unit), solid::resolve(sk, b, unit));
    let eps = ca.epsilon().min(cb.epsilon());
    let curved = ca.prims.iter().chain(&cb.prims).any(|p| p.facets.iter().any(|f| f.smooth));
    let facet_tol = if curved { 2.0 * sagitta(unit) } else { 0.0 };
    let compare = |measured: f64| {
        if facet_tol > 0.0 && (measured - gap).abs() <= facet_tol {
            None
        } else {
            Some(measured >= gap)
        }
    };
    match word {
        W::Clear => {
            if let Some((measured, uncertainty)) = overlap(&ca, &cb, eps) {
                let holds = if -measured - uncertainty > facet_tol { Some(false) } else { None };
                Verdict { measured, tolerance: facet_tol + uncertainty, holds }
            } else {
                let measured = boundary_gap(&pa, &pb);
                let holds = match compare(measured) {
                    Some(false) => Some(false),
                    _ if measured == 0.0 && facet_tol == 0.0 => Some(false),
                    _ if measured <= facet_tol => None, // disjointness is still uncertain
                    verdict => verdict,
                };
                Verdict { measured, tolerance: facet_tol, holds }
            }
        }
        W::Fits => {
            let inside = contained(&pa, &cb, eps);
            let d = boundary_gap(&pa, &pb);
            Verdict {
                measured: if inside { d } else { -d },
                tolerance: facet_tol,
                holds: if inside { compare(d) } else { Some(false) },
            }
        }
        W::Inside => {
            let inside = contained(&pa, &cb, eps);
            Verdict {
                measured: if inside { 1.0 } else { -1.0 },
                tolerance: 0.0,
                holds: Some(inside),
            }
        }
    }
}

/// The least the two boundaries come to each other, ignoring which side of which they are on —
/// what `fits` measures once containment has answered the sign.
fn boundary_gap(a: &[Piece], b: &[Piece]) -> f64 {
    let mut best = f64::INFINITY;
    for p in a {
        let pb = grow(&p.bbox_of(), best);
        for q in b {
            if !q.bbox_of().overlaps(&pb) {
                continue;
            }
            best = best.min(piece_gap(p, q));
        }
    }
    best
}
