//! **Clearance and interference between two solids** (Solvent §9.8) — issue #48, item 7.
//!
//! Claims were the best thing in the tool: a test suite for a drawing, with the diagnosis as its
//! runner.  What they could not say was anything about the *object*: the pocket that held
//! nothing would have failed a claim that the bolt's tension is reacted by material on the nut's
//! side of the head, and that is a statement about solids and not about lines.  These three
//! words are that, and they are **judged and never solved** — a claim about a solid can no more
//! move geometry than a `project` claim can, because nothing three-dimensional is an unknown.
//!
//! **The measurement is exact on the faceted solids, and a bound on the true ones.**  The
//! implicit reading of a term — `min`/`max` over its primitives — is only a *lower* bound for a
//! difference, so it can prove a clearance holds and can never prove one fails; it is used here
//! to skip the pairs that are obviously far apart.  The answer comes from the boundaries
//! themselves, piece against piece, which *is* the distance between the two faceted solids.  What
//! remains approximate is the faceting: a faceted arc lies inside the true one by the sagitta,
//! so a verdict carries that as its uncertainty rather than pretending to be exact.

use crate::csg::Piece;
use crate::model::Sketch;
use crate::plane;
use crate::solid::{self, Box3};

/// What a claim about two solids came to.
#[derive(Clone, Debug)]
pub struct Verdict {
    /// The distance between them, negative where they overlap.
    pub measured: f64,
    /// How far the faceting could be wrong — the sagitta of the coarsest round surface either
    /// solid has.  A verdict nearer than this to its own threshold is *undecided*, which is the
    /// honest answer and not a failure.
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
    // overlap first: a point of one inside the other is an interference, and no distance between
    // boundaries would say so — two solids one inside the other have a positive gap
    if let Some(d) = overlap(a, b, acsg, bcsg, eps) {
        return d;
    }
    let mut best = f64::INFINITY;
    for p in a {
        let pb = grow(&p.bbox_of(), best);
        for q in b {
            if !q.bbox_of().overlaps(&pb) {
                continue;
            }
            let d = piece_gap(p, q);
            if d < best {
                best = d;
            }
        }
    }
    best
}

/// How far *into* each other they reach, as a negative number, or `None` when they are apart.
///
/// Every vertex of each boundary and the middle of every edge of it, classified against the
/// other: the samples are the pieces' own corners rather than a grid, so nothing here is
/// arbitrary and a solid that pokes through a face by a hair is found at the corner that did it.
fn overlap(
    a: &[Piece],
    b: &[Piece],
    acsg: &solid::Csg,
    bcsg: &solid::Csg,
    eps: f64,
) -> Option<f64> {
    let mut deepest = 0.0f64;
    let mut hit = false;
    for (pieces, other) in [(a, bcsg), (b, acsg)] {
        for p in pieces {
            for i in 0..p.pts.len() {
                let v = p.pts[i];
                let w = p.pts[(i + 1) % p.pts.len()];
                let m = [(v[0] + w[0]) / 2.0, (v[1] + w[1]) / 2.0, (v[2] + w[2]) / 2.0];
                for x in [v, m] {
                    // pushed *inward* off the surface, so touching is not overlapping
                    let q = [
                        x[0] - eps * p.n[0],
                        x[1] - eps * p.n[1],
                        x[2] - eps * p.n[2],
                    ];
                    if other.inside(q) {
                        hit = true;
                        deepest = deepest.max(eps);
                    }
                }
            }
        }
    }
    hit.then_some(-deepest)
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
    let eps = sk.extent() * solid::EPS;
    let (pa, pb) = (sk.solid_boundary(a, unit), sk.solid_boundary(b, unit));
    let (ca, cb) = (solid::resolve(sk, a, unit), solid::resolve(sk, b, unit));
    let tol = sagitta(unit);
    let (measured, want) = match word {
        W::Clear => (distance(&pa, &pb, &ca, &cb, eps), gap),
        W::Fits => {
            // inside, with room: the distance from the left's boundary to the right's, signed
            // *positive* while it is contained
            let inside = contained(&pa, &cb, eps);
            let d = boundary_gap(&pa, &pb);
            (if inside { d } else { -d }, gap)
        }
        W::Inside => {
            let inside = contained(&pa, &cb, eps);
            (if inside { 1.0 } else { -1.0 }, 0.0)
        }
    };
    let holds = if word == W::Inside {
        Some(measured > 0.0)
    } else if (measured - want).abs() <= tol {
        None
    } else {
        Some(measured >= want)
    };
    Verdict { measured, tolerance: tol, holds }
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
