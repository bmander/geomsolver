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
use crate::solid;

mod evidence;
pub use evidence::{
    GeometricEvidence, Interval, Measurement, Nearest, Predicate, Verdict,
};

/// The distance between two boundaries: the least any piece of one comes to any piece of the
/// other, and negative where the two overlap.
///
/// Culled by bounding box before anything is measured, which is what keeps a part with thirty
/// features from paying for every pair of its faces.
pub fn distance(a: &[Piece], b: &[Piece], acsg: &solid::Csg, bcsg: &solid::Csg, eps: f64) -> f64 {
    // Boundaries can be separated even when one solid contains the other.
    overlap(acsg, bcsg, eps).map_or_else(|| boundary_gap(a, b).0, |(d, _, _)| d)
}

/// Negative common-material thickness: the diameter of the largest ball in A ∩ B.
/// Unlike a pushed boundary sample, this is a geometric length, also for identical solids
/// and crossings with no contained vertices. A Lipschitz branch-and-bound search supplies
/// an explicit error interval when the common region is nonconvex or the work limit is hit.
/// With it, the centre of the deepest ball found, where one was found inside both.
fn overlap(
    acsg: &solid::Csg,
    bcsg: &solid::Csg,
    eps: f64,
) -> Option<(f64, f64, Option<[f64; 3]>)> {
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
    let first = center(bounds.lo, bounds.hi);
    let mut lower = signed(first).max(0.0).min(upper);
    let mut deepest = (lower > 0.0).then_some(first);
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
            let x = center(lo, hi);
            let value = signed(x);
            if value > lower {
                deepest = Some(x);
            }
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
    Some((-(lower + remaining), remaining - lower, deepest))
}

/// Is every point of `a` a point of `b`? Evaluate A − B so an enclosed void is tested too.
pub fn contained(a: &[Piece], b: &solid::Csg, eps: f64) -> bool {
    crate::csg::contains_boundary(b, a, eps)
}

/// The least distance between two convex planar pieces: every vertex against the other's plane
/// and outline, and every pair of edges.  The closed forms rather than an iteration, since a
/// piece has three or four sides and the whole thing is a handful of dot products.
fn piece_gap(p: &Piece, q: &Piece) -> f64 {
    piece_nearest(p, q).0
}

/// `piece_gap` with where it is found: the nearest point of `p` and the nearest point of `q`.
fn piece_nearest(p: &Piece, q: &Piece) -> (f64, [f64; 3], [f64; 3]) {
    let mut best = (f64::INFINITY, p.pts[0], q.pts[0]);
    for (a, b, swap) in [(p, q, false), (q, p, true)] {
        for (i, v) in a.pts.iter().enumerate() {
            let w = a.pts[(i + 1) % a.pts.len()];
            let h = plane::dot(b.n, std::array::from_fn(|k| v[k] - b.pts[0][k]));
            let end = plane::dot(b.n, std::array::from_fn(|k| w[k] - b.pts[0][k]));
            if h * end < 0.0 {
                let t = h / (h - end);
                let x = std::array::from_fn(|k| v[k] + t * (w[k] - v[k]));
                if nearest_on_piece(x, b).0
                    <= plane::norm(std::array::from_fn(|k| w[k] - v[k])) * 1e-12
                {
                    return (0.0, x, x);
                }
            }
        }
        for v in &a.pts {
            let (d, foot) = nearest_on_piece(*v, b);
            if d < best.0 {
                best = if swap { (d, foot, *v) } else { (d, *v, foot) };
            }
        }
    }
    for i in 0..p.pts.len() {
        let a = (p.pts[i], p.pts[(i + 1) % p.pts.len()]);
        for j in 0..q.pts.len() {
            let b = (q.pts[j], q.pts[(j + 1) % q.pts.len()]);
            let found = seg_nearest(a, b);
            if found.0 < best.0 {
                best = found;
            }
        }
    }
    best
}

fn point_to_piece(x: [f64; 3], p: &Piece) -> f64 {
    nearest_on_piece(x, p).0
}

/// How far `x` is from piece `p`, and the point of `p` nearest it.
fn nearest_on_piece(x: [f64; 3], p: &Piece) -> (f64, [f64; 3]) {
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
        return (h.abs(), foot);
    }
    let mut best = (f64::INFINITY, p.pts[0]);
    for i in 0..p.pts.len() {
        let a = p.pts[i];
        let b = p.pts[(i + 1) % p.pts.len()];
        let found = nearest_on_seg(x, a, b);
        if found.0 < best.0 {
            best = found;
        }
    }
    best
}

fn nearest_on_seg(x: [f64; 3], a: [f64; 3], b: [f64; 3]) -> (f64, [f64; 3]) {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let dd = plane::dot(d, d);
    let t = if dd > 0.0 {
        (plane::dot([x[0] - a[0], x[1] - a[1], x[2] - a[2]], d) / dd).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let c = [a[0] + t * d[0], a[1] + t * d[1], a[2] + t * d[2]];
    (plane::norm([x[0] - c[0], x[1] - c[1], x[2] - c[2]]), c)
}

/// The least distance between two segments, by the clamped closed form, and the nearest point of
/// each.
fn seg_nearest(a: ([f64; 3], [f64; 3]), b: ([f64; 3], [f64; 3])) -> (f64, [f64; 3], [f64; 3]) {
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
    (plane::norm([p[0] - q[0], p[1] - q[1], p[2] - q[2]]), p, q)
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
    let requirement = match crate::model::SolidRequirement::from_word(word, gap) {
        Ok(r) => r,
        Err(reason) => return Verdict::failed(reason),
    };
    evaluate(sk, &requirement, a, b, unit)
}

/// Evaluate a complete requirement using the shared validated-solid interface (#53).
pub(crate) fn evaluate(
    sk: &Sketch,
    requirement: &crate::model::SolidRequirement,
    a: usize,
    b: usize,
    unit: f64,
) -> Verdict {
    let policy = solid::ApproximationPolicy::from_unit(unit);
    let pair = sk
        .evaluated_solid(a, policy)
        .and_then(|a| sk.evaluated_solid(b, policy).map(|b| (a, b)));
    match pair {
        Ok((a, b)) => evaluate_pair(requirement, &a, &b),
        Err(reason) => Verdict::failed(reason),
    }
}

/// Checked compatibility adapter. Invalid arguments cannot reach geometric evaluation.
pub fn judge_evaluated(
    word: crate::constraints::SolidWord,
    a: &solid::EvaluatedSolid,
    b: &solid::EvaluatedSolid,
    gap: f64,
) -> Verdict {
    match crate::model::SolidRequirement::from_word(word, gap) {
        Ok(requirement) => evaluate_pair(&requirement, a, b),
        Err(reason) => Verdict::failed(reason),
    }
}

/// Both classifiers and boundaries come from validated evaluations, in A's local frame.
pub fn evaluate_pair(
    requirement: &crate::model::SolidRequirement,
    a: &solid::EvaluatedSolid,
    b: &solid::EvaluatedSolid,
) -> Verdict {
    Verdict { evidence: measure_pair(requirement, a, b, None) }
}

/// `measure_pair` with `b` moved rigidly in space by `pose`: what a claim swept over a motion asks
/// at each roll, the solids evaluated once.
pub(crate) fn evaluate_moved(
    requirement: &crate::model::SolidRequirement,
    a: &solid::EvaluatedSolid,
    b: &solid::EvaluatedSolid,
    pose: crate::envelope::Motion,
) -> Result<GeometricEvidence, String> {
    measure_pair(requirement, a, b, Some(pose))
}

fn measure_pair(
    requirement: &crate::model::SolidRequirement,
    a: &solid::EvaluatedSolid,
    b: &solid::EvaluatedSolid,
    pose: Option<crate::envelope::Motion>,
) -> Result<GeometricEvidence, String> {
    use crate::model::SolidRequirement as R;
    let pa = a.boundary();
    let ca = a.classifier();
    let (cb, pb) = a.relative(b, pose);
    if pb
        .iter()
        .flat_map(|p| &p.pts)
        .flatten()
        .any(|x| !x.is_finite())
    {
        return Err("relative solid placement exceeds numerical precision".into());
    }
    let eps = a.epsilon().min(b.epsilon());
    let error = |s: &solid::EvaluatedSolid| {
        if s.boundary().iter().any(|p| p.smooth) {
            s.sagitta()
        } else {
            0.0
        }
    };
    let facet_tol = error(a) + error(b);
    if pa.is_empty() || pb.is_empty() {
        return Ok(GeometricEvidence {
            requirement: requirement.clone(),
            predicate: Predicate::Unresolved("empty solid has no measurable boundary".into()),
            measurement: Measurement::Unbounded,
            nearest: None,
        });
    }
    let (gap, pair) = boundary_gap(pa, &pb);
    // the face of each nearest a point of A's frame, said in the world
    // a face as a reader writes it: `near_screw[1].post.rim`, not its block's statement
    let named = |p: &Piece| crate::vocab::public_path(&p.path);
    let faces_at = |x: [f64; 3]| {
        let near = |pieces: &[Piece]| {
            let gap = |p: &&Piece| point_to_piece(x, p);
            pieces.iter().min_by(|p, q| gap(p).total_cmp(&gap(q))).map(named)
        };
        Some((near(pa)?, near(&pb)?))
    };
    let world = |x: [f64; 3]| a.to_world(solid::LocalPoint(x)).0;
    let apart_nearest = || {
        let (i, j) = pair?;
        let (_, x, y) = piece_nearest(&pa[i], &pb[j]);
        let (a, b) = (named(&pa[i]), named(&pb[j]));
        Some(Nearest { a, b, at_a: world(x), at_b: world(y), deepest: false })
    };
    let (status, interval, nearest) = match requirement {
        R::Clear { .. } => {
            // boundaries apart, and neither solid holding a point of the other: disjoint, and the
            // gap is the answer without classifying either boundary against the other
            let apart = gap > eps && !cb.inside(pa[0].pts[0]) && !ca.inside(pb[0].pts[0]);
            let (value, error, deepest) = if apart {
                (gap, 0.0, None)
            } else {
                overlap(ca, &cb, eps).unwrap_or((gap, 0.0, None))
            };
            // where they meet, the faces nearest the deepest of it; else where they come closest
            let nearest = deepest
                .and_then(|x| {
                    let (fa, fb) = faces_at(x)?;
                    Some(Nearest { a: fa, b: fb, at_a: world(x), at_b: world(x), deepest: true })
                })
                .or_else(apart_nearest);
            let interval = Interval::around(value, facet_tol + error)?;
            let status = if interval.lower() > 0.0 {
                Predicate::Satisfied
            } else if interval.upper() < 0.0
                || (interval.lower() == 0.0 && interval.upper() == 0.0)
            {
                Predicate::Refuted
            } else {
                Predicate::Unresolved("disjointness is within geometric uncertainty".into())
            };
            (status, interval, nearest)
        }
        R::Inside | R::Fits { .. } => {
            let inside = contained(pa, &cb, eps);
            let interval = Interval::around(if inside { gap } else { -gap }, facet_tol)?;
            let status = if facet_tol > 0.0 && interval.lower() <= 0.0 && interval.upper() >= 0.0 {
                Predicate::Unresolved("containment is within curved-faceting uncertainty".into())
            } else if inside {
                Predicate::Satisfied
            } else {
                Predicate::Refuted
            };
            (status, interval, apart_nearest())
        }
    };
    Ok(GeometricEvidence {
        requirement: requirement.clone(),
        predicate: status,
        measurement: Measurement::Bounded(interval),
        nearest,
    })
}

/// The least the two boundaries come to each other, ignoring which side of which they are on —
/// what `fits` measures once containment has answered the sign — and the piece of each it is
/// read between.
///
/// `b`'s pieces are boxed in a hierarchy, and a box farther from a piece of `a` than the least gap
/// found is passed by: a piece's gap is never less than its box's distance, so the answer is the
/// one every pair would give, read in the logarithm of `b` (a rotor's thousand pieces against a
/// housing's twelve thousand at every pose of a claim's sweep).
fn boundary_gap(a: &[Piece], b: &[Piece]) -> (f64, Option<(usize, usize)>) {
    let bounds = |q: &Piece| { let x = q.bbox_of(); crate::bvh::Bounds { lo: x.lo, hi: x.hi } };
    let tree = crate::bvh::Bvh::new(b.iter().map(bounds));
    let best = std::cell::Cell::new(f64::INFINITY);
    // the pair it is read between, and how squarely they face each other: of pairs as near, a
    // wall facing a wall is where a reader looks, not the cap whose edge is as near
    let pair = std::cell::Cell::new(None::<(usize, usize, f64)>);
    for (i, p) in a.iter().enumerate() {
        let pb = bounds(p);
        tree.query_nearest(|q| q.gap(pb), || best.get(), |k| {
            if plane_apart(p, &b[k]) <= best.get() {
                let d = piece_gap(p, &b[k]);
                let facing = plane::dot(p.n, b[k].n);
                let tie = (d - best.get()).abs() <= 1e-9 * (1.0 + best.get());
                let squarer = pair.get().is_some_and(|(_, _, f)| facing < f);
                if pair.get().is_none() || (d < best.get() && !tie) || (tie && squarer) {
                    pair.set(Some((i, k, facing)));
                }
                best.set(best.get().min(d));
            }
        });
    }
    (best.get(), pair.get().map(|(i, k, _)| (i, k)))
}

/// A lower bound on two pieces' gap from their planes: where every corner of one lies on one side
/// of the other's plane, the nearest of them is no nearer than that.
fn plane_apart(p: &Piece, q: &Piece) -> f64 {
    let side = |a: &Piece, b: &Piece| {
        let length = plane::norm(a.n);
        if !(length > 0.0) { return 0.0; }
        let (lo, hi) = b.pts.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
            let h = plane::dot(a.n, std::array::from_fn(|k| v[k] - a.pts[0][k])) / length;
            (lo.min(h), hi.max(h))
        });
        // a hair under, for the rounding of the reading
        (if lo > 0.0 { lo } else if hi < 0.0 { -hi } else { 0.0 }) * (1.0 - 1e-12)
    };
    side(p, q).max(side(q, p))
}
