//! **The join of sets, read** ([0.212]): `{ p | some q in A, r in B: p in Segment(q, r) }`, the
//! points on a segment from some point of one operand to some point of the other
//! (`RegionShape::Join`).  Of convex operands the join is their hull — every point of the hull of
//! two convex sets is on a segment between them — so where each operand is the hull of finitely
//! many points the join is the hull of them all, a convex polytope built by the kernel's
//! quickhull on the exact orientation test (`brep::build::hull`).  An operand is read as those
//! points where its set says it is one: a point of the drawing; a set holding one point (a ball
//! of no radius); a segment (on a line, its ordinate along it bounded at both ends); a drawn
//! polygon whose corners turn one way; a solid every face of which is a plane and every edge
//! convex, its corners; or another join.  Anything else — a set with a curved
//! boundary, a polygon turning both ways, a set unbounded along its line — is refused by name, for
//! the hull of its corners would be another set.
//!
//! The field is the most any of the hull's planes stands below the point (`Hull::value`): nothing
//! on its boundary, below nothing within, and changing no faster than the point moves — the signed
//! distance within, a lower bound on it without, as a common of half-spaces is.
use crate::model::{EntKind, RegionShape, RegionTerm, Sketch, Target};
use crate::space::{add, cross, distance, dot, norm, normalised, scale, sub};
use crate::vocab::Cmp;

type V = [f64; 3];

/// A join read: its corners, and its planes as `(n, d)`, each unit normal outward, the hull
/// `n · x <= d` for every one.
#[derive(Clone, Debug)]
pub struct Hull {
    pub corners: Vec<V>,
    pub planes: Vec<(V, f64)>,
}

impl Hull {
    /// The join of `of`'s operands as the drawing stands: the hull of the points each is the hull
    /// of.
    pub fn of(sk: &Sketch, of: &[Target]) -> Result<Hull, String> {
        let mut points = Vec::new();
        for t in of {
            corners(sk, t, &mut points, 0)?;
        }
        let b = crate::brep::build::hull(&points)
            .map_err(|e| format!("a join that bounds nothing: {e}"))?;
        let planes = b.faces.iter().map(|f| match &f.surface {
            crate::brep::geom::Surface::Plane(frame) => {
                let n = if f.reversed { scale(frame.z, -1.0) } else { frame.z };
                Ok((n, dot(n, frame.o)))
            }
            s => Err(format!("a hull's face on a {}", s.kind())),
        }).collect::<Result<Vec<_>, String>>()?;
        Ok(Hull { corners: b.vertices.iter().map(|v| v.p).collect(), planes })
    }

    /// How far the point stands past the hull's nearest plane: below nothing within it.
    pub fn value(&self, p: V) -> f64 {
        self.planes.iter().map(|&(n, d)| dot(n, p) - d).fold(f64::NEG_INFINITY, f64::max)
    }
}

/// The points operand `t` is the hull of, added to `out`; refused, by name, where it is none.
fn corners(sk: &Sketch, t: &Target, out: &mut Vec<V>, depth: usize) -> Result<(), String> {
    if depth > 16 {
        return Err("a join read in terms of itself".to_string());
    }
    match t {
        Target::Ent(e) if e.kind == EntKind::Point => {
            out.push(sk.world_point(e.i()));
            Ok(())
        }
        Target::Set { terms, .. } => set_corners(sk, terms, out, depth),
        _ => Err("a join is of points and of sets some point of which is bound".to_string()),
    }
}

/// The points a set's terms say it is the hull of (the module's list).
fn set_corners(sk: &Sketch, terms: &[RegionTerm], out: &mut Vec<V>, depth: usize)
    -> Result<(), String> {
    let refused = || "a join is read where each set it is of is the hull of finitely many points: \
        a point, a segment, a convex polygon, a solid of planes meeting in convex edges or another \
        join — a set bounded by a curve, or a polygon turning both ways, is not one".to_string();
    let at = |t: &RegionTerm| t.lo.read(sk);
    let at_nothing = |t: &RegionTerm| at(t) == Some(0.0)
        && (t.cmp == Cmp::Le || (t.cmp == Cmp::In && t.hi == Some(0.0)));
    if let [t] = terms {
        match &t.shape {
            // a solid of planes meeting in convex edges: the hull of its corners
            RegionShape::FaceSide { face } => {
                if !at_nothing(t) || face.surface || face.smooth {
                    return Err(refused());
                }
                let r = crate::solid::faces::reading(sk, face, &mut Vec::new())?;
                let (b, _, _) = r.body.brep();
                let whole = r.faces.len() == b.faces.len();
                let planes = (0..b.faces.len()).all(|f| r.body.plane(f).is_some());
                let convex = (0..b.edges.len() as u32)
                    .all(|e| r.body.faces_of(e).len() != 2 || r.body.convex(e) == Some(true));
                if !(whole && planes && convex) {
                    return Err("a join of a solid is read where every face of it is a plane and \
                        every edge convex: the hull of its corners is then the solid".to_string());
                }
                out.extend(b.vertices.iter().map(|v| r.body.back(v.p)));
                return Ok(());
            }
            // another join
            RegionShape::Join { of } => {
                if !at_nothing(t) {
                    return Err(refused());
                }
                for u in of {
                    corners(sk, u, out, depth + 1)?;
                }
                return Ok(());
            }
            // a set of one point: a ball of no radius
            &RegionShape::Ball { center } if at_nothing(t) => {
                out.push(sk.world_point(center as usize));
                return Ok(());
            }
            _ => {}
        }
    }
    // a drawn polygon, its corners turning one way: within its loops, on its plane
    if let [s, o] = terms {
        let on_plane = matches!(o.shape, RegionShape::Plane { .. }) && o.cmp == Cmp::In
            && at_nothing(o);
        if let RegionShape::Section { face } = s.shape {
            if on_plane && at_nothing(s) {
                return polygon(sk, face as usize, out).ok_or_else(refused);
            }
        }
    }
    // a segment: on a line, its ordinate along it bounded at both ends
    let line = terms.iter().find_map(|t| match t.shape {
        RegionShape::Cylinder { line } if at_nothing(t) => Some(line as usize),
        _ => None,
    }).ok_or_else(refused)?;
    let l = sk.lines.get(line).ok_or_else(refused)?;
    let o = sk.world_point(l.p1 as usize);
    let d = normalised(sub(sk.world_point(l.p2 as usize), o)).ok_or_else(refused)?;
    let (mut lo, mut hi) = (f64::NEG_INFINITY, f64::INFINITY);
    for t in terms {
        match t.shape {
            RegionShape::Cylinder { line: l2 } if l2 as usize == line => continue,
            RegionShape::Plane { along: a, from, sign }
                if a.kind == EntKind::Line && a.i() == line => {
                // the ordinate `sign (x − from) · d` at `o + s d` is `sign (s − s0)`, within
                // `[lower, upper]`
                let s0 = dot(sub(sk.world_point(from as usize), o), d);
                let n = at(t).ok_or_else(refused)?;
                let (lower, upper) = match t.cmp {
                    Cmp::Le => (f64::NEG_INFINITY, n),
                    Cmp::Ge => (n, f64::INFINITY),
                    Cmp::In => (n, t.hi.unwrap_or(n)),
                };
                let (a, b) = if sign > 0.0 { (s0 + lower, s0 + upper) }
                    else { (s0 - upper, s0 - lower) };
                lo = lo.max(a);
                hi = hi.min(b);
            }
            _ => return Err(refused()),
        }
    }
    if !(lo.is_finite() && hi.is_finite() && lo <= hi) {
        return Err(refused());
    }
    out.push(add(o, scale(d, lo)));
    out.push(add(o, scale(d, hi)));
    Ok(())
}

/// A drawn face's corners where it is a polygon of lines, with no hole, its corners turning one
/// way about its normal: the hull of them is the face.
fn polygon(sk: &Sketch, face: usize, out: &mut Vec<V>) -> Option<()> {
    let f = sk.faces.get(face)?;
    if !f.holes.is_empty() || f.edges.iter().any(|e| e.kind != EntKind::Line) {
        return None;
    }
    // each line's ends, walked round: the corners in order
    let corners: Vec<V> = crate::solid::profile::walk_loop(sk, &f.edges)?
        .into_iter().map(|(_, from, _)| sk.world_point(from as usize)).collect();
    let m = corners.len();
    if m < 3 {
        return None;
    }
    let normal = crate::solid::area_vector(&corners);
    let size = corners.iter().map(|&c| distance(c, corners[0])).fold(0.0, f64::max);
    let slack = 1e-12 * size * size * norm(normal);
    let turns_back = (0..m).any(|k| {
        let (a, b, c) = (corners[(k + m - 1) % m], corners[k], corners[(k + 1) % m]);
        dot(cross(sub(b, a), sub(c, b)), normal) < -slack
    });
    if turns_back {
        return None;
    }
    out.extend(corners);
    Some(())
}
