//! **An operand built exactly, read by its own boundary** (#419): a body whose terms read its
//! faces (a rounded plate) and that this kernel builds, read in another body's field as the
//! signed distance from its B-rep rather than as the composition of its terms.  The composition
//! reads a sheet of nothing where a round's cut lies on the face it rounds — the prism's top and
//! the round's region both zero there — and a fill crossing that sheet ends a feature curve on
//! another; the boundary has no such sheet.
//!
//! The distance is the nearest point of the boundary: a face's foot on its surface where that lies
//! on the face, else the nearest point of an edge (a vertex is an edge's end).  It is signed by the
//! face's outward normal at a foot, and at an edge by the boundary's own ray classification
//! (`Located::solid_place`; beyond the faces' box, outside).  Its pieces are the faces, each
//! carried by its whole surface's signed distance (`Surface::implicit`), so a crease — the plate's
//! foot — is found as a region's terms' are.  Read only where every face is a plane, a
//! cylinder, a cone, a sphere or a torus, whose feet are exact; a box's enclosure is the value at
//! its centre widened by its half-diagonal, the distance being one-Lipschitz.
use super::{Error, I, V};
use crate::brep::geom::Surface;
use crate::brep::query::{face_box, Located, Place};
use crate::space::{distance, scale, sub};
use std::sync::Arc;

type P3 = [f64; 3];

/// A boundary built exactly, read as a field (model units).
#[derive(Clone, Debug)]
pub struct BuiltField(Arc<Boundary>);

#[derive(Debug)]
struct Boundary {
    loc: Located<'static>,
    /// Millimetres to a model unit, and where the boundary is built about (model units): a point of
    /// space is `(p − origin)·mm` in it.
    mm: f64,
    origin: P3,
    /// The faces each edge bounds.
    bounding: Vec<Vec<usize>>,
    /// The box the faces lie within (model units).
    support: [[f64; 2]; 3],
    /// A rounding allowance for enclosures (model units): the boundary's tolerance and more.
    slack: f64,
}

impl BuiltField {
    /// An exact boundary as a field, or `None` where it has a face no foot is exact on.
    pub fn new(exact: &crate::solid::Exact) -> Option<Self> {
        let b = &exact.brep;
        let exact_feet = |f: &crate::brep::topo::Face| matches!(f.surface, Surface::Plane(_)
            | Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) | Surface::Torus(..));
        if b.faces.is_empty() || !b.faces.iter().all(exact_feet) {
            return None;
        }
        let tol = 1e-9 * b.size().max(1.0);
        let (mm, origin) = (exact.mm, exact.origin);
        let mut bounding = vec![Vec::new(); b.edges.len()];
        for (fi, f) in b.faces.iter().enumerate() {
            for c in f.loops.iter().flatten() {
                if !bounding[c.edge as usize].contains(&fi) {
                    bounding[c.edge as usize].push(fi);
                }
            }
        }
        let mut support = [[f64::INFINITY, f64::NEG_INFINITY]; 3];
        for fi in 0..b.faces.len() {
            let (lo, hi) = face_box(b, fi);
            for k in 0..3 {
                support[k] = [support[k][0].min(lo[k] / mm + origin[k] - tol / mm),
                              support[k][1].max(hi[k] / mm + origin[k] + tol / mm)];
            }
        }
        let size = support.iter().fold(1.0f64, |m, [a, z]| m.max(a.abs()).max(z.abs()));
        let loc = Located::owned(b.clone(), tol);
        // a point within `tol` of a face reads nothing (`Located`'s `On`)
        let slack = 4.0 * tol / mm + 1e-12 * size;
        Some(Self(Arc::new(Boundary { loc, mm, origin, bounding, support, slack })))
    }

    pub fn value(&self, p: P3) -> f64 { self.value_piece(p).0 }

    /// The signed distance from the boundary (positive outside) and the face nearest.
    pub fn value_piece(&self, p: P3) -> (f64, usize) {
        let s = &self.0;
        let b = &s.loc.b;
        let q = scale(sub(p, s.origin), s.mm);
        // the nearest face's foot, signed by its outward normal
        let mut best: Option<(f64, usize, f64)> = None;
        for (fi, f) in b.faces.iter().enumerate() {
            let uv = f.surface.inverse(q);
            let foot = f.surface.point(uv);
            let d = distance(q, foot);
            if best.is_some_and(|(e, ..)| e <= d) || s.loc.face_place(fi, foot) == Place::Out {
                continue;
            }
            let side = f.surface.implicit(q);
            best = Some((d, fi, if f.reversed { -side } else { side }));
        }
        // nearer an edge than any face's foot: signed by where the point stands
        let edge = (0..b.edges.len()).map(|e| (e, s.loc.nearest(e as u32, q).1))
            .min_by(|x, y| x.1.total_cmp(&y.1));
        let (d, face, sign) = match (best, edge) {
            (Some((d, fi, side)), Some((_, de))) if d <= de => (d, fi, side.signum()),
            (_, Some((e, de))) => {
                // (beyond the faces' box, outside without a ray)
                let beyond = (0..3).any(|k| p[k] < s.support[k][0] || p[k] > s.support[k][1]);
                let sign = match if beyond { Place::Out } else { s.loc.solid_place(q) } {
                    Place::Out => 1.0,
                    Place::In => -1.0,
                    Place::On => 0.0,
                };
                let v = sign * de / s.mm;
                // the face the edge bounds whose surface reads nearest the distance
                let off = |fi: usize| (self.carrier_of(p, fi) - v).abs();
                let fi = s.bounding[e].iter().copied().min_by(|&a, &c| off(a).total_cmp(&off(c)))
                    .unwrap_or(0);
                (de, fi, sign)
            }
            (Some((d, fi, side)), None) => (d, fi, side.signum()),
            (None, None) => return (f64::INFINITY, 0),
        };
        (sign * d / s.mm, face)
    }

    pub fn piece_count(&self) -> usize { self.0.loc.b.faces.len() }

    /// Face `piece`'s whole surface's signed distance, positive away from the material.
    pub fn carrier(&self, p: P3, piece: usize) -> Option<f64> {
        (piece < self.piece_count()).then(|| self.carrier_of(p, piece))
    }

    fn carrier_of(&self, p: P3, fi: usize) -> f64 {
        let s = &self.0;
        let f = &s.loc.b.faces[fi];
        let v = f.surface.implicit(scale(sub(p, s.origin), s.mm)) / s.mm;
        if f.reversed { -v } else { v }
    }

    pub fn support_bounds(&self) -> Result<Option<V>, Error> {
        let b = self.0.support;
        Ok(Some([I::new(b[0][0], b[0][1])?, I::new(b[1][0], b[1][1])?, I::new(b[2][0], b[2][1])?]))
    }

    /// The field over a box: its value at the centre within the half-diagonal (one-Lipschitz),
    /// widened by a rounding allowance.
    pub fn bounds(&self, p: V) -> Result<I, Error> {
        let (c, diagonal) = crate::space::box_centre_diagonal(&p);
        let v = self.value(c);
        let half = 0.5 * diagonal + self.0.slack;
        I::new(v - half, v + half)
    }
}
