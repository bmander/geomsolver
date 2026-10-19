//! **An operand built exactly, read by its own boundary** (#419): a body whose terms read its
//! faces (a rounded plate) and that this kernel builds, read in another body's field as the
//! signed distance from its B-rep rather than as the composition of its terms.  The composition
//! reads a sheet of nothing where a round's cut lies on the face it rounds — the prism's top and
//! the round's region both zero there — and a fill crossing that sheet ends a feature curve on
//! another; the boundary has no such sheet.
//!
//! The distance is `BodyFaces::signed_distance`, the boundary located once.  Its pieces are the
//! faces, each carried by its whole surface's signed distance (`BodyFaces::side`), so a crease —
//! the plate's foot — is found as a region's terms' are.  Read only where every face is a plane,
//! a cylinder, a cone, a sphere or a torus, whose feet are exact; a box's enclosure is the value
//! at its centre widened by its half-diagonal, the distance being one-Lipschitz.
use super::{Error, I, V};
use crate::brep::geom::Surface;
use crate::solid::faces::BodyFaces;
use std::sync::Arc;

type P3 = [f64; 3];

/// A boundary built exactly, read as a field (model units).
#[derive(Clone, Debug)]
pub struct BuiltField {
    faces: Arc<BodyFaces>,
    /// The box the faces lie within, grown by the boundary's tolerance.
    support: [[f64; 2]; 3],
    /// A rounding allowance for enclosures: a point within the tolerance of a face reads nothing
    /// (`Located`'s `On`).
    slack: f64,
}

impl BuiltField {
    /// An exact boundary as a field, or `None` where it has a face no foot is exact on.
    pub fn new(exact: &crate::solid::Exact) -> Option<Self> {
        let exact_feet = |f: &crate::brep::topo::Face| matches!(f.surface, Surface::Plane(_)
            | Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) | Surface::Torus(..));
        if exact.brep.faces.is_empty() || !exact.brep.faces.iter().all(exact_feet) {
            return None;
        }
        let faces = BodyFaces::of(exact.clone(), Vec::new());
        let (tol, (lo, hi)) = (faces.tolerance(), faces.reach());
        let support = std::array::from_fn(|k| [lo[k] - tol, hi[k] + tol]);
        let size = support.iter().fold(1.0f64, |m, [a, z]| m.max(a.abs()).max(z.abs()));
        Some(Self { faces: Arc::new(faces), support, slack: 4.0 * tol + 1e-12 * size })
    }

    pub fn value(&self, p: P3) -> f64 { self.faces.signed_distance(p).0 }

    /// The signed distance from the boundary (positive outside) and the face nearest.
    pub fn value_piece(&self, p: P3) -> (f64, usize) { self.faces.signed_distance(p) }

    pub fn piece_count(&self) -> usize { self.faces.brep().0.faces.len() }

    /// Face `piece`'s whole surface's signed distance, positive away from the material.
    pub fn carrier(&self, p: P3, piece: usize) -> Option<f64> {
        (piece < self.piece_count()).then(|| self.faces.side(piece, p))
    }

    pub fn support_bounds(&self) -> Result<Option<V>, Error> {
        let b = self.support;
        Ok(Some([I::new(b[0][0], b[0][1])?, I::new(b[1][0], b[1][1])?, I::new(b[2][0], b[2][1])?]))
    }

    /// The field over a box: its value at the centre within the half-diagonal (one-Lipschitz),
    /// widened by a rounding allowance.
    pub fn bounds(&self, p: V) -> Result<I, Error> {
        let (c, diagonal) = crate::space::box_centre_diagonal(&p);
        let v = self.value(c);
        let half = 0.5 * diagonal + self.slack;
        I::new(v - half, v + half)
    }
}
