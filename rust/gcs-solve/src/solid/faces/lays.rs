//! **How the faces of a body lie** (#763): what `when a parallel l` and `when a perpendicular l`
//! read, so a block over a body's faces keeps those along a direction or square to it — the edges
//! a round takes picked by the faces they lie between, written in Solvent, and not by an option of
//! the word.  Two faces along one direction meet in lines along it: the edges in plan of a part
//! drawn in plan.
use super::{cached, FacePath};
use crate::brep::geom::{Curve, Surface, V};
use crate::expr::Along;
use crate::model::Sketch;
use crate::space::{cross, dot, norm};
use std::collections::BTreeMap;

/// Two unit directions closer than this to square, or to one line, are taken to be: a face is
/// read where the drawing stands, so a direction it holds by construction is read to the bits.
const TOL: f64 = 1e-9;

/// How one patch's surface lies, read off its kind exactly: a plane by its normal, a surface
/// swept along a line (a cylinder, a side extruded) by that line's direction; a surface no line
/// lies along at every point (a cone, a sphere, a torus, a surface turned from a circle), none;
/// and a surface whose kind does not say (a fitted one, a blend, a family's wall), by that kind.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Lay {
    Plane(V),
    Along(V),
    Neither,
    Unread(&'static str),
}

impl Lay {
    fn of(s: &Surface) -> Lay {
        match s {
            Surface::Plane(f) => Lay::Plane(f.z),
            Surface::Cylinder(f, _) => Lay::Along(f.z),
            // a straight section swept along a line is a plane holding both
            Surface::Extrusion(f, c) => match &**c {
                Curve::Line { d, .. } => Lay::Plane(unit(cross(f.z, *d))),
                _ => Lay::Along(f.z),
            },
            // a line turned about an axis it runs along is a cylinder, one square to it a plane
            // square to the axis, any other a cone
            Surface::Revolution(f, c) => match &**c {
                Curve::Line { d, .. } if norm(cross(*d, f.z)) <= TOL => Lay::Along(f.z),
                Curve::Line { d, .. } if dot(*d, f.z).abs() <= TOL => Lay::Plane(f.z),
                Curve::Line { .. } | Curve::Circle(..) => Lay::Neither,
                _ => Lay::Unread(s.kind()),
            },
            Surface::Cone(..) | Surface::Sphere(..) | Surface::Torus(..) => Lay::Neither,
            Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..)
            | Surface::Affine(..) => Lay::Unread(s.kind()),
        }
    }

    /// Whether the surface lies `along` the unit `d`, or why that is not read.
    fn holds(self, along: Along, d: V) -> Result<bool, &'static str> {
        Ok(match (self, along) {
            (Lay::Unread(kind), _) => return Err(kind),
            (Lay::Plane(n), Along::Parallel) => dot(n, d).abs() <= TOL,
            (Lay::Along(a), Along::Parallel) => norm(cross(a, d)) <= TOL,
            (Lay::Plane(n), Along::Perpendicular) => norm(cross(n, d)) <= TOL,
            (Lay::Along(_) | Lay::Neither, _) => false,
        })
    }
}

fn unit(v: V) -> V {
    let n = norm(v);
    if n > 0.0 { v.map(|x| x / n) } else { v }
}

/// The faces a solid lists (`face_paths`), each by its name as written from the solid
/// (`FacePath::text`), and how each patch of it lies, read off the stratum (`cached`, as `Meets`
/// reads it) where the drawing stands.  A face a Boolean ate has no patch; `None` for a face a
/// round of the body made, which the stratum may not have.
#[derive(Clone, Debug)]
pub struct Lays {
    faces: BTreeMap<String, Option<Vec<Lay>>>,
}

impl Lays {
    /// The faces `listed` of solid `body`, or `None` where its boundary is not read here: a body
    /// with a swept cut, whose boundary is the field's, or one that does not build.
    pub fn read(sk: &Sketch, body: usize, listed: &[FacePath]) -> Option<Lays> {
        if body >= sk.solids.len() || crate::solid::cad::contains_sweep(sk, body) {
            return None;
        }
        let read = cached(sk, body).ok()?;
        let (brep, _, _) = read.brep();
        let faces = listed.iter().map(|f| {
            let own: Vec<Lay> = read.paths().iter().enumerate()
                .filter(|(_, p)| p.route == f.route && p.face == f.face)
                .map(|(k, _)| Lay::of(&brep.faces[k].surface))
                .collect();
            // a round's face the stratum has not: it cannot say how that one lies
            let unread = f.of.is_some() && own.is_empty();
            (f.text(), (!unread).then_some(own))
        }).collect();
        Some(Lays { faces })
    }

    /// Whether face `face` (by its name from the solid) lies `along` the direction `d` at every
    /// point — `None` where it is no face listed — or why that is not read: a face a round made,
    /// or a patch on a surface whose kind does not say.  A face with no patch lies along nothing.
    pub fn holds(&self, face: &str, along: Along, d: V) -> Option<Result<bool, String>> {
        let lays = self.faces.get(face)?;
        let Some(lays) = lays else {
            return Some(Err("it is a face a round of the body made, which is not read where the \
                             body's faces are named".to_string()));
        };
        let d = unit(d);
        if lays.is_empty() || norm(d) == 0.0 {
            return Some(Ok(false));
        }
        let mut all = true;
        for lay in lays {
            match lay.holds(along, d) {
                Ok(h) => all &= h,
                Err(kind) => return Some(Err(format!(
                    "it lies on a {kind} surface, whose kind does not say how it lies"
                ))),
            }
        }
        Some(Ok(all))
    }
}
