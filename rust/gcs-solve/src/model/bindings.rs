//! Solids of one binding (#566): a closed instance's solids are a function of its definition and
//! its arguments, so instances given the same are one evaluation placed by each one's frame.
//!
//! The language says which solids those are: it stamps each solid a closed instance makes with an
//! opaque key and the frame the instance stands in (`Binding`), and this module knows nothing
//! else of it — no definition, no argument, no Solvent.  Of the solids sharing a key, the first
//! by index is evaluated as its own (the *representative*); every other is the representative's
//! evaluation moved by the motion carrying the representative's frame onto its own, and its faces
//! renamed from the one instance's names to the other's (`Sketch::bound`).

use super::*;
use crate::plane::Basis;
use crate::solid::Rigid;

/// **What a solid is a function of**, where a closed instance made it: the key the language gave
/// it, which every solid of the same definition, arguments and path in the body shares, and where
/// the instance stands.
#[derive(Clone, Debug, PartialEq)]
pub struct Binding {
    /// Opaque: equal exactly where the solids are one function of one binding.
    pub key: String,
    /// The instance's prefix, as the solid's name and every name it reads start with: what a
    /// shared evaluation's names are rewritten from and to.
    pub at: String,
    /// The frame the instance stands in (`Sketch::frames`), `None` for the world.
    pub frame: Option<u32>,
    /// Where the instance is a copy of a `ring`'s body past the first: that frame turned `k`
    /// steps of `τ/n` about the ring's centre, as the copy's entities are (`Turn`).
    pub turn: Option<(EntRef, u32, u32)>,
}

/// A solid read as another's evaluation placed (`Sketch::bound`): the representative, the motion
/// carrying it onto this one, and the prefix its names are rewritten from and to.
#[derive(Clone, Debug)]
pub struct Bound {
    pub rep: usize,
    pub motion: Rigid,
    pub from: String,
    pub to: String,
}

impl Sketch {
    /// **Solid `i` as its binding's representative placed** (#566): `None` for a solid with no
    /// binding, for the representative itself, and for one whose binding does not hold — a
    /// swept solid (meshed elsewhere, by solid), or one reaching a solid no instance of its
    /// binding made (a tool joined from outside the instance).
    pub fn bound(&self, i: usize) -> Option<Bound> {
        let b = self.solids.get(i)?.binding.as_ref()?;
        if !self.shares(i) {
            return None;
        }
        let rep = (0..i).find(|&j| self.solids[j].binding.as_ref().is_some_and(|c| c.key == b.key)
            && self.shares(j))?;
        let from = self.solids[rep].binding.as_ref()?.at.clone();
        let motion = Rigid::between(&self.binding_pose(rep)?, &self.binding_pose(i)?);
        Some(Bound { rep, motion, from, to: b.at.clone() })
    }

    /// Whether solid `i` is one of several solids of one binding: what another solid's recipe
    /// then asks `exact_solid` for, the representative's B-rep built once and placed.
    pub fn shared(&self, i: usize) -> bool {
        let Some(b) = self.solids.get(i).and_then(|s| s.binding.as_ref()) else { return false };
        self.shares(i) && self.solids.iter().enumerate().any(|(j, s)| {
            j != i && s.binding.as_ref().is_some_and(|c| c.key == b.key) && self.shares(j)
        })
    }

    /// Whether solid `i`'s evaluation is its binding's: every solid it reaches was made by its
    /// own instance, and nothing it reaches is meshed elsewhere.
    fn shares(&self, i: usize) -> bool {
        let Some(at) = self.solids[i].binding.as_ref().map(|b| &b.at) else { return false };
        let mut pending = vec![i as u32];
        let mut seen = std::collections::BTreeSet::new();
        while let Some(j) = pending.pop() {
            if !seen.insert(j) {
                continue;
            }
            let Some(s) = self.solids.get(j as usize) else { return false };
            let own = s.binding.as_ref().is_some_and(|b| b.at == *at);
            if !own || matches!(s.def, SolidDef::Swept { .. }) {
                return false;
            }
            match crate::solid::evaluation_operands(self, j as usize) {
                Ok(o) => pending.extend(o),
                Err(_) => return false,
            }
        }
        true
    }

    /// Where solid `i`'s instance stands, as a plane in the world: its frame's (the world's own
    /// axes for none), turned where it is a ring's copy.  `None` for a solid with no binding or a
    /// turn about something that is not there.
    pub fn binding_pose(&self, i: usize) -> Option<Basis> {
        let b = self.solids.get(i)?.binding.as_ref()?;
        let at = match b.frame {
            Some(f) if (f as usize) < self.frames.len() => self.basis(f as usize),
            Some(_) => return None,
            None => Basis { u: [1.0, 0.0, 0.0], v: [0.0, 1.0, 0.0], o: [0.0; 3] },
        };
        let Some((about, k, n)) = b.turn else { return Some(at) };
        // the copy's own turn (`Turn::cos_sin`): about a point, counter-clockwise in its view,
        // which is right-handed about the view's normal; about an axis, right-handed about it
        let (c, s) = Turn { copy: about, rep: about, about, k, n }.cos_sin();
        let (o, d) = match about.kind {
            EntKind::Point if about.i() < self.points.len() => {
                let normal = match self.plane_of(about.i()) {
                    Some(p) => self.basis(p).normal(),
                    None => Basis::page().normal(),
                };
                (self.world_point(about.i()), normal)
            }
            EntKind::Axis if about.i() < self.axes.len() => {
                let x = &self.axes[about.i()];
                (x.a.map(|p| self.params[p as usize].value), x.d.map(|p| self.params[p as usize].value))
            }
            _ => return None,
        };
        let d = crate::space::normalised(d)?;
        let turn = |v: [f64; 3]| {
            let (dv, cross) = (crate::plane::dot(d, v), crate::space::cross(d, v));
            std::array::from_fn(|k| c * v[k] + s * cross[k] + (1.0 - c) * dv * d[k])
        };
        let from_o = crate::space::sub(at.o, o);
        let moved = turn(from_o);
        Some(Basis { u: turn(at.u), v: turn(at.v), o: std::array::from_fn(|k| o[k] + moved[k]) })
    }
}
