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
    /// Where the instance is mirrored (#578): the axis of the frame it stands in that its image
    /// reverses — 0 its `u` (the mirror in its side plane), 1 its `v` (its front), 2 its normal
    /// (its top).  Its body is solved where it stands unmirrored, and the solid is that
    /// evaluation reflected in the frame's plane square to the axis (`Rigid::reflection`).  Not
    /// part of the key: a left and a right of one binding are one evaluation, and the hand is
    /// recorded here, beside it, for whoever asks which is which.
    pub mirror: Option<u8>,
}

/// A solid read as another's evaluation placed (`Sketch::bound`): the representative, the motion
/// carrying its own evaluation onto where this one's preimage stands, the reflection a mirrored
/// one is published by after it (#578), and the prefix its names are rewritten from and to.
#[derive(Clone, Debug)]
pub struct Bound {
    pub rep: usize,
    pub raw: Rigid,
    pub mirror: Option<Rigid>,
    pub from: String,
    pub to: String,
}

impl Bound {
    /// The motion carrying the representative's own evaluation onto this solid as it is
    /// published: `raw`, then reflected where it is mirrored.
    pub fn motion(&self) -> Rigid {
        self.mirror.map_or(self.raw, |m| m.after(&self.raw))
    }

    /// The bindings solid `i` is read through, in the order they are asked: this one, and where
    /// its representative is another and it is mirrored, its own evaluation reflected — which a
    /// representative's refusal falls back to, so the solid is never published unreflected.
    pub fn tried(self, i: usize) -> impl Iterator<Item = Bound> {
        let alone = (self.rep != i && self.mirror.is_some()).then(|| Bound {
            rep: i, raw: Rigid::identity(), mirror: self.mirror, from: self.to.clone(),
            to: self.to.clone(),
        });
        std::iter::once(self).chain(alone)
    }
}

impl Sketch {
    /// **Solid `i` as its binding's representative placed** (#566): `None` for a solid with no
    /// binding and for the representative itself, unmirrored.  A solid whose binding does not
    /// hold for its peers — a swept solid (meshed elsewhere, by solid), or one reaching a solid
    /// no instance of its binding made (a tool joined from outside the instance) — is its own
    /// representative.  A mirrored solid (#578) is always placed: its representative's own
    /// evaluation, which may be its own, carried where its preimage stands and reflected there.
    pub fn bound(&self, i: usize) -> Option<Bound> {
        let b = self.solids.get(i)?.binding.as_ref()?;
        let rep = self.peers(i).and_then(|mut p| p.next()).filter(|&j| j < i).unwrap_or(i);
        let mirror = match b.mirror {
            Some(_) => Some(self.mirror_of(i)?),
            None if rep == i => return None,
            None => None,
        };
        let from = &self.solids[rep].binding.as_ref()?.at;
        let raw = if rep == i { Rigid::identity() } else {
            Rigid::between(&self.binding_pose(rep)?, &self.binding_pose(i)?)
        };
        Some(Bound { rep, raw, mirror, from: from.clone(), to: b.at.clone() })
    }

    /// **A swept solid as its binding's representative's exact build placed**: a closed instance's
    /// sweeps are, as its other solids are, a function of its definition and its arguments where
    /// it stands (its motions its own), so the exact export builds the first of a binding's
    /// unmirrored solids and places it for every other (`brep::export::exact`).  The
    /// representative and the motion from where it stands to where solid `i` does, the
    /// representative itself unmoved; `None` for a solid no other shares a build with.  Its field
    /// is meshed by solid, so nothing else reads a swept solid through this (`bound`).
    pub fn bound_swept(&self, i: usize) -> Option<(usize, Rigid)> {
        let b = self.solids.get(i)?.binding.as_ref()?;
        let alike = |j: usize| self.solids[j].binding.as_ref()
            .is_some_and(|c| c.key == b.key && c.mirror.is_none()) && self.shares_where(j, true);
        if b.mirror.is_some() || !self.shares_where(i, true) {
            return None;
        }
        let rep = (0..self.solids.len()).find(|&j| alike(j))?;
        if rep == i && !(i + 1..self.solids.len()).any(alike) {
            return None;
        }
        let raw = if rep == i { Rigid::identity() } else {
            Rigid::between(&self.binding_pose(rep)?, &self.binding_pose(i)?)
        };
        Some((rep, raw))
    }

    /// **The reflection a mirrored solid is published by** (#578): in the plane of the frame its
    /// instance stands in square to the axis its image reverses — `None` for any other solid.
    pub fn mirror_of(&self, i: usize) -> Option<Rigid> {
        let axis = self.solids.get(i)?.binding.as_ref()?.mirror?;
        Some(Rigid::reflection(&self.binding_pose(i)?, axis))
    }

    /// Whether solid `i` is one of several solids of one binding: what another solid's recipe
    /// then asks `exact_solid` for, the representative's B-rep built once and placed.
    pub fn shared(&self, i: usize) -> bool {
        self.peers(i).is_some_and(|mut p| p.next().is_some())
    }

    /// The other solids of solid `i`'s binding, by index, where it holds for `i` (`shares`).
    fn peers(&self, i: usize) -> Option<impl Iterator<Item = usize> + '_> {
        let key = &self.solids.get(i)?.binding.as_ref()?.key;
        self.shares(i).then(|| (0..self.solids.len()).filter(move |&j| {
            j != i && self.solids[j].binding.as_ref().is_some_and(|c| c.key == *key) && self.shares(j)
        }))
    }

    /// Whether solid `i`'s evaluation is its binding's: every solid it reaches was made by its
    /// own instance, and nothing it reaches is meshed elsewhere.
    fn shares(&self, i: usize) -> bool {
        self.shares_where(i, false)
    }

    /// `shares`, a swept solid of the instance's own (its motion the instance's) let through
    /// where `swept`: what its exact build, which no field meshes, may share.
    fn shares_where(&self, i: usize, swept: bool) -> bool {
        let Some(at) = self.solids[i].binding.as_ref().map(|b| &b.at) else { return false };
        let mut pending = vec![i as u32];
        let mut seen = std::collections::BTreeSet::new();
        while let Some(j) = pending.pop() {
            if !seen.insert(j) {
                continue;
            }
            let Some(s) = self.solids.get(j as usize) else { return false };
            let own = s.binding.as_ref().is_some_and(|b| b.at == *at);
            let alone = match s.def {
                SolidDef::Swept { motion, .. } => !swept || !self.motions.get(motion as usize)
                    .is_some_and(|m| m.name.starts_with(at.as_str())),
                _ => false,
            };
            if !own || alone {
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
            None => Basis::world(),
        };
        let Some((about, k, n)) = b.turn else { return Some(at) };
        // the copy's own turn (`Turn`): about a point, counter-clockwise in its view, which is
        // right-handed about the view's normal; about an axis, right-handed about it
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
        let m = crate::brep::geom::Rigid::turn(o, d, std::f64::consts::TAU * k as f64 / n as f64);
        Some(Basis { u: m.vector(at.u), v: m.vector(at.v), o: m.point(at.o) })
    }
}
