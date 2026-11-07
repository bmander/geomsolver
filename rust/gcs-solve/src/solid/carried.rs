//! **A face carried by a constant twist** (#846): a region whose one term is what a drawn face
//! occupies under a motion over an interval of its roll stated in place — `some t in (a, b): p in
//! sec at(under: m) t`, `RegionShape::Swept` — where the motion is a screw, a slide or a turn at a
//! constant rate (`motion::Family::twist`).  The section is rigid, so what it sweeps is the face's
//! profile where the interval begins carried by the motion to where it ends: built exactly as the
//! kernel's carried profile (`brep::build::carried`, the recipe's `carried` node) — a screw's walls
//! the screw's sheets of its edges, as a face family the statements say a screw carries is built
//! (`family::stated_carry`), with no station solved — and held to its field before it stands
//! (`built`).  It names no running parameter, so it stands in a component's body as anywhere.
//!
//! The field reads a screw's or a slide's point by point (`Reading`).  A slide's face crossed by
//! its direction, or a screw's square to its axis, is crossed once by each point's path, so one
//! roll carries the point back to the face's plane; a screw's face in a plane holding its axis (a
//! thread drawn through it) is reached once a turn, so the rolls a turn apart that could carry the
//! point back into it are read and the least taken.  Either way the point is within the sweep
//! where such a roll is in the interval and the point carried back is in the face.  A turn is read
//! about the line in the face's plane as before (`region::general`); it is built here too, so an
//! interval that does not begin at nothing is exact.
//!
//! Refused, with its reason, where the sweep would meet itself or has no volume: a screw's face
//! leaning on its axis, one through the axis that reaches it or — carried more than a turn — spans
//! as much as the screw rises a turn, a slide along the face's own plane, a turn about a line off
//! it.
use crate::model::{RegionShape, RegionTerm, Sketch, SolidDef};
use crate::motion::Screw;
use crate::plane::Basis;
use crate::space::{dot, sub};
use crate::brep::recipe::Writer;
use crate::json::{object, Json};
use crate::vocab::Cmp;
use std::f64::consts::TAU;
use std::rc::Rc;

type V = [f64; 3];

/// A face carried over an interval of a constant twist: the face, the motion's screw (a turn of
/// no advance, a slide of no ratio), and the interval of the roll, in order (radians).
#[derive(Clone, Debug)]
pub struct Carried {
    pub face: u32,
    pub motion: u32,
    pub screw: Screw,
    pub roll: [f64; 2],
    crossing: Crossing,
}

/// How a point's path crosses the section's plane: once (`Level`: a slide's crossing plane, a
/// screw's plane square to its axis), or once a turn where the plane holds a screw's axis
/// (`Axial`: the angle about the axis of the half-plane the section lies in, and the heights along
/// the axis it spans); a turn's, about a line in it, is `general`'s to read (`Turn`).
#[derive(Clone, Copy, Debug)]
enum Crossing {
    Level,
    Axial { side: f64, heights: [f64; 2] },
    Turn,
}

/// A screw's section in a plane holding its axis: the half-plane it lies in and the heights it
/// spans, refused where it reaches the axis or across it, or — carried more than a turn — where it
/// spans as much as the screw rises a turn, so the sweep would meet itself.
fn axial(sk: &Sketch, face: u32, screw: &Screw, roll: [f64; 2]) -> Result<Crossing, String> {
    let poly = super::face_poly(sk, face as usize, super::REPORT_UNIT).ok_or("no such face")?;
    let pts: Vec<V> = poly.pts.iter().map(|&(x, y)| poly.basis.lift(x, y)).collect();
    let n = poly.basis.normal();
    // across the plane from the axis, square to it
    let out = crate::space::cross(n, screw.axis);
    let sides: Vec<f64> = pts.iter().map(|&p| dot(sub(p, screw.origin), out)).collect();
    let size = pts.iter().map(|&p| crate::space::norm(sub(p, poly.basis.o))).fold(0.0, f64::max);
    let near = 1e-9 * (1.0 + size);
    let positive = sides.iter().all(|&x| x > near);
    if !positive && !sides.iter().all(|&x| x < -near) {
        return Err("a face swept by a screw about a line in its plane lies on one side of the \
            line: one that reaches the axis, or across it, sweeps a body that meets itself".into());
    }
    let heights = pts.iter().map(|&p| screw.height(p))
        .fold([f64::INFINITY, f64::NEG_INFINITY], |[a, b], h| [a.min(h), b.max(h)]);
    let rise = (screw.advance / screw.ratio).abs();
    if screw.ratio.abs() * (roll[1] - roll[0]) > TAU + 1e-12 && heights[1] - heights[0] >= rise - near {
        return Err(format!("a face carried more than a turn up a screw spans less along its axis \
            than the screw rises a turn ({}): this one spans {}, so its sweep meets itself",
            crate::vocab::num(rise), crate::vocab::num(heights[1] - heights[0])));
    }
    let side = screw.angle(crate::space::add(screw.origin, if positive { out } else { out.map(|x| -x) }));
    Ok(Crossing::Axial { side, heights })
}

/// The plane face `face` is drawn in (the page where it names none).
fn basis(sk: &Sketch, face: u32) -> Option<Basis> {
    let f = sk.faces.get(face as usize)?;
    Some(match f.plane {
        Some(p) => sk.basis(p as usize),
        None => Basis::page(),
    })
}

/// The constant twist motion `m` is, where it is one.
fn twist(sk: &Sketch, m: u32) -> Option<Screw> {
    crate::motion::Family::read(sk, m as usize).ok()?.twist()
}

impl Carried {
    /// Face `face` under motion `m` over the interval `roll` of its roll, where `m` is a constant
    /// twist: `None` where it is not one (a motion of several steps, one driven by a law), else the
    /// sweep or why it is refused.
    pub fn of_term(sk: &Sketch, m: u32, roll: [f64; 2], face: u32)
        -> Option<Result<Carried, String>> {
        let screw = twist(sk, m)?;
        Some(Carried::checked(sk, m, screw, roll, face))
    }

    /// `of_term` where the motion advances — a screw or a slide, whose path crosses the section's
    /// plane once: `None` for a turn as for any motion no constant twist.
    pub fn advancing(sk: &Sketch, m: u32, roll: [f64; 2], face: u32)
        -> Option<Result<Carried, String>> {
        let screw = twist(sk, m).filter(|s| !s.is_turn())?;
        Some(Carried::checked(sk, m, screw, roll, face))
    }

    fn checked(sk: &Sketch, m: u32, screw: Screw, roll: [f64; 2], face: u32)
        -> Result<Carried, String> {
        let b = basis(sk, face).ok_or("no such face")?;
        let roll = [roll[0].min(roll[1]), roll[0].max(roll[1])];
        if !(roll[1] > roll[0]) || !roll.iter().all(|r| r.is_finite()) {
            return Err("a face swept over an interval of a roll is swept over one of some length"
                .into());
        }
        let n = b.normal();
        let across = dot(n, screw.axis).abs();
        let crossing = if screw.is_turn() {
            if !super::defined::lies_in(&b, screw.origin, screw.axis) {
                return Err("a face swept by a turn is read point by point where the line it turns \
                    about lies in the face's plane".into());
            }
            Crossing::Turn
        } else if screw.is_slide() {
            if across <= 1e-9 {
                return Err("a face slid along its own plane sweeps no volume: a slide sweeps a \
                    face whose plane its direction crosses".into());
            }
            Crossing::Level
        } else if (1.0 - across).abs() <= 1e-9 {
            Crossing::Level
        } else if super::defined::lies_in(&b, screw.origin, screw.axis) {
            axial(sk, face, &screw, roll)?
        } else {
            return Err("a face swept by a screw is read where its plane is square to the screw's \
                axis or holds it, so the screw carries each point across it once a turn: a \
                section leaning on the axis is not read yet".into());
        };
        Ok(Carried { face, motion: m, screw, roll, crossing })
    }

    /// Region solid `solid`, where its one term is a face carried over an interval of a constant
    /// twist: the sweep, or why it is refused.  `None` for any other region.
    pub fn of(sk: &Sketch, solid: usize) -> Option<Result<Carried, String>> {
        let SolidDef::Region { terms, .. } = &sk.solids.get(solid)?.def else { return None };
        let [t] = terms.as_slice() else { return None };
        let RegionShape::Swept { motion, roll, face } = t.shape else { return None };
        if !held(t) {
            return None;
        }
        Carried::of_term(sk, motion, roll, face)
    }

    /// The recipe's `carried` node (`brep::build::carried`): the face's profile, the motion's pose
    /// where the interval begins, and its carry from there to where it ends.
    pub fn node(&self, sk: &Sketch, w: &Writer) -> Result<Json, String> {
        let start = crate::motion::Family::read(sk, self.motion as usize)?.at(self.roll[0])?;
        Ok(object([
            ("kind", "carried".into()),
            ("profile", super::cad::profile(sk, self.face as usize, w)?),
            ("start", w.matrix(super::cad::placement_matrix(start, w.scale))),
            ("carry", super::cad::carry_json(self.carry(w))),
        ]))
    }

    /// The motion's carry over the interval, in the recipe's units (`scale` model units a
    /// millimetre, about `w`'s origin): a screw's turn and step, a turn's, or a slide's step.
    pub fn carry(&self, w: &crate::brep::recipe::Writer) -> crate::brep::geom::Carry {
        let s = &self.screw;
        let run = self.roll[1] - self.roll[0];
        let advance = s.advance * run / TAU * w.scale;
        if s.is_slide() {
            return crate::brep::geom::Carry::Line { delta: s.axis.map(|x| x * advance) };
        }
        crate::brep::geom::Carry::Screw {
            center: w.moved(s.origin.map(|x| x * w.scale)),
            axis: s.axis,
            angle: s.ratio * run,
            advance,
        }
    }

    /// The field's reading of the sweep (a screw's or a slide's: a turn's is `general`'s): the
    /// section's plane and region, how the motion crosses it, and the planes it is within.
    pub(crate) fn reading(&self, sk: &Sketch) -> Result<Reading, String> {
        if let Crossing::Turn = self.crossing {
            return Err("a face swept by a turn is read about its line by the region's field".into());
        }
        let (b, region, reach) = crate::solid::field::section(sk, self.face as usize)?;
        let s = &self.screw;
        let mut planes = Vec::new();
        if let Some(r) = reach {
            let along = |t: f64| s.advance * t / TAU;
            if s.is_slide() {
                // the box of the section's square, at each end of the slide
                for k in 0..3 {
                    let ends = self.roll.map(|t| b.o[k] + s.axis[k] * along(t));
                    let mut e = [0.0; 3];
                    e[k] = 1.0;
                    planes.push((e, ends[0].max(ends[1]) + r));
                    e[k] = -1.0;
                    planes.push((e, -(ends[0].min(ends[1]) - r)));
                }
            } else {
                // within the cylinder about the axis the square about the section's origin turns
                // in, between the heights its square is carried to
                let reach = crate::space::norm(sub(b.o, s.origin)) + r;
                let (e1, e2) = crate::space::across(s.axis);
                for e in [e1, e2, e1.map(|x| -x), e2.map(|x| -x)] {
                    planes.push((e, dot(e, s.origin) + reach));
                }
                let h = self.roll.map(|t| s.height(b.o) + along(t));
                planes.push((s.axis, h[0].max(h[1]) + r));
                planes.push((s.axis.map(|x| -x), -(h[0].min(h[1]) - r)));
            }
        }
        Ok(Reading { screw: self.screw, roll: self.roll, basis: b, region,
                     crossing: self.crossing, planes })
    }
}

/// Region solid `solid` built as a face carried over an interval of a constant twist, where it is
/// one (`Carried::of`): standing only where its field agrees with the kernel's construction —
/// every probe a little inside its boundary material by the field, every one a little outside not
/// (`agreement`), as a region in general position is held (`bounded`).  Remembered against what
/// it reads (`Caches::carried`).
pub fn built(sk: &Sketch, solid: usize) -> Option<Result<Rc<Carried>, String>> {
    if !matches!(sk.solids.get(solid)?.def, SolidDef::Region { .. }) {
        return None;
    }
    sk.caches.carried.get_or(solid, super::MemoKey::of(sk, solid), || {
        Carried::of(sk, solid).map(|c| c.and_then(|c| {
            let brep = crate::brep::recipe::node(&c.node(sk, &Writer::world(1.0))?,
                                                 &Default::default())?;
            let field = super::MaterialField::read(sk, solid, super::cad::AXIS_TOLERANCE)?;
            super::agreement::gate(&field, &brep, None,
                                   "its exact construction disagrees with the field")?;
            Ok(Rc::new(c))
        }))
    })
}

/// **The faces a carried face's sweep is named by** (`defined`'s rule): region solid `solid`'s,
/// where its one term is a face swept over an interval of a motion — its section's sides by the
/// edges they are swept from, then `start` where the interval begins and `end` where it ends, a
/// turn through a whole one or more capped by neither.  Read off the definition alone, whatever
/// the motion: what the field reads and what the kernel builds are named alike.
pub fn face_names(sk: &Sketch, solid: usize) -> Option<Vec<String>> {
    let SolidDef::Region { terms, .. } = &sk.solids.get(solid)?.def else { return None };
    let [t] = terms.as_slice() else { return None };
    let RegionShape::Swept { motion, roll, face } = t.shape else { return None };
    if !held(t) {
        return None;
    }
    let f = sk.faces.get(face as usize)?;
    let mut v: Vec<String> = f.boundaries().flat_map(|(_, n)| n.iter().cloned()).collect();
    let whole = twist(sk, motion)
        .is_some_and(|s| s.is_turn() && (s.ratio * (roll[1] - roll[0])).abs() >= TAU - 1e-12);
    if !whole {
        v.extend(["start".to_string(), "end".to_string()]);
    }
    v.dedup();
    Some(v)
}

/// Whether a term holds where its shape is at most nothing — the one way a set is applied to a
/// region's point.
fn held(t: &RegionTerm) -> bool {
    t.cmp == Cmp::Le && t.hi.is_none() && t.grow.is_none() && t.lo.free.is_none() && t.lo.c == 0.0
}

/// **A screw's or a slide's sweep read point by point**: negative within it.  A point's path
/// crosses the section's plane at one roll (`Crossing::Level`), or once a turn into the half-plane
/// the section lies in (`Crossing::Axial`); it is within the sweep where such a roll is in the
/// interval and the motion carries it back there into the face — the region's value at the point
/// carried back, or how far along its path the roll falls outside the interval, the least over the
/// rolls that could carry it into the face.
pub(crate) struct Reading {
    screw: Screw,
    roll: [f64; 2],
    basis: Basis,
    region: crate::solid::field::PlanarField,
    crossing: Crossing,
    /// The half-spaces the sweep is within, `n · x ≤ d`, where the section's reach is known.
    pub planes: Vec<(V, f64)>,
}

impl Reading {
    /// At `p`, carried back over the roll `t`: the region's value there, or how far the roll falls
    /// outside the interval at `speed` (a length a radian of the roll).
    fn at(&self, p: V, t: f64, speed: f64) -> f64 {
        let (x, y) = self.basis.view_coords(self.screw.carry(p, -t));
        let [t0, t1] = self.roll;
        self.region.value([x, y]).max(speed * (t0 - t).max(t - t1))
    }

    /// The reading at `p`: negative within the sweep.
    pub fn value(&self, p: V) -> f64 {
        let s = &self.screw;
        let rise = s.advance / TAU;
        match self.crossing {
            Crossing::Level => {
                let n = self.basis.normal();
                let rate = dot(n, s.axis) * rise;
                self.at(p, dot(n, sub(p, self.basis.o)) / rate, rate.abs())
            }
            Crossing::Turn => unreachable!("a turn is read by the region's field (`reading`)"),
            Crossing::Axial { side, heights } => {
                let period = TAU / s.ratio.abs();
                let speed = (s.radius(p) * s.ratio).hypot(rise);
                // the rolls a turn apart carrying `p` into the section's half-plane, those that
                // carry it back to a height the section spans within a turn of the interval
                let first = (s.angle(p) - side) / s.ratio;
                let back = [heights[1], heights[0]].map(|h| (s.height(p) - h) / rise);
                let [t0, t1] = self.roll;
                let mut lo = back[0].min(back[1]).max(t0) - period;
                let mut hi = back[0].max(back[1]).min(t1) + period;
                if lo > hi {
                    let mid = ((lo + hi) / 2.0).clamp(t0, t1);
                    (lo, hi) = (mid - period, mid + period);
                }
                let k0 = ((lo - first) / period).ceil();
                let k1 = ((hi - first) / period).floor();
                let mut best = f64::INFINITY;
                let mut k = k0;
                while k <= k1 {
                    best = best.min(self.at(p, first + k * period, speed));
                    k += 1.0;
                }
                best
            }
        }
    }
}
