//! **What a region's set defines, recognised from its terms** (#737): a solid is `solid(SET)`,
//! the points inside a set, and the language has no prism, draft or revolution of its own.  Those
//! are words of `std.solids` written in Solvent; a set that says what one of them says — by its
//! terms, whoever wrote them, never by the word's name — is built as the evaluator's sweep of the
//! same points (`SolidDef::Prism`, `SolidDef::Revolve`), exactly, its faces named by the rule
//! below.  A set none of these entries recognises stays a region (`solid::region`), read by its
//! field and built exactly where the region's own routes build it.
//!
//! **The rule a set's bounds name its faces by**, which a hand-written set obeys as `std`'s words
//! do: a section's sides are named by the edges they are swept from; of two ordinates along the
//! section's normal the greater bounds the cap `near` and the lesser the cap `far`; a turn short of
//! a whole one is capped `start` where its interval begins and `end` where it ends.
//!
//! | the region's terms | built as |
//! |---|---|
//! | `p inside sec` (`Section`), an ordinate along `sec`'s normal at least `a` and one at most `b` | `Prism { face: sec, from: a, to: b }` |
//! | the same, and `distance(p, beyond) − distance(p, within) ≥ tan θ · distance(p, sec's plane)` over `beyond := { q \| q outside sec }`, `within := { q \| q inside sec }` | `Prism { …, draft: θ }` |
//! | `p coincident(under: m, over t in (a, b)) sec` (`Swept`: a face's patch), `m` a turn about a line in `sec`'s plane, one end of the interval nothing | `Revolve { face: sec, axis, sweep: b − a, sense }` |
//! | `p inside A` and `p inside`/`p outside` other solids, or `p outside { q \| q outside a; q outside b }`, each solid named whole | `Body { stock, on, through, bound }` |

use crate::model::{EntKind, EntRef, Extent, RegionShape, RegionTerm, Sketch, SolidDef};
use crate::vocab::Cmp;

/// The sweep a region's terms define, where an entry recognises them; `None` otherwise.  Every
/// number recognised is stated: a term reading an unknown of the drawing stays the region's.
pub fn recognised(sk: &Sketch, terms: &[RegionTerm]) -> Option<SolidDef> {
    prism(sk, terms).or_else(|| revolution(sk, terms)).or_else(|| boolean(terms))
}

/// The solid whose whole material a term is within (`Cmp::Le`) or outside (`Cmp::Ge`) — `p
/// inside A`, `p outside A`, `A` a solid named bare — where it is one.
fn whole(t: &RegionTerm) -> Option<(u32, Cmp)> {
    let RegionShape::FaceSide { face } = &t.shape else { return None };
    let bare = face.path.is_empty() && !face.surface && !face.smooth;
    (bare && t.hi.is_none() && t.cmp != Cmp::In && stated(t)? == 0.0).then_some((face.solid, t.cmp))
}

/// **A Boolean of solids** (#737 P3): the points inside a solid `A` and every solid in common with
/// it, outside every solid cut from it — or, for `A`'s place, outside what is outside each of
/// several (`{ q | q outside a; q outside b }`): their union.  Built as the body term over the
/// same solids (`SolidDef::Body`): the stock, what is in union with it, what cuts it, what it is in
/// common with — so `solid({ p | p inside plate; p outside bore })` is `plate` with `bore` cut
/// from it, whichever is written.
fn boolean(terms: &[RegionTerm]) -> Option<SolidDef> {
    let (mut united, mut within, mut cut) = (Vec::new(), Vec::new(), Vec::new());
    for t in terms {
        if let RegionShape::Outside { set } = &t.shape {
            let crate::model::Target::Set { terms: inner, .. } = set.as_ref() else { return None };
            if t.cmp != Cmp::Le || stated(t)? != 0.0 || !united.is_empty() || inner.len() < 2 {
                return None;
            }
            for u in inner {
                let (s, Cmp::Ge) = whole(u)? else { return None };
                united.push(s);
            }
            continue;
        }
        match whole(t)? {
            (s, Cmp::Le) => within.push(s),
            (s, _) => cut.push(s),
        }
    }
    // the stock: the first of a union, else the first solid it is within
    let (stock, on, bound) = match united.split_first() {
        Some((&s, rest)) => (s, rest.to_vec(), within),
        None => {
            let (&s, rest) = within.split_first()?;
            if rest.is_empty() && cut.is_empty() {
                return None;
            }
            (s, Vec::new(), rest.to_vec())
        }
    };
    Some(SolidDef::Body { stock, on, through: cut, bound })
}

/// **A revolution**: a face's patch swept by a turn about a line in its plane, over an interval
/// of the roll one end of which is nothing — turned `sweep` from where it is drawn, the way the
/// interval runs (`sense: cw` for one below nothing).
fn revolution(sk: &Sketch, terms: &[RegionTerm]) -> Option<SolidDef> {
    use crate::model::Sense;
    use std::f64::consts::TAU;
    let [t] = terms else { return None };
    let RegionShape::Swept { motion, roll, face } = t.shape else { return None };
    if t.cmp != Cmp::Le || t.hi.is_some() || stated(t)? != 0.0 {
        return None;
    }
    let (axis, a, d, ratio) = turn_of(sk, motion)?;
    let f = sk.faces.get(face as usize)?;
    let b = match f.plane { Some(p) => sk.basis(p as usize), None => crate::plane::Basis::page() };
    if !lies_in(&b, a, d) {
        return None;
    }
    // a line turned about lies in the face's own plane as drawn, as `about:` held it
    if axis.kind == EntKind::Line {
        let l = sk.lines.get(axis.i())?;
        if [l.p1, l.p2].iter().any(|&p| sk.plane_of(p as usize).map(|x| x as u32) != f.plane) {
            return None;
        }
    }
    let (t0, t1) = (ratio * roll[0], ratio * roll[1]);
    let (lo, hi) = (t0.min(t1), t0.max(t1));
    let (sweep, sense) = match (lo == 0.0, hi == 0.0) {
        (true, false) => (hi, Sense::Ccw),
        (false, true) => (-lo, Sense::Cw),
        _ => return None,
    };
    if sweep > TAU + 1e-12 {
        return None;
    }
    let sweep = match (sweep - TAU).abs() < 1e-12 {
        true => Extent { text: String::new(), value: TAU },
        false => Extent::at(sweep),
    };
    Some(SolidDef::Revolve { face, axis, sweep, sense })
}

/// A stated number of a term, its `lo`, where nothing grows it.
fn stated(t: &RegionTerm) -> Option<f64> {
    (t.grow.is_none() && t.lo.free.is_none()).then_some(t.lo.c)
}

/// The face plane's frame and the frame's origin point, where the face is drawn in a plane.
pub fn face_plane(sk: &Sketch, face: u32) -> Option<(usize, u32)> {
    let p = sk.faces.get(face as usize)?.plane? as usize;
    let origin = sk.frames.get(p)?.chart.as_ref()?.origin;
    Some((p, origin))
}

/// **A prism**: the points whose foot on `sec`'s plane is inside it, between two ordinates along
/// its normal from that plane — and, drafted, its section at each ordinate its own moved in by the
/// ordinate's size times the draft's tangent (`draft`).
fn prism(sk: &Sketch, terms: &[RegionTerm]) -> Option<SolidDef> {
    let mut face = None;
    let mut inside = false;
    let mut leaning = None;
    let (mut lo, mut hi) = (None, None);
    let one = |slot: &mut Option<u32>, f: u32| slot.replace(f).is_none_or(|g| g == f);
    for t in terms {
        match &t.shape {
            RegionShape::Section { face: f } if t.cmp == Cmp::Le && t.hi.is_none() => {
                if stated(t)? != 0.0 || inside || !one(&mut face, *f) {
                    return None;
                }
                inside = true;
            }
            RegionShape::Plane { .. } => {}
            RegionShape::Measured { of } if leaning.is_none() && t.cmp == Cmp::Ge => {
                let (f, k, plane) = lean(of)?;
                if stated(t)? != 0.0 || !one(&mut face, f) {
                    return None;
                }
                leaning = Some((k, plane));
            }
            _ => return None,
        }
    }
    let face = face?;
    let (frame, origin) = face_plane(sk, face)?;
    // the drawn plane a lean is measured from is the face's own
    if leaning.is_some_and(|(_, plane)| plane != EntRef::frame(frame)) {
        return None;
    }
    for t in terms {
        let RegionShape::Plane { along, from, sign } = t.shape else { continue };
        if along != EntRef::frame(frame) || from != origin || sign != 1.0 {
            return None;
        }
        let v = stated(t)?;
        let (a, b) = match t.cmp {
            Cmp::Ge => (Some(v), None),
            Cmp::Le => (None, Some(v)),
            Cmp::In => (Some(v), Some(t.hi?)),
        };
        for (slot, x) in [(&mut lo, a), (&mut hi, b)] {
            if let Some(x) = x {
                if slot.replace(x).is_some() {
                    return None;
                }
            }
        }
    }
    let (from, to) = (lo?, hi?);
    // undrafted, the section is the set's own; drafted, it is the lean's, and a section kept as
    // well holds only where the lean takes it in
    let draft = match leaning.map(|(k, _)| k) {
        None if inside => None,
        None => return None,
        Some(k) if inside && k < 0.0 => return None,
        Some(k) => {
            // the angle as a draughtsman states it, to a billionth of a degree
            let degrees = (k.atan().to_degrees() * 1e9).round() / 1e9;
            (degrees != 0.0).then(|| Extent {
                text: format!("{}deg", crate::vocab::num(degrees)),
                value: degrees.to_radians(),
            })
        }
    };
    (from < to).then(|| SolidDef::Prism { face, from: Extent::at(from), to: Extent::at(to), draft })
}

/// **A draft's lean**, `distance(p, beyond) − distance(p, within) ≥ k · distance(p, P)` over
/// `beyond := { q | q outside sec }`, `within := { q | q inside sec }` and `sec`'s plane `P`: the
/// face and `k`, the draft's tangent — the signed distance into the section at least `k` times
/// the ordinate's size.  `None` for any other sum.
fn lean(of: &[(crate::model::Target, f64)]) -> Option<(u32, f64, EntRef)> {
    use crate::model::Target;
    let mut face = None;
    let (mut out, mut into, mut plane) = (None, None, None);
    for (t, c) in of {
        match t {
            Target::Set { terms, .. } => {
                let [s] = terms.as_slice() else { return None };
                let RegionShape::Section { face: f } = s.shape else { return None };
                if s.hi.is_some() || stated(s)? != 0.0 || face.replace(f).is_some_and(|g| g != f) {
                    return None;
                }
                let slot = match s.cmp { Cmp::Ge => &mut out, Cmp::Le => &mut into, Cmp::In => return None };
                if slot.replace(*c).is_some() {
                    return None;
                }
            }
            Target::Ent(e) if e.kind == EntKind::Frame => {
                if plane.replace((*e, *c)).is_some() {
                    return None;
                }
            }
            _ => return None,
        }
    }
    let (a, b, (e, c)) = (out?, into?, plane?);
    (a > 0.0 && b == -a).then(|| (face.expect("a set"), -c / a, e))
}

/// **A turn**: motion `m` as a rotation about a fixed line with no advance — the line it turns
/// about as an entity (a drawn line, or a frame's axis), a point of it and its unit direction in
/// space, and its ratio (radians turned a radian of the roll).  `None` for any other motion.
pub fn turn_of(sk: &Sketch, m: u32) -> Option<(EntRef, [f64; 3], [f64; 3], f64)> {
    use crate::model::MotionDef;
    let node = sk.motions.get(m as usize)?;
    match node.def {
        MotionDef::Rotation { axis, .. } => {
            let (ratio, advance) = node.rotation(sk).ok()?;
            let l = sk.lines.get(axis as usize)?;
            let (a, b) = (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize));
            let d = crate::space::normalised(crate::space::sub(b, a))?;
            (advance == 0.0).then_some((EntRef::line(axis as usize), a, d, ratio))
        }
        MotionDef::Joint { axis, ratio, advance } => {
            let x = sk.axes.get(axis as usize)?;
            let a = x.a.map(|q| sk.params[q as usize].value);
            let d = crate::space::normalised(x.d.map(|q| sk.params[q as usize].value))?;
            (advance == 0.0).then_some((EntRef::new(EntKind::Axis, axis as usize), a, d, ratio))
        }
        _ => None,
    }
}

/// Whether the line through `a` along the unit `d` lies in the plane of `b`, to the solve's
/// acceptance at the plane's reach.
pub fn lies_in(b: &crate::plane::Basis, a: [f64; 3], d: [f64; 3]) -> bool {
    use crate::space::{dot, norm, sub};
    let n = b.normal();
    let reach = 1.0 + norm(sub(a, b.o));
    dot(n, d).abs() < 1e-9 && dot(n, sub(a, b.o)).abs() < 1e-9 * reach
}
