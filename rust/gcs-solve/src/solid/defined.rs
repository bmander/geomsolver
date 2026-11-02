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
//! | `p coincident(under: m, t in (a, b)) sec` (`Swept` of a `Section` patch), `m` a turn about a line in `sec`'s plane, one end of the interval nothing | `Revolve { face: sec, axis, sweep: b − a, sense }` |

use crate::model::{EntKind, EntRef, Extent, RegionShape, RegionTerm, Sketch, SolidDef};
use crate::vocab::Cmp;

/// The sweep a region's terms define, where an entry recognises them; `None` otherwise.  Every
/// number recognised is stated: a term reading an unknown of the drawing stays the region's.
pub fn recognised(sk: &Sketch, terms: &[RegionTerm]) -> Option<SolidDef> {
    prism(sk, terms)
}

/// A stated number of a term, its `lo`, where nothing grows it.
fn stated(t: &RegionTerm) -> Option<f64> {
    (t.grow.is_none() && t.lo.free.is_none()).then_some(t.lo.c)
}

/// The face plane's frame and the frame's origin point, where the face is drawn in a plane.
fn face_plane(sk: &Sketch, face: u32) -> Option<(usize, u32)> {
    let p = sk.faces.get(face as usize)?.plane? as usize;
    let origin = sk.frames.get(p)?.chart.as_ref()?.origin;
    Some((p, origin))
}

/// **A prism**: the points whose foot on `sec`'s plane is inside it, between two ordinates along
/// its normal from that plane.
fn prism(sk: &Sketch, terms: &[RegionTerm]) -> Option<SolidDef> {
    let mut face = None;
    let (mut lo, mut hi) = (None, None);
    for t in terms {
        match t.shape {
            RegionShape::Section { face: f } if t.cmp == Cmp::Le && t.hi.is_none() => {
                if stated(t)? != 0.0 || face.replace(f).is_some() {
                    return None;
                }
            }
            RegionShape::Plane { .. } => {}
            _ => return None,
        }
    }
    let face = face?;
    let (frame, origin) = face_plane(sk, face)?;
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
    (from < to).then(|| SolidDef::Prism {
        face,
        from: Extent::at(from),
        to: Extent::at(to),
        draft: None,
    })
}

/// Whether `e` is a face's entity (an ordinate written along a face before its plane is known).
pub fn along_face(e: EntRef) -> bool {
    e.kind == EntKind::Face
}
