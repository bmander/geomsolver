//! **A sweep is its path's, not its pace's** (#684 L): what a tool sweeps is the set of poses its
//! motion stands it at over the roll, whatever the rate it is taken at, how often, or which way.
//! A joint driven by a law of the roll (`MotionDef::Driven`, #391 S5) stands at the joint's own
//! pose at the law's value, so over a roll `[a, b]` it stands the tool at every pose the joint
//! stands it at over the values the law takes there: the law is continuous (`Law::range`), so
//! those are every value between its least and its greatest. Its sweep is the joint's sweep over
//! that interval — a turn, a slide or a screw at a constant rate, each a class the export builds
//! exactly — however the law rises, dwells and comes back.
//!
//! A body that cuts such a sweep is built from a copy of the drawing that writes it so
//! (`lowered`): each sweep it reaches under a driven joint is, in its own place, the sweep of the
//! joint the drive stands (`of`) over the law's range, a roll stated. Every other reader then
//! reads a constant twist: the classes' rows, the exact construction, the accuracy meter, the
//! page's stepped build. Nothing here reads geometry beyond the law: the copy's sweep is the
//! drawing's, pose for pose, and the export holds what it builds to the drawing's own field of the
//! driven sweep. A drive is read so only where it drives the sweep's motion alone: a driven joint
//! seen from another motion (a cam's follower seen from the cam) is a path of its own, whose screw
//! moves as the law runs, and stays the field's.
use crate::model::{MotionDef, Roll, Running, Sketch, SolidDef};

/// How far apart, in radians of the joint's value, the law's least and greatest must be for it to
/// move the tool: below it the tool stands still, and no sweep is read.
const STILL: f64 = 1e-9;

/// The joint sweep `swept`'s motion drives, and the least and greatest values the law takes over
/// the sweep's roll (radians): `None` where its motion is no joint driven by a law, where the law
/// is not read or ranged (an enclosure it carries none of), and where it takes one value only,
/// which stands the tool still. Remembered against what the sweep reads (`Caches::paced`).
pub fn restated(sk: &Sketch, swept: usize) -> Option<(u32, [f64; 2])> {
    let SolidDef::Swept { motion, .. } = &sk.solids.get(swept)?.def else { return None };
    let MotionDef::Driven { of, law, over } = &sk.motions.get(*motion as usize)?.def else {
        return None;
    };
    sk.caches.paced.get_or(swept, super::MemoKey::of(sk, swept), || {
        let law = crate::motion::Law::read(sk, law, over).ok()?;
        let [lo, hi] = law.range(super::roll::interval(sk, swept).ok()?).ok()?;
        (hi - lo > STILL).then_some((*of, [lo, hi]))
    })
}

/// The sweeps body `body` reaches whose motion is one driven joint `restated` reads, each with its
/// joint and its range.
fn driven(sk: &Sketch, body: usize) -> Vec<(usize, u32, [f64; 2])> {
    let mut seen = std::collections::BTreeSet::new();
    let mut stack = vec![body];
    let mut found = Vec::new();
    while let Some(s) = stack.pop() {
        let Some(solid) = sk.solids.get(s) else { continue };
        if !seen.insert(s) {
            continue;
        }
        if let Some((of, range)) = restated(sk, s) {
            found.push((s, of, range));
        }
        stack.extend(solid.operands().into_iter().map(|o| o as usize));
    }
    found
}

/// Whether `body` reaches a sweep under a driven joint, so that it is built from `lowered`.
pub fn lowers(sk: &Sketch, body: usize) -> bool {
    !driven(sk, body).is_empty()
}

/// `sk` with each sweep `body` reaches under a driven joint written, in its own place and under
/// its own name, as its joint's sweep over the law's range (`restated`); `None` where it reaches
/// none.
pub fn lowered(sk: &Sketch, body: usize) -> Option<Sketch> {
    let found = driven(sk, body);
    if found.is_empty() {
        return None;
    }
    let mut out = sk.clone();
    for (s, of, range) in found {
        if let SolidDef::Swept { motion, roll, .. } = &mut out.solids[s].def {
            *motion = of;
            *roll = Roll::Stated(Running::roll_radians(range));
        }
    }
    Some(out)
}
