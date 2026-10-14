//! **How far a sweep rolls** (§6.9, #271): `solid(tool, under: m, through: body)` rolls the tool
//! over the whole roll about 0 during which it meets `body`'s material — its stock and additions,
//! never its cuts, as a `through:` prism spans them — so a roll is no number a document guesses
//! at and admission checks afterwards: within the window the class admits, the cut does not
//! depend on it, and clear of the blank at both limits (E1) is how the interval is found.
//!
//! Found by marching out from 0 each way a `STEP` at a time until the tool, over a whole step, is
//! proved clear of the body: the tool's support split into boxes, each one the tool's own field
//! encloses away from its material dropped, the rest carried through the step's enclosed poses
//! (`Family::bounds`) and dropped where the body's field encloses them away from its material.
//! Conservative: a step not proved clear is marched past, so the roll may run a little past
//! where the tool first clears, never short of it.  A box with material of both at its centre at
//! the step's middle pose ends a step early, as one never proved clear does.  The roll is the
//! near end of each way's first clear step; read once after the solve and remembered against
//! `solid::reads` (`Caches::roll`), so admission, the field, the exact construction and the
//! native path read one interval.  A placed copy (`solid(removal, under: index, at: θ)`) moves
//! the sweep it was found for.
use super::SpatialField;
use crate::interval::Interval as I;
use crate::model::{Roll, Sketch, SolidDef};

/// The march's step, in the motion's own parameter (radians of its roll).
pub const STEP: f64 = std::f64::consts::PI / 180.0;
/// How far the march goes each way before refusing: two turns.
pub const REACH: f64 = 2.0 * std::f64::consts::TAU;
/// How many times a step may halve a box of the tool's support, and how many boxes it may ask,
/// before it stands unproved.
const DEPTH: u32 = 32;
const BOXES: usize = 20_000;

/// Sweep `swept`'s roll `[from, to]`, radians: as stated, or found through its body (cached).
pub fn interval(sk: &Sketch, swept: usize) -> Result<[f64; 2], String> {
    let s = sk.solids.get(swept).ok_or_else(|| format!("no solid at index {swept}"))?;
    let SolidDef::Swept { source, motion, roll } = &s.def else {
        return Err(format!("`{}` is not a continuous motion sweep", s.name));
    };
    let body = match roll {
        Roll::Stated { from, to } => return Ok([from.value, to.value]),
        Roll::Through { body } => *body as usize,
    };
    sk.caches.roll.get_or(swept, super::MemoKey::of(sk, swept), || {
        through(sk, *source as usize, *motion as usize, body)
            .map_err(|m| format!("`{}` rolls through `{}`: {m}", s.name, sk.solids[body].name))
    })
}

/// The ways the march goes from 0: down, then up.
const WAYS: [f64; 2] = [-1.0, 1.0];

fn fault(e: crate::interval::Error) -> String { format!("{e:?}") }

/// How wide an interval is, and a box's widest side.
fn side(x: I) -> f64 { x.bounds()[1] - x.bounds()[0] }
fn widest(b: &[I; 3]) -> f64 { b.iter().map(|&x| side(x)).fold(0.0, f64::max) }

/// The roll about 0 during which `source` under `motion` meets `body`'s material.
fn through(sk: &Sketch, source: usize, motion: usize, body: usize) -> Result<[f64; 2], String> {
    let tolerance = super::cad::AXIS_TOLERANCE;
    let tool = SpatialField::read(sk, source, tolerance)?;
    let Some(support) = tool.support_bounds().map_err(fault)? else {
        return Err(format!("its tool `{}` has no bounded support, so no roll is known to carry it \
                            clear", sk.solids[source].name));
    };
    let mut blank: Option<SpatialField> = None;
    for s in super::document::material_sources(sk, body as u32)? {
        let f = SpatialField::read(sk, s as usize, tolerance)?;
        blank = Some(match blank { Some(b) => b.union(f).map_err(fault)?, None => f });
    }
    let blank = blank.ok_or("it has no material")?;
    let family = crate::motion::Family::read(sk, motion)?;
    // both ways at once, a few steps of each ahead on every core: the first clear step of each
    // way is its end, and a step past it asked meanwhile is only wasted
    let ahead = crate::par::threads().clamp(1, 4) as u32;
    let mut ends: [Option<f64>; 2] = [None, None];
    let mut k = 0_u32;
    while ends.iter().any(Option::is_none) {
        let asks: Vec<(usize, u32)> = (0..2).filter(|&w| ends[w].is_none())
            .flat_map(|w| (k..k + ahead).map(move |j| (w, j))).collect();
        let answers = crate::par::map(&asks, |&(w, j)| {
            let [a, b] = [j, j + 1].map(|n| WAYS[w] * f64::from(n) * STEP);
            clear(&tool, support, &blank, &family, [a.min(b), a.max(b)])
        });
        for (&(w, j), clear) in asks.iter().zip(answers) {
            if ends[w].is_none() && clear? {
                ends[w] = Some(WAYS[w] * f64::from(j) * STEP);
            }
        }
        k += ahead;
        if let Some(w) = (0..2).find(|&w| ends[w].is_none() && f64::from(k) * STEP >= REACH) {
            return Err(format!("its tool is not clear of it within two turns {} of 0",
                               ["below", "above"][w]));
        }
    }
    let [Some(lo), Some(hi)] = ends else { unreachable!("each way has its end") };
    if lo >= hi {
        return Err("its tool does not meet it at 0, so no roll about 0 meets it".into());
    }
    Ok([lo, hi])
}

/// Whether `tool` under every pose of `family` over `span` is proved clear of `blank`: the
/// tool's support and the span split, a box and the poses over its span at a time, until each box
/// is proved free of the tool's material or carried clear of the blank's.  The span is halved
/// where the poses smear a box more than its own size (a fast turn far from its axis), the box
/// otherwise.  `false` where a box holds material of both at its centre at its span's middle
/// pose, or where the splitting runs out.
pub fn clear(tool: &SpatialField, support: [I; 3], blank: &SpatialField,
             family: &crate::motion::Family, span: [f64; 2]) -> Result<bool, String> {
    // a span's enclosed poses and its middle pose, asked once for every box under it
    let poses = |span: [f64; 2]| -> Result<Poses, String> {
        let enclosed = I::new(span[0], span[1]).and_then(|s| family.bounds(s)).ok();
        Ok(Poses { span, enclosed, middle: family.at(0.5 * (span[0] + span[1]))? })
    };
    let mut boxes = vec![(support, poses(span)?, 0_u32)];
    let mut asked = 0;
    while let Some((cell, at, depth)) = boxes.pop() {
        asked += 1;
        if asked > BOXES {
            return Ok(false);
        }
        // no tool here, or the tool here carried clear of the blank over the whole span
        if tool.bounds(cell).is_ok_and(|t| t.bounds()[0] > 0.0) {
            continue;
        }
        let posed = at.enclosed.and_then(|m| m.point(cell).ok());
        if posed.is_some_and(|p| blank.bounds(p).is_ok_and(|k| k.bounds()[0] > 0.0)) {
            continue;
        }
        let c = cell.map(I::mid);
        if tool.value(c) < 0.0 && blank.value(at.middle.point(c)) < 0.0 {
            return Ok(false);
        }
        if depth == DEPTH {
            return Ok(false);
        }
        if posed.is_some_and(|p| widest(&p) > 2.0 * widest(&cell)) {
            let m = 0.5 * (at.span[0] + at.span[1]);
            for half in [[at.span[0], m], [m, at.span[1]]] {
                boxes.push((cell, poses(half)?, depth + 1));
            }
            continue;
        }
        let k = (0..3).max_by(|&i, &j| side(cell[i]).total_cmp(&side(cell[j]))).expect("three sides");
        let [lo, hi] = cell[k].bounds();
        let m = cell[k].mid();
        for half in [I::new(lo, m), I::new(m, hi)] {
            let mut next = cell;
            next[k] = half.map_err(fault)?;
            boxes.push((next, at, depth + 1));
        }
    }
    Ok(true)
}

/// The poses over one span of a step: enclosed (`None` where they cannot be), and the middle one.
#[derive(Clone, Copy)]
struct Poses { span: [f64; 2], enclosed: Option<crate::motion::MotionBounds>, middle: crate::envelope::Motion }
