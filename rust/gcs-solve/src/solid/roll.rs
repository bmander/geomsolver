//! **How far a sweep rolls** (§6.9, #271): `sweep(tool, through: body, under: m)` rolls the tool
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
//! native path read one interval.  A placed copy (`removal at(under: index) θ`) moves
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
        Roll::Stated(r) => return Ok(r.radians()),
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
    // where each way's last step not clear ended, which its next steps are walked toward first
    let mut toward: [Option<Hint>; 2] = [None, None];
    let mut k = 0_u32;
    while ends.iter().any(Option::is_none) {
        let asks: Vec<(usize, u32)> = (0..2).filter(|&w| ends[w].is_none())
            .flat_map(|w| (k..k + ahead).map(move |j| (w, j))).collect();
        let answers = crate::par::map(&asks, |&(w, j)| {
            let [a, b] = [j, j + 1].map(|n| WAYS[w] * f64::from(n) * STEP);
            march(&tool, support, &blank, &family, [a.min(b), a.max(b)], toward[w])
        });
        for (&(w, j), answer) in asks.iter().zip(answers) {
            let (clear, ended) = answer?;
            toward[w] = ended.or(toward[w]);
            if ends[w].is_none() && clear {
                ends[w] = Some(WAYS[w] * f64::from(j) * STEP);
            }
        }
        k += ahead;
        if let Some(w) = (0..2).find(|&w| ends[w].is_none() && f64::from(k) * STEP >= REACH) {
            // a tool never clear either way is always in mesh: the roll to state is a period
            if ends.iter().all(Option::is_none) {
                if let Some(p) = period(&tool, support, &family) {
                    return Err(format!("its tool is always in mesh with it: roll it a whole period, `over t in (0deg, {}deg)`",
                                       (p.to_degrees()*1e6).round()/1e6));
                }
            }
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

/// The least roll, a degree at a time within `PERIOD_TURNS` turns, after which `family` stands
/// `tool` where it stood at 0 again — a whole period of the pair as the observer sees it, the
/// tool's own symmetry counted: the tool's field read alike, to rounding, at every point of a grid
/// over its support carried back from the later pose as from the first (a turn of the tool onto
/// itself leaves its distances where they were; any other moves some of them).
fn period(tool: &SpatialField, support: [I; 3], family: &crate::motion::Family) -> Option<f64> {
    let n = 12;
    let lo = support.map(|x| x.bounds()[0]);
    let hi = support.map(|x| x.bounds()[1]);
    let size = (0..3).map(|k| hi[k] - lo[k]).fold(0.0, f64::max);
    let first = family.pose_at(0.0).ok()?;
    let points: Vec<([f64; 3], f64)> = (0..n * n * n).map(|i| {
        let at = [i % n, (i / n) % n, i / (n * n)];
        let x: [f64; 3] = std::array::from_fn(|k| lo[k] + (hi[k] - lo[k]) * (at[k] as f64 + 0.5) / n as f64);
        (first.point(x), tool.value(x))
    }).collect();
    if !points.iter().any(|p| p.1 < 0.0) { return None; }
    let near = 1e-7 * (1.0 + size);
    (1..=(PERIOD_TURNS * 360.0) as u32).map(|k| f64::from(k) * STEP).find(|&t| {
        family.pose_at(t).is_ok_and(|m| { let back = m.inverse(); points.iter().all(|&(y, v)| (tool.value(back.point(y)) - v).abs() <= near) })
    })
}

/// How many turns `period` looks through.
const PERIOD_TURNS: f64 = 12.0;

/// Whether `tool` under every pose of `family` over `span` is proved clear of `blank`: the
/// tool's support and the span split, a box and the poses over its span at a time, until each box
/// is proved free of the tool's material or carried clear of the blank's.  The span is halved
/// where the poses smear a box more than its own size (a fast turn far from its axis), the box
/// otherwise.  `false` where a box holds material of both at its centre at its span's middle
/// pose, or where the splitting runs out.
pub fn clear(tool: &SpatialField, support: [I; 3], blank: &SpatialField,
             family: &crate::motion::Family, span: [f64; 2]) -> Result<bool, String> {
    march(tool, support, blank, family, span, None).map(|(clear, _)| clear)
}

/// **A step not clear is found where the step before ended** (#271): `clear`'s boxes make one
/// tree, each box's halves fixed by the box, and its answer is whether a box of it ends the march
/// or there are more than `BOXES` of them — however the tree is walked.  A step the tool has not
/// cleared ends where its splitting runs out, near where the step before it ended (`Hint`), and a
/// march taking the halves in their own order walks to it through hundreds of boxes dropped on
/// the way; so where a box is halved, the half toward that end is walked first.  The answer is the
/// march's (a pose that cannot be read is an error wherever it is met); only the boxes asked
/// before it are fewer.  `clear`, with where it ended where it is not clear.
fn march(tool: &SpatialField, support: [I; 3], blank: &SpatialField,
         family: &crate::motion::Family, span: [f64; 2], toward: Option<Hint>)
         -> Result<(bool, Option<Hint>), String> {
    // a span's enclosed poses and its middle pose, asked once for every box under it
    let poses = |span: [f64; 2]| -> Result<Poses, String> {
        let enclosed = I::new(span[0], span[1]).and_then(|s| family.bounds(s)).ok();
        Ok(Poses { span, enclosed, middle: family.at(0.5 * (span[0] + span[1]))? })
    };
    let ended = |cell: [I; 3], at: &Poses| Hint {
        point: cell.map(I::mid),
        at: (0.5 * (at.span[0] + at.span[1]) - span[0]) / (span[1] - span[0]),
    };
    let mut boxes = vec![(support, poses(span)?, 0_u32)];
    let mut asked = 0;
    while let Some(node) = boxes.pop() {
        asked += 1;
        if asked > BOXES {
            return Ok((false, None));
        }
        match visit(tool, blank, &poses, &node)? {
            Visit::Dropped => {}
            Visit::Ends => return Ok((false, Some(ended(node.0, &node.1)))),
            // the half toward where the step before ended on top, to be walked first
            Visit::Halves([a, b]) => match toward {
                Some(h) if !second(&[a, b], h, span) => boxes.extend([b, a]),
                _ => boxes.extend([a, b]),
            },
        }
    }
    Ok((true, None))
}

/// A box of `clear`'s march: its cell of the tool's support, the poses over its span, its depth.
type Node = ([I; 3], Poses, u32);

/// What one box of the march comes to.
enum Visit {
    /// No tool in it, or the tool in it carried clear of the blank over its whole span.
    Dropped,
    /// Material of both at its centre at its span's middle pose, or its splitting run out.
    Ends,
    /// Its two halves, in the order the march stacks them.
    Halves([Node; 2]),
}

/// One box of `clear`'s march, put to its tests.
fn visit(tool: &SpatialField, blank: &SpatialField,
         poses: &impl Fn([f64; 2]) -> Result<Poses, String>, node: &Node) -> Result<Visit, String> {
    let (cell, at, depth) = (node.0, &node.1, node.2);
    // no tool here, or the tool here carried clear of the blank over the whole span
    if tool.bounds(cell).is_ok_and(|t| t.bounds()[0] > 0.0) {
        return Ok(Visit::Dropped);
    }
    let posed = at.enclosed.and_then(|m| m.point(cell).ok());
    if posed.is_some_and(|p| blank.bounds(p).is_ok_and(|k| k.bounds()[0] > 0.0)) {
        return Ok(Visit::Dropped);
    }
    let c = cell.map(I::mid);
    if tool.value(c) < 0.0 && blank.value(at.middle.point(c)) < 0.0 {
        return Ok(Visit::Ends);
    }
    if depth == DEPTH {
        return Ok(Visit::Ends);
    }
    if posed.is_some_and(|p| widest(&p) > 2.0 * widest(&cell)) {
        let m = 0.5 * (at.span[0] + at.span[1]);
        let half = |span: [f64; 2]| -> Result<Node, String> { Ok((cell, poses(span)?, depth + 1)) };
        return Ok(Visit::Halves([half([at.span[0], m])?, half([m, at.span[1]])?]));
    }
    let k = (0..3).max_by(|&i, &j| side(cell[i]).total_cmp(&side(cell[j]))).expect("three sides");
    let [lo, hi] = cell[k].bounds();
    let m = cell[k].mid();
    let half = |h: Result<I, crate::interval::Error>| -> Result<Node, String> {
        let mut next = cell;
        next[k] = h.map_err(fault)?;
        Ok((next, *at, depth + 1))
    };
    Ok(Visit::Halves([half(I::new(lo, m))?, half(I::new(m, hi))?]))
}

/// Where a step's march ended (`march`): the centre of the box, and its span's middle as a
/// fraction of the step's.
#[derive(Clone, Copy)]
struct Hint { point: [f64; 3], at: f64 }

/// Whether the march's dive toward `h`, in a step over `span`, takes a box's second half: the
/// side its cell was halved on, or its span where that was.
fn second(halves: &[Node; 2], h: Hint, span: [f64; 2]) -> bool {
    let [a, b] = halves;
    match (0..3).find(|&i| a.0[i] != b.0[i]) {
        Some(i) => h.point[i] >= b.0[i].bounds()[0],
        None => span[0] + h.at * (span[1] - span[0]) >= b.1.span[0],
    }
}

/// The poses over one span of a step: enclosed (`None` where they cannot be), and the middle one.
#[derive(Clone, Copy)]
struct Poses { span: [f64; 2], enclosed: Option<crate::motion::MotionBounds>, middle: crate::envelope::Motion }
