//! **A bound chooses the root** (§9.6): how a bound reads on the drawing, the point it
//! measures carried across its edge, and a document solve steered to the root where every
//! bound holds.

use crate::constraints::Constraint;
use crate::model::{EntRef, Sketch};
use crate::newton;
use crate::system::System;

use super::{BlockMode, SolveOpts, SolveResult, BOUND_BROKEN};

/// How many bounds one solve steers, each once (`System::steer`).
const STEER_ROUNDS: usize = 4;

/// How far past its edge, as a fraction of the drawing's extent, a bound still holds: the
/// solve's own relative tolerance, so a bound the solution stands on holds.
const STEER_TOL: f64 = 1e-6;

/// A bound as the drawing reads it (§9.6): what the steering, the elaborator and the diagnosis
/// all ask, from one reading.
#[derive(Clone, Debug)]
pub struct BoundReading {
    /// Whether it holds — within the solve's own tolerance, so one the solution stands on does.
    pub holds: bool,
    /// Whether the solution stands on its edge.
    pub edge: bool,
    /// Where a point carried across it aims, nearest first (`Bound::aims`): the mirror of the
    /// reading, then farther past the edge.
    pub aims: Vec<f64>,
}

/// How bound `c` reads on the drawing, `tol` the solve's own (`bound_tol`); `None` for a
/// constraint that is no bound.
pub fn bound_reading(
    sk: &Sketch,
    c: &crate::constraints::Constraint,
    tol: f64,
) -> Option<BoundReading> {
    let b = c.bound?;
    let m = c.reading(sk)?;
    let lo = c.args[c.kind.dimension_slot()?].num();
    // `tol` is a length, the solve's relative tolerance over the drawing's extent; an angle is
    // held to the relative tolerance itself (#217): a cone's side read off the angle a point makes
    // with its axis, a grow of a hundredth of a millimetre on a drawing hundreds across, is a
    // turn smaller than the length tolerance read as radians
    let tol = if reads_angle(c) { tol / sk.extent() } else { tol };
    let edge = (m - lo).abs() <= tol || b.hi.is_some_and(|hi| (m - hi).abs() <= tol);
    let aims = b.aims(lo, m, c.reading_range());
    Some(BoundReading { holds: b.holds(lo, m, tol), edge, aims })
}

/// Whether bound `c` reads an angle: held to the solve's relative tolerance itself, not a
/// length's over the drawing's extent read as radians (`bound_reading`, `across`).
fn reads_angle(c: &Constraint) -> bool {
    c.kind.dimension_kind() == crate::constraints::SpecKind::Angle
}

/// How far past its edge a bound still holds: the solve's own relative tolerance, over the
/// drawing's extent.
pub fn bound_tol(sk: &Sketch) -> f64 {
    STEER_TOL * sk.extent()
}

/// Whether every bound holds on the drawing as it stands (§9.6).
pub fn bounds_hold(sk: &Sketch) -> bool {
    let tol = bound_tol(sk);
    sk.constraints.iter().all(|c| bound_reading(sk, c, tol).is_none_or(|r| r.holds))
}

/// Every bound the drawing breaks, by index, with where its point aims, nearest first.
fn broken(sk: &Sketch) -> Vec<(usize, Vec<f64>)> {
    let tol = bound_tol(sk);
    sk.constraints.iter().enumerate()
        .filter_map(|(i, c)| bound_reading(sk, c, tol).filter(|r| !r.holds).map(|r| (i, r.aims)))
        .collect()
}

/// The parameters a solve moves, the ones `across` may step: asked once a pass, since nothing
/// carried changes which they are.
pub(super) fn movable(sk: &Sketch) -> std::collections::BTreeSet<u32> {
    sk.free_indices().into_iter().map(|i| i as u32).collect()
}

/// **A bound chooses the seed's side** (§9.6): each bound the pose breaks has the point it
/// measures carried across its edge (`across`) — what the elaborator asks of the seeds, before
/// what is seeded from them is read.  The points carried.
pub(crate) fn cross_bounds(sk: &mut Sketch) -> Vec<usize> {
    let free = movable(sk);
    broken(sk).into_iter().filter_map(|(ci, aims)| across(sk, ci, aims[0], &free)).collect()
}

/// Carry the point bound `ci` measures across its edge, to where the bound reads `aim`: the last
/// of its point operands with a coordinate the solve moves, by Newton steps on the reading alone
/// over those coordinates (by differences, so a reading through a lift or a squared kernel is
/// the same question), its twins and their lifts brought to where it stands.  What a seed on the
/// bound's side would have been.  The point moved, or `None` where none can be, or the reading
/// will not come round.
pub(super) fn across(
    sk: &mut Sketch,
    ci: usize,
    aim: f64,
    free: &std::collections::BTreeSet<u32>,
) -> Option<usize> {
    let c = sk.constraints[ci].clone();
    let own = |sk: &Sketch, p: EntRef| -> Vec<usize> {
        sk.own_params(p).into_iter().filter(|i| free.contains(i)).map(|i| i as usize).collect()
    };
    // the point operands, last first, then the other operands' points — a cone's angle is
    // between two lines, and the point it measures is the end of the second
    let is_point = |e: &EntRef| e.kind == crate::model::EntKind::Point;
    let (pts, others): (Vec<EntRef>, Vec<EntRef>) =
        c.entities().into_iter().rev().partition(is_point);
    let mut points = pts;
    for e in others {
        points.extend(sk.children(e).into_iter().rev().filter(is_point));
    }
    let q = points.into_iter().find(|&p| !own(sk, p).is_empty())?;
    let cols = own(sk, q);
    // the point's twins (§6.7), each seen where it stands in the twin's plane, and every lift
    // among them, found once
    let group: Vec<(usize, Option<usize>)> =
        sk.twin_group(q.i()).into_iter().map(|t| (t, sk.lift_of(t))).collect();
    let place = |sk: &mut Sketch| {
        let at = sk.world_point(q.i());
        for &(t, lift) in &group {
            if t != q.i() {
                let (u, v) = sk.on_view_sheet(at, sk.plane_of(t));
                for (i, x) in sk.own_params(EntRef::point(t)).into_iter().zip([u, v]) {
                    sk.params[i as usize].value = x;
                }
            }
            if let Some(k) = lift {
                for (x, v) in sk.lifts[k].x.into_iter().zip(at) {
                    sk.params[x as usize].value = v;
                }
            }
        }
    };
    let extent = sk.extent();
    // a length's reading to the drawing's size, an angle's to the tolerance itself
    let (tol, h) = (STEER_TOL * if reads_angle(&c) { 1.0 } else { extent }, 1e-7 * extent);
    let mut nudged = false;
    for _ in 0..ACROSS_STEPS {
        let m = c.reading(sk)?;
        if (m - aim).abs() <= tol {
            return Some(q.i());
        }
        let mut g = Vec::with_capacity(cols.len());
        for &i in &cols {
            sk.params[i].value += h;
            place(sk);
            g.push((c.reading(sk).unwrap_or(m) - m) / h);
            sk.params[i].value -= h;
        }
        let gg: f64 = g.iter().map(|x| x * x).sum();
        // where the reading is stationary — a point on a cone's axis, whose angle is alike all
        // round it — the point is nudged off it once, and stepped from there
        if !(gg > STEER_TOL * STEER_TOL) {
            if nudged {
                place(sk);
                return None;
            }
            nudged = true;
            for &i in &cols {
                sk.params[i].value += 1e-3 * extent;
            }
            place(sk);
            continue;
        }
        for (&i, gi) in cols.iter().zip(&g) {
            sk.params[i].value += (aim - m) * gi / gg;
        }
        place(sk);
    }
    c.reading(sk).filter(|m| (m - aim).abs() <= tol).map(|_| q.i())
}

/// How many Newton steps `across` takes the reading: one for an ordinate's, a few for a
/// distance's.
const ACROSS_STEPS: usize = 8;

impl System {
    /// **A bound chooses the root** (§9.6) — the second line, the elaborator having chosen the
    /// seeds' side (`program::elaborate`): read on the solution, and where one does not hold, the
    /// solve is steered to the root where it does, by reflect and re-solve as a traced body's
    /// orientation predicate is (`locus`).  The point the bound measures is carried across its edge
    /// to the mirror of the solution's reading (`Bound::aim`) in the pose the solve started from —
    /// carrying with it every bound steered before, or it would undo them — and the drawing solved
    /// again from there; failing that, from the solution.  Kept only where it solves and the bound
    /// holds; a bound still broken fails the solve (status `BOUND_BROKEN`), since a solver may not
    /// report a solution violating one.  A document solve's step (`retry`): a drag keeps the branch
    /// it is on.  Nothing is solved again where every bound holds, so a drawing with none, or with
    /// all kept, solves to the same bits as before.
    pub(super) fn steer(
        &mut self,
        sk: &mut Sketch,
        opts: SolveOpts,
        mut start: Vec<f64>,
        mut res: SolveResult,
    ) -> SolveResult {
        // each bound once, at most `STEER_ROUNDS` of them: a budget, so a drawing breaking many
        // does not pay a solve for each
        let mut steered = Vec::new();
        let free = movable(sk);
        let origin = start.clone();
        while let Some((ci, aims)) = broken(sk).into_iter().next() {
            if steered.contains(&ci) || steered.len() >= STEER_ROUNDS {
                break;
            }
            steered.push(ci);
            let kept = sk.get_x();
            // each aim, nearest first, from where the solve began and then from the solution
            let attempts = aims.iter().flat_map(|&aim| [(aim, true), (aim, false)]);
            let mut found = None;
            for (aim, from_start) in attempts {
                sk.set_x(if from_start { &start } else { &kept });
                if across(sk, ci, aim, &free).is_none() {
                    continue;
                }
                let crossed = sk.get_x();
                let again = self.solve_unsteered(sk, opts);
                let tol = bound_tol(sk);
                if again.success
                    && bound_reading(sk, &sk.constraints[ci], tol).is_some_and(|r| r.holds)
                {
                    found = Some((again, from_start.then_some(crossed)));
                    break;
                }
            }
            let Some((again, crossed)) = found else {
                sk.set_x(&kept);
                let _ = self.solve_unsteered(sk, opts); // back in step with the pose kept
                break;
            };
            res = again;
            // the next bound steered starts on this one's side too, or it would undo it
            if let Some(crossed) = crossed {
                start = crossed;
            }
        }
        // what crossing one point at a time cannot reach — a line run the other way round, an
        // offset's other hand, where every point of a block moves together — the block pass finds
        // by searching each block's roots for the one its bounds say (`Search`), from where the
        // solve began; kept only where it solves and every bound holds
        if !broken(sk).is_empty()
            && opts.blocks != BlockMode::Off
            && self.block_order().blocks.len() >= 2
        {
            let kept = sk.get_x();
            sk.set_x(&origin);
            let mut z = self.z0(sk);
            let (again, _) = self.blocks_then_polish(sk, &mut z, &opts);
            sk.set_x(&self.full_x(&z));
            if again.success && broken(sk).is_empty() {
                res = again;
            } else {
                sk.set_x(&kept);
                self.resync(sk); // back in step with the pose kept
            }
        }
        // named where it is reported (`Diagnosis::bounds_violated`), not here
        if !broken(sk).is_empty() {
            res.success = false;
            res.status = BOUND_BROKEN;
            res.message = newton::status_message(BOUND_BROKEN).to_string();
        }
        res
    }
}
