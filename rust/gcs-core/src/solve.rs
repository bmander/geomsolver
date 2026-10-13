//! Solving and interactive dragging.
//!
//! `Drag` is the one point-drag implementation (pull + polish), `RadiusDrag` its scalar
//! counterpart for circle/arc radii (a `Radius` with `soft` set — its residual is already
//! r − target, so it needs no kernel of its own).  Front ends only translate coordinates.
//!
//! Stage 5 robustness lives here: continuation (a far cursor jump is taken in increments so the
//! solution tracks its homotopy branch instead of teleporting across it) and order-type guards (a
//! step that would flip a guarded triangle's orientation is retried with smaller increments, and
//! an unavoidable flip is recorded and flagged).

use crate::constraints::{CKind, Constraint};
use crate::curve;
use crate::linalg::Mat;
use crate::model::{increments, orientation, EntRef, Sketch};
use crate::newton::{self, Info, Method, TrustRegion};
use crate::sparse::Ata;
use crate::system::{SolveBlock, Subset, System, DENSE_MAX};

/// A `SolveResult::status` of its own: every row holds, and a bound does not (§9.6).
pub const BOUND_BROKEN: i32 = 5;

/// How many times larger or smaller than the extent a system was compiled at a failed solve's
/// pose must be for it to be compiled again at that size (`System::resized`).
const RESIZE: f64 = 4.0;

/// How many times a solve that ends on a root of the constants it was compiled with, and no
/// root of the drawing's, is solved on from there (`System::solve_unsteered`).
const REFRAMES: usize = 3;

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
    let tol = if c.kind.dimension_kind() == crate::constraints::SpecKind::Angle {
        tol / sk.extent()
    } else {
        tol
    };
    let edge = (m - lo).abs() <= tol || b.hi.is_some_and(|hi| (m - hi).abs() <= tol);
    let aims = b.aims(lo, m, c.reading_range());
    Some(BoundReading { holds: b.holds(lo, m, tol), edge, aims })
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
fn movable(sk: &Sketch) -> std::collections::BTreeSet<u32> {
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
fn across(
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
    // (`bound_reading`)
    let angle = c.kind.dimension_kind() == crate::constraints::SpecKind::Angle;
    let (tol, h) = (STEER_TOL * if angle { 1.0 } else { extent }, 1e-7 * extent);
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

#[derive(Clone, Debug)]
pub struct SolveResult {
    /// Hard residuals satisfy the caller's acceptance tolerance. This is independent
    /// of why the numerical iteration stopped (`status`).
    pub success: bool,
    pub status: i32,
    pub message: String,
    /// Over all residuals, soft ones included.
    pub residual_norm: f64,
    /// Over hard residuals only — what "solved" means.
    pub max_residual: f64,
    pub nfev: i32,
    pub njev: i32,
    /// Filled in by the bindings, which own the clock.
    pub time_s: f64,
    pub method: String,
    pub iterations: i32,
    /// Numerical rank of J at the solution (dense path).
    pub rank: Option<i32>,
}

impl SolveResult {
    /// The pose is a solve's answer: accepted, *and* stopped for a reason other than the
    /// iteration limit.  A DogLeg still descending when its budget runs out can be under
    /// `acceptance_tol` in an ill-conditioned direction and a thousandth of the extent from the
    /// solution; it is a success, and not settled (docs/iteration-limit-rescue-plan.md).
    pub fn settled(&self) -> bool {
        self.success && self.status != 4
    }

    pub fn plain(method: &str, success: bool, max_residual: f64, nfev: i32) -> SolveResult {
        SolveResult {
            success,
            status: 0,
            message: method.to_string(),
            residual_norm: max_residual,
            max_residual,
            nfev,
            njev: 0,
            time_s: 0.0,
            method: method.to_string(),
            iterations: 0,
            rank: None,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SolveOpts {
    pub method: Method,
    /// The residual, in each row's own units (`System::row_scale`), below which the iteration
    /// stops. This is separate from the hard-row acceptance criterion below.
    pub tol: f64,
    /// Finite positive hard-row residual limit for success and the DogLeg-to-LM retry.
    /// Defaults to the interactive solver's 1e-6. Analytic geometry can require tighter
    /// accuracy without choosing an optimizer or reimplementing the retry policy.
    /// This scaled equation residual is not a bound on spatial position error.
    pub acceptance_tol: f64,
    pub max_nfev: i32,
    pub writeback: bool,
    pub max_iter: i32,
    pub dense: Option<bool>,
    /// Put curve contacts back on their curves and rebuild when one has moved to another span.
    /// True everywhere except where the caller owns a *pair* of systems that have to stay in
    /// step — `PullPolish`, which re-homes both of its own.
    pub rehome: bool,
    /// Rerun an unconverged DogLeg solve with LM, from the start it had — a stall is a
    /// stationary point, so restarting *there* learns nothing on either method.  True except in
    /// the per-frame drag systems, whose cost per frame is bounded on purpose (and whose pull is
    /// a compromise that rarely "converges" by the hard rows' measure anyway).
    pub retry: bool,
    /// Whether and when the equations are solved block by block, in their block-triangular
    /// order (`System::block_order`), before a whole-system polish.  See `BlockMode`.
    pub blocks: BlockMode,
}

/// When a solve takes the block-triangular path: each strongly connected block of the equations
/// minimised on its own, in order, every column it does not own held where the blocks before it
/// put them, and then one whole-system DogLeg from there (the *polish*, which also settles the
/// over- and under-determined parts no block holds).  Only a DogLeg solve takes it, and never a
/// system of fewer than two blocks, where the pass would be the whole-system solve again.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlockMode {
    /// After a whole-system DogLeg that did not settle (`SolveResult::settled`) and before the LM
    /// retry, only with `retry`: kept only if it succeeds after a failure, and only if it settles
    /// after a stop on the iteration limit (`System::block_rescue`).  A document the whole-system
    /// solve settles never sees it, so it solves to the same bits it always did.
    Rescue,
    /// Before the whole-system DogLeg — the block pass and its polish first, with the plain
    /// DogLeg and then LM as the retries.  For measuring; not the default.
    First,
    /// Never.
    Off,
}

/// The step, relative to the block's own unknowns, below which a block's minimisation stops
/// (`newton::Tol::xtol`).  A block is accepted only at the caller's `acceptance_tol`, and its few
/// columns make the whole solve's 1e-12 a coarse limit: an angle on a short line at 180° needs a
/// step of a few 1e-12 mm to take its last 4e-12 radians, which that stops short of — as the
/// whole-system polish does from there, with a norm the whole vector's size behind its `xtol`.
const BLOCK_XTOL: f64 = 1e-15;

impl Default for SolveOpts {
    fn default() -> SolveOpts {
        SolveOpts {
            method: Method::DogLeg,
            tol: 1e-14,
            acceptance_tol: 1e-6,
            max_nfev: 0,
            writeback: true,
            max_iter: 100,
            dense: None,
            rehome: true,
            retry: true,
            blocks: BlockMode::Rescue,
        }
    }
}

impl System {
    /// Solve, and put any curve contact back on the curve it names.
    ///
    /// A compiled system names one span of a spline in its columns, and the parameter that chose
    /// that span is itself an unknown the solve moves — so a contact can finish on a different
    /// piece of the curve than it started on, or off the end of it entirely.  Either way the
    /// columns are no longer the ones the answer belongs to, and the system rebuilds itself; a
    /// parameter pinned to the end of its curve is held for the retry, since free it walks
    /// straight back off the end and nothing is learned.  Neither can happen many times and
    /// `curve::MAX_REHOME` bounds it regardless.
    ///
    /// This is the one place the rule lives, so every caller gets it — the one-shot `solve`, the
    /// plan solver's fallback and a front end that compiled a system for itself alike.
    pub fn solve(&mut self, sk: &mut Sketch, opts: SolveOpts) -> SolveResult {
        // a document solve over bounds starts where it may be steered back to (§9.6)
        let steers =
            opts.retry && opts.writeback && sk.constraints.iter().any(|c| c.bound.is_some());
        let resizes = opts.retry && opts.writeback;
        let start = if steers || resizes { sk.get_x() } else { Vec::new() };
        let mut res = self.solve_unsteered(sk, opts);
        if resizes && !res.success {
            res = self.resized(sk, opts, &start, res);
        }
        if resizes && !res.success {
            res = self.recharted(sk, opts, res);
        }
        if !steers || !res.success {
            return res;
        }
        self.steer(sk, opts, start, res)
    }

    /// **A drawing is judged at its own size** (#217): a system's rows and columns are scaled by
    /// the extent the sketch had where it was compiled (`System::new`), and a start that is not
    /// the drawing's size — every point unseeded, about the origin at the size of a unit, in a
    /// drawing hundreds of millimetres across — judges a root against the wrong one: a squared
    /// length's last bits read as a residual over the acceptance, and the block pass stops at the
    /// first block that places something at the drawing's size.  A document solve that fails
    /// where it ends at another size, more than `RESIZE` either way, is compiled again there and
    /// solved again from where it began; kept where it succeeds, else the first stands.
    fn resized(
        &mut self,
        sk: &mut Sketch,
        opts: SolveOpts,
        start: &[f64],
        res: SolveResult,
    ) -> SolveResult {
        let (was, now) = (self.extent.max(1.0), sk.extent());
        if !(now > RESIZE * was || now * RESIZE < was) {
            return res;
        }
        let kept = sk.get_x();
        let compiled = std::mem::replace(self, System::new(sk));
        sk.set_x(start);
        let again = self.solve_unsteered(sk, opts);
        if again.success {
            return again;
        }
        *self = compiled;
        sk.set_x(&kept);
        let z = self.z0(sk);
        let _ = self.residuals(&z); // back in step with the pose kept
        res
    }

    /// **A chart is gauged where the solve ends** (#217): a set's derivative along a chart
    /// (`Toward::Chart`) takes the point's tangent as `e_a + s·e_c`, `c` the world axis the sets'
    /// normal ran most along where the point stood when the chart was chosen
    /// (`Sketch::choose_charts`) — for an unseeded point, about the origin, its sets' geometry
    /// turned from where the solve puts it.  There `c` can lie in the tangent plane the solve ends
    /// at, a direction no tangent there takes, and the derivative's rows cannot hold.  A document
    /// solve that fails is gauged again where it ends (`Sketch::rechoose_charts`) and, where a
    /// chart changed, compiled again and solved on from there; kept where it succeeds, else the
    /// first stands, its charts as they were.
    fn recharted(&mut self, sk: &mut Sketch, opts: SolveOpts, res: SolveResult) -> SolveResult {
        let before = sk.clone();
        if !sk.rechoose_charts() {
            return res;
        }
        let compiled = std::mem::replace(self, System::new(sk));
        let again = self.solve_unsteered(sk, opts);
        if again.success {
            return again;
        }
        *self = compiled;
        *sk = before;
        let z = self.z0(sk);
        let _ = self.residuals(&z); // back in step with the pose kept
        res
    }

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
    fn steer(
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
                let z = self.z0(sk);
                let _ = self.residuals(&z); // back in step with the pose kept
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

    fn solve_unsteered(&mut self, sk: &mut Sketch, opts: SolveOpts) -> SolveResult {
        let mut res = self.solve_rehomed(sk, opts);
        // **Constants read off the pose are read again where the solve ends** (#217): a row in
        // space taken across a line (`parallel`, a point on a line, a plane over a line's hidden
        // axis) is measured along two directions across the line as it stood when the system was
        // compiled, and its zero holds there only where the line has not turned square to that —
        // a seed far from the drawing, every point unseeded, can end on such a root, which is no
        // root of the drawing's.  So a document solve that succeeds re-reads its constants there
        // (`refresh_consts`) and, where its rows no longer hold, solves on from where it stopped.
        if opts.retry && opts.writeback {
            for _ in 0..REFRAMES {
                if !res.success {
                    break;
                }
                self.refresh_consts(sk);
                let z = self.z0(sk);
                if self.worst_hard(&z) < opts.acceptance_tol {
                    break;
                }
                res = self.solve_rehomed(sk, opts);
            }
        }
        res
    }

    /// The largest hard row at `z`, over its own units: what "solved" reads.
    fn worst_hard(&mut self, z: &[f64]) -> f64 {
        let r = self.residuals(z);
        let hard = r.iter().zip(&self.hard).filter(|(_, &h)| h);
        hard.fold(0.0, |w, (v, _)| if v.is_nan() { f64::NAN } else { w.max(v.abs()) })
    }

    fn solve_rehomed(&mut self, sk: &mut Sketch, opts: SolveOpts) -> SolveResult {
        if !opts.acceptance_tol.is_finite() || opts.acceptance_tol <= 0. {
            let mut result = SolveResult::plain(opts.method.as_str(),false,f64::INFINITY,0);
            result.status = -1;
            result.message = "acceptance tolerance must be finite and positive".into();
            return result;
        }
        // a curve family's contact has no span to walk off, but a domain to be clamped to
        let rehome =
            opts.rehome && opts.writeback && !(self.spans().is_empty() && sk.curves.is_empty());
        // A contact *seeded* off the end of its curve is brought onto it before anything is
        // solved, and left free.  The pin below is for a solve that walked off — free again, it
        // walks straight back — but a seed is only where the search begins, and one written a
        // knot span too far says nothing about where the answer is: pinned to the end for the
        // retry, `p coincident s hint(t: 2)` nailed `p` to the curve's last point and a document
        // with a unique solution came back UNSOLVED (#45.7, spec P3).  From the end and free, the
        // solve walks in to the answer, or back off to be clamped and pinned as before.
        if rehome && !curve::clamp_contacts(sk).is_empty()
            && curve::contact_spans(sk) != *self.spans()
        {
            *self = System::new(sk);
        }
        let mut res = self.solve_compiled(sk, opts);
        if !rehome {
            return res;
        }
        for _ in 0..curve::MAX_REHOME {
            let pinned = curve::clamp_contacts(sk);
            if pinned.is_empty() && curve::contact_spans(sk) == *self.spans() {
                break;
            }
            for &t in &pinned {
                sk.params[t as usize].fixed = true;
            }
            *self = System::new(sk);
            res = self.solve_compiled(sk, opts);
            if !pinned.is_empty() {
                for &t in &pinned {
                    sk.params[t as usize].fixed = false;
                }
                *self = System::new(sk); // the caller's system has to match the sketch it left
            }
        }
        res
    }

    fn solve_compiled(&mut self, sk: &mut Sketch, opts: SolveOpts) -> SolveResult {
        let z0 = self.z0(sk);
        let dogleg = opts.method == Method::DogLeg;
        let mut z = z0.clone();
        let (mut res, mut rel) = if dogleg && opts.blocks == BlockMode::First {
            self.blocks_then_polish(sk, &mut z, &opts)
        } else {
            self.minimise(&mut z, opts.method, &opts)
        };
        // the pose `self`'s x was last evaluated at is `z`'s, until a retry that is not kept
        let mut in_step = true;
        if opts.retry && dogleg && !res.settled() {
            // With fewer than two blocks the pass would be this DogLeg again, from the same start.
            if opts.blocks == BlockMode::Rescue && self.block_order().blocks.len() >= 2 {
                in_step = self.block_rescue(sk, &z0, &mut z, &mut res, &mut rel, &opts);
            } else if opts.blocks == BlockMode::First && !res.success {
                // `First` ran the block pass already: its retry is the plain DogLeg
                let mut z2 = z0.clone();
                let (res2, rel2) = self.minimise(&mut z2, Method::DogLeg, &opts);
                if res2.success || rel2 < rel || rel.is_nan() {
                    (z, res, rel, in_step) = (z2, res2, rel2, true);
                } else {
                    in_step = false;
                }
            }
        }
        // DogLeg is a local method, and a squared kernel's full Gauss–Newton step can overshoot
        // clean out of the solution's basin — a rectangle carrying its redundant perpendiculars,
        // asked for its second side length, stalls in a residual minimum that is no solution.
        // LM's damping takes the gradient path and converges; the stall itself is a stationary
        // point, so the retry is from the start the first run had, and the better pose is kept.
        if opts.retry && dogleg && !res.success {
            let mut z2 = z0;
            let (res2, rel2) = self.minimise(&mut z2, Method::Lm, &opts);
            if res2.success || rel2 < rel || rel.is_nan() {
                (z, res, in_step) = (z2, res2, true);
            } else {
                in_step = false;
            }
        }
        if !in_step {
            let _ = self.residuals(&z); // the core's x back in step with the pose kept
        }
        if opts.writeback {
            let x = self.full_x(&z);
            sk.set_x(&x);
        }
        res
    }

    /// The block-triangular rescue of a whole-system DogLeg that did not settle: the block pass
    /// and its polish, tried from one start after another, each kept only if it clears its own
    /// bar.  A system whose seeds are good only upstream of a block can fail whole and solve in
    /// order, each block from what the blocks before it made; and a DogLeg accepted on its
    /// iteration limit, still descending in an ill-conditioned direction, can be a thousandth of
    /// the extent from the solution (docs/iteration-limit-rescue-plan.md).  So:
    ///
    /// * **a failure** is tried from the start, kept if it succeeds;
    /// * **a stop on the limit** is tried from the stop, kept if it converges there (status 0) —
    ///   the root the DogLeg was making for, finished — and a pass that stalls again (a step or
    ///   gradient stop above the tolerance) says the stop is in a basin with no solution in it,
    ///   so it is tried from the start, kept if it settles.
    ///
    /// Nothing clearing its bar, the first pass that settled is kept; nothing settling, the
    /// DogLeg's own pose, status and success stand bit for bit — a failing document's pose is
    /// what the diagnosis reads.  Whether `self`'s x is still `z`'s is the answer: true exactly
    /// when the pass kept is the last one run.
    fn block_rescue(
        &mut self,
        sk: &Sketch,
        z0: &[f64],
        z: &mut Vec<f64>,
        res: &mut SolveResult,
        rel: &mut f64,
        opts: &SolveOpts,
    ) -> bool {
        type Bar = fn(&SolveResult) -> bool;
        let stop = z.clone();
        let tries: Vec<(&[f64], Bar)> = if res.success {
            vec![(&stop[..], |r| r.settled() && r.status == 0), (z0, SolveResult::settled)]
        } else {
            vec![(z0, |r| r.success)]
        };
        let mut passes = Vec::new();
        for (from, bar) in tries {
            let mut zb = from.to_vec();
            let (rb, relb) = self.blocks_then_polish(sk, &mut zb, opts);
            if bar(&rb) {
                (*z, *res, *rel) = (zb, rb, relb);
                return true;
            }
            passes.push((zb, rb, relb));
        }
        if let Some(pass) = passes.into_iter().find(|(_, rb, _)| rb.settled()) {
            (*z, *res, *rel) = pass;
        }
        false
    }

    /// The block pass from `z`, then the whole-system DogLeg from where it ends — the polish,
    /// which is the result.  A block is accepted when its rows meet `acceptance_tol` and every
    /// bound it decides holds (`Search`); where it does not, its root is searched for, and the
    /// first that does not come out so ends the pass, and the polish starts from the blocks solved
    /// so far (and whatever that one's own minimisation made of it).  With fewer than two blocks
    /// there is no order to exploit and this is the whole-system DogLeg alone.
    fn blocks_then_polish(
        &mut self,
        sk: &Sketch,
        z: &mut [f64],
        opts: &SolveOpts,
    ) -> (SolveResult, f64) {
        let order = self.block_order();
        let (mut nfev, mut njev) = (0, 0);
        if order.blocks.len() >= 2 {
            let mut search = Search::new(self, sk);
            let mut solved = std::collections::BTreeSet::new();
            // a constant read off the pose — the directions across a line its rows are taken
            // along — is read where the blocks before put it, not where it was seeded: a line
            // turned far from its seed admits a root no word says (#217); the compiled ones are
            // put back for the polish, which starts on the pass's solution, so a pass not kept
            // leaves the system as it found it
            let compiled: Vec<Vec<f64>> = self.blocks.iter().map(|b| b.consts.clone()).collect();
            for b in &order.blocks {
                search.sk.set_x(&self.full_x(z));
                self.refresh_consts(&search.sk);
                let start: Vec<f64> = b.cols.iter().map(|&c| z[c]).collect();
                let (mut ok, info) = self.solve_block(b, z, &start, opts);
                (nfev, njev) = (nfev + info.nfev, njev + info.njev);
                let decided = search.deciding(&b.cols, &solved);
                if !(ok && search.holds(self, z, &decided)) {
                    // the root the words say, or else what the block's own start came to
                    let first: Vec<f64> = b.cols.iter().map(|&c| z[c]).collect();
                    let mut found = false;
                    for attempt in search.attempts(self, z, b, &start, &solved, &decided) {
                        let (mut fine, mut info) = self.solve_block(b, z, &attempt, opts);
                        if !fine {
                            (fine, info) = self.follow_block(b, z, &attempt, opts);
                        }
                        (nfev, njev) = (nfev + info.nfev, njev + info.njev);
                        if fine && search.holds(self, z, &decided) {
                            (ok, found) = (true, true);
                            break;
                        }
                    }
                    if !found {
                        for (k, &c) in b.cols.iter().enumerate() {
                            z[c] = first[k];
                        }
                    }
                }
                if !ok {
                    break;
                }
                solved.extend(b.cols.iter().copied());
            }
            for (b, consts) in self.blocks.iter_mut().zip(compiled) {
                b.consts = consts;
            }
        }
        let (mut res, rel) = self.minimise(z, Method::DogLeg, opts);
        if nfev > 0 {
            res.method = "blocks".into();
            res.nfev += nfev;
            res.njev += njev;
        }
        (res, rel)
    }

    /// One block minimised from `start` (its own columns), every other column held at `z`, which
    /// it is written back into: whether its rows meet `acceptance_tol`, and the minimisation's
    /// count.
    fn solve_block(
        &mut self,
        b: &SolveBlock,
        z: &mut [f64],
        start: &[f64],
        opts: &SolveOpts,
    ) -> (bool, Info) {
        let gtol = 1e-16 / self.extent.max(1.0);
        let tol = newton::Tol { ftol: opts.tol, xtol: BLOCK_XTOL, gtol };
        let max_nfev = if opts.max_nfev <= 0 { 4 * opts.max_iter } else { opts.max_nfev };
        let sub = self.subset(&b.instances, &b.cols);
        let mut zb = start.to_vec();
        let mut t = BlockTr::new(self, sub, z);
        let mut r = vec![0.0; t.m()];
        t.residuals_into(&zb, &mut r);
        let info = newton::dogleg(&mut t, &mut zb, &mut r, tol, opts.max_iter, max_nfev);
        for (k, &c) in b.cols.iter().enumerate() {
            z[c] = zb[k];
        }
        // `dogleg` keeps `r` the residuals at `zb`; NaN fails the comparison
        (r.iter().all(|v| v.abs() < opts.acceptance_tol), info)
    }

    /// One block followed from `start` to a root along the Newton homotopy `r(z) = (1 − t)
    /// r(start)`, `t` from 0 to 1 in `HOMOTOPY_STEPS`, each step a DogLeg from the last: where
    /// the DogLeg from a start far from every root runs off, the path keeps to the root the start
    /// leads to.  Written back into `z` as `solve_block` does.
    fn follow_block(
        &mut self,
        b: &SolveBlock,
        z: &mut [f64],
        start: &[f64],
        opts: &SolveOpts,
    ) -> (bool, Info) {
        let gtol = 1e-16 / self.extent.max(1.0);
        let tol = newton::Tol { ftol: opts.tol, xtol: BLOCK_XTOL, gtol };
        let max_nfev = if opts.max_nfev <= 0 { 4 * opts.max_iter } else { opts.max_nfev };
        let sub = self.subset(&b.instances, &b.cols);
        let mut zb = start.to_vec();
        let mut path = Path { inner: BlockTr::new(self, sub, z), r0: Vec::new(), left: 1.0 };
        let mut r = vec![0.0; path.m()];
        path.inner.residuals_into(&zb, &mut r);
        path.r0 = r.clone();
        let mut total = Info { status: 0, nfev: 0, njev: 0, iterations: 0, rank: -1 };
        for k in 1..=HOMOTOPY_STEPS {
            path.left = 1.0 - k as f64 / HOMOTOPY_STEPS as f64;
            path.residuals_into(&zb, &mut r);
            let last = k == HOMOTOPY_STEPS;
            let info = newton::dogleg(&mut path, &mut zb, &mut r, tol, opts.max_iter, max_nfev);
            total.nfev += info.nfev;
            total.njev += info.njev;
            total.status = info.status;
            if last || !r.iter().all(|v| v.is_finite()) {
                break;
            }
        }
        for (k, &c) in b.cols.iter().enumerate() {
            z[c] = zb[k];
        }
        (r.iter().all(|v| v.abs() < opts.acceptance_tol), total)
    }

    /// One minimisation from `z`, updated in place, measured on the hard rows: the result and
    /// the max residual relative to each row's own units, which is what "solved" means.
    fn minimise(&mut self, z: &mut [f64], method: Method, opts: &SolveOpts) -> (SolveResult, f64) {
        // Every residual the system hands out is over its row's own units (`System::row_scale`),
        // so `tol` is the tolerance as stated and the gradient's units are 1/length: a scaled
        // row's derivative with respect to a world-length column is O(1/extent) whatever the
        // row's degree, so stationarity is judged against that.
        let info: Info = newton::solve_system(
            self,
            method,
            opts.tol,
            1e-12,
            1e-16 / self.extent.max(1.0),
            opts.max_iter,
            opts.max_nfev,
            opts.dense,
            z,
        );
        let r = self.residuals(z);
        let mut n2 = 0.0;
        let mut rel = 0.0f64;
        // NaN is not "no error", and `f64::max` would drop it: track it and let it win at the end
        let mut nan = false;
        for i in 0..r.len() {
            n2 += r[i] * r[i];
            if self.hard[i] {
                let a = r[i].abs();
                if a.is_nan() {
                    nan = true;
                } else {
                    rel = rel.max(a);
                }
            }
        }
        if nan {
            rel = f64::NAN;
        }
        let mx = rel;
        let res = SolveResult {
            // relative, not absolute: a radius kernel's residual is a length and a distance
            // kernel's is a length squared, so one absolute threshold cannot judge both
            success: info.status >= 0 && rel < opts.acceptance_tol,
            status: info.status,
            message: newton::status_message(info.status).to_string(),
            residual_norm: n2.sqrt(),
            max_residual: mx,
            nfev: info.nfev,
            njev: info.njev,
            time_s: 0.0,
            method: method.as_str().to_string(),
            iterations: info.iterations,
            rank: if info.rank < 0 { None } else { Some(info.rank) },
        };
        (res, rel)
    }
}

/// How many steps a block followed along its Newton homotopy takes (`System::follow_block`).
const HOMOTOPY_STEPS: usize = 16;

/// A block's rows less `left` of what they were at the path's start (`System::follow_block`).
struct Path<'a> {
    inner: BlockTr<'a>,
    r0: Vec<f64>,
    left: f64,
}

impl TrustRegion for Path<'_> {
    fn n(&self) -> usize {
        self.inner.n()
    }
    fn m(&self) -> usize {
        self.inner.m()
    }
    fn residuals_into(&mut self, z: &[f64], out: &mut [f64]) {
        self.inner.residuals_into(z, out);
        for (o, r) in out.iter_mut().zip(&self.r0) {
            *o -= self.left * r;
        }
    }
    fn jacobian_at(&mut self, z: &[f64]) {
        self.inner.jacobian_at(z)
    }
    fn jt_mul(&mut self, v: &[f64], out: &mut [f64]) {
        self.inner.jt_mul(v, out)
    }
    fn j_mul(&mut self, v: &[f64], out: &mut [f64]) {
        self.inner.j_mul(v, out)
    }
    fn gn_step(&mut self, r: &[f64], g: &[f64], p: &mut [f64]) {
        self.inner.gn_step(r, g, p)
    }
    fn rank(&self) -> i32 {
        self.inner.rank()
    }
}

/// How many starts a block's root search scatters its points from (`Search::attempts`), after
/// the crossings: enough that one lands in each basin a block of a few points has.
const RESTARTS: usize = 48;

/// **A block's root, as the words say it** (§9.6, #217): what the block pass reads to choose among
/// a block's roots.  Every bound, by the columns it reads, so a block is held to the bounds it
/// decides — each column it reads among those solved so far, and some the block's own; and every
/// point and axis the solve moves, by its columns, so a block whose own start comes to no root, or
/// to one a bound refuses, starts again elsewhere: first carried across each bound it breaks as a
/// steered solve is (`across`), then with its points scattered about what the blocks before it
/// placed, at their size, its axes turned (`attempts`).  Deterministic, a fixed stream per start:
/// the drawing chooses, and not a draw.  Read once a pass, off a scratch copy of the sketch set to
/// the pass's pose where a bound or a place is read.
struct Search {
    sk: Sketch,
    free: std::collections::BTreeSet<u32>,
    /// Each bound: its constraint, and the columns its reading reads.
    bounds: Vec<(usize, Vec<usize>)>,
    /// Each point the solve moves: the point, and its own columns with its lift's.
    points: Vec<(usize, Vec<usize>)>,
    /// Each axis the solve turns: its direction's columns.
    axes: Vec<[usize; 3]>,
}

impl Search {
    fn new(sys: &System, sk: &Sketch) -> Search {
        let col = |p: u32| usize::try_from(sys.col_of[p as usize]).ok();
        let cols = |ps: Vec<u32>| {
            let mut c: Vec<usize> = ps.into_iter().filter_map(col).collect();
            c.sort_unstable();
            c.dedup();
            c
        };
        let bounds = sk.constraints.iter().enumerate()
            .filter(|(_, c)| c.bound.is_some())
            .map(|(i, c)| (i, cols(c.params(sk))))
            .filter(|(_, c)| !c.is_empty())
            .collect();
        let points = (0..sk.points.len()).filter_map(|p| {
            let mut own = sk.point_all_params(p);
            if let Some(k) = sk.lift_of(p) {
                own.extend(sk.lifts[k].x);
            }
            let c = cols(own);
            (!c.is_empty()).then_some((p, c))
        }).collect();
        let axes = sk.axes.iter().filter_map(|a| {
            let d = a.d.map(col);
            d.iter().all(Option::is_some).then(|| d.map(Option::unwrap))
        }).collect();
        Search { sk: sk.clone(), free: movable(sk), bounds, points, axes }
    }

    /// The bounds block `cols` decides, given the columns `solved` before it.
    fn deciding(&self, cols: &[usize], solved: &std::collections::BTreeSet<usize>) -> Vec<usize> {
        let own = |c: &usize| cols.binary_search(c).is_ok();
        self.bounds.iter()
            .filter(|(_, read)| {
                read.iter().any(own) && read.iter().all(|c| own(c) || solved.contains(c))
            })
            .map(|&(i, _)| i)
            .collect()
    }

    /// Whether each of the bounds `decided` holds at the pose `z`.
    fn holds(&mut self, sys: &System, z: &[f64], decided: &[usize]) -> bool {
        if decided.is_empty() {
            return true;
        }
        self.sk.set_x(&sys.full_x(z));
        let tol = bound_tol(&self.sk);
        decided.iter().all(|&ci| {
            bound_reading(&self.sk, &self.sk.constraints[ci], tol).is_none_or(|r| r.holds)
        })
    }

    /// Point `p` put at `w` in space on the scratch sketch — a point drawn in a plane at its
    /// shadow there — its lift with it; only its free numbers are written.
    fn place(&mut self, p: usize, w: [f64; 3]) {
        let at = if self.sk.points[p].z.is_some() { w.to_vec() } else {
            let (u, v) = self.sk.on_view_sheet(w, self.sk.plane_of(p));
            vec![u, v]
        };
        for (i, v) in self.sk.point_all_params(p).into_iter().zip(at) {
            if self.free.contains(&i) {
                self.sk.params[i as usize].value = v;
            }
        }
        if let Some(l) = self.sk.lift_of(p) {
            let w = self.sk.world_point(p);
            for (i, v) in self.sk.lifts[l].x.into_iter().zip(w) {
                self.sk.params[i as usize].value = v;
            }
        }
    }

    /// The block's columns as the scratch sketch has them.
    fn block_of(&self, sys: &System, b: &SolveBlock) -> Vec<f64> {
        let x = self.sk.get_x();
        b.cols.iter().map(|&c| {
            let v = x[sys.free[c] as usize];
            v * sys.col_scale[c]
        }).collect()
    }

    /// Where block `b` may start again, its own start (`start`, with `z` the block's solution)
    /// having come to no root or to one a bound it decides refuses: carried across each such bound
    /// from its solution and from its start, then scattered.
    fn attempts(
        &mut self,
        sys: &System,
        z: &[f64],
        b: &SolveBlock,
        start: &[f64],
        solved: &std::collections::BTreeSet<usize>,
        decided: &[usize],
    ) -> Vec<Vec<f64>> {
        let mut out = Vec::new();
        let mut from_start = z.to_vec();
        for (k, &c) in b.cols.iter().enumerate() {
            from_start[c] = start[k];
        }
        let x_start = sys.full_x(&from_start);
        let x_solved = sys.full_x(z);
        // across each bound the solution breaks
        self.sk.set_x(&x_solved);
        let tol = bound_tol(&self.sk);
        let broken: Vec<(usize, f64)> = decided.iter().filter_map(|&ci| {
            let r = bound_reading(&self.sk, &self.sk.constraints[ci], tol)?;
            (!r.holds).then(|| (ci, r.aims[0]))
        }).collect();
        let own = |c: &usize| b.cols.binary_search(c).is_ok();
        let moving: Vec<usize> =
            self.points.iter().filter(|(_, c)| c.iter().any(own)).map(|&(p, _)| p).collect();
        let turning: Vec<[usize; 3]> =
            self.axes.iter().filter(|d| d.iter().any(own)).copied().collect();
        for x in [&x_solved, &x_start] {
            for &(ci, aim) in &broken {
                self.sk.set_x(x);
                let before: Vec<[f64; 3]> =
                    moving.iter().map(|&p| self.sk.world_point(p)).collect();
                let free = std::mem::take(&mut self.free);
                let moved = across(&mut self.sk, ci, aim, &free);
                self.free = free;
                let Some(q) = moved else { continue };
                out.push(self.block_of(sys, b));
                // and the block mirrored across the plane the point was carried through: the
                // other hand of what it is, where the point alone comes back to where it was
                let Some(k) = moving.iter().position(|&p| p == q) else { continue };
                let (from, to) = (before[k], self.sk.world_point(q));
                let Some(n) = crate::space::normalised(crate::space::sub(to, from)) else {
                    continue;
                };
                let mid = [0, 1, 2].map(|i| (from[i] + to[i]) / 2.0);
                let mirror = |w: [f64; 3]| {
                    let h = crate::space::dot(crate::space::sub(w, mid), n);
                    [0, 1, 2].map(|i| w[i] - 2.0 * h * n[i])
                };
                self.sk.set_x(x);
                for (&p, &w) in moving.iter().zip(&before) {
                    self.place(p, mirror(w));
                }
                for d in &turning {
                    let v = d.map(|c| self.sk.params[sys.free[c] as usize].value);
                    let h = crate::space::dot(v, n);
                    for (i, &c) in d.iter().enumerate() {
                        self.sk.params[sys.free[c] as usize].value = v[i] - 2.0 * h * n[i];
                    }
                }
                out.push(self.block_of(sys, b));
            }
        }
        // scattered: its points about what the blocks before it placed, at that size
        if (moving.is_empty() && turning.is_empty()) || b.cols.len() > DENSE_MAX {
            return out;
        }
        self.sk.set_x(&x_solved);
        let placed: Vec<[f64; 3]> = self.points.iter()
            .filter(|(_, c)| c.iter().all(|c| solved.contains(c)))
            .map(|&(p, _)| self.sk.world_point(p))
            .collect();
        let n = placed.len().max(1) as f64;
        let centre = placed
            .iter()
            .fold([0.0; 3], |a, w| [a[0] + w[0] / n, a[1] + w[1] / n, a[2] + w[2] / n]);
        let size = placed
            .iter()
            .map(|w| crate::space::norm(crate::space::sub(*w, centre)))
            .fold(0.0, f64::max);
        let size = if size > 0.0 { size } else { sys.extent.max(1.0) };
        for k in 0..RESTARTS {
            let seed = 0x2170_0000 ^ (k as u32).wrapping_mul(2_654_435_761);
            let mut rng = crate::rng::Rng::new(seed);
            self.sk.set_x(&x_start);
            for &p in &moving {
                let w = [0, 1, 2].map(|i| centre[i] + size * rng.uniform(-1.0, 1.0));
                self.place(p, w);
            }
            for d in &turning {
                let mut v = [0, 1, 2].map(|_| rng.uniform(-1.0, 1.0));
                let n = crate::space::norm(v).max(1e-9);
                v = v.map(|x| x / n);
                for (&c, x) in d.iter().zip(v) {
                    self.sk.params[sys.free[c] as usize].value = x;
                }
            }
            out.push(self.block_of(sys, b));
        }
        out
    }
}

/// One block of the block-triangular order as a `TrustRegion`: only its rows, only its columns,
/// every other column read from the whole free vector `z` it was handed and held there.  Rows
/// and columns keep the whole system's units (`System::subset`), so the block is minimised in
/// the same dimensionless measure the whole solve uses, and no `System` is compiled for it —
/// a compile forgets every traced contact's remembered pose.  The Gauss–Newton step is the
/// whole system's by size: minimum norm on a dense matrix up to `DENSE_MAX` columns, the
/// regularized normal equations above.
struct BlockTr<'a> {
    sys: &'a mut System,
    sub: Subset,
    z: Vec<f64>,
    dense: Option<Mat>,
    ata: Option<Ata>,
    rank: i32,
}

impl<'a> BlockTr<'a> {
    fn new(sys: &'a mut System, sub: Subset, z: &[f64]) -> BlockTr<'a> {
        let (m, n) = (sub.rows.len(), sub.cols.len());
        let dense = (n <= DENSE_MAX).then(|| Mat::zeros(m, n));
        BlockTr { sys, sub, z: z.to_vec(), dense, ata: None, rank: -1 }
    }

    /// The whole free vector with the block's own columns at `zb`.
    fn put(&mut self, zb: &[f64]) {
        for (k, &c) in self.sub.cols.iter().enumerate() {
            self.z[c] = zb[k];
        }
    }
}

impl TrustRegion for BlockTr<'_> {
    fn n(&self) -> usize {
        self.sub.cols.len()
    }
    fn m(&self) -> usize {
        self.sub.rows.len()
    }
    fn residuals_into(&mut self, zb: &[f64], out: &mut [f64]) {
        self.put(zb);
        self.sys.subset_residuals(&mut self.sub, &self.z, out);
    }
    fn jacobian_at(&mut self, zb: &[f64]) {
        self.put(zb);
        self.sys.subset_csr(&mut self.sub, &self.z);
        if let Some(j) = &mut self.dense {
            let n = j.cols;
            j.data.iter_mut().for_each(|v| *v = 0.0);
            for r in 0..j.rows {
                for p in self.sub.indptr[r]..self.sub.indptr[r + 1] {
                    let p = p as usize;
                    j.data[r * n + self.sub.indices[p] as usize] = self.sub.data[p];
                }
            }
        }
    }
    fn jt_mul(&mut self, v: &[f64], out: &mut [f64]) {
        self.sub.jt_mul(v, out);
    }
    fn j_mul(&mut self, v: &[f64], out: &mut [f64]) {
        self.sub.j_mul(v, out);
    }
    fn gn_step(&mut self, r: &[f64], g: &[f64], p: &mut [f64]) {
        if let Some(j) = &self.dense {
            self.rank = newton::min_norm_step(j, r, p);
            return;
        }
        let s = &self.sub;
        let ata = self
            .ata
            .get_or_insert_with(|| Ata::new(s.rows.len(), s.cols.len(), &s.indptr, &s.indices));
        newton::normal_step(ata, &s.data, g, p);
    }
    fn rank(&self) -> i32 {
        self.rank
    }
}

/// One-shot: compile and solve, writing the result back into the sketch.
///
/// A sketch with curve contacts in it solves in rounds.  A compiled system names one span of a
/// spline in its columns, and the parameter that chose that span is itself an unknown the solve
/// moves — so a contact can finish the solve on a different piece of the curve than it started
/// on, or off the end of it entirely.  Both are re-homed and the system built again; neither can
/// happen many times, and `curve::MAX_REHOME` bounds it regardless.  A sketch with no curves in
/// it never enters the loop.
pub fn solve(sk: &mut Sketch, opts: SolveOpts) -> SolveResult {
    System::new(sk).solve(sk, opts)
}

pub type Triangle = (usize, usize, usize);

/// The pull/polish protocol every interactive drag shares.
///
/// A soft constraint pulls the geometry toward what the cursor asks for; the hard constraints are
/// then polished on their own so they hold exactly.  Both systems are compiled once, at drag
/// start, and reused for every move — dragging never re-analyses the sketch.  The compile order is
/// load-bearing: `polish` must be built before the soft target joins the sketch, so it contains
/// the hard constraints only.
pub struct PullPolish {
    pub polish: System,
    pub pull: System,
    pub target: u32,
    pub method: Method,
    pub active: bool,
}

/// A drag's two systems have to stay in step, so neither re-homes on its own: `pull` rebuilding
/// itself would leave `polish` on the old span, and `polish` rebuilding itself would pick up the
/// soft drag target it is compiled to exclude.  The pair is re-homed together, below.
const PAIRED: SolveOpts = SolveOpts { rehome: false, ..NO_REHOME };
const NO_REHOME: SolveOpts = SolveOpts {
    method: Method::DogLeg,
    tol: 1e-14,
    acceptance_tol: 1e-6,
    max_nfev: 0,
    writeback: true,
    max_iter: 100,
    dense: None,
    rehome: false,
    retry: false,
    blocks: BlockMode::Rescue,
};

const PULL_ITER: i32 = 4; // the pull is a soft compromise; polish makes it exact
const POLISH_ITER: i32 = 20;

impl PullPolish {
    pub fn new(sk: &mut Sketch, target: Constraint, method: Method) -> PullPolish {
        let polish = System::new(sk);
        let id = sk.add(target);
        let pull = System::new(sk);
        PullPolish { polish, pull, target: id, method, active: true }
    }

    /// Rebuild both systems around the target that is already in the sketch — `polish` has to
    /// see the hard constraints alone, so the target comes out for the length of the compile.
    fn recompile(&mut self, sk: &mut Sketch) {
        let Some(target) = sk.constraint(self.target).cloned() else { return };
        sk.remove(self.target);
        self.polish = System::new(sk);
        sk.add(target);
        self.pull = System::new(sk);
    }

    /// Put every contact back on the curve, and rebuild the pair if one has moved to another
    /// span — the one place a drag ever re-analyses anything, and only for the drags that touch
    /// a curve.  `polish` is the authority on which spans were compiled: it is the system built
    /// from the hard constraints alone.
    fn rehome(&mut self, sk: &mut Sketch) {
        if self.polish.spans().is_empty() {
            return;
        }
        curve::clamp_contacts(sk);
        if curve::contact_spans(sk) != *self.polish.spans() {
            self.recompile(sk);
        }
    }

    /// One frame: push the target's new value in, pull, then make the hard ones exact.
    pub fn pull_polish(&mut self, sk: &mut Sketch) -> SolveResult {
        self.pull.update_consts(sk, self.target);
        self.pull.solve(sk, SolveOpts { method: self.method, max_iter: PULL_ITER, ..PAIRED });
        let r =
            self.polish.solve(sk, SolveOpts { method: self.method, max_iter: POLISH_ITER, ..PAIRED });
        self.rehome(sk);
        r
    }

    pub fn end(&mut self, sk: &mut Sketch) {
        if self.active {
            sk.remove(self.target);
            self.active = false;
        }
    }
}

/// Interactive drag of one point: pull toward the cursor, then polish.
pub struct Drag {
    pub pp: PullPolish,
    pub point: usize,
    pub guards: Vec<Triangle>,
    pub flips: Vec<Triangle>,
    signs: Vec<bool>,
    /// Continuation increment; `PlanDrag` sets it from the document rather than the part.
    pub(crate) max_step: f64,
    last_good: Vec<f64>,
    /// The eye's picture plane (right, up), for a point in space dragged where it is seen
    /// (`Drag::seen`); `None` for a point dragged on its own page.
    eye: Option<([f64; 3], [f64; 3])>,
}

impl Drag {
    pub fn new(
        sk: &mut Sketch,
        point: usize,
        x: f64,
        y: f64,
        method: Method,
        weight: f64,
        guards: Vec<Triangle>,
        max_step_rel: f64,
    ) -> Drag {
        let target = Constraint::drag_target(EntRef::point(point), x, y, weight);
        Drag::with(sk, point, target, None, method, guards, max_step_rel)
    }

    /// A drag of a point in space, read where the eye at bearing `az` and elevation `el` sees it:
    /// (`x`, `y`) and every later target are on the eye's picture plane, and how deep the point
    /// stands along the line of sight is the constraints' to say (`CKind::DragSeen`).
    #[allow(clippy::too_many_arguments)]
    pub fn seen(
        sk: &mut Sketch,
        point: usize,
        x: f64,
        y: f64,
        (az, el): (f64, f64),
        method: Method,
        weight: f64,
        guards: Vec<Triangle>,
        max_step_rel: f64,
    ) -> Drag {
        let target = Constraint::drag_seen(EntRef::point(point), x, y, weight, az, el);
        let eye = Some(crate::overview::eye(az, el));
        Drag::with(sk, point, target, eye, method, guards, max_step_rel)
    }

    fn with(
        sk: &mut Sketch,
        point: usize,
        target: Constraint,
        eye: Option<([f64; 3], [f64; 3])>,
        method: Method,
        guards: Vec<Triangle>,
        max_step_rel: f64,
    ) -> Drag {
        let max_step = max_step_rel * sk.extent().max(1.0);
        let signs = guards.iter().map(|t| orientation(sk, t.0, t.1, t.2) >= 0.0).collect();
        let last_good = sk.get_x();
        let pp = PullPolish::new(sk, target, method);
        Drag { pp, point, guards, flips: Vec::new(), signs, max_step, last_good, eye }
    }

    /// Where the dragged point is, as the targets read it: on its page, or where the eye sees it.
    pub fn at(&self, sk: &Sketch) -> (f64, f64) {
        match self.eye {
            Some((right, up)) => {
                let x = sk.world_point(self.point);
                (crate::space::dot(right, x), crate::space::dot(up, x))
            }
            None => sk.point_xy(self.point),
        }
    }

    fn step(&mut self, sk: &mut Sketch, x: f64, y: f64) -> SolveResult {
        if let Some(c) = sk.constraint_mut(self.pp.target) {
            c.set_target(x, y);
        }
        self.pp.pull_polish(sk)
    }

    fn flipped(&self, sk: &Sketch) -> Vec<usize> {
        (0..self.guards.len())
            .filter(|&i| {
                let t = self.guards[i];
                (orientation(sk, t.0, t.1, t.2) >= 0.0) != self.signs[i]
            })
            .collect()
    }

    /// One increment that would flip a guard: bisect the remaining interval from the last good
    /// state, keeping whatever prefix stays on the branch, within a sub-step budget.
    fn damped(&mut self, sk: &mut Sketch, tx: f64, ty: f64, mut budget: i32) -> (SolveResult, i32) {
        let mut res = self.step(sk, tx, ty);
        // (fx, fy) is the far end of the interval still under suspicion, measured from the last
        // good state.  Halving it *is* the bisection: without that the midpoint below is the same
        // point every time round, and the budget goes on re-testing it.
        let (mut fx, mut fy) = (tx, ty);
        while !self.flipped(sk).is_empty() && budget > 0 {
            let lg = self.last_good.clone();
            sk.set_x(&lg);
            let (bx, by) = self.at(sk);
            let (mx, my) = ((bx + fx) / 2.0, (by + fy) / 2.0);
            res = self.step(sk, mx, my);
            budget -= 1;
            if !self.flipped(sk).is_empty() {
                (fx, fy) = (mx, my); // the flip is in the first half: bisect that
                continue;
            }
            self.last_good = sk.get_x(); // the whole first half is on the branch: keep it
            (fx, fy) = (tx, ty);
            res = self.step(sk, tx, ty); // and try the rest again
            budget -= 1;
        }
        (res, budget)
    }

    pub fn move_to(&mut self, sk: &mut Sketch, x: f64, y: f64) -> SolveResult {
        let n_flips = self.flips.len();
        let mut budget = 12; // cap the sub-steps a single frame may spend
        let (px, py) = self.at(sk);
        self.last_good = sk.get_x();
        let mut res = self.step(sk, px, py);
        for (tx, ty) in increments(px, py, x, y, self.max_step) {
            res = self.step(sk, tx, ty);
            if !self.guards.is_empty() && !self.flipped(sk).is_empty() {
                let (r2, b2) = self.damped(sk, tx, ty, budget);
                res = r2;
                budget = b2;
                for k in self.flipped(sk) {
                    // unavoidable: accept, record, flag
                    self.signs[k] = !self.signs[k];
                    self.flips.push(self.guards[k]);
                }
            }
            self.last_good = sk.get_x();
        }
        if self.flips.len() > n_flips {
            res.message =
                format!("order-type flip in {} triangle(s)", self.flips.len() - n_flips);
        }
        res
    }

    pub fn end(&mut self, sk: &mut Sketch) {
        self.pp.end(sk)
    }
}

/// A `Radius` that does not have to hold: its residual is already exactly r − target, so the
/// scalar pull needs no kernel of its own.
fn soft_radius(circle: EntRef, r: f64) -> Constraint {
    let mut c = Constraint::radius(circle, r);
    c.soft = true;
    c
}

/// Interactive drag of a circle's or arc's radius — the scalar counterpart of `Drag`.
///
/// A radius that is fixed or dimensioned simply does not move: the polish wins, exactly as a point
/// drag compromises on an over-constrained sketch.  An `EqualRadius` chain is a relation rather
/// than a dimension, so the whole chain resizes together.
pub struct RadiusDrag {
    pub pp: PullPolish,
    pub circle: EntRef,
}

impl RadiusDrag {
    pub fn new(sk: &mut Sketch, circle: EntRef, r: f64, method: Method) -> RadiusDrag {
        let pp = PullPolish::new(sk, soft_radius(circle, r), method);
        RadiusDrag { pp, circle }
    }

    pub fn move_to(&mut self, sk: &mut Sketch, r: f64) -> SolveResult {
        // a radius through zero would flip the geometry
        let v = r.max(1e-9);
        if let Some(c) = sk.constraint_mut(self.pp.target) {
            debug_assert_eq!(c.kind, CKind::Radius);
            c.set_num("r", v);
        }
        self.pp.pull_polish(sk)
    }

    pub fn end(&mut self, sk: &mut Sketch) {
        self.pp.end(sk)
    }
}
