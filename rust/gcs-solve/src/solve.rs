//! Solving and interactive dragging.
//!
//! `System::solve` is a document's solve: a minimisation (`System::solve_compiled`, retried by
//! `RETRIES`), its curve contacts re-homed, its constants read again where it ends, a failure
//! recovered by `RECOVERIES`, and a bound it breaks steered (`bounds`).  The block-triangular
//! pass and its root search are `blocks`; the drags `drag`, and a body turned whole `turn`; a
//! parent solved through instances it treats as functions of their inputs, `through` (#454).

mod blocks;
mod bounds;
mod drag;
mod through;
mod turn;

pub use bounds::cross_bounds;
pub use bounds::{bound_reading, bound_tol, bounds_hold, BoundReading};
pub use drag::{Drag, PullPolish, RadiusDrag, Triangle};
pub use through::{
    solve_through, Boundaries, Boundary, Evaluation, Failure, Reason, Through, Times,
};
pub use turn::{TurnDrag, Turning};

use crate::curve;
use crate::model::Sketch;
use crate::newton::{self, Info, Method};
use crate::system::System;

/// A `SolveResult::status` of its own: every row holds, and a bound does not (§9.6).
pub const BOUND_BROKEN: i32 = 5;

/// A `SolveResult::status` of its own: every row holds, and a line a tangency reads only by its
/// direction has no length (#479, `collapsed`).
pub const COLLAPSED: i32 = 6;

/// How short, over the drawing's extent, a line a tangency reads by its direction may be and
/// still be a line (`collapsed`): the solve's own relative tolerance, as a bound's.
const COLLAPSE_TOL: f64 = 1e-6;

/// How many times larger or smaller than the extent a system was compiled at a failed solve's
/// pose must be for it to be compiled again at that size (`System::resized`).
const RESIZE: f64 = 4.0;

/// How many times a solve that ends on a root of the constants it was compiled with, and no
/// root of the drawing's, is solved on from there (`System::solve_unsteered`).
pub(super) const REFRAMES: usize = 3;

/// Whether a row of the drawing has constants read off the pose (`CKind::reads_pose`): only then
/// are they read again as a solve goes (`REFRAMES`, the block pass).
fn reads_pose(sk: &Sketch) -> bool {
    sk.constraints.iter().any(|c| c.kind.reads_pose() && c.states_rows())
}

/// **A tangency at a contact is no tangency along a line of no length** (#479): every acting
/// constraint, by id, whose row reads a line only by its direction (`Constraint::direction_line`)
/// where that line is shorter than `COLLAPSE_TOL` of the extent.  Such a row holds whatever the
/// contact, so a run tangent at both its ends is satisfied by its ends together where its two
/// rounds cross: a root of full rank, isolated, that no rank or diagnosis can tell from the
/// tangent.  A document solve ending there fails (`COLLAPSED`), never reported solved.
pub fn collapsed(sk: &Sketch) -> Vec<u32> {
    let floor = COLLAPSE_TOL * sk.extent();
    sk.constraints.iter().filter(|c| c.acts())
        .filter(|c| c.direction_line().is_some_and(|l| !(sk.line_length(l) > floor)))
        .map(|c| c.id)
        .collect()
}

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

    /// What the solve came to, in the words every surface reports it in (#449): *solved* only
    /// where it settled, since a stop on the iteration limit is no answer however small its
    /// residual reads.
    pub fn outcome(&self) -> &'static str {
        if self.settled() {
            "solved"
        } else if self.success {
            "stopped at the iteration limit"
        } else {
            "did not solve"
        }
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

impl SolveOpts {
    /// Whether this is a document's solve, not a drag's frame: it retries (`retry`) and writes
    /// its answer back (`writeback`), so it may be recovered, read again where it ends and
    /// steered.
    pub fn document(&self) -> bool {
        self.retry && self.writeback
    }
}

/// What a failed document solve is tried again under, in order, each only while the solve has
/// not succeeded (`System::solve`): at the drawing's own size, then with its charts gauged where
/// it ended.  Each says whether it applies, setting up the sketch the system is compiled from
/// again, and where that solve starts and what is put back where it fails (`Recovery`).  (A
/// contact on an envelope or a meshing needs none: it is seeded where its block comes up,
/// `blocks::seed_contacts`.)
const RECOVERIES: [fn(&System, &mut Sketch, &[f64]) -> Option<Recovery>; 2] =
    [System::resized, System::recharted];

/// One recovery of a failed document solve (`RECOVERIES`): the system compiled again where the
/// sketch stands, solved on from `from` (where it stands, when `None`), and kept only where that
/// succeeds; else `back` puts the sketch back as it was (`System::recompiled`).
struct Recovery {
    from: Option<Vec<f64>>,
    back: Back,
}

/// What a recovery that fails puts back (`Recovery::back`).
enum Back {
    /// The pose the failed solve ended at.
    Pose(Vec<f64>),
    /// The whole sketch, where the recovery changed more than its pose.
    Sketch(Box<Sketch>),
}

/// How a DogLeg document solve that did not come out is tried again (`System::solve_compiled`),
/// each in turn where it is wanted (`System::wants`), on what the tries before it left.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Retry {
    /// The block pass, after a DogLeg that did not settle, under `BlockMode::Rescue`
    /// (`System::block_rescue`).
    Rescue,
    /// The plain whole-system DogLeg from the start, after a failure under `BlockMode::First`,
    /// which ran the block pass already; the better pose kept.
    DogLeg,
    /// LM from the start, after a failure.  DogLeg is a local method, and a squared kernel's full
    /// Gauss–Newton step can overshoot clean out of the solution's basin — a rectangle carrying
    /// its redundant perpendiculars, asked for its second side length, stalls in a residual
    /// minimum that is no solution.  LM's damping takes the gradient path and converges; the
    /// stall itself is a stationary point, so the retry is from the start the first run had, and
    /// the better pose is kept.
    Lm,
}

/// The retries, in the order they are tried.
const RETRIES: [Retry; 3] = [Retry::Rescue, Retry::DogLeg, Retry::Lm];

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
        let res = self.solve_steered(sk, opts);
        // a circle's gauge taken again where the normal now stands: it moves nothing a row
        // reads but the gauge's own, which still holds (`Sketch::retake_round_gauges`)
        if sk.retake_round_gauges() {
            self.resync(sk);
        }
        res
    }

    fn solve_steered(&mut self, sk: &mut Sketch, opts: SolveOpts) -> SolveResult {
        // a document solve over bounds starts where it may be steered back to (§9.6), and one
        // that fails is tried again from there (`RECOVERIES`)
        let steers = opts.document() && sk.constraints.iter().any(|c| c.bound.is_some());
        let start = if opts.document() { sk.get_x() } else { Vec::new() };
        let mut res = self.solve_unsteered(sk, opts);
        if opts.document() {
            for recovery in RECOVERIES {
                if res.success {
                    break;
                }
                if let Some(again) = recovery(self, sk, &start) {
                    res = self.recompiled(sk, opts, res, again);
                }
            }
        }
        if opts.document() && !res.success {
            res = self.at_own_sizes(sk, opts, res);
        }
        if steers && res.success {
            res = self.steer(sk, opts, start, res);
        }
        // named where it is reported (`Diagnosis::collapsed`), not here
        if opts.document() && res.success && !collapsed(sk).is_empty() {
            res.success = false;
            res.status = COLLAPSED;
            res.message = newton::status_message(COLLAPSED).to_string();
        }
        res
    }

    /// **A drawing is judged at its own size** (#217): a system's rows and columns are scaled by
    /// the extent the sketch had where it was compiled (`System::new`), and a start that is not
    /// the drawing's size — every point unseeded, about the origin at the size of a unit, in a
    /// drawing hundreds of millimetres across — judges a root against the wrong one: a squared
    /// length's last bits read as a residual over the acceptance, and the block pass stops at the
    /// first block that places something at the drawing's size.  A document solve that fails
    /// where it ends at another size, more than `RESIZE` either way, is compiled again there and
    /// solved again from where it began; kept where it succeeds, else the first stands.
    fn resized(&self, sk: &mut Sketch, start: &[f64]) -> Option<Recovery> {
        let (was, now) = (self.extent.max(1.0), sk.extent());
        if !(now > RESIZE * was || now * RESIZE < was) {
            return None;
        }
        Some(Recovery { from: Some(start.to_vec()), back: Back::Pose(sk.get_x()) })
    }

    /// **A chart is gauged where the solve ends** (#217): a set's derivative along a chart
    /// (`Toward::Chart`) takes the point's tangent as `e_a + s·e_c`, `c` the world axis the sets'
    /// normal ran most along where the point stood when the chart was chosen
    /// (`Sketch::choose_charts`) — for an unseeded point, about the origin, its sets' geometry
    /// turned from where the solve puts it.  There `c` can lie in the tangent plane the solve ends
    /// at, a direction no tangent there takes, and the derivative's rows cannot hold.  A document
    /// solve that fails is gauged again where it ends (`Sketch::charts_moved`) and, where a
    /// chart changed, compiled again and solved on from there; kept where it succeeds, else the
    /// first stands, its charts as they were.
    fn recharted(&self, sk: &mut Sketch, _start: &[f64]) -> Option<Recovery> {
        let moved = sk.charts_moved();
        if moved.is_empty() {
            return None;
        }
        let before = sk.clone();
        for (d, c) in moved {
            sk.set_chart(d, c); // which tangents are free: compiled with them
        }
        Some(Recovery { from: None, back: Back::Sketch(Box::new(before)) })
    }

    /// A system compiled where `sk` stands — as the recovery left it — then solved on from the
    /// recovery's start: kept where that settles (`SolveResult::settled`), else the system that
    /// was is put back, the sketch it stood on with it, and `res` stands.  A stop on the iteration
    /// limit is not kept even under the acceptance (#449): a recompile changes the yardstick, and
    /// a solve that ran a point off to infinity is compiled at the extent the runaway left —
    /// two equal circles' external tangents held to meet, every row then over a drawing ten
    /// kilometres across, read a contact 0.13 mm off its 20 mm circle as 3e-8 and the drawing as
    /// solved.  Compiled at its own size from an unseeded start, a drawing converges there.
    fn recompiled(
        &mut self,
        sk: &mut Sketch,
        opts: SolveOpts,
        res: SolveResult,
        again: Recovery,
    ) -> SolveResult {
        let compiled = std::mem::replace(self, System::new(sk));
        if let Some(from) = &again.from {
            sk.set_x(from);
        }
        let solved = self.solve_unsteered(sk, opts);
        if solved.settled() {
            return solved;
        }
        *self = compiled;
        match again.back {
            Back::Pose(x) => {
                sk.set_x(&x);
            }
            Back::Sketch(before) => *sk = *before,
        }
        self.resync(sk);
        res
    }

    /// **A drawing is solved at its rows' own sizes** (#492): a failed document solve whose
    /// every row holds over the drawing's extent, and not over its own size (`System::row_size`),
    /// stopped where the extent, stretched by a point far out, read the rows it left as solved —
    /// a DogLeg ending on its gradient after one step, `1e-16 / extent` with the extent at 1e6,
    /// its contacts on a 20 mm circle a micrometre off, or on its residual at once, `1e-14` of
    /// a squared extent.  It is solved on from where it stands with every row over its own size
    /// for the iteration as well (`System::scale_rows`), and kept where that settles nearer to
    /// holding than it was, no more than `RESIZE` times as wide as it began: a solve that runs a
    /// point off to infinity reads as holding there at any measure — two parallel tangents meet
    /// within 1e-14 of a 20 mm circle some 1e16 out.  Its rows are put back over the extent
    /// either way, which a rank is judged at.  A solve the extent's measure calls failed too,
    /// and every solve that succeeds, never sees it.
    fn at_own_sizes(&mut self, sk: &mut Sketch, opts: SolveOpts, res: SolveResult) -> SolveResult {
        let z = self.z0(sk);
        let r = self.residuals(&z);
        let rows = self.hard_rows();
        if !rows.iter().all(|&i| r[i].abs() < opts.acceptance_tol) {
            return res;
        }
        let (x, wide, compiled) = (sk.get_x(), RESIZE * sk.extent(), self.compiled_size.clone());
        // its rows' sizes where it stopped, which is where it is solved on from
        self.take_sizes();
        let own = rows.iter().map(|&i| self.judged(i, r[i])).fold(0.0, f64::max);
        self.scale_rows(true);
        let solved = self.solve_unsteered(sk, opts);
        if solved.settled() && solved.max_residual < own && sk.extent() <= wide {
            self.scale_rows(false);
            self.resync(sk);
            return solved;
        }
        sk.set_x(&x);
        self.compiled_size = compiled;
        self.scale_rows(false);
        self.resync(sk);
        self.size_rows();
        res
    }

    /// The system's own numbers back in step with the pose `sk` stands at.
    fn resync(&mut self, sk: &Sketch) {
        let z = self.z0(sk);
        let _ = self.residuals(&z);
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
        if opts.document() && reads_pose(sk) {
            for _ in 0..REFRAMES {
                if !res.success {
                    break;
                }
                self.refresh_consts(sk);
                let z = self.z0(sk);
                if self.max_relative_residual(&z) < opts.acceptance_tol {
                    break;
                }
                res = self.solve_rehomed(sk, opts);
            }
        }
        res
    }

    fn solve_rehomed(&mut self, sk: &mut Sketch, opts: SolveOpts) -> SolveResult {
        if !opts.acceptance_tol.is_finite() || opts.acceptance_tol <= 0. {
            let mut result = SolveResult::plain(opts.method.as_str(),false,f64::INFINITY,0);
            result.status = -1;
            result.message = "acceptance tolerance must be finite and positive".into();
            return result;
        }
        // a curve family's contact has no span to walk off, but a domain to be clamped to
        // and a fraction of an arc's turn, held to the arc (#248)
        let rehome = opts.rehome && opts.writeback
            && !(self.spans().is_empty() && sk.curves.is_empty() && !self.on_arcs());
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
        // a document holding a point to an envelope or a meshing is solved a block at a time
        // first: each contact is seeded where its sources stand once the blocks before it have
        // placed them (#304), which a whole-system DogLeg from the seeds never sees
        let first =
            opts.blocks == BlockMode::Rescue && opts.document() && blocks::holds_contacts(sk);
        let opts = if first {
            SolveOpts { blocks: BlockMode::First, ..opts }
        } else {
            opts
        };
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
        if opts.retry && dogleg {
            for retry in RETRIES {
                if !self.wants(retry, &res, &opts) {
                    continue;
                }
                in_step = match retry {
                    Retry::Rescue => self.block_rescue(sk, &z0, &mut z, &mut res, &mut rel, &opts),
                    Retry::DogLeg | Retry::Lm => {
                        let method = if retry == Retry::Lm { Method::Lm } else { Method::DogLeg };
                        let mut z2 = z0.clone();
                        let (res2, rel2) = self.minimise(&mut z2, method, &opts);
                        let kept = res2.success || rel2 < rel || rel.is_nan();
                        if kept {
                            (z, res, rel) = (z2, res2, rel2);
                        }
                        kept
                    }
                };
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

    /// Whether a DogLeg solve ending at `res` is tried again by `retry` (`RETRIES`).
    fn wants(&mut self, retry: Retry, res: &SolveResult, opts: &SolveOpts) -> bool {
        match retry {
            // with fewer than two blocks the pass would be this DogLeg again, from the same start
            Retry::Rescue => {
                opts.blocks == BlockMode::Rescue
                    && !res.settled()
                    && self.block_order().blocks.len() >= 2
            }
            Retry::DogLeg => opts.blocks == BlockMode::First && !res.success,
            Retry::Lm => !res.success,
        }
    }


    /// One minimisation from `z`, updated in place, measured on the hard rows: the result and
    /// the max residual relative to each row's own units, which is what "solved" means.
    /// Where a document's minimisation stops: `opts.tol` as stated, a step of 1e-12 of `z`, and
    /// a gradient in 1/length — the same for a solve through boundaries (`through`).
    pub(super) fn tolerances(&self, opts: &SolveOpts) -> newton::Tol {
        newton::Tol { ftol: opts.tol, xtol: 1e-12, gtol: 1e-16 / self.extent.max(1.0) }
    }

    fn minimise(&mut self, z: &mut [f64], method: Method, opts: &SolveOpts) -> (SolveResult, f64) {
        // Every residual the system hands out is over its row's own units (`System::row_scale`),
        // so `tol` is the tolerance as stated and the gradient's units are 1/length: a scaled
        // row's derivative with respect to a world-length column is O(1/extent) whatever the
        // row's degree, so stationarity is judged against that.
        let tol = self.tolerances(opts);
        let info: Info = newton::solve_system(
            self,
            method,
            tol.ftol,
            tol.xtol,
            tol.gtol,
            opts.max_iter,
            opts.max_nfev,
            opts.dense,
            opts.document(),
            z,
        );
        let r = self.residuals(z);
        self.size_rows();
        let mut n2 = 0.0;
        let mut rel = 0.0f64;
        // NaN is not "no error", and `f64::max` would drop it: track it and let it win at the end
        let mut nan = false;
        for i in 0..r.len() {
            n2 += r[i] * r[i];
            if self.hard[i] {
                // over the row's own size (#492): the iteration's `row_scale` is the drawing's
                // extent, which one far-out point stretches
                let a = self.judged(i, r[i]);
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
