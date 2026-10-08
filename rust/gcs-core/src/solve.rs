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
use crate::system::{Subset, System, DENSE_MAX};

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
            self.blocks_then_polish(&mut z, &opts)
        } else {
            self.minimise(&mut z, opts.method, &opts)
        };
        // the pose `self`'s x was last evaluated at is `z`'s, until a retry that is not kept
        let mut in_step = true;
        if opts.retry && dogleg && !res.settled() {
            // With fewer than two blocks the pass would be this DogLeg again, from the same start.
            if opts.blocks == BlockMode::Rescue && self.block_order().blocks.len() >= 2 {
                in_step = self.block_rescue(&z0, &mut z, &mut res, &mut rel, &opts);
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
            let (rb, relb) = self.blocks_then_polish(&mut zb, opts);
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
    /// which is the result.  A block is accepted when its rows meet `acceptance_tol`; the first
    /// that does not ends the pass, and the polish starts from the blocks solved so far (and
    /// whatever that one's own minimisation made of it).  With fewer than two blocks there is no
    /// order to exploit and this is the whole-system DogLeg alone.
    fn blocks_then_polish(&mut self, z: &mut [f64], opts: &SolveOpts) -> (SolveResult, f64) {
        let order = self.block_order();
        let (mut nfev, mut njev) = (0, 0);
        if order.blocks.len() >= 2 {
            let gtol = 1e-16 / self.extent.max(1.0);
            let tol = newton::Tol { ftol: opts.tol, xtol: BLOCK_XTOL, gtol };
            let max_nfev = if opts.max_nfev <= 0 { 4 * opts.max_iter } else { opts.max_nfev };
            for b in &order.blocks {
                let sub = self.subset(&b.instances, &b.cols);
                let mut zb: Vec<f64> = b.cols.iter().map(|&c| z[c]).collect();
                let mut t = BlockTr::new(self, sub, z);
                let mut r = vec![0.0; t.m()];
                t.residuals_into(&zb, &mut r);
                let info = newton::dogleg(&mut t, &mut zb, &mut r, tol, opts.max_iter, max_nfev);
                nfev += info.nfev;
                njev += info.njev;
                for (k, &c) in b.cols.iter().enumerate() {
                    z[c] = zb[k];
                }
                // `dogleg` keeps `r` the residuals at `zb`; NaN fails the comparison
                if !r.iter().all(|v| v.abs() < opts.acceptance_tol) {
                    break;
                }
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
