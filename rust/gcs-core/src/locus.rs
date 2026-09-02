//! A locus: a curve defined by the constraints that force it, rather than by expressions.
//!
//! `curve involute(c: circle, ...)(u) = trace p where { ... }` says what Wikipedia says about an
//! involute — "the curve traced by the end of a taut string as it unwinds" — and leaves working
//! out where that puts `p` to the solver.  The block's statements are ordinary constraints, so
//! `C(u)` has no formula: it is the position the block forces on the traced point, given `u` and
//! the geometry the family is written over, and evaluating it is a little Newton solve.
//!
//! The block is lowered once, at family compile time (`program::compile_trace`), to *rows*: the
//! same static kernels every other constraint runs on, each with a map from its columns into one
//! variable table — the outer variables `[u, θ…, values…]` the family's tapes read, then the
//! inner unknowns `q` (the block's points), then the derived values `w` (a dimension written as
//! an expression over `u` and the geometry, computed by a tape and read by the dimension's free
//! twin kernel, so `(m, c)` absorb the units and no new derivative code exists anywhere).
//!
//! Derivatives are the implicit function theorem on the same rows: with `r(q; u, θ, w(u, θ)) = 0`
//! at the solution, one factorisation of `∂r/∂q` answers `∂C/∂u` and `∂C/∂θ` together — which is
//! exactly the pair a contact kernel needs (spec §6.5), and why a point solved onto a trace curve
//! follows it when the geometry it is written over is dragged.
//!
//! Like a tape, a locus encodes to a flat `Vec<f64>` and rides in the contact's constants, so no
//! kernel signature learns about curves — and `eval_at` here is the one evaluator: the kernel,
//! the tessellation and the tests all run the flat form, so there is no second walker to drift.
//! (`eval_flat` is its cold entry, the same walk named a contact it may carry a pose for.)
//!
//! **Branches are picked by seeds and carried by continuity.**  A taut string unwinds clockwise
//! or counter-clockwise, and no regular residual can say which — chirality is discrete.  This
//! solver's answer to branches everywhere else is a seed and a warm start, and it is the answer
//! here: the block's `at (…)` seeds are expressions over `(u, θ)`, evaluation starts from them at
//! the target parameter, and when that solve fails it marches from the low end of the domain,
//! each step warm-started from the last, so the branch chosen at the start is the branch the
//! whole curve is on.
//!
//! **And a branch once carried is kept.**  The outer solver moves `(u, θ)` a little and asks the
//! same contact again, which is a continuation step like any other — so each contact's pose is
//! remembered (`Seen`) and the next evaluation *resumes* from it rather than re-walking the
//! march from the home.  Replaying instead cost the traced gear thirty-four block solves for
//! every one it needed.  A resumed step is only trusted as far as it can be checked, and
//! `continues` checks it against the tangent that predicted it; what fails falls back to the
//! home and the full march, so the doctrine above is what decides every branch either way.
//!
//! So the kernel path carries a *history* where the drawing path (`sweep`, and `curve::closest`
//! through it) is always cold.  That is the intended reading and not an oversight: the drawn
//! curve is what a march from the home makes of `(u, θ)`, a contact is a point that got to its
//! `u` by a road, and the two agree because every step of that road was checked against the
//! curve's own tangent.  A change that lets them disagree is a change to `continues`.

use crate::kernels::KERNELS;
use crate::tape::{self, Tape};

/// Bounds on what a document may ask for.  A document is untrusted input and
/// `wasm32-unknown-unknown` aborts rather than unwinding, so the flat form is range-checked as
/// it is read and these keep a hostile encoding from asking for unbounded work.
pub const MAX_Q: usize = 32;
pub const MAX_ROWS: usize = 64;
pub const MAX_W: usize = 16;
pub const MAX_PREDS: usize = 8;

/// Continuation steps when the answer is carried rather than found in place: the parameter walks
/// from the home to the target, each step seeded with the last step's answer.
const MARCH: usize = 32;
const NEWTON_MAX: usize = 30;
/// Deterministic retries when the seeds are no use — a block with no seeds starts every point at
/// the origin, which for a point on a circle is the one singular spot.
const RESTARTS: usize = 8;
/// Rounds of predicate enforcement at the home: reflect, re-solve, re-read.
const PRED_ROUNDS: usize = 4;

/* -- the compiled form -------------------------------------------------------------- */

/// One residual row: a static kernel, the variable-table slot each of its columns reads, and its
/// constants.  `kid` is the *free twin* for a dimension whose value is a `w` — the twin reads the
/// value as its last column, which is what lets `∂r/∂w` come from the kernel like every other
/// derivative.
#[derive(Clone, Debug)]
pub struct Row {
    pub kid: usize,
    pub cols: Vec<u32>,
    pub consts: Vec<f64>,
}

/// An orientation predicate — `ccw(a, b, x)` in the block.  It contributes no residual: it
/// *selects among the discrete solution components* (spec §9.6), which is exactly what a branch
/// is.  Enforced at the home solve only — reflect the placed point across the oriented line and
/// solve again — and carried everywhere else by continuity, since along a march the components
/// never meet.  Six columns: a, b, then the point the block places.
#[derive(Clone, Debug)]
pub struct Pred {
    pub ccw: bool,
    pub cols: [u32; 6],
}

/// A compiled trace block, held only as its encoding — what rides in a contact's constants and
/// what `eval_flat` runs.  Nothing ever reads the structured pieces back after `new`, so keeping
/// them would be a second representation waiting to drift.  The variable table the rows index is
/// `[u, θ…, values…] ++ q ++ w`: the outer variables in the family's tape order (so `n_outer`
/// is `CurveDef::vars.len()`), then the inner unknowns, then the derived values.
#[derive(Clone, Debug)]
pub struct Locus {
    pub flat: Vec<f64>,
}

impl Locus {
    /// How many inner unknowns the block has — the width of an anchor pose.  Read through the
    /// one decoder of the encoding, so a malformed flat is 0 and never a garbage width.
    pub fn n_q(&self) -> usize {
        view(&self.flat).map_or(0, |v| v.n_q)
    }

    /// Validate and encode.  An `Err` is a diagnostic for the family, not a panic.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        n_outer: usize,
        n_theta: usize,
        n_q: usize,
        traced: usize,
        w: Vec<Tape>,
        seeds: Vec<Tape>,
        rows: Vec<Row>,
        preds: Vec<Pred>,
    ) -> Result<Locus, String> {
        if n_q == 0 || n_q > MAX_Q {
            return Err(format!("a trace block declares between 1 and {MAX_Q} unknowns"));
        }
        if rows.len() > MAX_ROWS || w.len() > MAX_W || preds.len() > MAX_PREDS {
            return Err(format!(
                "a trace block may state at most {MAX_ROWS} constraints, {MAX_W} expressions \
                 and {MAX_PREDS} orientations"
            ));
        }
        if seeds.len() != n_q || traced + 2 > n_q {
            return Err("trace block shape mismatch".to_string());
        }
        let n_res: usize = rows.iter().map(|r| KERNELS[r.kid].n_res).sum();
        if n_res != n_q {
            return Err(format!(
                "a trace block must determine its points: {n_q} unknowns against {n_res} equations"
            ));
        }
        for r in &rows {
            let kn = &KERNELS[r.kid];
            if r.cols.len() != kn.n_par || r.consts.len() != kn.n_const {
                return Err("trace row shape mismatch".to_string());
            }
        }
        for p in &preds {
            if p.cols.iter().any(|&c| c as usize >= n_outer + n_q) {
                return Err("an orientation names a column the block does not have".to_string());
            }
        }
        // the instructions as plain numbers — see the module docs and `Tape::flat`
        let mut f = vec![
            n_outer as f64,
            n_theta as f64,
            n_q as f64,
            w.len() as f64,
            rows.len() as f64,
            traced as f64,
            preds.len() as f64,
        ];
        for t in &w {
            f.push(t.flat.len() as f64);
            f.extend_from_slice(&t.flat);
        }
        for r in &rows {
            f.push(r.kid as f64);
            f.push(r.cols.len() as f64);
            f.push(r.consts.len() as f64);
            f.extend(r.cols.iter().map(|&c| c as f64));
            f.extend_from_slice(&r.consts);
        }
        for t in &seeds {
            f.push(t.flat.len() as f64);
            f.extend_from_slice(&t.flat);
        }
        for p in &preds {
            f.push(if p.ccw { 1.0 } else { 0.0 });
            f.extend(p.cols.iter().map(|&c| c as f64));
        }
        Ok(Locus { flat: f })
    }
}

/* -- evaluation --------------------------------------------------------------------- */

/// What one evaluation came to: the traced point and its derivatives in `[u, θ…]` — the same
/// width and order as a tape's gradient, so the contact kernel fills its row the same way for
/// either kind of family.  `ok` is false when the inner solve did not converge; the position is
/// then the best found, and the residual the outer solver sees says the rest.  A flat form that
/// could not be read at all comes back NaN — a residual through it must never read as satisfied,
/// and `System` already treats NaN as "not converged", never as "no error".
#[derive(Clone, Copy, Debug)]
pub struct Val {
    pub x: f64,
    pub y: f64,
    pub dx: [f64; tape::MAX_VARS],
    pub dy: [f64; tape::MAX_VARS],
    pub ok: bool,
}

impl Default for Val {
    fn default() -> Val {
        Val {
            x: f64::NAN,
            y: f64::NAN,
            dx: [0.0; tape::MAX_VARS],
            dy: [0.0; tape::MAX_VARS],
            ok: false,
        }
    }
}

/// Where one contact's evaluation last got to: the outer vector it was asked at, what it came
/// to, and the inner pose that answered it.  Both of its uses are the same fact read twice —
/// asked again at the *same* outer it is the answer (`System` asks for the Jacobian at the point
/// it just asked the residual of), and asked at a *neighbouring* one it is where the
/// continuation had reached, which is what `eval_at` carries forward.
#[derive(Default)]
struct Seen {
    outer: Vec<f64>,
    val: Val,
    q: Vec<f64>,
}

/// How many contacts' poses are remembered at once.  A cache, not a table: past it they all go,
/// so a document with thousands of contacts costs bounded memory and merely loses the warm
/// start — never an answer, since a forgotten pose is one the march works out again.
const SEEN_MAX: usize = 4096;

/// Scratch an evaluation runs in — held by the caller and reused, like `tape::Scratch`, because
/// a kernel evaluates the same locus for every constraint in its block.  It also carries the
/// kernel path's memory of where each contact last was (`Seen`), keyed by the address of the
/// contact's own constants.
pub struct Scratch {
    ts: tape::Scratch,
    xv: Vec<f64>,
    r: Vec<f64>,
    jq: Vec<f64>,
    b: Vec<f64>,
    lu: Vec<f64>,
    piv: Vec<usize>,
    rhs: Vec<f64>,
    q: Vec<f64>,
    wd: Vec<[f64; tape::MAX_VARS]>,
    v: Vec<f64>,
    jrow: Vec<f64>,
    seen: std::collections::BTreeMap<(usize, usize), Seen>,
}

impl Scratch {
    pub fn new() -> Scratch {
        Scratch {
            ts: tape::Scratch::new(),
            xv: Vec::new(),
            r: Vec::new(),
            jq: Vec::new(),
            b: Vec::new(),
            lu: Vec::new(),
            piv: Vec::new(),
            rhs: Vec::new(),
            q: Vec::new(),
            wd: Vec::new(),
            v: Vec::new(),
            jrow: Vec::new(),
            seen: std::collections::BTreeMap::new(),
        }
    }
}

impl Default for Scratch {
    fn default() -> Scratch {
        Scratch::new()
    }
}

/// The flat form, decoded to borrowed slices.  Every length and column index is checked on the
/// way in, because on `wasm32-unknown-unknown` an out-of-range index aborts.  The embedded tape
/// slices are trusted the way `Tape::flat` is everywhere else: a locus flat is only ever
/// produced by `Locus::encode` — a document carries the *source* of a trace block, never its
/// encoding — so a malformed one is a bug here, not input.
struct View<'a> {
    n_outer: usize,
    n_theta: usize,
    n_q: usize,
    traced: usize,
    w: Vec<&'a [f64]>,
    rows: Vec<(usize, &'a [f64], &'a [f64])>,
    seeds: Vec<&'a [f64]>,
    preds: Vec<(bool, [usize; 6])>,
}

/// Decode the flat form.  Trailing numbers past the encoding are ignored: a contact's constants
/// carry the anchor pose after the flat (`kernel_eval`), and the flat says where it ends.
fn view(flat: &[f64]) -> Option<View<'_>> {
    let g = |i: usize| flat.get(i).copied().filter(|v| v.is_finite() && *v >= 0.0);
    let n_outer = g(0)? as usize;
    let n_theta = g(1)? as usize;
    let n_q = g(2)? as usize;
    let n_w = g(3)? as usize;
    let n_rows = g(4)? as usize;
    let traced = g(5)? as usize;
    let n_preds = g(6)? as usize;
    if n_outer == 0
        || n_outer > tape::MAX_VARS
        || 1 + n_theta > n_outer
        || n_q == 0
        || n_q > MAX_Q
        || n_w > MAX_W
        || n_rows > MAX_ROWS
        || n_preds > MAX_PREDS
        || traced + 2 > n_q
    {
        return None;
    }
    let mut at = 7usize;
    // a length is bounded by the whole encoding before it indexes anything, so a corrupt one
    // is a `None` and never an overflow
    let take = |at: &mut usize, len: usize| -> Option<&[f64]> {
        if len > flat.len() {
            return None;
        }
        let s = flat.get(*at..at.checked_add(len)?)?;
        *at += len;
        Some(s)
    };
    let mut w = Vec::with_capacity(n_w);
    for _ in 0..n_w {
        let len = g(at)? as usize;
        at += 1;
        w.push(take(&mut at, len)?);
    }
    let mut rows = Vec::with_capacity(n_rows);
    let width = n_outer + n_q + n_w;
    for _ in 0..n_rows {
        let kid = g(at)? as usize;
        let n_par = g(at + 1)? as usize;
        let n_const = g(at + 2)? as usize;
        at += 3;
        let kn = KERNELS.get(kid)?;
        if n_par != kn.n_par || n_const != kn.n_const {
            return None;
        }
        let cols = take(&mut at, n_par)?;
        if cols.iter().any(|&c| !(c >= 0.0 && (c as usize) < width)) {
            return None;
        }
        let consts = take(&mut at, n_const)?;
        rows.push((kid, cols, consts));
    }
    let mut seeds = Vec::with_capacity(n_q);
    for _ in 0..n_q {
        let len = g(at)? as usize;
        at += 1;
        seeds.push(take(&mut at, len)?);
    }
    let mut preds = Vec::with_capacity(n_preds);
    for _ in 0..n_preds {
        let ccw = g(at)? != 0.0;
        at += 1;
        let mut cols = [0usize; 6];
        for c in cols.iter_mut() {
            let s = g(at)? as usize;
            if s >= n_outer + n_q {
                return None;
            }
            *c = s;
            at += 1;
        }
        preds.push((ccw, cols));
    }
    Some(View { n_outer, n_theta, n_q, traced, w, rows, seeds, preds })
}

/// Where an evaluation is anchored: the parameter value the block's predicates are read at,
/// and — for a curve of a drawn instance — the pose on the sheet the anchor solve starts from.
/// `None` for the pose leaves the seeds to start it.
#[derive(Clone, Copy, Debug)]
pub struct Anchor<'a> {
    pub u: f64,
    pub pose: Option<&'a [f64]>,
}

/// Below this relative residual the block is solved outright.
const TOL_DONE: f64 = 1e-12;
/// A solve that ran out of downhill or of step is still *accepted* under this — comfortably
/// inside the outer system's own 1e-6, so a contact never reads as satisfied off the back of a
/// sloppy inner solve.
const TOL_OK: f64 = 1e-8;
/// A stall above this is a failure; at or below it the point is as solved as it need be.
const TOL_STALL: f64 = 1e-10;
/// A step smaller than this fraction of the pose is no step.
const STEP_MIN: f64 = 1e-14;
/// A resumed solve's correction against the tangent that predicted it.  Below this fraction of
/// the predicted move the step is the continuation of the pose it started from; above it, the
/// step is something else and the answer is thrown away for the march.
const PREDICTED: f64 = 0.5;

/// One Newton solve of the block at the parameter already written into `s.xv[0]`, from the `q`
/// already in `s.xv`.  Residual rows are judged against their own kernel's power of length over
/// the magnitude of what they read — the same reasoning as `System::max_relative_residual`, in
/// miniature — so one tolerance serves a block at any size.
///
/// Deliberately not `newton::dogleg`: the block is square, so the plain Newton step *is* the
/// Gauss–Newton step; this runs per residual evaluation inside a kernel, where dogleg's
/// per-call allocations are the thing the scratch exists to avoid; and the convergence test is
/// unit-relative where dogleg's is absolute.  A second inner square solve is the point at which
/// this loop moves to `newton.rs` rather than being copied.
fn newton(v: &View, s: &mut Scratch) -> bool {
    let (n_q, q0) = (v.n_q, v.n_outer);
    let mut last = f64::INFINITY;
    for _ in 0..NEWTON_MAX {
        // the one full assembly per iteration; the trial steps below need only residuals
        let norm = assemble(v, s, Fill::Jac);
        if !norm.is_finite() {
            return false;
        }
        if norm <= TOL_DONE {
            return true;
        }
        // solve Jq Δ = -r
        s.lu.clear();
        s.lu.extend_from_slice(&s.jq);
        s.rhs.clear();
        s.rhs.extend(s.r.iter().map(|&x| -x));
        if !crate::linalg::lu_factor(n_q, &mut s.lu, &mut s.piv) {
            return false;
        }
        crate::linalg::lu_apply(n_q, &s.lu, &s.piv, &mut s.rhs);
        // backtrack when a full step overshoots — the block is tiny, so a residual evaluation
        // costs less than a wasted outer iteration
        s.q.clear();
        s.q.extend_from_slice(&s.xv[q0..q0 + n_q]);
        let mut step = 1.0;
        let mut best = f64::INFINITY;
        for _ in 0..8 {
            for i in 0..n_q {
                s.xv[q0 + i] = s.q[i] + step * s.rhs[i];
            }
            let nn = assemble(v, s, Fill::Res);
            if nn.is_finite() && nn < norm {
                best = nn;
                break;
            }
            step *= 0.5;
        }
        if best.is_infinite() {
            // no step down from here: converged as far as it goes
            for i in 0..n_q {
                s.xv[q0 + i] = s.q[i];
            }
            return norm <= TOL_OK;
        }
        let dq: f64 = s.rhs.iter().map(|d| d * d * step * step).sum::<f64>().sqrt();
        let qn: f64 = s.q.iter().map(|x| x * x).sum::<f64>().sqrt();
        if dq <= STEP_MIN * (1.0 + qn) {
            return best <= TOL_OK;
        }
        if best >= last * 0.999999 && best > TOL_STALL {
            // stalled without converging
            return false;
        }
        last = best;
    }
    last <= TOL_STALL
}

/// How much of the block `assemble` is asked for: residuals alone (a trial step needs only the
/// norm), the Jacobian in `q` too (a Newton iteration), or additionally `b` — the columns for
/// `[u, θ…]`, with a derived value's contribution chained through its tape (the implicit
/// function theorem at the end).
#[derive(Clone, Copy, PartialEq, PartialOrd)]
enum Fill {
    Res,
    Jac,
    JacB,
}

/// Residuals — and per `fill`, the Jacobian — of the block at `s.xv`.  Returns the worst
/// residual over its own row's units, which is dimensionless, so one tolerance judges a block
/// at any size.
fn assemble(v: &View, s: &mut Scratch, fill: Fill) -> f64 {
    let n_q = v.n_q;
    let n_dc = 1 + v.n_theta;
    let (q0, w0) = (v.n_outer, v.n_outer + n_q);
    s.r.clear();
    s.r.resize(n_q, 0.0);
    if fill >= Fill::Jac {
        s.jq.clear();
        s.jq.resize(n_q * n_q, 0.0);
    }
    if fill == Fill::JacB {
        s.b.clear();
        s.b.resize(n_q * n_dc, 0.0);
    }
    let mut row0 = 0usize;
    let mut worst = 0.0f64;
    for &(kid, cols, consts) in &v.rows {
        let kn = &KERNELS[kid];
        s.v.clear();
        let mut mag = 1.0f64;
        for &c in cols {
            let x = s.xv[c as usize];
            // the units a row is judged in come from the *lengths* it reads — a derived value
            // may be an angle in degrees, and letting one into the magnitude would loosen the
            // row's tolerance by the square of a number that is not a length
            if (c as usize) < w0 {
                mag = mag.max(x.abs());
            }
            s.v.push(x);
        }
        if row0 + kn.n_res > n_q {
            return f64::NAN;
        }
        (kn.res)(1, &s.v, consts, &mut s.r[row0..row0 + kn.n_res]);
        let unit = mag.powi(kn.degree as i32);
        for t in 0..kn.n_res {
            worst = worst.max(s.r[row0 + t].abs() / unit);
        }
        if fill >= Fill::Jac {
            s.jrow.clear();
            s.jrow.resize(kn.n_res * kn.n_par, 0.0);
            if let Some(cj) = kn.const_jac {
                s.jrow.copy_from_slice(cj);
            } else {
                (kn.jac)(1, &s.v, consts, &mut s.jrow);
            }
            for t in 0..kn.n_res {
                for (c, &col) in cols.iter().enumerate() {
                    let g = s.jrow[t * kn.n_par + c];
                    if g == 0.0 {
                        continue;
                    }
                    let col = col as usize;
                    if col >= w0 {
                        if fill == Fill::JacB {
                            let wd = &s.wd[col - w0];
                            for d in 0..n_dc {
                                s.b[(row0 + t) * n_dc + d] += g * wd[d];
                            }
                        }
                    } else if col >= q0 {
                        s.jq[(row0 + t) * n_q + (col - q0)] += g;
                    } else if fill == Fill::JacB && col < n_dc {
                        s.b[(row0 + t) * n_dc + col] += g;
                    }
                    // an outer column past the θ block is one of the instance's given numbers:
                    // a constant of this curve, whose gradient nobody is owed
                }
            }
        }
        row0 += kn.n_res;
    }
    if row0 != n_q {
        return f64::NAN;
    }
    worst
}

/// Write `[u, θ…, values…]` and the derived values into `s.xv`, leaving `q` alone.
fn refresh(v: &View, s: &mut Scratch, u: f64, outer: &[f64]) {
    s.xv[0] = u;
    s.xv[1..v.n_outer].copy_from_slice(&outer[1..v.n_outer]);
    let n_dc = 1 + v.n_theta;
    for (j, t) in v.w.iter().enumerate() {
        let val = tape::eval_flat(t, v.n_outer, &s.xv[..v.n_outer], &mut s.ts);
        s.xv[v.n_outer + v.n_q + j] = val.v;
        let mut d = [0.0; tape::MAX_VARS];
        d[..n_dc].copy_from_slice(&val.d[..n_dc]);
        s.wd[j] = d;
    }
}

/// Seed every `q` from its tape at the parameter already in `s.xv[0]`.
fn seed(v: &View, s: &mut Scratch) {
    for (i, t) in v.seeds.iter().enumerate() {
        s.xv[v.n_outer + i] = tape::eval_flat(t, v.n_outer, &s.xv[..v.n_outer], &mut s.ts).v;
    }
}

/// Decode the flat form and size the scratch for it — the shared preamble of `eval_flat` and
/// `sweep`, so the two cannot disagree about what a well-formed encoding is.
fn prepare<'a>(flat: &'a [f64], outer: &[f64], s: &mut Scratch) -> Option<View<'a>> {
    let v = view(flat)?;
    if outer.len() < v.n_outer {
        return None;
    }
    s.xv.clear();
    s.xv.resize(v.n_outer + v.n_q + v.w.len(), 0.0);
    s.wd.clear();
    s.wd.resize(v.w.len(), [0.0; tape::MAX_VARS]);
    Some(v)
}

/// Walk the parameter from `from` to `to` in `MARCH` steps, each warm-started from the last —
/// the continuation that carries a branch.  `keep_going` is the sweep's policy: a failed step
/// keeps the last answer as its seed and carries on, so a genuinely impossible block draws a
/// stationary run rather than nothing; evaluation stops instead, and the residual says the rest.
fn march(v: &View, s: &mut Scratch, outer: &[f64], from: f64, to: f64, keep_going: bool) -> bool {
    for k in 1..=MARCH {
        let uk = from + (to - from) * k as f64 / MARCH as f64;
        refresh(v, s, uk, outer);
        if !newton(v, s) && !keep_going {
            return false;
        }
    }
    true
}

/// Evaluate the locus at `outer = [u, θ…, values…]`: solve the block, then the implicit function
/// theorem for the derivatives.  A block without predicates tries the seeds at the target
/// parameter first, and marches from the home when that fails; one *with* predicates marches —
/// they are read at the home and carried by continuity, and a direct solve *from the seeds* at
/// the target could land in a component they forbid.
///
/// This is the cold form, which starts from the home however often it is called.  A caller that
/// names a contact (`eval_at`) gets the same walk with a branch it already holds carried into it.
pub fn eval_flat(flat: &[f64], outer: &[f64], anchor: Anchor, s: &mut Scratch) -> Val {
    eval_at(flat, outer, anchor, None, s)
}

/// `key` is where this contact's constants live: it both *resumes* from the pose remembered
/// there and remembers the pose this evaluation ends on.  `None` is the cold form.
///
/// **Why resuming is the same statement about branches.**  The march exists because the branch
/// is chosen at the home and carried by continuity (spec §6.5.1), and re-walking it from the
/// home is one way to carry it — but the outer solver moves `(u, θ)` by a little and asks again,
/// and *that* is a continuation step too.  Resuming honours the doctrine and replaying merely
/// pays for it again: 1056 evaluations of the traced gear cost 35904 block solves, one per march
/// step, for answers a hair from the ones beside them.  What it must not do is take the *hair*
/// on trust, so `continues` checks the step against the tangent it was predicted by, and
/// anything that fails falls back to the home and the full march below.
fn eval_at(
    flat: &[f64],
    outer: &[f64],
    anchor: Anchor,
    key: Option<(usize, usize)>,
    s: &mut Scratch,
) -> Val {
    let Some(v) = prepare(flat, outer, s) else { return Val::default() };
    let u = outer[0];
    // taken out of the map rather than borrowed from it: the solve below wants the scratch the
    // pose lives in.  It goes back in `keep`, whatever this evaluation makes of it.
    let from = key.and_then(|k| s.seen.remove(&k)).filter(|p| p.val.ok);
    if let Some(prev) = &from {
        // read where it landed before paying `finish` for it: a rejected resume owes no
        // factorisation, and the traced point is all `continues` ever asks about
        if warm(&v, s, u, outer, &prev.q) && continues(&v, s, prev) {
            let val = finish(&v, s, true);
            if val.ok {
                return keep(&v, s, key, outer, val);
            }
        }
    }
    if v.preds.is_empty() {
        refresh(&v, s, u, outer);
        seed(&v, s);
        if newton(&v, s) {
            let val = finish(&v, s, true);
            return keep(&v, s, key, outer, val);
        }
    }
    let mut ok = cold_start(&v, s, outer, anchor);
    if ok && u != anchor.u {
        ok = march(&v, s, outer, anchor.u, u, false);
    }
    let val = finish(&v, s, ok);
    keep(&v, s, key, outer, val)
}

/// Remember the pose this evaluation ended on, under the contact it belongs to.
fn keep(v: &View, s: &mut Scratch, key: Option<(usize, usize)>, outer: &[f64], val: Val) -> Val {
    let Some(k) = key else { return val };
    // a cache and not a table: past the bound every entry goes, since a forgotten pose is one
    // the march works out again.  `k` was removed on the way in, so it is not among them.
    if s.seen.len() >= SEEN_MAX {
        s.seen.clear();
    }
    let q0 = v.n_outer;
    let e = s.seen.entry(k).or_default();
    e.outer.clear();
    e.outer.extend_from_slice(outer);
    e.q.clear();
    e.q.extend_from_slice(&s.xv[q0..q0 + v.n_q]);
    e.val = val;
    val
}

/// Whether the pose just found *continues* the one resumed from, or is a different branch Newton
/// happened to fall into.
///
/// The old pose carries its own derivatives — `∂C/∂(u, θ)`, which its own `finish` already paid a
/// factorisation for — so where the point should have gone is a tangent step away, and the only
/// question is how much Newton had to correct it.  On the same branch the correction is second
/// order in the step and vanishes beside it; onto another branch it is the distance between the
/// branches, which no step made it small.  So the test is the two against each other and needs
/// no length of its own — the one absolute term is the inner solve's own accuracy, below which
/// nothing about a pose is knowable anyway.
fn continues(v: &View, s: &Scratch, prev: &Seen) -> bool {
    let q0 = v.n_outer;
    let (x, y) = (s.xv[q0 + v.traced], s.xv[q0 + v.traced + 1]);
    // the tangent step and the correction to it, both as displacements from the old pose: the
    // point itself is far from the origin and the two quantities compared here are not
    let (mut dx, mut dy) = (0.0f64, 0.0f64);
    for d in 0..1 + v.n_theta {
        let step = s.xv[d] - prev.outer[d];
        dx += prev.val.dx[d] * step;
        dy += prev.val.dy[d] * step;
    }
    let corrected = (x - prev.val.x - dx).hypot(y - prev.val.y - dy);
    let scale = 1.0 + prev.val.x.abs().max(prev.val.y.abs());
    corrected <= PREDICTED * dx.hypot(dy) + TOL_OK * scale
}

/// The solve everything else is carried from: the **pose** a drawn instance stands at when
/// there is one, else the seeds (or restarts) at the home, then the predicates.  A pose is the
/// strongest start there is — the sheet already holds the assembly the predicates name, at the
/// very parameter the home is — so it goes first and the seeds are the fallback.  The restarts
/// are drawn from a *fixed* seed, so every evaluation tries the same starts and the answer cannot
/// depend on history — and each violated predicate reflects its point across the oriented line
/// and solves again, which is the move between the two components with Newton finishing it.
fn cold_start(v: &View, s: &mut Scratch, outer: &[f64], anchor: Anchor) -> bool {
    refresh(v, s, anchor.u, outer);
    let mut ok = false;
    if let Some(p) = anchor.pose.filter(|p| p.len() == v.n_q) {
        s.xv[v.n_outer..v.n_outer + v.n_q].copy_from_slice(p);
        ok = newton(v, s);
    }
    if !ok {
        seed(v, s);
        ok = newton(v, s);
    }
    if !ok {
        let mut rng = crate::rng::Rng::new(0x7ace);
        // the geometry's scale — the entity formals' coordinates and where the seeds put the
        // block's own points, and nothing else.  Not the parameter, which is the value asked
        // for (a restart that read it made the anchor solve a lottery in `u`), and not the
        // values, which may be angles: a tooth's `phase` of 129° scattered the restarts across
        // eight times the base circle's radius, and the string never found the circle again.
        // The seeds count so that a component written over a point at the origin is not
        // restarted within a unit of it whatever size its linkage is
        let scale = outer[1..1 + v.n_theta]
            .iter()
            .chain(&s.xv[v.n_outer..v.n_outer + v.n_q])
            .fold(1.0f64, |m, &x| m.max(x.abs()));
        for _ in 0..RESTARTS {
            seed(v, s);
            for i in 0..v.n_q {
                s.xv[v.n_outer + i] += rng.uniform(-scale, scale);
            }
            ok = newton(v, s);
            if ok {
                break;
            }
        }
    }
    if !ok {
        return false;
    }
    for _ in 0..PRED_ROUNDS {
        let Some(&(_, cols)) =
            v.preds.iter().find(|&&(ccw, cols)| !holds(s, ccw, cols))
        else {
            return true;
        };
        reflect(s, cols);
        if !newton(v, s) {
            return false;
        }
    }
    v.preds.iter().all(|&(ccw, cols)| holds(s, ccw, cols))
}

/// Whether `ccw(a, b, x)` holds at the current solution.  Collinear passes either way: a
/// predicate read exactly on its boundary has nothing to say.
fn holds(s: &Scratch, ccw: bool, cols: [usize; 6]) -> bool {
    let g = |i: usize| s.xv[cols[i]];
    let cross = crate::model::orientation_xy(g(0), g(1), g(2), g(3), g(4), g(5));
    if ccw {
        cross >= 0.0
    } else {
        cross <= 0.0
    }
}

/// Reflect the predicate's placed point across the oriented line through its first two.
fn reflect(s: &mut Scratch, cols: [usize; 6]) {
    let (ax, ay) = (s.xv[cols[0]], s.xv[cols[1]]);
    let (dx, dy) = (s.xv[cols[2]] - ax, s.xv[cols[3]] - ay);
    let l2 = dx * dx + dy * dy;
    if l2 <= 1e-30 {
        return;
    }
    let (wx, wy) = (s.xv[cols[4]] - ax, s.xv[cols[5]] - ay);
    let t = (wx * dx + wy * dy) / l2;
    let (fx, fy) = (ax + t * dx, ay + t * dy);
    s.xv[cols[4]] = 2.0 * fx - s.xv[cols[4]];
    s.xv[cols[5]] = 2.0 * fy - s.xv[cols[5]];
}

/// The implicit function theorem at the solution in `s.xv`: `Jq · S = −B`, and the traced
/// point's rows of `S` are the derivatives the contact kernel wants.
fn finish(v: &View, s: &mut Scratch, ok: bool) -> Val {
    let n_q = v.n_q;
    let n_dc = 1 + v.n_theta;
    let q0 = v.n_outer;
    let mut out = Val {
        x: s.xv[q0 + v.traced],
        y: s.xv[q0 + v.traced + 1],
        dx: [0.0; tape::MAX_VARS],
        dy: [0.0; tape::MAX_VARS],
        ok,
    };
    if !ok {
        return out;
    }
    if !assemble(v, s, Fill::JacB).is_finite() {
        out.ok = false;
        return out;
    }
    // one factorisation for every derivative direction
    s.lu.clear();
    s.lu.extend_from_slice(&s.jq);
    if !crate::linalg::lu_factor(n_q, &mut s.lu, &mut s.piv) {
        out.ok = false;
        return out;
    }
    for d in 0..n_dc {
        s.rhs.clear();
        s.rhs.extend((0..n_q).map(|i| -s.b[i * n_dc + d]));
        crate::linalg::lu_apply(n_q, &s.lu, &s.piv, &mut s.rhs);
        out.dx[d] = s.rhs[v.traced];
        out.dy[d] = s.rhs[v.traced + 1];
    }
    out
}

/// The curve as a polyline: one march across `[u0, u1]`, each sample warm-started from the last.
/// What `Sketch::curve_polyline` draws and the pick test measures for a trace family.
///
/// The samples are walked **outward from the anchor** — down to `u0`, then, from the anchor's
/// pose again, up to `u1` — so every solve is a sample and the branch is still carried by one
/// continuation from the pose.  Marching from the anchor to `u0` first and then sampling paid
/// for the stretch between them twice, on every repaint of a curve of a drawn instance, whose
/// anchor is wherever the crank stands.  An anchor outside the interval is marched to the
/// nearer end, once, and the samples walked across from there.
pub fn sweep(
    flat: &[f64],
    outer: &[f64],
    u0: f64,
    u1: f64,
    n: usize,
    anchor: Anchor,
    s: &mut Scratch,
) -> Vec<(f64, f64)> {
    let Some(v) = prepare(flat, outer, s) else { return Vec::new() };
    if n == 0 {
        return Vec::new();
    }
    let q0 = v.n_outer;
    let at = |k: usize| u0 + (u1 - u0) * k as f64 / n as f64;
    let sample = |s: &mut Scratch, u: f64| {
        refresh(&v, s, u, outer);
        let _ = newton(&v, s);
        (s.xv[q0 + v.traced], s.xv[q0 + v.traced + 1])
    };
    let _ = cold_start(&v, s, outer, anchor);
    let mut out = vec![(0.0, 0.0); n + 1];
    // the samples on the anchor's near side of u0 (none when the anchor is past u1, all when
    // it is past u0), walked toward u0; the rest walked toward u1 from the anchor's pose again
    let toward_u0 = (0..=n).filter(|&k| (at(k) - anchor.u) * (u1 - u0) <= 0.0).count();
    if toward_u0 == 0 {
        let _ = march(&v, s, outer, anchor.u, u1, true);
    } else if toward_u0 == n + 1 {
        let _ = march(&v, s, outer, anchor.u, u0, true);
    }
    let pose: Vec<f64> = s.xv[q0..q0 + v.n_q].to_vec();
    for k in (0..toward_u0).rev() {
        out[k] = sample(s, at(k));
    }
    s.xv[q0..q0 + v.n_q].copy_from_slice(&pose);
    for k in toward_u0..=n {
        out[k] = sample(s, at(k));
    }
    out
}

/// The two derivative-carrying evaluations a contact kernel makes, bundled: `kernels` calls this
/// so the residual and the Jacobian read one code path.  What each contact last came to sits in
/// front of the solve, because `System` asks for the Jacobian at the point it just asked the
/// residual of: the answer is a pure function of the constants and the columns past the contact
/// point, so remembering it is deterministic and halves the block solves.
///
/// It is remembered **per contact** and not once for the whole path.  A block evaluates every
/// contact of its kernel before the Jacobian pass begins (`point_on_trace_res` walks all `n`,
/// then `point_on_trace_jac` walks all `n` again), so a single slot holds the *last* contact when
/// the Jacobian asks for the *first* and never once hits — the halving it was written for only
/// ever happened for a block of one.
pub fn kernel_eval(consts: &[f64], v: &[f64], n_par: usize) -> Val {
    // the point-on-curve columns: the point, the parameter, then the θ columns
    let theta = n_par.saturating_sub(3);
    if v.len() < 3 + theta {
        return Val::default();
    }
    kernel_eval_at(consts, v[2], &v[3..3 + theta])
}

/// `kernel_eval` given the parameter and the θ columns outright — what a kernel whose columns
/// are laid out otherwise (a tangency's line after the curve's coordinates) calls.
pub fn kernel_eval_at(consts: &[f64], u: f64, theta_cols: &[f64]) -> Val {
    let Some((anchor_u, values, has_pose, flat)) = decode(consts) else { return Val::default() };
    let Some(n_q) = view(flat).map(|w| w.n_q) else { return Val::default() };
    if flat.len() < n_q {
        return Val::default();
    }
    let pose = has_pose.then(|| &flat[flat.len() - n_q..]);
    let mut outer = [0.0f64; tape::MAX_VARS];
    let Some(outer) = outer_of(u, theta_cols, values, &mut outer) else { return Val::default() };
    LOCUS_SCRATCH.with(|s| {
        let s = &mut *s.borrow_mut();
        // A compiled block's constants stay where they are put: `refresh_consts` rewrites a
        // trace contact's in place (it is not among `System::spans` — those are a *spline*
        // contact's knots), so the address is the contact's for the life of the system.  What
        // the rewrite may change is the instance's `values`, and those ride in `outer` too, so
        // a changed one misses below rather than reading stale.  Only a **recompile** can put
        // another contact at this address, which is why `System::new` calls `forget`.
        let key = (flat.as_ptr() as usize, flat.len());
        if let Some(seen) = s.seen.get(&key) {
            if seen.outer == outer {
                return seen.val;
            }
        }
        eval_at(flat, outer, Anchor { u: anchor_u, pose }, Some(key), s)
    })
}

/// A contact's constants, read once for both entries: `[anchor u, n_values, values…, has_pose,
/// locus flat…, pose…]` — the pose is the drawn instance's, `n_q` wide (the flat says how wide)
/// and meaningful when `has_pose` says so, reserved either way so the block's constants are one
/// width.  The flat handed back still carries the pose after it, which `view` ignores.
fn decode(consts: &[f64]) -> Option<(f64, &[f64], bool, &[f64])> {
    let &u = consts.first()?;
    let &nv = consts.get(1)?;
    if !(nv >= 0.0 && nv <= tape::MAX_VARS as f64) {
        return None;
    }
    let nv = nv as usize;
    let values = consts.get(2..2 + nv)?;
    let &has_pose = consts.get(2 + nv)?;
    let flat = consts.get(3 + nv..)?;
    Some((u, values, has_pose == 1.0, flat))
}

/// The outer vector: the parameter column, the θ columns, then the given numbers.
fn outer_of<'a>(u: f64, theta: &[f64], values: &[f64], into: &'a mut [f64; tape::MAX_VARS]) -> Option<&'a [f64]> {
    let width = 1 + theta.len() + values.len();
    if width > tape::MAX_VARS {
        return None;
    }
    into[0] = u;
    into[1..1 + theta.len()].copy_from_slice(theta);
    into[1 + theta.len()..width].copy_from_slice(values);
    Some(&into[..width])
}

/// One Newton solve of the block at `u` from a pose already found — the step a resume and a
/// difference both take.  `true` when it converged; the pose is then in `s.xv`.
fn warm(v: &View, s: &mut Scratch, u: f64, outer: &[f64], q: &[f64]) -> bool {
    refresh(v, s, u, outer);
    s.xv[v.n_outer..v.n_outer + v.n_q].copy_from_slice(q);
    newton(v, s)
}

/// A traced curve's **frame** at a contact: the point, its exact first derivative in `[u, θ…]`
/// (the implicit function theorem, as `kernel_eval` gives it), and the Jacobian of that first
/// derivative — which the theorem does not give without second derivatives of the block's
/// kernels, and which is therefore a **forward difference** of the exact velocity from the
/// memoised centre, one warm-started block solve per column.  A tangency's residual is exact;
/// its Jacobian is accurate to the difference, which is what a solver's Jacobian needs and no
/// more.  A curvature needs the second derivative exactly, and a traced curve cannot give it —
/// see `constraints::validate`.
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub val: Val,
    /// `d1[k][j] = ∂(dC_k/du)/∂outer[j]` for `k` in x, y — by finite difference.
    pub d1: [[f64; tape::MAX_VARS]; 2],
}

pub fn kernel_frame(consts: &[f64], u: f64, theta_cols: &[f64]) -> Frame {
    let val = kernel_eval_at(consts, u, theta_cols);
    let mut d1 = [[0.0f64; tape::MAX_VARS]; 2];
    if !val.ok {
        return Frame { val, d1 };
    }
    let Some((_, values, _, flat)) = decode(consts) else { return Frame { val, d1 } };
    let mut outer = [0.0f64; tape::MAX_VARS];
    let Some(outer) = outer_of(u, theta_cols, values, &mut outer) else { return Frame { val, d1 } };
    let key = (flat.as_ptr() as usize, flat.len());
    LOCUS_SCRATCH.with(|s| {
        let s = &mut *s.borrow_mut();
        // the pose the centre was answered at — `kernel_eval_at` always leaves it here
        let mut q = [0.0f64; MAX_Q];
        let Some(n_q) = s.seen.get(&key).map(|p| {
            q[..p.q.len()].copy_from_slice(&p.q);
            p.q.len()
        }) else { return };
        let Some(vw) = prepare(flat, outer, s) else { return };
        let mut probe = [0.0f64; tape::MAX_VARS];
        probe[..outer.len()].copy_from_slice(outer);
        for j in 0..1 + theta_cols.len() {
            let h = 1e-5 * outer[j].abs().max(1.0);
            probe[j] = outer[j] + h;
            // a continuation step of one difference from the remembered pose, never a cold
            // start, so the branch cannot change under it; the perturbed pose is not kept
            let ok = warm(&vw, s, probe[0], &probe[..outer.len()], &q[..n_q]);
            let f = if ok { finish(&vw, s, true) } else { Val::default() };
            probe[j] = outer[j];
            if !f.ok {
                d1 = [[f64::NAN; tape::MAX_VARS]; 2];
                return;
            }
            d1[0][j] = (f.dx[0] - val.dx[0]) / h;
            d1[1][j] = (f.dy[0] - val.dy[0]) / h;
        }
    });
    Frame { val, d1 }
}

/// Drop every remembered pose.  `System::new` calls it: a contact's constants are addressed by
/// where they live, and a compile is the one thing that moves them.
pub fn forget() {
    LOCUS_SCRATCH.with(|s| s.borrow_mut().seen.clear());
}

thread_local! {
    /// Scratch the kernel path runs in — a kernel is a `fn` and cannot own state, and allocating
    /// per residual is the one thing the compile-to-plan seam exists to prevent.
    static LOCUS_SCRATCH: std::cell::RefCell<Scratch> = std::cell::RefCell::new(Scratch::new());
}
