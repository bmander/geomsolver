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
//! kernel signature learns about curves — and `eval_flat` here is the one evaluator: the kernel,
//! the tessellation and the tests all run the flat form, so there is no second walker to drift.
//!
//! **Branches are picked by seeds and carried by continuity.**  A taut string unwinds clockwise
//! or counter-clockwise, and no regular residual can say which — chirality is discrete.  This
//! solver's answer to branches everywhere else is a seed and a warm start, and it is the answer
//! here: the block's `at (…)` seeds are expressions over `(u, θ)`, evaluation starts from them at
//! the target parameter, and when that solve fails it marches from the low end of the domain,
//! each step warm-started from the last, so the branch chosen at the start is the branch the
//! whole curve is on.

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
        home: Option<Tape>,
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
            home.is_some() as u8 as f64,
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
        if let Some(t) = &home {
            f.push(t.flat.len() as f64);
            f.extend_from_slice(&t.flat);
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

/// Scratch an evaluation runs in — held by the caller and reused, like `tape::Scratch`, because
/// a kernel evaluates the same locus for every constraint in its block.  It also carries the
/// kernel path's one-entry memo: `System` always asks for the Jacobian at the point it just
/// asked the residual of, and each of those would otherwise solve the block again.
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
    memo_key: (usize, usize, Vec<f64>),
    memo: Option<Val>,
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
            memo_key: (0, 0, Vec::new()),
            memo: None,
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
    home: Option<&'a [f64]>,
}

fn view(flat: &[f64]) -> Option<View<'_>> {
    let g = |i: usize| flat.get(i).copied().filter(|v| v.is_finite() && *v >= 0.0);
    let n_outer = g(0)? as usize;
    let n_theta = g(1)? as usize;
    let n_q = g(2)? as usize;
    let n_w = g(3)? as usize;
    let n_rows = g(4)? as usize;
    let traced = g(5)? as usize;
    let n_preds = g(6)? as usize;
    let has_home = g(7)? as usize;
    if n_outer == 0
        || n_outer > tape::MAX_VARS
        || 1 + n_theta > n_outer
        || n_q == 0
        || n_q > MAX_Q
        || n_w > MAX_W
        || n_rows > MAX_ROWS
        || n_preds > MAX_PREDS
        || has_home > 1
        || traced + 2 > n_q
    {
        return None;
    }
    let mut at = 8usize;
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
    let home = if has_home == 1 {
        let len = g(at)? as usize;
        at += 1;
        Some(take(&mut at, len)?)
    } else {
        None
    };
    Some(View { n_outer, n_theta, n_q, traced, w, rows, seeds, preds, home })
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
/// parameter first, and marches from the home when that fails; one *with* predicates always
/// marches — they are read at the home and carried by continuity, and a direct solve at the
/// target could land in a component they forbid.
pub fn eval_flat(flat: &[f64], outer: &[f64], u_start: f64, s: &mut Scratch) -> Val {
    let Some(v) = prepare(flat, outer, s) else { return Val::default() };
    let u = outer[0];
    if v.preds.is_empty() {
        refresh(&v, s, u, outer);
        seed(&v, s);
        if newton(&v, s) {
            return finish(&v, s, true);
        }
    }
    let u_home = home_of(&v, s, outer, u_start);
    let mut ok = cold_start(&v, s, outer, u_home);
    if ok && u != u_home {
        ok = march(&v, s, outer, u_home, u, false);
    }
    finish(&v, s, ok)
}

/// Where evaluation is anchored: the family's `from` expression, read with the parameter as 0,
/// or the given start (the instance's domain) when it declares none.
fn home_of(v: &View, s: &mut Scratch, outer: &[f64], u_start: f64) -> f64 {
    match v.home {
        Some(t) => {
            let mut x = [0.0f64; tape::MAX_VARS];
            x[..v.n_outer].copy_from_slice(&outer[..v.n_outer]);
            x[0] = 0.0;
            let h = tape::eval_flat(t, v.n_outer, &x[..v.n_outer], &mut s.ts).v;
            if h.is_finite() {
                h
            } else {
                u_start
            }
        }
        None => u_start,
    }
}

/// The solve everything else is carried from: seeds (or restarts) at the home, then the
/// predicates.  The restarts are drawn from a *fixed* seed, so every evaluation tries the same
/// starts and the answer cannot depend on history — and each violated predicate reflects its
/// point across the oriented line and solves again, which is the move between the two components
/// with Newton finishing it.
fn cold_start(v: &View, s: &mut Scratch, outer: &[f64], u_home: f64) -> bool {
    refresh(v, s, u_home, outer);
    seed(v, s);
    let mut ok = newton(v, s);
    if !ok {
        let mut rng = crate::rng::Rng::new(0x7ace);
        let scale = outer[..v.n_outer].iter().fold(1.0f64, |m, &x| m.max(x.abs()));
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
pub fn sweep(flat: &[f64], outer: &[f64], u0: f64, u1: f64, n: usize, s: &mut Scratch)
    -> Vec<(f64, f64)>
{
    let Some(v) = prepare(flat, outer, s) else { return Vec::new() };
    if n == 0 {
        return Vec::new();
    }
    let q0 = v.n_outer;
    let mut out = Vec::with_capacity(n + 1);
    // anchor at the home, walk to the near end, then sample — one continuation throughout, so
    // the branch the home picks is the branch the whole polyline is on
    let u_home = home_of(&v, s, outer, u0);
    let _ = cold_start(&v, s, outer, u_home);
    if u_home != u0 {
        let _ = march(&v, s, outer, u_home, u0, true);
    }
    for k in 0..=n {
        let u = u0 + (u1 - u0) * k as f64 / n as f64;
        refresh(&v, s, u, outer);
        let _ = newton(&v, s);
        out.push((s.xv[q0 + v.traced], s.xv[q0 + v.traced + 1]));
    }
    out
}

/// The two derivative-carrying evaluations a contact kernel makes, bundled: `kernels` calls this
/// so the residual and the Jacobian read one code path.  A one-entry memo sits in front of the
/// solve, because `System` always asks for the Jacobian at the point it just asked the residual
/// of: the answer is a pure function of the constants and the columns past the contact point, so
/// remembering the last one is deterministic and halves the block solves.
pub fn kernel_eval(consts: &[f64], v: &[f64], n_par: usize) -> Val {
    // the contact's constants: [u_start, n_values, values…, locus flat…]
    let Some(&u_start) = consts.first() else { return Val::default() };
    let Some(&nv) = consts.get(1) else { return Val::default() };
    if !(nv >= 0.0 && nv <= tape::MAX_VARS as f64) {
        return Val::default();
    }
    let nv = nv as usize;
    let Some(values) = consts.get(2..2 + nv) else { return Val::default() };
    let Some(flat) = consts.get(2 + nv..) else { return Val::default() };
    // the outer vector: the parameter column, the θ columns, then the given numbers
    let theta = n_par.saturating_sub(3);
    if v.len() < 3 + theta {
        return Val::default();
    }
    let mut outer = [0.0f64; tape::MAX_VARS];
    if 1 + theta + nv > tape::MAX_VARS {
        return Val::default();
    }
    outer[0] = v[2];
    outer[1..1 + theta].copy_from_slice(&v[3..3 + theta]);
    outer[1 + theta..1 + theta + nv].copy_from_slice(values);
    let outer = &outer[..1 + theta + nv];
    LOCUS_SCRATCH.with(|s| {
        let s = &mut *s.borrow_mut();
        // the constants live in a compiled block and are only ever rewritten in place with the
        // same values, so (address, length) identifies them; the outer values carry the rest
        let key = (flat.as_ptr() as usize, flat.len());
        if let Some(val) = s.memo {
            if (s.memo_key.0, s.memo_key.1) == key && s.memo_key.2 == outer {
                return val;
            }
        }
        let val = eval_flat(flat, outer, u_start, s);
        s.memo_key.0 = key.0;
        s.memo_key.1 = key.1;
        s.memo_key.2.clear();
        s.memo_key.2.extend_from_slice(outer);
        s.memo = Some(val);
        val
    })
}

thread_local! {
    /// Scratch the kernel path runs in — a kernel is a `fn` and cannot own state, and allocating
    /// per residual is the one thing the compile-to-plan seam exists to prevent.
    static LOCUS_SCRATCH: std::cell::RefCell<Scratch> = std::cell::RefCell::new(Scratch::new());
}
