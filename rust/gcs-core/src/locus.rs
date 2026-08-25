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

/// Continuation steps when the direct solve fails: the parameter walks from the domain's low end
/// to the target, each step seeded with the last step's answer.
const MARCH: usize = 32;
const NEWTON_MAX: usize = 30;

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

/// A compiled trace block.  The variable table its rows index is
/// `[u, θ…, values…] ++ q ++ w`: the outer variables in the family's tape order (so `n_outer`
/// is `CurveDef::vars.len()`), then the inner unknowns, then the derived values.
#[derive(Clone, Debug)]
pub struct Locus {
    pub n_outer: usize,
    pub n_theta: usize,
    pub n_q: usize,
    /// The q slot of the traced point's x; its y is the slot after.
    pub traced: usize,
    /// One tape per derived value, over the outer variables.
    pub w: Vec<Tape>,
    /// One seed tape per q slot, over the outer variables — where the search starts, and the
    /// whole of how a branch is stated.  A slot nobody seeded holds a constant-zero tape.
    pub seeds: Vec<Tape>,
    pub rows: Vec<Row>,
    /// The encoding, built once — what rides in a contact's constants and what `eval_flat` runs.
    pub flat: Vec<f64>,
}

impl Locus {
    /// Assemble and encode.  Validates the shape the evaluator depends on; an `Err` is a
    /// diagnostic for the family, not a panic.
    pub fn new(
        n_outer: usize,
        n_theta: usize,
        n_q: usize,
        traced: usize,
        w: Vec<Tape>,
        seeds: Vec<Tape>,
        rows: Vec<Row>,
    ) -> Result<Locus, String> {
        if n_q == 0 || n_q > MAX_Q {
            return Err(format!("a trace block declares between 1 and {MAX_Q} unknowns"));
        }
        if rows.len() > MAX_ROWS || w.len() > MAX_W {
            return Err(format!(
                "a trace block may state at most {MAX_ROWS} constraints and {MAX_W} expressions"
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
        let mut l = Locus { n_outer, n_theta, n_q, traced, w, seeds, rows, flat: Vec::new() };
        l.flat = l.encode();
        Ok(l)
    }

    /// The instructions as plain numbers — see the module docs and `Tape::flat`.
    fn encode(&self) -> Vec<f64> {
        let mut f = vec![
            self.n_outer as f64,
            self.n_theta as f64,
            self.n_q as f64,
            self.w.len() as f64,
            self.rows.len() as f64,
            self.traced as f64,
        ];
        for t in &self.w {
            f.push(t.flat.len() as f64);
            f.extend_from_slice(&t.flat);
        }
        for r in &self.rows {
            f.push(r.kid as f64);
            f.push(r.cols.len() as f64);
            f.push(r.consts.len() as f64);
            f.extend(r.cols.iter().map(|&c| c as f64));
            f.extend_from_slice(&r.consts);
        }
        for t in &self.seeds {
            f.push(t.flat.len() as f64);
            f.extend_from_slice(&t.flat);
        }
        f
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
/// a kernel evaluates the same locus for every constraint in its block.
pub struct Scratch {
    ts: tape::Scratch,
    xv: Vec<f64>,
    r: Vec<f64>,
    jq: Vec<f64>,
    b: Vec<f64>,
    lu: Vec<f64>,
    rhs: Vec<f64>,
    q: Vec<f64>,
    wd: Vec<[f64; tape::MAX_VARS]>,
    v: Vec<f64>,
    jrow: Vec<f64>,
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
            rhs: Vec::new(),
            q: Vec::new(),
            wd: Vec::new(),
            v: Vec::new(),
            jrow: Vec::new(),
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
}

fn view(flat: &[f64]) -> Option<View<'_>> {
    let g = |i: usize| flat.get(i).copied().filter(|v| v.is_finite() && *v >= 0.0);
    let n_outer = g(0)? as usize;
    let n_theta = g(1)? as usize;
    let n_q = g(2)? as usize;
    let n_w = g(3)? as usize;
    let n_rows = g(4)? as usize;
    let traced = g(5)? as usize;
    if n_outer == 0
        || n_outer > tape::MAX_VARS
        || 1 + n_theta > n_outer
        || n_q == 0
        || n_q > MAX_Q
        || n_w > MAX_W
        || n_rows > MAX_ROWS
        || traced + 2 > n_q
    {
        return None;
    }
    let mut at = 6usize;
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
    Some(View { n_outer, n_theta, n_q, traced, w, rows, seeds })
}

/// One Newton solve of the block at the parameter already written into `s.xv[0]`, from the `q`
/// already in `s.xv`.  Residual rows are judged against their own kernel's power of length over
/// the magnitude of what they read — the same reasoning as `System::max_relative_residual`, in
/// miniature — so one tolerance serves a block at any size.
fn newton(v: &View, s: &mut Scratch) -> bool {
    let (n_q, q0) = (v.n_q, v.n_outer);
    let mut last = f64::INFINITY;
    for _ in 0..NEWTON_MAX {
        let norm = assemble(v, s, false);
        if !norm.is_finite() {
            return false;
        }
        if norm <= 1e-12 {
            return true;
        }
        // solve Jq Δ = -r
        s.lu.clear();
        s.lu.extend_from_slice(&s.jq);
        s.rhs.clear();
        s.rhs.extend(s.r.iter().map(|&x| -x));
        if !crate::linalg::lu_solve(n_q, &mut s.lu, &mut s.rhs) {
            return false;
        }
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
            let nn = assemble(v, s, false);
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
            return norm <= 1e-8;
        }
        let dq: f64 = s.rhs.iter().map(|d| d * d * step * step).sum::<f64>().sqrt();
        let qn: f64 = s.q.iter().map(|x| x * x).sum::<f64>().sqrt();
        if dq <= 1e-14 * (1.0 + qn) {
            return assemble(v, s, false) <= 1e-8;
        }
        if best >= last * 0.999999 && best > 1e-10 {
            // stalled without converging
            return false;
        }
        last = best;
    }
    assemble(v, s, false) <= 1e-10
}

/// Residuals and Jacobian of the block at `s.xv`.  Fills `s.r` and `s.jq`, and `s.b` — the
/// columns for `[u, θ…]`, with a derived value's contribution chained through its tape — when
/// `with_b`.  Returns the worst residual over its own row's units, which is dimensionless, so
/// one tolerance judges a block at any size.
fn assemble(v: &View, s: &mut Scratch, with_b: bool) -> f64 {
    let n_q = v.n_q;
    let n_dc = 1 + v.n_theta;
    let (q0, w0) = (v.n_outer, v.n_outer + n_q);
    s.r.clear();
    s.r.resize(n_q, 0.0);
    s.jq.clear();
    s.jq.resize(n_q * n_q, 0.0);
    if with_b {
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
        s.jrow.clear();
        s.jrow.resize(kn.n_res * kn.n_par, 0.0);
        if let Some(cj) = kn.const_jac {
            s.jrow.copy_from_slice(cj);
        } else {
            (kn.jac)(1, &s.v, consts, &mut s.jrow);
        }
        let unit = mag.powi(kn.degree as i32);
        for t in 0..kn.n_res {
            worst = worst.max(s.r[row0 + t].abs() / unit);
            for (c, &col) in cols.iter().enumerate() {
                let g = s.jrow[t * kn.n_par + c];
                if g == 0.0 {
                    continue;
                }
                let col = col as usize;
                if col >= w0 {
                    if with_b {
                        let wd = &s.wd[col - w0];
                        for d in 0..n_dc {
                            s.b[(row0 + t) * n_dc + d] += g * wd[d];
                        }
                    }
                } else if col >= q0 {
                    s.jq[(row0 + t) * n_q + (col - q0)] += g;
                } else if with_b && col < n_dc {
                    s.b[(row0 + t) * n_dc + col] += g;
                }
                // an outer column past the θ block is one of the instance's given numbers: a
                // constant of this curve, whose gradient nobody is owed
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

/// Evaluate the locus at `outer = [u, θ…, values…]`: solve the block, then the implicit function
/// theorem for the derivatives.  Tries the seeds at the target parameter first; when that solve
/// fails it marches from `u_start` (the low end of the instance's domain), each step seeded with
/// the last — which is what carries a branch along the curve.
pub fn eval_flat(flat: &[f64], outer: &[f64], u_start: f64, s: &mut Scratch) -> Val {
    let Some(v) = view(flat) else { return Val::default() };
    if outer.len() < v.n_outer {
        return Val::default();
    }
    let width = v.n_outer + v.n_q + v.w.len();
    s.xv.clear();
    s.xv.resize(width, 0.0);
    s.wd.clear();
    s.wd.resize(v.w.len(), [0.0; tape::MAX_VARS]);
    let u = outer[0];

    refresh(&v, s, u, outer);
    seed(&v, s);
    let mut ok = newton(&v, s);
    if !ok && u != u_start {
        // continuation: start where the seeds are trusted and walk to the target
        refresh(&v, s, u_start, outer);
        seed(&v, s);
        ok = newton(&v, s);
        if ok {
            for k in 1..=MARCH {
                let uk = u_start + (u - u_start) * k as f64 / MARCH as f64;
                refresh(&v, s, uk, outer);
                ok = newton(&v, s);
                if !ok {
                    break;
                }
            }
        }
    }
    finish(&v, s, ok)
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
    if !assemble(v, s, true).is_finite() {
        out.ok = false;
        return out;
    }
    for d in 0..n_dc {
        s.lu.clear();
        s.lu.extend_from_slice(&s.jq);
        s.rhs.clear();
        s.rhs.extend((0..n_q).map(|i| -s.b[i * n_dc + d]));
        if !crate::linalg::lu_solve(n_q, &mut s.lu, &mut s.rhs) {
            out.ok = false;
            return out;
        }
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
    let Some(v) = view(flat) else { return Vec::new() };
    if outer.len() < v.n_outer || n == 0 {
        return Vec::new();
    }
    let width = v.n_outer + v.n_q + v.w.len();
    s.xv.clear();
    s.xv.resize(width, 0.0);
    s.wd.clear();
    s.wd.resize(v.w.len(), [0.0; tape::MAX_VARS]);
    let q0 = v.n_outer;
    let mut out = Vec::with_capacity(n + 1);
    refresh(&v, s, u0, outer);
    seed(&v, s);
    for k in 0..=n {
        let u = u0 + (u1 - u0) * k as f64 / n as f64;
        refresh(&v, s, u, outer);
        if !newton(&v, s) {
            // a failed step falls back to the written seeds and tries once more; the drawn
            // point is then the best available, and a genuinely impossible block draws a
            // stationary run rather than nothing
            seed(&v, s);
            let _ = newton(&v, s);
        }
        out.push((s.xv[q0 + v.traced], s.xv[q0 + v.traced + 1]));
    }
    out
}

/// The two derivative-carrying evaluations a contact kernel makes, bundled: `kernels` calls this
/// so the residual and the Jacobian read one code path.
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
    LOCUS_SCRATCH.with(|s| eval_flat(flat, &outer[..1 + theta + nv], u_start, &mut s.borrow_mut()))
}

thread_local! {
    /// Scratch the kernel path runs in — a kernel is a `fn` and cannot own state, and allocating
    /// per residual is the one thing the compile-to-plan seam exists to prevent.
    static LOCUS_SCRATCH: std::cell::RefCell<Scratch> = std::cell::RefCell::new(Scratch::new());
}
