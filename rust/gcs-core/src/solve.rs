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
use crate::model::{increments, orientation, EntRef, Sketch};
use crate::newton::{self, Info, Method};
use crate::system::System;

#[derive(Clone, Debug)]
pub struct SolveResult {
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
    /// Relative to extent² (residual units for squared distances).
    pub tol: f64,
    pub max_nfev: i32,
    pub writeback: bool,
    pub max_iter: i32,
    pub dense: Option<bool>,
}

impl Default for SolveOpts {
    fn default() -> SolveOpts {
        SolveOpts {
            method: Method::DogLeg,
            tol: 1e-14,
            max_nfev: 0,
            writeback: true,
            max_iter: 100,
            dense: None,
        }
    }
}

impl System {
    pub fn solve(&mut self, sk: &mut Sketch, opts: SolveOpts) -> SolveResult {
        let mut z = self.z0(sk);
        let info: Info = newton::solve_system(
            self,
            opts.method,
            opts.tol * self.min_hard_scale,
            1e-12,
            1e-16 * self.scale,
            opts.max_iter,
            opts.max_nfev,
            opts.dense,
            &mut z,
        );
        if opts.writeback {
            let x = self.full_x(&z);
            sk.set_x(&x);
        }
        let r = self.residuals(&z);
        let mut n2 = 0.0;
        let mut mx = 0.0f64;
        let mut rel = 0.0f64;
        for i in 0..r.len() {
            n2 += r[i] * r[i];
            if self.hard[i] {
                let a = r[i].abs();
                if !(a <= mx) {
                    mx = a; // NaN wins: it is not "no error"
                }
                if !(a / self.row_scale[i] <= rel) {
                    rel = a / self.row_scale[i];
                }
            }
        }
        SolveResult {
            // relative, not absolute: a radius kernel's residual is a length and a distance
            // kernel's is a length squared, so one absolute threshold cannot judge both
            success: info.status >= 0 && rel < 1e-6,
            status: info.status,
            message: newton::status_message(info.status).to_string(),
            residual_norm: n2.sqrt(),
            max_residual: mx,
            nfev: info.nfev,
            njev: info.njev,
            time_s: 0.0,
            method: opts.method.as_str().to_string(),
            iterations: info.iterations,
            rank: if info.rank < 0 { None } else { Some(info.rank) },
        }
    }
}

/// One-shot: compile and solve, writing the result back into the sketch.
pub fn solve(sk: &mut Sketch, opts: SolveOpts) -> SolveResult {
    let mut s = System::new(sk);
    s.solve(sk, opts)
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

const PULL_ITER: i32 = 4; // the pull is a soft compromise; polish makes it exact
const POLISH_ITER: i32 = 20;

impl PullPolish {
    pub fn new(sk: &mut Sketch, target: Constraint, method: Method) -> PullPolish {
        let polish = System::new(sk);
        let id = sk.add(target);
        let pull = System::new(sk);
        PullPolish { polish, pull, target: id, method, active: true }
    }

    /// One frame: push the target's new value in, pull, then make the hard ones exact.
    pub fn pull_polish(&mut self, sk: &mut Sketch) -> SolveResult {
        self.pull.update_consts(sk, self.target);
        self.pull.solve(
            sk,
            SolveOpts { method: self.method, max_iter: PULL_ITER, ..SolveOpts::default() },
        );
        self.polish.solve(
            sk,
            SolveOpts { method: self.method, max_iter: POLISH_ITER, ..SolveOpts::default() },
        )
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
    max_step: f64,
    last_good: Vec<f64>,
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
        let max_step = max_step_rel * sk.extent().max(1.0);
        let signs = guards.iter().map(|t| orientation(sk, t.0, t.1, t.2) >= 0.0).collect();
        let last_good = sk.get_x();
        let pp = PullPolish::new(sk, target, method);
        Drag { pp, point, guards, flips: Vec::new(), signs, max_step, last_good }
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
            let (bx, by) = sk.point_xy(self.point);
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
        let (px, py) = sk.point_xy(self.point);
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
