//! Interactive dragging.
//!
//! `Drag` is the one point-drag implementation (pull + polish, `PullPolish`), `RadiusDrag` its
//! scalar counterpart for circle/arc radii (a `Radius` with `soft` set — its residual is already
//! r − target, so it needs no kernel of its own).  Front ends only translate coordinates.
//!
//! Stage 5 robustness lives here: continuation (a far cursor jump is taken in increments so the
//! solution tracks its homotopy branch instead of teleporting across it) and order-type guards (a
//! step that would flip a guarded triangle's orientation is retried with smaller increments, and
//! an unavoidable flip is recorded and flagged).

use crate::constraints::{CKind, Constraint};
use crate::curve;
use crate::model::{increments, orientation, EntRef, Sketch};
use crate::newton::Method;
use crate::system::System;

use super::{BlockMode, SolveOpts, SolveResult};

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
pub(super) const NO_REHOME: SolveOpts = SolveOpts {
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
pub(super) const POLISH_ITER: i32 = 20;

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
    /// The axis directions a dragged plane is held to while it is carried, let go at `end`.
    turning: Vec<u32>,
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
    /// stands along the line of sight is the constraints' to say, else where it was taken hold
    /// of (`CKind::DragSeen`).  A plane's origin is dragged so, moving where the plane stands:
    /// the plane is **carried, not turned** — its axes' directions are held while the drag lasts
    /// (`turning`), since its axes stand through its origin and a step would otherwise spread
    /// into turning them as much as into moving it.
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
        let (right, up) = crate::space::eye(az, el);
        let depth = crate::space::dot(crate::space::cross(right, up), sk.world_point(point));
        let target = Constraint::drag_seen(EntRef::point(point), x, y, weight, (az, el), depth);
        let eye = Some((right, up));
        let turning = match (sk.points[point].z, sk.plane_of_origin(point)) {
            (None, Some(pl)) => {
                let p = &sk.frames[pl];
                let d = [p.u, p.v].map(|r| sk.axes[r as usize].d);
                d.into_iter().flatten().filter(|&q| !sk.params[q as usize].fixed).collect()
            }
            _ => Vec::new(),
        };
        for &q in &turning {
            sk.params[q as usize].fixed = true;
        }
        let mut d = Drag::with(sk, point, target, eye, method, guards, max_step_rel);
        d.turning = turning;
        d
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
        let turning = Vec::new();
        Drag { pp, point, guards, flips: Vec::new(), signs, max_step, last_good, eye, turning }
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
        self.pp.end(sk);
        for q in self.turning.drain(..) {
            sk.params[q as usize].fixed = false;
        }
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
