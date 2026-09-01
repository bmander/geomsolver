//! Ellipse geometry: the drawn figure of an `EllipseE`, asked in world coordinates.
//!
//! An ellipse is a centre Point, a Point at one end of its major axis, and a minor radius of its
//! own — five numbers, which is exactly the 5 DOF an ellipse has, so unlike an arc it needs no
//! intrinsic constraint.  The major point is a real rim point, so it drags, snaps and constrains
//! with the tools that already exist, the same trick as an arc being a centre and two real
//! points.  Like a curve, the ellipse is *geometry*: the pick test, the bounds and the seed for
//! a fresh contact are computed here, and a front end only strokes the figure the same five
//! numbers already describe.

use crate::model::Sketch;

/// The five numbers an ellipse is drawn from, read once per question.
///
/// This is the **one** definition of the rim's parametrization: the kernels build one of these
/// out of their raw columns and ask it for E, E' and E'' exactly as the pick test does, so the
/// solver and the drawing cannot come to disagree about where the rim is.  Two copies of the
/// formula would be two formulas the moment one of them was edited — a sign convention changed
/// in one place and a contact that solves onto a rim nothing draws.
#[derive(Clone, Copy, Debug)]
pub struct Geom {
    pub cx: f64,
    pub cy: f64,
    /// Centre → major-axis endpoint.
    pub ux: f64,
    pub uy: f64,
    /// Semi-major length |u|, floored like a degenerate line so no question divides by zero.
    pub a: f64,
    /// Minor radius, **signed as stored**.  A negative one draws the same rim traced the other
    /// way, so nothing that measures the figure cares; the kernels do, because their columns are
    /// the raw parameter and their derivatives must match the value they differentiate.
    pub b: f64,
}

impl Geom {
    /// From the five numbers themselves — what a kernel has in its columns.
    pub fn new(cx: f64, cy: f64, mx: f64, my: f64, b: f64) -> Geom {
        let (ux, uy) = (mx - cx, my - cy);
        Geom { cx, cy, ux, uy, a: ux.hypot(uy).max(crate::kernels::MIN_LINE_LEN), b }
    }

    /// The unit normal the minor axis runs along: perp(u)/|u|.
    #[inline]
    pub fn normal(&self) -> (f64, f64) {
        (-self.uy / self.a, self.ux / self.a)
    }

    /// E(θ) = c + cos θ·u + sin θ·b·perp(u)/a.
    pub fn point_at(&self, theta: f64) -> (f64, f64) {
        let (s, c) = theta.sin_cos();
        let (nx, ny) = self.normal();
        (self.cx + c * self.ux + self.b * s * nx, self.cy + c * self.uy + self.b * s * ny)
    }

    /// E'(θ) and E''(θ) — the closest-point Newton reads them, the kernels differentiate against
    /// them, and a test checks a contact's tangent and curvature against them.
    pub fn derivs(&self, theta: f64) -> ((f64, f64), (f64, f64)) {
        let (s, c) = theta.sin_cos();
        let (nx, ny) = self.normal();
        (
            (-s * self.ux + self.b * c * nx, -s * self.uy + self.b * c * ny),
            (-c * self.ux - self.b * s * nx, -c * self.uy - self.b * s * ny),
        )
    }

    /// E, E' and E'' at one θ, for a caller that wants all three — one `sin_cos` rather than the
    /// two `point_at` and `derivs` would each take.  The projection Newton runs this per step on
    /// the pick path, which a pointer move walks for every entity in the sketch.
    pub fn frame(&self, theta: f64) -> ((f64, f64), (f64, f64), (f64, f64)) {
        let (s, c) = theta.sin_cos();
        let (nx, ny) = self.normal();
        (
            (self.cx + c * self.ux + self.b * s * nx, self.cy + c * self.uy + self.b * s * ny),
            (-s * self.ux + self.b * c * nx, -s * self.uy + self.b * c * ny),
            (-c * self.ux - self.b * s * nx, -c * self.uy - self.b * s * ny),
        )
    }

    /// `n` rim points, evenly spaced in θ, as (θ, E(θ)).  Every sweep this module does is one of
    /// these — the coarse pass a projection starts from, the one a tangency starts from, and the
    /// polyline a distance is measured against — so how the rim is walked is written once.
    pub fn samples(&self, n: usize) -> impl Iterator<Item = (f64, (f64, f64))> + '_ {
        (0..n).map(move |k| {
            let t = std::f64::consts::TAU * k as f64 / n as f64;
            (t, self.point_at(t))
        })
    }

    /// The sample whose `cost` is least — the basin a Newton pass then refines within.
    fn best_of(&self, n: usize, cost: impl Fn((f64, f64)) -> f64) -> (f64, f64) {
        self.samples(n)
            .map(|(t, p)| (t, cost(p)))
            .fold((0.0, f64::INFINITY), |acc, it| if it.1 < acc.1 { it } else { acc })
    }
}

pub fn geom(sk: &Sketch, i: usize) -> Geom {
    let e = &sk.ellipses[i];
    let (cx, cy) = sk.point_xy(e.center as usize);
    let (mx, my) = sk.point_xy(e.major as usize);
    Geom::new(cx, cy, mx, my, sk.params[e.minor as usize].value)
}

/// `n` rim points, evenly spaced in θ — the sweep `distance_between` measures a pair with no
/// closed form by, exactly as a curve is swept.
pub fn sample(sk: &Sketch, i: usize, n: usize) -> Vec<(f64, f64)> {
    geom(sk, i).samples(n).map(|(_, p)| p).collect()
}

/// How many chords a drawn rim is walked in.
pub const RIM: usize = 180;

/// The rim as a **closed** polyline: one turn of `RIM` chords and the first point again, which
/// is what the SVG export and the box both stroke.
pub fn rim(sk: &Sketch, i: usize) -> Vec<(f64, f64)> {
    let mut pts = sample(sk, i, RIM);
    if let Some(&first) = pts.first() {
        pts.push(first);
    }
    pts
}

/// The parameter of the rim point nearest (x, y), and how far that is.  A coarse sweep for the
/// basin — the nearest of an ellipse's up-to-four critical points is the branch a fresh contact
/// should start on — then Newton on (E(θ) − q)·E'(θ) = 0 to land on it.
pub fn closest(sk: &Sketch, i: usize, x: f64, y: f64) -> (f64, f64) {
    let g = geom(sk, i);
    let best = g.best_of(32, |p| (p.0 - x).hypot(p.1 - y));
    let mut t = best.0;
    for _ in 0..24 {
        let (p, d1, d2) = g.frame(t);
        let (wx, wy) = (p.0 - x, p.1 - y);
        let f = wx * d1.0 + wy * d1.1;
        let h = d1.0 * d1.0 + d1.1 * d1.1 + wx * d2.0 + wy * d2.1;
        if h.abs() <= 1e-30 {
            break;
        }
        let nt = t - f / h;
        if (nt - t).abs() <= 1e-14 * (1.0 + t.abs()) {
            t = nt;
            break;
        }
        t = nt;
    }
    let p = g.point_at(t);
    let d = (p.0 - x).hypot(p.1 - y);
    if d <= best.1 {
        (t, d)
    } else {
        best
    }
}

/// Distance from (x, y) to the rim — the pick test, and what `distance_between` measures.
pub fn distance_to(sk: &Sketch, i: usize, x: f64, y: f64) -> f64 {
    closest(sk, i, x, y).1
}

/// The parameter at which the rim comes closest to the infinite line through (ax, ay) and
/// (bx, by) — where a fresh tangency starts, so it lands on the branch the user drew.  The
/// nearest rim point is the one whose tangent already runs the line's way.
pub fn nearest_to_line(sk: &Sketch, i: usize, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let (dx, dy) = (bx - ax, by - ay);
    let l = dx.hypot(dy);
    if l <= 1e-12 {
        return closest(sk, i, ax, ay).0;
    }
    geom(sk, i).best_of(64, |p| ((dx * (p.1 - ay) - dy * (p.0 - ax)) / l).abs()).0
}

/// The world length one unit of θ is worth — see `Param::scale`.  (a + b)/2 is within a few
/// percent of the true mean speed, and the scale preconditions the step, so an estimate costs
/// convergence rate, never correctness.
pub fn speed(sk: &Sketch, i: usize) -> f64 {
    let g = geom(sk, i);
    // |b|, because the stored minor radius is signed and a negative one draws the same rim
    (0.5 * (g.a + g.b.abs())).max(crate::kernels::MIN_LINE_LEN)
}

/// Axis-aligned bounds of the rotated rim: the half-extents are hypot(a cos φ, b sin φ) and its
/// twin, and a·cos φ is just the major offset's own component.
pub fn bounds(sk: &Sketch, i: usize) -> crate::model::Box2 {
    let g = geom(sk, i);
    let dx = g.ux.hypot(g.b * g.uy / g.a);
    let dy = g.uy.hypot(g.b * g.ux / g.a);
    (g.cx - dx, g.cy - dy, g.cx + dx, g.cy + dy)
}

/// The minor radius that puts the rim of the ellipse (centre c, major end m) through (tx, ty) —
/// the third click of the ellipse tool, and where a rim drag holds the rim to the cursor.  In
/// the ellipse's own frame b = |y| / √(1 − (x/a)²); past the ends of the major axis (where no
/// minor radius reaches the target) the perpendicular offset itself is the honest answer.
/// `None` when centre and major end coincide, which names no axis.
pub fn minor_to(cx: f64, cy: f64, mx: f64, my: f64, tx: f64, ty: f64) -> Option<f64> {
    let (ux, uy) = (mx - cx, my - cy);
    let a = ux.hypot(uy);
    if a <= 1e-12 {
        return None;
    }
    let (wx, wy) = (tx - cx, ty - cy);
    let x = (wx * ux + wy * uy) / a;
    let y = (ux * wy - uy * wx) / a;
    let s = 1.0 - (x / a) * (x / a);
    Some(if s > 1e-4 { y.abs() / s.sqrt() } else { y.abs() })
}
