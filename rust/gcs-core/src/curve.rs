//! Parametric curves: the B-spline basis, a spline's geometry, and where a contact with one
//! starts.
//!
//! Everything the sketch already holds is *implicit* — a line and a circle both have a point-
//! membership test that is polynomial in the coordinates, so a residual is written in the
//! geometry's own parameters and nothing else.  A parametric curve has no usable implicit form,
//! so a contact with one carries its own curve parameter as an unknown and says `p − C(t) = 0`:
//! two residuals, one new unknown, and the net one equation that "a point lies on a curve"
//! ought to be.  Those unknowns are `SpecKind::Param` slots, allocated by `Sketch::add`.
//!
//! The curves here are the ones *linear in their control points*, `C(t) = Σ Bᵢ(t) Pᵢ` — every
//! B-spline, and so every Bézier (a clamped span with no interior knots).  That is the whole
//! extension point: a contact kernel needs the basis values and their first two derivatives at
//! t and nothing else about the curve, so the same kernels serve any knot vector, and the
//! control points are ordinary sketch Points that drag, snap and take constraints like any
//! other.
//!
//! Local support is what keeps the plan's fixed-width blocks intact.  Only `DEGREE + 1` control
//! points are non-zero at any t, so a contact addresses one *span* and occupies a fixed number
//! of columns however long the spline is.  The span is derived from t rather than stored, and
//! `Sketch::topology_key` carries it — so a contact walking past a knot is a recompile, the
//! same event as any other topology change and about as rare.

use crate::model::Sketch;
use std::collections::BTreeMap;

/// The one degree the kernels are compiled for.  The basis below is written for general `p`; a
/// second degree costs a kernel pair and a `CKind` that selects it, which is why the model
/// stores a knot vector (any shape of cubic) but not a degree.
pub const DEGREE: usize = 3;

/// Control points one span reads — the number of non-zero basis functions.
pub const SPAN_N: usize = DEGREE + 1;

/// The local knot window one span needs: u[span-p ..= span+p+1].
pub const SPAN_K: usize = 2 * DEGREE + 2;

/// How far a tessellated chord may stray from the curve, in screen pixels.
pub const FLATNESS_PX: f64 = 0.3;

/// Recursion cap per span, so a pathological curve cannot make an unbounded polyline.
const MAX_DEPTH: u32 = 10;

/* -- second-order jets ------------------------------------------------------ */

/// A value and its first two derivatives in t.  The Cox–de Boor recurrence is a handful of
/// sums and products of these, so differentiating the basis is the recurrence itself run over
/// jets rather than a second algorithm to keep in step with the first.
#[derive(Clone, Copy, Debug)]
struct Jet {
    v: f64,
    d: f64,
    dd: f64,
}

impl Jet {
    const ZERO: Jet = Jet { v: 0.0, d: 0.0, dd: 0.0 };
    const ONE: Jet = Jet { v: 1.0, d: 0.0, dd: 0.0 };

    /// `t - c`, as a function of t.
    fn rise(t: f64, c: f64) -> Jet {
        Jet { v: t - c, d: 1.0, dd: 0.0 }
    }

    /// `c - t`, as a function of t.
    fn fall(c: f64, t: f64) -> Jet {
        Jet { v: c - t, d: -1.0, dd: 0.0 }
    }

    fn add(self, o: Jet) -> Jet {
        Jet { v: self.v + o.v, d: self.d + o.d, dd: self.dd + o.dd }
    }

    fn mul(self, o: Jet) -> Jet {
        Jet {
            v: self.v * o.v,
            d: self.d * o.v + self.v * o.d,
            dd: self.dd * o.v + 2.0 * self.d * o.d + self.v * o.dd,
        }
    }

    fn scale(self, s: f64) -> Jet {
        Jet { v: self.v * s, d: self.d * s, dd: self.dd * s }
    }
}

/* -- the basis -------------------------------------------------------------- */

/// The `DEGREE + 1` basis functions that are non-zero on one span, and their first two
/// derivatives in t.  `lk` is the span's local knot window and `b`, `d` and `dd` are filled for
/// the control points `P[span-p ..= span]`, in that order.
///
/// This is Cox–de Boor run over jets.  A zero denominator is a knot repeated enough times to
/// collapse a span, and the basis function it divides is zero there: the convention is that the
/// whole term is zero, which is also what keeps a clamped knot vector (where `u[p] == u[0]`)
/// from putting a NaN in the Jacobian.
///
/// The recurrence is written for a general degree, but the buffers and the kernels' column
/// counts are sized from `DEGREE`, so a second degree costs a kernel pair and a `CKind` that
/// selects it — see the module docs.
pub fn basis(t: f64, lk: &[f64; SPAN_K], b: &mut [f64; SPAN_N], d: &mut [f64; SPAN_N],
             dd: &mut [f64; SPAN_N]) {
    const P: usize = DEGREE;
    // cur[a] holds N_{span-p+a, deg}; the slot at p+1 stays zero so the recurrence can read it
    let mut cur = [Jet::ZERO; P + 2];
    let mut next = [Jet::ZERO; P + 2];
    cur[P] = Jet::ONE; // N_{span,0} = 1 on this span, every other degree-0 function 0
    for deg in 1..=P {
        for slot in next.iter_mut() {
            *slot = Jet::ZERO;
        }
        for a in (P - deg)..=P {
            let mut acc = Jet::ZERO;
            let den = lk[a + deg] - lk[a];
            if den != 0.0 {
                acc = acc.add(Jet::rise(t, lk[a]).scale(1.0 / den).mul(cur[a]));
            }
            let den = lk[a + deg + 1] - lk[a + 1];
            if den != 0.0 {
                acc = acc.add(Jet::fall(lk[a + deg + 1], t).scale(1.0 / den).mul(cur[a + 1]));
            }
            next[a] = acc;
        }
        cur = next;
    }
    for a in 0..=P {
        b[a] = cur[a].v;
        d[a] = cur[a].d;
        dd[a] = cur[a].dd;
    }
}

/* -- knot vectors ----------------------------------------------------------- */

/// The clamped uniform knot vector for `n` control points: the curve runs from the first control
/// point to the last, and its parameter domain is `0 ..= n - DEGREE`, one unit per span.
pub fn clamped_uniform(n: usize) -> Vec<f64> {
    let p = DEGREE;
    let spans = n.saturating_sub(p).max(1);
    let mut u = vec![0.0; p + 1];
    for i in 1..spans {
        u.push(i as f64);
    }
    u.extend(std::iter::repeat_n(spans as f64, p + 1));
    u
}

/// Whether a knot vector is one `n` control points of degree `DEGREE` can be drawn with:
/// the right length, non-decreasing, and with a non-empty parameter domain.
pub fn knots_valid(u: &[f64], n: usize) -> bool {
    n > DEGREE
        && u.len() == n + DEGREE + 1
        && u.iter().all(|x| x.is_finite())
        && u.windows(2).all(|w| w[0] <= w[1])
        && u[DEGREE] < u[n]
}

/// The knot vector for a control polygon that has lost the control points at `gone`.
///
/// One interior knot goes with each, so a curve shaped by knot insertion degrades back toward
/// what it was rather than being re-spaced from scratch — deleting a control point is very
/// nearly the inverse of inserting one.  Falls back to the clamped uniform vector when what is
/// left is not a knot vector at all.
pub fn knots_without(u: &[f64], gone: &[usize], n_left: usize) -> Vec<f64> {
    let mut v = u.to_vec();
    // highest first, so a removal never shifts an index still to come
    for &j in gone.iter().rev() {
        let lo = DEGREE + 1;
        let hi = v.len().saturating_sub(DEGREE + 2);
        if lo > hi {
            break; // no interior knots left to give up
        }
        v.remove((j + DEGREE + 1).clamp(lo, hi));
    }
    if knots_valid(&v, n_left) {
        v
    } else {
        clamped_uniform(n_left)
    }
}

/// The span `t` falls in — an index into the knot vector with `u[s] <= t < u[s+1]`, skipping the
/// empty spans a repeated knot makes, and clamped to the ends of the domain.
pub fn span_index(u: &[f64], n: usize, t: f64) -> usize {
    let p = DEGREE;
    let mut best = p;
    for s in p..n {
        if u[s] == u[s + 1] {
            continue;
        }
        best = s;
        if t < u[s + 1] {
            break;
        }
    }
    best
}

/// `u[span-p ..= span+p+1]` — what `basis` reads.
pub fn local_knots(u: &[f64], span: usize) -> [f64; SPAN_K] {
    let mut out = [0.0; SPAN_K];
    out.copy_from_slice(&u[span - DEGREE..span + DEGREE + 2]);
    out
}

/* -- a spline's geometry ---------------------------------------------------- */

/// A point on the curve with its first two derivatives in t.
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub p: (f64, f64),
    pub d1: (f64, f64),
    pub d2: (f64, f64),
}

/// The parameter interval the curve is drawn over.
pub fn domain(sk: &Sketch, i: usize) -> (f64, f64) {
    let s = &sk.splines[i];
    (s.knots[DEGREE], s.knots[s.ctrl.len()])
}

pub fn span_of(sk: &Sketch, i: usize, t: f64) -> usize {
    let s = &sk.splines[i];
    span_index(&s.knots, s.ctrl.len(), t)
}

/// C(t), C'(t) and C''(t).
pub fn eval(sk: &Sketch, i: usize, t: f64) -> Frame {
    let s = &sk.splines[i];
    eval_on(sk, i, span_index(&s.knots, s.ctrl.len(), t), t)
}

/// The same on a span the caller already knows.  Every sweep below walks span by span, so
/// finding the span again per sample would make each of them quadratic in the span count — and
/// `closest` is on the pick path, which runs for every curve on every pointer move.
pub fn eval_on(sk: &Sketch, i: usize, span: usize, t: f64) -> Frame {
    let s = &sk.splines[i];
    let lk = local_knots(&s.knots, span);
    let (mut b, mut d, mut dd) = ([0.0; SPAN_N], [0.0; SPAN_N], [0.0; SPAN_N]);
    basis(t, &lk, &mut b, &mut d, &mut dd);
    let mut f = Frame { p: (0.0, 0.0), d1: (0.0, 0.0), d2: (0.0, 0.0) };
    for a in 0..SPAN_N {
        let (x, y) = sk.point_xy(s.ctrl[span - DEGREE + a] as usize);
        f.p.0 += b[a] * x;
        f.p.1 += b[a] * y;
        f.d1.0 += d[a] * x;
        f.d1.1 += d[a] * y;
        f.d2.0 += dd[a] * x;
        f.d2.1 += dd[a] * y;
    }
    f
}

pub fn point_at(sk: &Sketch, i: usize, t: f64) -> (f64, f64) {
    eval(sk, i, t).p
}

/// The non-empty spans and the parameter interval of each, in order.  Every knot is therefore a
/// vertex of a drawn polyline, so a repeated knot's corner is drawn as a corner and not smoothed
/// across — and every sweep below gets the span index for free rather than finding it again per
/// sample.
fn spans_with_bounds(sk: &Sketch, i: usize) -> Vec<(usize, f64, f64)> {
    let s = &sk.splines[i];
    let mut out = Vec::new();
    for span in DEGREE..s.ctrl.len() {
        let (a, b) = (s.knots[span], s.knots[span + 1]);
        if a < b {
            out.push((span, a, b));
        }
    }
    out
}


fn refine(
    sk: &Sketch,
    i: usize,
    span: usize,
    a: f64,
    pa: (f64, f64),
    b: f64,
    pb: (f64, f64),
    tol: f64,
    depth: u32,
    out: &mut Vec<(f64, f64)>,
) {
    let m = 0.5 * (a + b);
    let pm = eval_on(sk, i, span, m).p;
    let (cx, cy) = (0.5 * (pa.0 + pb.0), 0.5 * (pa.1 + pb.1));
    if depth == 0 || (pm.0 - cx).hypot(pm.1 - cy) <= tol {
        out.push(pb);
        return;
    }
    refine(sk, i, span, a, pa, m, pm, tol, depth - 1, out);
    refine(sk, i, span, m, pm, b, pb, tol, depth - 1, out);
}

/// The curve as a polyline, refined until a chord strays less than `FLATNESS_PX` screen pixels
/// from it.  `unit` is the world length of one screen pixel, exactly as the callouts use it: the
/// core lays the figure out and the front end only strokes what it is handed, so a front end
/// never evaluates a basis function.
pub fn tessellate(sk: &Sketch, i: usize, unit: f64) -> Vec<(f64, f64)> {
    let tol = (unit.abs() * FLATNESS_PX).max(1e-12);
    let spans = spans_with_bounds(sk, i);
    let start = spans.first().map(|&(s, a, _)| eval_on(sk, i, s, a).p).unwrap_or((0.0, 0.0));
    let mut out = vec![start];
    for &(span, a, b) in &spans {
        let (pa, pb) = (eval_on(sk, i, span, a).p, eval_on(sk, i, span, b).p);
        refine(sk, i, span, a, pa, b, pb, tol, MAX_DEPTH, &mut out);
    }
    out
}

/// The curve at a fixed resolution — `per_span` samples of each non-empty span, ends included.
/// What a bounding box or a coarse sweep wants, where an adaptive tessellation would be both
/// more work and less predictable.
pub fn sample(sk: &Sketch, i: usize, per_span: usize) -> Vec<(f64, f64)> {
    samples(sk, i, per_span).map(|(_, p)| p).collect()
}

/// The same walk with the parameter alongside each point, which is what a sweep looking for a
/// *place* on the curve needs.  One implementation: the bounding box, the arc length, the
/// nearest-point basin and the nearest-to-a-line basin are all this walk at different
/// resolutions.  Every sample carries the span it came from, so none of them re-derives it.
pub fn samples(
    sk: &Sketch,
    i: usize,
    per_span: usize,
) -> impl Iterator<Item = (f64, (f64, f64))> + '_ {
    let per_span = per_span.max(1);
    let spans = spans_with_bounds(sk, i);
    let first = spans.first().map(|&(s, a, _)| (a, eval_on(sk, i, s, a).p));
    first.into_iter().chain(spans.into_iter().flat_map(move |(span, a, b)| {
        (1..=per_span).map(move |k| {
            let t = a + (b - a) * k as f64 / per_span as f64;
            (t, eval_on(sk, i, span, t).p)
        })
    }))
}

/// Arc length, from a fixed sampling — a characteristic size, not a measurement.
fn length(sk: &Sketch, i: usize) -> f64 {
    let mut total = 0.0;
    let mut prev: Option<(f64, f64)> = None;
    for (_, p) in samples(sk, i, 8) {
        if let Some(q) = prev {
            total += (p.0 - q.0).hypot(p.1 - q.1);
        }
        prev = Some(p);
    }
    total
}

/// The world length one unit of a curve parameter is worth — the mean speed |C'| over the
/// domain.  What `Param::scale` wants: it makes a step in t comparable to a step in a
/// coordinate, so the minimum-norm update measures motion in world units and a jitter meant for
/// coordinates does not shove a contact across the whole curve.
pub fn speed(sk: &Sketch, i: usize) -> f64 {
    let (t0, t1) = domain(sk, i);
    let l = length(sk, i);
    if l > 0.0 && t1 > t0 {
        l / (t1 - t0)
    } else {
        1.0
    }
}

/// The parameter of the curve point nearest (x, y), and how far that is.  A coarse sweep for
/// the basin — a curve can be nearest a point in several places, and the nearest one is the
/// branch a fresh contact should start on — then Newton on (C(t) − q)·C'(t) = 0 to land on it.
pub fn closest(sk: &Sketch, i: usize, x: f64, y: f64) -> (f64, f64) {
    let (t0, t1) = domain(sk, i);
    let mut best = (t0, f64::INFINITY);
    for (t, p) in samples(sk, i, 16) {
        let d = (p.0 - x).hypot(p.1 - y);
        if d < best.1 {
            best = (t, d);
        }
    }
    let mut t = best.0;
    for _ in 0..24 {
        let f = eval(sk, i, t);
        let (wx, wy) = (f.p.0 - x, f.p.1 - y);
        let g = wx * f.d1.0 + wy * f.d1.1;
        let h = f.d1.0 * f.d1.0 + f.d1.1 * f.d1.1 + wx * f.d2.0 + wy * f.d2.1;
        if h.abs() <= 1e-30 {
            break;
        }
        let step = g / h;
        let nt = (t - step).clamp(t0, t1);
        if (nt - t).abs() <= 1e-14 * (1.0 + t.abs()) {
            t = nt;
            break;
        }
        t = nt;
    }
    let p = point_at(sk, i, t);
    let d = (p.0 - x).hypot(p.1 - y);
    if d <= best.1 {
        (t, d)
    } else {
        best
    }
}

/// The parameter at which the curve comes closest to being tangent to the infinite line through
/// (ax, ay) and (bx, by) — the point of it nearest that line.  Where a fresh tangency starts.
pub fn nearest_to_line(sk: &Sketch, i: usize, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let (dx, dy) = (bx - ax, by - ay);
    let l = dx.hypot(dy);
    if l <= 1e-12 {
        return closest(sk, i, ax, ay).0;
    }
    let mut best = (domain(sk, i).0, f64::INFINITY);
    for (t, p) in samples(sk, i, 32) {
        let d = ((dx * (p.1 - ay) - dy * (p.0 - ax)) / l).abs();
        if d < best.1 {
            best = (t, d);
        }
    }
    best.0
}

/// Distance from (x, y) to the curve — the pick test, and what `distance_between` measures.
pub fn distance_to(sk: &Sketch, i: usize, x: f64, y: f64) -> f64 {
    closest(sk, i, x, y).1
}

/* -- editing the control polygon -------------------------------------------- */

/// The control polygon and knot vector of the cubic B-spline that passes exactly through
/// `pts`, in order — the interpolation problem, which is the one most people mean when they
/// say "a curve through these points".
///
/// Chord-length parameters, an averaged knot vector, and the collocation system `N P = Q`
/// solved once (Piegl & Tiller, global curve interpolation).  It is a *construction*, not a
/// set of constraints: the answer is a control polygon like any other, so the curve it makes
/// drags, constrains and saves exactly as a drawn one does — the same bargain
/// `Sketch::arc_through` strikes for the three-point arc.  A user who wants the curve to *stay*
/// through a point says so with a `PointOnSpline`.
///
/// Returns the control points, the knot vector, and the parameter each given point sits at —
/// that last one matters: the fit *chose* where along the curve each point is, so a contact made
/// from it knows its parameter rather than having to solve for it.
///
/// `None` if there are too few points for a cubic, or if they are too close together to give a
/// parameterisation.
pub fn interpolating_ctrl(
    pts: &[(f64, f64)],
) -> Option<(Vec<(f64, f64)>, Vec<f64>, Vec<f64>)> {
    let m = pts.len();
    let p = DEGREE;
    if m < p + 1 || pts.iter().any(|q| !q.0.is_finite() || !q.1.is_finite()) {
        return None;
    }
    // chord-length parameters: points that are far apart get more of the curve
    let chords: Vec<f64> =
        (1..m).map(|k| (pts[k].0 - pts[k - 1].0).hypot(pts[k].1 - pts[k - 1].1)).collect();
    let total: f64 = chords.iter().sum();
    if !total.is_finite() || total <= 0.0 {
        return None;
    }
    let mut t = vec![0.0; m];
    for k in 1..m {
        t[k] = t[k - 1] + chords[k - 1] / total;
    }
    t[m - 1] = 1.0;
    // averaged knots: each interior knot is the mean of the p parameters it spans, which is
    // what keeps the collocation system well conditioned
    let mut u = vec![0.0; p + 1];
    for j in 1..m.saturating_sub(p) {
        u.push(t[j..j + p].iter().sum::<f64>() / p as f64);
    }
    u.extend(std::iter::repeat_n(1.0, p + 1));
    if !knots_valid(&u, m) {
        return None;
    }
    // N P = Q, one m*m solve per coordinate
    let mut n = vec![0.0; m * m];
    let (mut b, mut d, mut dd) = ([0.0; SPAN_N], [0.0; SPAN_N], [0.0; SPAN_N]);
    for k in 0..m {
        let span = span_index(&u, m, t[k]);
        basis(t[k], &local_knots(&u, span), &mut b, &mut d, &mut dd);
        for a in 0..SPAN_N {
            n[k * m + span - p + a] = b[a];
        }
    }
    let mut out = vec![(0.0, 0.0); m];
    for coord in 0..2 {
        let mut a = n.clone();
        let mut rhs: Vec<f64> =
            pts.iter().map(|q| if coord == 0 { q.0 } else { q.1 }).collect();
        if !crate::linalg::lu_solve(m, &mut a, &mut rhs) {
            return None;
        }
        for k in 0..m {
            if !rhs[k].is_finite() {
                return None;
            }
            if coord == 0 {
                out[k].0 = rhs[k];
            } else {
                out[k].1 = rhs[k];
            }
        }
    }
    Some((out, u, t))
}

/// Give the curve one more control point at `t`, without changing its shape.
///
/// This is Boehm's knot insertion, and shape preservation is the whole point of it: C(t) is
/// *identical* afterwards for every t, so a contact keeps both its parameter and the place on
/// the drawing it sits at.  Splicing a point into the control polygon by hand does not do that,
/// and a curve that squirms when you ask it for another handle is the thing that makes spline
/// editing feel arbitrary.
///
/// `DEGREE - 1` of the control points already there move (to convex combinations of themselves
/// and their neighbours) and one new one appears between them; everything else is untouched.
/// The moved ones keep their identity, so whatever was constrained to them still is — and if
/// one of them *is* constrained, the next solve honours that instead, which is a stronger thing
/// than "keep the shape" and the user said it first.
///
/// Returns the new control Point, or `None` if `t` is not a place a knot can go.
pub fn insert_control(sk: &mut Sketch, i: usize, t: f64) -> Option<usize> {
    if !t.is_finite() {
        return None;
    }
    let (t0, t1) = domain(sk, i);
    // never at the very ends: another knot on top of the clamp raises its multiplicity rather
    // than adding a span, and a control point at the endpoint is what extending is for
    let edge = (t1 - t0) * 1e-6;
    let t = t.clamp(t0 + edge, t1 - edge);
    let n = sk.splines[i].ctrl.len();
    let u = sk.splines[i].knots.clone();
    let ctrl = sk.splines[i].ctrl.clone();
    let k = span_index(&u, n, t);
    if k < DEGREE || k + DEGREE >= u.len() {
        return None;
    }
    // every combination is taken from the original positions, before any of them move
    let mut q: Vec<(usize, (f64, f64))> = Vec::with_capacity(DEGREE);
    for idx in (k + 1 - DEGREE)..=k {
        let den = u[idx + DEGREE] - u[idx];
        let a = if den != 0.0 { (t - u[idx]) / den } else { 0.0 };
        let (x1, y1) = sk.point_xy(ctrl[idx] as usize);
        let (x0, y0) = sk.point_xy(ctrl[idx - 1] as usize);
        q.push((idx, (a * x1 + (1.0 - a) * x0, a * y1 + (1.0 - a) * y0)));
    }
    let &(_, (nx, ny)) = q.last()?;
    let fresh = sk.point(nx, ny, false, &format!("k{}", sk.points.len()));
    for &(idx, (x, y)) in &q[..q.len() - 1] {
        let [px, py] = sk.point_params(ctrl[idx] as usize);
        sk.params[px as usize].value = x;
        sk.params[py as usize].value = y;
    }
    let s = &mut sk.splines[i];
    s.ctrl.insert(k, fresh as u32);
    s.knots.insert(k + 1, t);
    Some(fresh)
}

/* -- keeping a contact on the curve ----------------------------------------- */

/// How many times a solve will re-home its contacts and start again.  Each round either moves a
/// contact onto a different span or pins one to an end of the curve, and there are only so many
/// of either; the cap is what stops a contact that wants to be somewhere it cannot be from
/// looping.
pub const MAX_REHOME: usize = 4;

/// Whether anything in the sketch touches a curve — the guard every path below is behind, so a
/// sketch with no curves in it pays nothing for any of this.
pub fn has_contacts(sk: &Sketch) -> bool {
    sk.constraints.iter().any(|c| c.curve_contact().is_some())
}

/// The span each curve contact currently sits on, by constraint id.
///
/// A compiled system names one span's control points in its columns, so this *is* the part of
/// the topology a curve adds: when it changes, the plan has to be built again.  Comparing it
/// across a solve is how a caller finds out.
pub fn contact_spans(sk: &Sketch) -> BTreeMap<u32, usize> {
    sk.constraints
        .iter()
        .filter_map(|c| {
            let (s, t) = c.curve_contact()?;
            Some((c.id, span_of(sk, s, sk.params[t as usize].value)))
        })
        .collect()
}

/// Bring every curve contact back onto the drawn curve, returning the Params that had to move.
///
/// `t0 <= t <= t1` is a bound, and a least-squares problem has no way to say one.  Left alone a
/// contact will happily walk off the end: the basis is a polynomial and goes on evaluating past
/// the knot vector, so the solver finds a perfectly good tangency on a phantom extension of a
/// curve nobody drew.  Clamping says the bound instead, and the caller then re-solves with the
/// clamped parameters held — which is the active-set answer: the contact sits at the end of the
/// curve and the rest of the geometry accommodates it, or the constraint genuinely cannot hold.
pub fn clamp_contacts(sk: &mut Sketch) -> Vec<u32> {
    let mut moved = Vec::new();
    for i in 0..sk.constraints.len() {
        let Some((s, t)) = sk.constraints[i].curve_contact() else { continue };
        let (t0, t1) = domain(sk, s);
        let v = sk.params[t as usize].value;
        let c = if v.is_finite() { v.clamp(t0, t1) } else { 0.5 * (t0 + t1) };
        if c != v {
            sk.params[t as usize].value = c;
            moved.push(t);
        }
    }
    moved
}
