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
//! B-spline, and so every Bézier (a clamped span with no interior knots) — and, over fixed
//! weights, every rational one, `C = Σ Rᵢ Pᵢ` with `Rᵢ = Bᵢ wᵢ / Σ Bⱼ wⱼ` (`weigh`): a conic
//! exactly.  That is the whole
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

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::model::{EntKind, EntRef, Sketch};
use std::collections::BTreeMap;

/// The cubic B-spline basis, its knots and its interpolation are the kernel's (`brep::cubic`).
pub use crate::brep::cubic::{Frame,DEGREE,SPAN_N,MIN_CTRL,SPAN_K,SPAN_C,basis,weigh,clamped_uniform,knots_valid,knots_without,span_index,local_knots,local_weights,weights_valid,interpolating_ctrl,interpolating_ctrl_at};

/// How far a tessellated chord may stray from the curve, in screen pixels.
pub const FLATNESS_PX: f64 = 0.3;

/// Recursion cap per span, so a pathological curve cannot make an unbounded polyline.
const MAX_DEPTH: u32 = 10;

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
    let (mut b, mut d, mut dd, mut d3) =
        ([0.0; SPAN_N], [0.0; SPAN_N], [0.0; SPAN_N], [0.0; SPAN_N]);
    basis(t, &lk, &mut b, &mut d, &mut dd, &mut d3);
    weigh(&local_weights(s.weights.as_deref(), span), &mut b, &mut d, &mut dd, &mut d3);
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
    // the quarter points as well: a cubic is symmetric about its inflection, so one centred on
    // the stretch has its middle exactly on the chord however far the rest of it strays
    let quarters = || [0.25, 0.75].iter().all(|&f| {
        let (q, c) = (eval_on(sk, i, span, a + (b - a) * f).p, (pa.0 + (pb.0 - pa.0) * f, pa.1 + (pb.1 - pa.1) * f));
        (q.0 - c.0).dhypot(q.1 - c.1) <= tol
    });
    if depth == 0 || ((pm.0 - cx).dhypot(pm.1 - cy) <= tol && quarters()) {
        out.push(pb);
        return;
    }
    refine(sk, i, span, a, pa, m, pm, tol, depth - 1, out);
    refine(sk, i, span, m, pm, b, pb, tol, depth - 1, out);
}

/// The world length a chord may stray from its curve at `unit` (the world length of one screen
/// pixel): `FLATNESS_PX` on the screen, whatever the zoom.  The one conversion, so a spline and
/// a circle folded into the box are refined to the same rule.
pub fn flatness(unit: f64) -> f64 {
    (unit.abs() * FLATNESS_PX).max(1e-12)
}

/// The curve as a polyline, refined until a chord strays less than `FLATNESS_PX` screen pixels
/// from it.  `unit` is the world length of one screen pixel, exactly as the callouts use it: the
/// core lays the figure out and the front end only strokes what it is handed, so a front end
/// never evaluates a basis function.
pub fn tessellate(sk: &Sketch, i: usize, unit: f64) -> Vec<(f64, f64)> {
    let tol = flatness(unit);
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
            total += (p.0 - q.0).dhypot(p.1 - q.1);
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
        let d = (p.0 - x).dhypot(p.1 - y);
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
    let d = (p.0 - x).dhypot(p.1 - y);
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
    let l = dx.dhypot(dy);
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
    // every combination is taken from the original positions, before any of them move — a
    // rational curve's in homogeneous coordinates `(w x, w y, w)`, each new point the combination
    // over its new weight
    let lo = k + 1 - DEGREE;
    let w = sk.splines[i].weights.clone();
    let mut fresh_w: Vec<f64> = Vec::new();
    let q: Vec<(f64, f64)> = (lo..=k)
        .map(|idx| {
            let den = u[idx + DEGREE] - u[idx];
            let a = if den != 0.0 { (t - u[idx]) / den } else { 0.0 };
            let (x1, y1) = sk.point_xy(ctrl[idx] as usize);
            let (x0, y0) = sk.point_xy(ctrl[idx - 1] as usize);
            let Some(w) = &w else { return (a * x1 + (1.0 - a) * x0, a * y1 + (1.0 - a) * y0) };
            let (w1, w0) = (w[idx], w[idx - 1]);
            let wq = a * w1 + (1.0 - a) * w0;
            fresh_w.push(wq);
            ((a * w1 * x1 + (1.0 - a) * w0 * x0) / wq, (a * w1 * y1 + (1.0 - a) * w0 * y0) / wq)
        })
        .collect();
    // the last combination becomes the new control point; the rest move the ones already there
    let (nx, ny) = q[DEGREE - 1];
    let fresh = sk.point(nx, ny, false, &format!("k{}", sk.points.len()));
    for (idx, &(x, y)) in (lo..k).zip(&q) {
        let [px, py] = sk.point_params(ctrl[idx] as usize);
        sk.params[px as usize].value = x;
        sk.params[py as usize].value = y;
    }
    let s = &mut sk.splines[i];
    s.ctrl.insert(k, fresh as u32);
    s.knots.insert(k + 1, t);
    if let Some(w) = &mut s.weights {
        w.splice(lo..k, fresh_w[..DEGREE - 1].iter().copied());
        w.insert(k, fresh_w[DEGREE - 1]);
    }
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
        // a contact of either family: a spline's parameter runs over its knots, a curve
        // family's over the `over (a, b)` its instance declared — and a point constrained to a
        // quarter arc drawn `over (0, 90)` sat at 270°, off the drawn curve, with the
        // diagnosis calling the drawing well determined (#43.10)
        let c = &sk.constraints[i];
        let on_spline = c.curve_contact().map(|(s, t)| (EntRef::spline(s), t));
        // and a fraction of an arc's turn, which is on the arc from 0 to 1 (#248); a line's runs
        // on past its ends, as `coincident` on a line does, so it has no domain below
        let found = on_spline.or_else(|| c.family_contact()).or_else(|| c.fraction_contact());
        let Some((e, t)) = found else {
            continue;   // an ellipse's parameter is an angle, and wraps
        };
        // a family's instance may run its interval either way (`over (-u0, -u1)` is a flank
        // unwinding the other way), so the bounds are ordered before anything is held to them
        let (t0, t1) = match e.kind {
            EntKind::Spline => domain(sk, e.i()),
            EntKind::Arc => (0.0, 1.0),
            EntKind::Curve => {
                let (a, b) = sk.curve_domain(e.i());
                (a.min(b), a.max(b))
            }
            _ => continue,
        };
        if sk.params[t as usize].fixed {
            continue;   // pinned: somebody said where along, and the solver is not to argue
        }
        let v = sk.params[t as usize].value;
        // a closed curve has a seam and no end: a contact past it is the same point a turn
        // round, moved back onto the interval and left free
        if e.kind == EntKind::Curve && v.is_finite() && (v < t0 || v > t1) && sk.curve_closed(e.i()) {
            sk.params[t as usize].value = t0 + (v - t0).rem_euclid(t1 - t0);
            continue;
        }
        let c = if v.is_finite() { v.clamp(t0, t1) } else { 0.5 * (t0 + t1) };
        if c != v {
            sk.params[t as usize].value = c;
            // held for the retry only when it was *materially* off the end.  A contact that
            // belongs at an endpoint — a flank's roll at the root circle — lands a rounding
            // error past it, and pinning that would recompile the system for nothing: a
            // recompile forgets every remembered trace pose, and a traced gear re-walking a
            // hundred marches from home did not come back to the same drawing.
            if (c - v).abs() > 1e-9 * (t1 - t0).abs().max(1e-300) {
                moved.push(t);
            }
        }
    }
    moved
}
