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

/// The one degree the kernels are compiled for.  The basis below is written for general `p`; a
/// second degree costs a kernel pair and a `CKind` that selects it, which is why the model
/// stores a knot vector (any shape of cubic) but not a degree.
pub const DEGREE: usize = 3;

/// Control points one span reads — the number of non-zero basis functions.
pub const SPAN_N: usize = DEGREE + 1;

/// The local knot window one span needs: u[span-p ..= span+p+1].
pub const SPAN_K: usize = 2 * DEGREE + 2;

/// Highest degree `basis` will evaluate — the size of its working buffers.
pub const MAX_DEGREE: usize = 5;

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

/// The `p+1` basis functions that are non-zero on one span, and their first two derivatives in
/// t.  `lk` is the span's local knot window — `u[span-p ..= span+p+1]`, `2p+2` values — and
/// `b`, `d` and `dd` are filled for the control points `P[span-p ..= span]`, in that order.
///
/// This is Cox–de Boor run over jets.  A zero denominator is a knot repeated enough times to
/// collapse a span, and the basis function it divides is zero there: the convention is that the
/// whole term is zero, which is also what keeps a clamped knot vector (where `u[p] == u[0]`)
/// from putting a NaN in the Jacobian.
pub fn basis(p: usize, t: f64, lk: &[f64], b: &mut [f64], d: &mut [f64], dd: &mut [f64]) {
    debug_assert!(p <= MAX_DEGREE && lk.len() >= 2 * p + 2);
    // cur[a] holds N_{span-p+a, deg}; the slot at p+1 stays zero so the recurrence can read it
    let mut cur = [Jet::ZERO; MAX_DEGREE + 2];
    let mut next = [Jet::ZERO; MAX_DEGREE + 2];
    cur[p] = Jet::ONE; // N_{span,0} = 1 on this span, every other degree-0 function 0
    for deg in 1..=p {
        for slot in next.iter_mut() {
            *slot = Jet::ZERO;
        }
        for a in (p - deg)..=p {
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
    for a in 0..=p {
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
    let span = span_index(&s.knots, s.ctrl.len(), t);
    let lk = local_knots(&s.knots, span);
    let (mut b, mut d, mut dd) = ([0.0; SPAN_N], [0.0; SPAN_N], [0.0; SPAN_N]);
    basis(DEGREE, t, &lk, &mut b, &mut d, &mut dd);
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

/// The parameter values that bound the non-empty spans — every knot is a vertex of the drawn
/// polyline, so a repeated knot's corner is drawn as a corner and not smoothed across.
fn span_bounds(sk: &Sketch, i: usize) -> Vec<f64> {
    let s = &sk.splines[i];
    let (t0, t1) = domain(sk, i);
    let mut out = vec![t0];
    for k in DEGREE + 1..s.ctrl.len() {
        let u = s.knots[k];
        if u > t0 && u < t1 && *out.last().unwrap() < u {
            out.push(u);
        }
    }
    out.push(t1);
    out
}

fn refine(
    sk: &Sketch,
    i: usize,
    a: f64,
    pa: (f64, f64),
    b: f64,
    pb: (f64, f64),
    tol: f64,
    depth: u32,
    out: &mut Vec<(f64, f64)>,
) {
    let m = 0.5 * (a + b);
    let pm = point_at(sk, i, m);
    let (cx, cy) = (0.5 * (pa.0 + pb.0), 0.5 * (pa.1 + pb.1));
    if depth == 0 || (pm.0 - cx).hypot(pm.1 - cy) <= tol {
        out.push(pb);
        return;
    }
    refine(sk, i, a, pa, m, pm, tol, depth - 1, out);
    refine(sk, i, m, pm, b, pb, tol, depth - 1, out);
}

/// The curve as a polyline, refined until a chord strays less than `FLATNESS_PX` screen pixels
/// from it.  `unit` is the world length of one screen pixel, exactly as the callouts use it: the
/// core lays the figure out and the front end only strokes what it is handed, so a front end
/// never evaluates a basis function.
pub fn tessellate(sk: &Sketch, i: usize, unit: f64) -> Vec<(f64, f64)> {
    let tol = (unit.abs() * FLATNESS_PX).max(1e-12);
    let bounds = span_bounds(sk, i);
    let mut out = vec![point_at(sk, i, bounds[0])];
    for w in bounds.windows(2) {
        let (pa, pb) = (point_at(sk, i, w[0]), point_at(sk, i, w[1]));
        refine(sk, i, w[0], pa, w[1], pb, tol, MAX_DEPTH, &mut out);
    }
    out
}

/// The curve at a fixed resolution — `per_span` samples of each non-empty span, ends included.
/// What a bounding box or a coarse sweep wants, where an adaptive tessellation would be both
/// more work and less predictable.
pub fn sample(sk: &Sketch, i: usize, per_span: usize) -> Vec<(f64, f64)> {
    let per_span = per_span.max(1);
    let bounds = span_bounds(sk, i);
    let mut out = vec![point_at(sk, i, bounds[0])];
    for w in bounds.windows(2) {
        for k in 1..=per_span {
            let t = w[0] + (w[1] - w[0]) * k as f64 / per_span as f64;
            out.push(point_at(sk, i, t));
        }
    }
    out
}

/// Arc length, from a fixed sampling — a characteristic size, not a measurement.
pub fn length(sk: &Sketch, i: usize) -> f64 {
    let bounds = span_bounds(sk, i);
    let mut total = 0.0;
    for w in bounds.windows(2) {
        let mut prev = point_at(sk, i, w[0]);
        for k in 1..=8 {
            let t = w[0] + (w[1] - w[0]) * k as f64 / 8.0;
            let p = point_at(sk, i, t);
            total += (p.0 - prev.0).hypot(p.1 - prev.1);
            prev = p;
        }
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
    let bounds = span_bounds(sk, i);
    let (t0, t1) = (bounds[0], *bounds.last().unwrap());
    let mut best = (t0, f64::INFINITY);
    for w in bounds.windows(2) {
        for k in 0..=16 {
            let t = w[0] + (w[1] - w[0]) * k as f64 / 16.0;
            let p = point_at(sk, i, t);
            let d = (p.0 - x).hypot(p.1 - y);
            if d < best.1 {
                best = (t, d);
            }
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
    let bounds = span_bounds(sk, i);
    let mut best = (bounds[0], f64::INFINITY);
    for w in bounds.windows(2) {
        for k in 0..=32 {
            let t = w[0] + (w[1] - w[0]) * k as f64 / 32.0;
            let p = point_at(sk, i, t);
            let d = ((dx * (p.1 - ay) - dy * (p.0 - ax)) / l).abs();
            if d < best.1 {
                best = (t, d);
            }
        }
    }
    best.0
}

/// Distance from (x, y) to the curve — the pick test, and what `distance_between` measures.
pub fn distance_to(sk: &Sketch, i: usize, x: f64, y: f64) -> f64 {
    closest(sk, i, x, y).1
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
pub fn contact_spans(sk: &Sketch) -> Vec<(u32, usize)> {
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

/// Re-read every contact's `Param::scale` from the curve it is on.  The scale preconditions the
/// solver's columns, and a curve that has been dragged to twice its length has twice the speed;
/// left stale it costs convergence rate, never correctness, so this runs where a system is being
/// built anyway rather than on every frame.
pub fn refresh_scales(sk: &mut Sketch) {
    for i in 0..sk.constraints.len() {
        let Some((s, t)) = sk.constraints[i].curve_contact() else { continue };
        sk.params[t as usize].scale = speed(sk, s);
    }
}
