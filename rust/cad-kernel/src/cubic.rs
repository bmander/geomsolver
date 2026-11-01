//! **The cubic B-spline basis** (#721 F2c): the Cox–de Boor recurrence run over jets (a value and
//! its first three derivatives), a rational curve's weights, clamped knot vectors and their
//! checks, the span a parameter falls in, and Piegl and Tiller's global interpolation through
//! points. What a drawing's curves are read by, and what a curve sampled off a sweep is fitted
//! with.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::util::json::Json;

/// The one degree the kernels are compiled for.  The basis below is written for general `p`; a
/// second degree costs a kernel pair and a `CKind` that selects it, which is why the model
/// stores a knot vector (any shape of cubic) but not a degree.
pub const DEGREE: usize = 3;

/// Control points one span reads — the number of non-zero basis functions, and equally the
/// fewest a curve can be drawn with.  `knots_valid`, `Sketch::min_children` and the registry's
/// `curve.minCtrl` all read it here, so what `spline_with` will accept, what survives a
/// deletion and what a front end tells the user cannot disagree.
pub const SPAN_N: usize = DEGREE + 1;
pub const MIN_CTRL: usize = SPAN_N;

/// The local knot window one span needs: u[span-p ..= span+p+1].
pub const SPAN_K: usize = 2 * DEGREE + 2;

/* -- second-order jets ------------------------------------------------------ */

/// A value and its first three derivatives in t.  The Cox–de Boor recurrence is a handful of
/// sums and products of these, so differentiating the basis is the recurrence itself run over
/// jets rather than a second algorithm to keep in step with the first.
///
/// Three, not two, because curvature is written in C″ and a curvature residual differentiated
/// in t therefore needs C‴.  The extra order costs four multiplies in `mul` and nothing at all
/// to the kernels that ignore it.
#[derive(Clone, Copy, Debug)]
struct Jet {
    v: f64,
    d: f64,
    dd: f64,
    d3: f64,
}

impl Jet {
    const ZERO: Jet = Jet { v: 0.0, d: 0.0, dd: 0.0, d3: 0.0 };
    const ONE: Jet = Jet { v: 1.0, d: 0.0, dd: 0.0, d3: 0.0 };

    /// `t - c`, as a function of t.
    fn rise(t: f64, c: f64) -> Jet {
        Jet { v: t - c, d: 1.0, dd: 0.0, d3: 0.0 }
    }

    /// `c - t`, as a function of t.
    fn fall(c: f64, t: f64) -> Jet {
        Jet { v: c - t, d: -1.0, dd: 0.0, d3: 0.0 }
    }

    fn add(self, o: Jet) -> Jet {
        Jet {
            v: self.v + o.v,
            d: self.d + o.d,
            dd: self.dd + o.dd,
            d3: self.d3 + o.d3,
        }
    }

    fn mul(self, o: Jet) -> Jet {
        Jet {
            v: self.v * o.v,
            d: self.d * o.v + self.v * o.d,
            dd: self.dd * o.v + 2.0 * self.d * o.d + self.v * o.dd,
            d3: self.d3 * o.v + 3.0 * self.dd * o.d + 3.0 * self.d * o.dd + self.v * o.d3,
        }
    }

    fn scale(self, s: f64) -> Jet {
        Jet { v: self.v * s, d: self.d * s, dd: self.dd * s, d3: self.d3 * s }
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
             dd: &mut [f64; SPAN_N], d3: &mut [f64; SPAN_N]) {
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
        d3[a] = cur[a].d3;
    }
}

/// The basis `basis` filled, made rational by the span's weights `w`: `Rᵢ = Nᵢ wᵢ / Σ Nⱼ wⱼ` and its
/// first three derivatives in t, by the quotient rule.  A curve over fixed weights is still linear
/// in its control points, `C = Σ Rᵢ Pᵢ`, so every contact kernel reads `R` where it read `B` and
/// nothing else changes.  Weights all 1 leave the basis exactly as it was (no division), so a
/// polynomial spline solves to the same bits.
pub fn weigh(w: &[f64; SPAN_N], b: &mut [f64; SPAN_N], d: &mut [f64; SPAN_N],
             dd: &mut [f64; SPAN_N], d3: &mut [f64; SPAN_N]) {
    if w.iter().all(|&x| x == 1.0) {
        return;
    }
    let mut s = [0.0; 4];
    for a in 0..SPAN_N {
        b[a] *= w[a];
        d[a] *= w[a];
        dd[a] *= w[a];
        d3[a] *= w[a];
        s[0] += b[a];
        s[1] += d[a];
        s[2] += dd[a];
        s[3] += d3[a];
    }
    for a in 0..SPAN_N {
        let r0 = b[a] / s[0];
        let r1 = (d[a] - r0 * s[1]) / s[0];
        let r2 = (dd[a] - 2.0 * r1 * s[1] - r0 * s[2]) / s[0];
        let r3 = (d3[a] - 3.0 * r2 * s[1] - 3.0 * r1 * s[2] - r0 * s[3]) / s[0];
        (b[a], d[a], dd[a], d3[a]) = (r0, r1, r2, r3);
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
    n >= MIN_CTRL
        && u.len() == n + DEGREE + 1
        && u.iter().all(|x| x.is_finite())
        && u.windows(2).all(|w| w[0] <= w[1])
        && u[DEGREE] < u[n]
}

/// The knot vector for a control polygon that has lost the control points at `gone`.
///
/// One interior knot goes with each, so a curve shaped by knot insertion degrades back toward
/// what it was rather than being re-spaced from scratch.  Note that this is *not* the inverse
/// of `insert_control`: the curve changes shape, which it must, and dropping a knot
/// re-parameterises what is left — so a contact keeps its number but not the place on the
/// drawing that number named.  A free one solves its way back; a pinned one now names
/// somewhere slightly else, which is the price of having said where along on a curve that has
/// since lost a span.  Falls back to the clamped uniform vector when what is
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

/// What a spline contact's kernel is handed per instance: the span's knot window, then the weights
/// of its control points.
pub const SPAN_C: usize = SPAN_K + SPAN_N;

/// `u[span-p ..= span+p+1]` — what `basis` reads.
pub fn local_knots(u: &[f64], span: usize) -> [f64; SPAN_K] {
    let mut out = [0.0; SPAN_K];
    out.copy_from_slice(&u[span - DEGREE..span + DEGREE + 2]);
    out
}

/// `w[span-p ..= span]` — what `weigh` reads, the weights of the control points `basis` fills —
/// all 1 for a polynomial spline.
pub fn local_weights(w: Option<&[f64]>, span: usize) -> [f64; SPAN_N] {
    let mut out = [1.0; SPAN_N];
    if let Some(w) = w {
        out.copy_from_slice(&w[span - DEGREE..=span]);
    }
    out
}

/// Whether `w` may weigh `n` control points: one each, every one finite and positive (a weight at
/// or below zero lets the curve leave its control polygon's hull, or pass through infinity).
pub fn weights_valid(w: &[f64], n: usize) -> bool {
    w.len() == n && w.iter().all(|&x| x.is_finite() && x > 0.0)
}

/* -- a spline's geometry ---------------------------------------------------- */

/// A point on the curve with its first two derivatives in t.
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub p: (f64, f64),
    pub d1: (f64, f64),
    pub d2: (f64, f64),
}

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
    if m < DEGREE + 1 || pts.iter().any(|q| !q.0.is_finite() || !q.1.is_finite()) {
        return None;
    }
    // chord-length parameters: points that are far apart get more of the curve
    let chords: Vec<f64> =
        (1..m).map(|k| (pts[k].0 - pts[k - 1].0).dhypot(pts[k].1 - pts[k - 1].1)).collect();
    let total: f64 = chords.iter().sum();
    if !total.is_finite() || total <= 0.0 {
        return None;
    }
    let mut t = vec![0.0; m];
    for k in 1..m {
        t[k] = t[k - 1] + chords[k - 1] / total;
    }
    t[m - 1] = 1.0;
    interpolating_ctrl_at(pts, t)
}

/// `interpolating_ctrl` at parameters the caller chose (increasing, from 0 to 1): what a fit
/// whose parameter must follow another curve's asks for.
pub fn interpolating_ctrl_at(
    pts: &[(f64, f64)],
    t: Vec<f64>,
) -> Option<(Vec<(f64, f64)>, Vec<f64>, Vec<f64>)> {
    let m = pts.len();
    let p = DEGREE;
    if m < p + 1 || t.len() != m || pts.iter().any(|q| !q.0.is_finite() || !q.1.is_finite()) {
        return None;
    }
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
    let (mut b, mut d, mut dd, mut d3) =
        ([0.0; SPAN_N], [0.0; SPAN_N], [0.0; SPAN_N], [0.0; SPAN_N]);
    for k in 0..m {
        let span = span_index(&u, m, t[k]);
        basis(t[k], &local_knots(&u, span), &mut b, &mut d, &mut dd, &mut d3);
        for a in 0..SPAN_N {
            n[k * m + span - p + a] = b[a];
        }
    }
    // one factorisation, two right-hand sides
    let mut xs: Vec<f64> = pts.iter().map(|q| q.0).collect();
    let mut ys: Vec<f64> = pts.iter().map(|q| q.1).collect();
    let mut piv = Vec::with_capacity(m);
    if !crate::util::linalg::lu_factor(m, &mut n, &mut piv) {
        return None;
    }
    crate::util::linalg::lu_apply(m, &n, &piv, &mut xs);
    crate::util::linalg::lu_apply(m, &n, &piv, &mut ys);
    if xs.iter().chain(&ys).any(|v| !v.is_finite()) {
        return None;
    }
    Some((xs.into_iter().zip(ys).collect(), u, t))
}

/// A planar stretch read by `sweep` (`n` chords over `[lo, hi]`, `lo < hi`) over `(a, b)`
/// (decreasing where the stretch runs against the parameter) as a cubic B-spline within `tol`,
/// its ends put exactly at `ends` where given: its poles, knots and the error measured.
pub fn fit_sampled(sweep: &dyn Fn(f64,f64,usize) -> Vec<(f64,f64)>,(a,b): (f64,f64),ends: Option<((f64,f64),(f64,f64))>,tol: f64)
    -> Option<(Vec<(f64,f64)>,Vec<f64>,f64)> {
    use crate::nurbs::BSpline;
    let (lo,hi) = (a.min(b),a.max(b));
    let mut n = 8;
    loop {
        let mut fine = sweep(lo,hi,2*n);
        if a > b { fine.reverse(); }
        let mut through: Vec<(f64,f64)> = fine.iter().step_by(2).copied().collect();
        let last = through.len()-1;
        if let Some(ends) = ends {
            (through[0],through[last]) = ends;
        }
        let fractions = (0..=n).map(|k| k as f64/n as f64).collect();
        let (ctrl,knots,t) = interpolating_ctrl_at(&through,fractions)?;
        let s = BSpline::new(DEGREE,knots.clone(),ctrl.iter().map(|&(x,y)| [x,y,0.]).collect()).ok()?;
        // each halfway sample's distance to the fit, from the fit's own halfway parameter
        let err = (0..n).map(|k| {
            let q = [fine[2*k+1].0,fine[2*k+1].1,0.];
            let mut u = (t[k]+t[k+1])/2.;
            for _ in 0..8 {
                let (x,d,dd) = s.d2(u);
                let e = crate::util::space::sub(x,q);
                let h = crate::util::space::dot(d,d)+crate::util::space::dot(e,dd);
                if !(h > 0.) { break }
                u = (u-crate::util::space::dot(e,d)/h).clamp(t[k],t[k+1]);
            }
            crate::util::space::distance(s.point(u),q)
        }).fold(0.,f64::max);
        if err <= tol { return Some((ctrl,knots,err)) }
        n *= 2;
        if n > 2048 { return None }
    }
}

/// A profile's cubic B-spline edge as a recipe writes it: its knots, its poles lifted into space
/// (millimetres) and the fit it was measured within (mm).
pub fn bspline_edge(poles: &[(f64,f64)],knots: &[f64],fit_mm: f64,lift: &dyn Fn((f64,f64)) -> [f64;3]) -> Json {
    crate::util::json::object([("kind","bspline".into()),("degree",DEGREE.into()),
        ("knots",Json::Arr(knots.iter().map(|&k| k.into()).collect())),
        ("poles",Json::Arr(poles.iter().map(|&c| Json::Arr(lift(c).iter().map(|&x| x.into()).collect())).collect())),
        ("fit",fit_mm.into())])
}
