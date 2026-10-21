//! **A curve stated by its energy** (#144): `rope := curve(a, b)` with `rope minimizes
//! integral(…)` is the solution of that energy's Euler–Lagrange equation, integrated from `a`
//! and landing on `b`, of the length the drawing gives it — not a spline bent towards it.
//!
//! The curve is a function of its ends and its length: its shape is solved inside it, a small
//! boundary-value Newton (`shoot`) over the flow the energy makes (`flow`), and what the drawing
//! asks of it — a point, the tangent, the curvature, their derivatives in the ends and the
//! length — is read off that solution, the derivatives by the implicit function theorem from the
//! Jacobian the solve ended on.  So the curve adds no unknown to the drawing: what it hangs
//! between and how long it is are the drawing's, and its shape is not.  A held point it passes
//! (a peg) is part of the problem: the curve is two arcs there, meeting at a corner.
//!
//! Its kernels are the curve contacts every curve has (`PointOnCurve`, `CurveTangentLine`,
//! `CurveCurvature`), over the columns `[u, a, b, L]`; this module gives them `C` and its
//! derivatives in the shape `locus::Val`/`Frame` carry, memoised per contact.

pub mod flow;
pub mod shoot;

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::locus::{Frame, Val};
use crate::tape::MAX_VARS;
use crate::variational::Extremum;
pub use flow::Lagrangian;
pub use shoot::{Ends, Shape, Stop};

/* -- the problem as a kernel's constants -------------------------------------------------- */

/// A curve's problem, as its contacts' constants carry it: the Lagrangian, then the pegs (each
/// where it is), then how many lines it touches — whose points are its columns (`Ends::lines`),
/// not constants, so a line the drawing does not hold moves with the solve.
pub fn write(lag: &Lagrangian, pegs: &[[f64; 2]], slides: usize, out: &mut Vec<f64>) {
    lag.write(out);
    out.push(pegs.len() as f64);
    for p in pegs {
        out.extend_from_slice(p);
    }
    out.push(slides as f64);
}

/// How many numbers `write` writes.
pub fn width(lag: &Lagrangian, pegs: usize) -> usize {
    3 + lag.tapes.iter().map(Vec::len).sum::<usize>() + 2 + 2 * pegs
}

/// What `write` wrote: the Lagrangian and the stops, pegs first, then each slide.
pub fn read(k: &[f64]) -> Option<(Lagrangian, Vec<Stop>)> {
    let (lag, rest) = Lagrangian::read(k)?;
    let n = *rest.first()? as usize;
    let at = |i: usize| rest.get(i).copied();
    let mut stops = (0..n).map(|i| Some(Stop::Peg([at(1 + 2 * i)?, at(2 + 2 * i)?]))).collect::<Option<Vec<_>>>()?;
    stops.extend((0..at(1 + 2 * n)? as usize).map(Stop::Slide));
    Some((lag, stops))
}

/// How many lines the problem `k` touches — how many of its columns past the length there are,
/// four a line.
fn slides_in(k: &[f64]) -> Option<usize> {
    let (_, rest) = Lagrangian::read(k)?;
    let n = *rest.first()? as usize;
    Some(*rest.get(1 + 2 * n)? as usize)
}

/// The ends a contact's columns (`a`, `b`, the length, the lines' points) come to, where there are
/// enough of them.
fn ends_in(k: &[f64], theta: &[f64]) -> Option<Ends> {
    let s = slides_in(k)?;
    (theta.len() >= 5 + 4 * s).then(|| Ends::of(theta, s))
}

/* -- the memo ----------------------------------------------------------------------------- */

thread_local! {
    /// The last shapes each curve's problem was solved to, by the problem's constants — what a
    /// contact, the transversality row and the drawing all read, so a curve is solved once for
    /// all of them.  Asked again at the same ends a shape is the answer, at neighbouring ones the
    /// nearest is where the next solve starts.  By content, not by where the constants live, so a
    /// recompile keeps every curve warm; a few a problem, for two curves under one energy and a
    /// rejected trial step beside the pose it was tried from.
    static SEEN: std::cell::RefCell<std::collections::BTreeMap<Vec<u64>, Vec<Shape>>> =
        std::cell::RefCell::new(std::collections::BTreeMap::new());
}

const SEEN_MAX: usize = 1024;
const KEEP: usize = 4;

/// The shape a curve's problem (its constants, `write`'s) and ends come to, warm from the
/// nearest remembered — with the problem's Lagrangian, read off the constants.
pub fn shape_for(k: &[f64], ends: &Ends) -> Option<(Lagrangian, Shape)> {
    let (lag, stops) = read(k)?;
    if stops.iter().filter(|s| s.slides()).count() != ends.lines.len() {
        return None;
    }
    let key: Vec<u64> = k.iter().map(|v| v.to_bits()).collect();
    let o = ends.outer();
    let far = |sh: &Shape| sh.ends.outer().iter().zip(&o).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
    let prev = SEEN.with(|s| {
        let s = s.borrow();
        let v = s.get(&key)?;
        v.iter().min_by(|a, b| far(a).total_cmp(&far(b))).cloned()
    });
    if let Some(p) = prev.as_ref().filter(|p| p.ends == *ends) {
        return Some((lag, p.clone()));
    }
    let sh = shoot::solve(&lag, ends, &stops, prev.as_ref())?;
    SEEN.with(|s| {
        let mut s = s.borrow_mut();
        if s.len() >= SEEN_MAX {
            s.clear();
        }
        let v = s.entry(key).or_default();
        v.insert(0, sh.clone());
        v.truncate(KEEP);
    });
    Some((lag, sh))
}

/* -- reading a shape ---------------------------------------------------------------------- */

/// `C`, `C'`, `C''` at `u` and — where asked — the gradients of the first two over `[u, a, b, L,
/// lines…]`.
struct Read {
    c: [[f64; 2]; 3],
    g: [[[f64; MAX_VARS]; 2]; 2],
    /// How many of the gradients' columns there are: `u` and the outer ones.
    n: usize,
}

fn read_at(lag: &Lagrangian, sh: &Shape, u: f64, deriv: bool) -> Option<Read> {
    let at = shoot::at(lag, sh, u, deriv)?;
    let len = sh.ends.len;
    let (sn, cs) = at.theta.dsin_cos();
    let tz = at.point.theta_z();
    // dθ/ds, and dθ over the columns
    let rate: f64 = (0..4).map(|i| tz[i] * at.f[i]).sum();
    let n = at.stride;
    if n > MAX_VARS {
        return None;
    }
    let mut g = [[[0.0f64; MAX_VARS]; 2]; 2];
    if deriv {
        for j in 0..n {
            let dth: f64 = (0..4).map(|i| tz[i] * at.dz[i * n + j]).sum();
            g[0][0][j] = at.dz[j];
            g[0][1][j] = at.dz[n + j];
            g[1][0][j] = -len * sn * dth;
            g[1][1][j] = len * cs * dth;
        }
        g[1][0][5] += cs;
        g[1][1][5] += sn;
    }
    let k2 = len * len * rate;
    Some(Read { c: [[at.z[0], at.z[1]], [len * cs, len * sn], [-k2 * sn, k2 * cs]], g, n })
}

/// `C'''` at `u`, by a central difference of `C''` in the parameter (the integrand's third
/// derivatives are not to be had), one-sided at the ends.
fn third(lag: &Lagrangian, sh: &Shape, u: f64) -> Option<[f64; 2]> {
    const DU: f64 = 1e-5;
    let (lo, hi) = ((u - DU).max(0.0), (u + DU).min(1.0));
    let a = read_at(lag, sh, lo, false)?.c[2];
    let b = read_at(lag, sh, hi, false)?.c[2];
    Some([(b[0] - a[0]) / (hi - lo), (b[1] - a[1]) / (hi - lo)])
}

/// A contact's reading at `u`: the shape its constants and columns come to, and what it is there.
fn frame_at(k: &[f64], u: f64, theta: &[f64], deriv: bool) -> Option<(Lagrangian, Shape, Read)> {
    let ends = ends_in(k, theta)?;
    let (lag, sh) = shape_for(k, &ends)?;
    let r = read_at(&lag, &sh, u, deriv)?;
    Some((lag, sh, r))
}

fn val_of(lag: &Lagrangian, sh: &Shape, r: &Read, u: f64, need: u8) -> Val {
    let mut v = Val::default();
    v.x = r.c[0][0];
    v.y = r.c[0][1];
    for j in 0..r.n {
        v.dx[j] = r.g[0][0][j];
        v.dy[j] = r.g[0][1][j];
    }
    // `C'` is the gradient's `u` column, there whether the rest was asked for or not
    v.dx[0] = r.c[1][0];
    v.dy[0] = r.c[1][1];
    v.d2 = r.c[2];
    v.orders = 2;
    if need >= 3 {
        if let Some(d3) = third(lag, sh, u) {
            v.d3 = d3;
            v.orders = 3;
        }
    }
    v.ok = true;
    v
}

/// A contact's evaluation (`kernels::body_val`'s for this body): `C` and its derivatives in `u`
/// to `need`, and — where `gradient` — the gradient of `C` over `[u, θ…]` (`θ…` = `a`, `b`, `L`
/// and the points of the lines it touches).
/// NaN where the shape cannot be had.
pub fn kernel_eval(k: &[f64], u: f64, theta: &[f64], need: u8, gradient: bool) -> Val {
    match frame_at(k, u, theta, gradient) {
        Some((lag, sh, r)) => val_of(&lag, &sh, &r, u, need),
        None => Val::default(),
    }
}

/// A contact's frame (`kernels::curve_frame`'s): the evaluation, the gradient of `C'`, and, asked
/// for (`need` 3), of `C''` — along `u` that is `C'''`, along the ends and the length a central
/// difference of solves warm from this one.
pub fn kernel_frame(k: &[f64], u: f64, theta: &[f64], need: u8) -> Frame {
    let mut d1 = [[0.0f64; MAX_VARS]; 2];
    let mut d2 = [[f64::NAN; MAX_VARS]; 2];
    let Some((lag, sh, r)) = frame_at(k, u, theta, true) else { return Frame { val: Val::default(), d1, d2 } };
    let val = val_of(&lag, &sh, &r, u, need);
    for j in 0..r.n {
        d1[0][j] = r.g[1][0][j];
        d1[1][j] = r.g[1][1][j];
    }
    if need >= 3 {
        d2[0][0] = val.d3[0];
        d2[1][0] = val.d3[1];
        let o = sh.ends.outer();
        let h = 1e-6 * sh.ends.len.abs().max(1.0);
        for c in 0..o.len() {
            let at = |sign: f64| -> Option<[f64; 2]> {
                let mut oo = o.clone();
                oo[c] += sign * h;
                let s = shoot::solve(&lag, &Ends::of(&oo, sh.ends.lines.len()), &sh.stops, Some(&sh))?;
                Some(read_at(&lag, &s, u, false)?.c[2])
            };
            if let (Some(p), Some(m)) = (at(1.0), at(-1.0)) {
                d2[0][1 + c] = (p[0] - m[0]) / (2.0 * h);
                d2[1][1 + c] = (p[1] - m[1]) / (2.0 * h);
            }
        }
    }
    Frame { val, d1, d2 }
}

/* -- the length's stationarity ------------------------------------------------------------ */

/// The transversality row of a curve whose length nothing holds (`variational::kernel`): `H` at
/// its end, over `[a, b, L, lines…]` — the energy's rate in the length, zero where the length is
/// where the energy is stationary in it.
pub fn transversal_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let nk = k.len() / n.max(1);
    let w = v.len() / n.max(1);
    for i in 0..n {
        let ki = &k[nk * i..nk * (i + 1)];
        r[i] = ends_in(ki, &v[w * i..w * (i + 1)])
            .and_then(|ends| shape_for(ki, &ends))
            .and_then(|(lag, sh)| shoot::at(&lag, &sh, 1.0, false))
            .map_or(f64::NAN, |a| a.point.h);
    }
}

/// Its gradient: `∂H/∂z · dz/d(a, b, L, lines…)` at the end, `H_θ` being zero.
pub fn transversal_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let nk = k.len() / n.max(1);
    let w = v.len() / n.max(1);
    for i in 0..n {
        let row = &mut j[w * i..w * (i + 1)];
        row.fill(f64::NAN);
        let ki = &k[nk * i..nk * (i + 1)];
        let Some(ends) = ends_in(ki, &v[w * i..w * (i + 1)]) else { continue };
        let Some((lag, sh)) = shape_for(ki, &ends) else { continue };
        let Some(a) = shoot::at(&lag, &sh, 1.0, true) else { continue };
        let hz = a.point.h_z();
        for c in 0..w.min(a.stride - 1) {
            row[c] = (0..4).map(|q| hz[q] * a.dz[q * a.stride + 1 + c]).sum();
        }
    }
}

/// The curve over `[a, b]` of its parameter at `n + 1` even steps, for drawing and picking —
/// positions alone.
pub fn points(lag: &Lagrangian, sh: &Shape, a: f64, b: f64, n: usize) -> Vec<(f64, f64)> {
    (0..=n)
        .map(|i| {
            let u = a + (b - a) * i as f64 / n as f64;
            shoot::position(lag, sh, u).map_or((f64::NAN, f64::NAN), |p| (p[0], p[1]))
        })
        .collect()
}

/* -- the verdict -------------------------------------------------------------------------- */

/// What the stationary shape is, of the Lagrangian as it is minimised (so a `maximizes`'s
/// "maximum" is this's minimum, turned by the caller), by the classical conditions:
///
/// - **Legendre**: `H_θθ` of one sign along every arc (positive where the direction makes `H`
///   least);
/// - **Jacobi**: no point conjugate to an arc's start before its end — where `det ∂p(s)/∂λ(start)`
///   changes sign, a neighbouring extremal from the start crosses this one again, and the arc
///   can be shortened in energy past it;
/// - **the places**: with the curve's stops (and its length, where `free_len`) moving, the
///   energy as a function of where they are, its Hessian by central differences of its gradient
///   — the jump in `H` at each stop for its place along the curve, the costate's jump along a
///   slide's line for its place on the line, and `H` at the end for the length — re-solved with
///   the places held.
///
/// The Morse index is the conjugate points counted and the places' negative directions; a
/// minimum has none and Legendre's sign positive, a maximum none of the opposite.
pub fn verdict(lag: &Lagrangian, sh: &Shape, free_len: bool) -> Extremum {
    let legendre = legendre(lag, sh);
    let Some(sign) = legendre else { return Extremum::Degenerate };
    let mut index = 0usize;
    let k = sh.stops.len();
    for j in 0..=k {
        match conjugate(lag, sh, j) {
            Some(n) => index += n,
            None => return Extremum::Degenerate,
        }
    }
    match places(lag, sh, free_len) {
        Some(eigs) => {
            let scale = eigs.iter().fold(0.0f64, |a, e| a.max(e.abs()));
            for e in eigs {
                if e.abs() <= 1e-7 * scale.max(1e-300) {
                    return Extremum::Degenerate;
                }
                if e * sign < 0.0 {
                    index += 1;
                }
            }
        }
        None => return Extremum::Degenerate,
    }
    match (index, sign > 0.0) {
        (0, true) => Extremum::Minimum,
        (0, false) => Extremum::Maximum,
        _ => Extremum::Saddle,
    }
}

/// `H_θθ`'s sign along the whole curve, `None` where it vanishes or changes.
fn legendre(lag: &Lagrangian, sh: &Shape) -> Option<f64> {
    let mut sign = 0.0;
    for piece in &sh.pieces {
        for n in &piece.nodes {
            let mut fl = flow::Flow::new(lag, n.theta);
            let p = fl.at(&n.z, false)?;
            let scale = n.z[2].dhypot(n.z[3]).max(p.d.f.abs()).max(1e-300);
            if p.htt.abs() <= 1e-9 * scale {
                return None;
            }
            let s = p.htt.signum();
            if sign == 0.0 {
                sign = s;
            } else if s != sign {
                return None;
            }
        }
    }
    (sign != 0.0).then_some(sign)
}

/// Points conjugate to arc `j`'s start, counted: sign changes of `det ∂p/∂λ(start)` along it,
/// sampled at every step and between.  `None` where the arc's end is itself conjugate.
fn conjugate(lag: &Lagrangian, sh: &Shape, j: usize) -> Option<usize> {
    let scale = sh.ends.len.abs().max(1e-9);
    let mut psi = shoot::IDENTITY;
    let det_of = |m: &[f64; 16]| {
        let (a, b, c, d) = (m[2], m[3], m[6], m[7]);
        let n = a.dhypot(c) * b.dhypot(d);
        if n == 0.0 { 0.0 } else { (a * d - b * c) / n }
    };
    let mul = |a: &[f64; 16], b: &[f64; 16]| -> [f64; 16] {
        std::array::from_fn(|ij| {
            let (i, jj) = (ij / 4, ij % 4);
            (0..4).map(|k| a[i * 4 + k] * b[k * 4 + jj]).sum()
        })
    };
    let mut last = 0.0f64;
    let mut changes = 0usize;
    let mut end = 0.0;
    for m in 0..shoot::SEGMENTS {
        let piece = &sh.pieces[j * shoot::SEGMENTS + m];
        let start = psi;
        let mut samples: Vec<[f64; 16]> = Vec::new();
        for w in piece.nodes.windows(2) {
            for q in 1..4 {
                let s = w[0].s + (w[1].s - w[0].s) * q as f64 / 4.0;
                let n = shoot::within(lag, piece, s, scale, true)?;
                samples.push(n.phi);
            }
            samples.push(w[1].phi);
        }
        for phi in samples {
            let total = mul(&phi, &start);
            let d = det_of(&total);
            if d.abs() > 1e-9 {
                if last != 0.0 && d.signum() != last.signum() {
                    changes += 1;
                }
                last = d;
            }
            end = d;
            psi = total;
        }
    }
    (end.abs() > 1e-8).then_some(changes)
}

/// The eigenvalues of the energy's Hessian in the curve's places — each stop's along the curve
/// and a slide's along its line, and its length's where `free_len` — or none to have where there
/// are none.
fn places(lag: &Lagrangian, sh: &Shape, free_len: bool) -> Option<Vec<f64>> {
    let k = sh.stops.len();
    // the places, in order: each stop's along the curve, then a slide's along its line
    let vars: Vec<(usize, bool)> =
        (0..k).flat_map(|j| std::iter::once((j, false)).chain(sh.stops[j].slides().then_some((j, true)))).collect();
    let n = vars.len() + usize::from(free_len);
    if n == 0 {
        return Some(Vec::new());
    }
    // ∂V/∂(places): an arc's energy grows with its length as its `H` (the length's multiplier),
    // so moving a stop along the curve is the jump in `H` there, and the length `H` at the end;
    // and with its end as its costate there (falling with its start as it), so moving a corner
    // along its line is the costate's jump along the line
    let gradient = |s: &Shape| -> Option<Vec<f64>> {
        let node = |piece: usize, last: bool| -> Option<shoot::Node> {
            let nodes = &s.pieces[piece].nodes;
            if last { nodes.last().copied() } else { nodes.first().copied() }
        };
        let h_of = |nd: shoot::Node| -> Option<f64> { Some(flow::Flow::new(lag, nd.theta).at(&nd.z, false)?.h) };
        let mut g = Vec::with_capacity(n);
        for &(j, along) in &vars {
            let before = node((j + 1) * shoot::SEGMENTS - 1, true)?;
            let after = node((j + 1) * shoot::SEGMENTS, false)?;
            g.push(match (along, s.stops[j]) {
                (true, Stop::Slide(k)) => {
                    let (_, d, _) = s.ends.line(k);
                    (after.z[2] - before.z[2]) * d[0] + (after.z[3] - before.z[3]) * d[1]
                }
                _ => h_of(before)? - h_of(after)?,
            });
        }
        if free_len {
            g.push(h_of(node(s.pieces.len() - 1, true)?)?);
        }
        Some(g)
    };
    let held: Vec<[f64; 2]> = (0..k).map(|j| [sh.places[j + 1], sh.sigmas[j]]).collect();
    let h = 1e-5 * sh.ends.len.abs().max(1e-9);
    let mut hess = vec![0.0; n * n];
    for c in 0..n {
        let solve = |sign: f64| -> Option<Vec<f64>> {
            let mut pl = held.clone();
            let mut from = sh.clone();
            match vars.get(c) {
                Some(&(j, along)) => pl[j][usize::from(along)] += sign * h,
                None => from.ends.len += sign * h,
            }
            let s = shoot::solve_held(lag, &from, &pl)?;
            gradient(&s)
        };
        let (p, m) = (solve(1.0)?, solve(-1.0)?);
        for r in 0..n {
            hess[r * n + c] = (p[r] - m[r]) / (2.0 * h);
        }
    }
    let sym: Vec<f64> = (0..n * n).map(|ij| 0.5 * (hess[ij] + hess[(ij % n) * n + ij / n])).collect();
    Some(crate::linalg::sym_eigenvalues(&crate::linalg::Mat::from_vec(n, n, sym)))
}
