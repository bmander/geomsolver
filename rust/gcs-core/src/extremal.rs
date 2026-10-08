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
pub use shoot::{Ends, Shape};

/* -- the problem as a kernel's constants -------------------------------------------------- */

/// A curve's problem, as its contacts' constants carry it: the Lagrangian, then the pegs.
pub fn write(lag: &Lagrangian, pegs: &[[f64; 2]], out: &mut Vec<f64>) {
    lag.write(out);
    out.push(pegs.len() as f64);
    for p in pegs {
        out.extend_from_slice(p);
    }
}

/// What `write` wrote.
pub fn read(k: &[f64]) -> Option<(Lagrangian, Vec<[f64; 2]>)> {
    let (lag, rest) = Lagrangian::read(k)?;
    let n = *rest.first()? as usize;
    let pegs = (0..n).map(|i| Some([*rest.get(1 + 2 * i)?, *rest.get(2 + 2 * i)?])).collect::<Option<Vec<_>>>()?;
    Some((lag, pegs))
}

fn ends_of(theta: &[f64]) -> Option<Ends> {
    (theta.len() >= 5).then(|| Ends { a: [theta[0], theta[1]], b: [theta[2], theta[3]], len: theta[4] })
}

/* -- the memo ----------------------------------------------------------------------------- */

thread_local! {
    /// The last shape each contact's constants were solved to, by where they live (as
    /// `locus`'s poses are): asked again at the same ends it is the answer, at neighbouring ones
    /// where the next solve starts.
    static SEEN: std::cell::RefCell<std::collections::BTreeMap<(usize, usize), Shape>> =
        std::cell::RefCell::new(std::collections::BTreeMap::new());
}

const SEEN_MAX: usize = 4096;

/// Forget every remembered shape — `System::new`, where constants move.
pub fn forget() {
    SEEN.with(|s| s.borrow_mut().clear());
}

/// The shape a contact's constants and outer columns come to, warm from the last.
fn shape_for(k: &[f64], lag: &Lagrangian, pegs: &[[f64; 2]], ends: &Ends) -> Option<Shape> {
    let key = crate::locus::key_of(k);
    let prev = SEEN.with(|s| s.borrow().get(&key).cloned());
    let sh = shoot::solve(lag, ends, pegs, prev.as_ref())?;
    SEEN.with(|s| {
        let mut s = s.borrow_mut();
        if s.len() >= SEEN_MAX {
            s.clear();
        }
        s.insert(key, sh.clone());
    });
    Some(sh)
}

/* -- reading a shape ---------------------------------------------------------------------- */

/// `C`, `C'`, `C''` at `u` and the gradients of the first two over `[u, a, b, L]`.
struct Read {
    c: [[f64; 2]; 3],
    g: [[[f64; 6]; 2]; 2],
}

fn read_at(lag: &Lagrangian, sh: &Shape, u: f64) -> Option<Read> {
    let at = shoot::at(lag, sh, u)?;
    let len = sh.ends.len;
    let (sn, cs) = at.theta.dsin_cos();
    // dθ/ds, and dθ over the columns
    let rate: f64 = (0..4).map(|i| at.tz[i] * at.f[i]).sum();
    let dth: [f64; 6] = std::array::from_fn(|j| (0..4).map(|i| at.tz[i] * at.dz[i * 6 + j]).sum());
    let mut g = [[[0.0f64; 6]; 2]; 2];
    for j in 0..6 {
        g[0][0][j] = at.dz[j];
        g[0][1][j] = at.dz[6 + j];
        g[1][0][j] = -len * sn * dth[j];
        g[1][1][j] = len * cs * dth[j];
    }
    g[1][0][5] += cs;
    g[1][1][5] += sn;
    let k2 = len * len * rate;
    Some(Read { c: [[at.z[0], at.z[1]], [len * cs, len * sn], [-k2 * sn, k2 * cs]], g })
}

/// `C'''` at `u`, by a central difference of `C''` in the parameter (the integrand's third
/// derivatives are not to be had), one-sided at the ends.
fn third(lag: &Lagrangian, sh: &Shape, u: f64) -> Option<[f64; 2]> {
    const DU: f64 = 1e-5;
    let (lo, hi) = ((u - DU).max(0.0), (u + DU).min(1.0));
    let a = read_at(lag, sh, lo)?.c[2];
    let b = read_at(lag, sh, hi)?.c[2];
    Some([(b[0] - a[0]) / (hi - lo), (b[1] - a[1]) / (hi - lo)])
}

/// A contact's evaluation (`kernels::body_val`'s for this body): `C` and its derivatives in `u`
/// to `need`, and the gradient of `C` over `[u, θ…]` (`θ…` = `a`, `b`, `L`).  NaN where the
/// shape cannot be had.
pub fn kernel_eval(k: &[f64], u: f64, theta: &[f64], need: u8) -> Val {
    let mut v = Val::default();
    let Some((lag, pegs)) = read(k) else { return v };
    let Some(ends) = ends_of(theta) else { return v };
    let Some(sh) = shape_for(k, &lag, &pegs, &ends) else { return v };
    let Some(r) = read_at(&lag, &sh, u) else { return v };
    v.x = r.c[0][0];
    v.y = r.c[0][1];
    for j in 0..6.min(MAX_VARS) {
        v.dx[j] = r.g[0][0][j];
        v.dy[j] = r.g[0][1][j];
    }
    v.d2 = r.c[2];
    v.orders = 2;
    if need >= 3 {
        if let Some(d3) = third(&lag, &sh, u) {
            v.d3 = d3;
            v.orders = 3;
        }
    }
    v.ok = true;
    v
}

/// A contact's frame (`kernels::curve_frame`'s): the evaluation, the gradient of `C'`, and, asked
/// for (`need` 3), of `C''` — along `u` that is `C'''`, along the ends and the length a central
/// difference of solves warm from this one.
pub fn kernel_frame(k: &[f64], u: f64, theta: &[f64], need: u8) -> Frame {
    let val = kernel_eval(k, u, theta, need);
    let mut d1 = [[0.0f64; MAX_VARS]; 2];
    let mut d2 = [[f64::NAN; MAX_VARS]; 2];
    let fr = |val| Frame { val, d1, d2 };
    if !val.ok {
        return fr(val);
    }
    let (Some((lag, pegs)), Some(ends)) = (read(k), ends_of(theta)) else { return fr(val) };
    let Some(sh) = shape_for(k, &lag, &pegs, &ends) else { return fr(val) };
    let Some(r) = read_at(&lag, &sh, u) else { return fr(val) };
    for j in 0..6 {
        d1[0][j] = r.g[1][0][j];
        d1[1][j] = r.g[1][1][j];
    }
    if need >= 3 {
        d2[0][0] = val.d3[0];
        d2[1][0] = val.d3[1];
        let o = ends.outer();
        for c in 0..5 {
            let h = 1e-6 * ends.len.abs().max(1.0);
            let at = |sign: f64| -> Option<[f64; 2]> {
                let mut oo = o;
                oo[c] += sign * h;
                let s = shoot::solve(&lag, &Ends::of(&oo), &pegs, Some(&sh))?;
                Some(read_at(&lag, &s, u)?.c[2])
            };
            if let (Some(p), Some(m)) = (at(1.0), at(-1.0)) {
                d2[0][1 + c] = (p[0] - m[0]) / (2.0 * h);
                d2[1][1 + c] = (p[1] - m[1]) / (2.0 * h);
            }
        }
    }
    Frame { val, d1, d2 }
}

/// The curve sampled at `n + 1` even steps of its parameter, for drawing and picking.
pub fn sweep(lag: &Lagrangian, sh: &Shape, n: usize) -> Vec<(f64, f64)> {
    (0..=n)
        .map(|i| {
            let u = i as f64 / n as f64;
            match shoot::at(lag, sh, u) {
                Some(a) => (a.z[0], a.z[1]),
                None => (f64::NAN, f64::NAN),
            }
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
/// - **the places**: with the curve's pegs (and its length, where `free_len`) moving, the energy
///   as a function of where they are, its Hessian by central differences of `∂V/∂s` — the jump
///   in `H` at each peg, and `H` at the end for the length — re-solved with the places held.
///
/// The Morse index is the conjugate points counted and the places' negative directions; a
/// minimum has none and Legendre's sign positive, a maximum none of the opposite.
pub fn verdict(lag: &Lagrangian, sh: &Shape, free_len: bool) -> Extremum {
    let legendre = legendre(lag, sh);
    let Some(sign) = legendre else { return Extremum::Degenerate };
    let mut index = 0usize;
    let k = sh.pegs.len();
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
    let mut psi = [1.0, 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.];
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
                let n = shoot::within(lag, piece, s, scale)?;
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

/// The eigenvalues of the energy's Hessian in the curve's places — its pegs', and its length's
/// where `free_len` — or none to have where there are none.
fn places(lag: &Lagrangian, sh: &Shape, free_len: bool) -> Option<Vec<f64>> {
    let k = sh.pegs.len();
    let n = k + usize::from(free_len);
    if n == 0 {
        return Some(Vec::new());
    }
    // ∂V/∂(places): an arc's energy grows with its length as its `H` (the length's multiplier),
    // so moving a peg along the curve is the jump in `H` there, and the length `H` at the end
    let gradient = |s: &Shape| -> Option<Vec<f64>> {
        let h_at = |piece: usize, last: bool| -> Option<f64> {
            let nodes = &s.pieces[piece].nodes;
            let nd = if last { nodes.last()? } else { nodes.first()? };
            Some(flow::Flow::new(lag, nd.theta).at(&nd.z, false)?.h)
        };
        let mut g = Vec::with_capacity(n);
        for j in 1..=k {
            g.push(h_at(j * shoot::SEGMENTS - 1, true)? - h_at(j * shoot::SEGMENTS, false)?);
        }
        if free_len {
            g.push(h_at(s.pieces.len() - 1, true)?);
        }
        Some(g)
    };
    let held: Vec<f64> = sh.places[1..=k].to_vec();
    let h = 1e-5 * sh.ends.len.abs().max(1e-9);
    let mut hess = vec![0.0; n * n];
    for c in 0..n {
        let solve = |sign: f64| -> Option<Vec<f64>> {
            let mut pl = held.clone();
            let mut from = sh.clone();
            if c < k {
                pl[c] += sign * h;
            } else {
                from.ends.len += sign * h;
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
