//! The flow an energy's Euler–Lagrange equation makes, and its integration (#144).
//!
//! In arc length `s`, with the tangent at angle `θ`, an energy `∫ f(p, t) ds` is an optimal
//! control problem: the state is the point `p`, the control the direction, `p' = t(θ)`.  Its
//! Hamiltonian is `H = f(p, t(θ)) + λ·t(θ)`, the costate `λ` runs `λ' = −f_p`, and the direction
//! is wherever `H` is stationary in it, `H_θ = 0` — algebraic, solved at every point by Newton
//! from where the direction last was.  So the flow is over `z = (x, y, λx, λy)` alone, and its
//! Jacobian needs only the integrand's second derivatives: `dθ/dz = −H_θz / H_θθ`.  The length
//! is the interval, so it takes no multiplier; `H` is conserved, and is that multiplier's
//! negative.
//!
//! Integrated by Gragg–Bulirsch–Stoer: the explicit midpoint rule, its error even in the step,
//! extrapolated to zero step by Aitken–Neville, the step adapted to the tolerance.  A smooth flow
//! at a tight tolerance is what it is best at, it has no tableau to carry, and every operation is
//! linear in the substeps' results — so the **variational equations** integrated through the same
//! substeps are the exact derivative of the step the flow took.

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::expr::{Ast, Op};
use crate::tape::{self, Tape};
use crate::units::Dim;

/// The integrand, at a unit tangent `t = (cos θ, sin θ)`, as three tapes over `(θ, x, y)` — the
/// `k`th variable first in the `k`th, so a tape's series differentiates in it and the gradient of
/// its first order is the Hessian's `k`th row (`tape::Series`).  Its terms are summed, each times
/// its coefficient, and signed so that it is always the least that is sought: a `maximizes`
/// negates it.
#[derive(Clone, Debug)]
pub struct Lagrangian {
    pub tapes: [Vec<f64>; 3],
}

const ORDER: [[&str; 3]; 3] = [["th", "x", "y"], ["x", "th", "y"], ["y", "th", "x"]];

impl Lagrangian {
    /// `Σ coef · text` over `p.x`, `p.y`, `t.x`, `t.y` (each `flatten::values::settle_integrand`
    /// settled), negated where `maximize`.
    pub fn compile(terms: &[(f64, &str)], maximize: bool, units: crate::units::Units) -> Result<Lagrangian, String> {
        let deg = Ast::Num(180.0 / std::f64::consts::PI, Dim::SCALAR);
        let angle = Ast::Bin(Op::Mul, Box::new(Ast::Var("th".into())), Box::new(deg));
        fn rewrite(a: &Ast, angle: &Ast) -> Ast {
            match a {
                Ast::Var(v) => match v.as_str() {
                    "p.x" => Ast::Var("x".into()),
                    "p.y" => Ast::Var("y".into()),
                    // the tapes' trigonometry is in degrees
                    "t.x" => Ast::Call("cos".into(), vec![angle.clone()]),
                    "t.y" => Ast::Call("sin".into(), vec![angle.clone()]),
                    _ => a.clone(),
                },
                Ast::Neg(x) => Ast::Neg(Box::new(rewrite(x, angle))),
                Ast::Bin(o, x, y) => Ast::Bin(*o, Box::new(rewrite(x, angle)), Box::new(rewrite(y, angle))),
                Ast::Call(f, xs) => Ast::Call(f.clone(), xs.iter().map(|x| rewrite(x, angle)).collect()),
                other => other.clone(),
            }
        }
        let mut sum: Option<Ast> = None;
        for &(coef, text) in terms {
            let body = rewrite(&crate::expr::parse_in(text, units)?.body, &angle);
            let c = if maximize { -coef } else { coef };
            let term = Ast::Bin(Op::Mul, Box::new(Ast::Num(c, Dim::SCALAR)), Box::new(body));
            sum = Some(match sum {
                None => term,
                Some(s) => Ast::Bin(Op::Add, Box::new(s), Box::new(term)),
            });
        }
        let f = sum.ok_or("an energy has a term")?;
        let tape = |k: usize| -> Result<Vec<f64>, String> {
            let vars: Vec<String> = ORDER[k].iter().map(|v| v.to_string()).collect();
            Ok(Tape::compile(&f, &vars)?.flat)
        };
        Ok(Lagrangian { tapes: [tape(0)?, tape(1)?, tape(2)?] })
    }

    /// The flat encoding a kernel's constants carry: each tape's length, then the tapes.
    pub fn write(&self, out: &mut Vec<f64>) {
        out.extend(self.tapes.iter().map(|t| t.len() as f64));
        for t in &self.tapes {
            out.extend_from_slice(t);
        }
    }

    /// What `write` wrote, and the rest of the constants after it.
    pub fn read(k: &[f64]) -> Option<(Lagrangian, &[f64])> {
        let lens: Vec<usize> = k.get(..3)?.iter().map(|&n| n as usize).collect();
        let mut at = 3;
        let mut tapes: [Vec<f64>; 3] = Default::default();
        for (t, &n) in tapes.iter_mut().zip(&lens) {
            *t = k.get(at..at + n)?.to_vec();
            at += n;
        }
        Some((Lagrangian { tapes }, &k[at..]))
    }
}

/// The integrand's derivatives at one point and direction: `f` and its derivatives in `θ`,
/// `x`, `y`, to the second order (the third in `θ` alone besides).
#[derive(Clone, Copy, Debug, Default)]
pub struct Partials {
    pub f: f64,
    pub t: f64,
    pub tt: f64,
    pub x: f64,
    pub y: f64,
    pub tx: f64,
    pub ty: f64,
    pub xx: f64,
    pub xy: f64,
    pub yy: f64,
}

/// The flow at one state, with what it took to get there.
#[derive(Clone, Copy, Debug)]
pub struct Point {
    pub theta: f64,
    pub d: Partials,
    /// `H_θθ`, positive: the direction makes `H` least (Legendre, Pontryagin).
    pub htt: f64,
    pub h: f64,
}

/// The flow's evaluator: the integrand, a tape scratch, and where the direction last was, which
/// the next `H_θ = 0` starts from — so the branch is the one the curve is on.
pub struct Flow<'a> {
    pub lag: &'a Lagrangian,
    sc: tape::Scratch,
    pub theta: f64,
}

const NEWTON_MAX: usize = 30;

impl<'a> Flow<'a> {
    pub fn new(lag: &'a Lagrangian, theta: f64) -> Flow<'a> {
        Flow { lag, sc: tape::Scratch::new(), theta }
    }

    /// The integrand's derivatives at `(θ, x, y)`; the Hessian's spatial rows only when `full`.
    pub fn partials(&mut self, th: f64, x: f64, y: f64, full: bool) -> Partials {
        let s0 = tape::eval_series_flat(&self.lag.tapes[0], 3, &[th, x, y], &mut self.sc);
        let mut d = Partials {
            f: s0.c[0],
            t: s0.c[1],
            tt: s0.c[2],
            x: s0.g[0][1],
            y: s0.g[0][2],
            tx: s0.g[1][1],
            ty: s0.g[1][2],
            ..Partials::default()
        };
        if full {
            let s1 = tape::eval_series_flat(&self.lag.tapes[1], 3, &[x, th, y], &mut self.sc);
            let s2 = tape::eval_series_flat(&self.lag.tapes[2], 3, &[y, th, x], &mut self.sc);
            d.xx = s1.g[1][0];
            d.xy = s1.g[1][2];
            d.yy = s2.g[1][0];
        }
        d
    }

    /// The direction at `z`, from where it last was: `H_θ = 0` by Newton, at a root where `H` is
    /// least in the direction (`H_θθ > 0`) — Pontryagin's principle, which every minimum keeps, a
    /// `maximizes` included since its Lagrangian is negated.  A root where `H` is greatest is the
    /// curve turned the other way (a hanging rope's arch), so the opposite direction is tried
    /// before giving up.  `None` where there is no such root near (the problem's own singularity,
    /// or a step too far for the branch).
    pub fn at(&mut self, z: &[f64], full: bool) -> Option<Point> {
        let guess = self.theta;
        for start in [guess, guess + std::f64::consts::PI] {
            if let Some(th) = self.root(z, start) {
                self.theta = th;
                let d = self.partials(th, z[0], z[1], full);
                let (s, c) = th.dsin_cos();
                let htt = d.tt - z[2] * c - z[3] * s;
                if htt > 0.0 {
                    return Some(Point { theta: th, d, htt, h: d.f + z[2] * c + z[3] * s });
                }
            }
        }
        self.theta = guess;
        None
    }

    /// A root of `H_θ` by Newton from `th`.
    fn root(&mut self, z: &[f64], mut th: f64) -> Option<f64> {
        let (lx, ly) = (z[2], z[3]);
        for _ in 0..NEWTON_MAX {
            let d = self.partials(th, z[0], z[1], false);
            let (s, c) = th.dsin_cos();
            let g = d.t - lx * s + ly * c;
            let gp = d.tt - lx * c - ly * s;
            if !(gp.is_finite() && g.is_finite()) || gp == 0.0 {
                return None;
            }
            let step = g / gp;
            th -= step;
            if step.abs() <= 1e-15 * (1.0 + th.abs()) {
                return Some(th);
            }
        }
        None
    }

    /// `z'` at `z`, and, asked for (`phi` the 4 × m sensitivities beside it), `A·Φ` with `A` the
    /// flow's Jacobian.  `false` where the direction cannot be had.
    pub fn rhs(&mut self, y: &[f64], out: &mut [f64], m: usize) -> bool {
        debug_assert!(m == 0 || m == 4);
        let Some(p) = self.at(&y[..4], m > 0) else { return false };
        let (s, c) = p.theta.dsin_cos();
        out[0] = c;
        out[1] = s;
        out[2] = -p.d.x;
        out[3] = -p.d.y;
        if m == 0 {
            return true;
        }
        if p.htt == 0.0 {
            return false;
        }
        // dθ/dz = −H_θz / H_θθ, H_θz = (f_θx, f_θy, −sin θ, cos θ)
        let tz = [-p.d.tx / p.htt, -p.d.ty / p.htt, s / p.htt, -c / p.htt];
        let mut a = [[0.0f64; 4]; 4];
        for j in 0..4 {
            a[0][j] = -s * tz[j];
            a[1][j] = c * tz[j];
            a[2][j] = -p.d.tx * tz[j];
            a[3][j] = -p.d.ty * tz[j];
        }
        a[2][0] -= p.d.xx;
        a[2][1] -= p.d.xy;
        a[3][0] -= p.d.xy;
        a[3][1] -= p.d.yy;
        let phi = &y[4..4 + 4 * m];
        let o = &mut out[4..4 + 4 * m];
        for i in 0..4 {
            for k in 0..m {
                let mut v = 0.0;
                for j in 0..4 {
                    v += a[i][j] * phi[j * m + k];
                }
                o[i * m + k] = v;
            }
        }
        true
    }
}

/* -- Gragg–Bulirsch–Stoer ----------------------------------------------------------------- */

/// The substep counts of the extrapolation: seven columns, to order fourteen.
const STEPS: [usize; 7] = [2, 4, 6, 8, 10, 12, 14];

/// The relative tolerance one step is held to.
pub const TOL: f64 = 1e-13;

/// The augmented state's width: the state, then its sensitivities to four starting values.
pub const WIDTH: usize = 20;

/// The explicit midpoint rule over `n` substeps of one step `h` from `y0` (`4 + 4m` wide).
fn midpoint(flow: &mut Flow, theta0: f64, y0: &[f64; WIDTH], h: f64, n: usize, m: usize, out: &mut [f64; WIDTH]) -> bool {
    let len = 4 + 4 * m;
    let hs = h / n as f64;
    flow.theta = theta0;
    let mut f = [0.0; WIDTH];
    let mut prev = *y0;
    if !flow.rhs(&prev, &mut f, m) {
        return false;
    }
    let mut cur = [0.0; WIDTH];
    for i in 0..len {
        cur[i] = prev[i] + hs * f[i];
    }
    for _ in 1..n {
        if !flow.rhs(&cur, &mut f, m) {
            return false;
        }
        for i in 0..len {
            let next = prev[i] + 2.0 * hs * f[i];
            prev[i] = cur[i];
            cur[i] = next;
        }
    }
    *out = cur;
    true
}

/// One extrapolated step of `h` from `y0` (the state, then — `m` 4 — its sensitivities): the
/// result and its error estimate over the state, as a fraction of `scale`.  Every column of the
/// table is taken (a fixed order), the last two telling the error.  `None` where the flow could
/// not be followed.
pub fn step(flow: &mut Flow, theta0: f64, y0: &[f64; WIDTH], h: f64, m: usize, scale: &[f64; 4]) -> Option<([f64; WIDTH], f64)> {
    let len = 4 + 4 * m;
    const K: usize = STEPS.len();
    let mut prev = [[0.0; WIDTH]; K];
    let mut row = [[0.0; WIDTH]; K];
    for (j, &n) in STEPS.iter().enumerate() {
        if !midpoint(flow, theta0, y0, h, n, m, &mut row[0]) {
            return None;
        }
        for c in 1..=j {
            let q = n as f64 / STEPS[j - c] as f64;
            let r = q * q - 1.0;
            for i in 0..len {
                row[c][i] = row[c - 1][i] + (row[c - 1][i] - prev[c - 1][i]) / r;
            }
        }
        prev[..=j].copy_from_slice(&row[..=j]);
    }
    let err = (0..4).map(|i| (prev[K - 1][i] - prev[K - 2][i]).abs() / scale[i]).fold(0.0, f64::max);
    let out = prev[K - 1];
    out[..len].iter().all(|v| v.is_finite()).then_some((out, err))
}

/// The order the error estimate is of, for the step-size rule.
const ESTIMATE_ORDER: f64 = (2 * STEPS.len() - 2) as f64;

/// The next step's size after one whose relative error was `err`.
pub fn next_h(h: f64, err: f64) -> f64 {
    let f = if err == 0.0 {
        4.0
    } else {
        (0.9 * (TOL / err).dpowf(1.0 / (ESTIMATE_ORDER + 1.0))).clamp(0.2, 4.0)
    };
    h * f
}
