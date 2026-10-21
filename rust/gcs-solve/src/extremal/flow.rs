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
//! The flow is a `crate::ode::Rate`, its branch the direction, integrated by the core's
//! Gragg–Bulirsch–Stoer (`ode`, #339 D1) with the variational equations through the same
//! substeps: the exact derivative of the step the flow took.

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
        let rewrite = |v: &str| match v {
            "p.x" => Some(Ast::Var("x".into())),
            "p.y" => Some(Ast::Var("y".into())),
            // the tapes' trigonometry is in degrees
            "t.x" => Some(Ast::Call("cos".into(), vec![angle.clone()])),
            "t.y" => Some(Ast::Call("sin".into(), vec![angle.clone()])),
            _ => None,
        };
        let mut sum: Option<Ast> = None;
        for &(coef, text) in terms {
            let body = crate::expr::parse_in(text, units)?.body.map_vars(&rewrite);
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
            if let Some((th, mut d)) = self.root(z, start) {
                let (s, c) = th.dsin_cos();
                let htt = d.tt - z[2] * c - z[3] * s;
                if htt > 0.0 {
                    self.theta = th;
                    if full {
                        d = self.partials(th, z[0], z[1], true);
                    }
                    return Some(Point { theta: th, d, htt, h: d.f + z[2] * c + z[3] * s });
                }
            }
        }
        self.theta = guess;
        None
    }

    /// A root of `H_θ` by Newton from `th`, with the integrand's partials there (the last
    /// iteration's, at a direction within rounding of the root).
    fn root(&mut self, z: &[f64], mut th: f64) -> Option<(f64, Partials)> {
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
                return Some((th, d));
            }
        }
        None
    }
}

/// The flow as a rate in arc length, its branch the direction (`H_θ = 0` solved from where it
/// last was): `z' = (cos θ, sin θ, −f_x, −f_y)`, and its Jacobian through `dθ/dz`.
impl crate::ode::Rate for Flow<'_> {
    type Branch = f64;

    fn width(&self) -> usize {
        4
    }

    fn rate(&mut self, _s: f64, z: &[f64], out: &mut [f64], jac: Option<&mut [f64]>) -> bool {
        let Some(p) = self.at(&z[..4], jac.is_some()) else { return false };
        out[..4].copy_from_slice(&p.flow());
        let Some(a) = jac else { return true };
        let (s, c) = p.theta.dsin_cos();
        let tz = p.theta_z();
        for j in 0..4 {
            a[j] = -s * tz[j];
            a[4 + j] = c * tz[j];
            a[8 + j] = -p.d.tx * tz[j];
            a[12 + j] = -p.d.ty * tz[j];
        }
        a[8] -= p.d.xx;
        a[9] -= p.d.xy;
        a[12] -= p.d.xy;
        a[13] -= p.d.yy;
        true
    }

    fn branch(&self) -> f64 {
        self.theta
    }

    fn set_branch(&mut self, theta: f64) {
        self.theta = theta;
    }
}

impl Point {
    /// The flow here: `z' = (cos θ, sin θ, −f_x, −f_y)`.
    pub fn flow(&self) -> [f64; 4] {
        let (s, c) = self.theta.dsin_cos();
        [c, s, -self.d.x, -self.d.y]
    }

    /// `dθ/dz = −H_θz / H_θθ`, with `H_θz = (f_θx, f_θy, −sin θ, cos θ)`.
    pub fn theta_z(&self) -> [f64; 4] {
        let (s, c) = self.theta.dsin_cos();
        [-self.d.tx / self.htt, -self.d.ty / self.htt, s / self.htt, -c / self.htt]
    }

    /// `∂H/∂z = (f_x, f_y, cos θ, sin θ)` — the direction's own term vanishing, `H_θ` being zero.
    pub fn h_z(&self) -> [f64; 4] {
        let (s, c) = self.theta.dsin_cos();
        [self.d.x, self.d.y, c, s]
    }
}
