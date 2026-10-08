//! Truncated Taylor arithmetic in one direction, and the static kernels read in it: what lets a
//! traced curve give `C''` and `C'''` exactly (spec §6.5).
//!
//! A trace's point is `q(u)` where `r(q(u), u, w(u)) = 0`.  Its first derivative is one linear
//! solve with the block's Jacobian `Jq` (`locus::finish`).  Each further order is another solve
//! with the *same* factorisation (Wagner, Walther & Schaefer, "On the efficient computation of
//! high-order derivatives for implicitly defined functions", CPC 2010): put the path
//! `z(ε) = z₀ + z₁ε + … + z_{k−1}ε^{k−1}` (with `q_k` not yet known, so 0) through every row,
//! read off the ε^k coefficient `R_k`, and `Jq q_k = −R_k`, since `q_k` enters that coefficient
//! only through `Jq`.  So what is needed of a kernel is its residual over a [`Jet`] — a
//! polynomial path in, the Taylor coefficients of its residual out — and nothing else.
//!
//! A kernel's **form** here is that reading.  An affine kernel's is its Jacobian (`r_k = J·z_k`);
//! a nonlinear one's is written out over `Jet`s beside its `f64` residual in `kernels.rs`, and
//! `tests/taylor.rs` holds every form to its kernel: the constant term to `res`, the first
//! coefficient to `jac`, the higher ones to differences of `jac` along the path.  A kernel with
//! no form is named where a curvature against a trace that uses it is refused.

use crate::kernels::{Kernel, KERNELS, MIN_LINE_LEN};

/// Coefficients kept: the value and four orders.  A curvature's Jacobian reads `C'''`
/// (`∂C''/∂u`), and a generated profile's third order reads the motion's translation one order
/// higher through its rate (`generate::at`), so a fourth is carried.
pub const ORDER: usize = 5;

/// `a₀ + a₁ε + a₂ε² + a₃ε³`, truncated: Taylor **coefficients**, not derivatives (`a_k` is the
/// `k`-th derivative over `k!`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Jet(pub [f64; ORDER]);

impl Jet {
    pub fn constant(v: f64) -> Jet {
        Jet::from(&[v])
    }

    /// `v + ε`: a variable moving at unit rate.
    pub fn var(v: f64) -> Jet {
        Jet::from(&[v, 1.0])
    }

    /// The leading coefficients given, the rest 0.
    pub fn from(c: &[f64]) -> Jet {
        let mut a = [0.0; ORDER];
        a[..c.len().min(ORDER)].copy_from_slice(&c[..c.len().min(ORDER)]);
        Jet(a)
    }

    /// The series with the given **derivatives** `f, f', f'', …` (each divided by `k!`).
    pub fn from_derivatives(d: &[f64]) -> Jet {
        let mut a = [0.0; ORDER];
        let mut fact = 1.0;
        for (k, &x) in d.iter().take(ORDER).enumerate() {
            fact *= k.max(1) as f64;
            a[k] = x / fact;
        }
        Jet(a)
    }

    /// The `k`-th derivative at ε = 0: the coefficient times `k!`.
    pub fn nth(self, k: usize) -> f64 {
        (1..=k).fold(self.0[k], |acc, j| acc * j as f64)
    }

    /// `Σ cₖ δᵏ` — this series read as a polynomial in the jet `delta` (no constant term): a
    /// function's Taylor series about `s₀` composed with a path `s(ε) = s₀ + δ(ε)`.
    pub fn compose(self, delta: Jet) -> Jet {
        let mut out = Jet::constant(self.0[0]);
        let mut pow = Jet::constant(1.0);
        for k in 1..ORDER {
            pow = pow * delta;
            out = out + pow * self.0[k];
        }
        out
    }

    /// `d/dε`, one order lost: the coefficients of the derivative series (the top one 0, since
    /// it is not known).
    pub fn derivative(self) -> Jet {
        let a = self.0;
        Jet(std::array::from_fn(|k| if k + 1 < ORDER { (k + 1) as f64 * a[k + 1] } else { 0.0 }))
    }

    /// The series whose derivative is `self` and whose constant term is `c0`.
    fn integral(self, c0: f64) -> Jet {
        let a = self.0;
        Jet(std::array::from_fn(|k| if k == 0 { c0 } else { a[k - 1] / k as f64 }))
    }

    pub fn sqrt(self) -> Jet {
        let a = self.0;
        let s0 = a[0].sqrt();
        let mut s = [0.0; ORDER];
        s[0] = s0;
        // s·s = a, order by order
        for k in 1..ORDER {
            let mut acc = a[k];
            for j in 1..k {
                acc -= s[j] * s[k - j];
            }
            s[k] = acc / (2.0 * s0);
        }
        Jet(s)
    }

    /// `|(x, y)|` held off zero as `kernels::line_len` is: below the floor the length is the
    /// floor, a constant.
    pub fn line_len(x: Jet, y: Jet) -> Jet {
        let h = (x * x + y * y).sqrt();
        if !(h.0[0] >= MIN_LINE_LEN) {
            return Jet::constant(MIN_LINE_LEN);
        }
        h
    }

    /// `atan2(y, x)`, its constant term the kernels' (`Det::datan2`), the rest integrated from
    /// `θ' = (x y' − y x') / (x² + y²)`.
    pub fn atan2(y: Jet, x: Jet) -> Jet {
        use crate::fmath::Det;
        let rate = (x * y.derivative() - y * x.derivative()) / (x * x + y * y);
        rate.integral(y.0[0].datan2(x.0[0]))
    }

    /// `(sin a, cos a)`, order by order: `s' = c a'` and `c' = −s a'`.
    pub fn sin_cos(self) -> (Jet, Jet) {
        use crate::fmath::Det;
        let a = self.0;
        let (mut s, mut c) = ([0.0; ORDER], [0.0; ORDER]);
        s[0] = a[0].dsin();
        c[0] = a[0].dcos();
        for k in 1..ORDER {
            let (mut sk, mut ck) = (0.0, 0.0);
            for j in 1..=k {
                sk += j as f64 * a[j] * c[k - j];
                ck -= j as f64 * a[j] * s[k - j];
            }
            s[k] = sk / k as f64;
            c[k] = ck / k as f64;
        }
        (Jet(s), Jet(c))
    }

    /// `|a|`, as the kernels read it: the sign of the constant term (`side_of`: zero is +).
    pub fn abs(self) -> Jet {
        if self.0[0] < 0.0 { -self } else { self }
    }

    /// The constant term moved by `d`, the rest kept — a wrap of a turn, a floor.
    pub fn shift(self, d: f64) -> Jet {
        let mut a = self.0;
        a[0] += d;
        Jet(a)
    }
}

impl std::ops::Add for Jet {
    type Output = Jet;
    fn add(self, o: Jet) -> Jet {
        Jet(std::array::from_fn(|k| self.0[k] + o.0[k]))
    }
}

impl std::ops::Sub for Jet {
    type Output = Jet;
    fn sub(self, o: Jet) -> Jet {
        Jet(std::array::from_fn(|k| self.0[k] - o.0[k]))
    }
}

impl std::ops::Neg for Jet {
    type Output = Jet;
    fn neg(self) -> Jet {
        Jet(self.0.map(|v| -v))
    }
}

impl std::ops::Mul for Jet {
    type Output = Jet;
    fn mul(self, o: Jet) -> Jet {
        let (a, b) = (self.0, o.0);
        Jet(std::array::from_fn(|k| (0..=k).map(|j| a[j] * b[k - j]).sum()))
    }
}

impl std::ops::Mul<f64> for Jet {
    type Output = Jet;
    fn mul(self, s: f64) -> Jet {
        Jet(self.0.map(|v| v * s))
    }
}

impl std::ops::Div for Jet {
    type Output = Jet;
    fn div(self, o: Jet) -> Jet {
        let (a, b) = (self.0, o.0);
        let mut q = [0.0; ORDER];
        for k in 0..ORDER {
            let mut acc = a[k];
            for j in 0..k {
                acc -= q[j] * b[k - j];
            }
            q[k] = acc / b[0];
        }
        Jet(q)
    }
}

/// How a kernel is read over `Jet`s.
#[derive(Clone, Copy)]
enum Form {
    /// Affine in its columns: `r₀ = res(z₀)` and `r_k = J·z_k`, with `J` constant.
    Affine,
    /// Written out over `Jet`s: the residual rows, from the columns and the constants.
    Jet(fn(&[Jet], &[f64], &mut [Jet])),
}

/// Each kernel's form, by id: read once from the names they are registered under.
fn form_of(kid: usize) -> Option<Form> {
    static FORMS: std::sync::OnceLock<Vec<Option<Form>>> = std::sync::OnceLock::new();
    FORMS.get_or_init(|| KERNELS.iter().map(form).collect()).get(kid).copied().flatten()
}

/// A kernel's form, by the name it is registered under — `None` for one that has none yet.
fn form(kn: &Kernel) -> Option<Form> {
    use Form::*;
    Some(match kn.name {
        "coincident" | "midpoint" | "horizontal" | "vertical" | "radius" | "equal_radius"
        | "ordinate_u" | "ordinate_v" | "annular_distance" | "radius_free"
        | "ordinate_u_free" | "ordinate_v_free" | "annular_distance_free" | "coincident3"
        | "midpoint3" | "project" => Affine,
        "distance" => Jet(distance),
        "distance_free" => Jet(distance_free),
        "parallel" => Jet(parallel),
        "perpendicular" => Jet(perpendicular),
        "angle" => Jet(angle),
        "angle_free" => Jet(angle_free),
        "equal_length" => Jet(equal_length),
        "point_on_line" => Jet(point_on_line),
        "point_on_circle" => Jet(point_on_circle),
        "tangent_line_circle" => Jet(tangent_line_circle),
        "tangent_circle_circle" => Jet(tangent_circle_circle),
        "tangent_arc_line" => Jet(tangent_arc_line),
        "symmetric" => Jet(symmetric),
        "parallel_distance" => Jet(parallel_distance),
        "parallel_distance_free" => Jet(parallel_distance_free),
        "point_line_distance" => Jet(point_line_distance),
        "point_line_distance_free" => Jet(point_line_distance_free),
        "point_line_magnitude" => Jet(point_line_magnitude),
        "point_line_magnitude_free" => Jet(point_line_magnitude_free),
        "parallel_magnitude" => Jet(parallel_magnitude),
        "parallel_magnitude_free" => Jet(parallel_magnitude_free),
        "arc_length" => Jet(arc_length),
        "ordinate_line" => Jet(ordinate_line),
        "ordinate_line_free" => Jet(ordinate_line_free),
        "arc_length_free" => Jet(arc_length_free),
        "equal_angle" => Jet(equal_angle),
        // the kernels in space are written once over `Num`, their forms with them
        name => return crate::kernels::num_form(name).map(Jet),
    })
}

/// Whether kernel `kid`'s form is affine: its second order is zero, so a derivative of it along
/// any motion has no Hessian to read (`kernels::dual_kernel`).
pub fn is_affine(kid: usize) -> bool {
    matches!(form_of(kid), Some(Form::Affine))
}

/// Whether kernel `kid` has a form — what a trace's rows are asked before a curvature is stated
/// against it.
pub fn has_form(kid: usize) -> bool {
    form_of(kid).is_some()
}

/// One row of kernel `kid` along `x + wε`, into `r` (`n_res` jets) — the path built into `path`,
/// scratch the caller keeps; `false` where the kernel has no form.  What a derivative row and an
/// energy's Hessian both read a row's second order through (`kernels::dual_kernel`,
/// `variational`).
pub fn along(kid: usize, x: &[f64], w: &[f64], k: &[f64], path: &mut Vec<Jet>, r: &mut [Jet],
             jrow: &mut Vec<f64>) -> bool {
    path.clear();
    path.extend(x.iter().zip(w).map(|(&xi, &wi)| Jet::from(&[xi, wi])));
    residual(kid, path, k, r, jrow)
}

/// One row of kernel `kid` over the path `v` (a `Jet` per column), into `r` (`n_res` jets).
/// `false` where the kernel has no form; `jrow` is scratch an affine form's Jacobian is read into.
pub fn residual(kid: usize, v: &[Jet], k: &[f64], r: &mut [Jet], jrow: &mut Vec<f64>) -> bool {
    let (Some(kn), Some(f)) = (KERNELS.get(kid), form_of(kid)) else { return false };
    if v.len() != kn.n_par || r.len() != kn.n_res {
        return false;
    }
    match f {
        Form::Jet(f) => f(v, k, r),
        Form::Affine => {
            // an affine kernel is a handful of columns: its point and residuals on the stack
            const W: usize = 16;
            if kn.n_par > W || kn.n_res > W {
                return false;
            }
            let (mut v0, mut r0) = ([0.0f64; W], [0.0f64; W]);
            for (x, j) in v0.iter_mut().zip(v) {
                *x = j.0[0];
            }
            let v0 = &v0[..kn.n_par];
            (kn.res)(1, v0, k, &mut r0[..kn.n_res]);
            jrow.clear();
            jrow.resize(kn.n_res * kn.n_par, 0.0);
            kn.jac_into(v0, k, jrow);
            for t in 0..kn.n_res {
                let mut out = [0.0; ORDER];
                out[0] = r0[t];
                for (c, col) in v.iter().enumerate() {
                    let g = jrow[t * kn.n_par + c];
                    for o in 1..ORDER {
                        out[o] += g * col.0[o];
                    }
                }
                r[t] = Jet(out);
            }
        }
    }
    true
}

/* -- the nonlinear forms, each beside the `f64` residual it reads ------------------------- */

const TURN: f64 = 2.0 * std::f64::consts::PI;

/// `kernels::wrap_turn` on the constant term: the lap is a constant, so the rest is untouched.
fn wrap_turn(a: Jet) -> Jet {
    let a0 = a.0[0];
    a.shift(-TURN * (a0 * (1.0 / TURN)).round())
}

fn dirs(v: &[Jet]) -> (Jet, Jet, Jet, Jet) {
    (v[2] - v[0], v[3] - v[1], v[6] - v[4], v[7] - v[5])
}

fn dist_sq(v: &[Jet]) -> Jet {
    let (dx, dy) = (v[0] - v[2], v[1] - v[3]);
    dx * dx + dy * dy
}

/// The free column's value, `m·a + c`.
fn free_dim(v: &[Jet], k: &[f64], at: usize) -> Jet {
    (v[at] * k[0]).shift(k[1])
}

/// The directed angle from the first line's direction to the second's, unwrapped.
fn bearing_gap(v: &[Jet]) -> Jet {
    let (d1x, d1y, d2x, d2y) = dirs(v);
    let (dot, cross) = (d1x * d2x + d1y * d2y, d1x * d2y - d1y * d2x);
    Jet::atan2(cross, dot)
}

fn angle_gap(v: &[Jet], theta: Jet) -> Jet {
    wrap_turn(bearing_gap(v) - theta)
}

fn parallel_gap(v: &[Jet]) -> Jet {
    let (d1x, d1y) = (v[2] - v[0], v[3] - v[1]);
    let (wx, wy) = (v[4] - v[0], v[5] - v[1]);
    (d1x * wy - d1y * wx) / Jet::line_len(d1x, d1y)
}

fn point_line_gap(v: &[Jet]) -> Jet {
    let (dx, dy) = (v[4] - v[2], v[5] - v[3]);
    let (wx, wy) = (v[0] - v[2], v[1] - v[3]);
    (dx * wy - dy * wx) / Jet::line_len(dx, dy)
}

/// How far `q` stands from `p` along the line `a → b`, all four in one view: `ordinate_line`'s
/// columns (p, q, a, b).
fn ordinate_gap(v: &[Jet]) -> Jet {
    let (dx, dy) = (v[6] - v[4], v[7] - v[5]);
    ((v[2] - v[0]) * dx + (v[3] - v[1]) * dy) / Jet::line_len(dx, dy)
}

fn ordinate_line(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = ordinate_gap(v).shift(-k[0]);
}

fn ordinate_line_free(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = ordinate_gap(v) - free_dim(v, k, 8);
}

fn arc_sweep(v: &[Jet]) -> Jet {
    let (ux, uy) = (v[2] - v[0], v[3] - v[1]);
    let (wx, wy) = (v[4] - v[0], v[5] - v[1]);
    let th = Jet::atan2(wy, wx) - Jet::atan2(uy, ux);
    if th.0[0] <= 0.0 { th.shift(TURN) } else { th }
}

fn distance(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = dist_sq(v).shift(-k[0] * k[0]);
}

fn distance_free(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    let d = free_dim(v, k, 4);
    r[0] = dist_sq(v) - d * d;
}

fn parallel(v: &[Jet], _k: &[f64], r: &mut [Jet]) {
    let (d1x, d1y, d2x, d2y) = dirs(v);
    r[0] = d1x * d2y - d1y * d2x;
}

fn perpendicular(v: &[Jet], _k: &[f64], r: &mut [Jet]) {
    let (d1x, d1y, d2x, d2y) = dirs(v);
    r[0] = d1x * d2x + d1y * d2y;
}

fn angle(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = angle_gap(v, Jet::constant(k[0]));
}

fn angle_free(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = angle_gap(v, free_dim(v, k, 8));
}

fn equal_angle(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = wrap_turn(bearing_gap(v) - bearing_gap(&v[8..]) * k[0]);
}

fn equal_length(v: &[Jet], _k: &[f64], r: &mut [Jet]) {
    let (d1x, d1y, d2x, d2y) = dirs(v);
    r[0] = d1x * d1x + d1y * d1y - d2x * d2x - d2y * d2y;
}

fn point_on_line(v: &[Jet], _k: &[f64], r: &mut [Jet]) {
    let (dx, dy) = (v[4] - v[2], v[5] - v[3]);
    let (wx, wy) = (v[0] - v[2], v[1] - v[3]);
    r[0] = dx * wy - dy * wx;
}

fn point_on_circle(v: &[Jet], _k: &[f64], r: &mut [Jet]) {
    let (ux, uy) = (v[0] - v[2], v[1] - v[3]);
    r[0] = ux * ux + uy * uy - v[4] * v[4];
}

fn tangent_line_circle(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    let (dx, dy) = (v[2] - v[0], v[3] - v[1]);
    let (wx, wy) = (v[4] - v[0], v[5] - v[1]);
    r[0] = (dx * wy - dy * wx) / Jet::line_len(dx, dy) - v[6] * k[0];
}

fn tangent_circle_circle(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    let (ux, uy) = (v[0] - v[3], v[1] - v[4]);
    let rr = v[2] + v[5] * k[0];
    r[0] = ux * ux + uy * uy - rr * rr;
}

fn tangent_arc_line(v: &[Jet], _k: &[f64], r: &mut [Jet]) {
    r[0] = (v[0] - v[2]) * (v[6] - v[4]) + (v[1] - v[3]) * (v[7] - v[5]);
}

fn symmetric(v: &[Jet], _k: &[f64], r: &mut [Jet]) {
    let (dx, dy) = (v[6] - v[4], v[7] - v[5]);
    let mx = v[0] + v[2] - v[4] * 2.0;
    let my = v[1] + v[3] - v[5] * 2.0;
    r[0] = dx * my - dy * mx;
    r[1] = (v[2] - v[0]) * dx + (v[3] - v[1]) * dy;
}

fn parallel_distance(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = parallel_gap(v).shift(-k[0]);
}

fn parallel_distance_free(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = parallel_gap(v) - free_dim(v, k, 8);
}

fn point_line_distance(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = point_line_gap(v).shift(-k[0]);
}

fn point_line_distance_free(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = point_line_gap(v) - free_dim(v, k, 6);
}

fn point_line_magnitude(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = point_line_gap(v).abs().shift(-k[0]);
}

fn point_line_magnitude_free(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = point_line_gap(v).abs() - free_dim(v, k, 6);
}

fn parallel_magnitude(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = parallel_gap(v).abs().shift(-k[0]);
}

fn parallel_magnitude_free(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = parallel_gap(v).abs() - free_dim(v, k, 8);
}

fn arc_length(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = (v[6] * arc_sweep(v)).shift(-k[0]);
}

fn arc_length_free(v: &[Jet], k: &[f64], r: &mut [Jet]) {
    r[0] = v[6] * arc_sweep(v) - free_dim(v, k, 7);
}
