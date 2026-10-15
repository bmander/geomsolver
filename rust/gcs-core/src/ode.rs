//! A rate integrated along a formal (#339 D1): `y' = f(u, y)` over a state of any width, from an
//! initial value, by Gragg–Bulirsch–Stoer.
//!
//! The explicit midpoint rule over an even number of substeps has an error even in the substep
//! (Gragg), so the results of `STEPS` substep counts extrapolate to zero substep by
//! Aitken–Neville, each column two orders more; the last two columns tell the error, and the
//! step adapts to the tolerance.  A smooth flow at a tight tolerance is what it is best at, it
//! has no tableau to carry, and every operation is linear in the substeps' results — so the
//! **variational equations** `Φ' = (∂f/∂y) Φ`, integrated through the same substeps beside the
//! state, are the exact derivative of the step the flow took, not an estimate of the flow's.
//!
//! The caller supplies the rate and its Jacobian (`Rate`); nothing is differenced.  A rate that
//! follows a branch — solves an algebraic condition by Newton from where it last was, as a free
//! curve's direction is — carries it as its `Branch`: taken where a step starts, given back
//! before each column, so every column follows the same branch from the same place, and kept
//! with each node.  The step sequence follows the tolerance alone (`Settings`), so a march is
//! the same on every target.  A march is refused, never followed, where the rate cannot be had
//! or the tolerance cannot be met in bounded steps (`Refusal`).
//!
//! **Dense output** between steps is one more step from the node before (`Path::at`): exact to
//! the tolerance, with the sensitivities the same arithmetic gives the nodes, where an
//! interpolant's would be its own derivative and not the flow's.

#[allow(unused_imports)]
use crate::fmath::Det;

/// A flow `y' = f(u, y)`, and what it carries from one evaluation to the next.
pub trait Rate {
    /// The branch the rate follows (`()` for one that follows none): where it last found the
    /// root of a condition it solves, which the next evaluation starts from.
    type Branch: Copy;

    /// The state's width.
    fn width(&self) -> usize;

    /// `f(u, y)` into `f` (`width` long), and, asked for, its Jacobian `∂f/∂y` into `jac`
    /// (`width × width`, row-major) — from the branch as it stands, which it moves to where
    /// this evaluation found it.  `false` where the rate cannot be had.
    fn rate(&mut self, u: f64, y: &[f64], f: &mut [f64], jac: Option<&mut [f64]>) -> bool;

    fn branch(&self) -> Self::Branch;

    fn set_branch(&mut self, b: Self::Branch);
}

/// The substep counts of the extrapolation, one per column: seven columns, to order fourteen.
pub const STEPS: [usize; 7] = [2, 4, 6, 8, 10, 12, 14];

/// The relative tolerance one step is held to, well below "solved" (relative residual 1e-6), so
/// an integrated quantity is never the reason a solve misses.
pub const TOL: f64 = 1e-13;

/// The most steps one march may take: a flow needing more is followed nowhere near its answer
/// (or is stiff), and is refused.
pub const STEPS_MAX: usize = 4000;

/// How a march is held: its tolerance, its columns, its bound on steps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
    /// The error one step may carry, over the state, each component a fraction of its scale.
    pub tol: f64,
    /// How many of `STEPS` the extrapolation takes (2 to 7): its order is twice that.
    pub columns: usize,
    pub steps_max: usize,
}

impl Default for Settings {
    fn default() -> Settings {
        Settings { tol: TOL, columns: STEPS.len(), steps_max: STEPS_MAX }
    }
}

impl Settings {
    /// The columns taken: `columns`, within 2 (one to extrapolate, one to tell the error) and
    /// `STEPS`.
    fn columns(&self) -> usize {
        self.columns.clamp(2, STEPS.len())
    }

    /// The next step's size after one whose relative error was `err`: the estimate, the last
    /// two columns' difference, is of order `2·columns − 2`.
    pub fn next_h(&self, h: f64, err: f64) -> f64 {
        let order = (2 * self.columns() - 2) as f64;
        let f = if err == 0.0 {
            4.0
        } else {
            (0.9 * (self.tol / err).dpowf(1.0 / (order + 1.0))).clamp(0.2, 4.0)
        };
        h * f
    }
}

/// Why a march stopped short, and where.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Refusal {
    /// The rate could not be had, or came out other than finite, in any step from `at`, the
    /// last place reached, down to a step at the floor.
    Undefined { at: f64 },
    /// The tolerance was not met in `Settings::steps_max` steps; `at` is how far it got.
    Steps { at: f64 },
}

impl Refusal {
    /// Where along the formal the march stopped.
    pub fn at(&self) -> f64 {
        match *self {
            Refusal::Undefined { at } | Refusal::Steps { at } => at,
        }
    }
}

/// The scratch one step uses, for a state `n` wide carrying `m` sensitivities: the augmented
/// state is `n + n·m` long, the state then `Φ` (`n × m`, row-major).
pub struct Work {
    n: usize,
    m: usize,
    columns: usize,
    jac: Vec<f64>,
    f0: Vec<f64>,
    f: Vec<f64>,
    prev: Vec<f64>,
    cur: Vec<f64>,
    /// The extrapolation table's row being built and the row before it, `columns` entries each.
    row: Vec<f64>,
    last: Vec<f64>,
}

impl Work {
    pub fn new(n: usize, m: usize, settings: &Settings) -> Work {
        let len = n * (1 + m);
        let columns = settings.columns();
        Work {
            n,
            m,
            columns,
            jac: vec![0.0; if m > 0 { n * n } else { 0 }],
            f0: vec![0.0; len],
            f: vec![0.0; len],
            prev: vec![0.0; len],
            cur: vec![0.0; len],
            row: vec![0.0; columns * len],
            last: vec![0.0; columns * len],
        }
    }

    /// The augmented state's length.
    fn len(&self) -> usize {
        self.n * (1 + self.m)
    }
}

/// The augmented rate at `(u, y)`: `f`, then `(∂f/∂y) Φ` where sensitivities are carried.
fn augmented<R: Rate>(
    rate: &mut R,
    n: usize,
    m: usize,
    jac: &mut [f64],
    u: f64,
    y: &[f64],
    out: &mut [f64],
) -> bool {
    if m == 0 {
        return rate.rate(u, &y[..n], &mut out[..n], None);
    }
    if !rate.rate(u, &y[..n], &mut out[..n], Some(jac)) {
        return false;
    }
    let (phi, out) = (&y[n..n + n * m], &mut out[n..n + n * m]);
    // a small state's sum at a length known in advance (a free curve's is 4)
    match n {
        1 => product::<1>(n, m, jac, phi, out),
        2 => product::<2>(n, m, jac, phi, out),
        3 => product::<3>(n, m, jac, phi, out),
        4 => product::<4>(n, m, jac, phi, out),
        _ => product::<0>(n, m, jac, phi, out),
    }
    true
}

/// `A Φ` into `out`, `A` `n × n` and `Φ` `n × m`.  `N` is `n` where it is known in advance (0
/// where it is not), and a square `Φ` (the sensitivities to the initial value) is `N × N`: the
/// sums' lengths constant, so they unroll.
fn product<const N: usize>(n: usize, m: usize, jac: &[f64], phi: &[f64], out: &mut [f64]) {
    if N > 0 && m == N {
        rows(N, N, jac, phi, out)
    } else {
        rows(if N > 0 { N } else { n }, m, jac, phi, out)
    }
}

/// `A Φ` row by row, each entry summed over `A`'s row in order.
#[inline(always)]
fn rows(n: usize, m: usize, jac: &[f64], phi: &[f64], out: &mut [f64]) {
    let (jac, phi, out) = (&jac[..n * n], &phi[..n * m], &mut out[..n * m]);
    for i in 0..n {
        for k in 0..m {
            let mut v = 0.0;
            for j in 0..n {
                v += jac[i * n + j] * phi[j * m + k];
            }
            out[i * m + k] = v;
        }
    }
}

/// The explicit midpoint rule over `substeps` of one step `h` from `(u0, y0)`, the rate at
/// `y0` given (`w.f0`, every column's first) and the branch it was found on (`b0`), into
/// `w.cur`.
fn midpoint<R: Rate>(
    rate: &mut R,
    w: &mut Work,
    b0: R::Branch,
    u0: f64,
    y0: &[f64],
    h: f64,
    substeps: usize,
) -> bool {
    let (n, m, len) = (w.n, w.m, w.len());
    let hs = h / substeps as f64;
    rate.set_branch(b0);
    w.prev[..len].copy_from_slice(&y0[..len]);
    for ((c, &y), &f) in w.cur[..len].iter_mut().zip(&y0[..len]).zip(&w.f0[..len]) {
        *c = y + hs * f;
    }
    for k in 1..substeps {
        if !augmented(rate, n, m, &mut w.jac, u0 + k as f64 * hs, &w.cur, &mut w.f) {
            return false;
        }
        for ((p, c), &f) in w.prev[..len].iter_mut().zip(&mut w.cur[..len]).zip(&w.f[..len]) {
            let next = *p + 2.0 * hs * f;
            *p = *c;
            *c = next;
        }
    }
    true
}

/// One extrapolated step of `h` from `(u0, y0)` (the augmented state, `w.len()` long), from the
/// rate's branch as it stands: the result into `out` and its error estimate over the state, as
/// a fraction of `scale` (`n` long).  Every column is taken (a fixed order), the last two
/// telling the error.  `None` where the flow could not be followed.  The rate is left on the
/// branch its last evaluation found.
pub fn step<R: Rate>(
    rate: &mut R,
    w: &mut Work,
    u0: f64,
    y0: &[f64],
    h: f64,
    scale: &[f64],
    out: &mut [f64],
) -> Option<f64> {
    let (n, m, len, k) = (w.n, w.m, w.len(), w.columns);
    if !augmented(rate, n, m, &mut w.jac, u0, y0, &mut w.f0) {
        return None;
    }
    let b0 = rate.branch();
    for (j, &substeps) in STEPS[..k].iter().enumerate() {
        if !midpoint(rate, w, b0, u0, y0, h, substeps) {
            return None;
        }
        w.row[..len].copy_from_slice(&w.cur[..len]);
        for c in 1..=j {
            let q = substeps as f64 / STEPS[j - c] as f64;
            let r = q * q - 1.0;
            let (done, row) = w.row.split_at_mut(c * len);
            let (a, b) = (&done[(c - 1) * len..], &w.last[(c - 1) * len..c * len]);
            for ((o, &a), &b) in row[..len].iter_mut().zip(a).zip(b) {
                *o = a + (a - b) / r;
            }
        }
        w.last[..(j + 1) * len].copy_from_slice(&w.row[..(j + 1) * len]);
    }
    let (best, next) = (&w.last[(k - 1) * len..k * len], &w.last[(k - 2) * len..(k - 1) * len]);
    let err = (0..n).map(|i| (best[i] - next[i]).abs() / scale[i]).fold(0.0, f64::max);
    out[..len].copy_from_slice(best);
    out[..len].iter().all(|v| v.is_finite()).then_some(err)
}

/// One accepted point of a march: where, the augmented state there, and the branch as the step's
/// last evaluation left it — a start for the next step's, on the branch the march is on, not the
/// root at `u` itself.
#[derive(Clone, Debug, PartialEq)]
pub struct Node<B> {
    pub u: f64,
    /// The state, then — where the march carried them — its sensitivities `Φ` (`n × m`,
    /// row-major).
    pub y: Vec<f64>,
    pub branch: B,
}

/// A march: its nodes in the order taken (`u` decreasing for a march backwards), and what it
/// was held to, which its dense output is held to too.
#[derive(Clone, Debug)]
pub struct Path<B> {
    pub n: usize,
    pub m: usize,
    pub scale: Vec<f64>,
    pub settings: Settings,
    pub nodes: Vec<Node<B>>,
}

/// Integrate `rate` from `(u0, y0)` over `span` (either sign), `y0` the augmented state: the
/// state, then the `m` sensitivities' seed `Φ(u0)` (`n × m`, row-major; the identity for
/// sensitivities to the initial value).  The rate's branch as it stands is the start's.  Each
/// step's error is held to `settings.tol` of `scale` (`n` long, a magnitude per component); the
/// first step is a quarter of the span, and a step at the floor (`1e-12` of it) is taken
/// whatever it carries, so a march ends.  A step whose rate cannot be had is tried shorter,
/// down to the floor, so a refusal is where the rate stops, not where the step that crossed it
/// began.
pub fn march<R: Rate>(
    rate: &mut R,
    u0: f64,
    y0: &[f64],
    m: usize,
    span: f64,
    scale: &[f64],
    settings: &Settings,
) -> Result<Path<R::Branch>, Refusal> {
    let n = rate.width();
    let mut w = Work::new(n, m, settings);
    let len = w.len();
    let mut branch = rate.branch();
    let mut nodes = vec![Node { u: u0, y: y0[..len].to_vec(), branch }];
    let (total, dir) = (span.abs(), if span < 0.0 { -1.0 } else { 1.0 });
    let mut y = y0[..len].to_vec();
    let mut out = vec![0.0; len];
    let mut done = 0.0;
    let mut h = total / 4.0;
    let mut taken = 0;
    while done < total {
        let here = u0 + dir * done;
        if taken >= settings.steps_max {
            return Err(Refusal::Steps { at: here });
        }
        let last = total - done <= h * (1.0 + 1e-12);
        let hh = if last { total - done } else { h };
        rate.set_branch(branch);
        let err = step(rate, &mut w, here, &y, dir * hh, scale, &mut out);
        taken += 1;
        let floor = hh <= total * 1e-12;
        let Some(err) = err else {
            // the rate is not to be had somewhere in the step: closer, until the step is at the
            // floor and the place is found
            if floor {
                return Err(Refusal::Undefined { at: here });
            }
            h = hh * 0.25;
            continue;
        };
        if err <= settings.tol || floor {
            done = if last { total } else { done + hh };
            branch = rate.branch();
            std::mem::swap(&mut y, &mut out);
            let u = if last { u0 + span } else { u0 + dir * done };
            nodes.push(Node { u, y: y.clone(), branch });
            h = settings.next_h(hh, err);
        } else {
            h = settings.next_h(hh, err).min(hh * 0.5);
        }
    }
    Ok(Path { n, m, scale: scale.to_vec(), settings: *settings, nodes })
}

impl<B: Copy> Path<B> {
    /// Which way the march ran: `1` forwards, `-1` backwards (`1` for a march of no span).
    fn dir(&self) -> f64 {
        match self.nodes.last() {
            Some(l) if l.u < self.nodes[0].u => -1.0,
            _ => 1.0,
        }
    }

    /// The state at `u` (within the march's span), one step from the node before it in the
    /// march's direction, with the sensitivities where `sens` (else the state alone).  At a
    /// node, the node.
    pub fn at<R: Rate<Branch = B>>(
        &self,
        rate: &mut R,
        u: f64,
        sens: bool,
    ) -> Result<Node<B>, Refusal> {
        let dir = self.dir();
        let i = self.nodes.partition_point(|nd| dir * (nd.u - u) <= 0.0).saturating_sub(1);
        let nd = &self.nodes[i];
        let m = if sens { self.m } else { 0 };
        let len = self.n * (1 + m);
        let h = u - nd.u;
        if dir * h <= 0.0 {
            return Ok(Node { u: nd.u, y: nd.y[..len].to_vec(), branch: nd.branch });
        }
        let mut w = Work::new(self.n, m, &self.settings);
        let mut out = vec![0.0; len];
        rate.set_branch(nd.branch);
        step(rate, &mut w, nd.u, &nd.y[..len], h, &self.scale, &mut out)
            .ok_or(Refusal::Undefined { at: nd.u })?;
        Ok(Node { u, y: out, branch: rate.branch() })
    }
}
