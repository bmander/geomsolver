//! The general integrator (#339 D1): Gragg–Bulirsch–Stoer over a state of any width
//! (`gcs_core::ode`).  Linear flows against `exp(At)`, their sensitivities `exp(At)` itself; the
//! extrapolation's order, measured column by column; the variational equations against
//! differences, of one step (exact for the step taken) and of a march; dense output between
//! steps; and a march refused where its rate cannot be had.  The free curves ported onto it are
//! held to their present results by `tests/{extremal,catenary,slide}.rs`.

use gcs_core::ode::{self, Rate, Refusal, Settings};

/// `y' = A y`, `A` row-major.
struct Linear {
    n: usize,
    a: Vec<f64>,
}

impl Rate for Linear {
    type Branch = ();
    fn width(&self) -> usize {
        self.n
    }
    fn rate(&mut self, _u: f64, y: &[f64], f: &mut [f64], jac: Option<&mut [f64]>) -> bool {
        for i in 0..self.n {
            f[i] = (0..self.n).map(|j| self.a[i * self.n + j] * y[j]).sum();
        }
        if let Some(j) = jac {
            j.copy_from_slice(&self.a);
        }
        true
    }
    fn branch(&self) {}
    fn set_branch(&mut self, _: ()) {}
}

/// A rate given by a closure of `(u, y, f, jac)`.
struct By<F>(usize, F);

impl<F: FnMut(f64, &[f64], &mut [f64], Option<&mut [f64]>) -> bool> Rate for By<F> {
    type Branch = ();
    fn width(&self) -> usize {
        self.0
    }
    fn rate(&mut self, u: f64, y: &[f64], f: &mut [f64], jac: Option<&mut [f64]>) -> bool {
        (self.1)(u, y, f, jac)
    }
    fn branch(&self) {}
    fn set_branch(&mut self, _: ()) {}
}

/// The pendulum, `θ'' = −sin θ`.
fn pendulum() -> By<impl FnMut(f64, &[f64], &mut [f64], Option<&mut [f64]>) -> bool> {
    By(2, |_u: f64, y: &[f64], f: &mut [f64], jac: Option<&mut [f64]>| {
        f[0] = y[1];
        f[1] = -y[0].sin();
        if let Some(j) = jac {
            j.copy_from_slice(&[0.0, 1.0, -y[0].cos(), 0.0]);
        }
        true
    })
}

/// Lorenz's flow, three wide and not autonomous in its forcing: `x' = σ(y − x) + sin u`.
fn lorenz() -> By<impl FnMut(f64, &[f64], &mut [f64], Option<&mut [f64]>) -> bool> {
    let (s, r, b) = (10.0, 28.0, 8.0 / 3.0);
    By(3, move |u: f64, y: &[f64], f: &mut [f64], jac: Option<&mut [f64]>| {
        f[0] = s * (y[1] - y[0]) + u.sin();
        f[1] = y[0] * (r - y[2]) - y[1];
        f[2] = y[0] * y[1] - b * y[2];
        if let Some(j) = jac {
            j.copy_from_slice(&[-s, s, 0.0, r - y[2], -1.0, -y[0], y[1], y[0], -b]);
        }
        true
    })
}

/// `got` within `tol` of `want`, relative where `want` is large.
fn close(got: f64, want: f64, tol: f64, what: &str) {
    assert!((got - want).abs() <= tol * (1.0 + want.abs()), "{what}: {got} against {want}");
}

/// The `n × n` identity, row-major.
fn identity(n: usize) -> Vec<f64> {
    (0..n * n).map(|k| if k / n == k % n { 1.0 } else { 0.0 }).collect()
}

/// `A v`, `A` row-major and as many columns as `v` has entries.
fn mat_vec(a: &[f64], v: &[f64]) -> Vec<f64> {
    a.chunks_exact(v.len()).map(|r| r.iter().zip(v).map(|(a, v)| a * v).sum()).collect()
}

/// The augmented start: `y0`, then the identity (sensitivities to the initial value).
fn seeded(y0: &[f64]) -> Vec<f64> {
    [y0, &identity(y0.len())].concat()
}

/// `exp(A t)` by its Taylor series, scaled and squared — an independent construction.
fn expm(a: &[f64], n: usize, t: f64) -> Vec<f64> {
    let mul = |x: &[f64], y: &[f64]| -> Vec<f64> {
        (0..n * n).map(|ij| (0..n).map(|k| x[ij / n * n + k] * y[k * n + ij % n]).sum()).collect()
    };
    let norm = a.iter().map(|v| v.abs()).sum::<f64>() * t.abs();
    let sq = (norm.max(1.0).log2().ceil() as i32 + 4).max(0);
    let at: Vec<f64> = a.iter().map(|v| v * t / 2f64.powi(sq)).collect();
    let mut e = identity(n);
    let mut term = e.clone();
    for k in 1..30 {
        term = mul(&term, &at).iter().map(|v| v / k as f64).collect();
        e.iter_mut().zip(&term).for_each(|(x, d)| *x += d);
    }
    for _ in 0..sq {
        e = mul(&e, &e);
    }
    e
}

/// The flows: a spiral, a Jordan block, a damped oscillator coupled to a growth.
fn linear_flows() -> Vec<Linear> {
    vec![
        Linear { n: 2, a: vec![-0.1, 2.0, -2.0, -0.1] },
        Linear { n: 3, a: vec![0.3, 1.0, 0.0, 0.0, 0.3, 1.0, 0.0, 0.0, 0.3] },
        Linear {
            n: 4,
            a: vec![0.0, 1.0, 0.0, 0.0, -4.0, -0.4, 0.5, 0.0, 0.0, 0.0, 0.2, 1.0, 0.0, 0.0, -1.0, 0.2],
        },
    ]
}

/// A linear flow marched forwards and back lands on `exp(At) y0`, and its sensitivities on
/// `exp(At)` itself, closed forms independent of the integrator.
#[test]
fn a_linear_flow_is_its_exponential() {
    for mut lin in linear_flows() {
        let n = lin.n;
        let y0: Vec<f64> = (0..n).map(|i| 1.0 - 0.3 * i as f64).collect();
        for span in [5.0, -3.0] {
            let settings = Settings::default();
            let path = ode::march(&mut lin, 0.5, &seeded(&y0), n, span, &vec![1.0; n], &settings).unwrap();
            let end = path.nodes.last().unwrap();
            assert_eq!(end.u, 0.5 + span);
            let e = expm(&lin.a, n, span);
            for (i, want) in mat_vec(&e, &y0).into_iter().enumerate() {
                close(end.y[i], want, 1e-11, &format!("n {n} span {span} y{i}"));
            }
            for (ik, &want) in e.iter().enumerate() {
                close(end.y[n + ik], want, 1e-11, &format!("n {n} span {span} Φ{ik}"));
            }
        }
    }
}

/// A rate that reads the formal, `y' = u y`: `y = y0 exp((u² − u0²)/2)`, its sensitivity the
/// same factor.
#[test]
fn the_formal_is_threaded_through_the_substeps() {
    let mut r = By(1, |u: f64, y: &[f64], f: &mut [f64], jac: Option<&mut [f64]>| {
        f[0] = u * y[0];
        if let Some(j) = jac {
            j[0] = u;
        }
        true
    });
    for (u0, span) in [(-1.0, 3.0), (2.0, -2.5)] {
        let path = ode::march(&mut r, u0, &[0.7, 1.0], 1, span, &[1.0], &Settings::default()).unwrap();
        let u1: f64 = u0 + span;
        let g = ((u1 * u1 - u0 * u0) / 2.0).exp();
        let end = path.nodes.last().unwrap();
        close(end.y[0], 0.7 * g, 1e-12, &format!("from {u0} over {span}"));
        close(end.y[1], g, 1e-12, &format!("from {u0} over {span}, Φ"));
    }
}

/// One step's error against the closed form on the oscillator `y'' = −y` (an entire flow, so
/// the error's leading term rules while it still stands well above rounding), with `columns` of
/// the table.
fn one_step_error(columns: usize, h: f64) -> f64 {
    let mut r = Linear { n: 2, a: vec![0.0, 1.0, -1.0, 0.0] };
    let settings = Settings { columns, ..Settings::default() };
    let mut w = ode::Work::new(2, 0, &settings);
    let mut out = [0.0; 2];
    ode::step(&mut r, &mut w, 0.0, &[1.0, 0.0], h, &[1.0, 1.0], &mut out).unwrap();
    (out[0] - h.cos()).abs().max((out[1] + h.sin()).abs())
}

/// Each column of the extrapolation adds two orders: one step with `k` columns carries an error
/// of order `2k + 1`, measured by halving the step (4.90, 6.94, 8.96, 10.97, 12.98, 14.99).
#[test]
fn the_extrapolations_order_is_measured() {
    for k in 2..=7 {
        let (e1, e2) = (one_step_error(k, 2.0), one_step_error(k, 1.0));
        assert!(e2 > 1e-13, "{k} columns: the error {e2} is rounding's");
        let order = (e1 / e2).log2();
        let want = (2 * k + 1) as f64;
        assert!((order - want).abs() < 0.15, "{k} columns: order {order}, not {want}");
    }
}

/// `Φ` as a march carries it, and the end state, from `y0` over `span`.
fn marched<R: Rate<Branch = ()>>(r: &mut R, y0: &[f64], span: f64) -> (Vec<f64>, Vec<f64>) {
    let n = y0.len();
    let path = ode::march(r, 0.0, &seeded(y0), n, span, &vec![1.0; n], &Settings::default()).unwrap();
    let end = &path.nodes.last().unwrap().y;
    (end[..n].to_vec(), end[n..].to_vec())
}

/// One step's end state and `Φ`, at a fixed step.
fn stepped<R: Rate<Branch = ()>>(r: &mut R, y0: &[f64], h: f64) -> (Vec<f64>, Vec<f64>) {
    let n = y0.len();
    let mut w = ode::Work::new(n, n, &Settings::default());
    let mut out = vec![0.0; n * (1 + n)];
    ode::step(r, &mut w, 0.0, &seeded(y0), h, &vec![1.0; n], &mut out).unwrap();
    (out[..n].to_vec(), out[n..].to_vec())
}

/// `Φ` against central differences of what carried it, each start moved by `e`.
fn against_differences(
    phi: &[f64],
    y0: &[f64],
    e: f64,
    tol: f64,
    end: impl Fn(&[f64]) -> Vec<f64>,
    what: &str,
) {
    let n = y0.len();
    for k in 0..n {
        let moved = |sign: f64| {
            let mut y = y0.to_vec();
            y[k] += sign * e;
            end(&y)
        };
        let (p, m) = (moved(1.0), moved(-1.0));
        for i in 0..n {
            close(phi[i * n + k], (p[i] - m[i]) / (2.0 * e), tol, &format!("{what} Φ{i}{k}"));
        }
    }
}

/// The variational equations are the exact derivative of the step taken: one step's `Φ` is its
/// differences to their own truncation (a difference of `1e-3` agrees to `6e-11` on Lorenz's
/// step; a smaller one only sees rounding), and a march's is the march's.
#[test]
fn the_sensitivities_are_the_differences() {
    let pend = [2.0, 0.3];
    let lor = [1.0, 2.0, 20.0];
    let step_p = |y: &[f64]| stepped(&mut pendulum(), y, 1.5);
    let step_l = |y: &[f64]| stepped(&mut lorenz(), y, 0.05);
    let march_p = |y: &[f64]| marched(&mut pendulum(), y, 12.0);
    let march_l = |y: &[f64]| marched(&mut lorenz(), y, 1.0);
    against_differences(&step_p(&pend).1, &pend, 1e-5, 1e-9, |y| step_p(y).0, "pendulum, a step");
    against_differences(&step_l(&lor).1, &lor, 1e-3, 1e-9, |y| step_l(y).0, "Lorenz, a step");
    against_differences(&march_p(&pend).1, &pend, 1e-4, 1e-6, |y| march_p(y).0, "pendulum, a march");
    against_differences(&march_l(&lor).1, &lor, 1e-4, 1e-6, |y| march_l(y).0, "Lorenz, a march");
}

/// A seed of fewer columns than the state carries `Φ(u) Φ(u0)`: the sensitivity to a direction
/// alone, the same numbers as the whole `Φ` times it.
#[test]
fn a_seed_may_be_any_columns() {
    let y0 = [1.0, 2.0, 20.0];
    let v = [0.6, -0.8, 0.0];
    let (_, phi) = marched(&mut lorenz(), &y0, 0.8);
    let seed = [y0, v].concat();
    let path = ode::march(&mut lorenz(), 0.0, &seed, 1, 0.8, &[1.0; 3], &Settings::default()).unwrap();
    let end = path.nodes.last().unwrap();
    for (i, want) in mat_vec(&phi, &v).into_iter().enumerate() {
        close(end.y[3 + i], want, 1e-9, &format!("Φv{i}"));
    }
}

/// Between steps, the state and its sensitivities are one step from the node before — on a
/// linear flow, `exp(A(u − u0))` there too, forwards and back; at a node, the node.
#[test]
fn dense_output_is_the_flow_between_steps() {
    let mut lin = linear_flows().pop().unwrap();
    let n = lin.n;
    let y0 = [1.0, -0.5, 0.25, 2.0];
    for span in [6.0, -4.0] {
        let settings = Settings::default();
        let path = ode::march(&mut lin, 1.0, &seeded(&y0), n, span, &[1.0; 4], &settings).unwrap();
        assert!(path.nodes.len() > 3, "a march of several steps");
        for q in 0..=40 {
            let u = 1.0 + span * q as f64 / 40.0;
            let at = path.at(&mut lin, u, true).unwrap();
            let state = path.at(&mut lin, u, false).unwrap();
            assert_eq!(state.y.len(), n);
            let e = expm(&lin.a, n, u - 1.0);
            for (i, want) in mat_vec(&e, &y0).into_iter().enumerate() {
                close(at.y[i], want, 1e-11, &format!("at {u}: y{i}"));
                close(state.y[i], at.y[i], 1e-13, &format!("at {u}: y{i} without Φ"));
            }
            for (ik, &want) in e.iter().enumerate() {
                close(at.y[n + ik], want, 1e-11, &format!("at {u}: Φ{ik}"));
            }
        }
        for nd in &path.nodes {
            assert_eq!(&path.at(&mut lin, nd.u, true).unwrap(), nd);
        }
    }
}

/// `y' = r` with `r² = y`, `r` found by Newton from where it last was: the branch is carried,
/// so the march started on `r = −√y` stays on it, `y = (1 − u/2)²` — until the two roots meet
/// at `u = 2`, where the rate is not to be had and the march is refused.
struct Root {
    r: f64,
}

impl Rate for Root {
    type Branch = f64;
    fn width(&self) -> usize {
        1
    }
    fn rate(&mut self, _u: f64, y: &[f64], f: &mut [f64], jac: Option<&mut [f64]>) -> bool {
        let mut r = self.r;
        for _ in 0..60 {
            let d = (r * r - y[0]) / (2.0 * r);
            if !d.is_finite() {
                return false;
            }
            r -= d;
            if d.abs() <= 1e-15 * r.abs() {
                if r * self.r <= 0.0 {
                    return false; // the root crossed to the other branch
                }
                self.r = r;
                f[0] = r;
                if let Some(j) = jac {
                    j[0] = 1.0 / (2.0 * r);
                }
                return true;
            }
        }
        false
    }
    fn branch(&self) -> f64 {
        self.r
    }
    fn set_branch(&mut self, r: f64) {
        self.r = r;
    }
}

#[test]
fn a_branch_is_carried_and_its_end_refused() {
    let mut root = Root { r: -1.0 };
    let path = ode::march(&mut root, 0.0, &[1.0, 1.0], 1, 1.6, &[1.0], &Settings::default()).unwrap();
    for nd in &path.nodes {
        close(nd.y[0], (1.0 - nd.u / 2.0).powi(2), 1e-12, &format!("at {}", nd.u));
        // where the step's last evaluation left it: a start for the next, on the same side
        let off = nd.branch + (1.0 - nd.u / 2.0);
        assert!(nd.branch < 0.0 && off.abs() <= 0.1, "the branch at {}: {}", nd.u, nd.branch);
        // `∂y/∂y0` with `y = (√y0 − u/2)²`
        close(nd.y[1], 1.0 - nd.u / 2.0, 1e-10, &format!("∂y/∂y0 at {}", nd.u));
    }
    let mut root = Root { r: -1.0 };
    let refused = ode::march(&mut root, 0.0, &[1.0], 0, 3.0, &[1.0], &Settings::default()).unwrap_err();
    assert!((refused.at() - 2.0).abs() < 1e-3, "refused at {}", refused.at());
}

/// A rate undefined past a place is refused there, not at the start of the step that first
/// crossed it: the step closes in to the floor, and the march stops at the last place it reached
/// (past the edge by at most the last step's final substep, which evaluates nothing beyond it).
/// A flow that will not meet its tolerance in bounded steps is refused too.
#[test]
fn a_march_is_refused_where_it_cannot_go_on() {
    let mut edge = By(1, |u: f64, _y: &[f64], f: &mut [f64], _jac: Option<&mut [f64]>| {
        f[0] = u.cos();
        u <= 1.0
    });
    let refused = ode::march(&mut edge, 0.0, &[0.0], 0, 2.0, &[1.0], &Settings::default()).unwrap_err();
    assert!(matches!(refused, Refusal::Undefined { .. }), "{refused:?}");
    assert!((refused.at() - 1.0).abs() < 1e-4, "refused at {}", refused.at());
    let mut fast = Linear { n: 2, a: vec![0.0, 1.0, -1e6, 0.0] };
    let settings = Settings { steps_max: 50, ..Settings::default() };
    let refused = ode::march(&mut fast, 0.0, &[1.0, 0.0], 0, 1.0, &[1.0, 1e3], &settings).unwrap_err();
    assert!(matches!(refused, Refusal::Steps { at } if at > 0.0 && at < 1.0), "{refused:?}");
}

