//! Every kernel's Taylor form is its kernel (`taylor.rs`): read along a random cubic path, the
//! constant term is `res`, the first coefficient is `jac` times the path's velocity, and the
//! second and third are what differences of that exact first derivative say they are.
//!
//! This is the gate `locus` relies on when it states a trace's `C''` and `C'''` as exact: a form
//! that drifted from its kernel would put a curvature contact on a curve the block does not trace.

use gcs_core::kernels::KERNELS;
use gcs_core::rng::Rng;
use gcs_core::taylor::{has_form, residual, Jet, ORDER};

/// The path `v(ε) = Σ a_k ε^k`, at `e`.
fn at(path: &[Jet], e: f64) -> Vec<f64> {
    path.iter().map(|j| j.0.iter().rev().fold(0.0, |acc, &c| acc * e + c)).collect()
}

/// `d/dε r(v(ε))` at `e`, exactly: the kernel's Jacobian times the path's velocity.
fn rate(kid: usize, path: &[Jet], k: &[f64], e: f64) -> Vec<f64> {
    let kn = &KERNELS[kid];
    let v = at(path, e);
    let vel: Vec<f64> = path
        .iter()
        .map(|j| j.0[1] + 2.0 * j.0[2] * e + 3.0 * j.0[3] * e * e)
        .collect();
    let mut jrow = vec![0.0; kn.n_res * kn.n_par];
    (kn.jac)(1, &v, k, &mut jrow);
    (0..kn.n_res)
        .map(|t| (0..kn.n_par).map(|c| jrow[t * kn.n_par + c] * vel[c]).sum())
        .collect()
}

/// A spline contact's constants are its span's knot window and weights, and its parameter sits
/// inside that span: drawn as anything else's are (unsorted knots, a parameter far outside them)
/// the basis is extrapolated past where any drawing reads it, and a central difference of a
/// derivative row there is all cancellation.  So for the derivative rows a spline contact's knots
/// are drawn sorted, its weights near 1, and the column its parameter is in returned, to be put
/// inside the span.  `None` for any other kernel.  (The forms keep the wild draw: a form is a
/// polynomial or a quotient of two, exact wherever it is read, and there the check is of exact
/// rates, which do not cancel — where sensible knots on random points would leave a nearly
/// straight span whose curvature row's second difference is the one thing that does.)
fn spline_contact(name: &str, rng: &mut Rng) -> Option<(usize, Vec<f64>)> {
    let t = match name {
        "point_on_spline" => 2,
        "spline_tangent_line" | "spline_curvature" => 0,
        _ => return None,
    };
    let mut k = vec![0.0];
    for _ in 1..gcs_core::curve::SPAN_K {
        let last = *k.last().unwrap();
        k.push(last + rng.uniform(0.5, 1.5));
    }
    k.extend((0..gcs_core::curve::SPAN_N).map(|_| rng.uniform(0.7, 1.4)));
    Some((t, k))
}

/// A kernel's constants, and its columns adjusted to them: a spline contact's parameter put inside
/// its span (`spline_contact`).
fn constants(name: &str, n_const: usize, rng: &mut Rng, t_at: &mut dyn FnMut(usize, f64)) -> Vec<f64> {
    match spline_contact(name, rng) {
        Some((t, k)) => {
            t_at(t, rng.uniform(k[3], k[4]));
            k
        }
        None => (0..n_const).map(|_| rng.uniform(0.5, 2.0)).collect(),
    }
}

#[test]
fn every_taylor_form_is_its_kernel() {
    let mut checked = 0;
    for kid in 0..KERNELS.len() {
        if !has_form(kid) {
            continue;
        }
        let kn = &KERNELS[kid];
        for seed in 0..8u32 {
            let mut rng = Rng::new(1000 * kid as u32 + seed + 1);
            // columns well apart and positive where a radius might sit; constants a free twin's
            // (m, c) or a stated number
            let path: Vec<Jet> = (0..kn.n_par)
                .map(|_| {
                    Jet::from(&[
                        rng.uniform(1.0, 10.0) * if rng.uniform(0.0, 1.0) < 0.5 { -1.0 } else { 1.0 },
                        rng.uniform(-1.0, 1.0),
                        rng.uniform(-1.0, 1.0),
                        rng.uniform(-1.0, 1.0),
                    ])
                })
                .collect();
            let k: Vec<f64> = (0..kn.n_const).map(|_| rng.uniform(0.5, 2.0)).collect();
            let mut r = vec![Jet::default(); kn.n_res];
            let mut jrow = Vec::new();
            assert!(residual(kid, &path, &k, &mut r, &mut jrow), "{}", kn.name);
            let v0 = at(&path, 0.0);
            let mut r0 = vec![0.0; kn.n_res];
            (kn.res)(1, &v0, &k, &mut r0);
            let h = 1e-4;
            let (g0, gp, gm) = (rate(kid, &path, &k, 0.0), rate(kid, &path, &k, h), rate(kid, &path, &k, -h));
            for t in 0..kn.n_res {
                let scale = 1.0 + r0[t].abs() + g0[t].abs();
                let close = |got: f64, want: f64, tol: f64, what: &str| {
                    assert!(
                        (got - want).abs() <= tol * scale,
                        "{} row {t} seed {seed}: {what} is {got}, the kernel says {want}",
                        kn.name
                    );
                };
                close(r[t].0[0], r0[t], 1e-12, "the value");
                close(r[t].0[1], g0[t], 1e-10, "the first coefficient");
                close(r[t].0[2], (gp[t] - gm[t]) / (4.0 * h), 1e-6, "the second coefficient");
                close(r[t].0[3], (gp[t] - 2.0 * g0[t] + gm[t]) / (6.0 * h * h), 1e-4, "the third");
            }
        }
        checked += 1;
    }
    assert!(checked >= 75, "only {checked} kernels have forms");
    assert_eq!(ORDER, 5);
}

/// The kernels a 2D trace body is built from all have forms — so a curvature against any trace
/// written with them is stated exactly.  A new 2D kernel joins `taylor::form` or this list says
/// which one did not.
#[test]
fn the_planar_kernels_have_forms() {
    let planar = [
        "coincident", "distance", "midpoint", "horizontal", "vertical", "parallel", "perpendicular",
        "angle", "equal_length", "point_on_line", "point_on_circle", "radius", "equal_radius",
        "tangent_line_circle", "tangent_circle_circle", "tangent_arc_line", "symmetric",
        "parallel_distance", "point_line_distance", "annular_distance", "ordinate_u",
        "ordinate_v", "ordinate_line", "ordinate_line_free",
        "distance_free", "angle_free", "radius_free", "parallel_distance_free",
        "point_line_distance_free", "annular_distance_free", "ordinate_u_free",
        "ordinate_v_free", "point_line_magnitude", "point_line_magnitude_free",
        "parallel_magnitude", "parallel_magnitude_free", "arc_length", "arc_length_free",
    ];
    for name in planar {
        let kid = KERNELS.iter().position(|k| k.name == name).unwrap_or_else(|| panic!("{name}"));
        assert!(has_form(kid), "{name} has no Taylor form");
    }
}

/// Every kernel a body can state has a form, in space as on the page — so the derivative row a
/// tangency states of each (`kernels::dual_kernel`) is exact.  What has none is a soft drag, which
/// no body states, and a free curve's gauge, whose second derivative an energy reads off its
/// spans' lengths instead (`variational.rs`).
#[test]
fn every_kernel_a_body_states_has_a_form() {
    let formless: Vec<&str> =
        (0..KERNELS.len()).filter(|&k| !has_form(k)).map(|k| KERNELS[k].name).collect();
    assert_eq!(
        formless,
        ["drag", "drag_seen"]
    );
}

/// **Every derivative row is exact** (`kernels::dual_kernel`): over every kernel with a form, its
/// columns, tangent columns and a line's ends random, each column moving at random — held, by a
/// component of the line's direction, or by its tangent column — the Jacobian it writes is the
/// derivative of its residual, column by column, against a central difference.
#[test]
fn every_derivative_rows_jacobian_is_its_derivative() {
    use gcs_core::kernels::{dual_kernel, eval_with, TANGENT};
    let mut checked = 0;
    for kid in 0..KERNELS.len() {
        if !has_form(kid) {
            continue;
        }
        let (inner, dual) = (&KERNELS[kid], dual_kernel(kid));
        let m = inner.n_par;
        for seed in 0..4u32 {
            let mut rng = Rng::new(7000 * kid as u32 + seed + 1);
            let mut v: Vec<f64> = (0..dual.n_par)
                .map(|_| {
                    let s = if rng.uniform(0.0, 1.0) < 0.5 { -1.0 } else { 1.0 };
                    s * rng.uniform(1.0, 10.0)
                })
                .collect();
            let mut k = vec![kid as f64];
            k.extend(constants(inner.name, inner.n_const, &mut rng, &mut |t, at| v[t] = at));
            k.extend((0..m).map(|_| [0, 1, 2, 3, TANGENT][rng.uniform(0.0, 4.999) as usize] as f64));
            let (r0, jac) = eval_with(&dual, &v, &k);
            let h = 1e-6;
            for c in 0..dual.n_par {
                let (mut vp, mut vm) = (v.clone(), v.clone());
                vp[c] += h;
                vm[c] -= h;
                let (rp, rm) = (eval_with(&dual, &vp, &k).0, eval_with(&dual, &vm, &k).0);
                for t in 0..dual.n_res {
                    let fd = (rp[t] - rm[t]) / (2.0 * h);
                    let got = jac[t * dual.n_par + c];
                    let scale = 1.0 + r0[t].abs() + fd.abs();
                    assert!(
                        (got - fd).abs() <= 1e-5 * scale,
                        "{} seed {seed} row {t} column {c}: {got} against {fd}",
                        inner.name
                    );
                }
            }
        }
        checked += 1;
    }
    assert!(checked >= 75, "{checked}");
}

/// The arithmetic itself, against closed forms: `sqrt`, `atan2` and a quotient along a line.
#[test]
fn jets_agree_with_closed_forms() {
    // (1 + ε)² under a square root is 1 + ε exactly
    let a = Jet::from(&[1.0, 2.0, 1.0]).sqrt();
    assert!(a.0.iter().zip([1.0, 1.0, 0.0, 0.0]).all(|(x, y)| (x - y).abs() < 1e-15), "{a:?}");
    // atan2(ε, 1) = ε − ε³/3
    let t = Jet::atan2(Jet::var(0.0), Jet::constant(1.0));
    assert!(t.0.iter().zip([0.0, 1.0, 0.0, -1.0 / 3.0]).all(|(x, y)| (x - y).abs() < 1e-15), "{t:?}");
    // 1 / (1 − ε) = 1 + ε + ε² + ε³
    let q = Jet::constant(1.0) / Jet::from(&[1.0, -1.0]);
    assert!(q.0.iter().all(|x| (x - 1.0).abs() < 1e-15), "{q:?}");
    // sin and cos of ε: ε − ε³/6, 1 − ε²/2 + ε⁴/24
    let (sn, cs) = Jet::var(0.0).sin_cos();
    assert!(sn.0.iter().zip([0.0, 1.0, 0.0, -1.0 / 6.0, 0.0]).all(|(x, y)| (x - y).abs() < 1e-15), "{sn:?}");
    assert!(cs.0.iter().zip([1.0, 0.0, -0.5, 0.0, 1.0 / 24.0]).all(|(x, y)| (x - y).abs() < 1e-15), "{cs:?}");
}
