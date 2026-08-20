//! Finite-difference verification of analytic Jacobians.
//!
//! A first-class module because it stays useful forever: every new constraint type is checked
//! against it, and the bindings expose it so the same check runs from Python and from the browser.

use crate::constraints::Constraint;
use crate::linalg::Mat;
use crate::model::Sketch;
use crate::system::System;

/// Central differences, one column per input.
pub fn fd_jacobian<F: FnMut(&[f64]) -> Vec<f64>>(mut f: F, v: &[f64], h: f64) -> Mat {
    let f0 = f(v);
    let mut j = Mat::zeros(f0.len(), v.len());
    let mut w = v.to_vec();
    for c in 0..v.len() {
        w[c] = v[c] + h;
        let fp = f(&w);
        w[c] = v[c] - h;
        let fm = f(&w);
        w[c] = v[c];
        for r in 0..f0.len() {
            j.data[r * v.len() + c] = (fp[r] - fm[r]) / (2.0 * h);
        }
    }
    j
}

fn max_err(ja: &Mat, jn: &Mat, rtol: f64, atol: f64, label: &str) -> Result<f64, String> {
    let err = ja
        .data
        .iter()
        .zip(&jn.data)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f64, f64::max);
    let scale = crate::linalg::absmax(&jn.data);
    if err > atol + rtol * scale {
        return Err(format!("{label}: Jacobian mismatch, max err {err:.3e}"));
    }
    Ok(err)
}

/// Max abs error between a constraint's analytic and FD Jacobian; `Err` if too large.
pub fn check_constraint(
    sk: &Sketch,
    c: &Constraint,
    rtol: f64,
    atol: f64,
) -> Result<f64, String> {
    let v = c.local_values(sk);
    let n_par = v.len();
    let ja = Mat::from_vec(c.n_residuals(), n_par, c.jacobian(&v));
    let jn = fd_jacobian(|w| c.residual(w), &v, 1e-6);
    max_err(&ja, &jn, rtol, atol, c.type_name())
}

/// The assembled Jacobian of a whole sketch against finite differences.
pub fn check_sketch(sk: &Sketch, rtol: f64, atol: f64) -> Result<f64, String> {
    let mut sys = System::new(sk);
    let z = sys.z0(sk);
    let ja = sys.jacobian_dense(&z);
    let jn = fd_jacobian(|w| sys.residuals(w), &z, 1e-6);
    max_err(&ja, &jn, rtol, atol, "sketch")
}
