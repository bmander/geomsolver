//! What the curve tests share: a document built or the test fails saying why, the involute's
//! closed form, and the finite-difference check of a compiled system's Jacobian.
#![allow(dead_code)]

use gcs_core::model::Sketch;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::parse;
use gcs_core::system::System;

pub fn build(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| (d.code.as_str(), &d.message)).collect::<Vec<_>>());
    e
}

/// Where the involute of the circle at `(cx, cy)` of base radius `rb` is at roll `u_deg`,
/// worked out here rather than asked of the core — so a test and the thing it tests do not
/// share an implementation.
pub fn involute_at(cx: f64, cy: f64, rb: f64, u_deg: f64) -> (f64, f64) {
    let r = u_deg.to_radians();
    (cx + rb * (r.cos() + r * r.sin()), cy + rb * (r.sin() - r * r.cos()))
}

/// **The Jacobian the kernels write is the system's own derivative**: every column of the
/// assembled Jacobian against a central difference of the assembled residuals, so a tape's
/// gradient, a kernel's column order and `params_on`'s are all checked at once.
pub fn fd_jacobian(sk: &Sketch, tol: f64) {
    let mut sys = System::new(sk);
    let z = sys.z0(sk);
    let dense = sys.jacobian_dense(&z);
    for j in 0..z.len() {
        let h = 1e-6 * z[j].abs().max(1.0);
        let (mut lo, mut hi) = (z.clone(), z.clone());
        lo[j] -= h;
        hi[j] += h;
        let (a, b) = (sys.residuals(&lo), sys.residuals(&hi));
        for i in 0..sys.n_res {
            let fd = (b[i] - a[i]) / (2.0 * h);
            let got = dense.at(i, j);
            assert!(
                (got - fd).abs() <= tol * fd.abs().max(1.0),
                "d r{i} / d z{j}: kernel {got}, finite difference {fd}",
            );
        }
    }
}
