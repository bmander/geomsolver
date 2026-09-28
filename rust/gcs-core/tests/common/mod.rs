//! What the curve tests share: a document built or the test fails saying why, the involute's
//! closed form, and the finite-difference check of a compiled system's Jacobian — what the
//! spatial tests share: a document read or refused, and the arithmetic of lines in space — and
//! what the block-solve tests share: a solve under one `BlockMode`, and a pose to the bit.
#![allow(dead_code)]

use gcs_core::model::{EntRef, Sketch};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{BlockMode, SolveOpts};
use gcs_core::space::{cross, norm, sub};
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

/* -- the spatial tests' --------------------------------------------------------------------- */

/// `src` parsed and elaborated with no error, or the test fails with every diagnostic.
pub fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let e = elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}\n{src}",
        e.errors().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect::<Vec<_>>()
    );
    e
}

/// `src` is refused with `code` and a message holding `needle`, shown at `at`.
pub fn refused(src: &str, code: &str, needle: &str, at: &str) {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    let saw: Vec<String> =
        e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect();
    let d = e.diags.iter().find(|d| d.code.as_str() == code && d.message.contains(needle))
        .unwrap_or_else(|| panic!("expected {code} `{needle}`\n{src}\n{saw:#?}"));
    assert_eq!(d.span.slice(prog.text()), at, "{saw:?}");
}

/// The entity the document calls `n`.
pub fn ent(e: &Elaborated, n: &str) -> EntRef {
    e.map.ent_named(n).unwrap_or_else(|| panic!("no `{n}`"))
}

/// A line's two ends where they stand in space.
pub fn ends(sk: &Sketch, l: EntRef) -> ([f64; 3], [f64; 3]) {
    let l = &sk.lines[l.i()];
    (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize))
}

/// The unit vector along `a`.
pub fn unit(a: [f64; 3]) -> [f64; 3] {
    let l = norm(a);
    [a[0] / l, a[1] / l, a[2] / l]
}

/// `p`'s distance from the line through `a` along `d`.
pub fn off_line(p: [f64; 3], a: [f64; 3], d: [f64; 3]) -> f64 {
    norm(cross(sub(p, a), d)) / norm(d)
}

/* -- the block-solve tests' ----------------------------------------------------------------- */

/// The default solve, as solventc runs it, with the block pass under `blocks`.
pub fn with_blocks(blocks: BlockMode) -> SolveOpts {
    SolveOpts { blocks, ..SolveOpts::default() }
}

/// Every parameter's value, bit for bit: what "the same pose" means when a path must not move it.
pub fn bits(sk: &Sketch) -> Vec<u64> {
    sk.get_x().into_iter().map(f64::to_bits).collect()
}
