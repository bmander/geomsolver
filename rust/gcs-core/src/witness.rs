//! Stage 4 — the witness configuration method (Michelucci & Foufou 2006).
//!
//! Structural analysis (Stage 2) cannot see dependencies that follow from geometric theorems
//! (three concurrent altitudes, an EqualLength cycle, Pappus).  A witness is a configuration with
//! the sketch's incidence structure but generic dimensions; the Jacobian there tells the truth
//! about the system:
//!
//! * rank deficiency in the rows = dependent constraints (theorem-induced ones included) — pivoted
//!   QR on Jᵀ picks a maximal independent set and, for each leftover equation, the equations it is
//!   implied by;
//! * the null space of J = the infinitesimal motions = exactly which DOFs remain and what they
//!   look like (rigid-body motions separated from internal ones, modes localised).
//!
//! The user's own sketch is often an adequate witness (it satisfies the incidences by
//! construction).  Otherwise we jitter every dimension the constraints declare and re-solve; if
//! that cannot converge we satisfy the incidence-type constraints alone from a perturbed start.
//! The rank test is relative, and pivoted QR is cross-checked against the SVD.

use crate::linalg::{absmax, min_norm_lstsq, norm, orthonormalize, rrqr, svd, Mat};
use crate::model::Sketch;
use crate::newton::Method;
use crate::rng::Rng;
use crate::solve::SolveOpts;
use crate::system::System;
use std::collections::BTreeSet;

#[derive(Clone, Debug)]
pub struct Dependency {
    /// A dependent (redundant) equation's constraint.
    pub constraint: u32,
    /// Constraints whose equations span it.
    pub implied_by: Vec<u32>,
    /// Structural analysis could not see it.
    pub theorem: bool,
}

/// An infinitesimal motion: velocity per free parameter, scaled to unit max displacement.
#[derive(Clone, Debug)]
pub struct Motion {
    pub velocity: Vec<f64>,
    /// A rigid-body motion of the whole sketch.
    pub rigid: bool,
}

#[derive(Clone, Debug)]
pub struct WitnessReport {
    pub x_witness: Vec<f64>,
    /// The sketch itself served as witness.
    pub used_current: bool,
    pub numeric_rank: usize,
    pub dependencies: Vec<Dependency>,
    /// Null-space basis: rigid modes first, then internal DOFs.
    pub motions: Vec<Motion>,
    /// Free-parameter indices taking part in some motion.
    pub movable: Vec<usize>,
    /// The sketch Param index behind each column of a motion's velocity.
    pub params: Vec<u32>,
    pub warnings: Vec<String>,
}

impl WitnessReport {
    pub fn n_dof(&self) -> usize {
        self.motions.len()
    }
    pub fn n_internal_dof(&self) -> usize {
        self.motions.iter().filter(|m| !m.rigid).count()
    }

    pub fn summary(&self) -> String {
        let mut parts = vec![
            format!("witness rank {}", self.numeric_rank),
            format!(
                "{} DOF ({} internal, {} rigid-body)",
                self.n_dof(),
                self.n_internal_dof(),
                self.n_dof() - self.n_internal_dof()
            ),
        ];
        if !self.dependencies.is_empty() {
            let th = self.dependencies.iter().filter(|d| d.theorem).count();
            parts.push(format!(
                "{} dependent constraint(s){}",
                self.dependencies.len(),
                if th > 0 { format!(", {th} theorem-type") } else { String::new() }
            ));
        }
        parts.extend(self.warnings.iter().cloned());
        parts.join("; ")
    }

    /// The Params a motion actually moves.
    pub fn moving_params(&self, m: &Motion, rel: f64) -> Vec<u32> {
        let mx = {
            let a = absmax(&m.velocity);
            if a == 0.0 {
                1.0
            } else {
                a
            }
        };
        self.params
            .iter()
            .enumerate()
            .filter(|(i, _)| m.velocity[*i].abs() > rel * mx)
            .map(|(_, &p)| p)
            .collect()
    }
}

/// A configuration with the sketch's incidence structure and generic dimensions.  Leaves the
/// sketch's values and dimensions untouched.
pub fn make_witness(sk: &mut Sketch, seed: u32, jitter: f64, tol: f64) -> Vec<f64> {
    let x0 = sk.get_x();
    let saved_constraints = sk.constraints.clone();
    let mut rng = Rng::new(seed);

    // 1. generic dimensions (lengths scaled, angles offset), re-solved from current geometry
    let mut edits: Vec<(usize, usize, f64)> = Vec::new(); // (constraint index, arg, new value)
    for (ci, c) in sk.constraints.iter().enumerate() {
        // a dimension written in terms of a free variable states no number, so there is no
        // number to make generic: it is structure, and it stays exactly as it is
        if c.soft || c.free.is_some() {
            continue;
        }
        for (ai, _, kind) in c.dimensions() {
            let v = c.args[ai].num();
            let nv = if kind == crate::constraints::SpecKind::Length {
                v * (1.0 + jitter * rng.normal(0.0, 1.0))
            } else {
                v + jitter * rng.normal(0.0, 1.0)
            };
            edits.push((ci, ai, nv));
        }
    }
    for &(ci, ai, nv) in &edits {
        sk.constraints[ci].args[ai] = crate::constraints::Arg::Num(nv);
    }
    sk.constraints.retain(|c| !c.soft);
    let mut sys = System::new(sk);
    let res = sys.solve(sk, SolveOpts { max_iter: 60, ..SolveOpts::default() });
    let z = sys.z0(sk);
    if res.success && sys.max_relative_residual(&z) <= tol {
        let xw = sk.get_x();
        sk.constraints = saved_constraints;
        sk.set_x(&x0);
        return xw;
    }
    // 2. incidences only (always satisfiable) from a perturbed start
    sk.set_x(&x0);
    // the free ones stay for the same reason, and because dropping them would leave the
    // unknowns they name in the parameter vector with no equation mentioning them
    sk.constraints.retain(|c| c.dimensions().is_empty() || c.free.is_some());
    let sigma = 0.02 * sk.extent().max(1.0);
    sk.perturb(sigma, seed);
    let mut sys2 = System::new(sk);
    sys2.solve(sk, SolveOpts { max_iter: 60, ..SolveOpts::default() });
    let xw = sk.get_x();
    sk.constraints = saved_constraints;
    sk.set_x(&x0);
    xw
}

/// Rows of the null-space basis that are nonzero: the parameters taking part in some infinitesimal
/// motion of the configuration.
pub fn movable_columns(n: &Mat, rtol: f64) -> Vec<usize> {
    if n.rows == 0 || n.cols == 0 {
        return Vec::new();
    }
    let mut w = vec![0.0; n.rows];
    let mut wmax = 0.0f64;
    for i in 0..n.rows {
        w[i] = absmax(n.row(i));
        wmax = wmax.max(w[i]);
    }
    (0..n.rows).filter(|&i| w[i] > rtol * wmax).collect()
}

/// Rank, dependencies and motions of the sketch's constraint system at a witness.
///
/// `over_ids` are the constraints the structural analysis already put in its over-determined
/// block; a dependency outside that set is theorem-type — invisible to the graph.
pub fn analyze_with(
    sk: &mut Sketch,
    sys: &mut System,
    x_witness: Option<Vec<f64>>,
    over_ids: &BTreeSet<u32>,
    rtol: f64,
    seed: u32,
) -> WitnessReport {
    let x0 = sk.get_x();
    let used_current = if x_witness.is_none() {
        let z = sys.z0(sk);
        sys.max_relative_residual(&z) <= 1e-9
    } else {
        false
    };
    let xw = match x_witness {
        Some(x) => x,
        None => {
            if used_current {
                x0.clone()
            } else {
                make_witness(sk, seed, 0.05, 1e-8)
            }
        }
    };
    sk.set_x(&xw);
    let free_params: Vec<u32> = sys.free.iter().map(|&i| i as u32).collect();
    let z = sys.z0(sk);
    let dense = sys.jacobian_dense(&z);
    let hard_rows = sys.hard_rows();
    let j = dense.select_rows(&hard_rows);
    let (_, row_c) = sys.structure();
    let (m, n) = (j.rows, j.cols);
    let mut warnings: Vec<String> = Vec::new();
    if m == 0 || n == 0 {
        let identity = Mat::identity(n);
        let motions = classify_motions(&identity, &free_params, sk);
        sk.set_x(&x0);
        return WitnessReport {
            x_witness: xw,
            used_current,
            numeric_rank: 0,
            dependencies: Vec::new(),
            motions,
            movable: (0..n).collect(),
            params: free_params,
            warnings,
        };
    }
    // rank: RRQR on Jᵀ (pivots = a maximal independent row set), cross-checked with the SVD that
    // also yields the null space
    let (rank_qr, piv) = rrqr(&j.transpose(), rtol);
    let d = svd(&j, false);
    let mn = m.min(n);
    let mut rank_svd = 0;
    if mn > 0 && d.s[0] > 0.0 {
        for i in 0..mn {
            if d.s[i] > rtol * d.s[0] {
                rank_svd += 1;
            }
        }
    }
    if !d.converged {
        warnings.push(
            "the SVD did not converge: the null space and the rank below are the QR's alone"
                .to_string(),
        );
        rank_svd = rank_qr; // do not let a failed factorisation drag the rank down to zero
    }
    let mut rank = rank_qr;
    if rank_qr != rank_svd {
        warnings.push(format!(
            "rank ambiguous: QR {rank_qr} vs SVD {rank_svd} (near-degenerate witness)"
        ));
        rank = rank_qr.min(rank_svd);
    }
    // dependent rows: the non-pivot rows, each expressed in the pivot rows' span (one
    // factorisation for all of them)
    let indep: Vec<usize> = piv[..rank].iter().map(|&x| x as usize).collect();
    let dep_rows: Vec<usize> =
        piv[rank..].iter().map(|&x| x as usize).filter(|&r| r < row_c.len()).collect();
    let mut deps: Vec<Dependency> = Vec::new();
    if !dep_rows.is_empty() && !indep.is_empty() {
        let a = j.select_rows(&indep).transpose(); // n x rank
        let b = j.select_rows(&dep_rows).transpose(); // n x |dep|
        let (coefs, _) = min_norm_lstsq(&a, &b, 1e-12); // rank x |dep|
        for (col, &r) in dep_rows.iter().enumerate() {
            let c = row_c[r];
            if deps.iter().any(|d| d.constraint == c) {
                continue;
            }
            let coef: Vec<f64> = (0..rank).map(|k| coefs.data[k * coefs.cols + col]).collect();
            let lim = 1e-6 * { let a = absmax(&coef); if a == 0.0 { 1.0 } else { a } };
            let mut order: Vec<(f64, usize)> =
                coef.iter().enumerate().map(|(k, v)| (v.abs(), k)).collect();
            order.sort_by(|x, y| y.0.partial_cmp(&x.0).unwrap());
            let mut implied: Vec<u32> = Vec::new();
            for (a, k) in order {
                if a <= lim {
                    continue;
                }
                let s2 = row_c[indep[k]];
                if s2 != c && !implied.contains(&s2) {
                    implied.push(s2);
                }
            }
            deps.push(Dependency { constraint: c, implied_by: implied, theorem: !over_ids.contains(&c) });
        }
    }
    let nn = n - rank;
    let mut null = Mat::zeros(n, nn);
    for i in 0..n {
        for jj in 0..nn {
            null.data[i * nn.max(1) + jj] = d.vt.data[(rank + jj) * n + i];
        }
    }
    let motions = classify_motions(&null, &free_params, sk);
    let movable = movable_columns(&null, 1e-8);
    sk.set_x(&x0);
    WitnessReport {
        x_witness: xw,
        used_current,
        numeric_rank: rank,
        dependencies: deps,
        motions,
        movable,
        params: free_params,
        warnings,
    }
}

pub fn analyze(sk: &mut Sketch, x_witness: Option<Vec<f64>>, seed: u32) -> WitnessReport {
    let mut sys = System::new(sk);
    // the structural over-block, so a dependency the graph *can* see is not reported as
    // theorem-type.  `diagnose_with` passes the set it already has; standing alone we compute it,
    // rather than passing an empty set and labelling every dependency invisible to the graph.
    let over = structural_over(&mut sys);
    analyze_with(sk, &mut sys, x_witness, &over, 1e-9, seed)
}

/// Constraints in the Dulmage–Mendelsohn over-determined block — the redundancy structural
/// analysis can see for itself.
pub fn structural_over(sys: &mut System) -> BTreeSet<u32> {
    let (adj, row_c) = sys.structure();
    let dm = crate::graph::dulmage_mendelsohn(&adj, sys.n_free);
    dm.over_rows.iter().map(|&r| row_c[r]).collect()
}

/// Split the null space into rigid-body modes (translations/rotation of everything that can move
/// together) and internal DOFs; localise the internal ones (sparse basis).
fn classify_motions(null: &Mat, params: &[u32], sk: &Sketch) -> Vec<Motion> {
    let (n, d) = (null.rows, null.cols);
    if d == 0 {
        return Vec::new();
    }
    // rigid-body generators, from the model's own parameter identity (not from names)
    // axis[param] = (which coordinate, x, y)
    let mut axis: Vec<Option<(u8, f64, f64)>> = vec![None; sk.params.len()];
    for i in 0..sk.points.len() {
        let (x, y) = sk.point_xy(i);
        let p = &sk.points[i];
        axis[p.x as usize] = Some((0, x, y));
        axis[p.y as usize] = Some((1, x, y));
    }
    let (mut cx, mut cy) = (0.0, 0.0);
    if !sk.points.is_empty() {
        for i in 0..sk.points.len() {
            let (x, y) = sk.point_xy(i);
            cx += x;
            cy += y;
        }
        cx /= sk.points.len() as f64;
        cy /= sk.points.len() as f64;
    }
    let mut tx = vec![0.0; n];
    let mut ty = vec![0.0; n];
    let mut rot = vec![0.0; n];
    for (i, &p) in params.iter().enumerate() {
        // a radius: invariant under rigid motions
        let Some((which, x, y)) = axis[p as usize] else { continue };
        if which == 0 {
            tx[i] = 1.0;
            rot[i] = -(y - cy);
        } else {
            ty[i] = 1.0;
            rot[i] = x - cx;
        }
    }
    // N has orthonormal columns, so a vector lies in its span iff the projection keeps its norm
    let in_null = |v: &[f64]| -> bool {
        let mut s = 0.0;
        for jj in 0..d {
            let mut acc = 0.0;
            for i in 0..n {
                acc += null.data[i * d + jj] * v[i];
            }
            s += acc * acc;
        }
        s.sqrt() >= (1.0 - 1e-6) * norm(v)
    };
    let scaled = |v: &[f64]| -> Vec<f64> {
        let mx = {
            let a = absmax(v);
            if a == 0.0 {
                1.0
            } else {
                a
            }
        };
        v.iter().map(|x| x / mx).collect()
    };
    let mut rigid: Vec<Vec<f64>> = Vec::new();
    for v in [&tx, &ty, &rot] {
        if v.iter().any(|&a| a != 0.0) && in_null(v) {
            rigid.push(scaled(v));
        }
    }
    let mut motions: Vec<Motion> =
        rigid.iter().map(|v| Motion { velocity: v.clone(), rigid: true }).collect();

    let ni: Mat = if !rigid.is_empty() {
        // internal DOFs = the null space minus the rigid span
        let q = orthonormalize(&rigid, 1e-12);
        let mut m = null.clone();
        for qv in &q {
            let mut proj = vec![0.0; d];
            for jj in 0..d {
                let mut acc = 0.0;
                for i in 0..n {
                    acc += qv[i] * m.data[i * d + jj];
                }
                proj[jj] = acc;
            }
            for i in 0..n {
                for jj in 0..d {
                    m.data[i * d + jj] -= qv[i] * proj[jj];
                }
            }
        }
        let sv = svd(&m, true);
        // N orthonormal: an absolute threshold is right here
        let keep: Vec<usize> = (0..sv.s.len()).filter(|&jj| sv.s[jj] > 1e-6).collect();
        let mut out = Mat::zeros(n, keep.len());
        for i in 0..n {
            for (c, &jj) in keep.iter().enumerate() {
                out.data[i * keep.len().max(1) + c] = sv.u.data[i * sv.u.cols + jj];
            }
        }
        out
    } else {
        null.clone()
    };

    if ni.cols > 0 {
        // localise: rotate the basis so each mode is 1 at a pivot parameter and 0 at the others
        let k = ni.cols;
        let (_, piv) = rrqr(&ni.transpose(), 1e-10);
        let rows: Vec<usize> = piv[..k].iter().map(|&x| x as usize).collect();
        let a = ni.select_rows(&rows).transpose(); // k x k
        let b = ni.transpose(); // k x n
        let (sol, _) = min_norm_lstsq(&a, &b, 1e-12); // k x n
        for c in 0..k {
            let v: Vec<f64> = (0..n).map(|i| sol.data[c * sol.cols + i]).collect();
            motions.push(Motion { velocity: scaled(&v), rigid: false });
        }
    }
    motions
}

/// The solver method witness construction uses (the default everywhere).
pub const WITNESS_METHOD: Method = Method::DogLeg;
