//! The core's dense linear algebra checked against an independent implementation.
//!
//! There is no LAPACK/BLAS anywhere in the project: the pivoted QR, the complete orthogonal
//! decomposition behind the minimum-norm step, the SVD and the LU are ours.  `nalgebra` is the
//! reference that keeps them honest — the one place two implementations are still compared, on
//! purpose.  It is a **dev-dependency**, so nothing it brings reaches the cdylib or the wasm;
//! the library's own promise of no dependencies is unchanged.
//!
//! Each test also states the property the reference cannot: `A ≈ QR` on the pivots, `NᵀN ≈ I`,
//! a minimum-norm solution orthogonal to the null space.  A reference agreeing is evidence; a
//! property holding is the contract.

use gcs_core::examples;
use gcs_core::linalg::{
    lu_solve, min_norm_lstsq, min_norm_solve, rank_and_nullspace, rank_rrqr, rrqr, svd, Mat,
};
use gcs_core::rng::Rng;
use gcs_core::system::{System, RANK_TOL};

use nalgebra::DMatrix;

const RCOND: f64 = 1e-12;

/// A random matrix, and the same numbers as an `nalgebra` matrix.  One draw feeds both, so the
/// two implementations are never compared on different inputs.
fn pair(seed: u32, m: usize, n: usize) -> (Mat, DMatrix<f64>) {
    let mut rng = Rng::new(seed);
    let data: Vec<f64> = (0..m * n).map(|_| rng.normal(0.0, 1.0)).collect();
    // ours is row-major, nalgebra's `from_row_slice` reads the same order
    (Mat::from_vec(m, n, data.clone()), DMatrix::from_row_slice(m, n, &data))
}

fn dm(a: &Mat) -> DMatrix<f64> {
    DMatrix::from_row_slice(a.rows, a.cols, &a.data)
}

fn close(a: f64, b: f64, atol: f64) -> bool {
    (a - b).abs() <= atol
}

fn assert_vec_close(got: &[f64], want: &[f64], atol: f64, what: &str) {
    assert_eq!(got.len(), want.len(), "{what}: length");
    for (i, (&g, &w)) in got.iter().zip(want).enumerate() {
        assert!(close(g, w, atol), "{what}[{i}]: {g} vs {w}");
    }
}

/// nalgebra's SVD-based least squares: the pseudo-inverse solution, which *is* the minimum-norm
/// one — the solution a rank-revealing least squares is expected to pick.
fn ref_min_norm(a: &DMatrix<f64>, b: &[f64]) -> (Vec<f64>, usize) {
    let svd = a.clone().svd(true, true);
    let smax = svd.singular_values.iter().cloned().fold(0.0f64, f64::max);
    let cut = RCOND * smax;
    let rank = svd.singular_values.iter().filter(|&&s| s > cut).count();
    let u = svd.u.as_ref().expect("u");
    let vt = svd.v_t.as_ref().expect("v_t");
    let bv = DMatrix::from_column_slice(b.len(), 1, b);
    let mut x = vec![0.0; a.ncols()];
    for i in 0..rank {
        let s = svd.singular_values[i];
        let c = (u.column(i).transpose() * &bv)[(0, 0)] / s;
        for j in 0..a.ncols() {
            x[j] += c * vt[(i, j)];
        }
    }
    (x, rank)
}

#[test]
fn min_norm_lstsq_matches_the_reference() {
    for (m, n) in [(6usize, 4usize), (4, 6), (5, 5), (12, 3), (3, 12)] {
        let (a, ar) = pair((m * 31 + n) as u32, m, n);
        let mut rng = Rng::new(9_000 + (m * 31 + n) as u32);
        let b: Vec<f64> = (0..m).map(|_| rng.normal(0.0, 1.0)).collect();
        let (x, rank) = min_norm_solve(&a, &b, RCOND);
        let (xr, rank_r) = ref_min_norm(&ar, &b);
        assert_eq!(rank, rank_r, "rank for {m}x{n}");
        assert_vec_close(&x, &xr, 1e-9, &format!("min_norm_solve {m}x{n}"));
    }
}

#[test]
fn min_norm_lstsq_is_minimum_norm_when_rank_deficient() {
    let (mut a, _) = pair(7, 6, 3);
    // a duplicated column: rank 3 of 4
    let mut wide = Mat::zeros(6, 4);
    for i in 0..6 {
        for j in 0..3 {
            wide.set(i, j, a.at(i, j));
        }
        wide.set(i, 3, a.at(i, 0));
    }
    a = wide;
    let mut rng = Rng::new(77);
    let b: Vec<f64> = (0..6).map(|_| rng.normal(0.0, 1.0)).collect();
    let (x, rank) = min_norm_solve(&a, &b, RCOND);
    assert_eq!(rank, 3);
    let (xr, _) = ref_min_norm(&dm(&a), &b);
    assert_vec_close(&x, &xr, 1e-8, "pseudo-inverse solution");

    // and the property the reference cannot state: no component in the null space
    let rn = rank_and_nullspace(&a, RCOND);
    let null = rn.null();
    for c in 0..null.cols {
        let d: f64 = (0..null.rows).map(|i| x[i] * null.at(i, c)).sum();
        assert!(d.abs() < 1e-9, "solution has a null-space component: {d}");
    }
}

#[test]
fn min_norm_lstsq_takes_several_right_hand_sides() {
    let (a, _) = pair(11, 7, 5);
    let (b, _) = pair(12, 7, 3);
    let (x, rank) = min_norm_lstsq(&a, &b, RCOND);
    assert_eq!(rank, 5);
    assert_eq!((x.rows, x.cols), (5, 3));
    for c in 0..3 {
        let (one, _) = min_norm_solve(&a, &b.col(c), RCOND);
        assert_vec_close(&x.col(c), &one, 1e-9, &format!("column {c}"));
    }
}

#[test]
fn rrqr_rank_matches_the_reference() {
    for seed in 0..4u32 {
        let (a, ar) = pair(seed + 1, 8, 5);
        assert_eq!(rank_rrqr(&a, RCOND), ar.rank(1e-10), "full rank, seed {seed}");

        // one dependent column
        let mut b = a.clone();
        for i in 0..8 {
            b.set(i, 3, a.at(i, 0) + 2.0 * a.at(i, 1));
        }
        assert_eq!(rank_rrqr(&b, RCOND), 4);
        let (rank, piv) = rrqr(&b, RCOND);
        assert_eq!(rank, 4);
        let mut sorted: Vec<i32> = piv.clone();
        sorted.sort_unstable();
        assert_eq!(sorted, (0..5).collect::<Vec<i32>>(), "pivots are a permutation");
        // the first `rank` pivots index a maximal independent set of columns
        let keep: Vec<usize> = piv[..rank].iter().map(|&p| p as usize).collect();
        assert_eq!(dm(&b.select_cols(&keep)).rank(1e-10), rank);
    }
}

#[test]
fn svd_singular_values_match_the_reference() {
    for (m, n) in [(6usize, 4usize), (4, 6), (5, 5)] {
        let (a, ar) = pair((m * 13 + n) as u32, m, n);
        let d = svd(&a, true);
        assert!(d.converged);
        let mut want: Vec<f64> = ar.clone().svd(false, false).singular_values.iter().cloned().collect();
        want.sort_by(|x, y| y.partial_cmp(x).unwrap());
        want.truncate(m.min(n));
        assert_vec_close(&d.s, &want, 1e-10, &format!("singular values {m}x{n}"));

        // and the factorization reconstructs A: U diag(s) Vt[:mn]
        let mn = m.min(n);
        for i in 0..m {
            for j in 0..n {
                let v: f64 = (0..mn).map(|k| d.u.at(i, k) * d.s[k] * d.vt.at(k, j)).sum();
                assert!(close(v, a.at(i, j), 1e-9), "reconstruction at ({i},{j})");
            }
        }
    }
}

#[test]
fn rank_and_nullspace_spans_the_kernel() {
    let (a, _) = pair(3, 4, 7);
    let rn = rank_and_nullspace(&a, RCOND);
    assert_eq!(rn.rank, 4);
    let n = rn.null();
    assert_eq!((n.rows, n.cols), (7, 3));
    // A N = 0
    for c in 0..n.cols {
        for r in 0..a.rows {
            let v: f64 = (0..a.cols).map(|k| a.at(r, k) * n.at(k, c)).sum();
            assert!(v.abs() < 1e-9, "A N is not zero at ({r},{c}): {v}");
        }
    }
    // orthonormal columns
    for i in 0..n.cols {
        for j in 0..n.cols {
            let v: f64 = (0..n.rows).map(|k| n.at(k, i) * n.at(k, j)).sum();
            assert!(close(v, if i == j { 1.0 } else { 0.0 }, 1e-9), "NᵀN at ({i},{j}): {v}");
        }
    }
}

#[test]
fn lu_solve_matches_the_reference() {
    let (a, ar) = pair(5, 6, 6);
    let mut rng = Rng::new(555);
    let b: Vec<f64> = (0..6).map(|_| rng.normal(0.0, 1.0)).collect();
    let mut aw = a.data.clone();
    let mut bw = b.clone();
    assert!(lu_solve(6, &mut aw, &mut bw));
    let want = ar
        .lu()
        .solve(&DMatrix::from_column_slice(6, 1, &b))
        .expect("reference LU solve");
    assert_vec_close(&bw, want.as_slice(), 1e-9, "lu_solve");
}

#[test]
fn sparse_and_dense_jacobians_agree() {
    // The CSR structure is fixed at compile time; only the values are refilled.
    let mut sk = examples::truss(6, 100.0, 30.0, true);
    examples::jitter(&mut sk, 1.0, 1);
    let mut s = System::new(&sk);
    let z = s.z0(&sk);
    let dense = s.jacobian_dense(&z);
    let data = s.compute_csr(&z).to_vec();
    let (indptr, indices) = (s.csr_indptr.clone(), s.csr_indices.clone());
    let mut from_csr = Mat::zeros(s.n_res, s.n_free);
    for r in 0..s.n_res {
        for p in indptr[r]..indptr[r + 1] {
            from_csr.set(r, indices[p as usize] as usize, data[p as usize]);
        }
    }
    assert_eq!(from_csr, dense);
}

#[test]
fn jacobian_rank_agrees_with_the_reference_on_a_real_sketch() {
    // the reference judges the same matrix by the same rule: the conditioned Jacobian, an
    // absolute tolerance
    let mut sk = examples::rect_fillets(80.0, 50.0, 10.0, 0.0);
    examples::jitter(&mut sk, 1.0, 1);
    let mut s = System::new(&sk);
    let z = s.z0(&sk);
    let c = s.conditioned(&z);
    assert_eq!(c.rows(), s.hard_rows().len());
    assert_eq!(c.cols(), s.n_free);
    let ours = s.rank(&z, RANK_TOL, true);
    assert_eq!(ours, dm(c.as_mat()).rank(RANK_TOL));
}

#[test]
fn a_non_convergent_svd_says_so_instead_of_reporting_rank_zero() {
    // `gr_svd` gives up after a fixed number of QR sweeps; discarding that return read a
    // non-finite Jacobian as rank 0 with a full null space.
    let bad = Mat::from_vec(2, 2, vec![1.0, f64::NAN, 0.0, 1.0]);
    assert!(!rank_and_nullspace(&bad, RCOND).converged);
    assert!(!svd(&bad, false).converged);
    let ok = rank_and_nullspace(&Mat::identity(2), RCOND);
    assert!(ok.converged);
    assert_eq!(ok.rank, 2);
}
