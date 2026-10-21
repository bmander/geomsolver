//! The core's dense linear algebra checked against an independent implementation.
//!
//! There is no LAPACK/BLAS anywhere in the project: the pivoted QR, the complete orthogonal
//! decomposition behind the minimum-norm step, the SVD and the LU are ours.  `nalgebra` is the
//! reference that keeps them honest — the one place two implementations are still compared, on
//! purpose.  It is a **dev-dependency**, so nothing it brings reaches the cdylib or the wasm;
//! the library's own promise of no dependencies is unchanged.
//!
//! Each test also states the property the reference cannot: `A ≈ QR` on the pivots,
//! `NᵀN ≈ I`, a minimum-norm solution orthogonal to the null space.  A reference agreeing is
//! evidence; a property holding is the contract.

use gcs_core::cholesky::{Cholesky, Ordering, Symmetric};
use gcs_core::examples;
use gcs_core::linalg::{
    dot, lu_solve, min_norm_lstsq, min_norm_solve, rank_and_nullspace, rank_rrqr, rrqr, svd, Mat,
};
use gcs_core::rng::Rng;
use gcs_core::system::{System, RANK_TOL};

use nalgebra::DMatrix;

const RCOND: f64 = 1e-12;

/// A random matrix.  `dm` turns it into the reference's, so one draw feeds both and the two
/// implementations are never compared on different inputs.
fn rand(seed: u32, m: usize, n: usize) -> Mat {
    let mut rng = Rng::new(seed);
    Mat::from_vec(m, n, (0..m * n).map(|_| rng.normal(0.0, 1.0)).collect())
}

/// The same numbers as an `nalgebra` matrix — ours is row-major, and `from_row_slice` reads
/// the same order.
fn dm(a: &Mat) -> DMatrix<f64> {
    DMatrix::from_row_slice(a.rows, a.cols, &a.data)
}

fn rand_vec(seed: u32, n: usize) -> Vec<f64> {
    let mut rng = Rng::new(seed);
    (0..n).map(|_| rng.normal(0.0, 1.0)).collect()
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
    let bv = DMatrix::from_column_slice(b.len(), 1, b);
    let x = svd.solve(&bv, cut).expect("reference least squares");
    (x.as_slice().to_vec(), svd.rank(cut))
}

#[test]
fn min_norm_solve_matches_the_reference() {
    for (m, n) in [(6usize, 4usize), (4, 6), (5, 5), (12, 3), (3, 12)] {
        let a = rand((m * 31 + n) as u32, m, n);
        let b = rand_vec(9_000 + (m * 31 + n) as u32, m);
        let (x, rank) = min_norm_solve(&a, &b, RCOND);
        let (xr, rank_r) = ref_min_norm(&dm(&a), &b);
        assert_eq!(rank, rank_r, "rank for {m}x{n}");
        assert_vec_close(&x, &xr, 1e-9, &format!("min_norm_solve {m}x{n}"));
    }
}

#[test]
fn min_norm_solve_is_minimum_norm_when_rank_deficient() {
    // column 0 repeated: rank 3 of 4
    let a = rand(7, 6, 3).select_cols(&[0, 1, 2, 0]);
    let b = rand_vec(77, 6);
    let (x, rank) = min_norm_solve(&a, &b, RCOND);
    assert_eq!(rank, 3);
    let (xr, _) = ref_min_norm(&dm(&a), &b);
    assert_vec_close(&x, &xr, 1e-8, "pseudo-inverse solution");

    // and the property the reference cannot state: no component in the null space
    let null = rank_and_nullspace(&a, RCOND).null();
    for c in 0..null.cols {
        let d = dot(&x, &null.col(c));
        assert!(d.abs() < 1e-9, "solution has a null-space component: {d}");
    }
}

#[test]
fn min_norm_lstsq_takes_several_right_hand_sides() {
    let a = rand(11, 7, 5);
    let b = rand(12, 7, 3);
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
        let a = rand(seed + 1, 8, 5);
        assert_eq!(rank_rrqr(&a, RCOND), dm(&a).rank(1e-10), "full rank, seed {seed}");

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
        let a = rand((m * 13 + n) as u32, m, n);
        let d = svd(&a, true);
        assert!(d.converged);
        // nalgebra's are min(m, n) long but in no promised order
        let mut want: Vec<f64> =
            dm(&a).svd(false, false).singular_values.iter().cloned().collect();
        want.sort_by(|x, y| y.partial_cmp(x).unwrap());
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
    let a = rand(3, 4, 7);
    let rn = rank_and_nullspace(&a, RCOND);
    assert_eq!(rn.rank, 4);
    let n = rn.null();
    assert_eq!((n.rows, n.cols), (7, 3));
    // A N = 0
    for c in 0..n.cols {
        for (r, v) in a.mul_vec(&n.col(c)).iter().enumerate() {
            assert!(v.abs() < 1e-9, "A N is not zero at ({r},{c}): {v}");
        }
    }
    // orthonormal columns
    for i in 0..n.cols {
        for j in 0..n.cols {
            let v = dot(&n.col(i), &n.col(j));
            assert!(close(v, if i == j { 1.0 } else { 0.0 }, 1e-9), "NᵀN at ({i},{j}): {v}");
        }
    }
}

#[test]
fn lu_solve_matches_the_reference() {
    let a = rand(5, 6, 6);
    let b = rand_vec(555, 6);
    let mut aw = a.data.clone();
    let mut bw = b.clone();
    assert!(lu_solve(6, &mut aw, &mut bw));
    let want = dm(&a)
        .lu()
        .solve(&DMatrix::from_column_slice(6, 1, &b))
        .expect("reference LU solve");
    assert_vec_close(&bw, want.as_slice(), 1e-9, "lu_solve");
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

/// The symmetric eigenvalues, against nalgebra's, on random symmetric matrices of mixed sign —
/// and the property: their sum is the trace.
#[test]
fn symmetric_eigenvalues_agree_with_nalgebra() {
    for (seed, n) in [(1u32, 1usize), (2, 2), (3, 5), (4, 9), (5, 16)] {
        let r = rand(seed, n, n);
        let mut a = Mat::zeros(n, n);
        for i in 0..n {
            for j in 0..n {
                a.data[i * n + j] = r.data[i * n + j] + r.data[j * n + i];
            }
        }
        let got = gcs_core::linalg::sym_eigenvalues(&a);
        let mut want: Vec<f64> = dm(&a).symmetric_eigen().eigenvalues.iter().copied().collect();
        want.sort_by(|x, y| x.total_cmp(y));
        assert_vec_close(&got, &want, 1e-10, "eigenvalues");
        let trace: f64 = (0..n).map(|i| a.data[i * n + i]).sum();
        assert!(close(got.iter().sum(), trace, 1e-10));
    }
}

// --- the sparse Cholesky (#510, P1) ------------------------------------------------------------

/// The stiffness of linear elasticity on a block of `nx × ny × nz` unit cubes, each cut into the
/// six tetrahedra about its main diagonal (Kuhn's, so neighbours share their faces), three
/// unknowns a node.  `fixed` holds the nodes of the base `z = 0` still, removing their unknowns;
/// without it the body is free and the stiffness singular six ways.  The matrix P1 is for.
fn tet_stiffness(nx: usize, ny: usize, nz: usize, fixed: bool) -> Symmetric {
    let (e, nu) = (200e3, 0.3);
    let node = |i: usize, j: usize, k: usize| (k * (ny + 1) + j) * (nx + 1) + i;
    let at = |v: usize| {
        let (i, j, k) = (v % (nx + 1), (v / (nx + 1)) % (ny + 1), v / ((nx + 1) * (ny + 1)));
        [i as f64, j as f64, k as f64]
    };
    let nodes = (nx + 1) * (ny + 1) * (nz + 1);
    let mut dof = vec![usize::MAX; 3 * nodes];
    let mut n = 0;
    for v in 0..nodes {
        if fixed && at(v)[2] == 0.0 {
            continue;
        }
        for c in 0..3 {
            dof[3 * v + c] = n;
            n += 1;
        }
    }
    // isotropic D, Voigt order xx yy zz yz xz xy
    let lam = e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
    let mu = e / (2.0 * (1.0 + nu));
    let mut d = [[0.0; 6]; 6];
    for a in 0..3 {
        for b in 0..3 {
            d[a][b] = lam + if a == b { 2.0 * mu } else { 0.0 };
        }
        d[a + 3][a + 3] = mu;
    }
    let mut t = Vec::new();
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                for p in [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]] {
                    let mut c = [i, j, k];
                    let mut tet = [node(c[0], c[1], c[2]); 4];
                    for (s, &axis) in p.iter().enumerate() {
                        c[axis] += 1;
                        tet[s + 1] = node(c[0], c[1], c[2]);
                    }
                    let x: Vec<[f64; 3]> = tet.iter().map(|&v| at(v)).collect();
                    // J's columns are the edges from vertex 0; the gradients of N1..N3 are
                    // the rows of J⁻¹
                    let jm: Vec<[f64; 3]> =
                        (1..4).map(|s| [0, 1, 2].map(|r| x[s][r] - x[0][r])).collect();
                    let m = Mat::from_vec(3, 3, (0..9).map(|q| jm[q % 3][q / 3]).collect());
                    let inv = dm(&m).try_inverse().expect("a tetrahedron of volume");
                    let vol = dm(&m).determinant().abs() / 6.0;
                    let mut g = [[0.0; 3]; 4];
                    for s in 1..4 {
                        for r in 0..3 {
                            g[s][r] = inv[(s - 1, r)];
                            g[0][r] -= g[s][r];
                        }
                    }
                    // B: 6 × 12
                    let mut bm = [[0.0; 12]; 6];
                    for s in 0..4 {
                        let [gx, gy, gz] = g[s];
                        bm[0][3 * s] = gx;
                        bm[1][3 * s + 1] = gy;
                        bm[2][3 * s + 2] = gz;
                        bm[3][3 * s + 1] = gz;
                        bm[3][3 * s + 2] = gy;
                        bm[4][3 * s] = gz;
                        bm[4][3 * s + 2] = gx;
                        bm[5][3 * s] = gy;
                        bm[5][3 * s + 1] = gx;
                    }
                    for a in 0..12 {
                        let ra = dof[3 * tet[a / 3] + a % 3];
                        for b in 0..12 {
                            let rb = dof[3 * tet[b / 3] + b % 3];
                            if ra == usize::MAX || rb == usize::MAX {
                                continue;
                            }
                            let mut v = 0.0;
                            for p in 0..6 {
                                for q in 0..6 {
                                    v += bm[p][a] * d[p][q] * bm[q][b];
                                }
                            }
                            t.push((ra, rb, vol * v));
                        }
                    }
                }
            }
        }
    }
    Symmetric::from_triplets(n, &t)
}

/// A random symmetric positive definite pattern: `extra` random off-diagonal entries per column,
/// the diagonal dominating.
fn random_spd(seed: u32, n: usize, extra: usize) -> Symmetric {
    let mut rng = Rng::new(seed);
    let mut t = Vec::new();
    let mut row = vec![0.0; n];
    for j in 0..n {
        for _ in 0..extra {
            let i = ((rng.uniform(0.0, 1.0) * n as f64) as usize).min(n - 1);
            if i != j {
                let v = rng.normal(0.0, 1.0);
                t.push((i.max(j), i.min(j), v));
                row[i] += v.abs();
                row[j] += v.abs();
            }
        }
    }
    for j in 0..n {
        t.push((j, j, row[j] + 0.5 + rng.uniform(0.0, 1.0)));
    }
    Symmetric::from_triplets(n, &t)
}

fn dense_of(a: &Symmetric) -> DMatrix<f64> {
    let mut m = DMatrix::zeros(a.n, a.n);
    for j in 0..a.n {
        for q in a.colptr[j]..a.colptr[j + 1] {
            m[(a.rowind[q], j)] = a.values[q];
            m[(j, a.rowind[q])] = a.values[q];
        }
    }
    m
}

fn is_permutation(p: &[usize]) -> bool {
    let mut seen = vec![false; p.len()];
    p.iter().all(|&i| i < p.len() && !std::mem::replace(&mut seen[i], true))
}

const ORDERINGS: [Ordering; 3] = [Ordering::Amd, Ordering::Rcm, Ordering::Natural];

/// The factor is unique, so ours is the reference's: L of PAPᵀ entry for entry against
/// `nalgebra`'s dense Cholesky of the same permuted matrix — and the property, `PAPᵀ ≈ LLᵀ`,
/// with every entry L holds inside the structure the analysis counted.
#[test]
fn sparse_cholesky_is_the_reference_factor() {
    let cases = [
        tet_stiffness(2, 2, 3, true),
        tet_stiffness(3, 1, 2, true),
        random_spd(1, 60, 2),
        random_spd(2, 90, 4),
        random_spd(3, 1, 0),
    ];
    for (ci, a) in cases.iter().enumerate() {
        let n = a.n;
        for ordering in ORDERINGS {
            let mut ch = Cholesky::analyse(a, ordering.clone());
            assert!(is_permutation(ch.perm()), "case {ci} {ordering:?}: not a permutation");
            ch.factor(a).unwrap_or_else(|e| panic!("case {ci} {ordering:?}: {e:?}"));
            let p = ch.perm().to_vec();
            let full = dense_of(a);
            let pap = DMatrix::from_fn(n, n, |i, j| full[(p[i], p[j])]);
            let want = pap.clone().cholesky().expect("reference: positive definite").l();
            let mut l = DMatrix::zeros(n, n);
            let entries = ch.l();
            assert_eq!(entries.len(), ch.factor_nnz(), "case {ci} {ordering:?}: L's count");
            for &(i, j, v) in &entries {
                assert!(i >= j);
                l[(i, j)] = v;
            }
            let scale = want.amax();
            for i in 0..n {
                for j in 0..n {
                    assert!(
                        close(l[(i, j)], want[(i, j)], 1e-10 * scale),
                        "case {ci} {ordering:?}: L[{i},{j}] {} vs {}",
                        l[(i, j)],
                        want[(i, j)]
                    );
                }
            }
            let r = (&pap - &l * l.transpose()).amax();
            assert!(r <= 1e-12 * pap.amax(), "case {ci} {ordering:?}: |PAPᵀ − LLᵀ| = {r}");
        }
    }
}

/// A stiffness of a few thousand unknowns solved: `K u = f` to the bit against the reference's
/// dense solve, and the residual small — with the factor of the same pattern taken twice, for a
/// mesh moved within a solve refactors without its analysis.
#[test]
fn sparse_cholesky_solves_a_tetrahedral_stiffness() {
    let mut a = tet_stiffness(6, 5, 7, true);
    let n = a.n;
    let mut ch = Cholesky::analyse(&a, Ordering::Amd);
    for pass in 0..2 {
        if pass == 1 {
            // new values on the same pattern: a stiffer material, a perturbed matrix
            let mut rng = Rng::new(9);
            for j in 0..n {
                for q in a.colptr[j]..a.colptr[j + 1] {
                    a.values[q] *= 3.0;
                    if a.rowind[q] == j {
                        a.values[q] += rng.uniform(0.0, 1.0);
                    }
                }
            }
        }
        ch.factor(&a).expect("a supported body");
        let f = rand_vec(510 + pass, n);
        let mut u = f.clone();
        ch.solve(&mut u);
        let r: f64 = a.mul_vec(&u).iter().zip(&f).map(|(x, y)| (x - y).powi(2)).sum();
        let fnorm: f64 = f.iter().map(|x| x * x).sum();
        assert!(r.sqrt() <= 1e-11 * fnorm.sqrt(), "pass {pass}: residual {}", r.sqrt());
        let want = dense_of(&a)
            .cholesky()
            .expect("reference: positive definite")
            .solve(&DMatrix::from_column_slice(n, 1, &f));
        let umax = u.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert_vec_close(&u, want.as_slice(), 1e-9 * umax, "u against the dense solve");
    }
}

/// The 7-point Laplacian of a `k³` grid, the scalar model of a 3D mesh.
fn grid_laplacian(k: usize) -> Symmetric {
    let id = |i: usize, j: usize, l: usize| (l * k + j) * k + i;
    let mut t = Vec::new();
    for l in 0..k {
        for j in 0..k {
            for i in 0..k {
                let v = id(i, j, l);
                t.push((v, v, 6.1));
                if i + 1 < k {
                    t.push((id(i + 1, j, l), v, -1.0));
                }
                if j + 1 < k {
                    t.push((id(i, j + 1, l), v, -1.0));
                }
                if l + 1 < k {
                    t.push((id(i, j, l + 1), v, -1.0));
                }
            }
        }
    }
    Symmetric::from_triplets(k * k * k, &t)
}

/// The reason for P1: a band order fills a 3D mesh like `n^{5/3}`, a minimum-degree order far
/// more slowly.  On the 7-point grid AMD's factor is 0.55 of RCM's at 10³ and under half at
/// 20³, the gap widening with the block; on the tetrahedral block of 3630 unknowns it is under
/// 0.85 of RCM's, and its supernodes average at least the three unknowns of a node wide.
#[test]
fn amd_fills_a_3d_mesh_less_than_a_band_order() {
    let ratio = |a: &Symmetric| {
        let amd = Cholesky::analyse(a, Ordering::Amd).factor_nnz() as f64;
        amd / Cholesky::analyse(a, Ordering::Rcm).factor_nnz() as f64
    };
    let (r10, r20) = (ratio(&grid_laplacian(10)), ratio(&grid_laplacian(20)));
    assert!(r10 < 0.6 && r20 < 0.5 && r20 < r10, "AMD/RCM fill: {r10} at 10³, {r20} at 20³");

    let a = tet_stiffness(10, 10, 10, true);
    assert_eq!(a.n, 3630);
    let r = ratio(&a);
    assert!(r < 0.85, "AMD/RCM fill on the tetrahedral block: {r}");
    let amd = Cholesky::analyse(&a, Ordering::Amd);
    assert!(3 * amd.supernodes() <= a.n, "{} supernodes of {} columns", amd.supernodes(), a.n);
}

/// A pivot within the rounding of its own column is refused, naming the column: one exactly
/// zero, one negative, and a free body's, whose six rigid-body modes AMD's order leaves to
/// rounding in the blocks measured (held at one node, it still turns).  The factor does not
/// promise more: under a band order a free body's pivots can round to just above the bound,
/// and a slender supported one's fall to 1e-9 of their diagonal honestly, so naming a freedom
/// is the diagnosis's (#510, P7), from the supports.  A supported body factors with every
/// pivot a fair fraction of its diagonal.
#[test]
fn a_pivot_lost_to_rounding_is_refused() {
    let zero = Symmetric::from_triplets(2, &[(0, 0, 1.0), (1, 0, 1.0), (1, 1, 1.0)]);
    let mut ch = Cholesky::analyse(&zero, Ordering::Natural);
    assert_eq!(ch.factor(&zero).map_err(|e| e.column), Err(1));
    let neg = Symmetric::from_triplets(2, &[(0, 0, 1.0), (1, 0, 2.0), (1, 1, 1.0)]);
    let mut ch = Cholesky::analyse(&neg, Ordering::Natural);
    assert_eq!(ch.factor(&neg).map_err(|e| e.column), Err(1));

    for k in [3usize, 6, 12] {
        let free = tet_stiffness(k, k, k, false);
        let mut ch = Cholesky::analyse(&free, Ordering::Amd);
        let err = ch.factor(&free).expect_err("a free body is singular");
        assert!(err.column < free.n && err.pivot.is_finite());
    }
    // one node held: drop its three unknowns, here node 0's
    let free = tet_stiffness(3, 3, 3, false);
    let mut t = Vec::new();
    for j in 3..free.n {
        for q in free.colptr[j]..free.colptr[j + 1] {
            t.push((free.rowind[q] - 3, j - 3, free.values[q]));
        }
    }
    let pinned = Symmetric::from_triplets(free.n - 3, &t);
    let mut ch = Cholesky::analyse(&pinned, Ordering::Amd);
    assert!(ch.factor(&pinned).is_err(), "a body held at one point turns about it");

    let held = tet_stiffness(6, 6, 6, true);
    let mut diag = vec![0.0; held.n];
    for j in 0..held.n {
        diag[j] = held.values[held.colptr[j]];
        assert_eq!(held.rowind[held.colptr[j]], j);
    }
    for ordering in ORDERINGS {
        let mut ch = Cholesky::analyse(&held, ordering.clone());
        ch.factor(&held).expect("a supported body");
        let p = ch.perm().to_vec();
        for (i, j, v) in ch.l() {
            if i == j {
                let a = diag[p[i]];
                assert!(v * v > 0.1 * a, "{ordering:?}: pivot {} of {a}", v * v);
            }
        }
    }
}

/// A hub every other column touches is *dense* by AMD's rule and ordered last, where it costs
/// one row of L and nothing fills; first, it fills the whole factor.
#[test]
fn amd_orders_a_dense_hub_last() {
    let n = 400;
    let mut t: Vec<(usize, usize, f64)> = (0..n).map(|j| (j, j, n as f64)).collect();
    t.extend((1..n).map(|i| (i, 0, 1.0)));
    let a = Symmetric::from_triplets(n, &t);
    let amd = Cholesky::analyse(&a, Ordering::Amd);
    assert_eq!(amd.perm()[n - 1], 0, "the hub last");
    assert_eq!(amd.factor_nnz(), a.rowind.len(), "no fill");
    let natural = Cholesky::analyse(&a, Ordering::Natural);
    assert_eq!(natural.factor_nnz(), n * (n + 1) / 2, "the hub first fills everything");
}

/// The orders are permutations on every shape of pattern: empty, diagonal, disconnected, one
/// column, dense — and the factor of each solves.
#[test]
fn sparse_cholesky_orders_are_permutations() {
    let mut cases = vec![Symmetric::from_triplets(0, &[])];
    cases.push(Symmetric::from_triplets(5, &(0..5).map(|j| (j, j, 2.0)).collect::<Vec<_>>()));
    // two blocks with nothing between them
    let mut t = Vec::new();
    for j in 0..6 {
        t.push((j, j, 4.0));
        if j % 3 != 2 {
            t.push((j + 1, j, -1.0));
        }
    }
    cases.push(Symmetric::from_triplets(6, &t));
    let mut t = Vec::new();
    for j in 0..12 {
        for i in j..12 {
            t.push((i, j, if i == j { 20.0 } else { 1.0 }));
        }
    }
    cases.push(Symmetric::from_triplets(12, &t));
    for seed in 0..20u32 {
        cases.push(random_spd(100 + seed, 2 + 7 * seed as usize, 1 + (seed % 5) as usize));
    }
    for (ci, a) in cases.iter().enumerate() {
        for ordering in ORDERINGS {
            let mut ch = Cholesky::analyse(a, ordering.clone());
            assert_eq!(ch.perm().len(), a.n);
            assert!(is_permutation(ch.perm()), "case {ci} {ordering:?}");
            ch.factor(a).unwrap_or_else(|e| panic!("case {ci} {ordering:?}: {e:?}"));
            let b = rand_vec(ci as u32, a.n);
            let mut x = b.clone();
            ch.solve(&mut x);
            assert_vec_close(&a.mul_vec(&x), &b, 1e-10, &format!("case {ci} {ordering:?}"));
        }
    }
}
