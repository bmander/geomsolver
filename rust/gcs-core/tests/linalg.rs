//! The core's dense linear algebra checked against an independent implementation.
//!
//! There is no LAPACK/BLAS anywhere in the project: the pivoted QR, the complete orthogonal
//! decomposition behind the minimum-norm step, the SVD, the LU, the sparse LDLᵀ and the sparse
//! eigensolver (`eigen.rs`, #510) are ours.  `nalgebra` is the
//! reference that keeps them honest — the one place two implementations are still compared, on
//! purpose.  It is a **dev-dependency**, so nothing it brings reaches the cdylib or the wasm;
//! the library's own promise of no dependencies is unchanged.
//!
//! Each test also states the property the reference cannot: `A ≈ QR` on the pivots,
//! `NᵀN ≈ I`, a minimum-norm solution orthogonal to the null space, eigenvectors B-orthonormal
//! and nothing missed below the highest found.  A reference agreeing is
//! evidence; a property holding is the contract.

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
/// and the properties: their sum is the trace, and `sym_eigen`'s vectors are orthonormal
/// eigenvectors.
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
        // and with vectors: the same values, `A V = V Λ`, `VᵀV = I`
        let (values, v) = gcs_core::linalg::sym_eigen(&a);
        assert_eq!(values, got, "sym_eigen's values are sym_eigenvalues'");
        for c in 0..n {
            let lv: Vec<f64> = v.col(c).iter().map(|x| x * values[c]).collect();
            assert_vec_close(&a.mul_vec(&v.col(c)), &lv, 1e-10, "A v = λ v");
            for d in 0..n {
                let vv = dot(&v.col(c), &v.col(d));
                assert!(close(vv, if c == d { 1.0 } else { 0.0 }, 1e-12), "VᵀV at ({c},{d})");
            }
        }
    }
}

/* -- the sparse symmetric factor and the eigensolver (#510, P2) ------------------------------- */

use gcs_core::eigen::{self, Opts, Pairs, Refusal};
use gcs_core::sparse::{Ldl, Sym};

/// A `Sym` as the reference's dense matrix.
fn dense(a: &Sym) -> DMatrix<f64> {
    let mut d = DMatrix::zeros(a.n, a.n);
    for (i, j, v) in a.entries() {
        d[(i, j)] += v;
    }
    d
}

/// The springs of an `nx × ny` grid, random stiffnesses: a membrane's stiffness, as an assembly
/// adds it (each spring's 2×2 block), with the boundary's nodes held to the ground by a spring
/// of their own when `held`.
fn grid(seed: u32, nx: usize, ny: usize, held: bool) -> Sym {
    let mut rng = Rng::new(seed);
    let id = |i: usize, j: usize| i * ny + j;
    let mut t: Vec<(usize, usize, f64)> = Vec::new();
    let spring = |t: &mut Vec<(usize, usize, f64)>, a: usize, b: usize, k: f64| {
        t.extend([(a, a, k), (b, b, k), (a, b, -k), (b, a, -k)]);
    };
    for i in 0..nx {
        for j in 0..ny {
            if i + 1 < nx {
                spring(&mut t, id(i, j), id(i + 1, j), rng.uniform(0.5, 2.0));
            }
            if j + 1 < ny {
                spring(&mut t, id(i, j), id(i, j + 1), rng.uniform(0.5, 2.0));
            }
            if held && (i == 0 || j == 0 || i + 1 == nx || j + 1 == ny) {
                t.push((id(i, j), id(i, j), rng.uniform(0.5, 2.0)));
            }
        }
    }
    Sym::from_triplets(nx * ny, &t)
}

/// A consistent-looking mass on the same grid: each node's own, and a sixth of the lighter
/// neighbour's coupling along every spring — diagonally dominant, so positive definite.
fn grid_mass(seed: u32, nx: usize, ny: usize) -> Sym {
    let mut rng = Rng::new(seed);
    let n = nx * ny;
    let m: Vec<f64> = (0..n).map(|_| rng.uniform(1.0, 3.0)).collect();
    let mut t: Vec<(usize, usize, f64)> = (0..n).map(|i| (i, i, m[i])).collect();
    for i in 0..nx {
        for j in 0..ny {
            let a = i * ny + j;
            let right = [(i + 1 < nx).then(|| a + ny), (j + 1 < ny).then(|| a + 1)];
            for b in right.into_iter().flatten() {
                let c = m[a].min(m[b]) / 6.0;
                t.extend([(a, b, c), (b, a, c)]);
            }
        }
    }
    Sym::from_triplets(n, &t)
}

/// The pencil's eigenvalues by the reference, ascending: `B = L Lᵀ`, then `L⁻¹ A L⁻ᵀ`'s.
fn ref_pencil(a: &DMatrix<f64>, b: &DMatrix<f64>) -> Vec<f64> {
    let l = b.clone().cholesky().expect("a positive definite B").l();
    let li = l.try_inverse().expect("an invertible factor");
    let c = &li * a * li.transpose();
    let c = (&c + c.transpose()) * 0.5;
    let mut ev: Vec<f64> = c.symmetric_eigen().eigenvalues.iter().copied().collect();
    ev.sort_by(|x, y| x.total_cmp(y));
    ev
}

/// The properties a solve of `A φ = λ B φ` is held to, whatever the reference says: the vectors
/// B-orthonormal, A-orthogonal with λ on the diagonal, every residual small, and nothing missed:
/// Sylvester's count just below the highest value found equals the number found below that, and
/// just above it is at least the number found (more when `count` cut a repeated value).
fn assert_pencil_pairs(a: &Sym, b: &Sym, got: &Pairs, what: &str) {
    let k = got.values.len();
    let ab: Vec<(Vec<f64>, Vec<f64>)> =
        got.vectors.iter().map(|x| (a.mul_vec(x), b.mul_vec(x))).collect();
    for i in 0..k {
        assert!(got.residuals[i] < 1e-9, "{what}: residual of pair {i}: {}", got.residuals[i]);
        for j in 0..k {
            let bij = dot(&got.vectors[i], &ab[j].1);
            let aij = dot(&got.vectors[i], &ab[j].0);
            let want = if i == j { 1.0 } else { 0.0 };
            assert!(close(bij, want, 1e-9), "{what}: ΦᵀBΦ at ({i},{j}): {bij}");
            let scale = got.values[k - 1].abs().max(1.0);
            assert!(close(aij, want * got.values[i], 1e-9 * scale), "{what}: ΦᵀAΦ at ({i},{j})");
        }
    }
    assert!(got.values.windows(2).all(|w| w[0] <= w[1]), "{what}: values ascending");
    let top = got.values[k - 1];
    let eta = 1e-7 * top.abs().max(1.0);
    let below = |mu: f64| Ldl::new(&a.combine(1.0, b, -mu)).expect("a factor").negative_pivots();
    let under = got.values.iter().filter(|&&v| v < top - eta).count();
    assert_eq!(below(top - eta), under, "{what}: Sylvester's count below {top}");
    assert!(below(top + eta) >= k, "{what}: Sylvester's count above {top}");
}

/// The factor solves, and its pivots carry the matrix's inertia: an indefinite membrane (the
/// grid shifted into its spectrum) against the reference's solve and its eigenvalues' signs.
#[test]
fn sparse_ldl_solves_and_counts_negative_eigenvalues() {
    for (seed, nx, ny, shift) in [(1u32, 6usize, 5usize, 0.0), (2, 7, 7, 1.3), (3, 4, 9, 3.1)] {
        let a = grid(seed, nx, ny, true).combine(1.0, &grid_mass(seed + 10, nx, ny), -shift);
        let n = a.n;
        let f = Ldl::new(&a).expect("a factor");
        let b = rand_vec(100 + seed, n);
        let mut x = b.clone();
        f.solve(&mut x);
        // the property: A x = b
        assert_vec_close(&a.mul_vec(&x), &b, 1e-9, "A x");
        let want = dense(&a).lu().solve(&DMatrix::from_column_slice(n, 1, &b)).unwrap();
        assert_vec_close(&x, want.as_slice(), 1e-9, "solve against the reference");
        let negative = dense(&a).symmetric_eigen().eigenvalues.iter().filter(|&&e| e < 0.0).count();
        assert_eq!(f.negative_pivots(), negative, "inertia, seed {seed}");
        if shift == 0.0 {
            assert_eq!(negative, 0, "the held membrane is positive definite");
        } else {
            assert!(negative > 0, "the shift is inside the spectrum");
        }
    }
}

/// The lowest modes of a held membrane with a consistent mass: distinct values, against the
/// reference's dense generalized eigen-decomposition.
#[test]
fn lowest_modes_agree_with_the_dense_reference() {
    for (seed, nx, ny, count) in [(4u32, 8usize, 6usize, 6usize), (5, 12, 9, 10), (6, 3, 3, 9)] {
        let (k, m) = (grid(seed, nx, ny, true), grid_mass(seed + 20, nx, ny));
        let got = eigen::lowest(&k, &m, count, &Opts::default()).expect("the modes");
        assert_eq!(got.values.len(), count);
        let want = ref_pencil(&dense(&k), &dense(&m));
        for (i, (&g, &w)) in got.values.iter().zip(&want).enumerate() {
            assert!(close(g, w, 1e-9 * w.abs()), "mode {i}, seed {seed}: {g} vs {w}");
        }
        assert_pencil_pairs(&k, &m, &got, &format!("membrane {nx}×{ny}"));
    }
}

/// A free ring of equal masses and springs: a rigid-body turn at zero, and every other mode a
/// pair, `λ_j = 4 (k/m) sin²(π j / N)` twice.  Ten cut the fifth pair: Sylvester's count above
/// the tenth value says two are there, the deflated iteration finds the copy the first run did not
/// return, and the answer is the lowest ten.
#[test]
fn a_free_ring_has_its_rigid_mode_and_every_pair() {
    let (nodes, k, m) = (120usize, 3.0, 2.0);
    let mut t: Vec<(usize, usize, f64)> = Vec::new();
    for a in 0..nodes {
        let b = (a + 1) % nodes;
        t.extend([(a, a, k), (b, b, k), (a, b, -k), (b, a, -k)]);
    }
    let stiff = Sym::from_triplets(nodes, &t);
    let mass = Sym::from_triplets(nodes, &(0..nodes).map(|i| (i, i, m)).collect::<Vec<_>>());
    let count = 10;
    let got = eigen::lowest(&stiff, &mass, count, &Opts::default()).expect("the modes");
    assert_eq!(got.values.len(), count);
    let pi = std::f64::consts::PI;
    let lam = |j: usize| 4.0 * k / m * (pi * j as f64 / nodes as f64).sin().powi(2);
    let analytic = [0.0, lam(1), lam(1), lam(2), lam(2), lam(3), lam(3), lam(4), lam(4), lam(5)];
    assert_vec_close(&got.values, &analytic, 1e-10, "the ring's modes");
    let want = ref_pencil(&dense(&stiff), &dense(&mass));
    assert_vec_close(&got.values, &want[..count], 1e-10, "against the reference");
    // the zero is the ring turning rigidly: every node moves alike
    let turn = &got.vectors[0];
    assert!(turn.iter().all(|v| close(*v, turn[0], 1e-8)), "a rigid turn: {turn:?}");
    assert_pencil_pairs(&stiff, &mass, &got, "the ring");
}

/// A larger held membrane, past the size a dense solve would be asked to take: the properties
/// alone, Sylvester's count among them.
#[test]
fn lowest_modes_of_a_large_membrane_hold_their_properties() {
    let (nx, ny) = (60usize, 50usize);
    let (k, m) = (grid(7, nx, ny, true), grid_mass(8, nx, ny));
    let got = eigen::lowest(&k, &m, 8, &Opts::default()).expect("the modes");
    assert_eq!(got.values.len(), 8);
    assert_pencil_pairs(&k, &m, &got, "membrane 60×50");
}

/// Euler–Bernoulli elements on a column of `elements` equal lengths: the stiffness and the
/// geometric stiffness of a unit compression, each element's 4×4 on (w, θ) at its two ends.
/// `held` lists the freedoms fixed (dropped).
fn column(elements: usize, length: f64, ei: f64, held: &[usize]) -> (Sym, Sym) {
    let h = length / elements as f64;
    let ke = [
        [12.0, 6.0 * h, -12.0, 6.0 * h],
        [6.0 * h, 4.0 * h * h, -6.0 * h, 2.0 * h * h],
        [-12.0, -6.0 * h, 12.0, -6.0 * h],
        [6.0 * h, 2.0 * h * h, -6.0 * h, 4.0 * h * h],
    ];
    let kg = [
        [36.0, 3.0 * h, -36.0, 3.0 * h],
        [3.0 * h, 4.0 * h * h, -3.0 * h, -h * h],
        [-36.0, -3.0 * h, 36.0, -3.0 * h],
        [3.0 * h, -h * h, -3.0 * h, 4.0 * h * h],
    ];
    let all = 2 * (elements + 1);
    let mut index = vec![usize::MAX; all];
    let mut n = 0;
    for d in 0..all {
        if !held.contains(&d) {
            index[d] = n;
            n += 1;
        }
    }
    let (mut tk, mut tg) = (Vec::new(), Vec::new());
    for e in 0..elements {
        for a in 0..4 {
            for b in 0..4 {
                let (i, j) = (index[2 * e + a], index[2 * e + b]);
                if i != usize::MAX && j != usize::MAX {
                    tk.push((i, j, ei / (h * h * h) * ke[a][b]));
                    // a compression softens: K_G = −P/(30h)·kg for P = 1
                    tg.push((i, j, -kg[a][b] / (30.0 * h)));
                }
            }
        }
    }
    (Sym::from_triplets(n, &tk), Sym::from_triplets(n, &tg))
}

/// Euler's loads: a pinned–pinned column buckles at `j² π² EI / L²`, a cantilever first at
/// `π² EI / (4 L²)` — against the analytic values (to the elements' discretisation) and the
/// reference's dense decomposition of the same pencil (to rounding).
#[test]
fn buckling_finds_eulers_loads() {
    let (elements, length, ei) = (24usize, 2.0, 5.0);
    let last = 2 * elements;
    let pi = std::f64::consts::PI;
    let euler = pi * pi * ei / (length * length);
    for (held, analytic) in [
        (vec![0, last], vec![euler, 4.0 * euler, 9.0 * euler]),
        (vec![0, 1], vec![euler / 4.0, 9.0 * euler / 4.0, 25.0 * euler / 4.0]),
    ] {
        let (k, kg) = column(elements, length, ei, &held);
        let got = eigen::buckling(&k, &kg, 3, &Opts::default()).expect("the loads");
        assert_eq!(got.values.len(), 3);
        for (i, (&g, &w)) in got.values.iter().zip(&analytic).enumerate() {
            // cubic elements: the error grows as (j h / L)⁴, 3e-5 for the third pinned load
            assert!(close(g, w, 1e-4 * w), "load {i} held at {held:?}: {g} vs Euler's {w}");
        }
        // the reference: −K_G φ = θ K φ, θ = 1/λ, the largest θ first
        let mut theta = ref_pencil(&(-dense(&kg)), &dense(&k));
        theta.reverse();
        for (i, &g) in got.values.iter().enumerate() {
            assert!(close(g, 1.0 / theta[i], 1e-9 * g), "load {i}: {g} vs {}", 1.0 / theta[i]);
        }
        for i in 0..3 {
            assert!(got.residuals[i] < 1e-9, "residual {i}: {}", got.residuals[i]);
            let kx = k.mul_vec(&got.vectors[i]);
            for j in 0..3 {
                let v = dot(&got.vectors[j], &kx);
                assert!(close(v, if i == j { 1.0 } else { 0.0 }, 1e-9), "ΦᵀKΦ at ({i},{j})");
            }
        }
        // nothing missed: K + μ K_G has as many negative pivots as loads below μ
        let mu = got.values[2] * (1.0 + 1e-7);
        assert_eq!(Ldl::new(&k.combine(1.0, &kg, mu)).unwrap().negative_pivots(), 3);
    }
}

/// A tension buckles nothing, and a column with nothing holding it is free to move rigidly:
/// no load, and a stiffness the iteration cannot draw its inner product from.
#[test]
fn buckling_refuses_what_has_no_load() {
    let (k, kg) = column(10, 1.0, 1.0, &[0, 20]);
    let tension = kg.combine(-1.0, &kg, 0.0);
    let got = eigen::buckling(&k, &tension, 3, &Opts::default()).expect("a solve");
    assert!(got.values.is_empty(), "a tension's loads: {:?}", got.values);
    let (free, kg) = column(10, 1.0, 1.0, &[]);
    assert_eq!(eigen::buckling(&free, &kg, 3, &Opts::default()).err(), Some(Refusal::NotDefinite));
    // and a mass that is not definite is refused for the modes
    let massless = Sym::from_triplets(free.n, &[(0, 0, 1.0)]);
    let modes = eigen::lowest(&free, &massless, 2, &Opts::default());
    assert_eq!(modes.err(), Some(Refusal::NotDefinite));
}
