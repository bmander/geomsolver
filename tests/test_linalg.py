"""The core's dense linear algebra checked against numpy.

There is no LAPACK/BLAS anywhere in the project: the pivoted QR, the complete orthogonal
decomposition behind the minimum-norm step, the SVD and the LU are ours.  numpy is the independent
reference that keeps them honest — the one place this suite still compares two implementations.
"""

from __future__ import annotations

import numpy as np
import pytest

from gcs import examples
from gcs.linalg import lu_solve, min_norm_lstsq, rank_and_nullspace, rank_rrqr, rrqr, svd
from gcs.solve import System


def _rng(seed: int) -> np.random.Generator:
    return np.random.default_rng(seed)


@pytest.mark.parametrize("shape", [(6, 4), (4, 6), (5, 5), (12, 3), (3, 12)])
def test_min_norm_lstsq_matches_numpy(shape: tuple[int, int]) -> None:
    rng = _rng(shape[0] * 31 + shape[1])
    m, n = shape
    A = rng.normal(size=(m, n))
    b = rng.normal(size=m)
    x, rank = min_norm_lstsq(A, b)
    x_ref, _, rank_ref, _ = np.linalg.lstsq(A, b, rcond=None)
    assert rank == rank_ref
    np.testing.assert_allclose(x, x_ref, atol=1e-9)


def test_min_norm_lstsq_is_minimum_norm_when_rank_deficient() -> None:
    rng = _rng(7)
    A = rng.normal(size=(6, 3))
    A = np.hstack([A, A[:, :1]])            # a duplicated column: rank 3 of 4
    b = rng.normal(size=6)
    x, rank = min_norm_lstsq(A, b)
    assert rank == 3
    x_ref = np.linalg.pinv(A) @ b           # the pseudo-inverse solution *is* the minimum norm one
    np.testing.assert_allclose(x, x_ref, atol=1e-8)


def test_min_norm_lstsq_takes_several_right_hand_sides() -> None:
    rng = _rng(11)
    A = rng.normal(size=(7, 5))
    B = rng.normal(size=(7, 3))
    X, rank = min_norm_lstsq(A, B)
    assert rank == 5 and X.shape == (5, 3)
    for c in range(3):
        np.testing.assert_allclose(X[:, c], min_norm_lstsq(A, B[:, c])[0], atol=1e-9)


@pytest.mark.parametrize("seed", range(4))
def test_rrqr_rank_matches_numpy(seed: int) -> None:
    rng = _rng(seed)
    A = rng.normal(size=(8, 5))
    assert rank_rrqr(A) == np.linalg.matrix_rank(A)
    A[:, 3] = A[:, 0] + 2 * A[:, 1]         # one dependent column
    assert rank_rrqr(A) == 4
    rank, piv = rrqr(A)
    assert rank == 4
    assert sorted(piv.tolist()) == list(range(5))
    # the first `rank` pivots index a maximal independent set of columns
    assert np.linalg.matrix_rank(A[:, piv[:rank]]) == rank


@pytest.mark.parametrize("shape", [(6, 4), (4, 6), (5, 5)])
def test_svd_singular_values_match_numpy(shape: tuple[int, int]) -> None:
    rng = _rng(shape[0] * 13 + shape[1])
    A = rng.normal(size=shape)
    U, s, Vt = svd(A)
    np.testing.assert_allclose(s, np.linalg.svd(A, compute_uv=False), atol=1e-10)
    # and the factorization reconstructs A
    mn = min(shape)
    np.testing.assert_allclose(U @ np.diag(s) @ Vt[:mn], A, atol=1e-9)


def test_rank_and_nullspace_spans_the_kernel() -> None:
    rng = _rng(3)
    A = rng.normal(size=(4, 7))
    rank, N, s = rank_and_nullspace(A)
    assert rank == 4 and N.shape == (7, 3)
    np.testing.assert_allclose(A @ N, 0.0, atol=1e-9)
    np.testing.assert_allclose(N.T @ N, np.eye(3), atol=1e-9)   # orthonormal columns


def test_lu_solve_matches_numpy() -> None:
    rng = _rng(5)
    A = rng.normal(size=(6, 6))
    b = rng.normal(size=6)
    np.testing.assert_allclose(lu_solve(A, b), np.linalg.solve(A, b), atol=1e-9)


def test_sparse_and_dense_jacobians_agree() -> None:
    """The CSR structure is fixed at compile time; only the values are refilled."""
    sk = examples.truss(6)
    examples.perturb(sk, 1.0)
    s = System(sk)
    z = s.z0()
    data, indices, indptr = s.csr(z)
    dense = np.zeros((s.n_res, s.n_free))
    for r in range(s.n_res):
        for p in range(indptr[r], indptr[r + 1]):
            dense[r, indices[p]] = data[p]
    np.testing.assert_allclose(dense, s.jacobian_dense(z))
    s.dispose()


def test_jacobian_rank_agrees_with_numpy_on_a_real_sketch() -> None:
    sk = examples.rect_fillets()
    examples.perturb(sk, 1.0)
    s = System(sk)
    J = s.jacobian_dense(s.z0())
    assert s.rank() == np.linalg.matrix_rank(J)
    s.dispose()
