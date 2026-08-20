"""Dense linear algebra: thin wrappers over the core's own factorizations.

No LAPACK/BLAS anywhere in the project — the QR, complete-orthogonal, SVD and LDLᵀ routines are
ours, and `tests/test_linalg.py` checks them against numpy.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from gcs import _ffi
from gcs._ffi import Vec, lib

_RCOND = 1e-12


def min_norm_lstsq(J: Any, b: Any, rcond: float = _RCOND) -> tuple[Vec, int]:
    """Minimum-norm least-squares solution of `J p = b` via a complete orthogonal decomposition
    (rank-revealing QR + RZ).  Returns (p, numerical rank); `b` may be a matrix of right-hand
    sides, and `p` then has a column per right-hand side."""
    A = _ffi.as_f64(J)
    B = np.asarray(b, dtype=np.float64)
    vector = B.ndim == 1
    m, n = A.shape
    B = np.ascontiguousarray(B.reshape(m, -1))
    x = _ffi.f64(max(n * B.shape[1], 1))
    rank = lib.gcs_min_norm_lstsq(m, n, B.shape[1], _ffi.pf(A), _ffi.pf(B), rcond, _ffi.pf(x))
    X = x[: n * B.shape[1]].reshape(n, B.shape[1])
    return (X[:, 0] if vector else X), int(rank)


def rrqr(J: Any, rcond: float = 1e-10) -> tuple[int, Any]:
    """Rank-revealing QR: (numerical rank, column pivots).  The first `rank` pivots index a
    maximal independent set of columns — the codebase's one rank convention."""
    A = _ffi.as_f64(J)
    if A.size == 0:
        return 0, np.zeros(0, dtype=np.int32)
    m, n = A.shape
    piv = _ffi.i32(n)
    rank = lib.gcs_rrqr(m, n, _ffi.pf(A), rcond, _ffi.pi(piv))
    return int(rank), piv


def rank_rrqr(J: Any, rcond: float = 1e-10) -> int:
    return rrqr(J, rcond)[0]


def svd(J: Any, want_u: bool = True) -> tuple[Vec, Vec, Vec]:
    """(U, singular values, Vᵀ) by Golub–Reinsch."""
    A = _ffi.as_f64(J)
    m, n = A.shape
    mn = min(m, n)
    u = _ffi.f64(max(m * mn, 1))
    s = _ffi.f64(max(mn, 1))
    vt = _ffi.f64(max(n * n, 1))
    if lib.gcs_svd(m, n, _ffi.pf(A), _ffi.pf(u) if want_u else None,
                   _ffi.pf(s), _ffi.pf(vt)) != 0:
        raise np.linalg.LinAlgError(_ffi.last_error() or "SVD did not converge")
    U = u[: m * mn].reshape(m, mn) if want_u else np.zeros((m, 0))
    return U, s[:mn], vt[: n * n].reshape(n, n)


def rank_and_nullspace(J: Any, rcond: float = 1e-10) -> tuple[int, Vec, Vec]:
    """(numerical rank, null-space basis, singular values) from a single SVD — the shared seam for
    diagnosis, witness analysis and decomposition, so they agree on what "rank" means."""
    A = _ffi.as_f64(J)
    m, n = A.shape
    if n == 0:
        return 0, np.zeros((0, 0)), np.zeros(0)
    N = _ffi.f64(max(n * n, 1))
    s = _ffi.f64(max(n, 1))
    rank = int(lib.gcs_rank_nullspace(m, n, _ffi.pf(A), rcond, _ffi.pf(N), _ffi.pf(s)))
    if rank < 0:
        raise np.linalg.LinAlgError(_ffi.last_error() or "SVD did not converge")
    nn = n - rank
    return rank, N[: n * nn].reshape(n, nn) if nn else np.zeros((n, 0)), s[: min(m, n)]


def lu_solve(A: Any, b: Any) -> Vec:
    """Solve the square system `A x = b` (partial-pivoting LU)."""
    M = _ffi.as_f64(A).copy()
    v = _ffi.as_f64(b).copy()
    n = M.shape[0]
    if lib.gcs_lu_solve(n, _ffi.pf(M), _ffi.pf(v)) != 0:
        raise np.linalg.LinAlgError("singular matrix")
    return v


__all__ = ["lu_solve", "min_norm_lstsq", "rank_and_nullspace", "rank_rrqr", "rrqr", "svd"]
