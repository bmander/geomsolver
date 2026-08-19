"""Our own least-squares solvers: Powell's DogLeg (default) and Levenberg–Marquardt.

Both minimise ½‖r(z)‖² given callbacks for r(z) and J(z), with the uniform
signature `solver(fun, jac, z0, *, ftol, xtol, gtol, max_iter, max_nfev) -> (z, Info)`
recorded in `SOLVERS` (scipy's least_squares methods are wrapped there too, as
references).  The Gauss–Newton step is the *minimum-norm* least-squares
solution of J p = −r, so under-constrained sketches (the normal case while
editing) move as little as possible — least-change behaviour is what users
expect from dragging.

Dense J (ndarray): LAPACK dgelsy (rank-revealing QR) — also gives the rank.
Sparse J (csr): SuperLU on the regularized normal equations.
Reference: Nocedal & Wright ch. 4 & 10; PlaneGCS's DogLeg.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import numpy.typing as npt
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from scipy.linalg import lapack, qr

Vec = npt.NDArray[np.float64]
Mat = Any            # dense ndarray or scipy.sparse csr matrix
Fun = Callable[[Vec], Vec]
Jac = Callable[[Vec], Mat]

STATUS = {0: "residual tolerance reached", 1: "step size below xtol", 2: "gradient below gtol",
          3: "trust region collapsed / damping exhausted", 4: "max iterations reached", -1: "failed"}


@dataclass
class Info:
    status: int
    nfev: int
    njev: int
    iterations: int
    rank: int | None = None          # numerical rank of J at the solution (dense path only)
    r: Vec | None = None             # residual at the returned z (saves a re-evaluation)
    message: str = field(init=False)

    def __post_init__(self) -> None:
        self.message = STATUS.get(self.status, "unknown")


# -- linear algebra primitives -----------------------------------------------

_RCOND = 1e-12
_lwork_cache: dict[tuple[int, int, int], int] = {}


def min_norm_lstsq(J: Vec, b: Vec, rcond: float = _RCOND) -> tuple[Vec, int]:
    """Minimum-norm least-squares solution of J p = b via LAPACK dgelsy (rank-revealing
    QR with column pivoting).  Returns (p, numerical rank).  ~6× faster than
    np.linalg.lstsq (SVD) at sketch sizes.  `b` may be a matrix of right-hand sides —
    they share the one factorisation, and `p` then has a column per right-hand side."""
    m, n = J.shape
    B = np.asarray(b)
    vector = B.ndim == 1
    B = B.reshape(m, -1)
    nrhs = B.shape[1]
    lwork = _lwork_cache.get((m, n, nrhs))
    if lwork is None:
        lwork = _lwork_cache[(m, n, nrhs)] = int(lapack.dgelsy_lwork(m, n, nrhs, rcond)[0].real)
    bb = np.zeros((max(m, n), nrhs))     # LAPACK wants ldb >= max(m, n)
    bb[:m] = B
    _, x, _, rank, info = lapack.dgelsy(J, bb, np.zeros(n, dtype=np.int32), rcond, lwork)
    if info != 0:
        raise np.linalg.LinAlgError(f"dgelsy failed: info={info}")
    return (x[:n, 0] if vector else x[:n]), int(rank)


def rrqr(J: Vec, rcond: float = 1e-10) -> tuple[int, npt.NDArray[np.intp]]:
    """Rank-revealing QR: (numerical rank, column pivots).  The first `rank` pivots index a
    maximal independent set of columns — the codebase's one rank convention: |R_ii| > rcond·|R_00|."""
    if J.size == 0:
        return 0, np.zeros(0, dtype=np.intp)
    R, piv = qr(J, mode="r", pivoting=True, check_finite=False)
    d = np.abs(np.diag(R))
    rank = int(np.count_nonzero(d > rcond * d[0])) if d.size and d[0] > 0 else 0
    return rank, np.asarray(piv, dtype=np.intp)


def rank_rrqr(J: Vec, rcond: float = 1e-10) -> int:
    """Numerical rank via pivoted QR."""
    return rrqr(J, rcond)[0]


def rank_and_nullspace(J: Vec, rcond: float = 1e-10) -> tuple[int, Vec, Vec]:
    """(numerical rank, null-space basis, singular values) from a single SVD — the shared seam
    for diagnosis, witness analysis and decomposition, so they agree on what "rank" means."""
    m, n = J.shape
    if J.size == 0:
        return 0, np.eye(n), np.zeros(0)
    _, sv, Vt = np.linalg.svd(J, full_matrices=True)
    rank = int(np.count_nonzero(sv > rcond * sv[0])) if sv.size and sv[0] > 0 else 0
    return rank, Vt[rank:].T, sv


_EPS_REL = 1e-12


def sparse_gn_step(J: Mat, g: Vec) -> Vec:
    """Sparse Gauss–Newton step from the regularized normal equations
    (JᵀJ + εI) p = −g, g = Jᵀr, factored with SuperLU.  ε = 1e-12·max diag keeps
    rank-deficient (under-constrained) systems solvable and the step within
    ~1e-6 relative of the true minimum-norm one — LSMR/LSQR are 30–50× slower
    here and dense QR is O(n³).  (Sparse QR — SPQR — is the Stage-1 C option.)"""
    A = (J.T @ J).tocsc()
    d = A.diagonal()
    A.setdiag(d + (_EPS_REL * float(d.max(initial=0.0)) or 1e-30))
    return spla.splu(A, permc_spec="MMD_AT_PLUS_A").solve(-g)


def _gn_step(J: Mat, r: Vec, g: Vec) -> tuple[Vec, int | None]:
    """Gauss–Newton step p solving J p ≈ −r (exact minimum-norm on the dense path)."""
    if isinstance(J, np.ndarray):
        return min_norm_lstsq(J, -r)
    return sparse_gn_step(J, g), None   # sp.issparse is an ABC check (25 µs) — avoid on the hot path


def _norm(v: Vec) -> float:
    return math.sqrt(float(v @ v))


def _absmax(v: Vec) -> float:
    return float(np.max(np.abs(v))) if v.size else 0.0


# -- solvers ------------------------------------------------------------------

def dogleg(
    fun: Fun,
    jac: Jac,
    z0: Vec,
    *,
    ftol: float = 1e-12,      # stop when max|r| < ftol (absolute; caller scales by sketch extent²)
    xtol: float = 1e-12,      # stop when ‖p‖ < xtol·(1 + ‖z‖)
    gtol: float = 1e-14,      # stop when ‖Jᵀr‖∞ < gtol
    max_iter: int = 100,
    max_nfev: int | None = None,
) -> tuple[Vec, Info]:
    z = np.array(z0, dtype=np.float64)
    r = fun(z)
    nfev, njev = 1, 0
    delta = np.inf            # first step is the full Gauss–Newton step
    rank: int | None = None
    max_nfev = max_nfev or 4 * max_iter
    for it in range(max_iter):
        if _absmax(r) < ftol:
            return z, Info(0, nfev, njev, it, rank, r)
        J = jac(z)
        njev += 1
        f = 0.5 * float(r @ r)
        g = J.T @ r
        if _absmax(g) < gtol:
            return z, Info(2, nfev, njev, it, rank, r)
        p_gn, rank = _gn_step(J, r, g)
        gn_norm = _norm(p_gn)
        # -- choose the dogleg step inside the trust region --
        if gn_norm <= delta:
            p = p_gn
        else:
            Jg = J @ g
            alpha = float(g @ g) / float(Jg @ Jg)
            p_sd = -alpha * g
            sd_norm = _norm(p_sd)
            if sd_norm >= delta:
                p = p_sd * (delta / sd_norm)
            else:  # on the segment p_sd → p_gn where ‖p‖ = delta
                d = p_gn - p_sd
                a, b, c = float(d @ d), 2 * float(p_sd @ d), sd_norm**2 - delta**2
                tau = (-b + math.sqrt(b * b - 4 * a * c)) / (2 * a)
                p = p_sd + tau * d
        pnorm = _norm(p)
        if pnorm < xtol * (1 + _norm(z)):
            return z, Info(1, nfev, njev, it, rank, r)
        # -- evaluate and update the trust region --
        z_new = z + p
        r_new = fun(z_new)
        nfev += 1
        f_new = 0.5 * float(r_new @ r_new)
        lin = r + J @ p
        pred = f - 0.5 * float(lin @ lin)
        rho = (f - f_new) / pred if pred > 0 else (1.0 if f_new < f else -1.0)
        if rho > 0:
            z, r = z_new, r_new
            if rho > 0.75:
                delta = max(delta, 3 * pnorm) if np.isfinite(delta) else np.inf
            elif rho < 0.25:
                delta = 0.5 * pnorm
        else:
            delta = 0.25 * pnorm
        if delta < 1e-15 * (1 + _norm(z)):
            return z, Info(3, nfev, njev, it + 1, rank, r)
        if nfev >= max_nfev:
            break
    return z, Info(4, nfev, njev, max_iter, rank, r)


def levenberg_marquardt(
    fun: Fun,
    jac: Jac,
    z0: Vec,
    *,
    ftol: float = 1e-12,
    xtol: float = 1e-12,
    gtol: float = 1e-14,
    max_iter: int = 100,
    max_nfev: int | None = None,
    tau: float = 1e-8,
) -> tuple[Vec, Info]:
    """LM with Nielsen's damping update; solves (JᵀJ + λD)p = −Jᵀr with D = diag(JᵀJ)
    (floored so unconstrained parameters stay well-posed).  Small initial τ: sketch
    solves are zero-residual problems started from a warm start, so the first steps
    should be nearly Gauss–Newton."""
    z = np.array(z0, dtype=np.float64)
    r = fun(z)
    nfev, njev = 1, 0
    max_nfev = max_nfev or 4 * max_iter
    lam, nu = -1.0, 2.0
    for it in range(max_iter):
        if _absmax(r) < ftol:
            return z, Info(0, nfev, njev, it, None, r)
        J = jac(z)
        njev += 1
        dense = isinstance(J, np.ndarray)
        A = J.T @ J if dense else (J.T @ J).tocsc()
        g = J.T @ r
        if _absmax(g) < gtol:
            return z, Info(2, nfev, njev, it, None, r)
        diag = np.asarray(A.diagonal()).ravel()
        dmax = float(diag.max(initial=0.0))
        D = np.maximum(diag, 1e-8 * dmax if dmax > 0 else 1e-8)
        if lam < 0:
            lam = tau * (dmax if dmax > 0 else 1.0)
        f = 0.5 * float(r @ r)
        while True:
            if dense:
                Ad = A.copy()
                Ad.flat[:: Ad.shape[0] + 1] += lam * D
                p = np.linalg.solve(Ad, -g)
            else:
                As = A.copy()
                As.setdiag(diag + lam * D)
                p = spla.splu(As, permc_spec="MMD_AT_PLUS_A").solve(-g)
            pnorm = _norm(p)
            if pnorm < xtol * (1 + _norm(z)):
                return z, Info(1, nfev, njev, it, None, r)
            z_new = z + p
            r_new = fun(z_new)
            nfev += 1
            f_new = 0.5 * float(r_new @ r_new)
            pred = 0.5 * float(p @ (lam * D * p - g))
            rho = (f - f_new) / pred if pred > 0 else -1.0
            if rho > 0:
                z, r = z_new, r_new
                lam *= max(1 / 3, 1 - (2 * rho - 1) ** 3)
                nu = 2.0
                break
            lam *= nu
            nu *= 2
            if nfev >= max_nfev or lam > 1e32:
                return z, Info(4 if nfev >= max_nfev else 3, nfev, njev, it + 1, None, r)
    return z, Info(4, nfev, njev, max_iter, None, r)


# -- scipy references ---------------------------------------------------------

def _scipy_solver(method: str) -> Callable[..., tuple[Vec, Info]]:
    """Adapter giving scipy.optimize.least_squares the same signature as our solvers.
    Kept as a reference for benchmarking; 'lm' needs a dense J and m >= n (we pad)."""

    def solver(fun: Fun, jac: Jac, z0: Vec, *, ftol: float = 1e-12, xtol: float = 1e-12,
               gtol: float = 1e-12, max_iter: int = 100, max_nfev: int | None = None) -> tuple[Vec, Info]:
        from scipy.optimize import least_squares as _ls

        least_squares: Any = _ls  # scipy-stubs overloads are too narrow for our kwargs
        kw = dict(ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=max_nfev, x_scale="jac")
        if method == "lm":
            m = fun(z0).size
            pad = max(0, z0.size - m)

            def jac_d(z: Vec) -> Vec:
                J = jac(z)
                J = J.toarray() if not isinstance(J, np.ndarray) else J
                return np.vstack([J, np.zeros((pad, z0.size))]) if pad else J

            res = least_squares(lambda z: np.concatenate([fun(z), np.zeros(pad)]), z0, jac=jac_d, method="lm", **kw)
            r = res.fun[:m]
        else:
            J0 = jac(z0)
            res = least_squares(fun, z0, jac=jac, method=method,
                                tr_solver="exact" if isinstance(J0, np.ndarray) else "lsmr", **kw)
            r = res.fun
        return res.x, Info(int(res.status), int(res.nfev), int(res.njev or 0), int(res.nfev), None, r)

    return solver


SOLVERS: dict[str, Callable[..., tuple[Vec, Info]]] = {
    "dogleg": dogleg,
    "lm": levenberg_marquardt,
    "scipy-dogbox": _scipy_solver("dogbox"),
    "scipy-trf": _scipy_solver("trf"),
    "scipy-lm": _scipy_solver("lm"),
}
