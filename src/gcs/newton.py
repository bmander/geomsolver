"""Our own least-squares solvers: Powell's DogLeg (default) and Levenberg–Marquardt.

Both minimise ½‖r(z)‖² given callbacks for r(z) and J(z).  The Gauss–Newton
step is the *minimum-norm* least-squares solution of J p = −r, so
under-constrained sketches (the normal case while editing) move as little as
possible — least-change behaviour is what users expect from dragging.

Dense path (n_free ≤ DENSE_MAX): J as ndarray, LAPACK lstsq (SVD) — also gives
the numerical rank.  Sparse path: CSR J, LSMR from a zero start (min-norm in
the limit).  Reference: Nocedal & Wright ch. 4 & 10; PlaneGCS's DogLeg.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from scipy.linalg import lapack

Vec = Any  # npt.NDArray[np.float64]; kept loose so callbacks may return csr or ndarray


@dataclass
class Info:
    status: int          # 0 = residual below tol, 1 = step below xtol, 2 = gradient below gtol,
    #                      3 = trust region collapsed, 4 = max iterations, -1 = failed
    message: str
    nfev: int
    njev: int
    iterations: int
    rank: int | None     # numerical rank of J at the solution (dense path only)


_MSG = {0: "residual tolerance reached", 1: "step size below xtol", 2: "gradient below gtol",
        3: "trust region collapsed", 4: "max iterations reached", -1: "failed"}


_RCOND = 1e-12
_lwork_cache: dict[tuple[int, int], int] = {}


def min_norm_lstsq(J: Vec, b: Vec, rcond: float = _RCOND) -> tuple[Vec, int]:
    """Minimum-norm least-squares solution of J p = b via LAPACK dgelsy (rank-revealing
    QR with column pivoting).  Returns (p, numerical rank).  ~6× faster than
    np.linalg.lstsq (SVD) at sketch sizes and gives the rank Stage 2/4 need."""
    m, n = J.shape
    key = (m, n)
    lwork = _lwork_cache.get(key)
    if lwork is None:
        lwork = _lwork_cache[key] = int(lapack.dgelsy_lwork(m, n, 1, rcond)[0].real)
    bb = np.zeros((max(m, n), 1))     # LAPACK wants ldb >= max(m, n)
    bb[:m, 0] = b
    _, x, _, rank, info = lapack.dgelsy(J, bb, np.zeros(n, dtype=np.int32), rcond, lwork)
    if info != 0:
        raise np.linalg.LinAlgError(f"dgelsy failed: info={info}")
    return x[:n, 0], int(rank)


def _norm(v: Vec) -> float:
    return math.sqrt(float(v @ v))


def _absmax(v: Vec) -> float:
    return float(np.max(np.abs(v))) if v.size else 0.0


_EPS_REL = 1e-12


def sparse_gn_step(J: Any, r: Vec) -> Vec:
    """Sparse Gauss–Newton step from the regularized normal equations
    (JᵀJ + εI) p = −Jᵀr, factored with SuperLU.  ε = 1e-12·max diag keeps
    rank-deficient (under-constrained) systems solvable and the step within
    ~1e-6 relative of the true minimum-norm one — LSMR/LSQR are 30–50× slower
    here and dense QR is O(n³).  (Sparse QR — SPQR — is the Stage-1 C option.)"""
    A = (J.T @ J).tocsc()
    eps = _EPS_REL * float(A.diagonal().max(initial=0.0)) or 1e-30
    A = A + sp.identity(A.shape[0], format="csc") * eps
    return spla.splu(A).solve(-(J.T @ r))


def _gn_step(J: Any, r: Vec) -> tuple[Vec, int | None]:
    """Gauss–Newton step p solving J p ≈ −r (minimum-norm on the dense path)."""
    if not isinstance(J, np.ndarray):   # sparse (sp.issparse is an ABC check — 25 µs)
        return sparse_gn_step(J, r), None
    return min_norm_lstsq(J, -r)


def dogleg(
    fun: Callable[[Vec], Vec],
    jac: Callable[[Vec], Any],
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
    max_nfev = max_nfev if max_nfev is not None else 4 * max_iter
    for it in range(max_iter):
        if _absmax(r) < ftol:
            return z, Info(0, _MSG[0], nfev, njev, it, rank)
        J = jac(z)
        njev += 1
        f = 0.5 * float(r @ r)
        g = J.T @ r
        gnorm = _absmax(g)
        if gnorm < gtol:
            return z, Info(2, _MSG[2], nfev, njev, it, rank)
        p_gn, rank = _gn_step(J, r)
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
                tau = (-b + np.sqrt(b * b - 4 * a * c)) / (2 * a)
                p = p_sd + tau * d
        pnorm = _norm(p)
        if pnorm < xtol * (1 + _norm(z)):
            return z, Info(1, _MSG[1], nfev, njev, it, rank)
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
            return z, Info(3, _MSG[3], nfev, njev, it + 1, rank)
        if nfev >= max_nfev:
            break
    return z, Info(4, _MSG[4], nfev, njev, max_iter, rank)


def levenberg_marquardt(
    fun: Callable[[Vec], Vec],
    jac: Callable[[Vec], Any],
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
    max_nfev = max_nfev if max_nfev is not None else 4 * max_iter
    lam, nu = -1.0, 2.0
    for it in range(max_iter):
        if _absmax(r) < ftol:
            return z, Info(0, _MSG[0], nfev, njev, it, None)
        J = jac(z)
        njev += 1
        sparse = not isinstance(J, np.ndarray)
        A = (J.T @ J).tocsc() if sparse else J.T @ J
        g = J.T @ r
        if _absmax(g) < gtol:
            return z, Info(2, _MSG[2], nfev, njev, it, None)
        diag = np.asarray(A.diagonal()).ravel()
        dmax = float(diag.max(initial=0.0))
        D = np.maximum(diag, 1e-8 * dmax if dmax > 0 else 1e-8)
        if lam < 0:
            lam = tau * (dmax if dmax > 0 else 1.0)
        f = 0.5 * float(r @ r)
        while True:
            if sparse:
                p = spla.spsolve(A + sp.diags(lam * D), -g)
            else:
                p = np.linalg.solve(A + np.diag(lam * D), -g)
            pnorm = _norm(p)
            if pnorm < xtol * (1 + _norm(z)):
                return z, Info(1, _MSG[1], nfev, njev, it, None)
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
                return z, Info(4 if nfev >= max_nfev else 3, _MSG[4 if nfev >= max_nfev else 3], nfev, njev, it + 1, None)
        if nfev >= max_nfev:
            break
    return z, Info(4, _MSG[4], nfev, njev, max_iter, None)
