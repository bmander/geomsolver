"""Compile a sketch to a flat evaluation plan; evaluate r(z), J(z); solve.

`System` groups the sketch's constraints by kernel type into *blocks* — pure
arrays: (kernel id, global param indices (n, k), constants (n, m), row
offset).  Evaluating the whole sketch is one kernel call per block, and the
Jacobian's sparsity structure (CSR indices, duplicate-summing scatter map) is
computed once at compile time; each evaluation only refills `data`.  This
plan is exactly what a C core would consume — the compile-to-plan boundary
from the program's Stage 1.

Solvers: our own DogLeg (default) and LM (gcs.newton); scipy's least_squares
methods are kept as `scipy-*` references for benchmarking.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Literal, NamedTuple, get_args

import numpy as np
import numpy.typing as npt
import scipy.sparse as sp

from gcs import newton
from gcs.constraints import Constraint, DragTarget
from gcs.kernels import KERNELS, Kernel
from gcs.model import Point, Sketch, Vec

Method = Literal["dogleg", "lm", "scipy-dogbox", "scipy-trf", "scipy-lm"]
METHODS: tuple[Method, ...] = get_args(Method)
DENSE_MAX = 120   # free params up to which J is dense (LAPACK dgelsy: exact min-norm step + rank); sparse SuperLU above


@dataclass
class SolveResult:
    success: bool
    status: int
    message: str
    residual_norm: float      # over all residuals, soft ones included
    max_residual: float       # over hard residuals only (what "solved" means)
    nfev: int
    njev: int
    time_s: float
    method: str
    iterations: int = 0
    rank: int | None = None   # numerical rank of J at the solution (dense path)

    def __repr__(self) -> str:
        return (
            f"SolveResult(ok={self.success}, |r|={self.residual_norm:.3e}, "
            f"max={self.max_residual:.3e}, it={self.iterations}, nfev={self.nfev}, njev={self.njev}, "
            f"{self.time_s * 1e3:.2f} ms, {self.method})"
        )


class Block(NamedTuple):
    kernel: Kernel
    constraints: list[Constraint]
    gidx: npt.NDArray[np.intp]     # (n, k) global param index per local column
    consts: Vec                    # (n, m)
    row0: int                      # residual rows row0 .. row0 + n*n_res


class System:
    """Compiled evaluation plan for one sketch topology."""

    def __init__(self, sketch: Sketch) -> None:
        self.sketch = sketch
        self.constraints: list[Constraint] = list(sketch.constraints)  # snapshot: the plan is fixed at compile time
        n = len(sketch.params)
        for i, p in enumerate(sketch.params):
            p.index = i  # keep indices honest even if the list was edited
        self.free = sketch.free_indices()
        self.n_free = len(self.free)
        self.col_of = np.full(n, -1, dtype=np.intp)   # global param index -> free column, or -1
        self.col_of[self.free] = np.arange(self.n_free)
        self.extent = sketch.extent()

        # -- group constraints by kernel (kernel order, then sketch order → deterministic) --
        by_kernel: dict[int, list[Constraint]] = {}
        for c in self.constraints:
            by_kernel.setdefault(KERNELS.index(c.kernel), []).append(c)
        self.blocks: list[Block] = []
        self.row_of: dict[int, int] = {}          # id(constraint) -> first residual row
        self._slot_of: dict[int, tuple[int, int]] = {}   # id(constraint) -> (block index, row in block)
        hard: list[bool] = []
        rows_l: list[npt.NDArray[np.intp]] = []
        cols_l: list[npt.NDArray[np.intp]] = []
        row0 = 0
        for kid in sorted(by_kernel):
            cs = by_kernel[kid]
            k = KERNELS[kid]
            gidx = np.array([[p.index for p in c.params] for c in cs], dtype=np.intp).reshape(len(cs), k.n_par)
            consts = (np.array([c.consts() for c in cs], dtype=np.float64).reshape(len(cs), k.n_const)
                      if k.n_const else np.zeros((len(cs), 0)))
            self.blocks.append(Block(k, cs, gidx, consts, row0))
            for i, c in enumerate(cs):
                self.row_of[id(c)] = row0 + i * k.n_res
                self._slot_of[id(c)] = (len(self.blocks) - 1, i)
                hard.extend([not c.soft] * k.n_res)
            # Jacobian entry (i, j, col) of this block → row row0 + i*n_res + j, col col_of[gidx[i, col]]
            nb = len(cs)
            r = row0 + (np.arange(nb) * k.n_res)[:, None, None] + np.arange(k.n_res)[None, :, None]
            rows_l.append(np.broadcast_to(r, (nb, k.n_res, k.n_par)).ravel())
            cols_l.append(np.broadcast_to(self.col_of[gidx][:, None, :], (nb, k.n_res, k.n_par)).ravel())
            row0 += nb * k.n_res
        self.n_res = row0
        self.hard = np.array(hard, dtype=bool)
        rows = np.concatenate(rows_l) if rows_l else np.zeros(0, dtype=np.intp)
        cols = np.concatenate(cols_l) if cols_l else np.zeros(0, dtype=np.intp)
        self._keep = cols >= 0                     # drop columns of fixed params
        if self._keep.all():
            self._keep = None  # type: ignore[assignment]
        rows, cols = rows[cols >= 0], cols[cols >= 0]
        self.jac_rows, self.jac_cols = rows, cols
        # CSR structure with duplicates (shared params) merged: entry e → slot inv[e]
        ncols = max(self.n_free, 1)
        key = rows * ncols + cols
        uniq, self._slot = np.unique(key, return_inverse=True)
        self._csr_indices = (uniq % ncols).astype(np.int32)
        self._csr_indptr = np.searchsorted(uniq // ncols, np.arange(self.n_res + 1)).astype(np.int32)
        self._csr_rows = (uniq // ncols).astype(np.intp)   # row of each CSR slot (dense scatter)
        self._nnz = len(uniq)
        # Blocks with constant Jacobians (linear constraints) are evaluated once, here.
        self._jac_const: list[Vec | None] = [
            None if b.kernel.const_jac is None else np.broadcast_to(b.kernel.const_jac, (b.gidx.shape[0],) + b.kernel.const_jac.shape).ravel()
            for b in self.blocks
        ]
        self._x = sketch.get_x()

    def update_consts(self, c: Constraint) -> None:
        """Push a constraint's (mutated) constants into the compiled plan — e.g. a moving
        drag target or an edited dimension.  Topology is unchanged, so no recompile."""
        b, i = self._slot_of[id(c)]
        self.blocks[b].consts[i] = c.consts()

    # -- evaluation ---------------------------------------------------------

    def full_x(self, z: Vec) -> Vec:
        x = self._x.copy()
        x[self.free] = z
        return x

    def z0(self) -> Vec:
        self._x = self.sketch.get_x()
        return self._x[self.free].copy()

    def residuals(self, z: Vec) -> Vec:
        x = self.full_x(z)
        r = np.empty(self.n_res)
        for k, _, gidx, consts, row0 in self.blocks:
            r[row0 : row0 + gidx.shape[0] * k.n_res] = k.res(x[gidx], consts).ravel()
        return r

    def _jac_data(self, z: Vec) -> Vec:
        """Jacobian entries in (jac_rows, jac_cols) order (fixed-param columns dropped)."""
        x = self.full_x(z)
        parts = [cj if cj is not None else k.jac(x[gidx], consts).ravel()
                 for (k, _, gidx, consts, _), cj in zip(self.blocks, self._jac_const, strict=True)]
        data = np.concatenate(parts) if parts else np.zeros(0)
        return data if self._keep is None else data[self._keep]

    def _csr_data(self, z: Vec) -> Vec:
        return np.bincount(self._slot, weights=self._jac_data(z), minlength=self._nnz)

    def jacobian(self, z: Vec) -> sp.csr_matrix:
        return sp.csr_matrix((self._csr_data(z), self._csr_indices, self._csr_indptr), shape=(self.n_res, self.n_free))

    def jacobian_dense(self, z: Vec) -> Vec:
        J = np.zeros((self.n_res, max(self.n_free, 1)))
        J[self._csr_rows, self._csr_indices] = self._csr_data(z)
        return J[:, : self.n_free]

    def rank(self, z: Vec | None = None, rcond: float = 1e-10) -> int:
        """Numerical rank of the Jacobian at z (default: current sketch values) via
        rank-revealing QR — the workhorse of Stage 2/4 diagnosis."""
        z = self.z0() if z is None else z
        if self.n_free == 0 or self.n_res == 0:
            return 0
        return newton.min_norm_lstsq(self.jacobian_dense(z), np.zeros(self.n_res), rcond)[1]

    # -- solving ------------------------------------------------------------

    def solve(
        self,
        method: Method = "dogleg",
        tol: float = 1e-14,          # relative to extent² (residual units for squared distances)
        max_nfev: int | None = None,
        writeback: bool = True,
        max_iter: int = 100,
        dense: bool | None = None,   # force the dense/sparse Jacobian path (None = by DENSE_MAX)
    ) -> SolveResult:
        t0 = time.perf_counter()
        z0 = self.z0()
        if self.n_free == 0 or self.n_res == 0:
            r = self.residuals(z0)
            return SolveResult(True, 0, "nothing to solve", float(np.linalg.norm(r)),
                               float(np.max(np.abs(r))) if r.size else 0.0, 1, 0,
                               time.perf_counter() - t0, method)
        scale = max(1.0, self.extent) ** 2
        if dense is None:
            dense = self.n_free <= DENSE_MAX
        jfun = self.jacobian_dense if dense else self.jacobian
        rank: int | None = None
        if method in ("dogleg", "lm"):
            algo = newton.dogleg if method == "dogleg" else newton.levenberg_marquardt
            z, info = algo(self.residuals, jfun, z0, ftol=tol * scale, xtol=1e-12, gtol=1e-16 * scale,
                           max_iter=max_iter, max_nfev=max_nfev)
            status, message, nfev, njev, iters, rank = info.status, info.message, info.nfev, info.njev, info.iterations, info.rank
            r = self.residuals(z)
        else:
            z, status, message, nfev, njev, iters, r = self._solve_scipy(method, z0, tol, max_nfev, dense)
        if writeback:
            self.sketch.set_x(self.full_x(z))
        rh = r[self.hard]
        mx = float(np.max(np.abs(rh))) if rh.size else 0.0
        ok = status >= 0 and mx < 1e-6 * scale
        return SolveResult(ok, status, message, float(np.linalg.norm(r)), mx, nfev, njev,
                           time.perf_counter() - t0, method, iters, rank)

    def _solve_scipy(self, method: str, z0: Vec, tol: float, max_nfev: int | None, dense: bool) -> tuple[Any, ...]:
        from scipy.optimize import least_squares as _ls

        least_squares: Any = _ls  # scipy-stubs overloads are too narrow for our kwargs
        kw: dict[str, Any] = dict(ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=max_nfev, x_scale="jac")
        m = method.removeprefix("scipy-")
        if m == "lm":
            pad = max(0, self.n_free - self.n_res)  # MINPACK needs m >= n and a dense J
            res = least_squares(lambda z: np.concatenate([self.residuals(z), np.zeros(pad)]), z0,
                                jac=lambda z: np.vstack([self.jacobian_dense(z), np.zeros((pad, self.n_free))]),
                                method="lm", **kw)
            r = res.fun[: self.n_res]
        else:
            res = least_squares(self.residuals, z0, jac=self.jacobian_dense if dense else self.jacobian,
                                method=m, tr_solver="exact" if dense else "lsmr", **kw)
            r = res.fun
        return res.x, int(res.status), str(res.message), int(res.nfev), int(res.njev or 0), int(res.nfev), r


def solve(sketch: Sketch, method: Method = "dogleg", **kw: object) -> SolveResult:
    """One-shot: compile and solve, writing results back into the sketch."""
    return System(sketch).solve(method=method, **kw)  # type: ignore[arg-type]


class Drag:
    """Interactive drag of one point: pull toward the cursor with a soft target,
    then polish with the hard constraints only so they hold exactly.

    Both systems are compiled once at drag start and reused for every move —
    dragging never re-analyses the sketch.  Frontends only translate cursor
    coordinates; this is the home for Stage-5 continuation/root-tracking later.
    """

    PULL_ITER = 4      # the pull is a soft compromise; a few GN steps suffice, polish makes it exact
    POLISH_ITER = 20

    def __init__(self, sketch: Sketch, point: Point, x: float, y: float,
                 method: Method = "dogleg", weight: float = 1.0) -> None:
        self.sketch, self.point, self.method = sketch, point, method
        self.polish = System(sketch)
        self.target = DragTarget(point, x, y, weight=weight)
        sketch.add(self.target)
        self.pull = System(sketch)
        self.active = True

    def move(self, x: float, y: float) -> SolveResult:
        t0 = time.perf_counter()
        self.target.set_target(x, y)
        self.pull.update_consts(self.target)
        self.pull.solve(method=self.method, max_iter=self.PULL_ITER)
        res = self.polish.solve(method=self.method, max_iter=self.POLISH_ITER)
        res.time_s = time.perf_counter() - t0
        return res

    def end(self) -> None:
        if self.active:
            self.sketch.remove(self.target)
            self.active = False
