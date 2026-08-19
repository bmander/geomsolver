"""Assemble the sketch's residual vector and sparse Jacobian; solve with scipy.

`System` compiles a Sketch into a flat evaluation plan (constraint, local
column indices, row offset, free-column mask) once, then evaluates r(z) and
J(z) over the free parameters z repeatedly.  That compile-once / evaluate-many
seam is the boundary that becomes the C core in Stage 1.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Literal, NamedTuple, get_args

import numpy as np
import numpy.typing as npt
import scipy.sparse as sp
from scipy.optimize import least_squares as _least_squares

from gcs.constraints import Constraint, DragTarget
from gcs.model import Point, Sketch, Vec

Method = Literal["trf", "dogbox", "lm"]
METHODS: tuple[Method, ...] = get_args(Method)
least_squares: Any = _least_squares  # scipy-stubs overloads are too narrow for our kwargs dict


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

    def __repr__(self) -> str:
        return (
            f"SolveResult(ok={self.success}, |r|={self.residual_norm:.3e}, "
            f"max={self.max_residual:.3e}, nfev={self.nfev}, njev={self.njev}, "
            f"{self.time_s * 1e3:.2f} ms, {self.method})"
        )


class _Step(NamedTuple):
    off: int                          # first residual row
    a: int                            # slice [a:b] of the gathered local-values vector
    b: int
    keep: npt.NDArray[np.bool_] | None  # local columns that are free (None = all)


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
        # global param index -> free column, or -1 if fixed
        self.col_of = np.full(n, -1, dtype=np.intp)
        self.col_of[self.free] = np.arange(self.n_free)
        self.extent = sketch.extent()

        # Plain Python lists here: per-constraint arrays are ~4 elements, where numpy
        # call overhead dominates (this runs on every drag start).
        col_of = self.col_of.tolist()
        self.plan: list[_Step] = []
        gidx_all: list[int] = []      # every constraint's param indices, concatenated
        rows: list[int] = []
        cols: list[int] = []
        hard: list[bool] = []
        off = 0
        for c in self.constraints:
            gidx = [p.index for p in c.params]
            lc = [col_of[i] for i in gidx]
            kept = [j for j in lc if j >= 0]
            keep = None if len(kept) == len(lc) else np.array([j >= 0 for j in lc], dtype=bool)
            self.plan.append(_Step(off, len(gidx_all), len(gidx_all) + len(gidx), keep))
            gidx_all.extend(gidx)
            for i in range(c.n_residuals):
                rows.extend([off + i] * len(kept))
                cols.extend(kept)
            hard.extend([not c.soft] * c.n_residuals)
            off += c.n_residuals
        self.n_res = off
        self.gidx_all = np.array(gidx_all, dtype=np.intp)
        self.hard = np.array(hard, dtype=bool)
        self.jac_rows = np.array(rows, dtype=np.intp)
        self.jac_cols = np.array(cols, dtype=np.intp)
        self._flat = self.jac_rows * max(self.n_free, 1) + self.jac_cols  # for dense bincount assembly
        self._x = sketch.get_x()

    # -- evaluation ---------------------------------------------------------

    def full_x(self, z: Vec) -> Vec:
        x = self._x.copy()
        x[self.free] = z
        return x

    def z0(self) -> Vec:
        self._x = self.sketch.get_x()
        return self._x[self.free].copy()

    def _gather(self, z: Vec) -> Vec:
        """Local value vectors of all constraints, concatenated (one fancy-index, then slices)."""
        return self.full_x(z)[self.gidx_all]

    def residuals(self, z: Vec) -> Vec:
        xv = self._gather(z)
        r = np.empty(self.n_res)
        for c, (off, a, b, _) in zip(self.constraints, self.plan, strict=True):
            r[off : off + c.n_residuals] = c.residual(xv[a:b])
        return r

    def _jac_data(self, z: Vec) -> Vec:
        """Non-zero Jacobian entries in (jac_rows, jac_cols) order."""
        xv = self._gather(z)
        parts: list[Vec] = []
        for c, (_, a, b, keep) in zip(self.constraints, self.plan, strict=True):
            J = c.jacobian(xv[a:b])
            parts.append((J if keep is None else J[:, keep]).ravel())
        return np.concatenate(parts) if parts else np.zeros(0)

    def jacobian(self, z: Vec) -> sp.csr_matrix:
        m = sp.coo_matrix((self._jac_data(z), (self.jac_rows, self.jac_cols)), shape=(self.n_res, self.n_free))
        return m.tocsr()  # duplicates (shared params) are summed

    def jacobian_dense(self, z: Vec) -> Vec:
        # bincount sums duplicate (row, col) entries — same semantics as the sparse path, no scipy objects
        out = np.bincount(self._flat, weights=self._jac_data(z), minlength=self.n_res * max(self.n_free, 1))
        return out.reshape(self.n_res, max(self.n_free, 1))[:, : self.n_free]

    # -- solving ------------------------------------------------------------

    def solve(
        self,
        method: Method = "dogbox",
        tol: float = 1e-12,
        max_nfev: int | None = None,
        writeback: bool = True,
        jac: Literal["analytic", "fd"] = "analytic",
    ) -> SolveResult:
        t0 = time.perf_counter()
        z0 = self.z0()
        if self.n_free == 0 or self.n_res == 0:
            r = self.residuals(z0)
            return SolveResult(True, 0, "nothing to solve", float(np.linalg.norm(r)),
                               float(np.max(np.abs(r))) if r.size else 0.0, 1, 0,
                               time.perf_counter() - t0, method)

        kw: dict[str, object] = dict(ftol=tol, xtol=tol, gtol=tol, max_nfev=max_nfev, x_scale="jac")
        if method == "lm":
            # scipy's MINPACK wrapper wants dense J and m >= n; pad with zero rows if needed.
            pad = max(0, self.n_free - self.n_res)

            def jac_dense(z: Vec) -> Vec:
                J = self.jacobian_dense(z)
                return np.vstack([J, np.zeros((pad, self.n_free))]) if pad else J

            def res_pad(z: Vec) -> Vec:
                r = self.residuals(z)
                return np.concatenate([r, np.zeros(pad)]) if pad else r

            res = least_squares(res_pad, z0, jac=jac_dense if jac == "analytic" else "2-point",
                                method="lm", **kw)
            r = res.fun[: self.n_res]
        else:
            # Small systems: dense J + exact (SVD) trust-region subproblem — gives the
            # minimum-norm Gauss-Newton step, i.e. least-change behaviour when
            # under-constrained.  Large systems: sparse J + LSMR.
            sparse = self.n_free > 200
            jfun = (self.jacobian if sparse else self.jacobian_dense) if jac == "analytic" else "2-point"
            res = least_squares(self.residuals, z0, jac=jfun, method=method,
                                tr_solver="lsmr" if sparse else "exact", **kw)
            r = res.fun
        if writeback:
            self.sketch.set_x(self.full_x(res.x))
        rh = r[self.hard]
        mx = float(np.max(np.abs(rh))) if rh.size else 0.0
        ok = bool(res.status >= 0) and mx < 1e-6 * self.extent**2
        return SolveResult(ok, int(res.status), str(res.message), float(np.linalg.norm(r)), mx,
                           int(res.nfev), int(res.njev or 0), time.perf_counter() - t0, method)


def solve(sketch: Sketch, method: Method = "dogbox", **kw: object) -> SolveResult:
    """One-shot: compile and solve, writing results back into the sketch."""
    return System(sketch).solve(method=method, **kw)  # type: ignore[arg-type]


class Drag:
    """Interactive drag of one point: pull toward the cursor with a soft target,
    then polish with the hard constraints only so they hold exactly.

    Both systems are compiled once at drag start and reused for every move —
    dragging never re-analyses the sketch.  Frontends only translate cursor
    coordinates; this is the home for Stage-5 continuation/root-tracking later.
    """

    PULL_NFEV = 50
    POLISH_NFEV = 20

    def __init__(self, sketch: Sketch, point: Point, x: float, y: float,
                 method: Method = "dogbox", weight: float = 1.0) -> None:
        self.sketch, self.point, self.method = sketch, point, method
        self.polish = System(sketch)
        self.target = DragTarget(point, x, y, weight=weight)
        sketch.add(self.target)
        self.pull = System(sketch)
        self.active = True

    def move(self, x: float, y: float) -> SolveResult:
        t0 = time.perf_counter()
        self.target.set_target(x, y)
        self.pull.solve(method=self.method, max_nfev=self.PULL_NFEV)
        res = self.polish.solve(method=self.method, max_nfev=self.POLISH_NFEV)
        res.time_s = time.perf_counter() - t0
        return res

    def end(self) -> None:
        if self.active:
            self.sketch.remove(self.target)
            self.active = False
