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

import math
import time
from dataclasses import dataclass
from typing import Any, Literal, NamedTuple, get_args

import numpy as np
import numpy.typing as npt
import scipy.sparse as sp

from gcs import newton
from gcs.constraints import Constraint, DragTarget
from gcs.kernels import KERNEL_ID, KERNELS, Kernel
from gcs.model import Point, Sketch, Vec
from gcs.newton import SOLVERS

Method = Literal["dogleg", "lm", "scipy-dogbox", "scipy-trf", "scipy-lm"]
METHODS: tuple[Method, ...] = get_args(Method)
assert set(METHODS) == set(SOLVERS)
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
    jac_const: Vec | None          # ravelled Jacobian entries if the kernel's J is constant


def _consts_of(k: Kernel, cs: list[Constraint]) -> Vec:
    if not k.n_const:
        return np.zeros((len(cs), 0))
    return np.array([c.consts() for c in cs], dtype=np.float64).reshape(len(cs), k.n_const)


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
        self.scale = max(1.0, self.extent) ** 2     # residual units for squared distances

        # -- group constraints by kernel (kernel id order, then sketch order → deterministic) --
        by_kernel: dict[int, list[Constraint]] = {}
        for c in self.constraints:
            by_kernel.setdefault(KERNEL_ID[c.kernel.name], []).append(c)
        self.blocks: list[Block] = []
        self._slot_of: dict[int, tuple[int, int]] = {}   # id(constraint) -> (block index, row in block)
        hard: list[bool] = []
        rows_l: list[npt.NDArray[np.intp]] = []
        cols_l: list[npt.NDArray[np.intp]] = []
        row0 = 0
        for kid in sorted(by_kernel):
            cs = by_kernel[kid]
            k = KERNELS[kid]
            nb = len(cs)
            gidx = np.array([[p.index for p in c.params] for c in cs], dtype=np.intp).reshape(nb, k.n_par)
            consts = _consts_of(k, cs)
            jc = None if k.const_jac is None else np.broadcast_to(k.const_jac, (nb,) + k.const_jac.shape).ravel()
            self.blocks.append(Block(k, cs, gidx, consts, row0, jc))
            for i, c in enumerate(cs):
                self._slot_of[id(c)] = (len(self.blocks) - 1, i)
                hard.extend([not c.soft] * k.n_res)
            # Jacobian entry (i, j, col) of this block → row row0 + i*n_res + j, col col_of[gidx[i, col]]
            r = row0 + (np.arange(nb) * k.n_res)[:, None, None] + np.arange(k.n_res)[None, :, None]
            rows_l.append(np.broadcast_to(r, (nb, k.n_res, k.n_par)).ravel())
            cols_l.append(np.broadcast_to(self.col_of[gidx][:, None, :], (nb, k.n_res, k.n_par)).ravel())
            row0 += nb * k.n_res
        self.n_res = row0
        self.hard = np.array(hard, dtype=bool)
        rows = np.concatenate(rows_l) if rows_l else np.zeros(0, dtype=np.intp)
        cols = np.concatenate(cols_l) if cols_l else np.zeros(0, dtype=np.intp)
        self._keep = np.flatnonzero(cols >= 0)     # entries whose column is a free param
        rows, cols = rows[self._keep], cols[self._keep]
        # CSR structure with duplicates (shared params) merged: entry e → slot _slot[e]
        ncols = max(self.n_free, 1)
        uniq, self._slot = np.unique(rows * ncols + cols, return_inverse=True)
        self._nnz = len(uniq)
        self._csr_flat = uniq                       # dense scatter index into J.ravel()
        self._csr_indices = (uniq % ncols).astype(np.int32)
        self._csr_indptr = np.searchsorted(uniq // ncols, np.arange(self.n_res + 1)).astype(np.int32)
        self._x = sketch.get_x()

    def update_consts(self, c: Constraint) -> None:
        """Push a constraint's (mutated) constants into the compiled plan — e.g. a moving
        drag target or an edited dimension.  Topology is unchanged, so no recompile."""
        b, i = self._slot_of[id(c)]
        self.blocks[b].consts[i] = c.consts()

    def refresh_consts(self) -> None:
        """Re-read every constraint's constants (after arbitrary dimension edits)."""
        for k, cs, _, consts, _, _ in self.blocks:
            if k.n_const:
                consts[:] = _consts_of(k, cs)

    def max_hard_residual(self, z: Vec | None = None) -> float:
        """max |r| over hard rows at z (default: current sketch values) — what "solved" means."""
        r = self.residuals(self.z0() if z is None else z)
        rh = r[self.hard]
        return float(np.max(np.abs(rh))) if rh.size else 0.0

    def row_of(self, c: Constraint) -> int:
        """First residual row of a constraint."""
        b, i = self._slot_of[id(c)]
        return self.blocks[b].row0 + i * self.blocks[b].kernel.n_res

    def structure(self, hard_only: bool = True) -> tuple[list[list[int]], list[Constraint]]:
        """Structural Jacobian as a bipartite graph: adj[row] = sorted free columns with a
        (structural) nonzero, plus row → owning constraint.  The public surface for
        diagnosis/decomposition — derived from the compiled blocks, so it stays in step
        with what the solver actually evaluates."""
        adj: list[list[int]] = []
        row_c: list[Constraint] = []
        for k, cs, gidx, _, _, _ in self.blocks:
            cols_all = self.col_of[gidx]
            for i, c in enumerate(cs):
                if hard_only and c.soft:
                    continue
                cols = sorted({int(j) for j in cols_all[i] if j >= 0})
                for _ in range(k.n_res):
                    adj.append(cols)
                    row_c.append(c)
        return adj, row_c

    def constraint_errors(self, z: Vec | None = None) -> dict[int, float]:
        """max |residual| per constraint (keyed by id) from one vectorized evaluation."""
        r = np.abs(self.residuals(self.z0() if z is None else z))
        out: dict[int, float] = {}
        for k, cs, _, _, row0, _ in self.blocks:
            m = r[row0 : row0 + len(cs) * k.n_res].reshape(len(cs), k.n_res).max(axis=1)
            for c, e in zip(cs, m.tolist(), strict=True):
                out[id(c)] = e
        return out

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
        for k, _, gidx, consts, row0, _ in self.blocks:
            r[row0 : row0 + gidx.shape[0] * k.n_res] = k.res(x[gidx], consts).ravel()
        return r

    def _jac_data(self, z: Vec) -> Vec:
        """Jacobian entries in block/row/col order, fixed-param columns dropped."""
        x = self.full_x(z)
        parts = [jc if jc is not None else k.jac(x[gidx], consts).ravel()
                 for k, _, gidx, consts, _, jc in self.blocks]
        return np.take(np.concatenate(parts), self._keep) if parts else np.zeros(0)

    def _csr_data(self, z: Vec) -> Vec:
        return np.bincount(self._slot, weights=self._jac_data(z), minlength=self._nnz)

    def jacobian(self, z: Vec) -> sp.csr_matrix:
        return sp.csr_matrix((self._csr_data(z), self._csr_indices, self._csr_indptr), shape=(self.n_res, self.n_free))

    def jacobian_dense(self, z: Vec) -> Vec:
        J = np.zeros(self.n_res * max(self.n_free, 1))
        J[self._csr_flat] = self._csr_data(z)
        return J.reshape(self.n_res, max(self.n_free, 1))[:, : self.n_free]

    def rank(self, z: Vec | None = None, rcond: float = 1e-10, hard_only: bool = False) -> int:
        """Numerical rank of the Jacobian at z (default: current sketch values) via
        rank-revealing QR — the workhorse of Stage 2/4 diagnosis."""
        J = self.jacobian_dense(self.z0() if z is None else z)
        return newton.rank_rrqr(J[self.hard] if hard_only else J, rcond)

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
        z = self.z0()
        scale = self.scale
        if self.n_free == 0 or self.n_res == 0:
            info = newton.Info(0, 1, 0, 0, None, self.residuals(z))
        else:
            if dense is None:
                dense = self.n_free <= DENSE_MAX
            z, info = SOLVERS[method](self.residuals, self.jacobian_dense if dense else self.jacobian, z,
                                      ftol=tol * scale, xtol=1e-12, gtol=1e-16 * scale,
                                      max_iter=max_iter, max_nfev=max_nfev)
        if writeback:
            self.sketch.set_x(self.full_x(z))
        r = info.r if info.r is not None else self.residuals(z)
        rh = r[self.hard]
        mx = float(np.max(np.abs(rh))) if rh.size else 0.0
        ok = info.status >= 0 and mx < 1e-6 * scale
        return SolveResult(ok, info.status, info.message, float(np.linalg.norm(r)), mx, info.nfev, info.njev,
                           time.perf_counter() - t0, method, info.iterations, info.rank)


def solve(sketch: Sketch, method: Method = "dogleg", **kw: object) -> SolveResult:
    """One-shot: compile and solve, writing results back into the sketch."""
    return System(sketch).solve(method=method, **kw)  # type: ignore[arg-type]


Triangle = tuple[Point, Point, Point]


def orientation(a: Point, b: Point, c: Point) -> float:
    """Twice the signed area of (a, b, c) — the order-type invariant we guard."""
    return (b.x.value - a.x.value) * (c.y.value - a.y.value) - (b.y.value - a.y.value) * (c.x.value - a.x.value)


class Drag:
    """Interactive drag of one point: pull toward the cursor with a soft target,
    then polish with the hard constraints only so they hold exactly.

    Both systems are compiled once at drag start and reused for every move —
    dragging never re-analyses the sketch.  Stage 5 robustness:
      * continuation — a far cursor jump is taken in increments (≤ max_step_rel
        of the sketch extent) so the solution tracks its homotopy branch instead
        of teleporting across it;
      * order-type guards — the orientations of `guards` triangles (typically
        the plan's closed-form triples) are watched; a step that would flip one
        is retried with smaller increments, and if a flip is unavoidable it is
        recorded in `flips` and flagged in the result's message.
    """

    PULL_ITER = 4      # the pull is a soft compromise; a few GN steps suffice, polish makes it exact
    POLISH_ITER = 20

    def __init__(self, sketch: Sketch, point: Point, x: float, y: float,
                 method: Method = "dogleg", weight: float = 1.0,
                 guards: list[Triangle] | None = None, max_step_rel: float = 0.05) -> None:
        self.sketch, self.point, self.method = sketch, point, method
        self.polish = System(sketch)
        self.target = DragTarget(point, x, y, weight=weight)
        sketch.add(self.target)
        self.pull = System(sketch)
        self.active = True
        self.guards = guards or []
        self.max_step = max_step_rel * max(1.0, sketch.extent())
        self.signs = [orientation(*t) >= 0 for t in self.guards]
        self.flips: list[Triangle] = []

    def _step(self, x: float, y: float) -> SolveResult:
        self.target.set_target(x, y)
        self.pull.update_consts(self.target)
        self.pull.solve(method=self.method, max_iter=self.PULL_ITER)
        return self.polish.solve(method=self.method, max_iter=self.POLISH_ITER)

    def _flipped(self) -> list[int]:
        return [i for i, t in enumerate(self.guards) if (orientation(*t) >= 0) != self.signs[i]]

    def move(self, x: float, y: float) -> SolveResult:
        t0 = time.perf_counter()
        px, py = self.point.xy
        n = max(1, int(math.ceil(math.hypot(x - px, y - py) / self.max_step)))
        res = None
        flipped_now: list[int] = []
        for i in range(1, n + 1):
            tx, ty = px + (x - px) * i / n, py + (y - py) * i / n
            x_before = self.sketch.get_x()
            res = self._step(tx, ty)
            if self.guards:
                bad = self._flipped()
                halvings = 0
                while bad and halvings < 4:          # damp: retry from the last good state in smaller steps
                    self.sketch.set_x(x_before)
                    halvings += 1
                    m = 2 ** halvings
                    bx, by = self.point.xy
                    for j in range(1, m + 1):
                        res = self._step(bx + (tx - bx) * j / m, by + (ty - by) * j / m)
                    bad = self._flipped()
                if bad:                              # unavoidable: accept, record, flag
                    for k in bad:
                        self.signs[k] = not self.signs[k]
                        self.flips.append(self.guards[k])
                    flipped_now += bad
        assert res is not None
        res.time_s = time.perf_counter() - t0
        if flipped_now:
            res.message = f"order-type flip in {len(flipped_now)} triangle(s)"
        return res

    def end(self) -> None:
        if self.active:
            self.sketch.remove(self.target)
            self.active = False
