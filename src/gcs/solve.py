"""The compiled system, solving, and interactive dragging.

`System` is the compile-once / evaluate-many seam: it owns a handle to the core's evaluation plan,
so the object model never enters the hot loop.  Call `dispose()` when you drop one (the drags, the
plan solver and diagnosis all do).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Literal, Sequence, get_args

import numpy as np
import numpy.typing as npt

from gcs import _ffi
from gcs._ffi import Vec, lib
from gcs.constraints import Constraint
from gcs.model import Arc, Circle, KIND_ID, Point, Sketch

Method = Literal["dogleg", "lm"]
METHODS: tuple[Method, ...] = get_args(Method)
_METHOD_ID = {"dogleg": 0, "lm": 1}
Triangle = tuple[Point, Point, Point]

#: Free params up to which J is dense (exact minimum-norm step + rank); sparse above.
DENSE_MAX = 120


@dataclass
class SolveResult:
    success: bool
    status: int
    message: str
    residual_norm: float       # over all residuals, soft ones included
    max_residual: float        # over hard residuals only (what "solved" means)
    nfev: int
    njev: int
    time_s: float
    method: str
    iterations: int = 0
    rank: int | None = None    # numerical rank of J at the solution (dense path)

    def __repr__(self) -> str:
        return (
            f"SolveResult(ok={self.success}, |r|={self.residual_norm:.3e}, "
            f"max={self.max_residual:.3e}, it={self.iterations}, nfev={self.nfev}, "
            f"njev={self.njev}, {self.time_s * 1e3:.2f} ms, {self.method})"
        )


def _result(out: Vec, message: str, method: str, t0: float) -> SolveResult:
    rank = int(out[7])
    return SolveResult(
        success=bool(out[0]), status=int(out[1]), message=message,
        residual_norm=float(out[2]), max_residual=float(out[3]),
        nfev=int(out[4]), njev=int(out[5]), time_s=time.perf_counter() - t0,
        method=method, iterations=int(out[6]), rank=None if rank < 0 else rank,
    )


#: The tolerance a rank is judged at: the core's `RANK_TOL`, absolute and dimensionless.
RANK_TOL: float = float(lib.gcs_rank_tol())


class System:
    """Compiled evaluation plan for one sketch topology."""

    def __init__(self, sketch: Sketch, handle: Any = None, owner: Any = None) -> None:
        self.sketch = sketch
        self._handle = handle if handle is not None else lib.gcs_system_new(sketch._h)
        #: Whoever owns a borrowed handle (a PlanSolver's inner System), held so it cannot be
        #: collected while we still point into its box.  None when the handle is ours.
        self._owner = owner
        self._disposed = False
        self.n_res = int(lib.gcs_system_n_res(self._h))
        self.n_free = int(lib.gcs_system_n_free(self._h))
        self.nnz = int(lib.gcs_system_nnz(self._h))
        #: constraints the plan was compiled from — not the live sketch's count once edited
        self.n_constraints = int(lib.gcs_system_n_constraints(self._h))
        self.scale = float(lib.gcs_system_scale(self._h))
        self.extent = sketch.extent()
        hard = np.zeros(max(self.n_res, 1), dtype=np.uint8)
        lib.gcs_system_hard(self._h, _ffi.pu8(hard))
        self.hard = hard[: self.n_res].astype(bool)
        free = _ffi.i32(max(self.n_free, 1))
        lib.gcs_system_free_indices(self._h, _ffi.pi(free))
        self.free = free[: self.n_free].astype(np.intp)

    @property
    def _h(self) -> Any:
        """The core handle.  Every entry point goes through here so a use-after-dispose raises
        instead of calling into freed heap."""
        if self._disposed:
            raise RuntimeError("System used after dispose()")
        return self._handle

    def dispose(self) -> None:
        if self._disposed:
            return
        self._disposed = True
        if self._owner is None and self._handle:
            lib.gcs_system_free(self._handle)
        self._handle = None
        self._owner = None

    def __del__(self) -> None:  # pragma: no cover - interpreter shutdown ordering
        try:
            self.dispose()
        except Exception:
            pass

    # -- constants ----------------------------------------------------------

    def update_consts(self, c: Constraint) -> None:
        """Push a constraint's (mutated) constants into the compiled plan.  Topology is
        unchanged, so no recompile."""
        lib.gcs_system_update_consts(self._h, self.sketch._h, c._id)

    def refresh_consts(self) -> None:
        lib.gcs_system_refresh_consts(self._h, self.sketch._h)

    def row_of(self, c: Constraint) -> int:
        """First residual row of a constraint."""
        return int(lib.gcs_system_row_of(self._h, c._id))

    def constraint_params(self, c: Constraint) -> list[int]:
        return c.param_indices()

    # -- evaluation ---------------------------------------------------------

    def z0(self) -> Vec:
        out = _ffi.f64(max(self.n_free, 1))
        lib.gcs_system_z0(self._h, self.sketch._h, _ffi.pf(out))
        return out[: self.n_free]

    def residuals(self, z: Any) -> Vec:
        zz = _ffi.as_f64(z)
        out = _ffi.f64(max(self.n_res, 1))
        lib.gcs_system_residuals(self._h, _ffi.pf(zz), _ffi.pf(out))
        return out[: self.n_res]

    def jacobian_dense(self, z: Any) -> Vec:
        zz = _ffi.as_f64(z)
        out = _ffi.f64(max(self.n_res * self.n_free, 1))
        lib.gcs_system_jacobian_dense(self._h, _ffi.pf(zz), _ffi.pf(out))
        return out[: self.n_res * self.n_free].reshape(self.n_res, self.n_free)

    def csr(self, z: Any) -> tuple[Vec, npt.NDArray[np.int32], npt.NDArray[np.int32]]:
        """(data, indices, indptr) — the sparse Jacobian in CSR, structure fixed at compile time."""
        zz = _ffi.as_f64(z)
        indptr = _ffi.i32(self.n_res + 1)
        indices = _ffi.i32(max(self.nnz, 1))
        lib.gcs_system_csr_structure(self._h, _ffi.pi(indptr), _ffi.pi(indices))
        data = _ffi.f64(max(self.nnz, 1))
        lib.gcs_system_csr_data(self._h, _ffi.pf(zz), _ffi.pf(data))
        return data[: self.nnz], indices[: self.nnz], indptr

    def max_hard_residual(self) -> float:
        """max |r| over hard rows at the current sketch values, in the residuals' own units."""
        return float(lib.gcs_system_max_hard_residual(self._h, self.sketch._h))

    def max_relative_residual(self) -> float:
        """max |r| / that row's units over the hard rows — dimensionless, so one threshold judges
        every kernel.  This, not `max_hard_residual`, is what "solved" means."""
        return float(lib.gcs_system_max_relative_residual(self._h, self.sketch._h))

    def constraint_errors(self) -> dict[int, float]:
        """max |residual| per constraint, keyed by constraint id."""
        n = self.n_constraints
        ids = _ffi.i32(max(n, 1))
        vals = _ffi.f64(max(n, 1))
        m = lib.gcs_system_constraint_errors(self._h, self.sketch._h, _ffi.pi(ids),
                                             _ffi.pf(vals), n)
        return {int(ids[i]): float(vals[i]) for i in range(m)}

    def rank(self, tol: float = RANK_TOL, hard_only: bool = False) -> int:
        """Numerical rank of the Jacobian at the current sketch values.  `tol` is absolute and
        dimensionless: it is judged on `conditioned()`, not on `jacobian_dense`."""
        return int(lib.gcs_system_rank(self._h, self.sketch._h, tol, 1 if hard_only else 0))

    def conditioned(self) -> Vec:
        """The hard rows of the Jacobian at the current sketch values with their units divided
        out — the matrix every rank and null space in the core is judged on.  One row per row
        of `structure()`, in its order."""
        n_hard = int(self.hard.sum())
        out = _ffi.f64(max(n_hard * self.n_free, 1))
        m = lib.gcs_system_conditioned(self._h, self.sketch._h, _ffi.pf(out))
        return out[: m * self.n_free].reshape(m, self.n_free)

    def structure(self) -> tuple[list[list[int]], list[Constraint]]:
        """Structural Jacobian as a bipartite graph, plus row → owning constraint.  Soft rows
        (drag targets) are never part of it."""
        d = _ffi.take_json(lib.gcs_system_structure_json(self._h))
        self.sketch._sync_constraints()
        rows = [self.sketch._by_id[i] for i in d["rowC"]]
        return d["adj"], rows

    # -- solving ------------------------------------------------------------

    def solve(self, method: Method = "dogleg", tol: float = 1e-14, max_nfev: int | None = None,
              writeback: bool = True, max_iter: int = 100,
              dense: bool | None = None) -> SolveResult:
        t0 = time.perf_counter()
        out = _ffi.f64(8)
        msg = _ffi.take_str(lib.gcs_system_solve(
            self._h, self.sketch._h, _METHOD_ID[method], tol, max_iter, max_nfev or 0,
            -1 if dense is None else int(dense), 1 if writeback else 0, _ffi.pf(out)))
        return _result(out, msg, method, t0)


def solve(sketch: Sketch, method: Method = "dogleg", tol: float = 1e-14,
          max_iter: int = 100, max_nfev: int | None = None,
          dense: bool | None = None) -> SolveResult:
    """One-shot: compile and solve, writing results back into the sketch."""
    t0 = time.perf_counter()
    out = _ffi.f64(8)
    msg = _ffi.take_str(lib.gcs_solve(
        sketch._h, _METHOD_ID[method], tol, max_iter, max_nfev or 0,
        -1 if dense is None else int(dense), _ffi.pf(out)))
    return _result(out, msg, method, t0)


def orientation(a: Point, b: Point, c: Point) -> float:
    """Twice the signed area of (a, b, c) — the order-type invariant the drag guards."""
    return float(lib.gcs_orientation(a.sketch._h, a.index, b.index, c.index))


def _read_flips(sketch: Sketch, fn: Any, handle: Any, n: int) -> list[Triangle]:
    if n <= 0:
        return []
    buf = _ffi.i32(3 * n)
    fn(handle, _ffi.pi(buf))
    pts = sketch.points
    return [(pts[int(buf[3 * i])], pts[int(buf[3 * i + 1])], pts[int(buf[3 * i + 2])])
            for i in range(n)]


def _guard_buffer(guards: Sequence[Triangle] | None) -> tuple[Any, int]:
    if not guards:
        return None, 0
    flat = _ffi.i32(3 * len(guards))
    for i, t in enumerate(guards):
        flat[3 * i], flat[3 * i + 1], flat[3 * i + 2] = t[0].index, t[1].index, t[2].index
    return _ffi.pi(flat), len(guards)


class Drag:
    """Interactive drag of one point: pull toward the cursor with a soft target, then polish with
    the hard constraints only so they hold exactly.

    Stage 5 robustness: continuation (a far cursor jump is taken in increments so the solution
    tracks its homotopy branch) and order-type guards (a step that would flip a guarded triangle is
    retried with smaller increments; an unavoidable flip is recorded and flagged)."""

    def __init__(self, sketch: Sketch, point: Point, x: float, y: float,
                 method: Method = "dogleg", weight: float = 1.0,
                 guards: Sequence[Triangle] | None = None, max_step_rel: float = 0.05) -> None:
        self.sketch = sketch
        self.point = point
        self.method = method
        self.guards = list(guards or [])
        ptr, n = _guard_buffer(self.guards)
        self._h = lib.gcs_drag_new(sketch._h, point.index, float(x), float(y),
                                   _METHOD_ID[method], weight, ptr, n, max_step_rel)
        sketch.touch()
        self.active = True

    @property
    def flips(self) -> list[Triangle]:
        """The guarded triangles whose orientation the drag could not preserve."""
        return _read_flips(self.sketch, lib.gcs_drag_flip_list, self._h,
                           int(lib.gcs_drag_flips(self._h)))

    def move(self, x: float, y: float) -> SolveResult:
        t0 = time.perf_counter()
        out = _ffi.f64(8)
        msg = _ffi.take_str(lib.gcs_drag_move(self._h, self.sketch._h, float(x), float(y),
                                              _ffi.pf(out)))
        return _result(out, msg, self.method, t0)

    def end(self) -> None:
        if self.active:
            lib.gcs_drag_end(self._h, self.sketch._h)
            self.sketch.touch()
            self.active = False

    def __del__(self) -> None:  # pragma: no cover
        try:
            lib.gcs_drag_free(self._h)
        except Exception:
            pass


class RadiusDrag:
    """Interactive drag of a circle's or arc's radius — the scalar counterpart of `Drag`.

    A radius that is fixed or dimensioned simply does not move: the polish wins, exactly as a point
    drag compromises on an over-constrained sketch.  An `EqualRadius` chain is a relation rather
    than a dimension, so the whole chain resizes together."""

    def __init__(self, sketch: Sketch, circle: Circle | Arc, r: float,
                 method: Method = "dogleg") -> None:
        self.sketch = sketch
        self.circle = circle
        self.method = method
        self._h = lib.gcs_radius_drag_new(sketch._h, KIND_ID[circle.kind], circle.index,
                                          float(r), _METHOD_ID[method])
        sketch.touch()
        self.active = True

    def move(self, r: float) -> SolveResult:
        t0 = time.perf_counter()
        out = _ffi.f64(8)
        msg = _ffi.take_str(lib.gcs_radius_drag_move(self._h, self.sketch._h, float(r),
                                                     _ffi.pf(out)))
        return _result(out, msg, self.method, t0)

    def end(self) -> None:
        if self.active:
            lib.gcs_radius_drag_end(self._h, self.sketch._h)
            self.sketch.touch()
            self.active = False

    def __del__(self) -> None:  # pragma: no cover
        try:
            lib.gcs_radius_drag_free(self._h)
        except Exception:
            pass


__all__ = ["DENSE_MAX", "Drag", "METHODS", "Method", "RadiusDrag", "SolveResult", "System",
           "Triangle", "orientation", "solve"]
