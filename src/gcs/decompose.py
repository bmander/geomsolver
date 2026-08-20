"""Stage 3 — cluster merging (Fudos–Hoffmann, generalised) → plan → replay.

Decomposition and replay run in the core.  `PlanSolver` compiles once per topology and replays the
plan on every solve; `PlanDrag` is the DCM-style drag that never re-analyses the graph.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Sequence

from gcs import _ffi
from gcs._ffi import lib
from gcs.model import Point, Sketch
from gcs.solve import Method, SolveResult, System, Triangle, _METHOD_ID, _result, _guard_buffer


@dataclass
class Step:
    """One merge, lowered for replay.  `ppp` is the (x, y, z) closed-form construction and
    `branch` its ±1 chirality; `key` is the document-stable identity that persists."""

    ids: list[int]
    ppp: list[list[Any]] | None
    branch: int | None
    key: str | None
    n_pairs: int
    n_dpairs: int


@dataclass
class Plan:
    leaves: int
    steps: list[Step]
    roots: list[int]
    fully_decomposed: bool
    sticky_branches: bool
    summary: str
    ppp_triangles: list[list[int]]


def _plan_from(d: dict[str, Any]) -> Plan:
    return Plan(
        leaves=d["leaves"],
        steps=[Step(s["ids"], s["ppp"], s["branch"], s["key"], s["nPairs"], s["nDpairs"])
               for s in d["steps"]],
        roots=d["roots"], fully_decomposed=bool(d["fullyDecomposed"]),
        sticky_branches=bool(d["stickyBranches"]), summary=d["summary"],
        ppp_triangles=d["pppTriangles"],
    )


@dataclass
class PlanResult:
    success: bool
    max_residual: float
    fell_back: bool
    time_s: float
    n_steps: int
    numeric: dict[str, Any] | None

    def __repr__(self) -> str:
        return (f"PlanResult(ok={self.success}, max={self.max_residual:.3e}, "
                f"fell_back={self.fell_back}, {self.time_s * 1e3:.2f} ms)")


class PlanSolver:
    """Compile once per topology (graph + decomposition + a System for verification); `solve`
    replays the plan and falls back to the numeric core when the residual says the plan did not
    (fully) determine the sketch."""

    def __init__(self, sketch: Sketch, sticky: bool = False) -> None:
        self.sketch = sketch
        self._h = lib.gcs_plan_solver_new(sketch._h, 1 if sticky else 0)
        #: Owned by this PlanSolver — borrow it (diagnosis does), never dispose it.  It holds a
        #: reference back to us, so the box it points into outlives it.
        self.system = System(sketch, lib.gcs_plan_solver_system(self._h), owner=self)

    def dispose(self) -> None:
        if self._h:
            self.system.dispose()
            lib.gcs_plan_solver_free(self._h)
            self._h = None

    def __del__(self) -> None:  # pragma: no cover
        try:
            self.dispose()
        except Exception:
            pass

    @property
    def plan(self) -> Plan:
        return _plan_from(_ffi.take_json(lib.gcs_plan_solver_plan_json(self._h)))

    @property
    def graph(self) -> dict[str, Any]:
        g: dict[str, Any] = _ffi.take_json(lib.gcs_plan_solver_graph_json(self._h))
        return g

    @property
    def sticky_branches(self) -> bool:
        return self.plan.sticky_branches

    @sticky_branches.setter
    def sticky_branches(self, v: bool) -> None:
        lib.gcs_plan_solver_sticky(self._h, 1 if v else 0)

    def point_element(self, p: Point) -> int:
        """The point element (coincidence class) a sketch point belongs to."""
        return int(lib.gcs_plan_solver_point_element(self._h, p.index))

    def steps_placing(self, p: Point) -> list[int]:
        """The merges that place `p`: closed-form ones where it is the constructed apex, else the
        numeric merges that share it."""
        buf = _ffi.i32(max(len(self.plan.steps), 1))
        n = lib.gcs_plan_steps_placing(self._h, p.index, _ffi.pi(buf))
        return [int(i) for i in buf[:n]]

    def flip(self, p: Point) -> int:
        """Flip the root of every closed-form construction that places `p`; returns how many.
        Root choices are document state — `solve` reads them back."""
        return int(lib.gcs_plan_solver_flip(self._h, self.sketch._h, p.index))

    def execute(self) -> None:
        """Replay the plan on the current geometry (no solving, no fallback)."""
        lib.gcs_plan_solver_execute(self._h, self.sketch._h)

    def branches(self) -> dict[str, int]:
        return {s.key: s.branch for s in self.plan.steps if s.key and s.branch is not None}

    def ppp_triangles(self) -> list[Triangle]:
        pts = self.sketch.points
        return [(pts[a], pts[b], pts[c]) for a, b, c in self.plan.ppp_triangles]

    def solve(self, tol: float = 1e-9, fallback: bool = True,
              method: Method = "dogleg") -> PlanResult:
        t0 = time.perf_counter()
        d = _ffi.take_json(lib.gcs_plan_solver_solve(
            self._h, self.sketch._h, tol, 1 if fallback else 0, _METHOD_ID[method]))
        return PlanResult(bool(d["success"]), float(d["maxResidual"]), bool(d["fellBack"]),
                          time.perf_counter() - t0, int(d["nSteps"]), d["numeric"])


class PlanDrag:
    """DCM-style drag: the dragged point joins the ground and the cached plan replays per frame —
    no graph analysis while dragging, recorded roots are sticky, and under-constrained roots move
    least.  If the plan cannot determine the sketch with the point pinned, the numeric pull/polish
    `Drag` takes over."""

    def __init__(self, sketch: Sketch, point: Point, x: float, y: float,
                 guards: Sequence[Triangle] | None = None, max_step_rel: float = 0.05) -> None:
        self.sketch = sketch
        self.point = point
        ptr, n = _guard_buffer(guards)
        self._h = lib.gcs_plan_drag_new(sketch._h, point.index, float(x), float(y),
                                        ptr, n if guards is not None else -1, max_step_rel)
        sketch.touch()
        self.active = True

    @property
    def usable(self) -> bool:
        """True while the cached plan is driving the drag (false once it handed over)."""
        return bool(lib.gcs_plan_drag_usable(self._h))

    @property
    def numeric(self) -> bool:
        """Truthy once the numeric drag has taken over — the reference API's `drag.numeric`."""
        return not self.usable

    @property
    def flips(self) -> list[Triangle]:
        from gcs.solve import _read_flips
        return _read_flips(self.sketch, lib.gcs_plan_drag_flip_list, self._h,
                           int(lib.gcs_plan_drag_flips(self._h)))

    def guard_triangles(self) -> list[Triangle]:
        buf = _ffi.i32(3 * 4096)
        n = lib.gcs_plan_drag_guards(self._h, self.sketch._h, _ffi.pi(buf))
        pts = self.sketch.points
        return [(pts[int(buf[3 * i])], pts[int(buf[3 * i + 1])], pts[int(buf[3 * i + 2])])
                for i in range(n)]

    def branches(self) -> dict[str, int]:
        return _ffi.take_json(lib.gcs_plan_drag_branches_json(self._h)) or {}

    def move(self, x: float, y: float) -> SolveResult:
        t0 = time.perf_counter()
        out = _ffi.f64(8)
        msg = _ffi.take_str(lib.gcs_plan_drag_move(self._h, self.sketch._h, float(x), float(y),
                                                   _ffi.pf(out)))
        return _result(out, msg, "plan", t0)

    def end(self) -> None:
        if self.active:
            lib.gcs_plan_drag_end(self._h, self.sketch._h)
            self.sketch.touch()
            self.active = False

    def __del__(self) -> None:  # pragma: no cover
        try:
            lib.gcs_plan_drag_free(self._h)
        except Exception:
            pass


def build_graph(sketch: Sketch) -> dict[str, Any]:
    """The F–H constraint graph of a sketch (elements, edges, direction relations)."""
    g: dict[str, Any] = _ffi.take_json(lib.gcs_graph_json(sketch._h))
    return g


__all__ = ["Plan", "PlanDrag", "PlanResult", "PlanSolver", "Step", "build_graph"]
