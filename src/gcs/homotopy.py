"""Stage 5 — homotopy continuation to enumerate the real solutions of one small merge.

"We can show you the other solutions": the tracking runs in the core; this module presents the
alternatives and applies one.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from gcs import _ffi
from gcs._ffi import Vec, lib
from gcs.decompose import PlanSolver
from gcs.model import Point


@dataclass
class Alternative:
    u: Vec                                   # (theta, tx, ty) per movable cluster
    distance: float                          # 0 for the root the sketch is on
    location: tuple[float, float] | None     # where a requested point element would land

    @property
    def is_current(self) -> bool:
        return self.distance < 1e-6


def enumerate_step(solver: PlanSolver, step_index: int, locate: Point | None = None,
                   seed: int = 0, max_paths: int = 256) -> list[Alternative]:
    """Real solutions of the merge at `step_index` (the current one first).  Empty if the merge is
    not isolated (under-determined) or too large."""
    d = _ffi.take_json(lib.gcs_enumerate_step_json(
        solver._h, solver.sketch._h, int(step_index),
        -1 if locate is None else locate.index, seed & 0xFFFFFFFF, int(max_paths)))
    return [
        Alternative(np.array(a["u"], dtype=np.float64), float(a["distance"]),
                    None if a["location"] is None else (a["location"][0], a["location"][1]))
        for a in (d or [])
    ]


def apply_alternative(solver: PlanSolver, step_index: int, alt: Alternative) -> None:
    """Put the sketch on this root, then replay the whole plan so dependent geometry follows."""
    p, n = _ffi.send_json({"u": list(map(float, alt.u)), "distance": alt.distance})
    lib.gcs_apply_alternative(solver._h, solver.sketch._h, int(step_index), p, n)


__all__ = ["Alternative", "apply_alternative", "enumerate_step"]
