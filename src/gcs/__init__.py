"""gcs — geometric constraint solver.

A thin Python binding over the Rust core (`rust/gcs-core`), reached through the flat C ABI in
`rust/gcs-ffi`.  The model, the numerics, structural diagnosis, cluster decomposition, witness
analysis and solution management all live there; nothing is reimplemented here.  See
`gcs-solver-program.md`.
"""

from gcs.model import Arc, Circle, Line, Param, Point, Sketch, distance_between, expand
from gcs.constraints import (
    Angle,
    AnnularDistance,
    Coincident,
    Constraint,
    Distance,
    DragTarget,
    EqualLength,
    EqualRadius,
    Horizontal,
    Midpoint,
    Parallel,
    ParallelDistance,
    Perpendicular,
    PointLineDistance,
    PointOnCircle,
    PointOnLine,
    Radius,
    Symmetric,
    TangentArcLine,
    TangentCircleCircle,
    TangentLineCircle,
    Vertical,
    same_constraint,
)
from gcs.solve import Drag, RadiusDrag, SolveResult, System, solve

__all__ = [
    "Angle", "AnnularDistance", "Arc", "Circle", "Coincident", "Constraint", "Distance",
    "DragTarget", "Drag", "EqualLength", "EqualRadius", "Horizontal", "Line", "Midpoint",
    "Parallel", "ParallelDistance", "Param", "Perpendicular", "Point", "PointLineDistance",
    "PointOnCircle", "PointOnLine", "Radius", "RadiusDrag", "Sketch", "SolveResult", "Symmetric",
    "System", "TangentArcLine", "TangentCircleCircle", "TangentLineCircle", "Vertical",
    "distance_between", "expand", "same_constraint", "solve",
]
