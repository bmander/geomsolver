"""gcs — geometric constraint solver, Stage 0 (pure Python).

Core model: a sketch is a flat parameter vector x; every constraint contributes
residuals r_i(x) that vanish when satisfied; solving minimises ||r(x)||² from a
warm start.  See gcs-solver-program.md.
"""

from gcs.model import Arc, Circle, Line, Param, Point, Sketch
from gcs.constraints import (
    Angle,
    Coincident,
    Constraint,
    Distance,
    DragTarget,
    EqualLength,
    EqualRadius,
    Horizontal,
    Midpoint,
    Parallel,
    Perpendicular,
    PointOnCircle,
    PointOnLine,
    Radius,
    Symmetric,
    TangentArcLine,
    TangentCircleCircle,
    TangentLineCircle,
    Vertical,
)
from gcs.solve import SolveResult, solve

__all__ = [
    "Angle", "Arc", "Circle", "Coincident", "Constraint", "Distance", "DragTarget",
    "EqualLength", "EqualRadius", "Horizontal", "Line", "Midpoint", "Parallel",
    "Param", "Perpendicular", "Point", "PointOnCircle", "PointOnLine", "Radius",
    "Sketch", "SolveResult", "Symmetric", "TangentArcLine", "TangentCircleCircle",
    "TangentLineCircle", "Vertical", "solve",
]
