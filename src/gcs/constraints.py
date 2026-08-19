"""Constraint types: each knows its parameters, residuals and analytic Jacobian.

A constraint operates on a *local* value vector `v` (its params, in order) and
returns residuals of shape (n_residuals,) and a dense local Jacobian of shape
(n_residuals, len(params)).  The solver scatters these into the global sparse
system.  A Param may appear more than once in one constraint's param list (e.g.
two lines sharing a point); the assembler sums duplicate entries.

Residual forms follow the plan: distance uses ‖p−q‖² − d² (no sqrt), parallel
is a 2×2 determinant, angle is a dot/cross combination, tangency is signed
distance minus radius with a chirality flag fixed at construction.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from typing import Any, Literal

import numpy as np

from gcs.model import Arc, Circle, Line, Param, Point, Vec


ENTITY_KINDS = frozenset({"point", "line", "circle", "arc", "circle_or_arc"})
Entity = Point | Line | Circle | Arc


class Constraint(ABC):
    """Base class. Subclasses set `params` and `n_residuals` in __init__ and declare
    `spec`: the constructor arguments as (attribute name, kind) pairs, kind being an
    entity kind (see ENTITY_KINDS) or a scalar kind (length, angle, float, int, str,
    bool).  Serialization, the UI's constraint list and value editing all read it."""

    params: tuple[Param, ...]
    n_residuals: int
    spec: tuple[tuple[str, str], ...] = ()
    soft: bool = False        # soft constraints (drag targets) don't count toward convergence
    intrinsic: bool = False   # implied by a primitive's definition (e.g. arc endpoints at radius)

    @abstractmethod
    def residual(self, v: Vec) -> Vec: ...

    @abstractmethod
    def jacobian(self, v: Vec) -> Vec: ...

    def entities(self) -> list[Entity]:
        """Entities this constraint references directly, in spec order."""
        return [getattr(self, name) for name, kind in self.spec if kind in ENTITY_KINDS]

    def args(self) -> list[Any]:
        """Constructor arguments in spec order (round-trips through `type(self)(*args)`)."""
        return [getattr(self, name) for name, _ in self.spec]

    def local_values(self) -> Vec:
        return np.array([p.value for p in self.params], dtype=np.float64)

    def error(self) -> float:
        """Current residual norm (convenience for reporting)."""
        return float(np.linalg.norm(self.residual(self.local_values())))

    def __repr__(self) -> str:
        return f"{type(self).__name__}(n={self.n_residuals})"


# ---------------------------------------------------------------------------
# Point–point


class Coincident(Constraint):
    spec = (("p", "point"), ("q", "point"))
    n_residuals = 2

    def __init__(self, p: Point, q: Point) -> None:
        self.p, self.q = p, q
        self.params = p.params + q.params

    def residual(self, v: Vec) -> Vec:
        return np.array([v[0] - v[2], v[1] - v[3]])

    _J = np.array([[1.0, 0.0, -1.0, 0.0], [0.0, 1.0, 0.0, -1.0]])

    def jacobian(self, v: Vec) -> Vec:
        return self._J


class Distance(Constraint):
    """‖p − q‖² − d² = 0."""

    spec = (("p", "point"), ("q", "point"), ("d", "length"))
    n_residuals = 1

    def __init__(self, p: Point, q: Point, d: float) -> None:
        self.p, self.q, self.d = p, q, float(d)
        self.params = p.params + q.params

    def residual(self, v: Vec) -> Vec:
        dx, dy = v[0] - v[2], v[1] - v[3]
        return np.array([dx * dx + dy * dy - self.d * self.d])

    def jacobian(self, v: Vec) -> Vec:
        dx, dy = v[0] - v[2], v[1] - v[3]
        return np.array([[2 * dx, 2 * dy, -2 * dx, -2 * dy]])


class Midpoint(Constraint):
    spec = (("p", "point"), ("line", "line"))
    n_residuals = 2

    def __init__(self, p: Point, line: Line) -> None:
        self.p, self.line = p, line
        self.params = p.params + line.params

    def residual(self, v: Vec) -> Vec:
        return np.array([2 * v[0] - v[2] - v[4], 2 * v[1] - v[3] - v[5]])

    _J = np.array([[2.0, 0, -1, 0, -1, 0], [0, 2.0, 0, -1, 0, -1]])

    def jacobian(self, v: Vec) -> Vec:
        return self._J


class DragTarget(Constraint):
    """Soft constraint pulling `p` toward a (mutable) target; used for dragging."""

    spec = (("p", "point"), ("tx", "float"), ("ty", "float"), ("weight", "float"))
    n_residuals = 2
    soft = True

    def __init__(self, p: Point, tx: float, ty: float, weight: float = 1.0) -> None:
        self.p, self.tx, self.ty, self.weight = p, float(tx), float(ty), float(weight)
        self.params = p.params

    def set_target(self, tx: float, ty: float) -> None:
        self.tx, self.ty = float(tx), float(ty)

    def residual(self, v: Vec) -> Vec:
        return self.weight * np.array([v[0] - self.tx, v[1] - self.ty])

    _EYE = np.eye(2)

    def jacobian(self, v: Vec) -> Vec:
        return self.weight * self._EYE


# ---------------------------------------------------------------------------
# Line orientation


class Horizontal(Constraint):
    spec = (("line", "line"),)
    n_residuals = 1

    def __init__(self, line: Line) -> None:
        self.line = line
        self.params = line.params

    def residual(self, v: Vec) -> Vec:
        return np.array([v[1] - v[3]])

    _J = np.array([[0.0, 1.0, 0.0, -1.0]])

    def jacobian(self, v: Vec) -> Vec:
        return self._J


class Vertical(Constraint):
    spec = (("line", "line"),)
    n_residuals = 1

    def __init__(self, line: Line) -> None:
        self.line = line
        self.params = line.params

    def residual(self, v: Vec) -> Vec:
        return np.array([v[0] - v[2]])

    _J = np.array([[1.0, 0.0, -1.0, 0.0]])

    def jacobian(self, v: Vec) -> Vec:
        return self._J


class _TwoLine(Constraint):
    """Shared plumbing: local v = (a1x,a1y,b1x,b1y,a2x,a2y,b2x,b2y)."""
    spec: tuple[tuple[str, str], ...] = (("l1", "line"), ("l2", "line"))
    spec = (("l1", "line"), ("l2", "line"))
    n_residuals = 1

    def __init__(self, l1: Line, l2: Line) -> None:
        self.l1, self.l2 = l1, l2
        self.params = l1.params + l2.params

    @staticmethod
    def _dirs(v: Vec) -> tuple[float, float, float, float]:
        return v[2] - v[0], v[3] - v[1], v[6] - v[4], v[7] - v[5]

    @staticmethod
    def _cross(v: Vec) -> float:
        d1x, d1y, d2x, d2y = _TwoLine._dirs(v)
        return d1x * d2y - d1y * d2x

    @staticmethod
    def _dot(v: Vec) -> float:
        d1x, d1y, d2x, d2y = _TwoLine._dirs(v)
        return d1x * d2x + d1y * d2y

    @staticmethod
    def _cross_jac(v: Vec) -> Vec:
        d1x, d1y, d2x, d2y = _TwoLine._dirs(v)
        return np.array([-d2y, d2x, d2y, -d2x, d1y, -d1x, -d1y, d1x])

    @staticmethod
    def _dot_jac(v: Vec) -> Vec:
        d1x, d1y, d2x, d2y = _TwoLine._dirs(v)
        return np.array([-d2x, -d2y, d2x, d2y, -d1x, -d1y, d1x, d1y])


class Parallel(_TwoLine):
    """d1 × d2 = 0."""

    def residual(self, v: Vec) -> Vec:
        return np.array([self._cross(v)])

    def jacobian(self, v: Vec) -> Vec:
        return self._cross_jac(v)[None, :]


class Perpendicular(_TwoLine):
    """d1 · d2 = 0."""

    def residual(self, v: Vec) -> Vec:
        return np.array([self._dot(v)])

    def jacobian(self, v: Vec) -> Vec:
        return self._dot_jac(v)[None, :]


class Angle(_TwoLine):
    """CCW angle from l1 to l2 equals theta (mod π): dot·sinθ − cross·cosθ = 0."""

    spec = (("l1", "line"), ("l2", "line"), ("theta", "angle"))
    def __init__(self, l1: Line, l2: Line, theta: float) -> None:
        super().__init__(l1, l2)
        self.theta = theta

    @property
    def theta(self) -> float:
        return self._theta

    @theta.setter
    def theta(self, value: float) -> None:
        self._theta = float(value)
        self._s, self._c = math.sin(self._theta), math.cos(self._theta)

    def residual(self, v: Vec) -> Vec:
        return np.array([self._dot(v) * self._s - self._cross(v) * self._c])

    def jacobian(self, v: Vec) -> Vec:
        return (self._dot_jac(v) * self._s - self._cross_jac(v) * self._c)[None, :]


class EqualLength(_TwoLine):
    """|d1|² − |d2|² = 0."""

    def residual(self, v: Vec) -> Vec:
        d1x, d1y, d2x, d2y = self._dirs(v)
        return np.array([d1x * d1x + d1y * d1y - d2x * d2x - d2y * d2y])

    def jacobian(self, v: Vec) -> Vec:
        d1x, d1y, d2x, d2y = self._dirs(v)
        return 2 * np.array([[-d1x, -d1y, d1x, d1y, d2x, d2y, -d2x, -d2y]])


# ---------------------------------------------------------------------------
# Incidence


class PointOnLine(Constraint):
    """(b−a) × (p−a) = 0.  Local v = (px,py,ax,ay,bx,by)."""

    spec = (("p", "point"), ("line", "line"))
    n_residuals = 1

    def __init__(self, p: Point, line: Line) -> None:
        self.p, self.line = p, line
        self.params = p.params + line.params

    def residual(self, v: Vec) -> Vec:
        dx, dy = v[4] - v[2], v[5] - v[3]
        wx, wy = v[0] - v[2], v[1] - v[3]
        return np.array([dx * wy - dy * wx])

    def jacobian(self, v: Vec) -> Vec:
        dx, dy = v[4] - v[2], v[5] - v[3]
        wx, wy = v[0] - v[2], v[1] - v[3]
        return np.array([[-dy, dx, dy - wy, wx - dx, wy, -wx]])


class PointOnCircle(Constraint):
    """‖p − c‖² − r² = 0.  Local v = (px,py,cx,cy,r)."""

    spec = (("p", "point"), ("circle", "circle_or_arc"))
    n_residuals = 1

    def __init__(self, p: Point, circle: Circle | Arc, *, intrinsic: bool = False) -> None:
        self.p, self.circle = p, circle
        self.params = p.params + circle.center.params + (circle.radius,)
        self.intrinsic = intrinsic

    def residual(self, v: Vec) -> Vec:
        ux, uy = v[0] - v[2], v[1] - v[3]
        return np.array([ux * ux + uy * uy - v[4] * v[4]])

    def jacobian(self, v: Vec) -> Vec:
        ux, uy = v[0] - v[2], v[1] - v[3]
        return np.array([[2 * ux, 2 * uy, -2 * ux, -2 * uy, -2 * v[4]]])



# ---------------------------------------------------------------------------
# Radii


class Radius(Constraint):
    spec = (("circle", "circle_or_arc"), ("r", "length"))
    n_residuals = 1

    def __init__(self, circle: Circle | Arc, r: float) -> None:
        self.circle, self.r = circle, float(r)
        self.params = (circle.radius,)

    def residual(self, v: Vec) -> Vec:
        return np.array([v[0] - self.r])

    _J = np.array([[1.0]])

    def jacobian(self, v: Vec) -> Vec:
        return self._J


class EqualRadius(Constraint):
    spec = (("c1", "circle_or_arc"), ("c2", "circle_or_arc"))
    n_residuals = 1

    def __init__(self, c1: Circle | Arc, c2: Circle | Arc) -> None:
        self.c1, self.c2 = c1, c2
        self.params = (c1.radius, c2.radius)

    def residual(self, v: Vec) -> Vec:
        return np.array([v[0] - v[1]])

    _J = np.array([[1.0, -1.0]])

    def jacobian(self, v: Vec) -> Vec:
        return self._J


# ---------------------------------------------------------------------------
# Tangency


class TangentLineCircle(Constraint):
    """Signed distance from centre to line equals ±r.

    cross(b−a, c−a)/|b−a| − side·r = 0, local v = (ax,ay,bx,by,cx,cy,r).
    `side` (+1/−1) is a chirality flag; if None it is read off the current
    geometry, so the solver keeps the circle on the side it already is.
    """

    n_residuals = 1
    spec = (("line", "line"), ("circle", "circle_or_arc"), ("side", "int"))

    def __init__(self, line: Line, circle: Circle | Arc, side: int | None = None) -> None:
        self.line, self.circle = line, circle
        self.params = line.params + circle.center.params + (circle.radius,)
        if side is None:
            v = self.local_values()
            dx, dy = v[2] - v[0], v[3] - v[1]
            wx, wy = v[4] - v[0], v[5] - v[1]
            side = 1 if dx * wy - dy * wx >= 0 else -1
        self.side = int(side)

    def residual(self, v: Vec) -> Vec:
        dx, dy = v[2] - v[0], v[3] - v[1]
        wx, wy = v[4] - v[0], v[5] - v[1]
        L = math.hypot(dx, dy)
        return np.array([(dx * wy - dy * wx) / L - self.side * v[6]])

    def jacobian(self, v: Vec) -> Vec:
        dx, dy = v[2] - v[0], v[3] - v[1]
        wx, wy = v[4] - v[0], v[5] - v[1]
        L = math.hypot(dx, dy)
        C = dx * wy - dy * wx
        # ∂C/∂(ax,ay,bx,by,cx,cy), ∂L/∂(ax,ay,bx,by)
        dC = np.array([dy - wy, wx - dx, wy, -wx, -dy, dx, 0.0])
        dL = np.array([-dx / L, -dy / L, dx / L, dy / L, 0.0, 0.0, 0.0])
        J = dC / L - C * dL / (L * L)
        J[6] = -self.side
        return np.asarray(J[None, :], dtype=np.float64)


class TangentCircleCircle(Constraint):
    """‖c1 − c2‖² − (r1 ± r2)² = 0 (external: +, internal: −)."""

    n_residuals = 1
    spec = (("c1", "circle_or_arc"), ("c2", "circle_or_arc"), ("external", "bool"))

    def __init__(self, c1: Circle | Arc, c2: Circle | Arc, external: bool = True) -> None:
        self.c1, self.c2, self.external = c1, c2, external
        self.params = c1.center.params + (c1.radius,) + c2.center.params + (c2.radius,)

    def residual(self, v: Vec) -> Vec:
        ux, uy = v[0] - v[3], v[1] - v[4]
        R = v[2] + v[5] if self.external else v[2] - v[5]
        return np.array([ux * ux + uy * uy - R * R])

    def jacobian(self, v: Vec) -> Vec:
        ux, uy = v[0] - v[3], v[1] - v[4]
        R = v[2] + v[5] if self.external else v[2] - v[5]
        dr2 = -2 * R if self.external else 2 * R
        return np.array([[2 * ux, 2 * uy, -2 * R, -2 * ux, -2 * uy, dr2]])


class TangentArcLine(Constraint):
    """Line is tangent to the arc at the arc's `at` endpoint: (p − c)·(b − a) = 0.

    Pair this with a Coincident between the arc endpoint and a line endpoint
    (the fillet pattern).  Local v = (px,py,cx,cy,ax,ay,bx,by).
    """

    n_residuals = 1
    spec = (("arc", "arc"), ("line", "line"), ("at", "str"))

    def __init__(self, arc: Arc, line: Line, at: Literal["start", "end"]) -> None:
        self.arc, self.line, self.at = arc, line, at
        p = arc.start if at == "start" else arc.end
        self.params = p.params + arc.center.params + line.params

    def residual(self, v: Vec) -> Vec:
        ux, uy = v[0] - v[2], v[1] - v[3]
        dx, dy = v[6] - v[4], v[7] - v[5]
        return np.array([ux * dx + uy * dy])

    def jacobian(self, v: Vec) -> Vec:
        ux, uy = v[0] - v[2], v[1] - v[3]
        dx, dy = v[6] - v[4], v[7] - v[5]
        return np.array([[dx, dy, -dx, -dy, -ux, -uy, ux, uy]])
