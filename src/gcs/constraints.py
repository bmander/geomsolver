"""Constraint types: entities → local parameter tuple + constants + a vectorized kernel.

Each class declares
  * `kernel`  — the vectorized residual/Jacobian kernel (gcs.kernels) for its type,
  * `params`  — the ordered tuple of Params the kernel's columns refer to,
  * `consts()` — the per-constraint constants the kernel needs (dimension values, flags),
  * `spec`    — its constructor arguments as (attribute, kind) pairs; kind is an entity
                kind (see ENTITY_KINDS) or a scalar kind (length, angle, float, int, str,
                bool).  Serialization, the UI's constraint list and value editing read it.
The scalar `residual(v)`/`jacobian(v)` methods are one-row views of the kernel,
kept for tests, diagnostics and the FD checker.

Residual forms follow the plan: distance uses ‖p−q‖² − d² (no sqrt), parallel
is a 2×2 determinant, angle is a dot/cross combination, tangency is signed
distance minus radius with a chirality flag fixed at construction.
"""

from __future__ import annotations

import math
from typing import Any, ClassVar, Literal

import numpy as np

from gcs import kernels as K
from gcs.kernels import Kernel
from gcs.model import Arc, Circle, Line, Param, Point, Vec

ENTITY_KINDS = frozenset({"point", "line", "circle", "arc", "circle_or_arc"})
Entity = Point | Line | Circle | Arc
_NO_CONSTS = np.zeros(0)


class Constraint:
    kernel: ClassVar[Kernel]
    params: tuple[Param, ...]
    spec: tuple[tuple[str, str], ...] = ()
    soft: bool = False        # soft constraints (drag targets) don't count toward convergence
    intrinsic: bool = False   # implied by a primitive's definition (e.g. arc endpoints at radius)
    commutative: ClassVar[bool] = False   # the first two spec entities may be swapped (see same_constraint)

    @property
    def n_residuals(self) -> int:
        return self.kernel.n_res

    def consts(self) -> Vec:
        return _NO_CONSTS

    def residual(self, v: Vec) -> Vec:
        return np.asarray(self.kernel.res(np.asarray(v, dtype=np.float64)[None, :], self.consts()[None, :])[0])

    def jacobian(self, v: Vec) -> Vec:
        return np.asarray(self.kernel.jac(np.asarray(v, dtype=np.float64)[None, :], self.consts()[None, :])[0])

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


def _same_args(a: Constraint, b: Constraint, swap: bool) -> bool:
    ents = [i for i, (_, kind) in enumerate(a.spec) if kind in ENTITY_KINDS]
    order = list(range(len(a.spec)))
    if swap:
        if len(ents) < 2:
            return False
        order[ents[0]], order[ents[1]] = order[ents[1]], order[ents[0]]
    av, bv = a.args(), b.args()
    for i, (_, kind) in enumerate(a.spec):
        x, y = av[i], bv[order[i]]
        if (x is not y) if kind in ENTITY_KINDS else (x != y):
            return False
    return True


def same_constraint(a: Constraint, b: Constraint) -> bool:
    """True when two constraints say exactly the same thing: same type, the same entities in
    the same roles, the same values.  `commutative` types also match with their first two
    entities swapped, since picking the pair in the other order means the same relation.

    Driven by `spec`, so a new constraint type is covered as soon as it declares one.

    An exact duplicate is worth keeping out of a sketch: it adds equations without adding rank,
    and a structural matching cannot see that — two identical rows still match two different
    variables — so it stays invisible until some unrelated edit tips the block into a
    (spurious) over-constrained report.
    """
    if type(a) is not type(b):
        return False
    return _same_args(a, b, swap=False) or (a.commutative and _same_args(a, b, swap=True))


# ---------------------------------------------------------------------------
# Point–point


class Coincident(Constraint):
    kernel = K.coincident
    commutative = True
    spec = (("p", "point"), ("q", "point"))

    def __init__(self, p: Point, q: Point) -> None:
        self.p, self.q = p, q
        self.params = p.params + q.params


class Distance(Constraint):
    """‖p − q‖² − d² = 0."""

    kernel = K.distance
    commutative = True
    spec = (("p", "point"), ("q", "point"), ("d", "length"))

    def __init__(self, p: Point, q: Point, d: float) -> None:
        self.p, self.q, self.d = p, q, float(d)
        self.params = p.params + q.params

    def consts(self) -> Vec:
        return np.array([self.d])


class Midpoint(Constraint):
    kernel = K.midpoint
    spec = (("p", "point"), ("line", "line"))

    def __init__(self, p: Point, line: Line) -> None:
        self.p, self.line = p, line
        self.params = p.params + line.params


class DragTarget(Constraint):
    """Soft constraint pulling `p` toward a (mutable) target; used for dragging."""

    kernel = K.drag
    spec = (("p", "point"), ("tx", "float"), ("ty", "float"), ("weight", "float"))
    soft = True

    def __init__(self, p: Point, tx: float, ty: float, weight: float = 1.0) -> None:
        self.p, self.tx, self.ty, self.weight = p, float(tx), float(ty), float(weight)
        self.params = p.params

    def set_target(self, tx: float, ty: float) -> None:
        self.tx, self.ty = float(tx), float(ty)

    def consts(self) -> Vec:
        return np.array([self.tx, self.ty, self.weight])


# ---------------------------------------------------------------------------
# Line orientation


class Horizontal(Constraint):
    kernel = K.horizontal
    spec = (("line", "line"),)

    def __init__(self, line: Line) -> None:
        self.line = line
        self.params = line.params


class Vertical(Constraint):
    kernel = K.vertical
    spec = (("line", "line"),)

    def __init__(self, line: Line) -> None:
        self.line = line
        self.params = line.params


class _TwoLine(Constraint):
    spec: tuple[tuple[str, str], ...] = (("l1", "line"), ("l2", "line"))

    def __init__(self, l1: Line, l2: Line) -> None:
        self.l1, self.l2 = l1, l2
        self.params = l1.params + l2.params


class Parallel(_TwoLine):
    """d1 × d2 = 0."""

    kernel = K.parallel
    commutative = True


class Perpendicular(_TwoLine):
    """d1 · d2 = 0."""

    kernel = K.perpendicular
    commutative = True


class Angle(_TwoLine):
    """CCW angle from l1 to l2 equals theta (mod π): dot·sinθ − cross·cosθ = 0."""

    kernel = K.angle
    spec = (("l1", "line"), ("l2", "line"), ("theta", "angle"))

    def __init__(self, l1: Line, l2: Line, theta: float) -> None:
        super().__init__(l1, l2)
        self.theta = float(theta)

    def consts(self) -> Vec:
        return np.array([math.sin(self.theta), math.cos(self.theta)])


class ParallelDistance(_TwoLine):
    """The gap between two parallel lines: l2's first endpoint sits a signed distance `d` from
    l1's infinite line, positive to the left of l1's direction.

    One residual.  It dimensions the gap; it does not *create* the parallelism — add
    `Parallel` for that if nothing else already implies it.  Bundling both in duplicated a
    parallelism that the rest of a sketch has usually already forced (a symmetry plus a chain
    of perpendiculars is enough), and the resulting redundancy was hard to see.
    """

    kernel = K.parallel_distance
    spec = (("l1", "line"), ("l2", "line"), ("d", "length"))

    def __init__(self, l1: Line, l2: Line, d: float) -> None:
        super().__init__(l1, l2)
        self.d = float(d)

    def consts(self) -> Vec:
        return np.array([self.d])


class EqualLength(_TwoLine):
    """|d1|² − |d2|² = 0."""

    kernel = K.equal_length
    commutative = True


# ---------------------------------------------------------------------------
# Incidence


class PointOnLine(Constraint):
    """(b−a) × (p−a) = 0."""

    kernel = K.point_on_line
    spec = (("p", "point"), ("line", "line"))

    def __init__(self, p: Point, line: Line) -> None:
        self.p, self.line = p, line
        self.params = p.params + line.params


class PointLineDistance(Constraint):
    """Signed perpendicular distance from `p` to `line`'s infinite line equals `d`, positive to
    the left of the line's direction.

    Signed rather than absolute: the residual has no kink at zero, and negating `d` moves the
    point to the other side the way a tangency's `side` flag does.  `PointOnLine` is the d = 0
    case and stays separate — it is a polynomial and needs no division.
    """

    kernel = K.point_line_distance
    spec = (("p", "point"), ("line", "line"), ("d", "length"))

    def __init__(self, p: Point, line: Line, d: float) -> None:
        self.p, self.line, self.d = p, line, float(d)
        self.params = p.params + line.params

    def consts(self) -> Vec:
        return np.array([self.d])


class PointOnCircle(Constraint):
    """‖p − c‖² − r² = 0."""

    kernel = K.point_on_circle
    spec = (("p", "point"), ("circle", "circle_or_arc"))

    def __init__(self, p: Point, circle: Circle | Arc, *, intrinsic: bool = False) -> None:
        self.p, self.circle = p, circle
        self.params = p.params + circle.center.params + (circle.radius,)
        self.intrinsic = intrinsic


# ---------------------------------------------------------------------------
# Radii


class Radius(Constraint):
    kernel = K.radius
    spec = (("circle", "circle_or_arc"), ("r", "length"))

    def __init__(self, circle: Circle | Arc, r: float) -> None:
        self.circle, self.r = circle, float(r)
        self.params = (circle.radius,)

    def consts(self) -> Vec:
        return np.array([self.r])


class EqualRadius(Constraint):
    kernel = K.equal_radius
    commutative = True
    spec = (("c1", "circle_or_arc"), ("c2", "circle_or_arc"))

    def __init__(self, c1: Circle | Arc, c2: Circle | Arc) -> None:
        self.c1, self.c2 = c1, c2
        self.params = (c1.radius, c2.radius)


class AnnularDistance(Constraint):
    """The annulus between two concentric circles: r2 − r1 = d, so `d` is the ring's radial
    thickness, positive when `c2` is the outer one.

    One residual, on the radii alone.  It does not make the pair concentric — `Coincident` on
    the two centres does that, and folding it in here would restate a constraint the sketch
    almost always already carries (the same trap `ParallelDistance` fell into).  Off-centre it
    still means "r2 − r1", which is the eccentric-annulus reading, not a rim-to-rim gap.
    """

    kernel = K.annular_distance
    spec = (("c1", "circle_or_arc"), ("c2", "circle_or_arc"), ("d", "length"))

    def __init__(self, c1: Circle | Arc, c2: Circle | Arc, d: float) -> None:
        self.c1, self.c2, self.d = c1, c2, float(d)
        self.params = (c1.radius, c2.radius)

    def consts(self) -> Vec:
        return np.array([self.d])


# ---------------------------------------------------------------------------
# Tangency


class TangentLineCircle(Constraint):
    """Signed distance from centre to line equals ±r: cross(b−a, c−a)/|b−a| − side·r = 0.

    `side` (+1/−1) is a chirality flag; if None it is read off the current
    geometry, so the solver keeps the circle on the side it already is.
    """

    kernel = K.tangent_line_circle
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

    def consts(self) -> Vec:
        return np.array([float(self.side)])


class TangentCircleCircle(Constraint):
    """‖c1 − c2‖² − (r1 ± r2)² = 0 (external: +, internal: −)."""

    kernel = K.tangent_circle_circle
    commutative = True
    spec = (("c1", "circle_or_arc"), ("c2", "circle_or_arc"), ("external", "bool"))

    def __init__(self, c1: Circle | Arc, c2: Circle | Arc, external: bool = True) -> None:
        self.c1, self.c2, self.external = c1, c2, external
        self.params = c1.center.params + (c1.radius,) + c2.center.params + (c2.radius,)

    def consts(self) -> Vec:
        return np.array([1.0 if self.external else -1.0])


class Symmetric(Constraint):
    """p and q mirror each other across `line`: their midpoint is on it and p→q crosses it
    at a right angle.  Two residuals, and the line itself is free to move."""

    kernel = K.symmetric
    commutative = True
    spec = (("p", "point"), ("q", "point"), ("line", "line"))

    def __init__(self, p: Point, q: Point, line: Line) -> None:
        self.p, self.q, self.line = p, q, line
        self.params = p.params + q.params + line.params


class TangentArcLine(Constraint):
    """Line is tangent to the arc at the arc's `at` endpoint: (p − c)·(b − a) = 0.

    Pair this with a Coincident between the arc endpoint and a line endpoint
    (the fillet pattern).
    """

    kernel = K.tangent_arc_line
    spec = (("arc", "arc"), ("line", "line"), ("at", "str"))

    def __init__(self, arc: Arc, line: Line, at: Literal["start", "end"]) -> None:
        self.arc, self.line, self.at = arc, line, at
        p = arc.start if at == "start" else arc.end
        self.params = p.params + arc.center.params + line.params
