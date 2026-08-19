"""Constraint graph for decomposition (Stage 3).

Fudos–Hoffmann work on *geometric elements* with 2 DOF each in the plane —
points and (infinite) lines — joined by valency-1 constraints: point–point
distance, point–line (signed) distance (0 = incidence), line–line angle.
This module maps a Sketch onto that model:

* points are contracted by Coincident (one element per equivalence class);
* a Line becomes a line element the first time a supported constraint refers
  to it (its endpoints are incident); lines nobody refers to are *passive* —
  their endpoints determine them — and get no element;
* a circle/arc whose radius is *known* (Radius constraint, fixed param, or an
  EqualRadius chain to a known one) contributes distance edges: point-on-circle
  → dist(centre, point) = r; line tangency → signed dist(centre, line) = ±r;
  arc-endpoint tangency → a *virtual radius line* through centre and endpoint,
  perpendicular to the tangent line (a transversal intersection, not a double
  root);
* angle-type constraints (Horizontal/Vertical against a ground x-axis line,
  Parallel, Perpendicular, Angle) are *direction relations* — a parallel pair
  is not a rigid pair (the separation is free), so these are not cluster
  leaves; they contribute one equation when clusters merge.  Fixed points and
  the x-axis form the *ground* elements (world frame);
* everything else (variable radii, EqualLength, EqualRadius as such, Midpoint,
  circle–circle tangency, soft constraints…) is listed as `unsupported` and is
  left to the numeric residual step.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Literal, NamedTuple

from gcs import constraints as C
from gcs.constraints import Constraint
from gcs.graph import UnionFind
from gcs.model import Arc, Circle, Line, Param, Point, Sketch

Kind = Literal["P", "L", "V"]      # point class | line | virtual line (through two point classes)


class El(NamedTuple):
    """A geometric element.  A NamedTuple so hashing/equality are C-level (dict keys on the hot path)."""

    kind: Kind
    idx: int

    def __repr__(self) -> str:
        return f"{self.kind}{self.idx}"


@dataclass
class Edge:
    """A valency-1 constraint between two elements: 'PP' distance | 'PL' signed
    distance (0 = incidence).  value(): live dimension (reads the constraint each
    execution, so edits/drags replay without recompiling)."""

    kind: str
    a: El
    b: El
    value_fn: Callable[[], float]
    source: Constraint | None        # None for implicit edges (line endpoints, virtual radius lines)

    def value(self) -> float:
        return float(self.value_fn())

    def __repr__(self) -> str:
        return f"Edge({self.kind} {self.a}-{self.b} = {self.value():.4g})"


@dataclass
class DirRelation:
    """dir(b) = dir(a) + phi  (normals: n_b = rot(phi)·n_a).  phi is the branch (mod π)
    nearest the current geometry at build time — a chirality-like choice."""

    a: El
    b: El
    phi: float
    source: Constraint


X_AXIS = El("L", -1)   # ground line y = 0 (normal (0,1), c = 0)


@dataclass
class ConstraintGraph:
    sketch: Sketch
    point_of: dict[int, int] = field(default_factory=dict)      # id(Point) → point-element index
    members: list[list[Point]] = field(default_factory=list)    # point-element → its (coincident) Points
    lines: list[Line] = field(default_factory=list)              # line-element index → Line (registered lazily)
    line_of: dict[int, int] = field(default_factory=dict)       # id(Line) → line-element index
    virtual: list[tuple[El, El]] = field(default_factory=list)   # virtual line index → (P, P) it passes through
    edges: list[Edge] = field(default_factory=list)
    dirs: list[DirRelation] = field(default_factory=list)
    unsupported: list[Constraint] = field(default_factory=list)
    ground_points: list[int] = field(default_factory=list)      # point-elements with fixed coordinates
    known_radius: dict[int, float] = field(default_factory=dict)  # id(radius Param) → value
    passive: list[Line] = field(default_factory=list)            # lines determined by their endpoints only
    X_AXIS: El = X_AXIS

    def P(self, p: Point) -> El:  # noqa: N802
        return El("P", self.point_of[id(p)])

    def L(self, ln: Line) -> El:  # noqa: N802
        """Line element for `ln`, registered on first use."""
        i = self.line_of.get(id(ln))
        if i is None:
            i = self.line_of[id(ln)] = len(self.lines)
            self.lines.append(ln)
        return El("L", i)

    def virtual_line(self, a: El, b: El) -> El:
        """Line element through two point elements (e.g. an arc's radius at an endpoint)."""
        self.virtual.append((a, b))
        return El("V", len(self.virtual) - 1)

    @property
    def n_points(self) -> int:
        return len(self.members)

    @property
    def elements(self) -> list[El]:
        return ([El("P", i) for i in range(self.n_points)] + [El("L", i) for i in range(len(self.lines))]
                + [El("V", i) for i in range(len(self.virtual))] + [self.X_AXIS])

    def summary(self) -> str:
        return (f"{self.n_points} point elements, {len(self.lines)} lines (+{len(self.passive)} passive, "
                f"{len(self.virtual)} virtual), {len(self.edges)} edges, {len(self.dirs)} direction relations "
                f"({len(self.unsupported)} unsupported constraints), {len(self.ground_points)} ground points")


def coincident_classes(sketch: Sketch) -> tuple[dict[int, int], list[list[Point]]]:
    """Points contracted by Coincident: (id(Point) → class index, class → member Points)."""
    pts = sketch.points
    idx = {id(p): i for i, p in enumerate(pts)}
    uf = UnionFind(len(pts))
    for c in sketch.constraints:
        if isinstance(c, C.Coincident):
            uf.union(idx[id(c.p)], idx[id(c.q)])
    lab, n = uf.labels()
    members: list[list[Point]] = [[] for _ in range(n)]
    for i, p in enumerate(pts):
        members[lab[i]].append(p)
    return {id(p): lab[i] for i, p in enumerate(pts)}, members


def known_radii(sketch: Sketch) -> dict[int, float]:
    """id(radius Param) → value for every radius fixed, dimensioned by Radius, or EqualRadius-chained to one."""
    rounds: list[Circle | Arc] = [*sketch.circles, *sketch.arcs]
    radii: list[Param] = [c.radius for c in rounds]
    ridx = {id(r): i for i, r in enumerate(radii)}
    ruf = UnionFind(len(radii))
    for c in sketch.constraints:
        if isinstance(c, C.EqualRadius):
            ruf.union(ridx[id(c.c1.radius)], ridx[id(c.c2.radius)])
    known: dict[int, float] = {}
    for c in sketch.constraints:            # after all unions, so class roots are final
        if isinstance(c, C.Radius):
            known[ruf.find(ridx[id(c.circle.radius)])] = c.r
    for r in radii:
        if r.fixed:
            known.setdefault(ruf.find(ridx[id(r)]), r.value)
    return {id(r): known[ruf.find(ridx[id(r)])] for r in radii if ruf.find(ridx[id(r)]) in known}


def build(sketch: Sketch) -> ConstraintGraph:
    g = ConstraintGraph(sketch)
    g.point_of, g.members = coincident_classes(sketch)
    g.ground_points = [k for k, ms in enumerate(g.members) if any(p.is_fixed for p in ms)]
    g.known_radius = known_radii(sketch)
    kr = g.known_radius

    def rad(circle: Circle | Arc) -> float | None:
        return kr.get(id(circle.radius))

    for c in sketch.constraints:
        if c.soft or isinstance(c, (C.Coincident, C.Radius)):
            continue                                     # contracted / absorbed into known radii
        if isinstance(c, C.Distance):
            g.edges.append(Edge("PP", g.P(c.p), g.P(c.q), _attr(c, "d"), c))
        elif isinstance(c, C.PointOnLine):
            g.edges.append(Edge("PL", g.P(c.p), g.L(c.line), (lambda: 0.0), c))
        elif isinstance(c, C.Horizontal):
            g.dirs.append(DirRelation(X_AXIS, g.L(c.line), _branch((0.0, 1.0), line_normal(c.line), 0.0), c))
        elif isinstance(c, C.Vertical):
            g.dirs.append(DirRelation(X_AXIS, g.L(c.line), _branch((0.0, 1.0), line_normal(c.line), math.pi / 2), c))
        elif isinstance(c, (C.Parallel, C.Perpendicular, C.Angle)):
            target = 0.0 if isinstance(c, C.Parallel) else math.pi / 2 if isinstance(c, C.Perpendicular) else c.theta
            g.dirs.append(DirRelation(g.L(c.l1), g.L(c.l2), _branch(line_normal(c.l1), line_normal(c.l2), target), c))
        elif isinstance(c, C.PointOnCircle) and rad(c.circle) is not None:
            g.edges.append(Edge("PP", g.P(c.circle.center), g.P(c.p), _radius(kr, c.circle, 1.0), c))
        elif isinstance(c, C.TangentLineCircle) and rad(c.circle) is not None:
            # signed distance from centre to line = side·r  (kernel: cross(d, c−a)/|d| − side·r;
            # our line normal n = (−dy, dx)/|d| gives n·(c − a) = cross(d, c−a)/|d|)
            g.edges.append(Edge("PL", g.P(c.circle.center), g.L(c.line), _radius(kr, c.circle, float(c.side)), c))
        elif isinstance(c, C.TangentArcLine) and rad(c.arc) is not None:
            # tangent at endpoint p ⇔ radius c–p ⟂ line (p on the line and |c−p| = r come from elsewhere)
            pt = c.arc.start if c.at == "start" else c.arc.end
            cen, pe = g.P(c.arc.center), g.P(pt)
            R = g.virtual_line(cen, pe)
            g.edges.append(Edge("PL", cen, R, (lambda: 0.0), None))
            g.edges.append(Edge("PL", pe, R, (lambda: 0.0), None))
            nR = normal_of(c.arc.center.x.value, c.arc.center.y.value, pt.x.value, pt.y.value)
            g.dirs.append(DirRelation(g.L(c.line), R, _branch(line_normal(c.line), nR, math.pi / 2), c))
        elif isinstance(c, C.EqualRadius) and rad(c.c1) is not None:
            pass                                         # absorbed into known radii
        else:
            g.unsupported.append(c)
    # endpoints lie on their (registered) line — implicit incidences; the rest are passive
    for ln in list(g.lines):
        for p in (ln.p1, ln.p2):
            g.edges.append(Edge("PL", g.P(p), g.L(ln), (lambda: 0.0), None))
    g.passive = [ln for ln in sketch.lines if id(ln) not in g.line_of]
    return g


# -- value sources (factories, so each closure binds its own constraint) -----

def _attr(c: Constraint, name: str) -> Callable[[], float]:
    return lambda: float(getattr(c, name))


def _radius(kr: dict[int, float], circle: Circle | Arc, sign: float) -> Callable[[], float]:
    key = id(circle.radius)
    return lambda: sign * kr[key]


# -- line helpers -----------------------------------------------------------

def normal_of(ax: float, ay: float, bx: float, by: float) -> tuple[float, float, float]:
    """(nx, ny, c) for the line a→b: n = unit left normal, n·X = c on the line."""
    dx, dy = bx - ax, by - ay
    L = math.hypot(dx, dy) or 1.0
    nx, ny = -dy / L, dx / L
    return nx, ny, nx * ax + ny * ay


def line_normal(ln: Line) -> tuple[float, float, float]:
    return normal_of(ln.p1.x.value, ln.p1.y.value, ln.p2.x.value, ln.p2.y.value)


def _branch(n1: tuple[float, ...], n2: tuple[float, ...], target: float) -> float:
    """Angle from normal n1 to normal n2 on the branch of `target` (mod π) nearest the current geometry."""
    cur = math.atan2(n1[0] * n2[1] - n1[1] * n2[0], n1[0] * n2[0] + n1[1] * n2[1])
    k = round((cur - target) / math.pi)
    return target + k * math.pi
