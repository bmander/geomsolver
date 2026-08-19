"""Constraint graph for decomposition (Stage 3).

Fudos–Hoffmann work on *geometric elements* with 2 DOF each in the plane —
points and (infinite) lines — joined by valency-1 constraints: point–point
distance, point–line (signed) distance (0 = incidence), line–line angle.
This module maps a Sketch onto that model:

* points are contracted by Coincident (one element per equivalence class);
* every Line primitive is a line element, incident to its two endpoints;
* a circle/arc whose radius is *known* (Radius constraint, fixed param, or an
  EqualRadius chain to a known one) contributes distance edges: point-on-circle
  → dist(centre, point) = r; line/arc tangency → signed dist(centre, line) = ±r;
* angle-type constraints (Horizontal/Vertical against a ground x-axis line,
  Parallel, Perpendicular, Angle) are *direction relations*: a weighted
  union-find puts lines into direction classes with fixed relative angles.
  (A parallel pair is not a rigid pair — the separation is free — so these are
  not cluster leaves; they contribute one equation when clusters merge.)
  Fixed points and the x-axis form the *ground* elements (world frame);
* lines that are only referenced by their own endpoints (truss members…) are
  *passive* and get no element — their endpoints determine them;
* everything else (variable radii, EqualLength, EqualRadius as such, Midpoint,
  circle–circle tangency, soft constraints…) is listed as `unsupported` and is
  left to the numeric residual step.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Literal, NamedTuple

from gcs import constraints as C
from gcs.constraints import Constraint
from gcs.graph import UnionFind
from gcs.model import Arc, Circle, Line, Param, Point, Sketch

Kind = Literal["P", "L"]


class El(NamedTuple):
    """A geometric element: ('P', i) point class or ('L', i) line.  A NamedTuple so that
    hashing/equality are C-level — these are dict keys on the hot path."""

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
    value_fn: object                 # Callable[[], float]
    source: Constraint | None        # None for implicit edges (line endpoints, ground)

    def value(self) -> float:
        return float(self.value_fn())  # type: ignore[operator]

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


@dataclass
class ConstraintGraph:
    sketch: Sketch
    point_of: dict[int, int] = field(default_factory=dict)      # id(Point) → point-element index
    members: list[list[Point]] = field(default_factory=list)    # point-element → its (coincident) Points
    lines: list[Line] = field(default_factory=list)              # line-element index → Line
    line_of: dict[int, int] = field(default_factory=dict)       # id(Line) → line-element index
    edges: list[Edge] = field(default_factory=list)
    dirs: list[DirRelation] = field(default_factory=list)
    unsupported: list[Constraint] = field(default_factory=list)
    ground_points: list[int] = field(default_factory=list)      # point-elements with fixed coordinates
    known_radius: dict[int, float] = field(default_factory=dict)  # id(radius Param) → value
    passive: list[Line] = field(default_factory=list)            # lines determined by their endpoints only
    virtual: dict[int, tuple[El, El]] = field(default_factory=dict)  # line-element index → (P, P) it passes through
    X_AXIS: El = El("L", -1)                                    # ground line y = 0 (normal (0,1), c = 0)

    def P(self, p: Point) -> El:  # noqa: N802
        return El("P", self.point_of[id(p)])

    def L(self, ln: Line) -> El:  # noqa: N802
        return El("L", self.line_of[id(ln)])

    @property
    def n_points(self) -> int:
        return len(self.members)

    @property
    def n_lines(self) -> int:
        return len(self.lines) + len(self.virtual)

    @property
    def elements(self) -> list[El]:
        return [El("P", i) for i in range(self.n_points)] + [El("L", i) for i in range(self.n_lines)] + [self.X_AXIS]

    def virtual_line(self, a: El, b: El) -> El:
        """Line element through two point elements (e.g. an arc's radius at an endpoint)."""
        e = El("L", len(self.lines) + len(self.virtual))
        self.virtual[e.idx] = (a, b)
        return e

    def summary(self) -> str:
        return (f"{self.n_points} point elements, {len(self.lines)} lines (+{len(self.passive)} passive), "
                f"{len(self.edges)} edges, {len(self.dirs)} direction relations "
                f"({len(self.unsupported)} unsupported constraints), {len(self.ground_points)} ground points")


def build(sketch: Sketch) -> ConstraintGraph:
    g = ConstraintGraph(sketch)
    pts = sketch.points
    idx = {id(p): i for i, p in enumerate(pts)}
    uf = UnionFind(len(pts))
    for c in sketch.constraints:
        if isinstance(c, C.Coincident):
            uf.union(idx[id(c.p)], idx[id(c.q)])
    lab, n = uf.labels()
    g.members = [[] for _ in range(n)]
    for i, p in enumerate(pts):
        g.point_of[id(p)] = lab[i]
        g.members[lab[i]].append(p)
    for k, ms in enumerate(g.members):
        if any(p.is_fixed for p in ms):
            g.ground_points.append(k)
    for i, ln in enumerate(sketch.lines):
        g.line_of[id(ln)] = i
        g.lines.append(ln)

    # -- radii: known if fixed / Radius / EqualRadius-chained to a known one --
    rounds: list[Circle | Arc] = [*sketch.circles, *sketch.arcs]
    radii: list[Param] = [c.radius for c in rounds]
    ridx = {id(r): i for i, r in enumerate(radii)}
    ruf = UnionFind(len(radii))
    known: dict[int, float] = {}
    for c in sketch.constraints:
        if isinstance(c, C.EqualRadius):
            ruf.union(ridx[id(c.c1.radius)], ridx[id(c.c2.radius)])
        elif isinstance(c, C.Radius):
            known[ruf.find(ridx[id(c.circle.radius)])] = c.r
    for r in radii:
        if r.fixed:
            known.setdefault(ruf.find(ridx[id(r)]), r.value)
    # propagate through unions (a class is known if any member is)
    for r in radii:
        root = ruf.find(ridx[id(r)])
        for r2 in radii:
            if ruf.find(ridx[id(r2)]) == root and root in known:
                g.known_radius[id(r2)] = known[root]

    def rad(circle: Circle | Arc) -> float | None:
        return g.known_radius.get(id(circle.radius))

    for c in sketch.constraints:
        if c.soft or isinstance(c, C.Coincident):
            continue
        if isinstance(c, C.Distance):
            g.edges.append(Edge("PP", g.P(c.p), g.P(c.q), (lambda c=c: c.d), c))
        elif isinstance(c, C.PointOnLine):
            g.edges.append(Edge("PL", g.P(c.p), g.L(c.line), (lambda: 0.0), c))
        elif isinstance(c, C.Horizontal):
            g.dirs.append(DirRelation(g.X_AXIS, g.L(c.line), _branch(c.line, 0.0), c))
        elif isinstance(c, C.Vertical):
            g.dirs.append(DirRelation(g.X_AXIS, g.L(c.line), _branch(c.line, math.pi / 2), c))
        elif isinstance(c, C.Parallel):
            g.dirs.append(DirRelation(g.L(c.l1), g.L(c.l2), _branch2(c.l1, c.l2, 0.0), c))
        elif isinstance(c, C.Perpendicular):
            g.dirs.append(DirRelation(g.L(c.l1), g.L(c.l2), _branch2(c.l1, c.l2, math.pi / 2), c))
        elif isinstance(c, C.Angle):
            g.dirs.append(DirRelation(g.L(c.l1), g.L(c.l2), _branch2(c.l1, c.l2, c.theta), c))
        elif isinstance(c, C.PointOnCircle) and rad(c.circle) is not None:
            g.edges.append(Edge("PP", g.P(c.circle.center), g.P(c.p), (lambda c=c: g.known_radius[id(c.circle.radius)]), c))
        elif isinstance(c, C.TangentLineCircle) and rad(c.circle) is not None:
            # signed distance from centre to line = side·r  (kernel: cross(d, c−a)/|d| − side·r;
            # our line normal n = (−dy, dx)/|d| gives n·(c − a) = cross(d, c−a)/|d|)
            g.edges.append(Edge("PL", g.P(c.circle.center), g.L(c.line),
                                (lambda c=c: c.side * g.known_radius[id(c.circle.radius)]), c))
        elif isinstance(c, C.TangentArcLine) and rad(c.arc) is not None:
            # tangent at endpoint p ⇔ the radius c–p is perpendicular to the line (with p on the
            # line and |c−p| = r from elsewhere).  Modelled with a virtual radius line R through
            # c and p: R ⟂ line — a transversal intersection, not a double root (which would
            # make every fillet merge ill-conditioned).
            pass  # handled below, after passive-line resolution
        elif isinstance(c, C.Radius) or (isinstance(c, C.EqualRadius) and rad(c.c1) is not None):
            pass  # absorbed into known radii
        else:
            g.unsupported.append(c)
    tangents = [c for c in sketch.constraints if isinstance(c, C.TangentArcLine) and rad(c.arc) is not None]

    # -- passive lines: no supported constraint refers to them (their endpoints determine them) --
    used = ({e.b for e in g.edges if e.b.kind == "L"} | {d.a for d in g.dirs} | {d.b for d in g.dirs}
            | {g.L(c.line) for c in tangents})
    active = [ln for i, ln in enumerate(g.lines) if El("L", i) in used]
    g.passive = [ln for i, ln in enumerate(g.lines) if El("L", i) not in used]
    old_idx = {id(ln): i for i, ln in enumerate(g.lines)}
    new_idx = {id(ln): i for i, ln in enumerate(active)}
    remap = {El("L", old_idx[id(ln)]): El("L", new_idx[id(ln)]) for ln in active}
    remap[g.X_AXIS] = g.X_AXIS
    for e in g.edges:
        if e.b.kind == "L":
            e.b = remap[e.b]
    for d in g.dirs:
        d.a, d.b = remap[d.a], remap[d.b]
    g.lines = active
    g.line_of = {id(ln): i for i, ln in enumerate(active)}
    # endpoints lie on their (active) line — implicit incidences
    for ln in active:
        for p in (ln.p1, ln.p2):
            g.edges.append(Edge("PL", g.P(p), g.L(ln), (lambda: 0.0), None))
    # arc-endpoint tangency: virtual radius line R through (centre, endpoint), R ⟂ line
    for c in tangents:
        pt = c.arc.start if c.at == "start" else c.arc.end
        cen, pe = g.P(c.arc.center), g.P(pt)
        R = g.virtual_line(cen, pe)
        g.edges.append(Edge("PL", cen, R, (lambda: 0.0), None))
        g.edges.append(Edge("PL", pe, R, (lambda: 0.0), None))
        g.dirs.append(DirRelation(g.L(c.line), R, _branch_pts(c.line, c.arc.center, pt, math.pi / 2), c))
    return g


# -- line helpers -----------------------------------------------------------

def line_normal(ln: Line) -> tuple[float, float, float]:
    """(nx, ny, c) with n the unit left normal of p1→p2 and n·X = c on the line."""
    dx, dy = ln.direction()
    L = math.hypot(dx, dy) or 1.0
    nx, ny = -dy / L, dx / L
    return nx, ny, nx * ln.p1.x.value + ny * ln.p1.y.value


def _side(p: Point, ln: Line) -> float:
    nx, ny, c = line_normal(ln)
    return 1.0 if nx * p.x.value + ny * p.y.value - c >= 0 else -1.0


def _branch(ln: Line, target: float) -> float:
    """Angle φ (n = rot(φ)·(0,1)) of the branch of `target` (mod π) nearest the line's current direction."""
    nx, ny, _ = line_normal(ln)
    cur = math.atan2(-nx, ny)          # angle of n relative to (0,1)
    return _nearest_mod_pi(cur, target)


def _branch_pts(l1: Line, a: Point, b: Point, target: float) -> float:
    """Branch of the angle from l1's normal to the normal of line a→b nearest the current geometry."""
    n1 = line_normal(l1)
    dx, dy = b.x.value - a.x.value, b.y.value - a.y.value
    L = math.hypot(dx, dy) or 1.0
    n2 = (-dy / L, dx / L)
    cur = math.atan2(n1[0] * n2[1] - n1[1] * n2[0], n1[0] * n2[0] + n1[1] * n2[1])
    return _nearest_mod_pi(cur, target)


def _branch2(l1: Line, l2: Line, target: float) -> float:
    n1 = line_normal(l1)
    n2 = line_normal(l2)
    cur = math.atan2(n1[0] * n2[1] - n1[1] * n2[0], n1[0] * n2[0] + n1[1] * n2[1])
    return _nearest_mod_pi(cur, target)


def _nearest_mod_pi(cur: float, target: float) -> float:
    k = round((cur - target) / math.pi)
    return target + k * math.pi
