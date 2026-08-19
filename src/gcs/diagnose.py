"""Stage 2 — structural constraint diagnosis.

Turns "solver failed" into "these constraints conflict / this entity has 2 DOF":

* Bipartite equations↔free-parameters graph from the compiled System, maximum
  matching (Hopcroft–Karp) → structural rank; Dulmage–Mendelsohn coarse
  decomposition → over-determined (structurally redundant equations),
  under-determined (structurally free parameters) and well-determined parts;
  per connected component DOF bookkeeping.
* (2,3) pebble game on the point-distance subgraph → rigid clusters and
  redundant distances (feeds Stage 3).
* Minimal conflict set (deletion filter) when the solve is infeasible.
* Structural vs numeric rank cross-check: everything above is *structural* and
  cannot see theorem-induced dependencies; when the Jacobian rank is lower
  than the matching we log it — that residue is Stage 4's motivation.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
from typing import Literal

import numpy as np

from gcs import graph
from gcs.constraints import Coincident, Constraint, Distance
from gcs.model import Arc, Circle, Param, Point, Primitive, Sketch, Vec
from gcs.newton import rank_rrqr
from gcs.solve import Method, System

State = Literal["well", "under", "over", "conflict"]
_SEVERITY: dict[str, int] = {"well": 0, "under": 1, "over": 2, "conflict": 3}


@dataclass
class Component:
    """A connected component of the constraint graph with its own DOF accounting."""

    params: list[Param]
    constraints: list[Constraint]
    n_equations: int
    structural_rank: int

    @property
    def dof(self) -> int:
        return len(self.params) - self.structural_rank


@dataclass
class Diagnosis:
    n_params: int                        # free parameters
    n_equations: int                     # hard residual rows
    structural_rank: int                 # maximum matching size
    numeric_rank: int | None             # Jacobian rank at the current configuration (dense path only)
    dof: int                             # structural DOF = n_params − structural_rank
    n_redundant: int                     # structurally redundant equations = n_equations − structural_rank
    over: list[Constraint]               # constraints in the over-determined block (redundancy suspects)
    under_params: list[Param]            # structurally free parameters
    components: list[Component]
    entity_state: dict[int, State]       # id(entity) → state, for UI colouring
    rigid_clusters: list[frozenset[Point]]   # from the pebble game on the distance graph
    redundant_distances: list[Constraint]
    violated: list[Constraint]           # constraints with nonzero residual at the current configuration
    conflicts: list[Constraint] | None   # minimal conflict set (only computed when asked / infeasible)
    warnings: list[str] = field(default_factory=list)

    @property
    def status(self) -> State:
        if self.conflicts or self.violated:
            return "conflict"
        if self.n_redundant:
            return "over"
        return "under" if self.dof > 0 else "well"

    def summary(self) -> str:
        parts = [f"{self.n_params} params, {self.n_equations} equations, structural rank {self.structural_rank}",
                 f"DOF {self.dof}"]
        if self.n_redundant:
            parts.append(f"{self.n_redundant} redundant equation(s) among {len(self.over)} constraint(s)")
        if self.numeric_rank is not None and self.numeric_rank < self.structural_rank:
            parts.append(f"numeric rank {self.numeric_rank} < structural {self.structural_rank}: "
                         f"{self.structural_rank - self.numeric_rank} geometric (theorem-type) dependency")
        if self.conflicts:
            parts.append(f"CONFLICT — remove one of: {', '.join(type(c).__name__ for c in self.conflicts)}")
        elif self.violated:
            parts.append(f"{len(self.violated)} constraint(s) violated")
        if len(self.components) > 1:
            parts.append(f"{len(self.components)} components: DOF " + ", ".join(str(c.dof) for c in self.components))
        if self.rigid_clusters:
            parts.append(f"{len(self.rigid_clusters)} rigid cluster(s) in the distance graph")
        return "; ".join(parts)


# ---------------------------------------------------------------------------

def _structural_graph(sys_: System) -> tuple[list[list[int]], list[Constraint], list[int]]:
    """Rows = hard residual equations, cols = free params.  Returns (adj, row→constraint, row→param-count)."""
    adj: list[list[int]] = []
    row_c: list[Constraint] = []
    for k, cs, gidx, _, _, _ in sys_.blocks:
        cols_all = sys_.col_of[gidx]                       # (n, k) free-column or -1
        for i, c in enumerate(cs):
            if c.soft:
                continue
            cols = sorted({int(j) for j in cols_all[i] if j >= 0})
            for _ in range(k.n_res):
                adj.append(cols)
                row_c.append(c)
    return adj, row_c, [len(a) for a in adj]


def diagnose(sketch: Sketch, *, numeric: bool = True, conflicts: bool | None = None, tol: float = 1e-6) -> Diagnosis:
    """Structural (and optionally numeric) diagnosis of a sketch at its current configuration.

    conflicts=None computes the minimal conflict set only when some constraint is violated."""
    sys_ = System(sketch)
    adj, row_c, _ = _structural_graph(sys_)
    n_cols = sys_.n_free
    dm = graph.dulmage_mendelsohn(adj, n_cols)
    free_params = [sketch.params[i] for i in sys_.free]

    over_c: list[Constraint] = []
    seen: set[int] = set()
    for r in dm.over_rows:
        c = row_c[r]
        if id(c) not in seen:
            seen.add(id(c))
            over_c.append(c)
    under_params = [free_params[j] for j in dm.under_cols]

    # -- components --
    comp_row, comp_col, n_comp = graph.bipartite_components(adj, n_cols)
    comp_params: list[list[Param]] = [[] for _ in range(n_comp)]
    comp_cs: list[list[Constraint]] = [[] for _ in range(n_comp)]
    comp_rows = [0] * n_comp
    comp_rank = [0] * n_comp
    for j, cid in enumerate(comp_col):
        comp_params[cid].append(free_params[j])
        if dm.mate_col[j] >= 0:
            comp_rank[cid] += 1
    seen_c: set[int] = set()
    for r, cid in enumerate(comp_row):
        comp_rows[cid] += 1
        if id(row_c[r]) not in seen_c:
            seen_c.add(id(row_c[r]))
            comp_cs[cid].append(row_c[r])
    components = [Component(comp_params[i], comp_cs[i], comp_rows[i], comp_rank[i]) for i in range(n_comp)]
    components.sort(key=lambda c: -len(c.params))

    # -- numeric cross-check --
    numeric_rank: int | None = None
    warnings: list[str] = []
    if numeric and n_cols and sys_.n_res:
        Jd = sys_.jacobian_dense(sys_.z0())[sys_.hard]
        numeric_rank = graph_rank(Jd)
        if numeric_rank < dm.rank:
            warnings.append(f"structural rank {dm.rank} but numeric rank {numeric_rank}: "
                            "a dependency the graph cannot see (theorem-induced or degenerate configuration) — Stage 4")

    # -- violated / conflicts --
    scale = max(1.0, sketch.extent()) ** 2
    violated = [c for c in sketch.constraints if not c.soft and c.error() > tol * scale]
    conflict_set: list[Constraint] | None = None
    if conflicts or (conflicts is None and violated):
        conflict_set = minimal_conflict_set(sketch, tol=tol)

    # -- pebble game on the point-distance graph --
    clusters, redundant_d = distance_rigidity(sketch)

    # -- entity states --
    under_ids = {id(p) for p in under_params}
    over_ids = {id(c) for c in over_c}
    conflict_ids = {id(c) for c in (conflict_set or violated)}
    state: dict[int, State] = {}
    ents: list[Primitive] = [*sketch.points, *sketch.lines, *sketch.circles, *sketch.arcs]
    touched: dict[int, list[Constraint]] = defaultdict(list)
    for c in sketch.constraints:
        if c.soft:
            continue
        for e in c.entities():
            touched[id(e)].append(c)
            for ch in e.children:
                touched[id(ch)].append(c)
    for e in ents:
        own = _own_params(e)
        st: State = "well"
        if any(id(c) in conflict_ids for c in touched[id(e)]):
            st = "conflict"
        elif any(id(c) in over_ids for c in touched[id(e)]):
            st = "over"
        elif any(id(p) in under_ids for p in own):
            st = "under"
        state[id(e)] = st
    # a line/circle/arc inherits the most severe state of its points
    for e in ents:
        for ch in e.children:
            if _SEVERITY[state[id(ch)]] > _SEVERITY[state[id(e)]]:
                state[id(e)] = state[id(ch)]

    return Diagnosis(n_cols, len(adj), dm.rank, numeric_rank, n_cols - dm.rank, dm.n_redundant, over_c,
                     under_params, components, state, clusters, redundant_d, violated, conflict_set, warnings)


def _own_params(e: Primitive) -> list[Param]:
    if isinstance(e, Point):
        return [e.x, e.y]
    if isinstance(e, (Circle, Arc)):
        return [e.radius]
    return []


def graph_rank(J: Vec, rcond: float = 1e-10) -> int:
    return rank_rrqr(np.ascontiguousarray(J), rcond)


# ---------------------------------------------------------------------------

def minimal_conflict_set(sketch: Sketch, candidates: list[Constraint] | None = None,
                         tol: float = 1e-6, method: Method = "dogleg") -> list[Constraint]:
    """Deletion filter: drop constraints one at a time, keeping a drop whenever the
    rest is still infeasible.  What remains is a *minimal* infeasible subset (not
    necessarily minimum) — "remove one of these".  Each step is one solve from
    the current geometry, so ~1 ms per constraint at sketch sizes.  Returns []
    if the full system is feasible."""
    x0 = sketch.get_x()
    hard = [c for c in sketch.constraints if not c.soft]
    cands = [c for c in (candidates or hard) if not c.soft]
    others = [c for c in hard if c not in cands]
    scale = max(1.0, sketch.extent()) ** 2

    def feasible(cs: list[Constraint]) -> bool:
        sketch.set_x(x0)
        saved = sketch.constraints
        sketch.constraints = cs
        try:
            System(sketch).solve(method=method, max_iter=60)
            return all(c.error() <= tol * scale for c in cs)
        finally:
            sketch.constraints = saved
    try:
        if feasible(others + cands):
            return []
        keep = list(cands)
        for c in cands:
            trial = [k for k in keep if k is not c]
            if not feasible(others + trial):
                keep = trial          # still infeasible without c → c is not needed for the conflict
        return keep
    finally:
        sketch.set_x(x0)


# ---------------------------------------------------------------------------

def distance_rigidity(sketch: Sketch) -> tuple[list[frozenset[Point]], list[Constraint]]:
    """(2,3) pebble game on the point-distance graph: vertices are points with
    Coincident points merged; edges are Distance constraints.  Returns rigid
    clusters (as sets of Points) and the redundant Distance constraints."""
    pts = sketch.points
    idx = {id(p): i for i, p in enumerate(pts)}
    parent = list(range(len(pts)))

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for c in sketch.constraints:
        if isinstance(c, Coincident):
            a, b = find(idx[id(c.p)]), find(idx[id(c.q)])
            if a != b:
                parent[b] = a
    verts: dict[int, int] = {}
    for i in range(len(pts)):
        verts.setdefault(find(i), len(verts))
    edges: list[tuple[int, int]] = []
    edge_c: list[Constraint] = []
    for c in sketch.constraints:
        if isinstance(c, Distance):
            edges.append((verts[find(idx[id(c.p)])], verts[find(idx[id(c.q)])]))
            edge_c.append(c)
    if not edges:
        return [], []
    res = graph.pebble_game(len(verts), edges)
    members: dict[int, list[Point]] = defaultdict(list)
    for i, p in enumerate(pts):
        members[verts[find(i)]].append(p)
    clusters = [frozenset(p for v in comp for p in members[v]) for comp in res.components]
    red_set = set()
    redundant: list[Constraint] = []
    for (u, v), c in zip(edges, edge_c, strict=True):
        if (u, v) in res.redundant and (u, v) not in red_set:
            red_set.add((u, v))
            redundant.append(c)
    return clusters, redundant
