"""Stage 2 — structural constraint diagnosis.

Turns "solver failed" into "these constraints conflict / this entity has 2 DOF":

* Bipartite equations↔free-parameters graph (`System.structure()`), maximum
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

from gcs import graph
from gcs.constraints import Coincident, Constraint, Distance
from gcs.model import Param, Point, Primitive, Sketch, Vec
from gcs.solve import Method, System

State = Literal["well", "under", "over", "conflict"]
_SEVERITY: dict[str, int] = {"well": 0, "under": 1, "over": 2, "conflict": 3}


@dataclass
class Component:
    """A connected component of the constraint graph with its own DOF accounting."""

    params: list[Param]
    constraints: list[Constraint]
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
    def dof(self) -> int:
        """Structural DOF (rigid-body motions included unless something is fixed)."""
        return self.n_params - self.structural_rank

    @property
    def n_redundant(self) -> int:
        """Structurally redundant equations."""
        return self.n_equations - self.structural_rank

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

def _tol_abs(sketch: Sketch, tol: float) -> float:
    return tol * max(1.0, sketch.extent()) ** 2


def violated_constraints(sys_: System, tol: float = 1e-6) -> list[Constraint]:
    """Hard constraints whose residual is not (numerically) zero at the current configuration."""
    lim = _tol_abs(sys_.sketch, tol)
    err = sys_.constraint_errors()
    return [c for c in sys_.constraints if not c.soft and err[id(c)] > lim]


def diagnose(sketch: Sketch, *, system: System | None = None, numeric: bool = True,
             conflicts: bool | None = None, tol: float = 1e-6) -> Diagnosis:
    """Structural (and optionally numeric) diagnosis of a sketch at its current configuration.

    Pass the `System` you just solved with to avoid a recompile.  conflicts=None
    computes the minimal conflict set only when some constraint is violated."""
    sys_ = system if system is not None and system.sketch is sketch else System(sketch)
    adj, row_c = sys_.structure()
    n_cols = sys_.n_free
    dm = graph.dulmage_mendelsohn(adj, n_cols)
    free_params = [sketch.params[i] for i in sys_.free]

    over_c = list(dict.fromkeys(row_c[r] for r in dm.over_rows))
    under_params = [free_params[j] for j in dm.under_cols]

    # -- components --
    comp_row, comp_col, n_comp = graph.bipartite_components(adj, n_cols)
    comp_params: list[list[Param]] = [[] for _ in range(n_comp)]
    comp_cs: list[dict[Constraint, None]] = [{} for _ in range(n_comp)]
    comp_rank = [0] * n_comp
    for j, cid in enumerate(comp_col):
        comp_params[cid].append(free_params[j])
        comp_rank[cid] += dm.mate_col[j] >= 0
    for r, cid in enumerate(comp_row):
        comp_cs[cid][row_c[r]] = None
    components = [Component(comp_params[i], list(comp_cs[i]), comp_rank[i]) for i in range(n_comp)]
    components.sort(key=lambda c: -len(c.params))

    # -- numeric cross-check --
    numeric_rank: int | None = None
    warnings: list[str] = []
    if numeric and n_cols and sys_.n_res:
        numeric_rank = sys_.rank(hard_only=True)
        if numeric_rank < dm.rank:
            warnings.append(f"structural rank {dm.rank} but numeric rank {numeric_rank}: "
                            "a dependency the graph cannot see (theorem-induced or degenerate configuration) — Stage 4")

    # -- violated / conflicts --
    violated = violated_constraints(sys_, tol)
    conflict_set: list[Constraint] | None = None
    if conflicts or (conflicts is None and violated):
        # Candidates = the structurally over-determined block (where a redundancy must
        # live); if the graph sees nothing wrong (e.g. triangle inequality) fall back to
        # the violated constraints.  Everything else stays fixed, so the result is
        # minimal "among the suspects" — and the filter costs |candidates| solves, not |all|.
        conflict_set = minimal_conflict_set(sketch, over_c or violated, tol=tol)

    # -- pebble game on the point-distance graph --
    clusters, redundant_d = distance_rigidity(sketch)

    # -- entity states: own params under? touching constraints over/conflicting? then
    #    a line/circle/arc inherits the most severe state of its points --
    under_ids = {id(p) for p in under_params}
    over_ids = {id(c) for c in over_c}
    conflict_ids = {id(c) for c in (conflict_set or violated)}
    touched: dict[int, list[Constraint]] = defaultdict(list)
    for c in sketch.hard_constraints():
        for e in c.entities():
            touched[id(e)].append(c)
            for ch in e.children:
                touched[id(ch)].append(c)
    ents: list[Primitive] = [*sketch.points, *sketch.lines, *sketch.circles, *sketch.arcs]
    state: dict[int, State] = {}
    for e in ents:
        st: State = "well"
        if any(id(c) in conflict_ids for c in touched[id(e)]):
            st = "conflict"
        elif any(id(c) in over_ids for c in touched[id(e)]):
            st = "over"
        elif any(id(p) in under_ids for p in e.params):
            st = "under"
        state[id(e)] = st
    for e in ents:
        for ch in e.children:
            if _SEVERITY[state[id(ch)]] > _SEVERITY[state[id(e)]]:
                state[id(e)] = state[id(ch)]

    return Diagnosis(n_cols, len(adj), dm.rank, numeric_rank, over_c, under_params, components, state,
                     clusters, redundant_d, violated, conflict_set, warnings)


# ---------------------------------------------------------------------------

def minimal_conflict_set(sketch: Sketch, candidates: list[Constraint] | None = None,
                         tol: float = 1e-6, method: Method = "dogleg", max_iter: int = 60) -> list[Constraint]:
    """Minimal infeasible subset among `candidates` (default: all hard constraints);
    non-candidates stay in the system throughout.  "Remove one of these."

    Grow-then-shrink: add candidates one at a time, each solve warm-started from
    the previous *feasible* configuration, until one breaks feasibility (it is in
    the conflict); then delete the earlier ones one at a time, keeping a deletion
    whenever the rest is still infeasible.  Warm-starting from feasible states is
    what makes the trials reliable — after a failed solve the sketch geometry can
    be far from anything, and a trial solve from there may stall and masquerade
    as "infeasible".  Returns [] if the system is feasible."""
    x0 = sketch.get_x()
    hard = sketch.hard_constraints()
    cands = [c for c in (hard if candidates is None else candidates) if not c.soft]
    others = [c for c in hard if c not in cands]
    lim = _tol_abs(sketch, tol)
    saved = sketch.constraints

    def solve_with(cs: list[Constraint], x_start: Vec) -> tuple[bool, Vec]:
        sketch.set_x(x_start)
        sketch.constraints = cs
        try:
            ok = System(sketch).solve(method=method, max_iter=max_iter).max_residual <= lim
            return ok, sketch.get_x()
        finally:
            sketch.constraints = saved
    try:
        ok, x_base = solve_with(others, x0)      # a state satisfying the non-candidates
        if not ok:
            x_base = x0
        # grow
        accepted: list[Constraint] = []
        x_feas = x_base
        culprit: Constraint | None = None
        for c in cands:
            ok, x = solve_with(others + accepted + [c], x_feas)
            if ok:
                accepted.append(c)
                x_feas = x
            else:
                culprit = c
                break
        if culprit is None:
            return []
        # shrink: which of the accepted ones are needed to make `culprit` infeasible?
        keep = list(accepted)
        for c in accepted:
            trial = [k for k in keep if k is not c]
            ok, _ = solve_with(others + trial + [culprit], x_feas)
            if not ok:
                keep = trial
        return keep + [culprit]
    finally:
        sketch.set_x(x0)


# ---------------------------------------------------------------------------

def distance_rigidity(sketch: Sketch) -> tuple[list[frozenset[Point]], list[Constraint]]:
    """(2,3) pebble game on the point-distance graph: vertices are points with
    Coincident points merged; edges are Distance constraints.  Returns rigid
    clusters (as sets of Points) and the redundant Distance constraints."""
    pts = sketch.points
    idx = {id(p): i for i, p in enumerate(pts)}
    uf = graph.UnionFind(len(pts))
    for c in sketch.constraints:
        if isinstance(c, Coincident):
            uf.union(idx[id(c.p)], idx[id(c.q)])
    vert, n_vert = uf.labels()
    edge_c = [c for c in sketch.constraints if isinstance(c, Distance)]
    if not edge_c:
        return [], []
    edges = [(vert[idx[id(c.p)]], vert[idx[id(c.q)]]) for c in edge_c]
    res = graph.pebble_game(n_vert, edges)
    members: dict[int, list[Point]] = defaultdict(list)
    for i, p in enumerate(pts):
        members[vert[i]].append(p)
    clusters = [frozenset(p for v in comp for p in members[v]) for comp in res.components]
    return clusters, [edge_c[i] for i in res.redundant]
