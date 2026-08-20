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

import numpy as np
from gcs import graph
from gcs.constraints import Constraint, Distance
from gcs.model import Param, Point, Primitive, Sketch, Vec
from gcs.newton import rank_and_nullspace
from gcs.solve import Method, System
from gcs.witness import WitnessReport, analyze, movable_columns

State = Literal["well", "under", "over", "conflict"]
_SEVERITY: dict[str, int] = {"well": 0, "under": 1, "over": 2, "conflict": 3}
NUMERIC_MAX = 300   # free params up to which the automatic numeric cross-check runs (dense SVD)


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
    numeric_skipped: bool                # the cross-check was skipped: past the dense limit
    over: list[Constraint]               # redundancy suspects: the structural over-determined block,
    #                                      plus the rows sharing a numeric-only dependency
    under_params: list[Param]            # parameters that can move: numeric (Jacobian null space) when
    #                                      available, else structural (DM under-block, which is generous)
    structural_under_params: list[Param]
    components: list[Component]
    entity_state: dict[int, State]       # id(entity) → state, for UI colouring
    rigid_clusters: list[frozenset[Point]]   # from the pebble game on the distance graph
    redundant_distances: list[Constraint]
    violated: list[Constraint]           # constraints with nonzero residual at the current configuration
    conflicts: list[Constraint] | None   # minimal conflict set (only computed when asked / infeasible)
    warnings: list[str] = field(default_factory=list)
    witness: WitnessReport | None = None  # Stage 4 analysis (on demand): dependencies + motions

    @property
    def effective_rank(self) -> int:
        """The rank the solver actually sees.  The structural matching is a *generic* upper
        bound; when the numeric cross-check ran, it is the truth at this configuration."""
        return self.structural_rank if self.numeric_rank is None else self.numeric_rank

    @property
    def dof(self) -> int:
        """Degrees of freedom left at the current configuration — what can still be dragged.

        Uses the numeric rank when the cross-check ran, because a matching cannot tell that
        two equations say the same thing: it counts them both and calls the sketch rigid while
        the geometry moves freely.  `structural_dof` is the matching's generous answer.
        """
        return self.n_params - self.effective_rank

    @property
    def structural_dof(self) -> int:
        """DOF the matching alone sees — an upper bound on the rank, so a lower bound on DOF."""
        return self.n_params - self.structural_rank

    @property
    def n_redundant(self) -> int:
        """Equations beyond the rank — the ones carrying no information.  Numeric when the
        cross-check ran, for the same reason as `dof`: a matching counts two equations that say
        the same thing as two."""
        return self.n_equations - self.effective_rank

    @property
    def structural_n_redundant(self) -> int:
        """What the matching alone sees."""
        return self.n_equations - self.structural_rank

    @property
    def geometric_dependency(self) -> int:
        """Dependencies only the numbers can see (0 when the cross-check did not run)."""
        return 0 if self.numeric_rank is None else max(0, self.structural_rank - self.numeric_rank)

    @property
    def status(self) -> State:
        """`over` only when something is actually worth removing.  A rank deficiency alone is
        not enough: a dependency can be shared between a user constraint that is still doing
        work and a primitive's own definition, and there is nothing to delete.  `n_redundant`
        still counts it, and the summary still says so."""
        if self.conflicts or self.violated:
            return "conflict"
        if self.over:
            return "over"
        return "under" if self.dof > 0 else "well"

    def summary(self) -> str:
        parts = [f"{self.n_params} params, {self.n_equations} equations, structural rank {self.structural_rank}",
                 f"DOF {self.dof}"]
        if self.dof != self.structural_dof:
            parts.append(f"the matching alone would say DOF {self.structural_dof} — "
                         f"{self.geometric_dependency} equation(s) carry no information")
        if self.n_redundant:
            parts.append(f"{self.n_redundant} redundant equation(s) among {len(self.over)} constraint(s)")
        if self.geometric_dependency:
            parts.append(f"numeric rank {self.numeric_rank} < structural {self.structural_rank}: "
                         f"{self.geometric_dependency} geometric (theorem-type) dependency")
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

def removable_constraints(W: Vec, row_c: list[Constraint], rtol: float = 1e-8) -> list[Constraint]:
    """Constraints that could be deleted without losing any information, given `W`, a basis of
    the left null space of the Jacobian (one column per dependency).

    With W orthonormal, dropping a set of rows R gives
        rank(J minus R) = rank(J) − |R| + rank(W[R]),
    so a constraint is free to delete exactly when its own rows are *independent* in W.  That
    distinction is the whole point: an arc whose endpoints are mirrored about a line through
    its centre makes one of `Symmetric`'s two residuals implied by the arc's radius equations,
    but `Symmetric` still carries the perpendicularity — its two rows are dependent in W, it is
    doing real work, and telling the user to remove it would be wrong.  Only a constraint that
    is *wholly* implied is worth naming.

    Intrinsic constraints are skipped: they come with the primitive and there is no way to
    delete one, so naming them is noise.
    """
    if W.size == 0:
        return []
    rows: dict[int, tuple[Constraint, list[int]]] = {}
    for r, c in enumerate(row_c):
        rows.setdefault(id(c), (c, []))[1].append(r)
    out = []
    for c, rs in rows.values():
        sub = np.asarray(W)[rs]
        if c.intrinsic or not np.abs(sub).max() > rtol:
            continue
        if int(np.linalg.matrix_rank(sub, tol=rtol)) == len(rs):
            out.append(c)
    return out


def _tol_abs(sketch: Sketch, tol: float) -> float:
    return tol * max(1.0, sketch.extent()) ** 2


def violated_constraints(sys_: System, tol: float = 1e-6) -> list[Constraint]:
    """Hard constraints whose residual is not (numerically) zero at the current configuration."""
    lim = _tol_abs(sys_.sketch, tol)
    err = sys_.constraint_errors()
    return [c for c in sys_.constraints if not c.soft and err[id(c)] > lim]


def diagnose(sketch: Sketch, *, system: System | None = None, numeric: bool | None = None,
             conflicts: bool | None = None, witness: bool = False, tol: float = 1e-6,
             numeric_max: int = NUMERIC_MAX) -> Diagnosis:
    """Structural (and optionally numeric) diagnosis of a sketch at its current configuration.

    Pass the `System` you just solved with to avoid a recompile.  conflicts=None
    computes the minimal conflict set only when some constraint is violated.
    witness=True adds the Stage-4 witness analysis (dependent constraints with what
    implies them, and the remaining motions).

    numeric=None (the default) runs the Jacobian rank / null-space cross-check only while the
    system is small enough for a dense SVD (`numeric_max` free parameters); above that the
    diagnosis stays structural — which is what Stage 2 is for — and says so in `warnings`.
    Diagnosis runs after every edit, and one dense SVD of a 1000-entity sketch costs more than
    every other step put together.  Pass numeric=True to force it."""
    sys_ = system if system is not None and system.sketch is sketch else System(sketch)
    adj, row_c = sys_.structure()
    n_cols = sys_.n_free
    dm = graph.dulmage_mendelsohn(adj, n_cols)
    free_params = [sketch.params[i] for i in sys_.free]

    over_c = list(dict.fromkeys(row_c[r] for r in dm.over_rows))
    over_ids = frozenset(id(c) for c in over_c)     # the structurally redundant block
    structural_under = [free_params[j] for j in dm.under_cols]
    under_params = structural_under

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

    # -- witness analysis (Stage 4), on demand --
    warnings: list[str] = []
    wit: WitnessReport | None = None
    if witness and n_cols and sys_.n_res:
        wit = analyze(sketch, system=sys_, over_ids=over_ids)

    # -- numeric cross-check: rank and the parameters that can actually move --
    numeric_rank: int | None = None
    want_numeric = (n_cols <= numeric_max) if numeric is None else numeric
    numeric_skipped = numeric is None and not want_numeric
    if numeric_skipped:
        warnings.append(f"numeric cross-check skipped: {n_cols} free parameters is above the dense limit "
                        f"({numeric_max}) — the diagnosis below is structural only")
    if want_numeric and n_cols and sys_.n_res:
        if wit is not None and wit.used_current:
            numeric_rank, movable = wit.numeric_rank, wit.movable      # same J at the same x
        else:
            _, N, _ = rank_and_nullspace(sys_.jacobian_dense(sys_.z0())[sys_.hard])
            numeric_rank, movable = n_cols - N.shape[1], movable_columns(N)
        # Which parameters can actually move: rows of the null space that are nonzero.
        # Sharper than the DM under-block (which counts a parameter as free if it *could*
        # be in some generic assignment); evaluated at the current configuration, so a
        # degenerate placement can still fool it — the witness step generalises this.
        under_params = [free_params[j] for j in movable]
        if numeric_rank < dm.rank:
            warnings.append(f"structural rank {dm.rank} but numeric rank {numeric_rank}: "
                            "a dependency the graph cannot see (theorem-induced or degenerate configuration) — Stage 4")
            # ...and name the constraints worth removing, or the report would say
            # "over-constrained" with nothing to point at.  One extra SVD, only on this path:
            # a healthy sketch never reaches it.
            _, W, _ = rank_and_nullspace(sys_.jacobian_dense(sys_.z0())[sys_.hard].T)
            over_c = list(dict.fromkeys([*over_c, *removable_constraints(W, row_c)]))


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

    if wit is not None:
        warnings += wit.warnings
    return Diagnosis(n_cols, len(adj), dm.rank, numeric_rank, numeric_skipped, over_c, under_params,
                     structural_under, components, state, clusters, redundant_d, violated, conflict_set,
                     warnings, wit)


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
    from gcs.cgraph import coincident_classes

    vert_of, members = coincident_classes(sketch)
    edge_c = [c for c in sketch.constraints if isinstance(c, Distance)]
    if not edge_c:
        return [], []
    edges = [(vert_of[id(c.p)], vert_of[id(c.q)]) for c in edge_c]
    res = graph.pebble_game(len(members), edges)
    clusters = [frozenset(p for v in comp for p in members[v]) for comp in res.components]
    return clusters, [edge_c[i] for i in res.redundant]
