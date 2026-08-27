"""Stage 2 — structural constraint diagnosis.

Matching / Dulmage–Mendelsohn on the equations-vs-parameters graph, the (2,3) pebble game on the
point-distance subgraph, minimal conflict sets, and the structural-vs-numeric rank cross-check.
All of it runs in the core; this module presents the report in Python terms.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal, Sequence

from gcs import _ffi
from gcs._ffi import lib
from gcs.constraints import Constraint
from gcs.model import Param, Point, Primitive, Sketch
from gcs.solve import System
from gcs.witness import WitnessReport, report_from

State = Literal["well", "under", "over", "conflict"]

#: Free parameters up to which the automatic numeric cross-check (a dense SVD) runs.
NUMERIC_MAX = 300


@dataclass
class Component:
    """A connected component of the constraint graph with its own DOF accounting."""

    params: list[Param]
    constraints: list[Constraint]
    structural_rank: int
    dof: int


@dataclass
class Diagnosis:
    n_params: int                       # free parameters
    n_equations: int                    # hard residual rows
    structural_rank: int                # maximum matching size
    numeric_rank: int | None            # Jacobian rank at the current configuration
    numeric_skipped: bool
    geometric_dependency: int           # dependencies only the numbers can see
    shaky: int                          # motions blocked at second order: rigid, not DOF
    over: list[Constraint]              # "remove one of these"
    implied: list[Constraint]           # implied by a relation-only theorem: consistent, no fix
    claims_theorem: list[Constraint]    # `claim …` statements: hold, and add no rank
    claims_violated: list[Constraint]   # do not hold at this solution
    claims_consuming: list[Constraint]  # hold only by the pose; enforcing one would take a DOF
    under_params: list[Param]           # what can move at the configuration diagnosed
    structural_under_params: list[Param]
    components: list[Component]
    entity_state: dict[Primitive, State]
    rigid_clusters: list[list[Point]]   # from the pebble game on the distance graph
    redundant_distances: list[Constraint]
    violated: list[Constraint]
    conflicts: list[Constraint] | None  # minimal conflict set
    warnings: list[str]
    witness: WitnessReport | None       # Stage 4 analysis, on demand
    dof: int
    structural_dof: int
    n_redundant: int
    structural_n_redundant: int
    status: State
    summary: str = field(default="")

    def __repr__(self) -> str:
        return f"Diagnosis({self.summary})"


def _from_json(sk: Sketch, d: dict[str, Any]) -> Diagnosis:
    sk._sync_constraints()

    def con(i: int) -> Constraint:
        c = sk.constraint_by_id(i)
        assert c is not None, i
        return c

    def cons(v: Sequence[int]) -> list[Constraint]:
        return [con(i) for i in v]

    def prm(v: Sequence[int]) -> list[Param]:
        return [sk.param_at(i) for i in v]

    ents = {(e.kind, e.index): e for e in sk.primitives()}
    return Diagnosis(
        n_params=d["nParams"], n_equations=d["nEquations"], structural_rank=d["structuralRank"],
        numeric_rank=d["numericRank"], numeric_skipped=bool(d["numericSkipped"]),
        geometric_dependency=d["geometricDependency"],
        shaky=d["shaky"],
        over=cons(d["over"]),
        implied=cons(d["implied"]),
        claims_theorem=cons(d["claimsTheorem"]),
        claims_violated=cons(d["claimsViolated"]),
        claims_consuming=cons(d["claimsConsuming"]),
        under_params=prm(d["underParams"]),
        structural_under_params=prm(d["structuralUnderParams"]),
        components=[Component(prm(c["params"]), cons(c["constraints"]),
                              c["structuralRank"], c["dof"]) for c in d["components"]],
        entity_state={ents[(k, i)]: s for k, i, s in d["entityState"]},
        rigid_clusters=[[sk.points[i] for i in c] for c in d["rigidClusters"]],
        redundant_distances=cons(d["redundantDistances"]),
        violated=cons(d["violated"]),
        conflicts=None if d["conflicts"] is None else cons(d["conflicts"]),
        warnings=list(d["warnings"]),
        witness=None if d["witness"] is None else report_from(sk, d["witness"]),
        dof=d["dof"], structural_dof=d["structuralDof"], n_redundant=d["nRedundant"],
        structural_n_redundant=d["structuralNRedundant"], status=d["status"],
        summary=d["summary"],
    )


def diagnose(sketch: Sketch, system: System | None = None, numeric: bool | None = None,
             conflicts: bool | None = None, witness: bool = False, tol: float = 1e-6,
             numeric_max: int = NUMERIC_MAX) -> Diagnosis:
    """Structural (and optionally numeric) diagnosis at the sketch's current configuration.

    Pass the `System` you just solved with to avoid a recompile.  `conflicts` left None computes
    the minimal conflict set only when some constraint is violated.  `numeric` left None runs the
    Jacobian rank / null-space cross-check only while the system is small enough for a dense SVD.
    """
    opts = {"numeric": numeric, "conflicts": conflicts, "witness": witness,
            "tol": tol, "numericMax": numeric_max}
    p, n = _ffi.send_json(opts)
    if system is not None and system.sketch is sketch:
        d = _ffi.take_json(lib.gcs_diagnose_with_json(sketch._h, system._h, p, n))
    else:
        d = _ffi.take_json(lib.gcs_diagnose_json(sketch._h, p, n))
    return _from_json(sketch, d)


def summary(d: Diagnosis) -> str:
    return d.summary


def violated_constraints(sys_: System, tol: float = 1e-6) -> list[Constraint]:
    """Hard constraints whose residual is not (numerically) zero at the current configuration."""
    sk = sys_.sketch
    ids = _ffi.take_json(lib.gcs_violated_json(sk._h, tol)) or []
    sk._sync_constraints()
    return [c for c in (sk.constraint_by_id(i) for i in ids) if c is not None]


def minimal_conflict_set(sketch: Sketch, candidates: Sequence[Constraint] | None = None,
                         tol: float = 1e-6) -> list[Constraint]:
    """Minimal infeasible subset among `candidates` (default: all hard constraints).
    "Remove one of these." """
    payload = None if candidates is None else [c._id for c in candidates]
    p, n = _ffi.send_json(payload)
    ids = _ffi.take_json(lib.gcs_minimal_conflict_set_json(sketch._h, p, n, tol)) or []
    sketch._sync_constraints()
    return [c for c in (sketch.constraint_by_id(i) for i in ids) if c is not None]


def distance_rigidity(sketch: Sketch) -> tuple[list[list[Point]], list[Constraint]]:
    """(2,3) pebble game on the point-distance graph: the rigid clusters and the redundant
    Distance constraints."""
    d = _ffi.take_json(lib.gcs_distance_rigidity_json(sketch._h))
    sketch._sync_constraints()
    pts = sketch.points
    clusters = [[pts[i] for i in c] for c in d["clusters"]]
    red = [c for c in (sketch.constraint_by_id(i) for i in d["redundant"]) if c is not None]
    return clusters, red


__all__ = ["Component", "Diagnosis", "NUMERIC_MAX", "State", "diagnose", "distance_rigidity",
           "minimal_conflict_set", "summary", "violated_constraints"]
