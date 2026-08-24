"""Stage 4 — the witness configuration method (Michelucci & Foufou 2006).

Structural analysis cannot see dependencies that follow from geometric theorems; a witness is a
configuration with the sketch's incidence structure but generic dimensions, and the Jacobian there
tells the truth.  The analysis runs in the core; this module presents its report.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from gcs import _ffi
from gcs._ffi import Vec, lib
from gcs.constraints import Constraint
from gcs.model import Param, Sketch


@dataclass
class Dependency:
    constraint: Constraint          # a dependent (redundant) equation's constraint
    implied_by: list[Constraint]    # constraints whose equations span it
    theorem: bool                   # structural analysis could not see it

    def __repr__(self) -> str:
        return f"Dependency({self.constraint!r}, theorem={self.theorem})"


@dataclass
class Motion:
    """An infinitesimal motion: velocity per free parameter, scaled to unit max displacement."""

    velocity: Vec
    rigid: bool                     # a rigid-body motion of the whole sketch
    #: The Params this motion actually moves — the core's own reading of its velocities, since
    #: which of them count as moving is a fact about the analysis and not about how a caller
    #: chooses to print it.
    moving: list[Param]


@dataclass
class WitnessReport:
    x_witness: Vec
    used_current: bool              # the sketch itself served as witness
    numeric_rank: int
    dependencies: list[Dependency]
    motions: list[Motion]           # rigid modes first, then internal DOFs
    movable: list[int]              # free-parameter indices taking part in some motion
    warnings: list[str] = field(default_factory=list)
    summary: str = ""

    @property
    def n_dof(self) -> int:
        return len(self.motions)

    @property
    def n_internal_dof(self) -> int:
        return sum(1 for m in self.motions if not m.rigid)


def report_from(sk: Sketch, d: dict[str, Any]) -> WitnessReport:
    sk._sync_constraints()

    def con(i: int) -> Constraint:
        c = sk.constraint_by_id(i)
        assert c is not None
        return c

    return WitnessReport(
        x_witness=np.array(d["xWitness"], dtype=np.float64),
        used_current=bool(d["usedCurrent"]),
        numeric_rank=int(d["numericRank"]),
        dependencies=[
            Dependency(con(x["constraint"]), [con(i) for i in x["impliedBy"]], bool(x["theorem"]))
            for x in d["dependencies"]
        ],
        motions=[
            Motion(np.array(m["velocity"], dtype=np.float64), bool(m["rigid"]),
                   [sk.param_at(int(i)) for i in m["movingParams"]])
            for m in d["motions"]
        ],
        movable=[int(i) for i in d["movable"]],
        warnings=list(d["warnings"]),
        summary=str(d["summary"]),
    )


def analyze(sk: Sketch, x_witness: Any = None, seed: int = 0) -> WitnessReport:
    """Rank, dependencies and motions of the sketch's constraint system at a witness."""
    if x_witness is not None:
        x0 = sk.get_x()
        sk.set_x(x_witness)
        try:
            d = _ffi.take_json(lib.gcs_witness_json(sk._h, seed & 0xFFFFFFFF))
        finally:
            sk.set_x(x0)
        rep = report_from(sk, d)
        rep.x_witness = np.array(x_witness, dtype=np.float64)
        return rep
    return report_from(sk, _ffi.take_json(lib.gcs_witness_json(sk._h, seed & 0xFFFFFFFF)))


def make_witness(sk: Sketch, seed: int = 0) -> Vec:
    """A configuration with the sketch's incidence structure and generic dimensions.  Leaves the
    sketch's values and dimensions untouched."""
    out = _ffi.f64(len(sk.params))
    lib.gcs_make_witness(sk._h, seed & 0xFFFFFFFF, _ffi.pf(out))
    return out


def dimensions(c: Constraint) -> list[tuple[str, str]]:
    """The (attribute, kind) pairs of a constraint's dimension values."""
    return c.dimensions()


__all__ = ["Dependency", "Motion", "WitnessReport", "analyze", "dimensions", "make_witness",
           "report_from"]
