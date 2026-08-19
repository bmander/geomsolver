"""Stage 4 — the witness configuration method (Michelucci & Foufou 2006).

Structural analysis (Stage 2) cannot see dependencies that follow from
geometric theorems (three concurrent altitudes, an EqualLength cycle, Pappus…).
A *witness* is a configuration with the sketch's incidence structure but
generic dimensions; the Jacobian there tells the truth about the system:

  * rank deficiency in the rows = dependent constraints (theorem-induced ones
    included) — pivoted QR on Jᵀ picks a maximal independent set and, for each
    leftover equation, the equations it is implied by;
  * the null space of J = the infinitesimal motions = exactly which DOFs remain
    and what they look like (rigid-body motions separated from internal ones,
    modes localised for readability — animate them in the UI).

The user's own sketch is often an adequate witness (it satisfies the incidences
by construction).  We build one by jittering every dimension the constraints
*declare* (`spec` kinds 'length'/'angle') and re-solving from the current
geometry; if that cannot converge (over/conflicting sketch) we satisfy the
incidence-type constraints alone from a perturbed start.

Numerical rank needs care: the rank test is relative (to the largest singular
value / |R₀₀|) and pivoted QR is cross-checked against the SVD, which disagree
only on a near-degenerate witness.  Per-column balancing is *not* needed while
every parameter is a length (coordinates and radii); a future angle- or
scale-valued parameter would need it, and `System` is where that scale vector
would live.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import svd

from gcs.constraints import Constraint
from gcs.model import Param, Sketch, Vec
from gcs.newton import min_norm_lstsq, rrqr
from gcs.solve import System

DIMENSION_KINDS = frozenset({"length", "angle"})


def dimensions(c: Constraint) -> list[tuple[str, str]]:
    """The (attribute, kind) pairs of a constraint's dimension values, from its own declaration."""
    return [(name, kind) for name, kind in c.spec if kind in DIMENSION_KINDS]


@dataclass
class Dependency:
    constraint: Constraint          # a dependent (redundant) equation's constraint
    implied_by: list[Constraint]    # constraints whose equations span it (support of the LS fit)
    theorem: bool                   # structural analysis could not see it (matched in DM)


@dataclass
class Motion:
    """An infinitesimal motion: velocity per free parameter (unit max displacement)."""

    velocity: Vec                   # length n_free
    rigid: bool                     # a rigid-body motion of the whole sketch (translation/rotation)
    params: list[Param]             # free params, aligned with velocity

    def moving_params(self, rel: float = 1e-3) -> list[Param]:
        m = float(np.abs(self.velocity).max()) or 1.0
        return [p for p, v in zip(self.params, self.velocity, strict=True) if abs(v) > rel * m]


@dataclass
class WitnessReport:
    x_witness: Vec                  # the witness configuration (all params)
    used_current: bool              # the sketch itself served as witness (no re-solve needed)
    numeric_rank: int
    dependencies: list[Dependency]
    motions: list[Motion]           # null-space basis: rigid modes first, then internal DOFs
    movable: list[int]              # free-parameter indices that take part in some motion
    warnings: list[str] = field(default_factory=list)

    @property
    def n_dof(self) -> int:
        return len(self.motions)

    @property
    def n_internal_dof(self) -> int:
        return sum(not m.rigid for m in self.motions)

    def summary(self) -> str:
        parts = [f"witness rank {self.numeric_rank}",
                 f"{self.n_dof} DOF ({self.n_internal_dof} internal, {self.n_dof - self.n_internal_dof} rigid-body)"]
        if self.dependencies:
            th = sum(d.theorem for d in self.dependencies)
            parts.append(f"{len(self.dependencies)} dependent constraint(s)" + (f", {th} theorem-type" if th else ""))
        return "; ".join(parts + self.warnings)


# ---------------------------------------------------------------------------

def make_witness(sketch: Sketch, seed: int = 0, jitter: float = 0.05, tol: float = 1e-8) -> Vec:
    """A configuration with the sketch's incidence structure and generic dimensions.
    Leaves the sketch's values and dimensions untouched."""
    x0 = sketch.get_x()
    rng = np.random.default_rng(seed)
    hard = sketch.hard_constraints()
    dimensioned = [(c, name, kind) for c in hard for name, kind in dimensions(c)]
    saved = [(c, name, getattr(c, name)) for c, name, _ in dimensioned]
    saved_c = sketch.constraints
    try:
        # 1. generic dimensions (lengths scaled, angles offset), re-solved from the current geometry
        for c, name, kind in dimensioned:
            v = getattr(c, name)
            setattr(c, name, v * (1 + jitter * rng.standard_normal()) if kind == "length"
                    else v + jitter * rng.standard_normal())
        sketch.constraints = hard
        sys_ = System(sketch)
        res = sys_.solve(max_iter=60)
        if res.success and res.max_residual <= tol * sys_.scale:
            return sketch.get_x()
        # 2. incidences only (always satisfiable) from a perturbed start
        sketch.set_x(x0)
        sketch.constraints = [c for c in hard if not dimensions(c)]
        sketch.perturb(0.02 * max(1.0, sketch.extent()), seed)
        System(sketch).solve(max_iter=60)
        return sketch.get_x()
    finally:
        sketch.constraints = saved_c
        for c, name, v in saved:
            setattr(c, name, v)
        sketch.set_x(x0)


def analyze(sketch: Sketch, x_witness: Vec | None = None, *, system: System | None = None,
            over_ids: frozenset[int] = frozenset(), rtol: float = 1e-9, seed: int = 0) -> WitnessReport:
    """Rank / dependencies / motions of the sketch's constraint system at a witness.

    `over_ids` are the constraints the structural analysis already put in its over-determined
    block; a dependency outside that set is theorem-type — invisible to the graph.  Pass the
    `System` you already compiled for this sketch to avoid a recompile."""
    x0 = sketch.get_x()
    try:
        sys_ = system if system is not None and system.sketch is sketch else System(sketch)
        used_current = x_witness is None and sys_.max_hard_residual() <= 1e-9 * sys_.scale
        if x_witness is None:
            x_witness = x0 if used_current else make_witness(sketch, seed=seed)
        sketch.set_x(x_witness)
        free_params = [sketch.params[i] for i in sys_.free]
        J = sys_.jacobian_dense(sys_.z0())[sys_.hard]
        _, rows_c = sys_.structure()          # row → constraint, in the Jacobian's own row order
        m, n = J.shape
        warnings: list[str] = []
        if m == 0 or n == 0:
            motions = _classify_motions(np.eye(n), free_params, sketch)
            return WitnessReport(x_witness, used_current, 0, [], motions, list(range(n)), warnings)
        # rank: RRQR on Jᵀ (pivots = a maximal independent row set), cross-checked with the SVD
        # that also yields the null space
        rank_qr, piv = rrqr(J.T, rtol)
        _, sv, Vt = svd(J, full_matrices=True)
        rank_svd = int(np.count_nonzero(sv > rtol * sv[0])) if sv.size and sv[0] > 0 else 0
        rank = rank_qr
        if rank_qr != rank_svd:
            warnings.append(f"rank ambiguous: QR {rank_qr} vs SVD {rank_svd} (near-degenerate witness)")
            rank = min(rank_qr, rank_svd)
        # dependent rows: the non-pivot rows, each expressed in the pivot rows' span (one
        # factorisation for all of them)
        indep = list(piv[:rank])
        dep_rows = [r for r in piv[rank:] if rows_c[r] is not None]
        deps: list[Dependency] = []
        if dep_rows:
            coefs, _ = min_norm_lstsq(J[indep].T, J[dep_rows].T)
            for col, r in enumerate(dep_rows):
                c = rows_c[r]
                if any(d.constraint is c for d in deps):
                    continue
                coef = coefs[:, col]
                lim = 1e-6 * (float(np.abs(coef).max()) or 1.0)
                support = (rows_c[indep[k]] for k in np.argsort(-np.abs(coef)) if abs(coef[k]) > lim)
                implied = list(dict.fromkeys(s for s in support if s is not c))
                deps.append(Dependency(c, implied, id(c) not in over_ids))
        N = Vt[rank:].T
        motions = _classify_motions(N, free_params, sketch)
        return WitnessReport(x_witness, used_current, rank, deps, motions, movable_columns(N), warnings)
    finally:
        sketch.set_x(x0)


def movable_columns(N: Vec, rtol: float = 1e-8) -> list[int]:
    """Rows of the null-space basis that are nonzero: the parameters that take part in some
    infinitesimal motion of the configuration."""
    if N.size == 0:
        return []
    w = np.abs(N).max(axis=1)
    return [int(i) for i in np.flatnonzero(w > rtol * w.max())]


def _classify_motions(N: Vec, params: list[Param], sketch: Sketch) -> list[Motion]:
    """Split the null space into rigid-body modes (translations/rotation of everything that
    can move together) and internal DOFs; localise the internal ones (sparse basis)."""
    n, d = N.shape
    if d == 0:
        return []
    # rigid-body generators, from the model's own parameter identity (not from names)
    axis: dict[int, tuple[int, tuple[float, float]]] = {}
    for pt in sketch.points:
        axis[id(pt.x)] = (0, pt.xy)
        axis[id(pt.y)] = (1, pt.xy)
    cx, cy = np.mean([p.xy for p in sketch.points], axis=0) if sketch.points else (0.0, 0.0)
    tx, ty, rot = np.zeros(n), np.zeros(n), np.zeros(n)
    for i, p in enumerate(params):
        got = axis.get(id(p))
        if got is None:
            continue                       # a radius: invariant under rigid motions
        which, (x, y) = got
        (tx if which == 0 else ty)[i] = 1.0
        rot[i] = -(y - cy) if which == 0 else (x - cx)
    # a generator is a rigid mode iff it lies in the null space (N has orthonormal columns)
    rigid = [v / (np.abs(v).max() or 1.0) for v in (tx, ty, rot)
             if np.any(v) and np.linalg.norm(N.T @ v) >= (1 - 1e-6) * np.linalg.norm(v)]
    motions = [Motion(v, True, params) for v in rigid]
    if rigid:                              # internal DOFs = the null space minus the rigid span
        Qr, _ = np.linalg.qr(np.array(rigid).T)
        U, s, _ = np.linalg.svd(N - Qr @ (Qr.T @ N), full_matrices=False)
        Ni = U[:, s > 1e-6]                # N is orthonormal: an absolute threshold is right
    else:
        Ni = N
    if Ni.shape[1]:
        # localise: rotate the basis so each mode is 1 at a pivot parameter and 0 at the others
        k = Ni.shape[1]
        piv = rrqr(Ni.T)[1]
        loc = np.linalg.solve(Ni[piv[:k]].T, Ni.T).T
        motions += [Motion(v / (np.abs(v).max() or 1.0), False, params) for v in loc.T]
    return motions
