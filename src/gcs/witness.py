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
by construction).  We build one by randomising the metric constants and
re-solving from the current geometry; if that cannot converge (over/conflicting
sketch) we satisfy the incidence-type constraints alone from a perturbed start.
Numerical rank needs care: columns are scaled by the sketch extent, the
tolerance is relative, and RRQR is cross-checked against SVD.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import qr, svd

from gcs import constraints as C
from gcs.constraints import Constraint
from gcs.model import Param, Sketch, Vec
from gcs.solve import System

METRIC = (C.Distance, C.Radius, C.Angle)      # constraints carrying a dimension value


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
    structural_rank: int | None     # maximum matching size, if provided (for the theorem flag)
    dependencies: list[Dependency]
    motions: list[Motion]           # null-space basis: rigid modes first, then internal DOFs
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

def make_witness(sketch: Sketch, seed: int = 0, jitter: float = 0.05, tol: float = 1e-8) -> tuple[Vec, bool]:
    """A configuration with the sketch's incidence structure and generic dimensions.
    Returns (x, used_current).  Leaves the sketch's values untouched."""
    x0 = sketch.get_x()
    rng = np.random.default_rng(seed)
    hard = sketch.hard_constraints()
    saved = {c: c.args() for c in hard if isinstance(c, METRIC)}
    saved_c = sketch.constraints
    try:
        # 1. generic dimensions: multiplicative jitter on every metric value, re-solve
        for c in saved:
            if isinstance(c, C.Distance):
                c.d *= 1 + jitter * rng.standard_normal()
            elif isinstance(c, C.Radius):
                c.r *= 1 + jitter * rng.standard_normal()
            elif isinstance(c, C.Angle):
                c.theta += jitter * rng.standard_normal()
        sketch.constraints = hard
        sys_ = System(sketch)
        res = sys_.solve(max_iter=60)
        if res.success and res.max_residual <= tol * sys_.scale:
            xw = sketch.get_x()
            return xw, False
        # 2. incidences only (always satisfiable) from a perturbed start
        sketch.set_x(x0)
        sketch.constraints = [c for c in hard if not isinstance(c, METRIC)]
        x = sketch.get_x()
        free = sketch.free_indices()
        x[free] += 0.02 * max(1.0, sketch.extent()) * rng.standard_normal(len(free))
        sketch.set_x(x)
        System(sketch).solve(max_iter=60)
        return sketch.get_x(), False
    finally:
        sketch.constraints = saved_c
        for c, args in saved.items():
            for (name, _), v in zip(c.spec, args, strict=True):
                setattr(c, name, v)
        sketch.set_x(x0)


def analyze(sketch: Sketch, x_witness: Vec | None = None, *, structural_rank: int | None = None,
            matched_constraints: set[int] | None = None, rtol: float = 1e-9, seed: int = 0) -> WitnessReport:
    """Rank / dependencies / motions of the sketch's constraint system at a witness."""
    x0 = sketch.get_x()
    used_current = x_witness is None
    try:
        if x_witness is None:
            # the current sketch is a witness iff it satisfies its constraints
            s0 = System(sketch)
            if s0.max_hard_residual() <= 1e-9 * s0.scale:
                x_witness = x0
            else:
                x_witness, _ = make_witness(sketch, seed=seed)
                used_current = False
        sketch.set_x(x_witness)
        sys_ = System(sketch)
        free_params = [sketch.params[i] for i in sys_.free]
        z = sys_.z0()
        J = sys_.jacobian_dense(z)[sys_.hard]
        rows_c = [c for c in sys_.constraints if not c.soft for _ in range(c.n_residuals)]
        m, n = J.shape
        warnings: list[str] = []
        if m == 0 or n == 0:
            return WitnessReport(x_witness, used_current, 0, structural_rank, [],
                                 [Motion(np.eye(n)[:, i], False, free_params) for i in range(n)], warnings)
        # column scaling: all params are lengths here (extent) — keeps radii/coords comparable
        scale = max(1.0, sketch.extent())
        Js = J / scale
        # rank: RRQR on Jᵀ (pivots = a maximal independent row set), cross-checked with SVD
        Q, R, piv = qr(Js.T, mode="economic", pivoting=True, check_finite=False)
        d = np.abs(np.diag(R))
        rank_qr = int(np.count_nonzero(d > rtol * d[0])) if d.size and d[0] > 0 else 0
        sv = svd(Js, compute_uv=False)
        rank_svd = int(np.count_nonzero(sv > rtol * sv[0])) if sv.size and sv[0] > 0 else 0
        rank = rank_qr
        if rank_qr != rank_svd:
            warnings.append(f"rank ambiguous: QR {rank_qr} vs SVD {rank_svd} (near-degenerate witness)")
            rank = min(rank_qr, rank_svd)
        # dependent rows: the non-pivot rows; each expressed in the pivot rows' span
        indep = list(piv[:rank])
        dep_rows = list(piv[rank:])
        deps: list[Dependency] = []
        seen: set[int] = set()
        for r in dep_rows:
            c = rows_c[r]
            if id(c) in seen:
                continue
            seen.add(id(c))
            coef, *_ = np.linalg.lstsq(Js[indep].T, Js[r], rcond=None)
            support = [rows_c[indep[k]] for k in np.argsort(-np.abs(coef)) if abs(coef[k]) > 1e-6 * (np.abs(coef).max() or 1)]
            implied = list(dict.fromkeys(s for s in support if s is not c))
            theorem = matched_constraints is None or id(c) in matched_constraints
            deps.append(Dependency(c, implied, theorem))
        # motions: null space of Js (n − rank), rigid-body modes identified & separated
        _, _, Vt = svd(Js, full_matrices=True)
        N = Vt[rank:].T if n > rank else np.zeros((n, 0))
        motions = _classify_motions(N, free_params, sketch)
        if structural_rank is not None and rank < structural_rank:
            warnings.append(f"structural rank {structural_rank} but witness rank {rank}: theorem-type dependency")
        return WitnessReport(x_witness, used_current, rank, structural_rank, deps, motions, warnings)
    finally:
        sketch.set_x(x0)


def _classify_motions(N: Vec, params: list[Param], sketch: Sketch) -> list[Motion]:
    """Split the null space into rigid-body modes (translations/rotation of everything that
    can move together) and internal DOFs; localise the internal ones (sparse basis)."""
    n, d = N.shape
    if d == 0:
        return []
    # rigid-body candidate velocities on the free params
    is_x = np.array([p.name.endswith(".x") for p in params])
    is_y = np.array([p.name.endswith(".y") for p in params])
    xy: dict[int, tuple[float, float]] = {}
    for pt in sketch.points:
        xy[id(pt.x)] = xy[id(pt.y)] = pt.xy
    cx, cy = (np.mean([p.xy for p in sketch.points], axis=0) if sketch.points else (0.0, 0.0))
    tx = is_x.astype(float)
    ty = is_y.astype(float)
    rot = np.zeros(n)
    for i, p in enumerate(params):
        if id(p) in xy:
            x, y = xy[id(p)]
            rot[i] = -(y - cy) if is_x[i] else (x - cx) if is_y[i] else 0.0
    rigid: list[Vec] = []
    P = N @ N.T                                   # projector onto the null space
    for v in (tx, ty, rot):
        if not np.any(v):
            continue
        pv = P @ v
        if np.linalg.norm(pv) > 1e-6 * np.linalg.norm(v) and np.linalg.norm(pv - v) < 1e-6 * np.linalg.norm(v):
            rigid.append(v / (np.abs(v).max() or 1.0))
    motions: list[Motion] = []
    if rigid:
        Rb = np.array(rigid).T
        # orthonormalise rigid modes, then take the internal complement within the null space
        Qr, _ = np.linalg.qr(Rb)
        motions += [Motion(Rb[:, i], True, params) for i in range(Rb.shape[1])]
        Ni = N - Qr @ (Qr.T @ N)
        U, s, _ = np.linalg.svd(Ni, full_matrices=False)
        Ni = U[:, s > 1e-6]                       # N is orthonormal: an absolute threshold is right
    else:
        Ni = N
    # localise: sparse basis by rotating toward coordinate directions (greedy pivoted QR on rows)
    if Ni.shape[1]:
        _, _, piv = qr(Ni.T, mode="economic", pivoting=True, check_finite=False)
        B = np.linalg.solve(Ni[piv[: Ni.shape[1]]], np.eye(Ni.shape[1]))     # rows at pivots become identity
        loc = Ni @ B
        for i in range(loc.shape[1]):
            v = loc[:, i]
            motions.append(Motion(v / (np.abs(v).max() or 1.0), False, params))
    return motions
