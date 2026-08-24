"""Stage 4: witness configuration method — dependencies (theorem-type included) and motions."""

from __future__ import annotations

import numpy as np

from gcs import constraints as C
from gcs import examples
from gcs.diagnose import diagnose
from gcs.witness import analyze, dimensions, make_witness


def test_stage2_residue_is_diagnosed_with_culprit() -> None:
    sk = examples.polygon_chain(8)
    d = diagnose(sk, witness=True)
    rep = d.witness
    assert rep is not None
    assert d.structural_rank == 24 and rep.numeric_rank == 23
    assert len(rep.dependencies) == 1
    dep = rep.dependencies[0]
    assert isinstance(dep.constraint, C.EqualLength) and dep.theorem
    assert all(isinstance(c, (C.EqualLength, C.Coincident)) for c in dep.implied_by)
    assert dep.implied_by
    assert rep.n_dof == 7 and rep.n_internal_dof == 7          # one point fixed: no rigid modes


def test_concurrent_altitudes_theorem() -> None:
    sk = examples.altitudes()
    d = diagnose(sk, witness=True)
    rep = d.witness
    assert rep is not None
    assert d.structural_rank == 6 and rep.numeric_rank == 5   # the graph is blind to it
    assert len(rep.dependencies) == 1 and rep.dependencies[0].theorem
    assert isinstance(rep.dependencies[0].constraint, C.PointOnLine)
    assert any(isinstance(c, C.Perpendicular) for c in rep.dependencies[0].implied_by)
    assert rep.n_internal_dof == 3                             # the feet slide along the altitudes
    assert not rep.used_current                                # P did not satisfy the incidences


def test_witness_of_well_constrained_is_current_and_full_rank() -> None:
    for name in ("rect_fillets", "slotted_link", "truss"):
        sk = examples.EXAMPLES[name]()
        rep = analyze(sk)
        assert rep.used_current and rep.n_dof == 0 and not rep.dependencies, name


def test_rigid_body_modes_are_separated() -> None:
    sk = examples.truss_floating(4)
    rep = analyze(sk)
    assert rep.n_dof == 3 and rep.n_internal_dof == 0
    assert all(m.rigid for m in rep.motions)


def test_motions_are_localised_and_unit_scaled() -> None:
    sk = examples.rect_fillets_under()
    rep = analyze(sk)
    assert rep.n_dof == 1 and rep.n_internal_dof == 1
    m = rep.motions[0]
    assert np.abs(m.velocity).max() == 1.0
    assert {p.name for p in m.moving} == {"b2.x", "r1.x", "r2.x", "t1.x", "c_br.x", "c_tr.x"}


def test_make_witness_restores_sketch_and_generalises_dimensions() -> None:
    sk = examples.rect_fillets()
    x0 = sk.get_x()
    dims = [c.d for c in sk.constraints if isinstance(c, C.Distance)]
    xw = make_witness(sk, seed=1)
    assert np.array_equal(sk.get_x(), x0)
    assert [c.d for c in sk.constraints if isinstance(c, C.Distance)] == dims
    assert not np.allclose(xw, x0)                             # generic dimensions moved it
    rep = analyze(sk, xw)
    assert rep.numeric_rank == 26 and rep.n_dof == 0


def test_reported_dependencies_are_genuinely_redundant() -> None:
    """The row→constraint map must follow the Jacobian's own row order (kernel blocks), not sketch
    order: every named dependency must be removable without changing the rank."""
    from gcs.solve import RANK_TOL, System

    for sk in (examples.polygon_chain(8), examples.altitudes(), examples.rect_fillets()):
        rep = analyze(sk)
        sk.set_x(rep.x_witness)
        s = System(sk)
        # the matrix the core judged, at the tolerance it judged it — not a raw Jacobian
        J = s.conditioned()
        _, rows_c = s.structure()
        full = np.linalg.matrix_rank(J, tol=RANK_TOL)
        for dep in rep.dependencies:
            keep = [i for i, c in enumerate(rows_c) if c is not dep.constraint]
            assert np.linalg.matrix_rank(J[keep], tol=RANK_TOL) == full, \
                f"{dep.constraint} is not actually redundant"
            assert dep.constraint not in dep.implied_by
        s.dispose()


def test_dimension_jitter_follows_the_constraint_declarations() -> None:
    """make_witness generalises every value a constraint declares as a dimension (spec kinds
    'length'/'angle') — a new dimensioned constraint type needs no change there."""
    sk = examples.rect_fillets()
    dimensioned = [c for c in sk.hard_constraints() if dimensions(c)]
    assert {type(c).__name__ for c in dimensioned} == {"Distance", "Radius"}
    assert all(dimensions(c)[0][1] in ("length", "angle") for c in dimensioned)
    before = [(c, n, getattr(c, n)) for c in dimensioned for n, _ in dimensions(c)]
    xw = make_witness(sk, seed=3)
    assert not np.allclose(xw, sk.get_x())                     # the witness is a different shape
    assert all(getattr(c, n) == v for c, n, v in before)       # dimensions restored exactly
