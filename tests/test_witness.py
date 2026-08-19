"""Stage 4: witness configuration method — dependencies (theorem-type included) and motions."""

import numpy as np

from gcs import constraints as C
from gcs import examples
from gcs.diagnose import diagnose
from gcs.model import Sketch
from gcs.witness import analyze, make_witness


altitudes_sketch = examples.altitudes


def test_stage2_residue_is_diagnosed_with_culprit() -> None:
    sk = examples.polygon_chain(8)
    d = diagnose(sk, witness=True)
    rep = d.witness
    assert rep is not None
    assert d.structural_rank == 24 and rep.numeric_rank == 23
    assert len(rep.dependencies) == 1
    dep = rep.dependencies[0]
    assert isinstance(dep.constraint, C.EqualLength) and dep.theorem
    assert all(isinstance(c, (C.EqualLength, C.Coincident)) for c in dep.implied_by) and dep.implied_by
    assert rep.n_dof == 7 and rep.n_internal_dof == 7          # one point fixed: no rigid modes


def test_concurrent_altitudes_theorem() -> None:
    sk = altitudes_sketch()
    d = diagnose(sk, witness=True)
    rep = d.witness
    assert rep is not None
    assert d.structural_rank == 6 and rep.numeric_rank == 5   # the graph is blind to the concurrency
    assert len(rep.dependencies) == 1 and rep.dependencies[0].theorem
    assert isinstance(rep.dependencies[0].constraint, C.PointOnLine)
    assert any(isinstance(c, C.Perpendicular) for c in rep.dependencies[0].implied_by)
    assert rep.n_internal_dof == 3                             # the three feet slide along their altitudes
    assert not rep.used_current                                # P did not satisfy the incidences: a witness was built


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
    assert {p.name for p in m.moving_params()} == {"b2.x", "r1.x", "r2.x", "t1.x", "c_br.x", "c_tr.x"}


def test_make_witness_restores_sketch_and_generalises_dimensions() -> None:
    sk = examples.rect_fillets()
    x0 = sk.get_x()
    dims = [c.d for c in sk.constraints if isinstance(c, C.Distance)]
    xw, used = make_witness(sk, seed=1)
    assert np.array_equal(sk.get_x(), x0) and [c.d for c in sk.constraints if isinstance(c, C.Distance)] == dims
    assert not used and not np.allclose(xw, x0)                # generic dimensions moved it
    rep = analyze(sk, xw)
    assert rep.numeric_rank == 26 and rep.n_dof == 0
