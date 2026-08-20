"""Stage 5: homotopy enumeration of a construction's real roots, and applying one."""

from __future__ import annotations

import os

import numpy as np
import pytest

from gcs import constraints as C
from gcs import examples
from gcs.decompose import PlanSolver
from gcs.homotopy import apply_alternative, enumerate_step
from gcs.model import Sketch


def test_triangle_has_two_roots_and_the_other_can_be_applied() -> None:
    sk = Sketch()
    a, b = sk.point(0, 0, fixed=True), sk.point(10, 0, fixed=True)
    c = sk.point(5, 4)
    sk.add(C.Distance(a, c, 6), C.Distance(b, c, 6))
    ps = PlanSolver(sk, sticky=True)
    ps.solve()
    idx = next(i for i, st in enumerate(ps.plan.steps) if st.ppp is not None)
    alts = enumerate_step(ps, idx)
    assert len(alts) == 2 and alts[0].is_current and not alts[1].is_current
    apply_alternative(ps, idx, alts[1])
    assert c.y.value < 0
    ps.solve()
    assert c.y.value < 0 and all(k.error() < 1e-6 for k in sk.hard_constraints())


@pytest.mark.skipif(os.environ.get("GCS_SLOW") != "1",
                    reason="~3 s (256 homotopy paths); set GCS_SLOW=1")
def test_k33_core_has_several_real_realizations() -> None:
    sk = examples.k33()
    ps = PlanSolver(sk, sticky=True)
    ps.solve()
    steps = ps.plan.steps
    idx = max(range(len(steps)), key=lambda i: len(steps[i].ids))
    assert len(steps[idx].ids) == 9
    alts = enumerate_step(ps, idx)
    assert len(alts) >= 2 and any(a.is_current for a in alts)
    x0 = sk.get_x()
    apply_alternative(ps, idx, next(a for a in alts if not a.is_current))
    r = ps.solve(fallback=False)
    assert r.success and np.abs(sk.get_x() - x0).max() > 1.0
    assert all(k.error() < 1e-6 for k in sk.hard_constraints())


def test_under_determined_merges_are_skipped() -> None:
    sk = examples.rect_fillets_under()
    ps = PlanSolver(sk, sticky=True)
    ps.solve()
    # a pair merge with a direction (1 movable cluster) is square and isolated: 2 roots at most;
    # whatever the step, enumeration must return a list and never raise
    for i in range(len(ps.plan.steps)):
        assert isinstance(enumerate_step(ps, i, max_paths=16), list)
