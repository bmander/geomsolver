"""Stage 1: vectorized kernels + our own DogLeg / LM solvers."""

import numpy as np
import pytest

from gcs import constraints as C
from gcs import examples, newton, solve
from gcs.kernels import KERNELS
from gcs.model import Sketch
from gcs.solve import METHODS, System
from tests.test_jacobians import all_constraints


def test_every_constraint_type_has_a_kernel() -> None:
    kernels = {type(c).kernel.name for c in all_constraints(0)}
    assert kernels == {k.name for k in KERNELS}


def test_vectorized_kernel_matches_scalar_rows() -> None:
    """Stacking n constraints of a type and evaluating once must equal n scalar evaluations."""
    rng = np.random.default_rng(1)
    for c in all_constraints(2):
        k = c.kernel
        V = rng.uniform(-5, 5, (7, k.n_par))
        Kc = np.repeat(c.consts()[None, :], 7, axis=0)
        R, J = k.res(V, Kc), k.jac(V, Kc)
        assert R.shape == (7, k.n_res) and J.shape == (7, k.n_res, k.n_par)
        for i in range(7):
            np.testing.assert_allclose(R[i], c.residual(V[i]), rtol=1e-12, atol=1e-12)
            np.testing.assert_allclose(J[i], c.jacobian(V[i]), rtol=1e-12, atol=1e-12)


def test_system_blocks_cover_every_constraint_once() -> None:
    sk = examples.rect_fillets()
    s = System(sk)
    assert sum(len(b.constraints) for b in s.blocks) == len(sk.constraints)
    assert s.n_res == sk.n_residuals()
    # residual rows are contiguous per constraint at row_of
    x = s.z0()
    r = s.residuals(x)
    for c in sk.constraints:
        off = s.row_of[id(c)]
        np.testing.assert_allclose(r[off : off + c.n_residuals], c.residual(c.local_values()))


def test_sparse_and_dense_jacobians_agree() -> None:
    sk = examples.truss(6)
    examples.perturb(sk, 1.0)
    s = System(sk)
    z = s.z0()
    np.testing.assert_allclose(s.jacobian(z).toarray(), s.jacobian_dense(z))


@pytest.mark.parametrize("method", ["dogleg", "lm"])
@pytest.mark.parametrize("name", list(examples.EXAMPLES))
def test_own_solvers_converge(name: str, method: str) -> None:
    sk = examples.EXAMPLES[name]()
    examples.perturb(sk, 3.0, seed=7)
    res = solve(sk, method=method)
    assert res.success, res
    assert res.max_residual < 1e-8
    assert res.iterations <= 30


def test_dogleg_min_norm_step_gives_least_change() -> None:
    sk = Sketch()
    p, q = sk.point(0, 0), sk.point(12, 0)
    sk.add(C.Distance(p, q, 10))
    res = solve(sk, method="dogleg")
    assert res.success
    assert p.x.value == pytest.approx(1.0, abs=1e-6)
    assert q.x.value == pytest.approx(11.0, abs=1e-6)


def test_rank_reported_on_dense_path() -> None:
    sk = examples.rect_fillets()
    examples.perturb(sk, 1.0)
    res = solve(sk)
    assert res.rank == sk.n_residuals()  # fully constrained: full row rank
    assert System(sk).rank() == sk.n_residuals()
    sk = examples.polygon_chain(8)
    assert System(sk).rank() < len(sk.free_indices())  # under-constrained: rank-deficient columns
    # ...and one redundant equation: the cycle e0=e1=...=e7=e0 of EqualLength closes on itself
    assert System(sk).rank() == sk.n_residuals() - 1


def test_sparse_path_matches_dense_solution() -> None:
    sk1, sk2 = examples.truss(30), examples.truss(30)
    examples.perturb(sk1, 1.0, seed=3)
    examples.perturb(sk2, 1.0, seed=3)
    r1 = System(sk1).solve(dense=True)
    r2 = System(sk2).solve(dense=False)
    assert r1.success and r2.success
    np.testing.assert_allclose(sk1.get_x(), sk2.get_x(), atol=1e-6)


def test_update_consts_moves_drag_target_without_recompile() -> None:
    sk = examples.polygon_chain(6)
    p = sk.lines[2].p1
    tgt = C.DragTarget(p, *p.xy)
    sk.add(tgt)
    s = System(sk)
    tgt.set_target(p.xy[0] + 3, p.xy[1] + 4)
    s.update_consts(tgt)
    res = s.solve()
    assert res.success
    assert p.xy == pytest.approx((tgt.tx, tgt.ty), abs=1e-6)


def test_lm_and_dogleg_agree_on_conflicting_soft_target() -> None:
    """Fully constrained sketch + soft target: both converge to a stationary point of ½‖r‖²."""
    sk = examples.rect_fillets()
    p = sk.lines[0].p2
    sk.add(C.DragTarget(p, p.xy[0] + 5, p.xy[1] + 5))
    s = System(sk)
    z0 = s.z0()
    z1, i1 = newton.dogleg(s.residuals, s.jacobian_dense, z0, ftol=1e-20, max_iter=50)
    z2, i2 = newton.levenberg_marquardt(s.residuals, s.jacobian_dense, z0, ftol=1e-20, max_iter=100)
    g1 = s.jacobian_dense(z1).T @ s.residuals(z1)
    g2 = s.jacobian_dense(z2).T @ s.residuals(z2)
    assert np.abs(g1).max() < 1e-6 and np.abs(g2).max() < 1e-6


def test_all_methods_listed_work() -> None:
    for m in METHODS:
        sk = examples.slotted_link()
        examples.perturb(sk, 1.0)
        assert solve(sk, method=m).success, m
