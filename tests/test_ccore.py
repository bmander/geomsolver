"""The C core (csrc/) checked against the Python reference implementation.

Build with `make -C . build/libgcs.dylib` (see Makefile); the tests skip if it is absent.
Everything the WASM front end relies on is compared here against gcs.* directly:
kernels, the compiled plan's residuals/Jacobian/CSR structure, the dense linear algebra
and both solvers.
"""

from __future__ import annotations

import ctypes as ct
import platform
from pathlib import Path

import numpy as np
import pytest

from gcs import examples
from gcs import newton
from gcs.kernels import KERNELS
from gcs.newton import min_norm_lstsq, rank_and_nullspace, rank_rrqr
from gcs.model import Sketch
from gcs.solve import System

_EXT = {"Darwin": ".dylib", "Windows": ".dll"}.get(platform.system(), ".so")
_LIB = Path(__file__).resolve().parents[1] / "build" / f"libgcs{_EXT}"
pytestmark = pytest.mark.skipif(not _LIB.exists(), reason="C core not built (run make)")

F64 = ct.POINTER(ct.c_double)
I32 = ct.POINTER(ct.c_int32)


class Info(ct.Structure):
    _fields_ = [("status", ct.c_int), ("nfev", ct.c_int), ("njev", ct.c_int),
                ("iterations", ct.c_int), ("rank", ct.c_int)]


def _lib() -> ct.CDLL:
    lib = ct.CDLL(str(_LIB))
    lib.gcs_min_norm_lstsq.argtypes = [ct.c_int] * 3 + [F64, F64, ct.c_double, F64]
    lib.gcs_rrqr.argtypes = [ct.c_int, ct.c_int, F64, ct.c_double, I32]
    lib.gcs_kernel_info.argtypes = [ct.c_int, I32]
    lib.gcs_svd.argtypes = [ct.c_int, ct.c_int, F64, F64, F64, F64]
    lib.gcs_rank_nullspace.argtypes = [ct.c_int, ct.c_int, F64, ct.c_double, F64, F64]
    lib.gcs_lu_solve.argtypes = [ct.c_int, F64, F64]
    lib.gcs_system_new.argtypes = [ct.c_int, F64, ct.c_int, I32, ct.c_int, I32, I32, I32, F64, I32]
    lib.gcs_system_new.restype = ct.c_void_p
    for name in ("gcs_system_free", "gcs_system_n_res", "gcs_system_n_free", "gcs_system_nnz"):
        getattr(lib, name).argtypes = [ct.c_void_p]
    lib.gcs_system_residuals.argtypes = [ct.c_void_p, F64, F64]
    lib.gcs_system_jacobian_dense.argtypes = [ct.c_void_p, F64, F64]
    lib.gcs_system_csr_indptr.argtypes = [ct.c_void_p]
    lib.gcs_system_csr_indptr.restype = I32
    lib.gcs_system_csr_indices.argtypes = [ct.c_void_p]
    lib.gcs_system_csr_indices.restype = I32
    lib.gcs_system_solve.argtypes = [ct.c_void_p, ct.c_int, ct.c_double, ct.c_double, ct.c_double,
                                     ct.c_int, ct.c_int, ct.c_int, F64, ct.POINTER(Info)]
    lib.gcs_system_max_hard_residual.argtypes = [ct.c_void_p, F64]
    lib.gcs_system_max_hard_residual.restype = ct.c_double
    return lib


LIB = _lib() if _LIB.exists() else None


def p64(a: np.ndarray) -> ct.Array:
    return a.ctypes.data_as(F64)


def p32(a: np.ndarray) -> ct.Array:
    return a.ctypes.data_as(I32)


def plan_of(sys_: System) -> dict[str, np.ndarray]:
    """The compiled Python plan as the flat arrays gcs_system_new takes."""
    kid = np.array([KERNELS.index(b.kernel) for b in sys_.blocks], dtype=np.int32)
    cnt = np.array([len(b.constraints) for b in sys_.blocks], dtype=np.int32)
    gidx = np.concatenate([b.gidx.ravel() for b in sys_.blocks]).astype(np.int32) if sys_.blocks else np.zeros(0, np.int32)
    consts = (np.concatenate([b.consts.ravel() for b in sys_.blocks]).astype(np.float64)
              if sys_.blocks else np.zeros(0))
    soft = np.array([c.soft for b in sys_.blocks for c in b.constraints], dtype=np.int32)
    return {"kid": kid, "cnt": cnt, "gidx": gidx, "consts": consts, "soft": soft}


def csystem(sys_: System) -> tuple[int, dict[str, np.ndarray]]:
    d = plan_of(sys_)
    x0 = sys_.sketch.get_x()
    free = sys_.free.astype(np.int32)
    h = LIB.gcs_system_new(len(x0), p64(x0), len(free), p32(free), len(d["kid"]),
                           p32(d["kid"]), p32(d["cnt"]), p32(d["gidx"]), p64(d["consts"]), p32(d["soft"]))
    return h, d


CASES = ["rect_fillets", "slotted_link", "truss", "polygon_chain"]


@pytest.mark.parametrize("m,n", [(12, 5), (5, 12), (8, 8), (30, 17), (4, 9), (1, 3), (3, 1)])
def test_min_norm_lstsq_matches_lapack(m: int, n: int) -> None:
    rng = np.random.default_rng(m * 100 + n)
    for rank_def in (0, 1, 2):
        A = rng.standard_normal((m, n))
        if rank_def and min(m, n) > rank_def:      # force a rank deficiency
            A[:, -rank_def:] = A[:, :rank_def] * 2.0
        b = rng.standard_normal(m)
        want, want_rank = min_norm_lstsq(A.copy(), b.copy())
        X = np.zeros(n)
        got_rank = LIB.gcs_min_norm_lstsq(m, n, 1, p64(np.ascontiguousarray(A.copy())),
                                          p64(np.ascontiguousarray(b.copy().reshape(m, 1))), 1e-12, p64(X))
        assert got_rank == want_rank
        assert np.allclose(A @ X, A @ want, atol=1e-9)          # same least-squares fit
        assert np.linalg.norm(X) <= np.linalg.norm(want) + 1e-9  # and minimum norm


def test_min_norm_lstsq_multiple_rhs() -> None:
    rng = np.random.default_rng(7)
    m, n, k = 14, 9, 3
    A = rng.standard_normal((m, n))
    A[:, 8] = A[:, 0]
    B = rng.standard_normal((m, k))
    want, _ = min_norm_lstsq(A.copy(), B.copy())
    X = np.zeros((n, k))
    LIB.gcs_min_norm_lstsq(m, n, k, p64(np.ascontiguousarray(A.copy())),
                           p64(np.ascontiguousarray(B.copy())), 1e-12, p64(X))
    assert np.allclose(A @ X, A @ want, atol=1e-9)


@pytest.mark.parametrize("seed", range(6))
def test_rrqr_rank(seed: int) -> None:
    rng = np.random.default_rng(seed)
    m, n = 20, 13
    A = rng.standard_normal((m, n))
    A[:, 5] = A[:, 1] + A[:, 2]
    A[:, 9] = A[:, 3]
    piv = np.zeros(n, dtype=np.int32)
    got = LIB.gcs_rrqr(m, n, p64(np.ascontiguousarray(A.copy())), 1e-10, p32(piv))
    assert got == rank_rrqr(A.copy(), 1e-10) == 11
    assert sorted(piv.tolist()) == list(range(n))


@pytest.mark.parametrize("name", CASES)
def test_system_residuals_and_jacobian(name: str) -> None:
    sk = examples.EXAMPLES[name]()
    sk.perturb(0.7, seed=3)
    sys_ = System(sk)
    h, _ = csystem(sys_)
    try:
        z = sys_.z0()
        r = np.zeros(sys_.n_res)
        LIB.gcs_system_residuals(h, p64(z), p64(r))
        assert np.allclose(r, sys_.residuals(z), atol=1e-12)
        J = np.zeros((sys_.n_res, sys_.n_free))
        LIB.gcs_system_jacobian_dense(h, p64(z), p64(J))
        assert np.allclose(J, sys_.jacobian_dense(z), atol=1e-12)
        assert LIB.gcs_system_nnz(h) == sys_._nnz
        indptr = np.ctypeslib.as_array(LIB.gcs_system_csr_indptr(h), (sys_.n_res + 1,))
        indices = np.ctypeslib.as_array(LIB.gcs_system_csr_indices(h), (sys_._nnz,))
        assert np.array_equal(indptr, sys_._csr_indptr)
        assert np.array_equal(indices, sys_._csr_indices)
    finally:
        LIB.gcs_system_free(h)


@pytest.mark.parametrize("name", CASES)
@pytest.mark.parametrize("method,dense", [(0, 1), (1, 1), (0, 0)])
def test_solve_converges(name: str, method: int, dense: int) -> None:
    sk = examples.EXAMPLES[name]()
    sk.perturb(0.5, seed=1)
    sys_ = System(sk)
    h, _ = csystem(sys_)
    try:
        z = sys_.z0()
        info = Info()
        LIB.gcs_system_solve(h, method, 1e-14 * sys_.scale, 1e-12, 1e-16 * sys_.scale,
                             100, 0, dense, p64(z), ct.byref(info))
        mx = LIB.gcs_system_max_hard_residual(h, p64(z))
        assert mx < 1e-6 * sys_.scale, f"{name}: max|r| = {mx}"
    finally:
        LIB.gcs_system_free(h)


def test_solve_matches_python_solution() -> None:
    """Same sketch, same warm start: C and Python must land on the same configuration."""
    sk = examples.truss(6)
    sk.perturb(0.4, seed=11)
    sys_ = System(sk)
    h, _ = csystem(sys_)
    try:
        z = sys_.z0()
        info = Info()
        LIB.gcs_system_solve(h, 0, 1e-14 * sys_.scale, 1e-12, 1e-16 * sys_.scale, 100, 0, 1,
                             p64(z), ct.byref(info))
        res = sys_.solve()
        assert res.success
        assert np.allclose(z, sys_.z0(), atol=1e-6)
        assert info.status == res.status        # the stop-reason numbering is shared
        assert info.status in newton.STATUS
    finally:
        LIB.gcs_system_free(h)


def test_sparse_path_on_a_big_truss() -> None:
    sk = examples.truss(40)
    sk.perturb(0.3, seed=2)
    sys_ = System(sk)
    assert sys_.n_free > 120
    h, _ = csystem(sys_)
    try:
        z = sys_.z0()
        info = Info()
        LIB.gcs_system_solve(h, 0, 1e-14 * sys_.scale, 1e-12, 1e-16 * sys_.scale, 200, 0, 0,
                             p64(z), ct.byref(info))
        assert LIB.gcs_system_max_hard_residual(h, p64(z)) < 1e-6 * sys_.scale
    finally:
        LIB.gcs_system_free(h)


@pytest.mark.parametrize("m,n,rank", [(9, 5, 5), (5, 9, 5), (7, 7, 7), (40, 6, 6),
                                      (60, 40, 40), (40, 60, 35), (50, 50, 45), (100, 30, 12), (3, 9, 3)])
def test_svd_reconstructs_and_gives_the_null_space(m: int, n: int, rank: int) -> None:
    """Singular values, orthonormal factors and reconstruction against numpy, including the
    rank-deficient cases the null space work depends on."""
    rng = np.random.default_rng(m * 31 + n)
    A = rng.standard_normal((m, rank)) @ rng.standard_normal((rank, n))
    S = np.zeros(min(m, n))
    Vt = np.zeros((n, n))
    U = np.zeros((m, min(m, n)))
    assert LIB.gcs_svd(m, n, p64(np.ascontiguousarray(A)), p64(U), p64(S), p64(Vt)) == 0
    want = np.linalg.svd(A, compute_uv=False)
    assert np.allclose(S, want, atol=1e-9)
    assert np.allclose(Vt @ Vt.T, np.eye(n), atol=1e-9)
    assert np.allclose(U @ np.diag(S) @ Vt[: min(m, n)], A, atol=1e-8)
    k = int(np.count_nonzero(S > 1e-9 * S[0]))
    assert k == min(rank, m, n)
    assert np.allclose(A @ Vt[k:].T, 0, atol=1e-8)      # the trailing rows of Vt span the null space


def test_rank_nullspace_of_a_sketch_jacobian() -> None:
    sk = examples.rect_fillets_under()
    sys_ = System(sk)
    J = sys_.jacobian_dense(sys_.z0())[sys_.hard]
    m, n = J.shape
    N = np.zeros((n, n))
    S = np.zeros(n)
    rank = LIB.gcs_rank_nullspace(m, n, p64(np.ascontiguousarray(J)), 1e-10, p64(N), p64(S))
    want_rank, want_N, _ = rank_and_nullspace(J)
    assert rank == want_rank
    nn = n - rank
    assert nn == want_N.shape[1] == 1
    assert np.allclose(J @ N[:, :nn], 0, atol=1e-8)


def test_kernel_metadata_matches_the_c_registry() -> None:
    """The kernel ids and their arities are the ABI between gcs.kernels and csrc/kernels.c;
    adding a kernel to one and not the other must fail here rather than silently misalign a
    compiled plan."""
    out = np.zeros(4, dtype=np.int32)
    assert LIB.gcs_kernel_count() == len(KERNELS)
    for kid, k in enumerate(KERNELS):
        LIB.gcs_kernel_info(kid, p32(out))
        assert (int(out[0]), int(out[1]), int(out[2])) == (k.n_res, k.n_par, k.n_const), k.name
        assert bool(out[3]) == (k.const_jac is not None), k.name


def test_every_constraint_type_matches_the_c_kernel() -> None:
    """Every constraint type, not just the ones the example sketches happen to use: one
    single-constraint system per type, residuals and Jacobian compared with the Python
    kernel at a random configuration."""
    from tests.test_jacobians import all_constraints

    seen = set()
    for c in all_constraints(7):
        seen.add(c.kernel.name)
        sk = Sketch()
        sk.params = [p for p in dict.fromkeys(c.params)]      # only what this constraint touches
        for i, prm in enumerate(sk.params):
            prm.index = i
        sk.constraints = [c]
        sys_ = System(sk)
        h, _ = csystem(sys_)
        try:
            z = sys_.z0()
            r = np.zeros(sys_.n_res)
            LIB.gcs_system_residuals(h, p64(z), p64(r))
            assert np.allclose(r, sys_.residuals(z), atol=1e-12), c
            J = np.zeros((sys_.n_res, sys_.n_free))
            LIB.gcs_system_jacobian_dense(h, p64(z), p64(J))
            assert np.allclose(J, sys_.jacobian_dense(z), atol=1e-12), c
        finally:
            LIB.gcs_system_free(h)
    assert seen == {k.name for k in KERNELS}, "a kernel has no constraint exercising it"
