"""Stage 5 — homotopy continuation to enumerate the solutions of a small merge system
(a decomposition core or a closed-form triangle): "we can show you the other solutions".

The merge system in the (c, s, tx, ty) parametrisation per movable cluster is polynomial:
shared points, line normals and direction rows are linear in the unknowns, line offsets are
bilinear, and c² + s² = 1 is quadratic.  We square the system with random complex
combinations (linear rows among themselves, degree-2 rows among themselves), keep the linear
part fixed along the path (it is shared by start and target system), and run a total-degree
homotopy on the quadratic rows with the γ-trick:

    H(w, t) = (1 − t)·γ·(w_σ(q)² − 1)  +  t·Q̃(w)     (with Ã w = b̃ throughout)

tracked from the 2^(n_Q) start points by Euler prediction + Newton correction in complex
arithmetic.  Real endpoints (polished on the original system) are the alternatives, sorted
by distance from the current solution (identity: leaves are re-derived from the current
geometry each replay).  Small cores only — the number of paths is exponential in the number
of rotations, which is exactly the cost the decomposition minimises.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.linalg import null_space, qr

from gcs.cgraph import El
from gcs.decompose import Cluster, Plan, Step, _apply, _T, execute, write_point
from gcs.newton import rank_rrqr

CVec = np.ndarray  # complex vectors


@dataclass
class Alternative:
    u: np.ndarray            # transform vector (θ, tx, ty) per movable cluster, relative to the current leaves
    distance: float          # ‖w − w_identity‖: 0 for the root the sketch is on
    location: tuple[float, float] | None = None   # where a requested point element would land

    @property
    def is_current(self) -> bool:
        return self.distance < 1e-6


class _Poly:
    """Merge system F(w) = [A w − b ; Q(w)] in (c, s, tx, ty) per movable cluster:
    A (constant, degree-1 rows) and Q (degree-2 rows: line offsets and c² + s² − 1)."""

    def __init__(self, parts: list[Cluster], step: Step) -> None:
        self.k = len(parts) - 1
        self.n = 4 * self.k
        rows_A: list[np.ndarray] = []
        rhs: list[float] = []
        self.offsets: list[tuple[int, int, El]] = []      # (i, j, line) pairs with a bilinear offset row

        def lin_pose(ci: int, el: El) -> tuple[np.ndarray, np.ndarray]:
            """Affine part of a pose (2 rows): coefficient matrix and constant vector.
            Points contribute both coordinates; lines contribute their normal (the offset
            coordinate is bilinear and lives in Q)."""
            p = np.asarray(parts[ci].els[el], dtype=float)
            M = np.zeros((2, self.n))
            if ci == 0:
                return M, p[:2]
            o = 4 * (ci - 1)
            a, b = (p[0], p[1])
            M[0, o:o + 4] = [a, -b, 1, 0] if el.kind == "P" else [a, -b, 0, 0]
            M[1, o:o + 4] = [b, a, 0, 1] if el.kind == "P" else [b, a, 0, 0]
            return M, np.zeros(2)

        for i, j, e in step.pairs:
            Ai, ci = lin_pose(i, e)
            Aj, cj = lin_pose(j, e)
            rows_A.append(Ai - Aj)
            rhs.append(cj - ci)
            if e.kind != "P":
                self.offsets.append((i, j, e))
        for i, j, la, lb, phi in step.dpairs:            # n_b' − rot(phi) n_a' = 0
            Aa, ca = lin_pose(i, la)
            Ab, cb = lin_pose(j, lb)
            R = np.array([[math.cos(phi), -math.sin(phi)], [math.sin(phi), math.cos(phi)]])
            rows_A.append(Ab - R @ Aa)
            rhs.append(R @ ca - cb)
        self.A = np.vstack(rows_A) if rows_A else np.zeros((0, self.n))
        self.b = np.concatenate(rhs) if rhs else np.zeros(0)
        self.parts = parts
        self.m_q = len(self.offsets) + self.k
        # per-offset constants, hoisted out of the tracking loops
        self._off = [((i, *np.asarray(parts[i].els[e], dtype=float)),
                      (j, *np.asarray(parts[j].els[e], dtype=float))) for i, j, e in self.offsets]

    def _offset(self, w: CVec, data: tuple[int, float, float, float], grad: CVec | None) -> complex:
        """Offset coordinate of a line pose under a cluster's transform; fills `grad` if given."""
        ci, nx, ny, cc = data
        if ci == 0:
            return complex(cc)
        c, s, tx, ty = w[4 * (ci - 1): 4 * ci]
        n0, n1 = c * nx - s * ny, s * nx + c * ny
        if grad is not None:
            o = 4 * (ci - 1)
            grad[o:o + 4] += np.array([nx * tx + ny * ty, -ny * tx + nx * ty, n0, n1])
        return complex(cc + n0 * tx + n1 * ty)

    def QJ(self, w: CVec, want_jac: bool = True) -> tuple[CVec, CVec | None]:
        """Quadratic rows and (optionally) their Jacobian — one pass, since the offset rows
        produce value and gradient together."""
        q_ = np.empty(self.m_q, dtype=complex)
        J = np.zeros((self.m_q, self.n), dtype=complex) if want_jac else None
        n_off = len(self._off)
        for r, (a, b) in enumerate(self._off):
            ga = np.zeros(self.n, dtype=complex) if want_jac else None
            gb = np.zeros(self.n, dtype=complex) if want_jac else None
            q_[r] = self._offset(w, a, ga) - self._offset(w, b, gb)
            if J is not None and ga is not None and gb is not None:
                J[r] = ga - gb
        for q in range(self.k):
            c, s = w[4 * q], w[4 * q + 1]
            q_[n_off + q] = c * c + s * s - 1
            if J is not None:
                J[n_off + q, 4 * q] = 2 * c
                J[n_off + q, 4 * q + 1] = 2 * s
        return q_, J

    def Q(self, w: CVec) -> CVec:
        return self.QJ(w, want_jac=False)[0]

    def F(self, w: CVec) -> CVec:
        return np.concatenate([self.A @ w - self.b, self.Q(w)])

    def J(self, w: CVec) -> CVec:
        jq = self.QJ(w)[1]
        assert jq is not None
        return np.vstack([self.A.astype(complex), jq])


def _w_to_u(w: CVec) -> np.ndarray:
    k = w.size // 4
    u = np.zeros(3 * k)
    for q in range(k):
        c, s, tx, ty = w[4 * q: 4 * q + 4].real
        u[3 * q: 3 * q + 3] = [math.atan2(s, c), tx, ty]
    return u


def enumerate_step(plan: Plan, step_index: int, *, locate: El | None = None, seed: int = 0,
                   max_paths: int = 256, max_steps: int = 400, diverge_rel: float = 50.0) -> list[Alternative]:
    """Real solutions of the merge at `step_index` (the current one first).  Returns [] if the
    merge is not isolated (under-determined) or too large (> max_paths).  `locate` asks where
    that point element would land under each alternative."""
    rng = np.random.default_rng(seed)
    parts = execute(plan, capture=step_index)
    if parts is None or len(parts) < 2:
        return []
    step = plan.steps[step_index]
    P = _Poly(parts, step)
    n, k = P.n, P.k
    w_id = np.tile([1.0, 0.0, 0.0, 0.0], k).astype(complex)   # the current solution: identity
    # -- square the system: Ã w = b̃ (rank r) and n − r combinations of the quadratic rows --
    r = rank_rrqr(P.A, 1e-9) if P.A.size else 0
    n_q = n - r
    if n_q <= 0 or P.m_q < n_q or 2 ** n_q > max_paths:
        return []
    M1 = rng.standard_normal((r, P.A.shape[0])) + 1j * rng.standard_normal((r, P.A.shape[0]))
    M2 = rng.standard_normal((n_q, P.m_q)) + 1j * rng.standard_normal((n_q, P.m_q))
    At, bt = M1 @ P.A, M1 @ P.b
    # -- start system: the same linear rows + w_σ² − 1 on variables free w.r.t. the linear part --
    N = null_space(At) if At.size else np.eye(n, dtype=complex)
    _, _, piv = qr(N.T, pivoting=True)
    sigma = [int(i) for i in piv[:n_q]]
    gamma = np.exp(2j * math.pi * rng.random())

    def G(w: CVec) -> CVec:
        return np.array([w[s] ** 2 - 1 for s in sigma])

    def JG(w: CVec) -> CVec:
        Jg = np.zeros((n_q, n), dtype=complex)
        for q, s in enumerate(sigma):
            Jg[q, s] = 2 * w[s]
        return Jg

    def HJ(w: CVec, t: float) -> tuple[CVec, CVec]:
        """H(w, t) and its Jacobian — P's offset rows give value and gradient in one pass."""
        q_, jq = P.QJ(w)
        assert jq is not None
        return (np.concatenate([At @ w - bt, (1 - t) * gamma * G(w) + t * (M2 @ q_)]),
                np.vstack([At, (1 - t) * gamma * JG(w) + t * (M2 @ jq)]))

    def Ht(w: CVec) -> CVec:
        return np.concatenate([np.zeros(r, dtype=complex), -gamma * G(w) + M2 @ P.Q(w)])

    # start points: every sign pattern on the σ variables; one factorisation, all right-hand sides
    E = np.eye(n, dtype=complex)[sigma]
    signs = np.array([[1.0 if (bits >> q) & 1 else -1.0 for q in range(n_q)] for bits in range(2 ** n_q)],
                     dtype=complex)
    rhs = np.concatenate([np.broadcast_to(bt[:, None], (r, len(signs))), signs.T])
    starts = list(np.linalg.solve(np.vstack([At, E]), rhs).T)

    def newton(w: CVec, t: float, iters: int = 4, tol: float = 1e-10) -> tuple[CVec, bool]:
        for _ in range(iters):
            h, jh = HJ(w, t)
            if np.linalg.norm(h) < tol * (1 + np.linalg.norm(w)):
                return w, True
            try:
                w = w - np.linalg.solve(jh, h)
            except np.linalg.LinAlgError:
                return w, False
        return w, bool(np.linalg.norm(HJ(w, t)[0]) < 1e-6 * (1 + np.linalg.norm(w)))

    # Paths that run off to infinity are dead ends: cut them at a multiple of the sketch scale
    # (w holds cos/sin and translations, so an absolute bound would depend on the sketch size).
    # Tighter is faster but eventually cuts live paths — on the K3,3 core, 50× keeps all four
    # real roots at ~70 % of the cost of a loose bound, 25× is the measured cliff.
    scale = max(1.0, float(np.abs(np.concatenate([P.b, [1.0]])).max()))
    diverge = diverge_rel * scale
    ends: list[CVec] = []
    for w in starts:
        t, dt = 0.0, 0.02
        for _ in range(max_steps):
            if t >= 1.0 or np.linalg.norm(w) > diverge:
                break
            t1 = min(1.0, t + dt)
            try:
                dw = -np.linalg.solve(HJ(w, t)[1], Ht(w))        # Euler predictor
            except np.linalg.LinAlgError:
                dt *= 0.5
                if dt < 1e-10:
                    break
                continue
            w_new, ok = newton(w + dw * (t1 - t), t1)
            if ok and np.linalg.norm(w_new - w) < 0.5 * (1 + np.linalg.norm(w)):
                w, t = w_new, t1
                dt = min(0.2, dt * 1.5)
            else:
                dt *= 0.5
                if dt < 1e-10:
                    break
        if t >= 1.0 and np.linalg.norm(w) <= diverge:
            for _ in range(5):                                   # polish on the original system
                f = P.F(w)
                if np.linalg.norm(f) < 1e-12:
                    break
                w = w - np.linalg.lstsq(P.J(w), f, rcond=None)[0]
            ends.append(w)
    out: list[Alternative] = []
    kept: list[CVec] = []
    q_of = next((qi for qi, c in enumerate(parts[1:]) if locate is not None and locate in c.els), None)
    for w in ends:
        if np.abs(w.imag).max() > 1e-6 * (1 + np.abs(w.real).max()):
            continue
        wr = w.real.astype(complex)
        if np.linalg.norm(P.F(wr)) > 1e-6:
            continue
        if any(np.linalg.norm(wr - k) < 1e-6 for k in kept):
            continue
        kept.append(wr)
        u = _w_to_u(wr)
        loc = None
        if locate is not None and q_of is not None:
            pos = _apply(_T(*u[3 * q_of: 3 * q_of + 3]), locate, parts[q_of + 1].els[locate])
            loc = (float(pos[0]), float(pos[1]))
        out.append(Alternative(u, float(np.linalg.norm(wr - w_id)), loc))
    out.sort(key=lambda a: a.distance)
    return out


def apply_alternative(plan: Plan, step_index: int, alt: Alternative) -> None:
    """Put the sketch on this root: write the alternative placement of the merged clusters
    into the points (leaves are re-derived from geometry, so later replays stay on it), then
    replay the whole plan so dependent geometry follows.  Triangles also flip their branch."""
    parts = execute(plan, capture=step_index)
    if parts is None:
        return
    g = plan.graph
    st = plan.steps[step_index]
    if st.ppp is not None and not alt.is_current and st.branch is not None:
        st.branch = -st.branch
        g.sketch.branches.update(plan.branches())      # document state: survives the next solve
    for q, c in enumerate(parts[1:]):
        T = _T(*alt.u[3 * q: 3 * q + 3])
        for e, pose in c.els.items():
            write_point(g, e, _apply(T, e, pose))
    plan.sticky_branches = True
    execute(plan)
