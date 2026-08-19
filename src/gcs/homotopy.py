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
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from scipy.linalg import qr

from gcs.cgraph import El
from gcs.decompose import Cluster, Plan, Step, _apply, _T, execute

CVec = np.ndarray  # complex vectors


@dataclass
class Alternative:
    u: np.ndarray            # transform vector (θ, tx, ty) per movable cluster, relative to the current leaves
    distance: float          # ‖w − w_identity‖
    is_current: bool


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
            """Affine part of the pose: rows × (n + 1) [coefficients | constant] for the linear rows."""
            p = np.asarray(parts[ci].els[el], dtype=float)
            m = 2                                          # point: 2 rows; line: 2 normal rows (offset separate)
            M = np.zeros((m, self.n + 1))
            if ci == 0:
                M[:, -1] = p[:2]
                return M[:, :-1], M[:, -1]
            o = 4 * (ci - 1)
            if el.kind == "P":
                x, y = p
                M[0, o:o + 4] = [x, -y, 1, 0]
                M[1, o:o + 4] = [y, x, 0, 1]
            else:
                nx, ny = p[0], p[1]
                M[0, o:o + 4] = [nx, -ny, 0, 0]
                M[1, o:o + 4] = [ny, nx, 0, 0]
            return M[:, :-1], M[:, -1]

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

    def _offset(self, w: CVec, ci: int, el: El) -> tuple[complex, CVec]:
        """Offset coordinate of a line pose under cluster ci's transform, with its gradient."""
        p = self.parts[ci].els[el]
        g = np.zeros(self.n, dtype=complex)
        if ci == 0:
            return complex(p[2]), g
        c, s, tx, ty = w[4 * (ci - 1): 4 * ci]
        nx, ny, cc = float(p[0]), float(p[1]), float(p[2])
        n0, n1 = c * nx - s * ny, s * nx + c * ny
        o = 4 * (ci - 1)
        g[o:o + 4] = [nx * tx + ny * ty, -ny * tx + nx * ty, n0, n1]
        return cc + n0 * tx + n1 * ty, g

    def Q(self, w: CVec) -> CVec:
        out = np.empty(self.m_q, dtype=complex)
        for r, (i, j, e) in enumerate(self.offsets):
            out[r] = self._offset(w, i, e)[0] - self._offset(w, j, e)[0]
        for q in range(self.k):
            c, s = w[4 * q], w[4 * q + 1]
            out[len(self.offsets) + q] = c * c + s * s - 1
        return out

    def JQ(self, w: CVec) -> CVec:
        J = np.zeros((self.m_q, self.n), dtype=complex)
        for r, (i, j, e) in enumerate(self.offsets):
            J[r] = self._offset(w, i, e)[1] - self._offset(w, j, e)[1]
        for q in range(self.k):
            J[len(self.offsets) + q, 4 * q] = 2 * w[4 * q]
            J[len(self.offsets) + q, 4 * q + 1] = 2 * w[4 * q + 1]
        return J

    def F(self, w: CVec) -> CVec:
        return np.concatenate([self.A @ w - self.b, self.Q(w)])

    def J(self, w: CVec) -> CVec:
        return np.vstack([self.A.astype(complex), self.JQ(w)])


def _w_to_u(w: CVec) -> np.ndarray:
    k = w.size // 4
    u = np.zeros(3 * k)
    for q in range(k):
        c, s, tx, ty = w[4 * q: 4 * q + 4].real
        u[3 * q: 3 * q + 3] = [math.atan2(s, c), tx, ty]
    return u


def enumerate_step(plan: Plan, step_index: int, *, seed: int = 0, max_paths: int = 256,
                   max_steps: int = 400, diverge: float = 1e6) -> list[Alternative]:
    """Real solutions of the merge at `step_index` (the current one first).  Returns [] if the
    merge is not isolated (under-determined) or too large (> max_paths)."""
    rng = np.random.default_rng(seed)
    parts = execute(plan, capture=step_index)
    if parts is None or len(parts) < 2:
        return []
    step = plan.steps[step_index]
    P = _Poly(parts, step)
    n, k = P.n, P.k
    w_id = np.tile([1.0, 0.0, 0.0, 0.0], k).astype(complex)   # the current solution: identity
    # -- square the system: Ã w = b̃ (rank r) and n − r combinations of the quadratic rows --
    r = int(np.linalg.matrix_rank(P.A, tol=1e-9)) if P.A.size else 0
    n_q = n - r
    if n_q <= 0 or P.m_q < n_q or 2 ** n_q > max_paths:
        return []
    M1 = rng.standard_normal((r, P.A.shape[0])) + 1j * rng.standard_normal((r, P.A.shape[0]))
    M2 = rng.standard_normal((n_q, P.m_q)) + 1j * rng.standard_normal((n_q, P.m_q))
    At, bt = M1 @ P.A, M1 @ P.b
    # -- start system: the same linear rows + w_σ² − 1 on variables free w.r.t. the linear part --
    _, _, Vh = np.linalg.svd(At) if At.size else (None, None, np.eye(n, dtype=complex))
    N = Vh[r:].conj().T
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

    def H(w: CVec, t: float) -> CVec:
        return np.concatenate([At @ w - bt, (1 - t) * gamma * G(w) + t * (M2 @ P.Q(w))])

    def JH(w: CVec, t: float) -> CVec:
        return np.vstack([At, (1 - t) * gamma * JG(w) + t * (M2 @ P.JQ(w))])

    def Ht(w: CVec) -> CVec:
        return np.concatenate([np.zeros(r, dtype=complex), -gamma * G(w) + M2 @ P.Q(w)])

    E = np.zeros((n_q, n), dtype=complex)
    for q, s in enumerate(sigma):
        E[q, s] = 1.0
    starts = []
    for bits in range(2 ** n_q):
        signs = np.array([1.0 if (bits >> q) & 1 else -1.0 for q in range(n_q)], dtype=complex)
        starts.append(np.linalg.solve(np.vstack([At, E]), np.concatenate([bt, signs])))

    def newton(w: CVec, t: float, iters: int = 4, tol: float = 1e-10) -> tuple[CVec, bool]:
        for _ in range(iters):
            h = H(w, t)
            if np.linalg.norm(h) < tol * (1 + np.linalg.norm(w)):
                return w, True
            try:
                w = w - np.linalg.solve(JH(w, t), h)
            except np.linalg.LinAlgError:
                return w, False
        return w, bool(np.linalg.norm(H(w, t)) < 1e-6 * (1 + np.linalg.norm(w)))

    ends: list[CVec] = []
    for w in starts:
        t, dt = 0.0, 0.02
        ok_path = True
        for _ in range(max_steps):
            if t >= 1.0:
                break
            t1 = min(1.0, t + dt)
            try:
                dw = -np.linalg.solve(JH(w, t), Ht(w))          # Euler predictor
            except np.linalg.LinAlgError:
                dt *= 0.5
                if dt < 1e-10:
                    ok_path = False
                    break
                continue
            w_new, ok = newton(w + dw * (t1 - t), t1)
            if ok and np.linalg.norm(w_new - w) < 0.5 * (1 + np.linalg.norm(w)):
                w, t = w_new, t1
                dt = min(0.2, dt * 1.5)
            else:
                dt *= 0.5
                if dt < 1e-10:
                    ok_path = False
                    break
            if np.linalg.norm(w) > diverge:                      # a path to infinity: give up early
                ok_path = False
                break
        else:
            ok_path = False
        if ok_path and t >= 1.0:
            for _ in range(5):                                   # polish on the original system
                f = P.F(w)
                if np.linalg.norm(f) < 1e-12:
                    break
                w = w - np.linalg.lstsq(P.J(w), f, rcond=None)[0]
            ends.append(w)
    out: list[Alternative] = []
    for w in ends:
        if np.abs(w.imag).max() > 1e-6 * (1 + np.abs(w.real).max()):
            continue
        wr = w.real.astype(complex)
        if np.linalg.norm(P.F(wr)) > 1e-6:
            continue
        if any(np.linalg.norm(wr - a_w) < 1e-6 for a_w in (_u_to_w(a.u) for a in out)):
            continue
        d = float(np.linalg.norm(wr - w_id))
        out.append(Alternative(_w_to_u(wr), d, d < 1e-6))
    out.sort(key=lambda a: a.distance)
    return out


def _u_to_w(u: np.ndarray) -> CVec:
    k = u.size // 3
    w = np.zeros(4 * k, dtype=complex)
    for q in range(k):
        th, tx, ty = u[3 * q: 3 * q + 3]
        w[4 * q: 4 * q + 4] = [math.cos(th), math.sin(th), tx, ty]
    return w


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
    for q, c in enumerate(parts[1:]):
        T = _T(*alt.u[3 * q: 3 * q + 3])
        for e, pose in c.els.items():
            if e.kind == "P":
                p2 = _apply(T, e, pose)
                for p in g.members[e.idx]:
                    p.x.value, p.y.value = float(p2[0]), float(p2[1])
    plan.sticky_branches = True
    execute(plan)
