"""Stage 3 — cluster merging (Fudos–Hoffmann, generalised) → plan → replay.

Decomposition (once per topology):
  * every PP/PL edge seeds a 2-element rigid *cluster*; ground (fixed points +
    x-axis) is a fixed cluster; lines carry *direction classes* (union-find with
    angular offsets over all angle-type constraints);
  * repeatedly merge a pair or a triple of clusters when what they share —
    points, lines, directions — *determines* their relative rigid transforms.
    F–H's triangle rule (three clusters pairwise sharing one element) is the
    common case; the decision is made generally by the rank of the small merge
    Jacobian at generic (witness) poses, with self-motions of degenerate
    clusters (a lone point, parallel lines) accounted for — so parallels,
    perpendiculars and H/V need no special cases;
  * when pair/triple merging stalls, look for a small *core*: a minimal subset
    of clusters that is rigid as a whole (Stage 3b — DR-planning / Owen's idea
    of isolating the non-tree-decomposable part): grow from each seed by
    generic-rank deficiency, take the smallest rigid subset found (capped in
    size — the numeric cost is exponential in exactly this), merge it as one
    numeric step, resume tree merging;
  * the merge sequence is the *plan*: each Step is lowered to flat data
    (reference-first cluster ids, shared-element rows, direction rows, the
    closed-form construction if any); the clusters left over are the roots.

Execution (every solve / drag frame, no graph analysis):
  * leaf poses from the live dimension values, warm-started on the current
    geometry (this is also what picks roots close to what the user sees);
  * PPP triangle merges by ruler-and-compass (circle–circle intersection) with
    an explicit chirality flag; other merges by a small min-norm Newton
    (DogLeg if it does not converge);
  * unfixed roots placed by least-change (Procrustes onto current positions);
  * write back; verify with the compiled System; numeric fallback if needed.
"""

from __future__ import annotations

import math
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from gcs import newton
from gcs.cgraph import X_AXIS, ConstraintGraph, Edge, El, build, line_normal, normal_of
from gcs.model import Arc, Circle, Point, Sketch, Vec
from gcs.solve import Method, SolveResult, System

Pose = Vec  # point: (x, y); line: (nx, ny, c)
_X_POSE = np.array([0.0, 1.0, 0.0])
Rel = tuple[list[tuple[int, int, El]], list[tuple[int, int, El, El, float]]]   # (pairs, dpairs) between clusters


class Cluster:
    __slots__ = ("id", "els", "fixed")

    def __init__(self, cid: int, els: dict[El, Pose], fixed: bool) -> None:
        self.id, self.els, self.fixed = cid, els, fixed

    def __repr__(self) -> str:
        return f"Cluster({self.id}{'*' if self.fixed else ''} {sorted(self.els)})"


@dataclass
class Step:
    """One merge, lowered for replay.  ids[0] is the reference cluster (identity transform);
    pairs/dpairs use positions into `ids`; a PPP triangle carries its (x, y, z) construction
    and `branch` (±1 chirality: orientation of (x, z, y)) — set by the replay from the sketch
    when None, so a persisted plan replays the recorded root (Stage 5 substrate)."""

    ids: tuple[int, ...]
    pairs: list[tuple[int, int, El]]
    dpairs: list[tuple[int, int, El, El, float]]
    ppp: tuple[El, El, El] | None = None
    branch: int | None = None

    @property
    def out(self) -> int:
        return self.ids[0]

    @property
    def key(self) -> str:
        """Stable identity across recompiles of the same topology: the shared elements."""
        if self.ppp is not None:
            return "ppp:" + "|".join(f"{e.kind}{e.idx}" for e in self.ppp)
        return "merge:" + "|".join(sorted(f"{e.kind}{e.idx}" for _, _, e in self.pairs))


@dataclass
class Plan:
    graph: ConstraintGraph
    leaves: list[tuple[int, int]]          # (cluster id, edge index)
    ground_id: int
    singletons: list[tuple[int, El]]
    steps: list[Step]
    roots: list[int]
    sticky_branches: bool = False          # True: replay recorded chirality even if the sketch moved (Stage 5)

    @property
    def fully_decomposed(self) -> bool:
        return not self.graph.unsupported and len(self.roots) == 1

    # -- chirality (Stage 5) ------------------------------------------------

    def branches(self) -> dict[str, int]:
        """Recorded root choices of the closed-form merges, keyed stably for persistence."""
        return {st.key: st.branch for st in self.steps if st.ppp is not None and st.branch is not None}

    def apply_branches(self, branches: dict[str, int]) -> int:
        """Install recorded root choices (e.g. from a document); returns how many matched."""
        n = 0
        for st in self.steps:
            if st.ppp is not None and st.key in branches:
                st.branch = 1 if branches[st.key] >= 0 else -1
                n += 1
        return n

    def steps_placing(self, e: El) -> list[Step]:
        """Closed-form merges whose constructed point is `e` (the apex of the triangle)."""
        return [st for st in self.steps if st.ppp is not None and st.ppp[1] == e]

    def flip(self, e: El) -> int:
        """Flip the root of every closed-form merge that constructs `e`; returns how many."""
        sts = self.steps_placing(e)
        for st in sts:
            st.branch = -(st.branch or 1)
        return len(sts)

    def summary(self) -> str:
        return (f"{len(self.leaves)} leaves, {len(self.steps)} merges → {len(self.roots)} root(s); "
                f"{len(self.graph.unsupported)} unsupported constraint(s)")


# ---------------------------------------------------------------------------
# direction classes: weighted union-find, potential[l] = angle of n_l relative to the root's

class _Dirs:
    def __init__(self) -> None:
        self.parent: dict[El, El] = {}
        self.pot: dict[El, float] = {}

    def add(self, e: El) -> None:
        self.parent.setdefault(e, e)
        self.pot.setdefault(e, 0.0)

    def find(self, e: El) -> tuple[El, float]:
        """(root, angle of e relative to root)."""
        self.add(e)
        path = []
        while self.parent[e] != e:
            path.append(e)
            e = self.parent[e]
        root = e
        acc = 0.0
        for x in reversed(path):                     # path compression with potentials
            acc += self.pot[x]
            self.parent[x] = root
            self.pot[x] = acc
        return root, self.pot[path[0]] if path else 0.0

    def join(self, a: El, b: El, phi: float) -> bool:
        """Impose n_b = rot(phi)·n_a.  Returns False if it contradicts an existing relation."""
        ra, pa = self.find(a)
        rb, pb = self.find(b)
        if ra == rb:
            return abs(math.remainder(pb - pa - phi, math.pi)) < 1e-9
        self.parent[rb] = ra
        self.pot[rb] = pa + phi - pb
        return True

    def offset(self, a: El, b: El) -> float | None:
        ra, pa = self.find(a)
        rb, pb = self.find(b)
        return pb - pa if ra == rb else None


# ---------------------------------------------------------------------------
# rigid transforms

def _apply(T: Vec, el: El, pose: Pose) -> Pose:
    c, s, tx, ty = T
    if el.kind == "P":
        x, y = pose
        return np.array([c * x - s * y + tx, s * x + c * y + ty])
    nx, ny, cc = pose
    nx2, ny2 = c * nx - s * ny, s * nx + c * ny
    return np.array([nx2, ny2, cc + nx2 * tx + ny2 * ty])


def _T(theta: float, tx: float, ty: float) -> Vec:
    return np.array([math.cos(theta), math.sin(theta), tx, ty])


def _pose_of(el: El, pose: Pose, th: float, tx: float, ty: float) -> Pose:
    """Pose of `el` under (θ, t) — no Jacobian (residual evaluations)."""
    c, s = math.cos(th), math.sin(th)
    if el.kind == "P":
        x, y = pose
        return np.array([c * x - s * y + tx, s * x + c * y + ty])
    nx, ny, cc = pose
    n0, n1 = c * nx - s * ny, s * nx + c * ny
    return np.array([n0, n1, cc + n0 * tx + n1 * ty])


def _pose_jac(el: El, pose: Pose, th: float, tx: float, ty: float) -> tuple[Pose, Vec]:
    """Pose of `el` under (θ, t) and its Jacobian wrt (θ, tx, ty)."""
    c, s = math.cos(th), math.sin(th)
    if el.kind == "P":
        x, y = pose
        return (np.array([c * x - s * y + tx, s * x + c * y + ty]),
                np.array([[-s * x - c * y, 1.0, 0.0], [c * x - s * y, 0.0, 1.0]]))
    nx, ny, cc = pose
    n2 = np.array([c * nx - s * ny, s * nx + c * ny])
    dn = np.array([-s * nx - c * ny, c * nx - s * ny])
    return (np.array([n2[0], n2[1], cc + n2[0] * tx + n2[1] * ty]),
            np.array([[dn[0], 0.0, 0.0], [dn[1], 0.0, 0.0], [dn[0] * tx + dn[1] * ty, n2[0], n2[1]]]))


def _procrustes(src: Vec, dst: Vec) -> Vec:
    """Rigid transform (c, s, tx, ty) mapping points src (n,2) onto dst in least squares."""
    n = len(src)
    if n == 0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    ms, md = src.mean(axis=0), dst.mean(axis=0)
    if n == 1:
        return np.array([1.0, 0.0, md[0] - ms[0], md[1] - ms[1]])
    A, B = src - ms, dst - md
    c = float((A * B).sum())
    s = float((A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0]).sum())
    L = math.hypot(c, s) or 1.0
    c, s = c / L, s / L
    return np.array([c, s, md[0] - (c * ms[0] - s * ms[1]), md[1] - (s * ms[0] + c * ms[1])])


def _fit2(p: Pose, q: Pose, p2: Pose, q2: Pose) -> Vec:
    """Rigid transform taking segment p→q onto p2→q2 (exact when lengths agree; scalar math —
    this is the inner loop of every closed-form merge)."""
    ux, uy = q[0] - p[0], q[1] - p[1]
    vx, vy = q2[0] - p2[0], q2[1] - p2[1]
    c = ux * vx + uy * vy
    s = ux * vy - uy * vx
    L = math.hypot(c, s) or 1.0
    c, s = c / L, s / L
    return np.array([c, s, p2[0] - (c * p[0] - s * p[1]), p2[1] - (s * p[0] + c * p[1])])


# ---------------------------------------------------------------------------
# merge system (shared by the generic-rank decision and by execution)

def _merge_system(cl: list[Cluster], pairs: list[tuple[int, int, El]],
                  dpairs: list[tuple[int, int, El, El, float]], k_movable: int
                  ) -> tuple[Callable[[Vec], Vec], Callable[[Vec], Vec]]:
    """Residual/Jacobian callables for transforms of cl[1..] (cl[0] = reference, identity)."""

    def pose(u: Vec, ci: int, el: El) -> Pose:
        p = cl[ci].els[el]
        return p if ci == 0 else _pose_of(el, p, *u[3 * (ci - 1): 3 * ci])

    def dpose(u: Vec, ci: int, el: El) -> tuple[Pose, Vec]:
        p = cl[ci].els[el]
        return (p, np.zeros((p.size, 3))) if ci == 0 else _pose_jac(el, p, *u[3 * (ci - 1): 3 * ci])

    def fun(u: Vec) -> Vec:
        parts = [pose(u, i, e) - pose(u, j, e) for i, j, e in pairs]
        for i, j, la, lb, phi in dpairs:      # angle(n_a', n_b') = phi — scalar, linear in the θ's
            na, nb = pose(u, i, la), pose(u, j, lb)
            ang = math.atan2(na[0] * nb[1] - na[1] * nb[0], na[0] * nb[0] + na[1] * nb[1])
            parts.append(np.array([math.remainder(ang - phi, 2 * math.pi)]))
        return np.concatenate(parts) if parts else np.zeros(0)

    def jac(u: Vec) -> Vec:
        rows = []
        for i, j, e in pairs:
            pi, Ji = dpose(u, i, e)
            _, Jj = dpose(u, j, e)
            row = np.zeros((pi.size, 3 * k_movable))
            if i > 0:
                row[:, 3 * (i - 1): 3 * i] += Ji
            if j > 0:
                row[:, 3 * (j - 1): 3 * j] -= Jj
            rows.append(row)
        for i, j, la, lb, phi in dpairs:      # d angle / dθ_b = 1, / dθ_a = −1
            row = np.zeros((1, 3 * k_movable))
            if j > 0:
                row[0, 3 * (j - 1)] += 1.0
            if i > 0:
                row[0, 3 * (i - 1)] -= 1.0
            rows.append(row)
        return np.vstack(rows) if rows else np.zeros((0, 3 * k_movable))

    return fun, jac


def _newton_small(fun: Callable[[Vec], Vec], jac: Callable[[Vec], Vec], u: Vec,
                  tol: float = 1e-13, max_iter: int = 40) -> tuple[Vec, float]:
    """Plain min-norm Newton for the tiny merge systems (3k unknowns, warm-started at the
    identity).  Returns (u, max |residual|).  No trust region: merges are near-linear from a
    warm start, and the caller falls back to DogLeg if this does not converge."""
    r = fun(u)
    for _ in range(max_iter):
        if r.size == 0 or float(np.abs(r).max()) < tol:
            break
        p, _ = newton.min_norm_lstsq(jac(u), -r)
        u = u + p
        r = fun(u)
        if float(np.abs(p).max()) < 1e-15:
            break
    return u, float(np.abs(r).max()) if r.size else 0.0


def _self_motion(cl: Cluster) -> int:
    """Dimension of rigid motions that leave every element of the cluster in place:
    empty 3; a lone point 1; lines only, all parallel 1; otherwise 0.  (Poses are generic,
    so two points are distinct and non-parallel lines are transversal.)"""
    n_pts = 0
    first_n: Pose | None = None
    for e, pose in cl.els.items():
        if e.kind == "P":
            n_pts += 1
            if n_pts >= 2:
                return 0
        else:
            if n_pts:
                return 0
            if first_n is None:
                first_n = pose
            elif abs(first_n[0] * pose[1] - first_n[1] * pose[0]) > 1e-9:
                return 0
    if n_pts == 1:
        return 1 if first_n is None else 0
    return 3 if first_n is None else 1


def _order_ref_first(cl: dict[int, Cluster], ids: list[int]) -> list[int]:
    """Cluster ids with the reference (fixed if any, else the largest) first."""
    ref = next((i for i in ids if cl[i].fixed), max(ids, key=lambda i: len(cl[i].els)))
    return [ref] + [i for i in ids if i != ref]


# ---------------------------------------------------------------------------
# decomposition (topology only)

def decompose(graph: ConstraintGraph, seed: int = 0, core_max: int = 12) -> Plan:
    rng = np.random.default_rng(seed)
    dirs = _Dirs()
    for d in graph.dirs:
        dirs.join(d.a, d.b, d.phi)
    elements = graph.elements
    # generic (witness) poses: random points; lines get a random normal per direction class
    # (+ their class offset) and a random offset — merge decisions are structural, so they
    # must not depend on the user's possibly-degenerate geometry
    base_angle: dict[El, float] = {}
    generic: dict[El, Pose] = {}
    droot: dict[El, El] = {}
    for e in elements:
        if e.kind == "P":
            generic[e] = rng.uniform(-100, 100, 2)
        else:
            root, pot = dirs.find(e)
            droot[e] = root
            ang = base_angle.setdefault(root, rng.uniform(0, 2 * math.pi)) + pot
            generic[e] = np.array([math.cos(ang), math.sin(ang), rng.uniform(-100, 100)])

    clusters: dict[int, Cluster] = {}
    of: dict[El, set[int]] = {e: set() for e in elements}
    dir_of: dict[El, set[int]] = {}          # direction root → clusters containing a line of that class
    cdirs: dict[int, dict[El, El]] = {}      # cluster → {direction root: one of its lines}
    next_id = 0

    def register(cid: int, els: list[El]) -> None:
        for e in els:
            of[e].add(cid)
            if e.kind != "P":
                r = droot[e]
                dir_of.setdefault(r, set()).add(cid)
                cdirs[cid].setdefault(r, e)

    def add(els: set[El], fx: bool) -> int:
        nonlocal next_id
        cid = next_id
        next_id += 1
        clusters[cid] = Cluster(cid, {e: generic[e] for e in els}, fx)
        cdirs[cid] = {}
        register(cid, sorted(els))
        return cid

    def remove(cid: int) -> Cluster:
        c = clusters.pop(cid)
        for e in c.els:
            of[e].discard(cid)
        for r in cdirs.pop(cid):
            dir_of[r].discard(cid)
        for key in rel_keys.pop(cid, ()):
            rel_memo.pop(key, None)
        return c

    ground = add({X_AXIS, *(El("P", i) for i in graph.ground_points)}, True)
    leaves = [(add({e.a, e.b}, False), i) for i, e in enumerate(graph.edges)]
    singletons = [(add({e}, False), e) for e in elements if not of[e]]
    steps: list[Step] = []

    # -- what two clusters share (memoised; entries die with either cluster) --
    rel_memo: dict[tuple[int, int], tuple[list[El], list[tuple[El, El, float]]]] = {}
    rel_keys: dict[int, set[tuple[int, int]]] = {}

    def pair_rel(a: int, b: int) -> tuple[list[El], list[tuple[El, El, float]]]:
        key = (a, b) if a < b else (b, a)
        hit = rel_memo.get(key)
        if hit is not None:
            return hit
        A, B = clusters[key[0]], clusters[key[1]]
        small, big = (A, B) if len(A.els) <= len(B.els) else (B, A)
        common = sorted(e for e in small.els if e in big.els)      # O(min) with dict membership
        seen = {droot[e] for e in common if e.kind != "P"}
        da, db = cdirs[key[0]], cdirs[key[1]]
        drels: list[tuple[El, El, float]] = []
        for root, la in (da if len(da) <= len(db) else db).items():
            if root in seen or root not in (db if len(da) <= len(db) else da):
                continue
            la_, lb_ = (la, db[root]) if la in A.els else (da[root], la)
            phi = dirs.offset(la_, lb_)
            assert phi is not None
            drels.append((la_, lb_, phi))
        rel_memo[key] = (common, drels)
        rel_keys.setdefault(a, set()).add(key)
        rel_keys.setdefault(b, set()).add(key)
        return common, drels

    def relations(ids: list[int]) -> Rel:
        """Shared rows between clusters `ids` (positions into ids); (i, j) with i < j and the
        direction relation oriented from ids[i]'s line to ids[j]'s line."""
        pairs: list[tuple[int, int, El]] = []
        dpairs: list[tuple[int, int, El, El, float]] = []
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                common, drels = pair_rel(ids[i], ids[j])
                pairs += [(i, j, e) for e in common]
                for la, lb, phi in drels:
                    if la in clusters[ids[i]].els:
                        dpairs.append((i, j, la, lb, phi))
                    else:
                        dpairs.append((i, j, lb, la, -phi))
        return pairs, dpairs

    selfm: dict[int, int] = {}

    def self_motion(cid: int) -> int:
        if cid not in selfm:
            selfm[cid] = _self_motion(clusters[cid])
        return selfm[cid]

    def deficiency(ids: list[int]) -> int:
        """Relative rigid-transform DOF left after imposing everything the clusters share
        (0 ⇔ the merge is determined).  Generic rank of the merge Jacobian at witness poses."""
        ids = _order_ref_first(clusters, ids)
        pairs, dpairs = relations(ids)
        k = len(ids) - 1
        need = 3 * k - sum(self_motion(i) for i in ids[1:])
        if need <= 0:
            return 0
        bound = 2 * len(pairs) + len(dpairs)
        if bound < need:                              # cheap upper bound on the rank
            return need - bound
        _, jac = _merge_system([clusters[i] for i in ids], pairs, dpairs, k)
        J = jac(np.zeros(3 * k))
        rank = newton.rank_rrqr(J, 1e-9) if J.size else 0
        return max(0, need - rank)

    def determined(ids: list[int]) -> bool:
        return deficiency(ids) == 0

    def merge(ids: list[int]) -> int:
        ids = _order_ref_first(clusters, ids)
        pairs, dpairs = relations(ids)
        ppp = None
        if len(ids) == 3 and not dpairs and len(pairs) == 3 and all(e.kind == "P" for _, _, e in pairs) \
                and len({(i, j) for i, j, _ in pairs}) == 3:
            share = {(i, j): e for i, j, e in pairs}
            ppp = (share[(0, 1)], share[(1, 2)], share[(0, 2)])          # x = ref∩B, y = B∩C, z = C∩ref
        keep = ids[0]
        K = clusters[keep]                                # small-into-large: absorb into the reference
        selfm.pop(keep, None)
        for key in rel_keys.pop(keep, ()):
            rel_memo.pop(key, None)
        for i in ids[1:]:
            c = remove(i)
            selfm.pop(i, None)
            new = [e for e in c.els if e not in K.els]
            for e in new:
                K.els[e] = c.els[e]
            register(keep, new)
            K.fixed = K.fixed or c.fixed
        steps.append(Step(tuple(ids), pairs, dpairs, ppp))
        return keep

    def neighbours(a: int) -> set[int]:
        nb: set[int] = set()
        for e in clusters[a].els:
            nb |= of[e]
        for r in cdirs[a]:
            nb |= dir_of.get(r, set())
        nb.discard(a)
        return nb

    def maximal_clusters() -> list[int]:
        keys = {cid: set(c.els) for cid, c in clusters.items()}
        return [cid for cid in sorted(clusters)
                if not any(cid != o and keys[cid] < keys[o] for o in clusters)]

    def tree_merges(seed_ids: list[int]) -> None:
        """Worklist: a cluster is re-examined when it is created or a neighbour changes."""
        work: deque[int] = deque(seed_ids)
        queued = set(work)
        while work:
            a = work.popleft()
            queued.discard(a)
            if a not in clusters:
                continue
            nbs = sorted(neighbours(a))
            out = -1
            for b in nbs:                                   # pair merges
                if determined([a, b]):
                    out = merge([a, b])
                    break
            if out < 0:
                for i, b in enumerate(nbs):                 # triple merges
                    nb_b = neighbours(b)
                    for c in nbs[i + 1:]:
                        if c in nb_b and determined([a, b, c]):
                            out = merge([a, b, c])
                            break
                    if out >= 0:
                        break
            if out >= 0:
                for x in [out, *sorted(neighbours(out))]:
                    if x not in queued:
                        work.append(x)
                        queued.add(x)

    def find_core() -> list[int] | None:
        """Smallest rigid subset of ≥ 4 clusters found by greedy growth from every seed
        (pairs/triples are already exhausted).  None if nothing rigid within `core_max`."""
        best: list[int] | None = None
        live = maximal_clusters()
        if len(live) > 400:
            return None
        for seed in live:
            S = [seed]
            inS = {seed}
            while len(S) < core_max and (best is None or len(S) + 1 < len(best)):
                frontier = set()
                for x in S:
                    frontier |= neighbours(x)
                frontier -= inS
                if not frontier:
                    break
                d, _, n = min((deficiency(S + [n]), len(clusters[n].els), n) for n in frontier)
                S.append(n)
                inS.add(n)
                if d == 0:
                    if best is None or len(S) < len(best):
                        best = list(S)
                    break
        return best

    tree_merges(sorted(clusters))
    while True:
        core = find_core()
        if core is None:
            break
        out = merge(core)
        tree_merges([out, *sorted(neighbours(out))])
    return Plan(graph, leaves, ground, singletons, steps, maximal_clusters())


# ---------------------------------------------------------------------------
# execution

def _world_pose(graph: ConstraintGraph, e: El) -> Pose:
    if e.kind == "P":
        p = graph.members[e.idx][0]
        return np.array([p.x.value, p.y.value])
    if e == X_AXIS:
        return _X_POSE
    if e.kind == "L":
        return np.array(line_normal(graph.lines[e.idx]))
    a, b = (_world_pose(graph, q) for q in graph.virtual[e.idx])   # virtual line through two points
    return np.array(normal_of(a[0], a[1], b[0], b[1]))


def _leaf(graph: ConstraintGraph, edge: Edge) -> dict[El, Pose]:
    """Poses of a 2-element cluster satisfying its edge, nearest the current geometry."""
    a, b = _world_pose(graph, edge.a), _world_pose(graph, edge.b)
    v = edge.value()
    if edge.kind == "PP":
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        ux, uy = (dx / L, dy / L) if L > 1e-12 else (1.0, 0.0)
        return {edge.a: a, edge.b: np.array([a[0] + v * ux, a[1] + v * uy])}
    nx, ny, c = float(b[0]), float(b[1]), float(b[2])      # PL: n·p − c = v
    off = nx * a[0] + ny * a[1] - c - v
    return {edge.a: np.array([a[0] - off * nx, a[1] - off * ny]), edge.b: b}


def _merge_ppp(ref: Cluster, B: Cluster, Cc: Cluster, x: El, y: El, z: El, sign: int) -> tuple[Vec, Vec]:
    """Triangle merge with all shared elements points: ref∩B = {x}, B∩C = {y}, C∩ref = {z}.
    In ref's frame y is a circle–circle intersection; `sign` (±1) is the chirality —
    the orientation of the triangle (x, z, y) — invariant under rigid motions of the whole
    (unlike "nearest position")."""
    xa, za = ref.els[x], ref.els[z]
    bx, by, cz, cy = B.els[x], B.els[y], Cc.els[z], Cc.els[y]
    dB = math.hypot(by[0] - bx[0], by[1] - bx[1])
    dC = math.hypot(cy[0] - cz[0], cy[1] - cz[1])
    ex, ey = za[0] - xa[0], za[1] - xa[1]
    L = math.hypot(ex, ey)
    ux, uy = (ex / L, ey / L) if L > 1e-12 else (1.0, 0.0)
    aa = (dB * dB - dC * dC + L * L) / (2 * L) if L > 1e-12 else 0.0
    h2 = dB * dB - aa * aa
    h = math.sqrt(h2) if h2 > 0 else 0.0
    fx, fy = xa[0] + aa * ux, xa[1] + aa * uy
    ya = np.array((fx - h * uy, fy + h * ux) if sign > 0 else (fx + h * uy, fy - h * ux))   # +1: y left of x→z
    return _fit2(bx, by, xa, ya), _fit2(cz, cy, za, ya)


def _place_root(c: Cluster, placed: dict[El, Pose], g: ConstraintGraph) -> Vec:
    """Transform for an unfixed root: elements already placed by earlier roots are aligned
    exactly (≥ 2 shared points: rigid fit on them; 1 shared point: it pins the translation and
    the rest of the points vote on the rotation); the remainder is least-change onto the
    current geometry."""
    pts = [e for e in c.els if e.kind == "P"]
    shared = [e for e in pts if e in placed]
    src_all = np.array([c.els[e] for e in pts]).reshape(-1, 2)
    dst_all = np.array([placed[e] if e in placed else _world_pose(g, e) for e in pts]).reshape(-1, 2)
    if len(shared) >= 2:
        return _procrustes(np.array([c.els[e] for e in shared]), np.array([placed[e] for e in shared]))
    T = _procrustes(src_all, dst_all)
    if len(shared) == 1:
        e = shared[0]
        moved = _apply(T, e, c.els[e])
        T = T.copy()
        T[2] += placed[e][0] - moved[0]
        T[3] += placed[e][1] - moved[1]
    return T


def execute(plan: Plan, capture: int | None = None) -> list[Cluster] | None:
    """Replay the plan on the current sketch values and write the result back.
    capture=i returns copies of the clusters entering step i instead (no write-back)."""
    g = plan.graph
    cl: dict[int, Cluster] = {}
    gels = {X_AXIS: _X_POSE}
    for i in g.ground_points:
        gels[El("P", i)] = _world_pose(g, El("P", i))
    cl[plan.ground_id] = Cluster(plan.ground_id, gels, True)
    for cid, ei in plan.leaves:
        cl[cid] = Cluster(cid, _leaf(g, g.edges[ei]), False)
    for cid, e in plan.singletons:
        cl[cid] = Cluster(cid, {e: _world_pose(g, e)}, False)

    for si, st in enumerate(plan.steps):
        parts = [cl.pop(i) for i in st.ids]
        if capture == si:
            return [Cluster(c.id, dict(c.els), c.fixed) for c in parts]
        ref, movable = parts[0], parts[1:]
        if st.ppp is not None:
            x, y, z = st.ppp
            if st.branch is None or not plan.sticky_branches:
                xw, zw, yw = _world_pose(g, x), _world_pose(g, z), _world_pose(g, y)
                orient = (zw[0] - xw[0]) * (yw[1] - xw[1]) - (zw[1] - xw[1]) * (yw[0] - xw[0])
                st.branch = 1 if orient >= 0 else -1                # sketch-guided chirality
            Ts = list(_merge_ppp(ref, movable[0], movable[1], x, y, z, st.branch))
        elif movable:
            # identity is the natural warm start: leaves are re-derived from the current geometry,
            # so the root the sketch is on is the one nearest the identity (sticky by nature)
            fun, jac = _merge_system(parts, st.pairs, st.dpairs, len(movable))
            u, res = _newton_small(fun, jac, np.zeros(3 * len(movable)))
            if res > 1e-9:      # cores (many clusters) or bad warm starts: use the globalised solver
                u, _ = newton.dogleg(fun, jac, np.zeros(3 * len(movable)), ftol=1e-13, xtol=1e-14,
                                     gtol=1e-18, max_iter=300)
            Ts = [_T(u[3 * i], u[3 * i + 1], u[3 * i + 2]) for i in range(len(movable))]
        else:
            Ts = []
        els = ref.els                                    # absorb in place (parts were popped)
        for c, T in zip(movable, Ts, strict=True):
            for e, pose in c.els.items():
                if e not in els:
                    els[e] = _apply(T, e, pose)
            ref.fixed = ref.fixed or c.fixed
        cl[st.out] = ref

    # -- place roots: fixed ones are in world frame; others least-change onto current geometry,
    #    aligning any element already placed by an earlier root exactly --
    placed: dict[El, Pose] = {}
    for rid in sorted(plan.roots, key=lambda i: (not cl[i].fixed, -len(cl[i].els), i)):
        c = cl[rid]
        if not c.fixed:
            T = _place_root(c, placed, g)
            for e in c.els:
                c.els[e] = _apply(T, e, c.els[e])
        for e, pose in c.els.items():
            placed.setdefault(e, pose)
    for e, pose in placed.items():
        if e.kind == "P":
            for p in g.members[e.idx]:
                p.x.value, p.y.value = float(pose[0]), float(pose[1])
    rounds: list[Circle | Arc] = [*g.sketch.circles, *g.sketch.arcs]
    for circle in rounds:
        r = g.known_radius.get(id(circle.radius))
        if r is not None and not circle.radius.fixed:
            circle.radius.value = r
    return None


# ---------------------------------------------------------------------------

@dataclass
class PlanResult:
    success: bool
    max_residual: float
    fell_back: bool
    time_s: float
    plan: Plan
    numeric: SolveResult | None = None

    def __repr__(self) -> str:
        return (f"PlanResult(ok={self.success}, max|r|={self.max_residual:.2e}, fallback={self.fell_back}, "
                f"{self.time_s * 1e3:.2f} ms, {self.plan.summary()})")

    def as_solve_result(self) -> SolveResult:
        """The same outcome in the solver's common result type (method 'plan' or the fallback's)."""
        if self.numeric is not None:
            return self.numeric
        return SolveResult(self.success, 0, "plan", self.max_residual, self.max_residual,
                           len(self.plan.steps), 0, self.time_s, "plan")


class PlanSolver:
    """Compile once per topology (graph + decomposition + System for verification);
    `solve()` replays the plan and falls back to the numeric core when the residual
    says the plan did not (fully) determine the sketch."""

    def __init__(self, sketch: Sketch, branches: dict[str, int] | None = None, sticky: bool = False) -> None:
        self.sketch = sketch
        self.graph = build(sketch)
        self.plan = decompose(self.graph)
        self.plan.sticky_branches = sticky
        if branches:
            self.plan.apply_branches(branches)
        self.system = System(sketch)

    def solve(self, tol: float = 1e-9, fallback: bool = True, method: Method = "dogleg") -> PlanResult:
        t0 = time.perf_counter()
        execute(self.plan)
        self.system.refresh_consts()          # dimensions may have been edited since compile
        mx = self.system.max_hard_residual()
        num = None
        fell = False
        if mx > tol * self.system.scale and fallback:
            fell = True
            num = self.system.solve(method=method)
            mx = num.max_residual
        return PlanResult(mx <= 1e-6 * self.system.scale, mx, fell, time.perf_counter() - t0, self.plan, num)


def ppp_triangles(plan: Plan) -> list[tuple[Point, Point, Point]]:
    """The closed-form merges' triangles (x, z, y) as Points — the order-type invariants to guard."""
    g = plan.graph
    out = []
    for st in plan.steps:
        if st.ppp is not None:
            x, y, z = st.ppp
            out.append((g.members[x.idx][0], g.members[z.idx][0], g.members[y.idx][0]))
    return out


class PlanDrag:
    """DCM-style drag: the dragged point joins the ground (fixed at the cursor) and the cached
    plan replays per frame — no graph analysis while dragging, recorded roots are sticky, and
    under-constrained roots move least.  Large cursor jumps are taken in increments so the
    solution tracks its branch (continuation).  If the plan cannot determine the sketch with
    the point pinned (fully constrained sketches, unsupported constraints) the numeric
    pull/polish `Drag` is used instead."""

    def __init__(self, sketch: Sketch, point: Point, x: float, y: float, branches: dict[str, int] | None = None,
                 max_step_rel: float = 0.05) -> None:
        from gcs.solve import Drag

        self.sketch, self.point = sketch, point
        self.max_step = max_step_rel * max(1.0, sketch.extent())
        self.x0 = sketch.get_x()
        self.numeric: Drag | None = None
        was = point.is_fixed
        point.fix(True)
        try:
            self.solver = PlanSolver(sketch, branches, sticky=True)
        finally:
            point.fix(was)
        # usable iff the plan understands every constraint and pinning the point does not
        # over-determine the sketch structurally (then the plan could only compromise)
        from gcs.diagnose import diagnose

        self.usable = not self.solver.graph.unsupported
        if self.usable:
            point.fix(True)
            try:
                self.usable = diagnose(sketch, numeric=False, conflicts=False).n_redundant == 0
            finally:
                point.fix(was)
        if self.usable:                       # probe the replay at the current position
            self.usable = self._replay(*point.xy) <= 1e-9 * self.solver.system.scale
        if not self.usable:
            sketch.set_x(self.x0)
            self.numeric = Drag(sketch, point, x, y, guards=self._guards())

    def _guards(self) -> list[tuple[Point, Point, Point]]:
        """Order-type invariants to watch on the numeric path: the closed-form triangles of the
        sketch's own (unpinned) plan."""
        return ppp_triangles(decompose(build(self.sketch)))

    def _replay(self, x: float, y: float) -> float:
        self.point.x.value, self.point.y.value = x, y
        execute(self.solver.plan)
        return self.solver.system.max_hard_residual()

    def move(self, x: float, y: float) -> SolveResult:
        if self.numeric is not None:
            return self.numeric.move(x, y)
        from gcs.solve import Drag

        t0 = time.perf_counter()
        x_prev = self.sketch.get_x()
        px, py = self.point.xy
        d = math.hypot(x - px, y - py)
        n = max(1, int(math.ceil(d / self.max_step)))   # continuation: never jump far in one replay
        mx = 0.0
        for i in range(1, n + 1):
            mx = self._replay(px + (x - px) * i / n, py + (y - py) * i / n)
            if mx > 1e-6 * self.solver.system.scale:
                # the plan cannot follow (a limit of the geometry was hit): hand over to the
                # numeric drag from the last good state
                self.sketch.set_x(x_prev)
                self.numeric = Drag(self.sketch, self.point, px, py, guards=self._guards())
                return self.numeric.move(x, y)
        return SolveResult(True, 0, "plan-drag", mx, mx, n, 0, time.perf_counter() - t0, "plan")

    def end(self) -> None:
        if self.numeric is not None:
            self.numeric.end()

    @property
    def flips(self) -> list[tuple[Point, Point, Point]]:
        return self.numeric.flips if self.numeric is not None else []

    def branches(self) -> dict[str, int]:
        return self.solver.plan.branches()
