"""Stage 3a — cluster merging (Fudos–Hoffmann, generalised) and plan execution.

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
    of clusters that is rigid as a whole (Stage 3b — DR-planning / Owen's
    idea of isolating the non-tree-decomposable part): grow from each seed by
    generic-rank deficiency, take the smallest rigid subset found (capped in
    size — the numeric cost is exponential in exactly this), merge it as one
    numeric step, resume tree merging;
  * the merge sequence is the *plan*; the clusters left over are the roots.

Execution (every solve / drag frame, no graph analysis):
  * leaf poses from the live dimension values, warm-started on the current
    geometry (this is also what picks roots close to what the user sees);
  * PPP triangle merges by ruler-and-compass (circle–circle intersection) with
    an explicit chirality flag; other merges by a 3k-unknown DogLeg with
    analytic Jacobian (k = movable clusters);
  * unfixed roots placed by least-change (Procrustes onto current positions);
  * write back; verify with the compiled System; numeric fallback if needed.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from gcs import newton
from gcs.cgraph import ConstraintGraph, Edge, El, build, line_normal
from gcs.model import Arc, Circle, Sketch, Vec
from gcs.solve import Method, SolveResult, System

Pose = Vec  # point: (x, y); line: (nx, ny, c)
_ID = np.array([1.0, 0.0, 0.0, 0.0])


class Cluster:
    __slots__ = ("id", "els", "fixed")

    def __init__(self, cid: int, els: dict[El, Pose], fixed: bool) -> None:
        self.id, self.els, self.fixed = cid, els, fixed

    def __repr__(self) -> str:
        return f"Cluster({self.id}{'*' if self.fixed else ''} {sorted(self.els)})"


@dataclass
class Step:
    """Merge of clusters `ids` into `out`.  shared[(i, j)] = elements shared by ids[i], ids[j];
    dirs[(i, j)] = (line in i, line in j, phi) pairs with n_j = rot(phi)·n_i (direction classes)."""

    ids: tuple[int, ...]
    shared: dict[tuple[int, int], list[El]]
    dirs: dict[tuple[int, int], list[tuple[El, El, float]]]
    out: int


@dataclass
class Plan:
    graph: ConstraintGraph
    leaves: list[tuple[int, int]]          # (cluster id, edge index)
    ground_id: int
    singletons: list[tuple[int, El]]
    steps: list[Step]
    roots: list[int]
    chirality: dict[int, int] = field(default_factory=dict)   # step index → ±1 (PPP merges)

    @property
    def fully_decomposed(self) -> bool:
        return not self.graph.unsupported and len(self.roots) == 1

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
        # path compression with potentials
        acc = 0.0
        for x in reversed(path):
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
        # attach rb under ra: pot[rb] = angle(rb rel ra) = pa + phi − pb
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
        return _ID.copy()
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
# merge system (shared for the generic-rank decision and for execution)

def _merge_system(cl: list[Cluster], pairs: list[tuple[int, int, El]],
                  dpairs: list[tuple[int, int, El, El, float]], k_movable: int
                  ) -> tuple[Callable[[Vec], Vec], Callable[[Vec], Vec]]:
    """Residual/Jacobian callables for transforms of cl[1..] (cl[0] = reference, identity)."""

    def dpose(u: Vec, ci: int, el: El) -> tuple[Pose, Vec]:
        pose = cl[ci].els[el]
        if ci == 0:
            return pose, np.zeros((pose.size, 3))
        return _pose_jac(el, pose, *u[3 * (ci - 1): 3 * ci])

    def fun(u: Vec) -> Vec:
        parts = [dpose(u, i, e)[0] - dpose(u, j, e)[0] for i, j, e in pairs]
        for i, j, la, lb, phi in dpairs:      # angle(n_a', n_b') = phi — scalar, linear in the θ's
            na, nb = dpose(u, i, la)[0], dpose(u, j, lb)[0]
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
                  tol: float = 1e-13, max_iter: int = 40) -> Vec:
    """Plain min-norm Newton for the tiny merge systems (3k unknowns, warm-started at
    the identity).  No trust region: merges are near-linear from a warm start, and the
    plan is verified afterwards anyway (numeric fallback if anything went wrong)."""
    for _ in range(max_iter):
        r = fun(u)
        if r.size == 0 or float(np.abs(r).max()) < tol:
            break
        p, _ = newton.min_norm_lstsq(jac(u), -r)
        u = u + p
        if float(np.abs(p).max()) < 1e-15:
            break
    return u


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


# ---------------------------------------------------------------------------
# decomposition (topology only)

def decompose(graph: ConstraintGraph, seed: int = 0, core_max: int = 12) -> Plan:
    rng = np.random.default_rng(seed)
    dirs = _Dirs()
    for d in graph.dirs:
        dirs.join(d.a, d.b, d.phi)
    # generic (witness) poses: random points; lines get a random normal per direction class
    # (+ their class offset) and a random offset — merge decisions are structural, so they
    # must not depend on the user's possibly-degenerate geometry
    base_angle: dict[El, float] = {}
    generic: dict[El, Pose] = {}
    for e in graph.elements:
        if e.kind == "P":
            generic[e] = rng.uniform(-100, 100, 2)
        else:
            root, pot = dirs.find(e)
            ang = base_angle.setdefault(root, rng.uniform(0, 2 * math.pi)) + pot
            generic[e] = np.array([math.cos(ang), math.sin(ang), rng.uniform(-100, 100)])

    clusters: dict[int, Cluster] = {}
    of: dict[El, set[int]] = {e: set() for e in graph.elements}
    dir_of: dict[El, set[int]] = {}          # direction root → clusters containing a line of that class
    cdirs: dict[int, dict[El, El]] = {}      # cluster → {direction root: one of its lines}
    droot: dict[El, El] = {e: dirs.find(e)[0] for e in graph.elements if e.kind == "L"}
    next_id = 0

    def register(cid: int, els: list[El]) -> None:
        for e in els:
            of[e].add(cid)
            if e.kind == "L":
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
        return c

    ground = add({graph.X_AXIS, *(El("P", i) for i in graph.ground_points)}, True)
    leaves = [(add({e.a, e.b}, False), i) for i, e in enumerate(graph.edges)]
    singletons = [(add({e}, False), e) for e in graph.elements if not of[e]]
    steps: list[Step] = []

    def relations(ids: list[int]) -> tuple[list[tuple[int, int, El]], list[tuple[int, int, El, El, float]],
                                           dict[tuple[int, int], list[El]], dict[tuple[int, int], list[tuple[El, El, float]]]]:
        pairs, dpairs = [], []
        shared: dict[tuple[int, int], list[El]] = {}
        sdirs: dict[tuple[int, int], list[tuple[El, El, float]]] = {}
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                A, B = clusters[ids[i]], clusters[ids[j]]
                small, big = (A, B) if len(A.els) <= len(B.els) else (B, A)
                common = sorted(e for e in small.els if e in big.els)      # O(min) with dict membership
                if common:
                    shared[(i, j)] = common
                    pairs += [(i, j, e) for e in common]
                # one representative line pair per direction class shared but not via a common line
                seen = {droot[e] for e in common if e.kind == "L"}
                da, db = cdirs[ids[i]], cdirs[ids[j]]
                if len(da) > len(db):
                    da, db = db, da
                for root, la in da.items():
                    if root in seen or root not in db:
                        continue
                    la, lb = (la, db[root]) if la in A.els else (db[root], la)
                    phi = dirs.offset(la, lb)
                    assert phi is not None
                    sdirs.setdefault((i, j), []).append((la, lb, phi))
                    dpairs.append((i, j, la, lb, phi))
        return pairs, dpairs, shared, sdirs

    selfm: dict[int, int] = {}

    def self_motion(cid: int) -> int:
        if cid not in selfm:
            selfm[cid] = _self_motion(clusters[cid])
        return selfm[cid]

    def deficiency(ids: list[int]) -> int:
        """Relative rigid-transform DOF left after imposing everything the clusters share
        (0 ⇔ the merge is determined).  Generic rank of the merge Jacobian at witness poses."""
        cl = [clusters[i] for i in ids]
        ref = next((i for i, c in enumerate(cl) if c.fixed), max(range(len(cl)), key=lambda i: len(cl[i].els)))
        order = [ref] + [i for i in range(len(cl)) if i != ref]
        cl_o = [cl[i] for i in order]
        pos = {pi: k for k, pi in enumerate(order)}
        pairs, dpairs, _, _ = relations(ids)
        k = len(cl_o) - 1
        need = 3 * k - sum(self_motion(ids[i]) for i in order[1:])
        if need <= 0:
            return 0
        if 2 * len(pairs) + len(dpairs) < need:       # cheap upper bound on the rank
            return need - min(need, 2 * len(pairs) + len(dpairs))
        pairs = [(pos[i], pos[j], e) for i, j, e in pairs]
        dpairs = [(pos[i], pos[j], la, lb, phi) for i, j, la, lb, phi in dpairs]
        _, jac = _merge_system(cl_o, pairs, dpairs, k)
        J = jac(np.zeros(3 * k))
        rank = int(np.linalg.matrix_rank(J, tol=1e-7)) if J.size else 0
        return max(0, need - rank)

    def determined(ids: list[int]) -> bool:
        return deficiency(ids) == 0

    def merge(ids: list[int]) -> int:
        _, _, shared, sdirs = relations(ids)
        # small-into-large: keep the biggest (or fixed) cluster's dict, absorb the others
        keep = next((i for i in ids if clusters[i].fixed), max(ids, key=lambda i: len(clusters[i].els)))
        K = clusters[keep]
        selfm.pop(keep, None)
        for i in ids:
            if i == keep:
                continue
            c = remove(i)
            selfm.pop(i, None)
            new = [e for e in c.els if e not in K.els]
            for e in new:
                K.els[e] = c.els[e]
            register(keep, new)
            K.fixed = K.fixed or c.fixed
        steps.append(Step(tuple(ids), shared, sdirs, keep))
        return keep

    def neighbours(a: int) -> list[int]:
        A = clusters[a]
        nb: set[int] = set()
        for e in A.els:
            nb |= of[e]
        for r in cdirs[a]:
            nb |= dir_of.get(r, set())
        nb.discard(a)
        return sorted(nb)

    # worklist: a cluster is re-examined when it is created or one of its neighbours changes
    from collections import deque

    def tree_merges(seed_ids: list[int]) -> None:
        work: deque[int] = deque(seed_ids)
        queued = set(work)
        while work:
            a = work.popleft()
            queued.discard(a)
            if a not in clusters:
                continue
            nbs = neighbours(a)
            out = -1
            for b in nbs:                                   # pair merges
                if determined([a, b]):
                    out = merge([a, b])
                    break
            if out < 0:
                for i, b in enumerate(nbs):                 # triple merges
                    nb_b = set(neighbours(b))
                    for c in nbs[i + 1:]:
                        if c in nb_b and determined([a, b, c]):
                            out = merge([a, b, c])
                            break
                    if out >= 0:
                        break
            if out >= 0:
                for x in [out, *neighbours(out)]:
                    if x not in queued:
                        work.append(x)
                        queued.add(x)

    def find_core() -> list[int] | None:
        """Smallest rigid subset of ≥ 4 clusters found by greedy growth from every seed
        (pairs/triples are already exhausted).  None if nothing rigid within `core_max`."""
        best: list[int] | None = None
        live = [cid for cid in sorted(clusters)
                if not any(cid != o and set(clusters[cid].els) < set(clusters[o].els) for o in clusters)]
        if len(live) > 400:
            return None
        for seed in live:
            S = [seed]
            inS = {seed}
            while len(S) < core_max and (best is None or len(S) + 1 < len(best)):
                frontier = sorted({n for x in S for n in neighbours(x) if n not in inS})
                if not frontier:
                    break
                scored = sorted(((deficiency(S + [n]), len(clusters[n].els), n) for n in frontier))
                d, _, n = scored[0]
                S.append(n)
                inS.add(n)
                if d == 0:
                    if len(S) >= 2 and (best is None or len(S) < len(best)):
                        best = list(S)
                    break
        return best

    tree_merges(sorted(clusters))
    while True:
        core = find_core()
        if core is None:
            break
        out = merge(core)
        tree_merges([out, *neighbours(out)])
    roots = [cid for cid in sorted(clusters)
             if not any(cid != o and set(clusters[cid].els) < set(clusters[o].els) for o in clusters)]
    return Plan(graph, leaves, ground, singletons, steps, roots)


# ---------------------------------------------------------------------------
# execution

def _world_pose(graph: ConstraintGraph, e: El) -> Pose:
    if e == graph.X_AXIS:
        return np.array([0.0, 1.0, 0.0])
    if e.kind == "P":
        p = graph.members[e.idx][0]
        return np.array([p.x.value, p.y.value])
    if e.idx < len(graph.lines):
        return np.array(line_normal(graph.lines[e.idx]))
    a, b = (_world_pose(graph, q) for q in graph.virtual[e.idx])   # virtual line through two points
    d = b - a
    L = float(np.hypot(*d)) or 1.0
    n = np.array([-d[1] / L, d[0] / L])
    return np.array([n[0], n[1], float(n @ a)])


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


def _merge_ppp(ref: Cluster, B: Cluster, Cc: Cluster, x: El, y: El, z: El, sign: int) -> tuple[Vec, Vec, int]:
    """Triangle merge with all shared elements points: ref∩B = {x}, B∩C = {y}, C∩ref = {z}.
    In ref's frame y is a circle–circle intersection; `sign` (±1) is the chirality —
    the orientation of the triangle (x, z, y) — taken from the current sketch, which is
    invariant under rigid motions of the whole (unlike "nearest position")."""
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
    # orientation +1 ⇔ y lies to the left of x→z
    ya = np.array((fx - h * uy, fy + h * ux) if sign > 0 else (fx + h * uy, fy - h * ux))
    return _fit2(bx, by, xa, ya), _fit2(cz, cy, za, ya), sign


def execute(plan: Plan) -> None:
    """Replay the plan on the current sketch values and write the result back."""
    g = plan.graph
    cl: dict[int, Cluster] = {}
    gels = {g.X_AXIS: _world_pose(g, g.X_AXIS)}
    for i in g.ground_points:
        gels[El("P", i)] = _world_pose(g, El("P", i))
    cl[plan.ground_id] = Cluster(plan.ground_id, gels, True)
    for cid, ei in plan.leaves:
        cl[cid] = Cluster(cid, _leaf(g, g.edges[ei]), False)
    for cid, e in plan.singletons:
        cl[cid] = Cluster(cid, {e: _world_pose(g, e)}, False)

    for si, st in enumerate(plan.steps):
        parts = [cl.pop(i) for i in st.ids]
        ref_i = next((i for i, c in enumerate(parts) if c.fixed),
                     max(range(len(parts)), key=lambda i: len(parts[i].els)))
        order = [ref_i] + [i for i in range(len(parts)) if i != ref_i]
        pos = {pi: k for k, pi in enumerate(order)}
        cl_o = [parts[i] for i in order]
        pairs = [(pos[i], pos[j], e) for (i, j), els in st.shared.items() for e in els]
        dpairs = [(pos[i], pos[j], la, lb, phi) for (i, j), ds in st.dirs.items() for la, lb, phi in ds]
        movable = cl_o[1:]
        ppp = (len(parts) == 3 and not dpairs and len(pairs) == 3 and all(e.kind == "P" for _, _, e in pairs)
               and len({frozenset((i, j)) for i, j, _ in pairs}) == 3)
        if ppp:
            share = {frozenset((i, j)): e for i, j, e in pairs}
            x, z, y = share[frozenset((0, 1))], share[frozenset((0, 2))], share[frozenset((1, 2))]
            xw, zw, yw = _world_pose(g, x), _world_pose(g, z), _world_pose(g, y)
            orient = (zw[0] - xw[0]) * (yw[1] - xw[1]) - (zw[1] - xw[1]) * (yw[0] - xw[0])
            sign = 1 if orient >= 0 else -1                 # sketch-guided chirality
            TB, TC, sign = _merge_ppp(cl_o[0], cl_o[1], cl_o[2], x, y, z, sign)
            plan.chirality[si] = sign
            Ts = [TB, TC]
        elif movable:
            fun, jac = _merge_system(cl_o, pairs, dpairs, len(movable))
            u = _newton_small(fun, jac, np.zeros(3 * len(movable)))
            if float(np.abs(fun(u)).max(initial=0.0)) > 1e-9:
                # cores (many clusters) or bad warm starts: use the globalised solver
                u, _ = newton.dogleg(fun, jac, np.zeros(3 * len(movable)), ftol=1e-13, xtol=1e-14,
                                     gtol=1e-18, max_iter=300)
            Ts = [_T(u[3 * i], u[3 * i + 1], u[3 * i + 2]) for i in range(len(movable))]
        else:
            Ts = []
        els = dict(cl_o[0].els)
        for c, T in zip(movable, Ts, strict=True):
            for e, pose in c.els.items():
                if e not in els:
                    els[e] = _apply(T, e, pose)
        cl[st.out] = Cluster(st.out, els, any(c.fixed for c in parts))

    # -- place roots: fixed ones are in world frame; others least-change onto current geometry,
    #    aligning any element already placed by an earlier root exactly --
    placed: dict[El, Pose] = {}
    for rid in sorted(plan.roots, key=lambda i: (not cl[i].fixed, -len(cl[i].els), i)):
        c = cl[rid]
        if not c.fixed:
            pts = [e for e in c.els if e.kind == "P"]
            src = np.array([c.els[e] for e in pts]).reshape(-1, 2)
            dst = np.array([placed[e] if e in placed else _world_pose(g, e) for e in pts]).reshape(-1, 2)
            T = _procrustes(src, dst)
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


class PlanSolver:
    """Compile once per topology (graph + decomposition + System for verification);
    `solve()` replays the plan and falls back to the numeric core when the residual
    says the plan did not (fully) determine the sketch."""

    def __init__(self, sketch: Sketch) -> None:
        self.sketch = sketch
        self.graph = build(sketch)
        self.plan = decompose(self.graph)
        self.system = System(sketch)

    def solve(self, tol: float = 1e-9, fallback: bool = True, method: Method = "dogleg") -> PlanResult:
        t0 = time.perf_counter()
        execute(self.plan)
        self.system.refresh_consts()          # dimensions may have been edited since compile
        r = self.system.residuals(self.system.z0())
        rh = np.abs(r[self.system.hard])
        mx = float(rh.max()) if rh.size else 0.0
        scale = max(1.0, self.system.extent) ** 2
        num = None
        fell = False
        if mx > tol * scale and fallback:
            fell = True
            num = self.system.solve(method=method)
            mx = num.max_residual
        return PlanResult(mx <= 1e-6 * scale, mx, fell, time.perf_counter() - t0, self.plan, num)


def solve_plan(sketch: Sketch, **kw: object) -> PlanResult:
    return PlanSolver(sketch).solve(**kw)  # type: ignore[arg-type]
