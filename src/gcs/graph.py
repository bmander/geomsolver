"""Pure graph algorithms used by diagnosis: Hopcroft–Karp bipartite matching,
Dulmage–Mendelsohn coarse decomposition, and the (2,3) pebble game.

All inputs are plain integer adjacency lists so these stay independent of the
sketch object model (and portable later).  Iteration order is deterministic.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Sequence

INF = float("inf")


# ---------------------------------------------------------------------------
# Hopcroft–Karp

def hopcroft_karp(adj: Sequence[Sequence[int]], n_right: int) -> tuple[list[int], list[int]]:
    """Maximum bipartite matching.  adj[u] = right-vertices adjacent to left u.
    Returns (mate_left, mate_right) with -1 for unmatched."""
    n_left = len(adj)
    mate_l = [-1] * n_left
    mate_r = [-1] * n_right
    dist = [0] * n_left

    def bfs() -> bool:
        q: deque[int] = deque()
        found = False
        for u in range(n_left):
            if mate_l[u] < 0:
                dist[u] = 0
                q.append(u)
            else:
                dist[u] = -1
        while q:
            u = q.popleft()
            for v in adj[u]:
                w = mate_r[v]
                if w < 0:
                    found = True
                elif dist[w] < 0:
                    dist[w] = dist[u] + 1
                    q.append(w)
        return found

    def dfs(u: int) -> bool:
        for v in adj[u]:
            w = mate_r[v]
            if w < 0 or (dist[w] == dist[u] + 1 and dfs(w)):
                mate_l[u] = v
                mate_r[v] = u
                return True
        dist[u] = -1
        return False

    while bfs():
        for u in range(n_left):
            if mate_l[u] < 0:
                dfs(u)
    return mate_l, mate_r


# ---------------------------------------------------------------------------
# Dulmage–Mendelsohn (coarse)

@dataclass
class DM:
    """Coarse Dulmage–Mendelsohn decomposition of a bipartite graph rows × cols.

    over  : rows/cols reachable from an unmatched *row* by alternating paths — the
            over-determined (vertical) block: |rows| > |cols|, rows−cols redundant equations.
    under : rows/cols reachable from an unmatched *column* — the under-determined
            (horizontal) block: |cols| > |rows|, cols−rows structurally free parameters.
    well  : everything else (square, perfectly matched).
    """

    mate_row: list[int]
    mate_col: list[int]
    over_rows: list[int] = field(default_factory=list)
    over_cols: list[int] = field(default_factory=list)
    under_rows: list[int] = field(default_factory=list)
    under_cols: list[int] = field(default_factory=list)
    well_rows: list[int] = field(default_factory=list)
    well_cols: list[int] = field(default_factory=list)

    @property
    def rank(self) -> int:
        return sum(1 for m in self.mate_row if m >= 0)

    @property
    def n_redundant(self) -> int:
        return len(self.over_rows) - len(self.over_cols)

    @property
    def n_free(self) -> int:
        return len(self.under_cols) - len(self.under_rows)


def dulmage_mendelsohn(adj: Sequence[Sequence[int]], n_cols: int) -> DM:
    """adj[row] = columns with a nonzero in that row (structural Jacobian)."""
    n_rows = len(adj)
    mate_r, mate_c = hopcroft_karp(adj, n_cols)
    col_adj: list[list[int]] = [[] for _ in range(n_cols)]
    for r, cols in enumerate(adj):
        for c in cols:
            col_adj[c].append(r)

    # over: alternating BFS from unmatched rows: row -(any)-> col -(matching)-> row
    o_rows, o_cols = [False] * n_rows, [False] * n_cols
    q = deque(r for r in range(n_rows) if mate_r[r] < 0)
    for r in q:
        o_rows[r] = True
    while q:
        r = q.popleft()
        for c in adj[r]:
            if not o_cols[c]:
                o_cols[c] = True
                r2 = mate_c[c]
                if r2 >= 0 and not o_rows[r2]:
                    o_rows[r2] = True
                    q.append(r2)
    # under: alternating BFS from unmatched cols: col -(any)-> row -(matching)-> col
    u_rows, u_cols = [False] * n_rows, [False] * n_cols
    q = deque(c for c in range(n_cols) if mate_c[c] < 0)
    for c in q:
        u_cols[c] = True
    while q:
        c = q.popleft()
        for r in col_adj[c]:
            if not u_rows[r]:
                u_rows[r] = True
                c2 = mate_r[r]
                if c2 >= 0 and not u_cols[c2]:
                    u_cols[c2] = True
                    q.append(c2)
    dm = DM(mate_r, mate_c)
    for r in range(n_rows):
        (dm.over_rows if o_rows[r] else dm.under_rows if u_rows[r] else dm.well_rows).append(r)
    for c in range(n_cols):
        (dm.over_cols if o_cols[c] else dm.under_cols if u_cols[c] else dm.well_cols).append(c)
    return dm


# ---------------------------------------------------------------------------
# Connected components of a bipartite graph (rows ∪ cols)

def bipartite_components(adj: Sequence[Sequence[int]], n_cols: int) -> tuple[list[int], list[int], int]:
    """Union-find over rows and columns.  Returns (comp_of_row, comp_of_col, n_components);
    isolated columns get their own component."""
    n_rows = len(adj)
    parent = list(range(n_rows + n_cols))

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for r, cols in enumerate(adj):
        for c in cols:
            ra, rb = find(r), find(n_rows + c)
            if ra != rb:
                parent[rb] = ra
    roots: dict[int, int] = {}
    comp = [roots.setdefault(find(i), len(roots)) for i in range(n_rows + n_cols)]
    return comp[:n_rows], comp[n_rows:], len(roots)


# ---------------------------------------------------------------------------
# (2,3) pebble game — Jacobs & Hendrickson 1997; component detection per Lee & Streinu 2008

@dataclass
class PebbleResult:
    independent: list[tuple[int, int]]      # edges accepted (in insertion order)
    redundant: list[tuple[int, int]]        # edges rejected: dependent on earlier ones
    components: list[frozenset[int]]        # maximal rigid clusters (size ≥ 2)
    dof: int                                # 2n − 3 − |independent| for n ≥ 2 (0 if fully rigid)

    def is_rigid(self) -> bool:
        return self.dof == 0


def pebble_game(n: int, edges: Sequence[tuple[int, int]]) -> PebbleResult:
    """(k=2, l=3) pebble game on a graph with vertices 0..n−1.  Decides generic
    rigidity of bar frameworks in the plane (Laman): an edge is independent iff
    4 pebbles can be gathered on its endpoints; a rigid component is found when,
    after inserting an edge, no free pebble outside its endpoints is reachable."""
    peb = [2] * n
    out: list[set[int]] = [set() for _ in range(n)]
    independent: list[tuple[int, int]] = []
    redundant: list[tuple[int, int]] = []
    components: list[frozenset[int]] = []

    def find_pebble(src: int, exclude: int) -> bool:
        """DFS from src for a vertex (≠ src, exclude) holding a pebble; on success move
        it to src by reversing the path.  Returns True if a pebble was moved."""
        stack = [src]
        seen = {src, exclude}
        parent: dict[int, int] = {}
        while stack:
            u = stack.pop()
            for w in out[u]:
                if w in seen:
                    continue
                seen.add(w)
                parent[w] = u
                if peb[w] > 0:
                    # reverse the path src → ... → w
                    peb[w] -= 1
                    peb[src] += 1
                    x = w
                    while x != src:
                        p = parent[x]
                        out[p].discard(x)
                        out[x].add(p)
                        x = p
                    return True
                stack.append(w)
        return False

    def reach(srcs: Sequence[int]) -> set[int]:
        seen = set(srcs)
        stack = list(srcs)
        while stack:
            u = stack.pop()
            for w in out[u]:
                if w not in seen:
                    seen.add(w)
                    stack.append(w)
        return seen

    for u, v in edges:
        if u == v:
            redundant.append((u, v))
            continue
        # skip work if the edge is already inside a rigid component (dependent)
        if any(u in comp and v in comp for comp in components):
            redundant.append((u, v))
            continue
        while peb[u] + peb[v] < 4:
            if not (find_pebble(u, v) or find_pebble(v, u)):
                break
        if peb[u] + peb[v] < 4:
            redundant.append((u, v))
            continue
        # accept: orient u -> v, consuming a pebble from u
        peb[u] -= 1
        out[u].add(v)
        independent.append((u, v))
        # component detection: u,v now hold exactly 3 pebbles.  If some other free
        # pebble is reachable, no new component; else the component is every vertex
        # that cannot reach a free pebble outside {u, v}.
        R = reach([u, v])
        if any(peb[w] > 0 for w in R if w not in (u, v)):
            continue
        # backward search from free-pebble vertices (other than u, v)
        free = [w for w in range(n) if peb[w] > 0 and w not in (u, v)]
        rev: list[list[int]] = [[] for _ in range(n)]
        for a in range(n):
            for b in out[a]:
                rev[b].append(a)
        can_reach_free = set(free)
        stack = list(free)
        while stack:
            b = stack.pop()
            for a in rev[b]:
                if a not in can_reach_free:
                    can_reach_free.add(a)
                    stack.append(a)
        comp = frozenset(w for w in range(n) if w not in can_reach_free)
        components = [c for c in components if not c <= comp] + [comp]
    dof = max(0, 2 * n - 3 - len(independent)) if n >= 2 else 0
    return PebbleResult(independent, redundant, sorted(components, key=lambda c: (min(c), len(c))), dof)
