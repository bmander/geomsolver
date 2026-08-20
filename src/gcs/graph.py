"""Pure graph algorithms used by diagnosis: Hopcroft–Karp bipartite matching, the coarse
Dulmage–Mendelsohn decomposition, connected components and the (2,3) pebble game.

Inputs are plain integer adjacency lists, so these stay independent of the sketch object model.
The algorithms live in the core; iteration order is deterministic.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from gcs import _ffi
from gcs._ffi import lib


def hopcroft_karp(adj: Sequence[Sequence[int]], n_right: int) -> tuple[list[int], list[int]]:
    """Maximum bipartite matching.  `adj[u]` = right vertices adjacent to left u.  Returns
    (mate_left, mate_right) with -1 for unmatched."""
    p, n = _ffi.send_json([list(r) for r in adj])
    d = _ffi.take_json(lib.gcs_hopcroft_karp_json(p, n, int(n_right)))
    return d["mateL"], d["mateR"]


@dataclass
class DM:
    mate_row: list[int]
    mate_col: list[int]
    over_rows: list[int]
    over_cols: list[int]
    under_rows: list[int]
    under_cols: list[int]
    well_rows: list[int]
    well_cols: list[int]
    rank: int
    n_redundant: int
    n_free: int


def dulmage_mendelsohn(adj: Sequence[Sequence[int]], n_cols: int) -> DM:
    """Coarse Dulmage–Mendelsohn decomposition of a bipartite graph rows x cols.

    over  : rows/cols reachable from an unmatched row (the difference is redundant equations);
    under : rows/cols reachable from an unmatched column (the difference is free parameters);
    well  : everything else (square, perfectly matched)."""
    p, n = _ffi.send_json([list(r) for r in adj])
    d = _ffi.take_json(lib.gcs_dulmage_mendelsohn_json(p, n, int(n_cols)))
    return DM(d["mateRow"], d["mateCol"], d["overRows"], d["overCols"], d["underRows"],
              d["underCols"], d["wellRows"], d["wellCols"], d["rank"], d["nRedundant"],
              d["nFree"])


@dataclass
class Components:
    comp_row: list[int]
    comp_col: list[int]
    count: int


def bipartite_components(adj: Sequence[Sequence[int]], n_cols: int) -> Components:
    p, n = _ffi.send_json([list(r) for r in adj])
    d = _ffi.take_json(lib.gcs_bipartite_components_json(p, n, int(n_cols)))
    return Components(d["compRow"], d["compCol"], d["count"])


@dataclass
class PebbleResult:
    independent: list[int]              # edge indices accepted, in insertion order
    redundant: list[int]                # edge indices rejected: dependent on earlier ones
    components: list[frozenset[int]]    # maximal rigid clusters (size >= 2)
    dof: int                            # 2n - 3 - |independent| for n >= 2

    def is_rigid(self) -> bool:
        return self.dof == 0


def pebble_game(n: int, edges: Sequence[tuple[int, int]]) -> PebbleResult:
    """(k=2, l=3) pebble game (Jacobs & Hendrickson; components per Lee & Streinu).  Decides
    generic rigidity of bar frameworks in the plane."""
    p, ln = _ffi.send_json([[int(a), int(b)] for a, b in edges])
    d = _ffi.take_json(lib.gcs_pebble_game_json(int(n), p, ln))
    return PebbleResult(d["independent"], d["redundant"],
                        [frozenset(c) for c in d["components"]], d["dof"])


__all__ = ["Components", "DM", "PebbleResult", "bipartite_components", "dulmage_mendelsohn",
           "hopcroft_karp", "pebble_game"]
