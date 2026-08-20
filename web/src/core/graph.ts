/* Pure graph algorithms used by diagnosis: Hopcroft–Karp bipartite matching, the coarse
 * Dulmage–Mendelsohn decomposition, connected components and the (2,3) pebble game.
 *
 * Inputs are plain integer adjacency lists, so these stay independent of the sketch object model.
 * The algorithms live in the core; iteration order is deterministic. */
import { core, takeJson, withJson } from './wasm.js';

/** Maximum bipartite matching.  `adj[u]` = right vertices adjacent to left u; -1 = unmatched. */
export function hopcroftKarp(adj: readonly (readonly number[])[], nRight: number):
    { mateL: number[]; mateR: number[] } {
  return withJson(adj, (p, n) =>
    takeJson<{ mateL: number[]; mateR: number[] }>(core().gcs_hopcroft_karp_json(p, n, nRight)));
}

export interface DM {
  mateRow: number[];
  mateCol: number[];
  overRows: number[];
  overCols: number[];
  underRows: number[];
  underCols: number[];
  wellRows: number[];
  wellCols: number[];
  rank: number;
  nRedundant: number;
  nFree: number;
}

/** Coarse Dulmage–Mendelsohn decomposition of a bipartite graph rows x cols. */
export function dulmageMendelsohn(adj: readonly (readonly number[])[], nCols: number): DM {
  return withJson(adj, (p, n) => takeJson<DM>(core().gcs_dulmage_mendelsohn_json(p, n, nCols)));
}

export function bipartiteComponents(adj: readonly (readonly number[])[], nCols: number):
    { compRow: number[]; compCol: number[]; count: number } {
  return withJson(adj, (p, n) => takeJson(core().gcs_bipartite_components_json(p, n, nCols)));
}

export interface PebbleResult {
  independent: number[];      /* edge indices accepted, in insertion order */
  redundant: number[];        /* edge indices rejected: dependent on earlier ones */
  components: number[][];     /* maximal rigid clusters (size >= 2), each sorted */
  dof: number;                /* 2n - 3 - |independent| for n >= 2 */
  isRigid: boolean;
}

/** (k=2, l=3) pebble game (Jacobs & Hendrickson; components per Lee & Streinu). */
export function pebbleGame(n: number,
                           edges: readonly (readonly [number, number])[]): PebbleResult {
  return withJson(edges, (p, len) => takeJson<PebbleResult>(core().gcs_pebble_game_json(n, p, len)));
}
