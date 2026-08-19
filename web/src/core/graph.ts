/* Pure graph algorithms used by diagnosis: Hopcroft-Karp bipartite matching, the coarse
 * Dulmage-Mendelsohn decomposition, connected components and the (2,3) pebble game.
 *
 * Inputs are plain integer adjacency lists so these stay independent of the sketch object
 * model.  Iteration order is deterministic. */

export class UnionFind {
  parent: Int32Array;

  constructor(n: number) {
    this.parent = new Int32Array(n);
    for (let i = 0; i < n; i++) this.parent[i] = i;
  }

  find(a: number): number {
    const p = this.parent;
    while (p[a] !== a) {
      p[a] = p[p[a]];
      a = p[a];
    }
    return a;
  }

  union(a: number, b: number): void {
    const ra = this.find(a), rb = this.find(b);
    if (ra !== rb) this.parent[rb] = ra;
  }

  /** Dense component label per element (0..k-1 in first-seen order) and k. */
  labels(): { label: Int32Array; count: number } {
    const roots = new Map<number, number>();
    const label = new Int32Array(this.parent.length);
    for (let i = 0; i < this.parent.length; i++) {
      const r = this.find(i);
      let l = roots.get(r);
      if (l === undefined) roots.set(r, (l = roots.size));
      label[i] = l;
    }
    return { label, count: roots.size };
  }
}

/* -- Hopcroft-Karp ---------------------------------------------------------- */

/** Maximum bipartite matching.  adj[u] = right vertices adjacent to left u.
 *  Returns mates with -1 for unmatched. */
export function hopcroftKarp(adj: readonly (readonly number[])[], nRight: number):
    { mateL: Int32Array; mateR: Int32Array } {
  const nLeft = adj.length;
  const mateL = new Int32Array(nLeft).fill(-1);
  const mateR = new Int32Array(nRight).fill(-1);
  const dist = new Int32Array(nLeft);

  const bfs = (): boolean => {
    const q: number[] = [];
    let head = 0;
    let found = false;
    for (let u = 0; u < nLeft; u++) {
      if (mateL[u] < 0) { dist[u] = 0; q.push(u); } else dist[u] = -1;
    }
    while (head < q.length) {
      const u = q[head++];
      for (const v of adj[u]) {
        const w = mateR[v];
        if (w < 0) found = true;
        else if (dist[w] < 0) { dist[w] = dist[u] + 1; q.push(w); }
      }
    }
    return found;
  };

  const dfs = (u: number): boolean => {
    for (const v of adj[u]) {
      const w = mateR[v];
      if (w < 0 || (dist[w] === dist[u] + 1 && dfs(w))) {
        mateL[u] = v;
        mateR[v] = u;
        return true;
      }
    }
    dist[u] = -1;
    return false;
  };

  while (bfs()) for (let u = 0; u < nLeft; u++) if (mateL[u] < 0) dfs(u);
  return { mateL, mateR };
}

/* -- Dulmage-Mendelsohn (coarse) -------------------------------------------- */

/** Coarse Dulmage-Mendelsohn decomposition of a bipartite graph rows x cols.
 *
 *  over  : rows/cols reachable from an unmatched row by alternating paths — the
 *          over-determined block (|rows| > |cols|, the difference is redundant equations).
 *  under : rows/cols reachable from an unmatched column — the under-determined block
 *          (|cols| > |rows|, the difference is structurally free parameters).
 *  well  : everything else (square, perfectly matched). */
export interface DM {
  mateRow: Int32Array;
  mateCol: Int32Array;
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

export function dulmageMendelsohn(adj: readonly (readonly number[])[], nCols: number): DM {
  const nRows = adj.length;
  const { mateL: mateRow, mateR: mateCol } = hopcroftKarp(adj, nCols);
  const colAdj: number[][] = Array.from({ length: nCols }, () => []);
  for (let r = 0; r < nRows; r++) for (const c of adj[r]) colAdj[c].push(r);

  // over: alternating BFS from unmatched rows: row -(any)-> col -(matching)-> row
  const oRows = new Uint8Array(nRows), oCols = new Uint8Array(nCols);
  const q1: number[] = [];
  for (let r = 0; r < nRows; r++) if (mateRow[r] < 0) { oRows[r] = 1; q1.push(r); }
  for (let h = 0; h < q1.length; h++) {
    const r = q1[h];
    for (const c of adj[r]) {
      if (oCols[c]) continue;
      oCols[c] = 1;
      const r2 = mateCol[c];
      if (r2 >= 0 && !oRows[r2]) { oRows[r2] = 1; q1.push(r2); }
    }
  }
  // under: alternating BFS from unmatched cols: col -(any)-> row -(matching)-> col
  const uRows = new Uint8Array(nRows), uCols = new Uint8Array(nCols);
  const q2: number[] = [];
  for (let c = 0; c < nCols; c++) if (mateCol[c] < 0) { uCols[c] = 1; q2.push(c); }
  for (let h = 0; h < q2.length; h++) {
    const c = q2[h];
    for (const r of colAdj[c]) {
      if (uRows[r]) continue;
      uRows[r] = 1;
      const c2 = mateRow[r];
      if (c2 >= 0 && !uCols[c2]) { uCols[c2] = 1; q2.push(c2); }
    }
  }
  const dm: DM = {
    mateRow, mateCol,
    overRows: [], overCols: [], underRows: [], underCols: [], wellRows: [], wellCols: [],
    rank: 0, nRedundant: 0, nFree: 0,
  };
  for (let r = 0; r < nRows; r++) (oRows[r] ? dm.overRows : uRows[r] ? dm.underRows : dm.wellRows).push(r);
  for (let c = 0; c < nCols; c++) (oCols[c] ? dm.overCols : uCols[c] ? dm.underCols : dm.wellCols).push(c);
  let rank = 0;
  for (let r = 0; r < nRows; r++) if (mateRow[r] >= 0) rank++;
  dm.rank = rank;
  dm.nRedundant = dm.overRows.length - dm.overCols.length;
  dm.nFree = dm.underCols.length - dm.underRows.length;
  return dm;
}

/* -- connected components of a bipartite graph ------------------------------- */

export function bipartiteComponents(adj: readonly (readonly number[])[], nCols: number):
    { compRow: Int32Array; compCol: Int32Array; count: number } {
  const nRows = adj.length;
  const uf = new UnionFind(nRows + nCols);
  for (let r = 0; r < nRows; r++) for (const c of adj[r]) uf.union(r, nRows + c);
  const { label, count } = uf.labels();
  return { compRow: label.slice(0, nRows), compCol: label.slice(nRows), count };
}

/* -- (2,3) pebble game ------------------------------------------------------ */

export interface PebbleResult {
  independent: number[];          /* edge indices accepted, in insertion order */
  redundant: number[];            /* edge indices rejected: dependent on earlier ones */
  components: number[][];         /* maximal rigid clusters (size >= 2), each sorted */
  dof: number;                    /* 2n - 3 - |independent| for n >= 2 */
}

/** (k=2, l=3) pebble game (Jacobs & Hendrickson; components per Lee & Streinu).  Decides
 *  generic rigidity of bar frameworks in the plane: an edge is independent iff 4 pebbles
 *  can be gathered on its endpoints; a rigid component is found when, after inserting an
 *  edge, no free pebble outside its endpoints is reachable. */
export function pebbleGame(n: number, edges: readonly (readonly [number, number])[]): PebbleResult {
  const peb = new Int32Array(n).fill(2);
  const out: Set<number>[] = Array.from({ length: n }, () => new Set());
  const rev: Set<number>[] = Array.from({ length: n }, () => new Set());
  const independent: number[] = [];
  const redundant: number[] = [];
  let components: number[][] = [];
  const compsOf: Set<number>[] = Array.from({ length: n }, () => new Set());

  const orient = (a: number, b: number): void => { out[a].add(b); rev[b].add(a); };
  const unorient = (a: number, b: number): void => { out[a].delete(b); rev[b].delete(a); };

  /** DFS from src for a vertex (other than src, exclude) holding a pebble; on success move
   *  it to src by reversing the path. */
  const findPebble = (src: number, exclude: number): boolean => {
    const stack = [src];
    const seen = new Set([src, exclude]);
    const parent = new Map<number, number>();
    while (stack.length) {
      const u = stack.pop()!;
      for (const w of out[u]) {
        if (seen.has(w)) continue;
        seen.add(w);
        parent.set(w, u);
        if (peb[w] > 0) {
          peb[w]--;
          peb[src]++;
          let x = w;
          while (x !== src) {
            const p = parent.get(x)!;
            unorient(p, x);
            orient(x, p);
            x = p;
          }
          return true;
        }
        stack.push(w);
      }
    }
    return false;
  };

  const reach = (srcs: number[]): Set<number> => {
    const seen = new Set(srcs);
    const stack = [...srcs];
    while (stack.length) {
      const u = stack.pop()!;
      for (const w of out[u]) if (!seen.has(w)) { seen.add(w); stack.push(w); }
    }
    return seen;
  };

  const shareComponent = (u: number, v: number): boolean => {
    for (const c of compsOf[u]) if (compsOf[v].has(c)) return true;
    return false;
  };

  for (let ei = 0; ei < edges.length; ei++) {
    const [u, v] = edges[ei];
    if (u === v || shareComponent(u, v)) { redundant.push(ei); continue; }
    while (peb[u] + peb[v] < 4) if (!(findPebble(u, v) || findPebble(v, u))) break;
    if (peb[u] + peb[v] < 4) { redundant.push(ei); continue; }
    peb[u]--;
    orient(u, v);
    independent.push(ei);
    // component detection: u,v now hold exactly 3 pebbles.  If some other free pebble is
    // reachable, no new component; else the component is every vertex that cannot reach a
    // free pebble outside {u, v}.
    const R = reach([u, v]);
    let other = false;
    for (const w of R) if (w !== u && w !== v && peb[w] > 0) { other = true; break; }
    if (other) continue;
    const free: number[] = [];
    for (let w = 0; w < n; w++) if (peb[w] > 0 && w !== u && w !== v) free.push(w);
    const canReachFree = new Set(free);
    const stack = [...free];
    while (stack.length) {
      const b = stack.pop()!;
      for (const a of rev[b]) if (!canReachFree.has(a)) { canReachFree.add(a); stack.push(a); }
    }
    const comp: number[] = [];
    for (let w = 0; w < n; w++) if (!canReachFree.has(w)) comp.push(w);
    const cset = new Set(comp);
    const subsumed = (c: number[]): boolean => c.every((w) => cset.has(w));
    components = components.filter((c) => !subsumed(c));
    components.push(comp);
    for (let w = 0; w < n; w++) compsOf[w].clear();
    components.forEach((c, ci) => { for (const w of c) compsOf[w].add(ci); });
  }
  const dof = n >= 2 ? Math.max(0, 2 * n - 3 - independent.length) : 0;
  components.sort((a, b) => (a[0] - b[0]) || (a.length - b.length));
  return { independent, redundant, components, dof };
}
