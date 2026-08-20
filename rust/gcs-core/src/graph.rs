//! Pure graph algorithms used by diagnosis: Hopcroft–Karp bipartite matching, the coarse
//! Dulmage–Mendelsohn decomposition, connected components and the (2,3) pebble game.
//!
//! Inputs are plain integer adjacency lists so these stay independent of the sketch object model.
//! Iteration order is deterministic.

use std::collections::BTreeSet;

pub struct UnionFind {
    pub parent: Vec<i32>,
}

impl UnionFind {
    pub fn new(n: usize) -> UnionFind {
        UnionFind { parent: (0..n as i32).collect() }
    }

    pub fn find(&mut self, a: usize) -> usize {
        let mut a = a;
        while self.parent[a] as usize != a {
            let g = self.parent[self.parent[a] as usize];
            self.parent[a] = g;
            a = g as usize;
        }
        a
    }

    pub fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.parent[rb] = ra as i32;
        }
    }

    /// Dense component label per element (0..k-1 in first-seen order) and k.
    pub fn labels(&mut self) -> (Vec<usize>, usize) {
        let n = self.parent.len();
        let mut roots: Vec<(usize, usize)> = Vec::new();
        let mut label = vec![0usize; n];
        for i in 0..n {
            let r = self.find(i);
            let l = match roots.iter().find(|&&(k, _)| k == r) {
                Some(&(_, v)) => v,
                None => {
                    let v = roots.len();
                    roots.push((r, v));
                    v
                }
            };
            label[i] = l;
        }
        (label, roots.len())
    }
}

/* -- Hopcroft–Karp ---------------------------------------------------------- */

/// Maximum bipartite matching.  `adj[u]` = right vertices adjacent to left u.  Returns mates with
/// -1 for unmatched.
pub fn hopcroft_karp(adj: &[Vec<usize>], n_right: usize) -> (Vec<i32>, Vec<i32>) {
    let n_left = adj.len();
    let mut mate_l = vec![-1i32; n_left];
    let mut mate_r = vec![-1i32; n_right];
    let mut dist = vec![0i32; n_left];

    loop {
        // BFS layering from the unmatched left vertices
        let mut q: Vec<usize> = Vec::new();
        let mut found = false;
        for u in 0..n_left {
            if mate_l[u] < 0 {
                dist[u] = 0;
                q.push(u);
            } else {
                dist[u] = -1;
            }
        }
        let mut head = 0;
        while head < q.len() {
            let u = q[head];
            head += 1;
            for &v in &adj[u] {
                let w = mate_r[v];
                if w < 0 {
                    found = true;
                } else if dist[w as usize] < 0 {
                    dist[w as usize] = dist[u] + 1;
                    q.push(w as usize);
                }
            }
        }
        if !found {
            break;
        }
        for u in 0..n_left {
            if mate_l[u] < 0 {
                dfs_augment(adj, u, &mut mate_l, &mut mate_r, &mut dist);
            }
        }
    }
    (mate_l, mate_r)
}

/// Iterative augmenting DFS along the BFS layers (recursion would blow the stack on the large
/// sketches the program targets).
fn dfs_augment(
    adj: &[Vec<usize>],
    root: usize,
    mate_l: &mut [i32],
    mate_r: &mut [i32],
    dist: &mut [i32],
) -> bool {
    // stack of (left vertex, next adjacency index to try) — the recursion, unrolled
    let mut stack: Vec<(usize, usize)> = vec![(root, 0)];
    while !stack.is_empty() {
        let (u, i) = *stack.last().unwrap();
        if i >= adj[u].len() {
            dist[u] = -1;
            stack.pop();
            if let Some(top) = stack.last_mut() {
                top.1 += 1;
            }
            continue;
        }
        let v = adj[u][i];
        let w = mate_r[v];
        if w < 0 {
            // free right vertex: every frame on the stack takes the edge it was trying
            for &(uu, ii) in stack.iter() {
                let vv = adj[uu][ii];
                mate_l[uu] = vv as i32;
                mate_r[vv] = uu as i32;
            }
            return true;
        }
        let w = w as usize;
        if dist[w] == dist[u] + 1 {
            stack.push((w, 0));
        } else {
            stack.last_mut().unwrap().1 += 1;
        }
    }
    false
}

/* -- Dulmage–Mendelsohn (coarse) -------------------------------------------- */

/// Coarse Dulmage–Mendelsohn decomposition of a bipartite graph rows x cols.
///
/// * `over`  — rows/cols reachable from an unmatched row by alternating paths: the
///   over-determined block (the difference is redundant equations);
/// * `under` — rows/cols reachable from an unmatched column: the under-determined block (the
///   difference is structurally free parameters);
/// * `well`  — everything else (square, perfectly matched).
#[derive(Clone, Debug)]
pub struct Dm {
    pub mate_row: Vec<i32>,
    pub mate_col: Vec<i32>,
    pub over_rows: Vec<usize>,
    pub over_cols: Vec<usize>,
    pub under_rows: Vec<usize>,
    pub under_cols: Vec<usize>,
    pub well_rows: Vec<usize>,
    pub well_cols: Vec<usize>,
    pub rank: usize,
    pub n_redundant: i64,
    pub n_free: i64,
}

pub fn dulmage_mendelsohn(adj: &[Vec<usize>], n_cols: usize) -> Dm {
    let n_rows = adj.len();
    let (mate_row, mate_col) = hopcroft_karp(adj, n_cols);
    let mut col_adj: Vec<Vec<usize>> = vec![Vec::new(); n_cols];
    for r in 0..n_rows {
        for &c in &adj[r] {
            col_adj[c].push(r);
        }
    }
    // over: alternating BFS from unmatched rows: row -(any)-> col -(matching)-> row
    let mut o_rows = vec![false; n_rows];
    let mut o_cols = vec![false; n_cols];
    let mut q: Vec<usize> = Vec::new();
    for r in 0..n_rows {
        if mate_row[r] < 0 {
            o_rows[r] = true;
            q.push(r);
        }
    }
    let mut h = 0;
    while h < q.len() {
        let r = q[h];
        h += 1;
        for &c in &adj[r] {
            if o_cols[c] {
                continue;
            }
            o_cols[c] = true;
            let r2 = mate_col[c];
            if r2 >= 0 && !o_rows[r2 as usize] {
                o_rows[r2 as usize] = true;
                q.push(r2 as usize);
            }
        }
    }
    // under: alternating BFS from unmatched cols: col -(any)-> row -(matching)-> col
    let mut u_rows = vec![false; n_rows];
    let mut u_cols = vec![false; n_cols];
    let mut q2: Vec<usize> = Vec::new();
    for c in 0..n_cols {
        if mate_col[c] < 0 {
            u_cols[c] = true;
            q2.push(c);
        }
    }
    let mut h = 0;
    while h < q2.len() {
        let c = q2[h];
        h += 1;
        for &r in &col_adj[c] {
            if u_rows[r] {
                continue;
            }
            u_rows[r] = true;
            let c2 = mate_row[r];
            if c2 >= 0 && !u_cols[c2 as usize] {
                u_cols[c2 as usize] = true;
                q2.push(c2 as usize);
            }
        }
    }
    let mut dm = Dm {
        mate_row,
        mate_col,
        over_rows: Vec::new(),
        over_cols: Vec::new(),
        under_rows: Vec::new(),
        under_cols: Vec::new(),
        well_rows: Vec::new(),
        well_cols: Vec::new(),
        rank: 0,
        n_redundant: 0,
        n_free: 0,
    };
    for r in 0..n_rows {
        if o_rows[r] {
            dm.over_rows.push(r)
        } else if u_rows[r] {
            dm.under_rows.push(r)
        } else {
            dm.well_rows.push(r)
        }
    }
    for c in 0..n_cols {
        if o_cols[c] {
            dm.over_cols.push(c)
        } else if u_cols[c] {
            dm.under_cols.push(c)
        } else {
            dm.well_cols.push(c)
        }
    }
    dm.rank = dm.mate_row.iter().filter(|&&m| m >= 0).count();
    dm.n_redundant = dm.over_rows.len() as i64 - dm.over_cols.len() as i64;
    dm.n_free = dm.under_cols.len() as i64 - dm.under_rows.len() as i64;
    dm
}

/* -- connected components of a bipartite graph ------------------------------- */

pub struct BipComponents {
    pub comp_row: Vec<usize>,
    pub comp_col: Vec<usize>,
    pub count: usize,
}

pub fn bipartite_components(adj: &[Vec<usize>], n_cols: usize) -> BipComponents {
    let n_rows = adj.len();
    let mut uf = UnionFind::new(n_rows + n_cols);
    for r in 0..n_rows {
        for &c in &adj[r] {
            uf.union(r, n_rows + c);
        }
    }
    let (label, count) = uf.labels();
    BipComponents {
        comp_row: label[..n_rows].to_vec(),
        comp_col: label[n_rows..].to_vec(),
        count,
    }
}

/* -- (2,3) pebble game ------------------------------------------------------ */

#[derive(Clone, Debug)]
pub struct PebbleResult {
    /// Edge indices accepted, in insertion order.
    pub independent: Vec<usize>,
    /// Edge indices rejected: dependent on earlier ones.
    pub redundant: Vec<usize>,
    /// Maximal rigid clusters (size >= 2), each sorted.
    pub components: Vec<Vec<usize>>,
    /// 2n - 3 - |independent| for n >= 2.
    pub dof: usize,
}

impl PebbleResult {
    pub fn is_rigid(&self) -> bool {
        self.dof == 0
    }
}

/// (k=2, l=3) pebble game (Jacobs & Hendrickson; components per Lee & Streinu).  Decides generic
/// rigidity of bar frameworks in the plane: an edge is independent iff 4 pebbles can be gathered
/// on its endpoints; a rigid component is found when, after inserting an edge, no free pebble
/// outside its endpoints is reachable.
pub fn pebble_game(n: usize, edges: &[(usize, usize)]) -> PebbleResult {
    let mut peb = vec![2i32; n];
    let mut out: Vec<BTreeSet<usize>> = vec![BTreeSet::new(); n];
    let mut rev: Vec<BTreeSet<usize>> = vec![BTreeSet::new(); n];
    let mut independent = Vec::new();
    let mut redundant = Vec::new();
    // components by identity: a vertex records which component *slots* contain it, so subsuming
    // one only touches its own members rather than all n vertices
    let mut components: Vec<Vec<usize>> = Vec::new();
    let mut comps_of: Vec<BTreeSet<usize>> = vec![BTreeSet::new(); n];
    let mut alive: Vec<bool> = Vec::new();
    let mut free_pebbles: BTreeSet<usize> = (0..n).collect();

    for (ei, &(u, v)) in edges.iter().enumerate() {
        if u == v || comps_of[u].intersection(&comps_of[v]).next().is_some() {
            redundant.push(ei);
            continue;
        }
        while peb[u] + peb[v] < 4 {
            if !find_pebble(u, v, &mut peb, &mut out, &mut rev, &mut free_pebbles)
                && !find_pebble(v, u, &mut peb, &mut out, &mut rev, &mut free_pebbles)
            {
                break;
            }
        }
        if peb[u] + peb[v] < 4 {
            redundant.push(ei);
            continue;
        }
        peb[u] -= 1;
        if peb[u] == 0 {
            free_pebbles.remove(&u);
        }
        out[u].insert(v);
        rev[v].insert(u);
        independent.push(ei);

        // component detection: u, v now hold exactly 3 pebbles.  If some other free pebble is
        // reachable, no new component; else the component is every vertex that cannot reach a
        // free pebble outside {u, v}.
        let reach = reachable(&out, &[u, v]);
        if reach.iter().any(|&w| w != u && w != v && peb[w] > 0) {
            continue;
        }
        let free: Vec<usize> =
            free_pebbles.iter().copied().filter(|&w| w != u && w != v).collect();
        let can_reach_free = reachable(&rev, &free);
        let comp: Vec<usize> = (0..n).filter(|w| !can_reach_free.contains(w)).collect();
        let cset: BTreeSet<usize> = comp.iter().copied().collect();
        let slot = components.len();
        // subsume the components contained in the new one, touching only their own vertices
        for (si, c) in components.iter().enumerate() {
            if alive[si] && c.iter().all(|w| cset.contains(w)) {
                alive[si] = false;
                for &w in c {
                    comps_of[w].remove(&si);
                }
            }
        }
        for &w in &comp {
            comps_of[w].insert(slot);
        }
        components.push(comp);
        alive.push(true);
    }
    let mut kept: Vec<Vec<usize>> = components
        .into_iter()
        .zip(alive)
        .filter(|(_, a)| *a)
        .map(|(c, _)| c)
        .collect();
    kept.sort_by(|a, b| {
        (a.first().copied().unwrap_or(0), a.len()).cmp(&(b.first().copied().unwrap_or(0), b.len()))
    });
    let dof = if n >= 2 { (2 * n).saturating_sub(3 + independent.len()) } else { 0 };
    PebbleResult { independent, redundant, components: kept, dof }
}

/// DFS from `src` for a vertex (other than `src`, `exclude`) holding a pebble; on success move it
/// to `src` by reversing the path.
fn find_pebble(
    src: usize,
    exclude: usize,
    peb: &mut [i32],
    out: &mut [BTreeSet<usize>],
    rev: &mut [BTreeSet<usize>],
    free_pebbles: &mut BTreeSet<usize>,
) -> bool {
    let mut stack = vec![src];
    let mut seen: BTreeSet<usize> = BTreeSet::new();
    seen.insert(src);
    seen.insert(exclude);
    let mut parent: Vec<i32> = vec![-1; peb.len()];
    while let Some(u) = stack.pop() {
        let succ: Vec<usize> = out[u].iter().copied().collect();
        for w in succ {
            if seen.contains(&w) {
                continue;
            }
            seen.insert(w);
            parent[w] = u as i32;
            if peb[w] > 0 {
                peb[w] -= 1;
                if peb[w] == 0 {
                    free_pebbles.remove(&w);
                }
                peb[src] += 1;
                free_pebbles.insert(src);
                let mut x = w;
                while x != src {
                    let p = parent[x] as usize;
                    out[p].remove(&x);
                    rev[x].remove(&p);
                    out[x].insert(p);
                    rev[p].insert(x);
                    x = p;
                }
                return true;
            }
            stack.push(w);
        }
    }
    false
}

fn reachable(adj: &[BTreeSet<usize>], srcs: &[usize]) -> BTreeSet<usize> {
    let mut seen: BTreeSet<usize> = srcs.iter().copied().collect();
    let mut stack: Vec<usize> = srcs.to_vec();
    while let Some(u) = stack.pop() {
        for &w in &adj[u] {
            if seen.insert(w) {
                stack.push(w);
            }
        }
    }
    seen
}
