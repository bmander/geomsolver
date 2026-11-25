//! Flat bounding-volume hierarchies: the renderer's ray and projected-edge queries, a profile
//! field's walls (`solid::field::profile`) and a clearance's nearest pieces (`clear`). Subtree end
//! indices let traversal skip whole regions without recursion, a heap stack, or per-query
//! allocation. A nearest search is `query_nearest`: best first, pruning by the best found so far.

#[derive(Clone, Copy, Debug)]
pub struct Bounds<const D: usize> {
    pub lo: [f64; D],
    pub hi: [f64; D],
}
impl<const D: usize> Bounds<D> {
    fn union(self, other: Self) -> Self {
        Self {
            lo: std::array::from_fn(|k| self.lo[k].min(other.lo[k])),
            hi: std::array::from_fn(|k| self.hi[k].max(other.hi[k])),
        }
    }
    pub fn overlaps(self, other: Self) -> bool {
        (0..D).all(|k| self.lo[k] <= other.hi[k] && other.lo[k] <= self.hi[k])
    }
    /// How far apart two boxes are (zero where they meet).
    pub fn gap(self, other: Self) -> f64 {
        (0..D).map(|k| (self.lo[k] - other.hi[k]).max(other.lo[k] - self.hi[k]).max(0.0).powi(2))
            .sum::<f64>().sqrt()
    }
}
#[derive(Clone, Debug)]
struct Node<const D: usize> {
    bounds: Bounds<D>,
    end: usize,
    first: usize,
    count: usize,
}
/// How many levels of a tree over `n` items are grown on threads of their own (`grow`): none for a
/// small tree or without threads, else one a doubling of the cores, up to three.
fn spread(n: usize) -> usize {
    #[cfg(target_arch = "wasm32")]
    { let _ = n; 0 }
    #[cfg(not(target_arch = "wasm32"))]
    {
        if n < SPREAD_ITEMS { return 0 }
        let cores = std::thread::available_parallelism().map_or(1, |c| c.get());
        (usize::BITS - 1 - cores.max(1).leading_zeros()).min(3) as usize
    }
}

/// The fewest items a tree has for its first levels to be grown on threads of their own.
const SPREAD_ITEMS: usize = 1 << 15;

/// The subtree over `items` (the tree's items from `first` on) appended to `nodes` in preorder,
/// each node's `end` the index past its subtree in `nodes`: the box of them all, and, past eight
/// items, its halves by their middles along its longest side. Where `spread`, the halves grown on
/// two threads into lists of their own and appended in order, their ends carried along — the very
/// tree a single thread grows.
fn grow<const D: usize>(items: &mut [(usize, Bounds<D>)], first: usize, nodes: &mut Vec<Node<D>>,
    spread: usize) {
    let count = items.len();
    let bounds = items.iter().map(|(_, b)| *b).reduce(Bounds::union).unwrap();
    let slot = nodes.len();
    nodes.push(Node { bounds, end: slot + 1, first, count });
    if count > 8 {
        let axis = (0..D).max_by(|&a, &b| {
            (bounds.hi[a] - bounds.lo[a]).total_cmp(&(bounds.hi[b] - bounds.lo[b]))
        }).unwrap();
        let mid = count / 2;
        items.select_nth_unstable_by(mid, |(i, a), (j, b)| {
            (a.lo[axis] * 0.5 + a.hi[axis] * 0.5)
                .total_cmp(&(b.lo[axis] * 0.5 + b.hi[axis] * 0.5)).then(i.cmp(j))
        });
        nodes[slot].count = 0;
        let (left, right) = items.split_at_mut(mid);
        #[cfg(not(target_arch = "wasm32"))]
        if spread > 0 {
            let (mut a, mut b) = (Vec::new(), Vec::new());
            std::thread::scope(|scope| {
                let other = scope.spawn(|| grow(right, first + mid, &mut b, spread - 1));
                grow(left, first, &mut a, spread - 1);
                other.join().unwrap_or_else(|e| std::panic::resume_unwind(e));
            });
            for part in [a, b] {
                let base = nodes.len();
                nodes.extend(part.into_iter().map(|n| Node { end: n.end + base, ..n }));
            }
            nodes[slot].end = nodes.len();
            return;
        }
        let _ = spread;
        grow(left, first, nodes, 0);
        grow(right, first + mid, nodes, 0);
        nodes[slot].end = nodes.len();
    }
}
#[derive(Clone, Debug)]
pub struct Bvh<const D: usize> {
    nodes: Vec<Node<D>>,
    items: Vec<(usize, Bounds<D>)>,
}
impl<const D: usize> Bvh<D> {
    pub fn new(bounds: impl Iterator<Item = Bounds<D>>) -> Self {
        let mut items: Vec<_> = bounds.enumerate().collect();
        let mut nodes = Vec::with_capacity(items.len());
        let n = items.len();
        if n > 0 { grow(&mut items, 0, &mut nodes, spread(n)); }
        Self { nodes, items }
    }
    /// Visit every item whose box `intersects` accepts, skipping each subtree whose box it refuses.
    pub fn query(&self, intersects: impl Fn(Bounds<D>) -> bool, mut visit: impl FnMut(usize)) {
        let mut i = 0;
        while i < self.nodes.len() {
            let node = &self.nodes[i];
            if !intersects(node.bounds) {
                i = node.end;
                continue;
            }
            for &(id, b) in &self.items[node.first..node.first + node.count] {
                if intersects(b) { visit(id); }
            }
            i += 1;
        }
    }
    /// Visit items nearer subtree first by `lower`, a lower bound on how far anything in a box
    /// is, passing by each subtree and item whose bound exceeds `best()` (the nearest found so
    /// far, which `visit` may shrink): a nearest search that prunes from its first leaf on. The
    /// stack is fixed (a subtree a level, and the tree is balanced), so a query allocates nothing.
    pub fn query_nearest(&self, lower: impl Fn(Bounds<D>) -> f64, best: impl Fn() -> f64,
        mut visit: impl FnMut(usize)) {
        if self.nodes.is_empty() { return; }
        let (mut stack, mut len) = ([0usize; 64], 1);
        while len > 0 {
            len -= 1;
            let i = stack[len];
            let node = &self.nodes[i];
            if lower(node.bounds) > best() { continue; }
            if node.count > 0 {
                for &(id, b) in &self.items[node.first..node.first + node.count] {
                    if lower(b) <= best() { visit(id); }
                }
                continue;
            }
            // an inner node's children: the next node, and the one past the first's subtree
            let (a, b) = (i + 1, self.nodes[i + 1].end);
            let (near, far) = if lower(self.nodes[a].bounds) <= lower(self.nodes[b].bounds) { (a, b) } else { (b, a) };
            stack[len] = far;
            stack[len + 1] = near;
            len += 2;
        }
    }
    pub fn query_box(&self, bounds: Bounds<D>, visit: impl FnMut(usize)) {
        self.query(|b| b.overlaps(bounds), visit);
    }
    pub fn query_ray(&self, p: [f64; D], d: [f64; D], near: f64, far: f64, visit: impl FnMut(usize)) {
        let inv: [f64; D] = std::array::from_fn(|k| 1.0 / d[k]);
        self.query(|b| {
            let (mut lo, mut hi) = (near, far);
            for k in 0..D {
                if d[k] == 0.0 {
                    if p[k] < b.lo[k] || p[k] > b.hi[k] { return false; }
                } else {
                    let a = (b.lo[k] - p[k]) * inv[k];
                    let z = (b.hi[k] - p[k]) * inv[k];
                    lo = lo.max(a.min(z));
                    hi = hi.min(a.max(z));
                    if hi < lo { return false; }
                }
            }
            true
        }, visit);
    }
}
