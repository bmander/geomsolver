//! Flat bounding-volume hierarchies for ray and projected-edge queries. Subtree end indices
//! let traversal skip whole regions without recursion, a heap stack, or per-query allocation.

#[derive(Clone, Copy, Debug)]
pub(super) struct Bounds<const D: usize> {
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
    fn overlaps(self, other: Self) -> bool {
        (0..D).all(|k| self.lo[k] <= other.hi[k] && other.lo[k] <= self.hi[k])
    }
}
struct Node<const D: usize> {
    bounds: Bounds<D>,
    end: usize,
    first: usize,
    count: usize,
}
pub(super) struct Bvh<const D: usize> {
    nodes: Vec<Node<D>>,
    items: Vec<(usize, Bounds<D>)>,
}
impl<const D: usize> Bvh<D> {
    pub fn new(bounds: impl Iterator<Item = Bounds<D>>) -> Self {
        let items: Vec<_> = bounds.enumerate().collect();
        let mut tree = Self { nodes: Vec::with_capacity(items.len()), items };
        if !tree.items.is_empty() { tree.build(0, tree.items.len()); }
        tree
    }
    fn build(&mut self, first: usize, count: usize) {
        let bounds = self.items[first..first + count].iter().map(|(_, b)| *b)
            .reduce(Bounds::union).unwrap();
        let slot = self.nodes.len();
        self.nodes.push(Node { bounds, end: slot + 1, first, count });
        if count > 8 {
            let axis = (0..D).max_by(|&a, &b| {
                (bounds.hi[a] - bounds.lo[a]).total_cmp(&(bounds.hi[b] - bounds.lo[b]))
            }).unwrap();
            let mid = count / 2;
            self.items[first..first + count].select_nth_unstable_by(mid, |(i, a), (j, b)| {
                (a.lo[axis] * 0.5 + a.hi[axis] * 0.5)
                    .total_cmp(&(b.lo[axis] * 0.5 + b.hi[axis] * 0.5)).then(i.cmp(j))
            });
            self.nodes[slot].count = 0;
            self.build(first, mid);
            self.build(first + mid, count - mid);
            self.nodes[slot].end = self.nodes.len();
        }
    }
    fn query(&self, intersects: impl Fn(Bounds<D>) -> bool, mut visit: impl FnMut(usize)) {
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
