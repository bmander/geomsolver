//! Disjoint sets with path halving: which of `n` elements have been joined.

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
