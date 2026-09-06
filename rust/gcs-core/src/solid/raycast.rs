//! Conservative ray culling over immutable primitive facets. Classification still uses the
//! original polygons and the same ray directions; the tree only skips impossible crossings.
use super::{Box3, Prim, SNAP};

#[derive(Clone, Debug)]
pub(crate) struct RayIndex {
    bounds: Box3,
    node: Node,
}

#[derive(Clone, Debug)]
enum Node {
    Leaf(Vec<(usize, Box3)>),
    Branch(Box<RayIndex>, Box<RayIndex>),
}

impl RayIndex {
    pub(crate) fn new(prim: &Prim) -> Self {
        let mut faces: Vec<_> = prim.facets.iter().enumerate().map(|(i, f)| {
            let b = f.bbox();
            // The polygon test admits points a tolerance away from an edge. Its longest
            // edge is at most the box diagonal, which is less than twice its widest span.
            let span = (b.hi[0] - b.lo[0]).max(b.hi[1] - b.lo[1]).max(b.hi[2] - b.lo[2]);
            (i, b.grown(2.0 * span.max(1e-12) * SNAP))
        }).collect();
        Self::build(&mut faces)
    }

    fn build(faces: &mut [(usize, Box3)]) -> Self {
        let mut bounds = Box3::empty();
        for (_, b) in faces.iter() {
            bounds.add(b.lo);
            bounds.add(b.hi);
        }
        let node = if faces.len() <= 8 {
            Node::Leaf(faces.to_vec())
        } else {
            let axis = (0..3).max_by(|&a, &b| {
                (bounds.hi[a] - bounds.lo[a]).total_cmp(&(bounds.hi[b] - bounds.lo[b]))
            }).unwrap();
            let mid = faces.len() / 2;
            faces.select_nth_unstable_by(mid, |(i, a), (j, b)| {
                (a.lo[axis] * 0.5 + a.hi[axis] * 0.5)
                    .total_cmp(&(b.lo[axis] * 0.5 + b.hi[axis] * 0.5)).then(i.cmp(j))
            });
            let (a, b) = faces.split_at_mut(mid);
            Node::Branch(Box::new(Self::build(a)), Box::new(Self::build(b)))
        };
        Self { bounds, node }
    }

    /// Visit every possible forward crossing. None propagates an ambiguous polygon hit so
    /// the classifier can retry with its next deterministic ray direction.
    pub(crate) fn visit(&self, p: [f64; 3], d: [f64; 3],
                       f: &mut impl FnMut(usize) -> Option<()>) -> Option<()> {
        if !intersects(self.bounds, p, d) { return Some(()) }
        match &self.node {
            Node::Leaf(faces) => {
                for (i, b) in faces {
                    if intersects(*b, p, d) { f(*i)?; }
                }
            }
            Node::Branch(a, b) => { a.visit(p, d, f)?; b.visit(p, d, f)?; }
        }
        Some(())
    }
}

fn intersects(b: Box3, p: [f64; 3], d: [f64; 3]) -> bool {
    if b.is_empty() { return false }
    let (mut near, mut far) = (0.0f64, f64::INFINITY);
    for k in 0..3 {
        if d[k] == 0.0 {
            if p[k] < b.lo[k] || p[k] > b.hi[k] { return false }
        } else {
            let a = (b.lo[k] - p[k]) / d[k];
            let z = (b.hi[k] - p[k]) / d[k];
            near = near.max(a.min(z));
            far = far.min(a.max(z));
            if far < near { return false }
        }
    }
    true
}
