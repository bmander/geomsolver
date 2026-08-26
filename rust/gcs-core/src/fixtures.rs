//! Random frameworks the property tests are run over.
//!
//! These are **not** examples.  A case in the library is a drawing somebody could have drawn, and
//! is written as a Solvent document in `rust/examples/`; what is here is a *generator* — it makes
//! a random graph and then measures the positions it happened to place, so there is no statement
//! behind any of its numbers and no document that could express it.  It lives in the core because
//! the core owns algorithms and because all three language test suites run the same generator
//! over the same seeds; it lives in its own module so that nothing mistakes it for a case.

use crate::constraints::{CKind, Constraint};
use crate::model::{EntRef, Sketch};
use crate::rng::Rng;

fn add(sk: &mut Sketch, c: Constraint) {
    sk.add(c);
}

fn dist(sk: &Sketch, a: usize, b: usize) -> f64 {
    let (ax, ay) = sk.point_xy(a);
    let (bx, by) = sk.point_xy(b);
    (ax - bx).hypot(ay - by)
}

/// Random Laman graph on n >= 2 vertices by Henneberg I (add a vertex and 2 edges) and II
/// (subdivide an edge and connect to a third vertex) moves — minimally rigid by construction.
pub fn henneberg_edges(n: usize, rng: &mut Rng) -> Vec<(usize, usize)> {
    let mut edges = vec![(0usize, 1usize)];
    for v in 2..n {
        if v == 2 || rng.next() < 0.6 {
            // type I
            let s = rng.sample(v, 2);
            edges.push((v, s[0]));
            edges.push((v, s[1]));
        } else {
            // type II
            let i = rng.int(edges.len());
            let (a, b) = edges.remove(i);
            let cands: Vec<usize> = (0..v).filter(|&w| w != a && w != b).collect();
            let c = rng.choice(&cands);
            edges.push((v, a));
            edges.push((v, b));
            edges.push((v, c));
        }
    }
    edges
}

/// Random minimally rigid framework with a horizontal member and a fixed node.
pub fn laman(n: usize, seed: u32, ground: bool) -> Sketch {
    let mut rng = Rng::new(seed);
    let mut sk = Sketch::new();
    let pts: Vec<usize> = (0..n)
        .map(|i| {
            let x = rng.uniform(0.0, 60.0);
            let y = rng.uniform(0.0, 60.0);
            sk.point(x, y, false, &format!("n{i}"))
        })
        .collect();
    for (a, b) in henneberg_edges(n, &mut rng) {
        let d = dist(&sk, pts[a], pts[b]);
        add(&mut sk, Constraint::distance(EntRef::point(pts[a]), EntRef::point(pts[b]), d));
    }
    if ground {
        sk.fix_point(pts[0], true);
        let l = sk.line(pts[0], pts[1]);
        add(&mut sk, Constraint::one_line(CKind::Horizontal, EntRef::line(l)));
    }
    sk
}
