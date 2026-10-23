//! **A mesh moved with the drawing** (#510 S2): the nodes of a plane structure as functions of
//! the drawing's own numbers, so a row over a region reads them as its columns.
//!
//! The mesh's topology is the compile's: its elements never change within a solve, and its nodes
//! move as a smooth function of the drawing — the epic's first difficulty, met by keeping the
//! topology and moving the places.  A node at a point of the drawing (a side's end, a point the
//! scenario names) is that point (`Node::At`, its two columns); a node along a side is the side's
//! own point at the parameter it was meshed at (`Node::On`: a line's `a + s (b − a)`, an arc's
//! point at the fraction `s` of its turn, a circle's at the bearing `2π s`), so a side that grows
//! or bends carries its nodes with it.  The nodes inside follow the boundary as a harmonic map:
//! their moves solve `L_II δ_I = −L_IB δ_B`, `L` the cotangent Laplacian of the compile's mesh
//! (its elements each cut into their four straight triangles).  A cotangent Laplacian of a plane
//! mesh leaves a linear function harmonic, so with no move the nodes stand where they were meshed,
//! to rounding; and it is the stiffness of the linear Poisson element, positive definite once the
//! boundary is held, so it factors (`cholesky::Cholesky` under minimum degree, as the structure
//! over the same mesh does) and its rate is one more solve on that factor:
//! a gradient in the inner nodes reaches the boundary as `−L_IBᵀ L_II⁻¹ g_I`.
//!
//! A solid read in three dimensions over a prism (#510 S3) is its face's mesh stacked through its
//! depth (`Stack`): each node of the solid is a node of the plane at an ordinate along the
//! plane's normal, so it moves as that node does, lifted into space, and its rate reaches the
//! plane node along the plane's two axes.

use crate::kernels::{Dual, Num};
use crate::cholesky::{assembled, Cholesky, Ordering};

/// A side of the region, as the row's columns read it: each point by the column of its x (its y
/// the next), a radius by its column.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Edge {
    Line { a: usize, b: usize },
    Arc { c: usize, from: usize, to: usize, r: usize, ccw: bool },
    Circle { c: usize, r: usize },
}

/// Where a node stands: at a point of the drawing (the column of its x), along a side at a
/// parameter, or inside, following the boundary.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Node {
    At(usize),
    On(usize, f64),
    Inner,
}

/// **The nodes as functions of the drawing's columns**: how many columns, the sides, each node's
/// standing, every node's place at the compile (`x0`, two each), and the cotangent Laplacian's
/// weights over the compile's mesh, each pair of nodes once — and where a solid is stacked over
/// the plane's nodes (#510 S3), how.
#[derive(Clone, Debug, PartialEq)]
pub struct Moved {
    pub columns: usize,
    pub edges: Vec<Edge>,
    pub nodes: Vec<Node>,
    pub x0: Vec<f64>,
    pub laplace: Vec<(usize, usize, f64)>,
    pub stack: Option<Stack>,
}

/// **A solid's nodes stacked over a plane's** (#510 S3): the plane as its origin, its two axes
/// and its normal, and per node of the solid the plane's node it stands over and its ordinate
/// along the normal — so it stands at `o + p₀u + p₁v + h n`.
#[derive(Clone, Debug, PartialEq)]
pub struct Stack {
    pub basis: [[f64; 3]; 4],
    pub of: Vec<(usize, f64)>,
}

impl Stack {
    /// Every node's place in space, the plane's nodes at `x` (two each).
    fn lift(&self, x: &[f64]) -> Vec<f64> {
        let [o, u, v, n] = self.basis;
        self.of.iter().flat_map(|&(p, h)| {
            let (a, b) = (x[2 * p], x[2 * p + 1]);
            [0, 1, 2].map(|c| o[c] + a * u[c] + b * v[c] + h * n[c])
        }).collect()
    }

    /// A gradient in the solid's places, folded onto the plane's `plane` nodes along its two
    /// axes.
    fn fold(&self, g: &[f64], plane: usize) -> Vec<f64> {
        let [_, u, v, _] = self.basis;
        let mut out = vec![0.0; 2 * plane];
        for (i, &(p, _)) in self.of.iter().enumerate() {
            let gi = &g[3 * i..3 * i + 3];
            out[2 * p] += (0..3).map(|c| gi[c] * u[c]).sum::<f64>();
            out[2 * p + 1] += (0..3).map(|c| gi[c] * v[c]).sum::<f64>();
        }
        out
    }

    /// Its constants: how many nodes, the basis's twelve numbers, then each node's two.
    fn consts(&self) -> Vec<f64> {
        let mut k = vec![self.of.len() as f64];
        k.extend(self.basis.iter().flatten());
        for &(p, h) in &self.of {
            k.extend([p as f64, h]);
        }
        k
    }

    fn width(&self) -> usize {
        13 + 2 * self.of.len()
    }

    /// What `consts` wrote over a plane of `plane` nodes, and how many it read.
    fn read(k: &[f64], plane: usize) -> Option<(Stack, usize)> {
        let count = |v: f64| (v >= 0.0 && v.fract() == 0.0 && v < 1e9).then_some(v as usize);
        let n = count(*k.first()?)?;
        let b = k.get(1..13)?;
        let basis = std::array::from_fn(|r| [b[3 * r], b[3 * r + 1], b[3 * r + 2]]);
        let of = k.get(13..13 + 2 * n)?.as_chunks::<2>().0.iter()
            .map(|c| Some((count(c[0]).filter(|&p| p < plane)?, c[1]))).collect::<Option<_>>()?;
        Some((Stack { basis, of }, 13 + 2 * n))
    }

    /// The places `x` of the plane's nodes, lifted where there is a stack.
    fn over(stack: &Option<Stack>, x: Vec<f64>) -> Vec<f64> {
        match stack {
            Some(st) => st.lift(&x),
            None => x,
        }
    }
}

/// The places worked out at some columns: every node's (in space where the nodes are stacked), and
/// the factor of the inner nodes' Laplacian, for the rate.
pub struct Placed {
    pub x: Vec<f64>,
    inner: Vec<Option<usize>>,
    factor: Option<Cholesky>,
}

impl Edge {
    /// The columns it reads, in the order `at` reads them.
    fn columns(&self) -> Vec<usize> {
        match *self {
            Edge::Line { a, b } => vec![a, a + 1, b, b + 1],
            Edge::Arc { c, from, to, r, .. } => vec![c, c + 1, from, from + 1, to, to + 1, r],
            Edge::Circle { c, r } => vec![c, c + 1, r],
        }
    }

    /// Its point at `s`, over its columns' numbers `q` (in `columns`'s order): the one reading of a
    /// side, whose meshing (`prisms::Bound::at`) and moving read alike.
    pub(crate) fn at<T: Num>(&self, q: &[T], s: f64) -> [T; 2] {
        let k = T::cst;
        match *self {
            Edge::Line { .. } => [q[0] + k(s) * (q[2] - q[0]), q[1] + k(s) * (q[3] - q[1])],
            Edge::Arc { ccw, .. } => {
                let a0 = T::atan2(q[3] - q[1], q[2] - q[0]);
                let a1 = T::atan2(q[5] - q[1], q[4] - q[0]);
                let mut sweep = if ccw { a1 - a0 } else { a0 - a1 };
                while sweep.val() <= 0.0 {
                    sweep = sweep + k(std::f64::consts::TAU);
                }
                let a = if ccw { a0 + k(s) * sweep } else { a0 - k(s) * sweep };
                let (sin, cos) = a.sin_cos();
                [q[0] + q[6] * cos, q[1] + q[6] * sin]
            }
            Edge::Circle { .. } => {
                let (sin, cos) = (std::f64::consts::TAU * s).sin_cos();
                [q[0] + q[2] * k(cos), q[1] + q[2] * k(sin)]
            }
        }
    }
}

impl Moved {
    /// The nodes inside, numbered among themselves.
    fn inner(&self) -> (Vec<Option<usize>>, usize) {
        let mut n = 0;
        let at = self.nodes.iter().map(|k| matches!(k, Node::Inner).then(|| { n += 1; n - 1 }))
            .collect();
        (at, n)
    }

    /// **Every node's place at columns `v`**: the boundary's from the drawing, the inner nodes'
    /// by the harmonic map of the boundary's move — or `None` where the Laplacian does not factor.
    pub fn places(&self, v: &[f64]) -> Option<Placed> {
        let n = self.nodes.len();
        let mut x = self.x0.clone();
        for (i, node) in self.nodes.iter().enumerate() {
            match *node {
                Node::At(c) => x[2 * i..2 * i + 2].copy_from_slice(&v[c..c + 2]),
                Node::On(e, s) => {
                    let q: Vec<Dual<0>> = self.edges[e].columns().iter()
                        .map(|&c| <Dual<0> as Num>::cst(v[c])).collect();
                    let p = self.edges[e].at(&q, s);
                    x[2 * i] = p[0].v;
                    x[2 * i + 1] = p[1].v;
                }
                Node::Inner => {}
            }
        }
        let (inner, m) = self.inner();
        if m == 0 {
            return Some(Placed { x: Stack::over(&self.stack, x), inner, factor: None });
        }
        let mut t: Vec<(usize, usize, f64)> = Vec::new();
        // the boundary's move, into each inner node's right-hand side: −L_IB δ_B = Σ w δ_B
        let mut rhs = [vec![0.0; m], vec![0.0; m]];
        for &(i, j, w) in &self.laplace {
            for (a, b) in [(i, j), (j, i)] {
                let Some(ia) = inner[a] else { continue };
                t.push((ia, ia, w));
                match inner[b] {
                    Some(ib) => t.push((ia, ib, -w)),
                    None => {
                        for c in 0..2 {
                            rhs[c][ia] += w * (x[2 * b + c] - self.x0[2 * b + c]);
                        }
                    }
                }
            }
        }
        let (l, mut factor) = assembled(m, &t, Ordering::Amd);
        factor.factor(&l).ok()?;
        for (c, r) in rhs.iter_mut().enumerate() {
            factor.solve(r);
            for i in 0..n {
                if let Some(k) = inner[i] {
                    x[2 * i + c] = self.x0[2 * i + c] + r[k];
                }
            }
        }
        Some(Placed { x: Stack::over(&self.stack, x), inner, factor: Some(factor) })
    }

    /// **A gradient in the nodes' places carried to the columns**, added into `row`: the inner
    /// nodes' through the harmonic map onto the boundary's (`−L_IBᵀ L_II⁻¹ g_I`, one solve per
    /// axis on the factor in hand), then each boundary node's onto the columns its place reads.
    pub fn chain(&self, v: &[f64], placed: &Placed, g: &[f64], row: &mut [f64]) {
        let mut g = match &self.stack {
            Some(st) => st.fold(&g[..3 * st.of.len()], self.nodes.len()),
            None => g[..2 * self.nodes.len()].to_vec(),
        };
        if let Some(f) = &placed.factor {
            let m = placed.inner.iter().flatten().count();
            for c in 0..2 {
                let mut gi = vec![0.0; m];
                for (i, k) in placed.inner.iter().enumerate() {
                    if let Some(k) = *k {
                        gi[k] = g[2 * i + c];
                    }
                }
                f.solve(&mut gi);
                // δ_I = L_II⁻¹ Σ_B w δ_B, so a boundary node b gains Σ_{i inner} w_ib λ_i
                for &(i, j, w) in &self.laplace {
                    for (a, b) in [(i, j), (j, i)] {
                        if let (Some(ia), None) = (placed.inner[a], placed.inner[b]) {
                            g[2 * b + c] += w * gi[ia];
                        }
                    }
                }
            }
        }
        for (i, node) in self.nodes.iter().enumerate() {
            match *node {
                Node::At(c) => {
                    row[c] += g[2 * i];
                    row[c + 1] += g[2 * i + 1];
                }
                Node::On(e, s) => {
                    let cols = self.edges[e].columns();
                    let q: Vec<Dual<7>> =
                        cols.iter().enumerate().map(|(k, &c)| Dual::var(v[c], k)).collect();
                    let p = self.edges[e].at(&q, s);
                    for (k, &c) in cols.iter().enumerate() {
                        row[c] += g[2 * i] * p[0].g[k] + g[2 * i + 1] * p[1].g[k];
                    }
                }
                Node::Inner => {}
            }
        }
    }

    /// **The cotangent Laplacian of a mesh of quadratic triangles** (`tris`, their nodes' places
    /// `x`): each element cut into its four straight triangles, each of their sides weighed by
    /// half the cotangent of the angle across it, summed over the triangles a side is on.
    pub fn laplacian(tris: &[[usize; 6]], x: &[f64]) -> Vec<(usize, usize, f64)> {
        let mut w: std::collections::BTreeMap<(usize, usize), f64> = Default::default();
        let p = |i: usize| [x[2 * i], x[2 * i + 1]];
        for t in tris {
            for [a, b, c] in [[t[0], t[3], t[5]], [t[3], t[1], t[4]], [t[5], t[4], t[2]],
                              [t[3], t[4], t[5]]] {
                for (i, j, k) in [(a, b, c), (b, c, a), (c, a, b)] {
                    // the side i–j, its angle at k
                    let (u, v) = (sub(p(i), p(k)), sub(p(j), p(k)));
                    let cot = (u[0] * v[0] + u[1] * v[1]) / (u[0] * v[1] - u[1] * v[0]).abs();
                    *w.entry((i.min(j), i.max(j))).or_default() += 0.5 * cot;
                }
            }
        }
        w.into_iter().map(|((i, j), v)| (i, j, v)).collect()
    }

    /// Its constants: `[columns, edges, nodes, weights]`, each side's six numbers, each node's
    /// three, the places, then the weights' three each.
    pub fn consts(&self) -> Vec<f64> {
        let mut k = vec![self.columns as f64, self.edges.len() as f64, self.nodes.len() as f64,
                         self.laplace.len() as f64];
        for e in &self.edges {
            k.extend(match *e {
                Edge::Line { a, b } => [0.0, a as f64, b as f64, 0.0, 0.0, 0.0],
                Edge::Arc { c, from, to, r, ccw } => {
                    [1.0, c as f64, from as f64, to as f64, r as f64, ccw as u8 as f64]
                }
                Edge::Circle { c, r } => [2.0, c as f64, r as f64, 0.0, 0.0, 0.0],
            });
        }
        for n in &self.nodes {
            k.extend(match *n {
                Node::At(c) => [0.0, c as f64, 0.0],
                Node::On(e, s) => [1.0, e as f64, s],
                Node::Inner => [2.0, 0.0, 0.0],
            });
        }
        k.extend(&self.x0);
        for &(i, j, w) in &self.laplace {
            k.extend([i as f64, j as f64, w]);
        }
        if let Some(st) = &self.stack {
            k.extend(st.consts());
        }
        k
    }

    /// How many constants it is.
    pub fn width(&self) -> usize {
        4 + 6 * self.edges.len() + 5 * self.nodes.len() + 3 * self.laplace.len()
            + self.stack.as_ref().map_or(0, Stack::width)
    }

    /// What `consts` wrote, from `k`, and how many it read — `None` for constants that are not
    /// whole, or a column or a node out of range.  `stacked` where a solid is stacked over it.
    pub fn read(k: &[f64], stacked: bool) -> Option<(Moved, usize)> {
        let count = |v: f64| (v >= 0.0 && v.fract() == 0.0 && v < 1e9).then_some(v as usize);
        let head = k.get(..4)?;
        let (columns, n_edges, n_nodes, n_lap) =
            (count(head[0])?, count(head[1])?, count(head[2])?, count(head[3])?);
        let mut at: usize = 4;
        let mut take = |n: usize| -> Option<&[f64]> {
            let out = k.get(at..at.checked_add(n)?)?;
            at += n;
            Some(out)
        };
        let point = |v: f64| count(v).filter(|&c| c + 1 < columns);
        let col = |v: f64| count(v).filter(|&c| c < columns);
        let edges: Vec<Edge> = take(6 * n_edges)?.as_chunks::<6>().0.iter().map(|e| {
            Some(match e[0] {
                k if k == 0.0 => Edge::Line { a: point(e[1])?, b: point(e[2])? },
                k if k == 1.0 => Edge::Arc { c: point(e[1])?, from: point(e[2])?, to: point(e[3])?,
                                             r: col(e[4])?, ccw: e[5] != 0.0 },
                k if k == 2.0 => Edge::Circle { c: point(e[1])?, r: col(e[2])? },
                _ => return None,
            })
        }).collect::<Option<_>>()?;
        let nodes: Vec<Node> = take(3 * n_nodes)?.as_chunks::<3>().0.iter().map(|n| {
            Some(match n[0] {
                k if k == 0.0 => Node::At(point(n[1])?),
                k if k == 1.0 => Node::On(count(n[1]).filter(|&e| e < n_edges)?, n[2]),
                k if k == 2.0 => Node::Inner,
                _ => return None,
            })
        }).collect::<Option<_>>()?;
        let x0 = take(2 * n_nodes)?.to_vec();
        let laplace = take(3 * n_lap)?.as_chunks::<3>().0.iter().map(|w| {
            Some((count(w[0]).filter(|&i| i < n_nodes)?, count(w[1]).filter(|&j| j < n_nodes)?,
                  w[2]))
        }).collect::<Option<_>>()?;
        let stack = match stacked {
            false => None,
            true => {
                let (st, used) = Stack::read(k.get(at..)?, n_nodes)?;
                at += used;
                Some(st)
            }
        };
        Some((Moved { columns, edges, nodes, x0, laplace, stack }, at))
    }
}

fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}
