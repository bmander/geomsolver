//! Integrals over a whole spline (#121): its length, and an energy along it.
//!
//! A contact reads one span of a curve; an integral reads all of them, so its row's columns are
//! every control point's coordinates and its kernel is built per control-point count
//! (`kernels::spline_length_kernel`), as a curve family's is per definition.  The quadrature is
//! Gauss–Legendre on equal pieces of each non-empty span (`nodes`): the curve is linear in its
//! control points over a fixed basis, so a node is the basis there and nothing else — its value,
//! gradient and Hessian in the control points follow from the integrand's in `C` and `C'` by the
//! chain rule, exactly.  Not exact in the integral: `|C'|` is no polynomial, so the length is good
//! to the rule's order per span, which on a drawing's splines is far below the solve's tolerance.
//!
//! The nodes are document data — the knots and weights are — so they are worked out once, where a
//! kernel's constants are written (`write_nodes`), and read back on every evaluation
//! (`read_nodes`) rather than run through the basis again.

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::curve::{self, DEGREE, SPAN_N};

/// Eight-point Gauss–Legendre nodes and weights on [−1, 1].
pub const GAUSS8: [(f64, f64); 8] = [
    (-0.9602898564975363, 0.1012285362903763),
    (-0.7966664774136267, 0.2223810344533745),
    (-0.5255324099163290, 0.3137066458778873),
    (-0.1834346424956498, 0.3626837833783620),
    (0.1834346424956498, 0.3626837833783620),
    (0.5255324099163290, 0.3137066458778873),
    (0.7966664774136267, 0.2223810344533745),
    (0.9602898564975363, 0.1012285362903763),
];

/// Equal pieces each span's rule runs on.  One is not enough where a span bends hard: `|C'|`
/// is no polynomial, and a single Bézier with a tight turn reads ~1e-4 of its length off by
/// eight points, against 2e-10 on four pieces.
pub const PIECES: usize = 4;

/// The nodes one span is integrated over.
pub const PER_SPAN: usize = PIECES * GAUSS8.len();

/// One quadrature node of a spline: the span's first control point, the basis there and its
/// derivative in t (rational where the spline is weighted), the node's weight in t, and the
/// span it is on (counting non-empty spans from 0).
#[derive(Clone, Copy, Debug)]
pub struct Node {
    pub first: usize,
    pub b: [f64; SPAN_N],
    pub d: [f64; SPAN_N],
    pub w: f64,
    pub span: usize,
}

/// Every node of a spline of `n` control points over `knots` (and `weights`, all 1 when
/// `None`), span by span in parameter order.
pub fn nodes(knots: &[f64], weights: Option<&[f64]>, n: usize) -> Vec<Node> {
    let mut out = Vec::with_capacity(PER_SPAN * n.saturating_sub(DEGREE));
    let mut k = 0;
    for s in DEGREE..n {
        let (t0, t1) = (knots[s], knots[s + 1]);
        if t1 <= t0 {
            continue;
        }
        let lk = curve::local_knots(knots, s);
        let lw = curve::local_weights(weights, s);
        let h = (t1 - t0) / (2 * PIECES) as f64;
        for piece in 0..PIECES {
            let mid = t0 + h * (2 * piece + 1) as f64;
            for &(x, w) in &GAUSS8 {
                let (mut b, mut d, mut dd, mut d3) =
                    ([0.0; SPAN_N], [0.0; SPAN_N], [0.0; SPAN_N], [0.0; SPAN_N]);
                curve::basis(mid + x * h, &lk, &mut b, &mut d, &mut dd, &mut d3);
                curve::weigh(&lw, &mut b, &mut d, &mut dd, &mut d3);
                out.push(Node { first: s - DEGREE, b, d, w: w * h, span: k });
            }
        }
        k += 1;
    }
    out
}

/* -- nodes as constants ------------------------------------------------------------------- */

/// How many numbers one node is in a kernel's constants: `[first, span, b…, d…, w]`.
pub const NODE_W: usize = 3 + 2 * SPAN_N;

/// The nodes into `out`, then as many empty ones (weight 0, read back as none) as it takes to
/// make `room` — a kernel's constants are one width for every spline of its count.
pub fn write_nodes(nodes: &[Node], room: usize, out: &mut Vec<f64>) {
    for q in nodes {
        out.extend([q.first as f64, q.span as f64]);
        out.extend(q.b);
        out.extend(q.d);
        out.push(q.w);
    }
    out.extend(std::iter::repeat_n(0.0, NODE_W * room.saturating_sub(nodes.len())));
}

/// The nodes `write_nodes` wrote, the empty ones left out.
pub fn read_nodes(k: &[f64]) -> Vec<Node> {
    k.chunks_exact(NODE_W)
        .filter(|c| c[NODE_W - 1] != 0.0)
        .map(|c| Node {
            first: c[0] as usize,
            span: c[1] as usize,
            b: c[2..2 + SPAN_N].try_into().expect("a node's basis"),
            d: c[2 + SPAN_N..2 + 2 * SPAN_N].try_into().expect("a node's derivative"),
            w: c[NODE_W - 1],
        })
        .collect()
}

/// The most nodes a spline of `n` control points has: every span non-empty.
pub fn most_nodes(n: usize) -> usize {
    PER_SPAN * n.saturating_sub(DEGREE)
}

/* -- length ------------------------------------------------------------------------------- */

/// Below this speed the tangent is no direction, and a node reads its speed as this instead —
/// a curve collapsed to a point has no length to differentiate.
const MIN_SPEED: f64 = 1e-12;

impl Node {
    /// `C` and `C'` here, over control points `p` (x, y interleaved, the whole curve's).
    pub fn frame(&self, p: &[f64]) -> ([f64; 2], [f64; 2]) {
        let (mut c, mut d) = ([0.0; 2], [0.0; 2]);
        for a in 0..SPAN_N {
            let (x, y) = (p[2 * (self.first + a)], p[2 * (self.first + a) + 1]);
            c[0] += self.b[a] * x;
            c[1] += self.b[a] * y;
            d[0] += self.d[a] * x;
            d[1] += self.d[a] * y;
        }
        (c, d)
    }
}

fn speed(d: [f64; 2]) -> f64 {
    d[0].dhypot(d[1]).max(MIN_SPEED)
}

/// The nodes of span `only`, or all of them.
fn of_span(nodes: &[Node], only: Option<usize>) -> impl Iterator<Item = &Node> {
    nodes.iter().filter(move |q| only.is_none_or(|k| q.span == k))
}

/// A spline's length over control points `p` (2n numbers) — span `only`'s, where it is given.
pub fn length(nodes: &[Node], p: &[f64], only: Option<usize>) -> f64 {
    of_span(nodes, only).map(|q| q.w * speed(q.frame(p).1)).sum()
}

/// The length's gradient in the control points, scaled by `s`, added into `g` (2n).
pub fn length_grad(nodes: &[Node], p: &[f64], s: f64, only: Option<usize>, g: &mut [f64]) {
    for q in of_span(nodes, only) {
        let d = q.frame(p).1;
        let v = speed(d);
        let (ux, uy) = (d[0] / v, d[1] / v);
        for a in 0..SPAN_N {
            let c = 2 * (q.first + a);
            g[c] += s * q.w * q.d[a] * ux;
            g[c + 1] += s * q.w * q.d[a] * uy;
        }
    }
}

/// The length's Hessian in the control points, scaled by `s`, added into `h` (2n × 2n, row
/// major, `stride` columns a row): per node `w · d_a d_b (I − ûûᵀ) / |C'|`.
pub fn length_hess(nodes: &[Node], p: &[f64], s: f64, only: Option<usize>, h: &mut [f64],
                   stride: usize) {
    for q in of_span(nodes, only) {
        let d = q.frame(p).1;
        let v = speed(d);
        let (ux, uy) = (d[0] / v, d[1] / v);
        let k = [[(1.0 - ux * ux) / v, -ux * uy / v], [-ux * uy / v, (1.0 - uy * uy) / v]];
        for a in 0..SPAN_N {
            for b in 0..SPAN_N {
                let f = s * q.w * q.d[a] * q.d[b];
                for i in 0..2 {
                    for j in 0..2 {
                        h[(2 * (q.first + a) + i) * stride + 2 * (q.first + b) + j] += f * k[i][j];
                    }
                }
            }
        }
    }
}

/* -- the length row's constants ----------------------------------------------------------- */

/// A spline length row's constants for `n` control points: what it states — `L`, or `(m, c)`
/// for a length an unknown sets — then the spline's nodes, as many as its count allows.
pub fn length_consts(knots: &[f64], weights: Option<&[f64]>, n: usize, stated: &[f64]) -> Vec<f64> {
    let mut k = Vec::with_capacity(stated.len() + NODE_W * most_nodes(n));
    k.extend_from_slice(stated);
    write_nodes(&nodes(knots, weights, n), most_nodes(n), &mut k);
    k
}

/// The control-point count and whether the length is free, read off one instance's columns:
/// `2n`, or `2n + 1` with the unknown's.
pub fn length_widths(n_par: usize) -> (usize, bool) {
    (n_par / 2, n_par % 2 == 1)
}

/// What one instance's constants state (`L`, or `(m, c)`), and its nodes.
pub fn length_parts(free: bool, k: &[f64]) -> (&[f64], Vec<Node>) {
    let (stated, nodes) = k.split_at(if free { 2 } else { 1 });
    (stated, read_nodes(nodes))
}
