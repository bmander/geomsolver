//! The boundary-value problem a curve's shape is the solution of (#144): integrated from its
//! start, landing on its end, through every peg pressing it.
//!
//! The curve is cut into **arcs** at its pegs, and each arc into `SEGMENTS` equal pieces, each
//! integrated from a state of its own (multiple shooting: a piece's error grows only over the
//! piece, and a seed sets every piece's start).  The unknowns are the costate at the start, each
//! piece's starting state, and at each peg its place along the curve and the costate leaving
//! it; the rows join the pieces, put each peg's arc end on the peg with `H` unbroken across it
//! (the place is free), and put the last arc's end on `b`.  So the costate may jump at a peg —
//! a point force — and the direction with it: a corner.  Square, solved by a damped Newton over
//! its dense Jacobian, which the pieces' sensitivities give exactly.

#[allow(unused_imports)]
use crate::fmath::Det;
use super::flow::{self, Flow, Lagrangian};

/// Pieces per arc.
pub const SEGMENTS: usize = 4;

/// What a curve's shape is asked for: its ends, its length, and the held points it passes.
#[derive(Clone, Debug, PartialEq)]
pub struct Ends {
    pub a: [f64; 2],
    pub b: [f64; 2],
    pub len: f64,
}

impl Ends {
    /// The outer columns, in the curve's order: `a`, `b`, the length.
    pub fn outer(&self) -> [f64; 5] {
        [self.a[0], self.a[1], self.b[0], self.b[1], self.len]
    }

    pub fn of(o: &[f64; 5]) -> Ends {
        Ends { a: [o[0], o[1]], b: [o[2], o[3]], len: o[4] }
    }

    /// A length to measure positions against.
    fn scale(&self) -> f64 {
        self.len.abs().max((self.b[0] - self.a[0]).dhypot(self.b[1] - self.a[1])).max(1e-9)
    }
}

/// One integrated piece: where it starts along the curve, how long it is, and the state at each
/// step's end with its sensitivity to the piece's start (`phi`, 4 × 4, row-major).
#[derive(Clone, Debug)]
pub struct Piece {
    pub s0: f64,
    pub len: f64,
    pub nodes: Vec<Node>,
}

#[derive(Clone, Copy, Debug)]
pub struct Node {
    pub s: f64,
    pub z: [f64; 4],
    pub theta: f64,
    pub phi: [f64; 16],
}

const IDENTITY: [f64; 16] = [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.];

/// The most steps a piece may take — a flow needing more is followed nowhere near its answer.
const STEPS_MAX: usize = 4000;

/// Integrate one piece of length `len` from `z0`, the direction found from `theta`, with the
/// sensitivities to the start where `sens` (`None` where the flow cannot be followed).
pub fn integrate(lag: &Lagrangian, s0: f64, z0: [f64; 4], theta: f64, len: f64, scale_len: f64, sens: bool) -> Option<Piece> {
    let m = if sens { 4 } else { 0 };
    let mut flow = Flow::new(lag, theta);
    let p0 = flow.at(&z0, false)?;
    let lam = z0[2].dhypot(z0[3]).max(p0.h.abs()).max(1e-300);
    let scale = [scale_len, scale_len, lam, lam];
    let mut nodes = vec![Node { s: s0, z: z0, theta: p0.theta, phi: IDENTITY }];
    let mut done = 0.0;
    let mut h = len / 4.0;
    let mut y = [0.0; flow::WIDTH];
    y[..4].copy_from_slice(&z0);
    y[4..].copy_from_slice(&IDENTITY);
    let mut th = p0.theta;
    let mut taken = 0;
    while done < len {
        if taken >= STEPS_MAX {
            return None;
        }
        let last = len - done <= h * (1.0 + 1e-12);
        let hh = if last { len - done } else { h };
        let (y1, err) = flow::step(&mut flow, th, &y, hh, m, &scale)?;
        taken += 1;
        if err <= flow::TOL || hh <= len * 1e-12 {
            done = if last { len } else { done + hh };
            th = flow.theta;
            y = y1;
            let mut phi = IDENTITY;
            if sens {
                phi.copy_from_slice(&y[4..20]);
            }
            nodes.push(Node { s: s0 + done, z: [y[0], y[1], y[2], y[3]], theta: th, phi });
            h = flow::next_h(hh, err);
        } else {
            h = flow::next_h(hh, err).min(hh * 0.5);
        }
    }
    Some(Piece { s0, len, nodes })
}

/// The state at `s` within a piece (its start ≤ `s` ≤ its end): one step from the node before
/// it, with the sensitivity to the piece's start (the piece integrated with them).
pub fn within(lag: &Lagrangian, piece: &Piece, s: f64, scale_len: f64) -> Option<Node> {
    let i = piece.nodes.partition_point(|n| n.s <= s).saturating_sub(1);
    let n = piece.nodes[i];
    let h = s - n.s;
    if h <= 0.0 {
        return Some(n);
    }
    let mut flow = Flow::new(lag, n.theta);
    let lam = n.z[2].dhypot(n.z[3]).max(1e-300);
    let mut y = [0.0; flow::WIDTH];
    y[..4].copy_from_slice(&n.z);
    y[4..].copy_from_slice(&n.phi);
    let (y1, _) = flow::step(&mut flow, n.theta, &y, h, 4, &[scale_len, scale_len, lam, lam])?;
    let mut phi = [0.0; 16];
    phi.copy_from_slice(&y1[4..20]);
    Some(Node { s, z: [y1[0], y1[1], y1[2], y1[3]], theta: flow.theta, phi })
}

/* -- the problem's unknowns --------------------------------------------------------------- */

/// Where the unknowns sit, for `k` pegs: arc 0's starting costate, then per arc after a peg its
/// place and starting costate, and per arc each later piece's starting state.  With `held`
/// places (the verdict's re-solves) a place is no unknown and its `H` row is not stated.
#[derive(Clone, Copy, Debug)]
pub struct Layout {
    pub pegs: usize,
    pub held: bool,
}

/// What a quantity depends on: an unknown, an outer column (`a`, `b`, the length), or nothing.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Var {
    Q(usize),
    O(usize),
}

impl Layout {
    fn arc_start(&self, j: usize) -> usize {
        if j == 0 {
            0
        } else {
            let per = if self.held { 2 } else { 3 };
            2 + 4 * (SEGMENTS - 1) + (j - 1) * (per + 4 * (SEGMENTS - 1))
        }
    }

    /// Arc `j`'s place (j ≥ 1), when it is an unknown.
    fn place(&self, j: usize) -> Option<usize> {
        (j >= 1 && !self.held).then(|| self.arc_start(j))
    }

    /// Arc `j`'s starting costate.
    fn lambda(&self, j: usize) -> usize {
        self.arc_start(j) + if j == 0 || self.held { 0 } else { 1 }
    }

    /// Piece `m ≥ 1` of arc `j`'s starting state.
    fn node(&self, j: usize, m: usize) -> usize {
        self.lambda(j) + 2 + 4 * (m - 1)
    }

    pub fn len(&self) -> usize {
        self.arc_start(self.pegs + 1)
    }
}

/// A solved shape: the problem it answers, its unknowns, where its arcs start, its pieces, and
/// the unknowns' derivative in the outer columns (`dq`, `n × 5`, row-major), the implicit
/// function theorem's, from the Jacobian the solve ended on.
#[derive(Clone, Debug)]
pub struct Shape {
    pub ends: Ends,
    pub pegs: Vec<[f64; 2]>,
    pub q: Vec<f64>,
    /// Each arc's start along the curve, then the length: `0, s₁ … s_k, L`.
    pub places: Vec<f64>,
    pub pieces: Vec<Piece>,
    pub dq: Vec<f64>,
    /// Each piece's direction where it starts, to start the next solve's from.
    pub thetas: Vec<f64>,
}

/// The rows, their Jacobian in the unknowns and in the outer columns, at one set of unknowns.
struct Eval {
    r: Vec<f64>,
    jq: Vec<f64>,
    jo: Vec<f64>,
    pieces: Vec<Piece>,
    places: Vec<f64>,
    thetas: Vec<f64>,
    /// What each row is measured against, for convergence.
    w: Vec<f64>,
}

struct Problem<'a> {
    lag: &'a Lagrangian,
    ends: &'a Ends,
    pegs: &'a [[f64; 2]],
    lay: Layout,
    /// Places held, where `lay.held`.
    held: &'a [f64],
}

impl Problem<'_> {
    fn places(&self, q: &[f64]) -> Vec<f64> {
        let k = self.lay.pegs;
        let mut p = Vec::with_capacity(k + 2);
        p.push(0.0);
        for j in 1..=k {
            p.push(match self.lay.place(j) {
                Some(i) => q[i],
                None => self.held[j - 1],
            });
        }
        p.push(self.ends.len);
        p
    }

    /// Arc `j`'s start point and what it depends on.
    fn arc_point(&self, j: usize) -> ([f64; 2], [Option<Var>; 2]) {
        if j == 0 {
            (self.ends.a, [Some(Var::O(0)), Some(Var::O(1))])
        } else {
            (self.pegs[j - 1], [None, None])
        }
    }

    /// The place `j` and what it is (0 and the length are a constant and an outer column).
    fn place_var(&self, j: usize) -> Option<Var> {
        if j == 0 {
            None
        } else if j == self.lay.pegs + 1 {
            Some(Var::O(4))
        } else {
            self.lay.place(j).map(Var::Q)
        }
    }

    /// Piece `(j, m)`'s starting state and what each component depends on.
    fn start(&self, q: &[f64], j: usize, m: usize) -> ([f64; 4], [Option<Var>; 4]) {
        if m == 0 {
            let (p, dp) = self.arc_point(j);
            let l = self.lay.lambda(j);
            ([p[0], p[1], q[l], q[l + 1]], [dp[0], dp[1], Some(Var::Q(l)), Some(Var::Q(l + 1))])
        } else {
            let n = self.lay.node(j, m);
            (
                [q[n], q[n + 1], q[n + 2], q[n + 3]],
                [Some(Var::Q(n)), Some(Var::Q(n + 1)), Some(Var::Q(n + 2)), Some(Var::Q(n + 3))],
            )
        }
    }

    fn eval(&self, q: &[f64], thetas: &[f64], jac: bool) -> Option<Eval> {
        let n = self.lay.len();
        let k = self.lay.pegs;
        let places = self.places(q);
        let scale_len = self.ends.scale();
        let mut r = Vec::with_capacity(n);
        let mut w = Vec::with_capacity(n);
        let mut jq = vec![0.0; if jac { n * n } else { 0 }];
        let mut jo = vec![0.0; if jac { n * 5 } else { 0 }];
        let mut pieces = Vec::with_capacity((k + 1) * SEGMENTS);
        let mut new_thetas = Vec::with_capacity((k + 1) * SEGMENTS);
        let lam_scale = (0..=k)
            .map(|j| q[self.lay.lambda(j)].dhypot(q[self.lay.lambda(j) + 1]))
            .fold(1e-300, f64::max);
        let add = |jq: &mut Vec<f64>, jo: &mut Vec<f64>, row: usize, v: Var, f: f64| match v {
            Var::Q(i) => jq[row * n + i] += f,
            Var::O(i) => jo[row * 5 + i] += f,
        };
        let mut h_start = vec![0.0; k + 1];
        let mut h_start_d: Vec<([f64; 4], [Option<Var>; 4])> = Vec::with_capacity(k + 1);
        for j in 0..=k {
            let len_arc = places[j + 1] - places[j];
            if !(len_arc > 0.0) {
                return None;
            }
            let h = len_arc / SEGMENTS as f64;
            for m in 0..SEGMENTS {
                let (z0, dz0) = self.start(q, j, m);
                let s0 = places[j] + h * m as f64;
                let th = thetas.get(j * SEGMENTS + m).copied().unwrap_or(0.0);
                let piece = integrate(self.lag, s0, z0, th, h, scale_len, jac)?;
                new_thetas.push(piece.nodes[0].theta);
                if m == 0 {
                    let mut fl = Flow::new(self.lag, piece.nodes[0].theta);
                    let p = fl.at(&z0, false)?;
                    let (sn, cs) = p.theta.dsin_cos();
                    h_start[j] = p.h;
                    h_start_d.push(([p.d.x, p.d.y, cs, sn], dz0));
                }
                let end = *piece.nodes.last().expect("a node");
                // the piece's end, and its derivative in its length: the flow there
                let mut fl = Flow::new(self.lag, end.theta);
                let mut fz = [0.0; 4];
                if !fl.rhs(&end.z, &mut fz, 0) {
                    return None;
                }
                let pe = fl.at(&end.z, false)?;
                // ∂h/∂(places): h = (s_{j+1} − s_j)/SEGMENTS
                let dh = [(self.place_var(j), -1.0 / SEGMENTS as f64), (self.place_var(j + 1), 1.0 / SEGMENTS as f64)];
                let emit = |r: &mut Vec<f64>, w: &mut Vec<f64>, jq: &mut Vec<f64>, jo: &mut Vec<f64>,
                                comps: std::ops::Range<usize>, target: &[f64], tdep: &[Option<Var>], scale: &[f64]| {
                    for (ii, c) in comps.enumerate() {
                        let row = r.len();
                        r.push(end.z[c] - target[ii]);
                        w.push(scale[ii]);
                        if !jac {
                            continue;
                        }
                        for (cc, d) in dz0.iter().enumerate() {
                            if let Some(v) = d {
                                add(jq, jo, row, *v, end.phi[c * 4 + cc]);
                            }
                        }
                        for (pv, f) in dh {
                            if let Some(v) = pv {
                                add(jq, jo, row, v, fz[c] * f);
                            }
                        }
                        if let Some(v) = tdep[ii] {
                            add(jq, jo, row, v, -1.0);
                        }
                    }
                };
                if m + 1 < SEGMENTS {
                    let (z1, dz1) = self.start(q, j, m + 1);
                    emit(&mut r, &mut w, &mut jq, &mut jo, 0..4, &z1, &dz1,
                         &[scale_len, scale_len, lam_scale, lam_scale]);
                } else if j < k {
                    let peg = self.pegs[j];
                    emit(&mut r, &mut w, &mut jq, &mut jo, 0..2, &peg, &[None, None], &[scale_len, scale_len]);
                    // H unbroken across the peg, where its place is free
                    if !self.lay.held {
                        let row = r.len();
                        // the next arc's start H is filled in once that arc is integrated
                        r.push(-pe.h);
                        w.push(lam_scale.max(pe.h.abs()));
                        if jac {
                            let (sn, cs) = pe.theta.dsin_cos();
                            let hz = [pe.d.x, pe.d.y, cs, sn];
                            // −H at the arc's end: through the piece's start and its length
                            for c in 0..4 {
                                for (cc, d) in dz0.iter().enumerate() {
                                    if let Some(v) = d {
                                        add(&mut jq, &mut jo, row, *v, -hz[c] * end.phi[c * 4 + cc]);
                                    }
                                }
                                for (pv, f) in dh {
                                    if let Some(v) = pv {
                                        add(&mut jq, &mut jo, row, v, -hz[c] * fz[c] * f);
                                    }
                                }
                            }
                        }
                    }
                } else {
                    emit(&mut r, &mut w, &mut jq, &mut jo, 0..2, &self.ends.b,
                         &[Some(Var::O(2)), Some(Var::O(3))], &[scale_len, scale_len]);
                }
                pieces.push(piece);
            }
        }
        // the H rows' other side: the arc after each peg, where it starts
        if !self.lay.held {
            let per_arc = 4 * (SEGMENTS - 1) + 3;
            for j in 1..=k {
                let row = (j - 1) * per_arc + 4 * (SEGMENTS - 1) + 2;
                r[row] += h_start[j];
                if jac {
                    let (hz, dz) = h_start_d[j];
                    for c in 0..4 {
                        if let Some(v) = dz[c] {
                            add(&mut jq, &mut jo, row, v, hz[c]);
                        }
                    }
                }
            }
        }
        debug_assert_eq!(r.len(), n);
        Some(Eval { r, jq, jo, pieces, places, thetas: new_thetas, w })
    }
}

/* -- the solve ---------------------------------------------------------------------------- */

/// Iterations from a seed, and from a prediction a walk's step makes (which, near, converges
/// in a few — a step that needs more is a step too long, halved).
const NEWTON_MAX: usize = 40;
const CORRECT_MAX: usize = 8;
/// Converged: every row within this of what it is measured against.
const DONE: f64 = 1e-12;
/// Accepted, where Newton stalls short of `DONE` (the integration's own noise).
const GOOD: f64 = 1e-9;

/// The largest row, each against its measure: what convergence asks.
fn norm(e: &Eval) -> f64 {
    e.r.iter().zip(&e.w).map(|(r, w)| (r / w).abs()).fold(0.0, f64::max)
}

/// The rows' squares, each against its measure: what a step must lower.
fn merit(e: &Eval) -> f64 {
    e.r.iter().zip(&e.w).map(|(r, w)| (r / w) * (r / w)).sum()
}

/// Newton from `q`, damped by halving, for at most `most` iterations.
fn newton(pb: &Problem, mut q: Vec<f64>, mut thetas: Vec<f64>, most: usize) -> Option<(Vec<f64>, Eval)> {
    let n = pb.lay.len();
    let mut e = pb.eval(&q, &thetas, true)?;
    let mut piv = Vec::new();
    for _ in 0..most {
        let f = norm(&e);
        if f <= DONE {
            return Some((q, e));
        }
        let m0 = merit(&e);
        let mut a = e.jq.clone();
        if !crate::linalg::lu_factor(n, &mut a, &mut piv) {
            return None;
        }
        let mut dq: Vec<f64> = e.r.iter().map(|v| -v).collect();
        crate::linalg::lu_apply(n, &a, &piv, &mut dq);
        let mut t = 1.0;
        let mut moved = false;
        while t > 1e-6 {
            let trial: Vec<f64> = q.iter().zip(&dq).map(|(a, b)| a + t * b).collect();
            if let Some(et) = pb.eval(&trial, &e.thetas, false) {
                if merit(&et) < m0 * (1.0 - 0.5 * t) {
                    q = trial;
                    thetas = et.thetas.clone();
                    moved = true;
                    break;
                }
            }
            t *= 0.5;
        }
        if !moved {
            return (f <= GOOD).then_some((q, e));
        }
        e = pb.eval(&q, &thetas, true)?;
    }
    (norm(&e) <= GOOD).then_some((q, e))
}

/// The solved shape from a converged `Eval`: the unknowns' derivative in the outer columns.
fn shape(pb: &Problem, q: Vec<f64>, e: Eval) -> Option<Shape> {
    let n = pb.lay.len();
    let mut a = e.jq.clone();
    let mut piv = Vec::new();
    if !crate::linalg::lu_factor(n, &mut a, &mut piv) {
        return None;
    }
    let mut dq = vec![0.0; n * 5];
    for c in 0..5 {
        let mut col: Vec<f64> = (0..n).map(|i| -e.jo[i * 5 + c]).collect();
        crate::linalg::lu_apply(n, &a, &piv, &mut col);
        for i in 0..n {
            dq[i * 5 + c] = col[i];
        }
    }
    Some(Shape {
        ends: pb.ends.clone(),
        pegs: pb.pegs.to_vec(),
        q,
        places: e.places,
        pieces: e.pieces,
        dq,
        thetas: e.thetas,
    })
}

/// The shape of a curve from `ends`, through `pegs`: warm from `prev` where there is one, else
/// from the seed of a gently sagging curve (`EASY`), its length walked out to this one's
/// (`walk`) — an arc seed for a long curve between close ends is nothing like its answer (a long
/// rope is two strands and a tight bend), and from it Newton may find another stationary curve
/// (one with a loop); walked out, the curve stays on the branch it started on.
pub fn solve(lag: &Lagrangian, ends: &Ends, pegs: &[[f64; 2]], prev: Option<&Shape>) -> Option<Shape> {
    let lay = Layout { pegs: pegs.len(), held: false };
    if let Some(p) = prev.filter(|p| p.pegs.len() == pegs.len() && p.q.len() == lay.len()) {
        let pegs = paired(&p.pegs, pegs);
        if p.ends == *ends && p.pegs == pegs {
            return Some(p.clone());
        }
        if let Some(s) = walk(lag, p, ends, &pegs) {
            return Some(s);
        }
    }
    if !pegs.is_empty() {
        return pegged(lag, ends, pegs);
    }
    let chord = (ends.b[0] - ends.a[0]).dhypot(ends.b[1] - ends.a[1]);
    let short = chord * EASY;
    if ends.len <= short {
        return cold(lag, ends, pegs);
    }
    let easy = cold(lag, &Ends { len: short, ..ends.clone() }, pegs)?;
    walk(lag, &easy, ends, pegs)
}

/// `pegs` in the order `before` had them along the curve, each paired with the nearest of the
/// last solve's (pegs move a little between solves; their order along the curve does not).
fn paired(before: &[[f64; 2]], pegs: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let mut left: Vec<[f64; 2]> = pegs.to_vec();
    before
        .iter()
        .map(|b| {
            let i = (0..left.len())
                .min_by(|&i, &j| {
                    let d = |p: &[f64; 2]| (p[0] - b[0]).dhypot(p[1] - b[1]);
                    d(&left[i]).total_cmp(&d(&left[j]))
                })
                .expect("as many pegs");
            left.remove(i)
        })
        .collect()
}

/// A curve through pegs, from nothing: the curve without them, each peg first put where that
/// curve already passes nearest it — a shape the pegs press with no force, exactly solved — and
/// then walked to where it is.  Their order along the curve is that curve's.
fn pegged(lag: &Lagrangian, ends: &Ends, pegs: &[[f64; 2]]) -> Option<Shape> {
    let free = solve(lag, ends, &[], None)?;
    let mut near: Vec<(f64, [f64; 2])> = pegs.iter().map(|&p| (nearest(lag, &free, p), p)).collect();
    near.sort_by(|a, b| a.0.total_cmp(&b.0));
    if near.windows(2).any(|w| w[1].0 <= w[0].0) || near.iter().any(|n| n.0 <= 0.0 || n.0 >= 1.0) {
        return None;
    }
    let len = ends.len;
    let lay = Layout { pegs: pegs.len(), held: false };
    let mut q = vec![0.0; lay.len()];
    let mut thetas = Vec::with_capacity((pegs.len() + 1) * SEGMENTS);
    let mut virtual_pegs = Vec::with_capacity(pegs.len());
    let mut places = vec![0.0];
    places.extend(near.iter().map(|n| n.0 * len));
    places.push(len);
    for j in 0..=pegs.len() {
        let h = (places[j + 1] - places[j]) / SEGMENTS as f64;
        for m in 0..SEGMENTS {
            let at = at(lag, &free, (places[j] + h * m as f64) / len)?;
            thetas.push(at.theta);
            if m == 0 {
                if j > 0 {
                    q[lay.place(j).expect("a place")] = places[j];
                    virtual_pegs.push([at.z[0], at.z[1]]);
                }
                let l = lay.lambda(j);
                q[l] = at.z[2];
                q[l + 1] = at.z[3];
            } else {
                let n = lay.node(j, m);
                q[n..n + 4].copy_from_slice(&at.z);
            }
        }
    }
    let pb = Problem { lag, ends, pegs: &virtual_pegs, lay, held: &[] };
    let (q, e) = newton(&pb, q, thetas, NEWTON_MAX)?;
    let start = shape(&pb, q, e)?;
    let real: Vec<[f64; 2]> = near.iter().map(|n| n.1).collect();
    walk(lag, &start, ends, &real)
}

/// Where along `sh` (in `u`) it passes nearest `p`: the best of an even sampling, refined by
/// golden section.
fn nearest(lag: &Lagrangian, sh: &Shape, p: [f64; 2]) -> f64 {
    const N: usize = 256;
    let d = |u: f64| match at(lag, sh, u) {
        Some(a) => (a.z[0] - p[0]).dhypot(a.z[1] - p[1]),
        None => f64::INFINITY,
    };
    let best = (0..=N).min_by(|&i, &j| d(i as f64 / N as f64).total_cmp(&d(j as f64 / N as f64))).unwrap_or(0);
    let (mut lo, mut hi) = (((best as f64 - 1.0) / N as f64).max(0.0), ((best as f64 + 1.0) / N as f64).min(1.0));
    let g = 0.5 * (5f64.sqrt() - 1.0);
    for _ in 0..80 {
        let (a, b) = (hi - g * (hi - lo), lo + g * (hi - lo));
        if d(a) < d(b) { hi = b } else { lo = a }
    }
    0.5 * (lo + hi)
}

/// A length over the chord whose arc seed lies near its answer: a gentle sag.
const EASY: f64 = 1.2;

fn cold(lag: &Lagrangian, ends: &Ends, pegs: &[[f64; 2]]) -> Option<Shape> {
    let lay = Layout { pegs: pegs.len(), held: false };
    let (q, thetas) = seed(lag, ends, pegs)?;
    let pb = Problem { lag, ends, pegs, lay, held: &[] };
    let (q, e) = newton(&pb, q, thetas, NEWTON_MAX)?;
    shape(&pb, q, e)
}

/// From `p` to `ends` along the straight path between their outer columns (the pegs alike),
/// each step predicted along the last shape's derivative and corrected, the step halved where a
/// correction fails and grown where it succeeds.
fn walk(lag: &Lagrangian, p: &Shape, ends: &Ends, pegs: &[[f64; 2]]) -> Option<Shape> {
    let o0 = p.ends.outer();
    let o1 = ends.outer();
    let at = |f: f64| -> (Ends, Vec<[f64; 2]>) {
        let o: [f64; 5] = std::array::from_fn(|c| o0[c] + f * (o1[c] - o0[c]));
        let pg = p.pegs.iter().zip(pegs).map(|(a, b)| [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])]).collect();
        (Ends::of(&o), pg)
    };
    let mut cur = p.clone();
    let (mut done, mut step) = (0.0f64, 1.0f64);
    while done < 1.0 {
        if step < 1.0 / 256.0 {
            return None;
        }
        let f = (done + step).min(1.0);
        let (e, pg) = at(f);
        match correct(lag, &cur, &e, &pg) {
            Some(s) => {
                cur = s;
                done = f;
                step *= 1.5;
            }
            None => step *= 0.5,
        }
    }
    // a step's correction may stop at `GOOD`; the last is taken to `DONE`
    let lay = Layout { pegs: pegs.len(), held: false };
    let pb = Problem { lag, ends: &cur.ends, pegs: &cur.pegs, lay, held: &[] };
    let (q, e) = newton(&pb, cur.q.clone(), cur.thetas.clone(), NEWTON_MAX)?;
    shape(&pb, q, e)
}

/// `cur`'s unknowns predicted to `ends` along its derivative, and corrected there.
fn correct(lag: &Lagrangian, cur: &Shape, ends: &Ends, pegs: &[[f64; 2]]) -> Option<Shape> {
    let (o, oc) = (ends.outer(), cur.ends.outer());
    let n = cur.q.len();
    let q: Vec<f64> = (0..n)
        .map(|r| cur.q[r] + (0..5).map(|c| cur.dq[r * 5 + c] * (o[c] - oc[c])).sum::<f64>())
        .collect();
    let lay = Layout { pegs: pegs.len(), held: false };
    let pb = Problem { lag, ends, pegs, lay, held: &[] };
    let (q, ev) = newton(&pb, q, cur.thetas.clone(), CORRECT_MAX)?;
    shape(&pb, q, ev)
}

/// The shape re-solved with its pegs' places held at `held` — the verdict's question of how the
/// energy changes as they move.
pub fn solve_held(lag: &Lagrangian, from: &Shape, held: &[f64]) -> Option<Shape> {
    let lay = Layout { pegs: from.pegs.len(), held: true };
    let pb = Problem { lag, ends: &from.ends, pegs: &from.pegs, lay, held };
    // the free layout's unknowns less the places
    let free = Layout { pegs: from.pegs.len(), held: false };
    let mut q = Vec::with_capacity(lay.len());
    for (i, v) in from.q.iter().enumerate() {
        if !(1..=from.pegs.len()).any(|j| free.place(j) == Some(i)) {
            q.push(*v);
        }
    }
    let (q, e) = newton(&pb, q, from.thetas.clone(), CORRECT_MAX)?;
    shape(&pb, q, e)
}

/* -- the seed ----------------------------------------------------------------------------- */

/// A circular arc from `p` to `q` of length `len`, bowed to the side `side` (±1): its start
/// direction and curvature.  A length at or under the chord is a straight seed.
fn arc(p: [f64; 2], q: [f64; 2], len: f64, side: f64) -> (f64, f64) {
    let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
    let c = dx.dhypot(dy);
    let chord = dy.datan2(dx);
    if len <= c * (1.0 + 1e-9) || c == 0.0 {
        return (chord, 0.0);
    }
    // sin α / α = c / len, α in (0, π)
    let want = c / len;
    let (mut lo, mut hi) = (1e-9, std::f64::consts::PI - 1e-12);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if mid.dsin() / mid > want {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let alpha = 0.5 * (lo + hi);
    (chord + side * alpha, -side * 2.0 * alpha / len)
}

/// Where an arc of start direction `th0` and curvature `k` from `p` is at `s`, and its direction.
fn on_arc(p: [f64; 2], th0: f64, k: f64, s: f64) -> ([f64; 2], f64) {
    let th = th0 + k * s;
    if k.abs() < 1e-14 {
        let (sn, cs) = th0.dsin_cos();
        return ([p[0] + s * cs, p[1] + s * sn], th);
    }
    let (s0, c0) = th0.dsin_cos();
    let (s1, c1) = th.dsin_cos();
    ([p[0] + (s1 - s0) / k, p[1] - (c1 - c0) / k], th)
}

const SEED_SAMPLES: usize = 256;

/// The seed: per arc (the length shared over the arcs by their chords) the circular arc bowed to
/// the side of less energy, the costate at its start by least squares on `H_θ = 0` along it (the
/// costate is its start less `∫ f_p`, so the rows are linear in it), each piece's start read off
/// it.
fn seed(lag: &Lagrangian, ends: &Ends, pegs: &[[f64; 2]]) -> Option<(Vec<f64>, Vec<f64>)> {
    let mut pts = vec![ends.a];
    pts.extend_from_slice(pegs);
    pts.push(ends.b);
    let chords: Vec<f64> = pts.windows(2).map(|w| (w[1][0] - w[0][0]).dhypot(w[1][1] - w[0][1])).collect();
    let total: f64 = chords.iter().sum();
    if !(ends.len > 0.0) {
        return None;
    }
    let lay = Layout { pegs: pegs.len(), held: false };
    let mut q = vec![0.0; lay.len()];
    let mut thetas = Vec::new();
    let mut flow = Flow::new(lag, 0.0);
    let mut s_at = 0.0;
    for j in 0..pts.len() - 1 {
        let len = if total > 0.0 { ends.len * chords[j] / total } else { ends.len / chords.len() as f64 };
        // the side of less energy
        let energy = |flow: &mut Flow, side: f64| {
            let (th0, k) = arc(pts[j], pts[j + 1], len, side);
            (0..SEED_SAMPLES)
                .map(|i| {
                    let s = len * (i as f64 + 0.5) / SEED_SAMPLES as f64;
                    let (p, th) = on_arc(pts[j], th0, k, s);
                    flow.partials(th, p[0], p[1], false).f
                })
                .sum::<f64>()
        };
        let side = if energy(&mut flow, 1.0) <= energy(&mut flow, -1.0) { 1.0 } else { -1.0 };
        let (th0, k) = arc(pts[j], pts[j + 1], len, side);
        // ∫ f_p along it, and the least-squares rows for the starting costate
        let ds = len / SEED_SAMPLES as f64;
        let mut integral = [0.0f64; 2];
        let mut at = Vec::with_capacity(SEED_SAMPLES + 1);
        let (mut ata, mut atb) = ([[0.0f64; 2]; 2], [0.0f64; 2]);
        for i in 0..=SEED_SAMPLES {
            let s = i as f64 * ds;
            let (p, th) = on_arc(pts[j], th0, k, s);
            let d = flow.partials(th, p[0], p[1], false);
            if i > 0 {
                let prev: &(f64, [f64; 2], f64, [f64; 2], [f64; 2]) = &at[i - 1];
                integral[0] += 0.5 * ds * (prev.4[0] + d.x);
                integral[1] += 0.5 * ds * (prev.4[1] + d.y);
            }
            let (sn, cs) = th.dsin_cos();
            let row = [sn, -cs];
            let rhs = d.t + integral[0] * sn - integral[1] * cs;
            for a in 0..2 {
                for b in 0..2 {
                    ata[a][b] += row[a] * row[b];
                }
                atb[a] += row[a] * rhs;
            }
            at.push((s, p, th, integral, [d.x, d.y]));
        }
        let lam0 = {
            let a = crate::linalg::Mat::from_vec(2, 2, vec![ata[0][0], ata[0][1], ata[1][0], ata[1][1]]);
            let (x, _) = crate::linalg::min_norm_solve(&a, &atb, 1e-12);
            [x[0], x[1]]
        };
        if !(lam0[0].is_finite() && lam0[1].is_finite()) {
            return None;
        }
        if j > 0 {
            q[lay.place(j).expect("a peg's place")] = s_at;
        }
        let l = lay.lambda(j);
        q[l] = lam0[0];
        q[l + 1] = lam0[1];
        for m in 0..SEGMENTS {
            let i = m * SEED_SAMPLES / SEGMENTS;
            let (_, p, th, integ, _) = at[i];
            if m > 0 {
                let nd = lay.node(j, m);
                q[nd] = p[0];
                q[nd + 1] = p[1];
                q[nd + 2] = lam0[0] - integ[0];
                q[nd + 3] = lam0[1] - integ[1];
            }
            thetas.push(th);
        }
        s_at += len;
    }
    Some((q, thetas))
}

/* -- reading a shape ---------------------------------------------------------------------- */

/// The shape at `u` in [0, 1] (`s = uL`): the state there, its direction, and its derivative in
/// the outer columns and in `u`, the unknowns' dependence carried through `dq`.
pub struct At {
    pub z: [f64; 4],
    pub theta: f64,
    /// `dz/d[u, a.x, a.y, b.x, b.y, L]`, 4 × 6, row-major.
    pub dz: [f64; 24],
    /// The flow there.
    pub f: [f64; 4],
    /// `dθ/dz` there.
    pub tz: [f64; 4],
    pub point: flow::Point,
}

pub fn at(lag: &Lagrangian, sh: &Shape, u: f64) -> Option<At> {
    let lay = Layout { pegs: sh.pegs.len(), held: false };
    let pb = Problem { lag, ends: &sh.ends, pegs: &sh.pegs, lay, held: &[] };
    let len = sh.ends.len;
    let s = (u * len).clamp(0.0, len);
    let k = sh.pegs.len();
    let j = (1..=k).rev().find(|&j| s >= sh.places[j]).unwrap_or(0);
    let h = (sh.places[j + 1] - sh.places[j]) / SEGMENTS as f64;
    let m = (((s - sh.places[j]) / h).floor() as usize).min(SEGMENTS - 1);
    let piece = &sh.pieces[j * SEGMENTS + m];
    let node = within(lag, piece, s, sh.ends.scale())?;
    let (_, dz0) = pb.start(&sh.q, j, m);
    let mut fl = Flow::new(lag, node.theta);
    let mut f = [0.0; 4];
    if !fl.rhs(&node.z, &mut f, 0) {
        return None;
    }
    let p = fl.at(&node.z, false)?;
    let (sn, cs) = p.theta.dsin_cos();
    if p.htt == 0.0 {
        return None;
    }
    let tz = [-p.d.tx / p.htt, -p.d.ty / p.htt, sn / p.htt, -cs / p.htt];
    let n = sh.q.len();
    // dz/dq (4 × n) and dz/do directly (4 × 5)
    let mut dq = vec![0.0; 4 * n];
    let mut dodirect = [0.0f64; 20];
    let put = |dq: &mut Vec<f64>, dod: &mut [f64; 20], i: usize, v: Var, x: f64| match v {
        Var::Q(c) => dq[i * n + c] += x,
        Var::O(c) => dod[i * 5 + c] += x,
    };
    for i in 0..4 {
        for (c, d) in dz0.iter().enumerate() {
            if let Some(v) = d {
                put(&mut dq, &mut dodirect, i, *v, node.phi[i * 4 + c]);
            }
        }
        // the piece's start moves with the places it lies between: −f per unit
        let wts = [(pb.place_var(j), 1.0 - m as f64 / SEGMENTS as f64), (pb.place_var(j + 1), m as f64 / SEGMENTS as f64)];
        for (pv, wt) in wts {
            if let Some(v) = pv {
                put(&mut dq, &mut dodirect, i, v, -f[i] * wt);
            }
        }
        // and s = uL itself moves with the length
        dodirect[i * 5 + 4] += f[i] * u;
    }
    let mut dz = [0.0f64; 24];
    for i in 0..4 {
        dz[i * 6] = f[i] * len;
        for c in 0..5 {
            let mut v = dodirect[i * 5 + c];
            for r in 0..n {
                v += dq[i * n + r] * sh.dq[r * 5 + c];
            }
            dz[i * 6 + 1 + c] = v;
        }
    }
    Some(At { z: node.z, theta: p.theta, dz, f, tz, point: p })
}

