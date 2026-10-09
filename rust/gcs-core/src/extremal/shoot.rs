//! The boundary-value problem a curve's shape is the solution of (#144): integrated from its
//! start, landing on its end, through every stop pressing it.
//!
//! The curve is cut into **arcs** at its stops, and each arc into `SEGMENTS` equal pieces, each
//! integrated from a state of its own (multiple shooting: a piece's error grows only over the
//! piece, and a seed sets every piece's start).  The unknowns are the costate at the start, each
//! piece's starting state, and at each stop its place along the curve and the costate leaving
//! it; the rows join the pieces, put each stop's arc end on the stop with `H` unbroken across it
//! (the place is free), and put the last arc's end on `b`.  So the costate may jump at a stop —
//! a point force — and the direction with it: a corner.  Square, solved by a damped Newton over
//! its dense Jacobian, which the pieces' sensitivities give exactly.
//!
//! A stop is a **peg**, a held point, or a **slide** (#149), a held line the curve touches where
//! it chooses: there the place on the line is one more unknown, and the force along the line
//! none one more row — the jump in the costate (the gradient of the energy in where the corner
//! is) square to the line.

#[allow(unused_imports)]
use crate::fmath::Det;
use super::flow::{self, Flow, Lagrangian};

/// Pieces per arc.
pub const SEGMENTS: usize = 4;

/// What a curve's shape is asked for: its ends and its length.
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

    /// The ends from the curve's columns, `a`, `b`, the length (five numbers or more).
    pub fn of(o: &[f64]) -> Ends {
        Ends { a: [o[0], o[1]], b: [o[2], o[3]], len: o[4] }
    }

    /// A length to measure positions against.
    fn scale(&self) -> f64 {
        self.len.abs().max((self.b[0] - self.a[0]).dhypot(self.b[1] - self.a[1])).max(1e-9)
    }
}

/// Where a curve is pressed: at a held point it passes (a peg), or somewhere along a held line it
/// touches (a slide, #149) — the line through `o` along the unit `d`, the corner at `o + σd` with
/// `σ` the problem's to find.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Stop {
    Peg([f64; 2]),
    Slide { o: [f64; 2], d: [f64; 2] },
}

impl Stop {
    pub fn slides(&self) -> bool {
        matches!(self, Stop::Slide { .. })
    }

    /// The corner, `σ` along a slide's line (a peg is where it is).
    pub fn at(&self, sigma: f64) -> [f64; 2] {
        match *self {
            Stop::Peg(p) => p,
            Stop::Slide { o, d } => [o[0] + sigma * d[0], o[1] + sigma * d[1]],
        }
    }

    /// How far apart two stops of one kind are, to pair them across solves; none between kinds.
    fn far(&self, other: &Stop) -> Option<f64> {
        match (self, other) {
            (Stop::Peg(p), Stop::Peg(q)) => Some((p[0] - q[0]).dhypot(p[1] - q[1])),
            (Stop::Slide { o, d }, Stop::Slide { o: o2, d: d2 }) => {
                let n = [-d[1], d[0]];
                // the lines' separation where the first stands, and their turn
                Some(((o2[0] - o[0]) * n[0] + (o2[1] - o[1]) * n[1]).abs() + (d[0] - d2[0]).dhypot(d[1] - d2[1]))
            }
            _ => None,
        }
    }

    /// The stop `f` of the way to `to` (of its kind): a peg along the segment, a slide's line
    /// through the point between and turned between.
    fn toward(&self, to: &Stop, f: f64) -> Stop {
        let lerp = |a: [f64; 2], b: [f64; 2]| [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])];
        match (*self, *to) {
            (Stop::Slide { o, d }, Stop::Slide { o: o2, d: d2 }) => {
                let m = lerp(d, d2);
                let n = m[0].dhypot(m[1]);
                Stop::Slide { o: lerp(o, o2), d: [m[0] / n, m[1] / n] }
            }
            (Stop::Peg(p), Stop::Peg(q)) => Stop::Peg(lerp(p, q)),
            _ => *to,
        }
    }
}

/// One integrated piece: the state at each step's end with its sensitivity to the piece's start
/// (`phi`, 4 × 4, row-major).
#[derive(Clone, Debug)]
pub struct Piece {
    pub nodes: Vec<Node>,
}

#[derive(Clone, Copy, Debug)]
pub struct Node {
    pub s: f64,
    pub z: [f64; 4],
    pub theta: f64,
    pub phi: [f64; 16],
}

pub(super) const IDENTITY: [f64; 16] = [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.];

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
    Some(Piece { nodes })
}

/// The state at `s` within a piece (its start ≤ `s` ≤ its end): one step from the node before
/// it, with the sensitivity to the piece's start where `sens` (the piece integrated with them).
pub fn within(lag: &Lagrangian, piece: &Piece, s: f64, scale_len: f64, sens: bool) -> Option<Node> {
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
    let m = if sens { 4 } else { 0 };
    let (y1, _) = flow::step(&mut flow, n.theta, &y, h, m, &[scale_len, scale_len, lam, lam])?;
    let mut phi = IDENTITY;
    if sens {
        phi.copy_from_slice(&y1[4..20]);
    }
    Some(Node { s, z: [y1[0], y1[1], y1[2], y1[3]], theta: flow.theta, phi })
}

/* -- the problem's unknowns --------------------------------------------------------------- */

/// Where the unknowns sit, for its stops (`slides[j]` whether stop `j` is a slide): arc 0's
/// starting costate, then per arc after a stop its place, its place on a slide's line and its
/// starting costate, and per arc each later piece's starting state.  With `held` places (the
/// verdict's re-solves) neither place is an unknown and their rows are not stated.
#[derive(Clone, Debug)]
pub struct Layout {
    pub slides: Vec<bool>,
    pub held: bool,
}

/// What a quantity depends on: an unknown, an outer column (`a`, `b`, the length), or nothing.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Var {
    Q(usize),
    O(usize),
}

impl Layout {
    /// The layout over `stops`.
    pub fn of(stops: &[Stop], held: bool) -> Layout {
        Layout { slides: stops.iter().map(Stop::slides).collect(), held }
    }

    pub fn stops(&self) -> usize {
        self.slides.len()
    }

    /// The places heading arc `j ≥ 1` that are unknowns: along the curve, and along a slide's line.
    fn places_at(&self, j: usize) -> usize {
        if self.held { 0 } else { 1 + usize::from(self.slides[j - 1]) }
    }

    fn arc_start(&self, j: usize) -> usize {
        if j == 0 {
            0
        } else {
            (1..j).map(|i| 2 + self.places_at(i) + 4 * (SEGMENTS - 1)).sum::<usize>() + 2 + 4 * (SEGMENTS - 1)
        }
    }

    /// Arc `j`'s place (j ≥ 1), when it is an unknown.
    fn place(&self, j: usize) -> Option<usize> {
        (j >= 1 && !self.held).then(|| self.arc_start(j))
    }

    /// Arc `j`'s place on its slide's line (j ≥ 1), when it is an unknown.
    fn sigma(&self, j: usize) -> Option<usize> {
        (j >= 1 && !self.held && self.slides[j - 1]).then(|| self.arc_start(j) + 1)
    }

    /// Arc `j`'s starting costate.
    fn lambda(&self, j: usize) -> usize {
        self.arc_start(j) + if j == 0 { 0 } else { self.places_at(j) }
    }

    /// Piece `m ≥ 1` of arc `j`'s starting state.
    fn node(&self, j: usize, m: usize) -> usize {
        self.lambda(j) + 2 + 4 * (m - 1)
    }

    pub fn len(&self) -> usize {
        self.arc_start(self.stops() + 1)
    }
}

/// A solved shape: the problem it answers, its unknowns, where its arcs start, its pieces, and
/// the unknowns' derivative in the outer columns (`dq`, `n × 5`, row-major), the implicit
/// function theorem's, from the Jacobian the solve ended on.
#[derive(Clone, Debug)]
pub struct Shape {
    pub ends: Ends,
    pub stops: Vec<Stop>,
    pub q: Vec<f64>,
    /// Each arc's start along the curve, then the length: `0, s₁ … s_k, L`.
    pub places: Vec<f64>,
    /// Each stop's place on its line, `σ` (0 for a peg).
    pub sigmas: Vec<f64>,
    pub pieces: Vec<Piece>,
    pub dq: Vec<f64>,
}

/// Each piece's direction where it starts — where the next solve's start from.
fn thetas(pieces: &[Piece]) -> Vec<f64> {
    pieces.iter().map(|p| p.nodes[0].theta).collect()
}

/// The rows, their Jacobian in the unknowns and in the outer columns, at one set of unknowns.
struct Eval {
    r: Vec<f64>,
    /// What each row is measured against, for convergence.
    w: Vec<f64>,
    jq: Vec<f64>,
    jo: Vec<f64>,
    pieces: Vec<Piece>,
    places: Vec<f64>,
    sigmas: Vec<f64>,
}

impl Eval {
    /// Push a row: its value, what it is measured against, and its derivative over what it
    /// depends on (where the Jacobian is asked for).
    fn push(&mut self, n: usize, value: f64, scale: f64, deps: impl IntoIterator<Item = (Var, f64)>) -> usize {
        let row = self.r.len();
        self.r.push(value);
        self.w.push(scale);
        if !self.jq.is_empty() {
            for (v, f) in deps {
                match v {
                    Var::Q(i) => self.jq[row * n + i] += f,
                    Var::O(i) => self.jo[row * 5 + i] += f,
                }
            }
        }
        row
    }
}

struct Problem<'a> {
    lag: &'a Lagrangian,
    ends: &'a Ends,
    stops: &'a [Stop],
    lay: Layout,
    /// Each stop's places held — along the curve, and along a slide's line — where `lay.held`.
    held: &'a [[f64; 2]],
    /// The force along each slide's line its row asks for (none where empty): only while a
    /// pinned corner is let go (`release`).
    pull: &'a [f64],
}

/// What one of a piece's starting components moves with, and how much: at most one unknown or
/// outer column (a slide's corner moves along its line with its place there, `d` per unit).
type Dep = Option<(Var, f64)>;

impl<'a> Problem<'a> {
    /// The problem with every stop's place free.
    fn free(lag: &'a Lagrangian, ends: &'a Ends, stops: &'a [Stop]) -> Problem<'a> {
        Problem { lag, ends, stops, lay: Layout::of(stops, false), held: &[], pull: &[] }
    }

    fn places(&self, q: &[f64]) -> Vec<f64> {
        let k = self.lay.stops();
        let mut p = Vec::with_capacity(k + 2);
        p.push(0.0);
        for j in 1..=k {
            p.push(match self.lay.place(j) {
                Some(i) => q[i],
                None => self.held[j - 1][0],
            });
        }
        p.push(self.ends.len);
        p
    }

    /// Stop `j`'s (j ≥ 1) place on its line: 0 for a peg.
    fn sigma(&self, q: &[f64], j: usize) -> f64 {
        match (self.lay.sigma(j), self.stops[j - 1].slides()) {
            (Some(i), _) => q[i],
            (None, true) => self.held[j - 1][1],
            (None, false) => 0.0,
        }
    }

    /// The place `j` and what it is (0 and the length are a constant and an outer column).
    fn place_var(&self, j: usize) -> Option<Var> {
        if j == 0 {
            None
        } else if j == self.lay.stops() + 1 {
            Some(Var::O(4))
        } else {
            self.lay.place(j).map(Var::Q)
        }
    }

    /// Stop `j`'s (j ≥ 1) corner, and what each coordinate moves with.
    fn corner(&self, q: &[f64], j: usize) -> ([f64; 2], [Dep; 2]) {
        let stop = &self.stops[j - 1];
        let p = stop.at(self.sigma(q, j));
        match (stop, self.lay.sigma(j)) {
            (Stop::Slide { d, .. }, Some(i)) => (p, [Some((Var::Q(i), d[0])), Some((Var::Q(i), d[1]))]),
            _ => (p, [None, None]),
        }
    }

    /// Piece `(j, m)`'s starting state and what each component depends on: arc 0's start point
    /// on `a`, a later arc's on its stop, its costate an unknown; a later piece's all four.
    fn start(&self, q: &[f64], j: usize, m: usize) -> ([f64; 4], [Dep; 4]) {
        if m == 0 {
            let (p, dp) = match j {
                0 => (self.ends.a, [Some((Var::O(0), 1.0)), Some((Var::O(1), 1.0))]),
                _ => self.corner(q, j),
            };
            let l = self.lay.lambda(j);
            ([p[0], p[1], q[l], q[l + 1]], [dp[0], dp[1], Some((Var::Q(l), 1.0)), Some((Var::Q(l + 1), 1.0))])
        } else {
            let n = self.lay.node(j, m);
            ([q[n], q[n + 1], q[n + 2], q[n + 3]], std::array::from_fn(|c| Some((Var::Q(n + c), 1.0))))
        }
    }

    fn eval(&self, q: &[f64], thetas: &[f64], jac: bool) -> Option<Eval> {
        let n = self.lay.len();
        let k = self.lay.stops();
        let places = self.places(q);
        let scale_len = self.ends.scale();
        let mut e = Eval {
            r: Vec::with_capacity(n),
            w: Vec::with_capacity(n),
            jq: vec![0.0; if jac { n * n } else { 0 }],
            jo: vec![0.0; if jac { n * 5 } else { 0 }],
            pieces: Vec::with_capacity((k + 1) * SEGMENTS),
            places: Vec::new(),
            sigmas: (1..=k).map(|j| self.sigma(q, j)).collect(),
        };
        let lam_scale = (0..=k)
            .map(|j| q[self.lay.lambda(j)].dhypot(q[self.lay.lambda(j) + 1]))
            .fold(1e-300, f64::max);
        // each `H` row (the arc before a stop's end), and the arc after's start, filled in below
        let mut h_rows = Vec::with_capacity(k);
        let mut h_starts = Vec::with_capacity(k);
        for j in 0..=k {
            let len_arc = places[j + 1] - places[j];
            if !(len_arc > 0.0) {
                return None;
            }
            let h = len_arc / SEGMENTS as f64;
            // ∂h/∂(places): h = (s_{j+1} − s_j)/SEGMENTS
            let dh = [(self.place_var(j), -1.0 / SEGMENTS as f64), (self.place_var(j + 1), 1.0 / SEGMENTS as f64)];
            for m in 0..SEGMENTS {
                let (z0, dz0) = self.start(q, j, m);
                let th = thetas.get(j * SEGMENTS + m).copied().unwrap_or(0.0);
                let piece = integrate(self.lag, places[j] + h * m as f64, z0, th, h, scale_len, jac)?;
                if m == 0 && j > 0 && !self.lay.held {
                    let p = Flow::new(self.lag, piece.nodes[0].theta).at(&z0, false)?;
                    h_starts.push((p.h, p.h_z(), dz0));
                }
                let end = *piece.nodes.last().expect("a node");
                let pe = Flow::new(self.lag, end.theta).at(&end.z, false)?;
                // the end's derivative: through the piece's start, and its length (the flow there)
                let fz = pe.flow();
                let dend = |c: usize| {
                    let start =
                        dz0.iter().enumerate().filter_map(move |(cc, d)| d.map(|(v, w)| (v, w * end.phi[c * 4 + cc])));
                    let length = dh.into_iter().filter_map(move |(pv, f)| pv.map(|v| (v, fz[c] * f)));
                    start.chain(length)
                };
                if m + 1 < SEGMENTS {
                    let (z1, dz1) = self.start(q, j, m + 1);
                    let scales = [scale_len, scale_len, lam_scale, lam_scale];
                    for c in 0..4 {
                        e.push(n, end.z[c] - z1[c], scales[c], dend(c).chain(dz1[c].map(|(v, w)| (v, -w))));
                    }
                } else if j < k {
                    let (p, dp) = self.corner(q, j + 1);
                    for c in 0..2 {
                        e.push(n, end.z[c] - p[c], scale_len, dend(c).chain(dp[c].map(|(v, w)| (v, -w))));
                    }
                    // H unbroken across the stop, where its place is free: −H at this end here
                    if !self.lay.held {
                        let hz = pe.h_z();
                        let deps = (0..4).flat_map(|c| dend(c).map(move |(v, f)| (v, -hz[c] * f)));
                        h_rows.push(e.push(n, -pe.h, lam_scale.max(pe.h.abs()), deps));
                    }
                    // and, where its place on a slide's line is free, no force along the line:
                    // the costate's jump square to it
                    if let (Stop::Slide { d, .. }, Some(_)) = (self.stops[j], self.lay.sigma(j + 1)) {
                        let l = self.lay.lambda(j + 1);
                        let jump = (q[l] - end.z[2]) * d[0] + (q[l + 1] - end.z[3]) * d[1]
                            - self.pull.get(j).copied().unwrap_or(0.0);
                        let before = (0..2).flat_map(|c| dend(2 + c).map(move |(v, f)| (v, -d[c] * f)));
                        let after = (0..2).map(|c| (Var::Q(l + c), d[c]));
                        e.push(n, jump, lam_scale, before.chain(after));
                    }
                } else {
                    for c in 0..2 {
                        e.push(n, end.z[c] - self.ends.b[c], scale_len, dend(c).chain([(Var::O(2 + c), -1.0)]));
                    }
                }
                e.pieces.push(piece);
            }
        }
        // the H rows' other side: the arc after each stop, where it starts
        for (&row, (h, hz, dz)) in h_rows.iter().zip(h_starts) {
            e.r[row] += h;
            if jac {
                for c in 0..4 {
                    match dz[c] {
                        Some((Var::Q(i), w)) => e.jq[row * n + i] += hz[c] * w,
                        Some((Var::O(i), w)) => e.jo[row * 5 + i] += hz[c] * w,
                        None => {}
                    }
                }
            }
        }
        debug_assert_eq!(e.r.len(), n);
        e.places = places;
        Some(e)
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

/// Newton from `q`, damped by halving, for at most `most` iterations: the unknowns, the rows
/// there, and whether they reached `DONE` (else only `GOOD`).  The full step is tried with its
/// Jacobian, since it is nearly always the one taken.
fn newton(pb: &Problem, mut q: Vec<f64>, from: &[f64], most: usize) -> Option<(Vec<f64>, Eval, bool)> {
    let n = pb.lay.len();
    let mut e = pb.eval(&q, from, true)?;
    let mut piv = Vec::new();
    for _ in 0..most {
        let f = norm(&e);
        if f <= DONE {
            return Some((q, e, true));
        }
        let m0 = merit(&e);
        let mut a = e.jq.clone();
        if !crate::linalg::lu_factor(n, &mut a, &mut piv) {
            return None;
        }
        let mut dq: Vec<f64> = e.r.iter().map(|v| -v).collect();
        crate::linalg::lu_apply(n, &a, &piv, &mut dq);
        let from = thetas(&e.pieces);
        let mut t = 1.0;
        let mut next = None;
        while t > 1e-6 {
            let trial: Vec<f64> = q.iter().zip(&dq).map(|(a, b)| a + t * b).collect();
            if let Some(et) = pb.eval(&trial, &from, t == 1.0) {
                if merit(&et) < m0 * (1.0 - 0.5 * t) {
                    next = Some((trial, et));
                    break;
                }
            }
            t *= 0.5;
        }
        let Some((trial, et)) = next else { return (f <= GOOD).then_some((q, e, false)) };
        q = trial;
        e = if t == 1.0 { et } else { pb.eval(&q, &thetas(&et.pieces), true)? };
    }
    let f = norm(&e);
    (f <= GOOD).then_some((q, e, f <= DONE))
}

/// The solved shape from a converged `Eval`: the unknowns' derivative in the outer columns.
fn shape(pb: &Problem, q: Vec<f64>, e: Eval) -> Option<Shape> {
    let n = pb.lay.len();
    let mut a = e.jq;
    let mut piv = Vec::new();
    if !crate::linalg::lu_factor(n, &mut a, &mut piv) {
        return None;
    }
    let mut dq = vec![0.0; n * 5];
    let mut col = vec![0.0; n];
    for c in 0..5 {
        for i in 0..n {
            col[i] = -e.jo[i * 5 + c];
        }
        crate::linalg::lu_apply(n, &a, &piv, &mut col);
        for i in 0..n {
            dq[i * 5 + c] = col[i];
        }
    }
    Some(Shape {
        ends: pb.ends.clone(),
        stops: pb.stops.to_vec(),
        q,
        places: e.places,
        sigmas: e.sigmas,
        pieces: e.pieces,
        dq,
    })
}

/// Newton on `pb` from `q`, and the shape it comes to, with whether it reached `DONE`.
fn settle(pb: &Problem, q: Vec<f64>, thetas: &[f64], most: usize) -> Option<(Shape, bool)> {
    let (q, e, done) = newton(pb, q, thetas, most)?;
    Some((shape(pb, q, e)?, done))
}

/// The shape of a curve from `ends`, through `stops`: warm from `prev` where there is one, else
/// from the seed of a gently sagging curve (`EASY`), its length walked out to this one's
/// (`walk`) — an arc seed for a long curve between close ends is nothing like its answer (a long
/// rope is two strands and a tight bend), and from it Newton may find another stationary curve
/// (one with a loop); walked out, the curve stays on the branch it started on.
pub fn solve(lag: &Lagrangian, ends: &Ends, stops: &[Stop], prev: Option<&Shape>) -> Option<Shape> {
    if let Some((p, stops)) = prev.and_then(|p| Some((p, paired(&p.stops, stops)?))) {
        if p.ends == *ends && p.stops == stops {
            return Some(p.clone());
        }
        if let Some(s) = walk(lag, p, ends, &stops) {
            return Some(s);
        }
    }
    if !stops.is_empty() {
        return pressed(lag, ends, stops);
    }
    let chord = (ends.b[0] - ends.a[0]).dhypot(ends.b[1] - ends.a[1]);
    let short = chord * EASY;
    if ends.len <= short {
        return cold(lag, ends);
    }
    let easy = cold(lag, &Ends { len: short, ..ends.clone() })?;
    walk(lag, &easy, ends, &[])
}

/// `stops` in the order `before` had them along the curve, each paired with the nearest of the
/// last solve's of its kind (stops move a little between solves; their order along the curve
/// does not) — none where the two do not pair.
fn paired(before: &[Stop], stops: &[Stop]) -> Option<Vec<Stop>> {
    if before.len() != stops.len() {
        return None;
    }
    let mut left: Vec<Stop> = stops.to_vec();
    before
        .iter()
        .map(|b| {
            let (i, _) = left
                .iter()
                .enumerate()
                .filter_map(|(i, s)| Some((i, b.far(s)?)))
                .min_by(|x, y| x.1.total_cmp(&y.1))?;
            Some(left.remove(i))
        })
        .collect()
}

/// A curve through stops, from nothing: the curve without them, each stop first put where that
/// curve already passes nearest it — a shape the stops press with no force, exactly solved — and
/// then walked to where it is.  Their order along the curve is that curve's.  A slide is first
/// pinned, a peg where the curve's nearest point falls on its line (`foot`), and then let go
/// along the line (`release`): where the line misses the curve, the corner-free shape touching
/// a line moved onto it is a fork of the problem (sliding along the line and along the curve
/// are one motion there), so a slide is never walked in from one.
fn pressed(lag: &Lagrangian, ends: &Ends, stops: &[Stop]) -> Option<Shape> {
    let free = solve(lag, ends, &[], None)?;
    let mut near: Vec<(f64, Stop)> = stops.iter().map(|&p| (nearest(lag, &free, &p), p)).collect();
    near.sort_by(|a, b| a.0.total_cmp(&b.0));
    if near.windows(2).any(|w| w[1].0 <= w[0].0) || near.iter().any(|n| n.0 <= 0.0 || n.0 >= 1.0) {
        return None;
    }
    let len = ends.len;
    let real: Vec<Stop> = near.iter().map(|n| n.1).collect();
    let mut pins = Vec::with_capacity(real.len());
    for &(u, s) in &near {
        pins.push(match s {
            Stop::Peg(_) => s,
            Stop::Slide { o, d } => {
                let c = position(lag, &free, u)?;
                Stop::Peg(s.at((c[0] - o[0]) * d[0] + (c[1] - o[1]) * d[1]))
            }
        });
    }
    let lay = Layout::of(&pins, false);
    let mut q = vec![0.0; lay.len()];
    let mut thetas = Vec::with_capacity((stops.len() + 1) * SEGMENTS);
    let mut virtual_pegs = Vec::with_capacity(stops.len());
    let mut places = vec![0.0];
    places.extend(near.iter().map(|n| n.0 * len));
    places.push(len);
    for j in 0..=stops.len() {
        let h = (places[j + 1] - places[j]) / SEGMENTS as f64;
        for m in 0..SEGMENTS {
            let at = at(lag, &free, (places[j] + h * m as f64) / len, false)?;
            thetas.push(at.theta);
            if m == 0 {
                if j > 0 {
                    q[lay.place(j).expect("a place")] = places[j];
                    virtual_pegs.push(Stop::Peg([at.z[0], at.z[1]]));
                }
                let l = lay.lambda(j);
                q[l..l + 2].copy_from_slice(&at.z[2..]);
            } else {
                let n = lay.node(j, m);
                q[n..n + 4].copy_from_slice(&at.z);
            }
        }
    }
    let (start, _) = settle(&Problem::free(lag, ends, &virtual_pegs), q, &thetas, NEWTON_MAX)?;
    let pinned = walk(lag, &start, ends, &pins)?;
    if !real.iter().any(Stop::slides) {
        return Some(pinned);
    }
    release(lag, &pinned, &real)
}

/// A shape pinned at pegs let go along its slides' lines (`stops`, a slide where `pinned` has
/// a peg on its line): first the pinned shape itself, each slide holding the force along its
/// line the peg bore, then that force walked down to none — a corner on a line pushed along it
/// and eased off.
fn release(lag: &Lagrangian, pinned: &Shape, stops: &[Stop]) -> Option<Shape> {
    let (old, lay) = (Layout::of(&pinned.stops, false), Layout::of(stops, false));
    let k = stops.len();
    let mut q = vec![0.0; lay.len()];
    let mut pull = vec![0.0; k];
    q[..2].copy_from_slice(&pinned.q[..2]);
    for j in 0..=k {
        if j > 0 {
            q[lay.place(j)?] = pinned.q[old.place(j)?];
            let (l0, l1) = (old.lambda(j), lay.lambda(j));
            q[l1..l1 + 2].copy_from_slice(&pinned.q[l0..l0 + 2]);
            if let (Stop::Slide { o, d }, Stop::Peg(p)) = (stops[j - 1], pinned.stops[j - 1]) {
                q[lay.sigma(j)?] = (p[0] - o[0]) * d[0] + (p[1] - o[1]) * d[1];
                let before = *pinned.pieces[j * SEGMENTS - 1].nodes.last()?;
                pull[j - 1] = (q[l1] - before.z[2]) * d[0] + (q[l1 + 1] - before.z[3]) * d[1];
            }
        }
        for m in 1..SEGMENTS {
            let (n0, n1) = (old.node(j, m), lay.node(j, m));
            q[n1..n1 + 4].copy_from_slice(&pinned.q[n0..n0 + 4]);
        }
    }
    let mut from = thetas(&pinned.pieces);
    let (mut done, mut step) = (0.0f64, 1.0f64);
    let mut last: Option<(Shape, bool)> = None;
    while done < 1.0 {
        if step < 1.0 / 256.0 {
            return None;
        }
        let f = (done + step).min(1.0);
        let eased: Vec<f64> = pull.iter().map(|p| (1.0 - f) * p).collect();
        let problem = Problem { pull: &eased, ..Problem::free(lag, &pinned.ends, stops) };
        match settle(&problem, q.clone(), &from, CORRECT_MAX) {
            Some((s, d)) => {
                q = s.q.clone();
                from = thetas(&s.pieces);
                last = Some((s, d));
                done = f;
                step *= 1.5;
            }
            None => step *= 0.5,
        }
    }
    let (s, d) = last?;
    if d {
        return Some(s);
    }
    Some(settle(&Problem::free(lag, &s.ends, stops), s.q.clone(), &from, NEWTON_MAX)?.0)
}

/// Where along `sh` (in `u`) it passes nearest stop `p` — a slide's line where it passes nearest
/// it (the first it would touch, moved square to itself; where it crosses, the crossing): the
/// best of an even sampling, refined by Brent's minimisation between its neighbours.
fn nearest(lag: &Lagrangian, sh: &Shape, p: &Stop) -> f64 {
    const N: usize = 256;
    let d = |u: f64| {
        position(lag, sh, u).map_or(f64::INFINITY, |c| match *p {
            Stop::Peg(p) => (c[0] - p[0]).dhypot(c[1] - p[1]),
            Stop::Slide { o, d } => ((c[0] - o[0]) * d[1] - (c[1] - o[1]) * d[0]).abs(),
        })
    };
    let best = (0..=N).map(|i| (d(i as f64 / N as f64), i)).min_by(|a, b| a.0.total_cmp(&b.0)).map_or(0, |b| b.1);
    let (lo, hi) = (((best as f64 - 1.0) / N as f64).max(0.0), ((best as f64 + 1.0) / N as f64).min(1.0));
    crate::roots::brent(&d, lo, hi, 1e-13, 100, |_, _| false).1
}

/// A length over the chord whose arc seed lies near its answer: a gentle sag.
pub const EASY: f64 = 1.2;

/// A curve with no stops from its seed.
fn cold(lag: &Lagrangian, ends: &Ends) -> Option<Shape> {
    let (q, thetas) = seed(lag, ends)?;
    Some(settle(&Problem::free(lag, ends, &[]), q, &thetas, NEWTON_MAX)?.0)
}

/// From `p` to `ends` along the straight path between their outer columns (the stops alike),
/// each step predicted along the last shape's derivative and corrected, the step halved where a
/// correction fails and grown where it succeeds; the last taken to `DONE` if its correction
/// stopped at `GOOD`.
fn walk(lag: &Lagrangian, p: &Shape, ends: &Ends, stops: &[Stop]) -> Option<Shape> {
    let o0 = p.ends.outer();
    let o1 = ends.outer();
    let at = |f: f64| -> (Ends, Vec<Stop>) {
        let o: [f64; 5] = std::array::from_fn(|c| o0[c] + f * (o1[c] - o0[c]));
        let st = p.stops.iter().zip(stops).map(|(a, b)| a.toward(b, f)).collect();
        (Ends::of(&o), st)
    };
    let mut cur = p.clone();
    let mut done_last = true;
    let (mut done, mut step) = (0.0f64, 1.0f64);
    while done < 1.0 {
        if step < 1.0 / 256.0 {
            return None;
        }
        let f = (done + step).min(1.0);
        let (e, st) = at(f);
        match correct(lag, &cur, &e, &st) {
            Some((s, d)) => {
                cur = s;
                done_last = d;
                done = f;
                step *= 1.5;
            }
            None => step *= 0.5,
        }
    }
    if done_last {
        return Some(cur);
    }
    let pb = Problem::free(lag, &cur.ends, &cur.stops);
    Some(settle(&pb, cur.q.clone(), &thetas(&cur.pieces), NEWTON_MAX)?.0)
}

/// `cur`'s unknowns predicted to `ends` along its derivative, and corrected there.
fn correct(lag: &Lagrangian, cur: &Shape, ends: &Ends, stops: &[Stop]) -> Option<(Shape, bool)> {
    let (o, oc) = (ends.outer(), cur.ends.outer());
    let q: Vec<f64> = (0..cur.q.len())
        .map(|r| cur.q[r] + (0..5).map(|c| cur.dq[r * 5 + c] * (o[c] - oc[c])).sum::<f64>())
        .collect();
    settle(&Problem::free(lag, ends, stops), q, &thetas(&cur.pieces), CORRECT_MAX)
}

/// The shape re-solved with its stops' places held at `held` — along the curve, and along a
/// slide's line — the verdict's question of how the energy changes as they move.
pub fn solve_held(lag: &Lagrangian, from: &Shape, held: &[[f64; 2]]) -> Option<Shape> {
    let pb = Problem { lag, ends: &from.ends, stops: &from.stops, lay: Layout::of(&from.stops, true), held, pull: &[] };
    // the free layout's unknowns less the places
    let free = Layout::of(&from.stops, false);
    let k = from.stops.len();
    let q: Vec<f64> = (0..from.q.len())
        .filter(|&i| !(1..=k).any(|j| free.place(j) == Some(i) || free.sigma(j) == Some(i)))
        .map(|i| from.q[i])
        .collect();
    Some(settle(&pb, q, &thetas(&from.pieces), CORRECT_MAX)?.0)
}

/* -- the seed ----------------------------------------------------------------------------- */

/// A circular arc from `p` to `q` of length `len`, bowed to the side `side` (±1): its start
/// direction and curvature (`sin α / α = c / len`, increasing towards 0, so bracketed).  A
/// length at or under the chord is a straight seed.
fn arc(p: [f64; 2], q: [f64; 2], len: f64, side: f64) -> (f64, f64) {
    let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
    let c = dx.dhypot(dy);
    let chord = dy.datan2(dx);
    if len <= c * (1.0 + 1e-9) || c == 0.0 {
        return (chord, 0.0);
    }
    let g = |a: f64| a.dsin() / a - c / len;
    let (lo, hi) = (1e-9, std::f64::consts::PI - 1e-12);
    let alpha = crate::roots::bracketed_root(|a| Some(g(a)), lo, g(lo), hi, g(hi), 1e-14);
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

/// The seed of a curve with no stops (a pressed one starts from it, `pressed`): the circular arc of
/// its length through its ends, bowed to the side of less energy, the costate at its start by
/// least squares on `H_θ = 0` along it (the costate is its start less `∫ f_p`, so the rows are
/// linear in it), each piece's start read off it.
fn seed(lag: &Lagrangian, ends: &Ends) -> Option<(Vec<f64>, Vec<f64>)> {
    if !(ends.len > 0.0) {
        return None;
    }
    let (a, b, len) = (ends.a, ends.b, ends.len);
    let mut flow = Flow::new(lag, 0.0);
    let energy = |flow: &mut Flow, side: f64| {
        let (th0, k) = arc(a, b, len, side);
        (0..SEED_SAMPLES)
            .map(|i| {
                let (p, th) = on_arc(a, th0, k, len * (i as f64 + 0.5) / SEED_SAMPLES as f64);
                flow.partials(th, p[0], p[1], false).f
            })
            .sum::<f64>()
    };
    let side = if energy(&mut flow, 1.0) <= energy(&mut flow, -1.0) { 1.0 } else { -1.0 };
    let (th0, k) = arc(a, b, len, side);
    // ∫ f_p along it (trapezoidal), and the least-squares rows `[sin θ, −cos θ]·λ₀ = …` there
    let ds = len / SEED_SAMPLES as f64;
    let mut integral = [0.0f64; 2];
    let mut last = [0.0f64; 2];
    let mut along = Vec::with_capacity(SEED_SAMPLES + 1);
    let mut rows = crate::linalg::Mat::zeros(SEED_SAMPLES + 1, 2);
    let mut rhs = Vec::with_capacity(SEED_SAMPLES + 1);
    for i in 0..=SEED_SAMPLES {
        let (p, th) = on_arc(a, th0, k, i as f64 * ds);
        let d = flow.partials(th, p[0], p[1], false);
        if i > 0 {
            integral[0] += 0.5 * ds * (last[0] + d.x);
            integral[1] += 0.5 * ds * (last[1] + d.y);
        }
        last = [d.x, d.y];
        let (sn, cs) = th.dsin_cos();
        rows.data[2 * i] = sn;
        rows.data[2 * i + 1] = -cs;
        rhs.push(d.t + integral[0] * sn - integral[1] * cs);
        along.push((p, th, integral));
    }
    let (lam0, _) = crate::linalg::min_norm_solve(&rows, &rhs, 1e-12);
    if !(lam0[0].is_finite() && lam0[1].is_finite()) {
        return None;
    }
    let lay = Layout::of(&[], false);
    let mut q = vec![0.0; lay.len()];
    q[..2].copy_from_slice(&lam0[..2]);
    let mut thetas = Vec::with_capacity(SEGMENTS);
    for m in 0..SEGMENTS {
        let (p, th, integ) = along[m * SEED_SAMPLES / SEGMENTS];
        if m > 0 {
            let nd = lay.node(0, m);
            q[nd..nd + 4].copy_from_slice(&[p[0], p[1], lam0[0] - integ[0], lam0[1] - integ[1]]);
        }
        thetas.push(th);
    }
    Some((q, thetas))
}

/* -- reading a shape ---------------------------------------------------------------------- */

/// The shape at `u` in [0, 1] (`s = uL`): the state there, its direction, and — where asked —
/// its derivative in the outer columns and in `u`, the unknowns' dependence carried through `dq`.
pub struct At {
    pub z: [f64; 4],
    pub theta: f64,
    /// `dz/d[u, a.x, a.y, b.x, b.y, L]`, 4 × 6, row-major (zero where not asked for).
    pub dz: [f64; 24],
    /// The flow there.
    pub f: [f64; 4],
    pub point: flow::Point,
}

/// Where `u` is: its arc length `s`, and the arc and piece it falls in.
fn locate(sh: &Shape, u: f64) -> (f64, usize, usize) {
    let len = sh.ends.len;
    let s = (u * len).clamp(0.0, len);
    let j = (1..=sh.stops.len()).rev().find(|&j| s >= sh.places[j]).unwrap_or(0);
    let h = (sh.places[j + 1] - sh.places[j]) / SEGMENTS as f64;
    (s, j, (((s - sh.places[j]) / h).floor() as usize).min(SEGMENTS - 1))
}

pub fn at(lag: &Lagrangian, sh: &Shape, u: f64, deriv: bool) -> Option<At> {
    let len = sh.ends.len;
    let (s, j, m) = locate(sh, u);
    let piece = &sh.pieces[j * SEGMENTS + m];
    let node = within(lag, piece, s, sh.ends.scale(), deriv)?;
    let p = Flow::new(lag, node.theta).at(&node.z, false)?;
    let f = p.flow();
    let mut dz = [0.0f64; 24];
    if deriv {
        let pb = Problem::free(lag, &sh.ends, &sh.stops);
        let (_, dz0) = pb.start(&sh.q, j, m);
        // the piece's start moves with the places it lies between: −f per unit
        let wts = [(pb.place_var(j), 1.0 - m as f64 / SEGMENTS as f64), (pb.place_var(j + 1), m as f64 / SEGMENTS as f64)];
        for i in 0..4 {
            let row = &mut dz[i * 6..i * 6 + 6];
            row[0] = f[i] * len;
            // and s = uL itself moves with the length
            row[5] += f[i] * u;
            let deps = dz0.iter().enumerate().filter_map(|(c, d)| d.map(|(v, w)| (v, w * node.phi[i * 4 + c])));
            let places = wts.iter().filter_map(|&(pv, wt)| pv.map(|v| (v, -f[i] * wt)));
            for (v, x) in deps.chain(places) {
                match v {
                    Var::O(c) => row[1 + c] += x,
                    Var::Q(r) => {
                        for c in 0..5 {
                            row[1 + c] += x * sh.dq[r * 5 + c];
                        }
                    }
                }
            }
        }
    }
    Some(At { z: node.z, theta: p.theta, dz, f, point: p })
}

/// Where the shape is at `u`, alone — one step from the node before it, no sensitivities.
pub fn position(lag: &Lagrangian, sh: &Shape, u: f64) -> Option<[f64; 2]> {
    let (s, j, m) = locate(sh, u);
    let n = within(lag, &sh.pieces[j * SEGMENTS + m], s, sh.ends.scale(), false)?;
    Some([n.z[0], n.z[1]])
}
