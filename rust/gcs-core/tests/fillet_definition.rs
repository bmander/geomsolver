//! **X2 of #183: what a round is, by brute force.**  Candidate definitions of `A round(r) B`,
//! each a classifier of points, their volumes integrated and held against the closed forms the
//! Rust fillet stack (#79–#110) checks its fixtures by.  Nothing here reads the core: bodies are
//! Booleans of planes and cylinders written out below, each with its exact signed distance, so the
//! test and the thing it decides share no implementation.
//!
//! A named edge is where a face of `A` meets a face of `B`.  Each face's carrier `g` is a signed
//! distance, positive on the material's side.  Where the two meet convex (the Boolean joining
//! them is an intersection) the ball rolls inside the material, and a point of the body is
//! **removed** when no ball of radius `r` lying on the material side of both carriers covers it:
//! its distance from `C = {g_a ≥ r} ∩ {g_b ≥ r}`, the balls' centres, is more than `r`.  Where
//! they meet concave (a union), the same with every side turned round, and a point of the void
//! is **added**.  A term acts where its ball is the ball of a point of the body's edge
//! (`Term::reaches`), so carriers meeting where the body has no edge (two cylinders' far
//! meeting, a cap a cut ate) make nothing, and a run ends where its edge does.  The candidates
//! (#183):
//!
//! - **(a), pairwise:** one term per named edge.
//! - **(a + v):** (a), plus a term of three faces at each vertex where three named edges meet.
//! - **(b), joint:** one term of every named face at once.
//!
//! **X3** (the words in Solvent, on paper) asks the same cases two more questions: whether the
//! reach can be said in the point's own distances (`Reach`), as a set must say it, and whether a
//! chamfer is a bound on a sum of two distances (`Round::chamfer`).
//!
//! Volumes are integrated over an octree pruned by each term's 1-Lipschitz bound (`Round::at`)
//! and cut to a thirty-second of the radius, each last cell read at one point placed in it at
//! random (by a hash of where it is).
use std::f64::consts::PI;

type V = [f64; 3];

fn add(a: V, b: V) -> V { [a[0] + b[0], a[1] + b[1], a[2] + b[2]] }
fn sub(a: V, b: V) -> V { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
fn scale(a: V, k: f64) -> V { [a[0] * k, a[1] * k, a[2] * k] }
fn dot(a: V, b: V) -> f64 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn norm(a: V) -> f64 { dot(a, a).sqrt() }
fn unit(a: V) -> V { scale(a, 1.0 / norm(a)) }

/// A 3 × 3 system `rows · x = rhs`, by Cramer's rule; none where it is singular.
fn solve3(rows: [V; 3], rhs: V) -> Option<V> {
    let det = dot(rows[0], cross(rows[1], rows[2]));
    if det.abs() < 1e-12 {
        return None;
    }
    let col = |k: usize| {
        let mut m = rows;
        for (i, row) in m.iter_mut().enumerate() {
            row[k] = rhs[i];
        }
        dot(m[0], cross(m[1], m[2])) / det
    };
    Some([col(0), col(1), col(2)])
}

// -- carriers ----------------------------------------------------------------------------------

/// A face's carrier, by its material side: `n · p ≤ d`, or within (`inside`) or without radius
/// `r` of the line through `a` along the unit `u`.  `g` is the exact signed distance, positive on
/// the material side.
#[derive(Clone, Copy, Debug)]
enum Surf {
    Plane { n: V, d: f64 },
    Cyl { a: V, u: V, r: f64, inside: bool },
}

impl Surf {
    fn g(&self, p: V) -> f64 {
        match *self {
            Surf::Plane { n, d } => d - dot(n, p),
            Surf::Cyl { a, u, r, inside } => {
                let rho = norm(cross(sub(p, a), u));
                if inside { r - rho } else { rho - r }
            }
        }
    }
    /// The other side of the same surface.
    fn flip(self) -> Surf {
        match self {
            Surf::Plane { n, d } => Surf::Plane { n: scale(n, -1.0), d: -d },
            Surf::Cyl { a, u, r, inside } => Surf::Cyl { a, u, r, inside: !inside },
        }
    }
    /// `{g ≥ s}`, as a carrier whose `g` is `g − s`; none where it is empty.
    fn offset(self, s: f64) -> Option<Surf> {
        match self {
            Surf::Plane { n, d } => Some(Surf::Plane { n, d: d - s }),
            Surf::Cyl { a, u, r, inside } => {
                let r = if inside { r - s } else { r + s };
                (r > 0.0).then_some(Surf::Cyl { a, u, r, inside })
            }
        }
    }
    /// The surface's point nearest `p` (none on a cylinder's axis).
    fn foot(&self, p: V) -> Option<V> {
        match *self {
            Surf::Plane { .. } => Some(add(p, scale(self.normal(), self.g(p)))),
            Surf::Cyl { a, u, r, .. } => {
                let v = sub(p, a);
                let along = scale(u, dot(v, u));
                let radial = sub(v, along);
                let rho = norm(radial);
                (rho > 1e-12).then(|| add(add(a, along), scale(radial, r / rho)))
            }
        }
    }
    /// The gradient of `g`: into the material side.
    fn grad(&self, p: V) -> V {
        match *self {
            Surf::Plane { n, .. } => scale(n, -1.0),
            Surf::Cyl { a, u, inside, .. } => {
                let v = sub(p, a);
                let radial = unit(sub(v, scale(u, dot(v, u))));
                if inside { scale(radial, -1.0) } else { radial }
            }
        }
    }
    /// A plane's normal, out of the material.
    fn normal(&self) -> V {
        match *self {
            Surf::Plane { n, .. } => n,
            Surf::Cyl { .. } => unreachable!("a plane's"),
        }
    }
}

/// The material side `n · p ≤ d` of a plane along `n` (made unit).
fn plane(n: V, d: f64) -> Surf {
    let l = norm(n);
    Surf::Plane { n: scale(n, 1.0 / l), d: d / l }
}

/// The six carriers of the box from `lo` to `hi`: x low, x high, y low, y high, z low, z high.
fn boxed(lo: V, hi: V) -> Vec<Surf> {
    let mut out = Vec::new();
    for k in 0..3 {
        let mut e = [0.0; 3];
        e[k] = 1.0;
        out.push(plane(scale(e, -1.0), -lo[k]));
        out.push(plane(e, hi[k]));
    }
    out
}

/// An orthonormal pair square to the unit `u`.
fn square_to(u: V) -> (V, V) {
    let t = if u[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let e1 = unit(cross(u, t));
    (e1, cross(u, e1))
}

/// The points of the curve where two carriers meet that are nearest `p`, one to a branch: a line
/// of two planes in closed form, else round the (narrower) cylinder by its angle.
fn curve_nearest(p: V, s: Surf, q: Surf) -> Vec<V> {
    if let (Surf::Plane { n: n1, d: d1 }, Surf::Plane { n: n2, d: d2 }) = (s, q) {
        let dir = cross(n1, n2);
        if norm(dir) < 1e-12 {
            return Vec::new();
        }
        return solve3([n1, n2, dir], [d1, d2, dot(dir, p)]).into_iter().collect();
    }
    let (k, x) = match (s, q) {
        (Surf::Cyl { r: r1, .. }, Surf::Cyl { r: r2, .. }) => {
            if r1 <= r2 { (s, q) } else { (q, s) }
        }
        (Surf::Cyl { .. }, _) => (s, q),
        _ => (q, s),
    };
    let Surf::Cyl { a, u, r, .. } = k else { unreachable!() };
    let (e1, e2) = square_to(u);
    // the points at angle `phi` round `k` that lie on `x`, lowest along `u` first
    let at = |phi: f64| -> [Option<V>; 2] {
        let w = add(a, add(scale(e1, r * phi.cos()), scale(e2, r * phi.sin())));
        let ts: [Option<f64>; 2] = match x {
            Surf::Plane { n, d } => {
                let nu = dot(n, u);
                [(nu.abs() >= 1e-12).then(|| (d - dot(n, w)) / nu), None]
            }
            Surf::Cyl { a: a2, u: u2, r: r2, .. } => {
                let b = cross(sub(w, a2), u2);
                let c = cross(u, u2);
                let (qa, qb, qc) = (dot(c, c), 2.0 * dot(b, c), dot(b, b) - r2 * r2);
                let disc = qb * qb - 4.0 * qa * qc;
                if qa < 1e-15 || disc < 0.0 {
                    [None, None]
                } else {
                    let w = disc.sqrt();
                    [Some((-qb - w) / (2.0 * qa)), Some((-qb + w) / (2.0 * qa))]
                }
            }
        };
        ts.map(|t| t.map(|t| add(w, scale(u, t))))
    };
    // the nearest point is near `p`'s own angle round `k` (a round's material is within a few
    // radii of its spine): golden section over a radian either side, each branch
    let v = sub(p, a);
    let around = dot(v, e2).atan2(dot(v, e1));
    let mut out = Vec::new();
    for branch in 0..2 {
        let d = |phi: f64| at(phi)[branch].map_or(f64::INFINITY, |q| norm(sub(q, p)));
        let (mut lo, mut hi) = (around - 1.0, around + 1.0);
        let g = (5f64.sqrt() - 1.0) / 2.0;
        let (mut m1, mut m2) = (hi - g * (hi - lo), lo + g * (hi - lo));
        let (mut v1, mut v2) = (d(m1), d(m2));
        for _ in 0..36 {
            if v1 < v2 {
                hi = m2;
                (m2, v2) = (m1, v1);
                m1 = hi - g * (hi - lo);
                v1 = d(m1);
            } else {
                lo = m1;
                (m1, v1) = (m2, v2);
                m2 = lo + g * (hi - lo);
                v2 = d(m2);
            }
        }
        if let Some(q) = at((lo + hi) / 2.0)[branch] {
            out.push(q);
        }
    }
    out
}

/// Where three planes meet (none where they do not meet in a point, or one is no plane).
fn corner(a: Surf, b: Surf, c: Surf) -> Option<V> {
    match (a, b, c) {
        (Surf::Plane { n: a, d: da }, Surf::Plane { n: b, d: db }, Surf::Plane { n: c, d: dc }) => {
            solve3([a, b, c], [da, db, dc])
        }
        _ => None,
    }
}

/// The point of `C = ∩ {h_k ≥ 0}` nearest `p`, its distance and the carriers it lies on: `p`
/// itself, or a foot on one carrier, or a point of a curve where two meet, or a corner of three
/// planes, whichever is in `C` and nearest.  None where `C` has no point.
fn nearest(p: V, hs: &[Surf]) -> Option<(f64, V, Vec<usize>)> {
    if hs.iter().all(|h| h.g(p) >= 0.0) {
        return Some((0.0, p, Vec::new()));
    }
    let mut best: Option<(f64, V, Vec<usize>)> = None;
    let mut offer = |q: V, active: Vec<usize>| {
        let holds = hs.iter().enumerate().all(|(j, h)| active.contains(&j) || h.g(q) >= -1e-9);
        let d = norm(sub(q, p));
        if holds && best.as_ref().is_none_or(|b| d < b.0) {
            best = Some((d, q, active));
        }
    };
    for (k, h) in hs.iter().enumerate() {
        if let Some(q) = h.foot(p) {
            offer(q, vec![k]);
        }
    }
    for k in 0..hs.len() {
        for l in k + 1..hs.len() {
            for q in curve_nearest(p, hs[k], hs[l]) {
                offer(q, vec![k, l]);
            }
        }
    }
    for k in 0..hs.len() {
        for l in k + 1..hs.len() {
            for m in l + 1..hs.len() {
                if let Some(q) = corner(hs[k], hs[l], hs[m]) {
                    offer(q, vec![k, l, m]);
                }
            }
        }
    }
    best
}

// -- bodies ------------------------------------------------------------------------------------

/// A body as a Boolean of carriers' material sides, every complement turned into its leaves.
#[derive(Clone, Debug)]
enum Node {
    Leaf(usize),
    And(Vec<Node>),
    Or(Vec<Node>),
}

fn and(range: std::ops::Range<usize>) -> Node { Node::And(range.map(Node::Leaf).collect()) }
fn or(range: std::ops::Range<usize>) -> Node { Node::Or(range.map(Node::Leaf).collect()) }

struct Body {
    leaves: Vec<Surf>,
    tree: Node,
}

impl Body {
    /// A 1-Lipschitz function positive inside, its leaves' `g` by min and max; `lifted` leaves
    /// read as no bound (each within an intersection).
    fn s_with(&self, p: V, lifted: &[usize]) -> f64 {
        fn eval(n: &Node, p: V, leaves: &[Surf], lifted: &[usize]) -> f64 {
            match n {
                Node::Leaf(k) if lifted.contains(k) => f64::INFINITY,
                Node::Leaf(k) => leaves[*k].g(p),
                Node::And(v) => {
                    v.iter().map(|c| eval(c, p, leaves, lifted)).fold(f64::INFINITY, f64::min)
                }
                Node::Or(v) => {
                    v.iter().map(|c| eval(c, p, leaves, lifted)).fold(f64::NEG_INFINITY, f64::max)
                }
            }
        }
        eval(&self.tree, p, &self.leaves, lifted)
    }
    fn s(&self, p: V) -> f64 { self.s_with(p, &[]) }
    /// Whether the Boolean that brings leaves `a` and `b` together is an intersection: whether
    /// their edge is convex.
    fn convex(&self, a: usize, b: usize) -> bool {
        fn find(n: &Node, a: usize, b: usize) -> (bool, bool, Option<bool>) {
            match n {
                Node::Leaf(k) => (*k == a, *k == b, None),
                Node::And(v) | Node::Or(v) => {
                    let (mut fa, mut fb) = (false, false);
                    for c in v {
                        let (x, y, met) = find(c, a, b);
                        if met.is_some() {
                            return (true, true, met);
                        }
                        fa |= x;
                        fb |= y;
                    }
                    (fa, fb, (fa && fb).then_some(matches!(n, Node::And(_))))
                }
            }
        }
        find(&self.tree, a, b).2.expect("both leaves in the body")
    }
}

// -- terms and candidates ----------------------------------------------------------------------

/// One term of a round: faces whose carriers' balls of radius `r` take material away (`sign` +1,
/// convex) or add it (`sign` −1, concave).  `sides` are the faces' sides the ball rolls on (the
/// material's, or the void's), `offsets` those sides shrunk by `r`: the balls' centres.
struct Term {
    faces: Vec<usize>,
    sides: Vec<Surf>,
    offsets: Option<Vec<Surf>>,
    sign: f64,
    /// A chamfer's (X3): no ball, the flat through the points of each face its leg from the
    /// other — `[on a, on b]`.
    flat: Option<[f64; 2]>,
}

/// How far a term reaches.  X2's rule reads the ball's centre, which Solvent has no word for;
/// X3 asks whether one stated by the point's own distances does as well.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Reach {
    /// X2's: the faces' meeting point nearest the ball's centre is a point of the edge.
    Ball,
    /// The faces' meeting point nearest `p` is a point of the edge: `distance(p, edge) <=
    /// distance(p, meeting)`, the edge `{ q | q coincident a; q coincident b }`.
    Edge,
    /// `p` lies over each face: its nearest point on each face's surface is on the face,
    /// `distance(p, a) <= distance(p, a.surface)`.
    Faces,
}

impl Term {
    fn new(body: &Body, faces: Vec<usize>, convex: bool, r: f64) -> Term {
        let sides: Vec<Surf> = faces.iter().map(|&k| {
            let s = body.leaves[k];
            if convex { s } else { s.flip() }
        }).collect();
        let offsets = sides.iter().map(|s| s.offset(r)).collect();
        Term { faces, sides, offsets, sign: if convex { 1.0 } else { -1.0 }, flat: None }
    }
    /// A chamfer's term between faces `a` and `b`: the points nearer both than the flat through
    /// the points of `a` its leg `legs[0]` from `b` and of `b` its leg `legs[1]` from `a` —
    /// `g_a / legs[1] + g_b / legs[0] < 1`, the flat itself where the faces are planes.
    fn flat(body: &Body, faces: [usize; 2], convex: bool, legs: [f64; 2]) -> Term {
        let mut t = Term::new(body, faces.to_vec(), convex, legs[0].min(legs[1]));
        t.offsets = None;
        t.flat = Some(legs);
        t
    }
    /// Whether the meeting of faces `k` and `l` at `x` is the body's edge: a step along each face
    /// away from the other is on the boundary, material on its side.
    fn edge_at(&self, body: &Body, x: V, k: usize, l: usize, size: f64, lifted: &[usize]) -> bool {
        let s = |p: V| body.s_with(p, lifted);
        // along the face, then a thousandth of that across it (a face is narrow by an acute
        // corner)
        let step = 1e-5 * size.max(1e-3);
        let across = step * 1e-3;
        [(k, l), (l, k)].iter().all(|&(a, b)| {
            let (na, nb) = (unit(self.sides[a].grad(x)), self.sides[b].grad(x));
            let along = sub(nb, scale(na, dot(nb, na)));
            if norm(along) < 1e-9 {
                return false;
            }
            let y = add(x, scale(unit(along), step));
            let inward = unit(body.leaves[self.faces[a]].grad(x));
            s(add(y, scale(inward, across))) > 0.0 && s(sub(y, scale(inward, across))) < 0.0
        })
    }
    /// The term at `p`, its ball centred at `c` on the centres' carriers `active`, reaches by
    /// `rule`.
    fn reaches_by(&self, body: &Body, p: V, c: V, active: &[usize], lifted: &[usize], rule: Reach)
        -> bool {
        let at = match rule {
            Reach::Ball if self.flat.is_none() => c,
            Reach::Faces => {
                return self.faces.iter().all(|&f| {
                    let leaf = body.leaves[f];
                    let Some(q) = leaf.foot(p) else { return false };
                    let n = scale(unit(leaf.grad(q)), 1e-8 * norm(sub(p, q)).max(1e-3));
                    let s = |x: V| body.s_with(x, lifted);
                    s(add(q, n)) > 0.0 && s(sub(q, n)) < 0.0
                });
            }
            _ => p,
        };
        let pair = if self.faces.len() == 2 { &[0, 1][..] } else { active };
        match *pair {
            [] | [_] => true,
            [k, l] => {
                let nearest = curve_nearest(at, self.sides[k], self.sides[l]).into_iter()
                    .min_by(|a, b| norm(sub(*a, at)).total_cmp(&norm(sub(*b, at))));
                nearest.is_some_and(|x| self.edge_at(body, x, k, l, norm(sub(at, x)), lifted))
            }
            _ => self.reaches(body, at, active, lifted),
        }
    }
    /// Whether the ball centred at `c`, on the centres' carriers `active`, is the ball of a point
    /// of the body's edge where those faces meet: at the point of their meeting nearest `c`, each
    /// face is a face of the body (a step along it away from the other is on the body's boundary,
    /// material on its side).  So carriers meeting where the body has no edge (a cylinder's far
    /// crossing, a cap a cut ate, two pieces' shared inner face, a wall's plane past the seam
    /// where it turns into a corner) make nothing, and a run ends where its edge does.  At a
    /// corner of three, the corner is on the boundary with material toward the ball.  `lifted`
    /// leaves read as no bound.
    fn reaches(&self, body: &Body, c: V, active: &[usize], lifted: &[usize]) -> bool {
        let s = |p: V| body.s_with(p, lifted);
        match *active {
            [] | [_] => true,
            [k, l] => {
                let nearest = curve_nearest(c, self.sides[k], self.sides[l]).into_iter()
                    .min_by(|a, b| norm(sub(*a, c)).total_cmp(&norm(sub(*b, c))));
                let Some(x) = nearest else { return false };
                // along the face, then a thousandth of that across it (a face is narrow by an
                // acute corner)
                let step = 1e-5 * norm(sub(c, x)).max(1e-3);
                let across = step * 1e-3;
                [(k, l), (l, k)].iter().all(|&(a, b)| {
                    let (na, nb) = (unit(self.sides[a].grad(x)), self.sides[b].grad(x));
                    let along = sub(nb, scale(na, dot(nb, na)));
                    if norm(along) < 1e-9 {
                        return false;
                    }
                    let y = add(x, scale(unit(along), step));
                    let inward = unit(body.leaves[self.faces[a]].grad(x));
                    s(add(y, scale(inward, across))) > 0.0 && s(sub(y, scale(inward, across))) < 0.0
                })
            }
            [k, l, m] => {
                let Some(x) = corner(self.sides[k], self.sides[l], self.sides[m]) else {
                    return false;
                };
                let toward = scale(unit(sub(c, x)), 1e-6 * norm(sub(c, x)));
                let side = |p: V| self.sign * s(p);
                s(x).abs() <= 1e-9 && side(add(x, toward)) > 0.0 && side(sub(x, toward)) < 0.0
            }
            _ => false,
        }
    }
}

/// A round as candidate terms, how far they reach (`lifted` end faces carried past, `clips`
/// their sides kept), and the body.
struct Round {
    body: Body,
    terms: Vec<Term>,
    r: f64,
    lifted: Vec<usize>,
    clips: Vec<Surf>,
    reach: Reach,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Candidate {
    Pairwise,
    PairwiseVertices,
    Joint,
}

/// Whether two leaves' carriers are parallel planes (no edge between them).
fn parallel(a: Surf, b: Surf) -> bool {
    matches!((a, b), (Surf::Plane { n: m, .. }, Surf::Plane { n, .. }) if norm(cross(m, n)) < 1e-12)
}

impl Round {
    /// `A round(r) B` over `body` by `candidate`: the edges where a face of `a` meets a face of
    /// `b` (both lists of leaves; the same list for the prefix form, every edge among them).
    fn new(body: Body, a: &[usize], b: &[usize], r: f64, candidate: Candidate) -> Round {
        let mut pairs: Vec<(usize, usize)> = Vec::new();
        for &k in a {
            for &l in b {
                let pair = (k.min(l), k.max(l));
                if k != l && !parallel(body.leaves[k], body.leaves[l]) && !pairs.contains(&pair) {
                    pairs.push(pair);
                }
            }
        }
        let named = |k: usize, l: usize| pairs.contains(&(k.min(l), k.max(l)));
        let mut terms = Vec::new();
        match candidate {
            Candidate::Pairwise | Candidate::PairwiseVertices => {
                for &(k, l) in &pairs {
                    terms.push(Term::new(&body, vec![k, l], body.convex(k, l), r));
                }
            }
            Candidate::Joint => {
                let mut faces: Vec<usize> = a.iter().chain(b).copied().collect();
                faces.sort();
                faces.dedup();
                let convex = body.convex(pairs[0].0, pairs[0].1);
                assert!(pairs.iter().all(|&(k, l)| body.convex(k, l) == convex), "one side");
                terms.push(Term::new(&body, faces, convex, r));
            }
        }
        if candidate == Candidate::PairwiseVertices {
            // three planes pairwise named, alike, meeting at a point of the body's boundary
            let leaves: std::collections::BTreeSet<usize> = pairs.iter().flat_map(|&(k, l)| [k, l])
                .collect();
            let leaves: Vec<usize> = leaves.into_iter().collect();
            for (i, &k) in leaves.iter().enumerate() {
                for (j, &l) in leaves.iter().enumerate().skip(i + 1) {
                    for &m in leaves.iter().skip(j + 1) {
                        if !(named(k, l) && named(l, m) && named(k, m)) {
                            continue;
                        }
                        let convex = body.convex(k, l);
                        if body.convex(l, m) != convex || body.convex(k, m) != convex {
                            continue;
                        }
                        let at = corner(body.leaves[k], body.leaves[l], body.leaves[m]);
                        if at.is_some_and(|x| body.s(x).abs() <= 1e-9) {
                            terms.push(Term::new(&body, vec![k, l, m], convex, r));
                        }
                    }
                }
            }
        }
        Round { body, terms, r, lifted: Vec::new(), clips: Vec::new(), reach: Reach::Ball }
    }

    /// `A chamfer(d) B`, its legs measured from the other face (X1's finding 4): one flat per
    /// named edge, and nothing at a corner.
    fn chamfer(body: Body, a: &[usize], b: &[usize], d: f64) -> Round {
        let mut round = Round::new(body, a, b, d, Candidate::Pairwise);
        round.terms = round.terms.iter().map(|t| {
            Term::flat(&round.body, [t.faces[0], t.faces[1]], t.sign > 0.0, [d, d])
        }).collect();
        round.reach = Reach::Edge;
        round
    }

    /// The same, reaching by `rule`.
    fn reaching(mut self, rule: Reach) -> Round {
        self.reach = rule;
        self
    }

    /// A run that ends at `ends` carried on past them along its carriers, and cut off by them.
    fn carried_past(mut self, ends: &[usize]) -> Round {
        self.lifted = ends.to_vec();
        self.clips = ends.iter().map(|&k| self.body.leaves[k]).collect();
        self
    }

    /// At `p`: the largest of the terms' 1-Lipschitz bounds (the round's material is where one is
    /// positive), and the largest of those whose ball reaches its faces.
    fn at(&self, p: V) -> (f64, f64) {
        let s = self.body.s(p);
        let clip = self.clips.iter().map(|c| c.g(p)).fold(f64::INFINITY, f64::min);
        let (mut upper, mut reached) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
        for t in &self.terms {
            let mut v = (t.sign * s).min(clip);
            for side in &t.sides {
                v = v.min(side.g(p));
            }
            if let Some([la, lb]) = t.flat {
                let (ga, gb) = (t.sides[0].g(p), t.sides[1].g(p));
                v = v.min((1.0 - ga / lb - gb / la) / (1.0 / lb + 1.0 / la));
                if v <= reached {
                    continue;
                }
                upper = upper.max(v);
                if t.reaches_by(&self.body, p, p, &[0, 1], &self.lifted, self.reach) {
                    reached = v;
                }
                continue;
            }
            // no ball on one face alone may cover the point: where the ball centre nearest `p` on
            // one face's offset is among the centres, `p` is within `r` of it, unless `p` stands
            // more than `2r` into that face's side (no round's material does)
            if let Some(hs) = &t.offsets {
                for (k, h) in hs.iter().enumerate() {
                    let Some(q) = h.foot(p) else { continue };
                    let inside = hs.iter().enumerate().filter(|&(j, _)| j != k)
                        .map(|(_, o)| o.g(q)).fold(f64::INFINITY, f64::min);
                    v = v.min((-inside / 2.0).max(t.sides[k].g(p) - 2.0 * self.r));
                }
            }
            // a term whose bound is under what is reached already cannot raise it, nor one under
            // the largest bound and too far out to count in a last cell
            if v <= reached || (v <= upper && v < -self.r / 16.0) {
                continue;
            }
            let near = t.offsets.as_ref().and_then(|hs| nearest(p, hs));
            let lip = match &near {
                Some((d, _, _)) => v.min(d - self.r),
                None => v,
            };
            upper = upper.max(lip);
            if lip > reached {
                let reach = match &near {
                    Some((_, c, active)) => {
                        t.reaches_by(&self.body, p, *c, active, &self.lifted, self.reach)
                    }
                    None => true,
                };
                if reach {
                    reached = lip;
                }
            }
        }
        (upper, reached)
    }

    /// Whether `p` is in the round's material (taken away where convex, added where concave).
    fn takes(&self, p: V) -> bool { self.at(p).1 > 0.0 }

    /// The volume of the round's material within the box `lo`–`hi`: cubes of side `r`, each cut
    /// down to `r / K`, a cube skipped where the bound says no term reaches it, counted whole
    /// where one covers it and its corners agree, and a last cell by its centre's bound.
    fn volume(&self, lo: V, hi: V) -> f64 {
        const K: u32 = 32;
        let side = self.r;
        let n = |k: usize| ((hi[k] - lo[k]) / side).ceil() as usize;
        let [nx, ny, nz] = [n(0), n(1), n(2)];
        // the cells side by side, summed in order: the same total as one walk over them
        gcs_core::par::indices(nx * ny * nz, |c| {
            let (i, j, k) = (c / (ny * nz), c / nz % ny, c % nz);
            let at = add(lo, scale([i as f64, j as f64, k as f64], side));
            self.cell(at, side, side / K as f64)
        })
        .into_iter()
        .fold(0.0, |total, v| total + v)
    }

    fn cell(&self, lo: V, side: f64, least: f64) -> f64 {
        let half = side / 2.0;
        let c = add(lo, [half; 3]);
        let reach = half * 3f64.sqrt();
        let (upper, reached) = self.at(c);
        if upper < -reach {
            return 0.0;
        }
        if reached > reach {
            let corners = (0..8).all(|m| {
                self.at(vertex(lo, side, m)).1 > 0.0
            });
            if corners {
                return side * side * side;
            }
        }
        if side <= least * 1.5 {
            // one point placed in the cell by a hash of where it is: its material's fraction in
            // expectation (a centre never falls in a cusp thinner than half a cell along a face
            // the cells line up with, which loses a fixed share of every round)
            let mut h = lo.iter().fold(0x9e37_79b9_7f4a_7c15u64, |h, x| {
                (h ^ x.to_bits()).wrapping_mul(0x100_0000_01b3).rotate_left(29)
            });
            let mut next = || {
                h = h.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                (h >> 11) as f64 / (1u64 << 53) as f64
            };
            let q = add(lo, [side * next(), side * next(), side * next()]);
            return if self.at(q).1 > 0.0 { side * side * side } else { 0.0 };
        }
        let mut total = 0.0;
        for m in 0..8 {
            total += self.cell(vertex(lo, half, m), half, least);
        }
        total
    }
}

/// Corner `m` (its bits x, y, z) of the cube of `side` from `lo`.
fn vertex(lo: V, side: f64, m: usize) -> V {
    add(lo, [(m & 1) as f64 * side, (m >> 1 & 1) as f64 * side, (m >> 2 & 1) as f64 * side])
}

// -- closed forms ------------------------------------------------------------------------------

/// The stack's Steiner volume of a box `l × w × h` rounded all over: the inset box grown by a
/// ball.
fn rounded_box([l, w, h]: [f64; 3], r: f64) -> f64 {
    let (a, b, c) = (l - 2.0 * r, w - 2.0 * r, h - 2.0 * r);
    a * b * c + 2.0 * r * (a * b + b * c + a * c) + PI * r * r * (a + b + c)
        + 4.0 * PI * r * r * r / 3.0
}

/// The stack's mitre: a convex polygon's prism with one cap's rim rounded alone loses `∫δ` along
/// its perimeter less `∫δ²` at each corner (`cots` the sum of `cot(θ/2)` over its angles).
fn mitred(perimeter: f64, cots: f64, r: f64) -> f64 {
    (1.0 - PI / 4.0) * r * r * perimeter - (5.0 / 3.0 - PI / 2.0) * r * r * r * cots
}

/// The stack's ring a ball of radius `r` fills where a cylinder of radius `big` stands on a
/// plane (Pappus).
fn root_ring(big: f64, r: f64) -> f64 {
    let moment = r * r * (big + r / 2.0) - PI * r * r / 4.0 * (big + r - 4.0 * r / (3.0 * PI));
    2.0 * PI * moment
}

/// The least positive root of `a t² + b t + c` (infinite where there is none).
fn first_root(a: f64, b: f64, c: f64) -> f64 {
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return f64::INFINITY;
    }
    [(-b - disc.sqrt()) / (2.0 * a), (-b + disc.sqrt()) / (2.0 * a)].into_iter()
        .filter(|&t| t > 1e-12).fold(f64::INFINITY, f64::min)
}

/// Where a ray from `o` along `w` first meets the cylinder of `radius` about the coordinate axis
/// `axis` through `at`.
fn ray_cylinder(o: V, w: V, at: V, axis: usize, radius: f64) -> f64 {
    let o = sub(o, at);
    let (i, j) = ((axis + 1) % 3, (axis + 2) % 3);
    let (a, b) = (w[i] * w[i] + w[j] * w[j], 2.0 * (o[i] * w[i] + o[j] * w[j]));
    first_root(a, b, o[i] * o[i] + o[j] * o[j] - radius * radius)
}

/// The stack's `rolled` (#85): a ball of radius `r` rolled round a closed spine `c(φ)`, by
/// quadrature over its sections square to the spine, swept with the spine's curvature.
fn rolled(c: &dyn Fn(f64) -> V, toward: [&dyn Fn(V) -> V; 2], hit: [&dyn Fn(V, V) -> f64; 2],
          r: f64) -> f64 {
    let (n_phi, n_psi) = (720, 96);
    let gauss: Vec<(f64, f64)> = (0..n_psi).map(|i| {
        let mut x = (PI * (i as f64 + 0.75) / (n_psi as f64 + 0.5)).cos();
        let mut dp = 0.0;
        for _ in 0..100 {
            let (mut p0, mut p1) = (1.0, x);
            for k in 2..=n_psi {
                let p2 = ((2 * k - 1) as f64 * x * p1 - (k - 1) as f64 * p0) / k as f64;
                p0 = p1;
                p1 = p2;
            }
            dp = n_psi as f64 * (x * p1 - p0) / (x * x - 1.0);
            let dx = p1 / dp;
            x -= dx;
            if dx.abs() < 1e-16 {
                break;
            }
        }
        (x, 2.0 / ((1.0 - x * x) * dp * dp))
    }).collect();
    let h = 1e-4;
    let mut total = 0.0;
    for k in 0..n_phi {
        let phi = 2.0 * PI * k as f64 / n_phi as f64;
        let p = c(phi);
        let (pa, pb) = (c(phi - h), c(phi + h));
        let d1 = scale(sub(pb, pa), 1.0 / (2.0 * h));
        let d2 = scale(sub(add(pb, pa), scale(p, 2.0)), 1.0 / (h * h));
        let speed = dot(d1, d1).sqrt();
        let t = scale(d1, 1.0 / speed);
        let kn = scale(sub(d2, scale(t, dot(d2, t))), 1.0 / (speed * speed));
        let (ea, eb) = (toward[0](p), toward[1](p));
        let span = dot(ea, eb).clamp(-1.0, 1.0).acos();
        let side = unit(sub(eb, scale(ea, dot(ea, eb))));
        let dir = |psi: f64| [0, 1, 2].map(|i| ea[i] * psi.cos() + side[i] * psi.sin());
        let switch = {
            let gap = |psi: f64| hit[0](p, dir(psi)) - hit[1](p, dir(psi));
            let (mut lo, mut hi) = (0.0, span);
            for _ in 0..80 {
                let m = 0.5 * (lo + hi);
                if (gap(m) < 0.0) == (gap(lo) < 0.0) { lo = m } else { hi = m }
            }
            0.5 * (lo + hi)
        };
        let mut area = 0.0;
        for (a, b) in [(0.0, switch), (switch, span)] {
            for &(x, w) in &gauss {
                let psi = a + 0.5 * (b - a) * (x + 1.0);
                let d = dir(psi);
                let reach = hit[0](p, d).min(hit[1](p, d));
                let q = dot(d, kn);
                let dv = (reach * reach - r * r) / 2.0 - q * (reach.powi(3) - r.powi(3)) / 3.0;
                area += w * 0.5 * (b - a) * dv;
            }
        }
        total += area * speed * 2.0 * PI / n_phi as f64;
    }
    total
}

/// The stack's tee crotch: the main pipe of radius `big` about x, a branch of radius `small`
/// about z standing on it, the ball of radius `r` rolled round where they meet.
pub(crate) fn tee_crotch(big: f64, small: f64, r: f64) -> f64 {
    let (rb, rs) = (big + r, small + r);
    rolled(
        &|phi: f64| {
            let (s, co) = phi.sin_cos();
            [rs * co, rs * s, (rb * rb - rs * rs * s * s).sqrt()]
        },
        [&|p: V| unit([0.0, -p[1], -p[2]]), &|p: V| unit([-p[0], -p[1], 0.0])],
        [&|o, w| ray_cylinder(o, w, [0.0; 3], 0, big),
            &|o, w| ray_cylinder(o, w, [0.0; 3], 2, small)],
        r,
    )
}

/// The stack's bore rim: a bore of radius `small` about z drilled into a solid pipe of radius
/// `big` about x, its rim rolled off.
pub(crate) fn bore_rim(big: f64, small: f64, r: f64) -> f64 {
    rolled(
        &|phi: f64| {
            let (s, co) = phi.sin_cos();
            let (rb, rs) = (big - r, small + r);
            [rs * co, rs * s, (rb * rb - rs * rs * s * s).sqrt()]
        },
        [&|p: V| unit([0.0, p[1], p[2]]), &|p: V| unit([-p[0], -p[1], 0.0])],
        [&|o, w| ray_cylinder(o, w, [0.0; 3], 0, big),
            &|o, w| ray_cylinder(o, w, [0.0; 3], 2, small)],
        r,
    )
}

/// The stack's root of a boss of radius `big` about `(cx, cy)` standing through the plane
/// `z = h0 − s x`, the whole way round.
pub(crate) fn sloped_root(big: f64, [cx, cy]: [f64; 2], [h0, s]: [f64; 2], r: f64) -> f64 {
    let n = unit([s, 0.0, 1.0]);
    let lift = r / n[2];
    rolled(
        &|phi: f64| {
            let (sn, co) = phi.sin_cos();
            let x = cx + (big + r) * co;
            [x, cy + (big + r) * sn, h0 - s * x + lift]
        },
        [&|_: V| scale(n, -1.0), &|p: V| unit([cx - p[0], cy - p[1], 0.0])],
        [&|o: V, w: V| {
            let d = dot(w, n);
            if d < 0.0 { -(dot(o, n) - h0 * n[2]) / d } else { f64::INFINITY }
        },
            &|o, w| ray_cylinder(o, w, [cx, cy, 0.0], 2, big)],
        r,
    )
}

// -- fixtures ----------------------------------------------------------------------------------

/// A box, every face a leaf: x low, x high, y low, y high, z low (the bottom), z high (the top).
fn block(size: V) -> Body {
    Body { leaves: boxed([0.0; 3], size), tree: and(0..6) }
}

/// The stack's triangle (#93, #99), its prism `h` deep: three sides, then its two caps.
fn prism(tri: [[f64; 2]; 3], h: f64) -> Body {
    let mut leaves = Vec::new();
    for k in 0..3 {
        let (a, b) = (tri[k], tri[(k + 1) % 3]);
        let n = [b[1] - a[1], a[0] - b[0], 0.0];
        leaves.push(plane(n, n[0] * a[0] + n[1] * a[1]));
    }
    leaves.push(plane([0.0, 0.0, -1.0], 0.0));
    leaves.push(plane([0.0, 0.0, 1.0], h));
    Body { leaves, tree: and(0..5) }
}

/// The triangle's perimeter, area and `Σ cot(θ/2)` over its angles.
fn triangle(tri: [[f64; 2]; 3]) -> (f64, f64, f64) {
    let side = |a: [f64; 2], b: [f64; 2]| (b[0] - a[0]).hypot(b[1] - a[1]);
    let perimeter = (0..3).map(|k| side(tri[k], tri[(k + 1) % 3])).sum();
    let area = 0.5 * ((tri[1][0] - tri[0][0]) * (tri[2][1] - tri[0][1])
        - (tri[2][0] - tri[0][0]) * (tri[1][1] - tri[0][1])).abs();
    let cots = (0..3).map(|k| {
        let (p, a, b) = (tri[k], tri[(k + 1) % 3], tri[(k + 2) % 3]);
        let (u, v) = ([a[0] - p[0], a[1] - p[1]], [b[0] - p[0], b[1] - p[1]]);
        let theta = (u[0] * v[1] - u[1] * v[0]).abs().atan2(u[0] * v[0] + u[1] * v[1]);
        1.0 / (theta / 2.0).tan()
    }).sum();
    (perimeter, area, cots)
}

/// The stack's tray (#93, #99): a 60 × 40 plate 10 thick under `z = 0`, a 30 × 20 pocket `d`
/// deep cut from it (the cutter from `−d` to 1).  Leaves 0–5 the plate's, 6–11 the pocket's,
/// each its complement's side: x low, x high, y low, y high, its floor, its cap.
fn tray(d: f64) -> Body {
    let mut leaves = boxed([0.0, 0.0, -10.0], [60.0, 40.0, 0.0]);
    leaves.extend(boxed([15.0, 10.0, -d], [45.0, 30.0, 1.0]).into_iter().map(Surf::flip));
    Body { leaves, tree: Node::And(vec![and(0..6), or(6..12)]) }
}

/// The stack's rounded boss (#90): a 30 × 20 outline about (30, 20) with corners of radius
/// `rr`, 20 tall, standing on the plate (leaves 0–5, its top 5).  The boss (leaves 6–29) is the
/// union of two crossed boxes and four corner cylinders, each with its foot and top; most of its
/// leaves are no face of the body (the crossed boxes' inner sides, the pieces' feet).
fn rounded_boss(rr: f64) -> Body {
    let (x0, x1, y0, y1) = (15.0, 45.0, 10.0, 30.0);
    let mut leaves = boxed([0.0, 0.0, -10.0], [60.0, 40.0, 0.0]);
    leaves.extend(boxed([x0 + rr, y0, 0.0], [x1 - rr, y1, 20.0]));
    leaves.extend(boxed([x0, y0 + rr, 0.0], [x1, y1 - rr, 20.0]));
    let mut pieces = vec![and(6..12), and(12..18)];
    let centres = [(x0 + rr, y0 + rr), (x1 - rr, y0 + rr), (x1 - rr, y1 - rr), (x0 + rr, y1 - rr)];
    for (cx, cy) in centres {
        let k = leaves.len();
        leaves.push(Surf::Cyl { a: [cx, cy, 0.0], u: [0.0, 0.0, 1.0], r: rr, inside: true });
        leaves.push(plane([0.0, 0.0, -1.0], 0.0));
        leaves.push(plane([0.0, 0.0, 1.0], 20.0));
        pieces.push(and(k..k + 3));
    }
    Body { leaves, tree: Node::Or(vec![and(0..6), Node::Or(pieces)]) }
}

/// Whether a measured volume is the closed form's, to `tol` of it.
fn agrees(got: f64, want: f64, tol: f64) -> bool { (got - want).abs() <= tol * want.abs() }

/// The three candidates' volumes for one round.
fn candidates(make: &(dyn Fn() -> Body + Sync), a: &[usize], b: &[usize], r: f64, lo: V, hi: V)
    -> [f64; 3] {
    std::thread::scope(|scope| {
        [Candidate::Pairwise, Candidate::PairwiseVertices, Candidate::Joint]
            .map(|c| scope.spawn(move || Round::new(make(), a, b, r, c).volume(lo, hi)))
            .map(|h| h.join().unwrap())
    })
}

fn report(name: &str, want: f64, got: [f64; 3]) {
    let rel = |v: f64| (v - want) / want;
    eprintln!("{name}: closed form {want:.6}; (a) {:+.2e}, (a + v) {:+.2e}, (b) {:+.2e}",
        rel(got[0]), rel(got[1]), rel(got[2]));
}

const TOL: f64 = 1e-3;

// -- the cases ---------------------------------------------------------------------------------

/// #183's hand calculation: the point `(−0.3, −0.3, −0.3) r` from a square corner is `0.99 r`
/// from each edge's nearest ball centre, so every pairwise ball covers it, and `1.21 r` from the
/// corner ball's, so the corner's ball does not.
#[test]
fn a_corner_point_is_kept_by_pairs_and_taken_by_the_corner() {
    let r = 1.0;
    let all: Vec<usize> = (0..6).collect();
    let p = [0.3 * r; 3];
    let pairwise = Round::new(block([10.0; 3]), &all, &all, r, Candidate::Pairwise);
    let corner = Round::new(block([10.0; 3]), &all, &all, r, Candidate::PairwiseVertices);
    assert!(!pairwise.takes(p) && corner.takes(p));
    // and the distances are as said
    let pair = nearest(p, &[plane([-1.0, 0.0, 0.0], -r), plane([0.0, -1.0, 0.0], -r)]).unwrap().0;
    let lows: Vec<Surf> = boxed([r; 3], [9.0; 3]).into_iter().step_by(2).collect();
    let ball = nearest(p, &lows).unwrap().0;
    assert!((pair - 0.99 * r).abs() < 0.001 && (ball - 1.212 * r).abs() < 0.001, "{pair} {ball}");
}

/// Steiner (#93): a block rounded all over is the inset block grown by the ball.  Three named
/// edges meet at every corner: the pairs alone leave material there, the corner terms take it.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 15 s: three candidates, a block")]
fn a_block_rounded_all_over_needs_its_corners() {
    let (size, r) = ([60.0, 40.0, 10.0], 2.0);
    let all: Vec<usize> = (0..6).collect();
    let want = size[0] * size[1] * size[2] - rounded_box(size, r);
    let got = candidates(&|| block(size), &all, &all, r, [0.0; 3], size);
    report("block rounded all over", want, got);
    assert!(agrees(got[1], want, TOL) && agrees(got[2], want, TOL));
    assert!(got[0] < want * (1.0 - 10.0 * TOL), "the pairs alone leave the corners");
}

/// A mitre (#99): the block's top rim rounded alone, its upright edges sharp.  Two named edges
/// meet at each top corner and the third is sharp: the pairs cross, as OCCT's do; the joint
/// opening rounds the upright edges, which nothing named.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 8 s: three candidates, a mitred rim")]
fn a_blocks_top_rim_is_mitred() {
    let (size, r) = ([60.0, 40.0, 10.0], 2.0);
    let all: Vec<usize> = (0..6).collect();
    let want = mitred(2.0 * (size[0] + size[1]), 4.0, r);
    let got = candidates(&|| block(size), &[5], &all, r, [0.0; 3], size);
    report("block's top rim", want, got);
    assert!(agrees(got[0], want, TOL) && agrees(got[1], want, TOL));
    assert!(got[2] > want * (1.0 + 10.0 * TOL), "the joint opening rounds the upright edges");
}

/// Steiner at slanted corners (#93), and mitres at an obtuse and two acute corners (#99): the
/// stack's triangle, rounded all over and by its cap alone.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 12 s: six candidates, a triangle")]
fn a_triangular_prism_rounds_and_mitres_its_slanted_corners() {
    let (tri, h, r) = ([[10.0, 5.0], [50.0, 5.0], [5.0, 20.0]], 10.0, 2.0);
    let (perimeter, area, cots) = triangle(tri);
    let all: Vec<usize> = (0..5).collect();
    let (lo, hi) = ([5.0, 5.0, 0.0], [50.0, 20.0, h]);
    // Steiner over the prism inset by `r`
    let inradius = 2.0 * area / perimeter;
    let s = (inradius - r) / inradius;
    let (a2, p2, h2) = (s * s * area, s * perimeter, h - 2.0 * r);
    let rounded = a2 * h2 + (2.0 * a2 + p2 * h2) * r + PI * r * r / 2.0 * p2 + PI * r * r * h2
        + 4.0 * PI * r * r * r / 3.0;
    let want = area * h - rounded;
    let got = candidates(&|| prism(tri, h), &all, &all, r, lo, hi);
    report("triangular prism rounded all over", want, got);
    assert!(agrees(got[1], want, TOL) && agrees(got[2], want, TOL));
    assert!(got[0] < want * (1.0 - 10.0 * TOL));
    let want = mitred(perimeter, cots, 1.5);
    let got = candidates(&|| prism(tri, h), &[4], &all, 1.5, lo, hi);
    report("triangular prism's cap", want, got);
    assert!(agrees(got[0], want, TOL) && agrees(got[1], want, TOL));
    assert!(got[2] > want * (1.0 + 10.0 * TOL));
}

/// Concave, the same statements with the sides turned round (#93, #99): a pocket rounded inside
/// fills a ball's patch at each floor corner, and its upright corners' runs stop flush in the
/// plate's top; its floor rounded alone is mitred at the corners.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 12 s: six candidates, a pocket")]
fn a_pocket_is_filled_as_a_block_is_rounded() {
    let (r, [l, w], d) = (1.5, [30.0, 20.0], 6.0);
    // the pocket's faces: its walls and floor (its cutter's cap, above the plate, is none)
    let pocket: Vec<usize> = (6..11).collect();
    let (lo, hi) = ([14.0, 9.0, -7.0], [46.0, 31.0, 1.0]);
    let run = (1.0 - PI / 4.0) * r * r;
    let want = 4.0 * run * (d - r) + run * (2.0 * (l - 2.0 * r) + 2.0 * (w - 2.0 * r))
        + 4.0 * (r * r * r - PI * r * r * r / 6.0);
    let got = candidates(&|| tray(d), &pocket, &pocket, r, lo, hi);
    report("pocket rounded inside", want, got);
    assert!(agrees(got[1], want, TOL), "the corner terms fill the floor's corners");
    assert!(got[0] < want * (1.0 - 10.0 * TOL));
    let want = mitred(2.0 * (l + w), 4.0, r);
    let got = candidates(&|| tray(d), &[10], &pocket, r, lo, hi);
    report("pocket's floor", want, got);
    assert!(agrees(got[0], want, TOL) && agrees(got[1], want, TOL));
    assert!(got[2] > want * (1.0 + 10.0 * TOL));
}

/// A chain of tangent edges (#90): a rounded boss's foot is one run, its straight pieces and its
/// corners' quarter rings meeting where the boss's walls meet tangent.  Each pair's ball acts
/// only where it touches the faces themselves, so no run carries on past a seam.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 14 s: a tangent chain's foot")]
fn a_rounded_bosss_foot_is_seamless() {
    let (rr, r, size) = (4.0, 2.0, [30.0, 20.0]);
    let boss: Vec<usize> = (6..30).collect();
    let runs = 2.0 * (size[0] - 2.0 * rr) + 2.0 * (size[1] - 2.0 * rr);
    let want = (1.0 - PI / 4.0) * r * r * runs + root_ring(rr, r);
    let round = Round::new(rounded_boss(rr), &boss, &[5], r, Candidate::Pairwise);
    let got = round.volume([12.0, 7.0, -1.0], [48.0, 33.0, 3.0]);
    eprintln!("rounded boss's foot: closed form {want:.6}; (a) {:+.2e}", (got - want) / want);
    assert!(agrees(got, want, TOL));
}

/// Runs that end (#110): a bore's rim and a tee's crotch, each halved by a plane through the
/// pipes' axes, and a boss's root on a slope halved by the wedge's near face.  Convex, the half
/// rim runs out into the void; concave, the crotch and the root run out onto the face, and stop
/// there.  By symmetry the ball's contacts leave the faces where the plane cuts them, so each is
/// exactly half the whole round's material.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 24 s: five rounds that end")]
fn runs_that_end_on_a_face_stop_there() {
    // a solid pipe of radius 10 about x, from x = 0 (the half) to 30; a bore of radius 6 about z
    // from z = 0 up: leaves pipe, its ends, the bore (its side, its floor, its cap)
    let pipe = Surf::Cyl { a: [0.0; 3], u: [1.0, 0.0, 0.0], r: 10.0, inside: true };
    let bore = Surf::Cyl { a: [0.0; 3], u: [0.0, 0.0, 1.0], r: 6.0, inside: false };
    let drilled = || Body {
        leaves: vec![pipe, plane([-1.0, 0.0, 0.0], 0.0), plane([1.0, 0.0, 0.0], 30.0), bore,
            plane([0.0, 0.0, 1.0], 0.0), plane([0.0, 0.0, -1.0], -20.0)],
        tree: Node::And(vec![and(0..3), or(3..6)]),
    };
    let want = bore_rim(10.0, 6.0, 1.0) / 2.0;
    let got = Round::new(drilled(), &[0], &[3], 1.0, Candidate::Pairwise)
        .volume([-4.0, -10.0, 0.0], [10.0, 10.0, 11.0]);
    eprintln!("half a bore's rim: closed form {want:.6}; (a) {:+.2e}", (got - want) / want);
    assert!(agrees(got, want, TOL));
    // a main pipe of radius 10 about x and a branch of radius 6 about z from z = 0 to 25, the
    // whole from x = 0: leaves main, its ends, the branch, its foot and top, the half
    let stem = Surf::Cyl { a: [0.0; 3], u: [0.0, 0.0, 1.0], r: 6.0, inside: true };
    let tee = || Body {
        leaves: vec![pipe, plane([-1.0, 0.0, 0.0], 30.0), plane([1.0, 0.0, 0.0], 30.0), stem,
            plane([0.0, 0.0, -1.0], 0.0), plane([0.0, 0.0, 1.0], 25.0),
            plane([-1.0, 0.0, 0.0], 0.0)],
        tree: Node::And(vec![Node::Or(vec![and(0..3), and(3..6)]), Node::Leaf(6)]),
    };
    let want = tee_crotch(10.0, 6.0, 2.0) / 2.0;
    let got = Round::new(tee(), &[0], &[3], 2.0, Candidate::Pairwise)
        .volume([-4.0, -10.0, 4.0], [10.0, 10.0, 14.0]);
    eprintln!("half a tee's crotch: closed form {want:.6}; (a) {:+.2e}", (got - want) / want);
    assert!(agrees(got, want, TOL));
    // a wedge 40 long, 24 deep from its near face y = 0, its top sloping from 20 high to 10, and a
    // boss of radius 6 about (20, cy) from the floor to 30: leaves the wedge's ends, near and far
    // faces, floor and slope, then the boss, its foot and its top
    let wedge = |cy: f64| Body {
        leaves: vec![plane([-1.0, 0.0, 0.0], 0.0), plane([1.0, 0.0, 0.0], 40.0),
            plane([0.0, -1.0, 0.0], 0.0), plane([0.0, 1.0, 0.0], 24.0),
            plane([0.0, 0.0, -1.0], 0.0), plane([0.25, 0.0, 1.0], 20.0),
            Surf::Cyl { a: [20.0, cy, 0.0], u: [0.0, 0.0, 1.0], r: 6.0, inside: true },
            plane([0.0, 0.0, -1.0], 0.0), plane([0.0, 0.0, 1.0], 30.0)],
        tree: Node::Or(vec![and(0..6), and(6..9)]),
    };
    let want = sloped_root(6.0, [20.0, 0.0], [20.0, 0.25], 2.0) / 2.0;
    let (lo, hi) = ([10.0, -8.0, 9.0], [30.0, 14.0, 21.0]);
    let got = Round::new(wedge(0.0), &[6], &[5], 2.0, Candidate::Pairwise).volume(lo, hi);
    eprintln!("half a boss's root on a slope: closed form {want:.6}; (a) {:+.2e}",
        (got - want) / want);
    assert!(agrees(got, want, TOL));
    // the boss standing 3 in from the near face (`x0/runout.sv`): the root ends where the face cuts
    // it obliquely.  Ended with its edge, the run stops square to it and hangs past the face beside
    // the overhanging boss; carried on past and cut off by the face, it stops flush with the face
    // (the stack's, and OCCT's)
    let square = Round::new(wedge(3.0), &[6], &[5], 2.0, Candidate::Pairwise);
    let past = square.volume(lo, [hi[0], 0.0, hi[2]]);
    let within = square.volume([lo[0], 0.0, lo[2]], hi);
    let flush = Round::new(wedge(3.0), &[6], &[5], 2.0, Candidate::Pairwise).carried_past(&[2])
        .volume(lo, hi);
    eprintln!("an oblique run-out: cut by the face {flush:.6}; ended square {:.6}, {past:.6} of it \
               past the face and {:.6} short of the cut within it", past + within, flush - within);
    assert!(past > 10.0 * TOL * flush, "ended square, the round hangs past the face");
    assert!(agrees(within, flush, TOL), "and short of it, it is the face's cut");
    // where the face cuts it square, the two are one
    let halved = Round::new(wedge(0.0), &[6], &[5], 2.0, Candidate::Pairwise).carried_past(&[2])
        .volume(lo, hi);
    assert!(agrees(halved, sloped_root(6.0, [20.0, 0.0], [20.0, 0.25], 2.0) / 2.0, TOL));
}

/// A round wider than a face holds (#183 X0, finding 9): a plate 3 thick, rounded all over by
/// 2.  The pairs leave a ridge where the top's and the bottom's rounds cross, which is no full
/// round; the joint opening leaves nothing, since no ball of 2 fits in 3.
#[test]
fn a_round_wider_than_its_face_is_no_full_round() {
    let (size, r) = ([20.0, 20.0, 3.0], 2.0);
    let all: Vec<usize> = (0..6).collect();
    let (pairs, joint) = (Round::new(block(size), &all, &all, r, Candidate::PairwiseVertices),
        Round::new(block(size), &all, &all, r, Candidate::Joint));
    // on the front face's middle line: taken by both; half a radius in: kept by the pairs only
    let (edge, ridge) = ([10.0, 0.01, 1.5], [10.0, 0.5, 1.5]);
    assert!(pairs.takes(edge) && joint.takes(edge));
    assert!(!pairs.takes(ridge) && joint.takes(ridge));
    // the plate's middle: the joint takes it all
    assert!(!pairs.takes([10.0, 10.0, 1.5]) && joint.takes([10.0, 10.0, 1.5]));
}

// -- X3: the definition said in the point's own distances --------------------------------------

/// One of X2's cases: the body, the round's two face lists, its radius, the box its material is
/// in, its closed form, and where a run is carried past and cut off.
struct Case {
    name: &'static str,
    body: fn() -> Body,
    a: Vec<usize>,
    b: Vec<usize>,
    r: f64,
    lo: V,
    hi: V,
    want: f64,
}

fn block_case() -> Body { block([60.0, 40.0, 10.0]) }
fn prism_case() -> Body { prism([[10.0, 5.0], [50.0, 5.0], [5.0, 20.0]], 10.0) }
fn tray_case() -> Body { tray(6.0) }
fn boss_case() -> Body { rounded_boss(4.0) }

/// The solid pipe drilled from z = 0 up, halved at x = 0 (X2's).
fn drilled() -> Body {
    let pipe = Surf::Cyl { a: [0.0; 3], u: [1.0, 0.0, 0.0], r: 10.0, inside: true };
    let bore = Surf::Cyl { a: [0.0; 3], u: [0.0, 0.0, 1.0], r: 6.0, inside: false };
    Body {
        leaves: vec![pipe, plane([-1.0, 0.0, 0.0], 0.0), plane([1.0, 0.0, 0.0], 30.0), bore,
            plane([0.0, 0.0, 1.0], 0.0), plane([0.0, 0.0, -1.0], -20.0)],
        tree: Node::And(vec![and(0..3), or(3..6)]),
    }
}

/// The tee halved at x = 0 (X2's).
fn tee() -> Body {
    let pipe = Surf::Cyl { a: [0.0; 3], u: [1.0, 0.0, 0.0], r: 10.0, inside: true };
    let stem = Surf::Cyl { a: [0.0; 3], u: [0.0, 0.0, 1.0], r: 6.0, inside: true };
    Body {
        leaves: vec![pipe, plane([-1.0, 0.0, 0.0], 30.0), plane([1.0, 0.0, 0.0], 30.0), stem,
            plane([0.0, 0.0, -1.0], 0.0), plane([0.0, 0.0, 1.0], 25.0),
            plane([-1.0, 0.0, 0.0], 0.0)],
        tree: Node::And(vec![Node::Or(vec![and(0..3), and(3..6)]), Node::Leaf(6)]),
    }
}

/// The wedge and its boss about (20, `cy`) (X2's).
fn wedge(cy: f64) -> Body {
    Body {
        leaves: vec![plane([-1.0, 0.0, 0.0], 0.0), plane([1.0, 0.0, 0.0], 40.0),
            plane([0.0, -1.0, 0.0], 0.0), plane([0.0, 1.0, 0.0], 24.0),
            plane([0.0, 0.0, -1.0], 0.0), plane([0.25, 0.0, 1.0], 20.0),
            Surf::Cyl { a: [20.0, cy, 0.0], u: [0.0, 0.0, 1.0], r: 6.0, inside: true },
            plane([0.0, 0.0, -1.0], 0.0), plane([0.0, 0.0, 1.0], 30.0)],
        tree: Node::Or(vec![and(0..6), and(6..9)]),
    }
}

/// X2's cases with a closed form, each by (a + v).
fn x2_cases() -> Vec<Case> {
    let (tri, h) = ([[10.0, 5.0], [50.0, 5.0], [5.0, 20.0]], 10.0);
    let (perimeter, area, cots) = triangle(tri);
    let inradius = 2.0 * area / perimeter;
    let steiner = |r: f64| {
        let s = (inradius - r) / inradius;
        let (a2, p2, h2) = (s * s * area, s * perimeter, h - 2.0 * r);
        area * h - (a2 * h2 + (2.0 * a2 + p2 * h2) * r + PI * r * r / 2.0 * p2 + PI * r * r * h2
            + 4.0 * PI * r * r * r / 3.0)
    };
    let (pr, run) = (1.5, (1.0 - PI / 4.0) * 1.5 * 1.5);
    let size = [60.0, 40.0, 10.0];
    vec![
        Case { name: "block rounded all over", body: block_case, a: (0..6).collect(),
            b: (0..6).collect(), r: 2.0, lo: [0.0; 3], hi: size,
            want: 24000.0 - rounded_box(size, 2.0) },
        Case { name: "block's top rim", body: block_case, a: vec![5], b: (0..6).collect(), r: 2.0,
            lo: [0.0; 3], hi: size, want: mitred(200.0, 4.0, 2.0) },
        Case { name: "triangular prism all over", body: prism_case, a: (0..5).collect(),
            b: (0..5).collect(), r: 2.0, lo: [5.0, 5.0, 0.0], hi: [50.0, 20.0, h],
            want: steiner(2.0) },
        Case { name: "triangular prism's cap", body: prism_case, a: vec![4], b: (0..5).collect(),
            r: 1.5, lo: [5.0, 5.0, 0.0], hi: [50.0, 20.0, h], want: mitred(perimeter, cots, 1.5) },
        Case { name: "pocket rounded inside", body: tray_case, a: (6..11).collect(),
            b: (6..11).collect(), r: pr, lo: [14.0, 9.0, -7.0], hi: [46.0, 31.0, 1.0],
            want: 4.0 * run * (6.0 - pr) + run * (2.0 * (30.0 - 2.0 * pr) + 2.0 * (20.0 - 2.0 * pr))
                + 4.0 * (pr * pr * pr - PI * pr * pr * pr / 6.0) },
        Case { name: "pocket's floor", body: tray_case, a: vec![10], b: (6..11).collect(), r: pr,
            lo: [14.0, 9.0, -7.0], hi: [46.0, 31.0, 1.0], want: mitred(100.0, 4.0, pr) },
        Case { name: "rounded boss's foot", body: boss_case, a: (6..30).collect(), b: vec![5],
            r: 2.0, lo: [12.0, 7.0, -1.0], hi: [48.0, 33.0, 3.0],
            want: (1.0 - PI / 4.0) * 4.0 * (2.0 * 22.0 + 2.0 * 12.0) + root_ring(4.0, 2.0) },
        Case { name: "half a bore's rim", body: drilled, a: vec![0], b: vec![3], r: 1.0,
            lo: [-4.0, -10.0, 0.0], hi: [10.0, 10.0, 11.0], want: bore_rim(10.0, 6.0, 1.0) / 2.0 },
        Case { name: "half a tee's crotch", body: tee, a: vec![0], b: vec![3], r: 2.0,
            lo: [-4.0, -10.0, 4.0], hi: [10.0, 10.0, 14.0],
            want: tee_crotch(10.0, 6.0, 2.0) / 2.0 },
        Case { name: "half a boss's root on a slope", body: || wedge(0.0), a: vec![6], b: vec![5],
            r: 2.0, lo: [10.0, -8.0, 9.0], hi: [30.0, 14.0, 21.0],
            want: sloped_root(6.0, [20.0, 0.0], [20.0, 0.25], 2.0) / 2.0 },
    ]
}

/// X2's reach reads the ball's centre, which Solvent cannot say.  Stated by the point's own
/// distances instead — its nearest point of the faces' meeting on the edge (`Reach::Edge`), or
/// its nearest point on each face's surface on the face (`Reach::Faces`) — each against X2's
/// closed forms, and the oblique run-out against the face's cut.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 50 s: X2's cases by two more rules")]
fn the_reach_said_in_the_points_own_distances() {
    let cases = x2_cases();
    let got: Vec<[f64; 2]> = std::thread::scope(|scope| {
        let handles: Vec<_> = cases.iter().map(|c| scope.spawn(move || {
            [Reach::Edge, Reach::Faces].map(|rule| {
                Round::new((c.body)(), &c.a, &c.b, c.r, Candidate::PairwiseVertices).reaching(rule)
                    .volume(c.lo, c.hi)
            })
        })).collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });
    let mut off = [Vec::new(), Vec::new()];
    for (c, g) in cases.iter().zip(&got) {
        let rel = g.map(|v| (v - c.want) / c.want);
        eprintln!("{}: Edge {:+.2e}, Faces {:+.2e}", c.name, rel[0], rel[1]);
        for k in 0..2 {
            if !agrees(g[k], c.want, TOL) {
                off[k].push(c.name);
            }
        }
    }
    // the oblique run-out, against the face's cut
    let (lo, hi) = ([10.0, -8.0, 9.0], [30.0, 14.0, 21.0]);
    let flush = Round::new(wedge(3.0), &[6], &[5], 2.0, Candidate::Pairwise).carried_past(&[2])
        .volume(lo, hi);
    let by = [Reach::Edge, Reach::Faces].map(|rule| {
        Round::new(wedge(3.0), &[6], &[5], 2.0, Candidate::Pairwise).reaching(rule).volume(lo, hi)
    });
    // the edge's own end, then cut off by the face it ends on (that face's side kept, nothing
    // carried past)
    let mut clipped = Round::new(wedge(3.0), &[6], &[5], 2.0, Candidate::Pairwise)
        .reaching(Reach::Edge);
    clipped.clips = vec![clipped.body.leaves[2]];
    let clipped = clipped.volume(lo, hi);
    let rel = |v: f64| (v - flush) / flush;
    eprintln!("oblique run-out: cut by the face {flush:.6}; Edge {:+.2e}, Faces {:+.2e}, Edge cut \
               by the face {:+.2e}", rel(by[0]), rel(by[1]), rel(clipped));
    eprintln!("off by more than {TOL}: Edge {:?}, Faces {:?}", off[0], off[1]);
    assert!(off[0].is_empty(), "the point's nearest point of the edge does as well as X2's");
    assert!(off[1].contains(&"half a tee's crotch"), "lying over both faces loses a sliver");
    assert!(by[0] > flush * (1.0 + 10.0 * TOL), "ended with its edge, the run hangs past the face");
    assert!(agrees(clipped, flush, TOL), "cut by the face it ends on, it is the face's cut");
}

/// `A chamfer(d) B` as a bound on a sum of two distances (X3): the points of the wedge nearer
/// both faces than `d` in all, `distance(p, a) + distance(p, b) < d` — through the points of each
/// face `d` from the other, the legs X1 recommends.  Between planes it is the flat; about an
/// axis (a boss's foot) the cone.  A block chamfered all over by inclusion and exclusion (three
/// flats meet at each corner, nothing more), its top rim alone mitred, and a boss's foot.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 20 s: three chamfers")]
fn a_chamfer_is_a_sum_of_two_distances() {
    let (size, d) = ([60.0, 40.0, 10.0], 2.0);
    let all: Vec<usize> = (0..6).collect();
    let boss = || Body {
        leaves: {
            let mut l = boxed([0.0, 0.0, -10.0], [60.0, 40.0, 0.0]);
            l.push(Surf::Cyl { a: [30.0, 20.0, 0.0], u: [0.0, 0.0, 1.0], r: 8.0, inside: true });
            l.push(plane([0.0, 0.0, -1.0], 0.0));
            l.push(plane([0.0, 0.0, 1.0], 20.0));
            l
        },
        tree: Node::Or(vec![and(0..6), and(6..9)]),
    };
    let cases: Vec<(&str, f64, f64)> = std::thread::scope(|scope| {
        let all = &all;
        let jobs = vec![
            scope.spawn(move || ("block chamfered all over",
                Round::chamfer(block(size), all, all, d).volume([0.0; 3], size),
                2.0 * d * d * (size[0] + size[1] + size[2]) - 6.0 * d * d * d)),
            scope.spawn(move || ("block's top rim chamfered",
                Round::chamfer(block(size), &[5], all, d).volume([0.0; 3], size),
                2.0 * (size[0] + size[1]) * d * d / 2.0 - 4.0 * d * d * d / 3.0)),
            scope.spawn(move || ("boss's foot chamfered",
                Round::chamfer(boss(), &[6], &[5], d).volume([20.0, 10.0, -1.0], [40.0, 30.0, 3.0]),
                2.0 * PI * (8.0 + d / 3.0) * d * d / 2.0)),
        ];
        jobs.into_iter().map(|h| h.join().unwrap()).collect()
    });
    for (name, got, want) in cases {
        eprintln!("{name}: closed form {want:.6}, {:+.2e}", (got - want) / want);
        assert!(agrees(got, want, TOL), "{name}");
    }
}
