//! **Diagnosis words for analysis** (#510 P7): what an analysis cannot answer, said in the
//! solver's own terms, the way the diagnosis names an under-constrained sketch's freedom.
//!
//! * **A rigid-body motion the supports leave** (`freedoms`), named from the supports alone: a
//!   slide along a direction, a turn about an axis through a point, a screw.  Never from a
//!   factor: P1's Cholesky refuses only a pivot lost to rounding and P5's interior a least pivot
//!   under `interior::SINGULAR`, and no pivot ratio tells a free body from a slender held one.
//! * **Unbounded stress at an edge** (`reentrant`): a sharp edge where the material's angle
//!   passes a half turn.  Elastic stress there grows without bound toward the edge, as
//!   `r^(λ−1)` with λ Williams' least exponent (`williams`), so a stress requirement on it has
//!   no answer for a finer mesh to converge to: it is said, never given a mesh's number.
//! * **A degenerate modal pair** (`repeated`): eigenvalues the iteration found more than once,
//!   which have no derivative there, and the ring that makes them a pair (`Repeated::wave`).
//! * **A requirement out of reach** (`reach`): a row whose value does not move with the numbers
//!   that drive it, or stands at its extreme short of what it states.
//!
//! Each reads geometry or numbers already in hand — supports, a body's boundary, the pairs a
//! solve returned, a row's value and gradient — and no mesh.

use crate::eigen::{Pairs, GAP};
use crate::json::fmt_g;
use crate::linalg::{dot as vdot, min_norm_solve, orthonormalize, rank_and_nullspace, Mat};
use crate::model::{Applied, EntKind, EntRef, Hold, Output, Scenario, Sketch};
use crate::solid::faces::BodyFaces;
use crate::space::{add, cross, dot, norm, scale, sub};
use crate::sparse::Sym;
use std::collections::BTreeMap;
use std::f64::consts::PI;
use std::sync::Arc;

type V = [f64; 3];

/// Below this, relative to the largest singular value of the supports' rows, a twist is left
/// free.
const RIGID: f64 = 1e-9;
/// A unit direction within this of a world axis is named by it, and a basis of freedoms prefers
/// the axes it holds within this.
const SNAP: f64 = 1e-9;
/// A material angle within this of a half turn is no corner of the stress's.
const STRAIGHT: f64 = 1e-6;
/// Eigenvalues below this fraction of the largest are a rigid-body motion's, which `freedoms`
/// names: no pair of theirs is said to be degenerate.
const AT_REST: f64 = 1e-8;
/// A row is solved within this of what it states, relative to the larger of the two: what
/// "solved" means for every row (`docs/contract/solver.md`).
const SOLVED: f64 = 1e-8;
/// A row's first-order change over the numbers that drive it, each moved by its own scale,
/// below this fraction of its value: it stands at its extreme.
const STATIONARY: f64 = 1e-6;

/* -- the motions the supports leave --------------------------------------------------------- */

/// What one support holds of a body: each point of `at` held along `along` (every direction
/// where `None`), and with `turn` the body's turn held as well (`fixed`, a clamp; `pinned` holds
/// the points alone).
#[derive(Clone, Debug, PartialEq)]
pub struct Held {
    pub at: Vec<V>,
    pub along: Option<V>,
    pub turn: bool,
}

/// A rigid-body motion the supports leave free.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Freedom {
    /// Along a unit direction.
    Slide { along: V },
    /// About the axis through `through` along the unit direction `about`, advancing `pitch` along
    /// it a radian: a turn where `pitch` is zero, else a screw.  `through` is the axis's point
    /// nearest the supports' middle.
    Turn { about: V, through: V, pitch: f64 },
}

/// A number as a report reads it: six figures, and a hair of rounding (below `SNAP` of `size`)
/// none.
fn figure(v: f64, size: f64) -> String {
    fmt_g(if v.abs() <= SNAP * size.max(1.0) { 0.0 } else { v }, 6)
}

/// A unit direction by its world axis's name where it is one, else by its components.
fn direction_text(d: V, dim: usize) -> String {
    for (k, name) in ["x", "y", "z"].iter().enumerate() {
        let mut e = [0.0; 3];
        e[k] = 1.0;
        if norm(sub(d, e)) <= SNAP || norm(add(d, e)) <= SNAP {
            return name.to_string();
        }
    }
    let parts: Vec<String> = d[..dim].iter().map(|v| figure(*v, 1.0)).collect();
    format!("({})", parts.join(", "))
}

fn point_text(p: V, dim: usize) -> String {
    let size = p.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let parts: Vec<String> = p[..dim].iter().map(|v| figure(*v, size)).collect();
    format!("({})", parts.join(", "))
}

impl Freedom {
    /// What the body is free to do: `slide along x`, `turn about z through (0, 0, 0)` — in the
    /// plane, `turn about (30, 0)`.
    pub fn text(&self, dim: usize) -> String {
        match *self {
            Freedom::Slide { along } => format!("slide along {}", direction_text(along, dim)),
            Freedom::Turn { through, .. } if dim == 2 => {
                format!("turn about {}", point_text(through, 2))
            }
            Freedom::Turn { about, through, pitch } => {
                let axis =
                    format!("{} through {}", direction_text(about, 3), point_text(through, 3));
                if pitch == 0.0 {
                    format!("turn about {axis}")
                } else {
                    format!("screw about {axis}, advancing {} a radian", figure(pitch, 1.0))
                }
            }
        }
    }

    /// How a point at `p` moves under the motion at unit rate.
    pub fn velocity(&self, p: V) -> V {
        match *self {
            Freedom::Slide { along } => along,
            Freedom::Turn { about, through, pitch } => {
                add(cross(about, sub(p, through)), scale(about, pitch))
            }
        }
    }
}

/// `free to slide along x and turn about z through (0, 0, 0)`: the freedoms as one clause, or
/// `None` where there are none.
pub fn freedoms_text(fs: &[Freedom], dim: usize) -> Option<String> {
    let each: Vec<String> = fs.iter().map(|f| f.text(dim)).collect();
    let (last, rest) = each.split_last()?;
    Some(if rest.is_empty() {
        format!("free to {last}")
    } else {
        format!("free to {} and {last}", rest.join(", "))
    })
}

/// A sign that names a direction once: its largest component positive.
fn canonical(v: &mut [f64]) {
    let big = v.iter().copied().fold(0.0f64, |m, x| if x.abs() > m.abs() { x } else { m });
    if big < 0.0 {
        v.iter_mut().for_each(|x| *x = -*x);
    }
}

/// An orthonormal basis of the span of `basis` (orthonormal, in `m` coordinates) that holds
/// every coordinate axis the span holds: those first, then what is left of the span.
fn axes_first(basis: &[Vec<f64>], m: usize) -> Vec<Vec<f64>> {
    let mut cols: Vec<Vec<f64>> = Vec::new();
    for k in 0..m {
        // the axis's projection onto the span keeps its length only if the span holds it
        let along: f64 = basis.iter().map(|b| b[k] * b[k]).sum();
        if along >= 1.0 - SNAP {
            let mut e = vec![0.0; m];
            e[k] = 1.0;
            cols.push(e);
        }
    }
    cols.extend(basis.iter().cloned());
    let mut out = orthonormalize(&cols, 1e-6);
    out.truncate(basis.len());
    for v in &mut out {
        canonical(v);
    }
    out
}

/// **The rigid-body motions supports `holds` leave a body in `dim` (2 or 3) dimensions free in**:
/// the twists `(v, ω)` — a velocity at the supports' middle `c` and a turn — under which no held
/// point moves along a held direction, `e·(v + ω×(p − c)) = 0`, and no held turn turns.  The
/// slides are the twists that do not turn; each turn the one about its axis, through the point
/// of it nearest `c` (its velocity there along the axis: the pitch, zero for a turn).  World axes
/// are preferred where the freedoms hold them, so a body free about z says so.  In the plane a
/// twist is `(vx, vy, ω_z)`; a body held nowhere is free in all of them.
///
/// `body` is the points the body is: a twist that moves none of them (bars all on one line,
/// turning about it) moves nothing, and is no freedom.  Empty for a body that spans its space (a
/// solid), where only the twist of no motion moves no point of it.
pub fn freedoms(dim: usize, body: &[V], holds: &[Held]) -> Vec<Freedom> {
    let planar = dim == 2;
    let points: Vec<V> = holds.iter().flat_map(|h| h.at.iter().copied()).collect();
    let c = if points.is_empty() {
        [0.0; 3]
    } else {
        scale(points.iter().fold([0.0; 3], |s, p| add(s, *p)), 1.0 / points.len() as f64)
    };
    // the turn's columns are ω times the supports' size, so every column is a velocity
    let size = points.iter().map(|p| norm(sub(*p, c))).fold(0.0f64, f64::max);
    let size = if size > 0.0 { size } else { 1.0 };
    let (n, at) = if planar { (3, 2) } else { (6, 3) };
    let mut rows: Vec<f64> = Vec::new();
    let mut count = 0;
    for h in holds {
        let dirs: Vec<V> = match h.along {
            Some(d) => {
                let d = if planar { [d[0], d[1], 0.0] } else { d };
                let l = norm(d);
                if l > 0.0 { vec![scale(d, 1.0 / l)] } else { Vec::new() }
            }
            None => (0..dim).map(|k| { let mut e = [0.0; 3]; e[k] = 1.0; e }).collect(),
        };
        for p in &h.at {
            let q = scale(sub(*p, c), 1.0 / size);
            for e in &dirs {
                // e·(v + ω×q) = e·v + ω·(q×e)
                let m = cross(q, *e);
                let mut r = vec![0.0; n];
                r[..dim].copy_from_slice(&e[..dim]);
                if planar { r[2] = m[2] } else { r[3..].copy_from_slice(&m) }
                rows.extend(r);
                count += 1;
            }
        }
        if h.turn {
            for j in at..n {
                let mut r = vec![0.0; n];
                r[j] = 1.0;
                rows.extend(r);
                count += 1;
            }
        }
    }
    // the twists that move no point of the body, which every freedom is held square to
    if !body.is_empty() {
        let mut still: Vec<f64> = Vec::new();
        for p in body {
            let q = scale(sub(*p, c), 1.0 / size);
            for k in 0..dim {
                let mut e = [0.0; 3];
                e[k] = 1.0;
                let m = cross(q, e);
                let mut r = vec![0.0; n];
                r[k] = 1.0;
                if planar { r[2] = m[2] } else { r[3..].copy_from_slice(&m) }
                still.extend(r);
            }
        }
        let none = rank_and_nullspace(&Mat::from_vec(still.len() / n, n, still), RIGID).null();
        for j in 0..none.cols {
            rows.extend(none.col(j));
            count += 1;
        }
    }
    let null = rank_and_nullspace(&Mat::from_vec(count, n, rows), RIGID).null();
    let k = null.cols;
    if k == 0 {
        return Vec::new();
    }
    // the turn's block of the free twists, and the twists in which it is zero: the slides
    let turn = null.select_rows(&(at..n).collect::<Vec<_>>());
    let still = rank_and_nullspace(&turn, RIGID).null();
    let slides: Vec<Vec<f64>> = (0..still.cols)
        .map(|j| null.mul_vec(&still.col(j))[..dim].to_vec())
        .collect();
    let slides = axes_first(&orthonormalize(&slides, 1e-6), dim);
    let mut out: Vec<Freedom> = slides.iter().map(|s| {
        let mut along = [0.0; 3];
        along[..dim].copy_from_slice(s);
        Freedom::Slide { along }
    }).collect();
    // the turns: every direction the free twists turn in, world axes first
    let turns: Vec<Vec<f64>> = (0..k).map(|j| turn.col(j)).collect();
    let turns = axes_first(&orthonormalize(&turns, 1e-6), n - at);
    for w in turns {
        // the free twist turning at `w`, its velocity cleared of every slide
        let (y, _) = min_norm_solve(&turn, &w, RIGID);
        let t = null.mul_vec(&y);
        let mut v = [0.0; 3];
        v[..dim].copy_from_slice(&t[..dim]);
        // ω' = ω·size is the unit `w`: the twist at unit turn is size times it
        v = scale(v, size);
        for s in &slides {
            let d: f64 = (0..dim).map(|i| v[i] * s[i]).sum();
            for i in 0..dim {
                v[i] -= d * s[i];
            }
        }
        let about = if planar { [0.0, 0.0, 1.0] } else { [w[0], w[1], w[2]] };
        let pitch = dot(about, v);
        let pitch = if pitch.abs() <= SNAP * size { 0.0 } else { pitch };
        out.push(Freedom::Turn { about, through: add(c, cross(about, v)), pitch });
    }
    out
}

/* -- unbounded stress at an edge -------------------------------------------------------------- */

/// **Williams' least exponent** (1952) for a traction-free wedge of material angle `alpha`
/// (radians), in its opening mode: the least positive root λ of `sin(λα) + λ sin α = 0`.  Elastic
/// stress near the edge grows as `r^(λ−1)`, so it is bounded where λ ≥ 1 — a wedge of a half turn
/// or less, where this is 1 — and unbounded at a re-entrant one: 0.5445 at three quarters of a
/// turn, ½ at a crack.  The opening mode is the strongest: the sliding and tearing modes' roots
/// are larger.
pub fn williams(alpha: f64) -> f64 {
    if alpha <= PI {
        return 1.0;
    }
    let f = |l: f64| (l * alpha).sin() + l * alpha.sin();
    // positive just past zero, `2 sin α < 0` at one: the first change of sign is the least root
    let steps = 400;
    let mut lo = 0.0;
    for i in 1..=steps {
        let hi = i as f64 / steps as f64;
        if f(hi) <= 0.0 {
            let (mut a, mut b) = (lo, hi);
            for _ in 0..80 {
                let m = 0.5 * (a + b);
                if f(m) > 0.0 { a = m } else { b = m }
            }
            return 0.5 * (a + b);
        }
        lo = hi;
    }
    1.0
}

/// A sharp re-entrant edge, where stress is unbounded: the faces it parts (by their paths from
/// the solid), its middle (model units), the material's angle there and the exponent stress
/// grows by toward it, `λ − 1` (negative).
#[derive(Clone, Debug, PartialEq)]
pub struct Reentrant {
    pub faces: [String; 2],
    pub at: V,
    pub angle: f64,
    pub exponent: f64,
}

/// **Every sharp re-entrant edge of a body** (`BodyFaces::whole`, its rounds made): each pair of
/// faces meeting at a corner where the material's angle (`BodyFaces::angle`) passes a half turn,
/// once per pair at its sharpest.  A rounded edge is no corner — its faces meet tangent — and
/// is not one; a convex edge's stress is bounded.
pub fn reentrant(body: &BodyFaces) -> Vec<Reentrant> {
    let mut by_pair: BTreeMap<(usize, usize), Reentrant> = BTreeMap::new();
    for e in 0..body.brep().0.edges.len() as u32 {
        if body.convex(e) != Some(false) || body.seam(e) {
            continue;
        }
        let &[fa, fb] = body.faces_of(e) else { continue };
        let at = body.middle(e);
        let Some(angle) = body.angle(&[e], at) else { continue };
        if angle <= PI + STRAIGHT {
            continue;
        }
        let names = body.paths();
        let found = Reentrant {
            faces: [names[fa].text(), names[fb].text()],
            at,
            angle,
            exponent: williams(angle) - 1.0,
        };
        let key = (fa.min(fb), fa.max(fb));
        if by_pair.get(&key).is_none_or(|r| angle > r.angle) {
            by_pair.insert(key, found);
        }
    }
    by_pair.into_values().collect()
}

impl Reentrant {
    /// `stress is unbounded at the edge of `web` and `flange` near (60, 20, 2), where the
    /// material's angle is 270°: it grows as r^-0.456 toward the edge; round it`.
    pub fn text(&self) -> String {
        format!(
            "stress is unbounded at the edge of `{}` and `{}` near {}, where the material's angle \
             is {}°: it grows as r^{} toward the edge; round it",
            self.faces[0],
            self.faces[1],
            point_text(self.at, 3),
            fmt_g(self.angle.to_degrees(), 4),
            fmt_g(self.exponent, 3),
        )
    }
}

/* -- a degenerate modal pair ------------------------------------------------------------------ */

/// Eigenvalues found more than once: the pairs `first..first + count` of a solve (counted from
/// zero) share `value`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Repeated {
    pub first: usize,
    pub count: usize,
    pub value: f64,
}

/// **The repeated eigenvalues of a solve**: runs of its ascending values within `eigen::GAP` of
/// each other, the gap Sylvester's count already reads as one value.  A run at rest (below
/// `AT_REST` of the largest) is a rigid-body motion's, which `freedoms` names, and is left out.
/// An eigenvalue of more than one vector has no derivative there (`φᵀ(L' − λR')φ` is no longer
/// one number), so a requirement on one of the run reads all of it.
pub fn repeated(p: &Pairs) -> Vec<Repeated> {
    let scale = p.values.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let mut out = Vec::new();
    let mut i = 0;
    while i < p.values.len() {
        let v = p.values[i];
        let mut j = i + 1;
        while j < p.values.len() && p.values[j] - v <= GAP * v.abs().max(AT_REST * scale) {
            j += 1;
        }
        if j - i > 1 && v.abs() > AT_REST * scale {
            out.push(Repeated { first: i, count: j - i, value: v });
        }
        i = j;
    }
    out
}

impl Repeated {
    /// **The ring a pair is symmetric under**: with `turn` one step of a ring of `order` (a
    /// permutation of the freedoms that keeps the pencil, so it maps the pair's space onto
    /// itself) and `b` the inner product the pair is orthonormal in, the step acts on the pair as
    /// the 2×2 `A = Φᵀ B T Φ`.  A ring's pair turns by `2πk/order` under it, `k` its wave:
    /// `Some(k)`.
    /// `None` where the run is not a pair, where the step keeps each mode (the pair is not the
    /// ring's), or where it acts as no turn of the ring.
    pub fn wave(&self, p: &Pairs, b: &Sym, turn: &dyn Fn(&[f64]) -> Vec<f64>, order: usize)
        -> Option<usize> {
        if self.count != 2 || order < 3 {
            return None;
        }
        let phi = &p.vectors[self.first..self.first + 2];
        let bt: Vec<Vec<f64>> = phi.iter().map(|f| b.mul_vec(&turn(f))).collect();
        let a = |i: usize, j: usize| vdot(&phi[i], &bt[j]);
        let (c, s) = (0.5 * (a(0, 0) + a(1, 1)), 0.5 * (a(1, 0) - a(0, 1)));
        // a turn of the plane: orthogonal, its determinant one
        let det = a(0, 0) * a(1, 1) - a(0, 1) * a(1, 0);
        if (det - 1.0).abs() > 1e-6 || (a(0, 0) - a(1, 1)).abs() > 1e-6
            || (a(1, 0) + a(0, 1)).abs() > 1e-6 {
            return None;
        }
        let theta = s.atan2(c).abs();
        let k = (theta * order as f64 / (2.0 * PI)).round() as usize;
        let want = 2.0 * PI * k as f64 / order as f64;
        (k > 0 && (theta - want).abs() <= 1e-6).then_some(k)
    }

    /// `modes 2 and 3 are one degenerate pair, symmetric under the ring of 120 (wave 1): …` —
    /// the modes counted from one, and the ring where `wave` gives it as `(k, order)`.
    pub fn text(&self, wave: Option<(usize, usize)>) -> String {
        let modes: Vec<String> =
            (self.first + 1..=self.first + self.count).map(|m| m.to_string()).collect();
        let (last, rest) = modes.split_last().expect("a run is two or more");
        let what = if self.count == 2 { "one degenerate pair" } else { "one repeated value" };
        let ring = match wave {
            Some((k, order)) => format!(", symmetric under the ring of {order} (wave {k})"),
            None => String::new(),
        };
        format!(
            "modes {} and {last} are {what}{ring}: no derivative is defined there, so a \
             requirement reads them together and not either alone",
            rest.join(", ")
        )
    }
}

/* -- a requirement out of reach --------------------------------------------------------------- */

/// What the numbers that drive a row can do for it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reach {
    /// It holds: within `SOLVED` of what it states.
    Reached,
    /// It does not hold yet, and the numbers that drive it move it: no verdict.
    Moves,
    /// It does not hold, and every number it reads is held: nothing drives it.
    Held,
    /// It does not hold, and the numbers that drive it stand at an extreme of its value: `value`
    /// is the least they reach here where `least` (it states less), else the most.
    Extreme { value: f64, least: bool },
}

/// **Whether a row is out of reach of what drives it**: its `value` against what it states
/// (`target`), its gradient in its columns, each column's `scale` (how far that number is moved
/// to be moved at all) and which columns are `free`.  A row that does not hold is out of reach
/// where no column it reads is free, or where its change over the free ones, each moved by its
/// scale, is below `STATIONARY` of its value: the solve that stopped there stopped at the
/// extreme of what the row can be, short of what it states.  The extreme is a least where the
/// row states less (a solve shrinking the gap stops at a minimum below it), else a most.
pub fn reach(value: f64, target: f64, grad: &[f64], scale: &[f64], free: &[bool]) -> Reach {
    let size = value.abs().max(target.abs());
    if (value - target).abs() <= SOLVED * size {
        return Reach::Reached;
    }
    if !free.iter().any(|f| *f) {
        return Reach::Held;
    }
    let rate: f64 = (0..grad.len()).filter(|&i| free[i]).map(|i| (grad[i] * scale[i]).powi(2))
        .sum::<f64>().sqrt();
    if rate <= STATIONARY * size {
        Reach::Extreme { value, least: value > target }
    } else {
        Reach::Moves
    }
}

impl Reach {
    /// The verdict in words, its numbers spelled by `spell`, where there is one.
    pub fn text(&self, spell: &dyn Fn(f64) -> String) -> Option<String> {
        match *self {
            Reach::Reached | Reach::Moves => None,
            Reach::Held => Some(
                "out of reach: every number it reads is held, so nothing drives it".to_string(),
            ),
            Reach::Extreme { value, least } => Some(format!(
                "out of reach of the dimensions that drive it: {} is the {} they reach here",
                spell(value),
                if least { "least" } else { "most" }
            )),
        }
    }
}

/* -- a scenario, read -------------------------------------------------------------------------- */

/// The bodies a scenario reads, built once each: `BodyFaces::whole`, remembered.
struct Bodies<'a> {
    sk: &'a Sketch,
    built: BTreeMap<usize, Result<Arc<BodyFaces>, String>>,
}

impl Bodies<'_> {
    fn of(&mut self, solid: usize) -> Result<Arc<BodyFaces>, String> {
        let sk = self.sk;
        self.built.entry(solid).or_insert_with(|| BodyFaces::whole(sk, solid).map(Arc::new)).clone()
    }
}

/// The unit direction an `along:` names: an axis's, or a line's from its first end.
fn direction(sk: &Sketch, d: EntRef) -> Result<V, String> {
    let v = match d.kind {
        EntKind::Axis => sk.axes[d.i()].d.map(|p| sk.params[p as usize].value),
        EntKind::Line => {
            let [a, b] = sk.line_ends(d.i());
            sub(sk.world_point(b), sk.world_point(a))
        }
        _ => return Err("`along:` names no direction".to_string()),
    };
    let l = norm(v);
    if l > 0.0 {
        Ok(scale(v, 1.0 / l))
    } else {
        Err("`along:` names a direction of no length".into())
    }
}

/// The points a support holds: a point, a line's ends, a solid's box (its affine hull is space),
/// or every face a path names, by its edges' ends and quarters (so a closed edge holds a circle
/// of points, not one).
fn points(sk: &Sketch, bodies: &mut Bodies, at: &Applied) -> Result<Vec<V>, String> {
    match at {
        Applied::Ent(e) => match e.kind {
            EntKind::Point => Ok(vec![sk.world_point(e.i())]),
            EntKind::Line => Ok(sk.line_ends(e.i()).map(|p| sk.world_point(p)).to_vec()),
            EntKind::Solid => {
                let (lo, hi) = bodies.of(e.i())?.bounds();
                Ok((0..8).map(|k| [0, 1, 2].map(|i| if k >> i & 1 == 0 { lo[i] } else { hi[i] }))
                    .collect())
            }
            kind => {
                let kind = format!("{kind:?}").to_lowercase();
                Err(format!("a support on a {kind} is not read yet"))
            }
        },
        Applied::Face(f) => {
            let body = bodies.of(f.solid as usize)?;
            let faces = body.faces(f);
            let name = &sk.solids[f.solid as usize].name;
            if faces.is_empty() {
                return Err(format!("`{name}.{}` names no face of `{name}`", f.path.join(".")));
            }
            let mut out = Vec::new();
            for fi in faces {
                for &e in body.face_edges(fi) {
                    out.extend([0.0, 0.25, 0.5, 0.75].map(|s| body.along(e, s)));
                }
            }
            Ok(out)
        }
    }
}

/// The solids a scenario names, in the order first named: what its supports hold as one body
/// until an analysis joins bodies of its own (S0).
fn named_solids(sc: &Scenario) -> Vec<usize> {
    let mut out: Vec<usize> = Vec::new();
    let named = sc.supports.iter().map(|s| &s.at)
        .chain(sc.loads.iter().map(|l| &l.at))
        .chain(sc.requirements.iter().map(|r| &r.of));
    for a in named {
        let e = a.entity();
        if e.kind == EntKind::Solid && !out.contains(&e.i()) {
            out.push(e.i());
        }
    }
    out
}

/// **What can be said of a scenario before it is analysed** (#510 P7), at the solved drawing:
/// the rigid-body motions its supports leave (`freedoms`, in space, the supports held as one
/// body), `scenario `lift` leaves `arm` free to turn about z through (0, 0, 0)`; and for every
/// `safety` requirement, each sharp re-entrant edge of its body, where the stress it asks about
/// is unbounded.  A support or a body that cannot be read is said too.  Nothing where all is
/// well.
pub fn scenario_words(sk: &Sketch, sc: &Scenario) -> Vec<String> {
    let mut bodies = Bodies { sk, built: BTreeMap::new() };
    let mut out = Vec::new();
    let mut holds = Vec::new();
    for s in &sc.supports {
        let read = points(sk, &mut bodies, &s.at)
            .and_then(|at| Ok(Held { at, along: s.along.map(|d| direction(sk, d)).transpose()?,
                                     turn: s.hold == Hold::Fixed }));
        match read {
            Ok(h) => holds.push(h),
            Err(e) => out.push(format!("scenario `{}`: {e}", sc.name)),
        }
    }
    if out.is_empty() {
        let fs = freedoms(3, &[], &holds);
        if let Some(free) = freedoms_text(&fs, 3) {
            let solids = named_solids(sc);
            let names: Vec<String> =
                solids.iter().map(|&i| format!("`{}`", sk.solids[i].name)).collect();
            let what = match names.as_slice() {
                [] => "its structure".to_string(),
                [one] => one.clone(),
                [rest @ .., last] => format!("{} and {last}, held as one body,", rest.join(", ")),
            };
            out.push(format!("scenario `{}` leaves {what} {free}", sc.name));
        }
    }
    for r in sc.requirements.iter().filter(|r| r.output == Output::Safety) {
        let e = r.of.entity();
        if e.kind != EntKind::Solid {
            continue;
        }
        let name = &sk.solids[e.i()].name;
        match bodies.of(e.i()) {
            Ok(body) => {
                for edge in reentrant(&body) {
                    out.push(format!("scenario `{}`: `safety({name})` has no answer: {}",
                                     sc.name, edge.text()));
                }
            }
            Err(err) => out.push(format!("scenario `{}`: {err}", sc.name)),
        }
    }
    out
}
