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

use crate::eigen::Pairs;
use crate::json::fmt_g;
use crate::linalg::{dot as vdot, min_norm_solve, orthonormalize, rank_and_nullspace, Mat};
use crate::model::{Applied, EntKind, EntRef, Hold, Output, Scenario, Sketch};
use crate::solid::faces::BodyFaces;
use crate::space::{add, cross, dot, normalised, norm, scale, sub};
use crate::sparse::Sym;
use crate::vocab::all_of;
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;

type V = [f64; 3];

/// Below this, relative to the largest singular value of the supports' rows, a twist is left
/// free: the diagnosis's rank cut (`system::RANK_TOL`), the supports' rows conditioned as the
/// Jacobian's are, each a unit direction and every column a velocity.
const RIGID: f64 = crate::system::RANK_TOL;
/// A unit direction within this of a world axis is named by it, and a basis of freedoms prefers
/// the axes it holds within this.
const SNAP: f64 = 1e-9;
/// A material angle within this of a half turn is no corner of the stress's.
const STRAIGHT: f64 = 1e-6;
/// Eigenvalues below this fraction of the largest are a rigid-body motion's, which `freedoms`
/// names: no pair of theirs is said to be degenerate.
const AT_REST: f64 = 1e-8;
/// A row's first-order change over the numbers that drive it, each moved by its own scale,
/// below this fraction of its value: it stands at its extreme.
const STATIONARY: f64 = 1e-6;

/* -- the motions the supports leave --------------------------------------------------------- */

/// World axis `k`'s unit direction.
pub fn axis(k: usize) -> V {
    let mut e = [0.0; 3];
    e[k] = 1.0;
    e
}

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
        let e = axis(k);
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
    (!each.is_empty()).then(|| format!("free to {}", all_of(&each)))
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
    // point `p` held along unit `e`: e·(v + ω×q) = e·v + ω·(q×e), q = (p − c)/size
    let twist = |rows: &mut Vec<f64>, p: V, e: V| {
        let m = cross(scale(sub(p, c), 1.0 / size), e);
        rows.extend_from_slice(&e[..dim]);
        rows.extend_from_slice(if planar { &m[2..] } else { &m[..] });
    };
    let mut rows: Vec<f64> = Vec::new();
    for h in holds {
        let dirs: Vec<V> = match h.along {
            Some(d) => normalised(if planar { [d[0], d[1], 0.0] } else { d }).into_iter().collect(),
            None => (0..dim).map(axis).collect(),
        };
        for p in &h.at {
            for e in &dirs {
                twist(&mut rows, *p, *e);
            }
        }
        if h.turn {
            for j in at..n {
                rows.extend((0..n).map(|i| if i == j { 1.0 } else { 0.0 }));
            }
        }
    }
    // the twists that move no point of the body, which every freedom is held square to
    if !body.is_empty() {
        let mut still: Vec<f64> = Vec::new();
        for p in body {
            for k in 0..dim {
                twist(&mut still, *p, axis(k));
            }
        }
        let none = rank_and_nullspace(&Mat::from_vec(still.len() / n, n, still), RIGID).null();
        for j in 0..none.cols {
            rows.extend(none.col(j));
        }
    }
    let null = rank_and_nullspace(&Mat::from_vec(rows.len() / n, n, rows), RIGID).null();
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
    // orthonormal already: heads of orthonormal twists that do not turn
    let slides = axes_first(&slides, dim);
    let mut out: Vec<Freedom> = slides.iter().map(|s| {
        let mut along = [0.0; 3];
        along[..dim].copy_from_slice(s);
        Freedom::Slide { along }
    }).collect();
    // the turns: every direction the free twists turn in, world axes first
    let turns: Vec<Vec<f64>> = (0..k).map(|j| turn.col(j)).collect();
    let turns = axes_first(&orthonormalize(&turns, 1e-6), n - at);
    for w in turns {
        // the free twist turning at `w`, the least-norm one: square to every slide
        let (y, _) = min_norm_solve(&turn, &w, RIGID);
        let t = null.mul_vec(&y);
        let mut v = [0.0; 3];
        v[..dim].copy_from_slice(&t[..dim]);
        // ω' = ω·size is the unit `w`: the twist at unit turn is size times it
        let v = scale(v, size);
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
            // bisection, not `roots::bracketed_root`: at a crack the root is this bracket's end,
            // which regula falsi never moves off, and returns the bracket's middle
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
    // the sharpest edge of each pair, by its faces, its middle and its angle
    let mut by_pair: BTreeMap<(usize, usize), (usize, usize, V, f64)> = BTreeMap::new();
    for e in 0..body.brep().0.edges.len() as u32 {
        // a seam is no corner: `convex` is `None` there
        if body.convex(e) != Some(false) {
            continue;
        }
        let &[fa, fb] = body.faces_of(e) else { continue };
        let at = body.middle(e);
        let Some(angle) = body.angle(&[e], at) else { continue };
        let key = (fa.min(fb), fa.max(fb));
        if angle > PI + STRAIGHT && by_pair.get(&key).is_none_or(|r| angle > r.3) {
            by_pair.insert(key, (fa, fb, at, angle));
        }
    }
    let names = body.paths();
    by_pair.into_values().map(|(fa, fb, at, angle)| Reentrant {
        faces: [names[fa].text(), names[fb].text()],
        at,
        angle,
        exponent: williams(angle) - 1.0,
    }).collect()
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

/// **The repeated eigenvalues of a solve**: its runs of one value (`Pairs::runs`, the eigensolver's
/// own reading of when values are one) of more than one pair.  A run at rest (below `AT_REST` of
/// the largest) is a rigid-body motion's, which `freedoms` names, and is left out.  An eigenvalue
/// of more than one vector has no derivative there (`φᵀ(L' − λR')φ` is no longer one number), so
/// a requirement on one of the run reads all of it.
pub fn repeated(p: &Pairs) -> Vec<Repeated> {
    let scale = p.values.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    p.runs().into_iter()
        .filter(|r| r.len() > 1 && p.values[r.start].abs() > AT_REST * scale)
        .map(|r| Repeated { first: r.start, count: r.len(), value: p.values[r.start] })
        .collect()
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
        let what = if self.count == 2 { "one degenerate pair" } else { "one repeated value" };
        let ring = match wave {
            Some((k, order)) => format!(", symmetric under the ring of {order} (wave {k})"),
            None => String::new(),
        };
        format!(
            "modes {} are {what}{ring}: no derivative is defined there, so a requirement reads \
             them together and not either alone",
            all_of(&modes)
        )
    }
}

/* -- a requirement out of reach --------------------------------------------------------------- */

/// What the numbers that drive a row can do for it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reach {
    /// It holds: within what the solver accepts (`SolveOpts::acceptance_tol`) of what it states,
    /// at the size of the larger.
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
    if (value - target).abs() <= crate::solve::SolveOpts::default().acceptance_tol * size {
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

/// The unit direction an `along:` names: an axis's, or a line's from its first end.
pub(crate) fn direction(sk: &Sketch, d: EntRef) -> Result<V, String> {
    let v = match d.kind {
        EntKind::Axis => sk.axes[d.i()].d.map(|p| sk.params[p as usize].value),
        EntKind::Line => {
            let [a, b] = sk.line_ends(d.i());
            sub(sk.world_point(b), sk.world_point(a))
        }
        _ => return Err("`along:` names no direction".to_string()),
    };
    normalised(v).ok_or_else(|| "`along:` names a direction of no length".into())
}

/// The points a support holds: a point, a line's ends, a solid's box (its affine hull is space),
/// or every face a path names, by its edges' ends and quarters (so a closed edge holds a circle
/// of points, not one).
fn points(sk: &Sketch, at: &Applied) -> Result<Vec<V>, String> {
    match at {
        Applied::Ent(e) => match e.kind {
            EntKind::Point => Ok(vec![sk.world_point(e.i())]),
            EntKind::Line => Ok(sk.line_ends(e.i()).map(|p| sk.world_point(p)).to_vec()),
            EntKind::Solid => {
                let (lo, hi) = BodyFaces::whole(sk, e.i())?.bounds();
                Ok((0..8).map(|k| [0, 1, 2].map(|i| if k >> i & 1 == 0 { lo[i] } else { hi[i] }))
                    .collect())
            }
            kind => {
                let kind = format!("{kind:?}").to_lowercase();
                Err(format!("a support on a {kind} is not read yet"))
            }
        },
        Applied::Face(f) => {
            let body = BodyFaces::whole(sk, f.solid as usize)?;
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
        Applied::Beam(b) => Err(format!("a support holds points, and `{}` is a beam",
                                        sk.beams[*b].name)),
    }
}

/// The solids a scenario names, in the order first named: what its supports hold as one body,
/// where a frame's are read with the frame (`beams::read`).
fn named_solids(sc: &Scenario) -> Vec<usize> {
    let mut out: Vec<usize> = Vec::new();
    for i in sc.applied().filter_map(|a| a.solid()) {
        if !out.contains(&i) {
            out.push(i);
        }
    }
    out
}

/// **What can be said of a scenario** (#510 P7), at the solved drawing: the rigid-body motions
/// its supports leave (`freedoms`, in space, the supports held as one body), `scenario `lift`
/// leaves `arm` free to turn about z through (0, 0, 0)`; and for every `safety` requirement,
/// each sharp re-entrant edge of its body, where the stress it asks about is unbounded, once per
/// solid.  A support or a body that cannot be read is said too.  Nothing where all is well.  Each
/// body is the drawing's, built once (`BodyFaces::whole`).  A support that holds a frame of
/// beams is the frame's, and what a frame leaves free is said with its answers (#510 S0,
/// `scenario_read`).
pub fn scenario_words(sk: &Sketch, sc: &Scenario) -> Vec<String> {
    scenario_read(sk, sc).0
}

/// **A scenario, read** (#510 S0): its words (`scenario_words`'s, then its frames'), and what
/// its frames answer of each requirement (`beams::read`).
pub fn scenario_read(sk: &Sketch, sc: &Scenario) -> (Vec<String>, crate::beams::Read) {
    let frames = crate::beams::read(sk, sc);
    let mut out = Vec::new();
    let mut holds = Vec::new();
    for (k, s) in sc.supports.iter().enumerate() {
        if frames.holds_frame[k] {
            continue;
        }
        let read = points(sk, &s.at)
            .and_then(|at| Ok(Held { at, along: s.along.map(|d| direction(sk, d)).transpose()?,
                                     turn: s.hold == Hold::Fixed }));
        match read {
            Ok(h) => holds.push(h),
            Err(e) => out.push(format!("scenario `{}`: {e}", sc.name)),
        }
    }
    let solids = named_solids(sc);
    // a scenario of frames alone holds no body of its own
    let of_frames = frames.reads_a_frame() && solids.is_empty() && holds.is_empty();
    if out.is_empty() && !of_frames {
        let fs = freedoms(3, &[], &holds);
        if let Some(free) = freedoms_text(&fs, 3) {
            let names: Vec<String> =
                solids.iter().map(|&i| format!("`{}`", sk.solids[i].name)).collect();
            let what = match names.len() {
                0 => "its structure".to_string(),
                1 => names[0].clone(),
                _ => format!("{}, held as one body,", all_of(&names)),
            };
            out.push(format!("scenario `{}` leaves {what} {free}", sc.name));
        }
    }
    let asked: BTreeSet<usize> = sc.requirements.iter()
        .filter(|r| r.output == Output::Safety)
        .filter_map(|r| r.of.solid())
        .collect();
    for solid in asked {
        let name = &sk.solids[solid].name;
        match BodyFaces::whole(sk, solid) {
            Ok(body) => {
                for edge in reentrant(&body) {
                    out.push(format!("scenario `{}`: `{}({name})` has no answer: {}",
                                     sc.name, Output::Safety.word(), edge.text()));
                }
            }
            Err(err) => out.push(format!("scenario `{}`: {err}", sc.name)),
        }
    }
    out.extend(frames.words.iter().cloned());
    (out, frames)
}
