//! **A ball swept, its tube** (#238, #684 D, #721 F2c): a ball carried along a trajectory sweeps
//! the points within its radius of the path its centre runs, whatever the motion turns the ball by
//! — the ball's disc square to the path carried along it, with the ball where the path starts and
//! where it ends. So the tool need not end clear of anything, and a sweep standing alone (a spring)
//! is a solid like any other.
//!
//! The path is read off the trajectory. A constant twist (a turn, a screw, a slide) carries it in
//! closed form — an arc, a helix, a line — and `build::canal` builds the tube on it exactly
//! (`Spine::Screw`). Any other trajectory carries the centre along a curve read point by point: its
//! disc is the one round wall of a face family whose stations are read off the trajectory
//! (`along`), the frame square to the path carried by double reflection
//! (`trajectory::reflected`), and between them the wall's polynomials are held within
//! `family::EXACT` of the disc's size (`Spine::Tube`). Refused (two rows, sampled, so said) where the
//! tube would meet itself: the path turning tighter than the ball's radius (B1, `Row::Curvature`),
//! and the path coming back within the ball's diameter of itself (B2, `Row::Return`). A screw's tube
//! that would meet itself is not the canal's to build (`tube` answers `None`): the constant twist's
//! rows are asked of it.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::KernelError;
use crate::family::{EXACT, LONGEST_PIECE, NODES, Run, weights};
use crate::recipe::{Family as Walls, FamilyWall, Spine};
use crate::trajectory::{Motion, Trajectory, reflected};
use crate::util::space::{across, add, cross, dot, norm, normalised, scale, sub, Grid};
use super::constant_twist::twist_of;
use std::f64::consts::{PI, TAU};

type V = [f64; 3];

/// A ball's tube: the ball of `radius` about `centre` where the roll starts, carried along its
/// centre's path (`spine`: a constant twist's, or the face family its disc makes).
#[derive(Clone, Debug)]
pub struct Tube {
    pub centre: V,
    pub radius: f64,
    pub spine: Spine,
}

/// The screw that moves the centre nowhere: `build::canal` builds the ball itself.
pub const STILL: Spine = Spine::Screw { origin: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 0.0, advance: 0.0 };

/// The rows a ball's tube is refused by.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Row {
    /// B1: the centre's path turns tighter than the ball's radius, or stops, so the tube would fold.
    Curvature,
    /// B2: the centre's path comes back within the ball's diameter of itself.
    Return,
}

/// What a row saw, at a point of the centre's path (`Refusal::witness`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Seen {
    /// The path stops at `at` of the roll, where it has no tangent.
    Stops { at: f64 },
    /// The path turns on a radius of `radius` at `at` of the roll.
    Turns { radius: f64, at: f64 },
    /// The path comes back `distance` from itself between `from` and `to` of the roll, its points
    /// read `spacing` apart.
    Returns { distance: f64, from: f64, to: f64, spacing: f64 },
}

/// A tube refused by a row: the row, what it saw, how many points of the path it was read at, and
/// a point of the path where it saw it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Refusal {
    pub row: Row,
    pub seen: Seen,
    pub samples: usize,
    pub witness: V,
}

/// Why a ball's sweep is no tube.
#[derive(Clone, Debug, PartialEq)]
pub enum Why {
    /// A row refuses it.
    Refused(Refusal),
    /// The centre's path stops at this roll, where the frame square to it cannot be carried.
    Stops(f64),
    /// Following the path within `tolerance` takes more than `most` pieces.
    Pieces { most: usize, tolerance: f64 },
    /// The path does not run smoothly between `from` and `to` of the roll: the tube's polynomials
    /// miss its disc there by `miss`.
    Rough { from: f64, to: f64, miss: f64 },
    /// The trajectory could not be read.
    Path(KernelError),
}

impl From<KernelError> for Why {
    fn from(e: KernelError) -> Why { Why::Path(e) }
}

/// How many pieces a tube along a path no closed form carries may have.
pub const MOST_PIECES: usize = 4096;

/// **The tube the ball of radius `r` about `ball` sweeps** under `path` over `[from, to]`, its
/// wall named `face`: a constant twist's in closed form, any other's fitted (`along`), the ball
/// itself where the centre never moves; `None` where it is a screw's that would meet itself.
pub fn tube(path: &Trajectory, ball: V, r: f64, face: &str, [from, to]: [f64; 2]) -> Result<Option<Tube>, Why> {
    let centre = pose(path, from)?.point(ball);
    let Some(turn) = twist_of(path) else {
        let spine = along(&Centre { path, ball }, r, face, [from, to])?.map_or(STILL, Spine::Tube);
        return Ok(Some(Tube { centre, radius: r, spine }));
    };
    let tau = to - from;
    let (angle, advance) = (turn.ratio * tau, turn.advance * tau / TAU);
    // the tube meets itself nowhere: its centre's path off the axis by more than the radius, and a
    // helix of more than a turn climbing more than its diameter a turn (a slide's line never does)
    let a = turn.axis;
    let radial = sub(centre, add(turn.origin, scale(a, dot(sub(centre, turn.origin), a))));
    let moves = norm(add(scale(cross(a, radial), angle), scale(a, advance))) > 1e-12 * r.max(1.0);
    if moves && angle != 0.0 && norm(radial) <= r * (1.0 + 1e-9)
        || advance != 0.0 && angle.abs() > TAU && turn.advance.abs() / turn.ratio.abs() <= 2.0 * r {
        return Ok(None);
    }
    Ok(Some(Tube { centre, radius: r, spine: Spine::Screw { origin: turn.origin, axis: a, angle, advance } }))
}

/// The trajectory's pose at the roll `s`.
fn pose(path: &Trajectory, s: f64) -> Result<Motion, KernelError> {
    if !s.is_finite() {
        return Err(KernelError::new(crate::Kind::Failed, "a motion angle must be finite"));
    }
    path.at(s)
}

/// A ball's centre as a trajectory carries it: the trajectory, and where the centre is drawn.
struct Centre<'a> {
    path: &'a Trajectory,
    ball: V,
}

impl Centre<'_> {
    /// The centre at the roll `s`, and its velocity a unit of the roll.
    fn at(&self, s: f64) -> Result<(V, V), Why> {
        let m = pose(self.path, s)?;
        Ok((m.point(self.ball), m.velocity(self.ball)))
    }
}

/// A frame square to the path: its tangent, then the two radius vectors' directions.
type Frame = [V; 3];

/// **The frame square to the centre's path, carried** from where the roll starts by double
/// reflection over `GRID` steps of the roll, once: the centre and the frame at each node, so any
/// other roll is one step from its nearest, wherever the pieces fall.
struct Carried<'a> {
    path: &'a Centre<'a>,
    from: f64,
    step: f64,
    nodes: Vec<(V, Frame)>,
}

impl<'a> Carried<'a> {
    fn new(path: &'a Centre<'a>, [from, to]: [f64; 2], start: Frame) -> Result<Carried<'a>, Why> {
        let step = (to - from) / GRID as f64;
        let mut nodes = vec![(path.at(from)?.0, start)];
        for k in 1..=GRID {
            let s = from + step * k as f64;
            let (x, f) = Self::reflected(path, nodes[k - 1], s)?;
            nodes.push((x, f));
        }
        Ok(Carried { path, from, step, nodes })
    }

    /// The node `(x, f)` carried one step on to the roll `s`.
    fn reflected(path: &Centre, (x0, f): (V, Frame), s: f64) -> Result<(V, Frame), Why> {
        let (x1, v1) = path.at(s)?;
        let t1 = normalised(v1).ok_or(Why::Stops(s))?;
        Ok((x1, reflected(&f, x0, x1, t1)))
    }

    /// The centre and the frame at the roll `s`.
    fn at(&self, s: f64) -> Result<(V, Frame), Why> {
        let k = ((s - self.from) / self.step).round().clamp(0.0, GRID as f64) as usize;
        if s == self.from + self.step * k as f64 {
            return Ok(self.nodes[k]);
        }
        Self::reflected(self.path, self.nodes[k], s)
    }
}

/// How many points of the centre's path its length is first read at, and how many its rows are
/// read at, at least, and at most: as many as keep them an eighth of the radius apart along it,
/// between the two.
const FIRST_SAMPLES: usize = 64;
const SAMPLES: usize = 1024;
const MOST_SAMPLES: usize = 1 << 15;

/// The first piece of the roll the wall's polynomials span, and the shortest (the longest is a face
/// family's, `LONGEST_PIECE`).
const COARSEST: f64 = 1.0 / 16.0;
const FINEST: f64 = 1e-9;

/// How many steps of the roll the frame square to the path is carried over, end to end.
const GRID: usize = 4096;

/// Where between a piece's first two and last two nodes the disc is read again to check its
/// polynomials, as fractions of the piece: where interpolation on Gauss–Lobatto points errs most.
const CHECKED_AT: [f64; 2] = [0.06, 0.94];

/// **The tube along a path no closed form carries** (#684 D): the face family of the ball's disc of
/// radius `r` square to the centre's path over the roll `[from, to]` — its centre the path's point,
/// its radius vectors along the frame carried from the start (`Carried`), the seam turned off the
/// path's own normal as `build::canal`'s is (`build::seam`) — in pieces whose degree-five
/// polynomials hold to the disc read between their nodes within a face family's `EXACT` of its
/// diameter; `None` where the centre never moves. Refused where it would meet itself, read at
/// points of the path (`rows`).
fn along(path: &Centre, r: f64, face: &str, [from, to]: [f64; 2]) -> Result<Option<Walls>, Why> {
    let span = to - from;
    let Some(start) = rows(path, r, [from, to])? else { return Ok(None) };
    let carried = Carried::new(path, [from, to], start)?;
    // the disc's numbers at the roll `s` (`family::Run::Round`), and its points a quarter turn apart
    let disc = |s: f64| -> Result<Vec<f64>, Why> {
        let (p, f) = carried.at(s)?;
        let mut n = p.to_vec();
        n.extend(scale(f[1], r));
        n.extend(scale(f[2], r));
        n.push(TAU);
        Ok(n)
    };
    let rim = |n: &[f64]| -> [V; 4] { std::array::from_fn(|k| Run::Round.at(n, k as f64 / 4.0).0) };
    let tol = EXACT * 2.0 * r;
    let (longest, finest) = (span * LONGEST_PIECE, span * FINEST);
    let mut h = span * COARSEST;
    let mut s = from;
    let mut here = disc(from)?;
    let (mut spans, mut at, mut nodes) = (Vec::new(), Vec::new(), Vec::new());
    while to - s > span * 1e-12 {
        if spans.len() >= MOST_PIECES {
            return Err(Why::Pieces { most: MOST_PIECES, tolerance: tol });
        }
        let end = if h >= to - s { to } else { s + h };
        let ss: [f64; 6] = NODES.map(|t| s + (end - s) * (t + 1.0) / 2.0);
        let mut piece = vec![here.clone()];
        for &q in &ss[1..] {
            piece.push(disc(q)?);
        }
        // the disc read near each end of the piece, held to the polynomials
        let mut miss = 0.0f64;
        for c in CHECKED_AT {
            let q = s + (end - s) * c;
            let w = weights(&ss, q);
            let want = disc(q)?;
            let got: Vec<f64> = (0..want.len()).map(|j| (0..6).map(|i| w[i] * piece[i][j]).sum()).collect();
            for (p, q) in rim(&got).into_iter().zip(rim(&want)) {
                miss = miss.max(norm(sub(p, q)));
            }
        }
        let grow = if miss > 0.0 { 0.9 * (tol / miss).powf(1.0 / 6.0) } else { 2.0 };
        if miss > tol && end - s > finest {
            h = (end - s) * grow.clamp(0.1, 0.5);
            continue;
        }
        if miss > tol * 1e3 {
            return Err(Why::Rough { from: s, to: end, miss });
        }
        h = ((end - s) * grow.clamp(0.5, 2.0)).clamp(finest, longest);
        spans.push([s, end]);
        at.push(ss);
        (s, here) = (end, piece[5].clone());
        nodes.extend(piece);
    }
    let wall = FamilyWall { name: face.to_string(), run: Run::Round, closed: true, nodes };
    Ok(Some(Walls { spans, at, vanish: 1e-6 * r, loops: vec![vec![wall]], carry: None }))
}

/// **The rows of a ball's tube along a path no closed form carries** (#684 D), read at points of
/// the centre's path over the roll, an eighth of the radius apart along it or as near as
/// `MOST_SAMPLES` points come: **B1**, the path turns nowhere tighter than the radius `r` (the tube
/// would fold) and never stops; **B2** (`returns`), the path comes back within the ball's diameter
/// of itself nowhere it has run half a turn of a circle of the radius from (the tube would cross
/// itself). Sampled, so a turn or a return between the points is not seen: the caller holds what is
/// built to the sweep's own reading. The frame square to the path where it starts, its seam off the
/// path's own normal at the first point it turns at (any square to it where it never does), or
/// `None` where the centre never moves.
fn rows(path: &Centre, r: f64, [from, to]: [f64; 2]) -> Result<Option<Frame>, Why> {
    let span = to - from;
    let read = |n: usize| -> Result<Vec<(f64, V, V)>, Why> {
        (0..=n).map(|i| {
            let s = from + span * i as f64 / n as f64;
            path.at(s).map(|(p, v)| (s, p, v))
        }).collect()
    };
    // the path's length along each point, by its speed
    let length = |pts: &[(f64, V, V)]| -> Vec<f64> {
        let mut along = vec![0.0];
        for w in pts.windows(2) {
            along.push(along.last().unwrap() + 0.5 * (w[1].0 - w[0].0) * (norm(w[0].2) + norm(w[1].2)));
        }
        along
    };
    let total = *length(&read(FIRST_SAMPLES)?).last().unwrap();
    if total <= 1e-12 * r.max(1.0) {
        return Ok(None);
    }
    let pts = read(((8.0 * total / r).ceil() as usize).clamp(SAMPLES, MOST_SAMPLES))?;
    let along = length(&pts);
    let refused = |row: Row, seen: Seen, witness: V| Why::Refused(Refusal { row, seen, samples: pts.len(), witness });
    let t0 = normalised(pts[0].2).ok_or_else(|| refused(Row::Curvature, Seen::Stops { at: from }, pts[0].1))?;
    // B1: its radius of curvature at each point, its rate of turning read off its neighbours'
    let mut normal = None;
    for (i, &(s, p, v)) in pts.iter().enumerate() {
        let speed = norm(v);
        if speed <= 1e-12 * total / span {
            return Err(refused(Row::Curvature, Seen::Stops { at: s }, p));
        }
        let (a, b) = (&pts[i.saturating_sub(1)], &pts[(i + 1).min(pts.len() - 1)]);
        let rate = scale(sub(b.2, a.2), 1.0 / (b.0 - a.0));
        let bend = norm(cross(v, rate));
        if bend > 0.0 && speed.powi(3) / bend <= r * (1.0 + 1e-9) {
            return Err(refused(Row::Curvature, Seen::Turns { radius: speed.powi(3) / bend, at: s }, p));
        }
        if normal.is_none() && bend > 1e-9 * speed * speed {
            let t = scale(v, 1.0 / speed);
            normal = normalised(sub(rate, scale(t, dot(rate, t)))).and_then(|n| normalised(sub(n, scale(t0, dot(n, t0)))));
        }
    }
    if let Some(why) = returns(&pts, &along, r) {
        return Err(why);
    }
    let x = crate::build::seam(t0, normal.unwrap_or_else(|| across(t0).0));
    Ok(Some([t0, x, cross(t0, x)]))
}

/// **B2**: two of the points `pts` of the path (`along` its length at each) within the diameter
/// `2 r` of each other, with the points' spacing to spare, half a turn on a circle of the radius or
/// more apart along it.
fn returns(pts: &[(f64, V, V)], along: &[f64], r: f64) -> Option<Why> {
    let spacing = along.windows(2).map(|w| w[1] - w[0]).fold(0.0, f64::max);
    let reach = 2.0 * r + spacing;
    let mut grid = Grid::new(reach);
    for (i, &(_, p, _)) in pts.iter().enumerate() {
        grid.insert(p, i as u32);
    }
    for (i, &(s, p, _)) in pts.iter().enumerate() {
        let mut near = None;
        grid.around(p, |j| {
            let j = j as usize;
            if near.is_none() && j > i && along[j] - along[i] >= PI * r && norm(sub(p, pts[j].1)) < reach {
                near = Some(j);
            }
        });
        if let Some(j) = near {
            let (sj, q, _) = pts[j];
            return Some(Why::Refused(Refusal { row: Row::Return, seen: Seen::Returns { distance: norm(sub(p, q)),
                from: s, to: sj, spacing }, samples: pts.len(), witness: scale(add(p, q), 0.5) }));
        }
    }
    None
}
