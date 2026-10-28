//! **A ball swept, built exactly** (#238, #684 D): a continuous sweep whose tool is a ball sweeps
//! the points within the ball's radius of the path its centre runs, whatever the motion turns the
//! ball by — the ball's disc square to the path carried along it, with the ball where the path
//! starts and where it ends. So the tool need not end clear of anything: its end poses are the
//! construction's own, and a sweep standing alone (a spring) is a solid like any other.
//!
//! The path is read off the motion. One turn about one axis, a screw or a slide (`Family::twist`)
//! carries it in closed form — an arc, a helix, a line — and `brep::build::canal` builds the tube on
//! it exactly. Any other motion — a roll, a frame along a path (#398 C), one the drawing moves, a
//! joint driven by a law — carries the centre along a curve read point by point: its disc is the
//! one round wall of a face family whose stations are read off the motion (`along`), the frame
//! square to the path carried by double reflection (`motion::reflected`), and between them the
//! wall's polynomials are held within `EXACT` of the disc's size, so the wall is fitted, and lies
//! on a cylinder or a torus where its numbers do (`brep::family`). Refused (two rows, sampled, so
//! said) where the tube would meet itself: the path turning tighter than the ball's radius (B1), and
//! the path coming back within the ball's diameter of itself (B2). The recipe's `canal` node, built
//! by this kernel; held to the sweep's own field before it stands (`gate`), as the lowering table's
//! pieces are. Any other tool stays the generating class's, the constant twist's, or the field's.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::admission::Condition;
use super::family::{EXACT, LONGEST_PIECE};
use crate::brep::family::{Run, NODES};
use crate::brep::recipe::{Family as Walls, FamilyWall, Spine, Writer};
use crate::json::{object, Json};
use crate::model::{Sketch, SolidDef};
use crate::motion::Family;
use crate::space::{add, cross, dot, norm, normalised, scale, sub};
use crate::vocab::num;
use std::f64::consts::{PI, TAU};
use std::rc::Rc;

type V = [f64; 3];

/// A ball's sweep (model units): the ball of `radius` about `centre` where the roll starts, carried
/// along its centre's path (`spine`: a screw's, or the face family its disc makes, `along`), and the
/// name of the ball's face, which names every face the tube has.
#[derive(Clone, Debug)]
pub struct Canal {
    centre: V,
    radius: f64,
    spine: Spine,
    face: String,
}

impl Canal {
    /// The recipe's node, written by `w`.
    pub fn node(&self, w: &Writer) -> Json {
        let mut o = object([("kind", "canal".into()), ("centre", w.point(self.centre)),
            ("radius", w.length(self.radius)), ("face", self.face.clone().into())]);
        match &self.spine {
            &Spine::Screw { origin, axis, angle, advance } => {
                let mm = |p: V| p.map(|x| x * w.scale);
                o.set("origin", w.line_origin(mm(origin), axis));
                o.set("axis", Writer::triple(axis));
                o.set("angle", angle.into());
                o.set("advance", w.length(advance));
            }
            Spine::Tube(walls) => o.set("tube", super::family::json(walls, w)),
        }
        o
    }

    /// Whether the tube is fitted: its centre's path one no closed form carries (#684 D).
    pub fn fitted(&self) -> bool {
        matches!(self.spine, Spine::Tube(_))
    }
}

/// The screw that moves the centre nowhere: `build::canal` builds the ball itself.
const STILL: Spine = Spine::Screw { origin: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 0.0, advance: 0.0 };

/// A ball's sweep refused by a row of its own (#684 D): the row, what was seen, and a point of the
/// centre's path where it was (the sweep's coordinates).
#[derive(Clone, Debug)]
pub struct Refusal {
    pub condition: Condition,
    pub message: String,
    pub witness: Option<V>,
}

/// Why a ball's sweep is no tube: a row refuses it, or it could not be read, built or gated.
#[derive(Clone, Debug)]
pub(crate) enum Why {
    Refused(Refusal),
    Failed(String),
}

impl From<String> for Why {
    fn from(m: String) -> Why { Why::Failed(m) }
}

fn refused(condition: Condition, message: String, witness: V) -> Why {
    Why::Refused(Refusal { condition, message, witness: Some(witness) })
}

/// Sweep `swept` built as a ball's tube, where its tool is a ball and the tube meets itself nowhere
/// (`None` otherwise: a sweep of another kind, or a tube a row refuses, `refusal`): the
/// construction, or why it stands not — its gate refused it.  Remembered against `solid::reads`
/// (`Caches::canal`).
pub fn built(sk: &Sketch, swept: usize) -> Option<Result<Rc<Canal>, String>> {
    match outcome(sk, swept)? {
        Ok(c) => Some(Ok(c)),
        Err(Why::Failed(m)) => Some(Err(m)),
        Err(Why::Refused(_)) => None,
    }
}

/// Why sweep `swept`, a ball's along a path no closed form carries, is no tube: the row its
/// centre's path fails (`None` for a tube, a sweep whose tool is no ball, and one the constant
/// twist's rows are asked of).
pub fn refusal(sk: &Sketch, swept: usize) -> Option<Refusal> {
    match outcome(sk, swept)? {
        Err(Why::Refused(r)) => Some(r),
        _ => None,
    }
}

fn outcome(sk: &Sketch, swept: usize) -> Option<Result<Rc<Canal>, Why>> {
    if !matches!(sk.solids.get(swept)?.def, SolidDef::Swept { .. }) {
        return None;
    }
    sk.caches.canal.get_or(swept, super::MemoKey::of(sk, swept), || read(sk, swept).map(|c| {
        let c = c?;
        gate(sk, swept, &c)?;
        Ok(Rc::new(c))
    }))
}

/// Whether sweep `i` is built as a ball's tube: not a continuous sweep another class or the field
/// answers.
pub fn is_canal(sk: &Sketch, i: usize) -> bool {
    matches!(built(sk, i), Some(Ok(_)))
}

/// The ball's sweep `swept` is, read off its tool's recipe and its motion where the tool is a ball:
/// `None` where it is not, or where it is a screw's whose tube would meet itself (the constant
/// twist's rows are asked of it instead); why not where its path is refused or cannot be read.
fn read(sk: &Sketch, swept: usize) -> Option<Result<Canal, Why>> {
    let SolidDef::Swept { source, motion, .. } = &sk.solids[swept].def else { return None };
    let family = Family::read(sk, *motion as usize).ok()?;
    let scale_mm = super::cad::millimetres(sk);
    // the tool: one node, a ball (millimetres back to the model's units)
    let recipe = crate::brep::recipe::Recipe::read(&super::cad::recipe(sk, *source as usize).ok()?).ok()?;
    let [n] = &recipe.nodes[..] else { return None };
    let (c, r) = n.node.ball(1e-9 * scale_mm.max(1.0)).ok()??;
    let (c, r) = (c.map(|x| x / scale_mm), r / scale_mm);
    let crate::brep::recipe::Node::Revolve { profile, .. } = &n.node else { return None };
    let profile = crate::brep::build::Profile::of(profile).ok()?;
    let face = (0..profile.loops.len()).flat_map(|l| (0..profile.loops[l].len()).map(move |k| (l, k)))
        .find(|&(l, k)| matches!(profile.loops[l][k], crate::brep::build::ProfileEdge::Arc { .. }))
        .map_or(String::new(), |(l, k)| profile.name(l, k));
    tube(sk, swept, &family, c, r, face).transpose()
}

/// The tube the ball of radius `r` about `c` sweeps under `family` over sweep `swept`'s roll, its
/// faces named `face`: `None` where it is a screw's that would meet itself.
fn tube(sk: &Sketch, swept: usize, family: &Family, c: V, r: f64, face: String) -> Result<Option<Canal>, Why> {
    // where the roll starts, and the turn and the step it takes to its end
    let [from, to] = super::roll::interval(sk, swept)?;
    let centre = family.at(from)?.point(c);
    // a path no closed form carries, read off the motion (#684 D); a centre that never moves sweeps
    // the ball itself
    let Some(turn) = family.twist() else {
        let spine = along(&Centre { family, ball: c }, r, &face, [from, to])?.map_or(STILL, Spine::Tube);
        return Ok(Some(Canal { centre, radius: r, spine, face }));
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
    Ok(Some(Canal { centre, radius: r, spine: Spine::Screw { origin: turn.origin, axis: a, angle, advance }, face }))
}

/// **The gate**, as the lowering table's (`lowering::gate`): the tube stands only where the sweep's
/// own field agrees with it — every probe a little inside its boundary material by the field, every
/// one a little outside not (`agreement`, sampled by area).
fn gate(sk: &Sketch, swept: usize, c: &Canal) -> Result<(), String> {
    let brep = crate::brep::recipe::node(&c.node(&Writer::world(1.0)), &Default::default())?;
    let field = super::MaterialField::read(sk, swept, super::cad::AXIS_TOLERANCE)?;
    let what = format!("`{}`: the ball's tube disagrees with the sweep's field", sk.solids[swept].name);
    super::agreement::gate(&field, &brep, None, &what)
}

/// A ball's centre as a motion carries it: the motion, and where the centre is drawn.
struct Centre<'a> {
    family: &'a Family,
    ball: V,
}

impl Centre<'_> {
    /// The centre at the roll `s`, and its velocity a unit of the roll.
    fn at(&self, s: f64) -> Result<(V, V), String> {
        let m = self.family.at(s)?;
        Ok((m.point(self.ball), m.velocity(self.ball)))
    }
}

/// A frame square to the path: its tangent, then the two radius vectors' directions.
type Frame = [V; 3];

/// **The frame square to the centre's path, carried** from where the roll starts by double
/// reflection (`motion::reflected`) over `GRID` steps of the roll, once: the centre and the frame
/// at each node, so any other roll is one step from its nearest, wherever the pieces fall.
struct Carried<'a> {
    path: &'a Centre<'a>,
    from: f64,
    step: f64,
    nodes: Vec<(V, Frame)>,
}

impl<'a> Carried<'a> {
    fn new(path: &'a Centre<'a>, [from, to]: [f64; 2], start: Frame) -> Result<Carried<'a>, String> {
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
    fn reflected(path: &Centre, (x0, f): (V, Frame), s: f64) -> Result<(V, Frame), String> {
        let (x1, v1) = path.at(s)?;
        let t1 = normalised(v1).ok_or_else(|| stops(s))?;
        Ok((x1, crate::motion::reflected(&f, x0, x1, t1)))
    }

    /// The centre and the frame at the roll `s`.
    fn at(&self, s: f64) -> Result<(V, Frame), String> {
        let k = ((s - self.from) / self.step).round().clamp(0.0, GRID as f64) as usize;
        if s == self.from + self.step * k as f64 {
            return Ok(self.nodes[k]);
        }
        Self::reflected(self.path, self.nodes[k], s)
    }
}

/// Why a centre's path that stops at `s` has no disc square to it there.
fn stops(s: f64) -> String {
    format!("the ball's centre stops at {} of the roll, where its path has no tangent", num(s))
}

/// How many points of the centre's path its length is first read at, and how many its rows are
/// read at, at least, and at most: as many as keep them an eighth of the radius apart along it,
/// between the two.
const FIRST_SAMPLES: usize = 64;
const SAMPLES: usize = 1024;
const MOST_SAMPLES: usize = 1 << 15;

/// The first piece of the roll the wall's polynomials span, the shortest, and how many there may
/// be (the longest is a face family's, `LONGEST_PIECE`).
const COARSEST: f64 = 1.0 / 16.0;
const FINEST: f64 = 1e-9;
const MOST_PIECES: usize = 4096;

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
    // the disc's numbers at the roll `s` (`brep::family::Run::Round`), and its points a quarter turn
    // apart
    let disc = |s: f64| -> Result<Vec<f64>, String> {
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
            return Err(Why::Failed(format!("the ball's tube needs more than {MOST_PIECES} pieces to follow its \
                centre's path within {}", num(tol))));
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
            let w = super::family::weights(&ss, q);
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
            return Err(Why::Failed(format!("the ball's centre's path does not run smoothly between {} and {} of \
                the roll: the tube's polynomials miss its disc between them by {}", num(s), num(end), num(miss))));
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
/// itself). Sampled, so a turn or a return between the points is not seen: the gate holds what is
/// built to the sweep's field. The frame square to the path where it starts, its seam off the
/// path's own normal at the first point it turns at (any square to it where it never does), or
/// `None` where the centre never moves.
fn rows(path: &Centre, r: f64, [from, to]: [f64; 2]) -> Result<Option<Frame>, Why> {
    let span = to - from;
    let read = |n: usize| -> Result<Vec<(f64, V, V)>, String> {
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
    let said = format!("read at {} points of its path", pts.len());
    let t0 = normalised(pts[0].2).ok_or_else(|| refused(Condition::Curvature, format!("{} ({said})", stops(from)), pts[0].1))?;
    // B1: its radius of curvature at each point, its rate of turning read off its neighbours'
    let mut normal = None;
    for (i, &(s, p, v)) in pts.iter().enumerate() {
        let speed = norm(v);
        if speed <= 1e-12 * total / span {
            return Err(refused(Condition::Curvature, format!("{} ({said})", stops(s)), p));
        }
        let (a, b) = (&pts[i.saturating_sub(1)], &pts[(i + 1).min(pts.len() - 1)]);
        let rate = scale(sub(b.2, a.2), 1.0 / (b.0 - a.0));
        let bend = norm(cross(v, rate));
        if bend > 0.0 && speed.powi(3) / bend <= r * (1.0 + 1e-9) {
            return Err(refused(Condition::Curvature, format!("its centre's path turns on a radius of {} at {} of \
                the roll, within its radius {} ({said})", num(speed.powi(3) / bend), num(s), num(r)), p));
        }
        if normal.is_none() && bend > 1e-9 * speed * speed {
            let t = scale(v, 1.0 / speed);
            normal = normalised(sub(rate, scale(t, dot(rate, t)))).and_then(|n| normalised(sub(n, scale(t0, dot(n, t0)))));
        }
    }
    if let Some(why) = returns(&pts, &along, r) {
        return Err(why);
    }
    let x = crate::brep::build::seam(t0, normal.unwrap_or_else(|| crate::space::across(t0).0));
    Ok(Some([t0, x, cross(t0, x)]))
}

/// **B2**: two of the points `pts` of the path (`along` its length at each) within the diameter
/// `2 r` of each other, with the points' spacing to spare, half a turn on a circle of the radius or
/// more apart along it.
fn returns(pts: &[(f64, V, V)], along: &[f64], r: f64) -> Option<Why> {
    let spacing = along.windows(2).map(|w| w[1] - w[0]).fold(0.0, f64::max);
    let reach = 2.0 * r + spacing;
    let mut grid = crate::space::Grid::new(reach);
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
            return Some(refused(Condition::Return, format!("its centre's path comes back {} from itself between {} \
                and {} of the roll, within its diameter {} and the {} its points stand apart (read at {} points of \
                its path)", num(norm(sub(p, q))), num(s), num(sj), num(2.0 * r), num(spacing), pts.len()),
                scale(add(p, q), 0.5)));
        }
    }
    None
}
