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
use crate::brep::family::{Run, NODES};
use crate::brep::recipe::{Family as Walls, FamilyWall, Writer};
use crate::json::{object, Json};
use crate::model::{Sketch, SolidDef};
use crate::motion::Family;
use crate::space::{add, cross, dot, norm, normalised, scale, sub};
use crate::vocab::num;
use std::f64::consts::{PI, TAU};
use std::rc::Rc;

type V = [f64; 3];

/// A ball's sweep (model units): the ball of `radius` about `centre` where the roll starts, carried
/// along its centre's path (`Path`), and the name of the ball's face, which names every face the
/// tube has.
#[derive(Clone, Debug)]
pub struct Canal {
    centre: V,
    radius: f64,
    path: Path,
    face: String,
}

/// The path a ball's centre runs: carried by the screw about the line through `origin` along the
/// unit `axis` — a turn by `angle` and a step of `advance` (no step, a turn; no turn, a slide) — or,
/// where no closed form carries it, the face family the ball's disc square to it makes (`along`).
#[derive(Clone, Debug)]
enum Path {
    Screw { origin: V, axis: V, angle: f64, advance: f64 },
    Tube(Walls),
}

impl Canal {
    /// The recipe's node, written by `w`.
    pub fn node(&self, w: &Writer) -> Json {
        let mut o = object([("kind", "canal".into()), ("centre", w.point(self.centre)),
            ("radius", w.length(self.radius)), ("face", self.face.clone().into())]);
        match &self.path {
            &Path::Screw { origin, axis, angle, advance } => {
                let mm = |p: V| p.map(|x| x * w.scale);
                o.set("origin", w.line_origin(mm(origin), axis));
                o.set("axis", Writer::triple(axis));
                o.set("angle", angle.into());
                o.set("advance", w.length(advance));
            }
            Path::Tube(walls) => o.set("tube", super::family::json(walls, w)),
        }
        o
    }

    /// Whether the tube is fitted: its centre's path one no closed form carries (#684 D).
    pub fn fitted(&self) -> bool {
        matches!(self.path, Path::Tube(_))
    }
}

/// A ball's sweep refused by a row of its own (#684 D): the row, what was seen, and a point of the
/// centre's path where it was (the sweep's coordinates).
#[derive(Clone, Debug)]
pub struct Refusal {
    pub condition: Condition,
    pub message: String,
    pub witness: Option<V>,
}

/// What a ball's sweep is: its tube, refused by a row, or a tube its gate refused or that could not
/// be read or built.
#[derive(Clone, Debug)]
pub(crate) enum Outcome {
    Built(Rc<Canal>),
    Refused(Refusal),
    Failed(String),
}

/// Sweep `swept` built as a ball's tube, where its tool is a ball and the tube meets itself nowhere
/// (`None` otherwise: a sweep of another kind, or a tube a row refuses, `refusal`): the
/// construction, or why it stands not — its gate refused it.  Remembered against `solid::reads`
/// (`Caches::canal`).
pub fn built(sk: &Sketch, swept: usize) -> Option<Result<Rc<Canal>, String>> {
    match outcome(sk, swept)? {
        Outcome::Built(c) => Some(Ok(c)),
        Outcome::Failed(m) => Some(Err(m)),
        Outcome::Refused(_) => None,
    }
}

/// Why sweep `swept`, a ball's, is no tube: the row its centre's path fails (`None` for a tube, and
/// for a sweep whose tool is no ball).
pub fn refusal(sk: &Sketch, swept: usize) -> Option<Refusal> {
    match outcome(sk, swept)? {
        Outcome::Refused(r) => Some(r),
        _ => None,
    }
}

fn outcome(sk: &Sketch, swept: usize) -> Option<Outcome> {
    if !matches!(sk.solids.get(swept)?.def, SolidDef::Swept { .. }) {
        return None;
    }
    sk.caches.canal.get_or(swept, super::MemoKey::of(sk, swept), || read(sk, swept).map(|c| match c {
        Ok(c) => match gate(sk, swept, &c) {
            Ok(()) => Outcome::Built(Rc::new(c)),
            Err(m) => Outcome::Failed(m),
        },
        Err(why) => why,
    }))
}

/// Whether sweep `i` is built as a ball's tube: not a continuous sweep another class or the field
/// answers.
pub fn is_canal(sk: &Sketch, i: usize) -> bool {
    matches!(built(sk, i), Some(Ok(_)))
}

/// The ball's sweep `swept` is, read off its tool's recipe and its motion where the tool is a ball:
/// `None` where it is not, an outcome other than a tube where its path is refused or cannot be read.
fn read(sk: &Sketch, swept: usize) -> Option<Result<Canal, Outcome>> {
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
    let failed = |e: String| Some(Err(Outcome::Failed(e)));
    let refused = |condition, message, witness| Some(Err(Outcome::Refused(Refusal { condition, message, witness })));
    // where the roll starts, and the turn and the step it takes to its end
    let [from, to] = match super::roll::interval(sk, swept) {
        Ok(roll) => roll,
        Err(e) => return failed(e),
    };
    let centre = match family.at(from) {
        Ok(pose) => pose.point(c),
        Err(e) => return failed(e),
    };
    // a path no closed form carries, read off the motion (#684 D)
    let Some(turn) = family.twist() else {
        return Some(match along(&Centre { family: &family, ball: c }, r, &face, [from, to]) {
            Ok(Some(walls)) => Ok(Canal { centre, radius: r, path: Path::Tube(walls), face }),
            // a centre that never moves sweeps the ball itself
            Ok(None) => Ok(Canal { centre, radius: r, face,
                path: Path::Screw { origin: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 0.0, advance: 0.0 } }),
            Err(why) => Err(why),
        });
    };
    let tau = to - from;
    let (angle, advance) = (turn.ratio * tau, turn.advance * tau / TAU);
    // the tube meets itself nowhere: its centre's path off the axis by more than the radius, and a
    // helix of more than a turn climbing more than its diameter a turn (a slide's line never does)
    let a = turn.axis;
    let radial = sub(centre, add(turn.origin, scale(a, dot(sub(centre, turn.origin), a))));
    let moves = norm(add(scale(cross(a, radial), angle), scale(a, advance))) > 1e-12 * r.max(1.0);
    if moves && angle != 0.0 && norm(radial) <= r * (1.0 + 1e-9) {
        return refused(Condition::Curvature, format!("its centre turns {} from the axis, within its radius {}",
            num(norm(radial)), num(r)), Some(centre));
    }
    if advance != 0.0 && angle.abs() > TAU && turn.advance.abs() / turn.ratio.abs() <= 2.0 * r {
        return refused(Condition::Return, format!("its centre's helix climbs {} a turn over more than a turn, within \
            its diameter {}", num(turn.advance.abs() / turn.ratio.abs()), num(2.0 * r)), Some(centre));
    }
    Some(Ok(Canal { centre, radius: r, path: Path::Screw { origin: turn.origin, axis: a, angle, advance }, face }))
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

    /// The frame `f` square to the path at `s0` carried to `s1` by double reflection, in steps of the
    /// roll no longer than `step`.
    fn carried(&self, f: &Frame, s0: f64, s1: f64, step: f64) -> Result<Frame, String> {
        let n = ((s1 - s0).abs() / step).ceil().max(1.0) as usize;
        let (mut x0, _) = self.at(s0)?;
        let mut f = *f;
        for k in 1..=n {
            let s = s0 + (s1 - s0) * k as f64 / n as f64;
            let (x1, v1) = self.at(s)?;
            let t1 = normalised(v1).ok_or_else(|| stops(s))?;
            f = crate::motion::reflected(&f, x0, x1, t1);
            x0 = x1;
        }
        Ok(f)
    }
}

/// A frame square to the path: its tangent, then the two radius vectors' directions.
type Frame = [V; 3];

/// Why a centre's path that stops at `s` has no disc square to it there.
fn stops(s: f64) -> String {
    format!("the ball's centre stops at {} of the roll, where its path has no tangent", num(s))
}

/// How many points of the centre's path its rows are read at, at least, and at most: as many as
/// keep them an eighth of the radius apart along it, between the two.
const SAMPLES: usize = 1024;
const MOST_SAMPLES: usize = 1 << 15;

/// How near the wall's polynomials in the roll must pass the disc read between their nodes, as a
/// part of the disc's diameter: the exact surface's error, well under the kernel's tolerance.
const EXACT: f64 = 1e-10;

/// The first and the longest pieces of the roll the wall's polynomials span, the shortest, and how
/// many there may be.
const COARSEST: f64 = 1.0 / 16.0;
const LONGEST_PIECE: f64 = 0.25;
const FINEST: f64 = 1e-9;
const MOST_PIECES: usize = 4096;

/// How many steps of the roll the frame square to the path is carried over, end to end.
const GRID: f64 = 4096.0;

/// Where between a piece's first two and last two nodes the disc is read again to check its
/// polynomials, as fractions of the piece: where interpolation on Gauss–Lobatto points errs most.
const CHECKED_AT: [f64; 2] = [0.06, 0.94];

/// **The tube along a path no closed form carries** (#684 D): the face family of the ball's disc of
/// radius `r` square to the centre's path over the roll `[from, to]` — its centre the path's point,
/// its radius vectors along the frame carried from the start by double reflection, the seam turned
/// off the path's own normal by `build::SEAM`'s angle as `canal`'s is — in pieces whose degree-five
/// polynomials hold to the disc read between their nodes within `EXACT` of its diameter; `None`
/// where the centre never moves. Refused where it would meet itself, read at points of the path
/// (`rows`).
fn along(path: &Centre, r: f64, face: &str, [from, to]: [f64; 2]) -> Result<Option<Walls>, Outcome> {
    let failed = |e: String| Outcome::Failed(e);
    let span = to - from;
    let Some(start) = rows(path, r, [from, to])? else { return Ok(None) };
    let step = span / GRID;
    let numbers = |p: V, f: &Frame| -> Vec<f64> {
        let mut n = p.to_vec();
        n.extend(scale(f[1], r));
        n.extend(scale(f[2], r));
        n.push(TAU);
        n
    };
    // the disc's points a quarter turn apart, read off its numbers
    let rim = |n: &[f64]| -> [V; 4] {
        std::array::from_fn(|k| Run::Round.at(n, k as f64 / 4.0).0)
    };
    let tol = EXACT * 2.0 * r;
    let (longest, finest) = (span * LONGEST_PIECE, span * FINEST);
    let mut h = span * COARSEST;
    let (mut s, mut frame) = (from, start);
    let mut here = numbers(path.at(from).map_err(failed)?.0, &frame);
    let (mut spans, mut at, mut nodes) = (Vec::new(), Vec::new(), Vec::new());
    while to - s > span * 1e-12 {
        if spans.len() >= MOST_PIECES {
            return Err(failed(format!("the ball's tube needs more than {MOST_PIECES} pieces to follow its \
                centre's path within {}", num(tol))));
        }
        let left = to - s;
        let end = if h >= left { to } else { s + h };
        let ss: [f64; 6] = NODES.map(|t| s + (end - s) * (t + 1.0) / 2.0);
        let mut frames = vec![frame];
        let mut piece = vec![here.clone()];
        for i in 1..6 {
            let f = path.carried(&frames[i - 1], ss[i - 1], ss[i], step).map_err(failed)?;
            piece.push(numbers(path.at(ss[i]).map_err(failed)?.0, &f));
            frames.push(f);
        }
        // the disc read near each end of the piece, held to the polynomials
        let mut miss = 0.0f64;
        for (k, &c) in CHECKED_AT.iter().enumerate() {
            let q = s + (end - s) * c;
            let near = if k == 0 { 0 } else { 4 };
            let f = path.carried(&frames[near], ss[near], q, step).map_err(failed)?;
            let want = numbers(path.at(q).map_err(failed)?.0, &f);
            let w = super::family::weights(&ss, q);
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
            return Err(failed(format!("the ball's centre's path does not run smoothly between {} and {} of the \
                roll: the tube's polynomials miss its disc between them by {}", num(s), num(end), num(miss))));
        }
        h = ((end - s) * grow.clamp(0.5, 2.0)).clamp(finest, longest);
        spans.push([s, end]);
        at.push(ss);
        (s, frame, here) = (end, frames[5], piece[5].clone());
        nodes.extend(piece);
    }
    let wall = FamilyWall { name: face.to_string(), run: Run::Round, closed: true, nodes };
    Ok(Some(Walls { spans, at, vanish: 1e-6 * r, loops: vec![vec![wall]], carry: None }))
}

/// **The rows of a ball's tube along a path no closed form carries** (#684 D), read at points of
/// the centre's path over the roll, an eighth of the radius apart along it or as near as
/// `MOST_SAMPLES` points come: **B1**, the path turns nowhere tighter than the radius `r` (the tube
/// would fold) and never stops; **B2**, the path comes back within the ball's diameter of itself
/// nowhere it has run half a turn of a circle of the radius from (the tube would cross itself; read
/// with the points' spacing to spare). Sampled, so a turn or a return between the points is not
/// seen: the gate holds what is built to the sweep's field. The frame square to the path where it
/// starts, or `None` where the centre never moves.
fn rows(path: &Centre, r: f64, [from, to]: [f64; 2]) -> Result<Option<Frame>, Outcome> {
    let failed = |e: String| Outcome::Failed(e);
    let refused = |condition, message, witness| Outcome::Refused(Refusal { condition, message, witness });
    let span = to - from;
    let read = |n: usize| -> Result<Vec<(f64, V, V)>, Outcome> {
        (0..=n).map(|i| {
            let s = from + span * i as f64 / n as f64;
            path.at(s).map(|(p, v)| (s, p, v)).map_err(failed)
        }).collect()
    };
    // the path's length, from a first reading, then the points an eighth of the radius apart
    let length = |pts: &[(f64, V, V)]| -> Vec<f64> {
        let mut along = vec![0.0];
        for w in pts.windows(2) {
            let ds = w[1].0 - w[0].0;
            along.push(along.last().unwrap() + 0.5 * ds * (norm(w[0].2) + norm(w[1].2)));
        }
        along
    };
    let first = read(SAMPLES)?;
    let total = *length(&first).last().unwrap();
    if total <= 1e-12 * r.max(1.0) {
        return Ok(None);
    }
    let n = ((8.0 * total / r).ceil() as usize).clamp(SAMPLES, MOST_SAMPLES);
    let pts = if n == SAMPLES { first } else { read(n)? };
    let along = length(&pts);
    let spacing = along.windows(2).map(|w| w[1] - w[0]).fold(0.0, f64::max);
    let said = format!("read at {} points of its path", pts.len());
    // B1: its radius of curvature at each point, its rate of turning read a sixteenth of a spacing on
    let dh = span / n as f64 / 16.0;
    let mut start = None;
    for (i, &(s, p, v)) in pts.iter().enumerate() {
        let speed = norm(v);
        if speed <= 1e-12 * total / span {
            return Err(refused(Condition::Curvature, format!("{} ({said})", stops(s)), Some(p)));
        }
        let (a, b) = if i == 0 { (s, s + dh) } else if i == pts.len() - 1 { (s - dh, s) } else { (s - dh, s + dh) };
        let (va, vb) = (path.at(a).map_err(failed)?.1, path.at(b).map_err(failed)?.1);
        let rate = scale(sub(vb, va), 1.0 / (b - a));
        let bend = norm(cross(v, rate));
        if bend > 0.0 && speed.powi(3) / bend <= r * (1.0 + 1e-9) {
            return Err(refused(Condition::Curvature, format!("its centre's path turns on a radius of {} at {} of \
                the roll, within its radius {} ({said})", num(speed.powi(3) / bend), num(s), num(r)), Some(p)));
        }
        // the frame where the path starts: its seam off the path's own normal, read at the first
        // point it turns at (any square to it where it never does)
        let t = scale(v, 1.0 / speed);
        if start.is_none() && bend > 1e-9 * speed * speed {
            let t0 = normalised(pts[0].2).unwrap();
            let normal = sub(rate, scale(t, dot(rate, t)));
            if let Some(n0) = normalised(sub(normal, scale(t0, dot(normal, t0)))) {
                start = Some((t0, n0));
            }
        }
    }
    // B2: two points of the path within the diameter (and the spacing) of each other, half a turn
    // on a circle of the radius or more apart along it; the points binned in cubes of that reach
    let reach = 2.0 * r + spacing;
    let cube = |p: V| p.map(|x| (x / reach).floor() as i64);
    let mut bins: std::collections::BTreeMap<[i64; 3], Vec<usize>> = Default::default();
    for (i, &(_, p, _)) in pts.iter().enumerate() {
        bins.entry(cube(p)).or_default().push(i);
    }
    for (i, &(s, p, _)) in pts.iter().enumerate() {
        let k = cube(p);
        for dk in 0..27 {
            let key = [k[0] + dk % 3 - 1, k[1] + dk / 3 % 3 - 1, k[2] + dk / 9 - 1];
            for &j in bins.get(&key).map_or(&[][..], |v| &v[..]) {
                let (sj, q, _) = pts[j];
                if j > i && along[j] - along[i] >= PI * r && norm(sub(p, q)) < reach {
                    return Err(refused(Condition::Return, format!("its centre's path comes back {} from itself \
                        between {} and {} of the roll, within its diameter {} and the {} its points stand apart \
                        ({said})", num(norm(sub(p, q))), num(s), num(sj), num(2.0 * r), num(spacing)),
                        Some(scale(add(p, q), 0.5))));
                }
            }
        }
    }
    let t0 = normalised(pts[0].2).unwrap();
    let (t0, n0) = start.unwrap_or_else(|| (t0, crate::brep::geom::Frame::about([0.0; 3], t0).x));
    let (sn, cs) = SEAM.dsin_cos();
    let x = add(scale(n0, cs), scale(cross(t0, n0), sn));
    Ok(Some([t0, x, cross(t0, x)]))
}

/// How far round from the path's own normal a fitted tube's seam runs (radians): `brep::build`'s
/// angle for a turn's tube, the golden angle's fraction of a turn, which no symmetry of a design's
/// makes.
const SEAM: f64 = 0.6180339887498949;
