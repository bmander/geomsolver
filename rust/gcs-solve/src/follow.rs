//! **A drawing followed along a parameter** (#391 S4): continuation.
//!
//! A relation that reads a declared parameter (`param t: Angle over (0deg, 360deg)`, `ground
//! angle(t) line(a, b)`) holds for every value of it, and so does every other relation of the
//! drawing — one that reads no parameter holds of geometry that does not move, or moves with what
//! does — so the drawing is a function of `t`.  It is drawn where the parameter is held
//! (`Sketch::held`), its interval's start, and is **continued** from there to the end: each station
//! solved warm from where the stations before it carry it, so the branch the drawing stands on at
//! its start (a linkage's assembly mode) is the one carried.  It is the face family's march
//! (`solid::family`), its stations whole drawings.
//!
//! **The contract** (the epic's open question 6):
//!
//! - *Predictor and corrector*: a station starts where the secant through the two before it
//!   carries it (the first, where the drawing stands), and is the drawing's own solve from there.
//! - *A step is kept* when it solves, when the solve moved the predicted station no further than
//!   `TURN` of the step's own motion (the secant turned by about that many radians at most: flat
//!   enough to draw, and too little to have reached another branch), and when the drawing's
//!   Jacobian keeps its **orientation** (the sign of its determinant, where the drawing has as many
//!   equations as unknowns): an assembly mode keeps its orientation, and a step that changed it
//!   either jumped to another branch or crossed a singular instant.  A step not kept is halved; one
//!   kept easily (`EASY`) is lengthened by `GROW`, up to `COARSEST` of the interval.  Motion is
//!   measured in world length (`Param::scale`), over the drawing's unknowns.
//! - *A singular instant* is where two branches meet: no step past it can be kept however short
//!   (`FINEST` of the interval) — the solve fails, or the orientation flips — or a station kept is
//!   short of rank.  What it is is read past it (`PROBE` of the interval, `beyond`): where no
//!   drawing stands there the motion turns back (`Kind::Limit`, a linkage at a dead point of its
//!   driver), and where one does the branches cross (`Kind::Branch`, a linkage that folds flat).
//!   The march stops at the last station that stands and says so: past a singular instant which
//!   branch goes on is no longer the drawing's to say.  A turn the solve makes at the finest step
//!   is kept: the stations close in on a limit until none stands past it.
//! - *Held*: the drawing must be held where it starts — of full column rank there.  A freedom
//!   left at an instant is a free *function* of the parameter, and nothing says how it moves as
//!   the parameter runs, so it is refused naming the count (`not_held`).
//! - *Still* (#498): what the parameter does not reach — the blocks of the drawing's order no
//!   reader's row is in or reads through — is set aside at every station and keeps its numbers to
//!   the bit (`Still`): only what moves is solved.
//! - *Implied* (#591): a statement the others already imply — a row with no column of its own,
//!   the over part of the drawing's order, holding at home — is left out of every station solve
//!   (`implied`) and judged where each station ends: a station where it does not hold does not
//!   stand.  Solved, it made the drawing's order not square, which read every station's rank whole
//!   and set nothing aside.

use std::collections::{BTreeMap, BTreeSet};
use std::rc::Rc;

use crate::constraints::Constraint;
use crate::model::{Running, Sketch};
use crate::units::Units;
use crate::vocab::num;
use crate::system::System;

thread_local! {
    /// Whether this thread is in `followed`'s march.
    static MARCHING: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Clears `MARCHING` however the march ends (the ABI catches a panic).
struct Marching;

impl Drop for Marching {
    fn drop(&mut self) {
        MARCHING.set(false);
    }
}

/// The first step, the longest, and the shortest, as fractions of the interval.
const FIRST_STEP: f64 = 1.0 / 256.0;
const COARSEST: f64 = 1.0 / 128.0;
pub const FINEST: f64 = 1e-7;

/// How far the solve may move a predicted station, against the step's own motion, before the
/// step is halved; and below what it is lengthened, by how much.
const TURN: f64 = 0.1;
const EASY: f64 = 0.02;
const GROW: f64 = 1.5;

/// How far past two branches meeting a drawing is looked for, as a fraction of the interval, to
/// tell a crossing from a turn.
const PROBE: f64 = 1e-4;

/// Where in its interval a row is asked whether it reads the parameter (`Still::of`), as a
/// fraction of it: the golden section, irrational, so no law periodic in the interval reads there
/// what it reads at its start.
const ELSEWHERE: f64 = 0.618_033_988_749_894_8;

/// The most stations a march keeps before it is refused: a step kept only at the finest, all the
/// way, would be ten million.
const MAX_STATIONS: usize = 1 << 15;

/// The widest block of the Jacobian whose determinant's sign is read (a dense factorisation).
const ORIENTED: usize = 512;

/// One station: the parameter's value there, in the units it is declared in (degrees for an
/// angle), and every parameter of the drawing standing there.
#[derive(Clone, Debug)]
pub struct Station {
    pub at: f64,
    pub x: Vec<f64>,
}

/// Why a march stopped before its interval's end.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// No drawing stands past it: the motion turns back.
    Limit,
    /// The drawing goes on past it on more than one branch: two branches cross.
    Branch,
}

/// A singular instant: where the march stopped (its last station), and why.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Singular {
    pub at: f64,
    pub kind: Kind,
}

impl Singular {
    /// What it is, in words, the parameter's value in the document's `units`.
    pub fn text(&self, over: &Running, units: Units) -> String {
        let at = at_text(over, self.at, units);
        match self.kind {
            Kind::Limit => format!("singular at {at}: no drawing stands past it, the motion turns \
                                    back"),
            Kind::Branch => format!("singular at {at}: two of its branches cross there"),
        }
    }
}

/// **A drawing followed** over its parameter: the stations in order from the interval's start,
/// and, where the march stopped short of its end, the singular instant it stopped at.
#[derive(Clone, Debug)]
pub struct Followed {
    pub over: Running,
    pub stations: Vec<Station>,
    pub singular: Option<Singular>,
    /// The parameter the running number is held in.
    held: u32,
    /// What the parameter does not reach, left where it stands at every instant.
    still: Still,
    /// Whether a row reads the parameter: where none does, only a placement's value (#697), the
    /// stations are the drawing at home and only the held number moves — nothing is solved.
    solved: bool,
}

impl Followed {
    /// The last value of the parameter a station stands at.
    pub fn reached(&self) -> f64 {
        self.stations.last().map_or(self.over.from, |s| s.at)
    }

    /// Stand `sk` (the drawing followed, or a copy) at `t`, on the branch the march carried:
    /// solved warm from where the stations on either side carry it.  `t` must lie in what was
    /// reached.
    pub fn put(&self, sk: &mut Sketch, t: f64) -> Result<(), String> {
        if !(self.over.from..=self.reached()).contains(&t) {
            return Err(format!("{} is not reached", num(t)));
        }
        let k = self.stations.partition_point(|s| s.at <= t).max(1) - 1;
        let a = &self.stations[k];
        if a.at == t || k + 1 == self.stations.len() || !self.solved {
            // the station itself, its texts worked out there: what a station's solve read, and
            // where a placement's value puts what is drawn in its frame
            sk.set_x(&a.x);
            sk.params[self.held as usize].value = t;
            let _ = crate::expr::evaluate(sk);
            return Ok(());
        }
        let b = &self.stations[k + 1];
        let w = (t - a.at) / (b.at - a.at);
        let x: Vec<f64> = a.x.iter().zip(&b.x).map(|(p, q)| p + w * (q - p)).collect();
        sk.set_x(&x);
        settle(sk, self.held, t, &[], &self.still).map(|_| ())
    }
}

/// **Where a march stands the drawing** (#398 L, `Sketch::standing`): the parameter and its value
/// there, and the drawing at home — every parameter and every constraint's numbers as they were
/// before it was stood anywhere, which `put` moves (the numbers that read the parameter are worked
/// out again at every instant).
#[derive(Clone, Debug, PartialEq)]
pub struct Standing {
    pub name: String,
    pub at: f64,
    pub home: Vec<f64>,
    numbers: Vec<f64>,
}

/// Every number the drawing's constraints state, in order: what a march is a function of besides
/// where the drawing stands.
fn numbers(sk: &Sketch) -> Vec<f64> {
    use crate::constraints::Arg;
    sk.constraints.iter().flat_map(|c| c.args.iter().filter_map(|a| match a {
        Arg::Num(v) => Some(*v),
        Arg::Expr(e) => Some(e.value),
        _ => None,
    })).collect()
}

/// **Stand the drawing `sk` at `t`** of its declared parameter `name` (#398 L), on the branch its
/// march carried (`Followed::put`), remembering home: what an app shows at an instant of a
/// timeline.  Stood at one parameter, it is first sent home from any other, which stands at its
/// start (each parameter is followed alone).  Where it cannot be stood there it is left at home,
/// and why is said: the drawing is not followed, or `t` was not reached.
pub fn stand(sk: &mut Sketch, name: &str, t: f64) -> Result<(), String> {
    let march = followed(sk, name);
    let f = march.as_ref().as_ref().map_err(|e| e.clone())?;
    home(sk);
    let standing = Standing {
        name: name.to_string(),
        at: t,
        home: sk.get_x(),
        numbers: numbers(sk),
    };
    let put = f.put(sk, t);
    sk.standing = Some(standing);
    if put.is_err() {
        home(sk);
    }
    put
}

/// **An instant of the parameter `name` as written** (#398 L): `90deg` for an angle, `12mm` for a
/// length, in the drawing's units and in the ones a person reads (degrees for an angle), as the
/// interval's ends are — a number, of the parameter's dimension, finite.  What a drawing's `model m
/// from "…" at t = 90deg` and a timeline's readout are read by.
pub fn instant(sk: &Sketch, name: &str, text: &str) -> Result<f64, String> {
    let over = sk.parameters.get(name)
        .ok_or_else(|| format!("`{name}` is no parameter the drawing declares"))?;
    let said = |e: String| format!("`{}`: {e}", text.trim());
    let p = crate::expr::parse_in(text.trim(), sk.units).map_err(said)?;
    let a = crate::expr::eval(&p.body, &Default::default()).map_err(|e| said(e.to_string()))?;
    a.dim.require(over.dim, name).map_err(said)?;
    match a.as_dim(over.dim).number() {
        Some(v) if v.is_finite() => Ok(v),
        _ => Err(said("an instant is a finite number".into())),
    }
}

/// **Send the drawing `sk` home** (#398 L): where it was drawn before it was stood at an instant,
/// its numbers worked out there again.  A drawing standing nowhere is home already.  One grown
/// since it was stood (something drawn while it stood there: a sketch's parameters are only ever
/// appended to) goes home with what was drawn left where it was put; one of fewer parameters is
/// not the drawing home was taken of, and is only forgotten.
pub fn home(sk: &mut Sketch) {
    let Some(s) = sk.standing.take() else { return };
    let mut x = sk.get_x();
    let Some(prefix) = x.get_mut(..s.home.len()) else { return };
    prefix.copy_from_slice(&s.home);
    sk.set_x(&x);
    // the texts that read the parameter, read at its start again
    let _ = crate::expr::evaluate(sk);
}

/// **A drag along a march** (#398 L): a point of a followed drawing taken hold of, and the
/// instant found where it comes nearest the pointer — the drawing driven by where its point is
/// put, the way a crank is turned by its pin.  The point's track (where it stands at each station,
/// seen where the pointer is read: in its own plane, or on the eye's picture plane for a point in
/// space) is read once; each move walks it from the instant shown, downhill in distance to the
/// pointer, so a track crossing itself is not jumped across and the instant found is one the
/// march reached.  Over a whole turn of an angle whose track closes the walk goes on round.  The
/// chord walked to is a first guess, and the instant is the nearest on the parabola in the
/// parameter through its stations and the next: no solve, and at `COARSEST` a step a ten-thousandth
/// of a degree from the track's own on a crank.
#[derive(Clone, Debug)]
pub struct Scrub {
    pub name: String,
    track: Vec<(f64, [f64; 2])>,
    /// The segment of the track the instant shown lies on.
    at: usize,
    round: bool,
}

impl Scrub {
    /// Take hold of point `p` of `sk`, followed along `name`, read in its own plane or, with
    /// `eye` (bearing and elevation, radians), where the eye sees it — or why it is not dragged
    /// along the march: the drawing is not followed, or the point does not move as it runs.
    pub fn new(sk: &Sketch, name: &str, p: usize, eye: Option<(f64, f64)>)
        -> Result<Scrub, String> {
        let march = followed(sk, name);
        let f = march.as_ref().as_ref().map_err(|e| e.clone())?;
        if p >= sk.points.len() {
            return Err(format!("there is no point {p}"));
        }
        let mut sketch = sk.clone();
        let eye = eye.map(|(az, el)| crate::space::eye(az, el));
        let track: Vec<(f64, [f64; 2])> = f.stations.iter().map(|s| {
            sketch.set_x(&s.x);
            (s.at, seen(&sketch, p, eye))
        }).collect();
        let reach = track.iter().map(|(_, q)| q[0].hypot(q[1])).fold(0.0, f64::max);
        let moves = track.windows(2).any(|w| {
            (w[1].1[0] - w[0].1[0]).hypot(w[1].1[1] - w[0].1[1]) > 1e-9 * (1.0 + reach)
        });
        if track.len() < 2 || !moves {
            return Err(format!("it does not move as `{name}` runs"));
        }
        let shown = sk.standing.as_ref().filter(|s| s.name == name).map_or(f.over.from, |s| s.at);
        let at = track.partition_point(|s| s.0 <= shown).clamp(1, track.len() - 1) - 1;
        let (a, b) = (track[0].1, track[track.len() - 1].1);
        let whole_turn = f.over.turns() && (f.over.to - f.over.from - 360.0).abs() < 1e-9;
        let round = whole_turn && f.singular.is_none()
            && (a[0] - b[0]).hypot(a[1] - b[1]) <= 1e-6 * (1.0 + reach);
        Ok(Scrub { name: name.to_string(), track, at, round })
    }

    /// The instant at which the point comes nearest (`x`, `y`), walked to from the one before.
    pub fn to(&mut self, x: f64, y: f64) -> f64 {
        let n = self.track.len() - 1;
        // how far the pointer is from segment `k`'s chord
        let seg = |k: usize| -> f64 {
            let ((_, a), (_, b)) = (self.track[k], self.track[k + 1]);
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let len = dx * dx + dy * dy;
            let w = if len > 0.0 {
                (((x - a[0]) * dx + (y - a[1]) * dy) / len).clamp(0.0, 1.0)
            } else {
                0.0
            };
            (x - a[0] - w * dx).hypot(y - a[1] - w * dy)
        };
        let step = |k: usize, up: bool| -> Option<usize> {
            match (up, self.round) {
                (true, _) if k + 1 < n => Some(k + 1),
                (true, true) => Some(0),
                (false, _) if k > 0 => Some(k - 1),
                (false, true) => Some(n - 1),
                _ => None,
            }
        };
        let mut k = self.at;
        // downhill, one way and then the other, and never round more than once
        for up in [true, false] {
            for _ in 0..n {
                match step(k, up) {
                    Some(j) if seg(j) < seg(k) => k = j,
                    _ => break,
                }
            }
        }
        // the segment walked to or one either side: seen from outside a bend a chord's corner is
        // nearer than its middle, so the walk may stop a segment short
        let (mut best, mut t) = (f64::INFINITY, self.track[k].0);
        for j in [step(k, false), Some(k), step(k, true)].into_iter().flatten() {
            let (u, d) = self.nearest_on(j, n, x, y);
            if d < best {
                (best, t, self.at) = (d, u, j);
            }
        }
        t
    }

    /// The instant on segment `k` of the track (of `n`) nearest (`x`, `y`), and how near: on the
    /// parabola in the parameter through the segment's stations and the next, Lagrange's.
    fn nearest_on(&self, k: usize, n: usize, x: f64, y: f64) -> (f64, f64) {
        let (ta, tb) = (self.track[k].0, self.track[k + 1].0);
        let j = if k + 2 <= n { k + 2 } else { k.saturating_sub(1) };
        let ps = [self.track[k], self.track[k + 1], self.track[j]];
        // the place, the velocity and the acceleration at `t`
        let at = |t: f64| -> [[f64; 2]; 3] {
            let mut out = [[0.0; 2]; 3];
            if j == k {
                // two stations only: the chord
                let w = (t - ta) / (tb - ta);
                for c in 0..2 {
                    out[0][c] = ps[0].1[c] + w * (ps[1].1[c] - ps[0].1[c]);
                    out[1][c] = (ps[1].1[c] - ps[0].1[c]) / (tb - ta);
                }
                return out;
            }
            for (i, &(ti, qi)) in ps.iter().enumerate() {
                let (a, b) = match i {
                    0 => (ps[1].0, ps[2].0),
                    1 => (ps[0].0, ps[2].0),
                    _ => (ps[0].0, ps[1].0),
                };
                let den = (ti - a) * (ti - b);
                let (w, dw, ddw) = ((t - a) * (t - b) / den, (2.0 * t - a - b) / den, 2.0 / den);
                for c in 0..2 {
                    out[0][c] += w * qi[c];
                    out[1][c] += dw * qi[c];
                    out[2][c] += ddw * qi[c];
                }
            }
            out
        };
        let far = |t: f64| {
            let q = at(t)[0];
            (q[0] - x).hypot(q[1] - y)
        };
        // Newton's from the nearest of the ends and the middle, kept to the segment
        let mut t = [ta, 0.5 * (ta + tb), tb].into_iter()
            .min_by(|&a, &b| far(a).total_cmp(&far(b))).unwrap_or(ta);
        for _ in 0..8 {
            let [q, d, dd] = at(t);
            let r = [q[0] - x, q[1] - y];
            let g = r[0] * d[0] + r[1] * d[1];
            let h = d[0] * d[0] + d[1] * d[1] + r[0] * dd[0] + r[1] * dd[1];
            if h <= 0.0 {
                break;
            }
            let next = (t - g / h).clamp(ta, tb);
            let done = (next - t).abs() <= 1e-12 * (tb - ta);
            t = next;
            if done {
                break;
            }
        }
        (t, far(t))
    }
}

/// Where point `p` of `sk` is seen: in its own plane, or on the eye's picture plane (its right and
/// up) for a point in space.
fn seen(sk: &Sketch, p: usize, eye: Option<([f64; 3], [f64; 3])>) -> [f64; 2] {
    match eye {
        Some((r, u)) => {
            let w = sk.world_point(p);
            let dot = |a: [f64; 3]| a[0] * w[0] + a[1] * w[1] + a[2] * w[2];
            [dot(r), dot(u)]
        }
        None => {
            let (x, y) = sk.point_xy(p);
            [x, y]
        }
    }
}

/// The drawing `sk` followed along its declared parameter `name`, remembered against every number
/// of the drawing and of its statements, and the parameter's interval (`Caches::followed`).
pub fn followed(sk: &Sketch, name: &str) -> Rc<Result<Followed, String>> {
    // a march's own station solve asks nothing of a march: a question that would reach one from
    // inside it (a curve drawn from the drawing being followed) is answered that there is none
    if MARCHING.get() {
        return Rc::new(Err("the drawing is being followed".into()));
    }
    let key = march_key(sk, name);
    if let Some((k, f)) = sk.caches.followed.borrow().get(name) {
        if *k == key {
            return f.clone();
        }
    }
    let f = {
        MARCHING.set(true);
        let _marching = Marching;
        // said to whoever listens, how far it has got on its gauge: a long march is no hang
        let following = || crate::progress::Work::follow(name);
        Rc::new(crate::progress::during(following, || follow(sk, name)))
    };
    sk.caches.followed.borrow_mut().insert(name.to_string(), (key, f.clone()));
    f
}

/// What the march along `name` is a function of: the drawing at home, wherever it is shown
/// standing (a march is of the drawing, not of an instant of it), the parameter's interval, how
/// many placements read it, and every number the drawing's statements state.
pub(crate) fn march_key(sk: &Sketch, name: &str) -> Vec<f64> {
    let (mut key, numbers) = match &sk.standing {
        Some(s) => (s.home.clone(), s.numbers.clone()),
        None => (sk.get_x(), numbers(sk)),
    };
    key.extend(sk.parameters.get(name).map_or([0.0; 2], |p| [p.from, p.to]));
    key.push(placed(sk, name).len() as f64);
    key.extend(numbers);
    key
}

/// **The drawing `sk` followed along its declared parameter `name`**, from the interval's start
/// to its end or to the singular instant that stops it — or why it cannot be: the parameter is
/// not declared or nothing reads it, the drawing does not solve at the start, or it is not held
/// there.  Where no row reads it and a placement's value does (`placed`, #697) the march is the
/// drawing at home at both ends of the interval: what is drawn in the valued frame moves, and
/// nothing is solved.
pub fn follow(sk: &Sketch, name: &str) -> Result<Followed, String> {
    let over = sk.parameters.get(name)
        .ok_or_else(|| format!("`{name}` is no parameter the drawing declares"))?.clone();
    let reading = readers(sk, name);
    let held = sk.held.get(name).map(|h| h.param)
        .filter(|_| !reading.is_empty() || !placed(sk, name).is_empty())
        .ok_or_else(|| format!("nothing the drawing states reads `{name}`"))?;
    let mut sk = sk.clone();
    home(&mut sk);
    if reading.is_empty() {
        let stations = [over.from, over.to].map(|at| {
            sk.params[held as usize].value = at;
            Station { at, x: sk.get_x() }
        });
        return Ok(Followed { over, stations: stations.into(), singular: None, held,
                             still: Still::default(), solved: false });
    }
    let elsewhere = over.from + (over.to - over.from) * ELSEWHERE;
    let implied = implied(&sk);
    let mut reading = reading;
    loop {
        let still = Still::of(&sk, held, &reading, elsewhere, &implied);
        let followed = march(sk.clone(), &over, held, &still);
        // a row set aside that read otherwise at a station reads the parameter: marched again
        // with it solved, so every orientation is read over the one system
        let missed = still.missed.take();
        if missed.is_empty() {
            return followed;
        }
        reading.extend(missed);
    }
}

/// The march of `follow`, from the drawing `sk` at home, with what `still` sets aside.
fn march(mut sk: Sketch, over: &Running, held: u32, still: &Still) -> Result<Followed, String> {
    let span = over.to - over.from;
    let (coarsest, finest) = (span * COARSEST, span * FINEST);
    let mut sys = settle(&mut sk, held, over.from, &[], still)
        .map_err(|e| format!("the drawing at {}: {e}", at_text(over, over.from, sk.units)))?;
    let jac = Jacobian::read(&mut sys, &sk);
    if let Some(n) = jac.short(&mut sys, &sk) {
        return Err(not_held(over, n, sk.units));
    }
    // a motion too small to judge a turn by: the solve's own reach, in world length
    let floor = 1e-9 * (1.0 + sys.extent);
    let weights: Vec<f64> = sk.params.iter().enumerate()
        .map(|(i, p)| if p.fixed || i == held as usize { 0.0 } else { p.scale })
        .collect();
    let apart = |x: &[f64], y: &[f64]| -> f64 {
        x.iter().zip(y).zip(&weights).map(|((a, b), w)| (w * (a - b)).powi(2)).sum::<f64>().sqrt()
    };
    let mut sign = jac.sign;
    let mut trail = vec![Station { at: over.from, x: sk.get_x() }];
    let mut h = span * FIRST_STEP;
    let stop = |trail: Vec<Station>, kind: Kind| {
        let at = trail.last().map_or(over.from, |s| s.at);
        Ok(Followed { over: over.clone(), stations: trail, singular: Some(Singular { at, kind }),
                      held, still: still.clone(), solved: true })
    };
    loop {
        let prev = trail.last().expect("a march starts somewhere").clone();
        let left = over.to - prev.at;
        if left <= 0.0 {
            break;
        }
        if trail.len() >= MAX_STATIONS {
            return Err(format!("it takes more than {MAX_STATIONS} stations to follow"));
        }
        let step = h.min(left);
        let t = if step >= left { over.to } else { prev.at + step };
        // the solve starts where the last two stations carry it
        let predicted = match trail.as_slice() {
            [.., a, b] => {
                let k = (t - b.at) / (b.at - a.at);
                a.x.iter().zip(&b.x).map(|(p, q)| q + k * (q - p)).collect()
            }
            _ => prev.x.clone(),
        };
        sk.set_x(&predicted);
        let solved = settle(&mut sk, held, t, &[], still);
        if !still.missed.borrow().is_empty() {
            return Err("a row set aside reads the parameter".into());
        }
        let finest_step = step <= finest;
        let Ok(mut sys) = solved else {
            if finest_step {
                return stop(trail, Kind::Limit);
            }
            h = step / 2.0;
            continue;
        };
        let x = sk.get_x();
        let moved = apart(&x, &prev.x);
        let off = apart(&x, &predicted);
        let jac = Jacobian::read(&mut sys, &sk);
        let o = jac.sign;
        if sign != 0 && o != sign {
            if finest_step {
                let kind = beyond(&mut sk, held, still, &prev,
                                  (prev.at + span * PROBE).min(over.to));
                return stop(trail, kind);
            }
            h = step / 2.0;
            continue;
        }
        if trail.len() >= 2 && off > TURN * moved + floor && !finest_step {
            h = step / 2.0;
            continue;
        }
        if jac.short(&mut sys, &sk).is_some() {
            let kind = beyond(&mut sk, held, still, &prev, (t + span * PROBE).min(over.to));
            trail.push(Station { at: t, x });
            return stop(trail, kind);
        }
        sign = o;
        if off <= EASY * moved + floor || trail.len() < 2 {
            h = (step * GROW).min(coarsest);
        }
        crate::progress::gauge(|| format!("now at {} of ({}, {})", reading(over, t, sk.units),
                                          reading(over, over.from, sk.units),
                                          reading(over, over.to, sk.units)));
        trail.push(Station { at: t, x });
    }
    Ok(Followed { over: over.clone(), stations: trail, singular: None, held, still: still.clone(),
                  solved: true })
}

/// The ids of the constraints whose numbers read the declared parameter `name` (#391 S4): the
/// relations that say how the drawing moves as it runs.
pub fn readers(sk: &Sketch, name: &str) -> Vec<u32> {
    sk.constraints.iter().filter(|c| reading_slot(sk, c, name).is_some()).map(|c| c.id).collect()
}

/// The solids placed at a value whose text reads the declared parameter `name` (#697), by index:
/// `n := x in f`, `fix(value == t) f` — what moves as it runs where no row of the drawing does.
pub fn placed(sk: &Sketch, name: &str) -> Vec<usize> {
    sk.solids.iter().enumerate().filter(|(_, s)| match &s.def {
        crate::model::SolidDef::Placed { at, .. } => crate::expr::parse_in(&at.text, sk.units)
            .is_ok_and(|p| p.body.deps().iter().any(|d| d == name)),
        _ => false,
    }).map(|(i, _)| i).collect()
}

/// The slot of `c` whose text reads the declared parameter `name`, where one does.
pub fn reading_slot(sk: &Sketch, c: &crate::constraints::Constraint, name: &str)
    -> Option<usize> {
    use crate::constraints::Arg;
    c.args.iter().position(|a| match a {
        Arg::Expr(e) => crate::expr::parse_in(&e.text, sk.units)
            .is_ok_and(|p| p.body.deps().iter().any(|d| d == name)),
        _ => false,
    })
}

/// The parameter's value as a reader reads it: `t = 30deg`.
fn at_text(over: &Running, v: f64, units: Units) -> String {
    format!("{} = {}", over.name, reading(over, v, units))
}

/// A value of the parameter `over` as a reader reads it, to the figures any reading is
/// (`expr::READING_SIG`) and in its own units: `129.838deg`.
pub fn reading(over: &Running, v: f64, units: Units) -> String {
    let read = crate::expr::reading(crate::constraints::SpecKind::Scalar, v).parse().unwrap_or(v);
    over.dim.number_text(read, units)
}

/// What two branches meeting just past station `prev` are, by whether a drawing stands at `probe`
/// past them: where one goes on they cross, and where none does the motion turns back.
fn beyond(sk: &mut Sketch, held: u32, still: &Still, prev: &Station, probe: f64) -> Kind {
    sk.set_x(&prev.x);
    if settle(sk, held, probe, &[], still).is_ok() { Kind::Branch } else { Kind::Limit }
}

/// Why a drawing short of rank where it starts is not followed.
pub fn not_held(over: &Running, n: usize, units: Units) -> String {
    let (s, each) = if n == 1 { ("", "it is") } else { ("s", "each is") };
    let (at, t) = (at_text(over, over.from, units), &over.name);
    format!("the drawing is not held at {at}: {n} degree{s} of freedom, and {each} a free function \
             of `{t}`: nothing says how it moves as `{t}` runs")
}

/// Solve the drawing with its running number held at `t`, warm from where it stands, the
/// unknowns named `outer` held as they stand and what `still` sets aside left where it stands:
/// the compiled system, or why it does not solve.  The one station solve of a march along a
/// number, a declared parameter's here and a face family's (`solid::family`), whose station holds
/// the drawing's unknowns it was handed.
pub(crate) fn settle(sk: &mut Sketch, held: u32, t: f64, outer: &[String], still: &Still)
    -> Result<System, String> {
    sk.params[held as usize].value = t;
    for item in crate::expr::evaluate(sk) {
        if let Some(e) = item.error {
            return Err(format!("`{}` does not work out there: {e}", item.text));
        }
    }
    // after the texts are worked out, which free and hold the unknowns they read
    for n in outer {
        if let Some(&p) = sk.free_vars.get(n) {
            sk.params[p as usize].fixed = true;
        }
    }
    let aside = still.aside(sk);
    let mut sys = System::new(sk);
    let solved = sys.solve(sk, crate::solve::SolveOpts::default()).success;
    aside.give_back(sk);
    if !solved {
        return Err("does not solve".into());
    }
    if !still.implied_hold(sk) {
        return Err("a statement implied where the drawing starts does not hold".into());
    }
    Ok(sys)
}

/// **The statements the rest of `sk` implies** (#591), by id, ascending: rows of the over part
/// of its block-triangular order (`System::block_order`) — which have no column of their own —
/// that hold where it stands and whose rank the rest of that part has without them, so what is
/// left of it is square and of full rank.  Each is a whole statement, the last written tried
/// first.  Carried, such a row left the drawing's order not square, and every station of a march
/// had its rank read whole and nothing set aside (`Still`, `Jacobian`): one row restating what a
/// point's two planes say made a march of a few hundred unknowns a hundred times slower.  None
/// where the order has an under part (the march refuses it, `not_held`), where its over part is
/// short of rank or wider than `ORIENTED`, or where no choice of whole statements squares it.
fn implied(sk: &Sketch) -> Vec<u32> {
    let mut sys = System::new(sk);
    let order = sys.block_order();
    let n = order.over_cols.len();
    if order.over_rows.is_empty() || !order.under_rows.is_empty() || !order.under_cols.is_empty()
        || n > ORIENTED {
        return Vec::new();
    }
    let z = sys.z0(sk);
    let r = sys.residuals(&z);
    sys.size_rows();
    let acceptance = crate::solve::SolveOpts::default().acceptance_tol;
    if order.over_rows.iter().any(|&row| !(sys.judged(row, r[row]) < acceptance)) {
        return Vec::new();
    }
    let j = sys.conditioned_sparse(&z);
    let tol = crate::linalg::Tol::Abs(crate::system::RANK_TOL);
    let rank = |rows: &[usize]| crate::linalg::rrqr_with(&j.dense(rows, &order.over_cols), tol).0;
    if rank(&order.over_rows) < n {
        return Vec::new();
    }
    // each statement by its id, the last written first
    let mut statements: Vec<(u32, (usize, usize))> = order.over_rows.iter().map(|&row| {
        let (b, i) = sys.instance_of(row);
        (sys.blocks[b].cids[i], (b, i))
    }).collect();
    statements.sort_unstable_by(|a, b| b.cmp(a));
    statements.dedup();
    let (mut rows, mut out) = (order.over_rows.clone(), Vec::new());
    for (cid, inst) in statements {
        if rows.len() == n {
            break;
        }
        let own = sys.rows_of(&[inst]);
        let rest: Vec<usize> = rows.iter().filter(|r| !own.contains(r)).copied().collect();
        if rank(&rest) == n {
            rows = rest;
            out.push(cid);
        }
    }
    if rows.len() != n {
        return Vec::new();
    }
    out.sort_unstable();
    out
}

/// **What a march leaves where it stands** (#498): the statements and unknowns of the blocks of
/// the drawing's block-triangular order (`System::block_order`) the parameter does not reach —
/// no row of one reads it, nor an unknown of a block that does, and each holds where the drawing
/// stands at home and is of full rank there.  Nothing such a block reads moves as the parameter
/// runs, so its unknowns are where they stand at every instant: each station solves the rest
/// with them set aside (`aside`) and they keep their numbers to the bit.  Solved with the rest,
/// a part that does not move moved by the solve's rounding, and a solid that only moved read as
/// another shape (`solid::ShapeKey`).  A row reads the parameter where its text does, where it
/// reads a held number (worked out where the drawing is settled), and where what it reads — its
/// unknowns' numbers and the constants it compiles to — is otherwise with the parameter held
/// elsewhere (`ELSEWHERE`).  One that still reads otherwise at a station than at home, a constant
/// read off where what moves stands, is not set aside there (`missed`), and the march is made
/// again with it among the readers (`follow`).  Nothing is set aside of a drawing whose order is
/// not square, whose rank is read whole — once what it implies (`implied`) is left out, which
/// every station solve leaves out too, and judges where it ends (`implied_hold`).
#[derive(Clone, Debug, Default)]
pub(crate) struct Still {
    /// The statements set aside, by id, ascending.
    rows: Vec<u32>,
    /// The unknowns their blocks are matched to, ascending.
    params: Vec<u32>,
    /// What each statement reads at home (`read`), by id.
    reads: BTreeMap<u32, Vec<f64>>,
    /// The statements the rest implies (`implied`), by id, ascending: no row of any station's
    /// solve, judged where it ends.
    implied: Vec<u32>,
    /// The statements a station found reading otherwise (`aside`): rows reading the parameter.
    missed: std::cell::RefCell<BTreeSet<u32>>,
}

impl Still {
    /// What the parameter held in `held` and read by the statements `reading` does not reach in
    /// `sk`, at home — nor where it is held at `elsewhere`, where a row reading it some way no
    /// text says reads otherwise — with the statements `implied` left out.
    fn of(sk: &Sketch, held: u32, reading: &[u32], elsewhere: f64, implied: &[u32]) -> Still {
        let only = Still { implied: implied.to_vec(), ..Still::default() };
        let mut left = None;
        let sk: &Sketch = match implied {
            [] => sk,
            _ => {
                let left = left.insert(sk.clone());
                let _ = only.aside(left);
                left
            }
        };
        let mut sys = System::new(sk);
        let order = sys.block_order();
        let square = order.over_rows.is_empty() && order.over_cols.is_empty()
            && order.under_rows.is_empty() && order.under_cols.is_empty();
        if !square || order.blocks.len() < 2 {
            return only;
        }
        let z = sys.z0(sk);
        let r = sys.residuals(&z);
        sys.size_rows();
        let j = sys.conditioned_sparse(&z);
        let acceptance = crate::solve::SolveOpts::default().acceptance_tol;
        let cid = |&(b, i): &(usize, usize)| sys.blocks[b].cids[i];
        let mut there = sk.clone();
        there.params[held as usize].value = elsewhere;
        let _ = crate::expr::evaluate(&mut there);
        let spans = (crate::curve::contact_spans(sk), crate::curve::contact_spans(&there));
        // a held number is written where the drawing is settled, read off where it stands
        let numbers: BTreeSet<u32> = sk.held.values().map(|h| h.param).chain([held]).collect();
        let reads_t: BTreeSet<u32> = sk.constraints.iter().zip(&there.constraints)
            .filter(|(c, d)| reading.contains(&c.id)
                || c.params(sk).iter().any(|p| numbers.contains(p))
                || read(sk, c, &spans.0) != read(&there, d, &spans.1))
            .map(|(c, _)| c.id)
            .collect();
        let mut block = vec![usize::MAX; sys.n_free];
        for (k, b) in order.blocks.iter().enumerate() {
            for &c in &b.cols {
                block[c] = k;
            }
        }
        let mut reached: Vec<bool> = order.blocks.iter().map(|b| {
            let tol = crate::linalg::Tol::Abs(crate::system::RANK_TOL);
            b.instances.iter().any(|i| reads_t.contains(&cid(i)))
                || b.rows.iter().any(|&row| !(sys.judged(row, r[row]) < acceptance))
                || crate::linalg::rrqr_with(&j.dense(&b.rows, &b.cols), tol).0 < b.cols.len()
        }).collect();
        // and every block reading an unknown of one reached, until there is none
        let mut more = true;
        while more {
            more = false;
            for (k, b) in order.blocks.iter().enumerate() {
                if !reached[k] && b.rows.iter()
                    .any(|&row| j.row(row).any(|(c, _)| block[c] != k && reached[block[c]])) {
                    (reached[k], more) = (true, true);
                }
            }
        }
        let still = order.blocks.iter().zip(&reached).filter(|(_, &r)| !r).map(|(b, _)| b);
        let (mut rows, mut params) = (Vec::new(), Vec::new());
        for b in still {
            rows.extend(b.instances.iter().map(cid));
            params.extend(b.cols.iter().map(|&c| sys.free[c] as u32));
        }
        rows.sort_unstable();
        rows.dedup();
        params.sort_unstable();
        let reads = sk.constraints.iter().filter(|c| rows.binary_search(&c.id).is_ok())
            .map(|c| (c.id, read(sk, c, &spans.0)))
            .collect();
        Still { rows, params, reads, ..only }
    }

    /// Set it aside in `sk`, where what it reads is what it read at home: its statements carried
    /// (`Constraint::carried`, which compiles no row) and its unknowns held, and what is implied
    /// carried wherever — what was changed, for `Aside::give_back`.
    fn aside(&self, sk: &mut Sketch) -> Aside {
        let carry = |sk: &Sketch, of: &[u32]| -> Vec<usize> {
            sk.constraints.iter().enumerate()
                .filter(|(_, c)| !c.carried && of.binary_search(&c.id).is_ok())
                .map(|(i, _)| i)
                .collect()
        };
        let (mut rows, mut params) = (carry(sk, &self.implied), Vec::new());
        if !self.rows.is_empty() && !self.misses(sk) {
            rows.extend(carry(sk, &self.rows));
            params = self.params.iter().map(|&p| p as usize)
                .filter(|&p| sk.params.get(p).is_some_and(|q| !q.fixed))
                .collect();
        }
        for &i in &rows {
            sk.constraints[i].carried = true;
        }
        for &p in &params {
            sk.params[p].fixed = true;
        }
        Aside { rows, params }
    }

    /// Whether a statement set aside reads otherwise in `sk` than it read at home: one that does
    /// reads the parameter, and is kept in `missed`.
    fn misses(&self, sk: &Sketch) -> bool {
        let spans = crate::curve::contact_spans(sk);
        let missed: Vec<u32> = sk.constraints.iter()
            .filter(|c| self.reads.get(&c.id).is_some_and(|was| *was != read(sk, c, &spans)))
            .map(|c| c.id)
            .collect();
        let any = !missed.is_empty();
        self.missed.borrow_mut().extend(missed);
        any
    }

    /// Whether every statement the rest implies holds where `sk` stands, each row judged as a
    /// solve judges it (`System::max_relative_residual`): compiled alone, the rest carried.
    fn implied_hold(&self, sk: &mut Sketch) -> bool {
        if self.implied.is_empty() {
            return true;
        }
        let was: Vec<bool> = sk.constraints.iter().map(|c| c.carried).collect();
        for c in &mut sk.constraints {
            c.carried = c.carried || self.implied.binary_search(&c.id).is_err();
        }
        let mut sys = System::new(sk);
        let z = sys.z0(sk);
        let acceptance = crate::solve::SolveOpts::default().acceptance_tol;
        let holds = sys.max_relative_residual(&z) < acceptance;
        for (c, w) in sk.constraints.iter_mut().zip(was) {
            c.carried = w;
        }
        holds
    }
}

/// What constraint `c` of `sk` reads: its unknowns' numbers and the constants it compiles to
/// (`System::new`'s, on the span `spans` says).
fn read(sk: &Sketch, c: &Constraint, spans: &BTreeMap<u32, usize>) -> Vec<f64> {
    let span = spans.get(&c.id).copied();
    let mut v: Vec<f64> = c.params_on(sk, span).iter().map(|&p| sk.params[p as usize].value)
        .collect();
    v.extend(c.consts_on(sk, span));
    v
}

/// What `Still::aside` changed: the statements it carried and the unknowns it held, by index.
#[derive(Default)]
struct Aside {
    rows: Vec<usize>,
    params: Vec<usize>,
}

impl Aside {
    /// Give `sk` back what was set aside: its statements' rows and its unknowns' freedom.
    fn give_back(self, sk: &mut Sketch) {
        for i in self.rows {
            sk.constraints[i].carried = false;
        }
        for p in self.params {
            sk.params[p].fixed = false;
        }
    }
}

/// How many degrees of freedom the solved system `sys` leaves, where it leaves any: a station
/// short of rank.
pub(crate) fn short(sys: &mut System, sk: &Sketch) -> Option<usize> {
    Jacobian::read(sys, sk).short(sys, sk)
}

/// **The drawing's Jacobian where it stands, read a block at a time** (#398 L): the square
/// blocks of its block-triangular order (`System::block_order`), each conditioned (by positive
/// scales, which keep a determinant's sign).  A block-triangular matrix is singular exactly
/// where a block on its diagonal is, and its determinant is theirs times the sign of the order,
/// which is the same at every station of a march (each compiles the same structure) — so the
/// rank and the orientation are a few small factorisations rather than one of the whole drawing,
/// which on a drawing of a thousand unknowns was the march's whole cost.  A drawing whose order
/// is not square (an over or an under part) is read whole.
struct Jacobian {
    /// Every block of full rank: the drawing is held.  `false` asks the whole matrix.
    held: bool,
    /// The sign of the determinant, up to the order's: `0` where it is singular, not square, or
    /// has a block wider than `ORIENTED`, so no orientation is asked of it.
    sign: i8,
}

impl Jacobian {
    fn read(sys: &mut System, sk: &Sketch) -> Jacobian {
        let order = sys.block_order();
        let square = order.over_rows.is_empty() && order.over_cols.is_empty()
            && order.under_rows.is_empty() && order.under_cols.is_empty();
        if !square || sys.n_free == 0 {
            return Jacobian { held: false, sign: 0 };
        }
        let z = sys.z0(sk);
        let j = sys.conditioned_sparse(&z);
        let mut sign = 1;
        for b in &order.blocks {
            let m = j.dense(&b.rows, &b.cols);
            let n = b.cols.len();
            let tol = crate::linalg::Tol::Abs(crate::system::RANK_TOL);
            if crate::linalg::rrqr_with(&m, tol).0 < n {
                return Jacobian { held: false, sign: 0 };
            }
            sign *= if n > ORIENTED { 0 } else { det_sign(n, m.data) };
        }
        Jacobian { held: true, sign }
    }

    /// How many degrees of freedom the drawing leaves, where it leaves any: none where every
    /// block is of full rank, and otherwise the whole matrix's rank says how many.
    fn short(&self, sys: &mut System, sk: &Sketch) -> Option<usize> {
        if self.held {
            return None;
        }
        let z = sys.z0(sk);
        let rank = sys.rank(&z, crate::system::RANK_TOL, true);
        (rank < sys.n_free).then(|| sys.n_free - rank)
    }
}

/// The sign of the determinant of the `n`×`n` matrix `a` (row-major), `0` where it is singular.
fn det_sign(n: usize, mut a: Vec<f64>) -> i8 {
    let mut piv = Vec::with_capacity(n);
    if !crate::linalg::lu_factor(n, &mut a, &mut piv) {
        return 0;
    }
    crate::linalg::lu_det_sign(n, &a, &piv)
}
