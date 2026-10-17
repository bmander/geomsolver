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

use std::rc::Rc;

use crate::model::{Running, Sketch};
use crate::units::Units;
use crate::syntax::num;
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

/// The most stations a march keeps before it is refused: a step kept only at the finest, all the
/// way, would be ten million.
const MAX_STATIONS: usize = 1 << 15;

/// The most unknowns whose Jacobian's orientation is read (a dense factorisation a station).
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
        if a.at == t || k + 1 == self.stations.len() {
            sk.set_x(&a.x);
            return Ok(());
        }
        let b = &self.stations[k + 1];
        let w = (t - a.at) / (b.at - a.at);
        let x: Vec<f64> = a.x.iter().zip(&b.x).map(|(p, q)| p + w * (q - p)).collect();
        sk.set_x(&x);
        settle(sk, self.held, t, &[]).map(|_| ())
    }
}

/// The drawing `sk` followed along its declared parameter `name`, remembered against every number
/// of the drawing and of its statements, and the parameter's interval (`Caches::followed`).
pub fn followed(sk: &Sketch, name: &str) -> Rc<Result<Followed, String>> {
    use crate::constraints::Arg;
    // a march's own station solve asks nothing of a march: a question that would reach one from
    // inside it (a curve drawn from the drawing being followed) is answered that there is none
    if MARCHING.get() {
        return Rc::new(Err("the drawing is being followed".into()));
    }
    let mut key = sk.get_x();
    key.extend(sk.parameters.get(name).map_or([0.0; 2], |p| [p.from, p.to]));
    key.extend(sk.constraints.iter().flat_map(|c| c.args.iter().filter_map(|a| match a {
        Arg::Num(v) => Some(*v),
        Arg::Expr(e) => Some(e.value),
        _ => None,
    })));
    if let Some((k, f)) = sk.caches.followed.borrow().get(name) {
        if *k == key {
            return f.clone();
        }
    }
    let f = {
        MARCHING.set(true);
        let _marching = Marching;
        Rc::new(follow(sk, name))
    };
    sk.caches.followed.borrow_mut().insert(name.to_string(), (key, f.clone()));
    f
}

/// **The drawing `sk` followed along its declared parameter `name`**, from the interval's start
/// to its end or to the singular instant that stops it — or why it cannot be: the parameter is
/// not declared or nothing reads it, the drawing does not solve at the start, or it is not held
/// there.
pub fn follow(sk: &Sketch, name: &str) -> Result<Followed, String> {
    let over = sk.parameters.get(name)
        .ok_or_else(|| format!("`{name}` is no parameter the drawing declares"))?.clone();
    let held = sk.held.get(name).map(|h| h.param)
        .filter(|_| !readers(sk, name).is_empty())
        .ok_or_else(|| format!("nothing the drawing states reads `{name}`"))?;
    let mut sk = sk.clone();
    let span = over.to - over.from;
    let (coarsest, finest) = (span * COARSEST, span * FINEST);
    let mut sys = settle(&mut sk, held, over.from, &[])
        .map_err(|e| format!("the drawing at {}: {e}", at_text(&over, over.from, sk.units)))?;
    if let Some(n) = short(&mut sys, &sk) {
        return Err(not_held(&over, n, sk.units));
    }
    // a motion too small to judge a turn by: the solve's own reach, in world length
    let floor = 1e-9 * (1.0 + sys.extent);
    let weights: Vec<f64> = sk.params.iter().enumerate()
        .map(|(i, p)| if p.fixed || i == held as usize { 0.0 } else { p.scale })
        .collect();
    let apart = |x: &[f64], y: &[f64]| -> f64 {
        x.iter().zip(y).zip(&weights).map(|((a, b), w)| (w * (a - b)).powi(2)).sum::<f64>().sqrt()
    };
    let mut sign = orientation(&mut sys, &sk);
    let mut trail = vec![Station { at: over.from, x: sk.get_x() }];
    let mut h = span * FIRST_STEP;
    let stop = |trail: Vec<Station>, kind: Kind| {
        let at = trail.last().map_or(over.from, |s| s.at);
        Ok(Followed { over: over.clone(), stations: trail, singular: Some(Singular { at, kind }),
                      held })
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
        let solved = settle(&mut sk, held, t, &[]);
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
        let o = orientation(&mut sys, &sk);
        if sign != 0 && o != sign {
            if finest_step {
                let kind = beyond(&mut sk, held, &prev, (prev.at + span * PROBE).min(over.to));
                return stop(trail, kind);
            }
            h = step / 2.0;
            continue;
        }
        if trail.len() >= 2 && off > TURN * moved + floor && !finest_step {
            h = step / 2.0;
            continue;
        }
        if short(&mut sys, &sk).is_some() {
            let kind = beyond(&mut sk, held, &prev, (t + span * PROBE).min(over.to));
            trail.push(Station { at: t, x });
            return stop(trail, kind);
        }
        sign = o;
        if off <= EASY * moved + floor || trail.len() < 2 {
            h = (step * GROW).min(coarsest);
        }
        trail.push(Station { at: t, x });
    }
    Ok(Followed { over, stations: trail, singular: None, held })
}

/// The ids of the constraints whose numbers read the declared parameter `name` (#391 S4): the
/// relations that say how the drawing moves as it runs.
pub fn readers(sk: &Sketch, name: &str) -> Vec<u32> {
    sk.constraints.iter().filter(|c| reading_slot(sk, c, name).is_some()).map(|c| c.id).collect()
}

/// The slot of `c` whose text reads the declared parameter `name`, where one does.
pub(crate) fn reading_slot(sk: &Sketch, c: &crate::constraints::Constraint, name: &str)
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
/// (`io::READING_SIG`) and in its own units: `129.838deg`.
pub fn reading(over: &Running, v: f64, units: Units) -> String {
    let read = crate::io::reading(crate::constraints::SpecKind::Scalar, v).parse().unwrap_or(v);
    over.dim.number_text(read, units)
}

/// What two branches meeting just past station `prev` are, by whether a drawing stands at `probe`
/// past them: where one goes on they cross, and where none does the motion turns back.
fn beyond(sk: &mut Sketch, held: u32, prev: &Station, probe: f64) -> Kind {
    sk.set_x(&prev.x);
    if settle(sk, held, probe, &[]).is_ok() { Kind::Branch } else { Kind::Limit }
}

/// Why a drawing short of rank where it starts is not followed.
pub fn not_held(over: &Running, n: usize, units: Units) -> String {
    let (s, each) = if n == 1 { ("", "it is") } else { ("s", "each is") };
    let (at, t) = (at_text(over, over.from, units), &over.name);
    format!("the drawing is not held at {at}: {n} degree{s} of freedom, and {each} a free function \
             of `{t}`: nothing says how it moves as `{t}` runs")
}

/// Solve the drawing with its running number held at `t`, warm from where it stands, the
/// unknowns named `outer` held as they stand: the compiled system, or why it does not solve.  The
/// one station solve of a march along a number, a declared parameter's here and a face family's
/// (`solid::family`), whose station holds the drawing's unknowns it was handed.
pub(crate) fn settle(sk: &mut Sketch, held: u32, t: f64, outer: &[String])
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
    let mut sys = System::new(sk);
    if !sys.solve(sk, crate::solve::SolveOpts::default()).success {
        return Err("does not solve".into());
    }
    Ok(sys)
}

/// How many degrees of freedom the solved system `sys` leaves, where it leaves any: a station
/// short of rank.
pub(crate) fn short(sys: &mut System, sk: &Sketch) -> Option<usize> {
    let z = sys.z0(sk);
    let rank = sys.rank(&z, crate::system::RANK_TOL, true);
    (rank < sys.n_free).then(|| sys.n_free - rank)
}

/// The sign of the determinant of the drawing's Jacobian where it stands, its rows and columns
/// conditioned (by positive scales, which keep the sign): `0` where it is singular, and where it
/// is not square or too large to factor densely, so no orientation is asked of it.
fn orientation(sys: &mut System, sk: &Sketch) -> i8 {
    let z = sys.z0(sk);
    let j = sys.conditioned(&z);
    let n = j.cols();
    if n == 0 || j.rows() != n || n > ORIENTED {
        return 0;
    }
    let m = j.as_mat();
    let mut a: Vec<f64> = (0..n).flat_map(|r| m.row(r).to_vec()).collect();
    let mut piv = Vec::with_capacity(n);
    if !crate::linalg::lu_factor(n, &mut a, &mut piv) {
        return 0;
    }
    let swaps = piv.iter().enumerate().filter(|&(k, &p)| p != k).count();
    let negative = (0..n).filter(|&k| a[k * n + k] < 0.0).count();
    if (swaps + negative) % 2 == 0 { 1 } else { -1 }
}
