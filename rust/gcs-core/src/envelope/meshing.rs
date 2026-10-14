//! **The meshing of two flanks** (#283, step 3): tooth contact analysis as a locus.
//!
//! `contact := meshing(pinion_flank, gear_flank, under: mesh, from: a, to: b)` turns each flank
//! with its own member — the first with the observer of `mesh := motion(gear_turn, relative_to:
//! pinion_turn)`, the second with its source — and asks, at each **phase** `φ` of the pinion, how
//! far the gear has turned when the two touch.  Conjugate flanks touch with the gear exactly where
//! the ratio puts it; any other pair touches with the gear behind or ahead of that by its **lag**
//! `δ`, the transmission error as a roll of the mesh (the gear's own angle off by its ratio times
//! it).  The contact is where the two flanks share a point and a normal:
//!
//! ```text
//! P(φ)·X₁(u₁, v₁, t₁) = G(φ + δ)·X₂(u₂, v₂, t₂)        three rows
//! P(φ)·n₁  ∥  G(φ + δ)·n₂                               two rows
//! ```
//!
//! with `P` the observer's pose, `G` the source's — the phase and the lag in degrees, as a curve's
//! parameter over an angle is — and each flank either **generated** — an
//! envelope in space (`envelope::held`), its place `[u, v, t]` held to its own meshing equation
//! `n · ∂X/∂t = 0` — or **formed**, its source standing still, its place `[u, v]` and its `t` held
//! at nothing.  So the unknowns at a phase are the lag and the two places, seven against seven
//! rows: a regular root is a point contact, and a root where the Jacobian loses one rank a line
//! contact (a conjugate pair, which touches along a line at every phase and so leaves the point
//! free along it).  Over the phase the contact is a curve in space — the **path of contact** — and
//! on each flank a curve of its member — the **bearing**; the lag over the phase is the
//! transmission error.
//!
//! **In the solve** a point held to the contact, `P coincident contact`, owns the phase, the lag
//! and both places, and reads both flanks' sources and motions and the mesh's axes as columns, so
//! a cutter's number solves from a contact or a transmission error the document states — the
//! phase and the lag are slots a pin states (`P coincident(t == 10deg, lag == 0deg) contact`).
//! **After the solve** `Meshing` traces it over its phase from where such a point holds it.
//!
//! **Layout.** A contact's columns are `[lead…, φ, δ, u₁, v₁, t₁, u₂, v₂, t₂, flank₁…, flank₂…,
//! motion…]`: the point's lift (or nothing), the unknowns, then each flank's source and motion
//! columns as an envelope contact reads them (`held::Parts`), then the mesh's axes
//! (`held::Moving`).
//! Its constants are `[lead, n₁, n₂, steps, flank₁…, flank₂…, (STEP)×steps]`: each flank's
//! constants as `held::consts` writes them with no lead, then the mesh's steps, the last a
//! relative whose source is the gear's turn and whose observer the pinion's.
//!
//! What this certifies is local: a root near where it was seeded, not which tooth pair or flank
//! side, not that the contact is within either flank's trimmed face, and not that no other point
//! of the two members touches first.

use super::held::{self, col3, cross, dot, frames, local, unit, vadd, vsub, Layout, Step, STEP};
use crate::kernels::{Dual, Kernel, Num};
use crate::linalg::Mat;
use std::ops::Range;

/// Constants before the flanks'.
const HEADER: usize = 4;
/// The unknowns a contact owns: the phase, the lag and the two places.
pub const UNKNOWNS: usize = 8;
/// The rows at a phase: the shared point, the shared normal, and each flank's own.
pub const ROWS: usize = 7;
/// A point's lift.
pub const LEAD: usize = 3;

/* -- the layout ------------------------------------------------------------------------------ */

/// A meshing's constants read back.
#[derive(Clone, Debug)]
struct Mesh {
    lead: usize,
    /// The pinion's flank (the observer's) and the gear's (the source's), and where each one's
    /// constants are.
    flanks: [Layout; 2],
    consts: [Range<usize>; 2],
    steps: Vec<Step>,
    /// The steps the gear's and the pinion's turns are.
    source: usize,
    observer: usize,
}

impl Mesh {
    fn read(k: &[f64]) -> Option<Mesh> {
        if k.len() < HEADER {
            return None;
        }
        let count = |x: f64| (x >= 0.0 && x <= 1e6 && x.fract() == 0.0).then_some(x as usize);
        let (lead, n1, n2, ns) = (count(k[0])?, count(k[1])?, count(k[2])?, count(k[3])?);
        if k.len() != HEADER + n1 + n2 + STEP * ns {
            return None;
        }
        let consts = [HEADER..HEADER + n1, HEADER + n1..HEADER + n1 + n2];
        let flank = |r: &Range<usize>| Layout::read_any(&k[r.clone()]).filter(|l| l.lead == 0);
        let flanks = [flank(&consts[0])?, flank(&consts[1])?];
        let steps = held::read_steps(&k[HEADER + n1 + n2..])?;
        let Some(&Step::Relative { source, observer }) = steps.last() else { return None };
        Some(Mesh { lead, flanks, consts, steps, source, observer })
    }

    /// The columns of flank `i`'s source and motion.
    fn flank_width(&self, i: usize) -> usize {
        self.flanks[i].source_width() + self.flanks[i].motion_width()
    }

    /// Every column a contact reads.
    fn width(&self) -> usize {
        let flanks = self.flank_width(0) + self.flank_width(1);
        self.lead + UNKNOWNS + flanks + held::steps_width(&self.steps)
    }

    fn rows(&self) -> usize {
        self.lead + ROWS
    }

    /// The pinion's pose at the phase, and the gear's at the phase and the lag (radians): each
    /// step's turn worked out only as far as the one asked for.
    fn poses<T: Num>(&self, motion: &[T], phase: T, gear: T) -> [held::Frame<T>; 2] {
        let pose = |i: usize, t: T| frames(&self.steps[..=i], motion, t)[i];
        [pose(self.observer, phase), pose(self.source, gear)]
    }
}

/// A meshing's constants: the lead's width, each flank's constants (`held::Parts::consts_at(0)`)
/// and the mesh's steps.
fn consts(lead: usize, flanks: [&[f64]; 2], steps: &[Step]) -> Vec<f64> {
    let mut k = vec![lead as f64, flanks[0].len() as f64, flanks[1].len() as f64,
                     steps.len() as f64];
    k.extend_from_slice(flanks[0]);
    k.extend_from_slice(flanks[1]);
    k.extend(held::encode_steps(steps));
    k
}

/* -- the contact at a phase -------------------------------------------------------------------- */

/// What the rows read at the unknowns: each flank's point and normal in the fixed frame, a
/// direction along the gear's flank there, each flank's own row, and each flank's point in its
/// own member.
struct At<T> {
    x: [[T; 3]; 2],
    n: [[T; 3]; 2],
    along: [T; 3],
    own: [T; 2],
    member: [[T; 3]; 2],
    /// A length the normal's rows are read over: how far the contact is from the first point the
    /// mesh's axes read, so they are lengths as the others are.
    lever: T,
}

fn rotate<T: Num>(r: &[[T; 3]; 3], x: [T; 3]) -> [T; 3] {
    std::array::from_fn(|i| r[i][0] * x[0] + r[i][1] * x[1] + r[i][2] * x[2])
}

/// The contact's frame over its columns, the lead's included (it is skipped).
fn at<T: Num>(m: &Mesh, cols: &[T]) -> At<T> {
    let o = m.lead;
    // the phase and the lag in degrees, as a curve's parameter over an angle is
    let deg = T::cst(std::f64::consts::PI / 180.0);
    let (phase, lag) = (cols[o] * deg, cols[o + 1] * deg);
    let mut from = o + UNKNOWNS;
    let mut flank = |i: usize| {
        let w = m.flank_width(i);
        let place = &cols[o + 2 + 3 * i..o + 5 + 3 * i];
        let c: Vec<T> = place.iter().chain(&cols[from..from + w]).copied().collect();
        from += w;
        local(&m.flanks[i], &c, false)
    };
    let (f1, f2) = (flank(0), flank(1));
    let motion = &cols[from..];
    let [(r1, p1), (r2, p2)] = m.poses(motion, phase, phase + lag);
    let x1 = vadd(rotate(&r1, f1.x), p1);
    let x2 = vadd(rotate(&r2, f2.x), p2);
    let lever = dot(vsub(x2, col3(motion, 0)), vsub(x2, col3(motion, 0))).sqrt();
    let lever = if lever.val() > 1e-9 { lever } else { T::cst(1.0) };
    // a formed flank's roll, held at none, over the lever so it is a length as a meshing
    // equation is
    let own = |l: &Layout, g: T, t: T| if l.steps.is_empty() { t * lever } else { g };
    At {
        x: [x1, x2],
        n: [rotate(&r1, f1.n), rotate(&r2, f2.n)],
        along: unit(rotate(&r2, f2.dx[0])),
        own: [own(&m.flanks[0], f1.g, cols[o + 4]), own(&m.flanks[1], f2.g, cols[o + 7])],
        member: [f1.x, f2.x],
        lever,
    }
}

/// A contact's rows over its columns `cols` and constants `k`, into `out`: the point's offset
/// from the pinion's flank where there is a point, the two flanks' offset, the pinion's normal
/// against two directions square to the gear's (each times the lever, a length), and each
/// flank's own row — a generated one's meshing equation, a formed one's roll, held at none.
/// `None` where the constants do not describe a meshing over this many columns.
pub(crate) fn rows<T: Num>(cols: &[T], k: &[f64], out: &mut [T]) -> Option<()> {
    let m = Mesh::read(k)?;
    if cols.len() != m.width() || out.len() != m.rows() {
        return None;
    }
    let f = at(&m, cols);
    let mut i = 0;
    if m.lead == LEAD {
        let p = col3(cols, 0);
        for c in 0..3 {
            out[c] = p[c] - f.x[0][c];
        }
        i = 3;
    }
    for c in 0..3 {
        out[i + c] = f.x[0][c] - f.x[1][c];
    }
    let across = cross(f.n[1], f.along);
    out[i + 3] = dot(f.n[0], f.along) * f.lever;
    out[i + 4] = dot(f.n[0], across) * f.lever;
    out[i + 5] = f.own[0];
    out[i + 6] = f.own[1];
    Some(())
}

/* -- the kernel ------------------------------------------------------------------------------ */

fn res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    held::block_res(n, v, k, r, LEAD + ROWS, rows);
}

fn jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    held::block_jac(n, v, k, j, LEAD + ROWS, rows);
}

/// The kernel of a point on a meshing at a block's widths (`system::build_kernel`).
pub fn kernel(n_par: usize, n_const: usize) -> Kernel {
    Kernel { name: "point_on_meshing", n_res: LEAD + ROWS, n_par, n_const, degree: 1, res, jac,
             const_jac: None, form: None }
}

/* -- reading the sketch ------------------------------------------------------------------------ */

use crate::model::{Flank, Sketch};

/// What a contact reads of a meshing, off the sketch: each flank's parts and the mesh's.
#[derive(Clone, Debug)]
pub struct Parts {
    pub flanks: [held::Parts; 2],
    pub moving: held::Moving,
}

impl Parts {
    /// Meshing `index`'s parts, or why the solve cannot read it.
    pub fn of(sk: &Sketch, index: usize) -> Result<Parts, String> {
        let e = sk.meshings.get(index).ok_or("no such meshing")?;
        let flank = |f: &Flank, which: &str| -> Result<held::Parts, String> {
            match f {
                Flank::Envelope(i) => held::Parts::of(sk, *i as usize).map_err(|why| {
                    format!("its {which} flank is an envelope the solve cannot read: {why}")
                }),
                Flank::Formed(h) => held::Parts::still(sk, h, &e.name).map_err(|why| {
                    format!("its {which} flank is a surface the solve cannot read: {why}")
                }),
            }
        };
        let flanks = [flank(&e.flanks[0], "first")?, flank(&e.flanks[1], "second")?];
        let moving = held::Moving::of(sk, e.motion as usize)?;
        if !matches!(moving.steps.last(), Some(Step::Relative { .. })) {
            let name = &sk.motions[e.motion as usize].name;
            return Err(format!("`{name}` is no relative motion: a meshing turns its flanks under \
                                `motion(gear_turn, relative_to: pinion_turn)`, the first flank with \
                                the pinion and the second with the gear"));
        }
        Ok(Parts { flanks, moving })
    }

    /// The points whose lifts the columns read.
    pub fn lifted(&self) -> Vec<usize> {
        [self.flanks[0].lifted(), self.flanks[1].lifted(), self.moving.lifted()].concat()
    }

    /// The columns past the unknowns, every point's lift minted.
    pub fn columns(&self, sk: &Sketch) -> Vec<u32> {
        [self.flanks[0].columns(sk), self.flanks[1].columns(sk), self.moving.columns(sk)].concat()
    }

    /// Their values as the drawing stands.
    pub fn values(&self, sk: &Sketch) -> Vec<f64> {
        [self.flanks[0].values(sk), self.flanks[1].values(sk), self.moving.values(sk)].concat()
    }

    /// The constants of a contact with a lead of `lead` columns.
    pub fn consts(&self, lead: usize) -> Vec<f64> {
        let flanks = [self.flanks[0].consts_at(0), self.flanks[1].consts_at(0)];
        consts(lead, [&flanks[0], &flanks[1]], &self.moving.steps)
    }

    pub fn n_const(&self) -> usize {
        let flanks = self.flanks[0].n_const() + self.flanks[1].n_const();
        HEADER + flanks + STEP * self.moving.steps.len()
    }

    /// How many columns a contact with a lead of `lead` reads.
    pub fn width(&self, lead: usize) -> usize {
        let flanks = self.flanks[0].own_width() + self.flanks[1].own_width();
        lead + UNKNOWNS + flanks + self.moving.width()
    }
}

/* -- the contact, apart from the solve --------------------------------------------------------- */

/// The unknowns a contact owns: the phase and the lag, in degrees, and the place on each flank.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Place {
    pub phase: f64,
    pub lag: f64,
    pub pinion: [f64; 3],
    pub gear: [f64; 3],
}

impl Place {
    pub fn of(x: &[f64]) -> Place {
        Place { phase: x[0], lag: x[1], pinion: [x[2], x[3], x[4]], gear: [x[5], x[6], x[7]] }
    }

    pub fn values(&self) -> [f64; UNKNOWNS] {
        [self.phase, self.lag, self.pinion[0], self.pinion[1], self.pinion[2], self.gear[0],
         self.gear[1], self.gear[2]]
    }
}

/// The contact at one phase.
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub place: Place,
    /// The gear's own angle behind (negative) or ahead of the ratio's, in degrees, where the
    /// gear's turn is a turn about a line at a ratio: its ratio times the lag.
    pub gear_error: Option<f64>,
    /// The contact in the fixed frame: a point of the path of contact.
    pub position: [f64; 3],
    /// The contact on each flank, in its own member: a point of each bearing.
    pub on_pinion: [f64; 3],
    pub on_gear: [f64; 3],
    /// Whether the two touch along a line rather than at a point: the rows' Jacobian in the lag
    /// and the places loses a rank, the contact free along it.
    pub line: bool,
    /// The largest row at the root, in the drawing's lengths.
    pub residual: f64,
}

/// A meshing as numbers: its constants, the values of its columns past the unknowns, and its
/// phase interval.  What the after-the-solve reading and a seed solve.
#[derive(Clone, Debug)]
pub struct Meshing {
    k: Vec<f64>,
    parts: Vec<f64>,
    /// The phase interval, in degrees.
    pub roll: [f64; 2],
    ratio: Option<f64>,
    /// The anchor: where a point the drawing holds to the contact holds it, once solved.
    pub anchor: Option<Place>,
    /// A length the drawing is drawn in, for its tolerances.
    scale: f64,
}

impl Meshing {
    /// A meshing over constants `k` (no lead) and its columns' values, its phase interval `roll`
    /// in radians (as `MeshingE` keeps it).
    pub fn new(k: Vec<f64>, parts: Vec<f64>, roll: [f64; 2]) -> Option<Meshing> {
        let m = Mesh::read(&k)?;
        if m.lead != 0 || parts.len() != m.width() - UNKNOWNS {
            return None;
        }
        let ratio = match m.steps[m.source] {
            Step::Rotation { ratio, advance, .. } if advance == 0.0 => Some(ratio),
            _ => None,
        };
        let scale = parts.iter().fold(0.0f64, |a, x| a.max(x.abs())).max(1.0);
        Some(Meshing { k, parts, roll: roll.map(f64::to_degrees), ratio, anchor: None, scale })
    }

    /// Meshing `index` as the drawing stands, anchored where the first point held to it is.
    pub fn named(sk: &Sketch, index: usize) -> Result<Meshing, String> {
        let p = Parts::of(sk, index)?;
        let mut m = Meshing::new(p.consts(0), p.values(sk), sk.meshings[index].roll)
            .ok_or("the meshing's parts are not a meshing")?;
        m.anchor = sk.constraints.iter().find_map(|c| {
            c.meshing_of().filter(|e| e.i() == index)?;
            let x: Vec<f64> = c.args[2..2 + UNKNOWNS].iter().map(|a| a.value(sk)).collect();
            x.iter().all(|v| v.is_finite()).then(|| Place::of(&x))
        });
        Ok(m)
    }

    fn mesh(&self) -> Mesh {
        Mesh::read(&self.k).expect("a meshing's constants were read at its making")
    }

    fn cols(&self, x: &[f64; UNKNOWNS]) -> Vec<f64> {
        x.iter().chain(&self.parts).copied().collect()
    }

    /// The rows at the unknowns `x`: NaN where the constants are not a meshing's.
    fn residuals(&self, x: &[f64; UNKNOWNS]) -> [f64; ROWS] {
        let mut out = [f64::NAN; ROWS];
        let _ = rows::<f64>(&self.cols(x), &self.k, &mut out);
        out
    }

    /// The columns past the unknowns of flank `i`, and of the mesh's motion.
    fn flank_parts(&self, mesh: &Mesh, i: usize) -> &[f64] {
        let w0 = mesh.flank_width(0);
        if i == 0 { &self.parts[..w0] } else { &self.parts[w0..w0 + mesh.flank_width(1)] }
    }

    /// The contact at `phase`, solved from `start` (its phase ignored): the lag and the two places
    /// by `newton::dogleg`, minimum-norm steps where the two touch along a line.  `None` where
    /// the solve finds no root.
    pub fn at(&self, phase: f64, start: &Place) -> Option<Sample> {
        let mut x = start.values();
        x[0] = phase;
        let mut tr = AtPhase { m: self, x, j: Mat::zeros(ROWS, UNKNOWNS - 1) };
        let mut z: Vec<f64> = x[1..].to_vec();
        let mut r = vec![0.0; ROWS];
        crate::newton::TrustRegion::residuals_into(&mut tr, &z, &mut r);
        if !r.iter().all(|v| v.is_finite()) {
            return None;
        }
        let tol = crate::newton::Tol { ftol: 1e-13 * self.scale, xtol: 1e-15, gtol: 1e-30 };
        crate::newton::dogleg(&mut tr, &mut z, &mut r, tol, 200, 600);
        x[1..].copy_from_slice(&z);
        self.sample(&x)
    }

    /// The contact at `phase` reached from `from` through phases between where a step is too long
    /// for the solve: half the way, then the rest, `depth` halvings at most.
    fn reach(&self, from: &Sample, phase: f64, depth: u32) -> Option<Sample> {
        if let Some(s) = self.at(phase, &from.place) {
            return Some(s);
        }
        if depth == 0 {
            return None;
        }
        let mid = self.reach(from, (from.place.phase + phase) / 2.0, depth - 1)?;
        self.reach(&mid, phase, depth - 1)
    }

    /// The contact at the unknowns `x`, where its rows vanish there.
    fn sample(&self, x: &[f64; UNKNOWNS]) -> Option<Sample> {
        let r = self.residuals(x);
        let residual = r.iter().fold(0.0f64, |a, v| a.max(v.abs()));
        if !(residual <= 1e-8 * self.scale) {
            return None;
        }
        let m = self.mesh();
        let f = at(&m, &self.cols(x));
        let mut tr = AtPhase { m: self, x: *x, j: Mat::zeros(ROWS, UNKNOWNS - 1) };
        crate::newton::TrustRegion::jacobian_at(&mut tr, &x[1..]);
        let s = crate::linalg::svd(&tr.j, false).s;
        let line = s.last().is_some_and(|&least| least <= 1e-7 * s[0].max(1e-300));
        let place = Place::of(x);
        Some(Sample {
            place,
            gear_error: self.ratio.map(|r| r * place.lag),
            position: f.x[0],
            on_pinion: f.member[0],
            on_gear: f.member[1],
            line,
            residual,
        })
    }

    /// The contact over `phases`, walked outward from the anchor: each phase solved from the one
    /// beside it nearer the anchor, carried on by the step the last two took.  `None` at a phase
    /// the walk lost the contact, and past it on that side.
    pub fn trace(&self, start: &Place, phases: &[f64]) -> Vec<Option<Sample>> {
        let mut out = vec![None; phases.len()];
        let Some(first) = self.at(start.phase, start) else { return out };
        let mut order: Vec<usize> = (0..phases.len()).collect();
        order.sort_by(|&a, &b| phases[a].total_cmp(&phases[b]));
        let split = order.iter().position(|&i| phases[i] >= start.phase).unwrap_or(order.len());
        let (below, above) = order.split_at(split);
        for side in [above.to_vec(), below.iter().rev().copied().collect::<Vec<_>>()] {
            let (mut last, mut before): (Sample, Option<Sample>) = (first, None);
            for i in side {
                let mut guess = last.place;
                if let Some(b) = before {
                    let (d, k) = (last.place.phase - b.place.phase, phases[i] - last.place.phase);
                    if d.abs() > 0.0 {
                        let w = k / d;
                        let (a, c) = (last.place.values(), b.place.values());
                        let ahead: [f64; UNKNOWNS] = std::array::from_fn(|q| a[q] + w * (a[q] - c[q]));
                        guess = Place::of(&ahead);
                    }
                }
                let found = self.at(phases[i], &guess).or_else(|| self.reach(&last, phases[i], 6));
                let Some(s) = found else { break };
                out[i] = Some(s);
                before = Some(last);
                last = s;
            }
        }
        out
    }

    /// The contact at `n + 1` phases evenly over the interval, walked from the anchor; `Err`
    /// where no point is held to it to start from.
    pub fn traced(&self, n: usize) -> Result<Trace, String> {
        let start = self.anchor.ok_or("no point is held to it, so no trace starts")?;
        let phases = self.phases(n);
        let samples = self.trace(&start, &phases);
        Ok(Trace { phases, samples })
    }

    /// `n + 1` phases evenly over the meshing's interval, in degrees.
    fn phases(&self, n: usize) -> Vec<f64> {
        let [a, b] = self.roll;
        (0..=n).map(|i| a + (b - a) * i as f64 / n.max(1) as f64).collect()
    }
}

/// A meshing traced over its interval: the phases, in degrees, and the contact at each — `None`
/// where the walk lost it.
#[derive(Clone, Debug)]
pub struct Trace {
    pub phases: Vec<f64>,
    pub samples: Vec<Option<Sample>>,
}

impl Trace {
    /// The samples found, in phase order.
    pub fn found(&self) -> impl Iterator<Item = &Sample> {
        self.samples.iter().flatten()
    }

    /// The transmission error's range over the samples found, in degrees: the gear's angle where
    /// its turn has a ratio, else the lag.
    pub fn error_range(&self) -> Option<[f64; 2]> {
        let e = |s: &Sample| s.gear_error.unwrap_or(s.place.lag);
        self.found().map(e).fold(None, |r, x| Some(match r {
            None => [x, x],
            Some([a, b]) => [a.min(x), b.max(x)],
        }))
    }

    /// How long the path of contact is, as the polyline through the samples found.
    pub fn path_length(&self) -> f64 {
        let p: Vec<[f64; 3]> = self.found().map(|s| s.position).collect();
        crate::space::polyline_length(&p)
    }
}

/// The rows at one phase over the lag and the places, the rest held: what `Meshing::at` walks.
struct AtPhase<'a> {
    m: &'a Meshing,
    x: [f64; UNKNOWNS],
    j: Mat,
}

impl AtPhase<'_> {
    fn with(&self, z: &[f64]) -> [f64; UNKNOWNS] {
        let mut x = self.x;
        x[1..].copy_from_slice(z);
        x
    }
}

impl crate::newton::TrustRegion for AtPhase<'_> {
    fn n(&self) -> usize {
        UNKNOWNS - 1
    }
    fn m(&self) -> usize {
        ROWS
    }
    fn residuals_into(&mut self, z: &[f64], out: &mut [f64]) {
        let r = self.m.residuals(&self.with(z));
        let finite = r.iter().all(|x| x.is_finite());
        out.copy_from_slice(&if finite { r } else { [f64::INFINITY; ROWS] });
    }
    fn jacobian_at(&mut self, z: &[f64]) {
        let x = self.with(z);
        let cols: Vec<Dual<7>> = x.iter().chain(&self.m.parts).enumerate().map(|(q, &v)| match q {
            1..=7 => Dual::var(v, q - 1),
            _ => Dual::cst(v),
        }).collect();
        let mut out = [Dual::<7>::cst(0.0); ROWS];
        let ok = rows(&cols, &self.m.k, &mut out).is_some();
        for (row, v) in out.iter().enumerate() {
            for q in 0..UNKNOWNS - 1 {
                self.j.set(row, q, if ok { v.g[q] } else { f64::NAN });
            }
        }
    }
    fn jt_mul(&mut self, v: &[f64], out: &mut [f64]) {
        out.copy_from_slice(&self.j.mul_t_vec(v));
    }
    fn j_mul(&mut self, v: &[f64], out: &mut [f64]) {
        out.copy_from_slice(&self.j.mul_vec(v));
    }
    fn gn_step(&mut self, r: &[f64], _g: &[f64], p: &mut [f64]) {
        crate::newton::dense_gn_step(&self.j, r, p, 1e-10);
    }
}

/// Where a point `p` (in the fixed frame) starts on a meshing: the phase given in degrees (a
/// pin's, a seed's), no lag, and each flank's place nearest `p` seen from its member at that
/// phase — a generated flank's by `held::seed` over its roll, a formed one's over a coarse grid of
/// its source — then the contact solved there from those.  `rolls` are the flanks' roll
/// intervals (a formed one's unread).  `None` where nothing is finite.
pub fn seed(m: &Meshing, rolls: [[f64; 2]; 2], p: [f64; 3], phase: f64) -> Option<Place> {
    let mesh = m.mesh();
    let motion = &m.parts[mesh.flank_width(0) + mesh.flank_width(1)..];
    let phase_r = phase.to_radians();
    let poses = mesh.poses(motion, phase_r, phase_r);
    // `p` seen from flank `i`'s member
    let seen = |i: usize| -> [f64; 3] {
        let (r, t) = poses[i];
        let d = vsub(p, t);
        std::array::from_fn(|c| r[0][c] * d[0] + r[1][c] * d[1] + r[2][c] * d[2])
    };
    let place = |i: usize| -> Option<[f64; 3]> {
        let (l, parts) = (&mesh.flanks[i], m.flank_parts(&mesh, i));
        if l.steps.is_empty() {
            still_seed(l, parts, seen(i))
        } else {
            let mut k = m.k[mesh.consts[i].clone()].to_vec();
            k[0] = LEAD as f64;
            held::seed(&k, parts, seen(i), rolls[i])
        }
    };
    let start = Place { phase, lag: 0.0, pinion: place(0)?, gear: place(1)? };
    Some(m.at(phase, &start).map_or(start, |s| s.place))
}

/// A formed flank's place nearest `q`: a coarse grid of its source, then `newton::dogleg` on the
/// point's three rows.
fn still_seed(l: &Layout, parts: &[f64], q: [f64; 3]) -> Option<[f64; 3]> {
    let point = |u: f64, v: f64| -> Option<[f64; 3]> {
        let c: Vec<f64> = [u, v, 0.0].iter().chain(parts).copied().collect();
        let f = local(l, &c, false);
        f.x.iter().all(|x| x.is_finite()).then_some(f.x)
    };
    let reach = crate::space::distance(q, [parts[0], parts[1], parts[2]]).max(1e-3);
    let us = held::meridian_spans(l.source, reach);
    let mut best: Option<([f64; 2], f64)> = None;
    for &u in &us {
        for b in 0..36 {
            let v = b as f64 * std::f64::consts::TAU / 36.0;
            if let Some(x) = point(u, v) {
                let d = crate::space::distance(x, q);
                if best.is_none_or(|(_, c)| d < c) {
                    best = Some(([u, v], d));
                }
            }
        }
    }
    let (start, _) = best?;
    let mut tr = Nearest { l, parts, q, j: Mat::zeros(3, 2) };
    let mut z = start.to_vec();
    let mut r = vec![0.0; 3];
    crate::newton::TrustRegion::residuals_into(&mut tr, &z, &mut r);
    let tol = crate::newton::Tol { ftol: 1e-12 * (1.0 + reach), xtol: 1e-14, gtol: 1e-14 };
    crate::newton::dogleg(&mut tr, &mut z, &mut r, tol, 100, 400);
    z.iter().all(|v| v.is_finite()).then_some([z[0], z[1], 0.0])
}

/// A formed flank's point at `[u, v]` against `q`.
struct Nearest<'a> {
    l: &'a Layout,
    parts: &'a [f64],
    q: [f64; 3],
    j: Mat,
}

impl Nearest<'_> {
    fn eval<T: Num>(&self, u: T, v: T) -> [T; 3] {
        let parts = self.parts.iter().map(|&x| T::cst(x));
        let c: Vec<T> = [u, v, T::cst(0.0)].into_iter().chain(parts).collect();
        let f = local(self.l, &c, false);
        vsub(f.x, self.q.map(T::cst))
    }
}

impl crate::newton::TrustRegion for Nearest<'_> {
    fn n(&self) -> usize {
        2
    }
    fn m(&self) -> usize {
        3
    }
    fn residuals_into(&mut self, z: &[f64], out: &mut [f64]) {
        let r = self.eval(z[0], z[1]);
        let finite = r.iter().all(|x| x.is_finite());
        out.copy_from_slice(&if finite { r } else { [f64::INFINITY; 3] });
    }
    fn jacobian_at(&mut self, z: &[f64]) {
        let r = self.eval(Dual::<2>::var(z[0], 0), Dual::<2>::var(z[1], 1));
        for (row, v) in r.iter().enumerate() {
            for q in 0..2 {
                self.j.set(row, q, v.g[q]);
            }
        }
    }
    fn jt_mul(&mut self, v: &[f64], out: &mut [f64]) {
        out.copy_from_slice(&self.j.mul_t_vec(v));
    }
    fn j_mul(&mut self, v: &[f64], out: &mut [f64]) {
        out.copy_from_slice(&self.j.mul_vec(v));
    }
    fn gn_step(&mut self, r: &[f64], _g: &[f64], p: &mut [f64]) {
        crate::newton::dense_gn_step(&self.j, r, p, 1e-12);
    }
}
