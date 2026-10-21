//! **An envelope the solve sees** (#283, step 2).
//!
//! `flank := envelope(source over t in (a, b), under: m)` over a surface of revolution — a cone or
//! a cylinder set (`std.Cone`, `std.ConeThrough`, `std.Cylinder`), or a line, arc or circle of a
//! revolution's profile (`surface(rev, edge: e)`) — under a motion in space is the locus where the
//! moving source's normal is square to its velocity, `n · ∂X/∂t = 0`.  A point held to it, a line
//! in its tangent plane or along its normal there, a plane touching it and a circle osculating it
//! are rows of the drawing: each owns or shares a **place** `[u, v, t]` — along the source's
//! meridian, round its axis, and the motion's roll — and reads the source's geometry and the
//! motion's axes as columns, so the numbers that make the source (a cutter's lean, where its axis
//! stands) solve from what the envelope is held to.
//!
//! The envelope's normal at a point is the moving source's normal there (the two touch), so its
//! curvature needs no second solve: along a direction `w` in `ker ∇g` of the place, the shape
//! operator is `−dn·dX` read over the place, `dn` and `dX` the derivatives of the moved normal and
//! point.  So everything is written once, over `kernels::Num`, as a truncated second-order Taylor
//! series in the place (`H2`): the residual over `f64`, and the Jacobian over tangents (`Dual`),
//! eight columns a pass — exact, no differences.
//!
//! **Layout.** A contact's columns are `[lead…, u, v, t, source…, motion…]`: what the contact
//! states about (a point's lift, a line's ends, a plane's axes, a circle), the place, the source
//! (`Source`), and two lifted ends per axis the motion turns about or slides along.  Its constants
//! are `[lead, source, axis, c, m, free, ref×3, steps, (code, at, source | free, observer | back,
//! ratio, advance)×steps]` (`HEADER`, `STEP`): the source's axis as two points (`axis` 0, a line's
//! ends) or a point and a direction (1, an axis's), its number `c + m·x` (a cone's half angle, a
//! cylinder's radius; `x` its last column where `free`), a direction its axis does not run along
//! (`ref`, which fixes where a set's `v` starts), and the motion as the steps `motion::Family`
//! reads — past them, where a joint is driven, each driven step's law (`encode_laws`, #398 G), so
//! the step stands at the law's value of the roll and every row reads the law exactly.
//!
//! What this certifies is local: a root of the rows is a regular point of the envelope near the
//! place it was seeded at — not which branch, not that the roll is within `from:`/`to:`, and not
//! that the material is on either side.

use crate::fmath::Det;
use crate::kernels::{curve_widths, Dual, Kernel, Num};
use std::ops::{Add, Mul, Sub};

/* -- the layout ------------------------------------------------------------------------------ */

/// Constants before the motion's steps.
pub const HEADER: usize = 10;
/// Constants per step of the motion.
pub const STEP: usize = 6;

/// What a source is a revolution of: the meridian element turned about the axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// A segment `p1 → p2` (a revolution's line, `surface(rev, edge: l)`): `[a, b, p1, p2]`.
    Line = 1,
    /// A cone's generator from the axis's start at the half angle (`std.Cone`, and
    /// `std.ConeThrough` about its spine): `[a, b]`, the half its number.
    Cone = 2,
    /// A line parallel to the axis at the radius (`std.Cylinder`): `[a, b]`, the radius its
    /// number.
    Cylinder = 3,
    /// A circle about a centre in a plane (a revolution's arc or circle): `[a, b, c, r, U, V]`.
    Round = 4,
    /// A plane square to the axis, its number along the axis from a point `o` (`std.HalfSpace`'s
    /// face, #326): `[a, b, o]`. Not a revolution: its place is `[u, v]` along two directions
    /// in it, both lengths, so its normal is the axis's direction everywhere.
    Plane = 5,
}

impl Source {
    pub fn of(code: f64) -> Option<Source> {
        Some(match code as i64 {
            1 => Source::Line,
            2 => Source::Cone,
            3 => Source::Cylinder,
            4 => Source::Round,
            5 => Source::Plane,
            _ => return None,
        })
    }

    /// Its own columns, before a free number's.
    pub fn width(self) -> usize {
        match self {
            Source::Line => 12,
            Source::Cone | Source::Cylinder => 6,
            Source::Round => 16,
            Source::Plane => 9,
        }
    }
}

/// Which statement a contact makes: a point on the envelope.  What is read at a point a contact
/// holds — its normal, a plane or a line touching it there, a section's osculating circle — is the
/// contact's frame, read by a rate of it (`contact_rate`, #365; `frame` here).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Contact {
    /// `p coincident flank`: `p − X`, and the meshing equation. Lead: the point's lift.
    Point = 0,
}

impl Contact {
    pub const ALL: [Contact; 1] = [Contact::Point];

    pub fn rows(self) -> usize {
        4
    }

    /// The lead's width (the columns before the place).
    pub fn lead(self) -> usize {
        3
    }

    fn name(self) -> &'static str {
        "point_on_envelope"
    }

    /// A point's rows are lengths.
    fn degree(self) -> u32 {
        1
    }
}

/// **A joint's numbers the solve finds** (#383 stage 5): the motion columns `at..at + 2`, its
/// ratio and its advance, read in place of a step's constants — negated where the step runs
/// back, an inverse's.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Free {
    pub at: usize,
    pub back: bool,
}

/// One step of a motion, as its constants say it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Step {
    /// About the line whose lifted ends are motion columns `at..at + 6` — or, `dir`, the axis
    /// through the point at `at..at + 3` along the direction at `at + 3..at + 6`: a joint held to
    /// an axis no drawn line is, a frame's own (#366).
    Rotation { at: usize, ratio: f64, advance: f64, free: Option<Free>, dir: bool },
    /// Along the line whose lifted ends are motion columns `at..at + 6`, or the axis `dir` says.
    Translation { at: usize, advance: f64, free: Option<Free>, dir: bool },
    /// About the line through the lifted centre at motion columns `at..at + 3`, square to the
    /// view whose two axes' directions are columns `at + 3..at + 9`: a turn about a point of a
    /// view, as it reads in space (#326, read by `Moving::of_any` alone).
    Turn { at: usize, ratio: f64, free: Option<Free> },
    /// `observer⁻¹ ∘ source`, by step index.
    Relative { source: usize, observer: usize },
}

/// The constants of one contact, built from its parts (the inverse of `Layout::read`): a
/// driven step's law past the steps (`encode_laws`), where the motion has one (#398 G).
#[allow(clippy::too_many_arguments)]
pub fn consts(lead: usize, source: Source, axis: u8, number: (f64, f64, bool), reference: [f64; 3],
              steps: &[Step], laws: &[Driven]) -> Vec<f64> {
    let mut k = vec![lead as f64, source as i64 as f64, axis as f64,
                     number.0, number.1, if number.2 { 1.0 } else { 0.0 },
                     reference[0], reference[1], reference[2], steps.len() as f64];
    k.extend(encode_steps(steps));
    if !laws.is_empty() {
        k.extend(encode_laws(laws));
    }
    k
}

/// A motion's steps as constants, `STEP` each: a joint's free numbers as where their columns
/// start, one past, and whether it runs back.
pub fn encode_steps(steps: &[Step]) -> Vec<f64> {
    let free = |f: Option<Free>| f.map_or((0.0, 0.0), |f| ((f.at + 1) as f64, f.back as u8 as f64));
    steps.iter().flat_map(|s| match *s {
        Step::Rotation { at, ratio, advance, free: f, dir } => {
            let (a, b) = free(f);
            [if dir { 5.0 } else { 1.0 }, at as f64, a, b, ratio, advance]
        }
        Step::Translation { at, advance, free: f, dir } => {
            let (a, b) = free(f);
            [if dir { 6.0 } else { 2.0 }, at as f64, a, b, 0.0, advance]
        }
        Step::Turn { at, ratio, free: f } => {
            let (a, b) = free(f);
            [4.0, at as f64, a, b, ratio, 0.0]
        }
        Step::Relative { source, observer } => [3.0, 0.0, source as f64, observer as f64, 0.0, 0.0],
    }).collect()
}

impl Step {
    /// The free numbers, where the solve finds them.
    pub fn free(&self) -> Option<Free> {
        match *self {
            Step::Rotation { free, .. } | Step::Translation { free, .. }
                | Step::Turn { free, .. } => free,
            Step::Relative { .. } => None,
        }
    }

    /// Its ratio and its advance over the motion's columns `m`: its constants, or its free
    /// columns where it has them.
    pub(crate) fn numbers<T: Num>(&self, m: &[T]) -> (T, T) {
        let (ratio, advance) = match *self {
            Step::Rotation { ratio, advance, .. } => (ratio, advance),
            Step::Translation { advance, .. } => (0.0, advance),
            Step::Turn { ratio, .. } => (ratio, 0.0),
            Step::Relative { .. } => (0.0, 0.0),
        };
        match self.free() {
            Some(f) => {
                let k = T::cst(if f.back { -1.0 } else { 1.0 });
                let ratio = if matches!(self, Step::Translation { .. }) { T::cst(0.0) }
                    else { k * m[f.at] };
                let advance = if matches!(self, Step::Turn { .. }) { T::cst(0.0) }
                    else { k * m[f.at + 1] };
                (ratio, advance)
            }
            None => (T::cst(ratio), T::cst(advance)),
        }
    }
}

/// Steps read back off `encode_steps`' constants: `None` where they are not steps, or a relative
/// names a step not before it.
pub fn read_steps(k: &[f64]) -> Option<Vec<Step>> {
    if k.len() % STEP != 0 {
        return None;
    }
    let mut steps = Vec::with_capacity(k.len() / STEP);
    let free = |s: &[f64]| -> Option<Option<Free>> {
        match s[2] {
            a if a == 0.0 => Some(None),
            a if a >= 1.0 && a.fract() == 0.0 =>
                Some(Some(Free { at: a as usize - 1, back: s[3] != 0.0 })),
            _ => None,
        }
    };
    for (i, s) in k.chunks(STEP).enumerate() {
        let idx = |x: f64| (x >= 0.0 && (x as usize) < i).then_some(x as usize);
        steps.push(match s[0] as i64 {
            c @ (1 | 5) => Step::Rotation { at: s[1] as usize, ratio: s[4], advance: s[5],
                                            free: free(s)?, dir: c == 5 },
            c @ (2 | 6) => Step::Translation { at: s[1] as usize, advance: s[5], free: free(s)?,
                                               dir: c == 6 },
            4 => Step::Turn { at: s[1] as usize, ratio: s[4], free: free(s)? },
            3 => Step::Relative { source: idx(s[2])?, observer: idx(s[3])? },
            _ => return None,
        });
    }
    Some(steps)
}

/// The columns a motion's steps read: six per axis, and two a joint whose numbers are free.
pub fn steps_width(steps: &[Step]) -> usize {
    steps.iter().filter_map(|s| {
        let axis = match *s {
            Step::Rotation { at, .. } | Step::Translation { at, .. } => at + 6,
            Step::Turn { at, .. } => at + 9,
            Step::Relative { .. } => return None,
        };
        Some(axis.max(s.free().map_or(0, |f| f.at + 2)))
    }).max().unwrap_or(0)
}

/// **A driven step's law** (#398 G): step `step` stands at the law's value of the roll rather
/// than at the roll, its tape borrowed from the constants (`generate::Drive`, as a profile in the
/// plane carries one).
#[derive(Clone, Copy, Debug)]
pub struct Driven<'a> {
    pub step: usize,
    pub law: crate::generate::Drive<'a>,
}

/// Each driven step's law as constants: `[laws, (step, len, flat…, n_x, x…, at)…]`.
pub fn encode_laws(laws: &[Driven]) -> Vec<f64> {
    let mut k = vec![laws.len() as f64];
    for &Driven { step, law: d } in laws {
        k.extend([step as f64, d.flat.len() as f64]);
        k.extend_from_slice(d.flat);
        k.push(d.x.len() as f64);
        k.extend_from_slice(d.x);
        k.push(d.at as f64);
    }
    k
}

/// The laws `encode_laws` wrote, read back off `k` from `at` over `steps`, and where they end:
/// `None` where they are no laws', or a law drives a step there is none of, or a relative.
fn read_laws<'a>(k: &'a [f64], mut at: usize, steps: &[Step]) -> Option<(Vec<Driven<'a>>, usize)> {
    let ix = |i: usize| k.get(i).copied().filter(|v| *v >= 0.0 && v.fract() == 0.0)
        .map(|v| v as usize);
    let n_laws = ix(at).filter(|&l| l <= steps.len())?;
    at += 1;
    let mut laws = Vec::with_capacity(n_laws);
    for _ in 0..n_laws {
        let step = ix(at)
            .filter(|&s| s < steps.len() && !matches!(steps[s], Step::Relative { .. }))?;
        let len = ix(at + 1)?;
        let flat = k.get(at + 2..at + 2 + len)?;
        at += 2 + len;
        let nx = ix(at)?;
        let x = k.get(at + 1..at + 1 + nx)?;
        at += 1 + nx;
        let slot = ix(at).filter(|&s| s < nx)?;
        at += 1;
        laws.push(Driven { step, law: crate::generate::Drive { flat, x, at: slot } });
    }
    Some((laws, at))
}

/// A motion a relation reads as constants (`Moving::encode`): `[n, steps…, laws…]` — the steps as
/// `encode_steps` writes them, then each driven step's law (`encode_laws`).
pub fn encode_motion(steps: &[Step], laws: &[Driven]) -> Vec<f64> {
    let mut k = vec![steps.len() as f64];
    k.extend(encode_steps(steps));
    k.extend(encode_laws(laws));
    k
}

/// A motion read back off `encode_motion`'s constants: `None` where they are no motion's, or a
/// law drives a step there is none of, or a relative.
pub fn read_motion(k: &[f64]) -> Option<(Vec<Step>, Vec<Driven<'_>>)> {
    let n = k.first().copied().filter(|v| (0.0..=64.0).contains(v) && v.fract() == 0.0)? as usize;
    let steps = read_steps(k.get(1..1 + STEP * n)?)?;
    let (laws, at) = read_laws(k, 1 + STEP * n, &steps)?;
    (at == k.len()).then_some((steps, laws))
}

/// A contact's constants read back, a driven step's law borrowed from them.
#[derive(Clone, Debug)]
pub struct Layout<'a> {
    pub lead: usize,
    pub source: Source,
    /// The source's axis as two points (0) or a point and a direction (1).
    pub axis: u8,
    pub number: (f64, f64, bool),
    pub reference: [f64; 3],
    pub steps: Vec<Step>,
    /// Each driven step's law (#398 G): the envelope generated under a drive.
    pub laws: Vec<Driven<'a>>,
}

impl Layout<'_> {
    /// A contact's constants read back: an envelope's, so its motion has a step.
    pub fn read(k: &[f64]) -> Option<Layout<'_>> {
        Self::read_any(k).filter(|l| !l.steps.is_empty())
    }

    /// A flank's constants read back (#283, `envelope::meshing`): an envelope's, or a formed
    /// flank's — the source itself, standing still, with no step.  Past the steps, the laws of
    /// the steps a drive drives, where there are any (`consts`).
    pub fn read_any(k: &[f64]) -> Option<Layout<'_>> {
        if k.len() < HEADER {
            return None;
        }
        let source = Source::of(k[1])?;
        let n = k[9];
        if !(n >= 0.0 && n <= 64.0) || k.len() < HEADER + STEP * n as usize {
            return None;
        }
        let end = HEADER + STEP * n as usize;
        let steps = read_steps(&k[HEADER..end])?;
        let laws = match k.len() == end {
            true => Vec::new(),
            false => {
                let (laws, at) = read_laws(k, end, &steps)?;
                if at != k.len() {
                    return None;
                }
                laws
            }
        };
        Some(Layout {
            lead: k[0] as usize,
            source,
            axis: k[2] as u8,
            number: (k[3], k[4], k[5] != 0.0),
            reference: [k[6], k[7], k[8]],
            steps,
            laws,
        })
    }

    /// The source's columns, a free number's included.
    pub fn source_width(&self) -> usize {
        self.source.width() + self.number.2 as usize
    }

    /// The motion's columns: six per axis a step reads.
    pub fn motion_width(&self) -> usize {
        steps_width(&self.steps)
    }

    /// Every column the contact reads.
    pub fn width(&self) -> usize {
        self.lead + 3 + self.source_width() + self.motion_width()
    }
}

/* -- numbers ----------------------------------------------------------------------------------- */

impl Num for f64 {
    fn cst(v: f64) -> Self {
        v
    }
    fn val(self) -> f64 {
        self
    }
    fn sqrt(self) -> Self {
        f64::sqrt(self.max(0.0))
    }
    fn sin_cos(self) -> (Self, Self) {
        self.dsin_cos()
    }
    fn atan2(y: Self, x: Self) -> Self {
        y.datan2(x)
    }
    fn moves(self) -> bool {
        false
    }
}

fn zero<T: Num>() -> T {
    T::cst(0.0)
}

/// A number and its first two derivatives in the place `[u, v, t]`: `f(x + δ) ≈ v + g·δ +
/// δ·h·δ/2` — `h` carried only where the place's variables ask for it (`second`), since only
/// a curvature reads it: otherwise left zero, and `v` and `g` the same either way.
#[derive(Clone, Copy, Debug)]
struct H2<T> {
    v: T,
    g: [T; 3],
    h: [[T; 3]; 3],
    second: bool,
}

impl<T: Num> H2<T> {
    fn cst(v: T) -> Self {
        H2 { v, g: [zero(); 3], h: [[zero(); 3]; 3], second: false }
    }
    fn var(v: T, i: usize, second: bool) -> Self {
        let mut x = Self::cst(v);
        x.g[i] = T::cst(1.0);
        x.second = second;
        x
    }
    /// `f(self)`, where `f`, `f'` and `f''` at the value are given.
    fn chain(self, f0: T, f1: T, f2: T) -> Self {
        let mut r = Self::cst(f0);
        r.second = self.second;
        for i in 0..3 {
            r.g[i] = f1 * self.g[i];
            for j in (i..3).filter(|_| self.second) {
                let x = f1 * self.h[i][j] + f2 * self.g[i] * self.g[j];
                r.h[i][j] = x;
                r.h[j][i] = x;
            }
        }
        r
    }
    fn sin_cos(self) -> (Self, Self) {
        let (s, c) = self.v.sin_cos();
        (self.chain(s, c, zero::<T>() - s), self.chain(c, zero::<T>() - s, zero::<T>() - c))
    }
    fn scale(self, k: T) -> Self {
        let h = if self.second { self.h.map(|r| r.map(|x| x * k)) } else { self.h };
        H2 { v: self.v * k, g: self.g.map(|x| x * k), h, second: self.second }
    }
    /// `h` combined term by term, where either operand carries one.
    fn hessian(self, o: Self, f: impl Fn(T, T) -> T) -> [[T; 3]; 3] {
        if !(self.second || o.second) {
            return self.h;
        }
        std::array::from_fn(|i| std::array::from_fn(|j| f(self.h[i][j], o.h[i][j])))
    }
    /// The derivative along the `i`th of the place, to first order.
    fn d(&self, i: usize) -> H1<T> {
        H1 { v: self.g[i], g: self.h[i] }
    }
}

impl<T: Num> Add for H2<T> {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        H2 {
            v: self.v + o.v,
            g: std::array::from_fn(|i| self.g[i] + o.g[i]),
            h: self.hessian(o, |a, b| a + b),
            second: self.second || o.second,
        }
    }
}
impl<T: Num> Sub for H2<T> {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        H2 {
            v: self.v - o.v,
            g: std::array::from_fn(|i| self.g[i] - o.g[i]),
            h: self.hessian(o, |a, b| a - b),
            second: self.second || o.second,
        }
    }
}
impl<T: Num> Mul for H2<T> {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        let mut r = H2::cst(self.v * o.v);
        r.second = self.second || o.second;
        for i in 0..3 {
            r.g[i] = self.v * o.g[i] + self.g[i] * o.v;
            for j in (i..3).filter(|_| r.second) {
                let x = self.v * o.h[i][j] + self.h[i][j] * o.v + self.g[i] * o.g[j] + self.g[j] * o.g[i];
                r.h[i][j] = x;
                r.h[j][i] = x;
            }
        }
        r
    }
}

/// A number and its first derivatives in the place.
#[derive(Clone, Copy, Debug)]
struct H1<T> {
    v: T,
    g: [T; 3],
}

impl<T: Num> H1<T> {
    fn chain(self, f0: T, f1: T) -> Self {
        H1 { v: f0, g: self.g.map(|x| f1 * x) }
    }
    fn sqrt(self) -> Self {
        let s = self.v.sqrt();
        let d = if s.val() > 0.0 { T::cst(0.5) / s } else { zero() };
        self.chain(s, d)
    }
    fn recip(self) -> Self {
        let r = T::cst(1.0) / self.v;
        self.chain(r, zero::<T>() - r * r)
    }
}

impl<T: Num> Add for H1<T> {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        H1 { v: self.v + o.v, g: std::array::from_fn(|i| self.g[i] + o.g[i]) }
    }
}
impl<T: Num> Sub for H1<T> {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        H1 { v: self.v - o.v, g: std::array::from_fn(|i| self.g[i] - o.g[i]) }
    }
}
impl<T: Num> Mul for H1<T> {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        H1 { v: self.v * o.v, g: std::array::from_fn(|i| self.v * o.g[i] + self.g[i] * o.v) }
    }
}

/* -- vectors, over whatever adds and multiplies ------------------------------------------------- */

pub(crate) fn dot<F: Copy + Add<Output = F> + Mul<Output = F>>(a: [F; 3], b: [F; 3]) -> F {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub(crate) fn cross<F: Copy + Sub<Output = F> + Mul<Output = F>>(a: [F; 3], b: [F; 3]) -> [F; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub(crate) fn vadd<F: Copy + Add<Output = F>>(a: [F; 3], b: [F; 3]) -> [F; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub(crate) fn vsub<F: Copy + Sub<Output = F>>(a: [F; 3], b: [F; 3]) -> [F; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub(crate) fn vscale<F: Copy + Mul<Output = F>>(a: [F; 3], k: F) -> [F; 3] {
    [a[0] * k, a[1] * k, a[2] * k]
}

/// A unit vector along `x`; `x` itself where it has no length (a degenerate figure, whose rows
/// read as whatever it is rather than as NaN).
pub(crate) fn unit<T: Num>(x: [T; 3]) -> [T; 3] {
    let n = dot(x, x).sqrt();
    if n.val() > 0.0 {
        vscale(x, T::cst(1.0) / n)
    } else {
        x
    }
}

pub(crate) fn col3<T: Num>(c: &[T], at: usize) -> [T; 3] {
    [c[at], c[at + 1], c[at + 2]]
}
fn lift<T: Num>(x: [T; 3]) -> [H2<T>; 3] {
    x.map(H2::cst)
}

/* -- the source, the motion, the moving point ------------------------------------------------ */

/// How far from the axis, as the sine of the angle between them, a source's reference must stand
/// to say where its turn starts (`across`).
const REFERENCE_LEAST: f64 = 0.5;

/// Where a source's turn round its axis `d` starts: its reference (`SetShape::reference`) square
/// to the axis.  The reference is read once, before the solve has placed the axis, so the solve
/// can turn the axis onto it (a cutter's spine read off an unseeded pose, then stood up square to
/// the pitch plane along the very direction it was read as not running along); square to an
/// axis it nearly runs along, it says nowhere, and which way rounding tips it moves every place
/// a quarter turn.  So a reference within 30° of the axis gives way to the first of the world's
/// axes the axis does not nearly run along — the same one wherever the axis stands near there.
fn across<T: Num>(d: [T; 3], reference: [f64; 3]) -> [T; 3] {
    let off = |r: [T; 3]| vsub(r, vscale(d, dot(r, d)));
    let w = off(reference.map(T::cst));
    if dot(w, w).val() >= REFERENCE_LEAST * REFERENCE_LEAST {
        return unit(w);
    }
    let k = (0..3).find(|&i| d[i].val().abs() < 0.9).unwrap_or(0);
    unit(off(std::array::from_fn(|i| T::cst(if i == k { 1.0 } else { 0.0 }))))
}

/// The source's point at place `(u, v)`: its meridian element at `u`, turned `v` about its axis.
fn surface<T: Num>(l: &Layout, s: &[T], u: H2<T>, v: H2<T>) -> [H2<T>; 3] {
    let a = col3(s, 0);
    let b = if l.axis == 1 { vadd(a, col3(s, 3)) } else { col3(s, 3) };
    let d = unit(vsub(b, a));
    let e1 = across(d, l.reference);
    let number = || {
        let (c, m, free) = l.number;
        let x = if free { s[l.source.width()] } else { zero() };
        T::cst(c) + T::cst(m) * x
    };
    if l.source == Source::Plane {
        // the plane through `o` moved its number along the axis, `u` and `v` along it
        let o = vadd(col3(s, 6), vscale(d, number()));
        let e2 = cross(d, e1);
        return vadd(lift(o), vadd(vscale(lift(e1), u), vscale(lift(e2), v)));
    }
    let meridian: [H2<T>; 3] = match l.source {
        Source::Line => {
            let (p1, p2) = (col3(s, 6), col3(s, 9));
            vadd(lift(p1), vscale(lift(vsub(p2, p1)), u))
        }
        Source::Cone => {
            let (sh, ch) = number().sin_cos();
            let g = vadd(vscale(d, ch), vscale(e1, sh));
            vadd(lift(a), vscale(lift(g), u))
        }
        Source::Cylinder => {
            let r = number();
            vadd(lift(vadd(a, vscale(e1, r))), vscale(lift(d), u))
        }
        Source::Round => {
            let (c, r) = (col3(s, 6), s[9]);
            let uo = unit(col3(s, 10));
            let vv = col3(s, 13);
            let vo = unit(vsub(vv, vscale(uo, dot(vv, uo))));
            let (su, cu) = u.sin_cos();
            let rr = H2::cst(r);
            vadd(lift(c), vadd(vscale(lift(uo), cu * rr), vscale(lift(vo), su * rr)))
        }
        Source::Plane => unreachable!("a plane is no revolution"),
    };
    // Rodrigues: w cos v + (d × w) sin v + d (d·w)(1 − cos v)
    let w = vsub(meridian, lift(a));
    let dd = lift(d);
    let (sv, cv) = v.sin_cos();
    let along = dot(dd, w) * (H2::cst(T::cst(1.0)) - cv);
    let turned = vadd(vadd(vscale(w, cv), vscale(cross(dd, w), sv)), vscale(dd, along));
    vadd(lift(a), turned)
}

/// A rigid pose over the roll: `x ↦ r·x + p`.
#[derive(Clone, Copy)]
struct Pose<T> {
    r: [[H2<T>; 3]; 3],
    p: [H2<T>; 3],
}

impl<T: Num> Pose<T> {
    fn apply(&self, x: [H2<T>; 3]) -> [H2<T>; 3] {
        std::array::from_fn(|i| self.r[i][0] * x[0] + self.r[i][1] * x[1] + self.r[i][2] * x[2] + self.p[i])
    }
    /// `observer⁻¹ ∘ source`.
    fn relative(source: &Self, observer: &Self) -> Self {
        let rt = |i: usize, j: usize| observer.r[j][i];
        let r = std::array::from_fn(|i| std::array::from_fn(|j| {
            rt(i, 0) * source.r[0][j] + rt(i, 1) * source.r[1][j] + rt(i, 2) * source.r[2][j]
        }));
        let q = vsub(source.p, observer.p);
        let p = std::array::from_fn(|i| rt(i, 0) * q[0] + rt(i, 1) * q[1] + rt(i, 2) * q[2]);
        Pose { r, p }
    }
}

/// The motion's pose at roll `t` (the place's third), as `motion::Family::at` reads it — a driven
/// step at its law's value of `t` (#398 G); where it has no step (a formed flank, #283), the
/// source standing still, and where a law has no value at `t`, no pose (NaN).
fn pose<T: Num>(l: &Layout, m: &[T], t: H2<T>) -> Pose<T> {
    let (one, o) = (H2::cst(T::cst(1.0)), H2::cst(zero()));
    if l.steps.is_empty() {
        return Pose { r: [[one, o, o], [o, one, o], [o, o, one]], p: [o; 3] };
    }
    poses(&l.steps, &l.laws, m, t).and_then(|mut p| p.pop()).unwrap_or_else(|| {
        let nan = H2::cst(T::cst(f64::NAN));
        Pose { r: [[nan; 3]; 3], p: [nan; 3] }
    })
}

/// A rigid pose's rotation and translation, as numbers.
pub(crate) type Frame<T> = ([[T; 3]; 3], [T; 3]);

/// Every step's pose at roll `t`, as numbers: what a meshing reads of its motion at two rolls
/// (#283, `envelope::meshing`).
pub(crate) fn frames<T: Num>(steps: &[Step], m: &[T], t: T) -> Vec<Frame<T>> {
    poses(steps, &[], m, H2::cst(t)).unwrap_or_default().into_iter()
        .map(|p| (p.r.map(|r| r.map(|x| x.v)), p.p.map(|x| x.v))).collect()
}

/// Where the motion's pose at roll `t` (radians) carries `x`, and how fast that point runs a
/// radian of the roll there (#365): `Pose(t)·x` and `∂(Pose(t)·x)/∂t` — a driven step standing
/// at its law's value of `t` (#398 G).  `None` where the motion has no step, or a law no value
/// at `t`.
pub(crate) fn carried<T: Num>(steps: &[Step], laws: &[Driven], m: &[T], t: T, x: [T; 3])
                              -> Option<([T; 3], [T; 3])> {
    let p = poses(steps, laws, m, H2::var(t, 0, false))?.pop()?;
    let y = p.apply(x.map(H2::cst));
    Some((y.map(|c| c.v), y.map(|c| c.g[0])))
}

/// **The roll a driven step stands at** (#398 G): the law's value of the roll `t` (radians in,
/// radians out) with its derivatives along the place.  The law's series about `t`'s value is
/// read in degrees (`Drive::series`, a Taylor jet: nothing is differenced) and turned to radians
/// — the `k`-th coefficient times `(180/π)^(k−1)` — and composed with what `t` carries besides
/// its value, so a column the roll is (an owned roll) reads the law's rate, and its second
/// order where the place asks for one.
fn law_of<T: Num>(law: &crate::generate::Drive, t: H2<T>) -> Option<H2<T>> {
    use crate::taylor::Jet;
    let deg = 180.0 / std::f64::consts::PI;
    let v0 = t.v.val();
    let b = law.series(Jet::var(v0 * deg))?;
    let a = Jet(std::array::from_fn(|k| b.0[k] * deg.powi(k as i32 - 1)));
    // `L(v)`, `L'(v)` and `L''(v)` at `t`'s value with what it carries besides: each series read
    // by Horner in `δ = v − v₀`, which is zero in value
    let delta = t.v - T::cst(v0);
    let at = |c: Jet| c.0.iter().rev().fold(T::cst(0.0), |acc, &c| acc * delta + T::cst(c));
    Some(t.chain(at(a), at(a.derivative()), at(a.derivative().derivative())))
}

/// Each step's pose at roll `t`, in order — a driven step's (`laws`) at its law's value of `t`.
/// `None` where a law has no value at `t`.
fn poses<T: Num>(steps: &[Step], laws: &[Driven], m: &[T], t: H2<T>) -> Option<Vec<Pose<T>>> {
    let tau = std::f64::consts::TAU;
    let mut poses: Vec<Pose<T>> = Vec::with_capacity(steps.len());
    for (i, s) in steps.iter().enumerate() {
        let t = match laws.iter().find(|d| d.step == i) {
            Some(d) => law_of(&d.law, t)?,
            None => t,
        };
        let next = match *s {
            Step::Rotation { .. } | Step::Turn { .. } => {
                let (ratio, advance) = s.numbers(m);
                let (a, d) = match *s {
                    Step::Rotation { at, dir, .. } => {
                        let a = col3(m, at);
                        (a, unit(if dir { col3(m, at + 3) } else { vsub(col3(m, at + 3), a) }))
                    }
                    Step::Turn { at, .. } => {
                        (col3(m, at), unit(cross(col3(m, at + 3), col3(m, at + 6))))
                    }
                    _ => unreachable!(),
                };
                let theta = t.scale(ratio);
                let (st, ct) = theta.sin_cos();
                let one_c = H2::cst(T::cst(1.0)) - ct;
                let k = [[zero(), zero::<T>() - d[2], d[1]], [d[2], zero(), zero::<T>() - d[0]],
                         [zero::<T>() - d[1], d[0], zero()]];
                let r: [[H2<T>; 3]; 3] = std::array::from_fn(|i| std::array::from_fn(|j| {
                    let id = if i == j { ct } else { H2::cst(zero()) };
                    id + st.scale(k[i][j]) + one_c.scale(d[i] * d[j])
                }));
                let al = lift(a);
                let ra: [H2<T>; 3] = std::array::from_fn(|i| r[i][0] * al[0] + r[i][1] * al[1] + r[i][2] * al[2]);
                let screw = t.scale(advance / T::cst(tau));
                let p = vadd(vsub(al, ra), vscale(lift(d), screw));
                Pose { r, p }
            }
            Step::Translation { at, dir, .. } => {
                let (_, advance) = s.numbers(m);
                let a = col3(m, at);
                let d = unit(if dir { col3(m, at + 3) } else { vsub(col3(m, at + 3), a) });
                let one = H2::cst(T::cst(1.0));
                let o = H2::cst(zero());
                let r = [[one, o, o], [o, one, o], [o, o, one]];
                Pose { r, p: vscale(lift(d), t.scale(advance / T::cst(tau))) }
            }
            Step::Relative { source, observer } => Pose::relative(&poses[source], &poses[observer]),
        };
        poses.push(next);
    }
    Some(poses)
}

/// What the rows read at a place: the moving point, its normal and the meshing equation, each
/// with its derivatives along the place.
pub(crate) struct Local<T> {
    pub x: [T; 3],
    /// `∂X/∂u`, `∂X/∂v`, `∂X/∂t`.
    pub dx: [[T; 3]; 3],
    /// The unit normal of the moving source.
    pub n: [T; 3],
    pub dn: [[T; 3]; 3],
    /// `n · ∂X/∂t`: zero on the envelope.
    pub g: T,
    pub dg: [T; 3],
}

/// The local frame at the place in columns `lead..lead + 3`; `dn` and `dg` only where `second`
/// asks for them (a curvature), else zero.
pub(crate) fn local<T: Num>(l: &Layout, cols: &[T], second: bool) -> Local<T> {
    let t = H2::var(cols[l.lead + 2], 2, second);
    local_at(l, cols, second, &pose(l, &cols[l.lead + 3 + l.source_width()..], t))
}

/// The motion's pose at roll `t`, read once for a grid of places at that roll (#345,
/// `envelope::regular`'s search): `cols` as `local` reads them, the place's roll aside.
pub(crate) struct Posed<T>(Pose<T>);

pub(crate) fn posed<T: Num>(l: &Layout, cols: &[T], t: T) -> Posed<T> {
    Posed(pose(l, &cols[l.lead + 3 + l.source_width()..], H2::var(t, 2, false)))
}

/// `local` at a place of the roll `posed` was read at, first order: the point, the normal and
/// the meshing equation's value.
pub(crate) fn local_posed<T: Num>(l: &Layout, cols: &[T], posed: &Posed<T>) -> Local<T> {
    local_at(l, cols, false, &posed.0)
}

/// The source's own point at the place's `(u, v)`, standing as drawn: where a tool's region is
/// read of it (#345, `envelope::regular`).
pub(crate) fn source_at<T: Num>(l: &Layout, cols: &[T]) -> [T; 3] {
    let at = l.lead;
    let (u, v) = (H2::cst(cols[at]), H2::cst(cols[at + 1]));
    surface(l, &cols[at + 3..at + 3 + l.source_width()], u, v).map(|c| c.v)
}

/// `local`, the motion's pose at the place's roll given: a seed's grid asks many places at one
/// roll (`seed`).
fn local_at<T: Num>(l: &Layout, cols: &[T], second: bool, pose: &Pose<T>) -> Local<T> {
    let at = l.lead;
    let var = |i: usize| H2::var(cols[at + i], i, second);
    let (u, v) = (var(0), var(1));
    let s = &cols[at + 3..at + 3 + l.source_width()];
    let x = pose.apply(surface(l, s, u, v));
    let d = |i: usize| -> [H1<T>; 3] { [x[0].d(i), x[1].d(i), x[2].d(i)] };
    let (xu, xv, xt) = (d(0), d(1), d(2));
    let nn = cross(xu, xv);
    let len = dot(nn, nn).sqrt();
    let len = if len.v.val() > 0.0 { len } else { H1 { v: T::cst(1.0), g: [zero(); 3] } };
    let inv = len.recip();
    let n = nn.map(|c| c * inv);
    let g = dot(n, xt);
    Local {
        x: x.map(|c| c.v),
        dx: std::array::from_fn(|i| [x[0].g[i], x[1].g[i], x[2].g[i]]),
        n: n.map(|c| c.v),
        dn: std::array::from_fn(|i| [n[0].g[i], n[1].g[i], n[2].g[i]]),
        g: g.v,
        dg: g.g,
    }
}

/// Two directions of the place square to `∇g` — so along the envelope — chosen by `∇g`'s largest
/// component, so the choice holds still near a place.
pub(crate) fn along_envelope<T: Num>(g: [T; 3]) -> [[T; 3]; 2] {
    let big = (0..3).max_by(|&i, &j| g[i].val().abs().total_cmp(&g[j].val().abs())).unwrap_or(2);
    let z = zero::<T>();
    let neg = |x: T| z - x;
    match big {
        0 => [[neg(g[1]), g[0], z], [neg(g[2]), z, g[0]]],
        1 => [[g[1], neg(g[0]), z], [z, neg(g[2]), g[1]]],
        _ => [[g[2], z, neg(g[0])], [z, g[2], neg(g[1])]],
    }
}

/// A derivative along the place's direction `k`: `d` its derivatives along the place's three.
pub(crate) fn along<T: Num>(d: &[[T; 3]; 3], k: [T; 3]) -> [T; 3] {
    vadd(vadd(vscale(d[0], k[0]), vscale(d[1], k[1])), vscale(d[2], k[2]))
}

/// **The envelope's frame** at the place in columns `lead..lead + 3` (#365, `contact_rate`): the
/// point, `∂X/∂u` to read a normal across, the unit normal, and — where `second` — two directions
/// along the envelope (`along_envelope`) and its second fundamental form along them, from the
/// normal's derivatives.
pub(crate) fn frame<T: Num>(l: &Layout, cols: &[T], second: bool) -> crate::contact_rate::Frame<T> {
    let f = local(l, cols, second);
    let mut out = crate::contact_rate::Frame::zero();
    out.x = f.x;
    out.across = f.dx[0];
    out.n = f.n;
    if second {
        let neg = |x: T| zero::<T>() - x;
        let [k1, k2] = along_envelope(f.dg);
        let (t1, t2) = (along(&f.dx, k1), along(&f.dx, k2));
        let (n1, n2) = (along(&f.dn, k1), along(&f.dn, k2));
        let half = T::cst(0.5);
        out.tan = [t1, t2];
        out.second = [neg(dot(n1, t1)), neg((dot(n1, t2) + dot(n2, t1)) * half), neg(dot(n2, t2))];
    }
    out
}

/// A contact's rows over its columns `cols` and constants `k`, into `out`.  `None` where the
/// constants do not describe a contact over this many columns.
pub(crate) fn rows<T: Num>(c: Contact, cols: &[T], k: &[f64], out: &mut [T]) -> Option<()> {
    let l = Layout::read(k)?;
    if l.lead != c.lead() || cols.len() != l.width() || out.len() != c.rows() {
        return None;
    }
    let f = local(&l, cols, false);
    match c {
        Contact::Point => point_rows(cols, &f, out),
    }
    Some(())
}

/// A point contact's rows: the point less the moving point, and the meshing equation.
fn point_rows<T: Num>(cols: &[T], f: &Local<T>, out: &mut [T]) {
    let p = col3(cols, 0);
    for i in 0..3 {
        out[i] = p[i] - f.x[i];
    }
    out[3] = f.g;
}

/* -- the kernels ------------------------------------------------------------------------------ */

/// Columns differentiated per pass.
pub(crate) const PASS: usize = 8;

fn res<const C: u8>(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let c = Contact::ALL[C as usize];
    block_res(n, v, k, r, c.rows(), |x, kc, out| rows(c, x, kc, out));
}

fn jac<const C: u8>(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let c = Contact::ALL[C as usize];
    block_jac(n, v, k, j, c.rows(), |x, kc, out| rows(c, x, kc, out));
}

/// A kernel's `m` outputs a block over `n` blocks, each written by `rows` over its block's columns
/// and constants — a kernel built at its block's widths (`curve_widths`), NaN where `rows` cannot
/// read them: the envelope contacts' and a point on a meshing's (#283) residuals, and with `m`
/// their rows times their columns, their Jacobians (`block_jac`, `meshing::jac`).
pub(crate) fn block_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64], m: usize,
                        mut rows: impl FnMut(&[f64], &[f64], &mut [f64]) -> Option<()>) {
    let (n_par, n_const) = curve_widths(n, v, k);
    for i in 0..n {
        let out = &mut r[m * i..m * (i + 1)];
        if rows(&v[n_par * i..n_par * (i + 1)], &k[n_const * i..n_const * (i + 1)], out).is_none() {
            out.fill(f64::NAN);
        }
    }
}

/// `block_res`'s Jacobian, exact by forward differentiation, `PASS` columns a pass.
pub(crate) fn block_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64], m: usize,
                        rows: impl Fn(&[Dual<PASS>], &[f64], &mut [Dual<PASS>]) -> Option<()>) {
    let (n_par, _) = curve_widths(n, v, k);
    let mut cols = vec![Dual::<PASS>::cst(0.0); n_par];
    let mut out = vec![Dual::<PASS>::cst(0.0); m];
    block_res(n, v, k, j, m * n_par, |x, kc, block| {
        for start in (0..n_par).step_by(PASS) {
            seed_pass(&mut cols, x, start);
            rows(&cols, kc, &mut out)?;
            scatter_pass(&out, block, start);
        }
        Some(())
    });
}

/// `x` as numbers carrying their derivatives in columns `start..start + PASS`.
pub(crate) fn seed_pass(cols: &mut [Dual<PASS>], x: &[f64], start: usize) {
    for (q, col) in cols.iter_mut().enumerate() {
        let seeded = (start..start + PASS).contains(&q);
        *col = if seeded { Dual::var(x[q], q - start) } else { Dual::cst(x[q]) };
    }
}

/// A pass's rows `out` into the row-major Jacobian `j`, at columns `start..start + PASS`.
pub(crate) fn scatter_pass(out: &[Dual<PASS>], j: &mut [f64], start: usize) {
    let w = j.len() / out.len();
    for (row, val) in out.iter().enumerate() {
        for q in start..(start + PASS).min(w) {
            j[row * w + q] = val.g[q - start];
        }
    }
}

/// The kernel of contact `c` at a block's widths: built per width (`system::build_kernel`), as a
/// curve's are per definition, its widths read back off the slices it is handed.
pub fn kernel(c: Contact, n_par: usize, n_const: usize) -> Kernel {
    let (res, jac): (crate::kernels::KernelFn, crate::kernels::KernelFn) = match c {
        Contact::Point => (res::<0>, jac::<0>),
    };
    Kernel { name: c.name(), n_res: c.rows(), n_par, n_const, degree: c.degree(), res, jac, const_jac: None, form: None }
}

/* -- reading a place, apart from the solve ----------------------------------------------------- */

/// The moving point, its unit normal and `n · ∂X/∂t` at a place, over a source's and a motion's
/// columns as numbers: what a seed and a check read.
#[derive(Clone, Copy, Debug)]
pub struct Reading {
    pub position: [f64; 3],
    pub normal: [f64; 3],
    pub velocity: [f64; 3],
    pub normal_velocity: f64,
}

/// Read the envelope's frame at `place` over `parts` — the source's columns, then the motion's —
/// with any contact's constants `k` (its lead is left zero).
pub fn read(k: &[f64], parts: &[f64], place: [f64; 3]) -> Option<Reading> {
    let l = Layout::read(k)?;
    if parts.len() != l.source_width() + l.motion_width() {
        return None;
    }
    let mut cols = vec![0.0; l.lead];
    cols.extend(place);
    cols.extend_from_slice(parts);
    let f = local(&l, &cols, false);
    let all = f.x.iter().chain(f.n.iter()).chain(f.dx.iter().flatten()).all(|x| x.is_finite());
    all.then_some(Reading {
        position: f.x,
        normal: f.n,
        velocity: f.dx[2],
        normal_velocity: f.g,
    })
}

/// Where a solve holding `p` to the envelope starts: the place over a coarse grid — rolls across
/// the interval, turns round the axis, a few spans of the meridian — whose point and meshing
/// equation come nearest `p` and zero, then `newton::dogleg` on those four rows together (a
/// least-squares fit where `p` is off the envelope). `None` where nothing is finite. A seed, not a
/// projection: no claim it is the nearest root.
pub fn seed(k: &[f64], parts: &[f64], p: [f64; 3], roll: [f64; 2]) -> Option<[f64; 3]> {
    let l = Layout::read(k)?;
    if l.lead != Contact::Point.lead() || parts.len() != l.source_width() + l.motion_width() {
        return None;
    }
    let mut tr = Seeding { k, cols: vec![0.0; l.lead + 3 + parts.len()], j: crate::linalg::Mat::zeros(4, 3) };
    tr.cols[..3].copy_from_slice(&p);
    tr.cols[l.lead + 3..].copy_from_slice(parts);
    let cost = |r: [f64; 4]| r.iter().map(|x| x * x).sum::<f64>();
    let reach = reach(l.source, parts, p);
    let us = meridian_spans(l.source, reach);
    let vs = turn_spans(l.source, reach, 24);
    let mut best: Option<([f64; 3], f64)> = None;
    let n_roll = 25;
    for a in 0..n_roll {
        let t = roll[0] + (roll[1] - roll[0]) * a as f64 / (n_roll - 1) as f64;
        // the motion's pose at this roll, read once for every place of it
        let posed = pose(&l, &tr.cols[l.lead + 3 + l.source_width()..], H2::var(t, 2, false));
        for &v in &vs {
            for &u in &us {
                let r = tr.posed(&l, [u, v, t], &posed);
                if r.iter().all(|x| x.is_finite()) && best.is_none_or(|(_, c)| cost(r) < c) {
                    best = Some(([u, v, t], cost(r)));
                }
            }
        }
    }
    let (start, _) = best?;
    let mut z = start.to_vec();
    let mut r = tr.at(start).to_vec();
    let tol = crate::newton::Tol { ftol: 1e-12 * (1.0 + reach), xtol: 1e-14, gtol: 1e-14 };
    crate::newton::dogleg(&mut tr, &mut z, &mut r, tol, 100, 400);
    let x = [z[0], z[1], z[2]];
    tr.at(x).iter().all(|v| v.is_finite()).then_some(x)
}

/// Where a seed's coarse grid samples a source's meridian: a fraction of a line, a turn round a
/// circle, and along a cone's or a cylinder's generator as far either way as `reach`, how far the
/// point sought is from the axis's start.
pub(crate) fn meridian_spans(source: Source, reach: f64) -> Vec<f64> {
    match source {
        Source::Line => vec![-0.5, 0.0, 0.25, 0.5, 0.75, 1.0, 1.5],
        Source::Round => (0..12).map(|i| i as f64 * std::f64::consts::TAU / 12.0).collect(),
        Source::Cone | Source::Cylinder | Source::Plane => {
            (-6..=6).map(|i| i as f64 * reach / 3.0).collect()
        }
    }
}

/// Where a seed's coarse grid samples round a source's axis, `n` turns of a revolution — or, on a
/// plane, its second direction as its first is sampled.
pub(crate) fn turn_spans(source: Source, reach: f64, n: usize) -> Vec<f64> {
    match source {
        Source::Plane => meridian_spans(source, reach),
        _ => (0..n).map(|b| b as f64 * std::f64::consts::TAU / n as f64).collect(),
    }
}

/// How far a point sought, `p`, is from where a source's place starts — its axis's start, or a
/// plane's point — over the source's columns `parts`: how far a seed's grid reaches.
pub(crate) fn reach(source: Source, parts: &[f64], p: [f64; 3]) -> f64 {
    let at = if source == Source::Plane { 6 } else { 0 };
    crate::space::distance(p, [parts[at], parts[at + 1], parts[at + 2]]).max(1e-3)
}

/// A point contact's four rows over its place alone, the rest of its columns held: what `seed`
/// walks downhill on. Its Jacobian is the kernel's, over three tangents.
struct Seeding<'a> {
    k: &'a [f64],
    cols: Vec<f64>,
    j: crate::linalg::Mat,
}

impl Seeding<'_> {
    fn at(&mut self, place: [f64; 3]) -> [f64; 4] {
        self.cols[3..6].copy_from_slice(&place);
        let mut out = [0.0; 4];
        match rows::<f64>(Contact::Point, &self.cols, self.k, &mut out) {
            Some(()) => out,
            None => [f64::INFINITY; 4],
        }
    }

    /// `at`, the motion's pose at the place's roll given (`l` read off the seed's own constants,
    /// which `seed` has checked describe a point contact over these columns).
    fn posed(&mut self, l: &Layout, place: [f64; 3], pose: &Pose<f64>) -> [f64; 4] {
        self.cols[3..6].copy_from_slice(&place);
        let mut out = [0.0; 4];
        point_rows(&self.cols, &local_at(l, &self.cols, false, pose), &mut out);
        out
    }
}

impl crate::newton::TrustRegion for Seeding<'_> {
    fn n(&self) -> usize {
        3
    }
    fn m(&self) -> usize {
        4
    }
    fn residuals_into(&mut self, z: &[f64], out: &mut [f64]) {
        let r = self.at([z[0], z[1], z[2]]);
        // a trial off the finite numbers is refused, the region shrunk
        let finite = r.iter().all(|x| x.is_finite());
        out.copy_from_slice(&if finite { r } else { [f64::INFINITY; 4] });
    }
    fn jacobian_at(&mut self, z: &[f64]) {
        let cols: Vec<Dual<3>> = self.cols.iter().enumerate().map(|(q, &x)| match q {
            3..=5 => Dual::var(z[q - 3], q - 3),
            _ => Dual::cst(x),
        }).collect();
        let mut out = [Dual::<3>::cst(0.0); 4];
        let ok = rows(Contact::Point, &cols, self.k, &mut out).is_some();
        for (row, v) in out.iter().enumerate() {
            for q in 0..3 {
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
        crate::newton::dense_gn_step(&self.j, r, p, 1e-12);
    }
}

/* -- reading the sketch ------------------------------------------------------------------------ */

use crate::model::{EntKind, Held, MotionDef, SetKind, Sketch, TermNumber};

/// What a contact reads of an envelope, off the sketch: the source and the motion as constants,
/// and what each of their columns is.
#[derive(Clone, Debug)]
pub struct Parts {
    pub source: Source,
    /// The source's axis as two points (0) or a point and a direction (1).
    pub axis: u8,
    pub number: (f64, f64, bool),
    pub reference: [f64; 3],
    pub steps: Vec<Step>,
    /// Each driven step and its law (#398 G): the envelope generated under a drive.
    pub laws: Vec<(usize, crate::motion::Law)>,
    cols: Vec<Col>,
}

/// One part of the columns.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Col {
    /// A point's lift: three columns.
    Lift(usize),
    /// One number.
    One(u32),
    /// Three numbers: an axis's place or direction.
    Three([u32; 3]),
}

impl Parts {
    /// Envelope `index`'s parts, or why a contact cannot read it: its motion read as a carried
    /// point's is (#398 G), a turn about a point of a view as the turn about the line square to
    /// the view and a driven joint at its law.
    pub fn of(sk: &Sketch, index: usize) -> Result<Parts, String> {
        let e = sk.envelopes.get(index).ok_or("no such envelope")?;
        let mut parts = Parts::still(sk, &e.held, &e.name)?;
        let mut done = std::collections::BTreeMap::new();
        motion(sk, e.motion as usize, &mut parts.steps, &mut parts.cols, &mut done, 0, true,
               Some(&mut parts.laws))?;
        Ok(parts)
    }

    /// The parts of a source the solve reads standing still — a formed flank (#283,
    /// `envelope::meshing`): its own columns and no step.  `name` is what a set's refusal says
    /// stated nothing at its point.
    pub fn still(sk: &Sketch, held: &Held, name: &str) -> Result<Parts, String> {
        let mut cols = Vec::new();
        let ends = |l: u32| -> Result<[usize; 2], String> {
            let l = sk.lines.get(l as usize).ok_or("no such line")?;
            Ok([l.p1 as usize, l.p2 as usize])
        };
        let (source, axis, number, reference) = match held {
            Held::Not(why) => return Err(why.clone()),
            Held::Revolution { axis, edge } => {
                let form = match axis.kind {
                    EntKind::Line => {
                        cols.extend(ends(axis.idx)?.map(Col::Lift));
                        0
                    }
                    EntKind::Axis => {
                        let x = sk.axes.get(axis.i()).ok_or("no such axis")?;
                        cols.extend([Col::Three(x.a), Col::Three(x.d)]);
                        1
                    }
                    _ => return Err("a revolution turns about a line or an axis".into()),
                };
                let source = match edge.kind {
                    EntKind::Line => {
                        cols.extend(ends(edge.idx)?.map(Col::Lift));
                        Source::Line
                    }
                    EntKind::Arc | EntKind::Circle => {
                        let centre = sk.round_center(*edge);
                        let plane = sk.plane_of(centre)
                            .ok_or("a round profile edge in space is drawn in a plane")?;
                        let pl = &sk.frames[plane];
                        cols.extend([Col::Lift(centre), Col::One(sk.round_radius(*edge) as u32),
                                     Col::Three(sk.axes[pl.u as usize].d), Col::Three(sk.axes[pl.v as usize].d)]);
                        Source::Round
                    }
                    _ => return Err("a revolution's surface is turned from a line, an arc or a circle".into()),
                };
                (source, form, (0.0, 0.0, false), [0.0; 3])
            }
            Held::Set { shape: None, .. } => {
                return Err(format!("`{name}`'s set states nothing at its point a surface can be \
                                    read from"))
            }
            Held::Set { shape: Some(shape), .. } => {
                let source = match shape.kind {
                    SetKind::Cone => Source::Cone,
                    SetKind::Cylinder => Source::Cylinder,
                    SetKind::Plane => Source::Plane,
                };
                cols.extend(ends(shape.axis)?.map(Col::Lift));
                if source == Source::Plane {
                    let o = shape.origin.ok_or("a plane's set names the point it is measured from")?;
                    cols.push(Col::Lift(o as usize));
                }
                let (n, free) = number(sk, &shape.number)?;
                cols.extend(free.map(Col::One));
                (source, 0, n, shape.reference)
            }
        };
        Ok(Parts { source, axis, number, reference, steps: Vec::new(), laws: Vec::new(), cols })
    }

    /// The points whose lifts the columns read: what `Sketch::add` lifts.
    pub fn lifted(&self) -> Vec<usize> {
        lifted(&self.cols)
    }

    /// The columns, every point's lift minted.
    pub fn columns(&self, sk: &Sketch) -> Vec<u32> {
        columns(sk, &self.cols)
    }

    /// The columns' values as the drawing stands, a point not lifted yet read where it is.
    pub fn values(&self, sk: &Sketch) -> Vec<f64> {
        values(sk, &self.cols)
    }

    /// A contact's constants over these parts.
    pub fn consts(&self, c: Contact) -> Vec<f64> {
        self.consts_at(c.lead())
    }

    /// The constants over these parts with a lead of `lead` columns: a flank's are `0`.
    pub fn consts_at(&self, lead: usize) -> Vec<f64> {
        consts(lead, self.source, self.axis, self.number, self.reference, &self.steps,
               &drives(&self.laws))
    }

    /// How many constants a contact reads (`consts`).
    pub fn n_const(&self) -> usize {
        match self.laws.is_empty() {
            true => HEADER + STEP * self.steps.len(),
            false => self.consts_at(0).len(),
        }
    }

    /// How many columns a contact of kind `c` reads.
    pub fn width(&self, c: Contact) -> usize {
        c.lead() + 3 + self.own_width()
    }

    /// The columns of the source and the motion alone, past the place.
    pub fn own_width(&self) -> usize {
        width(&self.cols)
    }

}

/// A motion the solve reads apart from any source (#283, `envelope::meshing`): its steps and the
/// columns of the axes they turn about or slide along.
#[derive(Clone, Debug)]
pub struct Moving {
    pub steps: Vec<Step>,
    /// Each driven step and its law (#398 G): none but where `Moving::driven` read it.
    pub laws: Vec<(usize, crate::motion::Law)>,
    cols: Vec<Col>,
}

impl Moving {
    /// Motion `m`'s steps and columns, or why the solve cannot read it.
    pub fn of(sk: &Sketch, m: usize) -> Result<Moving, String> {
        Self::read(sk, m, false, false)
    }

    /// The same, a turn about a point of a view read as the turn about the line through it square
    /// to the view (`Step::Turn`): what a relation reading the motion itself takes (#326), and an
    /// envelope in space (#398 G), where a meshing still turns its flanks about lines.
    pub fn of_any(sk: &Sketch, m: usize) -> Result<Moving, String> {
        Self::read(sk, m, true, false)
    }

    /// **`of_any`, a driven joint read too** (#398 G): its step standing at its law's value of
    /// the roll, the law carried in the constants (`encode`) — what a point carried to a roll
    /// reads, and its rate.  A law reading a number of the geometry, which the solve moves while
    /// the constants stand still, is refused (`motion::Law::held`).
    pub fn driven(sk: &Sketch, m: usize) -> Result<Moving, String> {
        Self::read(sk, m, true, true)
    }

    fn read(sk: &Sketch, m: usize, turns: bool, drives: bool) -> Result<Moving, String> {
        let (mut steps, mut cols, mut laws) = (Vec::new(), Vec::new(), Vec::new());
        motion(sk, m, &mut steps, &mut cols, &mut std::collections::BTreeMap::new(), 0, turns,
               drives.then_some(&mut laws))?;
        Ok(Moving { steps, laws, cols })
    }

    /// Its laws as the rows read them, borrowed.
    pub fn drives(&self) -> Vec<Driven<'_>> {
        drives(&self.laws)
    }

    /// The constants a relation reads the motion by (`encode_motion`).
    pub fn encode(&self) -> Vec<f64> {
        encode_motion(&self.steps, &self.drives())
    }

    pub fn lifted(&self) -> Vec<usize> {
        lifted(&self.cols)
    }

    pub fn columns(&self, sk: &Sketch) -> Vec<u32> {
        columns(sk, &self.cols)
    }

    pub fn values(&self, sk: &Sketch) -> Vec<f64> {
        values(sk, &self.cols)
    }

    pub fn width(&self) -> usize {
        width(&self.cols)
    }
}

/// Each driven step's law as the rows read it, borrowed.
fn drives(laws: &[(usize, crate::motion::Law)]) -> Vec<Driven<'_>> {
    laws.iter().map(|(step, law)| Driven { step: *step, law: law.drive() }).collect()
}

/// A number as `c + m·x` and whether `x` is a column — the unknown it reads, where it reads one.
pub(crate) fn number(sk: &Sketch, t: &TermNumber)
                     -> Result<((f64, f64, bool), Option<u32>), String> {
    match &t.free {
        None => Ok(((t.c, 0.0, false), None)),
        Some((name, m)) => match sk.free_vars.get(name) {
            Some(&p) => Ok(((t.c, *m, true), Some(p))),
            None => Err(format!("`{name}` is an unknown nothing on the drawing reads: state it, or \
                                 a relation that holds it")),
        },
    }
}

/// The points whose lifts columns read.
pub(crate) fn lifted(cols: &[Col]) -> Vec<usize> {
    cols.iter().filter_map(|c| match *c {
        Col::Lift(p) => Some(p),
        _ => None,
    }).collect()
}

/// The columns' parameters, every point's lift minted.
pub(crate) fn columns(sk: &Sketch, cols: &[Col]) -> Vec<u32> {
    cols.iter().flat_map(|c| match *c {
        Col::Lift(p) => {
            let k = sk.lift_of(p).expect("an envelope's points are lifted at the add");
            sk.lifts[k].x.to_vec()
        }
        Col::One(p) => vec![p],
        Col::Three(x) => x.to_vec(),
    }).collect()
}

/// The columns' values as the drawing stands, a point not lifted yet read where it is.
pub(crate) fn values(sk: &Sketch, cols: &[Col]) -> Vec<f64> {
    let v = |p: u32| sk.params[p as usize].value;
    cols.iter().flat_map(|c| match *c {
        Col::Lift(p) => sk.lifted(p).to_vec(),
        Col::One(p) => vec![v(p)],
        Col::Three(x) => x.map(v).to_vec(),
    }).collect()
}

pub(crate) fn width(cols: &[Col]) -> usize {
    cols.iter().map(|x| match x {
        Col::Lift(_) | Col::Three(_) => 3,
        Col::One(_) => 1,
    }).sum()
}

/// Motion `m` as steps in the order `motion::Family::read` takes them — each once, its source
/// and observer before a relative — each axis's ends appended to `cols`, six motion columns past
/// the last; returns its step.
#[allow(clippy::too_many_arguments)]
fn motion(sk: &Sketch, m: usize, steps: &mut Vec<Step>, cols: &mut Vec<Col>,
          done: &mut std::collections::BTreeMap<usize, usize>, depth: usize, turns: bool,
          mut laws: Option<&mut Vec<(usize, crate::motion::Law)>>) -> Result<usize, String> {
    if let Some(&s) = done.get(&m) {
        return Ok(s);
    }
    if depth > 64 {
        return Err("a motion nests too deeply".into());
    }
    let me = sk.motions.get(m).ok_or("no such motion")?;
    // where the next axis's columns start in the motion's block: past every axis already read
    let at = steps_width(steps);
    let mut axis = |l: u32| -> Result<(), String> {
        let line = sk.lines.get(l as usize).ok_or("no such motion axis")?;
        cols.extend([Col::Lift(line.p1 as usize), Col::Lift(line.p2 as usize)]);
        Ok(())
    };
    // a joint's numbers the solve finds are two columns past its axis's (#383 stage 5)
    let free = |cols: &mut Vec<Col>, past: usize| me.numbers.map(|[r, a]| {
        cols.extend([Col::One(r), Col::One(a)]);
        Free { at: at + past, back: false }
    });
    let step = match me.def {
        MotionDef::Rotation { axis: l, .. } => {
            let (ratio, advance) = me.rotation(sk)?;
            axis(l)?;
            Step::Rotation { at, ratio, advance, free: free(cols, 6), dir: false }
        }
        MotionDef::Translation { axis: l, .. } => {
            let advance = me.advance(sk)?;
            axis(l)?;
            Step::Translation { at, advance, free: free(cols, 6), dir: false }
        }
        MotionDef::Turn { centre, .. } if turns => {
            let (ratio, _) = me.rotation(sk)?;
            let view = sk.plane_of(centre as usize).ok_or_else(|| {
                format!("`{}` turns about a point drawn in no view, so about no line in space", me.name)
            })?;
            let pl = sk.frames.get(view).ok_or("no such view")?;
            cols.extend([Col::Lift(centre as usize), Col::Three(sk.axes[pl.u as usize].d),
                         Col::Three(sk.axes[pl.v as usize].d)]);
            Step::Turn { at, ratio, free: free(cols, 9) }
        }
        MotionDef::Turn { .. } => {
            return Err(format!("`{}` turns about a point of a view: a meshing turns its flanks \
                                about lines", me.name))
        }
        MotionDef::Relative { source, observer } => {
            let source = motion(sk, source as usize, steps, cols, done, depth + 1, turns,
                                laws.as_deref_mut())?;
            let observer = motion(sk, observer as usize, steps, cols, done, depth + 1, turns,
                                  laws.as_deref_mut())?;
            Step::Relative { source, observer }
        }
        // a joint about an axis no drawn line holds — a frame's own, `f turns(r) P.normal` —
        // turns about that axis as the solve places it: its point and its direction (#366); a
        // ratio stated 0 slides along it
        MotionDef::Joint { axis: x, ratio, advance } => {
            let ax = sk.axes.get(x as usize).ok_or("no such motion axis")?;
            cols.extend([Col::Three(ax.a), Col::Three(ax.d)]);
            let free = free(cols, 6);
            match ratio == 0.0 && me.numbers.is_none() {
                true => Step::Translation { at, advance, free, dir: true },
                false => Step::Rotation { at, ratio, advance, free, dir: true },
            }
        }
        // a row reads no march (#398 F): a frame the drawing moves is read off it after the solve
        MotionDef::Followed { ref over, .. } => {
            return Err(crate::motion::unread_march(&me.name, over))
        }
        // a row reads no guide (#398 C): a frame along a path is read off the path after the solve
        MotionDef::Guided { .. } => {
            return Err(format!("`{}` rides along a path, its frame read off the path after the \
                solve: a relation the solve holds reads no frame along a path — sweep what rides \
                on it as the parameter runs, `sweep(sec over s)`", me.name))
        }
        // a driven frame (#391 S5): where a point carried to a roll or an envelope in space reads
        // it (#398 G), the joint's own step again over the same axis columns, standing at the
        // law's value of the roll, the law carried in the constants; a meshing's own motion
        // reads no law
        MotionDef::Driven { of, ref law, ref over } => {
            let f = sk.motions.get(of as usize).map_or(me.name.as_str(), |j| j.name.as_str());
            let Some(laws) = laws.as_deref_mut() else {
                return Err(crate::motion::unread_drive(f, over))
            };
            let law = crate::motion::Law::held(sk, law, over).map_err(|m| format!("`{f}`: {m}"))?;
            let k = motion(sk, of as usize, steps, cols, done, depth + 1, turns, Some(&mut *laws))?;
            if matches!(steps[k], Step::Relative { .. }) {
                return Err(format!("`{}` drives no joint", me.name));
            }
            laws.push((steps.len(), law));
            steps[k]
        }
        // a joint's inverse runs back over the same axis columns, a relative's swaps its sides —
        // a driven joint's at the same law
        MotionDef::Inverse { of } => {
            let k = motion(sk, of as usize, steps, cols, done, depth + 1, turns,
                           laws.as_deref_mut())?;
            if let Some(laws) = laws.as_deref_mut() {
                if let Some(law) = laws.iter().find(|(s, _)| *s == k).map(|(_, l)| l.clone()) {
                    laws.push((steps.len(), law));
                }
            }
            let back = |f: Option<Free>| f.map(|f| Free { back: !f.back, ..f });
            match steps[k] {
                Step::Rotation { at, ratio, advance, free, dir } =>
                    Step::Rotation { at, ratio: -ratio, advance: -advance, free: back(free), dir },
                Step::Translation { at, advance, free, dir } =>
                    Step::Translation { at, advance: -advance, free: back(free), dir },
                Step::Turn { at, ratio, free } =>
                    Step::Turn { at, ratio: -ratio, free: back(free) },
                Step::Relative { source, observer } => Step::Relative { source: observer, observer: source },
            }
        }
    };
    steps.push(step);
    done.insert(m, steps.len() - 1);
    Ok(steps.len() - 1)
}
