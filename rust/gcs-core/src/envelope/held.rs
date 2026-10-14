//! **An envelope the solve sees** (#283, step 2).
//!
//! `flank := envelope(source, under: m, from: a, to: b)` over a surface of revolution — a cone or a
//! cylinder set (`std.Cone`, `std.ConeThrough`, `std.Cylinder`), or a line, arc or circle of a
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
//! are `[lead, source, axis, c, m, free, ref×3, steps, (code, at, source, observer, ratio,
//! phase, advance)×steps]` (`HEADER`, `STEP`): the source's axis as two points (`axis` 0, a line's
//! ends) or a point and a direction (1, an axis's), its number `c + m·x` (a cone's half angle, a
//! cylinder's radius; `x` its last column where `free`), a direction its axis does not run along
//! (`ref`, which fixes where a set's `v` starts), and the motion as the steps `motion::Family`
//! reads.
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
pub const STEP: usize = 7;

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

/// Which statement a contact makes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Contact {
    /// `p coincident flank`: `p − X`, and the meshing equation. Lead: the point's lift.
    Point = 0,
    /// `l tangent(at: p) flank`: the line runs in the tangent plane. Lead: two points along it.
    Tangent = 1,
    /// `l perpendicular(at: p) flank`: the line runs along the normal. Lead: two points along it.
    Normal = 2,
    /// `flank curvature(at: p) k`: the circle osculates the envelope's section by its plane at
    /// `p`, which is in that plane (drawn there, or stated).  Lead: the centre's lift, the radius,
    /// the plane's two axes.
    Curvature = 3,
    /// `P tangent(at: p) flank`: the plane's normal runs along the envelope's. Lead: the plane's
    /// two axes.
    Plane = 4,
}

impl Contact {
    pub const ALL: [Contact; 5] =
        [Contact::Point, Contact::Tangent, Contact::Normal, Contact::Curvature, Contact::Plane];

    pub fn rows(self) -> usize {
        match self {
            Contact::Point => 4,
            Contact::Tangent => 1,
            Contact::Normal | Contact::Plane => 2,
            Contact::Curvature => 3,
        }
    }

    /// The lead's width (the columns before the place).
    pub fn lead(self) -> usize {
        match self {
            Contact::Point => 3,
            Contact::Tangent | Contact::Normal | Contact::Plane => 6,
            Contact::Curvature => 10,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Contact::Point => "point_on_envelope",
            Contact::Tangent => "envelope_tangent_at",
            Contact::Normal => "envelope_normal_at",
            Contact::Plane => "envelope_plane_at",
            Contact::Curvature => "envelope_curvature_at",
        }
    }

    /// A point's and a circle's rows are lengths; a direction's are cosines.
    fn degree(self) -> u32 {
        match self {
            Contact::Point | Contact::Curvature => 1,
            Contact::Tangent | Contact::Normal | Contact::Plane => 0,
        }
    }
}

/// One step of a motion, as its constants say it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Step {
    /// About the line whose lifted ends are motion columns `at..at + 6`.
    Rotation { at: usize, ratio: f64, phase: f64, advance: f64 },
    /// Along the line whose lifted ends are motion columns `at..at + 6`.
    Translation { at: usize, advance: f64 },
    /// About the line through the lifted centre at motion columns `at..at + 3`, square to the
    /// view whose two axes' directions are columns `at + 3..at + 9`: a turn about a point of a
    /// view, as it reads in space (#326, read by `Moving::of_any` alone).
    Turn { at: usize, ratio: f64, phase: f64 },
    /// `observer⁻¹ ∘ source`, by step index.
    Relative { source: usize, observer: usize },
}

/// The constants of one contact, built from its parts (the inverse of `Layout::read`).
#[allow(clippy::too_many_arguments)]
pub fn consts(lead: usize, source: Source, axis: u8, number: (f64, f64, bool), reference: [f64; 3],
              steps: &[Step]) -> Vec<f64> {
    let mut k = vec![lead as f64, source as i64 as f64, axis as f64,
                     number.0, number.1, if number.2 { 1.0 } else { 0.0 },
                     reference[0], reference[1], reference[2], steps.len() as f64];
    k.extend(encode_steps(steps));
    k
}

/// A motion's steps as constants, `STEP` each.
pub fn encode_steps(steps: &[Step]) -> Vec<f64> {
    steps.iter().flat_map(|s| match *s {
        Step::Rotation { at, ratio, phase, advance } => [1.0, at as f64, 0.0, 0.0, ratio, phase, advance],
        Step::Translation { at, advance } => [2.0, at as f64, 0.0, 0.0, 0.0, 0.0, advance],
        Step::Turn { at, ratio, phase } => [4.0, at as f64, 0.0, 0.0, ratio, phase, 0.0],
        Step::Relative { source, observer } => [3.0, 0.0, source as f64, observer as f64, 0.0, 0.0, 0.0],
    }).collect()
}

/// Steps read back off `encode_steps`' constants: `None` where they are not steps, or a relative
/// names a step not before it.
pub fn read_steps(k: &[f64]) -> Option<Vec<Step>> {
    if k.len() % STEP != 0 {
        return None;
    }
    let mut steps = Vec::with_capacity(k.len() / STEP);
    for (i, s) in k.chunks(STEP).enumerate() {
        let idx = |x: f64| (x >= 0.0 && (x as usize) < i).then_some(x as usize);
        steps.push(match s[0] as i64 {
            1 => Step::Rotation { at: s[1] as usize, ratio: s[4], phase: s[5], advance: s[6] },
            2 => Step::Translation { at: s[1] as usize, advance: s[6] },
            4 => Step::Turn { at: s[1] as usize, ratio: s[4], phase: s[5] },
            3 => Step::Relative { source: idx(s[2])?, observer: idx(s[3])? },
            _ => return None,
        });
    }
    Some(steps)
}

/// The columns a motion's steps read: six per axis.
pub fn steps_width(steps: &[Step]) -> usize {
    steps.iter().filter_map(|s| match *s {
        Step::Rotation { at, .. } | Step::Translation { at, .. } => Some(at + 6),
        Step::Turn { at, .. } => Some(at + 9),
        Step::Relative { .. } => None,
    }).max().unwrap_or(0)
}

/// A contact's constants read back.
#[derive(Clone, Debug)]
pub struct Layout {
    pub lead: usize,
    pub source: Source,
    /// The source's axis as two points (0) or a point and a direction (1).
    pub axis: u8,
    pub number: (f64, f64, bool),
    pub reference: [f64; 3],
    pub steps: Vec<Step>,
}

impl Layout {
    /// A contact's constants read back: an envelope's, so its motion has a step.
    pub fn read(k: &[f64]) -> Option<Layout> {
        Self::read_any(k).filter(|l| !l.steps.is_empty())
    }

    /// A flank's constants read back (#283, `envelope::meshing`): an envelope's, or a formed
    /// flank's — the source itself, standing still, with no step.
    pub fn read_any(k: &[f64]) -> Option<Layout> {
        if k.len() < HEADER {
            return None;
        }
        let source = Source::of(k[1])?;
        let n = k[9];
        if !(n >= 0.0 && n <= 64.0) || k.len() != HEADER + STEP * n as usize {
            return None;
        }
        let steps = read_steps(&k[HEADER..])?;
        Some(Layout {
            lead: k[0] as usize,
            source,
            axis: k[2] as u8,
            number: (k[3], k[4], k[5] != 0.0),
            reference: [k[6], k[7], k[8]],
            steps,
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
fn vscale<F: Copy + Mul<Output = F>>(a: [F; 3], k: F) -> [F; 3] {
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

/// The motion's pose at roll `t` (the place's third), as `motion::Family::at` reads it; where it
/// has no step (a formed flank, #283), the source standing still.
fn pose<T: Num>(l: &Layout, m: &[T], t: H2<T>) -> Pose<T> {
    poses(&l.steps, m, t).pop().unwrap_or_else(|| {
        let (one, o) = (H2::cst(T::cst(1.0)), H2::cst(zero()));
        Pose { r: [[one, o, o], [o, one, o], [o, o, one]], p: [o; 3] }
    })
}

/// A rigid pose's rotation and translation, as numbers.
pub(crate) type Frame<T> = ([[T; 3]; 3], [T; 3]);

/// Every step's pose at roll `t`, as numbers: what a meshing reads of its motion at two rolls
/// (#283, `envelope::meshing`).
pub(crate) fn frames<T: Num>(steps: &[Step], m: &[T], t: T) -> Vec<Frame<T>> {
    poses(steps, m, H2::cst(t)).into_iter()
        .map(|p| (p.r.map(|r| r.map(|x| x.v)), p.p.map(|x| x.v))).collect()
}

/// Each step's pose at roll `t`, in order.
fn poses<T: Num>(steps: &[Step], m: &[T], t: H2<T>) -> Vec<Pose<T>> {
    let tau = std::f64::consts::TAU;
    let mut poses: Vec<Pose<T>> = Vec::with_capacity(steps.len());
    for s in steps {
        let next = match *s {
            Step::Rotation { .. } | Step::Turn { .. } => {
                let (a, d, ratio, phase, advance) = match *s {
                    Step::Rotation { at, ratio, phase, advance } => {
                        let a = col3(m, at);
                        (a, unit(vsub(col3(m, at + 3), a)), ratio, phase, advance)
                    }
                    Step::Turn { at, ratio, phase } => {
                        (col3(m, at), unit(cross(col3(m, at + 3), col3(m, at + 6))), ratio, phase, 0.0)
                    }
                    _ => unreachable!(),
                };
                let theta = H2::cst(T::cst(phase)) + t.scale(T::cst(ratio));
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
                let screw = t.scale(T::cst(advance / tau));
                let p = vadd(vsub(al, ra), vscale(lift(d), screw));
                Pose { r, p }
            }
            Step::Translation { at, advance } => {
                let a = col3(m, at);
                let d = unit(vsub(col3(m, at + 3), a));
                let one = H2::cst(T::cst(1.0));
                let o = H2::cst(zero());
                let r = [[one, o, o], [o, one, o], [o, o, one]];
                Pose { r, p: vscale(lift(d), t.scale(T::cst(advance / tau))) }
            }
            Step::Relative { source, observer } => Pose::relative(&poses[source], &poses[observer]),
        };
        poses.push(next);
    }
    poses
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

/// The normal curvature of the envelope along `w` (a direction in its tangent plane), with
/// respect to `f.n`: its centre of curvature is `x + n/κ`.  Read over two directions of the place
/// square to `∇g` — so along the envelope — chosen by `∇g`'s largest component.
fn curvature<T: Num>(f: &Local<T>, w: [T; 3]) -> T {
    let g = f.dg;
    let big = (0..3).max_by(|&i, &j| g[i].val().abs().total_cmp(&g[j].val().abs())).unwrap_or(2);
    let z = zero::<T>();
    let neg = |x: T| z - x;
    let (k1, k2) = match big {
        0 => ([neg(g[1]), g[0], z], [neg(g[2]), z, g[0]]),
        1 => ([g[1], neg(g[0]), z], [z, neg(g[2]), g[1]]),
        _ => ([g[2], z, neg(g[0])], [z, g[2], neg(g[1])]),
    };
    let along = |d: &[[T; 3]; 3], k: [T; 3]| vadd(vadd(vscale(d[0], k[0]), vscale(d[1], k[1])), vscale(d[2], k[2]));
    let (t1, t2) = (along(&f.dx, k1), along(&f.dx, k2));
    let (n1, n2) = (along(&f.dn, k1), along(&f.dn, k2));
    let (e, ff, gg) = (dot(t1, t1), dot(t1, t2), dot(t2, t2));
    let half = T::cst(0.5);
    let (ll, mm, nn) = (neg(dot(n1, t1)), neg((dot(n1, t2) + dot(n2, t1)) * half), neg(dot(n2, t2)));
    let (b1, b2) = (dot(t1, w), dot(t2, w));
    let det = e * gg - ff * ff;
    let det = if det.val() != 0.0 { det } else { T::cst(1.0) };
    let (a, b) = ((gg * b1 - ff * b2) / det, (e * b2 - ff * b1) / det);
    let two = T::cst(2.0);
    let second = ll * a * a + two * mm * a * b + nn * b * b;
    let first = e * a * a + two * ff * a * b + gg * b * b;
    let first = if first.val() != 0.0 { first } else { T::cst(1.0) };
    second / first
}

/// The smallest curvature a contact divides by: a section flatter than this is read as this.
const FLAT: f64 = 1e-12;

/// A contact's rows over its columns `cols` and constants `k`, into `out`.  `None` where the
/// constants do not describe a contact over this many columns.
pub(crate) fn rows<T: Num>(c: Contact, cols: &[T], k: &[f64], out: &mut [T]) -> Option<()> {
    let l = Layout::read(k)?;
    if l.lead != c.lead() || cols.len() != l.width() || out.len() != c.rows() {
        return None;
    }
    let f = local(&l, cols, c == Contact::Curvature);
    match c {
        Contact::Point => point_rows(cols, &f, out),
        Contact::Tangent => {
            let d = unit(vsub(col3(cols, 3), col3(cols, 0)));
            out[0] = dot(d, f.n);
        }
        Contact::Normal | Contact::Plane => {
            let (a, b) = (col3(cols, 0), col3(cols, 3));
            let d = unit(if c == Contact::Plane { cross(a, b) } else { vsub(b, a) });
            let t1 = unit(f.dx[0]);
            let t2 = cross(f.n, t1);
            out[0] = dot(d, t1);
            out[1] = dot(d, t2);
        }
        // Meusnier: the section by a plane of normal `m` through the point is a curve turning
        // `κₙ / cos θ` toward the normal's part in the plane, `n − (n·m)m` (of length `cos θ`),
        // `κₙ` the normal curvature along the section's tangent `m × n` — so its centre of
        // curvature is `x + (n − (n·m)m)/κₙ`, and that is the circle's centre, in the plane's two
        // directions, the radius its distance from the point
        Contact::Curvature => {
            let (centre, r) = (col3(cols, 0), cols[3]);
            let (pu, pv) = (col3(cols, 4), col3(cols, 7));
            let m = unit(cross(pu, pv));
            let kappa = curvature(&f, unit(cross(m, f.n)));
            let kappa = if kappa.val().abs() >= FLAT { kappa }
                else { T::cst(if kappa.val() < 0.0 { -FLAT } else { FLAT }) };
            let inward = vsub(f.n, vscale(m, dot(f.n, m)));
            let to = vsub(centre, f.x);
            let off = vsub(to, vscale(inward, T::cst(1.0) / kappa));
            let a = unit(pu);
            let b = unit(cross(m, a));
            out[0] = dot(off, a);
            out[1] = dot(off, b);
            out[2] = dot(to, to).sqrt() - r;
        }
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
    let (res, jac): (fn(usize, &[f64], &[f64], &mut [f64]), fn(usize, &[f64], &[f64], &mut [f64])) = match c {
        Contact::Point => (res::<0>, jac::<0>),
        Contact::Tangent => (res::<1>, jac::<1>),
        Contact::Normal => (res::<2>, jac::<2>),
        Contact::Curvature => (res::<3>, jac::<3>),
        Contact::Plane => (res::<4>, jac::<4>),
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
    cols: Vec<Col>,
}

/// One part of the columns.
#[derive(Clone, Copy, Debug)]
enum Col {
    /// A point's lift: three columns.
    Lift(usize),
    /// One number.
    One(u32),
    /// Three numbers: an axis's place or direction.
    Three([u32; 3]),
}

impl Parts {
    /// Envelope `index`'s parts, or why a contact cannot read it.
    pub fn of(sk: &Sketch, index: usize) -> Result<Parts, String> {
        let e = sk.envelopes.get(index).ok_or("no such envelope")?;
        let mut parts = Parts::still(sk, &e.held, &e.name)?;
        let mut done = std::collections::BTreeMap::new();
        motion(sk, e.motion as usize, &mut parts.steps, &mut parts.cols, &mut done, 0, false)?;
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
        let number = |t: &TermNumber| -> Result<((f64, f64, bool), Option<u32>), String> {
            match &t.free {
                None => Ok(((t.c, 0.0, false), None)),
                Some((name, m)) => match sk.free_vars.get(name) {
                    Some(&p) => Ok(((t.c, *m, true), Some(p))),
                    None => Err(format!("`{name}` is an unknown nothing on the drawing reads: state \
                                         it, or a relation that holds it")),
                },
            }
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
                        let pl = &sk.planes[plane];
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
                let (n, free) = number(&shape.number)?;
                cols.extend(free.map(Col::One));
                (source, 0, n, shape.reference)
            }
        };
        Ok(Parts { source, axis, number, reference, steps: Vec::new(), cols })
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
        consts(lead, self.source, self.axis, self.number, self.reference, &self.steps)
    }

    /// How many constants a contact reads (`consts`).
    pub fn n_const(&self) -> usize {
        HEADER + STEP * self.steps.len()
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
    cols: Vec<Col>,
}

impl Moving {
    /// Motion `m`'s steps and columns, or why the solve cannot read it.
    pub fn of(sk: &Sketch, m: usize) -> Result<Moving, String> {
        Self::read(sk, m, false)
    }

    /// The same, a turn about a point of a view read as the turn about the line through it square
    /// to the view (`Step::Turn`): what a relation reading the motion itself takes (#326), where
    /// an envelope in space is still generated under turns about lines.
    pub fn of_any(sk: &Sketch, m: usize) -> Result<Moving, String> {
        Self::read(sk, m, true)
    }

    fn read(sk: &Sketch, m: usize, turns: bool) -> Result<Moving, String> {
        let (mut steps, mut cols) = (Vec::new(), Vec::new());
        motion(sk, m, &mut steps, &mut cols, &mut std::collections::BTreeMap::new(), 0, turns)?;
        Ok(Moving { steps, cols })
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

/// The points whose lifts columns read.
fn lifted(cols: &[Col]) -> Vec<usize> {
    cols.iter().filter_map(|c| match *c {
        Col::Lift(p) => Some(p),
        _ => None,
    }).collect()
}

/// The columns' parameters, every point's lift minted.
fn columns(sk: &Sketch, cols: &[Col]) -> Vec<u32> {
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
fn values(sk: &Sketch, cols: &[Col]) -> Vec<f64> {
    let v = |p: u32| sk.params[p as usize].value;
    cols.iter().flat_map(|c| match *c {
        Col::Lift(p) => sk.lifted(p).to_vec(),
        Col::One(p) => vec![v(p)],
        Col::Three(x) => x.map(v).to_vec(),
    }).collect()
}

fn width(cols: &[Col]) -> usize {
    cols.iter().map(|x| match x {
        Col::Lift(_) | Col::Three(_) => 3,
        Col::One(_) => 1,
    }).sum()
}

/// Motion `m` as steps in the order `motion::Family::read` takes them — each once, its source
/// and observer before a relative — each axis's ends appended to `cols`, six motion columns past
/// the last; returns its step.
fn motion(sk: &Sketch, m: usize, steps: &mut Vec<Step>, cols: &mut Vec<Col>,
          done: &mut std::collections::BTreeMap<usize, usize>, depth: usize, turns: bool)
          -> Result<usize, String> {
    if let Some(&s) = done.get(&m) {
        return Ok(s);
    }
    if depth > 64 {
        return Err("a motion nests too deeply".into());
    }
    let me = sk.motions.get(m).ok_or("no such motion")?;
    if !me.measured.is_empty() {
        return Err(format!("`{}`'s numbers are measured after the solve, and an envelope the solve \
                            holds is generated within it: state them", me.name));
    }
    // where the next axis's columns start in the motion's block: past every axis already read
    let at = steps_width(steps);
    let mut axis = |l: u32| -> Result<(), String> {
        let line = sk.lines.get(l as usize).ok_or("no such motion axis")?;
        cols.extend([Col::Lift(line.p1 as usize), Col::Lift(line.p2 as usize)]);
        Ok(())
    };
    let step = match me.def {
        MotionDef::Rotation { axis: l, .. } => {
            let (ratio, phase, advance) = me.rotation(sk)?;
            axis(l)?;
            Step::Rotation { at, ratio, phase, advance }
        }
        MotionDef::Translation { axis: l, .. } => {
            let advance = me.advance(sk)?;
            axis(l)?;
            Step::Translation { at, advance }
        }
        MotionDef::Turn { centre, .. } if turns => {
            let (ratio, phase, _) = me.rotation(sk)?;
            let view = sk.plane_of(centre as usize).ok_or_else(|| {
                format!("`{}` turns about a point drawn in no view, so about no line in space", me.name)
            })?;
            let pl = sk.planes.get(view).ok_or("no such view")?;
            cols.extend([Col::Lift(centre as usize), Col::Three(sk.axes[pl.u as usize].d),
                         Col::Three(sk.axes[pl.v as usize].d)]);
            Step::Turn { at, ratio, phase }
        }
        MotionDef::Turn { .. } => {
            return Err(format!("`{}` turns about a point of a view: an envelope in space is \
                                generated under turns about lines", me.name))
        }
        MotionDef::Relative { source, observer } => {
            let source = motion(sk, source as usize, steps, cols, done, depth + 1, turns)?;
            let observer = motion(sk, observer as usize, steps, cols, done, depth + 1, turns)?;
            Step::Relative { source, observer }
        }
    };
    steps.push(step);
    done.insert(m, steps.len() - 1);
    Ok(steps.len() - 1)
}
