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
//! series in the place (`H2`): the residual over `f64`, and the Jacobian over tangents (`Tan`),
//! eight columns a pass — exact, no differences.
//!
//! **Layout.** A contact's columns are `[lead…, u, v, t, source…, motion…]`: what the contact
//! states about (a point's lift, a line's ends, a plane's axes, a circle), the place, the source
//! (`Source`), and two lifted ends per axis the motion turns about or slides along.  Its constants
//! are `[lead, mode, source, axis, c, m, free, ref×3, steps, (code, at, source, observer, ratio,
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
use crate::kernels::{Kernel, Num};
use std::ops::{Add, Div, Mul, Sub};

/* -- the layout ------------------------------------------------------------------------------ */

/// Constants before the motion's steps.
pub const HEADER: usize = 11;
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
}

impl Source {
    pub fn of(code: f64) -> Option<Source> {
        Some(match code as i64 {
            1 => Source::Line,
            2 => Source::Cone,
            3 => Source::Cylinder,
            4 => Source::Round,
            _ => return None,
        })
    }

    /// Its own columns, before a free number's.
    pub fn width(self) -> usize {
        match self {
            Source::Line => 12,
            Source::Cone | Source::Cylinder => 6,
            Source::Round => 16,
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
    /// `l perpendicular(at: p) flank` (lead mode 0, two points along it) and `P tangent(at: p)
    /// flank` (mode 1, the plane's two axes): the direction along the normal.
    Normal = 2,
    /// `flank curvature(at: p) k`: the circle osculates the envelope's section by its plane at
    /// `p`, which is in that plane (drawn there, or stated).  Lead: the centre's lift, the radius,
    /// the plane's two axes.
    Curvature = 3,
}

impl Contact {
    pub const ALL: [Contact; 4] = [Contact::Point, Contact::Tangent, Contact::Normal, Contact::Curvature];

    pub fn rows(self) -> usize {
        match self {
            Contact::Point => 4,
            Contact::Tangent => 1,
            Contact::Normal => 2,
            Contact::Curvature => 3,
        }
    }

    /// The lead's width (the columns before the place).
    pub fn lead(self) -> usize {
        match self {
            Contact::Point => 3,
            Contact::Tangent | Contact::Normal => 6,
            Contact::Curvature => 10,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Contact::Point => "point_on_envelope",
            Contact::Tangent => "envelope_tangent_at",
            Contact::Normal => "envelope_normal_at",
            Contact::Curvature => "envelope_curvature_at",
        }
    }

    /// A point's and a circle's rows are lengths; a direction's are cosines.
    fn degree(self) -> u32 {
        match self {
            Contact::Point | Contact::Curvature => 1,
            Contact::Tangent | Contact::Normal => 0,
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
    /// `observer⁻¹ ∘ source`, by step index.
    Relative { source: usize, observer: usize },
}

/// The constants of one contact, built from its parts (the inverse of `Layout::read`).
#[allow(clippy::too_many_arguments)]
pub fn consts(lead: usize, mode: f64, source: Source, axis: u8, number: (f64, f64, bool), reference: [f64; 3],
              steps: &[Step]) -> Vec<f64> {
    let mut k = vec![lead as f64, mode, source as i64 as f64, axis as f64,
                     number.0, number.1, if number.2 { 1.0 } else { 0.0 },
                     reference[0], reference[1], reference[2], steps.len() as f64];
    for s in steps {
        k.extend(match *s {
            Step::Rotation { at, ratio, phase, advance } => [1.0, at as f64, 0.0, 0.0, ratio, phase, advance],
            Step::Translation { at, advance } => [2.0, at as f64, 0.0, 0.0, 0.0, 0.0, advance],
            Step::Relative { source, observer } => [3.0, 0.0, source as f64, observer as f64, 0.0, 0.0, 0.0],
        });
    }
    k
}

/// A contact's constants read back.
#[derive(Clone, Debug)]
pub struct Layout {
    pub lead: usize,
    pub mode: u8,
    pub source: Source,
    /// The source's axis as two points (0) or a point and a direction (1).
    pub axis: u8,
    pub number: (f64, f64, bool),
    pub reference: [f64; 3],
    pub steps: Vec<Step>,
}

impl Layout {
    pub fn read(k: &[f64]) -> Option<Layout> {
        if k.len() < HEADER {
            return None;
        }
        let source = Source::of(k[2])?;
        let n = k[10];
        if !(n >= 1.0 && n <= 64.0) || k.len() != HEADER + STEP * n as usize {
            return None;
        }
        let mut steps = Vec::with_capacity(n as usize);
        for (i, s) in k[HEADER..].chunks(STEP).enumerate() {
            let idx = |x: f64| (x >= 0.0 && (x as usize) < i).then_some(x as usize);
            steps.push(match s[0] as i64 {
                1 => Step::Rotation { at: s[1] as usize, ratio: s[4], phase: s[5], advance: s[6] },
                2 => Step::Translation { at: s[1] as usize, advance: s[6] },
                3 => Step::Relative { source: idx(s[2])?, observer: idx(s[3])? },
                _ => return None,
            });
        }
        Some(Layout {
            lead: k[0] as usize,
            mode: k[1] as u8,
            source,
            axis: k[3] as u8,
            number: (k[4], k[5], k[6] != 0.0),
            reference: [k[7], k[8], k[9]],
            steps,
        })
    }

    /// The source's columns, a free number's included.
    pub fn source_width(&self) -> usize {
        self.source.width() + self.number.2 as usize
    }

    /// The motion's columns: six per axis a step reads.
    pub fn motion_width(&self) -> usize {
        self.steps.iter().filter_map(|s| match *s {
            Step::Rotation { at, .. } | Step::Translation { at, .. } => Some(at + 6),
            Step::Relative { .. } => None,
        }).max().unwrap_or(0)
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

/// A number and its rate along `W` columns at once: forward differentiation, exact.
#[derive(Clone, Copy, Debug)]
pub struct Tan<const W: usize> {
    pub v: f64,
    pub g: [f64; W],
}

impl<const W: usize> Tan<W> {
    fn map(self, v: f64, d: f64) -> Self {
        Tan { v, g: self.g.map(|x| x * d) }
    }
}

impl<const W: usize> Add for Tan<W> {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Tan { v: self.v + o.v, g: std::array::from_fn(|i| self.g[i] + o.g[i]) }
    }
}
impl<const W: usize> Sub for Tan<W> {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Tan { v: self.v - o.v, g: std::array::from_fn(|i| self.g[i] - o.g[i]) }
    }
}
impl<const W: usize> Mul for Tan<W> {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        Tan { v: self.v * o.v, g: std::array::from_fn(|i| self.v * o.g[i] + self.g[i] * o.v) }
    }
}
impl<const W: usize> Div for Tan<W> {
    type Output = Self;
    fn div(self, o: Self) -> Self {
        let q = self.v / o.v;
        Tan { v: q, g: std::array::from_fn(|i| (self.g[i] - q * o.g[i]) / o.v) }
    }
}
impl<const W: usize> Num for Tan<W> {
    fn cst(v: f64) -> Self {
        Tan { v, g: [0.0; W] }
    }
    fn val(self) -> f64 {
        self.v
    }
    fn sqrt(self) -> Self {
        let s = self.v.max(0.0).sqrt();
        // no rate where the root is zero: a degenerate figure, which a row reads as itself
        self.map(s, if s > 0.0 { 0.5 / s } else { 0.0 })
    }
    fn sin_cos(self) -> (Self, Self) {
        let (s, c) = self.v.dsin_cos();
        (self.map(s, c), self.map(c, -s))
    }
    fn atan2(y: Self, x: Self) -> Self {
        let q = x.v * x.v + y.v * y.v;
        let q = if q > 0.0 { q } else { 1.0 };
        Tan { v: y.v.datan2(x.v), g: std::array::from_fn(|i| (x.v * y.g[i] - y.v * x.g[i]) / q) }
    }
}

fn zero<T: Num>() -> T {
    T::cst(0.0)
}

/// A number and its first two derivatives in the place `[u, v, t]`: `f(x + δ) ≈ v + g·δ +
/// δ·h·δ/2`.
#[derive(Clone, Copy, Debug)]
struct H2<T> {
    v: T,
    g: [T; 3],
    h: [[T; 3]; 3],
}

impl<T: Num> H2<T> {
    fn cst(v: T) -> Self {
        H2 { v, g: [zero(); 3], h: [[zero(); 3]; 3] }
    }
    fn var(v: T, i: usize) -> Self {
        let mut x = Self::cst(v);
        x.g[i] = T::cst(1.0);
        x
    }
    /// `f(self)`, where `f`, `f'` and `f''` at the value are given.
    fn chain(self, f0: T, f1: T, f2: T) -> Self {
        let mut r = Self::cst(f0);
        for i in 0..3 {
            r.g[i] = f1 * self.g[i];
            for j in i..3 {
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
        H2 { v: self.v * k, g: self.g.map(|x| x * k), h: self.h.map(|r| r.map(|x| x * k)) }
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
            h: std::array::from_fn(|i| std::array::from_fn(|j| self.h[i][j] + o.h[i][j])),
        }
    }
}
impl<T: Num> Sub for H2<T> {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        H2 {
            v: self.v - o.v,
            g: std::array::from_fn(|i| self.g[i] - o.g[i]),
            h: std::array::from_fn(|i| std::array::from_fn(|j| self.h[i][j] - o.h[i][j])),
        }
    }
}
impl<T: Num> Mul for H2<T> {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        let mut r = H2::cst(self.v * o.v);
        for i in 0..3 {
            r.g[i] = self.v * o.g[i] + self.g[i] * o.v;
            for j in i..3 {
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

fn dot<F: Copy + Add<Output = F> + Mul<Output = F>>(a: [F; 3], b: [F; 3]) -> F {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross<F: Copy + Sub<Output = F> + Mul<Output = F>>(a: [F; 3], b: [F; 3]) -> [F; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn vadd<F: Copy + Add<Output = F>>(a: [F; 3], b: [F; 3]) -> [F; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn vsub<F: Copy + Sub<Output = F>>(a: [F; 3], b: [F; 3]) -> [F; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn vscale<F: Copy + Mul<Output = F>>(a: [F; 3], k: F) -> [F; 3] {
    [a[0] * k, a[1] * k, a[2] * k]
}

/// A unit vector along `x`; `x` itself where it has no length (a degenerate figure, whose rows
/// read as whatever it is rather than as NaN).
fn unit<T: Num>(x: [T; 3]) -> [T; 3] {
    let n = dot(x, x).sqrt();
    if n.val() > 0.0 {
        vscale(x, T::cst(1.0) / n)
    } else {
        x
    }
}

fn col3<T: Num>(c: &[T], at: usize) -> [T; 3] {
    [c[at], c[at + 1], c[at + 2]]
}
fn lift<T: Num>(x: [T; 3]) -> [H2<T>; 3] {
    x.map(H2::cst)
}

/* -- the source, the motion, the moving point ------------------------------------------------ */

/// The source's point at place `(u, v)`: its meridian element at `u`, turned `v` about its axis.
fn surface<T: Num>(l: &Layout, s: &[T], u: H2<T>, v: H2<T>) -> [H2<T>; 3] {
    let a = col3(s, 0);
    let b = if l.axis == 1 { vadd(a, col3(s, 3)) } else { col3(s, 3) };
    let d = unit(vsub(b, a));
    let r = l.reference.map(T::cst);
    let e1 = unit(vsub(r, vscale(d, dot(r, d))));
    let number = || {
        let (c, m, free) = l.number;
        let x = if free { s[l.source.width()] } else { zero() };
        T::cst(c) + T::cst(m) * x
    };
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

/// The motion's pose at roll `t` (the place's third), as `motion::Family::at` reads it.
fn pose<T: Num>(l: &Layout, m: &[T], t: H2<T>) -> Pose<T> {
    let tau = std::f64::consts::TAU;
    let mut poses: Vec<Pose<T>> = Vec::with_capacity(l.steps.len());
    for s in &l.steps {
        let next = match *s {
            Step::Rotation { at, ratio, phase, advance } => {
                let a = col3(m, at);
                let d = unit(vsub(col3(m, at + 3), a));
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
    *poses.last().expect("a motion has a step")
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

/// The local frame at the place in columns `lead..lead + 3`.
pub(crate) fn local<T: Num>(l: &Layout, cols: &[T]) -> Local<T> {
    let at = l.lead;
    let (u, v, t) = (H2::var(cols[at], 0), H2::var(cols[at + 1], 1), H2::var(cols[at + 2], 2));
    let s = &cols[at + 3..at + 3 + l.source_width()];
    let m = &cols[at + 3 + l.source_width()..];
    let x = pose(l, m, t).apply(surface(l, s, u, v));
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
    let f = local(&l, cols);
    match c {
        Contact::Point => {
            let p = col3(cols, 0);
            for i in 0..3 {
                out[i] = p[i] - f.x[i];
            }
            out[3] = f.g;
        }
        Contact::Tangent => {
            let d = unit(vsub(col3(cols, 3), col3(cols, 0)));
            out[0] = dot(d, f.n);
        }
        Contact::Normal => {
            let (a, b) = (col3(cols, 0), col3(cols, 3));
            let d = unit(if l.mode == 1 { cross(a, b) } else { vsub(b, a) });
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

/* -- the kernels ------------------------------------------------------------------------------ */

/// Columns differentiated per pass.
const PASS: usize = 8;

fn widths(n: usize, v: &[f64], k: &[f64]) -> (usize, usize) {
    if n == 0 { (0, 0) } else { (v.len() / n, k.len() / n) }
}

fn res<const C: u8>(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let c = Contact::ALL[C as usize];
    let (n_par, n_const) = widths(n, v, k);
    let m = c.rows();
    for i in 0..n {
        let out = &mut r[m * i..m * (i + 1)];
        if rows::<f64>(c, &v[n_par * i..n_par * (i + 1)], &k[n_const * i..n_const * (i + 1)], out).is_none() {
            out.fill(f64::NAN);
        }
    }
}

fn jac<const C: u8>(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let c = Contact::ALL[C as usize];
    let (n_par, n_const) = widths(n, v, k);
    let m = c.rows();
    let mut cols = vec![Tan::<PASS>::cst(0.0); n_par];
    let mut out = vec![Tan::<PASS>::cst(0.0); m];
    for i in 0..n {
        let x = &v[n_par * i..n_par * (i + 1)];
        let kc = &k[n_const * i..n_const * (i + 1)];
        let block = &mut j[m * n_par * i..m * n_par * (i + 1)];
        for start in (0..n_par).step_by(PASS) {
            for (q, col) in cols.iter_mut().enumerate() {
                *col = Tan::cst(x[q]);
                if (start..start + PASS).contains(&q) {
                    col.g[q - start] = 1.0;
                }
            }
            if rows(c, &cols, kc, &mut out).is_none() {
                block.fill(f64::NAN);
                break;
            }
            for (row, val) in out.iter().enumerate() {
                for q in start..(start + PASS).min(n_par) {
                    block[row * n_par + q] = val.g[q - start];
                }
            }
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
    /// `∂X/∂u`, `∂X/∂v`.
    pub du: [f64; 3],
    pub dv: [f64; 3],
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
    let f = local(&l, &cols);
    let all = f.x.iter().chain(f.n.iter()).chain(f.dx.iter().flatten()).all(|x| x.is_finite());
    all.then_some(Reading {
        position: f.x,
        normal: f.n,
        velocity: f.dx[2],
        normal_velocity: f.g,
        du: f.dx[0],
        dv: f.dx[1],
    })
}

/// The normal curvature along `w` at `place` (`curvature`'s, over numbers), with respect to the
/// reading's normal: the centre of curvature is `position + normal / κ`.
pub fn normal_curvature(k: &[f64], parts: &[f64], place: [f64; 3], w: [f64; 3]) -> Option<f64> {
    let l = Layout::read(k)?;
    let mut cols = vec![0.0; l.lead];
    cols.extend(place);
    cols.extend_from_slice(parts);
    let f = local(&l, &cols);
    let kappa = curvature(&f, w);
    kappa.is_finite().then_some(kappa)
}

/// The place on the envelope nearest `p` that a solve should start from: over the roll interval,
/// the source's nearest point to `p` carried back by the pose, the roll where `n · ∂X/∂t` comes
/// nearest zero there, then Gauss–Newton on `p − X` and the meshing equation together.  `None`
/// where nothing is finite.  A seed, not a projection: no claim it is the nearest root.
pub fn seed(k: &[f64], parts: &[f64], p: [f64; 3], roll: [f64; 2]) -> Option<[f64; 3]> {
    let l = Layout::read(k)?;
    if l.lead != Contact::Point.lead() || parts.len() != l.source_width() + l.motion_width() {
        return None;
    }
    let mut pc = vec![0.0; l.lead + 3 + parts.len()];
    pc[..3].copy_from_slice(&p);
    pc[l.lead + 3..].copy_from_slice(parts);
    let residual = |place: [f64; 3], pc: &mut Vec<f64>| -> Option<[f64; 4]> {
        pc[3..6].copy_from_slice(&place);
        let mut out = [0.0; 4];
        rows::<f64>(Contact::Point, pc, k, &mut out)?;
        out.iter().all(|x| x.is_finite()).then_some(out)
    };
    let cost = |r: [f64; 4]| r.iter().map(|x| x * x).sum::<f64>();
    // a coarse search over the place: rolls across the interval, turns round the axis, and the
    // meridian's parameter at a few spans
    let scale = parts.iter().take(6).map(|x| x.abs()).fold(1.0f64, f64::max);
    let us: Vec<f64> = match l.source {
        Source::Line => vec![-0.5, 0.0, 0.25, 0.5, 0.75, 1.0, 1.5],
        Source::Round => (0..12).map(|i| i as f64 * std::f64::consts::TAU / 12.0).collect(),
        Source::Cone | Source::Cylinder => (-6..=6).map(|i| i as f64 * scale / 3.0).collect(),
    };
    let mut best: Option<([f64; 3], f64)> = None;
    let n_roll = 25;
    for a in 0..n_roll {
        let t = roll[0] + (roll[1] - roll[0]) * a as f64 / (n_roll - 1) as f64;
        for b in 0..24 {
            let v = b as f64 * std::f64::consts::TAU / 24.0;
            for &u in &us {
                if let Some(r) = residual([u, v, t], &mut pc) {
                    let c = cost(r);
                    if best.is_none_or(|(_, b)| c < b) {
                        best = Some(([u, v, t], c));
                    }
                }
            }
        }
    }
    let (mut x, mut c) = best?;
    // Levenberg–Marquardt on the four rows over the three numbers, differenced: a seed needs no
    // more than a downhill walk
    let mut lambda = 1e-3;
    for _ in 0..100 {
        let r = residual(x, &mut pc)?;
        let mut jm = [[0.0; 3]; 4];
        for q in 0..3 {
            let h = 1e-7 * (1.0 + x[q].abs());
            let mut y = x;
            y[q] += h;
            let ry = residual(y, &mut pc)?;
            for row in 0..4 {
                jm[row][q] = (ry[row] - r[row]) / h;
            }
        }
        let mut a = [[0.0; 3]; 3];
        let mut g = [0.0; 3];
        for i in 0..3 {
            for j in 0..3 {
                a[i][j] = (0..4).map(|row| jm[row][i] * jm[row][j]).sum();
            }
            g[i] = (0..4).map(|row| jm[row][i] * r[row]).sum();
        }
        let mut stepped = false;
        for _ in 0..10 {
            let mut aa = a;
            for i in 0..3 {
                aa[i][i] += lambda * (1.0 + a[i][i]);
            }
            let Some(d) = solve3(aa, g) else { lambda *= 10.0; continue };
            let y = [x[0] - d[0], x[1] - d[1], x[2] - d[2]];
            if let Some(ry) = residual(y, &mut pc) {
                let cy = cost(ry);
                if cy < c {
                    x = y;
                    c = cy;
                    lambda = (lambda * 0.3).max(1e-12);
                    stepped = true;
                    break;
                }
            }
            lambda *= 10.0;
        }
        if !stepped || c < 1e-24 {
            break;
        }
    }
    Some(x)
}

fn solve3(a: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let det = |m: [[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let d = det(a);
    if d == 0.0 || !d.is_finite() {
        return None;
    }
    let mut x = [0.0; 3];
    for c in 0..3 {
        let mut m = a;
        for r in 0..3 {
            m[r][c] = b[r];
        }
        x[c] = det(m) / d;
    }
    Some(x)
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
        let (source, axis, number, reference) = match &e.held {
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
                return Err(format!("`{}`'s set states nothing at its point a surface can be read \
                                    from", e.name))
            }
            Held::Set { shape: Some(shape), .. } => {
                let source = match shape.kind {
                    SetKind::Cone => Source::Cone,
                    SetKind::Cylinder => Source::Cylinder,
                };
                cols.extend(ends(shape.axis)?.map(Col::Lift));
                let (n, free) = number(&shape.number)?;
                cols.extend(free.map(Col::One));
                (source, 0, n, shape.reference)
            }
        };
        let mut steps = Vec::new();
        let mut done = std::collections::BTreeMap::new();
        motion(sk, e.motion as usize, &mut steps, &mut cols, &mut done, 0)?;
        Ok(Parts { source, axis, number, reference, steps, cols })
    }

    /// The points whose lifts the columns read: what `Sketch::add` lifts.
    pub fn lifted(&self) -> Vec<usize> {
        self.cols.iter().filter_map(|c| match *c {
            Col::Lift(p) => Some(p),
            _ => None,
        }).collect()
    }

    /// The columns, every point's lift minted.
    pub fn columns(&self, sk: &Sketch) -> Vec<u32> {
        self.cols.iter().flat_map(|c| match *c {
            Col::Lift(p) => {
                let k = sk.lift_of(p).expect("an envelope's points are lifted at the add");
                sk.lifts[k].x.to_vec()
            }
            Col::One(p) => vec![p],
            Col::Three(x) => x.to_vec(),
        }).collect()
    }

    /// The columns' values as the drawing stands, a point not lifted yet read where it is.
    pub fn values(&self, sk: &Sketch) -> Vec<f64> {
        let v = |p: u32| sk.params[p as usize].value;
        self.cols.iter().flat_map(|c| match *c {
            Col::Lift(p) => sk.lifted(p).to_vec(),
            Col::One(p) => vec![v(p)],
            Col::Three(x) => x.map(v).to_vec(),
        }).collect()
    }

    /// A contact's constants over these parts.
    pub fn consts(&self, c: Contact, mode: u8) -> Vec<f64> {
        consts(c.lead(), mode as f64, self.source, self.axis, self.number, self.reference, &self.steps)
    }

    /// How many columns a contact of kind `c` reads.
    pub fn width(&self, c: Contact) -> usize {
        c.lead() + 3 + self.cols.iter().map(|x| match x {
            Col::Lift(_) | Col::Three(_) => 3,
            Col::One(_) => 1,
        }).sum::<usize>()
    }
}

/// Motion `m` as steps in the order `motion::Family::read` takes them — each once, its source
/// and observer before a relative — each axis's ends appended to `cols`, six motion columns past
/// the last; returns its step.
fn motion(sk: &Sketch, m: usize, steps: &mut Vec<Step>, cols: &mut Vec<Col>,
          done: &mut std::collections::BTreeMap<usize, usize>, depth: usize) -> Result<usize, String> {
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
    let at = 6 * steps.iter().filter(|s| !matches!(s, Step::Relative { .. })).count();
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
        MotionDef::Turn { .. } => {
            return Err(format!("`{}` turns about a point of a view: an envelope in space is \
                                generated under turns about lines", me.name))
        }
        MotionDef::Relative { source, observer } => {
            let source = motion(sk, source as usize, steps, cols, done, depth + 1)?;
            let observer = motion(sk, observer as usize, steps, cols, done, depth + 1)?;
            Step::Relative { source, observer }
        }
    };
    steps.push(step);
    done.insert(m, steps.len() - 1);
    Ok(steps.len() - 1)
}
