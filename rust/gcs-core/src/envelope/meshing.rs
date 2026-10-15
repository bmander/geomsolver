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
//! **At a point held there** (#345) two more relations share the point's phase, lag and places,
//! as an envelope's `at:` contacts share their point's: `contact perpendicular(at: q) l`, the line
//! along the common normal (`Contact::Normal`), and `contact tangent(at: q) l`, the line along the
//! path of contact (`Contact::Path`): the contact point's derivative along the phase, read by the
//! implicit function theorem off the rows' Jacobian in the lag and the places.  (Not the bearing:
//! a pair whose contact stands still on the gear's flank — `spiral_bevel/contact.sv`'s, step 3's
//! turned cutter — has a path all the same, the gear's turn carrying the contact.)  Its own
//! Jacobian is the rows' second derivatives: the rows are read once over a number carrying their
//! derivatives in the eight unknowns (`Fwd`), each itself a number of the kernel's kind, so a
//! `Dual` pass differentiates the path exactly.  At a line contact that Jacobian is singular and
//! the path has no direction: the rows are not finite there.
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

use super::held::{self, col3, cross, dot, frames, local, unit, vadd, vsub, Layout, Step, PASS,
                   STEP};
use crate::kernels::{curve_widths, Dual, Kernel, Num};
use crate::linalg::Mat;
use std::iter::Chain;
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

/// Which statement a contact with a meshing makes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Contact {
    /// `p coincident contact`: the point on the pinion's flank, where the two touch. It owns the
    /// unknowns. Lead: the point's lift.
    Point = 0,
    /// `contact perpendicular(at: q) l`: the line runs along the common normal at `q` (#345).
    /// Lead: two points along it.
    Normal = 1,
    /// `contact tangent(at: q) l`: the line runs along the path of contact at `q` (#345). Lead:
    /// two points along it.
    Path = 2,
}

impl Contact {
    pub const ALL: [Contact; 3] = [Contact::Point, Contact::Normal, Contact::Path];

    pub fn rows(self) -> usize {
        match self {
            Contact::Point => LEAD + ROWS,
            Contact::Normal | Contact::Path => 2,
        }
    }

    /// The lead's width (the columns before the unknowns).
    pub fn lead(self) -> usize {
        match self {
            Contact::Point => LEAD,
            Contact::Normal | Contact::Path => 6,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Contact::Point => "point_on_meshing",
            Contact::Normal => "meshing_normal_at",
            Contact::Path => "meshing_tangent_at",
        }
    }

    /// A point's rows are lengths; a direction's are cosines.
    fn degree(self) -> u32 {
        match self {
            Contact::Point => 1,
            Contact::Normal | Contact::Path => 0,
        }
    }
}

/// A flank as a meshing's constants say it: an envelope or a source standing still (`held`), or
/// a surface traced in space as its member sees it (#326, `traced`: its domain and constants).
#[derive(Clone, Debug)]
enum Side {
    Held(Layout),
    Traced { domain: [[f64; 2]; 2], k: Vec<f64>, width: usize },
}

impl Side {
    fn read(k: &[f64]) -> Option<Side> {
        if let Some((domain, c)) = crate::traced::flank_parts(k) {
            return Some(Side::Traced { domain, k: c.to_vec(), width: crate::traced::flank_width(k) });
        }
        Layout::read_any(k).filter(|l| l.lead == 0).map(Side::Held)
    }

    /// Whether its own row is a meshing equation: a generated flank's, not a formed or a traced
    /// one's, whose roll is held at none.
    fn generated(&self) -> bool {
        matches!(self, Side::Held(l) if !l.steps.is_empty())
    }
}

/// A meshing's constants read back.
#[derive(Clone, Debug)]
struct Mesh {
    lead: usize,
    /// The pinion's flank (the observer's) and the gear's (the source's), and where each one's
    /// constants are.
    flanks: [Side; 2],
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
        let flank = |r: &Range<usize>| Side::read(&k[r.clone()]);
        let flanks = [flank(&consts[0])?, flank(&consts[1])?];
        let steps = held::read_steps(&k[HEADER + n1 + n2..])?;
        let Some(&Step::Relative { source, observer }) = steps.last() else { return None };
        Some(Mesh { lead, flanks, consts, steps, source, observer })
    }

    /// The columns of flank `i`'s source and motion.
    fn flank_width(&self, i: usize) -> usize {
        match &self.flanks[i] {
            Side::Held(l) => l.source_width() + l.motion_width(),
            Side::Traced { width, .. } => *width,
        }
    }

    /// Every column a contact reads.
    fn width(&self) -> usize {
        self.motion() + held::steps_width(&self.steps)
    }

    fn rows(&self) -> usize {
        self.lead + ROWS
    }

    /// Where the mesh's axes start among the columns.
    fn motion(&self) -> usize {
        self.lead + UNKNOWNS + self.flank_width(0) + self.flank_width(1)
    }

    /// The columns flank `i` reads: its place and its own columns.  No column is read by both
    /// flanks or by a flank and the poses (`pose_cols`); the rows read some besides, directly
    /// (`assemble`): the lead, the places' rolls (a formed flank's own row) and the mesh's first
    /// point (the lever).
    fn flank_cols(&self, i: usize) -> Chain<Range<usize>, Range<usize>> {
        let o = self.lead;
        let from = o + UNKNOWNS + (0..i).map(|f| self.flank_width(f)).sum::<usize>();
        (o + 2 + 3 * i..o + 5 + 3 * i).chain(from..from + self.flank_width(i))
    }

    /// The columns the poses read: the phase, the lag and the mesh's axes.
    fn pose_cols(&self) -> Chain<Range<usize>, Range<usize>> {
        (self.lead..self.lead + 2).chain(self.motion()..self.width())
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

/// What the rows read of a flank, in its own member: its point, its normal, `∂X/∂u` and its own
/// row (`held::Local`'s `x`, `n`, `dx[0]` and `g`).
struct Seen<T> {
    x: [T; 3],
    n: [T; 3],
    du: [T; 3],
    g: T,
}

/// A `Seen` as numbers, and the two poses (each a rotation, row-major, and a translation).
const SEEN: usize = 10;
const POSES: usize = 24;

impl<T: Num> Seen<T> {
    /// Flank `i` over its columns (`Mesh::flank_cols`).
    fn of(m: &Mesh, i: usize, c: &[T]) -> Self {
        match &m.flanks[i] {
            Side::Held(l) => {
                let f = local(l, c, false);
                Seen { x: f.x, n: f.n, du: f.dx[0], g: f.g }
            }
            // a traced surface as its member sees it: no meshing equation, its roll held
            Side::Traced { k, .. } => match crate::traced::seen(k, c) {
                Some((x, n, du)) => Seen { x, n, du, g: T::cst(0.0) },
                None => {
                    let nan = [T::cst(f64::NAN); 3];
                    Seen { x: nan, n: nan, du: nan, g: T::cst(f64::NAN) }
                }
            },
        }
    }
    fn flat(&self) -> [T; SEEN] {
        let [x, n, d] = [self.x, self.n, self.du];
        [x[0], x[1], x[2], n[0], n[1], n[2], d[0], d[1], d[2], self.g]
    }
    fn read(a: &[T; SEEN]) -> Self {
        Seen { x: col3(a, 0), n: col3(a, 3), du: col3(a, 6), g: a[9] }
    }
}

/// The two poses at the phase and the lag, in degrees as a curve's parameter over an angle is.
fn poses<T: Num>(m: &Mesh, phase: T, lag: T, motion: &[T]) -> [held::Frame<T>; 2] {
    let deg = T::cst(std::f64::consts::PI / 180.0);
    m.poses(motion, phase * deg, (phase + lag) * deg)
}

fn flat_poses<T: Num>(p: [held::Frame<T>; 2]) -> [T; POSES] {
    let [(r1, p1), (r2, p2)] = p;
    std::array::from_fn(|q| {
        let (r, t) = if q < 12 { (&r1, &p1) } else { (&r2, &p2) };
        let q = q % 12;
        if q < 9 { r[q / 3][q % 3] } else { t[q - 9] }
    })
}

fn read_poses<T: Num>(a: &[T; POSES]) -> [held::Frame<T>; 2] {
    let pose = |o: usize| -> held::Frame<T> {
        (std::array::from_fn(|i| col3(a, o + 3 * i)), col3(a, o + 9))
    };
    [pose(0), pose(12)]
}

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
    let seen = |i: usize| {
        let c: Vec<T> = m.flank_cols(i).map(|q| cols[q]).collect();
        Seen::of(m, i, &c)
    };
    let o = m.lead;
    assemble(m, cols, [seen(0), seen(1)], poses(m, cols[o], cols[o + 1], &cols[m.motion()..]))
}

/// The contact's frame from what it reads of each flank and the two poses, and the columns the
/// rows read besides.
fn assemble<T: Num>(m: &Mesh, cols: &[T], f: [Seen<T>; 2], p: [held::Frame<T>; 2]) -> At<T> {
    let o = m.lead;
    let motion = &cols[m.motion()..];
    let [(r1, p1), (r2, p2)] = p;
    let x1 = vadd(rotate(&r1, f[0].x), p1);
    let x2 = vadd(rotate(&r2, f[1].x), p2);
    let lever = dot(vsub(x2, col3(motion, 0)), vsub(x2, col3(motion, 0))).sqrt();
    let lever = if lever.val() > 1e-9 { lever } else { T::cst(1.0) };
    // a formed (or traced) flank's roll, held at none, over the lever so it is a length as a
    // meshing equation is
    let own = |l: &Side, g: T, t: T| if l.generated() { g } else { t * lever };
    At {
        x: [x1, x2],
        n: [rotate(&r1, f[0].n), rotate(&r2, f[1].n)],
        along: unit(rotate(&r2, f[1].du)),
        own: [own(&m.flanks[0], f[0].g, cols[o + 4]), own(&m.flanks[1], f[1].g, cols[o + 7])],
        member: [f[0].x, f[1].x],
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
    write(&m, cols, &at(&m, cols), out);
    Some(())
}

/// The rows from the contact's frame: the point's, where there is one, then the contact's.
fn write<T: Num>(m: &Mesh, cols: &[T], f: &At<T>, out: &mut [T]) {
    let mut i = 0;
    if m.lead == LEAD {
        let p = col3(cols, 0);
        for c in 0..3 {
            out[c] = p[c] - f.x[0][c];
        }
        i = 3;
    }
    touching(f, &mut out[i..i + ROWS]);
}

/// The contact's own rows from its frame: the two flanks' offset, the pinion's normal against two
/// directions square to the gear's (each times the lever), and each flank's own row.
fn touching<T: Num>(f: &At<T>, out: &mut [T]) {
    for c in 0..3 {
        out[c] = f.x[0][c] - f.x[1][c];
    }
    let across = cross(f.n[1], f.along);
    out[3] = dot(f.n[0], f.along) * f.lever;
    out[4] = dot(f.n[0], across) * f.lever;
    out[5] = f.own[0];
    out[6] = f.own[1];
}

/* -- the normal and the path of contact at a point (#345) -------------------------------------- */

/// A line's rows at a point held to the contact, over its columns `cols` and constants `k`, into
/// `out`: its direction (from its first lifted end to its second, or an axis's) square to two
/// directions square to the common normal, or to the path of contact.  `None` where the constants
/// do not describe this contact over this many columns.
fn line_rows<T: Num>(c: Contact, cols: &[T], k: &[f64], out: &mut [T]) -> Option<()> {
    let m = Mesh::read(k)?;
    let fits = m.lead == c.lead() && cols.len() == m.width() && out.len() == c.rows();
    if c == Contact::Point || !fits {
        return None;
    }
    let d = unit(vsub(col3(cols, 3), col3(cols, 0)));
    let [a, b] = match c {
        // the gear's tangent plane, which at a root is the pinion's
        Contact::Normal => {
            let f = at(&m, cols);
            [f.along, cross(f.n[1], f.along)]
        }
        _ => square_to(path(&m, cols)),
    };
    out[0] = dot(d, a);
    out[1] = dot(d, b);
    Some(())
}

/// The path of contact at the unknowns in `cols`: the contact point's derivative along the phase,
/// `∂x/∂φ + ∂x/∂z · dz/dφ`, `z` the lag and the places, `dz/dφ` solving `J_z dz/dφ = −J_φ` over
/// the rows' Jacobian in the unknowns — not finite where `J_z` is singular (a line contact).
fn path<T: Num>(m: &Mesh, cols: &[T]) -> [T; 3] {
    path_of(&at(m, &unknowns(m, cols)))
}

/// `cols` as numbers carrying their derivatives in the contact's unknowns.
fn unknowns<T: Num>(m: &Mesh, cols: &[T]) -> Vec<Fwd<T, UNKNOWNS>> {
    let o = m.lead;
    cols.iter().enumerate().map(|(q, &v)| {
        if (o..o + UNKNOWNS).contains(&q) { Fwd::var(v, q - o) } else { Fwd::of(v) }
    }).collect()
}

/// The path of contact from the contact's frame over numbers carrying their derivatives in the
/// unknowns (`path`).
fn path_of<T: Num>(f: &At<Fwd<T, UNKNOWNS>>) -> [T; 3] {
    let mut r = [Fwd::of(T::cst(0.0)); ROWS];
    touching(f, &mut r);
    let zero = T::cst(0.0);
    let mut a: [[T; ROWS + 1]; ROWS] = std::array::from_fn(|i| std::array::from_fn(|j| {
        if j < ROWS { r[i].g[j + 1] } else { zero - r[i].g[0] }
    }));
    let w = solve(&mut a);
    std::array::from_fn(|c| {
        let p = f.x[0][c];
        (0..ROWS).fold(p.g[0], |s, q| s + p.g[q + 1] * w[q])
    })
}

/// Two unit directions square to `t` and to each other: across the world axis `t` runs least
/// along, so the choice holds still near a point (as `held::curvature` chooses by `∇g`).  Where
/// two axes tie the pair turns about `t` but spans the same plane, so how far a line is off `t`
/// does not jump.
fn square_to<T: Num>(t: [T; 3]) -> [[T; 3]; 2] {
    let t = unit(t);
    let least = (0..3).min_by(|&i, &j| t[i].val().abs().total_cmp(&t[j].val().abs())).unwrap_or(0);
    let mut e = [T::cst(0.0); 3];
    e[least] = T::cst(1.0);
    let a = unit(cross(t, e));
    [a, cross(t, a)]
}

/// `A w = b`, the augmented `[A | b]` given, by elimination with partial pivoting on the values:
/// not finite where `A` is singular.
fn solve<T: Num, const N: usize, const M: usize>(a: &mut [[T; M]; N]) -> [T; N] {
    for k in 0..N {
        let p = (k..N).max_by(|&i, &j| a[i][k].val().abs().total_cmp(&a[j][k].val().abs()))
            .unwrap_or(k);
        a.swap(k, p);
        for i in k + 1..N {
            let f = a[i][k] / a[k][k];
            for j in k..M {
                a[i][j] = a[i][j] - f * a[k][j];
            }
        }
    }
    let mut w = [T::cst(0.0); N];
    for k in (0..N).rev() {
        let s = (k + 1..N).fold(a[k][N], |s, j| s - a[k][j] * w[j]);
        w[k] = s / a[k][k];
    }
    w
}

/// A number and its derivatives in `N` of a contact's unknowns, each itself a `T`: the rows read
/// over it give their Jacobian in the unknowns as numbers of the kernel's own kind, so what is
/// read off that Jacobian (`path`) is differentiated exactly where `T` is a `Dual`.
#[derive(Clone, Copy)]
struct Fwd<T, const N: usize> {
    v: T,
    g: [T; N],
}

impl<T: Num, const N: usize> Fwd<T, N> {
    fn of(v: T) -> Self {
        Fwd { v, g: [T::cst(0.0); N] }
    }
    fn var(v: T, i: usize) -> Self {
        let mut x = Fwd::of(v);
        x.g[i] = T::cst(1.0);
        x
    }
    /// `v`, its derivatives this one's times `d`.
    fn chain(self, v: T, d: T) -> Self {
        Fwd { v, g: self.g.map(|x| x * d) }
    }
}

impl<T: Num, const N: usize> std::ops::Add for Fwd<T, N> {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Fwd { v: self.v + o.v, g: std::array::from_fn(|k| self.g[k] + o.g[k]) }
    }
}

impl<T: Num, const N: usize> std::ops::Sub for Fwd<T, N> {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Fwd { v: self.v - o.v, g: std::array::from_fn(|k| self.g[k] - o.g[k]) }
    }
}

impl<T: Num, const N: usize> std::ops::Mul for Fwd<T, N> {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        Fwd { v: self.v * o.v, g: std::array::from_fn(|k| self.g[k] * o.v + self.v * o.g[k]) }
    }
}

impl<T: Num, const N: usize> std::ops::Div for Fwd<T, N> {
    type Output = Self;
    fn div(self, o: Self) -> Self {
        let q = self.v / o.v;
        Fwd { v: q, g: std::array::from_fn(|k| (self.g[k] - q * o.g[k]) / o.v) }
    }
}

impl<T: Num, const N: usize> Num for Fwd<T, N> {
    fn cst(v: f64) -> Self {
        Fwd::of(T::cst(v))
    }
    fn val(self) -> f64 {
        self.v.val()
    }
    /// No derivatives where the root is zero, as `Dual`'s.
    fn sqrt(self) -> Self {
        let s = self.v.sqrt();
        let d = if s.val() > 0.0 { T::cst(0.5) / s } else { T::cst(0.0) };
        self.chain(s, d)
    }
    fn sin_cos(self) -> (Self, Self) {
        let (s, c) = self.v.sin_cos();
        (self.chain(s, c), self.chain(c, T::cst(0.0) - s))
    }
    fn atan2(y: Self, x: Self) -> Self {
        let q = x.v * x.v + y.v * y.v;
        let q = if q.val() > 0.0 { q } else { T::cst(1.0) };
        let g = std::array::from_fn(|k| (x.v * y.g[k] - y.v * x.g[k]) / q);
        Fwd { v: T::atan2(y.v, x.v), g }
    }
}

/* -- the kernel ------------------------------------------------------------------------------ */

fn res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    held::block_res(n, v, k, r, LEAD + ROWS, rows);
}

fn line_res<const C: u8>(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let c = Contact::ALL[C as usize];
    held::block_res(n, v, k, r, c.rows(), |x, kc, out| line_rows(c, x, kc, out));
}

/// A line along the common normal's Jacobian, exact by `Dual` over every column at once
/// (`held::block_jac`).
fn line_jac<const C: u8>(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let c = Contact::ALL[C as usize];
    held::block_jac(n, v, k, j, c.rows(), |x, kc, out| line_rows(c, x, kc, out));
}

/// A line along the path of contact's Jacobian: its second derivatives, exact by `Dual` over
/// `Fwd`, a block at a time as the point's are (`jac`).
fn path_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let (n_par, _) = curve_widths(n, v, k);
    held::block_res(n, v, k, j, Contact::Path.rows() * n_par, path_jac_block);
}

/// One line's: each flank read over its own columns with its derivatives in its own place, and
/// the poses over theirs with their derivatives in the phase and the lag, each differentiated
/// in its own columns only; then the contact's frame over every column by the chain rule, and the
/// path read off it (`path_of`).
fn path_jac_block(x: &[f64], k: &[f64], j: &mut [f64]) -> Option<()> {
    let c = Contact::Path;
    let m = Mesh::read(k)?;
    let (w, o) = (x.len(), m.lead);
    if o != c.lead() || w != m.width() || j.len() != c.rows() * w {
        return None;
    }
    type Place = Fwd<Dual<PASS>, 3>;
    type Phase = Fwd<Dual<PASS>, 2>;
    let f = [0, 1].map(|i| derive::<{ SEEN * 4 }>(x, m.flank_cols(i), |cols| {
        let c: Vec<Place> = cols.iter().enumerate()
            .map(|(q, &v)| if q < 3 { Fwd::var(v, q) } else { Fwd::of(v) }).collect();
        let s = Seen::of(&m, i, &c).flat();
        std::array::from_fn(|q| if q % 4 == 0 { s[q / 4].v } else { s[q / 4].g[q % 4 - 1] })
    }));
    let p = derive::<{ POSES * 3 }>(x, m.pose_cols(), |cols| {
        let c: Vec<Phase> = cols.iter().enumerate()
            .map(|(q, &v)| if q < 2 { Fwd::var(v, q) } else { Fwd::of(v) }).collect();
        let a = flat_poses(poses(&m, c[0], c[1], &c[2..]));
        std::array::from_fn(|q| if q % 3 == 0 { a[q / 3].v } else { a[q / 3].g[q % 3 - 1] })
    });
    // a value and its derivatives in `slots` of the unknowns, from it and its derivatives in those
    let spread = |a: &[Dual<PASS>], slots: Range<usize>| {
        let mut g = [Dual::<PASS>::cst(0.0); UNKNOWNS];
        for (k, s) in slots.enumerate() {
            g[s] = a[1 + k];
        }
        Fwd { v: a[0], g }
    };
    let mut cols = vec![Dual::<PASS>::cst(0.0); w];
    for start in (0..w).step_by(PASS) {
        held::seed_pass(&mut cols, x, start);
        let seen = [0, 1].map(|i| {
            let a = f[i].at(start);
            let place = 2 + 3 * i..5 + 3 * i;
            Seen::read(&std::array::from_fn(|q| spread(&a[4 * q..4 * q + 4], place.clone())))
        });
        let a = p.at(start);
        let poses = read_poses(&std::array::from_fn(|q| spread(&a[3 * q..3 * q + 3], 0..2)));
        let tau = path_of(&assemble(&m, &unknowns(&m, &cols), seen, poses));
        let d = unit(vsub(col3(&cols, 3), col3(&cols, 0)));
        let [a1, b1] = square_to(tau);
        held::scatter_pass(&[dot(d, a1), dot(d, b1)], j, start);
    }
    Some(())
}

/// The Jacobian, exact by `Dual`, a block at a time (#302): each flank and the poses over their
/// own columns only, then the rows over those by the chain rule (`docs/contract/analytic.md`).
fn jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let (n_par, _) = curve_widths(n, v, k);
    held::block_res(n, v, k, j, (LEAD + ROWS) * n_par, jac_block);
}

fn jac_block(x: &[f64], k: &[f64], j: &mut [f64]) -> Option<()> {
    let m = Mesh::read(k)?;
    let w = x.len();
    if w != m.width() || j.len() != m.rows() * w {
        return None;
    }
    let f = [0, 1].map(|i| derive(x, m.flank_cols(i), |c| Seen::of(&m, i, c).flat()));
    let p = derive(x, m.pose_cols(), |c| flat_poses(poses(&m, c[0], c[1], &c[2..])));
    let mut cols = vec![Dual::<PASS>::cst(0.0); w];
    let mut out = vec![Dual::<PASS>::cst(0.0); m.rows()];
    for start in (0..w).step_by(PASS) {
        held::seed_pass(&mut cols, x, start);
        let seen = f.each_ref().map(|d| Seen::read(&d.at(start)));
        write(&m, &cols, &assemble(&m, &cols, seen, read_poses(&p.at(start))), &mut out);
        held::scatter_pass(&out, j, start);
    }
    Some(())
}

/// A block's `K` values and their Jacobian in every column of the contact (zero off the block's),
/// row-major.
struct Derived<const K: usize> {
    v: [f64; K],
    j: Vec<f64>,
}

impl<const K: usize> Derived<K> {
    /// The values as numbers carrying their derivatives in the contact's columns
    /// `start..start + PASS`.
    fn at(&self, start: usize) -> [Dual<PASS>; K] {
        let w = self.j.len() / K;
        std::array::from_fn(|o| {
            let mut d = Dual::cst(self.v[o]);
            for q in start..(start + PASS).min(w) {
                d.g[q - start] = self.j[o * w + q];
            }
            d
        })
    }
}

/// `f` over the columns `idx` of `x`, differentiated in those columns, `PASS` a pass.
fn derive<const K: usize>(x: &[f64], idx: impl Iterator<Item = usize>,
                          f: impl Fn(&[Dual<PASS>]) -> [Dual<PASS>; K]) -> Derived<K> {
    let w = x.len();
    let idx: Vec<usize> = idx.collect();
    let xs: Vec<f64> = idx.iter().map(|&q| x[q]).collect();
    let mut d = Derived { v: [0.0; K], j: vec![0.0; K * w] };
    let mut c = vec![Dual::<PASS>::cst(0.0); idx.len()];
    for start in (0..idx.len()).step_by(PASS) {
        held::seed_pass(&mut c, &xs, start);
        let y = f(&c);
        for (o, val) in y.iter().enumerate() {
            d.v[o] = val.v;
            for p in start..(start + PASS).min(idx.len()) {
                d.j[o * w + idx[p]] = val.g[p - start];
            }
        }
    }
    d
}

/// The kernel of contact `c` with a meshing at a block's widths (`system::build_kernel`).
pub fn kernel(c: Contact, n_par: usize, n_const: usize) -> Kernel {
    let (res, jac): (crate::kernels::KernelFn, crate::kernels::KernelFn) = match c {
        Contact::Point => (res, jac),
        Contact::Normal => (line_res::<1>, line_jac::<1>),
        Contact::Path => (line_res::<2>, path_jac),
    };
    Kernel { name: c.name(), n_res: c.rows(), n_par, n_const, degree: c.degree(), res, jac,
             const_jac: None, form: None }
}

/* -- reading the sketch ------------------------------------------------------------------------ */

use crate::constraints::CKind;
use crate::model::{Flank, Sketch};

/// What a contact reads of one flank, off the sketch: an envelope's or a source's (`held`), or a
/// traced surface's (#326), by its index.
#[derive(Clone, Debug)]
pub enum FlankParts {
    Held(held::Parts),
    Traced(usize),
}

impl FlankParts {
    fn lifted(&self, sk: &Sketch) -> Vec<usize> {
        match self {
            FlankParts::Held(p) => p.lifted(),
            FlankParts::Traced(i) => crate::traced::lifted(sk, &sk.traced[*i]),
        }
    }
    fn columns(&self, sk: &Sketch) -> Vec<u32> {
        match self {
            FlankParts::Held(p) => p.columns(sk),
            FlankParts::Traced(i) => crate::traced::columns(sk, &sk.traced[*i]),
        }
    }
    fn values(&self, sk: &Sketch) -> Vec<f64> {
        match self {
            FlankParts::Held(p) => p.values(sk),
            FlankParts::Traced(i) => crate::traced::theta(sk, &sk.traced[*i]),
        }
    }
    fn consts(&self, sk: &Sketch) -> Vec<f64> {
        match self {
            FlankParts::Held(p) => p.consts_at(0),
            FlankParts::Traced(i) => crate::traced::flank_consts(&sk.traced[*i]),
        }
    }
    fn own_width(&self, sk: &Sketch) -> usize {
        match self {
            FlankParts::Held(p) => p.own_width(),
            FlankParts::Traced(i) => sk.traced[*i].body.n_theta(),
        }
    }
}

/// What a contact reads of a meshing, off the sketch: each flank's parts and the mesh's.
#[derive(Clone, Debug)]
pub struct Parts {
    pub flanks: [FlankParts; 2],
    pub moving: held::Moving,
}

impl Parts {
    /// Meshing `index`'s parts, or why the solve cannot read it.
    pub fn of(sk: &Sketch, index: usize) -> Result<Parts, String> {
        let e = sk.meshings.get(index).ok_or("no such meshing")?;
        let flank = |f: &Flank, which: &str| -> Result<FlankParts, String> {
            match f {
                Flank::Envelope(i) => held::Parts::of(sk, *i as usize).map(FlankParts::Held).map_err(|why| {
                    format!("its {which} flank is an envelope the solve cannot read: {why}")
                }),
                Flank::Formed(h) => held::Parts::still(sk, h, &e.name).map(FlankParts::Held).map_err(|why| {
                    format!("its {which} flank is a surface the solve cannot read: {why}")
                }),
                Flank::Traced(i) => match sk.traced.get(*i as usize) {
                    Some(t) if t.formals() == 2 => Ok(FlankParts::Traced(*i as usize)),
                    _ => Err(format!("its {which} flank is no surface traced in space the solve \
                                      can read")),
                },
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
    pub fn lifted(&self, sk: &Sketch) -> Vec<usize> {
        [self.flanks[0].lifted(sk), self.flanks[1].lifted(sk), self.moving.lifted()].concat()
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
    pub fn consts(&self, sk: &Sketch, lead: usize) -> Vec<f64> {
        let flanks = [self.flanks[0].consts(sk), self.flanks[1].consts(sk)];
        consts(lead, [&flanks[0], &flanks[1]], &self.moving.steps)
    }

    pub fn n_const(&self, sk: &Sketch) -> usize {
        self.consts(sk, 0).len()
    }

    /// How many columns a contact with a lead of `lead` reads.
    pub fn width(&self, sk: &Sketch, lead: usize) -> usize {
        let flanks = self.flanks[0].own_width(sk) + self.flanks[1].own_width(sk);
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
            Step::Turn { ratio, .. } => Some(ratio),
            _ => None,
        };
        let scale = parts.iter().fold(0.0f64, |a, x| a.max(x.abs())).max(1.0);
        Some(Meshing { k, parts, roll: roll.map(f64::to_degrees), ratio, anchor: None, scale })
    }

    /// Meshing `index` as the drawing stands, anchored where the first point held to it is.
    pub fn named(sk: &Sketch, index: usize) -> Result<Meshing, String> {
        let p = Parts::of(sk, index)?;
        let mut m = Meshing::new(p.consts(sk, 0), p.values(sk), sk.meshings[index].roll)
            .ok_or("the meshing's parts are not a meshing")?;
        let mut points = sk.constraints.iter().filter(|c| c.kind == CKind::PointOnMeshing);
        m.anchor = points.find_map(|c| {
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
        let parts = m.flank_parts(&mesh, i);
        match &mesh.flanks[i] {
            Side::Traced { domain, k, .. } => {
                crate::traced::nearest(k, parts, seen(i), *domain).map(|[u, s]| [u, s, 0.0])
            }
            Side::Held(l) if l.steps.is_empty() => still_seed(l, parts, seen(i)),
            Side::Held(_) => {
                let mut k = m.k[mesh.consts[i].clone()].to_vec();
                k[0] = LEAD as f64;
                held::seed(&k, parts, seen(i), rolls[i])
            }
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
    let reach = held::reach(l.source, parts, q);
    let us = held::meridian_spans(l.source, reach);
    let vs = held::turn_spans(l.source, reach, 36);
    let mut best: Option<([f64; 2], f64)> = None;
    for &u in &us {
        for &v in &vs {
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
