//! **The rate of a contact: one kernel** (#365).
//!
//! A contact holds a point at its place — a curve's parameter, a traced body's formals, a
//! meshing's phase, an envelope's place, a carried point's roll — and every relation at that point
//! reads the same thing: the contact's **frame** there, the derivative of its place in its slots.
//! The rate in one slot, `rate(p over t)`, held along a line or square to one or measured along
//! one; the normal, square to every rate of a surface; a plane along that normal; a line in the
//! tangent plane, square to it; the circle osculating a section, the second rates read through
//! Meusnier; and a curve's centre of curvature, from its first rate and its second.  So one
//! kernel reads them all (`CKind::Rate`, `CKind::RateDistance`), its rows formed here over a
//! `Frame` in the form the source's own kernel had (`Form`), and each **source** only says what
//! its frame is: a spline's and a curve's in their view (`kernels::spline_rate`, `curve_rate`), a
//! traced body's (`traced::frame`), a meshing's (`meshing::frame`, the path of contact by the
//! implicit function theorem, the common normal the gear's flank's), a carried point's
//! (`held::carried`) and an envelope's (`held::frame`).
//!
//! **Partial, never total**: a source's frame is the derivative in its own slots with every other
//! column held — a contact's own other unknowns following its rows where the source says so (a
//! meshing's lag and places, an envelope's roll).  **Exact**: each source gives its frame and the
//! frame's derivative in each of its own columns, worked out once a block; the Jacobian reads the
//! frame linearised over a pass of `Dual`s (`held::PASS` columns), and the words are written once
//! over `kernels::Num`.
//!
//! **Layout.**  Columns `[lead…, source…, free?]`: what the frame is held against — a line drawn in
//! the view (four), a line's lifted ends or an axis's place and direction (six), a plane's two axes
//! (six), a circle's centre's lift, radius and plane's two axes (ten), a point's lift (three) —
//! then the source's own
//! columns, then a distance's unknown where it is written free.  Constants `[HEADER…, source…]`:
//! the source, the word, the slot, whether the distance is free, the distance as `(m, c)`, then
//! the source's own.
use crate::envelope::held::{self, col3, cross, dot, unit, vscale, vsub, PASS};
use crate::kernels::{curve_widths, Dual, Kernel, KernelFn, Num};

/// The constants before a source's own.
pub const HEADER: usize = 6;

/// The numbers of a frame, flat.
const FLAT: usize = 24;

/// The smallest normal curvature a section's circle divides by.
const FLATTEST: f64 = 1e-12;

/// What a contact's frame is read off.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// A point on a spline, in its view: `[t, one span's control points]`.
    Spline = 0,
    /// A point on a curve written in the language, in its view, by its body (`kernels::FORMULA`
    /// …): `[u, θ…]`.
    Formula = 1,
    Trace = 2,
    Profile = 3,
    Extremal = 4,
    /// A point on a body traced in space: `[u, s, θ…]`.
    Traced = 5,
    /// A point on a meshing: `[φ, δ, u₁, v₁, t₁, u₂, v₂, t₂, flanks…, motion…, ease…]`.
    Meshing = 6,
    /// A point a motion carries: `[x's lift, τ, motion…]`.
    Carried = 7,
    /// A point on an envelope in space: `[u, v, t, source…, motion…]`.
    Envelope = 8,
}

impl Source {
    pub const ALL: [Source; 9] = [Source::Spline, Source::Formula, Source::Trace, Source::Profile,
                                  Source::Extremal, Source::Traced, Source::Meshing,
                                  Source::Carried, Source::Envelope];

    /// Read in a view, as against in space.
    pub fn planar(self) -> bool {
        (self as u8) <= Source::Extremal as u8
    }

    /// A curve's body as its kernels say it (`kernels::FORMULA` …).
    fn body(self) -> u8 {
        (self as u8).saturating_sub(1)
    }

    /// A curve of this body.
    pub fn curve(body: u8) -> Source {
        Source::ALL[1 + body.min(3) as usize]
    }

    /// The form its rate is held to a line in.
    pub fn form(self) -> Form {
        match self {
            _ if self.planar() => Form::Planar,
            Source::Carried => Form::Velocity,
            Source::Meshing => Form::Path,
            _ => Form::Direction,
        }
    }
}

/// **How a source's rate is held to a line**: one word, its rows the form the source's own kernel
/// had (#365's second question, settled in `docs/contract/curves.md`), so folding a kind onto the
/// rate changed no value it solved to and no path its solve took.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Form {
    /// A curve in its view: `(C′ × (B − A))/‖B − A‖`, the rate across the line drawn there.
    Planar = 0,
    /// A carried point's velocity across or along the line — a length a radian, regular where it
    /// vanishes (a point on the axis of a screw of no pitch).
    Velocity = 1,
    /// A traced curve's tangent: the line's direction against two unit directions square to the
    /// rate (`screw::square_to`), or along it — cosines.
    Direction = 2,
    /// A meshing's path of contact: the same, the two directions square to the path across the
    /// world axis it runs least along (`across_least`).
    Path = 3,
}

impl Form {
    pub const ALL: [Form; 4] = [Form::Planar, Form::Velocity, Form::Direction, Form::Path];

    pub fn planar(self) -> bool {
        self == Form::Planar
    }
}

/// What a frame is held to: a word of the language, as a rate of a contact states it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Word {
    /// `rate(p over t) parallel l` — `g tangent(at: p) l` of a curve: the rate across the line.
    Parallel = 0,
    /// `rate(p over t) perpendicular l`: the rate along the line.
    Square = 1,
    /// `rate(p over t) distance(d, along: l)`: the rate along the line, less `d`.
    Along = 2,
    /// `S perpendicular(at: p) l`: the line along the normal, square to every rate.
    Normal = 3,
    /// `S tangent(at: p) P`: the plane's normal along the surface's.
    Plane = 4,
    /// `S tangent(at: p) l`: the line in the tangent plane, square to the normal.
    InPlane = 5,
    /// `S curvature(at: p) k`: the circle osculating the section by its plane at `p`.
    Curvature = 6,
    /// `k curvature(at: p) c`, `k` a curve in space and `c` a point: the centre of its osculating
    /// circle (#363 S5).
    Centre = 7,
}

impl Word {
    pub const ALL: [Word; 8] = [Word::Parallel, Word::Square, Word::Along, Word::Normal,
                                Word::Plane, Word::InPlane, Word::Curvature, Word::Centre];

    /// The word a `Rate` holds in its `word` slot, and a distance's.
    pub fn of(written: &str) -> Option<Word> {
        Some(match written {
            "tangent" | "parallel" => Word::Parallel,
            "perpendicular" => Word::Square,
            "along" => Word::Along,
            "normal" => Word::Normal,
            "plane" => Word::Plane,
            "in_plane" => Word::InPlane,
            "curvature" => Word::Curvature,
            "centre" => Word::Centre,
            _ => return None,
        })
    }

    pub fn rows(self, planar: bool) -> usize {
        match self {
            Word::Parallel if planar => 1,
            Word::Parallel | Word::Normal | Word::Plane => 2,
            Word::Square | Word::Along | Word::InPlane => 1,
            Word::Curvature | Word::Centre => 3,
        }
    }

    /// The columns before the source's.
    pub fn lead(self, planar: bool) -> usize {
        match self {
            _ if planar => 4,
            Word::Curvature => 10,
            Word::Centre => 3,
            _ => 6,
        }
    }

    /// A rate's components are lengths a unit of the slot, and a circle's offsets lengths; a
    /// direction against the normal or the unit rate is cosines.
    pub fn degree(self, form: Form) -> u32 {
        match self {
            Word::Normal | Word::Plane | Word::InPlane => 0,
            Word::Parallel | Word::Square if matches!(form, Form::Direction | Form::Path) => 0,
            _ => 1,
        }
    }

    /// Whether the frame's second order is read.
    pub fn second(self) -> bool {
        matches!(self, Word::Curvature | Word::Centre)
    }

    fn name(self) -> &'static str {
        match self {
            Word::Parallel => "rate_parallel",
            Word::Square => "rate_square",
            Word::Along => "rate_along",
            Word::Normal => "rate_normal",
            Word::Plane => "rate_plane",
            Word::InPlane => "rate_in_plane",
            Word::Curvature => "rate_curvature",
            Word::Centre => "rate_centre",
        }
    }
}

/// **A contact's frame** at the point it holds: the point, the rate in the slot a statement names,
/// a direction a normal's rows are read across, the unit normal, and — its second order asked —
/// the second rate in that slot and, of a surface, two directions of its tangent plane and its
/// second fundamental form along them, `[L, M, N]`.  What a source has not (a curve's normal) is
/// zero.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Frame<T> {
    pub x: [T; 3],
    pub rate: [T; 3],
    pub across: [T; 3],
    pub n: [T; 3],
    pub tan: [[T; 3]; 2],
    pub second: [T; 3],
    pub accel: [T; 3],
}

impl<T: Num> Frame<T> {
    pub fn zero() -> Frame<T> {
        let z = [T::cst(0.0); 3];
        Frame { x: z, rate: z, across: z, n: z, tan: [z; 2], second: z, accel: z }
    }

    fn flat(&self) -> [T; FLAT] {
        let mut a = [T::cst(0.0); FLAT];
        let parts = [self.x, self.rate, self.across, self.n, self.tan[0], self.tan[1], self.second,
                     self.accel];
        for (k, p) in parts.iter().enumerate() {
            a[3 * k..3 * k + 3].copy_from_slice(p);
        }
        a
    }

    fn read(a: &[T; FLAT]) -> Frame<T> {
        let c = |k: usize| col3(a, 3 * k);
        Frame { x: c(0), rate: c(1), across: c(2), n: c(3), tan: [c(4), c(5)], second: c(6),
                accel: c(7) }
    }
}

/// A frame and its derivative in each of its source's columns, as numbers.
pub(crate) struct Read {
    pub frame: Frame<f64>,
    pub partial: Vec<Frame<f64>>,
}

impl Read {
    pub fn value(frame: Frame<f64>) -> Read {
        Read { frame, partial: Vec::new() }
    }

    /// A frame read over numbers carrying their derivatives in a source's columns, `PASS` a pass:
    /// `of` reads it over the columns `x` seeded for the pass at `start`.
    pub fn by_passes(x: &[f64], of: impl Fn(&[Dual<PASS>], usize) -> Option<Frame<Dual<PASS>>>)
        -> Option<Read> {
        let mut cols = vec![Dual::<PASS>::cst(0.0); x.len()];
        let mut frame = Frame::<f64>::zero();
        let mut partial = vec![Frame::<f64>::zero(); x.len()];
        for start in (0..x.len().max(1)).step_by(PASS) {
            held::seed_pass(&mut cols, x, start);
            let f = of(&cols, start)?.flat();
            frame = Frame::read(&f.map(|d| d.v));
            for q in start..(start + PASS).min(x.len()) {
                partial[q] = Frame::read(&f.map(|d| d.g[q - start]));
            }
        }
        Some(Read { frame, partial })
    }

    /// The frame over `cols`, the source's columns starting at `from`, to first order: the value,
    /// and each column's derivative times that column's change — exact where `T` is a `Dual`.
    fn linear<T: Num>(&self, cols: &[T], from: usize) -> Frame<T> {
        let v = self.frame.flat();
        let mut a: [T; FLAT] = v.map(T::cst);
        for (q, p) in self.partial.iter().enumerate() {
            let Some(&c) = cols.get(from + q) else { continue };
            let delta = c - T::cst(c.val());
            let p = p.flat();
            for i in 0..FLAT {
                if p[i] != 0.0 {
                    a[i] = a[i] + delta * T::cst(p[i]);
                }
            }
        }
        Frame::read(&a)
    }
}

/// The constants of a rate of a contact: what its frame is read off, the word, the slot, the
/// distance — `(m, c)`, `d = m·free + c` where `free`, else `c` — then the source's own.
pub fn consts(source: Source, word: Word, slot: usize, free: bool, stated: [f64; 2],
              own: &[f64]) -> Vec<f64> {
    let mut k = vec![source as u8 as f64, word as u8 as f64, slot as f64,
                     if free { 1.0 } else { 0.0 }, stated[0], stated[1]];
    k.extend_from_slice(own);
    k
}

/// The header read back.
struct Header<'k> {
    source: Source,
    word: Word,
    slot: usize,
    free: bool,
    stated: [f64; 2],
    own: &'k [f64],
}

fn header(k: &[f64]) -> Option<Header<'_>> {
    let tag = |i: usize, n: usize| k.get(i).copied()
        .filter(|v| v.is_finite() && *v >= 0.0 && (*v as usize) < n).map(|v| v as usize);
    Some(Header {
        source: Source::ALL[tag(0, Source::ALL.len())?],
        word: Word::ALL[tag(1, Word::ALL.len())?],
        slot: tag(2, 2)?,
        free: *k.get(3)? != 0.0,
        stated: [*k.get(4)?, *k.get(5)?],
        own: k.get(HEADER..)?,
    })
}

/// A source's frame over its own columns `cols` and constants `own`: the rate in `slot`, the
/// second order where `second`, and — where `grad` — its derivative in each column.
fn source_frame(h: &Header, cols: &[f64], grad: bool) -> Option<Read> {
    let second = h.word.second();
    let planar = |r: Option<([f64; 2], Vec<[f64; 2]>)>| {
        let (rate, partial) = r?;
        let one = |v: [f64; 2]| {
            let mut f = Frame::zero();
            f.rate = [v[0], v[1], 0.0];
            f
        };
        Some(Read { frame: one(rate), partial: partial.into_iter().map(one).collect() })
    };
    match h.source {
        Source::Spline => planar(crate::kernels::spline_rate(cols, h.own, grad)),
        Source::Formula | Source::Trace | Source::Profile | Source::Extremal => {
            planar(crate::kernels::curve_rate(h.source.body(), h.own, cols, grad))
        }
        Source::Traced => crate::traced::frame(cols, h.own, h.slot, second, grad),
        Source::Meshing => crate::envelope::meshing::frame(cols, h.own, grad),
        Source::Carried => {
            let steps = held::read_steps(h.own)?;
            if cols.len() != 4 + held::steps_width(&steps) {
                return None;
            }
            fn carried<T: Num>(steps: &[held::Step], cols: &[T]) -> Option<Frame<T>> {
                let deg = T::cst(std::f64::consts::PI / 180.0);
                let (x, rate) = held::carried(steps, &cols[4..], cols[3] * deg, col3(cols, 0))?;
                let mut f = Frame::zero();
                f.x = x;
                f.rate = rate;
                Some(f)
            }
            match grad {
                true => Read::by_passes(cols, |c, _| carried(&steps, c)),
                false => carried(&steps, cols).map(Read::value),
            }
        }
        Source::Envelope => {
            let l = held::Layout::read(h.own)?;
            if l.lead != 0 || cols.len() != l.width() {
                return None;
            }
            match grad {
                true => Read::by_passes(cols, |c, _| Some(held::frame(&l, c, second))),
                false => Some(Read::value(held::frame(&l, cols, second))),
            }
        }
    }
}

/// The normal curvature of a surface along `w`, a direction of its tangent plane: its second
/// fundamental form over its first along the frame's two directions.
fn normal_curvature<T: Num>(f: &Frame<T>, w: [T; 3]) -> T {
    let [t1, t2] = f.tan;
    let (e, ff, g) = (dot(t1, t1), dot(t1, t2), dot(t2, t2));
    let [l, m, n] = f.second;
    let (b1, b2) = (dot(t1, w), dot(t2, w));
    let det = e * g - ff * ff;
    let det = if det.val() != 0.0 { det } else { T::cst(1.0) };
    let (a, b) = ((g * b1 - ff * b2) / det, (e * b2 - ff * b1) / det);
    let two = T::cst(2.0);
    let second = l * a * a + two * m * a * b + n * b * b;
    let first = e * a * a + two * ff * a * b + g * b * b;
    let first = if first.val() != 0.0 { first } else { T::cst(1.0) };
    second / first
}

/// Two unit directions square to `t` and to each other, across the world axis `t` runs least
/// along — the meshing's path-of-contact basis, kept (`Form::Path`).
fn across_least<T: Num>(t: [T; 3]) -> [[T; 3]; 2] {
    let t = unit(t);
    let least = (0..3).min_by(|&i, &j| t[i].val().abs().total_cmp(&t[j].val().abs())).unwrap_or(0);
    let mut e = [T::cst(0.0); 3];
    e[least] = T::cst(1.0);
    let a = unit(cross(t, e));
    [a, cross(t, a)]
}

/// A word's rows over the lead `lead` and the frame `f`, the distance `stated`, into `out`, in the
/// source's form.
fn rows<T: Num>(w: Word, form: Form, lead: &[T], f: &Frame<T>, stated: T, out: &mut [T]) {
    if form.planar() {
        // a line drawn in the view: the rate across it and along it over its length, as a
        // curve's tangency is read
        let (dx, dy) = (lead[2] - lead[0], lead[3] - lead[1]);
        let len = (dx * dx + dy * dy).sqrt();
        let len = if len.val() > crate::kernels::MIN_LINE_LEN { len }
            else { T::cst(crate::kernels::MIN_LINE_LEN) };
        let [rx, ry, _] = f.rate;
        out[0] = match w {
            Word::Parallel => (rx * dy - ry * dx) / len,
            Word::Square => (rx * dx + ry * dy) / len,
            _ => (rx * dx + ry * dy) / len - stated,
        };
        return;
    }
    let dir = || unit(vsub(col3(lead, 3), col3(lead, 0)));
    match w {
        Word::Parallel => {
            let (a, b, against) = match form {
                Form::Velocity => {
                    let (e1, e2) = crate::envelope::screw::square_to(dir());
                    (e1, e2, f.rate)
                }
                Form::Path => {
                    let [e1, e2] = across_least(f.rate);
                    (e1, e2, dir())
                }
                _ => {
                    let (e1, e2) = crate::envelope::screw::square_to(unit(f.rate));
                    (e1, e2, dir())
                }
            };
            out[0] = dot(against, a);
            out[1] = dot(against, b);
        }
        Word::Square if form == Form::Velocity => out[0] = dot(f.rate, dir()),
        Word::Square => out[0] = dot(dir(), unit(f.rate)),
        Word::Along => out[0] = dot(f.rate, dir()) - stated,
        Word::InPlane => out[0] = dot(dir(), f.n),
        Word::Normal | Word::Plane => {
            let d = match w {
                Word::Plane => unit(cross(col3(lead, 0), col3(lead, 3))),
                _ => dir(),
            };
            let t1 = unit(f.across);
            let t2 = cross(f.n, t1);
            out[0] = dot(d, t1);
            out[1] = dot(d, t2);
        }
        // Meusnier: the section by a plane of normal `m` through the point is a curve turning
        // `κₙ / cos θ` toward the normal's part in the plane, `n − (n·m)m`, `κₙ` the normal
        // curvature along the section's tangent `m × n` — so its centre of curvature is
        // `x + (n − (n·m)m)/κₙ`, the circle's centre, in the plane's two directions, the radius
        // its distance from the point
        Word::Curvature => {
            let (centre, r) = (col3(lead, 0), lead[3]);
            let (pu, pv) = (col3(lead, 4), col3(lead, 7));
            let m = unit(cross(pu, pv));
            let kappa = normal_curvature(f, unit(cross(m, f.n)));
            let kappa = if kappa.val().abs() >= FLATTEST { kappa }
                else { T::cst(if kappa.val() < 0.0 { -FLATTEST } else { FLATTEST }) };
            let inward = vsub(f.n, vscale(m, dot(f.n, m)));
            let to = vsub(centre, f.x);
            let off = vsub(to, vscale(inward, T::cst(1.0) / kappa));
            let a = unit(pu);
            let b = unit(cross(m, a));
            out[0] = dot(off, a);
            out[1] = dot(off, b);
            out[2] = dot(to, to).sqrt() - r;
        }
        // the osculating circle's centre: `X + |X′|²·(b × X′)/|b|²`, `b = X′ × X″` the binormal
        // times `|X′|³κ` — no centre where it vanishes, its square floored so the rows stay
        // finite and the rank says so
        Word::Centre => {
            let b = cross(f.rate, f.accel);
            let bb = dot(b, b);
            let floor = FLATTEST * FLATTEST;
            let bb = if bb.val() > floor { bb } else { T::cst(floor) };
            let off = vscale(cross(b, f.rate), dot(f.rate, f.rate) / bb);
            for i in 0..3 {
                out[i] = lead[i] - (f.x[i] + off[i]);
            }
        }
    }
}

/// One block's widths: the lead, the source's columns and whether a free column ends them.
fn widths(h: &Header, n_cols: usize) -> Option<(usize, usize)> {
    let lead = h.word.lead(h.source.planar());
    let own = n_cols.checked_sub(lead + h.free as usize)?;
    Some((lead, own))
}

/// One block's rows, `out` its residuals.
fn block_rows(x: &[f64], k: &[f64], out: &mut [f64]) -> Option<()> {
    let h = header(k)?;
    let form = h.source.form();
    let (lead, own) = widths(&h, x.len())?;
    if out.len() != h.word.rows(form.planar()) {
        return None;
    }
    let f = source_frame(&h, &x[lead..lead + own], false)?.frame;
    let stated = if h.free { h.stated[0] * x[lead + own] + h.stated[1] } else { h.stated[1] };
    rows(h.word, form, &x[..lead], &f, stated, out);
    out.iter().all(|v| v.is_finite()).then_some(())
}

/// One block's Jacobian, row-major into `j`: the source's frame and its partials read once, then
/// the rows over the frame linearised a pass of columns at a time.
fn block_jac(x: &[f64], k: &[f64], j: &mut [f64]) -> Option<()> {
    let h = header(k)?;
    let form = h.source.form();
    let (lead, own) = widths(&h, x.len())?;
    let m = h.word.rows(form.planar());
    if j.len() != m * x.len() {
        return None;
    }
    let read = source_frame(&h, &x[lead..lead + own], true)?;
    let mut cols = vec![Dual::<PASS>::cst(0.0); x.len()];
    let mut out = vec![Dual::<PASS>::cst(0.0); m];
    for start in (0..x.len()).step_by(PASS) {
        held::seed_pass(&mut cols, x, start);
        let f = read.linear(&cols, lead);
        let stated = match h.free {
            true => Dual::cst(h.stated[0]) * cols[lead + own] + Dual::cst(h.stated[1]),
            false => Dual::cst(h.stated[1]),
        };
        rows(h.word, form, &cols[..lead], &f, stated, &mut out);
        held::scatter_pass(&out, j, start);
    }
    Some(())
}

fn res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let m = if n == 0 { 0 } else { r.len() / n };
    held::block_res(n, v, k, r, m, block_rows);
}

fn jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let (n_par, _) = curve_widths(n, v, k);
    let m = if n == 0 || n_par == 0 { 0 } else { j.len() / (n * n_par) };
    held::block_res(n, v, k, j, m * n_par, block_jac);
}

/// **The kernel** of a rate of a contact, at a block's widths: one, whatever the source, its rows
/// the word's (`system::build_kernel`).
pub fn kernel(word: Word, form: Form, n_par: usize, n_const: usize) -> Kernel {
    Kernel { name: word.name(), n_res: word.rows(form.planar()), n_par, n_const,
             degree: word.degree(form),
             res: res as KernelFn, jac: jac as KernelFn, const_jac: None, form: None }
}
