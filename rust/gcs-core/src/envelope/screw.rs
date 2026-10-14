//! **The screw of a motion, as the solve reads it** (#326, R2).
//!
//! A turn about a fixed line, a screw along one or a slide moves every point at a velocity that is
//! the same at every time in the fixed frame: its **twist**, `v(x) = v₀ + ω × x` per radian of the
//! motion's roll.  So does one such motion seen from another (`relative_to:`) — two members turning
//! at a constant ratio about fixed shafts — whose twist is the source's less the observer's.  Its
//! **instantaneous screw axis** is the line along `ω` through `ω × v₀ / |ω|²`, where every point
//! moves along it, by the **pitch** `ω·v₀ / |ω|²` a radian (Ball).  Two relations read it:
//!
//! - `l reciprocal mesh`: the line `l` is **reciprocal** to the motion's twist — a force along it does
//!   no work on the relative motion, `d · v(p) = 0` for its direction `d` and any point `p` of it (the
//!   same number at every point: `d · (ω × d) = 0`).  One row, of degree 2, the direction left
//!   unnormalised.  For a mesh it is the meshing law of a line of action: the members move as
//!   fast along it (Phillips' spatial involute gearing).
//! - `isa screw mesh`: the axis `isa` is the motion's instantaneous screw axis and the pitch its
//!   own unknown — two rows putting its direction along `ω` and two its place on the line, each a
//!   length (the direction's times the lever, the first axis the motion reads, as long as it is
//!   drawn), and the pitch.
//!
//! **Layout.** Columns `[lead…, motion…]`: the line's two lifted ends (six), or the axis's
//! direction, its place and the pitch (seven), then two lifted ends per axis the motion turns about
//! or slides along (`held::Moving`).  Constants: the motion's steps (`held::encode_steps`).  Every
//! row is written once over `kernels::Num`, so the Jacobian is exact by forward differentiation.
use super::held::{self, cross, dot, vadd, vsub, Step, STEP};
use crate::kernels::{Kernel, Num};

/// Which statement the twist is read for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reading {
    /// `l reciprocal mesh`: one row.
    Reciprocal = 0,
    /// `isa screw mesh`: five rows.
    Axis = 1,
}

impl Reading {
    pub const ALL: [Reading; 2] = [Reading::Reciprocal, Reading::Axis];

    pub fn rows(self) -> usize {
        match self {
            Reading::Reciprocal => 1,
            Reading::Axis => 5,
        }
    }

    /// The columns before the motion's.
    pub fn lead(self) -> usize {
        match self {
            Reading::Reciprocal => 6,
            Reading::Axis => 7,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Reading::Reciprocal => "reciprocal",
            Reading::Axis => "screw_axis",
        }
    }

    /// A force's work is a length times a velocity; an axis's place and its pitch are lengths.
    fn degree(self) -> u32 {
        match self {
            Reading::Reciprocal => 2,
            Reading::Axis => 1,
        }
    }
}

fn scale<T: Num>(a: [T; 3], k: T) -> [T; 3] {
    [a[0] * k, a[1] * k, a[2] * k]
}

/// One step's twist `(ω, v₀)`, where it is a turn, a screw or a slide about a fixed line.
fn own<T: Num>(s: &Step, m: &[T]) -> Option<([T; 3], [T; 3])> {
    let tau = std::f64::consts::TAU;
    let axis = |at: usize| {
        let a = held::col3(m, at);
        (a, held::unit(vsub(held::col3(m, at + 3), a)))
    };
    match *s {
        Step::Rotation { at, ratio, advance, .. } => {
            let (a, d) = axis(at);
            let w = scale(d, T::cst(ratio));
            Some((w, vadd(cross(a, w), scale(d, T::cst(advance / tau)))))
        }
        Step::Translation { at, advance } => {
            let (_, d) = axis(at);
            Some(([T::cst(0.0); 3], scale(d, T::cst(advance / tau))))
        }
        Step::Relative { .. } => None,
    }
}

/// The motion's twist in the fixed frame, `v(x) = v₀ + ω × x` per radian: its last step's, a
/// relative one's its source's less its observer's.  `None` where it is not constant — a relative
/// of a relative.
pub(crate) fn twist<T: Num>(steps: &[Step], m: &[T]) -> Option<([T; 3], [T; 3])> {
    match steps.last()? {
        Step::Relative { source, observer } => {
            let (s, o) = (own(&steps[*source], m)?, own(&steps[*observer], m)?);
            Some((vsub(s.0, o.0), vsub(s.1, o.1)))
        }
        s => own(s, m),
    }
}

/// Whether a motion's steps have a constant twist: what a relation reading one asks first.
pub fn constant(steps: &[Step]) -> bool {
    match steps.last() {
        Some(Step::Relative { source, observer }) => [source, observer].iter()
            .all(|&&i| !matches!(steps.get(i), Some(Step::Relative { .. }) | None)),
        Some(_) => true,
        None => false,
    }
}

/// The instantaneous screw of a twist: a point of its axis (the nearest the origin), the axis's
/// unit direction along `ω`, and the pitch, a length a radian.  `None` where `ω` is zero (a
/// slide, whose axis is at infinity).
pub(crate) fn axis_of<T: Num>(w: [T; 3], v0: [T; 3]) -> Option<([T; 3], [T; 3], T)> {
    let ww = dot(w, w);
    if !(ww.val() > 0.0) {
        return None;
    }
    let inv = T::cst(1.0) / ww;
    Some((scale(cross(w, v0), inv), held::unit(w), dot(w, v0) * inv))
}

/// Two unit directions square to the unit `d` and to one another: the first the world axis `d`
/// leans least toward, `d`'s part taken off.
fn square_to<T: Num>(d: [T; 3]) -> ([T; 3], [T; 3]) {
    let k = (0..3).min_by(|&i, &j| d[i].val().abs().total_cmp(&d[j].val().abs())).unwrap_or(0);
    let e: [T; 3] = std::array::from_fn(|i| T::cst(if i == k { 1.0 } else { 0.0 }));
    let t1 = held::unit(vsub(e, scale(d, d[k])));
    (t1, cross(d, t1))
}

/// A reading's rows over its columns `cols` and the motion's steps `k`, into `out`.  `None` where
/// the constants are no steps of a constant twist over this many columns, or the twist turns
/// nothing and has no axis.
pub(crate) fn rows<T: Num>(r: Reading, cols: &[T], k: &[f64], out: &mut [T]) -> Option<()> {
    let steps = held::read_steps(k)?;
    if !constant(&steps) || cols.len() != r.lead() + held::steps_width(&steps) || out.len() != r.rows() {
        return None;
    }
    let m = &cols[r.lead()..];
    let (w, v0) = twist(&steps, m)?;
    match r {
        Reading::Reciprocal => {
            let (p1, p2) = (held::col3(cols, 0), held::col3(cols, 3));
            out[0] = dot(vsub(p2, p1), vadd(v0, cross(w, p1)));
        }
        Reading::Axis => {
            let (d, a, pitch) = (held::col3(cols, 0), held::col3(cols, 3), cols[6]);
            let (c, along, h) = axis_of(w, v0)?;
            let (t1, t2) = square_to(along);
            // the lever: the first axis the motion reads, as long as it is drawn
            let lever = dot(vsub(held::col3(m, 3), held::col3(m, 0)), vsub(held::col3(m, 3), held::col3(m, 0))).sqrt();
            let off = vsub(a, c);
            out[0] = dot(d, t1) * lever;
            out[1] = dot(d, t2) * lever;
            out[2] = dot(off, t1);
            out[3] = dot(off, t2);
            out[4] = pitch - h;
        }
    }
    Some(())
}

fn res<const R: u8>(n: usize, v: &[f64], k: &[f64], out: &mut [f64]) {
    let r = Reading::ALL[R as usize];
    held::block_res(n, v, k, out, r.rows(), |x, kc, o| rows(r, x, kc, o));
}

fn jac<const R: u8>(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let r = Reading::ALL[R as usize];
    held::block_jac(n, v, k, j, r.rows(), |x, kc, o| rows(r, x, kc, o));
}

/// The kernel of a reading at a block's widths (`system::build_kernel`).
pub fn kernel(r: Reading, n_par: usize, n_const: usize) -> Kernel {
    let (res, jac): (fn(usize, &[f64], &[f64], &mut [f64]), fn(usize, &[f64], &[f64], &mut [f64])) = match r {
        Reading::Reciprocal => (res::<0>, jac::<0>),
        Reading::Axis => (res::<1>, jac::<1>),
    };
    Kernel { name: r.name(), n_res: r.rows(), n_par, n_const, degree: r.degree(), res, jac, const_jac: None, form: None }
}

/// The constants a reading of a motion with these steps takes.
pub fn consts(steps: &[Step]) -> Vec<f64> {
    held::encode_steps(steps)
}

/// How many constants a reading of `steps` takes.
pub fn n_const(steps: &[Step]) -> usize {
    STEP * steps.len()
}

/// The pitch and the axis of the screw a motion's steps make over its columns' values `m`, as
/// numbers: `(point, direction, pitch)`, or `None` where it has none.
pub fn screw(steps: &[Step], m: &[f64]) -> Option<([f64; 3], [f64; 3], f64)> {
    if !constant(steps) || m.len() != held::steps_width(steps) {
        return None;
    }
    let (w, v0) = twist(steps, m)?;
    axis_of(w, v0)
}
