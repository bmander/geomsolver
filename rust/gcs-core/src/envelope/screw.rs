//! **A point a motion carries, and its rate** (#326, #365).
//!
//! `X coincident(under: m, t == τ) x`: `X` is `x` **as the moving frame sees it**, carried by the
//! motion's pose at the roll `τ` (in degrees, as a meshing's phase is) — `X = Pose(τ)·x`, three
//! rows (R4).  Under `motion(turn, …)` at `τ = −φ` it is a point of the fixed frame seen from a
//! member turned `φ`: a follower's contact seen by its cam, a rolling circle's point seen by the
//! fixed one.  Any motion the solve reads.
//!
//! **Its rate** (#365) is `rate(X over t)`, the contact's own slot: `∂(Pose(τ)·x)/∂τ`, a length a
//! radian of the roll, every other column held — what the motion's **twist** is at `X`.  A
//! motion's twist has no words of its own: the standard library reads it by rates of carried
//! points (`std.sv`'s `reciprocal`, a line a force along which does no work on the motion, and
//! `Screw`, its instantaneous screw axis and its pitch).  Three readings of the rate:
//!
//! - `rate(X over t) parallel l`: the rate's part across the line's direction, two rows over two
//!   directions square to it — a velocity, so a point that stands still (on the axis of a turn)
//!   holds it regularly;
//! - `rate(X over t) perpendicular l`: its part along the line, one row;
//! - `rate(X over t) distance(d, along: l)`: that part less `d`, one row — `d` stated or free.
//!
//! **Layout.** Columns `[lead…, motion…]`: the two points' lifts and the roll (seven), or a rate's
//! direction (a line's two lifted ends, or an axis's place and direction: six), the carried point's
//! lift and the roll (ten), then two lifted ends per axis the motion turns about or slides along
//! (`held::Moving`), then — a rate's distance written free — its unknown.  Constants: a rate's
//! distance as `(m, c)` (`d = m·free + c`), then the motion's steps (`held::encode_steps`).  Every
//! row is written once over `kernels::Num`, so the Jacobian is exact by forward differentiation.
use super::held::{self, cross, dot, vscale as scale, vsub, Step, STEP};
use crate::kernels::{Kernel, KernelFn, Num};

/// Which statement a motion is read for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reading {
    /// `X coincident(under: m, t == τ) x`: three rows.
    Carried = 0,
    /// `rate(X over t) parallel l`: two rows.
    Rate = 1,
    /// `rate(X over t) perpendicular l`: one row.
    RateSquare = 2,
    /// `rate(X over t) distance(d, along: l)`: one row.
    RateAlong = 3,
}

impl Reading {
    pub const ALL: [Reading; 4] =
        [Reading::Carried, Reading::Rate, Reading::RateSquare, Reading::RateAlong];

    pub fn rows(self) -> usize {
        match self {
            Reading::Carried => 3,
            Reading::Rate => 2,
            Reading::RateSquare | Reading::RateAlong => 1,
        }
    }

    /// The columns before the motion's.
    pub fn lead(self) -> usize {
        match self {
            Reading::Carried => 7,
            _ => 10,
        }
    }

    /// The constants before the motion's steps: a rate's distance, `(m, c)`.
    pub fn header(self) -> usize {
        match self {
            Reading::RateAlong => 2,
            _ => 0,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Reading::Carried => "carried",
            Reading::Rate => "carried_rate",
            Reading::RateSquare => "carried_rate_square",
            Reading::RateAlong => "carried_rate_along",
        }
    }

    /// A point's offset and a rate's components are lengths (a length a radian).
    fn degree(self) -> u32 {
        1
    }
}

/// Two unit directions square to the unit `d` and to one another: the first the world axis `d`
/// leans least toward, `d`'s part taken off.
pub(crate) fn square_to<T: Num>(d: [T; 3]) -> ([T; 3], [T; 3]) {
    let k = (0..3).min_by(|&i, &j| d[i].val().abs().total_cmp(&d[j].val().abs())).unwrap_or(0);
    let e: [T; 3] = std::array::from_fn(|i| T::cst(if i == k { 1.0 } else { 0.0 }));
    let t1 = held::unit(vsub(e, scale(d, d[k])));
    (t1, cross(d, t1))
}

/// A reading's rows over its columns `cols` and constants `k`, into `out`.  `None` where the
/// constants are no motion's steps over this many columns.
pub(crate) fn rows<T: Num>(r: Reading, cols: &[T], k: &[f64], out: &mut [T]) -> Option<()> {
    let (head, k) = (k.get(..r.header())?, k.get(r.header()..)?);
    let steps = held::read_steps(k)?;
    let width = r.lead() + held::steps_width(&steps);
    // a rate's distance written free reads its unknown last
    let free = r == Reading::RateAlong && cols.len() == width + 1;
    if (cols.len() != width && !free) || out.len() != r.rows() {
        return None;
    }
    let m = &cols[r.lead()..width];
    let deg = T::cst(std::f64::consts::PI / 180.0);
    if r == Reading::Carried {
        // `X − Pose(τ)·x`, the roll in degrees
        let (x, at, t) = (held::col3(cols, 0), held::col3(cols, 3), cols[6]);
        let (moved, _) = held::carried(&steps, m, t * deg, at)?;
        for i in 0..3 {
            out[i] = x[i] - moved[i];
        }
        return Some(());
    }
    // the rate a radian of the roll, against the line's direction
    let d = held::unit(vsub(held::col3(cols, 3), held::col3(cols, 0)));
    let (at, t) = (held::col3(cols, 6), cols[9]);
    let (_, rate) = held::carried(&steps, m, t * deg, at)?;
    match r {
        Reading::Rate => {
            let (e1, e2) = square_to(d);
            out[0] = dot(rate, e1);
            out[1] = dot(rate, e2);
        }
        Reading::RateSquare => out[0] = dot(rate, d),
        Reading::RateAlong => {
            let stated = match free {
                true => T::cst(head[0]) * cols[width] + T::cst(head[1]),
                false => T::cst(head[1]),
            };
            out[0] = dot(rate, d) - stated;
        }
        Reading::Carried => unreachable!("read above"),
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
    let (res, jac): (KernelFn, KernelFn) = match r {
        Reading::Carried => (res::<0>, jac::<0>),
        Reading::Rate => (res::<1>, jac::<1>),
        Reading::RateSquare => (res::<2>, jac::<2>),
        Reading::RateAlong => (res::<3>, jac::<3>),
    };
    Kernel { name: r.name(), n_res: r.rows(), n_par, n_const, degree: r.degree(), res, jac,
             const_jac: None, form: None }
}

/// The constants a reading of a motion with these steps takes, a rate's distance `(m, c)` first.
pub fn consts(r: Reading, stated: [f64; 2], steps: &[Step]) -> Vec<f64> {
    let mut k = stated[..r.header()].to_vec();
    k.extend(held::encode_steps(steps));
    k
}

/// How many constants a reading of `steps` takes.
pub fn n_const(r: Reading, steps: &[Step]) -> usize {
    r.header() + STEP * steps.len()
}
