//! **A point a motion carries** (#326, R4).
//!
//! `X coincident(under: m, t == τ) x`: `X` is `x` **as the moving frame sees it**, carried by the
//! motion's pose at the roll `τ` (in degrees, as a meshing's phase is) — `X = Pose(τ)·x`, three
//! rows.  Under `motion(turn, …)` at `τ = −φ` it is a point of the fixed frame seen from a member
//! turned `φ`: a follower's contact seen by its cam, a rolling circle's point seen by the fixed
//! one.  Any motion the solve reads.
//!
//! **Its rate** (#365), `rate(X over t)` over the contact's own slot, is `∂(Pose(τ)·x)/∂τ`, a
//! length a radian of the roll, every other column held — what the motion's **twist** is at `X`,
//! read by the one kernel every rate of a contact is (`contact_rate`, `held::carried`).  A motion's
//! twist has no words of its own: the standard library reads it by rates of carried points
//! (`std.sv`'s `reciprocal`, a line a force along which does no work on the motion, and `Screw`,
//! its instantaneous screw axis and its pitch).
//!
//! **At a stated roll** (#366), `X coincident x at(under: m) τ`: the same rows, `τ` a dimension
//! the statement states (or a free variable's affine map), so a face family's station may be
//! carried to a roll written over its running number and read again at every station — the
//! station words of `std.sv` are motions read so, each at its frame's value (`n := base in
//! screw`, #391 S3).  Drawn in a plane, `X` is held where the plane
//! sees the carried point: the two rows along the plane's axes.
//!
//! **Under a drive** (#398 G), `X coincident x at(under: f) τ`, `f`'s value written over the
//! declared parameter the roll stands for (`fix(value == 5deg * (1 - cos(t))) f`): the driven
//! joint stands at its law's value of `τ`, the law's tape carried in the constants as a profile in
//! the plane carries one (`generate::Drive`) and read in Taylor arithmetic (`held::carried`), so
//! the rows, their Jacobian and the rate are exact through the law — an owned roll's column
//! reads the law's rate.  A law reads stated numbers alone (`motion::Law::held`).
//!
//! **Layout.** Columns `[X's lift, x's lift, τ, motion…]`, then two lifted ends per axis the motion
//! turns about or slides along (`held::Moving`).  Constants: the motion's steps and its laws
//! (`held::encode_motion`).  At a stated roll, no `τ` column: `[X's lift, x's lift, (X's plane's
//! û, v̂), motion…, (a)]`, constants `[free, m, c, motion…]`, the roll `m·a + c` radians where a
//! free variable `a` is the last column and `c` alone otherwise.  The rows are written once over
//! `kernels::Num`, so the Jacobian is exact by forward differentiation.
use super::held::{self, cross, vscale as scale, vsub};
use crate::kernels::{Kernel, KernelFn, Num};

/// Which statement a motion is read for: a point it carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reading {
    /// `X coincident(under: m, t == τ) x`: three rows.
    Carried = 0,
    /// `X coincident x at(under: m) τ`, `X` in space: three rows.
    At = 1,
    /// The same, `X` drawn in a plane: two rows, along the plane's axes.
    AtIn = 2,
}

impl Reading {
    pub const ALL: [Reading; 3] = [Reading::Carried, Reading::At, Reading::AtIn];

    pub fn rows(self) -> usize {
        match self {
            Reading::Carried | Reading::At => 3,
            Reading::AtIn => 2,
        }
    }

    /// The columns before the motion's.
    pub fn lead(self) -> usize {
        match self {
            Reading::Carried => 7,
            Reading::At => 6,
            Reading::AtIn => 12,
        }
    }

    /// The constants before the motion's steps: a stated roll's `[free, m, c]`.
    pub fn lead_consts(self) -> usize {
        match self {
            Reading::Carried => 0,
            Reading::At | Reading::AtIn => 3,
        }
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

/// A reading's rows over its columns `cols` and the motion's steps `k`, into `out`.  `None` where
/// the constants are no motion's steps over this many columns.
pub(crate) fn rows<T: Num>(r: Reading, cols: &[T], k: &[f64], out: &mut [T]) -> Option<()> {
    let lead = r.lead_consts();
    let (steps, laws) = held::read_motion(k.get(lead..)?)?;
    let width = held::steps_width(&steps);
    // a stated roll's free variable is the last column
    let free = lead > 0 && k[0] != 0.0;
    if cols.len() != r.lead() + width + usize::from(free) || out.len() != r.rows() {
        return None;
    }
    let (x, at) = (held::col3(cols, 0), held::col3(cols, 3));
    // the roll in radians: the relation's own unknown in degrees, or the number stated
    let t = match r {
        Reading::Carried => cols[6] * T::cst(std::f64::consts::PI / 180.0),
        Reading::At | Reading::AtIn if free => cols[r.lead() + width] * T::cst(k[1]) + T::cst(k[2]),
        Reading::At | Reading::AtIn => T::cst(k[2]),
    };
    let (moved, _) = held::carried(&steps, &laws, &cols[r.lead()..r.lead() + width], t, at)?;
    // `X − Pose(τ)·x`, whole in space or along the axes of the plane `X` is drawn in
    let d: [T; 3] = std::array::from_fn(|i| x[i] - moved[i]);
    match r {
        Reading::Carried | Reading::At => out.copy_from_slice(&d),
        Reading::AtIn => {
            let (u, v) = (held::col3(cols, 6), held::col3(cols, 9));
            out[0] = d[0] * u[0] + d[1] * u[1] + d[2] * u[2];
            out[1] = d[0] * v[0] + d[1] * v[1] + d[2] * v[2];
        }
    }
    Some(())
}

fn res_of<const R: usize>(n: usize, v: &[f64], k: &[f64], out: &mut [f64]) {
    let r = Reading::ALL[R];
    held::block_res(n, v, k, out, r.rows(), |x, kc, o| rows(r, x, kc, o));
}

fn jac_of<const R: usize>(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let r = Reading::ALL[R];
    held::block_jac(n, v, k, j, r.rows(), |x, kc, o| rows(r, x, kc, o));
}

/// The kernel of a reading at a block's widths (`system::build_kernel`).
pub fn kernel(r: Reading, n_par: usize, n_const: usize) -> Kernel {
    let (res, jac): (KernelFn, KernelFn) = match r {
        Reading::Carried => (res_of::<0>, jac_of::<0>),
        Reading::At => (res_of::<1>, jac_of::<1>),
        Reading::AtIn => (res_of::<2>, jac_of::<2>),
    };
    let name = match r {
        Reading::Carried => "carried",
        Reading::At => "carried_at",
        Reading::AtIn => "carried_at_in",
    };
    Kernel { name, n_res: r.rows(), n_par, n_const, degree: 1, res, jac, const_jac: None,
             form: None }
}

/// The constants a reading of a motion takes: a stated roll's `[free, m, c]` (`free` 0 or 1, the
/// roll `m·a + c` radians over a free variable `a`, `c` where there is none), then the motion as
/// `held::encode_motion` writes it.
pub fn consts(r: Reading, roll: [f64; 3], motion: &[f64]) -> Vec<f64> {
    [&roll[..r.lead_consts()], motion].concat()
}
