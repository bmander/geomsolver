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
//! **Layout.** Columns `[X's lift, x's lift, τ, motion…]`, then two lifted ends per axis the motion
//! turns about or slides along (`held::Moving`).  Constants: the motion's steps
//! (`held::encode_steps`).  The rows are written once over `kernels::Num`, so the Jacobian is exact
//! by forward differentiation.
use super::held::{self, cross, vscale as scale, vsub, Step, STEP};
use crate::kernels::{Kernel, KernelFn, Num};

/// Which statement a motion is read for: a point it carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reading {
    /// `X coincident(under: m, t == τ) x`: three rows.
    Carried = 0,
}

impl Reading {
    pub const ALL: [Reading; 1] = [Reading::Carried];

    pub fn rows(self) -> usize {
        3
    }

    /// The columns before the motion's.
    pub fn lead(self) -> usize {
        7
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
    let steps = held::read_steps(k)?;
    if cols.len() != r.lead() + held::steps_width(&steps) || out.len() != r.rows() {
        return None;
    }
    // `X − Pose(τ)·x`, the roll in degrees
    let (x, at, t) = (held::col3(cols, 0), held::col3(cols, 3), cols[6]);
    let deg = T::cst(std::f64::consts::PI / 180.0);
    let (moved, _) = held::carried(&steps, &cols[r.lead()..], t * deg, at)?;
    for i in 0..3 {
        out[i] = x[i] - moved[i];
    }
    Some(())
}

fn res(n: usize, v: &[f64], k: &[f64], out: &mut [f64]) {
    let r = Reading::Carried;
    held::block_res(n, v, k, out, r.rows(), |x, kc, o| rows(r, x, kc, o));
}

fn jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let r = Reading::Carried;
    held::block_jac(n, v, k, j, r.rows(), |x, kc, o| rows(r, x, kc, o));
}

/// The kernel of a reading at a block's widths (`system::build_kernel`).
pub fn kernel(_: Reading, n_par: usize, n_const: usize) -> Kernel {
    Kernel { name: "carried", n_res: 3, n_par, n_const, degree: 1, res: res as KernelFn,
             jac: jac as KernelFn, const_jac: None, form: None }
}

/// The constants a reading of a motion with these steps takes.
pub fn consts(steps: &[Step]) -> Vec<f64> {
    held::encode_steps(steps)
}

/// How many constants a reading of `steps` takes.
pub fn n_const(steps: &[Step]) -> usize {
    STEP * steps.len()
}
