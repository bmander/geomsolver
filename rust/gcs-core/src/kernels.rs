//! Residual / Jacobian kernels — one per constraint type, evaluated for a whole block of
//! same-typed constraints per call.
//!
//! `v` is (n * n_par) local parameter values, `k` is (n * n_const) constants, `r` is (n * n_res)
//! residuals and `j` is (n * n_res * n_par).  Column conventions match the `params` tuples the
//! model builds; see the comment above each kernel.  Residual forms follow the program: squared
//! distances (no sqrt), a determinant for parallel, a wrapped atan2 gap for the directed angle,
//! signed distance minus radius for tangency.
//!
//! The order of `KERNELS` **is** the kernel id and is part of the plan ABI.

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::taylor::Jet;
/// Kernel ids, in registration order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum K {
    Coincident = 0,
    Distance,
    Midpoint,
    Drag,
    Horizontal,
    Vertical,
    Parallel,
    Perpendicular,
    Angle,
    EqualLength,
    PointOnLine,
    PointOnCircle,
    Radius,
    EqualRadius,
    TangentLineCircle,
    TangentCircleCircle,
    TangentArcLine,
    Symmetric,
    ParallelDistance,
    PointLineDistance,
    AnnularDistance,
    PointOnSpline,
    SplineTangentLine,
    SplineCurvature,
    // an ordinate along a view's own axes, between two points drawn in it: the run and the rise
    OrdinateU,
    OrdinateV,
    // the same dimensions again, with the number they state left to the solver
    DistanceFree,
    AngleFree,
    RadiusFree,
    ParallelDistanceFree,
    PointLineDistanceFree,
    AnnularDistanceFree,
    OrdinateUFree,
    OrdinateVFree,
    Project,
    // and the magnitude forms of the two distances measured *from a line*, which a statement
    // that names no side compiles to: both sides are solutions and the seed picks (issue #48,
    // item 4).  Each with its own free twin, since a magnitude may state a shared unknown too.
    PointLineMagnitude,
    PointLineMagnitudeFree,
    ParallelMagnitude,
    ParallelMagnitudeFree,
    // a hidden point held at the lift of the point it stands for, drawn in a plane
    Lift,
    // the relations in space, over the hidden points drawn points lift to: each dimension with
    // its free twin beside it, and the two that read a plane's axes and origin
    Coincident3,
    Distance3,
    Distance3Free,
    PointLine3,
    PointLine3Free,
    LineLine3,
    LineLine3Free,
    Angle3,
    Angle3Free,
    Perpendicular3,
    Parallel3,
    PointOnPlane,
    PointOnCircle3,
    // a projection between two images either of whose planes moves in the solve
    ProjectSolved,
    // a point on a line in space, true lengths equal, and an ordinate along a line drawn in the
    // points' own view, and free
    PointOnLine3,
    EqualLength3,
    OrdinateLine,
    OrdinateLineFree,
    LineOnPlane,
    // the midpoint and the mirror in a line, in space
    Midpoint3,
    Symmetric3,
    // two directed angles equal, and an arc's length along itself stated and free
    EqualAngle,
    ArcLength,
    ArcLengthFree,
    // an axis's own two rows, and a point on one
    AxisUnit,
    AxisFoot,
    PointOnAxis,
    // an ordinate in space along an axis or a line, and from a plane's origin along its frame's
    // `û`, `v̂` and normal, each free too
    OrdinateSpace,
    OrdinateFrameU,
    OrdinateSpaceFree,
    OrdinateFrameUFree,
    OrdinateFrameV,
    OrdinateFrameVFree,
    OrdinateFrameN,
    OrdinateFrameNFree,
    // an axis in a plane, square to its normal and along it, and two planes apart
    AxisOnPlane,
    AxisParallelPlane,
    AxisPerpendicularPlane,
    PlaneDistance,
    PlaneDistanceFree,
    // two axes on one line, either way
    AxisCoincident,
    // two planes facing alike, either way
    PlaneParallel,
    // the soft drag target of a point in space: seen by the eye where the pointer is
    DragSeen,
    // a line lying on an axis
    LineOnAxis,
}

pub const N_KERNELS: usize = 84;

#[derive(Clone, Copy)]
pub struct Kernel {
    pub name: &'static str,
    pub n_res: usize,
    pub n_par: usize,
    pub n_const: usize,
    /// Power of length the residual carries: 1 for the ones written as a signed distance
    /// (coincident, radius, the distance-to-a-line family), 2 for the ones written squared
    /// (distance, the dot/cross forms, tangency).  A residual of 1e-6 means something quite
    /// different in the two, so the tolerance a row is judged against is `tol * extent^degree`
    /// rather than one threshold for the whole system.
    pub degree: u32,
    pub res: fn(n: usize, v: &[f64], k: &[f64], r: &mut [f64]),
    pub jac: fn(n: usize, v: &[f64], k: &[f64], j: &mut [f64]),
    /// n_res*n_par entries when the Jacobian is instance-independent.
    pub const_jac: Option<&'static [f64]>,
}

impl Kernel {
    /// One instance's Jacobian at `v` into `j` (`n_res × n_par`, row-major): the constant one
    /// where the kernel has it, else computed.
    pub fn jac_into(&self, v: &[f64], k: &[f64], j: &mut [f64]) {
        match self.const_jac {
            Some(cj) => j.copy_from_slice(cj),
            None => (self.jac)(1, v, k, j),
        }
    }
}

pub fn kernel(id: K) -> &'static Kernel {
    &KERNELS[id as usize]
}

pub fn kernel_by_id(id: usize) -> &'static Kernel {
    &KERNELS[id]
}

/// The kernel for one curve definition: `point_on_curve` at that definition's widths.
///
/// Not in `KERNELS` because there is no fixed number of them — a document defines as many curve
/// families as it likes.  `System` builds its own table of the static ones plus one of these per
/// definition, which is what keeps a block's columns a fixed width while letting two different
/// curves have different ones.
pub fn curve_kernel(n_theta: usize, n_const: usize) -> Kernel {
    Kernel {
        name: "point_on_curve",
        n_res: 2,
        n_par: 3 + n_theta,
        n_const,
        degree: 1,
        res: point_on_curve_res,
        jac: point_on_curve_jac,
        const_jac: None,
    }
}

/// **A row's derivative** (§6.21): the twin of static kernel `inner` that a set's body row is
/// stated as for `l tangent S` — `J(x)·ẋ`, the row's rate as the use's geometry moves along the
/// set, where `ẋ` is, column by column, a component of the line's direction `b − a` (the
/// contact's place in space), a **tangent unknown** of its own (geometry the use made privately,
/// or a contact whose direction is solved for), or 0 (a number the set was given, held).  The
/// columns are the row's, then a tangent column for each (the fixed zero where it has none), then
/// the line's ends in space; the constants the row's kernel, its constants, and how each column
/// moves (`Constraint::consts_on`): 0 held, 1 to 3 a component of the direction, `TANGENT` its
/// tangent column.
///
/// Of the same degree as the row: a Jacobian of degree `g − 1` times a length.  Every number is
/// read from the row's Taylor form (`taylor::residual`), along `x + ẋε`: the residual is the first
/// coefficient, and its derivative in `x` the Hessian along `ẋ`, `ẋᵀH e_c`, by polarisation of
/// the second, `[c₂(ẋ + s e_c) − c₂(ẋ − s e_c)] / 2s` — exact, no step.  In `ẋ` and the line's
/// ends it is the row's own Jacobian, routed by how each column moves.
pub fn dual_kernel(inner: usize) -> Kernel {
    let kn = &KERNELS[inner];
    Kernel {
        name: "dual",
        n_res: kn.n_res,
        n_par: 2 * kn.n_par + 6,
        n_const: 1 + kn.n_const + kn.n_par,
        degree: kn.degree,
        res: dual_row_res,
        jac: dual_row_jac,
        const_jac: None,
    }
}

/// How a dual row's column moves that has a tangent column of its own (`dual_kernel`).
pub const TANGENT: usize = 4;

/// One derivative row's parts: the row's kernel id and kernel, its columns, its tangent columns,
/// its constants, how each column moves, and the line's direction.
struct DualRow<'a> {
    kid: usize,
    kn: &'static Kernel,
    x: &'a [f64],
    t: &'a [f64],
    kc: &'a [f64],
    mask: &'a [f64],
    dir: [f64; 3],
}

impl<'a> DualRow<'a> {
    fn read(v: &'a [f64], k: &'a [f64]) -> Option<DualRow<'a>> {
        let kid = *k.first()? as usize;
        let kn = KERNELS.get(kid)?;
        let m = kn.n_par;
        if v.len() != 2 * m + 6 || k.len() != 1 + kn.n_const + m {
            return None;
        }
        let (a, b) = (&v[2 * m..2 * m + 3], &v[2 * m + 3..2 * m + 6]);
        Some(DualRow {
            kid,
            kn,
            x: &v[..m],
            t: &v[m..2 * m],
            kc: &k[1..1 + kn.n_const],
            mask: &k[1 + kn.n_const..],
            dir: [b[0] - a[0], b[1] - a[1], b[2] - a[2]],
        })
    }

    /// `ẋ`, column by column, into `out`.
    fn xdot(&self, out: &mut Vec<f64>) {
        out.clear();
        out.extend((0..self.x.len()).map(|c| match self.mask[c] as usize {
            d @ 1..=3 => self.dir[d - 1],
            TANGENT => self.t[c],
            _ => 0.0,
        }));
    }

    /// The row's Taylor coefficients along `x + wε`, into `r`; `false` where it has no form.
    /// `path` is scratch, its constant terms the row's columns.
    fn along(&self, w: &[f64], path: &mut Vec<Jet>, r: &mut [Jet], jrow: &mut Vec<f64>) -> bool {
        path.clear();
        path.extend(self.x.iter().zip(w).map(|(&x, &d)| Jet::from(&[x, d])));
        crate::taylor::residual(self.kid, path, self.kc, r, jrow)
    }
}

fn dual_row_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    let (mut jets, mut jrow, mut path, mut xd) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for i in 0..n {
        let Some(d) = DualRow::read(&v[n_par * i..n_par * (i + 1)], &k[n_const * i..n_const * (i + 1)])
        else {
            continue;
        };
        let nr = d.kn.n_res;
        jets.resize(nr, Jet::default());
        d.xdot(&mut xd);
        let ok = d.along(&xd, &mut path, &mut jets, &mut jrow);
        for row in 0..nr {
            r[nr * i + row] = if ok { jets[row].0[1] } else { f64::NAN };
        }
    }
}

fn dual_row_jac(n: usize, v: &[f64], k: &[f64], out: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    let (mut j, mut jrow, mut path, mut w) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let (mut plus, mut minus) = (Vec::new(), Vec::new());
    for i in 0..n {
        let Some(d) = DualRow::read(&v[n_par * i..n_par * (i + 1)], &k[n_const * i..n_const * (i + 1)])
        else {
            continue;
        };
        let (m, nr) = (d.kn.n_par, d.kn.n_res);
        let o = nr * n_par * i;
        out[o..o + nr * n_par].iter_mut().for_each(|e| *e = 0.0);
        j.resize(nr * m, 0.0);
        d.kn.jac_into(d.x, d.kc, &mut j);
        // over the tangent columns and the line's ends: the row's Jacobian where each moves one
        for row in 0..nr {
            for c in 0..m {
                let g = j[row * m + c];
                match d.mask[c] as usize {
                    comp @ 1..=3 => {
                        out[o + row * n_par + 2 * m + 3 + comp - 1] += g;
                        out[o + row * n_par + 2 * m + comp - 1] -= g;
                    }
                    TANGENT => out[o + row * n_par + m + c] = g,
                    _ => {}
                }
            }
        }
        // over the row's columns: the Hessian along `ẋ`, by polarisation of the second order —
        // none where the row is affine
        if crate::taylor::is_affine(d.kid) {
            continue;
        }
        d.xdot(&mut w);
        let s = w.iter().map(|x| x * x).sum::<f64>().sqrt().max(1.0);
        plus.resize(nr, Jet::default());
        minus.resize(nr, Jet::default());
        for c in 0..m {
            let at = w[c];
            w[c] = at + s;
            let ok = d.along(&w, &mut path, &mut plus, &mut jrow);
            w[c] = at - s;
            let ok = ok && d.along(&w, &mut path, &mut minus, &mut jrow);
            w[c] = at;
            for row in 0..nr {
                out[o + row * n_par + c] =
                    if ok { (plus[row].0[2] - minus[row].0[2]) / (2.0 * s) } else { f64::NAN };
            }
        }
    }
}

/// The kernel for one *trace* definition: the same contact — `p − C(u) = 0`, the same columns —
/// with `C` and its derivatives found by solving the family's block (`locus::eval_flat`) rather
/// than by running a pair of tapes.  The whole compiled block rides in the constants, so this is
/// still a plain `fn` and `System` still knows nothing about curves.
pub fn trace_kernel(n_theta: usize, n_const: usize) -> Kernel {
    Kernel {
        name: "point_on_trace",
        n_res: 2,
        n_par: 3 + n_theta,
        n_const,
        degree: 1,
        res: point_on_body_res::<TRACE>,
        jac: point_on_body_jac::<TRACE>,
        const_jac: None,
    }
}

/// A point on a profile generated by a motion (`generate.rs`): `(px, py, t, θ…)`, the rows of
/// `trace_kernel` with `C` and its gradient from the generated profile.  One per definition.
pub fn envelope_kernel(n_theta: usize, n_const: usize) -> Kernel {
    Kernel {
        name: "point_on_envelope",
        n_res: 2,
        n_par: 3 + n_theta,
        n_const,
        degree: 1,
        res: point_on_body_res::<ENVELOPE>,
        jac: point_on_body_jac::<ENVELOPE>,
        const_jac: None,
    }
}

/// The view's numbers a point on an extrusion reads before its contact's (`extrusion_kernel`):
/// its plane's basis `(u, v, o)`.
pub const EXTRUSION_FRAME: usize = 9;

/// A point in space on the surface a curve of a view stands for (`CKind::PointOnExtrusion`,
/// issue #70): `(x, y, z, t, θ…)`, the point's lift, the curve's parameter and its columns, over
/// the constants `EXTRUSION_FRAME` and then the contact's.  The point read into its plane's
/// coordinates, less `C(t)`: `point_on_envelope`'s rows with the point's place in the view for
/// its coordinates.  Only a generated profile stands for a surface, so any other body is
/// `refused`.  One per definition.
pub fn extrusion_kernel(n_theta: usize, n_const: usize, body: u8) -> Kernel {
    let (res, jac): (KernelFn, KernelFn) = match body {
        ENVELOPE => (point_on_extrusion_res, point_on_extrusion_jac),
        _ => (refused_res, refused_jac),
    };
    Kernel {
        name: "point_on_extrusion",
        n_res: 2,
        n_par: 4 + n_theta,
        n_const: EXTRUSION_FRAME + n_const,
        degree: 1,
        res,
        jac,
        const_jac: None,
    }
}

/// A line tangent to a curve written in the language: `(u, θ…, ax, ay, bx, by)` — the same two
/// rows as `spline_tangent_line`, with `C` and `C'` from the definition's tapes (a formula) or
/// its block (a trace).  One kernel per definition, as the contact's is.
pub fn curve_tangent_kernel(n_theta: usize, n_const: usize, body: u8) -> Kernel {
    let (name, res, jac): (&'static str, KernelFn, KernelFn) = match body {
        TRACE => ("trace_tangent_line", curve_tangent_res::<TRACE>, curve_tangent_jac::<TRACE>),
        ENVELOPE => ("envelope_tangent_line", curve_tangent_res::<ENVELOPE>, curve_tangent_jac::<ENVELOPE>),
        _ => ("curve_tangent_line", curve_tangent_res::<FORMULA>, curve_tangent_jac::<FORMULA>),
    };
    Kernel { name, n_res: 2, n_par: 1 + n_theta + 4, n_const, degree: 1, res, jac, const_jac: None }
}

/// A kernel's residual or Jacobian, as the table holds it.
type KernelFn = fn(usize, &[f64], &[f64], &mut [f64]);

/// A circle osculating a curve written in the language: `(u, θ…, cx, cy, r)` — the three rows
/// of `spline_curvature`, with `C`, `C'`, `C''` and `C'''` from the definition's tapes, or from
/// its block's Taylor orders (`locus::higher_orders`).  A trace whose block has a row with no
/// Taylor form has no second derivative to give, so its slot is `refused`.
pub fn curve_curvature_kernel(n_theta: usize, n_const: usize, body: u8, formed: bool) -> Kernel {
    let (name, res, jac): (&'static str, KernelFn, KernelFn) = match body {
        TRACE => ("trace_curvature", curve_curvature_res::<TRACE>, curve_curvature_jac::<TRACE>),
        ENVELOPE => ("envelope_curvature", curve_curvature_res::<ENVELOPE>, curve_curvature_jac::<ENVELOPE>),
        _ => ("curve_curvature", curve_curvature_res::<FORMULA>, curve_curvature_jac::<FORMULA>),
    };
    let (res, jac): (KernelFn, KernelFn) = if formed { (res, jac) } else { (refused_res, refused_jac) };
    Kernel {
        name,
        n_res: 3,
        n_par: 1 + n_theta + 3,
        n_const,
        degree: 1,
        res,
        jac,
        const_jac: None,
    }
}

/* -- linear kernels: r = J v with a constant J ----------------------------- */

fn lin_res(n: usize, v: &[f64], j: &'static [f64], n_res: usize, n_par: usize, r: &mut [f64]) {
    for i in 0..n {
        let vv = &v[i * n_par..(i + 1) * n_par];
        for t in 0..n_res {
            let mut s = 0.0;
            for c in 0..n_par {
                s += j[t * n_par + c] * vv[c];
            }
            r[i * n_res + t] = s;
        }
    }
}

fn lin_jac(n: usize, j: &'static [f64], out: &mut [f64]) {
    let sz = j.len();
    for i in 0..n {
        out[i * sz..(i + 1) * sz].copy_from_slice(j);
    }
}

/// A line's length, floored.  A line whose endpoints have collapsed has no direction; dividing
/// by its length would put a NaN in the residual vector, and a NaN reads as "no error" at every
/// max we take downstream — the sketch would be reported solved on iteration zero.  The floor
/// keeps the residual finite (a degenerate line is at distance 0 from everything, so the error is
/// the whole target) and the Jacobian large, which is what pushes the endpoints back apart.
pub const MIN_LINE_LEN: f64 = 1e-12;

fn line_len(dx: f64, dy: f64) -> f64 {
    dx.dhypot(dy).max(MIN_LINE_LEN)
}

/// Jacobian of C/L from dC and dL — the quotient rule, shared by the signed distance-to-a-line
/// kernels.
fn ratio_jac(dc: &[f64], dl: &[f64], l: f64, c: f64, j: &mut [f64]) {
    let f = c / (l * l);
    for t in 0..dc.len() {
        j[t] = dc[t] / l - f * dl[t];
    }
}

/// `r = J v` with a compile-time constant J: the residual, the Jacobian and the shapes all
/// derive from J, exactly as in the reference `linear_kernel`.
macro_rules! linear_kernel {
    ($name:ident, $nres:expr, $npar:expr, [$($x:expr),*]) => {
        pub mod $name {
            pub const J: &[f64] = &[$($x as f64),*];
            pub fn res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
                super::lin_res(n, v, J, $nres, $npar, r)
            }
            pub fn jac(n: usize, _v: &[f64], _k: &[f64], j: &mut [f64]) {
                super::lin_jac(n, J, j)
            }
        }
    };
}

// (px,py,qx,qy)
linear_kernel!(coincident, 2, 4, [1, 0, -1, 0, 0, 1, 0, -1]);
// (px,py,ax,ay,bx,by)
linear_kernel!(midpoint, 2, 6, [2, 0, -1, 0, -1, 0, 0, 2, 0, -1, 0, -1]);
// (ax,ay,bx,by): ay - by
linear_kernel!(horizontal, 1, 4, [0, 1, 0, -1]);
// (ax,ay,bx,by): ax - bx
linear_kernel!(vertical, 1, 4, [1, 0, -1, 0]);
// (r1,r2)
linear_kernel!(equal_radius, 1, 2, [1, -1]);

/* -- point / point --------------------------------------------------------- */

/* The geometry each dimension measures, written once for the two forms that measure it: the
 * number it is compared against may be stated (a constant) or free (a column), and that is the
 * only difference between a kernel and its free twin — see `expr::Free`. */

/// |p-q|², from (px,py,qx,qy).
#[inline]
fn dist_sq(v: &[f64]) -> f64 {
    let (dx, dy) = (v[0] - v[2], v[1] - v[3]);
    dx * dx + dy * dy
}

/// Its gradient in those four columns.
#[inline]
fn dist_sq_jac(v: &[f64], j: &mut [f64]) {
    let dx = 2.0 * (v[0] - v[2]);
    let dy = 2.0 * (v[1] - v[3]);
    j[0] = dx;
    j[1] = dy;
    j[2] = -dx;
    j[3] = -dy;
}

/// (px,py,qx,qy), K = (d): |p-q|² - d²
fn distance_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        r[i] = dist_sq(&v[4 * i..]) - k[i] * k[i];
    }
}

fn distance_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 4 * i;
        dist_sq_jac(&v[o..], &mut j[o..o + 4]);
    }
}

/// (px,py,qx,qy), K = (d): the run from p to q across the page, signed — (qx - px) - d.
/// Nothing is squared and nothing is divided, so the Jacobian below is a constant: this is the
/// best-conditioned row in the system, and it stays that way with the two points one directly
/// above the other, which is exactly the pose someone reaches for a horizontal dimension in.
/// The sign is the price: negating `d` moves the second point across, as it does for the other
/// dimensions written as a signed distance.
fn ordinate_u_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 4 * i;
        r[i] = v[o + 2] - v[o] - k[i];
    }
}

/// And the rise: (qy - py) - d.
fn ordinate_v_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 4 * i;
        r[i] = v[o + 3] - v[o + 1] - k[i];
    }
}

static ORDINATE_U_J: &[f64] = &[-1.0, 0.0, 1.0, 0.0];
static ORDINATE_V_J: &[f64] = &[0.0, -1.0, 0.0, 1.0];

fn ordinate_u_jac(n: usize, _v: &[f64], _k: &[f64], j: &mut [f64]) {
    lin_jac(n, ORDINATE_U_J, j)
}

fn ordinate_v_jac(n: usize, _v: &[f64], _k: &[f64], j: &mut [f64]) {
    lin_jac(n, ORDINATE_V_J, j)
}

/// (px,py), K = (tx,ty,w): the soft drag target
fn drag_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let (o, ko) = (2 * i, 3 * i);
        r[2 * i] = k[ko + 2] * (v[o] - k[ko]);
        r[2 * i + 1] = k[ko + 2] * (v[o + 1] - k[ko + 1]);
    }
}

fn drag_jac(n: usize, _v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let w = k[3 * i + 2];
        let o = 4 * i;
        j[o] = w;
        j[o + 1] = 0.0;
        j[o + 2] = 0.0;
        j[o + 3] = w;
    }
}

/// (x,y,z), K = (tx,ty,w, right, up): the soft drag target of a point in space, which the eye sees
/// at (right·X, up·X).  Two rows, so the depth along the eye's line of sight is left to whatever
/// else holds the point: a free one keeps it (the step is minimum-norm), one on a line slides.
fn drag_seen_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let (x, k) = (&v[3 * i..3 * i + 3], &k[9 * i..9 * i + 9]);
        let (right, up) = (&k[3..6], &k[6..9]);
        let dot = |d: &[f64]| d[0] * x[0] + d[1] * x[1] + d[2] * x[2];
        r[2 * i] = k[2] * (dot(right) - k[0]);
        r[2 * i + 1] = k[2] * (dot(up) - k[1]);
    }
}

fn drag_seen_jac(n: usize, _v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let k = &k[9 * i..9 * i + 9];
        for c in 0..3 {
            j[6 * i + c] = k[2] * k[3 + c];
            j[6 * i + 3 + c] = k[2] * k[6 + c];
        }
    }
}

/* -- line orientation ------------------------------------------------------ */
/* (a1x,a1y,b1x,b1y,a2x,a2y,b2x,b2y) */

#[inline]
fn dirs(v: &[f64]) -> (f64, f64, f64, f64) {
    (v[2] - v[0], v[3] - v[1], v[6] - v[4], v[7] - v[5])
}

fn cross_jac(v: &[f64], j: &mut [f64]) {
    let (d1x, d1y, d2x, d2y) = dirs(v);
    j[0] = -d2y;
    j[1] = d2x;
    j[2] = d2y;
    j[3] = -d2x;
    j[4] = d1y;
    j[5] = -d1x;
    j[6] = -d1y;
    j[7] = d1x;
}

fn dot_jac(v: &[f64], j: &mut [f64]) {
    let (d1x, d1y, d2x, d2y) = dirs(v);
    j[0] = -d2x;
    j[1] = -d2y;
    j[2] = d2x;
    j[3] = d2y;
    j[4] = -d1x;
    j[5] = -d1y;
    j[6] = d1x;
    j[7] = d1y;
}

fn parallel_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let (d1x, d1y, d2x, d2y) = dirs(&v[8 * i..8 * i + 8]);
        r[i] = d1x * d2y - d1y * d2x;
    }
}

fn parallel_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let mut tmp = [0.0f64; 8];
        cross_jac(&v[8 * i..8 * i + 8], &mut tmp);
        j[8 * i..8 * i + 8].copy_from_slice(&tmp);
    }
}

fn perpendicular_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let (d1x, d1y, d2x, d2y) = dirs(&v[8 * i..8 * i + 8]);
        r[i] = d1x * d2x + d1y * d2y;
    }
}

fn perpendicular_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let mut tmp = [0.0f64; 8];
        dot_jac(&v[8 * i..8 * i + 8], &mut tmp);
        j[8 * i..8 * i + 8].copy_from_slice(&tmp);
    }
}

/// The dot and the cross of the two directions — the two components an angle is read from.
#[inline]
fn dot_cross(v: &[f64]) -> (f64, f64) {
    let (d1x, d1y, d2x, d2y) = dirs(v);
    (d1x * d2x + d1y * d2y, d1x * d2y - d1y * d2x)
}

/// An angle difference brought back to within half a turn: the gap between two bearings, with no
/// lap in it.
///
/// Subtracting the nearest whole turn, rather than taking a remainder and folding it: `rem_euclid`
/// is `fmod`, whose argument reduction costs more the further out the angle is, and a stated
/// dimension is under no obligation to be within one lap of the pose (`u + phase` over a cycle of
/// teeth is several).  This is flat at any magnitude.
///
/// At exactly half a turn `round` goes away from zero, so this is odd there — `+π ↦ −π` and
/// `−π ↦ +π` — where folding a remainder would answer `+π` to both.  It is the one input whose
/// sign is not determined by the arithmetic that produced it, and it is the pose farthest from
/// the solution: the magnitude is π either way, both directions out of it are equally good, and
/// nothing downstream reads the sign for anything but which way to turn first.  Worth knowing
/// before assuming this and a placement wrap (`callout::wrap`) may be exchanged: they agree
/// everywhere else and disagree here.
#[inline]
fn wrap_turn(a: f64) -> f64 {
    const TURN: f64 = 2.0 * std::f64::consts::PI;
    a - TURN * (a * (1.0 / TURN)).round()
}

/// `wrap(atan2(cross, dot) − θ)`: zero exactly when the angle from l1's direction to l2's is θ,
/// on the full turn and not merely mod half of one — see `CKind::Angle` for why that is the
/// statement.
///
/// The residual is the angular gap itself, so it carries no power of length at all: degree 0,
/// judged in absolute radians, and its gradient carries 1/length — which is exactly what
/// `extent^(degree − 1)` says of it.  The wrap's cut sits at the pose farthest from the
/// solution, where the residual is at its largest, so a descent runs away from it rather than
/// across it.
#[inline]
fn angle_gap(v: &[f64], theta: f64) -> f64 {
    let (dot, cross) = dot_cross(v);
    wrap_turn(cross.datan2(dot) - theta)
}

/// Its gradient in the eight direction columns.
///
/// The gap is a *difference* of two bearings — `atan2(d2) − atan2(d1)` — so each line's four
/// columns see only its own direction: `∂/∂d1 = (d1y, −d1x)/|d1|²` and `∂/∂d2 = (−d2y, d2x)/|d2|²`,
/// with each line's first endpoint carrying the negation of its second.  Written as one quotient
/// over `dot² + cross²` the two lengths appear to be coupled and every slot needs a division;
/// they are not, and `|d2|²` cancels out of the first four slots as `|d1|²` does out of the last
/// four.  So the eight entries are four numbers and their negations, at two reciprocals.
///
/// Each `|dᵢ|²` is floored at `MIN_LINE_LEN` *squared*, so it gives out at exactly the line
/// length `line_len` gives out at and for the same reason: a collapsed line would otherwise put a
/// NaN where the solver reads "no error".  Both halves of that matter.  Flooring the two lines
/// separately is what makes the guard a statement about a line at all — one floor over the
/// product clamps `|d1|²|d2|²`, a length to the *fourth* power, and so bites at a line a million
/// times longer than `MIN_LINE_LEN` names.  Squaring the constant is the rest of it: below the
/// floor the gradient decays instead of growing, which is the opposite of what `MIN_LINE_LEN` is
/// for, so the floor belongs where the length it is named for actually runs out.
#[inline]
fn angle_gap_jac(v: &[f64], j: &mut [f64]) {
    const MIN_LEN_SQ: f64 = MIN_LINE_LEN * MIN_LINE_LEN;
    let (d1x, d1y, d2x, d2y) = dirs(v);
    let i1 = 1.0 / (d1x * d1x + d1y * d1y).max(MIN_LEN_SQ);
    let i2 = 1.0 / (d2x * d2x + d2y * d2y).max(MIN_LEN_SQ);
    let (ax, ay) = (-d1y * i1, d1x * i1);
    let (bx, by) = (d2y * i2, -d2x * i2);
    j[0] = ax;
    j[1] = ay;
    j[2] = -ax;
    j[3] = -ay;
    j[4] = bx;
    j[5] = by;
    j[6] = -bx;
    j[7] = -by;
}

/// K = (theta): the directed angle from l1 to l2 is theta, mod a full turn
fn angle_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        r[i] = angle_gap(&v[8 * i..], k[i]);
    }
}

fn angle_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 8 * i;
        angle_gap_jac(&v[o..], &mut j[o..o + 8]);
    }
}

fn equal_length_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let (d1x, d1y, d2x, d2y) = dirs(&v[8 * i..8 * i + 8]);
        r[i] = d1x * d1x + d1y * d1y - d2x * d2x - d2y * d2y;
    }
}

fn equal_length_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let (d1x, d1y, d2x, d2y) = dirs(&v[8 * i..8 * i + 8]);
        let o = 8 * i;
        j[o] = -2.0 * d1x;
        j[o + 1] = -2.0 * d1y;
        j[o + 2] = 2.0 * d1x;
        j[o + 3] = 2.0 * d1y;
        j[o + 4] = 2.0 * d2x;
        j[o + 5] = 2.0 * d2y;
        j[o + 6] = -2.0 * d2x;
        j[o + 7] = -2.0 * d2y;
    }
}

/* -- incidence ------------------------------------------------------------- */

/// (px,py,ax,ay,bx,by): (b-a) x (p-a)
fn point_on_line_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 6 * i;
        let (dx, dy) = (v[o + 4] - v[o + 2], v[o + 5] - v[o + 3]);
        let (wx, wy) = (v[o] - v[o + 2], v[o + 1] - v[o + 3]);
        r[i] = dx * wy - dy * wx;
    }
}

fn point_on_line_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 6 * i;
        let (dx, dy) = (v[o + 4] - v[o + 2], v[o + 5] - v[o + 3]);
        let (wx, wy) = (v[o] - v[o + 2], v[o + 1] - v[o + 3]);
        j[o] = -dy;
        j[o + 1] = dx;
        j[o + 2] = dy - wy;
        j[o + 3] = wx - dx;
        j[o + 4] = wy;
        j[o + 5] = -wx;
    }
}

/// (px,py,cx,cy,r): |p-c|² - r²
fn point_on_circle_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 5 * i;
        let (ux, uy) = (v[o] - v[o + 2], v[o + 1] - v[o + 3]);
        r[i] = ux * ux + uy * uy - v[o + 4] * v[o + 4];
    }
}

fn point_on_circle_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 5 * i;
        let (ux, uy) = (v[o] - v[o + 2], v[o + 1] - v[o + 3]);
        j[o] = 2.0 * ux;
        j[o + 1] = 2.0 * uy;
        j[o + 2] = -2.0 * ux;
        j[o + 3] = -2.0 * uy;
        j[o + 4] = -2.0 * v[o + 4];
    }
}

/* -- radii ----------------------------------------------------------------- */

const RADIUS_J: &[f64] = &[1.0];

fn radius_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        r[i] = v[i] - k[i];
    }
}

fn radius_jac(n: usize, _v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        j[i] = 1.0;
    }
}

/* -- tangency -------------------------------------------------------------- */

/// (ax,ay,bx,by,cx,cy,r), K = (side): cross(b-a, c-a)/|b-a| - side*r
fn tangent_line_circle_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 7 * i;
        let (dx, dy) = (v[o + 2] - v[o], v[o + 3] - v[o + 1]);
        let (wx, wy) = (v[o + 4] - v[o], v[o + 5] - v[o + 1]);
        let l = line_len(dx, dy);
        let c = dx * wy - dy * wx;
        r[i] = c / l - k[i] * v[o + 6];
    }
}

fn tangent_line_circle_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 7 * i;
        let (dx, dy) = (v[o + 2] - v[o], v[o + 3] - v[o + 1]);
        let (wx, wy) = (v[o + 4] - v[o], v[o + 5] - v[o + 1]);
        let l = line_len(dx, dy);
        let c = dx * wy - dy * wx;
        let dc = [dy - wy, wx - dx, wy, -wx, -dy, dx, 0.0];
        let dl = [-dx / l, -dy / l, dx / l, dy / l, 0.0, 0.0, 0.0];
        ratio_jac(&dc, &dl, l, c, &mut j[o..o + 7]);
        j[o + 6] = -k[i]; // the radius column is not part of the ratio
    }
}

/// (c1x,c1y,r1,c2x,c2y,r2), K = (sign): +1 external, -1 internal
fn tangent_circle_circle_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 6 * i;
        let (ux, uy) = (v[o] - v[o + 3], v[o + 1] - v[o + 4]);
        let rr = v[o + 2] + k[i] * v[o + 5];
        r[i] = ux * ux + uy * uy - rr * rr;
    }
}

fn tangent_circle_circle_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 6 * i;
        let (ux, uy) = (v[o] - v[o + 3], v[o + 1] - v[o + 4]);
        let rr = v[o + 2] + k[i] * v[o + 5];
        j[o] = 2.0 * ux;
        j[o + 1] = 2.0 * uy;
        j[o + 2] = -2.0 * rr;
        j[o + 3] = -2.0 * ux;
        j[o + 4] = -2.0 * uy;
        j[o + 5] = -2.0 * rr * k[i];
    }
}

/// (px,py,cx,cy,ax,ay,bx,by): (p-c)·(b-a)
fn tangent_arc_line_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 8 * i;
        r[i] = (v[o] - v[o + 2]) * (v[o + 6] - v[o + 4])
            + (v[o + 1] - v[o + 3]) * (v[o + 7] - v[o + 5]);
    }
}

fn tangent_arc_line_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 8 * i;
        let (ux, uy) = (v[o] - v[o + 2], v[o + 1] - v[o + 3]);
        let (dx, dy) = (v[o + 6] - v[o + 4], v[o + 7] - v[o + 5]);
        j[o] = dx;
        j[o + 1] = dy;
        j[o + 2] = -dx;
        j[o + 3] = -dy;
        j[o + 4] = -ux;
        j[o + 5] = -uy;
        j[o + 6] = ux;
        j[o + 7] = uy;
    }
}

/* -- symmetry -------------------------------------------------------------- */

/// (px,py,qx,qy,ax,ay,bx,by): p and q mirror each other across the line a->b.  Two residuals:
/// the midpoint lies on the line (written as p + q - 2a to avoid the halving), and p->q is
/// perpendicular to it.
fn symmetric_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 8 * i;
        let (dx, dy) = (v[o + 6] - v[o + 4], v[o + 7] - v[o + 5]);
        let mx = v[o] + v[o + 2] - 2.0 * v[o + 4];
        let my = v[o + 1] + v[o + 3] - 2.0 * v[o + 5];
        r[2 * i] = dx * my - dy * mx;
        r[2 * i + 1] = (v[o + 2] - v[o]) * dx + (v[o + 3] - v[o + 1]) * dy;
    }
}

fn symmetric_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 8 * i;
        let jo = 16 * i;
        let (dx, dy) = (v[o + 6] - v[o + 4], v[o + 7] - v[o + 5]);
        let mx = v[o] + v[o + 2] - 2.0 * v[o + 4];
        let my = v[o + 1] + v[o + 3] - 2.0 * v[o + 5];
        let (ex, ey) = (v[o + 2] - v[o], v[o + 3] - v[o + 1]);
        j[jo] = -dy;
        j[jo + 1] = dx;
        j[jo + 2] = -dy;
        j[jo + 3] = dx;
        j[jo + 4] = 2.0 * dy - my;
        j[jo + 5] = mx - 2.0 * dx;
        j[jo + 6] = my;
        j[jo + 7] = -mx;
        j[jo + 8] = -dx;
        j[jo + 9] = -dy;
        j[jo + 10] = dx;
        j[jo + 11] = dy;
        j[jo + 12] = -ex;
        j[jo + 13] = -ey;
        j[jo + 14] = ex;
        j[jo + 15] = ey;
    }
}

/// The signed perpendicular distance from l2's first endpoint to l1's infinite line, from
/// (a1x,a1y,b1x,b1y,a2x,a2y,b2x,b2y).
#[inline]
fn parallel_gap(v: &[f64]) -> f64 {
    let (d1x, d1y) = (v[2] - v[0], v[3] - v[1]);
    let (wx, wy) = (v[4] - v[0], v[5] - v[1]);
    (d1x * wy - d1y * wx) / line_len(d1x, d1y)
}

/// Its gradient in those eight columns.
#[inline]
fn parallel_gap_jac(v: &[f64], j: &mut [f64]) {
    let (d1x, d1y) = (v[2] - v[0], v[3] - v[1]);
    let (wx, wy) = (v[4] - v[0], v[5] - v[1]);
    let l = line_len(d1x, d1y);
    let c = d1x * wy - d1y * wx;
    let dc = [d1y - wy, wx - d1x, wy, -wx, -d1y, d1x, 0.0, 0.0];
    let dl = [-d1x / l, -d1y / l, d1x / l, d1y / l, 0.0, 0.0, 0.0, 0.0];
    ratio_jac(&dc, &dl, l, c, j);
}

/// (a1x,a1y,b1x,b1y,a2x,a2y,b2x,b2y), K = (d): signed perpendicular distance from l2's first
/// endpoint to l1's infinite line.  It does NOT make them parallel.
fn parallel_distance_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        r[i] = parallel_gap(&v[8 * i..]) - k[i];
    }
}

fn parallel_distance_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 8 * i;
        parallel_gap_jac(&v[o..], &mut j[o..o + 8]);
    }
}

/// The signed perpendicular distance from p to the infinite line through a,b, positive to the
/// left of a→b, from (px,py,ax,ay,bx,by).
#[inline]
fn point_line_gap(v: &[f64]) -> f64 {
    let (dx, dy) = (v[4] - v[2], v[5] - v[3]);
    let (wx, wy) = (v[0] - v[2], v[1] - v[3]);
    (dx * wy - dy * wx) / line_len(dx, dy)
}

/// Its gradient in those six columns.
#[inline]
fn point_line_gap_jac(v: &[f64], j: &mut [f64]) {
    let (dx, dy) = (v[4] - v[2], v[5] - v[3]);
    let (wx, wy) = (v[0] - v[2], v[1] - v[3]);
    let l = line_len(dx, dy);
    let c = dx * wy - dy * wx;
    let dc = [-dy, dx, dy - wy, wx - dx, wy, -wx];
    let dl = [0.0, 0.0, -dx / l, -dy / l, dx / l, dy / l];
    ratio_jac(&dc, &dl, l, c, j);
}

/// (px,py,ax,ay,bx,by), K = (d): signed perpendicular distance from p to the infinite line
/// through a,b, positive to the left of a→b.
fn point_line_distance_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        r[i] = point_line_gap(&v[6 * i..]) - k[i];
    }
}

fn point_line_distance_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 6 * i;
        point_line_gap_jac(&v[o..], &mut j[o..o + 6]);
    }
}

/* -- the magnitude forms ----------------------------------------------------
 *
 * **A distance from a line is a magnitude** (spec §9.2, issue #48, item 4): `p distance(12) l`
 * says p is 12 from l and says nothing about which side, so *both* sides are solutions and the
 * seed picks between them, which is what a seed is for (P3) and what every other sketcher does.
 * The signed kernels above are what a statement that *pins* a side compiles to, the sign coming
 * from the word rather than from a minus somebody had to work out.
 *
 * `|g| − d` and **not** the squared form `g² − d²` that `distance` uses between two points.
 * Squared, the gradient is `2g·∂g` and vanishes where g does — and `distance(0)` from a line is
 * an idiom a drawing writes (a point on the axis, repeated thirty times over in one
 * cylinder), which squared is a double root with no gradient at all: rank deficient, reported
 * over-constrained, a freedom that is not there.  The absolute value keeps the signed form's
 * degree and conditioning — its Jacobian is `±∂g`, a unit-length row wherever the point is — at
 * the price of a crease at g = 0, which is the boundary *between* the two sides and the one
 * place a solve should not be walking through anyway.
 */

/// Which way the gap runs, as a multiplier: at exactly zero it counts as the left, so the
/// gradient never vanishes and a point seeded on the line still knows which way to move.
#[inline]
fn side_of(g: f64) -> f64 {
    if g < 0.0 {
        -1.0
    } else {
        1.0
    }
}

/// (px,py,ax,ay,bx,by), K = (d): |g| − d, g the perpendicular distance from p to the line.
fn point_line_magnitude_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        r[i] = point_line_gap(&v[6 * i..]).abs() - k[i];
    }
}

fn point_line_magnitude_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 6 * i;
        let s = side_of(point_line_gap(&v[o..]));
        point_line_gap_jac(&v[o..], &mut j[o..o + 6]);
        for x in &mut j[o..o + 6] {
            *x *= s;
        }
    }
}

/// (px,py,ax,ay,bx,by,a), K = (m,c): |g| − d, d = m*a + c
fn point_line_magnitude_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 7 * i;
        r[i] = point_line_gap(&v[o..]).abs() - free_dim(v, k, i, o + 6).0;
    }
}

fn point_line_magnitude_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 7 * i;
        let s = side_of(point_line_gap(&v[o..]));
        point_line_gap_jac(&v[o..], &mut j[o..o + 6]);
        for x in &mut j[o..o + 6] {
            *x *= s;
        }
        j[o + 6] = -k[2 * i];
    }
}

/// (a1x,a1y,b1x,b1y,a2x,a2y,b2x,b2y), K = (d): |g| − d, g the gap from l2's first endpoint to
/// l1's infinite line.  It does NOT make them parallel, any more than the signed form does.
fn parallel_magnitude_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        r[i] = parallel_gap(&v[8 * i..]).abs() - k[i];
    }
}

fn parallel_magnitude_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 8 * i;
        let s = side_of(parallel_gap(&v[o..]));
        parallel_gap_jac(&v[o..], &mut j[o..o + 8]);
        for x in &mut j[o..o + 8] {
            *x *= s;
        }
    }
}

/// (a1x,a1y,b1x,b1y,a2x,a2y,b2x,b2y,a), K = (m,c): |g| − d, d = m*a + c
fn parallel_magnitude_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        r[i] = parallel_gap(&v[o..]).abs() - free_dim(v, k, i, o + 8).0;
    }
}

fn parallel_magnitude_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        let s = side_of(parallel_gap(&v[o..]));
        parallel_gap_jac(&v[o..], &mut j[o..o + 8]);
        for x in &mut j[o..o + 8] {
            *x *= s;
        }
        j[o + 8] = -k[2 * i];
    }
}

/// (r1,r2), K = (d): r2 - r1 - d, the radial gap between two concentric circles.
const ANNULAR_DISTANCE_J: &[f64] = &[-1.0, 1.0];

fn annular_distance_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        r[i] = v[2 * i + 1] - v[2 * i] - k[i];
    }
}

fn annular_distance_jac(n: usize, _v: &[f64], _k: &[f64], j: &mut [f64]) {
    lin_jac(n, ANNULAR_DISTANCE_J, j)
}

/* -- parametric curves ----------------------------------------------------- */
/*
 * The two contact kernels.  Both are written against the basis alone — the values `b`, the
 * first derivatives `d` and the second derivatives `dd` of the `SPAN_N` functions that are
 * non-zero at t — so they say nothing about the curve beyond that it is a linear combination of
 * its control points.  Their column counts are sized from `SPAN_N`, so a second degree costs a
 * kernel pair and a `CKind` that selects it; a second curve family of the same degree costs
 * neither.
 *
 * The curve's columns are one span's control points, `SPAN_N` of them, whichever span t is in;
 * the span itself is chosen at compile time and carried in `Sketch::topology_key`.  The local
 * knot window and its control points' weights (`curve::weigh`) are the constants.
 */

use crate::curve::{self, SPAN_C, SPAN_K, SPAN_N};

/// One span evaluated at t: the basis, and the curve point and derivatives its control points
/// give.  Both kernels' residual and Jacobian need some part of this, and they differ only in
/// where their columns put t and the control points — which is the whole of what makes them two
/// kernels rather than one.
struct Span {
    b: [f64; SPAN_N],
    d: [f64; SPAN_N],
    dd: [f64; SPAN_N],
    d3: [f64; SPAN_N],
    p: (f64, f64),
    d1: (f64, f64),
    d2: (f64, f64),
    d3v: (f64, f64),
}

/// `v` is one instance's columns; `t` and `ctrl` are the offsets into it of the parameter and of
/// the first control point.
fn span_frame(v: &[f64], t: usize, ctrl: usize, k: &[f64; SPAN_C]) -> Span {
    let mut f = Span {
        b: [0.0; SPAN_N],
        d: [0.0; SPAN_N],
        dd: [0.0; SPAN_N],
        d3: [0.0; SPAN_N],
        p: (0.0, 0.0),
        d1: (0.0, 0.0),
        d2: (0.0, 0.0),
        d3v: (0.0, 0.0),
    };
    let (knots, weights) = k.split_at(SPAN_K);
    curve::basis(v[t], knots.try_into().unwrap(), &mut f.b, &mut f.d, &mut f.dd, &mut f.d3);
    curve::weigh(weights.try_into().unwrap(), &mut f.b, &mut f.d, &mut f.dd, &mut f.d3);
    for a in 0..SPAN_N {
        let (x, y) = (v[ctrl + 2 * a], v[ctrl + 2 * a + 1]);
        f.p.0 += f.b[a] * x;
        f.p.1 += f.b[a] * y;
        f.d1.0 += f.d[a] * x;
        f.d1.1 += f.d[a] * y;
        f.d2.0 += f.dd[a] * x;
        f.d2.1 += f.dd[a] * y;
        f.d3v.0 += f.d3[a] * x;
        f.d3v.1 += f.d3[a] * y;
    }
    f
}

/// The i-th instance's local knot window and weights out of a block's constants.
#[inline]
fn span_knots(k: &[f64], i: usize) -> &[f64; SPAN_C] {
    k[SPAN_C * i..SPAN_C * (i + 1)].try_into().expect("a block's constants are SPAN_C per row")
}

/// Columns of `point_on_spline`: (px, py, t, c0x, c0y, ... c3x, c3y).
pub const N_PAR_ON_SPLINE: usize = 3 + 2 * SPAN_N;
/// Columns of `spline_tangent_line`: (t, c0x, c0y, ... c3x, c3y, ax, ay, bx, by).
pub const N_PAR_SPLINE_LINE: usize = 1 + 2 * SPAN_N + 4;

/// `r = p − C(t)`.  Two residuals against one new unknown: the net one equation a point lying
/// on a curve is worth.  A signed displacement, so degree 1.
fn point_on_spline_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_ON_SPLINE * i;
        let f = span_frame(&v[o..], 2, 3, span_knots(k, i));
        r[2 * i] = v[o] - f.p.0;
        r[2 * i + 1] = v[o + 1] - f.p.1;
    }
}

fn point_on_spline_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_ON_SPLINE * i;
        let jo = 2 * N_PAR_ON_SPLINE * i;
        let row1 = jo + N_PAR_ON_SPLINE;
        let f = span_frame(&v[o..], 2, 3, span_knots(k, i));
        for t in 0..2 * N_PAR_ON_SPLINE {
            j[jo + t] = 0.0;
        }
        j[jo] = 1.0;
        j[row1 + 1] = 1.0;
        j[jo + 2] = -f.d1.0;
        j[row1 + 2] = -f.d1.1;
        for a in 0..SPAN_N {
            j[jo + 3 + 2 * a] = -f.b[a];
            j[row1 + 4 + 2 * a] = -f.b[a];
        }
    }
}

/// Tangency of a curve and an infinite line, as one constraint owning one parameter: the point
/// at t lies on the line, and the curve's direction there is the line's.  Two residuals against
/// one new unknown, so the net one equation a tangency is worth.
///
/// It has to be one constraint.  Split into "a point is on the curve" and "the direction
/// matches" it would be two contacts with two parameters of their own, tangent to each other
/// only if something else made the parameters agree.
///
/// Both rows are divided by the line's length, which is what makes them mean something: row 0 is
/// then the distance from the contact to the line and row 1 is |C'| sin θ, both signed lengths,
/// so the kernel is degree 1.  Without the division a line whose endpoints had collapsed would
/// satisfy the pair exactly — a cross product with a zero vector is zero — and the solver is
/// perfectly happy to find that.  `line_len`'s floor is what keeps the residual finite and the
/// Jacobian large there, exactly as in the line/circle tangency.
///
/// (t, c0x, c0y ... c3x, c3y, ax, ay, bx, by)
fn spline_tangent_line_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_SPLINE_LINE * i;
        let l = o + 1 + 2 * SPAN_N;
        let f = span_frame(&v[o..], 0, 1, span_knots(k, i));
        let (dx, dy) = (v[l + 2] - v[l], v[l + 3] - v[l + 1]);
        let (wx, wy) = (f.p.0 - v[l], f.p.1 - v[l + 1]);
        let len = line_len(dx, dy);
        r[2 * i] = (dx * wy - dy * wx) / len;
        r[2 * i + 1] = (f.d1.0 * dy - f.d1.1 * dx) / len;
    }
}

fn spline_tangent_line_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let mut dc = [0.0f64; N_PAR_SPLINE_LINE];
    let mut dl = [0.0f64; N_PAR_SPLINE_LINE];
    for i in 0..n {
        let o = N_PAR_SPLINE_LINE * i;
        let l = o + 1 + 2 * SPAN_N;
        let jo = 2 * N_PAR_SPLINE_LINE * i;
        let row1 = jo + N_PAR_SPLINE_LINE;
        let f = span_frame(&v[o..], 0, 1, span_knots(k, i));
        let (tx, ty) = f.d1;
        let (dx, dy) = (v[l + 2] - v[l], v[l + 3] - v[l + 1]);
        let (wx, wy) = (f.p.0 - v[l], f.p.1 - v[l + 1]);
        let len = line_len(dx, dy);
        let ll = l - o; // first line column
        // |b - a| moves with the line's endpoints only
        for t in 0..N_PAR_SPLINE_LINE {
            dl[t] = 0.0;
        }
        dl[ll] = -dx / len;
        dl[ll + 1] = -dy / len;
        dl[ll + 2] = dx / len;
        dl[ll + 3] = dy / len;

        // row 0: cross(b - a, C(t) - a)
        for t in 0..N_PAR_SPLINE_LINE {
            dc[t] = 0.0;
        }
        dc[0] = dx * ty - dy * tx;
        for a in 0..SPAN_N {
            dc[1 + 2 * a] = -dy * f.b[a];
            dc[2 + 2 * a] = dx * f.b[a];
        }
        dc[ll] = dy - wy;
        dc[ll + 1] = wx - dx;
        dc[ll + 2] = wy;
        dc[ll + 3] = -wx;
        ratio_jac(&dc, &dl, len, dx * wy - dy * wx, &mut j[jo..jo + N_PAR_SPLINE_LINE]);

        // row 1: cross(C'(t), b - a)
        for t in 0..N_PAR_SPLINE_LINE {
            dc[t] = 0.0;
        }
        dc[0] = f.d2.0 * dy - f.d2.1 * dx;
        for a in 0..SPAN_N {
            dc[1 + 2 * a] = f.d[a] * dy;
            dc[2 + 2 * a] = -f.d[a] * dx;
        }
        dc[ll] = ty;
        dc[ll + 1] = -tx;
        dc[ll + 2] = -ty;
        dc[ll + 3] = tx;
        ratio_jac(&dc, &dl, len, tx * dy - ty * dx, &mut j[row1..row1 + N_PAR_SPLINE_LINE]);
    }
}

/// Columns of `spline_curvature`: (t, c0x, c0y ... c3x, c3y, cx, cy, r).
pub const N_PAR_SPLINE_CURVE: usize = 1 + 2 * SPAN_N + 3;

/// `cross(C', C'')`, floored.  It is the curve's turning, and the osculating circle's centre is
/// a whole `(C'·C')/turn` away along the normal — so where the curve does not turn there is no
/// finite circle to be had, and the floor keeps that as a very large residual rather than an
/// infinity.  Same bargain as `MIN_LINE_LEN`, for the same reason.
const MIN_TURN: f64 = 1e-12;

fn turn(k: f64) -> f64 {
    if k.abs() < MIN_TURN {
        MIN_TURN.copysign(if k == 0.0 { 1.0 } else { k })
    } else {
        k
    }
}

/// A circle that osculates the curve: it touches, shares the tangent, and bends by the same
/// amount — the circle a draughtsman would call the radius *of* the curve there.
///
/// Written as "the centre is the centre of curvature", which says all three at once and leaves
/// no branch to choose: that centre is `C + ((C'·C')/cross(C',C'')) · perp(C')`, and the first
/// two rows are the two components of `centre − C` minus that offset.  Placing the centre
/// exactly is what makes a `side` argument unnecessary — the sign of the turning already says
/// which way the curve bends.  The third row is the radius, so all three are signed lengths and
/// the kernel is degree 1.
///
/// Dividing by the turning rather than multiplying by it is load-bearing.  Multiplied through,
/// every row would vanish as `C'` did, and the solver could satisfy the constraint by bunching
/// the control points until the parameterisation collapsed instead of by bending the curve —
/// which it promptly does, given the freedom.  Divided, a collapsing curve leaves `centre − C`
/// standing at nearly the whole radius, and the residual pushes back.
///
/// Three residuals against one new unknown: net two, which is what an osculating circle costs.
/// It keeps the one degree of freedom it should — it can slide along the curve.
fn spline_curvature_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_SPLINE_CURVE * i;
        let c = o + 1 + 2 * SPAN_N;
        let f = span_frame(&v[o..], 0, 1, span_knots(k, i));
        let (tx, ty) = f.d1;
        let (dx, dy) = (v[c] - f.p.0, v[c + 1] - f.p.1);
        let g = (tx * tx + ty * ty) / turn(tx * f.d2.1 - ty * f.d2.0);
        r[3 * i] = dx + g * ty;
        r[3 * i + 1] = dy - g * tx;
        r[3 * i + 2] = line_len(dx, dy) - v[c + 2];
    }
}

fn spline_curvature_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_SPLINE_CURVE * i;
        let c = o + 1 + 2 * SPAN_N;
        let cc = c - o; // first circle column, relative to this instance
        let jo = 3 * N_PAR_SPLINE_CURVE * i;
        let (row1, row2) = (jo + N_PAR_SPLINE_CURVE, jo + 2 * N_PAR_SPLINE_CURVE);
        let f = span_frame(&v[o..], 0, 1, span_knots(k, i));
        let (tx, ty) = f.d1;
        let (sx, sy) = f.d2;
        let (ux, uy) = f.d3v;
        let (dx, dy) = (v[c] - f.p.0, v[c + 1] - f.p.1);
        let q = tx * tx + ty * ty;
        let kk = turn(tx * sy - ty * sx);
        let g = q / kk;
        let len = line_len(dx, dy);
        for s in 0..3 * N_PAR_SPLINE_CURVE {
            j[jo + s] = 0.0;
        }

        // t: C moves along C', C' along C'', and the turning along cross(C', C''')
        let dq = 2.0 * (tx * sx + ty * sy);
        let dg = (dq * kk - q * (tx * uy - ty * ux)) / (kk * kk);
        j[jo] = -tx + dg * ty + g * sy;
        j[row1] = -ty - dg * tx - g * sx;
        j[row2] = (dx * -tx + dy * -ty) / len;

        for a in 0..SPAN_N {
            // a control point moves C, C' and C'' at once, each by its own basis function
            let dgx = (2.0 * tx * f.d[a] * kk - q * (f.d[a] * sy - ty * f.dd[a])) / (kk * kk);
            let dgy = (2.0 * ty * f.d[a] * kk - q * (tx * f.dd[a] - f.d[a] * sx)) / (kk * kk);
            j[jo + 1 + 2 * a] = -f.b[a] + dgx * ty;
            j[jo + 2 + 2 * a] = dgy * ty + g * f.d[a];
            j[row1 + 1 + 2 * a] = -dgx * tx - g * f.d[a];
            j[row1 + 2 + 2 * a] = -f.b[a] - dgy * tx;
            j[row2 + 1 + 2 * a] = -dx * f.b[a] / len;
            j[row2 + 2 + 2 * a] = -dy * f.b[a] / len;
        }

        j[jo + cc] = 1.0;
        j[row1 + cc + 1] = 1.0;
        j[row2 + cc] = dx / len;
        j[row2 + cc + 1] = dy / len;
        j[row2 + cc + 2] = -1.0;
    }
}

/* -- dimensions written in terms of a free variable ------------------------ */
/*
 * A dimension whose number is not stated but *shared* — `a` on two of them, `a / 2` on a third —
 * has an unknown where its constant was.  These are the same kernels again with that one change:
 * the value comes off the end of the columns as `m * a + c` rather than out of the constants, and
 * `m` and `c` are what the constants now hold.  See `expr::Free` for why the tie is affine: one
 * column and two constants is the whole of what a fixed-width block can carry, and it is enough
 * for everything a draughtsman writes — the same length again, half of it, ten more than it.
 *
 * The free column always comes last, so `Constraint::params_on` appends it and needs to know
 * nothing else about which kernel it is feeding.
 */

/// The value the free column stands for, and how fast it moves with it.
#[inline]
fn free_dim(v: &[f64], k: &[f64], i: usize, at: usize) -> (f64, f64) {
    let (m, c) = (k[2 * i], k[2 * i + 1]);
    (m * v[at] + c, m)
}

/// (px,py,qx,qy,a), K = (m,c): |p-q|² - d², d = m*a + c
fn distance_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 5 * i;
        let (d, _) = free_dim(v, k, i, o + 4);
        r[i] = dist_sq(&v[o..]) - d * d;
    }
}

fn distance_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 5 * i;
        let (d, m) = free_dim(v, k, i, o + 4);
        dist_sq_jac(&v[o..], &mut j[o..o + 4]);
        j[o + 4] = -2.0 * d * m;
    }
}

/// (px,py,qx,qy,a), K = (m,c): (qx - px) - (m*a + c)
fn ordinate_u_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 5 * i;
        r[i] = v[o + 2] - v[o] - free_dim(v, k, i, o + 4).0;
    }
}

fn ordinate_u_free_jac(n: usize, _v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 5 * i;
        j[o..o + 4].copy_from_slice(ORDINATE_U_J);
        j[o + 4] = -k[2 * i];
    }
}

/// (px,py,qx,qy,a), K = (m,c): (qy - py) - (m*a + c)
fn ordinate_v_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 5 * i;
        r[i] = v[o + 3] - v[o + 1] - free_dim(v, k, i, o + 4).0;
    }
}

fn ordinate_v_free_jac(n: usize, _v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 5 * i;
        j[o..o + 4].copy_from_slice(ORDINATE_V_J);
        j[o + 4] = -k[2 * i];
    }
}

/// (a1x,a1y,b1x,b1y,a2x,a2y,b2x,b2y,a), K = (m,c): wrap(atan2(cross, dot) − θ), θ = m*a + c
fn angle_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        r[i] = angle_gap(&v[o..], free_dim(v, k, i, o + 8).0);
    }
}

fn angle_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        angle_gap_jac(&v[o..], &mut j[o..o + 8]);
        j[o + 8] = -k[2 * i];
    }
}

/// (a1x,a1y,b1x,b1y,a2x,a2y,b2x,b2y,a), K = (m,c): the signed gap, less m*a + c
fn parallel_distance_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        r[i] = parallel_gap(&v[o..]) - free_dim(v, k, i, o + 8).0;
    }
}

fn parallel_distance_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        parallel_gap_jac(&v[o..], &mut j[o..o + 8]);
        j[o + 8] = -k[2 * i];
    }
}

/// (px,py,ax,ay,bx,by,a), K = (m,c): the signed distance to the line, less m*a + c
fn point_line_distance_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 7 * i;
        r[i] = point_line_gap(&v[o..]) - free_dim(v, k, i, o + 6).0;
    }
}

fn point_line_distance_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 7 * i;
        point_line_gap_jac(&v[o..], &mut j[o..o + 6]);
        j[o + 6] = -k[2 * i];
    }
}

/// (r1,r2,a), K = (m,c): r2 - r1 - (m*a + c)
fn annular_distance_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 3 * i;
        r[i] = v[o + 1] - v[o] - free_dim(v, k, i, o + 2).0;
    }
}

fn annular_distance_free_jac(n: usize, _v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 3 * i;
        j[o..o + 2].copy_from_slice(ANNULAR_DISTANCE_J);
        j[o + 2] = -k[2 * i];
    }
}

/// (r,a), K = (m,c): r - (m*a + c)
fn radius_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 2 * i;
        r[i] = v[o] - free_dim(v, k, i, o + 1).0;
    }
}

fn radius_free_jac(n: usize, _v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 2 * i;
        j[o..o + 1].copy_from_slice(RADIUS_J);
        j[o + 1] = -k[2 * i];
    }
}

/* -- registry (order == kernel id, shared with the bindings) --------------- */

/* -- curves written in the language ------------------------------------------------
 *
 * One point on one curve: `p - C(u) = 0`, two residuals against the one parameter the contact
 * owns — the same bargain `point_on_spline` strikes, and the same net one equation.
 *
 * What differs is where `C` comes from.  A spline has a basis the kernel knows; a curve written
 * in the language is a pair of *expressions*, so the compiled tapes ride in the constraint's
 * constants and `tape::eval_flat` runs them.  The gradient that comes back is already in the
 * kernel's column order — the parameter, then every scalar the curve's arguments contribute —
 * because `Sketch::curve_vars` and `Constraint::params_on` are written against the same order.
 * So the tape's gradient *is* the Jacobian row, with nothing to rearrange.
 *
 * The constants are `[n_vars, len_x, len_y, x…, y…, values…]`: two tapes and whatever numbers
 * the instance was given, which the expressions read as constants and whose gradients are
 * computed and ignored.
 */

thread_local! {
    /// Scratch the tapes run in.  A kernel is a `fn` and cannot own state, and allocating per
    /// residual is the one thing the compile-to-plan seam exists to prevent.
    static CURVE_SCRATCH: std::cell::RefCell<crate::tape::Scratch> =
        std::cell::RefCell::new(crate::tape::Scratch::new());
}

/// Split a curve contact's constants into its two tapes and the numbers it was given.
fn curve_parts(k: &[f64]) -> (usize, &[f64], &[f64], &[f64]) {
    let n_vars = k[0] as usize;
    let (lx, ly) = (k[1] as usize, k[2] as usize);
    let x = &k[3..3 + lx];
    let y = &k[3 + lx..3 + lx + ly];
    (n_vars, x, y, &k[3 + lx + ly..])
}

/// The variable vector a tape is evaluated at: the parameter, then the curve's coordinates, then
/// its given numbers — the definition's own order.
fn curve_x(u: f64, theta: &[f64], values: &[f64], out: &mut [f64]) -> usize {
    out[0] = u;
    out[1..1 + theta.len()].copy_from_slice(theta);
    out[1 + theta.len()..1 + theta.len() + values.len()].copy_from_slice(values);
    1 + theta.len() + values.len()
}

/// A curve's **frame** at a contact, whichever way the definition places it: the point and its
/// first three derivatives in the parameter, and the gradient of the first three orders in the
/// outer columns `[u, θ…]`.  A formula gives all of it exactly (`tape::Series`); a trace gives
/// `C` to `C'''` exactly when asked for its higher orders (`locus::higher_orders`, over the
/// kernels' Taylor forms) and the gradients along θ by difference (`locus::kernel_frame`) —
/// without them, `C''` on are NaN, and a curvature is not stated (`constraints::validate`).
///
/// A **residual** reads only the derivatives (`curve_value`), never the gradient: a rejected
/// trust-region step evaluates residuals without ever asking for a Jacobian, and for a trace the
/// gradient is a sweep of block solves — the `EllFrame` bargain, kept here for the same reason.
struct CurveFrame {
    /// `c[k]` is `dᵏC/duᵏ` for `k` in 0..4, as (x, y).
    c: [[f64; 2]; 4],
    /// `g[k][j]` is `∂c[k]/∂outer[j]` for `k` in 0..3, as (x, y).
    g: [[[f64; crate::tape::MAX_VARS]; 2]; 3],
}

/// Which body a curve kernel's constants hold — a formula's two tapes, a trace's block, or a
/// generated profile's tool and motion (`generate.rs`).  A const parameter of the kernels, so
/// each body is its own monomorphised `fn` and the table stays `fn`-pointered.
pub const FORMULA: u8 = 0;
pub const TRACE: u8 = 1;
pub const ENVELOPE: u8 = 2;

/// `C` and its derivatives in the parameter, for a residual.  `BODY` says which body the
/// constants hold — a formula's two tapes, a trace's block, a generated profile's tool and
/// motion; `need` how many orders a trace or a profile is to work out (a formula gives all).
fn curve_value<const BODY: u8>(k: &[f64], u: f64, theta: &[f64], need: u8) -> [[f64; 2]; 4] {
    if BODY != FORMULA {
        let val = body_val::<BODY>(k, u, theta, need, false);
        return [[val.x, val.y], [val.dx[0], val.dy[0]], val.d2, val.d3];
    }
    CURVE_SCRATCH.with(|sc| {
        let sc = &mut *sc.borrow_mut();
        let (n_vars, tx, ty, values) = curve_parts(k);
        let mut x = [0.0f64; crate::tape::MAX_VARS];
        curve_x(u, theta, values, &mut x);
        let sx = crate::tape::eval_series_flat(tx, n_vars, &x, sc);
        let sy = crate::tape::eval_series_flat(ty, n_vars, &x, sc);
        std::array::from_fn(|k| [sx.c[k], sy.c[k]])
    })
}

/// A trace's or a generated profile's evaluation, orders to `need`, with the gradient along
/// the columns when `gradient` is asked (a trace's always has it).
fn body_val<const BODY: u8>(k: &[f64], u: f64, theta: &[f64], need: u8, gradient: bool) -> crate::locus::Val {
    if BODY == ENVELOPE {
        crate::generate::kernel_eval(k, u, theta, need, gradient)
    } else {
        crate::locus::kernel_eval_to(k, u, theta, need)
    }
}

/// The whole frame, for a Jacobian: orders to `need`, and the gradients of one fewer.
fn curve_frame<const BODY: u8>(k: &[f64], u: f64, theta: &[f64], need: u8) -> CurveFrame {
    if BODY != FORMULA {
        let fr = if BODY == ENVELOPE {
            crate::generate::kernel_frame(k, u, theta, need)
        } else {
            crate::locus::kernel_frame(k, u, theta, need)
        };
        let v = fr.val;
        return CurveFrame {
            c: [[v.x, v.y], [v.dx[0], v.dy[0]], v.d2, v.d3],
            g: [[v.dx, v.dy], fr.d1, fr.d2],
        };
    }
    CURVE_SCRATCH.with(|sc| {
        let sc = &mut *sc.borrow_mut();
        let (n_vars, tx, ty, values) = curve_parts(k);
        let mut x = [0.0f64; crate::tape::MAX_VARS];
        curve_x(u, theta, values, &mut x);
        let sx = crate::tape::eval_series_flat(tx, n_vars, &x, sc);
        let sy = crate::tape::eval_series_flat(ty, n_vars, &x, sc);
        CurveFrame {
            c: std::array::from_fn(|k| [sx.c[k], sy.c[k]]),
            g: std::array::from_fn(|k| [sx.g[k], sy.g[k]]),
        }
    })
}

/* -- a line tangent to a curve: (u, θ…, ax, ay, bx, by) ----------------------------------- */

/// Rows: `cross(b − a, C − a) / |b − a|` and `cross(C', b − a) / |b − a|` — the spline
/// tangency's, over the curve's own frame.
fn curve_tangent_res<const BODY: u8>(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    if n_par < 5 || n_const < 2 {
        return;
    }
    let n_theta = n_par - 5;
    for i in 0..n {
        let (o, ko) = (n_par * i, n_const * i);
        let c = curve_value::<BODY>(&k[ko..ko + n_const], v[o], &v[o + 1..o + 1 + n_theta], 1);
        let l = o + 1 + n_theta;
        let (dx, dy) = (v[l + 2] - v[l], v[l + 3] - v[l + 1]);
        let (wx, wy) = (c[0][0] - v[l], c[0][1] - v[l + 1]);
        let len = line_len(dx, dy);
        r[2 * i] = (dx * wy - dy * wx) / len;
        r[2 * i + 1] = (c[1][0] * dy - c[1][1] * dx) / len;
    }
}

fn curve_tangent_jac<const BODY: u8>(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    const W: usize = crate::tape::MAX_VARS + 4;
    let (n_par, n_const) = curve_widths(n, v, k);
    if n_par < 5 || n_const < 2 || n_par > W {
        return;
    }
    let n_theta = n_par - 5;
    let (mut dc, mut dl) = ([0.0f64; W], [0.0f64; W]);
    for i in 0..n {
        let (o, ko) = (n_par * i, n_const * i);
        let f = curve_frame::<BODY>(&k[ko..ko + n_const], v[o], &v[o + 1..o + 1 + n_theta], 2);
        let l = o + 1 + n_theta;
        let ll = l - o;
        let jo = 2 * n_par * i;
        let (dx, dy) = (v[l + 2] - v[l], v[l + 3] - v[l + 1]);
        let (wx, wy) = (f.c[0][0] - v[l], f.c[0][1] - v[l + 1]);
        let (tx, ty) = (f.c[1][0], f.c[1][1]);
        let len = line_len(dx, dy);
        // |b − a| moves with the line's ends only
        dl[..n_par].fill(0.0);
        dl[ll..ll + 4].copy_from_slice(&[-dx / len, -dy / len, dx / len, dy / len]);
        // row 0: cross(b − a, C − a) — the curve's columns move C, the line's move both ends
        for c in 0..1 + n_theta {
            dc[c] = dx * f.g[0][1][c] - dy * f.g[0][0][c];
        }
        dc[ll..ll + 4].copy_from_slice(&[dy - wy, wx - dx, wy, -wx]);
        ratio_jac(&dc[..n_par], &dl[..n_par], len, dx * wy - dy * wx, &mut j[jo..jo + n_par]);
        // row 1: cross(C', b − a) — the curve's columns move C'
        for c in 0..1 + n_theta {
            dc[c] = f.g[1][0][c] * dy - f.g[1][1][c] * dx;
        }
        dc[ll..ll + 4].copy_from_slice(&[ty, -tx, -ty, tx]);
        ratio_jac(&dc[..n_par], &dl[..n_par], len, tx * dy - ty * dx, &mut j[jo + n_par..jo + 2 * n_par]);
    }
}

/* -- a circle osculating a curve: (u, θ…, cx, cy, r) ------------------------------------- */

/// Rows: the spline curvature's — the centre is the centre of curvature, and the radius is the
/// distance to it — over a formula's frame or a trace's (its higher orders exact, over the
/// kernels' Taylor forms).  The u column and every θ column go through one formula, since
/// `g[k][·][0]` is `c[k + 1]`: a column moves C, C' and C'' by its own three gradients, and for
/// `u` those are C', C'' and C'''.
fn curve_curvature_res<const BODY: u8>(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    if n_par < 4 || n_const < 2 {
        return;
    }
    let n_theta = n_par - 4;
    for i in 0..n {
        let (o, ko) = (n_par * i, n_const * i);
        let f = curve_value::<BODY>(&k[ko..ko + n_const], v[o], &v[o + 1..o + 1 + n_theta], 2);
        let c = o + 1 + n_theta;
        let ([tx, ty], [sx, sy]) = (f[1], f[2]);
        let (dx, dy) = (v[c] - f[0][0], v[c + 1] - f[0][1]);
        let g = (tx * tx + ty * ty) / turn(tx * sy - ty * sx);
        r[3 * i] = dx + g * ty;
        r[3 * i + 1] = dy - g * tx;
        r[3 * i + 2] = line_len(dx, dy) - v[c + 2];
    }
}

fn curve_curvature_jac<const BODY: u8>(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    if n_par < 4 || n_const < 2 {
        return;
    }
    let n_theta = n_par - 4;
    for i in 0..n {
        let (o, ko) = (n_par * i, n_const * i);
        let f = curve_frame::<BODY>(&k[ko..ko + n_const], v[o], &v[o + 1..o + 1 + n_theta], 3);
        let c = o + 1 + n_theta;
        let cc = c - o;
        let jo = 3 * n_par * i;
        let (row1, row2) = (jo + n_par, jo + 2 * n_par);
        let ([tx, ty], [sx, sy]) = (f.c[1], f.c[2]);
        let (dx, dy) = (v[c] - f.c[0][0], v[c + 1] - f.c[0][1]);
        let q = tx * tx + ty * ty;
        let kk = turn(tx * sy - ty * sx);
        let g = q / kk;
        let len = line_len(dx, dy);
        j[jo..jo + 3 * n_par].fill(0.0);
        for col in 0..1 + n_theta {
            let [ax, ay] = [f.g[0][0][col], f.g[0][1][col]];
            let [bx, by] = [f.g[1][0][col], f.g[1][1][col]];
            let [ex, ey] = [f.g[2][0][col], f.g[2][1][col]];
            let dq = 2.0 * (tx * bx + ty * by);
            let dkk = bx * sy + tx * ey - by * sx - ty * ex;
            let dg = (dq * kk - q * dkk) / (kk * kk);
            j[jo + col] = -ax + dg * ty + g * by;
            j[row1 + col] = -ay - dg * tx - g * bx;
            j[row2 + col] = (dx * -ax + dy * -ay) / len;
        }
        j[jo + cc] = 1.0;
        j[row1 + cc + 1] = 1.0;
        j[row2 + cc] = dx / len;
        j[row2 + cc + 1] = dy / len;
        j[row2 + cc + 2] = -1.0;
    }
}

/// A slot in the kernel table for a constraint a body cannot serve — a curvature against a
/// traced curve — filled so the table keeps its stride and `kernel_id_in` stays a function of
/// the kind and the definition.  `constraints::validate` refuses the constraint before it is
/// compiled; if one is compiled all the same, every row is NaN, which `System` reads as
/// "not converged" and never as "no error".
fn refused_res(_n: usize, _v: &[f64], _k: &[f64], r: &mut [f64]) {
    r.fill(f64::NAN);
}
fn refused_jac(_n: usize, _v: &[f64], _k: &[f64], j: &mut [f64]) {
    j.fill(f64::NAN);
}

/// The block's widths, read off the slices it was handed.
///
/// A kernel `fn` is not told its own `n_par` and `n_const` — every other kernel knows them as
/// constants of its own type.  A curve kernel's are per *definition*, so they are recovered here
/// from the lengths the block passed, which are `count * n_par` and `count * n_const` exactly.
fn curve_widths(n: usize, v: &[f64], k: &[f64]) -> (usize, usize) {
    if n == 0 {
        return (0, 0);
    }
    (v.len() / n, k.len() / n)
}

fn point_on_curve_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    if n_par < 3 || n_const < 3 {
        return;
    }
    CURVE_SCRATCH.with(|sc| {
        let mut sc = sc.borrow_mut();
        let mut x = [0.0f64; crate::tape::MAX_VARS];
        for i in 0..n {
            let (o, ko) = (n_par * i, n_const * i);
            let (n_vars, tx, ty, values) = curve_parts(&k[ko..ko + n_const]);
            curve_x(v[o + 2], &v[o + 3..o + n_par], values, &mut x);
            let cx = crate::tape::eval_flat(tx, n_vars, &x, &mut sc);
            let cy = crate::tape::eval_flat(ty, n_vars, &x, &mut sc);
            r[2 * i] = v[o] - cx.v;
            r[2 * i + 1] = v[o + 1] - cy.v;
        }
    });
}

fn point_on_curve_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    if n_par < 3 || n_const < 3 {
        return;
    }
    CURVE_SCRATCH.with(|sc| {
        let mut sc = sc.borrow_mut();
        let mut x = [0.0f64; crate::tape::MAX_VARS];
        for i in 0..n {
            let (o, ko) = (n_par * i, n_const * i);
            let jo = 2 * n_par * i;
            let row1 = jo + n_par;
            let (n_vars, tx, ty, values) = curve_parts(&k[ko..ko + n_const]);
            curve_x(v[o + 2], &v[o + 3..o + n_par], values, &mut x);
            let cx = crate::tape::eval_flat(tx, n_vars, &x, &mut sc);
            let cy = crate::tape::eval_flat(ty, n_vars, &x, &mut sc);
            for t in 0..2 * n_par {
                j[jo + t] = 0.0;
            }
            j[jo] = 1.0;
            j[row1 + 1] = 1.0;
            // the parameter, then every coordinate the curve reads — the tape's own order
            for c in 0..n_par - 2 {
                j[jo + 2 + c] = -cx.d[c];
                j[row1 + 2 + c] = -cy.d[c];
            }
        }
    });
}

/// A point on a trace's or a generated profile's curve: `p − C(u) = 0`, with `C` and its
/// gradient along the columns from the body.
fn point_on_body_res<const BODY: u8>(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    if n_par < 3 || n_const < 2 {
        return;
    }
    for i in 0..n {
        let (o, ko) = (n_par * i, n_const * i);
        let c = body_val::<BODY>(&k[ko..ko + n_const], v[o + 2], &v[o + 3..o + n_par], 1, false);
        r[2 * i] = v[o] - c.x;
        r[2 * i + 1] = v[o + 1] - c.y;
    }
}

fn point_on_body_jac<const BODY: u8>(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    if n_par < 3 || n_const < 2 {
        return;
    }
    for i in 0..n {
        let (o, ko) = (n_par * i, n_const * i);
        let jo = 2 * n_par * i;
        let row1 = jo + n_par;
        let c = body_val::<BODY>(&k[ko..ko + n_const], v[o + 2], &v[o + 3..o + n_par], 1, true);
        for t in 0..2 * n_par {
            j[jo + t] = 0.0;
        }
        j[jo] = 1.0;
        j[row1 + 1] = 1.0;
        // the parameter, then every coordinate the curve reads — the block's own order, which
        // is the tape order, which is the column order
        for t in 0..n_par - 2 {
            j[jo + 2 + t] = -c.dx[t];
            j[row1 + 2 + t] = -c.dy[t];
        }
    }
}

/// A point's lift read into its plane's coordinates, and the gradient of that place along the
/// lift: `k` is `EXTRUSION_FRAME`'s numbers, the plane's `u`, `v` and `o`.
fn extrusion_place(p: &[f64], k: &[f64]) -> ([f64; 2], [[f64; 3]; 2]) {
    let (u, w, o) = (&k[0..3], &k[3..6], &k[6..9]);
    let d = [p[0] - o[0], p[1] - o[1], p[2] - o[2]];
    let (a, b) = (d[0] * u[0] + d[1] * u[1] + d[2] * u[2], d[0] * w[0] + d[1] * w[1] + d[2] * w[2]);
    let grad = [std::array::from_fn(|i| u[i]), std::array::from_fn(|i| w[i])];
    ([a, b], grad)
}

fn point_on_extrusion_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    if n_par < 4 || n_const < EXTRUSION_FRAME + 2 {
        return;
    }
    for i in 0..n {
        let (o, ko) = (n_par * i, n_const * i);
        let (place, _) = extrusion_place(&v[o..o + 3], &k[ko..ko + EXTRUSION_FRAME]);
        let c = body_val::<ENVELOPE>(&k[ko + EXTRUSION_FRAME..ko + n_const], v[o + 3], &v[o + 4..o + n_par], 1, false);
        r[2 * i] = place[0] - c.x;
        r[2 * i + 1] = place[1] - c.y;
    }
}

fn point_on_extrusion_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    if n_par < 4 || n_const < EXTRUSION_FRAME + 2 {
        return;
    }
    for i in 0..n {
        let (o, ko) = (n_par * i, n_const * i);
        let (jo, row1) = (2 * n_par * i, 2 * n_par * i + n_par);
        let (_, grad) = extrusion_place(&v[o..o + 3], &k[ko..ko + EXTRUSION_FRAME]);
        let c = body_val::<ENVELOPE>(&k[ko + EXTRUSION_FRAME..ko + n_const], v[o + 3], &v[o + 4..o + n_par], 1, true);
        for t in 0..3 {
            j[jo + t] = grad[0][t];
            j[row1 + t] = grad[1][t];
        }
        // the parameter, then every coordinate the curve reads, as the contact's
        for t in 0..n_par - 3 {
            j[jo + 3 + t] = -c.dx[t];
            j[row1 + 3 + t] = -c.dy[t];
        }
    }
}

/* -- relations in space ------------------------------------------------------
 *
 * What a relation between two views says (`docs/spatial-constraints-plan.md`).  Every one
 * reads the **hidden points** a view point lifts to (`model::LiftE`), three columns each, and
 * never a view's attitude — the `lift` rows are where a drawn point and its place in space are
 * tied together, so a relation here is plain vector algebra over points in space.  Those that
 * read a *plane* rather than points (a point or a line on a plane, the plane row of a circle)
 * read its origin and its two axes' directions (`dframe`), as `lift` does; a plane a `fix`
 * holds is the same kernel with those columns held.
 */

use crate::space::{cross as cross3, dot as dot3, norm as norm3, sub as sub3};

#[inline]
fn at3(v: &[f64], o: usize) -> [f64; 3] {
    [v[o], v[o + 1], v[o + 2]]
}

/// Columns of `coincident3`: (X, Y) — two hidden points.  `X − Y = 0`: three signed
/// displacements, degree 1.  The same point in space, whatever views its two images are in.
fn coincident3_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 6 * i;
        for t in 0..3 {
            r[3 * i + t] = v[o + t] - v[o + 3 + t];
        }
    }
}

fn coincident3_jac(n: usize, _v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let jo = 18 * i;
        j[jo..jo + 18].fill(0.0);
        for t in 0..3 {
            j[jo + 6 * t + t] = 1.0;
            j[jo + 6 * t + 3 + t] = -1.0;
        }
    }
}

/// `|X − Y|²` and its gradient in (X, Y).
#[inline]
fn dist3_sq(v: &[f64], j: &mut [f64]) -> f64 {
    let d = sub3(at3(v, 0), at3(v, 3));
    for t in 0..3 {
        j[t] = 2.0 * d[t];
        j[3 + t] = -2.0 * d[t];
    }
    dot3(d, d)
}

/// Columns of `distance3`: (X, Y), K = (d).  `|X − Y|² − d²` — the true length between two
/// points drawn in different views, `distance`'s form one dimension up: degree 2.
fn distance3_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let mut g = [0.0; 6];
    for i in 0..n {
        r[i] = dist3_sq(&v[6 * i..], &mut g) - k[i] * k[i];
    }
}

fn distance3_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 6 * i;
        dist3_sq(&v[o..], &mut j[o..o + 6]);
    }
}

/// (X, Y, a), K = (m, c): `|X − Y|² − d²`, d = m·a + c.
fn distance3_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let mut g = [0.0; 6];
    for i in 0..n {
        let o = 7 * i;
        let (d, _) = free_dim(v, k, i, o + 6);
        r[i] = dist3_sq(&v[o..], &mut g) - d * d;
    }
}

fn distance3_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 7 * i;
        let (d, m) = free_dim(v, k, i, o + 6);
        dist3_sq(&v[o..], &mut j[o..o + 6]);
        j[o + 6] = -2.0 * d * m;
    }
}

/// How far the hidden point X stands from the infinite line through A and B, and its gradient in
/// (X, A, B): `|(X − A) × (B − A)| / |B − A|`.  A magnitude — in space a point has no side of a
/// line to be on — so the gradient is taken as zero exactly on the line, where the norm has none:
/// "on the line" is two equations, and one row cannot say it (that is `coincident`'s business).
fn point_line3_gap(v: &[f64], j: &mut [f64]) -> f64 {
    let (x, a, b) = (at3(v, 0), at3(v, 3), at3(v, 6));
    let (w, e) = (sub3(x, a), sub3(b, a));
    let c = cross3(w, e);
    let (lc, le) = (norm3(c), norm3(e).max(MIN_LINE_LEN));
    let g = lc / le;
    let ch = if lc > 0.0 { c.map(|t| t / lc) } else { [0.0; 3] };
    // ∂|c|/∂w = e × ĉ and ∂|c|/∂e = ĉ × w, with w = X − A and e = B − A
    let (gw, ce) = (cross3(e, ch), cross3(ch, w));
    for t in 0..3 {
        let gw = gw[t] / le;
        let ge = ce[t] / le - g * e[t] / (le * le);
        j[t] = gw;
        j[3 + t] = -gw - ge;
        j[6 + t] = ge;
    }
    g
}

/// Columns of `point_line3`: (X, A, B), K = (d).  The distance from a point to a line drawn in
/// another view, `|(X − A) × (B − A)| / |B − A| − d`: a length, degree 1.
fn point_line3_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let mut g = [0.0; 9];
    for i in 0..n {
        r[i] = point_line3_gap(&v[9 * i..], &mut g) - k[i];
    }
}

fn point_line3_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        point_line3_gap(&v[o..], &mut j[o..o + 9]);
    }
}

/// (X, A, B, a), K = (m, c): the same, d = m·a + c.
fn point_line3_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let mut g = [0.0; 9];
    for i in 0..n {
        let o = 10 * i;
        r[i] = point_line3_gap(&v[o..], &mut g) - free_dim(v, k, i, o + 9).0;
    }
}

fn point_line3_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 10 * i;
        point_line3_gap(&v[o..], &mut j[o..o + 9]);
        j[o + 9] = -k[2 * i];
    }
}

/// The **signed** common-perpendicular distance between two lines in space and its gradient in
/// (A, B, C, D): `(e₁ × e₂)·(C − A) / |e₁ × e₂|`, e₁ = B − A and e₂ = D − C.  Signed, and not
/// its magnitude, because the magnitude has a crease where the lines meet, which is exactly where
/// a skew distance of zero asks a solve to go; which sign is the constraint's, read off the seed
/// when it was stated and folded into the number it compares against (`Constraint::consts_on`).
/// Parallel lines have no common perpendicular, which is refused where it is stated.
fn skew_gap(v: &[f64], j: &mut [f64]) -> f64 {
    let (a, b, c, d) = (at3(v, 0), at3(v, 3), at3(v, 6), at3(v, 9));
    let (e1, e2, w) = (sub3(b, a), sub3(d, c), sub3(c, a));
    let m = cross3(e1, e2);
    let l = norm3(m).max(MIN_LINE_LEN * MIN_LINE_LEN);
    let mh = m.map(|t| t / l);
    let s = dot3(m, w) / l;
    // s = N/L: ∂N/∂e₁ = e₂ × w, ∂N/∂e₂ = w × e₁, ∂N/∂w = m; ∂L/∂e₁ = e₂ × m̂, ∂L/∂e₂ = m̂ × e₁
    let (n1, l1, n2, l2) = (cross3(e2, w), cross3(e2, mh), cross3(w, e1), cross3(mh, e1));
    for t in 0..3 {
        let g1 = (n1[t] - s * l1[t]) / l;
        let g2 = (n2[t] - s * l2[t]) / l;
        j[t] = -g1 - mh[t];
        j[3 + t] = g1;
        j[6 + t] = -g2 + mh[t];
        j[9 + t] = g2;
    }
    s
}

/// Columns of `line_line3`: (A, B, C, D) — two lines' hidden endpoints — K = (d), the stated
/// distance already turned to the side the seed was on.  `s − d`: a length, degree 1.
fn line_line3_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let mut g = [0.0; 12];
    for i in 0..n {
        r[i] = skew_gap(&v[12 * i..], &mut g) - k[i];
    }
}

fn line_line3_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 12 * i;
        skew_gap(&v[o..], &mut j[o..o + 12]);
    }
}

/// (A, B, C, D, a), K = (m, c): `s − (m·a + c)`, the sign folded into m and c.
fn line_line3_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let mut g = [0.0; 12];
    for i in 0..n {
        let o = 13 * i;
        r[i] = skew_gap(&v[o..], &mut g) - free_dim(v, k, i, o + 12).0;
    }
}

fn line_line3_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 13 * i;
        skew_gap(&v[o..], &mut j[o..o + 12]);
        j[o + 12] = -k[2 * i];
    }
}

/// The cosine of the angle between two lines' directions in space, `â·b̂` with a = B − A and
/// b = D − C, and its gradient in (A, B, C, D).  Directed by each line's p1 → p2, so it runs
/// through the whole half turn and says nothing about a sense: in space there is no side of the
/// page to turn toward.
fn cos3(v: &[f64], j: &mut [f64]) -> f64 {
    let (a, b) = (sub3(at3(v, 3), at3(v, 0)), sub3(at3(v, 9), at3(v, 6)));
    let (la, lb) = (norm3(a).max(MIN_LINE_LEN), norm3(b).max(MIN_LINE_LEN));
    let (ah, bh) = (a.map(|t| t / la), b.map(|t| t / lb));
    let cs = dot3(ah, bh);
    for t in 0..3 {
        let ga = (bh[t] - cs * ah[t]) / la;
        let gb = (ah[t] - cs * bh[t]) / lb;
        j[t] = -ga;
        j[3 + t] = ga;
        j[6 + t] = -gb;
        j[9 + t] = gb;
    }
    cs
}

/// Columns of `angle3`: (A, B, C, D), K = (θ) in radians.  `â·b̂ − cos θ`: the unsigned angle,
/// 0 to half a turn, stated by its cosine — dimensionless, degree 0, `angle`'s rationale.  Its
/// gradient in θ vanishes at 0 and half a turn, where the lines are parallel and an angle
/// between them is better said as `parallel3`.
fn angle3_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let mut g = [0.0; 12];
    for i in 0..n {
        r[i] = cos3(&v[12 * i..], &mut g) - k[i].dcos();
    }
}

fn angle3_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 12 * i;
        cos3(&v[o..], &mut j[o..o + 12]);
    }
}

/// (A, B, C, D, a), K = (m, c): `â·b̂ − cos(m·a + c)`.
fn angle3_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let mut g = [0.0; 12];
    for i in 0..n {
        let o = 13 * i;
        r[i] = cos3(&v[o..], &mut g) - free_dim(v, k, i, o + 12).0.dcos();
    }
}

fn angle3_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 13 * i;
        cos3(&v[o..], &mut j[o..o + 12]);
        let (th, m) = free_dim(v, k, i, o + 12);
        j[o + 12] = th.dsin() * m;
    }
}

/// Columns of `perpendicular3`: (A, B, C, D).  `â·b̂ = 0`, normalised so a long line and a short
/// one weigh alike: degree 0.
fn perpendicular3_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    let mut g = [0.0; 12];
    for i in 0..n {
        r[i] = cos3(&v[12 * i..], &mut g);
    }
}

fn perpendicular3_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 12 * i;
        cos3(&v[o..], &mut j[o..o + 12]);
    }
}

/// Columns of `parallel3`: (A, B, C, D), K = (e₁, e₂) — two unit vectors perpendicular to the
/// first line's direction as it stood when the system was compiled or last refreshed.
///
/// Two lines in space are parallel when `â × b̂ = 0`, which is three rows of rank two: the cross
/// product is perpendicular to a, so it has only two components to lose.  Stating all three
/// would be a dependent row at every solution; stating two fixed components would be singular
/// wherever a turns onto one of them.  So the two are taken *across* a, as constants
/// (`Constraint::consts_on`, re-read by `System::refresh_consts`): `(â × b̂)·e_k = 0`.  Near a
/// solution â × b̂ lies in the plane of e₁ and e₂, and the rows are regular there.  Degree 0.
fn parallel3_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 12 * i;
        let (a, b) = (sub3(at3(v, o + 3), at3(v, o)), sub3(at3(v, o + 9), at3(v, o + 6)));
        let p = norm3(a).max(MIN_LINE_LEN) * norm3(b).max(MIN_LINE_LEN);
        let x = cross3(a, b);
        for t in 0..2 {
            r[2 * i + t] = dot3(x, at3(k, 6 * i + 3 * t)) / p;
        }
    }
}

fn parallel3_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 12 * i;
        let (a, b) = (sub3(at3(v, o + 3), at3(v, o)), sub3(at3(v, o + 9), at3(v, o + 6)));
        let (la, lb) = (norm3(a).max(MIN_LINE_LEN), norm3(b).max(MIN_LINE_LEN));
        let p = la * lb;
        let x = cross3(a, b);
        for t in 0..2 {
            let e = at3(k, 6 * i + 3 * t);
            let r = dot3(x, e) / p;
            // N = (a × b)·e: ∂N/∂a = b × e, ∂N/∂b = e × a; P = |a||b|
            let (na, nb) = (cross3(b, e), cross3(e, a));
            let row = &mut j[24 * i + 12 * t..24 * i + 12 * (t + 1)];
            for s in 0..3 {
                let ga = na[s] / p - r * a[s] / (la * la);
                let gb = nb[s] / p - r * b[s] / (lb * lb);
                row[s] = -ga;
                row[3 + s] = ga;
                row[6 + s] = -gb;
                row[9 + s] = gb;
            }
        }
    }
}

/* -- the rest of the spatial words --------------------------------------------------------------
 *
 * A point on a line in space, two lines of equal true length, and a point's signed distance from
 * a plane (`distance(along: n)`). */

/// Columns of `axis_unit`: (dx, dy, dz).  `|d|² − 1`: a direction held to the unit sphere,
/// dimensionless, degree 0.
fn axis_unit_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let d = at3(v, 3 * i);
        r[i] = dot3(d, d) - 1.0;
    }
}

fn axis_unit_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 3 * i;
        for t in 0..3 {
            j[o + t] = 2.0 * v[o + t];
        }
    }
}

/// Columns of `axis_foot`: (a, d).  `a·d`: the axis's point is the foot of the perpendicular from
/// the origin, so it cannot slide along the axis.  A length, degree 1.
fn axis_foot_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        r[i] = dot3(at3(v, 6 * i), at3(v, 6 * i + 3));
    }
}

fn axis_foot_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 6 * i;
        for t in 0..3 {
            j[o + t] = v[o + 3 + t];
            j[o + 3 + t] = v[o + t];
        }
    }
}

/// Columns of `point_on_axis`: (X, a, d), K = (e₁, e₂) — `point_on_line3` over an axis, whose
/// direction is `d` itself rather than `B − A`: `((X − a) × d)·e_k / |d| = 0`.  Degree 1.
fn point_on_axis_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        let (w, e) = (sub3(at3(v, o), at3(v, o + 3)), at3(v, o + 6));
        let le = norm3(e).max(MIN_LINE_LEN);
        let c = cross3(w, e);
        for t in 0..2 {
            r[2 * i + t] = dot3(c, at3(k, 6 * i + 3 * t)) / le;
        }
    }
}

fn point_on_axis_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        let (w, e) = (sub3(at3(v, o), at3(v, o + 3)), at3(v, o + 6));
        let le = norm3(e).max(MIN_LINE_LEN);
        let c = cross3(w, e);
        for t in 0..2 {
            let kk = at3(k, 6 * i + 3 * t);
            let r = dot3(c, kk) / le;
            // N = (w × e)·k = w·(e × k) = e·(k × w)
            let (gw, ge) = (cross3(e, kk), cross3(kk, w));
            let row = &mut j[18 * i + 9 * t..18 * i + 9 * (t + 1)];
            for s in 0..3 {
                let gw = gw[s] / le;
                row[s] = gw;
                row[3 + s] = -gw;
                row[6 + s] = ge[s] / le - r * e[s] / (le * le);
            }
        }
    }
}

/// Columns of `point_on_line3`: (X, A, B), K = (e₁, e₂) — two unit vectors across the line as it
/// stood when the system was compiled or last refreshed (`parallel3`'s device).  "On the line"
/// is two equations, and the magnitude `|w × e|/|e|` has no gradient where it holds, so the two
/// are stated as components: `((X − A) × (B − A))·e_k / |B − A| = 0`.  Degree 1.
fn point_on_line3_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        let (w, e) = (sub3(at3(v, o), at3(v, o + 3)), sub3(at3(v, o + 6), at3(v, o + 3)));
        let le = norm3(e).max(MIN_LINE_LEN);
        let c = cross3(w, e);
        for t in 0..2 {
            r[2 * i + t] = dot3(c, at3(k, 6 * i + 3 * t)) / le;
        }
    }
}

fn point_on_line3_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        let (w, e) = (sub3(at3(v, o), at3(v, o + 3)), sub3(at3(v, o + 6), at3(v, o + 3)));
        let le = norm3(e).max(MIN_LINE_LEN);
        let c = cross3(w, e);
        for t in 0..2 {
            let kk = at3(k, 6 * i + 3 * t);
            let r = dot3(c, kk) / le;
            // N = (w × e)·k = w·(e × k) = e·(k × w)
            let (gw, ge) = (cross3(e, kk), cross3(kk, w));
            let row = &mut j[18 * i + 9 * t..18 * i + 9 * (t + 1)];
            for s in 0..3 {
                let gw = gw[s] / le;
                let ge = ge[s] / le - r * e[s] / (le * le);
                row[s] = gw;
                row[3 + s] = -gw - ge;
                row[6 + s] = ge;
            }
        }
    }
}

/// Columns of `equal_length3`: (A, B, C, D).  `|B − A|² − |D − C|²`, the true lengths of two
/// lines drawn in different views; degree 2, `equal_length`'s form one dimension up.
fn equal_length3_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 12 * i;
        let (a, b) = (sub3(at3(v, o + 3), at3(v, o)), sub3(at3(v, o + 9), at3(v, o + 6)));
        r[i] = dot3(a, a) - dot3(b, b);
    }
}

fn equal_length3_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 12 * i;
        let (a, b) = (sub3(at3(v, o + 3), at3(v, o)), sub3(at3(v, o + 9), at3(v, o + 6)));
        let row = &mut j[o..o + 12];
        for s in 0..3 {
            row[s] = -2.0 * a[s];
            row[3 + s] = 2.0 * a[s];
            row[6 + s] = 2.0 * b[s];
            row[9 + s] = -2.0 * b[s];
        }
    }
}

/* -- the midpoint and the mirror in space ----------------------------------------------------- */

/// Columns of `midpoint3`: (X, A, B).  `X − (A + B)/2`, three rows, degree 1 — a point drawn in
/// one view the midpoint of a line drawn in another, in space.
fn midpoint3_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        for t in 0..3 {
            r[3 * i + t] = v[o + t] - 0.5 * (v[o + 3 + t] + v[o + 6 + t]);
        }
    }
}

fn midpoint3_jac(n: usize, _v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let jo = 27 * i;
        j[jo..jo + 27].fill(0.0);
        for t in 0..3 {
            j[jo + 9 * t + t] = 1.0;
            j[jo + 9 * t + 3 + t] = -0.5;
            j[jo + 9 * t + 6 + t] = -0.5;
        }
    }
}

/// Columns of `symmetric3`: (P, Q, A, B).  `Q + P − 2F`, F the foot of P on the line through A
/// and B: Q is P turned half way round the line, which in the plane of the page is its mirror in
/// it.  Three rows, degree 1, a unit gradient in Q.
fn symmetric3_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 12 * i;
        let (p, q, a, b) = (at3(v, o), at3(v, o + 3), at3(v, o + 6), at3(v, o + 9));
        let e = sub3(b, a);
        let le = norm3(e);
        if le == 0.0 {
            r[3 * i..3 * i + 3].fill(f64::NAN);
            continue;
        }
        let eh = e.map(|t| t / le);
        let s = dot3(sub3(p, a), eh);
        for t in 0..3 {
            r[3 * i + t] = q[t] + p[t] - 2.0 * a[t] - 2.0 * s * eh[t];
        }
    }
}

fn symmetric3_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 12 * i;
        let jo = 36 * i;
        let (p, a, b) = (at3(v, o), at3(v, o + 6), at3(v, o + 9));
        let e = sub3(b, a);
        let le = norm3(e);
        if le == 0.0 {
            j[jo..jo + 36].fill(f64::NAN);
            continue;
        }
        let eh = e.map(|t| t / le);
        let w = sub3(p, a);
        let s = dot3(w, eh);
        // d(s ê) = ê êᵀ dw + (ê wᵀ + s I) P de / |e|, P = I − ê êᵀ; G = (ê wᵀ + s I) P / |e|
        let proj = |x: usize, y: usize| (if x == y { 1.0 } else { 0.0 }) - eh[x] * eh[y];
        let g = |x: usize, y: usize| -> f64 {
            (0..3).map(|z| (eh[x] * w[z] + if x == z { s } else { 0.0 }) * proj(z, y)).sum::<f64>() / le
        };
        for x in 0..3 {
            let row = &mut j[jo + 12 * x..jo + 12 * x + 12];
            for y in 0..3 {
                let id = if x == y { 1.0 } else { 0.0 };
                let ee = eh[x] * eh[y];
                let gxy = g(x, y);
                row[y] = id - 2.0 * ee;
                row[3 + y] = id;
                row[6 + y] = -2.0 * id + 2.0 * ee + 2.0 * gxy;
                row[9 + y] = -2.0 * gxy;
            }
        }
    }
}

/* -- forward-mode derivatives -----------------------------------------------------------------
 *
 * The kernels below take their derivatives by `Dual`, a forward-mode number carrying the gradient
 * in every column of the block — exact, and one expression for the residual and its row, where
 * the hand-derived chains of the kernels above would be a page of vector calculus to get wrong. */

/// A number and its gradient in the `N` columns of one block.
#[derive(Clone, Copy)]
struct Dual<const N: usize> {
    v: f64,
    g: [f64; N],
}

impl<const N: usize> Dual<N> {
    fn var(v: f64, i: usize) -> Self {
        let mut g = [0.0; N];
        g[i] = 1.0;
        Dual { v, g }
    }
    fn map(self, v: f64, d: f64) -> Self {
        Dual { v, g: self.g.map(|x| x * d) }
    }
    fn sqrt(self) -> Self {
        let s = self.v.max(0.0).sqrt();
        // no gradient where the root is zero: a caller asking there is on a degenerate figure
        self.map(s, if s > 0.0 { 0.5 / s } else { 0.0 })
    }
}

impl<const N: usize> std::ops::Add for Dual<N> {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        let mut g = self.g;
        for (a, b) in g.iter_mut().zip(o.g) {
            *a += b;
        }
        Dual { v: self.v + o.v, g }
    }
}

impl<const N: usize> std::ops::Sub for Dual<N> {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        let mut g = self.g;
        for (a, b) in g.iter_mut().zip(o.g) {
            *a -= b;
        }
        Dual { v: self.v - o.v, g }
    }
}

impl<const N: usize> std::ops::Mul for Dual<N> {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        let mut g = [0.0; N];
        for k in 0..N {
            g[k] = self.g[k] * o.v + self.v * o.g[k];
        }
        Dual { v: self.v * o.v, g }
    }
}

impl<const N: usize> std::ops::Div for Dual<N> {
    type Output = Self;
    fn div(self, o: Self) -> Self {
        let q = self.v / o.v;
        let mut g = [0.0; N];
        for k in 0..N {
            g[k] = (self.g[k] - q * o.g[k]) / o.v;
        }
        Dual { v: q, g }
    }
}

/// A number a kernel written once is read over: a `Dual` for its residual and its row, a
/// Taylor `Jet` for its form (`taylor.rs`), so the form and the kernel are one expression.
pub(crate) trait Num:
    Copy
    + std::ops::Add<Output = Self>
    + std::ops::Sub<Output = Self>
    + std::ops::Mul<Output = Self>
    + std::ops::Div<Output = Self>
{
    /// A constant.
    fn cst(v: f64) -> Self;
    /// The value, where a branch reads it.
    fn val(self) -> f64;
    /// The root, its derivatives taken as zero where it is zero.
    fn sqrt(self) -> Self;
    fn cos(self) -> Self;
}

impl<const N: usize> Num for Dual<N> {
    fn cst(v: f64) -> Self {
        Dual { v, g: [0.0; N] }
    }
    fn val(self) -> f64 {
        self.v
    }
    fn sqrt(self) -> Self {
        Dual::sqrt(self)
    }
    fn cos(self) -> Self {
        self.map(self.v.dcos(), -self.v.dsin())
    }
}

impl Num for Jet {
    fn cst(v: f64) -> Self {
        Jet::constant(v)
    }
    fn val(self) -> f64 {
        self.0[0]
    }
    fn sqrt(self) -> Self {
        if self.0[0] > 0.0 { Jet::sqrt(self) } else { Self::cst(0.0) }
    }
    fn cos(self) -> Self {
        self.sin_cos().1
    }
}

pub(crate) type V3<T> = [T; 3];

/// Columns `at..at + 3` as a vector.
fn vec3<T: Num>(v: &[T], at: usize) -> V3<T> {
    [v[at], v[at + 1], v[at + 2]]
}

fn vsub<T: Num>(a: V3<T>, b: V3<T>) -> V3<T> {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn vdot<T: Num>(a: V3<T>, b: V3<T>) -> T {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn vcross<T: Num>(a: V3<T>, b: V3<T>) -> V3<T> {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// The unit vector along `a`; a vector shorter than `MIN_LINE_LEN` is divided by that instead,
/// as every kernel above guards a degenerate line.
fn vunit<T: Num>(a: V3<T>) -> V3<T> {
    let l = vlen(a, MIN_LINE_LEN);
    a.map(|x| x / l)
}

/// `|a|`, floored as the kernels floor a line's length.
fn vlen<T: Num>(a: V3<T>, floor: f64) -> T {
    let l = vdot(a, a).sqrt();
    if l.val() > floor { l } else { T::cst(floor) }
}

/// The free twin's number `m·a + c`, from its column and constants (m, c).
fn free_num<T: Num>(a: T, k: &[f64]) -> T {
    a * T::cst(k[0]) + T::cst(k[1])
}

/* -- planes over axes (`docs/planes-plan.md`) ------------------------------------------------
 *
 * A plane is two axes and an origin: right along the first axis's direction `du`, up along what
 * is left of the second's `dv` once its component along `du` is removed, out along their cross
 * product, standing at `o`.  Every kernel that reads a plane reads those nine numbers as columns
 * — held ones drop out of the Jacobian, so one kernel serves a plane a `fix` holds and one a
 * solve turns — and takes its derivatives by `Dual`, since the frame's are a page of vector
 * calculus to get wrong.
 */

/// The frame of a plane over its axes: `û` along `du`, `v̂` what is left of `dv`, `n̂ = û × v̂`.
fn frame<T: Num>(du: V3<T>, dv: V3<T>) -> (V3<T>, V3<T>, V3<T>) {
    let u = vunit(du);
    let a = vdot(dv, u);
    let v = vunit(vsub(dv, u.map(|t| t * a)));
    let n = vcross(u, v);
    (u, v, n)
}

/// Three constants from `k[at..at + 3]`, a vector with no gradient.
fn cst3<T: Num>(k: &[f64], at: usize) -> V3<T> {
    [T::cst(k[at]), T::cst(k[at + 1]), T::cst(k[at + 2])]
}

/// One instance's columns, each a variable of the block.
fn seeded<const N: usize>(v: &[f64]) -> [Dual<N>; N] {
    std::array::from_fn(|c| Dual::var(v[c], c))
}

/// The values of a kernel written once over `Dual`s, `R` rows of `N` columns and `C` constants
/// each.
fn dual_res<const N: usize, const R: usize, const C: usize>(
    n: usize,
    v: &[f64],
    k: &[f64],
    r: &mut [f64],
    f: fn(&[Dual<N>], &[f64]) -> [Dual<N>; R],
) {
    for i in 0..n {
        let rows = f(&seeded(&v[N * i..N * (i + 1)]), &k[C * i..C * (i + 1)]);
        for t in 0..R {
            r[R * i + t] = rows[t].v;
        }
    }
}

/// The same kernel's rows of the Jacobian.
fn dual_jac<const N: usize, const R: usize, const C: usize>(
    n: usize,
    v: &[f64],
    k: &[f64],
    j: &mut [f64],
    f: fn(&[Dual<N>], &[f64]) -> [Dual<N>; R],
) {
    for i in 0..n {
        let rows = f(&seeded(&v[N * i..N * (i + 1)]), &k[C * i..C * (i + 1)]);
        for t in 0..R {
            j[(R * i + t) * N..(R * i + t + 1) * N].copy_from_slice(&rows[t].g);
        }
    }
}

/// Columns of `lift`: (X, p, o, du, dv) — a hidden point, the point it lifts as drawn in its
/// plane, and the plane.  `X − (o + p.x·û + p.y·v̂)`: three rows over the hidden point's three
/// Params, so net nothing.  Degree 1.
fn lift_rows<T: Num>(v: &[T], _k: &[f64]) -> [T; 3] {
    let x = vec3(v, 0);
    let (a, b) = (v[3], v[4]);
    let o = vec3(v, 5);
    let (u, w, _) = frame(vec3(v, 8), vec3(v, 11));
    [0, 1, 2].map(|t| x[t] - (o[t] + a * u[t] + b * w[t]))
}

fn lift_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<14, 3, 0>(n, v, k, r, lift_rows)
}

fn lift_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<14, 3, 0>(n, v, k, j, lift_rows)
}

/// Columns of `point_on_plane`: (X, o, du, dv).  `(X − o)·n̂`: a point on a plane in space.
fn point_on_plane_rows<T: Num>(v: &[T], _k: &[f64]) -> [T; 1] {
    let (_, _, n) = frame(vec3(v, 6), vec3(v, 9));
    [vdot(vsub(vec3(v, 0), vec3(v, 3)), n)]
}

fn point_on_plane_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<12, 1, 0>(n, v, k, r, point_on_plane_rows)
}

fn point_on_plane_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<12, 1, 0>(n, v, k, j, point_on_plane_rows)
}

/// Columns of `line_on_plane`: (A, B, o, du, dv).  Both ends on the plane: two rows.
fn line_on_plane_rows<T: Num>(v: &[T], _k: &[f64]) -> [T; 2] {
    let o = vec3(v, 6);
    let (_, _, n) = frame(vec3(v, 9), vec3(v, 12));
    [vdot(vsub(vec3(v, 0), o), n), vdot(vsub(vec3(v, 3), o), n)]
}

fn line_on_plane_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<15, 2, 0>(n, v, k, r, line_on_plane_rows)
}

fn line_on_plane_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<15, 2, 0>(n, v, k, j, line_on_plane_rows)
}

/// Columns of `point_on_circle3`: (X, C, r, du, dv) — a hidden point, a circle's centre lifted
/// and its radius, and the plane it is drawn in.  `|X − C| − r` and `n̂·(X − C)`: on the sphere
/// of the circle's radius about its centre, and on its plane.  The radius row is the magnitude
/// form, a length like the plane row.  Both degree 1.
fn point_on_circle3_rows<T: Num>(v: &[T], _k: &[f64]) -> [T; 2] {
    let d = vsub(vec3(v, 0), vec3(v, 3));
    let (_, _, n) = frame(vec3(v, 7), vec3(v, 10));
    [vdot(d, d).sqrt() - v[6], vdot(n, d)]
}

fn point_on_circle3_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<13, 2, 0>(n, v, k, r, point_on_circle3_rows)
}

fn point_on_circle3_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<13, 2, 0>(n, v, k, j, point_on_circle3_rows)
}

/// Columns of `project_solved`: (X_A, X_B, du_A, dv_A, du_B, dv_B) — two images' hidden points
/// and their planes.  `(n̂_A × n̂_B)·(X_A − X_B)`: the projector rule in space, the two images on
/// one line square to the fold.  Degree 1.
fn project_solved_rows<T: Num>(v: &[T], _k: &[f64]) -> [T; 1] {
    let (_, _, na) = frame(vec3(v, 6), vec3(v, 9));
    let (_, _, nb) = frame(vec3(v, 12), vec3(v, 15));
    [vdot(vcross(na, nb), vsub(vec3(v, 0), vec3(v, 3)))]
}

fn project_solved_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<18, 1, 0>(n, v, k, r, project_solved_rows)
}

fn project_solved_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<18, 1, 0>(n, v, k, j, project_solved_rows)
}

/// Columns of `project`: (p, q) — two images drawn in two fixed planes — K = (d_A, d_B, k), the
/// fold line in each plane's coordinates and `d·(o_A − o_B)` (`plane::fold_line`,
/// `plane::fold_offset`).  `d_A·p − d_B·q + k`: one row, linear, degree 1.
fn project_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let (o, c) = (4 * i, 5 * i);
        r[i] = k[c] * v[o] + k[c + 1] * v[o + 1] - k[c + 2] * v[o + 2] - k[c + 3] * v[o + 3] + k[c + 4];
    }
}

fn project_jac(n: usize, _v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let c = 5 * i;
        j[4 * i..4 * i + 4].copy_from_slice(&[k[c], k[c + 1], -k[c + 2], -k[c + 3]]);
    }
}

/* -- ordinates (`docs/ordinate-plan.md`) ------------------------------------------------------
 *
 * How far `q` stands from `p` along a directed line, `(q − p)·t̂ − D`: degree 1, and linear in the
 * points.  The run and the rise are its form along a view's own axes (`ordinate_u`/`_v`, above);
 * these read a direction that is not one: a line drawn in the points' view, a line or an axis in
 * space (an axis handed over as the segment from the origin to its direction, as the direction
 * relations take one), and a plane's own frame from its origin.  Each has its free twin, the
 * number `m·a + c`.
 */

/// Columns of `ordinate_line`: (p, q, a, b) in one view, K = (D).  `(q − p)·(b − a)/|b − a| − D`:
/// along the line `a → b` drawn where the points are.
fn ordinate_line_gap<T: Num>(v: &[T]) -> T {
    let (px, py, qx, qy) = (v[0], v[1], v[2], v[3]);
    let (ax, ay, bx, by) = (v[4], v[5], v[6], v[7]);
    let t = vunit([bx - ax, by - ay, T::cst(0.0)]);
    (qx - px) * t[0] + (qy - py) * t[1]
}

fn ordinate_line_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    [ordinate_line_gap(v) - T::cst(k[0])]
}

/// The free twin's columns: (p, q, a, b, n), K = (m, c) — the number `m·n + c`.
fn ordinate_line_free_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    [ordinate_line_gap(v) - free_num(v[8], k)]
}

fn ordinate_line_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<8, 1, 1>(n, v, k, r, ordinate_line_rows)
}

fn ordinate_line_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<8, 1, 1>(n, v, k, j, ordinate_line_rows)
}

fn ordinate_line_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<9, 1, 2>(n, v, k, r, ordinate_line_free_rows)
}

fn ordinate_line_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<9, 1, 2>(n, v, k, j, ordinate_line_free_rows)
}

/// Columns of `ordinate_space`: (X, Y, A, B), K = (D).  `(Y − X)·(B − A)/|B − A| − D`: how far
/// `Y` stands from `X` along the line `A → B` in space — a drawn line's lifted ends, or an axis
/// as the segment from the origin to its direction.
fn ordinate_space_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    let t = vunit(vsub(vec3(v, 9), vec3(v, 6)));
    [vdot(vsub(vec3(v, 3), vec3(v, 0)), t) - T::cst(k[0])]
}

/// (X, Y, A, B, n), K = (m, c).
fn ordinate_space_free_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    let t = vunit(vsub(vec3(v, 9), vec3(v, 6)));
    let d = free_num(v[12], k);
    [vdot(vsub(vec3(v, 3), vec3(v, 0)), t) - d]
}

/// Columns of `ordinate_frame_u`: (X, o, du, dv), K = (D).  `(X − o)·û − D`: how far a point
/// stands from a plane's origin along the plane's `û` — `q distance(d, along: u) P` with `q` drawn
/// in no plane of `P`'s — and `_v` along its `v̂`, `_n` along its normal `n̂`.  Over the plane's
/// own columns, so its frame is the one its points are drawn in (`dframe`).
fn ordinate_frame_rows<T: Num>(v: &[T], k: &[f64], axis: usize) -> [T; 1] {
    let (u, w, n) = frame(vec3(v, 6), vec3(v, 9));
    let t = [u, w, n][axis];
    [vdot(vsub(vec3(v, 0), vec3(v, 3)), t) - T::cst(k[0])]
}

fn ordinate_frame_u_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    ordinate_frame_rows(v, k, 0)
}

fn ordinate_frame_v_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    ordinate_frame_rows(v, k, 1)
}

fn ordinate_frame_n_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    ordinate_frame_rows(v, k, 2)
}

/// (X, o, du, dv, a), K = (m, c): the number `m·a + c`.
fn ordinate_frame_free_rows<T: Num>(v: &[T], k: &[f64], axis: usize) -> [T; 1] {
    let (u, w, n) = frame(vec3(v, 6), vec3(v, 9));
    let t = [u, w, n][axis];
    let d = free_num(v[12], k);
    [vdot(vsub(vec3(v, 0), vec3(v, 3)), t) - d]
}

fn ordinate_frame_u_free_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    ordinate_frame_free_rows(v, k, 0)
}

fn ordinate_frame_v_free_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    ordinate_frame_free_rows(v, k, 1)
}

fn ordinate_frame_n_free_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    ordinate_frame_free_rows(v, k, 2)
}

fn ordinate_space_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<12, 1, 1>(n, v, k, r, ordinate_space_rows)
}

fn ordinate_space_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<12, 1, 1>(n, v, k, j, ordinate_space_rows)
}

fn ordinate_space_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<13, 1, 2>(n, v, k, r, ordinate_space_free_rows)
}

fn ordinate_space_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<13, 1, 2>(n, v, k, j, ordinate_space_free_rows)
}

fn ordinate_frame_u_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<12, 1, 1>(n, v, k, r, ordinate_frame_u_rows)
}

fn ordinate_frame_u_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<12, 1, 1>(n, v, k, j, ordinate_frame_u_rows)
}

fn ordinate_frame_v_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<12, 1, 1>(n, v, k, r, ordinate_frame_v_rows)
}

fn ordinate_frame_v_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<12, 1, 1>(n, v, k, j, ordinate_frame_v_rows)
}

fn ordinate_frame_n_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<12, 1, 1>(n, v, k, r, ordinate_frame_n_rows)
}

fn ordinate_frame_n_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<12, 1, 1>(n, v, k, j, ordinate_frame_n_rows)
}

fn ordinate_frame_u_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<13, 1, 2>(n, v, k, r, ordinate_frame_u_free_rows)
}

fn ordinate_frame_u_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<13, 1, 2>(n, v, k, j, ordinate_frame_u_free_rows)
}

fn ordinate_frame_v_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<13, 1, 2>(n, v, k, r, ordinate_frame_v_free_rows)
}

fn ordinate_frame_v_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<13, 1, 2>(n, v, k, j, ordinate_frame_v_free_rows)
}

fn ordinate_frame_n_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<13, 1, 2>(n, v, k, r, ordinate_frame_n_free_rows)
}

fn ordinate_frame_n_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<13, 1, 2>(n, v, k, j, ordinate_frame_n_free_rows)
}

/// Columns of `axis_on_plane`: (a, d, o, du, dv), K = (L) — an axis's place and direction, and the
/// plane.  `(a − o)·n̂` and `(a + L·d − o)·n̂`: two points on the axis a drawing's extent `L` apart,
/// each on the plane, so both rows are lengths and weigh alike.  Degree 1.
fn axis_on_plane_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 2] {
    let (a, d, o) = (vec3(v, 0), vec3(v, 3), vec3(v, 6));
    let (_, _, n) = frame(vec3(v, 9), vec3(v, 12));
    let l = T::cst(k[0]);
    let far = [a[0] + d[0] * l, a[1] + d[1] * l, a[2] + d[2] * l];
    [vdot(vsub(a, o), n), vdot(vsub(far, o), n)]
}

fn axis_on_plane_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<15, 2, 1>(n, v, k, r, axis_on_plane_rows)
}

fn axis_on_plane_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<15, 2, 1>(n, v, k, j, axis_on_plane_rows)
}

/// Columns of `axis_coincident`: (a₁, d₁, a₂, d₂), K = (e₁, e₂, L) — two directions across the
/// first axis's as it stood when compiled or refreshed, and a drawing's extent.  One line, either
/// way: `L·(d̂₂ × d̂₁)·e_k`, the second running along the first (the sense free, as `parallel3`'s),
/// and `((a₂ − a₁) × d̂₁)·e_k`, the second's place on the first.  All four lengths.  Degree 1.
fn axis_coincident_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 4] {
    let (a1, d1, a2, d2) = (vec3(v, 0), vunit(vec3(v, 3)), vec3(v, 6), vunit(vec3(v, 9)));
    let e = |t: usize| cst3(k, 3 * t);
    let l = T::cst(k[6]);
    let along = vcross(d2, d1);
    let off = vcross(vsub(a2, a1), d1);
    [vdot(along, e(0)) * l, vdot(along, e(1)) * l, vdot(off, e(0)), vdot(off, e(1))]
}

fn axis_coincident_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<12, 4, 7>(n, v, k, r, axis_coincident_rows)
}

fn axis_coincident_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<12, 4, 7>(n, v, k, j, axis_coincident_rows)
}

/// Columns of `line_on_axis`: (A, B, a, d), K = (e₁, e₂) — two directions across the axis as it
/// stood when compiled or refreshed.  Both ends on the axis's line, `point_on_axis`'s two rows
/// for each: `((X − a) × d̂)·e_k`.  A line of no length is still on it.  Degree 1.
fn line_on_axis_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 4] {
    let (p, q, a, d) = (vec3(v, 0), vec3(v, 3), vec3(v, 6), vunit(vec3(v, 9)));
    let e = |t: usize| cst3(k, 3 * t);
    let (wp, wq) = (vcross(vsub(p, a), d), vcross(vsub(q, a), d));
    [vdot(wp, e(0)), vdot(wp, e(1)), vdot(wq, e(0)), vdot(wq, e(1))]
}

fn line_on_axis_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<12, 4, 6>(n, v, k, r, line_on_axis_rows)
}

fn line_on_axis_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<12, 4, 6>(n, v, k, j, line_on_axis_rows)
}

/// Columns of `axis_parallel_plane`: (d, du, dv).  `d̂·n̂`: square to the plane's normal.  Degree 0.
fn axis_parallel_plane_rows<T: Num>(v: &[T], _k: &[f64]) -> [T; 1] {
    let (_, _, n) = frame(vec3(v, 3), vec3(v, 6));
    [vdot(vunit(vec3(v, 0)), n)]
}

fn axis_parallel_plane_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<9, 1, 0>(n, v, k, r, axis_parallel_plane_rows)
}

fn axis_parallel_plane_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<9, 1, 0>(n, v, k, j, axis_parallel_plane_rows)
}

/// Columns of `axis_perpendicular_plane`: (d, du, dv), K = (e₁, e₂) — two directions across the
/// plane's normal as it stood when compiled or refreshed, `parallel3`'s device.
/// `(d̂ × n̂)·e_k`: along the normal, either way.  Degree 0.
fn axis_perpendicular_plane_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 2] {
    let (_, _, n) = frame(vec3(v, 3), vec3(v, 6));
    let x = vcross(vunit(vec3(v, 0)), n);
    let e = |t: usize| cst3(k, 3 * t);
    [vdot(x, e(0)), vdot(x, e(1))]
}

fn axis_perpendicular_plane_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<9, 2, 6>(n, v, k, r, axis_perpendicular_plane_rows)
}

fn axis_perpendicular_plane_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<9, 2, 6>(n, v, k, j, axis_perpendicular_plane_rows)
}

/// Columns of `plane_parallel`: (du_P, dv_P, du_Q, dv_Q), K = (e₁, e₂), two directions across
/// `P`'s normal as it stood when compiled or refreshed.  `(n̂_Q × n̂_P)·e_k`: the normals alike,
/// either way.  Degree 0.
fn plane_parallel_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 2] {
    let (_, _, np) = frame(vec3(v, 0), vec3(v, 3));
    let (_, _, nq) = frame(vec3(v, 6), vec3(v, 9));
    let x = vcross(nq, np);
    let e = |t: usize| cst3(k, 3 * t);
    [vdot(x, e(0)), vdot(x, e(1))]
}

fn plane_parallel_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<12, 2, 6>(n, v, k, r, plane_parallel_rows)
}

fn plane_parallel_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<12, 2, 6>(n, v, k, j, plane_parallel_rows)
}

/// Columns of `plane_distance`: (o_P, du_P, dv_P, o_Q), K = (D).  `(o_Q − o_P)·n̂_P − D`: how far
/// `Q`'s origin stands along `P`'s normal.  Degree 1.
fn plane_distance_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    let (_, _, n) = frame(vec3(v, 3), vec3(v, 6));
    [vdot(vsub(vec3(v, 9), vec3(v, 0)), n) - T::cst(k[0])]
}

fn plane_distance_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<12, 1, 1>(n, v, k, r, plane_distance_rows)
}

fn plane_distance_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<12, 1, 1>(n, v, k, j, plane_distance_rows)
}

/// (o_P, du_P, dv_P, o_Q, a), K = (m, c): the gap `m·a + c`.
fn plane_distance_free_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    let (_, _, n) = frame(vec3(v, 3), vec3(v, 6));
    let d = free_num(v[12], k);
    [vdot(vsub(vec3(v, 9), vec3(v, 0)), n) - d]
}

fn plane_distance_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    dual_res::<13, 1, 2>(n, v, k, r, plane_distance_free_rows)
}

fn plane_distance_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    dual_jac::<13, 1, 2>(n, v, k, j, plane_distance_free_rows)
}

/* -- the hand-written kernels in space, read over `Num` --------------------------------------
 *
 * The kernels above whose rows were derived by hand keep their `f64` code; each is written again
 * here over `Num`, once, as the Taylor form a body's derivative row reads (`taylor.rs`,
 * `kernels::dual_kernel`).  `tests/taylor.rs` holds every form to its kernel. */

fn distance3_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    let d = vsub(vec3(v, 0), vec3(v, 3));
    [vdot(d, d) - T::cst(k[0] * k[0])]
}

fn distance3_free_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    let d = vsub(vec3(v, 0), vec3(v, 3));
    let n = free_num(v[6], k);
    [vdot(d, d) - n * n]
}

fn point_line3_gap_of<T: Num>(v: &[T]) -> T {
    let (w, e) = (vsub(vec3(v, 0), vec3(v, 3)), vsub(vec3(v, 6), vec3(v, 3)));
    vlen(vcross(w, e), 0.0) / vlen(e, MIN_LINE_LEN)
}

fn point_line3_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    [point_line3_gap_of(v) - T::cst(k[0])]
}

fn point_line3_free_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    [point_line3_gap_of(v) - free_num(v[9], k)]
}

fn skew_gap_of<T: Num>(v: &[T]) -> T {
    let (a, b, c, d) = (vec3(v, 0), vec3(v, 3), vec3(v, 6), vec3(v, 9));
    let m = vcross(vsub(b, a), vsub(d, c));
    vdot(m, vsub(c, a)) / vlen(m, MIN_LINE_LEN * MIN_LINE_LEN)
}

fn line_line3_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    [skew_gap_of(v) - T::cst(k[0])]
}

fn line_line3_free_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    [skew_gap_of(v) - free_num(v[12], k)]
}

/// The two lines' directions, (B − A, D − C).
fn two_dirs<T: Num>(v: &[T]) -> (V3<T>, V3<T>) {
    (vsub(vec3(v, 3), vec3(v, 0)), vsub(vec3(v, 9), vec3(v, 6)))
}

fn cos3_of<T: Num>(v: &[T]) -> T {
    let (a, b) = two_dirs(v);
    vdot(a, b) / (vlen(a, MIN_LINE_LEN) * vlen(b, MIN_LINE_LEN))
}

fn angle3_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    [cos3_of(v) - T::cst(k[0].dcos())]
}

fn angle3_free_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 1] {
    [cos3_of(v) - free_num(v[12], k).cos()]
}

fn perpendicular3_rows<T: Num>(v: &[T], _k: &[f64]) -> [T; 1] {
    [cos3_of(v)]
}

fn parallel3_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 2] {
    let (a, b) = two_dirs(v);
    let p = vlen(a, MIN_LINE_LEN) * vlen(b, MIN_LINE_LEN);
    let x = vcross(a, b);
    [vdot(x, cst3(k, 0)) / p, vdot(x, cst3(k, 3)) / p]
}

fn axis_unit_rows<T: Num>(v: &[T], _k: &[f64]) -> [T; 1] {
    let d = vec3(v, 0);
    [vdot(d, d) - T::cst(1.0)]
}

fn axis_foot_rows<T: Num>(v: &[T], _k: &[f64]) -> [T; 1] {
    [vdot(vec3(v, 0), vec3(v, 3))]
}

/// `((X − a) × e)·e_k / |e|`, the two components across a line of a point on it.
fn across<T: Num>(w: V3<T>, e: V3<T>, k: &[f64]) -> [T; 2] {
    let le = vlen(e, MIN_LINE_LEN);
    let c = vcross(w, e);
    [vdot(c, cst3(k, 0)) / le, vdot(c, cst3(k, 3)) / le]
}

fn point_on_axis_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 2] {
    across(vsub(vec3(v, 0), vec3(v, 3)), vec3(v, 6), k)
}

fn point_on_line3_rows<T: Num>(v: &[T], k: &[f64]) -> [T; 2] {
    across(vsub(vec3(v, 0), vec3(v, 3)), vsub(vec3(v, 6), vec3(v, 3)), k)
}

fn equal_length3_rows<T: Num>(v: &[T], _k: &[f64]) -> [T; 1] {
    let (a, b) = two_dirs(v);
    [vdot(a, a) - vdot(b, b)]
}

fn symmetric3_rows<T: Num>(v: &[T], _k: &[f64]) -> [T; 3] {
    let (p, q, a, b) = (vec3(v, 0), vec3(v, 3), vec3(v, 6), vec3(v, 9));
    let e = vsub(b, a);
    let le = vdot(e, e).sqrt();
    if le.val() == 0.0 {
        return [T::cst(f64::NAN); 3];
    }
    let eh = e.map(|t| t / le);
    let s = vdot(vsub(p, a), eh);
    let two = T::cst(2.0);
    [0, 1, 2].map(|t| q[t] + p[t] - two * a[t] - two * s * eh[t])
}

/// A kernel's Taylor form, its rows over `Jet`s into `r`.
pub(crate) type JetForm = fn(&[Jet], &[f64], &mut [Jet]);

/// The form of a kernel written over `Num`, by the name it is registered under: its rows read
/// over `Jet`s.
pub(crate) fn num_form(name: &str) -> Option<JetForm> {
    macro_rules! form {
        ($rows:ident) => {
            Some(|v: &[Jet], k: &[f64], r: &mut [Jet]| r.copy_from_slice(&$rows(v, k)))
        };
    }
    match name {
        "lift" => form!(lift_rows),
        "distance3" => form!(distance3_rows),
        "distance3_free" => form!(distance3_free_rows),
        "point_line3" => form!(point_line3_rows),
        "point_line3_free" => form!(point_line3_free_rows),
        "line_line3" => form!(line_line3_rows),
        "line_line3_free" => form!(line_line3_free_rows),
        "angle3" => form!(angle3_rows),
        "angle3_free" => form!(angle3_free_rows),
        "perpendicular3" => form!(perpendicular3_rows),
        "parallel3" => form!(parallel3_rows),
        "point_on_plane" => form!(point_on_plane_rows),
        "point_on_circle3" => form!(point_on_circle3_rows),
        "project_solved" => form!(project_solved_rows),
        "point_on_line3" => form!(point_on_line3_rows),
        "equal_length3" => form!(equal_length3_rows),
        "line_on_plane" => form!(line_on_plane_rows),
        "symmetric3" => form!(symmetric3_rows),
        "axis_unit" => form!(axis_unit_rows),
        "axis_foot" => form!(axis_foot_rows),
        "point_on_axis" => form!(point_on_axis_rows),
        "ordinate_space" => form!(ordinate_space_rows),
        "ordinate_space_free" => form!(ordinate_space_free_rows),
        "ordinate_frame_u" => form!(ordinate_frame_u_rows),
        "ordinate_frame_u_free" => form!(ordinate_frame_u_free_rows),
        "ordinate_frame_v" => form!(ordinate_frame_v_rows),
        "ordinate_frame_v_free" => form!(ordinate_frame_v_free_rows),
        "ordinate_frame_n" => form!(ordinate_frame_n_rows),
        "ordinate_frame_n_free" => form!(ordinate_frame_n_free_rows),
        "axis_on_plane" => form!(axis_on_plane_rows),
        "axis_parallel_plane" => form!(axis_parallel_plane_rows),
        "axis_perpendicular_plane" => form!(axis_perpendicular_plane_rows),
        "plane_distance" => form!(plane_distance_rows),
        "plane_distance_free" => form!(plane_distance_free_rows),
        "axis_coincident" => form!(axis_coincident_rows),
        "plane_parallel" => form!(plane_parallel_rows),
        "line_on_axis" => form!(line_on_axis_rows),
        _ => None,
    }
}

/* -- two directed angles equal, and an arc's length --------------------------------------- */

/// (l1, l2, l3, l4 — each a line's four coordinates), K = (s): `wrap(∠(l1→l2) − s·∠(l3→l4))`.
///
/// Two of `angle`'s bearings-differences and one wrap, so it is `angle`'s statement with the
/// second pair's angle where the number was: directed and on the full turn, degree 0, and the
/// gradient `angle_gap_jac` already writes for each pair — the second pair's scaled by `−s`.  One
/// wrap over the whole difference rather than one per pair, so two angles a lap apart are the
/// same angle, as `angle(30)` and `angle(390)` are.
fn equal_angle_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 16 * i;
        let (d12, c12) = dot_cross(&v[o..]);
        let (d34, c34) = dot_cross(&v[o + 8..]);
        r[i] = wrap_turn(c12.datan2(d12) - k[i] * c34.datan2(d34));
    }
}

fn equal_angle_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 16 * i;
        angle_gap_jac(&v[o..], &mut j[o..o + 8]);
        let mut second = [0.0; 8];
        angle_gap_jac(&v[o + 8..], &mut second);
        for (c, g) in second.iter().enumerate() {
            j[o + 8 + c] = -k[i] * g;
        }
    }
}

/// An arc's sweep counter-clockwise from its start to its end, in (0, 2π] — the reading
/// `Sketch::arc_angles` takes, so the kernel and the drawing agree about which way round the arc
/// runs — and its gradient in the centre's, the start's and the end's columns.
///
/// The sweep is a difference of two bearings about the centre, so, as in `angle_gap_jac`, each
/// end sees only its own radius vector: `∂θ/∂e = (−w_y, w_x)/|w|²` for the end and the negation of
/// the same form for the start, with the centre carrying minus both.  The cut at a full turn is
/// where the arc closes up on itself, which no drawing is solved towards.
fn arc_sweep(v: &[f64]) -> (f64, [f64; 6]) {
    const TURN: f64 = 2.0 * std::f64::consts::PI;
    const MIN_LEN_SQ: f64 = MIN_LINE_LEN * MIN_LINE_LEN;
    let (ux, uy) = (v[2] - v[0], v[3] - v[1]);
    let (wx, wy) = (v[4] - v[0], v[5] - v[1]);
    let mut th = wy.datan2(wx) - uy.datan2(ux);
    if th <= 0.0 {
        th += TURN;
    }
    let iu = 1.0 / (ux * ux + uy * uy).max(MIN_LEN_SQ);
    let iw = 1.0 / (wx * wx + wy * wy).max(MIN_LEN_SQ);
    let (sx, sy) = (uy * iu, -ux * iu);
    let (ex, ey) = (-wy * iw, wx * iw);
    (th, [-(sx + ex), -(sy + ey), sx, sy, ex, ey])
}

/// (cx,cy,sx,sy,ex,ey,r), K = (L): `r·θ − L`, θ the arc's own sweep
fn arc_length_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 7 * i;
        r[i] = v[o + 6] * arc_sweep(&v[o..]).0 - k[i];
    }
}

fn arc_length_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 7 * i;
        let (th, g) = arc_sweep(&v[o..]);
        for c in 0..6 {
            j[o + c] = v[o + 6] * g[c];
        }
        j[o + 6] = th;
    }
}

/// (cx,cy,sx,sy,ex,ey,r,a), K = (m,c): `r·θ − (m·a + c)`
fn arc_length_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 8 * i;
        r[i] = v[o + 6] * arc_sweep(&v[o..]).0 - free_dim(v, k, i, o + 7).0;
    }
}

fn arc_length_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 8 * i;
        let (th, g) = arc_sweep(&v[o..]);
        for c in 0..6 {
            j[o + c] = v[o + 6] * g[c];
        }
        j[o + 6] = th;
        j[o + 7] = -k[2 * i];
    }
}

pub static KERNELS: [Kernel; N_KERNELS] = [
    Kernel { name: "coincident", n_res: 2, n_par: 4, degree: 1, n_const: 0, res: coincident::res, jac: coincident::jac, const_jac: Some(coincident::J) },
    Kernel { name: "distance", n_res: 1, n_par: 4, degree: 2, n_const: 1, res: distance_res, jac: distance_jac, const_jac: None },
    Kernel { name: "midpoint", n_res: 2, n_par: 6, degree: 1, n_const: 0, res: midpoint::res, jac: midpoint::jac, const_jac: Some(midpoint::J) },
    Kernel { name: "drag", n_res: 2, n_par: 2, degree: 1, n_const: 3, res: drag_res, jac: drag_jac, const_jac: None },
    Kernel { name: "horizontal", n_res: 1, n_par: 4, degree: 1, n_const: 0, res: horizontal::res, jac: horizontal::jac, const_jac: Some(horizontal::J) },
    Kernel { name: "vertical", n_res: 1, n_par: 4, degree: 1, n_const: 0, res: vertical::res, jac: vertical::jac, const_jac: Some(vertical::J) },
    Kernel { name: "parallel", n_res: 1, n_par: 8, degree: 2, n_const: 0, res: parallel_res, jac: parallel_jac, const_jac: None },
    Kernel { name: "perpendicular", n_res: 1, n_par: 8, degree: 2, n_const: 0, res: perpendicular_res, jac: perpendicular_jac, const_jac: None },
    Kernel { name: "angle", n_res: 1, n_par: 8, degree: 0, n_const: 1, res: angle_res, jac: angle_jac, const_jac: None },
    Kernel { name: "equal_length", n_res: 1, n_par: 8, degree: 2, n_const: 0, res: equal_length_res, jac: equal_length_jac, const_jac: None },
    Kernel { name: "point_on_line", n_res: 1, n_par: 6, degree: 2, n_const: 0, res: point_on_line_res, jac: point_on_line_jac, const_jac: None },
    Kernel { name: "point_on_circle", n_res: 1, n_par: 5, degree: 2, n_const: 0, res: point_on_circle_res, jac: point_on_circle_jac, const_jac: None },
    Kernel { name: "radius", n_res: 1, n_par: 1, degree: 1, n_const: 1, res: radius_res, jac: radius_jac, const_jac: Some(RADIUS_J) },
    Kernel { name: "equal_radius", n_res: 1, n_par: 2, degree: 1, n_const: 0, res: equal_radius::res, jac: equal_radius::jac, const_jac: Some(equal_radius::J) },
    Kernel { name: "tangent_line_circle", n_res: 1, n_par: 7, degree: 1, n_const: 1, res: tangent_line_circle_res, jac: tangent_line_circle_jac, const_jac: None },
    Kernel { name: "tangent_circle_circle", n_res: 1, n_par: 6, degree: 2, n_const: 1, res: tangent_circle_circle_res, jac: tangent_circle_circle_jac, const_jac: None },
    Kernel { name: "tangent_arc_line", n_res: 1, n_par: 8, degree: 2, n_const: 0, res: tangent_arc_line_res, jac: tangent_arc_line_jac, const_jac: None },
    Kernel { name: "symmetric", n_res: 2, n_par: 8, degree: 2, n_const: 0, res: symmetric_res, jac: symmetric_jac, const_jac: None },
    Kernel { name: "parallel_distance", n_res: 1, n_par: 8, degree: 1, n_const: 1, res: parallel_distance_res, jac: parallel_distance_jac, const_jac: None },
    Kernel { name: "point_line_distance", n_res: 1, n_par: 6, degree: 1, n_const: 1, res: point_line_distance_res, jac: point_line_distance_jac, const_jac: None },
    Kernel { name: "annular_distance", n_res: 1, n_par: 2, degree: 1, n_const: 1, res: annular_distance_res, jac: annular_distance_jac, const_jac: Some(ANNULAR_DISTANCE_J) },
    Kernel { name: "point_on_spline", n_res: 2, n_par: N_PAR_ON_SPLINE, degree: 1, n_const: SPAN_C, res: point_on_spline_res, jac: point_on_spline_jac, const_jac: None },
    Kernel { name: "spline_tangent_line", n_res: 2, n_par: N_PAR_SPLINE_LINE, degree: 1, n_const: SPAN_C, res: spline_tangent_line_res, jac: spline_tangent_line_jac, const_jac: None },
    Kernel { name: "spline_curvature", n_res: 3, n_par: N_PAR_SPLINE_CURVE, degree: 1, n_const: SPAN_C, res: spline_curvature_res, jac: spline_curvature_jac, const_jac: None },
    Kernel { name: "ordinate_u", n_res: 1, n_par: 4, degree: 1, n_const: 1, res: ordinate_u_res, jac: ordinate_u_jac, const_jac: Some(ORDINATE_U_J) },
    Kernel { name: "ordinate_v", n_res: 1, n_par: 4, degree: 1, n_const: 1, res: ordinate_v_res, jac: ordinate_v_jac, const_jac: Some(ORDINATE_V_J) },
    Kernel { name: "distance_free", n_res: 1, n_par: 5, degree: 2, n_const: 2, res: distance_free_res, jac: distance_free_jac, const_jac: None },
    Kernel { name: "angle_free", n_res: 1, n_par: 9, degree: 0, n_const: 2, res: angle_free_res, jac: angle_free_jac, const_jac: None },
    Kernel { name: "radius_free", n_res: 1, n_par: 2, degree: 1, n_const: 2, res: radius_free_res, jac: radius_free_jac, const_jac: None },
    Kernel { name: "parallel_distance_free", n_res: 1, n_par: 9, degree: 1, n_const: 2, res: parallel_distance_free_res, jac: parallel_distance_free_jac, const_jac: None },
    Kernel { name: "point_line_distance_free", n_res: 1, n_par: 7, degree: 1, n_const: 2, res: point_line_distance_free_res, jac: point_line_distance_free_jac, const_jac: None },
    Kernel { name: "annular_distance_free", n_res: 1, n_par: 3, degree: 1, n_const: 2, res: annular_distance_free_res, jac: annular_distance_free_jac, const_jac: None },
    Kernel { name: "ordinate_u_free", n_res: 1, n_par: 5, degree: 1, n_const: 2, res: ordinate_u_free_res, jac: ordinate_u_free_jac, const_jac: None },
    Kernel { name: "ordinate_v_free", n_res: 1, n_par: 5, degree: 1, n_const: 2, res: ordinate_v_free_res, jac: ordinate_v_free_jac, const_jac: None },
    Kernel { name: "project", n_res: 1, n_par: 4, degree: 1, n_const: 5, res: project_res, jac: project_jac, const_jac: None },
    Kernel { name: "point_line_magnitude", n_res: 1, n_par: 6, degree: 1, n_const: 1, res: point_line_magnitude_res, jac: point_line_magnitude_jac, const_jac: None },
    Kernel { name: "point_line_magnitude_free", n_res: 1, n_par: 7, degree: 1, n_const: 2, res: point_line_magnitude_free_res, jac: point_line_magnitude_free_jac, const_jac: None },
    Kernel { name: "parallel_magnitude", n_res: 1, n_par: 8, degree: 1, n_const: 1, res: parallel_magnitude_res, jac: parallel_magnitude_jac, const_jac: None },
    Kernel { name: "parallel_magnitude_free", n_res: 1, n_par: 9, degree: 1, n_const: 2, res: parallel_magnitude_free_res, jac: parallel_magnitude_free_jac, const_jac: None },
    Kernel { name: "lift", n_res: 3, n_par: 14, degree: 1, n_const: 0, res: lift_res, jac: lift_jac, const_jac: None },
    Kernel { name: "coincident3", n_res: 3, n_par: 6, degree: 1, n_const: 0, res: coincident3_res, jac: coincident3_jac, const_jac: None },
    Kernel { name: "distance3", n_res: 1, n_par: 6, degree: 2, n_const: 1, res: distance3_res, jac: distance3_jac, const_jac: None },
    Kernel { name: "distance3_free", n_res: 1, n_par: 7, degree: 2, n_const: 2, res: distance3_free_res, jac: distance3_free_jac, const_jac: None },
    Kernel { name: "point_line3", n_res: 1, n_par: 9, degree: 1, n_const: 1, res: point_line3_res, jac: point_line3_jac, const_jac: None },
    Kernel { name: "point_line3_free", n_res: 1, n_par: 10, degree: 1, n_const: 2, res: point_line3_free_res, jac: point_line3_free_jac, const_jac: None },
    Kernel { name: "line_line3", n_res: 1, n_par: 12, degree: 1, n_const: 1, res: line_line3_res, jac: line_line3_jac, const_jac: None },
    Kernel { name: "line_line3_free", n_res: 1, n_par: 13, degree: 1, n_const: 2, res: line_line3_free_res, jac: line_line3_free_jac, const_jac: None },
    Kernel { name: "angle3", n_res: 1, n_par: 12, degree: 0, n_const: 1, res: angle3_res, jac: angle3_jac, const_jac: None },
    Kernel { name: "angle3_free", n_res: 1, n_par: 13, degree: 0, n_const: 2, res: angle3_free_res, jac: angle3_free_jac, const_jac: None },
    Kernel { name: "perpendicular3", n_res: 1, n_par: 12, degree: 0, n_const: 0, res: perpendicular3_res, jac: perpendicular3_jac, const_jac: None },
    Kernel { name: "parallel3", n_res: 2, n_par: 12, degree: 0, n_const: 6, res: parallel3_res, jac: parallel3_jac, const_jac: None },
    Kernel { name: "point_on_plane", n_res: 1, n_par: 12, degree: 1, n_const: 0, res: point_on_plane_res, jac: point_on_plane_jac, const_jac: None },
    Kernel { name: "point_on_circle3", n_res: 2, n_par: 13, degree: 1, n_const: 0, res: point_on_circle3_res, jac: point_on_circle3_jac, const_jac: None },
    Kernel { name: "project_solved", n_res: 1, n_par: 18, degree: 1, n_const: 0, res: project_solved_res, jac: project_solved_jac, const_jac: None },
    Kernel { name: "point_on_line3", n_res: 2, n_par: 9, degree: 1, n_const: 6, res: point_on_line3_res, jac: point_on_line3_jac, const_jac: None },
    Kernel { name: "equal_length3", n_res: 1, n_par: 12, degree: 2, n_const: 0, res: equal_length3_res, jac: equal_length3_jac, const_jac: None },
    Kernel { name: "ordinate_line", n_res: 1, n_par: 8, degree: 1, n_const: 1, res: ordinate_line_res, jac: ordinate_line_jac, const_jac: None },
    Kernel { name: "ordinate_line_free", n_res: 1, n_par: 9, degree: 1, n_const: 2, res: ordinate_line_free_res, jac: ordinate_line_free_jac, const_jac: None },
    Kernel { name: "line_on_plane", n_res: 2, n_par: 15, degree: 1, n_const: 0, res: line_on_plane_res, jac: line_on_plane_jac, const_jac: None },
    Kernel { name: "midpoint3", n_res: 3, n_par: 9, degree: 1, n_const: 0, res: midpoint3_res, jac: midpoint3_jac, const_jac: None },
    Kernel { name: "symmetric3", n_res: 3, n_par: 12, degree: 1, n_const: 0, res: symmetric3_res, jac: symmetric3_jac, const_jac: None },
    Kernel { name: "equal_angle", n_res: 1, n_par: 16, degree: 0, n_const: 1, res: equal_angle_res, jac: equal_angle_jac, const_jac: None },
    Kernel { name: "arc_length", n_res: 1, n_par: 7, degree: 1, n_const: 1, res: arc_length_res, jac: arc_length_jac, const_jac: None },
    Kernel { name: "arc_length_free", n_res: 1, n_par: 8, degree: 1, n_const: 2, res: arc_length_free_res, jac: arc_length_free_jac, const_jac: None },
    Kernel { name: "axis_unit", n_res: 1, n_par: 3, degree: 0, n_const: 0, res: axis_unit_res, jac: axis_unit_jac, const_jac: None },
    Kernel { name: "axis_foot", n_res: 1, n_par: 6, degree: 1, n_const: 0, res: axis_foot_res, jac: axis_foot_jac, const_jac: None },
    Kernel { name: "point_on_axis", n_res: 2, n_par: 9, degree: 1, n_const: 6, res: point_on_axis_res, jac: point_on_axis_jac, const_jac: None },
    Kernel { name: "ordinate_space", n_res: 1, n_par: 12, degree: 1, n_const: 1, res: ordinate_space_res, jac: ordinate_space_jac, const_jac: None },
    Kernel { name: "ordinate_frame_u", n_res: 1, n_par: 12, degree: 1, n_const: 1, res: ordinate_frame_u_res, jac: ordinate_frame_u_jac, const_jac: None },
    Kernel { name: "ordinate_space_free", n_res: 1, n_par: 13, degree: 1, n_const: 2, res: ordinate_space_free_res, jac: ordinate_space_free_jac, const_jac: None },
    Kernel { name: "ordinate_frame_u_free", n_res: 1, n_par: 13, degree: 1, n_const: 2, res: ordinate_frame_u_free_res, jac: ordinate_frame_u_free_jac, const_jac: None },
    Kernel { name: "ordinate_frame_v", n_res: 1, n_par: 12, degree: 1, n_const: 1, res: ordinate_frame_v_res, jac: ordinate_frame_v_jac, const_jac: None },
    Kernel { name: "ordinate_frame_v_free", n_res: 1, n_par: 13, degree: 1, n_const: 2, res: ordinate_frame_v_free_res, jac: ordinate_frame_v_free_jac, const_jac: None },
    Kernel { name: "ordinate_frame_n", n_res: 1, n_par: 12, degree: 1, n_const: 1, res: ordinate_frame_n_res, jac: ordinate_frame_n_jac, const_jac: None },
    Kernel { name: "ordinate_frame_n_free", n_res: 1, n_par: 13, degree: 1, n_const: 2, res: ordinate_frame_n_free_res, jac: ordinate_frame_n_free_jac, const_jac: None },
    Kernel { name: "axis_on_plane", n_res: 2, n_par: 15, degree: 1, n_const: 1, res: axis_on_plane_res, jac: axis_on_plane_jac, const_jac: None },
    Kernel { name: "axis_parallel_plane", n_res: 1, n_par: 9, degree: 0, n_const: 0, res: axis_parallel_plane_res, jac: axis_parallel_plane_jac, const_jac: None },
    Kernel { name: "axis_perpendicular_plane", n_res: 2, n_par: 9, degree: 0, n_const: 6, res: axis_perpendicular_plane_res, jac: axis_perpendicular_plane_jac, const_jac: None },
    Kernel { name: "plane_distance", n_res: 1, n_par: 12, degree: 1, n_const: 1, res: plane_distance_res, jac: plane_distance_jac, const_jac: None },
    Kernel { name: "plane_distance_free", n_res: 1, n_par: 13, degree: 1, n_const: 2, res: plane_distance_free_res, jac: plane_distance_free_jac, const_jac: None },
    Kernel { name: "axis_coincident", n_res: 4, n_par: 12, degree: 1, n_const: 7, res: axis_coincident_res, jac: axis_coincident_jac, const_jac: None },
    Kernel { name: "plane_parallel", n_res: 2, n_par: 12, degree: 0, n_const: 6, res: plane_parallel_res, jac: plane_parallel_jac, const_jac: None },
    Kernel { name: "drag_seen", n_res: 2, n_par: 3, degree: 1, n_const: 9, res: drag_seen_res, jac: drag_seen_jac, const_jac: None },
    Kernel { name: "line_on_axis", n_res: 4, n_par: 12, degree: 1, n_const: 6, res: line_on_axis_res, jac: line_on_axis_jac, const_jac: None },
];

/// One row of a kernel: residual and Jacobian for a single constraint's local values.  The
/// scalar view of the vectorized kernels, kept for the finite-difference checker and reporting.
pub fn eval_one(id: usize, v: &[f64], c: &[f64]) -> (Vec<f64>, Vec<f64>) {
    eval_with(&KERNELS[id], v, c)
}

/// `eval_one` over a kernel in hand — one built rather than registered (`dual_kernel`).
pub fn eval_with(k: &Kernel, v: &[f64], c: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let mut r = vec![0.0; k.n_res];
    let mut j = vec![0.0; k.n_res * k.n_par];
    (k.res)(1, v, c, &mut r);
    (k.jac)(1, v, c, &mut j);
    (r, j)
}
