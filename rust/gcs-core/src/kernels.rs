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
    HorizontalDistance,
    VerticalDistance,
    // the same dimensions again, with the number they state left to the solver
    DistanceFree,
    AngleFree,
    RadiusFree,
    ParallelDistanceFree,
    PointLineDistanceFree,
    AnnularDistanceFree,
    HorizontalDistanceFree,
    VerticalDistanceFree,
    FrameUnit,
    FrameAlign,
    Project,
    // and the magnitude forms of the two distances measured *from a line*, which a statement
    // that names no side compiles to: both sides are solutions and the seed picks (issue #48,
    // item 4).  Each with its own free twin, since a magnitude may state a shared unknown too.
    PointLineMagnitude,
    PointLineMagnitudeFree,
    ParallelMagnitude,
    ParallelMagnitudeFree,
    CoordinateU,
    CoordinateV,
    CoordinateUFree,
    CoordinateVFree,
    // a solved view's quaternion on the unit sphere, and a hidden point held at the lift of
    // its view point over a solved view and over a stated one
    QuatUnit,
    Lift,
    LiftFixed,
    // the relations in space, over the hidden points views lift to: each dimension with its
    // free twin beside it, and the two that read a plane in its solved and stated forms
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
    PointOnPlaneFixed,
    PointOnCircle3,
    PointOnCircle3Fixed,
    // a view folded from a solved one: its quaternion tied to its parent's by a stated
    // fold, a solved one, or a line drawn in the parent — and a projection between two views
    // either of which is solved
    Hinge,
    HingeFree,
    HingeAlong,
    ProjectFree,
    // a point on a line in space, true lengths equal, a point's signed distance along a
    // plane's normal (its stated-plane form is `point_on_plane_fixed`), and the sphere's own two
    PointOnLine3,
    EqualLength3,
    PointPlaneDistance,
    PointPlaneDistanceFree,
    PointPlaneDistanceFixedFree,
    SphereOn,
    SphereSphere,
    LineOnPlane,
    LineOnPlaneFixed,
    // a circle drawn in a view on a sphere, over the view solved and stated, and the
    // midpoint and the mirror in a line, in space
    CircleOnSphere,
    CircleOnSphereFixed,
    Midpoint3,
    Symmetric3,
    // a point on a cone, a cone's half-angle stated and free, and two cones touching
    ConeOn,
    HalfAngle,
    HalfAngleFree,
    ConeCone,
    // a mate between two solved views' offsets
    Mate,
    // two directed angles equal, and an arc's length along itself stated and free
    EqualAngle,
    ArcLength,
    ArcLengthFree,
    // a ray's own two rows, and a point on one
    RayUnit,
    RayFoot,
    PointOnRay,
}

pub const N_KERNELS: usize = 91;

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
/// the datum on the sheet `(ox, oy, c, s)` and the basis `(u, v, o)`.
pub const EXTRUSION_FRAME: usize = 13;

/// A point in space on the surface a curve of a view stands for (`CKind::PointOnExtrusion`,
/// issue #70): `(x, y, z, t, θ…)`, the point's lift, the curve's parameter and its columns, over
/// the constants `EXTRUSION_FRAME` and then the contact's.  The point read into the view and put
/// on the sheet, less `C(t)`: `point_on_envelope`'s rows with the point's place in the view for
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
fn horizontal_distance_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 4 * i;
        r[i] = v[o + 2] - v[o] - k[i];
    }
}

/// And the rise: (qy - py) - d.
fn vertical_distance_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 4 * i;
        r[i] = v[o + 3] - v[o + 1] - k[i];
    }
}

static HORIZONTAL_DISTANCE_J: &[f64] = &[-1.0, 0.0, 1.0, 0.0];
static VERTICAL_DISTANCE_J: &[f64] = &[0.0, -1.0, 0.0, 1.0];

fn horizontal_distance_jac(n: usize, _v: &[f64], _k: &[f64], j: &mut [f64]) {
    lin_jac(n, HORIZONTAL_DISTANCE_J, j)
}

fn vertical_distance_jac(n: usize, _v: &[f64], _k: &[f64], j: &mut [f64]) {
    lin_jac(n, VERTICAL_DISTANCE_J, j)
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
 * knot window is the constants.
 */

use crate::curve::{self, SPAN_K, SPAN_N};

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
fn span_frame(v: &[f64], t: usize, ctrl: usize, k: &[f64; SPAN_K]) -> Span {
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
    curve::basis(v[t], k, &mut f.b, &mut f.d, &mut f.dd, &mut f.d3);
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

/// The i-th instance's local knot window out of a block's constants.
#[inline]
fn span_knots(k: &[f64], i: usize) -> &[f64; SPAN_K] {
    k[SPAN_K * i..SPAN_K * (i + 1)].try_into().expect("a block's constants are SPAN_K per row")
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
fn horizontal_distance_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 5 * i;
        r[i] = v[o + 2] - v[o] - free_dim(v, k, i, o + 4).0;
    }
}

fn horizontal_distance_free_jac(n: usize, _v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 5 * i;
        j[o..o + 4].copy_from_slice(HORIZONTAL_DISTANCE_J);
        j[o + 4] = -k[2 * i];
    }
}

/// (px,py,qx,qy,a), K = (m,c): (qy - py) - (m*a + c)
fn vertical_distance_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 5 * i;
        r[i] = v[o + 3] - v[o + 1] - free_dim(v, k, i, o + 4).0;
    }
}

fn vertical_distance_free_jac(n: usize, _v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 5 * i;
        j[o..o + 4].copy_from_slice(VERTICAL_DISTANCE_J);
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

/// A point's lift read into a view and put on the sheet, and the gradient of that place along the
/// lift: `k` is `EXTRUSION_FRAME`'s numbers.
fn extrusion_place(p: &[f64], k: &[f64]) -> ([f64; 2], [[f64; 3]; 2]) {
    let (c, s) = (k[2], k[3]);
    let (u, w, o) = (&k[4..7], &k[7..10], &k[10..13]);
    let d = [p[0] - o[0], p[1] - o[1], p[2] - o[2]];
    let (a, b) = (d[0] * u[0] + d[1] * u[1] + d[2] * u[2], d[0] * w[0] + d[1] * w[1] + d[2] * w[2]);
    let (x, y) = crate::plane::on_page(c, s, (k[0], k[1]), (a, b));
    let grad = [std::array::from_fn(|i| c * u[i] - s * w[i]), std::array::from_fn(|i| s * u[i] + c * w[i])];
    ([x, y], grad)
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

/// Columns of `frame_unit`: (c, s).
///
/// `r = c² + s² − 1`, the rotor held to the unit circle.  Dimensionless residual, judged in
/// absolute units like the angular gap — degree 0 — and its gradient carries 1/length once the
/// rotor columns are scaled by the chord, which is exactly what `extent^(degree − 1)` says of it
/// (the `angle` kernel's rationale).
fn frame_unit_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 2 * i;
        r[i] = v[o] * v[o] + v[o + 1] * v[o + 1] - 1.0;
    }
}

fn frame_unit_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 2 * i;
        j[o] = 2.0 * v[o];
        j[o + 1] = 2.0 * v[o + 1];
    }
}

/// Columns of `frame_align`: (r, ox, oy, tx, ty, c, s).
pub const N_PAR_FRAME_ALIGN: usize = 7;

/// `(t − o) − r·(c, s) = 0`: the rotor kept on the chord `origin → toward`.  Two residuals
/// against one owned unknown `r` (the chord's length), the net one equation an attitude is
/// worth — and the *directed* form: with `frame_unit` holding `(c, s)` to the unit circle, `r`
/// stays positive by continuity, where a bare cross-product row would be satisfied by the
/// reversed frame too.  A signed displacement, so degree 1.
fn frame_align_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_FRAME_ALIGN * i;
        r[2 * i] = (v[o + 3] - v[o + 1]) - v[o] * v[o + 5];
        r[2 * i + 1] = (v[o + 4] - v[o + 2]) - v[o] * v[o + 6];
    }
}

fn frame_align_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_FRAME_ALIGN * i;
        let jo = 2 * N_PAR_FRAME_ALIGN * i;
        let row1 = jo + N_PAR_FRAME_ALIGN;
        for t in 0..2 * N_PAR_FRAME_ALIGN {
            j[jo + t] = 0.0;
        }
        j[jo] = -v[o + 5]; // ∂/∂r = -c
        j[jo + 1] = -1.0;
        j[jo + 3] = 1.0;
        j[jo + 5] = -v[o]; // ∂/∂c = -r
        j[row1] = -v[o + 6]; // ∂/∂r = -s
        j[row1 + 2] = -1.0;
        j[row1 + 4] = 1.0;
        j[row1 + 6] = -v[o]; // ∂/∂s = -r
    }
}

/// Columns of `project`: (px, py, qx, qy, oax, oay, ca, sa, obx, oby, cb, sb) — the two images,
/// then each plane's origin and rotor.  Constants: (dax, day, dbx, dby), the fold line the two
/// planes share in each plane's own 2D coordinates (`plane::fold_line`).
pub const N_PAR_PROJECT: usize = 12;

/// The projector rule of descriptive geometry: two images of one point agree on their
/// coordinate along the fold line their planes share, and on nothing else.  Each image is read
/// in its plane's own frame — `Rᵀ(c, s)(p − o)` with `Rᵀ(c, s)(x, y) = (c·x + s·y, −s·x + c·y)`
/// — and dotted with the fold direction there:
///
/// `r = d_A · Rᵀ(c_A, s_A)(p − o_A) − d_B · Rᵀ(c_B, s_B)(q − o_B)`
///
/// One row, bilinear in rotor × length, so degree 1 like `frame_align`.  A view slides freely
/// along its projectors (perpendicular to the fold on the page) without moving the row, which
/// is the free spacing between the views of a drawing.
fn project_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_PROJECT * i;
        let (dax, day, dbx, dby) = (k[4 * i], k[4 * i + 1], k[4 * i + 2], k[4 * i + 3]);
        let (ca, sa) = (v[o + 6], v[o + 7]);
        let (cb, sb) = (v[o + 10], v[o + 11]);
        // each image read in its own view — the one reading, shared with `overview`
        let (ax, ay) = crate::plane::in_view(ca, sa, (v[o + 4], v[o + 5]), (v[o], v[o + 1]));
        let (bx, by) = crate::plane::in_view(cb, sb, (v[o + 8], v[o + 9]), (v[o + 2], v[o + 3]));
        r[i] = (dax * ax + day * ay) - (dbx * bx + dby * by);
    }
}

fn project_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_PROJECT * i;
        let (dax, day, dbx, dby) = (k[4 * i], k[4 * i + 1], k[4 * i + 2], k[4 * i + 3]);
        let (wx, wy) = (v[o] - v[o + 4], v[o + 1] - v[o + 5]);
        let (ca, sa) = (v[o + 6], v[o + 7]);
        let (ux, uy) = (v[o + 2] - v[o + 8], v[o + 3] - v[o + 9]);
        let (cb, sb) = (v[o + 10], v[o + 11]);
        // the fold direction as drawn in each view: `R(c, s)·d`
        let (gax, gay) = (dax * ca - day * sa, dax * sa + day * ca);
        let (gbx, gby) = (dbx * cb - dby * sb, dbx * sb + dby * cb);
        j[o] = gax;
        j[o + 1] = gay;
        j[o + 2] = -gbx;
        j[o + 3] = -gby;
        j[o + 4] = -gax;
        j[o + 5] = -gay;
        j[o + 6] = dax * wx + day * wy;
        j[o + 7] = -day * wx + dax * wy;
        j[o + 8] = gbx;
        j[o + 9] = gby;
        j[o + 10] = -(dbx * ux + dby * uy);
        j[o + 11] = -(-dby * ux + dbx * uy);
    }
}

// Signed coordinates. Columns: point.xy, origin.xy, rotor.cs, then an optional
// free dimension. The datum is an ordinary participant in the same solve.
fn coordinate_res<const V: bool, const FREE: bool>(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let width = if FREE { 7 } else { 6 };
    for i in 0..n {
        let o = i * width;
        let (dx, dy, c, s) = (v[o] - v[o + 2], v[o + 1] - v[o + 3], v[o + 4], v[o + 5]);
        let d = if FREE { free_dim(v, k, i, o + 6).0 } else { k[i] };
        r[i] = if V { -s * dx + c * dy - d } else { c * dx + s * dy - d };
    }
}

fn coordinate_jac<const V: bool, const FREE: bool>(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let width = if FREE { 7 } else { 6 };
    for i in 0..n {
        let o = i * width;
        let (dx, dy, c, s) = (v[o] - v[o + 2], v[o + 1] - v[o + 3], v[o + 4], v[o + 5]);
        let row = if V { [-s, c, s, -c, dy, -dx] } else { [c, s, -c, -s, dx, dy] };
        j[o..o + 6].copy_from_slice(&row);
        if FREE { j[o + 6] = -k[2 * i]; }
    }
}

/// Columns of `quat_unit`: (w, x, y, z).
///
/// `r = |q|² − 1`, `frame_unit` one dimension up: dimensionless, judged absolute, degree 0.  The
/// `lift` reads the direction of `q` only, so this row is the whole of what fixes its length.
fn quat_unit_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 4 * i;
        r[i] = v[o] * v[o] + v[o + 1] * v[o + 1] + v[o + 2] * v[o + 2] + v[o + 3] * v[o + 3] - 1.0;
    }
}

fn quat_unit_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 4 * i;
        for t in 0..4 {
            j[o + t] = 2.0 * v[o + t];
        }
    }
}

/// Columns of `lift`: (X, Y, Z, px, py, ox, oy, c, s, qw, qx, qy, qz, d) — the hidden point, the
/// view point, its datum's origin and rotor, and the view's quaternion and offset.  Constants:
/// (a, b), the in-plane part of the view's origin (`model::Att`).
pub const N_PAR_LIFT: usize = 14;

/// `X − R(q)·(a + a′, b + b′, d) = 0`, with `(a′, b′) = plane::in_view(c, s, o, p)` — the point
/// as the draughtsman measured it on its view, stood up in space by the view's solved attitude
/// (`plane::lift_q`, the one statement of it).  Three signed displacements: degree 1.
fn lift_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_LIFT * i;
        let (a, b) =
            crate::plane::in_view(v[o + 7], v[o + 8], (v[o + 5], v[o + 6]), (v[o + 3], v[o + 4]));
        let q = quat_at(v, o + 9);
        let w = [k[2 * i] + a, k[2 * i + 1] + b, v[o + 13]];
        let l = crate::plane::lift_q(q, w).map_or([f64::NAN; 3], |(l, _, _)| l);
        for t in 0..3 {
            r[3 * i + t] = v[o + t] - l[t];
        }
    }
}

fn lift_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_LIFT * i;
        let (px, py, ox, oy, c, s) = (v[o + 3], v[o + 4], v[o + 5], v[o + 6], v[o + 7], v[o + 8]);
        let (dx, dy) = (px - ox, py - oy);
        let (a, b) = crate::plane::in_view(c, s, (ox, oy), (px, py));
        let q = quat_at(v, o + 9);
        let w = [k[2 * i] + a, k[2 * i + 1] + b, v[o + 13]];
        let jo = 3 * N_PAR_LIFT * i;
        let Some((_, rm, dq)) = crate::plane::lift_q(q, w) else {
            j[jo..jo + 3 * N_PAR_LIFT].fill(f64::NAN);
            continue;
        };
        for t in 0..3 {
            let row = &mut j[jo + t * N_PAR_LIFT..jo + (t + 1) * N_PAR_LIFT];
            // ∂L/∂(a′, b′) are the view's u and v
            lift_view_row(row, t, (rm[t][0], rm[t][1]), (c, s), (dx, dy));
            for m in 0..4 {
                row[9 + m] = -dq[t][m];
            }
            row[13] = -rm[t][2];
        }
    }
}

/// Row `t` of a lift's Jacobian over the columns the two lifts share — the hidden point, the
/// view point, its datum's origin and rotor — with the rest zeroed: `(gu, gv)` is component `t` of
/// the view's u and v, `∂L/∂(a′, b′)`, and the view coordinates' own derivatives are `in_view`'s,
/// a′ = c·dx + s·dy, b′ = −s·dx + c·dy, with `(dx, dy)` the view point less the datum's origin.
fn lift_view_row(row: &mut [f64], t: usize, (gu, gv): (f64, f64), (c, s): (f64, f64),
                 (dx, dy): (f64, f64)) {
    row.fill(0.0);
    row[t] = 1.0;
    row[3] = -(gu * c - gv * s);
    row[4] = -(gu * s + gv * c);
    row[5] = -row[3];
    row[6] = -row[4];
    row[7] = -(gu * dx + gv * dy);
    row[8] = -(gu * dy - gv * dx);
}

/// Columns of `lift_fixed`: (X, Y, Z, px, py, ox, oy, c, s).  Constants: the stated basis
/// (u, v, o), nine numbers.
pub const N_PAR_LIFT_FIXED: usize = 9;

/// `X − (o + a′·u + b′·v) = 0` — `lift`'s statement over a view whose attitude is document data,
/// which is `plane::Basis::lift` exactly.  Degree 1.
fn lift_fixed_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_LIFT_FIXED * i;
        let kb = &k[9 * i..9 * i + 9];
        let (a, b) =
            crate::plane::in_view(v[o + 7], v[o + 8], (v[o + 5], v[o + 6]), (v[o + 3], v[o + 4]));
        for t in 0..3 {
            r[3 * i + t] = v[o + t] - (kb[6 + t] + a * kb[t] + b * kb[3 + t]);
        }
    }
}

fn lift_fixed_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_LIFT_FIXED * i;
        let kb = &k[9 * i..9 * i + 9];
        let (dx, dy, c, s) = (v[o + 3] - v[o + 5], v[o + 4] - v[o + 6], v[o + 7], v[o + 8]);
        let jo = 3 * N_PAR_LIFT_FIXED * i;
        for t in 0..3 {
            let row = &mut j[jo + t * N_PAR_LIFT_FIXED..jo + (t + 1) * N_PAR_LIFT_FIXED];
            lift_view_row(row, t, (kb[t], kb[3 + t]), (c, s), (dx, dy));
        }
    }
}

/* -- relations in space ------------------------------------------------------
 *
 * What a relation between two views says (`docs/spatial-constraints-plan.md`).  Every one
 * reads the **hidden points** a view point lifts to (`model::LiftE`), three columns each, and
 * never a view's attitude — the `lift` rows are where a drawn point and its place in space are
 * tied together, so a relation here is plain vector algebra over points in space.  The two that
 * read a *plane* rather than points (a point on a plane, the plane row of a circle) read its
 * quaternion and offset where the view is solved and its normal as constants where it is
 * stated, which is `lift`/`lift_fixed`'s split again.
 */

use crate::space::{cross as cross3, dot as dot3, norm as norm3, sub as sub3};

#[inline]
fn at3(v: &[f64], o: usize) -> [f64; 3] {
    [v[o], v[o + 1], v[o + 2]]
}

/// A solved view's quaternion, four columns from `o`.
#[inline]
fn quat_at(v: &[f64], o: usize) -> crate::plane::Quat {
    [v[o], v[o + 1], v[o + 2], v[o + 3]]
}

/// `|d|` and the unit vector along `d`, or zero where `d` has no length — the gradient of a
/// distance in space, which has none at its own centre.
#[inline]
fn length_and_unit(d: [f64; 3]) -> (f64, [f64; 3]) {
    let l = norm3(d);
    (l, if l > 0.0 { d.map(|t| t / l) } else { [0.0; 3] })
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

/// A solved view's normal, `R(q)·e₃`, and its derivative in q — `plane::lift_q` of the unit
/// normal, the one statement of the rotation.
#[inline]
fn quat_normal(q: [f64; 4]) -> Option<([f64; 3], [[f64; 4]; 3])> {
    crate::plane::lift_q(q, [0.0, 0.0, 1.0]).map(|(nv, _, dq)| (nv, dq))
}

/// Columns of `point_on_plane`: (X, qw, qx, qy, qz, d) — a hidden point, and a solved plane's
/// quaternion and offset.  `n(q)·X − d = 0`: every lift of the plane's own points has `n·L = d`
/// exactly, since L = R(q)·(a, b, d), so this says X is on the plane wherever in it.  Degree 1.
pub const N_PAR_POINT_ON_PLANE: usize = 8;

fn point_on_plane_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    let mut g = [0.0; N_PAR_POINT_ON_PLANE];
    for i in 0..n {
        r[i] = plane_gap(&v[N_PAR_POINT_ON_PLANE * i..], &mut g);
    }
}

fn point_on_plane_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_POINT_ON_PLANE * i;
        plane_gap(&v[o..], &mut j[o..o + N_PAR_POINT_ON_PLANE]);
    }
}

/// Columns of `point_on_plane_fixed`: (X), K = (n, h) — a stated plane's normal and its origin
/// along it.  `n·X − h = 0`, `point_on_plane` with the attitude as constants.  Degree 1.
fn point_on_plane_fixed_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let kk = &k[4 * i..];
        r[i] = dot3(at3(kk, 0), at3(v, 3 * i)) - kk[3];
    }
}

fn point_on_plane_fixed_jac(n: usize, _v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        j[3 * i..3 * i + 3].copy_from_slice(&k[4 * i..4 * i + 3]);
    }
}

/// The two rows of a hidden point on a circle drawn in a view — the circle's centre lifted to C,
/// its radius r and its view's normal n — and their gradients in (X, C, r), with the plane row's
/// dot against n handed back for the q columns: `|X − C| − r` and `n·(X − C)`.
///
/// The radius row is the **magnitude** form, not the squared one the page's `point_on_circle`
/// uses: a kernel has one degree and the plane row is a length, so the radius row is stated as a
/// length too — which is also the better-conditioned of the two, having a unit gradient wherever
/// X is off the centre.  Both rows degree 1.
fn circle3_rows(v: &[f64], nv: [f64; 3], j0: &mut [f64], j1: &mut [f64]) -> ([f64; 2], [f64; 3]) {
    let d = sub3(at3(v, 0), at3(v, 3));
    let (l, u) = length_and_unit(d);
    for t in 0..3 {
        j0[t] = u[t];
        j0[3 + t] = -u[t];
        j1[t] = nv[t];
        j1[3 + t] = -nv[t];
    }
    j0[6] = -1.0;
    j1[6] = 0.0;
    ([l - v[6], dot3(nv, d)], d)
}

/// Columns of `point_on_circle3`: (X, C, r, qw, qx, qy, qz) — the circle's view solved, its
/// normal read off the quaternion.  Two rows, net two equations: on the sphere of the circle's
/// radius about its centre, and on its plane.
pub const N_PAR_POINT_ON_CIRCLE3: usize = 11;

fn point_on_circle3_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    let (mut j0, mut j1) = ([0.0; 7], [0.0; 7]);
    for i in 0..n {
        let o = N_PAR_POINT_ON_CIRCLE3 * i;
        let q = quat_at(v, o + 7);
        let Some((nv, _)) = quat_normal(q) else {
            r[2 * i..2 * i + 2].fill(f64::NAN);
            continue;
        };
        let (rows, _) = circle3_rows(&v[o..], nv, &mut j0, &mut j1);
        r[2 * i..2 * i + 2].copy_from_slice(&rows);
    }
}

fn point_on_circle3_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    const W: usize = N_PAR_POINT_ON_CIRCLE3;
    for i in 0..n {
        let o = W * i;
        let jo = 2 * W * i;
        let q = quat_at(v, o + 7);
        let Some((nv, dq)) = quat_normal(q) else {
            j[jo..jo + 2 * W].fill(f64::NAN);
            continue;
        };
        let (j0, j1) = j[jo..jo + 2 * W].split_at_mut(W);
        let (_, d) = circle3_rows(&v[o..], nv, j0, j1);
        for m in 0..4 {
            j0[7 + m] = 0.0;
            j1[7 + m] = (0..3).map(|t| d[t] * dq[t][m]).sum();
        }
    }
}

/// Columns of `point_on_circle3_fixed`: (X, C, r), K = (n) — the circle's view stated.
fn point_on_circle3_fixed_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let (mut j0, mut j1) = ([0.0; 7], [0.0; 7]);
    for i in 0..n {
        let (rows, _) = circle3_rows(&v[7 * i..], at3(k, 3 * i), &mut j0, &mut j1);
        r[2 * i..2 * i + 2].copy_from_slice(&rows);
    }
}

fn point_on_circle3_fixed_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let (j0, j1) = j[14 * i..14 * i + 14].split_at_mut(7);
        circle3_rows(&v[7 * i..], at3(k, 3 * i), j0, j1);
    }
}

/* -- hinges and the projection between solved views ------------------------------------------
 *
 * A view folded from a *solved* view (`docs/spatial-constraints-plan.md`) is not a constant
 * of the document: its attitude follows its parent's, turned by the fold.  `Basis::fold(θ)` is
 * the rotation `Rz(θ)·Rx(−90°)` in the parent's own axes — `u = cos θ·u_P + sin θ·v_P`,
 * `v = −n_P` — so the child's quaternion is the parent's times that one, `q_P ⊗ q_rel(θ)`, and a
 * hinge is the four rows saying so.  No unit row on the child: a product of unit quaternions is
 * one.  The fold is a constant (`hinge`, and the identity for a plane stood off its parent), the
 * document's free variable (`hinge_free`), or the bearing of a line drawn in the parent
 * (`hinge_along`, over a half-angle rotor of its own).
 */

/// `qz(θ) ⊗ qx(−90°)` and its derivative in θ: the turn a fold at bearing θ is, in the parent's
/// own axes (`plane::fold_rotor`, the one statement of it).
#[inline]
fn fold_rel(theta: f64) -> ([f64; 4], [f64; 4]) {
    let (s, c) = (0.5 * theta).dsin_cos();
    let d = crate::plane::fold_turn(-0.5 * s, 0.5 * c);
    (crate::plane::fold_rotor(theta), d)
}

/// The four hinge rows `q_c − q_p ⊗ k` and their Jacobian in (q_c, q_p), written into `j` at
/// `stride` per row.
#[inline]
fn hinge_rows(v: &[f64], k: [f64; 4], r: &mut [f64], j: Option<(&mut [f64], usize)>) {
    let qp = quat_at(v, 4);
    let p = crate::plane::quat_mul(qp, k);
    for t in 0..4 {
        r[t] = v[t] - p[t];
    }
    if let Some((j, stride)) = j {
        for m in 0..4 {
            let mut e = [0.0; 4];
            e[m] = 1.0;
            let d = crate::plane::quat_mul(e, k);
            for t in 0..4 {
                j[t * stride + m] = if t == m { 1.0 } else { 0.0 };
                j[t * stride + 4 + m] = -d[t];
            }
        }
    }
}

/// Columns of `hinge`: (q_c, q_p) — the child view's quaternion and its parent's.  Constants:
/// `q_rel`, the fold's turn (`fold_rel`) or the identity for a plane stood off its parent.  Four
/// dimensionless rows, degree 0.
fn hinge_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let kk = [k[4 * i], k[4 * i + 1], k[4 * i + 2], k[4 * i + 3]];
        hinge_rows(&v[8 * i..], kk, &mut r[4 * i..4 * i + 4], None);
    }
}

fn hinge_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let mut r = [0.0; 4];
    for i in 0..n {
        let kk = [k[4 * i], k[4 * i + 1], k[4 * i + 2], k[4 * i + 3]];
        hinge_rows(&v[8 * i..], kk, &mut r, Some((&mut j[32 * i..32 * i + 32], 8)));
    }
}

/// Columns of `hinge_free`: (q_c, q_p, a), K = (m, c) — the fold the document's free variable
/// `a` makes, θ = m·a + c (radians).
fn hinge_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        let (th, _) = free_dim(v, k, i, o + 8);
        hinge_rows(&v[o..], fold_rel(th).0, &mut r[4 * i..4 * i + 4], None);
    }
}

fn hinge_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let mut r = [0.0; 4];
    for i in 0..n {
        let o = 9 * i;
        let (th, m) = free_dim(v, k, i, o + 8);
        let (rel, drel) = fold_rel(th);
        let jo = 36 * i;
        hinge_rows(&v[o..], rel, &mut r, Some((&mut j[jo..jo + 36], 9)));
        let d = crate::plane::quat_mul(quat_at(v, o + 4), drel);
        for t in 0..4 {
            j[jo + 9 * t + 8] = -d[t] * m;
        }
    }
}

/// Columns of `hinge_along`: (q_c, q_p, hc, hs, c, s, p1x, p1y, p2x, p2y) — the child view, its
/// parent, the fold's half-angle rotor, the parent datum's rotor and the line's two ends.
pub const N_PAR_HINGE_ALONG: usize = 16;

/// `fold: along l` — the child view contains the direction of a line drawn in its parent.  Six
/// rows, all dimensionless (degree 0): the four hinge rows over `q_rel = qz(h) ⊗ qx(−90°)` with
/// `qz(h) = (hc, 0, 0, hs)`, the rotor on the unit circle, and the fold's bearing along the line,
/// `(cos θ, sin θ) × Rᵀ(c, s)(p2 − p1) / |p2 − p1| = 0` with `cos θ = hc² − hs²`,
/// `sin θ = 2·hc·hs` — the line read in its view, `plane::in_view`'s rotation.  Either way along
/// the line is a solution, and the seed picks one; where the line crosses the fold is the
/// `point_on_plane` row the elaborator states beside it.
fn hinge_along_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    const W: usize = N_PAR_HINGE_ALONG;
    for i in 0..n {
        let o = W * i;
        let (hc, hs) = (v[o + 8], v[o + 9]);
        let rel = crate::plane::fold_turn(hc, hs);
        hinge_rows(&v[o..], rel, &mut r[6 * i..6 * i + 4], None);
        r[6 * i + 4] = hc * hc + hs * hs - 1.0;
        r[6 * i + 5] = along_row(&v[o..], None);
    }
}

/// The bearing row of `hinge_along` and, when asked, its gradient over the 16 columns.
fn along_row(v: &[f64], g: Option<&mut [f64]>) -> f64 {
    let (hc, hs, c, s) = (v[8], v[9], v[10], v[11]);
    let (dx, dy) = (v[14] - v[12], v[15] - v[13]);
    let l = dx.dhypot(dy).max(MIN_LINE_LEN);
    let (ex, ey) = (c * dx + s * dy, -s * dx + c * dy);
    let (cc, ss) = (hc * hc - hs * hs, 2.0 * hc * hs);
    let nn = cc * ey - ss * ex;
    if let Some(g) = g {
        g[..8].fill(0.0);
        g[8] = (2.0 * hc * ey - 2.0 * hs * ex) / l;
        g[9] = (-2.0 * hs * ey - 2.0 * hc * ex) / l;
        g[10] = (cc * dy - ss * dx) / l;
        g[11] = (-cc * dx - ss * dy) / l;
        let gx = (cc * -s - ss * c) / l - nn * dx / (l * l * l);
        let gy = (cc * c - ss * s) / l - nn * dy / (l * l * l);
        g[12] = -gx;
        g[13] = -gy;
        g[14] = gx;
        g[15] = gy;
    }
    nn / l
}

fn hinge_along_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    const W: usize = N_PAR_HINGE_ALONG;
    let mut r = [0.0; 4];
    for i in 0..n {
        let o = W * i;
        let jo = 6 * W * i;
        let rows = &mut j[jo..jo + 6 * W];
        rows.fill(0.0);
        let (hc, hs) = (v[o + 8], v[o + 9]);
        let rel = crate::plane::fold_turn(hc, hs);
        hinge_rows(&v[o..], rel, &mut r, Some((&mut rows[..4 * W], W)));
        // ∂(q_p ⊗ qz(h) ⊗ x)/∂h: q_p ⊗ (∂qz ⊗ x), each of ∂qz a unit quaternion's axis — the
        // turn being linear in its rotor
        let qp = quat_at(v, o + 4);
        let dc = crate::plane::quat_mul(qp, crate::plane::fold_turn(1.0, 0.0));
        let ds = crate::plane::quat_mul(qp, crate::plane::fold_turn(0.0, 1.0));
        for t in 0..4 {
            rows[t * W + 8] = -dc[t];
            rows[t * W + 9] = -ds[t];
        }
        rows[4 * W + 8] = 2.0 * hc;
        rows[4 * W + 9] = 2.0 * hs;
        along_row(&v[o..], Some(&mut rows[5 * W..6 * W]));
    }
}

/// Columns of `project_free`: (X_A, X_B, q_A, q_B) — the two images' hidden points and the two
/// views' quaternions.
pub const N_PAR_PROJECT_FREE: usize = 14;

/// **The projector rule in space**: `(n_A × n_B)·(X_A − X_B) = 0`, with `n = R(q)·e₃`.  Two
/// images of one point differ by something in the span of the two normals — each is the point
/// less its depth along its own view's normal — so their difference has nothing along the fold
/// line `n_A × n_B` the views share.  Unnormalised, so it reads the stated `project` times the
/// sine between the views, and vanishes with it where they come out parallel (E065, after the
/// solve).  What `project` compiles to wherever either view is solved; degree 1.
fn project_free_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_PROJECT_FREE * i;
        let qa = quat_at(v, o + 6);
        let qb = quat_at(v, o + 10);
        r[i] = match (quat_normal(qa), quat_normal(qb)) {
            (Some((na, _)), Some((nb, _))) => {
                dot3(cross3(na, nb), sub3(at3(v, o), at3(v, o + 3)))
            }
            _ => f64::NAN,
        };
    }
}

fn project_free_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    const W: usize = N_PAR_PROJECT_FREE;
    for i in 0..n {
        let o = W * i;
        let row = &mut j[o..o + W];
        let qa = quat_at(v, o + 6);
        let qb = quat_at(v, o + 10);
        let (Some((na, da)), Some((nb, db))) = (quat_normal(qa), quat_normal(qb)) else {
            row.fill(f64::NAN);
            continue;
        };
        let w = sub3(at3(v, o), at3(v, o + 3));
        let m = cross3(na, nb);
        // m·w = n_A·(n_B × w) = n_B·(w × n_A)
        let (ga, gb) = (cross3(nb, w), cross3(w, na));
        for t in 0..3 {
            row[t] = m[t];
            row[3 + t] = -m[t];
        }
        for k in 0..4 {
            row[6 + k] = (0..3).map(|t| ga[t] * da[t][k]).sum();
            row[10 + k] = (0..3).map(|t| gb[t] * db[t][k]).sum();
        }
    }
}

/* -- the rest of the spatial words, and the sphere ---------------------------------------------
 *
 * A point on a line in space, two lines of equal true length, a point's signed distance from a
 * plane (`distance(along: n)`), and a sphere's two relations of its own: a point on it and two
 * spheres touching.  A sphere's radius and its tangency to a line reuse `radius` and
 * `point_line3_free` (the line's distance from the centre, stated as the radius column). */

/// Columns of `ray_unit`: (dx, dy, dz).  `|d|² − 1`: a direction held to the unit sphere,
/// dimensionless, degree 0 — `quat_unit` one dimension down.
fn ray_unit_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let d = at3(v, 3 * i);
        r[i] = dot3(d, d) - 1.0;
    }
}

fn ray_unit_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 3 * i;
        for t in 0..3 {
            j[o + t] = 2.0 * v[o + t];
        }
    }
}

/// Columns of `ray_foot`: (a, d).  `a·d`: the ray's point is the foot of the perpendicular from
/// the origin, so it cannot slide along the ray.  A length, degree 1.
fn ray_foot_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        r[i] = dot3(at3(v, 6 * i), at3(v, 6 * i + 3));
    }
}

fn ray_foot_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 6 * i;
        for t in 0..3 {
            j[o + t] = v[o + 3 + t];
            j[o + 3 + t] = v[o + t];
        }
    }
}

/// Columns of `point_on_ray`: (X, a, d), K = (e₁, e₂) — `point_on_line3` over a ray, whose
/// direction is `d` itself rather than `B − A`: `((X − a) × d)·e_k / |d| = 0`.  Degree 1.
fn point_on_ray_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
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

fn point_on_ray_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
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

/// `n(q)·X − d` and its gradient in (X, q, d): a hidden point's signed distance along a solved
/// plane's normal — `point_on_plane`'s row, and `point_plane_distance`'s less the number.
fn plane_gap(v: &[f64], row: &mut [f64]) -> f64 {
    let q = quat_at(v, 3);
    let Some((nv, dq)) = quat_normal(q) else {
        row[..8].fill(f64::NAN);
        return f64::NAN;
    };
    row[..3].copy_from_slice(&nv);
    for m in 0..4 {
        row[3 + m] = (0..3).map(|t| v[t] * dq[t][m]).sum();
    }
    row[7] = -1.0;
    dot3(nv, at3(v, 0)) - v[7]
}

/// Columns of `point_plane_distance`: (X, qw, qx, qy, qz, d), K = (D).  `distance(along: n)`
/// over a solved plane: the point stands D along the normal from it.  Degree 1.
fn point_plane_distance_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let mut g = [0.0; 8];
    for i in 0..n {
        r[i] = plane_gap(&v[8 * i..], &mut g) - k[i];
    }
}

fn point_plane_distance_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        plane_gap(&v[8 * i..], &mut j[8 * i..8 * i + 8]);
    }
}

/// (X, q, d, a), K = (m, c): the same, D = m·a + c.
fn point_plane_distance_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let mut g = [0.0; 8];
    for i in 0..n {
        let o = 9 * i;
        r[i] = plane_gap(&v[o..], &mut g) - free_dim(v, k, i, o + 8).0;
    }
}

fn point_plane_distance_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        plane_gap(&v[o..], &mut j[o..o + 8]);
        j[o + 8] = -k[2 * i];
    }
}

/// Columns of `point_plane_distance_fixed_free`: (X, a), K = (n, h, m, c) — a stated plane's
/// normal and its origin along it, and the free variable's (m, c).  `n·X − h − (m·a + c)`; the
/// stated number over a stated plane needs no kernel of its own, being `point_on_plane_fixed`
/// with D folded into h.
fn point_plane_distance_fixed_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let (o, kk) = (4 * i, &k[6 * i..6 * i + 6]);
        r[i] = dot3(at3(kk, 0), at3(v, o)) - kk[3] - (kk[4] * v[o + 3] + kk[5]);
    }
}

fn point_plane_distance_fixed_free_jac(n: usize, _v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let kk = &k[6 * i..6 * i + 6];
        j[4 * i..4 * i + 3].copy_from_slice(&kk[..3]);
        j[4 * i + 3] = -kk[4];
    }
}

/// Columns of `sphere_on`: (X, C, r).  `|X − C| − r`, a point in some view on a sphere about a
/// centre drawn in another: the magnitude, degree 1, with a unit gradient wherever X is off C.
fn sphere_on_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 7 * i;
        r[i] = norm3(sub3(at3(v, o), at3(v, o + 3))) - v[o + 6];
    }
}

fn sphere_on_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 7 * i;
        let (_, u) = length_and_unit(sub3(at3(v, o), at3(v, o + 3)));
        for t in 0..3 {
            j[o + t] = u[t];
            j[o + 3 + t] = -u[t];
        }
        j[o + 6] = -1.0;
    }
}

/// Columns of `sphere_sphere`: (C₁, C₂, r₁, r₂), K = (a, b).  `|C₁ − C₂| − (a·r₁ + b·r₂)`: two
/// spheres touching outside (a = b = 1) or inside (one of them −1, whichever the seed makes
/// positive), degree 1.
fn sphere_sphere_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 8 * i;
        let l = norm3(sub3(at3(v, o), at3(v, o + 3)));
        r[i] = l - (k[2 * i] * v[o + 6] + k[2 * i + 1] * v[o + 7]);
    }
}

fn sphere_sphere_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 8 * i;
        let (_, u) = length_and_unit(sub3(at3(v, o), at3(v, o + 3)));
        for t in 0..3 {
            j[o + t] = u[t];
            j[o + 3 + t] = -u[t];
        }
        j[o + 6] = -k[2 * i];
        j[o + 7] = -k[2 * i + 1];
    }
}

/// Columns of `line_on_plane`: (A, B, qw, qx, qy, qz, d) — a line's two hidden ends on a solved
/// plane, `point_on_plane`'s row once for each end.  Degree 1.
fn line_on_plane_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 11 * i;
        let q = quat_at(v, o + 6);
        for t in 0..2 {
            r[2 * i + t] = quat_normal(q)
                .map_or(f64::NAN, |(nv, _)| dot3(nv, at3(v, o + 3 * t)) - v[o + 10]);
        }
    }
}

fn line_on_plane_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 11 * i;
        let q = quat_at(v, o + 6);
        let rows = &mut j[22 * i..22 * i + 22];
        let Some((nv, dq)) = quat_normal(q) else {
            rows.fill(f64::NAN);
            continue;
        };
        rows.fill(0.0);
        for t in 0..2 {
            let row = &mut rows[11 * t..11 * (t + 1)];
            row[3 * t..3 * t + 3].copy_from_slice(&nv);
            for m in 0..4 {
                row[6 + m] = (0..3).map(|s| v[o + 3 * t + s] * dq[s][m]).sum();
            }
            row[10] = -1.0;
        }
    }
}

/// Columns of `line_on_plane_fixed`: (A, B), K = (n, h) — the same over a stated plane.
fn line_on_plane_fixed_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let kk = &k[4 * i..];
        for t in 0..2 {
            r[2 * i + t] = dot3(at3(kk, 0), at3(v, 6 * i + 3 * t)) - kk[3];
        }
    }
}

fn line_on_plane_fixed_jac(n: usize, _v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let rows = &mut j[12 * i..12 * i + 12];
        rows.fill(0.0);
        for t in 0..2 {
            rows[6 * t + 3 * t..6 * t + 3 * t + 3].copy_from_slice(&k[4 * i..4 * i + 3]);
        }
    }
}

/* -- a circle on a sphere, and the midpoint and the mirror in space ------------------------ */

/// The three rows of a circle on a sphere — the circle's centre lifted to C, its radius r and its
/// view's in-plane axes u and v; the sphere's centre S and radius R — and their gradients in
/// (C, S, r, R), with the two axis rows' `S − C` handed back for the q columns:
/// `u·(S − C)`, `v·(S − C)` (the sphere's centre on the circle's axis) and `√(|S − C|² + r²) − R`
/// (every point of the circle at R from it).  All degree 1; the third has a gradient wherever the
/// circle has a radius.
fn circle_sphere_rows(v: &[f64], u: [f64; 3], w: [f64; 3], j: [&mut [f64]; 3]) -> ([f64; 3], [f64; 3]) {
    let d = sub3(at3(v, 3), at3(v, 0));
    let r = v[6];
    let l = (dot3(d, d) + r * r).sqrt();
    let [j0, j1, j2] = j;
    for t in 0..3 {
        j0[t] = -u[t];
        j0[3 + t] = u[t];
        j1[t] = -w[t];
        j1[3 + t] = w[t];
        let g = if l > 0.0 { d[t] / l } else { 0.0 };
        j2[t] = -g;
        j2[3 + t] = g;
    }
    j0[6] = 0.0;
    j0[7] = 0.0;
    j1[6] = 0.0;
    j1[7] = 0.0;
    j2[6] = if l > 0.0 { r / l } else { 0.0 };
    j2[7] = -1.0;
    ([dot3(u, d), dot3(w, d), l - v[7]], d)
}

/// Columns of `circle_on_sphere`: (C, S, r, R, qw, qx, qy, qz) — the circle's view solved, its
/// axes read off the quaternion.  Three rows, net three equations.
pub const N_PAR_CIRCLE_ON_SPHERE: usize = 12;

fn quat_axes(q: [f64; 4]) -> Option<([[f64; 3]; 2], [[[f64; 4]; 3]; 2])> {
    let (u, _, du) = crate::plane::lift_q(q, [1.0, 0.0, 0.0])?;
    let (w, _, dw) = crate::plane::lift_q(q, [0.0, 1.0, 0.0])?;
    Some(([u, w], [du, dw]))
}

fn circle_on_sphere_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    const W: usize = N_PAR_CIRCLE_ON_SPHERE;
    let mut scratch = [0.0; 3 * W];
    for i in 0..n {
        let o = W * i;
        let q = quat_at(v, o + 8);
        let Some(([u, w], _)) = quat_axes(q) else {
            r[3 * i..3 * i + 3].fill(f64::NAN);
            continue;
        };
        let (a, rest) = scratch.split_at_mut(W);
        let (b, c) = rest.split_at_mut(W);
        let (rows, _) = circle_sphere_rows(&v[o..], u, w, [a, b, c]);
        r[3 * i..3 * i + 3].copy_from_slice(&rows);
    }
}

fn circle_on_sphere_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    const W: usize = N_PAR_CIRCLE_ON_SPHERE;
    for i in 0..n {
        let o = W * i;
        let jo = 3 * W * i;
        let q = quat_at(v, o + 8);
        let Some(([u, w], [du, dw])) = quat_axes(q) else {
            j[jo..jo + 3 * W].fill(f64::NAN);
            continue;
        };
        let (a, rest) = j[jo..jo + 3 * W].split_at_mut(W);
        let (b, c) = rest.split_at_mut(W);
        let (_, d) = circle_sphere_rows(&v[o..], u, w, [&mut *a, &mut *b, &mut *c]);
        for m in 0..4 {
            a[8 + m] = (0..3).map(|t| d[t] * du[t][m]).sum();
            b[8 + m] = (0..3).map(|t| d[t] * dw[t][m]).sum();
            c[8 + m] = 0.0;
        }
    }
}

/// Columns of `circle_on_sphere_fixed`: (C, S, r, R), K = (u, v) — the circle's view stated.
fn circle_on_sphere_fixed_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let mut scratch = [0.0; 24];
    for i in 0..n {
        let (a, rest) = scratch.split_at_mut(8);
        let (b, c) = rest.split_at_mut(8);
        let (rows, _) = circle_sphere_rows(&v[8 * i..], at3(k, 6 * i), at3(k, 6 * i + 3), [a, b, c]);
        r[3 * i..3 * i + 3].copy_from_slice(&rows);
    }
}

fn circle_on_sphere_fixed_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let (a, rest) = j[24 * i..24 * i + 24].split_at_mut(8);
        let (b, c) = rest.split_at_mut(8);
        circle_sphere_rows(&v[8 * i..], at3(k, 6 * i), at3(k, 6 * i + 3), [a, b, c]);
    }
}

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

/* -- cones and cylinders -----------------------------------------------------------------------
 *
 * A cylinder's relations reuse the kernels a sphere's do — a point on it is `point_line3_free`
 * (its distance from the axis, stated as the radius column), a line touching it `line_line3_free`
 * (the common perpendicular with the axis), its radius `radius`.  A cone's are new: a point on
 * it, its half-angle (an angle row of degree 0, the radius kernel's arithmetic), and two cones
 * touching at a point.  Their derivatives are taken by `Dual`, a forward-mode number carrying
 * the gradient in every column of the block — exact, and one expression for the residual and its
 * row, where the hand-derived chains of the kernels above would be a page of vector calculus to
 * get wrong. */

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
    fn sin(self) -> Self {
        self.map(self.v.dsin(), self.v.dcos())
    }
    fn cos(self) -> Self {
        self.map(self.v.dcos(), -self.v.dsin())
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

type V3<const N: usize> = [Dual<N>; 3];

fn dvec<const N: usize>(v: &[f64], at: usize) -> V3<N> {
    [0, 1, 2].map(|t| Dual::var(v[at + t], at + t))
}

fn dsub<const N: usize>(a: V3<N>, b: V3<N>) -> V3<N> {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn ddot<const N: usize>(a: V3<N>, b: V3<N>) -> Dual<N> {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn dcross<const N: usize>(a: V3<N>, b: V3<N>) -> V3<N> {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// The unit vector along `a`; a vector shorter than `MIN_LINE_LEN` is divided by that instead,
/// as every kernel above guards a degenerate line.
fn dunit<const N: usize>(a: V3<N>) -> V3<N> {
    let l = ddot(a, a).sqrt();
    let l = if l.v > MIN_LINE_LEN { l } else { Dual { v: MIN_LINE_LEN, g: [0.0; N] } };
    a.map(|x| x / l)
}

/// A cone at a point X, read in X's meridian half-plane: the unit axis ê (apex A toward B), the
/// height `h = (X − A)·ê`, the distance ρ from the axis and the unit radial direction û.
struct Meridian<const N: usize> {
    e: V3<N>,
    h: Dual<N>,
    rho: Dual<N>,
    u: V3<N>,
}

fn meridian<const N: usize>(x: V3<N>, a: V3<N>, b: V3<N>) -> Meridian<N> {
    let e = dunit(dsub(b, a));
    let w = dsub(x, a);
    let h = ddot(w, e);
    let radial = dsub(w, e.map(|t| t * h));
    let rho = ddot(radial, radial).sqrt();
    let u = dunit(radial);
    Meridian { e, h, rho, u }
}

/// `ρ cos α − h sin α` over (X, A, B, α): how far X stands from the cone's generator in its
/// meridian half-plane — zero on the nappe the axis points into, a length.
fn cone_gap(v: &[f64]) -> Dual<10> {
    let m = meridian::<10>(dvec(v, 0), dvec(v, 3), dvec(v, 6));
    let al = Dual::<10>::var(v[9], 9);
    m.rho * al.cos() - m.h * al.sin()
}

/// Columns of `cone_on`: (X, A, B, α) — a point's hidden point, the axis's two, the cone's
/// half-angle.  `ρ cos α − h sin α`, degree 1.
fn cone_on_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        r[i] = cone_gap(&v[10 * i..10 * i + 10]).v;
    }
}

fn cone_on_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let d = cone_gap(&v[10 * i..10 * i + 10]);
        j[10 * i..10 * i + 10].copy_from_slice(&d.g);
    }
}

pub const N_PAR_CONE_CONE: usize = 17;

/// Two cones at M: the second's surface normal against the first's generator and circle
/// directions there.  Over (M, A₁, B₁, α₁, A₂, B₂, α₂).  A cone's outward normal at a point of
/// its meridian is `û cos α − ê sin α`, its generator `ê cos α + û sin α` and its circle `ê × û`.
fn cone_contact(v: &[f64]) -> [Dual<N_PAR_CONE_CONE>; 2] {
    const N: usize = N_PAR_CONE_CONE;
    let x = dvec::<N>(v, 0);
    let (m1, m2) = (meridian(x, dvec(v, 3), dvec(v, 6)), meridian(x, dvec(v, 10), dvec(v, 13)));
    let (a1, a2) = (Dual::<N>::var(v[9], 9), Dual::<N>::var(v[16], 16));
    let (c1, s1, c2, s2) = (a1.cos(), a1.sin(), a2.cos(), a2.sin());
    let normal = [0, 1, 2].map(|t| m2.u[t] * c2 - m2.e[t] * s2);
    let generator = [0, 1, 2].map(|t| m1.e[t] * c1 + m1.u[t] * s1);
    let circle = dcross(m1.e, m1.u);
    [ddot(normal, generator), ddot(normal, circle)]
}

/// Columns of `cone_cone`: (M, A₁, B₁, α₁, A₂, B₂, α₂).  Two rows, degree 0: the second cone's
/// normal at M square to the first's two tangent directions there — one tangent plane at M.
fn cone_cone_res(n: usize, v: &[f64], _k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_CONE_CONE * i;
        let d = cone_contact(&v[o..o + N_PAR_CONE_CONE]);
        r[2 * i] = d[0].v;
        r[2 * i + 1] = d[1].v;
    }
}

fn cone_cone_jac(n: usize, v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = N_PAR_CONE_CONE * i;
        let d = cone_contact(&v[o..o + N_PAR_CONE_CONE]);
        let jo = 2 * N_PAR_CONE_CONE * i;
        j[jo..jo + N_PAR_CONE_CONE].copy_from_slice(&d[0].g);
        j[jo + N_PAR_CONE_CONE..jo + 2 * N_PAR_CONE_CONE].copy_from_slice(&d[1].g);
    }
}

/* -- a mate between solved views ----------------------------------------------------------- */

const MATE_J: &[f64] = &[1.0, -1.0];

/// Columns of `mate`: (d_f, d_g), K = (gap).  `d_f − d_g − gap`: a placed view's offset along
/// the normal it shares with the view it bears on, held at that view's offset and the gap between
/// the two faces' ordinates.  Degree 1.
fn mate_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        r[i] = v[2 * i] - v[2 * i + 1] - k[i];
    }
}

fn mate_jac(n: usize, _v: &[f64], _k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        j[2 * i..2 * i + 2].copy_from_slice(MATE_J);
    }
}

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
    Kernel { name: "point_on_spline", n_res: 2, n_par: N_PAR_ON_SPLINE, degree: 1, n_const: SPAN_K, res: point_on_spline_res, jac: point_on_spline_jac, const_jac: None },
    Kernel { name: "spline_tangent_line", n_res: 2, n_par: N_PAR_SPLINE_LINE, degree: 1, n_const: SPAN_K, res: spline_tangent_line_res, jac: spline_tangent_line_jac, const_jac: None },
    Kernel { name: "spline_curvature", n_res: 3, n_par: N_PAR_SPLINE_CURVE, degree: 1, n_const: SPAN_K, res: spline_curvature_res, jac: spline_curvature_jac, const_jac: None },
    Kernel { name: "horizontal_distance", n_res: 1, n_par: 4, degree: 1, n_const: 1, res: horizontal_distance_res, jac: horizontal_distance_jac, const_jac: Some(HORIZONTAL_DISTANCE_J) },
    Kernel { name: "vertical_distance", n_res: 1, n_par: 4, degree: 1, n_const: 1, res: vertical_distance_res, jac: vertical_distance_jac, const_jac: Some(VERTICAL_DISTANCE_J) },
    Kernel { name: "distance_free", n_res: 1, n_par: 5, degree: 2, n_const: 2, res: distance_free_res, jac: distance_free_jac, const_jac: None },
    Kernel { name: "angle_free", n_res: 1, n_par: 9, degree: 0, n_const: 2, res: angle_free_res, jac: angle_free_jac, const_jac: None },
    Kernel { name: "radius_free", n_res: 1, n_par: 2, degree: 1, n_const: 2, res: radius_free_res, jac: radius_free_jac, const_jac: None },
    Kernel { name: "parallel_distance_free", n_res: 1, n_par: 9, degree: 1, n_const: 2, res: parallel_distance_free_res, jac: parallel_distance_free_jac, const_jac: None },
    Kernel { name: "point_line_distance_free", n_res: 1, n_par: 7, degree: 1, n_const: 2, res: point_line_distance_free_res, jac: point_line_distance_free_jac, const_jac: None },
    Kernel { name: "annular_distance_free", n_res: 1, n_par: 3, degree: 1, n_const: 2, res: annular_distance_free_res, jac: annular_distance_free_jac, const_jac: None },
    Kernel { name: "horizontal_distance_free", n_res: 1, n_par: 5, degree: 1, n_const: 2, res: horizontal_distance_free_res, jac: horizontal_distance_free_jac, const_jac: None },
    Kernel { name: "vertical_distance_free", n_res: 1, n_par: 5, degree: 1, n_const: 2, res: vertical_distance_free_res, jac: vertical_distance_free_jac, const_jac: None },
    Kernel { name: "frame_unit", n_res: 1, n_par: 2, degree: 0, n_const: 0, res: frame_unit_res, jac: frame_unit_jac, const_jac: None },
    Kernel { name: "frame_align", n_res: 2, n_par: N_PAR_FRAME_ALIGN, degree: 1, n_const: 0, res: frame_align_res, jac: frame_align_jac, const_jac: None },
    Kernel { name: "project", n_res: 1, n_par: N_PAR_PROJECT, degree: 1, n_const: 4, res: project_res, jac: project_jac, const_jac: None },
    Kernel { name: "point_line_magnitude", n_res: 1, n_par: 6, degree: 1, n_const: 1, res: point_line_magnitude_res, jac: point_line_magnitude_jac, const_jac: None },
    Kernel { name: "point_line_magnitude_free", n_res: 1, n_par: 7, degree: 1, n_const: 2, res: point_line_magnitude_free_res, jac: point_line_magnitude_free_jac, const_jac: None },
    Kernel { name: "parallel_magnitude", n_res: 1, n_par: 8, degree: 1, n_const: 1, res: parallel_magnitude_res, jac: parallel_magnitude_jac, const_jac: None },
    Kernel { name: "parallel_magnitude_free", n_res: 1, n_par: 9, degree: 1, n_const: 2, res: parallel_magnitude_free_res, jac: parallel_magnitude_free_jac, const_jac: None },
    Kernel { name: "coordinate_u", n_res: 1, n_par: 6, degree: 1, n_const: 1, res: coordinate_res::<false, false>, jac: coordinate_jac::<false, false>, const_jac: None },
    Kernel { name: "coordinate_v", n_res: 1, n_par: 6, degree: 1, n_const: 1, res: coordinate_res::<true, false>, jac: coordinate_jac::<true, false>, const_jac: None },
    Kernel { name: "coordinate_u_free", n_res: 1, n_par: 7, degree: 1, n_const: 2, res: coordinate_res::<false, true>, jac: coordinate_jac::<false, true>, const_jac: None },
    Kernel { name: "coordinate_v_free", n_res: 1, n_par: 7, degree: 1, n_const: 2, res: coordinate_res::<true, true>, jac: coordinate_jac::<true, true>, const_jac: None },
    Kernel { name: "quat_unit", n_res: 1, n_par: 4, degree: 0, n_const: 0, res: quat_unit_res, jac: quat_unit_jac, const_jac: None },
    Kernel { name: "lift", n_res: 3, n_par: N_PAR_LIFT, degree: 1, n_const: 2, res: lift_res, jac: lift_jac, const_jac: None },
    Kernel { name: "lift_fixed", n_res: 3, n_par: N_PAR_LIFT_FIXED, degree: 1, n_const: 9, res: lift_fixed_res, jac: lift_fixed_jac, const_jac: None },
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
    Kernel { name: "point_on_plane", n_res: 1, n_par: N_PAR_POINT_ON_PLANE, degree: 1, n_const: 0, res: point_on_plane_res, jac: point_on_plane_jac, const_jac: None },
    Kernel { name: "point_on_plane_fixed", n_res: 1, n_par: 3, degree: 1, n_const: 4, res: point_on_plane_fixed_res, jac: point_on_plane_fixed_jac, const_jac: None },
    Kernel { name: "point_on_circle3", n_res: 2, n_par: N_PAR_POINT_ON_CIRCLE3, degree: 1, n_const: 0, res: point_on_circle3_res, jac: point_on_circle3_jac, const_jac: None },
    Kernel { name: "point_on_circle3_fixed", n_res: 2, n_par: 7, degree: 1, n_const: 3, res: point_on_circle3_fixed_res, jac: point_on_circle3_fixed_jac, const_jac: None },
    Kernel { name: "hinge", n_res: 4, n_par: 8, degree: 0, n_const: 4, res: hinge_res, jac: hinge_jac, const_jac: None },
    Kernel { name: "hinge_free", n_res: 4, n_par: 9, degree: 0, n_const: 2, res: hinge_free_res, jac: hinge_free_jac, const_jac: None },
    Kernel { name: "hinge_along", n_res: 6, n_par: N_PAR_HINGE_ALONG, degree: 0, n_const: 0, res: hinge_along_res, jac: hinge_along_jac, const_jac: None },
    Kernel { name: "project_free", n_res: 1, n_par: N_PAR_PROJECT_FREE, degree: 1, n_const: 0, res: project_free_res, jac: project_free_jac, const_jac: None },
    Kernel { name: "point_on_line3", n_res: 2, n_par: 9, degree: 1, n_const: 6, res: point_on_line3_res, jac: point_on_line3_jac, const_jac: None },
    Kernel { name: "equal_length3", n_res: 1, n_par: 12, degree: 2, n_const: 0, res: equal_length3_res, jac: equal_length3_jac, const_jac: None },
    Kernel { name: "point_plane_distance", n_res: 1, n_par: 8, degree: 1, n_const: 1, res: point_plane_distance_res, jac: point_plane_distance_jac, const_jac: None },
    Kernel { name: "point_plane_distance_free", n_res: 1, n_par: 9, degree: 1, n_const: 2, res: point_plane_distance_free_res, jac: point_plane_distance_free_jac, const_jac: None },
    Kernel { name: "point_plane_distance_fixed_free", n_res: 1, n_par: 4, degree: 1, n_const: 6, res: point_plane_distance_fixed_free_res, jac: point_plane_distance_fixed_free_jac, const_jac: None },
    Kernel { name: "sphere_on", n_res: 1, n_par: 7, degree: 1, n_const: 0, res: sphere_on_res, jac: sphere_on_jac, const_jac: None },
    Kernel { name: "sphere_sphere", n_res: 1, n_par: 8, degree: 1, n_const: 2, res: sphere_sphere_res, jac: sphere_sphere_jac, const_jac: None },
    Kernel { name: "line_on_plane", n_res: 2, n_par: 11, degree: 1, n_const: 0, res: line_on_plane_res, jac: line_on_plane_jac, const_jac: None },
    Kernel { name: "line_on_plane_fixed", n_res: 2, n_par: 6, degree: 1, n_const: 4, res: line_on_plane_fixed_res, jac: line_on_plane_fixed_jac, const_jac: None },
    Kernel { name: "circle_on_sphere", n_res: 3, n_par: N_PAR_CIRCLE_ON_SPHERE, degree: 1, n_const: 0, res: circle_on_sphere_res, jac: circle_on_sphere_jac, const_jac: None },
    Kernel { name: "circle_on_sphere_fixed", n_res: 3, n_par: 8, degree: 1, n_const: 6, res: circle_on_sphere_fixed_res, jac: circle_on_sphere_fixed_jac, const_jac: None },
    Kernel { name: "midpoint3", n_res: 3, n_par: 9, degree: 1, n_const: 0, res: midpoint3_res, jac: midpoint3_jac, const_jac: None },
    Kernel { name: "symmetric3", n_res: 3, n_par: 12, degree: 1, n_const: 0, res: symmetric3_res, jac: symmetric3_jac, const_jac: None },
    Kernel { name: "cone_on", n_res: 1, n_par: 10, degree: 1, n_const: 0, res: cone_on_res, jac: cone_on_jac, const_jac: None },
    Kernel { name: "half_angle", n_res: 1, n_par: 1, degree: 0, n_const: 1, res: radius_res, jac: radius_jac, const_jac: Some(RADIUS_J) },
    Kernel { name: "half_angle_free", n_res: 1, n_par: 2, degree: 0, n_const: 2, res: radius_free_res, jac: radius_free_jac, const_jac: None },
    Kernel { name: "cone_cone", n_res: 2, n_par: N_PAR_CONE_CONE, degree: 0, n_const: 0, res: cone_cone_res, jac: cone_cone_jac, const_jac: None },
    Kernel { name: "mate", n_res: 1, n_par: 2, degree: 1, n_const: 1, res: mate_res, jac: mate_jac, const_jac: Some(MATE_J) },
    Kernel { name: "equal_angle", n_res: 1, n_par: 16, degree: 0, n_const: 1, res: equal_angle_res, jac: equal_angle_jac, const_jac: None },
    Kernel { name: "arc_length", n_res: 1, n_par: 7, degree: 1, n_const: 1, res: arc_length_res, jac: arc_length_jac, const_jac: None },
    Kernel { name: "arc_length_free", n_res: 1, n_par: 8, degree: 1, n_const: 2, res: arc_length_free_res, jac: arc_length_free_jac, const_jac: None },
    Kernel { name: "ray_unit", n_res: 1, n_par: 3, degree: 0, n_const: 0, res: ray_unit_res, jac: ray_unit_jac, const_jac: None },
    Kernel { name: "ray_foot", n_res: 1, n_par: 6, degree: 1, n_const: 0, res: ray_foot_res, jac: ray_foot_jac, const_jac: None },
    Kernel { name: "point_on_ray", n_res: 2, n_par: 9, degree: 1, n_const: 6, res: point_on_ray_res, jac: point_on_ray_jac, const_jac: None },
];

/// One row of a kernel: residual and Jacobian for a single constraint's local values.  The
/// scalar view of the vectorized kernels, kept for the finite-difference checker and reporting.
pub fn eval_one(id: usize, v: &[f64], c: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let k = &KERNELS[id];
    let mut r = vec![0.0; k.n_res];
    let mut j = vec![0.0; k.n_res * k.n_par];
    (k.res)(1, v, c, &mut r);
    (k.jac)(1, v, c, &mut j);
    (r, j)
}
