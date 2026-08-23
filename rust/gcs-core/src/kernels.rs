//! Residual / Jacobian kernels — one per constraint type, evaluated for a whole block of
//! same-typed constraints per call.
//!
//! `v` is (n * n_par) local parameter values, `k` is (n * n_const) constants, `r` is (n * n_res)
//! residuals and `j` is (n * n_res * n_par).  Column conventions match the `params` tuples the
//! model builds; see the comment above each kernel.  Residual forms follow the program: squared
//! distances (no sqrt), a determinant for parallel, dot/cross for angle, signed distance minus
//! radius for tangency.
//!
//! The order of `KERNELS` **is** the kernel id and is part of the plan ABI.

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
}

pub const N_KERNELS: usize = 34;

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
    dx.hypot(dy).max(MIN_LINE_LEN)
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

/// The dot and the cross of the two directions — what every angle form is written from.
#[inline]
fn dot_cross(v: &[f64]) -> (f64, f64) {
    let (d1x, d1y, d2x, d2y) = dirs(v);
    (d1x * d2x + d1y * d2y, d1x * d2y - d1y * d2x)
}

/// `dot·sin θ − cross·cos θ`: zero exactly when the angle from l1 to l2 is θ, whether θ is a
/// stated number or an unknown.  The caller passes its sine and cosine, which is where the two
/// forms differ — one has them precomputed as constants, the other takes them of a column.
#[inline]
fn angle_gap(v: &[f64], s: f64, c: f64) -> f64 {
    let (dot, cross) = dot_cross(v);
    dot * s - cross * c
}

/// Its gradient in the eight direction columns.
#[inline]
fn angle_gap_jac(v: &[f64], s: f64, c: f64, j: &mut [f64]) {
    let (mut jd, mut jc) = ([0.0f64; 8], [0.0f64; 8]);
    dot_jac(v, &mut jd);
    cross_jac(v, &mut jc);
    for t in 0..8 {
        j[t] = jd[t] * s - jc[t] * c;
    }
}

/// K = (sin theta, cos theta): dot*sin - cross*cos
fn angle_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        r[i] = angle_gap(&v[8 * i..], k[2 * i], k[2 * i + 1]);
    }
}

fn angle_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 8 * i;
        angle_gap_jac(&v[o..], k[2 * i], k[2 * i + 1], &mut j[o..o + 8]);
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

/// (a1x,a1y,b1x,b1y,a2x,a2y,b2x,b2y,a), K = (m,c): dot*sin θ - cross*cos θ, θ = m*a + c.
/// The angle itself is the unknown here rather than its sine and cosine, so the two constants
/// the stated form carries precomputed are the affine map instead and the trigonometry moves
/// into the kernel — which is what lets the column carry a derivative at all.
fn angle_free_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        let (t, _) = free_dim(v, k, i, o + 8);
        r[i] = angle_gap(&v[o..], t.sin(), t.cos());
    }
}

fn angle_free_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    for i in 0..n {
        let o = 9 * i;
        let (t, m) = free_dim(v, k, i, o + 8);
        let (s, c) = (t.sin(), t.cos());
        angle_gap_jac(&v[o..], s, c, &mut j[o..o + 8]);
        // d/dθ of (dot·sin θ − cross·cos θ), through the affine map
        let (dot, cross) = dot_cross(&v[o..]);
        j[o + 8] = (dot * c + cross * s) * m;
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

pub static KERNELS: [Kernel; N_KERNELS] = [
    Kernel { name: "coincident", n_res: 2, n_par: 4, degree: 1, n_const: 0, res: coincident::res, jac: coincident::jac, const_jac: Some(coincident::J) },
    Kernel { name: "distance", n_res: 1, n_par: 4, degree: 2, n_const: 1, res: distance_res, jac: distance_jac, const_jac: None },
    Kernel { name: "midpoint", n_res: 2, n_par: 6, degree: 1, n_const: 0, res: midpoint::res, jac: midpoint::jac, const_jac: Some(midpoint::J) },
    Kernel { name: "drag", n_res: 2, n_par: 2, degree: 1, n_const: 3, res: drag_res, jac: drag_jac, const_jac: None },
    Kernel { name: "horizontal", n_res: 1, n_par: 4, degree: 1, n_const: 0, res: horizontal::res, jac: horizontal::jac, const_jac: Some(horizontal::J) },
    Kernel { name: "vertical", n_res: 1, n_par: 4, degree: 1, n_const: 0, res: vertical::res, jac: vertical::jac, const_jac: Some(vertical::J) },
    Kernel { name: "parallel", n_res: 1, n_par: 8, degree: 2, n_const: 0, res: parallel_res, jac: parallel_jac, const_jac: None },
    Kernel { name: "perpendicular", n_res: 1, n_par: 8, degree: 2, n_const: 0, res: perpendicular_res, jac: perpendicular_jac, const_jac: None },
    Kernel { name: "angle", n_res: 1, n_par: 8, degree: 2, n_const: 2, res: angle_res, jac: angle_jac, const_jac: None },
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
    Kernel { name: "angle_free", n_res: 1, n_par: 9, degree: 2, n_const: 2, res: angle_free_res, jac: angle_free_jac, const_jac: None },
    Kernel { name: "radius_free", n_res: 1, n_par: 2, degree: 1, n_const: 2, res: radius_free_res, jac: radius_free_jac, const_jac: None },
    Kernel { name: "parallel_distance_free", n_res: 1, n_par: 9, degree: 1, n_const: 2, res: parallel_distance_free_res, jac: parallel_distance_free_jac, const_jac: None },
    Kernel { name: "point_line_distance_free", n_res: 1, n_par: 7, degree: 1, n_const: 2, res: point_line_distance_free_res, jac: point_line_distance_free_jac, const_jac: None },
    Kernel { name: "annular_distance_free", n_res: 1, n_par: 3, degree: 1, n_const: 2, res: annular_distance_free_res, jac: annular_distance_free_jac, const_jac: None },
    Kernel { name: "horizontal_distance_free", n_res: 1, n_par: 5, degree: 1, n_const: 2, res: horizontal_distance_free_res, jac: horizontal_distance_free_jac, const_jac: None },
    Kernel { name: "vertical_distance_free", n_res: 1, n_par: 5, degree: 1, n_const: 2, res: vertical_distance_free_res, jac: vertical_distance_free_jac, const_jac: None },
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
