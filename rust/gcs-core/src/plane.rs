//! The 3D attitude of a `plane`, and the fold line two planes share.
//!
//! A multiview drawing is several 2D pictures of one object on one sheet, each on a stated
//! plane in space (Solvent §6.7).  A stated plane's attitude is a constant of the document (a
//! solved one is a quaternion of unknowns, `Quat` below), and what it buys is the *projector
//! rule* of
//! descriptive geometry — two images of one point agree on their coordinate along the fold
//! line their planes share, and on nothing else.  That rule is `fold_line`, and `Project`'s
//! kernel is the one equation it comes to.
//!
//! An attitude is an orthonormal basis `(u, v)` of the plane with `n = u × v` toward the
//! viewer, and the point `o` in space its own origin stands at.  The page is the front view:
//! `u = x`, `v = z`, so `n = −y` and the viewer stands at −y looking in.  Every other plane is
//! either *folded* from one — the draughtsman's construction, which reaches any plane in two
//! folds — *offset* along one's normal, or given outright.
//!
//! **`o` is what a stack is written in.**  A drawing's views are images of one object and share
//! one origin, which is `o = 0` and every plane in every document written before solids; a
//! *part* standing a wall's thickness in front of another is a plane parallel to it with `o`
//! that far along the normal (Solvent §6.7, `offset:`).  The projector rule is unharmed, which
//! is why the offset may only be along the normal: `d` is perpendicular to both normals, so
//! `d·o = 0` and `fold_line` — the whole of what `Project` reads — cannot see it.

/// An orthonormal basis of a plane in space, and where its origin stands; `u × v` points toward
/// the viewer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Basis {
    pub u: [f64; 3],
    pub v: [f64; 3],
    /// Where this plane's own origin sits in space.  Zero for every view of one object — the
    /// shared origin `project` is written against — and non-zero for a plane a solid stands on.
    pub o: [f64; 3],
}

/// Normals closer to parallel than this share no fold line: a projection between the two
/// planes would say nothing.
pub const PARALLEL_TOL: f64 = 1e-9;

pub(crate) use crate::space::{cross, dot, norm};

pub(crate) fn scaled(a: [f64; 3], k: f64) -> [f64; 3] {
    [a[0] * k, a[1] * k, a[2] * k]
}

/// A component as it will be stored and printed: the trigonometric dust below a double's
/// resolution of a unit vector is zero, and a negative zero is zero — a basis is document data,
/// and `cos 90°` written into a source file as `6.1e-17` is a number nobody said.
fn tidy(a: [f64; 3]) -> [f64; 3] {
    a.map(|x| if x.abs() < 1e-15 { 0.0 } else { x })
}

pub(crate) fn unit(a: [f64; 3]) -> Option<[f64; 3]> {
    let n = norm(a);
    (n > PARALLEL_TOL && n.is_finite()).then(|| tidy(scaled(a, 1.0 / n)))
}

impl Basis {
    /// The page itself: the front view, `u = x`, `v = z`, viewer at −y.
    pub fn page() -> Basis {
        Basis { u: [1.0, 0.0, 0.0], v: [0.0, 0.0, 1.0], o: [0.0; 3] }
    }

    /// The plane folded from this one about the line at bearing `theta` (radians) in it — the
    /// auxiliary view a draughtsman takes off a fold line drawn at that angle.  The new plane is
    /// perpendicular to this one and contains the fold line, which is its `u`; its `v` points
    /// *away* from this plane's viewer, so distance from the fold line in the new view is depth
    /// behind this one — third-angle projection.  From the page, `fold(0)` is the top view
    /// (`u = x`, `v = y`, viewer at +z) and `fold(−90°)` the right view (`u = −z`, `v = y`,
    /// viewer at +x).
    pub fn fold(&self, theta: f64) -> Basis {
        let (s, c) = theta.sin_cos();
        let u = [
            c * self.u[0] + s * self.v[0],
            c * self.u[1] + s * self.v[1],
            c * self.u[2] + s * self.v[2],
        ];
        let v = scaled(self.normal(), -1.0);
        // a fold turns about a line *in* this plane, so the folded plane passes through the same
        // origin: a view folded from an offset one stands where that one does
        Basis { u: tidy(u), v: tidy(v), o: self.o }
    }

    /// A basis given outright, orthonormalised: `u` is normalised and `v` is what is left of it
    /// after its component along `u` is removed.  `None` when either is zero or the two are
    /// parallel — no plane is spanned.
    pub fn explicit(u: [f64; 3], v: [f64; 3]) -> Option<Basis> {
        let u = unit(u)?;
        let along = dot(v, u);
        let v = unit([v[0] - along * u[0], v[1] - along * u[1], v[2] - along * u[2]])?;
        Some(Basis { u, v, o: [0.0; 3] })
    }

    /// This plane moved `k` along its own normal — the same attitude, standing somewhere else.
    ///
    /// What `plane p(…, from: q, offset: k)` writes, and what a stack of parts is made of: the
    /// face of one part against the face of another is two parallel planes a thickness apart
    /// (Solvent §6.10).  Only along the normal, because an offset *in* the plane would move the
    /// origin `project` measures both images from and put a constant in its residual.
    pub fn offset(&self, k: f64) -> Basis {
        let n = self.normal();
        Basis { u: self.u, v: self.v, o: tidy([self.o[0] + k * n[0], self.o[1] + k * n[1], self.o[2] + k * n[2]]) }
    }

    /// How far this plane stands along its own normal from the shared origin.  Zero for a view;
    /// what `.offset` reads on a plane a solid stands on.
    pub fn along_normal(&self) -> f64 {
        dot(self.o, self.normal())
    }

    /// `u × v`: toward the viewer.
    pub fn normal(&self) -> [f64; 3] {
        cross(self.u, self.v)
    }

    /// Where a point drawn in this view sits **in space**: `a·u + b·v`, for the view
    /// coordinates `(a, b)` that `in_view` reads off the page.
    ///
    /// A view's origin is the image of one shared origin in space — the convention `project` is
    /// written against, and what lets its residual compare two views by one number — so for
    /// every view `o` is zero and this is `a·u + b·v`.  A plane offset along its normal carries
    /// `o`, and a point drawn on it stands that far out.
    pub fn lift(&self, a: f64, b: f64) -> [f64; 3] {
        [
            self.o[0] + a * self.u[0] + b * self.v[0],
            self.o[1] + a * self.u[1] + b * self.v[1],
            self.o[2] + a * self.u[2] + b * self.v[2],
        ]
    }

    /// The inverse of `lift` for a point *on* this plane: what the draughtsman would measure of
    /// `x` on this view.  A point off the plane is read by its shadow along the normal.
    pub fn view_coords(&self, x: [f64; 3]) -> (f64, f64) {
        let d = [x[0] - self.o[0], x[1] - self.o[1], x[2] - self.o[2]];
        (dot(d, self.u), dot(d, self.v))
    }
}

/// A point's coordinates **in the view it is drawn in**: `Rᵀ(c, s)(p − o)`, with
/// `Rᵀ(c, s)(x, y) = (c·x + s·y, −s·x + c·y)` — the page pose undone, leaving what the
/// draughtsman measured on that view.
///
/// The one place this is said.  `kernels::project_res` compares two of these along the fold
/// line and `overview` lifts one into space, and if the two ever disagreed about what a view
/// coordinate is, the drawing and the box would be pictures of different objects.
pub fn in_view(c: f64, s: f64, o: (f64, f64), p: (f64, f64)) -> (f64, f64) {
    let (x, y) = (p.0 - o.0, p.1 - o.1);
    (c * x + s * y, -s * x + c * y)
}

/// Where a point of a view lands **on the page**: the inverse of `in_view`, `o + R(c, s)(a, b)`
/// with `R(c, s)(x, y) = (c·x − s·y, s·x + c·y)`.
///
/// Written beside `in_view` and never anywhere else, for that function's own reason: a derived
/// view puts its edges back on the sheet through this, and if the two ever disagreed the drawing
/// and what is drawn over it would be pictures of different objects.
pub fn on_page(c: f64, s: f64, o: (f64, f64), p: (f64, f64)) -> (f64, f64) {
    (o.0 + c * p.0 - s * p.1, o.1 + s * p.0 + c * p.1)
}

/// The fold line two planes share, as a direction in each plane's own 2D coordinates:
/// `(d_A, d_B)`.  `None` when the planes are parallel and share none.
///
/// With `d = (n_A × n_B)/|n_A × n_B|` perpendicular to both normals, `d` lies in each plane, so
/// for any point `X` in space `d·X = (u_A·d)(u_A·X) + (v_A·d)(v_A·X)` — its coordinate along the
/// fold line, read off A's own 2D image of it — and likewise from B.  Two images of one point
/// therefore agree on `d_A · p_A = d_B · p_B` (each image measured from its plane's origin, the
/// images of one shared origin in space), and nothing else about them is shared: one equation,
/// which is the right count for four image numbers against three coordinates.
pub fn fold_line(a: &Basis, b: &Basis) -> Option<([f64; 2], [f64; 2])> {
    let d = unit(cross(a.normal(), b.normal()))?;
    Some(([dot(a.u, d), dot(a.v, d)], [dot(b.u, d), dot(b.v, d)]))
}

/// A quaternion `(w, x, y, z)`: the attitude of a **solved** view (`model::Att`).
///
/// Only the direction of one is read — every function here rotates by `q / |q|` — so a `q` a
/// solve has not yet brought back to the unit sphere still names a rotation, and the intrinsic
/// `quat_unit` row is what gauges its length.  The rotation carries the page's axes onto the
/// view's: `R(q)·e₁ = u`, `R(q)·e₂ = v`, `R(q)·e₃ = n`.
pub type Quat = [f64; 4];

/// `a ⊗ b`, the rotation `b` then `a`.
pub fn quat_mul(a: Quat, b: Quat) -> Quat {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// The homogeneous rotation matrix `M(q)` — `|q|²` times the rotation — row-major.  One
/// expression, read by `quat_matrix`, `quat_rotate` and `lift_q`, so the basis a view is read
/// with and the row a solve holds it by are the same arithmetic.
fn quat_m(q: Quat) -> [[f64; 3]; 3] {
    let [w, x, y, z] = q;
    [
        [w * w + x * x - y * y - z * z, 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), w * w - x * x + y * y - z * z, 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), w * w - x * x - y * y + z * z],
    ]
}

/// The rotation `q` names, `M(q) / |q|²`, row-major: its columns are `u`, `v` and `n`.  `None`
/// for a `q` of no length, which names none.
pub fn quat_matrix(q: Quat) -> Option<[[f64; 3]; 3]> {
    let n = q.iter().map(|x| x * x).sum::<f64>();
    (n > 0.0 && n.is_finite()).then(|| quat_m(q).map(|row| row.map(|x| x / n)))
}

/// `R(q)·a`.
pub fn quat_rotate(q: Quat, a: [f64; 3]) -> Option<[f64; 3]> {
    let r = quat_matrix(q)?;
    Some([0, 1, 2].map(|i| r[i][0] * a[0] + r[i][1] * a[1] + r[i][2] * a[2]))
}

/// The unit quaternion turning the page's axes onto `b`'s — `u`, `v`, `n` as the columns of the
/// rotation — with `w ≥ 0` (Shepperd's choice of the largest pivot, so no branch divides by a
/// small number).  `b` is taken to be orthonormal, which every stored basis is.
pub fn to_quat(b: &Basis) -> Quat {
    let n = b.normal();
    // r[i][j]: component i of column j
    let r = [[b.u[0], b.v[0], n[0]], [b.u[1], b.v[1], n[1]], [b.u[2], b.v[2], n[2]]];
    let t = r[0][0] + r[1][1] + r[2][2];
    let q = if t > 0.0 {
        let s = (t + 1.0).sqrt() * 2.0;
        [s / 4.0, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s]
    } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
        let s = (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt() * 2.0;
        [(r[2][1] - r[1][2]) / s, s / 4.0, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s]
    } else if r[1][1] > r[2][2] {
        let s = (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt() * 2.0;
        [(r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, s / 4.0, (r[1][2] + r[2][1]) / s]
    } else {
        let s = (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt() * 2.0;
        [(r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, s / 4.0]
    };
    let l = q.iter().map(|x| x * x).sum::<f64>().sqrt();
    let sign = if q[0] < 0.0 { -1.0 } else { 1.0 };
    q.map(|x| sign * x / l)
}

/// The basis `q` names, standing at `o`: `u = R(q)·e₁`, `v = R(q)·e₂`.  `None` for a `q` of
/// no length.
pub fn from_quat(q: Quat, o: [f64; 3]) -> Option<Basis> {
    let r = quat_matrix(q)?;
    Some(Basis { u: [r[0][0], r[1][0], r[2][0]], v: [r[0][1], r[1][1], r[2][1]], o })
}

/// **The lift of a solved view**: `L = R(q)·w` for `w = (a, b, d)` — a point's in-plane
/// coordinates and the view's offset along its normal — with the rotation `R` and `∂L/∂q`
/// (row-major, 3 × 4).  The one statement of it: the `lift` kernel reads its residual and its
/// columns here, and `Sketch::basis` reads the same `R`, so the view a solve holds and the view
/// a reader sees cannot come apart.  `R` is `M(q)/|q|²`, so `∂L/∂q` has no component along `q`
/// itself — the length of `q` is `quat_unit`'s business, not the lift's.  `None` for a `q` of
/// no length.
pub fn lift_q(q: Quat, w: [f64; 3]) -> Option<([f64; 3], [[f64; 3]; 3], [[f64; 4]; 3])> {
    let r = quat_matrix(q)?;
    let n = q.iter().map(|x| x * x).sum::<f64>();
    let mul =
        |m: [[f64; 3]; 3]| [0, 1, 2].map(|i| m[i][0] * w[0] + m[i][1] * w[1] + m[i][2] * w[2]);
    let l = mul(r);
    let [qw, qx, qy, qz] = q;
    // ∂M/∂q_k, each linear in q
    let dm = [
        [[qw, -qz, qy], [qz, qw, -qx], [-qy, qx, qw]],
        [[qx, qy, qz], [qy, -qx, -qw], [qz, qw, -qx]],
        [[-qy, qx, qw], [qx, qy, qz], [-qw, qz, -qy]],
        [[-qz, -qw, qx], [qw, -qz, qy], [qx, qy, qz]],
    ];
    let mut dq = [[0.0; 4]; 3];
    for (k, m) in dm.iter().enumerate() {
        let g = mul(m.map(|row| row.map(|x| 2.0 * x)));
        for i in 0..3 {
            dq[i][k] = (g[i] - 2.0 * q[k] * l[i]) / n;
        }
    }
    Some((l, r, dq))
}

/// The length of a plane glyph's tick, in screen pixels — a datum mark, sized like a callout's
/// arrowhead rather than like the drawing.
pub const TICK_PX: f64 = 8.0;

/// **The datum glyph a plane is drawn as**, in world coordinates: its chord, and a tick out of
/// the origin along the frame's own y-axis saying which side the view's second coordinate grows
/// to.  Two segments, as `[(from, to); 2]`.
///
/// Laid out here for the reason a dimension callout is laid out in `callout.rs`: it is geometry,
/// so the core says what the figure *is* and every front end only strokes what it is handed —
/// the SVG export and the canvas are then one picture of one drawing and not two.  Drawn twice
/// they had already come apart, at the tick's length.
///
/// `unit` is the world length of one screen pixel, so the tick is screen-constant the way a
/// callout's arrowhead is.  The **pick** is the chord alone (`model::point_to_drawn`), which is
/// not an oversight: a pick tolerance is a world length and has no `unit` to size a tick by, and
/// the chord is the extent a datum is taken hold of by.
pub fn glyph(sk: &crate::model::Sketch, i: usize, unit: f64) -> [((f64, f64), (f64, f64)); 2] {
    let f = &sk.planes[i].frame;
    let o = sk.point_xy(f.origin as usize);
    let t = sk.point_xy(f.toward as usize);
    let (c, s) = (sk.params[f.c as usize].value, sk.params[f.s as usize].value);
    let tick = (o.0 - s * TICK_PX * unit, o.1 + c * TICK_PX * unit);
    [(o, t), (o, tick)]
}
