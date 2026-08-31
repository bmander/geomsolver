//! The 3D attitude of a `plane`, and the fold line two planes share.
//!
//! A multiview drawing is several 2D pictures of one object on one sheet, each on a stated
//! plane in space (Solvent §6.7).  Nothing three-dimensional is ever solved for: a plane's
//! attitude is a constant of the document, and what it buys is the *projector rule* of
//! descriptive geometry — two images of one point agree on their coordinate along the fold
//! line their planes share, and on nothing else.  That rule is `fold_line`, and `Project`'s
//! kernel is the one equation it comes to.
//!
//! An attitude is an orthonormal basis `(u, v)` of the plane with `n = u × v` toward the
//! viewer.  The page is the front view: `u = x`, `v = z`, so `n = −y` and the viewer stands
//! at −y looking in.  Every other plane is either *folded* from one — the draughtsman's
//! construction, which reaches any plane in two folds — or given outright.

/// An orthonormal basis of a plane in space; `u × v` points toward the viewer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Basis {
    pub u: [f64; 3],
    pub v: [f64; 3],
}

/// Normals closer to parallel than this share no fold line: a projection between the two
/// planes would say nothing.
pub const PARALLEL_TOL: f64 = 1e-9;

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn scaled(a: [f64; 3], k: f64) -> [f64; 3] {
    [a[0] * k, a[1] * k, a[2] * k]
}

/// A component as it will be stored and printed: the trigonometric dust below a double's
/// resolution of a unit vector is zero, and a negative zero is zero — a basis is document data,
/// and `cos 90°` written into a source file as `6.1e-17` is a number nobody said.
fn tidy(a: [f64; 3]) -> [f64; 3] {
    a.map(|x| if x.abs() < 1e-15 { 0.0 } else { x })
}

fn unit(a: [f64; 3]) -> Option<[f64; 3]> {
    let n = norm(a);
    (n > PARALLEL_TOL && n.is_finite()).then(|| tidy(scaled(a, 1.0 / n)))
}

impl Basis {
    /// The page itself: the front view, `u = x`, `v = z`, viewer at −y.
    pub fn page() -> Basis {
        Basis { u: [1.0, 0.0, 0.0], v: [0.0, 0.0, 1.0] }
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
        Basis { u: tidy(u), v: tidy(v) }
    }

    /// A basis given outright, orthonormalised: `u` is normalised and `v` is what is left of it
    /// after its component along `u` is removed.  `None` when either is zero or the two are
    /// parallel — no plane is spanned.
    pub fn explicit(u: [f64; 3], v: [f64; 3]) -> Option<Basis> {
        let u = unit(u)?;
        let along = dot(v, u);
        let v = unit([v[0] - along * u[0], v[1] - along * u[1], v[2] - along * u[2]])?;
        Some(Basis { u, v })
    }

    /// `u × v`: toward the viewer.
    pub fn normal(&self) -> [f64; 3] {
        cross(self.u, self.v)
    }
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
