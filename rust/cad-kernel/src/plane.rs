//! A plane in space, and the fold line two planes share.
//!
//! A plane is an orthonormal basis `(u, v)` with `n = u × v` toward the viewer, and the point
//! `o` in space its own origin stands at.  A point drawn in it at `(a, b)` stands at
//! `o + a·u + b·v`; nothing about a plane says where its picture goes on paper.  What a plane
//! buys a multiview drawing is the *projector rule* of descriptive geometry — two images of one
//! point agree on their coordinate along the fold line their planes share, and on nothing else.
//! That rule is `fold_line`.
//!
//! The front plane is `u = x`, `v = z`, so `n = −y` and the viewer stands at −y looking in.

#[allow(unused_imports)]
use crate::util::fmath::Det;
/// An orthonormal basis of a plane in space, and where its origin stands; `u × v` points toward
/// the viewer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Basis {
    pub u: [f64; 3],
    pub v: [f64; 3],
    /// Where this plane's own origin sits in space.
    pub o: [f64; 3],
}

/// Normals closer to parallel than this share no fold line: a projection between the two
/// planes would say nothing.
pub const PARALLEL_TOL: f64 = 1e-9;

pub use crate::util::space::{cross, dot, norm};

pub fn scaled(a: [f64; 3], k: f64) -> [f64; 3] {
    [a[0] * k, a[1] * k, a[2] * k]
}

/// A component as it will be stored and printed: the trigonometric dust below a double's
/// resolution of a unit vector is zero, and a negative zero is zero — `cos 90°` written into a
/// source file as `6.1e-17` is a number nobody said.
fn tidy(a: [f64; 3]) -> [f64; 3] {
    a.map(|x| if x.abs() < 1e-15 { 0.0 } else { x })
}

pub fn unit(a: [f64; 3]) -> Option<[f64; 3]> {
    let n = norm(a);
    (n > PARALLEL_TOL && n.is_finite()).then(|| tidy(scaled(a, 1.0 / n)))
}

impl Basis {
    /// The world's own axes: `u = x`, `v = y`, out along z, at the origin — what a frame with no
    /// axes of its own reads as.
    pub fn world() -> Basis {
        Basis { u: [1.0, 0.0, 0.0], v: [0.0, 1.0, 0.0], o: [0.0; 3] }
    }

    /// The front plane: `u = x`, `v = z`, viewer at −y — `std.front`, and where a 2D sketch's
    /// points are read in space.
    pub fn page() -> Basis {
        Basis { u: [1.0, 0.0, 0.0], v: [0.0, 0.0, 1.0], o: [0.0; 3] }
    }

    /// The plane two directions span, orthonormalised: `u` is normalised and `v` is what is left
    /// of it after its component along `u` is removed — so out of the plane is `u × v` and up is
    /// `out × u`.  `None` when either is zero or the two are parallel: no plane is spanned.
    pub fn explicit(u: [f64; 3], v: [f64; 3]) -> Option<Basis> {
        let u = unit(u)?;
        let along = dot(v, u);
        let v = unit([v[0] - along * u[0], v[1] - along * u[1], v[2] - along * u[2]])?;
        Some(Basis { u, v, o: [0.0; 3] })
    }

    /// How far this plane stands along its own normal from the origin.
    pub fn along_normal(&self) -> f64 {
        dot(self.o, self.normal())
    }

    /// **This plane as a cut stands it**: moved `by` along its own unit normal, and turned to face
    /// the other way where `flip` (`u` and `v` exchanged, so the normal is reversed and the plane
    /// is the same) — the other side taken away.
    pub fn moved_along(&self, by: f64, flip: bool) -> Basis {
        let n = unit(self.normal()).unwrap_or([0.0; 3]);
        let o = std::array::from_fn(|k| self.o[k] + by * n[k]);
        if flip { Basis { u: self.v, v: self.u, o } } else { Basis { o, ..*self } }
    }

    /// `u × v`: toward the viewer.
    pub fn normal(&self) -> [f64; 3] {
        cross(self.u, self.v)
    }

    /// Where a point drawn in this plane at `(a, b)` sits in space: `o + a·u + b·v`.
    pub fn lift(&self, a: f64, b: f64) -> [f64; 3] {
        [
            self.o[0] + a * self.u[0] + b * self.v[0],
            self.o[1] + a * self.u[1] + b * self.v[1],
            self.o[2] + a * self.u[2] + b * self.v[2],
        ]
    }

    /// The same plane in space, however it is turned within itself: parallel, and through the
    /// same points, `tol` a length.  What makes two views one place for a reader (`std.up` is
    /// `std.front` turned), and what W113 says of two planes a relation reads.
    pub fn coplanar(&self, other: &Basis, tol: f64) -> bool {
        let (n, m) = (self.normal(), other.normal());
        let (nn, nm) = (crate::util::space::norm(n), crate::util::space::norm(m));
        crate::util::space::norm(cross(n, m)) <= PARALLEL_TOL * nn * nm
            && dot(n, crate::util::space::sub(other.o, self.o)).abs() <= tol * nn
    }

    /// The inverse of `lift` for a point *on* this plane: what the draughtsman would measure of
    /// `x` in it.  A point off the plane is read by its shadow along the normal.
    pub fn view_coords(&self, x: [f64; 3]) -> (f64, f64) {
        let d = [x[0] - self.o[0], x[1] - self.o[1], x[2] - self.o[2]];
        (dot(d, self.u), dot(d, self.v))
    }
}

/// The fold line two planes share, as a direction in each plane's own 2D coordinates:
/// `(d_A, d_B)`.  `None` when the planes are parallel and share none.
///
/// With `d = (n_A × n_B)/|n_A × n_B|` perpendicular to both normals, `d` lies in each plane, so
/// for any point `X` in space `d·X = d·o_A + (u_A·d)(u_A·(X − o_A)) + (v_A·d)(v_A·(X − o_A))` —
/// its coordinate along the fold line, read off A's own 2D image of it — and likewise from B.
/// Two images of one point therefore agree on `d_A·p_A + d·o_A = d_B·p_B + d·o_B`, and nothing
/// else about them is shared: one equation, the right count for four image numbers against
/// three coordinates.
pub fn fold_line(a: &Basis, b: &Basis) -> Option<([f64; 2], [f64; 2])> {
    let d = unit(cross(a.normal(), b.normal()))?;
    Some(([dot(a.u, d), dot(a.v, d)], [dot(b.u, d), dot(b.v, d)]))
}

/// The constant `d·(o_A − o_B)` the projector rule between two planes carries: zero where their
/// origins are one point, which is every view of one object written from a shared origin.
pub fn fold_offset(a: &Basis, b: &Basis) -> f64 {
    match unit(cross(a.normal(), b.normal())) {
        Some(d) => dot(d, [a.o[0] - b.o[0], a.o[1] - b.o[1], a.o[2] - b.o[2]]),
        None => 0.0,
    }
}
