//! **The plane element** (#510 S2): a quadratic triangle in plane stress or plane strain, the
//! interior's third element.
//!
//! Six nodes, numbered as CalculiX numbers a CPS6 — the corners counter-clockwise, then the
//! middles of the sides 0–1, 1–2 and 2–0 — each moving along x and y of the plane it is read in.
//! Isoparametric: the nodes' places are the element's columns, so a side whose middle node is off
//! the chord is curved, as a region's arc is.  Its stiffness is the Gram matrix of strain rows
//! like every other element's: at each of the three points of the degree-two rule, the three rows
//! `√(w·det J·t)·Cᵀ B` (`C` the Cholesky factor of the constitutive matrix `D`, `B` the strain of
//! the twelve moves), so `kₑ = Σ rᵣrᵣᵀ = t ∫ BᵀDB` — nine rows, the element's rank exactly (twelve
//! moves less the three rigid ones), so the rule leaves no spurious motion.  Its gradient is its
//! mutual energy's in `Dual<12>` over its twelve coordinates, as the beam's is over its run.
//!
//! A load laid on the plane (`Laid`) moves with it: a pressure on its sides, a force spread
//! evenly along sides or over an area, a body load — each its consistent load, its `∂f/∂x` in the
//! gradient.

use super::Failure;
use crate::kernels::{Dual, Num};

/// A quadratic triangle (#510 S2): its six nodes — corners counter-clockwise, then the middles of
/// the sides 0–1, 1–2, 2–0 — of modulus `e`, Poisson's ratio `nu` and density `rho`, `depth` its
/// thickness (plane stress) or its length along the prism (plane strain), read in plane strain
/// where `strain`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tri6 {
    pub n: [usize; 6],
    pub e: f64,
    pub nu: f64,
    pub rho: f64,
    pub depth: f64,
    pub strain: bool,
}

/// A node's place as the element reads it, over a number that may carry rates.
pub(crate) type P<T> = [T; 2];

/// The degree-two rule on the triangle `(0,0), (1,0), (0,1)`: its points as `(ξ, η)` and weights
/// summing to its area, ½.  Exact for a straight element's stiffness.
const RULE2: [(f64, f64, f64); 3] =
    [(1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0), (2.0 / 3.0, 1.0 / 6.0, 1.0 / 6.0),
     (1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0)];

/// Dunavant's degree-four rule of six points, for the consistent mass (`NᵀN`, of degree four).
pub(crate) fn rule4() -> [(f64, f64, f64); 6] {
    let (a, wa) = (0.445_948_490_915_965, 0.223_381_589_678_011 / 2.0);
    let (b, wb) = (0.091_576_213_509_771, 0.109_951_743_655_322 / 2.0);
    [(a, a, wa), (1.0 - 2.0 * a, a, wa), (a, 1.0 - 2.0 * a, wa),
     (b, b, wb), (1.0 - 2.0 * b, b, wb), (b, 1.0 - 2.0 * b, wb)]
}

/// Gauss–Legendre's three points on `[0, 1]` and their weights: a side's rule.
fn rule_side() -> [(f64, f64); 3] {
    let d = 0.6f64.sqrt() / 2.0;
    [(0.5 - d, 5.0 / 18.0), (0.5, 8.0 / 18.0), (0.5 + d, 5.0 / 18.0)]
}

/// Where each node stands in the element's own coordinates `(ξ, η)`.
const NODES: [(f64, f64); 6] = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (0.5, 0.0), (0.5, 0.5),
                                (0.0, 0.5)];

/// The six shape functions at `(ξ, η)`.
pub fn shape(xi: f64, eta: f64) -> [f64; 6] {
    let (l1, l2, l3) = (1.0 - xi - eta, xi, eta);
    [l1 * (2.0 * l1 - 1.0), l2 * (2.0 * l2 - 1.0), l3 * (2.0 * l3 - 1.0), 4.0 * l1 * l2,
     4.0 * l2 * l3, 4.0 * l3 * l1]
}

/// Their derivatives in `ξ` and in `η` at `(ξ, η)`.
pub(crate) fn shape_d(xi: f64, eta: f64) -> [[f64; 6]; 2] {
    let (l1, l2, l3) = (1.0 - xi - eta, xi, eta);
    [
        [1.0 - 4.0 * l1, 4.0 * l2 - 1.0, 0.0, 4.0 * (l1 - l2), 4.0 * l3, -4.0 * l3],
        [1.0 - 4.0 * l1, 0.0, 4.0 * l3 - 1.0, -4.0 * l2, 4.0 * l2, 4.0 * (l1 - l3)],
    ]
}

/// A side's three shape functions at `s` on `[0, 1]`: its first end, its middle, its last end.
pub fn side_shape(s: f64) -> [f64; 3] {
    [(1.0 - s) * (1.0 - 2.0 * s), 4.0 * s * (1.0 - s), s * (2.0 * s - 1.0)]
}

/// Their derivatives in `s`.
fn side_shape_d(s: f64) -> [f64; 3] {
    [4.0 * s - 3.0, 4.0 - 8.0 * s, 4.0 * s - 1.0]
}

/// The element's Jacobian at a point of it, `[[∂x/∂ξ, ∂y/∂ξ], [∂x/∂η, ∂y/∂η]]`, and its
/// determinant.
fn jacobian<T: Num>(x: &[P<T>; 6], d: &[[f64; 6]; 2]) -> ([[T; 2]; 2], T) {
    let mut j = [[T::cst(0.0); 2]; 2];
    for (r, row) in j.iter_mut().enumerate() {
        for (c, v) in row.iter_mut().enumerate() {
            for k in 0..6 {
                *v = *v + T::cst(d[r][k]) * x[k][c];
            }
        }
    }
    let det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
    (j, det)
}

/// The derivatives of the shape functions in x and y at a point, `d` theirs in `ξ` and `η`, by
/// the Jacobian `j` of determinant `det`.
fn in_plane<T: Num>(j: &[[T; 2]; 2], det: T, d: &[[f64; 6]; 2]) -> [[T; 6]; 2] {
    let mut out = [[T::cst(0.0); 6]; 2];
    for k in 0..6 {
        let (a, b) = (T::cst(d[0][k]), T::cst(d[1][k]));
        out[0][k] = (j[1][1] * a - j[0][1] * b) / det;
        out[1][k] = (j[0][0] * b - j[1][0] * a) / det;
    }
    out
}

/// The strains `(εx, εy, γxy)` of the twelve moves (each node's x, then its y) as three rows.
fn strain_rows<T: Num>(dn: &[[T; 6]; 2]) -> [[T; 12]; 3] {
    let mut b = [[T::cst(0.0); 12]; 3];
    for k in 0..6 {
        b[0][2 * k] = dn[0][k];
        b[1][2 * k + 1] = dn[1][k];
        b[2][2 * k] = dn[1][k];
        b[2][2 * k + 1] = dn[0][k];
    }
    b
}

impl Tri6 {
    /// **The constitutive matrix** as `(a, b, c)`, `D = [[a, b, 0], [b, a, 0], [0, 0, c]]`: in
    /// plane stress `E/(1 − ν²)·[1, ν, (1 − ν)/2]`, in plane strain `E/((1 + ν)(1 − 2ν))·[1 − ν,
    /// ν, (1 − 2ν)/2]`.
    pub fn constitutive(&self) -> [f64; 3] {
        let (e, nu) = (self.e, self.nu);
        if self.strain {
            let k = e / ((1.0 + nu) * (1.0 - 2.0 * nu));
            [k * (1.0 - nu), k * nu, k * (1.0 - 2.0 * nu) / 2.0]
        } else {
            let k = e / (1.0 - nu * nu);
            [k, k * nu, k * (1.0 - nu) / 2.0]
        }
    }

    /// A modulus, a ratio or a depth no element has: `E ≤ 0`, `ν` outside (−1, ½), no depth.
    fn check(&self) -> Result<(), Failure> {
        let ok = self.e > 0.0 && self.nu > -1.0 && self.nu < 0.5 && self.depth > 0.0;
        if ok { Ok(()) } else { Err(Failure::Unstiff) }
    }

    /// **Its nine strain rows** over its twelve moves at its nodes' places `x`: at each point of
    /// the degree-two rule, `√(w·det J·t)·Cᵀ B`, `C` the Cholesky factor of `D` — or why it is no
    /// element: turned over or flat somewhere (`Collapsed`), or of no stiffness (`Unstiff`).
    pub(crate) fn rows<T: Num>(&self, x: &[P<T>; 6]) -> Result<Vec<[T; 12]>, Failure> {
        self.check()?;
        let [a, b, c] = self.constitutive();
        let (c00, c01, c11, c22) = (a.sqrt(), b / a.sqrt(), (a - b * b / a).sqrt(), c.sqrt());
        let mut out = Vec::with_capacity(9);
        for &(xi, eta, w) in &RULE2 {
            let d = shape_d(xi, eta);
            let (j, det) = jacobian(x, &d);
            if !(det.val() > 0.0) {
                return Err(Failure::Collapsed);
            }
            let s = (T::cst(w * self.depth) * det).sqrt();
            let bm = strain_rows(&in_plane(&j, det, &d));
            let mut r = [[T::cst(0.0); 12]; 3];
            for k in 0..12 {
                r[0][k] = s * (T::cst(c00) * bm[0][k] + T::cst(c01) * bm[1][k]);
                r[1][k] = s * T::cst(c11) * bm[1][k];
                r[2][k] = s * T::cst(c22) * bm[2][k];
            }
            out.extend(r);
        }
        Ok(out)
    }

    /// Its stiffness, twelve by twelve over its moves: `Σ rrᵀ` of its rows.
    pub fn stiffness(&self, x: &[P<f64>; 6]) -> Result<[[f64; 12]; 12], Failure> {
        let rows = self.rows(&lift(x))?;
        let mut m = [[0.0; 12]; 12];
        for r in &rows {
            for a in 0..12 {
                for b in 0..12 {
                    m[a][b] += r[a].v * r[b].v;
                }
            }
        }
        Ok(m)
    }

    /// **Its consistent mass**, `ρ t ∫ NᵀN` by the degree-four rule, the same along x and y.
    pub fn mass(&self, x: &[P<f64>; 6]) -> Result<[[f64; 12]; 12], Failure> {
        let mut m = [[0.0; 12]; 12];
        for (xi, eta, w) in rule4() {
            let (_, det) = jacobian(&lift(x), &shape_d(xi, eta));
            if !(det.v > 0.0) {
                return Err(Failure::Collapsed);
            }
            let n = shape(xi, eta);
            let f = self.rho * self.depth * w * det.v;
            for a in 0..6 {
                for b in 0..6 {
                    m[2 * a][2 * b] += f * n[a] * n[b];
                    m[2 * a + 1][2 * b + 1] += f * n[a] * n[b];
                }
            }
        }
        Ok(m)
    }

    /// Its area, `∫ det J` by the degree-four rule (exact: a curved element's `det J` is of degree
    /// two).
    pub fn area(&self, x: &[P<f64>; 6]) -> f64 {
        rule4().iter().map(|&(xi, eta, w)| w * jacobian(&lift(x), &shape_d(xi, eta)).1.v).sum()
    }

    /// **The stresses at its nodes** under its moves `u` (each node's x, then its y): `(σx, σy,
    /// τxy)`, `D` times the strain read at each node — linear along a straight element, so exact
    /// there for the displacement it has.  `None` where it is turned over.
    pub fn stresses(&self, x: &[P<f64>; 6], u: &[f64; 12]) -> Option<[[f64; 3]; 6]> {
        let [a, b, c] = self.constitutive();
        let mut out = [[0.0; 3]; 6];
        for (k, &(xi, eta)) in NODES.iter().enumerate() {
            let d = shape_d(xi, eta);
            let (j, det) = jacobian(&lift(x), &d);
            if !(det.v > 0.0) {
                return None;
            }
            let bm = strain_rows(&in_plane(&j, det, &d));
            let e = bm.map(|r| (0..12).map(|i| r[i].v * u[i]).sum::<f64>());
            out[k] = [a * e[0] + b * e[1], b * e[0] + a * e[1], c * e[2]];
        }
        Some(out)
    }

    /// **The von Mises stress** of `(σx, σy, τxy)` as this element reads it: with `σz = 0` in
    /// plane stress, and `σz = ν(σx + σy)` in plane strain, which holds the prism's length.
    pub fn von_mises(&self, s: [f64; 3]) -> f64 {
        let z = if self.strain { self.nu * (s[0] + s[1]) } else { 0.0 };
        let (dx, dy, dz) = (s[0] - s[1], s[1] - z, z - s[0]);
        (0.5 * (dx * dx + dy * dy + dz * dz) + 3.0 * s[2] * s[2]).sqrt()
    }
}

/// Places as numbers with no rates, `D` coordinates each: the plane's and the solid's.
pub(crate) fn lift<const K: usize, const D: usize>(x: &[[f64; D]; K]) -> [[Dual<0>; D]; K] {
    x.map(|p| p.map(<Dual<0> as Num>::cst))
}

/* -- loads laid on the plane ------------------------------------------------------------------ */

/// **A load laid on the plane** (#510 S2), each its consistent load at the nodes, and each moving
/// with them: on sides, each a quadratic side `[first end, middle, last end]` of an element, run
/// with the material on its left (as an element's own sides run, counter-clockwise); over
/// elements, by index.
#[derive(Clone, Debug, PartialEq)]
pub enum Laid {
    /// A pressure `p` pushing into the material through sides of depth `depth`: `f = −p·t ∫ N n
    /// ds`, `n` the outward normal — linear in the places.
    Pressure { sides: Vec<[usize; 3]>, p: f64, depth: f64 },
    /// A force spread evenly along sides, `f = F ∫ N ds / Σ L`: the same force however long they
    /// grow.
    Along { sides: Vec<[usize; 3]>, force: [f64; 2] },
    /// A force spread evenly over elements, `f = F ∫ N dA / Σ A`.
    Over { elements: Vec<usize>, force: [f64; 2] },
    /// A body load, a force a volume (a density times an acceleration) over elements, `f = b·t
    /// ∫ N dA`, `t` each element's depth.
    Body { elements: Vec<usize>, force: [f64; 2] },
}

/// **What a load lays at each node of a side** at its places `x`, a number that may carry rates:
/// each of its three nodes' force (x, y), and its length — so `λᵀfₑ = Σ λ_k · f_k`, and an even
/// load's whole is `Σ fₑ / Σ length`.  The pressure's outward normal times `ds` is the tangent
/// turned a quarter clockwise, `(t_y, −t_x)`.
pub(crate) fn side_load<T: Num>(laid: &Laid, x: &[P<T>; 3]) -> ([[T; 2]; 3], T) {
    let zero = T::cst(0.0);
    let (mut f, mut length) = ([[zero; 2]; 3], zero);
    for (s, w) in rule_side() {
        let (n, d) = (side_shape(s), side_shape_d(s));
        let mut t = [zero; 2];
        for k in 0..3 {
            for c in 0..2 {
                t[c] = t[c] + T::cst(d[k]) * x[k][c];
            }
        }
        let ds = (t[0] * t[0] + t[1] * t[1]).sqrt() * T::cst(w);
        length = length + ds;
        let at = match laid {
            Laid::Pressure { p, depth, .. } => {
                let q = T::cst(-p * depth * w);
                [q * t[1], zero - q * t[0]]
            }
            Laid::Along { force, .. } => [ds * T::cst(force[0]), ds * T::cst(force[1])],
            _ => [zero; 2],
        };
        for k in 0..3 {
            for c in 0..2 {
                f[k][c] = f[k][c] + T::cst(n[k]) * at[c];
            }
        }
    }
    (f, length)
}

/// `side_load`'s over an element: each of its six nodes' force, and its area, the element's depth
/// `depth` read by a body load.
pub(crate) fn element_load<T: Num>(laid: &Laid, x: &[P<T>; 6], depth: f64) -> ([[T; 2]; 6], T) {
    let zero = T::cst(0.0);
    let (mut f, mut area) = ([[zero; 2]; 6], zero);
    let force = match laid {
        Laid::Over { force, .. } => *force,
        Laid::Body { force, .. } => force.map(|f| f * depth),
        _ => return (f, area),
    };
    for (xi, eta, w) in rule4() {
        let (_, det) = jacobian(x, &shape_d(xi, eta));
        let (n, da) = (shape(xi, eta), det * T::cst(w));
        area = area + da;
        for k in 0..6 {
            for c in 0..2 {
                f[k][c] = f[k][c] + da * T::cst(n[k] * force[c]);
            }
        }
    }
    (f, area)
}

impl Laid {
    /// Whether it is spread evenly, so its whole is divided by the measure it lies over.
    pub(crate) fn even(&self) -> bool {
        matches!(self, Laid::Along { .. } | Laid::Over { .. })
    }

    /// The sides it lies on, if any.
    pub(crate) fn sides(&self) -> &[[usize; 3]] {
        match self {
            Laid::Pressure { sides, .. } | Laid::Along { sides, .. } => sides,
            _ => &[],
        }
    }

    /// The elements it lies over, if any.
    pub(crate) fn elements(&self) -> &[usize] {
        match self {
            Laid::Over { elements, .. } | Laid::Body { elements, .. } => elements,
            _ => &[],
        }
    }
}

/// A node's place as a number carrying its rates among `N` columns, `at` its first.
pub(crate) fn var<const N: usize>(p: [f64; 2], at: usize) -> P<Dual<N>> {
    [Dual::var(p[0], at), Dual::var(p[1], at + 1)]
}
