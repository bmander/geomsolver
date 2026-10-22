//! **The solid element** (#510 S3): a quadratic tetrahedron, the interior's fourth element.
//!
//! Ten nodes, numbered as CalculiX numbers a C3D10 and as the volume mesh numbers a TET10
//! (`delaunay::tetra::EDGES`) — the corners, then the middles of the edges 0–1, 1–2, 2–0, 0–3,
//! 1–3 and 2–3 — each moving along x, y and z.  Isoparametric: an edge whose middle node is off
//! its chord is curved, as a cylinder's bore is.  Its stiffness is the Gram matrix of strain rows
//! like every other element's: at each of the four points of the degree-two rule, the six rows
//! `√(w·det J)·Lᵀ B` (`L` the Cholesky factor of the isotropic constitutive matrix `D`, `B` the
//! strain of the thirty moves), so `kₑ = Σ rᵣrᵣᵀ = ∫ BᵀDB` — twenty-four rows, the element's rank
//! exactly (thirty moves less the six rigid ones), so the rule leaves no spurious motion.  Its
//! gradient is its mutual energy's in `Dual<30>` over its thirty coordinates, as the plane
//! element's is over its twelve.
//!
//! A load borne by the solid (`Borne`) moves with it: a pressure on facets of its boundary, a
//! force spread evenly over facets or through a volume, a body load — each its consistent load,
//! its `∂f/∂x` in the gradient.  Its geometric stiffness under a stress (`Tet10::geometric`) is
//! what a linear buckling reads.

use super::Failure;
use crate::kernels::{Dual, Num};

/// A quadratic tetrahedron (#510 S3): its ten nodes — the corners, then the middles of the edges
/// 0–1, 1–2, 2–0, 0–3, 1–3, 2–3 — of modulus `e`, Poisson's ratio `nu` and density `rho`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tet10 {
    pub n: [usize; 10],
    pub e: f64,
    pub nu: f64,
    pub rho: f64,
}

/// A node's place as the element reads it, over a number that may carry rates.
pub(crate) type P<T> = [T; 3];

/// The degree-two rule on the tetrahedron `(0,0,0), (1,0,0), (0,1,0), (0,0,1)`: its points as
/// `(ξ, η, ζ)` and weights summing to its volume, ⅙.  Exact for a straight element's stiffness,
/// and of four points, so twenty-four rows: the element's rank.
fn rule2() -> [([f64; 3], f64); 4] {
    let (a, b) = (0.138_196_601_125_010_5, 0.585_410_196_624_968_5);
    let w = 1.0 / 24.0;
    [([a, a, a], w), ([b, a, a], w), ([a, b, a], w), ([a, a, b], w)]
}

/// Walkington's degree-five rule of fourteen points, every weight positive: for the consistent
/// mass (`NᵀN`, of degree four), the volume (a curved element's `det J` is of degree three), the
/// loads through it and the geometric stiffness.
fn rule5() -> Vec<([f64; 3], f64)> {
    let mut out = Vec::with_capacity(14);
    for (a, w) in [(0.092_735_250_310_891_226, 0.012_248_840_519_393_658),
                   (0.310_885_919_263_300_61, 0.018_781_320_953_002_642)] {
        let d = 1.0 - 3.0 * a;
        out.extend([([a, a, a], w), ([d, a, a], w), ([a, d, a], w), ([a, a, d], w)]);
    }
    let (c, w) = (0.454_496_295_874_350_35, 0.007_091_003_462_846_911);
    let d = 0.5 - c;
    out.extend([([c, c, d], w), ([c, d, c], w), ([d, c, c], w), ([d, d, c], w), ([d, c, d], w),
                ([c, d, d], w)]);
    out
}

/// The edges a middle node stands on, in its order: `delaunay::tetra::EDGES`.
const EDGES: [(usize, usize); 6] = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)];

/// Where each node stands in the element's own coordinates `(ξ, η, ζ)`.
fn nodes() -> [[f64; 3]; 10] {
    let corners = [[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    std::array::from_fn(|k| match k {
        0..4 => corners[k],
        _ => {
            let (i, j) = EDGES[k - 4];
            std::array::from_fn(|c| 0.5 * (corners[i][c] + corners[j][c]))
        }
    })
}

/// The barycentric coordinates of `(ξ, η, ζ)`.
fn bary(p: [f64; 3]) -> [f64; 4] {
    [1.0 - p[0] - p[1] - p[2], p[0], p[1], p[2]]
}

/// The ten shape functions at `(ξ, η, ζ)`.
pub fn shape(p: [f64; 3]) -> [f64; 10] {
    let l = bary(p);
    std::array::from_fn(|k| match k {
        0..4 => l[k] * (2.0 * l[k] - 1.0),
        _ => {
            let (i, j) = EDGES[k - 4];
            4.0 * l[i] * l[j]
        }
    })
}

/// Their derivatives in `ξ`, `η` and `ζ` at `(ξ, η, ζ)`.
fn shape_d(p: [f64; 3]) -> [[f64; 10]; 3] {
    let l = bary(p);
    // each barycentric coordinate's derivative in ξ, η, ζ
    let dl = |i: usize, r: usize| match i {
        0 => -1.0,
        _ => (i - 1 == r) as u8 as f64,
    };
    std::array::from_fn(|r| std::array::from_fn(|k| match k {
        0..4 => (4.0 * l[k] - 1.0) * dl(k, r),
        _ => {
            let (i, j) = EDGES[k - 4];
            4.0 * (l[j] * dl(i, r) + l[i] * dl(j, r))
        }
    }))
}

/// The element's Jacobian at a point of it, `J[r][c] = ∂x_c/∂ξ_r`, and its determinant.
fn jacobian<T: Num>(x: &[P<T>; 10], d: &[[f64; 10]; 3]) -> ([[T; 3]; 3], T) {
    let mut j = [[T::cst(0.0); 3]; 3];
    for (r, row) in j.iter_mut().enumerate() {
        for (c, v) in row.iter_mut().enumerate() {
            for k in 0..10 {
                if d[r][k] != 0.0 {
                    *v = *v + T::cst(d[r][k]) * x[k][c];
                }
            }
        }
    }
    let det = j[0][0] * (j[1][1] * j[2][2] - j[1][2] * j[2][1])
        - j[0][1] * (j[1][0] * j[2][2] - j[1][2] * j[2][0])
        + j[0][2] * (j[1][0] * j[2][1] - j[1][1] * j[2][0]);
    (j, det)
}

/// The derivatives of the shape functions in x, y and z at a point, `d` theirs in `ξ`, `η`, `ζ`:
/// `J⁻¹ d`, by the Jacobian's cofactors over its determinant.
fn in_space<T: Num>(j: &[[T; 3]; 3], det: T, d: &[[f64; 10]; 3]) -> [[T; 10]; 3] {
    let m = |r: usize, c: usize| j[r][c];
    // the inverse's entries: inv[c][r] = cofactor(r, c) / det
    let cof = |r: usize, c: usize| {
        let (r1, r2) = ((r + 1) % 3, (r + 2) % 3);
        let (c1, c2) = ((c + 1) % 3, (c + 2) % 3);
        m(r1, c1) * m(r2, c2) - m(r1, c2) * m(r2, c1)
    };
    let inv: [[T; 3]; 3] = std::array::from_fn(|c| std::array::from_fn(|r| cof(r, c) / det));
    let mut out = [[T::cst(0.0); 10]; 3];
    for k in 0..10 {
        for (c, o) in out.iter_mut().enumerate() {
            let mut v = T::cst(0.0);
            for r in 0..3 {
                if d[r][k] != 0.0 {
                    v = v + inv[c][r] * T::cst(d[r][k]);
                }
            }
            o[k] = v;
        }
    }
    out
}

/// The strains `(εx, εy, εz, γyz, γxz, γxy)` of the thirty moves (each node's x, y, z) as six
/// rows.
fn strain_rows<T: Num>(dn: &[[T; 10]; 3]) -> [[T; 30]; 6] {
    let mut b = [[T::cst(0.0); 30]; 6];
    for k in 0..10 {
        for c in 0..3 {
            b[c][3 * k + c] = dn[c][k];
        }
        b[3][3 * k + 1] = dn[2][k];
        b[3][3 * k + 2] = dn[1][k];
        b[4][3 * k] = dn[2][k];
        b[4][3 * k + 2] = dn[0][k];
        b[5][3 * k] = dn[1][k];
        b[5][3 * k + 1] = dn[0][k];
    }
    b
}

impl Tet10 {
    /// **The constitutive matrix** as Lamé's `(λ, μ)`: `D` is `λ + 2μ` on the stretches'
    /// diagonal, `λ` off it, `μ` on the shears'.
    pub fn lame(&self) -> (f64, f64) {
        let (e, nu) = (self.e, self.nu);
        (e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu)), e / (2.0 * (1.0 + nu)))
    }

    /// The stress `(σx, σy, σz, τyz, τxz, τxy)` of a strain.
    pub fn stress_of(&self, e: [f64; 6]) -> [f64; 6] {
        let (l, m) = self.lame();
        let tr = e[0] + e[1] + e[2];
        [l * tr + 2.0 * m * e[0], l * tr + 2.0 * m * e[1], l * tr + 2.0 * m * e[2], m * e[3],
         m * e[4], m * e[5]]
    }

    /// A modulus or a ratio no element has: `E ≤ 0`, `ν` outside (−1, ½).
    fn check(&self) -> Result<(), Failure> {
        let ok = self.e > 0.0 && self.nu > -1.0 && self.nu < 0.5;
        if ok { Ok(()) } else { Err(Failure::Unstiff) }
    }

    /// **Its twenty-four strain rows** over its thirty moves at its nodes' places `x`: at each
    /// point of the degree-two rule, `√(w·det J)·Lᵀ B`, `L` the Cholesky factor of `D` — or why it
    /// is no element: turned over or flat somewhere (`Collapsed`), or of no stiffness
    /// (`Unstiff`).
    pub(crate) fn rows<T: Num>(&self, x: &[P<T>; 10]) -> Result<Vec<[T; 30]>, Failure> {
        self.check()?;
        let (l, m) = self.lame();
        // the stretches' block [[a, b, b], [b, a, b], [b, b, a]], a = λ + 2μ and b = λ, factored
        let (a, b) = (l + 2.0 * m, l);
        let l00 = a.sqrt();
        let l10 = b / l00;
        let l11 = (a - l10 * l10).sqrt();
        let l21 = (b - l10 * l10) / l11;
        let l22 = (a - l10 * l10 - l21 * l21).sqrt();
        let shear = m.sqrt();
        let mut out = Vec::with_capacity(24);
        for (p, w) in rule2() {
            let d = shape_d(p);
            let (j, det) = jacobian(x, &d);
            if !(det.val() > 0.0) {
                return Err(Failure::Collapsed);
            }
            let s = (T::cst(w) * det).sqrt();
            let bm = strain_rows(&in_space(&j, det, &d));
            let mut r = [[T::cst(0.0); 30]; 6];
            let (c00, c10, c11, c21, c22) = (T::cst(l00), T::cst(l10), T::cst(l11), T::cst(l21),
                                             T::cst(l22));
            let sh = T::cst(shear);
            for k in 0..30 {
                r[0][k] = s * (c00 * bm[0][k] + c10 * (bm[1][k] + bm[2][k]));
                r[1][k] = s * (c11 * bm[1][k] + c21 * bm[2][k]);
                r[2][k] = s * c22 * bm[2][k];
                for c in 3..6 {
                    r[c][k] = s * sh * bm[c][k];
                }
            }
            out.extend(r);
        }
        Ok(out)
    }

    /// Its stiffness, thirty by thirty over its moves: `Σ rrᵀ` of its rows.
    pub fn stiffness(&self, x: &[P<f64>; 10]) -> Result<[[f64; 30]; 30], Failure> {
        let rows = self.rows(&lift(x))?;
        let mut m = [[0.0; 30]; 30];
        for r in &rows {
            for a in 0..30 {
                if r[a].v == 0.0 {
                    continue;
                }
                for b in 0..30 {
                    m[a][b] += r[a].v * r[b].v;
                }
            }
        }
        Ok(m)
    }

    /// **Its consistent mass**, `ρ ∫ NᵀN` by the degree-five rule, the same along x, y and z.
    pub fn mass(&self, x: &[P<f64>; 10]) -> Result<[[f64; 30]; 30], Failure> {
        let mut m = [[0.0; 30]; 30];
        for (p, w) in rule5() {
            let (_, det) = jacobian(&lift(x), &shape_d(p));
            if !(det.v > 0.0) {
                return Err(Failure::Collapsed);
            }
            let n = shape(p);
            let f = self.rho * w * det.v;
            for a in 0..10 {
                for b in 0..10 {
                    for c in 0..3 {
                        m[3 * a + c][3 * b + c] += f * n[a] * n[b];
                    }
                }
            }
        }
        Ok(m)
    }

    /// Its volume, `∫ det J` by the degree-five rule (exact: a curved element's `det J` is of
    /// degree three).
    pub fn volume(&self, x: &[P<f64>; 10]) -> f64 {
        rule5().iter().map(|&(p, w)| w * jacobian(&lift(x), &shape_d(p)).1.v).sum()
    }

    /// The strain at `(ξ, η, ζ)` under its moves `u`, or `None` where it is turned over there.
    fn strain_at(&self, x: &[P<f64>; 10], u: &[f64; 30], p: [f64; 3]) -> Option<[f64; 6]> {
        let d = shape_d(p);
        let (j, det) = jacobian(&lift(x), &d);
        if !(det.v > 0.0) {
            return None;
        }
        let bm = strain_rows(&in_space(&j, det, &d));
        Some(bm.map(|r| (0..30).map(|i| r[i].v * u[i]).sum::<f64>()))
    }

    /// **The stresses at its nodes** under its moves `u` (each node's x, y, z): `(σx, σy, σz,
    /// τyz, τxz, τxy)`, `D` times the strain read at each node — linear along a straight element,
    /// so exact there for the displacement it has.  `None` where it is turned over.
    pub fn stresses(&self, x: &[P<f64>; 10], u: &[f64; 30]) -> Option<[[f64; 6]; 10]> {
        let at = nodes();
        let mut out = [[0.0; 6]; 10];
        for k in 0..10 {
            out[k] = self.stress_of(self.strain_at(x, u, at[k])?);
        }
        Some(out)
    }

    /// **The von Mises stress** of `(σx, σy, σz, τyz, τxz, τxy)`.
    pub fn von_mises(s: [f64; 6]) -> f64 {
        let (dx, dy, dz) = (s[0] - s[1], s[1] - s[2], s[2] - s[0]);
        (0.5 * (dx * dx + dy * dy + dz * dz) + 3.0 * (s[3] * s[3] + s[4] * s[4] + s[5] * s[5]))
            .sqrt()
    }

    /// **Its geometric stiffness** under the stress its moves `u` make: `∫ ∇Nₐᵀ σ ∇N_b dV` along
    /// each axis alike, by the degree-five rule — what `eigen::buckling` reads beside the
    /// stiffness, a pull stiffening and a press softening.  `None` where it is turned over.
    pub fn geometric(&self, x: &[P<f64>; 10], u: &[f64; 30]) -> Option<[[f64; 30]; 30]> {
        let mut m = [[0.0; 30]; 30];
        for (p, w) in rule5() {
            let d = shape_d(p);
            let (j, det) = jacobian(&lift(x), &d);
            if !(det.v > 0.0) {
                return None;
            }
            let g = in_space(&j, det, &d).map(|r| r.map(|v| v.v));
            let s = self.stress_of(self.strain_at(x, u, p)?);
            let t = [[s[0], s[5], s[4]], [s[5], s[1], s[3]], [s[4], s[3], s[2]]];
            for a in 0..10 {
                let ta: [f64; 3] =
                    std::array::from_fn(|r| (0..3).map(|c| t[r][c] * g[c][a]).sum());
                for b in 0..10 {
                    let v = w * det.v * (0..3).map(|r| g[r][b] * ta[r]).sum::<f64>();
                    for c in 0..3 {
                        m[3 * a + c][3 * b + c] += v;
                    }
                }
            }
        }
        Some(m)
    }
}

/// Places as numbers with no rates.
fn lift<const K: usize>(x: &[P<f64>; K]) -> [P<Dual<0>>; K] {
    x.map(|p| p.map(<Dual<0> as Num>::cst))
}

/* -- loads borne by a solid ------------------------------------------------------------------- */

/// **A load borne by a solid** (#510 S3), each its consistent load at the nodes, and each moving
/// with them: on facets of its boundary, each a quadratic triangle `[a, b, c, ab, bc, ca]` turned
/// so that `(b − a) × (c − a)` points out of the material; through elements, by index.
#[derive(Clone, Debug, PartialEq)]
pub enum Borne {
    /// A pressure `p` pushing into the material through facets: `f = −p ∫ N n dA`, `n` the
    /// outward normal — of degree four in the facet's own coordinates, so exact.
    Pressure { facets: Vec<[usize; 6]>, p: f64 },
    /// A force spread evenly over facets, `f = F ∫ N dA / Σ A`: the same force however they grow.
    Across { facets: Vec<[usize; 6]>, force: [f64; 3] },
    /// A force spread evenly through elements, `f = F ∫ N dV / Σ V`.
    Through { elements: Vec<usize>, force: [f64; 3] },
    /// A body load, a force a volume (a density times an acceleration) through elements, `f = b
    /// ∫ N dV`.
    Body { elements: Vec<usize>, force: [f64; 3] },
}

/// The plane's degree-four rule on a facet `(0,0), (1,0), (0,1)`: its points and weights.
fn facet_rule() -> [(f64, f64, f64); 6] {
    let (a, wa) = (0.445_948_490_915_965, 0.223_381_589_678_011 / 2.0);
    let (b, wb) = (0.091_576_213_509_771, 0.109_951_743_655_322 / 2.0);
    [(a, a, wa), (1.0 - 2.0 * a, a, wa), (a, 1.0 - 2.0 * a, wa),
     (b, b, wb), (1.0 - 2.0 * b, b, wb), (b, 1.0 - 2.0 * b, wb)]
}

/// **What a load lays at each node of a facet** at its places `x`, a number that may carry
/// rates: each of its six nodes' force, and its area — so `λᵀfₑ = Σ λ_k · f_k`, and an even
/// load's whole is `Σ fₑ / Σ area`.
pub(crate) fn facet_load<T: Num>(borne: &Borne, x: &[P<T>; 6]) -> ([[T; 3]; 6], T) {
    let zero = T::cst(0.0);
    let (mut f, mut area) = ([[zero; 3]; 6], zero);
    for (s, t, w) in facet_rule() {
        let n = super::plane::shape(s, t);
        let d = super::plane::shape_d(s, t);
        let mut ts = [[zero; 3]; 2];
        for (r, tr) in ts.iter_mut().enumerate() {
            for k in 0..6 {
                for c in 0..3 {
                    tr[c] = tr[c] + T::cst(d[r][k]) * x[k][c];
                }
            }
        }
        let [a, b] = ts;
        // the outward normal times dA
        let nda = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
        let da = (nda[0] * nda[0] + nda[1] * nda[1] + nda[2] * nda[2]).sqrt() * T::cst(w);
        area = area + da;
        let at = match borne {
            Borne::Pressure { p, .. } => nda.map(|v| T::cst(-p * w) * v),
            Borne::Across { force, .. } => force.map(|c| da * T::cst(c)),
            _ => [zero; 3],
        };
        for k in 0..6 {
            for c in 0..3 {
                f[k][c] = f[k][c] + T::cst(n[k]) * at[c];
            }
        }
    }
    (f, area)
}

/// `facet_load`'s through an element: each of its ten nodes' force, and its volume.
pub(crate) fn element_load<T: Num>(borne: &Borne, x: &[P<T>; 10]) -> ([[T; 3]; 10], T) {
    let zero = T::cst(0.0);
    let (mut f, mut volume) = ([[zero; 3]; 10], zero);
    let force = match borne {
        Borne::Through { force, .. } | Borne::Body { force, .. } => *force,
        _ => return (f, volume),
    };
    for (p, w) in rule5() {
        let (_, det) = jacobian(x, &shape_d(p));
        let (n, dv) = (shape(p), det * T::cst(w));
        volume = volume + dv;
        for k in 0..10 {
            for c in 0..3 {
                f[k][c] = f[k][c] + dv * T::cst(n[k] * force[c]);
            }
        }
    }
    (f, volume)
}

impl Borne {
    /// Whether it is spread evenly, so its whole is divided by the measure it lies over.
    pub(crate) fn even(&self) -> bool {
        matches!(self, Borne::Across { .. } | Borne::Through { .. })
    }

    /// The facets it lies on, if any.
    pub fn facets(&self) -> &[[usize; 6]] {
        match self {
            Borne::Pressure { facets, .. } | Borne::Across { facets, .. } => facets,
            _ => &[],
        }
    }

    /// The elements it lies through, if any.
    pub fn elements(&self) -> &[usize] {
        match self {
            Borne::Through { elements, .. } | Borne::Body { elements, .. } => elements,
            _ => &[],
        }
    }
}

/// A node's place as a number carrying its rates among `N` columns, `at` its first.
pub(crate) fn var<const N: usize>(p: [f64; 3], at: usize) -> P<Dual<N>> {
    [Dual::var(p[0], at), Dual::var(p[1], at + 1), Dual::var(p[2], at + 2)]
}
