//! **What a measure reads of a structure besides its whole** (#953): a measure of a body is one
//! of the ten moments of what it is made of — `∫ w`, `∫ w x` and `∫ w x xᵀ`, `w` one or a
//! density — over its solid elements (the body) or over facets of its boundary (a face), and what
//! a row compares is a number made of them: the whole (`What::Total`, a volume, a mass, an area),
//! a centroid's ordinate along an axis of the world less a point's (`What::Centroid`), or a
//! second moment about a line (`What::Second`).
//!
//! The moments are exact on a conforming mesh of curved elements where the integrand is a
//! polynomial its rule reads exactly: a quadratic tetrahedron's `det J` is of degree three, so
//! `∫ x det J` of degree five, the degree-five rule's; a facet of a plane face has `|x_u × x_v|` of
//! degree two, so `∫ x xᵀ dA` of degree six, under the facet rule here (degree eight).  So a
//! moment of the body, like its volume, is its boundary's: its nodes inside stand where they were
//! meshed (`Moved::boundary`'s rule), and a face's moments read only its own facets.

use super::solid::P;
use crate::kernels::{Dual, Num};

/// **Which number of the moments a row reads**: the whole; the centroid's ordinate `k` (0, 1 or 2,
/// the world's x, y or z) less a point's, `∫ w x_k / ∫ w − g_k`; or the second moment about a
/// line through `a` along `d`, `∫ w |(x − a) × d̂|²`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum What {
    #[default]
    Total,
    Centroid { k: usize },
    Second,
}

impl What {
    /// Its tag and its one number, as a row's constants say it.
    pub(super) fn code(self) -> [f64; 2] {
        match self {
            What::Total => [0.0, 0.0],
            What::Centroid { k } => [1.0, k as f64],
            What::Second => [2.0, 0.0],
        }
    }

    /// What `code` wrote.
    pub(super) fn read(tag: usize, k: usize) -> Option<What> {
        match tag {
            0 => Some(What::Total),
            1 if k < 3 => Some(What::Centroid { k }),
            2 => Some(What::Second),
            _ => None,
        }
    }
}

/// **What a measure reads**: over which facets of the boundary (none: the solid elements), and
/// which number of the moments.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Gauge {
    pub facets: Vec<[usize; 6]>,
    pub what: What,
}

impl Gauge {
    /// Whether it reads only the structure's whole over its elements, as a mass or a volume does.
    pub fn whole(&self) -> bool {
        self.facets.is_empty() && self.what == What::Total
    }
}

/// The ten moments' order: `∫ w`, `∫ w x`, `∫ w y`, `∫ w z`, then `∫ w x x`, `y y`, `z z`, `x y`,
/// `x z`, `y z`.
pub const MOMENTS: usize = 10;

/// The ten moments of `w` at a point `x`, `dv` its measure there.
fn at<T: Num>(x: P<T>, dv: T, out: &mut [T; MOMENTS]) {
    let f = [T::cst(1.0), x[0], x[1], x[2], x[0] * x[0], x[1] * x[1], x[2] * x[2], x[0] * x[1],
             x[0] * x[2], x[1] * x[2]];
    for (o, f) in out.iter_mut().zip(f) {
        *o = *o + f * dv;
    }
}

/// **A facet's ten moments** at its six nodes' places (`[a, b, c, ab, bc, ca]`, a quadratic
/// triangle), `∫ f |x_u × x_v|` by the collapsed Gauss rule of five points a side — exact to
/// degree eight in the facet's parameters, so a plane facet's second moments exactly.
pub(crate) fn facet<T: Num>(x: &[P<T>; 6]) -> [T; MOMENTS] {
    let mut out = [T::cst(0.0); MOMENTS];
    for &(s, t, w) in triangle_rule() {
        let n = super::plane::shape(s, t);
        let d = super::plane::shape_d(s, t);
        let mut p = [T::cst(0.0); 3];
        let mut ts = [[T::cst(0.0); 3]; 2];
        for k in 0..6 {
            for c in 0..3 {
                p[c] = p[c] + T::cst(n[k]) * x[k][c];
                for (r, tr) in ts.iter_mut().enumerate() {
                    tr[c] = tr[c] + T::cst(d[r][k]) * x[k][c];
                }
            }
        }
        let [a, b] = ts;
        let nda = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
        let da = (nda[0] * nda[0] + nda[1] * nda[1] + nda[2] * nda[2]).sqrt() * T::cst(w);
        at(p, da, &mut out);
    }
    out
}

/// **A quadratic tetrahedron's ten moments** at its ten nodes' places, by the degree-five rule:
/// exact for its volume and its first moments, and for its second within the rule's order.
pub(crate) fn tet<T: Num>(x: &[P<T>; 10]) -> [T; MOMENTS] {
    let mut out = [T::cst(0.0); MOMENTS];
    for (p, w, det) in super::solid::points_and_measure(x) {
        at(p, det * T::cst(w), &mut out);
    }
    out
}

/// The collapsed (Duffy) product of Gauss–Legendre's five points on `[0, 1]` a side over the
/// triangle `(0,0), (1,0), (0,1)`: `s = u`, `t = v (1 − u)`, weighed by `1 − u`; its weights sum
/// to the triangle's area, ½.
fn triangle_rule() -> &'static [(f64, f64, f64)] {
    static RULE: std::sync::OnceLock<Vec<(f64, f64, f64)>> = std::sync::OnceLock::new();
    RULE.get_or_init(|| {
        let gl = [(0.0, 0.568_888_888_888_888_9), (-0.538_469_310_105_683_1, 0.478_628_670_499_366_5),
                  (0.538_469_310_105_683_1, 0.478_628_670_499_366_5),
                  (-0.906_179_845_938_664, 0.236_926_885_056_189_1),
                  (0.906_179_845_938_664, 0.236_926_885_056_189_1)];
        let unit: Vec<(f64, f64)> = gl.iter().map(|&(x, w)| ((1.0 + x) / 2.0, w / 2.0)).collect();
        let mut out = Vec::with_capacity(25);
        for &(u, wu) in &unit {
            for &(v, wv) in &unit {
                out.push((u, v * (1.0 - u), wu * wv * (1.0 - u)));
            }
        }
        out
    })
}

/// **The number a row reads of the moments `m`**: the whole, a centroid's ordinate less the
/// point `target`'s, or the second moment about the line through `a` along `d` (`axis`), over a
/// number that may carry rates — in the moments (`Dual<10>`, the row's rate in its nodes) or in a
/// column the point or the line reads.  NaN where what it reads is missing or the whole is none.
pub(crate) fn compose<T: Num>(what: What, m: &[T; MOMENTS], target: Option<P<T>>,
                              axis: Option<(P<T>, P<T>)>) -> T {
    let nan = T::cst(f64::NAN);
    match what {
        What::Total => m[0],
        What::Centroid { k } => match target {
            Some(g) => m[1 + k] / m[0] - g[k],
            None => nan,
        },
        What::Second => {
            let Some((a, d)) = axis else { return nan };
            let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            let d = d.map(|c| c / len);
            // ∫ |x − a|² − ((x − a)·d)², by the moments: tr S − 2 a·F + |a|² A, less
            // dᵀ S d − 2 (a·d)(d·F) + (a·d)² A
            let (area, f) = (m[0], [m[1], m[2], m[3]]);
            let s = [[m[4], m[7], m[8]], [m[7], m[5], m[9]], [m[8], m[9], m[6]]];
            let dot = |p: P<T>, q: P<T>| p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
            let sd: P<T> = std::array::from_fn(|r| dot(s[r], d));
            let (ad, df) = (dot(a, d), dot(d, f));
            let whole = s[0][0] + s[1][1] + s[2][2] - T::cst(2.0) * dot(a, f) + dot(a, a) * area;
            let along = dot(d, sd) - T::cst(2.0) * ad * df + ad * ad * area;
            whole - along
        }
    }
}

/// The rate of `compose` in the moments at `m`: what a row's rate in its nodes is carried by.
pub(crate) fn compose_rate(what: What, m: &[f64; MOMENTS], target: Option<P<f64>>,
                           axis: Option<(P<f64>, P<f64>)>) -> [f64; MOMENTS] {
    let md: [Dual<MOMENTS>; MOMENTS] = std::array::from_fn(|k| Dual::var(m[k], k));
    let c = |p: P<f64>| p.map(Dual::<MOMENTS>::cst);
    compose(what, &md, target.map(c), axis.map(|(a, d)| (c(a), c(d)))).g
}

/// `compose`'s value alone, over numbers with no rate: what a claim reads of the moments.
pub(crate) fn compose_value(what: What, m: &[Dual<0>; MOMENTS], target: Option<P<Dual<0>>>,
                     axis: Option<(P<Dual<0>>, P<Dual<0>>)>) -> f64 {
    compose(what, m, target, axis).v
}
