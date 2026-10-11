//! **A region in general position** (§6.21, #183): a region solid whose terms do not share an axis
//! — so no meridian folds them (`solid::region`) — read point by point, each term the exact signed
//! distance from its surface turned to its side, the region their largest (an intersection).  Each
//! term is one-Lipschitz, so the field is, and a box's enclosure is the value at its centre widened
//! by its half-diagonal: conservative, never sampled.  The measured terms that read a body's faces
//! (`solid::measured`) are leaves of the same kind.
use super::{Error, I, V};
use crate::syntax::Cmp;
use std::f64::consts::FRAC_PI_2;
use std::sync::Arc;

type P3 = [f64; 3];

fn sub(a: P3, b: P3) -> P3 { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
fn dot(a: P3, b: P3) -> f64 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
fn norm(a: P3) -> f64 { dot(a, a).sqrt() }

/// What a term measures at a point: one-Lipschitz, or a cone's angle from its axis (read through
/// the cone's signed distance once its half is known).
#[derive(Clone, Debug)]
pub enum Measure {
    /// The distance from a point.
    Point(P3),
    /// The distance from the line through `a` along the unit `d`.
    Line { a: P3, d: P3 },
    /// The angle from the unit axis `d` seen from the apex `a`.
    Cone { a: P3, d: P3 },
    /// The ordinate along the unit `n` from `o`.
    Ordinate { o: P3, n: P3 },
    /// Anything else that is one-Lipschitz and signed: negative within (`Cmp::Le` 0 holds there).
    Signed(Signed),
}

/// A one-Lipschitz signed measure read by a closure (`solid::measured`).
#[derive(Clone)]
pub struct Signed(pub Arc<dyn Fn(P3) -> f64 + Send + Sync>);

impl std::fmt::Debug for Signed {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { f.write_str("Signed") }
}

/// One term: a measure held to its number (`lo`, and `hi` for an interval), a cone's grown by
/// `grow`.
#[derive(Clone, Debug)]
pub struct Term {
    pub measure: Measure,
    pub cmp: Cmp,
    pub lo: f64,
    pub hi: Option<f64>,
    pub grow: f64,
}

impl Term {
    /// The signed distance from the surface where the measure reads `v`, positive where it reads
    /// more (outside a ball, a cylinder, a cone; past a plane along its normal).
    fn from_surface(&self, p: P3, v: f64) -> f64 {
        match &self.measure {
            Measure::Point(c) => norm(sub(p, *c)) - v,
            Measure::Line { a, d } => {
                let q = sub(p, *a);
                let along = dot(q, *d);
                (dot(q, q) - along * along).max(0.0).sqrt() - v
            }
            Measure::Cone { a, d } => {
                // the cone's signed distance: the generator nearest, or the apex past a quarter
                let q = sub(p, *a);
                let rho = norm(q);
                if rho == 0.0 {
                    return 0.0;
                }
                let phi = (dot(q, *d) / rho).clamp(-1.0, 1.0).acos();
                rho * (phi - v).clamp(-FRAC_PI_2, FRAC_PI_2).sin()
            }
            Measure::Ordinate { o, n } => dot(sub(p, *o), *n) - v,
            Measure::Signed(f) => (f.0)(p) - v,
        }
    }
    /// The term at `p`: negative where it holds.
    pub fn value(&self, p: P3) -> f64 {
        match self.cmp {
            Cmp::Le => self.from_surface(p, self.lo) - self.grow,
            Cmp::Ge => self.grow - self.from_surface(p, self.lo),
            Cmp::In => {
                let hi = self.from_surface(p, self.hi.unwrap_or(self.lo));
                hi.max(-self.from_surface(p, self.lo))
            }
        }
    }
}

/// A region in general position: its terms, and a box holding its material where one is known.
#[derive(Clone, Debug)]
pub struct RegionField {
    /// Holds nowhere (#183): a set over faces that never meet.
    empty: bool,
    terms: Vec<Term>,
    support: Option<[[f64; 2]; 3]>,
    /// The size of what it reads: a rounding allowance for enclosures.
    size: f64,
}

impl RegionField {
    /// The region of `terms`, its support the box `support` (or none known).
    pub fn new(terms: Vec<Term>, support: Option<[[f64; 2]; 3]>) -> Self {
        let size = support.map_or(1.0, |b| b.iter().fold(1.0f64, |m, [a, z]| m.max(a.abs()).max(z.abs())));
        Self { empty: false, terms, support, size }
    }
    /// The region that holds nowhere.
    pub fn empty() -> Self {
        let never = Term { measure: Measure::Signed(Signed(Arc::new(|_| 1e30))), cmp: Cmp::Le, lo: 0.0,
                           hi: None, grow: 0.0 };
        Self { empty: true, terms: vec![never], support: None, size: 1.0 }
    }
    pub fn is_empty(&self) -> bool { self.empty }
    pub fn value(&self, p: P3) -> f64 { self.value_piece(p).0 }
    /// The value and the term that decides it.
    pub fn value_piece(&self, p: P3) -> (f64, usize) {
        let mut best = (f64::NEG_INFINITY, 0);
        for (k, t) in self.terms.iter().enumerate() {
            let v = t.value(p);
            if v > best.0 {
                best = (v, k);
            }
        }
        best
    }
    pub fn piece_count(&self) -> usize { self.terms.len() }
    pub fn carrier(&self, p: P3, piece: usize) -> Option<f64> { self.terms.get(piece).map(|t| t.value(p)) }
    pub fn support_bounds(&self) -> Result<Option<V>, Error> {
        let Some(b) = self.support else { return Ok(None) };
        Ok(Some([I::new(b[0][0], b[0][1])?, I::new(b[1][0], b[1][1])?, I::new(b[2][0], b[2][1])?]))
    }
    /// The field over a box: its value at the centre within the half-diagonal (one-Lipschitz),
    /// widened by a rounding allowance.
    pub fn bounds(&self, p: V) -> Result<I, Error> {
        let c = [p[0].mid(), p[1].mid(), p[2].mid()];
        let half = p.iter().map(|x| { let [a, b] = x.bounds(); 0.5 * (b - a) }).map(|h| h * h).sum::<f64>().sqrt();
        let v = self.value(c);
        let slack = 1e-12 * self.size;
        I::new(v - half - slack, v + half + slack)
    }
}

/// A box holding every point the planes' sides `n · x <= d` hold in common, where they close
/// one: the corners where three meet that all hold, if any three meet and every direction is
/// bounded (checked by the corners' box holding the region's every corner, which a bounded
/// polyhedron's corners do).
pub fn polytope_box(planes: &[(P3, f64)]) -> Option<[[f64; 2]; 3]> {
    let k = planes.len();
    if k < 4 {
        return None;
    }
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    let cross = |a: P3, b: P3| [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    for i in 0..k {
        for j in i + 1..k {
            for m in j + 1..k {
                let (a, b, c) = (planes[i], planes[j], planes[m]);
                let det = dot(a.0, cross(b.0, c.0));
                if det.abs() < 1e-12 {
                    continue;
                }
                // Cramer's rule by the triple products
                let x = [
                    cross(b.0, c.0), cross(c.0, a.0), cross(a.0, b.0),
                ];
                let corner: P3 = std::array::from_fn(|q| (a.1 * x[0][q] + b.1 * x[1][q] + c.1 * x[2][q]) / det);
                let tol = 1e-9 * (1.0 + norm(corner));
                if planes.iter().all(|(n, d)| dot(*n, corner) <= d + tol) {
                    for q in 0..3 {
                        lo[q] = lo[q].min(corner[q]);
                        hi[q] = hi[q].max(corner[q]);
                    }
                }
            }
        }
    }
    if !lo.iter().chain(&hi).all(|x| x.is_finite()) {
        return None;
    }
    // bounded only where no direction escapes: no ray `v` with every `n · v <= 0` — the cone of
    // such rays, if it is more than the origin, has an edge where two planes' bounding lines meet
    for i in 0..k {
        for j in i + 1..k {
            let line = cross(planes[i].0, planes[j].0);
            let l = norm(line);
            if l < 1e-12 {
                continue;
            }
            for s in [1.0, -1.0] {
                let v: P3 = std::array::from_fn(|q| s * line[q] / l);
                if planes.iter().all(|(n, _)| dot(*n, v) <= 1e-12) {
                    return None;
                }
            }
        }
    }
    Some(std::array::from_fn(|q| [lo[q], hi[q]]))
}
