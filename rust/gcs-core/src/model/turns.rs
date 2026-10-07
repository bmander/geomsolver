//! Turned copies: what a `ring` is solved as (Solvent §12.3–12.4, issue #96).
//!
//! A ring's copies are entities like any other — drawn, picked, named `tip[2]`, read by a face
//! — but only the representative's (copy 0's) numbers are unknowns.  Copy `k`'s are the
//! representative's turned `k` steps of `τ/n` about the ring's centre: a **derived** parameter,
//! a fixed linear combination of parameters that are not (the representative's and the
//! centre's), so a free centre is no harder than a held one.  `System` solves the free columns
//! and folds a derived parameter's Jacobian into its bases by the chain rule (`Sketch::derived`);
//! nothing else in the sketch owns a column for it, and `settle_turns` writes its value wherever
//! a parameter vector is written.
//!
//! A turn about a **point** is in that point's view (or on the page), counter-clockwise in the
//! view's own coordinates; one about an **axis** is in space, right-handed about its direction,
//! which must be held — a free direction makes the turn nonlinear in it.  A curve owns no
//! parameter: a turned curve is evaluated as its representative and turned (`curve_point`).

use crate::fmath::Det;
use super::*;

/// Copy `copy` is the representative `rep` turned `k` steps of `τ/n` about `about`, a point or
/// an axis.  Of the same kind; points, circles, arcs and curves are recorded (the others own no
/// number a turn moves).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Turn {
    pub copy: EntRef,
    pub rep: EntRef,
    pub about: EntRef,
    pub k: u32,
    pub n: u32,
}

/// A derived parameter: its value is `Σ w·x[base]`, over parameters that are not derived.
#[derive(Clone, Debug, PartialEq)]
pub struct Derivation {
    pub param: u32,
    pub terms: Vec<(u32, f64)>,
}

impl Derivation {
    /// Its value over a parameter vector — the one sum `System` and `settle_turns` both take, so
    /// the two agree to the bit.
    pub fn value(&self, x: &[f64]) -> f64 {
        let mut v = 0.0;
        for &(b, w) in &self.terms {
            v += w * x[b as usize];
        }
        v
    }
}

impl Turn {
    /// (cos, sin) of the turn — the core's own trigonometry, so every platform's bits agree.
    pub fn cos_sin(&self) -> (f64, f64) {
        let th = std::f64::consts::TAU * self.k as f64 / self.n as f64;
        let (s, c) = th.dsin_cos();
        (c, s)
    }
}

impl Sketch {
    /// Record a turn.  A turned arc's intrinsic rows — its ends on its circle — are the images of
    /// the representative's, so they go: every reader of the constraints, the solve's and the
    /// plan's alike, sees the representative's alone.
    pub fn turn(&mut self, t: Turn) {
        if t.copy.kind == EntKind::Arc {
            self.constraints.retain(|c| !(c.intrinsic && c.entities().contains(&t.copy)));
        }
        self.turns.push(t);
    }

    /// The turn copy `e` is, if it is one.
    pub fn turn_of(&self, e: EntRef) -> Option<&Turn> {
        self.turns.iter().find(|t| t.copy == e)
    }

    /// The rotation of a turn about an axis, from the axis's held direction: Rodrigues' formula,
    /// row-major.
    fn axis_rotation(&self, t: &Turn) -> [[f64; 3]; 3] {
        let d = self.axes[t.about.i()].d.map(|p| self.params[p as usize].value);
        let d = crate::space::normalised(d).unwrap_or([0.0, 0.0, 1.0]);
        let (c, s) = t.cos_sin();
        let k = 1.0 - c;
        [
            [c + d[0] * d[0] * k, d[0] * d[1] * k - d[2] * s, d[0] * d[2] * k + d[1] * s],
            [d[1] * d[0] * k + d[2] * s, c + d[1] * d[1] * k, d[1] * d[2] * k - d[0] * s],
            [d[2] * d[0] * k - d[1] * s, d[2] * d[1] * k + d[0] * s, c + d[2] * d[2] * k],
        ]
    }

    /// Every derived parameter, in turn order: a turned point's coordinates over the
    /// representative's and the centre's, a turned circle's or arc's radius over the
    /// representative's.  A turn whose entities do not fit it (a point in space about a point,
    /// a planar point about an axis) derives nothing — elaboration refuses those (E023).
    pub fn derived(&self) -> Vec<Derivation> {
        let mut out = Vec::new();
        for t in &self.turns {
            match t.copy.kind {
                EntKind::Point => {
                    let (p, q) = (&self.points[t.copy.i()], &self.points[t.rep.i()]);
                    match t.about.kind {
                        EntKind::Point if p.z.is_none() && q.z.is_none() => {
                            let o = &self.points[t.about.i()];
                            let (c, s) = t.cos_sin();
                            out.push(Derivation {
                                param: p.x,
                                terms: vec![(q.x, c), (q.y, -s), (o.x, 1.0 - c), (o.y, s)],
                            });
                            out.push(Derivation {
                                param: p.y,
                                terms: vec![(q.x, s), (q.y, c), (o.x, -s), (o.y, 1.0 - c)],
                            });
                        }
                        EntKind::Axis => {
                            let (Some(pz), Some(qz)) = (p.z, q.z) else { continue };
                            let r = self.axis_rotation(t);
                            let a = self.axes[t.about.i()].a;
                            let (copy, rep) = ([p.x, p.y, pz], [q.x, q.y, qz]);
                            for i in 0..3 {
                                let mut terms = Vec::with_capacity(6);
                                for j in 0..3 {
                                    terms.push((rep[j], r[i][j]));
                                }
                                for j in 0..3 {
                                    let w = if i == j { 1.0 - r[i][j] } else { -r[i][j] };
                                    terms.push((a[j], w));
                                }
                                out.push(Derivation { param: copy[i], terms });
                            }
                        }
                        _ => {}
                    }
                }
                EntKind::Circle | EntKind::Arc => out.push(Derivation {
                    param: self.round_radius(t.copy) as u32,
                    terms: vec![(self.round_radius(t.rep) as u32, 1.0)],
                }),
                _ => {}
            }
        }
        out
    }

    /// Whether each parameter is derived, by index — empty for a sketch with no ring, which every
    /// reader takes as "none is".
    pub fn derived_mask(&self) -> Vec<bool> {
        if self.turns.is_empty() {
            return Vec::new();
        }
        let mut m = vec![false; self.params.len()];
        for d in self.derived() {
            m[d.param as usize] = true;
        }
        m
    }

    /// Write every derived parameter's value from its bases — after anything writes the
    /// parameter vector (`set_x`), and after a build.
    pub fn settle_turns(&mut self) {
        if self.turns.is_empty() {
            return;
        }
        // a base is never derived, so each value is read off the parameters as they stand
        for d in self.derived() {
            let mut v = 0.0;
            for &(b, w) in &d.terms {
                v += w * self.params[b as usize].value;
            }
            self.params[d.param as usize].value = v;
        }
    }

    /// The turn of a turned curve's view, about its centre's place in that view: worked out once
    /// and applied to every sample.
    pub fn turn_in_view(&self, t: &Turn) -> impl Fn((f64, f64)) -> (f64, f64) {
        let (ox, oy) = self.point_xy(t.about.i());
        let (c, s) = t.cos_sin();
        move |(x, y)| {
            let (dx, dy) = (x - ox, y - oy);
            (ox + c * dx - s * dy, oy + s * dx + c * dy)
        }
    }

    /// The turn a curve is, with the curve it is a turn of — `None` for a curve of its own.
    pub fn curve_turn(&self, i: usize) -> Option<(usize, Turn)> {
        let t = *self.turn_of(EntRef::new(EntKind::Curve, i))?;
        (t.about.kind == EntKind::Point).then_some((t.rep.i(), t))
    }
}
