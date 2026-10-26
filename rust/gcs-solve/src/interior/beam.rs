//! **The Euler–Bernoulli beam** (#510 S0), the interior's second element: its axes, its rows in
//! them, its consistent loads and end actions, its loads spread along it, its mass and its
//! geometric stiffness.

use super::{Beam, Failure, Spread, Structure, SECTION};
use crate::cholesky::Symmetric;
use crate::envelope::held;
use crate::kernels::{Dual, Num};
use crate::space::{add, dot, norm, scale};

/// A number and its rates in a beam's run `Δ = x_b − x_a`.
type D = Dual<3>;

/// One of a beam's rows over its twelve degrees of freedom in its own axes: its near end's moves
/// along e1, e2, e3 and turns about them, then its far end's.
type Local = [D; 12];

/// Which of a beam's section's columns a structure in the plane bends with: its stretch and its
/// bending about z, in `SECTION`'s order.  Across the plane and its twist move nothing there.
const IN_PLANE: [bool; SECTION] = [true, false, true, false];

/// **The consistent load of `q`** (a force a length, in a beam's axes) along a beam of length
/// `h`, in its axes: half at each end, and the cubic's end moments `q h²/12` in each plane it
/// bends in — what `Structure::forces` lays at its ends, and what a reading of its end actions
/// takes out (`beams.rs`).
pub fn fixed_end(q: [f64; 3], h: f64) -> [f64; 12] {
    let (m2, m3) = (q[2] * h * h / 12.0, q[1] * h * h / 12.0);
    [
        q[0] * h / 2.0, q[1] * h / 2.0, q[2] * h / 2.0, 0.0, -m2, m3,
        q[0] * h / 2.0, q[1] * h / 2.0, q[2] * h / 2.0, 0.0, m2, -m3,
    ]
}

/// A beam's axes: e1, e2, e3, its rows.
pub type Axes = [[f64; 3]; 3];

/// A world vector read along a beam's axes.
pub fn into_axes(axes: &Axes, v: [f64; 3]) -> [f64; 3] {
    axes.map(|e| dot(e, v))
}

/// A vector along a beam's axes, in the world's: `into_axes`'s inverse.
pub fn out_of_axes(axes: &Axes, v: [f64; 3]) -> [f64; 3] {
    [0, 1, 2].map(|k| (0..3).map(|j| axes[j][k] * v[j]).sum())
}

impl Beam {
    /// Its length and axes at `Δ = x_b − x_a`, their rates in `Δ` carried: e1 along it, e2 = z ×
    /// e1 square to it in its plane, e3 = e1 × e2, the plane's normal.  `None` where it has no
    /// length, or z runs along it.
    pub(crate) fn axes(&self, d: [f64; 3]) -> Option<(D, [[D; 3]; 3])> {
        let d = [0, 1, 2].map(|k| D::var(d[k], k));
        let l = held::dot(d, d).sqrt();
        if !(l.v > 0.0) {
            return None;
        }
        let e1 = d.map(|c| c / l);
        let c = held::cross(self.z.map(D::cst), e1);
        let n = held::dot(c, c).sqrt();
        if !(n.v > 1e-9) {
            return None;
        }
        let e2 = c.map(|v| v / n);
        Some((l, [e1, e2, held::cross(e1, e2)]))
    }

    /// **Its strain rows in its own axes, at a unit section**, per column of the section: the
    /// stretch `√(E/L)·(u_b − u_a)·e1`; across its plane and in it, each two rows `√(E/L)·(2φ_a +
    /// φ_b)` and `√(3E/L)·φ_b`, the ends' turns `φ` against the chord, a Cholesky factor of the
    /// bending stiffness `E·I/L·[4 2; 2 4]`; and the twist `√(G/L)·(θ_b − θ_a)·e1`.  Each scales by
    /// the root of its column, so the element's stiffness is linear in its section.
    fn local(&self, l: D) -> [Vec<Local>; SECTION] {
        let zero = D::cst(0.0);
        let (c, t) = ((D::cst(self.e) / l).sqrt(), (D::cst(self.g) / l).sqrt());
        let s3 = D::cst(3f64.sqrt());
        let row = |entries: &[(usize, D)]| {
            let mut r = [zero; 12];
            for &(k, v) in entries {
                r[k] = v;
            }
            r
        };
        let three = D::cst(3.0) * c / l;
        let root = s3 * c / l;
        let (neg, two) = (D::cst(-1.0), D::cst(2.0));
        [
            vec![row(&[(0, neg * c), (6, c)])],
            // about e2, its moves along e3: a turn about e2 is the slope's negative
            vec![
                row(&[(2, three), (8, neg * three), (4, neg * two * c), (10, neg * c)]),
                row(&[(2, root), (8, neg * root), (10, neg * s3 * c)]),
            ],
            // about e3, its moves along e2
            vec![
                row(&[(1, three), (7, neg * three), (5, two * c), (11, c)]),
                row(&[(1, root), (7, neg * root), (11, s3 * c)]),
            ],
            vec![row(&[(3, neg * t), (9, t)])],
        ]
    }
}

impl Structure {
    /// Beam `i`'s ends, as nodes.
    pub(super) fn beam_ends(&self, i: usize) -> [usize; 2] {
        [self.beams[i].a, self.beams[i].b]
    }

    /// Beam `i`'s axes at `x`, their rates in its run carried, and its length.
    fn beam_axes_d(&self, i: usize, x: &[f64]) -> Option<(D, [[D; 3]; 3])> {
        let b = &self.beams[i];
        b.axes(self.place(b.a, b.b, x))
    }

    /// Beam `i`'s axes at `x` as plain numbers, and its length.
    pub fn beam_axes(&self, i: usize, x: &[f64]) -> Option<(f64, Axes)> {
        let (l, axes) = self.beam_axes_d(i, x)?;
        Some((l.v, axes.map(|e| e.map(|c| c.v))))
    }

    /// Beam `i`'s rows in its own axes at a unit section, of length `l`, per column of its
    /// section, those the structure bends with (`IN_PLANE` in the plane) — or why it is no
    /// member: a modulus or a column it reads at zero or less.
    pub(super) fn beam_local(&self, i: usize, l: D, x: &[f64])
        -> Result<[Vec<Local>; SECTION], Failure>
    {
        let b = &self.beams[i];
        let mut out = b.local(l);
        for (group, rows) in out.iter_mut().enumerate() {
            if self.dim == 2 && !IN_PLANE[group] {
                rows.clear();
            }
        }
        let moduli = [b.e, b.e, b.e, b.g];
        for ((rows, &p), m) in out.iter().zip(self.section(i, x)).zip(moduli) {
            if !rows.is_empty() && !(p > 0.0 && m > 0.0) {
                return Err(Failure::Unstiff);
            }
        }
        Ok(out)
    }

    /// Beam `i`'s rows at `x`, per column of its section, each over its twelve degrees of
    /// freedom in space — the world's moves and turns at each end — at a unit section.
    pub(super) fn beam_rows(&self, i: usize, x: &[f64]) -> Result<[Vec<Local>; SECTION], Failure> {
        let (l, axes) = self.beam_axes_d(i, x).ok_or(Failure::Collapsed)?;
        let mut out = self.beam_local(i, l, x)?;
        for r in out.iter_mut().flatten() {
            let mut g = [D::cst(0.0); 12];
            for block in 0..4 {
                for k in 0..3 {
                    let mut s = D::cst(0.0);
                    for j in 0..3 {
                        s = s + r[3 * block + j] * axes[j][k];
                    }
                    g[3 * block + k] = s;
                }
            }
            *r = g;
        }
        Ok(out)
    }

    /// Beam `i`'s twelve components in space (each end's moves and turns) that the structure
    /// has, each with its degree of freedom.
    fn beam_dofs(&self, i: usize) -> impl Iterator<Item = (usize, usize)> + '_ {
        self.beam_ends(i).into_iter().enumerate().flat_map(move |(end, node)| {
            (0..6).filter_map(move |c| self.dof(node, c).map(|d| (6 * end + c, d)))
        })
    }

    /// Each entry of beam `i`'s row `r` (over its twelve degrees of freedom in space) at the
    /// structure's degree of freedom it falls on, given to `put`; one the structure lacks (in the
    /// plane, a move along z or a turn about x or y) is dropped.
    pub(super) fn beam_put(&self, i: usize, r: &Local, mut put: impl FnMut(usize, D)) {
        for (k, dof) in self.beam_dofs(i) {
            put(dof, r[k]);
        }
    }

    /// **Each beam's end actions under `u`** (#510 S0), in its own axes: `k_e u_e`, the forces
    /// and moments its ends' nodes hold it by — the near end's force along e1, e2, e3 and moment
    /// about them, then the far end's.  A load spread along the beam (`spread`) is the caller's
    /// to take out (its fixed-end actions): these are the element's own stiffness alone.  The
    /// stretch is `[6]`, the far end's pull; `None` for a beam with no axes.
    pub fn end_actions(&self, u: &[f64], x: &[f64]) -> Vec<Option<[f64; 12]>> {
        (0..self.beams.len()).map(|i| {
            let (l, axes) = self.beam_axes_d(i, x)?;
            let axes = axes.map(|e| e.map(|c| c.v));
            let mut ue = [0.0; 12];
            for (k, dof) in self.beam_dofs(i) {
                ue[k] = u[dof];
            }
            let mut ul = [0.0; 12];
            for block in 0..4 {
                let v = into_axes(&axes, [0, 1, 2].map(|k| ue[3 * block + k]));
                ul[3 * block..3 * block + 3].copy_from_slice(&v);
            }
            let mut f = [0.0; 12];
            for (rows, &p) in self.beam_local(i, l, x).ok()?.iter().zip(self.section(i, x)) {
                for r in rows {
                    let s: f64 = (0..12).map(|k| r[k].v * ul[k]).sum::<f64>() * p;
                    for k in 0..12 {
                        f[k] += r[k].v * s;
                    }
                }
            }
            Some(f)
        }).collect()
    }

    /// **Each beam's load spread along it** at `x` (#510 S1), a force a length in the world's
    /// axes: the sum of every `Spread` it carries, an even one's force over the length of all the
    /// beams it lies along, a body load's times the beam's area.
    pub fn spread_at(&self, x: &[f64]) -> Vec<[f64; 3]> {
        let mut q = vec![[0.0; 3]; self.beams.len()];
        for s in &self.spread {
            let total = if s.per_area { 1.0 } else { self.spread_length(s, x) };
            for &i in &s.beams {
                let w = if s.per_area { self.section(i, x)[0] } else { 1.0 / total };
                q[i] = add(q[i], scale(s.force, w));
            }
        }
        q
    }

    /// Beam `i`'s run at `x`, `x_b − x_a`.
    fn beam_run(&self, i: usize, x: &[f64]) -> [f64; 3] {
        self.place(self.beams[i].a, self.beams[i].b, x)
    }

    /// The length of every beam spread load `s` lies along, at `x`.
    fn spread_length(&self, s: &Spread, x: &[f64]) -> f64 {
        s.beams.iter().map(|&i| norm(self.beam_run(i, x))).sum()
    }

    /// **The loads at `x`**, per degree of freedom: the loads at the nodes, and each beam's
    /// spread load as its consistent load at its ends (`fixed_end`, in its axes, turned into the
    /// world's), exact for the cubic.  A component the structure lacks (in the plane, a turn about
    /// x or y) is dropped.
    pub fn forces(&self, x: &[f64]) -> Vec<f64> {
        let mut f = self.load.clone();
        self.laid_forces(x, &mut f);
        self.borne_forces(x, &mut f);
        if self.spread.is_empty() {
            return f;
        }
        for (i, q) in self.spread_at(x).into_iter().enumerate() {
            let Some((h, axes)) = self.beam_axes(i, x) else { continue };
            let local = fixed_end(into_axes(&axes, q), h);
            for (k, dof) in self.beam_dofs(i) {
                let block = k / 3 * 3;
                f[dof] += out_of_axes(&axes, [0, 1, 2].map(|j| local[block + j]))[k % 3];
            }
        }
        f
    }

    /// **`λᵀ ∂f/∂x`, added into `g`** (#510 S1): each beam's share of a spread load is linear in
    /// its `q`, `λₑᵀfₑ = q · wₑ` with `wₑ = L/2 (λ_a + λ_b) + L/12 (θ_a − θ_b) × Δ` (`λ` its
    /// ends' moves, `θ` their turns), and `wₑ` is formed in `Dual<3>` over `Δ`.  A body load's
    /// `q` is its area's times the force, so its column reads `force · wₑ`; an even one's is
    /// the force over the length `S` of every beam it lies along, so each of their runs reads
    /// `−(Σ force · w)/S² · Δ/L` besides its own `force · ∂w/S`.
    pub(super) fn spread_into(&self, lam: &[f64], x: &[f64], g: &mut [f64]) {
        let w = |i: usize| -> [D; 3] {
            let r = self.beam_run(i, x);
            let d = [0, 1, 2].map(|k| D::var(r[k], k));
            let h = held::dot(d, d).sqrt();
            // its ends' λ, each end's moves then turns, as `beam_dofs` lays them out
            let mut l = [0.0; 12];
            for (k, dof) in self.beam_dofs(i) {
                l[k] = lam[dof];
            }
            let turn = [0, 1, 2].map(|c| D::cst(l[3 + c] - l[9 + c]));
            let m = held::cross(turn, d);
            [0, 1, 2].map(|c| h * D::cst(0.5 * (l[c] + l[6 + c])) + h * D::cst(1.0 / 12.0) * m[c])
        };
        // a rate in a beam's run: its far end's coordinates gain it, its near end's lose it
        let run = |i: usize, by: [f64; 3], g: &mut [f64]| {
            let [a, b] = self.beam_ends(i);
            for k in 0..self.dim {
                g[self.dim * b + k] += by[k];
                g[self.dim * a + k] -= by[k];
            }
        };
        for s in &self.spread {
            let f = s.force.map(D::cst);
            if s.per_area {
                for &i in &s.beams {
                    let fw = held::dot(f, w(i));
                    let a = self.section(i, x)[0];
                    run(i, fw.g.map(|v| a * v), g);
                    g[self.section_at(i)] += fw.v;
                }
                continue;
            }
            let fws: Vec<D> = s.beams.iter().map(|&i| held::dot(f, w(i))).collect();
            let total = self.spread_length(s, x);
            let whole: f64 = fws.iter().map(|v| v.v).sum::<f64>() / total;
            for (&i, fw) in s.beams.iter().zip(&fws) {
                let d = self.beam_run(i, x);
                let l = norm(d);
                let by = [0, 1, 2].map(|k| fw.g[k] / total - whole / total * d[k] / l);
                run(i, by, g);
            }
        }
    }

    /// Every beam's twelve-by-twelve matrix in its own axes, `local(i, length, axes)`, turned
    /// into the world's and added at the equations `eq` numbers, as triplets of its lower
    /// triangle, all a `Symmetric` reads.
    pub(super) fn beams_into(&self, x: &[f64], eq: &[Option<usize>],
                             t: &mut Vec<(usize, usize, f64)>,
                             local: impl Fn(usize, D, &Axes) -> Result<[[f64; 12]; 12], Failure>)
        -> Result<(), Failure>
    {
        for i in 0..self.beams.len() {
            let (l, axes_d) = self.beam_axes_d(i, x).ok_or(Failure::Collapsed)?;
            let axes = axes_d.map(|e| e.map(|c| c.v));
            let m = local(i, l, &axes)?;
            // the world's component k of each end's moves and turns is Σ_j axes[j][k]·(local j)
            let at: Vec<(usize, [f64; 12])> = self.beam_dofs(i).filter_map(|(k, dof)| {
                let e = eq[dof]?;
                let mut col = [0.0; 12];
                for j in 0..3 {
                    col[k / 3 * 3 + j] = axes[j][k % 3];
                }
                Some((e, col))
            }).collect();
            for (ei, ci) in &at {
                for (ej, cj) in at.iter().filter(|(ej, _)| ej <= ei) {
                    let mut s = 0.0;
                    for p in 0..12 {
                        if ci[p] == 0.0 {
                            continue;
                        }
                        for q in 0..12 {
                            s += ci[p] * m[p][q] * cj[q];
                        }
                    }
                    if s != 0.0 {
                        t.push((*ei, *ej, s));
                    }
                }
            }
        }
        Ok(())
    }

    /// **The consistent mass** (#510 S0), over the equations no support holds: each beam's
    /// `ρAL/6·[2 1; 1 2]` along itself, `ρ(Iy + Iz)L/6·[2 1; 1 2]` in its twist, and the cubic's
    /// `ρAL/420·[156 22L 54 −13L; …]` in each plane it bends in.  A bar carries none.  Euler–
    /// Bernoulli's: no inertia of the section's turn in bending.
    /// A plane or a solid structure's is each element's consistent mass, assembled in place on
    /// the pattern its stiffness is (`in_place`).
    pub fn mass(&self, x: &[f64]) -> Result<Symmetric, Failure> {
        let (eq, n_eq) = self.equations();
        if self.meshed() {
            return self.in_place(&eq, n_eq, |i| self.tris[i].mass(&self.tri_places(i, x)),
                                 |i| self.tets[i].mass(&self.tet_places(i, x)));
        }
        let mut t = Vec::new();
        self.beams_into(x, &eq, &mut t, |i, l, _| {
            let (l, s) = (l.v, self.section(i, x));
            let (rho, a, ip) = (self.beams[i].rho, s[0], s[1] + s[2]);
            let mut m = [[0.0; 12]; 12];
            pair(&mut m, 0, 6, rho * a * l / 3.0, rho * a * l / 6.0);
            pair(&mut m, 3, 9, rho * ip * l / 3.0, rho * ip * l / 6.0);
            let cubic = [[156.0, 22.0 * l, 54.0, -13.0 * l], [22.0 * l, 4.0 * l * l, 13.0 * l,
                         -3.0 * l * l], [54.0, 13.0 * l, 156.0, -22.0 * l],
                         [-13.0 * l, -3.0 * l * l, -22.0 * l, 4.0 * l * l]];
            bending(&mut m, &cubic, rho * a * l / 420.0);
            Ok(m)
        })?;
        Ok(Symmetric::from_triplets(n_eq, &t))
    }

    /// **The geometric stiffness** (#510 S0) of each beam carrying the pull `axial[i]` (negative
    /// pressing), over the equations no support holds: the cubic's `N/30L·[36 3L −36 3L; …]` in
    /// each plane it bends in, and `N(Iy + Iz)/(AL)·[1 −1; −1 1]` in its twist — what
    /// `eigen::buckling` reads beside `stiffness`.
    pub fn geometric(&self, x: &[f64], axial: &[f64]) -> Result<Symmetric, Failure> {
        let (eq, n_eq) = self.equations();
        let mut t = Vec::new();
        self.beams_into(x, &eq, &mut t, |i, l, _| {
            let (l, s, n) = (l.v, self.section(i, x), axial[i]);
            let mut m = [[0.0; 12]; 12];
            let w = n * (s[1] + s[2]) / (s[0] * l);
            pair(&mut m, 3, 9, w, -w);
            let cubic = [[36.0, 3.0 * l, -36.0, 3.0 * l], [3.0 * l, 4.0 * l * l, -3.0 * l, -l * l],
                         [-36.0, -3.0 * l, 36.0, -3.0 * l], [3.0 * l, -l * l, -3.0 * l,
                         4.0 * l * l]];
            bending(&mut m, &cubic, n / (30.0 * l));
            Ok(m)
        })?;
        Ok(Symmetric::from_triplets(n_eq, &t))
    }
}

/// `[diag off; off diag]` added over degrees of freedom `p` and `q` of a beam's twelve.
fn pair(m: &mut [[f64; 12]; 12], p: usize, q: usize, diag: f64, off: f64) {
    m[p][p] += diag;
    m[q][q] += diag;
    m[p][q] += off;
    m[q][p] += off;
}

/// A cubic's four-by-four matrix `c`, times `w`, added in both planes a beam bends in: over its
/// moves along e2 and turns about e3, and over its moves along e3 and the negatives of its turns
/// about e2 (a turn about e2 is the slope's negative).
fn bending(m: &mut [[f64; 12]; 12], c: &[[f64; 4]; 4], w: f64) {
    for (dofs, signs) in [([1, 5, 7, 11], [1.0, 1.0, 1.0, 1.0]),
                          ([2, 4, 8, 10], [1.0, -1.0, 1.0, -1.0])] {
        for p in 0..4 {
            for q in 0..4 {
                m[dofs[p]][dofs[q]] += w * signs[p] * signs[q] * c[p][q];
            }
        }
    }
}
