//! A row whose value is the output of a linear solve (#510 P5): an analysis's interior.
//!
//! A requirement on a structure — a tip moves no more than half a millimetre — is a row whose
//! residual is an output `J(u)` of the displacements `u` that solve `K(x)u = f`, where the
//! stiffness `K` is assembled from the drawing's own numbers `x` (where the nodes stand, how
//! big each member is).  The row is a **kernel**, not a boundary of `solve/through.rs`: `u` is no
//! unknown of the drawing — a boundary's inner unknowns are parameters of the one `System` and
//! its inner solve is the square Newton of `newton::square` on a dense LU, where an analysis has
//! thousands of displacements, a solve that is linear, and nothing to show of them in the
//! drawing — and a kernel is a row like every other: the flat solve, the decomposition, the
//! diagnosis, a claim and a free twin take it as they take a spline's length (`integral.rs`),
//! which reads every control point of its spline as this reads every node and member.
//!
//! The value is one factorisation of `K` and one solve; the gradient is **the adjoint**, one more
//! solve on the factor already in hand, `Kᵀλ = ∂J/∂u`, then `dJ/dx = ∂J/∂x − λᵀ(∂K/∂x·u − ∂f/∂x)`
//! — whatever number of columns the row reads, where the implicit function theorem of a
//! boundary answers one column at a time.  `∂K/∂x·u` is never formed: `λᵀK(x)u` is a sum of
//! each element's mutual energy `λₑᵀkₑ(xₑ)uₑ`, and only its gradient in the element's own
//! columns is taken (`Bar::mutual`).
//!
//! The stiffness is the **Gram matrix of strain rows**: an element's `kₑ` is `Σ sᵣsᵣᵀ` over rows
//! `sᵣ` of its own degrees of freedom — one for a pin-jointed bar, `√(EA/L)·(−ĝ, ĝ)` — so `K` is
//! `SᵀS` and the normal equations the solver already orders and factors (`sparse::Ata`) are
//! the factorisation.  A support holds a degree of freedom at zero by leaving its column out.
//!
//! Two elements are here, each its rows.  The pin-jointed bar (#510 P5), in the plane or in
//! space: the least that is a structure.  And the Euler–Bernoulli beam (#510 S0), rigid at both
//! ends, whose nodes turn as well as move: degrees of freedom numbered apart from coordinates
//! (`Structure::per`), and a section of four columns (`SECTION`).  Its stiffness is six rows — its
//! stretch, its twist, and two for each plane it bends in, a Cholesky factor of `[4 2; 2 4]`
//! times its ends' turns against its chord — so it joins the solve, the gradient and the
//! mechanism as the bar does.  Plane elements and solids are further elements of the same
//! interior (#510 S2–S3).

use crate::diagnose::analysis::{self, Freedom, Held, Reach};
use crate::kernels::{curve_widths, Dual, Kernel, Num};
use crate::linalg::{rank_and_nullspace, Mat};
use crate::envelope::held;
use crate::space::{dot, norm, scale, sub};
use crate::sparse::{Ata, Sym};

/// Below this least pivot (`Ata::least_pivot`, a pivot as a fraction of its diagonal) the
/// stiffness is singular: a motion the supports leave free costs only rounding's worth of
/// energy, its pivot within a few thousand times the precision of zero, where a structure that
/// holds it pivots at a fraction of its diagonal.
pub const SINGULAR: f64 = 1e-12;

/// A pin-jointed bar between nodes `a` and `b`, of modulus `e`; its area is a column.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bar {
    pub a: usize,
    pub b: usize,
    pub e: f64,
}

/// An Euler–Bernoulli beam between nodes `a` and `b` (#510 S0), rigid at both: of modulus `e`,
/// shear modulus `g` and density `rho`, its section's z along `z` — the normal of the plane it is
/// drawn in, so it bends in that plane about z and across it about y.  Its section is four
/// columns, in `SECTION`'s order.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Beam {
    pub a: usize,
    pub b: usize,
    pub e: f64,
    pub g: f64,
    pub rho: f64,
    pub z: [f64; 3],
}

/// How many columns a beam's section is, as they run: its area, its second moments about its y
/// and its z (bending across its plane and in it), and its torsion constant
/// (`model::SectionProp::STIFFNESS`).
pub const SECTION: usize = 4;

/// A structure of bars and beams: `nodes` points of `dim` (2 or 3) coordinates each, and per
/// degree of freedom (`per · node + k`, `Structure::per`) whether a support holds it, the load on
/// it, and its weight in the output `J = Σ weight · u`.  A node's degrees of freedom are its
/// moves along the axes, then where there is a beam its turns: about z in the plane, about x, y
/// and z in space.  The weights are numbers, a direction's cosines, so the output is a
/// displacement: a length.
///
/// Its columns are every node's coordinates, then every bar's area, then every beam's section
/// (`Structure::columns`); the moduli, the densities, the supports, the loads and the weights are
/// constants.
#[derive(Clone, Debug, PartialEq)]
pub struct Structure {
    pub dim: usize,
    pub nodes: usize,
    pub bars: Vec<Bar>,
    pub beams: Vec<Beam>,
    pub fixed: Vec<bool>,
    pub load: Vec<f64>,
    pub weight: Vec<f64>,
}

/// Why a structure has no displacement to give.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Failure {
    /// `K` is singular: the supports leave a motion that costs no energy (`SINGULAR`).
    Free,
    /// A bar or a beam of no length, which has no direction to be stiff along — or a beam whose
    /// section's z runs along it, so it has no plane to bend in.
    Collapsed,
    /// A bar of no stiffness or less (`E·A ≤ 0`), or a beam with a modulus or a section number
    /// it reads at zero or less, which is no member.
    Unstiff,
}

/// The displacements of a structure at `x`, with the factor they were solved on held for the
/// adjoint.
pub struct State {
    ata: Ata,
    /// Per degree of freedom, its equation, or `None` where a support holds it.
    eq: Vec<Option<usize>>,
    /// Per degree of freedom, zero where held.
    pub u: Vec<f64>,
}

impl State {
    /// `f`, per degree of freedom, at the equations: the ones no support holds.
    fn gather(&self, f: &[f64]) -> Vec<f64> {
        self.eq.iter().zip(f).filter(|(e, _)| e.is_some()).map(|(_, v)| *v).collect()
    }

    /// A solution over the equations, per degree of freedom: zero where a support holds it.
    fn scatter(&self, r: &[f64]) -> Vec<f64> {
        self.eq.iter().map(|e| e.map_or(0.0, |e| r[e])).collect()
    }
}

impl Structure {
    /// How many columns the structure reads: the nodes' coordinates, the bars' areas, then the
    /// beams' sections.
    pub fn columns(&self) -> usize {
        self.dim * self.nodes + self.bars.len() + SECTION * self.beams.len()
    }

    /// Where beam `i`'s section starts among the columns.
    fn section_at(&self, i: usize) -> usize {
        self.dim * self.nodes + self.bars.len() + SECTION * i
    }

    /// Beam `i`'s section at `x`, in `SECTION`'s order.
    pub fn section<'a>(&self, i: usize, x: &'a [f64]) -> &'a [f64] {
        &x[self.section_at(i)..self.section_at(i) + SECTION]
    }

    /// **A node's components in space**, the ones the structure has, in the order its degrees of
    /// freedom run: 0–2 a move along x, y, z, 3–5 a turn about them.  Its moves, and where there
    /// is a beam its turns — one about z in the plane, three in space.
    fn components(&self) -> &'static [usize] {
        match (self.beams.is_empty(), self.dim) {
            (true, 2) => &[0, 1],
            (true, _) => &[0, 1, 2],
            (false, 2) => &[0, 1, 5],
            (false, _) => &[0, 1, 2, 3, 4, 5],
        }
    }

    /// How many degrees of freedom a node has.
    pub fn per(&self) -> usize {
        self.components().len()
    }

    /// The degree of freedom of node `n`'s component `c` in space, or `None` where the structure
    /// has none: in the plane, a move along z or a turn about x or y.
    pub fn dof(&self, n: usize, c: usize) -> Option<usize> {
        self.components().iter().position(|&k| k == c).map(|k| self.per() * n + k)
    }

    /// Degree of freedom `i` as its node and its component in space: `dof`'s inverse.
    fn component(&self, i: usize) -> (usize, usize) {
        (i / self.per(), self.components()[i % self.per()])
    }

    /// Bar `i`'s geometry at `x`: `Δ = x_b − x_a`, its length, and `E·A`.
    fn bar_at(&self, i: usize, x: &[f64]) -> ([f64; 3], f64, f64) {
        let b = &self.bars[i];
        let d = self.place(b.a, b.b, x);
        (d, norm(d), b.e * x[self.dim * self.nodes + i])
    }

    /// Node `b`'s place less node `a`'s, at `x`.
    fn place(&self, a: usize, b: usize, x: &[f64]) -> [f64; 3] {
        sub(self.node(b, x), self.node(a, x))
    }

    /// Bar `b`'s move at its far end less its near one, under `u` per degree of freedom.
    fn stretch(&self, b: &Bar, u: &[f64]) -> [f64; 3] {
        let mut d = [0.0; 3];
        for k in 0..self.dim {
            let (pa, pb) = (self.dof(b.a, k).unwrap(), self.dof(b.b, k).unwrap());
            d[k] = u[pb] - u[pa];
        }
        d
    }

    /// Assemble `K(x)` as strain rows, factor it and solve `K u = f`.
    pub fn solve(&self, x: &[f64]) -> Result<State, Failure> {
        let (eq, n_eq) = self.equations();
        let (mut indptr, mut indices, mut data) = (vec![0i32], Vec::new(), Vec::new());
        self.rows(x, &eq, |row| {
            for &(e, v) in row {
                indices.push(e as i32);
                data.push(v);
            }
            indptr.push(indices.len() as i32);
        })?;
        let mut ata = Ata::new(indptr.len() - 1, n_eq, &indptr, &indices);
        ata.fill(&data);
        let mut st = State { ata, eq, u: Vec::new() };
        let mut rhs = st.gather(&self.load);
        if !st.ata.solve(&vec![0.0; n_eq], &mut rhs) || st.ata.least_pivot() < SINGULAR {
            return Err(Failure::Free);
        }
        st.u = st.scatter(&rhs);
        Ok(st)
    }

    /// Every element's strain rows at `x`, each handed to `row` as its `(equation, entry)`s at the
    /// equations `eq` numbers: what `solve` factors.  A row every degree of freedom of which is
    /// held is handed on empty.
    fn rows(&self, x: &[f64], eq: &[Option<usize>], mut row: impl FnMut(&[(usize, f64)]))
        -> Result<(), Failure>
    {
        let mut at: Vec<(usize, f64)> = Vec::new();
        for (i, b) in self.bars.iter().enumerate() {
            let (d, l, ea) = self.bar_at(i, x);
            check_bar(l, ea)?;
            at.clear();
            self.strain(b, scale(d, (ea / l).sqrt() / l), eq, |e, v| at.push((e, v)));
            row(&at);
        }
        for i in 0..self.beams.len() {
            let section = self.section(i, x);
            let rows = self.beam_rows(i, x)?;
            for (rs, &p) in rows.iter().zip(section) {
                for r in rs {
                    at.clear();
                    self.beam_put(i, r, |dof, v| {
                        if let Some(e) = eq[dof] {
                            at.push((e, p.sqrt() * v.v));
                        }
                    });
                    row(&at);
                }
            }
        }
        Ok(())
    }

    /// **`K(x)`, assembled** over the equations no support holds — the matrix `solve` factors,
    /// for a pencil (`eigen.rs`) to read: each bar's strain row's outer product, each beam's
    /// twelve-by-twelve stiffness `Σ p rrᵀ` in its own axes, turned.
    pub fn stiffness(&self, x: &[f64]) -> Result<Sym, Failure> {
        let (eq, n_eq) = self.equations();
        let mut t = Vec::new();
        for (i, b) in self.bars.iter().enumerate() {
            let (d, l, ea) = self.bar_at(i, x);
            check_bar(l, ea)?;
            let mut row = Vec::new();
            self.strain(b, scale(d, (ea / l).sqrt() / l), &eq, |e, v| row.push((e, v)));
            for &(i, a) in &row {
                t.extend(row.iter().map(|&(j, b)| (i, j, a * b)));
            }
        }
        self.beams_into(x, &eq, &mut t, |i, l, _| {
            let mut m = [[0.0; 12]; 12];
            for (rs, &p) in self.beam_local(i, l, x)?.iter().zip(self.section(i, x)) {
                for r in rs {
                    for a in 0..12 {
                        for b in 0..12 {
                            m[a][b] += p * r[a].v * r[b].v;
                        }
                    }
                }
            }
            Ok(m)
        })?;
        Ok(Sym::from_triplets(n_eq, &t))
    }

    /// Per degree of freedom its equation, `None` where a support holds it, and how many there
    /// are: the numbering `solve` factors and `free` reads.
    fn equations(&self) -> (Vec<Option<usize>>, usize) {
        let mut n = 0;
        let eq = self.fixed.iter().map(|held| (!held).then(|| { n += 1; n - 1 })).collect();
        (eq, n)
    }

    /// Bar `b`'s strain row `g·(u_b − u_a)`, `g` its direction scaled as the caller weighs it, at
    /// the equations `eq` numbers: each `(equation, entry)` given to `put`.
    fn strain(&self, b: &Bar, g: [f64; 3], eq: &[Option<usize>], mut put: impl FnMut(usize, f64)) {
        for (node, sign) in [(b.a, -1.0), (b.b, 1.0)] {
            for k in 0..self.dim {
                if let Some(e) = eq[self.dof(node, k).unwrap()] {
                    put(e, sign * g[k]);
                }
            }
        }
    }

    /// The output, `Σ weight · u`.
    pub fn output(&self, st: &State) -> f64 {
        self.weight.iter().zip(&st.u).map(|(w, u)| w * u).sum()
    }

    /// The output's gradient in the columns at `x`, added into `g`: one solve for `λ` on the
    /// factor `st` holds — a `State` is only read, so no factorisation is made again — then each
    /// element's mutual energy differentiated.  The loads do not move with `x` and the output
    /// reads only `u`, so `∂f/∂x` and `∂J/∂x` are zero and what is left is `−λᵀ(∂K/∂x)u`.
    pub fn gradient(&self, st: &State, x: &[f64], g: &mut [f64]) {
        let mut rhs = st.gather(&self.weight);
        st.ata.resolve(&mut rhs);
        let lam = st.scatter(&rhs);
        for (i, b) in self.bars.iter().enumerate() {
            let (d, l, ea) = self.bar_at(i, x);
            let (al, be) = (self.stretch(b, &lam), self.stretch(b, &st.u));
            let (dm, da) = b.mutual(d, l, ea, al, be);
            for k in 0..self.dim {
                g[self.dim * b.b + k] -= dm[k];
                g[self.dim * b.a + k] += dm[k];
            }
            g[self.dim * self.nodes + i] -= da;
        }
        for i in 0..self.beams.len() {
            let Ok(rows) = self.beam_rows(i, x) else { continue };
            let (ends, at) = (self.beam_ends(i), self.section_at(i));
            for (group, rs) in rows.iter().enumerate() {
                // the group's mutual energy at a unit section: the whole is the section times it
                let mut m = Dual::<3>::cst(0.0);
                for r in rs {
                    let (mut rl, mut ru) = (Dual::cst(0.0), Dual::cst(0.0));
                    self.beam_put(i, r, |dof, v| {
                        rl = rl + v * Dual::cst(lam[dof]);
                        ru = ru + v * Dual::cst(st.u[dof]);
                    });
                    m = m + rl * ru;
                }
                let p = x[at + group];
                for k in 0..self.dim {
                    g[self.dim * ends[1] + k] -= p * m.g[k];
                    g[self.dim * ends[0] + k] += p * m.g[k];
                }
                g[at + group] -= m.v;
            }
        }
    }
}

/// A bar of length `l` and stiffness `ea` that is one, or why it is none.
fn check_bar(l: f64, ea: f64) -> Result<(), Failure> {
    if l.is_nan() || l <= 0.0 {
        return Err(Failure::Collapsed);
    }
    if ea.is_nan() || ea <= 0.0 {
        return Err(Failure::Unstiff);
    }
    Ok(())
}

/* -- the beam --------------------------------------------------------------------------------- */

/// A number and its rates in a beam's run `Δ = x_b − x_a`.
type D = Dual<3>;

/// One of a beam's rows over its twelve degrees of freedom in its own axes: its near end's moves
/// along e1, e2, e3 and turns about them, then its far end's.
type Local = [D; 12];

/// Which of a beam's section's columns a structure in the plane bends with: its stretch and its
/// bending about z, in `SECTION`'s order.  Across the plane and its twist move nothing there.
const IN_PLANE: [bool; SECTION] = [true, false, true, false];

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
    fn beam_ends(&self, i: usize) -> [usize; 2] {
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
    fn beam_local(&self, i: usize, l: D, x: &[f64]) -> Result<[Vec<Local>; SECTION], Failure> {
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
    fn beam_rows(&self, i: usize, x: &[f64]) -> Result<[Vec<Local>; SECTION], Failure> {
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
    fn beam_put(&self, i: usize, r: &Local, mut put: impl FnMut(usize, D)) {
        for (k, dof) in self.beam_dofs(i) {
            put(dof, r[k]);
        }
    }

    /// **Each beam's end actions under `u`** (#510 S0), in its own axes: `k_e u_e`, the forces
    /// and moments its ends' nodes hold it by — the near end's force along e1, e2, e3 and moment
    /// about them, then the far end's.  A load spread along the beam is the caller's to take
    /// out (its fixed-end actions), since the structure carries only its nodes' loads.  The
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

    /// Beam `i`'s twelve actions in its own axes, `local`, turned into the world's and added into
    /// `f` per degree of freedom: a load spread along it, given at its ends.
    pub fn beam_load(&self, i: usize, axes: &Axes, local: &[f64; 12], f: &mut [f64]) {
        let mut world = [0.0; 12];
        for block in 0..4 {
            let v = out_of_axes(axes, [0, 1, 2].map(|j| local[3 * block + j]));
            world[3 * block..3 * block + 3].copy_from_slice(&v);
        }
        for (k, dof) in self.beam_dofs(i) {
            f[dof] += world[k];
        }
    }

    /// Every beam's twelve-by-twelve matrix in its own axes, `local(i, length, axes)`, turned
    /// into the world's and added at the equations `eq` numbers, as triplets.
    fn beams_into(&self, x: &[f64], eq: &[Option<usize>], t: &mut Vec<(usize, usize, f64)>,
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
                for (ej, cj) in &at {
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
    pub fn mass(&self, x: &[f64]) -> Result<Sym, Failure> {
        let (eq, n_eq) = self.equations();
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
        Ok(Sym::from_triplets(n_eq, &t))
    }

    /// **The geometric stiffness** (#510 S0) of each beam carrying the pull `axial[i]` (negative
    /// pressing), over the equations no support holds: the cubic's `N/30L·[36 3L −36 3L; …]` in
    /// each plane it bends in, and `N(Iy + Iz)/(AL)·[1 −1; −1 1]` in its twist — what
    /// `eigen::buckling` reads beside `stiffness`.
    pub fn geometric(&self, x: &[f64], axial: &[f64]) -> Result<Sym, Failure> {
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
        Ok(Sym::from_triplets(n_eq, &t))
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

/// Past this many degrees of freedom no dense null space is taken to find a mechanism: the
/// diagnosis's own limit (`diagnose::NUMERIC_MAX`).
const MECHANISM_MAX: usize = crate::diagnose::NUMERIC_MAX;

/// **What leaves a structure free** (#510 P7), its `Failure::Free` named: the rigid-body motions
/// its supports leave (`analysis::freedoms`, from the held degrees of freedom alone), and the
/// nodes a mechanism moves — a motion of the nodes no bar resists and no rigid motion makes.
/// `mechanism` is `None` past `MECHANISM_MAX` free degrees of freedom, where it is not looked for.
#[derive(Clone, Debug, PartialEq)]
pub struct Free {
    pub rigid: Vec<Freedom>,
    pub mechanism: Option<Vec<usize>>,
}

impl Free {
    /// `free to turn about (0, 0)`, `a mechanism: nodes 2 and 3 move with no bar stretched` — or
    /// `None` where it is held.
    pub fn text(&self, dim: usize) -> Option<String> {
        let mut parts: Vec<String> =
            analysis::freedoms_text(&self.rigid, dim).into_iter().collect();
        if let Some(nodes) = self.mechanism.as_ref().filter(|m| !m.is_empty()) {
            let names: Vec<String> = nodes.iter().map(|n| n.to_string()).collect();
            let one = names.len() == 1;
            let (what, moves) = if one { ("node", "moves") } else { ("nodes", "move") };
            parts.push(format!("a mechanism: {what} {} {moves} with no bar stretched",
                               crate::vocab::all_of(&names)));
        }
        (!parts.is_empty()).then(|| parts.join("; "))
    }
}

impl Structure {
    /// Node `i`'s place at `x`, in space.
    fn node(&self, i: usize, x: &[f64]) -> [f64; 3] {
        let mut p = [0.0; 3];
        p[..self.dim].copy_from_slice(&x[self.dim * i..self.dim * (i + 1)]);
        p
    }

    /// **What the supports and the members leave free at `x`** (#510 P7): `Failure::Free`, named.
    /// The rigid motions are the supports': every held move holds its node along its axis, and a
    /// held turn holds the body's turn.  A mechanism is what is left of the motions no member
    /// strains (each element's strain rows, a bar's `ĝ·(u_b − u_a) = 0`) once the rigid ones are
    /// taken out: its nodes are those it moves.  A member of no length strains along no direction
    /// and is left out.  Where there are beams the nodes turn, so a twist that moves no node's
    /// place still turns them: the body is not the nodes' places alone.
    pub fn free(&self, x: &[f64]) -> Free {
        let (per, dim) = (self.per(), self.dim);
        let nodes: Vec<[f64; 3]> = (0..self.nodes).map(|i| self.node(i, x)).collect();
        let holds: Vec<Held> = (0..self.fixed.len()).filter(|&i| self.fixed[i]).map(|i| {
            match self.component(i) {
                (node, c) if c < 3 => Held { at: vec![nodes[node]], along: Some(analysis::axis(c)),
                                             turn: false },
                _ => Held { at: Vec::new(), along: None, turn: true },
            }
        }).collect();
        let body: &[[f64; 3]] = if self.beams.is_empty() { &nodes } else { &[] };
        let rigid = analysis::freedoms(dim, body, &holds);
        let (eq, n) = self.equations();
        if n > MECHANISM_MAX {
            return Free { rigid, mechanism: None };
        }
        // the rows `solve` factors, unweighted, one after another
        let mut rows: Vec<f64> = Vec::new();
        for b in &self.bars {
            let d = self.place(b.a, b.b, x);
            let l = norm(d);
            if !(l > 0.0) {
                continue;
            }
            let at = rows.len();
            rows.resize(at + n, 0.0);
            self.strain(b, scale(d, 1.0 / l), &eq, |e, v| rows[at + e] += v);
        }
        for i in 0..self.beams.len() {
            let Ok(groups) = self.beam_rows(i, x) else { continue };
            for r in groups.iter().flatten() {
                let at = rows.len();
                rows.resize(at + n, 0.0);
                self.beam_put(i, r, |dof, v| {
                    if let Some(e) = eq[dof] {
                        rows[at + e] += v.v;
                    }
                });
            }
        }
        // each rigid motion's field, so what is left is orthogonal to it: its nodes' moves and,
        // where they turn, their turns
        for f in &rigid {
            let at = rows.len();
            rows.resize(at + n, 0.0);
            let spin = match *f {
                Freedom::Slide { .. } => [0.0; 3],
                Freedom::Turn { about, .. } => about,
            };
            for (node, p) in nodes.iter().enumerate() {
                let v = f.velocity(*p);
                for c in 0..6 {
                    let w = if c < 3 { v[c] } else { spin[c - 3] };
                    if let Some(e) = self.dof(node, c).and_then(|d| eq[d]) {
                        rows[at + e] = w;
                    }
                }
            }
        }
        let null = rank_and_nullspace(&Mat::from_vec(rows.len() / n.max(1), n, rows),
                                      crate::system::RANK_TOL).null();
        let mut moved = vec![0.0f64; self.nodes];
        for c in 0..null.cols {
            for (i, e) in eq.iter().enumerate() {
                if let Some(e) = e {
                    moved[i / per] = moved[i / per].max(null.at(*e, c).abs());
                }
            }
        }
        let top = moved.iter().copied().fold(0.0, f64::max);
        let mechanism = (0..self.nodes).filter(|&i| top > 0.0 && moved[i] > 1e-6 * top).collect();
        Free { rigid, mechanism: Some(mechanism) }
    }

    /// **Whether a requirement on the output is out of reach** (#510 P7) at `x`, stating `target`,
    /// of the columns `free` says drive it (`analysis::reach`).  A node's coordinate is moved by
    /// the structure's extent, an area or a section's number by itself.
    pub fn reach(&self, x: &[f64], target: f64, free: &[bool]) -> Result<Reach, Failure> {
        let st = self.solve(x)?;
        let mut g = vec![0.0; self.columns()];
        self.gradient(&st, x, &mut g);
        let at = self.dim * self.nodes;
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for i in 0..self.nodes {
            let p = self.node(i, x);
            for k in 0..self.dim {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        let extent =
            (0..self.dim).map(|k| hi[k] - lo[k]).fold(0.0, f64::max).max(f64::MIN_POSITIVE);
        let scale: Vec<f64> = (0..self.columns()).map(|i| if i < at { extent } else { x[i].abs() })
            .collect();
        Ok(analysis::reach(self.output(&st), target, &g, &scale, free))
    }
}

impl Bar {
    /// The gradient of the mutual energy `aᵀkₑb = E·A·(Δ·α)(Δ·β)/L³` (with `α`, `β` the ends'
    /// relative displacements under `a` and `b`) in `Δ = x_b − x_a`, and in the area.
    fn mutual(&self, d: [f64; 3], l: f64, ea: f64, al: [f64; 3], be: [f64; 3]) -> ([f64; 3], f64) {
        let (da, db) = (dot(d, al), dot(d, be));
        let l3 = l * l * l;
        let m = ea * da * db / l3;
        let dm = [0, 1, 2].map(|k| ea * (al[k] * db + be[k] * da) / l3 - 3.0 * m * d[k] / (l * l));
        (dm, self.e * da * db / l3)
    }
}

/* -- the row's constants ------------------------------------------------------------------ */

/// The constants before the structure: `[dim, nodes, bars, beams]`, then the output stated,
/// `J₀`, or where an unknown sets it, its `(m, c)`.
pub const HEAD: usize = 4;

/// A beam's constants: its ends, its moduli, its density and its section's z.
const BEAM: usize = 8;

impl Structure {
    /// A row's constants: the head, every bar's `(a, b, E)`, every beam's `(a, b, E, G, ρ, z)`,
    /// then every degree of freedom's `(held, load, weight)`.  `stated` is `[J₀]`, or `[m, c]`
    /// for an output an unknown sets.
    pub fn consts(&self, stated: &[f64]) -> Vec<f64> {
        let mut k = Vec::with_capacity(self.widths(stated.len() == 2).1);
        k.extend([self.dim, self.nodes, self.bars.len(), self.beams.len()].map(|n| n as f64));
        k.extend(stated);
        for b in &self.bars {
            k.extend([b.a as f64, b.b as f64, b.e]);
        }
        for b in &self.beams {
            k.extend([b.a as f64, b.b as f64, b.e, b.g, b.rho]);
            k.extend(b.z);
        }
        for i in 0..self.fixed.len() {
            k.extend([self.fixed[i] as u8 as f64, self.load[i], self.weight[i]]);
        }
        k
    }

    /// What `consts` wrote, `free` where an unknown sets the output (the row has its column, as
    /// `integral::length_widths` reads a length's): the structure, and what it states.
    pub fn read(k: &[f64], free: bool) -> (Structure, &[f64]) {
        let (dim, nodes) = (k[0] as usize, k[1] as usize);
        let (n_bars, n_beams) = (k[2] as usize, k[3] as usize);
        let (stated, rest) = k[HEAD..].split_at(if free { 2 } else { 1 });
        let (bars, rest) = rest.split_at(3 * n_bars);
        let (beams, per) = rest.split_at(BEAM * n_beams);
        let per = per.as_chunks::<3>().0;
        let s = Structure {
            dim,
            nodes,
            bars: bars.as_chunks::<3>().0.iter()
                .map(|c| Bar { a: c[0] as usize, b: c[1] as usize, e: c[2] })
                .collect(),
            beams: beams.as_chunks::<BEAM>().0.iter()
                .map(|c| Beam { a: c[0] as usize, b: c[1] as usize, e: c[2], g: c[3], rho: c[4],
                                z: [c[5], c[6], c[7]] })
                .collect(),
            fixed: per.iter().map(|c| c[0] != 0.0).collect(),
            load: per.iter().map(|c| c[1]).collect(),
            weight: per.iter().map(|c| c[2]).collect(),
        };
        (s, stated)
    }

    /// A row's widths, `(n_par, n_const)`: its columns, and the unknown's where it has one.
    pub fn widths(&self, free: bool) -> (usize, usize) {
        let stated = if free { 2 } else { 1 };
        (self.columns() + free as usize,
         HEAD + stated + 3 * self.bars.len() + BEAM * self.beams.len() + 3 * self.fixed.len())
    }
}

/* -- the kernel --------------------------------------------------------------------------- */

/// The kernel at a block's widths (`system::build_kernel`, `KernelKey::Interior`): one row, the
/// output less what it states, `J(x) − J₀` or `J(x) − (m·a + c)`.  A displacement, so degree 1.
/// A structure with no displacement to give (`Failure`) reads NaN, which the step rejects.
pub fn kernel(n_par: usize, n_const: usize) -> Kernel {
    Kernel { name: "interior", n_res: 1, n_par, n_const, degree: 1, res, jac, const_jac: None,
             form: None }
}

/// Whether a row carries the unknown's column: one more than the columns its head says its
/// structure reads.
fn free_at(n_par: usize, k: &[f64]) -> bool {
    let (dim, nodes, bars, beams) = (k[0] as usize, k[1] as usize, k[2] as usize, k[3] as usize);
    n_par > dim * nodes + bars + SECTION * beams
}

fn res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    for i in 0..n {
        let (v, k) = (&v[n_par * i..n_par * (i + 1)], &k[n_const * i..n_const * (i + 1)]);
        let free = free_at(n_par, k);
        let (s, stated) = Structure::read(k, free);
        let target = if free { stated[0] * v[s.columns()] + stated[1] } else { stated[0] };
        r[i] = match s.solve(v) {
            Ok(st) => s.output(&st) - target,
            Err(_) => f64::NAN,
        };
    }
}

fn jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    for i in 0..n {
        let (v, k) = (&v[n_par * i..n_par * (i + 1)], &k[n_const * i..n_const * (i + 1)]);
        let row = &mut j[n_par * i..n_par * (i + 1)];
        row.fill(0.0);
        let free = free_at(n_par, k);
        let (s, stated) = Structure::read(k, free);
        match s.solve(v) {
            Ok(st) => s.gradient(&st, v, row),
            Err(_) => row.fill(f64::NAN),
        }
        if free {
            row[s.columns()] = -stated[0];
        }
    }
}
