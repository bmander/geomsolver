//! A row whose value is the output of a linear solve (#510 P5): an analysis's interior.
//!
//! A requirement on a structure — a tip moves no more than half a millimetre — is a row whose
//! residual is an output `J(u)` of the displacements `u` that solve `K(x)u = f`, where the
//! stiffness `K` is assembled from the drawing's own numbers `x` (where the nodes stand, how
//! big each member is).  The row is a **kernel**: `u` is no unknown of the drawing — an analysis
//! has thousands of displacements, a solve that is linear, and nothing to show of them in the
//! drawing — and a kernel is a row like every other: the flat solve, the decomposition, the
//! diagnosis, a claim and a free twin take it as they take a spline's length (`integral.rs`),
//! which reads every control point of its spline as this reads every node and member.
//!
//! The value is one factorisation of `K` and one solve; the gradient is **the adjoint**, one more
//! solve on the factor already in hand, `Kᵀλ = ∂J/∂u`, then `dJ/dx = ∂J/∂x − λᵀ(∂K/∂x·u − ∂f/∂x)`
//! — whatever number of columns the row reads, where the implicit function theorem through a
//! square Newton (`newton::square`, a locus's) answers one column at a time.  `∂K/∂x·u` is never formed: `λᵀK(x)u` is a sum of
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
//! mechanism as the bar does.  The quadratic triangle (#510 S2, `plane::Tri6`), in plane stress
//! or plane strain, is the third: nine strain rows over its six nodes' moves.  The quadratic
//! tetrahedron (#510 S3, `solid::Tet10`) is the fourth: twenty-four strain rows over its ten
//! nodes' moves in space.

use crate::diagnose::analysis::{self, Freedom, Held, Reach};
use crate::kernels::{curve_widths, Dual, Kernel, Num};
use crate::linalg::{rank_and_nullspace, Mat};
use crate::envelope::held;
use crate::space::{add, dot, norm, scale, sub};
use crate::delaunay::tetra::EDGES;
use crate::pcg::Lower;
use crate::sparse::{Ata, Sym};

mod moved;
mod plane;
mod solid;
pub use moved::{Edge, Moved, Node, Stack};
pub use plane::{Laid, Tri6};
pub use solid::{Borne, Tet10};

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

/// A structure of bars and beams, of plane elements (#510 S2), or of solid elements (#510 S3):
/// `nodes` points of `dim` (2 or
/// 3) coordinates each, and per degree of freedom (`per · node + k`, `Structure::per`) whether a
/// support holds it, the load on it, and its weight in the output `J = Σ weight · u`.  A node's
/// degrees of freedom are its moves along the axes, then where there is a beam its turns: about z
/// in the plane, about x, y and z in space.  The weights are numbers, a direction's cosines, so the output is a
/// displacement: a length.
///
/// Its columns are every node's coordinates, then every bar's area, then every beam's section
/// (`Structure::columns`); the moduli, the densities, the supports, the loads and the weights are
/// constants.  A load spread along beams (`spread`, #510 S1) is a force of their geometry and
/// their sections, so it moves with the columns: `f(x)`.  So does a load laid on plane elements
/// (`laid`, #510 S2), of their places, and a load borne by solid elements (`borne`, #510 S3).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Structure {
    pub dim: usize,
    pub nodes: usize,
    pub bars: Vec<Bar>,
    pub beams: Vec<Beam>,
    pub fixed: Vec<bool>,
    pub load: Vec<f64>,
    pub weight: Vec<f64>,
    pub spread: Vec<Spread>,
    /// The plane elements (#510 S2), in a structure of `dim` 2.
    pub tris: Vec<Tri6>,
    pub laid: Vec<Laid>,
    /// The solid elements (#510 S3), in a structure of `dim` 3.
    pub tets: Vec<Tet10>,
    pub borne: Vec<Borne>,
}

/// **A load spread along beams** (#510 S1), each carrying it as a force a length, `q`, in the
/// world's axes: a force spread evenly along them, `q = force / Σ L` (`per_area` false: `force`
/// the whole of it, so it is the same force however long they grow), or a body load, `q = force
/// · A` (`per_area`: `force` a density times an acceleration, so it grows with the section).  Each
/// beam's share is its consistent load at its ends (`Structure::forces`), exact for the cubic.
#[derive(Clone, Debug, PartialEq)]
pub struct Spread {
    pub beams: Vec<usize>,
    pub force: [f64; 3],
    pub per_area: bool,
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

/// The factor a structure's displacements were solved on: the normal equations of its strain
/// rows (`sparse::Ata`, reverse Cuthill–McKee) for bars and beams, or a plane or a solid
/// structure's stiffness assembled and factored by supernodal Cholesky under minimum degree
/// (`cholesky::Cholesky`, #510 S2, S3) — a mesh's thousands of unknowns, which a band order
/// fills.
enum Factor {
    Ata(Ata),
    Cholesky(crate::cholesky::Cholesky),
    /// A solid's stiffness held for CG over its corners' space (`pcg`, `ITERATIVE`), and how
    /// many iterations its solve took.
    Pcg(crate::pcg::Pcg, usize),
}

impl Factor {
    /// `b` overwritten by `K⁻¹ b` on the factor in hand (by CG, to `pcg::TOLERANCE`).
    fn resolve(&self, b: &mut [f64]) {
        match self {
            Factor::Ata(a) => a.resolve(b),
            Factor::Cholesky(c) => c.solve(b),
            Factor::Pcg(cg, _) => {
                let _ = cg.solve(b);
            }
        }
    }
}

/// **The fewest equations a solid structure is solved by CG at** (`pcg`), not factored: in three
/// dimensions a factor fills as `n^{4/3}` and costs `n²`, and at this size CG over the corners'
/// space is already the faster.  A plane structure, whose factor fills as `n log n`, is always
/// factored.
pub const ITERATIVE: usize = 50_000;

/// The displacements of a structure at `x`, with the factor they were solved on held for the
/// adjoint.
pub struct State {
    ata: Factor,
    /// Per degree of freedom, its equation, or `None` where a support holds it.
    eq: Vec<Option<usize>>,
    /// Per degree of freedom, zero where held.
    pub u: Vec<f64>,
}

impl State {
    /// Whether it was solved by CG (`pcg`), not on a factor.
    pub fn by_cg(&self) -> bool {
        self.iterations().is_some()
    }

    /// How many iterations CG took to solve it, where it was solved by CG.
    pub fn iterations(&self) -> Option<usize> {
        match self.ata {
            Factor::Pcg(_, its) => Some(its),
            _ => None,
        }
    }

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

    /// Assemble `K(x)` as strain rows, factor it and solve `K u = f`.  A plane or a solid
    /// structure's stiffness is assembled whole and factored under minimum degree (#510 S2, S3):
    /// a pivot lost to rounding is a motion the supports leave free.  A solid of `ITERATIVE`
    /// equations or more is solved by CG over its corners' space instead (`pcg`).
    pub fn solve(&self, x: &[f64]) -> Result<State, Failure> {
        self.solve_by(x, self.equations().1 >= ITERATIVE)
    }

    /// `solve`, by CG where `iterative` and the structure is a solid's (`pcg`), else factored —
    /// and factored after all where CG cannot be had (`State::by_cg` says which).
    pub fn solve_by(&self, x: &[f64], iterative: bool) -> Result<State, Failure> {
        let (eq, n_eq) = self.equations();
        let mut rhs: Vec<f64> = eq.iter().zip(self.forces(x)).filter(|(e, _)| e.is_some())
            .map(|(_, v)| v).collect();
        let factor = if self.meshed() {
            // a solid too large to factor is solved by CG over its corners' space (`pcg`); one
            // whose corners' factor fails, or that CG does not settle, is factored after all, on
            // the stiffness already assembled — and its factor says whether the supports leave
            // it free
            let k = if self.tris.is_empty() && iterative {
                let (k, up, coarse) = self.cornered(x, &eq, n_eq)?;
                match crate::pcg::Pcg::new(k, up, &coarse) {
                    Ok(cg) => {
                        let mut u = rhs.clone();
                        match cg.solve(&mut u) {
                            Some(its) if u.iter().all(|v| v.is_finite()) => {
                                let mut st = State { ata: Factor::Pcg(cg, its), eq, u: Vec::new() };
                                st.u = st.scatter(&u);
                                return Ok(st);
                            }
                            _ => cg.into_lower(),
                        }
                    }
                    Err(k) => k,
                }
            } else {
                self.assembled(x, &eq, n_eq)?
            };
            // the factor reads the lower triangle alone
            let k = k.symmetric();
            let mut c = crate::cholesky::Cholesky::analyse(&k, crate::cholesky::Ordering::Amd);
            c.factor(&k).map_err(|_| Failure::Free)?;
            c.solve(&mut rhs);
            if rhs.iter().any(|v| !v.is_finite()) {
                return Err(Failure::Free);
            }
            Factor::Cholesky(c)
        } else {
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
            if !ata.solve(&vec![0.0; n_eq], &mut rhs) || ata.least_pivot() < SINGULAR {
                return Err(Failure::Free);
            }
            Factor::Ata(ata)
        };
        let mut st = State { ata: factor, eq, u: Vec::new() };
        st.u = st.scatter(&rhs);
        Ok(st)
    }

    /// **A plane or a solid structure's stiffness**, its lower triangle over the equations `eq`
    /// numbers, assembled in place (`pcg::Lower`): its pattern from the elements' nodes, then
    /// each element's matrix added at its entries.
    fn assembled(&self, x: &[f64], eq: &[Option<usize>], n_eq: usize) -> Result<Lower, Failure> {
        let elements =
            self.tris.iter().map(|t| &t.n[..]).chain(self.tets.iter().map(|t| &t.n[..]));
        let mut k = Lower::pattern(self.nodes, self.per(), elements, eq, n_eq);
        for i in 0..self.tris.len() {
            let m = self.tris[i].stiffness(&self.tri_places(i, x))?;
            k.add(&m, &self.tri_dofs(i).map(|d| eq[d]));
        }
        for i in 0..self.tets.len() {
            let m = self.tets[i].stiffness(&self.tet_places(i, x))?;
            k.add(&m, &self.tet_dofs(i).map(|d| eq[d]));
        }
        Ok(k)
    }

    /// **A solid's stiffness and its corners' space** (`pcg`): `K` over the equations `eq`
    /// numbers (its lower triangle, `assembled`'s), per equation its row of `P` — a corner's move
    /// its own, a side's middle node's the mean of the side's ends', a support's held moves left
    /// out — and the Galerkin matrix `PᵀKP` over the corners' equations, assembled element by
    /// element as `P_eᵀ K_e P_e` (a side's ends are its own element's corners).
    fn cornered(&self, x: &[f64], eq: &[Option<usize>], n_eq: usize)
        -> Result<(Lower, Vec<[(u32, f64); 2]>, crate::cholesky::Symmetric), Failure>
    {
        // the corners' equations, by degree of freedom
        let mut corner = vec![false; self.nodes];
        for t in &self.tets {
            for &c in &t.n[..4] {
                corner[c] = true;
            }
        }
        let mut ceq: Vec<Option<usize>> = vec![None; eq.len()];
        let mut nc = 0;
        for (d, e) in eq.iter().enumerate() {
            if e.is_some() && corner[d / 3] {
                ceq[d] = Some(nc);
                nc += 1;
            }
        }
        // both patterns from the mesh, then every element added in place
        let mut k = Lower::pattern(self.nodes, 3, self.tets.iter().map(|t| &t.n[..]), eq, n_eq);
        let mut kc = Lower::pattern(self.nodes, 3, self.tets.iter().map(|t| &t.n[..4]), &ceq, nc);
        let mut ends: Vec<Option<(usize, usize)>> = vec![None; self.nodes];
        for i in 0..self.tets.len() {
            let m = self.tets[i].stiffness(&self.tet_places(i, x))?;
            let dofs = self.tet_dofs(i);
            k.add(&m, &dofs.map(|d| eq[d]));
            let pmp = cornered::<30, 12>(&m, &dofs, 3, &EDGES, eq);
            kc.add(&pmp, &std::array::from_fn(|c| ceq[dofs[c]]));
            let n = self.tets[i].n;
            for (k, &(a, b)) in EDGES.iter().enumerate() {
                ends[n[4 + k]] = Some((n[a], n[b]));
            }
        }
        let mut up = vec![[(0u32, 0.0); 2]; n_eq];
        for (d, e) in eq.iter().enumerate() {
            let Some(e) = *e else { continue };
            let (node, c) = (d / 3, d % 3);
            match (ceq[d], ends[node]) {
                (Some(j), _) => up[e][0] = (j as u32, 1.0),
                (None, Some((a, b))) => {
                    let from = [a, b].into_iter().filter_map(|v| ceq[3 * v + c]);
                    for (t, j) in from.enumerate() {
                        up[e][t] = (j as u32, 0.5);
                    }
                }
                (None, None) => {}
            }
        }
        Ok((k, up, kc.symmetric()))
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
        self.tris_into(&eq, false, &mut t, |i| self.tris[i].stiffness(&self.tri_places(i, x)))?;
        self.tets_into(&eq, false, &mut t, |i| self.tets[i].stiffness(&self.tet_places(i, x)))?;
        Ok(Sym::from_triplets(n_eq, &t))
    }

    /// Whether it is a mesh of plane or solid elements, assembled whole and factored under
    /// minimum degree.
    fn meshed(&self) -> bool {
        !self.tris.is_empty() || !self.tets.is_empty()
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
    /// element's mutual energy differentiated, and the loads spread along beams.  The output
    /// reads only `u`, so `∂J/∂x` is zero and what is left is `λᵀ(∂f/∂x − (∂K/∂x)u)`: a load at a
    /// node does not move with `x`, and one spread along a beam does (`Structure::spread_into`).
    pub fn gradient(&self, st: &State, x: &[f64], g: &mut [f64]) {
        let mut rhs = st.gather(&self.weight);
        st.ata.resolve(&mut rhs);
        let lam = st.scatter(&rhs);
        self.spread_into(&lam, x, g);
        self.laid_into(&lam, x, g);
        self.borne_into(&lam, x, g);
        // each element's mutual energy over its coordinates, its nodes' places
        for i in 0..self.tets.len() {
            let Ok(rows) = self.tets[i].rows(&self.tet_places_d::<30>(i, x)) else { continue };
            self.mutual_into(&rows, &self.tet_dofs(i), &self.tets[i].n, &lam, &st.u, g);
        }
        for i in 0..self.tris.len() {
            let Ok(rows) = self.tris[i].rows(&self.tri_places_d::<12>(i, x)) else { continue };
            self.mutual_into(&rows, &self.tri_dofs(i), &self.tris[i].n, &lam, &st.u, g);
        }
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
    fn spread_into(&self, lam: &[f64], x: &[f64], g: &mut [f64]) {
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
        self.tris_into(&eq, false, &mut t, |i| self.tris[i].mass(&self.tri_places(i, x)))?;
        self.tets_into(&eq, false, &mut t, |i| self.tets[i].mass(&self.tet_places(i, x)))?;
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

/* -- the plane element (#510 S2) ------------------------------------------------------------- */

impl Structure {
    /// Plane element `i`'s twelve degrees of freedom: each node's move along x, then along y.
    fn tri_dofs(&self, i: usize) -> [usize; 12] {
        let n = self.tris[i].n;
        std::array::from_fn(|k| self.dof(n[k / 2], k % 2).unwrap())
    }

    /// Plane element `i`'s nodes' places at `x`.
    pub fn tri_places(&self, i: usize, x: &[f64]) -> [[f64; 2]; 6] {
        self.tris[i].n.map(|n| [x[2 * n], x[2 * n + 1]])
    }

    /// `tri_places`, each coordinate carrying its rate among the element's twelve (`N` = 12), or
    /// none (`N` = 0).
    fn tri_places_d<const N: usize>(&self, i: usize, x: &[f64]) -> [plane::P<Dual<N>>; 6] {
        let p = self.tri_places(i, x);
        std::array::from_fn(|k| if N == 0 {
            p[k].map(Dual::cst)
        } else {
            plane::var(p[k], 2 * k)
        })
    }

    /// Every plane element's twelve-by-twelve matrix, `local(i)`, added at the equations `eq`
    /// numbers, as triplets (`local_into`).
    fn tris_into(&self, eq: &[Option<usize>], lower: bool, t: &mut Vec<(usize, usize, f64)>,
                 local: impl Fn(usize) -> Result<[[f64; 12]; 12], Failure>) -> Result<(), Failure>
    {
        for i in 0..self.tris.len() {
            local_into(&local(i)?, &self.tri_dofs(i), eq, lower, t);
        }
        Ok(())
    }

    /// Node `n`'s place in the plane at `x`.
    fn place2(&self, n: usize, x: &[f64]) -> [f64; 2] {
        [x[2 * n], x[2 * n + 1]]
    }

    /// **Each side's and element's share of a load laid on the plane** (#510 S2), at `x`, over a
    /// number that may carry rates (`N` = 12 its nodes' coordinates, a side's three the first six;
    /// 0 none): its nodes, their forces, and the measure an even load is spread by.
    fn laid_shares<const N: usize>(&self, l: &Laid, x: &[f64])
        -> Vec<Share<N, 2>>
    {
        let place = |n: usize, k: usize| -> plane::P<Dual<N>> {
            let p = self.place2(n, x);
            if N == 0 { p.map(Dual::cst) } else { plane::var(p, 2 * k) }
        };
        let mut out = Vec::new();
        for side in l.sides() {
            let (f, m) = plane::side_load(l, &std::array::from_fn(|k| place(side[k], k)));
            out.push((side.to_vec(), f.to_vec(), m));
        }
        for &e in l.elements() {
            let t = &self.tris[e];
            let (f, m) =
                plane::element_load(l, &std::array::from_fn(|k| place(t.n[k], k)), t.depth);
            out.push((t.n.to_vec(), f.to_vec(), m));
        }
        out
    }

    /// The loads laid on the plane at `x`, added into `f` per degree of freedom: each side's or
    /// element's consistent load at its nodes, an even load's over the measure it is spread by.
    fn laid_forces(&self, x: &[f64], f: &mut [f64]) {
        for l in &self.laid {
            self.shares_forces(self.laid_shares::<0>(l, x), l.even(), f);
        }
    }

    /// **`λᵀ ∂f/∂x` of the loads laid on the plane, added into `g`** (#510 S2): each side's or
    /// element's `λᵀfₑ` formed in `Dual` over its nodes' places; an even load's whole is `Σ λᵀfₑ /
    /// S`, `S` the length or the area it is spread over, so each place also reads `−(Σ λᵀfₑ)/S² ·
    /// ∂S`.
    fn laid_into(&self, lam: &[f64], x: &[f64], g: &mut [f64]) {
        for l in &self.laid {
            self.shares_into(self.laid_shares::<12>(l, x), l.even(), lam, g);
        }
    }

    /// One load's `shares` (each its nodes, their forces of `C` components and its measure),
    /// added into `f` per degree of freedom — over the whole measure where it is `even`: a load
    /// laid on the plane's, or borne by a solid's.
    fn shares_forces<const C: usize>(&self, shares: Vec<Share<0, C>>, even: bool, f: &mut [f64]) {
        let by = if even { shares.iter().map(|s| s.2.v).sum() } else { 1.0 };
        for (nodes, at, _) in shares {
            for (n, v) in nodes.into_iter().zip(at) {
                for c in 0..C {
                    f[self.dof(n, c).unwrap()] += v[c].v / by;
                }
            }
        }
    }

    /// **`λᵀ ∂f/∂x` of one load's `shares`, added into `g`**: each share's `λᵀfₑ` formed in `Dual`
    /// over its nodes' places; an even load's whole is `Σ λᵀfₑ / S`, `S` the measure it is spread
    /// over, so each place also reads `−(Σ λᵀfₑ)/S² · ∂S`.
    fn shares_into<const N: usize, const C: usize>(&self, shares: Vec<Share<N, C>>, even: bool,
                                                    lam: &[f64], g: &mut [f64])
    {
        let mut parts: Vec<(Vec<usize>, [f64; N], [f64; N])> = Vec::new();
        let (mut whole, mut measure) = (0.0, 0.0);
        for (nodes, f, m) in shares {
            // λᵀfₑ, its rates in the element's coordinates
            let mut w = Dual::<N>::cst(0.0);
            for (&n, fk) in nodes.iter().zip(&f) {
                for c in 0..C {
                    w = w + fk[c] * Dual::cst(lam[self.dof(n, c).unwrap()]);
                }
            }
            whole += w.v;
            measure += m.v;
            parts.push((nodes, w.g, m.g));
        }
        let (by, less) = if even { (measure, whole / (measure * measure)) } else { (1.0, 0.0) };
        for (nodes, dw, dm) in parts {
            for (k, &n) in nodes.iter().enumerate() {
                for c in 0..C {
                    g[C * n + c] += dw[C * k + c] / by - less * dm[C * k + c];
                }
            }
        }
    }

    /// An element's mutual energy `Σ (r·λ)(r·u)` over its strain rows `rows` of its degrees of
    /// freedom `dofs`, formed over its nodes' places (`N` its coordinates, `N / K` each of its
    /// `K` nodes'), and taken from `g` at its nodes `nodes`.
    fn mutual_into<const N: usize, const K: usize>(&self, rows: &[[Dual<N>; N]],
                                                    dofs: &[usize; N], nodes: &[usize; K],
                                                    lam: &[f64], u: &[f64], g: &mut [f64])
    {
        let d = N / K;
        let mut m = Dual::<N>::cst(0.0);
        for r in rows {
            let (mut rl, mut ru) = (Dual::cst(0.0), Dual::cst(0.0));
            for k in 0..N {
                rl = rl + r[k] * Dual::cst(lam[dofs[k]]);
                ru = ru + r[k] * Dual::cst(u[dofs[k]]);
            }
            m = m + rl * ru;
        }
        for (k, &n) in nodes.iter().enumerate() {
            for c in 0..d {
                g[d * n + c] -= m.g[d * k + c];
            }
        }
    }
}

/* -- the solid element (#510 S3) ------------------------------------------------------------- */

impl Structure {
    /// Solid element `i`'s thirty degrees of freedom: each node's move along x, y, then z.
    fn tet_dofs(&self, i: usize) -> [usize; 30] {
        let n = self.tets[i].n;
        std::array::from_fn(|k| 3 * n[k / 3] + k % 3)
    }

    /// Solid element `i`'s nodes' places at `x`.
    pub fn tet_places(&self, i: usize, x: &[f64]) -> [[f64; 3]; 10] {
        self.tets[i].n.map(|n| [x[3 * n], x[3 * n + 1], x[3 * n + 2]])
    }

    /// `tet_places`, each coordinate carrying its rate among the element's thirty (`N` = 30), or
    /// none (`N` = 0).
    fn tet_places_d<const N: usize>(&self, i: usize, x: &[f64]) -> [solid::P<Dual<N>>; 10] {
        let p = self.tet_places(i, x);
        std::array::from_fn(|k| if N == 0 {
            p[k].map(Dual::cst)
        } else {
            solid::var(p[k], 3 * k)
        })
    }

    /// Every solid element's thirty-by-thirty matrix, `local(i)`, added at the equations `eq`
    /// numbers, as triplets (`local_into`).
    fn tets_into(&self, eq: &[Option<usize>], lower: bool, t: &mut Vec<(usize, usize, f64)>,
                 local: impl Fn(usize) -> Result<[[f64; 30]; 30], Failure>) -> Result<(), Failure>
    {
        for i in 0..self.tets.len() {
            local_into(&local(i)?, &self.tet_dofs(i), eq, lower, t);
        }
        Ok(())
    }

    /// **The geometric stiffness of the solid elements** (#510 S3) under the stress the moves `u`
    /// (per degree of freedom) make, over the equations no support holds: each element's `∫ ∇Nᵀ
    /// σ ∇N dV` — what `eigen::buckling` reads beside `stiffness`.
    pub fn stressed(&self, x: &[f64], u: &[f64]) -> Result<Sym, Failure> {
        let (eq, n_eq) = self.equations();
        let mut t = Vec::new();
        self.tets_into(&eq, false, &mut t, |i| {
            let ue: [f64; 30] = self.tet_dofs(i).map(|d| u[d]);
            self.tets[i].geometric(&self.tet_places(i, x), &ue).ok_or(Failure::Collapsed)
        })?;
        Ok(Sym::from_triplets(n_eq, &t))
    }

    /// The solid elements' volume at `x`.
    pub fn volume(&self, x: &[f64]) -> f64 {
        (0..self.tets.len()).map(|i| self.tets[i].volume(&self.tet_places(i, x))).sum()
    }

    /// **Each facet's and element's share of a load borne by the solid** (#510 S3), at `x`, over
    /// a number that may carry rates (`N` = 30 its nodes' coordinates, a facet's six the first
    /// eighteen; 0 none): its nodes, their forces, and the measure an even load is spread by.
    fn borne_shares<const N: usize>(&self, l: &Borne, x: &[f64])
        -> Vec<Share<N, 3>>
    {
        let place = |n: usize, k: usize| -> solid::P<Dual<N>> {
            let p = [x[3 * n], x[3 * n + 1], x[3 * n + 2]];
            if N == 0 { p.map(Dual::cst) } else { solid::var(p, 3 * k) }
        };
        let mut out = Vec::new();
        for facet in l.facets() {
            let (f, m) = solid::facet_load(l, &std::array::from_fn(|k| place(facet[k], k)));
            out.push((facet.to_vec(), f.to_vec(), m));
        }
        for &e in l.elements() {
            let t = &self.tets[e];
            let (f, m) = solid::element_load(l, &std::array::from_fn(|k| place(t.n[k], k)));
            out.push((t.n.to_vec(), f.to_vec(), m));
        }
        out
    }

    /// The loads borne by the solid at `x`, added into `f` per degree of freedom: each facet's or
    /// element's consistent load at its nodes, an even load's over the measure it is spread by.
    fn borne_forces(&self, x: &[f64], f: &mut [f64]) {
        for l in &self.borne {
            self.shares_forces(self.borne_shares::<0>(l, x), l.even(), f);
        }
    }

    /// **`λᵀ ∂f/∂x` of the loads borne by the solid, added into `g`** (#510 S3), as the plane's
    /// are (`laid_into`): each share's `λᵀfₑ` in `Dual` over its nodes' places, an even load's
    /// whole over its measure `S` also reading `−(Σ λᵀfₑ)/S² · ∂S`.
    fn borne_into(&self, lam: &[f64], x: &[f64], g: &mut [f64]) {
        for l in &self.borne {
            self.shares_into(self.borne_shares::<30>(l, x), l.even(), lam, g);
        }
    }
}

/// One share of a load laid on the plane or borne by a solid: its nodes, each node's force of `C`
/// components, and the measure it lies over, over a number carrying `N` rates.
type Share<const N: usize, const C: usize> = (Vec<usize>, Vec<[Dual<N>; C]>, Dual<N>);

/// An element's `M`-by-`M` matrix `m` over its degrees of freedom `dofs`, added at the equations
/// `eq` numbers as triplets — its lower triangle alone where `lower`, which is all a Cholesky
/// reads.
fn local_into<const M: usize>(m: &[[f64; M]; M], dofs: &[usize; M], eq: &[Option<usize>],
                              lower: bool, t: &mut Vec<(usize, usize, f64)>)
{
    for a in 0..M {
        let Some(ea) = eq[dofs[a]] else { continue };
        for b in 0..M {
            if let (Some(eb), true) = (eq[dofs[b]], m[a][b] != 0.0) {
                if !lower || ea >= eb {
                    t.push((ea, eb, m[a][b]));
                }
            }
        }
    }
}

/// **An element's share of its mesh's corners' Galerkin matrix** (`pcg`): `P_eᵀ m P_e` over its
/// `C` corners' degrees of freedom (its own first `C`), `P_e` carrying each corner's move to
/// itself and each side's middle node's (`sides`, after the corners) from the side's ends', its
/// held moves (no `eq`) left out.
fn cornered<const M: usize, const C: usize>(m: &[[f64; M]; M], dofs: &[usize; M], dim: usize,
                                            sides: &[(usize, usize)], eq: &[Option<usize>])
    -> [[f64; C]; C]
{
    let corners = C / dim;
    // P_e's rows: each local degree of freedom's corners' ones and weights
    let from = |a: usize| -> ([(usize, f64); 2], usize) {
        let (node, k) = (a / dim, a % dim);
        if eq[dofs[a]].is_none() {
            ([(0, 0.0); 2], 0)
        } else if node < corners {
            ([(a, 1.0), (0, 0.0)], 1)
        } else {
            let (i, j) = sides[node - corners];
            ([(i * dim + k, 0.5), (j * dim + k, 0.5)], 2)
        }
    };
    let rows: [([(usize, f64); 2], usize); M] = std::array::from_fn(from);
    // m P_e, then P_eᵀ (m P_e)
    let mut mp = [[0.0; C]; M];
    for a in 0..M {
        for b in 0..M {
            let v = m[a][b];
            if v == 0.0 {
                continue;
            }
            let (r, n) = rows[b];
            for &(c, w) in &r[..n] {
                mp[a][c] += v * w;
            }
        }
    }
    let mut pmp = [[0.0; C]; C];
    for a in 0..M {
        let (r, n) = rows[a];
        for &(c, w) in &r[..n] {
            for d in 0..C {
                pmp[c][d] += w * mp[a][d];
            }
        }
    }
    pmp
}

/// An element's strain rows `rows` over its degrees of freedom `dofs`, each over `unit` and laid
/// out over the `n` equations `eq` numbers, appended to `out`: the mechanism's rows (`free`).
fn unit_rows<const N: usize>(rows: &[[Dual<0>; N]], dofs: &[usize; N], unit: f64,
                             eq: &[Option<usize>], n: usize, out: &mut Vec<f64>)
{
    for r in rows {
        let at = out.len();
        out.resize(at + n, 0.0);
        for k in 0..N {
            if let Some(e) = eq[dofs[k]] {
                out[at + e] += r[k].v / unit;
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
        for i in 0..self.tris.len() {
            let t = &self.tris[i];
            let Ok(groups) = t.rows(&self.tri_places_d::<0>(i, x)) else { continue };
            unit_rows(&groups, &self.tri_dofs(i), (t.e * t.depth).sqrt(), &eq, n, &mut rows);
        }
        for i in 0..self.tets.len() {
            let t = &self.tets[i];
            let Ok(groups) = t.rows(&self.tet_places_d::<0>(i, x)) else { continue };
            unit_rows(&groups, &self.tet_dofs(i), t.e.sqrt(), &eq, n, &mut rows);
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

/// The constants before the structure: `[dim, nodes, bars, beams, spreads, sized, tris, laid,
/// moved, tets, borne, stacked]` — `sized` how many unknowns the beams' sections are read over
/// (`Sizing`), or −1 where each section's numbers are columns of their own; `tris` and `laid` the
/// plane elements and the loads laid on them (#510 S2); `moved` how many columns of the drawing
/// the nodes are read over (`Moved`), or −1 where their coordinates are the columns; `tets` and
/// `borne` the solid elements and the loads they bear (#510 S3); `stacked` how many nodes of the
/// plane a solid's nodes are stacked over (`Stack`), or −1 where the nodes are the plane's own —
/// then the output stated, `J₀`, or where an unknown sets it, its `(m, c)`.
pub const HEAD: usize = 12;

/// A beam's constants: its ends, its moduli, its density and its section's z.
const BEAM: usize = 8;

/// A plane element's constants: its six nodes, its modulus, ratio, density and depth, and
/// whether it is read in plane strain.
const TRI: usize = 11;

/// A laid load's constants before what it lies on: its kind, how many sides or elements, and
/// two numbers (a pressure and its depth, or a force).
const LAID: usize = 4;

/// A solid element's constants: its ten nodes, its modulus, ratio and density.
const TET: usize = 13;

/// A borne load's constants before what it lies on: its kind, how many facets or elements, and
/// three numbers (a pressure and two zeros, or a force).
const BORNE: usize = 5;

impl Borne {
    /// Its constants: `[kind, count, a, b, c]`, then each facet's six nodes or each element.
    fn consts(&self) -> Vec<f64> {
        let (kind, head) = match self {
            Borne::Pressure { p, .. } => (0.0, [*p, 0.0, 0.0]),
            Borne::Across { force, .. } => (1.0, *force),
            Borne::Through { force, .. } => (2.0, *force),
            Borne::Body { force, .. } => (3.0, *force),
        };
        let count = self.facets().len() + self.elements().len();
        let mut k = vec![kind, count as f64];
        k.extend(head);
        k.extend(self.facets().iter().flatten().map(|&n| n as f64));
        k.extend(self.elements().iter().map(|&e| e as f64));
        k
    }

    /// How many constants it is.
    fn width(&self) -> usize {
        BORNE + 6 * self.facets().len() + self.elements().len()
    }
}

impl Laid {
    /// Its constants: `[kind, count, a, b]`, then each side's three nodes or each element.
    fn consts(&self) -> Vec<f64> {
        let (kind, head) = match self {
            Laid::Pressure { p, depth, .. } => (0.0, [*p, *depth]),
            Laid::Along { force, .. } => (1.0, *force),
            Laid::Over { force, .. } => (2.0, *force),
            Laid::Body { force, .. } => (3.0, *force),
        };
        let count = self.sides().len() + self.elements().len();
        let mut k = vec![kind, count as f64];
        k.extend(head);
        k.extend(self.sides().iter().flatten().map(|&n| n as f64));
        k.extend(self.elements().iter().map(|&e| e as f64));
        k
    }

    /// How many constants it is.
    fn width(&self) -> usize {
        LAID + 3 * self.sides().len() + self.elements().len()
    }
}

/// **A row's sections read over its unknowns** (#510 S1): a beam sized by a requirement states
/// its section over unknowns of the drawing (`Iz: b * h^3 / 12`, `h` declared), so the row's
/// columns after the bars' areas are those `unknowns`, and each section's number is a function
/// of them — a flat tape (`tape::Tape::flat`), per beam in `SECTION`'s order, a stated number a
/// tape of one constant.  The structure is solved at the sections the tapes give, and its
/// gradient in a section's number is carried to the unknowns by the tape's own.
#[derive(Clone, Debug, PartialEq)]
pub struct Sizing {
    pub unknowns: usize,
    pub sections: Vec<[Vec<f64>; SECTION]>,
}

impl Sizing {
    /// Beam `i`'s section at the unknowns `u`, each number with its gradient in them.
    fn section(&self, i: usize, u: &[f64], sc: &mut crate::tape::Scratch)
        -> [crate::tape::Value; SECTION]
    {
        self.sections[i].each_ref().map(|t| crate::tape::eval_flat(t, self.unknowns, u, sc))
    }
}

impl Structure {
    /// A row's constants: the head, every bar's `(a, b, E)`, every beam's `(a, b, E, G, ρ, z)`,
    /// every spread load's `(count, per_area, force, beams…)`, then every degree of freedom's
    /// `(held, load, weight)`.  `stated` is `[J₀]`, or `[m, c]` for an output an unknown sets.
    pub fn consts(&self, stated: &[f64]) -> Vec<f64> {
        self.consts_sized(stated, None)
    }

    /// `consts`, and after them, where the sections are read over unknowns, every beam's four
    /// tapes, each `(length, flat…)`.
    pub fn consts_sized(&self, stated: &[f64], sizing: Option<&Sizing>) -> Vec<f64> {
        self.consts_with(stated, sizing, None)
    }

    /// `consts_sized`, and after them, where the nodes are read over the drawing's columns
    /// (#510 S2), how (`Moved::consts`).
    pub fn consts_with(&self, stated: &[f64], sizing: Option<&Sizing>, moved: Option<&Moved>)
        -> Vec<f64>
    {
        let sized = sizing.map_or(-1.0, |z| z.unknowns as f64);
        let mut k = Vec::with_capacity(self.widths_with(stated.len() == 2, sizing, moved).1);
        k.extend([self.dim, self.nodes, self.bars.len(), self.beams.len(), self.spread.len()]
            .map(|n| n as f64));
        k.push(sized);
        k.extend([self.tris.len() as f64, self.laid.len() as f64]);
        k.push(moved.map_or(-1.0, |m| m.columns as f64));
        k.extend([self.tets.len() as f64, self.borne.len() as f64]);
        k.push(moved.filter(|m| m.stack.is_some()).map_or(-1.0, |m| m.nodes.len() as f64));
        k.extend(stated);
        for b in &self.bars {
            k.extend([b.a as f64, b.b as f64, b.e]);
        }
        for b in &self.beams {
            k.extend([b.a as f64, b.b as f64, b.e, b.g, b.rho]);
            k.extend(b.z);
        }
        for s in &self.spread {
            k.extend([s.beams.len() as f64, s.per_area as u8 as f64]);
            k.extend(s.force);
            k.extend(s.beams.iter().map(|&i| i as f64));
        }
        for t in &self.tris {
            k.extend(t.n.map(|n| n as f64));
            k.extend([t.e, t.nu, t.rho, t.depth, t.strain as u8 as f64]);
        }
        for l in &self.laid {
            k.extend(l.consts());
        }
        for t in &self.tets {
            k.extend(t.n.map(|n| n as f64));
            k.extend([t.e, t.nu, t.rho]);
        }
        for l in &self.borne {
            k.extend(l.consts());
        }
        for i in 0..self.fixed.len() {
            k.extend([self.fixed[i] as u8 as f64, self.load[i], self.weight[i]]);
        }
        for tapes in sizing.iter().flat_map(|z| &z.sections) {
            for t in tapes {
                k.push(t.len() as f64);
                k.extend(t);
            }
        }
        if let Some(m) = moved {
            k.extend(m.consts());
        }
        k
    }

    /// What `consts` wrote, `free` where an unknown sets the output (the row has its column, as
    /// `integral::length_widths` reads a length's): the structure, and what it states.
    pub fn read(k: &[f64], free: bool) -> (Structure, &[f64]) {
        let (s, stated, _) = Structure::read_sized(k, free).expect("a structure's constants");
        (s, stated)
    }

    /// What `consts_sized` wrote: the structure, what it states and its sizing — `None` for
    /// constants that are not whole (a document is untrusted input).
    pub fn read_sized(k: &[f64], free: bool) -> Option<(Structure, &[f64], Option<Sizing>)> {
        let (s, stated, sizing, moved) = Structure::read_with(k, free)?;
        moved.is_none().then_some((s, stated, sizing))
    }

    /// What `consts_with` wrote: `read_sized`'s, and how the nodes are read over the drawing's
    /// columns where they are (#510 S2) — every node held by it, in the plane.
    #[allow(clippy::type_complexity)]
    pub fn read_with(k: &[f64], free: bool)
        -> Option<(Structure, &[f64], Option<Sizing>, Option<Moved>)>
    {
        let count = |v: f64| (v >= 0.0 && v.fract() == 0.0 && v < 1e9).then_some(v as usize);
        let head = k.get(..HEAD)?;
        let (dim, nodes) = (count(head[0])?, count(head[1])?);
        let (n_bars, n_beams, n_spread) = (count(head[2])?, count(head[3])?, count(head[4])?);
        let (n_tris, n_laid) = (count(head[6])?, count(head[7])?);
        let (n_tets, n_borne) = (count(head[9])?, count(head[10])?);
        let mut at = HEAD;
        let mut take = |n: usize| -> Option<&[f64]> {
            let out = k.get(at..at.checked_add(n)?)?;
            at += n;
            Some(out)
        };
        let stated = take(if free { 2 } else { 1 })?;
        let bars = take(3 * n_bars)?;
        let beams = take(BEAM * n_beams)?;
        let mut spread = Vec::with_capacity(n_spread);
        for _ in 0..n_spread {
            let c = take(5)?;
            let along = take(count(c[0])?)?;
            let along: Option<Vec<usize>> =
                along.iter().map(|&v| count(v).filter(|&i| i < n_beams)).collect();
            spread.push(Spread { beams: along?, force: [c[2], c[3], c[4]], per_area: c[1] != 0.0 });
        }
        let node = |v: f64| count(v).filter(|&i| i < nodes);
        let tris: Vec<Tri6> = take(TRI * n_tris)?.as_chunks::<TRI>().0.iter().map(|c| {
            let n: Option<Vec<usize>> = c[..6].iter().map(|&v| node(v)).collect();
            Some(Tri6 { n: n?.try_into().ok()?, e: c[6], nu: c[7], rho: c[8], depth: c[9],
                        strain: c[10] != 0.0 })
        }).collect::<Option<_>>()?;
        let mut laid = Vec::with_capacity(n_laid);
        for _ in 0..n_laid {
            let c = take(LAID)?;
            let n = count(c[1])?;
            let (kind, a, b) = (count(c[0])?, c[2], c[3]);
            let l = if kind < 2 {
                let sides: Vec<[usize; 3]> = take(3 * n)?.as_chunks::<3>().0.iter()
                    .map(|s| Some([node(s[0])?, node(s[1])?, node(s[2])?])).collect::<Option<_>>()?;
                match kind {
                    0 => Laid::Pressure { sides, p: a, depth: b },
                    _ => Laid::Along { sides, force: [a, b] },
                }
            } else {
                let elements: Vec<usize> = take(n)?.iter()
                    .map(|&v| count(v).filter(|&i| i < n_tris)).collect::<Option<_>>()?;
                match kind {
                    2 => Laid::Over { elements, force: [a, b] },
                    3 => Laid::Body { elements, force: [a, b] },
                    _ => return None,
                }
            };
            laid.push(l);
        }
        let tets: Vec<Tet10> = take(TET * n_tets)?.as_chunks::<TET>().0.iter().map(|c| {
            let n: Option<Vec<usize>> = c[..10].iter().map(|&v| node(v)).collect();
            Some(Tet10 { n: n?.try_into().ok()?, e: c[10], nu: c[11], rho: c[12] })
        }).collect::<Option<_>>()?;
        let mut borne = Vec::with_capacity(n_borne);
        for _ in 0..n_borne {
            let c = take(BORNE)?;
            let n = count(c[1])?;
            let (kind, f) = (count(c[0])?, [c[2], c[3], c[4]]);
            let l = if kind < 2 {
                let facets: Vec<[usize; 6]> = take(6 * n)?.as_chunks::<6>().0.iter().map(|s| {
                    let s = s.map(node);
                    Some([s[0]?, s[1]?, s[2]?, s[3]?, s[4]?, s[5]?])
                }).collect::<Option<_>>()?;
                match kind {
                    0 => Borne::Pressure { facets, p: f[0] },
                    _ => Borne::Across { facets, force: f },
                }
            } else {
                let elements: Vec<usize> = take(n)?.iter()
                    .map(|&v| count(v).filter(|&i| i < n_tets)).collect::<Option<_>>()?;
                match kind {
                    2 => Borne::Through { elements, force: f },
                    3 => Borne::Body { elements, force: f },
                    _ => return None,
                }
            };
            borne.push(l);
        }
        let s = Structure {
            dim,
            nodes,
            bars: bars.as_chunks::<3>().0.iter()
                .map(|c| Some(Bar { a: node(c[0])?, b: node(c[1])?, e: c[2] }))
                .collect::<Option<_>>()?,
            beams: beams.as_chunks::<BEAM>().0.iter()
                .map(|c| Some(Beam { a: node(c[0])?, b: node(c[1])?, e: c[2], g: c[3], rho: c[4],
                                     z: [c[5], c[6], c[7]] }))
                .collect::<Option<_>>()?,
            fixed: Vec::new(),
            load: Vec::new(),
            weight: Vec::new(),
            spread,
            tris,
            laid,
            tets,
            borne,
        };
        // a plane element is of the plane, a solid one of space, each moves without turning, and
        // neither is a member's nor the other's
        let mixed = !s.beams.is_empty() || !s.bars.is_empty();
        if !(dim == 2 || dim == 3) || (!s.tris.is_empty() && (dim != 2 || mixed))
            || (!s.tets.is_empty() && (dim != 3 || mixed || !s.tris.is_empty()))
        {
            return None;
        }
        let per = take(3 * s.per() * nodes)?.as_chunks::<3>().0;
        let mut s = s;
        s.fixed = per.iter().map(|c| c[0] != 0.0).collect();
        s.load = per.iter().map(|c| c[1]).collect();
        s.weight = per.iter().map(|c| c[2]).collect();
        let sizing = match head[5] {
            v if v < 0.0 => None,
            v => {
                let unknowns = count(v).filter(|&n| n <= crate::tape::MAX_VARS)?;
                let mut sections = Vec::with_capacity(n_beams);
                for _ in 0..n_beams {
                    let mut four: [Vec<f64>; SECTION] = Default::default();
                    for t in &mut four {
                        let n = count(*take(1)?.first()?)?;
                        let flat = take(n)?;
                        if !crate::tape::reads_within(flat, unknowns) {
                            return None;
                        }
                        *t = flat.to_vec();
                    }
                    sections.push(four);
                }
                Some(Sizing { unknowns, sections })
            }
        };
        let moved = match head[8] {
            v if v < 0.0 => None,
            _ => {
                let (m, used) = Moved::read(k.get(at..)?, head[11] >= 0.0)?;
                at += used;
                let plane = m.nodes.len();
                let fits = m.x0.len() == 2 * plane && head[8] == m.columns as f64
                    && match &m.stack {
                        None => plane == s.nodes && s.dim == 2,
                        Some(st) => st.of.len() == s.nodes && s.dim == 3
                            && head[11] == plane as f64,
                    };
                Some(fits.then_some(m)?)
            }
        };
        (at == k.len()).then_some((s, stated, sizing, moved))
    }

    /// A row's widths, `(n_par, n_const)`: its columns, and the unknown's where it has one.
    pub fn widths(&self, free: bool) -> (usize, usize) {
        self.widths_sized(free, None)
    }

    /// `widths`, its sections read over `sizing`'s unknowns where there is one.
    pub fn widths_sized(&self, free: bool, sizing: Option<&Sizing>) -> (usize, usize) {
        self.widths_with(free, sizing, None)
    }

    /// `widths_sized`, its nodes read over the drawing's columns where `moved` is.
    pub fn widths_with(&self, free: bool, sizing: Option<&Sizing>, moved: Option<&Moved>)
        -> (usize, usize)
    {
        let stated = if free { 2 } else { 1 };
        let spread: usize = self.spread.iter().map(|s| 5 + s.beams.len()).sum();
        let tapes: usize =
            sizing.iter().flat_map(|z| z.sections.iter().flatten()).map(|t| 1 + t.len()).sum();
        let laid: usize = self.laid.iter().map(Laid::width).sum();
        let borne: usize = self.borne.iter().map(Borne::width).sum();
        let nodes = moved.map_or(self.dim * self.nodes, |m| m.columns);
        (self.row_columns(sizing) - self.dim * self.nodes + nodes + free as usize,
         HEAD + stated + 3 * self.bars.len() + BEAM * self.beams.len() + spread
             + TRI * self.tris.len() + laid + TET * self.tets.len() + borne
             + 3 * self.fixed.len() + tapes + moved.map_or(0, Moved::width))
    }

    /// How many columns a row over the structure reads before the unknown that may set its
    /// output: the nodes' coordinates, the bars' areas, then the beams' sections or the unknowns
    /// they are read over.
    pub fn row_columns(&self, sizing: Option<&Sizing>) -> usize {
        self.dim * self.nodes + self.bars.len()
            + sizing.map_or(SECTION * self.beams.len(), |z| z.unknowns)
    }
}

/* -- the kernel --------------------------------------------------------------------------- */

/// The kernel at a block's widths (`system::build_kernel`, `KernelKey::Interior`): one row, the
/// output less what it states, `J(x) − J₀` or `J(x) − (m·a + c)`.  A displacement, so degree 1.
/// A structure with no displacement to give (`Failure`), and constants that are no structure,
/// read NaN, which the step rejects.
pub fn kernel(n_par: usize, n_const: usize) -> Kernel {
    Kernel { name: "interior", n_res: 1, n_par, n_const, degree: 1, res, jac, const_jac: None,
             form: None }
}

/// Whether a row carries the unknown's column: one more than the columns its head says its
/// structure reads — the nodes' coordinates, the bars' areas, then the sections or the unknowns
/// they are read over.  `None` where there is no head.
fn free_at(n_par: usize, k: &[f64]) -> Option<bool> {
    let h = k.get(..HEAD)?;
    let sections = if h[5] < 0.0 { SECTION as f64 * h[3] } else { h[5] };
    let nodes = if h[8] < 0.0 { h[0] * h[1] } else { h[8] };
    Some(n_par as f64 > nodes + h[2] + sections)
}

/// One row read off its columns `v` and constants `k`: the structure, the columns it is solved
/// at, what it states, whether an unknown sets its output, and where its sections are read over
/// unknowns each one's value and rate in them.
struct Row<'a> {
    s: Structure,
    x: Vec<f64>,
    stated: &'a [f64],
    free: bool,
    sized: Option<(usize, Vec<[crate::tape::Value; SECTION]>)>,
    /// Where the nodes are read over the drawing's columns (#510 S2): how, and where they stand.
    moved: Option<(Moved, moved::Placed)>,
}

impl<'a> Row<'a> {
    fn read(v: &[f64], k: &'a [f64]) -> Option<Row<'a>> {
        let free = free_at(v.len(), k)?;
        let (s, stated, sizing, moved) = Structure::read_with(k, free)?;
        if let Some(m) = moved {
            // a plane structure's nodes, read over the drawing: no areas and no sections
            if v.len() != m.columns + free as usize || !s.bars.is_empty() || !s.beams.is_empty() {
                return None;
            }
            let placed = m.places(&v[..m.columns])?;
            let x = placed.x.clone();
            return Some(Row { s, x, stated, free, sized: None, moved: Some((m, placed)) });
        }
        let cols = s.row_columns(sizing.as_ref());
        if v.len() != cols + free as usize {
            return None;
        }
        let direct = s.section_at(0);
        let mut x = v[..direct].to_vec();
        let sized = sizing.map(|z| {
            let mut sc = crate::tape::Scratch::new();
            let sections: Vec<_> =
                (0..s.beams.len()).map(|i| z.section(i, &v[direct..cols], &mut sc)).collect();
            x.extend(sections.iter().flatten().map(|t| t.v));
            (z.unknowns, sections)
        });
        if sized.is_none() {
            x.extend(&v[direct..cols]);
        }
        Some(Row { s, x, stated, free, sized, moved: None })
    }

    /// The columns the structure is read over, before the unknown that may set its output.
    fn columns(&self) -> usize {
        if let Some((m, _)) = &self.moved {
            return m.columns;
        }
        self.s.section_at(0) + self.sized.as_ref().map_or(SECTION * self.s.beams.len(), |z| z.0)
    }

    /// What the row states the output is, at the unknown's column `v` reads.
    fn target(&self, v: &[f64]) -> f64 {
        match self.free {
            true => self.stated[0] * v[self.columns()] + self.stated[1],
            false => self.stated[0],
        }
    }
}

fn res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    for i in 0..n {
        let (v, k) = (&v[n_par * i..n_par * (i + 1)], &k[n_const * i..n_const * (i + 1)]);
        r[i] = Row::read(v, k).and_then(|row| {
            let st = row.s.solve(&row.x).ok()?;
            Some(row.s.output(&st) - row.target(v))
        }).unwrap_or(f64::NAN);
    }
}

fn jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    for i in 0..n {
        let (v, k) = (&v[n_par * i..n_par * (i + 1)], &k[n_const * i..n_const * (i + 1)]);
        let row = &mut j[n_par * i..n_par * (i + 1)];
        row.fill(0.0);
        let Some(read) = Row::read(v, k) else {
            row.fill(f64::NAN);
            continue;
        };
        let s = &read.s;
        let Ok(st) = s.solve(&read.x) else {
            row.fill(f64::NAN);
            continue;
        };
        let cols = read.columns();
        match (&read.sized, &read.moved) {
            // the nodes' rates carried through how they move with the drawing (#510 S2)
            (_, Some((m, placed))) => {
                let mut g = vec![0.0; s.columns()];
                s.gradient(&st, &read.x, &mut g);
                m.chain(&v[..cols], placed, &g, &mut row[..cols]);
            }
            (None, None) => s.gradient(&st, &read.x, &mut row[..cols]),
            // a section's rate carried to the unknowns it is read over
            (Some((unknowns, sections)), None) => {
                let mut g = vec![0.0; s.columns()];
                s.gradient(&st, &read.x, &mut g);
                let direct = s.section_at(0);
                row[..direct].copy_from_slice(&g[..direct]);
                for (b, four) in sections.iter().enumerate() {
                    for (c, t) in four.iter().enumerate() {
                        for u in 0..*unknowns {
                            row[direct + u] += g[s.section_at(b) + c] * t.d[u];
                        }
                    }
                }
            }
        }
        if read.free {
            row[cols] = -read.stated[0];
        }
    }
}
