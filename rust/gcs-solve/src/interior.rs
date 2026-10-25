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
use crate::kernels::{Dual, Num};
use crate::linalg::{rank_and_nullspace, Mat};
use crate::space::{dot, norm, scale, sub};
use crate::delaunay::tetra::EDGES;
use crate::cholesky::Symmetric;
use crate::sparse::Ata;

mod beam;
mod consts;
mod hull;
mod moved;
mod plane;
mod row;
mod solid;
pub use beam::{fixed_end, into_axes, out_of_axes, Axes};
pub use consts::{Drive, Mover, Sizing, HEAD};
pub use hull::{Axis, Frame, Hull, Profile, Sheet};
pub use moved::{Edge, Moved, Node, Stack};
pub use plane::{Laid, Tri6};
pub use solid::{Borne, Tet10};
pub use row::{forget, kernel, reach_of, reading};

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
/// What a row over it reads is `reads`: that displacement, or what the structure is made of —
/// its volume or its mass, which no load moves.
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
    pub reads: Reads,
}

/// **What a row reads of its structure**: the displacement `Σ weight · u` its loads give (#510
/// P5), one solve and its adjoint; or what it is made of, which no load moves and no solve reads
/// — its volume (each bar's and beam's `A L`, each plane element's `t ∫ det J`, each solid one's
/// `∫ det J`) or its mass (each beam's, plane element's and solid element's density times its
/// volume; a bar carries none).  A measure's gradient is its own `∂J/∂x`, with no factor, so a
/// structure its supports leave free, or holds by none, still has one (`Structure::measure`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Reads {
    #[default]
    Displacement,
    Volume,
    Mass,
}

impl Reads {
    /// Whether it is a measure of the structure rather than a displacement of it.
    pub fn measure(self) -> bool {
        self != Reads::Displacement
    }

    /// What a row of the drawing's `quantity` reads.
    pub fn of(quantity: crate::model::Quantity) -> Reads {
        match quantity {
            crate::model::Quantity::Mass => Reads::Mass,
            crate::model::Quantity::Volume => Reads::Volume,
        }
    }

    /// What weighs a member or an element of density `rho`: its density for a mass, one for a
    /// volume (or a displacement, which weighs nothing).
    pub fn density(self, rho: f64) -> f64 {
        if self == Reads::Mass { rho } else { 1.0 }
    }
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
    /// The factor, and the stiffness it was made from: what a pencil reads (`State::stiffness`),
    /// a twentieth of its factor's fill in three dimensions.
    Cholesky(crate::cholesky::Cholesky, Symmetric),
    /// A solid's stiffness held for CG over its corners' space (`pcg`, `ITERATIVE`), and how
    /// many iterations its solve took.
    Pcg(crate::pcg::Pcg, usize),
}

impl Factor {
    /// `b` overwritten by `K⁻¹ b` on the factor in hand (by CG, to `pcg::TOLERANCE`).
    fn resolve(&self, b: &mut [f64]) {
        match self {
            Factor::Ata(a) => a.resolve(b),
            Factor::Cholesky(c, _) => c.solve(b),
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

    /// **The stiffness it was solved with**, a plane or a solid structure's lower triangle over
    /// the equations no support holds — the matrix `Structure::stiffness` assembles, held where
    /// the solve already did, so a reader's modes and buckling assemble it no second time (#632).
    /// `None` for bars and beams, solved on their strain rows.
    pub fn stiffness(&self) -> Option<&Symmetric> {
        match &self.ata {
            Factor::Cholesky(_, k) => Some(k),
            Factor::Pcg(cg, _) => Some(cg.lower()),
            Factor::Ata(_) => None,
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
            // the factor reads the lower triangle alone; its pattern's analysis remembered between
            // evaluations, the mesh's topology fixed
            let mut c = crate::cholesky::analysed(&k, crate::cholesky::Ordering::Amd);
            c.factor(&k).map_err(|_| Failure::Free)?;
            c.solve(&mut rhs);
            if rhs.iter().any(|v| !v.is_finite()) {
                return Err(Failure::Free);
            }
            Factor::Cholesky(c, k)
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
    /// numbers, assembled in place (`in_place`).
    fn assembled(&self, x: &[f64], eq: &[Option<usize>], n_eq: usize)
        -> Result<Symmetric, Failure>
    {
        self.in_place(eq, n_eq, |i| self.tris[i].stiffness(&self.tri_places(i, x)),
                      |i| self.tets[i].stiffness(&self.tet_places(i, x)))
    }

    /// **A plane or a solid structure's matrix**, its lower triangle over the equations `eq`
    /// numbers, assembled in place (#632): its pattern from the elements' nodes
    /// (`Symmetric::pattern`), then each plane element's matrix `tri(i)` and each solid
    /// element's `tet(i)` added at its entries.  The stiffness, the mass and the geometric
    /// stiffness of one mesh all come out on the one pattern, so a pencil of two is on it too.
    fn in_place(&self, eq: &[Option<usize>], n_eq: usize,
                tri: impl Fn(usize) -> Result<[[f64; 12]; 12], Failure>,
                tet: impl Fn(usize) -> Result<[[f64; 30]; 30], Failure>)
        -> Result<Symmetric, Failure>
    {
        let elements =
            self.tris.iter().map(|t| &t.n[..]).chain(self.tets.iter().map(|t| &t.n[..]));
        let mut k = Symmetric::pattern(self.nodes, self.per(), elements, eq, n_eq);
        for i in 0..self.tris.len() {
            k.add(&tri(i)?, &self.tri_dofs(i).map(|d| eq[d]));
        }
        for i in 0..self.tets.len() {
            k.add(&tet(i)?, &self.tet_dofs(i).map(|d| eq[d]));
        }
        Ok(k)
    }

    /// **A solid's stiffness and its corners' space** (`pcg`): `K` over the equations `eq`
    /// numbers (its lower triangle, `assembled`'s), per equation its row of `P` — a corner's move
    /// its own, a side's middle node's the mean of the side's ends', a support's held moves left
    /// out — and the Galerkin matrix `PᵀKP` over the corners' equations, assembled element by
    /// element as `P_eᵀ K_e P_e` (a side's ends are its own element's corners).
    fn cornered(&self, x: &[f64], eq: &[Option<usize>], n_eq: usize)
        -> Result<(Symmetric, Vec<[(u32, f64); 2]>, Symmetric), Failure>
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
        let mut k =
            Symmetric::pattern(self.nodes, 3, self.tets.iter().map(|t| &t.n[..]), eq, n_eq);
        let mut kc =
            Symmetric::pattern(self.nodes, 3, self.tets.iter().map(|t| &t.n[..4]), &ceq, nc);
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
        Ok((k, up, kc))
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
    /// for a pencil (`eigen.rs`) to read, its lower triangle.  A plane or a solid structure's is
    /// assembled in place on its mesh's pattern (`in_place`), the one `solve` factors, which a
    /// solved `State` holds already (`State::stiffness`).  A frame's — each bar's strain row's
    /// outer product, each beam's twelve-by-twelve stiffness `Σ p rrᵀ` in its own axes, turned —
    /// is assembled from triplets, its lower triangle's alone: a frame has a few hundred
    /// unknowns, and no mesh to lay a pattern from.
    pub fn stiffness(&self, x: &[f64]) -> Result<Symmetric, Failure> {
        let (eq, n_eq) = self.equations();
        if self.meshed() {
            return self.assembled(x, &eq, n_eq);
        }
        let mut t = Vec::new();
        for (i, b) in self.bars.iter().enumerate() {
            let (d, l, ea) = self.bar_at(i, x);
            check_bar(l, ea)?;
            let mut row = Vec::new();
            self.strain(b, scale(d, (ea / l).sqrt() / l), &eq, |e, v| row.push((e, v)));
            for &(i, a) in &row {
                t.extend(row.iter().filter(|&&(j, _)| j <= i).map(|&(j, b)| (i, j, a * b)));
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
        Ok(Symmetric::from_triplets(n_eq, &t))
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

    /// **The geometric stiffness of the solid elements** (#510 S3) under the stress the moves `u`
    /// (per degree of freedom) make, over the equations no support holds: each element's `∫ ∇Nᵀ
    /// σ ∇N dV` — what `eigen::buckling` reads beside `stiffness`, assembled in place on the
    /// pattern the stiffness is (`in_place`), so their pencil is on it too.
    pub fn stressed(&self, x: &[f64], u: &[f64]) -> Result<Symmetric, Failure> {
        let (eq, n_eq) = self.equations();
        self.in_place(&eq, n_eq, |_| Ok([[0.0; 12]; 12]), |i| {
            let ue: [f64; 30] = self.tet_dofs(i).map(|d| u[d]);
            self.tets[i].geometric(&self.tet_places(i, x), &ue).ok_or(Failure::Collapsed)
        })
    }

    /// The solid elements' volume at `x`.
    pub fn volume(&self, x: &[f64]) -> f64 {
        (0..self.tets.len()).map(|i| self.tets[i].volume(&self.tet_places(i, x))).sum()
    }

    /// **What the structure is made of, at `x`**, as `reads` says (`Reads::Volume`,
    /// `Reads::Mass`): each bar's and beam's `A L`, each plane element's `t ∫ det J`, each solid
    /// one's `∫ det J`, each weighed by its density for a mass (a bar, of none, weighing
    /// nothing).  No displacement is solved for.
    pub fn measure(&self, x: &[f64]) -> f64 {
        let w = |rho: f64| self.reads.density(rho);
        let mut m = 0.0;
        for (i, b) in self.bars.iter().enumerate() {
            m += w(0.0) * norm(self.place(b.a, b.b, x)) * x[self.dim * self.nodes + i];
        }
        for (i, b) in self.beams.iter().enumerate() {
            m += w(b.rho) * norm(self.place(b.a, b.b, x)) * self.section(i, x)[0];
        }
        for (i, t) in self.tris.iter().enumerate() {
            m += w(t.rho) * t.depth * t.area(&self.tri_places(i, x));
        }
        for (i, t) in self.tets.iter().enumerate() {
            m += w(t.rho) * t.volume(&self.tet_places(i, x));
        }
        m
    }

    /// **The measure at `x`, where it is one**: a mass or a volume more than nothing — else why
    /// it has none (a material of no density), which a row is refused for where it is stated.
    pub fn measured(&self, x: &[f64]) -> Result<f64, String> {
        let m = self.measure(x);
        if m.is_finite() && m > 0.0 {
            return Ok(m);
        }
        let what = if self.reads == Reads::Mass { "mass" } else { "volume" };
        Err(format!("it has no {what} where the drawing stands, so nothing it could be sized by"))
    }

    /// **The measure's gradient in the columns at `x`, added into `g`**: its own `∂J/∂x`, with no
    /// adjoint — a member's `w A ĝ` at its ends and `w L` in its area, an element's `w ∂(∫ det J)`
    /// over its nodes' places (`Dual` over its twelve or thirty coordinates).
    pub fn measure_gradient(&self, x: &[f64], g: &mut [f64]) {
        let (dim, w) = (self.dim, |rho: f64| self.reads.density(rho));
        let member = |a: usize, b: usize, at: usize, area: f64, w: f64, g: &mut [f64]| {
            let d = self.place(a, b, x);
            let l = norm(d);
            for k in 0..dim {
                g[dim * b + k] += w * area * d[k] / l;
                g[dim * a + k] -= w * area * d[k] / l;
            }
            g[at] += w * l;
        };
        for (i, b) in self.bars.iter().enumerate() {
            let at = dim * self.nodes + i;
            member(b.a, b.b, at, x[at], w(0.0), g);
        }
        for (i, b) in self.beams.iter().enumerate() {
            let at = self.section_at(i);
            member(b.a, b.b, at, x[at], w(b.rho), g);
        }
        for (i, t) in self.tris.iter().enumerate() {
            let a = plane::area_of(&self.tri_places_d::<12>(i, x));
            let w = w(t.rho) * t.depth;
            for (k, &n) in t.n.iter().enumerate() {
                for c in 0..2 {
                    g[2 * n + c] += w * a.g[2 * k + c];
                }
            }
        }
        for (i, t) in self.tets.iter().enumerate() {
            let v = solid::volume_of(&self.tet_places_d::<30>(i, x));
            let w = w(t.rho);
            for (k, &n) in t.n.iter().enumerate() {
                for c in 0..3 {
                    g[3 * n + c] += w * v.g[3 * k + c];
                }
            }
        }
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
