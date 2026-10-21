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
//! The element here is the pin-jointed bar, in the plane or in space: the least that is a
//! structure, enough to hold the mechanism to its contract.  Frames, plane elements and solids
//! are further elements of the same interior (#510 S0–S3).

use crate::kernels::{curve_widths, Kernel};
use crate::sparse::Ata;

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

/// A structure of bars: `nodes` points of `dim` (2 or 3) coordinates each, and per degree of
/// freedom (`dim · node + axis`) whether a support holds it, the load on it, and its weight in
/// the output `J = Σ weight · u`.  The weights are numbers, a direction's cosines, so the output
/// is a displacement: a length.
///
/// Its columns are every node's coordinates, then every bar's area (`Structure::columns`); the
/// modulus, the supports, the loads and the weights are constants.
#[derive(Clone, Debug, PartialEq)]
pub struct Structure {
    pub dim: usize,
    pub nodes: usize,
    pub bars: Vec<Bar>,
    pub fixed: Vec<bool>,
    pub load: Vec<f64>,
    pub weight: Vec<f64>,
}

/// Why a structure has no displacement to give.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Failure {
    /// `K` is singular: the supports leave a motion that costs no energy (`SINGULAR`).
    Free,
    /// A bar of no length, which has no direction to be stiff along.
    Collapsed,
    /// A bar of no stiffness or less (`E·A ≤ 0`), which is no member.
    Unstiff,
}

/// One bar's geometry at `x`: `Δ = x_b − x_a`, its length, and `E·A`.
fn bar_at(s: &Structure, b: &Bar, i: usize, x: &[f64]) -> ([f64; 3], f64, f64) {
    let mut d = [0.0; 3];
    for k in 0..s.dim {
        d[k] = x[s.dim * b.b + k] - x[s.dim * b.a + k];
    }
    let l = d.iter().map(|c| c * c).sum::<f64>().sqrt();
    (d, l, b.e * x[s.dim * s.nodes + i])
}

/// The displacements of a structure at `x`, with the factor they were solved on held for the
/// adjoint.
pub struct State {
    ata: Ata,
    /// Per degree of freedom, its equation, or `None` where a support holds it.
    eq: Vec<Option<usize>>,
    /// Per degree of freedom, zero where held.
    pub u: Vec<f64>,
    /// The solves made on the factor: the displacements', and one for each gradient.
    pub solves: usize,
}

impl Structure {
    /// How many columns the structure reads: the nodes' coordinates, then the bars' areas.
    pub fn columns(&self) -> usize {
        self.dim * self.nodes + self.bars.len()
    }

    /// Assemble `K(x)` as strain rows, factor it and solve `K u = f`.
    pub fn solve(&self, x: &[f64]) -> Result<State, Failure> {
        let dofs = self.dim * self.nodes;
        let mut eq = vec![None; dofs];
        let mut n_eq = 0;
        for (i, held) in self.fixed.iter().enumerate() {
            if !held {
                eq[i] = Some(n_eq);
                n_eq += 1;
            }
        }
        let (mut indptr, mut indices, mut data) = (vec![0i32], Vec::new(), Vec::new());
        for (i, b) in self.bars.iter().enumerate() {
            let (d, l, ea) = bar_at(self, b, i, x);
            if l.is_nan() || l <= 0.0 {
                return Err(Failure::Collapsed);
            }
            if ea.is_nan() || ea <= 0.0 {
                return Err(Failure::Unstiff);
            }
            let s = (ea / l).sqrt() / l;
            for (node, sign) in [(b.a, -1.0), (b.b, 1.0)] {
                for k in 0..self.dim {
                    if let Some(e) = eq[self.dim * node + k] {
                        indices.push(e as i32);
                        data.push(sign * s * d[k]);
                    }
                }
            }
            indptr.push(indices.len() as i32);
        }
        let mut ata = Ata::new(self.bars.len(), n_eq, &indptr, &indices);
        ata.fill(&data);
        let mut rhs: Vec<f64> =
            (0..dofs).filter(|&i| eq[i].is_some()).map(|i| self.load[i]).collect();
        if !ata.solve(&vec![0.0; n_eq], &mut rhs) || ata.least_pivot() < SINGULAR {
            return Err(Failure::Free);
        }
        let u = eq.iter().map(|e| e.map_or(0.0, |e| rhs[e])).collect();
        Ok(State { ata, eq, u, solves: 1 })
    }

    /// The output, `Σ weight · u`.
    pub fn output(&self, st: &State) -> f64 {
        self.weight.iter().zip(&st.u).map(|(w, u)| w * u).sum()
    }

    /// The output's gradient in the columns at `x`, added into `g` scaled by `scale`: one solve
    /// for `λ` on the factor `st` holds, then each bar's mutual energy differentiated.  The loads
    /// do not move with `x` and the output reads only `u`, so `∂f/∂x` and `∂J/∂x` are zero and
    /// what is left is `−λᵀ(∂K/∂x)u`.
    pub fn gradient(&self, st: &mut State, x: &[f64], scale: f64, g: &mut [f64]) {
        let dofs = self.dim * self.nodes;
        let mut rhs: Vec<f64> =
            (0..dofs).filter(|&i| st.eq[i].is_some()).map(|i| self.weight[i]).collect();
        st.ata.resolve(&mut rhs);
        st.solves += 1;
        let lam: Vec<f64> = st.eq.iter().map(|e| e.map_or(0.0, |e| rhs[e])).collect();
        for (i, b) in self.bars.iter().enumerate() {
            let (d, l, ea) = bar_at(self, b, i, x);
            let (dm, da) = b.mutual(self.dim, d, l, ea, &lam, &st.u);
            for k in 0..self.dim {
                g[self.dim * b.b + k] -= scale * dm[k];
                g[self.dim * b.a + k] += scale * dm[k];
            }
            g[dofs + i] -= scale * da;
        }
    }
}

impl Bar {
    /// The gradient of the mutual energy `aᵀkₑb = E·A·(Δ·α)(Δ·β)/L³` (with `α`, `β` the ends'
    /// relative displacements under `a` and `b`) in `Δ = x_b − x_a`, and in the area.
    fn mutual(&self, dim: usize, d: [f64; 3], l: f64, ea: f64, a: &[f64], b: &[f64])
              -> ([f64; 3], f64) {
        let (mut al, mut be) = ([0.0; 3], [0.0; 3]);
        for k in 0..dim {
            al[k] = a[dim * self.b + k] - a[dim * self.a + k];
            be[k] = b[dim * self.b + k] - b[dim * self.a + k];
        }
        let (da, db) = (dot(d, al), dot(d, be));
        let l3 = l * l * l;
        let m = ea * da * db / l3;
        let mut dm = [0.0; 3];
        for k in 0..dim {
            dm[k] = ea * (al[k] * db + be[k] * da) / l3 - 3.0 * m * d[k] / (l * l);
        }
        (dm, self.e * da * db / l3)
    }
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/* -- the row's constants ------------------------------------------------------------------ */

/// The constants before the structure: `[dim, nodes, bars, free, s₀, s₁]`, with `(s₀, s₁)` the
/// output stated (`s₁` unread) or, where an unknown sets it, its `(m, c)`.
pub const HEAD: usize = 6;

impl Structure {
    /// A row's constants: the head, every bar's `(a, b, E)`, then every degree of freedom's
    /// `(held, load, weight)`.  `stated` is `[J]`, or `[m, c]` for an output an unknown sets.
    pub fn consts(&self, stated: &[f64]) -> Vec<f64> {
        let dofs = self.dim * self.nodes;
        let mut k = Vec::with_capacity(HEAD + 3 * self.bars.len() + 3 * dofs);
        let free = stated.len() == 2;
        k.extend([self.dim as f64, self.nodes as f64, self.bars.len() as f64, free as u8 as f64]);
        k.extend([stated[0], if free { stated[1] } else { 0.0 }]);
        for b in &self.bars {
            k.extend([b.a as f64, b.b as f64, b.e]);
        }
        for i in 0..dofs {
            k.extend([self.fixed[i] as u8 as f64, self.load[i], self.weight[i]]);
        }
        k
    }

    /// What `consts` wrote: the structure, whether an unknown sets the output, and `(s₀, s₁)`.
    pub fn read(k: &[f64]) -> (Structure, bool, [f64; 2]) {
        let (dim, nodes, n_bars) = (k[0] as usize, k[1] as usize, k[2] as usize);
        let bars = k[HEAD..HEAD + 3 * n_bars]
            .as_chunks::<3>()
            .0
            .iter()
            .map(|c| Bar { a: c[0] as usize, b: c[1] as usize, e: c[2] })
            .collect();
        let per = k[HEAD + 3 * n_bars..HEAD + 3 * n_bars + 3 * dim * nodes].as_chunks::<3>().0;
        let s = Structure {
            dim,
            nodes,
            bars,
            fixed: per.iter().map(|c| c[0] != 0.0).collect(),
            load: per.iter().map(|c| c[1]).collect(),
            weight: per.iter().map(|c| c[2]).collect(),
        };
        (s, k[3] != 0.0, [k[4], k[5]])
    }

    /// A row's widths, `(n_par, n_const)`: its columns, and the unknown's where it has one.
    pub fn widths(&self, free: bool) -> (usize, usize) {
        (self.columns() + free as usize, HEAD + 3 * self.bars.len() + 3 * self.dim * self.nodes)
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

fn res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let (n_par, n_const) = curve_widths(n, v, k);
    for i in 0..n {
        let (v, k) = (&v[n_par * i..n_par * (i + 1)], &k[n_const * i..n_const * (i + 1)]);
        let (s, free, stated) = Structure::read(k);
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
        let (s, free, stated) = Structure::read(k);
        match s.solve(v) {
            Ok(mut st) => s.gradient(&mut st, v, 1.0, row),
            Err(_) => row.fill(f64::NAN),
        }
        if free {
            row[s.columns()] = -stated[0];
        }
    }
}
