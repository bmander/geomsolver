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

use crate::diagnose::analysis::{self, Freedom, Held, Reach};
use crate::kernels::{curve_widths, Kernel};
use crate::linalg::{rank_and_nullspace, Mat};
use crate::space::{dot, norm};
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
    /// How many columns the structure reads: the nodes' coordinates, then the bars' areas.
    pub fn columns(&self) -> usize {
        self.dim * self.nodes + self.bars.len()
    }

    /// `v` at bar `b`'s far end less its near one, per axis: where a bar runs, or how its ends
    /// move apart.
    fn ends(&self, b: &Bar, v: &[f64]) -> [f64; 3] {
        let mut d = [0.0; 3];
        for k in 0..self.dim {
            d[k] = v[self.dim * b.b + k] - v[self.dim * b.a + k];
        }
        d
    }

    /// Bar `i`'s geometry at `x`: `Δ = x_b − x_a`, its length, and `E·A`.
    fn bar_at(&self, i: usize, x: &[f64]) -> ([f64; 3], f64, f64) {
        let b = &self.bars[i];
        let d = self.ends(b, x);
        (d, norm(d), b.e * x[self.dim * self.nodes + i])
    }

    /// Assemble `K(x)` as strain rows, factor it and solve `K u = f`.
    pub fn solve(&self, x: &[f64]) -> Result<State, Failure> {
        let mut eq = vec![None; self.fixed.len()];
        let mut n_eq = 0;
        for (i, held) in self.fixed.iter().enumerate() {
            if !held {
                eq[i] = Some(n_eq);
                n_eq += 1;
            }
        }
        let (mut indptr, mut indices, mut data) = (vec![0i32], Vec::new(), Vec::new());
        for (i, b) in self.bars.iter().enumerate() {
            let (d, l, ea) = self.bar_at(i, x);
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
        let mut st = State { ata, eq, u: Vec::new() };
        let mut rhs = st.gather(&self.load);
        if !st.ata.solve(&vec![0.0; n_eq], &mut rhs) || st.ata.least_pivot() < SINGULAR {
            return Err(Failure::Free);
        }
        st.u = st.scatter(&rhs);
        Ok(st)
    }

    /// The output, `Σ weight · u`.
    pub fn output(&self, st: &State) -> f64 {
        self.weight.iter().zip(&st.u).map(|(w, u)| w * u).sum()
    }

    /// The output's gradient in the columns at `x`, added into `g`: one solve for `λ` on the
    /// factor `st` holds — a `State` is only read, so no factorisation is made again — then each
    /// bar's mutual energy differentiated.  The loads do not move with `x` and the output reads
    /// only `u`, so `∂f/∂x` and `∂J/∂x` are zero and what is left is `−λᵀ(∂K/∂x)u`.
    pub fn gradient(&self, st: &State, x: &[f64], g: &mut [f64]) {
        let mut rhs = st.gather(&self.weight);
        st.ata.resolve(&mut rhs);
        let lam = st.scatter(&rhs);
        for (i, b) in self.bars.iter().enumerate() {
            let (d, l, ea) = self.bar_at(i, x);
            let (al, be) = (self.ends(b, &lam), self.ends(b, &st.u));
            let (dm, da) = b.mutual(d, l, ea, al, be);
            for k in 0..self.dim {
                g[self.dim * b.b + k] -= dm[k];
                g[self.dim * b.a + k] += dm[k];
            }
            g[self.dim * self.nodes + i] -= da;
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
    /// Whether nothing is left free (or nothing was looked for).
    pub fn held(&self) -> bool {
        self.rigid.is_empty() && self.mechanism.as_ref().is_none_or(|m| m.is_empty())
    }

    /// `free to turn about (0, 0)`, `a mechanism: nodes 2 and 3 move with no bar stretched` — or
    /// `None` where it is held.
    pub fn text(&self, dim: usize) -> Option<String> {
        let mut parts: Vec<String> =
            analysis::freedoms_text(&self.rigid, dim).into_iter().collect();
        if let Some(nodes) = self.mechanism.as_ref().filter(|m| !m.is_empty()) {
            let names: Vec<String> = nodes.iter().map(|n| n.to_string()).collect();
            parts.push(match names.split_last() {
                Some((one, [])) => format!("a mechanism: node {one} moves with no bar stretched"),
                Some((last, rest)) => format!("a mechanism: nodes {} and {last} move with no bar \
                                               stretched", rest.join(", ")),
                None => unreachable!(),
            });
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

    /// **What the supports and the bars leave free at `x`** (#510 P7): `Failure::Free`, named.  The
    /// rigid motions are the supports': every held degree of freedom holds its node along its
    /// axis.  A mechanism is what is left of the motions no bar stretches (each bar's strain row,
    /// `ĝ·(u_b − u_a) = 0`) once the rigid ones are taken out: its nodes are those it moves.  A
    /// bar of no length stretches along no direction and is left out.
    pub fn free(&self, x: &[f64]) -> Free {
        let holds: Vec<Held> = (0..self.fixed.len()).filter(|&i| self.fixed[i]).map(|i| {
            let mut along = [0.0; 3];
            along[i % self.dim] = 1.0;
            Held { at: vec![self.node(i / self.dim, x)], along: Some(along), turn: false }
        }).collect();
        let nodes: Vec<[f64; 3]> = (0..self.nodes).map(|i| self.node(i, x)).collect();
        let rigid = analysis::freedoms(self.dim, &nodes, &holds);
        let eq: Vec<usize> = (0..self.fixed.len()).filter(|&i| !self.fixed[i]).collect();
        if eq.len() > MECHANISM_MAX {
            return Free { rigid, mechanism: None };
        }
        let n = eq.len();
        let mut col = vec![usize::MAX; self.fixed.len()];
        for (j, &i) in eq.iter().enumerate() {
            col[i] = j;
        }
        let mut rows: Vec<Vec<f64>> = Vec::new();
        for b in &self.bars {
            let d = self.ends(b, x);
            let l = norm(d);
            if !(l > 0.0) {
                continue;
            }
            let mut r = vec![0.0; n];
            for (node, sign) in [(b.a, -1.0), (b.b, 1.0)] {
                for k in 0..self.dim {
                    if let Some(&j) = col.get(self.dim * node + k).filter(|&&j| j != usize::MAX) {
                        r[j] += sign * d[k] / l;
                    }
                }
            }
            rows.push(r);
        }
        // each rigid motion's field, so what is left is orthogonal to it
        for f in &rigid {
            let mut r = vec![0.0; n];
            for (j, &i) in eq.iter().enumerate() {
                r[j] = f.velocity(self.node(i / self.dim, x))[i % self.dim];
            }
            rows.push(r);
        }
        let flat: Vec<f64> = rows.iter().flatten().copied().collect();
        let null = rank_and_nullspace(&Mat::from_vec(rows.len(), n, flat), 1e-9).null();
        let mut moved = vec![0.0f64; self.nodes];
        for c in 0..null.cols {
            for (j, &i) in eq.iter().enumerate() {
                moved[i / self.dim] = moved[i / self.dim].max(null.at(j, c).abs());
            }
        }
        let top = moved.iter().copied().fold(0.0, f64::max);
        let mechanism = (0..self.nodes).filter(|&i| top > 0.0 && moved[i] > 1e-6 * top).collect();
        Free { rigid, mechanism: Some(mechanism) }
    }

    /// **Whether a requirement on the output is out of reach** (#510 P7) at `x`, stating `target`,
    /// of the columns `free` says drive it (`analysis::reach`).  A node's coordinate is moved by
    /// the structure's extent, an area by itself.
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

/// The constants before the structure: `[dim, nodes, bars]`, then the output stated, `J₀`, or
/// where an unknown sets it, its `(m, c)`.
pub const HEAD: usize = 3;

impl Structure {
    /// A row's constants: the head, every bar's `(a, b, E)`, then every degree of freedom's
    /// `(held, load, weight)`.  `stated` is `[J₀]`, or `[m, c]` for an output an unknown sets.
    pub fn consts(&self, stated: &[f64]) -> Vec<f64> {
        let mut k = Vec::with_capacity(self.widths(stated.len() == 2).1);
        k.extend([self.dim as f64, self.nodes as f64, self.bars.len() as f64]);
        k.extend(stated);
        for b in &self.bars {
            k.extend([b.a as f64, b.b as f64, b.e]);
        }
        for i in 0..self.fixed.len() {
            k.extend([self.fixed[i] as u8 as f64, self.load[i], self.weight[i]]);
        }
        k
    }

    /// What `consts` wrote, `free` where an unknown sets the output (the row has its column, as
    /// `integral::length_widths` reads a length's): the structure, and what it states.
    pub fn read(k: &[f64], free: bool) -> (Structure, &[f64]) {
        let (dim, nodes, n_bars) = (k[0] as usize, k[1] as usize, k[2] as usize);
        let (stated, rest) = k[HEAD..].split_at(if free { 2 } else { 1 });
        let (bars, per) = rest.split_at(3 * n_bars);
        let per = per.as_chunks::<3>().0;
        let s = Structure {
            dim,
            nodes,
            bars: bars.as_chunks::<3>().0.iter()
                .map(|c| Bar { a: c[0] as usize, b: c[1] as usize, e: c[2] })
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
        (self.columns() + free as usize, HEAD + stated + 3 * self.bars.len() + 3 * self.fixed.len())
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
    let (dim, nodes, bars) = (k[0] as usize, k[1] as usize, k[2] as usize);
    n_par > dim * nodes + bars
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
