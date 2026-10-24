//! **A filled body's mesh moved with the drawing** (#567): the nodes of a solid that is no prism
//! — its exact boundary refined and filled (`bodies::Mesh::filled`) — as functions of the
//! drawing's own numbers, so a row over it reads them as its columns, as a region's do
//! (`Moved`).
//!
//! The mesh's topology is the compile's.  What moves its boundary is the body's own surfaces:
//! every face of a body made of extrusions and turns, cut and united, is a piece of a **sheet**
//! — a cap of a prism (a plane at an ordinate of its face's plane), a side of a prism (a profile's
//! line or circle swept along the plane's normal: a plane or a cylinder), or a face of a turn (a
//! profile's line or circle turned about an axis: a plane, a cone, a cylinder, a sphere or a
//! torus) — and a boolean trims a sheet without changing it.  Each sheet is a closed form of the
//! points, radii and planes that define it (`Sheet::level`, its signed distance and that
//! distance's gradient in space), so its rate in them is carried by `Dual`s.
//!
//! Each node of the boundary is **tagged** with the sheets it lies on when the mesh is made: one
//! (a face's), two (an edge's) or three (a vertex's), meeting at the node (`hulls::hull_of`).  At
//! columns `v` it stands at `x₀ + π(v, s) − π(v₀, x₀)`, `π` its **foot**: from a start `s`, the
//! point on its sheets reached by Gauss–Newton steps of least length (`foot`) — on one sheet its
//! nearest point, on two the edge, on three the vertex.  So a node stands at the compile's pose
//! exactly where it was meshed (`π(v₀, x₀)` is the same arithmetic twice), and stands off the
//! moved boundary only by what it stood off the boundary it was meshed on.  A node that reaches no
//! foot (a sheet gone degenerate, sheets no longer meeting) has no place, and the row reads NaN: a
//! fold, met by meshing again (`System::stop_at`'s `Stop::Folded`).
//!
//! The nodes are placed **a stage at a time** (`Stage`): the vertices at their feet from where
//! they were meshed; the edges' nodes moved harmonically along their curves from the vertices, the
//! faces' over their surfaces from the edges — each then at its foot from there — and the nodes
//! inside harmonically through the volume from the whole boundary, `L_II δ_I = −L_IB δ_B` over the
//! linear Laplacian of the compile's mesh (each TET10 cut into its eight straight tetrahedra).  The
//! gradient takes the stages back in reverse, each harmonic map's on its factor
//! (`−L_IBᵀ L_II⁻¹ g_I`), each foot's in its columns and its start by `Dual`s.  A measure reads the
//! boundary alone, so its inner nodes stay where they were meshed.

use crate::cholesky::{assembled, Cholesky, Ordering};
use crate::envelope::held::{cross as vcross, dot as vdot, vadd, vscale, vsub};
use crate::kernels::{Dual, Num};
use crate::space::{cross, dot, norm, sub};

/// The most Gauss–Newton steps a foot takes.
const STEPS: usize = 40;

/// A foot is reached where a step is no longer than this of the hull's extent.
const STOP: f64 = 1e-13;

/// A foot is on its sheets where each reads within this of the hull's extent.
const ON: f64 = 1e-9;

/// **A plane the sheets are drawn in**: held where it stands (its origin and its two unit axes,
/// nine numbers), or read off nine columns from the one given — its origin, then its two axes'
/// directions, as `Sketch::basis` reads a plane: `u` their first made unit, `v` the second with
/// its part along `u` taken off.
#[derive(Clone, Debug, PartialEq)]
pub enum Frame {
    Held([f64; 9]),
    Read(usize),
}

/// **A profile's side, in its plane**: a line between two points, or a circle (an arc's, too)
/// about its centre — each point by the column of its x (its y the next), a radius by its column.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Profile {
    Line { a: usize, b: usize },
    Circle { c: usize, r: usize },
}

/// **The axis a profile is turned about**: held (a point of it and its direction), a line of the
/// profile's plane (its two points, read in that plane as a revolution reads them), or read off
/// columns (three from `a` its point, three from `d` its direction).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Axis {
    Held { a: [f64; 3], d: [f64; 3] },
    Line { a: usize, b: usize },
    Read { a: usize, d: usize },
}

/// **A sheet a face of the body is a piece of**: a prism's cap at ordinate `h` along its plane's
/// normal, a prism's side (a profile's line or circle swept along the normal), or a turn's face
/// (a profile's line or circle turned about an axis).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sheet {
    Cap { frame: usize, h: f64 },
    Extruded { frame: usize, curve: Profile },
    Turned { frame: usize, axis: Axis, curve: Profile },
}

/// **A filled body's nodes as functions of the drawing's columns**: how many columns, the planes
/// and the sheets, per node the sheets it lies on (none: inside, following the boundary), every
/// node's place at the compile (`x0`, three each), each tagged node's foot there (`feet0`), and
/// the hull's extent.
#[derive(Clone, Debug, PartialEq)]
pub struct Hull {
    pub columns: usize,
    pub frames: Vec<Frame>,
    pub sheets: Vec<Sheet>,
    pub nodes: Vec<Vec<usize>>,
    pub x0: Vec<f64>,
    pub feet0: Vec<f64>,
    pub extent: f64,
}

/// **Which stage a node is placed in**: a vertex at its foot; an edge's node, a face's and one
/// inside each moved harmonically from the stages before (an edge's and a face's then at its
/// foot from there).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Stage {
    Vertex,
    Edge,
    Face,
    Inner,
}

/// The stages moved harmonically, in the order they are placed, each by the Laplacian `weights`
/// reads for it; the vertices before them are placed at their feet alone.
const HARMONIC: [Stage; 3] = [Stage::Edge, Stage::Face, Stage::Inner];

/// **A stage moved harmonically** (`harmonic`): its free nodes numbered among themselves, its
/// Laplacian's factor over them, and each pair of a free node and a held one with its weight —
/// all its rate needs taken back (`adjoint`).
struct Harmonic {
    at: Vec<Option<usize>>,
    factor: Cholesky,
    held: Vec<(usize, usize, f64)>,
}

/// The places worked out at some columns: every node's, where each tagged node's foot started,
/// each node's stage, and each harmonic stage's map (`HARMONIC`'s order; none where nothing was
/// free or it was not asked for).
pub struct Placed {
    pub x: Vec<f64>,
    start: Vec<f64>,
    stage: Vec<Stage>,
    maps: [Option<Harmonic>; 3],
}

/// **The nodes `free` moved harmonically from the rest** over the Laplacian `weights` hands on —
/// `L_FF δ_F = −L_FH δ_H`, each axis its solve on one factor — their places in `x` (three a
/// node), every other node's move read off `x` against `x0`.  A Laplacian of a curve or a surface
/// (`regular`) may hold a piece no held node reaches (a closed circle's edge, a face of none): its
/// diagonal is taken a part in a billion heavier, so that piece stays where it was meshed.  The
/// map, none where nothing is free; `None` where the Laplacian does not factor.
fn harmonic(free: &[bool], weights: &dyn Fn(&mut dyn FnMut(usize, usize, f64)), x0: &[f64],
            x: &mut [f64], regular: bool) -> Option<Option<Harmonic>>
{
    let mut m = 0;
    let at: Vec<Option<usize>> =
        free.iter().map(|&f| f.then(|| { m += 1; m - 1 })).collect();
    if m == 0 {
        return Some(None);
    }
    let mut t: Vec<(usize, usize, f64)> = Vec::new();
    let mut held: Vec<(usize, usize, f64)> = Vec::new();
    let mut rhs = [vec![0.0; m], vec![0.0; m], vec![0.0; m]];
    let mut diagonal = vec![0.0; m];
    weights(&mut |a, b, w| {
        for (p, q) in [(a, b), (b, a)] {
            let Some(ip) = at[p] else { continue };
            t.push((ip, ip, w));
            diagonal[ip] += w;
            match at[q] {
                Some(iq) => t.push((ip, iq, -w)),
                None => {
                    held.push((ip, q, w));
                    for c in 0..3 {
                        rhs[c][ip] += w * (x[3 * q + c] - x0[3 * q + c]);
                    }
                }
            }
        }
    });
    if regular {
        let most = diagonal.iter().fold(0.0f64, |a, d| a.max(d.abs()));
        for i in 0..m {
            t.push((i, i, 1e-9 * most));
        }
    }
    let (l, mut factor) = assembled(m, &t, Ordering::Amd);
    factor.factor(&l).ok()?;
    for (c, r) in rhs.iter_mut().enumerate() {
        factor.solve(r);
        for (i, k) in at.iter().enumerate() {
            if let Some(k) = *k {
                x[3 * i + c] = moved(x0[3 * i + c], r[k]);
            }
        }
    }
    Some(Some(Harmonic { at, factor, held }))
}

/// `x` moved by `d`: itself, to the bit, where `d` is nothing.
fn moved(x: f64, d: f64) -> f64 {
    if d == 0.0 { x } else { x + d }
}

/// **A harmonic map's rate taken back**: the gradient `g` (three a node) at its free nodes
/// carried onto the nodes they were moved from, `λ = L_FF⁻¹ g_F` and each held node `h` gaining
/// `Σ_f w_fh λ_f` — the free nodes' gradient then spent.
fn adjoint(map: &Harmonic, g: &mut [f64]) {
    let at = &map.at;
    let m = at.iter().flatten().count();
    let mut lam = [vec![0.0; m], vec![0.0; m], vec![0.0; m]];
    for (c, l) in lam.iter_mut().enumerate() {
        for (i, k) in at.iter().enumerate() {
            if let Some(k) = *k {
                l[k] = g[3 * i + c];
            }
        }
        map.factor.solve(l);
    }
    for &(ip, q, w) in &map.held {
        for c in 0..3 {
            g[3 * q + c] += w * lam[c][ip];
        }
    }
    for (i, k) in at.iter().enumerate() {
        if k.is_some() {
            g[3 * i..3 * i + 3].fill(0.0);
        }
    }
}

/// **The boundary of a mesh of quadratic tetrahedra**, unturned: every face on one element, as
/// `[a, b, c, ab, bc, ca]` (`bodies::boundary`'s, which turns each out of the material: a
/// Laplacian reads no turn, and its weights summed in this order are the ones the row is held
/// to).
fn surface(tets: &[[usize; 10]]) -> Vec<[usize; 6]> {
    use crate::delaunay::tetra::EDGES;
    let mut faces: std::collections::BTreeMap<[usize; 3], (usize, usize, u32)> = Default::default();
    for (t, e) in tets.iter().enumerate() {
        for far in 0..4 {
            let mut f = [0, 1, 2, 3].map(|c| e[c]);
            f.swap(far, 3);
            let mut f = [f[0], f[1], f[2]];
            f.sort_unstable();
            faces.entry(f).or_insert((t, far, 0)).2 += 1;
        }
    }
    let middle = |e: &[usize; 10], i: usize, j: usize| {
        let m = EDGES.iter().position(|&q| q == (i, j) || q == (j, i)).unwrap();
        e[4 + m]
    };
    faces.into_values().filter(|f| f.2 == 1).map(|(t, far, _)| {
        let e = &tets[t];
        let c: Vec<usize> = (0..4).filter(|&k| k != far).collect();
        [e[c[0]], e[c[1]], e[c[2]], middle(e, c[0], c[1]), middle(e, c[1], c[2]),
         middle(e, c[2], c[0])]
    }).collect()
}

type V<T> = [T; 3];

/// A unit vector along `a`: NaN where it has no length, a degenerate sheet, which `foot` refuses.
fn unit<T: Num>(a: V<T>) -> V<T> {
    vscale(a, T::cst(1.0) / vdot(a, a).sqrt())
}

/// Node `i`'s three numbers of `x`.
fn at(x: &[f64], i: usize) -> [f64; 3] {
    [x[3 * i], x[3 * i + 1], x[3 * i + 2]]
}

impl Profile {
    /// The columns it reads.
    fn columns(&self) -> Vec<usize> {
        match *self {
            Profile::Line { a, b } => vec![a, a + 1, b, b + 1],
            Profile::Circle { c, r } => vec![c, c + 1, r],
        }
    }
}

impl Axis {
    /// The columns it reads.
    fn columns(&self) -> Vec<usize> {
        match *self {
            Axis::Held { .. } => Vec::new(),
            Axis::Line { a, b } => vec![a, a + 1, b, b + 1],
            Axis::Read { a, d } => (a..a + 3).chain(d..d + 3).collect(),
        }
    }
}

impl Sheet {
    fn frame(&self) -> usize {
        match *self {
            Sheet::Cap { frame, .. } | Sheet::Extruded { frame, .. } | Sheet::Turned { frame, .. } => {
                frame
            }
        }
    }
}

impl Hull {
    /// The columns sheet `s` reads, its plane's among them, in order and once each.
    fn sheet_columns(&self, s: usize) -> Vec<usize> {
        let sheet = &self.sheets[s];
        let mut out = match self.frames[sheet.frame()] {
            Frame::Read(c) => (c..c + 9).collect(),
            Frame::Held(_) => Vec::new(),
        };
        match sheet {
            Sheet::Cap { .. } => {}
            Sheet::Extruded { curve, .. } => out.extend(curve.columns()),
            Sheet::Turned { axis, curve, .. } => {
                out.extend(curve.columns());
                out.extend(axis.columns());
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }

    /// Plane `f`'s origin, its two axes and its normal, over the columns `col` reads.
    fn basis<T: Num>(&self, f: usize, col: &dyn Fn(usize) -> T) -> [V<T>; 4] {
        let (o, u, v) = match self.frames[f] {
            Frame::Held(k) => {
                let c = |i: usize| [T::cst(k[i]), T::cst(k[i + 1]), T::cst(k[i + 2])];
                (c(0), c(3), c(6))
            }
            Frame::Read(c) => {
                let at = |i: usize| [col(c + i), col(c + i + 1), col(c + i + 2)];
                let u = unit(at(3));
                let dv = at(6);
                (at(0), u, unit(vsub(dv, vscale(u, vdot(dv, u)))))
            }
        };
        let n = vcross(u, v);
        [o, u, v, n]
    }

    /// **Sheet `s`'s level at `p`**: its signed distance (on a turn's face, the distance in the
    /// meridian half-plane, the same thing) and that distance's gradient in space, over the
    /// columns `col` reads — NaN where the sheet is degenerate (a line of no length, a node on a
    /// turn's axis off a face square to it).
    fn level<T: Num>(&self, s: usize, col: &dyn Fn(usize) -> T, p: V<T>) -> (T, V<T>) {
        let [o, u, v, n] = self.basis(self.sheets[s].frame(), col);
        let lift = |c: usize| vadd(o, vadd(vscale(u, col(c)), vscale(v, col(c + 1))));
        let k = T::cst;
        match self.sheets[s] {
            Sheet::Cap { h, .. } => (vdot(vsub(p, o), n) - k(h), n),
            Sheet::Extruded { curve: Profile::Line { a, b }, .. } => {
                let pa = lift(a);
                let m = unit(vcross(vsub(lift(b), pa), n));
                (vdot(vsub(p, pa), m), m)
            }
            Sheet::Extruded { curve: Profile::Circle { c, r }, .. } => {
                let d = vsub(p, lift(c));
                let w = vsub(d, vscale(n, vdot(d, n)));
                let rho = vdot(w, w).sqrt();
                (rho - col(r), vscale(w, k(1.0) / rho))
            }
            Sheet::Turned { axis, curve, .. } => {
                let (a, d) = match axis {
                    Axis::Held { a, d } => (a.map(k), d.map(k)),
                    Axis::Line { a, b } => {
                        let pa = lift(a);
                        (pa, unit(vsub(lift(b), pa)))
                    }
                    Axis::Read { a, d } => {
                        ([col(a), col(a + 1), col(a + 2)], unit([col(d), col(d + 1), col(d + 2)]))
                    }
                };
                // a place as the meridian half-plane reads it: along the axis, and out from it
                let meridian = |q: V<T>| {
                    let r = vsub(q, a);
                    let t = vdot(r, d);
                    let w = vsub(r, vscale(d, t));
                    (t, w, vdot(w, w).sqrt())
                };
                let (tp, w, rho) = meridian(p);
                // out from the axis; nothing on it, where only a face square to it is read
                let out = if rho.val() > 0.0 { vscale(w, k(1.0) / rho) } else { [k(0.0); 3] };
                let (g, phi) = match curve {
                    Profile::Line { a: pa, b: pb } => {
                        let (ta, _, ra) = meridian(lift(pa));
                        let (tb, _, rb) = meridian(lift(pb));
                        let (et, er) = (tb - ta, rb - ra);
                        let len = (et * et + er * er).sqrt();
                        let (mt, mr) = (k(0.0) - er / len, et / len);
                        ((mt, mr), (tp - ta) * mt + (rho - ra) * mr)
                    }
                    Profile::Circle { c, r } => {
                        let (tc, _, rc) = meridian(lift(c));
                        let (dt, dr) = (tp - tc, rho - rc);
                        let len = (dt * dt + dr * dr).sqrt();
                        ((dt / len, dr / len), len - col(r))
                    }
                };
                if rho.val() <= 0.0 && g.1.val() != 0.0 {
                    let nan = k(f64::NAN);
                    return (nan, [nan; 3]);
                }
                (phi, vadd(vscale(d, g.0), vscale(out, g.1)))
            }
        }
    }

    /// **A node's foot on `sheets`** (one, two or three), from `start`, over the columns `col`
    /// reads: Gauss–Newton steps of least length, `p ← p − Jᵀ(JJᵀ)⁻¹φ(p)` — on one sheet its
    /// nearest point (each sheet's level is its distance), on two the edge, on three the vertex —
    /// until a step is no longer than `STOP` of the extent; `None` where none is reached, or one
    /// reached is not on every sheet.
    fn foot<T: Num>(&self, sheets: &[usize], col: &dyn Fn(usize) -> T, start: V<T>)
        -> Option<V<T>>
    {
        let mut p = start;
        let reached = |p: &V<T>| sheets.iter().all(|&s| {
            let (phi, _) = self.level(s, col, *p);
            phi.val().abs() <= ON * self.extent
        });
        for _ in 0..STEPS {
            let levels: Vec<(T, V<T>)> = sheets.iter().map(|&s| self.level(s, col, p)).collect();
            let mu = solve_gram(&levels)?;
            let mut step = [T::cst(0.0); 3];
            for (m, (_, g)) in mu.iter().zip(&levels) {
                step = vadd(step, vscale(*g, *m));
            }
            p = vsub(p, step);
            let size = step.iter().map(|s| s.val().abs()).fold(0.0, f64::max);
            if !size.is_finite() {
                return None;
            }
            if size <= STOP * self.extent {
                return reached(&p).then_some(p);
            }
        }
        None
    }

    /// The columns node `i`'s sheets read, in order and once each, each sheet's read from
    /// `sheets` (`sheet_columns`).
    fn node_columns(&self, i: usize, sheets: &[Vec<usize>]) -> Vec<usize> {
        let mut out: Vec<usize> = self.nodes[i].iter().flat_map(|&s| sheets[s].iter().copied())
            .collect();
        out.sort_unstable();
        out.dedup();
        out
    }

    /// Sheet `s`'s level at `p` where the columns stand at `v`: its signed distance and that
    /// distance's gradient in space.
    pub fn level_at(&self, s: usize, v: &[f64], p: [f64; 3]) -> (f64, [f64; 3]) {
        let col = |c: usize| <Dual<0> as Num>::cst(v[c]);
        let (phi, g) = self.level(s, &col, p.map(Dual::cst));
        (phi.v, g.map(|q| q.v))
    }

    /// Node `i`'s foot at columns `v` from `start`, its value alone.
    pub fn foot_at(&self, i: usize, v: &[f64], start: [f64; 3]) -> Option<[f64; 3]> {
        let col = |c: usize| <Dual<0> as Num>::cst(v[c]);
        self.foot(&self.nodes[i], &col, start.map(Dual::cst)).map(|p| p.map(|q| q.v))
    }

    /// Node `i`'s foot at columns `v` from `start`, with its rate in the columns `local` (its
    /// own, the first of the `N` numbers) and in its start (the three after them).
    fn foot_rate<const N: usize>(&self, i: usize, v: &[f64], local: &[usize], start: [f64; 3])
        -> Option<[Dual<N>; 3]>
    {
        let col = |c: usize| match local.binary_search(&c) {
            Ok(k) => Dual::<N>::var(v[c], k),
            Err(_) => Dual::<N>::cst(v[c]),
        };
        let k = local.len();
        let start = [0, 1, 2].map(|c| Dual::<N>::var(start[c], k + c));
        self.foot(&self.nodes[i], &col, start)
    }

    /// **Each node's stage**: a vertex's (three sheets), an edge's (two), a face's (one, or none
    /// where it is a node of a facet but on no sheet), or inside — as the nodes are placed, in
    /// that order, each stage held by the ones before it.
    fn stages(&self, facets: &[[usize; 6]]) -> Vec<Stage> {
        let mut out: Vec<Stage> = self.nodes.iter().map(|s| match s.len() {
            0 => Stage::Inner,
            1 => Stage::Face,
            2 => Stage::Edge,
            _ => Stage::Vertex,
        }).collect();
        for f in facets {
            for &n in f {
                if out[n] == Stage::Inner {
                    out[n] = Stage::Face;
                }
            }
        }
        out
    }

    /// **Every node's place at columns `v`** over the elements `tets`, a stage at a time: each
    /// vertex at its foot from where it was meshed; each node of an edge, of a face, then inside
    /// moved harmonically from the stages before it (`Placed::harmonic`) — along its curve, over
    /// its surface, through the volume — and a node of an edge or a face then at its foot from
    /// there, each tagged node standing at `x₀ + π(v, s) − π(v₀, x₀)`.  `None` where a node has
    /// no foot, or a Laplacian does not factor.  A measure asks for the boundary alone
    /// (`inside` false), its inner nodes left where they were meshed.
    pub fn places(&self, v: &[f64], tets: &[[usize; 10]], inside: bool) -> Option<Placed> {
        let n = self.nodes.len();
        let facets = surface(tets);
        let stage = self.stages(&facets);
        let mut x = self.x0.clone();
        let mut start = self.x0.clone();
        let feet = |s: Stage, start: &[f64], x: &mut [f64]| -> Option<()> {
            for i in (0..n).filter(|&i| stage[i] == s && !self.nodes[i].is_empty()) {
                let f = self.foot_at(i, v, at(start, i))?;
                for c in 0..3 {
                    x[3 * i + c] = moved(self.x0[3 * i + c], f[c] - self.feet0[3 * i + c]);
                }
            }
            Some(())
        };
        feet(Stage::Vertex, &start, &mut x)?;
        let mut maps: [Option<Harmonic>; 3] = [None, None, None];
        for (k, s) in HARMONIC.into_iter().enumerate() {
            if s == Stage::Inner && !inside {
                break;
            }
            // moved harmonically from the stages before
            let free: Vec<bool> = stage.iter().map(|&t| t == s).collect();
            let weights = |put: &mut dyn FnMut(usize, usize, f64)| {
                self.weights(s, &facets, &stage, tets, put)
            };
            maps[k] = harmonic(&free, &weights, &self.x0, &mut x, s != Stage::Inner)?;
            for i in (0..n).filter(|&i| free[i]) {
                start[3 * i..3 * i + 3].copy_from_slice(&x[3 * i..3 * i + 3]);
            }
            if s != Stage::Inner {
                feet(s, &start, &mut x)?;
            }
        }
        Some(Placed { x, start, stage, maps })
    }

    /// The Laplacian stage `s` is moved by: a curve's for an edge's nodes, the surface's for a
    /// face's, the volume's for those inside.
    fn weights(&self, s: Stage, facets: &[[usize; 6]], stage: &[Stage], tets: &[[usize; 10]],
               put: &mut dyn FnMut(usize, usize, f64)) {
        match s {
            Stage::Edge => self.curve_weights(facets, stage, put),
            Stage::Face => self.surface_weights(facets, put),
            _ => self.volume_weights(tets, put),
        }
    }

    /// **The curves' Laplacian**: each side of a facet whose middle is a node of an edge and whose
    /// ends lie on that edge's sheets too, its two halves weighed by their lengths' inverses, each
    /// handed to `put` once per facet it is a side of.
    fn curve_weights(&self, facets: &[[usize; 6]], stage: &[Stage],
                     put: &mut dyn FnMut(usize, usize, f64)) {
        let p = |i: usize| at(&self.x0, i);
        let along = |m: usize, e: usize| {
            matches!(stage[e], Stage::Edge | Stage::Vertex)
                && self.nodes[m].iter().all(|s| self.nodes[e].contains(s))
        };
        for f in facets {
            for (a, m, b) in [(f[0], f[3], f[1]), (f[1], f[4], f[2]), (f[2], f[5], f[0])] {
                if stage[m] != Stage::Edge || !along(m, a) || !along(m, b) {
                    continue;
                }
                for e in [a, b] {
                    let l = norm(sub(p(e), p(m)));
                    if l > 0.0 {
                        put(m, e, 1.0 / l);
                    }
                }
            }
        }
    }

    /// **The surface's Laplacian**: each facet cut into its four straight triangles, each of their
    /// sides weighed by half the cotangent of the angle across it, handed to `put` once a
    /// triangle.
    fn surface_weights(&self, facets: &[[usize; 6]], put: &mut dyn FnMut(usize, usize, f64)) {
        let p = |i: usize| at(&self.x0, i);
        for t in facets {
            for [a, b, c] in [[t[0], t[3], t[5]], [t[3], t[1], t[4]], [t[5], t[4], t[2]],
                              [t[3], t[4], t[5]]] {
                for (i, j, k) in [(a, b, c), (b, c, a), (c, a, b)] {
                    // the side i–j, its angle at k
                    let (u, w) = (sub(p(i), p(k)), sub(p(j), p(k)));
                    let s = norm(cross(u, w));
                    if s > 0.0 {
                        put(i, j, 0.5 * dot(u, w) / s);
                    }
                }
            }
        }
    }

    /// **The volume's Laplacian**, the linear one of the compile's mesh: each TET10 of `tets` cut
    /// into its eight straight tetrahedra (four at its corners, four about its inner octahedron's
    /// diagonal 01–23), each tetrahedron's `−V ∇λ_a·∇λ_b` handed to `put` for each of its edges.
    /// A sum of such pairs is the stiffness of the linear Poisson element, positive definite once
    /// the boundary is held.
    fn volume_weights(&self, tets: &[[usize; 10]], put: &mut dyn FnMut(usize, usize, f64)) {
        let p = |i: usize| at(&self.x0, i);
        for e in tets {
            // the middles of 0–1, 1–2, 2–0, 0–3, 1–3, 2–3 (`tetra::EDGES`)
            let [c0, c1, c2, c3, m01, m12, m20, m03, m13, m23] = *e;
            for q in [[c0, m01, m20, m03], [m01, c1, m12, m13], [m20, m12, c2, m23],
                      [m03, m13, m23, c3], [m01, m23, m20, m12], [m01, m23, m12, m13],
                      [m01, m23, m13, m03], [m01, m23, m03, m20]] {
                let x = q.map(p);
                let e1 = sub(x[1], x[0]);
                let e2 = sub(x[2], x[0]);
                let e3 = sub(x[3], x[0]);
                let det = dot(e1, cross(e2, e3));
                if !(det.abs() > 0.0) {
                    continue;
                }
                let g1 = cross(e2, e3).map(|c| c / det);
                let g2 = cross(e3, e1).map(|c| c / det);
                let g3 = cross(e1, e2).map(|c| c / det);
                let g0 = [0, 1, 2].map(|c| -(g1[c] + g2[c] + g3[c]));
                let g = [g0, g1, g2, g3];
                let vol = det.abs() / 6.0;
                for a in 0..4 {
                    for b in a + 1..4 {
                        put(q[a], q[b], -vol * dot(g[a], g[b]));
                    }
                }
            }
        }
    }

    /// **A gradient in the nodes' places carried to the columns**, added into `row`, the stages
    /// taken back in the order they were placed in reverse: the inner nodes' onto the boundary's
    /// through the harmonic map (`−L_IBᵀ L_II⁻¹ g_I`, one solve per axis on the factor in hand);
    /// a face's then an edge's nodes each onto the columns its foot reads and onto where it
    /// started (`Dual`s through the Gauss–Newton steps), that start's onto the stage before
    /// through its harmonic map; the vertices' onto their columns.  `None` where a foot is not
    /// reached, or reads more columns than a rate is carried in.
    pub fn chain(&self, v: &[f64], placed: &Placed, g: &[f64], row: &mut [f64]) -> Option<()> {
        let n = self.nodes.len();
        let mut g = g[..3 * n].to_vec();
        let sheets: Vec<Vec<usize>> =
            (0..self.sheets.len()).map(|s| self.sheet_columns(s)).collect();
        // a stage's tagged nodes onto their columns and where they started
        let carry = |s: Stage, g: &mut [f64], row: &mut [f64]| -> Option<()> {
            for i in (0..n).filter(|&i| placed.stage[i] == s && !self.nodes[i].is_empty()) {
                let gi = at(g, i);
                if gi.iter().all(|&q| q == 0.0) {
                    continue;
                }
                let local = self.node_columns(i, &sheets);
                let back = self.carried(i, v, &local, at(&placed.start, i), gi, row)?;
                g[3 * i..3 * i + 3].copy_from_slice(&back);
            }
            Some(())
        };
        for (k, s) in HARMONIC.into_iter().enumerate().rev() {
            if s != Stage::Inner {
                carry(s, &mut g, row)?;
            }
            // the stage's starts onto the stage before, through its harmonic map
            if let Some(map) = &placed.maps[k] {
                adjoint(map, &mut g);
            }
        }
        carry(Stage::Vertex, &mut g, row)
    }

    /// Node `i`'s rate carried, from `start`: `gᵀ ∂π/∂v` added into `row` at the columns `local`
    /// its foot reads, and `gᵀ ∂π/∂s` returned, its rate in where it started.
    fn carried(&self, i: usize, v: &[f64], local: &[usize], start: [f64; 3], g: [f64; 3],
               row: &mut [f64]) -> Option<[f64; 3]>
    {
        let k = local.len();
        macro_rules! carried {
            ($n:literal) => {{
                let f = self.foot_rate::<$n>(i, v, local, start)?;
                for (j, &c) in local.iter().enumerate() {
                    row[c] += (0..3).map(|a| g[a] * f[a].g[j]).sum::<f64>();
                }
                [0, 1, 2].map(|j| (0..3).map(|a| g[a] * f[a].g[k + j]).sum::<f64>())
            }};
        }
        Some(match k + 3 {
            0..=8 => carried!(8),
            9..=16 => carried!(16),
            17..=32 => carried!(32),
            33..=64 => carried!(64),
            _ => return None,
        })
    }

    /// Its constants: `[columns, frames, sheets, nodes, extent]`, each plane's ten (its kind,
    /// then nine numbers), each sheet's twelve, each node's four (how many sheets, then each),
    /// then the places and the feet, three a node each.
    pub fn consts(&self) -> Vec<f64> {
        let mut k = vec![self.columns as f64, self.frames.len() as f64, self.sheets.len() as f64,
                         self.nodes.len() as f64, self.extent];
        for f in &self.frames {
            match f {
                Frame::Held(b) => {
                    k.push(0.0);
                    k.extend(b);
                }
                Frame::Read(c) => {
                    k.extend([1.0, *c as f64]);
                    k.extend([0.0; 8]);
                }
            }
        }
        let curve = |c: &Profile| match *c {
            Profile::Line { a, b } => [0.0, a as f64, b as f64],
            Profile::Circle { c, r } => [1.0, c as f64, r as f64],
        };
        for s in &self.sheets {
            let mut w = [0.0; SHEET];
            match s {
                Sheet::Cap { frame, h } => {
                    w[..3].copy_from_slice(&[0.0, *frame as f64, *h]);
                }
                Sheet::Extruded { frame, curve: c } => {
                    w[..2].copy_from_slice(&[1.0, *frame as f64]);
                    w[2..5].copy_from_slice(&curve(c));
                }
                Sheet::Turned { frame, axis, curve: c } => {
                    w[..2].copy_from_slice(&[2.0, *frame as f64]);
                    w[2..5].copy_from_slice(&curve(c));
                    let (kind, rest): (f64, [f64; 6]) = match *axis {
                        Axis::Held { a, d } => (0.0, [a[0], a[1], a[2], d[0], d[1], d[2]]),
                        Axis::Line { a, b } => (1.0, [a as f64, b as f64, 0.0, 0.0, 0.0, 0.0]),
                        Axis::Read { a, d } => (2.0, [a as f64, d as f64, 0.0, 0.0, 0.0, 0.0]),
                    };
                    w[5] = kind;
                    w[6..].copy_from_slice(&rest);
                }
            }
            k.extend(w);
        }
        for s in &self.nodes {
            let mut w = [0.0; 4];
            w[0] = s.len() as f64;
            for (j, &t) in s.iter().enumerate() {
                w[1 + j] = t as f64;
            }
            k.extend(w);
        }
        k.extend(&self.x0);
        k.extend(&self.feet0);
        k
    }

    /// How many constants it is.
    pub fn width(&self) -> usize {
        5 + 10 * self.frames.len() + SHEET * self.sheets.len() + 10 * self.nodes.len()
    }

    /// What `consts` wrote, from `k`, and how many it read — `None` for constants that are not
    /// whole, or a column, a plane, a sheet or a count out of range.
    pub fn read(k: &[f64]) -> Option<(Hull, usize)> {
        let count = |v: f64| (v >= 0.0 && v.fract() == 0.0 && v < 1e9).then_some(v as usize);
        let head = k.get(..5)?;
        let (columns, n_frames, n_sheets, n_nodes) =
            (count(head[0])?, count(head[1])?, count(head[2])?, count(head[3])?);
        let extent = head[4];
        if !(extent.is_finite() && extent > 0.0) {
            return None;
        }
        let mut at: usize = 5;
        let mut take = |n: usize| -> Option<&[f64]> {
            let out = k.get(at..at.checked_add(n)?)?;
            at += n;
            Some(out)
        };
        // a column read with the `n` after it
        let col = |v: f64, n: usize| count(v).filter(|&c| c + n <= columns);
        let frames: Vec<Frame> = take(10 * n_frames)?.as_chunks::<10>().0.iter().map(|f| {
            Some(match f[0] {
                z if z == 0.0 => Frame::Held(f[1..].try_into().ok()?),
                z if z == 1.0 => Frame::Read(col(f[1], 9)?),
                _ => return None,
            })
        }).collect::<Option<_>>()?;
        let frame = |v: f64| count(v).filter(|&f| f < n_frames);
        let curve = |c: &[f64]| Some(match c[0] {
            z if z == 0.0 => Profile::Line { a: col(c[1], 2)?, b: col(c[2], 2)? },
            z if z == 1.0 => Profile::Circle { c: col(c[1], 2)?, r: col(c[2], 1)? },
            _ => return None,
        });
        let sheets: Vec<Sheet> = take(SHEET * n_sheets)?.as_chunks::<SHEET>().0.iter().map(|s| {
            Some(match s[0] {
                z if z == 0.0 => Sheet::Cap { frame: frame(s[1])?, h: s[2] },
                z if z == 1.0 => Sheet::Extruded { frame: frame(s[1])?, curve: curve(&s[2..5])? },
                z if z == 2.0 => {
                    let r = &s[6..];
                    let axis = match s[5] {
                        z if z == 0.0 => Axis::Held { a: [r[0], r[1], r[2]], d: [r[3], r[4], r[5]] },
                        z if z == 1.0 => Axis::Line { a: col(r[0], 2)?, b: col(r[1], 2)? },
                        z if z == 2.0 => Axis::Read { a: col(r[0], 3)?, d: col(r[1], 3)? },
                        _ => return None,
                    };
                    Sheet::Turned { frame: frame(s[1])?, axis, curve: curve(&s[2..5])? }
                }
                _ => return None,
            })
        }).collect::<Option<_>>()?;
        let nodes: Vec<Vec<usize>> = take(4 * n_nodes)?.as_chunks::<4>().0.iter().map(|w| {
            let n = count(w[0]).filter(|&n| n <= 3)?;
            w[1..1 + n].iter().map(|&s| count(s).filter(|&s| s < n_sheets)).collect()
        }).collect::<Option<_>>()?;
        let x0 = take(3 * n_nodes)?.to_vec();
        let feet0 = take(3 * n_nodes)?.to_vec();
        Some((Hull { columns, frames, sheets, nodes, x0, feet0, extent }, at))
    }
}

/// A sheet's constants: its kind, its plane, its profile's three, its axis's kind and six.
const SHEET: usize = 12;

/// `(JJᵀ)⁻¹ φ` over a node's levels and their gradients (one, two or three): `None` where the
/// sheets are not independent there.
fn solve_gram<T: Num>(levels: &[(T, V<T>)]) -> Option<Vec<T>> {
    let k = levels.len();
    let a = |i: usize, j: usize| vdot(levels[i].1, levels[j].1);
    let f = |i: usize| levels[i].0;
    Some(match k {
        1 => vec![f(0) / a(0, 0)],
        2 => {
            let (a00, a01, a11) = (a(0, 0), a(0, 1), a(1, 1));
            let det = a00 * a11 - a01 * a01;
            if !(det.val().abs() > 0.0) {
                return None;
            }
            vec![(f(0) * a11 - f(1) * a01) / det, (f(1) * a00 - f(0) * a01) / det]
        }
        3 => {
            let m = [[a(0, 0), a(0, 1), a(0, 2)], [a(1, 0), a(1, 1), a(1, 2)],
                     [a(2, 0), a(2, 1), a(2, 2)]];
            let det3 = |m: &[[T; 3]; 3]| {
                m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                    - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                    + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
            };
            let det = det3(&m);
            if !(det.val().abs() > 0.0) {
                return None;
            }
            (0..3).map(|c| {
                let mut mc = m;
                for (r, row) in mc.iter_mut().enumerate() {
                    row[c] = f(r);
                }
                det3(&mc) / det
            }).collect()
        }
        _ => return None,
    })
}
